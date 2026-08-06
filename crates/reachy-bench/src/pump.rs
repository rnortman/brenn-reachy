//! The driver: the one place a sans-I/O sequencer meets a real port.
//!
//! Every library under this one refuses to own a loop. [`ArmSequencer`] and
//! [`DisarmSequencer`] hand out one abstract request at a time and take back one
//! result; the tick takes a measurement and hands back goals. Somebody has to
//! execute those requests, spend the retry budget, watch the clock and decide
//! what a failure means, and that somebody is here.
//!
//! ## What a bus failure becomes
//!
//! A sequencer's vocabulary for how a transaction came out is [`BusResult`], and
//! it names four ways a *machine* can answer badly: silence, a refusal, a
//! read-back that disagrees, and a frame the wire mangled. Those four are what
//! this module translates a [`XactError`] into, and the sequencer decides what
//! they mean in the phase it is in.
//!
//! Everything else a transaction can fail with — the port itself dying, a value
//! whose width disagrees with its register, a reply of a width the request never
//! asked for — is not a verdict about the machine and has no [`BusResult`] to be.
//! Those stop the run as a [`PumpError`], because a sequencer told "no answer"
//! when the truth is "the adapter is unplugged" would report an absent servo.
//!
//! ## The clock
//!
//! A sequencer asks to be called back at a time on the driver's own epoch and
//! never sleeps itself. [`Clock`] is that epoch plus the sleep, behind a trait so
//! a test can run a thirty-second supply budget in no time at all and assert what
//! was waited for.
//!
//! ## The tick loop
//!
//! [`MotionPump`] is the other half: the fixed-rate loop that carries a move,
//! and the same loop is what a timed hold runs on, so the pacing every run
//! reports is the pacing of the loop that actually spaced its periods. Each
//! period reads all nine positions in one grouped request, hands them to
//! [`motion_tick`] with whatever command is pending, and writes back only the
//! joint groups whose goals changed. The read comes before the write within a
//! period, because the tick consumes the measurement it is given.
//!
//! Two rules the loop does not bend. A grouped read is **all-or-nothing**: any
//! servo short of a clean answer makes the whole period's measurement absent, and
//! the tick's read-loss budget — not a retry here — is what decides how long
//! blind motion may continue. And a fault **stops the loop without touching
//! torque**: the servos hold their last goal, which is the only stopped state
//! that does not drop the head, and recovery is the operator's next command.
//!
//! Neither grouped read goes absent quietly. The first miss of a run names the
//! servos that fell short and what each did instead, so a bench session
//! diagnoses one flaky servo from the run it is already having rather than from
//! the next one.

use core::time::Duration;
use std::fmt;
use std::time::Instant;

use thiserror::Error;

use reachy_bus::{
    Bus, BusPort, IdOutcome, MapError, RawValue, ServoMap, SyncReadOutcome, XactError,
};
use reachy_bus::{reg_for, with_retry};
use reachy_motion::{
    ArmConfig, ArmRecord, ArmSummary, BusRequest, BusResult, CommandDisposition, CommandRejection,
    Fault, JointGroup, JointId, JointVector, Mode, MotionCommand, MotionConfig, MotionState, RegId,
    RegValue, SeqAction, SeqError, SeqStep, Sequencer, ServoHealth, TickInputs, TickOutputs,
    motion_tick,
};

/// Actions every phase but the supply gate takes, together and with room to
/// spare.
///
/// Both sequencers walk nine servos over a few dozen registers, so the fixed
/// phases come to a few hundred actions between them. Two orders of magnitude
/// over that still stops a sequencer looping on its own cursor long before a
/// bench session notices.
const FIXED_ACTIONS: usize = 10_000;

/// How many actions the disarm sequence may take before the driver calls it
/// stuck.
///
/// Nothing about disarming is configurable in length: nine position reads, one
/// settle and nine verified releases, whatever the settle is set to. So the
/// bound is the fixed one, with the same two orders of magnitude of room.
pub const DISARM_ACTIONS: usize = FIXED_ACTIONS;

/// Actions one supply-gate poll cycle takes: a read per servo, and the wait
/// that spaces it from the next cycle.
const ACTIONS_PER_POLL: usize = JointId::COUNT + 1;

/// How much longer than its own commanded duration a move may run before the
/// loop calls it stuck, in seconds of control periods.
///
/// The move's length is what it was commanded with, so the budget is that plus
/// this: room for the accepting period, the endpoint period and a loop running
/// behind, and nowhere near enough to sit through a clock that stopped
/// advancing.
const STALL_MARGIN_SECS: u64 = 5;

/// How late a period may begin before the loop calls it an overrun, as a
/// divisor of the control period: half a period.
///
/// Not zero, because the sleep that paces this loop wakes *at or after* its
/// deadline and ordinarily overshoots by tens of microseconds to a millisecond.
/// A zero-tolerance rule would mark every period on any real clock, and a
/// counter that stands at its maximum from the first move says nothing when the
/// stall it exists to catch actually happens. Half a period is where enough of
/// the period's budget is gone to matter; the worst lateness seen is recorded
/// separately whatever its size, so the jitter itself is not lost.
const OVERRUN_DIVISOR: u32 = 2;

/// Bus order covers the nine joints, so a row lookup here cannot miss.
///
/// Spelled out rather than defaulted: what a miss would substitute on the goal
/// path is a commanded angle, and this project invents none.
const BUS_ORDER_IS_NINE_JOINTS: &str = "bus order covers the nine joints";

/// A grouped read that passed `all_ok` filled every row it asked for, each with
/// a value of the register's own width — the bus layer records nothing else as
/// `Ok`.
///
/// A missing row or a width that disagrees is therefore a host-side contract
/// break, not a verdict about the machine, and must not be reported as a period
/// the servos went unread: that would spend the tick's read-loss budget and end
/// in a `ReadLoss` fault blaming the servos for a bug up here.
const ALL_OK_FILLS_EVERY_ROW: &str =
    "an all-ok grouped read fills every row at the register's width";

/// The driver's clock: elapsed time on an epoch it owns, and the sleep.
///
/// Sequencers take `now` as a [`Duration`] since a caller-owned epoch and never
/// read a clock themselves. This is that epoch.
pub trait Clock {
    /// Elapsed time since the epoch.
    fn now(&self) -> Duration;

    /// Block until `until` has elapsed. A time already past returns at once.
    fn sleep_until(&mut self, until: Duration);
}

/// The real clock: a monotonic instant taken when the run began.
#[derive(Clone, Copy, Debug)]
pub struct MonotonicClock {
    epoch: Instant,
}

impl MonotonicClock {
    /// A clock whose epoch is now.
    #[must_use]
    pub fn new() -> Self {
        Self {
            epoch: Instant::now(),
        }
    }
}

impl Default for MonotonicClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for MonotonicClock {
    fn now(&self) -> Duration {
        self.epoch.elapsed()
    }

    fn sleep_until(&mut self, until: Duration) {
        let elapsed = self.now();
        if until > elapsed {
            std::thread::sleep(until - elapsed);
        }
    }
}

/// Why a run stopped.
#[derive(Debug, Error)]
pub enum PumpError {
    /// The sequence itself refused. This is the bring-up output: it names the
    /// phase, the servo, the register and both values.
    #[error("{0}")]
    Sequence(#[from] SeqError),

    /// A transaction failed in a way that is not a verdict about the machine.
    #[error("servo {id}: {source}")]
    Bus {
        /// The servo addressed.
        id: u8,
        /// What went wrong.
        source: XactError,
    },

    /// A register's value could not be put on the wire, or what came back could
    /// not be read as the register's own shape.
    #[error("servo {id} {reg}: {source}")]
    Map {
        /// The servo addressed.
        id: u8,
        /// The register concerned.
        reg: RegId,
        /// What the map refused.
        source: MapError,
    },

    /// The sequence addressed a servo the configured roster does not carry.
    /// A wiring mistake between the configuration and the sequencer, caught
    /// before a frame goes out to whatever holds that ID.
    #[error("the sequence addressed servo {id}, which is not in the configured roster")]
    UnknownServo {
        /// The ID asked for.
        id: u8,
    },

    /// The sequence neither finished nor failed within its action budget — the
    /// supply gate's configured polling, plus the fixed phases.
    #[error("the sequence took more than {budget} actions without finishing")]
    Runaway {
        /// The budget it ran past.
        budget: usize,
    },

    /// The tick stopped commanding. The servos hold their last goal and torque
    /// is untouched; the operator's next command is the recovery.
    #[error("the tick faulted: {0}")]
    Fault(#[from] Fault),

    /// The tick refused the command. Nothing changed, and the machine is in
    /// whatever mode it was already in.
    #[error("the command was refused: {0}")]
    Rejected(#[from] CommandRejection),

    /// The move neither finished nor faulted within its period budget — its own
    /// commanded duration in control periods, plus a fixed margin.
    #[error("the move did not finish within {budget} control periods")]
    Stalled {
        /// The budget it ran past.
        budget: u64,
    },

    /// A rate the loop cannot run at. Configuration refuses both of these, so
    /// reaching this means a caller built a pump by hand.
    #[error("a control rate must be positive: tick {tick_hz} Hz, health poll {health_poll_hz} Hz")]
    Rate {
        /// Control periods per second.
        tick_hz: u32,
        /// Health sweeps per second.
        health_poll_hz: u32,
    },
}

/// How many actions a sequence may take before the driver calls it stuck.
///
/// The supply gate dominates, and both of its numbers are configuration: a
/// thirty-second budget polled every hundred milliseconds is 300 cycles of nine
/// reads plus a wait, and the same budget polled every millisecond is a
/// thousand times that. A fixed bound would cut a gate configured to poll fast
/// short of the wait it was configured for, and blame the sequencer for doing
/// exactly what it was asked — so the bound is derived from what was asked.
#[must_use]
pub fn action_budget(arm: &ArmConfig) -> usize {
    let spacing = arm.voltage_poll_period.as_nanos().max(1);
    let cycles = arm.voltage_budget.as_nanos().div_ceil(spacing);
    usize::try_from(cycles)
        .unwrap_or(usize::MAX)
        .saturating_mul(ACTIONS_PER_POLL)
        .saturating_add(FIXED_ACTIONS)
}

/// Run `seq` to completion against the machine on `bus`, within `budget`
/// actions ([`action_budget`] sizes it from the configuration).
///
/// TODO(reachy-pod-motion-integration): this is the seam a second host arrives
/// at. The libraries below own no loop, so a payload hosting them supplies its
/// own port, clock and loop beside this one.
///
/// Each action is executed and its result handed straight back, so the order of
/// transactions on the wire is the sequencer's order and nothing here reorders,
/// batches or skips one. `phase` is called once each time the sequence moves on;
/// the supply gate alone can take thirty seconds, and a driver that said nothing
/// until the end would look hung.
pub fn drive<P, S>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    seq: &mut S,
    clock: &mut dyn Clock,
    budget: usize,
    phase: &mut dyn FnMut(SeqStep),
) -> Result<S::Summary, PumpError>
where
    P: BusPort,
    S: Sequencer,
{
    let mut prior: Option<BusResult> = None;
    let mut reported: Option<SeqStep> = None;

    for _ in 0..budget {
        let action = seq.next(clock.now(), prior.as_ref());
        let step = seq.step();
        if reported != Some(step) {
            phase(step);
            reported = Some(step);
        }
        prior = None;
        match action {
            SeqAction::Transact(request) => prior = Some(execute(bus, map, request)?),
            SeqAction::Wait { until } => clock.sleep_until(until),
            SeqAction::Done(summary) => return Ok(summary),
            SeqAction::Fail(error) => return Err(PumpError::Sequence(error)),
        }
    }
    Err(PumpError::Runaway { budget })
}

/// Run one request, and say how it came out in the sequencer's own vocabulary.
fn execute<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    request: BusRequest,
) -> Result<BusResult, PumpError> {
    let id = request.id();
    let row = map
        .ids()
        .iter()
        .position(|entry| *entry == id)
        .ok_or(PumpError::UnknownServo { id })?;

    match request {
        BusRequest::Ping { id } => match with_retry(bus, |bus| bus.ping(id)) {
            Ok(info) => Ok(BusResult::Pinged { model: info.model }),
            Err(error) => verdict(id, error),
        },
        BusRequest::ReadReg { id, reg } => {
            let entry = reg_for(reg);
            match with_retry(bus, |bus| bus.read_reg(id, entry)) {
                Ok(raw) => {
                    let value = map
                        .decode_value(row, reg, &raw)
                        .map_err(|source| PumpError::Map { id, reg, source })?;
                    Ok(BusResult::Value(value))
                }
                Err(error) => verdict(id, error),
            }
        }
        BusRequest::WriteRegVerified { id, reg, value } => {
            let entry = reg_for(reg);
            let raw = map
                .encode_value(row, reg, value)
                .map_err(|source| PumpError::Map { id, reg, source })?;
            match with_retry(bus, |bus| bus.write_reg_verified(id, entry, &raw)) {
                Ok(()) => Ok(BusResult::Written),
                Err(XactError::VerifyMismatch { read_back, .. }) => {
                    let held = map
                        .decode_value(row, reg, &read_back)
                        .map_err(|source| PumpError::Map { id, reg, source })?;
                    Ok(BusResult::VerifyMismatch { read_back: held })
                }
                Err(error) => verdict(id, error),
            }
        }
    }
}

/// A transaction failure as the machine's answer, or as the run's end.
///
/// The three that map are the three a servo can be responsible for. The retry
/// budget is already spent by the time a timeout arrives here, so silence means
/// silence.
fn verdict(id: u8, error: XactError) -> Result<BusResult, PumpError> {
    match error {
        XactError::Timeout { .. } => Ok(BusResult::NoAnswer),
        XactError::ServoError { error, .. } => Ok(BusResult::ServoError { code: error.0 }),
        XactError::Corrupt { .. } => Ok(BusResult::WireCorrupt),
        // A reply of the wrong width passed its own checksum, so it is neither
        // silence nor corruption, and no servo refused anything. There is no
        // honest BusResult for it, and inventing one would have a sequencer
        // report a phase failure for what is a host or wiring problem.
        other => Err(PumpError::Bus { id, source: other }),
    }
}

/// Arming's report, as a supervised run prints it.
///
/// Two records of the platform — where it was found and what it was left holding
/// — because they are different poses whenever a pin had to pull a joint, and
/// which one a reader is looking at matters. Everything else is the
/// registers-of-record a bring-up wants written down.
#[must_use]
pub fn arm_report(summary: &ArmSummary) -> String {
    let mut out = String::new();
    out.push_str(&record_lines("found", &summary.rest));
    out.push_str(&record_lines("armed", &summary.armed));

    let pulls: Vec<String> = summary
        .pull_in
        .iter()
        .map(|rad| format!("{:.3}", rad.to_degrees()))
        .collect();
    out.push_str(&format!(
        "pull-in    legs [{}] deg, worst {:.3} deg\n",
        pulls.join(", "),
        summary.worst_pull_in().to_degrees()
    ));
    out.push_str(&format!(
        "pull-in    antennas [{:.3}, {:.3}] deg, ungated\n",
        summary.antenna_pull_in[0].to_degrees(),
        summary.antenna_pull_in[1].to_degrees()
    ));
    // Two measurements rather than verdicts: nothing acts on either, and both
    // are quantities this project has so far only guessed at.
    let droop: Vec<String> = summary
        .droop
        .iter()
        .map(|gap| {
            gap.map_or_else(
                || "limp".to_string(),
                |rad| format!("{:.3}", rad.to_degrees()),
            )
        })
        .collect();
    out.push_str(&format!("droop      [{}] deg\n", droop.join(", ")));
    let shift: Vec<String> = summary
        .post_enable_shift
        .iter()
        .map(|rad| format!("{:.3}", rad.to_degrees()))
        .collect();
    out.push_str(&format!("torque-on  [{}] deg of shift\n", shift.join(", ")));

    out.push_str(&format!("models     {:?}\n", summary.models));
    let volts: Vec<String> = summary.voltages.iter().map(|v| format!("{v:.1}")).collect();
    out.push_str(&format!(
        "supply     [{}] V after {} poll(s)\n",
        volts.join(", "),
        summary.voltage_polls
    ));
    let health: Vec<String> = summary
        .health
        .iter()
        .map(|servo| format!("{:#04x}", servo.bits))
        .collect();
    out.push_str(&format!("health     [{}]\n", health.join(", ")));
    let torque: Vec<String> = summary
        .torque_before
        .iter()
        .map(|on| if *on { "on" } else { "off" }.to_string())
        .collect();
    out.push_str(&format!("torque was [{}]\n", torque.join(", ")));
    out.push_str(&format!(
        "registers  {} provisioned cells read\n",
        summary.provisioned.count()
    ));
    out
}

/// One of arming's two records of the platform.
fn record_lines(label: &str, record: &ArmRecord) -> String {
    let legs: Vec<String> = record
        .joints
        .legs
        .iter()
        .map(|rad| format!("{:.3}", rad.to_degrees()))
        .collect();
    format!(
        "{label:<10} head {height:.4} m, clearance {clearance:.3} mm\n\
         {blank:<10} legs [{legs}] deg, yaw {yaw:.3} deg, antennas [{right:.3}, {left:.3}] deg\n",
        height = record.head_pose_body.translation.z,
        clearance = record.min_margin * 1000.0,
        blank = "",
        legs = legs.join(", "),
        yaw = record.joints.body_yaw.to_degrees(),
        right = record.joints.antennas[0].to_degrees(),
        left = record.joints.antennas[1].to_degrees(),
    )
}

/// Which servos kept a grouped read from completing, and how.
///
/// A grouped read is all-or-nothing, so what survives it upward is a count of
/// unread periods and nothing else. The per-servo verdicts are what an operator
/// needs — which of six identical-looking legs, and whether it was silence, a
/// refusal or a frame of the wrong width — and they are in hand at the moment
/// of every miss. Fixed size, so naming them costs the loop no allocation.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ReadFailures {
    servos: [(u8, IdOutcome); JointId::COUNT],
    count: usize,
    corrupt: u32,
}

impl ReadFailures {
    /// The servos of `outcome` short of a clean answer, in the order they were
    /// asked.
    #[must_use]
    fn of(outcome: &SyncReadOutcome) -> Self {
        let mut failures = Self {
            servos: [(0, IdOutcome::Timeout); JointId::COUNT],
            count: 0,
            corrupt: outcome.corrupt_frames(),
        };
        for index in 0..outcome.len() {
            let Some((id, verdict)) = outcome.at(index) else {
                break;
            };
            if matches!(verdict, IdOutcome::Ok(_)) {
                continue;
            }
            if let Some(slot) = failures.servos.get_mut(failures.count) {
                *slot = (id, verdict);
                failures.count += 1;
            }
        }
        failures
    }

    /// The servos that fell short, with what each did instead.
    #[must_use]
    pub fn servos(&self) -> &[(u8, IdOutcome)] {
        &self.servos[..self.count]
    }

    /// Frames that arrived damaged during the read. Unattributable to a servo,
    /// so they are a count rather than a verdict against one.
    #[must_use]
    pub fn corrupt_frames(&self) -> u32 {
        self.corrupt
    }
}

impl fmt::Display for ReadFailures {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (index, (id, verdict)) in self.servos().iter().enumerate() {
            if index > 0 {
                f.write_str(", ")?;
            }
            write!(f, "servo {id} ")?;
            match verdict {
                IdOutcome::Timeout => f.write_str("silent")?,
                IdOutcome::ServoError(error) => write!(f, "refused with {:#04x}", error.0)?,
                IdOutcome::ShortReply { expected, actual }
                | IdOutcome::LongReply { expected, actual } => {
                    write!(f, "answered {actual} bytes of {expected}")?;
                }
                // Filtered out by `of`; rendering it beats a panic on a
                // reporting path.
                IdOutcome::Ok(_) => f.write_str("answered")?,
            }
        }
        if self.corrupt > 0 {
            if self.count > 0 {
                f.write_str(", ")?;
            }
            write!(f, "{} damaged frame(s)", self.corrupt)?;
        }
        Ok(())
    }
}

/// Something worth a line while a move runs.
///
/// A period at fifty hertz is not news; these are. A caller prints them as they
/// arrive, so a supervised run says what changed and nothing else.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum TickEvent {
    /// What became of the command the move was started with.
    Command(CommandDisposition),
    /// The first period of a run without a position read, and which servos kept
    /// the read from completing.
    ReadLost {
        /// The servos short of a clean answer.
        failed: ReadFailures,
    },
    /// A live read after a run of missing ones.
    ReadRestored {
        /// How many consecutive periods went unread.
        after: u32,
    },
    /// The first sweep of a run of health sweeps that did not complete, and
    /// which servos kept it from completing.
    HealthLost {
        /// The servos short of a clean answer.
        failed: ReadFailures,
    },
    /// A health sweep that completed after a run of failed ones.
    HealthRestored {
        /// How many consecutive sweeps fell short.
        after: u32,
    },
    /// The health sweep, the first time it ran and every time its answer
    /// changed. The input-voltage bit alone raises no fault and is reported
    /// exactly like any other byte.
    Health([ServoHealth; JointId::COUNT]),
    /// The first period that began materially late — at least half a period
    /// behind. Later ones are counted in the summary rather than printed one by
    /// one, and ordinary wake latency is neither printed nor counted.
    Overrun {
        /// Which period, counted from the start of the move.
        tick: u64,
        /// How far behind the fixed rate it started.
        late: Duration,
    },
    /// The move reached its endpoint.
    Completed,
    /// The tick stopped commanding.
    Faulted(Fault),
}

impl fmt::Display for TickEvent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Command(CommandDisposition::None) => f.write_str("no command"),
            Self::Command(CommandDisposition::Started) => f.write_str("moving"),
            Self::Command(CommandDisposition::Held) => f.write_str("holding"),
            Self::Command(CommandDisposition::Rejected(why)) => write!(f, "refused: {why}"),
            Self::ReadLost { failed } => write!(f, "position read lost: {failed}"),
            Self::ReadRestored { after } => {
                write!(f, "position read back after {after} period(s)")
            }
            Self::HealthLost { failed } => write!(f, "health sweep lost: {failed}"),
            Self::HealthRestored { after } => {
                write!(f, "health sweep back after {after} sweep(s)")
            }
            Self::Health(servos) => {
                write!(f, "health")?;
                for servo in servos {
                    write!(f, " {}:{:#04x}", servo.id, servo.bits)?;
                }
                Ok(())
            }
            Self::Overrun { tick, late } => {
                write!(
                    f,
                    "period {tick} began {:.1} ms late",
                    late.as_secs_f64() * 1e3
                )
            }
            Self::Completed => f.write_str("at the target"),
            Self::Faulted(fault) => write!(f, "faulted: {fault}"),
        }
    }
}

/// What a move cost, once it is over.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MoveSummary {
    /// Control periods run.
    pub ticks: u64,
    /// Periods that emitted goals. A period whose goals equal the last ones
    /// emits nothing and the servos hold.
    pub goals: u64,
    /// Grouped write frames sent. Fewer than three per emitting period whenever
    /// a group did not change.
    pub frames: u64,
    /// Periods with no position read.
    pub misses: u64,
    /// Health sweeps that did not complete. Counted in sweeps, not periods:
    /// the poll runs at its own rate.
    pub health_misses: u64,
    /// Periods that began materially late — at least half a period after they
    /// were due. Ordinary wake latency is not counted here; it is in
    /// `worst_jitter`.
    pub overruns: u64,
    /// The furthest any period began behind its due time, counted or not. The
    /// scale a loop's wake latency runs at, which is what says whether the
    /// overrun tolerance is the right size on a given host.
    pub worst_jitter: Duration,
    /// Wall time from the first period to the last.
    pub elapsed: Duration,
}

/// What ends a run of the fixed-rate loop.
#[derive(Clone, Copy, Debug)]
enum Until {
    /// The machine is holding again: the move reached its endpoint.
    Holding,
    /// The dwell has elapsed. A hold is holding from its first period, so
    /// there is nothing else for it to wait for.
    Elapsed(Duration),
}

/// The fixed-rate loop that carries one move.
///
/// Holds the loop-invariant configuration and the buffers a period reuses — the
/// tick's output, the two grouped-read outcomes — so a running move allocates
/// nothing per period.
///
/// It also holds what the servos were last told, per joint, which is what makes
/// the per-group write comparison possible. That starts as the goals arming
/// pinned: those are the values sitting in the goal registers when a move
/// begins, so a move that does not touch the antennas never writes them.
pub struct MotionPump<'a> {
    cfg: &'a MotionConfig,
    map: &'a ServoMap,
    period: Duration,
    health_every: u64,
    stall_margin: u64,
    written: JointVector,
    outputs: TickOutputs,
    present: SyncReadOutcome,
    health: SyncReadOutcome,
}

impl<'a> MotionPump<'a> {
    /// A pump for a machine currently holding `held`.
    ///
    /// `held` is arming's record of the goals it left in the servos. Getting it
    /// wrong costs correctness only in what is *not* sent: a group believed
    /// unchanged is a group whose frame never goes out.
    pub fn new(
        cfg: &'a MotionConfig,
        map: &'a ServoMap,
        tick_hz: u32,
        health_poll_hz: u32,
        held: JointVector,
    ) -> Result<Self, PumpError> {
        if tick_hz == 0 || health_poll_hz == 0 {
            return Err(PumpError::Rate {
                tick_hz,
                health_poll_hz,
            });
        }
        Ok(Self {
            cfg,
            map,
            period: Duration::from_secs(1) / tick_hz,
            // A health poll slower than the tick, rounded down to whole
            // periods, and never rarer than never: a poll rate at or above the
            // tick rate polls every period.
            health_every: u64::from((tick_hz / health_poll_hz).max(1)),
            stall_margin: u64::from(tick_hz) * STALL_MARGIN_SECS,
            written: held,
            outputs: TickOutputs::default(),
            present: SyncReadOutcome::new(),
            health: SyncReadOutcome::new(),
        })
    }

    /// The control period.
    #[must_use]
    pub fn period(&self) -> Duration {
        self.period
    }

    /// How many periods `command` may take before the loop calls it stuck: the
    /// travel it asked for, plus the fixed margin.
    ///
    /// Sized from the command rather than fixed, because a move's duration is
    /// configuration with no ceiling: a bound that did not follow it would abort
    /// a legal move mid-travel and report it as a loop that hung.
    #[must_use]
    pub fn stall_budget(&self, command: &MotionCommand) -> u64 {
        match command {
            MotionCommand::MoveTo { duration, .. } => self.budget_for(*duration),
            // A hold commands no travel: it takes the period it is asked in.
            MotionCommand::Hold => self.budget_for(Duration::ZERO),
        }
    }

    /// How many periods a run spanning `wanted` may take before the loop calls
    /// it stuck: the periods that span it, plus the fixed margin.
    fn budget_for(&self, wanted: Duration) -> u64 {
        let periods = wanted.as_nanos().div_ceil(self.period.as_nanos());
        u64::try_from(periods)
            .unwrap_or(u64::MAX)
            .saturating_add(self.stall_margin)
    }

    /// Run `command` to completion.
    ///
    /// Returns when the machine is holding again — the move reached its
    /// endpoint, or the command was a hold — and refuses on anything else: a
    /// refused command, a fault, a transaction that is not a machine verdict.
    /// Nothing here touches torque on the way out, whichever exit is taken.
    pub fn run<P: BusPort>(
        &mut self,
        bus: &mut Bus<P>,
        state: &mut MotionState,
        command: MotionCommand,
        clock: &mut dyn Clock,
        event: &mut dyn FnMut(TickEvent),
    ) -> Result<MoveSummary, PumpError> {
        self.carry(bus, state, command, Until::Holding, clock, event)
    }

    /// Watch the machine hold for `dwell`, commanding nothing.
    ///
    /// A hold is holding from its first period, so what makes this a
    /// measurement rather than a sleep is that every period still reads all
    /// nine positions and runs the tick: the tracking monitor, the read-loss
    /// budget and the health poll all apply, and the goals are already where
    /// they belong so nothing is written.
    ///
    /// Paced by this loop and not by the caller, which is what lets a hold
    /// report its own lateness — the telemetry the command exists for.
    pub fn hold<P: BusPort>(
        &mut self,
        bus: &mut Bus<P>,
        state: &mut MotionState,
        dwell: Duration,
        clock: &mut dyn Clock,
        event: &mut dyn FnMut(TickEvent),
    ) -> Result<MoveSummary, PumpError> {
        self.carry(
            bus,
            state,
            MotionCommand::Hold,
            Until::Elapsed(dwell),
            clock,
            event,
        )
    }

    /// The fixed-rate loop itself, run until `until` says the run is over.
    fn carry<P: BusPort>(
        &mut self,
        bus: &mut Bus<P>,
        state: &mut MotionState,
        command: MotionCommand,
        until: Until,
        clock: &mut dyn Clock,
        event: &mut dyn FnMut(TickEvent),
    ) -> Result<MoveSummary, PumpError> {
        let budget = match until {
            Until::Holding => self.stall_budget(&command),
            Until::Elapsed(dwell) => self.budget_for(dwell),
        };
        let epoch = clock.now();
        let mut due = epoch;
        let mut pending = Some(command);
        let mut summary = MoveSummary::default();
        let mut last_misses = 0;
        let mut health_misses: u32 = 0;
        let mut reported_health: Option<[ServoHealth; JointId::COUNT]> = None;
        let mut overrun_reported = false;

        for tick in 0..budget {
            let now = clock.now();
            // A period that starts materially after it was due is one the loop
            // ran late for. It proceeds immediately rather than skipping the
            // next one: the reads stay one per period and the trajectory is
            // sampled at the time it is actually being sampled at.
            let late = now.saturating_sub(due);
            summary.worst_jitter = summary.worst_jitter.max(late);
            if late >= self.period / OVERRUN_DIVISOR {
                summary.overruns += 1;
                if !overrun_reported {
                    event(TickEvent::Overrun { tick, late });
                    overrun_reported = true;
                }
            }

            let present = self.read_present(bus)?;
            let health = if tick % self.health_every == 0 {
                match self.read_health(bus)? {
                    Some(servos) => {
                        if health_misses > 0 {
                            event(TickEvent::HealthRestored {
                                after: health_misses,
                            });
                            health_misses = 0;
                        }
                        Some(servos)
                    }
                    None => {
                        health_misses += 1;
                        summary.health_misses += 1;
                        if health_misses == 1 {
                            event(TickEvent::HealthLost {
                                failed: ReadFailures::of(&self.health),
                            });
                        }
                        None
                    }
                }
            } else {
                None
            };

            let inputs = TickInputs {
                now,
                present: present.as_ref(),
                command: pending.as_ref(),
                health: health.as_ref(),
            };
            motion_tick(self.cfg, state, &inputs, &mut self.outputs);
            let asked = pending.take().is_some();
            summary.ticks += 1;

            if let Some(goal) = self.outputs.goal {
                summary.frames += self.write_goals(bus, &goal)?;
                summary.goals += 1;
            }

            let report = self.outputs.report;
            if asked {
                event(TickEvent::Command(report.command));
                if let CommandDisposition::Rejected(why) = report.command {
                    return Err(PumpError::Rejected(why));
                }
            }
            if report.misses > 0 {
                summary.misses += 1;
            }
            if report.misses == 1 && last_misses == 0 {
                event(TickEvent::ReadLost {
                    failed: ReadFailures::of(&self.present),
                });
            } else if report.misses == 0 && last_misses > 0 {
                event(TickEvent::ReadRestored { after: last_misses });
            }
            last_misses = report.misses;
            if let Some(servos) = report.health
                && reported_health != Some(servos)
            {
                event(TickEvent::Health(servos));
                reported_health = Some(servos);
            }
            if let Some(fault) = report.fault {
                event(TickEvent::Faulted(fault));
                return Err(PumpError::Fault(fault));
            }
            if report.completed {
                event(TickEvent::Completed);
            }
            let over = match until {
                Until::Holding => matches!(state.mode(), Mode::Holding),
                Until::Elapsed(dwell) => now.saturating_sub(epoch) >= dwell,
            };
            if over {
                summary.elapsed = clock.now().saturating_sub(epoch);
                return Ok(summary);
            }

            due += self.period;
            clock.sleep_until(due);
        }
        Err(PumpError::Stalled { budget })
    }

    /// All nine positions, or nothing at all.
    ///
    /// Any servo short of a clean answer makes the period's measurement absent.
    /// The tick's read-loss budget is the compensating mechanism, and it is the
    /// only one: retrying inside a control period would spend the period the
    /// retry was meant to serve.
    fn read_present<P: BusPort>(
        &mut self,
        bus: &mut Bus<P>,
    ) -> Result<Option<JointVector>, PumpError> {
        bus.sync_read(
            &self.map.ids(),
            reg_for(RegId::PresentPosition),
            &mut self.present,
        )
        .map_err(bus_failure)?;
        if !self.present.all_ok() {
            return Ok(None);
        }

        let mut joints = JointVector::default();
        for (row, joint) in JointId::ALL.iter().enumerate() {
            let counts = self
                .present
                .at(row)
                .and_then(|(_, outcome)| outcome.value()?.i32())
                .expect(ALL_OK_FILLS_EVERY_ROW);
            let angle = self
                .map
                .present_rad(row, counts)
                .map_err(|source| self.map_failure(row, RegId::PresentPosition, source))?;
            joints.set(*joint, angle);
        }
        Ok(Some(joints))
    }

    /// All nine hardware-error bytes, or nothing at all — same all-or-nothing
    /// rule as the position read, for the same reason: a health verdict on
    /// eight servos is not a health verdict.
    ///
    /// Where it differs from the position read is what an absent sweep costs.
    /// The position read has the tick's miss budget behind it, ending in a
    /// typed fault; a sweep that falls short is reported to the operator as it
    /// happens and counted, and the move carries on with no health verdict for
    /// as long as the sweeps keep falling short.
    ///
    /// TODO(health-read-budget): decide whether a run of failed sweeps should
    /// stop the loop the way a run of missed position reads does.
    fn read_health<P: BusPort>(
        &mut self,
        bus: &mut Bus<P>,
    ) -> Result<Option<[ServoHealth; JointId::COUNT]>, PumpError> {
        bus.sync_read(
            &self.map.ids(),
            reg_for(RegId::HardwareErrorStatus),
            &mut self.health,
        )
        .map_err(bus_failure)?;
        if !self.health.all_ok() {
            return Ok(None);
        }

        let mut servos = [ServoHealth::default(); JointId::COUNT];
        for (row, servo) in servos.iter_mut().enumerate() {
            let (id, outcome) = self.health.at(row).expect(ALL_OK_FILLS_EVERY_ROW);
            let bits = outcome
                .value()
                .and_then(|raw| raw.u8())
                .expect(ALL_OK_FILLS_EVERY_ROW);
            *servo = ServoHealth { id, bits };
        }
        Ok(Some(servos))
    }

    /// Write the goal groups that changed, and report how many frames that took.
    ///
    /// One grouped frame per group, and only for the groups whose goals changed
    /// since the last write — so a period that moves only the head sends one
    /// frame rather than nine goals. Which bus rows a group covers is
    /// [`JointGroup`]'s to say, not this loop's.
    fn write_goals<P: BusPort>(
        &mut self,
        bus: &mut Bus<P>,
        goal: &JointVector,
    ) -> Result<u64, PumpError> {
        let mut frames = 0;
        for group in JointGroup::ALL {
            let mut rows = [0usize; JointId::COUNT];
            let mut in_group = 0;
            for (row, joint) in JointId::ALL.into_iter().enumerate() {
                if joint.group() == group {
                    rows[in_group] = row;
                    in_group += 1;
                }
            }
            let rows = &rows[..in_group];

            let changed = rows.iter().any(|row| {
                let joint = JointId::ALL[*row];
                goal.get(joint) != self.written.get(joint)
            });
            if !changed {
                continue;
            }

            let mut entries = [(0u8, RawValue::default()); JointId::COUNT];
            let mut count = 0;
            for row in rows {
                let joint = JointId::ALL[*row];
                let angle = goal.get(joint).expect(BUS_ORDER_IS_NINE_JOINTS);
                // Through the map, as every other write in this binary is: it
                // is the one place a model angle becomes counts, and a goal the
                // count range cannot hold is refused rather than wrapped.
                let raw = self
                    .map
                    .encode_value(*row, RegId::GoalPosition, RegValue::Radians(angle))
                    .map_err(|source| self.map_failure(*row, RegId::GoalPosition, source))?;
                entries[count] = (self.servo_at(*row), raw);
                count += 1;
            }
            bus.sync_write(reg_for(RegId::GoalPosition), &entries[..count])
                .map_err(bus_failure)?;
            frames += 1;

            for row in rows {
                let joint = JointId::ALL[*row];
                self.written
                    .set(joint, goal.get(joint).expect(BUS_ORDER_IS_NINE_JOINTS));
            }
        }
        Ok(frames)
    }

    /// The servo a bus row addresses.
    fn servo_at(&self, row: usize) -> u8 {
        self.map
            .ids()
            .get(row)
            .copied()
            .expect(BUS_ORDER_IS_NINE_JOINTS)
    }

    /// A conversion the map refused, named against the servo it was for.
    fn map_failure(&self, row: usize, reg: RegId, source: MapError) -> PumpError {
        PumpError::Map {
            id: self.servo_at(row),
            reg,
            source,
        }
    }
}

/// A grouped transaction that failed as a whole.
///
/// Every per-servo verdict a grouped read can produce is in its own slot, so an
/// error out of one of these calls is never a machine's answer: it is the port,
/// the encoder, or a caller's arithmetic.
fn bus_failure(source: XactError) -> PumpError {
    PumpError::Bus {
        id: source.id(),
        source,
    }
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::rc::Rc;

    use dxl_proto::frame::{INST_PING, INST_READ, INST_SYNC_READ, INST_SYNC_WRITE, INST_WRITE};
    use reachy_bus::reg_for;
    use reachy_motion::{ArmSequencer, JointId, JointTargets, RegValue, Warp};

    use super::*;
    use crate::config::Resolved;
    use crate::testutil::{
        BrokenPort, FakeMachine, Spy, TestClock, datumed_config, machine_at, resolved, stow_legs,
        wind_down_bus,
    };

    /// Drive an arm sequence against `machine`, handing back the outcome, the
    /// phases as they were announced, and every instruction that crossed the
    /// wire.
    #[allow(clippy::type_complexity)]
    fn arm(
        cfg: &Resolved,
        machine: FakeMachine,
    ) -> (
        Result<ArmSummary, PumpError>,
        Vec<SeqStep>,
        Rc<RefCell<Vec<(u8, u8)>>>,
    ) {
        let spy = Spy::new(machine);
        let log = spy.log();
        let mut bus = Bus::new(spy, cfg.timing);
        let mut seq =
            ArmSequencer::new(&cfg.arm, &cfg.motion.geom, &cfg.motion.env, &cfg.motion.fk);
        let mut clock = TestClock::default();
        let mut phases = Vec::new();
        let outcome = drive(
            &mut bus,
            &cfg.map,
            &mut seq,
            &mut clock,
            action_budget(&cfg.arm),
            &mut |step| {
                phases.push(step);
            },
        );
        (outcome, phases, log)
    }

    /// A machine provisioned as configured and resting where it can be armed
    /// goes all the way through, and the goals that reached it are the pins.
    #[test]
    fn an_arm_sequence_runs_to_completion_over_the_port() {
        let cfg = resolved();
        let (outcome, phases, log) = arm(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let summary = outcome.expect("a correct machine arms");

        // Nothing had to be pulled: this machine rests inside every window and
        // with both antennas well inside their bound.
        assert_eq!(summary.pull_in, [0.0; 6]);
        assert_eq!(summary.antenna_pull_in, [0.0; 2]);
        // Nothing was found holding torque, so there is no droop to record.
        assert!(summary.droop.iter().all(Option::is_none));
        assert!(summary.torque_before.iter().all(|on| !on));
        assert_eq!(summary.voltage_polls, 1);

        assert_eq!(
            phases,
            vec![
                SeqStep::Presence,
                SeqStep::Identity,
                SeqStep::Provision,
                SeqStep::VoltageGate,
                SeqStep::Health,
                SeqStep::PoseAndDatum,
                SeqStep::StateDiscovery,
                SeqStep::GoalShadow,
                SeqStep::GainsProfiles,
                SeqStep::PinAndEnable,
            ]
        );

        // Writes happened, and none of them before the supply gate passed.
        let traffic = log.borrow();
        let first_write = traffic
            .iter()
            .position(|(_, instruction)| *instruction == INST_WRITE)
            .expect("arming writes");
        let reads_before = traffic[..first_write]
            .iter()
            .all(|(_, instruction)| *instruction == INST_PING || *instruction == INST_READ);
        assert!(reads_before, "nothing is written before the reads finish");
    }

    /// The goals left in the servos are the angles arming says it pinned, read
    /// back off the fixture's own register file rather than off the summary.
    #[test]
    fn the_goals_left_in_the_servos_are_the_pins() {
        let cfg = resolved();
        let spy = Spy::new(machine_at(&datumed_config(), &stow_legs()));
        let registers = spy.machine();
        let mut bus = Bus::new(spy, cfg.timing);
        let mut seq =
            ArmSequencer::new(&cfg.arm, &cfg.motion.geom, &cfg.motion.env, &cfg.motion.fk);
        let mut clock = TestClock::default();
        let summary = drive(
            &mut bus,
            &cfg.map,
            &mut seq,
            &mut clock,
            action_budget(&cfg.arm),
            &mut |_| {},
        )
        .expect("a correct machine arms");

        let machine = registers.borrow();
        for (row, id) in cfg.map.ids().iter().enumerate() {
            let held = machine
                .get(*id, reg_for(RegId::GoalPosition))
                .expect("a goal was written to every servo");
            let counts = i32::from_le_bytes(held.try_into().expect("a goal is four bytes"));
            let pinned = JointId::from_index(row)
                .and_then(|joint| summary.armed.joints.get(joint))
                .expect("the bus rows are the nine joints");
            let expected = cfg
                .map
                .goal_counts(row, pinned)
                .expect("a pinned angle places");
            assert_eq!(counts, expected, "servo {id}");
            assert_eq!(
                machine.get(*id, reg_for(RegId::TorqueEnable)),
                Some(&[1u8][..]),
                "servo {id} holds torque"
            );
        }
    }

    /// A second arm in one power cycle passes, over the machine the first one
    /// left behind.
    ///
    /// The lifecycle re-arms in every process, so this is the ordinary case and
    /// not an unusual one: the second arm finds the RAM the first wrote — the
    /// configured profile in the profile registers — and every servo still
    /// holding torque at the goal it was pinned to. Nothing about that is a
    /// disagreement with the platform's provisioning, and a sweep that judged
    /// those registers would refuse every command after the first arm until
    /// somebody power-cycled the unit.
    ///
    /// It also carries the torqued-before path end to end: the shadow assertion
    /// is skipped for a servo that is really holding, the droop is recorded, and
    /// the pins come off the held goals rather than off the sagged positions.
    #[test]
    fn a_second_arm_in_one_power_cycle_passes_over_the_machine_the_first_left() {
        let cfg = resolved();
        let registers = Rc::new(RefCell::new(machine_at(&datumed_config(), &stow_legs())));

        let drive_one = |registers: &Rc<RefCell<FakeMachine>>| {
            let mut bus = Bus::new(Spy::sharing(Rc::clone(registers)), cfg.timing);
            let mut seq =
                ArmSequencer::new(&cfg.arm, &cfg.motion.geom, &cfg.motion.env, &cfg.motion.fk);
            let mut clock = TestClock::default();
            drive(
                &mut bus,
                &cfg.map,
                &mut seq,
                &mut clock,
                action_budget(&cfg.arm),
                &mut |_| {},
            )
        };

        let first = drive_one(&registers).expect("a correct machine arms");
        assert!(first.torque_before.iter().all(|on| !on));

        // What the first arm wrote is what the second one finds: the profile
        // registers hold the configured figures, not the zero a fresh power-on
        // reads.
        {
            let machine = registers.borrow();
            for id in cfg.map.ids() {
                let held = machine
                    .get(id, reg_for(RegId::ProfileAcceleration))
                    .expect("arming wrote the profile");
                assert_eq!(
                    u32::from_le_bytes(held.try_into().expect("a profile is four bytes")),
                    cfg.arm.profile.acceleration,
                    "servo {id}"
                );
            }
        }

        let second = drive_one(&registers).expect("a second arm in one power cycle passes");
        assert!(
            second.torque_before.iter().all(|on| *on),
            "the second arm finds the machine holding"
        );
        assert!(
            second.droop.iter().all(Option::is_some),
            "every held servo's droop is recorded"
        );
        // The pins did not ratchet: the second arm pinned where the first did.
        for joint in JointId::ALL {
            let before = first.armed.joints.get(joint).expect("a pinned joint");
            let after = second.armed.joints.get(joint).expect("a pinned joint");
            assert!((before - after).abs() < 1e-12, "{joint}");
        }
    }

    /// A silent servo is silence, and the sequence stops with the refusal it
    /// earned rather than with a driver-level failure.
    #[test]
    fn a_silent_servo_stops_the_sequence_with_its_own_refusal() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        machine.silent = vec![13];
        let (outcome, phases, _) = arm(&cfg, machine);

        let refused = outcome.expect_err("a silent servo does not arm");
        assert!(
            matches!(refused, PumpError::Sequence(SeqError::AbsentServos { .. })),
            "{refused}"
        );
        assert!(refused.to_string().contains("13"), "{refused}");
        assert_eq!(phases, vec![SeqStep::Presence]);
    }

    /// A servo that answers with its error field set is a refusal, carried
    /// through with its code intact.
    #[test]
    fn a_refusal_arrives_as_the_code_the_servo_sent() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        machine
            .errors
            .insert((14, reg_for(RegId::HardwareErrorStatus).addr), 0x07);
        let (outcome, _, _) = arm(&cfg, machine);

        let refused = outcome.expect_err("a refused read does not arm");
        assert!(
            matches!(
                refused,
                PumpError::Sequence(SeqError::Refused { code: 0x07, .. })
            ),
            "{refused}"
        );
    }

    /// Bytes that arrive and disagree with themselves are the wire's fault, and
    /// they reach the sequencer as such — with the phase, the servo and the
    /// register on them, which is what separates a noisy wire from a sick servo
    /// on a bench session.
    #[test]
    fn a_damaged_reply_reaches_the_sequencer_as_a_wire_fault() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        machine
            .damaged
            .push((14, reg_for(RegId::HardwareErrorStatus).addr));
        let (outcome, _, _) = arm(&cfg, machine);

        let refused = outcome.expect_err("a damaged reply does not arm");
        let PumpError::Sequence(SeqError::WireCorrupt { context }) = refused else {
            panic!("expected a wire fault, got {refused}");
        };
        assert_eq!(context.id, 14);
        assert_eq!(context.step, SeqStep::Health);
        assert_eq!(context.reg, Some(RegId::HardwareErrorStatus));
    }

    /// A register that acknowledges a write and does not take it comes back as
    /// the value it actually holds, which is what the sequencer compares.
    #[test]
    fn a_read_back_mismatch_carries_what_the_register_holds() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        machine
            .ignored
            .push((12, reg_for(RegId::PositionGains).addr));
        let (outcome, _, _) = arm(&cfg, machine);

        let refused = outcome.expect_err("a gain that will not take does not arm");
        let PumpError::Sequence(SeqError::VerifyMismatch {
            context, read_back, ..
        }) = refused
        else {
            panic!("expected a read-back mismatch, got {refused}");
        };
        assert_eq!(context.id, 12);
        assert_eq!(read_back, RegValue::Gains { p: 0, i: 0, d: 0 });
    }

    /// The gains and profiles left in the servos are the configured ones, each
    /// joint group's own and each register's own.
    ///
    /// Read off the fixture's register file rather than off the traffic: what
    /// matters is what the machine is holding when arming is done with it, and
    /// these are the registers the whole motion path's behaviour rests on.
    #[test]
    fn the_gains_and_profiles_left_in_the_servos_are_the_configured_ones() {
        let cfg = resolved();
        let spy = Spy::new(machine_at(&datumed_config(), &stow_legs()));
        let registers = spy.machine();
        let mut bus = Bus::new(spy, cfg.timing);
        let mut seq =
            ArmSequencer::new(&cfg.arm, &cfg.motion.geom, &cfg.motion.env, &cfg.motion.fk);
        let mut clock = TestClock::default();
        drive(
            &mut bus,
            &cfg.map,
            &mut seq,
            &mut clock,
            action_budget(&cfg.arm),
            &mut |_| {},
        )
        .expect("a correct machine arms");

        // The legs carry the head and are tuned apart from the other three, and
        // the two profile figures differ from each other — so a lookup that
        // collapsed the groups, or a register written with its neighbour's
        // figure, has somewhere to show.
        assert_ne!(cfg.arm.gains.legs, cfg.arm.gains.yaw);
        assert_ne!(cfg.arm.profile.acceleration, cfg.arm.profile.velocity);

        let machine = registers.borrow();
        for (row, id) in cfg.map.ids().iter().enumerate() {
            let joint = JointId::ALL[row];
            for (reg, value) in [
                (RegId::PositionGains, cfg.arm.gains.for_joint(joint).into()),
                (
                    RegId::ProfileAcceleration,
                    RegValue::U32(cfg.arm.profile.acceleration),
                ),
                (
                    RegId::ProfileVelocity,
                    RegValue::U32(cfg.arm.profile.velocity),
                ),
            ] {
                let raw = cfg
                    .map
                    .encode_value(row, reg, value)
                    .expect("a configured figure encodes");
                assert_eq!(
                    machine.get(*id, reg_for(reg)),
                    Some(raw.as_slice()),
                    "servo {id}, {reg}"
                );
            }
        }
    }

    /// A reply that goes missing once is retried on the configured budget, and
    /// the sequencer never learns it happened.
    ///
    /// The retry budget is the driver's to spend: a sequencer told "no answer"
    /// when one dropped reply would have come back on a second ask refuses a
    /// healthy machine, and the same budget spent gives "silent" its meaning.
    #[test]
    fn a_dropped_reply_inside_the_retry_budget_is_retried() {
        let position = reg_for(RegId::PresentPosition).addr;

        let mut generous = datumed_config();
        wind_down_bus(&mut generous);
        generous.bus.retry_attempts = 2;
        let generous = generous.resolve().expect("a datumed example resolves");
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        machine.mute.insert((13, position), 1);
        let (outcome, _, _) = arm(&generous, machine);
        outcome.expect("one dropped reply inside the budget still arms");

        // The same machine with nothing to spend: one miss is silence.
        let single = resolved();
        assert_eq!(single.timing.retry_attempts, 1);
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        machine.mute.insert((13, position), 1);
        let (outcome, _, _) = arm(&single, machine);
        let refused = outcome.expect_err("a single attempt has nothing to fall back on");
        let PumpError::Sequence(SeqError::NoAnswer { context }) = refused else {
            panic!("expected silence, got {refused}");
        };
        assert_eq!(context.id, 13);
        assert_eq!(context.step, SeqStep::PoseAndDatum);
    }

    /// A transaction's result reaches the action that earned it, and nothing
    /// after that.
    ///
    /// A sequencer absorbs whatever it is handed as the answer to its own last
    /// request, so a result carried past a wait would be read as the answer to
    /// a request that was never made.
    #[test]
    fn a_result_answers_the_transaction_that_earned_it() {
        let cfg = resolved();
        let mut bus = Bus::new(
            Spy::new(machine_at(&datumed_config(), &stow_legs())),
            cfg.timing,
        );
        let mut seq = Recorder::default();
        let mut clock = TestClock::default();
        drive(&mut bus, &cfg.map, &mut seq, &mut clock, 16, &mut |_| {})
            .expect("the scripted sequence finishes");

        // Opening call, then the ping's answer, then the wait — which answers
        // nothing — then the second ping's answer.
        assert_eq!(seq.handed, vec![false, true, false, true]);
    }

    /// The supply gate's waits are the driver's, and a rail that never comes up
    /// spends the whole budget in poll-sized steps before refusing.
    #[test]
    fn the_supply_gate_polls_on_the_drivers_clock() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        for id in cfg.map.ids() {
            // Half the arming floor, on every servo.
            machine.set(
                id,
                reg_for(RegId::PresentInputVoltage),
                &30u16.to_le_bytes(),
            );
        }
        let mut bus = Bus::new(Spy::new(machine), cfg.timing);
        let mut seq =
            ArmSequencer::new(&cfg.arm, &cfg.motion.geom, &cfg.motion.env, &cfg.motion.fk);
        let mut clock = TestClock::default();
        let refused = drive(
            &mut bus,
            &cfg.map,
            &mut seq,
            &mut clock,
            action_budget(&cfg.arm),
            &mut |_| {},
        )
        .expect_err("a rail under the floor does not arm");

        assert!(
            matches!(refused, PumpError::Sequence(SeqError::VoltageLow { .. })),
            "{refused}"
        );
        let waits = clock.waits;
        assert!(waits.len() > 1, "the gate polled more than once: {waits:?}");
        // Each wait is one poll period after the one before it, and the last is
        // inside the budget the gate was given.
        for pair in waits.windows(2) {
            assert_eq!(pair[1] - pair[0], cfg.arm.voltage_poll_period);
        }
        assert!(waits[waits.len() - 1] <= cfg.arm.voltage_budget);
    }

    /// The port failing is not the machine answering. A run that cannot reach
    /// the wire says so, instead of reporting nine absent servos.
    #[test]
    fn a_port_failure_is_not_a_machine_verdict() {
        let cfg = resolved();
        let mut bus = Bus::new(BrokenPort, cfg.timing);
        let mut seq =
            ArmSequencer::new(&cfg.arm, &cfg.motion.geom, &cfg.motion.env, &cfg.motion.fk);
        let mut clock = TestClock::default();
        let refused = drive(
            &mut bus,
            &cfg.map,
            &mut seq,
            &mut clock,
            action_budget(&cfg.arm),
            &mut |_| {},
        )
        .expect_err("a dead port does not arm");

        let PumpError::Bus { id, source } = refused else {
            panic!("expected a bus failure, got {refused}");
        };
        assert_eq!(id, cfg.map.ids()[0]);
        assert!(matches!(source, XactError::Io { .. }), "{source}");
    }

    /// A sequencer that asks for a servo the roster does not carry is stopped
    /// before a frame goes out to whatever holds that ID.
    #[test]
    fn a_servo_outside_the_roster_is_refused_before_the_wire() {
        let cfg = resolved();
        let spy = Spy::new(machine_at(&datumed_config(), &stow_legs()));
        let log = spy.log();
        let mut bus = Bus::new(spy, cfg.timing);
        let mut seq = Toy::Addressing(200);
        let mut clock = TestClock::default();
        let refused = drive(
            &mut bus,
            &cfg.map,
            &mut seq,
            &mut clock,
            action_budget(&cfg.arm),
            &mut |_| {},
        )
        .expect_err("servo 200 is not in the roster");

        assert!(
            matches!(refused, PumpError::UnknownServo { id: 200 }),
            "{refused}"
        );
        assert!(log.borrow().is_empty(), "nothing was put on the wire");
    }

    /// A sequencer that never terminates is stopped by the budget rather than
    /// hanging a bench session.
    #[test]
    fn a_sequence_that_never_finishes_is_bounded() {
        let cfg = resolved();
        let mut bus = Bus::new(FakeMachine::new(), cfg.timing);
        let mut seq = Toy::Waiting;
        let mut clock = TestClock::default();
        let refused = drive(
            &mut bus,
            &cfg.map,
            &mut seq,
            &mut clock,
            action_budget(&cfg.arm),
            &mut |_| {},
        )
        .expect_err("a sequence that only waits never finishes");

        // Equality, not a `matches!` pattern: a bare name in a pattern binds
        // rather than compares, so a wrong budget would have passed.
        let PumpError::Runaway { budget } = refused else {
            panic!("expected a runaway, got {refused}");
        };
        assert_eq!(budget, action_budget(&cfg.arm));
    }

    /// The action budget follows the supply gate's configuration: a gate told
    /// to poll a hundred times faster gets a budget to match, rather than being
    /// cut short and blamed for polling as it was configured to.
    #[test]
    fn the_action_budget_covers_the_configured_supply_gate() {
        /// The actions the gate itself will take under `arm`.
        fn gate_actions(arm: &reachy_motion::ArmConfig) -> usize {
            let cycles = arm
                .voltage_budget
                .as_nanos()
                .div_ceil(arm.voltage_poll_period.as_nanos());
            usize::try_from(cycles).expect("a configured gate is countable") * ACTIONS_PER_POLL
        }

        let cfg = resolved();
        assert!(action_budget(&cfg.arm) > gate_actions(&cfg.arm));

        let mut fast = cfg.arm.clone();
        fast.voltage_poll_period = Duration::from_millis(1);
        assert!(
            action_budget(&fast) > gate_actions(&fast),
            "a gate polled every millisecond takes {} actions",
            gate_actions(&fast)
        );
        assert!(
            action_budget(&fast) > 10 * action_budget(&cfg.arm),
            "the budget follows the configuration rather than sitting still"
        );

        // Nothing here divides by a spacing of zero, however it was built.
        let mut stopped = cfg.arm.clone();
        stopped.voltage_poll_period = Duration::ZERO;
        assert!(action_budget(&stopped) >= FIXED_ACTIONS);
    }

    /// The report names both records of the platform and every register of
    /// record, so a supervised arm leaves a readable trace.
    #[test]
    fn the_report_names_both_records_and_the_registers() {
        let cfg = resolved();
        let (outcome, _, _) = arm(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let printed = arm_report(&outcome.expect("a correct machine arms"));

        for expected in [
            "found",
            "armed",
            "pull-in",
            "droop",
            "torque-on",
            "models",
            "supply",
            "health",
            "torque was",
            "registers",
        ] {
            assert!(
                printed.contains(expected),
                "{expected} missing from\n{printed}"
            );
        }
        assert!(
            printed.contains("1200"),
            "the models are printed:\n{printed}"
        );
        assert!(printed.contains("11.8"), "the rail is printed:\n{printed}");
    }

    /// The report over a machine whose numbers differ from each other and from
    /// zero: the parked antennas of a platform found as its own stow left it.
    ///
    /// The fixture above cannot see a units error, a left-for-right swap or a
    /// line rendered off the wrong record, because every number in it is zero
    /// and zero prints the same in every unit. This is the report an operator
    /// reads the antennas' pull-in off — the number the design leaves ungated
    /// precisely because it is watched.
    #[test]
    fn the_report_carries_the_numbers_the_operator_judges_by() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        // Run 1's own readings: the two antennas rest on opposite sides of the
        // wrap, both past the command bound.
        machine.set(17, reg_for(RegId::PresentPosition), &38i32.to_le_bytes());
        machine.set(18, reg_for(RegId::PresentPosition), &4051i32.to_le_bytes());
        let (outcome, _, _) = arm(&cfg, machine);
        let summary = outcome.expect("a parked antenna is not a refusal");
        let printed = arm_report(&summary);

        // The pulls, in degrees, right antenna first — the order the goals go
        // out in, and the order the two servos sit in on the bus.
        assert!(
            printed.contains("antennas [1.908, 1.293] deg, ungated"),
            "the antenna pull-in reads right then left, in degrees:\n{printed}"
        );
        // Two records, not one printed twice: the antennas were found outside
        // the bound and left on it. The head line is the same on both, because
        // no leg was pulled — the antennas are the joints that moved.
        let lines: Vec<&str> = printed.lines().collect();
        let found = lines
            .iter()
            .position(|line| line.starts_with("found"))
            .expect("a found record");
        let armed = lines
            .iter()
            .position(|line| line.starts_with("armed"))
            .expect("an armed record");
        assert_ne!(
            lines[found + 1],
            lines[armed + 1],
            "the two records are different poses:\n{printed}"
        );
        assert!(
            lines[found + 1].contains("antennas [-176.660, 176.045] deg"),
            "the found record is the measurement:\n{printed}"
        );
        assert!(
            lines[armed + 1].contains("antennas [-174.752, 174.752] deg"),
            "the armed record is where the pins put them:\n{printed}"
        );
        assert!(
            printed.contains(&format!(
                "clearance {:.3} mm",
                summary.armed.min_margin * 1000.0
            )),
            "the armed clearance is printed in millimetres:\n{printed}"
        );
        assert!(
            printed.contains(&format!(
                "head {:.4} m",
                summary.armed.head_pose_body.translation.z
            )),
            "the armed height is printed in metres:\n{printed}"
        );
        // The legs were resting inside their windows, so their pull is zero and
        // the antennas' is not — which is what says the two are separate lines.
        assert!(
            printed.contains("legs [0.000, 0.000, 0.000, 0.000, 0.000, 0.000] deg, worst 0.000"),
            "{printed}"
        );
    }

    /// A sequencer that does one thing, for the two driver-level failures no
    /// real sequence can produce.
    enum Toy {
        /// Asks for a servo outside the roster, once.
        Addressing(u8),
        /// Waits, forever.
        Waiting,
    }

    impl Sequencer for Toy {
        type Summary = ();

        fn next(&mut self, now: Duration, _prior: Option<&BusResult>) -> SeqAction<()> {
            match self {
                Self::Addressing(id) => SeqAction::Transact(BusRequest::Ping { id: *id }),
                Self::Waiting => SeqAction::Wait {
                    until: now + Duration::from_millis(1),
                },
            }
        }

        fn step(&self) -> SeqStep {
            SeqStep::Presence
        }
    }

    /// A scripted sequence that pings, waits, pings again and finishes,
    /// recording whether it was handed a prior result at each step.
    #[derive(Default)]
    struct Recorder {
        handed: Vec<bool>,
    }

    impl Sequencer for Recorder {
        type Summary = ();

        fn next(&mut self, now: Duration, prior: Option<&BusResult>) -> SeqAction<()> {
            self.handed.push(prior.is_some());
            match self.handed.len() {
                1 | 3 => SeqAction::Transact(BusRequest::Ping { id: 10 }),
                2 => SeqAction::Wait {
                    until: now + Duration::from_millis(1),
                },
                _ => SeqAction::Done(()),
            }
        }

        fn step(&self) -> SeqStep {
            SeqStep::Presence
        }
    }

    /// An armed machine, still attached to the port that armed it.
    ///
    /// Every move test starts here rather than from a hand-built state: a
    /// `MotionState` is constructible only from an arm record, and the goals
    /// the pump believes are in the servos have to be the ones arming actually
    /// left there.
    struct Bench {
        bus: Bus<Spy>,
        state: MotionState,
        held: JointVector,
        log: Rc<RefCell<Vec<(u8, u8)>>>,
        grouped: Rc<RefCell<Vec<u16>>>,
        addressed: Rc<RefCell<Vec<Vec<u8>>>>,
        registers: Rc<RefCell<FakeMachine>>,
    }

    /// Arm `machine` and hand back everything a move needs.
    fn armed(cfg: &Resolved, machine: FakeMachine) -> Bench {
        let spy = Spy::new(machine);
        let log = spy.log();
        let grouped = spy.grouped();
        let addressed = spy.addressed();
        let registers = spy.machine();
        let mut bus = Bus::new(spy, cfg.timing);
        let mut seq =
            ArmSequencer::new(&cfg.arm, &cfg.motion.geom, &cfg.motion.env, &cfg.motion.fk);
        let mut clock = TestClock::default();
        let summary = drive(
            &mut bus,
            &cfg.map,
            &mut seq,
            &mut clock,
            action_budget(&cfg.arm),
            &mut |_| {},
        )
        .expect("the fixture machine arms");

        // The arm sequence's own traffic is not this half's subject.
        log.borrow_mut().clear();
        grouped.borrow_mut().clear();
        addressed.borrow_mut().clear();
        Bench {
            bus,
            state: MotionState::new_armed(&summary.armed),
            held: summary.armed.joints,
            log,
            grouped,
            addressed,
            registers,
        }
    }

    /// The servos of one joint group, in bus order.
    fn servos_of(cfg: &Resolved, group: JointGroup) -> Vec<u8> {
        JointId::ALL
            .into_iter()
            .enumerate()
            .filter(|(_, joint)| joint.group() == group)
            .map(|(row, _)| cfg.map.ids()[row])
            .collect()
    }

    /// The goal register of every servo, against the angle `goal` holds for the
    /// joint that servo carries.
    fn goals_match(cfg: &Resolved, bench: &Bench, goal: &JointVector) {
        let machine = bench.registers.borrow();
        for (row, id) in cfg.map.ids().iter().enumerate() {
            let joint = JointId::ALL[row];
            let held = machine
                .get(*id, reg_for(RegId::GoalPosition))
                .expect("every servo has a goal");
            let counts = i32::from_le_bytes(held.try_into().expect("a goal is four bytes"));
            let expected = cfg
                .map
                .goal_counts(row, goal.get(joint).expect("nine joints"))
                .expect("a commanded angle places");
            assert_eq!(counts, expected, "servo {id}");
        }
    }

    /// A pump at the configured rates.
    fn pump<'a>(cfg: &'a Resolved, held: JointVector) -> MotionPump<'a> {
        MotionPump::new(&cfg.motion, &cfg.map, cfg.tick_hz, cfg.health_poll_hz, held)
            .expect("the configured rates are positive")
    }

    /// The move every `up` command makes: stow to neutral, over the configured
    /// duration.
    fn to_neutral(cfg: &Resolved) -> MotionCommand {
        MotionCommand::MoveTo {
            target: JointTargets::default(),
            duration: cfg.up_duration,
            warp: Warp::MinJerk,
        }
    }

    /// Run `command` on `bench`, collecting the events it announced.
    fn run(
        bench: &mut Bench,
        pump: &mut MotionPump<'_>,
        command: MotionCommand,
        clock: &mut dyn Clock,
    ) -> (Result<MoveSummary, PumpError>, Vec<TickEvent>) {
        let mut events = Vec::new();
        let outcome = pump.run(
            &mut bench.bus,
            &mut bench.state,
            command,
            clock,
            &mut |event| events.push(event),
        );
        (outcome, events)
    }

    /// Hold `bench` for `dwell`, collecting the events it announced.
    fn hold(
        bench: &mut Bench,
        pump: &mut MotionPump<'_>,
        dwell: Duration,
        clock: &mut dyn Clock,
    ) -> (Result<MoveSummary, PumpError>, Vec<TickEvent>) {
        let mut events = Vec::new();
        let outcome = pump.hold(
            &mut bench.bus,
            &mut bench.state,
            dwell,
            clock,
            &mut |event| events.push(event),
        );
        (outcome, events)
    }

    /// How many grouped frames of `instruction` crossed the wire.
    fn frames(log: &Rc<RefCell<Vec<(u8, u8)>>>, instruction: u8) -> usize {
        log.borrow()
            .iter()
            .filter(|(_, sent)| *sent == instruction)
            .count()
    }

    /// A move runs at the configured rate, ends holding on the target, and
    /// leaves the target's own angles in the servos' goal registers.
    #[test]
    fn a_move_carries_the_head_to_its_target() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let summary = outcome.expect("a machine tracking its goals reaches the target");

        assert!(matches!(bench.state.mode(), Mode::Holding));
        // One period per tick of the configured rate for the whole duration,
        // plus the accepting period, which samples the move's own start and
        // commands nothing.
        let periods = cfg.up_duration.as_secs() * u64::from(cfg.tick_hz);
        assert_eq!(summary.ticks, periods + 1, "{summary:?}");
        assert_eq!(summary.misses, 0);
        assert_eq!(summary.overruns, 0);
        assert!(summary.goals > periods - 5, "{summary:?}");
        assert!(events.contains(&TickEvent::Command(CommandDisposition::Started)));
        assert!(events.contains(&TickEvent::Completed));
        assert!(
            !events.iter().any(|e| matches!(e, TickEvent::Faulted(_))),
            "{events:?}"
        );

        // The goals sitting in the servos are the tick's last ones, which are
        // the target's own angles.
        let last = *bench.state.last_goal();
        goals_match(&cfg, &bench, &last);
    }

    /// A move that leaves body yaw and the antennas where they are writes
    /// neither: one grouped frame per emitting period, not three, and every
    /// frame carries the six legs.
    #[test]
    fn only_the_groups_that_changed_are_written() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, _) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let summary = outcome.expect("the move completes");

        assert_eq!(summary.frames, summary.goals, "{summary:?}");
        assert_eq!(
            frames(&bench.log, INST_SYNC_WRITE),
            usize::try_from(summary.frames).expect("a frame count fits"),
        );
        // The head moved; nothing else was commanded, so nothing else was sent.
        assert_eq!(bench.state.last_goal().body_yaw, bench.held.body_yaw);
        assert_eq!(bench.state.last_goal().antennas, bench.held.antennas);
        let legs = servos_of(&cfg, JointGroup::Legs);
        for addressed in bench.addressed.borrow().iter() {
            assert_eq!(*addressed, legs, "a leg move addresses the legs");
        }
    }

    /// A move that changes all three groups writes all three, each to its own
    /// servos.
    ///
    /// The head-only move above cannot see this: the yaw and antenna branches
    /// of the group walk never run, and the goal registers of those servos hold
    /// what arming wrote, which is also what the tick's last goal says for a
    /// joint nobody moved.
    #[test]
    fn a_move_that_changes_every_group_writes_each_to_its_own_servos() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, _) = run(
            &mut bench,
            &mut pump,
            MotionCommand::MoveTo {
                target: JointTargets {
                    body_yaw: 0.2,
                    antennas: [0.3, -0.3],
                    ..JointTargets::default()
                },
                duration: cfg.up_duration,
                warp: Warp::MinJerk,
            },
            &mut clock,
        );
        let summary = outcome.expect("every group of that target is inside the envelope");

        assert_eq!(summary.frames, 3 * summary.goals, "{summary:?}");
        let expected = [
            servos_of(&cfg, JointGroup::BodyYaw),
            servos_of(&cfg, JointGroup::Legs),
            servos_of(&cfg, JointGroup::Antennas),
        ];
        for period in bench.addressed.borrow().chunks(3) {
            assert_eq!(period, expected, "each period writes the three groups");
        }
        // And the angles landed on the servos that carry those joints, not
        // merely on some servo.
        let last = *bench.state.last_goal();
        assert!(
            last.body_yaw.abs() > 0.1,
            "the yaw actually moved: {last:?}"
        );
        assert!(last.antennas[0] > 0.1, "the antennas actually moved");
        goals_match(&cfg, &bench, &last);
    }

    /// A move that changes only the antennas leaves the legs' goal registers
    /// holding the pins arming left in them.
    #[test]
    fn a_move_that_changes_only_the_antennas_leaves_the_legs_where_they_were() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let pinned = bench.held;
        let mut pump = pump(&cfg, pinned);
        let mut clock = TestClock::default();

        let mut target = *bench.state.last_targets();
        target.antennas = [0.3, -0.3];
        let (outcome, _) = run(
            &mut bench,
            &mut pump,
            MotionCommand::MoveTo {
                target,
                duration: cfg.up_duration,
                warp: Warp::MinJerk,
            },
            &mut clock,
        );
        let summary = outcome.expect("moving the antennas alone is inside the envelope");

        let antennas = servos_of(&cfg, JointGroup::Antennas);
        let legs = servos_of(&cfg, JointGroup::Legs);
        let sent = bench.addressed.borrow();
        // The legs are addressed exactly once, on the first emitting period:
        // the goals arming pinned and the IK of the pose they hold differ by
        // about 1e-14 rad, which is a change. It is not a *count*, so the
        // registers below do not move.
        assert_eq!(sent.iter().filter(|ids| **ids == legs).count(), 1);
        assert_eq!(
            sent.iter().filter(|ids| **ids == antennas).count() as u64,
            summary.goals
        );
        assert_eq!(sent.len() as u64, summary.frames);
        drop(sent);

        // Body yaw was never addressed at all, and every leg still holds the
        // count arming put there.
        assert!(
            !bench
                .addressed
                .borrow()
                .iter()
                .any(|ids| *ids == servos_of(&cfg, JointGroup::BodyYaw))
        );
        let machine = bench.registers.borrow();
        for row in 1..=6 {
            let id = cfg.map.ids()[row];
            let held = machine
                .get(id, reg_for(RegId::GoalPosition))
                .expect("every servo has a goal");
            let counts = i32::from_le_bytes(held.try_into().expect("a goal is four bytes"));
            let pin = cfg
                .map
                .goal_counts(row, pinned.get(JointId::ALL[row]).expect("nine joints"))
                .expect("a pinned angle places");
            assert_eq!(counts, pin, "servo {id} still holds its pin");
        }
    }

    /// A hold is one period: the machine is already holding, so there is
    /// nothing to carry and nothing to write.
    #[test]
    fn a_hold_returns_after_one_period() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut pump, MotionCommand::Hold, &mut clock);
        let summary = outcome.expect("holding an armed machine is always available");

        assert_eq!(summary.ticks, 1);
        assert_eq!(summary.goals, 0);
        assert_eq!(summary.frames, 0);
        assert_eq!(frames(&bench.log, INST_SYNC_WRITE), 0);
        assert!(events.contains(&TickEvent::Command(CommandDisposition::Held)));
    }

    /// A timed hold runs one period per tick of its dwell, and still commands
    /// nothing: the goals are already where they belong.
    #[test]
    fn a_timed_hold_measures_every_period_of_its_dwell() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();
        let dwell = pump.period() * 10;

        let (outcome, events) = hold(&mut bench, &mut pump, dwell, &mut clock);
        let summary = outcome.expect("holding an armed machine is always available");

        // One period per tick of the dwell, plus the period the deadline is
        // noticed in.
        assert_eq!(summary.ticks, 11, "{summary:?}");
        assert_eq!(summary.elapsed, dwell, "{summary:?}");
        assert_eq!(summary.goals, 0);
        assert_eq!(frames(&bench.log, INST_SYNC_WRITE), 0);
        assert!(events.contains(&TickEvent::Command(CommandDisposition::Held)));
    }

    /// A hold on a host that wakes late says so.
    ///
    /// This is the command's whole purpose — a scheduler stall during a
    /// supervised hold is exactly the incident the telemetry exists for — and a
    /// hold paced outside the loop would report a clean run however badly the
    /// host kept time, because each inner run would take its own epoch afresh.
    #[test]
    fn a_timed_hold_counts_the_periods_it_began_late() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let late = pump.period();
        let dwell = pump.period() * 10;
        let mut clock = LateClock {
            now: Duration::ZERO,
            late,
        };

        let (outcome, events) = hold(&mut bench, &mut pump, dwell, &mut clock);
        let summary = outcome.expect("a late loop still holds");

        assert_eq!(summary.overruns, summary.ticks - 1, "{summary:?}");
        assert_eq!(summary.worst_jitter, late, "{summary:?}");
        assert_eq!(
            events
                .iter()
                .filter(|event| matches!(event, TickEvent::Overrun { .. }))
                .count(),
            1,
            "{events:?}"
        );
        // The dwell is wall time, not a period count. Each period here consumes
        // two periods of it, so a ten-period dwell is spanned by ten periods
        // rather than the eleven a punctual host takes — and it is spanned
        // exactly, which a loop that counted its own periods would not do.
        assert_eq!(summary.ticks, 10, "{summary:?}");
        assert_eq!(summary.elapsed, dwell, "{summary:?}");
    }

    /// A hold's stall budget follows the dwell it was asked for.
    ///
    /// The dwell is an operator-facing knob with no ceiling, and the obvious
    /// thing to do at the bench is turn it up. A budget that did not follow it
    /// would call a perfectly healthy machine stuck partway through the one
    /// command whose output exists to be believed about timing.
    #[test]
    fn a_hold_budget_follows_the_dwell_it_was_asked_for() {
        let cfg = resolved();
        // Longer than the fixed margin, so the margin alone cannot cover it.
        let dwell = Duration::from_secs(STALL_MARGIN_SECS + 5);
        let periods = dwell.as_secs() * u64::from(cfg.tick_hz);

        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut punctual = pump(&cfg, bench.held);
        let mut clock = TestClock::default();
        let (outcome, _) = hold(&mut bench, &mut punctual, dwell, &mut clock);
        let summary = outcome.expect("a long dwell is a legal dwell");
        assert_eq!(summary.ticks, periods + 1, "{summary:?}");
        assert_eq!(summary.elapsed, dwell, "{summary:?}");

        // And a clock that stopped advancing still ends, on a budget that is
        // the dwell plus the margin rather than the margin alone.
        let mut stuck_bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut stuck_pump = pump(&cfg, stuck_bench.held);
        let mut clock = FrozenClock;
        let (outcome, _) = hold(&mut stuck_bench, &mut stuck_pump, dwell, &mut clock);
        let error = outcome.expect_err("a hold on a stopped clock never reaches its deadline");
        let budget = periods + u64::from(cfg.tick_hz) * STALL_MARGIN_SECS;
        assert!(
            matches!(error, PumpError::Stalled { budget: ran } if ran == budget),
            "{error}"
        );
    }

    /// A target the envelope refuses stops the run without commanding
    /// anything, and leaves the machine armed and holding rather than faulted.
    #[test]
    fn a_refused_command_stops_the_run_and_commands_nothing() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let refused = run(
            &mut bench,
            &mut pump,
            MotionCommand::MoveTo {
                // Well past the antennas' command bound.
                target: JointTargets {
                    antennas: [4.0, 0.0],
                    ..JointTargets::default()
                },
                duration: cfg.up_duration,
                warp: Warp::MinJerk,
            },
            &mut clock,
        );

        let error = refused.0.expect_err("that target is outside the envelope");
        assert!(
            matches!(error, PumpError::Rejected(CommandRejection::Envelope(_))),
            "{error}"
        );
        assert!(matches!(bench.state.mode(), Mode::Holding));
        assert!(!bench.state.is_faulted());
        assert_eq!(frames(&bench.log, INST_SYNC_WRITE), 0);
    }

    /// One servo short of an answer makes the whole period blind, and the
    /// tick's read-loss budget — not a retry here — is what ends the run.
    #[test]
    fn one_silent_servo_makes_every_period_blind() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        bench.registers.borrow_mut().silent.push(13);
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("a machine nobody can read does not finish a move");

        let budget = cfg.motion.read_loss_ticks;
        assert!(
            matches!(error, PumpError::Fault(Fault::ReadLoss { misses }) if misses == budget + 1),
            "{error}"
        );
        assert!(
            events.contains(&TickEvent::Command(CommandDisposition::Started)),
            "{events:?}"
        );
        // The first blind period names the servo that made it blind, which is
        // the whole diagnosis of a flaky leg on a bus of nine.
        let lost = events
            .iter()
            .find_map(|event| match event {
                TickEvent::ReadLost { failed } => Some(failed),
                _ => None,
            })
            .expect("a blind period is announced");
        assert_eq!(lost.servos(), &[(13, IdOutcome::Timeout)], "{lost}");
        assert_eq!(lost.corrupt_frames(), 0);
        assert!(lost.to_string().contains("servo 13 silent"), "{lost}");
        // The same servo takes the health sweep down with it, and that is
        // reported too rather than passing for a period the poll was not due.
        let health_lost = events
            .iter()
            .find_map(|event| match event {
                TickEvent::HealthLost { failed } => Some(failed),
                _ => None,
            })
            .expect("a health sweep that fell short is announced");
        assert_eq!(health_lost.servos(), &[(13, IdOutcome::Timeout)]);
        // Every period of the run was blind, the accepting one included.
        let periods = u64::from(budget) + 1;
        let polls = periods.div_ceil(u64::from(cfg.tick_hz / cfg.health_poll_hz));
        assert_eq!(
            frames(&bench.log, INST_SYNC_READ) as u64,
            periods + polls,
            "one position read per period, and the health sweeps beside them",
        );
    }

    /// A goal no count can hold is refused where the angle becomes counts, and
    /// the refusal names the servo the goal was for — never a wrapped count,
    /// never a saturated one.
    ///
    /// One row per group, because the row-to-servo lookup is what a wrong
    /// entry in the group walk would get wrong: an antenna goal named against
    /// a leg servo is a bring-up reading the wrong message.
    #[test]
    fn a_goal_the_map_will_not_carry_names_the_servo_it_was_for() {
        let cfg = resolved();
        let bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let held = bench.held;
        let mut bus = bench.bus;
        let mut pump = pump(&cfg, held);

        for row in [0, 3, 8] {
            let mut goal = held;
            goal.set(JointId::ALL[row], f64::NAN);
            let refused = pump
                .write_goals(&mut bus, &goal)
                .expect_err("no count holds that angle");
            let PumpError::Map { id, reg, source } = refused else {
                panic!("expected a refused conversion, got {refused}");
            };
            assert_eq!(id, cfg.map.ids()[row], "row {row}");
            assert_eq!(reg, RegId::GoalPosition);
            assert!(matches!(source, MapError::Angle { .. }), "{source}");
        }

        // The read side has the same arm and cannot reach it: a measurement's
        // conversion refuses only a joint outside the nine, and the rows this
        // loop walks are always the nine. So a count nobody can convert is not
        // a failure mode there, and no period can be spent blind for one.
        for row in 0..JointId::COUNT {
            for counts in [i32::MIN, 0, i32::MAX] {
                assert!(cfg.map.present_rad(row, counts).is_ok(), "row {row}");
            }
        }
    }

    /// A dropout shorter than the budget is what the budget exists for: the
    /// move carries on blind and finishes, and both ends of the outage are
    /// announced.
    #[test]
    fn a_dropout_inside_the_budget_is_survived_and_reported() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let outage = 4;
        assert!(
            outage < cfg.motion.read_loss_ticks,
            "well inside the budget"
        );
        bench
            .registers
            .borrow_mut()
            .mute
            .insert((13, reg_for(RegId::PresentPosition).addr), outage);
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let summary = outcome.expect("a move survives a dropout inside the budget");

        assert_eq!(summary.misses, u64::from(outage));
        assert!(matches!(bench.state.mode(), Mode::Holding));
        assert!(events.contains(&TickEvent::Completed), "{events:?}");

        // Lost, then back, in that order, with the outage's own length.
        let outage_events: Vec<&TickEvent> = events
            .iter()
            .filter(|event| {
                matches!(
                    event,
                    TickEvent::ReadLost { .. } | TickEvent::ReadRestored { .. }
                )
            })
            .collect();
        assert!(
            matches!(outage_events[0], TickEvent::ReadLost { failed }
                if failed.servos() == [(13, IdOutcome::Timeout)]),
            "{outage_events:?}"
        );
        assert_eq!(
            *outage_events[1],
            TickEvent::ReadRestored { after: outage },
            "{outage_events:?}"
        );
        assert_eq!(
            outage_events.len(),
            2,
            "one outage, announced once each way"
        );
    }

    /// The budget is counted in consecutive periods: one period past it faults,
    /// even though the servo answers again straight afterwards.
    #[test]
    fn a_dropout_one_period_past_the_budget_faults() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let outage = cfg.motion.read_loss_ticks + 1;
        bench
            .registers
            .borrow_mut()
            .mute
            .insert((13, reg_for(RegId::PresentPosition).addr), outage);
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, _) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("one period past the budget is a fault");

        assert!(
            matches!(error, PumpError::Fault(Fault::ReadLoss { misses }) if misses == outage),
            "{error}"
        );
    }

    /// A servo that answers its position cleanly and refuses the health
    /// register takes the health sweep down with it — and says so, every time
    /// the run of failed sweeps begins and ends.
    #[test]
    fn a_health_sweep_that_falls_short_is_reported_rather_than_absent() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        bench
            .registers
            .borrow_mut()
            .errors
            .insert((14, reg_for(RegId::HardwareErrorStatus).addr), 0x07);
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let summary = outcome.expect("a health register nobody can read stops no move");

        // The move ran to its target with no health verdict at all. That it
        // does not fault is TODO(health-read-budget); that it does not pass
        // unnoticed is this test.
        let every_sweep = summary
            .ticks
            .div_ceil(u64::from(cfg.tick_hz / cfg.health_poll_hz));
        assert_eq!(summary.health_misses, every_sweep, "{summary:?}");
        assert_eq!(
            summary.misses, 0,
            "the position read is a different register"
        );
        assert!(
            !events
                .iter()
                .any(|event| matches!(event, TickEvent::Health(_))),
            "no sweep completed: {events:?}"
        );

        let lost: Vec<&ReadFailures> = events
            .iter()
            .filter_map(|event| match event {
                TickEvent::HealthLost { failed } => Some(failed),
                _ => None,
            })
            .collect();
        assert_eq!(lost.len(), 1, "announced once, not once a second");
        assert_eq!(
            lost[0].servos(),
            &[(14, IdOutcome::ServoError(dxl_proto::StatusError(0x07)))]
        );
        assert!(
            lost[0].to_string().contains("servo 14 refused with 0x07"),
            "{}",
            lost[0]
        );
    }

    /// A health sweep that comes back after a run of failed ones says so, with
    /// the length of the run.
    #[test]
    fn a_health_sweep_that_comes_back_says_how_long_it_was_gone() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let sweeps = 2;
        bench
            .registers
            .borrow_mut()
            .mute
            .insert((14, reg_for(RegId::HardwareErrorStatus).addr), sweeps);
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let summary = outcome.expect("the move finishes");

        assert_eq!(summary.health_misses, u64::from(sweeps));
        assert!(
            events
                .iter()
                .any(|event| matches!(event, TickEvent::HealthLost { .. })),
            "{events:?}"
        );
        assert!(
            events.contains(&TickEvent::HealthRestored { after: sweeps }),
            "{events:?}"
        );
    }

    /// The health sweep runs at its own rate, and a standing latch is reported
    /// once rather than once a second forever.
    #[test]
    fn the_health_sweep_runs_at_its_own_rate_and_reports_on_change() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        for id in cfg.map.ids() {
            bench.registers.borrow_mut().set(
                id,
                reg_for(RegId::HardwareErrorStatus),
                &[ServoHealth::INPUT_VOLTAGE],
            );
        }
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let summary = outcome.expect("the input-voltage bit raises no fault");

        let health: Vec<&TickEvent> = events
            .iter()
            .filter(|event| matches!(event, TickEvent::Health(_)))
            .collect();
        assert_eq!(health.len(), 1, "{events:?}");
        let TickEvent::Health(servos) = health[0] else {
            unreachable!("filtered")
        };
        assert!(
            servos.iter().all(|servo| servo.voltage_only()),
            "{servos:?}"
        );

        // One position read per period; one health sweep every tick_hz /
        // health_poll_hz of them, the first period included.
        let every = u64::from(cfg.tick_hz / cfg.health_poll_hz);
        let polls = summary.ticks.div_ceil(every);
        let grouped = bench.grouped.borrow();
        let position = reg_for(RegId::PresentPosition).addr;
        let hardware = reg_for(RegId::HardwareErrorStatus).addr;
        assert_eq!(
            grouped.iter().filter(|addr| **addr == position).count() as u64,
            summary.ticks
        );
        assert_eq!(
            grouped.iter().filter(|addr| **addr == hardware).count() as u64,
            polls
        );
    }

    /// A hardware error beyond the input-voltage bit stops the loop, and
    /// nothing here reboots or releases anything.
    #[test]
    fn a_hardware_error_stops_the_loop_with_torque_held() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        bench
            .registers
            .borrow_mut()
            .set(14, reg_for(RegId::HardwareErrorStatus), &[0x20]);
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("an overloaded servo stops the move");

        assert!(
            matches!(
                error,
                PumpError::Fault(Fault::HardwareError { id: 14, bits: 0x20 })
            ),
            "{error}"
        );
        assert!(
            events
                .iter()
                .any(|event| matches!(event, TickEvent::Faulted(_))),
            "{events:?}"
        );
        assert!(bench.state.is_faulted());
        // Torque is where arming left it: the loop wrote no torque register.
        let machine = bench.registers.borrow();
        for id in cfg.map.ids() {
            assert_eq!(
                machine.get(id, reg_for(RegId::TorqueEnable)),
                Some(&[1u8][..]),
                "servo {id}"
            );
        }
    }

    /// A clock that stopped advancing is a bench session that would otherwise
    /// hang with torque on. Every period samples the move's own start, so the
    /// move never finishes and the budget is the only way out.
    #[test]
    fn a_clock_that_never_advances_is_bounded() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = FrozenClock;

        let (outcome, _) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("a move on a stopped clock never finishes");

        // The budget is the move's own travel plus the margin, and the move
        // never advances a period's worth of it.
        let budget = u64::from(cfg.tick_hz) * (cfg.up_duration.as_secs() + STALL_MARGIN_SECS);
        assert_eq!(pump.stall_budget(&to_neutral(&cfg)), budget);
        assert!(
            matches!(error, PumpError::Stalled { budget: ran } if ran == budget),
            "{error}"
        );
        assert_eq!(frames(&bench.log, INST_SYNC_WRITE), 0, "nothing commanded");
    }

    /// A move slower than any fixed watchdog would allow runs to its target.
    ///
    /// Move durations are configuration with no ceiling, and the example file
    /// itself argues for a slow lift. A budget that did not follow the
    /// command's own duration would abort this one mid-travel, torque on, and
    /// report it as a loop that hung.
    #[test]
    fn a_move_longer_than_a_minute_still_finishes() {
        let mut cfg = resolved();
        // Ten hertz keeps the period count of a slow move testable; the
        // duration is what this is about.
        cfg.tick_hz = 10;
        cfg.up_duration = Duration::from_secs(90);
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let periods = cfg.up_duration.as_secs() * u64::from(cfg.tick_hz);
        assert!(
            periods > u64::from(cfg.tick_hz) * 60,
            "longer than a minute of periods"
        );

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let summary = outcome.expect("a slow move is a legal move");

        assert_eq!(summary.ticks, periods + 1, "{summary:?}");
        assert!(matches!(bench.state.mode(), Mode::Holding));
        assert!(events.contains(&TickEvent::Completed));
        assert_eq!(
            pump.stall_budget(&to_neutral(&cfg)),
            periods + u64::from(cfg.tick_hz) * STALL_MARGIN_SECS
        );
    }

    /// A period that runs late proceeds immediately rather than skipping the
    /// next one, and says so once.
    #[test]
    fn a_late_period_is_recorded_and_the_move_still_finishes() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let late = pump.period();
        let mut clock = LateClock {
            now: Duration::ZERO,
            late,
        };

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let summary = outcome.expect("a late loop still reaches the target");

        assert_eq!(summary.overruns, summary.ticks - 1, "{summary:?}");
        assert_eq!(
            events
                .iter()
                .filter(|event| matches!(event, TickEvent::Overrun { .. }))
                .count(),
            1,
            "{events:?}"
        );
        assert!(events.contains(&TickEvent::Completed));
        assert_eq!(summary.worst_jitter, late, "{summary:?}");
        // Every period still took its own reading: a late loop never skips one.
        assert_eq!(
            bench
                .grouped
                .borrow()
                .iter()
                .filter(|addr| **addr == reg_for(RegId::PresentPosition).addr)
                .count() as u64,
            summary.ticks
        );
    }

    /// Ordinary wake latency is not an overrun.
    #[test]
    fn wake_latency_under_the_tolerance_is_jitter_and_not_an_overrun() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let late = pump.period() / 10;
        let mut clock = LateClock {
            now: Duration::ZERO,
            late,
        };

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let summary = outcome.expect("a jittery loop still reaches the target");

        assert_eq!(summary.overruns, 0, "{summary:?}");
        assert_eq!(summary.worst_jitter, late, "{summary:?}");
        assert!(
            !events
                .iter()
                .any(|event| matches!(event, TickEvent::Overrun { .. })),
            "{events:?}"
        );
        assert!(events.contains(&TickEvent::Completed));
    }

    /// A health poll configured at or above the tick rate sweeps every period.
    ///
    /// The poll interval is whole periods, and a rate at or above the tick rate
    /// rounds to none of them — so the floor at one period is what stands
    /// between a fast poll and a loop dividing by it.
    #[test]
    fn a_health_poll_at_the_tick_rate_sweeps_every_period() {
        let mut cfg = resolved();
        cfg.health_poll_hz = cfg.tick_hz * 2;
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, _) = hold(
            &mut bench,
            &mut pump,
            Duration::from_millis(200),
            &mut clock,
        );
        let summary = outcome.expect("a hold runs at whatever poll rate it was given");

        let hardware = reg_for(RegId::HardwareErrorStatus).addr;
        assert!(summary.ticks > 1, "{summary:?}");
        assert_eq!(
            bench
                .grouped
                .borrow()
                .iter()
                .filter(|addr| **addr == hardware)
                .count() as u64,
            summary.ticks
        );
        assert_eq!(summary.health_misses, 0);
    }

    /// The jitter a run reports is the worst period of the whole run, not the
    /// last one.
    ///
    /// It is the number that says whether the overrun tolerance is the right
    /// size on a given host, and a spike that a later on-time period erased
    /// would say the opposite of what happened.
    #[test]
    fn the_reported_jitter_is_the_worst_period_not_the_last() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        // Three quarters of a period: over the overrun tolerance, and under a
        // whole period, so the loop is back on schedule for the next one.
        let spike = pump.period() * 3 / 4;
        let mut clock = SpikeClock {
            now: Duration::ZERO,
            spike,
            at: 2,
            woken: 0,
        };

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let summary = outcome.expect("one late period does not stop a move");

        assert_eq!(summary.worst_jitter, spike, "{summary:?}");
        assert_eq!(summary.overruns, 1, "{summary:?}");
        assert!(summary.ticks > 10, "later periods ran on time: {summary:?}");
        assert!(
            events.contains(&TickEvent::Overrun {
                tick: 2,
                late: spike
            }),
            "{events:?}"
        );
    }

    /// The tick is fed what the servos report, not what they were last told.
    ///
    /// The tracking monitor is the only thing that can tell a machine holding
    /// its goals from one being dragged off them, and it can only do that if
    /// the measurement is a measurement. A loop that handed the tick its own
    /// last goals would report perfect tracking on a machine that never moved.
    #[test]
    fn a_machine_that_never_reaches_its_goals_faults_on_tracking() {
        let mut cfg = resolved();
        cfg.motion.tracking.threshold_rad = 0.02;
        cfg.motion.tracking.ticks = 5;
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        // Thirty counts short of whatever it is told, on every servo: about
        // 0.046 rad, and a standing offset rather than a lag that closes.
        for id in cfg.map.ids() {
            bench.registers.borrow_mut().lag.insert(id, 30);
        }
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("a machine that never arrives is not tracking");

        assert!(
            matches!(error, PumpError::Fault(Fault::TrackingLost { .. })),
            "{error}"
        );
        assert!(
            events
                .iter()
                .any(|event| matches!(event, TickEvent::Faulted(Fault::TrackingLost { .. }))),
            "{events:?}"
        );
        // Stopped, not released: the head is still held where it got to.
        let machine = bench.registers.borrow();
        for id in cfg.map.ids() {
            assert_eq!(
                machine.get(id, reg_for(RegId::TorqueEnable)),
                Some(&[1u8][..]),
                "servo {id}"
            );
        }
    }

    /// The port failing mid-move is not the machine's answer, and it is not a
    /// missed read either.
    #[test]
    fn a_port_failure_mid_move_is_not_a_missed_read() {
        let cfg = resolved();
        let bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut state = bench.state;
        let mut bus = Bus::new(BrokenPort, cfg.timing);
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let error = pump
            .run(
                &mut bus,
                &mut state,
                to_neutral(&cfg),
                &mut clock,
                &mut |_| {},
            )
            .expect_err("a dead port carries no move");

        let PumpError::Bus { id, source } = error else {
            panic!("expected a bus failure, got {error}");
        };
        assert_eq!(id, dxl_proto::frame::BROADCAST_ID, "a grouped request");
        assert!(matches!(source, XactError::Io { .. }), "{source}");
        assert!(!state.is_faulted(), "the tick never saw a period");
    }

    /// A rate of zero is refused rather than dividing by it. Configuration
    /// refuses both of these, so this is the guard on a pump built by hand.
    #[test]
    fn a_control_rate_of_zero_is_refused() {
        let cfg = resolved();
        for (tick_hz, health_poll_hz) in [(0, 1), (50, 0), (0, 0)] {
            let refused = MotionPump::new(
                &cfg.motion,
                &cfg.map,
                tick_hz,
                health_poll_hz,
                JointVector::default(),
            )
            .err()
            .expect("a zero rate is refused");
            assert!(
                matches!(refused, PumpError::Rate { .. }),
                "{tick_hz}/{health_poll_hz}: {refused}"
            );
        }
    }

    /// Every event renders as its own line, so a supervised run reads.
    #[test]
    fn every_event_says_what_happened() {
        let servos = [ServoHealth { id: 10, bits: 1 }; JointId::COUNT];
        for (event, expected) in [
            (TickEvent::Command(CommandDisposition::None), "no command"),
            (TickEvent::Command(CommandDisposition::Started), "moving"),
            (TickEvent::Command(CommandDisposition::Held), "holding"),
            (
                TickEvent::Command(CommandDisposition::Rejected(
                    CommandRejection::AlreadyMoving,
                )),
                "refused",
            ),
            (
                TickEvent::ReadLost {
                    failed: ReadFailures::default(),
                },
                "lost",
            ),
            (TickEvent::ReadRestored { after: 3 }, "3"),
            (
                TickEvent::HealthLost {
                    failed: ReadFailures::default(),
                },
                "health sweep lost",
            ),
            (TickEvent::HealthRestored { after: 2 }, "2"),
            (TickEvent::Health(servos), "0x01"),
            (
                TickEvent::Overrun {
                    tick: 7,
                    late: Duration::from_millis(4),
                },
                "7",
            ),
            (TickEvent::Completed, "target"),
            (
                TickEvent::Faulted(Fault::ReadLoss { misses: 51 }),
                "faulted",
            ),
        ] {
            let printed = event.to_string();
            assert!(
                printed.contains(expected),
                "{event:?} rendered as {printed}"
            );
        }
    }

    /// A clock that never advances, however long it is asked to wait.
    struct FrozenClock;

    impl Clock for FrozenClock {
        fn now(&self) -> Duration {
            Duration::ZERO
        }

        fn sleep_until(&mut self, _until: Duration) {}
    }

    /// A clock that wakes late once, on one nominated period, and on time for
    /// every other.
    struct SpikeClock {
        now: Duration,
        spike: Duration,
        /// Which wake runs late, counted from one.
        at: u64,
        woken: u64,
    }

    impl Clock for SpikeClock {
        fn now(&self) -> Duration {
            self.now
        }

        fn sleep_until(&mut self, until: Duration) {
            self.woken += 1;
            let late = if self.woken == self.at {
                self.spike
            } else {
                Duration::ZERO
            };
            self.now = (until + late).max(self.now);
        }
    }

    /// A clock that always wakes late, by a fixed amount.
    struct LateClock {
        now: Duration,
        late: Duration,
    }

    impl Clock for LateClock {
        fn now(&self) -> Duration {
            self.now
        }

        fn sleep_until(&mut self, until: Duration) {
            self.now = (until + self.late).max(self.now);
        }
    }
}
