//! The driver: the one place a sans-I/O sequencer meets a real port.
//!
//! Every library under this one refuses to own a loop. [`CommissionSequencer`] and
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
//! One kind of run is exempt: a torque-off walk. [`drive_release`] carries on
//! past any transaction failure, because a driver that stopped at servo three
//! would leave six servos holding the head up — and nothing, at any layer, may
//! condition writing torque off.
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
//! blind motion may continue. And an ending **stops the loop and hands the
//! reason up**: the loop's own job ends there, and the maneuver that answers it
//! — a stow under control, or torque off on the spot — belongs to whoever holds
//! the engagement, because that is the thing with a release to run.
//! [`PumpError::class`] is what that caller acts on, and it names one of the
//! doctrine's responses rather than a yes-or-no about faulting.
//!
//! A run is not a commitment to the command that started it:
//! [`MotionPump::run_retargeting`] asks its caller at every period whether the
//! move in flight is still the one wanted, and a replacement is spliced in from
//! the setpoint the last period commanded. A caller executing a timeline off a
//! wire needs that — waiting out a raise before starting the fold makes the
//! head late by a whole move — and the stall budget follows the replacement, so
//! being steered is never mistaken for hanging.
//!
//! A run does not end when the trajectory does. The last goal going out says
//! only that commanding is over; the machine is still on its way there, and on
//! a move whose commanded clock is shorter than the machine's own response that
//! gap is most of the motion. So the loop keeps turning — reading every period,
//! commanding nothing — until every joint is measured within the settle
//! tolerance of the goal it was left on, and the summary carries both instants
//! so the gap is visible. A joint that never arrives ends the run at the window
//! and is named: an outcome and not a fault, because nothing here may gate or
//! delay what a caller does about torque.
//!
//! A run can also record itself: [`MotionPump::record_trace`] keeps one
//! [`TickSample`] per period — the nine measured angles and the nine goals they
//! were measured against — which is the whole velocity profile of a move at the
//! rate it was actually sampled at. Off by default, so an ordinary run still
//! allocates nothing while it turns.
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
    ArmConfig, ArmRecord, BusRequest, BusResult, ClockStretch, CommandDisposition,
    CommandRejection, CommissionSummary, EngageSummary, Fault, JointGroup, JointId, JointSet,
    JointVector, Maneuver, Mode, MotionCommand, MotionConfig, MotionState, MoveAbort, Outcome,
    RegId, RegValue, Response, SeqAction, SeqError, SeqStep, Sequencer, ServoHealth, TickInputs,
    TickOutputs, TickReport, floor_move_clock, motion_tick,
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

/// How many periods of trace one run may record before recording stops.
///
/// A ceiling and not a target: a bench move records a few hundred samples and a
/// long hold a few thousand, while a run steered by a caller that keeps
/// retargeting has no length of its own at all. At the bench's fifty hertz this
/// is forty minutes of motion and a few tens of megabytes, which is far past
/// any run worth reading and well short of a host running out of memory.
const MAX_TRACE_SAMPLES: usize = 120_000;

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

    /// A servo did not acknowledge its torque-off write, so it may still be
    /// holding. Reported after the whole release has run — every servo is
    /// always asked, and this is what the run has to say about the ones that
    /// did not answer, not something it could have refused up front.
    #[error("servo {id} did not acknowledge torque off and may still be holding")]
    TorqueOffUnacked {
        /// The lowest servo left unacknowledged; the run's own report lists all
        /// of them.
        id: u8,
    },

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

    /// A servo about to be provisioned answered as a part this platform does
    /// not carry. Whatever holds that ID is not the servo whose non-volatile
    /// registers this project writes.
    #[error("servo {id} reports model {model}, where this platform's is {expected}")]
    WrongPart {
        /// The servo addressed.
        id: u8,
        /// The model it answered with.
        model: u16,
        /// The model this platform carries at that position.
        expected: u16,
    },

    /// A servo was holding torque where a non-volatile write requires it
    /// released. Refused across the whole roster before anything is written, so
    /// a half-provisioned machine is not a state this leaves behind.
    #[error("servo {id} is holding torque; release it with `off` before provisioning")]
    TorqueHeld {
        /// The servo addressed.
        id: u8,
    },

    /// The sequence addressed a servo the configured roster does not carry.
    /// A wiring mistake between the configuration and the sequencer, caught
    /// before a frame goes out to whatever holds that ID.
    #[error("the sequence addressed servo {id}, which is not in the configured roster")]
    UnknownServo {
        /// The ID asked for.
        id: u8,
    },

    /// A command was asked for a servo the configured roster does not carry.
    /// Whatever holds that ID is not one of this machine's nine joints, so it
    /// is refused by name rather than skipped or addressed anyway.
    #[error("servo {id} is not in the configured roster {roster:?}")]
    OffRoster {
        /// The ID asked for.
        id: u8,
        /// The servos the configuration carries, in bus order.
        roster: [u8; JointId::COUNT],
    },

    /// A rebooted servo never answered again within the budget it was given.
    /// Nothing on the reboot path holds torque, so nothing is released in
    /// response: this is a report, and the servo is either still restarting or
    /// gone.
    #[error("servo {id} answered none of {polls} pings over {waited:?} after its reboot: {source}")]
    NotBack {
        /// The servo that stayed silent.
        id: u8,
        /// Pings it was asked.
        polls: u32,
        /// How long those pings took.
        waited: Duration,
        /// What the last of them failed with.
        source: XactError,
    },

    /// A rebooted servo answered again but came back still holding torque, so
    /// it never restarted: the instruction was lost on the wire, or refused.
    /// Nothing on the reboot path enables torque, so this is torque the servo
    /// held all along and the reboot did not take — reported rather than
    /// written off, because the command's whole promise is that what it reaches
    /// lets go.
    #[error("servo {id} answered after its reboot still holding torque, so it did not restart")]
    NotRestarted {
        /// The servo that kept its torque.
        id: u8,
    },

    /// A servo neither acknowledged its reboot nor came back with torque to
    /// drop, so nothing observed says it restarted. The case is a machine
    /// already limp — which is what a latched shutdown leaves, and the state an
    /// operator reaches for `reboot` in: a servo that never took the
    /// instruction answers pings exactly as one that did, and the torque that
    /// tells them apart was already off. Reported rather than passed, because
    /// the alternative is a success line over a latch that is still set.
    #[error(
        "servo {id} did not acknowledge its reboot and came back holding nothing, so no reading \
         says it restarted: {source}"
    )]
    RestartUnconfirmed {
        /// The servo whose restart could not be established.
        id: u8,
        /// What its reboot instruction failed with.
        source: XactError,
    },

    /// The sequence neither finished nor failed within its action budget — the
    /// supply gate's configured polling, plus the fixed phases.
    #[error("the sequence took more than {budget} actions without finishing")]
    Runaway {
        /// The budget it ran past.
        budget: usize,
    },

    /// The tick stopped commanding. Motor control or position feedback is no
    /// longer trusted, so the machine goes limp and the operator's next command
    /// is the recovery.
    #[error("the tick faulted: {0}")]
    Fault(#[from] Fault),

    /// The tick abandoned the move. The plan was wrong; the machine is healthy,
    /// still holding the last goal it was written, and still commandable.
    #[error("the move was abandoned: {0}")]
    Aborted(#[from] MoveAbort),

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

    /// A settle policy no comparison can be made against: a tolerance that is
    /// not a positive number, or a window of no time at all. Configuration
    /// refuses both, so reaching this means a caller built a pump by hand.
    #[error(
        "a settle tolerance must be a positive number of radians and its window a positive \
         duration: {tolerance} rad over {timeout:?}"
    )]
    Settle {
        /// How far a joint may be from its goal and still count as arrived.
        tolerance: f64,
        /// How long arrival is waited for.
        timeout: Duration,
    },
}

/// Whether torque was on when a run stopped.
///
/// The one thing a classification cannot read off the error itself. The same
/// unanswered transaction is a request declined when nothing is energized and a
/// machine that can no longer be manoeuvred when nine servos are holding the
/// head up, and the error carries no record of which it was.
///
/// Call sites that never write torque — commissioning, the register sweeps,
/// provisioning, the reboot command — pass [`Self::PreTorque`], and ones that
/// only ever run with servos holding pass [`Self::UnderTorque`]. The engage
/// drive is the one path that crosses the line mid-run; [`engage_phase`] is
/// where it decides which side it was on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    /// Nothing is energized: the machine is limp and stays where it stands.
    PreTorque,
    /// Servos are holding, so an ending leaves something to bring back to the
    /// minimum risk condition.
    UnderTorque,
}

/// What a run's ending asks its caller to do.
///
/// The response vocabulary of the fault doctrine, less the one response that is
/// not an ending: degrading the antennas leaves the move running, and travels
/// in the tick's report rather than as an error.
///
/// Every [`PumpError`] maps onto one of these, so what a caller does about an
/// ending is decided once, here, at compile time — never by a caller reading a
/// message, and never by a default that a new variant falls into.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorClass {
    /// The ask is declined and nothing changes.
    Refuse,
    /// Abandon the move, stow every joint that still commands with the checks
    /// live, then release. The machine is trusted to command.
    SlowStowToRest,
    /// Immediate best-effort torque-off of all nine; the next wake engages
    /// normally.
    ImmediateAllTorqueOffToRest,
    /// Release the faulted servo on the spot, stow on what still commands, then
    /// release everything and wait for an operator.
    MaskedSlowStowToPark,
    /// Immediate best-effort torque-off of all nine, then wait for an operator.
    ImmediateAllTorqueOffToPark,
}

impl ErrorClass {
    /// The ending that carries out `response`.
    fn answering(response: Response) -> Self {
        match response {
            Response::Refuse => Self::Refuse,
            Response::SlowStowToRest => Self::SlowStowToRest,
            Response::ImmediateAllTorqueOffToRest => Self::ImmediateAllTorqueOffToRest,
            Response::MaskedSlowStowToPark => Self::MaskedSlowStowToPark,
            Response::ImmediateAllTorqueOffToPark => Self::ImmediateAllTorqueOffToPark,
            // Degrading a pair is not an ending: the antennas go limp and the
            // move carries on, reported through the tick. An error that
            // nonetheless names this response says the head is healthy and
            // still commanding, and a stow under control is what that gets.
            Response::DegradeAntennas => Self::SlowStowToRest,
        }
    }

    /// What this ending actually does to the machine, for the record it is
    /// reported in.
    ///
    /// Beside [`Self::disposition`] and for the same reason: which maneuver a
    /// class runs is the class's own fact, and a caller spelling one out is a
    /// second table that can come to disagree with this one. A refusal runs
    /// none — nothing is done to a machine whose ask was declined.
    #[must_use]
    pub fn maneuver(self) -> Option<Maneuver> {
        match self {
            Self::Refuse => None,
            Self::SlowStowToRest => Some(Maneuver::SlowStow),
            Self::MaskedSlowStowToPark => Some(Maneuver::MaskedSlowStow),
            Self::ImmediateAllTorqueOffToRest | Self::ImmediateAllTorqueOffToPark => {
                Some(Maneuver::ImmediateAllTorqueOff)
            }
        }
    }

    /// Whether the machine this response leaves behind may be engaged again by
    /// whatever asks next, or has to wait for a person.
    ///
    /// A refusal changed nothing, so there is nothing to wait for.
    #[must_use]
    pub fn disposition(self) -> Disposition {
        match self {
            Self::Refuse | Self::SlowStowToRest | Self::ImmediateAllTorqueOffToRest => {
                Disposition::Rest
            }
            Self::MaskedSlowStowToPark | Self::ImmediateAllTorqueOffToPark => Disposition::Park,
        }
    }
}

/// Where a response leaves the machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Disposition {
    /// Torque off, and the next wake engages as usual.
    Rest,
    /// Torque off, and nothing engages until an operator has been.
    Park,
}

/// Which side of the torque line an engage failure fell on.
///
/// The voltage and health polls precede any torque write, while a write that
/// fails on the enable walk lands with servos already energized. The first
/// enable going out is the boundary, and the sequencer is what knows whether it
/// did.
#[must_use]
pub fn engage_phase(torque_written: bool) -> Phase {
    if torque_written {
        Phase::UnderTorque
    } else {
        Phase::PreTorque
    }
}

/// What a sequencer's verdict says about the machine.
///
/// The one partition of [`SeqError`], and the only place a variant of it is
/// judged: both the condition an ending names and the response it asks for are
/// read off this, so the two cannot come to disagree about a variant. A
/// sequencer error added tomorrow is one decision, made here, at compile time.
enum SeqVerdict {
    /// A condition of the machine, which carries its own response.
    Fault(Fault),
    /// Judged before a single transaction writes torque, so nothing was
    /// written, the machine is exactly where it was, and asking again later is
    /// the whole answer — whatever a caller says about the phase.
    Declined,
    /// Our own tables and our own asks, with a machine that is answering
    /// perfectly.
    Defect,
}

/// What `error` says about the machine.
///
/// Two families reach a fault. A transaction that failed says the bus is no
/// longer carrying commands; a pose the solver could not place says the
/// mechanism is outside its own model. They take the same response and would be
/// the same alert, which is exactly why they are told apart here: an operator
/// sent to the cabling over an unsolvable pose is an operator looking at the
/// wrong half of the machine.
fn sequence_verdict(error: &SeqError) -> SeqVerdict {
    match error {
        // The wire: silence, a refusal, a mangled reply, a register that does
        // not hold what was written, servos that answered no ping. A reading
        // that is not a number is here too — it came off the wire in that
        // shape, and no mechanism produces it.
        SeqError::NoAnswer { .. }
        | SeqError::Refused { .. }
        | SeqError::WireCorrupt { .. }
        | SeqError::VerifyMismatch { .. }
        | SeqError::AbsentServos { .. }
        | SeqError::UnplaceableAngle { .. } => {
            SeqVerdict::Fault(Fault::BusFailure { source: *error })
        }
        // The mechanism: angles that place no pose. One verdict, from one
        // measurement, which is what the sequencer took.
        SeqError::RestPoseImplausible { cause, .. }
        | SeqError::PinnedPoseUnsolvable { cause, .. } => {
            SeqVerdict::Fault(Fault::MeasuredPoseInvalid {
                failures: 1,
                source: *cause,
            })
        }
        // The two torque-on gates and the poll behind them, plus the identity
        // and provisioning verdicts: all read before anything is energized.
        SeqError::IdentityMismatch { .. }
        | SeqError::ProvisionMismatch { .. }
        | SeqError::VoltageLow { .. }
        | SeqError::SupplyBelowFloor { .. }
        | SeqError::UnhealthyServo { .. } => SeqVerdict::Declined,
        // The step and the register table disagree about what a register is:
        // our defect, with a machine that is answering perfectly.
        SeqError::WrongAnswer { .. } | SeqError::WrongValue { .. } => SeqVerdict::Defect,
    }
}

/// What a sequencer's verdict names about the machine, once torque is on.
fn sequence_fault(error: &SeqError) -> Option<Fault> {
    match sequence_verdict(error) {
        SeqVerdict::Fault(fault) => Some(fault),
        SeqVerdict::Declined | SeqVerdict::Defect => None,
    }
}

/// What a sequencer's verdict asks for.
fn sequence_class(error: &SeqError, phase: Phase) -> ErrorClass {
    match phase {
        // Nothing is energized, so whatever the verdict there is nothing to
        // bring back and nothing to undo.
        Phase::PreTorque => ErrorClass::Refuse,
        Phase::UnderTorque => match sequence_verdict(error) {
            SeqVerdict::Fault(fault) => ErrorClass::answering(fault.response()),
            SeqVerdict::Declined => ErrorClass::Refuse,
            SeqVerdict::Defect => ErrorClass::SlowStowToRest,
        },
    }
}

impl PumpError {
    /// What this ending asks its caller to do, given whether torque was on.
    ///
    /// The classification point on the driver side, and exhaustive by
    /// construction: a new variant is a decision made here, at compile time,
    /// and never a default anybody falls through to. Where the ending names a
    /// condition of the machine ([`Self::fault`]), that fault's own response is
    /// what this returns — the two never disagree, because only one of them
    /// decides.
    ///
    /// Before torque almost everything is a refusal: nothing is energized, so
    /// there is nothing to bring back and asking again later is correct. The
    /// exceptions are the endings that are about torque itself.
    #[must_use]
    pub fn class(&self, phase: Phase) -> ErrorClass {
        // A defect of ours with a healthy machine: it stows under control once
        // it is holding, and costs nothing at all before that.
        let defect = match phase {
            Phase::PreTorque => ErrorClass::Refuse,
            Phase::UnderTorque => ErrorClass::SlowStowToRest,
        };
        match self {
            Self::Sequence(error) => sequence_class(error, phase),
            // Degenerate: the release already ran, every servo was asked and
            // every write retried. What is left is the park and the alert,
            // because a minimum risk condition nobody confirmed must never be
            // reported as rest.
            Self::TorqueOffUnacked { .. } => ErrorClass::ImmediateAllTorqueOffToPark,
            // The port, the encoder or the frame. Under torque this is a
            // machine that can no longer be commanded, and nothing controlled
            // can be attempted over a wire that is not carrying.
            Self::Bus { .. } => match phase {
                Phase::PreTorque => ErrorClass::Refuse,
                Phase::UnderTorque => ErrorClass::ImmediateAllTorqueOffToPark,
            },
            // Our own arithmetic, our own roster, our own rates and windows.
            Self::Map { .. }
            | Self::UnknownServo { .. }
            | Self::Rate { .. }
            | Self::Settle { .. } => defect,
            // Our own accounting ran out. Whether the machine is stuck is
            // unproven either way, and a stow that meets a stuck one trips
            // tracking or the bus and escalates from there.
            Self::Runaway { .. } | Self::Stalled { .. } => defect,
            // The provisioning and reboot paths, where torque is off by
            // construction: the non-volatile writes refuse the whole roster
            // unless every servo is released, and nothing on the reboot path
            // enables torque.
            Self::WrongPart { .. }
            | Self::TorqueHeld { .. }
            | Self::OffRoster { .. }
            | Self::NotBack { .. }
            | Self::NotRestarted { .. }
            | Self::RestartUnconfirmed { .. } => ErrorClass::Refuse,
            Self::Fault(fault) => ErrorClass::answering(fault.response()),
            // The plan was wrong and the platform is fine: the move dies and
            // the machine stows.
            Self::Aborted(_) => defect,
            // The tick would not take the command. Nothing was changed, so
            // there is nothing to undo.
            Self::Rejected(_) => ErrorClass::Refuse,
        }
    }

    /// The condition of the machine this ending names, if it names one.
    ///
    /// The slug an operator hears and a report carries. Most endings name none:
    /// a refusal, a planner defect and an exhausted budget are all statements
    /// about our own asks. The ones that do are the tick's faults, which arrive
    /// already named, and the transactions that failed with servos holding.
    #[must_use]
    pub fn fault(&self, phase: Phase) -> Option<Fault> {
        match self {
            Self::Sequence(error) => match phase {
                Phase::PreTorque => None,
                Phase::UnderTorque => sequence_fault(error),
            },
            Self::TorqueOffUnacked { id } => Some(Fault::TorqueOffUnconfirmed { id: *id }),
            Self::Fault(fault) => Some(*fault),
            // A verdict about the port, not about the machine, and one with no
            // sequencer context to carry: it is its own detail, and its class
            // says what the wire failing under torque means.
            Self::Bus { .. }
            | Self::Map { .. }
            | Self::WrongPart { .. }
            | Self::TorqueHeld { .. }
            | Self::UnknownServo { .. }
            | Self::OffRoster { .. }
            | Self::NotBack { .. }
            | Self::NotRestarted { .. }
            | Self::RestartUnconfirmed { .. }
            | Self::Runaway { .. }
            | Self::Aborted(_)
            | Self::Rejected(_)
            | Self::Stalled { .. }
            | Self::Rate { .. }
            | Self::Settle { .. } => None,
        }
    }

    /// The fault this ending names that the session has not recorded yet.
    ///
    /// A [`Self::Fault`] ending carries a fault the tick already recorded; the
    /// rest are conditions only the wire-holding layer can see — a bus that
    /// stopped carrying, a torque-off nobody acknowledged — and whoever handles
    /// the ending records them.
    #[must_use]
    pub fn unrecorded_fault(&self, phase: Phase) -> Option<Fault> {
        match self {
            Self::Fault(_) => None,
            other => other.fault(phase),
        }
    }
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

/// Record the maneuver a mask entry belongs to, and say which one that is.
///
/// A servo leaving service is always part of a maneuver: on its own it is the
/// start of the one the fault's response names, and inside a wind-down that is
/// already running it is that wind-down growing — the escalation ladder never
/// begins a second answer to a machine already being answered. Which of the two
/// happened is read off the record itself, so the rule lives in one place
/// instead of in every caller that might be mid-stow.
///
/// `None` only where a mask entry came with no fault at all — no current code
/// path produces that.
fn record_mask_entry(
    state: &mut MotionState,
    report: &TickReport,
    at: Duration,
) -> Option<Maneuver> {
    if let Some(open) = state.timeline().open_maneuver() {
        state.record_response(open, Outcome::Expanded(report.newly_masked), at);
        return Some(open);
    }
    // The raise wins over the degrade where a period carried both — an antenna
    // running its tracking window out on the same period a leg servo flagged.
    // Its maneuver is the one that will actually run, and recording the
    // antennas' release as the answer would close it on the same period and
    // leave the wind-down opening a second answer to one incident. The pair's
    // own fault is in the record either way; what it does not get is a
    // maneuver of its own while a bigger one is starting.
    let maneuver = report
        .fault
        .or(report.degraded)
        .and_then(|fault| fault.response().maneuver())?;
    state.record_response(maneuver, Outcome::Started, at);
    Some(maneuver)
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
    run(bus, map, seq, clock, budget, phase, &mut Err)
}

/// Run a torque-off sequence, and let nothing stop it short of its last write.
///
/// The same loop as [`drive`] with one rule inverted: a transaction that fails
/// in a way no [`BusResult`] describes is handed to `absorbed` and the sequence
/// is told the wire mangled that exchange, so it records the servo and walks on
/// to the next. A long or short reply from one servo mid-release — the tail of
/// an abandoned read landing after the line was cleared is exactly the condition
/// a fault produces — would otherwise end the walk where it stood and leave the
/// servos after it holding the head up, which is this machine's only pinch
/// hazard.
///
/// The failures are not swallowed: each is pushed onto `absorbed` in the order
/// it happened, for the caller to report once the machine is limp, and the
/// servos that never acknowledged their release are in the summary the walk
/// returns.
pub fn drive_release<P, S>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    seq: &mut S,
    clock: &mut dyn Clock,
    budget: usize,
    phase: &mut dyn FnMut(SeqStep),
    absorbed: &mut Vec<PumpError>,
) -> Result<S::Summary, PumpError>
where
    P: BusPort,
    S: Sequencer,
{
    run(bus, map, seq, clock, budget, phase, &mut |error| {
        absorbed.push(error);
        Ok(BusResult::WireCorrupt)
    })
}

/// The driver loop both entry points run, differing only in `recover` — what a
/// transaction failure with no [`BusResult`] of its own becomes.
fn run<P, S>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    seq: &mut S,
    clock: &mut dyn Clock,
    budget: usize,
    phase: &mut dyn FnMut(SeqStep),
    recover: &mut dyn FnMut(PumpError) -> Result<BusResult, PumpError>,
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
            SeqAction::Transact(request) => {
                let result = match execute(bus, map, request) {
                    Ok(result) => result,
                    Err(error) => recover(error)?,
                };
                prior = Some(result);
            }
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

/// What commissioning established, as a supervised run prints it.
///
/// The registers-of-record a bring-up wants written down, and the rail the
/// ceremony finished on. Printed once per process, because commissioning
/// happens once per process — an engage that reprinted it would bury the two
/// lines that differ between one wake and the next in eight that never do.
#[must_use]
pub fn commission_report(commission: &CommissionSummary) -> String {
    let mut out = format!("models     {:?}\n", commission.models);
    let volts: Vec<String> = commission
        .rail
        .voltages
        .iter()
        .map(|v| format!("{v:.1}"))
        .collect();
    out.push_str(&format!(
        "supply     [{}] V after {} poll(s)\n",
        volts.join(", "),
        commission.voltage_polls
    ));
    let health: Vec<String> = commission
        .rail
        .health
        .iter()
        .map(|servo| format!("{:#04x}", servo.bits))
        .collect();
    out.push_str(&format!("health     [{}]\n", health.join(", ")));
    out.push_str(&format!(
        "registers  {} provisioned cells read\n",
        commission.provisioned.count()
    ));
    out
}

/// The torque-on path's report, as a supervised run prints it.
///
/// Two records of the platform — where the poll found it and where it reported
/// itself once torque was on — because they are different poses whenever torque
/// coming on moved a joint, and which one a reader is looking at matters.
#[must_use]
pub fn engage_report(engage: &EngageSummary) -> String {
    let mut out = String::new();
    out.push_str(&record_lines("found", &engage.rest));
    out.push_str(&record_lines("armed", &engage.armed));

    let pulls: Vec<String> = engage
        .pins
        .pull_in
        .iter()
        .map(|rad| format!("{:.3}", rad.to_degrees()))
        .collect();
    out.push_str(&format!(
        "pull-in    legs [{}] deg, worst {:.3} deg\n",
        pulls.join(", "),
        engage.worst_pull_in().to_degrees()
    ));
    // A measurement rather than a verdict: nothing acts on it, and it is a
    // quantity this project has so far only guessed at.
    let shift: Vec<String> = engage
        .post_enable_shift
        .iter()
        .map(|rad| format!("{:.3}", rad.to_degrees()))
        .collect();
    out.push_str(&format!("torque-on  [{}] deg of shift\n", shift.join(", ")));
    // Said only when there is something to say, and said here because this is
    // the one degradation nothing raised: the servo was already flagging when
    // the machine was found, and the gate engaged around it.
    if !engage.degraded.is_empty() {
        out.push_str(&format!(
            "degraded   {} left limp on latched error bits, never commanded\n",
            engage.degraded
        ));
    }
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

    /// The same value assembled from verdicts named directly, for a caller that
    /// has no read outcome in hand — a test standing in for a bus, or a program
    /// replaying what one reported.
    ///
    /// Clean answers are dropped exactly as they are on the read path, so
    /// [`servos`](Self::servos) is failures and nothing else, and verdicts past
    /// the ninth are dropped: the array is the joint count wide and a grouped
    /// read cannot exceed it.
    #[must_use]
    pub fn from_verdicts(verdicts: &[(u8, IdOutcome)], corrupt_frames: u32) -> Self {
        let mut failures = Self {
            servos: [(0, IdOutcome::Timeout); JointId::COUNT],
            count: 0,
            corrupt: corrupt_frames,
        };
        for &(id, verdict) in verdicts {
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
    /// The clocks a move was accepted on were not the ones asked for, or the
    /// antenna pair reached its crossing nearer mirrored than the tips clear
    /// each other by. Emitted for the command that starts a run and for every
    /// replacement that retargets one.
    Stretched(ClockStretch),
    /// The trajectory reached its endpoint: the last goal has gone out and
    /// nothing further will be commanded. Where the machine physically is at
    /// this moment is a separate question, answered by [`Self::Settled`].
    Completed,
    /// Every joint has been measured within the settle tolerance of the goal it
    /// was left on, this long after commanding finished.
    Settled {
        /// The gap between commanding finishing and the machine arriving.
        after: Duration,
    },
    /// The settle window ran out with a joint still outside the tolerance. A
    /// report and not a fault: the run ends saying where the machine actually
    /// got to.
    Unsettled {
        /// The joint furthest from its goal when the window ran out.
        joint: JointId,
        /// How far out it was, radians.
        error: f64,
        /// The window that ran out.
        waited: Duration,
    },
    /// The per-period trace filled its buffer and recording stopped. The
    /// periods themselves carry on unaffected.
    TraceFull {
        /// How many periods were recorded before it stopped.
        samples: usize,
    },
    /// The tick stopped commanding.
    Faulted(Fault),
    /// The antennas were taken out of service over this fault: neither is
    /// commanded again this session and the move carries on without them.
    ///
    /// Emitted on the period the tick raised it, before the release those
    /// joints are owed goes on the wire — so a release that fails is reported
    /// after this rather than instead of it.
    AntennasDegraded(Fault),
    /// The tick abandoned the move and went back to holding.
    Aborted(MoveAbort),
}

impl fmt::Display for TickEvent {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Command(CommandDisposition::None) => f.write_str("no command"),
            Self::Command(CommandDisposition::Started) => f.write_str("moving"),
            Self::Command(CommandDisposition::Retargeted) => {
                f.write_str("moving, replacing the move that was running")
            }
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
            Self::Stretched(stretch) => {
                if stretch.requested != stretch.effective {
                    let why = if stretch.dephased {
                        "to fit the span and de-phase the antennas"
                    } else {
                        "to fit the span"
                    };
                    write!(
                        f,
                        "clock stretched {why}: head {:.3} s to {:.3} s, right antenna \
                         {:.3} s to {:.3} s, left antenna {:.3} s to {:.3} s",
                        stretch.requested.head.as_secs_f64(),
                        stretch.effective.head.as_secs_f64(),
                        stretch.requested.antennas[0].as_secs_f64(),
                        stretch.effective.antennas[0].as_secs_f64(),
                        stretch.requested.antennas[1].as_secs_f64(),
                        stretch.effective.antennas[1].as_secs_f64(),
                    )?;
                }
                // Said whenever the pass measured one, because the number that
                // matters to an operator watching the tips cross is what the
                // pair came to and not whether a clock moved.
                let Some(pair) = stretch.separation else {
                    return Ok(());
                };
                if stretch.requested != stretch.effective {
                    f.write_str("; ")?;
                }
                write!(f, "the antennas cross {:.2} rad apart", pair.offset)?;
                if pair.met(stretch.separation_required) {
                    Ok(())
                } else {
                    write!(
                        f,
                        ", under the {:.2} rad that keeps their tips clear",
                        stretch.separation_required
                    )
                }
            }
            Self::Completed => f.write_str("commanding finished"),
            Self::Settled { after } => write!(
                f,
                "measurably at the goal, {:.2} s after commanding finished",
                after.as_secs_f64()
            ),
            Self::Unsettled {
                joint,
                error,
                waited,
            } => write!(
                f,
                "{:.2} s after commanding finished {joint} is still {:.2}° from its goal",
                waited.as_secs_f64(),
                error.to_degrees()
            ),
            Self::TraceFull { samples } => {
                write!(f, "trace buffer full at {samples} period(s); not recording")
            }
            Self::Faulted(fault) => write!(f, "faulted: {fault}"),
            Self::AntennasDegraded(fault) => {
                write!(
                    f,
                    "antennas out of service and left out of the move: {fault}"
                )
            }
            Self::Aborted(abort) => write!(f, "move abandoned: {abort}"),
        }
    }
}

/// What a move cost, once it is over.
///
/// Filled whichever way the run ended. A faulted move is the one whose numbers
/// are worth the most, so nothing in here waits on a clean exit to exist.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
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
    /// Wall time the run spent that the move's clock was not credited with:
    /// per period, whatever it began more than one period after the last one
    /// did.
    ///
    /// A move's trajectory advances by at most a nominal period per period, so
    /// a loop that stalls delays the gesture by the stall instead of jumping
    /// the path. This is how much of that happened — the difference between the
    /// timeline the caller scripted and the one the machine ran.
    pub slip: Duration,
    /// The furthest each joint was measured behind the goal it had been
    /// written, in bus order, radians, over every period with a live read.
    ///
    /// A measurement and not a verdict: what a proportional loop runs behind a
    /// moving goal is the quantity the tracking threshold, the window and the
    /// stow tolerance have all been guessed against, and every move produces it.
    ///
    /// A joint stops contributing once it is masked: it is written nothing from
    /// there on, and the distance from where it was released to a goal row
    /// nobody sent it is not a lag anything can be calibrated against.
    pub worst_lag: [f64; JointId::COUNT],
    /// Wall time from the first period to the last.
    pub elapsed: Duration,
    /// When the trajectory stopped commanding — the first period that found the
    /// machine holding — from the run's first period.
    ///
    /// `None` for a run that never got there, and for a timed hold, which
    /// commands nothing and so has no such instant.
    pub commanded: Option<Duration>,
    /// When every joint was first measured within the settle tolerance of the
    /// goal it was left on, from the run's first period.
    ///
    /// The instant the machine physically arrived, as against
    /// [`commanded`](Self::commanded), which is when the last goal went out. The
    /// gap between them is the settle, and on a move whose clock is shorter than
    /// the machine's own response it is most of the motion.
    pub settled: Option<Duration>,
    /// The joint furthest from its goal when the settle window ran out, and how
    /// far out it was in radians. `None` unless the window ran out.
    pub unsettled: Option<(JointId, f64)>,
}

/// How close the machine has to be measured to the goals it was left on before
/// a run is over, and how long that is waited for.
///
/// The tolerance answers a different question from the tick's tracking
/// threshold, which is why it is a separate and much tighter figure: tracking
/// asks whether the position loop is keeping up with a *moving* goal, this asks
/// where the machine physically came to rest once the goal stopped moving.
///
/// The window is bounded because a joint that never arrives — a stalled motor,
/// an antenna held by hand — must end the run with that reported rather than
/// leave the loop turning. Running out is an outcome, not a fault: nothing here
/// gates or delays what a caller does about torque.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SettleConfig {
    /// How far a joint may be from its goal and still count as arrived,
    /// radians.
    pub tolerance: f64,
    /// How long after commanding finished arrival is waited for.
    pub timeout: Duration,
}

/// One control period as it was measured, for the trace a run can record.
///
/// Both halves of the period are here because a velocity profile is only
/// readable against what was asked for: the measured angles, and the goals the
/// servos were holding when those angles were read.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TickSample {
    /// Periods since the run's first, counted from zero.
    pub tick: u64,
    /// When the period began, on the run's own epoch.
    pub at: Duration,
    /// The nine measured angles, or `None` for a period whose grouped read fell
    /// short.
    pub present: Option<JointVector>,
    /// The nine goals the servos were holding when this period read them.
    ///
    /// A servo in `released` holds none of it: its cell is whatever it was last
    /// commanded, kept only so the vector stays nine wide, and the trace blanks
    /// it rather than writing it out.
    pub goal: JointVector,
    /// The servos that were torqued off when this period read them, and so were
    /// holding no goal at all.
    pub released: JointSet,
    /// Whether commanding had already finished and the period was one of those
    /// spent waiting for the machine to arrive.
    pub settling: bool,
}

/// What the servos were holding while a period's read was taken.
///
/// One statement about the wire, captured once per period and before that
/// period's write, because the write is the answer to the measurement rather
/// than part of it: the goals standing on the servos, and the servos standing
/// on no goal at all because they have been released.
#[derive(Clone, Copy, Debug)]
struct Held {
    /// The nine goals, of which a released servo's is only the last it was sent.
    goal: JointVector,
    /// The servos that are torqued off and commanded nothing.
    released: JointSet,
}

/// What ends a run of the fixed-rate loop.
///
/// The caller's say over a run in flight lives on the endpoint arm and nowhere
/// else, because that is the only run whose ending is a place rather than a
/// time: a hold ends when its dwell is up and there is nothing about it to
/// change.
enum Until<'r> {
    /// The machine is holding again: the move reached its endpoint. `retarget`
    /// is asked at every period that has no command pending whether that
    /// endpoint is still the right one.
    Holding {
        retarget: &'r mut dyn FnMut() -> Option<MotionCommand>,
    },
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
    settle: SettleConfig,
    written: JointVector,
    outputs: TickOutputs,
    present: SyncReadOutcome,
    health: SyncReadOutcome,
    summary: MoveSummary,
    /// Whether a run records a sample per period. Off unless a caller asked,
    /// because the loop otherwise allocates nothing while it runs.
    tracing: bool,
    /// The last run's per-period samples, oldest first.
    trace: Vec<TickSample>,
    /// How many samples one run may keep before recording stops.
    /// [`MAX_TRACE_SAMPLES`] unless a test wound it down: a ceiling only
    /// reachable after forty minutes of motion is a ceiling no test can drive
    /// the loop to.
    max_samples: usize,
    /// Whether this run has already said the buffer filled.
    trace_full: bool,
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
        settle: SettleConfig,
    ) -> Result<Self, PumpError> {
        if tick_hz == 0 || health_poll_hz == 0 {
            return Err(PumpError::Rate {
                tick_hz,
                health_poll_hz,
            });
        }
        // An unplaceable tolerance is refused with the rest: an infinite one
        // makes every joint arrive the instant commanding stops, and a
        // comparison against a NaN answers false for every joint, so no run
        // would ever arrive at all.
        let comparable = settle.tolerance.is_finite() && settle.tolerance > 0.0;
        if !comparable || settle.timeout.is_zero() {
            return Err(PumpError::Settle {
                tolerance: settle.tolerance,
                timeout: settle.timeout,
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
            settle,
            written: held,
            outputs: TickOutputs::default(),
            present: SyncReadOutcome::new(),
            health: SyncReadOutcome::new(),
            summary: MoveSummary::default(),
            tracing: false,
            trace: Vec::new(),
            max_samples: MAX_TRACE_SAMPLES,
            trace_full: false,
        })
    }

    /// Wind the trace buffer's ceiling down to `samples`, for a test that has
    /// to reach it.
    #[cfg(test)]
    fn trace_ceiling(&mut self, samples: usize) {
        self.max_samples = samples;
    }

    /// Record a sample per control period from the next run onward, or stop.
    ///
    /// Off by default: a run that records nothing allocates nothing per period,
    /// and the trace exists for the diagnostic question — where the machine
    /// actually was, period by period — rather than for routine operation.
    pub fn record_trace(&mut self, on: bool) {
        self.tracing = on;
        if !on {
            self.trace = Vec::new();
        }
    }

    /// The last run's per-period samples, oldest first.
    ///
    /// Empty when the run was not recorded. Cleared at the start of every run,
    /// like the summary: this is never the previous run's motion.
    #[must_use]
    pub fn last_trace(&self) -> &[TickSample] {
        &self.trace
    }

    /// The control period.
    #[must_use]
    pub fn period(&self) -> Duration {
        self.period
    }

    /// What the last run measured, whether it reached its endpoint, faulted, or
    /// ran out of periods.
    ///
    /// The counts and the per-joint lag are the point of running a move on
    /// hardware at all, and a faulted run is where they matter most, so they are
    /// kept here rather than only handed back on the success path. Zeroed at the
    /// start of every run: this is never the previous move's record.
    #[must_use]
    pub fn last_summary(&self) -> MoveSummary {
        self.summary
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
            MotionCommand::MoveTo { durations, .. } => self.budget_for(durations.longest()),
            // A hold commands no travel: it takes the period it is asked in.
            MotionCommand::Hold => self.budget_for(Duration::ZERO),
        }
    }

    /// `command` on a clock that can carry its span, and the stretch when there
    /// was one.
    ///
    /// Applied here, and not inside the tick, because of what reads a command
    /// first: the stall budget is sized from the durations before the tick ever
    /// sees them, so a clock stretched downstream of this would leave the loop
    /// killing the very move the stretch exists to save. One dry pass per
    /// command or splice, never inside a move already running.
    fn right_sized(
        &self,
        state: &MotionState,
        command: MotionCommand,
    ) -> (MotionCommand, Option<ClockStretch>) {
        floor_move_clock(
            self.cfg,
            state.last_targets(),
            &command,
            1.0 / self.period.as_secs_f64(),
        )
    }

    /// How many periods a run spanning `wanted` may take before the loop calls
    /// it stuck: the periods that span it, plus the fixed margin.
    fn budget_for(&self, wanted: Duration) -> u64 {
        let periods = wanted.as_nanos().div_ceil(self.period.as_nanos());
        u64::try_from(periods)
            .unwrap_or(u64::MAX)
            .saturating_add(self.stall_margin)
    }

    /// How many periods the settle window spans, and one more so the period
    /// that finds the window over is inside the budget rather than outside it.
    ///
    /// Without this a settle would be spent out of the move's own stall budget
    /// and a machine that took its time arriving would be reported as a loop
    /// that hung.
    fn settle_periods(&self) -> u64 {
        let periods = self
            .settle
            .timeout
            .as_nanos()
            .div_ceil(self.period.as_nanos());
        u64::try_from(periods).unwrap_or(u64::MAX).saturating_add(1)
    }

    /// The joint furthest outside the settle tolerance of the goal it was last
    /// written, and how far out it is — or nothing, when every joint is inside
    /// it and the machine has measurably arrived.
    ///
    /// An error that will not compare — a joint whose measurement or goal is
    /// unplaceable — is outside every tolerance there is, which is what the
    /// negated comparison says.
    ///
    /// A joint in `masked` is skipped. It was released and is commanded no
    /// further, so where it came to rest is not a measurement of this run
    /// arriving, and waiting for it would end every degraded move at the window
    /// with a joint nobody expected to move named as the one that did not.
    fn straying(&self, present: &JointVector, masked: JointSet) -> Option<(JointId, f64)> {
        let mut worst: Option<(JointId, f64)> = None;
        for ((joint, angle), (_, goal)) in present.joints().into_iter().zip(self.written.joints()) {
            if masked.contains(joint) {
                continue;
            }
            let error = (angle - goal).abs();
            if error <= self.settle.tolerance {
                continue;
            }
            if worst.is_none_or(|(_, out)| error > out) {
                worst = Some((joint, error));
            }
        }
        worst
    }

    /// Ready the trace buffer for a run of at most `budget` periods.
    ///
    /// The whole run's room is taken here rather than grown period by period,
    /// so a recorded run allocates no more often than an unrecorded one — which
    /// allocates not at all.
    fn start_trace(&mut self, budget: u64) {
        self.trace.clear();
        self.trace_full = false;
        if !self.tracing {
            return;
        }
        let wanted = usize::try_from(budget).unwrap_or(self.max_samples);
        self.trace.reserve(wanted.min(self.max_samples));
    }

    /// Keep this period's measurement, if the run is being recorded.
    ///
    /// The buffer's ceiling is announced once, on the period that reaches it:
    /// a trace that stops halfway through a run is a fact about the trace and
    /// says nothing about the machine, so the periods carry on either way.
    fn record(
        &mut self,
        tick: u64,
        at: Duration,
        present: Option<&JointVector>,
        held: Held,
        settling: bool,
        event: &mut dyn FnMut(TickEvent),
    ) {
        if !self.tracing {
            return;
        }
        if self.trace.len() >= self.max_samples {
            if !self.trace_full {
                self.trace_full = true;
                event(TickEvent::TraceFull {
                    samples: self.trace.len(),
                });
            }
            return;
        }
        self.trace.push(TickSample {
            tick,
            at,
            present: present.copied(),
            goal: held.goal,
            released: held.released,
            settling,
        });
    }

    /// Run `command` to completion.
    ///
    /// Returns when the machine has measurably arrived — the trajectory
    /// finished commanding and every joint has since been read within the
    /// settle tolerance of its goal — or when the settle window ran out with a
    /// joint still short of it, which comes back as a summary saying so rather
    /// than as an error. It refuses on anything else: a refused command, a
    /// fault, a transaction that is not a machine verdict. Nothing here touches
    /// torque on the way out, whichever exit is taken.
    pub fn run<P: BusPort>(
        &mut self,
        bus: &mut Bus<P>,
        state: &mut MotionState,
        command: MotionCommand,
        clock: &mut dyn Clock,
        event: &mut dyn FnMut(TickEvent),
    ) -> Result<MoveSummary, PumpError> {
        self.carry(
            bus,
            state,
            command,
            Until::Holding {
                retarget: &mut || None,
            },
            clock,
            event,
        )
    }

    /// Run `command` to completion, asking `retarget` at every control period
    /// whether it has become the wrong command.
    ///
    /// `retarget` is consulted at the top of each period that has no command
    /// pending, and answering `Some` replaces the move in flight: the tick
    /// shapes the new path from the setpoint the previous period commanded, so
    /// nothing jumps and the machine never stops on the way. The run then ends
    /// when the *replacement* has been carried and arrived at, and the stall
    /// budget is re-sized to the replacement's own travel — a caller changing
    /// its mind is not a loop that hung.
    ///
    /// `retarget` is asked during the settle too, so a caller is never made to
    /// wait out an arrival it has already changed its mind about; a replacement
    /// taken there puts the run back to commanding.
    ///
    /// [`MotionPump::run`] is this with nothing to say. Two entry points for the
    /// same reason [`MotionPump::hold`] has two: a bench move is commanded once
    /// and watched to its end, while a program taking instructions off a wire
    /// has a schedule that can move under it mid-travel.
    pub fn run_retargeting<P: BusPort>(
        &mut self,
        bus: &mut Bus<P>,
        state: &mut MotionState,
        command: MotionCommand,
        clock: &mut dyn Clock,
        event: &mut dyn FnMut(TickEvent),
        retarget: &mut dyn FnMut() -> Option<MotionCommand>,
    ) -> Result<MoveSummary, PumpError> {
        self.carry(
            bus,
            state,
            command,
            Until::Holding { retarget },
            clock,
            event,
        )
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
        mut until: Until<'_>,
        clock: &mut dyn Clock,
        event: &mut dyn FnMut(TickEvent),
    ) -> Result<MoveSummary, PumpError> {
        let (command, stretch) = self.right_sized(state, command);
        if let Some(stretch) = stretch {
            event(TickEvent::Stretched(stretch));
        }
        let mut budget = match until {
            Until::Holding { .. } => self.stall_budget(&command),
            Until::Elapsed(dwell) => self.budget_for(dwell),
        };
        let epoch = clock.now();
        let mut due = epoch;
        // Where the previous period actually began, which is what the move's
        // clock advanced on.
        let mut prev_now = epoch;
        let mut pending = Some(command);
        // The run's own record, and never the last run's: a caller reading it
        // after a command refused on the accepting period — which ends the run
        // before any of it was measured — gets zeros.
        self.summary = MoveSummary::default();
        self.start_trace(budget);
        // When commanding finished, on the run's epoch. `Some` puts the run in
        // its settle phase: the periods after the last goal, which command
        // nothing and read every one of them.
        let mut commanded_at: Option<Duration> = None;
        let mut summary = MoveSummary::default();
        let mut last_misses = 0;
        let mut health_misses: u32 = 0;
        let mut reported_health: Option<[ServoHealth; JointId::COUNT]> = None;
        let mut overrun_reported = false;
        // Which servos were already out of service when the current period's
        // read was taken. Trails the tick's own mask by the period the mask is
        // raised on: a servo is released partway through that period, so the
        // read at the top of it still measured a servo holding the goal in
        // `written`, and that period is the whole diagnosis of why it was
        // masked. That trailing applies to a mask this run raised; one the
        // engagement arrived with is already true of the first period, and a
        // goal cell for a joint this run never commanded is a lag figure drawn
        // against a command that never went out.
        let mut released = state.masked();
        // How the loop ended, filled by the one period that ends it. Still
        // `None` when the periods run out, which is the stall.
        let mut ending: Option<Result<(), PumpError>> = None;

        let mut tick: u64 = 0;
        while tick < budget {
            // Asked before the period's read, so a replacement command reaches
            // the same period's tick and the machine turns on the very next
            // setpoint. Skipped while a command is already pending — the
            // accepting period, and the one that accepts a replacement, have
            // nothing to ask about yet.
            if pending.is_none()
                && let Until::Holding { retarget } = &mut until
                && let Some(replacement) = retarget()
            {
                // Right-sized against where the machine has got to, so a
                // replacement issued mid-recovery gets a clock for the span
                // still ahead of it rather than for the one the original
                // command had.
                let (replacement, stretch) = self.right_sized(state, replacement);
                if let Some(stretch) = stretch {
                    event(TickEvent::Stretched(stretch));
                }
                // Re-sized from here, not extended: what bounds the run is the
                // travel still ahead of it, and a caller that keeps changing
                // its mind is doing exactly what this entry point is for.
                budget = tick.saturating_add(self.stall_budget(&replacement));
                pending = Some(replacement);
                // A run in its settle phase is commanding again, so what it had
                // measured about arriving is about a goal that is no longer the
                // one it is heading for.
                commanded_at = None;
                summary.commanded = None;
                summary.settled = None;
                summary.unsettled = None;
            }
            let now = clock.now();
            // A period that starts materially after it was due is one the loop
            // ran late for. It proceeds immediately rather than skipping the
            // next one: the reads stay one per period, and the move's clock
            // takes the delay as a delay.
            let late = now.saturating_sub(due);
            summary.worst_jitter = summary.worst_jitter.max(late);
            summary.slip += now.saturating_sub(prev_now).saturating_sub(self.period);
            prev_now = now;
            if late >= self.period / OVERRUN_DIVISOR {
                summary.overruns += 1;
                if !overrun_reported {
                    event(TickEvent::Overrun { tick, late });
                    overrun_reported = true;
                }
            }
            // Past a whole period behind, the missed slots are abandoned rather
            // than replayed: the grid re-bases on where the loop actually is.
            // Marching through the backlog at bus-limited rate would spend a
            // read and a goal on each of them, running the machine through that
            // stretch of the path as fast as the wire allows.
            if late > self.period {
                due = now;
            }

            let present = match self.read_present(bus) {
                Ok(present) => present,
                Err(error) => {
                    ending = Some(Err(error));
                    break;
                }
            };
            let polled = if tick.is_multiple_of(self.health_every) {
                match self.read_health(bus) {
                    Ok(servos) => Some(servos),
                    Err(error) => {
                        ending = Some(Err(error));
                        break;
                    }
                }
            } else {
                None
            };
            let health = match polled {
                Some(Some(servos)) => {
                    if health_misses > 0 {
                        event(TickEvent::HealthRestored {
                            after: health_misses,
                        });
                        health_misses = 0;
                    }
                    Some(servos)
                }
                Some(None) => {
                    health_misses += 1;
                    summary.health_misses += 1;
                    if health_misses == 1 {
                        event(TickEvent::HealthLost {
                            failed: ReadFailures::of(&self.health),
                        });
                    }
                    None
                }
                None => None,
            };

            let inputs = TickInputs {
                now,
                period: self.period,
                present: present.as_ref(),
                command: pending.as_ref(),
                health: health.as_ref(),
            };
            motion_tick(self.cfg, state, &inputs, &mut self.outputs);
            let asked = pending.take().is_some();
            summary.ticks += 1;

            let held = Held {
                goal: self.written,
                released,
            };
            let elapsed = now.saturating_sub(epoch);
            // The period that finished commanding is a commanding period: it
            // put the last goal out. The ones after it are the settle.
            let settling = commanded_at.is_some_and(|since| since < elapsed);

            // Servos the tick took out of service are released here, before
            // anything else goes on the wire: the tick decided they are no
            // longer commanded, and a joint that is not commanded and still
            // torqued is holding a position nobody is watching.
            let masked = self.outputs.report.masked;
            let newly_masked = self.outputs.report.newly_masked;
            // Said before the wire work it obliges, because the release can
            // fail and its ending names the bus rather than the servo that
            // flagged.
            if let Some(fault) = self.outputs.report.degraded {
                event(TickEvent::AntennasDegraded(fault));
            }
            if !newly_masked.is_empty() {
                let maneuver = record_mask_entry(state, &self.outputs.report, now);
                match self.release(bus, newly_masked) {
                    Ok(()) => {
                        // Torquing the pair off *is* the whole of
                        // `antenna_torque_off`, so a fresh one is over as soon
                        // as the write lands. An expansion belongs to the
                        // maneuver that is still running and ends with it.
                        if maneuver == Some(Maneuver::AntennaTorqueOff) {
                            state.record_response(
                                Maneuver::AntennaTorqueOff,
                                Outcome::Completed,
                                now,
                            );
                        }
                    }
                    Err(error) => {
                        // Same reason the fault is said first: the branch below
                        // that would have announced it is not reached from
                        // here.
                        if let Some(fault) = self.outputs.report.fault {
                            event(TickEvent::Faulted(fault));
                        }
                        if let Some(maneuver) = maneuver {
                            state.record_response(maneuver, Outcome::FellThrough, now);
                        }
                        self.record(tick, elapsed, present.as_ref(), held, settling, event);
                        ending = Some(Err(error));
                        break;
                    }
                }
            }
            released = masked;

            if let Some(goal) = self.outputs.goal {
                match self.write_goals(bus, &goal, masked) {
                    Ok(frames) => summary.frames += frames,
                    Err(error) => {
                        // Kept before the break, like every other period the
                        // run counted: the trace is a sample per period turned,
                        // and the period a run ends badly on is the one a reader
                        // came for.
                        self.record(tick, elapsed, present.as_ref(), held, settling, event);
                        ending = Some(Err(error));
                        break;
                    }
                }
                summary.goals += 1;
            }

            let report = self.outputs.report;
            if let Some(errors) = report.tracking_errors {
                for (joint, (worst, error)) in JointId::ALL
                    .into_iter()
                    .zip(summary.worst_lag.iter_mut().zip(errors))
                {
                    // A masked joint is written nothing further while the plan
                    // its goal row follows keeps moving, so the difference the
                    // tick measures for it after the mask is against a command
                    // that never went out. The trace blanks that cell for the
                    // same reason; a lag figure the guards are calibrated
                    // against must not carry it either.
                    if masked.contains(joint) {
                        continue;
                    }
                    *worst = worst.max(error);
                }
            }
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
                // The measurement that crossed the line, kept: for a tracking
                // fault this period is the whole diagnosis, and a trace ending
                // one period short of it says nothing about why.
                self.record(tick, elapsed, present.as_ref(), held, settling, event);
                ending = Some(Err(PumpError::Fault(fault)));
                break;
            }
            if let Some(abort) = report.aborted {
                event(TickEvent::Aborted(abort));
                // The period that produced the sample nobody would emit, kept
                // for the same reason a faulted one is: it is the whole
                // diagnosis of a planner that went wrong.
                self.record(tick, elapsed, present.as_ref(), held, settling, event);
                ending = Some(Err(PumpError::Aborted(abort)));
                break;
            }
            if report.completed {
                event(TickEvent::Completed);
            }
            let over = match until {
                Until::Holding { .. } if matches!(state.mode(), Mode::Holding) => {
                    // The first such period is where commanding ended; the ones
                    // after it are the settle, and they command nothing.
                    let since = *commanded_at.get_or_insert_with(|| {
                        summary.commanded = Some(elapsed);
                        // The budget was sized for travel that is now over, so
                        // the window gets periods of its own rather than
                        // spending what is left of the move's.
                        budget = budget.max(tick.saturating_add(self.settle_periods()));
                        elapsed
                    });
                    let waited = elapsed.saturating_sub(since);
                    // A blind period judges nothing: the tick's read-loss budget
                    // is what a run of them ends in, and it is shorter than any
                    // sane window.
                    match present
                        .as_ref()
                        .map(|present| self.straying(present, masked))
                    {
                        Some(None) => {
                            summary.settled = Some(elapsed);
                            event(TickEvent::Settled { after: waited });
                            true
                        }
                        Some(Some((joint, error))) if waited >= self.settle.timeout => {
                            summary.unsettled = Some((joint, error));
                            event(TickEvent::Unsettled {
                                joint,
                                error,
                                waited,
                            });
                            true
                        }
                        _ => waited >= self.settle.timeout,
                    }
                }
                Until::Holding { .. } => false,
                Until::Elapsed(dwell) => elapsed >= dwell,
            };
            self.record(tick, elapsed, present.as_ref(), held, settling, event);
            if over {
                ending = Some(Ok(()));
                break;
            }

            tick += 1;
            due += self.period;
            clock.sleep_until(due);
        }

        // The one place a run is sealed. Every way the loop ends — reaching the
        // endpoint, faulting, losing the wire, or running out of periods —
        // stamps the elapsed time and keeps the record here, and what the
        // caller is handed is that same record, so `last_summary()` and the
        // returned value cannot come to disagree. A run that ended badly is the
        // one whose period counts and per-joint lag are worth the most, which
        // is why the record outlives the run rather than riding only on the
        // success path.
        summary.elapsed = clock.now().saturating_sub(epoch);
        self.summary = summary;
        match ending {
            Some(result) => result.map(|()| self.summary),
            None => Err(PumpError::Stalled { budget }),
        }
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
        masked: JointSet,
    ) -> Result<u64, PumpError> {
        let mut frames = 0;
        for group in JointGroup::ALL {
            let mut rows = [0usize; JointId::COUNT];
            let mut in_group = 0;
            for (row, joint) in JointId::ALL.into_iter().enumerate() {
                // A masked servo has been torqued off and is never commanded
                // again, so its row leaves the frame entirely — the write is
                // one place, and this is it.
                if joint.group() == group && !masked.contains(joint) {
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

    /// Release `joints`, one acknowledged write each.
    ///
    /// The rule every servo entering the mask goes through: masked is
    /// de-torqued, not merely unspoken to. What a servo's torque actually is
    /// after it flags an error is not something to be assumed, so the release
    /// is commanded and verified rather than believed, through the same retry
    /// policy every other write on this bus uses. Only failure past that policy
    /// ends the run — a bus that cannot release one servo cannot be trusted to
    /// carry the head down either.
    fn release<P: BusPort>(&mut self, bus: &mut Bus<P>, joints: JointSet) -> Result<(), PumpError> {
        for joint in joints.iter() {
            let row = joint.index().expect(BUS_ORDER_IS_NINE_JOINTS);
            let id = self.servo_at(row);
            let raw = self
                .map
                .encode_value(row, RegId::TorqueEnable, RegValue::U8(0))
                .map_err(|source| self.map_failure(row, RegId::TorqueEnable, source))?;
            with_retry(bus, |bus| {
                bus.write_reg_verified(id, reg_for(RegId::TorqueEnable), &raw)
            })
            .map_err(|source| PumpError::Bus { id, source })?;
        }
        Ok(())
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
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;

    use dxl_proto::frame::{INST_PING, INST_READ, INST_SYNC_READ, INST_SYNC_WRITE, INST_WRITE};
    use reachy_bus::reg_for;
    use reachy_kin::FkError;
    use reachy_motion::{
        CommissionSequencer, EngageSequencer, Entry, HEAD_GROUP_FLOOR_S, JointId, JointStep,
        JointTargets, MoveDurations, PhaseSeparation, PollCadence, PollSequencer, Posture,
        RegValue, StepContext, ValueKind, Warp,
    };

    use super::*;
    use crate::config::Resolved;
    use crate::testutil::{
        BrokenPort, FailsAfter, FakeMachine, Spy, TestClock, datumed_config, machine_at, resolved,
        rest_legs, stow_legs, wind_down_bus,
    };

    /// What the whole torque-on path handed back, in the order it ran.
    #[derive(Debug)]
    struct Armed {
        commission: CommissionSummary,
        posture: Posture,
        engage: EngageSummary,
    }

    /// Commission, poll and engage the machine on `bus`, as a bench command
    /// does, announcing phases through `phase`.
    fn arm_over<P: BusPort>(
        cfg: &Resolved,
        bus: &mut Bus<P>,
        clock: &mut dyn Clock,
        phase: &mut dyn FnMut(SeqStep),
    ) -> Result<Armed, PumpError> {
        let budget = action_budget(&cfg.arm);
        let mut commissioning = CommissionSequencer::new(&cfg.arm);
        let commission = drive(bus, &cfg.map, &mut commissioning, clock, budget, phase)?;
        let mut polling = PollSequencer::new(&cfg.arm, commission.rail, PollCadence::Positions);
        let posture = drive(bus, &cfg.map, &mut polling, clock, budget, phase)?;
        let mut engaging =
            EngageSequencer::new(&cfg.arm, &cfg.motion.geom, &cfg.motion.fk, &posture);
        let engage = drive(bus, &cfg.map, &mut engaging, clock, budget, phase)?;
        Ok(Armed {
            commission,
            posture,
            engage,
        })
    }

    /// Drive the whole torque-on path against `machine`, handing back the
    /// outcome, the phases as they were announced, and every instruction that
    /// crossed the wire.
    #[allow(clippy::type_complexity)]
    fn arm(
        cfg: &Resolved,
        machine: FakeMachine,
    ) -> (
        Result<Armed, PumpError>,
        Vec<SeqStep>,
        Rc<RefCell<Vec<(u8, u8)>>>,
    ) {
        let spy = Spy::new(machine);
        let log = spy.log();
        let mut bus = Bus::new(spy, cfg.timing);
        let mut clock = TestClock::default();
        let mut phases = Vec::new();
        let outcome = arm_over(cfg, &mut bus, &mut clock, &mut |step| {
            phases.push(step);
        });
        (outcome, phases, log)
    }

    /// A machine provisioned as configured and resting where it can be armed
    /// goes all the way through, and the goals that reached it are the pins.
    /// The two refusals `provision` raises itself, as an operator reads them.
    ///
    /// One of them carries the whole recovery — which command releases the
    /// torque that is in the way — and the other names the part that answered
    /// against the part this platform carries. Neither is reachable from a test
    /// that only destructures the variant, so the rendered strings are pinned
    /// here the way the tick's refusals are.
    #[test]
    fn the_provisioning_refusals_say_what_an_operator_does_next() {
        let refusals: [(PumpError, &str); 2] = [
            (
                PumpError::TorqueHeld { id: 17 },
                "servo 17 is holding torque; release it with `off` before provisioning",
            ),
            (
                PumpError::WrongPart {
                    id: 18,
                    model: 1200,
                    expected: 1190,
                },
                "servo 18 reports model 1200, where this platform's is 1190",
            ),
        ];
        for (refusal, expected) in refusals {
            assert_eq!(refusal.to_string(), expected);
        }
    }

    #[test]
    fn an_arm_sequence_runs_to_completion_over_the_port() {
        let cfg = resolved();
        let (outcome, phases, log) = arm(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let summary = outcome.expect("a correct machine arms");

        // Nothing had to be pulled: this machine rests inside every window.
        assert_eq!(summary.engage.pins.pull_in, [0.0; 6]);
        assert_eq!(summary.commission.voltage_polls, 1);
        // A positions-only sweep carries the commissioning rail forward.
        assert!(!summary.posture.rail_read);

        assert_eq!(
            phases,
            vec![
                SeqStep::Presence,
                SeqStep::Identity,
                SeqStep::Provision,
                SeqStep::VoltageGate,
                SeqStep::Health,
                SeqStep::GainsProfiles,
                SeqStep::PoseAndDatum,
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

    /// The pin sweep goes out before the enables, so on a platform whose goal
    /// register mirrors its present position while limp it is dropped — and
    /// engaging carries on regardless, leaving every servo holding torque where
    /// it stood.
    ///
    /// Read off the fixture's own register file, which models the mirroring, so
    /// this is a check on the machine rather than on the summary.
    #[test]
    fn the_pins_are_dropped_by_a_mirroring_register_and_the_engage_stands() {
        let cfg = resolved();
        let spy = Spy::new(machine_at(&datumed_config(), &stow_legs()));
        let registers = spy.machine();
        let mut bus = Bus::new(spy, cfg.timing);
        let mut clock = TestClock::default();
        let summary =
            arm_over(&cfg, &mut bus, &mut clock, &mut |_| {}).expect("a correct machine engages");

        let machine = registers.borrow();
        for (row, id) in cfg.map.ids().iter().enumerate() {
            assert_eq!(
                machine.get(*id, reg_for(RegId::GoalPosition)),
                None,
                "servo {id} kept a goal written while it was limp"
            );
            assert_eq!(
                machine.get(*id, reg_for(RegId::TorqueEnable)),
                Some(&[1u8][..]),
                "servo {id} holds torque"
            );
            let armed = JointId::from_index(row)
                .and_then(|joint| summary.engage.armed.joints.get(joint))
                .expect("the bus rows are the nine joints");
            let measured = JointId::from_index(row)
                .and_then(|joint| summary.posture.present.get(joint))
                .expect("the bus rows are the nine joints");
            assert!((armed - measured).abs() < 1e-12, "servo {id}");
        }
    }

    /// A second run in one power cycle passes, over the machine the first one
    /// left behind.
    ///
    /// A bench command is a fresh process every time, so this is the ordinary
    /// case and not an unusual one: the second run finds the RAM the first wrote
    /// — the configured profile in the profile registers — and every servo still
    /// holding torque. Nothing about that is a disagreement with the platform's
    /// provisioning, and a sweep that judged those registers would refuse every
    /// command after the first until somebody power-cycled the unit.
    #[test]
    fn a_second_arm_in_one_power_cycle_passes_over_the_machine_the_first_left() {
        let cfg = resolved();
        let registers = Rc::new(RefCell::new(machine_at(&datumed_config(), &stow_legs())));

        let drive_one = |registers: &Rc<RefCell<FakeMachine>>| {
            let mut bus = Bus::new(Spy::sharing(Rc::clone(registers)), cfg.timing);
            let mut clock = TestClock::default();
            arm_over(&cfg, &mut bus, &mut clock, &mut |_| {})
        };

        drive_one(&registers).expect("a correct machine engages");

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

        let second = drive_one(&registers).expect("a second run in one power cycle passes");
        for id in cfg.map.ids() {
            assert_eq!(
                registers.borrow().get(id, reg_for(RegId::TorqueEnable)),
                Some(&[1u8][..]),
                "servo {id} is still holding"
            );
        }
        assert_eq!(
            second.engage.armed.joints, second.posture.present,
            "the pose the tick starts from is the pose that was read back"
        );
    }

    /// An antenna already flagging when the machine is found costs the pair,
    /// not the wake.
    ///
    /// The residue of an interference incident is a latched overload on one
    /// antenna. The head engages around it: that antenna is never torqued and
    /// never commanded, and its bits — latched, so they read back on every
    /// health poll for the rest of the session — raise nothing.
    #[test]
    fn an_antenna_flagging_at_engage_costs_the_pair_and_not_the_wake() {
        let cfg = resolved();
        let flagged = cfg.map.ids()[7];
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        machine.set(flagged, reg_for(RegId::HardwareErrorStatus), &[0x20]);
        let mut bench = armed(&cfg, machine);

        assert!(bench.state.masked().contains(JointId::AntennaRight));
        assert!(!bench.state.masked().contains(JointId::AntennaLeft));
        {
            let registers = bench.registers.borrow();
            // Nothing at all was written there: the register holds no value
            // because no enable was ever addressed to that servo.
            assert_eq!(
                registers.get(flagged, reg_for(RegId::TorqueEnable)),
                None,
                "a joint out of service was written an enable"
            );
            for id in cfg.map.ids().iter().take(7) {
                assert_eq!(
                    registers.get(*id, reg_for(RegId::TorqueEnable)),
                    Some(&[1u8][..]),
                    "servo {id} holds the head"
                );
            }
        }

        // A sweep of both antennas: the one still in service makes it, and the
        // other is not written to on any period of the run.
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();
        let sweeping = MotionCommand::MoveTo {
            target: JointTargets {
                antennas: [1.2, 1.2],
                ..JointTargets::default()
            },
            durations: cfg.up_durations(),
            warp: Warp::MinJerk,
        };
        let (outcome, events) = run(&mut bench, &mut pump, sweeping, &mut clock);
        let summary = outcome.expect("a pair short one antenna still runs the move");
        assert_eq!(summary.unsettled, None, "{summary:?}");
        assert!(
            !events
                .iter()
                .any(|event| matches!(event, TickEvent::AntennasDegraded(_))),
            "the bits were judged at the gate, not raised again: {events:?}"
        );
        for addressed in bench.addressed.borrow().iter() {
            assert!(
                !addressed.contains(&flagged),
                "servo {flagged} was commanded: {addressed:?}"
            );
        }
        assert!(
            (bench.state.last_goal().antennas[1] - 1.2).abs() < 1e-9,
            "{:?}",
            bench.state.last_goal()
        );
    }

    /// A joint the engagement arrived masked commands nothing on the run's
    /// first period either, and the trace says so.
    ///
    /// The released set trails the tick's mask by one period on purpose — the
    /// period a mask goes up is the period whose read diagnosed it, and its
    /// goal cell is that diagnosis. A joint masked *before* the run has no such
    /// period: nothing commanded it, so a goal cell on row zero is a command
    /// that never went out, and every lag figure drawn from that row is against
    /// a phantom.
    #[test]
    fn a_joint_masked_before_the_run_is_commanded_nothing_from_the_first_period() {
        let cfg = resolved();
        let flagged = cfg.map.ids()[7];
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        machine.set(flagged, reg_for(RegId::HardwareErrorStatus), &[0x20]);
        let mut bench = armed(&cfg, machine);
        assert!(bench.state.masked().contains(JointId::AntennaRight));

        let mut pump = pump(&cfg, bench.held);
        pump.record_trace(true);
        let mut clock = TestClock::default();
        let sweeping = MotionCommand::MoveTo {
            target: JointTargets {
                antennas: [1.2, 1.2],
                ..JointTargets::default()
            },
            durations: cfg.up_durations(),
            warp: Warp::MinJerk,
        };
        run(&mut bench, &mut pump, sweeping, &mut clock)
            .0
            .expect("a pair short one antenna still runs the move");

        let trace = pump.last_trace();
        let first = trace.first().expect("the run recorded its periods");
        assert!(
            first.released.contains(JointId::AntennaRight),
            "the first period claims a goal for a joint the run never commanded"
        );
        assert!(
            trace
                .iter()
                .all(|sample| sample.released.contains(JointId::AntennaRight)),
            "and so does every period after it"
        );
        assert!(
            !first.released.contains(JointId::AntennaLeft),
            "the side still in service is commanded"
        );
    }

    /// A period that carried both a degrade and a raise is answered by the
    /// raise.
    ///
    /// An antenna's window running out on the period a leg's error bits are
    /// read leaves the tick reporting both, with all three joints newly masked.
    /// The maneuver that answers the leg is the one that will actually run, so
    /// it is the one the record opens: attributing the release to the antennas
    /// would close it on the same period the write lands and leave the
    /// wind-down opening a second answer to one incident.
    #[test]
    fn a_degrade_and_a_raise_on_one_period_open_the_maneuver_that_runs() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut report = TickOutputs::default().report;
        report.degraded = Some(Fault::AntennaObstructed {
            joint: JointId::AntennaRight,
            error: 0.5,
        });
        report.fault = Some(Fault::HeadServoFault {
            joint: JointId::Leg(3),
            id: 14,
            bits: 0x20,
        });
        for joint in [JointId::AntennaRight, JointId::AntennaLeft, JointId::Leg(3)] {
            report.newly_masked.insert(joint);
        }

        let maneuver = record_mask_entry(&mut bench.state, &report, Duration::from_millis(40));

        assert_eq!(maneuver, Some(Maneuver::MaskedSlowStow));
        assert_eq!(
            bench.state.timeline().open_maneuver(),
            Some(Maneuver::MaskedSlowStow),
            "the answer stays open for the wind-down to close"
        );
        assert!(
            matches!(
                bench.state.timeline().entries(),
                [Entry::Response {
                    maneuver: Maneuver::MaskedSlowStow,
                    outcome: Outcome::Started,
                    ..
                }]
            ),
            "{:?}",
            bench.state.timeline().entries()
        );
    }

    /// A fault the tick already recorded is not recorded a second time, and a
    /// condition only this layer can see is not lost.
    ///
    /// The whole of the dedupe contract, from both sides. A `Fault` ending came
    /// out of the tick's own raise, which appended it where it happened; every
    /// other ending that names a condition found it on the wire, and whoever
    /// handles that ending is the only chance the record has of hearing about
    /// it.
    #[test]
    fn only_the_conditions_the_tick_never_saw_are_left_to_record() {
        let context = StepContext::reg(SeqStep::PinAndEnable, 13, RegId::TorqueEnable);
        let waited = Duration::from_millis(5);

        for raised in [
            Fault::HeadServoFault {
                joint: JointId::Leg(2),
                id: 13,
                bits: 0x20,
            },
            Fault::HeadObstructed {
                joint: JointId::BodyYaw,
                error: 0.4,
            },
            Fault::PositionFeedbackLost { misses: 51 },
        ] {
            let ending = PumpError::Fault(raised);
            for phase in [Phase::PreTorque, Phase::UnderTorque] {
                assert_eq!(ending.fault(phase), Some(raised), "{ending}");
                assert_eq!(
                    ending.unrecorded_fault(phase),
                    None,
                    "the tick recorded this one on the period it raised it: {ending}"
                );
            }
        }

        // Found on the wire, under torque, and nowhere else.
        for wire in [
            PumpError::Sequence(SeqError::NoAnswer { context }),
            PumpError::Bus {
                id: 13,
                source: XactError::Timeout { id: 13, waited },
            },
        ] {
            assert_eq!(
                wire.unrecorded_fault(Phase::UnderTorque),
                wire.fault(Phase::UnderTorque),
                "{wire}"
            );
            assert_eq!(
                wire.unrecorded_fault(Phase::PreTorque),
                None,
                "nothing was energized, so nothing is a condition of the machine: {wire}"
            );
        }
        assert!(matches!(
            PumpError::Sequence(SeqError::NoAnswer { context })
                .unrecorded_fault(Phase::UnderTorque),
            Some(Fault::BusFailure { .. })
        ));
        assert_eq!(
            PumpError::TorqueOffUnacked { id: 13 }.unrecorded_fault(Phase::UnderTorque),
            Some(Fault::TorqueOffUnconfirmed { id: 13 })
        );
        // A defect of ours names no condition at all, in either phase.
        for phase in [Phase::PreTorque, Phase::UnderTorque] {
            assert_eq!(
                PumpError::Runaway { budget: 10 }.unrecorded_fault(phase),
                None
            );
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
        let mut clock = TestClock::default();
        arm_over(&cfg, &mut bus, &mut clock, &mut |_| {}).expect("a correct machine engages");

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
        let mut clock = TestClock::default();
        let refused = arm_over(&cfg, &mut bus, &mut clock, &mut |_| {})
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
        let mut clock = TestClock::default();
        let refused = arm_over(&cfg, &mut bus, &mut clock, &mut |_| {})
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
        let summary = outcome.expect("a correct machine engages");
        let printed = format!(
            "{}{}",
            engage_report(&summary.engage),
            commission_report(&summary.commission)
        );

        for expected in [
            "found",
            "armed",
            "pull-in",
            "torque-on",
            "models",
            "supply",
            "health",
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
        // Nothing was out of service, and a line saying so would be noise on
        // every ordinary run.
        assert!(
            !printed.contains("degraded"),
            "a healthy machine reports no degradation:\n{printed}"
        );

        // The one run where it is not noise: the operator is told which joint
        // is not going to move today, at the moment nothing raised it.
        let mut flagging = machine_at(&datumed_config(), &stow_legs());
        flagging.set(18, reg_for(RegId::HardwareErrorStatus), &[0x20]);
        let (outcome, _, _) = arm(&cfg, flagging);
        let printed = engage_report(&outcome.expect("the head engages").engage);
        let named = printed
            .lines()
            .find(|line| line.starts_with("degraded"))
            .expect("the degraded line is printed");
        assert!(named.contains("left antenna"), "{named}");
        assert!(!named.contains("right antenna"), "{named}");
    }

    /// The report over a machine whose numbers differ from each other and from
    /// zero: the parked antennas of a platform found as its own stow left it.
    ///
    /// The fixture above cannot see a units error, a left-for-right swap or a
    /// line rendered off the wrong record, because every number in it is zero
    /// and zero prints the same in every unit. This one is a machine resting
    /// where the pin has real work to do, with the antennas parked past the half
    /// turn where run 1 found them.
    #[test]
    fn the_report_carries_the_numbers_the_operator_judges_by() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &rest_legs());
        // Run 1's own readings: the two antennas rest on opposite sides of the
        // wrap, both past the half turn.
        machine.set(17, reg_for(RegId::PresentPosition), &38i32.to_le_bytes());
        machine.set(18, reg_for(RegId::PresentPosition), &4051i32.to_le_bytes());
        let (outcome, _, _) = arm(&cfg, machine);
        let summary = outcome.expect("a parked antenna is not a refusal");
        let printed = format!(
            "{}{}",
            engage_report(&summary.engage),
            commission_report(&summary.commission)
        );

        // The pulls are the legs' own, in the legs' own order, and not a row of
        // zeros: a report that lost the pin sweep would print six of those.
        let lines: Vec<&str> = printed.lines().collect();
        let found = lines
            .iter()
            .position(|line| line.starts_with("found"))
            .expect("a found record");
        let armed = lines
            .iter()
            .position(|line| line.starts_with("armed"))
            .expect("an armed record");
        let pulls = lines
            .iter()
            .find(|line| line.starts_with("pull-in"))
            .expect("a pull-in line");
        assert!(
            !pulls.contains("worst 0.000"),
            "this machine's legs were pulled:\n{printed}"
        );

        // The antennas read the same on both, right then left, in degrees — the
        // order the goals go out in and the order the servos sit in on the bus.
        // Nothing pulls an antenna, so the armed record is the measurement.
        for line in [lines[found + 1], lines[armed + 1]] {
            assert!(
                line.contains("antennas [-176.660, 176.045] deg"),
                "an antenna is pinned where it was found:\n{printed}"
            );
        }
        assert!(
            printed.contains(&format!(
                "clearance {:.3} mm",
                summary.engage.armed.min_margin * 1000.0
            )),
            "the armed clearance is printed in millimetres:\n{printed}"
        );
        assert!(
            printed.contains(&format!(
                "head {:.4} m",
                summary.engage.armed.head_pose_body.translation.z
            )),
            "the armed height is printed in metres:\n{printed}"
        );
        // The legs are the joints a pull-in line has anything to say about, and
        // four of them were pulled.
        assert!(
            printed.contains("legs [7.559, 2.461, 0.000, 0.000, 0.791, 10.547] deg, worst 10.547"),
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
        let mut clock = TestClock::default();
        let summary =
            arm_over(cfg, &mut bus, &mut clock, &mut |_| {}).expect("the fixture machine engages");

        // The torque-on path's own traffic is not this half's subject.
        log.borrow_mut().clear();
        grouped.borrow_mut().clear();
        addressed.borrow_mut().clear();
        Bench {
            bus,
            state: MotionState::new_armed(&summary.engage.armed, summary.engage.degraded),
            held: summary.engage.armed.joints,
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
    ///
    /// A servo whose goal register is absent is holding where it stands: its
    /// present position is the goal it is on.
    fn goals_match(cfg: &Resolved, bench: &Bench, goal: &JointVector) {
        let machine = bench.registers.borrow();
        for (row, id) in cfg.map.ids().iter().enumerate() {
            let joint = JointId::ALL[row];
            let held = machine
                .get(*id, reg_for(RegId::GoalPosition))
                .or_else(|| machine.get(*id, reg_for(RegId::PresentPosition)))
                .expect("every servo has a position");
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
        MotionPump::new(
            &cfg.motion,
            &cfg.map,
            cfg.tick_hz,
            cfg.health_poll_hz,
            held,
            cfg.settle,
        )
        .expect("the configured rates and settle policy are positive")
    }

    /// The move every `up` command makes: stow to neutral, over the configured
    /// duration.
    fn to_neutral(cfg: &Resolved) -> MotionCommand {
        MotionCommand::MoveTo {
            target: JointTargets::default(),
            durations: cfg.up_durations(),
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

    /// Run `command` on `bench`, letting `retarget` replace it mid-travel.
    fn run_retargeting(
        bench: &mut Bench,
        pump: &mut MotionPump<'_>,
        command: MotionCommand,
        clock: &mut dyn Clock,
        retarget: &mut dyn FnMut() -> Option<MotionCommand>,
    ) -> (Result<MoveSummary, PumpError>, Vec<TickEvent>) {
        let mut events = Vec::new();
        let outcome = pump.run_retargeting(
            &mut bench.bus,
            &mut bench.state,
            command,
            clock,
            &mut |event| events.push(event),
            retarget,
        );
        (outcome, events)
    }

    /// How many control periods a span of travel occupies at `tick_hz`.
    fn periods_for(span: Duration, tick_hz: u32) -> u64 {
        let period = Duration::from_secs(1) / tick_hz;
        u64::try_from(span.as_nanos() / period.as_nanos()).expect("a small count")
    }

    /// The move back down: the stow pose, over the configured fold clock.
    fn to_stow(cfg: &Resolved) -> MotionCommand {
        MotionCommand::MoveTo {
            target: crate::commands::stow_pose_targets(),
            durations: cfg.stow_durations(),
            warp: Warp::MinJerk,
        }
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
        let periods = periods_for(cfg.up_duration, cfg.tick_hz);
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

    /// A yaw-only move off wherever the machine is standing, over `span`.
    ///
    /// One joint moves, which is what makes a settle readable: the other eight
    /// are already on their goals, so what the run waits for is the one servo
    /// the case is about.
    fn yaw_by(state: &MotionState, degrees: f64, span: Duration) -> MotionCommand {
        let mut target = *state.last_targets();
        target.body_yaw = degrees.to_radians();
        MotionCommand::MoveTo {
            target,
            durations: MoveDurations::uniform(span),
            warp: Warp::MinJerk,
        }
    }

    /// The servo a joint's goals go to.
    fn servo_for(cfg: &Resolved, joint: JointId) -> u8 {
        let row = joint.index().expect("a named joint has a bus row");
        cfg.map.ids()[row]
    }

    /// How far a servo is measured from the goal it holds, radians.
    fn error_at(cfg: &Resolved, bench: &Bench, joint: JointId) -> f64 {
        let row = joint.index().expect("a named joint has a bus row");
        let id = servo_for(cfg, joint);
        let machine = bench.registers.borrow();
        let counts = |reg| {
            let bytes: [u8; 4] = machine
                .get(id, reg_for(reg))
                .expect("the servo has the register")
                .try_into()
                .expect("a position is four bytes");
            cfg.map
                .present_rad(row, i32::from_le_bytes(bytes))
                .expect("a stored count places")
        };
        (counts(RegId::PresentPosition) - counts(RegId::GoalPosition)).abs()
    }

    /// A move is not over when the last goal goes out. The loop keeps reading
    /// until the machine is measured at the goal, and reports both instants.
    ///
    /// The case a hardware run showed: a servo still closing on a goal that
    /// stopped moving, while the run that commanded it had already declared
    /// itself finished. The gap between the two instants is the settle, and a
    /// run that ended at the first is a run whose elapsed time described the
    /// trajectory rather than the machine.
    #[test]
    fn a_move_ends_when_the_machine_arrives_and_not_when_commanding_does() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        // A count a period: slower than the goal moves, so the servo is behind
        // when commanding ends and takes periods of its own to close.
        bench
            .registers
            .borrow_mut()
            .creep
            .insert(servo_for(&cfg, JointId::BodyYaw), 1);
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let command = yaw_by(&bench.state, 6.0, Duration::from_millis(500));
        let (outcome, events) = run(&mut bench, &mut pump, command, &mut clock);
        let summary = outcome.expect("a servo closing on its goal arrives");

        let commanded = summary.commanded.expect("the trajectory finished");
        let settled = summary.settled.expect("and the machine got there");
        assert!(
            settled > commanded,
            "the settle took periods of its own: {summary:?}"
        );
        assert_eq!(summary.unsettled, None, "{summary:?}");
        assert_eq!(
            summary.elapsed, settled,
            "the run ended on the period that measured it there: {summary:?}"
        );
        assert!(
            events.iter().any(|event| matches!(
                event,
                TickEvent::Settled { after } if *after == settled - commanded
            )),
            "{events:?}"
        );
        assert!(
            !events.iter().any(|e| matches!(e, TickEvent::Faulted(_))),
            "{events:?}"
        );
        assert!(
            error_at(&cfg, &bench, JointId::BodyYaw) <= cfg.settle.tolerance,
            "the machine really is within the tolerance it was waited on"
        );
    }

    /// A joint that never arrives ends the move at the settle window with that
    /// reported, and the run is a run that finished rather than one that failed.
    ///
    /// The distinction matters at the layer above: a fault takes the machine
    /// straight to torque off, and a servo sitting a few degrees from its goal
    /// is not that. The window is what stops the loop turning forever over a
    /// stalled motor.
    #[test]
    fn a_joint_that_never_arrives_ends_the_move_at_the_window() {
        let mut cfg = resolved();
        cfg.settle.timeout = Duration::from_millis(400);
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        bench
            .registers
            .borrow_mut()
            .stalled
            .push(servo_for(&cfg, JointId::BodyYaw));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let command = yaw_by(&bench.state, 6.0, Duration::from_millis(500));
        let (outcome, events) = run(&mut bench, &mut pump, command, &mut clock);
        let summary = outcome.expect("a joint short of its goal is a report, not a refusal");

        let commanded = summary.commanded.expect("the trajectory finished");
        assert_eq!(summary.settled, None, "{summary:?}");
        let (joint, error) = summary.unsettled.expect("the window ran out on a joint");
        assert_eq!(joint, JointId::BodyYaw);
        assert!(
            (error - 6f64.to_radians()).abs() < 1e-3,
            "the whole commanded travel is what it never made: {error}"
        );
        assert!(
            summary.elapsed >= commanded + cfg.settle.timeout,
            "the window was spent before the run gave up: {summary:?}"
        );
        assert!(
            events
                .iter()
                .any(|event| matches!(event, TickEvent::Unsettled { .. })),
            "{events:?}"
        );
        assert!(
            !events.iter().any(|e| matches!(e, TickEvent::Faulted(_))),
            "a joint that did not arrive is not a fault: {events:?}"
        );
    }

    /// A recorded run keeps one sample per period: the nine measured angles,
    /// the nine goals they were measured against, and which side of the end of
    /// commanding the period fell.
    #[test]
    fn a_recorded_run_keeps_every_period_it_turned() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        bench
            .registers
            .borrow_mut()
            .creep
            .insert(servo_for(&cfg, JointId::BodyYaw), 1);
        let mut pump = pump(&cfg, bench.held);
        pump.record_trace(true);
        let mut clock = TestClock::default();

        let command = yaw_by(&bench.state, 6.0, Duration::from_millis(500));
        let (outcome, _) = run(&mut bench, &mut pump, command, &mut clock);
        let summary = outcome.expect("a servo closing on its goal arrives");

        let trace = pump.last_trace();
        assert_eq!(
            trace.len() as u64,
            summary.ticks,
            "a sample for every period the run turned"
        );
        for (index, sample) in trace.iter().enumerate() {
            assert_eq!(sample.tick as usize, index, "periods in order");
            assert!(sample.present.is_some(), "every period read the machine");
        }
        assert_eq!(trace[0].at, Duration::ZERO);
        assert_eq!(
            trace.last().expect("the run turned periods").at,
            summary.elapsed
        );

        // The phases either side of commanding ending, and the boundary between
        // them where the summary says it is.
        let commanded = summary.commanded.expect("the trajectory finished");
        assert!(!trace[0].settling, "the first period was commanding");
        let settling: Vec<&TickSample> = trace.iter().filter(|s| s.settling).collect();
        assert!(!settling.is_empty(), "and some periods were the settle");
        for sample in settling {
            assert!(sample.at > commanded, "{sample:?}");
        }

        // The body yaw's goal ends where the move sent it, and the last
        // measurement of it is within the tolerance the run waited on.
        let last = trace.last().expect("the run turned periods");
        let present = last.present.expect("the last period read the machine");
        let error = (present.body_yaw - last.goal.body_yaw).abs();
        assert!(error <= cfg.settle.tolerance, "{last:?}");
    }

    /// A run that records nothing keeps nothing, and a run recorded after one
    /// that was not is not carrying the other's periods.
    #[test]
    fn an_unrecorded_run_keeps_no_periods() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, _) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        outcome.expect("the move runs");
        assert!(pump.last_trace().is_empty());

        pump.record_trace(true);
        let (outcome, _) = hold(
            &mut bench,
            &mut pump,
            Duration::from_millis(200),
            &mut clock,
        );
        let summary = outcome.expect("the hold runs");
        assert_eq!(pump.last_trace().len() as u64, summary.ticks);
        // A hold commands nothing, so no period of one is ever a settle.
        assert!(pump.last_trace().iter().all(|sample| !sample.settling));
        assert_eq!(summary.commanded, None, "{summary:?}");
        assert_eq!(summary.settled, None, "{summary:?}");

        // And switching recording off empties what it kept: the samples a
        // caller can still reach after that would be a run it did not ask
        // about.
        pump.record_trace(false);
        assert!(pump.last_trace().is_empty(), "the buffer went with it");
    }

    /// The period a run faults on is in the trace.
    ///
    /// A tracking fault is the one the trace exists to explain, and the
    /// measurement that crossed the line is the last period before the loop
    /// stops. A trace ending one period short of it costs the reader the whole
    /// diagnosis — and disagrees by one with the period count the summary
    /// reports for the same run.
    #[test]
    fn a_faulted_run_keeps_the_period_it_faulted_on() {
        let mut cfg = resolved();
        cfg.motion.tracking.threshold_rad = 0.02;
        cfg.motion.tracking.ticks = 5;
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        bench.registers.borrow_mut().stalled = cfg.map.ids().to_vec();
        let mut pump = pump(&cfg, bench.held);
        pump.record_trace(true);
        let mut clock = TestClock::default();

        let (outcome, _) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("a machine that never moves is not tracking");
        assert!(
            matches!(error, PumpError::Fault(Fault::HeadObstructed { .. })),
            "{error}"
        );

        let summary = pump.last_summary();
        let trace = pump.last_trace();
        assert_eq!(
            trace.len() as u64,
            summary.ticks,
            "a sample for every period the run turned, faulting period included"
        );
        // The kept period is the one that faulted: its measurement is the goal
        // it never followed, past the threshold the run was watching.
        let last = trace.last().expect("the run turned periods");
        let present = last.present.expect("the faulting period read the machine");
        let lag = present
            .joints()
            .into_iter()
            .zip(last.goal.joints())
            .map(|((_, angle), (_, goal))| (angle - goal).abs())
            .fold(0.0_f64, f64::max);
        assert!(
            lag > cfg.motion.tracking.threshold_rad,
            "the period the fault was raised on: {last:?}"
        );
    }

    /// The trace buffer's ceiling stops the recording and nothing else: it is
    /// announced once, and the run carries on to its endpoint.
    ///
    /// The ceiling is what stands between a run with no length of its own — one
    /// steered by a caller that keeps retargeting — and a host filling its
    /// memory with periods nobody will read.
    #[test]
    fn a_trace_that_fills_stops_recording_and_says_so_once() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let ceiling = 7;
        pump.record_trace(true);
        pump.trace_ceiling(ceiling);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let summary = outcome.expect("a full trace buffer is not a failed move");

        assert_eq!(pump.last_trace().len(), ceiling, "recording stopped at it");
        assert!(
            summary.ticks > ceiling as u64,
            "and the periods carried on: {summary:?}"
        );
        let full: Vec<&TickEvent> = events
            .iter()
            .filter(|event| matches!(event, TickEvent::TraceFull { .. }))
            .collect();
        assert_eq!(
            full,
            vec![&TickEvent::TraceFull { samples: ceiling }],
            "announced once, on the period that reached it: {events:?}"
        );
    }

    /// A caller that steers the run after the machine has arrived gets a
    /// summary about the goal it steered to, not the one it abandoned.
    ///
    /// `retarget` is asked during the settle as well as during travel, and a
    /// replacement taken there puts the run back to commanding. What the run
    /// had measured about arriving belongs to a goal it is no longer heading
    /// for, and a caller reading `commanded`/`settled` off that summary would
    /// be reading instants from a move that was abandoned.
    #[test]
    fn a_move_replaced_during_its_settle_reports_the_replacement() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        // A count a period: the machine is still closing on its goal when
        // commanding ends, so the settle spans periods a replacement can
        // arrive in.
        bench
            .registers
            .borrow_mut()
            .creep
            .insert(servo_for(&cfg, JointId::BodyYaw), 1);
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let span = Duration::from_millis(500);
        let first = yaw_by(&bench.state, 6.0, span);
        let second = yaw_by(&bench.state, 3.0, span);
        let arrived = Cell::new(false);
        let mut events = Vec::new();
        let mut steered = false;
        let outcome = pump.run_retargeting(
            &mut bench.bus,
            &mut bench.state,
            first,
            &mut clock,
            &mut |event| {
                if event == TickEvent::Completed {
                    arrived.set(true);
                }
                events.push(event);
            },
            &mut || {
                // Only once the first move has finished commanding, which is
                // what puts the replacement inside the settle.
                (arrived.get() && !std::mem::replace(&mut steered, true)).then_some(second)
            },
        );
        let summary = outcome.expect("the replacement carries to its own endpoint");

        // Two commands and two completions: the replacement was taken after the
        // first move had finished commanding, which is the settle phase and not
        // travel — so the tick starts it rather than splicing it.
        let started: Vec<usize> = events
            .iter()
            .enumerate()
            .filter(|(_, event)| **event == TickEvent::Command(CommandDisposition::Started))
            .map(|(at, _)| at)
            .collect();
        let completed: Vec<usize> = events
            .iter()
            .enumerate()
            .filter(|(_, event)| **event == TickEvent::Completed)
            .map(|(at, _)| at)
            .collect();
        assert_eq!(started.len(), 2, "{events:?}");
        assert_eq!(completed.len(), 2, "{events:?}");
        assert!(started[1] > completed[0], "{events:?}");

        let commanded = summary.commanded.expect("the replacement finished too");
        assert!(
            commanded > span,
            "the instants are the replacement's, not the abandoned move's: {summary:?}"
        );
        let settled = summary.settled.expect("and the machine got there");
        assert!(settled >= commanded, "{summary:?}");
        assert_eq!(summary.unsettled, None, "{summary:?}");
        assert!(
            (bench.state.last_targets().body_yaw - 3f64.to_radians()).abs() < 1e-9,
            "the run ended on the replacement's endpoint: {:?}",
            bench.state.last_targets().body_yaw
        );
    }

    /// A settle window longer than the loop's stall margin is a machine taking
    /// its time, not a loop that hung.
    ///
    /// The budget left when commanding ends was sized for travel that is over.
    /// Without the window getting periods of its own, a configuration whose
    /// settle window outlasts the stall margin would end every unsettled run as
    /// `Stalled` — a loop fault reported for a servo that was merely slow.
    #[test]
    fn a_settle_window_past_the_stall_margin_still_reports_the_joint() {
        let mut cfg = resolved();
        cfg.settle.timeout = Duration::from_secs(STALL_MARGIN_SECS + 1);
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        bench
            .registers
            .borrow_mut()
            .stalled
            .push(servo_for(&cfg, JointId::BodyYaw));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let command = yaw_by(&bench.state, 6.0, Duration::from_millis(500));
        let (outcome, _) = run(&mut bench, &mut pump, command, &mut clock);
        let summary = outcome.expect("a joint short of its goal is a report, not a stall");

        let (joint, _) = summary.unsettled.expect("the window ran out on a joint");
        assert_eq!(joint, JointId::BodyYaw);
        assert!(
            summary.elapsed >= cfg.settle.timeout,
            "the whole window was spent, past the margin the travel carried: {summary:?}"
        );
    }

    /// A run whose reads fall away across the end of its settle window ends
    /// with no verdict at all, and says so by having none.
    ///
    /// A blind period judges nothing, so a dropout straddling the window
    /// boundary leaves a run that neither arrived nor was found short. It is
    /// `Ok` with `commanded` set and both verdicts empty, which is what a
    /// caller reading `unsettled.is_none()` as arrival would get wrong.
    #[test]
    fn a_run_blind_at_the_end_of_its_window_returns_no_verdict() {
        let mut cfg = resolved();
        cfg.settle.timeout = Duration::from_millis(200);
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        bench
            .registers
            .borrow_mut()
            .stalled
            .push(servo_for(&cfg, JointId::BodyYaw));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        // The dropout starts when commanding ends and outlasts the window,
        // while staying well inside the read-loss budget the tick faults on.
        let outage = 30;
        assert!(outage < cfg.motion.read_loss_ticks, "inside the budget");
        let registers = Rc::clone(&bench.registers);
        let position = reg_for(RegId::PresentPosition).addr;
        let servo = servo_for(&cfg, JointId::BodyYaw);
        let mut events = Vec::new();
        let command = yaw_by(&bench.state, 6.0, Duration::from_millis(500));
        let outcome = pump.run(
            &mut bench.bus,
            &mut bench.state,
            command,
            &mut clock,
            &mut |event| {
                if event == TickEvent::Completed {
                    registers
                        .borrow_mut()
                        .mute
                        .insert((servo, position), outage);
                }
                events.push(event);
            },
        );
        let summary = outcome.expect("a run that measured nothing at the end is not a failure");

        assert!(summary.commanded.is_some(), "{summary:?}");
        assert_eq!(summary.settled, None, "nothing measured it arrived");
        assert_eq!(summary.unsettled, None, "and nothing measured it short");
        assert!(
            summary.misses > 0,
            "the window was spent blind: {summary:?}"
        );
        assert!(
            !events.iter().any(|e| matches!(e, TickEvent::Faulted(_))),
            "{events:?}"
        );
        assert!(
            !events
                .iter()
                .any(|e| matches!(e, TickEvent::Settled { .. } | TickEvent::Unsettled { .. })),
            "no verdict was announced either: {events:?}"
        );
    }

    /// A machine that follows its goals a few periods behind carries the move
    /// through, however far behind that puts it.
    ///
    /// Nine servos on a proportional loop, each sitting behind a streamed goal
    /// in proportion to how fast the goal is moving. The paired run is the
    /// control — the same machine, the same threshold, and a progress minimum
    /// nothing can meet — and it faults, which is what says the lag really did
    /// clear the threshold rather than the first run passing for want of
    /// anything to detect.
    #[test]
    fn a_machine_chasing_its_goals_carries_the_move() {
        let mut cfg = resolved();
        cfg.motion.tracking.threshold_rad = 0.02;
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        for id in cfg.map.ids() {
            bench.registers.borrow_mut().delay.insert(id, 4);
        }
        let mut driver = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut driver, to_neutral(&cfg), &mut clock);
        outcome.expect("a joint closing on its goal is a joint following it");
        assert!(events.contains(&TickEvent::Completed));
        assert!(
            !events.iter().any(|e| matches!(e, TickEvent::Faulted(_))),
            "{events:?}"
        );

        cfg.motion.tracking.progress_min_rad = 1.0;
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        for id in cfg.map.ids() {
            bench.registers.borrow_mut().delay.insert(id, 4);
        }
        let mut driver = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, _) = run(&mut bench, &mut driver, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("nothing closes a whole radian in a window");
        assert!(
            matches!(error, PumpError::Fault(Fault::HeadObstructed { .. })),
            "{error}"
        );
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

        // One record of the run, whichever way it is read: the value handed
        // back is the value kept, so the bench's printed line and a caller's
        // own copy cannot describe different runs.
        assert_eq!(summary, pump.last_summary(), "{summary:?}");
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
                durations: cfg.up_durations(),
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
                durations: cfg.up_durations(),
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
                .expect("every leg was commanded");
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

    /// The stow-to-neutral move on a clock far too short for it runs anyway, on
    /// a clock the loop stretched to fit, and says so once before the first
    /// period.
    ///
    /// Unfloored this is the fault that de-torques mid-travel. The loop is
    /// where the stretch lands because the loop is what reads a command first.
    #[test]
    fn a_command_too_short_for_its_span_runs_on_a_stretched_clock() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let asked = Duration::from_millis(200);
        let (outcome, events) = run(
            &mut bench,
            &mut pump,
            MotionCommand::MoveTo {
                target: JointTargets::default(),
                durations: MoveDurations::uniform(asked),
                warp: Warp::MinJerk,
            },
            &mut clock,
        );
        let summary = outcome.expect("a stretched fold reaches its endpoint");

        let stretch = events
            .iter()
            .find_map(|event| match event {
                TickEvent::Stretched(stretch) => Some(*stretch),
                _ => None,
            })
            .expect("a fifth of a second cannot carry the fold");
        assert_eq!(stretch.requested, MoveDurations::uniform(asked));
        // The head clock lands on the fold's exact floor, which sits inside the
        // last hundredth of a second the published figure's search stepped in.
        assert!(
            stretch.effective.head.as_secs_f64() > HEAD_GROUP_FLOOR_S - 0.01,
            "{stretch:?}"
        );
        // The stretch is announced before anything the loop did, so an operator
        // reads why the move is taking longer than the knob says.
        assert!(
            matches!(events.first(), Some(TickEvent::Stretched(_))),
            "{events:?}"
        );
        assert!(
            events.contains(&TickEvent::Completed)
                && !events.iter().any(|e| matches!(e, TickEvent::Faulted(_))),
            "{events:?}"
        );

        // And the periods it ran are the stretched clock's, not the asked-for
        // one's.
        assert!(
            summary.ticks > periods_for(asked, cfg.tick_hz),
            "{summary:?}"
        );
    }

    /// The stall budget is sized from the clock the move actually runs on.
    ///
    /// This is why the stretch belongs to the loop rather than to the tick: a
    /// budget still sized from the duration the caller named would abort the
    /// stretched move partway and report a healthy machine as one that hung.
    #[test]
    fn the_stall_budget_follows_the_stretched_clock() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);

        let asked = Duration::from_millis(200);
        let command = MotionCommand::MoveTo {
            target: JointTargets::default(),
            durations: MoveDurations::uniform(asked),
            warp: Warp::MinJerk,
        };
        let (floored, stretch) = floor_move_clock(
            &cfg.motion,
            bench.state.last_targets(),
            &command,
            f64::from(cfg.tick_hz),
        );
        assert!(stretch.is_some(), "the fixture move is the stretched one");

        // A clock that never advances never reaches the endpoint, so the run
        // ends on the budget and reports it.
        let mut clock = FrozenClock;
        let (outcome, _) = run(&mut bench, &mut pump, command, &mut clock);
        let error = outcome.expect_err("a move on a stopped clock never lands");
        let PumpError::Stalled { budget } = error else {
            panic!("{error}");
        };
        assert_eq!(budget, pump.stall_budget(&floored));
        assert!(
            budget > pump.stall_budget(&command),
            "the budget is the asked-for clock's, not the stretched one's"
        );
    }

    /// A replacement issued mid-travel goes through the same right-sizing, from
    /// the setpoint the previous period commanded.
    ///
    /// The retarget site sizes its own budget, so a replacement that skipped
    /// the pass would be the one move a stretch could not save.
    #[test]
    fn a_replacement_is_right_sized_before_its_budget_is() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        // A quarter of the way in: far enough that the replacement lands well
        // inside the first move, which itself needs no stretch.
        let replacement = Duration::from_millis(100);
        let mut periods = 0;
        let (outcome, events) = run_retargeting(
            &mut bench,
            &mut pump,
            to_neutral(&cfg),
            &mut clock,
            &mut || {
                periods += 1;
                (periods == 10).then(|| MotionCommand::MoveTo {
                    target: crate::commands::stow_pose_targets(),
                    durations: MoveDurations::uniform(replacement),
                    warp: Warp::MinJerk,
                })
            },
        );
        outcome.expect("the replacement reaches its own endpoint");

        let stretches: Vec<_> = events
            .iter()
            .filter_map(|event| match event {
                TickEvent::Stretched(stretch) => Some(*stretch),
                _ => None,
            })
            .collect();
        assert_eq!(stretches.len(), 1, "{events:?}");
        assert_eq!(stretches[0].requested, MoveDurations::uniform(replacement));
        let ClockStretch {
            requested,
            effective,
            ..
        } = stretches[0];
        assert!(effective.longest() > requested.longest(), "{stretches:?}");
        assert!(
            effective.head >= requested.head && effective.antennas >= requested.antennas,
            "a clock was shortened: {stretches:?}"
        );
        assert!(
            !events.iter().any(|e| matches!(e, TickEvent::Faulted(_))),
            "{events:?}"
        );
    }

    /// A replacement issued while the loop is running late lands like any
    /// other: right-sized, spliced from the setpoint the last period commanded,
    /// and carried to its own endpoint.
    ///
    /// The lateness cannot reach the new path either — the replacement's clock
    /// starts at zero and advances a period at a time, so the splice is the
    /// same one an on-time loop makes, arriving later.
    #[test]
    fn a_replacement_under_lateness_reaches_its_endpoint() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        // Three quarters of a period late, every period: an overrun each time,
        // and never enough to rebase the grid.
        let mut clock = LateClock {
            now: Duration::ZERO,
            late: pump.period() * 3 / 4,
        };

        let mut periods = 0;
        let (outcome, events) = run_retargeting(
            &mut bench,
            &mut pump,
            to_neutral(&cfg),
            &mut clock,
            &mut || {
                periods += 1;
                (periods == 10).then(|| to_stow(&cfg))
            },
        );
        let summary = outcome.expect("the replacement reaches its own endpoint");

        assert!(
            events.contains(&TickEvent::Command(CommandDisposition::Retargeted)),
            "{events:?}"
        );
        assert!(
            !events.iter().any(|e| matches!(e, TickEvent::Faulted(_))),
            "{events:?}"
        );
        assert!(events.contains(&TickEvent::Completed));
        assert!(matches!(bench.state.mode(), Mode::Holding));
        assert!(summary.slip > Duration::ZERO, "{summary:?}");
        // Its *own* endpoint, which is the whole claim: a replacement dropped
        // on the floor leaves the raise to finish, and everything above is
        // true of that run too. The antennas are the discriminator — by the
        // turn they are away from both poses — and the fold's sweep takes the
        // inboard arc, so the goal that arrives a turn below stow is stow.
        let ended = bench.state.last_targets().antennas;
        for (ended, stow) in ended
            .into_iter()
            .zip(crate::commands::stow_pose_targets().antennas)
        {
            let apart = (ended - stow).rem_euclid(core::f64::consts::TAU);
            assert!(
                apart < 1e-9 || core::f64::consts::TAU - apart < 1e-9,
                "the run ended at {ended} rad, not the {stow} rad it was turned to"
            );
        }
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
                // Well past the bench's body-yaw cap.
                target: JointTargets {
                    body_yaw: 3.0,
                    ..JointTargets::default()
                },
                durations: cfg.up_durations(),
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
            matches!(error, PumpError::Fault(Fault::PositionFeedbackLost { misses }) if misses == budget + 1),
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
                .write_goals(&mut bus, &goal, JointSet::EMPTY)
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
            matches!(error, PumpError::Fault(Fault::PositionFeedbackLost { misses }) if misses == outage),
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

        // The calm fold rather than the quick raise: the sweep runs at its own
        // hertz, so a run has to be long enough to hold a run of them and the
        // one that comes back.
        let (outcome, events) = run(&mut bench, &mut pump, to_stow(&cfg), &mut clock);
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

    /// A head servo's hardware error beyond the input-voltage bit stops the
    /// move and releases that servo — and only that servo. Nothing here reboots
    /// anything, and the eight that still command keep their torque: the
    /// maneuver this ending asks for is a stow driven on them.
    #[test]
    fn a_head_servo_error_stops_the_move_and_releases_that_servo() {
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
                PumpError::Fault(Fault::HeadServoFault {
                    joint: JointId::Leg(3),
                    id: 14,
                    bits: 0x20
                })
            ),
            "{error}"
        );
        assert!(
            events
                .iter()
                .any(|event| matches!(event, TickEvent::Faulted(_))),
            "{events:?}"
        );
        assert!(
            !bench.state.is_faulted() && bench.state.mode() == Mode::Holding,
            "the stow that answers this runs on this same state"
        );
        assert_eq!(
            bench.state.masked().iter().collect::<Vec<_>>(),
            vec![JointId::Leg(3)]
        );
        let machine = bench.registers.borrow();
        for (row, id) in cfg.map.ids().into_iter().enumerate() {
            let torque: &[u8] = if row == 4 { &[0] } else { &[1] };
            assert_eq!(
                machine.get(id, reg_for(RegId::TorqueEnable)),
                Some(torque),
                "servo {id}"
            );
        }
    }

    /// A release the bus will not carry ends the run, and still says which
    /// servo had flagged.
    ///
    /// "Masked" means released, not merely unspoken to. Four mechanisms start
    /// acting on that the moment the tick raises the mask — the goal frames
    /// drop the row, the settle stops waiting for it, the trace blanks its goal
    /// cell, and the tick checks it for nothing — so a release that does not
    /// land leaves a servo torqued, holding a frozen goal, that nothing is
    /// watching or commanding. The bus cannot be trusted to carry the head down
    /// either, and the ending says so.
    ///
    /// The ending names the wire, which is not the diagnosis: the servo that
    /// flagged is what a reader came for, and the fault branch that would have
    /// announced it is not reached from here.
    #[test]
    fn a_release_the_bus_will_not_carry_ends_the_run_and_names_the_fault() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let flagged = cfg.map.ids()[4];
        {
            let mut registers = bench.registers.borrow_mut();
            registers.set(flagged, reg_for(RegId::HardwareErrorStatus), &[0x20]);
            // From here the read-back of this servo's torque-off answers a byte
            // too wide: a frame that passes its own checksum and is nobody's
            // answer, so the verified write fails past every retry.
            registers
                .verbose
                .push((flagged, reg_for(RegId::TorqueEnable).addr));
        }
        let mut pump = pump(&cfg, bench.held);
        pump.record_trace(true);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("a release that cannot be written ends the run");

        assert!(
            matches!(error, PumpError::Bus { id, .. } if id == flagged),
            "{error}"
        );
        assert_eq!(
            error.class(Phase::UnderTorque),
            ErrorClass::ImmediateAllTorqueOffToPark,
            "a bus that cannot release one servo carries no maneuver"
        );
        assert!(
            events.iter().any(|event| matches!(
                event,
                TickEvent::Faulted(Fault::HeadServoFault { id, .. }) if *id == flagged
            )),
            "the servo that flagged went unreported: {events:?}"
        );
        // The run stopped where the release did — well short of the move — and
        // the period it stopped on is in the record.
        let summary = pump.last_summary();
        assert!(
            summary.ticks < periods_for(cfg.up_duration, cfg.tick_hz),
            "the run carried on past the release it could not write: {summary:?}"
        );
        assert_eq!(pump.last_trace().len() as u64, summary.ticks);
        // The maneuver the mask started did not get its write out, so the
        // record says so rather than leaving it open forever: what answers a
        // bus that will not release one servo is the immediate torque-off, and
        // that is the caller's to run.
        let entries = bench.state.timeline().entries();
        assert!(
            matches!(
                entries,
                [
                    Entry::Fault {
                        fault: Fault::HeadServoFault { .. },
                        ..
                    },
                    Entry::Response {
                        maneuver: Maneuver::MaskedSlowStow,
                        outcome: Outcome::Started,
                        ..
                    },
                    Entry::Response {
                        maneuver: Maneuver::MaskedSlowStow,
                        outcome: Outcome::FellThrough,
                        ..
                    },
                ]
            ),
            "{}",
            bench.state.timeline()
        );
        assert_eq!(bench.state.timeline().open_maneuver(), None);
    }

    /// A head servo dropping out is written down as the semi-controlled descent
    /// beginning, and the stow that follows is the rest of that one maneuver.
    #[test]
    fn a_masked_servo_starts_the_maneuver_that_answers_it() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let flagged = cfg.map.ids()[4];
        bench
            .registers
            .borrow_mut()
            .set(flagged, reg_for(RegId::HardwareErrorStatus), &[0x20]);
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, _) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("a servo dropping out ends the move");
        assert_eq!(
            error.class(Phase::UnderTorque),
            ErrorClass::MaskedSlowStowToPark
        );
        let entries = bench.state.timeline().entries();
        assert!(
            matches!(
                entries,
                [
                    Entry::Fault {
                        fault: Fault::HeadServoFault { .. },
                        ..
                    },
                    Entry::Response {
                        maneuver: Maneuver::MaskedSlowStow,
                        outcome: Outcome::Started,
                        ..
                    },
                ]
            ),
            "{}",
            bench.state.timeline()
        );
        // Still running: whatever happens next expands this rather than
        // starting a second answer.
        assert_eq!(
            bench.state.timeline().open_maneuver(),
            Some(Maneuver::MaskedSlowStow)
        );
        assert_eq!(entries[0].at(), entries[1].at(), "one period, one decision");
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
        let budget =
            periods_for(cfg.up_duration, cfg.tick_hz) + u64::from(cfg.tick_hz) * STALL_MARGIN_SECS;
        assert_eq!(pump.stall_budget(&to_neutral(&cfg)), budget);
        assert!(
            matches!(error, PumpError::Stalled { budget: ran } if ran == budget),
            "{error}"
        );
        assert_eq!(frames(&bench.log, INST_SYNC_WRITE), 0, "nothing commanded");
        // The run that ran out of periods is one whose period count is the
        // whole finding, so it is kept rather than handed back only on success.
        assert_eq!(pump.last_summary().ticks, budget);
    }

    /// The record of a run outlives whichever way the run ended, and is the run's
    /// own: a fault keeps everything the periods before it measured, and the
    /// next run starts from zero rather than from that.
    #[test]
    fn a_faulted_run_keeps_its_record_and_the_next_one_starts_clean() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        // The whole head group, which is what a hand on the head is: the six
        // cranks drive one rigid body through a parallel linkage, so a pose
        // with five legs where the plan wants them and one where it does not is
        // a pose the machine cannot hold and the measurement layer answers
        // first. Held at stow, every leg is exactly where the others say it is.
        for leg in 1..=6 {
            let stuck = cfg.map.ids()[leg];
            bench.registers.borrow_mut().stalled.push(stuck);
        }
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, _) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("a servo that takes its goals and does not move");
        assert!(
            matches!(error, PumpError::Fault(Fault::HeadObstructed { .. })),
            "{error}"
        );

        let faulted = pump.last_summary();
        assert!(faulted.ticks > 0, "{faulted:?}");
        assert!(faulted.goals > 0, "{faulted:?}");
        assert!(faulted.elapsed > Duration::ZERO, "{faulted:?}");
        assert!(
            faulted.worst_lag[2] > cfg.motion.tracking.threshold_rad,
            "the stalled leg's lag is what the fault was raised on: {faulted:?}"
        );

        // An obstruction leaves the machine commanding, so the next run is a
        // run. With the leg turning again it finishes, and what it leaves
        // behind is its own numbers rather than the faulted run's read a second
        // time.
        bench.registers.borrow_mut().stalled.clear();
        let (outcome, _) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        outcome.expect("the obstruction is gone and the machine still commands");
        let after = pump.last_summary();
        assert!(after.settled.is_some(), "{after:?}");
        assert_eq!(
            after.ticks,
            periods_for(cfg.up_duration, cfg.tick_hz) + 1,
            "its own periods, not the faulted run's counted a second time: {after:?}"
        );
    }

    /// One crank that stops while the other five follow the plan is not an
    /// obstruction: it is a set of six angles no rigid head can hold, and the
    /// measurement layer is what answers it.
    ///
    /// The six drive one body through a parallel linkage, so the pose stops
    /// solving long before the stalled leg has lagged its goal by the tracking
    /// threshold — and a frame nobody can place is a frame the tracking
    /// comparison never sees, because there is no measured pose to compare
    /// against. What ends the run is the run of unplaceable frames. Both
    /// answers end with the machine limp; this one parks rather than stowing,
    /// which is the right way round for a machine whose measurements have
    /// stopped meaning anything.
    #[test]
    fn a_single_crank_that_stops_is_a_pose_nobody_can_place() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        bench.registers.borrow_mut().stalled.push(cfg.map.ids()[2]);
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, _) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("five legs cannot hold a pose the sixth refuses");
        assert!(
            matches!(
                error,
                PumpError::Fault(Fault::MeasuredPoseInvalid { failures, .. })
                    if failures > cfg.motion.read_loss_ticks
            ),
            "{error}"
        );
        assert_eq!(
            error.class(Phase::UnderTorque),
            ErrorClass::ImmediateAllTorqueOffToPark
        );
    }

    /// A step the guard will not pass ends the run without faulting the
    /// machine: the sample never goes out, the trajectory is dropped, and the
    /// tick is left holding the last goal it wrote. The distinction is the
    /// whole point of the abort — nothing about the machine has gone wrong, so
    /// nothing about it needs to be given up.
    #[test]
    fn a_step_the_guard_stops_abandons_the_move_and_leaves_the_machine_holding() {
        let mut cfg = resolved();
        // Far under any step a shaped move takes, so the first commanded
        // period trips it.
        cfg.motion.max_step = JointStep {
            legs: 1e-5,
            body_yaw: 1e-5,
            antennas: 1e-5,
        };
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut guarded = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut guarded, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("a step past the bound is never emitted");
        assert!(
            matches!(error, PumpError::Aborted(MoveAbort::StepTooLarge { .. })),
            "{error}"
        );
        assert!(
            events
                .iter()
                .any(|event| matches!(event, TickEvent::Aborted(_))),
            "{events:?}"
        );
        assert!(!bench.state.is_faulted(), "the machine is healthy");
        assert!(matches!(bench.state.mode(), Mode::Holding));

        // Which the next command proves: the same state takes a wind-down, on
        // the bounds the shipped configuration carries.
        let relaxed = resolved();
        let mut wind_down = pump(&relaxed, *bench.state.last_goal());
        let (outcome, _) = run(&mut bench, &mut wind_down, to_neutral(&relaxed), &mut clock);
        outcome.expect("an abandoned move leaves a machine that still commands");
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

    /// A whole second lost mid-move delays the move by a second and costs it
    /// nothing else: no fault, no burst of periods run back to back to catch
    /// up, and the slip reported.
    ///
    /// The stall is the scale the control loop was measured stalling at on the
    /// machine. Replaying the missed grid slots would run fifty periods at
    /// whatever rate the bus allows, and under a move clock that advances per
    /// period that is the head crossing that stretch of its path at bus speed.
    #[test]
    fn a_stall_mid_move_delays_the_move_and_replays_nothing() {
        let cfg = resolved();
        let stall = Duration::from_secs(1);

        let mut clean_bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut clean_pump = pump(&cfg, clean_bench.held);
        let mut clean_clock = TestClock::default();
        let (outcome, _) = run(
            &mut clean_bench,
            &mut clean_pump,
            to_neutral(&cfg),
            &mut clean_clock,
        );
        let undisturbed = outcome.expect("the fixture move lands");
        assert_eq!(undisturbed.slip, Duration::ZERO, "{undisturbed:?}");

        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = StallClock {
            now: Duration::ZERO,
            stall,
            at: 10,
            woken: 0,
            behind: 0,
        };
        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let summary = outcome.expect("a stalled loop still reaches the target");

        assert!(
            !events.iter().any(|e| matches!(e, TickEvent::Faulted(_))),
            "{events:?}"
        );
        assert!(events.contains(&TickEvent::Completed));
        assert_eq!(
            events
                .iter()
                .filter(|event| matches!(event, TickEvent::Overrun { .. }))
                .count(),
            1,
            "{events:?}"
        );
        assert_eq!(
            clock.behind, 0,
            "the loop was sent back through periods that were already due"
        );
        assert_eq!(summary.slip, stall, "{summary:?}");
        // The move arrives a stall later, having run the same periods it would
        // have run without one.
        assert_eq!(summary.ticks, undisturbed.ticks, "{summary:?}");
        assert_eq!(
            summary.elapsed,
            undisturbed.elapsed + stall,
            "{summary:?} against {undisturbed:?}"
        );
    }

    /// A caller that changes its mind mid-travel gets the head turned around
    /// where it is, and the run ends at the replacement.
    ///
    /// Without mid-travel replacement the head must finish going up before it
    /// can come down — an instruction arriving mid-raise is late by a whole
    /// move. One run, one endpoint — the replacement's.
    #[test]
    fn a_move_replaced_part_way_ends_at_the_replacement() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();
        // Well into the raise and nowhere near its end: the antennas are the
        // discriminator below, and by here they are away from both poses.
        let turn_at = 40;
        let mut periods = 0;

        let (outcome, events) = run_retargeting(
            &mut bench,
            &mut pump,
            to_neutral(&cfg),
            &mut clock,
            &mut || {
                periods += 1;
                (periods == turn_at).then(|| to_stow(&cfg))
            },
        );
        let summary = outcome.expect("the replacement carries to its own endpoint");

        let retargets = events
            .iter()
            .filter(|event| **event == TickEvent::Command(CommandDisposition::Retargeted))
            .count();
        assert_eq!(retargets, 1, "{events:?}");
        assert_eq!(
            events
                .iter()
                .filter(|event| **event == TickEvent::Completed)
                .count(),
            1,
            "the first move never completed: {events:?}"
        );
        assert!(
            events
                .iter()
                .position(|event| *event == TickEvent::Completed)
                > events
                    .iter()
                    .position(|event| *event == TickEvent::Command(CommandDisposition::Retargeted)),
            "the completion is the replacement's: {events:?}"
        );
        // Compared as directions: the fold's antenna sweep takes the inboard
        // arc, so the goal that arrives at −3.05 rad is the one a turn above it.
        let ended = bench.state.last_targets().antennas;
        for (ended, stow) in ended
            .into_iter()
            .zip(crate::commands::stow_pose_targets().antennas)
        {
            let apart = (ended - stow).rem_euclid(core::f64::consts::TAU);
            assert!(
                apart < 1e-9 || core::f64::consts::TAU - apart < 1e-9,
                "the run ended commanding the pose it was turned to: {ended} against {stow}"
            );
        }
        // And it ran on the *replacement's* clock. Both halves of a replacement
        // are the caller's — targets and durations — and the fixture's calm
        // fold takes more than twice the quick raise, so a run that kept the
        // raise's clock for the fold lands sixty periods short of this.
        let turn_at = u64::try_from(turn_at).expect("a small count");
        let fold = periods_for(cfg.stow_duration, cfg.tick_hz);
        let raise = periods_for(cfg.up_duration, cfg.tick_hz);
        assert!(
            summary.ticks >= turn_at + fold && summary.ticks < turn_at + 2 * fold,
            "the fold's own clock is what ran: {} periods, turned at {turn_at}, fold {fold}, \
             raise {raise}",
            summary.ticks,
        );
    }

    /// Replacing the move for longer than the original command's own stall
    /// budget is a caller changing its mind, not a loop that hung.
    ///
    /// The budget follows the travel still ahead, so it is re-sized by every
    /// replacement. A fixed one sized off the first command would abort a run
    /// that was being steered, and report it as a stall — the one failure that
    /// says the loop stopped advancing.
    #[test]
    fn a_move_replaced_over_and_over_is_not_a_stall() {
        let cfg = resolved();
        let mut bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();
        let first = to_neutral(&cfg);
        let budget = pump.stall_budget(&first);
        let mut periods: u64 = 0;

        let (outcome, _) = run_retargeting(&mut bench, &mut pump, first, &mut clock, &mut || {
            periods += 1;
            (periods <= budget + 10).then(|| to_neutral(&cfg))
        });
        let summary = outcome.expect("a steered move is not a stalled one");

        assert!(
            periods > budget,
            "the fixture outlasted the first command's budget: {periods} against {budget}"
        );
        assert!(summary.ticks > budget, "{summary:?}");
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
        // Every servo takes its goals and stays where it is: the goal write
        // that never applied, and the stalled motor, are the same thing from
        // here.
        bench.registers.borrow_mut().stalled = cfg.map.ids().to_vec();
        let mut pump = pump(&cfg, bench.held);
        let mut clock = TestClock::default();

        let (outcome, events) = run(&mut bench, &mut pump, to_neutral(&cfg), &mut clock);
        let error = outcome.expect_err("a machine that never arrives is not tracking");

        assert!(
            matches!(error, PumpError::Fault(Fault::HeadObstructed { .. })),
            "{error}"
        );
        assert!(
            events
                .iter()
                .any(|event| matches!(event, TickEvent::Faulted(Fault::HeadObstructed { .. }))),
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
        assert_eq!(
            pump.last_summary().ticks,
            0,
            "and nothing was measured to keep"
        );
    }

    /// A wire lost in the middle of a run keeps what the run had measured.
    ///
    /// An intermittent adapter is the failure hardest to reproduce and the one
    /// whose period counts, jitter and per-joint lag are worth the most: the
    /// numbers say what the loop was doing when it went. The record is sealed
    /// on the way out rather than abandoned with the periods that filled it.
    #[test]
    fn a_bus_failure_mid_run_keeps_what_it_measured() {
        let cfg = resolved();
        let bench = armed(&cfg, machine_at(&datumed_config(), &stow_legs()));
        let mut state = bench.state;
        // Enough transactions for a few whole periods, and then nothing.
        let port = FailsAfter::new(machine_at(&datumed_config(), &stow_legs()), 12);
        let mut bus = Bus::new(port, cfg.timing);
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
            .expect_err("a port that goes away carries no move to its end");
        assert!(matches!(error, PumpError::Bus { .. }), "{error}");

        let summary = pump.last_summary();
        assert!(
            summary.ticks > 0 && summary.goals > 0,
            "the run turned periods and commanded before the wire went: {summary:?}"
        );
        assert!(
            summary.elapsed > Duration::ZERO,
            "and the elapsed time was stamped: {summary:?}"
        );
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
                cfg.settle,
            )
            .err()
            .expect("a zero rate is refused");
            assert!(
                matches!(refused, PumpError::Rate { .. }),
                "{tick_hz}/{health_poll_hz}: {refused}"
            );
        }
    }

    /// A settle policy nothing can be judged against is refused the same way a
    /// rate of zero is.
    ///
    /// Each of these fails differently if it is let through: an infinite or
    /// negative tolerance calls every joint arrived the instant commanding
    /// stops, a NaN one calls none of them arrived ever, and a window of no
    /// time gives the machine no periods to arrive in.
    #[test]
    fn a_settle_policy_nothing_can_be_measured_against_is_refused() {
        let cfg = resolved();
        for settle in [
            SettleConfig {
                tolerance: 0.0,
                ..cfg.settle
            },
            SettleConfig {
                tolerance: -0.1,
                ..cfg.settle
            },
            SettleConfig {
                tolerance: f64::NAN,
                ..cfg.settle
            },
            SettleConfig {
                tolerance: f64::INFINITY,
                ..cfg.settle
            },
            SettleConfig {
                timeout: Duration::ZERO,
                ..cfg.settle
            },
        ] {
            let refused = MotionPump::new(
                &cfg.motion,
                &cfg.map,
                cfg.tick_hz,
                cfg.health_poll_hz,
                JointVector::default(),
                settle,
            )
            .err()
            .expect("an unusable settle policy is refused");
            assert!(
                matches!(refused, PumpError::Settle { .. }),
                "{settle:?}: {refused}"
            );
        }
    }

    /// Verdicts named directly come out the same shape a read outcome makes:
    /// the failures in the order given, the clean answers gone, the damaged
    /// frames carried. A caller assembling one of these is describing a bus it
    /// does not have, and a value that behaved differently from the read path's
    /// would make that description a lie.
    #[test]
    fn failures_named_directly_hold_only_the_failures() {
        let failures = ReadFailures::from_verdicts(
            &[
                (11, IdOutcome::Timeout),
                (
                    12,
                    IdOutcome::Ok(RawValue::new(&[0]).expect("one byte fits")),
                ),
                (13, IdOutcome::ServoError(dxl_proto::StatusError(0x07))),
            ],
            2,
        );

        assert_eq!(
            failures.servos(),
            &[
                (11, IdOutcome::Timeout),
                (13, IdOutcome::ServoError(dxl_proto::StatusError(0x07))),
            ]
        );
        assert_eq!(failures.corrupt_frames(), 2);
        assert_ne!(
            failures,
            ReadFailures::from_verdicts(&[(11, IdOutcome::Timeout)], 2),
            "a different set of servos is a different condition"
        );
        assert_eq!(ReadFailures::from_verdicts(&[], 0), ReadFailures::default());
    }

    /// Every event renders as its own line, so a supervised run reads.
    #[test]
    fn every_event_says_what_happened() {
        let servos = [ServoHealth { id: 10, bits: 1 }; JointId::COUNT];
        let mut said = [false; 19];
        for (event, expected) in [
            (TickEvent::Command(CommandDisposition::None), "no command"),
            (TickEvent::Command(CommandDisposition::Started), "moving"),
            (
                TickEvent::Command(CommandDisposition::Retargeted),
                "replacing the move that was running",
            ),
            (TickEvent::Command(CommandDisposition::Held), "holding"),
            (
                TickEvent::Command(CommandDisposition::Rejected(
                    CommandRejection::AntennaUnreachable {
                        joint: JointId::AntennaLeft,
                        angle: 1e7,
                    },
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
            (TickEvent::Completed, "commanding finished"),
            (
                TickEvent::Settled {
                    after: Duration::from_millis(620),
                },
                "0.62 s after commanding finished",
            ),
            (
                TickEvent::Unsettled {
                    joint: JointId::Leg(2),
                    error: 0.1,
                    waited: Duration::from_secs(2),
                },
                "still 5.73° from its goal",
            ),
            (
                TickEvent::TraceFull { samples: 120_000 },
                "120000 period(s)",
            ),
            (
                TickEvent::Faulted(Fault::PositionFeedbackLost { misses: 51 }),
                "faulted",
            ),
            (
                TickEvent::Stretched(ClockStretch {
                    requested: MoveDurations::uniform(Duration::from_millis(300)),
                    effective: MoveDurations::uniform(Duration::from_millis(800)),
                    separation: None,
                    separation_required: shipped_separation(),
                    dephased: false,
                }),
                "clock stretched",
            ),
            (
                TickEvent::AntennasDegraded(Fault::AntennaObstructed {
                    joint: JointId::AntennaRight,
                    error: 0.5,
                }),
                "antennas out of service",
            ),
            (
                TickEvent::Aborted(MoveAbort::StepTooLarge {
                    joint: JointId::Leg(2),
                    delta: 0.4,
                }),
                "move abandoned",
            ),
        ] {
            let printed = event.to_string();
            assert!(
                printed.contains(expected),
                "{event:?} rendered as {printed}"
            );
            said[event_slot(&event)] = true;
        }
        // An array of instances is not an enumeration: without this, a variant
        // added tomorrow renders however it renders and nothing here notices.
        assert!(
            said.iter().all(|seen| *seen),
            "an event renders unread by this test: {said:?}"
        );
    }

    /// The three things the clock line can say, said in the operator's words:
    /// a clock lengthened for its span, a pair parted at its crossing, and a
    /// pair nothing could part.
    ///
    /// The last of them changes no clock at all, so the line is the whole
    /// report: a move that runs with its tips converging says so or says
    /// nothing.
    #[test]
    fn the_clock_line_says_which_of_the_two_it_did() {
        let requested = MoveDurations::uniform(Duration::from_millis(800));
        let stretched = MoveDurations {
            head: Duration::from_millis(800),
            antennas: [Duration::from_millis(970), Duration::from_millis(800)],
        };
        let parted = PhaseSeparation {
            offset: 0.61,
            later: JointId::AntennaRight,
            at: Duration::from_millis(580),
            leader_rate: 5.0,
        };
        let converging = PhaseSeparation {
            offset: 0.09,
            ..parted
        };

        let span = TickEvent::Stretched(ClockStretch {
            requested,
            effective: stretched,
            separation: None,
            separation_required: shipped_separation(),
            dephased: false,
        })
        .to_string();
        assert!(
            span.contains("clock stretched to fit the span") && !span.contains("antennas cross"),
            "{span}"
        );

        let phase = TickEvent::Stretched(ClockStretch {
            requested,
            effective: stretched,
            separation: Some(parted),
            separation_required: shipped_separation(),
            dephased: true,
        })
        .to_string();
        assert!(
            phase.contains("to fit the span and de-phase the antennas")
                && phase.contains("right antenna 0.800 s to 0.970 s")
                && phase.contains("the antennas cross 0.61 rad apart")
                && !phase.contains("under the"),
            "{phase}"
        );

        let unmet = TickEvent::Stretched(ClockStretch {
            requested,
            effective: requested,
            separation: Some(converging),
            separation_required: shipped_separation(),
            dephased: false,
        })
        .to_string();
        assert_eq!(
            unmet,
            "the antennas cross 0.09 rad apart, under the 0.60 rad that keeps their tips clear"
        );

        // The figure it says the pair fell short of is the one the pass was
        // holding them to, whatever a configuration set it to — the line is
        // read by an operator who moved it.
        let widened = TickEvent::Stretched(ClockStretch {
            requested,
            effective: requested,
            separation: Some(parted),
            separation_required: 0.9,
            dephased: false,
        })
        .to_string();
        assert_eq!(
            widened,
            "the antennas cross 0.61 rad apart, under the 0.90 rad that keeps their tips clear"
        );
    }

    /// The separation the shipped configuration holds a pair to.
    fn shipped_separation() -> f64 {
        reachy_motion::AntennaPhaseConfig::default().separation_rad
    }

    /// Which event this is, as a slot in the coverage above.
    ///
    /// Wildcard-free, so a new event does not compile until it is named here,
    /// and does not pass until the table above renders it.
    fn event_slot(event: &TickEvent) -> usize {
        match event {
            TickEvent::Command(CommandDisposition::None) => 0,
            TickEvent::Command(CommandDisposition::Started) => 1,
            TickEvent::Command(CommandDisposition::Retargeted) => 2,
            TickEvent::Command(CommandDisposition::Held) => 3,
            TickEvent::Command(CommandDisposition::Rejected(_)) => 4,
            TickEvent::ReadLost { .. } => 5,
            TickEvent::ReadRestored { .. } => 6,
            TickEvent::HealthLost { .. } => 7,
            TickEvent::HealthRestored { .. } => 8,
            TickEvent::Health(_) => 9,
            TickEvent::Overrun { .. } => 10,
            TickEvent::Stretched(_) => 11,
            TickEvent::Completed => 12,
            TickEvent::Settled { .. } => 13,
            TickEvent::Unsettled { .. } => 14,
            TickEvent::TraceFull { .. } => 15,
            TickEvent::Faulted(_) => 16,
            TickEvent::AntennasDegraded(_) => 17,
            TickEvent::Aborted(_) => 18,
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

    /// A clock that stalls once, for as long as it is told, and keeps every
    /// deadline it was handed.
    struct StallClock {
        now: Duration,
        stall: Duration,
        /// Which wake stalls, counted from one.
        at: u64,
        woken: u64,
        /// Deadlines that were already in the past when the loop asked to wait
        /// for them: periods the loop would run back to back.
        behind: u64,
    }

    impl Clock for StallClock {
        fn now(&self) -> Duration {
            self.now
        }

        fn sleep_until(&mut self, until: Duration) {
            self.woken += 1;
            if until <= self.now {
                self.behind += 1;
            }
            let late = if self.woken == self.at {
                self.stall
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

    /// Every way a run can end, and the response each asks for on either side
    /// of the torque line.
    ///
    /// The classification written out by hand, so what the code decides and
    /// what somebody decided are two things that have to agree. A variant added
    /// to `PumpError` does not compile until `class` names it and does not pass
    /// until it is judged here.
    #[test]
    fn every_ending_names_the_response_it_asks_for() {
        use ErrorClass::{
            ImmediateAllTorqueOffToPark as OffPark, MaskedSlowStowToPark as MaskedPark,
            Refuse as No, SlowStowToRest as Stow,
        };

        let context = StepContext::reg(SeqStep::PinAndEnable, 13, RegId::TorqueEnable);
        let waited = Duration::from_millis(5);
        let table: Vec<(PumpError, ErrorClass, ErrorClass)> = vec![
            // The wire failing under torque is a machine that can no longer be
            // manoeuvred; before torque it is a machine nobody asked to move.
            (SeqError::NoAnswer { context }.into(), No, OffPark),
            (
                PumpError::Bus {
                    id: 13,
                    source: XactError::Timeout { id: 13, waited },
                },
                No,
                OffPark,
            ),
            // A release nobody could confirm is never reported as rest.
            (PumpError::TorqueOffUnacked { id: 13 }, OffPark, OffPark),
            // Our own tables, roster, rates, windows and budgets: a healthy
            // machine and a defect of ours.
            (
                PumpError::Map {
                    id: 13,
                    reg: RegId::GoalPosition,
                    source: MapError::UnknownJoint { joint: 11 },
                },
                No,
                Stow,
            ),
            (PumpError::UnknownServo { id: 99 }, No, Stow),
            (
                PumpError::Rate {
                    tick_hz: 0,
                    health_poll_hz: 0,
                },
                No,
                Stow,
            ),
            (
                PumpError::Settle {
                    tolerance: 0.0,
                    timeout: Duration::ZERO,
                },
                No,
                Stow,
            ),
            (PumpError::Runaway { budget: 10 }, No, Stow),
            (PumpError::Stalled { budget: 10 }, No, Stow),
            (
                MoveAbort::StepTooLarge {
                    joint: JointId::Leg(2),
                    delta: 0.4,
                }
                .into(),
                No,
                Stow,
            ),
            // Provisioning and reboot: torque is off by construction on both
            // paths, so both phases decline.
            (
                PumpError::WrongPart {
                    id: 13,
                    model: 1,
                    expected: 1200,
                },
                No,
                No,
            ),
            (PumpError::TorqueHeld { id: 13 }, No, No),
            (
                PumpError::OffRoster {
                    id: 99,
                    roster: [10, 11, 12, 13, 14, 15, 16, 17, 18],
                },
                No,
                No,
            ),
            (
                PumpError::NotBack {
                    id: 13,
                    polls: 3,
                    waited,
                    source: XactError::Timeout { id: 13, waited },
                },
                No,
                No,
            ),
            (PumpError::NotRestarted { id: 13 }, No, No),
            (
                PumpError::RestartUnconfirmed {
                    id: 13,
                    source: XactError::Timeout { id: 13, waited },
                },
                No,
                No,
            ),
            // The tick's own answers, which the classification takes verbatim.
            //
            // The two antenna conditions are the only ones whose response is
            // not an ending. They arrive here from a caller that surfaces the
            // tick's degrade as one, and what they ask for is the stow a
            // healthy, still-commanding head can run.
            (
                Fault::AntennaObstructed {
                    joint: JointId::AntennaRight,
                    error: 0.5,
                }
                .into(),
                Stow,
                Stow,
            ),
            (
                Fault::AntennaServoFault {
                    joint: JointId::AntennaLeft,
                    id: 18,
                    bits: 0x20,
                }
                .into(),
                Stow,
                Stow,
            ),
            (
                Fault::HeadObstructed {
                    joint: JointId::Leg(2),
                    error: 0.3,
                }
                .into(),
                Stow,
                Stow,
            ),
            (
                Fault::MeasuredPoseInvalid {
                    failures: 1,
                    source: FkError::NoConvergence {
                        iters: 8,
                        residual: 1e-3,
                    },
                }
                .into(),
                OffPark,
                OffPark,
            ),
            (
                Fault::TorqueOffUnconfirmed { id: 13 }.into(),
                OffPark,
                OffPark,
            ),
            (
                Fault::HeadServoFault {
                    joint: JointId::Leg(2),
                    id: 13,
                    bits: 0x20,
                }
                .into(),
                MaskedPark,
                MaskedPark,
            ),
            (
                Fault::PositionFeedbackLost { misses: 51 }.into(),
                OffPark,
                OffPark,
            ),
            (
                CommandRejection::AntennaUnreachable {
                    joint: JointId::AntennaLeft,
                    angle: 9.0,
                }
                .into(),
                No,
                No,
            ),
        ];

        let mut judged = [false; 8];
        for (error, before, under) in table {
            assert_eq!(
                error.class(Phase::PreTorque),
                before,
                "before torque: {error}"
            );
            assert_eq!(
                error.class(Phase::UnderTorque),
                under,
                "under torque: {error}"
            );
            // One classification, not two: where an ending names a condition,
            // that condition's own response is what the class is.
            for phase in [Phase::PreTorque, Phase::UnderTorque] {
                if let Some(fault) = error.fault(phase) {
                    judged[fault_slot(&fault)] = true;
                    assert_eq!(
                        error.class(phase),
                        ErrorClass::answering(fault.response()),
                        "{error}"
                    );
                }
            }
        }
        // The agreement check above compares the classification against
        // itself, so it holds nothing on its own: what makes the hand-written
        // column above the judgement is that every condition reaches it.
        assert!(
            judged.iter().all(|seen| *seen),
            "a fault named no ending in the table: {judged:?}"
        );
    }

    /// Which condition this is, as a slot in the coverage above.
    ///
    /// Wildcard-free, so a fault added to the doctrine cannot be left out of
    /// the ending table by the table simply not mentioning it.
    fn fault_slot(fault: &Fault) -> usize {
        match fault {
            Fault::AntennaObstructed { .. } => 0,
            Fault::AntennaServoFault { .. } => 1,
            Fault::HeadObstructed { .. } => 2,
            Fault::HeadServoFault { .. } => 3,
            Fault::PositionFeedbackLost { .. } => 4,
            Fault::MeasuredPoseInvalid { .. } => 5,
            Fault::BusFailure { .. } => 6,
            Fault::TorqueOffUnconfirmed { .. } => 7,
        }
    }

    /// A class runs the maneuver its response runs, and answers with it
    /// itself, so nothing downstream has to spell one out.
    ///
    /// The one divergence is the response that is not an ending: an error that
    /// names `DegradeAntennas` comes from a caller that surfaced the tick's
    /// degrade as an ending, and what a still-commanding head gets for it is
    /// the stow — not the antenna torque-off, which has already happened.
    #[test]
    fn every_class_names_the_maneuver_it_runs() {
        let table: Vec<(Response, Option<Maneuver>)> = vec![
            (Response::Refuse, None),
            (Response::SlowStowToRest, Some(Maneuver::SlowStow)),
            (
                Response::MaskedSlowStowToPark,
                Some(Maneuver::MaskedSlowStow),
            ),
            (
                Response::ImmediateAllTorqueOffToRest,
                Some(Maneuver::ImmediateAllTorqueOff),
            ),
            (
                Response::ImmediateAllTorqueOffToPark,
                Some(Maneuver::ImmediateAllTorqueOff),
            ),
            (Response::DegradeAntennas, Some(Maneuver::SlowStow)),
        ];

        let mut judged = [false; 6];
        for (response, maneuver) in table {
            judged[response_slot(response)] = true;
            assert_eq!(
                ErrorClass::answering(response).maneuver(),
                maneuver,
                "{response:?}"
            );
            // Where the response is an ending at all, the class is not a second
            // opinion about which maneuver that is.
            if response != Response::DegradeAntennas {
                assert_eq!(
                    ErrorClass::answering(response).maneuver(),
                    response.maneuver(),
                    "{response:?}"
                );
            }
        }
        assert!(
            judged.iter().all(|seen| *seen),
            "a response named no maneuver: {judged:?}"
        );
    }

    /// Which answer this is, as a slot in the coverage above.
    ///
    /// Wildcard-free, so a response added to the doctrine cannot be left out of
    /// the maneuver table by the table simply not mentioning it.
    fn response_slot(response: Response) -> usize {
        match response {
            Response::Refuse => 0,
            Response::SlowStowToRest => 1,
            Response::DegradeAntennas => 2,
            Response::MaskedSlowStowToPark => 3,
            Response::ImmediateAllTorqueOffToRest => 4,
            Response::ImmediateAllTorqueOffToPark => 5,
        }
    }

    /// A sequencer's verdict under torque names the wire or the mechanism, and
    /// the two are told apart even though they take the same response.
    ///
    /// They are the same maneuver and would be the same alert, which is exactly
    /// why: an operator sent to the cabling over a pose the solver could not
    /// place is an operator looking at the wrong half of the machine.
    #[test]
    fn a_sequencer_verdict_under_torque_names_the_wire_or_the_mechanism() {
        let context = StepContext::reg(SeqStep::PinAndEnable, 13, RegId::TorqueEnable);
        let wire = PumpError::Sequence(SeqError::VerifyMismatch {
            context,
            expected: RegValue::U8(1),
            read_back: RegValue::U8(0),
        });
        let mechanism = PumpError::Sequence(SeqError::PinnedPoseUnsolvable {
            context,
            cause: FkError::NoConvergence {
                iters: 8,
                residual: 1e-3,
            },
        });
        let ours = PumpError::Sequence(SeqError::WrongValue {
            context,
            expected: ValueKind::U8,
            observed: ValueKind::I32,
        });

        assert!(matches!(
            wire.fault(Phase::UnderTorque),
            Some(Fault::BusFailure { .. })
        ));
        assert!(matches!(
            mechanism.fault(Phase::UnderTorque),
            Some(Fault::MeasuredPoseInvalid { .. })
        ));
        // A register table that disagrees with the step is a defect of ours,
        // with a machine that answered perfectly.
        assert_eq!(ours.fault(Phase::UnderTorque), None);
        assert_eq!(ours.class(Phase::UnderTorque), ErrorClass::SlowStowToRest);

        for error in [&wire, &mechanism] {
            assert_eq!(
                error.class(Phase::UnderTorque),
                ErrorClass::ImmediateAllTorqueOffToPark,
                "{error}"
            );
            assert_eq!(error.fault(Phase::PreTorque), None, "{error}");
            assert_eq!(error.class(Phase::PreTorque), ErrorClass::Refuse, "{error}");
        }
    }

    /// The engage drive is pre-torque until its first enable goes out.
    ///
    /// The one path that crosses the line mid-run, and the difference the
    /// crossing makes: the same failing write is a wake that may be asked for
    /// again, or nine servos energized with nobody driving them.
    #[test]
    fn an_engage_is_pre_torque_until_its_first_enable() {
        assert_eq!(engage_phase(false), Phase::PreTorque);
        assert_eq!(engage_phase(true), Phase::UnderTorque);

        let context = StepContext::reg(SeqStep::PinAndEnable, 13, RegId::TorqueEnable);
        let failed_enable = PumpError::Sequence(SeqError::VerifyMismatch {
            context,
            expected: RegValue::U8(1),
            read_back: RegValue::U8(0),
        });
        assert_eq!(
            failed_enable.class(engage_phase(false)),
            ErrorClass::Refuse,
            "nothing was energized"
        );
        assert_eq!(
            failed_enable.class(engage_phase(true)),
            ErrorClass::ImmediateAllTorqueOffToPark,
            "servos are holding and the wire is not carrying"
        );
    }

    /// A configuration that takes the antennas out of service on a sweep they
    /// run behind on: a threshold a following joint crosses and a progress
    /// minimum nothing closes, so a joint that merely lags reads as one that
    /// stopped following.
    fn degrading_cfg() -> Resolved {
        let mut cfg = resolved();
        cfg.motion.tracking.threshold_rad = 0.02;
        cfg.motion.tracking.progress_min_rad = 1.0;
        cfg
    }

    /// What a run that degraded its antennas left behind.
    struct Degraded {
        summary: MoveSummary,
        events: Vec<TickEvent>,
        trace: Vec<TickSample>,
        entries: Vec<Entry>,
    }

    impl Degraded {
        /// The period the pair was taken out of service, located by the goal
        /// column rather than by the released one — the column under test in
        /// two of the assertions below.
        ///
        /// Nothing writes a masked servo, so the wire's picture of the pair's
        /// goals stops moving on the period the mask went up and every later
        /// period repeats it. The sweep is monotone, so the first period
        /// carrying the value the run ends on is that one.
        fn raised_at(&self) -> usize {
            let frozen = self.trace.last().expect("a recorded run").goal.antennas;
            self.trace
                .iter()
                .position(|sample| sample.goal.antennas == frozen)
                .expect("the last period carries it")
        }

        /// How far the pair had drifted from the goals it was last written by
        /// the end of the run.
        fn parting(&self) -> [f64; 2] {
            let last = self.trace.last().expect("a recorded run");
            let measured = last.present.expect("a released servo is still read");
            [0, 1].map(|side| (measured.antennas[side] - last.goal.antennas[side]).abs())
        }
    }

    /// A lift that also sweeps both antennas, with the pair following far
    /// enough behind to be taken out of service part way up.
    ///
    /// A move the antennas do not make is one they cannot fall behind on, so
    /// the sweep is what makes this the incident the bench saw. Only the pair
    /// is delayed, so the head tracks perfectly and the move finishes.
    fn a_degrading_sweep(cfg: &Resolved) -> Degraded {
        let mut bench = armed(cfg, machine_at(&datumed_config(), &stow_legs()));
        for id in [cfg.map.ids()[7], cfg.map.ids()[8]] {
            bench.registers.borrow_mut().delay.insert(id, 6);
        }
        let mut pump = pump(cfg, bench.held);
        pump.record_trace(true);
        let mut clock = TestClock::default();

        let sweeping = MotionCommand::MoveTo {
            target: JointTargets {
                antennas: [1.2, 1.2],
                ..JointTargets::default()
            },
            durations: cfg.up_durations(),
            warp: Warp::MinJerk,
        };
        let (outcome, events) = run(&mut bench, &mut pump, sweeping, &mut clock);
        let summary = outcome.expect("a degraded pair does not end the head's move");
        Degraded {
            summary,
            events,
            trace: pump.last_trace().to_vec(),
            entries: bench.state.timeline().entries().to_vec(),
        }
    }

    /// Degrading the pair is a maneuver that starts and finishes inside the
    /// run, and the record says so.
    ///
    /// The whole of `antenna_torque_off` is the write that releases the pair,
    /// so it is over as soon as that lands — the head's move carries on past a
    /// closed entry rather than under an answer that never ends. And once the
    /// pair is out of service, the rest of the run adds nothing: the joints
    /// keep lagging by the whole remaining sweep and nobody is measuring them.
    #[test]
    fn degrading_the_pair_is_recorded_start_to_finish() {
        let cfg = degrading_cfg();
        let ended = a_degrading_sweep(&cfg);

        let Some(Entry::Fault { fault, at: raised }) = ended.entries.first() else {
            panic!(
                "the obstruction is the first thing that happened: {:?}",
                ended.entries
            );
        };
        assert!(
            matches!(fault, Fault::AntennaObstructed { .. }),
            "{fault}: the pair lags, and lagging is what took it out"
        );
        assert!(
            matches!(
                ended.entries[1..],
                [
                    Entry::Response {
                        maneuver: Maneuver::AntennaTorqueOff,
                        outcome: Outcome::Started,
                        ..
                    },
                    Entry::Response {
                        maneuver: Maneuver::AntennaTorqueOff,
                        outcome: Outcome::Completed,
                        ..
                    },
                ]
            ),
            "{:?}",
            ended.entries
        );
        let Entry::Response {
            outcome: Outcome::Started,
            at: started,
            ..
        } = ended.entries[1]
        else {
            panic!("nothing was under way, so the pair's release is a start");
        };
        assert_eq!(started, *raised, "released on the period it was raised on");
        assert_eq!(ended.entries.len(), 3, "and nothing after it");
    }

    /// A degraded pair is not waited for at the settle.
    ///
    /// An antenna the tick took out of service was released where it stood and
    /// is commanded no further, so it never arrives at the goal it was
    /// abandoned at. Waiting for it would end every degraded move at the window
    /// naming a joint nobody expected to move, and hide the head arriving.
    #[test]
    fn a_degraded_pair_is_not_waited_for_at_the_settle() {
        let cfg = degrading_cfg();
        let ended = a_degrading_sweep(&cfg);

        assert!(
            ended
                .events
                .iter()
                .any(|event| matches!(event, TickEvent::AntennasDegraded(_))),
            "{:?}",
            ended.events
        );
        // The gap the exclusion is about: the pair came to rest well outside
        // the tolerance of the goal each was last written, which is what the
        // settle would otherwise spend its whole window on.
        for (side, parting) in ended.parting().into_iter().enumerate() {
            assert!(
                parting > cfg.settle.tolerance,
                "antenna {side} was left {parting} rad short of its goal"
            );
        }
        assert!(
            ended.summary.settled.is_some(),
            "the head arrived and was measured there: {:?}",
            ended.summary
        );
        assert_eq!(
            ended.summary.unsettled, None,
            "a released antenna is not a joint that failed to arrive"
        );
    }

    /// A released servo stops contributing to the run's worst lag.
    ///
    /// The tick keeps advancing a masked joint's goal row with the plan — the
    /// mask decides what reaches the wire, not what the trajectory says — so
    /// the difference it measures for that joint afterwards is against a
    /// command that never went out. Reported, it would put the whole remaining
    /// sweep into the one figure the tracking threshold, the window and the
    /// stow tolerance are all calibrated against.
    #[test]
    fn a_released_servo_stops_counting_toward_the_worst_lag() {
        let cfg = degrading_cfg();
        let ended = a_degrading_sweep(&cfg);

        for (side, parting) in ended.parting().into_iter().enumerate() {
            let row = 7 + side;
            let lag = ended.summary.worst_lag[row];
            // What the pair really ran behind before it went: the delay it was
            // given, which is what tripped the tracking monitor.
            assert!(
                lag > cfg.motion.tracking.threshold_rad,
                "antenna {side} lagged {lag} rad, never enough to be taken out"
            );
            assert!(
                lag < parting,
                "antenna {side} reports {lag} rad of lag, past the {parting} rad it was left \
                 from a goal it was never sent"
            );
        }
        // The head, which was never masked, keeps every period it measured.
        assert!(
            ended.summary.worst_lag[..7].iter().any(|lag| *lag > 0.0),
            "the head's lag went missing: {:?}",
            ended.summary.worst_lag
        );
    }

    /// The trace stops commanding a released servo from the period after the
    /// one that released it, and keeps measuring it throughout.
    ///
    /// The trace is the diagnostic of record for a degrade, and what it must
    /// not show is a torqued-off servo under a goal: the last one it was
    /// written stays in the pump's picture of the wire, and repeated down the
    /// column it reads as a command error nothing ever committed. The period
    /// the mask is raised on is the exception and is not blanked — the servo
    /// was holding that goal when the period's read was taken, and that read is
    /// the whole diagnosis of why it went.
    #[test]
    fn a_released_servo_is_traced_as_commanded_nothing() {
        let cfg = degrading_cfg();
        let ended = a_degrading_sweep(&cfg);
        let trace = &ended.trace;
        let raised_at = ended.raised_at();
        assert!(raised_at > 0, "the run started with everything commanded");

        // The boundary itself: the period the mask went up still carries the
        // pair's goals, because the servos were holding them when its read was
        // taken, and the period after it is the first that carries none.
        for sample in &trace[..=raised_at] {
            assert!(
                sample.released.is_empty(),
                "period {} was commanded and holding",
                sample.tick
            );
        }
        for sample in &trace[raised_at + 1..] {
            for joint in [JointId::AntennaRight, JointId::AntennaLeft] {
                assert!(
                    sample.released.contains(joint),
                    "{joint} at period {}",
                    sample.tick
                );
            }
            assert!(
                sample.present.is_some(),
                "a released servo is still measured at period {}",
                sample.tick
            );
        }
        // The number the blanking keeps out of the file: the pair's goal cell
        // does not move again after the mask, because nothing writes it.
        assert_eq!(
            trace.last().expect("the run has periods").goal.antennas,
            trace[raised_at].goal.antennas,
            "the goal a released servo carries is the last one it was sent"
        );
    }
}
