//! One bus cycle: what the driver reads off the machine and what it writes to
//! it.
//!
//! The spine of a cycle, in the order it runs: read the nine present positions
//! in one grouped request, ask the gate what to write and write it, then hand
//! back the sample and at most one event for the cycle. Everything here is
//! blocking serial work and nothing here sleeps or waits on a socket — the
//! cycle's grid instant arrives as an argument, and the one clock read is the
//! stamp taken the moment the read completes, which is what makes
//! `sample_time − nominal_time` a free jitter measurement on every sample.
//!
//! What to write is not decided here. The gate, the dead-man and the torque
//! belief are [`reachy_driver`]'s, kept in this process's own memory as the same
//! schemas the simulated driver keeps in its state slot, so the two hosts cannot
//! disagree about which goal is due or when silence has gone on long enough to
//! de-torque the machine. This module is the wire: it turns a decision into
//! transactions and a set of replies into a sample.
//!
//! Three rules hold over everything below.
//!
//! **No retries inside a cycle.** [`reachy_bus::with_retry`] sleeps the thread
//! between attempts and is not reachable from here: a missed exchange is a row
//! missing from this cycle's sample, and whether the miss is persistent is the
//! blind-cycle counter's question rather than one exchange's. The one retry
//! mechanism this seam has is the session's own re-issue, which arrives as
//! another datagram like any other.
//!
//! **A read that answered nothing is a sample saying so.** The sample is
//! published every cycle without exception, because it is the clock the control
//! loop runs on; a cycle whose bus was silent carries every row in `missing`
//! and `present_valid` false, never no sample at all.
//!
//! **Nothing gates de-torquing.** A torque-off sweep is written whenever the
//! gate asks for one, before anything about it is verified, and a row whose
//! verified write failed is left un-believed rather than counted as released —
//! a de-torquing credited to a failed transaction is the one report this driver
//! must never make.
//!
//! The out-of-band half of a cycle is [`crate::aux`], run from here: at most
//! one transaction per cycle, picked by the driver layer's own slot and spent
//! on a host request, the torque-off confirmation's read-back or the health
//! rotation. What is left is the loop that runs a cycle on the grid, so nothing
//! in this crate runs one yet: a cycle's reports reach a caller rather than a
//! socket.

use brenn_reachy__cogs__session_cmd_clk_rs::{SessionCmd, SessionCmdKind};
use brenn_reachy__driver__aux_clk_rs::{AuxSlotState, AuxSlotStateWire, TorqueConfirmStateWire};
use brenn_reachy__driver__gate_clk_rs::{GateState, GateStateWire};
use brenn_reachy__driver__goal_clk_rs::GoalSetpoint;
use brenn_reachy__driver__health_clk_rs::{
    AuxOutcomeWire, AuxStatus, DriverEventWire, EventKind, HealthReportWire,
};
use brenn_reachy__driver__pose_clk_rs::{PoseSample, PoseSampleWire};
use brenn_reachy__hardware__dynamixel__registers_clk_rs::RegId;
use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKind;
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use clockwork_rs::SyncTime;
use dxl_proto::regs::{GOAL_POSITION, PRESENT_POSITION, TORQUE_ENABLE};
use reachy_bus::{Bus, BusPort, BusTiming, IdOutcome, RawValue, ServoMap, SyncReadOutcome};
use reachy_driver::report::{self, Event};
use reachy_driver::{
    AcceptOutcome, AuxOffer, AuxSlot, AuxTask, ConfirmReport, GateAction, GoalGate,
    TORQUE_OFF_CONFIRM_BUDGET_NS, TorqueOffConfirm,
};

use crate::aux::{self, Answer, CycleBounds, Request};
use reachy_motion::arm::{SERVO_IDS, row_of_id};
use reachy_motion::joints::{ROW_COUNT, flags, joint_ref, row, rows_of, write_rows};
use reachy_motion::value;

/// The driver's cross-cycle memory, as the schemas every host of these
/// decisions keeps.
///
/// Three slots in this process's own heap where a cog would have a state slot.
/// Held as the wire types rather than the validated views because that is what
/// the decision layer takes up: a fresh process starts all three cleared, which
/// is a gate that has been told nothing, no request pending with nothing
/// believed torqued, and a confirmation that is not running.
pub struct DriverState {
    gate: GateStateWire,
    aux: AuxSlotStateWire,
    confirm: TorqueConfirmStateWire,
}

impl Default for DriverState {
    fn default() -> Self {
        Self::new()
    }
}

impl DriverState {
    /// A driver that has just started.
    #[must_use]
    pub fn new() -> Self {
        let mut state = Self {
            gate: GateStateWire::new(),
            aux: AuxSlotStateWire::new(),
            confirm: TorqueConfirmStateWire::new(),
        };
        state.gate.clear_valid();
        state.aux.clear_valid();
        state.confirm.clear_valid();
        state
    }

    /// The gate and the aux slot together, which is how a cycle needs them: the
    /// dead-man is measured against what the driver believes about torque, and
    /// the belief lives in the slot.
    ///
    /// Both are cleared rather than raised on bytes that do not read as their
    /// state. This process is the only writer of either, so a refusal is memory
    /// nobody wrote, and the process whose job is to de-torque a machine does
    /// not get to panic over its own heap. A cleared gate holds nothing and
    /// commands nothing; a cleared belief is nothing torqued, which makes the
    /// dead-man stand down rather than latch against a machine it has lost
    /// track of.
    fn decide(&mut self) -> (&mut GateState, &mut AuxSlotState) {
        if self.gate.validate_mut().is_err() {
            self.gate.clear_valid();
        }
        if self.aux.validate_mut().is_err() {
            self.aux.clear_valid();
        }
        let gate = self
            .gate
            .validate_mut()
            .expect("a cleared gate reads as one");
        let aux = self
            .aux
            .validate_mut()
            .expect("a cleared slot reads as one");
        (gate, aux)
    }

    /// The confirmation pass, cleared first if its bytes do not read as one.
    fn confirming(&mut self) -> TorqueOffConfirm<'_> {
        if self.confirm.validate_mut().is_err() {
            self.confirm.clear_valid();
        }
        TorqueOffConfirm::over(
            self.confirm
                .validate_mut()
                .expect("a cleared pass reads as one"),
        )
    }
}

/// The numbers a cycle runs on, as configuration read at startup.
///
/// Three of [`crate::params`]'s fields, taken as one value because a cycle
/// needs all three and a driver built from two of them and a default would be a
/// driver running a schedule nobody wrote down.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TickConfig {
    /// The grid's spacing: how long a cycle has, end to end.
    pub period_ns: i64,
    /// How long the goal stream may be silent before the dead-man latches.
    pub hold_timeout_ns: i64,
    /// The minimum spacing between successive health reports. The rotation
    /// advances one row per report, so a whole lap takes nine of these.
    pub health_poll_period_ns: i64,
}

/// The per-exchange allowance a cycle's bus work runs under.
///
/// The bus layer's own default allowance is ten milliseconds, which is a
/// sensible timeout for a host commanding whole moves and an unusable one
/// inside a twenty-millisecond cycle: two exchanges under it already overrun
/// the period, so a driver waiting that long is a driver stealing the next
/// cycle to hear about an exchange that has already missed this one. Three
/// milliseconds is what a cycle can afford — the grouped read, the write and
/// one out-of-band pair fit inside the period with the margin below still
/// unspent — and an exchange that has not answered by then is a miss, which is
/// a reading this driver has somewhere to put.
///
/// The retry fields are inherited and unused: [`reachy_bus::with_retry`] sleeps
/// and is not reachable from a cycle.
#[must_use]
pub fn cycle_timing(baud: u32) -> BusTiming {
    BusTiming {
        host_allowance: std::time::Duration::from_millis(3),
        baud,
        ..BusTiming::default()
    }
}

/// What a cycle keeps clear of its own end, nanoseconds.
///
/// The margin the aux budget is measured against: a transaction is run only if
/// its worst case still leaves this much of the period unspent. It buys the
/// publish step and the loop's own wake, which are the work a cycle owes after
/// the bus falls quiet, and it is deliberately a tenth of the cycle rather than
/// a measurement — the budget's job is to decide before a transaction runs, and
/// what the publish will take is not knowable then. Written as the fraction it
/// is, so a machine built to run a different grid keeps a margin that is still a
/// tenth of it.
const CYCLE_MARGIN_NS: i64 = reachy_driver::NOMINAL_CYCLE_NS / 10;

/// What the driver has counted, as plain numbers since the process started.
///
/// Process-local: they reach an operator as the driver's own log lines and, for
/// the ones the event vocabulary names, as events. Nothing here is a Clockwork
/// signal, because this process is not a cog.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TickCounts {
    /// Goals that became the held setpoint and were written.
    pub goals_executed: u64,
    /// Goals refused because the queue was full.
    pub goals_dropped: u64,
    /// Times the dead-man latched torque off.
    pub hold_timeouts: u64,
    /// Rows that did not answer a grouped read, summed over cycles.
    pub read_misses: u64,
    /// Transactions this driver could not put on the wire, or could not verify:
    /// a grouped read or write the port refused, an angle no servo count
    /// represents, and a torque-enable write that did not read back.
    pub write_failures: u64,
    /// Cycles in which the bus answered nothing at all.
    pub blind_cycles: u64,
    /// Events a cycle raised after it already had one to publish.
    pub events_dropped: u64,
    /// Host transactions turned away because one was already pending, and
    /// datagrams asking for nothing at all.
    pub aux_refused: u64,
    /// Cycles that had an out-of-band transaction to run and no time left to
    /// run it in.
    pub aux_deferred: u64,
    /// Health reports published.
    pub health_reports: u64,
    /// Health reads the machine did not answer, so no report went out.
    pub health_misses: u64,
    /// Confirmation read-backs the machine did not answer. Each one counts as a
    /// row still holding torque.
    pub confirm_misses: u64,
}

impl TickCounts {
    /// Every count, as labelled numbers for a log line.
    #[must_use]
    pub fn read(&self) -> [(&'static str, u64); 12] {
        [
            ("goals_executed", self.goals_executed),
            ("goals_dropped", self.goals_dropped),
            ("hold_timeouts", self.hold_timeouts),
            ("read_misses", self.read_misses),
            ("write_failures", self.write_failures),
            ("blind_cycles", self.blind_cycles),
            ("events_dropped", self.events_dropped),
            ("aux_refused", self.aux_refused),
            ("aux_deferred", self.aux_deferred),
            ("health_reports", self.health_reports),
            ("health_misses", self.health_misses),
            ("confirm_misses", self.confirm_misses),
        ]
    }
}

/// What a cycle has to say: the sample, always, and at most one event.
///
/// Both are the schemas the simulated driver publishes on its channels, built
/// here as the bytes a datagram will carry. Nothing is sent from this module — a
/// cycle hands its reports to whoever ran it.
pub struct CycleReport {
    /// This cycle's pose sample. Present on every cycle without exception.
    pub sample: PoseSampleWire,
    /// The one event this cycle raised, where it raised any.
    pub event: Option<DriverEventWire>,
    /// The answer to the one out-of-band transaction a cycle ran, where it ran
    /// one a host is waiting on.
    pub outcome: Option<AuxOutcomeWire>,
    /// The health rotation's report, on the cycles it read a servo.
    pub health: Option<HealthReportWire>,
}

/// The bus half of the driver: a port, the servo map, and the decisions' state.
pub struct Tick<P: BusPort> {
    bus: Bus<P>,
    map: ServoMap,
    state: DriverState,
    config: TickConfig,
    /// What each part of a cycle's bus work can cost at worst, off the bus
    /// layer's own timing. Computed once: it is configuration and not a
    /// measurement.
    bounds: CycleBounds,
    /// Cycles the bus has answered nothing on, in a row.
    blind_run: u32,
    /// The event this cycle will publish.
    pending: Option<Event>,
    /// The answer this cycle will publish.
    answer: Option<Answer>,
    counts: TickCounts,
}

impl<P: BusPort> Tick<P> {
    /// A driver's bus half over `bus`, addressing the nine servos of this
    /// machine.
    ///
    /// The map is the machine's own wiring rather than configuration: the bus
    /// ids are fixed by how the unit is provisioned, and a driver addressing
    /// different ones would be a driver for a different machine.
    pub fn new(bus: Bus<P>, config: TickConfig) -> Self {
        let bounds = CycleBounds::of(bus.timing());
        Self {
            bus,
            map: ServoMap::new(SERVO_IDS),
            state: DriverState::new(),
            config,
            bounds,
            blind_run: 0,
            pending: None,
            answer: None,
            counts: TickCounts::default(),
        }
    }

    /// What this driver has counted so far.
    #[must_use]
    pub fn counts(&self) -> TickCounts {
        self.counts
    }

    /// Whether anything about torque is still outstanding: a row believed
    /// torqued, or a commanded de-torquing whose confirmation pass is still
    /// running.
    ///
    /// What a stop gesture asks before deciding whether it has work to do. An
    /// unconcluded pass counts as well as the belief because a de-torquing
    /// nobody has read back is not one anything here may call done — the belief
    /// being empty says a verified write said so, not that a servo answered a
    /// register read. A pass that has read every row back released is the one
    /// state where there is nothing outstanding to do.
    pub fn torque_outstanding(&mut self) -> bool {
        let believed = {
            let (_, slot) = self.state.decide();
            AuxSlot::over(slot).belief().any()
        };
        let confirming = {
            let pass = self.state.confirming();
            let state = pass.state();
            state.active.get() && !state.said_confirmed.get()
        };
        believed || confirming
    }

    /// What the running confirmation pass has said, if it has said anything.
    ///
    /// Each verdict goes out once; a caller that missed the cycle it went out
    /// on would wait for a second one that never comes.
    pub fn confirm_said(&mut self) -> Option<ConfirmReport> {
        self.state.confirming().said()
    }

    /// Offer a setpoint that arrived over the seam to the gate.
    ///
    /// Accepting one is liveness: it is the evidence that whatever is supposed
    /// to be commanding this machine is still there. A refused one is not — a
    /// sender overrunning the queue must not be able to hold torque on by
    /// overrunning it faster — and both of the gate's remarks about a goal go
    /// out as events, because a sender that cannot see them cannot fix them.
    pub fn offer_goal(&mut self, goal: &GoalSetpoint, nominal_ns: i64) {
        let due_at = goal.execute_at.as_nanos();
        let (gate, _) = self.state.decide();
        let queued = gate.queue.len();
        let outcome = GoalGate::over(gate).accept(goal, nominal_ns);
        match outcome {
            AcceptOutcome::Accepted => {}
            AcceptOutcome::AcceptedStaleOrOutOfOrder => self.raise(Event {
                kind: EventKind::GoalStaleOrOutOfOrder,
                // How far past its instant it arrived. Zero for a goal that is
                // merely out of order with the one before it, which has not
                // missed anything yet.
                silence_ns: (nominal_ns - due_at).max(0),
                ..Event::at(nominal_ns)
            }),
            AcceptOutcome::DroppedQueueFull => {
                self.counts.goals_dropped += 1;
                self.raise(Event {
                    kind: EventKind::GoalDroppedQueueFull,
                    count: u32::try_from(queued).unwrap_or(u32::MAX),
                    ..Event::at(nominal_ns)
                });
            }
        }
    }

    /// Take one datagram from the host that commands this machine.
    ///
    /// One rule, the same one the goal path runs: a datagram the driver *acted
    /// on* is liveness, and a refused one is not. The dead-man measures silence
    /// and a host with nothing to ask still owes the driver a word, so a
    /// keep-alive counts — but a host that only ever gets a refusal must not be
    /// able to hold a machine energised by repeating it faster, which is exactly
    /// what the gate's own refusal of an overrun goal queue exists to stop.
    ///
    /// Two kinds are refused here. A datagram asking nothing is a slot nobody
    /// wrote, published; and an out-of-band request arriving while one is
    /// already pending is a host that is not the serial one it claims to be.
    /// Both are counted, and neither is fed to the dead-man.
    pub fn offer_session_cmd(&mut self, cmd: &SessionCmd, nominal_ns: i64) {
        let accepted = match cmd.kind {
            SessionCmdKind::None => false,
            // Liveness and nothing else: there is nothing to act on, and the
            // word itself is what the dead-man was waiting for.
            SessionCmdKind::KeepAlive => true,
            SessionCmdKind::TorqueOffNow => {
                self.request_torque_off(nominal_ns);
                true
            }
            SessionCmdKind::Aux => {
                let offered = {
                    let (_, slot) = self.state.decide();
                    AuxSlot::over(slot).offer(cmd.corr, &cmd.txn)
                };
                if offered == AuxOffer::RefusedBusy {
                    // Loud both ways: an outcome against the turned-away
                    // request's own number, and a count.
                    self.note_answer(Answer::busy(cmd.corr));
                }
                offered != AuxOffer::RefusedBusy
            }
        };
        if accepted {
            let (gate, _) = self.state.decide();
            GoalGate::over(gate).note_liveness(nominal_ns);
        } else {
            self.counts.aux_refused += 1;
        }
    }

    /// De-torque the machine because the host asked for it.
    ///
    /// Idempotent, and nothing gates it: the goal queue goes with the latch, the
    /// confirmation pass opens if one is not already running, and the sweep goes
    /// out on this cycle and every cycle the latch stands. No event — the host
    /// asked, so it already knows; the edge worth reporting is the one nobody
    /// asked for.
    ///
    /// The abandonment of whatever was queued is the point rather than tidiness:
    /// a release outranks whatever was asked for before it, and a setpoint
    /// written out of the queue after the latch would command a machine this
    /// call exists to let go of.
    pub fn request_torque_off(&mut self, nominal_ns: i64) {
        {
            let (gate, aux) = self.state.decide();
            GoalGate::over(gate).latch_torque_off();
            AuxSlot::over(aux).abandon();
        }
        self.state.confirming().begin(nominal_ns);
    }

    /// Run one cycle at grid instant `nominal_ns`, and answer what to publish.
    pub fn run(&mut self, nominal_ns: i64) -> CycleReport {
        let read = self.read_positions();
        // The one clock read of a cycle, taken the moment the proprioception
        // came back: `sample_time - nominal_time` is then the cycle's jitter,
        // measured for free on every sample.
        let sample_time_ns = now_ns();
        let swept = self.write_from_gate(nominal_ns);
        // Blind is a cycle the machine said nothing on, and the read is the
        // whole of the evidence: a grouped write is acknowledged by nothing at
        // all, so a write that went out says nothing about anyone listening.
        self.count_blind(nominal_ns, flags::is_empty(read.answered));
        let health = self.run_aux(nominal_ns, sample_time_ns, swept);

        let mut sample = PoseSampleWire::new();
        self.write_sample(sample.clear_valid(), &read, nominal_ns, sample_time_ns);
        let event = self.pending.take().map(|event| {
            let mut message = DriverEventWire::new();
            event.write(message.clear_valid());
            message
        });
        let outcome = self.answer.take().map(|answer| {
            let mut message = AuxOutcomeWire::new();
            answer.write(message.clear_valid());
            message
        });
        CycleReport {
            sample,
            event,
            outcome,
            health,
        }
    }

    /// Spend this cycle's one out-of-band transaction.
    ///
    /// Three things want it and [`AuxSlot`] is what picks, in the order it
    /// documents: a host request first, then the confirmation pass's read-back,
    /// then the health rotation. Run after the proprioception and after the
    /// write, so every register a transaction reads answers about the machine as
    /// this cycle left it — a verification of a de-torquing that read a
    /// register before the sweep would confirm the cycle before.
    ///
    /// The confirmation is stepped before the slot is asked, because what it
    /// wants read is an input to the choice. Its two reports are events: a whole
    /// clean pass says the commanded de-torquing took, and a budget that ran out
    /// says it cannot be confirmed — after which the pass keeps reading and the
    /// sweep keeps being written, because nothing gates de-torquing.
    ///
    /// The one thing that does not run is a transaction whose worst case will
    /// not fit in what is left of the cycle. A host request and a health read
    /// are deferred then — the slot is not asked at all, so the request stays
    /// pending and the rotation's cadence is not stamped for a read nobody made.
    /// A confirmation read-back is not deferrable and is run anyway: it is the
    /// evidence a commanded de-torquing took, the cycle that overran is a cycle
    /// that swept every row, and the next grid point is the loop's problem
    /// rather than a reason to stop reading back a release.
    fn run_aux(
        &mut self,
        nominal_ns: i64,
        sample_time_ns: i64,
        swept: bool,
    ) -> Option<HealthReportWire> {
        let step = self
            .state
            .confirming()
            .step(nominal_ns, TORQUE_OFF_CONFIRM_BUDGET_NS);
        match step.report {
            ConfirmReport::Nothing => {}
            ConfirmReport::Confirmed => {
                // Every row read back released. The belief goes to nothing here
                // rather than when the sweep was commanded: a de-torquing that
                // has not been read back is one the dead-man must keep running
                // over.
                {
                    let (gate, slot) = self.state.decide();
                    AuxSlot::over(slot).belief().confirmed_off();
                    GoalGate::over(gate).clear_commanded();
                }
                self.raise(Event {
                    kind: EventKind::TorqueOffConfirmed,
                    ..Event::at(nominal_ns)
                });
            }
            ConfirmReport::Unconfirmed => {
                // Which servos the pass has not read back released: the rows the
                // cursor has not reached. What the driver has evidence about,
                // and it is deliberately not the belief — a row can be believed
                // released by a verified write and still be one this pass has
                // not seen for itself.
                let rows = self.unread_rows();
                self.raise(Event {
                    kind: EventKind::TorqueOffUnconfirmed,
                    rows,
                    ..Event::at(nominal_ns)
                });
            }
        }

        let health_period_ns = self.config.health_poll_period_ns;
        let (pending, health_due) = {
            let (_, slot) = self.state.decide();
            let slot = AuxSlot::over(slot);
            (
                slot.state().has_pending.get(),
                slot.health_due(nominal_ns, health_period_ns),
            )
        };
        // A pending host request runs on a swept cycle even though the sweep
        // alone is more than the period. It has to: the transaction that ends a
        // torque-off latch is a verified torque-enable write, which arrives as a
        // host request, and a latched gate sweeps on every cycle — so a budget
        // that admitted nothing while the sweep stands would make the latch an
        // absorbing state, with the machine unable to be armed again for the
        // life of the process. The cost is a grid point, which the loop already
        // counts and publishes rather than making up for.
        let task = if self.aux_fits(swept) || (swept && pending) {
            let (_, slot) = self.state.decide();
            AuxSlot::over(slot).take(nominal_ns, health_period_ns, step.read_row)
        } else if let Some(row) = step.read_row {
            AuxTask::ConfirmTorqueOff { row }
        } else {
            // Counted only where something wanted the slot: a cycle with nothing
            // to run has deferred nothing, and counting it anyway would put the
            // counter at fifty a second forever and drown the cycles that really
            // did hold work back.
            if pending || health_due {
                self.counts.aux_deferred += 1;
            }
            AuxTask::Nothing
        };
        match task {
            AuxTask::Nothing => None,
            AuxTask::Host { corr } => {
                // The record stays in the slot and the transaction runs from a
                // copy of it: running it writes the state the record lives in.
                let (_, slot) = self.state.decide();
                let request = AuxSlot::over(slot).taken(corr).map(Request::of);
                if let Some(request) = request {
                    let answer = aux::answer(&mut self.bus, &self.map, corr, &request);
                    if answer.status == AuxStatus::Ok {
                        self.credit_torque_write(&request, nominal_ns);
                    }
                    self.note_answer(answer);
                }
                None
            }
            AuxTask::ConfirmTorqueOff { row } => {
                let id = self.map.id_at(usize::from(row));
                // A row with no servo behind it reads as still holding, for the
                // reason an unanswered read does: nothing has been seen to go
                // limp.
                let (torqued, answered) =
                    id.map_or((true, false), |id| aux::reads_torqued(&mut self.bus, id));
                if !answered {
                    self.counts.confirm_misses += 1;
                }
                self.state.confirming().observed(row, torqued);
                None
            }
            AuxTask::Health { row } => {
                let mut message = HealthReportWire::new();
                if aux::health(
                    &mut self.bus,
                    &self.map,
                    usize::from(row),
                    sample_time_ns,
                    message.clear_valid(),
                ) {
                    self.counts.health_reports += 1;
                    Some(message)
                } else {
                    // A servo that did not answer gets no report rather than a
                    // report of zeroes about a machine nobody heard from. The
                    // rotation walks on either way: the cadence was stamped
                    // when the read was named.
                    self.counts.health_misses += 1;
                    None
                }
            }
        }
    }

    /// Move the belief a verified torque-enable write earns.
    ///
    /// The one transaction a host can run that changes what the driver believes
    /// about the machine, and the belief is what the dead-man is measured
    /// against — so a driver that ran the write and did not move its belief
    /// would hold a machine energised with nothing watching the goal stream.
    /// Only a verified write earns it: the bus layer read the register back, so
    /// this is what the servo holds and not what was sent.
    ///
    /// An arming also grants a fresh window, ends a standing torque-off latch
    /// and stands the confirmation pass down: the machine has been energised
    /// again on purpose, and a pass still reading back the release it replaced
    /// would report on a de-torquing nobody is asking for any more. A release
    /// that leaves nothing believed holding clears what was commanded, because a
    /// limp machine is holding no setpoint.
    fn credit_torque_write(&mut self, request: &Request, nominal_ns: i64) {
        if request.op != AuxOpKind::WriteRegVerified || request.reg != RegId::TorqueEnable {
            return;
        }
        let (Some(row), Some(enabled)) = (
            row_of_id(request.id),
            value::carried(request.value_kind, request.value).as_u8(),
        ) else {
            return;
        };
        let enabled = enabled != 0;
        let (gate, slot) = self.state.decide();
        let any = {
            let mut slot = AuxSlot::over(slot);
            slot.belief()
                .verified_write(u8::try_from(row).unwrap_or(u8::MAX), enabled);
            slot.belief().any()
        };
        let latched = {
            let mut gate = GoalGate::over(gate);
            let latched = gate.state().latched.get();
            if enabled {
                if latched {
                    gate.release_latch(nominal_ns);
                } else {
                    gate.note_liveness(nominal_ns);
                }
            } else if !any {
                gate.clear_commanded();
            }
            latched
        };
        if enabled && latched {
            self.state.confirming().stand_down();
        }
    }

    /// Whether the worst out-of-band transaction still fits in this cycle.
    ///
    /// Arithmetic over bounds and not a measurement, and it does not need to be
    /// one: every exchange runs under a deadline the bus layer computes, and the
    /// bounds spent here are that same deadline asked for ahead of time, so what
    /// a cycle's bus work can cost is known before any of it runs. A cycle is
    /// the grouped read, the write, one out-of-band transaction and the margin,
    /// and the question is whether that sum is inside the period.
    ///
    /// Which write depends on what was written: a grouped goal write is one
    /// unacknowledged frame, while a torque-off sweep is a verified write per
    /// row and overruns the period on its own. That is the sweep's business and
    /// not something to trade against — nothing gates de-torquing — so a swept
    /// cycle has no room left for surveillance. Its caller exempts one thing
    /// from that: a host request already pending, which is the only way a latch
    /// can end.
    fn aux_fits(&self, swept: bool) -> bool {
        let write_ns = if swept {
            self.bounds.sweep_ns
        } else {
            self.bounds.write_ns
        };
        let needed = self
            .bounds
            .read_ns
            .saturating_add(write_ns)
            .saturating_add(self.bounds.aux_ns)
            .saturating_add(CYCLE_MARGIN_NS);
        needed <= self.config.period_ns
    }

    /// The rows a running confirmation pass has not read back released.
    ///
    /// The pass walks from row 0 and its cursor is how far it has come, so the
    /// rows from there on are the ones nothing has been seen about. A pass that
    /// is not running names none.
    fn unread_rows(&mut self) -> JointFlags {
        let from = self.state.confirming().waiting_on();
        let mut rows = JointFlags::NONE;
        if let Some(from) = from {
            for index in usize::from(from)..ROW_COUNT {
                if let Some(joint) = joint_ref(index) {
                    rows |= flags::bit(joint);
                }
            }
        }
        rows
    }

    /// Offer an answer for this cycle's one outcome slot.
    ///
    /// At most one transaction runs per cycle, so the slot collides only when a
    /// request was turned away in the same cycle one was served. The first
    /// answer wins and the displaced one comes back to the host as a silence,
    /// which is the case its own re-issue exists for.
    ///
    /// TODO(driver-host-sample-glue): the same first-answer-wins rule, and the
    /// gate-derived fields of [`Self::write_sample`], are written out a second
    /// time in the simulated driver's host. They are not in the driver layer
    /// beside the event ranking because the outcome record and the sample would
    /// take the transaction and pose vocabularies in with them, and one host's
    /// answer type is built over the bus layer.
    fn note_answer(&mut self, answer: Answer) {
        if self.answer.is_none() {
            self.answer = Some(answer);
        }
    }

    /// The cycle's proprioception: one grouped read of the nine present
    /// positions.
    ///
    /// A row that refused, answered short or said nothing lands in its own slot
    /// and nowhere else, so a cycle reports a partial reading rather than
    /// discarding eight good rows over one bad one — which is what the sample's
    /// `missing` set is for. A read the port itself refused answers nothing at
    /// all and is counted as a failed transaction as well: the frame never
    /// reached the wire.
    ///
    /// TODO(unsendable-frame-condition): what a frame this process could not
    /// send says about the machine. Until it is decided, the rows it would have
    /// asked about are missing and the write it would have carried is
    /// unconfirmed, which is the reading that claims least.
    fn read_positions(&mut self) -> Reading {
        let ids = self.map.ids();
        let mut outcome = SyncReadOutcome::new();
        if self
            .bus
            .sync_read(&ids, PRESENT_POSITION, &mut outcome)
            .is_err()
        {
            self.counts.write_failures += 1;
            self.counts.read_misses += ROW_COUNT as u64;
            return Reading::default();
        }
        let mut reading = Reading::default();
        for index in 0..ROW_COUNT {
            let Some(id) = self.map.id_at(index) else {
                continue;
            };
            let counts = match outcome.get(id) {
                Some(IdOutcome::Ok(raw)) => raw.i32(),
                _ => None,
            };
            match counts.and_then(|counts| self.map.present_rad(index, counts).ok()) {
                Some(angle) => {
                    reading.present[index] = angle;
                    if let Some(joint) = joint_ref(index) {
                        reading.answered |= flags::bit(joint);
                    }
                }
                None => self.counts.read_misses += 1,
            }
        }
        reading
    }

    /// Ask the gate what to write, and write it. Answers whether the write was
    /// a torque-off sweep, which is the one action that costs the cycle its
    /// whole bus budget.
    fn write_from_gate(&mut self, nominal_ns: i64) -> bool {
        let hold_timeout_ns = self.config.hold_timeout_ns;
        let (action, silence_ns) = {
            let (gate, aux) = self.state.decide();
            // What the driver believes about torque, not what the machine is:
            // this process has no window onto the machine that the bus is not,
            // and a dead-man measured against a belief nothing verified would be
            // a dead-man measuring its own writes.
            let torqued = AuxSlot::over(aux).belief().any();
            let silence_ns = nominal_ns.saturating_sub(gate.last_accept.as_nanos());
            (
                GoalGate::over(gate).tick(nominal_ns, torqued, hold_timeout_ns),
                silence_ns,
            )
        };
        match action {
            GateAction::WriteTorqueOffSweep { just_latched } => {
                // Opened before the sweep is written, and kept at the instant it
                // opened at across the cycles a standing latch repeats the sweep
                // on: the budget is measured from when the de-torquing was
                // commanded.
                self.state.confirming().begin(nominal_ns);
                if just_latched {
                    self.counts.hold_timeouts += 1;
                    self.raise(Event {
                        kind: EventKind::HoldTimeoutTorqueOff,
                        // How long the goal stream was silent, which is what
                        // tells an operator whether the commander stopped or
                        // merely stuttered.
                        silence_ns: silence_ns.max(0),
                        ..Event::at(nominal_ns)
                    });
                }
                self.sweep_torque_off();
                return true;
            }
            GateAction::WriteGoal => {
                self.counts.goals_executed += 1;
                self.write_held();
            }
            // The same setpoint again, which is what holds a servo's position
            // loop awake between goals. Not counted: a rewrite is not a goal
            // executed.
            GateAction::Rewrite => self.write_held(),
            // Nothing has ever been commanded, so nothing goes out.
            GateAction::Nothing => {}
        }
        false
    }

    /// Write the held setpoint's rows in one grouped write.
    ///
    /// The mask is the setpoint's own and it is write-side filtering and nothing
    /// else: a setpoint applies to the servos it names and leaves every other
    /// one holding the angle it already had. That is the whole meaning of a
    /// partial mask, stated here because this is the only place in this process
    /// that turns a commanded setpoint into servo targets.
    fn write_held(&mut self) {
        let (mask, targets) = {
            let (gate, _) = self.state.decide();
            (gate.held.mask, rows_of(&gate.held.targets))
        };
        let blank = RawValue::new(&0i32.to_le_bytes()).expect("four bytes carry a goal");
        let mut entries = [(0u8, blank); ROW_COUNT];
        let mut written = 0;
        for joint in flags::iter(mask) {
            let Some(index) = row(joint) else {
                continue;
            };
            let angle = targets[index];
            let (Some(id), Ok(counts)) =
                (self.map.id_at(index), self.map.goal_counts(index, angle))
            else {
                // An angle no servo count represents. Refused rather than
                // rounded to the nearest one a servo has: the envelope check
                // runs above this process, and a driver quietly commanding the
                // closest reachable angle instead would be the clamp this stack
                // does not have.
                self.counts.write_failures += 1;
                continue;
            };
            let Some(value) = RawValue::new(&counts.to_le_bytes()) else {
                continue;
            };
            entries[written] = (id, value);
            written += 1;
        }
        if written == 0 {
            return;
        }
        if self
            .bus
            .sync_write(GOAL_POSITION, &entries[..written])
            .is_err()
        {
            self.counts.write_failures += 1;
        }
    }

    /// Write the minimum risk condition: torque off on every row, verified.
    ///
    /// Nine verified writes, one per row, because the protocol acknowledges a
    /// grouped write with nothing at all and a de-torquing nobody read back is
    /// not evidence of a de-torqued machine. A row whose write failed is left
    /// out of the belief and its failure counted: a standing latch asks for the
    /// sweep again every cycle, and the confirmation pass keeps reading. Nothing
    /// here is conditional on anything — the sweep runs whatever the bus has
    /// been doing, and whatever else this cycle could not do.
    fn sweep_torque_off(&mut self) {
        let off = RawValue::new(&[0]).expect("one byte carries a torque-enable value");
        // The belief is taken once for the whole sweep rather than per row: the
        // state is validated on the way in, and this is the most expensive cycle
        // the driver runs -- nine revalidations of it inside the loop would be
        // spent on the one path whose overrun costs a grid point.
        let Self {
            bus,
            map,
            state,
            counts,
            ..
        } = self;
        let (_, aux) = state.decide();
        let mut belief = AuxSlot::over(aux);
        for index in 0..ROW_COUNT {
            let Some(id) = map.id_at(index) else {
                continue;
            };
            if bus.write_reg_verified(id, TORQUE_ENABLE, &off).is_err() {
                counts.write_failures += 1;
                continue;
            }
            belief
                .belief()
                .verified_write(u8::try_from(index).unwrap_or(u8::MAX), false);
        }
    }

    /// Count how long the bus has been answering nothing, and say so once the
    /// run is long enough to mean the bus is gone.
    ///
    /// The counting and the once-only rule are the driver layer's, over the run
    /// this process keeps: the fault a driver raises about itself has to mean
    /// the same thing from either host. What is this host's is the total, which
    /// is a number an operator reads off a log line rather than a decision.
    fn count_blind(&mut self, nominal_ns: i64, blind: bool) {
        if blind {
            self.counts.blind_cycles += 1;
        }
        if let Some(event) = report::count_blind(&mut self.blind_run, blind, nominal_ns) {
            self.raise(event);
        }
    }

    /// This cycle's sample.
    ///
    /// `missing` is every row the read did not produce an angle for, and the
    /// angles are reported whatever that set says about them: what makes a
    /// missing row's value unusable is the set, not a blank a receiver would
    /// have to tell apart from a real zero.
    fn write_sample(
        &mut self,
        sample: &mut PoseSample,
        read: &Reading,
        nominal_ns: i64,
        sample_time_ns: i64,
    ) {
        let missing = flags::without(flags::all(), read.answered);
        sample.nominal_time = SyncTime::from_nanos(nominal_ns);
        sample.sample_time = SyncTime::from_nanos(sample_time_ns);
        sample.missing = missing;
        sample.present_valid = flags::is_empty(missing).into();
        write_rows(&mut sample.present, &read.present);
        let (gate, _) = self.state.decide();
        sample.commanded_valid = gate.has_held;
        sample.torque_off_latched = gate.latched;
        let commanded = rows_of(&gate.held.targets);
        write_rows(&mut sample.commanded, &commanded);
    }

    /// Offer an event for this cycle's one slot, counting the one it displaces.
    ///
    /// The ranking is the driver layer's, so this host and the simulated one
    /// publish the same one out of a cycle that hit two.
    fn raise(&mut self, event: Event) {
        report::raise(&mut self.pending, &mut self.counts.events_dropped, event);
    }
}

/// What one grouped read produced.
#[derive(Clone, Copy, Debug)]
struct Reading {
    /// The angles, in bus order. A row that did not answer keeps the zero it
    /// started at and is named in `answered`'s complement.
    present: [f64; ROW_COUNT],
    /// The rows that produced an angle.
    answered: JointFlags,
}

impl Default for Reading {
    fn default() -> Self {
        Self {
            present: [0.0; ROW_COUNT],
            answered: JointFlags::NONE,
        }
    }
}

/// Now, on the clock the grid is drawn on.
///
/// `CLOCK_REALTIME`, because a sample's `nominal_time` is a grid point on that
/// clock and the difference between the two is the measurement worth having. A
/// clock before the epoch is reported as the epoch rather than raised: a driver
/// does not stop de-torquing a machine over a system clock nobody set.
#[must_use]
pub fn now_ns() -> i64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map_or(0, |since| {
            i64::try_from(since.as_nanos()).unwrap_or(i64::MAX)
        })
}

#[cfg(test)]
mod tests {
    use super::{Tick, TickConfig, cycle_timing, now_ns};
    use brenn_reachy__cogs__session_cmd_clk_rs::{SessionCmd, SessionCmdKind, SessionCmdWire};
    use brenn_reachy__driver__goal_clk_rs::{GoalSetpoint, GoalSetpointWire};
    use brenn_reachy__driver__health_clk_rs::AuxStatus;
    use brenn_reachy__driver__health_clk_rs::EventKind;
    use brenn_reachy__hardware__dynamixel__registers_clk_rs::{RegId, ValueShape};
    use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKind;
    use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointRef};
    use clockwork_rs::SyncTime;
    use dxl_proto::frame::{
        HEADER, INST_PING, INST_READ, INST_STATUS, INST_SYNC_READ, INST_SYNC_WRITE, INST_WRITE,
    };
    use dxl_proto::regs::{GOAL_POSITION, PRESENT_POSITION, TORQUE_ENABLE};
    use dxl_proto::regs::{HARDWARE_ERROR_STATUS, PRESENT_INPUT_VOLTAGE, PRESENT_TEMPERATURE};
    use dxl_proto::{Reg, crc16};
    use reachy_bus::DEFAULT_BAUD;
    use reachy_bus::{Bus, BusPort, ServoMap};
    use reachy_driver::{BLIND_CYCLES_BEFORE_BUS_FAILURE, TORQUE_OFF_CONFIRM_BUDGET_NS};
    use reachy_motion::arm::SERVO_IDS;
    use reachy_motion::joints::{ROW_COUNT, flags, rows_of, set_angle};
    use reachy_motion::value::{self, Value};
    use std::cell::RefCell;
    use std::collections::{HashMap, VecDeque};
    use std::io;
    use std::rc::Rc;
    use std::time::Instant;

    /// A round instant, so a number read out of the wrong field is visible.
    const T0: i64 = 1_700_000_000_000_000_000;

    /// What a ping of this fixture answers with, so a case can tell a model
    /// number read off the reply from a zero.
    const MODEL: u16 = 1_060;

    /// The firmware byte a ping's reply carries after the model number.
    const FIRMWARE: u8 = 45;

    /// The dead-man's window in these cases: ten cycles of the nominal grid.
    const HOLD_NS: i64 = 200_000_000;

    /// The rotation's spacing in these cases: five cycles between reports, so a
    /// case can tell a report that was due from one that was not.
    const HEALTH_NS: i64 = 100_000_000;

    /// A scripted nine-servo machine behind the port seam.
    ///
    /// TODO(shared-servo-fixture): the bench crate's `FakeMachine` is a third
    /// scripted servo model in this tree, and the two can disagree about what a
    /// servo does with a write.
    ///
    /// Answers exactly what a driver cycle asks: a grouped read of one register,
    /// a grouped write of one, and the unicast write-then-read a verified write
    /// is. Anything else gets silence, which is what makes a case about an
    /// instruction this driver should not be sending fail on a timeout rather
    /// than on a fixture that helpfully answered it.
    struct Machine {
        /// The control tables, by servo and address.
        regs: HashMap<(u8, u16), Vec<u8>>,
        /// Servos that answer nothing at all.
        silent: Vec<u8>,
        /// Registers a servo acknowledges a write to and does not store, which
        /// is what a read-back mismatch looks like from the host.
        ignored: Vec<(u8, u16)>,
        /// Registers a servo answers nothing about, while answering everything
        /// else: one cell of a control table that does not come back, which is
        /// what a task reading three of them has to survive.
        deaf: Vec<(u8, u16)>,
        /// Every write that reached a control table, in order.
        wrote: Vec<(u8, u16, Vec<u8>)>,
        /// Every unicast read this machine was asked for, in order, as (servo,
        /// address, width): what a task put on the wire one exchange at a time,
        /// which is what a per-cycle budget is spent on.
        read: Vec<(u8, u16, usize)>,
        /// The model number every servo here answers a ping with.
        model: u16,
        /// What the port answers a send with instead of taking it, the way a
        /// serial device that has been unplugged does. Persistent: the failure a
        /// driver has to survive is the one that does not go away.
        fail_write: Option<io::ErrorKind>,
        out: VecDeque<u8>,
    }

    impl Machine {
        /// A machine holding `counts` on every present-position register, with
        /// torque on and its goal registers where it stands.
        fn at(counts: i32) -> Self {
            let mut machine = Self {
                regs: HashMap::new(),
                silent: Vec::new(),
                ignored: Vec::new(),
                deaf: Vec::new(),
                wrote: Vec::new(),
                read: Vec::new(),
                model: MODEL,
                fail_write: None,
                out: VecDeque::new(),
            };
            for id in SERVO_IDS {
                machine.set(id, PRESENT_POSITION, &counts.to_le_bytes());
                machine.set(id, GOAL_POSITION, &counts.to_le_bytes());
                machine.set(id, TORQUE_ENABLE, &[1]);
            }
            machine
        }

        fn set(&mut self, id: u8, reg: Reg, bytes: &[u8]) {
            self.regs.insert((id, reg.addr), bytes.to_vec());
        }

        fn get(&self, id: u8, reg: Reg) -> Option<&[u8]> {
            self.regs.get(&(id, reg.addr)).map(Vec::as_slice)
        }

        /// A status frame as a servo puts it on the wire.
        fn reply(&mut self, id: u8, params: &[u8]) {
            let mut frame = Vec::from(HEADER);
            frame.push(id);
            let len = u16::try_from(params.len() + 4).expect("a fixture reply is short");
            frame.extend_from_slice(&len.to_le_bytes());
            frame.push(INST_STATUS);
            frame.push(0);
            frame.extend_from_slice(params);
            frame.extend_from_slice(&crc16(&frame).to_le_bytes());
            self.out.extend(frame);
        }

        fn store(&mut self, id: u8, addr: u16, bytes: &[u8]) {
            self.wrote.push((id, addr, bytes.to_vec()));
            if !self.ignored.contains(&(id, addr)) {
                self.regs.insert((id, addr), bytes.to_vec());
            }
        }

        fn answer(&mut self, id: u8, addr: u16, width: usize) {
            if self.silent.contains(&id) || self.deaf.contains(&(id, addr)) {
                return;
            }
            let mut value = self.regs.get(&(id, addr)).cloned().unwrap_or_default();
            value.resize(width, 0);
            self.reply(id, &value);
        }
    }

    impl BusPort for Machine {
        fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
            if let Some(kind) = self.fail_write {
                return Err(io::Error::from(kind));
            }
            let id = buf[4];
            let len = usize::from(u16::from_le_bytes([buf[5], buf[6]]));
            let instruction = buf[7];
            let params = &buf[8..8 + len - 3];
            if instruction == INST_PING {
                // A ping carries no parameters, so it is answered before
                // anything is read out of them.
                if !self.silent.contains(&id) {
                    let model = self.model.to_le_bytes();
                    self.reply(id, &[model[0], model[1], FIRMWARE]);
                }
                return Ok(());
            }
            let addr = u16::from_le_bytes([params[0], params[1]]);
            match instruction {
                INST_READ => {
                    let width = usize::from(u16::from_le_bytes([params[2], params[3]]));
                    self.read.push((id, addr, width));
                    self.answer(id, addr, width);
                }
                INST_WRITE => {
                    if self.silent.contains(&id) {
                        return Ok(());
                    }
                    self.store(id, addr, &params[2..]);
                    // A write is acknowledged with no parameters at all.
                    self.reply(id, &[]);
                }
                INST_SYNC_READ => {
                    let width = usize::from(u16::from_le_bytes([params[2], params[3]]));
                    for asked in params[4..].iter().copied() {
                        self.answer(asked, addr, width);
                    }
                }
                INST_SYNC_WRITE => {
                    // Acknowledged by nothing at all, so neither is this.
                    let width = usize::from(u16::from_le_bytes([params[2], params[3]]));
                    for entry in params[4..].chunks_exact(1 + width) {
                        let target = entry[0];
                        if self.silent.contains(&target) {
                            continue;
                        }
                        self.store(target, addr, &entry[1..]);
                    }
                }
                // Silence, so a case about an instruction this driver should not
                // send fails on the timeout rather than on a helpful fixture.
                _ => {}
            }
            Ok(())
        }

        fn read_some(&mut self, buf: &mut [u8], _deadline: Instant) -> io::Result<usize> {
            let mut taken = 0;
            while taken < buf.len() {
                match self.out.pop_front() {
                    Some(byte) => {
                        buf[taken] = byte;
                        taken += 1;
                    }
                    None => break,
                }
            }
            Ok(taken)
        }

        fn discard_input(&mut self) -> io::Result<()> {
            self.out.clear();
            Ok(())
        }
    }

    /// The port a case keeps a handle on while the bus holds it.
    struct Shared(Rc<RefCell<Machine>>);

    impl BusPort for Shared {
        fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
            self.0.borrow_mut().write_all(buf)
        }

        fn read_some(&mut self, buf: &mut [u8], deadline: Instant) -> io::Result<usize> {
            self.0.borrow_mut().read_some(buf, deadline)
        }

        fn discard_input(&mut self) -> io::Result<()> {
            self.0.borrow_mut().discard_input()
        }
    }

    /// A driver over a machine the case still holds, on the timing and the
    /// schedule the shipped configuration runs it on.
    fn driver(machine: Machine) -> (Tick<Shared>, Rc<RefCell<Machine>>) {
        driver_on(machine, config())
    }

    /// As [`driver`], on a cycle a case names — for the cases about what a cycle
    /// has no room to do.
    fn driver_on(machine: Machine, config: TickConfig) -> (Tick<Shared>, Rc<RefCell<Machine>>) {
        let shared = Rc::new(RefCell::new(machine));
        let bus = Bus::new(Shared(Rc::clone(&shared)), cycle_timing(DEFAULT_BAUD));
        (Tick::new(bus, config), shared)
    }

    /// A cycle far too short for a single bus exchange, so nothing out of band
    /// fits in it.
    fn cramped() -> TickConfig {
        TickConfig {
            period_ns: 1_000_000,
            ..config()
        }
    }

    /// The cycle's numbers: the nominal grid, a ten-cycle dead-man, and a
    /// health rotation slow enough that a case can tell one lap's reports
    /// apart.
    fn config() -> TickConfig {
        TickConfig {
            period_ns: reachy_driver::NOMINAL_CYCLE_NS,
            hold_timeout_ns: HOLD_NS,
            health_poll_period_ns: HEALTH_NS,
        }
    }

    /// One datagram from the host.
    fn command(kind: SessionCmdKind) -> SessionCmdWire {
        let mut message = SessionCmdWire::new();
        message.clear_valid().kind = kind;
        message
    }

    /// A host request to run one transaction.
    fn request(corr: u32, op: AuxOpKind, id: u8, reg: RegId, held: Value) -> SessionCmdWire {
        let mut message = SessionCmdWire::new();
        let cmd = message.clear_valid();
        cmd.kind = SessionCmdKind::Aux;
        cmd.corr = corr;
        cmd.txn.active = true.into();
        cmd.txn.op = op;
        cmd.txn.id = id;
        cmd.txn.reg = reg;
        cmd.txn.value_kind = held.shape();
        cmd.txn.value = held.bits();
        message
    }

    fn asked(message: &SessionCmdWire) -> &SessionCmd {
        message.validate().expect("a datagram written here is one")
    }

    /// The angle a servo holding `counts` reads as.
    fn angle_at(counts: i32) -> f64 {
        ServoMap::new(SERVO_IDS)
            .present_rad(0, counts)
            .expect("row 0 is a servo")
    }

    /// A setpoint for `joints`, every named row at `angle`.
    fn goal(at_ns: i64, joints: &[JointRef], angle: f64) -> GoalSetpointWire {
        let mut message = GoalSetpointWire::new();
        let setpoint = message.clear_valid();
        setpoint.execute_at = SyncTime::from_nanos(at_ns);
        let mut mask = JointFlags::NONE;
        for joint in joints {
            mask |= flags::bit(*joint);
            set_angle(&mut setpoint.targets, *joint, angle);
        }
        setpoint.mask = mask;
        message
    }

    fn read(message: &GoalSetpointWire) -> &GoalSetpoint {
        message.validate().expect("a setpoint written here is one")
    }

    #[test]
    fn a_cycle_reads_the_nine_positions_and_reports_them() {
        let (mut tick, _machine) = driver(Machine::at(2048));
        let before = now_ns();
        let report = tick.run(T0);
        let after = now_ns();
        let sample = report.sample.validate().expect("a sample this cycle wrote");

        assert_eq!(sample.nominal_time.as_nanos(), T0);
        assert!(
            (before..=after).contains(&sample.sample_time.as_nanos()),
            "the read's stamp is taken during the cycle, not at its grid instant",
        );
        assert!(bool::from(sample.present_valid));
        assert_eq!(sample.missing, JointFlags::NONE);
        assert!(!bool::from(sample.commanded_valid));
        for row in 0..ROW_COUNT {
            assert_eq!(rows_of(&sample.present)[row], angle_at(2048),);
        }
        assert_eq!(tick.counts().read_misses, 0);
        assert_eq!(tick.counts().blind_cycles, 0);
        assert!(report.event.is_none());
    }

    #[test]
    fn a_row_that_says_nothing_is_missing_and_the_others_are_not() {
        let mut machine = Machine::at(2048);
        machine.silent.push(SERVO_IDS[3]);
        let (mut tick, _machine) = driver(machine);

        let report = tick.run(T0);
        let sample = report.sample.validate().expect("a sample this cycle wrote");
        assert_eq!(sample.missing, flags::bit(JointRef::Leg2));
        assert!(!bool::from(sample.present_valid));
        assert_eq!(rows_of(&sample.present)[3], 0.0);
        assert_eq!(rows_of(&sample.present)[4], angle_at(2048));
        assert_eq!(tick.counts().read_misses, 1);
        // One row is not the bus: a partial reading is a reading.
        assert_eq!(tick.counts().blind_cycles, 0);
    }

    #[test]
    fn a_bus_that_answers_nothing_says_so_once_the_run_is_long_enough() {
        let mut machine = Machine::at(2048);
        machine.silent.extend(SERVO_IDS);
        let (mut tick, _machine) = driver(machine);

        for cycle in 0..BLIND_CYCLES_BEFORE_BUS_FAILURE - 1 {
            let report = tick.run(T0 + i64::from(cycle) * 20_000_000);
            assert!(report.event.is_none(), "cycle {cycle} spoke too early");
            let sample = report.sample.validate().expect("a sample every cycle");
            assert_eq!(sample.missing, flags::all());
            assert!(!bool::from(sample.present_valid));
        }
        let at_threshold = BLIND_CYCLES_BEFORE_BUS_FAILURE - 1;
        let report = tick.run(T0 + i64::from(at_threshold) * 20_000_000);
        let event = report
            .event
            .as_ref()
            .expect("the run reaching its length is the event")
            .validate()
            .expect("an event this cycle wrote");
        assert_eq!(event.kind, EventKind::BusFailure);
        assert_eq!(event.count, BLIND_CYCLES_BEFORE_BUS_FAILURE);

        // A standing outage is not news.
        let next = tick.run(T0 + i64::from(BLIND_CYCLES_BEFORE_BUS_FAILURE) * 20_000_000);
        assert!(next.event.is_none());
        assert_eq!(
            tick.counts().blind_cycles,
            u64::from(BLIND_CYCLES_BEFORE_BUS_FAILURE) + 1,
        );
    }

    #[test]
    fn a_due_goal_is_written_to_the_rows_it_names_and_nowhere_else() {
        let (mut tick, machine) = driver(Machine::at(2048));
        let commanded = angle_at(2200);
        let setpoint = goal(T0, &[JointRef::BodyYaw, JointRef::AntennaLeft], commanded);
        tick.offer_goal(read(&setpoint), T0);

        let report = tick.run(T0);
        let sample = report.sample.validate().expect("a sample this cycle wrote");
        assert!(bool::from(sample.commanded_valid));
        assert_eq!(tick.counts().goals_executed, 1);

        let held = machine.borrow();
        let written: Vec<u8> = held
            .wrote
            .iter()
            .filter(|(_, addr, _)| *addr == GOAL_POSITION.addr)
            .map(|(id, _, _)| *id)
            .collect();
        assert_eq!(written, vec![SERVO_IDS[0], SERVO_IDS[8]]);
        assert_eq!(
            held.get(SERVO_IDS[0], GOAL_POSITION),
            Some(&2200i32.to_le_bytes()[..]),
        );
        // Every other row is holding what it was holding.
        assert_eq!(
            held.get(SERVO_IDS[4], GOAL_POSITION),
            Some(&2048i32.to_le_bytes()[..]),
        );
        assert_eq!(tick.counts().write_failures, 0);
    }

    #[test]
    fn the_held_setpoint_is_rewritten_every_cycle_and_counted_once() {
        let (mut tick, machine) = driver(Machine::at(2048));
        let setpoint = goal(T0, &[JointRef::BodyYaw], angle_at(2200));
        tick.offer_goal(read(&setpoint), T0);

        tick.run(T0);
        tick.run(T0 + 20_000_000);
        tick.run(T0 + 40_000_000);

        let writes = machine
            .borrow()
            .wrote
            .iter()
            .filter(|(_, addr, _)| *addr == GOAL_POSITION.addr)
            .count();
        assert_eq!(writes, 3, "the position loop is held awake between goals");
        assert_eq!(tick.counts().goals_executed, 1);
    }

    #[test]
    fn a_goal_no_servo_count_represents_is_refused_rather_than_rounded() {
        let (mut tick, machine) = driver(Machine::at(2048));
        // An angle no count of this servo's encoder represents at all. The
        // envelope check runs above this process; what a driver does with an
        // angle it cannot even encode is refuse it.
        let setpoint = goal(T0, &[JointRef::BodyYaw], 1e12);
        tick.offer_goal(read(&setpoint), T0);

        tick.run(T0);
        assert_eq!(tick.counts().write_failures, 1);
        assert_eq!(
            machine.borrow().get(SERVO_IDS[0], GOAL_POSITION),
            Some(&2048i32.to_le_bytes()[..]),
            "nothing reached the servo, and nothing was rounded to fit it",
        );
    }

    #[test]
    fn a_goal_offered_to_a_full_queue_is_dropped_and_said_so() {
        let (mut tick, _machine) = driver(Machine::at(2048));
        // Every one dated for the future, so none is taken as due and the queue
        // fills: five is the gate's own capacity.
        for step in 1..=5 {
            let setpoint = goal(T0 + step * 20_000_000, &[JointRef::BodyYaw], angle_at(2100));
            tick.offer_goal(read(&setpoint), T0);
        }
        let sixth = goal(T0 + 6 * 20_000_000, &[JointRef::BodyYaw], angle_at(2100));
        tick.offer_goal(read(&sixth), T0);

        assert_eq!(tick.counts().goals_dropped, 1);
        let report = tick.run(T0);
        let event = report
            .event
            .as_ref()
            .expect("a dropped goal is a remark to its sender")
            .validate()
            .expect("an event this cycle wrote");
        assert_eq!(event.kind, EventKind::GoalDroppedQueueFull);
        assert_eq!(event.count, 5);
    }

    #[test]
    fn a_goal_that_arrives_past_its_instant_reports_how_late_it_was() {
        let (mut tick, _machine) = driver(Machine::at(2048));
        let setpoint = goal(T0 - 60_000_000, &[JointRef::BodyYaw], angle_at(2100));
        tick.offer_goal(read(&setpoint), T0);

        let report = tick.run(T0);
        let event = report
            .event
            .as_ref()
            .expect("lateness is the sender's to fix and so is reported")
            .validate()
            .expect("an event this cycle wrote");
        assert_eq!(event.kind, EventKind::GoalStaleOrOutOfOrder);
        assert_eq!(event.silence.as_nanos(), 60_000_000);
        // Executed anyway, in arrival order.
        assert_eq!(tick.counts().goals_executed, 1);
    }

    #[test]
    fn a_requested_de_torquing_sweeps_every_row_verified_and_keeps_sweeping() {
        let (mut tick, machine) = driver(Machine::at(2048));
        let setpoint = goal(T0, &[JointRef::BodyYaw], angle_at(2200));
        tick.offer_goal(read(&setpoint), T0);
        tick.request_torque_off(T0);

        let report = tick.run(T0);
        let sample = report.sample.validate().expect("a sample this cycle wrote");
        assert!(bool::from(sample.torque_off_latched));
        // No event: the host asked, so it already knows.
        assert!(report.event.is_none());
        assert_eq!(tick.counts().write_failures, 0);

        let held = machine.borrow();
        for id in SERVO_IDS {
            assert_eq!(held.get(id, TORQUE_ENABLE), Some(&[0][..]), "servo {id}");
        }
        // The queued setpoint went with the latch: a release outranks whatever
        // was asked for before it.
        assert!(
            !held
                .wrote
                .iter()
                .any(|(_, addr, _)| *addr == GOAL_POSITION.addr),
        );
        drop(held);

        // A standing latch asks for the sweep again, every cycle.
        tick.run(T0 + 20_000_000);
        let sweeps = machine
            .borrow()
            .wrote
            .iter()
            .filter(|(_, addr, _)| *addr == TORQUE_ENABLE.addr)
            .count();
        assert_eq!(sweeps, 2 * ROW_COUNT);
    }

    #[test]
    fn a_row_whose_release_did_not_read_back_is_counted_and_the_rest_swept() {
        let mut machine = Machine::at(2048);
        // Acknowledged and not stored, which is what a servo that ignored a
        // write looks like from the host.
        machine.ignored.push((SERVO_IDS[5], TORQUE_ENABLE.addr));
        let (mut tick, machine) = driver(machine);
        tick.request_torque_off(T0);

        tick.run(T0);
        assert_eq!(tick.counts().write_failures, 1);
        let held = machine.borrow();
        assert_eq!(held.get(SERVO_IDS[5], TORQUE_ENABLE), Some(&[1][..]));
        for (index, id) in SERVO_IDS.into_iter().enumerate() {
            if index == 5 {
                continue;
            }
            assert_eq!(
                held.get(id, TORQUE_ENABLE),
                Some(&[0][..]),
                "nothing gates the rest of the sweep",
            );
        }
    }

    #[test]
    fn a_second_remark_in_one_cycle_is_counted_and_the_first_stands() {
        let (mut tick, _machine) = driver(Machine::at(2048));
        for _ in 0..2 {
            let setpoint = goal(T0 - 60_000_000, &[JointRef::BodyYaw], angle_at(2100));
            tick.offer_goal(read(&setpoint), T0);
        }

        assert_eq!(tick.counts().events_dropped, 1);
        let report = tick.run(T0);
        let event = report
            .event
            .as_ref()
            .expect("the first remark is the one published")
            .validate()
            .expect("an event this cycle wrote");
        assert_eq!(event.kind, EventKind::GoalStaleOrOutOfOrder);
    }

    /// The health rotation is the cycle's cheapest task and the first thing a
    /// budget too tight for anything would starve, so the shipped timing is
    /// pinned by a report arriving at all.
    #[test]
    fn the_cycles_own_timing_leaves_room_for_one_out_of_band_transaction() {
        let mut machine = Machine::at(2048);
        for id in SERVO_IDS {
            machine.set(id, HARDWARE_ERROR_STATUS, &[0]);
            machine.set(id, PRESENT_INPUT_VOLTAGE, &120u16.to_le_bytes());
            machine.set(id, PRESENT_TEMPERATURE, &[37]);
        }
        let (mut tick, _machine) = driver(machine);

        let report = tick.run(T0);
        let health = report
            .health
            .as_ref()
            .expect("a driver that has just started owes a reading of every servo")
            .validate()
            .expect("a report this cycle wrote");
        assert_eq!(health.id, SERVO_IDS[0]);
        assert_eq!(health.bits, 0);
        assert!((health.volts - 12.0).abs() < 1e-9);
        assert_eq!(health.temp_c, 37);
        assert_eq!(health.sample_time.as_nanos(), {
            let sample = report.sample.validate().expect("a sample every cycle");
            sample.sample_time.as_nanos()
        });
        assert_eq!(tick.counts().health_reports, 1);
        assert_eq!(tick.counts().aux_deferred, 0);
    }

    /// The out-of-band budget is charged for every read the health task makes.
    ///
    /// The budget names three register widths and the task makes three reads,
    /// and neither list can see the other: a fourth read added to the task
    /// without widening the budget under-charges the rotation's slice of the
    /// cycle, and the failure surfaces as an overrun on hardware rather than
    /// here. So the reads are counted off the wire and priced by the same
    /// timing, and the budget has to cover them.
    #[test]
    fn the_aux_budget_covers_the_reads_the_health_task_makes() {
        let mut machine = Machine::at(2048);
        for id in SERVO_IDS {
            machine.set(id, HARDWARE_ERROR_STATUS, &[0]);
            machine.set(id, PRESENT_INPUT_VOLTAGE, &120u16.to_le_bytes());
            machine.set(id, PRESENT_TEMPERATURE, &[37]);
        }
        let (mut tick, machine) = driver(machine);

        assert!(
            tick.run(T0).health.is_some(),
            "a driver that has just started owes a reading"
        );

        // The rotation's reads are the only unicast ones a plain cycle makes:
        // the pose is a grouped read and the goals a grouped write, and nothing
        // here sweeps torque or asks for an out-of-band transaction.
        let timing = cycle_timing(DEFAULT_BAUD);
        let reads = machine.borrow().read.clone();
        assert!(!reads.is_empty(), "the task read something");
        let charged: std::time::Duration = reads
            .iter()
            .map(|(_, _, width)| timing.read_reg_bound(*width))
            .sum();
        let charged_ns = i64::try_from(charged.as_nanos()).expect("a bound inside the grid");
        assert!(
            crate::aux::CycleBounds::of(&timing).aux_ns >= charged_ns,
            "the budget charges {charged_ns} ns of reads: {reads:?}",
        );
    }

    #[test]
    fn the_rotation_walks_one_row_per_period_and_not_faster() {
        let mut machine = Machine::at(2048);
        for id in SERVO_IDS {
            machine.set(id, HARDWARE_ERROR_STATUS, &[0]);
        }
        let (mut tick, _machine) = driver(machine);

        let first = tick.run(T0);
        assert_eq!(
            first
                .health
                .expect("row 0 is due")
                .validate()
                .expect("a report this cycle wrote")
                .id,
            SERVO_IDS[0],
        );
        // Inside the rotation's own spacing: due to nobody, so nothing is read.
        for cycle in 1..5 {
            let report = tick.run(T0 + i64::from(cycle) * 20_000_000);
            assert!(report.health.is_none(), "cycle {cycle} read a servo early");
        }
        let next = tick.run(T0 + HEALTH_NS);
        assert_eq!(
            next.health
                .expect("the rotation walks on")
                .validate()
                .expect("a report this cycle wrote")
                .id,
            SERVO_IDS[1],
        );
        assert_eq!(tick.counts().health_reports, 2);
    }

    /// A temperature this record cannot state is no report at all, in both of
    /// the ways it can happen: a cell that does not come back, and a byte too
    /// large for the `Int8` the report carries.
    ///
    /// The alternative is a plausible number beside a real error byte, which is
    /// the one shape of health reading nobody can act on. The rotation walks on
    /// either way, so the servo behind it is still read.
    #[test]
    fn a_temperature_the_report_cannot_carry_costs_the_report_and_nothing_else() {
        let mut machine = Machine::at(2048);
        for id in SERVO_IDS {
            machine.set(id, HARDWARE_ERROR_STATUS, &[0]);
            machine.set(id, PRESENT_TEMPERATURE, &[40]);
        }
        machine.deaf.push((SERVO_IDS[0], PRESENT_TEMPERATURE.addr));
        machine.set(SERVO_IDS[1], PRESENT_TEMPERATURE, &[200]);
        let (mut tick, _machine) = driver(machine);

        assert!(
            tick.run(T0).health.is_none(),
            "a servo that answered two of the three reads is not a reading",
        );
        assert!(
            tick.run(T0 + HEALTH_NS).health.is_none(),
            "200 degrees is not a number this record can state",
        );
        assert_eq!(tick.counts().health_misses, 2);

        let third = tick.run(T0 + 2 * HEALTH_NS);
        let health = third
            .health
            .as_ref()
            .expect("the rotation walks on to a servo that answers")
            .validate()
            .expect("a report this cycle wrote");
        assert_eq!(health.id, SERVO_IDS[2]);
        assert_eq!(health.temp_c, 40);
        assert_eq!(tick.counts().health_reports, 1);
    }

    #[test]
    fn a_servo_that_answers_nothing_gets_no_health_report_at_all() {
        let mut machine = Machine::at(2048);
        machine.silent.push(SERVO_IDS[0]);
        let (mut tick, _machine) = driver(machine);

        let report = tick.run(T0);
        assert!(
            report.health.is_none(),
            "a report of zeroes about a machine nobody heard from is worse than none",
        );
        assert_eq!(tick.counts().health_misses, 1);
        // The cadence was stamped when the read was named, so the rotation walks
        // on to a servo that does answer.
        let next = tick.run(T0 + HEALTH_NS);
        assert_eq!(
            next.health
                .expect("the next row answers")
                .validate()
                .expect("a report this cycle wrote")
                .id,
            SERVO_IDS[1],
        );
    }

    #[test]
    fn a_host_read_outranks_the_rotation_and_comes_back_with_the_register() {
        let (mut tick, _machine) = driver(Machine::at(2048));
        let ask = request(
            7,
            AuxOpKind::ReadReg,
            SERVO_IDS[2],
            RegId::PresentPosition,
            value::NONE,
        );
        tick.offer_session_cmd(asked(&ask), T0);

        let report = tick.run(T0);
        let outcome = report
            .outcome
            .as_ref()
            .expect("a host request is answered on the cycle it runs")
            .validate()
            .expect("an outcome this cycle wrote");
        assert_eq!(outcome.corr, 7);
        assert_eq!(outcome.status, AuxStatus::Ok);
        assert_eq!(outcome.value_kind, ValueShape::Radians);
        assert_eq!(
            value::carried(outcome.value_kind, outcome.value).as_radians(),
            Some(angle_at(2048)),
        );
        assert!(
            report.health.is_none(),
            "the host's request took the cycle's one transaction",
        );
    }

    #[test]
    fn a_verified_write_the_servo_ignored_comes_back_as_a_mismatch() {
        let mut machine = Machine::at(2048);
        // Acknowledged and not stored: the read-back says what it still holds.
        machine.ignored.push((SERVO_IDS[0], TORQUE_ENABLE.addr));
        let (mut tick, _machine) = driver(machine);
        let ask = request(
            3,
            AuxOpKind::WriteRegVerified,
            SERVO_IDS[0],
            RegId::TorqueEnable,
            value::u8(0),
        );
        tick.offer_session_cmd(asked(&ask), T0);

        let report = tick.run(T0);
        let outcome = report
            .outcome
            .as_ref()
            .expect("a write that did not take is still an answer")
            .validate()
            .expect("an outcome this cycle wrote");
        assert_eq!(outcome.corr, 3);
        assert_eq!(outcome.status, AuxStatus::VerifyMismatch);
        // What the servo holds, not what was sent.
        assert_eq!(
            value::carried(outcome.value_kind, outcome.value).as_u8(),
            Some(1),
        );
    }

    #[test]
    fn a_transaction_naming_a_servo_this_machine_does_not_have_reaches_no_wire() {
        let (mut tick, machine) = driver(Machine::at(2048));
        let ask = request(
            11,
            AuxOpKind::WriteRegVerified,
            99,
            RegId::TorqueEnable,
            value::u8(0),
        );
        tick.offer_session_cmd(asked(&ask), T0);

        let report = tick.run(T0);
        let outcome = report
            .outcome
            .as_ref()
            .expect("a refusal is an answer against the number that asked")
            .validate()
            .expect("an outcome this cycle wrote");
        assert_eq!(outcome.corr, 11);
        assert_eq!(outcome.status, AuxStatus::Refused);
        assert!(
            !machine
                .borrow()
                .wrote
                .iter()
                .any(|(id, _, _)| *id == 99 || *id == 0xFE),
        );
    }

    #[test]
    fn a_second_request_while_one_is_pending_is_refused_by_its_own_number() {
        let (mut tick, _machine) = driver(Machine::at(2048));
        let first = request(
            1,
            AuxOpKind::ReadReg,
            SERVO_IDS[0],
            RegId::PresentPosition,
            value::NONE,
        );
        let second = request(
            2,
            AuxOpKind::ReadReg,
            SERVO_IDS[1],
            RegId::PresentPosition,
            value::NONE,
        );
        tick.offer_session_cmd(asked(&first), T0);
        tick.offer_session_cmd(asked(&second), T0);

        assert_eq!(tick.counts().aux_refused, 1);
        let report = tick.run(T0);
        let outcome = report
            .outcome
            .as_ref()
            .expect("the turned-away request is answered")
            .validate()
            .expect("an outcome this cycle wrote");
        // Against the number that was refused, and the one that was accepted
        // runs on this cycle and comes back to a host whose answer the refusal
        // displaced -- which is what its own re-issue is for.
        assert_eq!(outcome.corr, 2);
        assert_eq!(outcome.status, AuxStatus::Refused);
    }

    #[test]
    fn a_datagram_asking_nothing_is_counted_and_is_not_liveness() {
        let (mut tick, _machine) = driver(Machine::at(2048));
        let nothing = command(SessionCmdKind::None);
        tick.offer_session_cmd(asked(&nothing), T0);

        assert_eq!(tick.counts().aux_refused, 1);
        // Feeding the dead-man off bytes nobody meant would hold a machine
        // energised on the strength of noise, so nothing was fed: with nothing
        // believed torqued there is nothing to latch either, and the cycle is
        // silent.
        let report = tick.run(T0);
        assert!(report.event.is_none());
    }

    #[test]
    fn a_de_torquing_every_row_reads_back_released_is_confirmed_once() {
        let (mut tick, _machine) = driver(Machine::at(2048));
        tick.offer_session_cmd(asked(&command(SessionCmdKind::TorqueOffNow)), T0);

        let mut confirmed = None;
        for cycle in 0..=ROW_COUNT {
            let at = T0 + (cycle as i64) * 20_000_000;
            let report = tick.run(at);
            if let Some(event) = report.event.as_ref() {
                let event = event.validate().expect("an event this cycle wrote");
                assert_eq!(event.kind, EventKind::TorqueOffConfirmed);
                assert!(confirmed.is_none(), "said twice");
                confirmed = Some(cycle);
            }
        }
        // One read-back per cycle, nine rows, and the pass reports on the cycle
        // after the last of them comes back clean.
        assert_eq!(confirmed, Some(ROW_COUNT));
        assert_eq!(tick.counts().confirm_misses, 0);
    }

    #[test]
    fn a_de_torquing_no_row_reads_back_is_unconfirmed_with_the_rows_unseen() {
        let mut machine = Machine::at(2048);
        // Every row acknowledges its release and keeps holding, which is the
        // one thing a driver must never credit as a de-torquing.
        for id in SERVO_IDS {
            machine.ignored.push((id, TORQUE_ENABLE.addr));
        }
        let (mut tick, _machine) = driver(machine);
        tick.offer_session_cmd(asked(&command(SessionCmdKind::TorqueOffNow)), T0);

        let mut unconfirmed = None;
        let cycles = TORQUE_OFF_CONFIRM_BUDGET_NS / 20_000_000 + 2;
        for cycle in 0..cycles {
            let report = tick.run(T0 + cycle * 20_000_000);
            if let Some(event) = report.event.as_ref() {
                let event = event.validate().expect("an event this cycle wrote");
                assert_eq!(event.kind, EventKind::TorqueOffUnconfirmed);
                assert_eq!(
                    event.rows,
                    flags::all(),
                    "no row has been seen released, so every one of them is named",
                );
                unconfirmed = Some(cycle);
                break;
            }
        }
        assert!(unconfirmed.is_some(), "a budget that ran out is reported");
        // And the sweep keeps going out: nothing gates de-torquing, least of
        // all a report that it has not been confirmed.
        let swept = tick.counts();
        tick.run(T0 + cycles * 20_000_000);
        assert_eq!(
            swept.write_failures + ROW_COUNT as u64,
            tick.counts().write_failures
        );
    }

    #[test]
    fn the_dead_man_latches_once_a_verified_write_makes_the_driver_believe() {
        let (mut tick, machine) = driver(Machine::at(2048));
        // A goal, so the gate has accepted something, and a verified
        // torque-enable write, so the driver believes a row is holding: both
        // halves the dead-man measures against.
        let setpoint = goal(T0, &[JointRef::BodyYaw], angle_at(2100));
        tick.offer_goal(read(&setpoint), T0);
        let arm = request(
            5,
            AuxOpKind::WriteRegVerified,
            SERVO_IDS[0],
            RegId::TorqueEnable,
            value::u8(1),
        );
        tick.offer_session_cmd(asked(&arm), T0);
        let report = tick.run(T0);
        assert_eq!(
            report
                .outcome
                .expect("the write is answered")
                .validate()
                .expect("an outcome this cycle wrote")
                .status,
            AuxStatus::Ok,
        );

        // Nothing else arrives. The dead-man's window is ten cycles.
        let mut latched = None;
        for cycle in 1..=12 {
            let report = tick.run(T0 + cycle * 20_000_000);
            if let Some(event) = report.event.as_ref() {
                let event = event.validate().expect("an event this cycle wrote");
                assert_eq!(event.kind, EventKind::HoldTimeoutTorqueOff);
                assert_eq!(event.silence.as_nanos(), cycle * 20_000_000);
                latched = Some(cycle);
                break;
            }
        }
        assert_eq!(latched, Some(11), "the window is ten cycles, exclusive");
        assert_eq!(tick.counts().hold_timeouts, 1);
        assert_eq!(
            machine.borrow().get(SERVO_IDS[0], TORQUE_ENABLE),
            Some(&[0][..]),
            "silence de-torques the machine",
        );
    }

    #[test]
    fn a_ping_comes_back_with_the_model_number_the_servo_answered() {
        let (mut tick, _machine) = driver(Machine::at(2048));
        let asked_for = request(9, AuxOpKind::Ping, SERVO_IDS[3], RegId::None, value::NONE);
        tick.offer_session_cmd(asked(&asked_for), T0);

        let report = tick.run(T0);
        let outcome = report
            .outcome
            .as_ref()
            .expect("the ping is answered")
            .validate()
            .expect("an outcome this cycle wrote");
        assert_eq!(outcome.corr, 9);
        assert_eq!(outcome.status, AuxStatus::Ok);
        assert_eq!(
            outcome.model, MODEL,
            "the reply's own model number, which is the whole point of asking"
        );
        // A ping reads no register, so there is no value to carry and the shape
        // says so rather than carrying a zero somebody could read as one.
        assert_eq!(outcome.value_kind, ValueShape::None);
        assert_eq!(outcome.value, 0);
    }

    #[test]
    fn a_servo_that_answers_nothing_fails_a_ping_as_a_silence() {
        let mut machine = Machine::at(2048);
        machine.silent.push(SERVO_IDS[3]);
        let (mut tick, _machine) = driver(machine);
        let asked_for = request(9, AuxOpKind::Ping, SERVO_IDS[3], RegId::None, value::NONE);
        tick.offer_session_cmd(asked(&asked_for), T0);

        let outcome = tick
            .run(T0)
            .outcome
            .as_ref()
            .expect("a silence is answered too")
            .validate()
            .expect("an outcome this cycle wrote")
            .status;
        assert_eq!(
            outcome,
            AuxStatus::Timeout,
            "the datagram went out and nothing came back, which is not a refusal"
        );
    }

    #[test]
    fn a_refused_request_is_counted_the_same_way_a_datagram_asking_nothing_is() {
        let (mut tick, _machine) = driver(Machine::at(2048));
        let first = request(
            1,
            AuxOpKind::ReadReg,
            SERVO_IDS[0],
            RegId::PresentPosition,
            value::NONE,
        );
        let second = request(
            2,
            AuxOpKind::ReadReg,
            SERVO_IDS[1],
            RegId::PresentPosition,
            value::NONE,
        );
        let nothing = command(SessionCmdKind::None);

        tick.offer_session_cmd(asked(&first), T0);
        tick.offer_session_cmd(asked(&second), T0);
        tick.offer_session_cmd(asked(&nothing), T0);

        // The two refusals are one condition as far as the dead-man is
        // concerned: a datagram the driver did not act on is not evidence that
        // the host commanding this machine is still there, which is the rule the
        // goal path runs on a queue it could not take a setpoint into.
        assert_eq!(tick.counts().aux_refused, 2);
        assert!(
            tick.run(T0).event.is_none(),
            "neither refusal fed the dead-man, so nothing latched and nothing was said"
        );
    }

    /// The transaction that arms one row: a verified torque-enable write.
    fn arming(corr: u32, id: u8, on: bool) -> SessionCmdWire {
        request(
            corr,
            AuxOpKind::WriteRegVerified,
            id,
            RegId::TorqueEnable,
            value::u8(u8::from(on)),
        )
    }

    /// How many torque-enable writes have reached the machine so far.
    fn torque_writes(machine: &Rc<RefCell<Machine>>) -> usize {
        machine
            .borrow()
            .wrote
            .iter()
            .filter(|(_, addr, _)| *addr == TORQUE_ENABLE.addr)
            .count()
    }

    #[test]
    fn an_arming_ends_a_standing_latch_and_the_sweep_stops() {
        let (mut tick, machine) = driver(Machine::at(2048));
        tick.offer_session_cmd(asked(&command(SessionCmdKind::TorqueOffNow)), T0);
        tick.run(T0);
        assert!(
            bool::from(
                tick.run(T0 + 20_000_000)
                    .sample
                    .validate()
                    .expect("a sample every cycle")
                    .torque_off_latched
            ),
            "the latch stands, and the sweep goes out under it",
        );

        // The way back: a host arms a row with a verified write. A latched gate
        // sweeps on every cycle and a sweep costs more than a period, so this is
        // also the case that says the budget does not lock the latch in — a
        // driver whose slot is never asked while latched can never be armed
        // again for the life of the process.
        tick.offer_session_cmd(asked(&arming(3, SERVO_IDS[0], true)), T0 + 40_000_000);
        let report = tick.run(T0 + 40_000_000);
        assert_eq!(
            report
                .outcome
                .expect("the arming is answered, swept cycle or not")
                .validate()
                .expect("an outcome this cycle wrote")
                .status,
            AuxStatus::Ok,
        );
        assert!(
            !bool::from(
                report
                    .sample
                    .validate()
                    .expect("a sample every cycle")
                    .torque_off_latched
            ),
            "the arming released the latch",
        );

        // And the sweep is over: the next cycle writes no torque-enable register
        // at all, and nothing reports on the release the arming superseded.
        let before = torque_writes(&machine);
        let next = tick.run(T0 + 60_000_000);
        assert_eq!(
            torque_writes(&machine),
            before,
            "a machine somebody armed is not swept",
        );
        assert!(
            next.event.is_none(),
            "the confirmation pass stood down, so there is nothing to report about it",
        );
        assert_eq!(
            machine.borrow().get(SERVO_IDS[0], TORQUE_ENABLE),
            Some(&[1][..]),
            "the row the host armed is holding",
        );
    }

    #[test]
    fn a_release_that_leaves_nothing_believed_holding_clears_what_was_commanded() {
        let (mut tick, _machine) = driver(Machine::at(2048));
        let setpoint = goal(T0, &[JointRef::BodyYaw], angle_at(2100));
        tick.offer_goal(read(&setpoint), T0);
        tick.offer_session_cmd(asked(&arming(1, SERVO_IDS[0], true)), T0);
        let report = tick.run(T0);
        assert!(
            bool::from(
                report
                    .sample
                    .validate()
                    .expect("a sample every cycle")
                    .commanded_valid
            ),
            "a setpoint was written, so the sample carries what was commanded",
        );

        // The one believed row goes limp, so the driver believes nothing is
        // holding — and a limp machine is holding no setpoint.
        tick.offer_session_cmd(asked(&arming(2, SERVO_IDS[0], false)), T0 + 20_000_000);
        let report = tick.run(T0 + 20_000_000);
        assert_eq!(
            report
                .outcome
                .expect("the release is answered")
                .validate()
                .expect("an outcome this cycle wrote")
                .status,
            AuxStatus::Ok,
        );
        assert!(
            !bool::from(
                report
                    .sample
                    .validate()
                    .expect("a sample every cycle")
                    .commanded_valid
            ),
            "nothing is believed holding, so nothing is commanded",
        );
    }

    #[test]
    fn a_cycle_with_no_room_holds_the_host_request_rather_than_dropping_it() {
        let (mut tick, machine) = driver_on(Machine::at(2048), cramped());
        let asked_for = request(
            4,
            AuxOpKind::ReadReg,
            SERVO_IDS[2],
            RegId::PresentPosition,
            value::NONE,
        );
        tick.offer_session_cmd(asked(&asked_for), T0);

        for cycle in 0..3 {
            let report = tick.run(T0 + cycle * 20_000_000);
            assert!(
                report.outcome.is_none() && report.health.is_none(),
                "nothing out of band ran on cycle {cycle}",
            );
            assert_eq!(
                tick.counts().aux_deferred,
                u64::try_from(cycle + 1).expect("three cycles"),
                "one deferral per cycle that had work and no room for it",
            );
        }
        // Held, not dropped: the slot still has it, which is why a second
        // request is refused as busy rather than accepted.
        tick.offer_session_cmd(asked(&asked_for), T0 + 60_000_000);
        let report = tick.run(T0 + 60_000_000);
        assert_eq!(
            report
                .outcome
                .expect("the turned-away request is answered")
                .validate()
                .expect("an outcome this cycle wrote")
                .status,
            AuxStatus::Refused,
        );
        assert_eq!(
            machine
                .borrow()
                .wrote
                .iter()
                .filter(|(_, addr, _)| *addr != GOAL_POSITION.addr)
                .count(),
            0,
            "a deferred transaction reached no register",
        );
    }

    #[test]
    fn a_confirmation_read_back_runs_in_a_cycle_that_has_no_room_for_anything() {
        let (mut tick, _machine) = driver_on(Machine::at(2048), cramped());
        tick.offer_session_cmd(asked(&command(SessionCmdKind::TorqueOffNow)), T0);

        // Nothing gates de-torquing, and the read-back is the evidence a
        // commanded one took, so it is exempt from the budget: the pass walks its
        // nine rows on cycles that have room for nothing at all.
        let mut confirmed = None;
        for cycle in 0..=ROW_COUNT {
            let report = tick.run(T0 + (cycle as i64) * 20_000_000);
            if let Some(event) = report.event.as_ref() {
                assert_eq!(
                    event.validate().expect("an event this cycle wrote").kind,
                    EventKind::TorqueOffConfirmed,
                );
                confirmed = Some(cycle);
                break;
            }
            assert_eq!(
                tick.counts().aux_deferred,
                0,
                "cycle {cycle} ran the read-back rather than deferring it",
            );
        }
        assert_eq!(confirmed, Some(ROW_COUNT));
        assert_eq!(tick.counts().confirm_misses, 0);
        // From here the pass is over and the rotation is what wants the slot,
        // which a cycle this short has no room for: that is the arm the counter
        // is about, and it moves once per cycle from here.
        let before = tick.counts().aux_deferred;
        tick.run(T0 + ((ROW_COUNT + 1) as i64) * 20_000_000);
        assert_eq!(tick.counts().aux_deferred, before + 1);
    }

    #[test]
    fn a_port_that_refuses_the_send_claims_no_reading_at_all() {
        let mut machine = Machine::at(2048);
        machine.fail_write = Some(io::ErrorKind::BrokenPipe);
        let (mut tick, _machine) = driver_on(machine, config());

        let report = tick.run(T0);
        let sample = report.sample.validate().expect("a sample every cycle");
        // The frame this process could not send is the one case where a driver
        // could credit itself a reading it never took. It claims least instead:
        // every row missing, and the cycle blind.
        assert_eq!(sample.missing, flags::all());
        assert!(!bool::from(sample.present_valid));
        assert_eq!(tick.counts().read_misses, ROW_COUNT as u64);
        assert_eq!(tick.counts().write_failures, 1);
        assert_eq!(tick.counts().blind_cycles, 1);
        assert!(report.event.is_none(), "one cycle is not a run of them");

        // And a run of them reaches the ladder's own rung.
        let mut failure = None;
        for cycle in 1..=BLIND_CYCLES_BEFORE_BUS_FAILURE {
            let report = tick.run(T0 + i64::from(cycle) * 20_000_000);
            if let Some(event) = report.event.as_ref() {
                let event = event.validate().expect("an event this cycle wrote");
                assert_eq!(event.kind, EventKind::BusFailure);
                assert_eq!(event.count, BLIND_CYCLES_BEFORE_BUS_FAILURE);
                failure = Some(cycle);
                break;
            }
        }
        assert_eq!(
            failure,
            Some(BLIND_CYCLES_BEFORE_BUS_FAILURE - 1),
            "a dead port is a blind bus, counted the same as a silent one",
        );
    }

    #[test]
    fn a_goal_write_the_port_refused_reaches_no_register_and_is_counted() {
        let (mut tick, machine) = driver(Machine::at(2048));
        let setpoint = goal(T0, &[JointRef::BodyYaw], angle_at(2200));
        tick.offer_goal(read(&setpoint), T0);
        machine.borrow_mut().fail_write = Some(io::ErrorKind::BrokenPipe);

        tick.run(T0);

        assert_eq!(
            tick.counts().write_failures,
            2,
            "the read that went nowhere and the write that went nowhere",
        );
        assert!(
            !machine
                .borrow()
                .wrote
                .iter()
                .any(|(_, addr, _)| *addr == GOAL_POSITION.addr),
            "the setpoint reached no register, so the write is unconfirmed",
        );
    }
}
