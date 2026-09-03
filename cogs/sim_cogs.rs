//! The simulated driver's cog body.
//!
//! One cycle of a motor driver: take in what arrived, decide what to write,
//! write it to the modelled servos, and report. The deciding half is not here
//! -- it is [`reachy_driver::GoalGate`], hosted rather than re-implemented, so
//! the simulated driver and the real one cannot disagree about which goal is
//! due or when silence has gone on long enough to de-torque the machine.
//!
//! What is here is the plant, and it is deliberately shallow: each energised,
//! unjammed servo closes at most a fixed fraction of the gap to its target per
//! cycle. That is enough to make an obstruction look like a growing tracking
//! error and a step-too-far look like a servo that could not keep up, which is
//! the evidence the motion tick classifies faults from. It is not a dynamics
//! model and no scenario may ask it a question about servo fidelity.
//!
//! Between the plant and anything that addresses a servo sits the register
//! file, [`sim_regs`]: the modelled control tables. A transaction reaches a
//! register and never the plant directly, which is what makes the simulated
//! driver's observable contract the real one's, and the cycle's own pose sample
//! is read from those cells too -- the sample is a read of the present-position
//! registers, and a second route from the plant to what the machine reports
//! about itself is a second thing to keep in step.
//!
//! The first execution is the process starting, and its first act is releasing
//! the machine: a driver that has just come up cannot know what a predecessor
//! left energised, so every row goes limp before anything else happens. That is
//! the real driver's process-start sweep, and it is here for the reason the
//! gate is hosted rather than re-implemented -- two drivers that disagree about
//! start of life are two contracts.
//!
//! What this driver says about its own run goes out on a cadence, whole: the
//! newest status record is the account of the run so far, which is what makes a
//! log verifiable however late its logger attached.
//!
//! Nothing here holds state of its own. The plant, the register file, the gate
//! and the run's totals are the state slot's own fields, read and written
//! through the validated view the cycle opens once at the top.

use brenn_reachy__cogs__config_clk_rs::{SimParams, SimParamsWire};
use brenn_reachy__cogs__session_cmd_clk_rs::SessionCmdKind;
use brenn_reachy__cogs__sim_clk_rs::{MotorSimDial, MotorSimOutputs, MotorSimSignals};
use brenn_reachy__cogs__sim_state_clk_rs::{SimCmd, SimOp, SimState, SimStateWire};
use brenn_reachy__driver__health_clk_rs::{DriverStatus, DriverStatusWire, EventKind};
use brenn_reachy__driver__pose_clk_rs::PoseSample;
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use clockwork_rs::SyncTime;
use motion_slots::{configured, counters};
use reachy_driver::report::{self, Event};
use reachy_driver::{
    AcceptOutcome, AuxOffer, AuxSlot, AuxTask, ConfirmReport, EngageRequest, EngageStep,
    GateAction, GoalGate, TORQUE_OFF_CONFIRM_BUDGET_NS, TorqueOffConfirm, credit_engagement,
    engage_verdict, fail_engagement,
};
use reachy_kin::default_geometry;
use reachy_motion::disarm::stow_targets;
use reachy_motion::joints::{
    JointGroup, JointRef, ROW_COUNT, angle_of, flags, group_of, row, rows_of, set_angle,
    write_rows, write_vector,
};

pub mod sim_aux;
pub mod sim_lag;
pub mod sim_regs;

use sim_aux::{Answer, Request};
use sim_regs::Regs;

/// How many cycles of plant motion one execution may advance.
///
/// Under the deterministic runner an execution is always one cycle behind the
/// last, and this never binds. Online it does: a process that lost the CPU for
/// half a second must not make up the distance in one step, because a plant
/// that teleports hides exactly the tracking error the control loop is being
/// tested on. Falling behind shows up as a machine that did not get where it
/// was asked, which is the truthful reading of a driver that missed its cycles.
const MAX_CATCHUP_CYCLES: i64 = 8;

/// The cycle this cog's execution condition waits for, nanoseconds.
///
/// Stated here because the timer is a literal in the declaration and the cycle
/// the plant advances by is configuration: the two are the same number, and the
/// first execution refuses a scenario that made them different rather than
/// modelling a bus running at a rate nobody asked for.
const TIMER_PERIOD_NS: i64 = 20_000_000;

/// How long between status records, nanoseconds.
///
/// The real driver publishes one per window of cycle statistics, which is a
/// second, and this is that second.
const STATUS_PERIOD_NS: i64 = 1_000_000_000;

/// The same, in cycles, which is how it is counted.
///
/// A constant and not a division of the configured period: `check_params`
/// refuses a scenario whose grid is not [`TIMER_PERIOD_NS`], so there is no
/// other grid for this to be worked out against. Computing it per execution
/// would say the opposite -- that this cog tolerates a period a scenario chose
/// -- which is the belief that refusal exists to refuse.
const STATUS_CYCLES: u32 = (STATUS_PERIOD_NS / TIMER_PERIOD_NS) as u32;

/// The simulated grid is the one the driver layer's budgets are sized against.
///
/// The constants both hosts share are counts of cycles — a confirm budget, a
/// blind-cycle limit — so they only mean the same duration in both while the
/// two grids are the same number. The real driver refuses a configured period
/// that is not the nominal one; this is the same refusal for the sim, made at
/// compile time.
const _: () = assert!(TIMER_PERIOD_NS == reachy_driver::NOMINAL_CYCLE_NS);

/// One cycle of the simulated driver.
pub fn execute_motor_sim(dial: &mut MotorSimDial<'_>) {
    let nominal = dial.start_time().as_nanos();
    let params = read_params(dial.configs.params);
    let hold_timeout_ns = params.hold_timeout_ns;
    let period_ns = params.period_ns;
    let since_last_ns = dial.conditions.tick.time_since_last_exec().as_nanos();
    // Read off the slot's own bytes before anything narrows them: the totals
    // are plain numbers whatever else the slot holds, and the clear below would
    // lose them.
    let before = Counters::read(dial.states.sim);
    let carried = StatusTotals::read(dial.states.sim);
    // The two instants the record names go back too, and for the same reason a
    // total does: each says when something first happened in this run, and a
    // run whose first sample went out a minute ago did not have its first
    // sample at the restart. The sweep instant is the exception and is not
    // carried -- the restart runs the release again, so the record of when this
    // state's release ran is the restart's own.
    let first_pose = dial.states.sim.first_pose();
    let first_session_cmd = dial.states.sim.first_session_cmd();
    let mut refused_state = 0;
    if dial.states.sim.validate_mut().is_err() {
        // Bytes that did not read as a state. This cog is the slot's only
        // writer, so a refusal is memory nobody wrote: the run starts again
        // from a cleared slot with the totals it had carried put back, and the
        // refusal is counted rather than raised -- the process whose job is to
        // de-torque a machine does not get to panic over its own memory.
        dial.states.sim.clear_valid();
        before.store(dial.states.sim);
        carried.store(dial.states.sim);
        dial.states.sim.set_first_pose(first_pose);
        dial.states.sim.set_first_session_cmd(first_session_cmd);
        refused_state = 1;
    }
    let Ok(state) = dial.states.sim.validate_mut() else {
        // A cleared slot that does not read as a state either: this build and
        // the schema disagree about what a cleared state is, and there is
        // nothing a cycle can do with the slot at all.
        return;
    };
    state.refused_state_fields += refused_state;
    let mut report = Report::default();

    let first = !state.initialized.get();
    if first {
        check_params(params);
        state.initialized = true.into();
        write_vector(
            &mut state.positions,
            &stow_targets(default_geometry()).expect("the baked geometry reaches stow"),
        );
        // The provisioning, before anything reads a register: a machine this
        // process has just met is a correctly provisioned one, and a scenario
        // that wants otherwise says so with an injection.
        Regs::over(&mut state.regs).init();
        // What the process met, where a scenario says it met an energised
        // machine. A restart from a slot the cycle could not read is not a
        // process start: whatever arming the run had is in the bytes that were
        // refused, so nothing is re-energised out of a memory fault -- the one
        // transition the latch exists to prevent.
        if bool::from(params.start_torqued) && refused_state == 0 {
            state.torqued = flags::all();
            believe(state, flags::all(), true);
        }
        release(state, nominal);
    }

    for cmd in dial.inputs.cmds.new_msgs() {
        // An injection this build cannot read is refused whole and counted: an
        // operation it does not know, or one naming servos it does not know.
        // A scenario written against a newer vocabulary than the binary running
        // it would otherwise have its hand on the machine discarded in silence.
        // The boundary refusal is this one call.
        let Ok(cmd) = cmd.validate() else {
            state.refused_injections += 1;
            continue;
        };
        inject(state, cmd, nominal);
    }

    // What the host asked of the driver. Every accepted datagram is evidence
    // the host is alive, whichever kind it is: the dead-man measures silence,
    // and a host with nothing to ask still owes the driver a word.
    for cmd in dial.inputs.session_cmds.new_msgs() {
        // Counted before it is read, and counted whatever it turns out to say:
        // this is what arrived on the channel, which is the number a reader of
        // that channel's census can check against.
        state.session_cmds_taken += 1;
        let Ok(cmd) = cmd.validate() else {
            // A datagram this build cannot read. Counted as the boundary failure
            // it is -- a schema-version mismatch, not an ask this driver would
            // not run -- and deliberately not counted as liveness: what the
            // sender meant is unknown, and feeding the dead-man off bytes nobody
            // could read is holding a machine energised on the strength of noise.
            state.undecodable_inbound += 1;
            continue;
        };
        // Stamped on every datagram that decodes, including one this cycle turns
        // away, and on no datagram that does not. The real driver's seam refuses
        // an unreadable datagram before its loop ever sees one, so a stamp taken
        // ahead of the reading here would put this driver's first-contact
        // instant somewhere the other driver's could never be -- and the
        // ordering the report reads off it would be proven against the wrong
        // machine.
        if state.first_session_cmd.as_nanos() == 0 {
            state.first_session_cmd = SyncTime::from_nanos(nominal);
        }
        if cmd.kind == SessionCmdKind::None {
            // A datagram asking nothing is a slot nothing wrote, published. Same
            // treatment for the same reason.
            state.aux_refused += 1;
            continue;
        }
        if cmd.kind == SessionCmdKind::EngageNow && cmd.rows == JointFlags::NONE {
            // An engagement naming no row is the same thing said another way:
            // there is no row to take hold of, so nothing would be written and
            // nothing answered. Refused and counted rather than fed to the
            // dead-man, so a host that sent one learns it from the census
            // instead of from its own silence deadline.
            state.aux_refused += 1;
            continue;
        }
        GoalGate::over(&mut state.gate).note_liveness(nominal);
        match cmd.kind {
            // Liveness and nothing else. The asking-nothing arm is unreachable:
            // it was answered and counted above.
            SessionCmdKind::None | SessionCmdKind::KeepAlive => {}
            SessionCmdKind::TorqueOffNow => {
                // Host-requested and idempotent: the goal queue goes with the
                // latch, the transaction the host had outstanding is abandoned,
                // the plant goes limp, and the confirmation pass opens if one is
                // not already running -- its budget runs from the first of these
                // rather than from the latest, because the de-torquing was
                // commanded once. Nothing gates any of it.
                //
                // The abandonment is the same one the host performs on its own
                // side: a release outranks whatever was asked for before it, and
                // a torque-enable write run out of the slot after the latch
                // would re-energise the row this datagram exists to release.
                // An engagement not yet written goes the same way and
                // unanswered, for the same reason.
                GoalGate::over(&mut state.gate).latch_torque_off();
                AuxSlot::over(&mut state.aux).abandon();
                EngageRequest::over(&mut state.engage).abandon();
                state.torqued = JointFlags::NONE;
                state.has_target = JointFlags::NONE;
                TorqueOffConfirm::over(&mut state.confirm).begin(nominal);
            }
            SessionCmdKind::EngageNow => {
                // Nothing is written here. What an engagement writes is written
                // below, at the reading this cycle took: the pin is where the
                // machine actually is, and a pin from an older reading would
                // command a limp machine back to where it used to be.
                EngageRequest::over(&mut state.engage).offer(cmd.rows);
            }
            SessionCmdKind::Aux => {
                match AuxSlot::over(&mut state.aux).offer(cmd.corr, &cmd.txn) {
                    AuxOffer::Accepted => {}
                    // A verbatim re-issue of the request the slot holds: the
                    // transport repeating itself, which the pending request's
                    // own outcome answers. Nothing goes back for it, and it is
                    // not a refusal.
                    AuxOffer::Duplicate => state.aux_duplicates += 1,
                    AuxOffer::RefusedBusy => {
                        // The host that fills this slot is serial by
                        // construction, so a request under another number, or
                        // under this one carrying other bytes, is a host that
                        // is not what it claims to be. Loud both ways: an
                        // outcome against the turned-away request's own number,
                        // and a count.
                        state.aux_refused += 1;
                        report.answer(Answer::busy(cmd.corr, &Request::of(&cmd.txn)));
                    }
                }
            }
        }
    }

    for setpoint in dial.inputs.goals.new_msgs() {
        // Counted before it is read, for the reason a session datagram is.
        state.goals_taken += 1;
        // A setpoint naming rows this build does not know is not a setpoint:
        // the bits mean something to whoever wrote them and this driver does
        // not know which servos they are, so there is nothing to queue and
        // nothing to report about it beyond the count. The boundary refusal is
        // this one call.
        let Ok(setpoint) = setpoint.validate() else {
            state.refused_goals += 1;
            continue;
        };
        let due_at = setpoint.execute_at.as_nanos();
        let depth = state.gate.queue.len();
        match GoalGate::over(&mut state.gate).accept(setpoint, nominal) {
            AcceptOutcome::Accepted => {}
            AcceptOutcome::AcceptedStaleOrOutOfOrder => {
                report.raise(
                    &mut state.events_dropped,
                    Event {
                        kind: EventKind::GoalStaleOrOutOfOrder,
                        // How far past its instant it arrived. Zero for a goal
                        // that is merely out of order with the one before it,
                        // which has not missed anything yet.
                        silence_ns: (nominal - due_at).max(0),
                        ..Event::at(nominal)
                    },
                );
            }
            AcceptOutcome::DroppedQueueFull => {
                state.goals_dropped += 1;
                report.raise(
                    &mut state.events_dropped,
                    Event {
                        kind: EventKind::GoalDroppedQueueFull,
                        count: u32::try_from(depth).unwrap_or(u32::MAX),
                        ..Event::at(nominal)
                    },
                );
            }
        }
    }

    let silence = nominal - state.gate.last_accept.as_nanos();
    // What the driver believes, not what the plant is: the real driver has no
    // window onto the plant, and a dead-man measured against one would be a
    // dead-man the real machine does not have.
    let torqued = AuxSlot::over(&mut state.aux).belief().any();
    match GoalGate::over(&mut state.gate).tick(nominal, torqued, hold_timeout_ns) {
        GateAction::WriteTorqueOffSweep { just_latched } => {
            // The sweep reaches every row, and a de-torqued servo holds where
            // it stands: this machine's gearboxes do not back-drive. The
            // modelled servos forget what they were asked for along with the
            // torque, so a machine energised again later stands still until
            // something commands it rather than resuming a move nobody is
            // asking for any more.
            state.torqued = JointFlags::NONE;
            state.has_target = JointFlags::NONE;
            // The sweep is being written, so what it took is a question worth
            // reading back. Idempotent: the pass keeps the instant it opened
            // at, because the budget is measured from when the de-torquing was
            // commanded.
            TorqueOffConfirm::over(&mut state.confirm).begin(nominal);
            if just_latched {
                state.hold_timeouts += 1;
                report.raise(
                    &mut state.events_dropped,
                    Event {
                        kind: EventKind::HoldTimeoutTorqueOff,
                        // How long the goal stream was silent, which is what
                        // tells an operator whether the commander stopped or
                        // merely stuttered.
                        silence_ns: silence.max(0),
                        ..Event::at(nominal)
                    },
                );
            }
        }
        GateAction::WriteGoal => {
            state.goals_executed += 1;
            command(state);
        }
        GateAction::Rewrite => {
            // The same setpoint again, which is what holds a servo's position
            // loop awake between goals. Nothing here counts it: a rewrite is
            // not a goal executed.
            command(state);
        }
        GateAction::Nothing => {}
    }

    let cycles = if first {
        1
    } else {
        elapsed_cycles(since_last_ns, period_ns)
    };
    state.cycles += 1;
    state.cycles_since_status = state.cycles_since_status.saturating_add(1);
    // Grid points this execution stepped over. An execution that covers more
    // than one cycle of plant motion is a process that lost the CPU, and the
    // points behind it are points no cycle attended -- which is the same
    // reading the real driver's loop makes of a slot it missed.
    state.skipped_cycles += u64::try_from(cycles - 1).unwrap_or(0);
    keep_history(state, cycles);
    advance(params, state, cycles);

    // A cycle in which the bus answers nothing at all. Decided before anything
    // is read, because it decides whether anything is: the outage is the wire's
    // and not one read's, so the proprioception, the host's transaction, the
    // confirmation's read-back and the health rotation are all silent for it.
    let blind = state.drop_replies_left > 0;
    state.drop_replies_left = state.drop_replies_left.saturating_sub(1);
    let engaged = run_engage(state, nominal, blind, &mut report);
    if !blind {
        read_registers(state);
    }
    run_aux(state, params, nominal, blind, engaged, &mut report);

    // The sample: published every cycle without exception, because it is the
    // clock the control loop runs on and a missing one is a cycle the loop
    // never sees. A cycle whose replies were lost is a sample saying so, never
    // an absent sample.
    count_blind(state, blind, nominal, &mut report);

    publish(&mut dial.outputs, state, blind, nominal, &report);

    Counters::read(dial.states.sim).report(&before, &mut dial.signals);
}

/// What a cycle has to say, in the three slots that carry it.
///
/// An output carries one message per execution, so a cycle that raises two
/// events publishes one. The dead-man's latch outranks the rest: it is the
/// machine changing state, where the others are remarks about a datagram that
/// the sender can see the consequences of anyway.
///
/// The other two are the aux path's. At most one transaction runs per cycle, so
/// the outcome slot collides only when a request was turned away in the same
/// cycle one was served; the first answer wins and the displaced one comes back
/// to the host as a silence. The real driver carries a second slot and publishes
/// both, so a scenario that overlaps two requests in one cycle sees a sim that
/// diverges from its subject. A health report cannot collide at all: the
/// rotation names at most one row.
// TODO(sim-aux-turned-away)
#[derive(Default)]
struct Report {
    /// What will be published, if anything.
    event: Option<Event>,
    /// The answer to one aux transaction.
    outcome: Option<Answer>,
    /// The row the health rotation read.
    health_row: Option<u8>,
}

impl Report {
    /// Offer an answer for this cycle's one outcome slot.
    fn answer(&mut self, answer: Answer) {
        if self.outcome.is_none() {
            self.outcome = Some(answer);
        }
    }

    /// Offer an event for this cycle's one slot, counting the one it displaces.
    ///
    /// The ranking is the driver layer's, so this host and the real driver
    /// publish the same one out of a cycle that hit two.
    fn raise(&mut self, dropped: &mut u64, event: Event) {
        report::raise(&mut self.event, dropped, event);
    }
}

/// Apply one of the scenario's injections to the modelled machine.
///
/// These stand in for the physical world and for the arming sequencer. An
/// operation this build does not know is refused and counted: the scenario is
/// describing something to a simulator that cannot do it, which is a fact
/// about the scenario and not about the machine.
///
/// Reading one is all-or-nothing, and that happens at the caller's one
/// validation: an operation this build does not know and a set of servos it
/// cannot read are the same fact about the scenario, and carrying out the
/// readable half of one would be worse than carrying out none of it -- a
/// torque-on whose mask was refused would energise nothing and still end a
/// torque-off latch, which is the one transition the latch exists to guard.
///
/// Two of them stand in for the bus rather than for the machine: a run of
/// transactions swallowed before they reach it, and a set of servos that answer
/// nothing at all. Neither touches the modelled machine -- what they model is a
/// host talking to something that is not listening.
///
/// A register write is the one injection that can be carried out for part of its
/// mask: the refusals a control table has are per servo -- a non-volatile
/// register takes a write from a de-torqued servo and not from an energised one
/// -- so the refusal is per row. It is refused outright for the cells the plant
/// owns, which this cycle's proprioception writes whatever anything else put
/// there.
fn inject(state: &mut SimState, cmd: &SimCmd, nominal: i64) {
    let mask = cmd.mask;
    match cmd.op {
        SimOp::TorqueOn => {
            state.torqued |= mask;
            believe(state, mask, true);
            // Arming grants a fresh hold-timeout window, whether or not there
            // was a latch to end: the first goal of a session cannot arrive
            // before the session starts. A fresh arming also ends whatever
            // confirmation pass was reading the old de-torquing back: the
            // machine is being energised deliberately, and read-backs saying so
            // are not evidence of anything failing.
            let mut gate = GoalGate::over(&mut state.gate);
            if gate.state().latched.get() {
                gate.release_latch(nominal);
                TorqueOffConfirm::over(&mut state.confirm).stand_down();
            } else {
                gate.note_liveness(nominal);
            }
        }
        SimOp::TorqueOff => {
            state.torqued = flags::without(state.torqued, mask);
            believe(state, mask, false);
            if flags::is_empty(state.torqued) {
                // A sweep that reached everything is a confirmed disarm, not a
                // fault: nothing latches, and nothing is being held any more.
                GoalGate::over(&mut state.gate).clear_commanded();
                state.has_target = JointFlags::NONE;
            }
        }
        SimOp::SetPositions => {
            for joint in flags::iter(mask) {
                if let Some(angle) = angle_of(&cmd.positions, joint) {
                    set_angle(&mut state.positions, joint, angle);
                }
            }
        }
        SimOp::Obstruct => state.obstructed |= mask,
        SimOp::ReleaseObstruction => state.obstructed = flags::without(state.obstructed, mask),
        SimOp::DropReplies => state.drop_replies_left = cmd.count,
        SimOp::RefuseAux => state.aux_unanswered_left = cmd.count,
        // The set replaces whatever was off the bus, so a scenario puts servos
        // back by naming fewer of them and puts the whole bus back by naming
        // none: an outage a scenario cannot end is one it cannot show a machine
        // surviving.
        SimOp::AbsentServo => state.absent = mask,
        // A response delay the rings can hold, or an injection refused whole:
        // a run whose premise is a servo nineteen cycles behind says nothing
        // about anything if the plant quietly gave it eight.
        SimOp::SetLag => {
            if sim_lag::representable(cmd.count) {
                // Each row *entering* the set has its whole ring filled with the
                // angle it is holding now, so a delay reaching back further than
                // the lag has existed chases that rather than the angle an
                // unwritten cell holds -- which on this machine is a real angle,
                // and a plant that slewed to it would be moving on nobody's
                // command. The target for a row that has one, and where the row
                // stands for a row that does not: a row nothing has commanded is
                // holding its position, and that is the only angle it can be
                // said to have been given.
                //
                // A row already lagged keeps its ring. Refilling it would hand a
                // row mid-move the target it holds now at every depth, so the
                // delayed target it is chasing would jump a whole lag's distance
                // forward for one cycle -- a step no scenario asked for, in the
                // one place a scenario is watching how far a row stands behind
                // its goal. The ring is what survives, not the depth: the depth
                // below is one number for the whole set, so naming a moving row
                // at a different one reads its kept history at the new offset
                // and steps that row's target by the difference on the next
                // cycle.
                let held = holding(state);
                for joint in flags::iter(flags::without(mask, state.lagged)) {
                    if let Some(index) = row(joint) {
                        sim_lag::seed(&mut state.target_history, index, held[index]);
                    }
                }
                state.lagged = mask;
                state.lag_cycles = cmd.count;
            } else {
                state.refused_injections += 1;
            }
        }
        SimOp::SetRegister => {
            for joint in flags::iter(mask) {
                let Some(index) = row(joint) else {
                    continue;
                };
                // A cell the plant owns is written from the plant every cycle,
                // so a value injected into one would be overwritten before
                // anything read it. Refused and counted rather than accepted:
                // a scenario whose hand is on where a servo is, or on whether
                // it is energised, reaches for the plant -- `set_positions`,
                // `torque_on`, `torque_off`, `obstruct` -- and one that reached
                // here instead has its premise answered rather than lost.
                if sim_regs::is_plant_owned(cmd.reg) {
                    state.refused_injections += 1;
                    continue;
                }
                let torqued = flags::contains(state.torqued, joint);
                if Regs::over(&mut state.regs)
                    .write_bits(index, cmd.reg, cmd.value, torqued)
                    .is_err()
                {
                    state.refused_injections += 1;
                }
            }
        }
        // The value a slot nothing wrote holds. A scenario never authors one,
        // and nothing about the machine changes for it.
        SimOp::Nop => {}
    }
}

/// Count how long the bus has been answering nothing, and say so once the run
/// is long enough to mean the bus is gone.
///
/// The counting and the once-only rule are the driver layer's, over the run this
/// state slot carries: the fault a driver raises about itself has to mean the
/// same thing from either host.
fn count_blind(state: &mut SimState, blind: bool, nominal: i64, report: &mut Report) {
    if blind {
        state.blind_cycles_total += 1;
    }
    if let Some(event) = report::count_blind(&mut state.blind_cycles, blind, nominal) {
        report.raise(&mut state.events_dropped, event);
    }
}

/// Write the held setpoint's rows into the plant, and remember which rows have
/// one.
///
/// The mask is the setpoint's own and it is write-side filtering and nothing
/// else: a setpoint applies to the servos it names and leaves every other one
/// holding the angle it already had. That is the whole meaning of a partial
/// mask, stated here because this is the only place in the tree that turns a
/// commanded setpoint into servo targets.
fn command(state: &mut SimState) {
    for joint in flags::iter(state.gate.held.mask) {
        if let Some(angle) = angle_of(&state.gate.held.targets, joint) {
            set_angle(&mut state.targets, joint, angle);
        }
    }
    state.has_target |= state.gate.held.mask;
}

/// Take the cycle's proprioception: bring the live register cells up to date
/// with the plant.
///
/// The only route from the modelled world into anything that addresses a servo.
/// Run
/// after the plant has moved and before any register is read, so a verification
/// of a stow or of a de-torquing reads where the machine actually is rather than
/// where it was last asked to be.
///
/// Not run at all on a cycle the bus answers nothing on: a driver reads its
/// cells over the same wire everything else goes over, so what it holds about
/// the machine through an outage is what it held when the outage began.
///
/// A reading with no eight-byte carriage -- a non-finite angle -- is counted
/// rather than stored: the cell keeps the last number it held, so what the
/// sample reports is a stale angle and the count is the only place that says
/// so. No scenario asserts that count is zero, so the substitution is loud only
/// in this cog's own cases.
/// TODO(sim-refused-readings-asserted)
fn read_registers(state: &mut SimState) {
    let positions = rows_of(&state.positions);
    let targets = rows_of(&state.targets);
    let torqued = flags::rows(state.torqued);
    let refused = Regs::over(&mut state.regs).refresh(&positions, &targets, &torqued);
    state.refused_readings += u64::from(refused);
}

/// The rows this cycle's grouped position read answered: every row on the bus.
///
/// A servo the scenario has taken off the bus answers no read, which is what
/// the real driver's grouped read finds too -- and an engagement may only pin a
/// row this cycle read, so the two drivers decline the same engagements.
fn answering_rows(state: &SimState) -> JointFlags {
    let mut answered = JointFlags::NONE;
    for joint in flags::iter(flags::all()) {
        let on_bus = row(joint).is_some_and(|index| !sim_aux::is_absent(state, index));
        if on_bus {
            answered |= flags::bit(joint);
        }
    }
    answered
}

/// Take hold of the machine, where an engagement is asked for and this cycle's
/// bus answered the rows it names.
///
/// The plant's side of the real driver's engage cycle: the goal registers
/// pinned where the modelled servos actually stand, torque on over the same
/// rows, and the enable read back. Run before the proprioception refresh, so
/// the cells the rest of the cycle reads -- the sample's own, and the
/// confirmation pass's -- answer about the machine as this cycle left it.
///
/// A cycle whose bus answers nothing answers no row, so nothing is written and
/// the request waits: the pin is the reading this cycle took, and there is no
/// older one to take hold of a machine on. A servo off the bus answers nothing
/// either, so an engagement naming one waits out its window and is declined
/// with nothing written -- which is what the real driver does with it, the
/// grouped position read being the same read there.
///
/// The unconfirmed ending is a row that answers the position read and then does
/// not read its enable back. Nothing injects that here: the plant energises
/// what it pins, and a servo that answers reads while refusing its own enable
/// is not a machine this simulator models.
///
/// No out-of-band transaction runs behind an engagement, which is the real
/// driver's own budget: three grouped exchanges leave no room for a fourth.
/// Here that is not a budget but the same shape kept, so a scenario's evidence
/// about what a cycle did carries. Answers whether this cycle wrote one.
fn run_engage(state: &mut SimState, nominal: i64, blind: bool, report: &mut Report) -> bool {
    let answered = if blind {
        JointFlags::NONE
    } else {
        answering_rows(state)
    };
    let believed = AuxSlot::over(&mut state.aux).belief().rows();
    let step = EngageRequest::over(&mut state.engage).step(answered, believed);
    // The two endings that write nothing are answered by the decision itself,
    // so this driver and the real one say the same thing about the same step.
    if let Some((kind, rows)) = step.answer() {
        report.raise(
            &mut state.events_dropped,
            Event {
                kind,
                rows,
                ..Event::at(nominal)
            },
        );
        return false;
    }
    let EngageStep::Run { rows } = step else {
        return false;
    };

    let mut confirmed = JointFlags::NONE;
    let mut unconfirmed = JointFlags::NONE;
    for joint in flags::iter(rows) {
        let Some(index) = row(joint) else {
            continue;
        };
        // Unreachable by way of the answered mask above, and kept because a
        // row off the bus must never be pinned or believed whatever route
        // reached this loop.
        if sim_aux::is_absent(state, index) {
            unconfirmed |= flags::bit(joint);
            continue;
        }
        // The pin: the goal register at the angle the plant is at, which is
        // what makes a servo that comes up hold where it stands.
        if let Some(angle) = angle_of(&state.positions, joint) {
            set_angle(&mut state.targets, joint, angle);
        }
        confirmed |= flags::bit(joint);
    }
    state.torqued |= confirmed;
    state.has_target |= confirmed;
    believe(state, confirmed, true);
    // Arming grants a fresh hold-timeout window and ends a standing latch, and a
    // fresh arming ends whatever confirmation pass was reading the old
    // de-torquing back: the machine is being energised deliberately. The plant's
    // own belief is written above; what the driver crate credits is the belief
    // the dead-man runs over, and the latch and the pass with it.
    credit_engagement(
        &mut state.aux,
        &mut state.gate,
        &mut state.confirm,
        confirmed,
        nominal,
    );
    let (kind, named) = engage_verdict(rows, unconfirmed);
    // A row that did not confirm leaves this driver holding a torque it cannot
    // vouch for, so it lets go on its own authority -- after the credit, so the
    // latch stands over whatever the confirmed rows released. Unreachable here
    // for the reason the unconfirmed ending is, and written because whatever
    // route reaches it, the two drivers must do the same thing about it.
    if kind == EventKind::EngageUnconfirmed {
        fail_engagement(&mut state.aux, &mut state.gate, &mut state.confirm, nominal);
    }
    report.raise(
        &mut state.events_dropped,
        Event {
            kind,
            rows: named,
            ..Event::at(nominal)
        },
    );
    true
}

/// Spend this cycle's one out-of-band transaction.
///
/// Three things want it and [`reachy_driver::AuxSlot`] is what picks, in the
/// order it documents: a host request first, then the confirmation pass's
/// read-back, then the health rotation. Run after the proprioception, so every
/// cell a transaction reads answers about the machine as this cycle left it --
/// a verification of a stow or of a de-torquing that read a stale cell would
/// confirm the cycle before.
///
/// The confirmation is stepped before the slot is asked, because what it wants
/// read is an input to the choice. Its two reports are events: a whole clean
/// pass says the commanded de-torquing took, and a budget that ran out says it
/// cannot be confirmed -- after which the pass keeps reading and the sweep keeps
/// being written, because nothing gates de-torquing.
///
/// `blind` is a cycle whose bus answered nothing. Everything below is a round
/// trip on that wire, so nothing completes: the host's transaction is dropped
/// unanswered, the read-back observes nothing -- which counts as a row still
/// holding, because a de-torquing credited to silence is the one report this
/// driver must never make -- and the rotation publishes nothing about a machine
/// nobody heard from.
fn run_aux(
    state: &mut SimState,
    params: &SimParams,
    nominal: i64,
    blind: bool,
    engaged: bool,
    report: &mut Report,
) {
    let step =
        TorqueOffConfirm::over(&mut state.confirm).step(nominal, TORQUE_OFF_CONFIRM_BUDGET_NS);
    match step.report {
        ConfirmReport::Nothing => {}
        ConfirmReport::Confirmed => {
            // Every row read back zero. The belief goes to nothing here rather
            // than when the sweep was commanded: a de-torquing that has not been
            // read back is one the dead-man must keep running over.
            AuxSlot::over(&mut state.aux).belief().confirmed_off();
            GoalGate::over(&mut state.gate).clear_commanded();
            report.raise(
                &mut state.events_dropped,
                Event {
                    kind: EventKind::TorqueOffConfirmed,
                    ..Event::at(nominal)
                },
            );
        }
        ConfirmReport::Unconfirmed => {
            // Which servos have not gone limp, read off their own control
            // tables: what the machine says, rather than what the writes said.
            let rows = sim_aux::rows_still_torqued(state);
            report.raise(
                &mut state.events_dropped,
                Event {
                    kind: EventKind::TorqueOffUnconfirmed,
                    rows,
                    ..Event::at(nominal)
                },
            );
        }
    }

    let task = if engaged {
        // Nothing out of band behind an engagement, which is the real driver's
        // budget: three grouped exchanges leave no room for a fourth. The
        // confirmation pass keeps its place either way -- an engagement that
        // confirmed anything stood it down, and one that did not left it
        // reading.
        AuxTask::Nothing
    } else {
        AuxSlot::over(&mut state.aux).take(nominal, params.health_poll_period_ns, step.read_row)
    };
    match task {
        AuxTask::Nothing => {}
        // The whole bus is silent, so the transaction is dropped exactly as a
        // swallowed one is -- and the swallow countdown is left alone, because
        // this datagram never reached the wire it was going to be swallowed on.
        AuxTask::Host { .. } if blind => {}
        AuxTask::Host { .. } if state.aux_unanswered_left > 0 => {
            // The request was swallowed before it reached the bus. Nothing about
            // the machine changes and nothing goes back to the host, which is
            // the silence its own delivery timeout is for -- an outcome saying
            // "timeout" would be an answer, and a host that got one would know
            // more than a host whose datagram was lost. The slot is free
            // either way: `take` consumed the request, so the re-issue this
            // provokes is accepted like any other.
            state.aux_unanswered_left -= 1;
        }
        AuxTask::Host { corr } => {
            // The record stays in the slot and the transaction is run from a
            // copy of it, because running it writes the state it lives in.
            let request = AuxSlot::over(&mut state.aux).taken(corr).map(Request::of);
            if let Some(request) = request {
                let answer = sim_aux::answer(state, nominal, corr, &request);
                report.answer(answer);
            }
        }
        AuxTask::ConfirmTorqueOff { row } => {
            let torqued = blind || sim_aux::reads_torqued(state, usize::from(row));
            if blind {
                // A read-back nothing answered, counted as a row still holding:
                // a de-torquing credited to silence is the one report this
                // driver must never make.
                state.confirm_misses += 1;
            }
            TorqueOffConfirm::over(&mut state.confirm).observed(row, torqued);
        }
        // A servo off the bus reports nothing, so the rotation's read of it goes
        // out as no report at all rather than as a report of zeroes about a
        // machine nobody heard from. The cadence was stamped when the read was
        // named, so the rotation walks on to the next row either way.
        AuxTask::Health { row } if blind || sim_aux::is_absent(state, usize::from(row)) => {
            state.health_misses += 1;
        }
        AuxTask::Health { row } => {
            state.health_reports += 1;
            report.health_row = Some(row);
        }
    }
}

/// Release the machine this process met, and record that it did.
///
/// The simulated driver's answer to the same question the real one answers
/// first on the bus: a process that has just started cannot know what a
/// predecessor left energised, so it de-torques before it does anything else.
/// Every row goes limp, the belief goes with it, and the gate stands in its
/// never-commanded state -- no latch, because the release is verified as it is
/// written and there is nothing left to keep reaching for.
///
/// The real sweep is nine verified writes and can leave rows it could not read
/// back. This one cannot: it is the observable machine that is modelled here,
/// and the plant has no write that fails -- the scenario's injections are read
/// after this, so nothing has yet put a servo off the bus or cut the replies.
/// So the record it stamps has no failed rows, and where a failed row is a
/// question -- the latch, the confirmation pass, the report's reading of them --
/// the answer is proven over the real driver's own bus.
fn release(state: &mut SimState, nominal: i64) {
    state.torqued = JointFlags::NONE;
    state.has_target = JointFlags::NONE;
    believe(state, flags::all(), false);
    GoalGate::over(&mut state.gate).clear_commanded();
    state.swept_at = SyncTime::from_nanos(nominal);
}

/// Record what a verified torque-enable write would have said about these rows.
///
/// What the injections stand in for: a hand on the machine, or an arming
/// sequencer that ran before this process started. The belief moves with the
/// plant because the write is modelled as verified -- an unverified one says
/// nothing about the machine, and a dead-man measured against beliefs built out
/// of those is measuring the wrong thing in both directions.
fn believe(state: &mut SimState, rows: JointFlags, enabled: bool) {
    let mut slot = AuxSlot::over(&mut state.aux);
    let mut belief = slot.belief();
    for joint in flags::iter(rows) {
        if let Some(index) = row(joint) {
            belief.verified_write(u8::try_from(index).unwrap_or(u8::MAX), enabled);
        }
    }
}

/// This cog's configuration, as the numbers a scenario wrote down.
fn read_params(message: &SimParamsWire) -> &SimParams {
    configured(message, "the plant's")
}

/// Refuse a scenario whose configuration does not describe a machine.
///
/// Run once, on the first execution, before anything is torqued. Every one of
/// these turns a scenario-authoring slip into a plausible-looking run, and a
/// simulator whose plant quietly ran at the wrong rate certifies nothing: the
/// assertions still pass, about a machine nobody meant to build.
fn check_params(params: &SimParams) {
    assert_eq!(
        params.period_ns, TIMER_PERIOD_NS,
        "the plant's cycle must be the cycle this cog is woken on",
    );
    assert!(
        params.hold_timeout_ns >= params.period_ns,
        "the dead-man must allow at least one cycle of silence, not {}ns",
        params.hold_timeout_ns,
    );
    for (rate, group) in [
        (params.slew_body_yaw_rad, "body yaw"),
        (params.slew_legs_rad, "legs"),
        (params.slew_antennas_rad, "antennas"),
    ] {
        assert!(
            rate.is_finite() && rate > 0.0,
            "the {group} slew must be a distance a servo covers in a cycle, not {rate}",
        );
    }
}

/// How many cycles of motion an execution covers.
///
/// One under the deterministic runner, where the runner advances simulated time
/// to the timer exactly. Online, however many whole cycles have passed, capped:
/// see [`MAX_CATCHUP_CYCLES`]. The period is positive because
/// [`check_params`] refused the scenario otherwise.
fn elapsed_cycles(elapsed_ns: i64, period_ns: i64) -> i64 {
    (elapsed_ns / period_ns).clamp(1, MAX_CATCHUP_CYCLES)
}

/// Write this execution's targets into the rings, one cell per cycle of plant
/// motion.
///
/// Every cycle, whether or not a goal arrived: a row asked for nothing new is a
/// row still being asked for the same thing, and a ring that only moved when a
/// setpoint did would measure a delay in goals rather than in cycles. An
/// execution that covered several cycles writes the same target into each of
/// them, which is a plant that was asked for nothing while the process was away.
///
/// Only while some row is lagged: a ring nothing reads back through is 288 cells
/// written per cycle for nobody. The lag's own injection fills the rings it is
/// about, so the history a delay reads begins where the delay does.
fn keep_history(state: &mut SimState, cycles: i64) {
    if flags::is_empty(state.lagged) {
        return;
    }
    let targets = holding(state);
    let mut cursor = state.history_cursor;
    for _ in 0..cycles.min(sim_lag::LAG_DEPTH as i64) {
        cursor = sim_lag::push(&mut state.target_history, cursor, &targets);
    }
    state.history_cursor = cursor;
}

/// What each row is holding, radians: the target it has been given, or -- for a
/// row nothing has commanded yet -- where it stands.
///
/// The distinction matters only to the rings. A row with no target carries the
/// schema's zero in `targets`, which on this machine is a real angle, so a ring
/// filled from it would hand a lagged row a goal nobody ever gave: a row nothing
/// has asked anything of is holding its position, and that is the only angle it
/// can be said to be chasing.
fn holding(state: &SimState) -> [f64; ROW_COUNT] {
    let mut held = rows_of(&state.targets);
    let standing = rows_of(&state.positions);
    for joint in flags::iter(flags::without(flags::all(), state.has_target)) {
        if let Some(index) = row(joint) {
            held[index] = standing[index];
        }
    }
    held
}

/// What `joint` is closing on this cycle, radians.
///
/// The target it holds now, or -- for a row the scenario gave a response delay
/// -- the one it was given that many cycles ago. A delay the rings cannot reach
/// back to is a delay nothing set: `set_lag` refused it, so there is no case in
/// which this silently shortens one.
fn chasing(state: &SimState, joint: JointRef) -> Option<f64> {
    if flags::contains(state.lagged, joint)
        && let Some(row) = row(joint)
        && let Some(angle) = sim_lag::delayed(
            &state.target_history,
            state.history_cursor,
            row,
            state.lag_cycles,
        )
    {
        return Some(angle);
    }
    angle_of(&state.targets, joint)
}

/// Move the modelled servos toward what they are being asked for.
///
/// A jammed servo holds where it stands whatever it is asked for -- that is
/// what an obstruction is to a position loop, and the growing error is what the
/// motion tick's obstruction detector reads. A de-torqued one holds too: these
/// gearboxes do not back-drive, which is why a de-torqued machine at stow is
/// the safe state and a de-torqued machine anywhere else is not.
fn advance(params: &SimParams, state: &mut SimState, cycles: i64) {
    for joint in flags::iter(state.torqued) {
        if flags::contains(state.obstructed, joint) || !flags::contains(state.has_target, joint) {
            continue;
        }
        let Some(group) = group_of(joint) else {
            continue;
        };
        let (Some(target), Some(position)) =
            (chasing(state, joint), angle_of(&state.positions, joint))
        else {
            continue;
        };
        let step = slew(params, group) * cycles as f64;
        let gap = target - position;
        let moved = if gap.abs() <= step {
            target
        } else {
            position + step.copysign(gap)
        };
        set_angle(&mut state.positions, joint, moved);
    }
}

/// How far a servo of `group` moves in one cycle, radians.
///
/// Per group rather than per servo: the six cranks carry the head between them
/// and are the same part, the antennas are a different and much faster one, and
/// the body yaw is its own. Each rate is a distance rather than a number that
/// might be one, because [`check_params`] refused the scenario otherwise.
fn slew(params: &SimParams, group: JointGroup) -> f64 {
    match group {
        JointGroup::BodyYaw => params.slew_body_yaw_rad,
        JointGroup::Legs => params.slew_legs_rad,
        JointGroup::Antennas => params.slew_antennas_rad,
    }
}

/// This cycle's sample, written into the slot that carries it.
///
/// Published every cycle without exception, because it is the clock the control
/// loop runs on and a missing one is a cycle the loop never sees. A cycle whose
/// replies were lost is a sample saying so, never an absent sample: `blind`
/// puts every row in `missing` and clears `present_valid`.
fn write_sample(sample: &mut PoseSample, state: &SimState, blind: bool, nominal: i64) {
    let nominal = SyncTime::from_nanos(nominal);
    sample.nominal_time = nominal;
    // The modelled bus answers the instant it is asked, so there is no read
    // jitter to report and the two instants are one.
    sample.sample_time = nominal;
    sample.present_valid = (!blind).into();
    sample.commanded_valid = state.gate.has_held;
    sample.torque_off_latched = state.gate.latched;
    sample.missing = if blind {
        flags::all()
    } else {
        JointFlags::NONE
    };
    // The present-position registers, out of the control tables. Reported
    // whatever the flags say about them: what makes a missing row's values
    // unusable is that set, not a blank the receiver would have to tell apart
    // from a real zero. Every cell is finite, because a reading with no
    // carriage was counted instead of stored -- so a sample never carries a
    // non-finite angle, and a plant that went non-finite shows up as a stale
    // one with `refused_readings` above zero.
    write_rows(&mut sample.present, &sim_regs::present_rows(&state.regs));
    write_rows(&mut sample.commanded, &rows_of(&state.gate.held.targets));
}

/// Put this cycle's sample, and its event if it has one, on the wire.
///
/// Both channels carry their schema, so both are written into the output slot
/// field by field: nothing is encoded and neither carries a sequence number of
/// its own.
fn publish(
    outputs: &mut MotorSimOutputs<'_>,
    state: &mut SimState,
    blind: bool,
    nominal: i64,
    report: &Report,
) {
    let out = &mut outputs.pose;
    write_sample(out.msg_mut().clear_valid(), state, blind, nominal);
    out.mark_for_publish();
    state.published += 1;
    if state.first_pose.as_nanos() == 0 {
        state.first_pose = SyncTime::from_nanos(nominal);
    }

    if let Some(event) = report.event.as_ref() {
        let out = &mut outputs.evt;
        event.write(out.msg_mut().clear_valid());
        out.mark_for_publish();
        state.published += 1;
    }

    if let Some(answer) = report.outcome.as_ref() {
        let out = &mut outputs.aux_out;
        answer.write(out.msg_mut().clear_valid());
        out.mark_for_publish();
        state.published += 1;
    }

    if let Some(row) = report.health_row {
        let out = &mut outputs.health_out;
        sim_aux::health(
            state,
            nominal,
            usize::from(row),
            out.msg_mut().clear_valid(),
        );
        out.mark_for_publish();
        state.published += 1;
    }

    if status_due(state) {
        let out = &mut outputs.status;
        write_status(out.msg_mut().clear_valid(), state, nominal);
        out.mark_for_publish();
        state.published += 1;
        state.cycles_since_status = 0;
    }
}

/// Whether this cycle is one the status record goes out on.
///
/// The first cycle, so the release this process wrote is on the wire from the
/// first instant, and then one cycle in every simulated second. Counted in
/// cycles rather than measured against the clock: the grid is pinned, so the
/// count and the duration are the same statement, and it is the real driver's
/// own cadence -- the window its cycle statistics ride.
fn status_due(state: &SimState) -> bool {
    if state.cycles <= 1 {
        return true;
    }
    state.cycles_since_status >= STATUS_CYCLES
}

/// The record's own size, which is what says it has not grown.
///
/// The mapping below is hand-written, and so is the real driver's
/// `publish_status`: two compositions of one record out of different state, with
/// nothing in either language joining them. A field added to the schema is
/// therefore a field one side can write and the other silently leave zero -- and
/// the analyzer that verifies a run reads whichever copy the log carries. The
/// size is the cheapest thing that changes when the record does; it fails the
/// build here, where the second mapping is, rather than passing a scenario log
/// that certifies less than it claims.
const _: () = assert!(size_of::<DriverStatusWire>() == 280);

/// The whole run so far, in the record the real driver publishes.
///
/// Cumulative and complete, which is the point of it: a reader that has seen
/// one copy has read the run, so it does not matter which copy it saw. Every
/// number here is this driver's own honest count. The fields naming a failure
/// an in-process channel cannot have -- a datagram of the wrong length, a queue
/// that overflowed, a reader thread that stopped, a send the operating system
/// refused -- read zero because none of them happened, which is what the schema
/// says a zero there means.
fn write_status(status: &mut DriverStatus, state: &SimState, nominal: i64) {
    status.time = SyncTime::from_nanos(nominal);
    status.sweep_time = state.swept_at;
    // The release wrote every row and cannot have left one behind: see
    // [`release`].
    status.sweep_failed_rows = JointFlags::NONE;
    status.torque_latched = state.gate.latched;
    status.first_pose = state.first_pose;
    status.first_session_cmd = state.first_session_cmd;

    let seam = &mut status.seam;
    // What reached a cycle. In this driver the seam is a channel and a cycle
    // reads it directly, so the datagrams handed to a cycle are the datagrams
    // that arrived.
    seam.queued = state.session_cmds_taken + state.goals_taken;
    seam.goals = state.goals_taken;
    seam.session_cmds = state.session_cmds_taken;
    // Datagrams that arrived whole and did not read as anything this build
    // knows. The other seam failures below are the transport's, and this
    // transport has none of them.
    seam.invalid = state.undecodable_inbound;

    let cycle = &mut status.cycle;
    cycle.goals_executed = state.goals_executed;
    cycle.goals_dropped = state.goals_dropped;
    cycle.hold_timeouts = state.hold_timeouts;
    // A cycle either reads every row's registers or reads none of them: the
    // outage this plant models is the wire's, so there is no partial answer to
    // a grouped read and no write that fails to land.
    cycle.blind_cycles = state.blind_cycles_total;
    cycle.events_dropped = state.events_dropped;
    cycle.aux_refused = state.aux_refused;
    cycle.aux_duplicates = state.aux_duplicates;
    cycle.health_reports = state.health_reports;
    cycle.health_misses = state.health_misses;
    cycle.confirm_misses = state.confirm_misses;

    let looped = &mut status.loop_counts;
    looped.cycles = state.cycles;
    looped.skipped = state.skipped_cycles;
    // Once, on the first execution, as it is on every run of the real driver.
    looped.startup_mrc = 1;
    looped.taken = state.session_cmds_taken + state.goals_taken;

    // The record's own publication is not in this: it has not happened yet.
    status.published = state.published;
    // A wind-down is a thing the process this cog stands in for does on its way
    // out. Nothing winds this one down -- the deterministic runner and the host
    // run both end by stopping the process -- so no copy of this record ever
    // says one happened, and a reader of one of these logs is reading a run
    // that did not finish one.
    status.wound_down = false.into();
}

counters! {
    /// The run's totals, as the slot holds them.
    ///
    /// Every one of these is an absolute count since the process started, so a
    /// report carries the run's number whichever reporting window it lands in.
    /// They are read off the slot's own bytes rather than the validated view,
    /// because they are the one part of the state that survives a slot this cog
    /// could not read: a plain number is a plain number whatever the bytes
    /// around it say.
    ///
    /// Reported on the executions where one of them moved. Writing them every
    /// cycle would put an observation in the group at the bus rate and roll its
    /// reporting window every few seconds, for numbers that mostly do not
    /// change.
    ///
    /// Untested: no assertion in this repo covers the values a signal carries,
    /// so which total reaches which signal is held by the pairs below.
    /// TODO(cogs-signal-report-contents)
    Counters of SimStateWire, MotorSimSignals<'_>, crossing the_run_totals_cross_the_slot {
        /// Goals written to the modelled servos.
        goals_executed / set_goals_executed,
        /// Goals refused because the queue was full.
        goals_dropped / set_goals_dropped,
        /// Times the dead-man latched torque off.
        hold_timeouts / set_hold_timeouts,
        /// Goals naming servos this build does not know.
        refused_goals / set_refused_goals,
        /// Events raised on a cycle that had already reported one.
        events_dropped / set_events_dropped,
        /// Times the slot did not read back as a state.
        refused_state_fields / set_refused_state_fields,
        /// Injections this build could not carry out: an operation it does not
        /// know, one naming servos it does not know, or a register write a
        /// control table refused.
        refused_injections / set_refused_injections,
        /// Plant readings that never reached a control table, the angles a
        /// non-finite modelled world produced.
        refused_readings / set_refused_readings,
        /// Host datagrams the driver turned away as asks it could not act on:
        /// one asking nothing, or a transaction offered while another was
        /// pending.
        aux_refused / set_aux_refused,
        /// Host datagrams this build could not read at all: a schema-version
        /// mismatch at the boundary rather than an ask the driver refused.
        undecodable_inbound / set_undecodable_inbound,
    }
}

counters! {
    /// The run's totals that only the status record reads.
    ///
    /// Separate from [`Counters`] because nothing reports these as signals: the
    /// record is where they are read, and putting them in the signal group would
    /// publish every one of them twice. They are carried across a refused state
    /// slot for the reason the reported totals are -- the record says of itself
    /// that it is cumulative since the process started, so a reader that has
    /// seen one copy has read the run, and a counter that went backwards
    /// mid-run would let a real loss read as a full account.
    StatusTotals of SimStateWire, crossing the_status_totals_cross_the_slot {
        /// Cycles run.
        cycles / set_cycles,
        /// Grid points the runner passed over without a cycle.
        skipped_cycles / set_skipped_cycles,
        /// Host datagrams taken off the input, whatever they said.
        session_cmds_taken / set_session_cmds_taken,
        /// Setpoints taken off the input.
        goals_taken / set_goals_taken,
        /// Messages this driver has published.
        published / set_published,
        /// Delivery re-issues the slot recognised.
        aux_duplicates / set_aux_duplicates,
        /// Health reports published.
        health_reports / set_health_reports,
        /// Rotation turns the modelled bus did not answer.
        health_misses / set_health_misses,
        /// Cycles in which the modelled bus answered nothing.
        blind_cycles_total / set_blind_cycles_total,
        /// Torque-off read-backs nothing answered.
        confirm_misses / set_confirm_misses,
    }
}
