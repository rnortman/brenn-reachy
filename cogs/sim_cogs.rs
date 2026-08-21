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
//! Nothing here holds state of its own. The plant, the gate and the run's
//! totals are the state slot's own fields, read and written through the
//! validated view the cycle opens once at the top.

use brenn_reachy__cogs__config_clk_rs::{SimParams, SimParamsWire};
use brenn_reachy__cogs__sim_clk_rs::{MotorSimDial, MotorSimOutputs, MotorSimSignals};
use brenn_reachy__cogs__sim_state_clk_rs::{SimCmd, SimOp, SimState, SimStateWire};
use brenn_reachy__driver__health_clk_rs::{DriverEvent, EventKind};
use brenn_reachy__driver__pose_clk_rs::PoseSample;
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use clockwork_rs::{Duration, SyncTime};
use motion_slots::{configured, counters};
use reachy_driver::{AcceptOutcome, GateAction, GoalGate};
use reachy_kin::default_geometry;
use reachy_motion::disarm::stow_targets;
use reachy_motion::joints::{
    JointGroup, angle_of, flags, group_of, rows_of, set_angle, write_rows, write_vector,
};

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
    let mut refused_state = 0;
    if dial.states.sim.validate_mut().is_err() {
        // Bytes that did not read as a state. This cog is the slot's only
        // writer, so a refusal is memory nobody wrote: the run starts again
        // from a cleared slot with the totals it had carried put back, and the
        // refusal is counted rather than raised -- the process whose job is to
        // de-torque a machine does not get to panic over its own memory.
        dial.states.sim.clear_valid();
        before.store(dial.states.sim);
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
        // Torqued only where this is the process starting. A restart from a slot
        // the cycle could not read is not that: whatever arming the run had is
        // in the bytes that were refused, and the dead-man may have latched the
        // machine off. So the modelled machine comes back de-torqued and waits
        // to be armed, rather than energising itself out of a memory fault --
        // the one transition the latch exists to prevent.
        if bool::from(params.start_torqued) && refused_state == 0 {
            state.torqued = all_rows();
            GoalGate::over(&mut state.gate).note_liveness(nominal);
        }
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

    for setpoint in dial.inputs.goals.new_msgs() {
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
    let torqued = !flags::is_empty(state.torqued);
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
    advance(params, state, cycles);

    // The sample: published every cycle without exception, because it is the
    // clock the control loop runs on and a missing one is a cycle the loop
    // never sees. A cycle whose replies were lost is a sample saying so, never
    // an absent sample.
    let blind = state.drop_replies_left > 0;
    state.drop_replies_left = state.drop_replies_left.saturating_sub(1);

    publish(
        &mut dial.outputs,
        state,
        blind,
        nominal,
        report.event.as_ref(),
    );

    Counters::read(dial.states.sim).report(&before, &mut dial.signals);
}

/// The one event a cycle reports, and how many it could not.
///
/// An output carries one message per execution, so a cycle that raises two
/// events publishes one. The dead-man's latch outranks the rest: it is the
/// machine changing state, where the others are remarks about a datagram that
/// the sender can see the consequences of anyway.
#[derive(Default)]
struct Report {
    /// What will be published, if anything.
    event: Option<Event>,
}

/// One edge the cycle hit, before it reaches the output slot.
///
/// Held as ordinary Rust because a cycle can raise more than one and the slot
/// carries one: the ranking is a decision made here rather than a sequence of
/// writes into the slot.
#[derive(Clone, Copy)]
struct Event {
    kind: EventKind,
    time_ns: i64,
    /// The silence or the lateness, where the kind names one.
    silence_ns: i64,
    /// The servos the kind names, where it names a set of them.
    rows: JointFlags,
    /// How many of whatever the kind counts.
    count: u32,
    /// The one servo the kind names, as its bus id.
    id: u8,
}

impl Event {
    /// An event at `time_ns` with no evidence yet: a raiser fills in the fields
    /// its own kind names and leaves the rest, which is what the schema says a
    /// kind that does not name a field carries.
    fn at(time_ns: i64) -> Self {
        Self {
            kind: EventKind::None,
            time_ns,
            silence_ns: 0,
            rows: JointFlags::NONE,
            count: 0,
            id: 0,
        }
    }
}

impl Report {
    /// Offer an event for this cycle's one slot, counting the one it displaces.
    fn raise(&mut self, dropped: &mut u64, event: Event) {
        match self.event {
            None => self.event = Some(event),
            Some(held) => {
                *dropped += 1;
                if event.kind == EventKind::HoldTimeoutTorqueOff && held.kind != event.kind {
                    self.event = Some(event);
                }
            }
        }
    }
}

/// Apply one of the scenario's injections to the modelled machine.
///
/// These stand in for the physical world and for the arming sequencer. An
/// operation this build does not know is refused and counted: the scenario is
/// describing something to a simulator that cannot do it, which is a fact
/// about the scenario and not about the machine.
///
/// A refusal is the whole injection and not part of one, and it happens at the
/// caller's one validation: an operation this build does not know and a set of
/// servos it cannot read are the same fact about the scenario, and carrying out
/// the readable half of one would be worse than carrying out none of it -- a
/// torque-on whose mask was refused would energise nothing and still end a
/// torque-off latch, which is the one transition the latch exists to guard.
fn inject(state: &mut SimState, cmd: &SimCmd, nominal: i64) {
    let mask = cmd.mask;
    match cmd.op {
        SimOp::TorqueOn => {
            state.torqued |= mask;
            // Arming grants a fresh hold-timeout window, whether or not there
            // was a latch to end: the first goal of a session cannot arrive
            // before the session starts.
            let mut gate = GoalGate::over(&mut state.gate);
            if gate.state().latched.get() {
                gate.release_latch(nominal);
            } else {
                gate.note_liveness(nominal);
            }
        }
        SimOp::TorqueOff => {
            state.torqued = flags::without(state.torqued, mask);
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
        // The value a slot nothing wrote holds. A scenario never authors one,
        // and nothing about the machine changes for it.
        SimOp::Nop => {}
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

/// Every servo on the bus.
fn all_rows() -> JointFlags {
    JointGroup::ALL
        .into_iter()
        .fold(JointFlags::NONE, |set, group| set | group.joints())
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
        let (Some(target), Some(position)) = (
            angle_of(&state.targets, joint),
            angle_of(&state.positions, joint),
        ) else {
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
    // The modelled truth, whatever the flags say about it. A driver that read
    // nothing this cycle has last cycle's numbers in hand and reports them
    // behind the missing rows; what makes them unusable is that set, not a
    // blank the receiver would have to tell apart from a real zero.
    write_rows(&mut sample.present, &rows_of(&state.positions));
    write_rows(&mut sample.commanded, &rows_of(&state.gate.held.targets));
}

/// Put this cycle's sample, and its event if it has one, on the wire.
///
/// Both channels carry their schema, so both are written into the output slot
/// field by field: nothing is encoded and neither carries a sequence number of
/// its own.
fn publish(
    outputs: &mut MotorSimOutputs<'_>,
    state: &SimState,
    blind: bool,
    nominal: i64,
    event: Option<&Event>,
) {
    let out = &mut outputs.pose;
    write_sample(out.msg_mut().clear_valid(), state, blind, nominal);
    out.mark_for_publish();

    let Some(event) = event else {
        return;
    };
    let out = &mut outputs.evt;
    write_event(out.msg_mut().clear_valid(), event);
    out.mark_for_publish();
}

fn write_event(out: &mut DriverEvent, event: &Event) {
    out.kind = event.kind;
    out.time = SyncTime::from_nanos(event.time_ns);
    out.silence = Duration::from_nanos(event.silence_ns);
    out.rows = event.rows;
    out.count = event.count;
    out.id = event.id;
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
        /// know, or one naming servos it does not know.
        refused_injections / set_refused_injections,
    }
}
