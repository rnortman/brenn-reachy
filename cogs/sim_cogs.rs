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
//! Nothing here holds state of its own: the gate, the plant and the run's
//! totals all cross the state slot through `sim_slots`, once each way.

use brenn_reachy__cogs__msgs_clk_rs::{SimCmd, SimOp};
use brenn_reachy__cogs__sim_clk_rs::MotorSimDial;
use clockwork_rs::Clear as _;
use motion_slots::{joint_set, read_joints, rows_from_joints};
use reachy_driver::{AcceptOutcome, GateAction, Goal, JOINT_MASK_ALL};
use reachy_kin::default_geometry;
use reachy_motion::disarm::stow_targets;
use reachy_motion::joints::{JointGroup, JointSet};
use reachy_wire::{DriverEvent, EventKind, GoalSetpoint, PoseSample, peek_header};
use sim_slots::{Counters, SimSlot, read_sim, rows_of, write_sim};

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
    let hold_timeout_ns = dial.configs.params.hold_timeout_ns();
    let period_ns = dial.configs.params.period_ns();
    // Read from the slot rather than from the crossing, because reading a set
    // of servos this build does not know is itself one of the things counted.
    let before = Counters::read(dial.states.sim);
    let mut sim = read_sim(dial.states.sim);
    let mut report = Report::default();

    let first = !sim.initialized;
    if first {
        check_params(dial);
        sim.initialized = true;
        sim.plant.positions = rows_from_joints(
            &stow_targets(default_geometry()).expect("the baked geometry reaches stow"),
        );
        if dial.configs.params.start_torqued() {
            sim.plant.torqued = all_rows();
            sim.gate.note_liveness(nominal);
        }
    }

    for cmd in dial.inputs.cmds.new_msgs() {
        inject(&mut sim, cmd, nominal);
    }

    for packet in dial.inputs.goals.new_msgs() {
        // A datagram that does not decode is not a setpoint: it names no
        // instant and no rows, so there is nothing to queue and nothing to
        // report about it beyond the count.
        let Ok((_, setpoint)) = GoalSetpoint::decode(packet.bytes().as_slice()) else {
            sim.counters.undecodable_goals += 1;
            continue;
        };
        let depth = sim.gate.queued().len();
        match sim.gate.accept(Goal::from(setpoint), nominal) {
            AcceptOutcome::Accepted => {}
            AcceptOutcome::AcceptedStaleOrOutOfOrder => {
                report.raise(
                    &mut sim.counters,
                    EventKind::GoalStaleOrOutOfOrder,
                    // How far past its instant it arrived. Zero for a goal that
                    // is merely out of order with the one before it, which has
                    // not missed anything yet.
                    micros(nominal - setpoint.execute_at_ns),
                    nominal,
                );
            }
            AcceptOutcome::DroppedQueueFull => {
                sim.counters.goals_dropped += 1;
                report.raise(
                    &mut sim.counters,
                    EventKind::GoalDroppedQueueFull,
                    u32::try_from(depth).unwrap_or(u32::MAX),
                    nominal,
                );
            }
        }
    }

    let silence = nominal - sim.gate.last_accept_ns;
    match sim
        .gate
        .tick(nominal, !sim.plant.torqued.is_empty(), hold_timeout_ns)
    {
        GateAction::WriteTorqueOffSweep { just_latched } => {
            // The sweep reaches every row, and a de-torqued servo holds where
            // it stands: this machine's gearboxes do not back-drive. The
            // modelled servos forget what they were asked for along with the
            // torque, so a machine energised again later stands still until
            // something commands it rather than resuming a move nobody is
            // asking for any more.
            sim.plant.torqued = JointSet::EMPTY;
            sim.plant.has_target = JointSet::EMPTY;
            if just_latched {
                sim.counters.hold_timeouts += 1;
                report.raise(
                    &mut sim.counters,
                    EventKind::HoldTimeoutTorqueOff,
                    // How long the goal stream was silent, which is what tells
                    // an operator whether the commander stopped or merely
                    // stuttered.
                    micros(silence),
                    nominal,
                );
            }
        }
        GateAction::WriteGoal(goal) => {
            sim.counters.goals_executed += 1;
            command(&mut sim, &goal);
        }
        GateAction::Rewrite(goal) => {
            // The same setpoint again, which is what holds a servo's position
            // loop awake between goals. Nothing here counts it: a rewrite is
            // not a goal executed.
            command(&mut sim, &goal);
        }
        GateAction::Nothing => {}
    }

    let cycles = if first {
        1
    } else {
        elapsed_cycles(
            dial.conditions.tick.time_since_last_exec().as_nanos(),
            period_ns,
        )
    };
    advance(dial, &mut sim, cycles);

    // The sample: published every cycle without exception, because it is the
    // clock the control loop runs on and a missing one is a cycle the loop
    // never sees. A cycle whose replies were lost is a sample saying so, never
    // an absent sample.
    let blind = sim.plant.drop_replies_left > 0;
    let sample = PoseSample {
        nominal_time_ns: nominal,
        sample_time_ns: nominal,
        present_valid: !blind,
        commanded_valid: sim.gate.has_held,
        torque_off_latched: sim.gate.latched,
        miss_mask: if blind { JOINT_MASK_ALL } else { 0 },
        // The modelled truth, whatever the flags say about it. A driver that
        // read nothing this cycle has last cycle's numbers in hand and reports
        // them behind a miss mask; what makes them unusable is the mask, not a
        // blank the receiver would have to tell apart from a real zero.
        present: sim.plant.positions,
        commanded: sim.gate.held.targets,
    };
    sim.plant.drop_replies_left = sim.plant.drop_replies_left.saturating_sub(1);

    publish(dial, &sample, report.event.as_ref());
    write_sim(dial.states.sim, &mut sim);
    signal(dial, &sim.counters, &before);
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
    event: Option<DriverEvent>,
}

impl Report {
    /// Offer an event for this cycle's one slot.
    fn raise(&mut self, counters: &mut Counters, kind: EventKind, detail: u32, time_ns: i64) {
        let event = DriverEvent {
            kind,
            detail,
            time_ns,
        };
        match self.event {
            None => self.event = Some(event),
            Some(held) => {
                counters.events_dropped += 1;
                if kind == EventKind::HoldTimeoutTorqueOff && held.kind != kind {
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
/// A refusal is the whole injection and not part of one. An operation whose set
/// of servos does not decode is not carried out at all, because the parts of it
/// that do not read that set are the parts that matter most: a torque-on whose
/// mask was refused would otherwise energise nothing and still end a torque-off
/// latch, which is the one transition the latch exists to guard.
fn inject(sim: &mut SimSlot, cmd: &SimCmd, nominal: i64) {
    let op = cmd.op();
    let mask = if names_servos(op) {
        match joint_set(cmd.mask()) {
            Ok(rows) => rows,
            Err(_) => {
                sim.counters.refused_injections += 1;
                return;
            }
        }
    } else {
        // The mask is not read by this operation, so a value in it decides
        // nothing and refusing over it would refuse an injection this build can
        // carry out perfectly well.
        JointSet::EMPTY
    };
    match op {
        SimOp::TORQUE_ON => {
            sim.plant.torqued = sim.plant.torqued.union(mask);
            // Arming grants a fresh hold-timeout window, whether or not there
            // was a latch to end: the first goal of a session cannot arrive
            // before the session starts.
            if sim.gate.latched {
                sim.gate.release_latch(nominal);
            } else {
                sim.gate.note_liveness(nominal);
            }
        }
        SimOp::TORQUE_OFF => {
            sim.plant.torqued = sim.plant.torqued.without(mask);
            if sim.plant.torqued.is_empty() {
                // A sweep that reached everything is a confirmed disarm, not a
                // fault: nothing latches, and nothing is being held any more.
                sim.gate.clear_commanded();
                sim.plant.has_target = JointSet::EMPTY;
            }
        }
        SimOp::SET_POSITIONS => {
            let positions = rows_from_joints(&read_joints(cmd.positions()));
            for joint in mask.iter() {
                if let Some(row) = joint.index() {
                    sim.plant.positions[row] = positions[row];
                }
            }
        }
        SimOp::OBSTRUCT => sim.plant.obstructed = sim.plant.obstructed.union(mask),
        SimOp::RELEASE_OBSTRUCTION => sim.plant.obstructed = sim.plant.obstructed.without(mask),
        SimOp::DROP_REPLIES => sim.plant.drop_replies_left = cmd.count(),
        // An operation this build does not know, counted where a refused
        // injection is counted: a scenario written against a newer vocabulary
        // than this binary would otherwise run green with its hand on the
        // machine discarded, and fail somewhere unrelated.
        _ => sim.counters.refused_injections += 1,
    }
}

/// Whether an operation acts on the servos its mask names.
///
/// The ones that do not — an unknown operation, and dropping replies, which is
/// about the bus rather than about any servo — read no mask at all, so nothing
/// about their mask can refuse them.
fn names_servos(op: SimOp) -> bool {
    matches!(
        op,
        SimOp::TORQUE_ON
            | SimOp::TORQUE_OFF
            | SimOp::SET_POSITIONS
            | SimOp::OBSTRUCT
            | SimOp::RELEASE_OBSTRUCTION
    )
}

/// Write a due goal's rows into the plant, and remember which rows have one.
///
/// The mask is the goal's own: a setpoint applies to the servos it names and
/// leaves every other one holding what it already had.
fn command(sim: &mut SimSlot, goal: &Goal) {
    let written = goal.apply_to(&mut sim.plant.targets);
    let rows = rows_of(written, &mut sim.counters.refused_state_fields);
    sim.plant.has_target = sim.plant.has_target.union(rows);
}

/// Every servo on the bus.
fn all_rows() -> JointSet {
    JointGroup::ALL
        .into_iter()
        .fold(JointSet::EMPTY, |set, group| set.union(group.joints()))
}

/// A length of time in microseconds, as an event's `detail` carries it.
///
/// Saturating at both ends: an interval that ran backwards is nothing, and one
/// past seventy minutes is as long as this field says.
fn micros(ns: i64) -> u32 {
    u32::try_from(ns.max(0) / 1_000).unwrap_or(u32::MAX)
}

/// Refuse a scenario whose configuration does not describe a machine.
///
/// Run once, on the first execution, before anything is torqued. Every one of
/// these turns a scenario-authoring slip into a plausible-looking run, and a
/// simulator whose plant quietly ran at the wrong rate certifies nothing: the
/// assertions still pass, about a machine nobody meant to build.
fn check_params(dial: &MotorSimDial<'_>) {
    let params = &dial.configs.params;
    assert_eq!(
        params.period_ns(),
        TIMER_PERIOD_NS,
        "the plant's cycle must be the cycle this cog is woken on",
    );
    assert!(
        params.hold_timeout_ns() >= params.period_ns(),
        "the dead-man must allow at least one cycle of silence, not {}ns",
        params.hold_timeout_ns(),
    );
    for (rate, group) in [
        (params.slew_body_yaw_rad(), "body yaw"),
        (params.slew_legs_rad(), "legs"),
        (params.slew_antennas_rad(), "antennas"),
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
fn advance(dial: &MotorSimDial<'_>, sim: &mut SimSlot, cycles: i64) {
    let plant = &mut sim.plant;
    for joint in plant.torqued.iter() {
        if plant.obstructed.contains(joint) || !plant.has_target.contains(joint) {
            continue;
        }
        let Some(row) = joint.index() else {
            continue;
        };
        let step = slew(dial, joint.group()) * cycles as f64;
        let gap = plant.targets[row] - plant.positions[row];
        plant.positions[row] = if gap.abs() <= step {
            plant.targets[row]
        } else {
            plant.positions[row] + step.copysign(gap)
        };
    }
}

/// How far a servo of `group` moves in one cycle, radians.
///
/// Per group rather than per servo: the six cranks carry the head between them
/// and are the same part, the antennas are a different and much faster one, and
/// the body yaw is its own. Each rate is a distance rather than a number that
/// might be one, because [`check_params`] refused the scenario otherwise.
fn slew(dial: &MotorSimDial<'_>, group: JointGroup) -> f64 {
    let params = &dial.configs.params;
    match group {
        JointGroup::BodyYaw => params.slew_body_yaw_rad(),
        JointGroup::Legs => params.slew_legs_rad(),
        JointGroup::Antennas => params.slew_antennas_rad(),
    }
}

/// Put this cycle's sample, and its event if it has one, on the wire.
///
/// Each carries the sequence number after the last one this cog published, read
/// back out of its own view of the channel rather than kept in a state field: a
/// channel is not a queue, so the view over the ring is a loss-free record of
/// what this cog sent, and an empty view means it has never sent one.
fn publish(dial: &mut MotorSimDial<'_>, sample: &PoseSample, event: Option<&DriverEvent>) {
    let pose_seq = next_seq(
        dial.inputs
            .own_pose
            .latest()
            .map(|msg| msg.bytes().as_slice()),
    );
    let out = &mut dial.outputs.pose;
    out.msg_mut().clear();
    assert!(
        out.msg_mut().try_set_bytes(&sample.encode(pose_seq)),
        "the sample carrier is too small for a PoseSample datagram",
    );
    out.mark_for_publish();

    let Some(event) = event else {
        return;
    };
    let evt_seq = next_seq(
        dial.inputs
            .own_evt
            .latest()
            .map(|msg| msg.bytes().as_slice()),
    );
    let out = &mut dial.outputs.evt;
    out.msg_mut().clear();
    assert!(
        out.msg_mut().try_set_bytes(&event.encode(evt_seq)),
        "the event carrier is too small for a DriverEvent datagram",
    );
    out.mark_for_publish();
}

/// The sequence number after the one those bytes carry, or zero if there are
/// none.
///
/// A datagram that does not parse reads as nothing published: this cog is the
/// only publisher on both channels and it writes whole datagrams, so that is a
/// build-time mistake rather than damage to recover from.
fn next_seq(published: Option<&[u8]>) -> u32 {
    published
        .and_then(|bytes| peek_header(bytes).ok())
        .map_or(0, |header| header.seq.wrapping_add(1))
}

/// Report the run's totals, on the executions where one of them moved.
///
/// Writing them every cycle would put an observation in the group at the bus
/// rate and roll its reporting window every few seconds, for numbers that
/// mostly do not change.
///
/// Written out here rather than generated beside the struct, as the
/// control-rate cogs' totals are: this cog's signals type belongs to the
/// generated crate that depends, through this one, on the crate the struct
/// lives in, so it cannot be named there.
///
/// Untested: no assertion in this repo covers the values a signal carries, so
/// which total reaches which signal is held by this function's own reading.
/// TODO(cogs-signal-report-contents)
fn signal(dial: &mut MotorSimDial<'_>, now: &Counters, before: &Counters) {
    if now.goals_executed != before.goals_executed {
        dial.signals.set_goals_executed(now.goals_executed);
    }
    if now.goals_dropped != before.goals_dropped {
        dial.signals.set_goals_dropped(now.goals_dropped);
    }
    if now.hold_timeouts != before.hold_timeouts {
        dial.signals.set_hold_timeouts(now.hold_timeouts);
    }
    if now.undecodable_goals != before.undecodable_goals {
        dial.signals.set_undecodable_goals(now.undecodable_goals);
    }
    if now.events_dropped != before.events_dropped {
        dial.signals.set_events_dropped(now.events_dropped);
    }
    if now.refused_state_fields != before.refused_state_fields {
        dial.signals
            .set_refused_state_fields(now.refused_state_fields);
    }
    if now.refused_injections != before.refused_injections {
        dial.signals.set_refused_injections(now.refused_injections);
    }
}
