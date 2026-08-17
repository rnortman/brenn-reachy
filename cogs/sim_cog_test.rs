//! Unit tests for the simulated driver, against the generated test wrapper.
//!
//! The wrapper wires no channels, so each case publishes the goals and
//! injections it wants seen, runs one cycle, and reads back the datagrams that
//! went out and what the slot holds. The one thing it does not do for us is the
//! self-loop: this cog reads its own last published sample and event to number
//! the next ones, so every cycle here hands the wrapper back what the previous
//! cycle published, which is exactly what a box's `connect` would have done.
//!
//! Time is passed per call rather than simulated. The cog is woken by a timer,
//! so a case says at what instant each cycle happens and the harness decides
//! whether the condition is met -- which is itself asserted: a cycle offered
//! early must not run.

use brenn_reachy__cogs__config_clk_rs::SimParams;
use brenn_reachy__cogs__msgs_clk_rs::{JointFlags, Joints, SimCmd, SimOp, SimState};
use brenn_reachy__cogs__sim_clk_rs_test::MotorSimTestWrapper;
use clockwork__clockwork__io__var_packet_clk_rs::{VarPacket__64, VarPacket__128, VarPacket__288};
use clockwork_rs::SyncTime;
use motion_slots::{joint_flags, rows_from_joints, write_joints};
use reachy_driver::QUEUE_CAP;
use reachy_kin::default_geometry;
use reachy_motion::disarm::stow_targets;
use reachy_motion::joints::{JointId, JointSet};
use reachy_wire::{DriverEvent, EventKind, GoalSetpoint, JOINT_COUNT, JOINT_MASK_ALL, PoseSample};
use sim_slots::read_sim;

/// The instant every case starts from. Round rather than zero, so a time that
/// travelled through the wrong field is a number nothing else in the case is.
const T0: i64 = 1_700_000_000_000_000_000;

/// The bus cycle, which is what the cog's one execution condition waits for.
const PERIOD: i64 = 20_000_000;

/// How long the goal stream may be silent before the gate de-torques.
const HOLD_TIMEOUT: i64 = 200_000_000;

/// Per-cycle slew of the cranks and the body yaw, radians.
const SLEW_LEGS: f64 = 0.15;

/// Per-cycle slew of the antennas, radians.
const SLEW_ANTENNAS: f64 = 0.65;

/// The lag a well-formed goal stream commands at: a goal names the grid instant
/// two cycles ahead of the sample that produced it.
const LAG: i64 = 2;

/// The most cycles of motion one execution may make up, which is the cog's own
/// `MAX_CATCHUP_CYCLES` -- a private constant of a crate this test drives
/// through its wrapper, so it is restated here and asserted against.
const MAX_CATCHUP_CYCLES: f64 = 8.0;

/// The nine angles the machine rests at, which is where every case finds it.
fn stow_rows() -> [f64; JOINT_COUNT] {
    rows_from_joints(&stow_targets(default_geometry()).expect("the baked geometry reaches stow"))
}

/// What one cycle published.
struct Cycle {
    /// The cycle's grid instant.
    nominal: i64,
    /// The sample, which every cycle publishes without exception.
    sample: PoseSample,
    /// Its wire sequence number.
    sample_seq: u32,
    /// The event, if the cycle had one to report.
    event: Option<(DriverEvent, u32)>,
}

/// A cog under test, with its own channels looped back by hand.
struct Sim {
    /// The wrapper.
    cog: MotorSimTestWrapper,
    /// The instant of the last cycle run.
    now: i64,
    /// The sequence number the next goal datagram carries. The control loop
    /// numbers its own datagrams; here the harness stands in for it.
    goal_seq: u32,
}

impl Sim {
    /// A simulated driver at stow, de-torqued, on the default parameters.
    fn new() -> Self {
        Self::with(false)
    }

    /// The same, starting energised -- what a scenario that arms the machine
    /// before the process starts would find.
    fn with(start_torqued: bool) -> Self {
        Self::with_params(start_torqued, |_| {})
    }

    /// The same, with the scenario's parameters altered before the cog reads
    /// them -- which is how a case says what a badly configured scenario is.
    fn with_params(start_torqued: bool, edit: impl FnOnce(&mut SimParams)) -> Self {
        let mut cog = MotorSimTestWrapper::new();
        cog.input_goals_set_num_slots(8);
        cog.input_cmds_set_num_slots(8);
        cog.input_own_pose_set_num_slots(1);
        cog.input_own_evt_set_num_slots(1);

        // Seeded after `initialize`: a config record is not reachable before
        // the wrapper has stood the cog up, and the first execution has not run
        // yet, so nothing reads it in between.
        cog.initialize(SyncTime::from_nanos(T0));

        let mut params = SimParams::new();
        params.set_period_ns(PERIOD);
        params.set_hold_timeout_ns(HOLD_TIMEOUT);
        params.set_start_torqued(start_torqued);
        params.set_slew_legs_rad(SLEW_LEGS);
        params.set_slew_body_yaw_rad(SLEW_LEGS);
        params.set_slew_antennas_rad(SLEW_ANTENNAS);
        edit(&mut params);
        cog.set_config_params(&params);

        Self {
            cog,
            now: T0,
            goal_seq: 0,
        }
    }

    /// Hand the cog a goal datagram, as the control loop's channel would.
    fn send_goal(&mut self, goal: &GoalSetpoint) {
        let seq = self.goal_seq;
        self.goal_seq = self.goal_seq.wrapping_add(1);
        let mut packet = VarPacket__128::new();
        assert!(
            packet.try_set_bytes(&goal.encode(seq)),
            "the carrier holds a GoalSetpoint datagram",
        );
        self.cog
            .publish_goals(&packet, SyncTime::from_nanos(self.now));
    }

    /// Hand the cog one of the scenario's injections.
    fn inject(&mut self, op: SimOp, mask: JointSet) {
        let mut cmd = SimCmd::new();
        cmd.set_op(op);
        cmd.set_mask(joint_flags(mask));
        self.cog.publish_cmds(&cmd, SyncTime::from_nanos(self.now));
    }

    /// Inject a command carrying a payload the plain form has no room for.
    fn inject_full(&mut self, cmd: &SimCmd) {
        self.cog.publish_cmds(cmd, SyncTime::from_nanos(self.now));
    }

    /// Run one cycle and loop this cog's own publications back to it.
    fn step(&mut self) -> Cycle {
        self.step_by(1)
    }

    /// The same, run `cycles` after the last one rather than one -- what a
    /// process that lost the CPU for a while looks like to a cog on a timer.
    fn step_by(&mut self, cycles: i64) -> Cycle {
        self.now += cycles * PERIOD;
        assert!(
            self.cog.execute(SyncTime::from_nanos(self.now)),
            "the cycle's timer is due at {}",
            self.now
        );

        let bytes = self
            .cog
            .try_next_pose()
            .map(|packet| packet.bytes().as_slice().to_vec())
            .expect("every cycle publishes a sample");
        let event_bytes = self
            .cog
            .try_next_evt()
            .map(|packet| packet.bytes().as_slice().to_vec());

        let (header, sample) = PoseSample::decode(&bytes).expect("a sample datagram");
        let event = event_bytes.as_ref().map(|bytes| {
            let (header, event) = DriverEvent::decode(bytes).expect("an event datagram");
            (event, header.seq)
        });

        // The self-loop, which a box would have made: the next cycle's sequence
        // numbers are read out of these.
        let mut packet = VarPacket__288::new();
        assert!(packet.try_set_bytes(&bytes));
        self.cog
            .publish_own_pose(&packet, SyncTime::from_nanos(self.now));
        if let Some(bytes) = event_bytes {
            let mut packet = VarPacket__64::new();
            assert!(packet.try_set_bytes(&bytes));
            self.cog
                .publish_own_evt(&packet, SyncTime::from_nanos(self.now));
        }

        Cycle {
            nominal: self.now,
            sample,
            sample_seq: header.seq,
            event,
        }
    }

    /// Run `count` cycles, sending nothing.
    fn quiet(&mut self, count: usize) -> Vec<Cycle> {
        (0..count).map(|_| self.step()).collect()
    }

    /// Run one cycle having commanded `targets` for the rows in `mask`, at the
    /// lag a well-formed goal stream uses.
    fn commanded_step(&mut self, targets: &[f64; JOINT_COUNT], mask: u16) -> Cycle {
        let goal = GoalSetpoint {
            execute_at_ns: self.now + PERIOD + LAG * PERIOD,
            mask,
            targets: *targets,
        };
        self.send_goal(&goal);
        self.step()
    }

    /// The same, `cycles` after the last one: the goal stream carries on, so
    /// what the late cycle shows is the plant catching up and not the dead-man.
    fn commanded_step_by(&mut self, targets: &[f64; JOINT_COUNT], mask: u16, cycles: i64) -> Cycle {
        let goal = GoalSetpoint {
            execute_at_ns: self.now + cycles * PERIOD,
            mask,
            targets: *targets,
        };
        self.send_goal(&goal);
        self.step_by(cycles)
    }

    /// What the state slot holds, read the way everything reads it.
    fn slot(&self) -> sim_slots::SimSlot {
        read_sim(self.cog.state_sim())
    }
}

#[test]
fn the_machine_starts_stowed_de_torqued_and_saying_so() {
    let mut sim = Sim::new();

    assert!(!sim.cog.execute(SyncTime::from_nanos(T0 + PERIOD - 1)));
    assert!(sim.cog.try_next_pose().is_none());

    let cycle = sim.step();
    assert_eq!(cycle.sample.nominal_time_ns, cycle.nominal);
    assert_eq!(
        cycle.sample.sample_time_ns, cycle.nominal,
        "a simulated bus read has no jitter to report"
    );
    assert_eq!(cycle.sample.present, stow_rows(), "the machine is at rest");
    assert!(cycle.sample.present_valid);
    assert!(
        !cycle.sample.commanded_valid,
        "nothing has been commanded yet"
    );
    assert!(!cycle.sample.torque_off_latched);
    assert_eq!(cycle.sample.miss_mask, 0);
    assert!(cycle.event.is_none(), "a quiet cycle reports nothing");

    let slot = sim.slot();
    assert!(slot.initialized);
    assert_eq!(
        slot.plant.torqued,
        JointSet::EMPTY,
        "the scenario has not armed it"
    );
    assert_eq!(slot.plant.positions, stow_rows());
}

#[test]
fn a_sample_goes_out_every_cycle_and_its_sequence_comes_from_the_last_one() {
    let mut sim = Sim::new();
    for (step, cycle) in sim.quiet(5).into_iter().enumerate() {
        assert_eq!(
            cycle.sample_seq, step as u32,
            "the first datagram is sequence zero and each one after is the last plus one",
        );
        assert_eq!(
            cycle.sample.nominal_time_ns,
            T0 + (step as i64 + 1) * PERIOD
        );
    }
}

#[test]
fn a_goal_is_written_at_its_instant_and_not_before() {
    let mut sim = Sim::with(true);
    let mut targets = stow_rows();
    targets[0] += 0.05;

    let first = sim.commanded_step(&targets, JOINT_MASK_ALL);
    assert!(
        !first.sample.commanded_valid,
        "the goal is queued, not yet due"
    );
    assert_eq!(sim.slot().gate.queued().len(), 1);

    let waiting = sim.step();
    assert!(!waiting.sample.commanded_valid, "still one cycle early");

    let due = sim.step();
    assert!(due.sample.commanded_valid, "its instant has come round");
    assert_eq!(due.sample.commanded, targets);
    assert_eq!(sim.slot().counters.goals_executed, 1);
    assert_eq!(
        due.sample.present[0],
        stow_rows()[0] + 0.05,
        "a step inside one cycle's slew arrives whole"
    );

    // Every quiet cycle after it rewrites the same setpoint, which is what
    // holds a servo's position loop awake. A rewrite is not a goal executed:
    // counting one would make this total climb at the bus rate and wreck the
    // number a scenario checker reads it for.
    for step in 0..4 {
        let quiet = sim.step();
        assert_eq!(quiet.sample.commanded, targets, "cycle {step}");
        assert_eq!(
            sim.slot().counters.goals_executed,
            1,
            "cycle {step}: one goal has been executed, however many rewrites",
        );
    }
}

#[test]
fn a_servo_moves_no_further_than_its_slew_in_one_cycle() {
    let mut sim = Sim::with(true);
    // Further than any servo can travel in a cycle, so every cycle is a
    // full-rate one and the rate is what is asserted.
    let mut targets = stow_rows();
    targets[0] += 10.0;
    targets[1] += 10.0;
    targets[7] += 10.0;

    let mut previous = stow_rows();
    for step in 0..8 {
        let cycle = sim.commanded_step(&targets, JOINT_MASK_ALL);
        if step < LAG {
            assert_eq!(cycle.sample.present, previous, "cycle {step}");
            continue;
        }
        assert_close(
            cycle.sample.present[0] - previous[0],
            SLEW_LEGS,
            "the body yaw moves at its own rate",
        );
        assert_close(
            cycle.sample.present[1] - previous[1],
            SLEW_LEGS,
            "a crank moves at the legs' rate",
        );
        assert_close(
            cycle.sample.present[7] - previous[7],
            SLEW_ANTENNAS,
            "an antenna moves at the antennas' rate",
        );
        previous = cycle.sample.present;
    }
}

#[test]
fn a_partial_mask_moves_only_its_own_rows() {
    let mut sim = Sim::with(true);
    let start = stow_rows();
    let mut targets = start;
    for row in &mut targets {
        *row += 0.05;
    }
    // Two rows, chosen apart so a mask applied to the wrong end is visible.
    let rows = set_of(&[JointId::BodyYaw, JointId::AntennaLeft]);
    let mask = rows.bits();

    for _ in 0..(LAG + 2) {
        sim.commanded_step(&targets, mask);
    }
    let settled = sim.step();

    assert_close(settled.sample.present[0], targets[0], "a masked row moves");
    assert_close(settled.sample.present[8], targets[8], "so does the other");
    for (row, resting) in start.iter().enumerate().take(8).skip(1) {
        assert_eq!(
            settled.sample.present[row], *resting,
            "row {row} was not in the mask and must not have moved",
        );
    }
    let slot = sim.slot();
    assert_eq!(slot.plant.has_target, rows, "only those rows are commanded");
}

#[test]
fn a_jammed_servo_holds_while_the_rest_of_the_machine_tracks() {
    let mut sim = Sim::with(true);
    let start = stow_rows();
    let mut targets = start;
    for row in &mut targets {
        *row += 0.05;
    }

    // One crank jammed where it stands. Everything else is asked for the same
    // move, so what separates them is the obstruction and nothing else.
    sim.inject(SimOp::OBSTRUCT, one(JointId::Leg(2)));
    for _ in 0..(LAG + 2) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }
    let jammed = sim.step();
    assert_eq!(
        jammed.sample.present[3], start[3],
        "a jammed servo does not move, whatever it is asked for"
    );
    assert_close(jammed.sample.present[1], targets[1], "its neighbour tracks");

    sim.inject(SimOp::RELEASE_OBSTRUCTION, one(JointId::Leg(2)));
    let freed = sim.commanded_step(&targets, JOINT_MASK_ALL);
    assert_close(
        freed.sample.present[3],
        targets[3],
        "a released servo resumes tracking",
    );
}

#[test]
fn a_de_torqued_servo_holds_where_it_stands() {
    let mut sim = Sim::new();
    let start = stow_rows();
    let mut targets = start;
    targets[1] += 1.0;

    for _ in 0..(LAG + 3) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }
    let cycle = sim.step();
    assert_eq!(
        cycle.sample.present, start,
        "a write to a de-torqued servo moves nothing"
    );
    assert!(
        cycle.sample.commanded_valid,
        "the setpoint was taken up all the same, as a real bus write would be"
    );
}

#[test]
fn a_teleport_puts_the_servos_where_the_scenario_says() {
    let mut sim = Sim::new();
    let mut positions = Joints::new();
    let mut wanted = stow_rows();
    wanted[2] = 1.25;
    wanted[8] = -0.5;
    write_joints(&mut positions, &motion_slots::joints_from_rows(&wanted));

    let mut cmd = SimCmd::new();
    cmd.set_op(SimOp::SET_POSITIONS);
    cmd.set_mask(joint_flags(set_of(&[
        JointId::Leg(1),
        JointId::AntennaLeft,
    ])));
    *cmd.positions_mut() = positions;
    sim.inject_full(&cmd);

    let cycle = sim.step();
    assert_eq!(
        cycle.sample.present, wanted,
        "the named rows, and only them"
    );
}

#[test]
fn a_cycle_whose_replies_were_lost_says_so_and_the_machine_keeps_moving() {
    let mut sim = Sim::with(true);
    let mut targets = stow_rows();
    targets[1] += 10.0;

    for _ in 0..(LAG + 1) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }

    let mut cmd = SimCmd::new();
    cmd.set_op(SimOp::DROP_REPLIES);
    cmd.set_count(3);
    sim.inject_full(&cmd);

    let mut last = None;
    for step in 0..3 {
        let cycle = sim.commanded_step(&targets, JOINT_MASK_ALL);
        assert!(
            !cycle.sample.present_valid,
            "cycle {step}: nothing was read this cycle"
        );
        assert_eq!(cycle.sample.miss_mask, JOINT_MASK_ALL, "cycle {step}");
        last = Some(cycle.sample.present[1]);
    }

    let back = sim.commanded_step(&targets, JOINT_MASK_ALL);
    assert!(
        back.sample.present_valid,
        "the outage was three cycles long"
    );
    assert_eq!(back.sample.miss_mask, 0);
    assert!(
        back.sample.present[1] > last.expect("three cycles ran"),
        "the machine kept moving while nobody could see it"
    );
}

#[test]
fn silence_de_torques_the_machine_and_is_announced_exactly_once() {
    let mut sim = Sim::with(true);
    let mut targets = stow_rows();
    targets[1] += 0.05;
    sim.commanded_step(&targets, JOINT_MASK_ALL);

    // The dead-man's window is measured from the last accepted goal, so it
    // runs out one cycle past the timeout.
    let mut latched = Vec::new();
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 4) {
        let cycle = sim.step();
        if let Some((event, _)) = cycle.event {
            latched.push((event, cycle.nominal));
        }
    }

    assert_eq!(latched.len(), 1, "one stall, one event");
    let (event, at) = latched[0];
    assert_eq!(event.kind, EventKind::HoldTimeoutTorqueOff);
    assert_eq!(event.time_ns, at);
    let silence = i64::from(event.detail) * 1_000;
    assert!(
        (HOLD_TIMEOUT..=HOLD_TIMEOUT + PERIOD).contains(&silence),
        "the event says how long the stream was silent, and it is the window it \
         ran past: {silence} ns",
    );
    assert!(
        at <= T0 + 2 * PERIOD + HOLD_TIMEOUT + PERIOD,
        "the dead-man fires within a cycle of its window running out, not at {at}",
    );

    let slot = sim.slot();
    assert!(slot.gate.latched, "the latch stands until a fresh arming");
    assert_eq!(slot.plant.torqued, JointSet::EMPTY);
    assert_eq!(slot.counters.hold_timeouts, 1);

    let after = sim.step();
    assert!(
        after.sample.torque_off_latched,
        "a standing latch is visible in every sample"
    );
    assert!(
        after.event.is_none(),
        "a standing condition is not re-announced"
    );
}

#[test]
fn silence_before_the_first_goal_de_torques_too() {
    // The window a dead-man written around the held setpoint would miss: armed,
    // torqued, and the commander never said anything at all.
    let mut sim = Sim::new();
    sim.inject(SimOp::TORQUE_ON, all_joints());

    let mut events = Vec::new();
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 3) {
        if let Some((event, _)) = sim.step().event {
            events.push(event);
        }
    }

    assert_eq!(events.len(), 1);
    assert_eq!(events[0].kind, EventKind::HoldTimeoutTorqueOff);
    let slot = sim.slot();
    assert!(!slot.gate.has_held, "nothing was ever commanded");
    assert_eq!(slot.plant.torqued, JointSet::EMPTY);
}

#[test]
fn arming_ends_a_latch_and_grants_a_fresh_window() {
    let mut sim = Sim::new();
    sim.inject(SimOp::TORQUE_ON, all_joints());
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 3) {
        sim.step();
    }
    assert!(
        sim.slot().gate.latched,
        "the case rests on a standing latch"
    );

    sim.inject(SimOp::TORQUE_ON, all_joints());
    let armed = sim.step();
    assert!(
        !armed.sample.torque_off_latched,
        "a fresh arming ends the latch"
    );
    assert_eq!(sim.slot().plant.torqued, all_joints());

    // And the window is measured from the arming rather than from the stall it
    // ended, so nothing fires for another whole timeout.
    for _ in 0..(HOLD_TIMEOUT / PERIOD - 1) {
        assert!(sim.step().event.is_none());
    }
}

/// A machine energised again after the dead-man took it down stands still until
/// something commands it.
///
/// The hazard is uncommanded motion at torque-on: a gate or a plant that kept
/// the dead session's setpoint would move the machine to a position no live
/// commander asked for, in the whole window before the new commander's first
/// goal lands. Recovery is a fresh engagement, and nothing from before the
/// de-torquing is part of it.
#[test]
fn a_machine_re_armed_after_a_stall_moves_nothing_until_it_is_told_to() {
    let mut sim = Sim::with(true);
    let mut targets = stow_rows();
    targets[1] += 10.0;
    for _ in 0..(LAG + 2) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }
    assert!(
        sim.step().sample.commanded_valid,
        "a setpoint is held, and the machine is well short of it"
    );

    // The stream stops. The machine slews on under the held setpoint until the
    // dead-man takes the torque away, and where it stood then is where it must
    // still stand when it is energised again.
    let mut stalled_at = stow_rows();
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 3) {
        stalled_at = sim.step().sample.present;
    }
    assert!(
        sim.slot().gate.latched,
        "the case rests on a standing latch"
    );

    sim.inject(SimOp::TORQUE_ON, all_joints());
    for step in 0..5 {
        let cycle = sim.step();
        assert!(!cycle.sample.torque_off_latched, "cycle {step}");
        assert_eq!(
            cycle.sample.present, stalled_at,
            "cycle {step}: the machine moved with nobody commanding it",
        );
        assert!(
            !cycle.sample.commanded_valid,
            "cycle {step}: the dead session's setpoint is not this session's",
        );
    }

    let mut fresh = stalled_at;
    fresh[1] += 0.05;
    for _ in 0..(LAG + 1) {
        sim.commanded_step(&fresh, JOINT_MASK_ALL);
    }
    let moving = sim.step();
    assert!(moving.sample.commanded_valid);
    assert_close(
        moving.sample.present[1],
        fresh[1],
        "the machine follows what this session asked for",
    );
}

/// A disarm that did not reach every servo is not a confirmed one: the rows
/// still energised are still being held, and forgetting the setpoint would stop
/// the gate rewriting to servos that are holding a position under torque.
#[test]
fn a_partial_disarm_keeps_the_setpoint_the_rest_of_the_machine_is_holding() {
    let mut sim = Sim::with(true);
    let mut targets = stow_rows();
    targets[1] += 0.05;
    for _ in 0..(LAG + 2) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }
    assert!(sim.slot().gate.has_held, "something is being held");

    let antennas = set_of(&[JointId::AntennaRight, JointId::AntennaLeft]);
    sim.inject(SimOp::TORQUE_OFF, antennas);
    let cycle = sim.step();

    assert!(
        cycle.sample.commanded_valid,
        "the cranks are still holding what they were asked for"
    );
    assert_eq!(cycle.sample.commanded, targets);
    assert!(!cycle.sample.torque_off_latched, "and nothing latched");

    let slot = sim.slot();
    assert!(slot.gate.has_held);
    assert_eq!(
        slot.plant.torqued,
        all_joints().without(antennas),
        "the named rows went off and no others"
    );
    assert_eq!(
        slot.plant.has_target,
        all_joints(),
        "and every row still has the setpoint it was given"
    );
}

#[test]
fn a_confirmed_disarm_forgets_the_setpoint_without_latching() {
    let mut sim = Sim::with(true);
    let mut targets = stow_rows();
    targets[1] += 0.05;
    for _ in 0..(LAG + 2) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }
    assert!(sim.slot().gate.has_held, "something is being held");

    sim.inject(SimOp::TORQUE_OFF, all_joints());
    let cycle = sim.step();
    assert!(
        !cycle.sample.torque_off_latched,
        "a deliberate disarm is not a fault"
    );
    assert!(
        !cycle.sample.commanded_valid,
        "nothing is being held any more"
    );

    let slot = sim.slot();
    assert_eq!(slot.plant.torqued, JointSet::EMPTY);
    assert_eq!(slot.plant.has_target, JointSet::EMPTY);
    assert!(!slot.gate.latched);
}

#[test]
fn a_sender_overrunning_the_queue_is_dropped_and_told() {
    let mut sim = Sim::with(true);
    let targets = stow_rows();

    // More goals in one cycle than the queue holds, all stamped far enough
    // ahead that none of them becomes due and drains a slot.
    for i in 0..(QUEUE_CAP as i64 + 2) {
        let goal = GoalSetpoint {
            execute_at_ns: sim.now + (10 + i) * PERIOD,
            mask: JOINT_MASK_ALL,
            targets,
        };
        sim.send_goal(&goal);
    }
    let cycle = sim.step();

    let (event, seq) = cycle.event.expect("the overrun is reported");
    assert_eq!(event.kind, EventKind::GoalDroppedQueueFull);
    assert_eq!(
        event.detail, QUEUE_CAP as u32,
        "and says how deep the queue was when it hit it"
    );
    assert_eq!(seq, 0, "the first event this cog ever published");
    let slot = sim.slot();
    assert_eq!(slot.counters.goals_dropped, 2);
    assert_eq!(
        slot.counters.events_dropped, 1,
        "two drops, one slot to report them in"
    );
    assert_eq!(slot.gate.queued().len(), QUEUE_CAP);
}

#[test]
fn a_goal_stamped_for_an_instant_already_past_is_taken_and_warned() {
    let mut sim = Sim::with(true);
    let goal = GoalSetpoint {
        execute_at_ns: T0 - PERIOD,
        mask: JOINT_MASK_ALL,
        targets: stow_rows(),
    };
    sim.send_goal(&goal);
    let cycle = sim.step();

    let (event, _) = cycle.event.expect("a stale goal is remarked on");
    assert_eq!(event.kind, EventKind::GoalStaleOrOutOfOrder);
    assert_eq!(
        event.detail,
        u32::try_from(2 * PERIOD / 1_000).expect("forty milliseconds is a small number"),
        "and says how far past its instant it arrived, microseconds",
    );
    assert!(
        cycle.sample.commanded_valid,
        "and executed all the same, in arrival order"
    );
}

/// A goal stamped ahead of now but behind the one before it has missed nothing
/// yet, so what it is late by is nothing. The `detail` a stale goal carries is
/// how far past its instant it arrived, and this is the other situation the
/// same outcome covers.
#[test]
fn a_goal_merely_out_of_order_says_it_is_late_by_nothing() {
    let mut sim = Sim::with(true);
    let targets = stow_rows();
    let later = GoalSetpoint {
        execute_at_ns: sim.now + 5 * PERIOD,
        mask: JOINT_MASK_ALL,
        targets,
    };
    let earlier = GoalSetpoint {
        execute_at_ns: sim.now + 3 * PERIOD,
        mask: JOINT_MASK_ALL,
        targets,
    };
    sim.send_goal(&later);
    sim.send_goal(&earlier);
    let cycle = sim.step();

    let (event, _) = cycle.event.expect("the reordering is remarked on");
    assert_eq!(event.kind, EventKind::GoalStaleOrOutOfOrder);
    assert_eq!(
        event.detail, 0,
        "a goal whose instant is still ahead is late by nothing"
    );
    assert_eq!(
        sim.slot().gate.queued().len(),
        2,
        "and both are queued, in arrival order"
    );
}

/// Two events on one cycle, and the slot goes to the one about the machine.
///
/// The queue is kept full so every cycle drops a goal and says so; a dropped
/// goal is not liveness, so the dead-man runs out underneath that stream and
/// one cycle raises both. The machine de-torquing itself is what an operator
/// has to see, and the goal event is the one that is counted instead.
#[test]
fn the_dead_mans_latch_takes_the_slot_from_a_goal_event_raised_first() {
    let mut sim = Sim::with(true);
    let targets = stow_rows();
    // Far enough ahead that none of them ever becomes due and drains a slot.
    fn overrun(sim: &mut Sim, targets: &[f64; JOINT_COUNT]) {
        let goal = GoalSetpoint {
            execute_at_ns: sim.now + 1_000 * PERIOD,
            mask: JOINT_MASK_ALL,
            targets: *targets,
        };
        sim.send_goal(&goal);
    }
    for _ in 0..QUEUE_CAP {
        overrun(&mut sim, &targets);
    }
    sim.step();
    assert_eq!(sim.slot().gate.queued().len(), QUEUE_CAP);

    let mut latches = Vec::new();
    let mut dropped_before_latch = 0;
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 4) {
        let before = sim.slot().counters.events_dropped;
        overrun(&mut sim, &targets);
        let cycle = sim.step();
        let after = sim.slot().counters.events_dropped;
        match cycle.event {
            Some((event, _)) if event.kind == EventKind::HoldTimeoutTorqueOff => {
                assert_eq!(
                    after,
                    before + 1,
                    "the goal event it displaced is counted, not lost quietly"
                );
                latches.push(event);
            }
            Some((event, _)) => {
                assert_eq!(event.kind, EventKind::GoalDroppedQueueFull);
                assert_eq!(after, before, "one event, one slot, nothing displaced");
                if latches.is_empty() {
                    dropped_before_latch += 1;
                }
            }
            None => {}
        }
    }

    assert_eq!(latches.len(), 1, "one stall, one latch, announced once");
    assert!(
        dropped_before_latch > 0,
        "the case rests on goal events being raised on the way there"
    );
    assert!(sim.slot().gate.latched);
}

/// An injection this build cannot read does not reach the machine at all -- and
/// the part of a torque-on that is not about its mask is the part that matters:
/// ending a torque-off latch. A refusal that still ended one would clear the
/// state the latch exists to hold, and the counter would say the injection did
/// nothing.
#[test]
fn a_refused_arming_does_not_end_a_latch() {
    let mut sim = Sim::new();
    sim.inject(SimOp::TORQUE_ON, all_joints());
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 3) {
        sim.step();
    }
    assert!(
        sim.slot().gate.latched,
        "the case rests on a standing latch"
    );

    let mut cmd = SimCmd::new();
    cmd.set_op(SimOp::TORQUE_ON);
    cmd.set_mask(JointFlags(1 << JOINT_COUNT));
    sim.inject_full(&cmd);
    let cycle = sim.step();

    assert!(
        cycle.sample.torque_off_latched,
        "a set nobody could read energises nothing and ends nothing"
    );
    let slot = sim.slot();
    assert!(slot.gate.latched);
    assert_eq!(slot.plant.torqued, JointSet::EMPTY);
    assert_eq!(
        slot.counters.refused_injections, 1,
        "counted once for the one injection"
    );
}

/// Dropping replies reads no mask, so nothing about its mask can refuse it.
#[test]
fn an_injection_that_reads_no_mask_is_not_refused_over_one() {
    let mut sim = Sim::with(true);
    let mut cmd = SimCmd::new();
    cmd.set_op(SimOp::DROP_REPLIES);
    cmd.set_mask(JointFlags(1 << JOINT_COUNT));
    cmd.set_count(2);
    sim.inject_full(&cmd);

    let cycle = sim.step();
    assert!(!cycle.sample.present_valid, "the outage happened");
    assert_eq!(
        sim.slot().counters.refused_injections,
        0,
        "and no refusal was counted for a set the operation never reads"
    );
}

#[test]
fn a_datagram_the_codec_refuses_is_counted_and_nothing_else() {
    let mut sim = Sim::with(true);
    let goal = GoalSetpoint {
        execute_at_ns: T0 + PERIOD,
        mask: JOINT_MASK_ALL,
        targets: stow_rows(),
    };
    // A well-formed goal with the version byte bumped past this build's, which
    // is the failure a driver upgrade produces online.
    let mut bytes = goal.encode(0);
    bytes[2] = bytes[2].wrapping_add(1);
    assert!(GoalSetpoint::decode(&bytes).is_err());
    let mut packet = VarPacket__128::new();
    assert!(packet.try_set_bytes(&bytes));
    sim.cog
        .publish_goals(&packet, SyncTime::from_nanos(sim.now));

    let cycle = sim.step();
    assert!(
        cycle.event.is_none(),
        "bytes that are not a setpoint name no instant to report at"
    );
    assert!(!cycle.sample.commanded_valid, "nothing was queued");
    let slot = sim.slot();
    assert_eq!(slot.counters.undecodable_goals, 1);
    assert_eq!(
        slot.counters.goals_dropped, 0,
        "a drop is a different thing"
    );
}

#[test]
fn an_event_sequence_carries_on_from_the_last_event_however_long_ago() {
    let mut sim = Sim::with(true);
    let stale = GoalSetpoint {
        execute_at_ns: T0 - PERIOD,
        mask: JOINT_MASK_ALL,
        targets: stow_rows(),
    };
    sim.send_goal(&stale);
    let first = sim.step();
    assert_eq!(first.event.expect("the first event").1, 0);

    sim.quiet(5);
    let mut later = stale;
    later.execute_at_ns = sim.now - PERIOD;
    sim.send_goal(&later);
    let second = sim.step();
    assert_eq!(
        second.event.expect("the second event").1,
        1,
        "an event view holds the last event however long ago it was sent",
    );
}

#[test]
fn a_late_cycle_makes_up_whole_cycles_of_motion_and_no_more() {
    let mut sim = Sim::with(true);
    // Further than any catch-up covers, so every cycle asserted here is a
    // full-rate one and the rate is what is being counted.
    let mut targets = stow_rows();
    targets[1] += 100.0;
    for _ in 0..(LAG + 2) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }

    let on_time = sim.commanded_step(&targets, JOINT_MASK_ALL);
    let late = sim.commanded_step_by(&targets, JOINT_MASK_ALL, 5);
    assert_close(
        late.sample.present[1] - on_time.sample.present[1],
        5.0 * SLEW_LEGS,
        "five cycles' worth of motion for five cycles of lost time",
    );

    let very_late = sim.commanded_step_by(&targets, JOINT_MASK_ALL, 20);
    assert_close(
        very_late.sample.present[1] - late.sample.present[1],
        MAX_CATCHUP_CYCLES * SLEW_LEGS,
        "and no further than the clamp, whatever the gap was",
    );
}

#[test]
fn an_injection_this_build_cannot_carry_out_is_counted_and_does_nothing() {
    let mut sim = Sim::with(true);

    // A set naming a tenth bus row, which no machine has: refused rather than
    // masked down to the nine it does have.
    let mut cmd = SimCmd::new();
    cmd.set_op(SimOp::TORQUE_OFF);
    cmd.set_mask(JointFlags(1 << JOINT_COUNT));
    sim.inject_full(&cmd);
    sim.step();

    let slot = sim.slot();
    assert_eq!(slot.counters.refused_injections, 1);
    assert_eq!(
        slot.plant.torqued,
        all_joints(),
        "nothing was de-torqued by a set nobody could read"
    );
    assert_eq!(
        slot.counters.refused_state_fields, 0,
        "an injection is not a field of this cog's own slot"
    );

    // An operation this build does not know, which is what a scenario written
    // against a newer vocabulary sends.
    let mut cmd = SimCmd::new();
    cmd.set_op(SimOp(200));
    cmd.set_mask(joint_flags(all_joints()));
    sim.inject_full(&cmd);
    sim.step();

    let slot = sim.slot();
    assert_eq!(slot.counters.refused_injections, 2);
    assert_eq!(slot.plant.torqued, all_joints());
}

/// A cycle that is not the cycle this cog is woken on is a scenario describing
/// a bus nobody built. Refused on the first execution, where it is a
/// configuration mistake, rather than modelled into plausible numbers.
///
/// What the refusal said is on stderr rather than in the panic these cases
/// see: a panic in a cog body crosses the generated C++ boundary and reaches
/// the wrapper as an exception, which re-panics in its own words.
#[test]
#[should_panic(expected = "MotorSim test wrapper: execute() failed")]
fn a_scenario_whose_cycle_is_not_the_bus_cycle_is_refused() {
    let mut sim = Sim::with_params(true, |params| params.set_period_ns(PERIOD / 2));
    sim.step();
}

/// The same for a slew that is not a distance: an unset rate leaves a machine
/// that never moves, and a negative one is nobody's intent.
#[test]
#[should_panic(expected = "MotorSim test wrapper: execute() failed")]
fn a_scenario_whose_servos_have_no_rate_is_refused() {
    let mut sim = Sim::with_params(true, |params| params.set_slew_antennas_rad(0.0));
    sim.step();
}

/// And the same for a dead-man that allows no silence at all: an unset
/// `hold_timeout_ns` is zero, and a machine that de-torques itself on the first
/// cycle it holds through passes every dead-man assertion a scenario could make
/// while failing every other one somewhere unrelated.
#[test]
#[should_panic(expected = "MotorSim test wrapper: execute() failed")]
fn a_scenario_whose_dead_man_allows_no_silence_is_refused() {
    let mut sim = Sim::with_params(true, |params| params.set_hold_timeout_ns(0));
    sim.step();
}

/// The queue depth is one number written in two languages: the schema's array
/// and the gate's constant. A disagreement must fail a build rather than panic
/// inside the cog that owns the dead-man.
#[test]
fn the_slots_queue_is_as_deep_as_the_gates() {
    assert_eq!(SimState::new().queue().capacity(), QUEUE_CAP);
}

/// A set holding one joint.
fn one(joint: JointId) -> JointSet {
    set_of(&[joint])
}

/// A set holding those joints.
fn set_of(joints: &[JointId]) -> JointSet {
    let mut set = JointSet::EMPTY;
    for joint in joints {
        set.insert(*joint);
    }
    set
}

/// Assert two angles are the same to within a band far tighter than any step
/// this plant takes.
fn assert_close(found: f64, wanted: f64, what: &str) {
    assert!(
        (found - wanted).abs() < 1e-9,
        "{what}: {found} is not {wanted}"
    );
}

/// Every servo on the bus.
fn all_joints() -> JointSet {
    JointSet::from_bits(JOINT_MASK_ALL).expect("every bus row is a bus row")
}
