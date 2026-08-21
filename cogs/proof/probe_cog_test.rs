//! Unit tests for the bring-up probe, against the generated test wrappers.
//!
//! The wrapper wires no channels, so the self-loop `probe.clk` composes is
//! reproduced here by hand: what an execution publishes is fed back into
//! `own_packet` before the next one. That is the same shape the motion cogs'
//! unit tests will use, and it is what makes the empty-view case -- the first
//! execution, which has published nothing -- reachable in a test at all.
//!
//! Time is passed per call rather than simulated: the harness has no clock, so
//! a case says at what time each execution happens.

use brenn_reachy__cogs__proof__probe_clk_rs_test::ProbeTestWrapper;
use brenn_reachy__cogs__proof__probe_msgs_clk_rs::{ProbeCmdWire, ProbeStep, ProbeStepWire};
use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
use brenn_reachy__motion__joints_clk_rs::JointFlagsWire;
use clockwork__clockwork__io__var_packet_clk_rs::VarPacket__288Wire;
use clockwork_rs::{Blob as _, Duration, SyncTime, blob_as_bytes, blob_from_bytes};
use probe_scenario::PROBE_MASK;

/// The instant every case starts from. Round rather than zero, so a time that
/// travelled through the wrong field is a number nothing else in the case is.
const T0: i64 = 1_700_000_000_000_000_000;

/// The grid the cases step on, matching the motion system's control period.
const PERIOD: i64 = 20_000_000;

/// One command, as the scenario's input log would carry it.
fn cmd(at_ns: i64, position: f64) -> ProbeCmdWire {
    let mut msg = ProbeCmdWire::new();
    msg.set_at(SyncTime::from_nanos(at_ns));
    msg.set_hold(Duration::from_nanos(PERIOD));
    msg.set_position(position);
    msg
}

/// A wrapper with both inputs sized and not yet primed.
///
/// Unprimed matters: an input can only be sized before `initialize`, so this is
/// the state every case starts from. `own_packet` gets one slot because the
/// cog's view of it is one message deep -- the last thing it published.
fn probe() -> ProbeTestWrapper {
    let mut cog = ProbeTestWrapper::new();
    cog.input_cmds_set_num_slots(4);
    cog.input_own_packet_set_num_slots(1);
    cog
}

/// The setpoint an execution published, or `None` if it published nothing.
fn published(cog: &mut ProbeTestWrapper) -> Option<GoalSetpointWire> {
    let packet = cog.try_next_packet()?;
    Some(
        blob_from_bytes::<GoalSetpointWire>(packet.bytes().as_slice())
            .expect("a whole setpoint's bytes"),
    )
}

/// Feed a datagram back into the cog's own-output view, as the channel does.
fn echo_back(cog: &mut ProbeTestWrapper, goal: &GoalSetpointWire, at_ns: i64) {
    let mut packet = VarPacket__288Wire::new();
    assert!(packet.try_set_bytes(blob_as_bytes(goal)));
    cog.publish_own_packet(&packet, SyncTime::from_nanos(at_ns));
}

#[test]
fn a_command_becomes_one_datagram_at_the_commanded_instant() {
    let mut cog = probe();
    cog.initialize(SyncTime::from_nanos(T0));

    // Nothing has arrived, so the one execution condition is unsatisfied and
    // the body never runs.
    assert!(!cog.execute(SyncTime::from_nanos(T0 + PERIOD)));

    cog.publish_cmds(
        &cmd(T0 + 2 * PERIOD, 0.25),
        SyncTime::from_nanos(T0 + PERIOD),
    );
    assert!(cog.execute(SyncTime::from_nanos(T0 + PERIOD)));

    let goal = published(&mut cog).expect("the execution published a datagram");
    assert_eq!(goal.execute_at().as_nanos(), T0 + 2 * PERIOD);
    assert_eq!(
        goal.mask().to_known(),
        Some(PROBE_MASK),
        "one servo, the one it writes"
    );
    assert_eq!(goal.targets().body_yaw(), 0.25);

    // Every field the cog did not write still holds the schema's declared
    // zero, asserted as bytes rather than field by field: a cleared setpoint
    // carrying only the three the cog sets is the whole property, and it stays
    // the whole property when a field is added.
    let mut expected = GoalSetpointWire::new();
    expected.set_execute_at(SyncTime::from_nanos(T0 + 2 * PERIOD));
    expected.set_mask(JointFlagsWire::from(PROBE_MASK));
    expected.targets_mut().set_body_yaw(0.25);
    assert_eq!(
        blob_as_bytes(&goal),
        blob_as_bytes(&expected),
        "unwritten fields stay at their declared zero"
    );
    assert_eq!(cog.state_kept().commands_seen(), 1);
}

#[test]
fn the_first_execution_reads_an_empty_view_of_its_own_output() {
    let mut cog = probe();
    cog.initialize(SyncTime::from_nanos(T0));
    cog.publish_cmds(&cmd(T0 + PERIOD, 1.0), SyncTime::from_nanos(T0));
    assert!(cog.execute(SyncTime::from_nanos(T0)));

    // Nothing was ever published before this execution, and the state records
    // the epoch that stands for it rather than an instant from anywhere.
    assert_eq!(cog.state_kept().echoed_due().as_nanos(), 0);
    assert!(
        published(&mut cog).is_some(),
        "and it published all the same"
    );
}

#[test]
fn what_was_published_comes_back_out_of_the_cogs_own_output() {
    let mut cog = probe();
    cog.initialize(SyncTime::from_nanos(T0));

    for step in 0..4u32 {
        let at = T0 + i64::from(step) * PERIOD;
        cog.publish_cmds(&cmd(at + PERIOD, f64::from(step)), SyncTime::from_nanos(at));
        assert!(cog.execute(SyncTime::from_nanos(at)));

        let goal = published(&mut cog).expect("a datagram per execution");
        assert_eq!(goal.targets().body_yaw(), f64::from(step));

        // What the channel does between executions, done by hand: the wrapper
        // wires nothing, so the loop is closed here.
        echo_back(&mut cog, &goal, at);
    }

    assert_eq!(cog.state_kept().commands_seen(), 4);
    // The last execution read back the datagram before its own, which named the
    // instant the third command asked for.
    assert_eq!(
        cog.state_kept().echoed_due().as_nanos(),
        T0 + 3 * PERIOD,
        "the one published before this execution's own",
    );
}

#[test]
fn a_burst_counts_every_command_and_publishes_the_last() {
    let mut cog = probe();
    cog.initialize(SyncTime::from_nanos(T0));

    for step in 0..3 {
        cog.publish_cmds(
            &cmd(T0 + i64::from(step + 1) * PERIOD, f64::from(step)),
            SyncTime::from_nanos(T0),
        );
    }
    assert!(cog.execute(SyncTime::from_nanos(T0)));

    let goal = published(&mut cog).expect("one datagram, not three");
    assert_eq!(
        goal.execute_at().as_nanos(),
        T0 + 3 * PERIOD,
        "the last command wins"
    );
    assert_eq!(goal.targets().body_yaw(), 2.0);
    assert!(
        published(&mut cog).is_none(),
        "an output slot carries one message per execution",
    );
    assert_eq!(
        cog.state_kept().commands_seen(),
        3,
        "all three were counted"
    );
}

/// Bytes in the own-output view that are not a whole setpoint read as "nothing
/// published", which leaves the read-back at the epoch.
///
/// This cog is the only publisher on that channel and it writes whole messages,
/// so the branch is unreachable in the system as composed -- but it is a branch,
/// the motion cogs take the same read-back-your-own-output shape, and a
/// consumer would otherwise read damage as a run that had just started. Stated
/// here so that it is a behaviour rather than an accident.
#[test]
fn bytes_that_are_not_a_whole_setpoint_read_as_nothing_published() {
    let mut cog = probe();
    cog.initialize(SyncTime::from_nanos(T0));

    let rubbish = [0xff_u8, 0x00, 0x7f];
    assert_ne!(
        rubbish.len(),
        GoalSetpointWire::SIZE,
        "the case rests on these bytes not being a whole message",
    );
    let mut packet = VarPacket__288Wire::new();
    assert!(packet.try_set_bytes(&rubbish));
    cog.publish_own_packet(&packet, SyncTime::from_nanos(T0));

    cog.publish_cmds(&cmd(T0 + PERIOD, 0.5), SyncTime::from_nanos(T0));
    assert!(cog.execute(SyncTime::from_nanos(T0)));

    assert!(
        published(&mut cog).is_some(),
        "an unreadable view is the same as an empty one",
    );
    assert_eq!(cog.state_kept().echoed_due().as_nanos(), 0);
}

#[test]
fn a_datagram_read_back_out_of_the_carrier_is_byte_identical() {
    // What the carrier is asked to do is hold bytes unchanged. The cog fills it
    // and the wrapper's own-input path hands it back, so this asserts across
    // both.
    let mut cog = probe();
    cog.initialize(SyncTime::from_nanos(T0));
    cog.publish_cmds(&cmd(T0, -1.5), SyncTime::from_nanos(T0));
    assert!(cog.execute(SyncTime::from_nanos(T0)));

    let packet = cog.try_next_packet().expect("a datagram");
    let bytes = packet.bytes().as_slice();
    assert_eq!(bytes.len(), GoalSetpointWire::SIZE);
    let goal = blob_from_bytes::<GoalSetpointWire>(bytes).expect("a whole setpoint");
    assert_eq!(bytes, blob_as_bytes(&goal), "the same bytes, reinterpreted");
    assert!(
        goal.validate().is_ok(),
        "and a message this build can read: no undeclared servo in the mask",
    );
}

/// The state slot read as its validated view: real fields, a real Rust enum,
/// and the same bytes the open accessors read.
///
/// The cog body keeps its state this way, and every cog in this repo is meant
/// to. Asserted from a test rather than trusted from upstream's own suite, which
/// exercises the validated layer outside a cog.
#[test]
fn the_state_slot_reads_as_its_validated_view() {
    let mut cog = probe();
    cog.initialize(SyncTime::from_nanos(T0));

    // A slot nobody has written yet: zero, which every field of the schema
    // narrows cleanly from.
    let fresh = cog.state_kept().validate().expect("a zeroed slot is valid");
    assert_eq!(fresh.commands_seen, 0);
    assert!(matches!(fresh.last_step, ProbeStep::Idle));

    cog.publish_cmds(&cmd(T0 + PERIOD, 0.5), SyncTime::from_nanos(T0));
    assert!(cog.execute(SyncTime::from_nanos(T0)));

    let after = cog.state_kept().validate().expect("still valid");
    assert!(matches!(after.last_step, ProbeStep::Published));
    // The view is a reinterpretation of the slot's own bytes, not a copy taken
    // from it: what the open accessor reads and what the field holds are one
    // value.
    assert_eq!(after.commands_seen, cog.state_kept().commands_seen());
    assert_eq!(after.echoed_due.as_nanos(), 0);
}

/// A slot whose bytes no longer validate is restarted from cleared, and the
/// restart is counted on the slot it cleared.
///
/// The damage is written through the open surface, which is the only way an
/// undeclared discriminant reaches a slot at all -- exactly as a peer writing
/// the shared memory would. What matters is that the count and the offset
/// survive the clear: an unrecorded reset reads afterwards like a process that
/// has just started.
#[test]
fn a_slot_that_fails_validation_is_cleared_and_counted() {
    let mut cog = probe();
    cog.initialize(SyncTime::from_nanos(T0));

    cog.publish_cmds(&cmd(T0, 0.25), SyncTime::from_nanos(T0));
    assert!(cog.execute(SyncTime::from_nanos(T0)));
    assert_eq!(cog.state_kept().commands_seen(), 1);

    // A discriminant the schema does not declare, in the one enum field the
    // slot carries.
    cog.state_kept_mut().set_last_step(ProbeStepWire(7));
    assert!(cog.state_kept().validate().is_err(), "the slot is damaged");

    cog.publish_cmds(&cmd(T0 + PERIOD, 0.5), SyncTime::from_nanos(T0 + PERIOD));
    assert!(cog.execute(SyncTime::from_nanos(T0 + PERIOD)));

    let after = cog.state_kept().validate().expect("restarted from cleared");
    assert_eq!(after.state_resets, 1, "the reset is on the record");
    assert_eq!(
        after.last_reset_offset, 20,
        "the offset names the enum field whose bytes were bad",
    );
    // The clear is a clear: the count the damaged slot carried is gone, and this
    // execution's own command is the only one counted.
    assert_eq!(after.commands_seen, 1);
    assert!(matches!(after.last_step, ProbeStep::Published));

    // A second reset counts on top of the first rather than restating it.
    cog.state_kept_mut().set_last_step(ProbeStepWire(7));
    cog.publish_cmds(
        &cmd(T0 + 2 * PERIOD, 0.75),
        SyncTime::from_nanos(T0 + 2 * PERIOD),
    );
    assert!(cog.execute(SyncTime::from_nanos(T0 + 2 * PERIOD)));
    assert_eq!(
        cog.state_kept()
            .validate()
            .expect("valid again")
            .state_resets,
        2,
    );
}
