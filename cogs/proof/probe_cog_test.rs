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
use brenn_reachy__cogs__proof__probe_msgs_clk_rs::ProbeCmd;
use clockwork__clockwork__io__var_packet_clk_rs::VarPacket__288;
use clockwork_rs::{Duration, SyncTime};
use reachy_wire::{GoalSetpoint, JOINT_MASK_ALL, peek_header};

/// The instant every case starts from. Round rather than zero, so a time that
/// travelled through the wrong field is a number nothing else in the case is.
const T0: i64 = 1_700_000_000_000_000_000;

/// The grid the cases step on, matching the motion system's control period.
const PERIOD: i64 = 20_000_000;

/// One command, as the scenario's input log would carry it.
fn cmd(at_ns: i64, position: f64) -> ProbeCmd {
    let mut msg = ProbeCmd::new();
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

/// The datagram an execution published, decoded, or `None` if it published
/// nothing.
fn published(cog: &mut ProbeTestWrapper) -> Option<(u32, GoalSetpoint)> {
    let packet = cog.try_next_packet()?;
    let (header, goal) =
        GoalSetpoint::decode(packet.bytes().as_slice()).expect("a whole GoalSetpoint datagram");
    Some((header.seq, goal))
}

/// Feed a datagram back into the cog's own-output view, as the channel does.
fn echo_back(cog: &mut ProbeTestWrapper, seq: u32, goal: &GoalSetpoint, at_ns: i64) {
    let mut packet = VarPacket__288::new();
    assert!(packet.try_set_bytes(&goal.encode(seq)));
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

    let (seq, goal) = published(&mut cog).expect("the execution published a datagram");
    assert_eq!(seq, 0, "the first datagram is sequence zero");
    assert_eq!(goal.execute_at_ns, T0 + 2 * PERIOD);
    assert_eq!(goal.mask, 1, "one masked row, the one the probe writes");
    assert_eq!(goal.targets[0], 0.25);
    assert_eq!(goal.targets[1..], [0.0; 8], "unwritten rows stay zero");
    assert_eq!(cog.state_kept().commands_seen(), 1);
}

#[test]
fn the_first_execution_reads_an_empty_view_of_its_own_output() {
    let mut cog = probe();
    cog.initialize(SyncTime::from_nanos(T0));
    cog.publish_cmds(&cmd(T0 + PERIOD, 1.0), SyncTime::from_nanos(T0));
    assert!(cog.execute(SyncTime::from_nanos(T0)));

    // Nothing was ever published before this execution, and the state records
    // the zero that stands for it rather than a sequence number from anywhere.
    assert_eq!(cog.state_kept().echoed_seq(), 0);
    assert_eq!(published(&mut cog).expect("a datagram").0, 0);
}

#[test]
fn the_sequence_number_comes_back_out_of_the_cogs_own_output() {
    let mut cog = probe();
    cog.initialize(SyncTime::from_nanos(T0));

    for step in 0..4u32 {
        let at = T0 + i64::from(step) * PERIOD;
        cog.publish_cmds(&cmd(at + PERIOD, f64::from(step)), SyncTime::from_nanos(at));
        assert!(cog.execute(SyncTime::from_nanos(at)));

        let (seq, goal) = published(&mut cog).expect("a datagram per execution");
        assert_eq!(seq, step, "sequence numbers run consecutively");
        assert_eq!(goal.targets[0], f64::from(step));

        // What the channel does between executions, done by hand: the wrapper
        // wires nothing, so the loop is closed here.
        echo_back(&mut cog, seq, &goal, at);
    }

    assert_eq!(cog.state_kept().commands_seen(), 4);
    // The last execution read back the datagram before its own.
    assert_eq!(cog.state_kept().echoed_seq(), 2);
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

    let (_, goal) = published(&mut cog).expect("one datagram, not three");
    assert_eq!(goal.execute_at_ns, T0 + 3 * PERIOD, "the last command wins");
    assert_eq!(goal.targets[0], 2.0);
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

/// Bytes in the own-output view that are not a datagram read as "nothing
/// published", which restarts the sequence at zero.
///
/// This cog is the only publisher on that channel and it writes whole
/// datagrams, so the branch is unreachable in the system as composed -- but it
/// is a branch, the motion cogs take the same read-back-your-own-output shape,
/// and a checker asserting a publisher's sequence is monotonic would read the
/// restart as a fresh run rather than as damage. Stated here so that it is a
/// behaviour rather than an accident.
#[test]
fn bytes_that_are_not_a_datagram_read_as_nothing_published() {
    let mut cog = probe();
    cog.initialize(SyncTime::from_nanos(T0));

    let rubbish = [0xff_u8, 0x00, 0x7f];
    assert!(
        peek_header(&rubbish).is_err(),
        "the case rests on these bytes not parsing as a header",
    );
    let mut packet = VarPacket__288::new();
    assert!(packet.try_set_bytes(&rubbish));
    cog.publish_own_packet(&packet, SyncTime::from_nanos(T0));

    cog.publish_cmds(&cmd(T0 + PERIOD, 0.5), SyncTime::from_nanos(T0));
    assert!(cog.execute(SyncTime::from_nanos(T0)));

    assert_eq!(
        published(&mut cog).expect("a datagram").0,
        0,
        "an unreadable view is the same as an empty one",
    );
    assert_eq!(cog.state_kept().echoed_seq(), 0);
}

/// The sequence is the wire format's: a counter that wraps rather than one
/// that saturates or panics at its last value.
#[test]
fn the_sequence_after_the_last_one_is_zero_again() {
    let mut cog = probe();
    cog.initialize(SyncTime::from_nanos(T0));

    let goal = GoalSetpoint {
        execute_at_ns: T0,
        mask: 1,
        targets: [0.0; 9],
    };
    echo_back(&mut cog, u32::MAX, &goal, T0);

    cog.publish_cmds(&cmd(T0 + PERIOD, 0.5), SyncTime::from_nanos(T0));
    assert!(cog.execute(SyncTime::from_nanos(T0)));

    assert_eq!(published(&mut cog).expect("a datagram").0, 0, "it wrapped");
    assert_eq!(cog.state_kept().echoed_seq(), u32::MAX);
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
    assert_eq!(bytes.len(), GoalSetpoint::LEN);
    let header = peek_header(bytes).expect("a well-formed header");
    assert_eq!(header.msg_type, GoalSetpoint::MSG_TYPE);
    let (_, goal) = GoalSetpoint::decode(bytes).expect("a whole datagram");
    assert_eq!(bytes, &goal.encode(header.seq)[..]);
    assert_eq!(goal.mask & !JOINT_MASK_ALL, 0, "no rows outside the bus");
}
