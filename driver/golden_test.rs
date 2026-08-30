//! Golden byte vectors for the schemas that cross the process edge.
//!
//! A datagram on either of this machine's two seams -- the driver's and the
//! intent edge's -- is a schema's own blob bytes, so the wire format is the
//! layout the generator produces. Two things pin that layout, and they pin
//! different halves of it:
//!
//! - Sizes, alignments and field offsets are pinned by the generated crate
//!   itself, which carries a const assert per field against the compiler's own
//!   offsets. A drift inside one generation fails the build.
//! - What a *value* looks like as bytes is pinned here: one vector per
//!   edge-crossing schema, built field by field and compared to the bytes it
//!   produced when the vector was written. A regeneration that moves a field or
//!   re-encodes a value is a loud diff in this file rather than a silent
//!   change of what goes out on a socket.
//!
//! The pins are soft: nothing deploys the two ends separately yet, so a
//! deliberate schema edit updates the vector beside it in the same commit. They
//! harden the day the two ends ship apart.
//!
//! Every vector is written little-endian, which is the only byte order
//! `clockwork_rs` compiles for.

use brenn_reachy__cogs__schedule_clk_rs::{PostureWire, StepKindWire};
use brenn_reachy__cogs__script_clk_rs::{ScriptOverlayWire, ScriptStepWire, ScriptWire};
use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
use brenn_reachy__driver__health_clk_rs::{
    AuxOutcomeWire, AuxStatus, DriverEventWire, DriverStatusWire, EventKind, HealthReportWire,
};
use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
use brenn_reachy__driver__wire_clk_rs::DatagramHeaderWire;
use brenn_reachy__hardware__dynamixel__registers_clk_rs::{RegId, ValueShape};
use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKind;
use brenn_reachy__motion__joints_clk_rs::{JointFlags, Joints};
use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, TimelineWire};
use clockwork_rs::{Duration, SyncTime, blob_as_bytes};

/// The instant every vector is stamped with: a round number of seconds since
/// the epoch, so the eight bytes it lands as are readable in the hex.
const T0_NS: i64 = 1_700_000_000_000_000_000;

/// Hex-encode `bytes` so a vector is readable by eye: a field's bytes are
/// found by counting its offset in pairs.
fn hex(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push_str(&format!("{byte:02x}"));
    }
    out
}

/// Compare what a schema's bytes are against what they were, and say which byte
/// disagrees first when they do not.
fn pins(what: &str, bytes: &[u8], golden: &str) {
    let actual = hex(bytes);
    if actual == golden {
        return;
    }
    let at = actual
        .chars()
        .zip(golden.chars())
        .position(|(a, b)| a != b)
        .map_or(golden.len().min(actual.len()) / 2, |nibble| nibble / 2);
    panic!(
        "{what} no longer encodes as its golden vector: first difference at byte {at}\n\
         now:    {actual}\n\
         golden: {golden}"
    );
}

/// Nine angles distinct in every row, so a pair of rows swapped by a layout
/// change is a difference in the bytes rather than a coincidence.
fn nine(out: &mut Joints, first: f64) {
    out.body_yaw = first;
    out.leg_0 = first + 1.0;
    out.leg_1 = first + 2.0;
    out.leg_2 = first + 3.0;
    out.leg_3 = first + 4.0;
    out.leg_4 = first + 5.0;
    out.leg_5 = first + 6.0;
    out.antenna_right = first + 7.0;
    out.antenna_left = first + 8.0;
}

#[test]
fn a_setpoint_is_the_bytes_it_was() {
    let mut msg = GoalSetpointWire::new();
    let goal = msg.clear_valid();
    goal.execute_at = SyncTime::from_nanos(T0_NS);
    goal.mask = JointFlags::BODY_YAW | JointFlags::ANTENNA_LEFT;
    nine(&mut goal.targets, 0.5);

    pins(
        "GoalSetpoint",
        blob_as_bytes(&msg),
        "000000000000e03f000000000000f83f00000000000004400000000000000c40\
         000000000000124000000000000016400000000000001a400000000000001e40\
         000000000000214000002a36fe9c97170101000000000000",
    );
}

#[test]
fn a_sample_is_the_bytes_it_was() {
    let mut msg = PoseSampleWire::new();
    let sample = msg.clear_valid();
    sample.nominal_time = SyncTime::from_nanos(T0_NS);
    sample.sample_time = SyncTime::from_nanos(T0_NS + 1_000_000);
    sample.missing = JointFlags::LEG_2;
    sample.present_valid = false.into();
    sample.commanded_valid = true.into();
    sample.torque_off_latched = true.into();
    nine(&mut sample.present, 1.5);
    nine(&mut sample.commanded, 10.5);

    pins(
        "PoseSample",
        blob_as_bytes(&msg),
        "000000000000f83f00000000000004400000000000000c400000000000001240\
         00000000000016400000000000001a400000000000001e400000000000002140\
         0000000000002340000000000000254000000000000027400000000000002940\
         0000000000002b400000000000002d400000000000002f400000000000803040\
         0000000000803140000000000080324000002a36fe9c971740423936fe9c9717\
         0800000101000000",
    );
}

#[test]
fn an_event_is_the_bytes_it_was() {
    let mut msg = DriverEventWire::new();
    let event = msg.clear_valid();
    event.kind = EventKind::HoldTimeoutTorqueOff;
    event.time = SyncTime::from_nanos(T0_NS);
    event.silence = Duration::from_nanos(250_000_000);
    event.work = Duration::from_nanos(21_500_000);
    event.exchange = Duration::from_nanos(3_250_000);
    event.drain = Duration::from_nanos(2_125_000);
    event.count = 3;
    event.out_of_band = 7;
    event.rows = JointFlags::LEG_0 | JointFlags::LEG_5;
    event.id = 42;

    pins(
        "DriverEvent",
        blob_as_bytes(&msg),
        "00002a36fe9c971780b2e60e0000000060104801000000005097310000000000c86c20000000000\
         003000000070000004200012a00000000",
    );
}

#[test]
fn a_health_report_is_the_bytes_it_was() {
    let mut msg = HealthReportWire::new();
    let report = msg.clear_valid();
    report.id = 7;
    report.bits = 0b0010_0001;
    report.volts = 11.75;
    report.temp_c = -3;
    report.sample_time = SyncTime::from_nanos(T0_NS);

    pins(
        "HealthReport",
        blob_as_bytes(&msg),
        "000000000080274000002a36fe9c97170721fd0000000000",
    );
}

#[test]
fn an_aux_outcome_is_the_bytes_it_was() {
    let mut msg = AuxOutcomeWire::new();
    let outcome = msg.clear_valid();
    outcome.corr = 0x0102_0304;
    outcome.status = AuxStatus::Timeout;
    outcome.value_kind = ValueShape::Radians;
    outcome.value = 0.25f64.to_bits();
    outcome.model = 0x0708;
    outcome.op = AuxOpKind::ReadReg;
    outcome.id = 0x0a;
    outcome.reg = RegId::PresentPosition;

    pins(
        "AuxOutcome",
        blob_as_bytes(&msg),
        "000000000000d03f04030201080703000105020a00000000",
    );
}

#[test]
fn a_driver_status_is_the_bytes_it_was() {
    let mut msg = DriverStatusWire::new();
    let status = msg.clear_valid();
    status.time = SyncTime::from_nanos(T0_NS);
    status.sweep_time = SyncTime::from_nanos(T0_NS - 1);
    status.sweep_failed_rows = JointFlags::LEG_2;
    status.torque_latched = true.into();
    status.first_pose = SyncTime::from_nanos(T0_NS + 1);
    status.first_session_cmd = SyncTime::from_nanos(T0_NS + 2);
    status.wound_down = true.into();
    // Distinct in every field, so a pair swapped by a layout change is a
    // difference in the bytes rather than a coincidence.
    status.seam.queued = 1;
    status.seam.goals = 2;
    status.seam.session_cmds = 3;
    status.seam.wrong_size = 4;
    status.seam.invalid = 5;
    status.seam.overflowed = 6;
    status.seam.undelivered = 7;
    status.seam.recv_errors = 8;
    status.seam.readers_stopped = 9;
    status.cycle.goals_executed = 10;
    status.cycle.goals_dropped = 11;
    status.cycle.hold_timeouts = 12;
    status.cycle.read_misses = 13;
    status.cycle.write_failures = 14;
    status.cycle.blind_cycles = 15;
    status.cycle.events_dropped = 16;
    status.cycle.aux_refused = 17;
    status.cycle.aux_duplicates = 18;
    status.cycle.aux_deferred = 19;
    status.cycle.health_reports = 20;
    status.cycle.health_misses = 21;
    status.cycle.confirm_misses = 22;
    status.loop_counts.cycles = 23;
    status.loop_counts.skipped = 24;
    status.loop_counts.startup_mrc = 25;
    status.loop_counts.wire_failures = 26;
    status.loop_counts.taken = 27;
    status.loop_counts.clock_steps = 28;
    status.published = 29;
    status.publish_failures = 30;

    pins(
        "DriverStatus",
        blob_as_bytes(&msg),
        "0a000000000000000b000000000000000c000000000000000d00000000000000\
         0e000000000000000f0000000000000010000000000000001100000000000000\
         1200000000000000130000000000000014000000000000001500000000000000\
         1600000000000000010000000000000002000000000000000300000000000000\
         0400000000000000050000000000000006000000000000000700000000000000\
         0800000000000000090000000000000017000000000000001800000000000000\
         19000000000000001a000000000000001b000000000000001c00000000000000\
         00002a36fe9c9717ffff2936fe9c971701002a36fe9c971702002a36fe9c9717\
         1d000000000000001e000000000000000800010100000000",
    );
}

/// A script as it goes out on 7409: two base steps and one overlay, every field
/// a distinct value so a layout change that swaps two of them is a difference in
/// the bytes.
///
/// The message is 352 bytes whatever it carries -- the step and overlay arrays
/// are fixed-capacity -- so most of the vector is the zeros of the rows this
/// script does not use, and the count fields are what say how far the used part
/// reaches.
#[test]
fn a_script_is_the_bytes_it_was() {
    // No `clear()` before the fields, here or in any other vector: `new()`
    // starts from an all-zero message, so the rows this script does not use are
    // zeros by construction and a byte that differs is a layout change rather
    // than an initialisation accident.
    let mut msg = ScriptWire::new();
    msg.set_script_id(7);
    msg.set_arrival(SyncTime::from_nanos(T0_NS));
    {
        let mut steps = msg.steps_mut();
        let up: &mut ScriptStepWire = steps.try_grow().expect("the schema holds sixteen");
        up.set_after_ms(8000);
        up.set_duration_ms(2000);
        up.set_kind(StepKindWire::BASE_POSTURE);
        up.set_posture(PostureWire::UP);
        let stow: &mut ScriptStepWire = steps.try_grow().expect("the schema holds sixteen");
        stow.set_after_ms(10_000);
        stow.set_duration_ms(3000);
        stow.set_kind(StepKindWire::BASE_POSTURE);
        stow.set_posture(PostureWire::STOW);
    }
    {
        let mut overlays = msg.overlays_mut();
        let play: &mut ScriptOverlayWire = overlays.try_grow().expect("the schema holds four");
        play.set_motion_id(3);
        play.set_after_ms(8500);
        play.set_duration_ms(1200);
        play.set_gain(1.0);
        play.set_speed(1.5);
    }

    pins(
        "Script",
        blob_as_bytes(&msg),
        "401f0000d00700000101000010270000b80b0000010000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0200000000000000000000000000f03f000000000000f83f34210000b0040000\
         0300000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000010000000000000000002a36fe9c97170700000000000000",
    );
}

/// The session's narration as it goes out on 7410: two rows and a count of rows
/// that fell off the front.
///
/// Fixed at 2064 bytes for the same reason the script is fixed at 352 -- the
/// entry array is the message -- so this vector is mostly the unused rows.
#[test]
fn a_timeline_is_the_bytes_it_was() {
    let mut msg = TimelineWire::new();
    msg.set_dropped(5);
    {
        let mut entries = msg.entries_mut();
        let first: &mut TimelineEntryWire = entries.try_grow().expect("the schema holds many");
        first.set_time(SyncTime::from_nanos(T0_NS));
        first.set_kind(ReportKindWire::SCRIPT_ACCEPTED);
        first.set_a(7);
        first.set_b(2);
        first.set_detail(0.25);
        let second: &mut TimelineEntryWire = entries.try_grow().expect("the schema holds many");
        second.set_time(SyncTime::from_nanos(T0_NS + 1_000_000));
        second.set_kind(ReportKindWire::PHASE_CHANGED);
        second.set_a(1);
        second.set_b(3);
        second.set_detail(-0.5);
    }

    pins(
        "Timeline",
        blob_as_bytes(&msg),
        "00002a36fe9c9717000000000000d03f07000000020000000200000000000000\
         40423936fe9c9717000000000000e0bf01000000030000000100000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         0000000000000000000000000000000000000000000000000000000000000000\
         02000000000000000500000000000000",
    );
}

#[test]
fn a_datagram_header_is_the_bytes_it_was() {
    let mut msg = DatagramHeaderWire::new();
    let header = msg.clear_valid();
    header.magic = 0x5257;
    header.version = 1;
    header.seq = 0x0000_0101;

    pins("DatagramHeader", blob_as_bytes(&msg), "0101000057520100");
}

/// Where the discriminator sits, said out loud rather than left implicit in the
/// hex above: the generator orders fields by alignment, so `seq` takes the
/// leading four bytes and the magic follows it. A receiver written against the
/// header reads this offset, and a layout change that moved it would otherwise
/// only show up as a vector nobody could read.
#[test]
fn the_magic_is_the_two_bytes_after_the_sequence_number() {
    const MAGIC_AT: usize = 4;

    let mut msg = DatagramHeaderWire::new();
    let header = msg.clear_valid();
    header.magic = 0x5257;
    header.version = 1;
    header.seq = u32::MAX;

    let bytes = blob_as_bytes(&msg);
    assert_eq!(
        &bytes[MAGIC_AT..MAGIC_AT + 2],
        b"WR",
        "the magic is not where the header says it is"
    );
    assert_eq!(
        &bytes[..MAGIC_AT],
        &u32::MAX.to_le_bytes(),
        "and what is in front of it is the sequence number"
    );
}
