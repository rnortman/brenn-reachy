//! What the probe, the scenario author and the checker have to agree on.
//!
//! Three programs share this: the cog body writes the servo named here, the
//! author binary turns the table below into the input log, and the checker
//! joins the output log back to that same table. A scenario whose expectations
//! were restated in the checker would pass by agreeing with itself; here there
//! is one statement of what the run is, and the checker's job is to find it in
//! the log.
//!
//! Nothing here reads or writes a log -- that keeps the cog body free of the
//! log crate, which no cog may link.
//!
//! Times are absolute simulated nanoseconds since the Unix epoch. The
//! deterministic runner jumps its clock to each logged message's transmit time,
//! so the table below is the run's schedule; nothing rebases it.

use clockwork_rs::{Duration, SyncTime};

use brenn_reachy__cogs__proof__probe_msgs_clk_rs::ProbeCmdWire;
use brenn_reachy__motion__joints_clk_rs::JointFlags;

/// The channel the system's `LogReaderPolicy` names as its source. The policy
/// carries no `source_name`, so the source is the channel's own name, and this
/// string has to equal it exactly or the runner publishes nothing.
pub const CMD_CHANNEL: &str = "ProbeCmds";

/// The channel the probe's datagrams are logged on.
pub const PACKET_CHANNEL: &str = "ProbePackets";

/// The servo the probe writes: the body's yaw, which is the field the cog sets
/// by name. Which one it is does not matter here, only that a masked write
/// names exactly one.
pub const PROBE_MASK: JointFlags = JointFlags::BODY_YAW;

/// The scenario epoch: an arbitrary round Unix time, far enough from zero that
/// a dropped or defaulted timestamp reads as obviously wrong rather than as a
/// plausible small number.
pub const T0_NS: i64 = 1_700_000_000_000_000_000;

/// How far apart the commands are. Nothing in the probe is time-triggered, so
/// this only has to be a spacing the runner can order.
pub const STEP_NS: i64 = 100_000_000;

/// How long the run continues past the last command, so that a late last
/// datagram is still captured.
pub const TAIL_NS: i64 = 2 * STEP_NS;

/// The commanded positions, one per step, in radians. Distinct, non-round, and
/// including a negative and a zero: a datagram that lost a field, sign-extended
/// wrongly, or was silently replayed from the previous step reads differently
/// from every other one here.
pub const POSITIONS: [f64; 5] = [0.125, -0.375, 0.0, 1.234_567_890_123, -2.5];

/// How long each commanded position is nominally held. Nothing acts on it; it
/// is the `Duration` field, carried so that the type crosses the log.
pub const HOLD_NS: i64 = 40_000_000;

/// The probe's declared `execution_duration`, which the deterministic runner
/// models as the gap between an execution starting and its outputs being
/// published. A datagram therefore lands this far after the command that caused
/// it, and it is not jitter: the value is in `probe.clk` and the run is exact.
///
/// The motion system states its own, for its own modules. Two systems declaring
/// the same millisecond is not one fact: either `.clk` may change it, and a
/// constant shared between them would move the other system's assertions.
pub const EXECUTION_DURATION_NS: i64 = 1_000_000;

/// When the command at `index` is published, and therefore the simulated time
/// the probe sees it at.
#[must_use]
pub fn command_time_ns(index: usize) -> i64 {
    T0_NS + (index as i64) * STEP_NS
}

/// When the datagram caused by the command at `index` is published.
#[must_use]
pub fn packet_time_ns(index: usize) -> i64 {
    command_time_ns(index) + EXECUTION_DURATION_NS
}

/// The simulated time the run ends at.
#[must_use]
pub fn end_time_ns() -> i64 {
    command_time_ns(POSITIONS.len() - 1) + TAIL_NS
}

/// The command the scenario publishes at `index`.
#[must_use]
pub fn command(index: usize) -> ProbeCmdWire {
    let mut cmd = ProbeCmdWire::default();
    cmd.set_at(SyncTime::from_nanos(command_time_ns(index)));
    cmd.set_hold(Duration::from_nanos(HOLD_NS));
    cmd.set_position(POSITIONS[index]);
    cmd
}
