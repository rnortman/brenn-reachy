//! S11, an antenna pair let go of: the scenario that says a fault can be
//! answered without ending the session.
//!
//! S1's run, with one antenna made to complain about itself while the machine
//! holds its working posture. What that exercises is the one rung of the ladder
//! that is scoped to a group rather than to the machine: the driver's rotating
//! read takes the servo's hardware-error byte off the bus, the session
//! classifies it as the antennas' condition, and the answer is the pair going
//! limp -- two verified torque-off writes the session issues itself, one per
//! wake, over the same aux path a sequence's asks ride.
//!
//! And then the session carries on. Nothing is stowed, nothing is parked, no
//! maneuver is opened: the head keeps its presence, runs the rest of the
//! schedule it was given, folds itself at the end of it and is let go of at
//! rest. That is the doctrine's own reading of a non-load-bearing pair -- an
//! antenna pair going limp while the head keeps its presence is a fault
//! answered, not an exception to the rule that a response de-torques what it
//! covers.
//!
//! The limp pair is then commanded to fold with everything else, because the
//! schedule's last step is the stow the session ends at. They cannot: nothing
//! holds them any more. So the decision tick's own tracking evidence finds two
//! joints that have stopped closing on their goals, takes them out of service
//! and carries the move on with what remains -- which is the consequence this
//! run is here to show is survivable. The head reaches the fold; the antennas
//! stay where they were when they let go; and the release the session ends at
//! reports the fold it could not measure.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use reachy_motion::joints::{JointGroup, JointRef, flags};
use scenario::author::Step;
use scenario::{answered_within, cycle_at, cycles_for, run_end_cycle, up_clocks};

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The script's number.
pub const SCRIPT_ID: u32 = 11;

// The bits the servo is made to hold: the same byte S8's head servo holds, so
// the two runs differ in which servo said it and in nothing else -- which is
// what makes the pair of them say that the response follows the group.
pub use scenario::ACTED_ON_ERROR_BITS as ERROR_BITS;

/// How many cycles after the upright move has run its course the servo starts
/// complaining.
///
/// After the move and not during it, which is what makes the run's arithmetic
/// hold for every phase the driver's rotation can be in when the byte is
/// written. A pair let go of while it is still travelling stalls away from goals
/// that keep moving, and the tick's tracking evidence then raises about the
/// antennas inside the upright step -- on a cycle decided by where the rotation
/// happened to be, which is nothing this scenario states. Let go of after the
/// antennas have arrived, they hold the angle they reached, follow a goal that no
/// longer moves perfectly, and raise nothing until the fold is commanded.
///
/// The margin is a settle: the move is given its whole budget -- the longest
/// clock any group of it runs on, which is an antenna's, since the pair is
/// parted at its crossing and arrives after the head does -- and the cycle it
/// arrives on is the last of that budget rather than a cycle this file counts.
pub const SETTLE_CYCLES: i64 = 5;

/// How long the upright step lasts, in cycles.
///
/// Long enough for the machine to arrive, for the rotation to have taken a whole
/// lap of the bus after the byte was written, and for the pair to have been let
/// go of well inside it -- and then to hold: a goal stream that stopped when the
/// machine arrived would trip the driver's dead-man, and this step is long enough
/// that it would. The checker asserts the fit rather than trusting this number.
pub const UP_CYCLES: i64 = 200;

/// How long the stow step lasts, in cycles: the move plus room to arrive and
/// hold. S1's number, because the fold is S1's fold -- what differs is that two
/// of the nine joints cannot join it.
pub const STOW_CYCLES: i64 = 150;

/// The servo that complains: the right antenna.
///
/// One of the pair, and the response is the pair: an antenna still holding
/// beside a dead one is a machine half-presenting, so the maneuver is scoped to
/// the group rather than to the servo whose byte was read. Which one it is is
/// what the checker joins the report to.
#[must_use]
pub fn faulted_joint() -> JointRef {
    JointRef::AntennaRight
}

/// The complaining servo, as the set an injection names.
#[must_use]
pub fn faulted_rows() -> JointFlags {
    flags::bit(faulted_joint())
}

/// The rows the answer lets go of: the antenna pair.
#[must_use]
pub fn degraded_rows() -> JointFlags {
    JointGroup::Antennas.joints()
}

/// The cycle the servo's error byte is written.
#[must_use]
pub fn fault_cycle() -> i64 {
    up_start_cycle() + up_clocks().cycles() + SETTLE_CYCLES
}

/// The cycle the session must have answered the condition by.
///
/// The read that carries it, plus the wake the report causes. An outer bound and
/// not an expectation: which cycle the rotation reaches the faulted row on is a
/// fact about the run, so what the checker asserts against the narration is that
/// the answer landed inside this.
#[must_use]
pub fn answered_by_cycle() -> i64 {
    answered_within(fault_cycle())
}

/// The cycle the pair must have let go by.
///
/// The answer, plus a wake per verified write and one for the report that says
/// the set is empty. One write goes out per wake because the bus is unicast and
/// the driver refuses a second outstanding transaction, so the drain costs as
/// many wakes as there are rows in the group.
#[must_use]
pub fn released_by_cycle() -> i64 {
    answered_by_cycle()
        + (flags::iter(degraded_rows()).count() as i64 + 1) * cycles_for(WAKE_ALLOWANCE_NS)
}

/// How long one turn of the drain is allowed, nanoseconds.
///
/// The session's wake floor, and the allowance is generous on purpose: a wake
/// comes sooner than the floor whenever a message arrives, and the outcome of
/// each write is itself a message. What the number bounds is a drain that
/// stalled, not the cycle each write went out on.
const WAKE_ALLOWANCE_NS: i64 = scenario::SESSION_WAKE_FLOOR_NS;

/// The cycle the stow step begins.
#[must_use]
pub fn stow_start_cycle() -> i64 {
    up_start_cycle() + UP_CYCLES
}

/// The cycle the schedule runs out on, which is what ends the session.
#[must_use]
pub fn disengage_cycle() -> i64 {
    stow_start_cycle() + STOW_CYCLES
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(disengage_cycle())
}

/// The two steps of the script: the working posture, and the fold to end at.
#[must_use]
pub fn steps() -> [Step; 2] {
    [
        Step {
            start_ns: cycle_at(up_start_cycle()),
            end_ns: cycle_at(stow_start_cycle()),
            posture: Some(PostureWire::UP),
        },
        Step {
            start_ns: cycle_at(stow_start_cycle()),
            end_ns: cycle_at(disengage_cycle()),
            posture: Some(PostureWire::STOW),
        },
    ]
}
