//! S9, a servo that is not there: the scenario that says a machine the survey
//! cannot establish is never commanded at all.
//!
//! One crank is off the bus from before the run begins -- dead, unplugged, or
//! never fitted -- and nothing else in the run is unusual. The session does what
//! it does first in every run: it surveys the machine, pinging each servo in turn
//! over the aux path. Eight answer and one does not, so the presence sweep ends
//! with a servo it could not find, the survey refuses the machine, and the
//! session parks.
//!
//! What that exercises is the one arm of the phase machine no other scenario
//! reaches: a survey that *fails*. Nothing was torqued to get there and nothing
//! is torqued afterwards -- the presence sweep is nine pings and reads no
//! register, let alone writes one -- so there is no maneuver to run and nothing
//! to make safe. The park is the whole response, and only an operator restarting
//! the process clears it. A script sent afterwards is refused as parked, which
//! is what a machine nobody has been to says to whoever is asking.
//!
//! The absence is on the unicast path and nowhere else. The modelled driver's
//! proprioception is a read of the plant rather than a round trip over the wire,
//! so the sample stream stays whole through all of it and the estimator keeps
//! reporting a pose: what this run is about is the survey, which is unicast and
//! does hear the silence. A machine whose feedback also goes away is S4's run.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use reachy_motion::joints::{JointRef, ROW_COUNT, flags};
use scenario::author::Step;
use scenario::{SESSION_WAKE_FLOOR_NS, cycle_at, cycles_for, hold_timeout_cycles};

// Where a run begins, and the cycle a script may first be taken on. The armed
// cycle every healthy run derives its steps from is deliberately absent: this
// machine is never armed.
pub use scenario::{START_CYCLE, script_cycle as script_sent_cycle};

/// The script's number. It is refused, so what the number buys is that the
/// refusal names the request this run made.
pub const SCRIPT_ID: u32 = 9;

/// How many cycles a transaction of the survey is allowed.
///
/// A datagram published inside one cycle is drained by the next, run there, and
/// answered inside it, and the answer wakes the session where it publishes the
/// next -- so a transaction costs a cycle when nothing is in its way. Three, for
/// the same reason [`scenario::commission_allowance_cycles`] allows three: the
/// aux slot's own health rotation takes its turn, and this is an allowance
/// rather than an expectation. What the run asserts exactly is the *count* of
/// datagrams the survey spent, which is the number that says where it stopped.
pub const CYCLES_PER_TRANSACTION: i64 = 3;

/// The servo that is not there: one of the six cranks that carry the head.
///
/// A crank rather than an antenna, so the machine the survey refuses is one that
/// could not have held its head up at all -- and the refusal is the same
/// whichever row it is, because a survey establishes the whole machine or none
/// of it.
#[must_use]
pub fn absent_joint() -> JointRef {
    JointRef::Leg3
}

/// The missing servo, as the set an injection names.
#[must_use]
pub fn absent_rows() -> JointFlags {
    flags::bit(absent_joint())
}

/// The cycle the machine is parked by at the latest.
///
/// The session's first wake, and then the presence sweep: one ping per row of
/// the bus, each allowed [`CYCLES_PER_TRANSACTION`]. An outer bound and not an
/// expectation -- which cycle the first wake lands on depends on whether the
/// wake floor or the driver's first health report gets there first -- and what
/// the checker asserts against the run's own narration is that the park landed
/// inside it.
#[must_use]
pub fn parked_by_cycle() -> i64 {
    cycles_for(SESSION_WAKE_FLOOR_NS) + CYCLES_PER_TRANSACTION * ROW_COUNT as i64
}

/// The last cycle of the run.
///
/// Well past the refusal, and past the dead-man's window: the survey's last
/// datagram is the last thing the session says to the driver, so a run this long
/// is what makes "the gate raised nothing" an assertion that nothing was ever
/// energised rather than one about a run that ended too early to tell.
#[must_use]
pub fn end_cycle() -> i64 {
    script_sent_cycle() + 4 * hold_timeout_cycles()
}

/// The step the script asks for, which nothing ever runs.
///
/// A well-formed request, so the refusal the checker asserts is about the state
/// the machine is in and not about anything wrong with the asking.
#[must_use]
pub fn steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(script_sent_cycle()),
        end_ns: cycle_at(end_cycle()),
        posture: Some(PostureWire::UP),
    }]
}
