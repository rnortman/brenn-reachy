//! S1, raise-hold-stow: the scenario that says the loop closes.
//!
//! One session, no faults, nothing in the way. The machine is energised, a
//! schedule sends it to the upright posture, holds it there, sends it back to
//! stow, and the session ends. What that exercises is every part of the control
//! loop at once: the driver's heartbeat clocking two cogs, the decision tick
//! arming off a measured pose and hosting the motion library, the goal stream
//! reaching the gate two cycles ahead of the instant it names, the plant
//! tracking it, and the estimator reading the result back.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use scenario::author::Step;
use scenario::{STOW_DURATION_NS, UP_DURATION_NS, cycle_at, cycles_for};

use brenn_reachy__cogs__msgs_clk_rs::Posture;

/// The cycle the session is engaged and the machine energised at.
///
/// Not the epoch itself: the simulated driver's first cycle is one period in,
/// and a schedule published at the epoch is in the decision tick's view by then
/// either way. Starting at the epoch keeps the log's first instant and the
/// scenario's first instant the same number.
pub const ENGAGE_CYCLE: i64 = 0;

/// How long the upright step lasts, in cycles: the move plus room to arrive and
/// hold. The hold is the point -- a goal stream that stopped when the machine
/// arrived would trip the driver's dead-man, and this step is long enough that
/// it would.
pub const UP_CYCLES: i64 = 100;

/// How long the stow step lasts, in cycles: the longer move plus the same room.
pub const STOW_CYCLES: i64 = 150;

/// The cycle the upright step begins.
pub const UP_START_CYCLE: i64 = ENGAGE_CYCLE;

/// The cycle the stow step begins.
pub const STOW_START_CYCLE: i64 = UP_START_CYCLE + UP_CYCLES;

/// The cycle the session disengages and the machine is de-energised at.
pub const DISENGAGE_CYCLE: i64 = STOW_START_CYCLE + STOW_CYCLES;

/// How long the run continues past the disengagement, in cycles.
///
/// Long enough for the driver's dead-man to have fired if the goal stream had
/// merely gone quiet -- it does go quiet here, but the machine has been
/// de-energised first, so nothing latches and the scenario asserts no event at
/// all. A shorter tail would leave that assertion untested.
pub const TAIL_CYCLES: i64 = 20;

/// The last cycle of the run.
pub const END_CYCLE: i64 = DISENGAGE_CYCLE + TAIL_CYCLES;

/// The simulated time the run ends at.
#[must_use]
pub fn end_time_ns() -> i64 {
    cycle_at(END_CYCLE)
}

/// The two steps of the session's schedule.
#[must_use]
pub fn steps() -> [Step; 2] {
    [
        Step {
            start_ns: cycle_at(UP_START_CYCLE),
            end_ns: cycle_at(STOW_START_CYCLE),
            posture: Some(Posture::UP),
        },
        Step {
            start_ns: cycle_at(STOW_START_CYCLE),
            end_ns: cycle_at(DISENGAGE_CYCLE),
            posture: Some(Posture::STOW),
        },
    ]
}

/// The epoch of the schedule published at engagement.
pub const ENGAGED_EPOCH: u32 = 1;

/// The epoch of the schedule published at disengagement.
pub const DISENGAGED_EPOCH: u32 = 2;

/// How many cycles a move to the upright posture is given, rounded up.
#[must_use]
pub fn up_cycles() -> i64 {
    cycles_for(UP_DURATION_NS)
}

/// How many cycles a move to stow is given, rounded up.
#[must_use]
pub fn stow_cycles() -> i64 {
    cycles_for(STOW_DURATION_NS)
}
