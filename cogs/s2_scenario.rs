//! S2, obstruction: the scenario that says a jammed machine is reported once
//! per event and never once per cycle.
//!
//! S1's session, with a hand held against the head cranks part way through the
//! raise. What that exercises is the half of the loop S1 cannot reach: the
//! motion library's tracking detector seeing a joint that stops closing on its
//! goal, the report reaching the log as one message per raise rather than one
//! per tick, the tick abandoning the move and holding, and the keep-alive goal
//! stream carrying on through all of it so that the driver's dead-man never
//! fires. A machine that must not move further is not the same as a machine
//! that cannot be commanded, and this is the scenario that holds those apart.
//!
//! The obstruction is released well before the session ends, so the run also
//! says what happens afterwards: the servos close on the setpoint they were
//! holding, nothing further is reported, and the stow step still arrives.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use brenn_reachy__motion__joints_clk_rs::JointFlags;
use scenario::author::Step;
use scenario::{STOW_DURATION_NS, cycle_at, cycles_for};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;
use reachy_motion::joints::JointGroup;

/// The cycle the session is engaged and the machine energised at.
pub const ENGAGE_CYCLE: i64 = 0;

/// The cycle the cranks are jammed on.
///
/// Part way into the raise rather than at its start: what the detector measures
/// is a joint that stopped closing on a goal that is still moving away from it,
/// and a machine jammed before it was ever commanded would be measured against
/// a goal it was already standing on.
pub const OBSTRUCT_CYCLE: i64 = 10;

/// The cycle the jam is released on.
///
/// Long enough after the first report for the detector's window to run out
/// several times over, because the property under test is the *spacing* of the
/// reports and one raise cannot show a spacing.
pub const RELEASE_CYCLE: i64 = 90;

/// How long the upright step lasts, in cycles: long enough to hold the machine
/// through the whole jam and past its release.
pub const UP_CYCLES: i64 = 120;

/// How long the stow step lasts, in cycles: the move from wherever the jam left
/// the machine, plus room to arrive and hold.
pub const STOW_CYCLES: i64 = 150;

/// The cycle the upright step begins.
pub const UP_START_CYCLE: i64 = ENGAGE_CYCLE;

/// The cycle the stow step begins.
pub const STOW_START_CYCLE: i64 = UP_START_CYCLE + UP_CYCLES;

/// The cycle the session disengages and the machine is de-energised at.
pub const DISENGAGE_CYCLE: i64 = STOW_START_CYCLE + STOW_CYCLES;

/// How long the run continues past the disengagement, in cycles: long enough
/// for the dead-man to have fired if the machine had been left energised.
pub const TAIL_CYCLES: i64 = 20;

/// The last cycle of the run.
pub const END_CYCLE: i64 = DISENGAGE_CYCLE + TAIL_CYCLES;

/// The epoch of the schedule published at engagement.
pub const ENGAGED_EPOCH: u32 = 1;

/// The epoch of the schedule published at disengagement.
pub const DISENGAGED_EPOCH: u32 = 2;

/// The rows the scenario jams: the six cranks that carry the head.
///
/// The whole group rather than one of them. A single frozen crank leaves the
/// platform in a shape the linkage cannot take, which the estimator reports as
/// a pose it cannot solve -- a real presentation, and a different scenario's
/// subject. Freezing all six holds the head exactly where it stood, so what the
/// run is about is the tracking error and nothing else.
#[must_use]
pub fn jammed_rows() -> JointFlags {
    JointGroup::Legs.joints()
}

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
            posture: Some(PostureWire::UP),
        },
        Step {
            start_ns: cycle_at(STOW_START_CYCLE),
            end_ns: cycle_at(DISENGAGE_CYCLE),
            posture: Some(PostureWire::STOW),
        },
    ]
}

/// How many cycles a move to stow is given, rounded up.
#[must_use]
pub fn stow_cycles() -> i64 {
    cycles_for(STOW_DURATION_NS)
}
