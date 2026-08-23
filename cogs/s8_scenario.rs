//! S8, a head servo in trouble: the scenario that says the machine is carried
//! down and left for an operator.
//!
//! S1's run, with one head servo made to complain about itself part way into the
//! raise. What that exercises is the evidence path no other scenario has: the
//! driver's rotating read taking a servo's hardware-error byte off the bus, the
//! session classifying it through the same library function the decision tick
//! classifies its own health poll with, and the doctrine's answer to a head
//! servo that has stopped being trustworthy -- the head carried down under
//! control by the servos that still work, and then a park nothing clears but an
//! operator.
//!
//! It is also the run where a condition arriving *during* the maneuver is
//! answered. The head cranks are jammed while the fold is under way, so the
//! decision tick raises about a machine that has stopped closing on the stow it
//! was commanded to; the session re-ranks the maneuver it is already running
//! rather than beginning a second one, and re-commands the stow on what is left
//! of the one clock it was opened with. The hand comes off in time for the
//! machine to reach the fold, so the maneuver is measured there -- and the park
//! is what the *first* condition decided, not the jam.
//!
//! A second script arrives once all of that is over, and is refused: a parked
//! machine takes nothing until an operator has been, which is the whole of what
//! park means.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use reachy_motion::joints::{JointGroup, JointRef, ROW_COUNT, flags};
use scenario::author::Step;
use scenario::{STOW_BUDGET_NS, TAIL_CYCLES, answered_within, cycle_at, cycles_for};

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The script's number.
pub const SCRIPT_ID: u32 = 8;

/// The number of the script sent after the machine is parked.
pub const REFUSED_SCRIPT_ID: u32 = 108;

// The bits the servo is made to hold: the shared byte the library classifies as
// a fault rather than as an informational reading.
pub use scenario::ACTED_ON_ERROR_BITS as ERROR_BITS;

/// How many cycles into the raise the servo starts complaining.
///
/// Part way in rather than at its start, so the machine is under command when
/// the condition arrives: a stow is a schedule the decision tick carries out,
/// and what the ladder does with a response it cannot run is a different arm and
/// a different run. Whether the head has arrived by the time the rotation
/// carries the byte is not fixed -- the read lands anywhere inside a lap -- and
/// nothing here depends on which.
pub const FAULT_AFTER: i64 = 10;

/// How long the upright step lasts, in cycles.
///
/// Nothing reaches its end -- the maneuver that answers the servo ends the
/// session first -- and it is long enough that the run would still have been
/// under command had the rotation taken a whole lap of the bus to reach the
/// faulted row.
pub const UP_CYCLES: i64 = 200;

/// How many cycles after the fault must have been read the cranks are jammed.
///
/// Measured from the outer bound rather than from the read itself: which cycle
/// the rotation reaches the faulted row on is a fact about the run, and the jam
/// has to land inside the maneuver whichever cycle that was.
pub const JAM_AFTER: i64 = 10;

/// How long the jam lasts, in cycles.
///
/// Long enough for the tracking detector's window to run out more than once, so
/// the maneuver is re-commanded rather than merely re-ranked; and short enough
/// that the fold still fits in what is left of the maneuver's one clock.
pub const JAM_CYCLES: i64 = 40;

/// The rows the scenario jams: the six cranks that carry the head.
///
/// The whole group rather than one of them, S2's reasoning: a single frozen
/// crank leaves the platform in a shape the linkage cannot take, which the
/// estimator reports as a pose it cannot solve, and this run is about the
/// tracking error and nothing else.
#[must_use]
pub fn jammed_rows() -> JointFlags {
    JointGroup::Legs.joints()
}

/// The servo that complains: the body yaw.
///
/// A head servo, so the library classifies its trouble as the head's -- the yaw
/// and the six cranks all hold the head up -- and the one head servo whose own
/// row the jam does not also name, so the two conditions in this run stay
/// distinguishable in the narration.
#[must_use]
pub fn faulted_joint() -> JointRef {
    JointRef::BodyYaw
}

/// The faulted servo, as the set an injection names.
#[must_use]
pub fn faulted_rows() -> JointFlags {
    flags::bit(faulted_joint())
}

/// The cycle the servo's error byte is written.
#[must_use]
pub fn fault_cycle() -> i64 {
    up_start_cycle() + FAULT_AFTER
}

/// The cycle the session must have answered the condition by.
///
/// The read that carries it, plus the wake the report causes. An outer bound and
/// not an expectation: what the checker asserts against the run's own narration
/// is that the answer landed inside this, and everything the scenario places
/// afterwards is placed from here.
#[must_use]
pub fn answered_by_cycle() -> i64 {
    answered_within(fault_cycle())
}

/// The cycle the cranks are jammed on.
#[must_use]
pub fn jam_cycle() -> i64 {
    answered_by_cycle() + JAM_AFTER
}

/// The cycle the jam is released on.
#[must_use]
pub fn jam_release_cycle() -> i64 {
    jam_cycle() + JAM_CYCLES
}

/// The cycle the maneuver's one clock runs out on at the latest.
///
/// From the outer bound on when it opened, so this is an outer bound too: the
/// machine reaches the fold well before it, and what the number is for is
/// placing the script that finds the machine parked.
#[must_use]
pub fn stow_deadline_cycle() -> i64 {
    answered_by_cycle() + cycles_for(STOW_BUDGET_NS)
}

/// The cycle the second script is sent on.
///
/// Past every way the maneuver could have ended, so what it meets is a parked
/// machine whatever the fold cost.
#[must_use]
pub fn refused_script_cycle() -> i64 {
    stow_deadline_cycle() + TAIL_CYCLES
}

/// The last cycle of the run.
///
/// Long enough past the refusal for the driver's read-back to have confirmed the
/// release and for the dead-man to have fired if the release had not been
/// commanded at all.
#[must_use]
pub fn end_cycle() -> i64 {
    refused_script_cycle() + ROW_COUNT as i64 + TAIL_CYCLES
}

/// The script's one step: upright, for as long as the machine is left alone.
///
/// One step rather than S1's two: a second posture would be commanding a machine
/// the session has already begun carrying down, and a schedule that ran out
/// would end the session at rest, which is the opposite of what this run is
/// about.
#[must_use]
pub fn steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(up_start_cycle()),
        end_ns: cycle_at(up_start_cycle() + UP_CYCLES),
        posture: Some(PostureWire::UP),
    }]
}

/// The step the refused script asks for, which nothing ever runs.
#[must_use]
pub fn refused_steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(refused_script_cycle()),
        end_ns: cycle_at(end_cycle()),
        posture: Some(PostureWire::UP),
    }]
}
