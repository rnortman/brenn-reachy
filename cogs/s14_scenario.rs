//! S14, the grab that does not let go: the scenario that says a hand kept on
//! the head defeats the stow that answers it, and the machine is let go of
//! where it stands.
//!
//! S2's run with a hand that never comes off. The jam goes in the same place --
//! far enough short of the cranks' target for their generator to open the
//! screen's own distance, and settled by the time the fault lands -- so the
//! first `head_obstructed` is raised on the same cycle S2 raises it on. What
//! happens after it is the whole of this run.
//!
//! A non-latching raise leaves the tick holding, and a hold is not silence: the
//! keep-alive goes on publishing the last goal that went out, which is the goal
//! the content had reached when the joint was judged -- the screen's distance or
//! more past the held cranks. So the servo is commanded past the hand for one
//! more window while the machine asks whether the obstruction relents. The
//! session commands the rest-class stow in the execution that reads the raise,
//! and a min-jerk lead-in a window into the fold's own clock has covered a
//! thousandth of the distance -- a few thousandths of a radian on a crank,
//! under the progress minimum -- so the joint neither closes on its prediction
//! nor paces it: the run that reopened after the raise runs out and the
//! detector raises a second time, exactly a window after the first.
//!
//! That second raise names a stow, and a stow names no motor to mask, so it
//! defeats the running maneuver rather than re-commanding it: the record
//! concludes fallen through, the release goes out in the same session
//! execution, and the machine is de-torqued where it stands -- not folded. The
//! clock left in hand is what tells this ending from the one the budget would
//! have reached: a whole window less than the four seconds it opened with.
//!
//! What this run costs is stated in the doctrine and asserted here: the head is
//! *not* at the fold when it is let go. Yielding is the point -- a stow driven
//! into a hand grinds where it should give -- and the alternative is the servos
//! pushing for the rest of the budget before reaching the same ending.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use brenn_reachy__motion__joints_clk_rs::JointFlags;
use scenario::author::Step;
use scenario::{STOW_BUDGET_NS, TAIL_CYCLES, cycle_at, cycles_for, run_end_cycle};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The script's number.
pub const SCRIPT_ID: u32 = 14;

/// The rows the scenario jams: the hand the suite lays on the head, which is
/// the six cranks that carry it.
///
/// The suite's own set, so that this run's hand is the recovering run's hand
/// and its first raise is that run's first raise.
#[must_use]
pub fn jammed_rows() -> JointFlags {
    scenario::head_jam_rows()
}

/// Where the hand goes and what the detector makes of it, read off the stepped
/// walk of the raise.
///
/// S2's placement, from the suite's own derivation: the jam leaves the
/// generator the screen's distance to travel, and the generator has stopped by
/// the time the fault lands. The second condition is not this run's premise --
/// nothing here is released -- but placing the hand where S2 places it is what
/// makes this run S2's run and its first raise S2's raise.
///
/// # Panics
///
/// As [`scenario::jam_on_the_raise`] does.
#[must_use]
pub fn hand() -> scenario::Jam {
    scenario::jam_on_the_raise(jammed_rows())
}

/// The cycle the cranks are jammed on, and never let go of.
#[must_use]
pub fn obstruct_cycle() -> i64 {
    up_start_cycle() + hand().jam
}

/// The cycle the detector raises `head_obstructed` on.
#[must_use]
pub fn raise_cycle() -> i64 {
    up_start_cycle() + hand().raise
}

/// The cycle the detector raises `head_obstructed` a second time, out of the
/// hold.
///
/// A whole window after the first. The run reopens on the tick after the raise
/// with the joint held where it was and the goal the keep-alive republishes
/// standing still past it, so nothing closes and nothing paces, and the window
/// runs out on its own count. The stow's lead-in is what could have restarted
/// it and does not: a window into the fold's clock a min-jerk goal has moved by
/// a thousandth of its distance, which on the cranks the hand is on is a few
/// thousandths of a radian against a progress minimum of a hundredth.
#[must_use]
pub fn second_raise_cycle() -> i64 {
    let cfg = reachy_motion::tick::default_motion_config();
    raise_cycle() + i64::from(cfg.tracking.ticks)
}

/// How long the upright step lasts, in cycles.
///
/// Nothing reaches its end -- the defeated maneuver ends the session first --
/// so it covers both raises and the whole release that follows them, with room
/// past that: a schedule running out is a second way to end a session and this
/// run is about the first.
#[must_use]
pub fn up_cycles() -> i64 {
    stow_deadline_cycle() + TAIL_CYCLES - up_start_cycle()
}

/// The cycle the maneuver's one clock would have run out on had nothing defeated
/// it.
///
/// An outer bound and this run's foil: the second raise concludes the maneuver a
/// window after the first, which is nearly four seconds inside this, and the
/// clock the record reports left in hand is what says so.
#[must_use]
pub fn stow_deadline_cycle() -> i64 {
    raise_cycle() + cycles_for(STOW_BUDGET_NS)
}

/// The nanoseconds of the maneuver's clock still unspent when it concluded.
///
/// The clock is opened in the session execution that reads the first raise and
/// concluded in the one that reads the second, and the two raises are a window
/// apart on the grid -- so what the record reports is the budget less that
/// window. An expiry would report nothing left, and a defeat the stow's own
/// motion produced would report seconds less than this, which is the whole
/// reason this figure is asserted rather than "strictly positive".
#[must_use]
pub fn clock_left_ns() -> i64 {
    let cfg = reachy_motion::tick::default_motion_config();
    STOW_BUDGET_NS - i64::from(cfg.tracking.ticks) * scenario::PERIOD_NS
}

/// The last cycle of the run.
///
/// From the second raise, which is what ends the session: the release the
/// defeat commands, and then a tail long enough for the driver's dead-man to
/// have fired had the machine merely been left energised.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(second_raise_cycle())
}

/// The script's one step: upright, for as long as the machine is left alone.
///
/// One step, as S2 has: the fault takes the schedule away, so a second posture
/// would be commanding a machine the session had already begun carrying down --
/// and in this run had already let go of.
#[must_use]
pub fn steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(up_start_cycle()),
        end_ns: cycle_at(up_start_cycle() + up_cycles()),
        posture: Some(PostureWire::UP),
    }]
}
