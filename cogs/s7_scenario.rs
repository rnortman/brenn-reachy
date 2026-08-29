//! S7, conversational presence: the scenario that says a session holds while
//! the thing driving it changes its mind.
//!
//! A presence sender does not ask for a gesture. It says "be up, and I will
//! tell you again shortly", refreshes that on a cadence for as long as the
//! conversation lasts, and closes with a script that ends at the fold. What
//! this run is, then, is one engagement carrying three scripts: the raise, a
//! refresh that extends the horizon and plays a motion over it, and a closing
//! script that takes the machine down. The refreshes arrive while the machine
//! is under command, and each one wholly replaces the schedule the session was
//! running.
//!
//! The load-bearing negative is what the machine does *not* do between them.
//! Nothing disarms, nothing writes torque, the goal stream never stops, and the
//! driver's dead-man never opens a window -- a session answering each refresh
//! with a disarm and a fresh engagement would cycle torque on the head once per
//! sentence, and every one of those is asserted rather than assumed.
//!
//! The refresh's own step starts a few cycles after it is taken, so the run
//! also covers a replacement that commands nothing yet: the epoch is bumped
//! onto a schedule no step of which covers the instant, and the stream carries
//! on out of what the machine was already holding.
//!
//! One more thing arrives that is not a refresh: the number of the refresh
//! already accepted, sent again. That is what a duplicated delivery looks like
//! to a session, and it is refused `stale` and moves nothing -- no epoch, no
//! schedule, no phase.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. Every instant is a cycle count from the epoch.

use scenario::author::{Overlay, Step};
use scenario::{cycle_at, run_end_cycle};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The number of the script that opens the engagement: head up, bounded.
pub const HOLD_SCRIPT_ID: u32 = 7;

/// The number of the refresh that replaces it.
///
/// Strictly greater than the one before, which is the whole of the ordering
/// rule a replacement is screened against.
pub const REFRESH_SCRIPT_ID: u32 = 8;

/// The number of the closing script: up, and then the fold.
pub const CLOSING_SCRIPT_ID: u32 = 9;

/// How long the opening script asks for the machine to be up, in cycles.
///
/// Long enough that the refresh below lands well inside it: a refresh arriving
/// after the schedule had run out would be a fresh engagement wearing a
/// replacement's name, and every assertion about torque staying on would pass
/// for the wrong reason.
pub const HOLD_CYCLES: i64 = 160;

/// How many cycles into the engagement the refresh arrives.
///
/// Past the raise and past the settle, so the machine is standing upright and
/// still when the schedule under it is swapped. What the replacement then
/// commands is the posture the machine is already in, which is what makes the
/// goal stream across the epoch change an assertion about continuity rather
/// than about a move.
pub const REFRESH_AFTER: i64 = 100;

/// How long after the refresh arrives its own step begins, in cycles.
///
/// A replacement whose first step is still in the future: for this stretch the
/// new schedule commands nothing at all, and what the mover has is a bumped
/// epoch and no step covering the instant. The old schedule's steps are dead
/// with the old epoch, so a session that needed a covering step to keep the
/// stream going would gap it right here -- and `check::goal_stream` and
/// `check::no_events` are what say it did not. Short, because what is being
/// exercised is the uncovered instant and not a wait.
pub const REFRESH_STARTS_AFTER: i64 = 5;

/// How long the refresh asks for the machine to be up, in cycles.
///
/// Past the closing script below, so the closing one replaces a schedule that
/// still had a future: a refresh that had already run out would end the session
/// on its own.
pub const REFRESH_CYCLES: i64 = 320;

/// The name of the motion the refresh plays over its posture.
///
/// A name and not a number: the numbering is generated and positional, so an
/// asset inserted ahead of this one renumbers it, and [`scenario::motion_id`]
/// is what reads the number out of the sidecar the emitter writes.
pub const MOTION_TOUR: &str = "bench/tour";

/// How much of the motion's delta the window asks for.
pub const GAIN: f64 = 0.5;

/// How fast it is played: at the rate its clips were authored at, so the
/// timeline below is the documents' own.
pub const SPEED: f64 = 1.0;

/// How long the motion's first segment plays, in cycles: `bench/tip`'s 25
/// frames, one frame per cycle of this system.
pub const TIP_CYCLES: i64 = 25;

/// How long the hold between the motion's segments lasts, in cycles: the
/// sequence's 400 ms gap.
pub const GAP_CYCLES: i64 = 20;

/// The delta the hold between the segments stands at, radians.
///
/// `bench/tip`'s last frame parts the antennas by 0.2 rad and the window asks
/// for half of it. What it buys this scenario is evidence that the *replacement's*
/// own overlay window reached the mover: a session that swapped the steps and
/// dropped the windows would leave the antennas exactly where the posture puts
/// them.
pub const HOLD_RAD: f64 = 0.1;

/// How long after the refresh the window opens, in cycles.
///
/// Past the base's own move: the replacement re-plans the posture it commands
/// from wherever the machine is, and what this scenario measures inside the
/// window is the layer's contribution over a base standing still.
pub const WINDOW_AFTER_REFRESH_CYCLES: i64 = 70;

/// How long the window stays open, in cycles: the whole motion and room after
/// it, so the window closes over a layer that has already played out.
///
/// Truncating a window is S6's subject and not this one's; what closes here is
/// a window nothing is left in, so the goal stream across the close says
/// something about the replacement rather than about the re-anchor.
pub const WINDOW_CYCLES: i64 = 100;

/// How many cycles after the refresh its own number is sent again.
///
/// Past the window's close, so the duplicate arrives at a session doing nothing
/// in particular: what it is refused for is its number, and nothing else about
/// the instant it lands on is meant to be part of the answer.
pub const DUPLICATE_AFTER_REFRESH: i64 = 200;

/// How many cycles after the refresh the closing script arrives.
pub const CLOSING_AFTER_REFRESH: i64 = 260;

/// How long the closing script holds the machine up before folding it, in
/// cycles: the sender's own last beat.
pub const CLOSING_UP_CYCLES: i64 = 20;

/// How long the closing script's fold lasts, in cycles: the longer move plus
/// room to arrive and hold.
pub const CLOSING_STOW_CYCLES: i64 = 150;

/// The cycle the refresh is sent on.
#[must_use]
pub fn refresh_cycle() -> i64 {
    up_start_cycle() + REFRESH_AFTER
}

/// The cycle the machine is standing upright and still on, before the window
/// opens: the reference the layer's contribution below is measured against.
#[must_use]
pub fn standing_cycle() -> i64 {
    refresh_cycle() + WINDOW_AFTER_REFRESH_CYCLES - 5
}

/// The cycle the refresh's window opens on.
#[must_use]
pub fn window_open_cycle() -> i64 {
    refresh_cycle() + WINDOW_AFTER_REFRESH_CYCLES
}

/// The cycle the window closes on, which is the first cycle it no longer
/// covers.
#[must_use]
pub fn window_close_cycle() -> i64 {
    window_open_cycle() + WINDOW_CYCLES
}

/// The first cycle of the motion's hold this scenario asserts about.
///
/// Inside the gap rather than at its edge: the seam either side of it is where
/// a frame's own arithmetic and a blend ramp are still moving, and what the
/// hold is about is the stretch where nothing is.
#[must_use]
pub fn motion_hold_from_cycle() -> i64 {
    window_open_cycle() + TIP_CYCLES + 3
}

/// The last cycle of the motion's hold this scenario asserts about.
#[must_use]
pub fn motion_hold_through_cycle() -> i64 {
    window_open_cycle() + TIP_CYCLES + GAP_CYCLES - 3
}

/// The cycle the refresh's own number is sent again on.
#[must_use]
pub fn duplicate_cycle() -> i64 {
    refresh_cycle() + DUPLICATE_AFTER_REFRESH
}

/// The cycle the closing script is sent on.
#[must_use]
pub fn closing_cycle() -> i64 {
    refresh_cycle() + CLOSING_AFTER_REFRESH
}

/// The cycle the closing script's fold begins on.
#[must_use]
pub fn stow_start_cycle() -> i64 {
    closing_cycle() + CLOSING_UP_CYCLES
}

/// The cycle the schedule runs out on, which is what ends the session.
#[must_use]
pub fn disengage_cycle() -> i64 {
    stow_start_cycle() + CLOSING_STOW_CYCLES
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(disengage_cycle())
}

/// The one step of the opening script: up, and bounded.
#[must_use]
pub fn hold_steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(up_start_cycle()),
        end_ns: cycle_at(up_start_cycle() + HOLD_CYCLES),
        posture: Some(PostureWire::UP),
    }]
}

/// The one step of the refresh: up again, further out, and starting shortly
/// after it arrives.
///
/// The same posture the machine is already holding, because that is what a
/// refresh is: the sender saying the same thing again with a later deadline on
/// it. It begins a few cycles late so that the accept and the first instant the
/// new schedule covers are not the same one.
#[must_use]
pub fn refresh_steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(refresh_cycle() + REFRESH_STARTS_AFTER),
        end_ns: cycle_at(refresh_cycle() + REFRESH_CYCLES),
        posture: Some(PostureWire::UP),
    }]
}

/// The one window of the refresh: a motion played over the posture it extends.
#[must_use]
pub fn refresh_overlays() -> [Overlay; 1] {
    [Overlay {
        motion_id: scenario::motion_id(MOTION_TOUR),
        start_ns: cycle_at(window_open_cycle()),
        end_ns: cycle_at(window_close_cycle()),
        gain: GAIN,
        speed: SPEED,
    }]
}

/// The steps of the duplicate: the refresh's shape, anchored at the instant it
/// is sent again.
///
/// Never read. The ordering screen answers on the number the script carries,
/// before anything in it is looked at, so what these say is beside the point --
/// and they are stated as a well-formed schedule so that the refusal cannot be
/// the times being wrong.
#[must_use]
pub fn duplicate_steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(duplicate_cycle()),
        end_ns: cycle_at(duplicate_cycle() + REFRESH_CYCLES),
        posture: Some(PostureWire::UP),
    }]
}

/// The two steps of the closing script: one last beat up, and then the fold.
#[must_use]
pub fn closing_steps() -> [Step; 2] {
    [
        Step {
            start_ns: cycle_at(closing_cycle()),
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
