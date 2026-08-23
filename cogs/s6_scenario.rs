//! S6, a composed motion over a posture: the scenario that says the overlay
//! layer plays.
//!
//! S1's healthy lifecycle with one thing added: the script asks for a motion to
//! be played over the posture it commands, and the window it asks for closes
//! while that motion is still playing. What that exercises is the layer nothing
//! else in the suite reaches -- the configured clip library established by the
//! decision tick, the composition of a played delta onto a base the mover is
//! sampling itself, the walk over a motion of two segments with a hold between
//! them, and the re-anchor that keeps the commanded stream continuous when the
//! window closes over a contribution that had not decayed.
//!
//! The motion is `bench/tour`: the antennas raised over half a second, a beat
//! holding them there, then one body sway. Three properties of the layer are
//! observable in the goal stream because of how it is put together -- the rise,
//! the hold at a delta the window's gain has scaled, and the seam where the
//! antennas hand over to the yaw -- and the window closes part way through the
//! sway, where the contribution the machine is carrying is at its largest.
//!
//! The session then ends the way S1's does, comes back to rest, and takes a
//! second script: the machine that let go is the machine that engages again,
//! which is the doctrine's own statement about what a session is.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. Every instant is a cycle count from the epoch.

use scenario::author::{Overlay, Step};
use scenario::{TAIL_CYCLES, UP_DURATION_NS, cycle_at, cycles_for, run_end_cycle};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The first script's number.
pub const SCRIPT_ID: u32 = 6;

/// The second script's number: the one the machine takes after it has let go of
/// the first.
pub const SECOND_SCRIPT_ID: u32 = 7;

/// The name of the motion the window plays.
///
/// A name and not a number: the numbering is generated and positional, so an
/// asset inserted ahead of this one renumbers it, and [`scenario::motion_id`]
/// is what reads the number out of the sidecar the emitter writes.
pub const MOTION_TOUR: &str = "bench/tour";

/// How much of the motion's delta the window asks for.
///
/// A half rather than the whole of it, so what the goal stream carries is a
/// scaled delta: a layer that ignored the gain would compose the clip's own
/// numbers and the hold below would be twice what this scenario asks for.
pub const GAIN: f64 = 0.5;

/// How fast the motion is played: at the rate its clips were authored at, so
/// the timeline below is the documents' own.
pub const SPEED: f64 = 1.0;

/// How long the first segment plays, in cycles.
///
/// `bench/tip` is 25 frames at the 50 Hz clips are sampled and played at, and a
/// cycle of this system is one frame. The three constants here are the motion's
/// timeline as its documents state it; a document edited without them is a
/// scenario asserting about a motion that no longer exists, which the assertions
/// bracketing each phase are what catches.
pub const TIP_CYCLES: i64 = 25;

/// How long the hold between the segments lasts, in cycles: the sequence's
/// 400 ms gap.
pub const GAP_CYCLES: i64 = 20;

/// How long the second segment plays, in cycles: `bench/sway`'s 40 frames.
pub const SWAY_CYCLES: i64 = 40;

/// How far into the sway the window closes, in cycles.
///
/// Two cycles past the yaw's own peak, which is where the contribution the
/// machine is carrying is largest: a window closing where the motion had already
/// come back to nothing would make the re-anchor's continuity vacuous.
pub const CLOSE_AFTER_SWAY_CYCLES: i64 = 12;

/// How long the window stays open, in cycles: the whole first segment, the whole
/// hold, and part of the second, which is what truncates the motion.
pub const WINDOW_CYCLES: i64 = TIP_CYCLES + GAP_CYCLES + CLOSE_AFTER_SWAY_CYCLES;

/// How long after its step begins the window opens, in cycles.
///
/// Past the move to the upright posture, so the base the motion is composed over
/// is a machine standing still: what the goal stream then carries over the
/// window is the layer's contribution and nothing else.
pub const WINDOW_AFTER_STEP_CYCLES: i64 = 60;

/// How long the upright step lasts, in cycles.
///
/// The move, the window, and then room for the contribution the close hands back
/// to decay and for the machine to be standing upright again before the step
/// ends.
pub const UP_CYCLES: i64 = 200;

/// How long the stow step lasts, in cycles: the longer move plus room to arrive
/// and hold.
pub const STOW_CYCLES: i64 = 150;

/// The delta the hold between the segments stands at, radians.
///
/// `bench/tip`'s last frame parts the antennas by 0.2 rad and the window asks
/// for half of it. Stated here because it is the one number in this scenario
/// that joins a document's contents to what the machine is commanded to do.
pub const HOLD_RAD: f64 = 0.1;

/// The cycle the window opens on.
#[must_use]
pub fn window_open_cycle() -> i64 {
    up_start_cycle() + WINDOW_AFTER_STEP_CYCLES
}

/// The cycle the window closes on, which is the first cycle it no longer covers.
#[must_use]
pub fn window_close_cycle() -> i64 {
    window_open_cycle() + WINDOW_CYCLES
}

/// The first cycle of the hold this scenario asserts about.
///
/// Inside the gap rather than at its edge: the seam either side of it is where a
/// frame's own arithmetic and a blend ramp are still moving, and what the hold
/// is about is the stretch where nothing is.
#[must_use]
pub fn hold_from_cycle() -> i64 {
    window_open_cycle() + TIP_CYCLES + 3
}

/// The last cycle of the hold this scenario asserts about.
#[must_use]
pub fn hold_through_cycle() -> i64 {
    window_open_cycle() + TIP_CYCLES + GAP_CYCLES - 3
}

/// The cycle the sway's own contribution is at its peak.
#[must_use]
pub fn sway_peak_cycle() -> i64 {
    window_open_cycle() + TIP_CYCLES + GAP_CYCLES + 10
}

/// The cycle the machine is upright and standing still on, before the window
/// opens.
#[must_use]
pub fn standing_cycle() -> i64 {
    window_open_cycle() - 2
}

/// The cycle the base has finished absorbing the truncated contribution by.
///
/// A whole configured posture clock past the close, which is the clock the
/// hand-back's own move is planned over.
#[must_use]
pub fn absorbed_cycle() -> i64 {
    window_close_cycle() + cycles_for(UP_DURATION_NS) + 5
}

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

/// The cycle the second script is sent on.
///
/// Past the whole allowance the orderly release is given, so the session it
/// reaches is one that has finished letting go and come back to rest. The run
/// ends a tail later, before the arming it begins could conclude: what this
/// scenario says about the second script is that it was taken.
#[must_use]
pub fn second_script_cycle() -> i64 {
    end_cycle() - TAIL_CYCLES
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(disengage_cycle())
}

/// The two steps of the first script.
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

/// The one window of the first script: the motion, played over the upright step
/// and cut off part way through.
#[must_use]
pub fn overlays() -> [Overlay; 1] {
    [Overlay {
        motion_id: scenario::motion_id(MOTION_TOUR),
        start_ns: cycle_at(window_open_cycle()),
        end_ns: cycle_at(window_close_cycle()),
        gain: GAIN,
        speed: SPEED,
    }]
}

/// The one step of the second script.
///
/// Whatever it asks for is beside the point -- the run ends before the machine
/// could carry it out -- so it asks for the posture the machine is already
/// standing in, over a step no assertion here is about.
#[must_use]
pub fn second_steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(second_script_cycle() + 5),
        end_ns: cycle_at(second_script_cycle() + 55),
        posture: Some(PostureWire::UP),
    }]
}
