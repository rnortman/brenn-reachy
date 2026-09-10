//! S15, streamed content that outruns the antennas: the scenario that says such
//! content plays through the whole stack unsmoothed and costs the machine
//! nothing.
//!
//! The machine is raised and held there for the whole run, and one window plays
//! `probe/antenna-step-a` over it at full gain. That document is the stimulus by
//! construction: each of its three poses is reached in a single frame and then
//! held, so every step asks the antennas for well over a radian in one period --
//! past the bound the planner refuses its *own* moves past, and against a servo
//! generator that carries about a third of a radian in a period. The joints go
//! nowhere near their goal for the next several periods, and the run is about
//! what happens next, which is nothing.
//!
//! What is asserted: a step the planner would refuse for its own moves is
//! composed through the overlay and the mover and reaches the servos as it was
//! authored; the goal stream stays whole across the window's opening and
//! closing, under the content-row exemption from the planner's own per-cycle
//! bound; nothing was raised, nothing let go, no torque touched; and each held
//! pose was reached, measured off the lagged plant, inside the antennas' own
//! travel for the arc. An overlay that ramped the step, a mover that refused
//! it, a composition that leaked into head rows, or a plant that failed to
//! arrive all fail here.
//!
//! What this run is *not*: an exercise of the tracking detector. The simulated
//! shaft is the tick's own plant model stepped on the same inputs, so an
//! unobstructed row and the tick's prediction of it agree cycle for cycle and
//! the residual is nil however far the goal jumps -- the detector's window does
//! not open on content in this harness, only on a jammed row. The detector's
//! rule is exercised by the pace cases in
//! `crates/reachy-motion/src/tick.rs`, on a synthetic plant that can be made to
//! deviate, and its silence on the real machine is the hardware record in
//! `docs/servo-tuning.md`.
//!
//! Nothing about the head: the clip drives the antennas alone and the base
//! holds the raised posture bit for bit, so `head_obstructed` has nothing to
//! judge here.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. Every instant is a cycle count from the epoch.

use scenario::author::{Overlay, Step};
use scenario::{ARRIVAL_SETTLE_CYCLES, LAG_K, cycle_at, run_end_cycle};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use reachy_motion::ANTENNA_OUTBOARD;
use reachy_motion::disarm::STOW_ANTENNAS;
use reachy_motion::joints::{JointGroup, JointTargets};
use reachy_motion::postures::{NEUTRAL_ANTENNAS, neutral_targets};

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The script's number.
pub const SCRIPT_ID: u32 = 15;

/// The name of the motion the window plays.
///
/// A name and not a number: the numbering is generated and positional, so an
/// asset inserted ahead of this one renumbers it, and [`scenario::motion_id`]
/// is what reads the number out of the sidecar the emitter writes.
pub const MOTION_STEP: &str = "probe/antenna-step-a";

/// How much of the motion's delta the window asks for: the whole of it.
///
/// A scaled delta would scale the step this scenario is about, and the step is
/// the stimulus.
pub const GAIN: f64 = 1.0;

/// How fast the motion is played: at the rate its clips were authored at, so
/// the timeline below is the document's own.
pub const SPEED: f64 = 1.0;

/// How many frames the probe spends on the base before its first step.
///
/// One: the document opens on the pose the base already holds, so the window
/// opening composes nothing and the first thing it does is step.
pub const BASE_FRAMES: i64 = 1;

/// How long the probe holds each pose it steps to, in cycles.
///
/// 6.5 s at the 50 Hz clips are sampled and played at, and a cycle of this
/// system is one frame. Far past the travel each step takes, which is what
/// makes the arrival assertions below readings of a machine standing still.
pub const HOLD_FRAMES: i64 = 325;

/// How long the whole document runs, in cycles.
///
/// The three constants here are the motion's timeline as its document states
/// it; a document edited without them is a scenario asserting about a motion
/// that no longer exists, which the assertions bracketing each step and each
/// hold are what catches.
pub const CLIP_FRAMES: i64 = BASE_FRAMES + 3 * HOLD_FRAMES;

/// How long after its step begins the window opens, in cycles.
///
/// Past the travel the move to the upright posture takes at the servos' own
/// profile, so the base the content is composed over is a machine standing
/// still: what the goal stream then carries over the window is the clip's own
/// steps and nothing else.
#[must_use]
pub fn window_after_step_cycles() -> i64 {
    scenario::posture_step_cycles()
}

/// How long the upright step lasts, in cycles: the travel, the whole document,
/// and room to stand still on the pose it ends on.
///
/// The document ends on the base's own antenna pose, so there is no
/// contribution left for the close to hand back.
#[must_use]
pub fn up_cycles() -> i64 {
    window_after_step_cycles() + CLIP_FRAMES + ARRIVAL_SETTLE_CYCLES
}

/// The cycle the window opens on.
#[must_use]
pub fn window_open_cycle() -> i64 {
    up_start_cycle() + window_after_step_cycles()
}

/// The cycle the window closes on, which is the first cycle it no longer
/// covers.
#[must_use]
pub fn window_close_cycle() -> i64 {
    window_open_cycle() + CLIP_FRAMES
}

/// The cycle the schedule runs out on, which is what ends the session.
#[must_use]
pub fn disengage_cycle() -> i64 {
    up_start_cycle() + up_cycles()
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(disengage_cycle())
}

/// The rows this run is about: the antenna pair, which is the only channel the
/// document drives.
#[must_use]
pub fn antenna_rows() -> JointFlags {
    JointGroup::Antennas.joints()
}

/// The three poses the document steps to, in order, with the name each is read
/// under.
///
/// The tree's own constants rather than the document's numbers: a probe carries
/// deltas over the base's antenna lean, and these are what those deltas are
/// differences of. A document authored against a fold the tree has since moved
/// fails the arrival assertions rather than passing against its own stale copy.
#[must_use]
pub fn held_poses() -> [(&'static str, [f64; 2]); 3] {
    [
        ("outboard", ANTENNA_OUTBOARD),
        ("folded", STOW_ANTENNAS),
        ("raised again", NEUTRAL_ANTENNAS),
    ]
}

/// A held pose as the whole commanded configuration it makes.
///
/// The base under the window is the upright posture and the document drives the
/// antennas alone, so the head is where the raise put it and only the pair
/// moves.
#[must_use]
pub fn held_targets(antennas: [f64; 2]) -> JointTargets {
    JointTargets {
        antennas,
        ..neutral_targets()
    }
}

/// The cycle each of the document's three steps reaches the goal stream on.
///
/// The window's own frame clock: the frame played on a cycle is that cycle's
/// offset into the window, at the speed of one.
#[must_use]
pub fn step_cycles() -> [i64; 3] {
    [
        window_open_cycle() + BASE_FRAMES,
        window_open_cycle() + BASE_FRAMES + HOLD_FRAMES,
        window_open_cycle() + BASE_FRAMES + 2 * HOLD_FRAMES,
    ]
}

/// The first cycle the tick judges a reading taken after `step` reached the
/// goal stream.
///
/// A goal decided on a cycle is due `LAG_K` cycles later, and the servo's own
/// response delay passes before a reading shows it answering. So this is the
/// earliest cycle on which the antennas could have begun to close the step at
/// all -- which is where a step of well over a radian leaves them furthest from
/// their goal.
#[must_use]
pub fn judged_after(step: i64) -> i64 {
    step + LAG_K + scenario::response_delay_cycles()
}

/// The cycle the antennas are expected to be standing on the pose `step` sent
/// them to.
///
/// The travel the plant needs for the step's own arc, from the cycle the goal
/// is due, plus room to have been standing still for a moment. Nothing else: a
/// machine that needed longer than its own profile is one whose joints did not
/// follow.
#[must_use]
pub fn arrived_after(step: i64, arc_rad: f64) -> i64 {
    step + LAG_K + scenario::travel_cycles(JointGroup::Antennas, arc_rad) + ARRIVAL_SETTLE_CYCLES
}

/// How far the antennas travel on each of the document's three steps, radians.
///
/// The widest of the pair, since both are asserted arrived and the later one
/// decides the instant. Read off the poses rather than stated, so a constant
/// that moves moves this with it.
#[must_use]
pub fn step_arcs() -> [f64; 3] {
    let poses = held_poses();
    let mut from = NEUTRAL_ANTENNAS;
    let mut arcs = [0.0; 3];
    for (arc, (_, to)) in arcs.iter_mut().zip(poses) {
        *arc = (to[0] - from[0]).abs().max((to[1] - from[1]).abs());
        from = to;
    }
    arcs
}

/// The one step of the script: the upright posture, held for the whole run.
#[must_use]
pub fn steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(up_start_cycle()),
        end_ns: cycle_at(disengage_cycle()),
        posture: Some(PostureWire::UP),
    }]
}

/// The one window of the script: the probe, played over the upright step from
/// the cycle the machine has arrived and is standing still.
#[must_use]
pub fn overlays() -> [Overlay; 1] {
    [Overlay {
        motion_id: scenario::motion_id(MOTION_STEP),
        start_ns: cycle_at(window_open_cycle()),
        end_ns: cycle_at(window_close_cycle()),
        gain: GAIN,
        speed: SPEED,
    }]
}

#[cfg(test)]
mod tests {
    use reachy_motion::tick::default_motion_config;

    use super::{HOLD_FRAMES, arrived_after, judged_after, step_arcs, step_cycles};

    /// Every step of the document asks the antennas for more in one period than
    /// the planner allows its own moves, so the content really does outrun the
    /// servos on each of them.
    ///
    /// Arithmetic over the tree's own constants, and it runs whether or not the
    /// simulated run produced a readable log -- which is exactly the state a
    /// document edited down to gentle steps would leave the tree in.
    #[test]
    fn every_step_the_document_makes_outruns_the_planners_own_bound() {
        let bound = default_motion_config().max_step.antennas;
        for arc in step_arcs() {
            assert!(
                arc > bound,
                "a step of {arc} rad is inside the {bound} rad the planner bounds its own moves \
                 to, so it is content this machine can follow"
            );
        }
    }

    /// Each step is answered and stood upon well inside the hold that follows
    /// it, so the next step lands on a machine standing still.
    #[test]
    fn each_hold_outlasts_the_travel_the_step_before_it_asks_for() {
        for (step, arc) in step_cycles().into_iter().zip(step_arcs()) {
            let arrived = arrived_after(step, arc);
            assert!(
                arrived < step + HOLD_FRAMES,
                "a step at cycle {step} is stood upon at {arrived}, past the {HOLD_FRAMES} cycles \
                 the document holds it for"
            );
            assert!(
                judged_after(step) < arrived,
                "the cycle the window is read on is not inside the step it is read for"
            );
        }
    }
}
