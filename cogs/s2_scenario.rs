//! S2, obstruction: the scenario that says a hand held against the head is
//! answered by carrying the machine down and letting it go.
//!
//! S1's run, with a hand held against the head cranks part way through the
//! raise. The tracking detector judges every joint against the trajectory its
//! own servo's generator is running, so cranks that stop closing on a goal still
//! moving away from them are a joint it screens on: a run opens when the
//! residual passes the threshold and `head_obstructed` is raised a window later.
//! What this run exercises is the whole of the doctrine's answer to that -- the
//! move abandoned and the tick holding, the rest-class stow the session commands
//! in its place, the machine measured at the fold and let go of at rest, and a
//! second script taken afterwards by the machine it let go of.
//!
//! The hand comes off inside the window the raise leaves: a released joint sets
//! off from rest, regains the progress minimum on the third step of its ramp,
//! and restarts the run that reopened after the raise, so nothing is raised a
//! second time and the stow runs end to end. Where the hand goes and when it
//! comes off are both derived from the stepped walk of the raise, because
//! both are conditions on the arithmetic: the cranks have to be far enough short
//! of their target for the generator to open the screen's own distance, their
//! generator has to have stopped by the time the fault lands -- otherwise a
//! release cannot pace it -- and the release has to land inside the reopened
//! window.
//!
//! The second half of the run is the ordinary life of a session, after one that
//! ended at the Minimum Risk Condition: rest is not park, so the next script is
//! taken and run. That is the half of the doctrine's park/rest split a refusal
//! cannot show -- S8's parked machine refuses a script, and this one runs it.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use brenn_reachy__motion__joints_clk_rs::JointFlags;
use scenario::author::Step;
use scenario::{
    STOW_BUDGET_NS, TAIL_CYCLES, cycle_at, cycles_for, engage_allowance_cycles,
    release_allowance_cycles, run_end_cycle,
};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The first script's number.
pub const SCRIPT_ID: u32 = 2;

/// The second script's number: the one a machine let go of at rest takes.
pub const SECOND_SCRIPT_ID: u32 = 12;

/// The rows the scenario jams: the hand the suite lays on the head, which is
/// the six cranks that carry it.
///
/// The suite's own set rather than this run's, because the run a hand that
/// never comes off produces has to be this run up to its first raise.
#[must_use]
pub fn jammed_rows() -> JointFlags {
    scenario::head_jam_rows()
}

/// Where the hand goes and what the detector makes of it, read off the stepped
/// walk of the raise.
///
/// The suite's own derivation over the cranks this run holds: the jam has to
/// leave the generator the screen's distance to travel, and the generator has
/// to have stopped by the time the fault lands, which is what makes this run's
/// release recoverable at all.
///
/// # Panics
///
/// As [`scenario::jam_on_the_raise`] does.
#[must_use]
pub fn hand() -> scenario::Jam {
    scenario::jam_on_the_raise(jammed_rows())
}

/// The cycle the cranks are jammed on.
#[must_use]
pub fn obstruct_cycle() -> i64 {
    up_start_cycle() + hand().jam
}

/// The cycle the detector raises `head_obstructed` on.
#[must_use]
pub fn raise_cycle() -> i64 {
    up_start_cycle() + hand().raise
}

/// The last cycle the hand may be off the cranks by for the stow to survive.
///
/// The run reopens on the tick after the raise and runs out `ticks` after it,
/// and what restarts it is the released joint regaining the progress minimum --
/// three steps of a from-rest ramp, which is what `pass_cycles` answers for that
/// distance. A release later than this is a second raise and a defeated stow.
#[must_use]
pub fn recovery_bound_cycle() -> i64 {
    let cfg = reachy_motion::tick::default_motion_config();
    raise_cycle() + i64::from(cfg.tracking.ticks)
        - scenario::pass_cycles(cfg.tracking.progress_min_rad)
        - 1
}

/// The cycle the jam is released on.
///
/// A cycle inside the bound, so the run this scenario states is not one sitting
/// on the last cycle that recovers.
#[must_use]
pub fn release_cycle() -> i64 {
    recovery_bound_cycle() - 1
}

/// The cycle the released cranks are asserted back inside the screen's own
/// distance of the goal they are being commanded to.
///
/// The catch-up the release owes, stated as what it costs: a joint setting off
/// from rest covers the screen's distance in `pass_cycles(threshold_rad)`, and
/// the goal it is closing on is descending under the stow meanwhile, which is
/// what the arrival settle past that figure allows for. An outer bound rather
/// than an expectation -- the machine is back inside the screen before it, and
/// what the number is for is naming a cycle by which a partial recovery is a
/// failure.
#[must_use]
pub fn caught_up_cycle() -> i64 {
    let cfg = reachy_motion::tick::default_motion_config();
    release_cycle()
        + scenario::pass_cycles(cfg.tracking.threshold_rad)
        + scenario::ARRIVAL_SETTLE_CYCLES
}

/// How long the upright step lasts, in cycles.
///
/// Nothing reaches its end -- the maneuver that answers the obstruction ends the
/// session first -- so it covers the raise, the hand, and the whole fold that
/// answers it, with room past that: a schedule running out is a second way to
/// end a session and this half of the run is about the first.
#[must_use]
pub fn up_cycles() -> i64 {
    stow_deadline_cycle() + TAIL_CYCLES - up_start_cycle()
}

/// The cycle the maneuver's one clock runs out on at the latest.
///
/// An outer bound: the machine reaches the fold well before it, and what the
/// number is for is placing the second script past every way the maneuver could
/// have ended.
#[must_use]
pub fn stow_deadline_cycle() -> i64 {
    raise_cycle() + cycles_for(STOW_BUDGET_NS)
}

/// The cycle the second script is sent on: a whole release allowance past every
/// way the first session could have ended, so what it meets is a machine resting
/// at the Minimum Risk Condition.
///
/// An outer bound and not an expectation -- which cycle the release concluded on
/// is a fact about the run, and the checker asserts the acceptance against the
/// phase change rather than against this.
#[must_use]
pub fn second_script_cycle() -> i64 {
    stow_deadline_cycle() + release_allowance_cycles() + TAIL_CYCLES
}

/// The cycle the second script's first step opens on: a whole arming allowance
/// after it, the suite's own convention, so what the step covers is a cycle this
/// module named.
#[must_use]
pub fn second_step_cycle() -> i64 {
    second_script_cycle() + engage_allowance_cycles()
}

/// How long the second script's raise lasts, in cycles: the travel, with
/// nothing in its way this time.
#[must_use]
pub fn second_up_cycles() -> i64 {
    scenario::posture_step_cycles()
}

/// How long the second script's fold lasts, in cycles: the same travel back.
#[must_use]
pub fn second_stow_cycles() -> i64 {
    scenario::posture_step_cycles()
}

/// The cycle the second script's fold opens on.
#[must_use]
pub fn second_stow_cycle() -> i64 {
    second_step_cycle() + second_up_cycles()
}

/// The cycle the second session's schedule runs out on, which is what ends it.
#[must_use]
pub fn second_disengage_cycle() -> i64 {
    second_stow_cycle() + second_stow_cycles()
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(second_disengage_cycle())
}

/// The script's one step: upright, for as long as the machine is left alone.
///
/// One step rather than S1's two: the fault takes the schedule away, so a second
/// posture would be commanding a machine the session had already begun carrying
/// down.
#[must_use]
pub fn steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(up_start_cycle()),
        end_ns: cycle_at(up_start_cycle() + up_cycles()),
        posture: Some(PostureWire::UP),
    }]
}

/// The second script's two steps: up, and back down to the fold.
///
/// A whole engagement rather than a token one. What this half of the run is
/// about is that a machine let go of at rest takes another script, so the second
/// engagement has to be a real one -- armed over the bus, streamed to, moved
/// somewhere, and released through the orderly path. It therefore asks for a
/// posture the machine is *not* standing in: a script whose only step named the
/// fold the first session left it in would produce a stream of frozen targets,
/// which a schedule that never reached the mover would produce too. It ends at
/// the fold, which is the posture the orderly release expects to measure.
#[must_use]
pub fn second_steps() -> [Step; 2] {
    [
        Step {
            start_ns: cycle_at(second_step_cycle()),
            end_ns: cycle_at(second_stow_cycle()),
            posture: Some(PostureWire::UP),
        },
        Step {
            start_ns: cycle_at(second_stow_cycle()),
            end_ns: cycle_at(second_disengage_cycle()),
            posture: Some(PostureWire::STOW),
        },
    ]
}
