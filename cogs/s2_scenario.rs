//! S2, obstruction: the scenario that says a jam is lag in the record and not a
//! fault.
//!
//! S1's run, with a hand held against the head cranks part way through the
//! raise. The tracking detector ships disarmed -- it judges a joint against its
//! goal, and until a plant model replaces what it judges against it raises
//! nothing -- so what this run exercises is the machine carrying on through a
//! stall: the jammed cranks sit far past the detector's threshold for the whole
//! jam and close again when the hand comes off, no fault is reported, the
//! keep-alive goal stream names every joint throughout so the driver's dead-man
//! never fires, and the raise runs out on its own clock with the fold opening
//! where the script said it would.
//!
//! The first session therefore runs its schedule out, and a second script is
//! taken afterwards by the machine it let go of -- the ordinary life of a
//! session, twice. That second half is the half of the doctrine's park/rest
//! split a refusal cannot show: S8's parked machine refuses a script, and this
//! one runs it.
//!
//! Re-arming the detector adds the report spacing, the tick abandoning the
//! move, and the rest-class response end to end
//! (`TODO(tracking-response-model)`). While the detector is disarmed that
//! coverage is the motion crate's, in the `motion_tick` cases that arm it.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use brenn_reachy__motion__joints_clk_rs::JointFlags;
use scenario::author::Step;
use scenario::{cycle_at, engage_allowance_cycles, release_allowance_cycles, run_end_cycle};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;
use reachy_motion::joints::JointGroup;

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The first script's number.
pub const SCRIPT_ID: u32 = 2;

/// The second script's number: the one a machine that ran its schedule out takes.
pub const SECOND_SCRIPT_ID: u32 = 12;

/// How many cycles into the raise the cranks are jammed.
///
/// Part way in rather than at its start: what the detector measures is a joint
/// that stopped closing on a goal that is still moving away from it, and a
/// machine jammed before it was ever commanded would be measured against a goal
/// it was already standing on.
pub const OBSTRUCT_AFTER: i64 = 10;

/// How long the jam lasts, in cycles.
///
/// Many times the detector's window, so a run of ticks long enough to have
/// raised an armed detector several times over passes under a disarmed one
/// leaving nothing behind but the lag in the samples.
pub const JAM_CYCLES: i64 = 80;

/// How long the upright step lasts, in cycles: long enough to hold the machine
/// through the whole jam and past its release.
pub const UP_CYCLES: i64 = 120;

/// How long the stow step lasts, in cycles.
pub const STOW_CYCLES: i64 = 150;

/// The cycle the cranks are jammed on.
#[must_use]
pub fn obstruct_cycle() -> i64 {
    up_start_cycle() + OBSTRUCT_AFTER
}

/// The cycle the jam is released on.
#[must_use]
pub fn release_cycle() -> i64 {
    obstruct_cycle() + JAM_CYCLES
}

/// The cycle the stow step begins.
#[must_use]
pub fn stow_start_cycle() -> i64 {
    up_start_cycle() + UP_CYCLES
}

/// The cycle the first script's schedule runs out on, which is what ends that
/// session: nothing answers the jam, so the run reaches here.
#[must_use]
pub fn disengage_cycle() -> i64 {
    stow_start_cycle() + STOW_CYCLES
}

/// The cycle the second script is sent on: a whole release allowance after the
/// first session's schedule ran out, so the orderly release is over by here and
/// the machine is resting.
///
/// An outer bound and not an expectation -- which cycle the release concluded on
/// is a fact about the run, and the checker asserts the acceptance against the
/// phase change rather than against this.
#[must_use]
pub fn second_script_cycle() -> i64 {
    disengage_cycle() + release_allowance_cycles()
}

/// The cycle the second script's first step opens on: a whole arming allowance
/// after it, the suite's own convention, so what the step covers is a cycle this
/// module named.
#[must_use]
pub fn second_step_cycle() -> i64 {
    second_script_cycle() + engage_allowance_cycles()
}

/// How long the second script's raise lasts, in cycles.
pub const SECOND_UP_CYCLES: i64 = 120;

/// How long the second script's fold lasts, in cycles.
pub const SECOND_STOW_CYCLES: i64 = 150;

/// The cycle the second script's fold opens on.
#[must_use]
pub fn second_stow_cycle() -> i64 {
    second_step_cycle() + SECOND_UP_CYCLES
}

/// The cycle the second session's schedule runs out on, which is what ends it.
#[must_use]
pub fn second_disengage_cycle() -> i64 {
    second_stow_cycle() + SECOND_STOW_CYCLES
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(second_disengage_cycle())
}

/// The second script's two steps: up, and back down to the fold.
///
/// A whole engagement rather than a token one. What this half of the run is
/// about is that a machine that ran its schedule out takes another script, so
/// the second engagement has to be a real one -- armed over the bus,
/// streamed to, moved somewhere, and released through the orderly path. It
/// therefore asks for a posture the machine is *not* standing in: a script whose
/// only step named the fold the first session left it in would produce a stream of
/// frozen targets, which a schedule that never reached the mover would produce
/// too. It ends at the fold, which is the posture the orderly release expects to
/// measure.
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

/// The two steps of the script.
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
