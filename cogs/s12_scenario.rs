//! S12, a reversal mid-stow: the scenario that says a goal turning round under
//! a lagging joint is not a snag.
//!
//! A machine folding itself away is asked to come back up, a second into a 2 s
//! fold. The head takes it as any replacement: the schedule is swapped under a
//! fresh epoch and the mover splices a raise from the last commanded targets.
//! What makes the run worth a scenario is the antennas, which have three
//! radians to unwind at a servo profile that carries a fifth of that in the
//! time the fold is given: the goal reverses under a joint still carrying the
//! old direction, and for as long as that lasts the joint is running *away*
//! from where it is being asked to go.
//!
//! That is the run the bench produced an `antenna_obstructed` on. Nothing is
//! wrong with the machine: the joint follows, late, and arrives. So the
//! load-bearing assertion here is a negative -- no fault reported, no antenna
//! pair let go -- and the positive beside it is that the head really did come
//! up, measured off the plant.
//!
//! No hand on the plant at all. The modelled servos run the profile the
//! commissioning sweep wrote into them, and the fold asks an antenna for
//! 2.9 rad in 2 s -- more than twice what that profile can carry -- so the joint
//! trails the fold's goal by more than the detector's threshold on its own.
//! Every answer the run gives is the system's own.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. Every instant is a cycle count from the epoch.

use scenario::author::Step;
use scenario::{cycle_at, cycles_for, run_end_cycle};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use reachy_motion::joints::JointGroup;

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The number of the script that raises the machine and then folds it.
pub const OPENING_SCRIPT_ID: u32 = 12;

/// The number of the script that arrives mid-fold and asks for the machine
/// back.
///
/// Strictly greater than the one before it, which is the ordering rule a
/// replacement is screened against.
pub const REVERSAL_SCRIPT_ID: u32 = 13;

/// How long the opening script holds the machine up before folding it, in
/// cycles.
///
/// Past the travel the raise itself takes, so the fold begins on a machine that
/// has arrived and is standing still: what reverses under the antennas is the
/// fold's own goal and not the tail of the raise. The clock the raise is given
/// is a third of that travel, which is the whole reason this figure is a travel
/// expression and not the clock.
#[must_use]
pub fn opening_up_cycles() -> i64 {
    scenario::posture_step_cycles()
}

/// How long each fold lasts, in cycles: the travel the fold takes at the
/// servos' own profile, plus room to settle onto it.
#[must_use]
pub fn stow_cycles() -> i64 {
    scenario::posture_step_cycles()
}

/// How many cycles into the fold the replacement arrives.
///
/// Where the joint stands furthest behind the fold's goal. The fold asks for
/// 2.9 rad of antenna in the 100 cycles its clock runs and the servo's profile
/// carries 0.024 rad a cycle, so the gap grows for as long as the goal is
/// moving and is widest on the last cycle of the move -- which is where this
/// lands, one cycle inside the move so that the goal turning round is a goal
/// that had not yet stopped.
#[must_use]
pub fn reversal_after_stow() -> i64 {
    stow_move_cycles() - 1
}

/// How long the replacement holds the machine up, in cycles.
///
/// Past the travel a raise from wherever the fold got to takes, with room to
/// spare, so the arrival assertion is made on a machine standing still rather
/// than on one still coming up.
#[must_use]
pub fn raise_cycles() -> i64 {
    scenario::posture_step_cycles()
}

/// The rows this run is about: the antenna pair.
///
/// The group rather than two named servos, because what is being measured is
/// the part -- both antennas are the same rotor on the same profile, and a
/// machine that grew another one would trail the fold with it.
#[must_use]
pub fn antenna_rows() -> JointFlags {
    JointGroup::Antennas.joints()
}

/// The cycle the opening script's fold begins on.
#[must_use]
pub fn stow_start_cycle() -> i64 {
    up_start_cycle() + opening_up_cycles()
}

/// The cycle the replacement is sent on: at the end of the fold's move, where
/// the lagging antennas stand furthest behind it.
#[must_use]
pub fn reversal_cycle() -> i64 {
    stow_start_cycle() + reversal_after_stow()
}

/// The cycle the antennas are expected to have arrived upright by.
///
/// The travel the raise takes at the plant's own profile, from the fold -- which
/// is further than wherever the interrupted fold had got to. Nothing else: a
/// machine that needed longer than the profile does is one whose joints did not
/// follow.
#[must_use]
pub fn raised_cycle() -> i64 {
    reversal_cycle() + scenario::posture_travel()
}

/// The cycle the replacement's own fold begins on.
#[must_use]
pub fn second_stow_start_cycle() -> i64 {
    reversal_cycle() + raise_cycles()
}

/// The cycle the replacement's schedule runs out on, which is what ends the
/// session.
#[must_use]
pub fn disengage_cycle() -> i64 {
    second_stow_start_cycle() + stow_cycles()
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(disengage_cycle())
}

/// How long the stow move itself takes, in cycles.
///
/// What says the reversal lands inside the move: the fold's clock, against
/// which [`reversal_after_stow`] is the instant the replacement arrives.
#[must_use]
pub fn stow_move_cycles() -> i64 {
    cycles_for(scenario::STOW_DURATION_NS)
}

/// The two steps of the opening script: up, and then the fold.
#[must_use]
pub fn opening_steps() -> [Step; 2] {
    [
        Step {
            start_ns: cycle_at(up_start_cycle()),
            end_ns: cycle_at(stow_start_cycle()),
            posture: Some(PostureWire::UP),
        },
        Step {
            start_ns: cycle_at(stow_start_cycle()),
            end_ns: cycle_at(stow_start_cycle() + stow_cycles()),
            posture: Some(PostureWire::STOW),
        },
    ]
}

/// The two steps of the replacement: up from the instant it arrives, and then
/// the fold that ends the session.
///
/// Its raise begins on the instant it is sent, which is what a wake word asks
/// for: the head coming up is how the machine answers, so there is nothing to
/// wait for.
#[must_use]
pub fn reversal_steps() -> [Step; 2] {
    [
        Step {
            start_ns: cycle_at(reversal_cycle()),
            end_ns: cycle_at(second_stow_start_cycle()),
            posture: Some(PostureWire::UP),
        },
        Step {
            start_ns: cycle_at(second_stow_start_cycle()),
            end_ns: cycle_at(disengage_cycle()),
            posture: Some(PostureWire::STOW),
        },
    ]
}
