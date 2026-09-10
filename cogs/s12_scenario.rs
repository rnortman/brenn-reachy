//! S12, the mid-fold re-raise: the scenario that says a wake word landing
//! inside the machine's own fold is answered by a splice and nothing else.
//!
//! A machine folding itself away is asked to come back up, a second into a 2 s
//! fold. The head takes it as any replacement: the schedule is swapped under a
//! fresh epoch and the mover splices a raise from the last commanded targets.
//! What makes the run worth a scenario is the instant it lands on -- one cycle
//! inside the fold's move, so what turns round is a goal that had not yet
//! stopped, and the joints reverse with it. No other scenario in this suite
//! carries a replacement that turns the joints' direction round mid-move.
//!
//! The load-bearing assertions are the replacement's mechanics -- the schedule
//! swapped under a fresh epoch on the wake it arrived, the reversal landing
//! inside the move, the raise reaching the posture measured off the plant --
//! and the negative beside them: nothing was raised for the splice, no pair was
//! let go, no torque was touched.
//!
//! At the shipped servo profiles the fold is *followed*, not chased: the
//! antennas' generator carries ten times the fold's arc in the fold's clock, so
//! on the reversal cycle every antenna stands nearer its goal than the figure
//! the detector screens at. The checker states that as a fact rather than
//! leaving it implied, because it is the headroom the commissioning was sized
//! to keep over the machine's own postures. It is a statement about goal error
//! and not about the detector: in this harness the simulated shaft is the
//! tick's own plant model, so an unobstructed row's residual is nil and the
//! screen is not exercised here at all. The detector's rule is exercised by the
//! pace cases in `crates/reachy-motion/src/tick.rs` and its silence on the real
//! machine is the hardware record in `docs/servo-tuning.md`.
//!
//! No hand on the plant at all: the modelled servos run the profile the
//! commissioning sweep wrote into them, and every answer the run gives is the
//! system's own.
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
/// One cycle inside the fold's move, so that the goal turning round is a goal
/// that had not yet stopped: what the splice reverses is a joint still
/// following a moving target, which is the run's whole subject.
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
/// machine that grew another one would follow the fold with them.
#[must_use]
pub fn antenna_rows() -> JointFlags {
    JointGroup::Antennas.joints()
}

/// The cycle the opening script's fold begins on.
#[must_use]
pub fn stow_start_cycle() -> i64 {
    up_start_cycle() + opening_up_cycles()
}

/// The cycle the replacement is sent on: the end of the fold's move, with the
/// goal still travelling.
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
