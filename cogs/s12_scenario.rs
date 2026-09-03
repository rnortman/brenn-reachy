//! S12, a reversal mid-stow: the scenario that says a goal turning round under
//! a lagging joint is not a snag.
//!
//! A machine folding itself away is asked to come back up, a second into a 2 s
//! fold. The head takes it as any replacement: the schedule is swapped under a
//! fresh epoch and the mover splices a raise from the last commanded targets.
//! What makes the run worth a scenario is the antennas, which have three
//! radians to unwind and a servo loop that takes time to turn round: the goal
//! reverses under a joint still carrying the old direction, and for as long as
//! that lasts the joint is running *away* from where it is being asked to go.
//!
//! That is the run the bench produced an `antenna_obstructed` on. Nothing is
//! wrong with the machine: the joint follows, late, and arrives. So the
//! load-bearing assertion here is a negative -- no fault reported, no antenna
//! pair let go -- and the positive beside it is that the head really did come
//! up, measured off the plant.
//!
//! One hand on the plant, laid before anything moves: both antennas are given
//! a response delay. The plant's ordinary row closes a fixed fraction of the
//! gap to its target every cycle and is never behind a goal it is following, so
//! without this the reversal would be a joint that turned round instantly and
//! the run would say nothing about anything. The delay is the *only* injection,
//! and every answer the run gives is the system's own.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. Every instant is a cycle count from the epoch.

use scenario::author::Step;
use scenario::{cycle_at, cycles_for, run_end_cycle, up_cycles};

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

/// How many cycles of response delay the antennas are given.
///
/// Nineteen cycles is 380 ms, which is about what the recorded worst antenna
/// lag amounts to at the rate a raise commands one. What it buys the run is a
/// joint that is a radian behind its goal at the instant the goal turns round,
/// which is the state the tracking window's crossing rule exists for: a
/// scenario whose joint was inside the window's own threshold at the reversal
/// would never open a run at all.
pub const ANTENNA_LAG_CYCLES: u32 = 19;

/// How long the opening script holds the machine up before folding it, in
/// cycles.
///
/// Past the raise and the antennas' own lag, so the fold begins on a machine
/// that has arrived and is standing still: what reverses under the antennas is
/// the fold's own goal and not the tail of the raise.
pub const OPENING_UP_CYCLES: i64 = 80;

/// How many cycles before the fold the antennas are given their delay.
///
/// The delay is laid on a machine standing still, and that is a statement about
/// the plant rather than a convenience. A response delay here is a transport
/// delay -- the row chases the target of some cycles ago -- so a row given one
/// while it is moving stops dead until the ring catches up, and a row given one
/// at the start of a move stands still for the whole delay. A real servo does
/// neither: it sets off at once and trails the goal by a distance that grows
/// with the rate. So the delay is laid where the two models agree, on a joint
/// that is holding, and what the run then says is about the fold it lags and
/// the reversal in the middle of it.
pub const LAG_LAID_BEFORE_STOW: i64 = 20;

/// How long the opening script's fold lasts, in cycles: the 2 s move and room
/// after it.
pub const STOW_CYCLES: i64 = 150;

/// How many cycles into the fold the replacement arrives.
///
/// A second into the 2 s stow, which is where the antennas stand furthest
/// behind it: a transport delay leaves a row trailing by the distance the goal
/// covered while it was waiting, and that distance is largest where the move is
/// fastest. The bench's own wake word landed 1.9 s into its stow, by which
/// point this plant's delayed row is within a third of a radian of its goal --
/// inside the detector's threshold, where no run is open at all and the
/// reversal would exercise a different rule than the one this scenario is for.
/// What the instant is chosen for is the state at the reversal: a joint about a
/// radian behind a goal it is following, which is what the bench recorded.
pub const REVERSAL_AFTER_STOW: i64 = 50;

/// How long the replacement holds the machine up, in cycles.
///
/// Past the raise and the lag with room to spare, so the arrival assertion is
/// made on a machine standing still rather than on one still coming up.
pub const RAISE_CYCLES: i64 = 80;

/// The rows the response delay is laid on: the antenna pair.
///
/// The group rather than two named servos, because what is being modelled is
/// the part -- both antennas are the same fast rotor on the same loop, and a
/// machine that grew another one would want it lagged too.
#[must_use]
pub fn lagged_rows() -> JointFlags {
    JointGroup::Antennas.joints()
}

/// The cycle the opening script's fold begins on.
#[must_use]
pub fn stow_start_cycle() -> i64 {
    up_start_cycle() + OPENING_UP_CYCLES
}

/// The cycle the antennas are given their response delay.
#[must_use]
pub fn lag_cycle() -> i64 {
    stow_start_cycle() - LAG_LAID_BEFORE_STOW
}

/// The cycle the replacement is sent on: a second into the fold, where the
/// lagging antennas stand furthest behind it.
#[must_use]
pub fn reversal_cycle() -> i64 {
    stow_start_cycle() + REVERSAL_AFTER_STOW
}

/// The cycle the antennas are expected to have arrived upright by.
///
/// The raise's own clock, the response delay the plant was given, and the
/// commanded lag a goal carries. Nothing else: a machine that needed longer
/// than its move plus its servos' delay is one whose joints did not follow.
#[must_use]
pub fn raised_cycle() -> i64 {
    reversal_cycle() + up_cycles() + i64::from(ANTENNA_LAG_CYCLES) + scenario::LAG_K
}

/// The cycle the replacement's own fold begins on.
#[must_use]
pub fn second_stow_start_cycle() -> i64 {
    reversal_cycle() + RAISE_CYCLES
}

/// The cycle the replacement's schedule runs out on, which is what ends the
/// session.
#[must_use]
pub fn disengage_cycle() -> i64 {
    second_stow_start_cycle() + STOW_CYCLES
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(disengage_cycle())
}

/// How long the stow move itself takes, in cycles.
///
/// What says the reversal lands inside the move: the fold's clock, against
/// which [`REVERSAL_AFTER_STOW`] is the instant the replacement arrives.
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
            end_ns: cycle_at(stow_start_cycle() + STOW_CYCLES),
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
