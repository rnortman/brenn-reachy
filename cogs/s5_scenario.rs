//! S5, retarget: the scenario that says a session may change its mind mid-move.
//!
//! The machine is energised and sent upright, and part way there -- while it is
//! still travelling -- a fresh schedule arrives that sends it to stow instead.
//! What that exercises is the one part of the decision tick no other scenario
//! reaches: a `MoveTo` issued over a move already in flight, from wherever the
//! machine happens to be rather than from a posture it is standing in.
//!
//! Three things have to hold across that, and each of them is a way the loop
//! could break quietly. The goal stream must not gap: a retarget is a new
//! command, not a new session, and a cycle without a datagram is a cycle of
//! silence the driver's dead-man is measuring. The instants must stay ordered
//! and the per-cycle travel must stay inside what the plant can do: a
//! turnaround that asked for the whole distance at once is a step no servo can
//! take, and the motion library abandons a move rather than command one. And
//! the machine must actually arrive at the posture it was redirected to,
//! because a retarget that left it holding half way would satisfy the first two.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use scenario::author::Step;
use scenario::{LAG_K, STOW_DURATION_NS, UP_DURATION_NS, cycle_at, cycles_for};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;

/// The cycle the session is engaged and the machine energised at.
pub const ENGAGE_CYCLE: i64 = 0;

/// The cycle the replacement schedule is published on.
///
/// Half way through the raise. Early enough that the machine is still moving --
/// which the checker asserts rather than assumes, because a retarget of a move
/// that had already finished is S1's second step under another name.
pub const RETARGET_CYCLE: i64 = 20;

/// How long the stow step lasts from the retarget, in cycles: the longer move
/// from wherever the raise had got to, plus room to arrive and hold.
pub const STOW_CYCLES: i64 = 150;

/// The cycle the session disengages and the machine is de-energised at.
pub const DISENGAGE_CYCLE: i64 = RETARGET_CYCLE + STOW_CYCLES;

/// How long the run continues past the disengagement, in cycles: long enough
/// for the dead-man to have fired if the machine had been left energised.
pub const TAIL_CYCLES: i64 = 20;

/// The last cycle of the run.
pub const END_CYCLE: i64 = DISENGAGE_CYCLE + TAIL_CYCLES;

/// The epoch of the schedule published at engagement.
pub const ENGAGED_EPOCH: u32 = 1;

/// The epoch of the schedule that replaces the posture.
///
/// A different number is the whole mechanism: the decision tick holds the last
/// epoch it acted on, and what makes a schedule news is that number changing.
/// Two schedules under one epoch would be the session saying nothing twice.
pub const RETARGET_EPOCH: u32 = 2;

/// The epoch of the schedule published at disengagement.
pub const DISENGAGED_EPOCH: u32 = 3;

/// How many cycles the machine keeps travelling the way it was after the
/// retarget, before the new setpoint can have reached the plant at all.
///
/// The goal decided on the retarget cycle names an instant `lag_k` cycles out,
/// and the driver executes it there. So the turnaround is not observable in the
/// measured pose until the cycle after that, and an assertion that the machine
/// is closing on its new posture has to start from then rather than from the
/// cycle the schedule landed on.
pub const TURNAROUND_CYCLES: i64 = LAG_K + 1;

/// The simulated time the run ends at.
#[must_use]
pub fn end_time_ns() -> i64 {
    cycle_at(END_CYCLE)
}

/// The engaged session's schedule: upright, for the whole session.
///
/// One step spanning everything, so that what the retarget replaces is a step
/// the machine is in the middle of rather than a step that was about to end.
#[must_use]
pub fn up_steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(ENGAGE_CYCLE),
        end_ns: cycle_at(DISENGAGE_CYCLE),
        posture: Some(PostureWire::UP),
    }]
}

/// The replacement schedule: stow, over the same span.
///
/// The same span rather than one beginning at the retarget, because a step's
/// bounds say when it applies and not when it was decided: a schedule whose
/// only step began at the instant it was published would leave the tick with no
/// step to be in if it were read a cycle late, which would make a timing
/// question out of a posture question.
#[must_use]
pub fn stow_steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(ENGAGE_CYCLE),
        end_ns: cycle_at(DISENGAGE_CYCLE),
        posture: Some(PostureWire::STOW),
    }]
}

/// How many cycles a move to the upright posture is given, rounded up.
#[must_use]
pub fn up_cycles() -> i64 {
    cycles_for(UP_DURATION_NS)
}

/// How many cycles a move to stow is given, rounded up.
#[must_use]
pub fn stow_cycles() -> i64 {
    cycles_for(STOW_DURATION_NS)
}
