//! S5, retarget: the scenario that says a session may change its mind mid-move.
//!
//! The machine is taken under command and sent upright, and part way there --
//! while it is still travelling -- the schedule's next step takes over and sends
//! it to stow instead. What that exercises is the one part of the decision tick
//! no other scenario reaches: a `MoveTo` issued over a move already in flight,
//! from wherever the machine happens to be rather than from a posture it is
//! standing in.
//!
//! One schedule and one epoch. A session mid-engagement refuses a fresh script
//! rather than replacing what it is running, so what redirects a machine inside
//! one session is a step boundary, and that is what this run is written on.
//!
//! Three things have to hold across that, and each of them is a way the loop
//! could break quietly. The goal stream must not gap: a retarget is a new
//! command inside one session, and a cycle without a datagram is a cycle of
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
use scenario::{LAG_K, cycle_at, run_end_cycle};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The script's number.
pub const SCRIPT_ID: u32 = 5;

/// How many cycles into the raise the posture changes.
///
/// Half way through it. Early enough that the machine is still moving -- which
/// the checker asserts rather than assumes, because a step boundary after the
/// move had finished is S1's second step under another name.
pub const RETARGET_AFTER: i64 = 20;

/// How long the stow step lasts from the retarget, in cycles: the longer move
/// from wherever the raise had got to, plus room to arrive and hold.
pub const STOW_CYCLES: i64 = 150;

/// The cycle the posture changes on: the instant the upright step gives up its
/// hold on the timeline and the stow step takes over.
#[must_use]
pub fn retarget_cycle() -> i64 {
    up_start_cycle() + RETARGET_AFTER
}

/// The cycle the schedule runs out on, which is what ends the session.
#[must_use]
pub fn disengage_cycle() -> i64 {
    retarget_cycle() + STOW_CYCLES
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(disengage_cycle())
}

/// How many cycles the machine keeps travelling the way it was after the
/// retarget, before the new setpoint can have reached the plant at all.
///
/// The goal decided on the retarget cycle names an instant `lag_k` cycles out,
/// and the driver executes it there. So the turnaround is not observable in the
/// measured pose until the cycle after that, and an assertion that the machine
/// is closing on its new posture has to start from then rather than from the
/// cycle the step changed on.
pub const TURNAROUND_CYCLES: i64 = LAG_K + 1;

/// The two steps of the script: upright, and then stow from part way there.
///
/// The boundary is the subject. One schedule and one epoch, so what redirects
/// the machine is a step handing the timeline over to the next while the move it
/// asked for is still in flight -- not a fresh request, which a session
/// mid-engagement refuses.
#[must_use]
pub fn steps() -> [Step; 2] {
    [
        Step {
            start_ns: cycle_at(up_start_cycle()),
            end_ns: cycle_at(retarget_cycle()),
            posture: Some(PostureWire::UP),
        },
        Step {
            start_ns: cycle_at(retarget_cycle()),
            end_ns: cycle_at(disengage_cycle()),
            posture: Some(PostureWire::STOW),
        },
    ]
}
