//! S5, retarget: the scenario that says a session may change its mind mid-move.
//!
//! The machine is taken under command and sent upright, and part way there --
//! while it is still travelling -- the schedule's next step takes over and sends
//! it to stow instead. What that exercises is the one part of the decision tick
//! no other scenario reaches: a `MoveTo` issued over a move already in flight,
//! from wherever the machine happens to be rather than from a posture it is
//! standing in.
//!
//! One schedule and one epoch, and that is what makes it this scenario. A
//! session mid-engagement will also take a replacement script, which redirects
//! the machine by swapping the schedule and bumping the epoch; here nothing
//! arrives at all. What turns the machine round is a step boundary inside the
//! one schedule it was given, so the retarget is proved on the path that has no
//! epoch change to explain it.
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

use reachy_motion::joints::JointGroup;
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
/// Half way through the head's own travel at the servos' profile, which is what
/// this run needs and not half way through the clock the raise was given: the
/// clock runs out with the head barely off the fold, so a fraction of it would
/// put the retarget on a machine that had hardly set off -- which the checker
/// asserts rather than assumes, because a step boundary after the move had
/// finished is S1's second step under another name.
#[must_use]
pub fn retarget_after() -> i64 {
    scenario::head_up_travel() / 2
}

/// How long the stow step lasts from the retarget, in cycles: the travel the
/// fold takes at the servos' own profile from wherever the raise had got to,
/// plus room to settle onto it.
///
/// A whole posture move's travel and then the raise's own half, because the
/// antennas were part way up when the goal turned round and the arc that takes
/// them back down is the planner's to resolve: the bound is the furthest either
/// of them could have to travel.
#[must_use]
pub fn stow_cycles() -> i64 {
    scenario::posture_step_cycles() + retarget_after()
}

/// The cycle the posture changes on: the instant the upright step gives up its
/// hold on the timeline and the stow step takes over.
#[must_use]
pub fn retarget_cycle() -> i64 {
    up_start_cycle() + retarget_after()
}

/// The cycle the schedule runs out on, which is what ends the session.
#[must_use]
pub fn disengage_cycle() -> i64 {
    retarget_cycle() + stow_cycles()
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(disengage_cycle())
}

/// How many cycles the machine keeps travelling the way it was after the
/// retarget, before an assertion that it is closing on the fold can be made.
///
/// Four terms, and every one of them is the plant's or the goal stream's:
///
/// - the commanded lag a goal carries, and the response delay the servos answer
///   a setpoint at, before the retarget's first setpoint can show in a reading
///   at all;
/// - the cycles the raise had been running, because the fold is planned from the
///   last commanded setpoint and the head is lagging that setpoint by up to
///   everything it has not yet travelled -- so the fold's own goal begins
///   *above* where the head stands and the head goes on climbing toward it,
///   following the new goal rather than the old one;
/// - the generator's own ramp, which is how long a crank at its profile velocity
///   takes to bring that speed through zero once the goal is finally below it.
///
/// An upper bound rather than the instant: the head turns round earlier than
/// this, and what the assertion needs is a cycle by which it certainly has.
#[must_use]
pub fn turnaround_cycles() -> i64 {
    LAG_K
        + scenario::response_delay_cycles()
        + retarget_after()
        + scenario::ramp_cycles(JointGroup::Legs)
}

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
