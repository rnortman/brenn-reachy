//! S1, raise-hold-stow: the scenario that says the loop closes.
//!
//! One session, no faults, nothing in the way. The session commissions the
//! machine, a script asks for the upright posture and then for stow, the session
//! takes hold of the machine over the bus, the schedule runs, and the session
//! lets go of it again. What that exercises is every part of the system at once:
//! the survey and the arming driven one transaction per wake, the driver's
//! heartbeat clocking two cogs, the decision tick arming off a measured pose and
//! hosting the motion library, the goal stream reaching the gate two cycles ahead
//! of the instant it names, the plant tracking it, the estimator reading the
//! result back, and the orderly release the session ends at -- with the
//! keep-alive rule carrying the dead-man through every stretch nothing is
//! streaming.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use scenario::author::Step;
use scenario::{cycle_at, run_end_cycle};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The script's number. Any number; what it buys is that the acceptance the
/// session narrates names the request the scenario made.
pub const SCRIPT_ID: u32 = 1;

/// How long the upright step lasts, in cycles: the move plus room to arrive and
/// hold. The hold is the point -- a goal stream that stopped when the machine
/// arrived would trip the driver's dead-man, and this step is long enough that
/// it would.
pub const UP_CYCLES: i64 = 100;

/// How long the stow step lasts, in cycles: the longer move plus the same room.
pub const STOW_CYCLES: i64 = 150;

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

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(disengage_cycle())
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
