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
///
/// Longer than the dead-man needs, and set by what the stillness section
/// requires instead: the move streams a new setpoint every cycle for its whole
/// clock, and only then does the watch spend its settle allowance and ask for
/// its minimum hold, so a step that arrives and holds a second is a run nothing
/// can be measured over. `scenario::unjudgeable_step` states that sum against
/// the watch's own config and the clocks this move runs on, the case below
/// drives it, S1's checker echoes it over each run, and
/// `//cogs:first_motion_report_over_s1_test` fails a run holding no antenna
/// long enough to ask.
pub const UP_CYCLES: i64 = 400;

/// How long the stow step lasts, in cycles: the travel the fold takes at the
/// servos' own profile, plus room to settle onto it.
///
/// Not the move's clock. The fold's clock is 100 cycles and an antenna's 3.4 rad
/// of travel takes half again as many, so a step sized on the clock would put
/// the arrival assertion on a machine still coming down.
#[must_use]
pub fn stow_cycles() -> i64 {
    scenario::posture_step_cycles()
}

/// The cycle the stow step begins.
#[must_use]
pub fn stow_start_cycle() -> i64 {
    up_start_cycle() + UP_CYCLES
}

/// The cycle the schedule runs out on, which is what ends the session.
#[must_use]
pub fn disengage_cycle() -> i64 {
    stow_start_cycle() + stow_cycles()
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

#[cfg(test)]
mod tests {
    use reachy_motion::stillness::StillnessConfig;
    use scenario::{PERIOD_NS, unjudgeable_step, up_clocks};

    use super::UP_CYCLES;

    /// The scenario's own step is long enough for the analyzer that reads its
    /// log to find a hold in it.
    ///
    /// Here rather than in the checker alone: the claim is arithmetic over
    /// constants, and a guard that only runs when the simulated run produced a
    /// readable log is silent in exactly the state a shortened step leaves the
    /// tree in.
    #[test]
    fn the_upright_step_outlasts_what_the_watch_needs_to_judge_a_hold() {
        let complaint = unjudgeable_step(
            "upright",
            UP_CYCLES * PERIOD_NS,
            &up_clocks(),
            &StillnessConfig::default(),
        );
        assert!(complaint.is_none(), "{complaint:?}");
    }
}
