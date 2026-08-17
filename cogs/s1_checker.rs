//! S1's assertions, over the output log.
//!
//! Three arguments: the output log directory and the two config textprotos the
//! process ran against. Everything asserted here is read out of those -- nothing
//! is taken from the process's console, and nothing is timed against this
//! machine's clock.
//!
//! What is asserted is the whole of a healthy run, and every one of those
//! properties is `scenario::check`'s rather than this file's, because S2 and S3
//! assert most of them too. What is S1's is the list: this is the scenario in
//! which *all* of them hold at once.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it. A scenario that stopped at the first surprise costs a
//! whole build per finding.

use std::process::ExitCode;

use reachy_motion::postures::{neutral_targets, stow_pose_targets};
use scenario::check;
use scenario::read::Run;

use s1_scenario::{
    DISENGAGE_CYCLE, END_CYCLE, ENGAGE_CYCLE, STOW_START_CYCLE, stow_cycles, up_cycles,
};

fn main() -> ExitCode {
    check::main("s1_checker", |run, failures| {
        check::heartbeat(run, END_CYCLE, failures);
        check::readings_present(run, failures);
        if let Some(stream) = check::goal_stream(run, failures) {
            check::stream_covers_session(&stream, ENGAGE_CYCLE, DISENGAGE_CYCLE, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        check_arrival(run, failures);
        check::no_faults(run, failures);
        check::no_events(run, failures);
        check::signal_groups(run, failures);
    })
}

/// The machine arrives: upright by the end of the step that sends it there, and
/// stowed by the end of the step that brings it back.
fn check_arrival(run: &Run, failures: &mut Vec<String>) {
    check::arrived_at(
        run,
        "upright",
        STOW_START_CYCLE - 1,
        &neutral_targets(),
        failures,
    );
    check::arrived_at(
        run,
        "stowed",
        DISENGAGE_CYCLE - 1,
        &stow_pose_targets(),
        failures,
    );
    check::room(
        "upright",
        STOW_START_CYCLE - ENGAGE_CYCLE,
        up_cycles(),
        failures,
    );
    check::room(
        "stow",
        DISENGAGE_CYCLE - STOW_START_CYCLE,
        stow_cycles(),
        failures,
    );
}
