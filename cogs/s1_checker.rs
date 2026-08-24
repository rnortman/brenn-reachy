//! S1's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
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

use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use reachy_motion::postures::{neutral_targets, stow_pose_targets};
use scenario::check;
use scenario::read::Run;

use scenario::{stow_clocks, up_clocks};

use s1_scenario::{
    SCRIPT_ID, UP_CYCLES, disengage_cycle, end_cycle, script_sent_cycle, stow_start_cycle,
    up_start_cycle,
};

fn main() -> ExitCode {
    check::main("s1_checker", |run, failures| {
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        // The one thing the scenario itself said, and that it reached the run at
        // all: everything below is the system's answer to this message.
        check::scripts_sent(run, &[(SCRIPT_ID, script_sent_cycle())], failures);
        let engaged = check::engagement(run, failures);
        check::ended_promptly(
            engaged.map(|engaged| engaged.released),
            disengage_cycle(),
            failures,
        );
        if let (Some(stream), Some(engaged)) = (check::goal_stream(run, failures), engaged) {
            // From the engagement rather than from the first step: the machine
            // is under command from the moment it is armed, and what it is
            // commanded to do before a step covers an instant is to hold where
            // it stands.
            check::stream_starts_with_session(&stream, engaged.taken, failures);
            check::stream_stops_with_release(&stream, engaged.released, failures);
        }
        // The profile the process was configured with, as the nine servos were
        // told it. S1's alone among the scenarios: the sweep is the same sweep
        // in every run, and this is the run in which nothing perturbs it.
        check::commissioned_profile(run, failures);
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        check_arrival(run, failures);
        check::no_faults(run, failures);
        check::no_events(run, failures);
        // Exactly this narration and nothing else: a refused script or a fault
        // read off a health rotation fails here rather than passing unseen.
        check::narration(
            run,
            &[
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::SCRIPT_ACCEPTED,
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::SCHEDULE_PUBLISHED,
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::SCHEDULE_PUBLISHED,
                ReportKindWire::SESSION_ENDED,
                ReportKindWire::PHASE_CHANGED,
            ],
            failures,
        );
        check::signal_groups(run, failures);
    })
}

/// The machine arrives: upright by the end of the step that sends it there, and
/// stowed by the end of the step that brings it back.
fn check_arrival(run: &Run, failures: &mut Vec<String>) {
    check::arrived_at(
        run,
        "upright",
        stow_start_cycle() - 1,
        &neutral_targets(),
        failures,
    );
    check::arrived_at(
        run,
        "stowed",
        disengage_cycle() - 1,
        &stow_pose_targets(),
        failures,
    );
    // The pair is parted at its crossing on both moves, each by what its own
    // floored clocks part by, less the allowance for the tails the detector
    // cannot see. Per move rather than one number for both: the two clocks are
    // parted by different amounts, and a threshold taken from the shorter move
    // says nothing about the longer one.
    check::pair_de_phased(
        run,
        "stand-up",
        up_start_cycle(),
        stow_start_cycle(),
        up_clocks().parting_least(),
        failures,
    );
    check::pair_de_phased(
        run,
        "fold",
        stow_start_cycle(),
        disengage_cycle(),
        stow_clocks().parting_least(),
        failures,
    );
    check::room("upright", UP_CYCLES, &up_clocks(), failures);
    check::room(
        "stow",
        disengage_cycle() - stow_start_cycle(),
        &stow_clocks(),
        failures,
    );
}
