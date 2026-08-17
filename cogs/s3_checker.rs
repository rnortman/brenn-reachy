//! S3's assertions, over the output log.
//!
//! Three arguments: the output log directory and the two config textprotos the
//! process ran against. What is asserted is that nothing commanded the machine,
//! that the driver noticed, that it said so once, and that the machine it left
//! behind is de-torqued and standing still.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use reachy_wire::EventKind;
use scenario::check;
use scenario::read::Run;
use scenario::{FIRST_CYCLE, dead_man_latch_cycle, silence_us};

use s3_scenario::{IDLE_EPOCH, TORQUE_ON_CYCLE, end_cycle};

fn main() -> ExitCode {
    check::main("s3_checker", |run, failures| {
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        // The decision tick never engages, so it never arms, never ticks and
        // has nothing to report. A fault here would be a tick running on a
        // machine no session asked for.
        check::no_faults(run, failures);
        check::signal_groups(run, failures);
        // The session said something, and what it said was "not engaged". Left
        // unasserted, a run in which the schedule went to the wrong channel or
        // never arrived would be bit-identical to this one -- and that run is
        // the case this scenario was written not to be.
        check::schedules_replayed(
            run,
            &[check::Session {
                cycle: TORQUE_ON_CYCLE,
                engaged: false,
                epoch: IDLE_EPOCH,
            }],
            failures,
        );

        check_silence(run, failures);
        let latched = check_dead_man(run, failures);
        check::latch_from(run, latched, failures);
        check::stands_still(
            run,
            FIRST_CYCLE,
            end_cycle(),
            "standing where it stood, and nothing in this run ever commanded it",
            failures,
        );
    })
}

/// Nothing commanded the machine: the goal stream is the silence the rest of
/// the scenario is about.
fn check_silence(run: &Run, failures: &mut Vec<String>) {
    if !run.goals.is_empty() {
        failures.push(format!(
            "{} goals were published on a run where no session ever engaged",
            run.goals.len()
        ));
    }
}

/// The dead-man: exactly one torque-off, of that kind, on the cycle the
/// configured timeout puts it on, carrying the silence it measured.
///
/// The arithmetic is this scenario's; asserting it is `scenario::check`'s. The
/// gate's window opens on the cycle the arming injection is drained -- an
/// injection is drained by the first driver execution at or after the instant
/// it names, and this one is published before the first execution of the run,
/// so that execution is the driver's first. The gate then latches on the first
/// cycle *past* the timeout, which is one further out than the timeout itself,
/// and the silence it reports is the distance from the window's opening to
/// there.
///
/// Measured from the stated first cycle rather than from the run's own, because
/// this is the only scenario with no goal stream to pin its clock: a regression
/// that delayed the driver would otherwise shift the expectation by exactly as
/// much as it shifted the run, and the scenario that exists to prove the last
/// line of defence fires on time would stop noticing when it did not.
///
/// The cycle it fired on, for the latch assertion to measure from.
fn check_dead_man(run: &Run, failures: &mut Vec<String>) -> Option<i64> {
    if TORQUE_ON_CYCLE > FIRST_CYCLE {
        failures.push(format!(
            "the machine was energised at cycle {TORQUE_ON_CYCLE} and the driver's first cycle is \
             {FIRST_CYCLE}: the window opens somewhere this file does not know how to name"
        ));
        return None;
    }
    let wanted = dead_man_latch_cycle(FIRST_CYCLE);
    check::sole_event(
        run,
        EventKind::HoldTimeoutTorqueOff,
        wanted,
        silence_us(FIRST_CYCLE, wanted),
        failures,
    )
}
