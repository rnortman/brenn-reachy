//! S3's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What is asserted is that nothing commanded the machine,
//! that the driver noticed, that it said so once, and that the machine it left
//! behind is de-torqued and standing still.
//!
//! "Nothing commanded the machine" is about the goal stream. The session does
//! talk to the driver -- it commissions the machine first, as it does in every
//! run -- and that traffic is what the dead-man's window opens after.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__driver__health_clk_rs::EventKind;
use scenario::check;
use scenario::read::Run;
use scenario::{FIRST_CYCLE, dead_man_latch_cycle, silence_ns};

use s3_scenario::{TORQUE_ON_CYCLE, end_cycle};

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
        // Nobody asked the machine for anything, so the session published no
        // schedule at all. Asserted rather than assumed: a run in which a
        // schedule went out anyway would be a session that engaged a machine
        // nobody sent a script for.
        check::scripts_sent(run, &[], failures);
        check::no_schedules(run, failures);

        // The survey the session runs before anything else, and what it cost:
        // this scenario's whole subject is a silence that begins when the survey
        // ends, so a survey that spent twice the traffic would move the window
        // the dead-man is measured in with it.
        check::commissioning(run, failures);

        // Nothing commanded the machine: the goal stream is the silence the
        // rest of the scenario is about.
        check::no_goals(run, "no session ever engaged", failures);
        let latched = check_dead_man(run, failures);
        check::latch_from(run, latched, failures);
        // And the driver read it back: the sweep it wrote is a claim, and a
        // whole clean pass over the bus is the evidence for it.
        check::confirmed_off(run, latched, failures);
        check::only_kinds(
            run,
            &[
                EventKind::HoldTimeoutTorqueOff,
                EventKind::TorqueOffConfirmed,
            ],
            failures,
        );
        check::stands_still(
            run,
            FIRST_CYCLE,
            end_cycle(),
            "standing where it stood, and nothing in this run ever commanded it",
            failures,
        );
    })
}

/// The dead-man: exactly one torque-off, of that kind, on the cycle the
/// configured timeout puts it on, carrying the silence it measured.
///
/// The arithmetic is this scenario's; asserting it is `scenario::check`'s. The
/// window this run's silence is measured in opens on the cycle the driver drains
/// the last thing the session said -- every accepted datagram is liveness, and
/// the survey the session runs first spends two hundred of them. The gate then
/// latches on the first cycle *past* the timeout, which is one further out than
/// the timeout itself, and the silence it reports is the distance from the
/// window's opening to there.
///
/// The window's opening is taken from the log rather than counted out here, for
/// the reason the timeout is not: how long the survey takes is the library's
/// arithmetic over its own sweeps, and a scenario that restated it would fail
/// whenever a register was added to one. What must not move is the *distance*
/// from the last word to the latch, which is the configured timeout, and that is
/// what this asserts.
///
/// The energising injection is checked to land inside the window all the same: a
/// machine energised after the survey ended would open a window this file is not
/// naming.
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
    // The survey ran and ended at rest: a run that parked instead would go
    // quiet for a different reason than the one this scenario is about.
    let surveyed = check::commissioning(run, failures);
    let opened = check::last_datagram(run, failures)?;
    if surveyed.is_none() {
        failures.push(format!(
            "the survey had not finished by the end of the run, and the last datagram of it went \
             out at cycle {opened}: a run that ends mid-survey has not reached the silence this \
             scenario is about"
        ));
        return None;
    }
    let wanted = dead_man_latch_cycle(opened);
    check::sole_event(
        run,
        EventKind::HoldTimeoutTorqueOff,
        wanted,
        silence_ns(opened, wanted),
        failures,
    )
}
