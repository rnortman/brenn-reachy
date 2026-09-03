//! S12's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. Everything asserted here is read out of those.
//!
//! Three claims are S12's own. The antennas really were lagging when the goal
//! turned round -- past the tracking window's own threshold, which is what says
//! the run reached the code it is about rather than skirting it. Nothing was
//! reported about them: no fault on the tick's channel, and no row of the
//! session's narration about a fault or a pair let go. And the head came up all
//! the same, measured off the plant, with the antennas pointing where the
//! posture puts them.
//!
//! Everything else is `scenario::check`'s and every one of them still holds: one
//! engagement across both scripts, one unbroken stretch of goal stream, the
//! schedule replaced under a fresh epoch, and the session ending when its
//! schedule ran out.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use reachy_motion::default_motion_config;
use reachy_motion::joints::{Name, flags, row};
use reachy_motion::postures::{neutral_targets, stow_pose_targets};
use scenario::check;
use scenario::read::Run;

use s12_scenario::{
    OPENING_SCRIPT_ID, REVERSAL_AFTER_STOW, REVERSAL_SCRIPT_ID, disengage_cycle, end_cycle,
    lagged_rows, raised_cycle, reversal_cycle, script_sent_cycle, second_stow_start_cycle,
    stow_move_cycles, stow_start_cycle,
};

fn main() -> ExitCode {
    check::main("s12_checker", |run, failures| {
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        check::scripts_sent(
            run,
            &[
                (OPENING_SCRIPT_ID, script_sent_cycle()),
                (REVERSAL_SCRIPT_ID, reversal_cycle()),
            ],
            failures,
        );
        // One engagement carrying both scripts. A session that answered the
        // replacement by letting go and taking hold again would carry four more
        // phase changes here, and the reversal this run is about would never
        // have happened at all: the mover would have re-planned from a machine
        // it had just armed.
        //
        // The sequence and the two cycles come from `ordinary_life` rather than
        // from `engagement`: what this scenario is about is a third schedule
        // this session published, so the schedule shape is `check_replacement`'s
        // below and not the ordinary one.
        let (engaged, _) = check::ordinary_life(run, &[], failures);
        check::ended_promptly(
            engaged.map(|engaged| engaged.released),
            disengage_cycle(),
            failures,
        );
        // One stretch of goal stream over the whole engagement: the replacement
        // changed the schedule without touching torque, so nothing stopped
        // commanding across it and no row ever left service -- an antenna the
        // session had let go of would be missing from every goal after it.
        if let (Some(stream), Some(engaged)) = (check::goal_stream(run, failures), engaged) {
            check::stream_starts_with_session(&stream, engaged.taken, failures);
            check::stream_stops_with_release(&stream, engaged.released, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        check_reversal_landed_mid_move(failures);
        check_the_antennas_were_lagging(run, failures);
        check_replacement(run, engaged, failures);
        check_arrival(run, failures);
        // The load-bearing negative, in both places a complaint about an
        // antenna would appear: the tick's own channel, and the session's
        // narration. The narration is stated whole, so a fault row, a response
        // or a pair let go is a failure here whatever else the run did.
        check::no_faults(run, failures);
        check::narration(
            run,
            &[
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::SCRIPT_ACCEPTED,
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::SCHEDULE_PUBLISHED,
                ReportKindWire::SCRIPT_REPLACED,
                ReportKindWire::SCHEDULE_PUBLISHED,
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::SCHEDULE_PUBLISHED,
                ReportKindWire::SESSION_ENDED,
                ReportKindWire::PHASE_CHANGED,
            ],
            failures,
        );
        // Nothing the driver's gate did: the reversal is a decision the mover
        // and the session made between them, and a de-torquing anywhere in it
        // would be an event.
        check::no_events(run, failures);
        check::signal_groups(run, failures);
    })
}

/// The replacement arrived while the fold was still moving.
///
/// Arithmetic on the scenario's own numbers rather than a reading of the run,
/// and it is here because it is the premise every other assertion rests on: a
/// replacement that landed after the stow had arrived would find the antennas
/// standing still, and a joint with nothing left to carry the old direction
/// with reverses instantly however lagged its loop is.
fn check_reversal_landed_mid_move(failures: &mut Vec<String>) {
    let moving = stow_move_cycles();
    if REVERSAL_AFTER_STOW >= moving {
        failures.push(format!(
            "the replacement arrives {REVERSAL_AFTER_STOW} cycles into a fold whose move takes \
             {moving}: what this run is about is a goal turning round under a joint that is still \
             following it"
        ));
    }
}

/// The antennas were behind their goal, by more than the detector's threshold,
/// on the wake the replacement arrived on.
///
/// The assertion that the run reached the rule it is about. The tracking window
/// does not look at a joint nearer its goal than `threshold_rad` at all, so a
/// plant that ignored the response delay -- or a delay too short to matter --
/// would produce a run in which no window was ever open, every negative below
/// would pass, and none of them would mean anything.
fn check_the_antennas_were_lagging(run: &Run, failures: &mut Vec<String>) {
    let threshold = default_motion_config().tracking.threshold_rad;
    let at = reversal_cycle();
    let (Some(sample), Some(goal)) = (check::sample_at(run, at), check::goal_at(run, at)) else {
        failures.push(format!(
            "cycle {at} carries no sample and goal to measure the antennas' lag against"
        ));
        return;
    };
    let present = check::present_rows(sample);
    for joint in flags::iter(lagged_rows()) {
        let Some(index) = row(joint) else {
            failures.push(format!("{} sits on no bus row", Name(joint)));
            continue;
        };
        let error = (present[index] - goal[index]).abs();
        if error <= threshold {
            failures.push(format!(
                "at cycle {at} {} stood {error} rad from its goal, inside the {threshold} rad the \
                 detector screens out: this run is about a goal turning round under a joint that \
                 is lagging it, and nothing was lagging",
                Name(joint)
            ));
        }
    }
}

/// The session replaced the running schedule under a fresh epoch, on the wake
/// the replacement arrived on, and nothing disengaged for it.
///
/// Three schedules: the arming's, the replacement's, and the one nobody is
/// running. What is this run's own is the wake the middle one went out on: a
/// replacement the session published a wake late is a machine that carried on
/// with the old schedule through the reversal this run is about.
fn check_replacement(run: &Run, engaged: Option<check::Engaged>, failures: &mut Vec<String>) {
    let published = check::schedules_under_one_engagement(
        run,
        &["the arming", "the replacement"],
        check::Tail::Nothing,
        engaged,
        failures,
    );
    let [_, replaced, _] = published.as_slice() else {
        return;
    };
    check::answered_on_its_wake(
        "replacement's schedule",
        replaced.0,
        reversal_cycle(),
        failures,
    );
}

/// The machine arrived where each schedule sent it, read off the plant.
///
/// Three instants: upright under the opening script, upright again after the
/// reversal -- which is the positive this whole run is for, since a joint the
/// window had faulted would have been let go of and left where it stood -- and
/// stowed at the end.
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
        "upright again after the reversal",
        raised_cycle(),
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
    if raised_cycle() >= second_stow_start_cycle() {
        failures.push(format!(
            "the raise is asserted arrived on cycle {} and the fold after it begins on {}: the \
             arrival is measured on a machine standing still",
            raised_cycle(),
            second_stow_start_cycle()
        ));
    }
}
