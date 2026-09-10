//! S15's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. Everything asserted here is read out of those.
//!
//! Two claims are S15's own, and they are a premise and a negative. The premise
//! is that the stimulus landed: on the first cycle the tick could judge a
//! reading taken after each of the document's three steps, both antennas stand
//! further from their goal than the planner's own per-tick step bound, so the
//! content really did outrun the servos three times over. The negative is what
//! the run is for -- nothing was raised, nothing let go, no torque touched --
//! and it means something only because the premise holds.
//!
//! The positive beside them: each held pose was reached, measured off the lagged
//! plant, inside the travel the antennas' own profile needs for the arc. A
//! composition that ramped what the clip asked for in one frame, or a plant that
//! never got there, would not be standing on the pose.
//!
//! No claim about the tracking detector is made here, in any wording. The
//! simulated shaft is the tick's own plant model stepped on the same inputs, so
//! an unobstructed row's residual is nil however far the goal jumps and the
//! detector's screen is not exercised on this run. The detector's rule is
//! exercised by the pace cases in `crates/reachy-motion/src/tick.rs`, and its
//! silence on the real machine is the hardware record in
//! `docs/servo-tuning.md`.
//!
//! The per-cycle travel bound the goal stream is otherwise held to is lifted
//! for the antenna rows over the window the clip plays across, and only there:
//! it is the bound the tick refuses *its own planned moves* past, a clip
//! carries no such ceiling, and the raise that brings the same antennas up
//! before the window opens is a planned move the bound still holds.
//!
//! Everything else is `scenario::check`'s: one engagement, one unbroken stretch
//! of goal stream, the session ending when its schedule ran out.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use reachy_motion::joints::{Name, row};
use scenario::check;
use scenario::read::Run;

use s15_scenario::{
    SCRIPT_ID, antenna_rows, arrived_after, disengage_cycle, end_cycle, held_poses, held_targets,
    judged_after, script_sent_cycle, step_arcs, step_cycles, window_close_cycle, window_open_cycle,
};

fn main() -> ExitCode {
    check::main("s15_checker", |run, failures| {
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        check::scripts_sent(run, &[(SCRIPT_ID, script_sent_cycle())], failures);
        let engaged = check::engagement(run, failures);
        check::ended_promptly(
            engaged.map(|engaged| engaged.released),
            disengage_cycle(),
            failures,
        );
        // One unbroken stretch: the window opening and closing changes what is
        // commanded and not whether anything is, so a break anywhere in it is
        // the composition dropping the machine rather than playing over it.
        if let (Some(stream), Some(engaged)) = (
            check::goal_stream_playing(
                run,
                antenna_rows(),
                window_open_cycle()..window_close_cycle(),
                failures,
            ),
            engaged,
        ) {
            check::stream_starts_with_session(&stream, engaged.taken, failures);
            check::stream_stops_with_release(&stream, engaged.released, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        check_the_content_outran_the_servos_on_every_step(run, failures);
        check_arrival(run, failures);
        // The load-bearing negatives, in all three places an answer to the
        // antennas would appear: the tick's own fault channel, the driver's
        // events, and the session's narration stated whole.
        check::no_faults(run, failures);
        check::no_events(run, failures);
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

/// Both antennas stood further from their goal than the planner's own per-tick
/// step bound, on the first cycle the tick could judge a reading after each
/// step.
///
/// The premise the negatives rest on, read against the step bound and not
/// against the detector's screen: the bound is what the planner refuses *its
/// own* moves past, so content past it is the content the goal stream's
/// exemption exists for and the content this machine must lag rather than
/// follow. Three steps, three readings: an edited document whose steps got
/// gentler, a commissioning that made the antennas fast enough to keep up, or
/// a composition that ramped what the clip asked for in one frame all fail
/// here.
fn check_the_content_outran_the_servos_on_every_step(run: &Run, failures: &mut Vec<String>) {
    for (step, (what, _)) in step_cycles().into_iter().zip(held_poses()) {
        let at = judged_after(step);
        let measured = format!("the step to {what}");
        let Some(errors) = check::goal_errors_at(run, at, antenna_rows(), &measured, failures)
        else {
            continue;
        };
        for (joint, error) in errors {
            let Some(bound) = row(joint).and_then(check::step_bound_of) else {
                failures.push(format!("{} sits on no bus row", Name(joint)));
                continue;
            };
            if error <= bound {
                failures.push(format!(
                    "at cycle {at}, one period after the step to {what} could first be answered, \
                     {} stood {error} rad from its goal -- inside the {bound} rad the planner \
                     bounds its own moves to, so this document, this composition or this \
                     commissioning has made the content followable and the run no longer says \
                     what it is kept for",
                    Name(joint)
                ));
            }
        }
    }
}

/// The machine stood on every pose the document sent it to, and on the base
/// when the window had closed.
///
/// Read off the plant through the estimator, as every arrival in this suite is.
/// The head is asserted with each of them: the document drives the antennas
/// alone, so a head that moved under it is a composition that leaked into rows
/// no overlay spoke for.
fn check_arrival(run: &Run, failures: &mut Vec<String>) {
    for ((step, arc), (what, antennas)) in
        step_cycles().into_iter().zip(step_arcs()).zip(held_poses())
    {
        check::arrived_at(
            run,
            what,
            arrived_after(step, arc),
            &held_targets(antennas),
            failures,
        );
    }
    check::arrived_at(
        run,
        "upright with the window closed",
        window_close_cycle(),
        &held_targets(reachy_motion::postures::NEUTRAL_ANTENNAS),
        failures,
    );
}
