//! S16's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What a healthy run shares with every other scenario is
//! asserted by `scenario::check`: one engagement across five scripts, the goal
//! stream whole from the raise to the release -- with the rows the clips drive
//! exempt from the planner's own per-cycle bound over the windows, seams
//! included, because a clip carries no ceiling -- and nothing faulted.
//!
//! What is S16's own is [`check_restarts`]: on the one sample each replacement
//! takes effect on, every row moves by no more than a re-anchored base's own
//! planned step, and on the sample before it the antennas stand well off the
//! posture, so the seam had a contribution to drop. Every other step in the run
//! is the clips' and is judged by nothing here.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use reachy_motion::joints::row;
use scenario::check;
use scenario::cycle_at;
use scenario::read::Run;

use s16_scenario::{
    CLOSING_SCRIPT_ID, MOTIONS, OPENING_SCRIPT_ID, REPLACEMENT_SCRIPT_IDS, closing_cycle,
    content_rows, disengage_cycle, end_cycle, last_close_cycle, opening_window_open_cycle,
    replacement_cycle, replacement_sent_ns, restart_cycle, script_sent_cycle, standing_cycle,
};

/// The least the antennas stand off the posture on the sample before each
/// restart, radians.
///
/// Each seam is placed where the outgoing clip carries the antennas well away
/// from the posture; a seam with nothing standing there would pass the restart
/// check whatever the mover did with it.
const SEAM_MIN_RAD: f64 = 0.1;

fn main() -> ExitCode {
    check::main("s16_checker", |run, failures| {
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        let mut sent = vec![(OPENING_SCRIPT_ID, cycle_at(script_sent_cycle()))];
        sent.extend(
            REPLACEMENT_SCRIPT_IDS
                .into_iter()
                .enumerate()
                .map(|(n, script_id)| (script_id, replacement_sent_ns(n))),
        );
        sent.push((CLOSING_SCRIPT_ID, cycle_at(closing_cycle())));
        check::scripts_sent_at(run, &sent, failures);
        // One engagement across all five scripts: a replacement that disarmed
        // and re-engaged would carry more phase changes than this.
        let (engaged, _) = check::ordinary_life(run, &[], failures);
        check::ended_promptly(
            engaged.map(|engaged| engaged.released),
            disengage_cycle(),
            failures,
        );
        if let (Some(stream), Some(engaged)) = (
            check::goal_stream_playing(
                run,
                content_rows(),
                opening_window_open_cycle()..last_close_cycle(),
                failures,
            ),
            engaged,
        ) {
            check::stream_starts_with_session(&stream, engaged.taken, failures);
            check::stream_stops_with_release(&stream, engaged.released, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        let answered = check::replacements(
            run,
            &[
                (REPLACEMENT_SCRIPT_IDS[0], replacement_cycle(0), 1),
                (REPLACEMENT_SCRIPT_IDS[1], replacement_cycle(1), 2),
                (REPLACEMENT_SCRIPT_IDS[2], replacement_cycle(2), 3),
                (CLOSING_SCRIPT_ID, closing_cycle(), 4),
            ],
            failures,
        );
        let replaced: [Option<i64>; 3] =
            std::array::from_fn(|n| answered.get(n).copied().flatten());
        check_restarts(run, &replaced, failures);
        check::no_faults(run, failures);
        check::no_events(run, failures);
        let mut narration = vec![
            ReportKindWire::PHASE_CHANGED,
            ReportKindWire::SCRIPT_ACCEPTED,
            ReportKindWire::PHASE_CHANGED,
            ReportKindWire::PHASE_CHANGED,
            ReportKindWire::SCHEDULE_PUBLISHED,
        ];
        for _ in 0..4 {
            narration.push(ReportKindWire::SCRIPT_REPLACED);
            narration.push(ReportKindWire::SCHEDULE_PUBLISHED);
        }
        narration.extend([
            ReportKindWire::PHASE_CHANGED,
            ReportKindWire::SCHEDULE_PUBLISHED,
            ReportKindWire::SESSION_ENDED,
            ReportKindWire::PHASE_CHANGED,
        ]);
        check::narration(run, &narration, failures);
        check::signal_groups(run, failures);
    })
}

/// The one sample each replacement takes effect on moves no row further than a
/// re-anchored base's own planned step, and the seam had something to drop.
///
/// The outgoing row is taken over by a fresh join that enters at weight zero,
/// so a mover that re-anchors on it commands, on that sample, the setpoint it
/// last commanded plus the base's first step toward the posture. One that did
/// not would command the bare base, which is the whole outgoing contribution in
/// one period.
fn check_restarts(run: &Run, replaced: &[Option<i64>; 3], failures: &mut Vec<String>) {
    let Some(standing) = check::goal_at_or(run, standing_cycle(), "standing upright", failures)
    else {
        return;
    };
    for (n, answered) in replaced.iter().enumerate() {
        let restart = restart_cycle(n);
        let seam = format!("replacement {} ({}→{})", n + 1, MOTIONS[n], MOTIONS[n + 1]);
        if let Some(answered) = *answered
            && restart <= answered
        {
            failures.push(format!(
                "{seam}: the restart is expected on cycle {restart}, and the session answered the \
                 replacement on cycle {answered}: the effect cannot precede the answer"
            ));
            continue;
        }
        let (Some(before), Some(at)) = (
            check::goal_at_or(run, restart - 1, "just before a replacement", failures),
            check::goal_at_or(run, restart, "on a replacement's first sample", failures),
        ) else {
            continue;
        };
        let mut wrong = Vec::new();
        for i in 0..at.len() {
            let step = (at[i] - before[i]).abs();
            if step > check::CONTINUITY_STEP_RAD {
                wrong.push(format!(
                    "{seam}: row {i} stepped {step} rad on cycle {restart}, the first sample under \
                     the replacing schedule; a take-over re-anchors, so that sample moves by the \
                     base's planned step alone"
                ));
            }
        }
        for (joint, _) in check::ANTENNAS {
            let Some(i) = row(joint) else {
                wrong.push(format!("{seam}: this build has no {joint:?} row"));
                continue;
            };
            let off = (before[i] - standing[i]).abs();
            if off < SEAM_MIN_RAD {
                wrong.push(format!(
                    "{seam}: on cycle {} {joint:?} stands {off} rad off the posture, under the \
                     {SEAM_MIN_RAD} rad this seam is placed to carry: the seam had nothing to drop",
                    restart - 1
                ));
            }
        }
        if !wrong.is_empty() {
            let diagnostic = largest_steps(run, replacement_cycle(n));
            for failure in wrong {
                failures.push(format!("{failure} [{diagnostic}]"));
            }
        }
    }
}

/// The largest one-cycle row step on each of the six cycles after `sent`, and
/// the row it is on: where the replacement actually took effect, for a failure
/// to name. Judges nothing.
fn largest_steps(run: &Run, sent: i64) -> String {
    let steps: Vec<String> = ((sent + 1)..=(sent + 6))
        .map(
            |cycle| match (check::goal_at(run, cycle - 1), check::goal_at(run, cycle)) {
                (Some(before), Some(after)) => {
                    let (i, step) = (0..after.len())
                        .map(|i| (i, (after[i] - before[i]).abs()))
                        .fold(
                            (0, 0.0),
                            |best, next| if next.1 > best.1 { next } else { best },
                        );
                    format!("cycle {cycle}: row {i} {step:.4}")
                }
                _ => format!("cycle {cycle}: no goal"),
            },
        )
        .collect();
    format!("largest steps after the send: {}", steps.join(", "))
}
