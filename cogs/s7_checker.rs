//! S7's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What a healthy run shares with every other scenario is
//! asserted by `scenario::check`, and one of those shared assertions is doing
//! most of the work here: `check::goal_stream` demands one datagram per sample
//! of the whole engagement, each within a cycle's travel of the one before, so
//! a replacement that gapped the stream or jumped the machine fails there
//! without this file saying anything.
//!
//! What is here is what makes the run a presence session rather than three
//! gestures. One engagement across three scripts and four schedules; the
//! epochs strictly increasing, one per accepted script, with the row the
//! session narrated naming the epoch that went out; torque untouched between
//! them; the layer the replacement asked for actually playing; and the
//! duplicate refused `stale` having moved nothing.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__cogs__schedule_clk_rs::{PostureWire, StepKindWire};
use brenn_reachy__cogs__session_cmd_clk_rs::SessionCmdKindWire;
use brenn_reachy__hardware__dynamixel__registers_clk_rs::RegIdWire;
use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKindWire;
use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKindWire};
use reachy_motion::joints::{JointRef, row};
use reachy_motion::postures::{neutral_targets, stow_pose_targets};
use scenario::check;
use scenario::read::Run;
use scenario::{cycle_at, cycle_within, drain_cycle, stow_clocks};

use s7_scenario::{
    CLOSING_SCRIPT_ID, CLOSING_STOW_CYCLES, HOLD_RAD, HOLD_SCRIPT_ID, REFRESH_SCRIPT_ID,
    closing_cycle, disengage_cycle, duplicate_cycle, end_cycle, motion_hold_from_cycle,
    motion_hold_through_cycle, refresh_cycle, script_sent_cycle, standing_cycle, stow_start_cycle,
};

/// How far a goal may be from the number this scenario derives for it, radians.
///
/// The composition is arithmetic on numbers the documents state, so what this
/// leaves room for is the base's own solve and nothing else.
const DERIVED_TOLERANCE: f64 = 1e-3;

fn main() -> ExitCode {
    check::main("s7_checker", |run, failures| {
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        check::scripts_sent(
            run,
            &[
                (HOLD_SCRIPT_ID, script_sent_cycle()),
                (REFRESH_SCRIPT_ID, refresh_cycle()),
                (REFRESH_SCRIPT_ID, duplicate_cycle()),
                (CLOSING_SCRIPT_ID, closing_cycle()),
            ],
            failures,
        );
        // One engagement and no more, across all three accepted scripts. The
        // phase sequence is the ordinary one: a session that had disarmed and
        // re-engaged for a refresh would carry four more changes here.
        let (engaged, _) = check::ordinary_life(run, &[], failures);
        let bracket = engaged.map(|engaged| (engaged.taken, engaged.released));
        check::ended_promptly(
            engaged.map(|engaged| engaged.released),
            disengage_cycle(),
            failures,
        );
        // One stretch of goal stream, spanning the whole engagement: the count
        // is the assertion that nothing stopped commanding between the
        // replacements, and everything within the stretch is the assertion that
        // no epoch change jumped the machine.
        if let (Some(stream), Some((taken, released))) =
            (check::goal_stream(run, failures), bracket)
        {
            check::stream_starts_with_session(&stream, taken, failures);
            check::stream_stops_with_release(&stream, released, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        check_schedules(run, engaged, failures);
        check_replacements(run, failures);
        check_torque_untouched(run, bracket, failures);
        check_presence(run, failures);
        check_overlay_played(run, failures);
        check_arrival(run, failures);
        // The duplicate, and nothing else refused: the machine took every
        // script a presence sender meant it to take.
        check::refusals(
            run,
            &[(
                REFRESH_SCRIPT_ID,
                RefusalReasonWire::STALE,
                duplicate_cycle(),
            )],
            failures,
        );
        check::no_faults(run, failures);
        // The load-bearing negative: the goal stream never stops while the
        // session holds, so the driver's dead-man has nothing to measure. An
        // event here is a session that let its stream lapse across a
        // replacement.
        check::no_events(run, failures);
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
                ReportKindWire::SCRIPT_REFUSED,
                ReportKindWire::SCRIPT_REPLACED,
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

/// The four schedules the session published, and what each of them says.
///
/// The engagement, one per replacement, and the one nobody is running. Each
/// replacement is *whole*: the refresh's schedule is its own step and its own
/// window, and the closing one's is its two steps and no window at all, rather
/// than either merged with what was running.
fn check_schedules(run: &Run, engaged: Option<check::Engaged>, failures: &mut Vec<String>) {
    let published = check::schedules_under_one_engagement(
        run,
        &["the arming", "the refresh", "the closing script"],
        check::Tail::Nothing,
        engaged,
        failures,
    );
    if published.is_empty() {
        return;
    }
    let shapes: Vec<(usize, usize)> = run
        .schedules
        .iter()
        .map(|logged| {
            (
                logged.message.steps().len(),
                logged.message.overlays().len(),
            )
        })
        .collect();
    let [_, refreshed, closing, _] = shapes.as_slice() else {
        return;
    };
    // The replacements are whole, which is what the schedules' shapes say: the
    // refresh's window is in the refresh's schedule and gone from the closing
    // one, rather than carried forward into it.
    if *refreshed != (1, 1) {
        failures.push(format!(
            "the refresh's schedule carries {} steps and {} windows, and the refresh asks for one \
             of each: a replacement is the whole commanded future",
            refreshed.0, refreshed.1
        ));
    }
    if *closing != (2, 0) {
        failures.push(format!(
            "the closing schedule carries {} steps and {} windows, and the closing script asks \
             for two steps and no window: a window the replacement did not ask for is one the \
             session merged rather than replaced",
            closing.0, closing.1
        ));
    }
    check_closing_steps(run, failures);
}

/// The closing script's schedule is the timeline it asked for: up until the
/// fold begins, then the fold, at the instants the scenario named.
///
/// The absolute instants and not just the count, because a replacement is
/// planned from the sender's own stamp: a schedule whose steps were re-anchored
/// on the session's clock would carry the right shape at the wrong times, and
/// every arrival assertion below would still pass on a machine that folded
/// early or late.
fn check_closing_steps(run: &Run, failures: &mut Vec<String>) {
    let Some(logged) = run.schedules.get(2) else {
        return;
    };
    let found: Vec<(StepKindWire, PostureWire, i64, i64)> = logged
        .message
        .steps()
        .iter()
        .map(|step| {
            (
                step.kind(),
                step.posture(),
                step.start().as_nanos(),
                step.end().as_nanos(),
            )
        })
        .collect();
    let wanted = [
        (
            StepKindWire::BASE_POSTURE,
            PostureWire::UP,
            cycle_at(closing_cycle()),
            cycle_at(stow_start_cycle()),
        ),
        (
            StepKindWire::BASE_POSTURE,
            PostureWire::STOW,
            cycle_at(stow_start_cycle()),
            cycle_at(disengage_cycle()),
        ),
    ];
    if found.as_slice() != wanted.as_slice() {
        failures.push(format!(
            "the closing schedule is {found:?}, and the closing script asks for {wanted:?}"
        ));
    }
}

/// What the session said about each replacement: the script's own number, the
/// epoch it was written under, and the wake it was decided on.
///
/// The epoch is the join. A row naming an epoch other than the one that went
/// out on the channel would leave an operator reading the timeline against a
/// mover that answered a different number, which is the whole use the row has.
fn check_replacements(run: &Run, failures: &mut Vec<String>) {
    let replaced: Vec<(i64, u32, u32)> = run
        .reports
        .iter()
        .filter(|report| report.message.kind() == ReportKindWire::SCRIPT_REPLACED)
        .map(|report| {
            (
                cycle_within(report.message.time().as_nanos()),
                report.message.a(),
                report.message.b(),
            )
        })
        .collect();
    let expected = [
        (REFRESH_SCRIPT_ID, refresh_cycle(), 1_usize),
        (CLOSING_SCRIPT_ID, closing_cycle(), 2_usize),
    ];
    if replaced.len() != expected.len() {
        failures.push(format!(
            "the session narrated {replaced:?} as replacements, and this run replaces the running \
             schedule {} times",
            expected.len()
        ));
        return;
    }
    for ((at, script_id, epoch), (wanted_id, sent_on, index)) in replaced.iter().zip(expected) {
        if *script_id != wanted_id {
            failures.push(format!(
                "the session replaced its schedule on script {script_id}, and this run sends \
                 {wanted_id}"
            ));
        }
        check::answered_on_its_wake("replacement", *at, sent_on, failures);
        match run.schedules.get(index) {
            Some(logged) if logged.message.epoch() == *epoch => {}
            Some(logged) => failures.push(format!(
                "the session narrated the replacement under epoch {epoch} and published epoch {} \
                 at cycle {}: the row and the channel name one schedule",
                logged.message.epoch(),
                cycle_within(logged.at_ns)
            )),
            None => {}
        }
    }
}

/// Nothing wrote torque between the arming and the release.
///
/// The claim the whole scenario exists for. A session that answered a refresh
/// by letting go and taking hold again would show it here twice over: a
/// torque-off command inside the engagement, and the arming sweep's
/// torque-enable writes after it. Both are read off the datagrams the session
/// actually published, which is the only place the machine hears about either.
fn check_torque_untouched(run: &Run, engaged: Option<(i64, i64)>, failures: &mut Vec<String>) {
    let Some((taken, released)) = engaged else {
        return;
    };
    for datagram in &run.datagrams {
        let at = drain_cycle(datagram.at_ns);
        if at <= taken || at >= released {
            continue;
        }
        if datagram.message.kind() == SessionCmdKindWire::TORQUE_OFF_NOW {
            failures.push(format!(
                "the session told the driver to let go at cycle {at}, inside an engagement that \
                 ran from {taken} to {released}: a refresh is not a reason to de-torque the head"
            ));
            continue;
        }
        let txn = datagram.message.txn();
        if datagram.message.kind() == SessionCmdKindWire::AUX
            && txn.reg() == RegIdWire::TORQUE_ENABLE
            && txn.op() != AuxOpKindWire::READ_REG
        {
            failures.push(format!(
                "the session wrote servo {}'s torque enable at cycle {at}, inside an engagement \
                 that ran from {taken} to {released}: nothing between two replacements arms or \
                 disarms anything",
                txn.id()
            ));
        }
    }
}

/// The head is up and stays up across the replacements: the presence the sender
/// asked for is what the machine held.
///
/// Three instants, one per schedule the engagement ran under, each read off the
/// plant rather than off what was commanded: before the refresh, inside the
/// window's aftermath, and on the closing script's last beat up. A session that
/// dipped the head between schedules -- through a stow it planned itself, or
/// through a base that lost its posture at an epoch change -- fails the middle
/// one while the ends pass.
fn check_presence(run: &Run, failures: &mut Vec<String>) {
    for (what, cycle) in [
        ("before the refresh", standing_cycle()),
        ("after the motion played", stow_start_cycle() - 5),
        ("on the closing script's last beat", stow_start_cycle() - 1),
    ] {
        check::arrived_at(
            run,
            &format!("upright {what}"),
            cycle,
            &neutral_targets(),
            failures,
        );
    }
}

/// The replacement's own overlay window played.
///
/// The motion holds its first segment's last frame through the gap between its
/// segments, so what the goal stream carries there is the posture the base is
/// standing in plus a delta the document states and the window's gain scales.
/// A session that swapped the steps and dropped the windows would leave the
/// antennas exactly where the posture puts them, and this is what notices.
fn check_overlay_played(run: &Run, failures: &mut Vec<String>) {
    let (Some(right), Some(left)) = (row(JointRef::AntennaRight), row(JointRef::AntennaLeft))
    else {
        failures.push("this build has no antenna row".to_owned());
        return;
    };
    let Some(standing) = check::goal_at_or(run, standing_cycle(), "standing upright", failures)
    else {
        return;
    };
    // One report for the whole hold: it is the same statement about every cycle
    // of it, and a run that broke it broke it for all of them.
    let told = failures.len();
    for cycle in motion_hold_from_cycle()..=motion_hold_through_cycle() {
        if failures.len() > told {
            break;
        }
        let Some(held) = check::goal_at_or(run, cycle, "in the motion's hold", failures) else {
            return;
        };
        for (antenna, at) in [
            (JointRef::AntennaRight, right),
            (JointRef::AntennaLeft, left),
        ] {
            let parted = held[at] - standing[at];
            if (parted.abs() - HOLD_RAD).abs() > DERIVED_TOLERANCE {
                failures.push(format!(
                    "at cycle {cycle} {antenna:?} is held {parted} rad off the posture, and the \
                     motion the replacement asked for holds it {HOLD_RAD} rad off at this \
                     window's gain"
                ));
                break;
            }
        }
    }
}

/// The machine ends where the closing script sends it, and the fold has room.
fn check_arrival(run: &Run, failures: &mut Vec<String>) {
    check::arrived_at(
        run,
        "stowed",
        disengage_cycle() - 1,
        &stow_pose_targets(),
        failures,
    );
    check::room("stow", CLOSING_STOW_CYCLES, &stow_clocks(), failures);
}
