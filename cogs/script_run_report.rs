//! Judge one supplied motion script against the log and names used to compile it.

use std::path::PathBuf;
use std::process::ExitCode;

use brenn_reachy__cogs__schedule_clk_rs::StepKindWire;
use brenn_reachy__cogs__script_clk_rs::ScriptWire;
use motion_run_report::{
    SettleResult, Span, every_window_moved, overlay_spans, prepare, settle, stillness,
    the_stream_held, whole_stream_measurements, window_measurements, windows,
};
use pose_reading::{RunConfig, no_faults};
use reachy_edge::{compile::MAX_STEPS, parse};
use run_report::{Report, verdict};

fn continuity_spans(
    run: &motion_run_report::Run,
    planned: &[motion_run_report::Window],
) -> Vec<Span> {
    let mut continuity = overlay_spans(planned);
    for schedule in &run.schedules {
        for step in schedule.message.steps().iter() {
            if step.kind() == StepKindWire::BASE_POSTURE {
                continuity.push(Span {
                    start_ns: step.start().as_nanos(),
                    end_ns: step.end().as_nanos(),
                });
            }
        }
    }
    continuity
}

fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let Some((log_dir, sidecar, strict)) = parse_args(&args) else {
        eprintln!("usage: script_run_report <log-dir> <names.json> [--settle-evidence]");
        return ExitCode::FAILURE;
    };
    let config = match RunConfig::read(&PathBuf::from(log_dir)) {
        Ok(config) => config,
        Err(error) => {
            eprintln!("reading the configuration this run was performed under: {error}");
            return ExitCode::FAILURE;
        }
    };
    let text = match std::fs::read_to_string(sidecar) {
        Ok(text) => text,
        Err(error) => {
            eprintln!("reading the names sidecar {sidecar}: {error}");
            return ExitCode::FAILURE;
        }
    };
    let (motions, poses) = match parse(&text) {
        Ok(tables) => tables,
        Err(error) => {
            eprintln!("the names sidecar {sidecar} is not one: {error}");
            return ExitCode::FAILURE;
        }
    };
    let run = match motion_run_report::read(&PathBuf::from(log_dir)) {
        Ok(run) => run,
        Err(error) => {
            eprintln!("reading the log under {log_dir}: {error}");
            return ExitCode::FAILURE;
        }
    };
    let mut report = Report::default();
    config.configuration(&mut report);
    for complaint in &run.complaints {
        report.fail(complaint.clone());
    }
    no_faults(&run.faults, &mut report);
    if run.scripts.len() != 1 {
        report.fail(format!(
            "the incoming stream carries {} scripts, expected exactly one",
            run.scripts.len()
        ));
    }
    if let Some(script) = run.scripts.first() {
        compare_request(script, &run, &motions, &poses, &mut report);
        if run.samples.is_empty() {
            report.fail(
                "the log holds no driver samples, so there is no record of what the machine did"
                    .to_owned(),
            );
        } else {
            let planned = windows(&run);
            let by_id: std::collections::BTreeMap<u16, String> = motions
                .entries()
                .map(|(name, entry)| (entry.motion_id, name.to_owned()))
                .collect();
            let prepared = match prepare(&run, &config) {
                Ok(prepared) => prepared,
                Err(error) => {
                    report.fail(error);
                    return verdict(
                        "script_run_report",
                        log_dir,
                        &report,
                        if strict {
                            "every nonterminal base move supplied settle evidence"
                        } else {
                            "the supplied script ran whole"
                        },
                    );
                }
            };
            every_window_moved(&prepared.ordered, &planned, &by_id, &mut report);
            let continuity = continuity_spans(&run, &planned);
            the_stream_held(
                &prepared.ordered,
                &continuity,
                prepared.grid,
                &prepared.skips,
                &mut report,
                "the script",
            );
            window_measurements(&prepared, &planned, &by_id, &mut report);
            whole_stream_measurements(&prepared, &run, &config, &mut report);
            stillness(
                &prepared.ordered,
                stillness_report::Standard::JudgedWhereHeadStill,
                &mut report,
            );
            let result = settle(&run, &prepared.ordered, &planned, &config, &mut report);
            if strict {
                strict_settle(
                    &script.message,
                    &result,
                    poses.stow().pose_id,
                    &poses,
                    &mut report,
                );
            }
        }
    }
    if strict {
        report.note("strict settle-evidence mode".to_owned());
    }
    verdict(
        "script_run_report",
        log_dir,
        &report,
        if strict {
            "every nonterminal base move supplied settle evidence"
        } else {
            "the supplied script ran whole"
        },
    )
}

fn parse_args(args: &[String]) -> Option<(&str, &str, bool)> {
    match args {
        [log_dir, sidecar] => Some((log_dir, sidecar, false)),
        [log_dir, sidecar, flag] if flag == "--settle-evidence" => Some((log_dir, sidecar, true)),
        _ => None,
    }
}

fn compare_request(
    script: &log_read::Logged<ScriptWire>,
    run: &motion_run_report::Run,
    motions: &reachy_edge::MotionTable,
    poses: &reachy_edge::PoseTable,
    report: &mut Report,
) {
    let request = &script.message;
    let arrival = request.arrival().as_nanos();
    let mut expected_steps = std::collections::BTreeMap::new();
    for step in request.steps().iter() {
        if step.kind() == StepKindWire::BASE_POSTURE {
            let start = arrival + i64::from(step.after_ms()) * 1_000_000;
            let end = start + i64::from(step.duration_ms()) * 1_000_000;
            let pace = i64::from(step.move_ms()) * 1_000_000;
            *expected_steps
                .entry((start, end, step.pose_id(), pace))
                .or_insert(0usize) += 1;
        }
    }
    let mut actual_steps = std::collections::BTreeMap::new();
    let mut schedule_states = std::collections::BTreeSet::new();
    for schedule in &run.schedules {
        let mut state = Vec::new();
        for step in schedule.message.steps().iter() {
            if step.kind() == StepKindWire::BASE_POSTURE {
                state.push((
                    step.start().as_nanos(),
                    step.end().as_nanos(),
                    step.pose_id(),
                    step.pace().as_nanos(),
                ));
            }
        }
        if schedule_states.insert(state.clone()) {
            for key in state {
                *actual_steps.entry(key).or_insert(0usize) += 1;
            }
        }
    }
    for (key, expected_count) in &expected_steps {
        let actual_count = actual_steps.get(key).copied().unwrap_or(0);
        if actual_count < *expected_count {
            if actual_steps.keys().any(|other| {
                other.0 == key.0 && other.1 == key.1 && other.2 == key.2 && other.3 != key.3
            }) {
                report.fail(format!(
                    "planned base row {} at {}..{} has the wrong pace",
                    poses_label(key.2, poses),
                    key.0,
                    key.1
                ));
            } else if actual_steps.keys().any(|other| {
                (other.0 != key.0 || other.1 != key.1) && other.2 == key.2 && other.3 == key.3
            }) {
                report.fail(format!(
                    "planned base row {} has the wrong time",
                    poses_label(key.2, poses)
                ));
            } else if actual_steps.keys().any(|other| {
                other.0 == key.0 && other.1 == key.1 && other.3 == key.3 && other.2 != key.2
            }) {
                report.fail(format!(
                    "planned base row {} at {}..{} has the wrong pose id",
                    poses_label(key.2, poses),
                    key.0,
                    key.1
                ));
            } else {
                report.fail(format!(
                    "planned base row {} at {}..{} is missing",
                    poses_label(key.2, poses),
                    key.0,
                    key.1
                ));
            }
        } else if actual_count > *expected_count {
            report.fail(format!(
                "planned base row {} at {}..{} is duplicated",
                poses_label(key.2, poses),
                key.0,
                key.1
            ));
        }
    }
    for key in actual_steps.keys() {
        if !expected_steps.contains_key(key) {
            report.fail(format!(
                "schedule carries an extra base row at {}..{}",
                key.0, key.1
            ));
        }
    }
    for window in request.overlays().iter() {
        if motions
            .entries()
            .all(|(_, entry)| entry.motion_id != window.motion_id())
        {
            report.fail(format!(
                "script requests motion {} which the run sidecar does not name",
                window.motion_id()
            ));
        }
    }
    let mut expected_overlays = std::collections::BTreeMap::new();
    for window in request.overlays().iter() {
        let start = arrival + i64::from(window.after_ms()) * 1_000_000;
        let end = start + i64::from(window.duration_ms()) * 1_000_000;
        *expected_overlays
            .entry((
                start,
                end,
                window.motion_id(),
                window.gain().to_bits(),
                window.speed().to_bits(),
            ))
            .or_insert(0usize) += 1;
    }
    let mut actual_overlays = std::collections::BTreeMap::new();
    let mut overlay_states = std::collections::BTreeSet::new();
    for schedule in &run.schedules {
        let mut state = Vec::new();
        for window in schedule.message.overlays().iter() {
            state.push((
                window.start().as_nanos(),
                window.end().as_nanos(),
                window.motion_id(),
                window.gain().to_bits(),
                window.speed().to_bits(),
            ));
        }
        if overlay_states.insert(state.clone()) {
            for key in state {
                *actual_overlays.entry(key).or_insert(0usize) += 1;
            }
        }
    }
    for (key, expected_count) in &expected_overlays {
        let actual_count = actual_overlays.get(key).copied().unwrap_or(0);
        if actual_count < *expected_count {
            if actual_overlays.keys().any(|other| {
                other.0 == key.0 && other.1 == key.1 && other.2 == key.2 && other.3 != key.3
            }) {
                report.fail(format!(
                    "planned overlay {} at {}..{} has the wrong gain",
                    key.2, key.0, key.1
                ));
            } else if actual_overlays.keys().any(|other| {
                other.0 == key.0
                    && other.1 == key.1
                    && other.2 == key.2
                    && other.3 == key.3
                    && other.4 != key.4
            }) {
                report.fail(format!(
                    "planned overlay {} at {}..{} has the wrong speed",
                    key.2, key.0, key.1
                ));
            } else if actual_overlays.keys().any(|other| {
                (other.0 != key.0 || other.1 != key.1)
                    && other.2 == key.2
                    && other.3 == key.3
                    && other.4 == key.4
            }) {
                report.fail(format!("planned overlay {} has the wrong time", key.2));
            } else {
                report.fail(format!(
                    "planned overlay {} at {}..{} is missing",
                    key.2, key.0, key.1
                ));
            }
        } else if actual_count > *expected_count {
            report.fail(format!(
                "planned overlay {} at {}..{} is duplicated",
                key.2, key.0, key.1
            ));
        }
    }
    for key in actual_overlays
        .keys()
        .filter(|key| !expected_overlays.contains_key(key))
    {
        report.fail(format!(
            "schedule carries an extra overlay {} at {}..{}",
            key.2, key.0, key.1
        ));
    }
    if request.steps().is_empty() || request.steps().len() > MAX_STEPS {
        report.fail("the supplied request has an invalid base-row count".to_owned());
    }
    if request.arrival().as_nanos() == 0 {
        report.fail("the supplied request has no arrival instant".to_owned());
    }
}

fn poses_label(id: u16, poses: &reachy_edge::PoseTable) -> String {
    poses
        .entries()
        .find(|(_, entry)| entry.pose_id == id)
        .map_or_else(
            || format!("pose {id}"),
            |(name, _)| format!("{name} ({id})"),
        )
}

fn strict_settle(
    script: &ScriptWire,
    result: &SettleResult,
    stow_id: u16,
    poses: &reachy_edge::PoseTable,
    report: &mut Report,
) {
    let rows = script
        .steps()
        .iter()
        .filter(|step| step.kind() == StepKindWire::BASE_POSTURE)
        .count();
    if result.moves.len() != rows {
        report.fail(format!(
            "settle measured {} scheduled base moves, but the script carries {rows}",
            result.moves.len()
        ));
    }
    let skipped: Vec<_> = result.moves.iter().filter(|item| !item.measured).collect();
    let terminal = result.moves.last();
    if skipped.len() != 1
        || terminal.is_none()
        || skipped[0].pose_id != stow_id
        || terminal != skipped.first().copied()
    {
        report.fail(
            "strict settle evidence requires the terminal stow to be the sole skipped move"
                .to_owned(),
        );
    }
    for item in result.moves.iter().filter(|item| !item.measured) {
        if terminal == Some(item) {
            continue;
        }
        let name = poses
            .entries()
            .find(|(_, entry)| entry.pose_id == item.pose_id)
            .map(|(name, _)| name)
            .unwrap_or("unknown");
        report.fail(format!(
            "pose {name} ({}) at {} lacks settle evidence: {}",
            item.pose_id,
            item.start_ns,
            item.reason.as_deref().unwrap_or("unknown reason")
        ));
    }
    if result.moves.iter().filter(|item| item.measured).count() != rows.saturating_sub(1) {
        report.fail(
            "strict settle evidence measured count does not match the nonterminal rows".to_owned(),
        );
    }
    if result.maxima.iter().any(Option::is_none) {
        report.fail("strict settle evidence has an unavailable leg maximum".to_owned());
    }
}

#[cfg(test)]
mod tests {
    use super::{SettleResult, compare_request, continuity_spans, parse_args, strict_settle};
    use brenn_reachy__cogs__schedule_clk_rs::{
        OverlayWindowWire, ScheduledStepWire, SessionScheduleWire, StepKindWire,
    };
    use brenn_reachy__cogs__script_clk_rs::{ScriptOverlayWire, ScriptWire};
    use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
    use clockwork_rs::{Duration, SyncTime};
    use log_read::Logged;
    use motion_proto::PlayWindow;
    use motion_proto::STOW_POSE;
    use motion_run_report::settle;
    use motion_run_report::{Run, SETTLE_BOUND_COUNTS, SettleMove};
    use motion_run_report::{Span, every_window_moved, windows};
    use pose_reading::RunConfig;
    use reachy_edge::names::{MotionEntry, MotionTable, PoseEntry, PoseTable};
    use reachy_motion::arm::DEFAULT_GAINS;
    use reachy_motion::joints::{JointRef, ROW_COUNT, row, write_rows};
    use reachy_motion::plant::SHIPPED_PROFILES;
    use reachy_motion::stillness::COUNT_RAD;
    use run_report::Report;

    fn args(values: &[&str]) -> Vec<String> {
        values.iter().map(|value| (*value).to_owned()).collect()
    }

    #[test]
    fn accepts_default_script_report_shape() {
        assert_eq!(
            parse_args(&args(&["run", "names"])),
            Some(("run", "names", false))
        );
    }

    #[test]
    fn accepts_strict_script_report_shape() {
        assert_eq!(
            parse_args(&args(&["run", "names", "--settle-evidence"])),
            Some(("run", "names", true))
        );
    }

    #[test]
    fn refuses_unknown_flag() {
        assert!(parse_args(&args(&["run", "names", "--strict"])).is_none());
    }

    #[test]
    fn refuses_flag_before_sidecar() {
        assert!(parse_args(&args(&["run", "--settle-evidence", "names"])).is_none());
    }

    #[test]
    fn refuses_extra_argument() {
        assert!(parse_args(&args(&["run", "names", "--settle-evidence", "extra"])).is_none());
    }

    fn request(move_ms: u32) -> Logged<ScriptWire> {
        let mut message = ScriptWire::new();
        let arrival = SyncTime::from_nanos(1_700_000_000_000_000_000);
        message.set_arrival(arrival);
        let mut steps = message.steps_mut();
        let step = steps.try_grow().expect("one step");
        step.set_after_ms(1000);
        step.set_duration_ms(2000);
        step.set_move_ms(move_ms);
        step.set_kind(StepKindWire::BASE_POSTURE);
        step.set_pose_id(0);
        Logged {
            at_ns: arrival.as_nanos(),
            sequence_number: 1,
            message,
        }
    }

    fn schedule(pace_ms: u32) -> Logged<SessionScheduleWire> {
        let mut message = SessionScheduleWire::new();
        let mut steps = message.steps_mut();
        let step: &mut ScheduledStepWire = steps.try_grow().expect("one step");
        step.set_start(SyncTime::from_nanos(1_700_000_001_000_000_000));
        step.set_end(SyncTime::from_nanos(1_700_000_003_000_000_000));
        step.set_kind(StepKindWire::BASE_POSTURE);
        step.set_pose_id(0);
        step.set_pace(Duration::from_nanos(i64::from(pace_ms) * 1_000_000));
        Logged {
            at_ns: 1_700_000_001_000_000_000,
            sequence_number: 1,
            message,
        }
    }

    fn schedule_custom(
        pose: u16,
        start_ms: u64,
        pace_ms: u32,
        duplicate: bool,
    ) -> Logged<SessionScheduleWire> {
        let mut message = SessionScheduleWire::new();
        let mut steps = message.steps_mut();
        for _ in 0..if duplicate { 2 } else { 1 } {
            let step: &mut ScheduledStepWire = steps.try_grow().expect("step");
            step.set_start(SyncTime::from_nanos(
                1_700_000_000_000_000_000 + i64::try_from(start_ms).unwrap() * 1_000_000,
            ));
            step.set_end(SyncTime::from_nanos(
                1_700_000_000_000_000_000 + i64::try_from(start_ms + 2000).unwrap() * 1_000_000,
            ));
            step.set_kind(StepKindWire::BASE_POSTURE);
            step.set_pose_id(pose);
            step.set_pace(Duration::from_nanos(i64::from(pace_ms) * 1_000_000));
        }
        Logged {
            at_ns: 1_700_000_000_000_000_000,
            sequence_number: 1,
            message,
        }
    }

    fn schedule_custom_end(end_ms: u64) -> Logged<SessionScheduleWire> {
        let mut message = SessionScheduleWire::new();
        let mut steps = message.steps_mut();
        let step: &mut ScheduledStepWire = steps.try_grow().expect("one step");
        step.set_start(SyncTime::from_nanos(1_700_000_001_000_000_000));
        step.set_end(SyncTime::from_nanos(
            1_700_000_000_000_000_000 + i64::try_from(end_ms).unwrap() * 1_000_000,
        ));
        step.set_kind(StepKindWire::BASE_POSTURE);
        step.set_pose_id(0);
        step.set_pace(Duration::from_nanos(500_000_000));
        Logged {
            at_ns: 1_700_000_001_000_000_000,
            sequence_number: 1,
            message,
        }
    }

    fn pose_sample(time_ns: i64, present_valid: bool, commanded: f64) -> Logged<PoseSampleWire> {
        let mut message = PoseSampleWire::new();
        let read = message.clear_valid();
        read.nominal_time = SyncTime::from_nanos(time_ns);
        read.sample_time = SyncTime::from_nanos(time_ns);
        read.present_valid = present_valid.into();
        read.commanded_valid = true.into();
        let mut commanded_rows = [0.0; ROW_COUNT];
        commanded_rows[row(JointRef::Leg0).expect("leg row")] = commanded;
        write_rows(&mut read.present, &[0.0; ROW_COUNT]);
        write_rows(&mut read.commanded, &commanded_rows);
        Logged {
            at_ns: time_ns,
            sequence_number: 0,
            message,
        }
    }

    fn script_with_plays() -> Logged<ScriptWire> {
        let mut script = request(500);
        let mut overlays = script.message.overlays_mut();
        for (motion_id, after_ms) in [(7, 3000), (8, 3500)] {
            let row: &mut ScriptOverlayWire = overlays.try_grow().expect("two plays");
            row.set_motion_id(motion_id);
            row.set_after_ms(after_ms);
            row.set_duration_ms(400);
            row.set_gain(1.0);
            row.set_speed(1.0);
        }
        script
    }

    fn schedule_with_plays(gain: f64, speed: f64) -> Logged<SessionScheduleWire> {
        let mut schedule = schedule(500);
        let mut overlays = schedule.message.overlays_mut();
        for (motion_id, start_ms) in [(7, 3000), (8, 3500)] {
            let row: &mut OverlayWindowWire = overlays.try_grow().expect("two plays");
            row.set_motion_id(motion_id);
            row.set_start(SyncTime::from_nanos(
                1_700_000_000_000_000_000 + i64::from(start_ms) * 1_000_000,
            ));
            row.set_end(SyncTime::from_nanos(
                1_700_000_000_000_000_000 + i64::from(start_ms + 400) * 1_000_000,
            ));
            row.set_gain(gain);
            row.set_speed(speed);
        }
        schedule
    }

    fn script_with_one_overlay() -> Logged<ScriptWire> {
        let mut script = request(500);
        let mut overlays = script.message.overlays_mut();
        let row = overlays.try_grow().expect("one overlay");
        row.set_motion_id(7);
        row.set_after_ms(3000);
        row.set_duration_ms(400);
        row.set_gain(1.0);
        row.set_speed(1.0);
        script
    }

    fn schedule_with_overlay(
        start_ms: u64,
        end_ms: u64,
        duplicate: bool,
        extra: bool,
    ) -> Logged<SessionScheduleWire> {
        let mut schedule = schedule(500);
        let mut overlays = schedule.message.overlays_mut();
        for motion_id in [7].into_iter().chain(extra.then_some(8)) {
            for _ in 0..if duplicate { 2 } else { 1 } {
                let row: &mut OverlayWindowWire = overlays.try_grow().expect("overlay");
                row.set_motion_id(motion_id);
                row.set_start(SyncTime::from_nanos(
                    1_700_000_000_000_000_000 + i64::try_from(start_ms).unwrap() * 1_000_000,
                ));
                row.set_end(SyncTime::from_nanos(
                    1_700_000_000_000_000_000 + i64::try_from(end_ms).unwrap() * 1_000_000,
                ));
                row.set_gain(1.0);
                row.set_speed(1.0);
            }
        }
        schedule
    }

    #[test]
    fn continuity_covers_the_union_of_base_and_overlay_extents() {
        let run = Run {
            schedules: vec![schedule_with_plays(1.0, 1.0)],
            ..Run::default()
        };
        let planned = windows(&run);
        let spans = continuity_spans(&run, &planned);
        assert_eq!(
            spans.iter().map(|span| span.start_ns).min(),
            Some(1_700_000_001_000_000_000)
        );
        assert_eq!(
            spans.iter().map(|span| span.end_ns).max(),
            Some(1_700_000_003_900_000_000)
        );
        assert!(spans.iter().any(|span| {
            *span
                == Span {
                    start_ns: 1_700_000_001_000_000_000,
                    end_ns: 1_700_000_003_000_000_000,
                }
        }));
        assert!(spans.iter().any(|span| {
            *span
                == Span {
                    start_ns: 1_700_000_003_500_000_000,
                    end_ns: 1_700_000_003_900_000_000,
                }
        }));
    }

    fn tables() -> (MotionTable, PoseTable) {
        (
            MotionTable::of([]),
            PoseTable::of([
                (
                    "neutral".to_owned(),
                    PoseEntry {
                        pose_id: 0,
                        duration_ms: 1000,
                    },
                ),
                (
                    STOW_POSE.to_owned(),
                    PoseEntry {
                        pose_id: 2,
                        duration_ms: 1000,
                    },
                ),
            ])
            .expect("stow table"),
        )
    }

    fn play_tables() -> (MotionTable, PoseTable) {
        (
            MotionTable::of([
                (
                    "hello/a".to_owned(),
                    MotionEntry {
                        motion_id: 7,
                        window: PlayWindow {
                            duration_ms: 400,
                            blend_out_ms: 0,
                        },
                    },
                ),
                (
                    "hello/b".to_owned(),
                    MotionEntry {
                        motion_id: 8,
                        window: PlayWindow {
                            duration_ms: 400,
                            blend_out_ms: 0,
                        },
                    },
                ),
            ]),
            tables().1,
        )
    }

    #[test]
    fn exact_single_script_rows_are_accepted() {
        let (motions, poses) = tables();
        let script = request(500);
        let run = Run {
            schedules: vec![schedule(500)],
            ..Run::default()
        };
        let mut report = Report::default();
        compare_request(&script, &run, &motions, &poses, &mut report);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    #[test]
    fn wrong_pace_is_named_as_pace() {
        let (motions, poses) = tables();
        let script = request(500);
        let run = Run {
            schedules: vec![schedule(600)],
            ..Run::default()
        };
        let mut report = Report::default();
        compare_request(&script, &run, &motions, &poses, &mut report);
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("wrong pace"))
        );
    }

    #[test]
    fn prematurely_ended_base_row_is_named_as_wrong_time() {
        let (motions, poses) = tables();
        let mut report = Report::default();
        compare_request(
            &request(500),
            &Run {
                schedules: vec![schedule_custom_end(2500)],
                ..Run::default()
            },
            &motions,
            &poses,
            &mut report,
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("wrong time")),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn wrong_overlay_gain_is_named_as_gain() {
        let (motions, poses) = play_tables();
        let mut report = Report::default();
        compare_request(
            &script_with_plays(),
            &Run {
                schedules: vec![schedule_with_plays(0.5, 1.0)],
                ..Run::default()
            },
            &motions,
            &poses,
            &mut report,
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("wrong gain")),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn wrong_overlay_speed_is_named_as_speed() {
        let (motions, poses) = play_tables();
        let mut report = Report::default();
        compare_request(
            &script_with_plays(),
            &Run {
                schedules: vec![schedule_with_plays(1.0, 0.5)],
                ..Run::default()
            },
            &motions,
            &poses,
            &mut report,
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("wrong speed")),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn zero_play_settle_and_two_play_greeting_shapes_are_accepted() {
        let (motions, poses) = play_tables();
        let mut settle_report = Report::default();
        compare_request(
            &request(500),
            &Run {
                schedules: vec![schedule(500)],
                ..Run::default()
            },
            &motions,
            &poses,
            &mut settle_report,
        );
        assert!(
            settle_report.findings.is_empty(),
            "{:?}",
            settle_report.findings
        );

        let mut greeting_report = Report::default();
        compare_request(
            &script_with_plays(),
            &Run {
                schedules: vec![schedule_with_plays(1.0, 1.0)],
                ..Run::default()
            },
            &motions,
            &poses,
            &mut greeting_report,
        );
        assert!(
            greeting_report.findings.is_empty(),
            "{:?}",
            greeting_report.findings
        );
    }

    #[test]
    fn zero_play_script_window_measurement_accepts_no_overlay() {
        let run = Run {
            schedules: vec![schedule(500)],
            ..Run::default()
        };
        let planned = windows(&run);
        assert!(planned.is_empty());
        let mut report = Report::default();
        every_window_moved(
            &run.ordered_samples(),
            &planned,
            &std::collections::BTreeMap::new(),
            &mut report,
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    #[test]
    fn unrelated_sidecar_motions_do_not_create_demands() {
        let (empty, poses) = tables();
        let motions = MotionTable::of([(
            "unrequested".to_owned(),
            MotionEntry {
                motion_id: 99,
                window: PlayWindow {
                    duration_ms: 400,
                    blend_out_ms: 0,
                },
            },
        )]);
        assert!(empty.entries().next().is_none());
        let mut report = Report::default();
        compare_request(
            &request(500),
            &Run {
                schedules: vec![schedule(500)],
                ..Run::default()
            },
            &motions,
            &poses,
            &mut report,
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    #[test]
    fn request_matching_names_missing_extra_duplicate_pose_and_time_rows() {
        let (motions, poses) = tables();
        let script = request(500);
        for (label, schedules, needle) in [
            ("missing", Vec::new(), "is missing"),
            (
                "duplicate",
                vec![schedule_custom(0, 1000, 500, true)],
                "duplicated",
            ),
            (
                "extra",
                vec![schedule(500), schedule_custom(2, 3000, 500, false)],
                "extra base row",
            ),
            (
                "wrong pose",
                vec![schedule_custom(1, 1000, 500, false)],
                "wrong pose id",
            ),
            (
                "wrong time",
                vec![schedule_custom(0, 2000, 500, false)],
                "wrong time",
            ),
        ] {
            let run = Run {
                schedules,
                ..Run::default()
            };
            let mut report = Report::default();
            compare_request(&script, &run, &motions, &poses, &mut report);
            assert!(
                report
                    .findings
                    .iter()
                    .any(|finding| finding.contains(needle)),
                "{label}: {:?}",
                report.findings
            );
        }
    }

    #[test]
    fn request_matching_reports_overlay_multiplicity_and_each_time_endpoint() {
        let (motions, poses) = play_tables();
        let script = script_with_one_overlay();
        for (label, schedules, needle) in [
            ("missing", vec![schedule(500)], "is missing"),
            (
                "duplicated",
                vec![schedule_with_overlay(3000, 3400, true, false)],
                "duplicated",
            ),
            (
                "extra",
                vec![schedule_with_overlay(3000, 3400, false, true)],
                "extra overlay",
            ),
            (
                "wrong start",
                vec![schedule_with_overlay(3001, 3401, false, false)],
                "wrong time",
            ),
            (
                "wrong end",
                vec![schedule_with_overlay(3000, 3401, false, false)],
                "wrong time",
            ),
        ] {
            let mut report = Report::default();
            compare_request(
                &script,
                &Run {
                    schedules,
                    ..Run::default()
                },
                &motions,
                &poses,
                &mut report,
            );
            assert!(
                report
                    .findings
                    .iter()
                    .any(|finding| finding.contains(needle)),
                "{label}: {:?}",
                report.findings
            );
        }

        let mut duplicate_script = script_with_one_overlay();
        let source = duplicate_script
            .message
            .overlays()
            .iter()
            .next()
            .expect("overlay")
            .clone();
        let mut overlays = duplicate_script.message.overlays_mut();
        let row = overlays.try_grow().expect("duplicate overlay");
        *row = source;
        let mut report = Report::default();
        compare_request(
            &duplicate_script,
            &Run {
                schedules: vec![schedule_with_overlay(3000, 3400, true, false)],
                ..Run::default()
            },
            &motions,
            &poses,
            &mut report,
        );
        assert!(
            report.findings.is_empty(),
            "identical overlay copies are valid: {:?}",
            report.findings
        );
    }

    #[test]
    fn request_matching_reports_missing_sidecar_empty_steps_and_zero_arrival() {
        let (_, poses) = tables();
        let mut report = Report::default();
        compare_request(
            &script_with_one_overlay(),
            &Run {
                schedules: vec![schedule_with_overlay(3000, 3400, false, false)],
                ..Run::default()
            },
            &MotionTable::of([]),
            &poses,
            &mut report,
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("does not name"))
        );

        let mut empty = ScriptWire::new();
        empty.set_arrival(SyncTime::from_nanos(1));
        let mut report = Report::default();
        compare_request(
            &Logged {
                at_ns: 1,
                sequence_number: 1,
                message: empty,
            },
            &Run::default(),
            &MotionTable::of([]),
            &poses,
            &mut report,
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("invalid base-row count"))
        );

        let mut zero = request(500);
        zero.message.set_arrival(SyncTime::from_nanos(0));
        let mut report = Report::default();
        compare_request(
            &zero,
            &Run::default(),
            &MotionTable::of([]),
            &poses,
            &mut report,
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("no arrival instant"))
        );
    }

    fn strict_script() -> ScriptWire {
        let mut message = ScriptWire::new();
        let mut steps = message.steps_mut();
        for (after, pose) in [(1000, 0), (4000, 2)] {
            let step = steps.try_grow().expect("step");
            step.set_after_ms(after);
            step.set_duration_ms(2000);
            step.set_move_ms(500);
            step.set_kind(StepKindWire::BASE_POSTURE);
            step.set_pose_id(pose);
        }
        message
    }

    fn settle_result(first_measured: bool, first_reason: Option<&str>, count: f64) -> SettleResult {
        SettleResult {
            moves: vec![
                SettleMove {
                    start_ns: 100,
                    end_ns: 200,
                    pose_id: 0,
                    pace_ns: 1,
                    measured: first_measured,
                    reason: first_reason.map(str::to_owned),
                },
                SettleMove {
                    start_ns: 300,
                    end_ns: 400,
                    pose_id: 2,
                    pace_ns: 1,
                    measured: false,
                    reason: Some("no commanded change".to_owned()),
                },
            ],
            maxima: [Some((count, 0, "hold-end", 123)); 6],
        }
    }

    #[test]
    fn strict_policy_rejects_nonterminal_skip_with_reason() {
        let (_, poses) = tables();
        let mut report = Report::default();
        strict_settle(
            &strict_script(),
            &settle_result(false, Some("no commanded change"), SETTLE_BOUND_COUNTS),
            2,
            &poses,
            &mut report,
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("no commanded change"))
        );
    }

    #[test]
    fn default_policy_notes_nonterminal_skip_and_stays_green() {
        let run = Run {
            schedules: vec![schedule(500)],
            ..Run::default()
        };
        let mut report = Report::default();
        let ordered = run.ordered_samples();
        settle(
            &run,
            &ordered,
            &[],
            &RunConfig::stated(SHIPPED_PROFILES, DEFAULT_GAINS, true),
            &mut report,
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|note| note.contains("no commanded change")),
            "{:?}",
            report.measured
        );
    }

    #[test]
    fn strict_policy_accepts_only_terminal_stow_skip() {
        let (_, poses) = tables();
        let mut report = Report::default();
        strict_settle(
            &strict_script(),
            &settle_result(true, None, SETTLE_BOUND_COUNTS),
            2,
            &poses,
            &mut report,
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    #[test]
    fn strict_policy_rejects_unavailable_leg_maximum() {
        let (_, poses) = tables();
        let mut unavailable = settle_result(true, None, SETTLE_BOUND_COUNTS);
        unavailable.maxima[0] = None;
        let mut report = Report::default();
        strict_settle(&strict_script(), &unavailable, 2, &poses, &mut report);
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("unavailable leg"))
        );
    }

    #[test]
    fn strict_policy_does_not_restate_shared_excess_count_finding() {
        let (_, poses) = tables();
        let excessive = settle_result(true, None, 18.1);
        let mut report = Report::default();
        report.fail(
            "base-move settle exceeds 18 counts at leg 1: 18.1 counts (pose 0, hold-end at 123)"
                .to_owned(),
        );
        let findings_before = report.findings.len();
        strict_settle(&strict_script(), &excessive, 2, &poses, &mut report);
        assert_eq!(
            report.findings.len(),
            findings_before,
            "strict settle duplicated the shared threshold finding"
        );
    }

    #[test]
    fn strict_policy_checks_real_settle_boundaries_and_skip_shapes() {
        let (_, poses) = tables();
        let make = |moves: Vec<SettleMove>| SettleResult {
            moves,
            maxima: [Some((SETTLE_BOUND_COUNTS, 0, "hold-end", 123)); 6],
        };
        let measured = SettleMove {
            start_ns: 100,
            end_ns: 200,
            pose_id: 0,
            pace_ns: 1,
            measured: true,
            reason: None,
        };
        let skipped = |pose_id| SettleMove {
            start_ns: 300,
            end_ns: 400,
            pose_id,
            pace_ns: 1,
            measured: false,
            reason: Some("reason".to_owned()),
        };
        for (label, result, expected) in [
            (
                "no skipped move",
                make(vec![measured.clone(), measured.clone()]),
                "terminal stow",
            ),
            (
                "nonterminal skipped",
                make(vec![skipped(0), measured.clone()]),
                "terminal stow",
            ),
            (
                "nonstow terminal skipped",
                make(vec![measured.clone(), skipped(0)]),
                "terminal stow",
            ),
            (
                "three moves",
                make(vec![measured.clone(), measured.clone(), skipped(2)]),
                "scheduled base moves",
            ),
        ] {
            let mut report = Report::default();
            strict_settle(&strict_script(), &result, 2, &poses, &mut report);
            assert!(
                report
                    .findings
                    .iter()
                    .any(|finding| finding.contains(expected)),
                "{label}: {:?}",
                report.findings
            );
        }
        let mut report = Report::default();
        strict_settle(
            &strict_script(),
            &make(vec![measured, skipped(2)]),
            2,
            &poses,
            &mut report,
        );
        assert!(
            report.findings.is_empty(),
            "exact terminal stow skip: {:?}",
            report.findings
        );
    }

    #[test]
    fn strict_policy_boundary_is_reported_by_settle_once() {
        let (_, poses) = tables();
        let start = 1_700_000_001_000_000_000;
        for (count, expected) in [
            (SETTLE_BOUND_COUNTS, false),
            (SETTLE_BOUND_COUNTS + 0.1, true),
        ] {
            let run = Run {
                schedules: vec![
                    schedule_custom(0, 1000, 500, false),
                    schedule_custom(2, 4000, 500, false),
                ],
                samples: vec![
                    pose_sample(start, true, 0.0),
                    pose_sample(start + 500_000_000, true, count * COUNT_RAD),
                    pose_sample(start + 1_000_000_000, true, count * COUNT_RAD),
                    pose_sample(start + 1_500_000_000, true, count * COUNT_RAD),
                ],
                ..Run::default()
            };
            let mut report = Report::default();
            let result = settle(
                &run,
                &run.ordered_samples(),
                &[],
                &RunConfig::stated(SHIPPED_PROFILES, DEFAULT_GAINS, true),
                &mut report,
            );
            let before = report.findings.len();
            strict_settle(&strict_script(), &result, 2, &poses, &mut report);
            let shared = report.findings[before..]
                .iter()
                .filter(|finding| finding.contains("settle exceeds"))
                .count();
            assert_eq!(
                shared, 0,
                "strict mode restated a shared finding: {:?}",
                report.findings
            );
            assert_eq!(
                report
                    .findings
                    .iter()
                    .filter(|finding| finding.contains("18.1 counts")
                        && finding.contains("leg 1")
                        && finding.contains("pose 0")
                        && finding.contains("endpoint at 1700000002000000000"))
                    .count(),
                usize::from(expected),
                "{:?}",
                report.findings
            );
        }
    }

    #[test]
    fn strict_policy_rejects_malformed_nonterminal_settle_reading() {
        let (_, poses) = tables();
        let start = 1_700_000_001_000_000_000;
        let run = Run {
            schedules: vec![
                schedule_custom(0, 1000, 500, false),
                schedule_custom(2, 4000, 500, false),
            ],
            samples: vec![
                pose_sample(start, true, 0.0),
                pose_sample(start + 500_000_000, true, 1.0),
                pose_sample(start + 700_000_000, false, 1.0),
                pose_sample(start + 1_500_000_000, true, 1.0),
            ],
            ..Run::default()
        };
        let ordered = run.ordered_samples();
        let mut settle_report = Report::default();
        let result = settle(
            &run,
            &ordered,
            &[],
            &RunConfig::stated(SHIPPED_PROFILES, DEFAULT_GAINS, true),
            &mut settle_report,
        );
        let mut report = Report::default();
        strict_settle(&strict_script(), &result, 2, &poses, &mut report);
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding
                    .contains("endpoint reading at 1700000001700000000 missing commanded or present rows before clean end 1700000003000000000")),
            "{:?}",
            report.findings
        );
        assert!(
            report.findings.iter().any(|finding| finding.contains(
                "strict settle evidence measured count does not match the nonterminal rows"
            )),
            "{:?}",
            report.findings
        );
    }
}
