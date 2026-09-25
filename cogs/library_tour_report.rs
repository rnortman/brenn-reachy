//! What one library tour did, read off its log.
//!
//! The tool a tour run is judged by. `reachy_ask --tour` asks the machine for
//! every motion the deployed library holds, one after the other, at recorded
//! pace; this reads the log that run wrote and answers one question -- did
//! every motion in the library play, whole -- beside the numbers the run exists
//! to take: how far each joint lagged its goal inside each motion, how far the
//! goal moved in a period, whether the driver missed a slot, and how near the
//! antenna tips came to meeting.
//!
//! The measurements are the point. There is no speed policy on content and no
//! import-time crossing check on a clip's antenna track; what stands in for
//! both is a record of what the servos actually did when the whole library was
//! played at them. A run that is clean here is a run whose numbers can be read.
//!
//! Two distances per window, and they answer different questions. The lag is
//! how far a joint stood behind its goal, which on velocity-capped servos is a
//! figure about how fast the content is. The residual is how far it stood from
//! where a healthy servo following its generator would stand -- that generator
//! with the position loop's own following lag on it -- stepped by the same
//! model the decision tick screens on, so a run is judged offline by the
//! arithmetic that judged it live.
//!
//! It reads the log and the names sidecar, and nothing else. The sidecar is what
//! says which motions the library holds and what they are called, which is the
//! list the tour is judged against -- the log carries indices and an index is
//! not a name. The profile the residual is measured against comes out of the
//! run's own records, beside them in `config/`: a log recorded under one set of
//! pairs cannot be judged under another, and a tuning campaign varies them per
//! run.
//!
//! Findings split the way the bring-up rule splits them. A motion never asked
//! for, or a gap in the sample stream, is a defect in the harness. A window
//! through which the goal never moved, a fault, or a lag worth a look is the
//! machine's own answer and is what the run was taken to find. Both are
//! findings here: this tool says what the run did, and which kind a finding is
//! is read off what it says.

use std::collections::{BTreeMap, BTreeSet};
use std::path::PathBuf;
use std::process::ExitCode;

#[cfg(test)]
use brenn_reachy__driver__health_clk_rs::{DriverEventWire, EventKindWire};
use pose_reading::{RunConfig, no_faults};
use reachy_edge::names::MotionTable;
use run_report::{Report, verdict};

use motion_run_report::{
    Run, Window, every_window_moved, held_standard, named_motion, overlay_spans, prepare, read,
    settle, stillness, the_stream_held, whole_stream_measurements, window_measurements, windows,
};

#[cfg(test)]
use brenn_reachy__cogs__schedule_clk_rs::SessionScheduleWire;
#[cfg(test)]
use brenn_reachy__cogs__script_clk_rs::ScriptWire;
#[cfg(test)]
use brenn_reachy__driver__health_clk_rs::HealthReportWire;
#[cfg(test)]
use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
#[cfg(test)]
use brenn_reachy__motion__faults_clk_rs::TickFaultWire;
#[cfg(test)]
use log_read::Logged;
#[cfg(test)]
use motion_channels::POSE_CHANNEL;
#[cfg(test)]
use motion_run_report::CHANNELS;
#[cfg(test)]
use reachy_motion::phase::ANTENNA_CONTACT_BAND_RAD;

/// Every motion the library holds was asked for, once, in the order the library
/// numbers them.
///
/// The tour's own contract, and the first thing to read: every check below is
/// about motions that were asked for, so a report that opened with a clean lag
/// figure over a tour that asked for three of sixty-eight would be a run
/// passing on a subset.
fn asked_for(run: &Run, by_id: &BTreeMap<u16, String>, report: &mut Report) -> Vec<u16> {
    let mut asked: Vec<u16> = Vec::new();
    for (index, script) in run.scripts.iter().enumerate() {
        let overlays = script.message.overlays();
        if overlays.len() == 1 {
            asked.push(overlays.iter().next().expect("one window").motion_id());
            continue;
        }
        report.fail(format!(
            "script {index} on the incoming stream carries {} overlay window(s), and a tour asks \
             for exactly one motion per script",
            overlays.len()
        ));
    }

    let expected: Vec<u16> = by_id.keys().copied().collect();
    let seen: BTreeSet<u16> = asked.iter().copied().collect();
    for motion_id in &expected {
        if !seen.contains(motion_id) {
            report.fail(format!(
                "{} was never asked for",
                named_motion(by_id, *motion_id)
            ));
        }
    }
    for motion_id in &seen {
        let count = asked.iter().filter(|id| *id == motion_id).count();
        if count > 1 {
            report.fail(format!(
                "{} was asked for {count} times, and a tour asks once",
                named_motion(by_id, *motion_id)
            ));
        }
        if !by_id.contains_key(motion_id) {
            report.fail(format!(
                "motion {motion_id} was asked for and the sidecar names no such motion"
            ));
        }
    }
    if asked.len() == expected.len() && seen.len() == asked.len() && asked != expected {
        report.fail(
            "every motion was asked for once but not in the order the library numbers them, so \
             the plan and the sidecar disagree about the run"
                .to_string(),
        );
    }
    report.note(format!(
        "{} script(s) asked for {} of the sidecar's {} motion(s)",
        run.scripts.len(),
        seen.len(),
        expected.len()
    ));
    asked
}

/// Every motion asked for was scheduled, and every window planned was asked
/// for.
///
/// A script the session refused plans no window, and every other check runs
/// over the windows that were planned. Without this check, motions refused
/// after a mid-tour park would be named by nothing at all: the run would read
/// as a fault plus an otherwise complete tour.
///
/// The other direction: a window over a motion no script asked for.
fn every_asked_motion_was_scheduled(
    asked: &[u16],
    planned: &[Window],
    by_id: &BTreeMap<u16, String>,
    report: &mut Report,
) {
    let scheduled: BTreeSet<u16> = planned.iter().map(|window| window.motion_id).collect();
    let mut named: BTreeSet<u16> = BTreeSet::new();
    for motion_id in asked {
        if !scheduled.contains(motion_id) && named.insert(*motion_id) {
            report.fail(format!(
                "{} was asked for and never scheduled, so the session refused or held that \
                 script and the machine never played it",
                named_motion(by_id, *motion_id)
            ));
        }
    }
    let asked_ids: BTreeSet<u16> = asked.iter().copied().collect();
    for motion_id in &scheduled {
        if !asked_ids.contains(motion_id) {
            report.fail(format!(
                "a window was planned over {} and no script on the incoming stream asked for it",
                named_motion(by_id, *motion_id)
            ));
        }
    }
}

/// Everything this tool has to say about one tour.
fn analyze(run: &Run, table: &MotionTable, config: &RunConfig) -> Report {
    let mut report = Report::default();
    for complaint in &run.complaints {
        report.fail(complaint.clone());
    }
    config.configuration(&mut report);
    for channel in &run.census {
        report.note(format!("  {} x{}", channel.name, channel.count));
    }
    if run.samples.is_empty() {
        report.fail(
            "the log holds no driver samples, so there is no record of what the machine did"
                .to_string(),
        );
        return report;
    }
    let by_id: BTreeMap<u16, String> = table
        .entries()
        .map(|(name, entry)| (entry.motion_id, name.to_string()))
        .collect();
    let planned = windows(run);
    let prepared = match prepare(run, config) {
        Ok(prepared) => prepared,
        Err(error) => {
            report.fail(error);
            return report;
        }
    };
    let asked = asked_for(run, &by_id, &mut report);
    no_faults(&run.faults, &mut report);
    every_asked_motion_was_scheduled(&asked, &planned, &by_id, &mut report);
    every_window_moved(&prepared.ordered, &planned, &by_id, &mut report);
    if planned.is_empty() {
        report.fail(
            "the session planned no overlay window at all, so the tour asked the machine for \
             nothing it could play"
                .to_string(),
        );
    }
    the_stream_held(
        &prepared.ordered,
        &overlay_spans(&planned),
        prepared.grid,
        &prepared.skips,
        &mut report,
        "the tour",
    );
    window_measurements(&prepared, &planned, &by_id, &mut report);
    whole_stream_measurements(&prepared, run, config, &mut report);
    stillness(&prepared.ordered, held_standard(table), &mut report);
    let settle_result = settle(run, &prepared.ordered, &planned, config, &mut report);
    let _ = (&settle_result.moves, &settle_result.maxima);
    report
}

fn main() -> ExitCode {
    const USAGE: &str = "usage: library_tour_report <log-dir> <names.json>";
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [log_dir, sidecar] = args.as_slice() else {
        eprintln!("{USAGE}");
        return ExitCode::FAILURE;
    };
    // The tour's own configuration, out of the records: a log is judged under
    // the pairs the machine that wrote it was commissioned with.
    let config = match RunConfig::read(&PathBuf::from(log_dir)) {
        Ok(config) => config,
        Err(err) => {
            eprintln!("reading the configuration this run was performed under: {err}");
            return ExitCode::FAILURE;
        }
    };
    let text = match std::fs::read_to_string(sidecar) {
        Ok(text) => text,
        Err(err) => {
            eprintln!("reading the names sidecar {sidecar}: {err}");
            return ExitCode::FAILURE;
        }
    };
    let table = match MotionTable::from_sidecar(&text) {
        Ok(table) => table,
        Err(err) => {
            eprintln!("the names sidecar {sidecar} is not one: {err}");
            return ExitCode::FAILURE;
        }
    };
    let run = match read(&PathBuf::from(log_dir)) {
        Ok(run) => run,
        Err(err) => {
            eprintln!("reading the log under {log_dir}: {err}");
            return ExitCode::FAILURE;
        }
    };
    let report = analyze(&run, &table, &config);
    verdict(
        "library_tour_report",
        log_dir,
        &report,
        "every motion in the library played, whole",
    )
}

#[cfg(test)]
mod tests {
    //! What the analyzer says about crafted runs.
    //!
    //! Each case builds the streams a tour writes rather than a log file; what
    //! is under test is the reading. A fixture run is two motions long, which
    //! is enough for order, repetition and omission to be different things.

    use super::{
        ANTENNA_CONTACT_BAND_RAD, CHANNELS, DriverEventWire, EventKindWire, HealthReportWire,
        Logged, MotionTable, POSE_CHANNEL, PoseSampleWire, Report, Run, RunConfig, ScriptWire,
        SessionScheduleWire, TickFaultWire, Window, analyze, settle, windows,
    };
    use motion_run_report::SETTLE_BOUND_COUNTS;
    use pose_reading::TEMPERATURE_STOP_C;
    use reachy_motion::arm::DEFAULT_GAINS;
    use reachy_motion::plant::{ClassProfile, GroupProfiles, SHIPPED_PROFILES};

    use brenn_reachy__cogs__schedule_clk_rs::OverlayWindowWire;
    use brenn_reachy__cogs__schedule_clk_rs::{ScheduledStepWire, StepKindWire};
    use brenn_reachy__cogs__script_clk_rs::ScriptOverlayWire;
    use brenn_reachy__motion__faults_clk_rs::FaultKindWire;
    use clockwork_rs::SyncTime;
    use motion_proto::PlayWindow;
    use reachy_driver::NOMINAL_CYCLE_NS;
    use reachy_edge::names::MotionEntry;
    use reachy_motion::joints::{JointGroup, JointRef, ROW_COUNT, row, write_rows};
    use reachy_motion::stillness::COUNT_RAD;

    /// An arbitrary instant a synthetic run starts at, chosen for being nothing
    /// round.
    const T0: i64 = 1_772_000_000_123_456_789;

    /// How many cycles each fixture window is open for.
    const WINDOW_CYCLES: i64 = 20;

    /// One message of a synthetic stream, at cycle `n` of the run.
    fn at<T>(n: i64, message: T) -> Logged<T> {
        Logged {
            at_ns: T0 + n * NOMINAL_CYCLE_NS,
            sequence_number: u32::try_from(n).unwrap_or(0),
            message,
        }
    }

    /// The instant cycle `n` of a synthetic run sits at.
    fn when(n: i64) -> SyncTime {
        SyncTime::from_nanos(T0 + n * NOMINAL_CYCLE_NS)
    }

    /// A two-motion library, numbered as the emitter numbers one.
    fn table() -> MotionTable {
        MotionTable::of([
            (
                "pollen/dances/simple_nod".to_string(),
                MotionEntry {
                    motion_id: 0,
                    window: PlayWindow {
                        duration_ms: 400,
                        blend_out_ms: 200,
                    },
                },
            ),
            (
                "pollen/emotions/curious1".to_string(),
                MotionEntry {
                    motion_id: 1,
                    window: PlayWindow {
                        duration_ms: 400,
                        blend_out_ms: 200,
                    },
                },
            ),
        ])
    }

    /// One script asking for `motion_id`, as the sender writes one.
    fn script(n: i64, motion_id: u16) -> Logged<ScriptWire> {
        let mut message = ScriptWire::new();
        message.set_arrival(when(n));
        {
            let mut rows = message.overlays_mut();
            rows.clear();
            let row: &mut ScriptOverlayWire = rows.try_grow().expect("a script of one window");
            row.set_motion_id(motion_id);
            row.set_after_ms(0);
            row.set_duration_ms(400);
            row.set_gain(1.0);
            row.set_speed(1.0);
        }
        at(n, message)
    }

    /// A schedule carrying one open window, as the session republishes one.
    fn schedule(n: i64, motion_id: u16, start: i64, end: i64) -> Logged<SessionScheduleWire> {
        let mut message = SessionScheduleWire::new();
        message.set_engaged(true);
        {
            let mut rows = message.overlays_mut();
            rows.clear();
            let row: &mut OverlayWindowWire = rows.try_grow().expect("a schedule of one window");
            row.set_motion_id(motion_id);
            row.set_start(when(start));
            row.set_end(when(end));
            row.set_gain(1.0);
            row.set_speed(1.0);
        }
        at(n, message)
    }

    /// One sample: read at `present`, holding `commanded`.
    fn sample(n: i64, present: &[f64; ROW_COUNT], commanded: &[f64; ROW_COUNT]) -> PoseSampleWire {
        let mut message = PoseSampleWire::new();
        {
            let read = message.clear_valid();
            read.nominal_time = when(n);
            read.sample_time = when(n);
            read.present_valid = true.into();
            read.commanded_valid = true.into();
            write_rows(&mut read.present, present);
            write_rows(&mut read.commanded, commanded);
        }
        message
    }

    /// The heartbeat of a tour that played: over each window the body yaw walks
    /// a milliradian per cycle and the machine reads where it was told, so
    /// every window moved and nothing lagged.
    fn heartbeat(cycles: i64) -> Vec<Logged<PoseSampleWire>> {
        (0..cycles)
            .map(|n| {
                let mut rows = [0.0; ROW_COUNT];
                rows[row(JointRef::BodyYaw).expect("a bus row")] = n as f64 * 1e-3;
                at(n, sample(n, &rows, &rows))
            })
            .collect()
    }

    /// The two windows a two-motion tour plans, back to back.
    fn planned() -> Vec<Logged<SessionScheduleWire>> {
        vec![
            schedule(0, 0, 1, 1 + WINDOW_CYCLES),
            schedule(
                1 + WINDOW_CYCLES,
                1,
                1 + WINDOW_CYCLES,
                1 + 2 * WINDOW_CYCLES,
            ),
        ]
    }

    /// A tour that did everything it claims to have done.
    fn clean() -> Run {
        Run {
            scripts: vec![script(0, 0), script(WINDOW_CYCLES, 1)],
            schedules: planned(),
            samples: heartbeat(2 + 2 * WINDOW_CYCLES),
            ..Run::default()
        }
    }

    /// The configuration a crafted run is judged under: what this tree ships,
    /// with the detector armed, which is what every case here is about the
    /// machine and not about the configuration.
    fn shipped() -> RunConfig {
        RunConfig::stated(SHIPPED_PROFILES, DEFAULT_GAINS, true)
    }

    #[test]
    fn preparation_reports_a_profile_that_cannot_form_a_plant() {
        let run = clean();
        let profiles = GroupProfiles::of_each(|_| ClassProfile {
            acceleration: 0,
            velocity: 0,
            following_lag_us: 0,
        });
        let result =
            motion_run_report::prepare(&run, &RunConfig::stated(profiles, DEFAULT_GAINS, true));
        let error = match result {
            Ok(_) => panic!("zero profiles cannot form a plant"),
            Err(error) => error,
        };
        assert!(error.contains("do not form a plant"), "{error}");
    }

    /// A schedule carrying one absolute base-posture step.
    fn base_schedule(
        sequence: i64,
        start: i64,
        end: i64,
        pose_id: u16,
        pace: i64,
    ) -> Logged<SessionScheduleWire> {
        let mut message = SessionScheduleWire::new();
        message.set_engaged(true);
        let mut steps = message.steps_mut();
        let step: &mut ScheduledStepWire = steps.try_grow().expect("one base step fits");
        step.set_start(SyncTime::from_nanos(start));
        step.set_end(SyncTime::from_nanos(end));
        step.set_kind(StepKindWire::BASE_POSTURE);
        step.set_pose_id(pose_id);
        step.set_pace(clockwork_rs::Duration::from_nanos(pace));
        Logged {
            at_ns: start,
            sequence_number: sequence as u32,
            message,
        }
    }

    /// A direct settle report fixture, without the unrelated tour verdicts.
    fn settle_report(
        schedules: Vec<Logged<SessionScheduleWire>>,
        samples: Vec<Logged<PoseSampleWire>>,
    ) -> Report {
        let run = Run {
            schedules,
            samples,
            ..Run::default()
        };
        let ordered = run.ordered_samples();
        let mut report = Report::default();
        settle(&run, &ordered, &[], &shipped(), &mut report);
        report
    }

    fn settle_report_with_windows(
        schedules: Vec<Logged<SessionScheduleWire>>,
        samples: Vec<Logged<PoseSampleWire>>,
        planned: &[Window],
    ) -> Report {
        let run = Run {
            schedules,
            samples,
            ..Run::default()
        };
        let ordered = run.ordered_samples();
        let mut report = Report::default();
        settle(&run, &ordered, planned, &shipped(), &mut report);
        report
    }

    /// One sample with one leg's commanded-present error in encoder counts.
    fn leg_sample(n: i64, leg: usize, counts: f64) -> Logged<PoseSampleWire> {
        let mut counts_by_leg = [0.0; 6];
        counts_by_leg[leg] = counts;
        legs_sample(n, counts_by_leg)
    }

    /// One valid sample with an error for each leg.
    fn legs_sample(n: i64, counts: [f64; 6]) -> Logged<PoseSampleWire> {
        let mut commanded = [0.0; ROW_COUNT];
        let present = [0.0; ROW_COUNT];
        for (leg, count) in counts.into_iter().enumerate() {
            commanded[row(JointRef::Leg0).expect("leg row") + leg] = count * COUNT_RAD;
        }
        at(n, sample(n, &present, &commanded))
    }

    #[test]
    fn base_move_settle_deduplicates_republishes_and_keeps_both_reading_provenances() {
        let start = T0;
        let pace = 5 * NOMINAL_CYCLE_NS;
        let end = T0 + 100 * NOMINAL_CYCLE_NS;
        let schedules = vec![
            base_schedule(0, start, end, 7, pace),
            base_schedule(1, start, end, 7, pace),
        ];
        let report = settle_report(
            schedules,
            vec![
                at(1, sample(1, &[0.0; ROW_COUNT], &[0.0; ROW_COUNT])),
                legs_sample(10, [SETTLE_BOUND_COUNTS, 10.0, 10.0, 10.0, 10.0, 10.0]),
                legs_sample(60, [SETTLE_BOUND_COUNTS, 10.0, 10.0, 10.0, 10.0, 10.0]),
            ],
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line == "1 measured, 0 skipped base moves"),
            "{:?}",
            report.measured
        );
        assert_eq!(
            report
                .measured
                .iter()
                .filter(|line| line.contains("base-move pose 7 measured"))
                .count(),
            1
        );
        for leg in 1..=6 {
            assert!(
                measured(&report, &format!("leg {leg}")),
                "{:?}",
                report.measured
            );
        }
        assert!(measured(&report, "pose 7"), "{:?}", report.measured);
    }

    #[test]
    fn base_move_settle_accepts_eighteen_and_rejects_eighteen_point_one_counts() {
        let schedule = base_schedule(0, T0, T0 + 20 * NOMINAL_CYCLE_NS, 9, 5 * NOMINAL_CYCLE_NS);
        let clean = settle_report(
            vec![base_schedule(
                0,
                T0,
                T0 + 20 * NOMINAL_CYCLE_NS,
                9,
                5 * NOMINAL_CYCLE_NS,
            )],
            vec![
                legs_sample(1, [0.0; 6]),
                legs_sample(10, [SETTLE_BOUND_COUNTS; 6]),
                legs_sample(19, [SETTLE_BOUND_COUNTS; 6]),
            ],
        );
        assert!(clean.findings.is_empty(), "{:?}", clean.findings);
        let red = settle_report(
            vec![schedule],
            vec![
                legs_sample(1, [0.0; 6]),
                legs_sample(
                    10,
                    [
                        18.1,
                        SETTLE_BOUND_COUNTS,
                        SETTLE_BOUND_COUNTS,
                        SETTLE_BOUND_COUNTS,
                        SETTLE_BOUND_COUNTS,
                        SETTLE_BOUND_COUNTS,
                    ],
                ),
                legs_sample(
                    19,
                    [
                        18.1,
                        SETTLE_BOUND_COUNTS,
                        SETTLE_BOUND_COUNTS,
                        SETTLE_BOUND_COUNTS,
                        SETTLE_BOUND_COUNTS,
                        SETTLE_BOUND_COUNTS,
                    ],
                ),
            ],
        );
        assert!(found(&red, "leg 1"), "{:?}", red.findings);
        assert!(found(&red, "18.1 counts"), "{:?}", red.findings);
    }

    #[test]
    fn base_move_settle_reports_missing_intervals_and_invalid_rows() {
        let pace = 5 * NOMINAL_CYCLE_NS;
        let no_hold = base_schedule(0, T0, T0 + pace, 3, pace);
        let no_hold_report = settle_report(vec![no_hold], Vec::new());
        assert!(
            no_hold_report
                .measured
                .iter()
                .any(|line| line.contains(&format!(
                    "base move pose 3 skipped: no commanded change in [{T0}, {})",
                    T0 + pace
                )))
        );
        assert_eq!(
            no_hold_report
                .measured
                .iter()
                .filter(|line| line.contains("base-move settle") && line.contains("unavailable"))
                .count(),
            6,
            "{:?}",
            no_hold_report.measured
        );

        let schedule = base_schedule(1, T0, T0 + 20 * NOMINAL_CYCLE_NS, 4, pace);
        let missing_endpoint = settle_report(
            vec![base_schedule(1, T0, T0 + 20 * NOMINAL_CYCLE_NS, 4, pace)],
            vec![legs_sample(1, [0.0; 6]), leg_sample(4, 0, 1.0)],
        );
        assert!(
            missing_endpoint
                .measured
                .iter()
                .any(|line| line.contains(&format!(
                    "hold shorter than the following lag: endpoint {}, first eligible {}, clean end {}",
                    T0 + 4 * NOMINAL_CYCLE_NS,
                    T0 + 4 * NOMINAL_CYCLE_NS + 24_000_000,
                    T0 + 20 * NOMINAL_CYCLE_NS
                ))),
            "{:?}",
            missing_endpoint.measured
        );
        let mut invalid = sample(10, &[0.0; ROW_COUNT], &[0.0; ROW_COUNT]);
        invalid.set_present_valid(false);
        let invalid_report = settle_report(
            vec![schedule],
            vec![
                at(1, sample(1, &[0.0; ROW_COUNT], &[0.0; ROW_COUNT])),
                at(10, invalid),
            ],
        );
        assert!(
            invalid_report
                .measured
                .iter()
                .any(|line| line.contains(&format!(
                    "no commanded change in [{}, {})",
                    T0,
                    T0 + 20 * NOMINAL_CYCLE_NS
                )))
        );

        let mut invalid_endpoint = sample(19, &[0.0; ROW_COUNT], &[COUNT_RAD; ROW_COUNT]);
        invalid_endpoint.set_present_valid(false);
        let invalid_endpoint_report = settle_report(
            vec![base_schedule(0, T0, T0 + 20 * NOMINAL_CYCLE_NS, 4, pace)],
            vec![
                at(1, sample(1, &[0.0; ROW_COUNT], &[0.0; ROW_COUNT])),
                at(4, sample(4, &[0.0; ROW_COUNT], &[COUNT_RAD; ROW_COUNT])),
                at(19, invalid_endpoint),
            ],
        );
        assert!(
            invalid_endpoint_report
                .measured
                .iter()
                .any(|line| line.contains(&format!(
                    "endpoint reading at {} missing commanded or present rows before clean end {}",
                    T0 + 19 * NOMINAL_CYCLE_NS,
                    T0 + 20 * NOMINAL_CYCLE_NS
                ))),
            "{:?}",
            invalid_endpoint_report.measured
        );

        let mut invalid_hold_end = sample(19, &[0.0; ROW_COUNT], &[COUNT_RAD; ROW_COUNT]);
        invalid_hold_end.set_present_valid(false);
        let malformed = settle_report(
            vec![base_schedule(0, T0, T0 + 20 * NOMINAL_CYCLE_NS, 5, pace)],
            vec![
                at(1, sample(1, &[0.0; ROW_COUNT], &[0.0; ROW_COUNT])),
                legs_sample(4, [1.0; 6]),
                legs_sample(10, [1.0; 6]),
                at(19, invalid_hold_end),
            ],
        );
        assert!(
            malformed
                .measured
                .iter()
                .any(|line| line.contains("0 measured, 1 skipped base moves")),
            "{:?}",
            malformed.measured
        );
        assert!(
            malformed
                .measured
                .iter()
                .any(|line| line.contains("hold-end reading at 1772000000503456789 missing commanded or present rows before clean end 1772000000523456789")),
            "{:?}",
            malformed.measured
        );
        assert!(
            !malformed
                .measured
                .iter()
                .any(|line| line.contains("base-move pose 5 measured")),
            "{:?}",
            malformed.measured
        );
        assert_eq!(
            malformed
                .measured
                .iter()
                .filter(|line| line.contains("base-move settle") && line.contains("unavailable"))
                .count(),
            6,
            "{:?}",
            malformed.measured
        );

        let repeated = settle_report(
            vec![
                base_schedule(0, T0, T0 + 20 * NOMINAL_CYCLE_NS, 15, pace),
                base_schedule(
                    0,
                    T0 + 30 * NOMINAL_CYCLE_NS,
                    T0 + 50 * NOMINAL_CYCLE_NS,
                    15,
                    pace,
                ),
            ],
            Vec::new(),
        );
        assert!(repeated.measured.iter().any(|line| line.contains(&format!(
            "no commanded change in [{}, {})",
            T0,
            T0 + 20 * NOMINAL_CYCLE_NS
        ))));
        assert!(repeated.measured.iter().any(|line| line.contains(&format!(
            "no commanded change in [{}, {})",
            T0 + 30 * NOMINAL_CYCLE_NS,
            T0 + 50 * NOMINAL_CYCLE_NS
        ))));
    }

    #[test]
    fn base_move_settle_uses_the_last_sample_before_clean_end_for_hold_end() {
        let pace = 5 * NOMINAL_CYCLE_NS;
        let reading = |n: i64, present_count: f64| {
            let present = [present_count * COUNT_RAD; ROW_COUNT];
            let commanded = [COUNT_RAD; ROW_COUNT];
            at(n, sample(n, &present, &commanded))
        };
        let report = settle_report(
            vec![base_schedule(0, T0, T0 + 20 * NOMINAL_CYCLE_NS, 15, pace)],
            vec![
                at(1, sample(1, &[0.0; ROW_COUNT], &[0.0; ROW_COUNT])),
                reading(4, 0.9),
                reading(10, 0.9),
                reading(19, 0.0),
            ],
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains(&format!("hold-end at {}", T0 + 19 * NOMINAL_CYCLE_NS))),
            "{:?}",
            report.measured
        );
    }

    #[test]
    fn base_move_settle_uses_record_changes_and_excludes_overlay_intervals() {
        let start = T0 + NOMINAL_CYCLE_NS;
        let end = T0 + 100 * NOMINAL_CYCLE_NS;
        let samples = || {
            vec![
                legs_sample(1, [0.0; 6]),
                legs_sample(5, [1.0; 6]),
                legs_sample(9, [2.0; 6]),
                legs_sample(10, [SETTLE_BOUND_COUNTS; 6]),
                legs_sample(60, [SETTLE_BOUND_COUNTS; 6]),
            ]
        };
        let record_keyed = settle_report(
            vec![base_schedule(0, start, end, 12, 5 * NOMINAL_CYCLE_NS)],
            samples(),
        );
        assert!(measured(
            &record_keyed,
            &format!("endpoint {}", T0 + 10 * NOMINAL_CYCLE_NS)
        ));

        let mixed_samples = || {
            let mut samples = Vec::new();
            for (n, legs, antenna) in [
                (1, 0.0, 0.0),
                (5, 1.0, 0.0),
                (9, 2.0, 0.0),
                (10, 2.0, 1.0),
                (11, 2.0, 2.0),
                (60, 2.0, 3.0),
            ] {
                let mut commanded = [0.0; ROW_COUNT];
                for leg in 0..6 {
                    commanded[row(JointRef::Leg0).expect("leg row") + leg] = legs;
                }
                commanded[row(JointRef::AntennaLeft).expect("antenna row")] = antenna;
                samples.push(at(n, sample(n, &[0.0; ROW_COUNT], &commanded)));
            }
            samples
        };
        let mixed = settle_report(
            vec![base_schedule(0, start, end, 16, 5 * NOMINAL_CYCLE_NS)],
            mixed_samples(),
        );
        assert!(
            measured(&mixed, &format!("endpoint {}", T0 + 9 * NOMINAL_CYCLE_NS)),
            "{:?}",
            mixed.measured
        );
        assert!(
            mixed.measured.iter().any(|line| {
                line.contains("base-move settle leg 1")
                    && line.contains(&format!("at {}", T0 + 11 * NOMINAL_CYCLE_NS))
            }),
            "{:?}",
            mixed.measured
        );

        let opening = Window {
            motion_id: 3,
            start_ns: T0 + 80 * NOMINAL_CYCLE_NS,
            end_ns: T0 + 90 * NOMINAL_CYCLE_NS,
        };
        let truncated = settle_report_with_windows(
            vec![base_schedule(0, start, end, 13, 5 * NOMINAL_CYCLE_NS)],
            samples(),
            &[opening],
        );
        assert!(measured(
            &truncated,
            &format!("clean end {}", opening.start_ns)
        ));

        let covered = settle_report_with_windows(
            vec![base_schedule(0, start, end, 14, 5 * NOMINAL_CYCLE_NS)],
            vec![legs_sample(5, [1.0; 6]), legs_sample(19, [1.0; 6])],
            &[Window {
                motion_id: 4,
                start_ns: start,
                end_ns: end,
            }],
        );
        assert!(covered.measured.iter().any(|line| line.contains(&format!(
            "base move pose 14 skipped: overlay motion 4 begins at {} before base start {}",
            start, start
        ))));

        let unchanged = settle_report(
            vec![base_schedule(0, start, end, 15, 5 * NOMINAL_CYCLE_NS)],
            vec![legs_sample(1, [0.0; 6]), legs_sample(19, [0.0; 6])],
        );
        assert!(unchanged.measured.iter().any(|line| line.contains(&format!(
            "base move pose 15 skipped: no commanded change in [{}, {})",
            start, end
        ))));
    }

    /// Whether any finding says `what`.
    fn found(report: &Report, what: &str) -> bool {
        report.findings.iter().any(|line| line.contains(what))
    }

    /// Whether any measurement says `what`.
    fn measured(report: &Report, what: &str) -> bool {
        report.measured.iter().any(|line| line.contains(what))
    }

    /// The case every other one is read against: a tour that asked for both
    /// motions in order, moved the machine through both windows and dropped no
    /// sample has nothing to report and still prints its numbers.
    #[test]
    fn a_tour_that_played_every_motion_has_no_findings() {
        let report = analyze(&clean(), &table(), &shipped());
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            measured(&report, "pollen/dances/simple_nod [") && measured(&report, "20 sample(s)"),
            "{:?}",
            report.measured
        );
        // The window's own instants, which are what `//cogs:trace_export`
        // needs to cut a replay fixture from the log.
        assert!(
            measured(
                &report,
                &format!(
                    "[{} .. {}]",
                    when(1).as_nanos(),
                    when(1 + WINDOW_CYCLES).as_nanos()
                )
            ),
            "{:?}",
            report.measured
        );
        assert!(
            measured(
                &report,
                "2 script(s) asked for 2 of the sidecar's 2 motion(s)"
            ),
            "{:?}",
            report.measured
        );
    }

    /// What the health rotation saw over the tour reaches the report, and a
    /// servo that latched something beyond the voltage bit is a finding.
    #[test]
    fn the_health_rotation_reaches_the_tour_report() {
        let mut warm = HealthReportWire::new();
        warm.set_id(10);
        warm.set_volts(7.4);
        warm.set_temp_c(48);
        warm.set_sample_time(when(3));
        let mut hurt = HealthReportWire::new();
        hurt.set_id(11);
        hurt.set_volts(7.4);
        hurt.set_bits(0x20);
        hurt.set_sample_time(when(3));
        let mut run = clean();
        run.readings = vec![
            Logged {
                at_ns: when(3).as_nanos(),
                sequence_number: 0,
                message: warm,
            },
            Logged {
                at_ns: when(3).as_nanos(),
                sequence_number: 1,
                message: hurt,
            },
        ];
        let report = analyze(&run, &table(), &shipped());
        assert!(
            measured(&report, "servo 10: 7.40 V"),
            "{:?}",
            report.measured
        );
        assert!(measured(&report, "peak 48 C"), "{:?}", report.measured);
        assert!(found(&report, "servo 11 (0x20)"), "{:?}", report.findings);
        // The warm servo sits under the temperature ceiling, so the error-bit
        // rule is the only verdict this fixture carries.
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
    }

    /// A servo that reached the temperature ceiling fails the tour report,
    /// through the same pass the per-servo lines come from.
    ///
    /// The reading is a verdict in both analyzers, and only this case says so
    /// of the tour: the unit case one module over would keep passing if the
    /// tour report stopped calling the health pass at all.
    #[test]
    fn a_servo_at_the_temperature_ceiling_fails_the_tour_report() {
        let mut hot = HealthReportWire::new();
        hot.set_id(18);
        hot.set_volts(7.4);
        hot.set_temp_c(TEMPERATURE_STOP_C);
        hot.set_sample_time(when(3));
        let mut run = clean();
        run.readings = vec![Logged {
            at_ns: when(3).as_nanos(),
            sequence_number: 0,
            message: hot,
        }];
        let report = analyze(&run, &table(), &shipped());
        // The rule's own wording and figure, not the servo id alone: any
        // per-servo verdict names an id, so an id is not evidence that this is
        // the temperature one.
        assert!(
            found(
                &report,
                &format!("reached {TEMPERATURE_STOP_C} C, which no healthy tour")
            ),
            "{:?}",
            report.findings
        );
        assert!(
            found(
                &report,
                &format!("servo 18 ({TEMPERATURE_STOP_C} C at 0.0 s)")
            ),
            "{:?}",
            report.findings
        );
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
    }

    /// A tour whose joints never fell behind a setpoint measures no capability
    /// and says so, rather than printing the content's own pace as the motors'.
    #[test]
    fn a_tour_that_never_saturated_a_motor_says_it_measured_nothing() {
        let report = analyze(&clean(), &table(), &shipped());
        for class in JointGroup::ALL.map(JointGroup::name) {
            assert!(
                measured(
                    &report,
                    &format!("capability {class}: no sample found this class chasing")
                ),
                "{:?}",
                report.measured
            );
        }
    }

    /// A log with no samples says exactly that rather than reporting a clean
    /// sweep of checks none of which had anything to read.
    #[test]
    fn a_log_with_no_samples_is_refused_at_once() {
        let report = analyze(&Run::default(), &table(), &shipped());
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(found(&report, "no driver samples"), "{:?}", report.findings);
    }

    /// A motion the tour never asked for is named, which is the finding a
    /// sender that stopped early produces.
    #[test]
    fn a_motion_never_asked_for_is_named() {
        let run = Run {
            scripts: vec![script(0, 0)],
            ..clean()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(&report, "pollen/emotions/curious1 was never asked for"),
            "{:?}",
            report.findings
        );
    }

    /// A motion asked for twice is a plan that lost its place, and is a finding
    /// even though every motion was played.
    #[test]
    fn a_motion_asked_for_twice_is_a_finding() {
        let run = Run {
            scripts: vec![script(0, 0), script(WINDOW_CYCLES, 1), script(41, 1)],
            ..clean()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(&report, "curious1 was asked for 2 times"),
            "{:?}",
            report.findings
        );
    }

    /// Every motion once, out of the order the library numbers them: the counts
    /// all agree and the run is still wrong.
    #[test]
    fn a_tour_that_asked_out_of_order_is_a_finding() {
        let run = Run {
            scripts: vec![script(0, 1), script(WINDOW_CYCLES, 0)],
            ..clean()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(&report, "not in the order the library numbers them"),
            "{:?}",
            report.findings
        );
    }

    /// A script carrying no overlay, or two, is not a tour's script at all.
    #[test]
    fn a_script_that_asks_for_no_motion_is_a_finding() {
        let mut empty = ScriptWire::new();
        empty.set_arrival(when(0));
        empty.overlays_mut().clear();
        let run = Run {
            scripts: vec![at(0, empty), script(WINDOW_CYCLES, 1)],
            ..clean()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(&report, "carries 0 overlay window(s)"),
            "{:?}",
            report.findings
        );
    }

    /// The whole-library assertion: a window across which the goal never moved
    /// is a composed setpoint the mover refused, and is named by the motion it
    /// was going to play.
    #[test]
    fn a_window_the_goal_never_moved_across_is_named() {
        let held = [0.0; ROW_COUNT];
        let samples: Vec<Logged<PoseSampleWire>> = (0..(2 + 2 * WINDOW_CYCLES))
            .map(|n| {
                let mut rows = [0.0; ROW_COUNT];
                if n < 1 + WINDOW_CYCLES {
                    rows[row(JointRef::BodyYaw).expect("a bus row")] = n as f64 * 1e-3;
                }
                at(
                    n,
                    sample(n, &rows, if n < 1 + WINDOW_CYCLES { &rows } else { &held }),
                )
            })
            .collect();
        let run = Run { samples, ..clean() };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(
                &report,
                "the goal never changed across the window over \
                            pollen/emotions/curious1"
            ),
            "{:?}",
            report.findings
        );
        assert_eq!(
            report
                .findings
                .iter()
                .filter(|finding| finding.contains("the goal never changed across the window"))
                .count(),
            1,
            "{:?}",
            report.findings
        );
    }

    /// A stretch of the record the log does not hold is one finding with the
    /// longest gap in it, whatever else the run did.
    #[test]
    fn a_gap_in_the_sample_stream_is_a_finding() {
        let clean = clean();
        let samples: Vec<Logged<PoseSampleWire>> = clean
            .samples
            .into_iter()
            .filter(|logged| {
                let at = logged.message.nominal_time().as_nanos();
                let cycle = (at - T0) / NOMINAL_CYCLE_NS;
                !(5..9).contains(&cycle)
            })
            .collect();
        let run = Run {
            samples,
            scripts: vec![script(0, 0), script(WINDOW_CYCLES, 1)],
            schedules: planned(),
            ..Run::default()
        };
        let report = analyze(&run, &table(), &shipped());
        assert_eq!(
            report
                .findings
                .iter()
                .filter(|line| line.contains("gap(s) over the tour"))
                .count(),
            1,
            "{:?}",
            report.findings
        );
        assert!(found(&report, "1 gap(s)"), "{:?}", report.findings);
    }

    /// The same stretch of missing samples, with the driver's own report of
    /// having missed those slots beside it: a machine reading rather than a
    /// hole in the record, and the bring-up rule classifies the two oppositely.
    #[test]
    fn a_gap_the_driver_reported_as_skipped_cycles_is_not_a_harness_finding() {
        let clean = clean();
        let samples: Vec<Logged<PoseSampleWire>> = clean
            .samples
            .into_iter()
            .filter(|logged| {
                let at = logged.message.nominal_time().as_nanos();
                let cycle = (at - T0) / NOMINAL_CYCLE_NS;
                !(5..9).contains(&cycle)
            })
            .collect();
        // The report is published by the first cycle attended after the run of
        // missed slots, and says how many they were.
        let mut event = DriverEventWire::new();
        event.set_kind(EventKindWire::CYCLE_SKIPPED);
        event.set_time(when(9));
        event.set_count(4);
        let run = Run {
            samples,
            events: vec![at(9, event)],
            scripts: vec![script(0, 0), script(WINDOW_CYCLES, 1)],
            schedules: planned(),
            ..Run::default()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            !found(&report, "gap(s) over the tour"),
            "{:?}",
            report.findings
        );
        assert!(
            measured(&report, "cycles the driver reported as skipped"),
            "{:?}",
            report.measured
        );
    }

    /// A fault is the machine's answer and is reported with the instant and the
    /// count it was raised at.
    #[test]
    fn a_fault_the_tick_raised_is_a_finding() {
        let mut fault = TickFaultWire::new();
        fault.set_kind(FaultKindWire::HEAD_SERVO_FAULT);
        fault.set_time(when(3));
        fault.set_count(1);
        let run = Run {
            faults: vec![at(3, fault)],
            ..clean()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(&report, "the decision tick raised"),
            "{:?}",
            report.findings
        );
    }

    /// The antenna measurement: a pair that came near meeting inside the band
    /// is reported as how near, and a pair that never both stood inside it says
    /// so instead of reporting a number taken where the tips cannot touch.
    #[test]
    fn the_antenna_pair_is_measured_only_inside_the_contact_band() {
        let near = 0.05;
        let samples: Vec<Logged<PoseSampleWire>> = (0..(2 + 2 * WINDOW_CYCLES))
            .map(|n| {
                let mut rows = [0.0; ROW_COUNT];
                rows[row(JointRef::BodyYaw).expect("a bus row")] = n as f64 * 1e-3;
                let (right, left) = if n < 1 + WINDOW_CYCLES {
                    (near, -near)
                } else {
                    // Both well outside the band, where a mirror offset says
                    // nothing about the tips meeting.
                    (
                        ANTENNA_CONTACT_BAND_RAD + 0.2,
                        ANTENNA_CONTACT_BAND_RAD + 0.2,
                    )
                };
                rows[row(JointRef::AntennaRight).expect("a bus row")] = right;
                rows[row(JointRef::AntennaLeft).expect("a bus row")] = left;
                at(n, sample(n, &rows, &rows))
            })
            .collect();
        let run = Run { samples, ..clean() };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            measured(&report, "simple_nod")
                && report
                    .measured
                    .iter()
                    .any(|line| line.contains("simple_nod")
                        && line.contains("commanded tips 0.000 rad from mirrored")),
            "{:?}",
            report.measured
        );
        assert!(
            report.measured.iter().any(|line| line.contains("curious1")
                && line.contains("commanded tips never both inside the band")),
            "{:?}",
            report.measured
        );
    }

    /// A window a replacement cut short ends where the schedule that dropped it
    /// landed, not where it was published to end. The session never republishes
    /// a window with a shorter end -- it publishes a schedule that no longer
    /// holds it -- so reading the published end would stretch that motion's
    /// measurements over the hand-back and into the next script.
    #[test]
    fn a_window_the_next_schedule_dropped_ends_where_that_schedule_landed() {
        let run = Run {
            // Published to run to cycle 40, and at cycle 12 the session says a
            // schedule that carries only the replacement.
            schedules: vec![schedule(0, 0, 1, 40), schedule(12, 1, 12, 40)],
            ..Run::default()
        };
        assert_eq!(
            windows(&run),
            vec![
                Window {
                    motion_id: 0,
                    start_ns: T0 + NOMINAL_CYCLE_NS,
                    end_ns: T0 + 12 * NOMINAL_CYCLE_NS,
                },
                Window {
                    motion_id: 1,
                    start_ns: T0 + 12 * NOMINAL_CYCLE_NS,
                    end_ns: T0 + 40 * NOMINAL_CYCLE_NS,
                },
            ]
        );
    }

    /// A window still carried by the last schedule keeps the end it was
    /// published with: nothing dropped it, so nothing cut it.
    #[test]
    fn a_window_nothing_dropped_keeps_the_end_it_was_published_with() {
        let run = Run {
            schedules: vec![schedule(0, 0, 1, 40), schedule(2, 0, 1, 40)],
            ..Run::default()
        };
        assert_eq!(
            windows(&run),
            vec![Window {
                motion_id: 0,
                start_ns: T0 + NOMINAL_CYCLE_NS,
                end_ns: T0 + 40 * NOMINAL_CYCLE_NS,
            }]
        );
    }

    /// The case a mid-tour park produces: the scripts kept going out, the
    /// session refused them, and the motions they named have no window for any
    /// other check to say anything about. Naming them is the whole per-motion
    /// accounting on the run the clock ending was designed around.
    #[test]
    fn a_motion_asked_for_and_never_scheduled_is_named() {
        let run = Run {
            schedules: vec![schedule(0, 0, 1, 1 + WINDOW_CYCLES)],
            ..clean()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(
                &report,
                "pollen/emotions/curious1 was asked for and never scheduled"
            ),
            "{:?}",
            report.findings
        );
    }

    /// The other direction: a window over a motion nothing asked for is a log
    /// this tool cannot account for.
    #[test]
    fn a_window_no_script_asked_for_is_named() {
        let run = Run {
            scripts: vec![script(0, 0)],
            schedules: planned(),
            ..clean()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(
                &report,
                "a window was planned over pollen/emotions/curious1 and no script"
            ),
            "{:?}",
            report.findings
        );
    }

    /// The staleness check: a log carrying indices from a library other than
    /// the committed name table the verdict is read against. The index is all
    /// the log holds, so the finding says the index and says the sidecar does
    /// not name it.
    #[test]
    fn a_motion_id_the_sidecar_does_not_name_is_a_finding() {
        let run = Run {
            scripts: vec![script(0, 0), script(WINDOW_CYCLES, 7)],
            ..clean()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(
                &report,
                "motion 7 was asked for and the sidecar names no such motion"
            ),
            "{:?}",
            report.findings
        );
        assert!(
            found(&report, "motion 7, which the sidecar does not name"),
            "the fallback names the index where there is no name to give: {:?}",
            report.findings
        );
    }

    /// The numbers the run exists to take, against a machine that lagged: the
    /// figures and the joints they are attributed to, not the labels around
    /// them.
    #[test]
    fn the_measurements_are_the_figures_and_the_joints_that_carried_them() {
        let lag = 0.05;
        let step = 1e-3;
        let samples: Vec<Logged<PoseSampleWire>> = (0..(2 + 2 * WINDOW_CYCLES))
            .map(|n| {
                let mut commanded = [0.0; ROW_COUNT];
                commanded[row(JointRef::BodyYaw).expect("a bus row")] = n as f64 * step;
                let mut present = commanded;
                // One head crank held a known distance behind its setpoint,
                // and the antennas read somewhere other than where they were
                // told, so the two tip figures are different numbers.
                present[row(JointRef::Leg2).expect("a bus row")] = -lag;
                present[row(JointRef::AntennaRight).expect("a bus row")] = 0.02;
                present[row(JointRef::AntennaLeft).expect("a bus row")] = -0.05;
                at(n, sample(n, &present, &commanded))
            })
            .collect();
        let run = Run { samples, ..clean() };
        let report = analyze(&run, &table(), &shipped());
        let line = report
            .measured
            .iter()
            .find(|line| line.contains("pollen/dances/simple_nod ["))
            .expect("the first window's measurements")
            .clone();
        assert!(line.contains("worst lag 0.0500 rad at leg 3"), "{line}");
        assert!(
            line.contains("peak step 0.0010 rad at body yaw per period"),
            "{line}"
        );
        assert!(
            line.contains("commanded tips 0.000 rad from mirrored"),
            "{line}"
        );
        assert!(
            line.contains("present tips 0.030 rad from mirrored"),
            "the pair was read somewhere other than where it was commanded, and the two \
             figures are different numbers: {line}"
        );
    }

    /// A skip is counted on the window it fell in and nowhere else: the figure
    /// exists to explain the lag on the line it sits on.
    #[test]
    fn a_skipped_cycle_is_counted_on_its_own_window_and_not_the_next() {
        let mut event = DriverEventWire::new();
        event.set_kind(EventKindWire::CYCLE_SKIPPED);
        event.set_time(when(3));
        let run = Run {
            events: vec![at(3, event)],
            ..clean()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("pollen/dances/simple_nod [")
                    && line.contains("1 skipped cycle(s)")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("pollen/emotions/curious1 [")
                    && line.contains("0 skipped cycle(s)")),
            "{:?}",
            report.measured
        );
    }

    /// A window with no sample in it at all: the plan says the machine was
    /// playing and the record says nothing about it.
    #[test]
    fn a_window_with_no_sample_in_it_is_named() {
        let run = Run {
            samples: heartbeat(1 + WINDOW_CYCLES),
            ..clean()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(&report, "carries no sample at all"),
            "{:?}",
            report.findings
        );
    }

    /// A window whose samples all carried an invalid setpoint: the machine was
    /// under no command for the whole of it, which is not the same finding as a
    /// goal that never moved.
    #[test]
    fn a_window_under_no_command_is_named() {
        let samples: Vec<Logged<PoseSampleWire>> = (0..(2 + 2 * WINDOW_CYCLES))
            .map(|n| {
                let mut rows = [0.0; ROW_COUNT];
                rows[row(JointRef::BodyYaw).expect("a bus row")] = n as f64 * 1e-3;
                let mut message = sample(n, &rows, &rows);
                if n > WINDOW_CYCLES {
                    // The second window's every sample: read, and holding
                    // nothing.
                    message.set_commanded_valid(false);
                }
                at(n, message)
            })
            .collect();
        let run = Run { samples, ..clean() };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(&report, "carried a setpoint"),
            "{:?}",
            report.findings
        );
    }

    /// The loudest thing a real tour can go wrong with: the scripts went out
    /// and the session scheduled nothing at all.
    #[test]
    fn a_run_that_planned_no_window_at_all_is_named() {
        let run = Run {
            schedules: Vec::new(),
            ..clean()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(&report, "planned no overlay window at all"),
            "{:?}",
            report.findings
        );
    }

    /// A stream that started after the first window opened, and one that
    /// stopped before the last one closed: the two ends of the stretch the tour
    /// is judged over are gaps like any other, and a run that dropped either
    /// end reads clean without them.
    #[test]
    fn a_stream_that_started_late_or_stopped_early_is_a_gap() {
        let late = Run {
            samples: heartbeat(2 + 2 * WINDOW_CYCLES)
                .into_iter()
                .skip(11)
                .collect(),
            ..clean()
        };
        assert!(
            found(
                &analyze(&late, &table(), &shipped()),
                "gap(s) over the tour"
            ),
            "a logger that came up after the tour started is a hole in the record",
        );
        let early = Run {
            samples: heartbeat(2 + 2 * WINDOW_CYCLES)
                .into_iter()
                .take(2 * WINDOW_CYCLES as usize - 10)
                .collect(),
            ..clean()
        };
        assert!(
            found(
                &analyze(&early, &table(), &shipped()),
                "gap(s) over the tour"
            ),
            "a logger that stopped before the last window closed is the same hole",
        );
    }

    /// A log whose samples all sit outside the stretch the tour is judged over:
    /// the driver ran, and not while the tour did.
    #[test]
    fn a_log_holding_no_sample_inside_the_tour_is_named() {
        let run = Run {
            samples: heartbeat(1),
            schedules: vec![schedule(0, 0, 10, 10 + WINDOW_CYCLES)],
            scripts: vec![script(0, 0)],
            ..Run::default()
        };
        let report = analyze(&run, &table(), &shipped());
        assert!(
            found(&report, "holds no sample between the first window opening"),
            "{:?}",
            report.findings
        );
    }

    /// The binding table, the one thing every case above skips by building its
    /// streams by hand: a duplicated or mistyped row routes a stream into
    /// nothing and the analyzer then reports a clean tour over a log it half
    /// read.
    #[test]
    fn every_channel_is_named_once() {
        let mut names: Vec<&str> = CHANNELS.iter().map(|bound| bound.name).collect();
        names.sort_unstable();
        let mut unique = names.clone();
        unique.dedup();
        assert_eq!(
            names, unique,
            "a channel bound twice routes one of them nowhere"
        );
        assert_eq!(names.len(), 6, "six streams is what a tour is judged on");
    }

    /// What the log itself said about the channels it carried reaches the
    /// report, so a run over a log missing a stream is readable as that.
    #[test]
    fn the_census_reaches_the_report() {
        let mut run = clean();
        run.census.push(log_read::Channel {
            name: POSE_CHANNEL.to_owned(),
            count: 42,
            first_seq: Some(0),
        });
        let report = analyze(&run, &table(), &shipped());
        assert!(
            measured(&report, &format!("{POSE_CHANNEL} x42")),
            "{:?}",
            report.measured
        );
    }

    /// How many cycles a synthetic long run gives the head to settle before the
    /// antennas are commanded to the pose they then hold: past the last window,
    /// so the head's own walk is over before the hold this asks about opens.
    const HEAD_SETTLED: i64 = 50;

    /// A tour-shaped run long enough to hold: the body yaw walks through both
    /// windows as `heartbeat` has it, the antennas are commanded to one pose at
    /// `HEAD_SETTLED` and hold it to the end reading `swing` either side of it,
    /// and the head either stands from the last window or is commanded
    /// somewhere new every hundred cycles after it.
    fn long_run(cycles: i64, swing: f64, head_moves: bool) -> Run {
        let yaw = row(JointRef::BodyYaw).expect("a bus row");
        let right = row(JointRef::AntennaRight).expect("a bus row");
        let left = row(JointRef::AntennaLeft).expect("a bus row");
        let windows = 2 + 2 * WINDOW_CYCLES;
        let samples = (0..cycles)
            .map(|n| {
                let mut commanded = [0.0; ROW_COUNT];
                commanded[yaw] = if n < windows {
                    n as f64 * 1e-3
                } else if head_moves {
                    (windows - 1) as f64 * 1e-3 + ((n - windows) / 100) as f64 * 0.1
                } else {
                    (windows - 1) as f64 * 1e-3
                };
                let goal = if n < HEAD_SETTLED { 0.0 } else { 0.5 };
                commanded[right] = goal;
                commanded[left] = goal;
                let mut present = commanded;
                let reading = if n % 2 == 0 { swing } else { -swing };
                present[right] += reading;
                present[left] += reading;
                at(n, sample(n, &present, &commanded))
            })
            .collect();
        Run { samples, ..clean() }
    }

    /// The tour's stillness verdict: a hold the head stood still across is
    /// judged, and a hunting antenna over a still head fails the run the way it
    /// does in the motion report.
    #[test]
    fn a_hunting_antenna_hold_with_the_head_still_fails_the_tour() {
        let report = analyze(&long_run(500, 3.0 * COUNT_RAD, false), &table(), &shipped());
        assert!(
            found(&report, "right antenna moved 6.0 counts"),
            "{:?}",
            report.findings
        );
        assert!(
            measured(&report, "stillness:") && measured(&report, "right antenna over a"),
            "{:?}",
            report.measured
        );
        assert!(
            !measured(&report, "under head motion"),
            "the head stood still across this hold: {:?}",
            report.measured
        );
    }

    /// The same hold with the head being commanded across it is printed with
    /// its figures and no verdict: the excursion is the rod following the
    /// platform, which is not what the bound is written for.
    #[test]
    fn an_antenna_hold_under_head_motion_is_printed_and_judges_nothing() {
        let report = analyze(&long_run(500, 3.0 * COUNT_RAD, true), &table(), &shipped());
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            measured(&report, "under head motion, not judged"),
            "{:?}",
            report.measured
        );
        assert!(measured(&report, "6.0 counts"), "{:?}", report.measured);
    }

    /// And a tour that held the antennas only under head motion says it
    /// measured nothing about them rather than failing: a content tour holds
    /// them almost only while the head moves, which is the content's shape and
    /// not a defect.
    #[test]
    fn a_tour_with_no_head_still_antenna_hold_measures_nothing() {
        let report = analyze(&long_run(500, 0.0, true), &table(), &shipped());
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            measured(&report, "with the head still, so this run says nothing"),
            "{:?}",
            report.measured
        );
        assert!(
            measured(&report, "2 antenna hold(s) read under head motion"),
            "{:?}",
            report.measured
        );
    }

    /// The table of a probe run: one instrument, played on its own.
    fn probe_table() -> MotionTable {
        MotionTable::of([(
            "probe/antenna-step-a".to_string(),
            MotionEntry {
                motion_id: 0,
                window: PlayWindow {
                    duration_ms: 400,
                    blend_out_ms: 200,
                },
            },
        )])
    }

    /// The same run, judged as a probe run: holding nothing with the head still
    /// is a **failure** there, where under the content tour it is a reading.
    ///
    /// Only the stillness sentence is read: a hand-built run's scripts and this
    /// one-motion table disagree about what was asked for, which is the tour
    /// contract's own finding and not this case's subject.
    #[test]
    fn a_probe_run_that_held_nothing_with_the_head_still_fails() {
        let run = long_run(500, 0.0, true);
        const SAYS: &str = "with the head still, so this run says nothing";
        let content = analyze(&run, &table(), &shipped());
        assert!(measured(&content, SAYS), "{:?}", content.measured);
        assert!(!found(&content, SAYS), "{:?}", content.findings);
        let probe = analyze(&run, &probe_table(), &shipped());
        assert!(found(&probe, SAYS), "{:?}", probe.findings);
        assert!(!measured(&probe, SAYS), "{:?}", probe.measured);
    }

    /// And where the probe run did hold, the qualifier judges it: the hold is
    /// the run's own, so a hunt in it is the finding either standard gives.
    #[test]
    fn a_probe_runs_head_still_hold_is_judged_as_any_other() {
        let probe = analyze(
            &long_run(500, 3.0 * COUNT_RAD, false),
            &probe_table(),
            &shipped(),
        );
        assert!(
            found(&probe, "right antenna moved 6.0 counts"),
            "{:?}",
            probe.findings
        );
        assert!(
            !found(&probe, "with the head still, so this run says nothing"),
            "{:?}",
            probe.findings
        );
    }

    /// The settle line, which is what a probe run is read by: the arrival the
    /// judged window drops is printed under the hold with its own figures, so a
    /// joint that rang down ten counts and then stood still passes with the
    /// ring-down on the page.
    #[test]
    fn the_settle_line_prints_the_ring_down_under_the_hold() {
        let yaw = row(JointRef::BodyYaw).expect("a bus row");
        let right = row(JointRef::AntennaRight).expect("a bus row");
        let left = row(JointRef::AntennaLeft).expect("a bus row");
        let windows = 2 + 2 * WINDOW_CYCLES;
        let samples = (0..500)
            .map(|n| {
                let mut commanded = [0.0; ROW_COUNT];
                commanded[yaw] = if n < windows {
                    n as f64 * 1e-3
                } else {
                    (windows - 1) as f64 * 1e-3
                };
                let goal = if n < HEAD_SETTLED { 0.0 } else { 0.5 };
                commanded[right] = goal;
                commanded[left] = goal;
                let mut present = commanded;
                // The first four seconds after the command are the arrival: a
                // five-count swing either side, turning round every cycle.
                if (HEAD_SETTLED..HEAD_SETTLED + 200).contains(&n) {
                    let reading = if n % 2 == 0 { 5.0 } else { -5.0 } * COUNT_RAD;
                    present[right] += reading;
                    present[left] += reading;
                }
                at(n, sample(n, &present, &commanded))
            })
            .collect();
        let report = analyze(&Run { samples, ..clean() }, &table(), &shipped());
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("settling into it")
                    && line.contains("10.0 counts")
                    && line.contains("25.0 Hz apparent")
                    && line.contains("not judged")),
            "{:?}",
            report.measured
        );
        // The judged tail is the joint at rest, which is what the verdict is.
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("right antenna over a") && line.contains("0.0 counts")),
            "{:?}",
            report.measured
        );
    }
}
