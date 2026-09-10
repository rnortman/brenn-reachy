//! What a recorded run looks like to the tracking detector, read off an
//! exported trace.
//!
//! `bazel run //cogs:trace_judge -- <trace.csv> <config-dir>`, with
//! `--lag <class>=<microseconds>` for a what-if replay at a lag the run was not
//! judged at.
//!
//! The offline half of the two live analyzers. A hardware run arrives as a
//! Clockwork log, and a log a later tree refuses to read is a recording nobody
//! can take a figure off; `//cogs:trace_export` run from a worktree at the
//! recording's own commit turns a whole run into a schema-free CSV, and this
//! reads that CSV. An exported recording is stated in the same words as a run
//! judged on the day.
//!
//! The configuration is the run's own `config/` directory, for the reason every
//! analyzer here reads one: the residual is the distance from a trajectory the
//! joint's class's two registers and its following lag define, so judging a
//! recording under any other profile measures a machine nobody ran.
//!
//! `--lag` is the one thing this tool does that a live report cannot. It
//! replaces a class's following lag with a stated one and says so at the top of
//! the report, which is how a lag read off the scan is checked against the
//! recording it was read from before anything is written into the profile file.
//! It changes what the model believes and nothing about what the machine did.
//!
//! Sans-I/O in the same sense as the rest of them: the reading is a pure
//! function over the parsed trace and the configuration, and `main` is what
//! binds files to it.

#![forbid(unsafe_code)]

use std::path::PathBuf;
use std::process::ExitCode;

use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
use clockwork_rs::SyncTime;
use log_read::Logged;
use pose_reading::{
    Grid, RunConfig, capabilities, capability, lag_scan, lag_scans, residual_stream, residuals,
};
use reachy_motion::joints::{JointGroup, PerGroup, ROW_COUNT, ROWS, row, write_rows};
use reachy_motion::plant::{ClassProfile, GroupPlants, GroupProfiles};
use reachy_motion::trace::{Run, Trace};
use run_report::{Report, verdict};

/// The instant the first period of a trace is placed at, nanoseconds.
///
/// A trace carries no absolute time -- it counts periods from the window it was
/// cut at -- and the walk needs instants on a grid. Zero, so that every instant
/// a figure is printed at is nanoseconds since the trace's own first period: an
/// offset into the file, which is what a reader holding the file can act on.
const ORIGIN_NS: i64 = 0;

/// A following lag stated on the command line, per class.
///
/// `None` is the class judged at the lag its own configuration states, which is
/// what the run was judged at when it ran.
type LagOverride = PerGroup<Option<u32>>;

/// The classes named by `--lag <class>=<microseconds>` arguments, and the trace
/// and configuration paths beside them.
struct Arguments {
    /// The exported trace to read.
    trace: PathBuf,
    /// The `config/` directory of the run it was exported from.
    config: PathBuf,
    /// What the model is told to believe instead of that configuration.
    lag: LagOverride,
}

/// Read the command line, or say what is wrong with it.
///
/// A class is named the way the configuration files name it, so an operator
/// spells `body_yaw` here exactly as the file they are about to edit does.
///
/// # Errors
///
/// A missing path, an argument that is neither, a class this machine has none
/// of, or a lag that is no unsigned figure of microseconds.
fn arguments(args: &[String]) -> Result<Arguments, String> {
    let mut positional: Vec<&String> = Vec::new();
    // In the order they were stated, so a class named twice reads as the last
    // statement of it rather than as whichever the walk below happened to keep.
    let mut stated_lags: Vec<(JointGroup, u32)> = Vec::new();
    let mut rest = args.iter();
    while let Some(arg) = rest.next() {
        if arg == "--lag" {
            let stated = rest
                .next()
                .ok_or_else(|| "--lag takes <class>=<microseconds>".to_string())?;
            let (class, us) = stated
                .split_once('=')
                .ok_or_else(|| format!("--lag {stated} is no <class>=<microseconds>"))?;
            let group = JointGroup::ALL
                .into_iter()
                .find(|group| group.config_prefix() == class)
                .ok_or_else(|| {
                    format!(
                        "--lag {stated} names no class: this machine's are {}",
                        JointGroup::ALL.map(JointGroup::config_prefix).join(", ")
                    )
                })?;
            let us: u32 = us.parse().map_err(|error| {
                format!("--lag {stated} states no lag in microseconds: {error}")
            })?;
            stated_lags.push((group, us));
        } else {
            positional.push(arg);
        }
    }
    let lag = LagOverride::of_each(|group| {
        stated_lags
            .iter()
            .rev()
            .find(|(stated, _)| *stated == group)
            .map(|(_, us)| *us)
    });
    let [trace, config] = positional.as_slice() else {
        return Err(format!(
            "expected a trace and a config directory, and was given {}",
            positional.len()
        ));
    };
    Ok(Arguments {
        trace: PathBuf::from(*trace),
        config: PathBuf::from(*config),
        lag,
    })
}

/// The profiles the run is judged under: its own, with any stated lag put in
/// their place.
fn judged_at(stated: &GroupProfiles, lag: &LagOverride) -> GroupProfiles {
    GroupProfiles::of_each(|group| ClassProfile {
        following_lag_us: lag.of(group).unwrap_or(stated.of(group).following_lag_us),
        ..stated.of(group)
    })
}

/// What the report says at the top about a lag the run was not judged at.
///
/// One line per overridden class, beside the figure its own configuration
/// states, because a what-if replay whose report does not say it is one is a
/// figure that will be quoted as the machine's.
fn overridden(stated: &GroupProfiles, lag: &LagOverride, report: &mut Report) {
    for group in JointGroup::ALL {
        if let Some(us) = lag.of(group) {
            report.note(format!(
                "--lag {}: this replay models a following lag of {us} us against the {} us this \
                 run's own configuration states, so its figures are a what-if about the model and \
                 not a reading of what the machine did",
                group.config_prefix(),
                stated.of(group).following_lag_us,
            ));
        }
    }
}

/// One trace period as the sample stream the readings are taken over.
///
/// The instant is the period's own tick on the trace's grid rather than its
/// recorded `t_s`: the walk steps the model per grid slot, and the seconds
/// column is the same count printed to a microsecond. A period whose reading
/// fell short carries none, exactly as the log did.
///
/// The commanded half is all nine or none, because the wire form the readings
/// are taken over states its nine goals together. A period carrying some of
/// them is treated as holding no goal, and the count of such periods is
/// printed.
fn samples_of(run: &Run, period_ns: i64, report: &mut Report) -> Vec<Logged<PoseSampleWire>> {
    let mut partial = 0_usize;
    let samples = run
        .samples
        .iter()
        .map(|sample| {
            let mut present = [0.0; ROW_COUNT];
            let mut commanded = [0.0; ROW_COUNT];
            let mut read = 0_usize;
            let mut held = 0_usize;
            for joint in ROWS {
                let Some(index) = row(joint) else {
                    continue;
                };
                if let Some(angle) = sample.present_of(joint) {
                    present[index] = angle;
                    read += 1;
                }
                if let Some(goal) = sample.goal_of(joint) {
                    commanded[index] = goal;
                    held += 1;
                }
            }
            if held > 0 && held < ROW_COUNT {
                partial += 1;
            }
            let at_ns = ORIGIN_NS + i64::try_from(sample.tick).unwrap_or(0) * period_ns;
            let mut message = PoseSampleWire::new();
            {
                let slot = message.clear_valid();
                slot.nominal_time = SyncTime::from_nanos(at_ns);
                slot.sample_time = slot.nominal_time;
                slot.present_valid = (read == ROW_COUNT).into();
                slot.commanded_valid = (held == ROW_COUNT).into();
                write_rows(&mut slot.present, &present);
                write_rows(&mut slot.commanded, &commanded);
            }
            Logged {
                at_ns,
                sequence_number: u32::try_from(sample.tick).unwrap_or(0),
                message,
            }
        })
        .collect();
    if partial > 0 {
        report.note(format!(
            "{partial} period(s) carried a goal for some joints and not others, and are read as \
             periods holding none: the sample form these figures are taken over states its nine \
             goals together"
        ));
    }
    samples
}

/// Everything this tool has to say about one run of a trace.
fn judge(run: &Run, index: usize, of: usize, profiles: &GroupProfiles, report: &mut Report) {
    if run.samples.len() < 2 {
        report.fail(format!(
            "run {index} of {of} holds {} period(s), which says nothing about the grid it was \
             driven on and nothing about what the machine did",
            run.samples.len()
        ));
        return;
    }
    let period_ns = run.period_ns();
    report.note(format!(
        "run {index} of {of}: {} period(s) over {:.3} s, on the recording's own {period_ns} ns \
         grid; every instant below is nanoseconds from this run's first period, not the clock of \
         the log the trace was cut from",
        run.samples.len(),
        run.span().as_secs_f64(),
    ));
    let samples = samples_of(run, period_ns, report);
    let grid = Grid {
        origin_ns: ORIGIN_NS,
        period_ns,
    };
    let plant = match GroupPlants::from_profiles(profiles, period_ns) {
        Ok(plant) => plant,
        Err(error) => {
            report.fail(format!(
                "the profile {profiles:?} on a {period_ns} ns grid is no plant to judge this run \
                 against: {error}"
            ));
            return;
        }
    };
    let stream = residual_stream(&samples, grid, &plant);
    residuals(&stream, &samples, &plant, report);
    let measured = capability(&samples, grid);
    capabilities(&measured, report);
    match lag_scan(&samples, grid, profiles, &measured) {
        Ok(scans) => lag_scans(&scans, report),
        Err(error) => report.fail(error),
    }
}

/// Everything this tool has to say about a trace.
///
/// Every run the file holds, each judged on its own: a trace cut across a stop
/// and a restart is two moves, and one walk over both would step the model
/// across a stretch in which the machine was doing nothing the model knows
/// about.
fn analyze(trace: &Trace, config: &RunConfig, lag: &LagOverride) -> Report {
    let mut report = Report::default();
    config.configuration(&mut report);
    overridden(&config.profiles, lag, &mut report);
    let profiles = judged_at(&config.profiles, lag);
    if trace.runs() == 0 {
        report.fail("the trace holds no periods, so there is nothing to judge".to_string());
        return report;
    }
    for index in 0..trace.runs() {
        judge(
            trace.run(index),
            index,
            trace.runs(),
            &profiles,
            &mut report,
        );
    }
    report
}

fn main() -> ExitCode {
    const USAGE: &str =
        "usage: trace_judge <trace.csv> <config-dir> [--lag <class>=<microseconds>]";
    let args: Vec<String> = std::env::args().skip(1).collect();
    let args = match arguments(&args) {
        Ok(args) => args,
        Err(error) => {
            eprintln!("{error}\n{USAGE}");
            return ExitCode::FAILURE;
        }
    };
    let config = match RunConfig::read_config(&args.config) {
        Ok(config) => config,
        Err(error) => {
            eprintln!("reading the configuration this run was performed under: {error}");
            return ExitCode::FAILURE;
        }
    };
    let text = match std::fs::read_to_string(&args.trace) {
        Ok(text) => text,
        Err(error) => {
            eprintln!("reading the trace {}: {error}", args.trace.display());
            return ExitCode::FAILURE;
        }
    };
    let trace = match Trace::try_parse(&text) {
        Ok(trace) => trace,
        Err(refusal) => {
            eprintln!("reading the trace {}: {refusal}", args.trace.display());
            return ExitCode::FAILURE;
        }
    };
    let report = analyze(&trace, &config, &args.lag);
    verdict(
        "trace_judge",
        &args.trace.display().to_string(),
        &report,
        "the recording holds no finding this tool can raise",
    )
}

#[cfg(test)]
mod tests {
    //! What the tool says about the recordings the replay suite is built on,
    //! and about a trace made from the model itself.
    //!
    //! The kept fixtures are the point of the first two cases: the replay suite
    //! pins their worst residuals through the tick's own walk, and this tool
    //! reaches those figures through the offline walk the live analyzers use.
    //! The two walks agreeing on a recording is what says the words this tool
    //! prints describe the same machine a live report does.

    use super::{Arguments, JointGroup, LagOverride, RunConfig, Trace, analyze, arguments};
    use reachy_motion::RESPONSE_DEAD_SAMPLES;
    use reachy_motion::arm::DEFAULT_GAINS;
    use reachy_motion::joints::{PerGroup, ROW_COUNT, ROWS};
    use reachy_motion::plant::{ClassProfile, GroupProfiles, PlantModel, Predicted};
    use reachy_motion::stillness::COUNT_RAD;
    use reachy_motion::trace::{goal_column, present_column};

    /// The grid every kept recording of the deployment was driven at.
    const PERIOD_NS: i64 = 20_000_000;

    /// The pair every class ran at on the nights the older tours were flown, in
    /// register units.
    const PAIR_2050: ClassProfile = ClassProfile {
        acceleration: 20,
        velocity: 50,
        following_lag_us: 0,
    };

    /// The profile the confirmation tour of 2026-09-09 was flown at, and the
    /// following lag of each class's loop on it.
    ///
    /// A literal, and the same literal the replay suite states: a recording
    /// carries the profile and the loop it was made on and cannot follow a
    /// constant anywhere. The legs ran `800 / 100 / 300` and the antennas the
    /// `200 / 0 / 0` the class ships, which are the two triples a lag has been
    /// read on; the body yaw's loop has no reading.
    const CONFIRM_TOUR_PROFILE: GroupProfiles = GroupProfiles {
        legs: ClassProfile {
            acceleration: 287,
            velocity: 326,
            following_lag_us: 24_000,
        },
        yaw: PAIR_2050,
        antennas: ClassProfile {
            following_lag_us: 24_000,
            ..PAIR_2050
        },
    };

    /// The worst residual a leg ran at on the confirmation tour's `grid-snap`,
    /// radians, as the replay suite pins it through the tick's own walk.
    const CONFIRM_WORST_LEG_RESIDUAL_RAD: f64 = 0.1983;

    /// The worst residual an antenna ran at on the same tour's
    /// `stumble-and-recover`, radians, pinned the same way.
    const TOUR_WORST_ANTENNA_RESIDUAL_RAD: f64 = 0.3625;

    /// How near this tool's figure has to stand to the suite's.
    ///
    /// Half a hundredth of a radian, which is a third of an encoder count short
    /// of nothing: the two walks are the same model over the same periods and
    /// differ only in how each finds the goal a period was answering.
    const AGREEMENT_RAD: f64 = 5e-3;

    /// Where the trace fixtures are, named by the test target beside the data
    /// attribute that supplies them.
    const TRACE_FIXTURES_ENV: &str = "REACHY_MOTION_TRACE_FIXTURES";

    fn fixture(name: &str) -> String {
        let dir = std::env::var(TRACE_FIXTURES_ENV)
            .unwrap_or_else(|_| panic!("{TRACE_FIXTURES_ENV} names the fixture directory"));
        let path = std::path::PathBuf::from(dir).join(format!("{name}.csv"));
        std::fs::read_to_string(&path).unwrap_or_else(|error| panic!("{}: {error}", path.display()))
    }

    /// A configuration stated rather than read off a run's records.
    fn stated(profiles: GroupProfiles) -> RunConfig {
        RunConfig::stated(profiles, DEFAULT_GAINS, true)
    }

    /// No class overridden.
    fn no_override() -> PerGroup<Option<u32>> {
        PerGroup::of_each(|_| None)
    }

    /// The figure a report line states after `prefix`, as a number.
    fn figure(report: &[String], prefix: &str) -> f64 {
        let line = report
            .iter()
            .find(|line| line.contains(prefix))
            .unwrap_or_else(|| panic!("no line holds {prefix:?}: {report:?}"));
        let rest = &line[line.find(prefix).expect("just found") + prefix.len()..];
        rest.split_whitespace()
            .next()
            .expect("a figure follows")
            .parse()
            .expect("and it is one")
    }

    /// The same off every line that states one, in the order the report prints
    /// them: a trace of several runs states its figures once per run.
    fn figures(report: &[String], prefix: &str) -> Vec<f64> {
        report
            .iter()
            .filter(|line| line.contains(prefix))
            .map(|line| {
                let rest = &line[line.find(prefix).expect("just found") + prefix.len()..];
                rest.split_whitespace()
                    .next()
                    .expect("a figure follows")
                    .parse()
                    .expect("and it is one")
            })
            .collect()
    }

    /// The tool reads the kept recordings to the figures the replay suite pins
    /// them at.
    ///
    /// Both classes and both fixtures in one case, because what would be wrong
    /// is a walk that agrees on one class and not the other -- the legs at
    /// their commissioned pair and the antennas at the pair the tour flew them
    /// at are the two models this recording exercises.
    #[test]
    fn the_kept_recordings_read_to_the_figures_the_suite_pins_them_at() {
        let snap = Trace::parse(&fixture("trace-tour-grid-snap"));
        let report = analyze(&snap, &stated(CONFIRM_TOUR_PROFILE), &no_override());
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        let legs = figure(&report.measured, "legs: worst residual ");
        assert!(
            (legs - CONFIRM_WORST_LEG_RESIDUAL_RAD).abs() < AGREEMENT_RAD,
            "grid-snap's worst leg residual reads {legs:.4} rad against the suite's \
             {CONFIRM_WORST_LEG_RESIDUAL_RAD:.4}"
        );

        let stumble = Trace::parse(&fixture("trace-tour-stumble-and-recover"));
        let report = analyze(&stumble, &stated(CONFIRM_TOUR_PROFILE), &no_override());
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        let antennas = figure(&report.measured, "antennas: worst residual ");
        assert!(
            (antennas - TOUR_WORST_ANTENNA_RESIDUAL_RAD).abs() < AGREEMENT_RAD,
            "stumble-and-recover's worst antenna residual reads {antennas:.4} rad against the \
             suite's {TOUR_WORST_ANTENNA_RESIDUAL_RAD:.4}"
        );
    }

    /// A trace whose readings are a plant carrying `lag_us` driven by the
    /// trapezoid of `profile`, written in the format the exporter writes.
    ///
    /// The content is the generator's own path at the commissioned pair, so
    /// every class is saturated and the lag is what stands between the two
    /// columns. A scan over this has one right answer.
    fn synthetic(profile: ClassProfile, lag_us: u32, periods: usize) -> String {
        let generator =
            PlantModel::from_registers(profile.velocity, profile.acceleration, 0, PERIOD_NS)
                .expect("a commissioned pair is a generator");
        let plant =
            PlantModel::from_registers(profile.velocity, profile.acceleration, lag_us, PERIOD_NS)
                .expect("and so is the same pair with a lag");
        // The goal is a target far enough away that the generator never
        // arrives, so the whole recording is a cruise.
        let far = 1_000.0;
        let mut commanded = Predicted::default();
        let mut shaft = Predicted::default();
        let mut goals = Vec::with_capacity(periods);
        let mut readings = Vec::with_capacity(periods);
        for tick in 0..periods {
            generator.step(&mut commanded, far);
            goals.push(commanded.position);
            if tick >= RESPONSE_DEAD_SAMPLES {
                plant.step(&mut shaft, goals[tick - RESPONSE_DEAD_SAMPLES]);
            }
            readings.push(shaft.position);
        }
        let mut text = String::from("run,tick,t_s,phase");
        for joint in ROWS {
            text.push(',');
            text.push_str(&present_column(joint));
        }
        for joint in ROWS {
            text.push(',');
            text.push_str(&goal_column(joint));
        }
        for tick in 0..periods {
            text.push_str(&format!(
                "\n0,{tick},{:.6},commanding",
                tick as f64 * PERIOD_NS as f64 * 1e-9
            ));
            for angle in [readings[tick], goals[tick]] {
                for _ in 0..ROW_COUNT {
                    text.push_str(&format!(",{angle:.6}"));
                }
            }
        }
        text
    }

    /// The scan over a trace the model itself made reads the lag it was made
    /// with, and the model at that lag is the reading.
    #[test]
    fn a_synthetic_lag_is_read_back_by_the_scan() {
        let profile = ClassProfile {
            acceleration: 522,
            velocity: 640,
            following_lag_us: 0,
        };
        // More than five hundred periods, so that the antennas' two rows
        // contribute over a thousand readings and the p99.9 the scan selects on
        // is a percentile rather than the series' own worst sample.
        let text = synthetic(profile, 30_000, 600);
        let trace = Trace::parse(&text);
        let profiles = GroupProfiles::of_each(|_| profile);
        let report = analyze(&trace, &stated(profiles), &no_override());
        let antennas_scan: Vec<&String> = report
            .measured
            .iter()
            .filter(|line| line.contains("antennas: lag scan minimum"))
            .collect();
        assert!(
            figures(
                &antennas_scan
                    .iter()
                    .map(|line| (*line).clone())
                    .collect::<Vec<String>>(),
                "above, over "
            )
            .first()
            .is_some_and(|readings| *readings > 1_000.0),
            "the scan read a percentile of the class's readings: {antennas_scan:?}"
        );

        for group in [JointGroup::Legs, JointGroup::Antennas, JointGroup::BodyYaw] {
            let name = group.name();
            let minimum = figure(&report.measured, &format!("{name}: lag scan minimum "));
            assert!(
                (minimum - 1.5).abs() < 1e-9,
                "{name} reads {minimum:.1} periods against the period and a half it was made with"
            );
        }
        let at_best = figure(&report.measured, "rad at no lag against ");
        assert!(
            at_best < COUNT_RAD,
            "the model at the reading's own lag is the reading: {at_best:.6} rad"
        );
    }

    /// A stated lag reaches the model and says so at the top of the report.
    ///
    /// Both halves, because either alone is the failure: an override the model
    /// ignored is a what-if that never happened, and one the report does not
    /// name is a what-if figure that will be quoted as the machine's.
    #[test]
    fn a_stated_lag_is_what_the_model_believes_and_the_report_says_so() {
        let profile = ClassProfile {
            acceleration: 522,
            velocity: 640,
            following_lag_us: 0,
        };
        let text = synthetic(profile, 30_000, 400);
        let trace = Trace::parse(&text);
        let profiles = GroupProfiles::of_each(|_| profile);
        let lag = LagOverride::of_each(|group| (group == JointGroup::Antennas).then_some(30_000));
        let report = analyze(&trace, &stated(profiles), &lag);

        assert!(
            report.measured.iter().any(|line| line.contains(
                "--lag antennas: this replay models a following lag of 30000 us against the 0 us"
            )),
            "{:?}",
            report.measured
        );
        // The antennas' model now carries the lag the readings were made with,
        // so their residual is the quantisation; the legs' does not, and theirs
        // is most of half a radian of it.
        let antennas = figure(&report.measured, "antennas: worst residual ");
        let legs = figure(&report.measured, "legs: worst residual ");
        assert!(
            antennas < COUNT_RAD,
            "the overridden class reads {antennas:.6} rad"
        );
        assert!(
            legs > 0.4,
            "and the class judged at its own configuration reads {legs:.4} rad"
        );
    }

    /// A trace with a hole in it is refused as a sentence rather than judged
    /// around, because what an operator is holding is a file the export was
    /// interrupted half way through and the tool's contract is a printed
    /// refusal and an exit code.
    #[test]
    fn a_trace_missing_a_cell_is_refused() {
        let profile = ClassProfile {
            acceleration: 522,
            velocity: 640,
            following_lag_us: 0,
        };
        let text = synthetic(profile, 30_000, 8);
        let holed: String = text
            .lines()
            .map(|line| match line.starts_with("0,4,") {
                true => line
                    .rsplit_once(',')
                    .expect("a row has cells")
                    .0
                    .to_string(),
                false => line.to_string(),
            })
            .collect::<Vec<_>>()
            .join("\n");
        assert!(
            Trace::try_parse(&holed).is_err_and(|refusal| refusal.contains("the header names")),
            "a period with a cell missing is refused naming its line"
        );
    }

    /// A trace holding nothing to judge says so as a finding, which is the
    /// tool's own non-zero exit.
    ///
    /// Both shapes an interrupted export leaves behind: a file with a header
    /// and no periods, and one with a single period, which carries no grid and
    /// so nothing the model could be stepped on.
    #[test]
    fn a_trace_with_no_run_to_judge_is_a_finding() {
        let profile = ClassProfile {
            acceleration: 522,
            velocity: 640,
            following_lag_us: 0,
        };
        let profiles = GroupProfiles::of_each(|_| profile);
        let text = synthetic(profile, 30_000, 4);
        let header = text.lines().next().expect("a header").to_string();

        let empty = Trace::try_parse(&header).expect("a header alone parses");
        let report = analyze(&empty, &stated(profiles), &no_override());
        assert!(
            report
                .findings
                .iter()
                .any(|line| line.contains("the trace holds no periods")),
            "{:?}",
            report.findings
        );

        let one: String = text.lines().take(2).collect::<Vec<_>>().join("\n");
        let one = Trace::try_parse(&one).expect("a header and a period parse");
        let report = analyze(&one, &stated(profiles), &no_override());
        assert!(
            report.findings.iter().any(|line| line
                .contains("run 0 of 1 holds 1 period(s), which says nothing about the grid")),
            "{:?}",
            report.findings
        );
    }

    /// A trace cut across a stop and a restart is two moves, judged one at a
    /// time.
    ///
    /// The reason the tool walks runs separately at all: one walk over both
    /// would step the model across the stretch the machine was not being driven
    /// through, and print residual the machine never ran. Each half here is a
    /// different plant, so a walk that ran them together could not read either
    /// figure.
    #[test]
    fn a_trace_of_two_runs_is_judged_one_run_at_a_time() {
        let profile = ClassProfile {
            acceleration: 522,
            velocity: 640,
            following_lag_us: 0,
        };
        let lagged = synthetic(profile, 30_000, 40);
        let plain = synthetic(profile, 0, 40);
        // The second file's periods count from zero again, which is what a
        // counter that fails to advance means: a fresh move.
        let text = format!(
            "{lagged}\n{}",
            plain.lines().skip(1).collect::<Vec<_>>().join("\n")
        );
        let trace = Trace::try_parse(&text).expect("two runs parse");
        assert_eq!(trace.runs(), 2, "a restarted counter is a second run");

        let profiles = GroupProfiles::of_each(|_| profile);
        let report = analyze(&trace, &stated(profiles), &no_override());
        for run in ["run 0 of 2", "run 1 of 2"] {
            assert!(
                report.measured.iter().any(|line| line.contains(run)),
                "{run} is missing from {:?}",
                report.measured
            );
        }
        let worst = figures(&report.measured, "antennas: worst residual ");
        assert_eq!(worst.len(), 2, "one reading per run: {worst:?}");
        assert!(
            worst[0] > 0.4,
            "the lagged half stands most of the screen from the trapezoid it is judged at: {:.4}",
            worst[0]
        );
        assert!(
            worst[1] < COUNT_RAD,
            "and the half that is the trapezoid reads as it: {:.6}",
            worst[1]
        );
    }

    /// A period holding a goal for some joints and not others holds none, and
    /// the report counts them.
    ///
    /// The wire form these figures are taken over states its nine goals
    /// together, so a mixture cannot be carried; what must not happen is a
    /// period read as commanding zeros, which is a residual of a radian against
    /// a goal nobody wrote.
    #[test]
    fn a_period_holding_some_goals_and_not_others_is_counted_and_holds_none() {
        let profile = ClassProfile {
            acceleration: 522,
            velocity: 640,
            following_lag_us: 0,
        };
        let text = synthetic(profile, 0, 40);
        let partial: String = text
            .lines()
            .map(|line| match line.starts_with("0,20,") {
                true => {
                    let mut cells: Vec<&str> = line.split(',').collect();
                    // One goal cell of the nine, which is the shape the wire
                    // form cannot carry.
                    cells[4 + ROW_COUNT] = "";
                    cells.join(",")
                }
                false => line.to_string(),
            })
            .collect::<Vec<_>>()
            .join("\n");
        let trace = Trace::try_parse(&partial).expect("a blanked goal cell still parses");
        let profiles = GroupProfiles::of_each(|_| profile);
        let report = analyze(&trace, &stated(profiles), &no_override());
        assert!(
            report.measured.iter().any(|line| line
                .starts_with("1 period(s) carried a goal for some joints and not others")),
            "{:?}",
            report.measured
        );
    }

    /// The command line names two paths and any number of classes.
    #[test]
    fn the_arguments_are_two_paths_and_the_classes_stated_beside_them() {
        let args = ["trace.csv", "--lag", "body_yaw=1500", "run/config"]
            .map(str::to_string)
            .to_vec();
        let Arguments { trace, config, lag } =
            arguments(&args).expect("a trace, a config directory and one class");
        assert_eq!(trace, std::path::PathBuf::from("trace.csv"));
        assert_eq!(config, std::path::PathBuf::from("run/config"));
        assert_eq!(lag.of(JointGroup::BodyYaw), Some(1500));
        assert_eq!(lag.of(JointGroup::Antennas), None);

        for wrong in [
            vec!["trace.csv".to_string()],
            vec!["a".to_string(), "b".to_string(), "c".to_string()],
            ["a", "b", "--lag"].map(str::to_string).to_vec(),
            ["a", "b", "--lag", "legs"].map(str::to_string).to_vec(),
            ["a", "b", "--lag", "cranks=10"]
                .map(str::to_string)
                .to_vec(),
            ["a", "b", "--lag", "legs=soon"]
                .map(str::to_string)
                .to_vec(),
        ] {
            assert!(arguments(&wrong).is_err(), "{wrong:?}");
        }
    }

    /// A period the trace holds no reading for is judged as one, not as a zero.
    #[test]
    fn a_period_that_read_nothing_measures_nothing() {
        let profile = ClassProfile {
            acceleration: 522,
            velocity: 640,
            following_lag_us: 0,
        };
        let text = synthetic(profile, 0, 40);
        let blanked: String = text
            .lines()
            .map(|line| match line.starts_with("0,20,") {
                true => {
                    let mut cells: Vec<&str> = line.split(',').collect();
                    for cell in cells.iter_mut().skip(4).take(ROW_COUNT) {
                        *cell = "";
                    }
                    cells.join(",")
                }
                false => line.to_string(),
            })
            .collect::<Vec<_>>()
            .join("\n");
        let trace = Trace::parse(&blanked);
        let profiles = GroupProfiles::of_each(|_| profile);
        let report = analyze(&trace, &stated(profiles), &no_override());
        assert!(
            report.measured.iter().any(|line| line
                .starts_with("39 of 40 samples were judged against the modelled trajectory")),
            "{:?}",
            report.measured
        );
    }
}
