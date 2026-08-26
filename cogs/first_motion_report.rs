//! What one run of the motion system did, read off its log.
//!
//! The tool a hardware run is judged by. Given the directory a run's log
//! was written to, it answers
//! one question -- did the wake gesture happen, whole -- and prints the numbers
//! an operator wants beside the answer: how the driver's heartbeat held up, what
//! the read jitter was, how far the machine lagged its own goals, what the
//! health rotation saw, and how many messages every channel carried.
//!
//! It reads a log and nothing else. No process is started, no clock on this
//! machine is consulted, and nothing about the run is taken from a console: a
//! run that happened on a unit last week is judged the same way as one that
//! finished a second ago.
//!
//! The channel set is the one both systems declare, so the same binary reads a
//! hardware log and a scenario log. That is what makes it testable without a
//! hardware log at all: the deterministic S1 run performs the same gesture, and
//! the test beside this file runs this analyzer over its output.
//!
//! Two things it deliberately does not do. It does not reason in cycle numbers
//! taken from a scenario's epoch -- the grid is derived from the samples the log
//! carries, because a real run starts at whatever top of a second it started at.
//! And it does not assert the deterministic run's exact arithmetic: the
//! tolerances here are sized for a machine with servo quantisation, linkage
//! compliance and a real bus in it, which is looser than a scenario's and is the
//! point of a separate tool.

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use brenn_reachy__cogs__schedule_clk_rs::SessionScheduleWire;
use brenn_reachy__cogs__script_clk_rs::ScriptWire;
use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__cogs__session_cmd_clk_rs::{SessionCmdKindWire, SessionCmdWire};
use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
use brenn_reachy__driver__health_clk_rs::{
    AuxOutcomeWire, AuxStatusWire, DriverEventWire, EventKindWire, HealthReportWire,
};
use brenn_reachy__driver__pose_clk_rs::{PoseEstimateWire, PoseSampleWire};
use brenn_reachy__hardware__dynamixel__registers_clk_rs::RegIdWire;
use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKindWire;
use brenn_reachy__motion__faults_clk_rs::TickFaultWire;
use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;
use log_read::{Bound, Census, Complaints, Logged, Streams, binding, read_with, typed};
use motion_channels::{
    AUX_OUT_CHANNEL, CMD_CHANNEL, ESTIMATE_CHANNEL, EVENT_CHANNEL, FAULT_CHANNEL, HEALTH_CHANNEL,
    POSE_CHANNEL, REPORT_CHANNEL, SCHEDULE_CHANNEL, SCRIPT_CHANNEL, SESSION_CMD_CHANNEL,
};
use motion_slots::joint_set;
use nalgebra::Isometry3;
use reachy_driver::NOMINAL_CYCLE_NS;
use reachy_motion::joints::{JointGroup, ROW_COUNT, ROWS, flags, group_of, row, rows_of};
use reachy_motion::postures::{neutral_targets, stow_pose_targets};
use reachy_motion::record;
use reachy_motion::tick::{
    RECORDED_WORST_ANTENNA_LAG_RAD, RECORDED_WORST_HEAD_LAG_RAD, TrackingFaultConfig,
};

/// How far the head may be from the posture it was sent to, in metres, and still
/// count as having arrived.
///
/// Five millimetres, which is a machine's tolerance rather than a solver's: the
/// servos quantise at 0.088 degrees of crank, the linkage flexes under the
/// head's own weight, and a real leg sits wherever its load leaves it. A
/// scenario asserts a thousand times tighter than this because its plant is
/// arithmetic; asserting that here would fail every hardware run on physics.
///
/// Nobody has measured this machine. This figure and the two below were sized
/// from the mechanism on paper, so what they are is a guess of the right order;
/// a verdict that turns on one of them is a verdict about a number an agent
/// chose. The runbook's first-run checks say to confirm or reset all three
/// before reading a report as a pass or a fail.
const ARRIVAL_OFFSET_M: f64 = 5e-3;

/// How far the head's orientation may be from the posture it was sent to,
/// radians, on the same terms, and unmeasured on the same terms.
const ARRIVAL_TURN_RAD: f64 = 0.05;

/// How far an antenna may point away from where the posture puts it, radians.
///
/// Looser than the head's: an antenna is a thin rod on an unloaded shaft with no
/// travel limit, and where its tip ends up is the one thing on this machine
/// nothing measures except the servo's own encoder. Also unmeasured: the ratio
/// to the head's figure is an argument, not an observation.
const ARRIVAL_ANTENNA_RAD: f64 = 0.1;

// The tolerances above are a machine's and not a solver's, which is the whole
// reason this tool is separate from the scenario checkers. Held apart from the
// figures a deterministic run is asserted to, so tightening one to a scenario's
// is a deliberate edit rather than a report that fails every hardware run on
// physics.
const _: () = assert!(ARRIVAL_OFFSET_M >= 1e-3);
const _: () = assert!(ARRIVAL_TURN_RAD >= 1e-2);
const _: () = assert!(ARRIVAL_ANTENNA_RAD >= ARRIVAL_TURN_RAD);

/// Everything one run put in the log.
///
/// Every stream is lifted out of the reader's borrowed view before anything is
/// asked of it, so the whole run can be looked at at once -- which is what lets
/// the gesture be judged by where the machine went rather than by what happened
/// on a cycle somebody named.
#[derive(Default)]
struct Run {
    /// What was asked of the machine.
    scripts: Vec<Logged<ScriptWire>>,
    /// What the session accepted.
    schedules: Vec<Logged<SessionScheduleWire>>,
    /// The driver's heartbeat: one per cycle, always.
    samples: Vec<Logged<PoseSampleWire>>,
    /// What the machine was asked to hold next.
    goals: Vec<Logged<GoalSetpointWire>>,
    /// What the driver did that the sample stream does not show.
    events: Vec<Logged<DriverEventWire>>,
    /// What the decision tick raised.
    faults: Vec<Logged<TickFaultWire>>,
    /// What the session said about all of it.
    reports: Vec<Logged<TimelineEntryWire>>,
    /// What the session asked the driver for.
    datagrams: Vec<Logged<SessionCmdWire>>,
    /// Where the head was.
    estimates: Vec<Logged<PoseEstimateWire>>,
    /// How each out-of-band transaction turned out.
    outcomes: Vec<Logged<AuxOutcomeWire>>,
    /// What the health rotation read.
    readings: Vec<Logged<HealthReportWire>>,
    /// Every channel the log carries and how many messages each held: a channel
    /// with no Rust type bound to it still says whether anything travelled on
    /// it.
    census: Census,
    /// Anything that went wrong reading the log itself. Every one of these is a
    /// failure of the run.
    complaints: Complaints,
    /// How far a sample's nominal instant may sit off the run's own grid and
    /// still count as sitting on it, in nanoseconds.
    ///
    /// Zero, and that is the figure a machine's log is read with: a driver
    /// computes each cycle's instant from an absolute grid rather than measuring
    /// it, so its samples land on exact multiples of the period and any offset at
    /// all is arithmetic that stopped working.
    ///
    /// It is a knob because one thing that produces this log is not a driver: the
    /// host smoke run's plant is a cog woken by the wall-clock runner, and what it
    /// stamps a sample with is when it actually ran. That jitter is the runner's,
    /// not the system's, and a run of it says so on the command line.
    grid_jitter_ns: i64,
}

impl Streams for Run {
    fn census(&mut self) -> &mut Census {
        &mut self.census
    }

    fn complaints(&mut self) -> &mut Complaints {
        &mut self.complaints
    }
}

/// Every channel this analyzer reads, which is the set both the simulated and
/// the online system declare a logging policy for.
///
/// Not the scenario harness's own table: that one binds the injection channel
/// only the simulated system has, and a channel the log cannot carry would be
/// reported as missing on every hardware run. Two channels the harness does not
/// read are bound here -- the out-of-band outcomes and the health readings --
/// because an operator's first question about a real machine is what the health
/// rotation saw.
const CHANNELS: [Bound<Run>; 11] = [
    Bound {
        name: SCRIPT_CHANNEL,
        check: binding::<ScriptWire>,
        route: |run, message| typed(message, &mut run.scripts, &mut run.complaints),
    },
    Bound {
        name: SCHEDULE_CHANNEL,
        check: binding::<SessionScheduleWire>,
        route: |run, message| typed(message, &mut run.schedules, &mut run.complaints),
    },
    Bound {
        name: POSE_CHANNEL,
        check: binding::<PoseSampleWire>,
        route: |run, message| typed(message, &mut run.samples, &mut run.complaints),
    },
    Bound {
        name: CMD_CHANNEL,
        check: binding::<GoalSetpointWire>,
        route: |run, message| typed(message, &mut run.goals, &mut run.complaints),
    },
    Bound {
        name: EVENT_CHANNEL,
        check: binding::<DriverEventWire>,
        route: |run, message| typed(message, &mut run.events, &mut run.complaints),
    },
    Bound {
        name: FAULT_CHANNEL,
        check: binding::<TickFaultWire>,
        route: |run, message| typed(message, &mut run.faults, &mut run.complaints),
    },
    Bound {
        name: REPORT_CHANNEL,
        check: binding::<TimelineEntryWire>,
        route: |run, message| typed(message, &mut run.reports, &mut run.complaints),
    },
    Bound {
        name: SESSION_CMD_CHANNEL,
        check: binding::<SessionCmdWire>,
        route: |run, message| typed(message, &mut run.datagrams, &mut run.complaints),
    },
    Bound {
        name: ESTIMATE_CHANNEL,
        check: binding::<PoseEstimateWire>,
        route: |run, message| typed(message, &mut run.estimates, &mut run.complaints),
    },
    Bound {
        name: AUX_OUT_CHANNEL,
        check: binding::<AuxOutcomeWire>,
        route: |run, message| typed(message, &mut run.outcomes, &mut run.complaints),
    },
    Bound {
        name: HEALTH_CHANNEL,
        check: binding::<HealthReportWire>,
        route: |run, message| typed(message, &mut run.readings, &mut run.complaints),
    },
];

impl Run {
    /// Read the log under `dir`.
    ///
    /// # Errors
    ///
    /// Whatever the shared pass refuses about the log as a whole. A message the
    /// reader yielded and this build could not make sense of is a complaint
    /// rather than an error: the point of the report is to state everything at
    /// once.
    fn read(dir: &Path) -> Result<Self, clockwork_logs::LogError> {
        read_with(dir, &CHANNELS)
    }
}

/// The report as it is being built: what the run failed, and what it measured.
///
/// Two lists rather than one, because they are read for different reasons. A
/// finding is a claim about the run that did not hold and is what the exit
/// status is about; a measurement is a number the run produced, printed whether
/// the run passed or not. A first hardware run that fails is exactly the run
/// whose numbers somebody needs.
#[derive(Default)]
struct Report {
    findings: Vec<String>,
    measured: Vec<String>,
}

impl Report {
    /// One way the run did not do what it claims to have done.
    fn fail(&mut self, what: impl Into<String>) {
        self.findings.push(what.into());
    }

    /// One number the run produced.
    fn note(&mut self, what: impl Into<String>) {
        self.measured.push(what.into());
    }
}

/// The grid the run's samples sit on, derived from the samples themselves.
///
/// A hardware run starts at whatever top of a second the driver started at, so
/// nothing about the epoch can be assumed; what can be is the period, which is
/// the one number both hosts are built against. The origin is the first sample's
/// own nominal instant.
struct Grid {
    origin_ns: i64,
    period_ns: i64,
}

impl Grid {
    /// The cycle index of a nominal instant, and how far off the grid it sits.
    fn at(&self, nominal_ns: i64) -> (i64, i64) {
        let elapsed = nominal_ns - self.origin_ns;
        (
            elapsed.div_euclid(self.period_ns),
            elapsed.rem_euclid(self.period_ns),
        )
    }

    /// The same, with an instant within `jitter_ns` of a cycle counted as being
    /// on it.
    ///
    /// An instant that arrived late is over its own cycle's mark by the offset;
    /// one that arrived early is under the *next* cycle's, which the remainder
    /// reports as nearly a whole period. So both ends of the band are checked and
    /// the answer is the cycle the instant is nearest, with a zero offset when it
    /// is inside the band.
    fn within(&self, nominal_ns: i64, jitter_ns: i64) -> (i64, i64) {
        let (cycle, off) = self.at(nominal_ns);
        if off <= jitter_ns {
            (cycle, 0)
        } else if off >= self.period_ns - jitter_ns {
            (cycle + 1, 0)
        } else {
            (cycle, off)
        }
    }
}

/// The driver's heartbeat: one sample per cycle, on one grid, without a gap.
///
/// The assertion every other one rests on, for the reason the scenario harness
/// states: the sample stream is the clock the control cogs run on, so a hole in
/// it is a control cycle that never happened. Unlike a scenario's, this one
/// derives the grid rather than knowing it, and reports the gaps instead of
/// naming the cycles they should have been on.
///
/// Three different failures, each said in its own words: a sample that does not
/// sit on the grid at all, a cycle the stream has already read, and a cycle
/// nothing was published on.
fn heartbeat(run: &Run, report: &mut Report) -> Option<Grid> {
    let first = run.samples.first()?;
    let grid = Grid {
        origin_ns: first.message.nominal_time().as_nanos(),
        period_ns: NOMINAL_CYCLE_NS,
    };
    let mut expected = 0_i64;
    let mut gaps = 0_usize;
    let mut off_grid = 0_usize;
    let mut repeated = 0_usize;
    for sample in &run.samples {
        let at = sample.message.nominal_time().as_nanos();
        let (cycle, off) = grid.within(at, run.grid_jitter_ns);
        // Whatever the sample turns out to be, the cycle it names is one the
        // stream has now reached: the position never moves backwards and never
        // stays put across a sample that failed a check of its own. A cycle
        // counted twice is a gap invented at every sample after it.
        let reached = expected.max(cycle + 1);
        if off != 0 {
            off_grid += 1;
            if off_grid == 1 {
                report.fail(format!(
                    "the sample at {at} sits {off} ns off the {} ns grid the run's first sample \
                     starts",
                    grid.period_ns
                ));
            }
            expected = reached;
            continue;
        }
        // Two different runs, said differently: a cycle short of where the
        // stream has got to is a repeated or reordered sample, and a cycle past
        // it is a hole. Folding them together prints a skip of a negative
        // number of cycles and points an operator at the wrong problem.
        if cycle < expected {
            repeated += 1;
            if repeated == 1 {
                report.fail(format!(
                    "the sample at {at} is cycle {cycle}, and the stream has already read cycle \
                     {}: the log repeats a cycle or carries them out of order",
                    expected - 1
                ));
            }
        } else if cycle > expected {
            gaps += 1;
            if gaps <= 4 {
                report.fail(format!(
                    "the driver's heartbeat skips from cycle {} to cycle {cycle}: {} cycles the \
                     control cogs never ran on",
                    expected - 1,
                    cycle - expected
                ));
            }
        }
        expected = reached;
    }
    if gaps > 4 {
        report.fail(format!(
            "the driver's heartbeat has {gaps} gaps in it, of which four are named above"
        ));
    }
    if off_grid > 1 {
        report.fail(format!(
            "{off_grid} samples sit off the run's own grid, of which one is named above"
        ));
    }
    if repeated > 1 {
        report.fail(format!(
            "{repeated} samples name a cycle the stream had already read, of which one is named \
             above"
        ));
    }
    report.note(format!(
        "{} samples over {} cycles, {:.3} s of run",
        run.samples.len(),
        expected,
        (expected * grid.period_ns) as f64 / 1e9
    ));
    Some(grid)
}

/// How far the bus read ran from the instant it was meant to run at.
///
/// The jitter measurement the driver takes for free on every cycle: a sample
/// carries both instants, so the difference is the whole of it. Reported and
/// never failed -- what an acceptable figure is on this machine is what the
/// first runs are for, and this is the tool that says what it was.
fn jitter(run: &Run, report: &mut Report) {
    let mut worst = 0_i64;
    let mut total = 0_i64;
    let mut counted = 0_i64;
    for sample in &run.samples {
        let late =
            sample.message.sample_time().as_nanos() - sample.message.nominal_time().as_nanos();
        worst = worst.max(late.abs());
        total += late;
        counted += 1;
    }
    if counted == 0 {
        return;
    }
    report.note(format!(
        "read jitter: worst {:.3} ms from nominal, mean {:.3} ms",
        worst as f64 / 1e6,
        (total / counted) as f64 / 1e6
    ));
}

/// What the bus answered, and what it did not.
///
/// Two numbers an operator wants first out of any run: how many
/// cycles read every row, and which rows are the ones that go missing. A row
/// that answers nothing all run is a wiring or an id problem, and it reads here
/// as its own line rather than as a share of one total.
fn reads(run: &Run, report: &mut Report) {
    let mut blind = 0_usize;
    let mut partial = 0_usize;
    let mut undecodable = 0_usize;
    let mut clean = 0_usize;
    let mut per_row = [0_usize; 9];
    for sample in &run.samples {
        let Ok(missing) = joint_set(sample.message.missing()) else {
            // Counted in a bucket of its own. A sample whose missing set this
            // build cannot read says nothing about how the read went, and
            // leaving it out of the arithmetic would have printed it as a clean
            // one -- a number contradicting the finding beside it.
            undecodable += 1;
            report.fail(format!(
                "the sample at {} names a set of servos this build cannot read",
                sample.message.nominal_time().as_nanos()
            ));
            continue;
        };
        if flags::is_empty(missing) {
            clean += 1;
            continue;
        }
        if sample.message.present_valid() {
            partial += 1;
        } else {
            blind += 1;
        }
        for joint in flags::iter(missing) {
            if let Some(index) = row(joint) {
                per_row[index] += 1;
            }
        }
    }
    report.note(format!(
        "reads: {clean} clean, {partial} partial, {blind} answered nothing, {undecodable} naming a \
         set this build cannot read"
    ));
    for joint in ROWS {
        let Some(index) = row(joint) else { continue };
        if per_row[index] > 0 {
            report.note(format!(
                "  {joint:?} did not answer on {} of {} cycles",
                per_row[index],
                run.samples.len()
            ));
        }
    }
}

/// How far the machine ran behind what it was told to hold.
///
/// Off the samples alone: each carries the setpoint the driver is holding beside
/// the position it read, so the lag needs no join against the goal stream. Two
/// figures, head and antennas, because those are the two the recorded hardware
/// gestures pinned and the tracking screen is sized against -- and both of those
/// numbers are printed beside the measurement, so a run can be read against the
/// only hardware evidence this repo has.
fn lags(run: &Run, report: &mut Report) {
    let mut head = 0_f64;
    let mut antenna = 0_f64;
    let mut compared = 0_usize;
    for sample in &run.samples {
        if !sample.message.present_valid() || !sample.message.commanded_valid() {
            continue;
        }
        let Ok(present) = sample.message.present().validate().map(rows_of) else {
            continue;
        };
        let Ok(commanded) = sample.message.commanded().validate().map(rows_of) else {
            continue;
        };
        compared += 1;
        for joint in ROWS {
            let Some(index) = row(joint) else { continue };
            let lag = (commanded[index] - present[index]).abs();
            match group_of(joint) {
                Some(JointGroup::Antennas) => antenna = antenna.max(lag),
                Some(_) => head = head.max(lag),
                None => {}
            }
        }
    }
    let threshold = TrackingFaultConfig::default().threshold_rad;
    // How many samples the two figures came off, because a zero lag and a
    // measurement nothing was compared on print the same otherwise -- and a run
    // in which the driver held nothing is the second one.
    report.note(format!(
        "{compared} of {} samples carried both a reading and a setpoint to compare",
        run.samples.len()
    ));
    report.note(format!(
        "worst head lag {head:.4} rad; the recorded healthy gesture ran at \
         {RECORDED_WORST_HEAD_LAG_RAD:.4} rad and the tracking screen sits at {threshold:.4} rad"
    ));
    report.note(format!(
        "worst antenna lag {antenna:.4} rad; the recorded fast sweep ran at \
         {RECORDED_WORST_ANTENNA_LAG_RAD:.4} rad"
    ));
}

/// The session's narration, in the order the wake gesture puts it in.
///
/// The five phase changes are the whole of the gesture's shape: the survey ends,
/// a script is taken, the machine is armed, the schedule runs out, and the
/// session lets go. Matched in order, pair for pair, so a run that took a
/// different path through the phases fails rather than passing on a subset.
///
/// Answers the instant the machine came under command and the instant it was
/// released, which is what the postures below are looked for between.
fn narration(run: &Run, report: &mut Report) -> Option<(i64, i64)> {
    let wanted = [
        (SessionPhaseWire::RESTING, SessionPhaseWire::STARTING),
        (SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
        (SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING),
        (SessionPhaseWire::STOPPING, SessionPhaseWire::ACTIVE),
        (SessionPhaseWire::RESTING, SessionPhaseWire::STOPPING),
    ];
    let mut seen: Vec<i64> = Vec::new();
    for entry in &run.reports {
        if entry.message.kind() != ReportKindWire::PHASE_CHANGED {
            continue;
        }
        let at = entry.message.time().as_nanos();
        let to = SessionPhaseWire(u8::try_from(entry.message.a()).unwrap_or(u8::MAX));
        let from = SessionPhaseWire(u8::try_from(entry.message.b()).unwrap_or(u8::MAX));
        match wanted.get(seen.len()) {
            Some((want_to, want_from)) if (to, from) == (*want_to, *want_from) => {}
            Some((want_to, want_from)) => report.fail(format!(
                "phase change {} is {from:?} to {to:?} at {at}, and the wake gesture's is \
                 {want_from:?} to {want_to:?}",
                seen.len() + 1
            )),
            None => report.fail(format!(
                "the session moved from {from:?} to {to:?} at {at}, past the {} changes the wake \
                 gesture has",
                wanted.len()
            )),
        }
        seen.push(at);
    }
    if seen.len() < wanted.len() {
        report.fail(format!(
            "the session narrated {} phase changes, and the wake gesture has {}: the run did not \
             get through it",
            seen.len(),
            wanted.len()
        ));
    }
    let taken = seen.get(2).copied();
    let released = seen.get(3).copied();
    match (taken, released) {
        (Some(taken), Some(released)) => Some((taken, released)),
        _ => None,
    }
}

/// What else the session said, and what it must not have said.
///
/// The gesture's own narration is one script accepted, two schedules published
/// and a session that ended -- and nothing else. Every other kind is named
/// rather than tolerated: a refused script, a recorded fault, a response taken,
/// a de-torquing that went unconfirmed and a bus declared failed are each a
/// different run from the one this tool is here to confirm, and each is worth
/// saying in its own words.
fn what_the_session_said(run: &Run, report: &mut Report) {
    // Keyed on the wire value, so the census's grouping and its order are the
    // vocabulary's own rather than the alphabet its variant names happen to
    // fall in. The name is written once, at print time.
    let mut census: BTreeMap<u8, usize> = BTreeMap::new();
    for entry in &run.reports {
        let kind = entry.message.kind();
        *census.entry(kind.0).or_default() += 1;
        let at = entry.message.time().as_nanos();
        let (a, b, detail) = (entry.message.a(), entry.message.b(), entry.message.detail());
        match kind {
            ReportKindWire::PHASE_CHANGED
            | ReportKindWire::SCRIPT_ACCEPTED
            | ReportKindWire::SCHEDULE_PUBLISHED
            | ReportKindWire::TORQUE_OFF_CONFIRMED => {}
            ReportKindWire::SESSION_ENDED => match joint_set_of(b) {
                Some(unread) => report.note(format!(
                    "the session ended at {at}: script {a}, {} unread at the release, worst \
                     deviation from stow {detail:.4} rad",
                    flags::Names(unread)
                )),
                None => report.fail(format!(
                    "the session's ending report at {at} names servo set {b:#x}, which this build \
                     cannot read: what it left unread at the release is undecodable"
                )),
            },
            ReportKindWire::SCRIPT_REFUSED => report.fail(format!(
                "the session refused script {a} at {at}, reason {b}: nothing moved, and nothing \
                 retries"
            )),
            ReportKindWire::FAULT_RECORDED => report.fail(format!(
                "the session recorded fault {a} at {at} on servo {b}, magnitude {detail}"
            )),
            ReportKindWire::RESPONSE_TAKEN => report.fail(format!(
                "the session took response {a} at {at}, for fault {b}: the gesture was answered \
                 rather than run"
            )),
            ReportKindWire::TORQUE_OFF_UNCONFIRMED => report.fail(format!(
                "the session could not confirm the de-torquing at {at}: {a} rows unread after \
                 {detail:.3} s of trying"
            )),
            ReportKindWire::BUS_FAILURE_DECLARED => report.fail(format!(
                "the session declared the bus failed at {at}, {detail:.3} s since a fresh sample"
            )),
            other => report.fail(format!(
                "the session reported {other:?} at {at} ({a}, {b}, {detail}), and a clean wake \
                 gesture says none of it"
            )),
        }
    }
    for (kind, count) in census {
        report.note(format!("  {:?} x{count}", ReportKindWire::from(kind)));
    }
    if run.scripts.len() != 1 {
        report.fail(format!(
            "{} scripts reached the session, and the wake gesture asks exactly once",
            run.scripts.len()
        ));
    }
}

/// The joints a report's second number names, or `None` if it names something
/// this build cannot read.
///
/// `None` and the empty set are two different answers and are never folded
/// together: the empty set is the machine having been let go of with nothing
/// left unread, which is the most reassuring reading there is, and a bit pattern
/// this build has no servos for is a report written against a vocabulary this
/// tool does not have. Answering the first for the second would launder an
/// unexpected value into a clean verdict on the one check that stands between a
/// green report and a machine nobody let go of.
fn joint_set_of(bits: u32) -> Option<brenn_reachy__motion__joints_clk_rs::JointFlags> {
    u16::try_from(bits)
        .ok()
        .map(brenn_reachy__motion__joints_clk_rs::JointFlagsWire)
        .and_then(|flags| joint_set(flags).ok())
}

/// The decision tick raised nothing.
fn no_faults(run: &Run, report: &mut Report) {
    for fault in &run.faults {
        report.fail(format!(
            "the decision tick raised {:?} at {}, and nothing in a wake gesture is wrong with the \
             machine",
            fault.message.kind(),
            fault.message.time().as_nanos()
        ));
    }
}

/// What the driver said about itself.
///
/// Five kinds fail the run, and each for its own reason: a bus that answered
/// nothing, a cycle that did not run in its slot, a dead-man that latched, a goal
/// dropped for want of queue, and a de-torquing no row confirmed. The rest are
/// censused -- a startup minimum-risk write is what a driver started before the
/// cogs is *supposed* to do, and a confirmed release is the evidence the gesture
/// ended the way it claims.
fn driver_events(run: &Run, report: &mut Report) {
    let mut census: BTreeMap<u8, usize> = BTreeMap::new();
    for event in &run.events {
        let kind = event.message.kind();
        *census.entry(kind.0).or_default() += 1;
        let at = event.message.time().as_nanos();
        let silence = event.message.silence().as_nanos() as f64 / 1e6;
        match kind {
            EventKindWire::BUS_FAILURE => report.fail(format!(
                "the driver declared its bus gone at {at}, after {} cycles nothing answered on",
                event.message.count()
            )),
            EventKindWire::CYCLE_SKIPPED => report.fail(format!(
                "the driver missed {} cycle slots at {at}, running {silence:.3} ms late",
                event.message.count()
            )),
            EventKindWire::HOLD_TIMEOUT_TORQUE_OFF => report.fail(format!(
                "the driver's dead-man latched at {at} after {silence:.3} ms of silence: the goal \
                 stream stopped while the machine was under command"
            )),
            EventKindWire::GOAL_DROPPED_QUEUE_FULL => report.fail(format!(
                "a goal was dropped at {at} with {} queued: the driver could not keep up with the \
                 commander",
                event.message.count()
            )),
            EventKindWire::TORQUE_OFF_UNCONFIRMED => report.fail(format!(
                "the driver could not confirm the de-torquing at {at}: {} did not read back",
                flags::Names(joint_set(event.message.rows()).unwrap_or_default())
            )),
            EventKindWire::STARTUP_MRC_WRITE => report.note(format!(
                "the driver wrote the minimum risk condition at start-up, at {at}, after \
                 {silence:.3} ms of nobody talking to it"
            )),
            EventKindWire::GOAL_STALE_OR_OUT_OF_ORDER => report.note(format!(
                "a goal arrived {silence:.3} ms past its instant at {at}, and was executed anyway"
            )),
            EventKindWire::TORQUE_OFF_CONFIRMED => {
                report.note(format!("the driver read every row back de-torqued at {at}"))
            }
            other => report.fail(format!(
                "the driver raised {other:?} at {at}, which this report has no reading for"
            )),
        }
    }
    for (kind, count) in census {
        report.note(format!("  {:?} x{count}", EventKindWire::from(kind)));
    }
}

/// The gesture itself: the machine went up, and then it stowed.
///
/// Judged by where the machine was measured rather than by when it was supposed
/// to be there. Between the instant the session took hold and the instant it let
/// go, the estimate stream is searched for a sample that is upright within both
/// tolerances at once and, after that, for one that is stowed on the same terms.
/// Each best approach is printed too, so a run that missed says by how much.
///
/// Order is part of the claim: the stow arrival is searched from the upright one
/// onwards, so a machine that folded and then stood up does not read as a
/// gesture performed.
fn the_gesture(run: &Run, engaged: Option<(i64, i64)>, report: &mut Report) {
    let Some((taken, released)) = engaged else {
        return;
    };
    let poses: Vec<(i64, Isometry3<f64>)> = run
        .estimates
        .iter()
        .filter(|estimate| {
            let at = estimate.message.time_of_validity().as_nanos();
            at >= taken && at <= released && estimate.message.valid()
        })
        .filter_map(|estimate| {
            solved_pose(&estimate.message)
                .map(|pose| (estimate.message.time_of_validity().as_nanos(), pose))
        })
        .collect();
    if poses.is_empty() {
        report.fail(
            "no estimate between the engagement and the release solved to a pose: nothing in the \
             log says where the machine went"
                .to_string(),
        );
        return;
    }
    let Some(up) = closest(&poses, 0, "upright", &neutral_targets(), report) else {
        return;
    };
    let from = poses.iter().position(|(at, _)| *at >= up).unwrap_or(0);
    closest(&poses, from, "stowed", &stow_pose_targets(), report);
    antennas_arrived(run, released, report);
}

/// Where the head came to `wanted` from `from` onwards, reported and judged
/// against the hardware tolerances. Answers the instant it arrived, or the
/// instant of its best approach where it never did.
///
/// Arrival is both tolerances at once, on one sample: a machine 4 mm away but
/// badly turned has not arrived, and a machine that arrived is a sample that was
/// close in translation *and* aligned. The sample reported is the best by one
/// score over both components, so the printed number and the verdict come from
/// the same definition -- ranking on translation alone would report a
/// well-placed, badly turned sample as the closest approach and then fail the
/// run on it while a further, aligned sample went unmentioned.
fn closest(
    poses: &[(i64, Isometry3<f64>)],
    from: usize,
    what: &str,
    wanted: &reachy_motion::joints::JointTargets,
    report: &mut Report,
) -> Option<i64> {
    let mut best: Option<(i64, f64, f64, f64)> = None;
    let mut arrived: Option<(i64, f64, f64)> = None;
    for (at, pose) in &poses[from.min(poses.len())..] {
        let offset = (pose.translation.vector - wanted.head_pose_body.translation.vector).norm();
        let turn = pose.rotation.angle_to(&wanted.head_pose_body.rotation);
        // Each component against its own tolerance, summed: a score of one is
        // the tolerance box's corner, and nothing about it is a distance.
        let score = offset / ARRIVAL_OFFSET_M + turn / ARRIVAL_TURN_RAD;
        if best.is_none_or(|(_, _, _, was)| score < was) {
            best = Some((*at, offset, turn, score));
        }
        if arrived.is_none() && offset <= ARRIVAL_OFFSET_M && turn <= ARRIVAL_TURN_RAD {
            arrived = Some((*at, offset, turn));
        }
    }
    let (best_at, offset, turn, _) = best?;
    report.note(format!(
        "closest to {what}: {offset:.4} m and {turn:.4} rad away, at {best_at}"
    ));
    let Some((at, offset, turn)) = arrived else {
        report.fail(format!(
            "the head never came within {ARRIVAL_OFFSET_M} m and {ARRIVAL_TURN_RAD} rad of \
             {what} on one sample: its best approach was {offset:.4} m and {turn:.4} rad, at \
             {best_at}"
        ));
        return Some(best_at);
    };
    report.note(format!(
        "reached {what} at {at}: {offset:.4} m and {turn:.4} rad away"
    ));
    Some(at)
}

/// The antennas ended up where stow puts them.
///
/// Read off the last sample the machine was measured on rather than off the
/// estimate stream: the estimator solves for the head, and where an antenna
/// points is only ever its own encoder's answer. Compared as a direction, since
/// an antenna runs in extended position mode and a correct arrival can be a whole
/// turn from the number the posture states.
///
/// Finding the sample is this function's half; judging it is
/// [`judge_antennas`], which takes the rows as the reading they are or the
/// absence of one, so both of its answers can be exercised without a schema
/// that refuses bytes.
fn antennas_arrived(run: &Run, released: i64, report: &mut Report) {
    let last = run.samples.iter().rev().find(|sample| {
        sample.message.nominal_time().as_nanos() <= released && sample.message.present_valid()
    });
    let Some(sample) = last else {
        report.fail(
            "no complete reading before the release: nothing says where the antennas ended up"
                .to_string(),
        );
        return;
    };
    judge_antennas(
        sample.message.present().validate().ok().map(rows_of),
        report,
    );
}

/// Where the antennas ended, from the last measured rows this build could read.
///
/// `None` is a reading that failed wire validation, and it fails the run rather
/// than being passed over: "nobody can tell where the antennas ended" and "they
/// ended correctly" must not read alike. Nothing in this tree produces it —
/// `Joints` is nine plain numbers and its generated validation accepts any
/// bytes — so it is a guard against a schema that later constrains a field.
fn judge_antennas(present: Option<[f64; ROW_COUNT]>, report: &mut Report) {
    let Some(present) = present else {
        report.fail(
            "the last complete reading before the release carries measured rows this build \
             cannot read: nothing says where the antennas ended up"
                .to_string(),
        );
        return;
    };
    let wanted = stow_pose_targets();
    for (joint, slot) in [
        (reachy_motion::joints::JointRef::AntennaRight, 0),
        (reachy_motion::joints::JointRef::AntennaLeft, 1),
    ] {
        let Some(index) = row(joint) else { continue };
        let error = reachy_kin::wrap_to_pi(present[index] - wanted.antennas[slot]).abs();
        report.note(format!("{joint:?} ended {error:.4} rad from stow"));
        if error > ARRIVAL_ANTENNA_RAD {
            report.fail(format!(
                "{joint:?} ended {error:.4} rad from where stow puts it, past the \
                 {ARRIVAL_ANTENNA_RAD} rad this report allows"
            ));
        }
    }
}

/// The machine was let go of, and the release was read back.
///
/// The one failure on this machine that can hurt somebody is a gesture that
/// ended with the machine energised, so it is checked from what both ends of the
/// seam published rather than from either alone -- and both legitimate ways of
/// letting go are accepted, because a run answering the safer of the two is not
/// a run that failed.
///
/// The ordinary ending is the session's own: its wind-down writes torque off row
/// by row as a verified write, so the servo's read-back is the evidence, and the
/// outcome the driver sent back for that transaction is where the read-back
/// landed. All nine rows, each answered `ok`.
///
/// The other ending is the driver's gate: a commanded release, or the dead-man,
/// latches torque off for the whole machine at once, and every sample after it
/// says so. A latch is reported and nothing about it fails -- and a latch that
/// went away again does, because nothing clears one.
fn the_release(run: &Run, report: &mut Report) {
    let answers: BTreeMap<u32, AuxStatusWire> = run
        .outcomes
        .iter()
        .map(|outcome| (outcome.message.corr(), outcome.message.status()))
        .collect();
    let mut released: BTreeMap<u8, AuxStatusWire> = BTreeMap::new();
    for datagram in &run.datagrams {
        let txn = datagram.message.txn();
        if datagram.message.kind() != SessionCmdKindWire::AUX
            || txn.op() != AuxOpKindWire::WRITE_REG_VERIFIED
            || txn.reg() != RegIdWire::TORQUE_ENABLE
            || txn.value() != 0
        {
            continue;
        }
        let answered = answers
            .get(&datagram.message.corr())
            .copied()
            .unwrap_or(AuxStatusWire::TIMEOUT);
        // The best answer any row got: a release re-issued after a lost
        // transaction is one release, and the reading that counts is the one
        // that came back.
        released
            .entry(txn.id())
            .and_modify(|status| {
                if answered == AuxStatusWire::OK {
                    *status = answered;
                }
            })
            .or_insert(answered);
    }
    let confirmed = released
        .values()
        .filter(|status| **status == AuxStatusWire::OK)
        .count();
    report.note(format!(
        "the session wrote torque off on {} rows, {confirmed} of them read back",
        released.len()
    ));
    for (id, status) in &released {
        if *status != AuxStatusWire::OK {
            report.note(format!("  servo {id}'s release answered {status:?}"));
        }
    }

    let latched = run
        .samples
        .iter()
        .find(|sample| sample.message.torque_off_latched())
        .map(|sample| sample.message.nominal_time().as_nanos());
    if let Some(at) = latched {
        report.note(format!("the driver latched torque off at {at}"));
        for sample in &run.samples {
            let sample_at = sample.message.nominal_time().as_nanos();
            if sample_at > at && !sample.message.torque_off_latched() {
                report.fail(format!(
                    "the driver's latch was gone again by {sample_at}, and nothing clears one"
                ));
                break;
            }
        }
    }
    if confirmed < ROWS.len() && latched.is_none() {
        report.fail(format!(
            "the machine was not let go of: {confirmed} of {} rows read back de-torqued and the \
             driver's gate never latched",
            ROWS.len()
        ));
    }
    if !run.reports.iter().any(|entry| {
        entry.message.kind() == ReportKindWire::SESSION_ENDED
            && joint_set_of(entry.message.b()).is_some_and(flags::is_empty)
    }) {
        report.fail(
            "no session ended with every row measured: the ending report is where the session \
             says it read the machine at rest"
                .to_string(),
        );
    }
}

/// What the health rotation saw, per servo.
///
/// The last reading each servo gave, which is the picture of the machine at the
/// end of the run. A latched error byte is a finding: the rotation is how this
/// stack learns a servo is complaining, and a run that ended with one complaining
/// is a run somebody should look at before the next.
fn health(run: &Run, report: &mut Report) {
    let mut latest: BTreeMap<u8, &HealthReportWire> = BTreeMap::new();
    for reading in &run.readings {
        latest.insert(reading.message.id(), &reading.message);
    }
    if latest.is_empty() {
        report.note("the health rotation reported nothing".to_string());
        return;
    }
    for (id, reading) in latest {
        report.note(format!(
            "servo {id}: {:.2} V, {} C, error byte 0x{:02x}",
            reading.volts(),
            reading.temp_c(),
            reading.bits()
        ));
        if reading.bits() != 0 {
            report.fail(format!(
                "servo {id} ended the run with error byte 0x{:02x} latched",
                reading.bits()
            ));
        }
    }
}

/// How the out-of-band transactions went, by status.
///
/// A census and not a judgement, with one exception: the statuses that mean
/// nothing reached the servo are the ones that say the bus is not carrying what
/// the session asks of it, and those are worth a line of their own.
fn transactions(run: &Run, report: &mut Report) {
    let mut census: BTreeMap<u8, usize> = BTreeMap::new();
    for outcome in &run.outcomes {
        *census.entry(outcome.message.status().0).or_default() += 1;
    }
    report.note(format!(
        "{} out-of-band transactions answered",
        run.outcomes.len()
    ));
    for (status, count) in census {
        report.note(format!("  {:?} x{count}", AuxStatusWire::from(status)));
    }
}

/// The pose an estimate describes, whatever its own flag says.
fn solved_pose(estimate: &PoseEstimateWire) -> Option<Isometry3<f64>> {
    let estimate = estimate.validate().ok()?;
    record::read_pose(&estimate.head_pos, &estimate.head_quat).ok()
}

/// Every check and every measurement this tool makes, over one run.
///
/// One function so the order is stated once: the log is checked for being
/// readable at all, then the heartbeat everything else rests on, then what the
/// two hosts said, then the gesture, then the numbers.
fn analyze(run: &Run) -> Report {
    let mut report = Report::default();
    for complaint in &run.complaints {
        report.fail(complaint.clone());
    }
    if run.samples.is_empty() {
        report.fail(
            "the log carries no samples: the driver's heartbeat is the clock every other stream \
             is read against, and there is no run without it"
                .to_string(),
        );
        return report;
    }
    heartbeat(run, &mut report);
    let engaged = narration(run, &mut report);
    what_the_session_said(run, &mut report);
    no_faults(run, &mut report);
    driver_events(run, &mut report);
    the_gesture(run, engaged, &mut report);
    the_release(run, &mut report);
    jitter(run, &mut report);
    reads(run, &mut report);
    lags(run, &mut report);
    health(run, &mut report);
    transactions(run, &mut report);
    for (name, count) in &run.census {
        report.note(format!("channel {name}: {count} messages"));
    }
    report
}

/// Read the log named on the command line, judge it, and print both halves.
///
/// The measurements go to stdout and the findings to stderr, so a run's numbers
/// can be filed with the run record while the findings are what an operator sees
/// on the terminal. The exit status is the verdict.
fn main() -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (log_dir, jitter_ns) = match args.as_slice() {
        [dir] => (dir, 0_i64),
        [flag, value, dir] if flag == "--grid-jitter-ns" => match value.parse::<i64>() {
            Ok(ns) if ns >= 0 => (dir, ns),
            _ => {
                eprintln!("--grid-jitter-ns takes a whole number of nanoseconds, not {value}");
                return ExitCode::FAILURE;
            }
        },
        _ => {
            eprintln!("usage: first_motion_report [--grid-jitter-ns <n>] <log-dir>");
            return ExitCode::FAILURE;
        }
    };
    let run = match Run::read(&PathBuf::from(log_dir)) {
        Ok(run) => run,
        Err(err) => {
            eprintln!("reading the log under {log_dir}: {err}");
            return ExitCode::FAILURE;
        }
    };
    let run = Run {
        grid_jitter_ns: jitter_ns,
        ..run
    };
    let report = analyze(&run);
    println!("first_motion_report over {log_dir}");
    for line in &report.measured {
        println!("{line}");
    }
    if report.findings.is_empty() {
        println!("the wake gesture happened, whole");
        return ExitCode::SUCCESS;
    }
    for finding in &report.findings {
        eprintln!("first_motion_report: {finding}");
    }
    eprintln!(
        "first_motion_report: {} findings over {log_dir}",
        report.findings.len()
    );
    ExitCode::FAILURE
}

#[cfg(test)]
mod tests {
    use super::{
        ARRIVAL_ANTENNA_RAD, AuxOpKindWire, AuxOutcomeWire, AuxStatusWire, CHANNELS,
        DriverEventWire, EventKindWire, Grid, HealthReportWire, Logged, PoseEstimateWire,
        PoseSampleWire, RegIdWire, Report, ReportKindWire, Run, SessionCmdKindWire, SessionCmdWire,
        SessionPhaseWire, TickFaultWire, TimelineEntryWire, analyze, joint_set_of, judge_antennas,
        neutral_targets, record, row, stow_pose_targets,
    };
    use brenn_reachy__motion__faults_clk_rs::FaultKindWire;
    use brenn_reachy__motion__joints_clk_rs::JointFlagsWire;
    use clockwork_rs::SyncTime;
    use nalgebra::{Isometry3, UnitQuaternion, Vector3};
    use reachy_driver::NOMINAL_CYCLE_NS;
    use reachy_motion::joints::{JointRef, ROW_COUNT, flags, write_rows};

    /// An arbitrary instant a synthetic run starts at, chosen for being nothing
    /// round: the analyzer derives its grid, so a run beginning off any tidy
    /// number is the case worth writing.
    const T0: i64 = 1_772_000_000_123_456_789;

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

    /// One sample of the heartbeat: cycle `n`, read cleanly, holding nothing.
    fn sample(n: i64) -> PoseSampleWire {
        let mut sample = PoseSampleWire::new();
        sample.set_nominal_time(when(n));
        sample.set_sample_time(when(n));
        sample.set_present_valid(true);
        sample
    }

    /// The driver's heartbeat, `cycles` of it, reading cleanly and holding
    /// nothing.
    fn heartbeat(cycles: i64) -> Vec<Logged<PoseSampleWire>> {
        (0..cycles).map(|n| at(n, sample(n))).collect()
    }

    /// One thing the session said, with the two numbers its kind reads.
    fn said(n: i64, kind: ReportKindWire, a: u32, b: u32) -> Logged<TimelineEntryWire> {
        let mut entry = TimelineEntryWire::new();
        entry.set_time(when(n));
        entry.set_kind(kind);
        entry.set_a(a);
        entry.set_b(b);
        at(n, entry)
    }

    /// One phase change, as the session narrates it: `a` the phase entered, `b`
    /// the one left.
    fn phase(n: i64, to: SessionPhaseWire, from: SessionPhaseWire) -> Logged<TimelineEntryWire> {
        said(
            n,
            ReportKindWire::PHASE_CHANGED,
            u32::from(to.0),
            u32::from(from.0),
        )
    }

    /// One thing the driver says it did.
    fn event(n: i64, kind: EventKindWire) -> Logged<DriverEventWire> {
        let mut message = DriverEventWire::new();
        message.set_kind(kind);
        message.set_time(when(n));
        at(n, message)
    }

    /// Whether any finding says `what`, and how many do.
    fn findings_about(report: &Report, what: &str) -> usize {
        report
            .findings
            .iter()
            .filter(|finding| finding.contains(what))
            .count()
    }

    /// Whether any measurement says `what`.
    fn measured_about(report: &Report, what: &str) -> bool {
        report.measured.iter().any(|line| line.contains(what))
    }

    /// A log with no samples in it is not a run this tool can say anything
    /// about, and it says exactly that rather than reporting a clean sweep of
    /// checks none of which had data.
    #[test]
    fn a_log_with_no_heartbeat_in_it_is_refused_at_once() {
        let report = analyze(&Run::default());
        assert_eq!(report.findings.len(), 1);
        assert!(
            report.findings[0].contains("no samples"),
            "{:?}",
            report.findings
        );
    }

    /// A run that stopped part way through the gesture fails on the phases it
    /// never narrated, which is the check every other one is read against.
    #[test]
    fn a_session_that_never_engaged_is_a_gesture_that_did_not_happen() {
        let run = Run {
            samples: heartbeat(50),
            reports: vec![
                phase(1, SessionPhaseWire::RESTING, SessionPhaseWire::STARTING),
                phase(2, SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
            ],
            ..Run::default()
        };
        let report = analyze(&run);
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("narrated 2 phase changes")),
            "{:?}",
            report.findings
        );
    }

    /// A run whose session took a different path through the phases fails on the
    /// change that differs, by name, rather than on a count.
    #[test]
    fn a_session_that_parked_instead_of_engaging_fails_on_that_change() {
        let run = Run {
            samples: heartbeat(50),
            reports: vec![
                phase(1, SessionPhaseWire::RESTING, SessionPhaseWire::STARTING),
                phase(2, SessionPhaseWire::PARKED, SessionPhaseWire::RESTING),
            ],
            ..Run::default()
        };
        let report = analyze(&run);
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("phase change 2 is")),
            "{:?}",
            report.findings
        );
    }

    /// The two driver events the design names are refusals, and the one a
    /// restarted driver is supposed to write is not: a driver that reached the
    /// minimum risk condition before anybody talked to it did its job.
    #[test]
    fn a_bus_that_failed_is_a_finding_and_a_startup_release_is_not() {
        let refused = Run {
            samples: heartbeat(10),
            events: vec![event(4, EventKindWire::BUS_FAILURE)],
            ..Run::default()
        };
        assert!(
            analyze(&refused)
                .findings
                .iter()
                .any(|finding| finding.contains("declared its bus gone")),
            "the bus failure went unreported"
        );

        let started = Run {
            samples: heartbeat(10),
            events: vec![event(1, EventKindWire::STARTUP_MRC_WRITE)],
            ..Run::default()
        };
        let report = analyze(&started);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("minimum risk condition")),
            "{:?}",
            report.measured
        );
        assert!(
            !report
                .findings
                .iter()
                .any(|finding| finding.contains("minimum risk")),
            "{:?}",
            report.findings
        );
    }

    /// The release evidence is the one check standing between a green report and
    /// a machine nobody let go of, so an ending report naming a servo set this
    /// build cannot read must not satisfy it.
    #[test]
    fn an_unreadable_set_at_the_release_is_not_evidence_of_one() {
        let mut ended = TimelineEntryWire::new();
        ended.set_time(SyncTime::from_nanos(T0 + 5 * NOMINAL_CYCLE_NS));
        ended.set_kind(ReportKindWire::SESSION_ENDED);
        // Past the ninth bus row: a set this build has no servos for.
        ended.set_b(0x400);
        let run = Run {
            samples: heartbeat(10),
            reports: vec![at(5, ended)],
            ..Run::default()
        };
        let report = analyze(&run);
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("which this build cannot read")),
            "{:?}",
            report.findings
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("no session ended with every row measured")),
            "{:?}",
            report.findings
        );
    }

    /// A hole in the heartbeat is a control cycle that never happened, and it is
    /// found without knowing what instant the run started at.
    #[test]
    fn a_gap_in_the_heartbeat_is_found_wherever_the_run_began() {
        let mut samples = heartbeat(10);
        samples.extend(heartbeat(30).into_iter().skip(20));
        let run = Run {
            samples,
            ..Run::default()
        };
        assert!(
            analyze(&run)
                .findings
                .iter()
                .any(|finding| finding.contains("heartbeat skips from cycle 9 to cycle 20")),
            "the gap went unreported"
        );
    }

    /// A jitter band moves what counts as on the grid, and nothing else.
    #[test]
    fn a_sample_inside_the_jitter_band_sits_on_its_cycle() {
        // Built twice rather than cloned: a logged wire message is not `Clone`.
        let jittered = || {
            let mut samples = heartbeat(10);
            let mut late = sample(4);
            late.set_nominal_time(SyncTime::from_nanos(T0 + 4 * NOMINAL_CYCLE_NS + 400_000));
            samples[4] = at(4, late);
            let mut early = sample(6);
            early.set_nominal_time(SyncTime::from_nanos(T0 + 6 * NOMINAL_CYCLE_NS - 400_000));
            samples[6] = at(6, early);
            samples
        };
        let report = analyze(&Run {
            samples: jittered(),
            grid_jitter_ns: 1_000_000,
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "off the"),
            0,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "skips from") + findings_about(&report, "already read"),
            0,
            "a sample inside the band was read as a gap or a repeat: {:?}",
            report.findings
        );

        // The same two samples with no band: both are off the grid, and the
        // default is no band.
        let report = analyze(&Run {
            samples: jittered(),
            ..Run::default()
        });
        assert!(
            findings_about(&report, "off the") > 0,
            "the band is not the default: {:?}",
            report.findings
        );
    }

    /// A sample outside the band is off the grid, band or no band.
    #[test]
    fn a_sample_past_the_jitter_band_is_still_off_the_grid() {
        let mut samples = heartbeat(10);
        let mut adrift = sample(5);
        adrift.set_nominal_time(SyncTime::from_nanos(T0 + 5 * NOMINAL_CYCLE_NS + 4_000_000));
        samples[5] = at(5, adrift);
        let report = analyze(&Run {
            samples,
            grid_jitter_ns: 1_000_000,
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "off the"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// A sample that does not sit on the grid does not invent a gap at the next
    /// one that does.
    ///
    /// The stream's position advances over it: an off-grid sample is one failure,
    /// and leaving the position where it was turns it into a second finding
    /// naming a skip that never happened.
    #[test]
    fn a_sample_off_the_grid_is_one_finding_and_not_a_gap_as_well() {
        let mut samples = heartbeat(10);
        let mut adrift = sample(5);
        adrift.set_nominal_time(SyncTime::from_nanos(T0 + 5 * NOMINAL_CYCLE_NS + 1_000_000));
        samples[5] = at(5, adrift);
        let report = analyze(&Run {
            samples,
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "off the"),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "skips from"),
            0,
            "the sample after the off-grid one read as a gap: {:?}",
            report.findings
        );
    }

    /// A cycle the stream has already read is said to be that, and not a skip of
    /// a negative number of cycles.
    #[test]
    fn a_repeated_cycle_is_reported_as_one_rather_than_as_a_backwards_skip() {
        let mut samples = heartbeat(6);
        samples.push(at(5, sample(5)));
        samples.extend(heartbeat(10).into_iter().skip(6));
        let report = analyze(&Run {
            samples,
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "already read cycle 5"),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "skips from"),
            0,
            "the repeat cascaded into gap findings: {:?}",
            report.findings
        );
    }

    /// Every kind the driver's event stream can carry, classified: five failures
    /// and three censused, each said in its own words and counted once.
    ///
    /// The table is the point. The verdict this tool prints lives in these arms,
    /// and an arm that notes what it should fail -- or fires twice, or leaks into
    /// the measurements -- is a green report over a log that says the dead-man
    /// latched.
    #[test]
    fn every_driver_event_kind_is_classified() {
        let bare = analyze(&Run {
            samples: heartbeat(10),
            ..Run::default()
        })
        .findings
        .len();
        let refusals = [
            (EventKindWire::BUS_FAILURE, "declared its bus gone"),
            (EventKindWire::CYCLE_SKIPPED, "cycle slots"),
            (EventKindWire::HOLD_TIMEOUT_TORQUE_OFF, "dead-man latched"),
            (EventKindWire::GOAL_DROPPED_QUEUE_FULL, "a goal was dropped"),
            (
                EventKindWire::TORQUE_OFF_UNCONFIRMED,
                "could not confirm the de-torquing",
            ),
            // An event kind this build has no reading for: a log written by a
            // newer tree than the analyzer.
            (EventKindWire::from(99), "no reading for"),
        ];
        for (kind, said) in refusals {
            let report = analyze(&Run {
                samples: heartbeat(10),
                events: vec![event(4, kind)],
                ..Run::default()
            });
            assert_eq!(
                findings_about(&report, said),
                1,
                "{kind:?}: {:?}",
                report.findings
            );
            assert_eq!(
                report.findings.len(),
                bare + 1,
                "{kind:?} raised more than its own finding: {:?}",
                report.findings
            );
            assert!(
                measured_about(&report, &format!("{kind:?} x1")),
                "{kind:?} went uncensused"
            );
        }
        let censused = [
            (EventKindWire::STARTUP_MRC_WRITE, "minimum risk condition"),
            (
                EventKindWire::GOAL_STALE_OR_OUT_OF_ORDER,
                "past its instant",
            ),
            (
                EventKindWire::TORQUE_OFF_CONFIRMED,
                "read every row back de-torqued",
            ),
        ];
        for (kind, said) in censused {
            let report = analyze(&Run {
                samples: heartbeat(10),
                events: vec![event(4, kind)],
                ..Run::default()
            });
            assert_eq!(
                report.findings.len(),
                bare,
                "{kind:?} failed a run it should only have been counted in: {:?}",
                report.findings
            );
            assert!(measured_about(&report, said), "{kind:?} went unmentioned");
        }
    }

    /// Every kind the session's narration can carry, on the same terms.
    ///
    /// The phase changes have their own cases; this is every other kind, which
    /// is where a run that faulted, refused the script or could not confirm its
    /// release says so.
    #[test]
    fn every_other_report_kind_the_session_sends_is_classified() {
        let bare = analyze(&Run {
            samples: heartbeat(10),
            ..Run::default()
        })
        .findings
        .len();
        let refusals = [
            (
                said(4, ReportKindWire::SCRIPT_REFUSED, 1, 2),
                "refused script 1",
            ),
            (
                said(4, ReportKindWire::FAULT_RECORDED, 3, 12),
                "recorded fault 3",
            ),
            (
                said(4, ReportKindWire::RESPONSE_TAKEN, 2, 5),
                "took response 2",
            ),
            (
                said(4, ReportKindWire::TORQUE_OFF_UNCONFIRMED, 4, 0),
                "could not confirm the de-torquing",
            ),
            (
                said(4, ReportKindWire::BUS_FAILURE_DECLARED, 0, 0),
                "declared the bus failed",
            ),
            (
                said(4, ReportKindWire::from(200), 0, 0),
                "a clean wake gesture says none of it",
            ),
        ];
        for (entry, complaint) in refusals {
            let kind = entry.message.kind();
            let report = analyze(&Run {
                samples: heartbeat(10),
                reports: vec![entry],
                ..Run::default()
            });
            assert_eq!(
                findings_about(&report, complaint),
                1,
                "{kind:?}: {:?}",
                report.findings
            );
            assert_eq!(
                report.findings.len(),
                bare + 1,
                "{kind:?} raised more than its own finding: {:?}",
                report.findings
            );
        }
        for kind in [
            ReportKindWire::SCRIPT_ACCEPTED,
            ReportKindWire::SCHEDULE_PUBLISHED,
            ReportKindWire::TORQUE_OFF_CONFIRMED,
        ] {
            let report = analyze(&Run {
                samples: heartbeat(10),
                reports: vec![said(4, kind, 1, 0)],
                ..Run::default()
            });
            assert_eq!(
                report.findings.len(),
                bare,
                "{kind:?} failed a run it belongs in: {:?}",
                report.findings
            );
            assert!(
                measured_about(&report, &format!("{kind:?} x1")),
                "{kind:?} went uncensused"
            );
        }
        // The ending report with nothing unread is the one kind that answers a
        // finding rather than raising one: it is where the session says it read
        // the machine at rest.
        let report = analyze(&Run {
            samples: heartbeat(10),
            reports: vec![said(4, ReportKindWire::SESSION_ENDED, 1, 0)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "no session ended with every row measured"),
            0,
            "{:?}",
            report.findings
        );
        assert!(measured_about(&report, "the session ended at"));
    }

    /// A fault raised by the decision tick fails the run, whatever it was: a
    /// wake gesture is a machine with nothing wrong with it.
    #[test]
    fn a_fault_from_the_decision_tick_fails_the_run() {
        let mut fault = TickFaultWire::new();
        fault.set_kind(FaultKindWire::HEAD_OBSTRUCTED);
        fault.set_time(when(4));
        let report = analyze(&Run {
            samples: heartbeat(10),
            faults: vec![at(4, fault)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "the decision tick raised"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// A servo that ended the run with its error byte latched is a finding, and
    /// one that ended clean is a measurement.
    #[test]
    fn a_latched_error_byte_is_a_finding_and_a_clean_one_is_a_number() {
        let mut clean = HealthReportWire::new();
        clean.set_id(10);
        clean.set_volts(7.4);
        let mut latched = HealthReportWire::new();
        latched.set_id(11);
        latched.set_volts(7.4);
        latched.set_bits(0x20);
        let report = analyze(&Run {
            samples: heartbeat(10),
            readings: vec![at(1, clean), at(1, latched)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "servo 11 ended the run with error byte 0x20"),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(findings_about(&report, "servo 10 ended"), 0);
        assert!(measured_about(&report, "servo 10: 7.40 V"));
    }

    /// The read census counts a partial read, a blind one and a sample naming a
    /// set this build cannot read in three different buckets.
    ///
    /// The clean figure is what an operator files with the run, so it is the one
    /// number that must not absorb the samples the checks beside it refused.
    #[test]
    fn the_read_census_keeps_partial_blind_and_undecodable_apart() {
        let mut samples = heartbeat(10);
        let mut partial = sample(3);
        partial.set_missing(JointFlagsWire::from(flags::bit(JointRef::AntennaLeft)));
        samples[3] = at(3, partial);
        let mut nothing = sample(4);
        nothing.set_present_valid(false);
        nothing.set_missing(JointFlagsWire::from(flags::all()));
        samples[4] = at(4, nothing);
        let mut odd = sample(5);
        // Past the ninth bus row: a set of servos this build does not have.
        odd.set_missing(JointFlagsWire(0x400));
        samples[5] = at(5, odd);
        let report = analyze(&Run {
            samples,
            ..Run::default()
        });
        assert!(
            measured_about(
                &report,
                "reads: 7 clean, 1 partial, 1 answered nothing, 1 naming a set this build cannot \
                 read"
            ),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "names a set of servos this build cannot read"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// The session's own release, row by row, with every row answered.
    fn released(
        from: u32,
        statuses: [AuxStatusWire; 9],
    ) -> (Vec<Logged<SessionCmdWire>>, Vec<Logged<AuxOutcomeWire>>) {
        let mut asked = Vec::new();
        let mut answers = Vec::new();
        for (row, status) in statuses.into_iter().enumerate() {
            let corr = from + u32::try_from(row).expect("nine rows");
            let mut cmd = SessionCmdWire::new();
            cmd.set_kind(SessionCmdKindWire::AUX);
            cmd.set_corr(corr);
            let txn = cmd.txn_mut();
            txn.set_active(true);
            txn.set_op(AuxOpKindWire::WRITE_REG_VERIFIED);
            txn.set_id(10 + u8::try_from(row).expect("nine rows"));
            txn.set_reg(RegIdWire::TORQUE_ENABLE);
            txn.set_value(0);
            asked.push(at(6, cmd));
            // A transaction with no outcome is the timeout the join defaults to,
            // so a status of `TIMEOUT` is expressed by publishing nothing.
            if status != AuxStatusWire::TIMEOUT {
                let mut outcome = AuxOutcomeWire::new();
                outcome.set_corr(corr);
                outcome.set_status(status);
                answers.push(at(7, outcome));
            }
        }
        (asked, answers)
    }

    /// Nine rows written and nine read back is a machine let go of, and the
    /// analyzer says so in a number rather than a finding.
    #[test]
    fn nine_rows_read_back_de_torqued_is_the_release() {
        let (datagrams, outcomes) = released(1, [AuxStatusWire::OK; 9]);
        let report = analyze(&Run {
            samples: heartbeat(10),
            reports: vec![said(8, ReportKindWire::SESSION_ENDED, 1, 0)],
            datagrams,
            outcomes,
            ..Run::default()
        });
        assert!(
            measured_about(&report, "torque off on 9 rows, 9 of them read back"),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "was not let go of"),
            0,
            "{:?}",
            report.findings
        );
    }

    /// A row whose release nothing answered is a machine that was not let go
    /// of, and the count names how many were.
    #[test]
    fn a_release_one_row_never_answered_is_not_a_machine_let_go_of() {
        let mut statuses = [AuxStatusWire::OK; 9];
        statuses[4] = AuxStatusWire::TIMEOUT;
        let (datagrams, outcomes) = released(1, statuses);
        let report = analyze(&Run {
            samples: heartbeat(10),
            reports: vec![said(8, ReportKindWire::SESSION_ENDED, 1, 0)],
            datagrams,
            outcomes,
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "was not let go of: 8 of 9 rows read back"),
            1,
            "{:?}",
            report.findings
        );
        assert!(
            measured_about(
                &report,
                "servo 14's release answered AuxStatusWire::TIMEOUT"
            ),
            "{:?}",
            report.measured
        );
    }

    /// A row re-issued after a lost transaction counts once, and by the answer
    /// that came back.
    #[test]
    fn a_release_re_issued_after_a_silence_counts_by_the_answer_that_arrived() {
        let (mut datagrams, mut outcomes) = released(1, [AuxStatusWire::TIMEOUT; 9]);
        let (again, answered) = released(20, [AuxStatusWire::OK; 9]);
        datagrams.extend(again);
        outcomes.extend(answered);
        let report = analyze(&Run {
            samples: heartbeat(10),
            reports: vec![said(8, ReportKindWire::SESSION_ENDED, 1, 0)],
            datagrams,
            outcomes,
            ..Run::default()
        });
        assert!(
            measured_about(&report, "torque off on 9 rows, 9 of them read back"),
            "{:?}",
            report.measured
        );
        assert_eq!(findings_about(&report, "was not let go of"), 0);
    }

    /// A latch that went away again fails the run: nothing clears one.
    ///
    /// The driver's gate is the other legitimate way of letting go, and a latch
    /// standing is reported rather than failed -- but a sample after it saying
    /// the machine is writable again is a driver doing something this stack has
    /// no reading for.
    #[test]
    fn a_latch_that_disappeared_mid_run_fails_the_run() {
        let mut samples = heartbeat(10);
        for cycle in 5..7 {
            let mut latched = sample(cycle);
            latched.set_torque_off_latched(true);
            samples[usize::try_from(cycle).expect("ten cycles")] = at(cycle, latched);
        }
        let report = analyze(&Run {
            samples,
            reports: vec![said(8, ReportKindWire::SESSION_ENDED, 1, 0)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "latch was gone again"),
            1,
            "{:?}",
            report.findings
        );
        // And a latch standing is the release, so the row count is not also a
        // finding.
        let mut standing = heartbeat(10);
        for cycle in 5..10 {
            let mut latched = sample(cycle);
            latched.set_torque_off_latched(true);
            standing[usize::try_from(cycle).expect("ten cycles")] = at(cycle, latched);
        }
        let report = analyze(&Run {
            samples: standing,
            reports: vec![said(8, ReportKindWire::SESSION_ENDED, 1, 0)],
            ..Run::default()
        });
        assert_eq!(findings_about(&report, "latch was gone again"), 0);
        assert_eq!(findings_about(&report, "was not let go of"), 0);
        assert!(measured_about(&report, "the driver latched torque off at"));
    }

    /// The five phase changes of the gesture, so the checks that need an
    /// engagement have one: taken at cycle 3, released at cycle 8.
    fn narrated() -> Vec<Logged<TimelineEntryWire>> {
        vec![
            phase(1, SessionPhaseWire::RESTING, SessionPhaseWire::STARTING),
            phase(2, SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
            phase(3, SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING),
            phase(8, SessionPhaseWire::STOPPING, SessionPhaseWire::ACTIVE),
            phase(9, SessionPhaseWire::RESTING, SessionPhaseWire::STOPPING),
        ]
    }

    /// Where the estimator says the head was, at cycle `n`.
    fn estimate(n: i64, pose: &Isometry3<f64>) -> Logged<PoseEstimateWire> {
        let mut msg = PoseEstimateWire::new();
        {
            let solved = msg.clear_valid();
            solved.time_of_validity = when(n);
            record::write_pose(&mut solved.head_pos, &mut solved.head_quat, pose);
            solved.valid = true.into();
        }
        at(n, msg)
    }

    /// A sample whose antennas read where a case puts them, the rest of the
    /// machine at zero.
    fn antennas_at(n: i64, right: f64, left: f64) -> Logged<PoseSampleWire> {
        let mut msg = PoseSampleWire::new();
        {
            let read = msg.clear_valid();
            read.nominal_time = when(n);
            read.sample_time = when(n);
            read.present_valid = true.into();
            let mut rows = [0.0; ROW_COUNT];
            rows[row(JointRef::AntennaRight).expect("a bus row")] = right;
            rows[row(JointRef::AntennaLeft).expect("a bus row")] = left;
            write_rows(&mut read.present, &rows);
        }
        at(n, msg)
    }

    /// A pose that is where it should be and turned away from it has not
    /// arrived.
    ///
    /// Both tolerances on one sample, which is the claim: a machine 4 mm from
    /// upright but pitched a fifth of a radian off it is not a machine standing
    /// up.
    #[test]
    fn a_well_placed_and_badly_turned_pose_has_not_arrived() {
        let mut turned = neutral_targets().head_pose_body;
        turned.rotation *= UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.2);
        let report = analyze(&Run {
            samples: heartbeat(10),
            reports: narrated(),
            estimates: (4..8).map(|n| estimate(n, &turned)).collect(),
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "never came within"),
            2,
            "upright and, from its best approach, stow: {:?}",
            report.findings
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("of upright on one sample")),
            "{:?}",
            report.findings
        );
    }

    /// A machine that folded and then stood up did not perform the gesture.
    ///
    /// The stow arrival is searched from the upright one onwards, so the order
    /// is part of the claim rather than a pair of independent sightings.
    #[test]
    fn a_fold_before_the_stand_is_not_a_gesture_performed() {
        let mut estimates = vec![estimate(4, &stow_pose_targets().head_pose_body)];
        estimates.extend((5..8).map(|n| estimate(n, &neutral_targets().head_pose_body)));
        let report = analyze(&Run {
            samples: heartbeat(10),
            reports: narrated(),
            estimates,
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "of upright on one sample"),
            0,
            "the machine did stand up: {:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "of stowed on one sample"),
            1,
            "the stow before the stand read as the gesture: {:?}",
            report.findings
        );
    }

    /// An antenna is compared as a direction, so an arrival a whole turn from
    /// the posture's own number is an arrival.
    #[test]
    fn an_antenna_a_whole_turn_from_stow_has_still_arrived() {
        let folded = stow_pose_targets().antennas;
        let turn = 2.0 * core::f64::consts::PI;
        let mut samples = heartbeat(10);
        samples[8] = antennas_at(8, folded[0] + turn, folded[1] - turn);
        let report = analyze(&Run {
            samples,
            reports: narrated(),
            estimates: (4..8)
                .map(|n| estimate(n, &neutral_targets().head_pose_body))
                .collect(),
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "from where stow puts it"),
            0,
            "{:?}",
            report.findings
        );

        // And one that ended a third of a radian off it has not.
        let mut samples = heartbeat(10);
        samples[8] = antennas_at(8, folded[0] + 3.0 * ARRIVAL_ANTENNA_RAD, folded[1]);
        let report = analyze(&Run {
            samples,
            reports: narrated(),
            estimates: (4..8)
                .map(|n| estimate(n, &neutral_targets().head_pose_body))
                .collect(),
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "from where stow puts it"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// A final reading this build cannot read fails the run rather than reading
    /// as antennas that ended correctly.
    ///
    /// Driven through `judge_antennas` because nothing in this tree makes
    /// `Joints` refuse bytes, and the arm exists for the schema that later
    /// constrains a field: the decision is testable even though the wire path
    /// into it is not.
    #[test]
    fn a_final_reading_this_build_cannot_read_fails_the_run() {
        let mut report = Report::default();
        judge_antennas(None, &mut report);
        assert_eq!(
            findings_about(&report, "cannot read"),
            1,
            "{:?}",
            report.findings
        );

        // And rows it can read are judged rather than refused: the same call
        // with the antennas at stow finds nothing.
        let folded = stow_pose_targets().antennas;
        let mut rows = [0.0; ROW_COUNT];
        for (joint, slot) in [(JointRef::AntennaRight, 0), (JointRef::AntennaLeft, 1)] {
            rows[row(joint).expect("an antenna has a row")] = folded[slot];
        }
        let mut arrived = Report::default();
        judge_antennas(Some(rows), &mut arrived);
        assert!(arrived.findings.is_empty(), "{:?}", arrived.findings);
    }

    /// The grid is derived, so its arithmetic is the one thing in here that has
    /// to hold for a run starting at any instant at all.
    #[test]
    fn the_grid_counts_from_the_first_sample_wherever_that_is() {
        let grid = Grid {
            origin_ns: 1_772_000_000_123_456_789,
            period_ns: NOMINAL_CYCLE_NS,
        };
        assert_eq!(grid.at(grid.origin_ns), (0, 0));
        assert_eq!(grid.at(grid.origin_ns + NOMINAL_CYCLE_NS), (1, 0));
        assert_eq!(grid.at(grid.origin_ns + 100 * NOMINAL_CYCLE_NS), (100, 0));
        // A sample a millisecond off the grid: the cycle it belongs to and the
        // offset that says it does not sit on the grid cleanly.
        assert_eq!(
            grid.at(grid.origin_ns + NOMINAL_CYCLE_NS + 1_000_000),
            (1, 1_000_000)
        );
        // Before the origin, which a log the reader ordered cannot hold but the
        // arithmetic has to answer for anyway: floored, not truncated toward
        // zero, so the offset is never negative.
        assert_eq!(grid.at(grid.origin_ns - 1), (-1, NOMINAL_CYCLE_NS - 1));
    }

    /// A report's two halves are separate, because the exit status is about one
    /// of them and a run that failed is exactly the run whose numbers matter.
    #[test]
    fn a_measurement_is_not_a_finding() {
        let mut report = Report::default();
        report.note("worst head lag 0.1 rad");
        report.fail("the head never came within a metre of upright");
        assert_eq!(report.measured.len(), 1);
        assert_eq!(report.findings.len(), 1);
    }

    /// The session's own reports carry a servo set as a plain number, which is
    /// the one place this tool reads a set of servos out of an open field. A set
    /// this build cannot read is refused rather than reduced to the empty one.
    #[test]
    fn a_report_naming_no_servo_set_this_build_knows_is_refused() {
        assert!(flags::is_empty(
            joint_set_of(0).expect("no servos is a set")
        ));
        assert!(flags::contains(
            joint_set_of(1).expect("the first row is a set"),
            JointRef::BodyYaw
        ));
        // Past the ninth bus row, and past a byte: neither is a set of servos,
        // and answering the empty set for either would make an undecodable
        // release read as a clean one.
        assert_eq!(joint_set_of(0x400), None);
        assert_eq!(joint_set_of(0xffff_ffff), None);
    }

    /// No channel is bound twice, and none is checked without being read: the
    /// table is the whole statement of what this tool reads, and a duplicate row
    /// would mean one of them never routed a message.
    #[test]
    fn every_channel_is_named_once() {
        for (index, bound) in CHANNELS.iter().enumerate() {
            for other in &CHANNELS[index + 1..] {
                assert_ne!(bound.name, other.name, "{} is bound twice", bound.name);
            }
        }
    }
}
