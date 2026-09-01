//! What one run of the motion system did, read off its log.
//!
//! The tool a hardware run is judged by. Given the directory a run's log
//! was written to, it answers
//! one question -- did the wake gesture happen, whole -- and prints the numbers
//! an operator wants beside the answer: how the driver's heartbeat held up, what
//! the read jitter was, how far the machine lagged its own goals, what the
//! health rotation saw, and how many messages every channel carried.
//!
//! It reads the log and nothing else. No process is started, no clock on this
//! machine is consulted and no file beside the log is opened, so a run that
//! happened on a unit last week is judged the same way as one that finished a
//! second ago.
//!
//! What makes that possible is that the log is self-contained by construction.
//! The logger attaches to each channel lazily and at the write head, so the
//! front of every stream is whatever it was late for; a run therefore loses
//! stream heads as a matter of course, and this tool measures that loss rather
//! than judging it. Every fact a verdict here turns on rides a carrier that
//! survives a late attach instead: the driver republishes its whole account of
//! the run on `DriverStatus`, the session republishes its whole story on
//! `ReportsOut`, and an out-of-band outcome names the transaction it answers.
//! A log missing one of those carriers is a run that cannot be verified, and
//! that is a finding of its own.
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

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use brenn_reachy__cogs__schedule_clk_rs::SessionScheduleWire;
use brenn_reachy__cogs__script_clk_rs::ScriptWire;
use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__cogs__session_cmd_clk_rs::{SessionCmdKindWire, SessionCmdWire};
use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
use brenn_reachy__driver__health_clk_rs::{
    AuxOutcomeWire, AuxStatusWire, DriverEventWire, DriverStatusWire, EventKindWire,
    HealthReportWire,
};
use brenn_reachy__driver__pose_clk_rs::{PoseEstimateWire, PoseSampleWire};
use brenn_reachy__hardware__dynamixel__registers_clk_rs::{RegIdWire, ValueShapeWire};
use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKindWire;
use brenn_reachy__motion__faults_clk_rs::TickFaultWire;
use brenn_reachy__motion__reports_clk_rs::{RefusalReason, RefusalReasonWire, ReportKindWire};
use brenn_reachy__motion__seq_clk_rs::SeqFailureKindWire;
use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, TimelineWire};
use dxl_proto::HardwareError;
use log_read::{Bound, Census, Complaints, Logged, Streams, binding, cumulative, read_with, typed};
use motion_channels::{
    AUX_OUT_CHANNEL, CMD_CHANNEL, ESTIMATE_CHANNEL, EVENT_CHANNEL, FAULT_CHANNEL, HEALTH_CHANNEL,
    POSE_CHANNEL, REPORT_CHANNEL, SCHEDULE_CHANNEL, SCRIPT_CHANNEL, SESSION_CMD_CHANNEL,
    STATUS_CHANNEL,
};
use motion_slots::joint_set;
use nalgebra::Isometry3;
use reachy_driver::NOMINAL_CYCLE_NS;
use reachy_motion::joints::{JointGroup, ROW_COUNT, ROWS, flags, group_of, row, rows_of};
use reachy_motion::postures::{neutral_targets, stow_pose_targets};
use reachy_motion::record;
use reachy_motion::seq::failure::Name as FailureName;
use reachy_motion::tick::{
    RECORDED_WORST_ANTENNA_LAG_RAD, RECORDED_WORST_HEAD_LAG_RAD, TrackingFaultConfig,
};
use reachy_motion::value;
use run_report::{Report, verdict};

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
    /// What the session said about all of it: the rows of the newest story it
    /// published, which is the whole of its narration.
    reports: Vec<Logged<TimelineEntryWire>>,
    /// How many rows the newest story says fell off the front of its ring.
    ///
    /// The one fact about that story which the rows it holds cannot be made to
    /// say: a reader handed sixty-four rows has no way to tell whether they are
    /// the whole narration or its tail.
    story_dropped: u32,
    /// What the session asked the driver for.
    datagrams: Vec<Logged<SessionCmdWire>>,
    /// Where the head was.
    estimates: Vec<Logged<PoseEstimateWire>>,
    /// How each out-of-band transaction turned out.
    outcomes: Vec<Logged<AuxOutcomeWire>>,
    /// What the health rotation read.
    readings: Vec<Logged<HealthReportWire>>,
    /// The driver's own account of its run, republished on a cadence and
    /// cumulative: each copy is the whole of it, so the last one is the run's
    /// last word and any one of them would have said the same about everything
    /// that had already happened.
    statuses: Vec<Logged<DriverStatusWire>>,
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
///
/// Bindings are strict: a log recorded under an older schema revision is
/// refused, not read approximately. Reading such a log means building this
/// tool at the revision the log was recorded under. See [`log_read::binding`].
const CHANNELS: [Bound<Run>; 12] = [
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
        check: binding::<TimelineWire>,
        route: |run, message| {
            cumulative(
                message,
                &mut run.reports,
                &mut run.complaints,
                |story: &TimelineWire, rows| rows.extend(story.entries().iter().cloned()),
            );
            // Taken off the same message a second time, because it is the one
            // thing the rows cannot say: the routing helper deals in rows, and
            // what a story lost off its front is a number beside them. A message
            // that did not decode has already been complained about.
            if let Ok(story) = message.to_message::<TimelineWire>() {
                run.story_dropped = story.dropped();
            }
        },
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
    Bound {
        name: STATUS_CHANNEL,
        check: binding::<DriverStatusWire>,
        route: |run, message| typed(message, &mut run.statuses, &mut run.complaints),
    },
];

/// What a log keeps of a channel, which is the logging policy the online
/// systems declare over it.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Retention {
    /// A ring the logger joins at the write head: what was published before it
    /// attached is not in the log.
    Regular,
    /// The newest message is retained and handed to a subscriber at attach, so
    /// a log holds it however late the logger started.
    Persistent,
}

/// How each channel above is retained, in the order they are bound.
///
/// One row per channel and no shorter list, so a channel added to the table
/// above has to say which it is rather than being taken for a regular one by
/// omission; a case pins the two tables against each other and against the
/// policies the online systems declare, which are the artifacts that decide it.
///
/// Two things turn on `Persistent`. A copy missing off the front costs the log
/// nothing, so the head-loss reading says nothing about those channels; and a
/// log holding *none* of one of the two carriers is a log missing something the
/// rest of this report reads, which is a finding.
const RETENTION: [(&str, Retention); CHANNELS.len()] = [
    (SCRIPT_CHANNEL, Retention::Regular),
    (SCHEDULE_CHANNEL, Retention::Persistent),
    (POSE_CHANNEL, Retention::Regular),
    (CMD_CHANNEL, Retention::Regular),
    (EVENT_CHANNEL, Retention::Regular),
    (FAULT_CHANNEL, Retention::Regular),
    (REPORT_CHANNEL, Retention::Persistent),
    (SESSION_CMD_CHANNEL, Retention::Regular),
    (ESTIMATE_CHANNEL, Retention::Regular),
    (AUX_OUT_CHANNEL, Retention::Regular),
    (HEALTH_CHANNEL, Retention::Regular),
    (STATUS_CHANNEL, Retention::Persistent),
];

/// Whether `name` is retained, and so whether a late attach can have cost the
/// log any of it.
///
/// A channel the table above does not name reads as regular: this is asked only
/// of the channels this tool binds, and the case beside `RETENTION` is what
/// makes that the whole set.
fn persistent(name: &str) -> bool {
    RETENTION
        .iter()
        .any(|(bound, kept)| *bound == name && *kept == Retention::Persistent)
}

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

    /// The driver's last word about its own run, or nothing where the log holds
    /// no status record at all.
    ///
    /// The last copy rather than the first: every copy is cumulative, so the
    /// newest is the one that has counted the most of the run. Which copy the
    /// log's own first one was does not enter into it.
    fn status(&self) -> Option<&DriverStatusWire> {
        self.statuses.last().map(|logged| &logged.message)
    }

    /// How many messages the log lost off the front of `name`.
    ///
    /// The publisher numbers from zero and skips nothing, so the lowest number
    /// the log holds is exactly the count that went past before the logger
    /// attached. A channel the log carries nothing of lost nothing that can be
    /// measured this way, and reads as zero: there is no number to subtract
    /// from, and the checks that care about an absent channel say so in their
    /// own terms.
    fn head_loss(&self, name: &str) -> usize {
        self.census
            .iter()
            .find(|channel| channel.name == name)
            .and_then(|channel| channel.first_seq)
            .map_or(0, |seq| usize::try_from(seq).unwrap_or(usize::MAX))
    }

    /// How many messages the log holds on `name`, whether or not this tool
    /// binds a type to it.
    fn held(&self, name: &str) -> usize {
        self.census
            .iter()
            .find(|channel| channel.name == name)
            .map_or(0, |channel| channel.count)
    }
}

/// The grid the run's samples sit on, derived from the samples themselves.
///
/// A hardware run starts at whatever top of a second the driver started at, so
/// nothing about the epoch can be assumed; what can be is the period, which is
/// the one number both hosts are built against. The origin is the first sample's
/// own nominal instant.
#[derive(Clone, Copy)]
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
fn heartbeat(run: &Run, grid: Grid, skips: &Skips<'_>, report: &mut Report) -> usize {
    let mut expected = 0_i64;
    let mut folded = 0_usize;
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
            // A gap the driver already reported as a skip is the same missing
            // slots said twice, and it is counted into that report's aggregate
            // instead. A gap nothing accounts for is a different bug -- cycles
            // the driver believes it ran -- and stays a finding of its own.
            if (expected..cycle).all(|missing| skips.missed.contains(&missing)) {
                folded += 1;
            } else {
                gaps += 1;
                if gaps <= 4 {
                    report.fail(format!(
                        "the driver's heartbeat skips from cycle {} to cycle {cycle}, and no \
                         skipped-slot report accounts for all of it: {} cycles the control cogs \
                         never ran on",
                        expected - 1,
                        cycle - expected
                    ));
                }
            }
        }
        expected = reached;
    }
    if gaps > 4 {
        report.fail(format!(
            "the driver's heartbeat has {gaps} unaccounted gaps in it, of which four are named \
             above"
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
    folded
}

/// The grid the run's samples sit on, or nothing where the run has none.
///
/// Derived once and handed to everything that reads a cycle number off an
/// instant, so the heartbeat, the skips and the timing all count from the same
/// origin.
fn grid_of(run: &Run) -> Option<Grid> {
    run.samples.first().map(|first| Grid {
        origin_ns: first.message.nominal_time().as_nanos(),
        period_ns: NOMINAL_CYCLE_NS,
    })
}

/// The grid points the driver reported missing, and the reports themselves.
///
/// A skip report is published by the first cycle attended after the run of
/// missed slots, and it says how many they were, so the slots it accounts for
/// are the ones immediately before it. Which cycles those are is what lets a
/// hole in the heartbeat be recognised as the same event rather than counted a
/// second time.
struct Skips<'a> {
    events: Vec<&'a Logged<DriverEventWire>>,
    /// Every cycle a report accounts for.
    missed: BTreeSet<i64>,
}

impl<'a> Skips<'a> {
    fn of(run: &'a Run, grid: Grid) -> Self {
        let events: Vec<&Logged<DriverEventWire>> = run
            .events
            .iter()
            .filter(|event| event.message.kind() == EventKindWire::CYCLE_SKIPPED)
            .collect();
        let mut missed = BTreeSet::new();
        for event in &events {
            let (cycle, off) = grid.within(event.message.time().as_nanos(), run.grid_jitter_ns);
            // A report that does not sit on the grid places no slots: it is
            // still counted as a report, and the gap it would have explained
            // stays unexplained rather than being explained by a guess.
            if off != 0 {
                continue;
            }
            for slot in cycle - i64::from(event.message.count())..cycle {
                missed.insert(slot);
            }
        }
        Self { events, missed }
    }

    /// The slots the reports account for, all told.
    fn slots(&self) -> u64 {
        self.events
            .iter()
            .map(|event| u64::from(event.message.count()))
            .sum()
    }
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

/// A refusal as this report says it.
///
/// An adapter rather than the variant's own identifier: the identifier is a
/// Rust name that a rename would rewrite this report with, and what a reader
/// holding the robot wants is the sentence. The `match` is wildcard-free, so a
/// reason the vocabulary grows is a compile error here rather than a line that
/// says nothing.
struct ReasonName(RefusalReason);

impl std::fmt::Display for ReasonName {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(match self.0 {
            RefusalReason::None => "no refusal at all",
            RefusalReason::Parked => "the machine is parked",
            RefusalReason::NotResting => "the machine was busy with a session",
            RefusalReason::TooManySteps => "more steps than a schedule holds",
            RefusalReason::TooManyOverlays => "more overlay windows than a schedule holds",
            RefusalReason::BadTimes => "times that are not a timeline",
            RefusalReason::UnknownMotion => "a motion index no library could hold",
            RefusalReason::Undecodable => "a datagram that is not a script",
            RefusalReason::Stale => "a number no higher than the running engagement's",
            RefusalReason::TooLong => "a schedule reaching past the session's horizon",
        })
    }
}

/// A refusal reason by name, or by number where this build has no name for it.
///
/// A reason this vocabulary does not declare is a log written by a build that
/// refuses scripts for reasons this one has never heard of, and saying so beats
/// printing a bare number that looks like one of the reasons here.
fn refusal_name(reason: u32) -> String {
    let wire = RefusalReasonWire(u8::try_from(reason).unwrap_or(u8::MAX));
    match wire.to_known() {
        Some(known) => format!("{}", ReasonName(known)),
        None => format!("reason {reason}, which this build does not name"),
    }
}

/// A commissioning failure by name, or by number where this build has no name
/// for it.
///
/// A number the vocabulary does not declare is a log written by a build with
/// more failures in it than this one knows, which is worth saying as itself
/// rather than as a guess.
fn failure_name(kind: u32) -> String {
    let wire = SeqFailureKindWire(u8::try_from(kind).unwrap_or(u8::MAX));
    match wire.to_known() {
        Some(known) => format!("{}", FailureName(known)),
        None => format!("failure kind {kind}, which this build does not name"),
    }
}

/// A `STARTING -> PARKED` narration that does not say why.
///
/// The drift guard on the row above: the only way a session parks out of
/// starting is a survey that refused the machine, and the verdict row is what
/// makes that recoverable from the log afterwards. A park with no verdict beside
/// it is a finding rather than a silence.
fn the_park_says_why(run: &Run, report: &mut Report) {
    let parked: Vec<i64> = run
        .reports
        .iter()
        .filter(|entry| entry.message.kind() == ReportKindWire::PHASE_CHANGED)
        .filter(|entry| {
            let to = SessionPhaseWire(u8::try_from(entry.message.a()).unwrap_or(u8::MAX));
            let from = SessionPhaseWire(u8::try_from(entry.message.b()).unwrap_or(u8::MAX));
            to == SessionPhaseWire::PARKED && from == SessionPhaseWire::STARTING
        })
        .map(|entry| entry.message.time().as_nanos())
        .collect();
    if parked.is_empty() {
        return;
    }
    let verdicts = run
        .reports
        .iter()
        .filter(|entry| entry.message.kind() == ReportKindWire::COMMISSION_FAILED)
        .count();
    if verdicts < parked.len() {
        report.fail(format!(
            "the session parked out of starting {} time(s) -- at {parked:?} -- and narrated \
             {verdicts} commission verdict(s): a park with no verdict beside it leaves why it \
             parked in a state slot no log carries",
            parked.len()
        ));
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
                "the session refused script {a} at {at}, {}: nothing moved, and nothing retries",
                refusal_name(b)
            )),
            // A gesture publishes one script and never a second, so a schedule
            // swapped in under the running one is a different run from this
            // one -- and the row is read out rather than left to the catch-all,
            // because what it says about the run is which script took over.
            ReportKindWire::SCRIPT_REPLACED => report.fail(format!(
                "the session replaced its schedule with script {a} at {at}, epoch {b}: the wake \
                 gesture asks once and holds what it asked for"
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
            // The row a parked run exists to explain, and the only one here that
            // says which servo the survey stopped on. Printed with the failure
            // named rather than numbered: the number is what the log carries and
            // the name is what sends somebody to the right half of the machine.
            ReportKindWire::COMMISSION_FAILED => report.fail(format!(
                "the survey refused the machine at {at}: {} at servo {b}, {detail}",
                failure_name(a)
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
/// Four kinds fail the run here, and each for its own reason: a bus that
/// answered nothing, a dead-man that latched, a goal dropped for want of queue,
/// and a de-torquing no row confirmed. A missed cycle slot fails it too, but as
/// one aggregate rather than one finding apiece, so it is judged by
/// [`cycle_skips`] instead. The rest are
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
            // Counted here and judged in one place by `cycle_skips`: a run
            // produces them by the hundred, and a hundred findings saying the
            // same thing is a report nobody reads to the end.
            EventKindWire::CYCLE_SKIPPED => {}
            // The one kind that is a measurement rather than an edge: read as a
            // distribution by `cycle_timing`, and a note apiece here would be
            // one a second of a run nobody would read.
            EventKindWire::CYCLE_STATS => {}
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
            // The instant itself is read off the status, which every log holds
            // whenever its logger attached; this stream carries the event only
            // where the attach was early enough to catch cycle 0, which is rare
            // and is not something to judge either way.
            EventKindWire::STARTUP_MRC_WRITE => report.note(format!(
                "the driver's event stream reaches back to the start-up release at {at}, which is \
                 the process's first act on the bus"
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

/// A measured span, in milliseconds, or what an unmeasured one says instead.
///
/// Zero is the field's own "nothing measured this" -- every field a kind does
/// not name carries it -- so a span of exactly zero is printed as unmeasured
/// rather than as an instant piece of work. A negative span is printed as it
/// was measured: the clock moved under the measurement, and rounding it away
/// would hide that.
fn span_of(ns: i64) -> String {
    if ns == 0 {
        return "an unmeasured span".to_string();
    }
    format!("{:.3} ms", ns as f64 / 1e6)
}

/// What the cycles cost.
///
/// The driver publishes a window of measurements once a second; this reads them
/// as one distribution rather than as events. What the distribution is *for* --
/// whether the cycles that ran past their slot are the ones carrying out-of-band
/// work -- is asked by [`cycle_skips`], beside the skips themselves.
/// The worst of a run of measured spans: the longest of them, or the most
/// negative where any came out negative.
///
/// A span the clock made negative is the reading that matters most -- the time
/// base the driver measures on moved backwards -- and a plain maximum would
/// drop it the moment one ordinary window stood beside it.
fn worst_span(spans: impl Iterator<Item = i64>) -> i64 {
    spans
        .fold(None, |held: Option<i64>, span| {
            Some(match held {
                None => span,
                Some(held) if held < 0 || span < 0 => held.min(span),
                Some(held) => held.max(span),
            })
        })
        .unwrap_or_default()
}

/// Above this, a write call spent its time somewhere other than handing bytes
/// to the kernel.
///
/// A write that only queues its frame costs tens to a few hundred microseconds
/// on this unit; a write that waits for the bytes to leave the UART costs
/// milliseconds, because the kernel sleeps in tick-sized units to do it. One
/// millisecond is above every write span measured on a healthy run and an order
/// of magnitude below the cheapest sleeping one, so a reading past it is a
/// blocking call back in the write path -- which a report has to say out loud,
/// since the cycle it eats need not be large enough to miss a slot.
const WORST_WRITE_FLOOR_NS: i64 = 1_000_000;

fn cycle_timing(run: &Run, skips: &Skips<'_>, report: &mut Report) {
    let windows: Vec<&Logged<DriverEventWire>> = run
        .events
        .iter()
        .filter(|event| event.message.kind() == EventKindWire::CYCLE_STATS)
        .collect();
    if windows.is_empty() {
        if !skips.events.is_empty() {
            report.note(
                "the log carries no cycle timing: this driver measures no cycle, so nothing here \
                 can say why a slot was missed"
                    .to_string(),
            );
        }
        return;
    }

    let cycles: u64 = windows.iter().map(|w| u64::from(w.message.count())).sum();
    let aux_cycles: u64 = windows
        .iter()
        .map(|w| u64::from(w.message.out_of_band()))
        .sum();
    let worst_work = worst_span(windows.iter().map(|w| w.message.work().as_nanos()));
    let worst_aux = worst_span(windows.iter().map(|w| w.message.exchange().as_nanos()));
    // Beside the exchange rather than inside it: the worst single write any
    // cycle of the window made, on a cycle that need not be the one the worst
    // exchange came off. Read as a share of the exchange only where a window's
    // out-of-band work is what it holds -- a cheap worst write against an
    // expensive exchange points at the servo or the scheduler, an expensive one
    // at the port.
    let worst_drain = worst_span(windows.iter().map(|w| w.message.drain().as_nanos()));
    let over_period = windows
        .iter()
        .filter(|w| w.message.work().as_nanos() > NOMINAL_CYCLE_NS)
        .count();
    report.note(format!(
        "cycle timing: {} windows over {cycles} cycles, {aux_cycles} of them carrying an \
         out-of-band transaction; worst cycle {}, worst exchange {}, worst single write on any \
         cycle {}, and \
         {over_period} windows whose worst cycle ran past the {:.3} ms grid",
        windows.len(),
        span_of(worst_work),
        span_of(worst_aux),
        span_of(worst_drain),
        NOMINAL_CYCLE_NS as f64 / 1e6,
    ));
    if worst_drain > WORST_WRITE_FLOOR_NS {
        report.fail(format!(
            "the worst single write on any cycle cost {}, past the {:.3} ms a write that only \
             hands its bytes to the kernel can account for: something in the write path is \
             blocking on the bytes leaving, and the cycle it spends is the cycle the grid \
             wanted for the servos",
            span_of(worst_drain),
            WORST_WRITE_FLOOR_NS as f64 / 1e6,
        ));
    }
    if worst_work < 0 || worst_aux < 0 {
        report.fail(
            "a cycle span came out negative: the clock the driver measures on moved backwards \
             between the two reads that bracket a cycle"
                .to_string(),
        );
    }
}

/// Every missed cycle slot in the run, as one finding.
///
/// A run misses them by the hundred and each one is the same fact, so the
/// verdict is one line carrying the whole of it: how many slots, over how long,
/// at what rate, and the worst single report. A report naming more than one slot
/// at once is also named on its own, because a cycle that ran two slots long is
/// a different size of problem from one that ran a little over.
///
/// The correlation with the out-of-band work is stated with its blind spot
/// rather than as a verdict. Two of the three things that take the out-of-band
/// slot leave a record in the log -- a host transaction's outcome and a health
/// reading -- and the third, a torque-off confirmation read-back, leaves none,
/// so a skip attributed to no aux cycle may still follow one.
fn cycle_skips(run: &Run, grid: Grid, skips: &Skips<'_>, folded: usize, report: &mut Report) {
    if skips.events.is_empty() {
        return;
    }
    let span_ns = match (run.samples.first(), run.samples.last()) {
        (Some(first), Some(last)) => {
            last.message.nominal_time().as_nanos() - first.message.nominal_time().as_nanos()
        }
        _ => 0,
    };
    let seconds = span_ns as f64 / 1e9;
    let rate = if seconds > 0.0 {
        skips.slots() as f64 / seconds
    } else {
        0.0
    };
    // The worst report and its own span, taken from one event rather than as
    // two maxima: the biggest span in the run can belong to a one-slot report,
    // and a line that read it beside another report's slot count would print a
    // cause and an effect that never met.
    let worst = skips
        .events
        .iter()
        .max_by_key(|event| (event.message.count(), event.message.work().as_nanos()));
    let (worst_slots, worst_span) = worst.map_or((0, 0), |event| {
        (event.message.count(), event.message.work().as_nanos())
    });
    let longest_span = skips
        .events
        .iter()
        .map(|event| event.message.work().as_nanos())
        .max()
        .unwrap_or_default();
    let carried = aux_carrying_cycles(run, grid);
    let mut after_health = 0_usize;
    let mut after_host = 0_usize;
    for event in &skips.events {
        let (cycle, off) = grid.within(event.message.time().as_nanos(), run.grid_jitter_ns);
        if off != 0 {
            continue;
        }
        // The cycle that ran long is the one before the run of points it ran
        // through, and the reporting cycle is the first one attended after them.
        let overlong = cycle - i64::from(event.message.count()) - 1;
        // A cycle running both is counted as the health one: the triple is
        // three exchanges against a host request's one, so it is the larger
        // half of what that cycle spent.
        if carried.health.contains(&overlong) {
            after_health += 1;
        } else if carried.host.contains(&overlong) {
            after_host += 1;
        }
    }
    let after_aux = after_health + after_host;
    // One finding, and any missed slot makes the run red. There is no budget:
    // a cycle that fits its grid with an order of magnitude to spare has no
    // explained reason to miss a slot, so the rate is printed to describe a
    // skip, never to excuse one.
    report.fail(format!(
        "the driver missed {} cycle slots over {seconds:.3} s of run, in {} reports, {rate:.2} \
         slots per second; the worst report names {worst_slots} slots at once, and the cycle \
         before that report spent {}; the longest cycle before any of them spent {}. At least \
         {after_aux} of the {} follow a cycle the log shows carrying out-of-band work: \
         {after_health} after a health reading and {after_host} after a host transaction. A \
         torque-off confirmation read-back carries one and leaves no record, so that is a floor",
        skips.slots(),
        skips.events.len(),
        span_of(worst_span),
        span_of(longest_span),
        skips.events.len(),
    ));
    if folded > 0 {
        report.note(format!(
            "{folded} of the driver's heartbeat gaps are these same missed slots, and are counted \
             once"
        ));
    }
    for event in &skips.events {
        if event.message.count() > 1 {
            report.fail(format!(
                "the driver missed {} cycle slots at once at {}, running {:.3} ms late after a \
                 cycle that spent {}",
                event.message.count(),
                event.message.time().as_nanos(),
                event.message.silence().as_nanos() as f64 / 1e6,
                span_of(event.message.work().as_nanos())
            ));
        }
    }
}

/// The cycles the log shows running out-of-band work, kept apart by which kind.
///
/// Two records place one: a health reading, which carries the very
/// `sample_time` its cycle's pose sample carries and so names its cycle
/// exactly, and a transaction outcome, which carries no instant at all and is
/// placed on the cycle whose sample was logged nearest it. The second is an
/// attribution rather than a reading, which is why what is built here is only
/// ever used to say "at least this many".
///
/// The two are kept apart because they are different sizes of work -- the
/// health rotation reads three registers where a host request runs one
/// exchange -- and a skip count that mixed them would say nothing about which
/// to measure first.
struct AuxCycles {
    /// Cycles a health reading names.
    health: BTreeSet<i64>,
    /// Cycles a host transaction's outcome was logged nearest.
    host: BTreeSet<i64>,
}

fn aux_carrying_cycles(run: &Run, grid: Grid) -> AuxCycles {
    let mut by_sample_time: BTreeMap<i64, i64> = BTreeMap::new();
    let mut by_log_instant: BTreeMap<i64, i64> = BTreeMap::new();
    for sample in &run.samples {
        let (cycle, off) =
            grid.within(sample.message.nominal_time().as_nanos(), run.grid_jitter_ns);
        if off != 0 {
            continue;
        }
        by_sample_time.insert(sample.message.sample_time().as_nanos(), cycle);
        by_log_instant.insert(sample.at_ns, cycle);
    }
    let mut carried = AuxCycles {
        health: BTreeSet::new(),
        host: BTreeSet::new(),
    };
    for reading in &run.readings {
        if let Some(cycle) = by_sample_time.get(&reading.message.sample_time().as_nanos()) {
            carried.health.insert(*cycle);
        }
    }
    for outcome in &run.outcomes {
        if let Some(cycle) = nearest(&by_log_instant, outcome.at_ns) {
            carried.host.insert(cycle);
        }
    }
    carried
}

/// The value in `map` whose key is nearest `at_ns`.
fn nearest(map: &BTreeMap<i64, i64>, at_ns: i64) -> Option<i64> {
    let before = map.range(..=at_ns).next_back();
    let after = map.range(at_ns..).next();
    match (before, after) {
        (Some((below, cycle)), Some((above, later))) => Some(if at_ns - below <= above - at_ns {
            *cycle
        } else {
            *later
        }),
        (Some((_, cycle)), None) | (None, Some((_, cycle))) => Some(*cycle),
        (None, None) => None,
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
fn the_gesture(
    run: &Run,
    engaged: Option<(i64, i64)>,
    traffic: &AuxTraffic<'_>,
    report: &mut Report,
) {
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
    antennas_arrived(run, traffic, released, report);
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
/// Judged where the machine was let go of, which is the last complete reading
/// before the session's first verified torque-off write. The schedule running
/// out is a different instant and an earlier one: the script's last commanded
/// point, with the wind-down still to run, and an antenna is still on its way
/// there. Where the antennas were at that earlier instant is printed as a
/// number, because the distance between the two is the scripted step's own lag,
/// and it is not judged.
///
/// A run whose log holds no verified release is a run with no such instant in
/// it. The judgement falls back to the end of the schedule and the missing
/// release is itself a finding: a moving run with no evidence the machine was
/// let go of is worse news than any antenna number. A run with no complete
/// reading at the end of its schedule is judged at neither instant: the sample
/// there is what places a release on the log's clock, and without it a
/// mid-gesture de-torque cannot be told from the wind-down's write.
///
/// Finding the sample is this function's half; judging it is
/// [`judge_antennas`], which takes the rows as the reading they are or the
/// absence of one, so both of its answers can be exercised without a schema
/// that refuses bytes.
fn antennas_arrived(run: &Run, traffic: &AuxTraffic<'_>, schedule_end: i64, report: &mut Report) {
    let at_end = run.samples.iter().rev().find(|sample| {
        sample.message.nominal_time().as_nanos() <= schedule_end && sample.message.present_valid()
    });
    if let Some(rows) = at_end.and_then(|sample| sample.message.present().validate().ok()) {
        for (joint, error) in antenna_errors(&rows_of(rows)) {
            report.note(format!(
                "{joint:?} at the end of the schedule: {error:.4} rad from stow, with the \
                 wind-down still to run"
            ));
        }
    }
    // The release is placed by the log's own instants and the schedule's end by
    // the driver's grid, so the two searches are against different clocks and
    // each sample is compared against the one its instant is on. The sample
    // that ends the schedule is the bridge between them, and without one there
    // is nothing to bridge with: a release could then only be placed against a
    // clock nothing here has read, and the earliest torque-off write in the log
    // -- which may be a scoped degrade from the middle of the gesture -- would
    // be taken for the wind-down's.
    let Some(anchor) = at_end else {
        report.fail(
            "no complete reading at or before the end of the schedule: nothing places the \
             release on the log's own clock, so where the antennas ended up is not judged"
                .to_string(),
        );
        return;
    };
    // The release the wind-down wrote, not one an earlier scoped de-torque
    // wrote: the search starts at the log instant of the sample that ends the
    // schedule, which is the last instant the gesture was still running.
    let release_ns = traffic.first_release_ns(anchor.at_ns);
    let sample = match release_ns {
        // The anchor itself is a complete reading at or before any release the
        // search can answer with, so the fallback is unreachable rather than a
        // choice; it is written rather than unwrapped because nothing about
        // this reading is worth a panic.
        Some(at_ns) => run
            .samples
            .iter()
            .rev()
            .find(|sample| sample.at_ns <= at_ns && sample.message.present_valid())
            .unwrap_or(anchor),
        None => {
            report.fail(
                "the log holds no verified torque-off write: where the antennas were when the \
                 machine was let go of is judged at the end of the schedule instead, and that \
                 the release left no evidence is the worse finding"
                    .to_string(),
            );
            anchor
        }
    };
    judge_antennas(
        sample.message.present().validate().ok().map(rows_of),
        release_ns.unwrap_or(schedule_end),
        report,
    );
}

/// How far each antenna is from where stow puts it, as a direction.
///
/// One arithmetic behind both readers -- the judged one and the note at the end
/// of the schedule -- so the two numbers a report prints for an antenna are the
/// same measurement of two instants rather than two definitions.
fn antenna_errors(present: &[f64; ROW_COUNT]) -> Vec<(reachy_motion::joints::JointRef, f64)> {
    let wanted = stow_pose_targets();
    let mut errors = Vec::new();
    for (joint, slot) in [
        (reachy_motion::joints::JointRef::AntennaRight, 0),
        (reachy_motion::joints::JointRef::AntennaLeft, 1),
    ] {
        let Some(index) = row(joint) else { continue };
        errors.push((
            joint,
            reachy_kin::wrap_to_pi(present[index] - wanted.antennas[slot]).abs(),
        ));
    }
    errors
}

/// Where the antennas ended, from the last measured rows this build could read.
///
/// `None` is a reading that failed wire validation, and it fails the run rather
/// than being passed over: "nobody can tell where the antennas ended" and "they
/// ended correctly" must not read alike. Nothing in this tree produces it —
/// `Joints` is nine plain numbers and its generated validation accepts any
/// bytes — so it is a guard against a schema that later constrains a field.
fn judge_antennas(present: Option<[f64; ROW_COUNT]>, at_ns: i64, report: &mut Report) {
    let Some(present) = present else {
        report.fail(
            "the last complete reading before the release carries measured rows this build \
             cannot read: nothing says where the antennas ended up"
                .to_string(),
        );
        return;
    };
    for (joint, error) in antenna_errors(&present) {
        report.note(format!("{joint:?} ended {error:.4} rad from stow"));
        if error > ARRIVAL_ANTENNA_RAD {
            report.fail(format!(
                "{joint:?} ended {error:.4} rad from where stow puts it, past the \
                 {ARRIVAL_ANTENNA_RAD} rad this report allows, measured at {at_ns}"
            ));
        }
    }
}

/// What one out-of-band request asked for, as the identity that names it.
///
/// Everything the datagram carried about the transaction and nothing about the
/// datagram itself: two requests with equal identities and equal correlation
/// numbers are the same request sent twice, which is what delivery retry is
/// allowed to do.
#[derive(Clone, Copy, PartialEq)]
struct AuxIdentity {
    op: AuxOpKindWire,
    id: u8,
    reg: RegIdWire,
    value_kind: ValueShapeWire,
    value: u64,
}

impl AuxIdentity {
    /// The identity as a report reads it: which transaction, to which servo,
    /// about which register, carrying what.
    fn line(&self) -> String {
        format!(
            "{:?} servo {} {:?} {}",
            self.op,
            self.id,
            self.reg,
            value_line(self.value_kind, self.value)
        )
    }
}

/// What an outcome says it is about, off its own echo of the request.
///
/// Everything the identity has except the value: an outcome carries which
/// transaction, which servo and which register, which is what an operator needs
/// to act, and the value it carries is the *answer* rather than what was asked
/// for. Read only where the request itself is not in the log, so a report never
/// prefers the echo to the thing it echoes.
fn echoed(outcome: &AuxOutcomeWire) -> String {
    format!(
        "{:?} servo {} {:?}",
        outcome.op(),
        outcome.id(),
        outcome.reg()
    )
}

/// One register value as the shape it is tagged with reads it.
///
/// The eight bytes are bits and not a number of anything, so an angle printed as
/// the integer its pattern happens to spell says nothing at all: a mismatch on a
/// goal position is a pair of angles, and a pair of twenty-digit integers is a
/// pair nobody can subtract.
///
/// The reading itself is [`value::Shown`], which is total over the shapes: a
/// shape added to the vocabulary reads correctly here the day it is added,
/// where a ladder private to this tool would fall through to a bit pattern.
fn value_line(kind: ValueShapeWire, value: u64) -> String {
    match kind.to_known() {
        Some(shape) => format!("{}", value::Shown(value::carried(shape, value))),
        None => format!("value {value} of a shape this build does not name ({kind:?})"),
    }
}

/// One request the log kept, with the instant it was published.
struct AuxRequest {
    identity: AuxIdentity,
    at_ns: i64,
}

/// What a correlation number carries on the request side besides the first
/// request under it.
///
/// Delivery retry re-issues an *unanswered* datagram verbatim, so a verbatim
/// duplicate is the re-issue slot pacing needs and a differing one is a
/// transaction of its own.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Reissue {
    /// The same request more than once, byte for byte.
    Verbatim,
    /// More than one request, and they do not agree.
    Differing,
    /// Exactly one request.
    None,
    /// No request at all: an outcome naming a number the recorded trail holds
    /// nothing for. Its own state rather than a case of `None`, because a
    /// number the log carries no request under is the number the log says the
    /// least about, and nothing read off it can be attributed to anything.
    Absent,
}

/// Every out-of-band request the log kept and every outcome, joined on the
/// correlation number.
///
/// Every reader of the outcome stream needs identity with it: an outcome carries
/// a status and a correlation number, and which servo and which register that
/// status is *about* lives only on the request side.
///
/// Both sides keep multiplicity, because both sides legitimately have it.
/// Delivery retry re-issues an unanswered datagram verbatim, correlation number
/// included, so one number can name several requests; and a re-issue that
/// arrived while the original still sat in the driver's slot is turned away
/// against its own number, so one number can carry a busy answer and the
/// original's real answer both. A map that kept one of each would silently drop
/// exactly the pair that says what happened.
struct AuxTraffic<'a> {
    /// Every `aux` request the log kept, by correlation number, in log order.
    requests: BTreeMap<u32, Vec<AuxRequest>>,
    /// Every outcome the log kept, by correlation number, in log order.
    outcomes: BTreeMap<u32, Vec<&'a AuxOutcomeWire>>,
    /// Datagrams asking for nothing at all. The driver counts one of these as a
    /// refusal and publishes no outcome for it, which is the one shape the
    /// counter cross-check has to forgive.
    none_kind: usize,
}

impl<'a> AuxTraffic<'a> {
    /// Join one run's request and outcome streams.
    fn of(run: &'a Run) -> Self {
        let mut traffic = Self {
            requests: BTreeMap::new(),
            outcomes: BTreeMap::new(),
            none_kind: 0,
        };
        for datagram in &run.datagrams {
            match datagram.message.kind() {
                SessionCmdKindWire::AUX => {}
                SessionCmdKindWire::NONE => {
                    traffic.none_kind += 1;
                    continue;
                }
                _ => continue,
            }
            let txn = datagram.message.txn();
            traffic
                .requests
                .entry(datagram.message.corr())
                .or_default()
                .push(AuxRequest {
                    identity: AuxIdentity {
                        op: txn.op(),
                        id: txn.id(),
                        reg: txn.reg(),
                        value_kind: txn.value_kind(),
                        value: txn.value(),
                    },
                    at_ns: datagram.at_ns,
                });
        }
        for outcome in &run.outcomes {
            traffic
                .outcomes
                .entry(outcome.message.corr())
                .or_default()
                .push(&outcome.message);
        }
        traffic
    }

    /// The best answer a correlation number got: `ok` if any outcome said so,
    /// otherwise the first thing that came back, and nothing at all if the log
    /// holds no outcome for it.
    ///
    /// A request re-issued after a lost answer is one transaction, and the
    /// reading that counts is the one that came back.
    fn best_status(&self, corr: u32) -> Option<AuxStatusWire> {
        let outcomes = self.outcomes.get(&corr)?;
        if outcomes
            .iter()
            .any(|outcome| outcome.status() == AuxStatusWire::OK)
        {
            return Some(AuxStatusWire::OK);
        }
        outcomes.first().map(|outcome| outcome.status())
    }

    /// The identity a correlation number names, or nothing if no request under
    /// that number is in the log.
    fn identity(&self, corr: u32) -> Option<AuxIdentity> {
        self.requests
            .get(&corr)
            .and_then(|requests| requests.first())
            .map(|request| request.identity)
    }

    /// Every request the log kept under this correlation number, in log order.
    ///
    /// Empty where the log holds none, which is a state the join really has: an
    /// outcome can name a number whose request the recorded trail lost.
    fn requests(&self, corr: u32) -> &[AuxRequest] {
        self.requests.get(&corr).map_or(&[], Vec::as_slice)
    }

    /// What else the trail holds under this correlation number.
    ///
    /// The request half of the pacing shape on its own: whether a re-issue is
    /// there at all, which is what decides whether a refusal the pacing rule
    /// turned down can be attributed to anything. A number the log holds no
    /// request under at all is [`Reissue::Absent`] rather than a lone request:
    /// the join really has that state, and nothing about a refusal under it can
    /// be read off the trail.
    ///
    /// The one classifier of the request side. Every reading of "what does this
    /// number carry besides the first request" goes through it -- the
    /// delivery-retry note, the differing-payload finding, the pacing rule's
    /// request half and the refusal narration -- because two of them disagreeing
    /// would let one report say a number carries a re-issue and another say it
    /// carries none.
    fn reissue(&self, corr: u32) -> Reissue {
        let Some((first, rest)) = self.requests(corr).split_first() else {
            return Reissue::Absent;
        };
        if rest.is_empty() {
            Reissue::None
        } else if rest
            .iter()
            .all(|request| request.identity == first.identity)
        {
            Reissue::Verbatim
        } else {
            Reissue::Differing
        }
    }

    /// The log instant of the first row the session wrote torque off on at or
    /// after `not_before`, or nothing where the log holds no such write.
    ///
    /// The instant the machine started being let go of, which is what the
    /// antennas' arrival is judged at: the wind-down writes the rows one after
    /// another, and the first of them is the last moment anything was still
    /// under command.
    ///
    /// `not_before` is what keeps that reading true, because the wind-down is not
    /// the only thing that writes torque off. A group-scoped degrade de-torques
    /// one group mid-run and leaves the rest of the machine under command, so a
    /// run that answered a fault that way has a release datagram on the wire
    /// while the gesture is still going. Judging the antennas at that instant
    /// would fail the run for being nowhere near stow in the middle of a
    /// movement, and say nothing about the degrade that actually happened.
    fn first_release_ns(&self, not_before: i64) -> Option<i64> {
        self.requests
            .values()
            .flatten()
            .filter(|request| is_a_release(&request.identity))
            .map(|request| request.at_ns)
            .filter(|at_ns| *at_ns >= not_before)
            .min()
    }
}

/// Whether a request is one row of the session's release: torque off, written
/// as its own read-back.
///
/// Named once, because two readers turn on it -- the release check itself and
/// the instant the antennas are judged at -- and a second spelling of it would
/// let them disagree about what a release is.
fn is_a_release(identity: &AuxIdentity) -> bool {
    identity.op == AuxOpKindWire::WRITE_REG_VERIFIED
        && identity.reg == RegIdWire::TORQUE_ENABLE
        && identity.value == 0
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
fn the_release(run: &Run, traffic: &AuxTraffic<'_>, report: &mut Report) {
    let mut released: BTreeMap<u8, AuxStatusWire> = BTreeMap::new();
    for (corr, requests) in &traffic.requests {
        for request in requests {
            let identity = request.identity;
            if !is_a_release(&identity) {
                continue;
            }
            let answered = traffic.best_status(*corr).unwrap_or(AuxStatusWire::TIMEOUT);
            // The best answer any row got: a release re-issued after a lost
            // transaction is one release, and the reading that counts is the
            // one that came back.
            released
                .entry(identity.id)
                .and_modify(|status| {
                    if answered == AuxStatusWire::OK {
                        *status = answered;
                    }
                })
                .or_insert(answered);
        }
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
///
/// One finding for the whole set rather than one per servo. A bus-wide condition
/// is one fact about the machine, and nine copies of it bury the rest of the
/// report. Which servos, and what each of them latched, is in the line.
///
/// The input-voltage bit on its own is the exception, and it is not a finding
/// on this machine: the servo bus rail is specified above the highest Max
/// Voltage Limit the register accepts, so a healthy unit sets that bit by
/// arithmetic. `dxl_proto::HardwareError` is the one predicate that says so and
/// this pass judges through it. The bit is never filtered away -- every servo's
/// byte is printed, and the set that latched it is named in a note of its own.
/// Any bit beyond input-voltage, on any servo, is a finding, and the byte is
/// named whole, so a voltage bit riding alongside an overload
/// launders nothing.
fn health(run: &Run, report: &mut Report) {
    let mut latest: BTreeMap<u8, &HealthReportWire> = BTreeMap::new();
    for reading in &run.readings {
        latest.insert(reading.message.id(), &reading.message);
    }
    if latest.is_empty() {
        report.note("the health rotation reported nothing".to_string());
        return;
    }
    let mut complaining: Vec<String> = Vec::new();
    let mut voltage_only: Vec<String> = Vec::new();
    for (id, reading) in latest {
        report.note(format!(
            "servo {id}: {:.2} V, {} C, error byte 0x{:02x}",
            reading.volts(),
            reading.temp_c(),
            reading.bits()
        ));
        let latched = HardwareError(reading.bits());
        if latched.bits_other_than_voltage() != 0 {
            complaining.push(format!("servo {id} (0x{:02x})", reading.bits()));
        } else if reading.bits() != 0 {
            voltage_only.push(format!("servo {id}"));
        }
    }
    if !voltage_only.is_empty() {
        report.note(format!(
            "the input-voltage bit is latched on {} of the servos the health rotation read: {} -- \
             expected on this machine, where the rail is specified above the register's Max \
             Voltage Limit, so a healthy unit sets that bit by arithmetic",
            voltage_only.len(),
            voltage_only.join(", ")
        ));
    }
    if !complaining.is_empty() {
        report.fail(format!(
            "the run ended with an error byte latched on {} of the servos the health rotation \
             read: {}",
            complaining.len(),
            complaining.join(", ")
        ));
    }
}

/// Why a busy answer is a finding, in the terms the trail can support.
///
/// `line` is the identity line the answer is printed under, and `reissue` what
/// else the correlation number carries on the request side.
///
/// A busy status is the driver saying its slot already held a request when this
/// one arrived, so what the trail is read for is which request that was. A
/// number the log holds once is the ordinary shape: two numbers overlapped,
/// which the session is serial and is not supposed to do. A number the log
/// holds twice byte for byte is a different claim entirely -- the driver's slot
/// answers a verbatim re-issue with the pending request's own outcome, so a
/// busy status against one is a driver that does not recognise duplicates, and
/// that is a finding against the build rather than against the run.
///
/// A number the log holds no request under at all supports neither reading: the
/// status still says the slot was full, and a trail missing the request cannot
/// say whether it was full of another number or of this one carrying other
/// bytes. The line says exactly that much, which is what a head-truncated log
/// leaves to say.
fn busy_line(line: &str, reissue: Reissue) -> String {
    match reissue {
        Reissue::Verbatim => format!(
            "{line}, and the log holds this request twice byte for byte: a verbatim re-issue is \
             answered by the pending request's own outcome, so a driver that answers it busy is \
             one built before its slot recognised a duplicate"
        ),
        Reissue::Differing => format!(
            "{line}, and the log holds another request under this number with a different \
             payload -- a finding of its own -- so the slot was holding a transaction this \
             number does not name"
        ),
        Reissue::None => format!(
            "{line}: the driver's slot was holding a request under another number when this one \
             arrived, and the session issues one at a time"
        ),
        Reissue::Absent => format!(
            "{line}, and the log holds no request under this number at all: the slot was full \
             when it arrived, and which request it was holding cannot be read off a trail that \
             is missing this one"
        ),
    }
}

/// How the out-of-band transactions went, each one named by what it asked for.
///
/// A status on its own says nothing an operator can act on: `REFUSED` is
/// several different driver-side decisions and `VERIFY_MISMATCH` is a servo
/// holding something other than what was written to it, and which servo and
/// which register is the whole of the question. So every outcome that is not
/// `ok` is printed with the identity of the request it answers.
///
/// Two of them are judged. A verified write that read back different is never
/// expected, so a mismatch fails the run. A refusal fails it too, unless the
/// join shows the shape slot pacing leaves -- a re-issue turned away while the
/// original was still pending, which is the transport doing its job.
///
/// The two pre-bus statuses are read directly rather than inferred. A refusal is
/// the driver declining the transaction itself, and a slot that was already full
/// says so under its own status, so neither reading depends on what else the
/// trail holds. What the trail is still read for is the busy answer's
/// companion: which request the slot was holding, per [`busy_line`].
///
/// An outcome names its own transaction, so a request that went past before the
/// logger attached costs the reading nothing: the identity comes off the
/// request where the log holds it and off the outcome's own echo where it does
/// not. A logged request nothing ever answered is still a finding -- the
/// sequencer's retry and timeout machinery guarantees every accepted
/// transaction an answer, so silence under a number the log *does* hold the
/// request for is either a lost outcome or a new bug.
fn transactions(run: &Run, traffic: &AuxTraffic<'_>, report: &mut Report) {
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

    for (corr, outcomes) in &traffic.outcomes {
        let identity = traffic.identity(*corr);
        for outcome in outcomes {
            let status = outcome.status();
            if status == AuxStatusWire::OK {
                continue;
            }
            let named = identity.map_or_else(|| echoed(outcome), |identity| identity.line());
            // Where the identity came off the echo, the line says so: the
            // reading is as good, and a reader comparing this report against
            // the request stream would otherwise look for a datagram the log
            // does not hold.
            let named = if identity.is_none() {
                format!(
                    "{named}, named by the outcome itself since the log holds no request under it"
                )
            } else {
                named
            };
            let line = format!("corr {corr}: {named} answered {status:?}");
            match status {
                AuxStatusWire::VERIFY_MISMATCH => report.fail(format!(
                    "{line}, reading back {}: a verified write is the read-back, so this servo \
                     holds something other than what the session wrote",
                    value_line(outcome.value_kind(), outcome.value())
                )),
                AuxStatusWire::BUSY => report.fail(busy_line(&line, traffic.reissue(*corr))),
                AuxStatusWire::REFUSED => report.fail(format!(
                    "{line}: the driver declined the transaction before anything reached the \
                     bus, and a slot that was already full answers busy instead, so this is a \
                     judgement on what was asked for"
                )),
                _ => report.note(line),
            }
        }
    }

    for (corr, requests) in &traffic.requests {
        let first = requests[0].identity;
        match traffic.reissue(*corr) {
            Reissue::Verbatim => report.note(format!(
                "corr {corr}: {} sent {} times, byte for byte, which is delivery retry",
                first.line(),
                requests.len()
            )),
            Reissue::Differing => report.fail(format!(
                "corr {corr} names {} different transactions: a re-issue is a verbatim repeat, so \
                 a number carrying two payloads is a retry with perturbed inputs",
                requests.len()
            )),
            // Absent cannot happen here: this loop walks the numbers the
            // request map holds, so each one carries at least one.
            Reissue::None | Reissue::Absent => {}
        }
        if !traffic.outcomes.contains_key(corr) {
            report.fail(format!(
                "corr {corr}: {} went out at {} and was never answered at all, and every accepted \
                 transaction gets an outcome",
                first.line(),
                requests[0].at_ns
            ));
        }
    }
}

/// The carriers this report verifies the run from are in the log.
///
/// Every other check here reads one of two cumulative records: the driver's
/// status, which carries what only the process itself observed, and the
/// session's story, which carries every row it narrated. Both are published
/// whole on every change and retained for a subscriber that attaches late, so
/// "the logger was not there yet" is not an innocent explanation for either
/// being absent -- a log holding none of one of them is a dead socket, a
/// removed publish, or a recording that stopped before the run began, and in
/// every case a run this tool cannot verify.
///
/// Said before anything is read off them, and unconditionally: no first-sequence
/// guard, because the guard would be the excuse this finding exists to refuse.
///
/// Two questions, asked the same way of both carriers and never conflated. The
/// census says whether the log holds the channel at all, which is a fact about
/// the recording; the decoded stream says whether this build could read what it
/// holds, which is a fact about the schemas. Answering the first out of the
/// second is how a log recorded under another schema revision gets diagnosed as
/// a dead socket -- a wrong first hypothesis handed to whoever is at the bench,
/// and one the complaint list beside it flatly contradicts.
fn carriers(run: &Run, report: &mut Report) {
    // A story is published only by an execution that added a row, so a message
    // on this channel always carries at least that row: no rows decoded out of
    // messages the log holds means none of them could be read.
    for (name, held, readable) in [
        (
            STATUS_CHANNEL,
            run.held(STATUS_CHANNEL),
            !run.statuses.is_empty(),
        ),
        (
            REPORT_CHANNEL,
            run.held(REPORT_CHANNEL),
            !run.reports.is_empty(),
        ),
    ] {
        if held == 0 {
            report.fail(format!(
                "the log holds no {name} message: it is republished whole and retained for a \
                 subscriber attaching at any instant, so a log without one was not recording that \
                 publisher, and what this report reads from it is unreadable"
            ));
        } else if !readable {
            report.fail(format!(
                "the log holds {held} {name} messages and this build could read none of them: the \
                 recording is there and the schemas differ, so read the complaints above rather \
                 than looking for a dead publisher"
            ));
        }
    }
}

/// The story the log carries is the whole of the session's narration.
///
/// The session narrates into a ring of sixty-four rows and republishes the whole
/// of it; past that it drops the oldest rows to make room and says how many in
/// the message. Nothing else can say it -- sixty-four rows read the same whether
/// they are the whole narration or its tail -- and the rows that go first are
/// the ones every check below walks in order: the first phase change, the script
/// the session accepted.
///
/// Said before those checks run, so their findings are read in its light: a
/// wrapped story makes them say the session never made the transitions, when
/// what happened is that it made more of them than the ring holds.
fn the_whole_story(run: &Run, report: &mut Report) {
    if run.story_dropped == 0 {
        return;
    }
    report.fail(format!(
        "the session's story dropped {} rows off its front: it narrated past the {} rows a story \
         carries, so the log holds the tail of the narration and the checks below are reading it \
         as the whole of it",
        run.story_dropped,
        run.reports.len(),
    ));
}

/// What the driver did to the machine before it did anything else.
///
/// The process writes the minimum risk condition as its first act on the bus,
/// so every run has this instant and a run that does not is a driver that did
/// not do it. The record is read off the status rather than off the event
/// stream: the cycle-0 event rides `DriverEvt`, which loses its head to the
/// logger's attach on essentially every healthy run, while every status copy
/// carries the same two facts.
///
/// A row whose verified write did not read back is judged the way an
/// unconfirmed de-torquing is judged anywhere else here -- as a finding naming
/// the rows -- because it is one: the machine was left with a row nobody could
/// see released.
fn the_startup_release(run: &Run, report: &mut Report) {
    let Some(status) = run.status() else {
        return;
    };
    let at = status.sweep_time().as_nanos();
    if at == 0 {
        report.fail(
            "the driver's status carries no instant for the minimum risk condition: the process \
             writes it before anything else it does on the bus, so a run with no instant for it \
             is a run that cannot say the machine was ever released"
                .to_string(),
        );
        return;
    }
    report.note(format!(
        "the driver wrote the minimum risk condition at {at}, before waiting for anything, which \
         is what every start does"
    ));
    match joint_set(status.sweep_failed_rows()) {
        Ok(failed) if flags::is_empty(failed) => {}
        Ok(failed) => report.fail(format!(
            "the driver could not verify the start-up release at {at}: {} did not read back, so \
             the process joined a machine it could not see let go of",
            flags::Names(failed)
        )),
        Err(err) => report.fail(format!(
            "the driver's status names the rows its start-up release could not verify as a set \
             this build has no servos for ({err}), so what it could not release cannot be read"
        )),
    }
}

/// The survey waited for the driver, rather than racing it.
///
/// The session's first datagram is the commissioning survey's first
/// transaction, and it is only worth sending to a driver that is cycling: a
/// driver still binding its inbox queues whatever arrives and answers the
/// delivery re-issue rather than the request, which the session reads as a
/// refusal and parks on. What says the driver is cycling is its sample stream,
/// so the order of those two firsts is the check.
///
/// Both instants are read off the driver's status, which is what makes the
/// check readable at all: they are stamped by one process on one clock, and
/// they are in every copy of a record that survives a late attach. Read off the
/// two streams instead, the answer would be a fact about when the logger
/// managed to attach to each of them.
///
/// A tie fails. The driver drains its inbox before it runs the cycle that
/// publishes a sample, so a command stamped at the same grid instant as the
/// first sample was taken *before* that sample existed, which is the race.
fn the_survey_waited(run: &Run, report: &mut Report) {
    let Some(status) = run.status() else {
        return;
    };
    let asked = status.first_session_cmd().as_nanos();
    let sampled = status.first_pose().as_nanos();
    // Zero is the schema's "has not happened": a run nobody commanded, or one
    // that published no sample. Neither is an ordering, and the sample-less
    // case is a run this report has already refused for having no heartbeat.
    if asked == 0 || sampled == 0 {
        return;
    }
    if asked > sampled {
        return;
    }
    report.fail(format!(
        "the driver took the session's first command at {asked} and published its first sample at \
         {sampled}: the survey asked before anything said the driver was cycling, and a driver \
         that has not started answers the delivery re-issue rather than the request"
    ));
}

/// Every channel the log carries, what it held, and where its copy starts.
///
/// The sequence number is the publisher's own count, so the lowest one a
/// channel carries says how much of that channel the log never got. Printed for
/// every channel, including the ones no Rust type is bound to: a report group
/// that starts late is the same fact about the recording as a payload stream
/// that does.
fn census(run: &Run, report: &mut Report) {
    for channel in &run.census {
        match channel.first_seq {
            Some(seq) => report.note(format!(
                "channel {}: {} messages, from sequence {seq}",
                channel.name, channel.count
            )),
            None => report.note(format!(
                "channel {}: {} messages",
                channel.name, channel.count
            )),
        }
    }
}

/// What the log lost at the front of each stream this tool reads.
///
/// A publisher numbers from zero and skips nothing, so a channel whose lowest
/// recorded sequence number is not zero was published on before the logger
/// attached, and exactly that many messages are gone. Said as a number, and
/// said per channel: the streams do not lose the same amount, and how much each
/// lost is what the counter cross-check charges the log before it charges the
/// run.
///
/// A note and never a finding. The logger attaches to each channel lazily and
/// at the write head, and nothing in the payload waits for it -- a logger is
/// never a precondition for driving motors -- so a run that publishes from
/// process start loses the front of every stream it feeds, and that is the
/// expected reading of a healthy run rather than a defect. What is lost is
/// redundancy: every fact a verdict here turns on rides a cumulative carrier or
/// a self-describing message instead.
///
/// The persistent channels are not read this way at all. One retained message
/// is their whole content, so where their copy starts says nothing about
/// completeness; what would be wrong is holding none of one, which is
/// [`carriers`]'s finding.
fn head_of_the_log(run: &Run, report: &mut Report) {
    for name in CHANNELS
        .iter()
        .map(|bound| bound.name)
        .filter(|name| !persistent(name))
    {
        let lost = run.head_loss(name);
        if lost == 0 {
            continue;
        }
        report.note(format!(
            "{name} begins at sequence {lost}: {lost} messages predate the logger's attach, so \
             the log holds that much less of the stream than the run published"
        ));
    }
}

/// What the log kept of the session's traffic, against what the driver counted
/// of it.
///
/// The drift guard, and it is a check on the *log* rather than on the run: the
/// driver counts every datagram it took off its socket and republishes the
/// count, so a stream the log is short of by more than its measured head loss
/// is a stream that lost messages mid-run -- a dropped datagram, a logger that
/// fell behind -- which is a real defect and not the attach.
///
/// So the head loss is charged first and the run is charged only with what is
/// left. Charging it the other way round would make the expected reading of
/// every healthy run a finding, and a finding every run carries is one nobody
/// reads.
///
/// Only a shortfall is ever a finding. The status is republished on a cadence,
/// so a log holding more than the last copy counted is ordinary rather than a
/// deficit, and a run that ended without a wind-down status has counters up to
/// one window older than the log.
///
/// The driver's refusal counter is checked against the *busy* outcomes, which
/// is what it produces: an offer turned away because the slot held another
/// request is answered `busy` and counted, and a transaction the driver takes
/// and then declines before the bus is answered `refused` and counted by
/// nothing. So a `refused` outcome has no counter to be checked against and
/// does not enter this arithmetic.
///
/// The two sides still do not correspond one for one. A datagram asking for
/// nothing is counted as a refusal and answered with nothing at all, so the
/// counter can legitimately run ahead of the logged busy outcomes by exactly
/// the number of those the log holds.
fn counter_cross_check(run: &Run, traffic: &AuxTraffic<'_>, report: &mut Report) {
    let Some(status) = run.status() else {
        return;
    };
    if !status.wound_down() {
        report.note(
            "the driver's last status is a periodic copy and not a wind-down's, so the run ended \
             without one and its counters can be up to one window older than the log"
                .to_string(),
        );
    }

    let logged = run.datagrams.len();
    let lost = run.head_loss(SESSION_CMD_CHANNEL);
    let counted = usize::try_from(status.seam().session_cmds()).unwrap_or(usize::MAX);
    report.note(format!(
        "the driver counted {counted} session datagrams; the log holds {logged} and lost {lost} \
         off the front"
    ));
    if counted > logged + lost {
        report.fail(format!(
            "the log holds {} fewer session datagrams than the driver counted, over and above \
             the {lost} its attach cost it ({logged} against {counted}): the trail lost messages \
             mid-run, so everything read off it is read off less than the run produced",
            counted - logged - lost
        ));
    }

    report.note(format!(
        "the driver recognised {} offers as a re-issue of the request it was already holding, and \
         answered each with that request's own outcome",
        status.cycle().aux_duplicates()
    ));

    let busy = run
        .outcomes
        .iter()
        .filter(|outcome| outcome.message.status() == AuxStatusWire::BUSY)
        .count();
    let counted = usize::try_from(status.cycle().aux_refused()).unwrap_or(usize::MAX);
    let lost = run.head_loss(AUX_OUT_CHANNEL);
    report.note(format!(
        "the driver counted {counted} turned-away offers; the log holds {busy} busy outcomes and \
         lost {lost} outcomes off the front"
    ));
    if counted > busy {
        let deficit = counted - busy;
        if deficit <= traffic.none_kind + lost {
            report.note(format!(
                "  {deficit} of them are accounted for by the do-nothing datagrams the driver \
                 counts and answers with no outcome, and by the outcomes the log lost at the front"
            ));
        } else {
            report.fail(format!(
                "the driver counted {} turned-away offers the log has neither a busy outcome, a \
                 do-nothing datagram, nor a lost outcome at the front to account for: either the \
                 trail lost messages mid-run, or a cycle turned away more than one request and \
                 only the first collision of a cycle leaves an outcome",
                deficit - traffic.none_kind - lost
            ));
        }
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
    let Some(grid) = grid_of(run) else {
        return report;
    };
    let skips = Skips::of(run, grid);
    carriers(run, &mut report);
    the_whole_story(run, &mut report);
    the_startup_release(run, &mut report);
    let folded = heartbeat(run, grid, &skips, &mut report);
    let engaged = narration(run, &mut report);
    what_the_session_said(run, &mut report);
    the_park_says_why(run, &mut report);
    the_survey_waited(run, &mut report);
    no_faults(run, &mut report);
    driver_events(run, &mut report);
    cycle_timing(run, &skips, &mut report);
    cycle_skips(run, grid, &skips, folded, &mut report);
    let traffic = AuxTraffic::of(run);
    the_gesture(run, engaged, &traffic, &mut report);
    the_release(run, &traffic, &mut report);
    jitter(run, &mut report);
    reads(run, &mut report);
    lags(run, &mut report);
    health(run, &mut report);
    transactions(run, &traffic, &mut report);
    head_of_the_log(run, &mut report);
    counter_cross_check(run, &traffic, &mut report);
    census(run, &mut report);
    report
}

/// Read the log named on the command line, judge it, and print both halves.
///
/// The measurements go to stdout and the findings to stderr, so a run's numbers
/// can be filed with the run record while the findings are what an operator sees
/// on the terminal. The exit status is the verdict.
fn main() -> ExitCode {
    const USAGE: &str = "usage: first_motion_report [--grid-jitter-ns <n>] <log-dir>";
    let mut args = std::env::args().skip(1);
    let mut jitter_ns = 0_i64;
    let mut log_dir: Option<String> = None;
    while let Some(arg) = args.next() {
        match arg.as_str() {
            "--grid-jitter-ns" => {
                let value = args.next().unwrap_or_default();
                match value.parse::<i64>() {
                    Ok(ns) if ns >= 0 => jitter_ns = ns,
                    _ => {
                        eprintln!(
                            "--grid-jitter-ns takes a whole number of nanoseconds, not {value}"
                        );
                        return ExitCode::FAILURE;
                    }
                }
            }
            _ if log_dir.is_none() && !arg.starts_with("--") => log_dir = Some(arg),
            _ => {
                eprintln!("{USAGE}");
                return ExitCode::FAILURE;
            }
        }
    }
    let Some(log_dir) = log_dir else {
        eprintln!("{USAGE}");
        return ExitCode::FAILURE;
    };
    let log_dir = &log_dir;
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
    verdict(
        "first_motion_report",
        log_dir,
        &report,
        "the wake gesture happened, whole",
    )
}

#[cfg(test)]
mod tests {
    use super::{
        ARRIVAL_ANTENNA_RAD, AUX_OUT_CHANNEL, AuxOpKindWire, AuxOutcomeWire, AuxStatusWire,
        CHANNELS, DriverEventWire, DriverStatusWire, EventKindWire, Grid, HealthReportWire, Logged,
        POSE_CHANNEL, PoseEstimateWire, PoseSampleWire, REPORT_CHANNEL, RETENTION, ReasonName,
        RegIdWire, Report, ReportKindWire, Retention, Run, SCHEDULE_CHANNEL, SESSION_CMD_CHANNEL,
        STATUS_CHANNEL, SeqFailureKindWire, SessionCmdKindWire, SessionCmdWire, SessionPhaseWire,
        TickFaultWire, TimelineEntryWire, ValueShapeWire, analyze, joint_set_of, judge_antennas,
        neutral_targets, record, row, stow_pose_targets,
    };
    use std::collections::BTreeSet;

    use brenn_reachy__motion__faults_clk_rs::FaultKindWire;
    use brenn_reachy__motion__joints_clk_rs::JointFlagsWire;
    use brenn_reachy__motion__reports_clk_rs::{RefusalReason, RefusalReasonWire};
    use clockwork_rs::{Duration, SyncTime};
    use log_read::Channel;
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
                .any(|line| line.contains("reaches back to the start-up release")),
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

    /// One window of the driver's cycle measurements.
    fn stats(n: i64, cycles: u32, aux: u32, work_ns: i64, aux_ns: i64) -> Logged<DriverEventWire> {
        drained_stats(n, cycles, aux, work_ns, aux_ns, 0)
    }

    /// A window that also says what its worst write call cost.
    fn drained_stats(
        n: i64,
        cycles: u32,
        aux: u32,
        work_ns: i64,
        aux_ns: i64,
        drain_ns: i64,
    ) -> Logged<DriverEventWire> {
        let mut logged = event(n, EventKindWire::CYCLE_STATS);
        logged.message.set_count(cycles);
        logged.message.set_out_of_band(aux);
        logged.message.set_work(Duration::from_nanos(work_ns));
        logged.message.set_exchange(Duration::from_nanos(aux_ns));
        logged.message.set_drain(Duration::from_nanos(drain_ns));
        logged
    }

    /// One skipped-slot report, naming what the cycle before it spent.
    fn skip(n: i64, slots: u32, work_ns: i64) -> Logged<DriverEventWire> {
        let mut logged = event(n, EventKindWire::CYCLE_SKIPPED);
        logged.message.set_count(slots);
        logged
            .message
            .set_silence(Duration::from_nanos(i64::from(slots) * NOMINAL_CYCLE_NS));
        logged.message.set_work(Duration::from_nanos(work_ns));
        logged
    }

    /// The lateness a skip reports is arithmetic off the grid; what the cycle
    /// before it spent is the only measured number in the report, and it is what
    /// says whether the slot was missed by a little or by a lot.
    #[test]
    fn a_skip_says_what_the_cycle_that_caused_it_spent() {
        let report = analyze(&Run {
            samples: heartbeat(10),
            events: vec![skip(5, 1, 25_400_000)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "the cycle before that report spent 25.400 ms"),
            1,
            "{:?}",
            report.findings
        );
        assert!(
            measured_about(&report, "the log carries no cycle timing"),
            "a run whose driver measured nothing has to say so: {:?}",
            report.measured
        );
    }

    /// A run that missed no slot says nothing about slots at all.
    ///
    /// The whole point of the drain measurement is a run whose cycles fit the
    /// grid; when one lands, the skip line has to be absent rather than printed
    /// as a zero, because a skip after that is a new problem and not a residual
    /// this report has learned to tolerate.
    #[test]
    fn a_run_that_missed_no_slot_prints_no_skip_finding() {
        let report = analyze(&Run {
            samples: heartbeat(20),
            events: vec![drained_stats(5, 50, 3, 18_000_000, 2_000_000, 40_000)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "missed"),
            0,
            "{:?}",
            report.findings
        );
        assert!(
            measured_about(&report, "worst single write on any cycle 0.040 ms"),
            "and the write span is there to read: {:?}",
            report.measured
        );
    }

    /// The worst report and the longest cycle need not be the same event, and
    /// the line says which number belongs to which.
    ///
    /// A two-slot report after a cycle that ran a little over, beside a one-slot
    /// report after a cycle that ran hugely over, is the mixed case: printing
    /// the run's longest span as the worst report's own would name a cause that
    /// belongs to another effect -- in the line a skip budget is to be sized
    /// from.
    #[test]
    fn the_worst_report_is_printed_with_its_own_span_and_not_the_runs_longest() {
        let report = analyze(&Run {
            samples: heartbeat(20),
            events: vec![skip(5, 1, 30_000_000), skip(9, 2, 21_000_000)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(
                &report,
                "the worst report names 2 slots at once, and the cycle before that report spent \
                 21.000 ms; the longest cycle before any of them spent 30.000 ms"
            ),
            1,
            "{:?}",
            report.findings
        );
    }

    /// A skip whose `work` field is zero is a cycle nobody measured, and a
    /// report that printed it as a duration would state a measurement the log
    /// does not hold.
    #[test]
    fn a_skip_from_a_driver_that_measured_nothing_says_the_span_is_unmeasured() {
        let report = analyze(&Run {
            samples: heartbeat(10),
            events: vec![skip(5, 1, 0)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "an unmeasured span"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// A hundred missed slots are one fact about the run, and the report says it
    /// once -- with the whole of it in the line, and the run still red.
    #[test]
    fn every_missed_slot_is_one_finding_and_the_run_is_still_red() {
        let report = analyze(&Run {
            samples: heartbeat(50),
            events: (10..30).map(|n| skip(n, 1, 22_000_000)).collect(),
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "the driver missed 20 cycle slots over"),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "cycle slots"),
            1,
            "the skips were reported more than once: {:?}",
            report.findings
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("slots per second")),
            "{:?}",
            report.findings
        );
    }

    /// A cycle that ran through two slots is a different size of problem from one
    /// that ran a little over, so it is named on its own as well as counted.
    #[test]
    fn a_skip_of_more_than_one_slot_is_named_on_its_own() {
        let report = analyze(&Run {
            samples: heartbeat(20),
            events: vec![skip(5, 1, 21_000_000), skip(9, 2, 44_000_000)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "the driver missed 3 cycle slots over"),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "missed 2 cycle slots at once"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// The hole a skip leaves in the heartbeat is the same missing slots the
    /// driver already reported, and counting it again reports one event twice.
    #[test]
    fn a_gap_a_skip_accounts_for_is_not_reported_a_second_time() {
        let mut samples = heartbeat(5);
        // Cycle 4 ran through cycles 5, 6 and 7; cycle 8 is the first attended
        // after them and the one that reports.
        samples.extend(heartbeat(12).into_iter().skip(8));
        let report = analyze(&Run {
            samples,
            events: vec![skip(8, 3, 62_000_000)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "heartbeat skips from"),
            0,
            "the gap was counted twice: {:?}",
            report.findings
        );
        assert!(
            measured_about(
                &report,
                "1 of the driver's heartbeat gaps are these same missed slots"
            ),
            "{:?}",
            report.measured
        );

        // A hole no report accounts for is a different bug -- cycles the driver
        // believes it ran -- and stays a finding of its own.
        let mut samples = heartbeat(5);
        samples.extend(heartbeat(12).into_iter().skip(8));
        let report = analyze(&Run {
            samples,
            events: vec![skip(8, 1, 21_000_000)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "heartbeat skips from cycle 4 to cycle 8"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// Thirty windows a run is one distribution, and thirty notes is a report
    /// nobody reads. The measurement is summarised, and it is never a finding.
    #[test]
    fn the_cycle_measurements_are_read_as_one_distribution() {
        let report = analyze(&Run {
            samples: heartbeat(10),
            events: vec![
                stats(1, 50, 3, 12_000_000, 4_000_000),
                stats(2, 50, 4, 24_000_000, 9_000_000),
            ],
            ..Run::default()
        });
        assert!(
            measured_about(
                &report,
                "cycle timing: 2 windows over 100 cycles, 7 of them carrying an out-of-band \
                 transaction; worst cycle 24.000 ms, worst exchange 9.000 ms, worst single \
                 write on any cycle an unmeasured span, and 1 windows"
            ),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "which this report has no reading for"),
            0,
            "{:?}",
            report.findings
        );
    }

    /// The write span is the guard against a blocking write coming back, so a
    /// millisecond in it is a finding and not a number to read.
    ///
    /// Both directions, because the guard is only worth having if the healthy
    /// reading is silent: a write in the microseconds a drainless one costs
    /// draws nothing, and the ~8 ms a drain costs draws a finding naming the
    /// write. Nothing else in the report catches this -- a drain small enough
    /// to fit the cycle misses no slot, and the skip finding stays silent.
    #[test]
    fn a_write_that_blocked_on_the_wire_is_a_finding_of_its_own() {
        let healthy = analyze(&Run {
            samples: heartbeat(10),
            events: vec![drained_stats(1, 50, 3, 12_000_000, 4_000_000, 40_000)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&healthy, "worst single write"),
            0,
            "a write that only queued its bytes says nothing: {:?}",
            healthy.findings
        );

        let blocked = analyze(&Run {
            samples: heartbeat(10),
            events: vec![drained_stats(1, 50, 3, 12_000_000, 4_000_000, 8_250_000)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&blocked, "worst single write on any cycle cost 8.250 ms"),
            1,
            "{:?}",
            blocked.findings
        );
        assert_eq!(
            findings_about(&blocked, "missed"),
            0,
            "and it is the write that is named, not a slot: {:?}",
            blocked.findings
        );
    }

    /// What the exchanges cost and what the worst single write cost are the two
    /// halves of the same reading -- the second over every cycle in the window
    /// rather than over the exchange alone -- and a window carrying only the
    /// first cannot say whether to look at the port or at the servo.
    #[test]
    fn a_window_says_what_the_worst_write_call_in_it_cost() {
        let report = analyze(&Run {
            samples: heartbeat(10),
            events: vec![
                drained_stats(1, 50, 3, 12_000_000, 4_000_000, 3_500_000),
                drained_stats(2, 50, 4, 24_000_000, 9_000_000, 8_250_000),
            ],
            ..Run::default()
        });
        assert!(
            measured_about(
                &report,
                "worst exchange 9.000 ms, worst single write on any cycle 8.250 ms, and 1 \
                 windows"
            ),
            "{:?}",
            report.measured
        );
    }

    /// The test of what the skips are: the cycle that ran through the missed
    /// slots is named by arithmetic off the grid, and the log says whether that
    /// cycle carried an out-of-band transaction.
    #[test]
    fn a_skip_after_an_out_of_band_cycle_is_counted_as_one() {
        let mut reading = HealthReportWire::new();
        reading.set_id(10);
        reading.set_volts(7.4);
        // The rotation stamps a reading with its own cycle's sample instant, so
        // it names cycle 3 exactly.
        reading.set_sample_time(when(3));
        let report = analyze(&Run {
            samples: heartbeat(10),
            readings: vec![at(3, reading)],
            // Cycle 3 ran through one slot, so cycle 5 is the first attended
            // after it and the one that reports.
            events: vec![
                skip(5, 1, 30_000_000),
                stats(9, 50, 1, 30_000_000, 8_000_000),
            ],
            ..Run::default()
        });
        assert!(
            report.findings.iter().any(|finding| finding.contains(
                "At least 1 of the 1 follow a cycle the log shows carrying out-of-band \
                 work: 1 after a health reading and 0 after a host transaction"
            )),
            "{:?}",
            report.findings
        );
    }

    /// A span measured across a clock that moved backwards comes out negative,
    /// and the driver publishes it rather than clamping it. The report says so:
    /// the negative number is the reading.
    #[test]
    fn a_negative_cycle_span_is_a_finding_about_the_time_base() {
        let report = analyze(&Run {
            samples: heartbeat(10),
            events: vec![stats(1, 50, 2, -5_000_000, 1_000_000)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "a cycle span came out negative"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// One negative window among ordinary ones is still the finding: a run of
    /// normal seconds beside the second the clock moved in must not read as a
    /// run whose worst cycle was normal.
    #[test]
    fn a_negative_window_beside_ordinary_ones_is_still_a_finding() {
        let report = analyze(&Run {
            samples: heartbeat(10),
            events: vec![
                stats(1, 50, 2, 18_000_000, 4_000_000),
                stats(2, 50, 2, -5_000_000, 1_000_000),
                stats(3, 50, 2, 19_000_000, 4_000_000),
            ],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "a cycle span came out negative"),
            1,
            "the negative window was averaged away by its neighbours: {:?}",
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
            (
                EventKindWire::STARTUP_MRC_WRITE,
                "reaches back to the start-up release",
            ),
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

    /// The verdict row a parked run exists to leave behind, printed with the
    /// failure named and the servo it names.
    #[test]
    fn a_commission_verdict_is_printed_with_the_failure_and_the_servo() {
        let mut verdict = said(
            4,
            ReportKindWire::COMMISSION_FAILED,
            u32::from(SeqFailureKindWire::ABSENT_SERVOS.0),
            13,
        );
        verdict.message.set_detail(1.0);

        let report = analyze(&Run {
            samples: heartbeat(10),
            reports: vec![
                verdict,
                phase(5, SessionPhaseWire::PARKED, SessionPhaseWire::STARTING),
            ],
            ..Run::default()
        });

        assert_eq!(
            findings_about(&report, "servos that did not answer a ping at servo 13"),
            1,
            "the survey's verdict is printed by name, with the servo: {:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "leaves why it parked in a state slot"),
            0,
            "a park with its verdict beside it raises no drift finding: {:?}",
            report.findings
        );
    }

    /// A number no vocabulary here declares is said as itself rather than
    /// guessed at.
    #[test]
    fn a_commission_verdict_this_build_cannot_name_is_printed_as_its_number() {
        let report = analyze(&Run {
            samples: heartbeat(10),
            reports: vec![said(4, ReportKindWire::COMMISSION_FAILED, 250, 0)],
            ..Run::default()
        });

        assert_eq!(
            findings_about(&report, "failure kind 250, which this build does not name"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// The drift guard: the only way out of starting into parked is a survey
    /// that refused the machine, and a log that does not say so is the gap the
    /// verdict row closes.
    #[test]
    fn a_park_out_of_starting_with_no_verdict_beside_it_is_a_finding() {
        let report = analyze(&Run {
            samples: heartbeat(10),
            reports: vec![phase(
                5,
                SessionPhaseWire::PARKED,
                SessionPhaseWire::STARTING,
            )],
            ..Run::default()
        });

        assert_eq!(
            findings_about(&report, "leaves why it parked in a state slot"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// Every reason the vocabulary declares says something of its own.
    ///
    /// Two reasons sharing a sentence, or one rendering as a token, would be
    /// invisible until the run that turned on which refusal it was. The guard
    /// comes with the adapter because the adapter's words are the whole content
    /// of the line a refusal prints.
    #[test]
    fn every_refusal_reason_is_distinct_and_says_something() {
        let mut seen = BTreeSet::new();
        for reason in RefusalReason::VARIANTS {
            let said = ReasonName(reason).to_string();
            assert!(said.len() > 3, "{reason:?} renders as {said:?}");
            assert!(seen.insert(said), "{reason:?} shares a sentence");
        }
        assert_eq!(seen.len(), RefusalReason::VARIANTS.len());
    }

    /// A refusal says its reason by name, and by number where this build has no
    /// name for it.
    ///
    /// The reason is the whole of what a refusal tells a reader, and a bare
    /// number is a lookup into a vocabulary that is not in front of them. A
    /// number this build's vocabulary does not declare is said as itself: it is
    /// a log written by a build that refuses scripts for reasons this one has
    /// never heard of, and guessing at it would be worse than saying so.
    #[test]
    fn a_refusal_names_the_reason_it_carries() {
        for (reason, said_as) in [
            (
                u32::from(RefusalReasonWire::STALE.0),
                "a number no higher than the running engagement's",
            ),
            (
                u32::from(RefusalReasonWire::TOO_LONG.0),
                "a schedule reaching past the session's horizon",
            ),
            (200, "reason 200, which this build does not name"),
        ] {
            let report = analyze(&Run {
                samples: heartbeat(10),
                reports: vec![said(4, ReportKindWire::SCRIPT_REFUSED, 1, reason)],
                ..Run::default()
            });
            assert_eq!(
                findings_about(&report, said_as),
                1,
                "reason {reason}: {:?}",
                report.findings
            );
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
                said(4, ReportKindWire::SCRIPT_REPLACED, 8, 3),
                "replaced its schedule with script 8",
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
            findings_about(
                &report,
                "latched on 1 of the servos the health rotation read: servo 11 (0x20)"
            ),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(findings_about(&report, "servo 10 ("), 0);
        assert!(measured_about(&report, "servo 10: 7.40 V"));
    }

    /// The input-voltage bit latched on every row is this machine's expected
    /// reading, not a finding: the rail sits above the highest limit the
    /// register accepts. One note names the set, and every byte still prints.
    #[test]
    fn the_input_voltage_bit_on_every_row_is_a_note_and_not_a_finding() {
        let readings: Vec<Logged<HealthReportWire>> = (10..19)
            .map(|id| {
                let mut reading = HealthReportWire::new();
                reading.set_id(id);
                reading.set_volts(7.4);
                reading.set_bits(0x01);
                at(1, reading)
            })
            .collect();
        let report = analyze(&Run {
            samples: heartbeat(10),
            readings,
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "error byte latched"),
            0,
            "{:?}",
            report.findings
        );
        assert!(measured_about(
            &report,
            "the input-voltage bit is latched on 9 of the servos the health rotation read"
        ));
        for id in 10..19 {
            assert_eq!(findings_about(&report, &format!("servo {id} (0x01)")), 0);
            assert!(measured_about(&report, &format!("servo {id}: 7.40 V")));
        }
    }

    /// A subset of the roster latching the input-voltage bit reads the same way:
    /// the bit is the machine's, not a row's, so the note names whoever has it.
    #[test]
    fn the_input_voltage_bit_on_a_subset_of_rows_is_still_a_note() {
        let mut clean = HealthReportWire::new();
        clean.set_id(10);
        clean.set_volts(7.4);
        let mut latched = HealthReportWire::new();
        latched.set_id(11);
        latched.set_volts(7.4);
        latched.set_bits(0x01);
        let report = analyze(&Run {
            samples: heartbeat(10),
            readings: vec![at(1, clean), at(1, latched)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "error byte latched"),
            0,
            "{:?}",
            report.findings
        );
        assert!(measured_about(
            &report,
            "the input-voltage bit is latched on 1 of the servos the health rotation read: servo 11"
        ));
    }

    /// The voltage bit launders nothing riding beside it: a byte carrying an
    /// overload as well is a finding, named whole.
    #[test]
    fn a_byte_carrying_more_than_the_voltage_bit_is_a_finding_named_whole() {
        let mut mixed = HealthReportWire::new();
        mixed.set_id(12);
        mixed.set_volts(7.4);
        mixed.set_bits(0x21);
        let report = analyze(&Run {
            samples: heartbeat(10),
            readings: vec![at(1, mixed)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(
                &report,
                "latched on 1 of the servos the health rotation read: servo 12 (0x21)"
            ),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "the input-voltage bit is latched"),
            0,
            "{:?}",
            report.findings
        );
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

    /// A release is the servo's own read-back or it is not evidence of
    /// anything. An unverified write answers with the driver's own send, so a
    /// run whose only torque-off writes were unverified reads as a run nobody
    /// can say was let go of -- which is the safe direction, and pinned here so
    /// that nobody widens the check to "either write" and has the report certify
    /// a machine as limp on the strength of a datagram the driver sent.
    #[test]
    fn a_release_written_unverified_is_no_release_at_all() {
        let (mut datagrams, outcomes) = released(1, [AuxStatusWire::OK; 9]);
        for logged in &mut datagrams {
            logged.message.txn_mut().set_op(AuxOpKindWire::WRITE_REG);
        }
        let report = analyze(&Run {
            samples: heartbeat(10),
            reports: vec![said(8, ReportKindWire::SESSION_ENDED, 1, 0)],
            datagrams,
            outcomes,
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "was not let go of: 0 of 9 rows read back"),
            1,
            "{:?}",
            report.findings
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

    /// The antennas are judged where the machine was let go of, which is after
    /// the wind-down that follows the schedule -- and where they were when the
    /// schedule ran out is printed rather than judged.
    #[test]
    fn the_antennas_are_judged_at_the_release_and_noted_at_the_end_of_the_schedule() {
        let folded = stow_pose_targets().antennas;
        let mut samples = heartbeat(10);
        // Still 0.3 rad out when the script's last point passes, and folded by
        // the time the wind-down writes the first row's torque off.
        samples[8] = antennas_at(8, folded[0] + 3.0 * ARRIVAL_ANTENNA_RAD, folded[1]);
        samples[9] = antennas_at(9, folded[0], folded[1]);
        let report = analyze(&Run {
            samples,
            reports: narrated(),
            estimates: (4..8)
                .map(|n| estimate(n, &neutral_targets().head_pose_body))
                .collect(),
            datagrams: vec![asked(9, 1, 10, RegIdWire::TORQUE_ENABLE, 0)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "from where stow puts it"),
            0,
            "the antennas were judged before the wind-down: {:?}",
            report.findings
        );
        assert!(
            measured_about(
                &report,
                "AntennaRight at the end of the schedule: 0.3000 rad from stow"
            ),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "no verified torque-off write"),
            0,
            "{:?}",
            report.findings
        );
    }

    /// The instant moved; the judgement did not. A machine let go of with its
    /// antennas nowhere near stow is the finding the 0.1 rad tolerance is kept
    /// for, and it is raised at the release rather than at the end of the
    /// schedule.
    #[test]
    fn antennas_still_out_at_the_release_are_a_finding_at_that_instant() {
        let folded = stow_pose_targets().antennas;
        let mut samples = heartbeat(10);
        // Folded when the script's last point passes and back out again by the
        // time the wind-down writes the first row's torque off, which is the
        // one instant that is judged.
        samples[8] = antennas_at(8, folded[0], folded[1]);
        samples[9] = antennas_at(9, folded[0] + 3.0 * ARRIVAL_ANTENNA_RAD, folded[1]);
        let release = T0 + 9 * NOMINAL_CYCLE_NS;
        let report = analyze(&Run {
            samples,
            reports: narrated(),
            estimates: (4..8)
                .map(|n| estimate(n, &neutral_targets().head_pose_body))
                .collect(),
            datagrams: vec![asked(9, 1, 10, RegIdWire::TORQUE_ENABLE, 0)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(
                &report,
                "AntennaRight ended 0.3000 rad from where stow puts it"
            ),
            1,
            "the release instant judged nothing: {:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, &format!("measured at {release}")),
            1,
            "the finding named an instant other than the release: {:?}",
            report.findings
        );
        assert!(
            measured_about(
                &report,
                "AntennaRight at the end of the schedule: 0.0000 rad from stow"
            ),
            "{:?}",
            report.measured
        );
    }

    /// A run with no complete reading at the end of its schedule has nothing to
    /// place a release against, so it is judged at neither instant and says so.
    ///
    /// The sample that ends the schedule is the bridge between the session's
    /// clock and the log's own. Without it the earliest torque-off write in the
    /// log -- which may be a scoped degrade from the middle of the gesture --
    /// would be taken for the wind-down's, and the antennas judged mid-movement.
    #[test]
    fn a_run_with_no_reading_at_the_end_of_the_schedule_is_judged_at_neither_instant() {
        let folded = stow_pose_targets().antennas;
        let mut samples = heartbeat(10);
        // Nowhere near stow at the mid-gesture de-torque below, and no reading
        // in the run answers for itself: every sample says its measured rows
        // are not a reading.
        samples[3] = antennas_at(3, folded[0] + 3.0 * ARRIVAL_ANTENNA_RAD, folded[1]);
        for sample in &mut samples {
            sample.message.set_present_valid(false);
        }
        let report = analyze(&Run {
            samples,
            reports: narrated(),
            estimates: (4..8)
                .map(|n| estimate(n, &neutral_targets().head_pose_body))
                .collect(),
            datagrams: vec![asked(3, 1, 10, RegIdWire::TORQUE_ENABLE, 0)],
            ..Run::default()
        });
        assert_eq!(
            findings_about(
                &report,
                "no complete reading at or before the end of the schedule"
            ),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "from where stow puts it"),
            0,
            "the antennas were judged at a de-torque from the middle of the gesture: {:?}",
            report.findings
        );
    }

    /// A torque-off write from before the schedule ran out is not the release.
    ///
    /// A group-scoped degrade de-torques one group mid-gesture and leaves the
    /// rest of the machine under command, so its write is on the wire while the
    /// antennas are still moving. Judging them there would fail the run for
    /// being mid-gesture in the middle of a gesture, and say nothing about the
    /// degrade.
    #[test]
    fn a_de_torque_from_before_the_schedule_ran_out_is_not_the_release() {
        let folded = stow_pose_targets().antennas;
        let mut samples = heartbeat(10);
        // Nowhere near stow while the gesture runs, folded at the end of it and
        // still folded when the wind-down writes the first row off.
        samples[5] = antennas_at(5, folded[0] + 3.0 * ARRIVAL_ANTENNA_RAD, folded[1]);
        samples[8] = antennas_at(8, folded[0], folded[1]);
        samples[9] = antennas_at(9, folded[0], folded[1]);
        let report = analyze(&Run {
            samples,
            reports: narrated(),
            estimates: (4..8)
                .map(|n| estimate(n, &neutral_targets().head_pose_body))
                .collect(),
            datagrams: vec![
                asked(5, 1, 10, RegIdWire::TORQUE_ENABLE, 0),
                asked(9, 2, 10, RegIdWire::TORQUE_ENABLE, 0),
            ],
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "from where stow puts it"),
            0,
            "the antennas were judged at the mid-run de-torque: {:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "no verified torque-off write"),
            0,
            "{:?}",
            report.findings
        );
    }

    /// A moving run whose log holds no release has no instant to judge at, and
    /// that it left no such evidence is the worse finding of the two.
    #[test]
    fn a_run_with_no_release_falls_back_to_the_end_of_the_schedule_and_says_so() {
        let folded = stow_pose_targets().antennas;
        let mut samples = heartbeat(10);
        samples[8] = antennas_at(8, folded[0] + 3.0 * ARRIVAL_ANTENNA_RAD, folded[1]);
        samples[9] = antennas_at(9, folded[0], folded[1]);
        let report = analyze(&Run {
            samples,
            reports: narrated(),
            estimates: (4..8)
                .map(|n| estimate(n, &neutral_targets().head_pose_body))
                .collect(),
            ..Run::default()
        });
        assert_eq!(
            findings_about(&report, "the log holds no verified torque-off write"),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(
                &report,
                "AntennaRight ended 0.3000 rad from where stow puts it"
            ),
            1,
            "the fallback judged an instant of its own choosing: {:?}",
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
        judge_antennas(None, 0, &mut report);
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
        judge_antennas(Some(rows), 0, &mut arrived);
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

    /// One out-of-band request, as the session publishes it.
    fn asked(n: i64, corr: u32, id: u8, reg: RegIdWire, value: u64) -> Logged<SessionCmdWire> {
        let mut cmd = SessionCmdWire::new();
        cmd.set_kind(SessionCmdKindWire::AUX);
        cmd.set_corr(corr);
        let txn = cmd.txn_mut();
        txn.set_active(true);
        txn.set_op(AuxOpKindWire::WRITE_REG_VERIFIED);
        txn.set_id(id);
        txn.set_reg(reg);
        txn.set_value_kind(ValueShapeWire::U32);
        txn.set_value(value);
        at(n, cmd)
    }

    /// One outcome, as the driver answers it.
    fn answered(n: i64, corr: u32, status: AuxStatusWire) -> Logged<AuxOutcomeWire> {
        let mut outcome = AuxOutcomeWire::new();
        outcome.set_corr(corr);
        outcome.set_status(status);
        outcome.set_value_kind(ValueShapeWire::U32);
        at(n, outcome)
    }

    /// An angle crosses the wire as its own IEEE-754 pattern, so a report that
    /// printed the eight bytes as an integer would print a mismatch nobody can
    /// subtract.
    #[test]
    fn an_angle_is_printed_as_an_angle_and_not_as_its_bit_pattern() {
        let mut cmd = SessionCmdWire::new();
        cmd.set_kind(SessionCmdKindWire::AUX);
        cmd.set_corr(7);
        let txn = cmd.txn_mut();
        txn.set_active(true);
        txn.set_op(AuxOpKindWire::WRITE_REG_VERIFIED);
        txn.set_id(15);
        txn.set_reg(RegIdWire::GOAL_POSITION);
        txn.set_value_kind(ValueShapeWire::RADIANS);
        txn.set_value(0.25_f64.to_bits());
        let mut outcome = answered(4, 7, AuxStatusWire::VERIFY_MISMATCH);
        outcome.message.set_value_kind(ValueShapeWire::RADIANS);
        outcome.message.set_value(0.5_f64.to_bits());
        let report = analyze(&traffic_run(vec![at(3, cmd)], vec![outcome]));
        assert!(
            report.findings.iter().any(|finding| {
                finding.contains("0.2500 rad") && finding.contains("reading back 0.5000 rad")
            }),
            "{:?}",
            report.findings
        );
    }

    /// A run of the four synthetic streams, with a heartbeat under it.
    fn traffic_run(
        datagrams: Vec<Logged<SessionCmdWire>>,
        outcomes: Vec<Logged<AuxOutcomeWire>>,
    ) -> Run {
        Run {
            samples: heartbeat(10),
            datagrams,
            outcomes,
            ..Run::default()
        }
    }

    /// A status on its own names nothing an operator can act on, so every
    /// outcome that is not `ok` is printed with the request it answers: which
    /// transaction, which servo, which register.
    #[test]
    fn a_failed_transaction_is_printed_with_the_identity_of_what_it_asked_for() {
        let report = analyze(&traffic_run(
            vec![asked(3, 7, 21, RegIdWire::GOAL_POSITION, 1234)],
            vec![answered(4, 7, AuxStatusWire::TIMEOUT)],
        ));
        assert!(
            measured_about(
                &report,
                "corr 7: AuxOpKindWire::WRITE_REG_VERIFIED servo 21 RegIdWire::GOAL_POSITION 1234"
            ),
            "{:?}",
            report.measured
        );
    }

    /// The pin sweep's write asks for no read-back, and an identity line says
    /// which of the two writes it was: a report that named them alike would
    /// leave an operator unable to tell a transaction that carries evidence
    /// from one that carries an acknowledgement.
    #[test]
    fn an_unverified_write_is_named_as_the_write_it_is() {
        let mut request = asked(3, 7, 21, RegIdWire::GOAL_POSITION, 1234);
        request.message.txn_mut().set_op(AuxOpKindWire::WRITE_REG);
        let report = analyze(&traffic_run(
            vec![request],
            vec![answered(4, 7, AuxStatusWire::TIMEOUT)],
        ));
        assert!(
            measured_about(
                &report,
                "corr 7: AuxOpKindWire::WRITE_REG servo 21 RegIdWire::GOAL_POSITION 1234"
            ),
            "{:?}",
            report.measured
        );
    }

    /// A verified write is its own read-back, so a servo holding something else
    /// is never expected and fails the run -- with what it read back, which is
    /// the number that says what happened.
    #[test]
    fn a_verify_mismatch_fails_the_run_and_says_what_read_back() {
        let mut outcome = answered(4, 7, AuxStatusWire::VERIFY_MISMATCH);
        outcome.message.set_value(99);
        let report = analyze(&traffic_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 1)],
            vec![outcome],
        ));
        assert_eq!(
            findings_about(
                &report,
                "servo holds something other than what the session wrote"
            ),
            1,
            "{:?}",
            report.findings
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("reading back 99")),
            "{:?}",
            report.findings
        );
    }

    /// An outcome names its own transaction, so a number whose request went
    /// past before the logger attached is still readable: the identity comes
    /// off the outcome's echo, and there is nothing left to fail the run for.
    #[test]
    fn an_outcome_with_no_request_behind_it_names_itself() {
        let mut outcome = answered(4, 7, AuxStatusWire::TIMEOUT);
        outcome.message.set_op(AuxOpKindWire::READ_REG);
        outcome.message.set_id(14);
        outcome.message.set_reg(RegIdWire::PRESENT_POSITION);
        let report = analyze(&traffic_run(Vec::new(), vec![outcome]));
        assert!(
            measured_about(
                &report,
                "AuxOpKindWire::READ_REG servo 14 RegIdWire::PRESENT_POSITION, named by the \
                 outcome itself"
            ),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "no request under that number is in the log"),
            0,
            "an outcome that says what it answers is not an orphan: {:?}",
            report.findings
        );
    }

    /// Delivery retry re-issues a datagram verbatim, so one number naming
    /// several byte-identical requests is the transport working and is counted,
    /// not judged.
    #[test]
    fn a_verbatim_re_issue_is_counted_and_not_a_finding() {
        let report = analyze(&traffic_run(
            vec![
                asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0),
                asked(4, 7, 21, RegIdWire::TORQUE_ENABLE, 0),
            ],
            vec![answered(5, 7, AuxStatusWire::OK)],
        ));
        assert!(
            measured_about(
                &report,
                "sent 2 times, byte for byte, which is delivery retry"
            ),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "different transactions"),
            0,
            "{:?}",
            report.findings
        );
    }

    /// A re-issue that is not verbatim is not a re-issue: it is the same
    /// transaction sent again with different inputs, which is the one retry this
    /// machine never performs.
    #[test]
    fn a_duplicate_number_with_a_different_payload_is_a_finding() {
        let report = analyze(&traffic_run(
            vec![
                asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0),
                asked(4, 7, 21, RegIdWire::TORQUE_ENABLE, 1),
            ],
            vec![answered(5, 7, AuxStatusWire::OK)],
        ));
        assert_eq!(
            findings_about(&report, "retry with perturbed inputs"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// The slot answers a collision with its own status, so a busy answer is
    /// read for which request the slot was holding rather than inferred from
    /// what else the trail carries. A number the log holds once is the ordinary
    /// collision: two numbers overlapped.
    #[test]
    fn a_busy_answer_under_a_number_the_log_holds_once_is_a_collision() {
        let report = analyze(&traffic_run(
            vec![
                asked(3, 6, 21, RegIdWire::TORQUE_ENABLE, 0),
                asked(4, 7, 22, RegIdWire::TORQUE_ENABLE, 0),
            ],
            vec![
                answered(5, 7, AuxStatusWire::BUSY),
                answered(6, 6, AuxStatusWire::OK),
            ],
        ));
        assert_eq!(
            findings_about(&report, "holding a request under another number"),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(
                &report,
                "declined the transaction before anything reached the bus"
            ),
            0,
            "a busy slot is not a decline: {:?}",
            report.findings
        );
    }

    /// A busy answer to a verbatim re-issue is a finding against the build
    /// rather than against the run: the slot answers a duplicate with the
    /// pending request's own outcome, so a driver that turns one away is one
    /// that does not recognise it.
    #[test]
    fn a_busy_answer_to_a_verbatim_re_issue_is_a_finding_against_the_driver() {
        let report = analyze(&traffic_run(
            vec![
                asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0),
                asked(4, 7, 21, RegIdWire::TORQUE_ENABLE, 0),
            ],
            vec![answered(5, 7, AuxStatusWire::BUSY)],
        ));
        assert_eq!(
            findings_about(&report, "built before its slot recognised a duplicate"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// One status record, as the driver republishes it: a healthy start, the
    /// two counters a case names, and a wind-down's copy.
    ///
    /// The two instants are ordered the way a run that waited orders them --
    /// the first command taken after the first sample was published -- so a
    /// case that is not about the survey does not accidentally state a race.
    fn stated(session_cmds: u64, aux_refused: u64) -> DriverStatusWire {
        let mut status = DriverStatusWire::new();
        status.set_sweep_time(when(-1));
        status.set_first_pose(when(0));
        status.set_first_session_cmd(when(2));
        status.set_wound_down(true);
        status.seam_mut().set_session_cmds(session_cmds);
        status.cycle_mut().set_aux_refused(aux_refused);
        status
    }

    /// The same run's records, with the driver's own account of them beside
    /// them.
    ///
    /// `session_cmds` is what the driver says it took off its socket, which is
    /// what the logged trail is cross-checked against. The record is a
    /// wound-down driver's, which is what makes it the run's last word;
    /// [`counted_run_killed`] is the same run without that.
    fn counted_run(
        datagrams: Vec<Logged<SessionCmdWire>>,
        outcomes: Vec<Logged<AuxOutcomeWire>>,
        session_cmds: u64,
    ) -> Run {
        Run {
            statuses: vec![at(9, stated(session_cmds, 0))],
            // The census says what the log holds, and a status this fixture
            // decoded is a status the log held: the two readings are of the
            // same message, and a run where they disagree is what `carriers`
            // has its own case for.
            census: vec![channel(STATUS_CHANNEL, 1, 0)],
            ..traffic_run(datagrams, outcomes)
        }
    }

    /// The same, over a run that ended without a wind-down: the last status is
    /// a periodic copy, so its counters can be up to one window older than the
    /// log.
    fn counted_run_killed(
        datagrams: Vec<Logged<SessionCmdWire>>,
        outcomes: Vec<Logged<AuxOutcomeWire>>,
        session_cmds: u64,
    ) -> Run {
        let mut run = counted_run(datagrams, outcomes, session_cmds);
        for status in &mut run.statuses {
            status.message.set_wound_down(false);
        }
        run
    }

    /// A refusal is the driver declining the transaction itself, and the slot
    /// that was already full says so under its own status. So the reading is
    /// direct: no counters, no re-issue and no outstanding number enter into
    /// it.
    #[test]
    fn a_refusal_is_read_as_a_decline_whatever_else_the_trail_holds() {
        for run in [
            counted_run(
                vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
                vec![answered(5, 7, AuxStatusWire::REFUSED)],
                1,
            ),
            // A trail the driver counted more datagrams than.
            counted_run(
                vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
                vec![answered(5, 7, AuxStatusWire::REFUSED)],
                2,
            ),
            // No counters at all.
            traffic_run(
                vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
                vec![answered(5, 7, AuxStatusWire::REFUSED)],
            ),
            // Another number still unanswered when it went out.
            counted_run(
                vec![
                    asked(3, 6, 21, RegIdWire::TORQUE_ENABLE, 0),
                    asked(4, 7, 22, RegIdWire::TORQUE_ENABLE, 0),
                ],
                vec![
                    answered(5, 7, AuxStatusWire::REFUSED),
                    answered(6, 6, AuxStatusWire::OK),
                ],
                2,
            ),
        ] {
            let report = analyze(&run);
            assert_eq!(
                findings_about(
                    &report,
                    "declined the transaction before anything reached the bus"
                ),
                1,
                "{:?}",
                report.findings
            );
        }
    }

    /// A number carrying two different payloads fails the run on its own, and a
    /// busy answer beside it says what the slot was holding was not what this
    /// number names.
    #[test]
    fn a_busy_answer_under_a_differing_payload_says_which_is_which() {
        let report = analyze(&counted_run(
            vec![
                asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0),
                asked(4, 7, 21, RegIdWire::TORQUE_ENABLE, 1),
            ],
            vec![answered(5, 7, AuxStatusWire::BUSY)],
            2,
        ));
        assert_eq!(
            findings_about(&report, "different payload"),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "retry with perturbed inputs"),
            1,
            "the two payloads are their own finding: {:?}",
            report.findings
        );
    }

    /// A refusal under a number the log holds no request for is still a
    /// refusal: the status says what the driver did, and the outcome's own echo
    /// says what it was about.
    #[test]
    fn a_refusal_with_no_request_behind_it_is_still_a_decline() {
        let report = analyze(&counted_run(
            Vec::new(),
            vec![answered(4, 7, AuxStatusWire::REFUSED)],
            0,
        ));
        assert_eq!(
            findings_about(
                &report,
                "declined the transaction before anything reached the bus"
            ),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "named by the outcome itself"),
            1,
            "the identity comes off the echo: {:?}",
            report.findings
        );
    }

    /// A busy answer says the slot was full; which request it was full of is
    /// read off the trail, and a trail holding no request under this number
    /// cannot say. The line says that much and no more -- the reading that
    /// separates a collision between two numbers from a slot holding this one
    /// under other bytes is not available.
    #[test]
    fn a_busy_answer_with_no_request_behind_it_says_what_the_trail_cannot() {
        let report = analyze(&counted_run(
            Vec::new(),
            vec![answered(4, 7, AuxStatusWire::BUSY)],
            0,
        ));
        assert_eq!(
            findings_about(
                &report,
                "cannot be read off a trail that is missing this one"
            ),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "was holding a request under another number"),
            0,
            "the trail does not support that reading: {:?}",
            report.findings
        );
    }

    /// The status only speaks for the whole run where the driver wound down.
    /// A run that ended some other way published its last copy on the window
    /// cadence, so its counters can be a window older than the log and a
    /// shortfall read off them would be arithmetic about the cadence.
    #[test]
    fn a_status_that_is_not_a_wind_downs_says_so() {
        let report = analyze(&counted_run_killed(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
            1,
        ));
        assert!(
            measured_about(&report, "and not a wind-down's"),
            "{:?}",
            report.measured
        );
    }

    /// The status is republished on a cadence, so a log holding *more* than the
    /// last copy counted is ordinary: no shortfall finding, and the trail still
    /// reads as whole.
    #[test]
    fn a_log_holding_more_than_the_status_counted_is_ordinary_and_not_a_shortfall() {
        let report = analyze(&counted_run(
            vec![
                asked(1, 6, 21, RegIdWire::TORQUE_ENABLE, 0),
                asked(3, 7, 22, RegIdWire::TORQUE_ENABLE, 0),
            ],
            vec![
                answered(2, 6, AuxStatusWire::OK),
                answered(4, 7, AuxStatusWire::REFUSED),
            ],
            1,
        ));
        assert_eq!(
            findings_about(&report, "fewer session datagrams than the driver counted"),
            0,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(
                &report,
                "declined the transaction before anything reached the bus"
            ),
            1,
            "{:?}",
            report.findings
        );
    }

    /// The driver counts the offers it turned away, and a turned-away offer is
    /// answered `busy`. So the counter is checked against the busy outcomes: a
    /// run holding one of each is whole, and a `refused` outcome -- a decline
    /// the driver counts nothing for -- neither makes a deficit up nor
    /// manufactures one.
    #[test]
    fn the_refusal_counter_is_checked_against_the_busy_outcomes() {
        let mut run = counted_run(
            vec![
                asked(1, 6, 21, RegIdWire::TORQUE_ENABLE, 0),
                asked(3, 7, 22, RegIdWire::TORQUE_ENABLE, 0),
            ],
            vec![
                answered(2, 6, AuxStatusWire::BUSY),
                answered(4, 7, AuxStatusWire::REFUSED),
            ],
            2,
        );
        run.statuses = vec![at(9, stated(2, 1))];
        let report = analyze(&run);
        assert!(
            measured_about(
                &report,
                "the driver counted 1 turned-away offers; the log holds 1 busy outcomes"
            ),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "nor a lost outcome at the front to account for"),
            0,
            "a busy outcome answers the counted offer: {:?}",
            report.findings
        );
    }

    /// One channel of a log, as the census keeps it.
    fn channel(name: &str, count: usize, first_seq: u32) -> Channel {
        Channel {
            name: name.to_string(),
            count,
            first_seq: Some(first_seq),
        }
    }

    /// The publisher numbers from zero and skips nothing, so the lowest number
    /// a channel carries is how much of it the log never got. Printed for every
    /// channel, and said again as a measured loss for the streams this tool
    /// reads.
    #[test]
    fn the_census_says_where_each_channels_copy_starts() {
        let mut run = counted_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
            1,
        );
        run.census = vec![
            channel(SESSION_CMD_CHANNEL, 1, 6),
            channel(POSE_CHANNEL, 10, 0),
            channel("/logger/status", 3, 4),
        ];
        let report = analyze(&run);
        assert!(
            measured_about(
                &report,
                "channel SessionCmdChan: 1 messages, from sequence 6"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            measured_about(
                &report,
                "SessionCmdChan begins at sequence 6: 6 messages predate the logger's attach"
            ),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "SessionCmdChan begins at sequence"),
            0,
            "a lost head is what a lazily attaching logger costs every run: {:?}",
            report.findings
        );
        assert!(
            !measured_about(&report, "DriverPose begins at sequence"),
            "a channel the log holds from the first publish lost nothing: {:?}",
            report.measured
        );
        assert!(
            !measured_about(&report, "/logger/status begins at sequence"),
            "the reading is about the streams this report reads: {:?}",
            report.measured
        );
    }

    /// Every channel this tool binds says how it is retained.
    ///
    /// The two tables are one table in two pieces, and the second is what the
    /// head-loss reading turns on. A channel bound above and forgotten here
    /// would be read as a regular one by omission, which is a decision nobody
    /// made.
    #[test]
    fn every_bound_channel_says_how_it_is_retained() {
        let bound: Vec<&str> = CHANNELS.iter().map(|channel| channel.name).collect();
        let declared: Vec<&str> = RETENTION.iter().map(|(name, _)| *name).collect();
        assert_eq!(
            bound, declared,
            "the retention table is the channel table, channel for channel and in its order",
        );
    }

    /// And what it says is what the systems that write these logs declare.
    ///
    /// The logging policy is the artifact that decides it; this table is a copy,
    /// and a copy of a policy is drift waiting to happen. Both online systems
    /// are read, because a log this tool judges comes off one or the other and
    /// they have to agree. The simulated system is deliberately not among them:
    /// its runner records every message from the first, so it has no attach
    /// instant for a retained message to be handed at, and its own policy says
    /// so.
    #[test]
    fn the_retention_table_is_what_the_online_systems_declare() {
        for system in ["SYSTEM_ROBOT_CLK", "SYSTEM_HOST_CLK"] {
            let text = runfile(system);
            for (name, kept) in RETENTION {
                let policy = policy_block(&text, name)
                    .unwrap_or_else(|| panic!("{system} declares no logging policy for {name}"));
                let declared = if policy.contains("ChannelType::persistent") {
                    Retention::Persistent
                } else {
                    Retention::Regular
                };
                assert_eq!(declared, kept, "{system}, channel {name}");
            }
        }
    }

    /// The environment variable `name` points at, read whole.
    ///
    /// Panics rather than answers: a missing runfile is a broken test target,
    /// not a case.
    fn runfile(name: &str) -> String {
        let path = std::env::var(name).unwrap_or_else(|_| {
            panic!(
                "{name} is unset: the test target has to name the file beside the data attribute \
                 that supplies it"
            )
        });
        std::fs::read_to_string(&path)
            .unwrap_or_else(|error| panic!("{name} names {path}, which does not read: {error}"))
    }

    /// The body of the logging policy `text` declares over the channel called
    /// `channel`, or `None` where it declares none.
    ///
    /// Matched on the channel's own name after the module qualifier, which is
    /// the join key `motion_channels` exists to hold: a channel renamed in one
    /// artifact and not the other fails as a missing policy.
    fn policy_block<'a>(text: &'a str, channel: &str) -> Option<&'a str> {
        let head = format!("policy ChannelLoggingPolicy for motion::{channel}\n");
        let start = text.find(&head)? + head.len();
        let rest = &text[start..];
        let end = rest.find('}')?;
        Some(&rest[..end])
    }

    /// One retained message is the whole of a persistent channel, so where its
    /// copy starts says nothing about completeness and nothing is read off it.
    /// What would be wrong is holding none, which is the carrier finding.
    #[test]
    fn a_persistent_channels_start_is_not_read_as_a_loss() {
        let mut run = counted_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
            1,
        );
        run.census = vec![
            channel(STATUS_CHANNEL, 1, 27),
            channel(REPORT_CHANNEL, 1, 9),
            channel(SCHEDULE_CHANNEL, 1, 3),
        ];
        let report = analyze(&run);
        for (name, _) in RETENTION
            .iter()
            .filter(|(_, kept)| *kept == Retention::Persistent)
        {
            assert!(
                !measured_about(&report, &format!("{name} begins at sequence")),
                "{name}: {:?}",
                report.measured
            );
        }
        assert_eq!(
            findings_about(&report, "the log holds no"),
            0,
            "both carriers are there: {:?}",
            report.findings
        );
    }

    /// The two carriers are what the rest of this report reads the run from,
    /// and both are retained for a subscriber that attaches at any instant. So
    /// a log holding none of one is a run that cannot be verified, and it is
    /// said unconditionally rather than excused by an attach time.
    #[test]
    fn a_log_missing_a_carrier_is_a_finding_naming_it() {
        let report = analyze(&traffic_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
        ));
        assert_eq!(
            findings_about(&report, "the log holds no DriverStatus message"),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "the log holds no ReportsOut message"),
            1,
            "{:?}",
            report.findings
        );

        let mut whole = counted_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
            1,
        );
        whole.census = vec![channel(STATUS_CHANNEL, 1, 0), channel(REPORT_CHANNEL, 2, 0)];
        let report = analyze(&whole);
        assert_eq!(
            findings_about(&report, "the log holds no"),
            0,
            "{:?}",
            report.findings
        );
    }

    /// A story that wrapped is said before the checks that walk its rows.
    ///
    /// The rows a wrapped story lost are its oldest, which are the ones those
    /// checks start from -- so what they say about a wrapped story is that the
    /// session never made the transitions, when what happened is that it made
    /// more of them than the ring holds. The count is the only thing that can
    /// tell the two apart, and it rides the message rather than the rows.
    #[test]
    fn a_story_that_dropped_rows_says_so_before_its_rows_are_read() {
        let mut run = counted_run(vec![], vec![], 0);
        run.census = vec![channel(STATUS_CHANNEL, 1, 0), channel(REPORT_CHANNEL, 4, 0)];
        run.reports = vec![phase(
            1,
            SessionPhaseWire::ACTIVE,
            SessionPhaseWire::ENGAGING,
        )];
        run.story_dropped = 7;

        let report = analyze(&run);

        assert_eq!(
            findings_about(&report, "dropped 7 rows off its front"),
            1,
            "{:?}",
            report.findings
        );

        run.story_dropped = 0;
        let report = analyze(&run);
        assert_eq!(
            findings_about(&report, "off its front"),
            0,
            "a story that lost nothing says nothing: {:?}",
            report.findings
        );
    }

    /// A carrier the log holds and this build could not read is said as that,
    /// and not as a dead publisher.
    ///
    /// The two questions are asked separately because the answers send whoever
    /// is at the bench in opposite directions: a channel the recording never
    /// held is a socket or a publish to go and look at, and a channel it held
    /// whole under another schema revision is this tool built at the wrong
    /// revision. Conflating them hands out the wrong first hypothesis, which
    /// the complaint list beside it then flatly contradicts.
    #[test]
    fn a_carrier_the_log_holds_and_this_build_cannot_read_says_which_it_is() {
        let mut run = traffic_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
        );
        // The recording holds both, and nothing decoded off either.
        run.census = vec![channel(STATUS_CHANNEL, 3, 0), channel(REPORT_CHANNEL, 2, 0)];

        let report = analyze(&run);

        assert_eq!(
            findings_about(&report, "could read none of them"),
            2,
            "one per carrier: {:?}",
            report.findings
        );
        assert_eq!(
            findings_about(&report, "the log holds no"),
            0,
            "a channel the log holds is not also reported as absent: {:?}",
            report.findings
        );
    }

    /// The loss the logger's attach costs a stream is charged to the log before
    /// the run is charged with anything: a trail short by exactly its measured
    /// head loss is whole from the first message the log holds.
    #[test]
    fn a_trail_short_by_its_head_loss_is_not_short_of_the_run() {
        let mut run = counted_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
            3,
        );
        run.census = vec![channel(SESSION_CMD_CHANNEL, 1, 2)];
        let report = analyze(&run);
        assert!(
            measured_about(
                &report,
                "counted 3 session datagrams; the log holds 1 and lost 2 off the front"
            ),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "over and above the 2 its attach cost it"),
            0,
            "{:?}",
            report.findings
        );
    }

    /// Loss beyond the head is loss mid-run -- a dropped datagram, a logger
    /// that fell behind -- and that is a defect the log's own head cannot
    /// excuse.
    #[test]
    fn a_trail_short_beyond_its_head_loss_is_a_finding() {
        let mut run = counted_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
            5,
        );
        run.census = vec![channel(SESSION_CMD_CHANNEL, 1, 2)];
        let report = analyze(&run);
        assert_eq!(
            findings_about(
                &report,
                "the log holds 2 fewer session datagrams than the driver counted, over and above \
                 the 2 its attach cost it"
            ),
            1,
            "{:?}",
            report.findings
        );
    }

    /// The outcome stream loses its head like any other, so the refusal
    /// arithmetic charges that loss before it charges the run.
    #[test]
    fn a_refusal_deficit_the_lost_outcomes_account_for_is_named_not_flagged() {
        let mut run = counted_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
            1,
        );
        run.statuses = vec![at(9, stated(1, 2))];
        run.census = vec![channel(AUX_OUT_CHANNEL, 1, 2)];
        let report = analyze(&run);
        assert!(
            measured_about(&report, "and by the outcomes the log lost at the front"),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "nor a lost outcome at the front to account for"),
            0,
            "{:?}",
            report.findings
        );
    }

    /// One class of refusal is invisible to everything but this cross-check: a
    /// datagram asking for nothing is counted and answered with nothing at all.
    /// A deficit of exactly that shape is named rather than read as loss.
    #[test]
    fn a_refusal_deficit_the_do_nothing_datagrams_account_for_is_named_not_flagged() {
        let mut nothing = SessionCmdWire::new();
        nothing.set_kind(SessionCmdKindWire::NONE);
        let mut run = counted_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0), at(4, nothing)],
            vec![answered(5, 7, AuxStatusWire::OK)],
            2,
        );
        run.statuses = vec![at(9, stated(2, 1))];
        let report = analyze(&run);
        assert!(
            measured_about(&report, "the do-nothing datagrams the driver counts"),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "nor a lost outcome at the front to account for"),
            0,
            "{:?}",
            report.findings
        );
    }

    /// A turned-away offer the driver counted that nothing in the log accounts
    /// for is a real gap, and the finding names both things that produce one.
    #[test]
    fn a_refusal_deficit_nothing_accounts_for_is_a_finding() {
        let mut run = counted_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
            1,
        );
        run.statuses = vec![at(9, stated(1, 2))];
        let report = analyze(&run);
        assert_eq!(
            findings_about(&report, "nor a lost outcome at the front to account for"),
            1,
            "{:?}",
            report.findings
        );
        assert_eq!(
            findings_about(
                &report,
                "only the first collision of a cycle leaves an outcome"
            ),
            1,
            "{:?}",
            report.findings
        );
    }

    /// The driver counts the re-issues its slot recognised, and the count is in
    /// every copy of the status, so it is read rather than inferred.
    #[test]
    fn the_duplicate_count_is_read_off_the_status() {
        let mut run = counted_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
            1,
        );
        run.statuses[0].message.cycle_mut().set_aux_duplicates(4);
        let report = analyze(&run);
        assert!(
            measured_about(&report, "recognised 4 offers as a re-issue"),
            "{:?}",
            report.measured
        );
    }

    /// A channel the log declared and carried nothing on has no first sequence
    /// number to say where its copy starts, and is counted rather than left
    /// out: a report group that reported nothing is what an operator is looking
    /// for when a group is missing from everything downstream.
    #[test]
    fn a_channel_the_log_carried_nothing_on_is_counted_without_a_start() {
        let mut run = traffic_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
        );
        run.census = vec![Channel {
            name: "/logger/status".to_string(),
            count: 0,
            first_seq: None,
        }];
        let report = analyze(&run);
        assert!(
            measured_about(&report, "channel /logger/status: 0 messages"),
            "{:?}",
            report.measured
        );
        assert!(
            !measured_about(&report, "/logger/status: 0 messages, from sequence"),
            "a channel with nothing on it says nothing about where it starts: {:?}",
            report.measured
        );
    }

    /// The session commissions only once the driver has shown a sample, so a
    /// command the driver took before it published one is the wait gone: it was
    /// still starting, and what it answers is the delivery re-issue.
    #[test]
    fn a_survey_that_asked_before_the_driver_answered_is_a_finding() {
        let mut run = counted_run(Vec::new(), Vec::new(), 1);
        run.statuses[0].message.set_first_session_cmd(when(-1));
        let report = analyze(&run);
        assert_eq!(
            findings_about(&report, "asked before anything said the driver was cycling"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// The driver drains its inbox before the cycle that publishes a sample, so
    /// a command stamped at the same instant as the first sample was taken
    /// before that sample existed. A tie is the race, not a near miss.
    #[test]
    fn a_command_taken_on_the_cycle_that_published_the_first_sample_is_the_race() {
        let mut run = counted_run(Vec::new(), Vec::new(), 1);
        run.statuses[0].message.set_first_session_cmd(when(0));
        let report = analyze(&run);
        assert_eq!(
            findings_about(&report, "asked before anything said the driver was cycling"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// Both instants are the driver's own, so the check is made whatever the
    /// log lost off the front of either stream -- which is the whole reason it
    /// reads them off the status.
    #[test]
    fn the_survey_is_read_off_the_status_and_not_off_the_streams() {
        let mut run = counted_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
            1,
        );
        run.census = vec![
            channel(POSE_CHANNEL, 10, 4),
            channel(SESSION_CMD_CHANNEL, 1, 8),
        ];
        let report = analyze(&run);
        assert_eq!(
            findings_about(&report, "asked before anything said the driver was cycling"),
            0,
            "a run that waited reads as one however much of either stream is gone: {:?}",
            report.findings
        );

        run.statuses[0].message.set_first_session_cmd(when(-1));
        let report = analyze(&run);
        assert_eq!(
            findings_about(&report, "asked before anything said the driver was cycling"),
            1,
            "and a run that raced reads as one on the same log: {:?}",
            report.findings
        );
    }

    /// A run nothing has commanded yet has no instant to order against, and a
    /// made-up one would read as an ordering.
    #[test]
    fn a_run_the_host_never_commanded_says_nothing_about_the_survey() {
        let mut run = counted_run(Vec::new(), Vec::new(), 0);
        run.statuses[0]
            .message
            .set_first_session_cmd(SyncTime::from_nanos(0));
        let report = analyze(&run);
        assert_eq!(
            findings_about(&report, "asked before anything said the driver was cycling"),
            0,
            "{:?}",
            report.findings
        );
    }

    /// The two kinds of out-of-band work are counted apart: a health triple is
    /// three exchanges where a host request is one, so which kind a run's skips
    /// follow is what says which to measure first.
    #[test]
    fn a_skip_after_a_host_transaction_is_attributed_to_it() {
        let report = analyze(&Run {
            samples: heartbeat(10),
            outcomes: vec![answered(3, 7, AuxStatusWire::OK)],
            events: vec![
                skip(5, 1, 30_000_000),
                stats(9, 50, 1, 30_000_000, 8_000_000),
            ],
            ..Run::default()
        });
        assert_eq!(
            findings_about(
                &report,
                "0 after a health reading and 1 after a host transaction"
            ),
            1,
            "{:?}",
            report.findings
        );
    }

    /// The mirror of the orphan rule: the sequencer's retry and timeout
    /// machinery guarantees every accepted transaction an answer, so a request
    /// the log holds and nothing answered is the log's problem or a new bug.
    #[test]
    fn a_request_nothing_ever_answered_is_a_finding() {
        let report = analyze(&traffic_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            Vec::new(),
        ));
        assert_eq!(
            findings_about(&report, "was never answered at all"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// The process writes the minimum risk condition before anything else it
    /// does on the bus, and the instant rides every status copy -- so the
    /// startup record is readable off a log whose event stream begins well
    /// after cycle 0, which is every healthy run.
    #[test]
    fn the_startup_release_is_read_off_the_status() {
        let report = analyze(&counted_run(Vec::new(), Vec::new(), 0));
        assert!(
            measured_about(&report, "wrote the minimum risk condition at"),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "could not verify the start-up release"),
            0,
            "{:?}",
            report.findings
        );
    }

    /// A row whose verified write did not read back is a machine the process
    /// could not see let go of, and that is judged the way an unconfirmed
    /// de-torquing is judged anywhere else here.
    #[test]
    fn a_start_that_left_a_row_unverified_is_a_finding_naming_it() {
        let mut run = counted_run(Vec::new(), Vec::new(), 0);
        run.statuses[0]
            .message
            .set_sweep_failed_rows(JointFlagsWire(u16::from(
                reachy_motion::joints::flags::bit(reachy_motion::joints::JointRef::AntennaLeft),
            )));
        let report = analyze(&run);
        assert_eq!(
            findings_about(&report, "could not verify the start-up release"),
            1,
            "{:?}",
            report.findings
        );
    }

    /// Every start has this instant, so a status without one is a run that
    /// cannot say the machine was ever released -- and unlike the event, it
    /// cannot be blamed on when the logger attached.
    #[test]
    fn a_status_with_no_sweep_instant_is_a_finding() {
        let mut run = counted_run(Vec::new(), Vec::new(), 0);
        run.statuses[0]
            .message
            .set_sweep_time(SyncTime::from_nanos(0));
        let report = analyze(&run);
        assert_eq!(
            findings_about(&report, "carries no instant for the minimum risk condition"),
            1,
            "{:?}",
            report.findings
        );
        assert!(
            !measured_about(&report, "wrote the minimum risk condition at"),
            "there is no instant to print: {:?}",
            report.measured
        );
    }

    /// The counters are cumulative and every copy is the whole run so far, so
    /// the last one is the run's totals and the earlier ones are the same
    /// numbers, smaller.
    #[test]
    fn the_counters_come_from_the_last_status_the_log_holds() {
        let mut run = counted_run(
            vec![asked(3, 7, 21, RegIdWire::TORQUE_ENABLE, 0)],
            vec![answered(5, 7, AuxStatusWire::OK)],
            1,
        );
        run.statuses = vec![at(4, stated(1, 0)), at(9, stated(1, 0))];
        run.statuses[0].message.seam_mut().set_session_cmds(0);
        let report = analyze(&run);
        assert!(
            measured_about(&report, "counted 1 session datagrams; the log holds 1"),
            "{:?}",
            report.measured
        );
        assert_eq!(
            findings_about(&report, "fewer session datagrams than the driver counted"),
            0,
            "{:?}",
            report.findings
        );
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
