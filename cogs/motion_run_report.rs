//! Shared motion-run log reading, measurements and the settle instrument used by the tour,
//! supplied-script and idle reports.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use brenn_reachy__cogs__schedule_clk_rs::{SessionScheduleWire, StepKindWire};
use brenn_reachy__cogs__script_clk_rs::ScriptWire;
use brenn_reachy__driver__health_clk_rs::{DriverEventWire, EventKindWire, HealthReportWire};
use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
use brenn_reachy__motion__faults_clk_rs::TickFaultWire;
use log_read::{Bound, Census, Complaints, Logged, Streams, binding, read_with, typed};
use motion_channels::{
    EVENT_CHANNEL, FAULT_CHANNEL, HEALTH_CHANNEL, POSE_CHANNEL, SCHEDULE_CHANNEL, SCRIPT_CHANNEL,
};
use pose_reading::{
    Grid, Residual, RunConfig, Skips, capabilities, capability, commanded_rows, health_summary,
    lag_scan, lag_scans, lags, present_rows, residual_stream, residuals,
};
use reachy_driver::NOMINAL_CYCLE_NS;
use reachy_edge::names::MotionTable;
use reachy_motion::joints::{JointRef, Name, ROWS, row};
use reachy_motion::phase::{ANTENNA_CONTACT_BAND_RAD, inside_band, mirror_offset};
use reachy_motion::plant::GroupPlants;
use reachy_motion::stillness::{COUNT_RAD, MAX_EXCURSION_RAD, whole_counts};
use run_report::Report;
use stillness_report::{Standard, Stillness, say};

const SAMPLE_GAP_NS: i64 = NOMINAL_CYCLE_NS + NOMINAL_CYCLE_NS / 2;
const MOVED_RAD: f64 = 1e-9;

#[derive(Default)]
pub struct Run {
    pub scripts: Vec<Logged<ScriptWire>>,
    pub schedules: Vec<Logged<SessionScheduleWire>>,
    pub samples: Vec<Logged<PoseSampleWire>>,
    pub events: Vec<Logged<DriverEventWire>>,
    pub faults: Vec<Logged<TickFaultWire>>,
    pub readings: Vec<Logged<HealthReportWire>>,
    pub census: Census,
    pub complaints: Complaints,
}

impl Streams for Run {
    fn census(&mut self) -> &mut Census {
        &mut self.census
    }
    fn complaints(&mut self) -> &mut Complaints {
        &mut self.complaints
    }
}

pub const CHANNELS: [Bound<Run>; 6] = [
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
        name: HEALTH_CHANNEL,
        check: binding::<HealthReportWire>,
        route: |run, message| typed(message, &mut run.readings, &mut run.complaints),
    },
];

pub fn read(dir: &Path) -> Result<Run, clockwork_logs::LogError> {
    read_with(dir, &CHANNELS)
}

impl Run {
    pub fn ordered_samples(&self) -> Vec<&Logged<PoseSampleWire>> {
        let mut ordered: Vec<_> = self.samples.iter().collect();
        ordered.sort_by_key(|sample| sample.message.nominal_time().as_nanos());
        ordered
    }

    pub fn ordered_events(&self) -> Vec<&Logged<DriverEventWire>> {
        let mut ordered: Vec<_> = self.events.iter().collect();
        ordered.sort_by_key(|event| event.message.time().as_nanos());
        ordered
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Window {
    pub motion_id: u16,
    pub start_ns: i64,
    pub end_ns: i64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Span {
    pub start_ns: i64,
    pub end_ns: i64,
}

impl Window {
    pub fn span(self) -> Span {
        Span {
            start_ns: self.start_ns,
            end_ns: self.end_ns,
        }
    }
}

pub fn overlay_spans(windows: &[Window]) -> Vec<Span> {
    windows.iter().copied().map(Window::span).collect()
}

/// One place a replacing schedule cut a playing window short.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Seam {
    /// The replacing schedule's log time.
    pub at_ns: i64,
    /// The window it dropped, ending at `at_ns`.
    pub outgoing: Window,
    /// The window the replacing schedule opened, as the plan ends it; none when
    /// the replacing schedule plays nothing new.
    pub incoming: Option<Window>,
}

pub struct Prepared<'a> {
    pub ordered: Vec<&'a Logged<PoseSampleWire>>,
    pub events: Vec<&'a Logged<DriverEventWire>>,
    pub grid: Grid,
    pub skips: Skips<'a>,
    pub plant: GroupPlants,
    pub stream: Vec<(i64, [Residual; ROWS.len()])>,
}

pub fn prepare<'a>(run: &'a Run, config: &RunConfig) -> Result<Prepared<'a>, String> {
    let ordered = run.ordered_samples();
    let origin = ordered
        .first()
        .expect("prepare is called after the nonempty-sample check")
        .message
        .nominal_time()
        .as_nanos();
    let grid = Grid {
        origin_ns: origin,
        period_ns: NOMINAL_CYCLE_NS,
    };
    let skips = Skips::of(&run.events, grid, 0);
    let plant = GroupPlants::from_profiles(&config.profiles, grid.period_ns)
        .map_err(|error| format!("the run's profiles do not form a plant: {error}"))?;
    let stream = residual_stream(&run.samples, grid, &plant);
    Ok(Prepared {
        ordered,
        events: run.ordered_events(),
        grid,
        skips,
        plant,
        stream,
    })
}

/// Every window that opened, as the schedules end it.
pub fn windows(run: &Run) -> Vec<Window> {
    windows_and_seams(run).0
}

/// Every window that opened, as the schedules end it, and every seam. A window
/// dropped at or before its start never opened and is not returned. A window still
/// open and not carried by a schedule is dropped at that schedule's log time. A
/// seam is a drop that cut a window already playing and not yet closed; a window
/// that closed before the next schedule, or one that never opened, is not one.
/// The seam's incoming window is the first window, in start order, that the
/// replacing schedule was first to carry, as the plan finally ends it.
pub fn windows_and_seams(run: &Run) -> (Vec<Window>, Vec<Seam>) {
    let mut ordered: Vec<_> = run.schedules.iter().collect();
    ordered.sort_by_key(|schedule| schedule.at_ns);
    let mut planned: BTreeMap<(i64, u16), (i64, bool)> = BTreeMap::new();
    let mut drops = Vec::new();
    for schedule in ordered {
        let at = schedule.at_ns;
        let mut carried = BTreeSet::new();
        let mut fresh = BTreeSet::new();
        for window in schedule.message.overlays().iter() {
            let key = (window.start().as_nanos(), window.motion_id());
            if !planned.contains_key(&key) {
                fresh.insert(key);
            }
            carried.insert(key);
            planned
                .entry(key)
                .and_modify(|held| {
                    if held.1 {
                        held.0 = window.end().as_nanos()
                    }
                })
                .or_insert((window.end().as_nanos(), true));
        }
        let opened = fresh.first().copied();
        let mut unopened: Vec<(i64, u16)> = Vec::new();
        for (key, held) in &mut planned {
            if held.1 && !carried.contains(key) {
                if at <= key.0 {
                    unopened.push(*key);
                } else {
                    if at < held.0 {
                        let outgoing = Window {
                            motion_id: key.1,
                            start_ns: key.0,
                            end_ns: at,
                        };
                        drops.push((at, outgoing, opened));
                    }
                    held.0 = held.0.min(at);
                }
                held.1 = false;
            }
        }
        for key in unopened {
            planned.remove(&key);
        }
    }
    let windows = planned
        .iter()
        .map(|(&(start_ns, motion_id), &(end_ns, _))| Window {
            motion_id,
            start_ns,
            end_ns,
        })
        .collect();
    let seams = drops
        .into_iter()
        .map(|(at_ns, outgoing, opened)| Seam {
            at_ns,
            outgoing,
            incoming: opened.and_then(|key| {
                planned.get(&key).map(|&(end_ns, _)| Window {
                    motion_id: key.1,
                    start_ns: key.0,
                    end_ns,
                })
            }),
        })
        .collect();
    (windows, seams)
}

/// What a base row asked for, which is what its settle lines are labelled by.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub enum SettleTarget {
    /// A library pose, by index.
    Pose(u16),
    /// A look, in the row's milliradians.
    Look {
        bearing_mrad: i32,
        elevation_mrad: i32,
    },
}

impl std::fmt::Display for SettleTarget {
    /// `pose 3`, or a look's bearing and elevation in whole degrees, the bearing
    /// signed: `look +30°/27°`.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let degrees = |mrad: i32| (f64::from(mrad) / 1000.0).to_degrees();
        match *self {
            Self::Pose(id) => write!(f, "pose {id}"),
            Self::Look {
                bearing_mrad,
                elevation_mrad,
            } => write!(
                f,
                "look {:+.0}°/{:.0}°",
                degrees(bearing_mrad),
                degrees(elevation_mrad)
            ),
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct SettleMove {
    pub start_ns: i64,
    pub end_ns: i64,
    pub target: SettleTarget,
    pub pace_ns: i64,
    /// When the last of the six legs came to rest; none for a move that was
    /// skipped or never settled.
    pub settled_at_ns: Option<i64>,
    pub reason: Option<String>,
}

impl SettleMove {
    /// Whether the move produced figures.
    pub fn measured(&self) -> bool {
        self.settled_at_ns.is_some()
    }
}

/// One leg's worst figure over a run, and where it was read.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LegFigure {
    pub counts: f64,
    pub target: SettleTarget,
    pub at_ns: i64,
}

/// One figure per leg, indexed Leg0..Leg5; none where no move was measured.
pub type LegTable = [Option<LegFigure>; 6];

#[derive(Clone, Debug)]
pub struct SettleResult {
    pub moves: Vec<SettleMove>,
    pub residual: LegTable,
    pub overshoot: LegTable,
    pub creep: LegTable,
}

impl SettleResult {
    /// Whether any leg of any of the three tables is unavailable.
    pub fn any_unavailable(&self) -> bool {
        [&self.residual, &self.overshoot, &self.creep]
            .iter()
            .any(|table| table.iter().any(Option::is_none))
    }
}

/// The bound each leg's `max(settled residual, overshoot)` is judged against,
/// in counts.
///
/// The largest judged figure over the four settle-evidence walks flown on
/// 2026-09-27 -- both walks, each flown twice, every directed transition among
/// the five committed poses -- rounded up to a whole count: 24.9 counts, leg 4,
/// `peek_tilt -> hello`, on walk 1's first flight and on its re-fly alike. The
/// first flights read per-leg maxima of 22.6, 16.2, 19.6, 24.9, 16.5 and 11.6
/// counts, and the re-flies 22.6, 15.2, 20.1, 24.9, 18.5 and 10.6; on
/// 2026-09-20 the same moves read 17.4 at most. The settled residual includes
/// the integral term's wind-down and the arrival overshoot on purpose: a leg at
/// rest short of its target, or past it, stands that far from where the
/// envelope checked it. The residual is whatever a run leaves between goal and
/// present, including servo behaviour, load, and possible head/body contact.
/// Because head/body interference is not modelled
/// (`TODO(head-body-interference)`), this is a run bound and not a measured
/// property of a servo. The envelope floor is derived to provide at least three
/// times this bound at the outer merge.
pub const SETTLE_BOUND_COUNTS: f64 = 25.0;

/// How far a leg at rest may wander, radians: the encoder's own flicker, two
/// counts, judged in whole counts.
pub const SETTLE_BAND_RAD: f64 = MAX_EXCURSION_RAD;

/// How long a leg must stay inside the band to count as at rest.
pub const SETTLE_WINDOW_NS: i64 = 200_000_000;

/// The readings a window holds on the driver's nominal grid: ten.
const SETTLE_WINDOW_SAMPLES: usize = (SETTLE_WINDOW_NS / NOMINAL_CYCLE_NS) as usize;

/// The six legs, in the order every settle table is indexed by.
const LEGS: [JointRef; 6] = [
    JointRef::Leg0,
    JointRef::Leg1,
    JointRef::Leg2,
    JointRef::Leg3,
    JointRef::Leg4,
    JointRef::Leg5,
];

/// The six legs' values out of a sample's nine rows.
fn leg_values(rows: &[f64; ROWS.len()]) -> [f64; 6] {
    LEGS.map(|joint| row(joint).map_or(0.0, |index| rows[index]))
}

/// One sample of a hold, decoded to what the settle figures read.
#[derive(Clone, Copy)]
struct Reading {
    at_ns: i64,
    commanded: [f64; 6],
    present: [f64; 6],
}

impl Reading {
    /// The sample's legs, or nothing where it holds no commanded or no present
    /// rows.
    fn of(sample: &Logged<PoseSampleWire>) -> Option<Self> {
        Some(Self {
            at_ns: sample.message.nominal_time().as_nanos(),
            commanded: leg_values(&commanded_rows(&sample.message)?),
            present: leg_values(&present_rows(&sample.message)?),
        })
    }

    /// Leg `k`'s distance from its goal, in counts.
    fn error_counts(&self, k: usize) -> f64 {
        (self.commanded[k] - self.present[k]).abs() / COUNT_RAD
    }
}

/// The largest of `figures`, with the instant it was first reached.
fn peak(figures: impl Iterator<Item = (f64, i64)>) -> Option<(f64, i64)> {
    let mut best: Option<(f64, i64)> = None;
    for (figure, at_ns) in figures {
        if best.is_none_or(|(old, _)| figure > old) {
            best = Some((figure, at_ns));
        }
    }
    best
}

/// Keep `figure` in `slot` where the slot is empty or the figure is larger.
fn offer(slot: &mut Option<LegFigure>, figure: LegFigure) {
    if slot.is_none_or(|old| figure.counts > old.counts) {
        *slot = Some(figure);
    }
}

/// One leg's three figures over one settled move.
struct LegFigures {
    settled_at_ns: i64,
    residual: (f64, i64),
    overshoot: (f64, i64),
    creep: f64,
}

/// Where each base move's legs came to rest, and what they read from there.
///
/// Every `BASE_POSTURE` and `BASE_LOOK` row is a move, judged over
/// `[endpoint, clean_end)`: from the last commanded leg change inside the row to
/// the row's end or the first overlay opening over it, whichever is sooner.
/// Rest is read from position, because no velocity is recorded: a leg is at
/// rest from the first reading whose next `SETTLE_WINDOW_NS` of present values
/// stay within `SETTLE_BAND_RAD`, read in whole counts. From there each leg
/// reports three figures, in counts:
///
/// - the settled residual, the largest `|commanded − present|` from rest to the
///   clean end;
/// - the overshoot, the largest distance past the target in the direction the
///   leg was commanded to travel, over the whole hold; at most zero is a leg
///   that never passed its target;
/// - the creep, the mean error over the hold's last window less the mean over
///   the window the leg came to rest in, signed: positive is a leg losing ground
///   off its target, negative the integral term winding it on.
///
/// `max(settled residual, overshoot)` is judged against `SETTLE_BOUND_COUNTS`;
/// the creep is printed and judges nothing. A move whose hold holds no complete
/// window, or on which a leg never comes to rest, is unsettled and skipped.
pub fn settle(
    run: &Run,
    ordered: &[&Logged<PoseSampleWire>],
    overlays: &[Window],
    report: &mut Report,
) -> SettleResult {
    let mut keys = BTreeSet::new();
    for schedule in &run.schedules {
        for step in schedule.message.steps().iter() {
            let target = match step.kind() {
                StepKindWire::BASE_POSTURE => SettleTarget::Pose(step.pose_id()),
                StepKindWire::BASE_LOOK => SettleTarget::Look {
                    bearing_mrad: step.bearing_mrad(),
                    elevation_mrad: step.elevation_mrad(),
                },
                _ => continue,
            };
            keys.insert((
                step.start().as_nanos(),
                step.end().as_nanos(),
                target,
                step.pace().as_nanos(),
            ));
        }
    }
    let mut residual: LegTable = [None; 6];
    let mut overshoot: LegTable = [None; 6];
    let mut creep: LegTable = [None; 6];
    let mut moves = Vec::new();
    for (start, end, target, pace_ns) in keys {
        let clean_end = overlays
            .iter()
            .filter(|window| window.start_ns < end && window.end_ns > start)
            .map(|window| window.start_ns)
            .min()
            .map_or(end, |at| end.min(at));
        let (settled_at_ns, reason) = match settle_move(ordered, overlays, start, clean_end) {
            Ok((endpoint, figures)) => {
                let settled_at = figures
                    .iter()
                    .map(|leg| leg.settled_at_ns)
                    .max()
                    .expect("six legs");
                report.measured.push(format!(
                    "base-move {target} measured: endpoint {endpoint}, settled at {settled_at}, \
                     clean end {clean_end}"
                ));
                for (k, leg) in figures.iter().enumerate() {
                    report.measured.push(format!(
                        "base-move {target} {}: settled at {}, residual {:.1}, overshoot {:.1}, \
                         creep {:+.1} counts",
                        Name(LEGS[k]),
                        leg.settled_at_ns,
                        leg.residual.0,
                        leg.overshoot.0,
                        leg.creep
                    ));
                    offer(
                        &mut residual[k],
                        LegFigure {
                            counts: leg.residual.0,
                            target,
                            at_ns: leg.residual.1,
                        },
                    );
                    offer(
                        &mut overshoot[k],
                        LegFigure {
                            counts: leg.overshoot.0,
                            target,
                            at_ns: leg.overshoot.1,
                        },
                    );
                    offer(
                        &mut creep[k],
                        LegFigure {
                            counts: leg.creep,
                            target,
                            at_ns: leg.settled_at_ns,
                        },
                    );
                }
                (Some(settled_at), None)
            }
            Err(reason) => (None, Some(reason)),
        };
        moves.push(SettleMove {
            start_ns: start,
            end_ns: end,
            target,
            pace_ns,
            settled_at_ns,
            reason,
        });
    }
    for (figure, table) in [("settled residual", &residual), ("overshoot", &overshoot)] {
        for (k, joint) in LEGS.iter().enumerate() {
            let leg = Name(*joint);
            match table[k] {
                Some(LegFigure {
                    counts,
                    target,
                    at_ns,
                }) => {
                    report.measured.push(format!(
                        "base-move {figure} {leg}: {counts:.1} counts ({target} at {at_ns})"
                    ));
                    if counts > SETTLE_BOUND_COUNTS {
                        report.fail(format!(
                            "base-move {figure} exceeds {SETTLE_BOUND_COUNTS:.0} counts at {leg}: \
                             {counts:.1} counts ({target} at {at_ns})"
                        ));
                    }
                }
                None => report
                    .measured
                    .push(format!("base-move {figure} {leg}: unavailable")),
            }
        }
    }
    for (k, joint) in LEGS.iter().enumerate() {
        let leg = Name(*joint);
        match creep[k] {
            Some(LegFigure {
                counts,
                target,
                at_ns,
            }) => report.measured.push(format!(
                "base-move creep {leg}: {counts:+.1} counts ({target} settled at {at_ns})"
            )),
            None => report
                .measured
                .push(format!("base-move creep {leg}: unavailable")),
        }
    }
    let measured = moves.iter().filter(|item| item.measured()).count();
    let skipped = moves.len() - measured;
    report.note(format!("{measured} measured, {skipped} skipped base moves"));
    for item in &moves {
        if let Some(reason) = item.reason.as_deref() {
            report.note(format!("base move {} skipped: {reason}", item.target));
        }
    }
    SettleResult {
        moves,
        residual,
        overshoot,
        creep,
    }
}

/// One move's endpoint and per-leg figures over `[start, clean_end)`, or why it
/// has none.
fn settle_move(
    ordered: &[&Logged<PoseSampleWire>],
    overlays: &[Window],
    start: i64,
    clean_end: i64,
) -> Result<(i64, [LegFigures; 6]), String> {
    if let Some(window) = overlays
        .iter()
        .find(|window| window.start_ns <= start && window.end_ns > start)
    {
        return Err(format!(
            "overlay motion {} begins at {} before base start {}",
            window.motion_id, window.start_ns, start
        ));
    }
    let mut endpoint = None;
    let mut origin = None;
    let mut endpoint_commanded = None;
    for pair in ordered.windows(2) {
        let at = pair[1].message.nominal_time().as_nanos();
        let (Some(before), Some(after)) = (
            commanded_rows(&pair[0].message),
            commanded_rows(&pair[1].message),
        ) else {
            continue;
        };
        let (before, after) = (leg_values(&before), leg_values(&after));
        let changed = before.iter().zip(&after).any(|(b, a)| b != a);
        if at >= start && at < clean_end && changed {
            endpoint = Some(at);
            origin.get_or_insert(before);
            endpoint_commanded = Some(after);
        }
    }
    let (Some(endpoint), Some(origin), Some(endpoint_commanded)) =
        (endpoint, origin, endpoint_commanded)
    else {
        return Err(format!("no commanded change in [{start}, {clean_end})"));
    };
    let at = |sample: &&Logged<PoseSampleWire>| sample.message.nominal_time().as_nanos();
    let lo = ordered.partition_point(|sample| at(sample) < endpoint);
    let hi = ordered.partition_point(|sample| at(sample) < clean_end);
    let hold = &ordered[lo..hi];
    let readings: Vec<Option<Reading>> = hold.iter().map(|sample| Reading::of(sample)).collect();
    let mut complete: Vec<(usize, usize)> = Vec::new();
    for (i, sample) in hold.iter().enumerate() {
        let closes = at(sample) + SETTLE_WINDOW_NS;
        if closes > clean_end {
            break;
        }
        let j = i + hold[i..].partition_point(|later| at(later) < closes);
        if j - i >= SETTLE_WINDOW_SAMPLES && readings[i..j].iter().all(Option::is_some) {
            complete.push((i, j));
        }
    }
    let Some(&last) = complete.last() else {
        return Err(format!(
            "unsettled before clean end {clean_end}: no complete {} ms of readings after \
             endpoint {endpoint}",
            SETTLE_WINDOW_NS / 1_000_000
        ));
    };
    let window = |(i, j): (usize, usize)| readings[i..j].iter().flatten();
    let rested: [Option<(usize, usize)>; 6] = std::array::from_fn(|k| {
        complete.iter().copied().find(|&span| {
            let (lo, hi) = window(span).fold((f64::INFINITY, f64::NEG_INFINITY), |(lo, hi), r| {
                (lo.min(r.present[k]), hi.max(r.present[k]))
            });
            // In whole counts: the band is two counts as stated, and a two-count flicker is inside it wherever on the encoder the leg stands.
            whole_counts(hi - lo) <= SETTLE_BAND_RAD / COUNT_RAD
        })
    });
    let unsettled: Vec<String> = LEGS
        .iter()
        .zip(&rested)
        .filter(|(_, span)| span.is_none())
        .map(|(joint, _)| Name(*joint).to_string())
        .collect();
    if !unsettled.is_empty() {
        return Err(format!(
            "unsettled before clean end {clean_end}: {} never stood within {:.0} counts for {} ms",
            unsettled.join(", "),
            SETTLE_BAND_RAD / COUNT_RAD,
            SETTLE_WINDOW_NS / 1_000_000
        ));
    }
    let mean_error = |span: (usize, usize), k: usize| {
        let (sum, n) =
            window(span).fold((0.0, 0_u32), |(sum, n), r| (sum + r.error_counts(k), n + 1));
        sum / f64::from(n)
    };
    let figures = std::array::from_fn(|k| {
        let rest = rested[k].expect("every leg came to rest");
        let settled_at_ns = hold[rest.0].message.nominal_time().as_nanos();
        let decoded = || readings.iter().flatten();
        let travel = endpoint_commanded[k] - origin[k];
        let dir = (travel.abs() > MOVED_RAD).then(|| travel.signum());
        LegFigures {
            settled_at_ns,
            residual: peak(
                decoded()
                    .filter(|r| r.at_ns >= settled_at_ns)
                    .map(|r| (r.error_counts(k), r.at_ns)),
            )
            .expect("the window the leg rested in holds readings"),
            overshoot: peak(decoded().map(|r| {
                let past = dir.map_or(0.0, |dir| (r.present[k] - r.commanded[k]) * dir / COUNT_RAD);
                (past, r.at_ns)
            }))
            .expect("the hold holds readings"),
            creep: mean_error(last, k) - mean_error(rest, k),
        }
    });
    Ok((endpoint, figures))
}

pub fn whole_stream_measurements(
    prepared: &Prepared<'_>,
    run: &Run,
    config: &RunConfig,
    report: &mut Report,
) {
    residuals(&prepared.stream, &run.samples, &prepared.plant, report);
    lags(&run.samples, report);
    let measured = capability(&run.samples, prepared.grid);
    capabilities(&measured, report);
    match lag_scan(&run.samples, prepared.grid, &config.profiles, &measured) {
        Ok(scans) => lag_scans(&scans, report),
        Err(error) => report.fail(error),
    }
    health_summary(&run.readings, report);
}
/// What the sidecar calls the motion at `motion_id`, or the index itself where
/// it names none.
pub fn named_motion(by_id: &BTreeMap<u16, String>, motion_id: u16) -> String {
    by_id
        .get(&motion_id)
        .cloned()
        .unwrap_or_else(|| format!("motion {motion_id}, which the sidecar does not name"))
}

/// The samples the sorted stream holds inside `window`.
///
/// A slice of the one sorted stream rather than a scan of it: every check below
/// walks every window, and a scan apiece is windows x samples on a tool whose
/// two factors both grow with the library.
pub fn inside<'a>(
    ordered: &'a [&'a Logged<PoseSampleWire>],
    window: &Window,
) -> &'a [&'a Logged<PoseSampleWire>] {
    let at = |sample: &&Logged<PoseSampleWire>| sample.message.nominal_time().as_nanos();
    let lo = ordered.partition_point(|sample| at(sample) < window.start_ns);
    let hi = ordered.partition_point(|sample| at(sample) < window.end_ns);
    &ordered[lo..hi]
}

/// The driver events the sorted stream holds inside `window`.
pub fn events_inside<'a>(
    ordered: &'a [&'a Logged<DriverEventWire>],
    window: &Window,
) -> &'a [&'a Logged<DriverEventWire>] {
    let at = |event: &&Logged<DriverEventWire>| event.message.time().as_nanos();
    let lo = ordered.partition_point(|event| at(event) < window.start_ns);
    let hi = ordered.partition_point(|event| at(event) < window.end_ns);
    &ordered[lo..hi]
}

/// Every window moved the machine.
///
/// A window the mover refused, or one whose layer was latched off for the
/// schedule epoch, leaves the composed setpoint exactly where the base was
/// holding it. So a window across which the goal never changed is a motion the
/// machine did not play, whatever the plan said -- and that is this tool's
/// whole-library assertion: it fails on the first clip the envelope refuses
/// over the raised base, which is the discovery the run exists for.
pub fn every_window_moved(
    ordered: &[&Logged<PoseSampleWire>],
    planned: &[Window],
    by_id: &BTreeMap<u16, String>,
    report: &mut Report,
) {
    for window in planned {
        let samples = inside(ordered, window);
        let mut held: Option<[f64; ROWS.len()]> = None;
        let mut moved = false;
        for sample in samples {
            let Some(commanded) = commanded_rows(&sample.message) else {
                continue;
            };
            match held {
                None => held = Some(commanded),
                Some(first) => {
                    if first
                        .iter()
                        .zip(commanded.iter())
                        .any(|(a, b)| (a - b).abs() > MOVED_RAD)
                    {
                        moved = true;
                    }
                }
            }
        }
        if samples.is_empty() {
            report.fail(format!(
                "the window over {} carries no sample at all, so nothing says what the machine \
                 did in it",
                named_motion(by_id, window.motion_id)
            ));
            continue;
        }
        if held.is_none() {
            report.fail(format!(
                "no sample inside the window over {} carried a setpoint, so the machine was \
                 under no command for the whole of it",
                named_motion(by_id, window.motion_id)
            ));
            continue;
        }
        if !moved {
            report.fail(format!(
                "the goal never changed across the window over {}, which is a composed setpoint \
                 the mover refused or a layer latched off rather than a motion that played",
                named_motion(by_id, window.motion_id)
            ));
        }
    }
}

/// The worst figure over the nine rows, and the joint that carried it.
#[derive(Clone, Copy, Default)]
pub struct Worst {
    /// The figure itself, radians.
    figure: f64,
    /// Which joint stood at it.
    joint: Option<JointRef>,
}

impl Worst {
    /// Keep `figure` if `joint` stands further out than anything so far.
    fn offer(&mut self, joint: JointRef, figure: f64) {
        if self.joint.is_none() || figure > self.figure {
            self.figure = figure;
            self.joint = Some(joint);
        }
    }
}

impl std::fmt::Display for Worst {
    /// The figure and the joint, or what a window nothing was measured over
    /// says instead.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.joint {
            Some(joint) => write!(f, "{:.4} rad at {}", self.figure, Name(joint)),
            None => f.write_str("nothing measured"),
        }
    }
}

/// How near a pair of antennas came to meeting over a stretch of samples.
///
/// The measurement that stands in for a crossing check on content: nothing
/// screens a clip's antenna track at import, so what the tips did is read off
/// the run. Only samples with both antennas inside the contact band count --
/// outside it a pair is clear of the other's arc whatever it is doing, and a
/// mirror offset taken there says nothing about meeting.
#[derive(Clone, Copy, Default)]
pub struct Nearest {
    /// The smallest mirror offset seen with both antennas inside the band.
    offset: Option<f64>,
}

impl Nearest {
    /// Offer one sample's pair.
    fn offer(&mut self, rows: &[f64; ROWS.len()]) {
        let (Some(right), Some(left)) = (row(JointRef::AntennaRight), row(JointRef::AntennaLeft))
        else {
            return;
        };
        if !inside_band(rows[right], ANTENNA_CONTACT_BAND_RAD)
            || !inside_band(rows[left], ANTENNA_CONTACT_BAND_RAD)
        {
            return;
        }
        let offset = mirror_offset(rows[right], rows[left]);
        self.offset = Some(match self.offset {
            Some(nearest) => nearest.min(offset),
            None => offset,
        });
    }
}

impl std::fmt::Display for Nearest {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self.offset {
            Some(offset) => write!(f, "{offset:.3} rad from mirrored"),
            None => f.write_str("never both inside the band"),
        }
    }
}

/// What each motion's window measured, one line apiece.
///
/// The numbers a plant model is fitted from and the numbers an operator reads
/// the run by, which are the same numbers. Nothing here fails a run: a lag is a
/// reading, and what a reading means is not this tool's to decide until
/// something knows what the servo can do.
pub fn measurements(
    ordered: &[&Logged<PoseSampleWire>],
    events: &[&Logged<DriverEventWire>],
    planned: &[Window],
    by_id: &BTreeMap<u16, String>,
    stream: &[(i64, [Residual; ROWS.len()])],
    report: &mut Report,
) {
    for window in planned {
        let mut samples = 0_usize;
        let mut lag = Worst::default();
        let mut residual = Worst::default();
        let mut step = Worst::default();
        let mut commanded_tips = Nearest::default();
        let mut present_tips = Nearest::default();
        let mut previous: Option<[f64; ROWS.len()]> = None;
        for sample in inside(ordered, window) {
            samples += 1;
            let commanded = commanded_rows(&sample.message);
            if let Some(commanded) = commanded {
                commanded_tips.offer(&commanded);
                if let Some(before) = previous {
                    for joint in ROWS {
                        let Some(index) = row(joint) else { continue };
                        step.offer(joint, (commanded[index] - before[index]).abs());
                    }
                }
                previous = Some(commanded);
            }
            if let Some(present) = present_rows(&sample.message) {
                present_tips.offer(&present);
                if let Some(commanded) = commanded {
                    for joint in ROWS {
                        let Some(index) = row(joint) else { continue };
                        lag.offer(joint, (commanded[index] - present[index]).abs());
                    }
                }
            }
        }
        // The residuals are the whole run's, stepped once in nominal order --
        // a prediction is history and cannot be restarted at a window's edge --
        // so a window reads the slice of them its own instants cover.
        let lo = stream.partition_point(|(nominal, _)| *nominal < window.start_ns);
        let hi = stream.partition_point(|(nominal, _)| *nominal < window.end_ns);
        for (_, figures) in &stream[lo..hi] {
            for joint in ROWS {
                let Some(index) = row(joint) else { continue };
                // The magnitude: the window's line names the joint furthest off
                // its own model, and which side of the model it stood is the
                // whole run's reading rather than a window's.
                residual.offer(joint, figures[index].magnitude());
            }
        }
        let skipped = events_inside(events, window)
            .iter()
            .filter(|event| event.message.kind() == EventKindWire::CYCLE_SKIPPED)
            .count();
        // The window's own two instants, printed so an operator can hand them
        // to `//cogs:trace_export` to cut a replay fixture.
        report.note(format!(
            "{} [{} .. {}]: {samples} sample(s), worst residual {residual}, worst lag {lag}, \
             peak step {step} per period, {skipped} skipped cycle(s), commanded tips \
             {commanded_tips}, present tips {present_tips}",
            named_motion(by_id, window.motion_id),
            window.start_ns,
            window.end_ns
        ));
    }
}

pub fn window_measurements(
    prepared: &Prepared<'_>,
    planned: &[Window],
    by_id: &BTreeMap<u16, String>,
    report: &mut Report,
) {
    measurements(
        &prepared.ordered,
        &prepared.events,
        planned,
        by_id,
        &prepared.stream,
        report,
    );
}

/// Whether the antennas stood still where the head let them.
///
/// The tour's own stillness verdict, over the same measurement the motion
/// report prints: the holds are cut out of the sample stream, and an antenna
/// hold is judged only where no head row was commanded somewhere new across it.
/// A tour holds the antennas mostly while the head is moving, and such a hold
/// reads the rod following the platform it is mounted on rather than the
/// antenna's own loop -- so it is printed with its figures and no verdict, and
/// a tour that held none with the head still says it measured nothing rather
/// than failing.
///
/// The whole stream rather than the windows: a hold that opens inside one
/// motion and closes inside the next is a hold, and the raise and the closing
/// stow are where the head stands longest.
///
/// `standard` is the one thing this tool's two kinds of run disagree about, and
/// it comes off the table the run was asked for: see [`held_standard`].
pub fn stillness(ordered: &[&Logged<PoseSampleWire>], standard: Standard, report: &mut Report) {
    let mut held = Stillness::default();
    for sample in ordered {
        held.sample(&sample.message);
    }
    held.finish();
    say(&held, standard, report);
}

/// Which stillness standard the run this table describes is judged under.
///
/// A probe run's table holds instruments alone, and a probe is a step goal
/// followed by a hold with the head standing at the raised base: the head-still
/// hold is the whole run, so a run that produced none measured nothing it was
/// asked to measure and fails. A content tour holds the antennas almost only
/// while the head moves, so the same absence there is a reading of the content
/// and fails nothing.
///
/// A table of the body yaw's probes alone is judged by the yaw's own row,
/// because the antennas stand still across the yaw's motion there and no
/// antenna hold is head-still.
pub fn held_standard(table: &MotionTable) -> Standard {
    if table.yaw_probes_only() {
        Standard::YawProbe
    } else if table.probes_only() {
        Standard::JudgedWhereHeadStillRequired
    } else {
        Standard::JudgedWhereHeadStill
    }
}
/// The sample stream held for the whole of the stretch judged, which `over`
/// names in what the report says.
///
/// The record is the point of the run, so a stretch of it the log does not hold
/// is a finding whatever else the run did. Judged between the first window
/// opening and the last one closing: what the driver did before the stretch
/// began and after it ended is not the stretch's.
///
/// A gap the driver itself reported as skipped cycles is not one of those. It
/// is the machine saying it missed its slots, which is a reading the run was
/// taken to get, and the two are classified oppositely: a hole in the record is
/// a harness defect to fix and re-run, a skipped cycle is the plant's own
/// answer. So the gaps a skip report accounts for are counted apart and said
/// apart.
///
/// One finding rather than one per gap: a run that dropped a stretch drops
/// hundreds of samples, and a report of hundreds of identical lines is one
/// nobody reads to the end.
pub fn the_stream_held(
    ordered: &[&Logged<PoseSampleWire>],
    planned: &[Span],
    grid: Grid,
    skips: &Skips<'_>,
    report: &mut Report,
    over: &str,
) {
    let (Some(first), Some(last)) = (
        planned.iter().map(|span| span.start_ns).min(),
        planned.iter().map(|span| span.end_ns).max(),
    ) else {
        return;
    };
    let held: Vec<i64> = ordered
        .iter()
        .map(|sample| sample.message.nominal_time().as_nanos())
        .filter(|at| *at >= first && *at < last)
        .collect();
    if held.is_empty() {
        report.fail(
            "the log holds no sample between the first window opening and the last one closing"
                .to_string(),
        );
        return;
    }
    let mut gaps = 0_usize;
    let mut accounted = 0_usize;
    let mut worst = (0_i64, 0_i64);
    // The stretch judged runs from the first window opening to
    // the last one closing, so the two ends are gaps of their own: a stream
    // that started late or stopped early holds no pair to read them off.
    let mut ends = vec![(first, held[0])];
    ends.extend(held.windows(2).map(|pair| (pair[0], pair[1])));
    ends.push((held[held.len() - 1], last));
    for (from, to) in ends {
        if to - from <= SAMPLE_GAP_NS {
            continue;
        }
        let missing = grid.at(from).0 + 1..grid.at(to).0;
        if !missing.is_empty() && skips.account_for(missing) {
            accounted += 1;
            continue;
        }
        gaps += 1;
        if to - from > worst.0 {
            worst = (to - from, from);
        }
    }
    if gaps > 0 {
        report.fail(format!(
            "the sample stream has {gaps} gap(s) over {over} that no skipped-cycle report \
             accounts for; the longest is {:.1} ms from {}",
            worst.0 as f64 / 1e6,
            worst.1
        ));
    }
    if accounted > 0 {
        report.note(format!(
            "{accounted} further stretch(es) of the stream are cycles the driver reported as \
             skipped, which is the machine's own answer rather than a hole in the record"
        ));
    }
    report.note(format!(
        "{} sample(s) over {over}, between the first window opening and the last one closing",
        held.len()
    ));
}

#[cfg(test)]
mod settle_invariant_tests {
    use super::SETTLE_BOUND_COUNTS;
    use reachy_kin::baked;
    use reachy_motion::stillness::COUNT_RAD;

    #[test]
    fn default_outer_clearance_is_three_times_the_settle_bound() {
        let margin = reachy_kin::envelope::EnvelopeConfig::default().min_toggle_margin;
        let a = baked::CRANK_LEN;
        let r = baked::ROD_LEN;
        let rho = a + r - margin;
        let angle = ((a * a + rho * rho - r * r) / (2.0 * a * rho)).acos();
        assert!(angle >= 3.0 * SETTLE_BOUND_COUNTS * COUNT_RAD);
    }
}

#[cfg(test)]
mod window_fold_tests {
    //! How schedules end windows and where seams fall.

    use brenn_reachy__cogs__schedule_clk_rs::{OverlayWindowWire, SessionScheduleWire};
    use clockwork_rs::SyncTime;
    use log_read::Logged;
    use reachy_driver::NOMINAL_CYCLE_NS;

    use super::{Run, Seam, Window, windows_and_seams};

    /// An arbitrary instant a synthetic run starts at, chosen for being nothing
    /// round.
    const T0: i64 = 1_772_000_000_123_456_789;

    /// The instant cycle `n` of a synthetic run sits at.
    fn at_cycle(n: i64) -> i64 {
        T0 + n * NOMINAL_CYCLE_NS
    }

    /// A schedule logged at cycle `n`, carrying one window per
    /// `(motion_id, start, end)`, in cycles.
    fn schedule(n: i64, windows: &[(u16, i64, i64)]) -> Logged<SessionScheduleWire> {
        let mut message = SessionScheduleWire::new();
        message.set_engaged(true);
        {
            let mut rows = message.overlays_mut();
            rows.clear();
            for &(motion_id, start, end) in windows {
                let row: &mut OverlayWindowWire =
                    rows.try_grow().expect("a schedule of few windows");
                row.set_motion_id(motion_id);
                row.set_start(SyncTime::from_nanos(at_cycle(start)));
                row.set_end(SyncTime::from_nanos(at_cycle(end)));
                row.set_gain(1.0);
                row.set_speed(1.0);
            }
        }
        Logged {
            at_ns: at_cycle(n),
            sequence_number: u32::try_from(n).expect("a small cycle"),
            message,
        }
    }

    /// A window over `motion_id` from cycle `start` to cycle `end`.
    fn w(motion_id: u16, start: i64, end: i64) -> Window {
        Window {
            motion_id,
            start_ns: at_cycle(start),
            end_ns: at_cycle(end),
        }
    }

    /// What the fold makes of `schedules`.
    fn fold(schedules: Vec<Logged<SessionScheduleWire>>) -> (Vec<Window>, Vec<Seam>) {
        windows_and_seams(&Run {
            schedules,
            ..Run::default()
        })
    }

    /// One row of the fold's table: a name, the schedules, and the windows and
    /// seams they fold to.
    type Case = (
        &'static str,
        Vec<Logged<SessionScheduleWire>>,
        Vec<Window>,
        Vec<Seam>,
    );

    #[test]
    fn schedules_end_windows_and_mark_seams() {
        let replaced = || vec![schedule(0, &[(0, 1, 40)]), schedule(20, &[(1, 21, 60)])];
        let mut twice = replaced();
        twice.push(schedule(40, &[(0, 41, 80)]));
        let cases: Vec<Case> = vec![
            (
                "no replacement",
                vec![schedule(0, &[(0, 1, 40)])],
                vec![w(0, 1, 40)],
                vec![],
            ),
            (
                "replacement mid-play",
                replaced(),
                vec![w(0, 1, 20), w(1, 21, 60)],
                vec![Seam {
                    at_ns: at_cycle(20),
                    outgoing: w(0, 1, 20),
                    incoming: Some(w(1, 21, 60)),
                }],
            ),
            (
                "two consecutive replacements",
                twice,
                vec![w(0, 1, 20), w(1, 21, 40), w(0, 41, 80)],
                vec![
                    Seam {
                        at_ns: at_cycle(20),
                        outgoing: w(0, 1, 20),
                        incoming: Some(w(1, 21, 40)),
                    },
                    Seam {
                        at_ns: at_cycle(40),
                        outgoing: w(1, 21, 40),
                        incoming: Some(w(0, 41, 80)),
                    },
                ],
            ),
            (
                "replacement landing exactly on the window's end",
                vec![schedule(0, &[(0, 1, 20)]), schedule(20, &[(1, 21, 60)])],
                vec![w(0, 1, 20), w(1, 21, 60)],
                vec![],
            ),
            (
                "the playing window carried alongside a new one",
                vec![
                    schedule(0, &[(0, 1, 40)]),
                    schedule(20, &[(0, 1, 40), (1, 41, 60)]),
                ],
                vec![w(0, 1, 40), w(1, 41, 60)],
                vec![],
            ),
            (
                "replacement playing nothing new",
                vec![schedule(0, &[(0, 1, 40)]), schedule(20, &[])],
                vec![w(0, 1, 20)],
                vec![Seam {
                    at_ns: at_cycle(20),
                    outgoing: w(0, 1, 20),
                    incoming: None,
                }],
            ),
            (
                "dropped before it opened",
                vec![schedule(0, &[(0, 100, 140)]), schedule(50, &[])],
                vec![],
                vec![],
            ),
            (
                "dropped exactly at its start",
                vec![schedule(0, &[(0, 50, 90)]), schedule(50, &[])],
                vec![],
                vec![],
            ),
        ];
        for (case, schedules, windows, seams) in cases {
            let (got_windows, got_seams) = fold(schedules);
            for window in &got_windows {
                assert!(window.start_ns < window.end_ns, "{case}: {window:?}");
            }
            assert_eq!(got_windows, windows, "{case}");
            assert_eq!(got_seams, seams, "{case}");
        }
    }
}

#[cfg(test)]
mod settle_tests {
    //! The settle instrument over synthetic runs: where legs come to rest and
    //! the three figures read from there.

    use brenn_reachy__cogs__schedule_clk_rs::{
        ScheduledStepWire, SessionScheduleWire, StepKindWire,
    };
    use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
    use clockwork_rs::{Duration, SyncTime};
    use dxl_proto::counts_to_rad;
    use log_read::Logged;
    use reachy_driver::NOMINAL_CYCLE_NS;
    use reachy_motion::joints::{JointRef, ROW_COUNT, row, write_rows};
    use reachy_motion::stillness::COUNT_RAD;
    use run_report::Report;

    use super::{Run, SETTLE_BOUND_COUNTS, SettleResult, SettleTarget, settle};

    /// An arbitrary instant a synthetic run starts at, chosen for being nothing
    /// round.
    const T0: i64 = 1_772_000_000_123_456_789;

    /// The instant cycle `n` of a synthetic run sits at.
    fn at_cycle(n: i64) -> i64 {
        T0 + n * NOMINAL_CYCLE_NS
    }

    /// A one-step schedule over cycles `[start, end)`.
    fn one_step(
        start: i64,
        end: i64,
        fill: impl FnOnce(&mut ScheduledStepWire),
    ) -> Logged<SessionScheduleWire> {
        let mut message = SessionScheduleWire::new();
        message.set_engaged(true);
        {
            let mut steps = message.steps_mut();
            let step: &mut ScheduledStepWire = steps.try_grow().expect("one base step fits");
            step.set_start(SyncTime::from_nanos(at_cycle(start)));
            step.set_end(SyncTime::from_nanos(at_cycle(end)));
            fill(step);
        }
        Logged {
            at_ns: at_cycle(start),
            sequence_number: 0,
            message,
        }
    }

    /// A posture row over cycles `[start, end)` at five cycles' pace.
    fn posture(start: i64, end: i64, pose_id: u16) -> Logged<SessionScheduleWire> {
        one_step(start, end, |step| {
            step.set_kind(StepKindWire::BASE_POSTURE);
            step.set_pose_id(pose_id);
            step.set_pace(Duration::from_nanos(5 * NOMINAL_CYCLE_NS));
        })
    }

    /// A look row over cycles `[start, end)`, which carries no pace.
    fn look(
        start: i64,
        end: i64,
        bearing_mrad: i32,
        elevation_mrad: i32,
    ) -> Logged<SessionScheduleWire> {
        one_step(start, end, |step| {
            step.set_kind(StepKindWire::BASE_LOOK);
            step.set_bearing_mrad(bearing_mrad);
            step.set_elevation_mrad(elevation_mrad);
        })
    }

    /// A valid sample at cycle `n`, legs in counts, every other row zero.
    fn legs(n: i64, commanded: [f64; 6], present: [f64; 6]) -> Logged<PoseSampleWire> {
        legs_rad(
            n,
            commanded.map(|c| c * COUNT_RAD),
            present.map(|c| c * COUNT_RAD),
        )
    }

    /// A valid sample at cycle `n`, legs in radians, every other row zero.
    fn legs_rad(n: i64, commanded: [f64; 6], present: [f64; 6]) -> Logged<PoseSampleWire> {
        let leg0 = row(JointRef::Leg0).expect("leg row");
        let mut commanded_rows = [0.0; ROW_COUNT];
        let mut present_rows = [0.0; ROW_COUNT];
        commanded_rows[leg0..leg0 + 6].copy_from_slice(&commanded);
        present_rows[leg0..leg0 + 6].copy_from_slice(&present);
        let mut message = PoseSampleWire::new();
        {
            let read = message.clear_valid();
            read.nominal_time = SyncTime::from_nanos(at_cycle(n));
            read.sample_time = SyncTime::from_nanos(at_cycle(n));
            read.present_valid = true.into();
            read.commanded_valid = true.into();
            write_rows(&mut read.present, &present_rows);
            write_rows(&mut read.commanded, &commanded_rows);
        }
        Logged {
            at_ns: at_cycle(n),
            sequence_number: u32::try_from(n).expect("a small cycle"),
            message,
        }
    }

    /// What the instrument reads off `schedules` and `samples`, with no overlay.
    fn read(
        schedules: Vec<Logged<SessionScheduleWire>>,
        samples: Vec<Logged<PoseSampleWire>>,
    ) -> (SettleResult, Report) {
        let run = Run {
            schedules,
            samples,
            ..Run::default()
        };
        let mut report = Report::default();
        let result = settle(&run, &run.ordered_samples(), &[], &mut report);
        (result, report)
    }

    /// Every leg commanded 100 from cycle 10, ramping in, flickering over three
    /// counts, then over one.
    fn ramp_and_flicker() -> Vec<Logged<PoseSampleWire>> {
        (0..100)
            .map(|n| {
                let commanded = if n < 10 { 0.0 } else { 100.0 };
                let present = match n {
                    0..=9 => 0.0,
                    10..=19 => 9.6 * (n - 9) as f64,
                    20..=24 => {
                        if n % 2 == 0 {
                            99.0
                        } else {
                            96.0
                        }
                    }
                    _ => {
                        if n % 2 == 0 {
                            97.0
                        } else {
                            96.0
                        }
                    }
                };
                legs(n, [commanded; 6], [present; 6])
            })
            .collect()
    }

    fn close(got: f64, want: f64) -> bool {
        (got - want).abs() < 1e-9
    }

    /// Every leg commanded to count 1025 from cycle 10 and reading `settled(n)`
    /// from there, every value converted from counts as the driver records it.
    fn flicker_at_1024(settled: impl Fn(i64) -> i32) -> Vec<Logged<PoseSampleWire>> {
        (0..60)
            .map(|n| {
                let (commanded, present) = if n < 10 {
                    (counts_to_rad(1000), counts_to_rad(1000))
                } else {
                    (counts_to_rad(1025), counts_to_rad(settled(n)))
                };
                legs_rad(n, [commanded; 6], [present; 6])
            })
            .collect()
    }

    /// A leg flickering over two counts is at rest at a position where the
    /// spread in radians reads a rounding past the two-count band.
    #[test]
    fn a_two_count_flicker_rests_wherever_on_the_encoder_the_leg_stands() {
        assert!(counts_to_rad(1026) - counts_to_rad(1024) > super::SETTLE_BAND_RAD);
        let samples = flicker_at_1024(|n| 1024 + (n % 3) as i32);
        let (result, report) = read(vec![posture(1, 60, 7)], samples);
        assert_eq!(result.moves[0].settled_at_ns, Some(at_cycle(10)));
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        for k in 0..6 {
            let residual = result.residual[k].expect("residual");
            assert!(close(residual.counts, 1.0), "{residual:?}");
        }
    }

    /// The negative control: at the same position a three-count flicker is
    /// never at rest.
    #[test]
    fn a_three_count_flicker_is_not_at_rest_wherever_the_leg_stands() {
        let samples = flicker_at_1024(|n| 1024 + (n % 4) as i32);
        let (result, _report) = read(vec![posture(1, 60, 7)], samples);
        let only = &result.moves[0];
        assert_eq!(only.settled_at_ns, None);
        let reason = only.reason.as_deref().expect("a reason");
        assert!(reason.contains("never stood within 2 counts"), "{reason}");
    }

    #[test]
    fn settled_at_follows_the_ramp_and_the_flicker() {
        let (result, report) = read(vec![posture(1, 100, 7)], ramp_and_flicker());
        assert_eq!(result.moves[0].settled_at_ns, Some(at_cycle(25)));
        for k in 0..6 {
            let residual = result.residual[k].expect("residual");
            assert!(close(residual.counts, 4.0), "{residual:?}");
            assert_eq!(residual.at_ns, at_cycle(25));
            let overshoot = result.overshoot[k].expect("overshoot");
            assert!(close(overshoot.counts, -1.0), "{overshoot:?}");
        }
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        let line = format!(
            "base-move pose 7 measured: endpoint {}, settled at {}, clean end {}",
            at_cycle(10),
            at_cycle(25),
            at_cycle(100)
        );
        assert!(report.measured.contains(&line), "{:?}", report.measured);
    }

    #[test]
    fn a_hold_that_never_comes_to_rest_or_ends_inside_the_window_is_unsettled() {
        let restless = (0..100)
            .map(|n| {
                let commanded = if n < 10 { 0.0 } else { 100.0 };
                let present = match n {
                    0..=9 => 0.0,
                    10..=19 => 9.6 * (n - 9) as f64,
                    _ => {
                        if n % 2 == 0 {
                            99.0
                        } else {
                            96.0
                        }
                    }
                };
                legs(n, [commanded; 6], [present; 6])
            })
            .collect();
        let (result, report) = read(vec![posture(1, 100, 7)], restless);
        let only = &result.moves[0];
        assert_eq!(only.settled_at_ns, None);
        assert!(!only.measured());
        let reason = only.reason.as_deref().expect("a reason");
        assert!(
            reason.contains("never stood within 2 counts for 200 ms"),
            "{reason}"
        );
        assert!(
            reason.contains("leg 1, leg 2, leg 3, leg 4, leg 5, leg 6"),
            "{reason}"
        );
        for table in [&result.residual, &result.overshoot, &result.creep] {
            assert!(table.iter().all(Option::is_none));
        }
        assert!(
            report
                .measured
                .iter()
                .any(|line| line == "0 measured, 1 skipped base moves"),
            "{:?}",
            report.measured
        );
        assert_eq!(
            report
                .measured
                .iter()
                .filter(|line| line.ends_with(": unavailable"))
                .count(),
            18,
            "{:?}",
            report.measured
        );

        let short = (0..18)
            .map(|n| {
                let value = if n < 10 { 0.0 } else { 100.0 };
                legs(n, [value; 6], [value; 6])
            })
            .collect();
        let (result, _) = read(vec![posture(1, 18, 7)], short);
        assert_eq!(
            result.moves[0].reason.as_deref(),
            Some(
                format!(
                    "unsettled before clean end {}: no complete 200 ms of readings after \
                     endpoint {}",
                    at_cycle(18),
                    at_cycle(10)
                )
                .as_str()
            )
        );
    }

    #[test]
    fn overshoot_counts_only_past_the_target() {
        let samples = (0..100)
            .map(|n| {
                let (commanded, present) = match n {
                    0..=9 => ([0.0; 6], [0.0; 6]),
                    10..=14 => (
                        [100.0, -100.0, -100.0, 0.0, 0.0, 0.0],
                        [103.0, -97.0, -103.0, 0.0, 0.0, 0.0],
                    ),
                    _ => (
                        [100.0, -100.0, -100.0, 0.0, 0.0, 0.0],
                        [100.0, -97.0, -100.0, 0.0, 0.0, 0.0],
                    ),
                };
                legs(n, commanded, present)
            })
            .collect();
        let (result, _) = read(vec![posture(1, 100, 7)], samples);
        let expected = [
            (3.0, 0.0),
            (-3.0, 3.0),
            (3.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
        ];
        for (k, (overshoot, residual)) in expected.into_iter().enumerate() {
            let got = result.overshoot[k].expect("overshoot").counts;
            assert!(close(got, overshoot), "leg {k} overshoot {got}");
            let got = result.residual[k].expect("residual").counts;
            assert!(close(got, residual), "leg {k} residual {got}");
        }
        assert_eq!(result.moves[0].settled_at_ns, Some(at_cycle(15)));
    }

    #[test]
    fn creep_reads_growth_away_from_the_target_and_the_wind_down_onto_it() {
        let samples = (0..210)
            .map(|n| {
                if n < 10 {
                    return legs(n, [0.0; 6], [0.0; 6]);
                }
                let q = ((n - 10) / 20) as f64;
                let flicker = if n % 2 == 0 { 50.0 } else { 51.0 };
                legs(
                    n,
                    [50.0; 6],
                    [50.0 + q, 40.0 + q, flicker, 50.0, 50.0, 50.0],
                )
            })
            .collect();
        let (result, report) = read(vec![posture(1, 210, 7)], samples);
        let expected = [
            (9.0, 9.0),
            (-9.0, 10.0),
            (0.0, 1.0),
            (0.0, 0.0),
            (0.0, 0.0),
            (0.0, 0.0),
        ];
        for (k, (creep, residual)) in expected.into_iter().enumerate() {
            let got = result.creep[k].expect("creep").counts;
            assert!(close(got, creep), "leg {k} creep {got}");
            let got = result.residual[k].expect("residual").counts;
            assert!(close(got, residual), "leg {k} residual {got}");
        }
        assert_eq!(result.moves[0].settled_at_ns, Some(at_cycle(10)));
        let line = format!(
            "base-move creep leg 2: -9.0 counts (pose 7 settled at {})",
            at_cycle(10)
        );
        assert!(report.measured.contains(&line), "{:?}", report.measured);
    }

    #[test]
    fn a_look_row_is_read_like_a_posture_row() {
        let (result, report) = read(vec![look(1, 100, 520, 471)], ramp_and_flicker());
        let target = SettleTarget::Look {
            bearing_mrad: 520,
            elevation_mrad: 471,
        };
        assert_eq!(result.moves[0].target, target);
        assert_eq!(result.residual[0].expect("residual").target, target);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.starts_with("base-move look +30°/27° measured: ")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.starts_with("base-move settled residual ")
                    && line.contains("(look +30°/27° at ")),
            "{:?}",
            report.measured
        );
        assert_eq!(
            SettleTarget::Look {
                bearing_mrad: -611,
                elevation_mrad: 0
            }
            .to_string(),
            "look -35°/0°"
        );
    }

    #[test]
    fn the_bound_judges_the_residual_and_the_overshoot() {
        let stream = |before: f64, after: f64| {
            (0..100)
                .map(|n| {
                    let commanded = if n < 10 { before } else { after };
                    legs(n, [commanded; 6], [0.0; 6])
                })
                .collect()
        };
        let b = SETTLE_BOUND_COUNTS;
        let (_, clean) = read(vec![posture(1, 100, 7)], stream(0.0, b));
        assert!(clean.findings.is_empty(), "{:?}", clean.findings);

        let (_, short) = read(vec![posture(1, 100, 7)], stream(0.0, b + 0.1));
        assert_eq!(short.findings.len(), 6, "{:?}", short.findings);
        let residual = format!(
            "base-move settled residual exceeds {b:.0} counts at leg 1: {:.1} counts (pose 7 at ",
            b + 0.1
        );
        assert!(
            short
                .findings
                .iter()
                .any(|finding| finding.contains(&residual)),
            "{:?}",
            short.findings
        );
        assert!(
            !short
                .findings
                .iter()
                .any(|finding| finding.contains("overshoot exceeds")),
            "{:?}",
            short.findings
        );

        let (_, past) = read(vec![posture(1, 100, 7)], stream(2.0 * b + 0.2, b + 0.1));
        let overshoot = format!(
            "base-move overshoot exceeds {b:.0} counts at leg 1: {:.1} counts",
            b + 0.1
        );
        assert!(
            past.findings
                .iter()
                .any(|finding| finding.contains(&overshoot)),
            "{:?}",
            past.findings
        );
    }

    #[test]
    fn republished_schedules_measure_once() {
        let (result, report) = read(
            vec![posture(1, 100, 7), posture(1, 100, 7)],
            ramp_and_flicker(),
        );
        assert_eq!(result.moves.len(), 1);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line == "1 measured, 0 skipped base moves"),
            "{:?}",
            report.measured
        );
    }
}
