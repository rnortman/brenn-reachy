//! Shared motion-run log decoding seam used by the tour and supplied-script reports.

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use brenn_reachy__cogs__schedule_clk_rs::SessionScheduleWire;
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
use reachy_motion::stillness::COUNT_RAD;
use run_report::Report;
use stillness_report::{Standard, Stillness, say};

const SAMPLE_GAP_NS: i64 = NOMINAL_CYCLE_NS + NOMINAL_CYCLE_NS / 2;
const MOVED_RAD: f64 = 1e-9;

#[derive(Default)]
pub(crate) struct Run {
    pub(crate) scripts: Vec<Logged<ScriptWire>>,
    pub(crate) schedules: Vec<Logged<SessionScheduleWire>>,
    pub(crate) samples: Vec<Logged<PoseSampleWire>>,
    pub(crate) events: Vec<Logged<DriverEventWire>>,
    pub(crate) faults: Vec<Logged<TickFaultWire>>,
    pub(crate) readings: Vec<Logged<HealthReportWire>>,
    pub(crate) census: Census,
    pub(crate) complaints: Complaints,
}

impl Streams for Run {
    fn census(&mut self) -> &mut Census {
        &mut self.census
    }
    fn complaints(&mut self) -> &mut Complaints {
        &mut self.complaints
    }
}

pub(crate) const CHANNELS: [Bound<Run>; 6] = [
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

pub(crate) fn read(dir: &Path) -> Result<Run, clockwork_logs::LogError> {
    read_with(dir, &CHANNELS)
}

impl Run {
    pub(crate) fn ordered_samples(&self) -> Vec<&Logged<PoseSampleWire>> {
        let mut ordered: Vec<_> = self.samples.iter().collect();
        ordered.sort_by_key(|sample| sample.message.nominal_time().as_nanos());
        ordered
    }

    pub(crate) fn ordered_events(&self) -> Vec<&Logged<DriverEventWire>> {
        let mut ordered: Vec<_> = self.events.iter().collect();
        ordered.sort_by_key(|event| event.message.time().as_nanos());
        ordered
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Window {
    pub(crate) motion_id: u16,
    pub(crate) start_ns: i64,
    pub(crate) end_ns: i64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Span {
    pub(crate) start_ns: i64,
    pub(crate) end_ns: i64,
}

impl Window {
    pub(crate) fn span(self) -> Span {
        Span {
            start_ns: self.start_ns,
            end_ns: self.end_ns,
        }
    }
}

pub(crate) fn overlay_spans(windows: &[Window]) -> Vec<Span> {
    windows.iter().copied().map(Window::span).collect()
}

pub(crate) struct Prepared<'a> {
    pub(crate) ordered: Vec<&'a Logged<PoseSampleWire>>,
    pub(crate) events: Vec<&'a Logged<DriverEventWire>>,
    pub(crate) grid: Grid,
    pub(crate) skips: Skips<'a>,
    pub(crate) plant: GroupPlants,
    pub(crate) stream: Vec<(i64, [Residual; ROWS.len()])>,
}

pub(crate) fn prepare<'a>(run: &'a Run, config: &RunConfig) -> Result<Prepared<'a>, String> {
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

pub(crate) fn windows(run: &Run) -> Vec<Window> {
    let mut ordered: Vec<_> = run.schedules.iter().collect();
    ordered.sort_by_key(|schedule| schedule.at_ns);
    let mut planned: BTreeMap<(i64, u16), (i64, bool)> = BTreeMap::new();
    for schedule in ordered {
        let mut carried = BTreeSet::new();
        for window in schedule.message.overlays().iter() {
            let key = (window.start().as_nanos(), window.motion_id());
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
        for (key, held) in &mut planned {
            if held.1 && !carried.contains(key) {
                held.0 = held.0.min(schedule.at_ns);
                held.1 = false;
            }
        }
    }
    planned
        .into_iter()
        .map(|((start_ns, motion_id), (end_ns, _))| Window {
            motion_id,
            start_ns,
            end_ns,
        })
        .collect()
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub(crate) struct SettleMove {
    pub(crate) start_ns: i64,
    pub(crate) end_ns: i64,
    pub(crate) pose_id: u16,
    pub(crate) pace_ns: i64,
    pub(crate) measured: bool,
    pub(crate) reason: Option<String>,
}

#[derive(Clone, Debug)]
pub(crate) struct SettleResult {
    pub(crate) moves: Vec<SettleMove>,
    pub(crate) maxima: [Option<(f64, u16, &'static str, i64)>; 6],
}

/// The largest observed end-of-move residual over the committed library,
/// rounded up to a whole count. The residual is whatever a run leaves between
/// goal and present, including servo behavior, load, and possible head/body
/// contact. Because head/body interference is not modelled
/// (`TODO(head-body-interference)`), this is a run bound and not a measured
/// property of a servo. The envelope floor is derived to provide five times
/// this bound at the outer merge.
pub(crate) const SETTLE_BOUND_COUNTS: f64 = 18.0;

pub(crate) fn settle(
    run: &Run,
    ordered: &[&Logged<PoseSampleWire>],
    overlays: &[Window],
    config: &RunConfig,
    report: &mut Report,
) -> SettleResult {
    let mut keys = BTreeSet::new();
    for schedule in &run.schedules {
        for step in schedule.message.steps().iter() {
            if step.kind() == brenn_reachy__cogs__schedule_clk_rs::StepKindWire::BASE_POSTURE {
                keys.insert((
                    step.start().as_nanos(),
                    step.end().as_nanos(),
                    step.pose_id(),
                    step.pace().as_nanos(),
                ));
            }
        }
    }
    let legs = [
        JointRef::Leg0,
        JointRef::Leg1,
        JointRef::Leg2,
        JointRef::Leg3,
        JointRef::Leg4,
        JointRef::Leg5,
    ];
    let lag_ns = i64::from(config.profiles.legs.following_lag_us).saturating_mul(1_000);
    let mut maxima: [Option<(f64, u16, &'static str, i64)>; 6] = [None; 6];
    let mut moves = Vec::new();
    for (start, end, pose_id, pace_ns) in keys {
        let clean_end = overlays
            .iter()
            .filter(|window| window.start_ns < end && window.end_ns > start)
            .map(|window| window.start_ns)
            .min()
            .map_or(end, |at| end.min(at));
        let mut reason = None;
        if let Some(window) = overlays
            .iter()
            .find(|window| window.start_ns <= start && window.end_ns > start)
        {
            reason = Some(format!(
                "overlay motion {} begins at {} before base start {}",
                window.motion_id, window.start_ns, start
            ));
        }
        let mut endpoint = None;
        if reason.is_none() {
            for pair in ordered.windows(2) {
                let at = pair[1].message.nominal_time().as_nanos();
                let (Some(before), Some(after)) = (
                    commanded_rows(&pair[0].message),
                    commanded_rows(&pair[1].message),
                ) else {
                    continue;
                };
                let changed = legs
                    .iter()
                    .any(|joint| row(*joint).is_some_and(|index| before[index] != after[index]));
                if at >= start && at < clean_end && changed {
                    endpoint = Some(at);
                }
            }
            if endpoint.is_none() {
                reason = Some(format!("no commanded change in [{start}, {clean_end})"));
            }
        }
        let mut first: Option<&Logged<PoseSampleWire>> = None;
        let mut hold = None;
        if let Some(endpoint) = endpoint {
            first = ordered
                .iter()
                .find(|sample| {
                    let at = sample.message.nominal_time().as_nanos();
                    at >= endpoint.saturating_add(lag_ns) && at < clean_end
                })
                .copied();
            if first.is_none() {
                reason = Some(format!(
                    "hold shorter than the following lag: endpoint {}, first eligible {}, clean end {}",
                    endpoint,
                    endpoint.saturating_add(lag_ns),
                    clean_end
                ));
            } else {
                hold = Some(
                    ordered
                        .iter()
                        .rev()
                        .find(|sample| {
                            let at = sample.message.nominal_time().as_nanos();
                            at >= start && at < clean_end
                        })
                        .copied()
                        .expect("the endpoint-plus-lag sample itself is in the clean interval"),
                );
            }
        }
        let mut selected_rows = None;
        if reason.is_none() {
            let mut decoded = Vec::new();
            for (label, sample) in [
                ("endpoint", first.expect("endpoint sample")),
                ("hold-end", hold.expect("hold sample")),
            ] {
                let (Some(commanded), Some(present)) = (
                    commanded_rows(&sample.message),
                    present_rows(&sample.message),
                ) else {
                    reason = Some(format!(
                        "{label} reading at {} missing commanded or present rows before clean end {}",
                        sample.message.nominal_time().as_nanos(),
                        clean_end
                    ));
                    break;
                };
                decoded.push((
                    label,
                    sample.message.nominal_time().as_nanos(),
                    commanded,
                    present,
                ));
            }
            if reason.is_none() {
                selected_rows = Some(decoded);
            }
        }
        let measured = selected_rows.is_some();
        if measured {
            report.measured.push(format!(
                "base-move pose {pose_id} measured: endpoint {}, clean end {clean_end}",
                endpoint.expect("measured endpoint")
            ));
            for (label, instant, commanded, present) in selected_rows.expect("selected rows") {
                for (index, joint) in legs.iter().enumerate() {
                    if let Some(row) = row(*joint) {
                        let counts = (commanded[row] - present[row]).abs() / COUNT_RAD;
                        if maxima[index].is_none_or(|old| counts > old.0) {
                            maxima[index] = Some((counts, pose_id, label, instant));
                        }
                    }
                }
            }
        }
        moves.push(SettleMove {
            start_ns: start,
            end_ns: end,
            pose_id,
            pace_ns,
            measured,
            reason,
        });
    }
    for (index, joint) in legs.iter().enumerate() {
        match maxima[index] {
            Some((counts, pose, label, instant)) => {
                report.measured.push(format!(
                    "base-move settle {}: {counts:.1} counts (pose {pose}, {label} at {instant})",
                    reachy_motion::joints::Name(*joint)
                ));
                if counts > SETTLE_BOUND_COUNTS {
                    report.fail(format!("base-move settle exceeds {SETTLE_BOUND_COUNTS:.0} counts at {}: {counts:.1} counts (pose {pose}, {label} at {instant})", reachy_motion::joints::Name(*joint)));
                }
            }
            None => report.measured.push(format!(
                "base-move settle {}: unavailable",
                reachy_motion::joints::Name(*joint)
            )),
        }
    }
    let measured = moves.iter().filter(|item| item.measured).count();
    let skipped = moves.len() - measured;
    report.note(format!("{measured} measured, {skipped} skipped base moves"));
    for item in &moves {
        if let Some(reason) = item.reason.as_deref() {
            report.note(format!("base move pose {} skipped: {reason}", item.pose_id));
        }
    }
    SettleResult { moves, maxima }
}

pub(crate) fn whole_stream_measurements(
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
pub(crate) fn named_motion(by_id: &BTreeMap<u16, String>, motion_id: u16) -> String {
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
pub(crate) fn inside<'a>(
    ordered: &'a [&'a Logged<PoseSampleWire>],
    window: &Window,
) -> &'a [&'a Logged<PoseSampleWire>] {
    let at = |sample: &&Logged<PoseSampleWire>| sample.message.nominal_time().as_nanos();
    let lo = ordered.partition_point(|sample| at(sample) < window.start_ns);
    let hi = ordered.partition_point(|sample| at(sample) < window.end_ns);
    &ordered[lo..hi]
}

/// The driver events the sorted stream holds inside `window`.
pub(crate) fn events_inside<'a>(
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
pub(crate) fn every_window_moved(
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
pub(crate) struct Worst {
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
pub(crate) struct Nearest {
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
pub(crate) fn measurements(
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

pub(crate) fn window_measurements(
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
pub(crate) fn stillness(
    ordered: &[&Logged<PoseSampleWire>],
    standard: Standard,
    report: &mut Report,
) {
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
#[allow(dead_code)]
pub(crate) fn held_standard(table: &MotionTable) -> Standard {
    if table.probes_only() {
        Standard::JudgedWhereHeadStillRequired
    } else {
        Standard::JudgedWhereHeadStill
    }
}
/// The sample stream held for the whole of the tour.
///
/// The record is the point of the run, so a stretch of it the log does not hold
/// is a finding whatever else the run did. Judged between the first window
/// opening and the last one closing: what the driver did before the tour began
/// and after it ended is not the tour's.
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
pub(crate) fn the_stream_held(
    ordered: &[&Logged<PoseSampleWire>],
    planned: &[Span],
    grid: Grid,
    skips: &Skips<'_>,
    report: &mut Report,
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
    // The stretch the tour is judged over runs from the first window opening to
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
            "the sample stream has {gaps} gap(s) over the tour that no skipped-cycle report \
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
        "{} sample(s) between the first window opening and the last one closing",
        held.len()
    ));
}

#[cfg(test)]
mod settle_invariant_tests {
    use super::SETTLE_BOUND_COUNTS;
    use reachy_kin::baked;
    use reachy_motion::stillness::COUNT_RAD;

    #[test]
    fn default_outer_clearance_is_five_times_the_settle_bound() {
        let margin = reachy_kin::envelope::EnvelopeConfig::default().min_toggle_margin;
        let a = baked::CRANK_LEN;
        let r = baked::ROD_LEN;
        let rho = a + r - margin;
        let angle = ((a * a + rho * rho - r * r) / (2.0 * a * rho)).acos();
        assert!(angle >= 5.0 * SETTLE_BOUND_COUNTS * COUNT_RAD);
    }
}
