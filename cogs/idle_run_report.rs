//! What one idle run did, read off its fetched records.
//!
//! The tool an idle-loop run is judged by. The voice host dances the idle
//! playlist until it is stopped, replacing each clip with the next before the
//! outgoing one closes; `make motion-fetch` brings the records home and this
//! reads them. It judges what the loop controls and what the health rotation
//! observed: the sample stream must hold inside every engagement -- a maximal
//! run of windows closer together than the loop's stow margin, never across a
//! rest -- the session must record no fault but an obstruction, and no servo
//! the health rotation read may have latched an error byte beyond the
//! input-voltage bit or reached the stop temperature, read by the same pass the
//! tour report uses. `head_obstructed` and `antenna_obstructed` are the outcome
//! the loop is built to yield to, so they are printed with where they happened,
//! never judged. A lag scan that could not run is said so and judged not at
//! all: following lag is then unmeasured.
//!
//! Two things it prints for a person and judges not at all. The seam table:
//! for each window a replacing schedule cut short, the commanded step on the
//! samples either side of that schedule's log time beside the largest step
//! inside each clip it joins. It is diagnostic because the log records neither
//! the sample on which the mover first composed under the replacing schedule
//! nor the lag between a decided goal and the setpoint the driver holds, so any
//! automatic verdict over it would be a magnitude over a neighbourhood -- a
//! speed bound applied to content. A step on one sample that dwarfs both clips'
//! own peaks is the shape to look at; a run of steps at the clips' pace is the
//! cross-fade. And the story summary: scripts, motions played, sessions and how
//! each ended, every fault row with the motion playing, the parks, and the
//! session's script rows by sender.
//!
//! The story is rebuilt across the session's 64-row ring from every logged
//! message, the way the host's console follows it, so a run of a few minutes
//! is read whole rather than as its last 64 rows; rows the ring overran between
//! two messages are counted and every count is then said to be a floor.
//!
//! The session's script rows are split between the idle loop and any other
//! sender by the script ids the loop names on its own console lines
//! (`idle_opened`, `idle_replaced`, `idle_resumed`, `idle_refused`) in the host
//! console beside the records. An `idle_*` line lost to a console tear leaves
//! that one id counted as another sender's; where no console came back, the
//! split is said to be not in these records. Nothing here judges speech.

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__driver__health_clk_rs::HealthReportWire;
use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
use brenn_reachy__motion__faults_clk_rs::FaultKindWire;
use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, TimelineWire};
use log_read::{Bound, Census, Complaints, Logged, Streams, binding, each, read_with};
use motion_channels::REPORT_CHANNEL;
use pose_reading::{
    LagScan, RunConfig, TEMPERATURE_STOP_C, capabilities, capability, commanded_rows,
    health_summary, lag_scan, lag_scans, lags, residuals,
};
use reachy_edge::names::MotionTable;
use reachy_edge::{Story, row_says};
use reachy_host::IDLE_SCRIPT_KINDS;
use reachy_host::idle::STOW_MARGIN_MS;
use reachy_motion::arm::SERVO_IDS;
use reachy_motion::joints::{JointGroup, Name, ROWS, row};
use run_report::{
    HOST_LOG, OLOG_EXTENSION, Report, console_dir, recover, run_directories, verdict,
};
use serde_json::Value;

use motion_run_report::{
    Run, Seam, Window, inside, named_motion, overlay_spans, prepare, read, the_stream_held,
    window_measurements, windows_and_seams,
};

/// The session's whole story, followed across every message the log holds.
#[derive(Default)]
struct StoryRows {
    /// The edge's own follower, which diffs each message against what it has
    /// already handed out.
    follower: Story,
    /// Every row, oldest first, across every message.
    rows: Vec<TimelineEntryWire>,
    /// Rows the ring overran between two logged messages.
    lost: u64,
    /// Times the story went backwards, which says its teller restarted.
    restarts: usize,
}

impl StoryRows {
    /// One logged message of the story.
    fn take(&mut self, story: &TimelineWire) {
        let update = self.follower.follow(story);
        if update.restarted {
            self.restarts += 1;
        }
        self.lost += update.lost;
        self.rows.extend(update.rows);
    }
}

/// What the log's story channel told.
#[derive(Default)]
struct Told {
    story: StoryRows,
    census: Census,
    complaints: Complaints,
}

impl Streams for Told {
    fn census(&mut self) -> &mut Census {
        &mut self.census
    }
    fn complaints(&mut self) -> &mut Complaints {
        &mut self.complaints
    }
}

/// The story channel, read message by message in log order.
const STORY: [Bound<Told>; 1] = [Bound {
    name: REPORT_CHANNEL,
    check: binding::<TimelineWire>,
    route: |told, message| {
        let Told {
            story, complaints, ..
        } = told;
        each(message, complaints, |logged: Logged<TimelineWire>| {
            story.take(&logged.message);
        });
    },
}];

/// The host console beside the records, as far as this report reads it.
struct Console {
    /// The file it was read from.
    at: PathBuf,
    /// Every script id the idle loop named on one of its own lines.
    loop_ids: BTreeSet<u32>,
}

/// Why no console was read beside the records.
struct NoConsole {
    /// The file that was looked for.
    at: PathBuf,
    /// The read error, when the file was there but could not be read;
    /// `None` when nothing came back at that path.
    why: Option<String>,
}

impl Console {
    /// Read the host's console beside a fetched record directory, or say which
    /// file was looked for and, if it was there, why it could not be read.
    ///
    /// Read as bytes and converted lossily: the console is a shared file that
    /// anything the process or its libraries print lands in, and one stray
    /// byte must not discard the loop's lines around it.
    fn read(records: &Path) -> Result<Self, NoConsole> {
        let at = console_dir(records).join(HOST_LOG);
        match std::fs::read(&at) {
            Ok(bytes) => {
                let text = String::from_utf8_lossy(&bytes).into_owned();
                Ok(Self::of_text(at, &text))
            }
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => {
                Err(NoConsole { at, why: None })
            }
            Err(error) => Err(NoConsole {
                at,
                why: Some(error.to_string()),
            }),
        }
    }

    /// The ids the loop named, off the console's text.
    ///
    /// Every whole JSON object on a line is read, so a loop line glued behind a
    /// torn sentence still counts; a `null` id -- a refusal at the edge or a
    /// script that could not be built -- names nothing.
    fn of_text(at: PathBuf, text: &str) -> Self {
        let mut loop_ids = BTreeSet::new();
        for line in text.lines() {
            if line.trim().is_empty() {
                continue;
            }
            for value in recover(line, "{\"").values {
                let is_edge = value.get("stream").and_then(Value::as_str) == Some("edge");
                let is_loop = value
                    .get("kind")
                    .and_then(Value::as_str)
                    .is_some_and(|kind| IDLE_SCRIPT_KINDS.contains(&kind));
                if !(is_edge && is_loop) {
                    continue;
                }
                if let Some(id) = value
                    .get("script_id")
                    .and_then(Value::as_u64)
                    .and_then(|id| u32::try_from(id).ok())
                {
                    loop_ids.insert(id);
                }
            }
        }
        Self { at, loop_ids }
    }
}

/// The commanded step on row `r` at sample `j`: its setpoint less the one
/// before it, where both samples exist and both carry one.
fn step(ordered: &[&Logged<PoseSampleWire>], j: usize, r: usize) -> Option<f64> {
    let before = ordered.get(j.checked_sub(1)?)?;
    let after = ordered.get(j)?;
    Some(commanded_rows(&after.message)?[r] - commanded_rows(&before.message)?[r])
}

/// The largest commanded step per row between consecutive samples inside
/// `window`, both of which carry a setpoint.
fn peaks(ordered: &[&Logged<PoseSampleWire>], window: &Window) -> [Option<f64>; ROWS.len()] {
    let mut peak: [Option<f64>; ROWS.len()] = [None; ROWS.len()];
    for pair in inside(ordered, window).windows(2) {
        let (Some(before), Some(after)) = (
            commanded_rows(&pair[0].message),
            commanded_rows(&pair[1].message),
        ) else {
            continue;
        };
        for (index, held) in peak.iter_mut().enumerate() {
            let figure = (after[index] - before[index]).abs();
            if held.is_none_or(|old| figure > old) {
                *held = Some(figure);
            }
        }
    }
    peak
}

/// A signed step as the table prints it, or `-` where there is none.
fn signed(figure: Option<f64>) -> String {
    figure.map_or_else(|| "-".to_owned(), |figure| format!("{figure:+.4}"))
}

/// A peak as the table prints it, or `-` where there is none.
fn magnitude(figure: Option<f64>) -> String {
    figure.map_or_else(|| "-".to_owned(), |figure| format!("{figure:.4}"))
}

/// The seam table: printed for a person to read, never judged.
fn seam_table(
    ordered: &[&Logged<PoseSampleWire>],
    seams: &[Seam],
    by_id: &BTreeMap<u16, String>,
    report: &mut Report,
) {
    report.note(format!(
        "seam table: {} seam(s); per row, the commanded step on the two samples before each \
         replacing schedule's log time and the three from it on, beside the largest step inside \
         each clip it joins; printed for a person to read, never judged",
        seams.len()
    ));
    for (k, seam) in seams.iter().enumerate() {
        let k = k + 1;
        let out = named_motion(by_id, seam.outgoing.motion_id);
        let incoming = seam
            .incoming
            .map(|window| (window, named_motion(by_id, window.motion_id)));
        let joined = match &incoming {
            Some((window, name)) => format!("{name} [{} .. {}]", window.start_ns, window.end_ns),
            None => "no window: the replacing schedule plays nothing".to_owned(),
        };
        report.note(format!(
            "seam {k} at {}: {out} [{} .. {}] -> {joined}",
            seam.at_ns, seam.outgoing.start_ns, seam.outgoing.end_ns
        ));
        let i =
            ordered.partition_point(|sample| sample.message.nominal_time().as_nanos() < seam.at_ns);
        let around = [
            i.checked_sub(2),
            i.checked_sub(1),
            Some(i),
            i.checked_add(1),
            i.checked_add(2),
        ];
        let out_peaks = peaks(ordered, &seam.outgoing);
        let in_peaks = incoming.as_ref().map(|(window, _)| peaks(ordered, window));
        for joint in ROWS {
            let Some(r) = row(joint) else { continue };
            let steps: Vec<String> = around
                .iter()
                .map(|j| signed(j.and_then(|j| step(ordered, j, r))))
                .collect();
            let tail = match (&incoming, &in_peaks) {
                (Some((_, name)), Some(in_peaks)) => format!(
                    "peak {} inside {out}, {} inside {name}",
                    magnitude(out_peaks[r]),
                    magnitude(in_peaks[r])
                ),
                _ => format!("peak {} inside {out}", magnitude(out_peaks[r])),
            };
            report.note(format!(
                "seam {k} {}: {} {} | {} {} {} rad per period; {tail}",
                Name(joint),
                steps[0],
                steps[1],
                steps[2],
                steps[3],
                steps[4]
            ));
        }
    }
}

/// The run's windows grouped into engagements: maximal runs of windows each
/// opening less than the loop's stow margin after the group before it closed.
fn engagements(planned: &[Window]) -> Vec<Vec<Window>> {
    let margin_ns = i64::try_from(STOW_MARGIN_MS)
        .expect("five seconds fits")
        .saturating_mul(1_000_000);
    let mut sorted = planned.to_vec();
    sorted.sort_by_key(|window| window.start_ns);
    let mut groups: Vec<Vec<Window>> = Vec::new();
    let mut group_end = 0_i64;
    for window in sorted {
        match groups.last_mut() {
            Some(group) if window.start_ns.saturating_sub(group_end) < margin_ns => {
                group.push(window);
                if window.end_ns > group_end {
                    group_end = window.end_ns;
                }
            }
            _ => {
                group_end = window.end_ns;
                groups.push(vec![window]);
            }
        }
    }
    groups
}

/// The sample stream held inside every engagement, each judged on its own and
/// none across a rest.
fn gaps(prepared: &motion_run_report::Prepared<'_>, planned: &[Window], report: &mut Report) {
    let groups = engagements(planned);
    for (n, group) in groups.iter().enumerate() {
        the_stream_held(
            &prepared.ordered,
            &overlay_spans(group),
            prepared.grid,
            &prepared.skips,
            report,
            &format!("engagement {}", n + 1),
        );
    }
    report.note(format!(
        "{} engagement(s): maximal runs of windows under {STOW_MARGIN_MS} ms apart, each judged \
         for gaps on its own and none across a rest",
        groups.len()
    ));
}

/// Whether a fault kind is one the loop is built to yield to.
fn obstruction(kind: FaultKindWire) -> bool {
    kind == FaultKindWire::HEAD_OBSTRUCTED || kind == FaultKindWire::ANTENNA_OBSTRUCTED
}

/// Every fault the run recorded, with where it happened; any that is not an
/// obstruction fails.
///
/// Both witnesses are read, the story's row and the decision tick's own
/// report, so a story row the ring lost cannot hide a fault.
fn faults(
    run: &Run,
    rows: &[TimelineEntryWire],
    planned: &[Window],
    seams: &[Seam],
    by_id: &BTreeMap<u16, String>,
    report: &mut Report,
) {
    for entry in rows
        .iter()
        .filter(|entry| entry.kind() == ReportKindWire::FAULT_RECORDED)
    {
        let obstructed = u8::try_from(entry.a())
            .map(FaultKindWire)
            .is_ok_and(obstruction);
        let at = entry.time().as_nanos();
        let playing = match planned
            .iter()
            .find(|window| window.start_ns <= at && at < window.end_ns)
        {
            Some(window) => format!(
                "{} was playing, {:.0} ms into it",
                named_motion(by_id, window.motion_id),
                (at - window.start_ns) as f64 / 1e6
            ),
            None => "no window was open".to_owned(),
        };
        let nearest = match seams
            .iter()
            .enumerate()
            .min_by_key(|(_, seam)| seam.at_ns.abs_diff(at))
        {
            Some((k, seam)) => format!(
                "nearest seam {} ({:+.0} ms)",
                k + 1,
                (at - seam.at_ns) as f64 / 1e6
            ),
            None => "no seam in these records".to_owned(),
        };
        let says = row_says(entry);
        report.note(format!("fault at {at}: {says}; {playing}; {nearest}"));
        if !obstructed {
            report.fail(format!(
                "the session recorded a fault that is not an obstruction at {at}: {says}"
            ));
        }
    }
    let mut tally: BTreeMap<String, usize> = BTreeMap::new();
    for fault in &run.faults {
        let kind = fault.message.kind();
        if obstruction(kind) {
            *tally.entry(format!("{kind:?}")).or_default() += 1;
        } else {
            report.fail(format!(
                "the decision tick raised {kind:?} at {}, which is not an obstruction",
                fault.message.time().as_nanos()
            ));
        }
    }
    let raised: usize = tally.values().sum();
    if raised > 0 {
        report.note(format!(
            "the decision tick raised {raised} obstruction fault(s): {}",
            tally
                .iter()
                .map(|(kind, count)| format!("{kind} x{count}"))
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
}

/// How the story's sessions ended.
#[derive(Debug, Default, PartialEq)]
struct Sessions {
    /// Returned to rest after the session said it ended.
    clean: usize,
    /// Returned to rest with no ending said since the last phase row: a fault
    /// response.
    fault: usize,
    /// Went to `parked`.
    parked: usize,
    /// Still open when the log ends.
    open: usize,
}

/// Count the story's sessions by how each ended.
///
/// A session opens on `engaging` or `active` and closes on `resting` or
/// `parked`; a `resting` is a clean ending when the session said it ended
/// since the last phase row, and a fault response otherwise. The boot row into
/// `resting` opens nothing and closes nothing.
fn sessions(rows: &[TimelineEntryWire]) -> Sessions {
    let mut counted = Sessions::default();
    let mut open = false;
    let mut ended = false;
    for entry in rows {
        if entry.kind() == ReportKindWire::SESSION_ENDED {
            ended = true;
            continue;
        }
        if entry.kind() != ReportKindWire::PHASE_CHANGED {
            continue;
        }
        let to = SessionPhaseWire(u8::try_from(entry.a()).unwrap_or(u8::MAX));
        if (to == SessionPhaseWire::ENGAGING || to == SessionPhaseWire::ACTIVE) && !open {
            open = true;
        } else if to == SessionPhaseWire::RESTING && open {
            if ended {
                counted.clean += 1;
            } else {
                counted.fault += 1;
            }
            open = false;
        } else if to == SessionPhaseWire::PARKED {
            if open {
                counted.parked += 1;
            }
            open = false;
        }
        ended = false;
    }
    counted.open = usize::from(open);
    counted
}

/// One sender's script rows, by kind.
#[derive(Default)]
struct Tally {
    accepted: usize,
    replaced: usize,
    held: usize,
    refused: usize,
    /// Each refusal's sentence, and how many rows said it.
    says: BTreeMap<String, usize>,
}

impl Tally {
    /// One script row.
    fn take(&mut self, entry: &TimelineEntryWire) {
        let kind = entry.kind();
        if kind == ReportKindWire::SCRIPT_ACCEPTED {
            self.accepted += 1;
        } else if kind == ReportKindWire::SCRIPT_REPLACED {
            self.replaced += 1;
        } else if kind == ReportKindWire::SCRIPT_HELD {
            self.held += 1;
        } else if kind == ReportKindWire::SCRIPT_REFUSED {
            self.refused += 1;
            *self.says.entry(row_says(entry)).or_default() += 1;
        }
    }

    /// The counts, in one clause.
    fn said(&self) -> String {
        let mut said = format!(
            "{} accepted, {} replaced, {} held, {} refused",
            self.accepted, self.replaced, self.held, self.refused
        );
        if !self.says.is_empty() {
            said.push_str("; ");
            said.push_str(
                &self
                    .says
                    .iter()
                    .map(|(says, count)| format!("{says} x{count}"))
                    .collect::<Vec<_>>()
                    .join(", "),
            );
        }
        said
    }
}

/// Whether a row is one of the session's four script rows.
fn script_row(entry: &TimelineEntryWire) -> bool {
    [
        ReportKindWire::SCRIPT_ACCEPTED,
        ReportKindWire::SCRIPT_REPLACED,
        ReportKindWire::SCRIPT_HELD,
        ReportKindWire::SCRIPT_REFUSED,
    ]
    .contains(&entry.kind())
}

/// The story summary, for a person to read.
fn summary(
    run: &Run,
    story: &StoryRows,
    console: Result<&Console, &NoConsole>,
    planned: &[Window],
    by_id: &BTreeMap<u16, String>,
    report: &mut Report,
) {
    report.note(format!(
        "{} script(s) reached the session's port",
        run.scripts.len()
    ));
    if planned.is_empty() {
        report.note("no window opened");
    } else {
        let mut played: BTreeMap<String, usize> = BTreeMap::new();
        for window in planned {
            *played
                .entry(named_motion(by_id, window.motion_id))
                .or_default() += 1;
        }
        let mut ranked: Vec<(String, usize)> = played.into_iter().collect();
        ranked.sort_by(|a, b| b.1.cmp(&a.1).then_with(|| a.0.cmp(&b.0)));
        report.note(format!(
            "{} window(s) over {} motion(s): {}",
            planned.len(),
            ranked.len(),
            ranked
                .iter()
                .map(|(name, count)| format!("{name} x{count}"))
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    let counted = sessions(&story.rows);
    report.note(format!(
        "{} session(s): {} ended cleanly, {} on a fault, {} parked, {} still open at the log's end",
        counted.clean + counted.fault + counted.parked + counted.open,
        counted.clean,
        counted.fault,
        counted.parked,
        counted.open
    ));
    for entry in &story.rows {
        if entry.kind() == ReportKindWire::PHASE_CHANGED
            && u8::try_from(entry.a())
                .is_ok_and(|to| SessionPhaseWire(to) == SessionPhaseWire::PARKED)
        {
            report.note(format!(
                "parked: the machine parked at {}: {}",
                entry.time().as_nanos(),
                row_says(entry)
            ));
        }
        if entry.kind() == ReportKindWire::COMMISSION_FAILED {
            report.note(format!("parked: {}", row_says(entry)));
        }
    }
    let scripts = story.rows.iter().filter(|entry| script_row(entry));
    match console {
        Ok(console) => {
            let mut idle = Tally::default();
            let mut other = Tally::default();
            for entry in scripts {
                if console.loop_ids.contains(&entry.a()) {
                    idle.take(entry);
                } else {
                    other.take(entry);
                }
            }
            report.note(format!(
                "script rows split by the console at {}",
                console.at.display()
            ));
            report.note(format!(
                "the idle loop's script rows (ids its console lines named): {}",
                idle.said()
            ));
            report.note(format!("another sender's script rows: {}", other.said()));
        }
        Err(missing) => {
            let mut every = Tally::default();
            for entry in scripts {
                every.take(entry);
            }
            match &missing.why {
                None => report.note(format!(
                    "no console came back beside these records ({}), so which of the session's \
                     script rows are the idle loop's is not in these records; every sender's: {}",
                    missing.at.display(),
                    every.said()
                )),
                Some(why) => report.note(format!(
                    "the console at {} could not be read ({why}), so which of the session's \
                     script rows are the idle loop's is not in these records; every sender's: {}",
                    missing.at.display(),
                    every.said()
                )),
            }
        }
    }
    if story.lost > 0 || story.restarts > 0 {
        report.note(format!(
            "the story lost {} row(s) between logged messages and restarted {} time(s); every \
             count above is a floor",
            story.lost, story.restarts
        ));
    }
    if story.rows.is_empty() {
        report.note(
            "the log carries no story, so sessions, faults and script rows are not in these records",
        );
    }
}

/// What the report says when the lag scan could not run: which diagnostic is
/// missing and what cannot be concluded without it.
fn lag_scan_missing(error: &str) -> String {
    format!(
        "the lag scan could not run ({error}); following lag is unmeasured on this run. Nothing \
         here says whether the servos kept pace with their generators; the residual and lag \
         lines above are the only tracking evidence, and this run is no instrument for the \
         following-lag figure"
    )
}

/// Writes the lag scan, or says it is missing: a scan that cannot run is a
/// missing diagnostic, not a finding.
fn judge_lag_scan(scan: Result<[LagScan; JointGroup::ALL.len()], String>, report: &mut Report) {
    match scan {
        Ok(scans) => lag_scans(&scans, report),
        Err(error) => report.note(lag_scan_missing(&error)),
    }
}

/// Names every configured servo the health rotation never read. The verdict's
/// health clause holds over the servos read, so a servo with no reading is said
/// here rather than passed as healthy by silence.
fn unread_servos(readings: &[Logged<HealthReportWire>], report: &mut Report) {
    if readings.is_empty() {
        return;
    }
    let read: BTreeSet<u8> = readings.iter().map(|r| r.message.id()).collect();
    let unread: Vec<String> = SERVO_IDS
        .iter()
        .filter(|id| !read.contains(id))
        .map(|id| format!("servo {id}"))
        .collect();
    if !unread.is_empty() {
        report.note(format!(
            "the health rotation never read {}; nothing here says whether {} latched an error \
             or reached {TEMPERATURE_STOP_C} C",
            unread.join(", "),
            if unread.len() == 1 { "it" } else { "they" }
        ));
    }
}

/// Everything this tool has to say about one idle run.
fn analyze(
    run: &Run,
    told: &Told,
    console: Result<&Console, &NoConsole>,
    table: &MotionTable,
    config: &RunConfig,
) -> Report {
    let mut report = Report::default();
    for complaint in run.complaints.iter().chain(told.complaints.iter()) {
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
    let (planned, seams) = windows_and_seams(run);
    let prepared = match prepare(run, config) {
        Ok(prepared) => prepared,
        Err(error) => {
            report.fail(error);
            return report;
        }
    };
    if planned.is_empty() {
        report.note("no overlay window opened in these records, so nothing danced");
    }
    faults(run, &told.story.rows, &planned, &seams, &by_id, &mut report);
    gaps(&prepared, &planned, &mut report);
    seam_table(&prepared.ordered, &seams, &by_id, &mut report);
    summary(run, &told.story, console, &planned, &by_id, &mut report);
    window_measurements(&prepared, &planned, &by_id, &mut report);
    residuals(&prepared.stream, &run.samples, &prepared.plant, &mut report);
    lags(&run.samples, &mut report);
    let measured = capability(&run.samples, prepared.grid);
    capabilities(&measured, &mut report);
    judge_lag_scan(
        lag_scan(&run.samples, prepared.grid, &config.profiles, &measured),
        &mut report,
    );
    health_summary(&run.readings, &mut report);
    unread_servos(&run.readings, &mut report);
    report
}

fn main() -> ExitCode {
    const USAGE: &str = "usage: idle_run_report <records> <names.json>";
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [records, sidecar] = args.as_slice() else {
        eprintln!("{USAGE}");
        return ExitCode::FAILURE;
    };
    let records_dir = PathBuf::from(records);
    let (found, unlisted) = run_directories(&records_dir);
    let Some(run_dir) = found.last() else {
        if unlisted.is_empty() {
            eprintln!(
                "{records} holds no run directory with a non-empty `.{OLOG_EXTENSION}` in it"
            );
        } else {
            eprintln!(
                "{records} holds no run directory with a non-empty `.{OLOG_EXTENSION}` in it; \
                 could not list {}",
                unlisted.join("; ")
            );
        }
        return ExitCode::FAILURE;
    };
    // The configuration the run was performed under: a plain fetch leaves
    // `config/` at the root of the records, and the tour and script modes move
    // it into the run directory.
    let config_root = if records_dir.join("config").is_dir() {
        records_dir.as_path()
    } else {
        run_dir.as_path()
    };
    let config = match RunConfig::read(config_root) {
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
    let run = match read(run_dir) {
        Ok(run) => run,
        Err(err) => {
            eprintln!("reading the log under {}: {err}", run_dir.display());
            return ExitCode::FAILURE;
        }
    };
    let told = match read_with::<Told>(run_dir, &STORY) {
        Ok(told) => told,
        Err(err) => {
            eprintln!("reading the story under {}: {err}", run_dir.display());
            return ExitCode::FAILURE;
        }
    };
    let console = Console::read(&records_dir);
    let mut report = analyze(&run, &told, console.as_ref(), &table, &config);
    if found.len() > 1 {
        report.note(format!(
            "{records} holds {} run directories; this report reads the newest, {}",
            found.len(),
            run_dir.display()
        ));
    }
    for entry in &unlisted {
        report.note(format!("could not list {entry}"));
    }
    verdict(
        "idle_run_report",
        records,
        &report,
        "the sample stream held through every engagement, no fault but an obstruction was \
         recorded, and no servo the health rotation read latched an error or reached the stop \
         temperature",
    )
}

#[cfg(test)]
mod tests {
    //! What the analyzer says about crafted idle runs.
    //!
    //! Each case builds the streams an idle run writes rather than a log file;
    //! what is under test is the reading. The base run is two clips, the
    //! second replacing the first mid-play.

    use std::collections::BTreeSet;
    use std::path::PathBuf;

    use brenn_reachy__cogs__schedule_clk_rs::{OverlayWindowWire, SessionScheduleWire};
    use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
    use brenn_reachy__driver__health_clk_rs::HealthReportWire;
    use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
    use brenn_reachy__motion__faults_clk_rs::{FaultKindWire, TickFaultWire};
    use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKindWire};
    use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, TimelineWire};
    use clockwork_rs::SyncTime;
    use log_read::Logged;
    use motion_proto::PlayWindow;
    use motion_run_report::windows_and_seams;
    use pose_reading::{RunConfig, TEMPERATURE_STOP_C};
    use reachy_driver::NOMINAL_CYCLE_NS;
    use reachy_edge::names::{MotionEntry, MotionTable};
    use reachy_host::{IDLE_OPENED, IDLE_REFUSED, IDLE_REPLACED, UNSENT};
    use reachy_motion::arm::{DEFAULT_GAINS, SERVO_IDS};
    use reachy_motion::joints::{JointRef, ROW_COUNT, row, write_rows};
    use reachy_motion::plant::SHIPPED_PROFILES;
    use run_report::{Report, console_dir};
    use serde_json::json;

    use super::{Console, HOST_LOG, NoConsole, Run, Sessions, StoryRows, Told, analyze, sessions};

    /// An arbitrary instant a synthetic run starts at, chosen for being nothing
    /// round.
    const T0: i64 = 1_772_000_000_123_456_789;

    /// The cycle the base run's replacing schedule lands on.
    const SEAM: i64 = 20;

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

    /// One health reading of servo `id` at cycle `n`: `temp_c` degrees, the
    /// error byte `bits`, the rail at 7.4 V.
    fn reading(n: i64, id: u8, temp_c: i8, bits: u8) -> Logged<HealthReportWire> {
        let mut message = HealthReportWire::new();
        message.set_id(id);
        message.set_volts(7.4);
        message.set_temp_c(temp_c);
        message.set_bits(bits);
        message.set_sample_time(when(n));
        at(n, message)
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

    /// A schedule carrying no window, as the session republishes one on a stow.
    fn stow(n: i64) -> Logged<SessionScheduleWire> {
        let mut message = SessionScheduleWire::new();
        message.set_engaged(true);
        message.overlays_mut().clear();
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

    /// The body yaw walking a milliradian per cycle, read where it was told.
    fn heartbeat(cycles: i64) -> Vec<Logged<PoseSampleWire>> {
        (0..cycles)
            .map(|n| {
                let mut rows = [0.0; ROW_COUNT];
                rows[row(JointRef::BodyYaw).expect("a bus row")] = n as f64 * 1e-3;
                at(n, sample(n, &rows, &rows))
            })
            .collect()
    }

    /// The base run: a clip opened at cycle 0, and at cycle 20 a schedule
    /// carrying only its replacement.
    fn base() -> Run {
        Run {
            schedules: vec![schedule(0, 0, 1, 40), schedule(SEAM, 1, SEAM + 1, 60)],
            samples: heartbeat(70),
            ..Run::default()
        }
    }

    /// The configuration a crafted run is judged under: what this tree ships,
    /// with the detector armed.
    fn shipped() -> RunConfig {
        RunConfig::stated(SHIPPED_PROFILES, DEFAULT_GAINS, true)
    }

    /// One row of the session's story, at cycle `cycle`.
    fn row_at(kind: ReportKindWire, a: u32, b: u32, cycle: i64) -> TimelineEntryWire {
        let mut entry = TimelineEntryWire::new();
        entry.set_time(when(cycle));
        entry.set_kind(kind);
        entry.set_a(a);
        entry.set_b(b);
        entry
    }

    /// One phase change: `a` the phase entered, `b` the one left.
    fn phase(to: SessionPhaseWire, from: SessionPhaseWire, cycle: i64) -> TimelineEntryWire {
        row_at(
            ReportKindWire::PHASE_CHANGED,
            u32::from(to.0),
            u32::from(from.0),
            cycle,
        )
    }

    /// The boot row and the two phase changes a session that took the machine
    /// narrates first.
    fn engagement() -> Vec<TimelineEntryWire> {
        vec![
            phase(SessionPhaseWire::RESTING, SessionPhaseWire::STARTING, 0),
            phase(SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING, 0),
            phase(SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING, 0),
        ]
    }

    /// A fault row of `kind` on servo 11.
    fn fault_row(kind: FaultKindWire, cycle: i64) -> TimelineEntryWire {
        row_at(ReportKindWire::FAULT_RECORDED, u32::from(kind.0), 11, cycle)
    }

    /// One story message carrying `rows`, with `dropped` rows before them.
    fn message(rows: &[TimelineEntryWire], dropped: u32) -> TimelineWire {
        let mut story = TimelineWire::new();
        {
            let mut entries = story.entries_mut();
            for entry in rows {
                *entries
                    .try_grow()
                    .expect("a story of no more rows than the message holds") = entry.clone();
            }
        }
        story.set_dropped(dropped);
        story
    }

    /// What the story channel told, as one message of `rows`.
    fn told(rows: &[TimelineEntryWire]) -> Told {
        let mut told = Told::default();
        told.story.take(&message(rows, 0));
        told
    }

    /// A console holding `lines`.
    fn console(lines: &[String]) -> Console {
        Console::of_text(PathBuf::from("/c"), &lines.join("\n"))
    }

    /// One of the idle loop's own lines, built by the edge's builder.
    fn idle_line(kind: &str, script_id: Option<u32>) -> String {
        reachy_edge::edge_line_with(
            kind,
            when(0),
            "the loop",
            &[(
                "script_id",
                script_id.map_or(serde_json::Value::Null, serde_json::Value::from),
            )],
        )
    }

    /// A console naming script ids 1 and 2 as the loop's.
    fn loop_console() -> Console {
        console(&[
            reachy_edge::edge_line_with(IDLE_OPENED, when(0), "…", &[("script_id", json!(1))]),
            reachy_edge::edge_line_with(IDLE_REPLACED, when(0), "…", &[("script_id", json!(2))]),
        ])
    }

    /// A tick fault of `kind` at cycle `n`.
    fn tick_fault(kind: FaultKindWire, n: i64) -> Logged<TickFaultWire> {
        let mut fault = TickFaultWire::new();
        fault.set_kind(kind);
        fault.set_time(when(n));
        fault.set_count(1);
        at(n, fault)
    }

    /// Whether any finding says `what`.
    fn found(report: &Report, what: &str) -> bool {
        report.findings.iter().any(|line| line.contains(what))
    }

    /// Whether any measurement says `what`.
    fn measured(report: &Report, what: &str) -> bool {
        report.measured.iter().any(|line| line.contains(what))
    }

    /// The measured line that says every one of `what`.
    fn line_with<'a>(report: &'a Report, what: &[&str]) -> Option<&'a String> {
        report
            .measured
            .iter()
            .find(|line| what.iter().all(|part| line.contains(part)))
    }

    #[test]
    fn one_replacement_passes_and_its_seam_is_tabled() {
        let mut rows = engagement();
        rows.push(row_at(ReportKindWire::SCRIPT_ACCEPTED, 1, 4, 0));
        rows.push(row_at(ReportKindWire::SCRIPT_REPLACED, 2, 4, SEAM));
        let console = loop_console();
        let report = analyze(&base(), &told(&rows), Ok(&console), &table(), &shipped());
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(measured(&report, "seam 1 at"), "{:?}", report.measured);
        assert!(
            measured(&report, "-> pollen/emotions/curious1"),
            "{:?}",
            report.measured
        );
        let yaw = line_with(&report, &["seam 1 body yaw:"]).expect("the body yaw's seam line");
        assert!(yaw.contains("+0.0010"), "{yaw}");
        assert!(
            yaw.contains("peak 0.0010 inside pollen/dances/simple_nod"),
            "{yaw}"
        );
        assert!(
            yaw.contains("0.0010 inside pollen/emotions/curious1"),
            "{yaw}"
        );
    }

    #[test]
    fn a_one_sample_jump_at_a_seam_is_printed_and_judges_nothing() {
        let mut run = base();
        let antenna = row(JointRef::AntennaLeft).expect("a bus row");
        let index = usize::try_from(SEAM).expect("a small index");
        let jumped = &mut run.samples[index];
        let mut rows = [0.0; ROW_COUNT];
        rows[row(JointRef::BodyYaw).expect("a bus row")] = SEAM as f64 * 1e-3;
        rows[antenna] = 0.5;
        jumped.message = sample(SEAM, &rows, &rows);
        let report = analyze(
            &run,
            &told(&engagement()),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        let line = line_with(&report, &["seam 1 left antenna:"]).expect("the antenna's seam line");
        assert!(line.contains("+0.5000"), "{line}");
        assert!(line.contains("-0.5000"), "{line}");
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    #[test]
    fn a_healthy_run_with_an_obstruction_is_still_green() {
        for kind in [
            FaultKindWire::HEAD_OBSTRUCTED,
            FaultKindWire::ANTENNA_OBSTRUCTED,
        ] {
            let mut rows = engagement();
            rows.push(fault_row(kind, 10));
            let run = Run {
                faults: vec![tick_fault(kind, 10)],
                readings: SERVO_IDS
                    .iter()
                    .map(|&id| reading(3, id, 30, 0x01))
                    .collect(),
                ..base()
            };
            let report = analyze(
                &run,
                &told(&rows),
                Ok(&loop_console()),
                &table(),
                &shipped(),
            );
            assert!(report.findings.is_empty(), "{:?}", report.findings);
            let line = line_with(&report, &["fault at"]).expect("the fault's line");
            assert!(
                line.contains("pollen/dances/simple_nod was playing"),
                "{line}"
            );
            assert!(line.contains("ms into it"), "{line}");
            assert!(line.contains("nearest seam 1"), "{line}");
            assert!(
                measured(&report, "the decision tick raised 1 obstruction fault(s)"),
                "{:?}",
                report.measured
            );
            assert!(measured(&report, "servo 10:"), "{:?}", report.measured);
            assert!(measured(&report, "servo 18:"), "{:?}", report.measured);
            assert!(
                !measured(&report, "the health rotation reported nothing"),
                "{:?}",
                report.measured
            );
            assert!(!measured(&report, "never read"), "{:?}", report.measured);
        }
    }

    #[test]
    fn a_servo_the_rotation_never_read_is_named() {
        let run = Run {
            readings: SERVO_IDS
                .iter()
                .filter(|&&id| id != 14)
                .map(|&id| reading(3, id, 30, 0x01))
                .collect(),
            ..base()
        };
        let report = analyze(
            &run,
            &told(&engagement()),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        let line = line_with(&report, &["never read servo 14"]).expect("the unread servo's line");
        assert!(line.contains("nothing here says whether it"), "{line}");
        assert!(
            !measured(&report, "never read servo 10"),
            "{:?}",
            report.measured
        );
    }

    #[test]
    fn an_obstruction_before_the_opening_plays_is_listed_and_breaks_nothing() {
        let mut rows = engagement();
        rows.push(fault_row(FaultKindWire::HEAD_OBSTRUCTED, 45));
        let run = Run {
            schedules: vec![schedule(0, 0, 100, 140), stow(50)],
            samples: heartbeat(160),
            faults: vec![tick_fault(FaultKindWire::HEAD_OBSTRUCTED, 45)],
            ..Run::default()
        };
        let report = analyze(
            &run,
            &told(&rows),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        let line = line_with(&report, &["fault at"]).expect("the fault's line");
        assert!(line.contains("no window was open"), "{line}");
        assert!(
            measured(&report, "no overlay window opened"),
            "{:?}",
            report.measured
        );
        let (windows, seams) = windows_and_seams(&run);
        assert!(windows.is_empty(), "{windows:?}");
        assert!(seams.is_empty(), "{seams:?}");
    }

    #[test]
    fn a_replacement_that_plays_nothing_is_tabled_without_an_incoming_window() {
        let run = Run {
            schedules: vec![schedule(0, 0, 1, 40), stow(SEAM)],
            samples: heartbeat(70),
            ..Run::default()
        };
        let report = analyze(
            &run,
            &told(&engagement()),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        let seam = line_with(&report, &["seam 1 at"]).expect("the seam's line");
        assert!(
            seam.contains("-> no window: the replacing schedule plays nothing"),
            "{seam}"
        );
        let yaw = line_with(&report, &["seam 1 body yaw:"]).expect("the body yaw's seam line");
        assert!(
            yaw.contains("peak 0.0010 inside pollen/dances/simple_nod"),
            "{yaw}"
        );
        assert!(!yaw.contains("pollen/emotions/curious1"), "{yaw}");
        let seams = windows_and_seams(&run).1;
        assert_eq!(seams.len(), 1, "{seams:?}");
        assert_eq!(seams[0].incoming, None);
    }

    #[test]
    fn a_fault_that_is_not_an_obstruction_fails() {
        let mut rows = engagement();
        rows.push(fault_row(FaultKindWire::HEAD_SERVO_FAULT, 10));
        let report = analyze(
            &base(),
            &told(&rows),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert!(
            found(&report, "not an obstruction"),
            "{:?}",
            report.findings
        );
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);

        let run = Run {
            faults: vec![tick_fault(FaultKindWire::BUS_FAILURE, 10)],
            ..base()
        };
        let report = analyze(
            &run,
            &told(&engagement()),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert!(
            found(&report, "not an obstruction"),
            "{:?}",
            report.findings
        );
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
    }

    #[test]
    fn a_gap_inside_an_engagement_fails() {
        let mut run = base();
        run.samples.retain(|logged| {
            let cycle = (logged.message.nominal_time().as_nanos() - T0) / NOMINAL_CYCLE_NS;
            !(30..36).contains(&cycle)
        });
        let report = analyze(
            &run,
            &told(&engagement()),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert!(
            found(&report, "gap(s) over engagement 1"),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn a_rest_between_engagements_is_not_a_gap() {
        let later = 340;
        let samples = heartbeat(later + 50)
            .into_iter()
            .filter(|logged| {
                let cycle = (logged.message.nominal_time().as_nanos() - T0) / NOMINAL_CYCLE_NS;
                cycle < 45 || cycle >= later - 5
            })
            .collect();
        let run = Run {
            schedules: vec![
                schedule(0, 0, 1, 40),
                schedule(later - 1, 1, later, later + 40),
            ],
            samples,
            ..Run::default()
        };
        let report = analyze(
            &run,
            &told(&engagement()),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            measured(&report, "2 engagement(s)"),
            "{:?}",
            report.measured
        );
    }

    #[test]
    fn the_story_summary_counts_sessions_by_how_they_ended() {
        use SessionPhaseWire as P;
        let rows = vec![
            phase(P::RESTING, P::STARTING, 0),
            phase(P::ENGAGING, P::RESTING, 1),
            phase(P::ACTIVE, P::ENGAGING, 2),
            row_at(ReportKindWire::SESSION_ENDED, 0, 0, 3),
            phase(P::RESTING, P::ACTIVE, 4),
            phase(P::ENGAGING, P::RESTING, 5),
            phase(P::ACTIVE, P::ENGAGING, 6),
            fault_row(FaultKindWire::HEAD_OBSTRUCTED, 7),
            phase(P::RESTING, P::ACTIVE, 8),
            phase(P::ENGAGING, P::RESTING, 9),
            phase(P::PARKED, P::ENGAGING, 10),
            phase(P::ENGAGING, P::PARKED, 11),
            phase(P::ACTIVE, P::ENGAGING, 12),
        ];
        assert_eq!(
            sessions(&rows),
            Sessions {
                clean: 1,
                fault: 1,
                parked: 1,
                open: 1
            }
        );
        let report = analyze(
            &base(),
            &told(&rows),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert!(
            measured(
                &report,
                "4 session(s): 1 ended cleanly, 1 on a fault, 1 parked, 1 still open"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            measured(&report, "parked: the machine parked at"),
            "{:?}",
            report.measured
        );
    }

    #[test]
    fn the_story_is_read_whole_across_the_ring() {
        let ring = |from: u32| -> Vec<TimelineEntryWire> {
            (from..from + 64)
                .map(|n| row_at(ReportKindWire::SCHEDULE_PUBLISHED, n, 0, i64::from(n)))
                .collect()
        };
        let mut story = StoryRows::default();
        story.take(&message(&ring(0), 0));
        story.take(&message(&ring(10), 10));
        story.take(&message(&ring(100), 100));
        assert_eq!(story.rows.len(), 64 + 10 + 64);
        assert_eq!(story.lost, 26);
        assert_eq!(story.restarts, 0);

        let told = Told {
            story,
            ..Told::default()
        };
        let report = analyze(&base(), &told, Ok(&loop_console()), &table(), &shipped());
        assert!(
            measured(&report, "every count above is a floor"),
            "{:?}",
            report.measured
        );
    }

    #[test]
    fn script_rows_are_split_by_the_ids_the_loop_named() {
        let mut rows = engagement();
        rows.push(row_at(ReportKindWire::SCRIPT_ACCEPTED, 1, 4, 0));
        rows.push(row_at(ReportKindWire::SCRIPT_REPLACED, 2, 4, 10));
        rows.push(row_at(ReportKindWire::SCRIPT_REPLACED, 3, 4, SEAM));
        rows.push(row_at(
            ReportKindWire::SCRIPT_REFUSED,
            4,
            u32::from(RefusalReasonWire::FAULT_ENDING.0),
            30,
        ));
        let report = analyze(
            &base(),
            &told(&rows),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert!(
            measured(
                &report,
                "the idle loop's script rows (ids its console lines named): 1 accepted, 1 \
                 replaced, 0 held, 0 refused"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            measured(
                &report,
                "another sender's script rows: 0 accepted, 1 replaced, 0 held, 1 refused"
            ),
            "{:?}",
            report.measured
        );
    }

    #[test]
    fn without_a_console_the_split_is_not_in_these_records() {
        let mut rows = engagement();
        rows.push(row_at(ReportKindWire::SCRIPT_ACCEPTED, 1, 4, 0));
        rows.push(row_at(ReportKindWire::SCRIPT_REPLACED, 2, 4, SEAM));
        let report = analyze(
            &base(),
            &told(&rows),
            Err(&NoConsole {
                at: PathBuf::from("/r"),
                why: None,
            }),
            &table(),
            &shipped(),
        );
        let line =
            line_with(&report, &["is not in these records"]).expect("the no-console summary");
        assert!(line.contains("(/r)"), "{line}");
        assert!(
            line.contains("every sender's: 1 accepted, 1 replaced, 0 held, 0 refused"),
            "{line}"
        );
    }

    #[test]
    fn an_unreadable_console_is_said_with_its_error() {
        let mut rows = engagement();
        rows.push(row_at(ReportKindWire::SCRIPT_ACCEPTED, 1, 4, 0));
        rows.push(row_at(ReportKindWire::SCRIPT_REPLACED, 2, 4, SEAM));
        let report = analyze(
            &base(),
            &told(&rows),
            Err(&NoConsole {
                at: PathBuf::from("/r"),
                why: Some("permission denied".to_owned()),
            }),
            &table(),
            &shipped(),
        );
        let line =
            line_with(&report, &["could not be read"]).expect("the unreadable-console summary");
        assert!(
            line.contains("the console at /r could not be read (permission denied)"),
            "{line}"
        );
        assert!(
            line.contains("every sender's: 1 accepted, 1 replaced, 0 held, 0 refused"),
            "{line}"
        );
        assert!(
            line_with(&report, &["no console came back"]).is_none(),
            "{:?}",
            report.measured
        );
    }

    #[test]
    fn the_console_reader_tells_absent_from_unreadable() {
        let root =
            std::env::temp_dir().join(format!("idle_run_report_console_{}", std::process::id()));
        std::fs::create_dir_all(&root).expect("a scratch directory");
        let records = root.join("records");
        let Err(absent) = Console::read(&records) else {
            panic!("a console was read where none exists");
        };
        assert!(absent.why.is_none(), "{:?}", absent.why);
        assert!(absent.at.ends_with(HOST_LOG), "{}", absent.at.display());
        std::fs::create_dir_all(console_dir(&records).join(HOST_LOG))
            .expect("a directory where the console belongs");
        let Err(unreadable) = Console::read(&records) else {
            panic!("a directory was read as a console");
        };
        assert!(unreadable.why.is_some(), "{}", unreadable.at.display());
        let _ = std::fs::remove_dir_all(&root);
    }

    #[test]
    fn the_console_reader_takes_ids_from_the_loop_s_lines_only() {
        let mut timeline = TimelineEntryWire::new();
        timeline.set_kind(ReportKindWire::SCRIPT_ACCEPTED);
        timeline.set_a(9);
        let read = console(&[
            idle_line(IDLE_OPENED, Some(1)),
            format!("half a sentence{}", idle_line(IDLE_REPLACED, Some(2))),
            idle_line(IDLE_REFUSED, None),
            reachy_edge::edge_line_with(UNSENT, when(0), "lost", &[("script_id", json!(7))]),
            reachy_edge::timeline_line(&timeline),
            "not json at all".to_owned(),
        ]);
        assert_eq!(read.loop_ids, BTreeSet::from([1, 2]));
    }

    #[test]
    fn a_log_with_no_samples_is_refused_at_once() {
        let report = analyze(
            &Run::default(),
            &Told::default(),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(found(&report, "no driver samples"), "{:?}", report.findings);
    }

    #[test]
    fn a_servo_at_the_stop_temperature_fails_the_idle_report() {
        let run = Run {
            readings: vec![reading(3, 18, TEMPERATURE_STOP_C, 0)],
            ..base()
        };
        let report = analyze(
            &run,
            &told(&engagement()),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert!(
            found(
                &report,
                &format!("reached {TEMPERATURE_STOP_C} C, which no healthy tour")
            ),
            "{:?}",
            report.findings
        );
        assert!(found(&report, "servo 18"), "{:?}", report.findings);
        assert!(
            !report
                .findings
                .iter()
                .chain(report.measured.iter())
                .any(|line| line.contains("not judged by this report")),
            "{:?} {:?}",
            report.findings,
            report.measured
        );
    }

    #[test]
    fn a_latched_error_byte_fails_the_idle_report() {
        let run = Run {
            readings: vec![reading(3, 11, 30, 0x01), reading(6, 11, 30, 0x21)],
            ..base()
        };
        let report = analyze(
            &run,
            &told(&engagement()),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert!(
            found(&report, "latched an error byte"),
            "{:?}",
            report.findings
        );
        assert!(found(&report, "servo 11 (0x21)"), "{:?}", report.findings);
    }

    #[test]
    fn the_voltage_bit_alone_does_not_fail_the_idle_report() {
        let run = Run {
            readings: vec![reading(3, 11, 30, 0x01)],
            ..base()
        };
        let report = analyze(
            &run,
            &told(&engagement()),
            Ok(&loop_console()),
            &table(),
            &shipped(),
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            measured(
                &report,
                "the input-voltage bit is latched on 1 of the servos"
            ),
            "{:?}",
            report.measured
        );
    }

    /// Drives the scan's `Err` arm directly: `analyze` refuses every
    /// unscannable configuration before the scan runs, so it cannot reach it.
    #[test]
    fn a_lag_scan_that_cannot_run_is_a_missing_diagnostic_not_a_finding() {
        let mut report = Report::default();
        super::judge_lag_scan(Err("a profile with zero velocity".to_string()), &mut report);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        for part in [
            "could not run",
            "unmeasured",
            "no instrument",
            "a profile with zero velocity",
        ] {
            assert!(
                line_with(&report, &[part]).is_some(),
                "{:?}",
                report.measured
            );
        }
    }
}
