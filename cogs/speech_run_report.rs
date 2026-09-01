//! What one supervised speech run did, read off the host's console.
//!
//! The tool a speech run is judged by, beside `first_motion_report` and on the
//! same premise: it reads the fetched records and nothing else, so a run that
//! happened on a unit last week is read the same way as one that finished a
//! second ago. What differs is the input and the standard.
//!
//! The input is the console side of the fetch rather than the channel log. A
//! speech run's evidence is the voice host's own stdout — the edge's narration,
//! the session's story as the edge renders it, the alerts the edge's table
//! raised, and, where the site sends its pipeline's JSONL to stdout, the
//! pipeline's own events riding the same file. That file is
//! `<records>.console/voice_host_0.log`, written by the launcher on the unit
//! and pulled back beside the records.
//!
//! The standard is deliberately permissive, because what a human said to the
//! robot decides what the log contains. A run where nobody spoke and a run
//! where somebody woke the robot four times are both fine runs; neither is
//! something this tool can hold an opinion about. What it can hold an opinion
//! about is whether the run was the production pipeline at all — whether the
//! host started, whether it composed a voice half rather than coming up deaf,
//! whether anything critical fired, and whether it ended by draining or by
//! refusing its own configuration. Everything else is measured and printed and
//! never fails: counts per kind with the sentence each kind said, the warning
//! alerts, the alerts that never reached the bus, the bodies the edge dropped.
//!
//! Kinds, streams and alert severities this build has no word for are carried
//! as-is rather than guessed at, which is the same promise the narrator makes:
//! a newer host's
//! vocabulary is a number in the output, not a finding. Lines that are not JSON
//! at all are counted as noise with a bounded sample, because the console is
//! shared — anything the process or its libraries write to stderr lands in the
//! same file.

use std::collections::BTreeMap;
use std::io::{BufRead, BufReader};
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use motion_proto::DecodeError;
use reachy_edge::{CompileError, Refusal, Severity, UNKNOWN_KIND_PREFIX, severity_word};
use reachy_host::{
    AWAITING_SPEECH_CONFIG, COMPOSED, REFUSAL_PREFIX, STARTED, UNPUBLISHED, VOICELESS,
};
use run_report::{Report, verdict};
use serde_json::Value;

/// The launcher's name for the voice host's console.
///
/// The app name from the production launcher config with the launcher's own
/// instance suffix. A run whose config renames the app writes somewhere else,
/// and this tool says it found nothing rather than guessing at a neighbour.
const HOST_LOG: &str = "voice_host_0.log";

/// The launcher's name for the pod's console.
///
/// Not read, only looked for: whether the pod ran at all is the first thing an
/// operator asks about a run the robot seemed deaf in, and the pipeline's own
/// half of that story is in a file this tool has no schema for.
const POD_LOG: &str = "pod_0.log";

/// What the fetch names the console directory beside a record directory.
const CONSOLE_SUFFIX: &str = ".console";

/// How many non-JSON lines are quoted in the summary.
const NOISE_SAMPLE: usize = 3;

/// How much of a line is quoted where one is quoted.
const QUOTE_LIMIT: usize = 160;

/// One line of the host's console, as much as this tool understands of it.
#[derive(Debug, PartialEq)]
enum Line {
    /// A line on the edge's own stream.
    Edge { kind: String, says: String },
    /// A row of the session's story, as the edge rendered it.
    Timeline { kind: String, says: String },
    /// An alert the edge's table raised.
    Alert {
        severity: String,
        title: String,
        says: String,
    },
    /// A JSON object naming a stream this build has no word for.
    Foreign { stream: String },
    /// A JSON object with no stream: the pipeline's own event, whose schema
    /// belongs to `speech-surface` and is summarized by its keys.
    Pipeline { keys: String },
    /// Anything that is not a JSON object at all.
    Noise { text: String },
}

/// Everything the console said, folded down as it was read.
///
/// A fold rather than a `Vec<Line>`: the recommended site setting sends the
/// whole pipeline's event stream into this one file, and the mode it judges is
/// unbounded in duration — a speech run ends when the operator ends it. Holding
/// the file and a structured copy of it would make the longest sessions, which
/// are the ones most worth judging, the ones the report cannot run over. Every
/// pass the analysis wants is an independent fold, so none of them needs the
/// lines to still exist.
///
/// What is kept in full is what the report prints in full anyway — the alerts
/// and the lines that never reached the bus — so nothing here grows that the
/// output would not.
#[derive(Default)]
struct Console {
    /// Where the file was, for the messages.
    at: PathBuf,
    /// Whether the pod's console came back beside it.
    pod: bool,
    /// How many non-empty lines there were at all.
    seen: usize,
    /// Per edge kind: how many, and the last sentence it said.
    edges: BTreeMap<String, (usize, String)>,
    /// The same, per timeline kind.
    rows: BTreeMap<String, (usize, String)>,
    /// Every critical alert, as title and sentence.
    critical: Vec<(String, String)>,
    /// Every warning alert, as title and sentence.
    warning: Vec<(String, String)>,
    /// Alerts whose loudness this build has no word for, counted by the word
    /// they carried.
    ///
    /// The edge spells two, and both of them are taken from the edge itself
    /// below rather than restated, so a rename is a compile error here. A third
    /// word is a newer host's, and it is carried as-is for the same reason an
    /// unknown kind is: an alert nobody counted is one nobody read.
    unworded: BTreeMap<String, usize>,
    /// What each `unpublished` line said.
    unpublished: Vec<String>,
    /// Streams this build has no reader for, counted by name.
    foreign: BTreeMap<String, usize>,
    /// The pipeline's own events, counted by their top-level key list.
    pipeline: BTreeMap<String, usize>,
    /// How many lines were not JSON at all.
    noise: usize,
    /// The first few of those, quoted.
    noise_sample: Vec<String>,
    /// The host's refusal message, if the console ends in one.
    ///
    /// Set by a line carrying the refusal prefix and cleared by any JSON line
    /// after it, rather than read off the last line: the binary's argument
    /// refusal prints its usage text after the message, so the last line of a
    /// console that ends that way is not the message at all.
    refused: Option<String>,
    /// Whether any line held bytes that are not UTF-8.
    lossy: bool,
}

impl Console {
    /// Read the console beside a fetched record directory.
    ///
    /// An unreadable host log is not an error of this tool's: it is the first
    /// finding, so a fetch that came back without one still prints a report.
    ///
    /// Read as bytes and converted per line rather than as a string: the console
    /// is a shared file that anything the process or its libraries print lands
    /// in, transcribed speech and third-party audio chatter included, and one
    /// stray byte must not discard the whole evidence of a session that cannot
    /// be repeated. What a replacement character costs is one unreadable word,
    /// and that it happened is said in the measured half.
    fn read(records: &Path) -> Self {
        let dir = console_dir(records);
        let at = dir.join(HOST_LOG);
        let pod = dir.join(POD_LOG).is_file();
        let mut console = Self {
            at,
            pod,
            ..Self::default()
        };
        let Ok(file) = std::fs::File::open(&console.at) else {
            return console;
        };
        let mut reader = BufReader::new(file);
        let mut raw: Vec<u8> = Vec::new();
        while matches!(reader.read_until(b'\n', &mut raw), Ok(read) if read > 0) {
            let text = match String::from_utf8_lossy(&raw) {
                std::borrow::Cow::Borrowed(text) => text.to_owned(),
                std::borrow::Cow::Owned(text) => {
                    console.lossy = true;
                    text
                }
            };
            raw.clear();
            let text = text.trim_end_matches(['\n', '\r']);
            if text.trim().is_empty() {
                continue;
            }
            console.absorb(text);
        }
        console
    }

    /// One line, into the counters it belongs to.
    fn absorb(&mut self, raw: &str) {
        self.seen += 1;
        if let Some(message) = raw.strip_prefix(REFUSAL_PREFIX) {
            self.refused = Some(message.to_owned());
        }
        match classify(raw) {
            Line::Edge { kind, says } => {
                if kind == UNPUBLISHED {
                    self.unpublished.push(says.clone());
                }
                count(&mut self.edges, kind, says);
            }
            Line::Timeline { kind, says } => count(&mut self.rows, kind, says),
            Line::Alert {
                severity,
                title,
                says,
            } => {
                if severity == severity_word(Severity::Critical) {
                    self.critical.push((title, says));
                } else if severity == severity_word(Severity::Warning) {
                    self.warning.push((title, says));
                } else {
                    *self.unworded.entry(severity).or_default() += 1;
                }
            }
            Line::Foreign { stream } => *self.foreign.entry(stream).or_default() += 1,
            Line::Pipeline { keys } => *self.pipeline.entry(keys).or_default() += 1,
            Line::Noise { text } => {
                self.noise += 1;
                if self.noise_sample.len() < NOISE_SAMPLE {
                    self.noise_sample.push(text);
                }
                return;
            }
        }
        // A line of the stream after a refusal means the process went on: the
        // refusal was something a run printed, not the way it ended.
        self.refused = None;
    }

    /// Whether the edge said a line of this kind.
    fn said(&self, kind: &str) -> bool {
        self.edges.contains_key(kind)
    }
}

/// One more of a kind, and the sentence it said this time.
///
/// The last sentence of a kind rather than the first: where a kind repeats, the
/// numbers in it move, and what an operator wants beside the count is where it
/// got to.
fn count(table: &mut BTreeMap<String, (usize, String)>, kind: String, says: String) {
    let seen = table.entry(kind).or_insert((0, String::new()));
    seen.0 += 1;
    seen.1 = says;
}

/// The console directory the fetch wrote beside `records`.
///
/// Built from the record path's own spelling rather than from a sibling scan:
/// the pair is named by construction, and a directory holding two runs would
/// otherwise be ambiguous.
fn console_dir(records: &Path) -> PathBuf {
    let mut name = records
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    name.push_str(CONSOLE_SUFFIX);
    records.with_file_name(name)
}

/// What one raw line is.
fn classify(raw: &str) -> Line {
    let Ok(Value::Object(object)) = serde_json::from_str::<Value>(raw) else {
        return Line::Noise { text: quote(raw) };
    };
    let text = |key: &str| -> String {
        object
            .get(key)
            .and_then(Value::as_str)
            .unwrap_or_default()
            .to_owned()
    };
    match object.get("stream").and_then(Value::as_str) {
        Some("edge") => Line::Edge {
            kind: text("kind"),
            says: text("says"),
        },
        Some("timeline") => Line::Timeline {
            kind: text("kind"),
            says: text("says"),
        },
        Some("alert") => Line::Alert {
            severity: text("severity"),
            title: text("title"),
            says: text("says"),
        },
        Some(other) => Line::Foreign {
            stream: other.to_owned(),
        },
        None => Line::Pipeline {
            keys: object
                .keys()
                .map(String::as_str)
                .collect::<Vec<_>>()
                .join(","),
        },
    }
}

/// A line's text, bounded and stripped of control characters.
///
/// The console is a shared file and the text in it is not all this tree's: a
/// pipeline event quoting what somebody said reaches the terminal through here.
fn quote(text: &str) -> String {
    let mut clean: String = text
        .chars()
        .take(QUOTE_LIMIT)
        .map(|c| if c.is_control() { ' ' } else { c })
        .collect();
    if text.chars().count() > QUOTE_LIMIT {
        clean.push('…');
    }
    clean
}

/// The words the edge spells a dropped body with.
///
/// Taken from the screens themselves rather than restated, so a screen the edge
/// grows is counted as a refusal here without an edit.
fn refusal_kinds() -> Vec<&'static str> {
    vec![
        Refusal::Oversize { bytes: 0, cap: 0 }.kind(),
        Refusal::NotText.kind(),
        Refusal::Undecodable(DecodeError::NotJson {
            detail: String::new(),
        })
        .kind(),
        Refusal::ForeignPod {
            addressed: String::new(),
            pod: String::new(),
        }
        .kind(),
        Refusal::Stale {
            seq: 0,
            accepted: 0,
        }
        .kind(),
        Refusal::Uncompilable(CompileError::NoPosture).kind(),
    ]
}

/// Did the pipeline come up at all: the failures, in the order they matter.
fn came_up(console: &Console, report: &mut Report) {
    if console.seen == 0 {
        report.fail(format!(
            "{} is missing or empty: the voice host wrote no console, so either the launcher \
             never started it or the fetch came back without it",
            console.at.display()
        ));
        return;
    }
    if !console.said(STARTED) {
        report.fail(format!(
            "no `{STARTED}` line in {}: nothing in this console is the voice host announcing \
             itself",
            console.at.display()
        ));
    }
    if console.said(VOICELESS) {
        report.fail(
            "the host ran its edge half alone: no speech configuration was named, so this run \
             had no wake word, no endpointer and no pipeline. `make speech-run` stages one; a \
             payload built without REACHY_SPEECH_CONFIG produces exactly this",
        );
    } else if console.said(AWAITING_SPEECH_CONFIG) {
        report.fail(
            "the host was told where its speech configuration is and did not find it on the \
             unit: the payload carried no `host/speech.toml`, so this run had no voice half",
        );
    } else if !console.said(COMPOSED) {
        report.fail(format!(
            "no `{COMPOSED}` line: the voice pipeline never came up, so whatever else this \
             console holds, it is not a run of the production pipeline"
        ));
    }
}

/// Anything critical is a finding, however the run otherwise went.
fn criticals(console: &Console, report: &mut Report) {
    for (title, says) in &console.critical {
        report.fail(format!("a critical alert fired: `{title}` — {says}"));
    }
}

/// How the console ends: a drain, or the host refusing on its way out.
fn ended(console: &Console, report: &mut Report) {
    if let Some(message) = &console.refused {
        report.fail(format!(
            "the console ends with the host refusing rather than draining: {}",
            quote(message)
        ));
    }
}

/// Counts per kind, with the sentence each kind said.
fn kinds(console: &Console, report: &mut Report) {
    for (kind, (count, says)) in &console.edges {
        report.note(format!("edge {kind} ×{count}: {says}"));
    }
    for (kind, (count, says)) in &console.rows {
        report.note(format!("timeline {kind} ×{count}: {says}"));
    }
    let refusals = refusal_kinds();
    let dropped: usize = console
        .edges
        .iter()
        .filter(|(kind, _)| refusals.contains(&kind.as_str()))
        .map(|(_, (count, _))| count)
        .sum();
    report.note(format!(
        "{dropped} body(ies) the edge dropped before they reached the session"
    ));
    let unnamed: Vec<&str> = console
        .edges
        .keys()
        .chain(console.rows.keys())
        .map(String::as_str)
        .filter(|kind| kind.starts_with(UNKNOWN_KIND_PREFIX))
        .collect();
    if !unnamed.is_empty() {
        report.note(format!(
            "{} kind(s) this build has no word for, carried as their numbers: {}",
            unnamed.len(),
            unnamed.join(", ")
        ));
    }
}

/// The alerts that did not fail the run, and the ones that never travelled.
fn alerts(console: &Console, report: &mut Report) {
    report.note(format!("{} warning alert(s)", console.warning.len()));
    for (title, says) in &console.warning {
        report.note(format!("  warning `{title}` — {says}"));
    }
    // An alert that did not travel is worth a human's eye and not a failure:
    // the condition it carried is in the line above it, and a critical among
    // them already fails on its own line.
    report.note(format!(
        "{} alert(s) raised that nothing carried to the bus",
        console.unpublished.len()
    ));
    for says in &console.unpublished {
        report.note(format!("  {UNPUBLISHED} — {says}"));
    }
    for (severity, count) in &console.unworded {
        report.note(format!(
            "×{count} alert(s) at severity `{severity}`, which this build has no word for"
        ));
    }
}

/// What else was in the file: foreign streams, the pipeline's own events, noise.
fn the_rest(console: &Console, report: &mut Report) {
    for (stream, count) in &console.foreign {
        report.note(format!(
            "×{count} on stream `{stream}`, which this build has no reader for"
        ));
    }
    let events: usize = console.pipeline.values().sum();
    if events > 0 {
        report.note(format!(
            "{events} pipeline event(s) on this console, in {} shape(s)",
            console.pipeline.len()
        ));
        for (keys, count) in &console.pipeline {
            report.note(format!("  ×{count} carrying {keys}"));
        }
    }
    report.note(format!("{} line(s) that are not JSON", console.noise));
    for text in &console.noise_sample {
        report.note(format!("  not JSON: {text}"));
    }
    if console.lossy {
        report.note(
            "some of this console is not UTF-8 and was read with replacement characters: the \
             file is shared, and a library's stray bytes cost a word rather than the session",
        );
    }
    report.note(if console.pod {
        format!("{POD_LOG} came back with the fetch")
    } else {
        format!("no {POD_LOG} beside it: the pod's own console is not in these records")
    });
}

/// The whole reading of one console.
fn analyze(console: &Console) -> Report {
    let mut report = Report::default();
    came_up(console, &mut report);
    criticals(console, &mut report);
    ended(console, &mut report);
    kinds(console, &mut report);
    alerts(console, &mut report);
    the_rest(console, &mut report);
    report
}

/// Read the records named on the command line, judge them, print both halves.
///
/// The measurements go to stdout and the findings to stderr, as with the motion
/// report: the numbers file with the run record and the findings are what an
/// operator sees on the terminal. The exit status is the verdict.
fn main() -> ExitCode {
    const USAGE: &str = "usage: speech_run_report <records>";
    let mut args = std::env::args().skip(1);
    let Some(records) = args.next().filter(|word| !word.starts_with("--")) else {
        eprintln!("{USAGE}");
        return ExitCode::FAILURE;
    };
    if args.next().is_some() {
        eprintln!("{USAGE}");
        return ExitCode::FAILURE;
    }
    let console = Console::read(Path::new(&records));
    let report = analyze(&console);
    verdict(
        "speech_run_report",
        &records,
        &report,
        "the pipeline came up, composed, and drained whole",
    )
}

#[cfg(test)]
mod tests {
    use std::path::PathBuf;

    use reachy_scratch::{Scratch, scratch_dir};

    use super::{Console, HOST_LOG, Line, POD_LOG, analyze, classify, console_dir, refusal_kinds};

    /// A console with these lines in it, and the records path to read it by.
    ///
    /// The scratch guard comes back with the path: it removes the directory
    /// when it drops, so a case that let it go would read a console that is no
    /// longer there.
    fn records(name: &str, lines: &[&str]) -> (Scratch, PathBuf) {
        let dir = scratch_dir(name);
        let records = dir.join("speech-log-20260831T000000Z");
        let console = dir.join("speech-log-20260831T000000Z.console");
        std::fs::create_dir_all(&console).expect("a console directory");
        std::fs::write(console.join(HOST_LOG), lines.join("\n") + "\n")
            .expect("the host's console");
        (dir, records)
    }

    /// A console written as raw bytes, for the cases about what is not text.
    fn byte_records(name: &str, bytes: &[u8]) -> (Scratch, PathBuf) {
        let dir = scratch_dir(name);
        let records = dir.join("speech-log-20260831T000000Z");
        let console = dir.join("speech-log-20260831T000000Z.console");
        std::fs::create_dir_all(&console).expect("a console directory");
        std::fs::write(console.join(HOST_LOG), bytes).expect("the host's console");
        (dir, records)
    }

    /// The two lines every good run has.
    const STARTED: &str = r#"{"stream":"edge","at_ns":1,"kind":"started","says":"the voice host answers for `reachy00`"}"#;
    const COMPOSED: &str = r#"{"stream":"edge","at_ns":2,"kind":"composed","says":"the voice pipeline is running","alerts":true}"#;

    #[test]
    fn a_run_that_came_up_and_composed_is_a_success() {
        let (_dir, at) = records("speech-report-clean", &[STARTED, COMPOSED]);
        let report = analyze(&Console::read(&at));
        assert!(
            report.findings.is_empty(),
            "a run that came up is not a finding: {:?}",
            report.findings
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("edge started ×1")),
            "{:?}",
            report.measured
        );
    }

    /// The whole run's evidence is one file, and a fetch that came back without
    /// it is the finding rather than a crash.
    #[test]
    fn a_missing_host_console_is_the_first_finding() {
        let dir = scratch_dir("speech-report-missing");
        let at = dir.join("speech-log-nothing");
        let report = analyze(&Console::read(&at));
        assert!(
            report.findings[0].contains("missing or empty"),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn an_empty_host_console_is_the_same_finding() {
        let (_dir, at) = records("speech-report-empty", &[]);
        let report = analyze(&Console::read(&at));
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(report.findings[0].contains("missing or empty"));
    }

    #[test]
    fn a_console_with_no_started_line_is_a_finding() {
        let (_dir, at) = records("speech-report-unstarted", &[COMPOSED]);
        let report = analyze(&Console::read(&at));
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("no `started` line")),
            "{:?}",
            report.findings
        );
    }

    /// A voiceless host is a missing `composed` with a better message: the
    /// operator's remedy is a rebuild, not a look at the pipeline.
    #[test]
    fn a_voiceless_host_fails_by_name() {
        let voiceless = r#"{"stream":"edge","at_ns":2,"kind":"voiceless","says":"no speech configuration was named"}"#;
        let (_dir, at) = records("speech-report-voiceless", &[STARTED, voiceless]);
        let report = analyze(&Console::read(&at));
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(
            report.findings[0].contains("REACHY_SPEECH_CONFIG"),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn a_host_awaiting_its_speech_config_fails_by_name() {
        let awaiting = r#"{"stream":"edge","at_ns":2,"kind":"awaiting_speech_config","says":"no speech configuration at host/speech.toml"}"#;
        let (_dir, at) = records("speech-report-awaiting", &[STARTED, awaiting]);
        let report = analyze(&Console::read(&at));
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(
            report.findings[0].contains("did not find it on the unit"),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn a_host_that_never_composed_fails() {
        let (_dir, at) = records("speech-report-uncomposed", &[STARTED]);
        let report = analyze(&Console::read(&at));
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(
            report.findings[0].contains("no `composed` line"),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn a_critical_alert_fails_the_run() {
        let critical = r#"{"stream":"alert","at_ns":3,"severity":"critical","title":"the head is parked","says":"a fault was recorded"}"#;
        let (_dir, at) = records("speech-report-critical", &[STARTED, COMPOSED, critical]);
        let report = analyze(&Console::read(&at));
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(
            report.findings[0].contains("the head is parked"),
            "{:?}",
            report.findings
        );
    }

    /// A warning is a human's business and not a verdict's.
    #[test]
    fn a_warning_alert_is_printed_and_does_not_fail() {
        let warning = r#"{"stream":"alert","at_ns":3,"severity":"warning","title":"the bus dropped","says":"reconnecting"}"#;
        let (_dir, at) = records("speech-report-warning", &[STARTED, COMPOSED, warning]);
        let report = analyze(&Console::read(&at));
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("1 warning alert(s)")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("the bus dropped")),
            "{:?}",
            report.measured
        );
    }

    /// An alert that never travelled is prominent and still not a failure: the
    /// condition it carried is the line above it.
    #[test]
    fn an_unpublished_alert_is_prominent_and_does_not_fail() {
        let unpublished = r#"{"stream":"edge","at_ns":4,"kind":"unpublished","says":"the alert `the head is parked` was not carried to the bus","title":"the head is parked"}"#;
        let (_dir, at) = records(
            "speech-report-unpublished",
            &[STARTED, COMPOSED, unpublished],
        );
        let report = analyze(&Console::read(&at));
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("1 alert(s) raised that nothing carried")),
            "{:?}",
            report.measured
        );
    }

    /// The host's own exit message is not JSON, and a console ending in one is
    /// a host that never ran rather than one that drained.
    #[test]
    fn a_console_ending_in_a_refusal_fails() {
        let refusal = "reachy-host: the speech configuration at host/speech.toml has an unknown \
                       field `wakeword`";
        let (_dir, at) = records("speech-report-refused", &[STARTED, refusal]);
        let report = analyze(&Console::read(&at));
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("refusing rather than draining")),
            "{:?}",
            report.findings
        );
    }

    /// The same message with the run continuing after it is not an exit.
    #[test]
    fn a_refusal_shaped_line_that_is_not_last_is_not_an_exit() {
        let refusal = "reachy-host: something it said and kept going";
        let (_dir, at) = records("speech-report-not-last", &[STARTED, refusal, COMPOSED]);
        let report = analyze(&Console::read(&at));
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("1 line(s) that are not JSON")),
            "{:?}",
            report.measured
        );
    }

    /// Every dropped body the edge has a screen for is counted as one.
    #[test]
    fn refusal_kinds_are_counted_as_dropped_bodies() {
        let mut lines = vec![STARTED.to_owned(), COMPOSED.to_owned()];
        for kind in refusal_kinds() {
            lines.push(format!(
                r#"{{"stream":"edge","at_ns":5,"kind":"{kind}","says":"a body was dropped"}}"#
            ));
        }
        let borrowed: Vec<&str> = lines.iter().map(String::as_str).collect();
        let (_dir, at) = records("speech-report-refusals", &borrowed);
        let report = analyze(&Console::read(&at));
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        let counted = format!("{} body(ies) the edge dropped", refusal_kinds().len());
        assert!(
            report.measured.iter().any(|line| line.contains(&counted)),
            "{:?}",
            report.measured
        );
    }

    /// The narrator promises numbers for kinds it has no word for, and this
    /// tool promises not to fail on them.
    #[test]
    fn unknown_kinds_are_carried_as_their_numbers() {
        let unknown = r#"{"stream":"timeline","at_ns":6,"kind":"kind_47","says":"47"}"#;
        let (_dir, at) = records("speech-report-unknown-kind", &[STARTED, COMPOSED, unknown]);
        let report = analyze(&Console::read(&at));
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("no word for") && line.contains("kind_47")),
            "{:?}",
            report.measured
        );
    }

    #[test]
    fn an_unknown_stream_is_counted_and_does_not_fail() {
        let foreign = r#"{"stream":"weather","at_ns":7,"kind":"rain"}"#;
        let (_dir, at) = records(
            "speech-report-unknown-stream",
            &[STARTED, COMPOSED, foreign],
        );
        let report = analyze(&Console::read(&at));
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("stream `weather`")),
            "{:?}",
            report.measured
        );
    }

    /// A site sending its pipeline's JSONL to stdout puts a second schema in
    /// this file. It is summarized and never judged.
    #[test]
    fn pipeline_events_are_summarized_by_their_keys() {
        let event = r#"{"event":"wake","score":0.8,"ts":9}"#;
        let (_dir, at) = records("speech-report-pipeline", &[STARTED, COMPOSED, event, event]);
        let report = analyze(&Console::read(&at));
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("2 pipeline event(s)")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("carrying event,score,ts")),
            "{:?}",
            report.measured
        );
    }

    /// The console is shared, and the sample is bounded so a library's chatter
    /// cannot bury the report.
    #[test]
    fn noise_is_counted_with_a_bounded_sample() {
        let mut lines = vec![STARTED.to_owned(), COMPOSED.to_owned()];
        for n in 0..10 {
            lines.push(format!("some library said {n}"));
        }
        let borrowed: Vec<&str> = lines.iter().map(String::as_str).collect();
        let (_dir, at) = records("speech-report-noise", &borrowed);
        let report = analyze(&Console::read(&at));
        let quoted = report
            .measured
            .iter()
            .filter(|line| line.contains("not JSON: "))
            .count();
        assert_eq!(quoted, 3, "{:?}", report.measured);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("10 line(s) that are not JSON")),
            "{:?}",
            report.measured
        );
    }

    /// A line long enough to bury a terminal is cut, and a newline in a quoted
    /// text cannot forge a second line of the report.
    #[test]
    fn a_quoted_line_is_bounded_and_single_line() {
        let long = "x".repeat(400);
        let (_dir, at) = records("speech-report-long", &[STARTED, COMPOSED, &long]);
        let report = analyze(&Console::read(&at));
        let quoted = report
            .measured
            .iter()
            .find(|line| line.contains("not JSON: "))
            .expect("the sample");
        assert!(quoted.chars().count() < 200, "{quoted}");
        assert!(quoted.ends_with('…'), "{quoted}");
    }

    /// The other half of the same promise, which the bound alone does not keep.
    #[test]
    fn a_quoted_line_carries_no_control_character_onward() {
        // A tab, a screen-clearing escape and a carriage return, in a line
        // this tool did not write: the console is shared, and text from a
        // library or from transcribed speech reaches an operator's terminal
        // and the file the run record is filed with through the quote.
        let hostile = "noise\twith \x1b[2J an escape\rand a break";
        let (_dir, at) = records("speech-report-control", &[STARTED, COMPOSED, hostile]);
        let report = analyze(&Console::read(&at));
        let quoted = report
            .measured
            .iter()
            .find(|line| line.contains("not JSON: "))
            .expect("the sample");
        assert!(
            !quoted.chars().any(char::is_control),
            "a control character reached the report: {quoted:?}"
        );
        assert!(quoted.contains("an escape"), "{quoted:?}");
        assert_eq!(quoted.lines().count(), 1, "{quoted:?}");
    }

    /// The count and the sentence beside it, where a kind repeats.
    #[test]
    fn a_kind_that_repeats_is_counted_and_says_where_it_got_to() {
        let row = |says: &str| {
            format!(r#"{{"stream":"timeline","at_ns":3,"kind":"phase","says":"{says}"}}"#)
        };
        let (first, second, third) = (row("first"), row("second"), row("third"));
        let (_dir, at) = records(
            "speech-report-repeats",
            &[STARTED, COMPOSED, &first, &second, &third],
        );
        let report = analyze(&Console::read(&at));
        let counted = report
            .measured
            .iter()
            .find(|line| line.starts_with("timeline phase"))
            .expect("the kind");
        assert!(counted.contains("×3"), "{counted}");
        assert!(counted.contains("third"), "{counted}");
        assert!(!counted.contains("first"), "{counted}");
    }

    /// A loudness this build has no word for is measured, not dropped.
    #[test]
    fn an_alert_at_an_unknown_severity_is_carried_rather_than_discarded() {
        let chatty = r#"{"stream":"alert","at_ns":3,"severity":"chatty","title":"a newer host","says":"something happened"}"#;
        let (_dir, at) = records("speech-report-severity", &[STARTED, COMPOSED, chatty]);
        let report = analyze(&Console::read(&at));
        assert!(
            report.findings.is_empty(),
            "a word this build lacks is not a fault: {:?}",
            report.findings
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("severity `chatty`")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("0 warning alert(s)")),
            "{:?}",
            report.measured
        );
    }

    #[test]
    fn the_pods_console_is_noted_either_way() {
        let (_dir, at) = records("speech-report-pod", &[STARTED, COMPOSED]);
        let report = analyze(&Console::read(&at));
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("no pod_0.log beside it")),
            "{:?}",
            report.measured
        );
        std::fs::write(
            at.with_file_name(format!(
                "{}.console",
                at.file_name().expect("a name").to_string_lossy()
            ))
            .join(POD_LOG),
            "",
        )
        .expect("the pod's console");
        let report = analyze(&Console::read(&at));
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("pod_0.log came back")),
            "{:?}",
            report.measured
        );
    }

    /// The pair is named by construction, and the console is found by the
    /// record path's own spelling.
    #[test]
    fn the_console_sits_beside_the_records_by_name() {
        assert_eq!(
            console_dir(&PathBuf::from("/records/speech-log-1")),
            PathBuf::from("/records/speech-log-1.console")
        );
    }

    #[test]
    fn a_line_with_no_stream_is_a_pipeline_event() {
        assert_eq!(
            classify(r#"{"a":1,"b":2}"#),
            Line::Pipeline {
                keys: "a,b".to_owned()
            }
        );
    }

    /// A JSON scalar is not an event: the stream is objects, and anything else
    /// on the console is somebody's print.
    #[test]
    fn a_json_scalar_is_noise() {
        assert_eq!(
            classify("42"),
            Line::Noise {
                text: "42".to_owned()
            }
        );
    }

    /// The console is shared with anything the process's libraries print, and a
    /// supervised session cannot be re-run: one stray byte must not read as a
    /// fetch that came back with nothing.
    #[test]
    fn bytes_that_are_not_utf8_cost_a_word_and_not_the_session() {
        let mut bytes = Vec::new();
        bytes.extend_from_slice(STARTED.as_bytes());
        bytes.push(b'\n');
        bytes.extend_from_slice(b"an audio library says \xff\xfe and means nothing by it\n");
        bytes.extend_from_slice(COMPOSED.as_bytes());
        bytes.push(b'\n');
        let (_dir, at) = byte_records("speech-report-not-utf8", &bytes);
        let report = analyze(&Console::read(&at));
        assert!(
            report.findings.is_empty(),
            "the run still came up and composed: {:?}",
            report.findings
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("not UTF-8")),
            "and the reading says it was lossy: {:?}",
            report.measured
        );
    }

    /// The words this tool looks for are the host's own, not a copy of them:
    /// what the host actually emits is what is read here, so a rename is a
    /// compile error rather than an analyzer that quietly stops detecting.
    #[test]
    fn the_hosts_own_lines_are_the_ones_this_reads() {
        let at_ns = clockwork_rs::SyncTime::from_nanos(1);
        let absent = reachy_host::absent_line(std::path::Path::new("host/speech.toml"), at_ns);
        let (_dir, at) = records("speech-report-host-emitted", &[STARTED, &absent]);
        let report = analyze(&Console::read(&at));
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("did not find it on the unit")),
            "{:?}",
            report.findings
        );
    }

    /// The binary's other refusal path prints its usage text after the message,
    /// so the last line of a console that ends that way is not the message. A
    /// refusal with nothing but noise after it is still how that run ended.
    #[test]
    fn a_refusal_followed_by_usage_text_is_still_a_refusal() {
        let (_dir, at) = records(
            "speech-report-refusal-usage",
            &[
                STARTED,
                "reachy-host: `--speach-config` is not an option this takes",
                "",
                "usage: reachy-host [--config <path>] [--speech-config <path>] [--check]",
            ],
        );
        let report = analyze(&Console::read(&at));
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("refusing rather than draining")),
            "{:?}",
            report.findings
        );
    }
}
