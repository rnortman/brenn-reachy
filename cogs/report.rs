//! The shape every log analyzer in this tree reports in, and the printer for it.
//!
//! Two analyzers read the two halves of one fetch — the channel log and the
//! voice host's console — and an operator reads their output side by side after
//! the same `make …-fetch`. That output is a contract with a person, not an
//! internal detail, so it is one implementation rather than one per tool: a
//! second copy drifts on the first change nobody made twice, and the divergence
//! is invisible until somebody compares two runs.
//!
//! What a tool keeps for itself is the analysis. What it gets from here is the
//! two lists, the words for adding to them, the printer that turns them into
//! stdout, stderr and an exit status, how the fetch's directories are named
//! beside a record directory — the layout both analyzers read the same fetch
//! through — how a console line that tore is read back into the events it
//! holds, the spellings that reading matches on, and the bounding every piece
//! of console text passes through before it reaches a person.

#![forbid(unsafe_code)]

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use serde_json::Value;

/// Where an event begins on a merged console: the leading key every line of the
/// voice pipeline's JSONL carries.
///
/// The argument [`recover`] takes, and the one spelling of it — a head written
/// out per analyzer is a tear one of them recovers and the other reads as
/// noise, which is the disagreement `recover` exists to prevent.
pub const EVENT_HEAD: &str = "{\"ts_ms\"";

/// The pipeline's `event` names both analyzers of a fetch match on.
///
/// Shared for the same reason the head is: they are another repository's
/// spellings, read here against a stream this tree does not write, and a rename
/// upstream that reaches one analyzer's literal and not the other's is two
/// tools disagreeing about what a run did. Only the names both read are here;
/// an event one analyzer alone cares about stays where it is read.
pub mod event {
    /// The pipeline announcing that it is transcribing. Its absence is what
    /// says nothing was listening over a session.
    pub const LISTENING: &str = "listening";
    /// One carve, with its transcript, confidence and host-receipt stamps.
    pub const UTTERANCE: &str = "utterance";
    /// The pod beginning to play a reply: the reply's first frame written to
    /// the device, which the device begins playing a playout hop later.
    pub const PLAYBACK_STARTED: &str = "playback_started";
    /// The last of a reply being heard. The pacer's estimate of where the
    /// device runs out of audio, so a start and its finish bound what came out
    /// of the speaker; the write finishing up to the pacer's lead earlier is
    /// `playback_written`, which neither analyzer pairs a playback by.
    pub const PLAYBACK_FINISHED: &str = "playback_finished";
    /// A reply cut short by somebody speaking over it. Terminal: the job
    /// settles on it and no finish follows, so a reader pairing playbacks by
    /// their starts and finishes alone never closes a barged one.
    pub const PLAYBACK_FLUSHED: &str = "playback_flushed";
}

/// The utterance an event names, off whichever field it names it in.
///
/// Four spellings, because the pipeline names an utterance four ways: the
/// utterance's own line calls it `id`, the events that answer it call it
/// `utterance`, the recogniser's failure calls it `utterance_seq`, and a
/// supersession names the whole identity with the sequence inside it.
///
/// Shared for the reason the names above are: both analyzers of a fetch pair a
/// reply's events by this number — one keying a turn ledger, the other pairing a
/// playback's start with its end — and a second copy of the spellings is a
/// rename upstream that reaches one analyzer and not the other, which is two
/// reports of one run disagreeing.
#[must_use]
pub fn utterance_id(object: &serde_json::Map<String, Value>) -> Option<u64> {
    for key in ["utterance", "id", "utterance_seq"] {
        if let Some(seq) = object.get(key).and_then(Value::as_u64) {
            return Some(seq);
        }
    }
    object
        .get("utterance_id")
        .and_then(Value::as_object)
        .and_then(|id| id.get("seq"))
        .and_then(Value::as_u64)
}

/// A line's text, bounded and stripped of control characters.
///
/// The console is a shared file and the text in it is not all this tree's: a
/// pipeline event quoting what somebody said, or naming a log the pod chose,
/// reaches a terminal and a document through here. A name carrying a newline
/// would otherwise fabricate a line of a report.
#[must_use]
pub fn quote(text: &str) -> String {
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

/// How much of a quoted line a report prints before eliding the rest.
const QUOTE_LIMIT: usize = 160;

/// What the fetch names the console directory beside a record directory.
pub const CONSOLE_SUFFIX: &str = ".console";

/// What the fetch names the recorded-audio store beside a record directory.
pub const AUDIO_SUFFIX: &str = ".audio";

/// A directory named beside `records` by suffixing the record directory's own
/// name.
///
/// Built from the record path's own spelling rather than from a sibling scan:
/// the set is named by construction, and a directory holding two runs would
/// otherwise be ambiguous. Shared, because the fetch's naming is a contract
/// every analyzer of the same fetch reads through — one of them left behind by
/// a change to the layout reports no log against a directory that is right
/// there.
#[must_use]
pub fn sibling(records: &Path, suffix: &str) -> PathBuf {
    let mut name = records
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    name.push_str(suffix);
    records.with_file_name(name)
}

/// The console directory the fetch wrote beside `records`.
#[must_use]
pub fn console_dir(records: &Path) -> PathBuf {
    sibling(records, CONSOLE_SUFFIX)
}

/// The recorded-audio store the fetch wrote beside `records`.
#[must_use]
pub fn audio_dir(records: &Path) -> PathBuf {
    sibling(records, AUDIO_SUFFIX)
}

/// What one console line holds: the whole JSON objects in it, and the text
/// around them.
///
/// `ahead` is whatever preceded the first object and `behind` whatever followed
/// the last one; both are the caller's noise, and both are `None` on a line that
/// held no object at all.
pub struct Recovered {
    /// The text ahead of the first object, where there was any.
    pub ahead: Option<String>,
    /// Every whole object the line holds, in order.
    pub values: Vec<Value>,
    /// The text behind the last object, where there was any.
    pub behind: Option<String>,
}

/// Read one line of a merged console back into the events it holds.
///
/// The voice host's two output streams are one file: the launcher redirects both
/// into the same descriptor, and they tear — a console sentence still without
/// its newline, and a JSONL event written behind it. A whole-line parse reads
/// every torn line as noise, which loses exactly the events the analyzers are
/// assembled from, so a line that does not parse whole is read as the whole
/// objects it holds and whatever text surrounds them.
///
/// The tear runs both directions and both are ordinary: an event glued onto an
/// unterminated sentence, and a sentence begun into the same descriptor behind
/// a whole event.
///
/// `head` is where an event begins — the leading key every line of the stream
/// carries — and every occurrence of it is tried rather than the first, because
/// the console text a real event is glued onto can hold that spelling too. From
/// the first occurrence that yields anything, every whole object on is kept, not
/// only the first: two writers interleaving cleanly put two finished events in
/// one descriptor with no newline between them, and the second is as real as the
/// first.
///
/// Objects only. A scalar or an array is not an event — the stream is objects,
/// and anything else on the console is somebody's print — so the read stops at
/// the first value that is not one, and a line with no whole object anywhere in
/// it recovers nothing, which is what a sentence merely quoting the spelling is.
///
/// Shared, because both analyzers of one fetch read a console the launcher
/// merged the same way, and a tear one of them recovers and the other does not
/// is two tools disagreeing about what a run did.
#[must_use]
pub fn recover(raw: &str, head: &str) -> Recovered {
    let starts = std::iter::once(0).chain(raw.match_indices(head).map(|(at, _)| at));
    for at in starts {
        let (values, until) = leading(&raw[at..]);
        if values.is_empty() {
            continue;
        }
        let behind = &raw[at + until..];
        return Recovered {
            ahead: (at > 0).then(|| raw[..at].to_owned()),
            values,
            behind: (!behind.is_empty()).then(|| behind.to_owned()),
        };
    }
    Recovered {
        ahead: None,
        values: Vec::new(),
        behind: None,
    }
}

/// Every whole JSON object from the start of some text, and how much of the text
/// they took.
///
/// A streaming read rather than a whole-text parse: trailing content is what a
/// tear leaves behind an event, and a parse that rejects the line for it throws
/// away the event with it.
fn leading(raw: &str) -> (Vec<Value>, usize) {
    let mut values = serde_json::Deserializer::from_str(raw).into_iter::<Value>();
    let mut held = Vec::new();
    let mut until = 0;
    while let Some(Ok(value)) = values.next() {
        if !value.is_object() {
            break;
        }
        until = values.byte_offset();
        held.push(value);
    }
    (held, until)
}

/// What an analyzer concluded about one run.
///
/// Two lists rather than one, because they are read for different reasons. A
/// finding is a claim about the run that did not hold and is what the exit
/// status is about; a measurement is a number the run produced, printed whether
/// the run passed or not. A run that fails is exactly the run whose numbers
/// somebody needs.
#[derive(Default)]
pub struct Report {
    /// The ways the run did not do what it claims to have done.
    pub findings: Vec<String>,
    /// What the run did, for a person to read.
    pub measured: Vec<String>,
}

impl Report {
    /// One way the run did not do what it claims to have done.
    pub fn fail(&mut self, what: impl Into<String>) {
        self.findings.push(what.into());
    }

    /// One thing the run did.
    pub fn note(&mut self, what: impl Into<String>) {
        self.measured.push(what.into());
    }
}

/// Print both halves and answer with the verdict.
///
/// The measurements go to stdout and the findings to stderr: the numbers file
/// with the run record and the findings are what an operator sees on the
/// terminal. `clean` is the one-line sentence a run with no findings gets — the
/// only part of this that is the tool's own.
pub fn verdict(tool: &str, over: &str, report: &Report, clean: &str) -> ExitCode {
    if write_verdict(
        &mut std::io::stdout().lock(),
        &mut std::io::stderr().lock(),
        tool,
        over,
        report,
        clean,
    ) {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

/// Both halves onto the two sinks, and whether the run held.
///
/// The split is the contract: an operator reads the findings off a terminal
/// while the measurements are redirected into the file that is filed with the
/// run record, and a finding that went to stdout is one nobody was told about.
/// Written to handed-in sinks so a case can hold the two apart, which a printer
/// asserted only through its exit status cannot.
///
/// # Panics
///
/// If either sink cannot be written, which for the caller's own streams is a
/// terminal or a file that went away mid-report.
pub fn write_verdict(
    out: &mut impl std::io::Write,
    err: &mut impl std::io::Write,
    tool: &str,
    over: &str,
    report: &Report,
    clean: &str,
) -> bool {
    writeln!(out, "{tool} over {over}").expect("a writable stream");
    for line in &report.measured {
        writeln!(out, "{line}").expect("a writable stream");
    }
    if report.findings.is_empty() {
        writeln!(out, "{clean}").expect("a writable stream");
        return true;
    }
    for finding in &report.findings {
        writeln!(err, "{tool}: {finding}").expect("a writable stream");
    }
    writeln!(
        err,
        "{tool}: {} finding(s) over {over}",
        report.findings.len()
    )
    .expect("a writable stream");
    false
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use super::{
        EVENT_HEAD as HEAD, Report, audio_dir, console_dir, quote, recover, verdict, write_verdict,
    };

    #[test]
    fn the_fetch_names_both_siblings_off_the_record_directory() {
        let records = Path::new("/tmp/runs/speech-log-20260910T000000Z");
        assert_eq!(
            console_dir(records),
            Path::new("/tmp/runs/speech-log-20260910T000000Z.console")
        );
        assert_eq!(
            audio_dir(records),
            Path::new("/tmp/runs/speech-log-20260910T000000Z.audio")
        );
    }

    #[test]
    fn a_whole_line_recovers_as_its_one_object() {
        let recovered = recover(r#"{"ts_ms":1,"event":"listening"}"#, HEAD);
        assert_eq!(recovered.values.len(), 1);
        assert_eq!(recovered.values[0]["event"], "listening");
        assert!(recovered.ahead.is_none());
        assert!(recovered.behind.is_none());
    }

    /// The two ordinary tears, and the two events one descriptor can hold.
    #[test]
    fn an_event_glued_to_a_sentence_recovers_with_the_sentence_beside_it() {
        let recovered = recover(
            r#"utterance #1 — "hello"{"ts_ms":1,"event":"utterance"}next line begins"#,
            HEAD,
        );
        assert_eq!(recovered.values.len(), 1);
        assert_eq!(recovered.values[0]["event"], "utterance");
        assert_eq!(
            recovered.ahead.as_deref(),
            Some(r#"utterance #1 — "hello""#)
        );
        assert_eq!(recovered.behind.as_deref(), Some("next line begins"));

        let two = recover(
            r#"{"ts_ms":1,"event":"playback_started"}{"ts_ms":2,"event":"playback_finished"}"#,
            HEAD,
        );
        assert_eq!(two.values.len(), 2);
        assert_eq!(two.values[1]["event"], "playback_finished");
    }

    /// A sentence quoting the spelling holds no event, and a scalar is nobody's
    /// event either.
    #[test]
    fn text_that_only_looks_like_an_event_recovers_nothing() {
        assert!(
            recover(r#"the host writes {"ts_ms" first on every line"#, HEAD)
                .values
                .is_empty()
        );
        assert!(recover("42", HEAD).values.is_empty());
        assert!(recover("[1,2]", HEAD).values.is_empty());
    }

    /// What the printer sent to each sink, as text.
    fn printed(report: &Report) -> (bool, String, String) {
        let mut out: Vec<u8> = Vec::new();
        let mut err: Vec<u8> = Vec::new();
        let held = write_verdict(
            &mut out,
            &mut err,
            "tool",
            "somewhere",
            report,
            "it happened",
        );
        (
            held,
            String::from_utf8(out).expect("text"),
            String::from_utf8(err).expect("text"),
        )
    }

    #[test]
    fn a_report_starts_with_nothing_in_either_half() {
        let report = Report::default();
        assert!(report.findings.is_empty());
        assert!(report.measured.is_empty());
    }

    #[test]
    fn each_half_keeps_what_it_was_given_in_order() {
        let mut report = Report::default();
        report.note("first");
        report.fail("broken");
        report.note("second");
        assert_eq!(report.measured, vec!["first", "second"]);
        assert_eq!(report.findings, vec!["broken"]);
    }

    #[test]
    fn a_report_with_no_findings_succeeds() {
        let mut report = Report::default();
        report.note("a number");
        assert_eq!(
            format!("{:?}", verdict("tool", "somewhere", &report, "it happened")),
            format!("{:?}", std::process::ExitCode::SUCCESS)
        );
    }

    #[test]
    fn a_clean_report_says_it_on_stdout_and_says_nothing_on_stderr() {
        let mut report = Report::default();
        report.note("a number");
        let (held, out, err) = printed(&report);
        assert!(held);
        assert_eq!(out, "tool over somewhere\na number\nit happened\n");
        assert_eq!(err, "");
    }

    #[test]
    fn the_findings_go_to_stderr_named_and_counted_and_the_numbers_stay_on_stdout() {
        let mut report = Report::default();
        report.note("a number");
        report.fail("it did not");
        report.fail("nor that");
        let (held, out, err) = printed(&report);
        assert!(!held);
        assert_eq!(out, "tool over somewhere\na number\n");
        assert_eq!(
            err,
            "tool: it did not\ntool: nor that\ntool: 2 finding(s) over somewhere\n"
        );
    }

    #[test]
    fn quoting_bounds_a_line_and_flattens_its_control_characters() {
        assert_eq!(
            quote("a.framelog\nturn clips: 9 written"),
            "a.framelog turn clips: 9 written"
        );
        let long = "x".repeat(200);
        let held = quote(&long);
        assert_eq!(held.chars().count(), 161);
        assert!(held.ends_with('…'));
        assert_eq!(quote("plain"), "plain");
    }

    #[test]
    fn a_report_with_a_finding_fails() {
        let mut report = Report::default();
        report.fail("it did not");
        assert_eq!(
            format!("{:?}", verdict("tool", "somewhere", &report, "it happened")),
            format!("{:?}", std::process::ExitCode::FAILURE)
        );
    }
}
