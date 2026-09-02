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
//! whether a bus brain kept the attachment its turns and its motion channel
//! ride, whether anything critical fired, and whether it ended by draining or
//! by refusing its own configuration. Everything else is measured and printed
//! and never fails: counts per kind with the sentence each kind said, the warning
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

/// The pipeline event a bus brain's composition is announced by.
///
/// A speech-surface event name and not this tree's, so it is spelled once here.
/// Its presence is what makes the attachment this run's business at all: a
/// deployment with an echo brain or none has no bus to lose.
const BRAIN_BRENN: &str = "brain_brenn";

/// The pipeline event an attachment that negotiated a wire version emits.
const BRENN_ATTACHED: &str = "brenn_attached";

/// The pipeline event the bus attachment's end emits.
///
/// Every run that composed a bus brain and whose driver returned emits one,
/// including an orderly shutdown's. What separates the two is its `unexpected`
/// field, so this check keys on that field and never on the event being
/// present.
const BRENN_BRIDGE_EXIT: &str = "brenn_bridge_exit";

/// The pipeline event a bus driver that ended without returning emits.
///
/// A speech-surface event name and not this tree's. The pipeline emits it only
/// where the driver task did not return cleanly, mid-run or at teardown; a
/// clean return is silent on both paths, so the event cannot appear in a
/// healthy run and its presence alone is the finding. The `reason` it carries
/// says how the driver ended, and is quoted rather than judged: the two
/// emission sites are indistinguishable in the line, so a reason this tool
/// read as orderly would pass a run whose bus died mid-run under that word.
const BRENN_DRIVER_EXITED: &str = "brenn_driver_exited";

/// Where a pipeline event begins when one is glued onto a console line.
///
/// Every event the pipeline emits leads with its timestamp, which is what makes
/// the torn shape below recoverable rather than guessed at.
const EVENT_HEAD: &str = "{\"ts_ms\"";

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
    /// belongs to `speech-surface` and is summarized by its name and its keys.
    Pipeline { event: String, keys: String },
    /// The four pipeline events the bridge check reads fields out of.
    ///
    /// Its own variant so that the only schema this tool holds for another
    /// repo's event stream sits in one place: [`Line::Pipeline`] stays the
    /// summary every other event is. A fifth event wanted here is two edits —
    /// an arm in [`object`]'s one `match` on the event name, and an arm in the
    /// fold [`Console::absorb`] does over this variant.
    ///
    /// It carries the key summary too, because these are pipeline events like
    /// any other and are counted as such.
    Bridge {
        keys: String,
        event: String,
        /// Whether nobody asked for this ending: an exit's own field, and true
        /// for every driver ending, which the pipeline emits for nothing else.
        /// False for the two events that end nothing.
        unexpected: bool,
        /// How the event names the loss, in its own words: the ending's
        /// `outcome`, or a driver's `reason` and `detail`. Empty for the two
        /// events that name no loss.
        loss: String,
    },
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
    /// The pipeline's own events, counted by their name and their top-level key
    /// list.
    ///
    /// The name is in the key because the vocabulary is another repo's and
    /// grows there: several of its events share one shape (a name, a `reason`
    /// and a `detail`), so a shape count alone tells an operator that something
    /// died and not what. An event this tool has no reader for at least appears
    /// here by name.
    pipeline: BTreeMap<(String, String), usize>,
    /// Whether the pipeline composed a brain on the bus.
    ///
    /// What makes the attachment this run's business: a deployment whose brain
    /// is elsewhere loses nothing when there is no bridge.
    brain_brenn: bool,
    /// Whether the bus attachment ever negotiated a wire version.
    attached: bool,
    /// The outcome each bus attachment that ended unexpectedly named.
    ///
    /// Kept rather than counted, and in full: the sentence carries the two
    /// version ranges that are the whole diagnosis of a skew.
    bridge_exits: Vec<String>,
    /// What each bus driver that died on its own said it died of.
    ///
    /// A driver death is the other way to lose the bus brain, and the one that
    /// leaves no `brenn_bridge_exit` behind: the ending line comes from the
    /// driver's own last act, which a death never reaches. A driver ending that
    /// was asked for ([`CANCELLED`]) is no death and is not kept here.
    driver_deaths: Vec<String>,
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
        let Classified { ahead, line } = classify(raw);
        // The host's refusal is console text, and console text tears the same
        // way an event does — the refusal arrives glued behind a sentence that
        // never got its newline. So it is looked for inside the console half of
        // the line rather than at the start of the raw one, on the half that is
        // console text at all: a JSON event quoting the prefix is an event, not
        // an exit.
        let console = match (&ahead, &line) {
            (Some(text), _) => Some(text.as_str()),
            (None, Line::Noise { .. }) => Some(raw),
            (None, _) => None,
        };
        let refusal = console.and_then(refused_in);
        let refused_here = refusal.is_some();
        if let Some(message) = refusal {
            self.refused = Some(message);
        }
        if let Some(text) = &ahead {
            self.noticed_noise(quote(text));
        }
        match line {
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
            Line::Pipeline { event, keys } => *self.pipeline.entry((event, keys)).or_default() += 1,
            Line::Bridge {
                keys,
                event,
                unexpected,
                loss,
            } => {
                match event.as_str() {
                    BRAIN_BRENN => self.brain_brenn = true,
                    BRENN_ATTACHED => self.attached = true,
                    // An orderly teardown emits this event too, saying so in
                    // the same field, so the event alone is not the finding.
                    BRENN_BRIDGE_EXIT if unexpected => self.bridge_exits.push(loss),
                    // A driver that returned cleanly emits nothing, on either
                    // emission path, so the line itself is the death.
                    BRENN_DRIVER_EXITED => self.driver_deaths.push(loss),
                    _ => {}
                }
                *self.pipeline.entry((event, keys)).or_default() += 1;
            }
            Line::Noise { text } => {
                self.noticed_noise(text);
                return;
            }
        }
        // A line of the stream after a refusal means the process went on: the
        // refusal was something a run printed, not the way it ended. A line
        // that carried a refusal of its own is not such a line, whatever was
        // glued behind it.
        if !refused_here {
            self.refused = None;
        }
    }

    /// One line's worth of text that is not JSON, counted and maybe quoted.
    fn noticed_noise(&mut self, text: String) {
        self.noise += 1;
        if self.noise_sample.len() < NOISE_SAMPLE {
            self.noise_sample.push(text);
        }
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

/// What one raw line is: any console text ahead of an event, and the line.
struct Classified {
    /// The console text a recovered event was glued onto, as it was written.
    /// Absent on every line that is one thing, which is nearly all of them.
    ahead: Option<String>,
    /// What the line, or what is left of it, is.
    line: Line,
}

/// What one raw line is, recovering an event glued onto console text.
///
/// The host's two output streams are one file: the launcher redirects both into
/// `voice_host_0.log`, and they tear — a console sentence still without its
/// newline, and a JSONL event written into the same descriptor behind it. A
/// whole-line parse reads every torn line as noise. So a line that does not
/// parse whole is split at an [`EVENT_HEAD`], the suffix read as the event and
/// the text ahead of it kept as noise. A suffix that still does not parse
/// leaves the whole line noise, which is what a sentence merely quoting that
/// spelling is.
///
/// Every occurrence is tried rather than the first, because the console text a
/// real event is glued onto can hold that spelling too — a sentence quoting an
/// event, or a half-written event glued ahead of a whole one. Splitting at the
/// first match alone loses the real event at the line's end to the trailing
/// content the parse then rejects, and a lost `brain_brenn` is a run that
/// exempts itself from [`bridged`].
fn classify(raw: &str) -> Classified {
    if let Some(line) = object(raw) {
        return Classified { ahead: None, line };
    }
    for (at, _) in raw.match_indices(EVENT_HEAD) {
        if let Some(line) = object(&raw[at..]) {
            return Classified {
                ahead: Some(raw[..at].to_owned()),
                line,
            };
        }
    }
    Classified {
        ahead: None,
        line: Line::Noise { text: quote(raw) },
    }
}

/// What the host refused with inside one line of console text, where it did.
///
/// A tear glues the refusal onto the sentence ahead of it with nothing between,
/// so a match at the start of the text or one written straight against the
/// preceding character is the refusal. A match with a space ahead of it is
/// somebody's sentence naming this host — "relaying to reachy-host: ready" —
/// and reading that as an exit would fail a good run while quoting words the
/// host never said. The last qualifying match wins, because a tear puts the
/// refusal at the line's tail.
fn refused_in(text: &str) -> Option<String> {
    text.match_indices(REFUSAL_PREFIX)
        .filter(|(at, _)| {
            *at == 0
                || !text[..*at]
                    .chars()
                    .next_back()
                    .is_some_and(char::is_whitespace)
        })
        .last()
        .map(|(at, _)| text[at + REFUSAL_PREFIX.len()..].to_owned())
}

/// What one JSON object is, or nothing where the text is not one.
fn object(raw: &str) -> Option<Line> {
    let Ok(Value::Object(object)) = serde_json::from_str::<Value>(raw) else {
        return None;
    };
    let text = |key: &str| -> String {
        object
            .get(key)
            .and_then(Value::as_str)
            .unwrap_or_default()
            .to_owned()
    };
    Some(match object.get("stream").and_then(Value::as_str) {
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
        None => {
            let keys = object
                .keys()
                .map(String::as_str)
                .collect::<Vec<_>>()
                .join(",");
            let event = text("event");
            let read = match event.as_str() {
                BRAIN_BRENN | BRENN_ATTACHED => Some((false, String::new())),
                BRENN_BRIDGE_EXIT => Some((unexpected(&object), text("outcome"))),
                BRENN_DRIVER_EXITED => {
                    Some((true, died_of(&text("reason"), &text("detail"), &keys)))
                }
                _ => None,
            };
            match read {
                Some((unexpected, loss)) => Line::Bridge {
                    keys,
                    event,
                    unexpected,
                    loss,
                },
                None => Line::Pipeline { event, keys },
            }
        }
    })
}

/// Whether a bridge event says nobody asked for this ending.
///
/// The field is `unexpected`. A pipeline that spells it `fatal` is an older one
/// than this tool reads for — the two are the same computation under two names,
/// and a fetched log outlives the payload that wrote it — so that spelling is
/// honoured where the first is absent.
///
/// Anything else answers true. A bridge event whose shape this tool does not
/// know is a report and a pipeline on different sides of a change, which is the
/// skew class this whole check exists for: it fails the run and an operator
/// reads the line, rather than passing a run whose bus may have died unread.
fn unexpected(object: &serde_json::Map<String, Value>) -> bool {
    object
        .get("unexpected")
        .or_else(|| object.get("fatal"))
        .and_then(Value::as_bool)
        .unwrap_or(true)
}

/// How a driver ending names itself, out of the fields that are there.
///
/// `reason` and `detail` are speech-surface's spelling, and a fetched log
/// outlives the payload that wrote it — the same skew this tool reads two
/// spellings of `unexpected` for. Where neither field is a string, the event's
/// own key list is what an operator gets, because a sentence with nothing on
/// either side of its dash is worse than no sentence at the moment somebody is
/// diagnosing a dead bus.
fn died_of(reason: &str, detail: &str, keys: &str) -> String {
    let named: Vec<&str> = [reason, detail]
        .into_iter()
        .filter(|field| !field.is_empty())
        .collect();
    if named.is_empty() {
        format!("it named no reason this build reads; the event carried {keys}")
    } else {
        named.join(" — ")
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

/// Did the bus brain keep its attachment, where the run had one.
///
/// The pipeline outlives a bridge that ends: turns speak the configured failure
/// message, the wake word and the endpointer go on working, the head stows on
/// its own schedule. That is the right posture for the machine and the wrong
/// verdict for this target — the brain and the motion channel both ride that
/// attachment, so a run without it is a run in which nothing a person says can
/// move the robot.
///
/// Three ways to have lost it. A bridge that ended unexpectedly is the mid-run
/// case, and it is a finding whatever brain this run composed: the motion
/// channel and the alert seam ride that attachment too, so a run that lost it
/// lost more than a brain. A driver that died on its own is the same loss by
/// another road, and it leaves no ending line — the ending is emitted by
/// the driver's last act, which a death never reaches — so it is read for
/// separately. A brain composed with no attachment event
/// behind it is the never-negotiated case, and that one is a `brain_brenn`
/// run's alone — a deployment that never asked for a bus is not a deployment
/// missing one.
///
/// An ending line and a driver death are two accounts and both are said. They
/// are two because the ending line answers for itself whether anyone asked for
/// it: its `unexpected` field is true iff nobody commanded the ending. A
/// shutdown the pipeline commanded is marked `false` and is not read here at
/// all, so an ending marked `true` is a loss in its own right whatever else the
/// console carries, and this check infers nothing from two lines' order in a
/// file it already knows tears. The ending's outcome — the two version ranges,
/// where the loss is a skew — is the half no death sentence repeats. The
/// never-negotiated arm is the one that stands down, whenever either of the
/// others spoke: it says only that no attachment happened, which is what the
/// other two already explain.
fn bridged(console: &Console, report: &mut Report) {
    for outcome in &console.bridge_exits {
        report.fail(format!(
            "the bus attachment ended mid-run: {}. this run's brain and its motion channel both \
             rode that attachment, so nothing said to the robot could reach either",
            quote(outcome)
        ));
    }
    for death in &console.driver_deaths {
        report.fail(format!(
            "the bus driver died: {}. the pipeline went on without it, so this run's brain and \
             its motion channel were both gone from that moment",
            quote(death)
        ));
    }
    if console.bridge_exits.is_empty()
        && console.driver_deaths.is_empty()
        && console.brain_brenn
        && !console.attached
    {
        report.fail(format!(
            "a `{BRAIN_BRENN}` brain was composed and no `{BRENN_ATTACHED}` event ever followed: \
             the bus attachment never negotiated, so this run had no brain and no motion channel \
             from the first second"
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
            "{events} pipeline event(s) on this console, of {} kind(s)",
            console.pipeline.len()
        ));
        for ((event, keys), count) in &console.pipeline {
            if event.is_empty() {
                report.note(format!("  ×{count} carrying {keys}"));
            } else {
                report.note(format!("  ×{count} `{event}` carrying {keys}"));
            }
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
    bridged(console, &mut report);
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
        "the pipeline came up, composed, kept its bus attachment, and drained whole",
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

    /// A bus brain, composed and attached: the shape a healthy run has.
    const BRAIN: &str = r#"{"ts_ms":3,"event":"brain_brenn","publish_channel":"c"}"#;
    const ATTACHED: &str = r#"{"ts_ms":4,"event":"brenn_attached","version":4}"#;
    /// The ending line every orderly shutdown writes.
    const TEARDOWN: &str =
        r#"{"ts_ms":5,"event":"brenn_bridge_exit","unexpected":false,"outcome":"teardown"}"#;
    /// A driver that died on its own, mid-run. The `detail` does not repeat the
    /// reason word, so a finding that says `panic` got it from `reason`.
    const DIED: &str = r#"{"ts_ms":5,"event":"brenn_driver_exited","reason":"panic","detail":"bridge driver died mid-run: index out of bounds"}"#;

    /// The attachment ending mid-run is a failed speech run even though the
    /// pipeline survives it: the brain and the motion channel both rode it.
    #[test]
    fn a_bus_brain_whose_bridge_ended_unexpectedly_fails() {
        let exit = r#"{"ts_ms":5,"event":"brenn_bridge_exit","unexpected":true,"outcome":"no wire version in common: this bridge speaks 3..=3, the server speaks 4..=4"}"#;
        let (_dir, at) = records(
            "speech-report-bridge-exit",
            &[STARTED, COMPOSED, BRAIN, ATTACHED, exit],
        );
        let report = analyze(&Console::read(&at));
        let finding = report
            .findings
            .iter()
            .find(|finding| finding.contains("ended mid-run"))
            .expect("the bridge finding");
        assert!(
            finding.contains("this bridge speaks 3..=3, the server speaks 4..=4"),
            "the versions are the diagnosis: {finding}",
        );
    }

    /// The never-negotiated case: a brain on the bus and no attachment behind
    /// it, which is what a version skew looks like from the first second.
    #[test]
    fn a_bus_brain_that_never_attached_fails() {
        let (_dir, at) = records("speech-report-never-attached", &[STARTED, COMPOSED, BRAIN]);
        let report = analyze(&Console::read(&at));
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("never negotiated")),
            "{:?}",
            report.findings
        );
    }

    /// A deployment whose brain is not on the bus has no attachment to lose.
    #[test]
    fn a_run_with_no_bus_brain_is_exempt() {
        let (_dir, at) = records("speech-report-no-bus-brain", &[STARTED, COMPOSED]);
        let report = analyze(&Console::read(&at));
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// The other half of that exemption: a run with an echo brain still carries
    /// its motion intents and its alerts on the attachment, so an attachment
    /// that ends under it is the same loss and the same finding.
    #[test]
    fn a_bridge_lost_without_a_bus_brain_still_fails() {
        let exit =
            r#"{"ts_ms":5,"event":"brenn_bridge_exit","unexpected":true,"outcome":"peer closed"}"#;
        let (_dir, at) = records(
            "speech-report-no-brain-bridge-exit",
            &[STARTED, COMPOSED, ATTACHED, exit],
        );
        let report = analyze(&Console::read(&at));
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("ended mid-run")),
            "{:?}",
            report.findings
        );
    }

    /// A bridge event this tool does not know the shape of is a report and a
    /// pipeline on different sides of a change — the skew class the check
    /// exists for. It fails rather than reading the absent field as an orderly
    /// teardown and passing a run whose bus may have died.
    #[test]
    fn a_bridge_exit_with_no_unexpected_field_fails() {
        let exit =
            r#"{"ts_ms":5,"event":"brenn_bridge_exit","outcome":"no wire version in common"}"#;
        let (_dir, at) = records(
            "speech-report-bridge-exit-unknown",
            &[STARTED, COMPOSED, BRAIN, ATTACHED, exit],
        );
        let report = analyze(&Console::read(&at));
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("ended mid-run")),
            "{:?}",
            report.findings
        );
    }

    /// A payload that spells the field `fatal` is read by its own words rather
    /// than by the fallback: its orderly teardown is still a teardown.
    #[test]
    fn a_bridge_exit_spelling_the_field_fatal_is_read_by_it() {
        let torn_down =
            r#"{"ts_ms":5,"event":"brenn_bridge_exit","fatal":false,"outcome":"teardown"}"#;
        let (_dir, at) = records(
            "speech-report-bridge-exit-fatal-false",
            &[STARTED, COMPOSED, BRAIN, ATTACHED, torn_down],
        );
        assert!(
            analyze(&Console::read(&at)).findings.is_empty(),
            "an orderly teardown under either spelling is not a finding",
        );

        let died = r#"{"ts_ms":5,"event":"brenn_bridge_exit","fatal":true,"outcome":"no wire version in common"}"#;
        let (_dir, at) = records(
            "speech-report-bridge-exit-fatal-true",
            &[STARTED, COMPOSED, BRAIN, ATTACHED, died],
        );
        let report = analyze(&Console::read(&at));
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("ended mid-run")),
            "{:?}",
            report.findings
        );
    }

    /// A driver that died on its own leaves no ending line, so this event is
    /// the only record of the loss — and the loss is the same one: no brain and
    /// no motion channel from that moment.
    #[test]
    fn a_driver_that_died_fails_naming_what_it_died_of() {
        let (_dir, at) = records(
            "speech-report-driver-death",
            &[STARTED, COMPOSED, BRAIN, ATTACHED, DIED],
        );
        let report = analyze(&Console::read(&at));
        let finding = report
            .findings
            .iter()
            .find(|finding| finding.contains("the bus driver died"))
            .expect("the driver finding");
        assert!(
            finding.contains("panic") && finding.contains("bridge driver died mid-run"),
            "the event's own words are the diagnosis: {finding}",
        );
    }

    /// A driver that died before the attachment negotiated is one loss with two
    /// symptoms, and the run gets the finding that says what happened.
    #[test]
    fn a_driver_death_before_attaching_is_one_finding() {
        let (_dir, at) = records(
            "speech-report-driver-death-unattached",
            &[STARTED, COMPOSED, BRAIN, DIED],
        );
        let report = analyze(&Console::read(&at));
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(
            report.findings[0].contains("the bus driver died"),
            "{:?}",
            report.findings
        );
    }

    /// Two accounts of a lost bus arriving together are both said, in either
    /// file order: the death sentence does not carry the ending's outcome, which
    /// is where a skew names its two version ranges, and nothing is inferred
    /// from which line came first.
    #[test]
    fn a_driver_death_beside_an_ending_line_says_both() {
        let exit = concat!(
            r#"{"ts_ms":6,"event":"brenn_bridge_exit","unexpected":true,"#,
            r#""outcome":"no wire version in common: this bridge speaks 3..=3, the server speaks 4..=4"}"#,
        );
        for (name, tail) in [
            ("speech-report-driver-death-then-exit", [DIED, exit]),
            ("speech-report-exit-then-driver-death", [exit, DIED]),
        ] {
            let (_dir, at) = records(
                name,
                &[STARTED, COMPOSED, BRAIN, ATTACHED, tail[0], tail[1]],
            );
            let report = analyze(&Console::read(&at));
            assert_eq!(report.findings.len(), 2, "{name}: {:?}", report.findings);
            assert!(
                report
                    .findings
                    .iter()
                    .any(|finding| finding.contains("the bus driver died")
                        && finding.contains("panic — bridge driver died mid-run")),
                "{name}: {:?}",
                report.findings
            );
            assert!(
                report
                    .findings
                    .iter()
                    .any(|finding| finding.contains("ended mid-run")
                        && finding.contains("this bridge speaks 3..=3")),
                "{name}: the skew's own diagnosis survives the death beside it: {:?}",
                report.findings
            );
        }
    }

    /// An ending the pipeline commanded says so itself, `unexpected: false`,
    /// and is not read; a driver death beside it is the one finding in either
    /// file order, and the death's own `reason` and `detail` are quoted.
    #[test]
    fn a_driver_death_beside_a_commanded_ending_is_the_only_finding() {
        let exit = r#"{"ts_ms":6,"event":"brenn_bridge_exit","unexpected":false,"outcome":"the embedder asked for a shutdown"}"#;
        for (name, tail) in [
            (
                "speech-report-driver-death-then-commanded-exit",
                [DIED, exit],
            ),
            (
                "speech-report-commanded-exit-then-driver-death",
                [exit, DIED],
            ),
        ] {
            let (_dir, at) = records(
                name,
                &[STARTED, COMPOSED, BRAIN, ATTACHED, tail[0], tail[1]],
            );
            let report = analyze(&Console::read(&at));
            assert_eq!(report.findings.len(), 1, "{name}: {:?}", report.findings);
            let finding = &report.findings[0];
            assert!(
                finding.contains("the bus driver died")
                    && finding.contains("panic — bridge driver died mid-run"),
                "{name}: {finding}"
            );
            assert!(
                !finding.contains("ended mid-run"),
                "{name}: a commanded ending is not a loss: {finding}"
            );
        }
    }

    /// The two emission sites are one line, so a reason this tool read as
    /// orderly would pass a bus that died mid-run under that word. Every
    /// driver ending is a death here, `cancelled` included.
    #[test]
    fn a_driver_ending_that_was_cancelled_is_still_a_death() {
        let cancelled = r#"{"ts_ms":5,"event":"brenn_driver_exited","reason":"cancelled","detail":"bridge driver died mid-run: task was cancelled"}"#;
        let (_dir, at) = records(
            "speech-report-driver-cancelled",
            &[STARTED, COMPOSED, BRAIN, ATTACHED, cancelled],
        );
        let report = analyze(&Console::read(&at));
        let finding = report
            .findings
            .iter()
            .find(|finding| finding.contains("the bus driver died"))
            .expect("the driver finding");
        assert!(finding.contains("cancelled"), "{finding}");
    }

    /// When only one of `reason` and `detail` is present, the finding quotes
    /// that field alone — no dangling dash from a missing half.
    #[test]
    fn a_driver_ending_naming_one_field_says_that_field_alone() {
        for (name, event, said) in [
            (
                "speech-report-driver-reason-only",
                r#"{"ts_ms":5,"event":"brenn_driver_exited","reason":"panic"}"#,
                "panic",
            ),
            (
                "speech-report-driver-detail-only",
                r#"{"ts_ms":5,"event":"brenn_driver_exited","detail":"it panicked"}"#,
                "it panicked",
            ),
        ] {
            let (_dir, at) = records(name, &[STARTED, COMPOSED, BRAIN, ATTACHED, event]);
            let report = analyze(&Console::read(&at));
            let finding = report
                .findings
                .iter()
                .find(|finding| finding.contains("the bus driver died"))
                .expect("the driver finding");
            assert!(
                finding.contains(said) && !finding.contains(" — "),
                "the field that is there and no dangling dash: {finding}",
            );
        }
    }

    /// The skew posture again: an ending whose reason this build has no word
    /// for is read as a death, and the operator gets the event's key list where
    /// its own words are missing rather than an empty sentence.
    #[test]
    fn a_driver_ending_naming_nothing_fails_carrying_its_keys() {
        let died =
            r#"{"ts_ms":5,"event":"brenn_driver_exited","why":"panic","what":"it panicked"}"#;
        let (_dir, at) = records(
            "speech-report-driver-death-unread",
            &[STARTED, COMPOSED, BRAIN, ATTACHED, died],
        );
        let report = analyze(&Console::read(&at));
        let finding = report
            .findings
            .iter()
            .find(|finding| finding.contains("the bus driver died"))
            .expect("the driver finding");
        assert!(
            finding.contains("event,ts_ms,what,why"),
            "the keys stand in for the words: {finding}",
        );
    }

    /// Every orderly shutdown emits the same ending event, so the event's
    /// presence is not the finding — the field is. A teardown with no driver
    /// line at all is a healthy ending, this check says nothing about it, and
    /// the event still appears by name in the measured half.
    #[test]
    fn an_orderly_teardown_with_no_driver_line_is_not_a_finding() {
        let (_dir, at) = records(
            "speech-report-teardown-no-driver-line",
            &[STARTED, COMPOSED, BRAIN, ATTACHED, TEARDOWN],
        );
        let report = analyze(&Console::read(&at));
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("`brenn_bridge_exit` carrying")),
            "an event an operator can search for by name: {:?}",
            report.measured
        );
    }

    /// The incident's own shape end to end: the console text and the events
    /// glued together the way the launcher's one file interleaves them. Both
    /// triggers are in torn lines here, and the verdict has to find them.
    #[test]
    fn a_torn_console_is_read_the_same_as_a_clean_one() {
        let brain = concat!(
            "11:21:29.389 stt configured — http://s:8000 model=m lang=en",
            r#"{"ts_ms":1788261689389,"event":"brain_brenn","publish_channel":"c"}"#,
        );
        let exit = concat!(
            "11:21:30.074 !!! brenn_bridge_exit unexpected=true outcome=no wire version in common",
            r#"{"ts_ms":1788261690074,"event":"brenn_bridge_exit","unexpected":true,"#,
            r#""outcome":"no wire version in common: this bridge speaks 3..=3, the server speaks 4..=4"}"#,
        );
        let (_dir, at) = records("speech-report-torn", &[STARTED, COMPOSED, brain, exit]);
        let console = Console::read(&at);
        assert_eq!(console.noise, 2, "the console text ahead of each event");
        let report = analyze(&console);
        // One loss, one finding: the run never attached *because* the bridge
        // ended the way this line says it did.
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(
            report.findings[0].contains("ended mid-run"),
            "{:?}",
            report.findings
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("2 pipeline event(s)")),
            "the recovered events are counted as events: {:?}",
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

    /// The refusal tears like everything else on this console: it is written to
    /// the stream the events use, behind a console sentence that never got its
    /// newline. The most consequential line in the file must survive that.
    #[test]
    fn a_refusal_glued_onto_a_console_line_still_fails() {
        let torn = concat!(
            "11:21:28.900 speech pipeline composed — listening on 127.0.0.1:9",
            "reachy-host: the bus attachment could not negotiate a wire version",
        );
        let (_dir, at) = records("speech-report-refused-torn", &[STARTED, torn]);
        let report = analyze(&Console::read(&at));
        let finding = report
            .findings
            .iter()
            .find(|finding| finding.contains("refusing rather than draining"))
            .expect("the refusal finding");
        assert!(
            finding.contains("could not negotiate a wire version"),
            "the refusal's own words are the diagnosis: {finding}",
        );
    }

    /// A refusal with an event glued onto *its* end is one line, and the event
    /// behind it is not the run going on afterwards.
    #[test]
    fn an_event_glued_behind_a_refusal_does_not_clear_it() {
        let torn = concat!(
            "reachy-host: the speech configuration is unreadable",
            r#"{"ts_ms":9,"event":"pipeline_drained"}"#,
        );
        let (_dir, at) = records("speech-report-refused-then-event", &[STARTED, torn]);
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
                .any(|line| line.contains("×2 `wake` carrying event,score,ts")),
            "{:?}",
            report.measured
        );
    }

    /// The name is half the summary: two of this vocabulary's events carry the
    /// same three fields, and a count by shape alone would fold them into one
    /// anonymous line that says something died and not what.
    #[test]
    fn pipeline_events_of_one_shape_are_split_by_their_names() {
        let script = r#"{"ts_ms":9,"event":"script_task_exited","reason":"panic","detail":"d"}"#;
        let alert = r#"{"ts_ms":9,"event":"alert_task_exited","reason":"panic","detail":"d"}"#;
        let (_dir, at) = records(
            "speech-report-pipeline-kinds",
            &[STARTED, COMPOSED, script, alert],
        );
        let report = analyze(&Console::read(&at));
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("2 pipeline event(s) on this console, of 2 kind(s)")),
            "{:?}",
            report.measured
        );
        for name in ["script_task_exited", "alert_task_exited"] {
            assert!(
                report.measured.iter().any(|line| line
                    .contains(&format!("×1 `{name}` carrying detail,event,reason,ts_ms"))),
                "{name} by its own name: {:?}",
                report.measured
            );
        }
    }

    /// A pipeline object with no `event` at all is still counted, and the name
    /// it does not have leaves no empty quoting behind it.
    #[test]
    fn a_nameless_pipeline_event_is_summarized_without_a_name() {
        let nameless = r#"{"a":1,"b":2}"#;
        let (_dir, at) = records(
            "speech-report-pipeline-nameless",
            &[STARTED, COMPOSED, nameless],
        );
        let report = analyze(&Console::read(&at));
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("×1 carrying a,b") && !line.contains('`')),
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
        let classified = classify(r#"{"a":1,"b":2}"#);
        assert!(classified.ahead.is_none(), "a whole line is one thing");
        assert_eq!(
            classified.line,
            Line::Pipeline {
                event: String::new(),
                keys: "a,b".to_owned(),
            }
        );
    }

    /// A JSON scalar is not an event: the stream is objects, and anything else
    /// on the console is somebody's print.
    #[test]
    fn a_json_scalar_is_noise() {
        assert_eq!(
            classify("42").line,
            Line::Noise {
                text: "42".to_owned()
            }
        );
    }

    /// The stdout-and-stderr tear: a console sentence with an event glued onto
    /// its end, which is how six of one incident's events actually arrived.
    #[test]
    fn an_event_glued_onto_a_console_line_is_recovered() {
        let torn = concat!(
            "11:21:30.074 !!! brenn_bridge_exit unexpected=true outcome=no wire version",
            r#"{"ts_ms":1788261690074,"event":"brenn_bridge_exit","unexpected":true,"#,
            r#""outcome":"no wire version in common"}"#,
        );
        let classified = classify(torn);
        assert_eq!(
            classified.ahead.as_deref(),
            Some("11:21:30.074 !!! brenn_bridge_exit unexpected=true outcome=no wire version"),
            "the console text ahead of it is kept",
        );
        assert_eq!(
            classified.line,
            Line::Bridge {
                keys: "event,outcome,ts_ms,unexpected".to_owned(),
                event: "brenn_bridge_exit".to_owned(),
                unexpected: true,
                loss: "no wire version in common".to_owned(),
            }
        );
    }

    /// The bridge's own variant is the three events and nothing else: an event
    /// this tool has no schema for is the key summary it always was.
    #[test]
    fn an_event_that_is_not_the_bridge_s_is_a_summary() {
        let line = classify(r#"{"ts_ms":1,"event":"listening","addr":"127.0.0.1:9"}"#).line;
        assert_eq!(
            line,
            Line::Pipeline {
                event: "listening".to_owned(),
                keys: "addr,event,ts_ms".to_owned(),
            }
        );
    }

    /// The recovery is a parse and not a search: a sentence that merely quotes
    /// the spelling stays what it was.
    #[test]
    fn a_line_whose_suffix_does_not_parse_stays_noise() {
        let raw = r#"11:21:30.074 the writer names its events {"ts_ms": and then stops"#;
        assert_eq!(
            classify(raw).line,
            Line::Noise {
                text: raw.to_owned()
            }
        );
    }

    /// The console text a real event tears onto can hold that spelling itself —
    /// here a half-written event glued ahead of a whole one. Splitting at the
    /// first match alone would lose the complete event at the line's end, and a
    /// lost `brain_brenn` is a run that exempts itself from the bridge check.
    #[test]
    fn a_console_line_quoting_the_event_head_does_not_hide_the_event_behind_it() {
        let torn = concat!(
            r#"11:21:29.389 the writer names its events {"ts_ms": and then stops"#,
            r#"{"ts_ms":1788261689389,"event":"brain_brenn","publish_channel":"c"}"#,
        );
        let classified = classify(torn);
        assert_eq!(
            classified.ahead.as_deref(),
            Some(r#"11:21:29.389 the writer names its events {"ts_ms": and then stops"#),
            "everything ahead of the whole event is the console's",
        );
        assert_eq!(
            classified.line,
            Line::Bridge {
                keys: "event,publish_channel,ts_ms".to_owned(),
                event: "brain_brenn".to_owned(),
                unexpected: false,
                loss: String::new(),
            }
        );
    }

    /// The refusal is looked for where a tear puts it, not wherever the words
    /// appear: a sentence naming this host is not this host refusing. Reading
    /// one as an exit fails a good run and quotes words nobody said.
    #[test]
    fn a_console_line_merely_naming_the_host_is_not_a_refusal() {
        let named = "11:21:29 relaying to reachy-host: ready";
        let (_dir, at) = records(
            "speech-report-named-not-refused",
            &[STARTED, COMPOSED, named],
        );
        let report = analyze(&Console::read(&at));
        assert!(report.findings.is_empty(), "{:?}", report.findings);
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
