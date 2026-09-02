//! What one supervised speech run did, read off both halves of its fetch.
//!
//! The tool a speech run is judged by, beside `first_motion_report` and on the
//! same premise: it reads the fetched records and nothing else, so a run that
//! happened on a unit last week is read the same way as one that finished a
//! second ago. What differs is the input and the standard.
//!
//! The input is both sides of the fetch: the console for what was asked and how
//! the process fared, the channel log for what the machine did. The console
//! side is the voice host's own stdout — the edge's narration, the session's
//! story as the edge renders it, the alerts the edge's table raised, and, where
//! the site sends its pipeline's JSONL to stdout, the pipeline's own events
//! riding the same file. That file is
//! `<records>.console/voice_host_0.log`, written by the launcher on the unit
//! and pulled back beside the records. The log side is the run directory inside
//! `<records>` that the logger wrote, read over three channels: the session's
//! own story, the head's pose estimates, and the scripts that reached the
//! session's port. What a person said to the robot decides what is in either,
//! and where they say the same thing twice the log is the better witness — the
//! session republishes its whole story on its own channel, and the console
//! holds whatever of it the edge was there for.
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
//! It holds one more opinion, and it is about the motion path. A script the
//! pipeline authored is a script this host wrote for itself: the scripter's
//! decision, this process's gate, one loopback datagram. So a run in which this
//! host dropped its own scripts, or authored scripts the session accepted none
//! of, is a run in which nothing anybody said could move the head — a failure
//! whatever the voice half did, and the failure this tool exists to have caught.
//! A run in which nobody spoke authors nothing and is green.
//!
//! The log is where that opinion stops being about paperwork. A script the
//! session accepted is a script the machine was supposed to move for, so an
//! accepted script with no engagement behind it, or with a head that never left
//! the tolerance box it started in, is the same failure read off the machine
//! rather than off the narration — and an accepted script with no log at all is
//! a run whose central question these records cannot answer. Where nothing was
//! accepted, none of that is asked: the excursion is printed and the run is
//! green.
//!
//! Absence of evidence is never read as evidence, in either direction. A story
//! whose oldest rows fell off the session's ring is a story that says less than
//! the run did, so what is missing from it is said rather than failed on and
//! every count off it is a floor. A log with no pair of solvable head estimates
//! in it says nothing about where the head was, which is a different finding
//! from a head that sat still and points an operator somewhere else.
//!
//! An alert that was raised and lost is the same class of failure one level up:
//! a run whose reporting path did not carry its own bad news is a run whose
//! verdict cannot be trusted next time, so an alert handed off while an
//! attachment that granted none held the link is a finding. The grant travels
//! with each hand-off and not with the run: the bridge re-attaches after every
//! closed socket, and an alert given to a granting attachment travelled however
//! the ones before it answered. An ungranted attachment that carried nothing is
//! a note: that grant is the far side's configuration, and a verdict red on
//! every run stops carrying the signal it was read for.
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

use brenn_reachy__cogs__script_clk_rs::ScriptWire;
use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__driver__pose_clk_rs::PoseEstimateWire;
use brenn_reachy__motion__reports_clk_rs::{ReportKind, ReportKindWire};
use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, TimelineWire};
use log_read::{
    Bound, Census, Complaints, Logged, Streams, binding, cumulative, each, read_with, typed,
};
use motion_channels::{ESTIMATE_CHANNEL, REPORT_CHANNEL, SCRIPT_CHANNEL};
use motion_evidence::{ARRIVAL_OFFSET_M, ARRIVAL_TURN_RAD, Motion};
use motion_proto::DecodeError;
use reachy_edge::{
    CompileError, Origin, Refusal, Severity, UNKNOWN_KIND_PREFIX, origin_word, row_says, row_word,
    severity_word,
};
use reachy_host::{
    AWAITING_SPEECH_CONFIG, COMPOSED, REFUSAL_PREFIX, STARTED, UNOFFERED, UNPUBLISHED, UNSENT,
    UNSPOKEN, VOICELESS,
};
use reachy_motion::postures::neutral_targets;
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

/// The extension the online logger writes its records under.
///
/// A run directory inside the fetch is one that holds a non-empty file of these,
/// which is the fetch's own rule for what a run directory is (`tools/lib.sh`).
const OLOG_EXTENSION: &str = "olog";

/// The file the fetch leaves at the root of the records naming the build that
/// recorded them.
///
/// Not read, only named: a log is read by a build of the schemas it was written
/// under, so a binding this build refuses is answered by rebuilding at the
/// revision this file names.
const PROVENANCE: &str = "provenance.txt";

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

/// The pipeline event one authored motion script is announced by.
///
/// A speech-surface event name and not this tree's. What it announces is the
/// scripter having decided on a gesture and handed it to this host's own gate,
/// which is what makes the count of them the expectation everything else in the
/// motion path is read against: a run with none of these asked for no motion.
const MOTION_SCRIPT: &str = "motion_script";

/// The pipeline event an alert handed to the bus attachment is announced by.
///
/// Also speech-surface's: the alert seam's far side, and the only evidence in
/// this file that a raised alert was given to something to carry.
const ALERT_HANDED_OFF: &str = "alert_handed_off";

/// The three pipeline events the announcement seam's far side writes.
///
/// The other half of the speaking path: the host's `unspoken` line covers a
/// sentence the seam's queue refused, and nothing else. Everything after that
/// queue — no pod in the room, a playback path too far behind, a seam wired to
/// a run with no voice — is said here and nowhere in this tree, so a critical
/// alert's sentence can be lost with the host's stream showing a clean raise.
/// Counted and quoted as notes: which of them is a fault is the pipeline's
/// business, and the sentence is the join key back to the alert that said it.
const ANNOUNCEMENT_SPOKEN: &str = "announcement_spoken";
const ANNOUNCEMENT_UNHEARD: &str = "announcement_unheard";
const ANNOUNCE_SEAM_UNUSED: &str = "announce_seam_unused";

/// Which sink a body that never reached the gate came from, on its own line.
///
/// The scripter's is this process's own decision; the bus's is a remote
/// sender's. Only the first is this run's failure.
const SCRIPTER_SOURCE: &str = "scripter";

/// Where a pipeline event begins when one is glued onto a console line.
///
/// Every event the pipeline emits leads with its timestamp, which is what makes
/// the torn shape below recoverable rather than guessed at.
const EVENT_HEAD: &str = "{\"ts_ms\"";

/// How many non-JSON lines are quoted in the summary.
const NOISE_SAMPLE: usize = 3;

/// How much of a line is quoted where one is quoted.
const QUOTE_LIMIT: usize = 160;

/// The fields of an edge line the motion ledger reads by name.
///
/// Beside the kind and the sentence rather than inside them: a count of
/// refusals says a run had trouble, and which side authored the refused body is
/// the difference between this machine disagreeing with itself and two senders
/// disagreeing with each other. Every field is empty or false on the lines that
/// do not carry it, which is every kind but three.
#[derive(Debug, Default, PartialEq)]
struct EdgeFields {
    /// Where a refused body was authored: the edge's own `origin` word.
    origin: String,
    /// Which sink dropped a body before the gate saw it.
    source: String,
    /// Whether the composition said alerts ride the bus attachment.
    alerts: bool,
    /// Whether the composition said the robot says its criticals out loud.
    speaks: bool,
}

/// One alert the pipeline gave the bus attachment to carry, and what the
/// attachment holding it at that moment had granted.
///
/// The grant travels with the alert rather than being asked of the run,
/// because it is a property of one attachment and a run holds as many as the
/// link dropped: an alert handed to a granting attachment travelled, whatever
/// an earlier or later one said.
#[derive(Debug, PartialEq)]
struct HandedOff {
    title: String,
    severity: String,
    delivery: String,
    /// What the attachment ahead of it said, where one said anything.
    granted: Option<bool>,
}

/// One line of the host's console, as much as this tool understands of it.
#[derive(Debug, PartialEq)]
enum Line {
    /// A line on the edge's own stream.
    Edge {
        kind: String,
        says: String,
        fields: EdgeFields,
    },
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
    /// One of three variants that hold a schema for another repo's event
    /// stream, each grouped by what reads it: [`Line::Pipeline`] stays the
    /// summary every other event is. An event wanted by field is two edits —
    /// an arm in [`object`]'s one `match` on the event name, and an arm in the
    /// fold [`Console::absorb`] does over the variant it lands in.
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
        /// Whether this attachment granted alerts, on the one event that says.
        /// `None` on the other three, and on a pipeline that does not write the
        /// field as a boolean — an older one than this tool reads for.
        granted: Option<bool>,
    },
    /// A motion script the pipeline authored, read by field.
    ///
    /// The expectation the whole motion ledger is read against, which is why it
    /// is read by field rather than counted as a shape: what a run asked the
    /// head to do is the number every finding below is relative to.
    Authored {
        keys: String,
        event: String,
        /// Why the scripter decided on a gesture, in its own word.
        cause: String,
    },
    /// An alert the pipeline handed to the bus attachment, read by field.
    HandedOff {
        keys: String,
        event: String,
        title: String,
        severity: String,
        /// Whether the far side confirmed the frame.
        delivery: String,
    },
    /// One of the announcement seam's far-side events, read by field.
    ///
    /// The sentence is what makes this worth reading by field rather than
    /// counting as a shape: it is what joins a pipeline-side loss back to the
    /// critical alert whose words those were.
    Announced {
        keys: String,
        event: String,
        /// The sentence the line carried. Empty on the seam-unused event, which
        /// is about the run rather than about anything said.
        text: String,
        /// Why it was not said, in the pipeline's own word; or which half of a
        /// speaking run was missing. Empty on the spoken event.
        reason: String,
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
/// and the lines saying one did not reach the bus or the room — so nothing here
/// grows that the output would not.
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
    /// What each `unspoken` line said.
    ///
    /// Beside the unpublished ones and not folded into the per-kind tally: they
    /// are the two delivery legs of one alert, and a run whose critical never
    /// reached the person standing in front of the robot is exactly as much a
    /// failure of this run's reporting as one whose alert died on the bus.
    unspoken: Vec<String>,
    /// Every sentence the pipeline queued for playback, in the order it said
    /// them. The third leg of the same alert, past the seam this host hands to.
    spoken: Vec<String>,
    /// Sentences the pipeline did not say, per its own reason word: how many,
    /// and the last sentence lost to it.
    unheard: BTreeMap<String, (usize, String)>,
    /// Why a run holding the announcement seam could not speak through it, in
    /// the pipeline's words. Empty on a run that could.
    seam_unused: Vec<String>,
    /// The words the edge spells a dropped body with, taken once.
    ///
    /// Held rather than asked per line: the fold runs over every line of a
    /// session that ends when an operator ends it, and the words do not change
    /// inside one.
    refusals: Vec<&'static str>,
    /// How many motion scripts the pipeline authored, by the cause it named.
    authored: BTreeMap<String, usize>,
    /// The scripts this host authored and then dropped itself, per kind: how
    /// many, and the last sentence that kind said.
    ///
    /// A refusal of a locally authored body, a body a sink never got as far as
    /// the gate with, and a compiled script that never left the machine, all in
    /// one table: they are three points on one path, and the count an operator
    /// wants is how many of this run's own gestures died on it.
    own_refused: BTreeMap<String, (usize, String)>,
    /// How many bodies from off the bus the gate refused.
    ///
    /// Counted and never failed on: the intent channel is not assumed to carry
    /// one machine's traffic, so a remote sender disagreeing with this machine
    /// is news about the sender.
    remote_refused: usize,
    /// Accepted scripts that never reached the session's port and that this
    /// host did not author, by kind and last sentence.
    ///
    /// A failure of the same size as the one above it and a different fact
    /// about the run: this host sends every accepted script, so a send that
    /// failed says a gesture was lost without saying whose it was.
    unsent_elsewhere: BTreeMap<String, (usize, String)>,
    /// How many refused bodies carried no origin word at all.
    ///
    /// A console written before the edge spelled one. Read as a remote sender's
    /// business, which is the safe way round, and said so nobody reads a run
    /// this tool could not attribute as one it attributed.
    origin_unsaid: usize,
    /// Every alert the pipeline handed to the bus attachment, in the order the
    /// console carried them.
    handed_off: Vec<HandedOff>,
    /// What the most recent attachment said about the alert grant.
    ///
    /// The current answer and not a latch: the bridge re-attaches after every
    /// closed socket, so several attachments a run is the ordinary shape and
    /// what one of them granted says nothing about the next. Each hand-off is
    /// stamped with this as it is read.
    grant: Option<bool>,
    /// Whether the composition said alerts ride the bus attachment.
    composed_alerts: bool,
    /// Whether the composition said the robot says its criticals out loud.
    composed_speaks: bool,
    /// Whether any attachment said it does not grant alerts.
    ungranted: bool,
    /// Whether any attachment said nothing either way about the grant.
    ///
    /// An older pipeline than this tool reads for. Noted rather than failed on,
    /// the same skew tolerance the bridge check's two spellings have.
    grant_unsaid: bool,
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
            refusals: refusal_kinds(),
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
            Line::Edge { kind, says, fields } => {
                if kind == UNPUBLISHED {
                    self.unpublished.push(says.clone());
                }
                if kind == UNSPOKEN {
                    self.unspoken.push(says.clone());
                }
                if kind == COMPOSED {
                    self.composed_alerts = fields.alerts;
                    self.composed_speaks = fields.speaks;
                }
                if self.refusals.contains(&kind.as_str()) && fields.origin.is_empty() {
                    self.origin_unsaid += 1;
                }
                if self.own_authored(&kind, &fields) {
                    count(&mut self.own_refused, kind.clone(), says.clone());
                } else if kind == UNSENT {
                    count(&mut self.unsent_elsewhere, kind.clone(), says.clone());
                } else if self.refusals.contains(&kind.as_str()) {
                    self.remote_refused += 1;
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
                granted,
            } => {
                match event.as_str() {
                    BRAIN_BRENN => self.brain_brenn = true,
                    BRENN_ATTACHED => {
                        self.attached = true;
                        self.grant = granted;
                        match granted {
                            Some(false) => self.ungranted = true,
                            Some(true) => {}
                            None => self.grant_unsaid = true,
                        }
                    }
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
            Line::Authored { keys, event, cause } => {
                *self.authored.entry(cause).or_default() += 1;
                *self.pipeline.entry((event, keys)).or_default() += 1;
            }
            Line::Announced {
                keys,
                event,
                text,
                reason,
            } => {
                match event.as_str() {
                    ANNOUNCEMENT_SPOKEN => self.spoken.push(text),
                    ANNOUNCEMENT_UNHEARD => {
                        let lost = self.unheard.entry(reason).or_default();
                        lost.0 += 1;
                        lost.1 = text;
                    }
                    // The event is its own reason: a seam handed to a run that
                    // has no way to say anything.
                    _ => self.seam_unused.push(reason),
                }
                *self.pipeline.entry((event, keys)).or_default() += 1;
            }
            Line::HandedOff {
                keys,
                event,
                title,
                severity,
                delivery,
            } => {
                self.handed_off.push(HandedOff {
                    title,
                    severity,
                    delivery,
                    granted: self.grant,
                });
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

    /// Whether this line is a script this host wrote for itself and lost.
    ///
    /// Three shapes, and the origin word is what separates each from a remote
    /// sender's body: a refusal of a locally authored script, a body the
    /// scripter's own sink could not hand to the gate, and a locally authored
    /// script that compiled and never reached the session's port. This host
    /// sends every accepted script, including one a remote sender wrote, so the
    /// send failure alone does not say whose gesture was lost.
    fn own_authored(&self, kind: &str, fields: &EdgeFields) -> bool {
        let local = fields.origin == origin_word(Origin::Local);
        (self.refusals.contains(&kind) && local)
            || (kind == UNOFFERED && fields.source == SCRIPTER_SOURCE)
            || (kind == UNSENT && local)
    }

    /// How many scripts this host authored and lost, across the three shapes.
    fn own_lost(&self) -> usize {
        self.own_refused.values().map(|(count, _)| count).sum()
    }

    /// How many motion scripts the pipeline authored, whatever its causes.
    fn authored(&self) -> usize {
        self.authored.values().sum()
    }
}

/// What the run's channel log held, over the three channels this tool reads.
///
/// Three and not the twelve `first_motion_report` binds: a speech run is not a
/// wake gesture, and the questions here are what the session said about the
/// scripts that reached it, where the head was while it said so, and whether
/// anything reached its port at all. Binding a channel this run has no question
/// about would buy a byte-equal refusal of a log for a schema nobody read.
struct Log {
    /// The rows of the newest story the session published, which is the whole
    /// of its narration: cumulative, so a logger that attached late still holds
    /// all of it.
    reports: Vec<Logged<TimelineEntryWire>>,
    /// How many rows the session dropped off the front of that story to make
    /// room for later ones, all run.
    ///
    /// The story is a ring, so "cumulative" means the whole of what the session
    /// still holds and not the whole of what it ever said. This number is the
    /// difference, and every count taken off the rows is a lower bound while it
    /// is non-zero.
    dropped: u32,
    /// What the head did, folded off the estimate stream as it was read.
    ///
    /// A fold and not the samples, for the reason [`Console`] is one: the
    /// channel carries a message per control cycle over a session that ends
    /// when an operator ends it, and the report's two questions of it are
    /// running maxima. Holding the stream would make the longest sessions the
    /// ones this tool cannot run over.
    motion: Motion,
    /// The scripts that reached the session's port.
    scripts: Vec<Logged<ScriptWire>>,
    /// Every channel the log carries and how much of each: a channel with no
    /// type bound here still says whether anything travelled on it.
    census: Census,
    /// Anything that went wrong reading the log. Each of these means the
    /// decoded streams above cannot be trusted, so a log with any of them is
    /// read as no log at all.
    complaints: Complaints,
}

/// Hand written rather than derived because the motion fold has to be told
/// which posture it measures an approach against, and that is `up` — the
/// posture both of the scripter's causes begin at, and the only one this tool
/// knows a speech run was ever sent to.
impl Default for Log {
    fn default() -> Self {
        Self {
            reports: Vec::new(),
            dropped: 0,
            motion: Motion::towards(&neutral_targets()),
            scripts: Vec::new(),
            census: Census::default(),
            complaints: Complaints::default(),
        }
    }
}

impl Streams for Log {
    fn census(&mut self) -> &mut Census {
        &mut self.census
    }

    fn complaints(&mut self) -> &mut Complaints {
        &mut self.complaints
    }
}

/// The three channels this tool binds, checked before anything is decoded.
///
/// Bindings are strict, as everywhere: a log recorded under other schemas than
/// this build's is refused rather than read approximately, because a payload of
/// the right size is not the right message and a report about a machine read
/// that way is nonsense. Reading such a log means building this tool at the
/// revision `provenance.txt` names.
const CHANNELS: [Bound<Log>; 3] = [
    Bound {
        name: REPORT_CHANNEL,
        check: binding::<TimelineWire>,
        route: |log, message| {
            let Log {
                reports,
                dropped,
                complaints,
                ..
            } = log;
            cumulative(
                message,
                reports,
                complaints,
                |story: &TimelineWire, rows| {
                    rows.extend(story.entries().iter().cloned());
                    *dropped = story.dropped();
                },
            );
        },
    },
    Bound {
        name: ESTIMATE_CHANNEL,
        check: binding::<PoseEstimateWire>,
        route: |log, message| {
            let Log {
                motion, complaints, ..
            } = log;
            each::<PoseEstimateWire>(message, complaints, |logged| {
                motion.estimate(logged.at_ns, &logged.message);
            });
        },
    },
    Bound {
        name: SCRIPT_CHANNEL,
        check: binding::<ScriptWire>,
        route: |log, message| typed(message, &mut log.scripts, &mut log.complaints),
    },
];

impl Log {
    /// The rows of the story of one kind, oldest first.
    fn rows(&self, kind: ReportKind) -> Vec<&TimelineEntryWire> {
        let wanted = ReportKindWire::from(kind);
        self.reports
            .iter()
            .map(|logged| &logged.message)
            .filter(|row| row.kind() == wanted)
            .collect()
    }

    /// Whether the session ever took the machine: engaged, and then went
    /// active.
    ///
    /// Both, in that order, off the phase changes the story narrates. A session
    /// that reached `ENGAGING` and stopped there commissioned the machine and
    /// never ran a schedule on it, which is a head that did not move for a
    /// reason the estimates cannot show.
    ///
    /// [`Took::NotInTheStory`] where the pair is absent from a story rows have
    /// fallen off the front of: a long run's engagement is the oldest thing it
    /// narrated and the first thing the ring drops, so absence there says
    /// nothing about the session and is not something to fail a run on.
    fn took_the_machine(&self) -> Took {
        let mut engaged = false;
        for row in self.rows(ReportKind::PhaseChanged) {
            let to = SessionPhaseWire(u8::try_from(row.a()).unwrap_or(u8::MAX));
            if to == SessionPhaseWire::ENGAGING {
                engaged = true;
            } else if engaged && to == SessionPhaseWire::ACTIVE {
                return Took::Yes;
            }
        }
        if self.dropped > 0 {
            Took::NotInTheStory
        } else {
            Took::No
        }
    }

    /// Whether rows have fallen off the front of the story this log holds.
    const fn truncated(&self) -> bool {
        self.dropped > 0
    }
}

/// What the session's surviving story says about it taking the machine.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Took {
    /// It engaged and then went active.
    Yes,
    /// It did not, over a story that holds everything it ever said.
    No,
    /// It did not over the rows that survived the ring, which held more.
    NotInTheStory,
}

/// The channel-log side of a fetch: which run directories it holds, and what
/// the newest of them said.
///
/// The newest by sort, which is the fetch's own rule for a run directory
/// (`run_directory` in `tools/lib.sh`) rather than a second one invented here:
/// a fetch the motion-run path reads cleanly is read the same way. A directory
/// with no non-empty record file in it is not a run directory at all, so a
/// logger that started and wrote nothing reads as no log rather than as an
/// empty run.
#[derive(Default)]
struct Records {
    /// Where the fetch itself is.
    at: PathBuf,
    /// Every run directory it holds, in sort order: the last is the newest.
    found: Vec<PathBuf>,
    /// What the newest one held, where it opened at all.
    read: Option<Log>,
    /// Why it did not open, where it did not.
    unopened: Option<String>,
    /// Directories this process could not list, and what the system said.
    ///
    /// Kept rather than folded into "nothing here", because a fetch this
    /// process cannot read is a third cause beside a logger that wrote nothing
    /// and a fetch that came back without one, and it is the only one of the
    /// three an operator fixes on the workstation.
    unlisted: Vec<String>,
}

impl Records {
    /// Read the newest run directory inside a fetch.
    ///
    /// A log that will not open is not an error of this tool's: it is what the
    /// findings below are read against, so a fetch that came back without a
    /// readable log still prints a report.
    fn of(records: &Path) -> Self {
        let (found, unlisted) = run_directories(records);
        let mut held = Self {
            at: records.to_path_buf(),
            found,
            unlisted,
            ..Self::default()
        };
        let Some(newest) = held.found.last() else {
            return held;
        };
        match read_with::<Log>(newest, &CHANNELS) {
            Ok(log) => held.read = Some(log),
            Err(err) => held.unopened = Some(err.to_string()),
        }
        held
    }

    /// The log this run is judged by, where there is one worth decoding.
    ///
    /// A log with a complaint against any of its bindings is not one: the
    /// complaint says this build's schemas are not the ones that wrote it, so
    /// its decoded streams are bytes read as the wrong message rather than
    /// evidence about a machine.
    fn readable(&self) -> Option<&Log> {
        self.read.as_ref().filter(|log| log.complaints.is_empty())
    }

    /// Which run directory was read, where one was.
    fn newest(&self) -> Option<&PathBuf> {
        self.found.last()
    }

    /// What this process could not list, where anything defeated it.
    fn unlisted_says(&self) -> String {
        if self.unlisted.is_empty() {
            return String::new();
        }
        format!(
            "this process could not list {}: {}",
            if self.unlisted.len() == 1 {
                "one directory".to_owned()
            } else {
                format!("{} directories", self.unlisted.len())
            },
            self.unlisted
                .iter()
                .map(|says| quote(says))
                .collect::<Vec<_>>()
                .join("; ")
        )
    }

    /// Why there is no log to read the head off, in one sentence.
    ///
    /// Four ways to have none, and they are four different things for an
    /// operator to do: no records came back, something in the fetch would not
    /// list, the records would not open at all, or they opened and what came
    /// out is not trustworthy — another build's schemas, or a file the reader
    /// could not walk. The last two are one sentence because the complaint the
    /// reader wrote is what tells them apart, and it is quoted.
    fn why(&self) -> String {
        let Some(newest) = self.newest() else {
            return format!(
                "{} holds no run directory with a non-empty `.{OLOG_EXTENSION}` in it, so the \
                 logger recorded nothing this run or the fetch came back without it{}",
                self.at.display(),
                if self.unlisted.is_empty() {
                    String::new()
                } else {
                    format!(". {}", self.unlisted_says())
                }
            );
        };
        if let Some(err) = &self.unopened {
            return format!(
                "{} would not open: {}",
                quote(&newest.to_string_lossy()),
                quote(err)
            );
        }
        let complaints = self
            .read
            .as_ref()
            .map(|log| log.complaints.join("; "))
            .unwrap_or_default();
        format!(
            "{} did not read as a log this build can trust — {}. read it with the build \
             {PROVENANCE} names",
            quote(&newest.to_string_lossy()),
            quote(&complaints)
        )
    }
}

/// Every run directory inside a fetch, in sort order.
///
/// A directory directly under the fetch holding a non-empty record file, which
/// is what the fetch's own rule counts. A directory this process cannot list is
/// not a run directory either, but it is said rather than silently absent: it
/// is a fetch or a filesystem an operator can fix, and reading it as a logger
/// that recorded nothing would point them at the machine instead.
fn run_directories(records: &Path) -> (Vec<PathBuf>, Vec<String>) {
    let mut unlisted = Vec::new();
    let entries = match std::fs::read_dir(records) {
        Ok(entries) => entries,
        Err(err) => {
            return (Vec::new(), vec![format!("{}: {err}", records.display())]);
        }
    };
    let mut found: Vec<PathBuf> = entries
        .flatten()
        .map(|entry| entry.path())
        .filter(|path| {
            path.is_dir()
                && match holds_records(path) {
                    Ok(holds) => holds,
                    Err(err) => {
                        unlisted.push(format!("{}: {err}", path.display()));
                        false
                    }
                }
        })
        .collect();
    found.sort();
    (found, unlisted)
}

/// Whether a directory holds a record file with anything in it.
///
/// Non-empty, because a logger that created its file and wrote nothing leaves
/// one of zero bytes behind and a report over it would say the head sat still
/// rather than that nothing was recorded. A file whose size will not answer is
/// counted as no record, on the same grounds: an unmeasurable file is not
/// evidence about a machine.
fn holds_records(dir: &Path) -> Result<bool, std::io::Error> {
    let entries = std::fs::read_dir(dir)?;
    Ok(entries.flatten().any(|entry| {
        entry
            .path()
            .extension()
            .is_some_and(|extension| extension == OLOG_EXTENSION)
            && entry.metadata().is_ok_and(|facts| facts.len() > 0)
    }))
}

/// What the session did with the scripts that reached it, and where that was
/// read.
///
/// A reading rather than a pair of counters, because there are two places the
/// same fact is written and they are not equally good: the session republishes
/// its whole story on its own channel, and the edge renders whatever of that
/// story it was there for. The console's rows are the fallback.
struct Session {
    /// How many scripts the session accepted.
    accepted: usize,
    /// How many it refused.
    refused: usize,
    /// The last thing a refusal row said, for the reason words in it.
    refusal_says: String,
    /// Where these numbers came from, for the operator reading them.
    from: &'static str,
    /// Whether the two counts are floors rather than totals.
    ///
    /// Either account can be one: the log holds a ring the session drops the
    /// oldest rows off, and the console holds whatever of the narration the
    /// edge was running for. A floor is still what the findings are read
    /// against — a script the account does not hold is one this tool cannot
    /// judge — but it is not a number to print as a total.
    at_least: bool,
}

impl Session {
    /// What the session did, off the better of the two accounts.
    ///
    /// The log where there is one: the session republishes its whole story on
    /// its own channel, so a log holds all of it however late the logger
    /// attached, while the console holds whatever of it the edge was there for.
    /// A run whose narration never reached the console is still read here.
    fn of(console: &Console, records: &Records) -> Self {
        match records.readable() {
            Some(log) => Self::told(log),
            None => Self::rendered(console),
        }
    }

    /// The session's story as it published it, off the channel log.
    fn told(log: &Log) -> Self {
        let refused = log.rows(ReportKind::ScriptRefused);
        Self {
            accepted: log.rows(ReportKind::ScriptAccepted).len(),
            refused: refused.len(),
            refusal_says: refused.last().map(|row| row_says(row)).unwrap_or_default(),
            from: "the session's own story in the channel log",
            at_least: log.truncated(),
        }
    }

    /// The session's story as the edge rendered it onto the console.
    fn rendered(console: &Console) -> Self {
        let rows = |kind| {
            console
                .rows
                .get(kind)
                .cloned()
                .unwrap_or_else(|| (0, String::new()))
        };
        let (accepted, _) = rows(row_word(ReportKind::ScriptAccepted));
        let (refused, refusal_says) = rows(row_word(ReportKind::ScriptRefused));
        Self {
            accepted,
            refused,
            refusal_says,
            from: "the session's story as the console renders it",
            at_least: false,
        }
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
            fields: EdgeFields {
                origin: text("origin"),
                source: text("source"),
                alerts: object
                    .get("alerts")
                    .and_then(Value::as_bool)
                    .unwrap_or_default(),
                speaks: object
                    .get("speaks")
                    .and_then(Value::as_bool)
                    .unwrap_or_default(),
            },
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
            let bridge = |unexpected, loss, granted| Line::Bridge {
                keys: keys.clone(),
                event: event.clone(),
                unexpected,
                loss,
                granted,
            };
            match event.as_str() {
                BRAIN_BRENN => bridge(false, String::new(), None),
                BRENN_ATTACHED => bridge(
                    false,
                    String::new(),
                    object.get("alert_granted").and_then(Value::as_bool),
                ),
                BRENN_BRIDGE_EXIT => bridge(unexpected(&object), text("outcome"), None),
                BRENN_DRIVER_EXITED => {
                    bridge(true, died_of(&text("reason"), &text("detail"), &keys), None)
                }
                MOTION_SCRIPT => Line::Authored {
                    keys,
                    event,
                    cause: text("cause"),
                },
                ALERT_HANDED_OFF => Line::HandedOff {
                    keys,
                    event,
                    title: text("title"),
                    severity: text("severity"),
                    delivery: text("delivery"),
                },
                ANNOUNCEMENT_SPOKEN | ANNOUNCEMENT_UNHEARD | ANNOUNCE_SEAM_UNUSED => {
                    Line::Announced {
                        keys,
                        text: text("text"),
                        reason: text("reason"),
                        event,
                    }
                }
                _ => Line::Pipeline { event, keys },
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
        report.fail(format!(
            "a critical alert fired: `{}` — {}",
            quote(title),
            quote(says)
        ));
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

/// What this run asked the head to do, and what became of it.
///
/// Three findings, in the ladder an operator diagnoses by. The first is the
/// scripts this host lost itself: the wake word's path to the motors is a
/// decision made in this process and screened in this process, so a body
/// dropped on it is this machine disagreeing with itself and no retry
/// anywhere. The second is the session declining what did reach it. The third
/// is the outcome independent of either account — scripts authored and none
/// accepted — so a run whose narration was lost still fails.
///
/// The counts are printed whatever the verdict, because a run in which nobody
/// spoke asks for no motion and is a good run: the permissive standard the rest
/// of this tool holds to.
fn the_motion_path(console: &Console, session: &Session, records: &Records, report: &mut Report) {
    let authored = console.authored();
    report.note(format!(
        "{authored} motion script(s) authored by the pipeline{}",
        if console.authored.is_empty() {
            ": nothing this run said asked the head to move".to_owned()
        } else {
            format!(
                ", by cause: {}",
                console
                    .authored
                    .iter()
                    .map(|(cause, count)| format!("`{}` ×{count}", quote(cause)))
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        }
    ));
    report.note(format!(
        "{}{} accepted and {} refused, read from {}",
        if session.at_least { "at least " } else { "" },
        session.accepted,
        session.refused,
        session.from
    ));
    // Both accounts, where both exist: the log is the one judged on, and a
    // console that says something else is a narration the edge missed part of
    // rather than a second verdict. A truncated story is not compared at all —
    // the two numbers are counted over different windows, and "disagrees" would
    // be this tool's own arithmetic reported as news about the run.
    if let Some(log) = records.readable() {
        let rendered = Session::rendered(console);
        if log.truncated() {
            report.note(format!(
                "the console renders {} accepted and {} refused; the log's story has dropped {} \
                 row(s) off its front, so the two are counted over different windows and are not \
                 compared",
                rendered.accepted, rendered.refused, log.dropped
            ));
        } else {
            report.note(format!(
                "the console renders {} accepted and {} refused, which {} the log",
                rendered.accepted,
                rendered.refused,
                if (rendered.accepted, rendered.refused) == (session.accepted, session.refused) {
                    "agrees with"
                } else {
                    "disagrees with"
                }
            ));
        }
    }
    report.note(format!(
        "{} body(ies) off the bus the gate refused: a remote sender's business, not this run's",
        console.remote_refused
    ));
    if console.origin_unsaid > 0 {
        report.note(format!(
            "{} refused body(ies) carried no origin word: a console written before the edge \
             spelled one, so which side authored them is not in these records and they are \
             counted above as a remote sender's",
            console.origin_unsaid
        ));
    }
    let lost = console.own_lost();
    if lost > 0 {
        // "Bodies this host offered" and not "scripts the pipeline authored":
        // the scripter re-offers an unchanged decision without announcing it as
        // a new script, so more bodies can meet the gate than there are
        // `motion_script` events, and the two numbers in this report have to be
        // countable against each other.
        report.fail(format!(
            "{lost} of the bodies this host offered to its own gate never reached the session: \
             {}. the scripter's decisions meet this host's own gate, {}, and nothing is retried",
            console
                .own_refused
                .iter()
                .map(|(kind, (count, says))| format!(
                    "`{}` ×{count} — {}",
                    quote(kind),
                    quote(says)
                ))
                .collect::<Vec<_>>()
                .join("; "),
            if session.accepted == 0 {
                "so nothing said to the robot moved its head"
            } else {
                "so this much of what was said to the robot never reached it"
            }
        ));
    }
    let unsent: usize = console
        .unsent_elsewhere
        .values()
        .map(|(count, _)| count)
        .sum();
    if unsent > 0 {
        report.fail(format!(
            "{unsent} accepted script(s) compiled and never reached the session's port: {}. this \
             host sends every script the gate accepted, whoever wrote it, so what was lost here \
             is not attributed to a side",
            console
                .unsent_elsewhere
                .iter()
                .map(|(kind, (count, says))| format!(
                    "`{}` ×{count} — {}",
                    quote(kind),
                    quote(says)
                ))
                .collect::<Vec<_>>()
                .join("; ")
        ));
    }
    if session.refused > 0 {
        report.fail(format!(
            "the session refused {} script(s): {}",
            session.refused,
            quote(&session.refusal_says)
        ));
    }
    if authored > 0 && session.accepted == 0 {
        report.fail(format!(
            "the pipeline authored {authored} motion script(s) and the session accepted none: \
             whatever the head did this run, it was not what anybody said to the robot"
        ));
    }
}

/// Did the alerts this run raised have anything to travel on.
///
/// The grant is the far side's configuration and this tool does not judge it:
/// an attachment without it on a run that raised nothing is a note. What fails
/// is an alert actually handed to it, because the peer answers such a frame by
/// closing the socket — the alert is lost, and a run whose reporting path
/// swallowed its own bad news cannot be read as a verdict about anything else.
fn alerts_travelled(console: &Console, report: &mut Report) {
    report.note(format!(
        "{} alert(s) handed to the bus attachment",
        console.handed_off.len()
    ));
    for alert in &console.handed_off {
        report.note(format!(
            "  {} `{}` — delivery {}{}",
            quote(&alert.severity),
            quote(&alert.title),
            quote(&alert.delivery),
            match alert.granted {
                Some(true) => "",
                Some(false) => ", to an attachment that granted no alerts",
                None => ", to an attachment that said nothing about the grant",
            }
        ));
    }
    if console.grant_unsaid {
        report.note(
            "an attachment said nothing about the alert grant: a pipeline older than this tool \
             reads for, so whether its alerts could travel is not in these records",
        );
    }
    // Per hand-off and not per run: the bridge re-attaches after every closed
    // socket, so an alert given to a granting attachment travelled however the
    // attachments before or after it answered.
    let lost: Vec<&HandedOff> = console
        .handed_off
        .iter()
        .filter(|alert| alert.granted == Some(false))
        .collect();
    if !console.ungranted {
        return;
    }
    if lost.is_empty() {
        report.note(
            "an attachment did not grant alerts; nothing was handed to it while it held the \
             link, so this run lost nothing to it",
        );
        return;
    }
    if !console.composed_alerts {
        report.note(
            "an attachment did not grant alerts, and this host composed with its alerts as \
             narration only: what was handed off is the pipeline's own half to account for",
        );
        return;
    }
    report.fail(format!(
        "{} alert(s) this run raised were handed off to an attachment that did not grant alerts; \
         the bus answers such a frame by closing the socket, so each of them was lost: {}",
        lost.len(),
        lost.iter()
            .map(|alert| format!("`{}`", quote(&alert.title)))
            .collect::<Vec<_>>()
            .join(", ")
    ));
}

/// Did the head move for the scripts the session took.
///
/// The question the console cannot answer, and the one a speech run is
/// ultimately read for: a pipeline that came up, composed, authored a gesture
/// and had it accepted has done everything the console can show, and the
/// machine may still have sat perfectly still. Three findings, and each needs
/// an accepted script behind it — a run that asked for no motion is a run whose
/// stillness is correct.
///
/// The excursion is measured from the run's own first solved estimate rather
/// than from a posture: the question is whether the machine moved, and a head
/// commanded nowhere sits wherever the last run left it. A departure past
/// either tolerance is movement; a run inside both left the box its own
/// arrival check would have called one pose.
fn the_head_moved(records: &Records, session: &Session, report: &mut Report) {
    let Some(log) = records.readable() else {
        if session.accepted > 0 {
            report.fail(format!(
                "{} script(s) were accepted and there is no channel log to show whether the head \
                 moved: {}",
                session.accepted,
                records.why()
            ));
        } else {
            report.note(format!(
                "no channel log to read the head's own account from: {}. nothing was accepted \
                 this run, so there was nothing for it to show",
                records.why()
            ));
        }
        return;
    };
    let excursion = log.motion.excursion();
    report.note(format!(
        "the head's largest excursion over the run was {:.4} m and {:.4} rad, at {}, over {} \
         solved estimate(s) of {} recorded",
        excursion.offset_m,
        excursion.turn_rad,
        excursion.at,
        log.motion.solved(),
        log.motion.seen()
    ));
    // Against `up` because both of the scripter's causes begin there: how near
    // the head got to it says whether a run that did move went where the
    // pipeline sends it. Printed and never failed on — this tool does not know
    // what the last step of a gesture asked for, and a bus overlay is free to
    // end anywhere.
    if let Some(near) = log.motion.approach() {
        report.note(format!(
            "closest to `up`, the posture both scripter causes begin at: {:.4} m and {:.4} rad \
             away, at {}",
            near.offset_m, near.turn_rad, near.at
        ));
    }
    if session.accepted == 0 {
        return;
    }
    match log.took_the_machine() {
        Took::Yes => {}
        Took::No => report.fail(format!(
            "{} script(s) were accepted and the session never took the machine: its story \
             narrates no engagement that reached `active`, so no schedule ever ran",
            session.accepted
        )),
        // The engagement is the oldest thing a session narrates and the first
        // row its ring drops, so its absence from a truncated story is this
        // tool reaching the end of the evidence rather than news about a run.
        Took::NotInTheStory => report.note(format!(
            "the session's surviving story narrates no engagement that reached `active`, but it \
             has dropped {} row(s) off its front: whether it took the machine is not in these \
             records",
            log.dropped
        )),
    }
    // Two solved estimates is the least that can show a departure, and the
    // absence of them is a different fact from a head that sat still: a run
    // whose estimates never solved says nothing about where the head was, and
    // reporting that as stillness points an operator at the machine instead of
    // at the recording.
    if log.motion.solved() < 2 {
        report.fail(format!(
            "{} script(s) were accepted and the log holds no pair of solvable head estimates to \
             show whether it moved: {} solved of {} recorded",
            session.accepted,
            log.motion.solved(),
            log.motion.seen()
        ));
        return;
    }
    if excursion.offset_m <= ARRIVAL_OFFSET_M && excursion.turn_rad <= ARRIVAL_TURN_RAD {
        report.fail(format!(
            "the head never moved: its largest excursion over the run was {:.4} m and {:.4} rad, \
             inside the {ARRIVAL_OFFSET_M} m and {ARRIVAL_TURN_RAD} rad an arrival is judged by, \
             and {} script(s) were accepted",
            excursion.offset_m, excursion.turn_rad, session.accepted
        ));
    }
}

/// What the fetch's records held, whatever was made of it.
///
/// The census is the difference between a channel that was silent and one this
/// run never had: an operator reading a still head asks which, and a count of
/// zero on a channel the log declares is a different fact from a channel the
/// log never names.
fn the_log(records: &Records, report: &mut Report) {
    match records.newest() {
        None => report.note(format!(
            "no run directory in {}: this fetch carries no channel log",
            records.at.display()
        )),
        Some(newest) => {
            report.note(format!(
                "channel log read from {}",
                quote(&newest.to_string_lossy())
            ));
            if records.found.len() > 1 {
                report.note(format!(
                    "{} run directories in this fetch; the newest by sort was read, as the \
                     motion-run path reads one: {}",
                    records.found.len(),
                    records
                        .found
                        .iter()
                        .filter_map(|dir| dir.file_name())
                        .map(|name| quote(&name.to_string_lossy()))
                        .collect::<Vec<_>>()
                        .join(", ")
                ));
            }
        }
    }
    if !records.unlisted.is_empty() {
        report.note(records.unlisted_says());
    }
    let Some(log) = &records.read else {
        if records.unopened.is_some() {
            report.note(records.why());
        }
        return;
    };
    for channel in &log.census {
        report.note(format!(
            "  {} ×{}{}",
            channel.name,
            channel.count,
            match channel.first_seq {
                Some(0) | None => String::new(),
                Some(first) => format!(", the log's copy starting at the publisher's {first}"),
            }
        ));
    }
    report.note(format!(
        "{} script(s) reached the session's port",
        log.scripts.len()
    ));
    if log.truncated() {
        report.note(format!(
            "the session dropped {} row(s) off the front of its story: it narrates in a ring, so \
             every count taken off these rows is a floor and the oldest of the run is not here",
            log.dropped
        ));
    }
    if !log.complaints.is_empty() {
        report.note(records.why());
    }
}

/// Counts per kind, with the sentence each kind said.
fn kinds(console: &Console, report: &mut Report) {
    for (kind, (count, says)) in &console.edges {
        report.note(format!("edge {} ×{count}: {}", quote(kind), quote(says)));
    }
    for (kind, (count, says)) in &console.rows {
        report.note(format!(
            "timeline {} ×{count}: {}",
            quote(kind),
            quote(says)
        ));
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
            unnamed
                .iter()
                .map(|kind| quote(kind))
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
}

/// The alerts that did not fail the run, and the ones that never travelled.
fn alerts(console: &Console, report: &mut Report) {
    report.note(format!("{} warning alert(s)", console.warning.len()));
    for (title, says) in &console.warning {
        report.note(format!("  warning `{}` — {}", quote(title), quote(says)));
    }
    // An alert that did not travel is worth a human's eye and not a failure:
    // the condition it carried is in the line above it, and a critical among
    // them already fails on its own line.
    report.note(format!(
        "{} alert(s) raised that nothing carried to the bus",
        console.unpublished.len()
    ));
    for says in &console.unpublished {
        report.note(format!("  {UNPUBLISHED} — {}", quote(says)));
    }
    // The other delivery leg, read the same way: an alert with a sentence and a
    // speaker installed that the queue would not take is a critical nobody in
    // the room heard.
    report.note(format!(
        "{} alert(s) whose sentence the robot was asked to say and did not",
        console.unspoken.len()
    ));
    for says in &console.unspoken {
        report.note(format!("  {UNSPOKEN} — {}", quote(says)));
    }
    // Past the seam, where this host's stream stops: the pipeline's own account
    // of what became of each sentence. A run whose criticals were all raised
    // and all taken by the queue can still have said none of them out loud, and
    // these are the only lines that say so.
    report.note(format!(
        "{} sentence(s) the pipeline queued for playback",
        console.spoken.len()
    ));
    for says in &console.spoken {
        report.note(format!("  {ANNOUNCEMENT_SPOKEN} — {}", quote(says)));
    }
    let unheard: usize = console.unheard.values().map(|(count, _)| count).sum();
    report.note(format!(
        "{unheard} sentence(s) the pipeline had and did not say"
    ));
    for (reason, (count, says)) in &console.unheard {
        report.note(format!(
            "  ×{count} {ANNOUNCEMENT_UNHEARD} `{}` — {}",
            quote(reason),
            quote(says)
        ));
    }
    for reason in &console.seam_unused {
        report.note(format!(
            "the pipeline was handed an announcement seam it could not speak through: `{}`",
            quote(reason)
        ));
    }
    if !console.composed_speaks && !console.critical.is_empty() {
        report.note(format!(
            "{} critical alert(s) were raised on a host composed with no voice: the robot could \
             not say them, so nobody in the room was told",
            console.critical.len()
        ));
    }
    for (severity, count) in &console.unworded {
        report.note(format!(
            "×{count} alert(s) at severity `{}`, which this build has no word for",
            quote(severity)
        ));
    }
}

/// What else was in the file: foreign streams, the pipeline's own events, noise.
fn the_rest(console: &Console, report: &mut Report) {
    for (stream, count) in &console.foreign {
        report.note(format!(
            "×{count} on stream `{}`, which this build has no reader for",
            quote(stream)
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
                report.note(format!("  ×{count} carrying {}", quote(keys)));
            } else {
                report.note(format!(
                    "  ×{count} `{}` carrying {}",
                    quote(event),
                    quote(keys)
                ));
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

/// The whole reading of one fetch: the console, and the records beside it.
///
/// The session's reading is taken once and handed to both halves, so the ledger
/// and the head's own account are judged against the same numbers.
fn analyze(console: &Console, records: &Records) -> Report {
    let mut report = Report::default();
    let session = Session::of(console, records);
    came_up(console, &mut report);
    criticals(console, &mut report);
    bridged(console, &mut report);
    ended(console, &mut report);
    the_motion_path(console, &session, records, &mut report);
    the_head_moved(records, &session, &mut report);
    alerts_travelled(console, &mut report);
    kinds(console, &mut report);
    alerts(console, &mut report);
    the_log(records, &mut report);
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
    let at = Path::new(&records);
    let report = analyze(&Console::read(at), &Records::of(at));
    verdict(
        "speech_run_report",
        &records,
        &report,
        "the pipeline came up, composed, kept its bus attachment, and drained whole",
    )
}

#[cfg(test)]
mod tests {
    use std::path::{Path, PathBuf};

    use brenn_reachy__motion__reports_clk_rs::RefusalReasonWire;
    use clockwork_logs::ChannelMetadata;
    use clockwork_logs::onboard::OnboardWriter;
    use clockwork_rs::{SyncTime, blob_as_bytes};
    use nalgebra::{Isometry3, UnitQuaternion, Vector3};
    use reachy_motion::record::write_pose;
    use reachy_scratch::{Scratch, scratch_dir};
    use run_report::Report;

    use super::{
        ARRIVAL_OFFSET_M, ARRIVAL_TURN_RAD, Console, ESTIMATE_CHANNEL, HOST_LOG, Line, POD_LOG,
        PROVENANCE, PoseEstimateWire, REPORT_CHANNEL, Records, ReportKind, ReportKindWire,
        SCRIPT_CHANNEL, ScriptWire, SessionPhaseWire, TimelineEntryWire, TimelineWire, analyze,
        classify, console_dir, neutral_targets, refusal_kinds,
    };

    /// The whole reading of one fetch, as a case has just written it.
    fn judge(at: &Path) -> Report {
        analyze(&Console::read(at), &Records::of(at))
    }

    /// The log instant of the `n`th thing a case records.
    fn when(n: i64) -> SyncTime {
        SyncTime::from_nanos(1_000 + n)
    }

    /// One row of the session's story, with the two numbers its kind reads.
    fn row(kind: ReportKindWire, a: u32, b: u32) -> TimelineEntryWire {
        let mut entry = TimelineEntryWire::new();
        entry.set_time(when(i64::from(a)));
        entry.set_kind(kind);
        entry.set_a(a);
        entry.set_b(b);
        entry
    }

    /// One phase change, as the session narrates it: `a` the phase entered, `b`
    /// the one left.
    fn phase(to: SessionPhaseWire, from: SessionPhaseWire) -> TimelineEntryWire {
        row(
            ReportKindWire::PHASE_CHANGED,
            u32::from(to.0),
            u32::from(from.0),
        )
    }

    /// The three phase changes a session that took the machine narrates first.
    fn engagement() -> Vec<TimelineEntryWire> {
        vec![
            phase(SessionPhaseWire::RESTING, SessionPhaseWire::STARTING),
            phase(SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
            phase(SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING),
        ]
    }

    /// The row the session publishes for a script it took.
    fn accepted(script: u32) -> TimelineEntryWire {
        row(ReportKindWire::SCRIPT_ACCEPTED, script, 4)
    }

    /// A pose displaced from `from` by `metres` along x and `radians` about z.
    fn moved(from: &Isometry3<f64>, metres: f64, radians: f64) -> Isometry3<f64> {
        Isometry3::from_parts(
            (from.translation.vector + Vector3::new(metres, 0.0, 0.0)).into(),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), radians) * from.rotation,
        )
    }

    /// What one case records into a run directory.
    #[derive(Default)]
    struct Recorded {
        /// The rows of the story the session published.
        story: Vec<TimelineEntryWire>,
        /// Where the head was, sample by sample.
        poses: Vec<Isometry3<f64>>,
        /// How many scripts reached the session's port.
        scripts: usize,
        /// How many rows the session says it dropped off the front of that
        /// story, for the cases about a truncated ring.
        dropped: u32,
        /// How many estimates the run recorded that no pose solves out of.
        unsolved: usize,
        /// What the story channel is declared to carry, where a case wants
        /// something other than what it holds.
        story_schema: Option<ChannelMetadata>,
    }

    /// Write a run directory under a fetch, through the framework's own writer.
    ///
    /// The writer rather than bytes of this module's own devising: a hand-built
    /// file would be this tool's idea of the format rather than the format, and
    /// the whole point of the log half is that it reads what a unit records.
    fn recorded_into(records: &Path, stamp: &str, recorded: &Recorded) {
        let dir = records.join(stamp);
        std::fs::create_dir_all(&dir).expect("a run directory");
        let mut writer = OnboardWriter::create(&dir, "onboard_").expect("an onboard log");
        if !recorded.story.is_empty() {
            let declared = recorded
                .story_schema
                .clone()
                .unwrap_or_else(|| ChannelMetadata::for_schema::<TimelineWire>(REPORT_CHANNEL));
            let channel = writer.add_channel(&declared).expect("a story channel");
            let mut story = TimelineWire::new();
            {
                let mut entries = story.entries_mut();
                for entry in &recorded.story {
                    *entries
                        .try_grow()
                        .expect("a story of no more rows than the message holds") = entry.clone();
                }
            }
            story.set_dropped(recorded.dropped);
            writer
                .log_message(channel, 0, when(0), when(0), &[], blob_as_bytes(&story))
                .expect("the story");
        }
        if !recorded.poses.is_empty() || recorded.unsolved > 0 {
            let channel = writer
                .add_channel(&ChannelMetadata::for_schema::<PoseEstimateWire>(
                    ESTIMATE_CHANNEL,
                ))
                .expect("an estimate channel");
            for n in 0..recorded.unsolved {
                writer
                    .log_message(
                        channel,
                        u32::try_from(n).expect("a small run"),
                        when(n as i64),
                        when(n as i64),
                        &[],
                        blob_as_bytes(&PoseEstimateWire::new()),
                    )
                    .expect("an estimate nothing solves");
            }
            for (n, pose) in recorded.poses.iter().enumerate() {
                let mut estimate = PoseEstimateWire::new();
                {
                    let solved = estimate.clear_valid();
                    solved.time_of_validity = when(n as i64);
                    write_pose(&mut solved.head_pos, &mut solved.head_quat, pose);
                    solved.valid = true.into();
                }
                writer
                    .log_message(
                        channel,
                        u32::try_from(n).expect("a small run"),
                        when(n as i64),
                        when(n as i64),
                        &[],
                        blob_as_bytes(&estimate),
                    )
                    .expect("an estimate");
            }
        }
        if recorded.scripts > 0 {
            let channel = writer
                .add_channel(&ChannelMetadata::for_schema::<ScriptWire>(SCRIPT_CHANNEL))
                .expect("a script channel");
            for n in 0..recorded.scripts {
                let script = ScriptWire::new();
                writer
                    .log_message(
                        channel,
                        u32::try_from(n).expect("a small run"),
                        when(n as i64),
                        when(n as i64),
                        &[],
                        blob_as_bytes(&script),
                    )
                    .expect("a script");
            }
        }
        writer.close().expect("a closed log");
    }

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
        let report = judge(&at);
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
        let report = judge(&at);
        assert!(
            report.findings[0].contains("missing or empty"),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn an_empty_host_console_is_the_same_finding() {
        let (_dir, at) = records("speech-report-empty", &[]);
        let report = judge(&at);
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(report.findings[0].contains("missing or empty"));
    }

    #[test]
    fn a_console_with_no_started_line_is_a_finding() {
        let (_dir, at) = records("speech-report-unstarted", &[COMPOSED]);
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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

    /// A sentence the robot was asked to say and did not is prominent on the
    /// same terms an unpublished alert is: the other delivery leg of the same
    /// alert, and the one whose audience was in the room.
    #[test]
    fn an_unspoken_alert_is_prominent_and_does_not_fail() {
        let unspoken = r#"{"stream":"edge","at_ns":4,"kind":"unspoken","says":"the alert `the head is parked` was not said out loud: the announcement queue is full","title":"the head is parked","severity":"critical","reason":"backlogged"}"#;
        let (_dir, at) = records("speech-report-unspoken", &[STARTED, COMPOSED, unspoken]);
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("1 alert(s) whose sentence the robot was asked to say")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("was not said out loud")),
            "{:?}",
            report.measured
        );
    }

    /// The pipeline's half of the speaking path, which this host's stream
    /// cannot see: a sentence the seam took and the room still never heard.
    /// Notes, because which of these is a fault is the pipeline's to say — but
    /// present, because a critical lost here leaves no other trace at all.
    #[test]
    fn the_pipelines_account_of_each_sentence_is_read() {
        let spoken = r#"{"ts_ms":5,"event":"announcement_spoken","pods":["reachy00"],"text":"My head is not moving.","chars":22}"#;
        let unheard = r#"{"ts_ms":6,"event":"announcement_unheard","reason":"no_pod_connected","stage":null,"pod":null,"text":"My head is dropping every motion script.","chars":39}"#;
        let (_dir, at) = records(
            "speech-report-announcements",
            &[STARTED, COMPOSED, spoken, unheard],
        );
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        for expected in [
            "1 sentence(s) the pipeline queued for playback",
            "My head is not moving.",
            "1 sentence(s) the pipeline had and did not say",
            "no_pod_connected",
            "My head is dropping every motion script.",
        ] {
            assert!(
                report.measured.iter().any(|line| line.contains(expected)),
                "{expected:?} in {:?}",
                report.measured
            );
        }
    }

    /// A seam the pipeline could not speak through: the composition said the
    /// robot speaks, and the far side says it has no way to. Noted with the
    /// half the pipeline names as missing, so an operator is sent after that
    /// one rather than the other.
    #[test]
    fn a_seam_the_pipeline_could_not_speak_through_is_noted() {
        let unused = r#"{"ts_ms":5,"event":"announce_seam_unused","reason":"no tts"}"#;
        let (_dir, at) = records("speech-report-seam-unused", &[STARTED, COMPOSED, unused]);
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        let note = report
            .measured
            .iter()
            .find(|line| line.contains("could not speak through"))
            .unwrap_or_else(|| panic!("{:?}", report.measured));
        assert!(note.contains("no tts"), "{note}");
    }

    /// A critical raised on a host that composed no voice is noted: the
    /// deployment state this whole path exists to make visible, and invisible
    /// in a count of lines.
    #[test]
    fn a_critical_on_a_voiceless_host_says_nobody_in_the_room_was_told() {
        let voiceless = r#"{"stream":"edge","at_ns":2,"kind":"composed","says":"the voice pipeline is running","alerts":true,"speaks":false}"#;
        let critical = r#"{"stream":"alert","at_ns":3,"severity":"critical","title":"reachy head refuses its own scripts","says":"the head will not move","spoken":"My head is not moving."}"#;
        let (_dir, at) = records(
            "speech-report-voiceless-critical",
            &[STARTED, voiceless, critical],
        );
        let report = judge(&at);
        let note = report
            .measured
            .iter()
            .find(|line| line.contains("composed with no voice"))
            .unwrap_or_else(|| panic!("{:?}", report.measured));
        assert!(
            note.contains("nobody in the room was told"),
            "the whole sentence, not its opening: {note}"
        );
        assert!(
            !note.contains("  "),
            "an operator-facing sentence carries no run of spaces: {note}"
        );
    }

    /// The same host with a voice says nothing of the kind.
    #[test]
    fn a_critical_on_a_speaking_host_is_not_noted_as_unheard() {
        let speaking = r#"{"stream":"edge","at_ns":2,"kind":"composed","says":"the voice pipeline is running","alerts":true,"speaks":true}"#;
        let critical = r#"{"stream":"alert","at_ns":3,"severity":"critical","title":"reachy head refuses its own scripts","says":"the head will not move","spoken":"My head is not moving."}"#;
        let (_dir, at) = records(
            "speech-report-speaking-critical",
            &[STARTED, speaking, critical],
        );
        let report = judge(&at);
        assert!(
            !report
                .measured
                .iter()
                .any(|line| line.contains("composed with no voice")),
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
            judge(&at).findings.is_empty(),
            "an orderly teardown under either spelling is not a finding",
        );

        let died = r#"{"ts_ms":5,"event":"brenn_bridge_exit","fatal":true,"outcome":"no wire version in common"}"#;
        let (_dir, at) = records(
            "speech-report-bridge-exit-fatal-true",
            &[STARTED, COMPOSED, BRAIN, ATTACHED, died],
        );
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
            let report = judge(&at);
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
            let report = judge(&at);
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
        let report = judge(&at);
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
            let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = analyze(&console, &Records::of(&at));
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
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
                granted: None,
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
                granted: None,
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
        let report = judge(&at);
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
        let report = judge(&at);
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
        let report = judge(&at);
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("did not find it on the unit")),
            "{:?}",
            report.findings
        );
    }

    /// One motion script the pipeline authored, on its own stream.
    fn authored(cause: &str) -> String {
        format!(
            r#"{{"ts_ms":1,"event":"motion_script","cause":"{cause}","pod":"reachy00","seq":1,"steps":[{{"after_ms":0,"posture":"up"}}],"timeout_ms":30000}}"#
        )
    }

    /// A body the gate refused, narrated by the edge itself so the words this
    /// reads are the words the host writes.
    fn refused(origin: reachy_edge::Origin) -> String {
        reachy_edge::refusal_line(
            &reachy_edge::Refusal::ForeignPod {
                addressed: "reachy00".to_owned(),
                pod: "kitchen-reachy".to_owned(),
            },
            origin,
            clockwork_rs::SyncTime::from_nanos(3),
        )
    }

    /// A run in which nobody spoke asks for no motion, and asking for no motion
    /// is not a failure: the permissive standard, in one case.
    #[test]
    fn a_run_that_asked_for_no_motion_is_green() {
        let (_dir, at) = records("speech-report-no-motion", &[STARTED, COMPOSED]);
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("asked the head to move")),
            "{:?}",
            report.measured
        );
    }

    /// The primary scenario: the pipeline authored, this host's own gate
    /// refused every one, and nothing reached the session.
    #[test]
    fn scripts_this_host_authored_and_refused_itself_fail_the_run() {
        let local = refused(reachy_edge::Origin::Local);
        let (_dir, at) = records(
            "speech-report-own-refused",
            &[
                STARTED,
                COMPOSED,
                &authored("wake"),
                &authored("closing"),
                &local,
                &local,
            ],
        );
        let report = judge(&at);
        assert!(
            report.findings.iter().any(|finding| finding
                .contains("2 of the bodies this host offered to its own gate")
                && finding.contains("kitchen-reachy")),
            "{:?}",
            report.findings
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("the session accepted none")),
            "the outcome is a finding of its own: {:?}",
            report.findings
        );
    }

    /// A remote sender disagreeing with this machine is news about the sender:
    /// counted, never a finding.
    #[test]
    fn a_remote_senders_refused_body_is_not_this_runs_failure() {
        let remote = refused(reachy_edge::Origin::Remote);
        let (_dir, at) = records(
            "speech-report-remote-refused",
            &[STARTED, COMPOSED, &remote],
        );
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("1 body(ies) off the bus")),
            "{:?}",
            report.measured
        );
    }

    /// The two shapes that are not a refusal at all: a sink that could not hand
    /// the scripter's body to the gate, and a compiled script that never left.
    #[test]
    fn a_script_lost_before_or_after_the_gate_is_this_runs_failure_too() {
        let unoffered = r#"{"stream":"edge","at_ns":3,"kind":"unoffered","says":"a script for `reachy00` at seq 4 never reached the gate","source":"scripter","pod":"reachy00","seq":4}"#;
        let unsent = r#"{"stream":"edge","at_ns":4,"kind":"unsent","says":"script 7 compiled and could not be sent to the control process","script_id":7,"origin":"local"}"#;
        let (_dir, at) = records(
            "speech-report-lost-scripts",
            &[STARTED, COMPOSED, &authored("wake"), unoffered, unsent],
        );
        let report = judge(&at);
        assert!(
            report.findings.iter().any(
                |finding| finding.contains("2 of the bodies this host offered to its own gate")
            ),
            "{:?}",
            report.findings
        );
    }

    /// A body the bus sink could not queue is the remote sender's loss, and the
    /// same line's `source` is what says so.
    #[test]
    fn a_bus_bodys_unoffered_line_is_not_this_runs_failure() {
        let unoffered = r#"{"stream":"edge","at_ns":3,"kind":"unoffered","says":"a script for `reachy00` at seq 4 never reached the gate","source":"bus","pod":"reachy00","seq":4}"#;
        let (_dir, at) = records(
            "speech-report-bus-unoffered",
            &[STARTED, COMPOSED, unoffered],
        );
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// A script that reached the session and was declined there is the second
    /// rung of the ladder, and the session's own words are quoted.
    #[test]
    fn a_script_the_session_refused_fails_the_run() {
        let refused_row = format!(
            r#"{{"stream":"timeline","at_ns":5,"kind":"{}","says":"the session declined it: the machine is parked"}}"#,
            reachy_edge::row_word(ReportKind::ScriptRefused)
        );
        let accepted_row = format!(
            r#"{{"stream":"timeline","at_ns":4,"kind":"{}","says":"script 1 accepted"}}"#,
            reachy_edge::row_word(ReportKind::ScriptAccepted)
        );
        let (_dir, at) = records(
            "speech-report-session-refused",
            &[
                STARTED,
                COMPOSED,
                &authored("wake"),
                &accepted_row,
                &refused_row,
            ],
        );
        let report = judge(&at);
        assert!(
            found(&report, "the session refused 1 script(s)")
                && found(&report, "the machine is parked"),
            "{:?}",
            report.findings
        );
        // The other finding is the accepted script's own: this fetch carries no
        // records, so what the head did for it is not in these files.
        assert_eq!(report.findings.len(), 2, "{:?}", report.findings);
        assert!(
            found(&report, "no channel log to show whether the head moved"),
            "{:?}",
            report.findings
        );
    }

    /// An attachment without the alert grant, on a run that raised an alert
    /// through it: the alert was lost, and a run that loses its own bad news is
    /// a run whose verdict says nothing.
    const UNGRANTED: &str = r#"{"ts_ms":2,"event":"brenn_attached","alert_granted":false,"participant_id":"remote:reachy00","version":4}"#;
    const HANDED_OFF: &str = r#"{"ts_ms":3,"event":"alert_handed_off","delivery":"unconfirmed","severity":"warning","title":"reachy motion scripts refused","body":"1 script(s) refused so far"}"#;

    #[test]
    fn an_alert_handed_to_an_attachment_that_grants_none_is_lost() {
        let (_dir, at) = records(
            "speech-report-alert-lost",
            &[STARTED, COMPOSED, UNGRANTED, HANDED_OFF],
        );
        let report = judge(&at);
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(
            report.findings[0].contains("did not grant alerts")
                && report.findings[0].contains("reachy motion scripts refused"),
            "{:?}",
            report.findings
        );
    }

    /// The grant is the far side's configuration and this tool does not judge
    /// it: a run that raised nothing lost nothing.
    #[test]
    fn an_ungranted_attachment_that_carried_nothing_is_a_note() {
        let (_dir, at) = records(
            "speech-report-alert-ungranted",
            &[STARTED, COMPOSED, UNGRANTED],
        );
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("nothing was handed to it while it held the link")),
            "{:?}",
            report.measured
        );
    }

    /// A pipeline that says nothing about the grant is an older one than this
    /// reads for: noted, on the same skew tolerance the bridge check has.
    #[test]
    fn an_attachment_that_says_nothing_about_the_grant_is_a_note() {
        let silent = r#"{"ts_ms":2,"event":"brenn_attached","participant_id":"remote:reachy00","version":4}"#;
        let (_dir, at) = records(
            "speech-report-grant-unsaid",
            &[STARTED, COMPOSED, silent, HANDED_OFF],
        );
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("said nothing about the alert grant")),
            "{:?}",
            report.measured
        );
    }

    /// A host whose alerts were narration only did not lose one to the bus: the
    /// composition's own field is what says which.
    #[test]
    fn a_host_composing_without_alerts_loses_none_to_the_grant() {
        let narrating = r#"{"stream":"edge","at_ns":2,"kind":"composed","says":"the voice pipeline is running","alerts":false}"#;
        let (_dir, at) = records(
            "speech-report-narration-only",
            &[STARTED, narrating, UNGRANTED, HANDED_OFF],
        );
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("narration only")),
            "{:?}",
            report.measured
        );
    }

    /// The pipeline's motion events are read by field, and everything else on
    /// its stream stays the shape summary it was.
    #[test]
    fn a_motion_script_event_is_read_by_field() {
        let line = classify(&authored("wake")).line;
        assert_eq!(
            line,
            Line::Authored {
                keys: "cause,event,pod,seq,steps,timeout_ms,ts_ms".to_owned(),
                event: "motion_script".to_owned(),
                cause: "wake".to_owned(),
            }
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
        let report = judge(&at);
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("refusing rather than draining")),
            "{:?}",
            report.findings
        );
    }
    /// A run directory of a stamp a case can order against another.
    const OLDER: &str = "1788000000000000000";
    const NEWER: &str = "1788999999999999999";

    /// One `script_accepted` row as the edge renders the story onto the
    /// console, for the cases with no log to read the story from.
    const ACCEPTED_ROW: &str = r#"{"stream":"timeline","at_ns":5,"kind":"script_accepted","says":"script 1 accepted: 4 step(s), schedule epoch 1"}"#;

    /// Whether any finding says `what`.
    fn found(report: &Report, what: &str) -> bool {
        report.findings.iter().any(|finding| finding.contains(what))
    }

    /// Whether any measurement says `what`.
    fn measured(report: &Report, what: &str) -> bool {
        report.measured.iter().any(|line| line.contains(what))
    }

    /// The permissive standard over the log half: a fetch with no records in it
    /// is a fetch of a run that asked for no motion, and that is a note.
    #[test]
    fn a_fetch_with_no_run_directory_is_a_note() {
        let (_dir, at) = records("speech-report-no-log", &[STARTED, COMPOSED]);
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            measured(&report, "no run directory"),
            "{:?}",
            report.measured
        );
    }

    /// The incident's own shape on the log side: nothing was accepted, so the
    /// head sitting still is not a finding of its own.
    #[test]
    fn a_still_head_on_a_run_that_accepted_nothing_is_not_a_finding() {
        let (_dir, at) = records("speech-report-still-unasked", &[STARTED, COMPOSED]);
        let held = neutral_targets().head_pose_body;
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story: engagement(),
                poses: vec![held, held, held],
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            measured(&report, "largest excursion over the run was 0.0000 m"),
            "{:?}",
            report.measured
        );
        assert!(
            measured(&report, "closest to `up`"),
            "{:?}",
            report.measured
        );
    }

    /// The failure this half exists for: the session took a script and the head
    /// never left the box an arrival is judged by.
    #[test]
    fn an_accepted_script_whose_head_never_moved_fails() {
        let (_dir, at) = records("speech-report-still", &[STARTED, COMPOSED]);
        let held = neutral_targets().head_pose_body;
        let mut story = engagement();
        story.push(accepted(1));
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story,
                poses: vec![held, held, held],
                scripts: 1,
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(
            found(&report, "the head never moved"),
            "{:?}",
            report.findings
        );
        assert!(
            measured(&report, "1 script(s) reached the session's port"),
            "{:?}",
            report.measured
        );
        assert!(
            measured(
                &report,
                "read from the session's own story in the channel log"
            ),
            "{:?}",
            report.measured
        );
    }

    /// A departure past either tolerance is movement, and movement is a green
    /// run whatever else the head did with it.
    #[test]
    fn an_accepted_script_whose_head_moved_is_green() {
        let (_dir, at) = records("speech-report-moved", &[STARTED, COMPOSED]);
        let held = neutral_targets().head_pose_body;
        let mut story = engagement();
        story.push(accepted(1));
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story,
                poses: vec![held, moved(&held, ARRIVAL_OFFSET_M * 10.0, 0.0), held],
                scripts: 1,
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// A turn past the other tolerance is movement too: the two components are
    /// maxima in their own right, so a head that only turned is not still.
    #[test]
    fn a_head_that_only_turned_is_movement() {
        let (_dir, at) = records("speech-report-turned", &[STARTED, COMPOSED]);
        let held = neutral_targets().head_pose_body;
        let mut story = engagement();
        story.push(accepted(1));
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story,
                poses: vec![held, moved(&held, 0.0, ARRIVAL_TURN_RAD * 4.0)],
                scripts: 1,
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// A script the session took and never ran a schedule for: the estimates
    /// cannot show why, and the story can.
    #[test]
    fn an_accepted_script_the_session_never_engaged_for_fails() {
        let (_dir, at) = records("speech-report-unengaged", &[STARTED, COMPOSED]);
        let held = neutral_targets().head_pose_body;
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story: vec![
                    phase(SessionPhaseWire::RESTING, SessionPhaseWire::STARTING),
                    phase(SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
                    accepted(1),
                ],
                poses: vec![held, moved(&held, ARRIVAL_OFFSET_M * 10.0, 0.0)],
                scripts: 1,
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(
            found(&report, "never took the machine"),
            "{:?}",
            report.findings
        );
    }

    /// An accepted script and no records at all: the central question of the
    /// run is one these records cannot answer, which is a finding rather than a
    /// silence.
    #[test]
    fn an_accepted_script_with_no_log_fails() {
        let (_dir, at) = records(
            "speech-report-accepted-unlogged",
            &[STARTED, COMPOSED, ACCEPTED_ROW],
        );
        let report = judge(&at);
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(
            found(&report, "no channel log to show whether the head moved"),
            "{:?}",
            report.findings
        );
    }

    /// A log another build's schemas wrote is no log: its bytes would decode as
    /// the wrong message, so it is refused and the accepted script has no
    /// account behind it.
    #[test]
    fn a_log_recorded_under_other_schemas_is_no_log() {
        let (_dir, at) = records(
            "speech-report-schema-skew",
            &[STARTED, COMPOSED, ACCEPTED_ROW],
        );
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story: engagement(),
                story_schema: Some(ChannelMetadata::for_schema::<PoseEstimateWire>(
                    REPORT_CHANNEL,
                )),
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(
            found(&report, "did not read as a log this build can trust")
                && found(&report, "ReportsOut carries")
                && found(&report, PROVENANCE),
            "{:?}",
            report.findings
        );
    }

    /// Several run directories are read the way the motion-run path reads one:
    /// the newest by sort, named in the output so a reader knows which.
    #[test]
    fn the_newest_run_directory_is_the_one_read() {
        let (_dir, at) = records("speech-report-two-runs", &[STARTED, COMPOSED]);
        let held = neutral_targets().head_pose_body;
        let mut story = engagement();
        story.push(accepted(1));
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story: story.clone(),
                poses: vec![held, held],
                ..Recorded::default()
            },
        );
        recorded_into(
            &at,
            NEWER,
            &Recorded {
                story,
                poses: vec![held, moved(&held, ARRIVAL_OFFSET_M * 10.0, 0.0)],
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert!(
            report.findings.is_empty(),
            "the newer run moved: {:?}",
            report.findings
        );
        assert!(
            measured(&report, NEWER) && measured(&report, "2 run directories"),
            "{:?}",
            report.measured
        );
    }

    /// The log is the better witness where both accounts exist, and a console
    /// that says something else is said to disagree rather than judged.
    #[test]
    fn the_logs_story_is_read_over_the_consoles() {
        let (_dir, at) = records("speech-report-two-accounts", &[STARTED, COMPOSED]);
        let held = neutral_targets().head_pose_body;
        let mut story = engagement();
        story.push(accepted(1));
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story,
                poses: vec![held, moved(&held, ARRIVAL_OFFSET_M * 10.0, 0.0)],
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            measured(
                &report,
                "1 accepted and 0 refused, read from the session's own story"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            measured(
                &report,
                "the console renders 0 accepted and 0 refused, which disagrees"
            ),
            "{:?}",
            report.measured
        );
    }

    /// A refusal in the log is the session's own words, quoted from the same
    /// table the edge would have rendered them with.
    #[test]
    fn a_refusal_in_the_log_fails_with_the_sessions_own_sentence() {
        let (_dir, at) = records("speech-report-log-refusal", &[STARTED, COMPOSED]);
        let mut story = engagement();
        story.push(row(
            ReportKindWire::SCRIPT_REFUSED,
            2,
            u32::from(RefusalReasonWire::NOT_RESTING.0),
        ));
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story,
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert!(
            found(&report, "the session refused 1 script(s)")
                && found(&report, "busy with a session"),
            "{:?}",
            report.findings
        );
    }

    /// The story is a ring, and a run long enough to fill it loses its
    /// engagement off the front. Absence there is the end of the evidence, not
    /// a session that never took the machine.
    #[test]
    fn an_engagement_dropped_off_the_front_of_the_story_is_not_a_finding() {
        let (_dir, at) = records("speech-report-truncated-story", &[STARTED, COMPOSED]);
        let held = neutral_targets().head_pose_body;
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                // The surviving rows: an acceptance, and no phase change at
                // all, which is the shape a full ring leaves behind.
                story: vec![accepted(9)],
                dropped: 40,
                poses: vec![held, moved(&held, ARRIVAL_OFFSET_M * 10.0, 0.0)],
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            measured(
                &report,
                "whether it took the machine is not in these records"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            measured(&report, "at least 1 accepted"),
            "the count off a ring is a floor and says so: {:?}",
            report.measured
        );
        assert!(
            measured(&report, "dropped 40 row(s) off the front"),
            "{:?}",
            report.measured
        );
        assert!(
            !measured(&report, "disagrees with the log"),
            "two windows are not compared: {:?}",
            report.measured
        );
    }

    /// A story that holds everything it ever said and no engagement in it is
    /// the finding, unchanged.
    #[test]
    fn a_whole_story_with_no_engagement_still_fails() {
        let (_dir, at) = records("speech-report-no-engagement", &[STARTED, COMPOSED]);
        let held = neutral_targets().head_pose_body;
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story: vec![accepted(9)],
                poses: vec![held, moved(&held, ARRIVAL_OFFSET_M * 10.0, 0.0)],
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert!(
            found(&report, "never took the machine"),
            "{:?}",
            report.findings
        );
    }

    /// Nothing recorded about where the head was is a different fact from a
    /// head that sat still, and saying the second would point an operator at
    /// the machine instead of at the recording.
    #[test]
    fn an_accepted_script_with_no_solvable_estimates_says_so_rather_than_still() {
        let (_dir, at) = records("speech-report-unsolved", &[STARTED, COMPOSED]);
        let mut story = engagement();
        story.push(accepted(1));
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story,
                unsolved: 3,
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        assert!(
            found(&report, "no pair of solvable head estimates")
                && found(&report, "0 solved of 3 recorded"),
            "{:?}",
            report.findings
        );
        assert!(
            !found(&report, "the head never moved"),
            "absence of evidence is not evidence of stillness: {:?}",
            report.findings
        );
    }

    /// One solved estimate is nothing to measure a departure against, and it
    /// is read the same way.
    #[test]
    fn a_single_solved_estimate_is_no_account_of_the_head_either() {
        let (_dir, at) = records("speech-report-one-estimate", &[STARTED, COMPOSED]);
        let mut story = engagement();
        story.push(accepted(1));
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story,
                poses: vec![neutral_targets().head_pose_body],
                ..Recorded::default()
            },
        );
        let report = judge(&at);
        assert!(
            found(&report, "no pair of solvable head estimates"),
            "{:?}",
            report.findings
        );
    }

    /// A logger that made its file and wrote nothing is no run directory: the
    /// newest by sort must not be an empty directory shadowing a real log.
    #[test]
    fn a_zero_byte_record_file_does_not_shadow_an_older_real_log() {
        let (_dir, at) = records("speech-report-zero-byte", &[STARTED, COMPOSED]);
        let held = neutral_targets().head_pose_body;
        let mut story = engagement();
        story.push(accepted(1));
        recorded_into(
            &at,
            OLDER,
            &Recorded {
                story,
                poses: vec![held, moved(&held, ARRIVAL_OFFSET_M * 10.0, 0.0)],
                ..Recorded::default()
            },
        );
        let empty = at.join(NEWER);
        std::fs::create_dir_all(&empty).expect("a run directory");
        std::fs::write(empty.join("onboard_0.olog"), []).expect("a record file of no bytes");

        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            measured(&report, OLDER) && !measured(&report, "2 run directories"),
            "the empty directory is not a run directory at all: {:?}",
            report.measured
        );
    }

    /// A record file with bytes in it that are not a log this build can walk is
    /// not a still head: the reader's own complaint is quoted, and the run is
    /// told there is nothing to read the head off.
    #[test]
    fn a_record_file_that_is_not_a_log_is_no_log_rather_than_a_still_head() {
        let (_dir, at) = records(
            "speech-report-unopenable",
            &[STARTED, COMPOSED, ACCEPTED_ROW],
        );
        let dir = at.join(OLDER);
        std::fs::create_dir_all(&dir).expect("a run directory");
        std::fs::write(dir.join("onboard_0.olog"), b"not a log at all\n")
            .expect("a record file of the wrong bytes");

        let report = judge(&at);
        assert!(
            found(&report, "no channel log to show whether the head moved")
                && found(&report, "did not read as a log this build can trust")
                && found(&report, PROVENANCE),
            "{:?}",
            report.findings
        );
        assert!(
            !found(&report, "the head never moved"),
            "a file nobody can walk is not a machine that sat still: {:?}",
            report.findings
        );
    }

    /// A refusal line from a build before the edge spelled an origin is read as
    /// a remote sender's business, which is the safe way round, and the run
    /// says the attribution was not in its records.
    #[test]
    fn a_refusal_with_no_origin_word_is_not_read_as_this_hosts_own() {
        let older = r#"{"stream":"edge","at_ns":3,"kind":"foreign_pod","says":"the script is addressed to `reachy00`; this machine is `kitchen-reachy`"}"#;
        let (_dir, at) = records(
            "speech-report-origin-unsaid",
            &[STARTED, COMPOSED, &authored("wake"), older],
        );
        let report = judge(&at);
        assert!(
            !found(&report, "offered to its own gate"),
            "an unattributable refusal is not attributed: {:?}",
            report.findings
        );
        assert!(
            measured(&report, "1 body(ies) off the bus"),
            "{:?}",
            report.measured
        );
        assert!(
            measured(&report, "carried no origin word"),
            "nobody reads a run this tool could not attribute as one it did: {:?}",
            report.measured
        );
    }

    /// This host sends every accepted script, whoever wrote it, so a send that
    /// failed is a failure without an author.
    #[test]
    fn an_unsent_script_this_host_did_not_author_is_its_own_finding() {
        let unsent = r#"{"stream":"edge","at_ns":4,"kind":"unsent","says":"script 7 compiled and could not be sent to the control process","script_id":7,"origin":"remote"}"#;
        let (_dir, at) = records("speech-report-unsent-remote", &[STARTED, COMPOSED, unsent]);
        let report = judge(&at);
        assert!(
            found(
                &report,
                "1 accepted script(s) compiled and never reached the session's port"
            ) && found(&report, "not attributed to a side"),
            "{:?}",
            report.findings
        );
        assert!(
            !found(&report, "offered to its own gate"),
            "{:?}",
            report.findings
        );
    }

    /// The bridge re-attaches after every closed socket, so what one
    /// attachment granted says nothing about the next: an alert handed to a
    /// granting one travelled.
    #[test]
    fn an_alert_handed_to_a_later_granting_attachment_is_not_lost() {
        const GRANTED: &str = r#"{"ts_ms":4,"event":"brenn_attached","alert_granted":true,"participant_id":"remote:reachy00","version":4}"#;
        let (_dir, at) = records(
            "speech-report-alert-regranted",
            &[STARTED, COMPOSED, UNGRANTED, GRANTED, HANDED_OFF],
        );
        let report = judge(&at);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            measured(&report, "nothing was handed to it while it held the link"),
            "{:?}",
            report.measured
        );
    }

    /// And the other order: an alert handed off while the ungranting
    /// attachment held the link is lost, however a later one answered.
    #[test]
    fn an_alert_handed_off_before_the_grant_landed_is_still_lost() {
        const GRANTED: &str = r#"{"ts_ms":9,"event":"brenn_attached","alert_granted":true,"participant_id":"remote:reachy00","version":4}"#;
        let (_dir, at) = records(
            "speech-report-alert-lost-then-granted",
            &[STARTED, COMPOSED, UNGRANTED, HANDED_OFF, GRANTED],
        );
        let report = judge(&at);
        assert!(
            found(&report, "1 alert(s) this run raised were handed off"),
            "{:?}",
            report.findings
        );
    }
}
