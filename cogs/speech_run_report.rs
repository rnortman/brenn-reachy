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
//! and never fails: the turns of the conversation, counts per kind with the
//! sentence each kind said, the warning alerts, the alerts that never reached
//! the bus, the bodies the edge dropped.
//!
//! The turns are the part a person reads first. One line per wake — what was
//! heard, how sure the recogniser was, what the confidence gate did with it,
//! and where the auto-select beam was pointing while it was said — because a
//! run of five wakes and five declines is a run in which nobody was answered,
//! and a report that says only that the pipeline came up and drained whole has
//! not told anybody that.
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

use std::collections::{BTreeMap, BTreeSet};
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

/// What the fetch names the recorded-audio store beside a record directory.
const AUDIO_SUFFIX: &str = ".audio";

/// What this tool names the directory it writes one clip per turn into.
const TURNS_SUFFIX: &str = ".turns";

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

/// The pipeline events one turn of a conversation is assembled out of.
///
/// Speech-surface's vocabulary and not this tree's, spelled once here. The
/// whole utterance lifecycle: the wake that opened the turn, the transcript it
/// produced, and the one event that says what became of it. A turn is what an
/// operator asks a speech run about — the counts per event name the rest of
/// this tool keeps say a pipeline was busy and never say whether anybody was
/// answered.
const WAKE_DETECTED: &str = "wake_detected";
const UTTERANCE: &str = "utterance";
const BRAIN_DISPATCHED: &str = "brain_dispatched";
const WAKE_COMMAND_ABSENT: &str = "wake_command_absent";
const BARGE_COMMAND_ABSENT: &str = "barge_command_absent";
const BRAIN_NO_TRANSCRIPT: &str = "brain_no_transcript";
const STT_FAILED: &str = "stt_failed";
const UTTERANCE_SUPERSEDED: &str = "utterance_superseded";

/// The wake that was kept back for the command it introduces.
///
/// Emitted where an utterance held the wake word alone: nothing is transcribed
/// and no utterance is minted, and the next speech inside the wait is carved
/// together with it. A turn carrying this line began further back than its own
/// onset, and a wake that ends `arm_expired` carrying it waited for a command
/// that never came -- which is a different failure from a wake nobody followed.
const WAKE_HELD: &str = "wake_held";

/// The reason word a wake goes unanswered under when its arm ran out.
const ARM_EXPIRED: &str = "arm_expired";

/// The events about a reply already playing: what cut it, and how long it ran.
///
/// `barge_in` names no utterance — it is the moment somebody spoke over the
/// pod, ahead of any decision about what was playing — so it is attributed to
/// the reply it cut, which is the last turn this console dispatched.
const BARGE_IN: &str = "barge_in";
const PLAYBACK_FLUSHED: &str = "playback_flushed";
const PLAYBACK_FINISHED: &str = "playback_finished";

/// The two events the auto-select beam's bearing is read off.
///
/// A segment closes, saying where its first sample sits in the pod's own index
/// space, and its tracking line follows with the bearings the beamformer held
/// through it. The pair is what converts a turn's pod-absolute span into the
/// segment-relative offsets the bearings are stamped with.
const SEGMENT_CLOSED: &str = "segment_closed";
const TRACKING: &str = "tracking";

/// A pod's connection announcing itself.
///
/// Read for one thing: the pod counts its samples from zero again on every
/// connection, so this line is where one index space ends and the next begins.
const CONN_HELLO: &str = "conn_hello";

/// Which of the four bearings a tracking sample carries is the auto-select
/// beam's.
///
/// The chip reports one bearing per beam and the last is the beam the pod
/// sends. A sample with fewer than this many is one this build has no reader
/// for and is skipped rather than guessed at.
const AUTO_BEAM: usize = 3;

/// The reason word the confidence gate declines a transcript under.
///
/// The other reasons a wake goes unanswered — an empty transcript, an arm that
/// expired — are declines too and counted as such; this word is what separates
/// the ones that had words in them, which is the class worth reading back.
const LOW_CONFIDENCE: &str = "low_confidence";

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

/// Where in the pod's audio a turn's clip was carved from.
///
/// Two index spaces meet here. `start_sample` and `end_sample` are
/// pod-absolute, as the span the utterance names is; `segment` is the segment
/// that span falls in, whose own bearings are stamped relative to its first
/// sample. Every field is optional because every field is read optionally: a
/// console written by a pipeline that names one of them differently prints the
/// figure that needs it as missing rather than a number nobody can check.
#[derive(Debug, Default, PartialEq)]
struct Span {
    /// Which frame log in the store holds those samples, store-root-relative,
    /// as the line named it. The clip is cut out of this file.
    log: Option<String>,
    start_sample: Option<i64>,
    end_sample: Option<i64>,
    segment: Option<u32>,
    /// Which cap-rollover part of that segment, where the span said. One
    /// segment id can close several times over — the host's length cap
    /// finalizes a part and opens a contiguous successor under the same id —
    /// and only the part tells the two apart.
    part: Option<u16>,
}

impl Span {
    /// Fill this span's empty fields from another's.
    ///
    /// A turn's span arrives in pieces: the utterance names the samples, and on
    /// a pipeline whose first utterance of a process carries no segment list
    /// the decline line beside it is where the segment is named. Neither
    /// overwrites what the other already said.
    fn absorb(&mut self, other: Span) {
        self.log = self.log.take().or(other.log);
        self.start_sample = self.start_sample.or(other.start_sample);
        self.end_sample = self.end_sample.or(other.end_sample);
        self.segment = self.segment.or(other.segment);
        self.part = self.part.or(other.part);
    }
}

/// Which closed segment a line names.
///
/// The id alone does not name one: a segment id closes twice over whenever the
/// host's length cap finalizes a part and opens a successor under the same id.
/// The pod is here because each one counts its samples in its own space, and a
/// segment id means nothing across two of them.
#[derive(Clone, Debug, Eq, Ord, PartialEq, PartialOrd)]
struct SegKey {
    /// Which pod said it, empty where the line named none.
    pod: String,
    /// Which connection generation the console was in when the segment closed.
    ///
    /// A pod counts samples from zero again on every connection, so a segment
    /// id and a sample index both mean something only inside one generation.
    /// The counter is console-wide: a hello from any pod fences every pod's
    /// space, which is what a console carrying one pod — every console this
    /// tool has been pointed at — needs it to do.
    generation: u32,
    segment: u32,
    /// Which cap-rollover part of the segment, zero for the first and for a
    /// console that does not say.
    part: u16,
}

/// What the console's closed segments say about a turn whose span names none of
/// them.
#[derive(Debug, PartialEq)]
enum Attribution<'a> {
    /// Exactly one holds the sample the turn is anchored by, and its bearings
    /// are the turn's.
    One(&'a SegKey),
    /// More than one does, so none of them answers for it: one sample sitting
    /// inside two segments is two segments that share no audio.
    Several,
    /// None does.
    None,
}

/// Where one closed segment sits in the pod's own index space.
///
/// The base is what converts a pod-absolute sample into the segment-relative
/// offsets its bearings are stamped in. The count is what makes the segment a
/// range rather than a point, which is how a turn that names no segment is
/// attributed to the one it was carved from.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Closed {
    base: i64,
    /// How many samples it held, where its line said. A console that does not
    /// say leaves the segment a point, and a turn naming no segment finds
    /// nothing to fall back on.
    ///
    /// A segment with gaps in it spans more of the pod's index space than it
    /// holds samples of, and the console does not say how many were dropped, so
    /// a carve near such a segment's tail is held by nothing.
    // TODO(beam-gappy-segment-extent): the console would have to carry the
    // dropped-sample count for the range to be the segment's true extent.
    samples: Option<i64>,
}

impl Closed {
    /// Whether a pod-absolute sample falls inside this segment.
    ///
    /// Every figure here is a number off the console, so the end is computed
    /// checked: a torn or corrupted line carrying a base or a count near the
    /// end of the range holds nothing rather than killing the report the run
    /// is read by.
    fn holds(&self, sample: i64) -> bool {
        self.samples
            .and_then(|samples| self.base.checked_add(samples))
            .is_some_and(|end| sample >= self.base && sample < end)
    }
}

/// What one transcript said and how sure the recogniser was of it.
#[derive(Debug, Default, PartialEq)]
struct Spoken {
    /// The utterance's sequence number, the key every later event joins on.
    id: Option<u64>,
    /// Which pod carved it, empty where the line named none. The sample
    /// indexes on this line are counted in that pod's space and nowhere else.
    pod: String,
    text: Option<String>,
    no_speech: Option<f64>,
    logprob: Option<f64>,
    compression: Option<f64>,
    /// Why the endpointer decided the utterance had ended, in its own word.
    endpoint_cause: String,
    span: Span,
}

/// A wake kept back for its command: the span held so far, and how long the
/// listener was prepared to wait past it.
///
/// The deadline is a budget and not an observation -- a command arriving ends
/// the wait early, and says so by the turn dispatching at all -- so the turn
/// line reads it as an upper bound.
#[derive(Debug, Default, Clone, Copy, PartialEq)]
struct Held {
    start_sample: Option<i64>,
    end_sample: Option<i64>,
    deadline_sample: Option<i64>,
}

impl Held {
    /// How long the listener was prepared to wait past the held speech, in
    /// seconds, where both ends of the wait were stated.
    fn waited_s(self) -> Option<f64> {
        let (end, deadline) = (self.end_sample?, self.deadline_sample?);
        Some((deadline - end) as f64 / f64::from(speech_pipeline::SPINE_FORMAT.sample_rate_hz))
    }
}

/// One event of the utterance lifecycle, read by field.
///
/// The fourth variant group [`Line`] holds another repo's schema for, grouped
/// by what reads it: these are the events one conversational turn is assembled
/// out of, and the assembly is a fold over them in the order the console
/// carried them.
#[derive(Debug, PartialEq)]
enum Turned {
    /// A wake, which opens a turn whether or not an utterance follows it.
    Wake {
        pod: String,
        score: Option<f64>,
        wake_end_sample: Option<i64>,
    },
    /// A pod's connection announced itself, fencing the sample-index space
    /// every line before it counted in off from every line after it.
    Connected,
    /// The wake was kept back for the command that follows it.
    Held(Held),
    /// The transcript itself.
    Said {
        spoken: Box<Spoken>,
        /// How many samples at the head of the carve the recogniser was not to
        /// be given, and where it was actually started. Not what was said, so
        /// not on [`Spoken`].
        stt_trim: Option<i64>,
        stt_sent_from: Option<i64>,
    },
    /// The turn reached the brain.
    Dispatched { id: Option<u64> },
    /// The gate declined it, in its own word, with the two numbers it judged on.
    Declined {
        id: Option<u64>,
        reason: String,
        no_speech: Option<f64>,
        logprob: Option<f64>,
        span: Span,
        /// How many samples of the carve the recogniser was not given, where
        /// this line said. Only the decline lines carry it.
        stt_trim: Option<i64>,
    },
    /// The brain was handed a turn with no words in it.
    NoTranscript { id: Option<u64> },
    /// The recogniser itself failed, in its own words.
    SttFailed { id: Option<u64>, detail: String },
    /// Speech resumed and the utterance was transcribed again.
    Superseded { id: Option<u64> },
    /// Somebody spoke over a reply that was playing.
    BargeIn,
    /// What was left of that reply was thrown away.
    Flushed { id: Option<u64> },
    /// A reply played to its end, and for how long.
    Played {
        id: Option<u64>,
        played_ms: Option<f64>,
    },
    /// A segment ended, saying where its first sample sits and how many it
    /// held.
    SegmentClosed {
        pod: String,
        segment: Option<u32>,
        part: u16,
        base_sample: Option<i64>,
        samples: Option<i64>,
    },
    /// The bearings the auto-select beam held through one segment, as
    /// (offset, radians) pairs in that segment's own index space.
    Tracking {
        pod: String,
        segment: Option<u32>,
        part: u16,
        beams: Vec<(i64, f64)>,
    },
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
    /// One event of the utterance lifecycle, read by field.
    ///
    /// It carries the key summary too, because these are pipeline events like
    /// any other and are counted as such.
    Turned {
        keys: String,
        event: String,
        turned: Box<Turned>,
    },
}

/// What became of one turn, in the pipeline's own decision.
#[derive(Debug, Default, PartialEq)]
enum Outcome {
    /// Nothing on this console said. A run the operator ended mid-turn.
    #[default]
    Open,
    /// It reached the brain.
    Dispatched,
    /// The gate declined it, in its own word.
    Declined(String),
    /// The brain was handed a turn with no words in it.
    NoTranscript,
    /// The recogniser failed, in its own words.
    SttFailed(String),
}

/// The suffix on a turn's whole-carve clip. These two names are the contract
/// between the report and `//crates/reachy-host:stt_compare`;
/// `the_two_clip_names_are_the_pair_the_comparison_looks_for` pins them.
const WHOLE_SUFFIX: &str = ".wav";

/// The suffix on the same turn's recogniser-facing clip.
const COMMAND_SUFFIX: &str = ".command.wav";

/// What one turn is called, in the two places it is named.
///
/// One value with two spellings rather than two spellings decided apart: the
/// token the turn line prints after `#` is the token the clip's file name
/// carries after `turn-`, which is what makes a clip findable from the report
/// by reading. Spelled in two functions they drift the first time a new kind of
/// turn is named, and the drift reads as a `.wav` no line points at.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Label {
    /// The utterance's own number, which every event of the turn joins on.
    Utterance(u64),
    /// Which wake of the run opened it, for a turn no utterance was minted for.
    Wake(usize),
}

impl Label {
    /// How the turn line names it.
    fn on_line(self) -> String {
        match self {
            Self::Utterance(id) => format!("#{id}"),
            Self::Wake(ordinal) => format!("#? (wake {ordinal})"),
        }
    }

    /// The turn's own token, which both of its clip names are built from.
    fn stem(self) -> String {
        match self {
            Self::Utterance(id) => format!("turn-{id:02}"),
            Self::Wake(ordinal) => format!("turn-wake-{ordinal}"),
        }
    }

    /// What the turn's clip is called.
    fn file_name(self) -> String {
        format!("{}{WHOLE_SUFFIX}", self.stem())
    }

    /// What the same turn's recogniser-facing clip is called: the carve from
    /// the wake-trim boundary, as a sibling of the whole one so a pairing is a
    /// name and never a record to parse.
    fn command_file_name(self) -> String {
        format!("{}{COMMAND_SUFFIX}", self.stem())
    }
}

/// One turn of a conversation: a wake, what was heard, and what was done.
///
/// The unit an operator judges a speech run in. A turn begins at a wake and
/// holds whatever followed it, including nothing: a wake that minted no
/// utterance is a turn the pipeline had and could not read, and it is printed
/// as such rather than left out of a count nobody would notice it missing
/// from.
#[derive(Debug, Default)]
struct Turn {
    /// Which wake of the run opened it, counting from one; zero on a turn no
    /// wake opened.
    ordinal: usize,
    /// Whether a `wake_detected` opened it. False on a turn a barge-in minted,
    /// which is as real as any other and is not a wake the model fired.
    woke: bool,
    /// Which pod's index space its samples are counted in, empty where no line
    /// of it named one.
    pod: String,
    /// Which connection generation the console was in when the turn opened.
    ///
    /// The pod counts samples from zero again on every connection, so a
    /// segment closed after a reconnect can hold a sample of a turn from
    /// before it. A turn and a segment answer for each other only inside one
    /// generation; a figure read out of another connection's audio is worse
    /// than none.
    generation: u32,
    /// What was said, once a transcript existed.
    said: Spoken,
    wake_score: Option<f64>,
    wake_end_sample: Option<i64>,
    outcome: Outcome,
    /// How long the reply to it played, where one played to its end.
    played_ms: Option<f64>,
    /// How many times speech resumed and the utterance was transcribed again.
    superseded: usize,
    /// How many times somebody spoke over this turn's reply.
    barge_ins: usize,
    /// How many times what was left of that reply was thrown away.
    flushed: usize,
    /// How many samples at the head of the carve the recogniser was not given,
    /// where a line of this turn said. Carried by both the decline lines and
    /// the utterance line, so the clip fragment states the boundary on a
    /// dispatched turn as well as on a declined one.
    stt_trim: Option<i64>,
    /// Which sample of the carve the recogniser was actually started at, where
    /// the utterance line said. Equal to [`Turn::stt_trim`] where the wake word
    /// was cut out of the clip and zero where it was left in, which is the one
    /// record of which way the run was configured.
    stt_sent_from: Option<i64>,
    /// The wake this turn opened with was kept back for its command, where the
    /// console said so.
    held: Option<Held>,
}

impl Turn {
    /// The pod-absolute sample this turn is attributed to a segment by.
    ///
    /// The carve's own start, because that is the sample the pipeline judged
    /// belonged to this utterance. The wake is the fallback for a start no
    /// closed range holds — a carve whose pre-roll reaches back past its
    /// segment's base — and for a span that stated no start at all; it is
    /// never what gives a spanless turn a figure, because such a turn states
    /// no window end and is not windowed at all.
    fn carved_at(&self) -> Option<i64> {
        self.said.span.start_sample.or(self.wake_end_sample)
    }

    /// The pod-absolute sample this turn's bearing window opens at.
    ///
    /// The wake's end, so the look direction the wake word was spoken from is
    /// not averaged into the utterance's; the carve's start only where no wake
    /// said. Deliberately the opposite precedence to [`Turn::carved_at`] — the
    /// two answer different questions — which is why [`Console::beam_of`]
    /// re-reads the attributing sample when the window opens outside the
    /// segment that holds the carve.
    fn window_from(&self) -> Option<i64> {
        self.wake_end_sample.or(self.said.span.start_sample)
    }

    /// What names this turn, wherever it is named.
    ///
    /// `None` for a turn the console named neither an utterance nor a wake
    /// for: it is printed as unnamed and no file is cut for it.
    fn label(&self) -> Option<Label> {
        match (self.said.id, self.woke) {
            (Some(id), _) => Some(Label::Utterance(id)),
            (None, true) => Some(Label::Wake(self.ordinal)),
            (None, false) => None,
        }
    }

    /// Whether the gate declined this turn as a likely hallucination.
    fn hallucinated(&self) -> bool {
        matches!(&self.outcome, Outcome::Declined(reason) if reason == LOW_CONFIDENCE)
            && self
                .said
                .text
                .as_deref()
                .is_some_and(|text| !text.is_empty())
    }
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
/// grows that the output would not. The one exception is [`Console::tracked`],
/// which holds one numeric column of one event kind because the turn a bearing
/// belongs to is not always on the console yet when the bearing is.
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
    /// Every turn of the conversation, in the order the wakes came.
    ///
    /// Kept in full, which is the exception this fold's rule already names: the
    /// report prints one line per turn anyway, so nothing here grows that the
    /// output would not. A turn is a person waking the robot, and a run holds
    /// as many as somebody had the patience for.
    turns: Vec<Turn>,
    /// Which turn each utterance sequence number belongs to.
    by_id: BTreeMap<u64, usize>,
    /// The turn a wake opened that no utterance has joined yet.
    open_turn: Option<usize>,
    /// The last turn this console dispatched, which is whose reply is playing.
    last_dispatched: Option<usize>,
    /// Where each closed segment sits in the pod's own index space.
    ///
    /// `None` under a key two closes disagreed on: one segment id and part can
    /// close twice in one connection, when a truncated segment resumes, and
    /// then nothing on the console says which of the two bases a tracking line
    /// belongs to. Such a segment converts nothing.
    segments: BTreeMap<SegKey, Option<Closed>>,
    /// The bearings each closed segment's tracking line carried, in that
    /// segment's own index space.
    ///
    /// Kept rather than folded into the turns as the line is read, because the
    /// turn a bearing belongs to is not always on the console yet when its
    /// segment's lines arrive: the segment closes at the VAD release while the
    /// utterance line waits for the recogniser to return, and a recogniser
    /// running long is what a degraded turn is. Every figure is a function of
    /// the whole map and none of the order the lines came in.
    ///
    /// The cost is one `(offset, bearing)` pair per few thousand samples of
    /// tracked audio — one numeric column of one event kind, still a fold and
    /// not the file.
    ///
    /// `None` under a key two tracking lines disagreed on, for the same reason
    /// two disagreeing closes withdraw a base.
    tracked: BTreeMap<SegKey, Option<Vec<(i64, f64)>>>,
    /// How many connections have announced themselves, which is the generation
    /// every segment and every turn read afterwards is stamped with.
    connection: u32,
    /// Barge-ins that cut no reply this console had dispatched.
    ///
    /// Counted apart rather than attributed to a guess: a barge-in names no
    /// utterance, and one arriving before anything was dispatched is the pod
    /// hearing something the report cannot join to a turn.
    stray_barge_ins: usize,
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
    ///
    /// A line is one event and nearly always exactly that, but a tear can put
    /// several on it, so every event the line carried is folded in the order it
    /// was written.
    fn absorb(&mut self, raw: &str) {
        self.seen += 1;
        let Classified {
            ahead,
            behind,
            lines,
        } = classify(raw);
        // The host's refusal is console text, and console text tears the same
        // way an event does — the refusal arrives glued behind a sentence that
        // never got its newline. So it is looked for inside the console halves
        // of the line rather than at the start of the raw one, on the halves
        // that are console text at all: a JSON event quoting the prefix is an
        // event, not an exit. The later half wins, because a tear puts the
        // refusal at the line's tail.
        let mut console: Vec<&str> = Vec::new();
        if let Some(text) = &ahead {
            console.push(text);
        }
        if lines.is_empty() {
            console.push(raw);
        }
        if let Some(text) = &behind {
            console.push(text);
        }
        let refusal = console.iter().rev().find_map(|text| refused_in(text));
        let refused_here = refusal.is_some();
        if let Some(message) = refusal {
            self.refused = Some(message);
        }
        for text in [&ahead, &behind].into_iter().flatten() {
            self.noticed_noise(quote(text));
        }
        if lines.is_empty() {
            self.noticed_noise(quote(raw));
            return;
        }
        for line in lines {
            self.fold(line);
        }
        // A line of the stream after a refusal means the process went on: the
        // refusal was something a run printed, not the way it ended. A line
        // that carried a refusal of its own is not such a line, whatever was
        // glued behind it.
        if !refused_here {
            self.refused = None;
        }
    }

    /// One event, into the counters it belongs to.
    fn fold(&mut self, line: Line) {
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
            Line::Turned {
                keys,
                event,
                turned,
            } => {
                self.turn_event(*turned);
                *self.pipeline.entry((event, keys)).or_default() += 1;
            }
        }
    }

    /// One event of the utterance lifecycle, into the turn it belongs to.
    ///
    /// The order the console carried them is the only join this has: a wake
    /// opens a turn, the utterance that follows binds its sequence number to
    /// that turn, and everything after joins on the number. An event whose
    /// number this console has not seen falls back to the open turn, and where
    /// there is none it opens one — a barge-in mints an utterance with no wake
    /// ahead of it, and that turn is as real as any other.
    fn turn_event(&mut self, turned: Turned) {
        match turned {
            Turned::Wake {
                pod,
                score,
                wake_end_sample,
            } => {
                self.turns.push(Turn {
                    ordinal: self.turns.iter().filter(|turn| turn.woke).count() + 1,
                    woke: true,
                    pod,
                    generation: self.connection,
                    wake_score: score,
                    wake_end_sample,
                    ..Turn::default()
                });
                self.open_turn = Some(self.turns.len() - 1);
            }
            // A connection fences the index space every sample read before it
            // was counted in: nothing read afterwards answers for a turn from
            // before it, and nothing before answers for a turn after.
            Turned::Connected => self.connection = self.connection.saturating_add(1),
            Turned::Said {
                spoken,
                stt_trim,
                stt_sent_from,
            } => {
                let at = self.turn_for(spoken.id);
                let turn = &mut self.turns[at];
                let mut span = spoken.span;
                span.absorb(std::mem::take(&mut turn.said.span));
                if turn.pod.is_empty() {
                    turn.pod = spoken.pod.clone();
                }
                turn.said = Spoken { span, ..*spoken };
                turn.stt_trim = turn.stt_trim.or(stt_trim);
                turn.stt_sent_from = turn.stt_sent_from.or(stt_sent_from);
            }
            // No utterance was minted, so there is no id to join on and no
            // open turn to consume: the hold belongs to the wake still standing,
            // and the command's own utterance line has yet to land on it. With no
            // wake standing this console never saw the wake line the hold belongs
            // to, and no other turn answers for it -- writing it onto whichever
            // turn came last would grow a fragment, and a tally, on a turn that
            // was never held.
            Turned::Held(held) => {
                if let Some(at) = self.open_turn {
                    self.turns[at].held = Some(held);
                }
            }
            Turned::Dispatched { id } => {
                let at = self.turn_for(id);
                self.turns[at].outcome = Outcome::Dispatched;
                self.last_dispatched = Some(at);
            }
            Turned::Declined {
                id,
                reason,
                no_speech,
                logprob,
                span,
                stt_trim,
            } => {
                let at = self.turn_for(id);
                let turn = &mut self.turns[at];
                turn.outcome = Outcome::Declined(reason);
                turn.stt_trim = turn.stt_trim.or(stt_trim);
                // The gate's own reading of the two numbers, where the
                // utterance line did not carry them: the same figures, and on
                // a turn whose transcript never reached this console they are
                // the only ones there are.
                turn.said.no_speech = turn.said.no_speech.or(no_speech);
                turn.said.logprob = turn.said.logprob.or(logprob);
                turn.said.span.absorb(span);
            }
            Turned::NoTranscript { id } => {
                let at = self.turn_for(id);
                self.turns[at].outcome = Outcome::NoTranscript;
            }
            Turned::SttFailed { id, detail } => {
                let at = self.turn_for(id);
                self.turns[at].outcome = Outcome::SttFailed(detail);
            }
            Turned::Superseded { id } => {
                let at = self.turn_for(id);
                self.turns[at].superseded += 1;
            }
            Turned::BargeIn => match self.last_dispatched {
                Some(at) => self.turns[at].barge_ins += 1,
                None => self.stray_barge_ins += 1,
            },
            // The two reply-side events name the turn they answer or name
            // nothing at all: the pipeline plays announcements nobody asked a
            // question for, and one of those is no turn's reply. An unnamed one
            // is left out rather than charged to whichever turn is open, which
            // would read as a wake that was answered.
            Turned::Flushed { id: Some(id) } => {
                let at = self.turn_for(Some(id));
                self.turns[at].flushed += 1;
            }
            Turned::Played {
                id: Some(id),
                played_ms,
            } => {
                let at = self.turn_for(Some(id));
                self.turns[at].played_ms = played_ms;
            }
            Turned::Flushed { id: None } | Turned::Played { id: None, .. } => {}
            Turned::SegmentClosed {
                pod,
                segment,
                part,
                base_sample,
                samples,
            } => {
                if let (Some(segment), Some(base)) = (segment, base_sample) {
                    let closed = Closed { base, samples };
                    let key = self.key(pod, segment, part);
                    self.segments
                        .entry(key)
                        .and_modify(|held| {
                            if *held != Some(closed) {
                                *held = None;
                            }
                        })
                        .or_insert(Some(closed));
                }
            }
            Turned::Tracking {
                pod,
                segment,
                part,
                beams,
            } => {
                if let Some(segment) = segment
                    && !beams.is_empty()
                {
                    let key = self.key(pod, segment, part);
                    self.tracked
                        .entry(key)
                        .and_modify(|held| {
                            if held.as_deref() != Some(beams.as_slice()) {
                                *held = None;
                            }
                        })
                        .or_insert(Some(beams));
                }
            }
        }
    }

    /// The turn one event belongs to, opening one where nothing holds it yet.
    fn turn_for(&mut self, id: Option<u64>) -> usize {
        if let Some(id) = id
            && let Some(at) = self.by_id.get(&id)
        {
            return *at;
        }
        let at = match self.open_turn.take() {
            Some(at) => at,
            None => {
                self.turns.push(Turn {
                    generation: self.connection,
                    ..Turn::default()
                });
                self.turns.len() - 1
            }
        };
        if let Some(id) = id {
            self.by_id.insert(id, at);
            self.turns[at].said.id = Some(id);
        }
        at
    }

    /// Which closed segment a turn whose span names none was carved from.
    ///
    /// The carve's own start decides it, because that is the sample the
    /// pipeline judged belonged to this utterance. Where no closed range holds
    /// that sample the wake decides instead: the pod's pre-roll is whatever
    /// history its ring held, so the first segment of a connection can begin
    /// after the host's own carve does, while the wake word that opened the
    /// segment is inside it by construction.
    ///
    /// Two ranges holding one anchor is no attribution at all. Two connections
    /// of a pod count samples in unrelated spaces, and a segment id says
    /// nothing across them, so one sample can sit inside two segments that
    /// share no audio: the figure such a pair would print is a confident
    /// reading of the wrong conversation. The generation is what keeps the
    /// two connections apart in the first place, and a carve one of them holds
    /// is judged against that connection's segments alone.
    fn attribution(&self, turn: &Turn) -> Attribution<'_> {
        for anchor in [turn.carved_at(), turn.wake_end_sample]
            .into_iter()
            .flatten()
        {
            let mut held = self.segments.iter().filter(|(key, closed)| {
                self.answers_for(turn, key) && closed.is_some_and(|closed| closed.holds(anchor))
            });
            match (held.next(), held.next()) {
                (Some((key, _)), None) => return Attribution::One(key),
                (Some(_), Some(_)) => return Attribution::Several,
                _ => {}
            }
        }
        Attribution::None
    }

    /// Whether a turn and a segment line agree about whose samples these are.
    ///
    /// A line that named no pod agrees with everything: an older pipeline than
    /// this tool reads for names one on some events and not others, and a
    /// console with one pod on it is every console this tool has been pointed
    /// at.
    fn same_space(one: &str, other: &str) -> bool {
        one.is_empty() || other.is_empty() || one == other
    }

    /// Whether a closed segment can answer for a turn at all.
    ///
    /// Same pod, same connection: a segment id, and every sample index beside
    /// it, means something only inside one connection of one pod.
    fn answers_for(&self, turn: &Turn, key: &SegKey) -> bool {
        Self::same_space(&turn.pod, &key.pod) && key.generation == turn.generation
    }

    /// The key a segment line read now is stamped with.
    fn key(&self, pod: String, segment: u32, part: u16) -> SegKey {
        SegKey {
            pod,
            generation: self.connection,
            segment,
            part,
        }
    }

    /// Which closed segment a turn reads its bearings out of.
    ///
    /// A span that names one is judged by that segment and no other, matched
    /// on its part as well as its id because one id can close more than once.
    /// A span that names none is attributed by [`Console::attribution`], to
    /// the one closed segment whose sample range holds it: the pipeline names
    /// a segment list only for a carve some segment had already closed over,
    /// so the first utterance of every run names none, and containment is what
    /// gives that turn — the one the beam figure is most wanted for — a
    /// segment at all.
    fn segment_of(&self, turn: &Turn) -> Option<&SegKey> {
        match turn.said.span.segment {
            Some(named) => self.segments.keys().find(|key| {
                key.segment == named
                    && turn.said.span.part.is_none_or(|part| part == key.part)
                    && self.answers_for(turn, key)
            }),
            None => match self.attribution(turn) {
                Attribution::One(key) => Some(key),
                Attribution::Several | Attribution::None => None,
            },
        }
    }

    /// The auto-select beam through one turn's span: mean and spread, in
    /// radians, or nothing where this console cannot state it.
    ///
    /// Read against the whole fold rather than as the tracking line arrives,
    /// because the console's order is not the turns' order: a segment closes
    /// at the VAD release and its utterance line waits for the recogniser, so
    /// a turn's span can trail its own segment's lines by the whole of an STT
    /// call — which on a degraded turn is exactly what is long. A figure that
    /// depended on which came first would go missing on the turns this report
    /// exists to read.
    ///
    /// A turn whose span states no end sample keeps no figure. Nothing on the
    /// console says where such a turn stopped, so the segment's own end is not
    /// a bound it has any claim to: it is whatever the gate did afterwards, a
    /// later turn's utterance included, and a mean over another speaker is the
    /// one shape that could mislead the reading two acceptance sessions are
    /// compared on. Every turn the console finished telling states its end,
    /// an expired arm included.
    ///
    /// The window converts between the two index spaces. A turn's wake and
    /// span are pod-absolute; the bearings are stamped relative to the
    /// segment's first sample. Without that base sample on the console there
    /// is no conversion and the turn keeps no figure.
    ///
    /// It opens at [`Turn::window_from`] and the segment is chosen by
    /// [`Turn::carved_at`], which are deliberately opposite precedences: a
    /// carve that straddles a close has its wake in the segment after the one
    /// holding it, so the window would open outside this segment and select
    /// nothing. A turn attributed by containment then opens its window at the
    /// sample that attributed it, so a successful attribution never reads as
    /// missing.
    fn beam_of(&self, turn: &Turn) -> Option<(f64, f64)> {
        let end = turn.said.span.end_sample?;
        let key = self.segment_of(turn)?;
        let closed = (*self.segments.get(key)?)?;
        let beams = self.tracked.get(key)?.as_deref()?;
        let base = closed.base;
        let mut opens = turn.window_from().unwrap_or(base);
        if turn.said.span.segment.is_none() && !closed.holds(opens) {
            opens = turn.carved_at().unwrap_or(base);
        }
        let from = opens.saturating_sub(base);
        let until = end.saturating_sub(base);
        let held: Vec<f64> = beams
            .iter()
            .filter(|(offset, _)| *offset >= from && *offset < until)
            .map(|(_, bearing)| *bearing)
            .collect();
        spread(&held)
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

/// The mean and the spread about it of a handful of readings, where there are
/// any.
///
/// The spread is the population standard deviation and not an interval: the
/// readings are the whole of what the run held, not a sample of something
/// larger, and what an operator reads it for is whether one turn's bearings sat
/// still or wandered.
fn spread(readings: &[f64]) -> Option<(f64, f64)> {
    if readings.is_empty() {
        return None;
    }
    let n = readings.len() as f64;
    let mean = readings.iter().sum::<f64>() / n;
    let variance = readings
        .iter()
        .map(|reading| (reading - mean).powi(2))
        .sum::<f64>()
        / n;
    Some((mean, variance.sqrt()))
}

/// A directory named beside `records` by suffixing the record directory's own
/// name.
///
/// Built from the record path's own spelling rather than from a sibling scan:
/// the set is named by construction, and a directory holding two runs would
/// otherwise be ambiguous. Three siblings are spelled this way — the console
/// and the audio store the fetch wrote, and the turn clips this tool writes.
fn sibling(records: &Path, suffix: &str) -> PathBuf {
    let mut name = records
        .file_name()
        .map(|name| name.to_string_lossy().into_owned())
        .unwrap_or_default();
    name.push_str(suffix);
    records.with_file_name(name)
}

/// The console directory the fetch wrote beside `records`.
fn console_dir(records: &Path) -> PathBuf {
    sibling(records, CONSOLE_SUFFIX)
}

/// What one raw line is: any console text around an event, and the line.
struct Classified {
    /// The console text a recovered event was glued onto, as it was written.
    /// Absent on every line that is one thing, which is nearly all of them.
    ahead: Option<String>,
    /// The console text glued onto the last event's own end, the same way. The
    /// tear runs both directions: an event written whole and a sentence begun
    /// into the same descriptor behind it.
    behind: Option<String>,
    /// The events on the line, in the order they were written. Empty where the
    /// line carries none, which is what console text is.
    lines: Vec<Line>,
}

/// What one raw line is, recovering an event glued onto console text.
///
/// The host's two output streams are one file: the launcher redirects both into
/// `voice_host_0.log`, and they tear — a console sentence still without its
/// newline, and a JSONL event written into the same descriptor behind it. A
/// whole-line parse reads every torn line as noise. So a line that does not
/// parse whole is read as the whole objects it holds and whatever text
/// surrounds them: the objects are the events and the text on either side is
/// kept as noise. A line with no whole object anywhere in it stays noise, which
/// is what a sentence merely quoting that spelling is.
///
/// The tear runs both directions, and both are the ordinary shape rather than
/// the exotic one — an event glued onto an unterminated sentence, and a
/// sentence begun into the same descriptor behind a whole event. The second is
/// how a transcript's own line arrives, so a reader that recovers only the
/// first loses exactly the events a turn is assembled from.
///
/// Every occurrence of an [`EVENT_HEAD`] is tried rather than the first,
/// because the console text a real event is glued onto can hold that spelling
/// too — a sentence quoting an event, or a half-written event glued ahead of a
/// whole one. Splitting at the first match alone loses the real event at the
/// line's end, and a lost `brain_brenn` is a run that exempts itself from
/// [`bridged`].
///
/// And every whole event from that point on is kept, not only the first: two
/// writers interleaving cleanly put two finished events in one descriptor with
/// no newline between them, and the second is as real as the first. Only the
/// text after the last whole object is noise.
fn classify(raw: &str) -> Classified {
    let starts = std::iter::once(0).chain(raw.match_indices(EVENT_HEAD).map(|(at, _)| at));
    for at in starts {
        let (lines, until) = leading(&raw[at..]);
        if lines.is_empty() {
            continue;
        }
        let behind = &raw[at + until..];
        return Classified {
            ahead: (at > 0).then(|| raw[..at].to_owned()),
            behind: (!behind.is_empty()).then(|| behind.to_owned()),
            lines,
        };
    }
    Classified {
        ahead: None,
        behind: None,
        lines: Vec::new(),
    }
}

/// Every whole JSON object from the start of some text, and how much of the
/// text they took.
///
/// A streaming read rather than a whole-text parse: trailing content is what a
/// tear leaves behind an event, and a parse that rejects the line for it throws
/// away the event with it. The read stops at the first thing that is not a
/// whole object — a half-written event, or a scalar somebody printed — and
/// whatever is left is the caller's noise.
fn leading(raw: &str) -> (Vec<Line>, usize) {
    let mut values = serde_json::Deserializer::from_str(raw).into_iter::<Value>();
    let mut lines = Vec::new();
    let mut until = 0;
    while let Some(Ok(value)) = values.next() {
        let Some(line) = object(value) else {
            break;
        };
        until = values.byte_offset();
        lines.push(line);
    }
    (lines, until)
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

/// What one JSON value is, or nothing where it is not an object.
///
/// A scalar or an array is not an event: the stream is objects, and anything
/// else on the console is somebody's print.
fn object(value: Value) -> Option<Line> {
    let Value::Object(object) = value else {
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
                _ => match turned(&event, &object) {
                    Some(turned) => Line::Turned {
                        keys,
                        event,
                        turned: Box::new(turned),
                    },
                    None => Line::Pipeline { event, keys },
                },
            }
        }
    })
}

/// A number anywhere in an object, as one, or nothing where it is not written
/// as a number.
///
/// Every figure a turn line prints goes through here, so a pipeline that stops
/// writing one of them as a number prints it as missing rather than as a zero
/// nobody can tell from a reading.
fn number(object: &serde_json::Map<String, Value>, key: &str) -> Option<f64> {
    object.get(key).and_then(Value::as_f64)
}

/// A whole number, the same way.
fn whole(object: &serde_json::Map<String, Value>, key: &str) -> Option<i64> {
    object.get(key).and_then(Value::as_i64)
}

/// An utterance's sequence number off whichever field this event names it in.
///
/// Three spellings, because the pipeline names an utterance three ways: the
/// utterance's own line calls it `id`, the events that answer it call it
/// `utterance`, and the recogniser's failure calls it `utterance_seq`. A
/// supersession names the whole identity and the sequence is inside it.
fn utterance_id(object: &serde_json::Map<String, Value>) -> Option<u64> {
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

/// The span an event names, out of the fields it names it in.
///
/// Two shapes: the utterance's line nests it under `audio_ref`, and the gate's
/// decline writes the same fields at the top level. The segment is the first
/// the span lists, which is the segment the turn's own line named; the log and
/// the two samples are what the clip is cut by, and they are stated whether or
/// not any segment is.
fn span_of(object: &serde_json::Map<String, Value>) -> Span {
    let held = object.get("audio_ref").and_then(Value::as_object);
    let at = held.unwrap_or(object);
    let first = at
        .get("segments")
        .and_then(Value::as_array)
        .and_then(|segments| segments.first())
        .and_then(Value::as_object);
    Span {
        log: at
            .get("log")
            .and_then(Value::as_str)
            .filter(|log| !log.is_empty())
            .map(str::to_owned),
        start_sample: whole(at, "start_sample"),
        end_sample: whole(at, "end_sample"),
        segment: first
            .and_then(|first| first.get("segment_id"))
            .and_then(Value::as_u64)
            .and_then(|id| u32::try_from(id).ok()),
        part: first
            .and_then(|first| first.get("part"))
            .and_then(Value::as_u64)
            .and_then(|part| u16::try_from(part).ok()),
    }
}

/// Which pod an event names, or nothing where it names none.
///
/// Two spellings: the transport's own lines call it `pod` and the connection's
/// hello calls it `pod_id`.
fn pod_of(object: &serde_json::Map<String, Value>) -> String {
    for key in ["pod", "pod_id"] {
        if let Some(pod) = object.get(key).and_then(Value::as_str) {
            return pod.to_owned();
        }
    }
    String::new()
}

/// Which cap-rollover part of its segment a segment line names, zero where it
/// names none.
fn part_of(object: &serde_json::Map<String, Value>) -> u16 {
    object
        .get("audio_ref")
        .and_then(Value::as_object)
        .and_then(|held| held.get("part"))
        .and_then(Value::as_u64)
        .and_then(|part| u16::try_from(part).ok())
        .unwrap_or_default()
}

/// What one transcript's line said, out of the fields it carries.
fn spoken(object: &serde_json::Map<String, Value>) -> Spoken {
    let transcript = object.get("transcript").and_then(Value::as_object);
    let confidence = transcript
        .and_then(|held| held.get("confidence"))
        .and_then(Value::as_object);
    Spoken {
        id: utterance_id(object),
        pod: pod_of(object),
        text: transcript
            .and_then(|held| held.get("text"))
            .and_then(Value::as_str)
            .map(str::to_owned),
        no_speech: confidence.and_then(|held| number(held, "no_speech_prob")),
        logprob: confidence.and_then(|held| number(held, "avg_logprob")),
        compression: confidence.and_then(|held| number(held, "compression_ratio")),
        endpoint_cause: object
            .get("endpoint_cause")
            .and_then(Value::as_str)
            .unwrap_or_default()
            .to_owned(),
        span: span_of(object),
    }
}

/// The auto-select beam's bearings out of one tracking line's `doa` array.
///
/// Each entry is an offset and one bearing per beam. An entry that is not that
/// shape, or that carries fewer beams than this build reads, is skipped: the
/// figure is a mean over the samples that were readable, and how many there
/// were is what the spread already says.
fn beams(object: &serde_json::Map<String, Value>) -> Vec<(i64, f64)> {
    let Some(doa) = object.get("doa").and_then(Value::as_array) else {
        return Vec::new();
    };
    doa.iter()
        .filter_map(|entry| {
            let entry = entry.as_array()?;
            let offset = entry.first()?.as_i64()?;
            let bearing = entry.get(1)?.as_array()?.get(AUTO_BEAM)?.as_f64()?;
            Some((offset, bearing))
        })
        .collect()
}

/// Which event of the utterance lifecycle this is, where it is one.
///
/// One arm per event name, which with the arm [`Console::turn_event`] holds is
/// the two edits an event wanted by field costs. An event this build has no
/// reader for answers nothing and stays the summary every other event is.
fn turned(event: &str, object: &serde_json::Map<String, Value>) -> Option<Turned> {
    let id = utterance_id(object);
    let segment = object
        .get("segment_id")
        .and_then(Value::as_u64)
        .and_then(|id| u32::try_from(id).ok());
    let reason = object
        .get("reason")
        .and_then(Value::as_str)
        .unwrap_or_default()
        .to_owned();
    Some(match event {
        WAKE_DETECTED => Turned::Wake {
            pod: pod_of(object),
            score: number(object, "score"),
            wake_end_sample: whole(object, "wake_end_sample"),
        },
        CONN_HELLO => Turned::Connected,
        WAKE_HELD => Turned::Held(Held {
            start_sample: whole(object, "start_sample"),
            end_sample: whole(object, "end_sample"),
            deadline_sample: whole(object, "deadline_sample"),
        }),
        UTTERANCE => Turned::Said {
            spoken: Box::new(spoken(object)),
            stt_trim: whole(object, "stt_trim_samples"),
            stt_sent_from: whole(object, "stt_sent_from_sample"),
        },
        BRAIN_DISPATCHED => Turned::Dispatched { id },
        WAKE_COMMAND_ABSENT | BARGE_COMMAND_ABSENT => Turned::Declined {
            id,
            reason,
            no_speech: number(object, "no_speech"),
            logprob: number(object, "logprob"),
            span: span_of(object),
            stt_trim: whole(object, "stt_trim_samples"),
        },
        BRAIN_NO_TRANSCRIPT => Turned::NoTranscript { id },
        STT_FAILED => Turned::SttFailed {
            id,
            detail: object
                .get("detail")
                .and_then(Value::as_str)
                .unwrap_or_default()
                .to_owned(),
        },
        UTTERANCE_SUPERSEDED => Turned::Superseded { id },
        BARGE_IN => Turned::BargeIn,
        PLAYBACK_FLUSHED => Turned::Flushed { id },
        PLAYBACK_FINISHED => Turned::Played {
            id,
            played_ms: number(object, "nominal_audio_ms"),
        },
        SEGMENT_CLOSED => Turned::SegmentClosed {
            pod: pod_of(object),
            segment,
            part: part_of(object),
            base_sample: whole(object, "base_sample"),
            samples: whole(object, "samples"),
        },
        TRACKING => Turned::Tracking {
            pod: pod_of(object),
            segment,
            part: part_of(object),
            beams: beams(object),
        },
        _ => return None,
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

/// A confidence reading, printed to the precision it is read at.
///
/// Two decimals down to a tenth and three below it: the gate sits at `0.2` and
/// the readings that pass it are hundredths of that, so a fixed width either
/// rounds the clean ones to nothing or pads the declined ones with digits the
/// recogniser did not mean.
fn reading(value: f64) -> String {
    if value.abs() >= 0.1 {
        format!("{value:.2}")
    } else {
        format!("{value:.3}")
    }
}

/// A figure that may not be on this console, printed as itself or as missing.
fn figure(value: Option<f64>) -> String {
    value.map(reading).unwrap_or_else(|| "?".to_owned())
}

/// The range a set of readings covered, or that there were none.
fn range(readings: &[f64]) -> String {
    let (Some(low), Some(high)) = (
        readings.iter().copied().reduce(f64::min),
        readings.iter().copied().reduce(f64::max),
    ) else {
        return "none".to_owned();
    };
    if reading(low) == reading(high) {
        reading(low)
    } else {
        format!("{}–{}", reading(low), reading(high))
    }
}

/// What one turn was, on one line.
fn turn_line(console: &Console, clips: &Clips, at: usize) -> String {
    let turn = &console.turns[at];
    let who = turn
        .label()
        .map_or_else(|| "#? (no wake)".to_owned(), Label::on_line);
    let wake = turn
        .wake_score
        .map(|score| format!("{score:.2}"))
        .unwrap_or_else(|| "none".to_owned());
    let mut line = format!("turn {who} — wake {wake} → ");
    match &turn.said.text {
        Some(text) => {
            line.push_str(&format!(
                "\"{}\" no_speech={} logprob={}",
                quote(text),
                figure(turn.said.no_speech),
                figure(turn.said.logprob)
            ));
            if let Some(compress) = turn.said.compression {
                line.push_str(&format!(" compress={}", reading(compress)));
            }
        }
        // A wake the pipeline minted no transcript for. Its confidence figures
        // are still printed where the gate carried them, which is the whole of
        // what is known about what was said.
        None => line.push_str(&format!(
            "no transcript (no_speech={} logprob={})",
            figure(turn.said.no_speech),
            figure(turn.said.logprob)
        )),
    }
    line.push_str(&match &turn.outcome {
        Outcome::Open => " → no outcome on this console".to_owned(),
        Outcome::Dispatched => " → dispatched".to_owned(),
        Outcome::Declined(reason) => format!(" → declined ({})", quote(reason)),
        Outcome::NoTranscript => " → the brain was handed no words".to_owned(),
        Outcome::SttFailed(detail) => format!(" → stt failed ({})", quote(detail)),
    });
    if let Some(played) = turn.played_ms {
        line.push_str(&format!("; reply played {:.1} s", played / 1000.0));
    }
    if !turn.said.endpoint_cause.is_empty() {
        line.push_str(&format!("; endpoint {}", quote(&turn.said.endpoint_cause)));
    }
    if let Some(held) = turn.held {
        line.push_str(&match held.waited_s() {
            Some(waited) => format!("; held up to {waited:.1} s for the command"),
            None => "; held for the command".to_owned(),
        });
    }
    for (count, what) in [
        (turn.superseded, "superseded"),
        (turn.barge_ins, "barge-in"),
        (turn.flushed, "reply flushed"),
    ] {
        if count > 0 {
            line.push_str(&format!("; {what} ×{count}"));
        }
    }
    line.push_str(&clip(console, clips, at));
    line.push_str(&match console.beam_of(turn) {
        Some((mean, spread)) => format!("; auto beam {mean:.2}±{spread:.2} rad"),
        None => "; auto beam ?".to_owned(),
    });
    line
}

/// Where this turn's clip is, on the line that reads the turn.
///
/// The file the clip was written to, or why it was not, and the pod-absolute
/// span it covers either way: the span is the window the beam figure is read
/// over and the coordinates anyone opening the frame log by other means needs,
/// so it stays on the line whether or not a `.wav` exists.
///
/// A turn whose span states no log or no sample bounds, and a turn with no
/// number to name a file by, print as missing — the report's rule for a value
/// it cannot state.
fn clip(console: &Console, clips: &Clips, at: usize) -> String {
    let turn = &console.turns[at];
    // A turn whose line tore in the field naming the log states no file and
    // still states where in the store its audio is.
    let span = match (turn.said.span.start_sample, turn.said.span.end_sample) {
        (Some(start), Some(end)) => format!(" [{start}–{end})"),
        _ => String::new(),
    };
    let Some(outcome) = clips.of(at) else {
        return format!("; clip ?{span}");
    };
    let mut said = format!("; clip {}{span}", outcome.what);
    said.push_str(&outcome.notes);
    // The boundary and what was done with it are two figures: a run configured
    // to keep the wake word in the clip computes the first and starts at zero.
    let seconds =
        |samples: i64| samples as f64 / f64::from(speech_pipeline::SPINE_FORMAT.sample_rate_hz);
    if let Some(trim) = turn.stt_trim {
        said.push_str(&format!(", STT boundary +{:.2} s", seconds(trim)));
    }
    if let Some(sent_from) = turn.stt_sent_from {
        said.push_str(&format!(", sent from +{:.2} s", seconds(sent_from)));
    }
    said
}

/// What became of one turn's clip: what the line says it is, and what else the
/// resolver had to say about the audio behind it.
#[derive(Debug)]
struct Clip {
    /// The file name, or `not written (<reason>)`, as the turn line prints it.
    what: String,
    /// Whatever qualifies the audio that was written: how much of it is
    /// silence, whether the log's tail was torn, how many protocol errors the
    /// replay met. Empty on a clip nobody could write.
    notes: String,
    /// Why nothing was written, where nothing was. The summary line names it
    /// when every unwritten turn of the run shares it.
    unwritten: Option<String>,
    /// Whether a file was written.
    written: bool,
}

impl Clip {
    /// A turn whose audio never became a file, in the words its line prints.
    fn not_written(reason: String) -> Self {
        Self {
            what: format!("not written ({reason})"),
            notes: String::new(),
            unwritten: Some(reason),
            written: false,
        }
    }
}

/// One `.wav` per turn, cut out of the store the fetch brought home.
///
/// Writing is a side effect and [`analyze`] is pure over what was read, so the
/// clips are cut before the reading and handed to it: the turn lines say which
/// file holds which turn, and nothing outside this repository is run to hear
/// one.
///
/// Keyed by the turn's index rather than its utterance id, because not every
/// turn has an id — a wake nobody answered has none, and a torn line can lose
/// one.
#[derive(Debug, Default)]
struct Clips {
    by_turn: BTreeMap<usize, Clip>,
    /// Where they went, for the line that counts them.
    into: PathBuf,
}

impl Clips {
    /// Cut every turn the console carved a span for out of `store`, writing
    /// into `into`.
    ///
    /// The store is checked once, not once per turn: a site whose configuration
    /// records nothing leaves an empty directory behind (the fetch's rsync of a
    /// store with nothing in it succeeds), and that is one sentence about the
    /// run rather than one refusal per turn.
    ///
    /// A store that would not open is not a store that recorded nothing: the
    /// fault is said, and never in the words for an empty store.
    fn write(console: &Console, store: &Path, into: &Path) -> Self {
        let mut held = Self {
            into: into.to_path_buf(),
            ..Self::default()
        };
        let recorded = match std::fs::read_dir(store) {
            Ok(mut entries) => Ok(entries.any(|entry| entry.is_ok())),
            Err(why) if why.kind() == std::io::ErrorKind::NotFound => Ok(false),
            Err(why) => Err(format!("{}: {why}", store.display())),
        };
        // The directory is made once, by the first turn that has audio to
        // write, and a directory that cannot be made is one fact about the
        // filesystem rather than one refusal per turn.
        let mut made = None;
        for (at, turn) in console.turns.iter().enumerate() {
            let Some((label, log, start, end)) = named(turn) else {
                continue;
            };
            match &recorded {
                Ok(true) => {}
                Ok(false) => {
                    held.by_turn.insert(
                        at,
                        Clip::not_written("no recorded audio beside this fetch".to_owned()),
                    );
                    continue;
                }
                Err(why) => {
                    held.by_turn.insert(at, Clip::not_written(why.clone()));
                    continue;
                }
            }
            held.by_turn.insert(
                at,
                cut(
                    label,
                    log,
                    start,
                    end,
                    turn.stt_trim,
                    store,
                    into,
                    &mut made,
                ),
            );
        }
        held
    }

    fn of(&self, at: usize) -> Option<&Clip> {
        self.by_turn.get(&at)
    }
}

/// What this turn's file would be called and what it would hold, where the
/// console said enough for both.
///
/// The name is [`Turn::label`]'s, which is the token the turn line prints after
/// `#`, so the clip is found from the report by reading. A turn the console
/// named neither an utterance nor a wake for cannot be named and is not cut.
fn named(turn: &Turn) -> Option<(Label, String, i64, i64)> {
    let label = turn.label()?;
    let log = turn.said.span.log.clone()?;
    Some((
        label,
        log,
        turn.said.span.start_sample?,
        turn.said.span.end_sample?,
    ))
}

/// Why one log's replay stopped short of the clip it was asked for.
///
/// A writer killed mid-write is routine and uninteresting; a corrupt record or
/// a parked protocol is the store itself failing, and the message the resolver
/// carries is the only description of what was wrong with it. Collapsed into
/// one word they read alike, and telling them apart afterwards means running a
/// tool from the other repository.
fn stopped_because(stop: &pod_ingest::SpliceStop) -> String {
    match stop {
        pod_ingest::SpliceStop::TornTail => "torn tail".to_owned(),
        pod_ingest::SpliceStop::Corrupt(why) => format!("corrupt: {}", quote(why)),
        pod_ingest::SpliceStop::ProtocolFatal => "protocol fatal".to_owned(),
    }
}

/// Cut one turn's carve out of the store and write it.
///
/// The span is built from the turn's own line and names no covering segment:
/// the resolver's segment refs are provenance and not a filter, and the console
/// leaves them empty on exactly the turns whose recogniser ran long — which are
/// the turns this report exists for.
///
/// A store fault is never mistaken for silence: it is said in the fragment and
/// the report goes on. A partial clip is written all the same and says what is
/// wrong with it, because partial audio beats none.
///
/// The log name is console-authored text and must be quoted: a name carrying
/// a newline would otherwise fabricate a line of this report.
///
/// `made` carries the one attempt at creating the output directory, made by the
/// first turn that gets as far as audio to write.
///
/// Where the wake-trim boundary is known, a second file holds the same span from
/// that boundary, so it is opened and heard rather than sought to. That is what
/// the recogniser was given when the run trimmed the wake word; under `[stt]
/// wake_word = "keep"` the recogniser got the whole first file instead, and the
/// turn line's `sent from` offset says which. It is a suffix of the first and
/// carries nothing the turn line does not, and it comes off the one decode this
/// call already paid for -- a second resolve would double the cost the TODO
/// below is about.
///
/// TODO(clip-one-pass-per-log): every turn of a run is carved from the same log
/// and each call here decodes it from the head, so a session's clips cost work
/// quadratic in its turns.
#[allow(clippy::too_many_arguments)]
fn cut(
    label: Label,
    log: String,
    start: i64,
    end: i64,
    stt_trim: Option<i64>,
    store: &Path,
    into: &Path,
    made: &mut Option<Result<(), String>>,
) -> Clip {
    let name = label.file_name();
    let name = name.as_str();
    // A sample index off a line that tore is a number this tool does not carve
    // with: the same fact the resolver names `InvalidSpan`, said in the same
    // words, rather than a panic in the report the run is read by.
    let (Ok(start_sample), Ok(end_sample)) = (u64::try_from(start), u64::try_from(end)) else {
        return Clip::not_written("invalid span".to_owned());
    };
    let span = speech_pipeline::AudioSpan {
        log: log.clone(),
        start_sample,
        end_sample,
        segments: Vec::new(),
    };
    let audio = match span.resolve(store) {
        Ok(audio) => audio,
        Err(speech_pipeline::SpanResolveError::InvalidSpan { .. }) => {
            return Clip::not_written("invalid span".to_owned());
        }
        Err(speech_pipeline::SpanResolveError::Resolve { log, source }) => {
            return Clip::not_written(format!("{}: {source}", quote(&log)));
        }
    };
    if audio.pruned.iter().any(|part| part.log == log) {
        return Clip::not_written(format!("{} is not in the store", quote(&log)));
    }
    if let Err(why) = made.get_or_insert_with(|| {
        std::fs::create_dir_all(into).map_err(|why| format!("{}: {why}", into.display()))
    }) {
        return Clip::not_written(why.clone());
    }
    if let Err(why) = speech_pipeline::write_spine_wav(&into.join(name), &audio.pcm) {
        return Clip::not_written(format!("{name}: {why}"));
    }
    let mut notes = String::new();
    // A boundary past the decoded audio states no suffix to write, and a write
    // that fails leaves the whole clip on disk regardless. Nothing is wrong with
    // that clip either way, so both are a note and not a refusal -- a refusal
    // here would tell the operator the audio they can open does not exist.
    if let Some(trim) = stt_trim {
        let command = label.command_file_name();
        match usize::try_from(trim)
            .ok()
            .and_then(|trim| audio.pcm.get(trim..))
        {
            Some(tail) => {
                if let Err(why) = speech_pipeline::write_spine_wav(&into.join(&command), tail) {
                    notes.push_str(&format!(", no {command} ({why})"));
                }
            }
            None => notes.push_str(&format!(
                ", no {command} (the STT boundary is outside this clip)"
            )),
        }
    }
    // `covered_samples` counts overlapping copies, so it is an upper bound on
    // distinct audio and `pcm.len() - covered_samples` is an upper bound on
    // silence when positive. A count past the carve's own length means the
    // store re-covered part of it, and the silence figure is then unknown
    // rather than zero: an outage read as silence is the one wrong inference.
    match audio
        .pcm
        .len()
        .checked_sub(usize::try_from(audio.covered_samples).unwrap_or(usize::MAX))
    {
        Some(0) => {}
        Some(silence) => notes.push_str(&format!(", at most {silence} samples of silence")),
        None => notes.push_str(
            ", how much of it is silence is unknown (the store covered part of this span twice)",
        ),
    }
    for (log, stop) in &audio.stopped {
        notes.push_str(&format!(
            ", {} torn ({})",
            quote(log),
            stopped_because(stop)
        ));
    }
    if audio.protocol_errors > 0 {
        notes.push_str(&format!(", {} protocol errors", audio.protocol_errors));
    }
    Clip {
        what: name.to_owned(),
        notes,
        unwritten: None,
        written: true,
    }
}

/// The turns of the conversation, one line each, and what the gate did to them.
///
/// The measured half and never a finding: a wake with nothing said after it
/// hallucinates and is rightly declined, and a run where nobody was answered is
/// a run a person reads rather than one this tool holds an opinion about. What
/// it does hold is that the decline be *visible* — a run of five wakes and five
/// declines would otherwise close as a pipeline that came up and drained whole.
fn turns(console: &Console, clips: &Clips, report: &mut Report) {
    for at in 0..console.turns.len() {
        report.note(turn_line(console, clips, at));
    }
    clips_written(console, clips, report);
    let count = |wanted: fn(&Turn) -> bool| console.turns.iter().filter(|t| wanted(t)).count();
    // The gate's own declines, apart from the other ways a wake goes
    // unanswered. An arm that expired mints an utterance with no transcript, and
    // an empty transcript is declined without the gate reading anything, and
    // counting either as a confidence decline would move the one figure two
    // acceptance sessions are compared on for a reason that has nothing to do
    // with the audio.
    let declined =
        count(|t| matches!(&t.outcome, Outcome::Declined(reason) if reason == LOW_CONFIDENCE));
    let mut otherwise: BTreeMap<&str, usize> = BTreeMap::new();
    for turn in &console.turns {
        if let Outcome::Declined(reason) = &turn.outcome
            && reason != LOW_CONFIDENCE
        {
            // An arm that expired after the wake was held waited out a command
            // that never came; a bare one was never followed at all. Counting
            // them under one word would hide the wait, which is the part an
            // operator can tune.
            let word = match reason.as_str() {
                ARM_EXPIRED if turn.held.is_some() => "arm_expired after a hold",
                "" => "unsaid",
                said => said,
            };
            *otherwise.entry(word).or_default() += 1;
        }
    }
    let barge_ins: usize =
        console.turns.iter().map(|t| t.barge_ins).sum::<usize>() + console.stray_barge_ins;
    // Wakes counted from the turns a wake actually opened. A barge-in mints a
    // turn with no wake ahead of it, and counting those as wakes would report
    // more than the model fired — in the one line two sessions are compared on.
    let woke = count(|t| t.woke);
    let unwoken = console.turns.len() - woke;
    report.note(format!(
        "{woke} wake(s): {} dispatched, {declined} declined by the confidence gate, {} with no \
         transcript, {} STT failure(s), {barge_ins} barge-in(s){}{}",
        count(|t| t.outcome == Outcome::Dispatched),
        count(|t| t.said.text.is_none()),
        count(|t| matches!(t.outcome, Outcome::SttFailed(_))),
        if otherwise.is_empty() {
            String::new()
        } else {
            format!(
                ", {} unanswered for other reasons ({})",
                otherwise.values().sum::<usize>(),
                otherwise
                    .iter()
                    .map(|(reason, count)| format!("{} ×{count}", quote(reason)))
                    .collect::<Vec<String>>()
                    .join(", ")
            )
        },
        if unwoken > 0 {
            format!("; {unwoken} turn(s) began with no wake ahead of them")
        } else {
            String::new()
        },
    ));
    let held_out = count(|turn| {
        turn.held.is_some() && matches!(&turn.outcome, Outcome::Declined(r) if r == ARM_EXPIRED)
    });
    if held_out > 0 {
        report.note(format!(
            "{held_out} wake(s) held for a command that never came"
        ));
    }
    let readings = |wanted: fn(&Turn) -> bool| -> Vec<f64> {
        console
            .turns
            .iter()
            .filter(|turn| wanted(turn))
            .filter_map(|turn| turn.said.no_speech)
            .collect()
    };
    report.note(format!(
        "no_speech: dispatched {}; declined {}",
        range(&readings(|t| t.outcome == Outcome::Dispatched)),
        range(&readings(|t| matches!(t.outcome, Outcome::Declined(_)))),
    ));
    // The declines again, together, because they are what a run that looked
    // fine and answered nobody consists of. Printed as measurements: the
    // verdict stays green and a person reads the turns.
    let hallucinated: Vec<usize> = console
        .turns
        .iter()
        .enumerate()
        .filter(|(_, t)| t.hallucinated())
        .map(|(at, _)| at)
        .collect();
    if !hallucinated.is_empty() {
        report.note(format!(
            "{} transcript(s) with words were declined as likely hallucination — no reply \
             followed those wakes",
            hallucinated.len()
        ));
        for at in hallucinated {
            report.note(format!("  {}", turn_line(console, clips, at)));
        }
    }
    if console.stray_barge_ins > 0 {
        report.note(format!(
            "{} barge-in(s) cut no reply this console dispatched",
            console.stray_barge_ins
        ));
    }
}

/// How many of the run's turns are audio somebody can listen to, and where.
///
/// Measured and never a finding: a fetch that brought no audio home, or a store
/// that would not open, is not a claim about the run that failed to hold. The
/// one reason is named here only when every turn that has no clip has the same
/// one — which is the shape a site recording nothing makes — and otherwise each
/// turn's own line carries its own.
fn clips_written(console: &Console, clips: &Clips, report: &mut Report) {
    // A run with no conversation in it has nothing to say here: a fact about
    // zero things is not a fact this report states.
    if console.turns.is_empty() {
        return;
    }
    let written = clips.by_turn.values().filter(|clip| clip.written).count();
    let unwritten = console.turns.len() - written;
    let given: Vec<&str> = clips
        .by_turn
        .values()
        .filter_map(|clip| clip.unwritten.as_deref())
        .collect();
    let distinct: BTreeSet<&str> = given.iter().copied().collect();
    // Every turn without a clip accounted for, and all of them by the same
    // sentence: a turn whose own line stated no span gave no reason at all, so
    // a run holding one of those has no single reason to name.
    let shared = match distinct.iter().next() {
        Some(reason) if given.len() == unwritten && distinct.len() == 1 && unwritten > 0 => {
            format!(" — {reason}")
        }
        _ => String::new(),
    };
    report.note(if written == 0 {
        format!("turn clips: none written{shared}")
    } else {
        format!(
            "turn clips: {written} of {} written to {}{shared}",
            console.turns.len(),
            clips.into.display()
        )
    });
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
fn analyze(console: &Console, clips: &Clips, records: &Records) -> Report {
    let mut report = Report::default();
    let session = Session::of(console, records);
    came_up(console, &mut report);
    criticals(console, &mut report);
    bridged(console, &mut report);
    ended(console, &mut report);
    the_motion_path(console, &session, records, &mut report);
    the_head_moved(records, &session, &mut report);
    alerts_travelled(console, &mut report);
    turns(console, clips, &mut report);
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
    let console = Console::read(at);
    let clips = Clips::write(
        &console,
        &sibling(at, AUDIO_SUFFIX),
        &sibling(at, TURNS_SUFFIX),
    );
    let report = analyze(&console, &clips, &Records::of(at));
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
        ARRIVAL_OFFSET_M, ARRIVAL_TURN_RAD, AUDIO_SUFFIX, Clips, Console, ESTIMATE_CHANNEL,
        HOST_LOG, Line, POD_LOG, PROVENANCE, PoseEstimateWire, REPORT_CHANNEL, Records, ReportKind,
        ReportKindWire, SCRIPT_CHANNEL, ScriptWire, SessionPhaseWire, TURNS_SUFFIX,
        TimelineEntryWire, TimelineWire, analyze, classify, console_dir, neutral_targets,
        refusal_kinds, sibling,
    };

    /// The whole reading of one fetch, as a case has just written it.
    fn judge(at: &Path) -> Report {
        let console = Console::read(at);
        let clips = Clips::write(
            &console,
            &sibling(at, AUDIO_SUFFIX),
            &sibling(at, TURNS_SUFFIX),
        );
        analyze(&console, &clips, &Records::of(at))
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
    fn records(name: &str, lines: &[impl AsRef<str>]) -> (Scratch, PathBuf) {
        let dir = scratch_dir(name);
        let records = dir.join("speech-log-20260831T000000Z");
        let console = dir.join("speech-log-20260831T000000Z.console");
        std::fs::create_dir_all(&console).expect("a console directory");
        let text: String = lines
            .iter()
            .flat_map(|line| [line.as_ref(), "\n"])
            .collect();
        std::fs::write(console.join(HOST_LOG), text).expect("the host's console");
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
        let (_dir, at) = records("speech-report-empty", &[] as &[&str]);
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
        let report = analyze(&console, &Clips::default(), &Records::of(&at));
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
        let (_dir, at) = records("speech-report-refusals", &lines);
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
        let (_dir, at) = records("speech-report-noise", &lines);
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
            classified.lines,
            vec![Line::Pipeline {
                event: String::new(),
                keys: "a,b".to_owned(),
            }]
        );
    }

    /// The one event a raw line carried, for a case about a line that carries
    /// exactly one — which is nearly every line a console holds.
    fn only(raw: &str) -> Line {
        let mut lines = classify(raw).lines;
        assert_eq!(lines.len(), 1, "one event on the line: {raw:?}");
        lines.remove(0)
    }

    /// Two writers interleaving cleanly put two finished events in one
    /// descriptor with no newline between them. Both are real, and a reader
    /// that keeps only the first drops whatever the second said — a
    /// `brain_brenn` among them is a run that exempts itself from the bridge
    /// check.
    #[test]
    fn two_whole_events_glued_together_are_both_read() {
        let torn = concat!(
            r#"{"ts_ms":1788261689389,"event":"listening","addr":"127.0.0.1:9"}"#,
            r#"{"ts_ms":1788261689390,"event":"brain_brenn","publish_channel":"c"}"#,
        );
        let classified = classify(torn);
        assert!(classified.ahead.is_none(), "neither is console text");
        assert!(classified.behind.is_none(), "and nothing is left over");
        assert_eq!(
            classified.lines,
            vec![
                Line::Pipeline {
                    event: "listening".to_owned(),
                    keys: "addr,event,ts_ms".to_owned(),
                },
                Line::Bridge {
                    keys: "event,publish_channel,ts_ms".to_owned(),
                    event: "brain_brenn".to_owned(),
                    unexpected: false,
                    loss: String::new(),
                    granted: None,
                },
            ]
        );
    }

    /// A JSON scalar is not an event: the stream is objects, and anything else
    /// on the console is somebody's print.
    #[test]
    fn a_json_scalar_is_noise() {
        assert!(classify("42").lines.is_empty());
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
            classified.lines,
            vec![Line::Bridge {
                keys: "event,outcome,ts_ms,unexpected".to_owned(),
                event: "brenn_bridge_exit".to_owned(),
                unexpected: true,
                loss: "no wire version in common".to_owned(),
                granted: None,
            }]
        );
    }

    /// The bridge's own variant is the three events and nothing else: an event
    /// this tool has no schema for is the key summary it always was.
    #[test]
    fn an_event_that_is_not_the_bridge_s_is_a_summary() {
        let line = only(r#"{"ts_ms":1,"event":"listening","addr":"127.0.0.1:9"}"#);
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
        assert!(classify(raw).lines.is_empty());
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
            classified.lines,
            vec![Line::Bridge {
                keys: "event,publish_channel,ts_ms".to_owned(),
                event: "brain_brenn".to_owned(),
                unexpected: false,
                loss: String::new(),
                granted: None,
            }]
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
        let line = only(&authored("wake"));
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

    /// A wake-detection event at the default end sample.
    fn wake(score: f64) -> String {
        woke(score, 34880)
    }

    /// A wake-detection event at a stated end sample.
    fn woke(score: f64, wake_end: i64) -> String {
        format!(
            r#"{{"ts_ms":1,"event":"wake_detected","pod":"reachy00","epoch":1,"score":{score},"wake_end_sample":{wake_end}}}"#
        )
    }

    /// An utterance line with a transcript and its three confidence readings.
    fn said(id: u64, text: &str, no_speech: f64) -> String {
        carved(id, 16128, 59968, Some(1), no_speech, text)
    }

    /// The same utterance, at a stated span and segment list. `None` is the
    /// first utterance of a run, whose carve no closed segment covers yet.
    fn carved(
        id: u64,
        start: i64,
        end: i64,
        segment: Option<u32>,
        no_speech: f64,
        text: &str,
    ) -> String {
        let segments = match segment {
            Some(segment) => format!(r#"[{{"log":"a.framelog","segment_id":{segment},"part":0}}]"#),
            None => "[]".to_owned(),
        };
        format!(
            r#"{{"ts_ms":2,"event":"utterance","id":{id},"pod":"reachy00","room":"r-office","endpoint_cause":"soft_endpoint","audio_ref":{{"log":"a.framelog","start_sample":{start},"end_sample":{end},"segments":{segments}}},"transcript":{{"text":"{text}","confidence":{{"avg_logprob":-0.38,"no_speech_prob":{no_speech},"compression_ratio":0.53,"segments":1}}}}}}"#
        )
    }

    /// The same utterance, carrying the two clip boundaries: where the wake
    /// word ends and where the recogniser was actually started.
    fn carved_with_boundary(id: u64, trim: i64, sent_from: i64, text: &str) -> String {
        let line = carved(id, 16128, 59968, Some(1), 0.04, text);
        format!(
            r#"{},"stt_trim_samples":{trim},"stt_sent_from_sample":{sent_from}}}"#,
            line.strip_suffix('}').expect("an object")
        )
    }

    /// The wake being kept back for the command that follows it.
    fn held_line(start: i64, end: i64, deadline: i64) -> String {
        format!(
            r#"{{"ts_ms":2,"event":"wake_held","pod":"reachy00","epoch":1,"start_sample":{start},"end_sample":{end},"wake_end_sample":34880,"deadline_sample":{deadline}}}"#
        )
    }

    /// The gate declining one utterance as a likely hallucination.
    fn declined(id: u64, no_speech: f64) -> String {
        format!(
            r#"{{"ts_ms":3,"event":"wake_command_absent","utterance":{id},"pod":"reachy00","reason":"low_confidence","no_speech":{no_speech},"logprob":-1.05,"score":0.76,"wake_end_sample":34880,"log":"a.framelog","start_sample":16128,"end_sample":59968,"segments":[{{"log":"a.framelog","segment_id":1,"part":0}}]}}"#
        )
    }

    /// The brain taking one utterance.
    fn dispatched(id: u64) -> String {
        format!(r#"{{"ts_ms":4,"event":"brain_dispatched","pod":"reachy00","utterance":{id}}}"#)
    }

    /// Both sinks of the printer over one report, and its verdict.
    fn printed(report: &Report) -> (bool, String, String) {
        let mut out: Vec<u8> = Vec::new();
        let mut err: Vec<u8> = Vec::new();
        let held = run_report::write_verdict(
            &mut out,
            &mut err,
            "speech_run_report",
            "somewhere",
            report,
            "it drained whole",
        );
        (
            held,
            String::from_utf8(out).expect("text"),
            String::from_utf8(err).expect("text"),
        )
    }

    /// The report the incident was invisible in: one turn answered, one
    /// declined with words in it. Both are measurements and the run is green —
    /// a wake with nothing said after it hallucinates and is rightly declined —
    /// but neither is silent.
    #[test]
    fn a_dispatched_turn_and_a_declined_one_are_both_printed_and_the_run_is_green() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "This is a test, one, two, three.", 0.075),
            dispatched(1),
            wake(0.76),
            said(2, "Test two.", 0.30),
            declined(2, 0.30),
        ];
        let (_dir, at) = records("speech-report-turns-two", &lines);
        let report = judge(&at);
        let (held, out, err) = printed(&report);
        assert!(held, "{:?}", report.findings);
        assert_eq!(err, "", "a decline is not a finding");
        for expected in [
            "turn #1 — wake 0.99 → \"This is a test, one, two, three.\" no_speech=0.075 \
             logprob=-0.38 compress=0.53 → dispatched",
            // The utterance's own reading, not the gate's copy of it: the two
            // are the same figures, and the transcript's line is where they
            // are written to the precision the recogniser produced.
            "turn #2 — wake 0.76 → \"Test two.\" no_speech=0.30 logprob=-0.38 compress=0.53 → \
             declined (low_confidence)",
            "2 wake(s): 1 dispatched, 1 declined by the confidence gate, 0 with no transcript, \
             0 STT failure(s), 0 barge-in(s)",
            "no_speech: dispatched 0.075; declined 0.30",
            "1 transcript(s) with words were declined as likely hallucination",
            "; clip not written (no recorded audio beside this fetch) [16128–59968); auto beam ?",
        ] {
            assert!(out.contains(expected), "{expected}\nnot in\n{out}");
        }
    }

    /// The run that closed as "came up, composed, drained whole": five wakes,
    /// five declines, nobody answered. Still green, and the declines are visible.
    #[test]
    fn five_declined_wakes_are_each_printed_and_counted() {
        let mut lines = vec![STARTED.to_owned(), COMPOSED.to_owned()];
        for id in 1..=5u64 {
            lines.push(wake(0.8));
            lines.push(said(
                id,
                "Thank you for watching.",
                0.42 + f64::from(id as u32) * 0.02,
            ));
            lines.push(declined(id, 0.42 + f64::from(id as u32) * 0.02));
        }
        let (_dir, at) = records("speech-report-turns-five", &lines);
        let report = judge(&at);
        let (held, out, _err) = printed(&report);
        assert!(held, "{:?}", report.findings);
        assert!(
            out.contains("5 transcript(s) with words were declined as likely hallucination"),
            "{out}"
        );
        assert!(
            out.contains(
                "5 wake(s): 0 dispatched, 5 declined by the confidence gate, 0 with no \
                 transcript, 0 STT failure(s), 0 barge-in(s)"
            ),
            "{out}"
        );
        assert!(
            out.contains("no_speech: dispatched none; declined 0.44–0.52"),
            "{out}"
        );
        // Once per turn in the turn section, and once more in the block that
        // reads the declines back.
        assert_eq!(
            out.matches("→ declined (low_confidence)").count(),
            10,
            "{out}"
        );
    }

    /// A wake the pipeline minted no utterance for is a turn the run had and
    /// could not read — printed, and not counted as a decline.
    #[test]
    fn a_wake_with_no_utterance_is_a_turn_with_no_transcript() {
        let lines = [STARTED.to_owned(), COMPOSED.to_owned(), wake(0.81)];
        let (_dir, at) = records("speech-report-turn-silent", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "turn #? (wake 1) — wake 0.81 → no transcript"),
            "{:?}",
            report.measured
        );
        assert!(
            measured(
                &report,
                "1 wake(s): 0 dispatched, 0 declined by the confidence gate, 1 with no transcript"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            !measured(&report, "declined as likely hallucination"),
            "{:?}",
            report.measured
        );
    }

    /// Speech resuming re-transcribes the same utterance, and the console
    /// carries two lines for it. One turn, from the final one, with the
    /// supersession counted.
    #[test]
    fn a_superseded_utterance_is_printed_once_from_its_final_line() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test", 0.30),
            r#"{"ts_ms":5,"event":"utterance_superseded","pod":"reachy00","utterance_id":{"epoch":1,"pod":"reachy00","seq":1}}"#.to_owned(),
            said(1, "Test one, two.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-turn-superseded", &lines);
        let report = judge(&at);
        let turns: Vec<&String> = report
            .measured
            .iter()
            .filter(|line| line.starts_with("turn #"))
            .collect();
        assert_eq!(turns.len(), 1, "{turns:?}");
        assert!(
            turns[0].contains("\"Test one, two.\" no_speech=0.04")
                && turns[0].contains("superseded ×1"),
            "{turns:?}"
        );
    }

    /// A transcript with no confidence block at all: the figures print as
    /// missing, and the range line counts the turn in neither mode.
    #[test]
    fn an_utterance_with_no_confidence_reads_as_missing_and_joins_no_range() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            r#"{"ts_ms":2,"event":"utterance","id":1,"pod":"reachy00","transcript":{"text":"Test."}}"#.to_owned(),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-turn-unsure", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "\"Test.\" no_speech=? logprob=? → dispatched"),
            "{:?}",
            report.measured
        );
        assert!(
            measured(&report, "no_speech: dispatched none; declined none"),
            "{:?}",
            report.measured
        );
    }

    /// The bearings a turn is judged by are stamped in the segment's own index
    /// space and the turn's span is pod-absolute. The window converts between
    /// them; a fixture whose segment begins far from zero is one an unconverted
    /// window would select nothing from.
    #[test]
    fn the_auto_beam_figure_covers_the_offsets_the_converted_window_selects() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":149760,"base_sample":170000}"#;
        // Offsets 0 and 1000 are ahead of the wake, 12000 is past the span's
        // end; the two between are the whole of the figure, at 2.0 and 2.4.
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[0,[0.0,0.0,0.0,9.0]],[1000,[0.0,0.0,0.0,9.0]],[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]],[12000,[0.0,0.0,0.0,9.0]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            // The wake ends at 174880 and the span at 179968, which inside the
            // segment is [4880, 9968).
            woke(0.99, 174880),
            carved(1, 173696, 179968, Some(1), 0.04, "Test."),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-beam", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// Without the segment's base sample there is no conversion between the two
    /// index spaces, and a figure computed anyway would be over the wrong
    /// offsets. It prints as missing.
    #[test]
    fn a_segment_that_does_not_say_where_it_begins_leaves_the_beam_unread() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":149760}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,2.0]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-no-base", &lines);
        let report = judge(&at);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// And a turn whose segment never tracked at all reads the same way.
    #[test]
    fn a_turn_with_no_tracking_line_leaves_the_beam_unread() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-turn-untracked", &lines);
        let report = judge(&at);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// The first utterance of a run is carved before any segment has closed, so
    /// its span names no segment at all. The closed segment whose sample range
    /// holds the carve is the one its bearings come out of — a run's first turn
    /// is the clean one, and it is the reading the degradation is measured
    /// against.
    #[test]
    fn a_turn_naming_no_segment_reads_the_closed_segment_that_holds_it() {
        // [170000, 319760) holds the carve at 173696.
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":149760,"base_sample":170000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[1000,[0.0,0.0,0.0,9.0]],[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]],[12000,[0.0,0.0,0.0,9.0]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 174880),
            carved(1, 173696, 179968, None, 0.04, "Test."),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-first-beam", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// Containment is the whole of the rule: a carve no closed segment's range
    /// holds is attributed to none of them, rather than to whichever segment
    /// tracked last.
    ///
    /// The carve sits in the gap after the segment, and its window converts to
    /// offsets this segment did track — so the containment test is the only
    /// thing between this turn and a bearing read out of audio it was not
    /// carved from, and dropping that test flips the reading to a figure.
    #[test]
    fn a_turn_no_closed_range_holds_leaves_the_beam_unread() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":10000,"base_sample":170000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[30500,[0.0,0.0,0.0,2.0]],[31000,[0.0,0.0,0.0,2.4]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            // [170000, 180000) ends well before this carve, whose window
            // converts to [30500, 35000) — offsets the segment tracked.
            woke(0.99, 200500),
            carved(1, 200000, 205000, None, 0.04, "Test."),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-uncontained", &lines);
        let report = judge(&at);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// A carve that straddles a close has its wake in the segment after the one
    /// holding it. The window then opens at the sample that attributed the
    /// turn rather than outside the segment, so a turn a closed range does hold
    /// always reads a figure.
    #[test]
    fn a_carve_whose_wake_ended_past_the_segment_still_reads_its_bearings() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":10000,"base_sample":170000}"#;
        // Offset 1000 is ahead of the carve and 25000 past the span's end; the
        // two between are the whole of the figure.
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[1000,[0.0,0.0,0.0,9.0]],[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]],[25000,[0.0,0.0,0.0,9.0]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            // The carve starts at 175000, inside [170000, 180000); the wake
            // ended at 181000, which is past it.
            woke(0.99, 181000),
            carved(1, 175000, 190000, None, 0.04, "Test."),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-straddled", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// A console that says where a segment begins and not how long it ran
    /// leaves it a point: there is no range to hold a carve, so a turn naming
    /// no segment has nothing to fall back on and prints as missing rather
    /// than borrowing the bearings of whatever segment closed.
    #[test]
    fn a_segment_that_does_not_say_how_long_it_ran_holds_no_carve() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"base_sample":170000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 174880),
            carved(1, 173696, 179968, None, 0.04, "Test."),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-pointlike", &lines);
        let report = judge(&at);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// With two segments closed, the carve picks the one whose range holds it.
    /// The other's bearings would render a figure of their own over the same
    /// window, which is what makes the reading checkable.
    #[test]
    fn a_turn_naming_no_segment_picks_the_range_it_falls_in() {
        // [0, 100000) does not hold the carve at 173696; [170000, 319760) does.
        const FIRST: &str = r#"{"ts_ms":5,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":100000,"base_sample":0}"#;
        const FIRST_TRACKED: &str = r#"{"ts_ms":6,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[175000,[0.0,0.0,0.0,1.0]]]}"#;
        const SECOND: &str = r#"{"ts_ms":7,"event":"segment_closed","pod":"reachy00","segment_id":2,"samples":149760,"base_sample":170000}"#;
        const SECOND_TRACKED: &str = r#"{"ts_ms":8,"event":"tracking","pod":"reachy00","segment_id":2,"doa":[[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 174880),
            carved(1, 173696, 179968, None, 0.04, "Test."),
            dispatched(1),
            FIRST.to_owned(),
            FIRST_TRACKED.to_owned(),
            SECOND.to_owned(),
            SECOND_TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-second-range", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// The range is closed at its first sample. Real segments are contiguous,
    /// so the sample a segment begins at is the one its predecessor ended
    /// before, and a carve landing exactly there is the ordinary case at a
    /// close.
    #[test]
    fn a_carve_at_the_first_sample_of_a_segment_is_held_by_it() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":10000,"base_sample":170000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[1000,[0.0,0.0,0.0,2.0]],[3000,[0.0,0.0,0.0,2.4]],[8000,[0.0,0.0,0.0,9.0]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 170000),
            carved(1, 170000, 175000, None, 0.04, "Test."),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-first-sample", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// And open at the sample after its last: the segment holds that many
    /// samples counting from its base, and the carve beginning there belongs to
    /// whatever closed next. A range that held both ends would have this turn
    /// take the earlier segment's bearings, which is a wrong figure and not a
    /// missing one.
    #[test]
    fn a_carve_at_the_sample_after_a_segment_is_held_by_nothing() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":10000,"base_sample":170000}"#;
        // Offsets the window would select, so an inclusive upper bound reads a
        // figure rather than nothing.
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[10000,[0.0,0.0,0.0,2.0]],[12000,[0.0,0.0,0.0,2.4]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 180000),
            carved(1, 180000, 185000, None, 0.04, "Test."),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-past-the-end", &lines);
        let report = judge(&at);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// A carve can begin before the segment it was taken from: the pod's
    /// pre-roll is whatever history its capture ring held, so the first segment
    /// of a connection can open after the host's own pad reaches back. The wake
    /// that opened the segment is inside it by construction, so it is the
    /// anchor that answers when the carve's own start is held by nothing.
    #[test]
    fn a_carve_beginning_before_its_segment_is_attributed_by_the_wake() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":10000,"base_sample":170000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[2000,[0.0,0.0,0.0,2.0]],[4000,[0.0,0.0,0.0,2.4]],[9000,[0.0,0.0,0.0,9.0]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            // The carve starts 1000 samples ahead of the segment's base; the
            // wake ended 1000 samples inside it.
            woke(0.99, 171000),
            carved(1, 169000, 178000, None, 0.04, "Test."),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-early-carve", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// A wake with no utterance behind it names no span at all, so nothing on
    /// the console says where the turn stopped. The segment's own end is not a
    /// bound the turn has any claim to — it is whatever the gate did
    /// afterwards, a later turn's speech included — so the figure is missing
    /// rather than a mean over somebody else.
    #[test]
    fn a_turn_with_no_carve_end_keeps_no_beam_figure() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":10000,"base_sample":170000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[1000,[0.0,0.0,0.0,9.0]],[6000,[0.0,0.0,0.0,2.0]],[8000,[0.0,0.0,0.0,2.4]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 175000),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-wake-only-beam", &lines);
        let report = judge(&at);
        assert!(measured(&report, "no transcript"), "{:?}", report.measured);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// A segment closed at [170000, 180000), tracked at four offsets: two
    /// inside the window an expiry at 178000 bounds, one ahead of the wake and
    /// one past the expiry.
    const EXPIRY_CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":10000,"base_sample":170000}"#;
    const EXPIRY_TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[1000,[0.0,0.0,0.0,9.0]],[3000,[0.0,0.0,0.0,2.0]],[5000,[0.0,0.0,0.0,2.4]],[9000,[0.0,0.0,0.0,9.0]]]}"#;
    /// An arm that expired with nobody answering: no transcript, and a span all
    /// the same, whose end is the expiry point.
    const EXPIRED: &str = r#"{"ts_ms":8,"event":"wake_command_absent","pod":"reachy00","reason":"arm_expired","score":0.99,"log":"a.framelog","start_sample":171000,"end_sample":178000,"segments":[]}"#;

    /// A wake nobody answered still says where it stopped: the arm's expiry is
    /// on its decline line as an ordinary span. The figure is the bearing after
    /// the wake, bounded at that expiry rather than running to the close of
    /// whatever segment held it.
    ///
    /// In the order the console produces: the segment closes at the VAD
    /// release, ahead of the line that states the span.
    #[test]
    fn an_expired_arm_reads_its_bearings_up_to_the_expiry() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 172000),
            EXPIRY_CLOSED.to_owned(),
            EXPIRY_TRACKED.to_owned(),
            EXPIRED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-expiry", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// The same lines with the span ahead of its segment's, which a fresh wake
    /// superseding an armed one also produces. The figure is a function of the
    /// whole console and not of the order its lines arrived in.
    #[test]
    fn an_expired_arm_reads_the_same_figure_with_its_span_first() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 172000),
            EXPIRED.to_owned(),
            EXPIRY_CLOSED.to_owned(),
            EXPIRY_TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-expiry-first", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// The shape a degraded turn has: the segment closes at the VAD release and
    /// the transcript's line waits for the recogniser, so the utterance trails
    /// its own segment's lines by the whole of an STT call. A figure computed
    /// as the tracking line was read would be missing on exactly these turns.
    #[test]
    fn an_utterance_that_trails_its_segment_still_reads_its_bearings() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 172000),
            EXPIRY_CLOSED.to_owned(),
            EXPIRY_TRACKED.to_owned(),
            carved(1, 171000, 178000, None, 0.04, "Test."),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-turn-trailing", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// A pod reconnecting counts its samples from zero again, so a segment id
    /// means nothing across the hello either: the segment 1 that closes after
    /// it is not the segment 1 an earlier turn's span named.
    #[test]
    fn a_named_segment_from_before_a_reconnect_is_a_different_segment() {
        const AGAIN: &str = r#"{"ts_ms":5,"event":"conn_hello","conn_seq":2,"pod_id":"reachy00","room":"r-office","unmapped":false}"#;
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":149760,"base_sample":170000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 174880),
            carved(1, 173696, 179968, Some(1), 0.04, "Test."),
            dispatched(1),
            AGAIN.to_owned(),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-named-across-hello", &lines);
        let report = judge(&at);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// And the same segment answers for a turn of its own connection, so the
    /// generation is a fence and not a stop: the reading survives a reconnect
    /// that happened before the turn.
    #[test]
    fn a_named_segment_of_the_live_connection_still_reads_its_bearings() {
        const AGAIN: &str = r#"{"ts_ms":1,"event":"conn_hello","conn_seq":2,"pod_id":"reachy00","room":"r-office","unmapped":false}"#;
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":149760,"base_sample":170000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            AGAIN.to_owned(),
            woke(0.99, 174880),
            carved(1, 173696, 179968, Some(1), 0.04, "Test."),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-named-after-hello", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// Two ranges holding one carve attribute neither whichever order the two
    /// closes and their tracking lines arrive in. The judgement is made once
    /// against the finished map, so there is no first answer for a second
    /// holder to withdraw.
    #[test]
    fn two_closed_ranges_holding_one_carve_attribute_neither_in_either_order() {
        const FIRST: &str = r#"{"ts_ms":5,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":149760,"base_sample":170000}"#;
        const FIRST_TRACKED: &str = r#"{"ts_ms":6,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        const SECOND: &str = r#"{"ts_ms":7,"event":"segment_closed","pod":"reachy00","segment_id":2,"samples":100000,"base_sample":170000}"#;
        const SECOND_TRACKED: &str = r#"{"ts_ms":8,"event":"tracking","pod":"reachy00","segment_id":2,"doa":[[5000,[0.0,0.0,0.0,1.0]],[7000,[0.0,0.0,0.0,1.0]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 174880),
            carved(1, 173696, 179968, None, 0.04, "Test."),
            dispatched(1),
            SECOND.to_owned(),
            SECOND_TRACKED.to_owned(),
            FIRST.to_owned(),
            FIRST_TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-two-holders-reversed", &lines);
        let report = judge(&at);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// One segment tracked twice in disagreement converts nothing: nothing on
    /// the console says which of the two readings the turn's samples were taken
    /// under, and a figure over the wrong one reads exactly like a figure over
    /// the right one. Two lines that agree are one reading and are kept — the
    /// rule is disagreement, not repetition.
    #[test]
    fn a_segment_tracked_twice_in_disagreement_converts_nothing() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":149760,"base_sample":170000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        const ELSEWHERE: &str = r#"{"ts_ms":8,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,1.0]],[7000,[0.0,0.0,0.0,1.0]]]}"#;
        const AGAIN: &str = r#"{"ts_ms":8,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        for (name, second, says) in [
            ("speech-report-turn-tracked-apart", ELSEWHERE, "auto beam ?"),
            (
                "speech-report-turn-tracked-alike",
                AGAIN,
                "auto beam 2.20±0.20 rad",
            ),
        ] {
            let lines = [
                STARTED.to_owned(),
                COMPOSED.to_owned(),
                woke(0.99, 174880),
                carved(1, 173696, 179968, Some(1), 0.04, "Test."),
                dispatched(1),
                CLOSED.to_owned(),
                TRACKED.to_owned(),
                second.to_owned(),
            ];
            let (_dir, at) = records(name, &lines);
            let report = judge(&at);
            assert!(measured(&report, says), "{name}: {:?}", report.measured);
        }
    }

    /// The window opens at the wake's end and not at the carve's start, so the
    /// look direction the wake word was spoken from is not averaged into the
    /// utterance's. The carve reaches back behind the wake by the host's
    /// pre-roll pad, which is where the two samples differ.
    #[test]
    fn the_window_opens_at_the_wake_and_not_at_the_carve() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":149760,"base_sample":170000}"#;
        // Offset 4000 is inside the carve and ahead of the wake: a window
        // opening at the carve would average it in.
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[4000,[0.0,0.0,0.0,9.0]],[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            // The carve starts at 173696, the wake ended at 174880: inside the
            // segment those are offsets 3696 and 4880.
            woke(0.99, 174880),
            carved(1, 173696, 179968, Some(1), 0.04, "Test."),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-window-origin", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// With no wake on the console the window opens at the carve's own start
    /// instead. The two precedences are opposite and only a turn missing one of
    /// the two samples tells them apart.
    #[test]
    fn a_carve_with_no_wake_opens_its_window_at_the_carve() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":149760,"base_sample":170000}"#;
        // Offset 1000 is inside the segment and ahead of the carve: a window
        // opening at the segment's base rather than at the carve would take it
        // and print a figure of its own.
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[1000,[0.0,0.0,0.0,9.0]],[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            carved(1, 173696, 179968, None, 0.04, "Test."),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-wakeless-window", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// A turn whose span names a segment is judged by that segment and no
    /// other. Containment is the fallback for a carve nothing named, never a
    /// second chance for one that was named and did not track.
    #[test]
    fn a_named_segment_is_not_overridden_by_the_range_holding_the_carve() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":2,"samples":149760,"base_sample":170000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":2,"doa":[[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 174880),
            // Segment 1 is what the span names; segment 2 is what holds the
            // carve and tracked.
            carved(1, 173696, 179968, Some(1), 0.04, "Test."),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-named-elsewhere", &lines);
        let report = judge(&at);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// Two closed ranges holding one carve attribute neither. A pod that
    /// restarted its index space mid-run has two unrelated segments answering
    /// to one sample, and a bearing read out of the wrong one is a confident
    /// figure over audio nobody spoke into.
    #[test]
    fn two_closed_ranges_holding_one_carve_attribute_neither() {
        const FIRST: &str = r#"{"ts_ms":5,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":149760,"base_sample":170000}"#;
        const FIRST_TRACKED: &str = r#"{"ts_ms":6,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        const SECOND: &str = r#"{"ts_ms":7,"event":"segment_closed","pod":"reachy00","segment_id":2,"samples":100000,"base_sample":170000}"#;
        const SECOND_TRACKED: &str = r#"{"ts_ms":8,"event":"tracking","pod":"reachy00","segment_id":2,"doa":[[5000,[0.0,0.0,0.0,1.0]],[7000,[0.0,0.0,0.0,1.0]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 174880),
            carved(1, 173696, 179968, None, 0.04, "Test."),
            dispatched(1),
            FIRST.to_owned(),
            FIRST_TRACKED.to_owned(),
            SECOND.to_owned(),
            SECOND_TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-two-holders", &lines);
        let report = judge(&at);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// One segment id closes more than once: the host's length cap finalizes a
    /// part and opens a contiguous successor under the same id. Each part
    /// converts by its own base, so the part is half of what names a segment.
    #[test]
    fn each_part_of_one_segment_converts_by_its_own_base() {
        const PART_ZERO: &str = r#"{"ts_ms":5,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":10000,"base_sample":170000,"audio_ref":{"log":"a.framelog","segment_id":1,"part":0}}"#;
        const PART_ONE: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":10000,"base_sample":180000,"audio_ref":{"log":"a.framelog","segment_id":1,"part":1}}"#;
        // Part zero's tracking line arrives behind both closes, which is the
        // ordering a pipeline lagging the connection by a part produces.
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[6000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]],"audio_ref":{"log":"a.framelog","segment_id":1,"part":0}}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            // The carve sits in part zero, at [175000, 178000).
            woke(0.99, 175000),
            carved(1, 175000, 178000, None, 0.04, "Test."),
            dispatched(1),
            PART_ZERO.to_owned(),
            PART_ONE.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-parts", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "auto beam 2.20±0.20 rad"),
            "{:?}",
            report.measured
        );
    }

    /// A segment id and part that close twice over — a truncated segment that
    /// resumed — convert nothing: the console does not say which of the two
    /// bases the tracking line that follows belongs to, and a figure converted
    /// by the wrong base is over the wrong audio.
    #[test]
    fn a_segment_that_closed_twice_under_one_name_converts_nothing() {
        const FIRST: &str = r#"{"ts_ms":5,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":10000,"base_sample":170000}"#;
        const AGAIN: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":10000,"base_sample":300000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            // Inside the second close's range, so the last base written would
            // have read a figure.
            woke(0.99, 305000),
            carved(1, 305000, 308000, None, 0.04, "Test."),
            dispatched(1),
            FIRST.to_owned(),
            AGAIN.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-reclosed", &lines);
        let report = judge(&at);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// A reconnect fences the index space its samples were counted in: the pod
    /// counts from zero again, so a segment closed afterwards can hold a stale
    /// sample of a turn from the connection before it. The carve here names no
    /// segment, so containment is the path the generation has to guard.
    #[test]
    fn a_reconnect_fences_the_space_an_earlier_turn_was_carved_in() {
        const AGAIN: &str = r#"{"ts_ms":5,"event":"conn_hello","conn_seq":2,"pod_id":"reachy00","room":"r-office","unmapped":false}"#;
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":149760,"base_sample":170000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 174880),
            carved(1, 173696, 179968, None, 0.04, "Test."),
            dispatched(1),
            AGAIN.to_owned(),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-reconnected", &lines);
        let report = judge(&at);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// Every sample index is a number off the console, and a console tears. A
    /// segment whose stated length runs off the end of the index space holds
    /// nothing, rather than killing the report the run is read by.
    #[test]
    fn a_segment_whose_length_overruns_the_index_space_holds_nothing() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":9000,"base_sample":9223372036854775000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,2.0]]]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            woke(0.99, 9223372036854775001),
            carved(
                1,
                9223372036854775001,
                9223372036854775005,
                None,
                0.04,
                "Test.",
            ),
            dispatched(1),
            CLOSED.to_owned(),
            TRACKED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-overrun", &lines);
        let report = judge(&at);
        assert!(measured(&report, "auto beam ?"), "{:?}", report.measured);
    }

    /// Somebody speaking over a reply is counted on the turn whose reply it
    /// cut, and in the summary — the ASR output has no echo suppressor in front
    /// of it, so a reply cutting itself off is the failure this counts.
    #[test]
    fn a_barge_in_is_counted_on_the_turn_whose_reply_it_cut() {
        const BARGED: &str = r#"{"ts_ms":8,"event":"barge_in","pod":"reachy00","epoch":1,"trigger_sample":5000,"host_rx_us":9}"#;
        const FLUSHED: &str = r#"{"ts_ms":9,"event":"playback_flushed","pod":"reachy00","utterance":1,"was_playing":true,"frames_written":10,"heard_ms":400,"total_ms":1600}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
            BARGED.to_owned(),
            FLUSHED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-barge", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "barge-in ×1; reply flushed ×1"),
            "{:?}",
            report.measured
        );
        assert!(measured(&report, "1 barge-in(s)"), "{:?}", report.measured);
    }

    /// A turn a barge-in minted is as real as any other and is not a wake. The
    /// summary is the line two acceptance sessions are compared on, so a count
    /// that mixed the two would make sessions with different amounts of
    /// interruption incomparable.
    #[test]
    fn a_turn_no_wake_opened_is_not_counted_as_a_wake() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
            // The barge-in's own utterance: minted with no wake ahead of it,
            // and carrying an id this console has not seen.
            said(2, "No, stop.", 0.05),
            dispatched(2),
        ];
        let (_dir, at) = records("speech-report-turn-unwoken", &lines);
        let report = judge(&at);
        assert!(
            measured(
                &report,
                "1 wake(s): 2 dispatched, 0 declined by the confidence gate, 0 with no \
                 transcript, 0 STT failure(s), 0 barge-in(s); 1 turn(s) began with no wake \
                 ahead of them"
            ),
            "{:?}",
            report.measured
        );
    }

    /// The recogniser itself failing is the class of run the ASR channel could
    /// newly produce, so the line it renders through and the counter it lands
    /// in are both read here.
    #[test]
    fn a_turn_the_recogniser_failed_on_renders_and_is_counted() {
        const FAILED: &str = r#"{"ts_ms":5,"event":"stt_failed","pod":"reachy00","utterance_seq":1,"detail":"the model returned no segments"}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            FAILED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-stt-failed", &lines);
        let report = judge(&at);
        assert!(
            measured(
                &report,
                "→ stt failed (the model returned no segments); clip ?"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            measured(
                &report,
                "1 wake(s): 0 dispatched, 0 declined by the confidence gate, 1 with no \
                 transcript, 1 STT failure(s), 0 barge-in(s)"
            ),
            "{:?}",
            report.measured
        );
    }

    /// A turn that reached the brain with nothing in it: its own clause, and
    /// counted as neither dispatched nor declined.
    #[test]
    fn a_turn_the_brain_was_handed_no_words_for_renders_and_is_counted() {
        const EMPTY: &str =
            r#"{"ts_ms":5,"event":"brain_no_transcript","pod":"reachy00","utterance":1}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "", 0.04),
            EMPTY.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-wordless", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "→ the brain was handed no words"),
            "{:?}",
            report.measured
        );
        assert!(
            measured(
                &report,
                "1 wake(s): 0 dispatched, 0 declined by the confidence gate, 0 with no \
                 transcript, 0 STT failure(s), 0 barge-in(s)"
            ),
            "{:?}",
            report.measured
        );
    }

    /// The gate declines a turn somebody barged in with through its own event,
    /// and that decline is the same decline: counted under the gate, and read
    /// back as a hallucination because it had words in it.
    #[test]
    fn a_barge_turn_the_gate_declined_is_counted_with_the_others() {
        const BARGE_DECLINED: &str = r#"{"ts_ms":5,"event":"barge_command_absent","utterance":1,"pod":"reachy00","reason":"low_confidence","no_speech":0.44,"logprob":-1.2}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Thank you for watching.", 0.44),
            BARGE_DECLINED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-barge-declined", &lines);
        let report = judge(&at);
        assert!(
            measured(
                &report,
                "1 wake(s): 0 dispatched, 1 declined by the confidence gate, 0 with no \
                 transcript, 0 STT failure(s), 0 barge-in(s)"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            measured(
                &report,
                "1 transcript(s) with words were declined as likely hallucination — no reply \
                 followed those wakes"
            ),
            "{:?}",
            report.measured
        );
    }

    /// A wake whose arm expired is unanswered without the gate having read
    /// anything, so it is said apart from the confidence declines — the count
    /// two acceptance sessions are compared on is about the audio.
    #[test]
    fn a_decline_the_gate_did_not_make_is_counted_apart_from_it() {
        const EXPIRED: &str = r#"{"ts_ms":5,"event":"wake_command_absent","utterance":1,"pod":"reachy00","reason":"arm_expired"}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            EXPIRED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-arm-expired", &lines);
        let report = judge(&at);
        assert!(
            measured(
                &report,
                "1 wake(s): 0 dispatched, 0 declined by the confidence gate, 1 with no \
                 transcript, 0 STT failure(s), 0 barge-in(s), 1 unanswered for other reasons \
                 (arm_expired ×1)"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            !measured(&report, "declined as likely hallucination"),
            "{:?}",
            report.measured
        );
    }

    /// A reply that played to its end is said in seconds on the turn it
    /// answered; an announcement nobody asked a question for names no turn and
    /// is charged to none, because a turn carrying it reads as a wake that was
    /// answered.
    #[test]
    fn a_reply_that_played_is_said_on_its_own_turn_and_an_announcement_on_none() {
        const PLAYED: &str = r#"{"ts_ms":6,"event":"playback_finished","pod":"reachy00","utterance":1,"nominal_audio_ms":1600}"#;
        const ANNOUNCED: &str = r#"{"ts_ms":7,"event":"playback_finished","pod":"reachy00","utterance":null,"nominal_audio_ms":9000}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
            PLAYED.to_owned(),
            wake(0.76),
            said(2, "Test two.", 0.30),
            declined(2, 0.30),
            ANNOUNCED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-turn-played", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "→ dispatched; reply played 1.6 s"),
            "{:?}",
            report.measured
        );
        assert_eq!(
            report
                .measured
                .iter()
                .filter(|line| line.contains("reply played"))
                .count(),
            1,
            "the announcement is no turn's reply: {:?}",
            report.measured
        );
    }

    /// A barge-in ahead of anything this console dispatched cut no reply it
    /// knows about. Said on its own line rather than charged to a turn, and
    /// still counted — the robot hearing itself is what that line would be.
    #[test]
    fn a_barge_in_before_any_reply_is_counted_on_its_own_line() {
        const BARGED: &str = r#"{"ts_ms":2,"event":"barge_in","pod":"reachy00","epoch":1,"trigger_sample":5000,"host_rx_us":9}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            BARGED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-turn-stray-barge", &lines);
        let report = judge(&at);
        assert!(
            measured(
                &report,
                "1 barge-in(s) cut no reply this console dispatched"
            ),
            "{:?}",
            report.measured
        );
        assert!(measured(&report, "1 barge-in(s)"), "{:?}", report.measured);
        assert!(
            !measured(&report, "barge-in ×1"),
            "no turn was charged with it: {:?}",
            report.measured
        );
    }

    /// The run an operator ended mid-turn: a transcript with nothing after it.
    /// Its own clause, and counted in neither column of the summary.
    #[test]
    fn a_turn_this_console_never_said_the_end_of_prints_as_open() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
        ];
        let (_dir, at) = records("speech-report-turn-open", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "→ no outcome on this console"),
            "{:?}",
            report.measured
        );
        assert!(
            measured(
                &report,
                "1 wake(s): 0 dispatched, 0 declined by the confidence gate, 0 with no \
                 transcript, 0 STT failure(s), 0 barge-in(s)"
            ),
            "{:?}",
            report.measured
        );
    }

    /// The first utterance of a process carries no segment list and the decline
    /// beside it does. The two halves of a turn's span are taken from whichever
    /// event named them, in either order, and neither overwrites the other.
    #[test]
    fn a_span_named_across_two_events_is_whole_in_either_order() {
        const CLOSED: &str = r#"{"ts_ms":6,"event":"segment_closed","pod":"reachy00","segment_id":1,"samples":149760,"base_sample":170000}"#;
        const TRACKED: &str = r#"{"ts_ms":7,"event":"tracking","pod":"reachy00","segment_id":1,"doa":[[5000,[0.0,0.0,0.0,2.0]],[7000,[0.0,0.0,0.0,2.4]]]}"#;
        // The transcript names the samples and no segment at all.
        const SEGMENTLESS: &str = r#"{"ts_ms":2,"event":"utterance","id":1,"pod":"reachy00","audio_ref":{"log":"a.framelog","start_sample":173696,"end_sample":179968},"transcript":{"text":"Test.","confidence":{"avg_logprob":-0.38,"no_speech_prob":0.44}}}"#;
        // The decline names the segment, and the samples again.
        const NAMED: &str = r#"{"ts_ms":3,"event":"wake_command_absent","utterance":1,"pod":"reachy00","reason":"low_confidence","no_speech":0.44,"logprob":-1.05,"log":"a.framelog","start_sample":173696,"end_sample":179968,"segments":[{"log":"a.framelog","segment_id":1,"part":0}]}"#;
        const WOKE: &str = r#"{"ts_ms":1,"event":"wake_detected","pod":"reachy00","epoch":1,"score":0.99,"wake_end_sample":174880}"#;
        for (name, order) in [
            ("speech-report-span-said-first", [SEGMENTLESS, NAMED]),
            ("speech-report-span-declined-first", [NAMED, SEGMENTLESS]),
        ] {
            let lines = [STARTED, COMPOSED, WOKE, order[0], order[1], CLOSED, TRACKED];
            let (_dir, at) = records(name, &lines);
            let report = judge(&at);
            assert!(
                measured(
                    &report,
                    "clip not written (no recorded audio beside this fetch) [173696–179968)"
                ),
                "{name}: {:?}",
                report.measured
            );
            assert!(
                measured(&report, "auto beam 2.20±0.20 rad"),
                "{name}: {:?}",
                report.measured
            );
        }
    }

    /// A transcript is somebody's words and reaches the terminal through the
    /// same bound every other quoted line does.
    #[test]
    fn a_transcript_is_quoted_like_any_other_text_off_the_console() {
        let long = "ha".repeat(200);
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, &format!("one\\ttwo {long}"), 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-turn-quoted", &lines);
        let report = judge(&at);
        let turn = report
            .measured
            .iter()
            .find(|line| line.starts_with("turn "))
            .unwrap_or_else(|| panic!("{:?}", report.measured));
        assert!(turn.contains("one two ha"), "the tab is a space: {turn}");
        assert!(turn.contains('…'), "the text is bounded: {turn}");
    }

    /// The other direction of the stdout-and-stderr tear: a whole event with a
    /// console sentence begun into the same descriptor behind it, which is how
    /// the transcript's own line actually arrives.
    #[test]
    fn a_console_sentence_glued_behind_an_event_leaves_the_event_readable() {
        let torn = format!(
            "{}22:56:35.293 [r-office/reachy00] utterance #1 — \"Test.\"",
            said(1, "Test.", 0.04)
        );
        let classified = classify(&torn);
        assert!(classified.ahead.is_none(), "the event leads the line");
        assert_eq!(
            classified.behind.as_deref(),
            Some("22:56:35.293 [r-office/reachy00] utterance #1 — \"Test.\""),
            "the sentence behind it is the console's",
        );
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            torn,
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-turn-torn", &lines);
        let report = judge(&at);
        assert!(
            measured(&report, "\"Test.\" no_speech=0.04"),
            "{:?}",
            report.measured
        );
    }
    // -----------------------------------------------------------------------
    // The turn clips
    // -----------------------------------------------------------------------

    /// How many samples one fixture audio frame holds.
    ///
    /// The wire payload caps a frame at 640 samples, so a carve-sized segment
    /// is several frames and the fixture's values restart at 1 in each of them.
    const FRAME: usize = 640;

    /// Where the fixture segment's first sample sits, below every span the
    /// cases carve so a clip is audio and not silence.
    const AUDIO_BASE: u64 = 15_360;

    /// The value the fixture writes at one absolute sample index.
    ///
    /// A sawtooth: non-zero wherever the log has audio, different at every
    /// offset inside a frame, so a slice off by one sample or one frame reads
    /// differently from the one that was asked for.
    fn sawtooth(sample: u64) -> i16 {
        ((sample - AUDIO_BASE) % FRAME as u64) as i16 + 1
    }

    /// A recorded-audio store beside `records`, holding one segment of
    /// `frames` frames in `log`.
    ///
    /// Written through brenn-pod's own frame-log writer, which is the only
    /// thing that writes this format: a fixture built any other way is a second
    /// writer of it to keep in step with the first.
    fn store(records: &Path, log: &str, frames: usize) -> PathBuf {
        let dir = sibling(records, AUDIO_SUFFIX);
        std::fs::create_dir_all(&dir).expect("an audio store");
        let mut written = vec![
            pod_ingest::test_fixtures::hello("reachy00"),
            pod_ingest::test_fixtures::seg_start(1, AUDIO_BASE),
        ];
        for frame in 0..frames {
            written.push(pod_ingest::test_fixtures::audio(
                1,
                AUDIO_BASE + (frame * FRAME) as u64,
                FRAME,
            ));
        }
        written.push(pod_ingest::test_fixtures::seg_end(
            1,
            (frames * FRAME) as u64,
        ));
        pod_ingest::test_fixtures::write_log(&dir.join(log), &written);
        dir
    }

    /// A store covering every span the default fixture lines carve.
    fn whole_store(records: &Path) -> PathBuf {
        store(records, "a.framelog", 72)
    }

    /// The samples one written clip holds, read back out of the file.
    ///
    /// The chunks are walked rather than the header assumed, so the reader is
    /// held to the file being a `.wav` and not to a byte count.
    fn clip_samples(at: &Path) -> Vec<i16> {
        let bytes = std::fs::read(at).unwrap_or_else(|why| panic!("{}: {why}", at.display()));
        assert_eq!(&bytes[..4], b"RIFF", "{}", at.display());
        assert_eq!(&bytes[8..12], b"WAVE", "{}", at.display());
        let mut cursor = 12;
        while cursor + 8 <= bytes.len() {
            let size = u32::from_le_bytes(bytes[cursor + 4..cursor + 8].try_into().unwrap());
            let body = cursor + 8;
            let end = body + size as usize;
            if &bytes[cursor..cursor + 4] == b"data" {
                return bytes[body..end.min(bytes.len())]
                    .chunks_exact(2)
                    .map(|pair| i16::from_le_bytes([pair[0], pair[1]]))
                    .collect();
            }
            cursor = end + (end % 2);
        }
        panic!("no data chunk in {}", at.display());
    }

    /// One clip line of a report, by the file it names.
    fn clip_line(report: &Report, holds: &str) -> String {
        report
            .measured
            .iter()
            .find(|line| line.starts_with("turn #") && line.contains(holds))
            .unwrap_or_else(|| panic!("no turn line holding {holds}: {:?}", report.measured))
            .clone()
    }

    /// The clip is the carve: the samples the listener handed the recogniser,
    /// out of the log the utterance's own line named.
    #[test]
    fn a_turn_clip_holds_exactly_the_carve() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-carve", &lines);
        whole_store(&at);
        let report = judge(&at);
        assert!(
            clip_line(&report, "clip turn-01.wav [16128–59968)").contains("dispatched"),
            "{:?}",
            report.measured
        );
        assert!(
            measured(&report, "turn clips: 1 of 1 written to "),
            "{:?}",
            report.measured
        );
        let held = clip_samples(&sibling(&at, TURNS_SUFFIX).join("turn-01.wav"));
        assert_eq!(held.len(), 59968 - 16128, "the carve's own length");
        for offset in [0usize, 1, 639, 640, 4321, 43839] {
            assert_eq!(
                held[offset],
                sawtooth(16128 + offset as u64),
                "sample {offset} of the carve"
            );
        }
    }

    /// A turn whose line named no segment is cut all the same: the segment list
    /// is empty on exactly the turns whose recogniser ran long, which are the
    /// ones this report exists for.
    #[test]
    fn a_turn_naming_no_segment_is_still_cut() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            carved(1, 16128, 59968, None, 0.04, "Test."),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-segmentless", &lines);
        whole_store(&at);
        let report = judge(&at);
        assert!(
            measured(&report, "clip turn-01.wav [16128–59968)"),
            "{:?}",
            report.measured
        );
        assert_eq!(
            clip_samples(&sibling(&at, TURNS_SUFFIX).join("turn-01.wav")).len(),
            59968 - 16128
        );
    }

    /// The file is named by the token its line prints after `#`, so the clip is
    /// found from the report by reading — including where the utterance ids and
    /// the wake ordinals have diverged.
    #[test]
    fn a_clip_is_named_by_the_token_its_turn_line_prints() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            carved(1, 16128, 20000, None, 0.04, "One."),
            dispatched(1),
            wake(0.80),
            wake(0.99),
            carved(2, 30000, 40000, None, 0.04, "Two."),
            dispatched(2),
        ];
        let (_dir, at) = records("speech-report-clip-names", &lines);
        whole_store(&at);
        let report = judge(&at);
        assert!(
            clip_line(&report, "clip turn-01.wav [16128–20000)").starts_with("turn #1 "),
            "{:?}",
            report.measured
        );
        assert!(
            clip_line(&report, "clip turn-02.wav [30000–40000)").starts_with("turn #2 "),
            "{:?}",
            report.measured
        );
        assert!(
            measured(&report, "turn #? (wake 2)"),
            "{:?}",
            report.measured
        );
        // The wake nobody answered stated no span, so it gave no reason for
        // having no clip and the summary names none for the run.
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.starts_with("turn clips: 2 of 3 written to ")
                    && !line.contains(" — ")),
            "{:?}",
            report.measured
        );
        let turns = sibling(&at, TURNS_SUFFIX);
        assert_eq!(
            clip_samples(&turns.join("turn-02.wav"))[0],
            sawtooth(30000),
            "the file each line names holds that line's span"
        );
        assert_eq!(
            std::fs::read_dir(&turns).expect("a clip directory").count(),
            2,
            "the wake nobody answered carved nothing"
        );
    }

    /// The recogniser heard the carve minus its trim, and the console states
    /// the trim on a decline. The clip is the whole carve either way; the line
    /// says where the recogniser began.
    #[test]
    fn a_declined_turn_says_where_the_recogniser_began() {
        let trimmed = r#"{"ts_ms":3,"event":"wake_command_absent","utterance":1,"pod":"reachy00","reason":"low_confidence","no_speech":0.45,"logprob":-1.05,"stt_trim_samples":13504,"log":"a.framelog","start_sample":16128,"end_sample":59968,"segments":[]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "service.", 0.45),
            trimmed.to_owned(),
        ];
        let (_dir, at) = records("speech-report-clip-trim", &lines);
        whole_store(&at);
        let report = judge(&at);
        assert!(
            clip_line(&report, "clip turn-01.wav").contains("STT boundary +0.84 s"),
            "{:?}",
            report.measured
        );
        let dispatched_lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
        ];
        let (_other, elsewhere) = records("speech-report-clip-untrimmed", &dispatched_lines);
        whole_store(&elsewhere);
        let report = judge(&elsewhere);
        assert!(
            !clip_line(&report, "clip turn-01.wav").contains("STT boundary"),
            "an utterance line with no boundary states none: {:?}",
            report.measured
        );
        assert!(
            !sibling(&elsewhere, TURNS_SUFFIX)
                .join("turn-01.command.wav")
                .exists(),
            "no boundary, no second clip"
        );
    }

    /// A dispatched turn whose utterance line carries the boundary: both clips
    /// are written, and the second is exactly the tail of the first.
    ///
    /// The one-decode invariant -- that both clips come from cutting one decoded
    /// buffer, not two independent resolves -- has no assertion seam here.
    /// Suffix identity is the proxy: two independent decodes would only
    /// coincidentally produce the same samples. A refactor into two `cut` calls
    /// would double a decode that is already quadratic per run
    /// (`TODO(clip-one-pass-per-log)`), and this test would not object.
    #[test]
    fn a_turn_with_a_boundary_writes_the_clip_the_recogniser_heard() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            carved_with_boundary(1, 13_504, 13_504, "Test."),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-command", &lines);
        whole_store(&at);
        let report = judge(&at);
        let line = clip_line(&report, "clip turn-01.wav");
        assert!(
            line.contains("STT boundary +0.84 s, sent from +0.84 s"),
            "{line}"
        );
        let turns = sibling(&at, TURNS_SUFFIX);
        let whole = clip_samples(&turns.join("turn-01.wav"));
        let command = clip_samples(&turns.join("turn-01.command.wav"));
        assert_eq!(whole.len(), 59968 - 16128, "the carve's own length");
        assert_eq!(command.len(), 59968 - 16128 - 13_504, "the carve past it");
        assert_eq!(command, whole[13_504..], "a suffix of the same audio");
    }

    /// A boundary past the end of the decoded audio -- a torn or truncated record
    /// -- states no suffix to write. The whole clip is still on disk and still
    /// named, and the line says why its companion is not: a refusal here would
    /// tell the operator the audio they can open does not exist.
    #[test]
    fn a_boundary_outside_the_clip_is_a_note_and_not_a_refusal() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            // The carve is 16128..59968; this boundary is past its far end.
            carved_with_boundary(1, 99_999, 99_999, "Test."),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-outside", &lines);
        whole_store(&at);
        let report = judge(&at);
        let line = clip_line(&report, "clip turn-01.wav");
        assert!(
            line.contains("no turn-01.command.wav (the STT boundary is outside this clip)"),
            "{line}"
        );
        let turns = sibling(&at, TURNS_SUFFIX);
        assert!(
            turns.join("turn-01.wav").exists(),
            "the clip the operator can open is written"
        );
        assert!(
            !turns.join("turn-01.command.wav").exists(),
            "and the one there is no audio for is not"
        );
    }

    /// A `wake_held` line that tore before its deadline states a hold with no
    /// budget. The turn still says the wake was held; it just does not invent a
    /// duration for it.
    #[test]
    fn a_hold_with_no_deadline_says_so_without_a_duration() {
        let torn = r#"{"ts_ms":2,"event":"wake_held","pod":"reachy00","epoch":1,"start_sample":16128,"end_sample":20000,"wake_end_sample":34880}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            torn.to_owned(),
            said(1, "Lights on.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-held-no-deadline", &lines);
        whole_store(&at);
        let report = judge(&at);
        let line = clip_line(&report, "clip turn-01.wav");
        assert!(line.contains("; held for the command"), "{line}");
        assert!(
            !line.contains("held up to"),
            "no duration is invented: {line}"
        );
    }

    /// The wake word kept in the clip: the boundary is still computed and still
    /// printed, and the second file still holds the audio from it.
    #[test]
    fn a_kept_wake_word_says_the_boundary_and_where_it_was_sent_from() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            carved_with_boundary(1, 13_504, 0, "Hey Jarvis, test."),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-kept", &lines);
        whole_store(&at);
        let report = judge(&at);
        let line = clip_line(&report, "clip turn-01.wav");
        assert!(
            line.contains("STT boundary +0.84 s, sent from +0.00 s"),
            "{line}"
        );
    }

    /// A wake held for its command, then answered: the turn line says the wait
    /// the listener was prepared to keep.
    #[test]
    fn a_held_wake_says_so_on_the_turn_it_opened() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            held_line(16_128, 20_000, 84_000),
            said(1, "Lights on.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-held", &lines);
        whole_store(&at);
        let report = judge(&at);
        let line = clip_line(&report, "clip turn-01.wav");
        assert!(line.contains("held up to 4.0 s for the command"), "{line}");
        assert!(line.contains("dispatched"), "one turn, not two: {line}");
    }

    /// A wake held for a command that never came. It is an arm expiry like any
    /// other and is counted apart from one: the wait ran out, which is the part
    /// an operator tunes.
    #[test]
    fn a_hold_that_ran_out_is_counted_apart_from_a_bare_arm_expiry() {
        const EXPIRED: &str = r#"{"ts_ms":8,"event":"wake_command_absent","pod":"reachy00","reason":"arm_expired","score":0.99,"log":"a.framelog","start_sample":16128,"end_sample":20000,"segments":[]}"#;
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            held_line(16_128, 20_000, 84_000),
            EXPIRED.to_owned(),
        ];
        let (_dir, at) = records("speech-report-held-expired", &lines);
        whole_store(&at);
        let report = judge(&at);
        assert!(
            measured(&report, "1 wake(s) held for a command that never came"),
            "{:?}",
            report.measured
        );
        assert!(
            measured(&report, "arm_expired after a hold ×1"),
            "{:?}",
            report.measured
        );
    }

    /// A hold whose wake line this console never carried belongs to no turn it
    /// saw. Written onto the last turn instead, it would grow a fragment -- and
    /// a tally entry -- on a turn that was never held.
    #[test]
    fn a_hold_with_no_wake_ahead_of_it_lands_on_no_turn() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Lights on.", 0.04),
            dispatched(1),
            held_line(60_000, 64_000, 128_000),
        ];
        let (_dir, at) = records("speech-report-held-orphan", &lines);
        whole_store(&at);
        let report = judge(&at);
        let line = clip_line(&report, "clip turn-01.wav");
        assert!(
            !line.contains("held up to"),
            "the finished turn was not the one held: {line}"
        );
    }

    /// The two clip names of a turn, as `//crates/reachy-host:stt_compare` pairs
    /// them: the second is the first with `.command` before the extension, for
    /// both kinds of turn. The comparison holds the same literals.
    #[test]
    fn the_two_clip_names_are_the_pair_the_comparison_looks_for() {
        assert_eq!(crate::Label::Utterance(1).file_name(), "turn-01.wav");
        assert_eq!(
            crate::Label::Utterance(1).command_file_name(),
            "turn-01.command.wav"
        );
        assert_eq!(crate::Label::Wake(3).file_name(), "turn-wake-3.wav");
        assert_eq!(
            crate::Label::Wake(3).command_file_name(),
            "turn-wake-3.command.wav"
        );
    }

    /// A carve whose pre-roll reaches back before the log's first frame is
    /// written all the same, and the line says how much of it is silence.
    #[test]
    fn a_carve_beginning_before_the_log_is_written_with_its_silence_stated() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            carved(1, 15_000, 16_000, None, 0.04, "Test."),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-preroll", &lines);
        whole_store(&at);
        let report = judge(&at);
        assert!(
            clip_line(&report, "clip turn-01.wav [15000–16000)")
                .contains("at most 360 samples of silence"),
            "{:?}",
            report.measured
        );
        let held = clip_samples(&sibling(&at, TURNS_SUFFIX).join("turn-01.wav"));
        assert!(held[..360].iter().all(|sample| *sample == 0), "the head");
        assert_eq!(held[360], sawtooth(AUDIO_BASE), "the log's first sample");
    }

    /// A fetch with no store beside it: one sentence about the run, not one
    /// refusal per turn, and no directory nobody put anything in.
    #[test]
    fn a_fetch_with_no_audio_writes_nothing_and_says_so_once() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-no-store", &lines);
        let report = judge(&at);
        assert!(
            measured(
                &report,
                "turn clips: none written — no recorded audio beside this fetch"
            ) && measured(
                &report,
                "clip not written (no recorded audio beside this fetch)"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            !sibling(&at, TURNS_SUFFIX).exists(),
            "nothing to hold means no directory"
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// And a store that came back empty, which is what a site recording nothing
    /// leaves: the rsync of a store with nothing in it succeeds, so the
    /// directory is there and holds no file.
    #[test]
    fn an_empty_store_reads_as_no_audio_and_not_as_a_missing_log() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-empty-store", &lines);
        std::fs::create_dir_all(sibling(&at, AUDIO_SUFFIX)).expect("an empty store");
        let report = judge(&at);
        assert!(
            measured(
                &report,
                "turn clips: none written — no recorded audio beside this fetch"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            !measured(&report, "is not in the store"),
            "an empty store is not a pruned log: {:?}",
            report.measured
        );
    }

    /// A store that holds no log of this name: the pruner is an independent
    /// actor, so this is a sentence about the audio and not a fault.
    #[test]
    fn a_log_the_store_does_not_hold_is_named_as_missing() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-pruned", &lines);
        store(&at, "b.framelog", 4);
        let report = judge(&at);
        assert!(
            measured(&report, "clip not written (a.framelog is not in the store)"),
            "{:?}",
            report.measured
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// A run nobody spoke in says nothing about clips: the count of an empty
    /// set is not a reading, and the summary already says the run had no wake
    /// in it.
    #[test]
    fn a_run_with_no_turns_says_nothing_about_clips() {
        let lines = [STARTED.to_owned(), COMPOSED.to_owned()];
        let (_dir, at) = records("speech-report-clip-no-turns", &lines);
        let report = judge(&at);
        assert!(
            !report
                .measured
                .iter()
                .any(|line| line.starts_with("turn clips:")),
            "{:?}",
            report.measured
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// A store fault is said and never mistaken for silence, and never kills
    /// the report the run is read by.
    #[test]
    fn a_log_that_will_not_open_is_said_and_the_report_goes_on() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-fault", &lines);
        let dir = sibling(&at, AUDIO_SUFFIX);
        std::fs::create_dir_all(&dir).expect("a store");
        std::fs::write(dir.join("a.framelog"), b"not a frame log at all").expect("a bad log");
        let report = judge(&at);
        assert!(
            clip_line(&report, "clip not written (a.framelog:").contains("dispatched"),
            "{:?}",
            report.measured
        );
        assert!(
            measured(&report, "turn clips: none written"),
            "{:?}",
            report.measured
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// A span the console did not finish stating is not carved with: the
    /// fragment prints as missing, as every value this tool cannot state does.
    #[test]
    fn a_turn_whose_span_is_missing_a_field_writes_no_clip() {
        const NO_END: &str = r#"{"ts_ms":2,"event":"utterance","id":1,"pod":"reachy00","audio_ref":{"log":"a.framelog","start_sample":16128,"segments":[]},"transcript":{"text":"Test.","confidence":{"no_speech_prob":0.04}}}"#;
        const NO_LOG: &str = r#"{"ts_ms":2,"event":"utterance","id":1,"pod":"reachy00","audio_ref":{"start_sample":16128,"end_sample":59968,"segments":[]},"transcript":{"text":"Test.","confidence":{"no_speech_prob":0.04}}}"#;
        for (name, line, says) in [
            ("speech-report-clip-no-end", NO_END, "; clip ?;"),
            // The one field that names the file is the one this line lost, so
            // the coordinates stay: they are what somebody opening the store by
            // hand has to have.
            (
                "speech-report-clip-no-log",
                NO_LOG,
                "; clip ? [16128–59968)",
            ),
        ] {
            let lines = [STARTED, COMPOSED, &wake(0.99), line, &dispatched(1)];
            let (_dir, at) = records(name, &lines);
            whole_store(&at);
            let report = judge(&at);
            assert!(measured(&report, says), "{name}: {:?}", report.measured);
            assert!(
                !sibling(&at, TURNS_SUFFIX).exists(),
                "{name}: nothing is written"
            );
        }
    }

    /// A sample index a torn line lost its sign on is refused in the resolver's
    /// own words, and nothing panics.
    #[test]
    fn a_span_that_is_not_a_sample_index_is_refused() {
        const NEGATIVE: &str = r#"{"ts_ms":2,"event":"utterance","id":1,"pod":"reachy00","audio_ref":{"log":"a.framelog","start_sample":-16128,"end_sample":59968,"segments":[]},"transcript":{"text":"Test.","confidence":{"no_speech_prob":0.04}}}"#;
        let lines = [STARTED, COMPOSED, &wake(0.99), NEGATIVE, &dispatched(1)];
        let (_dir, at) = records("speech-report-clip-negative", &lines);
        whole_store(&at);
        let report = judge(&at);
        assert!(
            measured(&report, "clip not written (invalid span)"),
            "{:?}",
            report.measured
        );
        assert!(!sibling(&at, TURNS_SUFFIX).exists(), "nothing is written");
    }

    /// The clips are a function of the fetch, so reading a run twice writes the
    /// same bytes — and leaves whatever else is in the directory alone.
    #[test]
    fn reading_one_fetch_twice_writes_the_same_clips() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-twice", &lines);
        whole_store(&at);
        judge(&at);
        let turns = sibling(&at, TURNS_SUFFIX);
        std::fs::write(turns.join("someone-elses.txt"), b"kept").expect("a stranger's file");
        let first = std::fs::read(turns.join("turn-01.wav")).expect("a clip");
        judge(&at);
        assert_eq!(
            std::fs::read(turns.join("turn-01.wav")).expect("a clip"),
            first,
            "the same fetch reads the same bytes"
        );
        assert_eq!(
            std::fs::read(turns.join("someone-elses.txt")).expect("the stranger's file"),
            b"kept",
            "nothing else in the directory is touched"
        );
    }
    /// A log whose tail was cut off mid-frame still yields the audio it holds:
    /// partial beats nothing, and the line says the record stopped so a silent
    /// stretch is not read as a wire gap.
    #[test]
    fn a_torn_log_is_written_and_the_line_says_it_was_torn() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-torn-log", &lines);
        let dir = whole_store(&at);
        let log = dir.join("a.framelog");
        let whole = std::fs::metadata(&log).expect("a log").len();
        std::fs::OpenOptions::new()
            .write(true)
            .open(&log)
            .expect("the log")
            .set_len(whole / 2)
            .expect("a torn tail");
        let report = judge(&at);
        assert!(
            clip_line(&report, "clip turn-01.wav [16128–59968)")
                .contains(", a.framelog torn (torn tail)"),
            "{:?}",
            report.measured
        );
        assert_eq!(
            clip_samples(&sibling(&at, TURNS_SUFFIX).join("turn-01.wav"))[0],
            sawtooth(16128),
            "what the log did hold is written"
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }
    /// A store that would not open is said, and never in the words for a store
    /// that recorded nothing: the audio is on disk and the fault is the one
    /// thing an operator has to know to go and get it.
    #[test]
    fn a_store_that_will_not_open_is_not_read_as_no_audio() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-unreadable-store", &lines);
        let store = sibling(&at, AUDIO_SUFFIX);
        std::fs::write(&store, b"not a store at all").expect("a store that is a file");
        let report = judge(&at);
        let line = clip_line(&report, "clip not written (");
        assert!(
            line.contains(&format!("clip not written ({}: ", store.display())),
            "{:?}",
            report.measured
        );
        assert!(
            !measured(&report, "no recorded audio beside this fetch"),
            "a fault is not silence: {:?}",
            report.measured
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// A wake nobody answered carries its own span on the decline line, so it
    /// is cut like any other turn — and the file is named by the ordinal its
    /// line prints, since there is no utterance id to name it by.
    #[test]
    fn a_wake_nobody_answered_is_cut_under_its_wake_name() {
        const UNANSWERED: &str = r#"{"ts_ms":3,"event":"wake_command_absent","pod":"reachy00","reason":"arm_expired","score":0.99,"log":"a.framelog","start_sample":16128,"end_sample":20000,"segments":[]}"#;
        let lines = [STARTED, COMPOSED, &wake(0.99), UNANSWERED];
        let (_dir, at) = records("speech-report-clip-wake-name", &lines);
        whole_store(&at);
        let report = judge(&at);
        assert!(
            clip_line(&report, "clip turn-wake-1.wav [16128–20000)")
                .starts_with("turn #? (wake 1)"),
            "{:?}",
            report.measured
        );
        let held = clip_samples(&sibling(&at, TURNS_SUFFIX).join("turn-wake-1.wav"));
        assert_eq!(held.len(), 20000 - 16128, "the carve's own length");
        assert_eq!(held[0], sawtooth(16128), "the decline line named the log");
    }

    /// A store that covered part of the carve twice says the silence figure is
    /// unknown rather than zero: an outage read as silence is the one wrong
    /// inference this fragment exists to prevent.
    #[test]
    fn a_span_the_store_covered_twice_leaves_its_silence_unknown() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            carved(
                1,
                AUDIO_BASE as i64,
                (AUDIO_BASE + FRAME as u64) as i64,
                None,
                0.04,
                "Test.",
            ),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-covered-twice", &lines);
        let dir = sibling(&at, AUDIO_SUFFIX);
        std::fs::create_dir_all(&dir).expect("an audio store");
        pod_ingest::test_fixtures::write_log(
            &dir.join("a.framelog"),
            &[
                pod_ingest::test_fixtures::hello("reachy00"),
                pod_ingest::test_fixtures::seg_start(1, AUDIO_BASE),
                pod_ingest::test_fixtures::audio(1, AUDIO_BASE, FRAME),
                // The same samples a second time, which is what a store that
                // re-covered a span holds.
                pod_ingest::test_fixtures::audio(1, AUDIO_BASE, FRAME),
                pod_ingest::test_fixtures::seg_end(1, FRAME as u64),
            ],
        );
        let report = judge(&at);
        assert!(
            clip_line(&report, "clip turn-01.wav").contains(
                ", how much of it is silence is unknown (the store covered part of this span twice)"
            ),
            "{:?}",
            report.measured
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// A replay that met a non-fatal protocol error counts it on the line: the
    /// samples it lost are silence in the clip and are otherwise
    /// indistinguishable from a wire gap.
    #[test]
    fn a_replay_that_met_a_protocol_error_says_how_many() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            carved(
                1,
                AUDIO_BASE as i64,
                (AUDIO_BASE + FRAME as u64) as i64,
                None,
                0.04,
                "Test.",
            ),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-protocol-errors", &lines);
        let dir = sibling(&at, AUDIO_SUFFIX);
        std::fs::create_dir_all(&dir).expect("an audio store");
        pod_ingest::test_fixtures::write_log(
            &dir.join("a.framelog"),
            &[
                pod_ingest::test_fixtures::hello("reachy00"),
                // Audio ahead of any segment: a non-fatal protocol error, and
                // replay goes on.
                pod_ingest::test_fixtures::audio(1, AUDIO_BASE, FRAME),
                pod_ingest::test_fixtures::seg_start(1, AUDIO_BASE),
                pod_ingest::test_fixtures::audio(1, AUDIO_BASE, FRAME),
                pod_ingest::test_fixtures::seg_end(1, FRAME as u64),
            ],
        );
        let report = judge(&at);
        assert!(
            clip_line(&report, "clip turn-01.wav").contains(", 1 protocol errors"),
            "{:?}",
            report.measured
        );
    }

    /// The three ways a replay stops read alike collapsed into one word, and
    /// the resolver's message is the only description of what was wrong with
    /// the store: a corrupt record says so, and says what it said.
    #[test]
    fn a_corrupt_record_names_what_stopped_the_replay() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            said(1, "Test.", 0.04),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-corrupt-log", &lines);
        let dir = store(&at, "a.framelog", 4);
        // A whole record header claiming a length no frame can have, which is
        // the corruption a torn tail is not.
        let mut header = 99u64.to_le_bytes().to_vec();
        header.extend_from_slice(&u16::MAX.to_le_bytes());
        std::io::Write::write_all(
            &mut std::fs::OpenOptions::new()
                .append(true)
                .open(dir.join("a.framelog"))
                .expect("the log"),
            &header,
        )
        .expect("a corrupt record");
        let report = judge(&at);
        assert!(
            clip_line(&report, "clip turn-01.wav").contains(", a.framelog torn (corrupt: "),
            "{:?}",
            report.measured
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// A span whose end precedes its start is the resolver's own refusal, in
    /// the resolver's own words — the same fact and the same sentence as a
    /// sample index this tool refused before asking.
    #[test]
    fn a_span_the_resolver_refuses_is_said_in_its_own_words() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            carved(1, 30000, 20000, None, 0.04, "Test."),
            dispatched(1),
        ];
        let (_dir, at) = records("speech-report-clip-backwards-span", &lines);
        whole_store(&at);
        let report = judge(&at);
        assert!(
            measured(&report, "clip not written (invalid span)"),
            "{:?}",
            report.measured
        );
        assert!(!sibling(&at, TURNS_SUFFIX).exists(), "nothing is written");
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }

    /// Two turns without clips for two different reasons name neither in the
    /// summary: the one reason is stated only where it is the whole story, and
    /// each turn's own line carries its own.
    #[test]
    fn two_unwritten_turns_with_different_reasons_share_none() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            carved(1, 16128, 20000, None, 0.04, "One."),
            dispatched(1),
            wake(0.99),
            carved(2, 30000, 20000, None, 0.04, "Two."),
            dispatched(2),
        ];
        let (_dir, at) = records("speech-report-clip-mixed-reasons", &lines);
        // A store holding some other run's log: turn 1's log is pruned from it.
        store(&at, "b.framelog", 4);
        let report = judge(&at);
        assert!(
            measured(&report, "clip not written (a.framelog is not in the store)")
                && measured(&report, "clip not written (invalid span)"),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line == "turn clips: none written"),
            "{:?}",
            report.measured
        );
    }

    /// A clips directory that cannot be made is one fact about the filesystem,
    /// said the same way on every turn and attempted once.
    #[test]
    fn a_clips_directory_that_cannot_be_made_is_one_fact() {
        let lines = [
            STARTED.to_owned(),
            COMPOSED.to_owned(),
            wake(0.99),
            carved(1, 16128, 20000, None, 0.04, "One."),
            dispatched(1),
            wake(0.99),
            carved(2, 30000, 40000, None, 0.04, "Two."),
            dispatched(2),
        ];
        let (_dir, at) = records("speech-report-clip-unwritable-dir", &lines);
        whole_store(&at);
        let turns = sibling(&at, TURNS_SUFFIX);
        std::fs::write(&turns, b"in the way").expect("a file where the directory goes");
        let report = judge(&at);
        let refusals: Vec<&String> = report
            .measured
            .iter()
            .filter(|line| line.starts_with("turn #"))
            .collect();
        assert_eq!(refusals.len(), 2, "{:?}", report.measured);
        for line in &refusals {
            assert!(
                line.contains(&format!("clip not written ({}: ", turns.display())),
                "{:?}",
                report.measured
            );
        }
        assert_eq!(
            std::fs::read(&turns).expect("the file in the way"),
            b"in the way",
            "nothing wrote through it"
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
    }
    /// A log name is console-authored text, so it is printed the way every
    /// other free string off the console is: a name carrying a newline would
    /// otherwise fabricate a line of this report, which reads by prefix.
    #[test]
    fn a_log_name_off_a_torn_line_cannot_fabricate_a_report_line() {
        const SPLIT: &str = r#"{"ts_ms":2,"event":"utterance","id":1,"pod":"reachy00","audio_ref":{"log":"a.framelog\nturn clips: 9 of 9 written to nowhere","start_sample":16128,"end_sample":20000,"segments":[]},"transcript":{"text":"Test.","confidence":{"no_speech_prob":0.04}}}"#;
        let lines = [STARTED, COMPOSED, &wake(0.99), SPLIT, &dispatched(1)];
        let (_dir, at) = records("speech-report-clip-split-log-name", &lines);
        whole_store(&at);
        let report = judge(&at);
        assert!(
            report.measured.iter().all(|line| !line.contains('\n')),
            "{:?}",
            report.measured
        );
        assert_eq!(
            report
                .measured
                .iter()
                .filter(|line| line.starts_with("turn clips:"))
                .count(),
            1,
            "{:?}",
            report.measured
        );
        assert!(
            measured(
                &report,
                "clip not written (a.framelog turn clips: 9 of 9 written to nowhere is not in the store)"
            ),
            "{:?}",
            report.measured
        );
    }
}
