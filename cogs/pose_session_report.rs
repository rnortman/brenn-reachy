//! What one recording session holds, read off the pose stream it wrote.
//!
//! The offline half of the pose recorder, beside `speech_run_report` and on the
//! same premise: it reads a fetched record directory and nothing else, so a
//! session recorded on a unit last week reads the same way as one that finished
//! a second ago. The input is `<records>.console/recorder_0.log` — one JSON
//! line per 50 Hz reading of the nine servos, taken with the torque off and a
//! pair of hands on the head — and what comes out is a session document an
//! operator and an LLM turn into clips.
//!
//! The stream's line schema is the recorder's own: `reachy_bench::poselog`
//! declares it and both ends read that declaration, because a reader with its
//! own copy of the field names would answer a renamed stream with "no samples"
//! and cost a hardware session to diagnose.
//!
//! Three things happen to the stream. It is checked for holes: a stamp step
//! outside 20 ± 10 ms is a gap and a backwards step is a wall-clock step, both
//! recorded rather than smoothed over. It is segmented into the still and
//! moving stretches by `reachy-motion`'s `MotionSegmenter`, run offline over the
//! samples rather than read off the recorder's own live notes — those were the
//! operator's feedback, the samples are the record, and the segmentation is
//! recomputed here under whatever configuration the flags give so that tuning a
//! threshold is a re-run and not a rebuild. And it is solved: forward
//! kinematics per sample, each seeded with the previous sample's answer and
//! falling back to the resting seeds, which is why the offline pass beats a live
//! one — a hold's pose is solved from the mean of the hold rather than from any
//! one reading of it.
//!
//! The session's speech is the other half. `<records>.console/voice_host_0.log`
//! is the voice pipeline's own JSONL record of everything the operator said —
//! one `utterance` line per carve, with the transcript, its confidence figures
//! and the host-receipt stamps of the audio that drove the endpointer. Those
//! stamps are walked back into the interval the speech itself covers, the
//! interval is joined to the segments around it, and the audio behind each one
//! is cut out of the fetched store as a `.wav` beside the document. Both streams
//! stamp `SystemTime::now()` on one compute module, so the join is arithmetic on
//! one clock rather than an offset anybody estimated — what is unequal is
//! latency, and a speech stamp is late by the pod's capture path and the hop to
//! the host where a pose stamp is within two milliseconds of the physical event.
//!
//! The join is three references and not one verdict: the hold before an
//! utterance, whatever it overlapped, and the hold after it. An operator speaks
//! before settling, while holding, or after letting go, and which they did is
//! what the LLM turn reads off their own words.
//!
//! One segment at a time then leaves as a clip draft. `--extract` writes the
//! stretch an operator names as a clip document in the format the daemon's own
//! loader reads: every channel a delta over the neutral base, one frame per
//! reading of a move and one frame at the mean of a hold. The conversion is
//! `reachy-clips`, not this tool, and the draft is raw — a hand's path at 50 Hz
//! with tremor in it, material for the smoothing and trimming step, and a
//! document the loader may well refuse because a hand can hold the head where
//! the envelope will not.
//!
//! The verdict is permissive in the way the speech analyzer's is. What an
//! operator did with their hands decides what is in the stream, and this tool
//! holds no opinion about it. It holds an opinion about whether the run was a
//! recording session at all: a fetch with no pose stream, a stream that refused
//! before it started, a stream with no samples in it, or a voice host that never
//! reported `listening` — a session recorded with no transcription running is
//! poses nobody can label. Everything else is measured and printed.

#![forbid(unsafe_code)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use dxl_proto::conv::counts_to_rad;
use nalgebra::Isometry3;
use reachy_bench::poselog::{
    POSE_LOG_PERIOD, POSE_LOG_PERIOD_MS, PoseLine, PoseLogEnd, SampleLine, SegmenterLine,
    StartedLine,
};
use reachy_clips::format::{ChannelMask, ClipDoc};
use reachy_clips::record::{Draft, RecordError, RecordedFrame, clip_doc};
use reachy_kin::{FkOptions, cone_angle, default_geometry, neutral_head_pose};
use reachy_motion::arm::ArmRecord;
use reachy_motion::joints::{
    JointGroup, JointRef, JointVector, ROW_COUNT, ROWS, flags, group_of_row, row,
};
use reachy_motion::rest_pose_seeds;
use reachy_motion::segments::{JointSample, MotionSegmenter, Segment, SegmentConfig, SegmentKind};
use run_report::event::{
    LISTENING, PLAYBACK_FINISHED, PLAYBACK_FLUSHED, PLAYBACK_STARTED, UTTERANCE,
};
use run_report::{EVENT_HEAD, Report, audio_dir, console_dir, quote, recover, write_verdict};
use serde::Serialize;
use serde_json::Value;
use speech_pipeline::listener::silero::{SILERO_CHUNK, SILERO_SAMPLE_RATE};

/// The launcher's name for the recorder's console, which is the pose stream.
const RECORDER_LOG: &str = "recorder_0.log";

/// The launcher's name for the voice host's console, which is the speech record.
const HOST_LOG: &str = "voice_host_0.log";

/// A playback the writer gave up on. Terminal, like a flush: the job settles on
/// it and no finish follows it.
const PLAYBACK_ABORTED: &str = "playback_aborted";

/// The directory, inside a fetched run, holding the configuration it ran under.
const CONFIG_DIR: &str = "config";

/// The recording session's speech configuration, at its payload-relative path.
///
/// Read for three endpointer values and nothing else. A run that carries no copy
/// of it is read under the pipeline's own defaults, which is a note rather than
/// a refusal: the defaults are what an unconfigured `[endpointer]` runs at, and
/// a session is worth reading whether or not its configuration came home.
const RECORD_SPEECH_CONFIG: &str = "host/speech-record.toml";

/// The grid the recorder samples on, milliseconds: the writer's own constant.
///
/// A sample step is judged against this rather than against the `started`
/// line's claim — a stream recorded on another period is a stream whose gaps
/// this tool would report against the wrong baseline rather than silently
/// accept — and it is the recorder's constant rather than a restatement of the
/// literal, so a change to the grid moves both ends at once. The stream's own
/// figure travels in the document beside the gaps, and a disagreement is a
/// note.
const PERIOD_MS: f64 = POSE_LOG_PERIOD_MS as f64;

/// How far a pair of samples may sit from what is expected before it is a hole,
/// milliseconds.
///
/// Two readings are judged against it: the grid clock's own interval against
/// [`PERIOD_MS`], which is a gap when it is off by more than this, and the wall
/// clock's interval against the grid's, which is a step when the two disagree
/// by more than this. One slack for both because both measure the same thing —
/// how far a pair of stamps may sit from where a healthy loop puts them before
/// a reader is told about it.
const SPACING_SLACK_MS: f64 = 10.0;

/// The whole pose stream, as parsed.
#[derive(Default)]
struct Stream {
    /// The opening line, absent on a stream that refused or was truncated.
    started: Option<StartedLine>,
    /// Every sample, in the order it was written.
    samples: Vec<Sample>,
    /// The closing line's cause, absent on a truncated stream. Carried as the
    /// writer's own type rather than as a word this file spells: a variant
    /// renamed on the wire would otherwise change the document's string and
    /// nothing would say so.
    ended: Option<PoseLogEnd>,
    /// What ended a run that ended on a failure, off the closing line.
    ended_error: Option<String>,
    /// What a refusal said, absent on a stream that ran.
    refused: Option<String>,
    /// Why the stream could not be read at all, for anything but a fetch that
    /// holds no such file: a directory that cannot be opened is not a session
    /// that recorded nothing.
    unreadable: Option<String>,
    /// How many `late` notes the recorder wrote.
    late: usize,
    /// Lines that opened like one of the recorder's own — a leading brace — and
    /// would not parse: a torn or truncated record.
    torn: usize,
    /// Lines that were not the record at all. The launcher merges an app's
    /// stderr into this file, so what the recorder said to the operator and
    /// anything else sharing the descriptor lands here; it is skipped without
    /// comment, which is what keeps `torn` a count worth reading.
    prose: usize,
}

/// One reading, converted.
struct Sample {
    /// When it was taken, on the wall clock the voice host also stamps.
    t_ns: i64,
    /// The same instant on the clock the recorder's grid is paced on. It does
    /// not step, so differencing the two says whether a widened interval was
    /// the wall clock moving or the stream missing a stretch.
    mono_ns: i64,
    /// Measured angles, radians. A row that did not answer holds zero and is
    /// named in `missing`, exactly as the segmenter's sample expects.
    present: JointVector,
    /// The rows that did not answer.
    missing: [bool; ROW_COUNT],
}

impl Stream {
    /// Read the pose stream out of a fetched record directory.
    ///
    /// A directory with no such file yields an empty stream, which is the shape
    /// the verdict is red on.
    ///
    /// The file holds more than the record: the launcher merges every app's
    /// stderr into it, so the recorder's own opening words to the operator are
    /// in there too. A line that is not the record is therefore only worth a
    /// note when it looks like one that tore — otherwise the count that catches
    /// a truncated fetch would have a non-zero floor and nobody would read it.
    ///
    /// Read as bytes and decoded a line at a time, not as one string: the
    /// operator's own lines in this file carry multibyte characters, and a fetch
    /// that copied it mid-write can tear one. Decoding the whole file at once
    /// would answer a single torn character by discarding a session of perfectly
    /// good ASCII records — and answer it as "there is no such file".
    fn read(records: &Path) -> Self {
        let path = console_dir(records).join(RECORDER_LOG);
        let bytes = match std::fs::read(&path) {
            Ok(bytes) => bytes,
            Err(error) if error.kind() == std::io::ErrorKind::NotFound => return Self::default(),
            Err(error) => {
                return Self {
                    unreadable: Some(format!("{} could not be read: {error}", path.display())),
                    ..Self::default()
                };
            }
        };
        let text = String::from_utf8_lossy(&bytes);
        let mut stream = Self::default();
        for raw in text.lines() {
            match serde_json::from_str::<PoseLine>(raw) {
                Ok(PoseLine::Started(started)) => stream.started = Some(started),
                Ok(PoseLine::Sample(sample)) => stream.samples.push(sample_of(&sample)),
                Ok(PoseLine::Late(_)) => stream.late += 1,
                // The live notes, parsed and dropped: they were the operator's
                // feedback, and the samples under them are what the
                // segmentation is recomputed from.
                Ok(PoseLine::Still(_) | PoseLine::Moving(_)) => {}
                Ok(PoseLine::Ended(ended)) => {
                    stream.ended = Some(ended.cause);
                    stream.ended_error = ended.error;
                }
                Ok(PoseLine::Refused(refused)) => stream.refused = Some(refused.error),
                Err(_) if raw.trim().is_empty() => {}
                Err(_) if raw.trim_start().starts_with('{') => stream.torn += 1,
                Err(_) => stream.prose += 1,
            }
        }
        stream
    }
}

/// One sample line, converted into angles and a missing-row set.
///
/// The whole line, not its fields: the reader takes what it needs off the
/// schema the recorder wrote, so a field added there costs no signature here
/// and two stamps of the same type cannot be handed over in the wrong order.
fn sample_of(line: &SampleLine) -> Sample {
    let mut present = JointVector::default();
    let mut missing = [false; ROW_COUNT];
    for (row, joint) in ROWS.into_iter().enumerate() {
        match line.counts[row] {
            Some(count) => {
                present.set(joint, counts_to_rad(count));
            }
            None => missing[row] = true,
        }
    }
    Sample {
        t_ns: line.t_ns,
        mono_ns: line.mono_ns,
        present,
        missing,
    }
}

/// The audio chunk the pipeline's voice detector counts in, milliseconds.
///
/// The endpointer's onset run is a whole number of these, which is why the
/// configuration states an onset in chunks and a hangover in milliseconds — and
/// why a hangover that is not a whole number of them is not the one the
/// detector ran. Derived from the detector's own quantum rather than restated:
/// the chunk is another repository's number, and a session derived against a
/// stale copy of it would be silently, systematically early.
const CHUNK_MS: u32 = (SILERO_CHUNK as u32) * 1_000 / (SILERO_SAMPLE_RATE as u32);

/// How many of the detector's chunks fit in `ms`, as the listener counts them.
///
/// Integer division: the detector quantizes to whole chunks, so a configured
/// 400 ms is twelve chunks and the hangover the detector held was 384 ms.
/// Subtracting the configured figure instead would put every utterance's end
/// stamp up to one chunk early, in one direction, in exactly the session
/// somebody tuned.
const fn whole_chunks(ms: u32) -> u32 {
    ms / CHUNK_MS * CHUNK_MS
}

/// The endpointer values a speech interval is derived from, milliseconds.
///
/// The three the derivation needs, in the units the derivation uses them in:
/// how much speech precedes the chunk that completed the onset run, how much
/// silence precedes the chunk that closed the utterance, and how far ahead of
/// its first audio a fallback carve begins. They travel in the document, because
/// an interval means nothing without them.
#[derive(Clone, Copy, Debug, Serialize)]
struct Endpointer {
    /// The onset run, milliseconds: `onset_chunks` of the detector's chunk.
    onset_ms: u32,
    /// The silence that closes an utterance, milliseconds, as the detector
    /// counted it: whole chunks, which is what a configured window quantizes
    /// to.
    soft_hangover_ms: u32,
    /// How far ahead of its first audio a missed-onset carve is taken to start,
    /// milliseconds.
    preroll_pad_ms: u32,
}

impl Default for Endpointer {
    /// The pipeline's own defaults, which is what an unstated `[endpointer]`
    /// runs at.
    ///
    /// Read off the detector's own configuration type rather than retyped:
    /// these are another repository's numbers behind a pin, and a bump that
    /// moved one would otherwise leave this tool deriving intervals against a
    /// value no session ever ran, printed in the document as if it were
    /// measured.
    fn default() -> Self {
        let ran = speech_pipeline::EndpointerConfig::default();
        Self {
            onset_ms: ran.onset_chunks * CHUNK_MS,
            soft_hangover_ms: ran.soft_hangover_chunks * CHUNK_MS,
            preroll_pad_ms: u32::try_from(
                ran.preroll_pad_samples * 1_000 / (SILERO_SAMPLE_RATE as u64),
            )
            .unwrap_or(u32::MAX),
        }
    }
}

impl Endpointer {
    /// What the run's own speech configuration says, defaulting per key.
    ///
    /// Per key rather than per file: a configuration stating one of the three
    /// runs the pipeline's default for the other two, and reading it any other
    /// way would derive intervals against values the session did not use. A
    /// whole-file deserialize into the surface's own table would answer a
    /// key it has not heard of, or a table the fetch copied mid-write, by
    /// falling back on all three at once.
    ///
    /// What is read is the file's figure; what is kept is the figure the
    /// detector ran on, which for the hangover is whole chunks of it.
    fn read(records: &Path, notes: &mut Vec<String>) -> Self {
        let path = records.join(CONFIG_DIR).join(RECORD_SPEECH_CONFIG);
        let mut endpointer = Self::default();
        let text = match std::fs::read_to_string(&path) {
            Ok(text) => text,
            Err(error) => {
                notes.push(format!(
                    "{}: {error}. The speech intervals are derived under the pipeline's own \
                     endpointer defaults",
                    path.display()
                ));
                return endpointer;
            }
        };
        let table = match text.parse::<toml::Table>() {
            Ok(table) => table,
            Err(error) => {
                notes.push(format!(
                    "{} could not be parsed: {error}. The speech intervals are derived under the \
                     pipeline's own endpointer defaults",
                    path.display()
                ));
                return endpointer;
            }
        };
        // A key the file states and this cannot read is said, where a key it
        // does not state is not: the surface types all three as whole numbers,
        // so a float, a string or a negative there is a configuration the host
        // itself would have refused. Reading it as "use the default" would
        // print the default under `session.endpointer` as if the session had
        // run it, and derive every interval against a value nothing ran.
        let mut malformed: Vec<String> = Vec::new();
        let mut whole = |key: &str| -> Option<u32> {
            let stated = table.get("endpointer")?.as_table()?.get(key)?;
            match stated
                .as_integer()
                .and_then(|value| u32::try_from(value).ok())
            {
                Some(value) => Some(value),
                None => {
                    malformed.push(quote(&format!("{key} = {stated}")));
                    None
                }
            }
        };
        if let Some(chunks) = whole("onset_chunks") {
            endpointer.onset_ms = chunks.saturating_mul(CHUNK_MS);
        }
        if let Some(ms) = whole("soft_hangover_ms") {
            endpointer.soft_hangover_ms = whole_chunks(ms);
        }
        if let Some(ms) = whole("preroll_pad_ms") {
            endpointer.preroll_pad_ms = ms;
        }
        if !malformed.is_empty() {
            notes.push(format!(
                "{} states {} under [endpointer], which is not a whole number of them: those \
                 intervals are derived under the pipeline's own defaults",
                path.display(),
                malformed.join(", ")
            ));
        }
        endpointer
    }
}

/// Where in the pod's audio one utterance was carved from.
#[derive(Clone, Debug)]
struct Carve {
    /// Which frame log in the fetched store holds those samples.
    log: String,
    /// The first sample, absolute in the pod's own index space.
    start_sample: i64,
    /// One past the last sample.
    end_sample: i64,
}

/// What one `utterance` line said.
///
/// Everything is read optionally, because everything is stated optionally: a
/// line written by a pipeline that names a field differently leaves the figure
/// that needs it missing rather than a number nobody can check.
struct Spoken {
    /// Which utterance of this record it is, counted from one. The pipeline's
    /// own id is beside it and is not the same number — a host restarted
    /// mid-session counts its own from zero again.
    seq: usize,
    /// The pipeline's id for it.
    id: Option<u64>,
    /// What the recogniser made of it. Empty on a line that carried no
    /// transcript at all, which is a carve the parrot declined to read back.
    text: String,
    /// The three confidence figures, as the line carried them.
    no_speech: Option<f64>,
    logprob: Option<f64>,
    compression: Option<f64>,
    /// Why the carve ended where it did.
    endpoint_cause: String,
    /// Whether the carve's t0 was projected off the device clock rather than
    /// measured, which makes every stamp derived from it fuzzy in one
    /// direction.
    t0_projected: bool,
    /// Host receipt of the audio that drove the onset, microseconds.
    onset_rx_us: Option<i64>,
    /// Host receipt of the audio the carve ends on, microseconds.
    soft_endpoint_rx_us: Option<i64>,
    /// Host receipt of the carve's first audio, microseconds.
    first_audio_rx_us: Option<i64>,
    /// Where its audio is, where the line said.
    carve: Option<Carve>,
}

/// The interval the speech itself covers, on the session's own clock.
struct Interval {
    t_start_ns: i64,
    t_end_ns: i64,
    /// Whether the start is the missed-onset estimate rather than the
    /// endpointer's own onset walked back.
    start_estimated: bool,
}

impl Spoken {
    /// The speech interval this utterance covers, or nothing where its line
    /// carried no stamp the interval can be built from.
    ///
    /// Both ends are walked back from a host-receipt stamp by the endpointer's
    /// own configuration: the onset stamp is the receipt of the chunk that
    /// *completed* the onset run, so the speech began an onset run earlier, and
    /// the endpoint stamp is the receipt of the chunk that closed the utterance
    /// after the hangover, so the speech ended a hangover earlier. A carve that
    /// never onset — the missed-onset fallback — is estimated forward from its
    /// first audio instead and says so.
    fn interval(&self, endpointer: &Endpointer) -> Option<Interval> {
        let end_us = self.soft_endpoint_rx_us?;
        let (start_us, start_estimated) = match self.onset_rx_us {
            Some(onset) => (onset - i64::from(endpointer.onset_ms) * 1_000, false),
            None => (
                self.first_audio_rx_us? + i64::from(endpointer.preroll_pad_ms) * 1_000,
                true,
            ),
        };
        let t_start_ns = start_us * 1_000;
        let t_end_ns = (end_us - i64::from(endpointer.soft_hangover_ms) * 1_000) * 1_000;
        Some(Interval {
            // A hangover longer than the carve, or an estimate that ran past
            // the endpoint, would otherwise be an interval that ends before it
            // starts: the join windows are written against a forward interval,
            // and an instant is the honest reading of one that inverted.
            t_end_ns: t_end_ns.max(t_start_ns),
            t_start_ns,
            start_estimated,
        })
    }
}

/// One stretch of the pod playing something back.
struct Played {
    start_ns: i64,
    end_ns: i64,
    /// Whether the record holds the line that closed it.
    closed: bool,
}

/// The session's speech, as the voice host recorded it.
#[derive(Default)]
struct Voice {
    /// Whether the pipeline ever said it was listening. A session recorded
    /// without it is poses nobody can label.
    listening: bool,
    /// Every utterance, in the order the record carries them.
    said: Vec<Spoken>,
    /// Every playback, for the overlap the parrot's read-back creates.
    playbacks: Vec<Played>,
    /// The endpointer the intervals are derived under.
    endpointer: Endpointer,
    /// What reading the record found worth saying.
    notes: Vec<String>,
}

impl Voice {
    /// Read the voice host's console out of a fetched record directory.
    ///
    /// A fetch with no such file yields a record with nothing in it, which is
    /// the shape the verdict is red on: the pose stream alone is a session
    /// nobody can label.
    ///
    /// Every line is read through the shared tear recovery: the launcher merges
    /// the host's stdout and stderr into this one file, so the pipeline's own
    /// human console — which is what the operator watched the session on — is
    /// interleaved with the JSONL and glued onto it.
    fn read(records: &Path) -> Self {
        let mut voice = Self::default();
        let path = console_dir(records).join(HOST_LOG);
        voice.endpointer = Endpointer::read(records, &mut voice.notes);
        let bytes = match std::fs::read(&path) {
            Ok(bytes) => bytes,
            Err(error) => {
                voice
                    .notes
                    .push(format!("{} could not be read: {error}", path.display()));
                return voice;
            }
        };
        let text = String::from_utf8_lossy(&bytes);
        let mut last_ns = 0_i64;
        for raw in text.lines() {
            for value in recover(raw, EVENT_HEAD).values {
                let Some(event) = value.get("event").and_then(Value::as_str) else {
                    continue;
                };
                let at_ns = value
                    .get("ts_ms")
                    .and_then(Value::as_i64)
                    .map_or(last_ns, |ms| ms * 1_000_000);
                last_ns = at_ns;
                match event {
                    LISTENING => voice.listening = true,
                    UTTERANCE => {
                        let seq = voice.said.len() + 1;
                        voice.said.push(spoken(seq, &value));
                    }
                    PLAYBACK_STARTED => voice.playbacks.push(Played {
                        start_ns: at_ns,
                        end_ns: at_ns,
                        closed: false,
                    }),
                    // Three closers and not one: a reply cut short by somebody
                    // speaking over it ends on a flush and a reply the writer
                    // gave up on ends on an abort, and neither is followed by a
                    // finish. Closing on the finish alone leaves a barged
                    // read-back open at zero length — and a barged read-back is
                    // exactly the playback an utterance overlaps.
                    //
                    // A closer with no start ahead of it is a record whose head
                    // the fetch or the launcher's own wipe took: the playback
                    // happened, and all that is known of it is where it ended.
                    PLAYBACK_FINISHED | PLAYBACK_FLUSHED | PLAYBACK_ABORTED => {
                        match voice.playbacks.last_mut() {
                            Some(open) if !open.closed => {
                                open.end_ns = at_ns.max(open.start_ns);
                                open.closed = true;
                            }
                            _ => voice.playbacks.push(Played {
                                start_ns: at_ns,
                                end_ns: at_ns,
                                closed: true,
                            }),
                        }
                    }
                    _ => {}
                }
            }
        }
        // A playback the record never closed ran until the record stopped: the
        // session ended over the parrot, which is the ordinary end of a run the
        // operator interrupted with Ctrl-C.
        if let Some(open) = voice.playbacks.last_mut()
            && !open.closed
        {
            open.end_ns = last_ns.max(open.start_ns);
            voice.notes.push(
                "the speech record ends with a playback still running: the session was stopped \
                 over the read-back"
                    .to_owned(),
            );
        }
        voice
    }

    /// Whether `interval` overlaps any playback.
    fn over_playback(&self, interval: &Interval) -> bool {
        self.playbacks.iter().any(|played| {
            played.start_ns <= interval.t_end_ns && played.end_ns >= interval.t_start_ns
        })
    }
}

/// What one `utterance` line said, out of the fields it says it in.
fn spoken(seq: usize, value: &Value) -> Spoken {
    let timings = value.get("timings");
    let stamp = |key: &str| -> Option<i64> {
        timings
            .and_then(|held| held.get(key))
            .and_then(Value::as_i64)
    };
    let transcript = value.get("transcript");
    let confidence = transcript.and_then(|held| held.get("confidence"));
    let figure = |key: &str| -> Option<f64> {
        confidence
            .and_then(|held| held.get(key))
            .and_then(Value::as_f64)
    };
    let audio_ref = value.get("audio_ref");
    let whole = |key: &str| -> Option<i64> {
        audio_ref
            .and_then(|held| held.get(key))
            .and_then(Value::as_i64)
    };
    Spoken {
        seq,
        id: value.get("id").and_then(Value::as_u64),
        text: transcript
            .and_then(|held| held.get("text"))
            .and_then(Value::as_str)
            .unwrap_or_default()
            .to_owned(),
        no_speech: figure("no_speech_prob"),
        logprob: figure("avg_logprob"),
        compression: figure("compression_ratio"),
        endpoint_cause: value
            .get("endpoint_cause")
            .and_then(Value::as_str)
            .unwrap_or_default()
            .to_owned(),
        t0_projected: timings
            .and_then(|held| held.get("t0_projected"))
            .and_then(Value::as_bool)
            .unwrap_or_default(),
        onset_rx_us: stamp("onset_rx"),
        soft_endpoint_rx_us: stamp("soft_endpoint_rx"),
        first_audio_rx_us: stamp("first_audio_rx"),
        carve: audio_ref
            .and_then(|held| held.get("log"))
            .and_then(Value::as_str)
            .filter(|log| !log.is_empty())
            .zip(whole("start_sample").zip(whole("end_sample")))
            .map(|(log, (start_sample, end_sample))| Carve {
                log: log.to_owned(),
                start_sample,
                end_sample,
            }),
    }
}

/// A hole in the stream, or a wall clock that stepped under it.
#[derive(Clone, Copy, Debug, Serialize)]
struct GapEntry {
    /// The stamp of the sample the step arrived at.
    t_ns: i64,
    /// How much time the step covered, milliseconds.
    ///
    /// For a gap, the grid clock's own interval, always positive: longer than
    /// the period is the ordinary reading and is the length of stream that is
    /// missing, and shorter than it is a loop that stamped two readings inside
    /// one period, which is a grid that is not the grid it says it is. Both are
    /// a stretch whose figures were measured against a baseline that does not
    /// hold, which is why one field carries both. For a clock step, how far the
    /// wall clock moved relative to the grid, signed: forward is positive, which
    /// is the direction a unit that boots from RAM and then syncs its time
    /// takes.
    ms: f64,
}

/// One hole, with where in the stream it is.
///
/// A pair of samples can carry both facts at once — a unit that stalled while
/// its time sync landed moved its clock *and* missed a stretch — so the two
/// readings are separate fields rather than one kind. Each is `Some` when that
/// reading is off by more than [`SPACING_SLACK_MS`]; a pair with neither is not
/// a hole at all.
///
/// The index is of the *later* sample of the pair the hole was measured across,
/// and it stays inside this file: what the document prints is the entries, and
/// what the analyzer needs the index for is judging which segment a hole falls
/// inside. By position and not by stamp, for the reason `frames_of` gives — a
/// session with a wall step in it is a session whose stamps do not sort, and an
/// interval test over them fails on exactly the hole it exists to mark.
#[derive(Debug)]
struct Hole {
    /// The later sample of the pair, as a stream index.
    at: usize,
    /// The stamp that sample carries.
    t_ns: i64,
    /// How far the wall clock moved out from under the grid, milliseconds,
    /// signed. The readings either side are consecutive on the grid, but every
    /// span measured across the step — the segmenter's still floor, its
    /// difference baseline, the join against the voice host — is measured on
    /// the clock that moved.
    step_ms: Option<f64>,
    /// The grid clock's own interval, milliseconds, when it is off the period:
    /// the stream is missing a stretch, because the loop fell behind or the bus
    /// went quiet. Nothing says what the head did over it.
    gap_ms: Option<f64>,
}

impl Hole {
    /// The step entry the document prints, if the clock moved here.
    fn step(&self) -> Option<GapEntry> {
        self.step_ms.map(|ms| GapEntry {
            t_ns: self.t_ns,
            ms,
        })
    }

    /// The gap entry the document prints, if the stream is missing a stretch
    /// here.
    fn gap(&self) -> Option<GapEntry> {
        self.gap_ms.map(|ms| GapEntry {
            t_ns: self.t_ns,
            ms,
        })
    }

    /// What a refusal and a note call it: the stamp it landed at, and how much
    /// time each reading covers.
    ///
    /// One wording for both callers, so an operator reading the note and the
    /// refusal of the same event reads the same sentence about it.
    fn named(&self) -> String {
        let at = self.t_ns;
        // The two readings of an off-grid interval are different facts about
        // the recorder and get different words: longer than the period is
        // stream that is gone, shorter than it is a loop that stamped two
        // readings inside one period. Saying "gap" for both would tell an
        // operator a length of stream is missing when none is.
        let gap = self.gap_ms.map(|ms| {
            if ms < PERIOD_MS {
                (
                    format!("two readings {ms:.0} ms apart"),
                    format!(
                        "they were stamped inside one {PERIOD_MS:.0} ms period, so the grid is \
                         not the grid it says it is"
                    ),
                )
            } else {
                (
                    format!("a {ms:.0} ms gap"),
                    "the recording holds no reading of that stretch".to_owned(),
                )
            }
        });
        match (self.step_ms, gap) {
            (Some(step), Some((gap, why))) => format!(
                "a wall-clock step of {step:+.0} ms across {gap} at t_ns {at} — the stamps \
                 moved and {why}"
            ),
            (Some(step), None) => format!(
                "a wall-clock step of {step:+.0} ms at t_ns {at} — the stamps moved, the \
                 readings did not"
            ),
            (None, Some((gap, why))) => format!("{gap} at t_ns {at} — {why}"),
            // A hole is one reading or the other; `holes` keeps no pair that is
            // neither.
            (None, None) => format!("a stretch off the grid at t_ns {at}"),
        }
    }
}

/// Every hole in the stream, in the order they occurred.
///
/// Each consecutive pair of samples is judged on both clocks at once, and on
/// both questions: the grid clock is monotonic and paced, so an interval of its
/// own off the period is a stretch the stream is missing; the wall clock is the
/// join key, and where it disagrees with the grid it moved, whichever way it
/// went. Neither answer is taken as the other's absence — a loop that stalled
/// while a time sync landed did both at once, and a reader told only about the
/// step would never learn how much stream is gone. A reader with one clock
/// cannot tell them apart at all, and treats a forward step as a slow loop.
fn holes(samples: &[Sample]) -> Vec<Hole> {
    let mut found = Vec::new();
    for (at, pair) in samples.windows(2).enumerate() {
        let wall_ms = (pair[1].t_ns - pair[0].t_ns) as f64 * 1e-6;
        let grid_ms = (pair[1].mono_ns - pair[0].mono_ns) as f64 * 1e-6;
        let drift_ms = wall_ms - grid_ms;
        let hole = Hole {
            at: at + 1,
            t_ns: pair[1].t_ns,
            step_ms: (drift_ms.abs() > SPACING_SLACK_MS).then_some(drift_ms),
            gap_ms: ((grid_ms - PERIOD_MS).abs() > SPACING_SLACK_MS).then_some(grid_ms),
        };
        if hole.step_ms.is_some() || hole.gap_ms.is_some() {
            found.push(hole);
        }
    }
    found
}

/// The hole a segment spans, if it spans one.
///
/// A hole marks the segment that owns the *earlier* sample of the pair it was
/// measured across, so every hole marks exactly one segment. Against the
/// half-open sample range that is `from < at && at <= until`: a hole whose later
/// sample opens a segment marks the segment before it, because segments tile in
/// stamps and that segment's `t1_ns` is the post-hole stamp, so its `t1_ns` and
/// its duration span the hole. The segment it opens is not marked — the
/// recording covers every reading in it.
///
/// A segment that spans a hole is a stretch whose figures were measured across
/// time the recording does not cover, and no draft is cut from it.
fn hole_in(holes: &[Hole], from: usize, until: usize) -> Option<&Hole> {
    holes.iter().find(|hole| from < hole.at && hole.at <= until)
}

/// A head pose, as the document prints one.
///
/// Every figure is of the pose *relative to neutral*, which is the convention a
/// clip frame is already in: the translation is the delta in the base head's own
/// frame, and because neutral is a pure translation the rotation figures are the
/// head's own tilt and yaw as the envelope defines them.
#[derive(Debug, Serialize)]
struct PoseFigures {
    /// Delta translation, millimetres.
    dt_mm: [f64; 3],
    /// Delta rotation as a unit quaternion, `[w, x, y, z]`.
    dq: [f64; 4],
    /// Head height relative to neutral, millimetres.
    height_mm: f64,
    /// Angle between the head's vertical and the base vertical, degrees.
    cone_deg: f64,
    /// Head yaw relative to the body, degrees.
    relative_yaw_deg: f64,
}

impl PoseFigures {
    /// The figures of an absolute head pose in the body frame.
    fn of(pose: &Isometry3<f64>) -> Self {
        let delta = neutral_head_pose().inverse() * pose;
        let q = delta.rotation.quaternion();
        let m = delta.rotation.to_rotation_matrix();
        let m = m.matrix();
        Self {
            dt_mm: [
                delta.translation.vector.x * 1e3,
                delta.translation.vector.y * 1e3,
                delta.translation.vector.z * 1e3,
            ],
            dq: [q.w, q.i, q.j, q.k],
            height_mm: delta.translation.vector.z * 1e3,
            cone_deg: cone_angle(&delta.rotation).to_degrees(),
            relative_yaw_deg: m[(1, 0)].atan2(m[(0, 0)]).to_degrees(),
        }
    }

    /// Roll, pitch and yaw of the delta, degrees, for the timeline's one line.
    fn rpy_deg(&self) -> [f64; 3] {
        let rotation = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            self.dq[0], self.dq[1], self.dq[2], self.dq[3],
        ));
        let (roll, pitch, yaw) = rotation.euler_angles();
        [roll.to_degrees(), pitch.to_degrees(), yaw.to_degrees()]
    }
}

/// What kind of stretch a segment is, as the document spells it.
///
/// The segmenter's own enum, carried rather than flattened into a string the
/// rest of this file then compares against: a mislabelled segment is a mis-cut
/// clip, and a typo in one of four string comparisons would have compiled.
#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize)]
#[serde(rename_all = "lowercase")]
enum DocKind {
    /// A hold: where the machine stood, and how still it stood there.
    Still,
    /// A move: how far it went and how fast.
    Motion,
}

impl From<SegmentKind> for DocKind {
    fn from(kind: SegmentKind) -> Self {
        match kind {
            SegmentKind::Still => Self::Still,
            SegmentKind::Motion => Self::Motion,
        }
    }
}

/// One segment, as the document prints one.
///
/// One struct for both kinds, with the figures that mean nothing for a kind
/// omitted from the JSON: a still stretch is described by where it stood and a
/// move by how far it went, and a reader of either should not have to know
/// which keys to ignore.
#[derive(Debug, Serialize)]
struct SegmentDoc {
    id: String,
    kind: DocKind,
    t0_ns: i64,
    t1_ns: i64,
    duration_s: f64,
    #[serde(skip_serializing_if = "Option::is_none")]
    joints_mean_rad: Option<[f64; ROW_COUNT]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    joints_excursion_rad: Option<[f64; ROW_COUNT]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    joints_drift_rad: Option<[f64; ROW_COUNT]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    joints_path_rad: Option<[f64; ROW_COUNT]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    peak_joint_speed_rad_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    rms_joint_speed_rad_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pose: Option<PoseFigures>,
    #[serde(skip_serializing_if = "Option::is_none")]
    body_yaw_rad: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    antennas_rad: Option<[Option<f64>; 2]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    head_path_mm: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    peak_head_speed_mm_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    peak_head_angular_speed_deg_s: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    net_dt_mm: Option<[f64; 3]>,
    #[serde(skip_serializing_if = "Option::is_none")]
    net_rotation_deg: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    from: Option<String>,
    #[serde(skip_serializing_if = "Option::is_none")]
    to: Option<String>,
    /// Whether a hole in the stream falls inside this stretch — a gap, or a
    /// wall clock that stepped under it. A stretch that spans one was measured
    /// across time the recording does not cover, so its figures are of a
    /// stretch nobody watched and no draft is cut from it.
    spans_hole: bool,
    fk_valid: bool,
    fk_valid_frames: usize,
    frames: usize,
}

/// What the session was, over the whole stream.
#[derive(Debug, Serialize)]
struct SessionDoc {
    records: String,
    t0_ns: i64,
    duration_s: f64,
    samples: usize,
    late: usize,
    gaps: Vec<GapEntry>,
    clock_steps: Vec<GapEntry>,
    #[serde(skip_serializing_if = "Option::is_none")]
    recorder: Option<StartedLine>,
    #[serde(skip_serializing_if = "Option::is_none")]
    ended: Option<PoseLogEnd>,
    #[serde(skip_serializing_if = "Option::is_none")]
    ended_error: Option<String>,
    segmenter: SegmenterLine,
    /// The endpointer the speech intervals were derived under: an interval is
    /// a stamp walked back by these, so a reader of one wants them beside it.
    endpointer: Endpointer,
    notes: Vec<String>,
}

/// One utterance, as the document prints one.
#[derive(Debug, Serialize)]
struct UtteranceDoc {
    /// Which utterance of the record it is, counted from one, and what the
    /// `.wav` beside the document is named after.
    seq: usize,
    #[serde(skip_serializing_if = "Option::is_none")]
    id: Option<u64>,
    text: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    no_speech: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    logprob: Option<f64>,
    #[serde(skip_serializing_if = "Option::is_none")]
    compression: Option<f64>,
    /// The speech interval, `null` on a line carrying no stamp to derive one
    /// from — which is also an utterance nothing is joined to.
    t_start_ns: Option<i64>,
    t_end_ns: Option<i64>,
    start_estimated: bool,
    endpoint_cause: String,
    t0_projected: bool,
    /// The cut beside the document, `null` where none was written.
    wav: Option<String>,
    /// Why no cut was written, where none was.
    #[serde(skip_serializing_if = "Option::is_none")]
    wav_error: Option<String>,
    /// The hold this was spoken out of or into.
    still_before: Option<String>,
    /// Every segment the speech overlapped.
    during: Vec<String>,
    /// The hold the machine settled into after it.
    still_after: Option<String>,
    /// Whether the pod was playing something back over it: the parrot's
    /// read-back of an earlier utterance, which makes this one a correction or
    /// an interruption rather than a fresh label.
    overlaps_playback: bool,
}

/// The whole document.
#[derive(Debug, Serialize)]
struct Document {
    session: SessionDoc,
    segments: Vec<SegmentDoc>,
    utterances: Vec<UtteranceDoc>,
}

/// The per-sample kinematic solve of a whole stream.
struct Solved {
    /// The head pose per sample, `None` where no seed closed the linkage.
    poses: Vec<Option<Isometry3<f64>>>,
}

impl Solved {
    /// Solve every sample, each seeded with the previous answer.
    ///
    /// A sample after a failure — and the first — tries the resting seeds in
    /// turn. A sample with a silent crank is not solved at all: the slot holds
    /// zero, zero is a legal crank angle, and six legal angles generically
    /// close the linkage somewhere, so the solve would answer with the pose
    /// *some other* head position holds and the document would print it as a
    /// reading. The cranks are kept and the pose is absent, which is the same
    /// answer a linkage that does not close gives.
    ///
    /// A skipped sample leaves the seed where it was rather than resetting it:
    /// nothing failed, so the next sample's own neighbour is still the best
    /// starting point and the assembly mode it carries is still the right one.
    fn of(samples: &[Sample]) -> Self {
        let geom = default_geometry();
        let opts = FkOptions::default();
        let rest = rest_pose_seeds();
        let mut poses = Vec::with_capacity(samples.len());
        let mut previous: Option<Isometry3<f64>> = None;
        for sample in samples {
            if !legs_answered(&sample.missing) {
                poses.push(None);
                continue;
            }
            let solved = solve_at(&sample.present, previous.as_ref(), &rest, geom, &opts);
            previous = solved;
            poses.push(solved);
        }
        Self { poses }
    }
}

/// Whether every crank the head hangs from answered in this reading.
///
/// The forward solve takes the six leg angles and nothing else, so this is the
/// question of whether there is a solve to attempt at all. Body yaw and the
/// antennas are not inputs to it.
fn legs_answered(missing: &[bool; ROW_COUNT]) -> bool {
    !missing
        .iter()
        .enumerate()
        .any(|(row, absent)| *absent && group_of_row(row) == Some(JointGroup::Legs))
}

/// How many readings each row answered over `samples`.
///
/// The mean, the excursion and the drift a segment carries are over the
/// readings that answered, so a row that answered none of them has no figure
/// rather than a figure of zero — and a leg among them has no pose.
fn answered(samples: &[Sample]) -> [usize; ROW_COUNT] {
    let mut counts = [0usize; ROW_COUNT];
    for sample in samples {
        for (row, count) in counts.iter_mut().enumerate() {
            if !sample.missing[row] {
                *count += 1;
            }
        }
    }
    counts
}

/// Solve one reading's legs, `seed` first and the resting seeds behind it.
fn solve_at(
    present: &JointVector,
    seed: Option<&Isometry3<f64>>,
    rest: &[Isometry3<f64>; 3],
    geom: &reachy_kin::HeadGeometry,
    opts: &FkOptions,
) -> Option<Isometry3<f64>> {
    let mut seeds = Vec::with_capacity(4);
    seeds.extend(seed.copied());
    seeds.extend_from_slice(rest);
    ArmRecord::solve(geom, opts, present, &seeds)
        .ok()
        .map(|record| record.head_pose_body)
}

/// Where each segment's samples are in the stream, as half-open index ranges
/// that tile it.
///
/// By position and not by stamp. The segmenter takes the samples in file order
/// and says how many fell in each stretch, so walking the two together is
/// exact; bisecting the stamps instead assumes they sort, and a session with a
/// wall-clock step in it is a session whose stamps do not. What such a bisection
/// answers is an arbitrary index range, and every figure over it — the frame
/// count, how many solved, the head path — is then a plausible wrong number.
///
/// A reading no row answered is not a sample to the segmenter, so it is counted
/// against nothing and falls in whichever stretch it sits inside.
fn frames_of(samples: &[Sample], segments: &[Segment]) -> Vec<(usize, usize)> {
    let mut ranges: Vec<(usize, usize)> = Vec::with_capacity(segments.len());
    let mut at = 0usize;
    for segment in segments {
        let from = at;
        let mut counted = 0usize;
        while at < samples.len() && counted < segment.samples {
            if samples[at].missing.iter().any(|absent| !*absent) {
                counted += 1;
            }
            at += 1;
        }
        ranges.push((from, at));
    }
    // Anything after the last counted reading belongs to the stretch it fell
    // inside, which is the last one.
    if let Some(last) = ranges.last_mut() {
        last.1 = samples.len();
    }
    ranges
}

/// The head figures over one stretch of solved samples.
struct HeadFigures {
    /// How far the head origin travelled, millimetres.
    path_mm: f64,
    /// Fastest head speed over the difference baseline, mm/s.
    peak_speed_mm_s: f64,
    /// Fastest head turn over the same baseline, deg/s.
    peak_angular_deg_s: f64,
    /// First solved pose to last, millimetres.
    net_dt_mm: [f64; 3],
    /// Angle between the first and last solved orientation, degrees.
    net_rotation_deg: f64,
    /// How many of the stretch's samples solved.
    valid: usize,
}

/// The head figures of `samples[from..until]`, over the same difference
/// baseline the segmenter measures joint activity across.
///
/// The path is summed over consecutive solved samples and the speeds are taken
/// across the baseline, so a stretch with an unsolved sample in the middle of it
/// contributes the pairs it has rather than a straight line across the hole.
fn head_figures(
    samples: &[Sample],
    poses: &[Option<Isometry3<f64>>],
    from: usize,
    until: usize,
    window: std::time::Duration,
) -> HeadFigures {
    let window_ns = i64::try_from(window.as_nanos()).unwrap_or(i64::MAX);
    let mut figures = HeadFigures {
        path_mm: 0.0,
        peak_speed_mm_s: 0.0,
        peak_angular_deg_s: 0.0,
        net_dt_mm: [0.0; 3],
        net_rotation_deg: 0.0,
        valid: 0,
    };
    let mut first: Option<Isometry3<f64>> = None;
    let mut last: Option<Isometry3<f64>> = None;
    // The baseline pointer, carried forward rather than searched for: both the
    // sample it is chosen from and the cutoff it is chosen against advance with
    // `index`, so the newest solved sample at or before the cutoff never moves
    // backwards. Searching instead cost a scan to the start of the stretch on
    // every sample whose window held nothing that solved, which is exactly the
    // hand-moved head this tool is for — the sessions hardest to read would
    // have been the slowest to read.
    let mut base: Option<usize> = None;
    let mut ahead = from;
    for index in from..until {
        let Some(pose) = poses[index] else { continue };
        figures.valid += 1;
        if first.is_none() {
            first = Some(pose);
        }
        if let Some(before) = last {
            figures.path_mm += (pose.translation.vector - before.translation.vector).norm() * 1e3;
        }
        last = Some(pose);
        // A stretch whose window holds no solved sample has no measured speed
        // at this index rather than a speed over a shorter span.
        let cutoff = samples[index].t_ns - window_ns;
        while ahead < index && samples[ahead].t_ns <= cutoff {
            if poses[ahead].is_some() {
                base = Some(ahead);
            }
            ahead += 1;
        }
        if let Some(base) = base {
            let earlier = poses[base].expect("the baseline index solved");
            let dt_s = (samples[index].t_ns - samples[base].t_ns) as f64 * 1e-9;
            if dt_s > 0.0 {
                let travel = (pose.translation.vector - earlier.translation.vector).norm() * 1e3;
                figures.peak_speed_mm_s = figures.peak_speed_mm_s.max(travel / dt_s);
                let turn = earlier.rotation.angle_to(&pose.rotation).to_degrees();
                figures.peak_angular_deg_s = figures.peak_angular_deg_s.max(turn / dt_s);
            }
        }
    }
    if let (Some(first), Some(last)) = (first, last) {
        let net = last.translation.vector - first.translation.vector;
        figures.net_dt_mm = [net.x * 1e3, net.y * 1e3, net.z * 1e3];
        figures.net_rotation_deg = first.rotation.angle_to(&last.rotation).to_degrees();
    }
    figures
}

/// The whole reading of one fetched session.
struct Analysis {
    /// The document, as it will be written.
    document: Document,
    /// The verdict and the numbers, for the operator's terminal.
    report: Report,
    /// What `--extract` produced, where it was asked for: a draft to write, or
    /// why that segment is not one. Beside the verdict rather than in it — the
    /// verdict is about whether the session is a recording session, and a
    /// segment an operator cannot extract is about this invocation.
    extraction: Option<Result<Extraction, String>>,
    /// The audio the document points at, decoded and waiting to be written.
    cuts: Vec<Cut>,
}

/// Read one session and say what it holds.
///
/// Reading only: every path this takes is a path it reads, and everything it
/// produces is handed back for the caller to write. A session whose document
/// cannot be written leaves nothing of itself on disk.
fn analyze(
    records: &Path,
    stream: &Stream,
    voice: &Voice,
    cfg: SegmentConfig,
    ask: Option<&Extract>,
) -> Analysis {
    let mut report = Report::default();
    let mut notes: Vec<String> = Vec::new();
    if let Some(why) = &stream.unreadable {
        report.fail(format!("the pose stream could not be read: {why}"));
    } else if stream.started.is_none() && stream.refused.is_none() && stream.samples.is_empty() {
        report.fail(format!(
            "no pose stream: {} holds no {RECORDER_LOG}",
            console_dir(records).display()
        ));
    }
    // A refusal is red for what it is — a run that recorded nothing — and not
    // for the word: a stream carrying samples and a refusal is a recording that
    // ended on something, which the note below names.
    match (&stream.refused, stream.samples.is_empty()) {
        (Some(error), true) => report.fail(format!("the recorder refused to run: {error}")),
        (Some(error), false) => notes.push(format!(
            "the stream carries samples and a `refused` line: the run ended on {error}"
        )),
        (None, _) => {}
    }
    if stream.samples.is_empty() {
        report.fail("the pose stream holds no samples");
    }
    if stream.ended.is_none() && !stream.samples.is_empty() {
        notes.push(
            "the stream has no `ended` line: the recorder was killed or the fetch truncated it"
                .to_owned(),
        );
    }
    if stream.ended == Some(PoseLogEnd::Aborted) {
        notes.push(format!(
            "the run ended on a failure rather than on the operator or the cap: {}. The samples \
             ahead of it are a recording",
            stream
                .ended_error
                .as_deref()
                .unwrap_or("the closing line does not say what")
        ));
    }
    if let Some(started) = &stream.started
        && started.period_ms != POSE_LOG_PERIOD_MS
    {
        notes.push(format!(
            "the stream says it sampled on a {} ms grid, and this reader judges spacing against \
             {POSE_LOG_PERIOD_MS} ms: it was written by another recorder",
            started.period_ms
        ));
    }
    if stream.torn > 0 {
        let said = format!(
            "{} line(s) of the stream opened like a record and would not parse: torn or \
             truncated, or written by a build whose line schema is not this one",
            stream.torn
        );
        // A stream with samples in it lost some lines; a stream with none and
        // nothing but unparseable records in it is a payload whose recorder and
        // reader disagree about the schema — a whole pre-append stream reads
        // exactly this way — and an operator who gets told "no samples" alone
        // goes looking at the servos.
        if stream.samples.is_empty() {
            report.fail(said);
        } else {
            notes.push(said);
        }
    }

    // The holes, before anything is measured over the stream: a figure taken
    // across one is a figure whose baseline is not what it says it is.
    let holes = holes(&stream.samples);
    let stepped: Vec<&Hole> = holes.iter().filter(|hole| hole.step_ms.is_some()).collect();
    if !stepped.is_empty() {
        // Loud, in either direction: the cut itself is not defined across one.
        // The segmenter's floor and its difference baseline are spans of wall
        // time, and wall time moved. The segments either side of the step are
        // as good as any others; the ones spanning it are not, and they are
        // marked. Each is named the way the refusal names it, so an operator
        // reading both reads one sentence about one event.
        let named: Vec<String> = stepped.iter().map(|hole| hole.named()).collect();
        notes.push(format!(
            "the wall clock stepped {} time(s) under this session ({}): the segmenter's \
             thresholds are spans of that clock, so a stretch spanning a step is cut on \
             arithmetic that does not hold there",
            stepped.len(),
            named.join("; ")
        ));
    }
    // A pair carrying both facts is in both lists: the step is what the join
    // key did, the gap is how much stream is missing, and an operator deciding
    // whether a session is usable needs each of them.
    let gaps: Vec<GapEntry> = holes.iter().filter_map(Hole::gap).collect();
    let clock_steps: Vec<GapEntry> = holes.iter().filter_map(Hole::step).collect();

    let solved = Solved::of(&stream.samples);
    let mut segments: Vec<Segment> = Vec::new();
    // A configuration the grid cannot be measured on refuses rather than
    // answering that nothing ever moved: the flags are what an operator tunes
    // thresholds with, and a widened window that quietly swallowed a whole
    // session would read as a machine nobody touched.
    match MotionSegmenter::new(cfg, POSE_LOG_PERIOD) {
        Ok(mut segmenter) => {
            for sample in &stream.samples {
                // The wall stamp, not the grid one: a segment's bounds are the
                // join key against the voice host's stamps, and both streams
                // carry the same clock. What that costs is a stretch whose
                // thresholds were measured across a step, and those stretches
                // are named and refused as drafts rather than re-paced.
                segmenter.look(
                    &JointSample {
                        t_ns: sample.t_ns,
                        present: &sample.present,
                        missing: flags::from_rows(&sample.missing),
                    },
                    &mut segments,
                );
            }
            segmenter.finish(&mut segments);
        }
        Err(error) => report.fail(format!("the segmentation cannot run: {error}")),
    }

    // One derivation of which samples fall in which segment, for the document
    // and for a draft cut out of it: the rule that places a reading is
    // non-obvious, and two callers deriving it separately are two answers the
    // day either one is handed a filtered stream.
    let frames = frames_of(&stream.samples, &segments);
    let documented = document_segments(&stream.samples, &solved, &segments, &frames, &holes, cfg);
    let cut = Segmented {
        samples: &stream.samples,
        solved: &solved,
        segments: &segments,
        frames: &frames,
        holes: &holes,
    };
    let extraction = ask.map(|ask| extract(records, &cut, &documented, ask));
    notes.extend(voice.notes.iter().cloned());
    // The speech half's own verdict: a recording session runs the pipeline with
    // no wake word, and a host that never announced itself was never
    // transcribing. Poses with nothing said over them are a session nobody can
    // label, which is what this tool exists to prevent shipping as one.
    if !voice.listening {
        report.fail(format!(
            "the voice host never reported `listening`: {} carries no such line, so nothing was \
             transcribing over this session",
            console_dir(records).join(HOST_LOG).display()
        ));
    }
    let (said, cuts) = document_utterances(voice, &documented, records, &mut notes);
    let t0_ns = stream.samples.first().map_or(0, |sample| sample.t_ns);
    let duration_s = stream
        .samples
        .last()
        .map_or(0.0, |sample| (sample.t_ns - t0_ns) as f64 * 1e-9);

    report.note(format!(
        "{} samples over {duration_s:.1} s, {} still and {} moving segment(s)",
        stream.samples.len(),
        documented
            .iter()
            .filter(|doc| doc.kind == DocKind::Still)
            .count(),
        documented
            .iter()
            .filter(|doc| doc.kind == DocKind::Motion)
            .count(),
    ));
    report.note(format!(
        "{} gap(s), {} clock step(s), {} late note(s)",
        gaps.len(),
        clock_steps.len(),
        stream.late
    ));
    let unsolved: usize = documented
        .iter()
        .map(|doc| doc.frames - doc.fk_valid_frames)
        .sum();
    report.note(format!("{unsolved} sample(s) with no head pose"));
    report.note(format!(
        "{} utterance(s), {} joined to a hold, {} cut to a .wav, {} over a read-back",
        said.len(),
        said.iter().filter(|doc| doc.still_before.is_some()).count(),
        said.iter().filter(|doc| doc.wav.is_some()).count(),
        said.iter().filter(|doc| doc.overlaps_playback).count(),
    ));

    Analysis {
        document: Document {
            session: SessionDoc {
                records: records.display().to_string(),
                t0_ns,
                duration_s,
                samples: stream.samples.len(),
                late: stream.late,
                gaps,
                clock_steps,
                recorder: stream.started.clone(),
                ended: stream.ended,
                ended_error: stream.ended_error.clone(),
                segmenter: SegmenterLine::of(cfg),
                endpointer: voice.endpointer,
                notes,
            },
            segments: documented,
            utterances: said,
        },
        report,
        extraction,
        cuts,
    }
}

/// Where the head stood over one hold, solved from the mean of the hold.
///
/// The mean of a still stretch is a better estimate of where the head stood
/// than any one reading of it, and the seed is the stretch's own first solved
/// sample so the answer comes back in the assembly mode the linkage was
/// actually in. A move has no such pose — its net displacement says where it
/// went instead — so this is only ever asked of a hold.
///
/// One function because both the document's figures and `--extract`'s single
/// frame are that pose, and a clip drafted off a different estimate than the
/// document printed would be a clip nobody could check against the session.
fn mean_pose(
    solved: &Solved,
    segment: &Segment,
    from: usize,
    until: usize,
    geom: &reachy_kin::HeadGeometry,
    opts: &FkOptions,
    rest: &[Isometry3<f64>; 3],
) -> Option<Isometry3<f64>> {
    let seed = (from..until).find_map(|at| solved.poses[at]);
    solve_at(&segment.mean, seed.as_ref(), rest, geom, opts)
}

/// Every segment, with the figures its kind is read by.
fn document_segments(
    samples: &[Sample],
    solved: &Solved,
    segments: &[Segment],
    frames: &[(usize, usize)],
    holes: &[Hole],
    cfg: SegmentConfig,
) -> Vec<SegmentDoc> {
    let geom = default_geometry();
    let opts = FkOptions::default();
    let rest = rest_pose_seeds();
    let mut out: Vec<SegmentDoc> = Vec::with_capacity(segments.len());
    for (index, (segment, &(from, until))) in segments.iter().zip(frames).enumerate() {
        let head = head_figures(samples, &solved.poses, from, until, cfg.speed_window);
        let frames = until - from;
        let counts = answered(&samples[from..until]);
        let kind = DocKind::from(segment.kind);
        let still = kind == DocKind::Still;
        // A row that answered nothing over the whole stretch has no mean, so a
        // leg among them has no pose to solve and the reported rows carry a
        // null rather than the zero their slot holds.
        let cranks_read = (0..ROW_COUNT)
            .all(|row| counts[row] > 0 || group_of_row(row) != Some(JointGroup::Legs));
        let pose = (still && cranks_read)
            .then(|| mean_pose(solved, segment, from, until, geom, &opts, &rest))
            .flatten();
        out.push(SegmentDoc {
            id: format!("S{:03}", index + 1),
            kind,
            t0_ns: segment.t0_ns,
            t1_ns: segment.t1_ns,
            duration_s: (segment.t1_ns - segment.t0_ns) as f64 * 1e-9,
            joints_mean_rad: still.then(|| segment.mean.rows()),
            joints_excursion_rad: still.then(|| segment.excursion.rows()),
            joints_drift_rad: still.then(|| segment.drift.rows()),
            joints_path_rad: (!still).then(|| segment.path.rows()),
            peak_joint_speed_rad_s: (!still).then_some(segment.peak_speed),
            rms_joint_speed_rad_s: (!still).then_some(segment.rms_speed),
            pose: still.then(|| pose.as_ref().map(PoseFigures::of)).flatten(),
            body_yaw_rad: still
                .then(|| row(JointRef::BodyYaw).filter(|row| counts[*row] > 0))
                .flatten()
                .map(|_| segment.mean.body_yaw),
            antennas_rad: still.then(|| {
                [JointRef::AntennaRight, JointRef::AntennaLeft].map(|joint| {
                    row(joint)
                        .filter(|row| counts[*row] > 0)
                        .and_then(|_| segment.mean.get(joint))
                })
            }),
            head_path_mm: (!still).then_some(head.path_mm),
            peak_head_speed_mm_s: (!still).then_some(head.peak_speed_mm_s),
            peak_head_angular_speed_deg_s: (!still).then_some(head.peak_angular_deg_s),
            net_dt_mm: (!still).then_some(head.net_dt_mm),
            net_rotation_deg: (!still).then_some(head.net_rotation_deg),
            from: None,
            to: None,
            spans_hole: hole_in(holes, from, until).is_some(),
            fk_valid: if still {
                pose.is_some()
            } else {
                head.valid > 0
            },
            fk_valid_frames: head.valid,
            frames,
        });
    }
    // The neighbours, once every id exists: a move is read from the pose it
    // left to the pose it reached.
    let ids: Vec<String> = out.iter().map(|doc| doc.id.clone()).collect();
    for (index, doc) in out.iter_mut().enumerate() {
        if doc.kind == DocKind::Motion {
            doc.from = index.checked_sub(1).and_then(|at| ids.get(at)).cloned();
            doc.to = ids.get(index + 1).cloned();
        }
    }
    out
}

/// What `--extract` asked for.
struct Extract {
    /// Which segment, as `session.json` prints the id.
    segment: String,
    /// The library name to file the draft under, where the operator named one.
    name: Option<String>,
    /// The channels the draft drives.
    channels: ChannelMask,
}

/// A clip draft, and what to call the file it goes in.
#[derive(Debug)]
struct Extraction {
    /// The file name, beside the session document.
    file: String,
    /// The draft itself.
    doc: ClipDoc,
}

/// The default library name for a segment of a fetched session.
///
/// The fetch directory's own name is the session's stamp, so a draft says which
/// session and which stretch of it it came off without an operator naming
/// either. Lowercased, because a fetch is stamped `record-20260910T120000Z` and
/// an asset name is `[a-z0-9_./-]`; anything else the directory's name carries
/// is refused by the name check rather than rewritten, and `--name` is the way
/// past it.
fn drafted_name(records: &Path, id: &str) -> String {
    let stamp = records.file_name().map_or_else(
        || "session".to_owned(),
        |name| name.to_string_lossy().into(),
    );
    format!("recorded/{stamp}/{id}").to_lowercase()
}

/// The stream as the analyzer has cut it: the readings, what each one solved
/// to, the stretches, where each stretch's readings are, and where the holes
/// are.
///
/// One bundle because every one of these is derived from the others and a pass
/// over the session needs all of them at once; handing them round separately is
/// five chances to pair a stretch with the ranges of a different cut.
struct Segmented<'a> {
    samples: &'a [Sample],
    solved: &'a Solved,
    segments: &'a [Segment],
    frames: &'a [(usize, usize)],
    holes: &'a [Hole],
}

/// The first frame of a draft that is not one grid step after the one before
/// it, as (which frame, how long after its predecessor in milliseconds).
///
/// A draft is one frame per reading played at one fixed rate, so the readings it
/// is built from have to be one grid step apart — measured on `mono_ns`, the
/// clock they were taken on, because the wall clock is the join key and can
/// step out from under a stretch whose readings are perfectly paced. This is the
/// check that catches the hole the entry lists do not see: a reading no row
/// answered, which is dropped from the frames and leaves a period of nothing
/// between two the draft would otherwise cross in a single tick, at a speed
/// nobody commanded. Refused rather than interpolated: what the head did over
/// that stretch is not in the recording, and inventing it is the one thing a
/// draft taken off a hand must not do.
fn off_grid_frame(samples: &[Sample], kept: &[usize]) -> Option<(usize, f64)> {
    kept.windows(2).enumerate().find_map(|(frame, pair)| {
        let step_ms = (samples[pair[1]].mono_ns - samples[pair[0]].mono_ns) as f64 * 1e-6;
        ((step_ms - PERIOD_MS).abs() > SPACING_SLACK_MS).then_some((frame + 1, step_ms))
    })
}

/// The clip draft one segment of the session extracts to.
///
/// A move extracts as one frame per reading, in the order they were taken; a
/// hold extracts as one frame at the mean of the hold, which is the same
/// document in the same shape because a pose is a clip a blend reaches. The
/// conversion itself — every channel as a delta over the neutral base — is
/// `reachy-clips`, so a draft and the library's own understanding of a clip
/// cannot drift apart.
///
/// A reading no row answered is not a frame: the segmenter did not count it as
/// a sample either, and a frame the recording holds nothing of is one this
/// would have to invent.
fn extract(
    records: &Path,
    cut: &Segmented<'_>,
    documented: &[SegmentDoc],
    ask: &Extract,
) -> Result<Extraction, String> {
    let (samples, solved, segments, frames, holes) =
        (cut.samples, cut.solved, cut.segments, cut.frames, cut.holes);
    let index = documented
        .iter()
        .position(|doc| doc.id == ask.segment)
        .ok_or_else(|| {
            format!(
                "{} is not a segment of this session, which holds {} of them, S001 to S{:03}",
                ask.segment,
                documented.len(),
                documented.len()
            )
        })?;
    let (from, until) = frames[index];
    // A hole first, and for either kind of segment. A draft is built only from
    // a stretch the recording covers on its own grid: what the head did across
    // a hole is not in the recording, and a hold whose mean was taken across
    // one can be a pose nobody held — a 0.2 s pause that spans a 30 s hole
    // reads as a 30 s hold, because the still floor is a span of stamp time.
    if let Some(hole) = hole_in(holes, from, until) {
        // No remediation: the operator has no tool or threshold that avoids
        // a hole a segment already spans.
        return Err(format!(
            "{} cannot be extracted: it spans {}",
            ask.segment,
            hole.named()
        ));
    }
    let segment = &segments[index];
    let read = |at: usize, joint: JointRef| -> Option<f64> {
        row(joint)
            .filter(|row| !samples[at].missing[*row])
            .and_then(|_| samples[at].present.get(joint))
    };
    let frames: Vec<RecordedFrame> = if documented[index].kind == DocKind::Still {
        let counts = answered(&samples[from..until]);
        let held = |joint: JointRef| -> Option<f64> {
            row(joint)
                .filter(|row| counts[*row] > 0)
                .and_then(|_| segment.mean.get(joint))
        };
        vec![RecordedFrame {
            head: mean_pose(
                solved,
                segment,
                from,
                until,
                default_geometry(),
                &FkOptions::default(),
                &rest_pose_seeds(),
            ),
            antennas: [JointRef::AntennaRight, JointRef::AntennaLeft].map(held),
            body_yaw: held(JointRef::BodyYaw),
        }]
    } else {
        let kept: Vec<usize> = (from..until)
            .filter(|at| samples[*at].missing.iter().any(|absent| !*absent))
            .collect();
        if let Some((frame, step_ms)) = off_grid_frame(samples, &kept) {
            // No remediation: no threshold ends a move at an unanswered
            // reading.
            return Err(format!(
                "{} cannot be extracted: frame {frame} is {step_ms:.0} ms after frame {}, not \
                 the {PERIOD_MS:.0} ms the draft's frames are played at. The readings it would \
                 be built from are not the grid the draft plays.",
                ask.segment,
                frame - 1,
            ));
        }
        kept.into_iter()
            .map(|at| RecordedFrame {
                head: solved.poses[at],
                antennas: [JointRef::AntennaRight, JointRef::AntennaLeft]
                    .map(|joint| read(at, joint)),
                body_yaw: read(at, JointRef::BodyYaw),
            })
            .collect()
    };
    let name = ask
        .name
        .clone()
        .unwrap_or_else(|| drafted_name(records, &ask.segment));
    let draft = Draft {
        name: &name,
        description: Some(format!(
            "{} of {}, extracted from the recorded pose stream",
            ask.segment,
            records.display()
        )),
        mask: ask.channels,
    };
    let doc = clip_doc(&draft, &frames).map_err(|error| match error {
        RecordError::Name { .. } => {
            format!("{error}. Name the draft yourself with --name")
        }
        // The frame index is the stretch's own, which is what an operator cuts
        // by: the fix is a different segment, a tighter one under other
        // thresholds, or a mask without the channel that went unread.
        other => format!("{} cannot be extracted: {other}", ask.segment),
    })?;
    Ok(Extraction {
        file: format!("clip-{}.json", ask.segment),
        doc,
    })
}

/// How far into an utterance a hold may still begin and be the hold it was
/// spoken about, nanoseconds.
///
/// An operator labelling a pose often starts talking while the head is still
/// settling under their hands, so a hold that opens just after they began is
/// the one they meant.
const HOLD_LEAD_NS: i64 = 500_000_000;

/// How far from an utterance a hold may sit and still be joined to it,
/// nanoseconds.
///
/// Either side: a hold that ended more than this before the speech began is not
/// what the speech is about, and neither is one that opens more than this after
/// it ended. Both are constants the first real session is expected to move.
const JOIN_WINDOW_NS: i64 = 3_000_000_000;

/// The three references one utterance is joined to.
struct Joined {
    still_before: Option<String>,
    during: Vec<String>,
    still_after: Option<String>,
}

/// Join one speech interval to the segments around it.
///
/// `still_before` is the latest hold that had opened by [`HOLD_LEAD_NS`] into
/// the speech and had not ended more than [`JOIN_WINDOW_NS`] before it — the
/// hold the operator was in, or the one they were settling into as they spoke.
/// `during` is everything the speech overlapped, hold or move. `still_after` is
/// the first hold to open within [`JOIN_WINDOW_NS`] of the speech ending, which
/// is the pose an instruction was followed by rather than the pose it described.
///
/// Three references and not one verdict, because which of them the operator
/// meant is in their own words and not in the arithmetic.
fn join(segments: &[SegmentDoc], interval: &Interval) -> Joined {
    let holds = || segments.iter().filter(|doc| doc.kind == DocKind::Still);
    Joined {
        still_before: holds()
            .rev()
            .find(|doc| {
                doc.t0_ns <= interval.t_start_ns + HOLD_LEAD_NS
                    && doc.t1_ns > interval.t_start_ns - JOIN_WINDOW_NS
            })
            .map(|doc| doc.id.clone()),
        during: segments
            .iter()
            .filter(|doc| doc.t0_ns <= interval.t_end_ns && doc.t1_ns >= interval.t_start_ns)
            .map(|doc| doc.id.clone())
            .collect(),
        still_after: holds()
            .find(|doc| {
                doc.t0_ns >= interval.t_end_ns && doc.t0_ns <= interval.t_end_ns + JOIN_WINDOW_NS
            })
            .map(|doc| doc.id.clone()),
    }
}

/// Every utterance, joined to the segments and decoded out of the fetched audio.
///
/// The audio is resolved here and written by the caller: whether a cut *could*
/// be made is a field of the document — an utterance whose audio was pruned, or
/// whose fetch brought no store home, says so where the transcript is, and an
/// operator reading a hallucinated transcript wants to know in the same breath
/// whether they can listen to it — but making the file is not a reading of the
/// session, and a document that failed to write must not leave wavs nothing
/// names behind it.
fn document_utterances(
    voice: &Voice,
    segments: &[SegmentDoc],
    records: &Path,
    notes: &mut Vec<String>,
) -> (Vec<UtteranceDoc>, Vec<Cut>) {
    let store = audio_dir(records);
    // Once, not once per utterance: what the store holds is one sentence about
    // the run and not one per thing that was said. Three answers and not two —
    // a store that will not open is not a session where nobody spoke.
    let recorded = run_audio::recorded(&store);
    match &recorded {
        Ok(true) => {}
        Ok(false) if voice.said.is_empty() => {}
        Ok(false) => notes.push(format!(
            "{} holds no recorded audio, so nothing was cut: the session's speech configuration \
             is what turns recording on",
            store.display()
        )),
        Err(why) => notes.push(format!(
            "the recorded-audio store would not open ({why}), so nothing was cut: this is the \
             store failing and not a session nobody spoke over"
        )),
    }
    let mut cuts: Vec<Cut> = Vec::new();
    let mut out_docs = Vec::with_capacity(voice.said.len());
    for said in &voice.said {
        let interval = said.interval(&voice.endpointer);
        if interval.is_none() {
            notes.push(format!(
                "utterance #{} carries no stamp to derive an interval from, so nothing is joined \
                 to it",
                said.seq
            ));
        }
        let joined = interval.as_ref().map(|interval| join(segments, interval));
        let (wav, wav_error) = decode(said, &store, &recorded, &mut cuts);
        out_docs.push(UtteranceDoc {
            seq: said.seq,
            id: said.id,
            text: said.text.clone(),
            no_speech: said.no_speech,
            logprob: said.logprob,
            compression: said.compression,
            t_start_ns: interval.as_ref().map(|held| held.t_start_ns),
            t_end_ns: interval.as_ref().map(|held| held.t_end_ns),
            start_estimated: interval.as_ref().is_some_and(|held| held.start_estimated),
            endpoint_cause: said.endpoint_cause.clone(),
            t0_projected: said.t0_projected,
            wav,
            wav_error,
            still_before: joined.as_ref().and_then(|held| held.still_before.clone()),
            during: joined
                .as_ref()
                .map(|held| held.during.clone())
                .unwrap_or_default(),
            still_after: joined.as_ref().and_then(|held| held.still_after.clone()),
            overlaps_playback: interval
                .as_ref()
                .is_some_and(|held| voice.over_playback(held)),
        });
    }
    (out_docs, cuts)
}

/// One utterance's audio out of the fetched store, ready for the caller to
/// write.
///
/// Each failure mode — no store, a faulted store, a pruned log, an invalid
/// span — is said in its own words: an outage must never read as a session
/// where nobody spoke.
fn decode(
    said: &Spoken,
    store: &Path,
    recorded: &Result<bool, String>,
    cuts: &mut Vec<Cut>,
) -> (Option<String>, Option<String>) {
    let Some(carve) = &said.carve else {
        return (
            None,
            Some("the utterance line names no audio to cut".to_owned()),
        );
    };
    match recorded {
        Ok(false) => return (None, Some("this fetch brought no audio home".to_owned())),
        Err(why) => return (None, Some(why.clone())),
        Ok(true) => {}
    }
    match run_audio::resolve(store, &carve.log, carve.start_sample, carve.end_sample) {
        Ok(audio) => {
            let name = format!("utt-{:03}.wav", said.seq);
            cuts.push(Cut {
                seq: said.seq,
                name: name.clone(),
                pcm: audio.pcm,
            });
            (Some(name), None)
        }
        Err(why) => (None, Some(why)),
    }
}

/// One utterance's decoded audio and what the document calls its file.
///
/// Held rather than written where it was decoded: reading a session and writing
/// its artefacts are two jobs, and the second is the caller's, in one place,
/// after the first has succeeded.
struct Cut {
    /// Which utterance of the record it is, so a write that failed is said
    /// where the transcript is and not only on the terminal.
    seq: usize,
    /// The file name the document points at.
    name: String,
    /// The samples, as the store resolved them.
    pcm: Vec<i16>,
}

/// The chronological account, one line per segment.
///
/// For a person, and for an LLM that will not want the JSON first. Times are
/// `mm:ss.cc` from the first sample of the session, which is how an operator
/// who was there reads their own session back.
fn timeline(document: &Document) -> String {
    let t0 = document.session.t0_ns;
    let mut lines: BTreeMap<(i64, usize), String> = BTreeMap::new();
    for (index, segment) in document.segments.iter().enumerate() {
        let at = stamp(segment.t0_ns - t0);
        // A stretch measured across time the recording does not cover: its
        // figures are of a stretch nobody watched, and asking for it as a clip
        // is refused. Said at the end of the line rather than beside the id,
        // which stays the bare token `--extract` takes.
        let marked = if segment.spans_hole { "  HOLE" } else { "" };
        let id = &segment.id;
        let line = if segment.kind == DocKind::Still {
            let where_it_stood = segment.pose.as_ref().map_or_else(
                || "pose unsolved".to_owned(),
                |pose| {
                    let [roll, pitch, yaw] = pose.rpy_deg();
                    format!(
                        "z={:+.0}mm pitch={pitch:+.0}° roll={roll:+.0}° yaw={yaw:+.0}°",
                        pose.height_mm
                    )
                },
            );
            let [right, left] = segment.antennas_rad.unwrap_or([None; 2]).map(|angle| {
                // A row that answered nothing over the hold has no angle, and
                // saying so is not the same as saying it sat at zero.
                angle.map_or_else(|| "unread".to_owned(), |angle| format!("{angle:+.2}"))
            });
            format!(
                "{at}  STILL  {id}  {:.2}s  {where_it_stood} | ant r={right} l={left}{marked}",
                segment.duration_s
            )
        } else {
            let antennas = segment
                .joints_path_rad
                .as_ref()
                .map_or(0.0, |path| path[7].max(path[8]));
            format!(
                "{at}  MOVE   {id}  {:.2}s  head {:.0}mm at ≤{:.0}mm/s, {:.0}° | ant \
                 {antennas:.1}rad{marked}",
                segment.duration_s,
                segment.head_path_mm.unwrap_or(0.0),
                segment.peak_head_speed_mm_s.unwrap_or(0.0),
                segment.net_rotation_deg.unwrap_or(0.0),
            )
        };
        lines.insert((segment.t0_ns, index), line);
    }
    // The utterances into the same order, keyed past every segment's slot so a
    // hold and a word beginning on the same nanosecond print hold first: the
    // segment is where the word was said, and reading the word first would
    // describe a pose the line has not named yet. An utterance with no interval
    // has no place in a chronology and is in the document alone.
    let after_segments = document.segments.len();
    for (index, said) in document.utterances.iter().enumerate() {
        let Some(t_start_ns) = said.t_start_ns else {
            continue;
        };
        let at = stamp(t_start_ns - t0);
        let before = said.still_before.as_deref().unwrap_or("-");
        let after = said.still_after.as_deref().unwrap_or("-");
        let over = if said.overlaps_playback {
            " over-readback"
        } else {
            ""
        };
        lines.insert(
            (t_start_ns, after_segments + index),
            format!(
                "{at}  SAY    #{}    {:?}  before={before} after={after}{over}",
                said.seq, said.text
            ),
        );
    }
    let mut text = String::new();
    for line in lines.values() {
        text.push_str(line);
        text.push('\n');
    }
    text
}

/// `mm:ss.cc` from the session's own start.
///
/// Signed, because a stamp before the session's first sample is what a
/// wall-clock step under a run produces, and `00:-3.40` is not a time anybody
/// reads.
fn stamp(offset_ns: i64) -> String {
    let sign = if offset_ns < 0 { "-" } else { "" };
    // Rounded to centiseconds and split as whole numbers: splitting the float
    // instead leaves a remainder that can round up to sixty and print as
    // `00:60.00`.
    let centis = ((offset_ns as f64 * 1e-9).abs() * 100.0).round() as i64;
    let (minutes, rest) = (centis / 6000, centis % 6000);
    format!("{sign}{minutes:02}:{:02}.{:02}", rest / 100, rest % 100)
}

/// What the flags asked for.
struct Invocation {
    /// The fetched record directory.
    records: PathBuf,
    /// Where the document is written.
    out: PathBuf,
    /// The segmentation the analyzer runs.
    cfg: SegmentConfig,
    /// The clip draft to write beside the document, where one was asked for.
    extract: Option<Extract>,
}

/// Parse the command line, or say what it should have been.
///
/// The four segmenter flags each default to the constant in `segments.rs`, so
/// tuning a threshold over a fetched session is a re-run rather than a source
/// edit and a rebuild. Which values were used travel in the document, because a
/// segment id means nothing without the configuration that produced it.
fn invocation(args: impl Iterator<Item = String>) -> Result<Invocation, String> {
    const USAGE: &str = "usage: pose_session_report <records> --out <dir> \
[--speed-window-ms MS] [--still-rad-s RAD_S] [--moving-rad-s RAD_S] [--min-still-ms MS] \
[--extract <segment-id> [--name <clip name>] [--channels head,antennas,body_yaw]]";
    let mut args = args.peekable();
    let records = args
        .next_if(|word| !word.starts_with("--"))
        .ok_or_else(|| USAGE.to_owned())?;
    let mut out: Option<String> = None;
    let mut cfg = SegmentConfig::default();
    let mut segment: Option<String> = None;
    let mut name: Option<String> = None;
    let mut channels: Option<ChannelMask> = None;
    while let Some(flag) = args.next() {
        let value = args
            .next()
            .ok_or_else(|| format!("{flag} takes a value\n{USAGE}"))?;
        match flag.as_str() {
            "--out" => out = Some(value),
            "--speed-window-ms" => cfg.speed_window = millis(&flag, &value)?,
            "--min-still-ms" => cfg.min_still = millis(&flag, &value)?,
            "--still-rad-s" => cfg.still_rad_per_s = number(&flag, &value)?,
            "--moving-rad-s" => cfg.moving_rad_per_s = number(&flag, &value)?,
            "--extract" => segment = Some(value),
            "--name" => name = Some(value),
            // The channel spellings are the format's own, parsed by the format:
            // a tool with its own list parser is a tool that disagrees with the
            // document it writes.
            "--channels" => {
                channels = Some(
                    ChannelMask::parse(&value).map_err(|error| format!("--channels: {error}"))?,
                );
            }
            other => return Err(format!("{other} is not a flag this tool takes\n{USAGE}")),
        }
    }
    let out = out.ok_or_else(|| USAGE.to_owned())?;
    // The thresholds are the crate's to judge, on the grid the stream is
    // written at: an invariant checked in one command line is an invariant the
    // next caller re-litigates or forgets.
    cfg.check(POSE_LOG_PERIOD)
        .map_err(|error| format!("{error}\n{USAGE}"))?;
    // A draft's name and its mask mean nothing without a segment to cut, and a
    // run that quietly ignored them would write a session document the operator
    // then reads as a failed extraction.
    if segment.is_none() && (name.is_some() || channels.is_some()) {
        return Err(format!("--name and --channels are --extract's\n{USAGE}"));
    }
    Ok(Invocation {
        records: PathBuf::from(records),
        out: PathBuf::from(out),
        cfg,
        extract: segment.map(|segment| Extract {
            segment,
            name,
            // Every channel by default: what a recording holds is where all
            // three stood, and dropping one is the operator's own act.
            channels: channels.unwrap_or_else(ChannelMask::all),
        }),
    })
}

/// A flag's millisecond value as a duration.
fn millis(flag: &str, value: &str) -> Result<std::time::Duration, String> {
    value
        .parse::<u64>()
        .map(std::time::Duration::from_millis)
        .map_err(|_| format!("{flag} wants whole milliseconds, not {value:?}"))
}

/// A flag's value as a finite, positive number.
fn number(flag: &str, value: &str) -> Result<f64, String> {
    match value.parse::<f64>() {
        Ok(number) if number.is_finite() && number > 0.0 => Ok(number),
        _ => Err(format!("{flag} wants a positive number, not {value:?}")),
    }
}

/// Write the document and the timeline into `out`.
///
/// # Errors
///
/// If the directory cannot be made or either file written.
fn write_out(out: &Path, document: &Document) -> std::io::Result<()> {
    std::fs::create_dir_all(out)?;
    let json = serde_json::to_string_pretty(document)
        .expect("a document of numbers and strings serializes");
    std::fs::write(out.join("session.json"), json + "\n")?;
    std::fs::write(out.join("timeline.txt"), timeline(document))
}

/// Write one clip draft into `out`.
///
/// The library's own extension, so a draft the operator is happy with is copied
/// into a library directory as it stands.
///
/// # Errors
///
/// If the file cannot be written.
fn write_draft(out: &Path, extraction: &Extraction) -> std::io::Result<()> {
    std::fs::create_dir_all(out)?;
    let json = serde_json::to_string_pretty(&extraction.doc)
        .expect("a clip document of numbers and strings serializes");
    std::fs::write(out.join(&extraction.file), json + "\n")
}

fn main() -> ExitCode {
    let invocation = match invocation(std::env::args().skip(1)) {
        Ok(invocation) => invocation,
        Err(usage) => {
            eprintln!("{usage}");
            return ExitCode::FAILURE;
        }
    };
    if run(&invocation) {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

/// One invocation, from the fetched records to whether it held.
fn run(invocation: &Invocation) -> bool {
    let stream = Stream::read(&invocation.records);
    let voice = Voice::read(&invocation.records);
    let mut analysis = analyze(
        &invocation.records,
        &stream,
        &voice,
        invocation.cfg,
        invocation.extract.as_ref(),
    );
    // Everything this run writes, written here and nowhere else. The audio
    // first, because the document names each cut by file name: a document
    // written ahead of them points at files a failed write never made, and a
    // session directory whose transcript names a wav that is not there is one
    // nobody can tell from a pruned store.
    let mut asked = true;
    let mut made = None;
    for cut in &analysis.cuts {
        if let Err(error) = run_audio::write_cut(&invocation.out, &cut.name, &cut.pcm, &mut made) {
            eprintln!("pose_session_report: {error}");
            // Said where the transcript is as well as on the terminal, in the
            // words every other missing cut is said in, so the document
            // describes what is on disk beside it.
            if let Some(said) = analysis
                .document
                .utterances
                .iter_mut()
                .find(|said| said.seq == cut.seq)
            {
                said.wav = None;
                said.wav_error = Some(error);
            }
            asked = false;
        }
    }
    if let Err(error) = write_out(&invocation.out, &analysis.document) {
        eprintln!(
            "pose_session_report: {} could not be written: {error}",
            invocation.out.display()
        );
        asked = false;
    }
    // The document is written either way: a refused extraction is still a
    // session worth reading, and the ids and figures in it are how the operator
    // picks the next segment to ask for.
    match &analysis.extraction {
        Some(Ok(extraction)) => {
            if let Err(error) = write_draft(&invocation.out, extraction) {
                eprintln!(
                    "pose_session_report: {} could not be written: {error}",
                    invocation.out.join(&extraction.file).display()
                );
                asked = false;
            } else {
                eprintln!(
                    "pose_session_report: wrote {} as {:?}, {} frame(s)",
                    invocation.out.join(&extraction.file).display(),
                    extraction.doc.name,
                    extraction.doc.frames.len()
                );
            }
        }
        Some(Err(error)) => {
            eprintln!("pose_session_report: {error}");
            asked = false;
        }
        None => {}
    }
    // The verdict prints whatever else went wrong, for the reason the document
    // is written whatever else went wrong: the segment an operator should have
    // asked for is in what it prints, and an invocation that swallowed the
    // session's own account would send them to read the JSON by hand. What this
    // invocation was asked to do and did not — a mistyped `--extract` id, a cut
    // that would not write — still fails the run, so the status is the worse of
    // the two.
    let held = write_verdict(
        &mut std::io::stdout().lock(),
        &mut std::io::stderr().lock(),
        "pose_session_report",
        &invocation.records.display().to_string(),
        &analysis.report,
        "the session recorded a pose stream, it segmented whole, and the voice host was \
         transcribing over it",
    );
    held && asked
}

#[cfg(test)]
mod tests {
    use std::path::Path;

    use nalgebra::Isometry3;
    use reachy_bench::poselog::{
        EndedLine, LateLine, POSE_LOG_PERIOD_MS, PoseLine, PoseLogEnd, RefusedLine, SampleLine,
        SegmenterLine, StartedLine, StateLine,
    };
    use reachy_clips::format::Channel;
    use reachy_kin::{
        LegAngles, default_geometry, inverse_kinematics, neutral_head_pose, stow_head_pose,
    };
    use reachy_motion::joints::{JointVector, ROWS};
    use reachy_motion::neutral_targets;
    use reachy_motion::segments::SegmentConfig;
    use reachy_scratch::{Scratch, scratch_dir};
    use run_report::{audio_dir, console_dir, sibling};

    use std::path::PathBuf;

    use super::{
        Analysis, ChannelMask, DocKind, Document, Endpointer, Extract, Extraction, Hole,
        Invocation, PERIOD_MS, ROW_COUNT, Sample, Stream, Voice, analyze, hole_in, holes,
        invocation, off_grid_frame, run, stamp, timeline, write_draft, write_out,
    };

    /// Radians to the counts the recorder writes.
    fn counts_of(radians: f64) -> i32 {
        dxl_proto::conv::rad_to_counts(radians).expect("an angle a servo can report")
    }

    /// The cranks that hold `pose`, as the nine-row reading of a limp machine.
    fn cranks_at(pose: &Isometry3<f64>) -> JointVector {
        let mut angles = LegAngles([0.0; 6]);
        inverse_kinematics(default_geometry(), pose, &mut angles).expect("a reachable pose");
        JointVector {
            body_yaw: 0.0,
            legs: angles.0,
            antennas: [0.0, 0.0],
        }
    }

    /// How many samples the fixture's first hold runs for.
    const HELD: usize = 150;
    /// How many samples the fixture's move runs for.
    const MOVED: usize = 30;
    /// How many samples the fixture's second hold runs for.
    const SETTLED: usize = 100;
    /// Which sample of the first hold is missing its left antenna.
    const ANTENNA_GONE: usize = 40;
    /// Which sample of the second hold is followed by a 60 ms hole.
    const HOLE_AT: usize = 50;

    /// What a fixture session holds beyond the two poses it moves between.
    #[derive(Clone, Copy, Default)]
    struct Extras {
        /// Rows that stop answering from this sample of the whole session on:
        /// servos that go away and do not come back.
        silent: Option<(&'static [usize], usize)>,
        /// A stretch of the move, as (first sample of the move, how many),
        /// whose cranks no linkage closes on: a hand putting the head where
        /// forward kinematics will not follow.
        unsolvable: Option<(usize, usize)>,
        /// Whether the closing line is written. A run the operator killed, or a
        /// fetch that truncated, has none.
        no_ending: bool,
        /// Whether the voice host's console is written at all. A session
        /// recorded with the pipeline down has none, which is the shape the
        /// verdict is red on.
        no_voice: bool,
        /// Which sample of the move is followed by a 60 ms hole: a reading the
        /// recorder was late for, inside a stretch somebody may ask to extract.
        hole_in_move: Option<usize>,
        /// A wall clock that steps under the session, as (which reading of the
        /// second hold it steps at, by how many nanoseconds). The grid clock
        /// keeps its period across it, which is what a unit setting its time
        /// from the network mid-session does to a stream.
        wall_step: Option<(usize, i64)>,
        /// A reading of the move no row answered: a sample the segmenter never
        /// counted, so it leaves a hole the stamp lists cannot see.
        unanswered_in_move: Option<usize>,
        /// Whether the second hold's own 60 ms hole is left out, so the rest of
        /// the stream is on the grid. A case about a hole placed somewhere else
        /// wants the only hole in the session to be its own.
        no_hole: bool,
    }

    /// A fixture session: a hold, a move of the six cranks, a second hold, with
    /// one null row, one late gap and the live notes the recorder writes.
    ///
    /// Written through the recorder's own line types, so a rename on either
    /// side of the wire format is a compile error here rather than a fetched
    /// session that reads as a recording with no samples in it.
    fn fixture(at: &Path, hold: &JointVector, moved: &JointVector) -> String {
        fixture_with(at, hold, moved, Extras::default())
    }

    /// [`fixture`], with whatever a case needs the session to have gone wrong.
    fn fixture_with(at: &Path, hold: &JointVector, moved: &JointVector, extras: Extras) -> String {
        // The whole session as readings first, so the live notes can be placed
        // where the recorder would have written them.
        let mut readings: Vec<(i64, JointVector, [bool; ROW_COUNT])> = Vec::new();
        let mut t_ns = 0_i64;
        let mut push = |t_ns: i64, joints: JointVector, null_row: Option<usize>| {
            let mut missing = [false; ROW_COUNT];
            if let Some(row) = null_row {
                missing[row] = true;
            }
            readings.push((t_ns, joints, missing));
        };
        for step in 0..HELD {
            push(t_ns, *hold, (step == ANTENNA_GONE).then_some(ROW_COUNT - 1));
            t_ns += 20_000_000;
        }
        for step in 1..=MOVED {
            let mut joints = *hold;
            for leg in 0..6 {
                joints.legs[leg] = hold.legs[leg]
                    + (moved.legs[leg] - hold.legs[leg]) * step as f64 / MOVED as f64;
            }
            if let Some((from, count)) = extras.unsolvable
                && step > from
                && step <= from + count
            {
                // Five cranks at zero and the sixth two and a half radians out:
                // six angles no rigid head holds at once.
                joints.legs = [0.0, 0.0, 0.0, 0.0, 0.0, 2.5];
            }
            push(t_ns, joints, None);
            t_ns += if extras.hole_in_move == Some(step) {
                60_000_000
            } else {
                20_000_000
            };
        }
        let mut hole_ns = None;
        for step in 0..SETTLED {
            push(t_ns, *moved, None);
            let holed = step == HOLE_AT && !extras.no_hole;
            t_ns += if holed { 60_000_000 } else { 20_000_000 };
            if holed {
                hole_ns = Some(t_ns);
            }
        }
        if let Some((rows, from)) = extras.silent {
            for (_, _, missing) in readings.iter_mut().skip(from) {
                for row in rows {
                    missing[*row] = true;
                }
            }
        }
        if let Some(step) = extras.unanswered_in_move {
            // A reading the whole bus was quiet for: the segmenter does not
            // count it as a sample, so nothing in the stamp lists sees it and
            // the draft's own frame spacing is what catches it.
            readings[HELD + step].2 = [true; ROW_COUNT];
        }

        // The wall clock, which is the grid clock plus whatever it stepped by.
        // Everything written below is stamped on both: `t_ns` is the join key
        // against the voice host and can move, `mono_ns` is the grid the
        // recorder paced on and cannot.
        let wall_of = |index: usize| -> i64 {
            let stepped = extras
                .wall_step
                .filter(|(at, _)| index >= *at)
                .map_or(0, |(_, by)| by);
            readings[index].0 + stepped
        };

        // Where the live segmenter's notes fall: the run opens inside a hold,
        // so the first note lands min_still after the first sample and carries
        // its stamp; the move is announced on the sample it was seen on; and the
        // hold after it is announced min_still after the difference baseline
        // stopped carrying any of the travel.
        let floor = 25;
        let settles = HELD + MOVED + 5;
        let notes: [(usize, PoseLine); 3] = [
            (floor, PoseLine::Still(StateLine { t0_ns: wall_of(0) })),
            (
                HELD,
                PoseLine::Moving(StateLine {
                    t0_ns: wall_of(HELD),
                }),
            ),
            (
                settles + floor,
                PoseLine::Still(StateLine {
                    t0_ns: wall_of(settles),
                }),
            ),
        ];

        let mut lines: Vec<String> = vec![written(&PoseLine::Started(StartedLine {
            t_ns: 0,
            device: "/dev/ttyAMA3".to_owned(),
            ids: [10, 11, 12, 13, 14, 15, 16, 17, 18],
            period_ms: POSE_LOG_PERIOD_MS,
            torque: [0; ROW_COUNT],
            segmenter: SegmenterLine::of(SegmentConfig::default()),
        }))];
        for (index, (mono_ns, joints, missing)) in readings.iter().enumerate() {
            let mut counts = [None; ROW_COUNT];
            for (row, (slot, joint)) in counts.iter_mut().zip(ROWS).enumerate() {
                if missing[row] {
                    continue;
                }
                *slot = Some(counts_of(joints.get(joint).expect("a servo row")));
            }
            lines.push(written(&PoseLine::Sample(SampleLine {
                t_ns: wall_of(index),
                mono_ns: *mono_ns,
                read_us: 1700,
                counts,
            })));
            if index == HELD + HOLE_AT
                && let Some(t_ns) = hole_ns
            {
                lines.push(written(&PoseLine::Late(LateLine {
                    t_ns,
                    behind_us: 40_000,
                })));
            }
            for (at, note) in &notes {
                if *at == index {
                    lines.push(written(note));
                }
            }
        }
        if !extras.no_ending {
            lines.push(written(&PoseLine::Ended(EndedLine {
                t_ns,
                samples: readings.len(),
                late: usize::from(!extras.no_hole),
                torque: [Some(0); ROW_COUNT],
                cause: PoseLogEnd::Signal,
                error: None,
            })));
        }
        let console = console_dir(at);
        std::fs::create_dir_all(&console).expect("a scratch console directory");
        std::fs::write(console.join("recorder_0.log"), lines.join("\n") + "\n")
            .expect("a writable fixture");
        if !extras.no_voice {
            voice_log(at);
        }
        lines.join("\n")
    }

    /// One line of the stream, as the recorder writes it.
    fn written(line: &PoseLine) -> String {
        serde_json::to_string(line).expect("a record of numbers and strings")
    }

    /// How far the fixture's move lifts the head, metres.
    const LIFT_M: f64 = 0.03;

    /// The two poses the fixture holds: stow, and stow lifted [`LIFT_M`].
    fn fixture_poses() -> (JointVector, JointVector) {
        let held = stow_head_pose();
        let lifted = Isometry3::from_parts(
            (held.translation.vector + nalgebra::Vector3::new(0.0, 0.0, LIFT_M)).into(),
            held.rotation,
        );
        (cranks_at(&held), cranks_at(&lifted))
    }

    /// The whole reading of one fetch, as a case has just written it.
    fn judge(records: &Path, cfg: SegmentConfig) -> Analysis {
        judge_extracting(records, cfg, None)
    }

    /// [`judge`], asking for one segment as a clip draft.
    fn judge_extracting(records: &Path, cfg: SegmentConfig, ask: Option<&Extract>) -> Analysis {
        let stream = Stream::read(records);
        let voice = Voice::read(records);
        analyze(records, &stream, &voice, cfg, ask)
    }

    /// The extraction of `segment` from a fixture session, every channel.
    fn drafted(records: &Path, segment: &str) -> Result<Extraction, String> {
        judge_extracting(
            records,
            SegmentConfig::default(),
            Some(&Extract {
                segment: segment.to_owned(),
                name: None,
                channels: ChannelMask::all(),
            }),
        )
        .extraction
        .expect("an extraction was asked for")
    }

    /// The document a fixture session reads as.
    fn read(at: &Scratch, cfg: SegmentConfig) -> Document {
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        fixture(&records, &hold, &moved);
        judge(&records, cfg).document
    }

    /// What the fixture's first utterance is stamped at, microseconds: an
    /// onset inside the first hold, closed inside it.
    const SAID_FIRST: (i64, i64) = (1_000_000, 2_000_000);
    /// The second, whose speech falls inside the move.
    const SAID_OVER_MOVE: (i64, i64) = (3_200_000, 3_500_000);
    /// The third, as (its first audio, its endpoint): it never onset, so its
    /// start is the missed-onset estimate off the first of those.
    const SAID_ESTIMATED: (i64, i64) = (4_500_000, 5_400_000);
    /// When the pod is playing the read-back of the second utterance back,
    /// milliseconds: over the third, and over nothing else.
    const READ_BACK_MS: (i64, i64) = (4_900, 5_600);

    /// The frame log the fixture's utterances name their audio in.
    const FIXTURE_LOG: &str = "a.framelog";
    /// One audio frame of the pod's, samples.
    const FRAME: usize = 640;
    /// What the wav writer puts ahead of the samples, bytes: the canonical
    /// PCM header, which is fixed for the one format the store is written in.
    const WAV_HEADER: u64 = 44;
    /// Where the fixture store's own audio begins, absolute samples.
    const AUDIO_BASE: u64 = 15_360;

    /// One `utterance` line, as the voice host writes one.
    ///
    /// Spelled out rather than built from a map, because the field order is
    /// load-bearing: every line of the stream leads with its timestamp, and
    /// that is what makes an event glued onto a console sentence recoverable at
    /// all. A fixture whose keys came out sorted would prove the reader works
    /// on a shape the pipeline never writes.
    fn utterance(
        id: u64,
        text: &str,
        first_audio_us: i64,
        onset_us: Option<i64>,
        endpoint_us: i64,
    ) -> String {
        let onset = onset_us.map_or_else(|| "null".to_owned(), |us| us.to_string());
        let (start_sample, end_sample) = (AUDIO_BASE + FRAME as u64, AUDIO_BASE + 2 * FRAME as u64);
        format!(
            r#"{{"ts_ms":{ts},"event":"utterance","id":{id},"pod":"reachy00","endpoint_cause":"soft_endpoint","timings":{{"first_audio_rx":{first_audio_us},"t0_projected":false,"onset_rx":{onset},"soft_endpoint_rx":{endpoint_us}}},"audio_ref":{{"log":"{FIXTURE_LOG}","start_sample":{start_sample},"end_sample":{end_sample},"segments":[]}},"transcript":{{"text":"{text}","confidence":{{"avg_logprob":-0.31,"no_speech_prob":0.02,"compression_ratio":1.4}}}}}}"#,
            ts = endpoint_us / 1_000,
        )
    }

    /// The voice host's console for a fixture session: the pipeline announcing
    /// itself, three utterances over the pose stream the fixture wrote, and the
    /// parrot reading the second one back over the third.
    ///
    /// The pipeline's own human console is interleaved with the JSONL in this
    /// file, and the first utterance is glued onto the console sentence about it
    /// the way the launcher's merged descriptor glues them, because that is the
    /// file every real session leaves behind.
    fn voice_log(at: &Path) {
        let first = utterance(
            1,
            "this is the resting pose",
            SAID_FIRST.0 - 300_000,
            Some(SAID_FIRST.0),
            SAID_FIRST.1,
        );
        let lines = [
            r#"{"ts_ms":10,"event":"listening","addr":"127.0.0.1:9"}"#.to_owned(),
            format!("utterance #1 — \"this is the resting pose\" conf: {first}"),
            utterance(
                2,
                "now it moves",
                SAID_OVER_MOVE.0 - 300_000,
                Some(SAID_OVER_MOVE.0),
                SAID_OVER_MOVE.1,
            ),
            format!(
                r#"{{"ts_ms":{},"event":"playback_started","pod":"reachy00","utterance":2,"interruptible":true}}"#,
                READ_BACK_MS.0
            ),
            // The carve that never onset: only its first audio and its endpoint
            // are stamped, so its start is the missed-onset estimate.
            utterance(
                3,
                "and this is the lifted pose",
                SAID_ESTIMATED.0,
                None,
                SAID_ESTIMATED.1,
            ),
            format!(
                r#"{{"ts_ms":{},"event":"playback_finished","pod":"reachy00","utterance":2}}"#,
                READ_BACK_MS.1
            ),
        ];
        let console = console_dir(at);
        std::fs::create_dir_all(&console).expect("a scratch console directory");
        std::fs::write(console.join("voice_host_0.log"), lines.join("\n") + "\n")
            .expect("a writable fixture");
    }

    /// A store beside `records` covering every span the fixture's utterances
    /// name, written through brenn-pod's own frame-log writer.
    fn audio_store(records: &Path) {
        let dir = audio_dir(records);
        std::fs::create_dir_all(&dir).expect("an audio store");
        let mut written = vec![
            pod_ingest::test_fixtures::hello("reachy00"),
            pod_ingest::test_fixtures::seg_start(1, AUDIO_BASE),
        ];
        for frame in 0..4 {
            written.push(pod_ingest::test_fixtures::audio(
                1,
                AUDIO_BASE + (frame * FRAME) as u64,
                FRAME,
            ));
        }
        written.push(pod_ingest::test_fixtures::seg_end(1, (4 * FRAME) as u64));
        pod_ingest::test_fixtures::write_log(&dir.join(FIXTURE_LOG), &written);
    }

    #[test]
    fn a_hold_a_move_and_a_hold_read_as_three_segments() {
        let at = scratch_dir("pose-session-three");
        let document = read(&at, SegmentConfig::default());
        let kinds: Vec<DocKind> = document.segments.iter().map(|doc| doc.kind).collect();
        assert_eq!(
            kinds,
            vec![DocKind::Still, DocKind::Motion, DocKind::Still],
            "{kinds:?}"
        );
        assert_eq!(document.session.samples, 280);
        assert_eq!(document.session.late, 1);
        // The move's neighbours are the two holds, and the segments tile.
        assert_eq!(document.segments[1].from.as_deref(), Some("S001"));
        assert_eq!(document.segments[1].to.as_deref(), Some("S003"));
        assert_eq!(document.segments[0].t1_ns, document.segments[1].t0_ns);
        assert_eq!(document.segments[1].t1_ns, document.segments[2].t0_ns);
    }

    #[test]
    fn the_holds_solve_to_the_poses_they_were_recorded_at() {
        let at = scratch_dir("pose-session-fk");
        let document = read(&at, SegmentConfig::default());
        let first = document.segments[0].pose.as_ref().expect("a solved hold");
        let stow = super::PoseFigures::of(&stow_head_pose());
        assert!(
            (first.height_mm - stow.height_mm).abs() < 0.5,
            "{first:?} against {stow:?}"
        );
        assert!(document.segments[0].fk_valid);
        // The second hold is the first lifted by LIFT_M, which is what the
        // move travelled and what the head figures say it travelled.
        let second = document.segments[2].pose.as_ref().expect("a solved hold");
        assert!(
            (second.height_mm - first.height_mm - LIFT_M * 1e3).abs() < 0.5,
            "{second:?} against {first:?}"
        );
        let move_mm = document.segments[1].head_path_mm.expect("a move's path");
        assert!((move_mm - LIFT_M * 1e3).abs() < 5.0, "{move_mm} mm");
        assert!(
            document.segments[1].peak_head_speed_mm_s.expect("a speed") > 10.0,
            "{:?}",
            document.segments[1].peak_head_speed_mm_s
        );
    }

    /// A hold recorded at a known tilt reads that tilt back: the rotation half
    /// of the pose figures, which is the half a clip's head delta is built out
    /// of and the half a translation-only fixture never exercises.
    #[test]
    fn a_tilted_hold_reads_its_own_pitch_and_yaw() {
        /// How far the fixture's second hold pitches, degrees.
        const PITCH_DEG: f64 = 8.0;
        /// How far it yaws, degrees.
        const YAW_DEG: f64 = 12.0;

        let at = scratch_dir("pose-session-tilt");
        let records = at.join("session");
        // Neutral rather than stow: the figures are all of the pose *relative
        // to neutral*, and neutral is where the two are the same, so the
        // generating angles are what the document should print.
        let held = neutral_head_pose();
        // Yaw about the base vertical after a pitch about its own y: the
        // tilted head's own vertical is `PITCH_DEG` off the base's whatever the
        // yaw, and the yaw is what the envelope calls the relative yaw.
        let turn = nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::z_axis(),
            YAW_DEG.to_radians(),
        ) * nalgebra::UnitQuaternion::from_axis_angle(
            &nalgebra::Vector3::y_axis(),
            PITCH_DEG.to_radians(),
        );
        let tilted = Isometry3::from_parts(held.translation, turn * held.rotation);
        assert!(
            held.rotation.angle().abs() < 1e-9,
            "the figures below are the delta from neutral, which wants an unturned neutral: {:?}",
            held.rotation
        );
        fixture(&records, &cranks_at(&held), &cranks_at(&tilted));
        let document = judge(&records, SegmentConfig::default()).document;

        let flat = document.segments[0].pose.as_ref().expect("a solved hold");
        assert!(flat.cone_deg < 0.5, "{flat:?}");
        assert!(flat.relative_yaw_deg.abs() < 0.5, "{flat:?}");

        let turned = document.segments[2].pose.as_ref().expect("a solved hold");
        assert!(
            (turned.cone_deg - PITCH_DEG).abs() < 0.5,
            "cone {} deg, expected {PITCH_DEG}: {turned:?}",
            turned.cone_deg
        );
        assert!(
            (turned.relative_yaw_deg - YAW_DEG).abs() < 0.5,
            "yaw {} deg, expected {YAW_DEG}: {turned:?}",
            turned.relative_yaw_deg
        );
        // And the quaternion the clip's head delta is built from is the turn
        // itself, sign and all.
        let expected = turn.quaternion();
        for (was, wanted) in turned
            .dq
            .iter()
            .zip([expected.w, expected.i, expected.j, expected.k])
        {
            assert!(
                (was - wanted).abs() < 5e-3,
                "{:?} against {expected}",
                turned.dq
            );
        }

        // The move between them turns by the angle between the two holds, and
        // it does it over the ramp's own six tenths of a second.
        let moved = &document.segments[1];
        let angle_deg = turn.angle().to_degrees();
        let net = moved.net_rotation_deg.expect("a move turns");
        // Not the whole angle: the hold before it keeps the sub-threshold
        // lead-in, which is the first few samples of the ramp, and the move is
        // what is left. All of the rest of it, and none of anyone else's.
        assert!(
            (0.85 * angle_deg..=angle_deg + 0.1).contains(&net),
            "net rotation {net} deg, expected nearly all of {angle_deg}"
        );
        let peak = moved
            .peak_head_angular_speed_deg_s
            .expect("a move has an angular speed");
        let nominal = angle_deg / 0.6;
        assert!(
            (peak - nominal).abs() < 0.5 * nominal,
            "peak angular speed {peak} deg/s, expected about {nominal}"
        );
    }

    /// A stretch of the move whose cranks no linkage closes on: the frames
    /// either side of the hole still solve, the hole contributes no travel of
    /// its own, and the count of solved frames says how many did.
    #[test]
    fn a_hole_in_the_middle_of_a_move_leaves_the_frames_around_it_solved() {
        /// Which sample of the move goes unsolvable.
        const FROM: usize = 10;
        /// How many of them do.
        const COUNT: usize = 10;

        let at = scratch_dir("pose-session-hole");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        fixture_with(
            &records,
            &hold,
            &moved,
            Extras {
                unsolvable: Some((FROM, COUNT)),
                ..Extras::default()
            },
        );
        let document = judge(&records, SegmentConfig::default()).document;

        let move_doc = document
            .segments
            .iter()
            .find(|doc| doc.kind == DocKind::Motion)
            .expect("a move");
        assert!(
            move_doc.fk_valid_frames > 0 && move_doc.fk_valid_frames < move_doc.frames,
            "{} of {} frames solved",
            move_doc.fk_valid_frames,
            move_doc.frames
        );
        assert_eq!(
            move_doc.frames - move_doc.fk_valid_frames,
            COUNT,
            "the unsolvable stretch is the only one that did not solve"
        );
        // The hole is a hole and not a detour: the head's path is still the
        // lift, and its speed is still the ramp's.
        let path = move_doc.head_path_mm.expect("a move's path");
        assert!((path - LIFT_M * 1e3).abs() < 5.0, "{path} mm");
        let peak = move_doc.peak_head_speed_mm_s.expect("a speed");
        let nominal = LIFT_M * 1e3 / 0.6;
        assert!(peak < 2.0 * nominal, "peak {peak} mm/s against {nominal}");
        // And the hold after it solves again, from the resting seeds.
        assert!(
            document.segments.last().expect("a last segment").fk_valid,
            "{:?}",
            document.segments.last()
        );

        // Every sample of the stream fell in exactly one segment.
        let framed: usize = document.segments.iter().map(|doc| doc.frames).sum();
        assert_eq!(framed, document.session.samples);
    }

    /// A leg servo that stops answering: the holds it covers have no pose at
    /// all rather than the pose a crank of zero would put the head at.
    #[test]
    fn a_silent_crank_is_no_pose_rather_than_a_crank_of_zero() {
        let at = scratch_dir("pose-session-silent-leg");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        // A leg and the left antenna, gone from the move onwards.
        fixture_with(
            &records,
            &hold,
            &moved,
            Extras {
                silent: Some((&[3, ROW_COUNT - 1], HELD)),
                ..Extras::default()
            },
        );
        let document = judge(&records, SegmentConfig::default()).document;

        // The hold before it is untouched.
        let first = &document.segments[0];
        assert!(first.pose.is_some(), "{first:?}");

        // The hold after it has the cranks it read and no pose: the sixth
        // angle is not a reading and the head was not solved from a zero.
        let last = document.segments.last().expect("a last segment");
        assert_eq!(last.kind, DocKind::Still);
        assert!(last.pose.is_none(), "{last:?}");
        assert!(!last.fk_valid);
        assert_eq!(last.fk_valid_frames, 0);
        assert!(last.joints_mean_rad.is_some(), "the cranks are kept");

        // And the rows themselves: the antenna that answered nothing is a null
        // rather than an angle of exactly zero, while body yaw still answered.
        let antennas = last.antennas_rad.expect("a hold reports its antennas");
        assert!(antennas[0].is_some(), "{antennas:?}");
        assert!(antennas[1].is_none(), "{antennas:?}");
        assert!(last.body_yaw_rad.is_some());
        let text = timeline(&document);
        assert!(text.contains("l=unread"), "{text}");

        // The move over the same stretch solved nothing either.
        let moved = document
            .segments
            .iter()
            .find(|doc| doc.kind == DocKind::Motion)
            .expect("a move");
        assert_eq!(moved.fk_valid_frames, 0, "{moved:?}");
        assert!(!moved.fk_valid);
    }

    /// A stream that stops mid-write: the note that says so, against a whole
    /// one that says how it ended.
    #[test]
    fn a_stream_with_no_closing_line_is_noted() {
        let at = scratch_dir("pose-session-truncated");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        fixture_with(
            &records,
            &hold,
            &moved,
            Extras {
                no_ending: true,
                ..Extras::default()
            },
        );
        let document = judge(&records, SegmentConfig::default()).document;
        assert!(document.session.ended.is_none());
        assert!(
            document
                .session
                .notes
                .iter()
                .any(|note| note.contains("no `ended` line")),
            "{:?}",
            document.session.notes
        );

        // The whole stream carries the writer's own word for how it ended, and
        // no note about it.
        let whole = read(&scratch_dir("pose-session-whole"), SegmentConfig::default());
        assert_eq!(whole.session.ended, Some(PoseLogEnd::Signal));
        assert!(
            !whole
                .session
                .notes
                .iter()
                .any(|note| note.contains("`ended`")),
            "{:?}",
            whole.session.notes
        );
        let json = serde_json::to_string(&whole.session.ended).expect("a word");
        assert_eq!(json, "\"signal\"");
    }

    /// A run the bus ended under: the samples are a recording, and the failure
    /// is a note rather than the verdict "this was not a recording run".
    #[test]
    fn a_run_the_bus_ended_keeps_its_samples_and_names_the_failure() {
        let at = scratch_dir("pose-session-aborted");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        let text = fixture(&records, &hold, &moved);
        let mut lines: Vec<String> = text.lines().map(str::to_owned).collect();
        let last = lines.len() - 1;
        lines[last] = written(&PoseLine::Ended(EndedLine {
            t_ns: 7,
            samples: 280,
            late: 1,
            torque: [None; ROW_COUNT],
            cause: PoseLogEnd::Aborted,
            error: Some("servo 12: reading present position failed".to_owned()),
        }));
        lines.push(written(&PoseLine::Refused(RefusedLine {
            t_ns: 7,
            error: "servo 12: reading present position failed".to_owned(),
        })));
        std::fs::write(
            console_dir(&records).join("recorder_0.log"),
            lines.join("\n") + "\n",
        )
        .expect("a writable fixture");
        let analysis = judge(&records, SegmentConfig::default());
        assert!(
            analysis.report.findings.is_empty(),
            "twenty good minutes are not a refusal: {:?}",
            analysis.report.findings
        );
        let notes = &analysis.document.session.notes;
        assert!(
            notes.iter().any(|note| note.contains("ended on a failure")),
            "{notes:?}"
        );
        assert!(
            notes
                .iter()
                .any(|note| note.contains("samples and a `refused` line")),
            "{notes:?}"
        );
        assert_eq!(
            analysis.document.session.ended_error.as_deref(),
            Some("servo 12: reading present position failed")
        );
    }

    /// A stream that cannot be read is not a fetch with no stream in it, and a
    /// torn multibyte character is not a session with no samples.
    #[test]
    fn an_unreadable_stream_says_so_and_a_torn_character_costs_one_line() {
        let at = scratch_dir("pose-session-unreadable");
        let records = at.join("session");
        let console = console_dir(&records);
        // A directory where the stream should be: readable directory, and
        // nothing that can be read out of it.
        std::fs::create_dir_all(console.join("recorder_0.log")).expect("a scratch directory");
        let stream = Stream::read(&records);
        assert!(stream.unreadable.is_some());
        let report = judge(&records, SegmentConfig::default()).report;
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("could not be read")),
            "{:?}",
            report.findings
        );

        // And the operator's own words in the file, torn mid-character by a
        // fetch that copied it while the recorder was writing: the session is
        // still the session.
        let at = scratch_dir("pose-session-torn-character");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        let text = fixture(&records, &hold, &moved);
        let mut bytes = "pose-log over /dev/ttyAMA3 — 1000000 baud"
            .as_bytes()
            .to_vec();
        bytes.truncate(bytes.len() - 2);
        bytes.push(b'\n');
        bytes.extend_from_slice(text.as_bytes());
        std::fs::write(console_dir(&records).join("recorder_0.log"), bytes)
            .expect("a writable fixture");
        let stream = Stream::read(&records);
        assert!(stream.unreadable.is_none());
        assert_eq!(stream.samples.len(), HELD + MOVED + SETTLED);
        assert_eq!((stream.torn, stream.prose), (0, 1));
    }

    #[test]
    fn cranks_no_linkage_closes_on_keep_their_angles_and_no_pose() {
        let at = scratch_dir("pose-session-nonsense");
        let records = at.join("session");
        // Five cranks at zero place the head; the sixth at 2.5 rad then has a
        // rod length it cannot have. Six angles no rigid head holds at once.
        let nonsense = JointVector {
            body_yaw: 0.0,
            legs: [0.0, 0.0, 0.0, 0.0, 0.0, 2.5],
            antennas: [0.0, 0.0],
        };
        fixture(&records, &nonsense, &nonsense);
        let document = judge(&records, SegmentConfig::default()).document;
        let still = document
            .segments
            .iter()
            .find(|doc| doc.kind == DocKind::Still)
            .expect("a hold");
        assert!(still.pose.is_none(), "{:?}", still.pose);
        assert!(!still.fk_valid);
        assert_eq!(still.fk_valid_frames, 0);
        let mean = still.joints_mean_rad.as_ref().expect("the cranks");
        assert!((mean[6] - 2.5).abs() < 2e-3, "{mean:?}");
    }

    #[test]
    fn the_hole_in_the_stream_is_a_gap_and_nothing_is_a_clock_step() {
        let at = scratch_dir("pose-session-gap");
        let records = at.join("session");
        let document = read(&at, SegmentConfig::default());
        assert_eq!(
            document.session.gaps.len(),
            1,
            "{:?}",
            document.session.gaps
        );
        assert!(
            (document.session.gaps[0].ms - 60.0).abs() < 1e-6,
            "{:?}",
            document.session.gaps[0]
        );
        assert!(document.session.clock_steps.is_empty());
        // The 60 ms step is a gap because it is outside the grid this reader
        // judges spacing against.
        assert!((document.session.gaps[0].ms - 3.0 * PERIOD_MS).abs() < 1e-6);

        // Both clocks stepped together, so it is the stream that is missing a
        // stretch and not the wall clock that moved — and the hold it falls in
        // is a hold whose mean was taken across time nobody watched. Marked,
        // said on the timeline, and refused as a draft.
        let held = &document.segments[2];
        assert_eq!(held.kind, DocKind::Still);
        assert!(held.spans_hole, "{held:?}");
        assert!(
            !document.segments[0].spans_hole,
            "{:?}",
            document.segments[0]
        );
        let text = timeline(&document);
        let marked: Vec<&str> = text
            .lines()
            .filter(|line| line.ends_with("  HOLE"))
            .collect();
        assert_eq!(marked.len(), 1, "{text}");
        assert!(marked[0].contains("STILL  S003  "), "{text}");
        assert!(!text.contains("S003 HOLE"), "{text}");
        let refused = drafted(&records, "S003").expect_err("a hold with a hole in it");
        assert!(
            refused.contains("a 60 ms gap at t_ns") && refused.contains("cannot be extracted"),
            "{refused}"
        );
        drafted(&records, "S001").expect("the hold before the hole");
    }

    /// Where a wall step is placed in the fixture: inside the first hold, far
    /// enough in that the hold is long open and far enough from its end that
    /// the move after it is measured entirely on one side of the step.
    const STEPS_AT: usize = 90;

    /// A fixture session whose wall clock steps by `by` at [`STEPS_AT`], and
    /// the document it reads as.
    fn stepped_session(at: &Scratch, by: i64) -> (PathBuf, Document) {
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        fixture_with(
            &records,
            &hold,
            &moved,
            Extras {
                wall_step: Some((STEPS_AT, by)),
                ..Extras::default()
            },
        );
        let document = judge(&records, SegmentConfig::default()).document;
        (records, document)
    }

    /// A wall clock that jumps forward is a step and not a gap, and the hold it
    /// lands in is marked and cannot be drafted.
    ///
    /// Forward is the likely direction on this unit: it boots from RAM and sets
    /// its clock from the network, so a session started before the first sync
    /// sees one large jump. On wall stamps alone that is indistinguishable from
    /// a loop that stalled, which is why every sample carries the grid stamp
    /// too.
    #[test]
    fn a_forward_wall_step_is_a_step_and_the_hold_it_lands_in_is_not_drafted() {
        let at = scratch_dir("pose-session-step-forward");
        let step_ms = 30_000.0;
        let (records, document) = stepped_session(&at, (step_ms * 1e6) as i64);

        let steps = &document.session.clock_steps;
        assert_eq!(steps.len(), 1, "{steps:?}");
        assert!((steps[0].ms - step_ms).abs() < 1.0, "{steps:?}");
        // The stream's own 60 ms hole is still a gap, and the step is not one:
        // the two are different facts about what the head was doing.
        assert_eq!(
            document.session.gaps.len(),
            1,
            "{:?}",
            document.session.gaps
        );
        assert!(
            (document.session.gaps[0].ms - 60.0).abs() < 1e-6,
            "{:?}",
            document.session.gaps
        );
        assert!(
            document
                .session
                .notes
                .iter()
                .any(|note| note.contains("stepped 1 time(s)") && note.contains("+30000 ms")),
            "{:?}",
            document.session.notes
        );

        // The hold it landed in is marked, on the line an operator picks
        // segments off, and refused as a draft by what is wrong with it.
        let first = &document.segments[0];
        assert_eq!(first.id, "S001");
        assert!(first.spans_hole, "{first:?}");
        let text = timeline(&document);
        assert!(
            text.lines()
                .any(|line| line.contains("STILL  S001  ") && line.ends_with("  HOLE")),
            "{text}"
        );
        assert!(!text.contains("S001 HOLE"), "{text}");
        let refused = drafted(&records, "S001").expect_err("a hold the clock stepped under");
        assert!(
            refused.contains("wall-clock step of +30000 ms")
                && refused.contains(
                    "cannot be \
             extracted"
                ),
            "{refused}"
        );

        // The move after it is on one side of the step and drafts as it always
        // did: a hole costs the segment it lands in and nothing else.
        assert!(
            !document.segments[1].spans_hole,
            "{:?}",
            document.segments[1]
        );
        drafted(&records, "S002").expect("the move after the step");
    }

    /// A wall clock that jumps backwards is the same fact, signed the other
    /// way — and it is seen by sample position, not by stamp.
    ///
    /// The step here lands the hold's later stamps back at or before the stamp
    /// the hold opened at, so an interval test over the stamps would place the
    /// entry outside the very segment it sits inside. That is the case
    /// membership by position exists for.
    #[test]
    fn a_backwards_wall_step_is_seen_by_position_and_not_by_stamp() {
        let at = scratch_dir("pose-session-step-back");
        let step_ms = -2_000.0;
        let (records, document) = stepped_session(&at, (step_ms * 1e6) as i64);

        let steps = &document.session.clock_steps;
        assert_eq!(steps.len(), 1, "{steps:?}");
        assert!((steps[0].ms - step_ms).abs() < 1.0, "{steps:?}");
        assert!(
            document
                .session
                .notes
                .iter()
                .any(|note| note.contains("stepped 1 time(s)") && note.contains("-2000 ms")),
            "{:?}",
            document.session.notes
        );

        // The stamp a reader would test against: it is at or before where the
        // hold opened, so no interval over (t0_ns, t1_ns] holds it.
        let first = &document.segments[0];
        assert!(steps[0].t_ns <= first.t0_ns, "{steps:?} {first:?}");
        assert!(first.spans_hole, "{first:?}");
        assert!(
            drafted(&records, "S001")
                .expect_err("a hold the clock stepped under")
                .contains("wall-clock step of -2000 ms")
        );

        // The frames are still assigned, and still assigned once each: the
        // stretch each sample belongs to is its position in the file and not
        // where its stamp sorts.
        let framed: usize = document.segments.iter().map(|doc| doc.frames).sum();
        assert_eq!(framed, document.session.samples);
    }

    /// A clock step that lands on a missing stretch is both facts at once.
    ///
    /// The likely story for a forward step on this unit is a loop that stalled
    /// while the network sync landed, which moved the stamps *and* lost a
    /// stretch of readings. Told only about the step, an operator deciding
    /// whether the session is usable would never learn how much stream is gone.
    #[test]
    fn a_step_on_a_gap_is_reported_as_both() {
        let at = scratch_dir("pose-session-step-on-gap");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        // The later sample of the pair the fixture's 60 ms hole is measured
        // across: the step is placed on the same pair.
        let on_the_gap = HELD + MOVED + HOLE_AT + 1;
        let step_ms = 30_000.0;
        fixture_with(
            &records,
            &hold,
            &moved,
            Extras {
                wall_step: Some((on_the_gap, (step_ms * 1e6) as i64)),
                ..Extras::default()
            },
        );
        let document = judge(&records, SegmentConfig::default()).document;

        let steps = &document.session.clock_steps;
        let gaps = &document.session.gaps;
        assert_eq!(steps.len(), 1, "{steps:?}");
        assert_eq!(gaps.len(), 1, "{gaps:?}");
        assert!((steps[0].ms - step_ms).abs() < 1.0, "{steps:?}");
        assert!((gaps[0].ms - 60.0).abs() < 1e-6, "{gaps:?}");
        assert_eq!(steps[0].t_ns, gaps[0].t_ns, "{steps:?} {gaps:?}");

        assert!(
            document
                .session
                .notes
                .iter()
                .any(|note| note.contains("stepped 1 time(s)") && note.contains("60 ms gap")),
            "{:?}",
            document.session.notes
        );
        let held = &document.segments[2];
        assert!(held.spans_hole, "{held:?}");
        let refused = drafted(&records, &held.id).expect_err("a hold with both under it");
        assert!(
            refused.contains("wall-clock step of") && refused.contains("60 ms gap"),
            "{refused}"
        );
    }

    /// A hole marks the segment that owns the earlier reading of its pair.
    ///
    /// The entry is made from a consecutive pair and its index is of the later
    /// sample, so a segment ending at that index owns the reading before the
    /// hole and its `t1_ns` — which is the post-hole stamp, because segments
    /// tile — spans it. The segment that index opens holds only readings the
    /// recording covers, and is not marked.
    #[test]
    fn a_hole_at_a_boundary_marks_the_stretch_that_owns_the_earlier_reading() {
        let holes = [Hole {
            at: 40,
            t_ns: 0,
            step_ms: None,
            gap_ms: Some(60.0),
        }];
        assert!(hole_in(&holes, 20, 40).is_some(), "at the end of a stretch");
        assert!(hole_in(&holes, 40, 60).is_none(), "at the start of one");
        assert!(hole_in(&holes, 39, 60).is_some(), "one sample inside it");
        assert!(hole_in(&holes, 20, 41).is_some(), "one sample inside it");
    }

    /// A wall step as a move ends: the move is marked and refused, the hold it
    /// opens is neither.
    ///
    /// This is the placement the segmenter itself produces, not one aimed at a
    /// boundary that already existed: the activity quotient is blind for one
    /// speed window after a hole, so a quiet run opens on the hole's own later
    /// reading whenever the head was moving there, and survives whenever the
    /// move was ending anyway. The move then owns the reading before the step
    /// and its `t1_ns` is the stepped stamp, so its reported duration is the
    /// step. The hold holds only readings the recording covers.
    #[test]
    fn a_wall_step_as_a_move_ends_marks_the_move_and_not_the_hold_it_opens() {
        let at = scratch_dir("pose-session-step-at-boundary");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        let step_ms = 30_000.0;
        fixture_with(
            &records,
            &hold,
            &moved,
            Extras {
                // The first reading of the settle, which is the reading after
                // the ramp's last.
                wall_step: Some((HELD + MOVED, (step_ms * 1e6) as i64)),
                no_hole: true,
                ..Extras::default()
            },
        );
        let document = judge(&records, SegmentConfig::default()).document;

        // The boundary first: the case is about a hole the segmenter put a
        // boundary on, so a fixture whose boundary drifted elsewhere fails here
        // rather than passing on a segment that never exercises the rule.
        let (move_doc, held) = (&document.segments[1], &document.segments[2]);
        assert_eq!(move_doc.kind, DocKind::Motion, "{move_doc:?}");
        assert_eq!(held.kind, DocKind::Still, "{held:?}");
        let steps = &document.session.clock_steps;
        assert_eq!(steps.len(), 1, "{steps:?}");
        assert_eq!(move_doc.t1_ns, steps[0].t_ns, "{move_doc:?} {steps:?}");
        assert_eq!(held.t0_ns, steps[0].t_ns, "{held:?} {steps:?}");
        assert!(document.session.gaps.is_empty(), "{:?}", document.session);

        assert!(move_doc.spans_hole, "{move_doc:?}");
        assert!(!held.spans_hole, "{held:?}");
        let text = timeline(&document);
        let marked: Vec<&str> = text
            .lines()
            .filter(|line| line.ends_with("  HOLE"))
            .collect();
        assert_eq!(marked.len(), 1, "{text}");
        assert!(marked[0].contains("MOVE   S002  "), "{text}");

        let refused = drafted(&records, &move_doc.id).expect_err("a move the clock stepped at");
        assert!(
            refused.contains("cannot be extracted")
                && refused.contains("wall-clock step of +30000 ms"),
            "{refused}"
        );
        // No remediation advice: no threshold avoids a hole a segment already
        // spans.
        assert!(!refused.contains("Cut a segment"), "{refused}");
        drafted(&records, &held.id).expect("the hold the step opened");
    }

    /// One reading of nine joints that all answered, on the two clocks.
    fn reading(t_ns: i64, mono_ns: i64) -> Sample {
        Sample {
            t_ns,
            mono_ns,
            present: JointVector::default(),
            missing: [false; ROW_COUNT],
        }
    }

    /// Two readings, the second `wall_ms` later on the wall and `grid_ms` later
    /// on the grid.
    fn paired(wall_ms: f64, grid_ms: f64) -> [Sample; 2] {
        [
            reading(0, 0),
            reading((wall_ms * 1e6) as i64, (grid_ms * 1e6) as i64),
        ]
    }

    /// The slack is what a reader is told about, in both readings, and the
    /// comparisons are the right way round.
    ///
    /// Every fixture in this suite derives its two stamps from each other by a
    /// constant offset, so nothing else in it sits anywhere near the threshold:
    /// a slack silently widened lets a real sub-slack clock adjustment through
    /// into a hold's mean, and one silently narrowed marks every segment of
    /// every session and refuses the lot.
    #[test]
    fn the_slack_decides_what_is_a_step_what_is_a_gap_and_what_is_neither() {
        // (wall interval, grid interval) → (steps, gaps).
        let table = [
            ((20.0, 20.0), (0, 0)),
            // The wall drifting out from under the grid: 9 ms of it is loop
            // jitter, 11 ms is the clock moving.
            ((29.0, 20.0), (0, 0)),
            ((31.0, 20.0), (1, 0)),
            ((20.0, 31.0), (1, 1)),
            // Both clocks together: the stream is the thing that is off the
            // grid, and nothing stepped.
            ((29.0, 29.0), (0, 0)),
            ((31.0, 31.0), (0, 1)),
            // Shorter than the period is the other reading of an off-grid
            // interval: 11 ms is on the grid, 9 ms is not.
            ((11.0, 11.0), (0, 0)),
            ((9.0, 9.0), (0, 1)),
        ];
        for ((wall_ms, grid_ms), expected) in table {
            let samples = paired(wall_ms, grid_ms);
            let found = holes(&samples);
            let counted = (
                found.iter().filter(|hole| hole.step_ms.is_some()).count(),
                found.iter().filter(|hole| hole.gap_ms.is_some()).count(),
            );
            assert_eq!(counted, expected, "wall {wall_ms} ms, grid {grid_ms} ms");
        }
    }

    /// Each wording is one sentence, whole, as an operator reads it off the
    /// note and off the refusal.
    ///
    /// Asserted entire rather than by fragment: a literal re-wrapped across two
    /// source lines carries its indentation into the message, and every
    /// `contains` in this suite sits on one side of where that lands.
    #[test]
    fn every_reading_of_a_hole_is_named_in_one_whole_sentence() {
        let hole = |step_ms, gap_ms| {
            Hole {
                at: 7,
                t_ns: 42,
                step_ms,
                gap_ms,
            }
            .named()
        };
        assert_eq!(
            hole(None, Some(60.0)),
            "a 60 ms gap at t_ns 42 — the recording holds no reading of that stretch"
        );
        assert_eq!(
            hole(Some(30_000.0), None),
            "a wall-clock step of +30000 ms at t_ns 42 — the stamps moved, the readings did not"
        );
        assert_eq!(
            hole(Some(-2_000.0), Some(60.0)),
            "a wall-clock step of -2000 ms across a 60 ms gap at t_ns 42 — the stamps moved and \
             the recording holds no reading of that stretch"
        );
        // An interval shorter than the period is not a length of stream that is
        // missing, and is not called one.
        assert_eq!(
            hole(None, Some(5.0)),
            "two readings 5 ms apart at t_ns 42 — they were stamped inside one 20 ms period, so \
             the grid is not the grid it says it is"
        );
    }

    /// The draft's frame spacing is measured on the grid stamp.
    ///
    /// Asserted here rather than through a fixture because the two clocks
    /// disagreeing inside a stretch is a `spans_hole` refusal, which takes
    /// precedence: this is the only way to see which clock the check itself
    /// reads, and reverting it to the wall stamp is otherwise invisible.
    #[test]
    fn the_drafts_frame_spacing_reads_the_grid_stamp_and_not_the_wall() {
        // Paced on the grid, with the wall stepping a second under it: the
        // draft is one frame per reading and the readings are one apart.
        let stepped = [
            reading(0, 0),
            reading(20_000_000, 20_000_000),
            reading(1_040_000_000, 40_000_000),
        ];
        assert_eq!(off_grid_frame(&stepped, &[0, 1, 2]), None);

        // Off the grid, with the wall clock saying the readings are a period
        // apart: the number the refusal carries is the grid's.
        let missed = [
            reading(0, 0),
            reading(20_000_000, 20_000_000),
            reading(40_000_000, 80_000_000),
        ];
        let (frame, step_ms) = off_grid_frame(&missed, &[0, 1, 2]).expect("a frame off the grid");
        assert_eq!(frame, 2);
        assert!((step_ms - 60.0).abs() < 1e-6, "{step_ms}");
    }

    /// A whole stream from a build whose sample line is not this one is a
    /// finding that says so, not a green-ish report of a session nobody moved
    /// in.
    ///
    /// The grid stamp is required rather than defaulted precisely so a
    /// mismatched payload is loud; an operator told only "no samples" goes
    /// looking at the servo bus.
    #[test]
    fn a_stream_with_one_clock_on_every_sample_is_a_finding_about_the_build() {
        let at = scratch_dir("pose-session-one-clock");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        let text = fixture(&records, &hold, &moved);
        let stripped: Vec<String> = text
            .lines()
            .map(|line| {
                let Some(from) = line.find(r#","mono_ns":"#) else {
                    return line.to_owned();
                };
                let rest = line[from + 1..]
                    .find(',')
                    .expect("a field after the grid stamp");
                format!("{}{}", &line[..from], &line[from + 1 + rest..])
            })
            .collect();
        std::fs::write(
            console_dir(&records).join("recorder_0.log"),
            stripped.join("\n") + "\n",
        )
        .expect("a writable fixture");

        let analysis = judge(&records, SegmentConfig::default());
        assert!(
            analysis
                .report
                .findings
                .iter()
                .any(|finding| finding.contains("would not parse")
                    && finding.contains("line schema")),
            "{:?}",
            analysis.report.findings
        );
        assert!(
            analysis
                .report
                .findings
                .iter()
                .any(|finding| finding.contains("holds no samples")),
            "{:?}",
            analysis.report.findings
        );
    }

    /// A reading the whole bus was quiet for is a hole no stamp list sees, and
    /// the draft's own frame spacing is what catches it — measured on the grid
    /// clock the readings were taken on.
    #[test]
    fn a_reading_no_row_answered_is_refused_on_frame_spacing() {
        let at = scratch_dir("pose-session-unanswered");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        fixture_with(
            &records,
            &hold,
            &moved,
            Extras {
                unanswered_in_move: Some(MOVED / 2),
                ..Extras::default()
            },
        );
        let document = judge(&records, SegmentConfig::default()).document;
        // Nothing about the stamps says anything happened: both clocks ran on
        // the grid across the reading nobody answered, and the stream's only
        // gap is the one the second hold carries.
        assert!(
            document.session.clock_steps.is_empty(),
            "{:?}",
            document.session.clock_steps
        );
        let move_doc = document
            .segments
            .iter()
            .find(|doc| doc.kind == DocKind::Motion)
            .expect("a move");
        assert!(!move_doc.spans_hole, "{move_doc:?}");
        let refused = drafted(&records, &move_doc.id).expect_err("a move with a reading missing");
        assert!(
            refused.contains("not the 20 ms the draft's frames are played at"),
            "{refused}"
        );
        // No remediation advice: no threshold ends a move at an unanswered
        // reading.
        assert!(!refused.contains("Cut a segment"), "{refused}");
    }

    #[test]
    fn the_timeline_is_one_line_per_segment_in_time_order() {
        let at = scratch_dir("pose-session-timeline");
        let document = read(&at, SegmentConfig::default());
        let text = timeline(&document);
        let segments: Vec<&str> = text
            .lines()
            .filter(|line| !line.contains("  SAY "))
            .collect();
        assert_eq!(segments.len(), document.segments.len(), "{text}");
        assert!(segments[0].starts_with("00:00.00  STILL  S001"), "{text}");
        assert!(segments[1].contains("MOVE   S002"), "{text}");
        assert!(segments[2].contains("STILL  S003"), "{text}");
        assert!(segments[0].contains("ant r="), "{text}");
    }

    #[test]
    fn the_configuration_the_analyzer_ran_travels_in_the_document() {
        let at = scratch_dir("pose-session-config");
        let cfg = SegmentConfig {
            min_still: std::time::Duration::from_millis(2500),
            ..SegmentConfig::default()
        };
        let document = read(&at, cfg);
        assert_eq!(document.session.segmenter.min_still_ms, 2500);
        // The fixture's second hold is two seconds long, so a floor above it
        // absorbs it into the move it followed and the session is two segments.
        let kinds: Vec<DocKind> = document.segments.iter().map(|doc| doc.kind).collect();
        assert_eq!(kinds, vec![DocKind::Still, DocKind::Motion], "{kinds:?}");
    }

    #[test]
    fn a_fetch_with_no_pose_stream_is_red() {
        let at = scratch_dir("pose-session-absent");
        let records = at.join("session");
        let report = judge(&records, SegmentConfig::default()).report;
        // Three ways one empty directory is not a session: no stream, no
        // samples in the stream it does not have, and no voice host either.
        assert_eq!(report.findings.len(), 3, "{:?}", report.findings);
        assert!(
            report.findings[0].contains("no pose stream"),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn a_refusal_and_an_empty_stream_are_each_red() {
        let at = scratch_dir("pose-session-refused");
        let records = at.join("session");
        let console = console_dir(&records);
        std::fs::create_dir_all(&console).expect("a scratch console directory");
        std::fs::write(
            console.join("recorder_0.log"),
            written(&PoseLine::Refused(RefusedLine {
                t_ns: 7,
                error: "servo 12 holds torque".to_owned(),
            })) + "\n",
        )
        .expect("a writable fixture");
        let report = judge(&records, SegmentConfig::default()).report;
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("refused")),
            "{:?}",
            report.findings
        );
        assert!(
            report
                .findings
                .iter()
                .any(|finding| finding.contains("no samples")),
            "{:?}",
            report.findings
        );
    }

    #[test]
    fn a_session_nobody_moved_or_spoke_in_is_green() {
        let at = scratch_dir("pose-session-quiet");
        let records = at.join("session");
        let (hold, _) = fixture_poses();
        fixture(&records, &hold, &hold);
        let analysis = judge(&records, SegmentConfig::default());
        assert!(
            analysis.report.findings.is_empty(),
            "{:?}",
            analysis.report.findings
        );
        assert!(
            analysis
                .document
                .segments
                .iter()
                .all(|doc| doc.kind == DocKind::Still),
            "{:?}",
            analysis.document.segments
        );
    }

    #[test]
    fn the_document_and_the_timeline_are_written_where_out_says() {
        let at = scratch_dir("pose-session-out");
        let document = read(&at, SegmentConfig::default());
        let out = at.join("report");
        write_out(&out, &document).expect("a writable output directory");
        let json = std::fs::read_to_string(out.join("session.json")).expect("the document");
        assert!(json.contains("\"segments\""), "{json}");
        assert!(json.contains("\"segmenter\""), "{json}");
        assert!(
            std::fs::read_to_string(out.join("timeline.txt"))
                .expect("the timeline")
                .contains("STILL")
        );
    }

    #[test]
    fn the_flags_are_the_four_thresholds_and_the_output_directory() {
        let parsed = invocation(
            [
                "records",
                "--out",
                "somewhere",
                "--speed-window-ms",
                "120",
                "--still-rad-s",
                "0.02",
                "--moving-rad-s",
                "0.2",
                "--min-still-ms",
                "800",
            ]
            .into_iter()
            .map(str::to_owned),
        )
        .expect("a well-formed command line");
        assert_eq!(parsed.records, Path::new("records"));
        assert_eq!(parsed.out, Path::new("somewhere"));
        assert_eq!(
            parsed.cfg.speed_window,
            std::time::Duration::from_millis(120)
        );
        assert_eq!(parsed.cfg.min_still, std::time::Duration::from_millis(800));
        assert!((parsed.cfg.still_rad_per_s - 0.02).abs() < 1e-12);
        assert!((parsed.cfg.moving_rad_per_s - 0.2).abs() < 1e-12);
        assert!(parsed.extract.is_none(), "no draft was asked for");
    }

    /// The extraction flags, and what each of them defaults to: every channel,
    /// because a recording holds where all three stood, and a derived name.
    #[test]
    fn the_extraction_flags_reach_the_draft_they_describe() {
        let bare = invocation(
            ["records", "--out", "somewhere", "--extract", "S002"]
                .into_iter()
                .map(str::to_owned),
        )
        .expect("a well-formed command line");
        let ask = bare.extract.expect("a draft was asked for");
        assert_eq!(ask.segment, "S002");
        assert_eq!(ask.name, None);
        assert_eq!(ask.channels, ChannelMask::all());

        let named = invocation(
            [
                "records",
                "--out",
                "somewhere",
                "--extract",
                "S002",
                "--name",
                "recorded/wake/raise",
                "--channels",
                "head,antennas",
            ]
            .into_iter()
            .map(str::to_owned),
        )
        .expect("a well-formed command line");
        let ask = named.extract.expect("a draft was asked for");
        assert_eq!(ask.name.as_deref(), Some("recorded/wake/raise"));
        assert_eq!(
            ask.channels,
            ChannelMask::of(Channel::Head).union(ChannelMask::of(Channel::Antennas))
        );
    }

    /// Each refusal names the thing that was wrong.
    ///
    /// Not just that something was: a refusal for the wrong reason is a refusal
    /// that would still pass if the flag it is about were quietly accepted, and
    /// the `--extract` case exists precisely to pin that this tool does not
    /// take that flag yet.
    #[test]
    fn a_command_line_this_tool_cannot_run_says_which_part() {
        for (words, expected) in [
            (vec!["records"], "usage: pose_session_report"),
            (vec!["--out", "somewhere"], "usage: pose_session_report"),
            (
                vec!["records", "--out", "somewhere", "--name", "a/clip"],
                "--name and --channels are --extract's",
            ),
            (
                vec![
                    "records",
                    "--out",
                    "somewhere",
                    "--extract",
                    "S001",
                    "--channels",
                    "legs",
                ],
                "--channels: \"legs\" is not a channel",
            ),
            (
                vec!["records", "--out", "somewhere", "--min-still-ms", "half"],
                "--min-still-ms wants whole milliseconds",
            ),
            (
                vec!["records", "--out", "somewhere", "--still-rad-s", "-1"],
                "--still-rad-s wants a positive number",
            ),
            (
                vec!["records", "--out", "somewhere", "--min-still-ms"],
                "--min-still-ms takes a value",
            ),
            (
                vec![
                    "records",
                    "--out",
                    "somewhere",
                    "--still-rad-s",
                    "0.9",
                    "--moving-rad-s",
                    "0.1",
                ],
                "inverts the hysteresis band",
            ),
        ] {
            let refused = invocation(words.iter().map(|word| (*word).to_owned()));
            let said = refused
                .err()
                .unwrap_or_else(|| panic!("{words:?} was admitted"));
            assert!(said.contains(expected), "{words:?} refused with {said:?}");
        }
    }

    /// The timeline's own clock, which every line of the artefact an operator
    /// reads first is stamped with — and which no fixture session runs long
    /// enough to exercise past five seconds.
    #[test]
    fn the_timeline_stamp_rolls_over_at_a_minute() {
        for (offset_ns, expected) in [
            (0_i64, "00:00.00"),
            (1_500_000_000, "00:01.50"),
            (59_990_000_000, "00:59.99"),
            // Rounded up to the minute rather than printed as a sixtieth
            // second.
            (59_999_000_000, "01:00.00"),
            (60_000_000_000, "01:00.00"),
            (61_500_000_000, "01:01.50"),
            (605_000_000_000, "10:05.00"),
            // A stamp before the session's first sample, which is what a
            // backwards clock step under a run leaves.
            (-3_400_000_000, "-00:03.40"),
        ] {
            assert_eq!(stamp(offset_ns), expected, "{offset_ns} ns");
        }
    }

    #[test]
    fn the_defaults_are_the_crates_own_constants() {
        let parsed = invocation(
            ["records", "--out", "somewhere"]
                .into_iter()
                .map(str::to_owned),
        )
        .expect("a well-formed command line");
        assert_eq!(parsed.cfg, SegmentConfig::default());
    }
    #[test]
    fn a_torn_line_is_noted_and_the_console_around_it_is_not() {
        let at = scratch_dir("pose-session-torn");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        let text = fixture(&records, &hold, &moved);
        let stream = Stream::read(&records);
        assert_eq!((stream.torn, stream.prose), (0, 0));
        // The launcher merges the recorder's stderr into this file, so its
        // opening words to the operator are in it. They are not a torn record
        // and must not raise the count that catches one.
        let mut lines: Vec<String> = text.lines().map(str::to_owned).collect();
        lines.insert(0, "pose-log over /dev/ttyAMA3 at 1000000 baud.".to_owned());
        lines.insert(1, "Ctrl-C ends the stream.".to_owned());
        lines[32].truncate(20);
        std::fs::write(
            console_dir(&records).join("recorder_0.log"),
            lines.join("\n") + "\n",
        )
        .expect("a writable fixture");
        let torn = Stream::read(&records);
        assert!(torn.unreadable.is_none());
        assert_eq!((torn.torn, torn.prose), (1, 2));
        let document = judge(&records, SegmentConfig::default()).document;
        assert!(
            document
                .session
                .notes
                .iter()
                .any(|note| note.contains("torn or truncated")),
            "{:?}",
            document.session.notes
        );
    }

    #[test]
    fn a_stream_recorded_on_another_grid_says_so() {
        let at = scratch_dir("pose-session-grid");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        let text = fixture(&records, &hold, &moved);
        let mut lines: Vec<String> = text.lines().map(str::to_owned).collect();
        lines[0] = lines[0].replace(
            &format!("\"period_ms\":{POSE_LOG_PERIOD_MS}"),
            "\"period_ms\":40",
        );
        std::fs::write(
            console_dir(&records).join("recorder_0.log"),
            lines.join("\n") + "\n",
        )
        .expect("a writable fixture");
        let document = judge(&records, SegmentConfig::default()).document;
        assert!(
            document
                .session
                .notes
                .iter()
                .any(|note| note.contains("40 ms grid")),
            "{:?}",
            document.session.notes
        );
    }

    #[test]
    fn a_baseline_the_segmenter_cannot_measure_over_is_refused_at_the_flag() {
        // The tuning mistake the four flags invite: a window widened past the
        // history, which used to answer with one enormous still segment and a
        // green verdict.
        let refused = invocation(
            ["records", "--out", "somewhere", "--speed-window-ms", "2000"]
                .into_iter()
                .map(str::to_owned),
        );
        let refused = refused.err().expect("a window no history covers");
        assert!(refused.contains("difference baseline"), "{refused}");
    }

    /// The join: the utterance spoken inside the first hold, the one spoken
    /// over the move, and the one whose line never onset.
    #[test]
    fn each_utterance_is_joined_to_the_hold_it_was_spoken_out_of_and_into() {
        let at = scratch_dir("pose-session-said");
        let document = read(&at, SegmentConfig::default());
        let said = &document.utterances;
        assert_eq!(said.len(), 3, "{said:?}");
        let endpointer = Endpointer::default();

        // The stamps walked back by the endpointer's own configuration: the
        // onset run ahead of the chunk that completed it, the hangover ahead of
        // the chunk that closed the utterance.
        assert_eq!(
            said[0].t_start_ns,
            Some((SAID_FIRST.0 - i64::from(endpointer.onset_ms) * 1_000) * 1_000)
        );
        assert_eq!(
            said[0].t_end_ns,
            Some((SAID_FIRST.1 - i64::from(endpointer.soft_hangover_ms) * 1_000) * 1_000)
        );
        assert!(!said[0].start_estimated);
        assert_eq!(said[0].text, "this is the resting pose");
        assert_eq!(said[0].id, Some(1));
        assert_eq!(said[0].no_speech, Some(0.02));

        // Spoken inside the first hold: that hold is what it is about, the move
        // after it is what followed, and the hold the machine settled into is
        // the one it reached.
        assert_eq!(said[0].still_before.as_deref(), Some("S001"));
        assert_eq!(said[0].during, vec!["S001".to_owned()]);
        assert_eq!(said[0].still_after.as_deref(), Some("S003"));

        // Spoken over the move: the move is what it overlapped, and the holds
        // either side are still named.
        assert_eq!(said[1].during, vec!["S002".to_owned()]);
        assert_eq!(said[1].still_before.as_deref(), Some("S001"));
        assert_eq!(said[1].still_after.as_deref(), Some("S003"));

        // A carve that never onset is estimated forward from its first audio
        // and says so, and it is the one the parrot's read-back ran over.
        assert!(said[2].start_estimated);
        assert_eq!(said[2].during, vec!["S003".to_owned()]);
        assert!(said[2].overlaps_playback, "{:?}", said[2]);
        assert!(!said[0].overlaps_playback);
        assert!(!said[1].overlaps_playback);

        // And the derivation travels with them.
        assert_eq!(document.session.endpointer.onset_ms, 96);
        assert_eq!(document.session.endpointer.soft_hangover_ms, 256);
        assert_eq!(document.session.endpointer.preroll_pad_ms, 500);
    }

    /// The endpointer values are the run's own where the fetch carried them.
    #[test]
    fn the_runs_own_endpointer_configuration_is_what_the_intervals_are_derived_under() {
        let at = scratch_dir("pose-session-endpointer");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        fixture(&records, &hold, &moved);
        let config = records.join("config").join("host");
        std::fs::create_dir_all(&config).expect("a scratch config directory");
        std::fs::write(
            config.join("speech-record.toml"),
            "[endpointer]\nonset_chunks = 5\nsoft_hangover_ms = 400\n",
        )
        .expect("a writable fixture");
        let document = judge(&records, SegmentConfig::default()).document;
        assert_eq!(document.session.endpointer.onset_ms, 160);
        // Not the file's 400: the detector counts whole chunks of 32 ms, so
        // the hangover it held was twelve of them. Deriving against the
        // configured figure would put every end stamp of a tuned session 16 ms
        // early, in one direction.
        assert_eq!(document.session.endpointer.soft_hangover_ms, 384);
        // The key the file does not state is the pipeline's own default, not
        // zero: an unstated key is a key the session ran the default under.
        assert_eq!(document.session.endpointer.preroll_pad_ms, 500);
        // And the defaults are the detector's own, not three numbers written
        // out here: a bump of the pin that moved one moves this.
        let ran = speech_pipeline::EndpointerConfig::default();
        let unstated = Endpointer::default();
        assert_eq!(unstated.onset_ms, ran.onset_chunks * 32);
        assert_eq!(unstated.soft_hangover_ms, ran.soft_hangover_chunks * 32);
        assert_eq!(
            document.utterances[0].t_start_ns,
            Some((SAID_FIRST.0 - 160_000) * 1_000)
        );
        // And a configuration this tool read whole says nothing: the note is
        // what a reader takes as "these are not the session's numbers".
        assert!(
            !document
                .session
                .notes
                .iter()
                .any(|note| note.contains("endpointer defaults") || note.contains("[endpointer]")),
            "{:?}",
            document.session.notes
        );
    }

    /// The timeline is one chronology, and a word lands in it beside the hold
    /// it was said over.
    #[test]
    fn the_timeline_carries_the_utterances_between_the_segments() {
        let at = scratch_dir("pose-session-timeline-said");
        let document = read(&at, SegmentConfig::default());
        let text = timeline(&document);
        let lines: Vec<&str> = text.lines().collect();
        let kinds: Vec<&str> = lines
            .iter()
            .map(|line| line.split_whitespace().nth(1).expect("a kind"))
            .collect();
        assert_eq!(
            kinds,
            vec!["STILL", "SAY", "MOVE", "SAY", "STILL", "SAY"],
            "{text}"
        );
        assert!(
            lines[1].contains("\"this is the resting pose\"")
                && lines[1].contains("before=S001")
                && lines[1].contains("after=S003"),
            "{text}"
        );
        assert!(lines[5].contains("over-readback"), "{text}");
    }

    /// The audio: one cut per utterance out of the store the fetch brought
    /// home, and a named reason where there is no store.
    #[test]
    fn every_utterance_is_cut_out_of_the_fetched_store() {
        let at = scratch_dir("pose-session-audio");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        fixture(&records, &hold, &moved);
        audio_store(&records);
        let out = sibling(&records, ".out");
        let analysis = judge(&records, SegmentConfig::default());
        // The reading decodes; the caller writes. What the document names and
        // what a run puts on disk are the same list, which is the property
        // that survives the two halves being separate.
        let mut made = None;
        for cut in &analysis.cuts {
            run_audio::write_cut(&out, &cut.name, &cut.pcm, &mut made).expect("a wav");
        }
        let document = analysis.document;
        assert_eq!(
            document
                .utterances
                .iter()
                .filter_map(|said| said.wav.clone())
                .collect::<Vec<String>>(),
            analysis
                .cuts
                .iter()
                .map(|cut| cut.name.clone())
                .collect::<Vec<String>>()
        );
        for (index, said) in document.utterances.iter().enumerate() {
            assert_eq!(
                said.wav.as_deref(),
                Some(format!("utt-{:03}.wav", index + 1).as_str()),
                "{said:?}"
            );
            assert!(said.wav_error.is_none(), "{said:?}");
            let written = out.join(said.wav.as_deref().expect("a cut"));
            let bytes = std::fs::metadata(&written)
                .unwrap_or_else(|why| panic!("{}: {why}", written.display()))
                .len();
            // Exactly the span the line named, not merely something: the store
            // holds four frames of the same length and each utterance names
            // one of them, so a resolve that returned the whole log, the
            // neighbouring frame, or a span a frame out would satisfy any
            // lower bound and none of these.
            assert_eq!(
                bytes,
                WAV_HEADER + 2 * FRAME as u64,
                "{} holds {bytes} bytes",
                written.display()
            );
            let cut = &analysis.cuts[index];
            assert_eq!(cut.pcm.len(), FRAME, "one frame of samples");
            // The fixture writer numbers each frame's samples from one, so a
            // span starting anywhere but a frame boundary reads back shifted.
            assert_eq!(cut.pcm[0], 1);
            assert_eq!(cut.pcm[FRAME - 1], FRAME as i16);
        }

        // And a fetch whose store never came home says why, once, rather than
        // leaving a reader to wonder whether nobody spoke.
        let at = scratch_dir("pose-session-no-audio");
        let document = read(&at, SegmentConfig::default());
        assert!(document.utterances.iter().all(|said| said.wav.is_none()));
        assert_eq!(
            document.utterances[0].wav_error.as_deref(),
            Some("this fetch brought no audio home")
        );
        assert!(
            document
                .session
                .notes
                .iter()
                .any(|note| note.contains("holds no recorded audio")),
            "{:?}",
            document.session.notes
        );

        // And a store that will not open is neither of those: the fault is
        // said, because a session where the audio was there and unreadable
        // must never read as a session nobody spoke over.
        let at = scratch_dir("pose-session-faulted-store");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        fixture(&records, &hold, &moved);
        std::fs::write(audio_dir(&records), b"not a store").expect("a file where the store goes");
        let document = judge(&records, SegmentConfig::default()).document;
        let why = document.utterances[0]
            .wav_error
            .as_deref()
            .expect("a named fault");
        assert_ne!(why, "this fetch brought no audio home", "{why}");
        assert!(
            document
                .session
                .notes
                .iter()
                .any(|note| note.contains("would not open")),
            "{:?}",
            document.session.notes
        );
    }

    /// A session recorded with nothing transcribing is red, and so is one whose
    /// host console never came home.
    #[test]
    fn a_voice_host_that_never_reported_listening_is_red() {
        let at = scratch_dir("pose-session-deaf");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        fixture_with(
            &records,
            &hold,
            &moved,
            Extras {
                no_voice: true,
                ..Extras::default()
            },
        );
        let analysis = judge(&records, SegmentConfig::default());
        assert!(
            analysis
                .report
                .findings
                .iter()
                .any(|finding| finding.contains("never reported `listening`")),
            "{:?}",
            analysis.report.findings
        );
        assert!(analysis.document.utterances.is_empty());

        // A host that ran and said nothing else is the same verdict: the line
        // is what says the pipeline was transcribing.
        std::fs::write(
            console_dir(&records).join("voice_host_0.log"),
            "reachy-host: composing the speech pipeline\n",
        )
        .expect("a writable fixture");
        let analysis = judge(&records, SegmentConfig::default());
        assert!(
            analysis
                .report
                .findings
                .iter()
                .any(|finding| finding.contains("never reported `listening`")),
            "{:?}",
            analysis.report.findings
        );
    }

    /// An utterance line with no stamp to derive an interval from is kept and
    /// joined to nothing, and it is not in the chronology.
    #[test]
    fn an_utterance_with_no_endpoint_stamp_is_kept_and_joined_to_nothing() {
        let at = scratch_dir("pose-session-stampless");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        fixture(&records, &hold, &moved);
        std::fs::write(
            console_dir(&records).join("voice_host_0.log"),
            format!(
                "{}\n{}\n",
                r#"{"ts_ms":10,"event":"listening","addr":"127.0.0.1:9"}"#,
                r#"{"ts_ms":20,"event":"utterance","id":4,"transcript":{"text":"where am I"}}"#
            ),
        )
        .expect("a writable fixture");
        let document = judge(&records, SegmentConfig::default()).document;
        let said = &document.utterances[0];
        assert_eq!(said.text, "where am I");
        assert_eq!(said.t_start_ns, None);
        assert_eq!(said.t_end_ns, None);
        assert!(said.still_before.is_none() && said.during.is_empty());
        // An utterance line naming no audio is a pipeline problem, and it must
        // not read as a fetch that brought no store home.
        assert!(said.wav.is_none());
        assert_eq!(
            said.wav_error.as_deref(),
            Some("the utterance line names no audio to cut")
        );
        assert!(
            document
                .session
                .notes
                .iter()
                .any(|note| note.contains("carries no stamp")),
            "{:?}",
            document.session.notes
        );
        assert!(!timeline(&document).contains("SAY"));
    }

    /// The reading a limp machine standing exactly at neutral writes: the
    /// cranks that hold the neutral pose, the antennas at their neutral lean,
    /// no yaw.
    fn neutral_reading() -> JointVector {
        let targets = neutral_targets();
        JointVector {
            body_yaw: targets.body_yaw,
            legs: cranks_at(&targets.head_pose_body).legs,
            antennas: targets.antennas,
        }
    }

    /// How far a figure derived from a recorded angle may sit from the angle
    /// that was recorded: the servo reports whole counts, so a reading written
    /// as counts and read back as radians is within half of one.
    const COUNT_RAD: f64 = std::f64::consts::TAU / 4096.0;

    /// How far the neutral fixture's move takes the head, metres.
    ///
    /// Downwards: neutral sits near the top of the linkage's travel, and the
    /// same distance up is a pose the legs do not reach.
    const DRAFT_TRAVEL_M: f64 = -0.03;

    /// A fixture session recorded at neutral and moved from it, so a draft off
    /// it has deltas an arithmetic error would show up in — every channel sits
    /// at the base the extraction subtracts.
    fn neutral_session(at: &Path) {
        let held = neutral_reading();
        let lowered = Isometry3::from_parts(
            (neutral_head_pose().translation.vector
                + nalgebra::Vector3::new(0.0, 0.0, DRAFT_TRAVEL_M))
            .into(),
            neutral_head_pose().rotation,
        );
        let mut moved = held;
        moved.legs = cranks_at(&lowered).legs;
        fixture(at, &held, &moved);
    }

    /// A hold extracts as the one frame the head stood at, and a machine left
    /// where a lift puts it drafts as a clip that does nothing at all: the head
    /// delta is the identity, both antenna deltas are zero and so is the yaw.
    ///
    /// The whole conversion is in this case. An antenna delta of the neutral
    /// lean, a head delta taken in the body frame, or a hold extracted from one
    /// reading rather than the mean of the hold would each show up here as a
    /// motion nobody performed.
    #[test]
    fn a_hold_at_neutral_extracts_as_one_frame_of_no_motion() {
        let at = scratch_dir("pose-session-draft-hold");
        let records = at.join("session");
        neutral_session(&records);
        let drafted = drafted(&records, "S001").expect("the first hold extracts");

        assert_eq!(drafted.file, "clip-S001.json");
        assert_eq!(drafted.doc.name, "recorded/session/s001");
        assert_eq!(drafted.doc.frames.len(), 1, "a pose is one frame");
        assert_eq!(
            drafted.doc.channels,
            vec![Channel::Head, Channel::BodyYaw, Channel::Antennas],
            "every channel, in document order"
        );
        let frame = &drafted.doc.frames[0];
        let antennas = frame.antennas.expect("the pair is masked");
        for (side, delta) in antennas.into_iter().enumerate() {
            assert!(
                delta.abs() < COUNT_RAD,
                "antenna {side} drafted as {delta} rad off its own rest"
            );
        }
        let yaw = frame.body_yaw.expect("yaw is masked");
        assert!(yaw.abs() < COUNT_RAD, "yaw drafted as {yaw} rad");
        let dt = frame.dt.expect("the head is masked");
        for axis in dt {
            assert!(
                axis.abs() < 1e-3,
                "the head drafted as {dt:?} m off neutral"
            );
        }
        let dq = frame.dq.expect("the head is masked");
        let rotation = nalgebra::UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
            dq[0], dq[1], dq[2], dq[3],
        ));
        assert!(
            rotation.angle().to_degrees() < 0.5,
            "the head drafted as {:.2}° off square",
            rotation.angle().to_degrees()
        );
        reachy_clips::format::Clip::from_doc(
            drafted.doc,
            &reachy_clips::envelope::ClipLimits::default(),
        )
        .expect("a draft of no motion loads");
    }

    /// A move extracts as one frame per reading of it, in order, and the file
    /// beside the session document is that document verbatim — the operator
    /// copies it into a library directory as it stands.
    #[test]
    fn a_move_extracts_as_one_frame_per_reading_and_lands_beside_the_document() {
        let at = scratch_dir("pose-session-draft-move");
        let records = at.join("session");
        neutral_session(&records);
        let analysis = judge_extracting(
            &records,
            SegmentConfig::default(),
            Some(&Extract {
                segment: "S002".to_owned(),
                name: Some("recorded/wake/raise".to_owned()),
                channels: ChannelMask::all(),
            }),
        );
        let drafted = analysis
            .extraction
            .expect("an extraction was asked for")
            .expect("the move extracts");
        let moved = analysis
            .document
            .segments
            .iter()
            .find(|doc| doc.id == "S002")
            .expect("the second segment");
        assert_eq!(moved.kind, DocKind::Motion);
        assert_eq!(
            drafted.doc.frames.len(),
            moved.frames,
            "one frame per reading of the move"
        );
        assert_eq!(drafted.doc.name, "recorded/wake/raise", "--name is used");
        assert!(
            drafted
                .doc
                .description
                .as_deref()
                .is_some_and(|said| said.contains("S002")),
            "{:?}",
            drafted.doc.description
        );
        // The travel is in the frames: the last one is 30 mm below the first.
        let height =
            |frame: &reachy_clips::format::FrameDoc| frame.dt.expect("the head is masked")[2];
        let travelled = height(drafted.doc.frames.last().expect("a last frame"))
            - height(&drafted.doc.frames[0]);
        assert!(
            (travelled - DRAFT_TRAVEL_M).abs() < 5e-3,
            "the draft travels {travelled} m, expected {DRAFT_TRAVEL_M}"
        );

        let out = at.join("out");
        write_draft(&out, &drafted).expect("a writable output directory");
        let written = std::fs::read_to_string(out.join(&drafted.file)).expect("the draft is there");
        let parsed: reachy_clips::format::ClipDoc =
            serde_json::from_str(&written).expect("the file is the document it was built from");
        assert_eq!(parsed, drafted.doc);
    }

    /// A stretch of a move the solver could not follow refuses the draft, by
    /// the frame it went missing in and the channel that needed it — and the
    /// same stretch extracts fine with the head left out of the mask, which is
    /// the operator's way past it.
    #[test]
    fn a_frame_with_no_head_pose_refuses_the_draft_and_names_it() {
        let at = scratch_dir("pose-session-draft-unsolved");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        fixture_with(
            &records,
            &hold,
            &moved,
            Extras {
                unsolvable: Some((10, 10)),
                ..Extras::default()
            },
        );
        let refused = drafted(&records, "S002").expect_err("a move with a hole in it");
        assert!(
            refused.contains("S002 cannot be extracted") && refused.contains("no reading of head"),
            "{refused:?}"
        );
        assert!(
            refused.contains("frame 1"),
            "the frame is named: {refused:?}"
        );

        let without_head = judge_extracting(
            &records,
            SegmentConfig::default(),
            Some(&Extract {
                segment: "S002".to_owned(),
                name: None,
                channels: ChannelMask::of(Channel::Antennas),
            }),
        )
        .extraction
        .expect("an extraction was asked for")
        .expect("the antennas were read throughout");
        assert_eq!(without_head.doc.channels, vec![Channel::Antennas]);
        assert!(
            without_head
                .doc
                .frames
                .iter()
                .all(|frame| frame.dt.is_none())
        );
    }

    /// A segment id this session does not hold, and a name the library would
    /// not file: each refused by what is wrong with it, with the session
    /// document still written for the operator to pick another id off.
    #[test]
    fn a_draft_nobody_could_file_is_refused_by_what_is_wrong_with_it() {
        let at = scratch_dir("pose-session-draft-refusals");
        let records = at.join("session");
        neutral_session(&records);
        let refused = drafted(&records, "S099").expect_err("no such segment");
        assert!(
            refused.contains("not a segment of this session") && refused.contains("S003"),
            "{refused:?}"
        );

        let named = judge_extracting(
            &records,
            SegmentConfig::default(),
            Some(&Extract {
                segment: "S001".to_owned(),
                name: Some("Wake/Raise".to_owned()),
                channels: ChannelMask::all(),
            }),
        )
        .extraction
        .expect("an extraction was asked for")
        .expect_err("a name the library refuses");
        assert!(
            named.contains("unusable") && named.contains("--name"),
            "{named:?}"
        );
    }

    /// The pipeline announcing that it is transcribing, which every fixture
    /// console opens with.
    const LISTENING_LINE: &str = r#"{"ts_ms":10,"event":"listening","addr":"127.0.0.1:9"}"#;

    /// One playback event of the pod's, as the router emits one.
    fn playback(event: &str, at_ms: i64) -> String {
        format!(r#"{{"ts_ms":{at_ms},"event":"{event}","pod":"reachy00","utterance":2}}"#)
    }

    /// Replace a fixture session's voice console with lines of a case's own.
    fn voice_lines(at: &Path, lines: &[String]) {
        let console = console_dir(at);
        std::fs::create_dir_all(&console).expect("a scratch console directory");
        std::fs::write(console.join("voice_host_0.log"), lines.join("\n") + "\n")
            .expect("a writable fixture");
    }

    /// A fixture session whose speech is whatever the case says it is.
    fn session_saying(at: &Scratch, name: &str, lines: &[String]) -> Document {
        let records = at.join(name);
        let (hold, moved) = fixture_poses();
        fixture(&records, &hold, &moved);
        voice_lines(&records, lines);
        judge(&records, SegmentConfig::default()).document
    }

    /// The stamps one utterance is written with to land a speech interval where
    /// a case wants it: the endpointer's own walk-back, undone.
    fn stamps_for(t_start_ns: i64, t_end_ns: i64) -> (i64, i64) {
        let endpointer = Endpointer::default();
        (
            t_start_ns / 1_000 + i64::from(endpointer.onset_ms) * 1_000,
            t_end_ns / 1_000 + i64::from(endpointer.soft_hangover_ms) * 1_000,
        )
    }

    /// A read-back the operator spoke over ends on a flush, not a finish, and
    /// the utterance that cut it is the one the flag exists for.
    ///
    /// Closing a playback on `playback_finished` alone leaves a barged one open
    /// at zero length, and the barge — the correction the LLM turn reads — is
    /// then the one utterance written `overlaps_playback: false`.
    #[test]
    fn a_read_back_the_operator_spoke_over_is_closed_by_the_flush() {
        let at = scratch_dir("pose-session-barge");
        let barge = stamps_for(1_100_000_000, 1_250_000_000);
        let after = stamps_for(2_900_000_000, 2_950_000_000);
        let document = session_saying(
            &at,
            "session",
            &[
                LISTENING_LINE.to_owned(),
                playback("playback_started", 1_000),
                utterance(
                    1,
                    "no, the other one",
                    barge.0 - 300_000,
                    Some(barge.0),
                    barge.1,
                ),
                playback("playback_flushed", 1_600),
                utterance(
                    2,
                    "this is the resting pose",
                    after.0 - 300_000,
                    Some(after.0),
                    after.1,
                ),
                playback("playback_started", 4_000),
                playback("playback_finished", 4_200),
            ],
        );
        let said = &document.utterances;
        assert_eq!(said.len(), 2, "{said:?}");
        assert!(
            said[0].overlaps_playback,
            "the barge is what the flag is for: {:?}",
            said[0]
        );
        // Spoken between the flush and the next read-back: the flushed playback
        // ended where the flush says it did, and does not swallow the session.
        assert!(!said[1].overlaps_playback, "{:?}", said[1]);
        let aborted = session_saying(
            &at,
            "aborted",
            &[
                LISTENING_LINE.to_owned(),
                playback("playback_started", 1_000),
                playback("playback_aborted", 1_600),
                utterance(
                    1,
                    "this is the resting pose",
                    after.0 - 300_000,
                    Some(after.0),
                    after.1,
                ),
            ],
        );
        assert!(
            !aborted.utterances[0].overlaps_playback,
            "{:?}",
            aborted.utterances[0]
        );
    }

    /// The ordinary end of a recording session: ^C while the parrot is still
    /// talking. The playback runs to the end of the record, an utterance inside
    /// it is flagged, and the reader says which reading it made.
    #[test]
    fn a_session_stopped_over_the_read_back_extends_the_playback_and_says_so() {
        let at = scratch_dir("pose-session-stopped-over");
        let over = stamps_for(1_400_000_000, 1_500_000_000);
        let document = session_saying(
            &at,
            "session",
            &[
                LISTENING_LINE.to_owned(),
                playback("playback_started", 1_000),
                utterance(1, "no, down a bit", over.0 - 300_000, Some(over.0), over.1),
            ],
        );
        assert!(
            document.utterances[0].overlaps_playback,
            "{:?}",
            document.utterances[0]
        );
        assert!(
            document
                .session
                .notes
                .iter()
                .any(|note| note.contains("still running")),
            "{:?}",
            document.session.notes
        );

        // A closer with no start ahead of it is a record whose head went
        // missing: it is where the playback ended and nothing is claimed about
        // what was spoken before it.
        let headless = session_saying(
            &at,
            "headless",
            &[
                LISTENING_LINE.to_owned(),
                utterance(
                    1,
                    "this is the resting pose",
                    over.0 - 300_000,
                    Some(over.0),
                    over.1,
                ),
                playback("playback_finished", 3_000),
            ],
        );
        assert!(
            !headless.utterances[0].overlaps_playback,
            "{:?}",
            headless.utterances[0]
        );
        assert!(
            !headless
                .session
                .notes
                .iter()
                .any(|note| note.contains("still running")),
            "{:?}",
            headless.session.notes
        );
    }

    /// A carve shorter than the hangover ahead of it is an instant, not an
    /// interval that runs backwards.
    ///
    /// The join's three windows are written against a forward interval: fed a
    /// reversed one they answer with plausible segment ids that are not the
    /// ones the operator spoke over.
    #[test]
    fn an_interval_that_would_end_before_it_starts_is_an_instant() {
        let at = scratch_dir("pose-session-inverted");
        // A hundred milliseconds of speech closed after a hangover of 256: the
        // walk-back takes the end further than the start.
        let document = session_saying(
            &at,
            "session",
            &[
                LISTENING_LINE.to_owned(),
                utterance(1, "stop", 1_700_000, Some(2_000_000), 2_100_000),
            ],
        );
        let said = &document.utterances[0];
        assert_eq!(said.t_start_ns, said.t_end_ns, "{said:?}");
        assert_eq!(
            said.t_start_ns,
            Some((2_000_000 - i64::from(Endpointer::default().onset_ms) * 1_000) * 1_000)
        );
        // And the join is the instant's: the hold it was spoken inside, once.
        assert_eq!(said.during, vec!["S001".to_owned()], "{said:?}");
    }

    /// The join windows exclude as well as include: a word said minutes from
    /// any hold is joined to none of them, and a hold that opens just after the
    /// operator started talking is the one they meant.
    #[test]
    fn a_word_said_far_from_every_hold_is_joined_to_none_of_them() {
        let at = scratch_dir("pose-session-join-window");
        // Twenty seconds into a session whose last sample is under six: past
        // every window, in both directions.
        let far = stamps_for(20_000_000_000, 20_400_000_000);
        let document = session_saying(
            &at,
            "far",
            &[
                LISTENING_LINE.to_owned(),
                utterance(1, "still here", far.0 - 300_000, Some(far.0), far.1),
            ],
        );
        let said = &document.utterances[0];
        assert_eq!(said.still_before, None, "{said:?}");
        assert_eq!(said.still_after, None, "{said:?}");
        assert!(said.during.is_empty(), "{said:?}");

        // The lead: the operator labels a pose while the head is still settling
        // under their hands, so a hold that opens within HOLD_LEAD_NS of the
        // first word is the hold that word is about.
        let settled = session_saying(&at, "settled", &[LISTENING_LINE.to_owned()]);
        let second = settled
            .segments
            .iter()
            .find(|doc| doc.id == "S003")
            .expect("the hold after the move")
            .t0_ns;
        for (offset, expected) in [(400_000_000_i64, "S003"), (600_000_000, "S001")] {
            let spoken = stamps_for(second - offset, second - offset + 100_000_000);
            let document = session_saying(
                &at,
                &format!("lead-{offset}"),
                &[
                    LISTENING_LINE.to_owned(),
                    utterance(
                        1,
                        "and this one",
                        spoken.0 - 300_000,
                        Some(spoken.0),
                        spoken.1,
                    ),
                ],
            );
            assert_eq!(
                document.utterances[0].still_before.as_deref(),
                Some(expected),
                "{} ns before the hold opened: {:?}",
                offset,
                document.utterances[0]
            );
        }
    }

    /// An endpointer configuration this tool cannot read runs the pipeline's
    /// own defaults and says which reading it made.
    ///
    /// Silence here is a document whose every speech interval is derived
    /// against values the session never ran, printed under `session.endpointer`
    /// as if they had been measured.
    #[test]
    fn an_endpointer_configuration_this_tool_cannot_read_runs_the_defaults_and_says_so() {
        let at = scratch_dir("pose-session-endpointer-faults");
        let written = |name: &str, text: &str| -> Document {
            let records = at.join(name);
            let (hold, moved) = fixture_poses();
            fixture(&records, &hold, &moved);
            let config = records.join("config").join("host");
            std::fs::create_dir_all(&config).expect("a scratch config directory");
            std::fs::write(config.join("speech-record.toml"), text).expect("a writable fixture");
            judge(&records, SegmentConfig::default()).document
        };
        let defaults = Endpointer::default();

        // A copy the fetch caught mid-write.
        let torn = written("torn", "[endpointer\nonset_chunks = 5\n");
        assert_eq!(torn.session.endpointer.onset_ms, defaults.onset_ms);
        assert!(
            torn.session
                .notes
                .iter()
                .any(|note| note.contains("could not be parsed")
                    && note.contains("speech-record.toml")),
            "{:?}",
            torn.session.notes
        );

        // A value the surface itself would have refused: the host types all
        // three as whole numbers, so a session never ran this file.
        let typed = written(
            "mistyped",
            "[endpointer]\nonset_chunks = 5\nsoft_hangover_ms = \"400\"\n",
        );
        assert_eq!(
            typed.session.endpointer.onset_ms, 160,
            "the key it could read"
        );
        assert_eq!(
            typed.session.endpointer.soft_hangover_ms, defaults.soft_hangover_ms,
            "the one it could not"
        );
        assert!(
            typed
                .session
                .notes
                .iter()
                .any(|note| note.contains("soft_hangover_ms") && note.contains("[endpointer]")),
            "{:?}",
            typed.session.notes
        );

        // And a fetch carrying no copy at all is the third reading, said in its
        // own words rather than left to look like a configured session.
        let absent = scratch_dir("pose-session-endpointer-absent");
        let document = read(&absent, SegmentConfig::default());
        assert!(
            document
                .session
                .notes
                .iter()
                .any(|note| note.contains("speech-record.toml")
                    && note.contains("endpointer defaults")),
            "{:?}",
            document.session.notes
        );
    }

    /// Each way a cut can fail to happen is said in its own words: a fetch
    /// problem and a pipeline problem must not read the same.
    #[test]
    fn an_utterance_whose_audio_the_store_does_not_hold_says_the_resolvers_words() {
        let at = scratch_dir("pose-session-pruned");
        let records = at.join("session");
        let (hold, moved) = fixture_poses();
        fixture(&records, &hold, &moved);
        audio_store(&records);
        let kept = stamps_for(1_000_000_000, 1_200_000_000);
        let pruned = stamps_for(2_000_000_000, 2_200_000_000);
        voice_lines(
            &records,
            &[
                LISTENING_LINE.to_owned(),
                utterance(
                    1,
                    "this one is here",
                    kept.0 - 300_000,
                    Some(kept.0),
                    kept.1,
                ),
                // The same line, naming a log this fetch's store does not hold.
                utterance(
                    2,
                    "this one was pruned",
                    pruned.0 - 300_000,
                    Some(pruned.0),
                    pruned.1,
                )
                .replace(FIXTURE_LOG, "gone.framelog"),
            ],
        );
        let document = judge(&records, SegmentConfig::default()).document;
        assert_eq!(document.utterances[0].wav.as_deref(), Some("utt-001.wav"));
        assert!(document.utterances[0].wav_error.is_none());
        let why = document.utterances[1]
            .wav_error
            .as_deref()
            .expect("a named reason");
        assert!(document.utterances[1].wav.is_none());
        assert!(why.contains("gone.framelog"), "{why}");
        assert_ne!(why, "this fetch brought no audio home", "{why}");
    }

    /// A move spanning a hole is not extracted.
    ///
    /// The draft is one frame per reading at a fixed rate, so a stretch the
    /// recording holds no reading of would be crossed in one tick, at a speed
    /// nobody commanded, in a clip that loads.
    #[test]
    fn a_move_spanning_a_hole_is_not_extracted() {
        let at = scratch_dir("pose-session-draft-hole");
        let records = at.join("session");
        let held = neutral_reading();
        let lowered = Isometry3::from_parts(
            (neutral_head_pose().translation.vector
                + nalgebra::Vector3::new(0.0, 0.0, DRAFT_TRAVEL_M))
            .into(),
            neutral_head_pose().rotation,
        );
        let mut moved = held;
        moved.legs = cranks_at(&lowered).legs;
        fixture_with(
            &records,
            &held,
            &moved,
            Extras {
                hole_in_move: Some(MOVED / 2),
                ..Extras::default()
            },
        );
        // Refused for spanning the hole, by the entry's own name, rather than
        // by the frame-spacing check further in: one rule for both kinds of
        // segment and both kinds of hole.
        let refused = drafted(&records, "S002").expect_err("a move with a hole in it");
        assert!(
            refused.contains("cannot be extracted") && refused.contains("a 60 ms gap at t_ns"),
            "{refused}"
        );
        drafted(&records, "S001").expect("the hold before the hole extracts");

        // The timeline is where an operator picks the id off before asking for
        // a draft, so a move that spans a hole says so there too — and the id
        // stays the bare token `--extract` takes.
        let document = judge(&records, SegmentConfig::default()).document;
        let text = timeline(&document);
        let marked: Vec<&str> = text
            .lines()
            .filter(|line| line.ends_with("  HOLE"))
            .collect();
        // The move, and the second hold the fixture's own hole falls in.
        assert_eq!(marked.len(), 2, "{text}");
        assert!(marked[0].contains("MOVE   S002  "), "{text}");
        assert!(!text.contains("S002 HOLE"), "{text}");
    }

    /// The whole invocation: what it writes, in what order, and what it exits.
    ///
    /// A mistyped `--extract` id still leaves the document whose ids are how an
    /// operator picks the right one, and still fails the run. Neither half is
    /// reachable through `analyze`.
    #[test]
    fn a_refused_extraction_leaves_the_session_document_and_fails_the_run() {
        let at = scratch_dir("pose-session-run");
        let records = at.join("session");
        neutral_session(&records);
        audio_store(&records);
        let out = at.join("out");
        let asked = |segment: &str| Invocation {
            records: records.clone(),
            out: out.clone(),
            cfg: SegmentConfig::default(),
            extract: Some(Extract {
                segment: segment.to_owned(),
                name: None,
                channels: ChannelMask::all(),
            }),
        };

        assert!(!run(&asked("S099")), "a segment this session has no id for");
        assert!(
            out.join("session.json").is_file(),
            "the document is written"
        );
        assert!(
            out.join("timeline.txt").is_file(),
            "and the timeline with it"
        );
        assert!(
            !out.join("clip-S099.json").exists(),
            "and no draft of a segment that is not there"
        );

        assert!(run(&asked("S001")), "the hold this session does hold");
        assert!(out.join("clip-S001.json").is_file(), "the draft is written");
        let document: serde_json::Value = serde_json::from_str(
            &std::fs::read_to_string(out.join("session.json")).expect("a document"),
        )
        .expect("a document of JSON");
        for said in document["utterances"].as_array().expect("the utterances") {
            let wav = said["wav"].as_str().expect("a cut");
            assert!(out.join(wav).is_file(), "{wav} is named and not there");
        }
    }
}
