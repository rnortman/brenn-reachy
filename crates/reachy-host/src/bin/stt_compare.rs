//! Ask the recogniser the same turn twice: the whole carve, and the carve from
//! the wake-word boundary.
//!
//! Expects paired clips per turn: `turn-NN.wav` (the whole carve) and
//! `turn-NN.command.wav` (the carve from the wake-word boundary). The pairing
//! is by file name; this tool parses no records.
//!
//! Offline, after the fact, from the run the fetch brought home. A live
//! dual-send would double the recogniser's load on every turn to answer a
//! question that is asked once; the clips are already on disk.
//!
//! Uses the same `[stt]` configuration and `HttpTranscriber` path the pipeline
//! sends through; a second HTTP path would answer a question about itself.
//!
//! Talks to the network, which is why it is a separate binary from the report.

#![forbid(unsafe_code)]

use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::Arc;

use futures::StreamExt;
use speech_pipeline::{
    HttpTranscriber, SegmentAudio, SttParams, SttStats, Transcriber, Transcript,
};
use speech_surface::config::Config;
use speech_surface::load_clip;

/// The suffix on the whole-carve clip of a turn.
const WHOLE_SUFFIX: &str = ".wav";

/// The suffix on the same turn's recogniser-facing clip.
const COMMAND_SUFFIX: &str = ".command.wav";

/// What the invocation asks for: a speech configuration to read `[stt]` from,
/// and the clip directory the report wrote.
#[derive(Debug)]
struct Options {
    /// The configuration the run used, for the recogniser endpoint and model.
    speech_config: PathBuf,
    /// The `<run>.turns` directory `//cogs:speech_run_report` wrote the clips
    /// into.
    turns: PathBuf,
}

/// How to invoke this, for a refusal to print.
fn usage() -> String {
    "usage: stt_compare --speech-config PATH <run>.turns\n\
     \n\
     Transcribes every turn clip in the directory twice -- the whole carve and the\n\
     carve from the wake-word boundary -- and prints both transcripts on one line per\n\
     turn, with a count of the turns whose two readings differ.\n\
     \n\
     --speech-config names the speech configuration the run was recorded under. Its\n\
     [stt] table is the recogniser this asks; a configuration without one, or one\n\
     naming an endpoint that does not answer, is a refusal and a nonzero exit.\n\
     \n\
     The directory is the one //cogs:speech_run_report writes beside the fetched\n\
     records. A turn with no turn-NN.command.wav beside its turn-NN.wav is a turn\n\
     whose boundary the records never stated; it is listed and not asked."
        .to_owned()
}

/// One turn's two clips, paired by name.
#[derive(Debug, PartialEq, Eq)]
struct Pair {
    /// What the turn is called on the line -- the clip's name without `.wav`.
    label: String,
    /// The whole carve.
    whole: PathBuf,
    /// The carve from the wake-word boundary, absent when the records never
    /// stated one.
    command: Option<PathBuf>,
}

/// What one turn came to.
#[derive(Debug)]
enum Compared {
    /// Both clips were asked, and this is what came back.
    Both {
        label: String,
        whole: Transcript,
        command: Transcript,
    },
    /// The turn had no second clip, so there was nothing to compare against.
    NoBoundary { label: String },
}

fn main() -> ExitCode {
    let options = match parse(std::env::args().skip(1)) {
        Ok(options) => options,
        Err(message) => {
            eprintln!("{}{message}\n\n{}", reachy_host::REFUSAL_PREFIX, usage());
            return ExitCode::FAILURE;
        }
    };
    match run(&options) {
        Ok(report) => {
            print!("{report}");
            ExitCode::SUCCESS
        }
        Err(message) => {
            eprintln!("{}{message}", reachy_host::REFUSAL_PREFIX);
            ExitCode::FAILURE
        }
    }
}

/// One of four copies of this argument-parsing shape; the host, the driver and
/// the harness carry the others.
/// TODO(cli-argv-shared)
fn parse(args: impl Iterator<Item = String>) -> Result<Options, String> {
    let mut speech_config = None;
    let mut turns = None;
    let mut args = args;
    while let Some(word) = args.next() {
        match word.as_str() {
            "--speech-config" => {
                let value = args.next().ok_or_else(|| {
                    "--speech-config needs the path of a configuration".to_owned()
                })?;
                if speech_config.is_some() {
                    return Err("--speech-config was given twice".to_owned());
                }
                speech_config = Some(PathBuf::from(value));
            }
            other if other.starts_with("--") => {
                return Err(format!("`{other}` is not an option this takes"));
            }
            other => {
                if turns.is_some() {
                    return Err("only one clip directory is read at a time".to_owned());
                }
                turns = Some(PathBuf::from(other));
            }
        }
    }
    Ok(Options {
        speech_config: speech_config
            .ok_or_else(|| "--speech-config names the configuration the run used".to_owned())?,
        turns: turns.ok_or_else(|| "no clip directory was named".to_owned())?,
    })
}

/// The runtime is built here rather than by an attribute so a failure to build
/// one is a refusal in the same shape as every other: this tool's async is one
/// request at a time, and a current-thread runtime is the whole of it.
fn run(options: &Options) -> Result<String, String> {
    let transcriber = transcriber(&options.speech_config)?;
    let pairs = pairs(&options.turns)?;
    let runtime = tokio::runtime::Builder::new_current_thread()
        .enable_all()
        .build()
        .map_err(|why| format!("no async runtime to ask the recogniser through: {why}"))?;
    let compared = runtime.block_on(compare(&transcriber, &pairs))?;
    Ok(render(&compared))
}

/// The recogniser the configuration's `[stt]` table names.
///
/// An absent table is a refusal: the run may well have been recorded without a
/// transcriber, but this tool has nothing to ask then, and a table of turns
/// with no transcripts in it would read as a recogniser that heard nothing.
fn transcriber(config: &Path) -> Result<HttpTranscriber, String> {
    let config = Config::load(config).map_err(|why| format!("{why}"))?;
    let stt = config
        .stt
        .ok_or_else(|| "this configuration has no [stt] table to ask".to_owned())?;
    let params = SttParams {
        url: stt
            .url
            .ok_or_else(|| "[stt] names no url to ask".to_owned())?,
        model: stt
            .model
            .ok_or_else(|| "[stt] names no model to ask for".to_owned())?,
        language: stt.language,
        timeout: std::time::Duration::from_millis(stt.timeout_ms),
        connect_timeout: std::time::Duration::from_millis(stt.connect_timeout_ms),
    };
    HttpTranscriber::new(params, Arc::new(SttStats::default()))
        .map_err(|why| format!("the recogniser this configuration names will not build: {why}"))
}

/// Every turn clip in `dir`, each with its boundary clip where there is one.
///
/// Named by construction: a `.command.wav` is the sibling of the `.wav` whose
/// name it extends, so the pairing needs no record and cannot pair two runs'
/// files. Unrecognised files are ignored.
fn pairs(dir: &Path) -> Result<Vec<Pair>, String> {
    let entries = std::fs::read_dir(dir).map_err(|why| format!("{}: {why}", dir.display()))?;
    let mut names: Vec<String> = Vec::new();
    for entry in entries {
        let entry = entry.map_err(|why| format!("{}: {why}", dir.display()))?;
        let name = entry.file_name().to_string_lossy().into_owned();
        if name.ends_with(WHOLE_SUFFIX) && !name.ends_with(COMMAND_SUFFIX) {
            names.push(name);
        }
    }
    // Read order is the filesystem's; the turns' order is the run's, and the
    // names carry it.
    names.sort();
    Ok(names
        .into_iter()
        .map(|name| {
            let label = name[..name.len() - WHOLE_SUFFIX.len()].to_owned();
            let command = dir.join(format!("{label}{COMMAND_SUFFIX}"));
            Pair {
                whole: dir.join(&name),
                command: command.exists().then_some(command),
                label,
            }
        })
        .collect())
}

/// Ask the recogniser about both clips of every pair, in the turns' order.
///
/// Sequential on purpose: the recogniser is one container an operator is
/// borrowing, and a comparison that saturates it says less about the clips than
/// about the queue. A clip that will not load or a request that fails ends the
/// run -- half a table would be read as a recogniser that disagreed with
/// itself.
async fn compare(transcriber: &dyn Transcriber, pairs: &[Pair]) -> Result<Vec<Compared>, String> {
    let mut compared = Vec::with_capacity(pairs.len());
    for pair in pairs {
        let Some(command) = &pair.command else {
            compared.push(Compared::NoBoundary {
                label: pair.label.clone(),
            });
            continue;
        };
        let whole = heard(transcriber, &pair.whole).await?;
        let command = heard(transcriber, command).await?;
        compared.push(Compared::Both {
            label: pair.label.clone(),
            whole,
            command,
        });
    }
    Ok(compared)
}

async fn heard(transcriber: &dyn Transcriber, clip: &Path) -> Result<Transcript, String> {
    let pcm = load_clip(clip).map_err(|why| format!("{why}"))?;
    transcribe(transcriber, pcm)
        .await
        .map_err(|why| format!("{}: {why}", clip.display()))
}

/// The stream contract ends the stream at the first `is_final` event, so that
/// event is the answer. This must drain the stream the way production does; a
/// comparison that reads differently would disagree for a reason the table does
/// not show. A stream that ends with neither a final event nor an error is an
/// implementation bug, not an empty transcript.
///
/// TODO(stt-compare-shared-drain)
async fn transcribe(transcriber: &dyn Transcriber, pcm: Arc<[i16]>) -> Result<Transcript, String> {
    let audio = SegmentAudio {
        pcm,
        sample_rate_hz: speech_pipeline::SPINE_FORMAT.sample_rate_hz,
    };
    let mut stream = transcriber.transcribe(audio);
    while let Some(event) = stream.next().await {
        let event = event.map_err(|why| format!("{why}"))?;
        if event.is_final {
            return Ok(Transcript {
                text: event.text,
                confidence: event.confidence,
            });
        }
    }
    Err("the recogniser's stream ended with no transcript".to_owned())
}

fn render(compared: &[Compared]) -> String {
    let mut out = String::new();
    for turn in compared {
        out.push_str(&line(turn));
        out.push('\n');
    }
    out.push_str(&footer(compared));
    out.push('\n');
    out
}

fn line(turn: &Compared) -> String {
    match turn {
        Compared::Both {
            label,
            whole,
            command,
        } => format!(
            "{label}  whole: {} | command: {}",
            said(whole),
            said(command)
        ),
        Compared::NoBoundary { label } => format!("{label}  no boundary"),
    }
}

fn said(heard: &Transcript) -> String {
    match &heard.confidence {
        Some(confidence) => format!(
            "{:?} no_speech={:.2} logprob={:.2}",
            heard.text, confidence.no_speech_prob, confidence.avg_logprob
        ),
        None => format!("{:?} no confidence reported", heard.text),
    }
}

fn footer(compared: &[Compared]) -> String {
    let asked = compared
        .iter()
        .filter(|turn| matches!(turn, Compared::Both { .. }))
        .count();
    let differ = compared
        .iter()
        .filter(|turn| match turn {
            Compared::Both { whole, command, .. } => differs(&whole.text, &command.text),
            Compared::NoBoundary { .. } => false,
        })
        .count();
    let unpaired = compared.len() - asked;
    let mut out = format!("{asked} turn(s) compared, {differ} of them read differently");
    if unpaired > 0 {
        out.push_str(&format!("; {unpaired} with no boundary"));
    }
    out.push('.');
    out
}

/// Folded first: the question is whether the wake word changed what was heard,
/// and a comma or a capital is not that.
fn differs(whole: &str, command: &str) -> bool {
    fold(whole) != fold(command)
}

fn fold(text: &str) -> String {
    text.split_whitespace()
        .map(|word| {
            word.chars()
                .filter(|c| c.is_alphanumeric())
                .flat_map(char::to_lowercase)
                .collect::<String>()
        })
        .filter(|word| !word.is_empty())
        .collect::<Vec<String>>()
        .join(" ")
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use futures::stream::{self, BoxStream};
    use reachy_scratch::scratch_dir;
    use speech_pipeline::{
        TranscribeError, TranscriptConfidence, TranscriptEvent, write_spine_wav,
    };

    use super::*;

    /// A recogniser that answers from a script, in the order it is asked, and
    /// remembers how many samples each question carried.
    struct Scripted {
        answers: Mutex<Vec<TranscriptEvent>>,
        asked: Mutex<Vec<usize>>,
    }

    impl Scripted {
        fn new(answers: Vec<&str>) -> Self {
            Self {
                answers: Mutex::new(
                    answers
                        .into_iter()
                        .rev()
                        .map(|text| TranscriptEvent {
                            text: text.to_owned(),
                            is_final: true,
                            confidence: Some(TranscriptConfidence {
                                avg_logprob: -0.31,
                                no_speech_prob: 0.04,
                                compression_ratio: 1.2,
                                segments: 1,
                            }),
                        })
                        .collect(),
                ),
                asked: Mutex::new(Vec::new()),
            }
        }
    }

    impl Transcriber for Scripted {
        fn transcribe(
            &self,
            audio: SegmentAudio,
        ) -> BoxStream<'static, Result<TranscriptEvent, TranscribeError>> {
            self.asked.lock().expect("the tally").push(audio.pcm.len());
            let next = self.answers.lock().expect("the script").pop();
            match next {
                Some(event) => stream::once(async move { Ok(event) }).boxed(),
                None => stream::empty().boxed(),
            }
        }
    }

    fn clip(path: &Path, samples: usize) {
        let pcm: Vec<i16> = (0..samples).map(|n| (n % 97) as i16).collect();
        write_spine_wav(path, &pcm).expect("a clip this test can write");
    }

    fn on(compared: &[Compared]) -> Vec<String> {
        compared.iter().map(line).collect()
    }

    /// The names the report writes, held here as literals. `//cogs:speech_run_report`
    /// asserts the same pair in
    /// `the_two_clip_names_are_the_pair_the_comparison_looks_for`; the two tests
    /// together are the contract, since neither binary can import the other.
    #[test]
    fn the_pair_this_looks_for_is_the_pair_the_report_writes() {
        let whole = format!("turn-01{WHOLE_SUFFIX}");
        let command = format!("turn-01{COMMAND_SUFFIX}");
        assert_eq!(whole, "turn-01.wav");
        assert_eq!(command, "turn-01.command.wav");
        assert!(
            command.ends_with(WHOLE_SUFFIX),
            "the second name is a turn name too, which is why the pairing excludes it"
        );
    }

    #[test]
    fn a_turn_is_paired_with_the_clip_the_recogniser_heard() {
        let dir = scratch_dir("stt-compare-pairs");
        clip(&dir.join("turn-02.wav"), 32);
        clip(&dir.join("turn-02.command.wav"), 16);
        clip(&dir.join("turn-01.wav"), 32);
        clip(&dir.join("turn-01.command.wav"), 16);
        std::fs::write(dir.join("someone-elses.txt"), b"kept").expect("a stranger's file");

        let found = pairs(dir.as_ref()).expect("the clip directory");

        assert_eq!(
            found.iter().map(|p| p.label.as_str()).collect::<Vec<_>>(),
            ["turn-01", "turn-02"],
            "the turns come back in the run's order and a stranger's file is not one"
        );
        assert!(found.iter().all(|p| p.command.is_some()));
    }

    #[test]
    fn a_turn_whose_boundary_was_never_stated_is_listed_and_not_asked() {
        let dir = scratch_dir("stt-compare-no-boundary");
        clip(&dir.join("turn-01.wav"), 32);
        clip(&dir.join("turn-01.command.wav"), 16);
        clip(&dir.join("turn-02.wav"), 32);

        let found = pairs(dir.as_ref()).expect("the clip directory");
        let scripted = Scripted::new(vec!["Hey Jarvis, stop.", "Stop."]);
        let compared = block_on(compare(&scripted, &found)).expect("both clips read");

        assert_eq!(
            on(&compared)[1],
            "turn-02  no boundary",
            "a turn with one clip is named, not transcribed"
        );
        assert_eq!(
            *scripted.asked.lock().expect("the tally"),
            vec![32, 16],
            "only the paired turn reached the recogniser"
        );
        assert!(
            footer(&compared).contains("1 with no boundary"),
            "{}",
            footer(&compared)
        );
    }

    #[test]
    fn two_readings_of_one_turn_are_printed_side_by_side() {
        let dir = scratch_dir("stt-compare-both");
        clip(&dir.join("turn-01.wav"), 32);
        clip(&dir.join("turn-01.command.wav"), 16);

        let found = pairs(dir.as_ref()).expect("the clip directory");
        let scripted = Scripted::new(vec!["Hey Jarvis, what time is it?", "What time is it?"]);
        let compared = block_on(compare(&scripted, &found)).expect("both clips read");

        assert_eq!(
            on(&compared)[0],
            "turn-01  whole: \"Hey Jarvis, what time is it?\" no_speech=0.04 logprob=-0.31 \
             | command: \"What time is it?\" no_speech=0.04 logprob=-0.31"
        );
    }

    #[test]
    fn punctuation_and_case_are_not_a_disagreement() {
        assert!(!differs("What time is it?", "what time is it"));
        assert!(!differs("Stop!  Now.", "stop now"));
        assert!(
            differs("Hey Jarvis.", "What time is it?"),
            "different words are what this counts"
        );
        assert!(
            differs("Stop.", ""),
            "a reading that heard nothing disagrees with one that heard something"
        );
    }

    #[test]
    fn the_footer_counts_the_turns_that_read_differently() {
        let compared = vec![
            Compared::Both {
                label: "turn-01".to_owned(),
                whole: heard_text("Hey Jarvis, stop."),
                command: heard_text("Stop."),
            },
            Compared::Both {
                label: "turn-02".to_owned(),
                whole: heard_text("What time is it?"),
                command: heard_text("what time is it"),
            },
        ];

        assert_eq!(
            footer(&compared),
            "2 turn(s) compared, 1 of them read differently."
        );
    }

    #[test]
    fn a_recogniser_that_settles_on_nothing_is_a_refusal() {
        let dir = scratch_dir("stt-compare-empty-stream");
        clip(&dir.join("turn-01.wav"), 32);
        clip(&dir.join("turn-01.command.wav"), 16);

        let found = pairs(dir.as_ref()).expect("the clip directory");
        let scripted = Scripted::new(Vec::new());
        let why = block_on(compare(&scripted, &found)).expect_err("no transcript came back");

        assert!(why.contains("no transcript"), "{why}");
        assert!(why.contains("turn-01.wav"), "{why}");
    }

    /// Every way the configuration can fail to name a recogniser is a refusal that
    /// says which piece is missing — a run recorded without a transcriber is the
    /// common case, and the operator has to be told that rather than handed an
    /// empty table. Each refusal names the key it lacks.
    #[test]
    fn a_configuration_that_names_no_recogniser_is_refused_by_the_piece_it_lacks() {
        let dir = scratch_dir("stt-compare-config");
        let base = "listen_addr = \"10.0.0.5:7380\"\npod_psk_file = \"/psk.toml\"\n";
        let written = |name: &str, body: String| {
            let path = dir.join(name);
            std::fs::write(&path, body).expect("a configuration this test can write");
            path
        };

        let why = transcriber(&dir.join("absent.toml")).expect_err("there is no such file");
        assert!(why.contains("absent.toml"), "{why}");

        let path = written("no-stt.toml", base.to_owned());
        let why = transcriber(&path).expect_err("there is no [stt] table to ask");
        assert!(why.contains("[stt]"), "{why}");

        let path = written(
            "no-url.toml",
            format!("{base}[stt]\nbackend = \"http\"\nmodel = \"whisper-1\"\n"),
        );
        let why = transcriber(&path).expect_err("no url to ask");
        assert!(why.contains("url"), "{why}");

        let path = written(
            "no-model.toml",
            format!("{base}[stt]\nbackend = \"http\"\nurl = \"http://10.0.0.5:8000\"\n"),
        );
        let why = transcriber(&path).expect_err("no model to ask for");
        assert!(why.contains("model"), "{why}");
    }

    #[test]
    fn the_invocation_names_a_configuration_and_a_directory() {
        let options = parse(
            ["--speech-config", "/run/reachy/speech.toml", "run.turns"]
                .into_iter()
                .map(str::to_owned),
        )
        .expect("both were given");
        assert_eq!(
            options.speech_config,
            PathBuf::from("/run/reachy/speech.toml")
        );
        assert_eq!(options.turns, PathBuf::from("run.turns"));

        for wrong in [
            vec!["run.turns"],
            vec!["--speech-config", "a.toml"],
            vec!["--speech-config", "a.toml", "run.turns", "other.turns"],
            vec![
                "--speech-config",
                "a.toml",
                "--speech-config",
                "b.toml",
                "r",
            ],
            vec!["--whole-only", "run.turns"],
        ] {
            assert!(
                parse(wrong.iter().map(|w| (*w).to_owned())).is_err(),
                "{wrong:?}"
            );
        }
    }

    fn heard_text(text: &str) -> Transcript {
        Transcript {
            text: text.to_owned(),
            confidence: None,
        }
    }

    fn block_on<T>(future: impl std::future::Future<Output = T>) -> T {
        tokio::runtime::Builder::new_current_thread()
            .build()
            .expect("a runtime")
            .block_on(future)
    }
}
