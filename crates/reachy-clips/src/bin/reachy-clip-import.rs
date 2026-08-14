//! `reachy-clip-import` — vendor recordings in, our clips out.
//!
//! An offline host-side converter. It reaches no network: the operator fetches
//! the dataset themselves (`hf download --repo-type dataset …`) and points this
//! at the directory. Every `*.json` in the root and in `data/` — the vendor's
//! own two locations — is read, converted, validated through the same loader
//! the daemon runs, and written out under a library name. A file that will not
//! convert is refused by name and listed; the rest go through.
//!
//! Nothing here decides anything about motion. The conversion and every refusal
//! live in `reachy_clips::vendor`, which is pure and tested; this file owns the
//! arguments, the directory walk, the writing and the report.
//!
//! The datasets carry their own per-repo licences, which are not determinable
//! offline. This tool converts whatever it is pointed at and takes no position
//! on whether you may.

#![forbid(unsafe_code)]

use std::collections::{BTreeMap, BTreeSet};
use std::path::{Path, PathBuf};

use anyhow::{Context as _, bail};

use reachy_clips::files::document_paths;
use reachy_clips::format::{Channel, ChannelMask};
use reachy_clips::speed::ClipLimits;
use reachy_clips::vendor::{Import, ImportError, ImportOptions, convert};

/// The sound files the vendor ships beside a recording, in the order their own
/// reader prefers them.
const SOUND_EXTENSIONS: [&str; 8] = ["wav", "mp3", "ogg", "oga", "opus", "flac", "m4a", "aac"];

#[derive(Debug)]
struct Args {
    input: PathBuf,
    output: PathBuf,
    prefix: String,
    channels: Option<ChannelMask>,
}

fn usage() -> String {
    "usage: reachy-clip-import --input DIR --output DIR --prefix NAME [--channels LIST]\n\
     \n\
     \x20 --input DIR      a directory of vendor recordings; *.json in it and in data/\n\
     \x20 --output DIR     where the converted clips are written; created if absent\n\
     \x20 --prefix NAME    the library namespace, e.g. pollen/emotions\n\
     \x20 --channels LIST  comma-separated mask override: head,antennas,body_yaw\n\
     \n\
     Each clip is named <prefix>/<file stem>. A sound file sharing a recording's\n\
     stem is copied verbatim beside the clip for a future audio surface; nothing\n\
     in the motion stack reads it.\n\
     \n\
     The mask override may only drop channels the recording carries and does not\n\
     move. Use it when the report says a masked channel never moves: a clip that\n\
     masks a channel it holds still pins that channel for the whole playback.\n\
     \n\
     This fetches nothing. Download the dataset first, and check its licence\n\
     before you use what comes out."
        .to_owned()
}

fn main() -> anyhow::Result<()> {
    let args = parse(std::env::args().skip(1))?;
    run(&args, &mut |line| println!("{line}"))
}

fn parse(words: impl Iterator<Item = String>) -> anyhow::Result<Args> {
    let (mut input, mut output, mut prefix, mut channels) = (None, None, None, None);
    let mut words = words.peekable();
    while let Some(word) = words.next() {
        let mut value = |flag: &str| -> anyhow::Result<String> {
            words
                .next()
                .with_context(|| format!("{flag} wants a value\n\n{}", usage()))
        };
        match word.as_str() {
            "--input" => input = Some(PathBuf::from(value("--input")?)),
            "--output" => output = Some(PathBuf::from(value("--output")?)),
            "--prefix" => prefix = Some(value("--prefix")?),
            "--channels" => channels = Some(mask(&value("--channels")?)?),
            "-h" | "--help" => {
                println!("{}", usage());
                std::process::exit(0);
            }
            other => bail!("unknown argument {other}\n\n{}", usage()),
        }
    }
    let missing = |what: &str| anyhow::anyhow!("{what} is required\n\n{}", usage());
    Ok(Args {
        input: input.ok_or_else(|| missing("--input"))?,
        output: output.ok_or_else(|| missing("--output"))?,
        prefix: prefix.ok_or_else(|| missing("--prefix"))?,
        channels,
    })
}

fn mask(list: &str) -> anyhow::Result<ChannelMask> {
    let mut mask = ChannelMask::empty();
    for word in list.split(',') {
        let channel = Channel::ALL
            .into_iter()
            .find(|channel| channel.as_str() == word.trim())
            .with_context(|| format!("{word} is not a channel; head, antennas or body_yaw"))?;
        if !mask.insert(channel) {
            bail!("{word} is named twice");
        }
    }
    if mask == ChannelMask::empty() {
        bail!("--channels names nothing");
    }
    Ok(mask)
}

/// A refusal does not stop the batch: stopping at the first bad recording in a
/// hundred would hide the other ninety-nine. The exit code says whether anything
/// was refused, so a script can still tell.
fn run(args: &Args, say: &mut dyn FnMut(String)) -> anyhow::Result<()> {
    let files = recordings(&args.input)?;
    if files.is_empty() {
        bail!("no *.json under {} or its data/", args.input.display());
    }
    std::fs::create_dir_all(&args.output)
        .with_context(|| format!("cannot create {}", args.output.display()))?;
    let limits = ClipLimits::default();
    let options = ImportOptions {
        channels: args.channels,
    };

    let mut refused = 0usize;
    // Both the library name and the output file come from the stem alone, so a
    // stem the root and `data/` both carry would write one recording over the
    // other and report two. Refused by name instead: a motion silently missing
    // from the library first surfaces as an unknown-motion refusal on a script
    // the report said would work.
    let mut written: BTreeMap<String, PathBuf> = BTreeMap::new();
    say(format!(
        "{} recording(s) from {} to {}",
        files.len(),
        args.input.display(),
        args.output.display()
    ));
    for file in &files {
        let stem = file
            .file_stem()
            .and_then(|stem| stem.to_str())
            .with_context(|| format!("{} has no usable name", file.display()))?;
        let name = format!("{}/{stem}", args.prefix.trim_end_matches('/'));
        if let Some(first) = written.get(&name) {
            refused += 1;
            say(format!(
                "REFUSED {name}: {} shares its stem with {}, which is already this name",
                file.display(),
                first.display()
            ));
            continue;
        }
        match one(file, &name, &args.output, &limits, &options) {
            Ok(import) => {
                written.insert(name.clone(), file.clone());
                say(converted_line(&name, &import));
            }
            Err(error) => {
                refused += 1;
                say(format!("REFUSED {name}: {error:#}"));
            }
        }
    }
    say(format!(
        "{} converted, {refused} refused",
        files.len() - refused
    ));
    if refused > 0 {
        bail!("{refused} recording(s) were refused; nothing was written for them");
    }
    Ok(())
}

fn one(
    file: &Path,
    name: &str,
    output: &Path,
    limits: &ClipLimits,
    options: &ImportOptions,
) -> anyhow::Result<Import> {
    let json =
        std::fs::read_to_string(file).with_context(|| format!("cannot read {}", file.display()))?;
    let import: Import = convert(&json, name, limits, options).map_err(refusal)?;
    let stem = file.file_stem().expect("a file with a stem");
    let target = output.join(stem).with_extension("json");
    let text = serde_json::to_string_pretty(&import.doc()).context("cannot render the clip")?;
    std::fs::write(&target, text).with_context(|| format!("cannot write {}", target.display()))?;
    if let Some(sound) = sidecar(file) {
        let beside = output.join(sound.file_name().expect("a sidecar with a name"));
        std::fs::copy(&sound, &beside)
            .with_context(|| format!("cannot copy {}", sound.display()))?;
    }
    Ok(import)
}

/// A conversion refusal as an error chain, so the frame index and the limit the
/// loader named survive into the report.
fn refusal(error: ImportError) -> anyhow::Error {
    anyhow::anyhow!(error)
}

fn converted_line(name: &str, import: &Import) -> String {
    let channels: Vec<&str> = import
        .clip
        .mask()
        .iter()
        .map(|channel| channel.as_str())
        .collect();
    let mut line = format!(
        "{name}: {:.2} s, {} frames (from {} over {:.2} s), [{}], max {:.2}x, blends {}/{} ms",
        import.clip.duration_s(),
        import.clip.frames().len(),
        import.source_frames,
        import.source_duration_s,
        channels.join(","),
        import.clip.max_speed(),
        import.clip.blend_in_ms(),
        import.clip.blend_out_ms(),
    );
    if !import.constant.is_empty() {
        let constant: Vec<&str> = import
            .constant
            .iter()
            .map(|channel| channel.as_str())
            .collect();
        line.push_str(&format!(
            "; masked but never moving: {} (consider --channels)",
            constant.join(",")
        ));
    }
    if !import.unknown_keys.is_empty() {
        line.push_str(&format!(
            "; keys we do not read: {}",
            import.unknown_keys.join(",")
        ));
    }
    line
}

/// The motion documents in `input` and in its `data/` — the two locations
/// vendor datasets use — so a dataset laid out either way converts whole.
///
/// Which files count is [`document_paths`]', not this tool's: the daemon and
/// the bench read a directory by the same rule, and a recording this converted
/// but they would not read is a clip that goes missing between the batch and
/// the machine.
fn recordings(input: &Path) -> anyhow::Result<Vec<PathBuf>> {
    let mut found = BTreeSet::new();
    for dir in [input.to_path_buf(), input.join("data")] {
        if !dir.is_dir() {
            continue;
        }
        let paths =
            document_paths(&dir).with_context(|| format!("cannot read {}", dir.display()))?;
        found.extend(paths.into_iter().filter(|path| path.is_file()));
    }
    Ok(found.into_iter().collect())
}

fn sidecar(file: &Path) -> Option<PathBuf> {
    SOUND_EXTENSIONS
        .into_iter()
        .map(|ext| file.with_extension(ext))
        .find(|candidate| candidate.is_file())
}

#[cfg(test)]
mod tests {
    use std::sync::atomic::{AtomicU32, Ordering};

    use super::*;

    /// The mask override reads the same spellings the format writes.
    #[test]
    fn a_channel_list_becomes_the_mask_it_names() {
        let parsed = mask("head, antennas").expect("two channels");
        let mut expected = ChannelMask::empty();
        expected.insert(Channel::Head);
        expected.insert(Channel::Antennas);
        assert_eq!(parsed, expected);
        assert!(mask("legs").is_err(), "a channel we do not have");
        assert!(mask("head,head").is_err(), "named twice");
        assert!(mask("").is_err(), "nothing at all");
    }

    /// Every flag is required, and a flag with no value says so rather than
    /// running on a default nobody chose.
    #[test]
    fn the_arguments_are_all_required() {
        let args = |words: &[&str]| parse(words.iter().map(|word| (*word).to_owned()));
        let complete = ["--input", "in", "--output", "out", "--prefix", "pollen/x"];
        let parsed = args(&complete).expect("a complete invocation");
        assert_eq!(parsed.prefix, "pollen/x");
        assert_eq!(parsed.channels, None);
        for drop in [0, 2, 4] {
            let mut words = complete.to_vec();
            words.drain(drop..drop + 2);
            assert!(args(&words).is_err(), "missing {}", complete[drop]);
        }
        assert!(args(&["--input"]).is_err(), "a flag with no value");
        assert!(args(&["--wat", "1"]).is_err(), "a flag we do not have");
    }

    /// A scratch directory of this test's own, named so one left behind by a
    /// panic says which test left it.
    fn scratch(name: &str) -> PathBuf {
        static NEXT: AtomicU32 = AtomicU32::new(0);
        let path = std::env::temp_dir().join(format!(
            "reachy-clip-import-{}-{}-{name}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        std::fs::create_dir_all(&path).expect("a scratch directory");
        path
    }

    /// A vendor recording of `frames` still frames, as JSON text.
    fn recording(frames: usize) -> String {
        let identity: Vec<Vec<f64>> = (0..4)
            .map(|row| (0..4).map(|col| f64::from(u8::from(row == col))).collect())
            .collect();
        let times: Vec<f64> = (0..frames).map(|index| index as f64 * 0.02).collect();
        let track: Vec<serde_json::Value> = (0..frames)
            .map(|_| serde_json::json!({ "head": identity, "antennas": [0.0, 0.0] }))
            .collect();
        serde_json::json!({
            "description": "a still recording",
            "time": times,
            "set_target_data": track,
        })
        .to_string()
    }

    /// The report line is the importer's whole output, so every field it
    /// carries is asserted: a format string that quietly lost the mask, the
    /// blends or the never-moving flag would tell an operator nothing and cost
    /// nothing in a suite that only checked the exit code.
    #[test]
    fn the_report_line_carries_what_the_operator_decides_on() {
        let input = scratch("report-in");
        let output = scratch("report-out");
        std::fs::write(input.join("nod.json"), recording(10)).expect("the recording writes");
        let args = Args {
            input: input.clone(),
            output: output.clone(),
            prefix: "pollen/test".to_owned(),
            channels: None,
        };
        let mut lines = Vec::new();
        run(&args, &mut |line| lines.push(line)).expect("a still recording converts");

        let converted = lines
            .iter()
            .find(|line| line.starts_with("pollen/test/nod:"))
            .expect("the clip is named in the report");
        assert!(converted.contains("0.20 s"), "{converted}");
        assert!(converted.contains("10 frames (from 10"), "{converted}");
        assert!(converted.contains("[head,antennas]"), "{converted}");
        assert!(converted.contains("max "), "{converted}");
        assert!(converted.contains("blends "), "{converted}");
        assert!(
            converted.contains("masked but never moving: head,antennas"),
            "a recording that never moved says so: {converted}"
        );
        assert_eq!(lines.last().expect("a tally"), "1 converted, 0 refused");
    }

    /// The sidecar copy is a stated deliverable and nothing in the motion path
    /// would miss it: deleted outright, only this notices.
    #[test]
    fn a_sound_beside_a_recording_is_copied_verbatim() {
        let input = scratch("sidecar-in");
        let output = scratch("sidecar-out");
        std::fs::write(input.join("nod.json"), recording(10)).expect("the recording writes");
        std::fs::write(input.join("nod.wav"), b"RIFF not really").expect("the sound writes");
        let args = Args {
            input,
            output: output.clone(),
            prefix: "pollen/test".to_owned(),
            channels: None,
        };
        run(&args, &mut |_| {}).expect("converts");

        assert!(output.join("nod.json").is_file(), "the clip landed");
        assert_eq!(
            std::fs::read(output.join("nod.wav")).expect("the sidecar landed"),
            b"RIFF not really",
            "copied verbatim"
        );
    }

    /// One bad recording does not stop the batch, is named in the report, and
    /// is what the exit code is about.
    #[test]
    fn a_refusal_is_reported_by_name_and_leaves_the_rest_converted() {
        let input = scratch("refusal-in");
        let output = scratch("refusal-out");
        std::fs::write(input.join("good.json"), recording(10)).expect("writes");
        std::fs::write(input.join("bad.json"), "{not json at all").expect("writes");
        let args = Args {
            input,
            output: output.clone(),
            prefix: "pollen/test".to_owned(),
            channels: None,
        };
        let mut lines = Vec::new();
        let outcome = run(&args, &mut |line| lines.push(line));

        assert!(outcome.is_err(), "a refusal is the exit code");
        assert!(output.join("good.json").is_file(), "the good one landed");
        assert!(!output.join("bad.json").exists(), "nothing was written");
        assert!(
            lines
                .iter()
                .any(|line| line.starts_with("REFUSED pollen/test/bad:")),
            "{lines:?}"
        );
        assert_eq!(lines.last().expect("a tally"), "1 converted, 1 refused");
    }

    /// Both vendor layouts at once, with a stem in each: the name and the
    /// output path come from the stem alone, so the second would land on top of
    /// the first. Refused by name instead, and counted.
    #[test]
    fn two_recordings_sharing_a_stem_are_refused_rather_than_overwritten() {
        let input = scratch("stem-in");
        let output = scratch("stem-out");
        let data = input.join("data");
        std::fs::create_dir_all(&data).expect("the vendor's other layout");
        std::fs::write(input.join("nod.json"), recording(10)).expect("writes");
        std::fs::write(data.join("nod.json"), recording(20)).expect("writes");
        let args = Args {
            input,
            output: output.clone(),
            prefix: "pollen/test".to_owned(),
            channels: None,
        };
        let mut lines = Vec::new();
        let outcome = run(&args, &mut |line| lines.push(line));

        assert!(outcome.is_err(), "a collision is a refusal");
        assert!(
            lines
                .iter()
                .any(|line| line.starts_with("REFUSED pollen/test/nod:")
                    && line.contains("shares its stem")),
            "{lines:?}"
        );
        assert_eq!(lines.last().expect("a tally"), "1 converted, 1 refused");
        // The one that did convert is the one still on disk, whole.
        let written = std::fs::read_to_string(output.join("nod.json")).expect("one clip landed");
        assert!(written.contains("pollen/test/nod"), "{written}");
    }
}
