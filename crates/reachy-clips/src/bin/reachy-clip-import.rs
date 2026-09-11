//! `reachy-clip-import` — vendor recordings in, our clips out.
//!
//! An offline host-side converter. It reaches no network: the operator fetches
//! the dataset themselves (`hf download --repo-type dataset …`) and points this
//! at the directory. Every `*.json` under it, at any depth, is read, converted,
//! validated through the same loader the daemon runs, and written out under a
//! library name. A file that will not convert is refused by name and listed;
//! the rest go through.
//!
//! A refusal is an outcome, not a failure: these datasets contain poses this
//! machine's envelope refuses, so a batch that reports refusals is the normal
//! batch. The report — on the console, and durably in `--report`'s file beside
//! the clips — is the record of why a recording is absent from the library.
//!
//! Nothing here decides anything about motion. The conversion and every refusal
//! live in `reachy_clips::vendor`, which is pure and tested; this file owns the
//! arguments, the directory walk, the writing and the report.
//!
//! The datasets carry their own per-repo licences, which are not determinable
//! offline. This tool converts whatever it is pointed at and takes no position
//! on whether you may.

#![forbid(unsafe_code)]

use std::collections::BTreeMap;
use std::path::{Path, PathBuf};

use anyhow::{Context as _, bail};

use reachy_clips::envelope::ClipLimits;
use reachy_clips::files::document_paths;
use reachy_clips::format::ChannelMask;
use reachy_clips::vendor::{Import, ImportError, ImportOptions, ROTATION_DRIFT_NOTED, convert};

/// The sound files the vendor ships beside a recording, in the order their own
/// reader prefers them.
const SOUND_EXTENSIONS: [&str; 8] = ["wav", "mp3", "ogg", "oga", "opus", "flac", "m4a", "aac"];

#[derive(Debug)]
struct Args {
    input: PathBuf,
    output: PathBuf,
    prefix: String,
    channels: Option<ChannelMask>,
    /// Where the report is written as well as said, if anywhere.
    report: Option<PathBuf>,
    /// Free text naming where the recordings came from, for the report header.
    source: Option<String>,
}

fn usage() -> String {
    "usage: reachy-clip-import --input DIR --output DIR --prefix NAME [--channels LIST]\n\
     \x20                      [--report FILE] [--source TEXT]\n\
     \n\
     \x20 --input DIR      a directory of vendor recordings; every *.json under it\n\
     \x20 --output DIR     where the converted clips are written; created if absent\n\
     \x20 --prefix NAME    the library namespace, e.g. pollen/emotions\n\
     \x20 --channels LIST  comma-separated mask override: head,antennas,body_yaw\n\
     \x20 --report FILE    write the report here as well as saying it\n\
     \x20 --source TEXT    what the recordings are, for the report header\n\
     \n\
     The report's header carries the prefix, the mask override and the source,\n\
     and no path: it is committed to a public tree, so no line of it names a\n\
     place on this machine. A recording it names is named relative to --input.\n\
     \n\
     Each clip is named <prefix>/<file stem>. A sound file sharing a recording's\n\
     stem is copied verbatim beside the clip for a future audio surface; nothing\n\
     in the motion stack reads it.\n\
     \n\
     A clip drives the channels the recording states and moves; a stated channel\n\
     that stands still is dropped, since masking it would pin it for the whole\n\
     playback. The override names the channels to drive instead: any the\n\
     recording states, still or moving, and never one it moves and the mask\n\
     would drop.\n\
     \n\
     A refused recording is a recorded outcome, not a process failure: the\n\
     summary line and the report say which and why. The exit status is 1 only\n\
     for an operational failure — input that will not read, output that will\n\
     not be written, no recordings found, or nothing converted at all.\n\
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
    let (mut report, mut source) = (None, None);
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
            "--report" => report = Some(PathBuf::from(value("--report")?)),
            "--source" => source = Some(value("--source")?),
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
        report,
        source,
    })
}

/// The mask a `--channels` list names, in the format's own spellings.
fn mask(list: &str) -> anyhow::Result<ChannelMask> {
    ChannelMask::parse(list).with_context(|| format!("--channels {list}"))
}

/// The report header: what this run was, so the file beside the clips answers
/// "where did these come from" without a scrollback. It carries the prefix, the
/// mask override when there was one, and the source — and no path.
///
/// `--source` is free text because the provenance of a directory on a disk is
/// not derivable from the directory: the operator fetched it and knows the
/// repository and the revision, and the input path names neither. The report is
/// committed to a public tree, so no line of it names a location on the
/// operator's machine; `--input` and `--output` are locations and stay out.
fn header(args: &Args) -> Vec<String> {
    let mut lines = vec![
        "reachy-clip-import".to_owned(),
        format!("--prefix {}", args.prefix),
    ];
    if let Some(channels) = args.channels {
        let named: Vec<&str> = channels.iter().map(|channel| channel.as_str()).collect();
        lines.push(format!("--channels {}", named.join(",")));
    }
    lines.push(format!(
        "source: {}",
        args.source.as_deref().unwrap_or("unstated")
    ));
    lines.push(String::new());
    lines
}

/// A refusal does not stop the batch: stopping at the first bad recording in a
/// hundred would hide the other ninety-nine, and for these datasets a refusal is
/// the expected outcome for a recording whose poses this machine will not take.
/// So the exit code is not about refusals — it is about whether the run
/// operated: unreadable input, unwritable output, no recordings, or a batch that
/// converted nothing and so produced no library at all.
fn run(args: &Args, say: &mut dyn FnMut(String)) -> anyhow::Result<()> {
    // Said and kept: the console gets every line as it happens, and `--report`
    // gets the same lines afterwards, so the durable record and the terminal
    // cannot disagree.
    let mut kept: Vec<String> = header(args);
    let say = &mut |line: String| {
        kept.push(line.clone());
        say(line);
    };
    let files = recordings(&args.input)?;
    if files.is_empty() {
        bail!("no *.json under {}", args.input.display());
    }
    std::fs::create_dir_all(&args.output)
        .with_context(|| format!("cannot create {}", args.output.display()))?;
    let limits = ClipLimits::default();
    let options = ImportOptions {
        channels: args.channels,
    };

    let mut refused = 0usize;
    // Both the library name and the output file come from the stem alone, so
    // one stem carried by two directories of the input tree would write one
    // recording over the other and report two. Refused by name instead: a
    // motion silently missing from the library first surfaces as an
    // unknown-motion refusal on a script the report said would work.
    let mut written: BTreeMap<String, PathBuf> = BTreeMap::new();
    // No path: the report is committed to a public tree, and `--prefix` and
    // `source:` already name the destination and provenance in the header.
    say(format!("{} recording(s)", files.len()));
    for file in &files {
        let stem = file
            .file_stem()
            .and_then(|stem| stem.to_str())
            .with_context(|| format!("{} has no usable name", file.display()))?;
        let name = format!("{}/{stem}", args.prefix.trim_end_matches('/'));
        if let Some(first) = written.get(&name) {
            refused += 1;
            // Named relative to the input: which two files of the dataset
            // collided is the dataset's own layout, and the stem alone would
            // not say it — the two share it by construction.
            say(format!(
                "REFUSED {name}: {} shares its stem with {}, which is already this name",
                inside(file, &args.input)?.display(),
                inside(first, &args.input)?.display()
            ));
            continue;
        }
        // The `?` is the whole distinction this loop turns on: a recording this
        // machine will not take is refused and counted, and a file that will not
        // read or an output that will not be written ends the run. Recording the
        // second as a refusal would blame the recording for the disk and leave a
        // committed report saying so.
        match one(file, &name, &args.output, &limits, &options)? {
            Ok(import) => {
                written.insert(name.clone(), file.clone());
                say(converted_line(&name, &import));
            }
            Err(refusal) => {
                refused += 1;
                say(format!("REFUSED {name}: {refusal}"));
            }
        }
    }
    let converted = files.len() - refused;
    say(format!("{converted} converted, {refused} refused"));
    if let Some(path) = &args.report {
        if let Some(parent) = path
            .parent()
            .filter(|parent| !parent.as_os_str().is_empty())
        {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("cannot create {}", parent.display()))?;
        }
        let mut text = kept.join("\n");
        text.push('\n');
        std::fs::write(path, text).with_context(|| format!("cannot write {}", path.display()))?;
    }
    if converted == 0 {
        bail!("nothing converted: {refused} recording(s) refused, so there is no library");
    }
    Ok(())
}

/// Convert one recording and write what it produced.
///
/// Two failures, and they are not the same thing: the outer `Err` is the run
/// not operating — a file that will not read, a document or a sidecar that will
/// not be written — and the inner one is the recording being refused, which is
/// an outcome this batch reports and carries on from.
fn one(
    file: &Path,
    name: &str,
    output: &Path,
    limits: &ClipLimits,
    options: &ImportOptions,
) -> anyhow::Result<Result<Import, ImportError>> {
    let json =
        std::fs::read_to_string(file).with_context(|| format!("cannot read {}", file.display()))?;
    let import: Import = match convert(&json, name, limits, options) {
        Ok(import) => import,
        Err(refusal) => return Ok(Err(refusal)),
    };
    let stem = file.file_stem().expect("a file with a stem");
    let target = output.join(stem).with_extension("json");
    let text = serde_json::to_string_pretty(&import.doc()).context("cannot render the clip")?;
    std::fs::write(&target, text).with_context(|| format!("cannot write {}", target.display()))?;
    if let Some(sound) = sidecar(file) {
        let beside = output.join(sound.file_name().expect("a sidecar with a name"));
        std::fs::copy(&sound, &beside)
            .with_context(|| format!("cannot copy {}", sound.display()))?;
    }
    Ok(Ok(import))
}

fn converted_line(name: &str, import: &Import) -> String {
    let channels: Vec<&str> = import
        .clip
        .mask()
        .iter()
        .map(|channel| channel.as_str())
        .collect();
    let mut line = format!(
        "{name}: {:.2} s, {} frames (from {} over {:.2} s), [{}], blends {}/{} ms",
        import.clip.duration_s(),
        import.clip.frames().len(),
        import.source_frames,
        import.source_duration_s,
        channels.join(","),
        import.clip.blend_in_ms(),
        import.clip.blend_out_ms(),
    );
    let still = |masked: bool| -> Vec<&str> {
        import
            .constant
            .iter()
            .filter(|channel| import.clip.mask().contains(**channel) == masked)
            .map(|channel| channel.as_str())
            .collect()
    };
    let dropped = still(false);
    if !dropped.is_empty() {
        line.push_str(&format!("; dropped as still: {}", dropped.join(",")));
    }
    let pinned = still(true);
    if !pinned.is_empty() {
        line.push_str(&format!("; pinned though still: {}", pinned.join(",")));
    }
    if import.rotation_drift > ROTATION_DRIFT_NOTED {
        line.push_str(&format!("; rotation drift {:.1e}", import.rotation_drift));
    }
    if !import.unknown_keys.is_empty() {
        line.push_str(&format!(
            "; keys we do not read: {}",
            import.unknown_keys.join(",")
        ));
    }
    line
}

/// Every motion document under `input`, at any depth: vendor datasets put their
/// recordings at the root or under `data/` and nothing says a later one will
/// not nest further, so the rule is the whole tree rather than a list of
/// locations.
///
/// Which files count is [`document_paths`]', not this tool's: the daemon and
/// the bench read a directory by the same rule, and a recording this converted
/// but they would not read is a clip that goes missing between the batch and
/// the machine.
fn recordings(input: &Path) -> anyhow::Result<Vec<PathBuf>> {
    let paths =
        document_paths(input).with_context(|| format!("cannot read {}", input.display()))?;
    Ok(paths.into_iter().filter(|path| path.is_file()).collect())
}

/// A recording named the way the report may name it: relative to `--input`.
///
/// The walk builds every path by joining onto the directory it was handed, so
/// for anything [`recordings`] returned the strip holds. Should it ever not,
/// that is an operational error rather than a quiet fall back to the absolute
/// path, which is the one thing the report must not carry.
fn inside<'a>(file: &'a Path, input: &Path) -> anyhow::Result<&'a Path> {
    file.strip_prefix(input)
        .with_context(|| format!("{} is not under the input directory", file.display()))
}

fn sidecar(file: &Path) -> Option<PathBuf> {
    SOUND_EXTENSIONS
        .into_iter()
        .map(|ext| file.with_extension(ext))
        .find(|candidate| candidate.is_file())
}

#[cfg(test)]
mod tests {
    use reachy_clips::format::Channel;
    use reachy_scratch::scratch_dir;

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

    /// A vendor recording of `frames` frames, as JSON text: the antennas creep,
    /// the head stands still, and the body yaw is stated and never touched —
    /// the shape of most of the published dataset, and the one that exercises
    /// both halves of the still-channel report.
    fn recording(frames: usize) -> String {
        let identity: Vec<Vec<f64>> = (0..4)
            .map(|row| (0..4).map(|col| f64::from(u8::from(row == col))).collect())
            .collect();
        let times: Vec<f64> = (0..frames).map(|index| index as f64 * 0.02).collect();
        let track: Vec<serde_json::Value> = (0..frames)
            .map(|index| {
                let angle = index as f64 * 0.005;
                serde_json::json!({
                    "head": identity,
                    "antennas": [angle, -angle],
                    "body_yaw": 0.0,
                })
            })
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
        let input = scratch_dir("reachy-clip-import-report-in");
        let output = scratch_dir("reachy-clip-import-report-out");
        std::fs::write(input.join("nod.json"), recording(10)).expect("the recording writes");
        let args = Args {
            input: input.as_ref().into(),
            output: output.as_ref().into(),
            prefix: "pollen/test".to_owned(),
            channels: None,
            report: None,
            source: None,
        };
        let mut lines = Vec::new();
        run(&args, &mut |line| lines.push(line)).expect("a still recording converts");

        let converted = lines
            .iter()
            .find(|line| line.starts_with("pollen/test/nod:"))
            .expect("the clip is named in the report");
        assert!(converted.contains("0.20 s"), "{converted}");
        assert!(converted.contains("10 frames (from 10"), "{converted}");
        assert!(converted.contains("[antennas]"), "{converted}");
        assert!(converted.contains("blends "), "{converted}");
        assert!(
            converted.contains("dropped as still: head,body_yaw"),
            "a channel the recording states and never moved says so: {converted}"
        );
        assert!(
            !converted.contains("pinned though still"),
            "nothing still was masked: {converted}"
        );
        assert!(
            !converted.contains("rotation drift"),
            "an exact matrix has none worth saying: {converted}"
        );
        assert_eq!(lines.last().expect("a tally"), "1 converted, 0 refused");
    }

    /// A channel force-pinned by the override is the other half of the same
    /// report: the operator asked for a still channel, and the line says the
    /// clip pins it rather than leaving them to infer it from the mask. The
    /// header's other two branches are here too — the `--channels` line is
    /// written only when an override was given, and an unstated source says so
    /// — because this is the only case that gives the header one.
    #[test]
    fn a_force_pinned_still_channel_is_named_in_the_report() {
        let input = scratch_dir("reachy-clip-import-pinned-in");
        let output = scratch_dir("reachy-clip-import-pinned-out");
        std::fs::write(input.join("nod.json"), recording(10)).expect("the recording writes");
        let mut channels = ChannelMask::of(Channel::Antennas);
        channels.insert(Channel::BodyYaw);
        let report = output.join("import-report.txt");
        let args = Args {
            input: input.as_ref().into(),
            output: output.as_ref().into(),
            prefix: "pollen/test".to_owned(),
            channels: Some(channels),
            report: Some(report.clone()),
            source: None,
        };
        let mut lines = Vec::new();
        run(&args, &mut |line| lines.push(line)).expect("pinning a still channel is allowed");

        let converted = lines
            .iter()
            .find(|line| line.starts_with("pollen/test/nod:"))
            .expect("the clip is named in the report");
        assert!(converted.contains("[body_yaw,antennas]"), "{converted}");
        assert!(converted.contains("dropped as still: head"), "{converted}");
        assert!(
            converted.contains("pinned though still: body_yaw"),
            "{converted}"
        );

        let text = std::fs::read_to_string(&report).expect("the report was written");
        assert!(text.contains("--channels body_yaw,antennas"), "{text}");
        assert!(text.contains("source: unstated"), "{text}");
    }

    /// The sidecar copy is a stated deliverable and nothing in the motion path
    /// would miss it: deleted outright, only this notices.
    #[test]
    fn a_sound_beside_a_recording_is_copied_verbatim() {
        let input = scratch_dir("reachy-clip-import-sidecar-in");
        let output = scratch_dir("reachy-clip-import-sidecar-out");
        std::fs::write(input.join("nod.json"), recording(10)).expect("the recording writes");
        std::fs::write(input.join("nod.wav"), b"RIFF not really").expect("the sound writes");
        let args = Args {
            input: input.as_ref().into(),
            output: output.as_ref().into(),
            prefix: "pollen/test".to_owned(),
            channels: None,
            report: None,
            source: None,
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
    /// is not what the exit code is about: a run that produced a library
    /// succeeded, however much it refused.
    #[test]
    fn a_refusal_is_reported_by_name_and_leaves_the_rest_converted() {
        let input = scratch_dir("reachy-clip-import-refusal-in");
        let output = scratch_dir("reachy-clip-import-refusal-out");
        std::fs::write(input.join("good.json"), recording(10)).expect("writes");
        std::fs::write(input.join("bad.json"), "{not json at all").expect("writes");
        let args = Args {
            input: input.as_ref().into(),
            output: output.as_ref().into(),
            prefix: "pollen/test".to_owned(),
            channels: None,
            report: None,
            source: None,
        };
        let mut lines = Vec::new();
        let outcome = run(&args, &mut |line| lines.push(line));

        outcome.expect("a batch that converted something succeeded");
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
        let input = scratch_dir("reachy-clip-import-stem-in");
        let output = scratch_dir("reachy-clip-import-stem-out");
        let data = input.join("data");
        std::fs::create_dir_all(&data).expect("the vendor's other layout");
        std::fs::write(input.join("nod.json"), recording(10)).expect("writes");
        std::fs::write(data.join("nod.json"), recording(20)).expect("writes");
        let args = Args {
            input: input.as_ref().into(),
            output: output.as_ref().into(),
            prefix: "pollen/test".to_owned(),
            channels: None,
            report: None,
            source: None,
        };
        let mut lines = Vec::new();
        let outcome = run(&args, &mut |line| lines.push(line));

        outcome.expect("a collision is a refusal, not a failed run");
        let refusal = lines
            .iter()
            .find(|line| line.starts_with("REFUSED pollen/test/nod:"))
            .expect("the collision is refused by name");
        // Both colliding files are named, and named as the dataset lays them
        // out rather than as this machine stores them.
        let (later, rest) = refusal
            .trim_start_matches("REFUSED pollen/test/nod: ")
            .split_once(" shares its stem with ")
            .expect("the refusal names both files");
        let earlier = rest
            .trim_end_matches(", which is already this name")
            .to_owned();
        let mut named = [later.to_owned(), earlier];
        named.sort();
        assert_eq!(named, ["data/nod.json".to_owned(), "nod.json".to_owned()]);
        assert!(
            !refusal.contains(&input.as_ref().display().to_string()),
            "{refusal}"
        );
        assert_eq!(lines.last().expect("a tally"), "1 converted, 1 refused");
        // The one that did convert is the one still on disk, whole.
        let written = std::fs::read_to_string(output.join("nod.json")).expect("one clip landed");
        assert!(written.contains("pollen/test/nod"), "{written}");
    }

    /// The report is the record of why a recording is absent from the library,
    /// so it has to survive the terminal: the file carries the header naming
    /// the run and its source, and every line the console got.
    ///
    /// It is also committed to a public tree, so the whole file is checked for
    /// the one thing it must never carry: a location on the operator's machine.
    /// The scratch directories are absolute paths under the system temp
    /// directory, which is exactly the shape of the thing banned.
    #[test]
    fn the_report_file_carries_the_header_and_the_lines() {
        let input = scratch_dir("reachy-clip-import-file-in");
        let output = scratch_dir("reachy-clip-import-file-out");
        std::fs::write(input.join("nod.json"), recording(10)).expect("writes");
        std::fs::write(input.join("bad.json"), "{not json at all").expect("writes");
        let report = output.join("reports").join("import-report.txt");
        let args = Args {
            input: input.as_ref().into(),
            output: output.as_ref().into(),
            prefix: "pollen/test".to_owned(),
            channels: None,
            report: Some(report.clone()),
            source: Some("pollen-robotics/dataset@abc123".to_owned()),
        };
        let mut lines = Vec::new();
        run(&args, &mut |line| lines.push(line)).expect("one conversion is a run that operated");

        let text = std::fs::read_to_string(&report).expect("the report was written");
        assert!(text.starts_with("reachy-clip-import\n"), "{text}");
        assert!(
            text.contains(&format!("--prefix {}", args.prefix)),
            "the prefix is in the header: {text}"
        );
        assert!(
            text.contains("source: pollen-robotics/dataset@abc123"),
            "{text}"
        );
        for banned in [input.as_ref(), output.as_ref()] {
            assert!(
                !text.contains(&banned.display().to_string()),
                "{} names this machine: {text}",
                banned.display()
            );
        }
        let (head, body) = text.split_once("\n\n").expect("a header and a body");
        assert!(
            !head.contains("--input") && !head.contains("--output"),
            "{head}"
        );
        assert_eq!(
            body.lines().next().expect("an opening line"),
            "2 recording(s)"
        );
        for line in &lines {
            assert!(
                text.contains(line.as_str()),
                "{line} is missing from {text}"
            );
        }
        assert!(text.ends_with("1 converted, 1 refused\n"), "{text}");
    }

    /// An output that will not be written is the run not operating, and the
    /// whole point of the two-level result: rendering it as `REFUSED <name>`
    /// would blame the recording for the disk, exit 0, and leave a committed
    /// report saying so beside an incomplete library.
    #[test]
    fn a_clip_that_cannot_be_written_ends_the_run() {
        let input = scratch_dir("reachy-clip-import-unwritable-in");
        let output = scratch_dir("reachy-clip-import-unwritable-out");
        std::fs::write(input.join("nod.json"), recording(10)).expect("writes");
        // The target of the write is a directory, so the write itself fails
        // after the conversion succeeded.
        std::fs::create_dir(output.join("nod.json")).expect("the target is in the way");
        let args = Args {
            input: input.as_ref().into(),
            output: output.as_ref().into(),
            prefix: "pollen/test".to_owned(),
            channels: None,
            report: None,
            source: None,
        };
        let mut lines = Vec::new();
        let outcome = run(&args, &mut |line| lines.push(line));

        let error = format!(
            "{:#}",
            outcome.expect_err("an unwritable clip fails the run")
        );
        assert!(
            error.contains(&output.join("nod.json").display().to_string()),
            "the error names what would not be written: {error}"
        );
        assert!(
            !lines.iter().any(|line| line.starts_with("REFUSED")),
            "the recording was not blamed: {lines:?}"
        );
    }

    /// The sidecar copy is the other outer-error site: it happens after the
    /// clip landed, and it is still the run not operating rather than a
    /// refusal.
    #[test]
    fn a_sidecar_that_cannot_be_copied_ends_the_run() {
        let input = scratch_dir("reachy-clip-import-sidecar-fail-in");
        let output = scratch_dir("reachy-clip-import-sidecar-fail-out");
        std::fs::write(input.join("nod.json"), recording(10)).expect("writes");
        std::fs::write(input.join("nod.wav"), b"RIFF not really").expect("the sound writes");
        std::fs::create_dir(output.join("nod.wav")).expect("the target is in the way");
        let args = Args {
            input: input.as_ref().into(),
            output: output.as_ref().into(),
            prefix: "pollen/test".to_owned(),
            channels: None,
            report: None,
            source: None,
        };
        let mut lines = Vec::new();
        let outcome = run(&args, &mut |line| lines.push(line));

        let error = format!(
            "{:#}",
            outcome.expect_err("an uncopyable sidecar fails the run")
        );
        assert!(
            error.contains(&input.join("nod.wav").display().to_string()),
            "the error names the sidecar: {error}"
        );
        assert!(
            !lines.iter().any(|line| line.starts_with("REFUSED")),
            "the recording was not blamed: {lines:?}"
        );
    }

    /// A batch that refused everything wrote no library, whatever its reasons,
    /// and that is the one refusal outcome the exit code is about.
    #[test]
    fn a_batch_that_converts_nothing_fails() {
        let input = scratch_dir("reachy-clip-import-empty-in");
        let output = scratch_dir("reachy-clip-import-empty-out");
        std::fs::write(input.join("bad.json"), "{not json at all").expect("writes");
        let report = output.join("import-report.txt");
        let args = Args {
            input: input.as_ref().into(),
            output: output.as_ref().into(),
            prefix: "pollen/test".to_owned(),
            channels: None,
            report: Some(report.clone()),
            source: None,
        };
        let outcome = run(&args, &mut |_| {});

        assert!(outcome.is_err(), "no library is a failed run");
        let text = std::fs::read_to_string(&report).expect("the report says why");
        assert!(text.contains("0 converted, 1 refused"), "{text}");
    }

    /// An input with no recordings at all is operator error rather than an
    /// empty library.
    #[test]
    fn an_input_with_no_recordings_fails() {
        let input = scratch_dir("reachy-clip-import-bare-in");
        let output = scratch_dir("reachy-clip-import-bare-out");
        let args = Args {
            input: input.as_ref().into(),
            output: output.as_ref().into(),
            prefix: "pollen/test".to_owned(),
            channels: None,
            report: None,
            source: None,
        };
        assert!(run(&args, &mut |_| {}).is_err(), "nothing to convert");
    }

    /// The input walk is the whole tree: a dataset that files its recordings
    /// under a subdirectory converts whole, and the name still comes from the
    /// stem alone.
    #[test]
    fn a_nested_recording_is_imported() {
        let input = scratch_dir("reachy-clip-import-nested-in");
        let output = scratch_dir("reachy-clip-import-nested-out");
        let nested = input.join("data").join("second");
        std::fs::create_dir_all(&nested).expect("a nested layout");
        std::fs::write(nested.join("deep.json"), recording(10)).expect("writes");
        let args = Args {
            input: input.as_ref().into(),
            output: output.as_ref().into(),
            prefix: "pollen/test".to_owned(),
            channels: None,
            report: None,
            source: None,
        };
        let mut lines = Vec::new();
        run(&args, &mut |line| lines.push(line)).expect("a nested recording converts");

        assert!(output.join("deep.json").is_file(), "{lines:?}");
        assert_eq!(lines.last().expect("a tally"), "1 converted, 0 refused");
    }

    /// Drift worth saying is said. The four stretched recordings in the
    /// published sets are the reason the line carries it at all, and the
    /// threshold is what keeps the hundred ordinary ones quiet.
    #[test]
    fn a_stretched_rotation_is_noted_in_the_report_line() {
        let input = scratch_dir("reachy-clip-import-drift-in");
        let output = scratch_dir("reachy-clip-import-drift-out");
        let mut value: serde_json::Value =
            serde_json::from_str(&recording(10)).expect("the recording parses");
        for frame in value["set_target_data"]
            .as_array_mut()
            .expect("a track")
            .iter_mut()
        {
            for row in frame["head"].as_array_mut().expect("a matrix")[..3].iter_mut() {
                for cell in row.as_array_mut().expect("a row")[..3].iter_mut() {
                    let scaled = cell.as_f64().expect("a number") * 1.000_25;
                    *cell = serde_json::json!(scaled);
                }
            }
        }
        std::fs::write(input.join("nod.json"), value.to_string()).expect("writes");
        let args = Args {
            input: input.as_ref().into(),
            output: output.as_ref().into(),
            prefix: "pollen/test".to_owned(),
            channels: None,
            report: None,
            source: None,
        };
        let mut lines = Vec::new();
        run(&args, &mut |line| lines.push(line)).expect("a stretched rotation is renormalised");

        let converted = lines
            .iter()
            .find(|line| line.starts_with("pollen/test/nod:"))
            .expect("the clip is named in the report");
        assert!(converted.contains("rotation drift 5.0e-4"), "{converted}");
    }
}
