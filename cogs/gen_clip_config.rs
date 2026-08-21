//! `gen-clip-config` — clip documents in, the library configuration out.
//!
//! An offline host-side emitter. It reads a directory of clip and sequence
//! documents the way a host that plays them reads it — `files::documents` into
//! `Library::load` under the machine's own `ClipLimits`, which resolves and
//! flattens every sequence — turns what it finds into a `ClipLibraryConfigWire`
//! message through the one mapping there is
//! ([`reachy_clips::config::write_library`]), and prints that message as the
//! protobuf text a box binds by path.
//!
//! An asset's identity is its position, in two numberings: `clip_id` indexes the
//! clips, `motion_id` indexes everything that plays — a motion per clip, plus
//! one per sequence — and both are the order the documents' paths sort in. So a
//! document that will not load is a **refusal of the whole emit**, not a skip
//! the way a running host would take it: dropping one asset renumbers every one
//! after it, and a script authored against the old numbering would then invoke
//! the wrong motion. The name tables ride out twice for the same reason — as
//! comments at the head of the asset, and as a JSON sidecar for whoever
//! authors scripts.
//!
//! Nothing here decides anything about motion. The validation is the loader's,
//! the mapping is `reachy_clips::config`'s, and the emitted asset is re-read
//! the way a cog reads it before it is written, so this tool cannot produce a
//! file the cogs would refuse.

#![forbid(unsafe_code)]

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use anyhow::{Context as _, bail};
use serde_json::json;

use brenn_reachy__cogs__config_clk_rs::{
    ClipFrame, ClipLibraryConfig, ClipLibraryConfigWire, MotionConfig,
};
use reachy_clips::config::{FRAME_FIELDS, UnplayableAsset, ValidatedLibrary, write_library};
use reachy_clips::files::documents;
use reachy_clips::format::Clip;
use reachy_clips::library::{Library, Motion};
use reachy_clips::speed::ClipLimits;

/// What the emitted asset says about itself before its first clip.
///
/// Fixed text: the drift check compares a fresh emit against the checked-in
/// file byte for byte, so nothing here may vary with the machine, the clock or
/// the input path.
const HEADER: &str = "\
# Everything the running system can play, as the cogs are handed it: the clips,
# and the motions that string them together.
#
# Protobuf text of `brenn_reachy.cogs.config_clk_proto.ClipLibraryConfig`,
# which the compiler generates from the `ClipLibraryConfig` schema in
# `config.clk`. A box binds it by path; the casing converts it to the message
# the dial hands the cog.
#
# Generated — do not edit. Regenerate with `make clip-config` after changing a
# document under `cogs/clips/`, and commit the three files together: this one,
# its name sidecar, and the documents.
#
# An asset's identity is its position here, so the order is load-bearing: it is
# the order the document paths sort in, and an asset inserted in the middle
# renumbers every one after it. The two numberings are separate: a clip id
# indexes the clips, a motion id indexes everything that plays.
#
# clip_id  name
";

#[derive(Debug)]
struct Args {
    /// The directory of clip documents.
    clips: PathBuf,
    /// Where the protobuf text is written.
    out: PathBuf,
    /// Where the name sidecar is written.
    names: PathBuf,
}

fn usage() -> String {
    "usage: gen-clip-config --clips DIR --out FILE --names FILE\n\
     \n\
     \x20 --clips DIR   a directory of clip and sequence documents; *.json in it\n\
     \x20 --out FILE    where the ClipLibraryConfig protobuf text is written\n\
     \x20 --names FILE  where the id-to-name sidecar is written\n\
     \n\
     Every document must load. An asset's id is its index in the emitted\n\
     library, which is the order the paths sort in, so a document that will not\n\
     load is refused rather than skipped: skipping one renumbers the rest."
        .to_owned()
}

fn main() -> anyhow::Result<()> {
    let args = parse(std::env::args().skip(1))?;
    run(&args, &mut |line| println!("{line}"))
}

fn parse(words: impl Iterator<Item = String>) -> anyhow::Result<Args> {
    let (mut clips, mut out, mut names) = (None, None, None);
    let mut words = words.peekable();
    while let Some(word) = words.next() {
        let mut value = |flag: &str| -> anyhow::Result<String> {
            words
                .next()
                .with_context(|| format!("{flag} wants a value\n\n{}", usage()))
        };
        match word.as_str() {
            "--clips" => clips = Some(PathBuf::from(value("--clips")?)),
            "--out" => out = Some(PathBuf::from(value("--out")?)),
            "--names" => names = Some(PathBuf::from(value("--names")?)),
            "-h" | "--help" => {
                println!("{}", usage());
                std::process::exit(0);
            }
            other => bail!("unknown argument {other}\n\n{}", usage()),
        }
    }
    let missing = |what: &str| anyhow::anyhow!("{what} is required\n\n{}", usage());
    Ok(Args {
        clips: clips.ok_or_else(|| missing("--clips"))?,
        out: out.ok_or_else(|| missing("--out"))?,
        names: names.ok_or_else(|| missing("--names"))?,
    })
}

/// Read the documents, emit both files, and say what was written.
fn run(args: &Args, say: &mut dyn FnMut(String)) -> anyhow::Result<()> {
    let texts = read_documents(&args.clips)?;
    let emitted = emit(&texts)?;
    write(&args.out, &emitted.textproto)?;
    write(&args.names, &emitted.names_json())?;
    emitted.report(say);
    say(format!(
        "{} clip(s) and {} motion(s) to {} and {}",
        emitted.clips.len(),
        emitted.motions.len(),
        args.out.display(),
        args.names.display()
    ));
    Ok(())
}

/// Every document under `dir`, by path ascending, text and all.
///
/// A file that will not read fails the emit rather than being carried as its own
/// error: the asset this writes is a numbering, and a numbering with a hole in
/// it is worse than no asset.
fn read_documents(dir: &Path) -> anyhow::Result<Vec<(String, String)>> {
    let entries =
        documents(dir).with_context(|| format!("cannot read the directory {}", dir.display()))?;
    if entries.is_empty() {
        bail!("no *.json under {}", dir.display());
    }
    entries
        .into_iter()
        .map(|(source, text)| {
            let text = text.with_context(|| format!("cannot read {source}"))?;
            Ok((source, text))
        })
        .collect()
}

/// What one emit produced: the asset, and the two numberings in it.
#[derive(Debug)]
struct Emitted {
    /// The protobuf text, ready to write.
    textproto: String,
    /// The clips, in clip-id order.
    clips: Numbering,
    /// The motions, in motion-id order.
    motions: Numbering,
    /// What the load changed about the assets it accepted, as lines.
    notes: Vec<String>,
}

/// One asset of a numbering: the name its id is looked up by, and how many
/// parts it carries.
#[derive(Debug)]
struct EmittedAsset {
    /// The library name, which is what a script or schedule author looks the id
    /// up by.
    name: String,
    /// How many parts it holds — frames for a clip, segments for a motion.
    parts: usize,
}

/// One numbering of the emit, and the words it is stated in.
///
/// Both numberings are the same thing — a name at a position with a count
/// beside it — and they go out three times each: as a report line, as a header
/// comment in the asset, and as a table in the sidecar. One type rendering all
/// three is what keeps a name table from drifting from the numbering the box
/// loads, which is a wrong-motion-invoked failure at the machine.
#[derive(Debug)]
struct Numbering {
    /// What one entry is: `clip` or `motion`. The report line's word and the
    /// sidecar's id key are both built from it.
    noun: &'static str,
    /// What an entry's parts are: `frame` or `segment`.
    part: &'static str,
    /// The assets, in id order.
    entries: Vec<EmittedAsset>,
}

impl Numbering {
    /// The numbering over `names`, each with the part count `parts` yields for
    /// it in the same order.
    fn of(
        noun: &'static str,
        part: &'static str,
        names: &[String],
        parts: impl IntoIterator<Item = usize>,
    ) -> Self {
        let entries = names
            .iter()
            .zip(parts)
            .map(|(name, parts)| EmittedAsset {
                name: name.clone(),
                parts,
            })
            .collect();
        Self {
            noun,
            part,
            entries,
        }
    }

    /// How many assets it numbers.
    fn len(&self) -> usize {
        self.entries.len()
    }

    /// The name at `id`.
    fn name(&self, id: usize) -> &str {
        &self.entries[id].name
    }

    /// One line per asset, under the id it is invoked by.
    fn report(&self, say: &mut dyn FnMut(String)) {
        for (id, asset) in self.entries.iter().enumerate() {
            say(format!(
                "{} {id}  {}  {} {}(s)",
                self.noun, asset.name, asset.parts, self.part
            ));
        }
    }

    /// The id-to-name table as the sidecar carries it.
    fn table(&self) -> Vec<serde_json::Value> {
        self.entries
            .iter()
            .enumerate()
            .map(|(id, asset)| {
                let mut row = serde_json::Map::new();
                row.insert(format!("{}_id", self.noun), json!(id));
                row.insert("name".to_owned(), json!(asset.name));
                serde_json::Value::Object(row)
            })
            .collect()
    }

    /// The id-to-name table as the comment lines at the head of the asset.
    fn header(&self, out: &mut String) {
        for (id, asset) in self.entries.iter().enumerate() {
            let _ = writeln!(out, "#   {id}  {}", asset.name);
        }
    }
}

impl Emitted {
    /// Say what the emit holds: whatever the load changed, then every clip and
    /// every motion under the id it is invoked by.
    ///
    /// A note goes out first because it is the only warning that an asset is not
    /// quite what its document authored, and the id lines are long.
    fn report(&self, say: &mut dyn FnMut(String)) {
        for note in &self.notes {
            say(format!("note: {note}"));
        }
        self.clips.report(say);
        self.motions.report(say);
    }

    /// The name sidecar: both id-to-name tables as JSON, for the host-side
    /// scripter that has to turn a name into the number the wire carries.
    ///
    /// Two tables, each keyed by its own id space. A schedule resolves names
    /// against the motions; the clips are there because a clip id is what a
    /// motion's segments name.
    fn names_json(&self) -> String {
        let table = json!({"clips": self.clips.table(), "motions": self.motions.table()});
        format!(
            "{}\n",
            serde_json::to_string_pretty(&table).expect("a table of strings and numbers is JSON")
        )
    }
}

/// Turn the documents into the asset, or refuse.
///
/// Pure: no clock, no filesystem, no environment. Everything the emitted bytes
/// depend on arrives in `texts`, which is what lets a case compare a fresh emit
/// against the checked-in file byte for byte.
fn emit(texts: &[(String, String)]) -> anyhow::Result<Emitted> {
    // The bounds the loader derives every clip's ceiling and blend floors
    // against: the machine's own, so what this accepts is what the tick can
    // command.
    let limits = ClipLimits::default();
    let (library, skips) = Library::load(
        texts.iter().map(|(source, text)| (source.clone(), text)),
        &limits,
    );
    if !skips.is_empty() {
        let listed: Vec<String> = skips
            .iter()
            .map(|skip| match &skip.name {
                Some(name) => format!("{}: {name}: {}", skip.source, skip.error),
                None => format!("{}: {}", skip.source, skip.error),
            })
            .collect();
        bail!(
            "{} document(s) would not load, so the numbering is refused:\n  {}",
            skips.len(),
            listed.join("\n  ")
        );
    }

    // Both numberings are the loader's own account of what it accepted, in the
    // order it read the documents. Taken from the loader rather than probed
    // here a second time: where an id is a position, two routings of one
    // directory that disagree renumber the library.
    let mut names = Vec::new();
    let mut clips: Vec<&Clip> = Vec::new();
    for asset in library.loaded() {
        let clip = library
            .clip(&asset.name)
            .with_context(|| format!("{}: {} is not in the library", asset.source, asset.name))?;
        names.push(asset.name.clone());
        clips.push(clip);
    }
    if clips.is_empty() {
        bail!("none of the documents is a clip");
    }

    // The motions are the second numbering, over every asset that plays: one
    // per clip, plus one per composed sequence. A schedule names these and
    // never the clips, so a bare clip is invoked the same way a composition is.
    let mut motion_names = Vec::new();
    let mut motions: Vec<&Motion> = Vec::new();
    for asset in library.motions_loaded() {
        let motion = library
            .motion(&asset.name)
            .with_context(|| format!("{}: {} is not a motion", asset.source, asset.name))?;
        motion_names.push(asset.name.clone());
        motions.push(motion);
    }

    let mut message = Box::new(ClipLibraryConfigWire::new());
    write_library(&clips, &motions, message.clear_valid())
        .context("the library does not fit the message")?;
    // What is written is read back the way a cog reads it -- one `validate()`
    // at the boundary, then the playability walk -- so this tool cannot emit an
    // asset the running system would refuse.
    let written = message
        .validate()
        .context("the emitted library is not a message this build can read")?;
    assert_eq!(
        written.clips.len(),
        names.len(),
        "every clip written has a name"
    );
    assert_eq!(
        written.motions.len(),
        motion_names.len(),
        "every motion written has a name"
    );
    ValidatedLibrary::of(written).map_err(|refusal| {
        let named = match refusal {
            UnplayableAsset::Clip { clip_id, .. } => format!("clip ({})", names[clip_id]),
            UnplayableAsset::Motion { motion_id, .. } => {
                format!("motion ({})", motion_names[motion_id])
            }
        };
        anyhow::Error::new(refusal).context(format!("{named} is not playable"))
    })?;

    let clips = Numbering::of(
        "clip",
        "frame",
        &names,
        clips.iter().map(|clip| clip.frames().len()),
    );
    let motions = Numbering::of(
        "motion",
        "segment",
        &motion_names,
        motions.iter().map(|motion| motion.segments().len()),
    );
    Ok(Emitted {
        textproto: print_library(written, &clips, &motions),
        clips,
        motions,
        notes: library.notes().iter().map(ToString::to_string).collect(),
    })
}

/// The message as the protobuf text a box binds.
///
/// Written from the message rather than from the clips, so the text states what
/// the mapping produced and not a second opinion about it.
fn print_library(library: &ClipLibraryConfig, clips: &Numbering, motions: &Numbering) -> String {
    let mut out = String::from(HEADER);
    clips.header(&mut out);
    let _ = writeln!(out, "#\n# motion_id  name");
    motions.header(&mut out);
    for (clip_id, clip) in library.clips.iter().enumerate() {
        let _ = writeln!(out, "\n# {}", clips.name(clip_id));
        let _ = writeln!(out, "clips {{");
        let _ = writeln!(out, "  mask: {}", clip.mask);
        let _ = writeln!(out, "  frame_rate_hz: {}", number(clip.frame_rate_hz));
        let _ = writeln!(out, "  blend_in_ms: {}", clip.blend_in_ms);
        let _ = writeln!(out, "  blend_out_ms: {}", clip.blend_out_ms);
        let _ = writeln!(out, "  max_speed: {}", number(clip.max_speed));
        for frame in clip.frames.iter() {
            print_frame(&mut out, frame);
        }
        let _ = writeln!(out, "}}");
    }
    for (motion_id, motion) in library.motions.iter().enumerate() {
        let _ = writeln!(out, "\n# {}", motions.name(motion_id));
        print_motion(&mut out, motion);
    }
    out
}

/// One motion: its lead gap, then a block per segment.
///
/// Every field is stated, zeros included, for the reason the frames are: the
/// generated protobuf conversion refuses a message with a field it was not told
/// about, so an omitted zero is a configuration that will not load.
fn print_motion(out: &mut String, motion: &MotionConfig) {
    let _ = writeln!(out, "motions {{");
    let _ = writeln!(out, "  lead_gap_ms: {}", motion.lead_gap_ms);
    for segment in motion.segments.iter() {
        let _ = writeln!(out, "  segments {{");
        let _ = writeln!(out, "    clip_id: {}", segment.clip_id);
        let _ = writeln!(out, "    speed: {}", number(segment.speed));
        let _ = writeln!(out, "    gap_after_ms: {}", segment.gap_after_ms);
        let _ = writeln!(out, "  }}");
    }
    let _ = writeln!(out, "}}");
}

/// One frame on one line: every field, in the schema's declared order.
///
/// Every field, including the zeros an unmasked channel is required to hold: the
/// generated protobuf conversion refuses a message with a field it was not told
/// about, so an omitted zero is a configuration that will not load rather than a
/// default. One line per frame because a frame is one instant and a clip is
/// hundreds of them.
fn print_frame(out: &mut String, frame: &ClipFrame) {
    let _ = write!(out, "  frames {{");
    for field in &FRAME_FIELDS {
        let _ = write!(out, " {}: {}", field.key, number((field.get)(frame)));
    }
    let _ = writeln!(out, " }}");
}

/// A double as text the protobuf parser reads back to the same bits.
///
/// `Debug` rather than `Display`: it is the shortest decimal that round-trips
/// and it uses an exponent where one is shorter, which `Display` never does —
/// a small delta would otherwise go out as a run of zeros. Non-finite values
/// cannot reach here; the loader refuses them.
fn number(value: f64) -> String {
    format!("{value:?}")
}

/// Write `text` to `path`, saying which file it was on the way out.
fn write(path: &Path, text: &str) -> anyhow::Result<()> {
    std::fs::write(path, text).with_context(|| format!("cannot write {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    use brenn_reachy__cogs__config_clk_rs::ClipFrameWire;
    use reachy_clips::config::{MAX_MOTIONS, MAX_SEGMENTS};

    /// The three checked-in documents, embedded rather than read: a case that
    /// compiles against the asset needs no runfiles and no working directory.
    const DOCUMENTS: [(&str, &str); 3] = [
        ("cogs/clips/nod.json", include_str!("clips/nod.json")),
        ("cogs/clips/perk.json", include_str!("clips/perk.json")),
        ("cogs/clips/sway.json", include_str!("clips/sway.json")),
    ];

    /// The asset those documents emit, as committed.
    const ASSET: &str = include_str!("clip_library.textproto");

    /// The name sidecar, as committed.
    const NAMES: &str = include_str!("clip_library.names.json");

    fn texts() -> Vec<(String, String)> {
        DOCUMENTS
            .iter()
            .map(|(source, text)| ((*source).to_owned(), (*text).to_owned()))
            .collect()
    }

    /// One document, as text, with `patch` applied to its parsed form.
    fn doc(text: &str, patch: impl FnOnce(&mut serde_json::Value)) -> String {
        let mut value: serde_json::Value = serde_json::from_str(text).expect("the document parses");
        patch(&mut value);
        serde_json::to_string(&value).expect("a patched document is JSON")
    }

    /// The whole point of the tool having a committed output: the emit is
    /// reproducible, so a change to a document, to the mapping, or to the
    /// printer that nobody regenerated for is a red test rather than an asset
    /// that no longer says what the documents do.
    #[test]
    fn the_committed_asset_is_what_the_committed_documents_emit() {
        let emitted = emit(&texts()).expect("the checked-in documents emit");
        assert_eq!(
            emitted.textproto, ASSET,
            "cogs/clip_library.textproto is stale; run `make clip-config`"
        );
        assert_eq!(
            emitted.names_json(),
            NAMES,
            "cogs/clip_library.names.json is stale; run `make clip-config`"
        );
    }

    /// The load's own opinion of the assets, which the emitter reports and does
    /// not silence. A note means the committed documents disagree with the
    /// derivation about something, and the operator gets to hear it.
    #[test]
    fn the_committed_documents_load_without_a_note() {
        let emitted = emit(&texts()).expect("the checked-in documents emit");
        assert!(emitted.notes.is_empty(), "notes: {:?}", emitted.notes);
    }

    /// A clip id is an index into the emitted order, which is the order the
    /// sources arrive in — not the alphabetical order of the names, which the
    /// library itself is keyed by.
    #[test]
    fn a_clip_id_is_the_position_the_document_was_read_in() {
        let mut reversed = texts();
        reversed.reverse();
        let forward = emit(&texts()).expect("the documents emit");
        let backward = emit(&reversed).expect("the documents emit either way");
        let names = |emitted: &Emitted| -> Vec<String> {
            emitted
                .clips
                .entries
                .iter()
                .map(|clip| clip.name.clone())
                .collect()
        };
        let mut flipped = names(&backward);
        flipped.reverse();
        assert_eq!(names(&forward), flipped);
        assert_ne!(
            forward.textproto, backward.textproto,
            "the order the documents arrive in is the numbering"
        );
    }

    /// The sidecar and the asset's own comment table are one table: a script
    /// author reading either gets the same id.
    #[test]
    fn the_sidecar_and_the_asset_agree_about_every_id() {
        let emitted = emit(&texts()).expect("the documents emit");
        let sidecar: serde_json::Value =
            serde_json::from_str(&emitted.names_json()).expect("the sidecar is JSON");
        let listed = sidecar["clips"].as_array().expect("clips is an array");
        assert_eq!(listed.len(), emitted.clips.len());
        for (clip_id, clip) in emitted.clips.entries.iter().enumerate() {
            assert_eq!(listed[clip_id]["clip_id"], clip_id);
            assert_eq!(listed[clip_id]["name"], clip.name);
            assert!(
                emitted
                    .textproto
                    .contains(&format!("#   {clip_id}  {}", clip.name)),
                "the asset's table is missing {clip_id}"
            );
        }
    }

    /// A document that will not load is not a skip here. The whole emit is
    /// refused, because dropping one clip renumbers the rest.
    #[test]
    fn one_document_that_will_not_load_refuses_the_whole_emit() {
        let mut broken = texts();
        broken[1].1 = doc(&broken[1].1, |value| {
            value["frame_hz"] = json!(30.0);
        });
        let error = emit(&broken).expect_err("a clip on another grid is refused");
        let text = format!("{error:#}");
        assert!(text.contains("numbering is refused"), "{text}");
        assert!(text.contains("perk"), "{text}");
    }

    /// Two documents claiming one name is the authoring mistake that attacks the
    /// numbering directly: the loader keeps one and skips the other, ids come
    /// from document order, and a library where one document silently vanished
    /// is one every later script id is wrong against. Refused, like any skip.
    #[test]
    fn two_documents_claiming_one_name_refuse_the_whole_emit() {
        let mut clashing = texts();
        clashing[1].1 = doc(&clashing[1].1, |value| {
            value["name"] = json!("bench/nod");
        });
        let error = emit(&clashing).expect_err("a duplicate name is refused");
        let text = format!("{error:#}");
        assert!(text.contains("numbering is refused"), "{text}");
        assert!(text.contains("bench/nod"), "{text}");
        assert!(text.contains("perk.json"), "{text}");
    }

    /// Documents that hold no clip at all: the tool has nothing to emit and says
    /// so, rather than writing a library a box would bind and a cog would find
    /// empty.
    #[test]
    fn documents_with_no_clip_in_them_are_refused() {
        let error = emit(&[]).expect_err("there is no library in nothing");
        assert!(
            format!("{error:#}").contains("none of the documents is a clip"),
            "{error:#}"
        );
    }

    /// What the loader changed about a clip is the operator's to hear: the note
    /// is the only warning that the asset's blends are not the ones the document
    /// authored.
    #[test]
    fn a_derivation_that_changed_a_clip_is_reported() {
        let mut stretched = texts();
        stretched[0].1 = doc(&stretched[0].1, |value| {
            value["blend_in_ms"] = json!(1);
        });
        let emitted = emit(&stretched).expect("a stretched ramp still emits");
        assert_eq!(emitted.notes.len(), 1, "notes: {:?}", emitted.notes);
        assert!(
            emitted.notes[0].contains("bench/nod") && emitted.notes[0].contains("stretched"),
            "{:?}",
            emitted.notes
        );
        let mut said = Vec::new();
        emitted.report(&mut |line| said.push(line));
        assert!(
            said.iter()
                .any(|line| line.starts_with("note: ") && line.contains("bench/nod")),
            "{said:?}"
        );
    }

    /// A sequence document, as JSON, over the entries given.
    fn sequence(name: &str, entries: serde_json::Value) -> String {
        serde_json::to_string(&json!({
            "version": 1,
            "kind": "sequence",
            "name": name,
            "entries": entries,
        }))
        .expect("the sequence is JSON")
    }

    /// A sequence emits as a motion of several segments, resolved and flattened
    /// at load: the clip ids its segments name, the speeds the nesting produced,
    /// and the holds between them all cross as numbers, so nothing downstream
    /// ever walks a reference.
    #[test]
    fn a_sequence_document_emits_as_a_multi_segment_motion() {
        let mut with_sequence = texts();
        with_sequence.push((
            "cogs/clips/zgreeting.json".to_owned(),
            sequence(
                "bench/greeting",
                json!([
                    {"gap_ms": 250},
                    {"ref": "bench/nod"},
                    {"gap_ms": 300},
                    {"ref": "bench/sway", "speed": 2.0},
                ]),
            ),
        ));
        let emitted = emit(&with_sequence).expect("a sequence emits");

        // Three clips, four motions: one per clip, then the sequence.
        assert_eq!(emitted.clips.len(), 3);
        let names: Vec<&str> = emitted
            .motions
            .entries
            .iter()
            .map(|motion| motion.name.as_str())
            .collect();
        assert_eq!(
            names,
            ["bench/nod", "bench/perk", "bench/sway", "bench/greeting"]
        );

        // Two clips strung together, with the leading gap held apart from them.
        assert_eq!(emitted.motions.entries[3].parts, 2);
        assert!(
            emitted.textproto.contains("#   3  bench/greeting"),
            "{}",
            emitted.textproto
        );

        let printed = emitted
            .textproto
            .split("\n# bench/greeting\nmotions {\n")
            .nth(1)
            .expect("the greeting motion is printed");
        assert!(printed.starts_with("  lead_gap_ms: 250\n"), "{printed}");
        assert!(
            printed.contains("    clip_id: 0\n    speed: 1.0\n    gap_after_ms: 300\n"),
            "{printed}"
        );
        assert!(
            printed.contains("    clip_id: 2\n    speed: 2.0\n    gap_after_ms: 0\n"),
            "{printed}"
        );
    }

    /// A one-segment motion stands for every clip, so a schedule names motions
    /// only and invoking a bare clip costs nothing.
    #[test]
    fn every_clip_is_also_a_motion_of_one_segment() {
        let emitted = emit(&texts()).expect("the checked-in documents emit");
        assert_eq!(emitted.clips.len(), emitted.motions.len());
        for (clip, motion) in emitted.clips.entries.iter().zip(&emitted.motions.entries) {
            assert_eq!(clip.name, motion.name);
            assert_eq!(motion.parts, 1);
        }
    }

    /// The message's motion bound, refused rather than truncated. Sixteen clips
    /// are sixteen motions already, so seventeen sequences over them are what
    /// carries the count past the bound.
    #[test]
    fn more_motions_than_the_message_holds_is_a_refusal() {
        let one = texts().remove(0);
        let mut many: Vec<(String, String)> = vec![(
            "cogs/clips/aclip.json".to_owned(),
            doc(&one.1, |value| {
                value["name"] = json!("bench/clip");
            }),
        )];
        for index in 0..MAX_MOTIONS {
            many.push((
                format!("cogs/clips/s{index:02}.json"),
                sequence(&format!("bench/seq{index}"), json!([{"ref": "bench/clip"}])),
            ));
        }
        let error = emit(&many).expect_err("thirty-three motions do not fit");
        let text = format!("{error:#}");
        assert!(text.contains("does not fit the message"), "{text}");
        assert!(
            text.contains(&format!("library has {} motions", MAX_MOTIONS + 1)),
            "the refusal is not the motion bound's: {text}"
        );
    }

    /// A flattened motion longer than the message's segment array is refused
    /// where it is flattened — before the numbering, and so before any emit —
    /// rather than truncated into a motion that plays part of itself.
    #[test]
    fn more_segments_than_the_message_holds_is_a_refusal() {
        let one = texts().remove(0);
        let entries: Vec<serde_json::Value> = (0..=MAX_SEGMENTS)
            .map(|_| json!({"ref": "bench/clip"}))
            .collect();
        let many = vec![
            (
                "cogs/clips/aclip.json".to_owned(),
                doc(&one.1, |value| {
                    value["name"] = json!("bench/clip");
                }),
            ),
            (
                "cogs/clips/zlong.json".to_owned(),
                sequence("bench/long", json!(entries)),
            ),
        ];
        let error = emit(&many).expect_err("thirty-three segments do not fit");
        let text = format!("{error:#}");
        assert!(text.contains("numbering is refused"), "{text}");
        assert!(
            text.contains("bench/long") && text.contains(&format!("past {MAX_SEGMENTS} segments")),
            "the refusal is not the segment bound's: {text}"
        );
    }

    /// A sequence that loads as a document and then will not resolve is a skip
    /// like any other, and a skip refuses the whole emit: dropping one asset
    /// renumbers every motion after it, and a schedule authored against the old
    /// sidecar would then invoke a different motion on the machine. The listing
    /// names the sequence — the asset — as well as the file.
    #[test]
    fn a_sequence_that_will_not_resolve_refuses_the_whole_emit() {
        let mut dangling = texts();
        dangling.push((
            "cogs/clips/zgreeting.json".to_owned(),
            sequence("bench/greeting", json!([{"ref": "bench/nope"}])),
        ));
        let error = emit(&dangling).expect_err("a reference to nothing is refused");
        let text = format!("{error:#}");
        assert!(text.contains("numbering is refused"), "{text}");
        assert!(text.contains("bench/greeting"), "{text}");
        assert!(text.contains("bench/nope"), "{text}");
    }

    /// The message's own bound, refused rather than truncated.
    #[test]
    fn more_clips_than_the_message_holds_is_a_refusal() {
        let one = texts().remove(0);
        let many: Vec<(String, String)> = (0..=reachy_clips::config::MAX_CLIPS)
            .map(|index| {
                (
                    format!("cogs/clips/clip{index}.json"),
                    doc(&one.1, |value| {
                        value["name"] = json!(format!("bench/clip{index}"));
                    }),
                )
            })
            .collect();
        let error = emit(&many).expect_err("seventeen clips do not fit");
        assert!(
            format!("{error:#}").contains("does not fit the message"),
            "{error:#}"
        );
    }

    /// A frame states every field, zeros included, in the schema's declared
    /// order. The zeros are not decoration: the protobuf conversion refuses a
    /// field it was not told about, so an asset with an implicit zero in it is
    /// an asset that will not load.
    #[test]
    fn a_frame_states_every_field_in_declared_order() {
        let mut message = ClipFrameWire::new();
        let frame = message.clear_valid();
        frame.quat_w = 1.0;
        frame.head_dz = -0.25;
        let mut out = String::new();
        print_frame(&mut out, frame);
        assert_eq!(
            out,
            "  frames { head_dx: 0.0 head_dy: 0.0 head_dz: -0.25 quat_w: 1.0 \
             quat_x: 0.0 quat_y: 0.0 quat_z: 0.0 body_yaw_d: 0.0 \
             antenna_right_d: 0.0 antenna_left_d: 0.0 }\n"
        );
    }

    /// Every field of the frame is printable, and each one is printed under its
    /// own key: a table that pairs a name with the wrong accessor is a silently
    /// wrong asset.
    #[test]
    fn every_frame_field_prints_under_its_own_key() {
        for (index, field) in FRAME_FIELDS.iter().enumerate() {
            let mut message = ClipFrameWire::new();
            let frame = message.clear_valid();
            let value = 1.0 + index as f64;
            (field.set)(frame, value);
            let mut out = String::new();
            print_frame(&mut out, frame);
            assert!(
                out.contains(&format!(" {}: {value:?} ", field.key)),
                "{} did not print its own value: {out}",
                field.key
            );
            assert_eq!(
                out.matches(": 0.0").count(),
                FRAME_FIELDS.len() - 1,
                "every other field is zero: {out}"
            );
        }
    }

    /// The printed number is the shortest decimal that reads back to the same
    /// bits, exponent and all: an asset the parser rounds is an asset that plays
    /// a different motion than the document recorded.
    #[test]
    fn a_number_goes_out_as_text_that_reads_back_to_the_same_bits() {
        for value in [
            0.1,
            1.0,
            -0.25,
            1.0e-9,
            f64::MIN_POSITIVE,
            f64::MAX,
            std::f64::consts::PI,
        ] {
            let text = number(value);
            let read: f64 = text.parse().expect("the printed number parses");
            assert_eq!(read.to_bits(), value.to_bits(), "{text}");
        }
    }

    /// The arguments, and the two ways of getting them wrong.
    #[test]
    fn the_arguments_are_all_three_or_a_refusal() {
        fn words(line: &str) -> impl Iterator<Item = String> + '_ {
            line.split_whitespace().map(ToOwned::to_owned)
        }
        let args = parse(words("--clips a --out b --names c")).expect("all three are given");
        assert_eq!(args.clips, PathBuf::from("a"));
        assert_eq!(args.out, PathBuf::from("b"));
        assert_eq!(args.names, PathBuf::from("c"));

        let missing = parse(words("--clips a --out b")).expect_err("--names is required");
        assert!(format!("{missing:#}").contains("--names is required"));
        let dangling = parse(words("--clips")).expect_err("a flag wants a value");
        assert!(format!("{dangling:#}").contains("--clips wants a value"));
        let unknown = parse(words("--library a")).expect_err("an unknown flag is refused");
        assert!(format!("{unknown:#}").contains("unknown argument --library"));
    }

    /// The whole tool over a directory, which is the one thing the pure emit
    /// cannot cover: what it reads, what it writes, and what it says.
    #[test]
    fn the_tool_writes_both_files_and_reports_every_clip() {
        let dir = std::env::temp_dir().join(format!(
            "gen-clip-config-{}-{}",
            std::process::id(),
            line!()
        ));
        let clips = dir.join("clips");
        std::fs::create_dir_all(&clips).expect("a temporary directory");
        for (source, text) in DOCUMENTS {
            let name = Path::new(source).file_name().expect("a file name");
            std::fs::write(clips.join(name), text).expect("the document is written");
        }
        let args = Args {
            clips,
            out: dir.join("clip_library.textproto"),
            names: dir.join("clip_library.names.json"),
        };
        let mut said = Vec::new();
        run(&args, &mut |line| said.push(line)).expect("the tool runs");
        assert_eq!(
            std::fs::read_to_string(&args.out).expect("the asset was written"),
            ASSET
        );
        assert_eq!(
            std::fs::read_to_string(&args.names).expect("the sidecar was written"),
            NAMES
        );
        assert!(
            said.iter().any(|line| line.contains("bench/nod")),
            "{said:?}"
        );
        assert!(
            said.last()
                .expect("a closing line")
                .contains("3 clip(s) and 3 motion(s)"),
            "{said:?}"
        );
        std::fs::remove_dir_all(&dir).expect("the temporary directory goes away");
    }

    /// An empty directory is the caller's own configuration being wrong, and
    /// says so rather than emitting a library with no clips.
    #[test]
    fn an_empty_directory_is_refused() {
        let dir =
            std::env::temp_dir().join(format!("gen-clip-config-empty-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("a temporary directory");
        let error = read_documents(&dir).expect_err("nothing to read");
        assert!(format!("{error:#}").contains("no *.json"), "{error:#}");
        std::fs::remove_dir_all(&dir).expect("the temporary directory goes away");
    }
}
