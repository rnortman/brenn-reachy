//! `gen-library-config` — the authored documents in, the library
//! configurations out.
//!
//! An offline host-side emitter over two libraries, which are separate assets
//! and separate vocabularies: the clips and motions a script layers over
//! whatever base is standing, and the poses a base step moves the whole machine
//! to. One tool because they share a sidecar — a script author resolves both
//! kinds of name out of one file — and because a single command that emits
//! everything cannot leave one of them behind.
//!
//! It reads each directory the way the process that uses it reads it: the clip
//! documents through `files::documents` into `Library::load` under the
//! machine's own `ClipLimits`, which resolves and flattens every sequence; the
//! pose documents through [`reachy_poses::format::Pose::from_text`], which runs
//! the envelope check on each. What it finds goes into the two messages through
//! the one mapping each has ([`reachy_clips::config::write_library`],
//! [`reachy_poses::config::write_library`]), and each message is printed as the
//! protobuf text a cog's configuration dial takes by path — the clip library
//! bound by a box today, the pose library emitted for the dials that will read
//! it and staged with them.
//!
//! An asset's identity is its position, in three numberings: `clip_id` indexes
//! the clips, `motion_id` indexes everything that plays — a motion per clip,
//! plus one per sequence — and `pose_id` indexes the poses. All three are the
//! order the documents' paths sort in. So a document that will not load is a
//! **refusal of the whole emit**, not a skip the way a running host would take
//! it: dropping one asset renumbers every one after it, and a script authored
//! against the old numbering would then invoke the wrong motion or move to the
//! wrong pose. The name tables ride out twice for the same reason — as comments
//! at the head of each asset, and as a JSON sidecar for whoever authors
//! scripts.
//!
//! Nothing here decides anything about motion. The validation is each loader's,
//! the mapping is each `config` module's, and both emitted assets are re-read
//! the way a cog reads them before they are written, so this tool cannot
//! produce a file the cogs would refuse.

#![forbid(unsafe_code)]

use std::fmt::Write as _;
use std::path::{Path, PathBuf};

use anyhow::{Context as _, bail};
use serde_json::json;

use brenn_reachy__cogs__config_clk_rs::{
    ClipFrame, ClipLibraryConfig, ClipLibraryConfigWire, MotionConfig, PoseConfig,
    PoseLibraryConfig, PoseLibraryConfigWire,
};
use reachy_clips::config::{FRAME_FIELDS, UnplayableAsset, ValidatedLibrary, write_library};
use reachy_clips::envelope::ClipLimits;
use reachy_clips::files::{DOCUMENT_EXT, Descend, documents};
use reachy_clips::format::Clip;
use reachy_clips::library::{Library, Motion};
use reachy_poses::config::{PACE_FIELD, POSE_FIELDS};
use reachy_poses::format::Pose as LoadedPose;

mod probe_clips;

use probe_clips::{Folds, PROBES};

/// What the emitted clip asset says about itself before its first clip.
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
# Generated — do not edit. Regenerate with `make library-config` after changing
# a document under `cogs/clips/`, and commit the three files together: this one,
# its name sidecar, and the documents.
#
# An asset's identity is its position here, so the order is load-bearing: it is
# the order the document paths sort in, and an asset inserted in the middle
# renumbers every one after it. The two numberings are separate: a clip id
# indexes the clips, a motion id indexes everything that plays.
#
# clip_id  name
";

/// What the emitted pose asset says about itself before its first pose.
///
/// Fixed text, for the reason [`HEADER`] is.
const POSE_HEADER: &str = "\
# Every base posture the machine has a name for: where the head, the body yaw
# and the antennas stand, and how fast a move that states no pace of its own
# goes there.
#
# Protobuf text of `brenn_reachy.cogs.config_clk_proto.PoseLibraryConfig`,
# which the compiler generates from the `PoseLibraryConfig` schema in
# `config.clk`. This is the asset a cog's configuration dial takes by path; the
# casing converts it to the message the dial hands the cog. Every box that has
# to agree about where a pose puts the machine binds this one file, and it
# rides to a unit in `cogs/BUILD.bazel`'s `robot_config_files`.
#
# Generated — do not edit. Regenerate with `make library-config` after changing
# a document under `cogs/poses/`, and commit the three files together: this one,
# its name sidecar, and the documents.
#
# A pose's identity is its position here, so the order is load-bearing: it is
# the order the document paths sort in, and a pose inserted in the middle
# renumbers every one after it. The `stow` field carries the index of the pose
# named `stow`, which is where the machine rests: what every schedule ends at,
# what the fault ladder commands, and what the disarm sequence judges folded
# against.
#
# The head is stated relative to the neutral head pose, in metres and a unit
# quaternion; the antennas are directions in radians, right then left.
#
# pose_id  name
";

#[derive(Debug)]
struct Args {
    /// The directory of clip documents.
    clips: PathBuf,
    /// Where the clip library's protobuf text is written.
    out: PathBuf,
    /// The directory of pose documents.
    poses: PathBuf,
    /// Where the pose library's protobuf text is written.
    poses_out: PathBuf,
    /// Where the name sidecar both libraries share is written.
    names: PathBuf,
}

fn usage() -> String {
    "usage: gen-library-config --clips DIR --out FILE --poses DIR --poses-out FILE --names FILE\n\
     \n\
     \x20 --clips DIR       a directory of clip and sequence documents; *.json in it\n\
     \x20 --out FILE        where the ClipLibraryConfig protobuf text is written\n\
     \x20 --poses DIR       a directory of pose documents; *.textproto in it\n\
     \x20 --poses-out FILE  where the PoseLibraryConfig protobuf text is written\n\
     \x20 --names FILE      where the id-to-name sidecar for both is written\n\
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
    let (mut clips, mut out, mut poses, mut poses_out, mut names) = (None, None, None, None, None);
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
            "--poses" => poses = Some(PathBuf::from(value("--poses")?)),
            "--poses-out" => poses_out = Some(PathBuf::from(value("--poses-out")?)),
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
        poses: poses.ok_or_else(|| missing("--poses"))?,
        poses_out: poses_out.ok_or_else(|| missing("--poses-out"))?,
        names: names.ok_or_else(|| missing("--names"))?,
    })
}

/// Read the documents, emit all three files, and say what was written.
///
/// The poses are read before the probes are written because the probes step
/// between the antenna folds the pose documents author: a probe is an
/// instrument whose poses are the machine's own, and the machine's own are now
/// the library's.
fn run(args: &Args, say: &mut dyn FnMut(String)) -> anyhow::Result<()> {
    let pose_texts = read_documents(&args.poses, reachy_poses::format::DOCUMENT_EXT, Descend::No)?;
    let poses = load_poses(&pose_texts)?;
    write_probes(&args.clips, &Folds::of(&poses)?, say)?;
    let texts = read_documents(&args.clips, DOCUMENT_EXT, Descend::Yes)?;
    let emitted = emit(&texts, &poses)?;
    write(&args.out, &emitted.textproto)?;
    write(&args.poses_out, &emitted.poses_textproto)?;
    write(&args.names, &emitted.names_json())?;
    emitted.report(say);
    say(format!(
        "{} clip(s), {} motion(s) and {} pose(s) to {}, {} and {}",
        emitted.clips.len(),
        emitted.motions.len(),
        emitted.poses.len(),
        args.out.display(),
        args.poses_out.display(),
        args.names.display()
    ));
    Ok(())
}

/// Write every probe document from its table, before the walk reads them.
///
/// The probes are instruments whose poses are the library's own folds, so they
/// are authored here rather than by hand: a document holding hundreds of copies
/// of a delta is re-transcribed whenever the fold behind it moves, and a
/// stale one loads and emits exactly as happily as a fresh one. The rest of the
/// library is recorded content and arrives as documents.
///
/// The write is part of the emit rather than a target of its own so that `make
/// library-config` is one command for both halves, and so the asset can never
/// be regenerated from probe documents the table has moved on from.
fn write_probes(clips: &Path, folds: &Folds, say: &mut dyn FnMut(String)) -> anyhow::Result<()> {
    for probe in PROBES {
        let path = probe.path(clips);
        let document = probe.document(folds).with_context(|| {
            format!("{}: the probe table does not author a document", probe.name)
        })?;
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("cannot make {}", parent.display()))?;
        }
        write(&path, &document)?;
        say(format!("probe {} to {}", probe.name, path.display()));
    }
    Ok(())
}

/// Every `ext` document under `dir`, by path ascending, text and all.
///
/// One walk for both libraries, through the rule's one home
/// ([`reachy_clips::files`]): what differs between them is the extension and
/// whether the walk descends — the clips are a tree that grows a subdirectory
/// per imported set, the poses a flat handful authored one at a time — and not
/// which entries count or the sort that fixes an id.
///
/// A file that will not read fails the emit rather than being carried as its own
/// error: each asset this writes is a numbering, and a numbering with a hole in
/// it is worse than no asset.
fn read_documents(
    dir: &Path,
    ext: &str,
    descend: Descend,
) -> anyhow::Result<Vec<(String, String)>> {
    let entries = documents(dir, ext, descend)
        .with_context(|| format!("cannot read the directory {}", dir.display()))?;
    if entries.is_empty() {
        bail!("no *.{ext} under {}", dir.display());
    }
    entries
        .into_iter()
        .map(|(source, text)| {
            let text = text.with_context(|| format!("cannot read {source}"))?;
            Ok((source, text))
        })
        .collect()
}

/// The pose documents, loaded, in the order they were read.
///
/// Every document is run through the loader the cogs' own screen sits behind —
/// the envelope check included — so a pose outside what the machine may be
/// commanded to never becomes an asset. A document that will not load refuses
/// the whole emit, as a clip document's does, and so does one whose name is not
/// its file stem: the stem is how an author finds the document a script names,
/// and the two disagreeing is a library nobody can navigate.
fn load_poses(texts: &[(String, String)]) -> anyhow::Result<Vec<LoadedPose>> {
    let mut poses = Vec::with_capacity(texts.len());
    for (source, text) in texts {
        let pose = LoadedPose::from_text(text)
            .with_context(|| format!("{source} is not a pose document"))?;
        let stem = Path::new(source)
            .file_stem()
            .map(|stem| stem.to_string_lossy().into_owned())
            .unwrap_or_default();
        if pose.name() != stem {
            bail!(
                "{source} authors the pose {:?}; a pose document's name is its file stem",
                pose.name()
            );
        }
        poses.push(pose);
    }
    Ok(poses)
}

/// What one emit produced: the two assets, and the three numberings in them.
#[derive(Debug)]
struct Emitted {
    /// The clip library's protobuf text, ready to write.
    textproto: String,
    /// The pose library's protobuf text, ready to write.
    poses_textproto: String,
    /// The clips, in clip-id order.
    clips: Numbering,
    /// The motions, in motion-id order.
    motions: Numbering,
    /// The poses, in pose-id order.
    poses: Numbering,
    /// What the load changed about the assets it accepted, as lines.
    notes: Vec<String>,
}

/// One asset of a numbering: the name its id is looked up by, and whatever its
/// kind states beside the name.
#[derive(Debug)]
struct EmittedAsset {
    /// The library name, which is what a script or schedule author looks the id
    /// up by.
    name: String,
    /// How many parts it holds — frames for a clip, segments for a motion.
    /// `None` where the kind has no parts: a pose is one configuration.
    parts: Option<usize>,
    /// What its kind states beside the name, where it states anything.
    extra: Option<Extra>,
}

/// The columns a numbering carries beside a name, by kind.
#[derive(Clone, Copy, Debug)]
enum Extra {
    /// How long invoking a motion occupies a timeline.
    Window(MotionWindow),
    /// The pace of a move to a pose when the command states none, milliseconds.
    Pace(u32),
}

/// How long a motion occupies a timeline, as the sidecar states it.
///
/// Two numbers rather than one because an invocation speed divides only the
/// first: the motion's own clock scales, and the blend-out that follows runs on
/// the wall clock at any speed, being the ramp that keeps the machine's per-tick
/// bounds satisfied.
#[derive(Clone, Copy, Debug)]
struct MotionWindow {
    /// The motion's own length at 1.0x, milliseconds.
    duration_ms: u64,
    /// The exit ramp, milliseconds.
    blend_out_ms: u64,
}

/// One numbering of the emit, and the words it is stated in.
///
/// All three numberings are the same thing — a name at a position, with
/// whatever its kind states beside it — and each goes out three times: as a
/// report line, as a header comment in the asset, and as a table in the
/// sidecar. One type rendering all three is what keeps a name table from
/// drifting from the numbering the box binds, which is a wrong-motion-invoked
/// or wrong-pose-commanded failure at the machine. What differs between the
/// kinds is columns, not rendering: a clip counts frames, a motion counts
/// segments and occupies a window, a pose counts nothing and states a pace.
#[derive(Debug)]
struct Numbering {
    /// What one entry is: `clip`, `motion` or `pose`. The report line's word
    /// and the sidecar's id key are both built from it.
    noun: &'static str,
    /// What an entry's parts are: `frame` or `segment`. `Some` exactly when the
    /// entries carry a part count.
    part: Option<&'static str>,
    /// The assets, in id order.
    entries: Vec<EmittedAsset>,
    /// The id the library reserves, where it reserves one: the pose named
    /// `stow`, which the report marks.
    reserved: Option<u16>,
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
                parts: Some(parts),
                extra: None,
            })
            .collect();
        Self {
            noun,
            part: Some(part),
            entries,
            reserved: None,
        }
    }

    /// The numbering over the poses of an emitted library: the name each was
    /// loaded under, the pace the message states for it, and the id the library
    /// reserves for the stow.
    ///
    /// The pace comes from the written message and not from the document it was
    /// written from, for the reason the motions' windows come off the validated
    /// library: the sidecar states the number the edge compiles a step against,
    /// and a second derivation of it is a second opinion.
    fn of_poses(entries: impl IntoIterator<Item = (String, u32)>, stow: u16) -> Self {
        Self {
            noun: "pose",
            part: None,
            entries: entries
                .into_iter()
                .map(|(name, duration_ms)| EmittedAsset {
                    name,
                    parts: None,
                    extra: Some(Extra::Pace(duration_ms)),
                })
                .collect(),
            reserved: Some(stow),
        }
    }

    /// The same numbering with a window against every entry, in id order.
    ///
    /// The windows come from the library the emit validated, so what the sidecar
    /// states about a motion's length is what the player derives from the same
    /// frames — never a second measurement.
    ///
    /// # Panics
    ///
    /// If `windows` does not hold one per entry, which would put a window
    /// against the wrong id.
    fn windowed(mut self, windows: Vec<MotionWindow>) -> Self {
        assert_eq!(
            windows.len(),
            self.entries.len(),
            "one window per numbered asset"
        );
        for (entry, window) in self.entries.iter_mut().zip(windows) {
            entry.extra = Some(Extra::Window(window));
        }
        self
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
            let mut line = format!("{} {id}  {}", self.noun, asset.name);
            if let (Some(parts), Some(part)) = (asset.parts, self.part) {
                let _ = write!(line, "  {parts} {part}(s)");
            }
            if let Some(Extra::Pace(duration_ms)) = asset.extra {
                let _ = write!(line, "  {duration_ms} ms");
            }
            if matches!(self.reserved, Some(reserved) if u16::try_from(id) == Ok(reserved)) {
                line.push_str("  (the stow)");
            }
            say(line);
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
                match asset.extra {
                    Some(Extra::Window(window)) => {
                        row.insert("duration_ms".to_owned(), json!(window.duration_ms));
                        row.insert("blend_out_ms".to_owned(), json!(window.blend_out_ms));
                    }
                    Some(Extra::Pace(duration_ms)) => {
                        row.insert("duration_ms".to_owned(), json!(duration_ms));
                    }
                    None => {}
                }
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
        self.poses.report(say);
    }

    /// The name sidecar: all three id-to-name tables as JSON, for the host-side
    /// scripter that has to turn a name into the number the wire carries.
    ///
    /// Three tables, each keyed by its own id space. A script's `play` step
    /// resolves against the motions and its base step against the poses; the
    /// clips are there because a clip id is what a motion's segments name. A
    /// motion row also carries its window and a pose row its pace, which is what
    /// an edge compiling a step into a timed one needs and cannot derive: the
    /// assets are not deployed where the compile runs.
    fn names_json(&self) -> String {
        let table = json!({
            "clips": self.clips.table(),
            "motions": self.motions.table(),
            "poses": self.poses.table(),
        });
        format!(
            "{}\n",
            serde_json::to_string_pretty(&table).expect("a table of strings and numbers is JSON")
        )
    }
}

/// Turn the documents into the two assets, or refuse.
///
/// Pure: no clock, no filesystem, no environment. Everything the emitted bytes
/// depend on arrives in `texts` and `poses`, which is what lets a case compare
/// a fresh emit against the checked-in files byte for byte.
fn emit(texts: &[(String, String)], poses: &[LoadedPose]) -> anyhow::Result<Emitted> {
    // The geometry and envelope the loader walks every clip's frames against:
    // the machine's own, so what this accepts is what the tick can command.
    let limits = ClipLimits::default();
    let (library, skips) = Library::load_resolved(
        texts.iter().map(|(source, text)| (source.clone(), text)),
        &limits,
        |name| {
            poses
                .iter()
                .find(|pose| pose.name() == name)
                .map(|pose| *pose.targets())
        },
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

    let mut message = ClipLibraryConfigWire::new_boxed();
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
    let checked = ValidatedLibrary::of(written).map_err(|refusal| {
        let named = match refusal {
            UnplayableAsset::Clip { clip_id, .. } => format!("clip ({})", names[clip_id]),
            UnplayableAsset::Motion { motion_id, .. } => {
                format!("motion ({})", motion_names[motion_id])
            }
        };
        anyhow::Error::new(refusal).context(format!("{named} is not playable"))
    })?;

    // The windows are read off the library that just validated, which derives
    // them from the frames: a script's compiler needs the same numbers the
    // player will use, and a second derivation is a second opinion.
    let mut windows = Vec::with_capacity(written.motions.len());
    for (motion_id, name) in motion_names.iter().enumerate() {
        let view = checked
            .playable_motion(motion_id)
            .with_context(|| format!("motion ({name}) has no window"))?;
        windows.push(MotionWindow {
            duration_ms: ms_ceil(view.duration_s()),
            blend_out_ms: u64::from(view.blend_out_ms()),
        });
    }

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
    )
    .windowed(windows);

    // The pose library, written and read back the same way: the mapping's, then
    // the message's own validation, then the screen every consumer runs on the
    // library its dial hands it. What this writes is therefore a library the
    // session will not refuse at startup.
    let mut pose_message = PoseLibraryConfigWire::new_boxed();
    reachy_poses::config::write_library(poses, pose_message.clear_valid())
        .context("the poses are not a library")?;
    let pose_written = pose_message
        .validate()
        .context("the emitted pose library is not a message this build can read")?;
    reachy_poses::config::screen(pose_written)
        .context("the emitted pose library does not screen")?;
    assert_eq!(
        pose_written.poses.len(),
        poses.len(),
        "every pose written has a name"
    );
    let numbered_poses = Numbering::of_poses(
        poses
            .iter()
            .zip(pose_written.poses.iter())
            .map(|(pose, written)| (pose.name().to_owned(), pace_ms(written.duration_ns))),
        pose_written.stow,
    );

    Ok(Emitted {
        textproto: print_library(written, &clips, &motions),
        poses_textproto: print_poses(pose_written, &numbered_poses),
        clips,
        motions,
        poses: numbered_poses,
        notes: library.notes().iter().map(ToString::to_string).collect(),
    })
}

/// The pose message as the protobuf text a box binds.
///
/// Written from the message rather than from the loaded poses, so the text
/// states what the mapping produced and not a second opinion about it. Every
/// field is stated, zeros included, for the reason a frame's are: the generated
/// conversion gives every field explicit presence, so an omitted zero is a
/// configuration that will not load rather than a default.
fn print_poses(library: &PoseLibraryConfig, poses: &Numbering) -> String {
    let mut out = String::from(POSE_HEADER);
    poses.header(&mut out);
    for (pose_id, pose) in library.poses.iter().enumerate() {
        let _ = writeln!(out, "\n# {}", poses.name(pose_id));
        print_pose(&mut out, pose);
    }
    let _ = writeln!(
        out,
        "\n# where the machine rests: the pose named {:?}\nstow: {}",
        poses.name(usize::from(library.stow)),
        library.stow
    );
    out
}

/// One pose: every field of it, in the schema's declared order.
///
/// The fields are the mapping's own table, as a frame's are: a channel added to
/// a pose is a row there and not an edit here.
fn print_pose(out: &mut String, pose: &PoseConfig) {
    let _ = writeln!(out, "poses {{");
    for field in &POSE_FIELDS {
        let _ = writeln!(out, "  {}: {}", field.key, number((field.get)(pose)));
    }
    let _ = writeln!(out, "  {PACE_FIELD}: {}\n}}", pose.duration_ns);
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
        let _ = writeln!(out, "  has_anchor: {}", clip.has_anchor);
        let _ = writeln!(out, "  anchor_head_dx: {}", number(clip.anchor_head_dx));
        let _ = writeln!(out, "  anchor_head_dy: {}", number(clip.anchor_head_dy));
        let _ = writeln!(out, "  anchor_head_dz: {}", number(clip.anchor_head_dz));
        let _ = writeln!(out, "  anchor_head_qw: {}", number(clip.anchor_head_qw));
        let _ = writeln!(out, "  anchor_head_qx: {}", number(clip.anchor_head_qx));
        let _ = writeln!(out, "  anchor_head_qy: {}", number(clip.anchor_head_qy));
        let _ = writeln!(out, "  anchor_head_qz: {}", number(clip.anchor_head_qz));
        let _ = writeln!(out, "  anchor_body_yaw: {}", number(clip.anchor_body_yaw));
        let _ = writeln!(
            out,
            "  anchor_antenna_right: {}",
            number(clip.anchor_antenna_right)
        );
        let _ = writeln!(
            out,
            "  anchor_antenna_left: {}",
            number(clip.anchor_antenna_left)
        );
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

/// Seconds as whole milliseconds, rounded up.
///
/// Up, because a window is how long a motion occupies a timeline: rounding down
/// would state a window the motion is still playing at the end of.
#[expect(
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    reason = "a validated motion's duration is finite, positive, and minutes at most"
)]
fn ms_ceil(seconds: f64) -> u64 {
    (seconds * 1000.0).ceil() as u64
}

/// A pose's pace as the sidecar states it: the message's nanoseconds as whole
/// milliseconds, rounded up.
///
/// Up for the reason a motion's window rounds up: the number is a duration a
/// compile lays a step out against, and rounding a sub-millisecond remainder
/// away would state a pace the move is still running at the end of. The message
/// has screened, so the value is positive and minutes at most.
///
/// # Panics
///
/// If the message's pace is negative, which the screen refuses.
fn pace_ms(duration_ns: i64) -> u32 {
    let ns = u64::try_from(duration_ns).expect("a screened pace is positive");
    u32::try_from(ns.div_ceil(1_000_000)).expect("a screened pace is minutes at most")
}

/// Write `text` to `path`, saying which file it was on the way out.
fn write(path: &Path, text: &str) -> anyhow::Result<()> {
    std::fs::write(path, text).with_context(|| format!("cannot write {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::collections::BTreeMap;
    use std::sync::OnceLock;

    use brenn_reachy__cogs__config_clk_rs::ClipFrameWire;
    use reachy_clips::config::{MAX_MOTIONS, MAX_SEGMENTS};
    use reachy_clips::format::{Channel, ClipDoc, document_kind};
    use reachy_scratch::scratch_dir;

    use super::probe_clips::Pose;

    /// The environment variable naming the committed clip documents' directory,
    /// relative to the runfiles root, which is a test's working directory.
    ///
    /// Read rather than embedded: the library is a tree of documents that grows
    /// by import, and a list of file names in this file would make adding one a
    /// code change.
    const DOCUMENTS_ENV: &str = "CLIP_DOCUMENTS";

    /// The environment variable naming the committed pose documents'
    /// directory, for the reason [`DOCUMENTS_ENV`] is read rather than
    /// embedded.
    const POSES_ENV: &str = "POSE_DOCUMENTS";

    /// The clip asset those documents emit, as committed.
    const ASSET: &str = include_str!("clip_library.textproto");

    /// The pose asset the pose documents emit, as committed.
    const POSE_ASSET: &str = include_str!("pose_library.textproto");

    /// The name sidecar both libraries share, as committed.
    const NAMES: &str = include_str!("library.names.json");

    /// The directory a named environment variable points at.
    ///
    /// Panics rather than answers: a missing runfile is a broken test target,
    /// not a case.
    fn root_of(variable: &str) -> PathBuf {
        let named = std::env::var(variable).unwrap_or_else(|_| {
            panic!(
                "{variable} is unset: the test target has to name the directory beside the data \
                 attribute that supplies it"
            )
        });
        PathBuf::from(named)
    }

    /// The directory the committed clip documents are in.
    fn documents_root() -> PathBuf {
        root_of(DOCUMENTS_ENV)
    }

    /// Every committed clip document, by path ascending — the same walk the
    /// tool's own run does, so what these cases emit is what `make
    /// library-config` emits.
    ///
    /// Panics below two documents: a runfiles arrangement that supplies none
    /// would otherwise emit an empty library, and comparing that against the
    /// committed asset fails in a way that reads as staleness and invites a
    /// regeneration that would delete the library.
    fn texts_ref() -> &'static [(String, String)] {
        static READ: OnceLock<Vec<(String, String)>> = OnceLock::new();
        READ.get_or_init(|| texts_under(&documents_root()))
    }

    /// A cloned corpus for cases that change document order or content.
    fn texts() -> Vec<(String, String)> {
        texts_ref().to_vec()
    }

    /// Every committed pose document, by path ascending, through the tool's own
    /// walk.
    fn pose_texts() -> Vec<(String, String)> {
        static READ: OnceLock<Vec<(String, String)>> = OnceLock::new();
        READ.get_or_init(|| {
            let root = root_of(POSES_ENV);
            let read = read_documents(&root, reachy_poses::format::DOCUMENT_EXT, Descend::No)
                .unwrap_or_else(|error| panic!("cannot read {}: {error:#}", root.display()));
            assert!(
                read.len() >= 2,
                "{} holds {} pose document(s): the test target's data attribute is not supplying \
                 them",
                root.display(),
                read.len()
            );
            read
        })
        .clone()
    }

    /// The committed pose documents, loaded, for every case that emits.
    fn poses() -> &'static [LoadedPose] {
        static LOADED: OnceLock<Vec<LoadedPose>> = OnceLock::new();
        LOADED.get_or_init(|| load_poses(&pose_texts()).expect("the committed pose documents load"))
    }

    /// The folds the committed poses author, which the probe table steps
    /// between.
    fn folds() -> Folds {
        Folds::of(poses()).expect("the committed library holds both reserved poses")
    }

    /// The emit of the committed tree, done once for every case that only reads
    /// it.
    ///
    /// The tree is tens of documents of up to a thousand frames and every frame
    /// costs a kinematic solve, so emitting it per case is most of this small
    /// target's runtime. The cases that patch a document emit their own; nothing
    /// mutates what this hands back.
    fn baseline() -> &'static Emitted {
        static EMITTED: OnceLock<Emitted> = OnceLock::new();
        EMITTED.get_or_init(|| emit(texts_ref(), poses()).expect("the checked-in documents emit"))
    }

    /// [`texts`] over a named directory, so the guard below has something to
    /// point at that is not the runfiles.
    fn texts_under(root: &Path) -> Vec<(String, String)> {
        let read = documents(root, DOCUMENT_EXT, Descend::Yes)
            .unwrap_or_else(|error| panic!("cannot read {}: {error}", root.display()));
        assert!(
            read.len() >= 2,
            "{} holds {} document(s): the test target's data attribute is not supplying the tree",
            root.display(),
            read.len()
        );
        read.into_iter()
            .map(|(source, text)| {
                let text = text.unwrap_or_else(|error| panic!("cannot read {source}: {error}"));
                (source, text)
            })
            .collect()
    }

    /// The committed clip documents, keyed by the names scripts invoke.
    fn documents_by_name() -> &'static BTreeMap<String, ClipDoc> {
        static DOCUMENTS: OnceLock<BTreeMap<String, ClipDoc>> = OnceLock::new();
        DOCUMENTS.get_or_init(|| {
            let mut documents = BTreeMap::new();
            for (source, text) in texts_ref() {
                if document_kind(text)
                    .unwrap_or_else(|error| panic!("{source} does not identify its kind: {error}"))
                    != "clip"
                {
                    continue;
                }
                let document: ClipDoc = serde_json::from_str(text).expect("the document parses");
                let name = document.name.clone();
                if documents.insert(name.clone(), document).is_some() {
                    panic!("duplicate clip document name {name:?} in {source}");
                }
            }
            documents
        })
    }

    /// The committed clip document named by its library name.
    fn document(name: &str) -> ClipDoc {
        documents_by_name()
            .get(name)
            .cloned()
            .unwrap_or_else(|| panic!("{name} is not a committed document"))
    }

    /// Count nonzero sign runs around `centre`, retaining the prior sign over
    /// samples inside the deadband.
    fn sign_runs(values: impl IntoIterator<Item = f64>, centre: f64) -> usize {
        let mut previous = 0;
        let mut runs = 0;
        for value in values {
            let delta = value - centre;
            let sign = if delta > 1e-12 {
                1
            } else if delta < -1e-12 {
                -1
            } else {
                0
            };
            if sign != 0 && sign != previous {
                runs += 1;
                previous = sign;
            }
        }
        runs
    }

    /// One document, as text, with `patch` applied to its parsed form.
    fn doc(text: &str, patch: impl FnOnce(&mut serde_json::Value)) -> String {
        let mut value: serde_json::Value = serde_json::from_str(text).expect("the document parses");
        patch(&mut value);
        serde_json::to_string(&value).expect("a patched document is JSON")
    }

    /// A runfiles arrangement that supplies no documents is a broken test
    /// target and has to read as one. Without this the drift case emits an
    /// empty library, the comparison fails as staleness, and the invited
    /// `make library-config` writes that empty library over the real asset.
    #[test]
    #[should_panic(expected = "document(s)")]
    fn a_walk_that_finds_no_tree_is_a_broken_target_rather_than_an_empty_library() {
        let empty = scratch_dir("gen-library-config-empty-tree");
        let _ = texts_under(empty.as_ref());
    }

    /// The documents the cases emit are the committed tree itself, walked the
    /// way the tool walks it: the drift check is against what `make
    /// library-config` would read, not against a list of names in this file.
    #[test]
    fn the_walk_finds_the_committed_documents_in_the_tree() {
        let sources: Vec<&str> = texts_ref()
            .iter()
            .map(|(source, _)| source.as_str())
            .collect();
        assert!(
            sources.iter().any(|source| source.ends_with("nod.json")),
            "{sources:?}"
        );
        assert!(sources.windows(2).all(|pair| pair[0] < pair[1]), "sorted");
    }

    #[test]
    fn the_authored_wave_overlays_are_zero_centred_mirrors() {
        let left_doc = document("wave_left");
        let right_doc = document("wave_right");
        for (name, doc) in [("wave_left", &left_doc), ("wave_right", &right_doc)] {
            assert_eq!(doc.name, name);
            assert!(doc.base.is_none());
            assert_eq!(doc.channels, vec![Channel::Antennas]);
            assert_eq!(doc.frame_hz, 50.0);
            assert_eq!(doc.blend_in_ms, Some(200));
            assert_eq!(doc.blend_out_ms, Some(200));
            assert_eq!(doc.frames.len(), 136);
        }

        let limits = ClipLimits::default();
        let left = Clip::from_doc(left_doc, &limits).expect("wave_left loads");
        let right = Clip::from_doc(right_doc, &limits).expect("wave_right loads");
        assert!(left.anchor().is_none() && right.anchor().is_none());
        assert!(left.mask().contains(Channel::Antennas));
        assert!(right.mask().contains(Channel::Antennas));
        for (index, (left_frame, right_frame)) in
            left.frames().iter().zip(right.frames()).enumerate()
        {
            let left_antennas = left_frame.antennas.expect("left antennas are present");
            let right_antennas = right_frame.antennas.expect("right antennas are present");
            assert_eq!(left_antennas[0], 0.0, "left right delta at {index}");
            assert_eq!(right_antennas[1], 0.0, "right left delta at {index}");
            assert!(
                (right_antennas[0] + left_antennas[1]).abs() <= 1e-12,
                "mirror at {index}"
            );
        }
        for index in [0, 45, 90, 135] {
            assert_eq!(
                left.frames()[index].antennas.expect("left antennas")[1],
                0.0
            );
            assert_eq!(
                right.frames()[index].antennas.expect("right antennas")[0],
                0.0
            );
        }
        for start in [0, 45, 90] {
            let left_sum: f64 = left.frames()[start..start + 45]
                .iter()
                .map(|frame| frame.antennas.expect("left antennas")[1])
                .sum();
            let right_sum: f64 = right.frames()[start..start + 45]
                .iter()
                .map(|frame| frame.antennas.expect("right antennas")[0])
                .sum();
            assert!(left_sum.abs() <= 1e-12, "left cycle {start}: {left_sum}");
            assert!(right_sum.abs() <= 1e-12, "right cycle {start}: {right_sum}");
        }
        let left_track: Vec<f64> = left
            .frames()
            .iter()
            .map(|frame| frame.antennas.expect("left antennas")[1])
            .collect();
        let right_track: Vec<f64> = right
            .frames()
            .iter()
            .map(|frame| frame.antennas.expect("right antennas")[0])
            .collect();
        for (name, track) in [("wave_left", &left_track), ("wave_right", &right_track)] {
            assert_eq!(sign_runs(track.iter().copied(), 0.0), 6, "{name} cycles");
            let minimum = track.iter().copied().fold(f64::INFINITY, f64::min);
            let maximum = track.iter().copied().fold(f64::NEG_INFINITY, f64::max);
            assert!(
                (minimum + 0.7333333333333333).abs() <= 1e-12,
                "{name} minimum"
            );
            assert!(
                (maximum - 0.7333333333333333).abs() <= 1e-12,
                "{name} maximum"
            );
        }
    }

    #[test]
    fn the_recorded_hello_wave_is_a_resolved_greeting() {
        let doc = document("hello_wave");
        assert_eq!(doc.version, 1);
        assert_eq!(doc.name, "hello_wave");
        assert_eq!(doc.base.as_deref(), Some("neutral"));
        assert_eq!(doc.channels, vec![Channel::Head, Channel::Antennas]);
        assert_eq!(doc.frame_hz, 50.0);
        assert_eq!(doc.blend_in_ms, Some(200));
        assert_eq!(doc.blend_out_ms, Some(200));
        assert_eq!(doc.frames.len(), 136);

        let neutral = *poses()
            .iter()
            .find(|pose| pose.name() == "neutral")
            .expect("neutral is committed")
            .targets();
        let hello = *poses()
            .iter()
            .find(|pose| pose.name() == "hello")
            .expect("hello is committed")
            .targets();
        let clip = Clip::from_doc_resolved(doc, &ClipLimits::default(), |name| {
            poses()
                .iter()
                .find(|pose| pose.name() == name)
                .map(|pose| *pose.targets())
        })
        .expect("hello_wave resolves and passes the envelope screen");
        assert_eq!(
            clip.anchor().expect("posed clip has an anchor").name(),
            "neutral"
        );
        assert_eq!(
            clip.anchor().expect("posed clip has an anchor").targets(),
            neutral
        );

        let expected_dt = [
            0.0006690716310147473,
            0.011657214299538974,
            0.011019355907297057,
        ];
        let expected_dq = [
            0.9727091900478037,
            0.22862791097815874,
            0.009840924041276436,
            0.03833100745248597,
        ];
        let expected_right = -0.6558838173726208;
        for (index, frame) in clip.frames().iter().enumerate() {
            let head = frame.head.expect("head is present");
            for (got, want) in head.translation.vector.iter().zip(expected_dt) {
                assert_eq!(*got, want, "head translation at {index}");
            }
            let q = head.rotation.quaternion();
            for (got, want) in [q.w, q.i, q.j, q.k].into_iter().zip(expected_dq) {
                assert!((got - want).abs() <= 1e-12, "head rotation at {index}");
            }
            assert_eq!(
                frame.antennas.expect("antennas are present")[0],
                expected_right,
                "right antenna at {index}"
            );
        }

        let first = clip.frames()[0].head.expect("head is present");
        let composed = neutral.head_pose_body * first;
        assert!(
            (composed.translation.vector - hello.head_pose_body.translation.vector).norm() < 1e-12
        );
        assert!(composed.rotation.angle_to(&hello.head_pose_body.rotation) < 1e-12);
        assert_eq!(neutral.antennas[0] + expected_right, -0.8303838173726208);
        assert!((neutral.antennas[0] + expected_right - hello.antennas[0]).abs() < 1e-12);

        for index in [0, 45, 90, 135] {
            let centre = clip.frames()[index].antennas.expect("antennas are present")[1];
            assert!((centre - 0.2872683519680999).abs() <= 1e-12);
        }
        for start in [0, 45, 90] {
            let mean: f64 = clip.frames()[start..start + 45]
                .iter()
                .map(|frame| frame.antennas.expect("antennas are present")[1])
                .sum::<f64>()
                / 45.0;
            assert!(
                (mean - 0.2872683519680999).abs() <= 1e-12,
                "cycle {start}: {mean}"
            );
        }
        let left: Vec<f64> = clip
            .frames()
            .iter()
            .map(|frame| frame.antennas.expect("antennas are present")[1])
            .collect();
        assert_eq!(
            sign_runs(left.iter().copied(), 0.2872683519680999),
            6,
            "hello_wave left antenna cycles"
        );
        assert!(left.iter().any(|value| *value < 0.2872683519680999));
        assert!(left.iter().any(|value| *value > 0.2872683519680999));
    }

    #[test]
    fn the_recorded_dance_is_a_resolved_clip() {
        let doc = document("dance");
        assert_eq!(doc.version, 1);
        assert_eq!(doc.kind, "clip");
        assert_eq!(doc.name, "dance");
        assert_eq!(doc.base.as_deref(), Some("neutral"));
        assert!(
            doc.description
                .as_deref()
                .is_some_and(|description| description.contains("S018"))
        );
        assert_eq!(
            doc.channels,
            vec![Channel::Head, Channel::BodyYaw, Channel::Antennas]
        );
        assert_eq!(doc.frame_hz, 50.0);
        assert_eq!(doc.blend_in_ms, Some(400));
        assert_eq!(doc.blend_out_ms, Some(800));
        assert_eq!(doc.frames.len(), 650);
        assert_eq!(
            doc.frames[0].dt,
            Some([
                0.006480514813959866,
                -0.0009933644307734695,
                0.01102205186999745
            ])
        );
        assert_eq!(
            doc.frames[0].dq,
            Some([
                0.9992271405262544,
                0.008569476403373952,
                -0.038307510142397895,
                0.002054355516246501
            ])
        );
        assert_eq!(
            doc.frames[649].dt,
            Some([
                -0.01751625164223465,
                0.00014378718547920316,
                -0.03414675607663002
            ])
        );
        assert_eq!(
            doc.frames[649].dq,
            Some([
                0.9818949619914762,
                0.013759037503941416,
                0.18891995529276592,
                -0.0014909711271409649
            ])
        );
        assert!(doc.frames.iter().all(|frame| frame.dt.is_some()
            && frame.dq.is_some()
            && frame.body_yaw.is_some()
            && frame.antennas.is_some()));

        let neutral = *poses()
            .iter()
            .find(|pose| pose.name() == "neutral")
            .expect("neutral is committed")
            .targets();
        let clip = Clip::from_doc_resolved(doc, &ClipLimits::default(), |name| {
            poses()
                .iter()
                .find(|pose| pose.name() == name)
                .map(|pose| *pose.targets())
        })
        .expect("dance resolves and passes the envelope screen");
        assert_eq!(
            clip.anchor().expect("posed clip has an anchor").name(),
            "neutral"
        );
        assert_eq!(
            clip.anchor().expect("posed clip has an anchor").targets(),
            neutral
        );
        let yaws: Vec<f64> = clip
            .frames()
            .iter()
            .map(|frame| frame.body_yaw.expect("body yaw"))
            .collect();
        assert!(
            (yaws.iter().copied().fold(f64::INFINITY, f64::min) + 0.23999929703873568).abs()
                <= 1e-12
        );
        assert!(
            (yaws.iter().copied().fold(f64::NEG_INFINITY, f64::max) - 0.23999929703873568).abs()
                <= 1e-12
        );
        assert_eq!(sign_runs(yaws.iter().copied(), 0.0), 16, "body-yaw sways");
        let antennas: Vec<f64> = clip
            .frames()
            .iter()
            .flat_map(|frame| frame.antennas.expect("antennas"))
            .collect();
        assert!(
            (antennas.iter().copied().fold(f64::INFINITY, f64::min) + 0.2599992384586303).abs()
                <= 1e-12
        );
        assert!(
            (antennas.iter().copied().fold(f64::NEG_INFINITY, f64::max) - 0.2599992384586303).abs()
                <= 1e-12
        );
        let right_antenna: Vec<f64> = clip
            .frames()
            .iter()
            .map(|frame| frame.antennas.expect("antennas")[0])
            .collect();
        assert_eq!(
            sign_runs(right_antenna.iter().copied(), 0.0),
            16,
            "right antenna beats"
        );
        assert!(clip.frames().iter().all(|frame| {
            let antennas = frame.antennas.expect("antennas");
            (antennas[0] + antennas[1]).abs() <= 1e-12
        }));
    }

    #[test]
    fn emit_bakes_a_supplied_pose_and_refuses_an_absent_one() {
        let pose = &poses()[0];
        let document = serde_json::to_string(&json!({
            "version": 1,
            "kind": "clip",
            "name": "synthetic/posed",
            "base": pose.name(),
            "channels": ["antennas"],
            "frame_hz": 50.0,
            "frames": [{"antennas": [0.0, 0.0]}]
        }))
        .unwrap();
        let emitted = emit(&[("synthetic.json".to_owned(), document)], poses())
            .expect("supplied pose resolves");
        assert!(emitted.textproto.contains("has_anchor: true"));
        let expected = pose.targets();
        for (key, value) in [
            (
                "anchor_head_dx",
                expected.head_pose_body.translation.vector.x,
            ),
            (
                "anchor_head_dy",
                expected.head_pose_body.translation.vector.y,
            ),
            (
                "anchor_head_dz",
                expected.head_pose_body.translation.vector.z,
            ),
            (
                "anchor_head_qw",
                expected.head_pose_body.rotation.quaternion().w,
            ),
            (
                "anchor_head_qx",
                expected.head_pose_body.rotation.quaternion().i,
            ),
            (
                "anchor_head_qy",
                expected.head_pose_body.rotation.quaternion().j,
            ),
            (
                "anchor_head_qz",
                expected.head_pose_body.rotation.quaternion().k,
            ),
            ("anchor_body_yaw", expected.body_yaw),
            ("anchor_antenna_right", expected.antennas[0]),
            ("anchor_antenna_left", expected.antennas[1]),
        ] {
            assert!(
                emitted.textproto.contains(&format!("{key}: {:?}", value)),
                "emitted anchor lacks {key}={value:?}"
            );
        }

        let missing = serde_json::to_string(&json!({
            "version": 1,
            "kind": "clip",
            "name": "synthetic/missing",
            "base": "not-a-pose",
            "channels": ["antennas"],
            "frame_hz": 50.0,
            "frames": [{"antennas": [0.0, 0.0]}]
        }))
        .unwrap();
        let error = emit(&[("missing.json".to_owned(), missing)], poses()).unwrap_err();
        assert!(error.to_string().contains("not-a-pose"));
    }

    /// The whole point of the tool having a committed output: the emit is
    /// reproducible, so a change to a document, to the mapping, or to the
    /// printer that nobody regenerated for is a red test rather than an asset
    /// that no longer says what the documents do.
    #[test]
    fn the_committed_asset_is_what_the_committed_documents_emit() {
        let emitted = baseline();
        assert_eq!(
            emitted.textproto, ASSET,
            "cogs/clip_library.textproto is stale; run `make library-config`"
        );
        assert_eq!(
            emitted.poses_textproto, POSE_ASSET,
            "cogs/pose_library.textproto is stale; run `make library-config`"
        );
        assert_eq!(
            emitted.names_json(),
            NAMES,
            "cogs/library.names.json is stale; run `make library-config`"
        );
    }

    /// The load's own opinion of the assets, which the emitter reports and does
    /// not silence. A note means the committed documents disagree with the
    /// derivation about something, and the operator gets to hear it.
    ///
    /// `ClipNote` is uninhabited today — nothing the load does can construct
    /// one — so this gate cannot currently fail. It is kept as the gate for the
    /// first note that returns, and the case below keeps the reporting path it
    /// would come out through covered meanwhile.
    #[test]
    fn the_committed_documents_load_without_a_note() {
        let emitted = baseline();
        assert!(emitted.notes.is_empty(), "notes: {:?}", emitted.notes);
    }

    /// The path a note reaches the operator by, driven from a synthetic emit
    /// because no load can produce a note today. The gate above is only worth
    /// anything if a note that appears is also said.
    #[test]
    fn a_note_is_said_before_the_numberings() {
        let emitted = Emitted {
            textproto: String::new(),
            poses_textproto: String::new(),
            clips: Numbering::of("clip", "frame", &[], []),
            motions: Numbering::of("motion", "segment", &[], []),
            poses: Numbering::of_poses([], 0),
            notes: vec!["bench/nod: the derivation changed something".to_owned()],
        };
        let mut said = Vec::new();
        emitted.report(&mut |line| said.push(line));
        assert_eq!(
            said,
            vec!["note: bench/nod: the derivation changed something".to_owned()]
        );
    }

    /// Every motion row states the window the compile of a `play` step needs,
    /// and the numbers are the library's own derivation from the frames.
    ///
    /// The clips carry none: no script names a clip, and a window against an id
    /// nothing can invoke would be a number nobody reads.
    #[test]
    fn a_motion_row_states_the_window_a_play_step_occupies() {
        let emitted = baseline();
        let sidecar: serde_json::Value =
            serde_json::from_str(&emitted.names_json()).expect("the sidecar is JSON");
        let tour = sidecar["motions"]
            .as_array()
            .expect("a motions table")
            .iter()
            .find(|row| row["name"] == json!("bench/tour"))
            .expect("the composed motion is numbered")
            .clone();
        assert_eq!(tour["duration_ms"], json!(1701));
        assert_eq!(tour["blend_out_ms"], json!(200));
        assert!(
            sidecar["clips"].as_array().expect("a clips table")[0]
                .get("duration_ms")
                .is_none(),
            "a clip id is not a thing a script can play",
        );
    }

    /// The committed probe documents are what the table authors, byte for byte.
    ///
    /// The same gate the emitted asset has, one level up: a probe's poses are
    /// the library's own folds, and a document holding hundreds of copies of a
    /// delta the fold has moved off would load, emit and hold exactly as
    /// happily while reading as the instrument it no longer is. Answered by
    /// `make library-config`, which rewrites them.
    #[test]
    fn the_committed_probe_documents_are_what_the_table_authors() {
        let texts = texts_ref();
        for probe in PROBES {
            let suffix = format!("{}.json", probe.name);
            let (source, text) = texts
                .iter()
                .find(|(source, _)| source.ends_with(&suffix))
                .unwrap_or_else(|| panic!("{} is not a committed document", probe.name));
            assert_eq!(
                *text,
                probe
                    .document(&folds())
                    .expect("the table authors a document"),
                "{source} is stale; run `make library-config`"
            );
        }
        // And nothing under `probe/` is a hand-authored document the table has
        // never heard of: such a file would emit into the library, be playable
        // by name, and answer to nothing.
        for (source, _) in texts {
            // The last `clips/` in the path and not the first: a checkout, or a
            // documents directory, that itself sits under a `clips/` would
            // otherwise leave `rest` starting somewhere above the tree and skip
            // every probe document without a word.
            if let Some(rest) = source.rsplit_once("clips/").map(|(_, rest)| rest)
                && rest.starts_with("probe/")
            {
                let name = rest.trim_end_matches(".json");
                assert!(
                    PROBES.iter().any(|probe| probe.name == name),
                    "{source} is a probe document with no row in the table"
                );
            }
        }
    }

    /// A probe is an instrument, and what makes it one is the shape of its
    /// frame track. The table states that shape; this is the assertion that the
    /// shape survives into the library the machine plays.
    ///
    /// Every named pose is counted rather than eyeballed: the step probes hold
    /// theirs 325 frames each — 6.5 s, the watch's shortest judgeable hold and
    /// half a second — and the sweep passes through its own on the frames its
    /// ramps hand over on. The figures are differences of the library's own
    /// folds and the antennas' outboard constant, so a moved fold or sideways
    /// point fails here.
    ///
    /// The entry blend is pinned at zero for a related reason: it is a *weight*
    /// ramp over the whole delta a frame carries, so a probe taking the format's
    /// default would compose its first pose a tenth at a time and its first
    /// arrival would be a ten-period ramp rather than the step or the stated
    /// ramp the table says.
    #[test]
    fn every_probe_the_library_carries_is_the_track_the_table_states() {
        let emitted = baseline();
        let sidecar: serde_json::Value =
            serde_json::from_str(&emitted.names_json()).expect("the sidecar is JSON");
        let motions = sidecar["motions"].as_array().expect("a motions table");
        for probe in PROBES {
            let name = probe.name;
            let frames = probe.antenna_frames(&folds()).expect("the table chains");
            let clip = emitted
                .clips
                .entries
                .iter()
                .find(|clip| clip.name == name)
                .unwrap_or_else(|| panic!("{name} is not in the library"));
            assert_eq!(clip.parts, Some(frames.len()), "{name}");
            let motion = motions
                .iter()
                .find(|row| row["name"] == json!(name))
                .unwrap_or_else(|| panic!("{name} is not playable"));
            // 20 ms a frame, which is the tick rate the format pins.
            assert_eq!(motion["duration_ms"], json!(frames.len() * 20), "{name}");
            assert_eq!(motion["blend_out_ms"], json!(200), "{name}");
            let block = emitted
                .textproto
                .split("\n# ")
                .find(|block| block.starts_with(&format!("{name}\nclips {{")))
                .unwrap_or_else(|| panic!("{name}'s clip block is not in the text"));
            assert!(
                block.contains("\n  blend_in_ms: 0\n"),
                "{name}: {block:.200}"
            );
            for pose in [Pose::Up, Pose::Sides, Pose::Down, Pose::HalfDown] {
                let angles = pose.antennas(&folds());
                let printed = format!(
                    "antenna_right_d: {} antenna_left_d: {}",
                    number(angles[0]),
                    number(angles[1])
                );
                assert_eq!(
                    block.matches(&printed).count(),
                    frames.iter().filter(|frame| **frame == angles).count(),
                    "{name} does not carry {pose:?} the number of times its table states"
                );
            }
        }
    }

    /// The sweep is the residual instrument: 15.5 s of streamed antenna content
    /// with one judgeable hold at the end, so a run of it has a hold to judge
    /// while everything before it is moving.
    #[test]
    fn the_sweep_is_a_streamed_run_with_one_judgeable_hold() {
        let sweep = PROBES
            .iter()
            .find(|probe| probe.name == "probe/antenna-sweep")
            .expect("the sweep is in the table");
        let frames = sweep.antenna_frames(&folds()).expect("the table chains");
        assert_eq!(frames.len(), 775);
        let base = Pose::Up.antennas(&folds());
        let tail = frames
            .iter()
            .rev()
            .take_while(|frame| **frame == base)
            .count();
        assert_eq!(tail, 325, "6.5 s of stillness, on the base");
        let (mut longest, mut run) = (0, 0);
        for window in frames[..frames.len() - 325].windows(2) {
            run = if window[0] == window[1] { run + 1 } else { 0 };
            longest = longest.max(run + 1);
        }
        assert!(
            longest < 325,
            "the sweep's longest other stretch of stillness is {longest} frames, which the watch \
             would judge as a second hold"
        );
    }

    /// A clip id is an index into the emitted order, which is the order the
    /// sources arrive in — not the alphabetical order of the names, which the
    /// library itself is keyed by.
    #[test]
    fn a_clip_id_is_the_position_the_document_was_read_in() {
        let mut reversed = texts();
        reversed.reverse();
        let forward = baseline();
        let backward = emit(&reversed, poses()).expect("the documents emit either way");
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
        assert_eq!(names(forward), flipped);
        assert_ne!(
            forward.textproto, backward.textproto,
            "the order the documents arrive in is the numbering"
        );
    }

    /// The sidecar and the asset's own comment table are one table: a script
    /// author reading either gets the same id.
    #[test]
    fn the_sidecar_and_the_asset_agree_about_every_id() {
        let emitted = baseline();
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
        let (_, text) = broken
            .iter_mut()
            .find(|(source, _)| source.ends_with("nod.json"))
            .expect("nod is a committed document");
        *text = doc(text, |value| {
            value["frame_hz"] = json!(30.0);
        });
        let error = emit(&broken, poses()).expect_err("a clip on another grid is refused");
        let text = format!("{error:#}");
        assert!(text.contains("numbering is refused"), "{text}");
        assert!(text.contains("nod"), "{text}");
    }

    /// Two documents claiming one name is the authoring mistake that attacks the
    /// numbering directly: the loader keeps one and skips the other, ids come
    /// from document order, and a library where one document silently vanished
    /// is one every later script id is wrong against. Refused, like any skip.
    #[test]
    fn two_documents_claiming_one_name_refuse_the_whole_emit() {
        let mut clashing = texts();
        let (_, text) = clashing
            .iter_mut()
            .find(|(source, _)| source.ends_with("perk.json"))
            .expect("perk is a committed document");
        *text = doc(text, |value| {
            value["name"] = json!("bench/nod");
        });
        let error = emit(&clashing, poses()).expect_err("a duplicate name is refused");
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
        let error = emit(&[], poses()).expect_err("there is no library in nothing");
        assert!(
            format!("{error:#}").contains("none of the documents is a clip"),
            "{error:#}"
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
        let emitted = emit(&with_sequence, poses()).expect("a sequence emits");

        // The committed documents, and then this one: the numbering is the read
        // order, so the sequence written last is the last motion.
        let names: Vec<&str> = emitted
            .motions
            .entries
            .iter()
            .map(|motion| motion.name.as_str())
            .collect();
        let motion_id = names.len() - 1;
        assert_eq!(names.last(), Some(&"bench/greeting"), "{names:?}");

        // Two clips strung together, with the leading gap held apart from them.
        assert_eq!(emitted.motions.entries[motion_id].parts, Some(2));
        assert!(
            emitted
                .textproto
                .contains(&format!("#   {motion_id}  bench/greeting")),
            "{}",
            emitted.textproto
        );

        let printed = emitted
            .textproto
            .split("\n# bench/greeting\nmotions {\n")
            .nth(1)
            .expect("the greeting motion is printed");
        assert!(printed.starts_with("  lead_gap_ms: 250\n"), "{printed}");

        // Clip IDs are positional; adding a document before these shifts them.
        let clip_id = |name: &str| {
            emitted
                .clips
                .entries
                .iter()
                .position(|clip| clip.name == name)
                .unwrap_or_else(|| panic!("{name} is a clip"))
        };
        assert!(
            printed.contains(&format!(
                "    clip_id: {}\n    speed: 1.0\n    gap_after_ms: 300\n",
                clip_id("bench/nod")
            )),
            "{printed}"
        );
        assert!(
            printed.contains(&format!(
                "    clip_id: {}\n    speed: 2.0\n    gap_after_ms: 0\n",
                clip_id("bench/sway")
            )),
            "{printed}"
        );
    }

    /// A one-segment motion stands for every clip, so a schedule names motions
    /// only and invoking a bare clip costs nothing.
    ///
    /// By name rather than by position, because the motions are every asset that
    /// plays and the clips are those of them a clip document authored: the
    /// committed set has a sequence in it, so the two numberings agree on the
    /// clips and the motions carry more.
    #[test]
    fn every_clip_is_also_a_motion_of_one_segment() {
        let emitted = baseline();
        let sidecar: serde_json::Value =
            serde_json::from_str(&emitted.names_json()).expect("the sidecar is JSON");
        assert!(emitted.motions.len() >= emitted.clips.len());
        for clip in &emitted.clips.entries {
            let motion = emitted
                .motions
                .entries
                .iter()
                .find(|motion| motion.name == clip.name)
                .unwrap_or_else(|| panic!("{} plays as a motion", clip.name));
            assert_eq!(motion.parts, Some(1));
            let window = match motion.extra {
                Some(Extra::Window(window)) => window,
                _ => panic!("{} has no motion window", clip.name),
            };
            let sidecar_motion = sidecar["motions"]
                .as_array()
                .expect("motions are an array")
                .iter()
                .find(|row| row["name"] == clip.name)
                .unwrap_or_else(|| panic!("{} is absent from the sidecar", clip.name));
            assert_eq!(sidecar_motion["duration_ms"], json!(window.duration_ms));
            assert_eq!(sidecar_motion["blend_out_ms"], json!(window.blend_out_ms));
            let document = document(&clip.name);
            assert_eq!(
                clip.parts,
                Some(document.frames.len()),
                "{} frame count",
                clip.name
            );
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
        let error = emit(&many, poses()).expect_err("thirty-three motions do not fit");
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
        let error = emit(&many, poses()).expect_err("thirty-three segments do not fit");
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
        let error = emit(&dangling, poses()).expect_err("a reference to nothing is refused");
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
        let error = emit(&many, poses()).expect_err("seventeen clips do not fit");
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
    fn the_arguments_are_all_five_or_a_refusal() {
        fn words(line: &str) -> impl Iterator<Item = String> + '_ {
            line.split_whitespace().map(ToOwned::to_owned)
        }
        let args = parse(words("--clips a --out b --poses c --poses-out d --names e"))
            .expect("all five are given");
        assert_eq!(args.clips, PathBuf::from("a"));
        assert_eq!(args.out, PathBuf::from("b"));
        assert_eq!(args.poses, PathBuf::from("c"));
        assert_eq!(args.poses_out, PathBuf::from("d"));
        assert_eq!(args.names, PathBuf::from("e"));

        // Neither library may be emitted without the other: one sidecar numbers
        // both, and a run that wrote it from half the documents would state a
        // table with nothing behind it.
        let half = parse(words("--clips a --out b --names c")).expect_err("--poses is required");
        assert!(format!("{half:#}").contains("--poses is required"));
        let missing = parse(words("--clips a --out b --poses c --poses-out d"))
            .expect_err("--names is required");
        assert!(format!("{missing:#}").contains("--names is required"));
        let dangling = parse(words("--clips")).expect_err("a flag wants a value");
        assert!(format!("{dangling:#}").contains("--clips wants a value"));
        let unknown = parse(words("--library a")).expect_err("an unknown flag is refused");
        assert!(format!("{unknown:#}").contains("unknown argument --library"));
    }

    /// The whole tool over a directory, which is the one thing the pure emit
    /// cannot cover: what it reads, what it writes, and what it says.
    #[test]
    fn the_tool_writes_all_three_files_and_reports_every_asset() {
        let dir = scratch_dir("gen-library-config-tool");
        let clips = dir.join("clips");
        let pose_dir = dir.join("poses");
        std::fs::create_dir_all(&clips).expect("a temporary directory");
        std::fs::create_dir_all(&pose_dir).expect("a temporary directory");
        for (source, text) in pose_texts() {
            let name = Path::new(&source).file_name().expect("a file name");
            std::fs::write(pose_dir.join(name), text).expect("the pose document is written");
        }
        let root = documents_root();
        for (source, text) in texts_ref() {
            // Copied at the same relative depth: a document's id is its full
            // path's position in the sort, so flattening the tree here would
            // emit a different numbering than the committed one.
            let target = clips.join(
                Path::new(&source)
                    .strip_prefix(&root)
                    .expect("a document under the documents root"),
            );
            std::fs::create_dir_all(target.parent().expect("a parent"))
                .expect("the document's directory");
            std::fs::write(target, text).expect("the document is written");
        }
        let args = Args {
            clips,
            out: dir.join("clip_library.textproto"),
            poses: pose_dir,
            poses_out: dir.join("pose_library.textproto"),
            names: dir.join("library.names.json"),
        };
        let mut said = Vec::new();
        run(&args, &mut |line| said.push(line)).expect("the tool runs");
        assert_eq!(
            std::fs::read_to_string(&args.out).expect("the asset was written"),
            ASSET
        );
        assert_eq!(
            std::fs::read_to_string(&args.poses_out).expect("the pose asset was written"),
            POSE_ASSET
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
            said.iter()
                .any(|line| line.starts_with("pose ") && line.contains("(the stow)")),
            "{said:?}"
        );
        // Counted off the same walk rather than written in: the tree is
        // whatever is committed, and adding a document is not a change to this
        // case.
        let baseline = baseline();
        assert!(
            said.last().expect("a closing line").contains(&format!(
                "{} clip(s), {} motion(s) and {} pose(s)",
                baseline.clips.entries.len(),
                baseline.motions.entries.len(),
                baseline.poses.entries.len()
            )),
            "{said:?}"
        );
    }

    /// An empty directory is the caller's own configuration being wrong, and
    /// says so rather than emitting a library with no clips.
    #[test]
    fn an_empty_directory_is_refused() {
        let dir = scratch_dir("gen-library-config-empty");
        let error =
            read_documents(dir.as_ref(), DOCUMENT_EXT, Descend::Yes).expect_err("nothing to read");
        assert!(format!("{error:#}").contains("no *.json"), "{error:#}");
        let error = read_documents(
            dir.as_ref(),
            reachy_poses::format::DOCUMENT_EXT,
            Descend::No,
        )
        .expect_err("nothing to read");
        assert!(format!("{error:#}").contains("no *.textproto"), "{error:#}");
    }

    /// The pose documents the cases emit are the committed ones, walked the way
    /// the tool walks them: the drift check is against what `make
    /// library-config` would read.
    #[test]
    fn the_walk_finds_the_committed_pose_documents() {
        let sources: Vec<String> = pose_texts().into_iter().map(|(source, _)| source).collect();
        for name in [
            "neutral.textproto",
            "peek.textproto",
            "peek_tilt.textproto",
            "hello.textproto",
            "stow.textproto",
        ] {
            assert!(
                sources.iter().any(|source| source.ends_with(name)),
                "{name}: {sources:?}"
            );
        }
        assert!(sources.windows(2).all(|pair| pair[0] < pair[1]), "sorted");
    }

    /// A pose id is the position of its document in the sort, and the `stow`
    /// field is the index of the document named `stow` — the one reserved name
    /// nothing that stows has to know a number for.
    #[test]
    fn a_pose_id_is_the_position_its_document_sorts_in() {
        let emitted = baseline();
        let names: Vec<&str> = emitted
            .poses
            .entries
            .iter()
            .map(|pose| pose.name.as_str())
            .collect();
        for name in ["neutral", "peek", "peek_tilt", "hello", "stow"] {
            assert!(names.contains(&name), "{name}: {names:?}");
        }
        assert_eq!(names.len(), 5);
        assert_eq!(
            usize::from(
                emitted
                    .poses
                    .reserved
                    .expect("the pose numbering names a stow")
            ),
            names
                .iter()
                .position(|name| *name == reachy_poses::STOW_POSE)
                .expect("the library holds a stow")
        );
        assert!(
            emitted.poses_textproto.contains(&format!(
                "stow: {}",
                emitted
                    .poses
                    .reserved
                    .expect("the pose numbering names a stow")
            )),
            "{}",
            emitted.poses_textproto
        );
    }

    /// The sidecar's pose table and the asset's own comment table are one
    /// table, and a row states the pace a move takes when the command states
    /// none — what the edge needs and cannot derive.
    #[test]
    fn the_sidecar_states_every_pose_under_its_id_and_its_pace() {
        let emitted = baseline();
        let sidecar: serde_json::Value =
            serde_json::from_str(&emitted.names_json()).expect("the sidecar is JSON");
        let listed = sidecar["poses"].as_array().expect("poses is an array");
        assert_eq!(listed.len(), emitted.poses.len());
        for (pose_id, pose) in emitted.poses.entries.iter().enumerate() {
            assert_eq!(listed[pose_id]["pose_id"], json!(pose_id));
            assert_eq!(listed[pose_id]["name"], json!(pose.name));
            let Some(Extra::Pace(duration_ms)) = pose.extra else {
                panic!("a pose row states a pace");
            };
            assert_eq!(listed[pose_id]["duration_ms"], json!(duration_ms));
            assert!(
                emitted
                    .poses_textproto
                    .contains(&format!("#   {pose_id}  {}", pose.name)),
                "the asset's table is missing {pose_id}"
            );
        }
        assert_eq!(
            listed
                .iter()
                .find(|row| row["name"] == json!(reachy_poses::STOW_POSE))
                .expect("the stow is numbered")["duration_ms"],
            json!(2000)
        );
    }

    /// A pose states every field, zeros included: the generated conversion
    /// gives every field explicit presence, so an omitted zero is a
    /// configuration that will not load rather than a default.
    ///
    /// Every channel carries a value of its own, so the text pins which field
    /// each key is read from and not only the order of the keys: a row whose
    /// key and accessor are mislabelled together round-trips through every
    /// reader in this repo and commands a different head or the antennas on the
    /// wrong side.
    #[test]
    fn a_pose_states_every_field_in_declared_order() {
        let mut message = PoseLibraryConfigWire::new_boxed();
        let library = message.clear_valid();
        let slot = library.poses.try_grow().expect("the message holds a pose");
        slot.dx = 0.1;
        slot.dy = 0.2;
        slot.dz = 0.3;
        slot.qx = 0.4;
        slot.qy = 0.5;
        slot.qz = 0.6;
        slot.body_yaw = 0.7;
        slot.antenna_right = 0.8;
        slot.antenna_left = 0.9;
        slot.qw = 1.1;
        slot.duration_ns = 800_000_000;
        let mut out = String::new();
        print_pose(&mut out, slot);
        assert_eq!(
            out,
            "poses {\n  dx: 0.1\n  dy: 0.2\n  dz: 0.3\n  qw: 1.1\n  qx: 0.4\n  qy: 0.5\n  \
             qz: 0.6\n  body_yaw: 0.7\n  antenna_right: 0.8\n  antenna_left: 0.9\n  \
             duration_ns: 800000000\n}\n"
        );
    }

    /// The report is what an operator reads after `make library-config`: which
    /// id each name is invoked by, what the kind states beside it, and which
    /// pose the machine rests at.
    ///
    /// Rendered here from a numbering of its own, so a marker against the wrong
    /// row, a dropped pace column or a lost part count is this case rather than
    /// a reading nobody takes.
    #[test]
    fn a_report_line_states_the_id_the_name_and_what_the_kind_carries() {
        let mut said = Vec::new();
        Numbering::of_poses(
            [
                ("neutral".to_owned(), 800),
                ("stow".to_owned(), 2000),
                ("peek".to_owned(), 650),
            ],
            1,
        )
        .report(&mut |line| said.push(line));
        assert_eq!(
            said,
            [
                "pose 0  neutral  800 ms",
                "pose 1  stow  2000 ms  (the stow)",
                "pose 2  peek  650 ms",
            ]
        );

        let mut said = Vec::new();
        Numbering::of(
            "clip",
            "frame",
            &["bench/nod".to_owned(), "bench/tour".to_owned()],
            [42, 85],
        )
        .report(&mut |line| said.push(line));
        assert_eq!(
            said,
            [
                "clip 0  bench/nod  42 frame(s)",
                "clip 1  bench/tour  85 frame(s)"
            ]
        );
    }

    /// A pace is whole milliseconds and a sub-millisecond remainder rounds up:
    /// rounding it away would state a pace the move is still running at the end
    /// of, and the edge lays a step out against the number.
    #[test]
    fn a_pace_rounds_a_sub_millisecond_remainder_up() {
        assert_eq!(pace_ms(1_000_000), 1);
        assert_eq!(pace_ms(1_000_001), 2);
        assert_eq!(pace_ms(1_999_999), 2);
        assert_eq!(pace_ms(2_000_000), 2);
    }

    /// The pace read out of the message is the screen's, and the screen refuses
    /// a negative one: the reader says so rather than casting it into a large
    /// positive pace.
    #[test]
    #[should_panic(expected = "a screened pace is positive")]
    fn a_negative_pace_is_not_one_the_screen_let_through() {
        let _ = pace_ms(-1);
    }

    /// What the emitted pose asset says is what the loaded documents say, read
    /// back through the reader every consumer of a bound library uses.
    #[test]
    fn the_emitted_pose_asset_reads_back_as_the_documents_it_came_from() {
        let emitted = baseline();
        let message = reachy_poses::config::parse_library(&emitted.poses_textproto)
            .expect("the emitted asset parses");
        let library = message.validate().expect("the asset is a message");
        let screened = reachy_poses::config::screen(library).expect("the asset screens");
        assert_eq!(screened.len(), poses().len());
        for (pose_id, pose) in poses().iter().enumerate() {
            let id = u16::try_from(pose_id).expect("a pose id fits");
            let (targets, pace) = screened.targets(id).expect("the pose is in the library");
            assert_eq!(
                targets.head_pose_body.translation,
                pose.targets().head_pose_body.translation
            );
            // The loader's UnitQuaternion normalization is the observed source
            // of the permitted round-trip difference in quaternion components.
            for (actual, expected) in targets
                .head_pose_body
                .rotation
                .coords
                .iter()
                .zip(pose.targets().head_pose_body.rotation.coords.iter())
            {
                assert!((actual - expected).abs() <= 1e-12);
            }
            assert_eq!(targets.body_yaw, pose.targets().body_yaw);
            assert_eq!(targets.antennas, pose.targets().antennas);
            assert_eq!(pace.as_millis(), u128::from(pose.duration_ms()));
        }
        assert_eq!(
            screened.stow().0,
            emitted
                .poses
                .reserved
                .expect("the pose numbering names a stow")
        );
    }

    /// A pose document whose name is not its file stem is refused: the stem is
    /// how an author finds the document a script names.
    #[test]
    fn a_pose_document_that_is_not_named_for_its_file_is_refused() {
        let mut renamed = pose_texts();
        let neutral = renamed
            .iter()
            .position(|(source, _)| source.ends_with("neutral.textproto"))
            .expect("the neutral document is present");
        renamed[neutral].0 = renamed[neutral].0.replace("neutral", "resting");
        let error = load_poses(&renamed).expect_err("the stem and the name disagree");
        assert!(format!("{error:#}").contains("file stem"), "{error:#}");
    }

    /// A library with no `stow` is refused: every schedule ends at it, the
    /// fault ladder commands it, and the disarm sequence judges folded against
    /// it.
    #[test]
    fn a_library_with_no_stow_refuses_the_whole_emit() {
        let mut without = pose_texts();
        without.retain(|(source, _)| !source.ends_with("stow.textproto"));
        let loaded = load_poses(&without).expect("the rest still load");
        let error = emit(texts_ref(), &loaded).expect_err("a library needs a stow");
        assert!(
            format!("{error:#}").contains("no pose named stow"),
            "{error:#}"
        );
    }

    /// Two documents under one name is the authoring mistake that attacks the
    /// numbering directly, as it is for a clip.
    #[test]
    fn two_pose_documents_claiming_one_name_refuse_the_whole_emit() {
        let mut clashing = pose_texts();
        let neutral = clashing[0].1.clone();
        clashing.push(("cogs/poses/zneutral.textproto".to_owned(), neutral));
        // Loaded directly: the file-stem rule would refuse this first, and what
        // is under test is the emit's own opinion of the set.
        let loaded: Vec<LoadedPose> = clashing
            .iter()
            .map(|(_, text)| LoadedPose::from_text(text).expect("the document loads"))
            .collect();
        let error = emit(texts_ref(), &loaded).expect_err("two poses under one name");
        assert!(
            format!("{error:#}").contains("two poses named"),
            "{error:#}"
        );
    }

    /// A pose outside the envelope never becomes an asset: the loader runs the
    /// check every command path runs, and the emit is refused whole.
    #[test]
    fn a_pose_document_outside_the_envelope_refuses_the_whole_emit() {
        let stow = pose_texts()
            .into_iter()
            .find(|(source, _)| source.ends_with("stow.textproto"))
            .expect("the stow is committed");
        let raised = stow
            .1
            .lines()
            .map(|line| {
                if line.starts_with("dt:") {
                    "dt: [0.0, 0.0, 0.5]\n".to_owned()
                } else {
                    format!("{line}\n")
                }
            })
            .collect::<String>();
        let error = load_poses(&[(stow.0, raised)]).expect_err("half a metre up is not reachable");
        assert!(format!("{error:#}").contains("envelope"), "{error:#}");
    }

    /// The probe documents step between the folds the library authors, so a
    /// stow document the author moves moves the instruments with it.
    #[test]
    fn the_probe_folds_are_the_library_s_own() {
        let folds = folds();
        let antennas = |name: &str| {
            poses()
                .iter()
                .find(|pose| pose.name() == name)
                .expect("the pose is committed")
                .targets()
                .antennas
        };
        let (neutral, stow) = (antennas("neutral"), antennas(reachy_poses::STOW_POSE));
        assert_eq!(
            Pose::Down.antennas(&folds),
            [stow[0] - neutral[0], stow[1] - neutral[1]]
        );
        // A library without both reserved poses is not one the table can author
        // an instrument against.
        let without: Vec<LoadedPose> = Vec::new();
        assert!(Folds::of(&without).is_err());
    }
}
