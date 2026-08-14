//! The motion library: named assets, resolved and flattened into playable
//! motions.
//!
//! Two things happen here. Assets arrive as documents and are read into their
//! validated forms, each failure skipped and reported rather than taking the
//! rest of the library down with it — a daemon with nineteen good clips and one
//! corrupt file should run nineteen clips. Then, once the whole set is in,
//! every sequence is resolved: references are looked up, cycles and dangling
//! names are refused, nesting depth is bounded, and the result is **flattened**
//! into a flat list of segments.
//!
//! Flattening is the point. Composition is a load-time construct; the runtime
//! only ever plays a flat list of (clip, effective speed, following gap). No
//! player walks a tree, no per-tick code can recurse, and a sequence that can
//! only ever be played out of bounds is refused here rather than at the moment
//! someone asks for it.
//!
//! Sans-I/O like the rest of the crate: documents arrive as strings the caller
//! read. Which directory they came from, and what to do about a skip, are the
//! daemon's business.

use std::collections::BTreeMap;
use std::fmt;
use std::sync::Arc;

use thiserror::Error;

use crate::format::{
    Channel, ChannelMask, Clip, ClipError, ClipNote, MAX_SPEED, MIN_SPEED, document_kind,
};
use crate::sequence::{Entry, Sequence, SequenceError};
use crate::speed::{ClipLimits, seam_step};

/// How deeply sequences may nest before a reference is refused.
///
/// Deep enough that composing composed motions is comfortable, shallow enough
/// that a flattened motion stays something a person can hold in their head when
/// it misbehaves on the bench.
pub const MAX_SEQUENCE_DEPTH: usize = 8;

/// How much slop a flattened speed is allowed against its bounds.
///
/// Effective speed is a product of the entry speeds through the nesting, and
/// binary floating point does not promise that `1.25 × 1.6` lands exactly on
/// `2.0`. Refusing a sequence over the last ulp of that product would be
/// refusing arithmetic rather than refusing motion; the tolerance is far below
/// any speed difference anybody could see or the machine could feel.
const SPEED_EPS: f64 = 1e-9;

/// One flattened step of a motion: a clip, the speed it runs at, and the hold
/// that follows it.
///
/// The speed is the product of every entry speed between the motion's root and
/// this clip — everything the nesting contributes. An invocation's own speed
/// multiplies it at play time and is not baked in here, because the same
/// flattened motion serves every invocation.
#[derive(Clone, Debug, PartialEq)]
pub struct Segment {
    clip: Arc<Clip>,
    speed: f64,
    gap_after_s: f64,
}

impl Segment {
    /// The clip this segment plays.
    #[must_use]
    pub fn clip(&self) -> &Arc<Clip> {
        &self.clip
    }

    /// The multiplier on the clip's own clock, from the nesting.
    #[must_use]
    pub fn speed(&self) -> f64 {
        self.speed
    }

    /// The hold following this clip's last frame, seconds, at 1.0×
    /// invocation speed. Zero when the next clip follows immediately.
    ///
    /// A hold freezes the clip's final delta; it does not return to the base.
    /// The base underneath is live throughout, so a hold rides it.
    #[must_use]
    pub fn gap_after_s(&self) -> f64 {
        self.gap_after_s
    }
}

/// A playable motion: what a name resolves to.
///
/// A clip becomes a one-segment motion and a sequence becomes an n-segment one,
/// so everything downstream — wire validation, the player, the daemon's window
/// arithmetic — has a single shape to handle and never asks which kind of asset
/// it was handed.
#[derive(Clone, Debug, PartialEq)]
pub struct Motion {
    name: String,
    description: Option<String>,
    mask: ChannelMask,
    lead_gap_s: f64,
    segments: Vec<Segment>,
    duration_s: f64,
    max_speed: f64,
    depth: usize,
}

impl Motion {
    /// The one-segment motion a bare clip plays as.
    fn from_clip(clip: Arc<Clip>) -> Self {
        let mask = clip.mask();
        let duration_s = clip.duration_s();
        let max_speed = clip.max_speed();
        Self {
            name: clip.name().to_owned(),
            description: clip.description().map(str::to_owned),
            mask,
            lead_gap_s: 0.0,
            segments: vec![Segment {
                clip,
                speed: 1.0,
                gap_after_s: 0.0,
            }],
            duration_s,
            max_speed,
            depth: 0,
        }
    }

    /// The library name this motion is invoked by.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The asset's description, if it carried one.
    #[must_use]
    pub fn description(&self) -> Option<&str> {
        self.description.as_deref()
    }

    /// Every channel any of this motion's clips drives.
    ///
    /// The union, not an intersection: a channel is *driven* by the motion even
    /// if only one segment touches it. Between such segments the channel simply
    /// contributes no delta, which is not the same as the motion having nothing
    /// to say about it.
    #[must_use]
    pub fn mask(&self) -> ChannelMask {
        self.mask
    }

    /// The hold before the first clip, seconds, at 1.0×.
    #[must_use]
    pub fn lead_gap_s(&self) -> f64 {
        self.lead_gap_s
    }

    /// The flattened segments, in play order. Never empty.
    #[must_use]
    pub fn segments(&self) -> &[Segment] {
        &self.segments
    }

    /// How long the motion runs at 1.0×, seconds, holds included.
    #[must_use]
    pub fn duration_s(&self) -> f64 {
        self.duration_s
    }

    /// How long the motion runs at `speed`, seconds.
    ///
    /// Gaps scale with the clips: an invocation at 2× plays the whole motion,
    /// holds and all, in half the time. A hold that kept its wall-clock length
    /// while the clips around it halved would be a different motion, not the
    /// same one faster.
    ///
    /// # Panics
    ///
    /// If `speed` is not finite and positive — the same invariant
    /// [`crate::ClipPlayer::joining_at`] holds, for the same reason: this
    /// duration is what the daemon's overlay-window arithmetic is built on, and
    /// a NaN or infinite window is a script that never stops being active.
    #[must_use]
    pub fn duration_s_at(&self, speed: f64) -> f64 {
        assert!(
            speed.is_finite() && speed > 0.0,
            "an invocation speed must be finite and positive, not {speed}"
        );
        self.duration_s / speed
    }

    /// The fastest invocation speed this motion may be played at.
    ///
    /// For a sequence this is the tightest child's limit divided by what the
    /// nesting already spends of it, so the bound holds for every segment at
    /// once, and never above the global ceiling.
    #[must_use]
    pub fn max_speed(&self) -> f64 {
        self.max_speed
    }

    /// How long this motion's overlay takes to fade out after its last frame,
    /// milliseconds. Determined by the final segment's clip alone.
    ///
    /// This is a wall-clock number independent of invocation speed — callers
    /// must add it to the motion's scaled duration, not scale it.
    #[must_use]
    pub fn blend_out_ms(&self) -> u32 {
        self.segments
            .last()
            .map_or(0, |segment| segment.clip.blend_out_ms())
    }
}

/// Why a sequence cannot be resolved against the library it lives in.
///
/// These are refusals of a whole asset, decided once at load. Nothing here can
/// happen at play time: a motion that exists has already been resolved.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum ResolveError {
    /// A reference naming no asset in the library.
    #[error("sequence {sequence:?} references {reference:?}, which is not in the library")]
    Dangling {
        /// The sequence holding the reference.
        sequence: String,
        /// The name it could not find.
        reference: String,
    },

    /// A reference that leads back to a sequence already being resolved.
    #[error("sequence reference cycle: {}", path.join(" -> "))]
    Cycle {
        /// The chain of sequence names, closing on its own first element.
        path: Vec<String>,
    },

    /// Nesting deeper than [`MAX_SEQUENCE_DEPTH`].
    ///
    /// Refused on the way down, before the descent it would cost: a sequence
    /// reached with the limit already spent is refused there and now, so the
    /// depth bounds the work and not only the number reported. Which asset a
    /// root's refusal names therefore depends on the chain, but *whether* a root
    /// is refused does not: the limit is spent inside a root's own tree exactly
    /// when that tree runs deeper than the limit, whichever order a directory is
    /// read in.
    #[error("sequence {sequence:?} nests {depth} deep; the limit is {MAX_SEQUENCE_DEPTH}")]
    TooDeep {
        /// The sequence at the offending depth.
        sequence: String,
        /// How deep the nesting had run when it was refused.
        depth: usize,
    },

    /// A sequence that flattens to nothing but holds.
    #[error("sequence {sequence:?} flattens to no clips at all; it would do nothing")]
    NoClips {
        /// The empty sequence.
        sequence: String,
    },

    /// A flattened speed outside the global invocation bounds.
    #[error(
        "sequence {sequence:?} plays clip {clip:?} at {speed}× even at 1.0×, \
         outside the {MIN_SPEED}–{MAX_SPEED} bounds"
    )]
    SpeedOutOfBounds {
        /// The sequence being resolved.
        sequence: String,
        /// The clip the nesting lands on.
        clip: String,
        /// The flattened speed.
        speed: f64,
    },

    /// A flattened speed above what that clip may be played at.
    #[error(
        "sequence {sequence:?} plays clip {clip:?} at {speed}× even at 1.0×, \
         above that clip's own limit of {max_speed}×"
    )]
    SpeedAboveClip {
        /// The sequence being resolved.
        sequence: String,
        /// The clip whose limit is exceeded.
        clip: String,
        /// The flattened speed.
        speed: f64,
        /// The clip's limit.
        max_speed: f64,
    },

    /// Two adjacent clips whose shared channels do not meet.
    ///
    /// At the tick where one segment's last frame gives way to the next
    /// segment's first, nothing interpolates and — for a channel both clips
    /// drive — nothing blends either, since the weight is already at one on
    /// both sides. The difference of the two deltas is commanded whole, in one
    /// tick, at every invocation speed, and a hold between them only postpones
    /// it. Past a step bound that tick is refused, which drops every overlay on
    /// the machine at the seam; refused at load instead, where the author can
    /// see it.
    #[error(
        "sequence {sequence:?} joins clip {from:?} to clip {to:?} with a step of \
         {step:.2}× the usable per-tick bound"
    )]
    Seam {
        /// The sequence being resolved.
        sequence: String,
        /// The outgoing clip.
        from: String,
        /// The incoming clip.
        to: String,
        /// The step the seam commands, as a multiple of the usable bound.
        step: f64,
    },
}

/// Why a document was skipped at load.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum LoadError {
    /// The bytes are not a document with a readable `kind`.
    #[error("document has no readable kind: {detail}")]
    Kind {
        /// The parser's own account of the problem.
        detail: String,
    },

    /// A `kind` this library has no reader for.
    #[error("document kind {kind:?} is neither a clip nor a sequence")]
    UnknownKind {
        /// What the document said.
        kind: String,
    },

    /// A clip document that would not load.
    #[error(transparent)]
    Clip(#[from] ClipError),

    /// A sequence document that would not load.
    #[error(transparent)]
    Sequence(#[from] SequenceError),

    /// A name a previously loaded asset already holds.
    ///
    /// Refused rather than overwritten: the name is what a script invokes, and
    /// which of two files wins would otherwise depend on directory order.
    #[error("name {name:?} is already loaded from {first_source:?}")]
    Duplicate {
        /// The contested name.
        name: String,
        /// Where the asset holding it came from.
        first_source: String,
    },

    /// A sequence that would not resolve against the rest of the library.
    #[error(transparent)]
    Resolve(#[from] ResolveError),
}

/// One skipped document, for the report the loader's caller writes.
#[derive(Clone, Debug, PartialEq)]
pub struct AssetSkip {
    /// Where the document came from, in whatever terms the caller used —
    /// typically a file path.
    pub source: String,
    /// The asset's name, when the document became an asset and was skipped
    /// afterwards — a duplicate, or a sequence that would not resolve.
    ///
    /// A document refused by its own reader carries none, whichever reader
    /// refused it: it never became an asset, so what identifies it is the file
    /// it came from. A name reported for some refusals and not others would be
    /// a field that means "sometimes".
    pub name: Option<String>,
    /// Why it was skipped.
    pub error: LoadError,
}

/// One thing a load changed about an asset it accepted.
///
/// The counterpart of [`AssetSkip`] for assets that loaded: a stretched blend
/// ramp or a cached speed the frames disagree with is a difference between the
/// file and what plays, and the operator hears about it the same way they hear
/// about a skip.
#[derive(Clone, Debug, PartialEq)]
pub struct AssetNote {
    /// Where the document came from.
    pub source: String,
    /// The asset's name.
    pub name: String,
    /// What the load changed.
    pub note: ClipNote,
}

impl fmt::Display for AssetNote {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}: {}", self.source, self.name, self.note)
    }
}

/// The loaded, resolved set of motions, addressed by name.
#[derive(Clone, Debug, Default)]
pub struct Library {
    motions: BTreeMap<String, Arc<Motion>>,
    notes: Vec<AssetNote>,
}

impl Library {
    /// The empty library — what a daemon with no clip directory runs.
    #[must_use]
    pub fn empty() -> Self {
        Self::default()
    }

    /// Load every document, then resolve.
    ///
    /// Each item is a source label and the document's text. `limits` are the
    /// bounds every clip's speed ceiling and blend floors are derived against —
    /// the running machine's, so a daemon whose step bounds are configured
    /// tighter than the defaults loads clips under its own.
    ///
    /// Returns the library alongside every document that was skipped, in the
    /// order the failures were decided.
    pub fn load<I, S, D>(documents: I, limits: &ClipLimits) -> (Self, Vec<AssetSkip>)
    where
        I: IntoIterator<Item = (S, D)>,
        S: Into<String>,
        D: AsRef<str>,
    {
        let mut builder = LibraryBuilder::new(limits.clone());
        for (source, document) in documents {
            builder.add_document(source, document.as_ref());
        }
        builder.build()
    }

    /// What the load changed about the assets it accepted.
    #[must_use]
    pub fn notes(&self) -> &[AssetNote] {
        &self.notes
    }

    /// The motion a name resolves to, if the library holds it.
    #[must_use]
    pub fn motion(&self, name: &str) -> Option<&Arc<Motion>> {
        self.motions.get(name)
    }

    /// Every loaded name, sorted.
    pub fn names(&self) -> impl Iterator<Item = &str> {
        self.motions.keys().map(String::as_str)
    }

    /// How many motions are loaded.
    #[must_use]
    pub fn len(&self) -> usize {
        self.motions.len()
    }

    /// Whether nothing is loaded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.motions.is_empty()
    }
}

/// The two-pass loader behind [`Library::load`].
///
/// Two passes because a directory hands its files over in an order nobody
/// chose, and a sequence may name an asset that has not been read yet.
#[derive(Debug, Default)]
pub struct LibraryBuilder {
    limits: ClipLimits,
    clips: BTreeMap<String, (String, Arc<Clip>)>,
    sequences: BTreeMap<String, (String, Sequence)>,
    skips: Vec<AssetSkip>,
    notes: Vec<AssetNote>,
}

impl LibraryBuilder {
    /// A loader holding nothing, deriving every clip against `limits`.
    #[must_use]
    pub fn new(limits: ClipLimits) -> Self {
        Self {
            limits,
            ..Self::default()
        }
    }

    /// Read one document, routing it by its `kind`.
    ///
    /// A document that will not load is recorded as a skip; the loader carries
    /// on with the rest.
    pub fn add_document<S: Into<String>>(&mut self, source: S, document: &str) {
        let source = source.into();
        let kind = match document_kind(document) {
            Ok(kind) => kind,
            Err(detail) => {
                self.skip(source, None, LoadError::Kind { detail });
                return;
            }
        };

        match kind.as_str() {
            "clip" => match Clip::from_json(document, &self.limits) {
                Ok(clip) => {
                    let name = clip.name().to_owned();
                    if let Some(first) = self.holder_of(&name) {
                        let error = LoadError::Duplicate {
                            name: name.clone(),
                            first_source: first,
                        };
                        self.skip(source, Some(name), error);
                    } else {
                        // Only an asset that is kept reports what its load
                        // changed: a note against a document losing a name
                        // fight describes something nobody will play.
                        for note in clip.notes() {
                            self.notes.push(AssetNote {
                                source: source.clone(),
                                name: name.clone(),
                                note: *note,
                            });
                        }
                        self.clips.insert(name, (source, Arc::new(clip)));
                    }
                }
                Err(error) => self.skip(source, None, error.into()),
            },
            "sequence" => match Sequence::from_json(document) {
                Ok(sequence) => {
                    let name = sequence.name().to_owned();
                    if let Some(first) = self.holder_of(&name) {
                        let error = LoadError::Duplicate {
                            name: name.clone(),
                            first_source: first,
                        };
                        self.skip(source, Some(name), error);
                    } else {
                        self.sequences.insert(name, (source, sequence));
                    }
                }
                Err(error) => self.skip(source, None, error.into()),
            },
            other => self.skip(
                source,
                None,
                LoadError::UnknownKind {
                    kind: other.to_owned(),
                },
            ),
        }
    }

    /// Resolve every sequence and produce the library.
    #[must_use]
    pub fn build(mut self) -> (Library, Vec<AssetSkip>) {
        // One `Motion` per clip, built once and shared by every reference to
        // it: two sequences naming the same clip hold the same `Arc`, so the
        // handle means what its type says it means.
        let mut clip_motions: BTreeMap<String, Arc<Motion>> = BTreeMap::new();
        for (name, (_, clip)) in &self.clips {
            clip_motions.insert(name.clone(), Arc::new(Motion::from_clip(clip.clone())));
        }
        let mut motions = clip_motions.clone();

        let mut resolved: BTreeMap<String, Result<Arc<Motion>, ResolveError>> = BTreeMap::new();
        let names: Vec<String> = self.sequences.keys().cloned().collect();
        for name in names {
            let mut stack = Vec::new();
            let outcome = resolve(
                &name,
                &clip_motions,
                &self.sequences,
                &self.limits,
                &mut resolved,
                &mut stack,
            );
            match outcome {
                Ok(motion) => {
                    motions.insert(name, motion);
                }
                Err(error) => {
                    let source = self.sequences[&name].0.clone();
                    self.skips.push(AssetSkip {
                        source,
                        name: Some(name),
                        error: error.into(),
                    });
                }
            }
        }

        (
            Library {
                motions,
                notes: self.notes,
            },
            self.skips,
        )
    }

    /// Where the asset already holding `name` came from, if one does.
    fn holder_of(&self, name: &str) -> Option<String> {
        self.clips
            .get(name)
            .map(|(source, _)| source.clone())
            .or_else(|| self.sequences.get(name).map(|(source, _)| source.clone()))
    }

    /// Record a skipped document.
    fn skip(&mut self, source: String, name: Option<String>, error: LoadError) {
        self.skips.push(AssetSkip {
            source,
            name,
            error,
        });
    }
}

/// Resolve one name to a flattened motion, memoising sequences.
///
/// `stack` holds the sequences currently being resolved: a reference back into
/// it is the cycle.
fn resolve(
    name: &str,
    clip_motions: &BTreeMap<String, Arc<Motion>>,
    sequences: &BTreeMap<String, (String, Sequence)>,
    limits: &ClipLimits,
    resolved: &mut BTreeMap<String, Result<Arc<Motion>, ResolveError>>,
    stack: &mut Vec<String>,
) -> Result<Arc<Motion>, ResolveError> {
    if let Some(motion) = clip_motions.get(name) {
        return Ok(motion.clone());
    }
    if let Some(outcome) = resolved.get(name) {
        return outcome.clone();
    }
    if stack.iter().any(|entry| entry == name) {
        let mut path = stack.clone();
        path.push(name.to_owned());
        return Err(ResolveError::Cycle { path });
    }
    let Some((_, sequence)) = sequences.get(name) else {
        // The caller is the sequence that named it; a root name that is not in
        // the library never reaches here, since only a reference does.
        return Err(ResolveError::Dangling {
            sequence: stack.last().cloned().unwrap_or_default(),
            reference: name.to_owned(),
        });
    };
    // The limit bounds the descent, not just the number it reports: a chain of
    // a few thousand generated sequences is a directory the deploy command can
    // push, and it has to come back as a skipped asset rather than as the
    // startup recursing until the stack gives out.
    if stack.len() >= MAX_SEQUENCE_DEPTH {
        return Err(ResolveError::TooDeep {
            sequence: name.to_owned(),
            depth: stack.len() + 1,
        });
    }

    stack.push(name.to_owned());
    let outcome = flatten(sequence, clip_motions, sequences, limits, resolved, stack).map(Arc::new);
    stack.pop();

    // Two refusals are properties of the chain rather than of the sequence:
    // a cycle's error names the whole chain, and a depth refusal counts the
    // chain that reached it — a sequence found too deep through one root can be
    // perfectly legal as its own. Memoising either would record one caller's
    // situation against every other's. Everything else is the sequence's own
    // and caches.
    if !matches!(
        outcome,
        Err(ResolveError::Cycle { .. } | ResolveError::TooDeep { .. })
    ) {
        resolved.insert(name.to_owned(), outcome.clone());
    }
    outcome
}

/// Flatten one sequence's entries into segments.
fn flatten(
    sequence: &Sequence,
    clip_motions: &BTreeMap<String, Arc<Motion>>,
    sequences: &BTreeMap<String, (String, Sequence)>,
    limits: &ClipLimits,
    resolved: &mut BTreeMap<String, Result<Arc<Motion>, ResolveError>>,
    stack: &mut Vec<String>,
) -> Result<Motion, ResolveError> {
    let mut lead_gap_s = 0.0;
    let mut segments: Vec<Segment> = Vec::new();
    let mut pending_gap_s = 0.0;
    let mut deepest_child = 0;

    for entry in sequence.entries() {
        match entry {
            Entry::Gap { ms } => pending_gap_s += f64::from(*ms) / 1000.0,
            Entry::Play { reference, speed } => {
                let child = resolve(reference, clip_motions, sequences, limits, resolved, stack)?;
                deepest_child = deepest_child.max(child.depth);
                // The child's own leading hold is a hold in this sequence too,
                // merging with whatever gap already stands before it.
                pending_gap_s += child.lead_gap_s / speed;
                place_gap(&mut lead_gap_s, &mut segments, &mut pending_gap_s);
                for segment in &child.segments {
                    let effective = segment.speed * speed;
                    check_speed(sequence.name(), &segment.clip, effective)?;
                    segments.push(Segment {
                        clip: segment.clip.clone(),
                        speed: effective,
                        gap_after_s: segment.gap_after_s / speed,
                    });
                }
            }
        }
    }

    if segments.is_empty() {
        return Err(ResolveError::NoClips {
            sequence: sequence.name().to_owned(),
        });
    }
    let depth = deepest_child + 1;
    if depth > MAX_SEQUENCE_DEPTH {
        return Err(ResolveError::TooDeep {
            sequence: sequence.name().to_owned(),
            depth,
        });
    }
    place_gap(&mut lead_gap_s, &mut segments, &mut pending_gap_s);
    check_seams(sequence.name(), &segments, limits)?;

    let mut mask = ChannelMask::empty();
    let mut duration_s = lead_gap_s;
    let mut max_speed = MAX_SPEED;
    for segment in &segments {
        for channel in Channel::ALL {
            if segment.clip.mask().contains(channel) {
                mask.insert(channel);
            }
        }
        duration_s += segment.clip.duration_s() / segment.speed + segment.gap_after_s;
        max_speed = max_speed.min(segment.clip.max_speed() / segment.speed);
    }
    // Every segment has just been proven legal at 1.0×, so the motion is
    // playable at 1.0× by construction. Without the floor it need not say so: a
    // nesting product landing an ulp above a child's own limit is admitted by
    // `SPEED_EPS` and divides back out an ulp *below* one, which would refuse
    // the default invocation of a motion the loader accepted.
    let max_speed = max_speed.max(1.0);

    Ok(Motion {
        name: sequence.name().to_owned(),
        description: sequence.description().map(str::to_owned),
        mask,
        lead_gap_s,
        segments,
        duration_s,
        max_speed,
        depth,
    })
}

/// Attach an accumulated hold to whatever precedes it.
///
/// Before the first clip that is the motion's leading hold; after one it
/// extends that clip's own trailing hold, which is how consecutive gaps — at
/// any nesting — merge into one.
fn place_gap(lead_gap_s: &mut f64, segments: &mut [Segment], pending_gap_s: &mut f64) {
    if *pending_gap_s == 0.0 {
        return;
    }
    match segments.last_mut() {
        Some(segment) => segment.gap_after_s += *pending_gap_s,
        None => *lead_gap_s += *pending_gap_s,
    }
    *pending_gap_s = 0.0;
}

/// Refuse a flattened list whose adjacent clips do not meet.
///
/// Run over the finished list rather than as each segment is pushed, so a seam
/// between two clips that arrived from different nestings is checked exactly
/// like one written side by side in a single sequence.
fn check_seams(
    sequence: &str,
    segments: &[Segment],
    limits: &ClipLimits,
) -> Result<(), ResolveError> {
    for pair in segments.windows(2) {
        let (_, out_last) = pair[0].clip.end_metrics();
        let (in_first, _) = pair[1].clip.end_metrics();
        let step = seam_step(
            &out_last,
            pair[0].clip.mask(),
            &in_first,
            pair[1].clip.mask(),
            limits,
        );
        if step > 1.0 {
            return Err(ResolveError::Seam {
                sequence: sequence.to_owned(),
                from: pair[0].clip.name().to_owned(),
                to: pair[1].clip.name().to_owned(),
                step,
            });
        }
    }
    Ok(())
}

/// Refuse a flattened speed the nesting alone has already pushed out of range.
fn check_speed(sequence: &str, clip: &Clip, speed: f64) -> Result<(), ResolveError> {
    if !(MIN_SPEED - SPEED_EPS..=MAX_SPEED + SPEED_EPS).contains(&speed) {
        return Err(ResolveError::SpeedOutOfBounds {
            sequence: sequence.to_owned(),
            clip: clip.name().to_owned(),
            speed,
        });
    }
    if speed > clip.max_speed() + SPEED_EPS {
        return Err(ResolveError::SpeedAboveClip {
            sequence: sequence.to_owned(),
            clip: clip.name().to_owned(),
            speed,
            max_speed: clip.max_speed(),
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::speed::STEP_MARGIN;

    /// The bounds every test derives clips against: the machine's defaults.
    fn limits() -> ClipLimits {
        ClipLimits::default()
    }

    use reachy_motion::FLOOR_TICK_HZ;

    /// A clip document of `frames` antennas-only frames, named `name`.
    /// The frames alternate between zero and one step, so the track derives
    /// exactly `max_speed` from its own per-frame deltas — the loader ignores
    /// the number in the document — and both its endpoints sit within one step
    /// of any other fixture's, which keeps a seam between two of them legal.
    fn clip_json(name: &str, frames: usize, max_speed: f64) -> String {
        let limits = limits();
        let step = limits.max_step.antennas * STEP_MARGIN / max_speed;
        let track: Vec<String> = (0..frames)
            .map(|index| format!("{{\"antennas\": [{}, 0.0]}}", (index % 2) as f64 * step))
            .collect();
        format!(
            r#"{{"version": 1, "kind": "clip", "name": "{name}",
                 "channels": ["antennas"], "frame_hz": {FLOOR_TICK_HZ},
                 "max_speed": {max_speed}, "frames": [{}]}}"#,
            track.join(",")
        )
    }

    /// A head-and-yaw clip, for mask-union checks.
    fn head_clip_json(name: &str) -> String {
        format!(
            r#"{{"version": 1, "kind": "clip", "name": "{name}",
                 "channels": ["head", "body_yaw"], "frame_hz": {FLOOR_TICK_HZ},
                 "max_speed": 2.0,
                 "frames": [{{"dt": [0.0, 0.0, 0.0], "dq": [1.0, 0.0, 0.0, 0.0],
                              "body_yaw": 0.1}}]}}"#
        )
    }

    /// A sequence document from raw entry JSON.
    fn sequence_json(name: &str, entries: &str) -> String {
        format!(r#"{{"version": 1, "kind": "sequence", "name": "{name}", "entries": [{entries}]}}"#)
    }

    /// A one-frame head clip holding the head `dz` metres off neutral.
    ///
    /// What a seam between two of these commands is the difference of their
    /// crank solutions, which is the branch of the seam check a track of
    /// antenna deltas never reaches.
    fn head_lift_json(name: &str, dz: f64) -> String {
        format!(
            r#"{{"version": 1, "kind": "clip", "name": "{name}",
                 "channels": ["head"], "frame_hz": {FLOOR_TICK_HZ},
                 "max_speed": 2.0,
                 "frames": [{{"dt": [0.0, 0.0, {dz}], "dq": [1.0, 0.0, 0.0, 0.0]}}]}}"#
        )
    }

    /// An antennas-only clip that *claims* a ceiling its frames do not derive,
    /// so the loader's disagreement is reportable.
    fn clip_json_claiming(name: &str, claims: f64) -> String {
        clip_json(name, 4, 2.0).replace("\"max_speed\": 2", &format!("\"max_speed\": {claims}"))
    }

    /// Load and require that nothing was skipped.
    fn library(documents: &[(&str, String)]) -> Library {
        let (library, skips) =
            Library::load(documents.iter().map(|(s, d)| (*s, d.as_str())), &limits());
        assert!(skips.is_empty(), "unexpected skips: {skips:?}");
        library
    }

    /// Load and require exactly one skip, returning it.
    fn one_skip(documents: &[(&str, String)]) -> AssetSkip {
        let (_, mut skips) =
            Library::load(documents.iter().map(|(s, d)| (*s, d.as_str())), &limits());
        assert_eq!(skips.len(), 1, "expected exactly one skip: {skips:?}");
        skips.remove(0)
    }

    #[test]
    fn a_clip_is_a_one_segment_motion() {
        let library = library(&[("a.json", clip_json("pod/wiggle", 50, 1.5))]);
        let motion = library.motion("pod/wiggle").expect("loaded");
        assert_eq!(motion.segments().len(), 1);
        assert_eq!(motion.segments()[0].speed(), 1.0);
        assert_eq!(motion.segments()[0].gap_after_s(), 0.0);
        assert_eq!(motion.lead_gap_s(), 0.0);
        assert!((motion.duration_s() - 1.0).abs() < 1e-12);
        assert!(
            (motion.max_speed() - 1.5).abs() < 1e-9,
            "{}",
            motion.max_speed()
        );
        assert!(motion.mask().contains(Channel::Antennas));
    }

    #[test]
    fn a_sequence_flattens_to_its_clips_with_gaps_between() {
        let library = library(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            ("b.json", clip_json("pod/b", 100, 2.0)),
            (
                "s.json",
                sequence_json(
                    "pod/greet",
                    r#"{"ref": "pod/a"}, {"gap_ms": 300}, {"ref": "pod/b", "speed": 2.0}"#,
                ),
            ),
        ]);
        let motion = library.motion("pod/greet").expect("loaded");
        assert_eq!(motion.segments().len(), 2);
        assert_eq!(motion.segments()[0].clip().name(), "pod/a");
        assert_eq!(motion.segments()[0].speed(), 1.0);
        assert!((motion.segments()[0].gap_after_s() - 0.3).abs() < 1e-12);
        assert_eq!(motion.segments()[1].clip().name(), "pod/b");
        assert_eq!(motion.segments()[1].speed(), 2.0);
        // 1 s + 0.3 s hold + 2 s at 2× = 2.3 s.
        assert!((motion.duration_s() - 2.3).abs() < 1e-12);
        assert!((motion.duration_s_at(2.0) - 1.15).abs() < 1e-12);
    }

    #[test]
    fn nested_entry_speeds_multiply_and_gaps_scale_with_them() {
        let library = library(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            (
                "inner.json",
                sequence_json(
                    "pod/inner",
                    r#"{"ref": "pod/a", "speed": 1.6}, {"gap_ms": 400}"#,
                ),
            ),
            (
                "outer.json",
                sequence_json("pod/outer", r#"{"ref": "pod/inner", "speed": 1.25}"#),
            ),
        ]);
        let motion = library.motion("pod/outer").expect("loaded");
        assert_eq!(motion.segments().len(), 1);
        // 1.6 × 1.25 = 2.0, the ceiling, admitted through the float tolerance.
        assert!((motion.segments()[0].speed() - 2.0).abs() < 1e-12);
        // The inner sequence's own hold is divided by the outer entry's speed.
        assert!((motion.segments()[0].gap_after_s() - 0.32).abs() < 1e-12);
        assert!((motion.duration_s() - (0.5 + 0.32)).abs() < 1e-12);
        // Nothing is left of the clip's own 2.0× ceiling — and the bound says
        // so from the legal side. The product lands an ulp either way of the
        // clip's limit, and a bound an ulp below 1.0 would refuse the default
        // invocation of a motion this load just accepted.
        assert!((motion.max_speed() - 1.0).abs() < 1e-9);
        assert!(motion.max_speed() >= 1.0, "{}", motion.max_speed());
    }

    #[test]
    fn a_motion_the_loader_accepted_is_always_playable_at_one_times() {
        // Every nesting that admits a clip at its exact limit, at each of the
        // speeds float arithmetic reaches it by.
        for (inner, outer) in [(1.6, 1.25), (0.5, 4.0), (2.0, 1.0), (0.8, 2.5)] {
            let library = library(&[
                ("a.json", clip_json("pod/a", 50, 2.0)),
                (
                    "inner.json",
                    sequence_json(
                        "pod/inner",
                        &format!(r#"{{"ref": "pod/a", "speed": {inner}}}"#),
                    ),
                ),
                (
                    "outer.json",
                    sequence_json(
                        "pod/outer",
                        &format!(r#"{{"ref": "pod/inner", "speed": {outer}}}"#),
                    ),
                ),
            ]);
            let motion = library.motion("pod/outer").expect("loaded");
            assert!(
                motion.max_speed() >= 1.0,
                "{inner} × {outer} gave {}",
                motion.max_speed()
            );
        }
    }

    #[test]
    fn a_leading_gap_holds_before_the_first_clip_and_consecutive_gaps_merge() {
        let library = library(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            (
                "s.json",
                sequence_json(
                    "pod/late",
                    r#"{"gap_ms": 200}, {"gap_ms": 300}, {"ref": "pod/a"}, {"gap_ms": 100}, {"gap_ms": 100}"#,
                ),
            ),
        ]);
        let motion = library.motion("pod/late").expect("loaded");
        assert!((motion.lead_gap_s() - 0.5).abs() < 1e-12);
        assert_eq!(motion.segments().len(), 1);
        assert!((motion.segments()[0].gap_after_s() - 0.2).abs() < 1e-12);
        assert!((motion.duration_s() - 1.7).abs() < 1e-12);
    }

    #[test]
    fn a_nested_leading_gap_merges_into_the_parents_hold() {
        let library = library(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            (
                "inner.json",
                sequence_json("pod/inner", r#"{"gap_ms": 400}, {"ref": "pod/a"}"#),
            ),
            (
                "outer.json",
                sequence_json(
                    "pod/outer",
                    r#"{"ref": "pod/a"}, {"gap_ms": 100}, {"ref": "pod/inner", "speed": 2.0}"#,
                ),
            ),
        ]);
        let motion = library.motion("pod/outer").expect("loaded");
        assert_eq!(motion.segments().len(), 2);
        // The parent's 100 ms plus the child's 400 ms halved by the entry speed.
        assert!((motion.segments()[0].gap_after_s() - 0.3).abs() < 1e-12);
        assert_eq!(motion.lead_gap_s(), 0.0);
    }

    #[test]
    fn a_sequence_mask_is_the_union_of_its_clips() {
        let library = library(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            ("h.json", head_clip_json("pod/h")),
            (
                "s.json",
                sequence_json("pod/both", r#"{"ref": "pod/a"}, {"ref": "pod/h"}"#),
            ),
        ]);
        let mask = library.motion("pod/both").expect("loaded").mask();
        assert!(mask.contains(Channel::Antennas));
        assert!(mask.contains(Channel::Head));
        assert!(mask.contains(Channel::BodyYaw));
    }

    #[test]
    fn a_sequences_max_speed_is_the_tightest_child_after_the_nesting_spends_it() {
        let library = library(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            ("b.json", clip_json("pod/b", 50, 1.2)),
            (
                "s.json",
                sequence_json("pod/s", r#"{"ref": "pod/a"}, {"ref": "pod/b"}"#),
            ),
        ]);
        let derived = library.motion("pod/s").expect("loaded").max_speed();
        assert!((derived - 1.2).abs() < 1e-9, "{derived}");
    }

    #[test]
    fn a_dangling_reference_is_refused() {
        let skip = one_skip(&[(
            "s.json",
            sequence_json("pod/s", r#"{"ref": "pod/missing"}"#),
        )]);
        assert_eq!(skip.name.as_deref(), Some("pod/s"));
        assert_eq!(
            skip.error,
            LoadError::Resolve(ResolveError::Dangling {
                sequence: "pod/s".to_owned(),
                reference: "pod/missing".to_owned(),
            })
        );
    }

    #[test]
    fn a_cycle_is_refused_and_names_the_chain() {
        let (library, skips) = Library::load(
            [
                ("a.json", sequence_json("pod/a", r#"{"ref": "pod/b"}"#)),
                ("b.json", sequence_json("pod/b", r#"{"ref": "pod/a"}"#)),
            ],
            &limits(),
        );
        assert!(library.is_empty());
        assert_eq!(skips.len(), 2, "{skips:?}");
        for skip in &skips {
            match &skip.error {
                LoadError::Resolve(ResolveError::Cycle { path }) => {
                    assert_eq!(path.first(), path.last(), "the chain closes: {path:?}");
                }
                other => panic!("expected a cycle refusal, got {other:?}"),
            }
        }
    }

    #[test]
    fn a_self_reference_is_a_cycle() {
        let skip = one_skip(&[("a.json", sequence_json("pod/a", r#"{"ref": "pod/a"}"#))]);
        assert!(matches!(
            skip.error,
            LoadError::Resolve(ResolveError::Cycle { .. })
        ));
    }

    #[test]
    fn nesting_deeper_than_the_limit_is_refused() {
        let mut documents = vec![("clip.json".to_owned(), clip_json("pod/a", 50, 2.0))];
        documents.push((
            "s0.json".to_owned(),
            sequence_json("pod/s0", r#"{"ref": "pod/a"}"#),
        ));
        for level in 1..=MAX_SEQUENCE_DEPTH {
            documents.push((
                format!("s{level}.json"),
                sequence_json(
                    &format!("pod/s{level}"),
                    &format!(r#"{{"ref": "pod/s{}"}}"#, level - 1),
                ),
            ));
        }
        let (library, skips) = Library::load(
            documents
                .iter()
                .map(|(source, doc)| (source.as_str(), doc.as_str())),
            &limits(),
        );
        // `pod/s0` is one deep, so `pod/s7` is the deepest admissible tree and
        // `pod/s8` is one past the limit however it is reached.
        assert!(library.motion("pod/s7").is_some());
        assert!(library.motion("pod/s8").is_none());
        assert_eq!(skips.len(), 1, "{skips:?}");
        assert_eq!(
            skips[0].error,
            LoadError::Resolve(ResolveError::TooDeep {
                sequence: "pod/s8".to_owned(),
                depth: MAX_SEQUENCE_DEPTH + 1,
            })
        );
    }

    #[test]
    fn a_long_chain_is_refused_on_the_way_down() {
        // The chain is named so the root sorts first and therefore resolves
        // before any of its links is memoised: the refusal has to come from the
        // descent itself, not from a cached child.
        const LINKS: usize = 200;
        let mut documents = vec![("clip.json".to_owned(), clip_json("pod/clip", 50, 2.0))];
        documents.push((
            "root.json".to_owned(),
            sequence_json("pod/root", r#"{"ref": "pod/z000"}"#),
        ));
        for link in 0..LINKS {
            let body = if link + 1 == LINKS {
                r#"{"ref": "pod/clip"}"#.to_owned()
            } else {
                format!(r#"{{"ref": "pod/z{:03}"}}"#, link + 1)
            };
            documents.push((
                format!("z{link:03}.json"),
                sequence_json(&format!("pod/z{link:03}"), &body),
            ));
        }
        let (library, skips) = Library::load(
            documents
                .iter()
                .map(|(source, doc)| (source.as_str(), doc.as_str())),
            &limits(),
        );

        assert!(library.motion("pod/root").is_none());
        let root_skip = skips
            .iter()
            .find(|skip| skip.name.as_deref() == Some("pod/root"))
            .expect("the root is skipped");
        // Eight names are on the stack — the root and seven links — when the
        // ninth is refused, so the recursion never goes past the limit.
        assert_eq!(
            root_skip.error,
            LoadError::Resolve(ResolveError::TooDeep {
                sequence: "pod/z007".to_owned(),
                depth: MAX_SEQUENCE_DEPTH + 1,
            })
        );
        // A refusal reached through one chain says nothing about the same
        // assets resolved as their own roots: the tail of the chain is shallow
        // and loads.
        assert!(library.motion(&format!("pod/z{:03}", LINKS - 1)).is_some());
        assert!(
            library
                .motion(&format!("pod/z{:03}", LINKS - MAX_SEQUENCE_DEPTH))
                .is_some()
        );
        assert!(
            library
                .motion(&format!("pod/z{:03}", LINKS - MAX_SEQUENCE_DEPTH - 1))
                .is_none()
        );
    }

    #[test]
    fn a_gap_only_sequence_is_refused() {
        let skip = one_skip(&[("s.json", sequence_json("pod/s", r#"{"gap_ms": 500}"#))]);
        assert_eq!(
            skip.error,
            LoadError::Resolve(ResolveError::NoClips {
                sequence: "pod/s".to_owned(),
            })
        );
    }

    #[test]
    fn a_gap_only_sequence_is_refused_through_nesting() {
        let (_, skips) = Library::load(
            [
                ("i.json", sequence_json("pod/i", r#"{"gap_ms": 500}"#)),
                ("o.json", sequence_json("pod/o", r#"{"ref": "pod/i"}"#)),
            ],
            &limits(),
        );
        assert_eq!(skips.len(), 2, "{skips:?}");
        assert!(skips.iter().all(|skip| matches!(
            &skip.error,
            LoadError::Resolve(ResolveError::NoClips { .. })
        )));
    }

    #[test]
    fn a_flattened_speed_above_the_clips_limit_is_refused() {
        let skip = one_skip(&[
            ("a.json", clip_json("pod/a", 50, 1.2)),
            (
                "s.json",
                sequence_json("pod/s", r#"{"ref": "pod/a", "speed": 1.5}"#),
            ),
        ]);
        match skip.error {
            LoadError::Resolve(ResolveError::SpeedAboveClip {
                clip, max_speed, ..
            }) => {
                assert_eq!(clip, "pod/a");
                assert_eq!(max_speed, 1.2);
            }
            other => panic!("expected a clip-limit refusal, got {other:?}"),
        }
    }

    #[test]
    fn a_flattened_speed_outside_the_global_bounds_is_refused() {
        let skip = one_skip(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            (
                "s.json",
                sequence_json("pod/s", r#"{"ref": "pod/a", "speed": 0.1}"#),
            ),
        ]);
        assert!(matches!(
            skip.error,
            LoadError::Resolve(ResolveError::SpeedOutOfBounds { .. })
        ));

        let skip = one_skip(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            (
                "s.json",
                sequence_json("pod/s", r#"{"ref": "pod/a", "speed": 4.0}"#),
            ),
        ]);
        assert!(matches!(
            skip.error,
            LoadError::Resolve(ResolveError::SpeedOutOfBounds { .. })
        ));
    }

    #[test]
    fn the_global_speed_bounds_admit_their_own_endpoints() {
        for speed in [MIN_SPEED, 1.0, MAX_SPEED] {
            let library = library(&[
                ("a.json", clip_json("pod/a", 50, 2.0)),
                (
                    "s.json",
                    sequence_json("pod/s", &format!(r#"{{"ref": "pod/a", "speed": {speed}}}"#)),
                ),
            ]);
            let motion = library.motion("pod/s").expect("loaded");
            assert!((motion.segments()[0].speed() - speed).abs() < 1e-12);
        }

        for speed in [MIN_SPEED - 1e-6, MAX_SPEED + 1e-6] {
            let skip = one_skip(&[
                ("a.json", clip_json("pod/a", 50, 2.0)),
                (
                    "s.json",
                    sequence_json("pod/s", &format!(r#"{{"ref": "pod/a", "speed": {speed}}}"#)),
                ),
            ]);
            match skip.error {
                LoadError::Resolve(ResolveError::SpeedOutOfBounds { speed: refused, .. }) => {
                    assert!((refused - speed).abs() < 1e-12);
                }
                other => panic!("expected a bounds refusal at {speed}×, got {other:?}"),
            }
        }
    }

    #[test]
    fn one_bad_document_does_not_take_the_library_with_it() {
        let (library, skips) = Library::load(
            [
                ("good.json", clip_json("pod/good", 50, 2.0)),
                ("corrupt.json", "{ not json".to_owned()),
                (
                    "wrong-rate.json",
                    clip_json("pod/bad", 50, 2.0).replace("50", "30"),
                ),
                ("empty-seq.json", sequence_json("pod/empty", "")),
            ],
            &limits(),
        );
        assert_eq!(library.len(), 1);
        assert!(library.motion("pod/good").is_some());
        assert_eq!(skips.len(), 3, "{skips:?}");
        assert!(matches!(skips[0].error, LoadError::Kind { .. }));
        assert_eq!(skips[0].source, "corrupt.json");
        assert!(matches!(
            skips[1].error,
            LoadError::Clip(ClipError::FrameRate { .. })
        ));
        assert_eq!(
            skips[2].error,
            LoadError::Sequence(SequenceError::NoEntries)
        );
        assert_eq!(skips[2].source, "empty-seq.json");
        assert_eq!(skips[2].name, None);
    }

    #[test]
    fn an_unreadable_kind_is_skipped() {
        let (library, skips) = Library::load(
            [(
                "x.json",
                r#"{"kind": "recording", "name": "pod/x"}"#.to_owned(),
            )],
            &limits(),
        );
        assert!(library.is_empty());
        assert_eq!(
            skips[0].error,
            LoadError::UnknownKind {
                kind: "recording".to_owned()
            }
        );
    }

    #[test]
    fn a_duplicate_name_is_refused_rather_than_overwritten() {
        let skip = one_skip(&[
            ("first.json", clip_json("pod/a", 50, 2.0)),
            ("second.json", clip_json("pod/a", 100, 1.0)),
        ]);
        assert_eq!(
            skip.error,
            LoadError::Duplicate {
                name: "pod/a".to_owned(),
                first_source: "first.json".to_owned(),
            }
        );

        // A sequence may not take a clip's name either.
        let skip = one_skip(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            ("s.json", sequence_json("pod/a", r#"{"ref": "pod/a"}"#)),
        ]);
        assert!(matches!(skip.error, LoadError::Duplicate { .. }));
    }

    #[test]
    fn the_empty_library_holds_nothing() {
        let library = Library::empty();
        assert!(library.is_empty());
        assert_eq!(library.motion("pod/anything"), None);
        assert_eq!(library.names().count(), 0);
    }

    #[test]
    fn a_sequence_referenced_twice_resolves_once_and_lands_twice() {
        let library = library(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            ("i.json", sequence_json("pod/i", r#"{"ref": "pod/a"}"#)),
            (
                "o.json",
                sequence_json("pod/o", r#"{"ref": "pod/i"}, {"ref": "pod/i"}"#),
            ),
        ]);
        let motion = library.motion("pod/o").expect("loaded");
        assert_eq!(motion.segments().len(), 2);
        assert!((motion.duration_s() - 2.0).abs() < 1e-12);
    }

    /// Two head clips that end and begin at poses further apart than one tick's
    /// leg travel: the handover commands the whole difference in one period, so
    /// the sequence is refused where the author can see it rather than on the
    /// machine mid-performance.
    #[test]
    fn a_sequence_whose_clips_do_not_meet_is_refused_at_load() {
        let skip = one_skip(&[
            ("low.json", head_lift_json("pod/low", 0.0)),
            ("high.json", head_lift_json("pod/high", 0.02)),
            (
                "s.json",
                sequence_json("pod/jump", r#"{"ref": "pod/low"}, {"ref": "pod/high"}"#),
            ),
        ]);
        let LoadError::Resolve(ResolveError::Seam {
            sequence,
            from,
            to,
            step,
        }) = skip.error
        else {
            panic!("expected a seam refusal: {:?}", skip.error);
        };
        assert_eq!(sequence, "pod/jump");
        assert_eq!(from, "pod/low");
        assert_eq!(to, "pod/high");
        assert!(step > 1.0, "{step}");

        // A seam the legs can cross in one period loads: the refusal is about
        // the distance, not about joining two head clips at all.
        let library = library(&[
            ("low.json", head_lift_json("pod/low", 0.0)),
            ("near.json", head_lift_json("pod/near", 0.0005)),
            (
                "s.json",
                sequence_json("pod/step", r#"{"ref": "pod/low"}, {"ref": "pod/near"}"#),
            ),
        ]);
        assert_eq!(
            library.motion("pod/step").expect("loaded").segments().len(),
            2
        );
    }

    /// What the loader changed about a clip reaches the operator through the
    /// library, named and attributed, rather than dying inside the clip.
    #[test]
    fn a_clips_load_notes_reach_the_library_named_and_attributed() {
        let library = library(&[("cautious.json", clip_json_claiming("pod/cautious", 1.05))]);
        let derived = library.motion("pod/cautious").expect("loaded").max_speed();
        assert_eq!(
            library.notes(),
            [AssetNote {
                source: "cautious.json".to_owned(),
                name: "pod/cautious".to_owned(),
                note: ClipNote::MaxSpeedDiffers {
                    stored: 1.05,
                    derived,
                },
            }]
        );
    }

    /// The document that loses a name fight reports nothing: a note describes
    /// an asset somebody will play, and this one is not in the library.
    #[test]
    fn the_losing_side_of_a_duplicate_name_notes_nothing() {
        let (library, skips) = Library::load(
            [
                ("first.json", clip_json("pod/a", 4, 2.0)),
                ("second.json", clip_json_claiming("pod/a", 1.05)),
            ],
            &limits(),
        );
        assert_eq!(skips.len(), 1);
        assert!(
            library.notes().is_empty(),
            "the loser's note escaped: {:?}",
            library.notes()
        );
    }

    #[test]
    fn a_motions_blend_out_is_its_last_clips() {
        let blending = |name: &str, blend_out_ms: u32| {
            clip_json(name, 4, 2.0).replace(
                "\"max_speed\": 2",
                &format!("\"blend_out_ms\": {blend_out_ms}, \"max_speed\": 2"),
            )
        };
        let library = library(&[
            ("first.json", blending("first", 600)),
            ("last.json", blending("last", 900)),
            (
                "seq.json",
                sequence_json(
                    "seq",
                    r#"{"ref": "first"}, {"gap_ms": 100}, {"ref": "last"}"#,
                ),
            ),
        ]);
        assert_eq!(library.motion("first").unwrap().blend_out_ms(), 600);
        assert_eq!(library.motion("seq").unwrap().blend_out_ms(), 900);
    }
}
