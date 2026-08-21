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
//! Flattening is the point. Composition is a load-time construct; what plays is
//! only ever a flat list of (clip, effective speed, following gap). No player
//! walks a tree, no per-tick code can recurse, and a sequence that can only ever
//! be played out of bounds is refused here rather than at the moment someone
//! asks for it.
//!
//! A clip is a one-segment motion, so a name resolves to the same shape
//! whichever kind of document it came from.
//!
//! Sans-I/O like the rest of the crate: documents arrive as strings the caller
//! read. Which directory they came from, and what to do about a skip, are the
//! daemon's business.

use std::collections::BTreeMap;
use std::fmt;

use thiserror::Error;

use crate::config::MAX_SEGMENTS;
use crate::format::{
    CLIP_KIND, Clip, ClipError, ClipNote, MAX_SPEED, MIN_SPEED, SEQUENCE_KIND, document_kind,
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
pub const SPEED_EPS: f64 = 1e-9;

/// Whether `speed` is a rate the machine plays at, to the tolerance above.
///
/// One statement of the arithmetic for everyone who asks: the flattening asks
/// of a nesting product and the play path asks again of the message that
/// carries it, and a tolerance the two read differently would let one accept
/// what the other refuses. A speed that is not finite is outside every range
/// and so is refused here too.
#[must_use]
pub fn speed_in_bounds(speed: f64) -> bool {
    (MIN_SPEED - SPEED_EPS..=MAX_SPEED + SPEED_EPS).contains(&speed)
}

/// Whether `speed` is within `ceiling`, the highest rate a clip's own frames
/// admit, to the same tolerance.
#[must_use]
pub fn within_ceiling(speed: f64, ceiling: f64) -> bool {
    speed <= ceiling + SPEED_EPS
}

/// One flattened step of a motion: a clip, the speed it runs at, and the hold
/// that follows it.
///
/// The speed is the product of every entry speed between the motion's root and
/// this clip — everything the nesting contributes. An invocation's own speed
/// multiplies it at play time and is not baked in here, because the same
/// flattened motion serves every invocation.
#[derive(Clone, Debug, PartialEq)]
pub struct Segment {
    clip: String,
    speed: f64,
    gap_after_s: f64,
}

impl Segment {
    /// The library name of the clip this segment plays.
    #[must_use]
    pub fn clip(&self) -> &str {
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
/// so everything downstream — the emitted configuration, the window arithmetic,
/// the player — has a single shape to handle and never asks which kind of asset
/// it was handed.
///
/// What the motion is *like* to play — how long it runs, which channels it
/// drives, how fast it may be invoked — is derived from its segments' clips
/// wherever it is needed, and is not stored here or in the emitted asset: a
/// stored derivation is a second opinion waiting to disagree with the frames.
#[derive(Clone, Debug, PartialEq)]
pub struct Motion {
    name: String,
    lead_gap_s: f64,
    segments: Vec<Segment>,
    depth: usize,
}

impl Motion {
    /// The one-segment motion a bare clip plays as.
    fn from_clip(name: &str) -> Self {
        Self {
            name: name.to_owned(),
            lead_gap_s: 0.0,
            segments: vec![Segment {
                clip: name.to_owned(),
                speed: 1.0,
                gap_after_s: 0.0,
            }],
            depth: 0,
        }
    }

    /// The library name this motion is invoked by.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The hold before the first clip, seconds, at 1.0×.
    ///
    /// The base alone holds through it: there is no previous segment whose
    /// delta could be frozen.
    #[must_use]
    pub fn lead_gap_s(&self) -> f64 {
        self.lead_gap_s
    }

    /// The flattened segments, in play order. Never empty.
    #[must_use]
    pub fn segments(&self) -> &[Segment] {
        &self.segments
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
        ///
        /// Only the cycle: a sequence that reaches one from outside is refused
        /// too, but the names it took to get there are not part of the loop and
        /// are not reported as if they were.
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

    /// A sequence flattening to more segments than one motion carries.
    ///
    /// Refused as the segments are pushed rather than once the list is
    /// finished: nesting multiplies, so eight small documents referencing each
    /// other can name more segments than any machine could hold, and the only
    /// bound that stops that is one applied where the list grows. What it is
    /// bounded to is what the emitted asset carries — a longer motion could
    /// never cross into the configuration, so flattening the rest of it buys
    /// nothing.
    #[error(
        "sequence {sequence:?} flattens past {MAX_SEGMENTS} segments, which is all one motion carries"
    )]
    TooManySegments {
        /// The sequence whose flattening ran past the bound.
        sequence: String,
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

    /// A sequence that would not resolve against the rest of the library.
    #[error(transparent)]
    Resolve(#[from] ResolveError),

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

/// One document a load accepted, in the order it was read.
///
/// The loader's own account of what became an asset and under which name. A
/// consumer that numbers clips or motions by document order takes the
/// association from here rather than probing the documents a second time: two
/// routings of one directory that disagree renumber the library.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AssetLoaded {
    /// Where the document came from.
    pub source: String,
    /// The name the asset loaded under.
    pub name: String,
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

/// The loaded, resolved set of assets, addressed by name.
#[derive(Clone, Debug, Default)]
pub struct Library {
    clips: BTreeMap<String, Clip>,
    motions: BTreeMap<String, Motion>,
    motions_loaded: Vec<AssetLoaded>,
    notes: Vec<AssetNote>,
}

impl Library {
    /// The empty library — what a daemon with no clip directory runs.
    #[must_use]
    pub fn empty() -> Self {
        Self::default()
    }

    /// Load every document.
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

    /// The clips the load accepted, in the order the documents were read.
    ///
    /// The motions filtered to the ones a clip document authored, which is the
    /// clip numbering: stating the filter here rather than storing a second
    /// vector is what keeps the two numberings the same read order.
    pub fn loaded(&self) -> impl Iterator<Item = &AssetLoaded> {
        self.motions_loaded
            .iter()
            .filter(|asset| self.clips.contains_key(&asset.name))
    }

    /// Every asset that became a motion, in the order the documents were read.
    ///
    /// Clips and resolved sequences together, since both play as motions. A
    /// sequence the resolution refused is not here: it is a skip.
    #[must_use]
    pub fn motions_loaded(&self) -> &[AssetLoaded] {
        &self.motions_loaded
    }

    /// The motion a name resolves to, if the library holds it.
    #[must_use]
    pub fn motion(&self, name: &str) -> Option<&Motion> {
        self.motions.get(name)
    }

    /// What the load changed about the assets it accepted.
    #[must_use]
    pub fn notes(&self) -> &[AssetNote] {
        &self.notes
    }

    /// The clip a name resolves to, if the library holds it.
    #[must_use]
    pub fn clip(&self, name: &str) -> Option<&Clip> {
        self.clips.get(name)
    }

    /// Every loaded clip name, sorted.
    pub fn clip_names(&self) -> impl Iterator<Item = &str> {
        self.clips.keys().map(String::as_str)
    }

    /// Every loaded motion name, sorted. Clips and resolved sequences together,
    /// since both play as motions.
    pub fn motion_names(&self) -> impl Iterator<Item = &str> {
        self.motions.keys().map(String::as_str)
    }

    /// How many clips are loaded. Not a motion count: a clip id and a motion id
    /// are different numberings, and only the second bounds what plays.
    #[must_use]
    pub fn clip_count(&self) -> usize {
        self.clips.len()
    }

    /// How many motions are loaded. A motion id is an index below this.
    #[must_use]
    pub fn motion_count(&self) -> usize {
        self.motions.len()
    }
}

/// The two-pass loader behind [`Library::load`].
///
/// Two passes because a directory hands its files over in an order nobody
/// chose, and a sequence may name an asset that has not been read yet.
#[derive(Debug, Default)]
pub struct LibraryBuilder {
    limits: ClipLimits,
    clips: BTreeMap<String, Clip>,
    sequences: BTreeMap<String, (String, Sequence)>,
    accepted: Vec<AssetLoaded>,
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
            CLIP_KIND => match Clip::from_json(document, &self.limits) {
                Ok(clip) => {
                    let name = clip.name().to_owned();
                    if let Some(error) = self.duplicate(&name) {
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
                        self.accepted.push(AssetLoaded {
                            source,
                            name: name.clone(),
                        });
                        self.clips.insert(name, clip);
                    }
                }
                Err(error) => self.skip(source, None, error.into()),
            },
            SEQUENCE_KIND => match Sequence::from_json(document) {
                Ok(sequence) => {
                    let name = sequence.name().to_owned();
                    if let Some(error) = self.duplicate(&name) {
                        self.skip(source, Some(name), error);
                    } else {
                        self.accepted.push(AssetLoaded {
                            source: source.clone(),
                            name: name.clone(),
                        });
                        self.sequences.insert(name, (source, sequence));
                    }
                }
                Err(error) => self.skip(source, None, error.into()),
            },
            _ => self.skip(source, None, LoadError::UnknownKind { kind }),
        }
    }

    /// Resolve every sequence and produce the library.
    #[must_use]
    pub fn build(mut self) -> (Library, Vec<AssetSkip>) {
        let mut motions: BTreeMap<String, Motion> = BTreeMap::new();
        for name in self.clips.keys() {
            motions.insert(name.clone(), Motion::from_clip(name));
        }

        let mut resolved: BTreeMap<String, Result<Motion, ResolveError>> = BTreeMap::new();
        let names: Vec<String> = self.sequences.keys().cloned().collect();
        for name in names {
            let mut stack = Vec::new();
            let outcome = resolve(
                &name,
                &self.clips,
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

        // The motion numbering is the read order of everything that survived
        // and plays; the clip numbering is that order filtered again, which
        // `Library::loaded` does.
        let motions_loaded: Vec<AssetLoaded> = self
            .accepted
            .into_iter()
            .filter(|asset| motions.contains_key(&asset.name))
            .collect();

        (
            Library {
                clips: self.clips,
                motions,
                motions_loaded,
                notes: self.notes,
            },
            self.skips,
        )
    }

    /// The refusal an already-taken name earns, if this one is taken.
    fn duplicate(&self, name: &str) -> Option<LoadError> {
        self.accepted
            .iter()
            .find(|asset| asset.name == name)
            .map(|first| LoadError::Duplicate {
                name: name.to_owned(),
                first_source: first.source.clone(),
            })
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
    clips: &BTreeMap<String, Clip>,
    sequences: &BTreeMap<String, (String, Sequence)>,
    limits: &ClipLimits,
    resolved: &mut BTreeMap<String, Result<Motion, ResolveError>>,
    stack: &mut Vec<String>,
) -> Result<Motion, ResolveError> {
    if clips.contains_key(name) {
        return Ok(Motion::from_clip(name));
    }
    if let Some(outcome) = resolved.get(name) {
        return outcome.clone();
    }
    if let Some(closes) = stack.iter().position(|entry| entry == name) {
        // From the first time the repeated name was entered: whatever chain
        // reached the loop from outside is refused with it, but naming those
        // sequences as members of the loop would point an author at files that
        // are only guilty of referring to it.
        let mut path = stack[closes..].to_vec();
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
    let outcome = flatten(sequence, clips, sequences, limits, resolved, stack);
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
    clips: &BTreeMap<String, Clip>,
    sequences: &BTreeMap<String, (String, Sequence)>,
    limits: &ClipLimits,
    resolved: &mut BTreeMap<String, Result<Motion, ResolveError>>,
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
                let child = resolve(reference, clips, sequences, limits, resolved, stack)?;
                deepest_child = deepest_child.max(child.depth);
                // The child's own leading hold is a hold in this sequence too,
                // merging with whatever gap already stands before it.
                pending_gap_s += child.lead_gap_s / speed;
                place_gap(&mut lead_gap_s, &mut segments, &mut pending_gap_s);
                for segment in &child.segments {
                    if segments.len() >= MAX_SEGMENTS {
                        return Err(ResolveError::TooManySegments {
                            sequence: sequence.name().to_owned(),
                        });
                    }
                    let effective = segment.speed * speed;
                    check_speed(sequence.name(), &segment.clip, clips, effective)?;
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
    check_seams(sequence.name(), &segments, clips, limits)?;

    Ok(Motion {
        name: sequence.name().to_owned(),
        lead_gap_s,
        segments,
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
    clips: &BTreeMap<String, Clip>,
    limits: &ClipLimits,
) -> Result<(), ResolveError> {
    for pair in segments.windows(2) {
        // A segment exists only because its name was found in this map, so the
        // lookups hold; a miss is this module having lost track of its own
        // invariant, not a document saying something.
        let (out_clip, in_clip) = (&clips[&pair[0].clip], &clips[&pair[1].clip]);
        let (_, out_last) = out_clip.end_metrics();
        let (in_first, _) = in_clip.end_metrics();
        let step = seam_step(
            &out_last,
            out_clip.mask(),
            &in_first,
            in_clip.mask(),
            limits,
        );
        if step > 1.0 {
            return Err(ResolveError::Seam {
                sequence: sequence.to_owned(),
                from: pair[0].clip.clone(),
                to: pair[1].clip.clone(),
                step,
            });
        }
    }
    Ok(())
}

/// Refuse a flattened speed the nesting alone has already pushed out of range.
fn check_speed(
    sequence: &str,
    clip: &str,
    clips: &BTreeMap<String, Clip>,
    speed: f64,
) -> Result<(), ResolveError> {
    if !speed_in_bounds(speed) {
        return Err(ResolveError::SpeedOutOfBounds {
            sequence: sequence.to_owned(),
            clip: clip.to_owned(),
            speed,
        });
    }
    // A segment exists only because its name was found in this map.
    let max_speed = clips[clip].max_speed();
    if !within_ceiling(speed, max_speed) {
        return Err(ResolveError::SpeedAboveClip {
            sequence: sequence.to_owned(),
            clip: clip.to_owned(),
            speed,
            max_speed,
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::format::Channel;
    use crate::speed::STEP_MARGIN;

    /// The bounds every test derives clips against: the machine's defaults.
    fn limits() -> ClipLimits {
        ClipLimits::default()
    }

    use reachy_motion::FLOOR_TICK_HZ;

    /// A clip document of `frames` antennas-only frames, named `name`.
    /// The frames alternate between zero and one step, so the track derives
    /// exactly `max_speed` from its own per-frame deltas — the loader ignores
    /// the number in the document.
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
    fn a_clip_loads_under_its_own_name() {
        let library = library(&[("a.json", clip_json("pod/wiggle", 50, 1.5))]);
        let clip = library.clip("pod/wiggle").expect("loaded");
        assert_eq!(clip.name(), "pod/wiggle");
        assert!((clip.duration_s() - 1.0).abs() < 1e-12);
        assert!(
            (clip.max_speed() - 1.5).abs() < 1e-9,
            "{}",
            clip.max_speed()
        );
        assert!(clip.mask().contains(Channel::Antennas));
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
        assert_eq!(library.clip_count(), 1);
        assert!(library.clip("pod/good").is_some());
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
        assert_eq!(library.clip_count(), 0);
        assert_eq!(
            skips[0].error,
            LoadError::UnknownKind {
                kind: "recording".to_owned()
            }
        );
    }

    #[test]
    fn a_clip_is_a_one_segment_motion() {
        let library = library(&[("a.json", clip_json("pod/wiggle", 50, 1.5))]);
        let motion = library.motion("pod/wiggle").expect("loaded");
        assert_eq!(motion.name(), "pod/wiggle");
        assert_eq!(motion.segments().len(), 1);
        assert_eq!(motion.segments()[0].clip(), "pod/wiggle");
        assert_eq!(motion.segments()[0].speed(), 1.0);
        assert_eq!(motion.segments()[0].gap_after_s(), 0.0);
        assert_eq!(motion.lead_gap_s(), 0.0);
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
        assert_eq!(motion.segments()[0].clip(), "pod/a");
        assert_eq!(motion.segments()[0].speed(), 1.0);
        assert!((motion.segments()[0].gap_after_s() - 0.3).abs() < 1e-12);
        assert_eq!(motion.segments()[1].clip(), "pod/b");
        assert_eq!(motion.segments()[1].speed(), 2.0);
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
        assert_eq!(library.motion_count(), 0);
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
    fn the_global_speed_bounds_admit_their_own_endpoints_and_refuse_past_them() {
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

    /// A sequence resolved once and referenced twice lands twice, and each
    /// landing carries its own entry's speed: what is memoised is the child as
    /// written, never a child already scaled by one caller's entry, which the
    /// second caller would then scale again.
    #[test]
    fn a_sequence_referenced_twice_lands_twice_at_each_entrys_own_speed() {
        let library = library(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            (
                "i.json",
                sequence_json(
                    "pod/i",
                    r#"{"ref": "pod/a"}, {"gap_ms": 200}, {"ref": "pod/a"}"#,
                ),
            ),
            (
                "o.json",
                sequence_json(
                    "pod/o",
                    r#"{"ref": "pod/i", "speed": 1.5}, {"ref": "pod/i", "speed": 0.5}"#,
                ),
            ),
        ]);
        let motion = library.motion("pod/o").expect("loaded");
        assert_eq!(motion.segments().len(), 4);
        for segment in motion.segments() {
            assert_eq!(segment.clip(), "pod/a");
        }
        let speeds: Vec<f64> = motion.segments().iter().map(Segment::speed).collect();
        assert_eq!(speeds, [1.5, 1.5, 0.5, 0.5]);

        // The child's own hold scales with the entry that reached it, so the
        // two landings hold for different lengths of time.
        assert!(
            (motion.segments()[0].gap_after_s() - 0.2 / 1.5).abs() < 1e-12,
            "{:?}",
            motion.segments()[0]
        );
        assert!(
            (motion.segments()[2].gap_after_s() - 0.2 / 0.5).abs() < 1e-12,
            "{:?}",
            motion.segments()[2]
        );

        // The child itself is untouched by either caller.
        let child = library.motion("pod/i").expect("loaded");
        assert_eq!(child.segments()[0].speed(), 1.0);
        assert_eq!(child.segments()[0].gap_after_s(), 0.2);
    }

    /// A sequence that is not itself in a cycle but reaches one is refused with
    /// it — and the chain it is refused with is the loop, not the approach.
    #[test]
    fn a_cycle_reached_from_outside_names_only_the_cycle() {
        let (library, skips) = Library::load(
            [
                ("a.json", sequence_json("pod/a", r#"{"ref": "pod/b"}"#)),
                ("b.json", sequence_json("pod/b", r#"{"ref": "pod/a"}"#)),
                ("c.json", sequence_json("pod/c", r#"{"ref": "pod/a"}"#)),
            ],
            &limits(),
        );
        assert_eq!(library.motion_count(), 0);
        assert_eq!(skips.len(), 3, "{skips:?}");
        let outside = skips
            .iter()
            .find(|skip| skip.name.as_deref() == Some("pod/c"))
            .expect("the approaching sequence is skipped too");
        let LoadError::Resolve(ResolveError::Cycle { path }) = &outside.error else {
            panic!("expected a cycle refusal: {:?}", outside.error);
        };
        assert_eq!(path.first(), path.last(), "the chain closes: {path:?}");
        assert!(
            !path.contains(&"pod/c".to_owned()),
            "the approach is named as part of the loop: {path:?}"
        );
        assert_eq!(path, &["pod/a", "pod/b", "pod/a"]);
    }

    /// Nesting multiplies, so the segment count is bounded where it grows: two
    /// levels of six references are thirty-six segments, and thirty-six is more
    /// than one motion carries. Without the bound at the push, eight levels of a
    /// handful of references each is a flattening no machine finishes.
    #[test]
    fn a_flattening_past_the_segment_bound_is_refused_as_it_grows() {
        let refs = |count: usize, name: &str| {
            (0..count)
                .map(|_| format!(r#"{{"ref": "{name}"}}"#))
                .collect::<Vec<String>>()
                .join(", ")
        };

        let skip = one_skip(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            ("l0.json", sequence_json("pod/l0", &refs(6, "pod/a"))),
            ("l1.json", sequence_json("pod/l1", &refs(6, "pod/l0"))),
        ]);
        assert_eq!(skip.name.as_deref(), Some("pod/l1"));
        assert_eq!(
            skip.error,
            LoadError::Resolve(ResolveError::TooManySegments {
                sequence: "pod/l1".to_owned(),
            })
        );

        // One past the bound is refused, and the bound itself loads: the last
        // segment a motion carries is one an author may write.
        let skip = one_skip(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            (
                "m.json",
                sequence_json("pod/many", &refs(MAX_SEGMENTS + 1, "pod/a")),
            ),
        ]);
        assert_eq!(
            skip.error,
            LoadError::Resolve(ResolveError::TooManySegments {
                sequence: "pod/many".to_owned(),
            })
        );
        let library = library(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            (
                "m.json",
                sequence_json("pod/many", &refs(MAX_SEGMENTS, "pod/a")),
            ),
        ]);
        assert_eq!(
            library.motion("pod/many").expect("loaded").segments().len(),
            MAX_SEGMENTS
        );
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

    /// The seam check runs over the finished flat list, so two clips that only
    /// ever meet because one sequence was nested after another are checked
    /// exactly like two written side by side. Neither child is refused: what
    /// does not meet is the composition, and that is what the refusal names.
    #[test]
    fn clips_that_meet_only_across_two_nestings_are_refused_the_same_way() {
        let (library, skips) = Library::load(
            [
                ("low.json", head_lift_json("pod/low", 0.0)),
                ("high.json", head_lift_json("pod/high", 0.02)),
                ("a.json", sequence_json("pod/a", r#"{"ref": "pod/low"}"#)),
                ("b.json", sequence_json("pod/b", r#"{"ref": "pod/high"}"#)),
                (
                    "r.json",
                    sequence_json("pod/root", r#"{"ref": "pod/a"}, {"ref": "pod/b"}"#),
                ),
            ],
            &limits(),
        );
        assert_eq!(skips.len(), 1, "{skips:?}");
        assert_eq!(skips[0].name.as_deref(), Some("pod/root"));
        let LoadError::Resolve(ResolveError::Seam {
            sequence, from, to, ..
        }) = &skips[0].error
        else {
            panic!("expected a seam refusal: {:?}", skips[0].error);
        };
        assert_eq!(
            (sequence.as_str(), from.as_str(), to.as_str()),
            ("pod/root", "pod/low", "pod/high")
        );
        assert!(library.motion("pod/a").is_some());
        assert!(library.motion("pod/b").is_some());
    }

    /// A hold between two clips that do not meet postpones the step; it does not
    /// soften it. The same whole difference is still commanded in one tick when
    /// the hold ends, so the seam is refused across a gap exactly as without it.
    #[test]
    fn a_hold_between_two_clips_does_not_excuse_a_seam() {
        let skip = one_skip(&[
            ("low.json", head_lift_json("pod/low", 0.0)),
            ("high.json", head_lift_json("pod/high", 0.02)),
            (
                "s.json",
                sequence_json(
                    "pod/jump",
                    r#"{"ref": "pod/low"}, {"gap_ms": 2000}, {"ref": "pod/high"}"#,
                ),
            ),
        ]);
        assert!(
            matches!(
                &skip.error,
                LoadError::Resolve(ResolveError::Seam { from, to, .. })
                    if from == "pod/low" && to == "pod/high"
            ),
            "{:?}",
            skip.error
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
        assert_eq!(
            skip.name.as_deref(),
            Some("pod/a"),
            "the document became an asset before it was refused, so it is named"
        );

        // A sequence may not take a clip's name either: what a script invokes
        // is one name over both kinds.
        let skip = one_skip(&[
            ("a.json", clip_json("pod/a", 50, 2.0)),
            ("s.json", sequence_json("pod/a", r#"{"ref": "pod/a"}"#)),
        ]);
        assert!(matches!(skip.error, LoadError::Duplicate { .. }));
    }

    /// What the load accepted, in the order it read the documents — the
    /// numbering a clip id is an index into. Not the names' order, which the
    /// library itself is kept in, and never the loser of a duplicate fight: a
    /// clip appearing here that nothing plays would shift every id after it.
    /// The motions carry a numbering of their own, over both kinds.
    #[test]
    fn what_loaded_answers_is_the_read_order_of_what_was_accepted() {
        let (library, skips) = Library::load(
            [
                ("c.json", clip_json("pod/c", 50, 1.0)),
                ("a.json", clip_json("pod/a", 50, 1.0)),
                ("again.json", clip_json("pod/a", 100, 1.0)),
                ("s.json", sequence_json("pod/s", r#"{"ref": "pod/a"}"#)),
                (
                    "dead.json",
                    sequence_json("pod/dead", r#"{"ref": "pod/x"}"#),
                ),
                ("b.json", clip_json("pod/b", 50, 1.0)),
            ],
            &limits(),
        );

        let motions: Vec<&str> = library
            .motions_loaded()
            .iter()
            .map(|asset| asset.name.as_str())
            .collect();
        assert_eq!(
            motions,
            ["pod/c", "pod/a", "pod/s", "pod/b"],
            "every asset that plays, in read order, and only those"
        );

        let loaded: Vec<(&str, &str)> = library
            .loaded()
            .map(|asset| (asset.source.as_str(), asset.name.as_str()))
            .collect();
        assert_eq!(
            loaded,
            [
                ("c.json", "pod/c"),
                ("a.json", "pod/a"),
                ("b.json", "pod/b")
            ]
        );
        assert_eq!(
            library.clip_names().collect::<Vec<_>>(),
            ["pod/a", "pod/b", "pod/c"],
            "the case rests on read order and name order differing"
        );
        assert_eq!(
            library.motion_names().collect::<Vec<_>>(),
            ["pod/a", "pod/b", "pod/c", "pod/s"],
            "a sequence is a motion and no clip"
        );
        assert_eq!(skips.len(), 2, "{skips:?}");
        assert_eq!(skips[0].source, "again.json");
        assert_eq!(skips[1].source, "dead.json");
    }

    #[test]
    fn the_empty_library_holds_nothing() {
        let library = Library::empty();
        assert_eq!(library.clip_count(), 0);
        assert_eq!(library.motion_count(), 0);
        assert!(library.clip("pod/anything").is_none());
        assert_eq!(library.clip_names().count(), 0);
        assert_eq!(library.motion_names().count(), 0);
    }

    /// What the loader changed about a clip reaches the operator through the
    /// library, named and attributed, rather than dying inside the clip.
    #[test]
    fn a_clips_load_notes_reach_the_library_named_and_attributed() {
        let library = library(&[("cautious.json", clip_json_claiming("pod/cautious", 1.05))]);
        let derived = library.clip("pod/cautious").expect("loaded").max_speed();
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
}
