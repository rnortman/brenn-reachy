//! The clip document: what is on disk, and what a load will accept.
//!
//! Two types for one asset, deliberately:
//!
//! - [`ClipDoc`] is the JSON, one field per key, nothing checked. It is what
//!   the importer writes and what `serde_json` parses.
//! - [`Clip`] is the validated form the player and the compositor take. Its
//!   invariants are established once, at load: the mask and the per-frame keys
//!   agree exactly, every number is finite, every rotation is a unit
//!   quaternion, and the frame rate is the tick rate.
//!
//! Nothing constructs a [`Clip`] except [`Clip::from_doc`] and its JSON
//! wrapper, so a `Clip` in hand is an asset that has already been refused the
//! chance to be malformed. Playback never re-checks a frame's shape; it indexes
//! and interpolates.
//!
//! The document carries an explicit `version`. The recorded-motion format this
//! replaces has none, which makes every future change to it undetectable by a
//! reader; a version that a wrong value fails loudly on is the cheapest
//! possible fix and is worth the one field.

use std::fmt;

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use reachy_motion::FLOOR_TICK_HZ;

use crate::speed::{ClipLimits, DeriveError, FrameMetrics, derive};

/// The document version this crate reads and writes.
///
/// A document carrying anything else is refused rather than read on a guess:
/// the fields a future version adds are exactly the ones whose absence a
/// permissive reader would silently take as a default.
pub const FORMAT_VERSION: u32 = 1;

/// The longest an asset name may be, characters.
///
/// Names are the join key between the wire and the library, so they travel in
/// every script that invokes a motion; a bound keeps a script's size a function
/// of its step count. The wire protocol defines an identical bound; charset
/// validation is this side's alone.
///
/// Must equal the corresponding constant in `motion-proto`; the two crates
/// share no dependency, so a drift guard in a downstream crate is the only
/// enforcement.
pub const MAX_MOTION_NAME_LEN: usize = 128;

/// The default blend ramp at an overlay's entry and exit, milliseconds, for a
/// clip that states neither.
///
/// Long enough that a clip opening on a large delta ramps in rather than steps,
/// short enough that a short emote is not mostly ramp. A clip that needs more
/// says so; one that asks for less than its own floor is stretched to the floor
/// rather than refused.
pub const DEFAULT_BLEND_MS: u32 = 200;

/// The slowest an invocation may run a motion.
///
/// Below this a motion degenerates into a creep that occupies the session
/// without reading as movement.
///
/// Must equal the wire protocol's bound in `motion-proto`; that copy is
/// authoritative.
pub const MIN_SPEED: f64 = 0.25;

/// The fastest an invocation may run a motion.
///
/// Above this even a gentle recording approaches the per-tick step bounds and
/// reads as a glitch rather than a motion. See [`MIN_SPEED`] for which copy of
/// this pair is authoritative.
pub const MAX_SPEED: f64 = 2.0;

/// How far a document's rotation quaternion may be from unit length before it
/// is refused, rather than renormalised.
///
/// JSON round-trips a normalised quaternion to well within this; anything
/// beyond it did not come out of a rotation, and normalising it would invent an
/// orientation nobody recorded.
pub const QUAT_NORM_TOL: f64 = 1e-6;

/// How far a document's cached `max_speed` may sit from the derived one before
/// the load says so.
///
/// The cache is written by a producer that ran the same derivation, so the only
/// difference a correct document shows is what the number lost passing through
/// a decimal literal. Anything wider is a document that was edited, produced by
/// a different derivation, or produced from different frames.
pub const SPEED_CACHE_TOL: f64 = 1e-6;

/// One of the three independently commandable target groups.
///
/// The head is a pose and the other two are angles, but for masking they are
/// peers: a clip drives a channel or says nothing about it. Per-side antenna
/// distinctions stay a tick-level concern — a clip that drives the antennas
/// drives the pair.
#[derive(Clone, Copy, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum Channel {
    /// The head pose in the body frame: a rigid delta, applied in the base
    /// head's own frame.
    Head,
    /// Body yaw, radians, additive.
    BodyYaw,
    /// Both antenna angles, right then left, radians, additive.
    Antennas,
}

impl Channel {
    /// Every channel, in document order.
    pub const ALL: [Self; 3] = [Self::Head, Self::BodyYaw, Self::Antennas];

    /// How many channels there are.
    pub const COUNT: usize = Self::ALL.len();

    /// The channel's slot in a [`PerChannel`] container.
    ///
    /// A match rather than a cast, so a variant added without a slot is a
    /// compile error rather than an index that aliases a neighbour's.
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::Head => 0,
            Self::BodyYaw => 1,
            Self::Antennas => 2,
        }
    }

    /// The channel's spelling in a document, for messages that name it.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Head => "head",
            Self::BodyYaw => "body_yaw",
            Self::Antennas => "antennas",
        }
    }
}

/// One `T` per [`Channel`].
///
/// The crate is full of per-channel quantities — the mask, the blend weights,
/// the fade ramp each channel is on — and each one hand-written is another
/// three-arm match to find when a channel is added or split. One container with
/// one [`Channel::index`] behind it keeps that dispatch in a single checked
/// place; the domain types above it stay distinct so a mask cannot be handed to
/// something expecting weights.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct PerChannel<T>([T; Channel::COUNT]);

impl<T> PerChannel<T> {
    /// A container holding `values`, in [`Channel::ALL`] order.
    pub const fn new(values: [T; Channel::COUNT]) -> Self {
        Self(values)
    }

    /// The value for `channel`.
    pub const fn get(&self, channel: Channel) -> &T {
        &self.0[channel.index()]
    }

    /// The value for `channel`, to write through.
    pub fn get_mut(&mut self, channel: Channel) -> &mut T {
        &mut self.0[channel.index()]
    }

    /// Replace the value for `channel`.
    pub fn set(&mut self, channel: Channel, value: T) {
        self.0[channel.index()] = value;
    }

    /// Every channel and its value, in document order.
    pub fn iter(&self) -> impl Iterator<Item = (Channel, &T)> {
        Channel::ALL.into_iter().map(|c| (c, self.get(c)))
    }
}

impl fmt::Display for Channel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.as_str())
    }
}

/// The set of channels an asset drives.
///
/// A set rather than a list: order carries no meaning, membership is what every
/// caller asks, and a sequence's mask is the union of its clips'. Three
/// channels, so it is three bools and nothing allocates.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct ChannelMask(PerChannel<bool>);

impl ChannelMask {
    /// The empty mask: an asset that drives nothing. Not a legal clip mask; it
    /// is the identity the union folds from.
    #[must_use]
    pub const fn empty() -> Self {
        Self(PerChannel::new([false; Channel::COUNT]))
    }

    /// The mask driving `channel` and nothing else.
    #[must_use]
    pub fn of(channel: Channel) -> Self {
        let mut mask = Self::empty();
        mask.insert(channel);
        mask
    }

    /// Whether `channel` is driven.
    #[must_use]
    pub const fn contains(self, channel: Channel) -> bool {
        *self.0.get(channel)
    }

    /// Add `channel`, reporting whether it was not already there.
    ///
    /// The polarity is that of the joint vocabulary's own set insertion:
    /// the return is what makes an entry an event, so a caller reading `if
    /// set.insert(x)` means the same thing whichever set it holds.
    pub fn insert(&mut self, channel: Channel) -> bool {
        let slot = self.0.get_mut(channel);
        let fresh = !*slot;
        *slot = true;
        fresh
    }

    /// The mask driving every channel either of these does.
    #[must_use]
    pub fn union(self, other: Self) -> Self {
        let mut out = self;
        for channel in Channel::ALL {
            if other.contains(channel) {
                out.insert(channel);
            }
        }
        out
    }

    /// Whether no channel at all is driven.
    #[must_use]
    pub fn is_empty(self) -> bool {
        Channel::ALL.into_iter().all(|c| !self.contains(c))
    }

    /// The driven channels, in document order.
    pub fn iter(self) -> impl Iterator<Item = Channel> {
        Channel::ALL.into_iter().filter(move |c| self.contains(*c))
    }
}

/// Why a name cannot be used.
///
/// Names reach a script, a report line and a file stem, so the charset is
/// narrow and the refusal says which character offended rather than restating
/// the rule.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum NameError {
    /// The name is the empty string.
    #[error("an asset name may not be empty")]
    Empty,

    /// The name is longer than [`MAX_MOTION_NAME_LEN`].
    #[error("an asset name may be at most {MAX_MOTION_NAME_LEN} characters; this one is {len}")]
    TooLong {
        /// The name's length in characters.
        len: usize,
    },

    /// A character outside `[a-z0-9_./-]`.
    #[error("an asset name may only hold [a-z0-9_./-]; this one holds {ch:?}")]
    BadChar {
        /// The first offending character.
        ch: char,
    },

    /// A leading `/`, a trailing `/`, or a `//`.
    #[error("an asset name may not hold an empty path segment")]
    EmptySegment,

    /// A `.` or `..` segment.
    #[error("an asset name may not hold a \".\" or \"..\" segment")]
    DotSegment,

    /// A leading `-`.
    #[error("an asset name may not begin with \"-\"")]
    LeadingDash,
}

/// Check an asset name against the charset, the length bound, and the shape a
/// relative path may take.
///
/// Every asset lives in one namespace, addressed by one wire field, so one
/// rule. The path shape belongs to that rule rather than to each consumer,
/// because a name *becomes* a path: the importer writes a clip and its audio
/// sidecar under it, and a name a consumer joins onto a directory is the whole
/// of what stops a downloaded, converted document from writing outside it. So a
/// name is a relative path with no navigation in it — no leading slash, no
/// empty segment, no `.` or `..` — and does not open with a `-`, which reads as
/// an option wherever a name reaches a command line.
pub fn validate_name(name: &str) -> Result<(), NameError> {
    if name.is_empty() {
        return Err(NameError::Empty);
    }
    let len = name.chars().count();
    if len > MAX_MOTION_NAME_LEN {
        return Err(NameError::TooLong { len });
    }
    if let Some(ch) = name
        .chars()
        .find(|ch| !matches!(ch, 'a'..='z' | '0'..='9' | '_' | '.' | '/' | '-'))
    {
        return Err(NameError::BadChar { ch });
    }
    if name.starts_with('-') {
        return Err(NameError::LeadingDash);
    }
    if name.split('/').any(str::is_empty) {
        return Err(NameError::EmptySegment);
    }
    if name
        .split('/')
        .any(|segment| segment == "." || segment == "..")
    {
        return Err(NameError::DotSegment);
    }
    Ok(())
}

/// Why a clip document cannot be loaded.
///
/// Every arm is a refusal of the whole asset. There is no partial load and no
/// repair: a clip whose frames disagree with its own mask describes a motion
/// nobody can say the shape of, and guessing which half is right is exactly the
/// silent substitution this stack refuses everywhere else.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum ClipError {
    /// The bytes are not the JSON this format is.
    #[error("clip document is malformed: {detail}")]
    Malformed {
        /// The parser's own account of the problem.
        detail: String,
    },

    /// A `version` this crate does not read.
    #[error("clip document is version {version}; this reader is version {FORMAT_VERSION}")]
    UnsupportedVersion {
        /// What the document said.
        version: u32,
    },

    /// A `kind` other than `clip`.
    #[error("expected a clip document; this one is kind {kind:?}")]
    WrongKind {
        /// What the document said.
        kind: String,
    },

    /// The `name` is not a usable asset name.
    #[error("clip name {name:?} is unusable: {source}")]
    Name {
        /// What the document said.
        name: String,
        /// Which rule it broke.
        source: NameError,
    },

    /// A `frame_hz` other than the tick rate the whole stack is floored at.
    ///
    /// Refused rather than resampled at load: resampling is the importer's job,
    /// paid once, offline. A daemon that quietly accepted another rate would
    /// play every frame at the wrong speed.
    #[error("clip frame rate is {frame_hz} Hz; clips are sampled at {FLOOR_TICK_HZ} Hz")]
    FrameRate {
        /// What the document said.
        frame_hz: f64,
    },

    /// An empty `channels` list: an asset that drives nothing does nothing.
    #[error("clip drives no channels")]
    NoChannels,

    /// The same channel listed twice.
    #[error("clip lists channel {channel} more than once")]
    DuplicateChannel {
        /// The repeated channel.
        channel: Channel,
    },

    /// An empty `frames` list.
    #[error("clip has no frames")]
    NoFrames,

    /// A frame missing a key for a channel the mask drives.
    #[error("frame {frame} is missing {key:?}, which channel {channel} needs")]
    MissingFrameKey {
        /// The frame's index.
        frame: usize,
        /// The absent key.
        key: &'static str,
        /// The channel that needs it.
        channel: Channel,
    },

    /// A frame carrying a key for a channel the mask does not drive.
    ///
    /// Refused, not ignored: a value present in the file and dropped at load is
    /// a motion the author believes is playing and nobody is commanding.
    #[error("frame {frame} carries {key:?}, but channel {channel} is not in the mask")]
    UnexpectedFrameKey {
        /// The frame's index.
        frame: usize,
        /// The stray key.
        key: &'static str,
        /// The channel it belongs to.
        channel: Channel,
    },

    /// A frame value that is not a finite number.
    #[error("frame {frame} key {key:?} is not finite: {value}")]
    NonFinite {
        /// The frame's index.
        frame: usize,
        /// The key holding it.
        key: &'static str,
        /// What the document said.
        value: f64,
    },

    /// A rotation quaternion too far from unit length to be one.
    #[error("frame {frame} rotation has norm {norm}, further than {QUAT_NORM_TOL} from unit")]
    Quaternion {
        /// The frame's index.
        frame: usize,
        /// The quaternion's norm as written.
        norm: f64,
    },

    /// A `max_speed` that is not a usable speed limit.
    #[error("clip max_speed must be finite and within (0, {MAX_SPEED}]; it is {max_speed}")]
    MaxSpeed {
        /// What the document said.
        max_speed: f64,
    },

    /// The frame track admits no derivation at all: applied to the neutral base
    /// it leaves the envelope, asks an antenna for an angle with no goal count,
    /// or blends in through a pose the head cannot hold.
    #[error("clip cannot be derived: {source}")]
    Derive {
        /// Which frame failed what.
        source: DeriveError,
    },

    /// The derived ceiling is below the slowest invocation the wire allows, so
    /// no legal speed plays this clip.
    ///
    /// Refused rather than loaded-and-unplayable: an asset that resolves by name
    /// and refuses every invocation of itself is a load-time fact reported at
    /// invocation time, which is the wrong place to find it.
    #[error(
        "clip's own frames admit at most {derived}x, below the slowest invocation {MIN_SPEED}x"
    )]
    Underivable {
        /// The speed ceiling the frames themselves impose.
        derived: f64,
    },

    /// An authored blend ramp longer than the clip it ramps.
    ///
    /// A blend-in longer than the clip never reaches full weight, so the motion
    /// is structurally attenuated and, at the factor-of-a-hundred typo this
    /// guards, invisible. A blend-out longer than the clip parks a finished
    /// overlay's final delta on the machine for as long as the fade runs. Both
    /// are content faults in the document, answered the way the format answers
    /// content faults. Only numbers the author wrote are judged: an omitted
    /// blend is capped at the clip's duration instead, and a ramp stretched to
    /// its derived floor is exempt.
    #[error("clip is {clip_ms} ms long; its authored {end} ramp of {blend_ms} ms is longer")]
    BlendExceedsClip {
        /// Which ramp.
        end: BlendEnd,
        /// What the document asked for.
        blend_ms: u32,
        /// The clip's own duration at 1.0x, milliseconds.
        clip_ms: f64,
    },
}

impl From<DeriveError> for ClipError {
    fn from(source: DeriveError) -> Self {
        Self::Derive { source }
    }
}

/// One frame of a clip document: the deltas for that instant, one key per
/// channel the clip drives.
///
/// Absent keys are how the document expresses its mask per frame, so the
/// options are load-bearing rather than convenience defaults; [`Clip::from_doc`]
/// requires them to agree with `channels` exactly, in both directions.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct FrameDoc {
    /// Head translation delta, metres, in the base head's own frame.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dt: Option<[f64; 3]>,
    /// Head rotation delta as a unit quaternion, `[w, x, y, z]`.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub dq: Option<[f64; 4]>,
    /// Antenna angle deltas, right then left, radians.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub antennas: Option<[f64; 2]>,
    /// Body yaw delta, radians.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub body_yaw: Option<f64>,
}

/// A clip as written on disk.
///
/// Unknown keys are refused. The format is ours end to end — the importer is
/// the only writer — so a key this reader does not know is a document from
/// somewhere else or a typo in a hand-authored asset, and both are worth
/// hearing about at load rather than at the bench.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct ClipDoc {
    /// Format version; [`FORMAT_VERSION`] or refused.
    pub version: u32,
    /// Discriminator against a sequence document; `clip`.
    pub kind: String,
    /// The library name this asset is invoked by.
    pub name: String,
    /// Free text, carried from the recording or written by the author.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// The mask: which channels this clip drives.
    pub channels: Vec<Channel>,
    /// Frame rate; the tick rate, or refused.
    pub frame_hz: f64,
    /// The highest invocation speed this clip may be played at.
    pub max_speed: f64,
    /// Entry blend ramp, milliseconds.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub blend_in_ms: Option<u32>,
    /// Exit blend ramp, milliseconds.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub blend_out_ms: Option<u32>,
    /// The frame track, uniformly sampled at `frame_hz`.
    pub frames: Vec<FrameDoc>,
}

/// One validated frame: the deltas for one instant, present exactly for the
/// channels the clip masks.
///
/// The head delta arrives as an isometry rather than the document's two arrays
/// because that is what composition multiplies and what interpolation walks;
/// converting once at load keeps the per-tick path free of it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DeltaFrame {
    /// Head delta, applied in the base head's local frame.
    pub head: Option<Isometry3<f64>>,
    /// Antenna deltas, right then left, radians.
    pub antennas: Option<[f64; 2]>,
    /// Body yaw delta, radians.
    pub body_yaw: Option<f64>,
}

impl DeltaFrame {
    /// The zero delta for `mask`: every masked channel present and neutral.
    ///
    /// What a leading gap contributes, and the identity a composition over no
    /// overlays reduces to.
    #[must_use]
    pub fn zero(mask: ChannelMask) -> Self {
        Self {
            head: mask.contains(Channel::Head).then(Isometry3::identity),
            antennas: mask.contains(Channel::Antennas).then_some([0.0, 0.0]),
            body_yaw: mask.contains(Channel::BodyYaw).then_some(0.0),
        }
    }

    /// The channels this frame drives.
    ///
    /// The inverse of [`DeltaFrame::zero`]'s argument: a frame carries a value
    /// exactly where its mask names a channel.
    #[must_use]
    pub fn mask(&self) -> ChannelMask {
        let mut mask = ChannelMask::empty();
        if self.head.is_some() {
            mask.insert(Channel::Head);
        }
        if self.antennas.is_some() {
            mask.insert(Channel::Antennas);
        }
        if self.body_yaw.is_some() {
            mask.insert(Channel::BodyYaw);
        }
        mask
    }

    /// Whether every number this frame carries is finite.
    ///
    /// Here beside the type rather than in a player, so a channel added to the
    /// frame is added to the check with it.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        let head = self.head.is_none_or(|head| {
            head.translation
                .vector
                .iter()
                .all(|value| value.is_finite())
                && head.rotation.coords.iter().all(|value| value.is_finite())
        });
        let antennas = self
            .antennas
            .is_none_or(|antennas| antennas.iter().all(|value| value.is_finite()));
        head && antennas && self.body_yaw.is_none_or(f64::is_finite)
    }

    /// How far this frame's rotation is from unit length, or `None` where the
    /// frame drives no head.
    ///
    /// A clip's frames are unit by construction; a frame from an unvalidated
    /// source is checked against [`QUAT_NORM_TOL`] through this.
    #[must_use]
    pub fn rotation_norm_error(&self) -> Option<f64> {
        self.head
            .map(|head| (head.rotation.coords.norm() - 1.0).abs())
    }

    /// The document form of this frame.
    fn to_doc(self) -> FrameDoc {
        let (dt, dq) = match self.head {
            Some(head) => {
                let q = head.rotation.quaternion();
                (
                    Some([
                        head.translation.vector.x,
                        head.translation.vector.y,
                        head.translation.vector.z,
                    ]),
                    Some([q.w, q.i, q.j, q.k]),
                )
            }
            None => (None, None),
        };
        FrameDoc {
            dt,
            dq,
            antennas: self.antennas,
            body_yaw: self.body_yaw,
        }
    }
}

/// A validated clip: a masked delta track, ready to play.
///
/// Every invariant a player relies on holds by construction — frames non-empty,
/// keys matching the mask, numbers finite, rotations unit — so sampling is
/// indexing and interpolation with nothing left to check.
#[derive(Clone, Debug, PartialEq)]
pub struct Clip {
    name: String,
    description: Option<String>,
    mask: ChannelMask,
    max_speed: f64,
    blend_in_ms: u32,
    blend_out_ms: u32,
    frames: Vec<DeltaFrame>,
    first_metrics: FrameMetrics,
    last_metrics: FrameMetrics,
    notes: Vec<ClipNote>,
}

/// Something a load changed about a clip, or found worth saying about it.
///
/// Not a refusal — the clip loaded — but not silent either: every one of these
/// means the asset on disk and the asset in memory differ, and an author
/// looking for why a motion is gentler than they wrote it needs the difference
/// reported rather than inferred.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum ClipNote {
    /// A configured ramp was shorter than its floor and was stretched to it.
    #[error(
        "{end} ramp of {configured_ms} ms is below its floor and was stretched to {floor_ms} ms"
    )]
    BlendStretched {
        /// Which ramp.
        end: BlendEnd,
        /// What the document asked for.
        configured_ms: u32,
        /// What it was stretched to.
        floor_ms: u32,
    },

    /// The document's cached `max_speed` was not what the frames derive.
    #[error("document caches max_speed {stored}x; the frames derive {derived}x, which stands")]
    MaxSpeedDiffers {
        /// What the document said.
        stored: f64,
        /// What the loader computed, and what the clip is played under.
        derived: f64,
    },
}

/// Which end of a clip a blend ramp belongs to.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BlendEnd {
    /// The entry ramp.
    In,
    /// The exit ramp.
    Out,
}

impl fmt::Display for BlendEnd {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(match self {
            Self::In => "blend-in",
            Self::Out => "blend-out",
        })
    }
}

impl Clip {
    /// Parse and validate a clip document.
    pub fn from_json(json: &str, limits: &ClipLimits) -> Result<Self, ClipError> {
        let doc: ClipDoc = serde_json::from_str(json).map_err(|err| ClipError::Malformed {
            detail: err.to_string(),
        })?;
        Self::from_doc(doc, limits)
    }

    /// Validate a parsed document.
    ///
    /// The order is deliberate: the document-level facts first — version, kind,
    /// name, rate, mask — so a wrong-version or wrong-kind file is refused as
    /// that rather than as whatever its frames happen to look like. The
    /// derivation runs last, because it is the only stage that solves the
    /// kinematics and it has nothing to say about a document whose shape is
    /// already wrong.
    pub fn from_doc(doc: ClipDoc, limits: &ClipLimits) -> Result<Self, ClipError> {
        if doc.version != FORMAT_VERSION {
            return Err(ClipError::UnsupportedVersion {
                version: doc.version,
            });
        }
        if doc.kind != CLIP_KIND {
            return Err(ClipError::WrongKind { kind: doc.kind });
        }
        validate_name(&doc.name).map_err(|source| ClipError::Name {
            name: doc.name.clone(),
            source,
        })?;
        if doc.frame_hz != FLOOR_TICK_HZ {
            return Err(ClipError::FrameRate {
                frame_hz: doc.frame_hz,
            });
        }
        if !doc.max_speed.is_finite() || doc.max_speed <= 0.0 || doc.max_speed > MAX_SPEED {
            return Err(ClipError::MaxSpeed {
                max_speed: doc.max_speed,
            });
        }

        let mut mask = ChannelMask::empty();
        for channel in &doc.channels {
            if !mask.insert(*channel) {
                return Err(ClipError::DuplicateChannel { channel: *channel });
            }
        }
        if mask.is_empty() {
            return Err(ClipError::NoChannels);
        }
        if doc.frames.is_empty() {
            return Err(ClipError::NoFrames);
        }

        let mut frames = Vec::with_capacity(doc.frames.len());
        for (index, frame) in doc.frames.iter().enumerate() {
            frames.push(delta_frame(index, frame, mask)?);
        }

        let derived = derive(&frames, mask, limits)?;
        if derived.max_speed < MIN_SPEED {
            return Err(ClipError::Underivable {
                derived: derived.max_speed,
            });
        }

        let mut notes = Vec::new();
        if (derived.max_speed - doc.max_speed).abs() > SPEED_CACHE_TOL {
            notes.push(ClipNote::MaxSpeedDiffers {
                stored: doc.max_speed,
                derived: derived.max_speed,
            });
        }
        let clip_ms = clip_duration_ms(frames.len());
        let blend_in_ms = floor_blend(
            authored_blend(doc.blend_in_ms, BlendEnd::In, clip_ms)?,
            derived.blend_in_floor_ms,
            BlendEnd::In,
            &mut notes,
        );
        let blend_out_ms = floor_blend(
            authored_blend(doc.blend_out_ms, BlendEnd::Out, clip_ms)?,
            derived.blend_out_floor_ms,
            BlendEnd::Out,
            &mut notes,
        );

        Ok(Self {
            name: doc.name,
            description: doc.description,
            mask,
            max_speed: derived.max_speed,
            blend_in_ms,
            blend_out_ms,
            frames,
            first_metrics: derived.first,
            last_metrics: derived.last,
            notes,
        })
    }

    /// The library name this clip is invoked by.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The author's or the recording's description, if it carried one.
    #[must_use]
    pub fn description(&self) -> Option<&str> {
        self.description.as_deref()
    }

    /// Which channels this clip drives.
    #[must_use]
    pub fn mask(&self) -> ChannelMask {
        self.mask
    }

    /// The highest invocation speed this clip may be played at.
    ///
    /// The loader's own computation over the frame track, never the document's.
    /// The number in the file is a cache written by whatever produced the clip;
    /// trusting it would let a hand-edited or mis-imported document raise its
    /// own ceiling, so it is checked for shape, compared, reported when it
    /// disagrees, and then discarded.
    #[must_use]
    pub fn max_speed(&self) -> f64 {
        self.max_speed
    }

    /// The first and last frames in joint coordinates over the neutral base.
    ///
    /// What a sequence measures a seam with: the step commanded where one
    /// clip's final delta is replaced by the next clip's first.
    #[must_use]
    pub fn end_metrics(&self) -> (FrameMetrics, FrameMetrics) {
        (self.first_metrics, self.last_metrics)
    }

    /// What the load changed about this clip, in the order it changed it.
    #[must_use]
    pub fn notes(&self) -> &[ClipNote] {
        &self.notes
    }

    /// Forget the notes, for a caller whose document was a request rather than
    /// a file.
    ///
    /// A note says the load disagreed with something a document claimed, which
    /// is a fact about a file somebody wrote. The importer's document is not
    /// one: it names the global ceiling as a placeholder and states no ramps at
    /// all, precisely so the derivation runs once — here, in the loader — and
    /// the numbers it settles on are the numbers that get written. Carrying the
    /// loader's corrections of those placeholders into a report would be the
    /// importer disagreeing with itself in front of an operator.
    pub(crate) fn forget_notes(&mut self) {
        self.notes.clear();
    }

    /// The entry blend ramp, milliseconds.
    #[must_use]
    pub fn blend_in_ms(&self) -> u32 {
        self.blend_in_ms
    }

    /// The exit blend ramp, milliseconds.
    #[must_use]
    pub fn blend_out_ms(&self) -> u32 {
        self.blend_out_ms
    }

    /// The frame track. Never empty.
    #[must_use]
    pub fn frames(&self) -> &[DeltaFrame] {
        &self.frames
    }

    /// How long the clip runs at 1.0×, seconds.
    ///
    /// One frame is one tick, and the first frame occupies its own period, so a
    /// track of `n` frames runs for `n` periods. A single-frame clip is a
    /// one-tick pose rather than an instant of nothing.
    #[must_use]
    pub fn duration_s(&self) -> f64 {
        self.duration_ms() / 1000.0
    }

    /// How long the clip runs at 1.0×, milliseconds.
    ///
    /// The same number the blend ceiling is judged against, from the same
    /// expression: a clip's length is one fact, and a second way of computing
    /// it is a second answer waiting to disagree with the first.
    #[must_use]
    pub fn duration_ms(&self) -> f64 {
        clip_duration_ms(self.frames.len())
    }

    /// The document form of this clip, for a writer.
    ///
    /// A ramp the load derived rather than read — a floor stretch, or a default
    /// capped at the clip — can be longer than the clip itself, which is a
    /// number no author may write. Written as authored it would produce a
    /// document this same loader refuses, so such a ramp is left unsaid and
    /// re-derived from the frames on the way back in. Everything within the
    /// clip's own length is written as it stands.
    ///
    /// TODO(clip-doc-faithful-blends): the omission makes the writer keep the
    /// reader's rule, and the document stops stating what plays.
    #[must_use]
    pub fn to_doc(&self) -> ClipDoc {
        let clip_ms = self.duration_ms();
        let writable = |ms: u32| (f64::from(ms) <= clip_ms).then_some(ms);
        ClipDoc {
            version: FORMAT_VERSION,
            kind: CLIP_KIND.to_owned(),
            name: self.name.clone(),
            description: self.description.clone(),
            channels: self.mask.iter().collect(),
            frame_hz: FLOOR_TICK_HZ,
            max_speed: self.max_speed,
            blend_in_ms: writable(self.blend_in_ms),
            blend_out_ms: writable(self.blend_out_ms),
            frames: self.frames.iter().map(|frame| frame.to_doc()).collect(),
        }
    }
}

/// A clip's own duration at 1.0x, milliseconds: one frame is one tick.
#[must_use]
pub fn clip_duration_ms(frames: usize) -> f64 {
    frames as f64 * 1000.0 / FLOOR_TICK_HZ
}

/// The ramp a document asks for, refused when the author wrote one longer than
/// the clip and capped at the clip's length when the author wrote nothing.
///
/// The two cases differ because the numbers have different authors. An
/// over-long ramp in the file is a content fault — the motion it describes
/// either never reaches full weight or fades for longer than it played — and
/// rewriting it silently would hand back a materially different motion with no
/// error, which is exactly how a factor-of-a-hundred typo stays invisible. The
/// default is the format's own number, not anyone's intent, so capping it is
/// the format correcting itself; refusing a short clip over a value its author
/// never wrote would leave no way to say "no blend" short of writing `0`.
///
/// Equality passes: a ramp exactly as long as the clip touches full weight at
/// the final frame.
fn authored_blend(
    configured_ms: Option<u32>,
    end: BlendEnd,
    clip_ms: f64,
) -> Result<u32, ClipError> {
    let Some(blend_ms) = configured_ms else {
        let cap = if clip_ms >= f64::from(DEFAULT_BLEND_MS) {
            DEFAULT_BLEND_MS
        } else {
            clip_ms as u32
        };
        return Ok(cap);
    };
    if f64::from(blend_ms) > clip_ms {
        return Err(ClipError::BlendExceedsClip {
            end,
            blend_ms,
            clip_ms,
        });
    }
    Ok(blend_ms)
}

/// Take the longer of a configured ramp and its floor, noting a stretch.
///
/// Never a refusal: a ramp too short for the step bounds is an authoring
/// mistake whose only consequence is a refused tick, and the stack's answer to
/// a duration below a floor everywhere else is to lengthen it and say so.
fn floor_blend(configured_ms: u32, floor_ms: u32, end: BlendEnd, notes: &mut Vec<ClipNote>) -> u32 {
    if configured_ms >= floor_ms {
        return configured_ms;
    }
    notes.push(ClipNote::BlendStretched {
        end,
        configured_ms,
        floor_ms,
    });
    floor_ms
}

/// Validate one document frame against the mask and convert it.
///
/// One rule, applied per key: a masked channel's key must be there, an unmasked
/// channel's key must not. The head needs both of its keys, so it applies the
/// rule twice and combines.
fn delta_frame(index: usize, frame: &FrameDoc, mask: ChannelMask) -> Result<DeltaFrame, ClipError> {
    let head_masked = mask.contains(Channel::Head);
    let dt = keyed(index, "dt", Channel::Head, head_masked, frame.dt)?;
    let dq = keyed(index, "dq", Channel::Head, head_masked, frame.dq)?;
    let head = match (dt, dq) {
        (Some(dt), Some(dq)) => Some(head_delta(index, dt, dq)?),
        _ => None,
    };

    let antennas = keyed(
        index,
        "antennas",
        Channel::Antennas,
        mask.contains(Channel::Antennas),
        frame.antennas,
    )?;
    if let Some(values) = antennas {
        finite(index, "antennas", values[0])?;
        finite(index, "antennas", values[1])?;
    }

    let body_yaw = keyed(
        index,
        "body_yaw",
        Channel::BodyYaw,
        mask.contains(Channel::BodyYaw),
        frame.body_yaw,
    )?;
    if let Some(value) = body_yaw {
        finite(index, "body_yaw", value)?;
    }

    Ok(DeltaFrame {
        head,
        antennas,
        body_yaw,
    })
}

/// Check one frame key against its channel's membership in the mask.
///
/// Present-and-masked passes the value through, absent-and-unmasked passes
/// nothing; the two disagreements are the two refusals, each naming the key and
/// the channel that wanted it.
fn keyed<T>(
    index: usize,
    key: &'static str,
    channel: Channel,
    masked: bool,
    value: Option<T>,
) -> Result<Option<T>, ClipError> {
    match (masked, value) {
        (true, Some(value)) => Ok(Some(value)),
        (true, None) => Err(ClipError::MissingFrameKey {
            frame: index,
            key,
            channel,
        }),
        (false, Some(_)) => Err(ClipError::UnexpectedFrameKey {
            frame: index,
            key,
            channel,
        }),
        (false, None) => Ok(None),
    }
}

/// Convert one frame's head keys into a rigid delta.
///
/// The quaternion is checked against unit length and then normalised: JSON's
/// decimal round-trip leaves a rotation a few ulps off unit, which is a
/// renormalisation, while anything past the tolerance is a number that was
/// never a rotation.
fn head_delta(index: usize, dt: [f64; 3], dq: [f64; 4]) -> Result<Isometry3<f64>, ClipError> {
    for value in dt {
        finite(index, "dt", value)?;
    }
    for value in dq {
        finite(index, "dq", value)?;
    }
    let quaternion = Quaternion::new(dq[0], dq[1], dq[2], dq[3]);
    let norm = quaternion.norm();
    if (norm - 1.0).abs() > QUAT_NORM_TOL {
        return Err(ClipError::Quaternion { frame: index, norm });
    }
    Ok(Isometry3::from_parts(
        Translation3::new(dt[0], dt[1], dt[2]),
        UnitQuaternion::from_quaternion(quaternion),
    ))
}

/// Refuse a frame value that is not a finite number.
fn finite(index: usize, key: &'static str, value: f64) -> Result<(), ClipError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(ClipError::NonFinite {
            frame: index,
            key,
            value,
        })
    }
}

/// Just the `kind` field of a document, tolerant of every other key.
#[derive(Clone, Debug, Deserialize)]
struct KindProbe {
    kind: String,
}

/// The `kind` a clip document declares.
pub const CLIP_KIND: &str = "clip";

/// The `kind` a sequence document declares.
pub const SEQUENCE_KIND: &str = "sequence";

/// Read a document's `kind` without committing to which asset it is.
///
/// A library directory holds whatever files somebody put in it, and deciding
/// which of them a clip reader is even offered happens before it can parse one.
/// Every routing site must use this probe: a second opinion about which files
/// are clips silently renumbers the library where ids are positions. Returns
/// the parser's account of an unreadable document rather than a [`ClipError`]:
/// the document may not be a clip at all, and the reader it routes to is what
/// refuses a malformed one.
pub fn document_kind(json: &str) -> Result<String, String> {
    serde_json::from_str::<KindProbe>(json)
        .map(|probe| probe.kind)
        .map_err(|err| err.to_string())
}

/// Just the `name` field of a document, tolerant of every other key.
#[derive(Clone, Debug, Deserialize)]
struct NameProbe {
    name: String,
}

/// Read the name a document claims, without loading it.
///
/// A name is how the whole stack addresses a motion — the wire carries names
/// and a [`crate::library::Library`] is keyed by them — while a path is how an operator
/// points at one file. This is the join between the two, for a caller holding a
/// path and needing the name the library will have filed that document under.
/// It validates nothing: what a name may be is the reader's business, and a
/// document whose name this answers may still be refused.
pub fn document_name(json: &str) -> Result<String, String> {
    serde_json::from_str::<NameProbe>(json)
        .map(|probe| probe.name)
        .map_err(|err| err.to_string())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The bounds every test derives clips against: the machine's defaults.
    fn limits() -> ClipLimits {
        ClipLimits::default()
    }

    /// A minimal well-formed document: one frame, every channel masked.
    fn full_doc() -> ClipDoc {
        ClipDoc {
            version: FORMAT_VERSION,
            kind: "clip".to_owned(),
            name: "pollen/emotions/loving1".to_owned(),
            description: Some("a test".to_owned()),
            channels: vec![Channel::Head, Channel::Antennas, Channel::BodyYaw],
            frame_hz: FLOOR_TICK_HZ,
            max_speed: 2.0,
            blend_in_ms: None,
            blend_out_ms: None,
            frames: vec![FrameDoc {
                dt: Some([0.0, 0.0, 0.01]),
                dq: Some([1.0, 0.0, 0.0, 0.0]),
                antennas: Some([0.1, -0.1]),
                body_yaw: Some(0.0),
            }],
        }
    }

    /// An antennas-only document — the canonical masked clip.
    fn antennas_doc() -> ClipDoc {
        ClipDoc {
            channels: vec![Channel::Antennas],
            frames: vec![
                FrameDoc {
                    antennas: Some([0.1, -0.1]),
                    ..FrameDoc::default()
                },
                FrameDoc {
                    antennas: Some([0.2, -0.2]),
                    ..FrameDoc::default()
                },
            ],
            ..full_doc()
        }
    }

    #[test]
    fn full_document_loads() {
        let clip = Clip::from_doc(full_doc(), &limits()).expect("well-formed");
        assert_eq!(clip.name(), "pollen/emotions/loving1");
        assert_eq!(clip.description(), Some("a test"));
        assert!(Channel::ALL.iter().all(|c| clip.mask().contains(*c)));
        assert_eq!(clip.frames().len(), 1);
        // One frame is 20 ms of clip, so the omitted default is capped there
        // and then stretched to the floor its own delta derives.
        assert_eq!(clip.blend_in_ms(), 180);
        assert_eq!(clip.blend_out_ms(), 60);
    }

    /// The two whole-frame checks a consumer of an unvalidated source makes
    /// before it hands frames to a player, over a frame that drives everything.
    #[test]
    fn a_frames_numbers_and_rotation_are_checked_whole() {
        let mask = ChannelMask::of(Channel::Head)
            .union(ChannelMask::of(Channel::Antennas))
            .union(ChannelMask::of(Channel::BodyYaw));
        let zero = DeltaFrame::zero(mask);
        assert!(zero.is_finite());
        assert_eq!(zero.rotation_norm_error(), Some(0.0));

        for spoiled in [
            DeltaFrame {
                head: Some(Isometry3::from_parts(
                    Translation3::new(0.0, f64::NAN, 0.0),
                    UnitQuaternion::identity(),
                )),
                ..zero
            },
            DeltaFrame {
                antennas: Some([0.0, f64::INFINITY]),
                ..zero
            },
            DeltaFrame {
                body_yaw: Some(f64::NAN),
                ..zero
            },
            // A rotation coordinate, which is the one number here the norm check
            // cannot catch: a NaN coordinate makes the norm NaN, and a NaN is not
            // further from unit than the tolerance.
            DeltaFrame {
                head: Some(Isometry3::from_parts(
                    Translation3::identity(),
                    UnitQuaternion::new_unchecked(Quaternion::new(f64::NAN, 0.0, 0.0, 1.0)),
                )),
                ..zero
            },
            DeltaFrame {
                head: Some(Isometry3::from_parts(
                    Translation3::identity(),
                    UnitQuaternion::new_unchecked(Quaternion::new(0.0, f64::INFINITY, 0.0, 1.0)),
                )),
                ..zero
            },
        ] {
            assert!(!spoiled.is_finite(), "{spoiled:?}");
        }

        // Why the finiteness check has to run first: a non-finite coordinate
        // leaves the norm error non-finite, so it is never *further* from unit
        // than the tolerance and the rotation check lets it past.
        let nan_rotation = DeltaFrame {
            head: Some(Isometry3::from_parts(
                Translation3::identity(),
                UnitQuaternion::new_unchecked(Quaternion::new(f64::NAN, 0.0, 0.0, 1.0)),
            )),
            ..zero
        };
        let error = nan_rotation
            .rotation_norm_error()
            .expect("the frame drives the head");
        assert!(error.is_nan());
        assert_eq!(error.partial_cmp(&QUAT_NORM_TOL), None);

        // A frame driving no head has no rotation to check.
        assert_eq!(
            DeltaFrame::zero(ChannelMask::of(Channel::BodyYaw)).rotation_norm_error(),
            None
        );
        let scaled = DeltaFrame {
            head: Some(Isometry3::from_parts(
                Translation3::identity(),
                UnitQuaternion::new_unchecked(Quaternion::new(1.5, 0.0, 0.0, 0.0)),
            )),
            ..zero
        };
        assert_eq!(scaled.rotation_norm_error(), Some(0.5));
    }

    #[test]
    fn duration_counts_one_period_per_frame() {
        let clip = Clip::from_doc(antennas_doc(), &limits()).expect("well-formed");
        assert!((clip.duration_s() - 2.0 / FLOOR_TICK_HZ).abs() < 1e-12);
    }

    #[test]
    fn masked_clip_leaves_other_channels_absent() {
        let clip = Clip::from_doc(antennas_doc(), &limits()).expect("well-formed");
        assert!(clip.mask().contains(Channel::Antennas));
        assert!(!clip.mask().contains(Channel::Head));
        assert!(!clip.mask().contains(Channel::BodyYaw));
        assert_eq!(clip.frames()[0].head, None);
        assert_eq!(clip.frames()[0].body_yaw, None);
        assert_eq!(clip.frames()[0].antennas, Some([0.1, -0.1]));
    }

    #[test]
    fn head_delta_becomes_an_isometry() {
        let clip = Clip::from_doc(full_doc(), &limits()).expect("well-formed");
        let head = clip.frames()[0].head.expect("head is masked");
        assert!((head.translation.vector.z - 0.01).abs() < 1e-15);
        assert_eq!(head.rotation, UnitQuaternion::identity());
    }

    #[test]
    fn wrong_version_is_refused() {
        let doc = ClipDoc {
            version: 2,
            ..full_doc()
        };
        assert_eq!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::UnsupportedVersion { version: 2 })
        );
    }

    #[test]
    fn wrong_kind_is_refused() {
        let doc = ClipDoc {
            kind: "sequence".to_owned(),
            ..full_doc()
        };
        assert_eq!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::WrongKind {
                kind: "sequence".to_owned()
            })
        );
    }

    #[test]
    fn frame_rate_must_be_the_tick_rate() {
        let doc = ClipDoc {
            frame_hz: 30.0,
            ..full_doc()
        };
        assert_eq!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::FrameRate { frame_hz: 30.0 })
        );
    }

    #[test]
    fn names_are_checked_against_the_charset() {
        assert_eq!(validate_name("pod/nod-twice_2.v1"), Ok(()));
        assert_eq!(validate_name(""), Err(NameError::Empty));
        assert_eq!(
            validate_name("Pollen/x"),
            Err(NameError::BadChar { ch: 'P' })
        );
        assert_eq!(validate_name("a b"), Err(NameError::BadChar { ch: ' ' }));
        let long = "a".repeat(MAX_MOTION_NAME_LEN + 1);
        assert_eq!(
            validate_name(&long),
            Err(NameError::TooLong {
                len: MAX_MOTION_NAME_LEN + 1
            })
        );
        // The bound itself is admissible: the refusal starts one past it.
        assert_eq!(validate_name(&"a".repeat(MAX_MOTION_NAME_LEN)), Ok(()));
    }

    #[test]
    fn names_may_not_navigate_a_filesystem() {
        // A name becomes a path — the importer writes each clip and its audio
        // sidecar under one — so the charset alone is not the rule: everything
        // below passes it and none of it may name a file outside the directory
        // it was joined onto.
        assert_eq!(validate_name("/etc/cron.d/x"), Err(NameError::EmptySegment));
        assert_eq!(validate_name("pollen//x"), Err(NameError::EmptySegment));
        assert_eq!(validate_name("pollen/"), Err(NameError::EmptySegment));
        assert_eq!(
            validate_name("../../persistent/x"),
            Err(NameError::DotSegment)
        );
        assert_eq!(validate_name("pollen/../../x"), Err(NameError::DotSegment));
        assert_eq!(validate_name("."), Err(NameError::DotSegment));
        assert_eq!(validate_name(".."), Err(NameError::DotSegment));
        assert_eq!(validate_name("--force"), Err(NameError::LeadingDash));
        assert_eq!(validate_name("-x"), Err(NameError::LeadingDash));

        // A dot inside a segment is still an ordinary character.
        assert_eq!(validate_name("pollen/emotions/loving1.v2"), Ok(()));
        assert_eq!(validate_name("...."), Ok(()));
    }

    #[test]
    fn bad_name_is_refused_by_the_clip_loader() {
        let doc = ClipDoc {
            name: "Loving1".to_owned(),
            ..full_doc()
        };
        assert_eq!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::Name {
                name: "Loving1".to_owned(),
                source: NameError::BadChar { ch: 'L' },
            })
        );
    }

    #[test]
    fn empty_mask_and_empty_track_are_refused() {
        let doc = ClipDoc {
            channels: vec![],
            frames: vec![FrameDoc::default()],
            ..full_doc()
        };
        assert_eq!(Clip::from_doc(doc, &limits()), Err(ClipError::NoChannels));

        let doc = ClipDoc {
            frames: vec![],
            ..full_doc()
        };
        assert_eq!(Clip::from_doc(doc, &limits()), Err(ClipError::NoFrames));
    }

    #[test]
    fn duplicate_channel_is_refused() {
        let doc = ClipDoc {
            channels: vec![Channel::Antennas, Channel::Antennas],
            ..antennas_doc()
        };
        assert_eq!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::DuplicateChannel {
                channel: Channel::Antennas
            })
        );
    }

    #[test]
    fn frame_missing_a_masked_key_is_refused() {
        let doc = ClipDoc {
            frames: vec![FrameDoc {
                antennas: None,
                ..antennas_doc().frames[0].clone()
            }],
            ..antennas_doc()
        };
        assert_eq!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::MissingFrameKey {
                frame: 0,
                key: "antennas",
                channel: Channel::Antennas,
            })
        );
    }

    #[test]
    fn head_needs_both_of_its_keys() {
        let mut doc = full_doc();
        doc.frames[0].dq = None;
        assert_eq!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::MissingFrameKey {
                frame: 0,
                key: "dq",
                channel: Channel::Head,
            })
        );

        let mut doc = full_doc();
        doc.frames[0].dt = None;
        assert_eq!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::MissingFrameKey {
                frame: 0,
                key: "dt",
                channel: Channel::Head,
            })
        );
    }

    #[test]
    fn frame_carrying_an_unmasked_key_is_refused() {
        let mut doc = antennas_doc();
        doc.frames[1].body_yaw = Some(0.4);
        assert_eq!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::UnexpectedFrameKey {
                frame: 1,
                key: "body_yaw",
                channel: Channel::BodyYaw,
            })
        );

        let mut doc = antennas_doc();
        doc.frames[0].dq = Some([1.0, 0.0, 0.0, 0.0]);
        assert_eq!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::UnexpectedFrameKey {
                frame: 0,
                key: "dq",
                channel: Channel::Head,
            })
        );
    }

    #[test]
    fn non_finite_values_are_refused() {
        let mut doc = antennas_doc();
        doc.frames[1].antennas = Some([0.1, f64::NAN]);
        // NaN is not equal to itself, so the refusal is matched rather than
        // compared.
        match Clip::from_doc(doc, &limits()) {
            Err(ClipError::NonFinite { frame, key, value }) => {
                assert_eq!((frame, key), (1, "antennas"));
                assert!(value.is_nan());
            }
            other => panic!("expected a finiteness refusal, got {other:?}"),
        }

        let mut doc = full_doc();
        doc.frames[0].dt = Some([0.0, f64::INFINITY, 0.0]);
        assert_eq!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::NonFinite {
                frame: 0,
                key: "dt",
                value: f64::INFINITY,
            })
        );
    }

    #[test]
    fn quaternion_beyond_tolerance_is_refused() {
        let mut doc = full_doc();
        doc.frames[0].dq = Some([1.1, 0.0, 0.0, 0.0]);
        match Clip::from_doc(doc, &limits()) {
            Err(ClipError::Quaternion { frame, norm }) => {
                assert_eq!(frame, 0);
                assert!((norm - 1.1).abs() < 1e-12);
            }
            other => panic!("expected a quaternion refusal, got {other:?}"),
        }
    }

    #[test]
    fn quaternion_within_tolerance_is_renormalised() {
        let mut doc = full_doc();
        // A round-trip's worth of drift, an order of magnitude inside the
        // tolerance.
        doc.frames[0].dq = Some([1.0 + 1e-7, 0.0, 0.0, 0.0]);
        let clip = Clip::from_doc(doc, &limits()).expect("within tolerance");
        let head = clip.frames()[0].head.expect("head is masked");
        assert!((head.rotation.quaternion().norm() - 1.0).abs() < 1e-15);
    }

    #[test]
    fn max_speed_must_be_a_usable_limit() {
        for bad in [0.0, -1.0, MAX_SPEED + 0.1, f64::NAN, f64::INFINITY] {
            let doc = ClipDoc {
                max_speed: bad,
                ..full_doc()
            };
            assert!(
                matches!(
                    Clip::from_doc(doc, &limits()),
                    Err(ClipError::MaxSpeed { .. })
                ),
                "max_speed {bad} should be refused"
            );
        }
    }

    #[test]
    fn unknown_keys_are_refused() {
        let json = r#"{
            "version": 1, "kind": "clip", "name": "pod/x",
            "channels": ["antennas"], "frame_hz": 50.0, "max_speed": 1.0,
            "loop": true,
            "frames": [{"antennas": [0.0, 0.0]}]
        }"#;
        assert!(matches!(
            Clip::from_json(json, &limits()),
            Err(ClipError::Malformed { .. })
        ));
    }

    #[test]
    fn json_round_trips_through_the_document() {
        let clip = Clip::from_doc(full_doc(), &limits()).expect("well-formed");
        let json = serde_json::to_string(&clip.to_doc()).expect("serialisable");
        let reloaded = Clip::from_json(&json, &limits()).expect("round-trips");
        assert_eq!(reloaded, clip);
    }

    #[test]
    fn document_omits_the_keys_a_mask_excludes() {
        let clip = Clip::from_doc(antennas_doc(), &limits()).expect("well-formed");
        let json = serde_json::to_string(&clip.to_doc()).expect("serialisable");
        assert!(!json.contains("\"dt\""), "{json}");
        assert!(!json.contains("\"dq\""), "{json}");
        assert!(!json.contains("\"body_yaw\""), "{json}");
        assert!(json.contains("\"antennas\""), "{json}");
    }

    #[test]
    fn kind_probe_reads_a_document_it_cannot_parse() {
        let json = r#"{"kind": "sequence", "entries": [{"ref": "pod/x"}]}"#;
        assert_eq!(document_kind(json).as_deref(), Ok("sequence"));
        assert!(document_kind("not json").is_err());
    }

    #[test]
    fn the_name_probe_reads_a_document_it_cannot_parse() {
        let json = r#"{"kind": "sequence", "name": "pod/greet", "entries": []}"#;
        assert_eq!(document_name(json).as_deref(), Ok("pod/greet"));
        assert!(document_name("not json").is_err());
        assert!(document_name(r#"{"kind": "clip"}"#).is_err());
    }

    #[test]
    fn zero_delta_is_present_exactly_for_the_mask() {
        let mut mask = ChannelMask::empty();
        mask.insert(Channel::Head);
        let zero = DeltaFrame::zero(mask);
        assert_eq!(zero.head, Some(Isometry3::identity()));
        assert_eq!(zero.antennas, None);
        assert_eq!(zero.body_yaw, None);
    }

    #[test]
    fn mask_union_and_iteration() {
        let mut left = ChannelMask::empty();
        left.insert(Channel::Head);
        let mut right = ChannelMask::empty();
        right.insert(Channel::Antennas);
        let union = left.union(right);
        assert_eq!(
            union.iter().collect::<Vec<_>>(),
            vec![Channel::Head, Channel::Antennas]
        );
        assert!(!union.contains(Channel::BodyYaw));
        assert!(ChannelMask::empty().is_empty());
    }

    #[test]
    fn channel_slots_match_the_declared_order() {
        for (slot, channel) in Channel::ALL.into_iter().enumerate() {
            assert_eq!(channel.index(), slot, "{channel} sits in its own slot");
        }
    }

    #[test]
    fn per_channel_holds_one_value_each() {
        let mut values = PerChannel::new([1_u32; Channel::COUNT]);
        values.set(Channel::BodyYaw, 7);
        assert_eq!(*values.get(Channel::BodyYaw), 7);
        assert_eq!(*values.get(Channel::Head), 1);
        assert_eq!(*values.get(Channel::Antennas), 1);
        assert_eq!(
            values.iter().map(|(c, _)| c).collect::<Vec<_>>(),
            Channel::ALL.to_vec()
        );
    }

    #[test]
    fn mask_insert_reports_a_fresh_channel() {
        let mut mask = ChannelMask::empty();
        assert!(mask.insert(Channel::Head), "first insert is the event");
        assert!(!mask.insert(Channel::Head), "the second is not");
    }

    /// A document asking for no entry ramp at all, over a delta several step
    /// bounds wide: the load lengthens the ramp to its floor and says so, which
    /// is the policy the player now relies on rather than defending itself.
    #[test]
    fn a_ramp_below_its_floor_is_stretched_and_noted() {
        let limits = limits();
        let big = limits.max_step.antennas * crate::speed::STEP_MARGIN * 4.0;
        let doc = ClipDoc {
            blend_in_ms: Some(0),
            blend_out_ms: Some(600),
            // Long enough that the authored exit ramp fits inside the clip:
            // holding the same delta leaves both derived floors where they are.
            frames: std::iter::once(FrameDoc {
                antennas: Some([0.0, 0.0]),
                ..FrameDoc::default()
            })
            .chain((0..30).map(|_| FrameDoc {
                antennas: Some([big, 0.0]),
                ..FrameDoc::default()
            }))
            .collect(),
            ..antennas_doc()
        };
        let clip = Clip::from_doc(doc, &limits).expect("in range");

        // Four usable-widths over the ramp's fifth of a bound: sixteen ticks.
        assert_eq!(clip.blend_in_ms(), 320);
        assert_eq!(clip.blend_out_ms(), 600, "a ramp above its floor is left");
        assert!(
            clip.notes().contains(&ClipNote::BlendStretched {
                end: BlendEnd::In,
                configured_ms: 0,
                floor_ms: 320,
            }),
            "{:?}",
            clip.notes()
        );
        assert!(
            !clip.notes().iter().any(|note| matches!(
                note,
                ClipNote::BlendStretched {
                    end: BlendEnd::Out,
                    ..
                }
            )),
            "the exit ramp was long enough and is not reported"
        );
    }

    /// The write and the read are one invariant across two functions: a clip
    /// whose derived ramps outrun its own length must survive a round trip
    /// through the document, or the writer emits files this loader refuses.
    #[test]
    fn a_clip_whose_derived_ramps_outrun_it_survives_a_round_trip() {
        let limits = limits();
        let big = limits.max_step.antennas * crate::speed::STEP_MARGIN * 4.0;
        let doc = ClipDoc {
            blend_in_ms: Some(0),
            blend_out_ms: Some(0),
            frames: vec![
                FrameDoc {
                    antennas: Some([0.0, 0.0]),
                    ..FrameDoc::default()
                },
                FrameDoc {
                    antennas: Some([big, 0.0]),
                    ..FrameDoc::default()
                },
            ],
            ..antennas_doc()
        };
        let clip = Clip::from_doc(doc, &limits).expect("in range");
        assert!(
            f64::from(clip.blend_in_ms()) > clip.duration_ms()
                && f64::from(clip.blend_out_ms()) > clip.duration_ms(),
            "both floors outrun the clip, or this proves nothing: {} / {} over {}",
            clip.blend_in_ms(),
            clip.blend_out_ms(),
            clip.duration_ms(),
        );

        let mut reloaded = Clip::from_doc(clip.to_doc(), &limits).expect("loads back");
        assert_eq!(reloaded.blend_in_ms(), clip.blend_in_ms());
        assert_eq!(reloaded.blend_out_ms(), clip.blend_out_ms());
        reloaded.forget_notes();
        let mut written = clip;
        written.forget_notes();
        assert_eq!(reloaded, written);
    }

    /// A track of `frames` frames the antennas hold still through: whatever the
    /// ceiling tests do to the blends, the derivation has nothing to say.
    fn still_doc(frames: usize) -> ClipDoc {
        ClipDoc {
            frames: (0..frames)
                .map(|_| FrameDoc {
                    antennas: Some([0.1, -0.1]),
                    ..FrameDoc::default()
                })
                .collect(),
            ..antennas_doc()
        }
    }

    #[test]
    fn an_authored_blend_in_longer_than_the_clip_is_refused() {
        // Fifty frames is a second of clip; the ramp is the hundredfold typo.
        let doc = ClipDoc {
            blend_in_ms: Some(100_000),
            blend_out_ms: Some(0),
            ..still_doc(50)
        };
        let err = Clip::from_doc(doc, &limits()).expect_err("longer than the clip");
        assert_eq!(
            err,
            ClipError::BlendExceedsClip {
                end: BlendEnd::In,
                blend_ms: 100_000,
                clip_ms: 1000.0,
            }
        );
        let message = err.to_string();
        assert!(message.contains("blend-in"), "{message}");
        assert!(message.contains("100000"), "{message}");
        assert!(message.contains("1000"), "{message}");

        // The boundary itself. Equality is lawful and one frame past it is not,
        // so this is the pair that says where the comparison sits — a typo a
        // hundredfold away says only that something rejects it.
        let doc = ClipDoc {
            blend_in_ms: Some(1020),
            blend_out_ms: Some(0),
            ..still_doc(50)
        };
        assert_eq!(
            Clip::from_doc(doc, &limits()).expect_err("one frame past the clip"),
            ClipError::BlendExceedsClip {
                end: BlendEnd::In,
                blend_ms: 1020,
                clip_ms: 1000.0,
            }
        );
    }

    #[test]
    fn an_authored_blend_out_longer_than_the_clip_is_refused() {
        let doc = ClipDoc {
            blend_in_ms: Some(0),
            blend_out_ms: Some(100_000),
            ..still_doc(50)
        };
        assert_eq!(
            Clip::from_doc(doc, &limits()).expect_err("longer than the clip"),
            ClipError::BlendExceedsClip {
                end: BlendEnd::Out,
                blend_ms: 100_000,
                clip_ms: 1000.0,
            }
        );
    }

    #[test]
    fn authored_blends_as_long_as_the_clip_are_accepted() {
        let doc = ClipDoc {
            blend_in_ms: Some(1000),
            blend_out_ms: Some(1000),
            ..still_doc(50)
        };
        let clip = Clip::from_doc(doc, &limits()).expect("equality is inside the ceiling");
        assert_eq!(clip.blend_in_ms(), 1000);
        assert_eq!(clip.blend_out_ms(), 1000);

        let doc = ClipDoc {
            blend_in_ms: Some(500),
            blend_out_ms: Some(500),
            ..still_doc(50)
        };
        let clip = Clip::from_doc(doc, &limits()).expect("below the ceiling");
        assert_eq!(clip.blend_in_ms(), 500);
        assert_eq!(clip.blend_out_ms(), 500);
    }

    /// A clip shorter than the default ramp, saying nothing about its blends:
    /// the default is the format's own number, so it is capped at the clip
    /// rather than held against an author who wrote nothing.
    #[test]
    fn omitted_blends_are_capped_at_the_clip_duration() {
        let doc = ClipDoc {
            blend_in_ms: None,
            blend_out_ms: None,
            ..still_doc(5)
        };
        let clip = Clip::from_doc(doc, &limits()).expect("a short clip still loads");
        assert!(clip.duration_ms() < f64::from(DEFAULT_BLEND_MS));
        // The clip's own length, not the number that happens to be its length
        // at this frame rate: the cap is a truncating cast, and a rate whose
        // frame is not a whole millisecond would silently cap short of the
        // clip while a literal went on passing.
        assert_eq!(f64::from(clip.blend_in_ms()), clip.duration_ms());
        assert_eq!(f64::from(clip.blend_out_ms()), clip.duration_ms());
        assert_eq!(clip.blend_in_ms(), 100);
    }

    /// A ceiling the author wrote *below* what the frames derive: the loader
    /// still plays under its own number, and the difference is a note rather
    /// than a silent substitution. The other direction is pinned in `vendor.rs`.
    #[test]
    fn a_conservative_stored_ceiling_is_replaced_and_noted() {
        let doc = ClipDoc {
            max_speed: 1.05,
            ..antennas_doc()
        };
        let clip = Clip::from_doc(doc, &limits()).expect("in range");
        assert!(
            clip.max_speed() > 1.05 + SPEED_CACHE_TOL,
            "the frames allow more than the document claimed: {}",
            clip.max_speed()
        );
        assert_eq!(
            clip.notes(),
            [ClipNote::MaxSpeedDiffers {
                stored: 1.05,
                derived: clip.max_speed(),
            }]
        );
    }
}
