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
//! A clip's base is a pose name or a per-channel numeric object; the keys a
//! numeric base carries are the clip's posed channels, and a name poses every
//! masked channel.
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
use serde::{Deserialize, Deserializer, Serialize};
use thiserror::Error;

use reachy_kin::neutral_head_pose;
use reachy_motion::FLOOR_TICK_HZ;
use reachy_motion::asset_name::{AssetNameError, check_asset_name};
use reachy_motion::joints::JointTargets;

use crate::envelope::{ClipLimits, FrameError, check_frames};

/// The document version this crate reads and writes.
///
/// A document carrying anything else is refused rather than read on a guess:
/// the fields a future version adds are exactly the ones whose absence a
/// permissive reader would silently take as a default.
pub const FORMAT_VERSION: u32 = 1;

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
/// authoritative, and `cogs/edge_caps_test` holds this one to it.
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

    /// The mask driving every channel there is.
    ///
    /// Here rather than folded together at each call site, for the reason
    /// [`Self::parse`] is here: the day a fourth channel is added, a hand-rolled
    /// union is still three channels wide and nothing says so.
    #[must_use]
    pub const fn all() -> Self {
        Self(PerChannel::new([true; Channel::COUNT]))
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

    /// The mask driving only the channels both of these do.
    #[must_use]
    pub fn intersection(self, other: Self) -> Self {
        let mut out = Self::empty();
        for channel in Channel::ALL {
            if self.contains(channel) && other.contains(channel) {
                out.insert(channel);
            }
        }
        out
    }

    /// Whether every channel this mask drives, `other` drives too.
    #[must_use]
    pub fn is_subset_of(self, other: Self) -> bool {
        self.iter().all(|channel| other.contains(channel))
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

    /// The mask a comma-separated list of channel names spells, as
    /// `head,antennas,body_yaw`.
    ///
    /// The spellings are [`Channel::as_str`]'s, so a list reads the way a
    /// document writes it. Here rather than in each tool that takes such a
    /// list: two command lines with their own parse are two tools that
    /// disagree about what `body-yaw` means, and the answer has to be the
    /// format's.
    ///
    /// # Errors
    ///
    /// [`MaskError`]: a word that is not a channel, a channel named twice, or
    /// a list naming nothing.
    pub fn parse(list: &str) -> Result<Self, MaskError> {
        // Checked ahead of the walk rather than after it: an empty list splits
        // into one empty word, and "" is not a channel is a worse answer to
        // `--channels ` than the one this arm gives.
        if list.trim().is_empty() {
            return Err(MaskError::Nothing);
        }
        let mut mask = Self::empty();
        for word in list.split(',') {
            let word = word.trim();
            let channel = Channel::ALL
                .into_iter()
                .find(|channel| channel.as_str() == word)
                .ok_or_else(|| MaskError::NotAChannel {
                    word: word.to_owned(),
                })?;
            if !mask.insert(channel) {
                return Err(MaskError::Twice { channel });
            }
        }
        Ok(mask)
    }
}

/// Why a channel list is not a mask.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum MaskError {
    /// A word in the list is not a channel's name.
    #[error("{word:?} is not a channel; head, antennas or body_yaw")]
    NotAChannel {
        /// The word as given.
        word: String,
    },
    /// A channel is named more than once.
    #[error("{} is named twice", channel.as_str())]
    Twice {
        /// The channel named twice.
        channel: Channel,
    },
    /// The list names no channel at all.
    #[error("a channel list names at least one channel")]
    Nothing,
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
        source: AssetNameError,
    },

    /// A numeric base poses a channel the clip does not drive.
    #[error("clip base poses {channel}, which is not in the mask")]
    BaseChannelUnmasked { channel: Channel },

    /// The authored base name is not a usable asset name.
    #[error("clip base name {name:?} is unusable: {source}")]
    BaseName {
        name: String,
        source: AssetNameError,
    },

    /// A numeric base with no keys: an overlay is spelled by omitting `base`.
    #[error("clip base poses no channel; omit `base` for an overlay")]
    BaseNoChannels,

    /// A base figure that is not a finite number.
    #[error("clip base key {key:?} is not finite")]
    BaseNonFinite { key: &'static str },

    /// A base head rotation too far from unit length to be one.
    #[error("clip base head rotation has norm {norm}, further than {QUAT_NORM_TOL} from unit")]
    BaseQuaternion { norm: f64 },

    /// The authored base is not among the poses supplied to the loader.
    #[error("clip base {name:?} is not a named pose")]
    UnknownBase { name: String },

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

    /// The frame track leaves the envelope over the neutral base, or asks an
    /// antenna for an angle with no goal count.
    #[error("a frame this machine cannot hold: {source}")]
    Frames {
        /// Which frame failed what.
        source: FrameError,
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

impl From<FrameError> for ClipError {
    fn from(source: FrameError) -> Self {
        Self::Frames { source }
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

/// What a clip's deltas are authored over: a pose name, or the numbers themselves.
///
/// Serialised untagged: the name as a bare string, the numbers as an object.
/// Deserialised by hand rather than untagged, so a malformed object reports
/// its own error — the misspelt key, the short array — instead of serde's
/// "did not match any variant".
#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(untagged)]
pub enum BaseDoc {
    /// A named pose, posing every channel the clip masks.
    Named(String),
    /// Numeric targets, posing exactly the channels they carry.
    Numeric(NumericBaseDoc),
}

impl<'de> Deserialize<'de> for BaseDoc {
    fn deserialize<D>(deserializer: D) -> Result<Self, D::Error>
    where
        D: Deserializer<'de>,
    {
        deserializer.deserialize_any(BaseDocVisitor)
    }
}

/// A string is a pose name; a map is a numeric base; anything else is refused.
struct BaseDocVisitor;

impl<'de> serde::de::Visitor<'de> for BaseDocVisitor {
    type Value = BaseDoc;

    fn expecting(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str("a pose name or a per-channel base object")
    }

    fn visit_str<E>(self, value: &str) -> Result<BaseDoc, E>
    where
        E: serde::de::Error,
    {
        Ok(BaseDoc::Named(value.to_owned()))
    }

    fn visit_string<E>(self, value: String) -> Result<BaseDoc, E>
    where
        E: serde::de::Error,
    {
        Ok(BaseDoc::Named(value))
    }

    fn visit_map<A>(self, map: A) -> Result<BaseDoc, A::Error>
    where
        A: serde::de::MapAccess<'de>,
    {
        NumericBaseDoc::deserialize(serde::de::value::MapAccessDeserializer::new(map))
            .map(BaseDoc::Numeric)
    }
}

/// The numeric base: one key per posed channel, in the pose document's
/// vocabulary. A key set to `null` is refused at parse: a channel is left
/// relative by omitting its key.
#[derive(Clone, Debug, Default, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct NumericBaseDoc {
    /// The head, relative to the neutral head pose.
    #[serde(
        default,
        deserialize_with = "present",
        skip_serializing_if = "Option::is_none"
    )]
    pub head: Option<HeadBaseDoc>,
    /// Body yaw, radians, absolute.
    #[serde(
        default,
        deserialize_with = "present",
        skip_serializing_if = "Option::is_none"
    )]
    pub body_yaw: Option<f64>,
    /// Antenna angles, right then left, radians, absolute.
    #[serde(
        default,
        deserialize_with = "present",
        skip_serializing_if = "Option::is_none"
    )]
    pub antennas: Option<[f64; 2]>,
}

/// Read a base key that is present. `null` is refused rather than read as
/// absent: a key that is there poses its channel, and a null one poses
/// nothing an author could mean — omitting the key is how a channel is
/// left relative.
fn present<'de, D, T>(deserializer: D) -> Result<Option<T>, D::Error>
where
    D: Deserializer<'de>,
    T: Deserialize<'de>,
{
    T::deserialize(deserializer).map(Some)
}

/// The head half of a numeric base, relative to the neutral head pose.
#[derive(Clone, Debug, PartialEq, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct HeadBaseDoc {
    /// Translation, metres, in the neutral head's frame.
    pub dt: [f64; 3],
    /// Rotation as a unit quaternion, `[w, x, y, z]`.
    pub dq: [f64; 4],
}

impl HeadBaseDoc {
    /// The head standing at the neutral head pose: no translation, identity
    /// rotation. In the base's own convention that is the vendor zero.
    pub const NEUTRAL: Self = Self {
        dt: [0.0, 0.0, 0.0],
        dq: [1.0, 0.0, 0.0, 0.0],
    };
}

/// A clip as written on disk.
///
/// Unknown keys are refused. The format is ours end to end — every writer of it
/// is this struct's own serialisation, the vendor importer and the probe
/// generator alike — so a key this reader does not know is a document from
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
    /// What the frames are authored over, if this is a posed clip: a pose
    /// name, posing every masked channel, or a numeric object whose keys are
    /// the posed channels. Absent for an overlay.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub base: Option<BaseDoc>,
    /// Free text, carried from the recording or written by the author.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// The mask: which channels this clip drives.
    pub channels: Vec<Channel>,
    /// Frame rate; the tick rate, or refused.
    pub frame_hz: f64,
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
    blend_in_ms: u32,
    blend_out_ms: u32,
    frames: Vec<DeltaFrame>,
    base: Option<ClipBase>,
    notes: Vec<ClipNote>,
}

/// How a base was written, for messages that name what a frame was checked
/// over.
#[derive(Clone, Debug, Eq, PartialEq)]
pub enum BaseLabel {
    /// A named pose.
    Named(String),
    /// A numeric base carried in the clip itself.
    Numeric,
}

impl fmt::Display for BaseLabel {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Named(name) => f.write_str(name),
            Self::Numeric => f.write_str("numeric"),
        }
    }
}

/// What a base was written as: the name it resolves through, or the
/// validated numeric document itself. `Clip::to_doc` writes this back
/// unchanged; `ClipBase::targets` is what it resolves to.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum BaseSource {
    Named(String),
    Numeric(NumericBaseDoc),
}

/// The validated base a clip's posed channels are composed toward.
#[derive(Clone, Debug, PartialEq)]
pub struct ClipBase {
    pub(crate) source: BaseSource,
    /// The posed channels: non-empty, and within the clip's mask.
    pub(crate) channels: ChannelMask,
    /// Absolute targets, meaningful on `channels`; `JointTargets::default()`
    /// placeholders on every other channel.
    pub(crate) targets: JointTargets,
}

impl ClipBase {
    /// How the base was written, for messages.
    #[must_use]
    pub fn label(&self) -> BaseLabel {
        match &self.source {
            BaseSource::Named(name) => BaseLabel::Named(name.clone()),
            BaseSource::Numeric(_) => BaseLabel::Numeric,
        }
    }

    /// The channels this base poses.
    #[must_use]
    pub fn channels(&self) -> ChannelMask {
        self.channels
    }

    /// The absolute targets baked into the runtime asset; read only on
    /// [`Self::channels`].
    #[must_use]
    pub fn targets(&self) -> JointTargets {
        self.targets
    }
}

/// Something a load changed about a clip, or found worth saying about it.
///
/// Not a refusal — the clip loaded — but not silent either: every one of these
/// means the asset on disk and the asset in memory differ, and an author
/// looking for why a motion is gentler than they wrote it needs the difference
/// reported rather than inferred.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum ClipNote {}

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

    /// Parse and validate a clip against named pose targets.
    pub fn from_json_resolved<F>(
        json: &str,
        limits: &ClipLimits,
        resolve: F,
    ) -> Result<Self, ClipError>
    where
        F: Fn(&str) -> Option<JointTargets>,
    {
        let doc: ClipDoc = serde_json::from_str(json).map_err(|err| ClipError::Malformed {
            detail: err.to_string(),
        })?;
        Self::from_doc_resolved(doc, limits, resolve)
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
        Self::from_doc_resolved(doc, limits, |_| None)
    }

    /// Validate a parsed document against named pose targets.
    pub fn from_doc_resolved<F>(
        doc: ClipDoc,
        limits: &ClipLimits,
        resolve: F,
    ) -> Result<Self, ClipError>
    where
        F: Fn(&str) -> Option<JointTargets>,
    {
        if doc.version != FORMAT_VERSION {
            return Err(ClipError::UnsupportedVersion {
                version: doc.version,
            });
        }
        if doc.kind != CLIP_KIND {
            return Err(ClipError::WrongKind { kind: doc.kind });
        }
        check_asset_name(&doc.name).map_err(|source| ClipError::Name {
            name: doc.name.clone(),
            source,
        })?;
        if doc.frame_hz != FLOOR_TICK_HZ {
            return Err(ClipError::FrameRate {
                frame_hz: doc.frame_hz,
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

        let base = match doc.base {
            None => None,
            Some(BaseDoc::Named(name)) => {
                check_asset_name(&name).map_err(|source| ClipError::BaseName {
                    name: name.clone(),
                    source,
                })?;
                let targets =
                    resolve(&name).ok_or_else(|| ClipError::UnknownBase { name: name.clone() })?;
                Some(ClipBase {
                    source: BaseSource::Named(name),
                    channels: mask,
                    targets,
                })
            }
            Some(BaseDoc::Numeric(numeric)) => Some(numeric_base(&numeric, mask)?),
        };
        let mut frames = Vec::with_capacity(doc.frames.len());
        for (index, frame) in doc.frames.iter().enumerate() {
            frames.push(delta_frame(index, frame, mask)?);
        }

        check_frames(&frames, base.as_ref(), limits)?;

        let clip_ms = clip_duration_ms(frames.len());
        let blend_in_ms = authored_blend(doc.blend_in_ms, BlendEnd::In, clip_ms)?;
        let blend_out_ms = authored_blend(doc.blend_out_ms, BlendEnd::Out, clip_ms)?;

        Ok(Self {
            name: doc.name,
            description: doc.description,
            mask,
            blend_in_ms,
            blend_out_ms,
            frames,
            base,
            notes: Vec::new(),
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

    /// The validated base, if this clip poses any channel.
    #[must_use]
    pub fn base(&self) -> Option<&ClipBase> {
        self.base.as_ref()
    }

    /// The channels composed toward the base rather than added to whatever
    /// stands; empty for an overlay.
    #[must_use]
    pub fn posed_channels(&self) -> ChannelMask {
        self.base
            .as_ref()
            .map_or(ChannelMask::empty(), |base| base.channels)
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
    /// one: it states no ramps at all, so what the loader settles on is what
    /// gets written, and reporting the loader's own defaults back would be the
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
    /// Every ramp a clip carries is within its own length — the loader caps an
    /// unstated one there and refuses an authored one past it — so each is
    /// written as it stands.
    #[must_use]
    pub fn to_doc(&self) -> ClipDoc {
        ClipDoc {
            version: FORMAT_VERSION,
            kind: CLIP_KIND.to_owned(),
            name: self.name.clone(),
            base: self.base.as_ref().map(|base| match &base.source {
                BaseSource::Named(name) => BaseDoc::Named(name.clone()),
                BaseSource::Numeric(doc) => BaseDoc::Numeric(doc.clone()),
            }),
            description: self.description.clone(),
            channels: self.mask.iter().collect(),
            frame_hz: FLOOR_TICK_HZ,
            blend_in_ms: Some(self.blend_in_ms),
            blend_out_ms: Some(self.blend_out_ms),
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

/// Which head key a rigid-delta refusal is about.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum HeadKey {
    Dt,
    Dq,
}

impl HeadKey {
    /// The key as a frame writes it.
    fn frame_key(self) -> &'static str {
        match self {
            Self::Dt => "dt",
            Self::Dq => "dq",
        }
    }

    /// The key as a numeric base writes it.
    fn base_key(self) -> &'static str {
        match self {
            Self::Dt => "head.dt",
            Self::Dq => "head.dq",
        }
    }
}

/// Why `dt`/`dq` did not make a rigid delta; the caller names the frame or the base.
#[derive(Clone, Copy, Debug, PartialEq)]
enum RigidError {
    NonFinite { key: HeadKey, value: f64 },
    Quaternion { norm: f64 },
}

/// Convert `dt`/`dq` — metres and `[w, x, y, z]` — into a rigid delta.
///
/// The quaternion is checked against unit length and then normalised: JSON's
/// decimal round-trip leaves a rotation a few ulps off unit, which is a
/// renormalisation, while anything past the tolerance is a number that was
/// never a rotation.
fn rigid(dt: [f64; 3], dq: [f64; 4]) -> Result<Isometry3<f64>, RigidError> {
    if let Some(value) = dt.into_iter().find(|value| !value.is_finite()) {
        return Err(RigidError::NonFinite {
            key: HeadKey::Dt,
            value,
        });
    }
    if let Some(value) = dq.into_iter().find(|value| !value.is_finite()) {
        return Err(RigidError::NonFinite {
            key: HeadKey::Dq,
            value,
        });
    }
    let quaternion = Quaternion::new(dq[0], dq[1], dq[2], dq[3]);
    let norm = quaternion.norm();
    if (norm - 1.0).abs() > QUAT_NORM_TOL {
        return Err(RigidError::Quaternion { norm });
    }
    Ok(Isometry3::from_parts(
        Translation3::new(dt[0], dt[1], dt[2]),
        UnitQuaternion::from_quaternion(quaternion),
    ))
}

/// One frame's head keys as a rigid delta.
fn head_delta(index: usize, dt: [f64; 3], dq: [f64; 4]) -> Result<Isometry3<f64>, ClipError> {
    rigid(dt, dq).map_err(|error| match error {
        RigidError::NonFinite { key, value } => ClipError::NonFinite {
            frame: index,
            key: key.frame_key(),
            value,
        },
        RigidError::Quaternion { norm } => ClipError::Quaternion { frame: index, norm },
    })
}

/// Validate a numeric base against the clip's mask and build its targets.
///
/// The keys present are the posed channels. The head is the pose document's
/// convention — relative to the neutral head pose — composed onto that pose;
/// its quaternion gets the frame rule: refused past [`QUAT_NORM_TOL`] from
/// unit, normalised within it.
fn numeric_base(doc: &NumericBaseDoc, mask: ChannelMask) -> Result<ClipBase, ClipError> {
    let mut channels = ChannelMask::empty();
    if doc.head.is_some() {
        channels.insert(Channel::Head);
    }
    if doc.body_yaw.is_some() {
        channels.insert(Channel::BodyYaw);
    }
    if doc.antennas.is_some() {
        channels.insert(Channel::Antennas);
    }
    if channels.is_empty() {
        return Err(ClipError::BaseNoChannels);
    }
    if let Some(channel) = channels.iter().find(|channel| !mask.contains(*channel)) {
        return Err(ClipError::BaseChannelUnmasked { channel });
    }

    let mut targets = JointTargets::default();
    if let Some(head) = &doc.head {
        let relative = rigid(head.dt, head.dq).map_err(|error| match error {
            RigidError::NonFinite { key, .. } => ClipError::BaseNonFinite {
                key: key.base_key(),
            },
            RigidError::Quaternion { norm } => ClipError::BaseQuaternion { norm },
        })?;
        targets.head_pose_body = neutral_head_pose() * relative;
    }
    if let Some(body_yaw) = doc.body_yaw {
        if !body_yaw.is_finite() {
            return Err(ClipError::BaseNonFinite { key: "body_yaw" });
        }
        targets.body_yaw = body_yaw;
    }
    if let Some(antennas) = doc.antennas {
        if !antennas.iter().all(|value| value.is_finite()) {
            return Err(ClipError::BaseNonFinite { key: "antennas" });
        }
        targets.antennas = antennas;
    }
    Ok(ClipBase {
        source: BaseSource::Numeric(doc.clone()),
        channels,
        targets,
    })
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
            base: None,
            description: Some("a test".to_owned()),
            channels: vec![Channel::Head, Channel::Antennas, Channel::BodyYaw],
            frame_hz: FLOOR_TICK_HZ,
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
        // One frame is 20 ms of clip, so the omitted default is capped there.
        assert_eq!(clip.blend_in_ms(), 20);
        assert_eq!(clip.blend_out_ms(), 20);
    }

    #[test]
    fn posed_document_round_trips_and_refuses_bad_base_names() {
        let mut doc = full_doc();
        doc.base = Some(BaseDoc::Named("neutral".to_owned()));
        let anchor = JointTargets::default();
        let clip = Clip::from_doc_resolved(doc.clone(), &limits(), |_| Some(anchor))
            .expect("posed document loads");
        let json = serde_json::to_string(&clip.to_doc()).expect("clip serializes");
        assert!(json.contains(r#""base":"neutral""#), "{json}");
        let round_trip = Clip::from_json_resolved(&json, &limits(), |_| Some(anchor))
            .expect("posed JSON round-trips");
        let base = round_trip.base().expect("base");
        assert_eq!(base.label(), BaseLabel::Named("neutral".to_owned()));
        assert_eq!(round_trip.posed_channels(), round_trip.mask());
        assert_eq!(round_trip.frames(), clip.frames());

        let mut unknown = doc.clone();
        unknown.base = Some(BaseDoc::Named("missing".to_owned()));
        assert!(matches!(
            Clip::from_doc_resolved(unknown, &limits(), |_| None),
            Err(ClipError::UnknownBase { name }) if name == "missing"
        ));
        let mut malformed = doc;
        malformed.base = Some(BaseDoc::Named("bad name".to_owned()));
        assert!(matches!(
            Clip::from_doc_resolved(malformed, &limits(), |_| Some(anchor)),
            Err(ClipError::BaseName { name, .. }) if name == "bad name"
        ));
    }

    #[test]
    fn unposed_document_preserves_frame_and_composition_bits() {
        let clip = Clip::from_doc(full_doc(), &limits()).expect("unposed document loads");
        let again = Clip::from_json(&serde_json::to_string(&clip.to_doc()).unwrap(), &limits())
            .expect("JSON round-trips");
        assert_eq!(clip, again);
        assert_eq!(clip.base(), None);
        assert_eq!(clip.posed_channels(), ChannelMask::empty());
    }

    #[test]
    fn posed_load_screens_absolute_targets_and_names_failures() {
        let yaw_doc = ClipDoc {
            channels: vec![Channel::BodyYaw],
            frames: vec![FrameDoc {
                body_yaw: Some(0.3),
                ..FrameDoc::default()
            }],
            ..full_doc()
        };
        assert!(Clip::from_doc(yaw_doc.clone(), &limits()).is_ok());
        let error = Clip::from_doc_resolved(
            ClipDoc {
                base: Some(BaseDoc::Named("wide".to_owned())),
                ..yaw_doc
            },
            &limits(),
            |_| {
                Some(JointTargets {
                    body_yaw: 2.8,
                    ..JointTargets::default()
                })
            },
        )
        .unwrap_err();
        match error {
            ClipError::Frames {
                source:
                    FrameError::AnchoredEnvelope {
                        frame,
                        base,
                        violations,
                    },
            } => {
                assert_eq!(frame, 0);
                assert_eq!(base, BaseLabel::Named("wide".to_owned()));
                assert!(violations.body_yaw);
                assert!(!violations.cone);
            }
            other => panic!("unexpected error: {other:?}"),
        }

        let antenna_doc = ClipDoc {
            channels: vec![Channel::Antennas],
            frames: vec![FrameDoc {
                antennas: Some([0.01, 0.0]),
                ..FrameDoc::default()
            }],
            ..full_doc()
        };
        let error = Clip::from_doc_resolved(
            ClipDoc {
                base: Some(BaseDoc::Named("antenna-base".to_owned())),
                ..antenna_doc
            },
            &limits(),
            |_| {
                Some(JointTargets {
                    antennas: [reachy_motion::ANTENNA_GOAL_MAX_RAD, 0.0],
                    ..JointTargets::default()
                })
            },
        )
        .unwrap_err();
        assert!(matches!(
            error,
            ClipError::Frames {
                source: FrameError::AnchoredAntennaGoal { frame: 0, base, side: 0, .. }
            } if base == BaseLabel::Named("antenna-base".to_owned())
        ));

        let head_doc = ClipDoc {
            base: Some(BaseDoc::Named("near-edge".to_owned())),
            channels: vec![Channel::Head],
            frames: vec![FrameDoc {
                dt: Some([0.0, 0.0, 0.2]),
                dq: Some([1.0, 0.0, 0.0, 0.0]),
                ..FrameDoc::default()
            }],
            ..full_doc()
        };
        let error = Clip::from_doc_resolved(head_doc, &limits(), |_| {
            Some(JointTargets {
                head_pose_body: reachy_kin::geometry::rest_head_pose(),
                ..JointTargets::default()
            })
        })
        .unwrap_err();
        assert!(matches!(
            error,
            ClipError::Frames {
                source: FrameError::AnchoredEnvelope { frame: 0, base, violations }
            } if base == BaseLabel::Named("near-edge".to_owned()) && violations.any()
        ));
    }

    /// A numeric head base at neutral lifted by `dz` metres.
    fn lifted_head_base(dz: f64) -> HeadBaseDoc {
        HeadBaseDoc {
            dt: [0.0, 0.0, dz],
            dq: [1.0, 0.0, 0.0, 0.0],
        }
    }

    #[test]
    fn numeric_base_round_trips_with_exactly_its_posed_channels() {
        let base = BaseDoc::Numeric(NumericBaseDoc {
            head: Some(lifted_head_base(0.01)),
            body_yaw: None,
            antennas: Some([0.1, -0.1]),
        });
        let doc = ClipDoc {
            base: Some(base),
            ..full_doc()
        };
        let clip = Clip::from_doc(doc, &limits()).expect("numeric base loads");
        let mut posed = ChannelMask::of(Channel::Head);
        posed.insert(Channel::Antennas);
        assert_eq!(clip.posed_channels(), posed);
        let loaded = clip.base().expect("posed");
        assert_eq!(loaded.label(), BaseLabel::Numeric);
        assert!(
            (loaded.targets().head_pose_body.translation.vector.z
                - (neutral_head_pose().translation.vector.z + 0.01))
                .abs()
                < 1e-12
        );
        assert_eq!(loaded.targets().antennas, [0.1, -0.1]);

        let written = clip.to_doc();
        assert_eq!(
            written.base,
            Some(BaseDoc::Numeric(NumericBaseDoc {
                head: Some(lifted_head_base(0.01)),
                body_yaw: None,
                antennas: Some([0.1, -0.1]),
            }))
        );
        let json = serde_json::to_string(&written).expect("clip serializes");
        let value: serde_json::Value = serde_json::from_str(&json).expect("JSON parses");
        let mut keys: Vec<&str> = value["base"]
            .as_object()
            .expect("a numeric base is an object")
            .keys()
            .map(String::as_str)
            .collect();
        keys.sort_unstable();
        assert_eq!(keys, ["antennas", "head"], "{json}");
        assert_eq!(
            Clip::from_json(&json, &limits()).expect("JSON round-trips"),
            clip
        );
    }

    #[test]
    fn numeric_base_refusals() {
        let numeric = |base: NumericBaseDoc, doc: ClipDoc| {
            Clip::from_doc(
                ClipDoc {
                    base: Some(BaseDoc::Numeric(base)),
                    ..doc
                },
                &limits(),
            )
        };
        assert_eq!(
            numeric(
                NumericBaseDoc {
                    head: Some(HeadBaseDoc::NEUTRAL),
                    ..NumericBaseDoc::default()
                },
                antennas_doc()
            ),
            Err(ClipError::BaseChannelUnmasked {
                channel: Channel::Head
            })
        );
        assert_eq!(
            numeric(NumericBaseDoc::default(), full_doc()),
            Err(ClipError::BaseNoChannels)
        );
        assert_eq!(
            numeric(
                NumericBaseDoc {
                    body_yaw: Some(f64::NAN),
                    ..NumericBaseDoc::default()
                },
                full_doc()
            ),
            Err(ClipError::BaseNonFinite { key: "body_yaw" })
        );
        match numeric(
            NumericBaseDoc {
                head: Some(HeadBaseDoc {
                    dt: [0.0, 0.0, 0.0],
                    dq: [1.1, 0.0, 0.0, 0.0],
                }),
                ..NumericBaseDoc::default()
            },
            full_doc(),
        ) {
            Err(ClipError::BaseQuaternion { norm }) => assert!((norm - 1.1).abs() < 1e-12),
            other => panic!("unexpected: {other:?}"),
        }
        assert_eq!(
            numeric(
                NumericBaseDoc {
                    head: Some(HeadBaseDoc {
                        dt: [0.0, f64::INFINITY, 0.0],
                        dq: [1.0, 0.0, 0.0, 0.0],
                    }),
                    ..NumericBaseDoc::default()
                },
                full_doc()
            ),
            Err(ClipError::BaseNonFinite { key: "head.dt" })
        );
        assert_eq!(
            numeric(
                NumericBaseDoc {
                    antennas: Some([0.0, f64::NAN]),
                    ..NumericBaseDoc::default()
                },
                full_doc()
            ),
            Err(ClipError::BaseNonFinite { key: "antennas" })
        );
        // Finiteness is checked before the norm, so a NaN component is named
        // as non-finite rather than as a bad quaternion.
        assert_eq!(
            numeric(
                NumericBaseDoc {
                    head: Some(HeadBaseDoc {
                        dt: [0.0; 3],
                        dq: [f64::NAN, 0.0, 0.0, 0.0],
                    }),
                    ..NumericBaseDoc::default()
                },
                full_doc()
            ),
            Err(ClipError::BaseNonFinite { key: "head.dq" })
        );
    }

    #[test]
    fn a_null_base_key_is_refused_not_read_as_absent() {
        let mut value = serde_json::to_value(full_doc()).expect("document serializes");
        value["channels"] = serde_json::json!(["head", "antennas"]);
        value["frames"] = serde_json::json!([
            {"dt": [0.0, 0.0, 0.01], "dq": [1.0, 0.0, 0.0, 0.0], "antennas": [0.1, -0.1]}
        ]);
        for base in [
            serde_json::json!({"head": null, "antennas": [0.0, 0.0]}),
            serde_json::json!({"body_yaw": null}),
        ] {
            value["base"] = base.clone();
            match Clip::from_json(&value.to_string(), &limits()) {
                Err(ClipError::Malformed { detail }) => {
                    assert!(detail.contains("null"), "{base}: {detail}");
                }
                other => panic!("{base}: unexpected: {other:?}"),
            }
        }
    }

    #[test]
    fn a_numeric_base_object_reports_its_own_parse_error() {
        let document = |base: &str| {
            format!(
                r#"{{"version": 1, "kind": "clip", "name": "misspelt", "base": {base},
                     "channels": ["antennas"], "frame_hz": {FLOOR_TICK_HZ},
                     "frames": [{{"antennas": [0.0, 0.0]}}]}}"#
            )
        };
        match Clip::from_json(&document(r#"{"antenas": [0.0, 0.0]}"#), &limits()) {
            Err(ClipError::Malformed { detail }) => {
                assert!(detail.contains("antenas"), "{detail}");
                assert!(!detail.contains("untagged"), "{detail}");
            }
            other => panic!("unexpected: {other:?}"),
        }
        match Clip::from_json(&document("3"), &limits()) {
            Err(ClipError::Malformed { detail }) => {
                assert!(detail.contains("pose name"), "{detail}");
            }
            other => panic!("unexpected: {other:?}"),
        }
    }

    #[test]
    fn a_numeric_base_head_is_screened_by_the_frame_walk() {
        let doc = ClipDoc {
            base: Some(BaseDoc::Numeric(NumericBaseDoc {
                head: Some(lifted_head_base(0.2)),
                ..NumericBaseDoc::default()
            })),
            channels: vec![Channel::Head],
            frames: vec![FrameDoc {
                dt: Some([0.0, 0.0, 0.0]),
                dq: Some([1.0, 0.0, 0.0, 0.0]),
                ..FrameDoc::default()
            }],
            ..full_doc()
        };
        assert!(matches!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::Frames {
                source: FrameError::AnchoredEnvelope {
                    frame: 0,
                    base: BaseLabel::Numeric,
                    ..
                }
            })
        ));
    }

    /// The two whole-frame checks a consumer of an unvalidated source makes
    /// before it hands frames to a player, over a frame that drives everything.
    #[test]
    fn a_frames_numbers_and_rotation_are_checked_whole() {
        let mask = ChannelMask::all();
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
    fn a_base_head_and_a_frame_head_share_the_quaternion_rule() {
        let with_base_dq = |dq: [f64; 4]| ClipDoc {
            base: Some(BaseDoc::Numeric(NumericBaseDoc {
                head: Some(HeadBaseDoc {
                    dt: [0.0, 0.0, 0.0],
                    dq,
                }),
                ..NumericBaseDoc::default()
            })),
            ..full_doc()
        };
        let with_frame_dq = |dq: [f64; 4]| {
            let mut doc = full_doc();
            doc.frames[0].dq = Some(dq);
            doc
        };

        let refused = [1.0 + 2.0 * QUAT_NORM_TOL, 0.0, 0.0, 0.0];
        assert!(matches!(
            Clip::from_doc(with_base_dq(refused), &limits()),
            Err(ClipError::BaseQuaternion { .. })
        ));
        assert!(matches!(
            Clip::from_doc(with_frame_dq(refused), &limits()),
            Err(ClipError::Quaternion { frame: 0, .. })
        ));

        let renormalised = [1.0 + QUAT_NORM_TOL / 2.0, 0.0, 0.0, 0.0];
        let base_clip =
            Clip::from_doc(with_base_dq(renormalised), &limits()).expect("base within tolerance");
        let base_rotation = base_clip
            .base()
            .expect("posed")
            .targets()
            .head_pose_body
            .rotation;
        assert!((base_rotation.quaternion().norm() - 1.0).abs() < 1e-15);
        let frame_clip =
            Clip::from_doc(with_frame_dq(renormalised), &limits()).expect("frame within tolerance");
        let frame_rotation = frame_clip.frames()[0]
            .head
            .expect("head is masked")
            .rotation;
        assert!((frame_rotation.quaternion().norm() - 1.0).abs() < 1e-15);
    }

    #[test]
    fn the_neutral_head_base_loads_to_the_neutral_head_pose() {
        let doc = ClipDoc {
            base: Some(BaseDoc::Numeric(NumericBaseDoc {
                head: Some(HeadBaseDoc::NEUTRAL),
                ..NumericBaseDoc::default()
            })),
            ..full_doc()
        };
        let clip = Clip::from_doc(doc, &limits()).expect("neutral base loads");
        assert_eq!(
            clip.base().expect("posed").targets().head_pose_body,
            neutral_head_pose()
        );
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
    fn bad_name_is_refused_by_the_clip_loader() {
        let doc = ClipDoc {
            name: "Loving1".to_owned(),
            ..full_doc()
        };
        assert_eq!(
            Clip::from_doc(doc, &limits()),
            Err(ClipError::Name {
                name: "Loving1".to_owned(),
                source: AssetNameError::BadChar { ch: 'L' },
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
    fn unknown_keys_are_refused() {
        let json = r#"{
            "version": 1, "kind": "clip", "name": "pod/x",
            "channels": ["antennas"], "frame_hz": 50.0,
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
    fn mask_intersection_and_subset() {
        let left = ChannelMask::of(Channel::Head).union(ChannelMask::of(Channel::Antennas));
        let right = ChannelMask::of(Channel::Antennas).union(ChannelMask::of(Channel::BodyYaw));
        assert_eq!(left.intersection(right), ChannelMask::of(Channel::Antennas));
        assert_eq!(
            left.intersection(ChannelMask::empty()),
            ChannelMask::empty()
        );
        assert_eq!(left.intersection(ChannelMask::all()), left);
        assert!(ChannelMask::of(Channel::Antennas).is_subset_of(left));
        assert!(left.is_subset_of(left));
        assert!(ChannelMask::empty().is_subset_of(ChannelMask::empty()));
        assert!(!left.is_subset_of(right));
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

    /// The list spelling every tool that takes one reads: the format's own
    /// words, in any order, and each of the three ways a list is not a mask.
    #[test]
    fn a_channel_list_parses_to_the_mask_it_names() {
        let parsed = ChannelMask::parse("antennas, head").expect("two channels");
        assert_eq!(
            parsed.iter().collect::<Vec<_>>(),
            vec![Channel::Head, Channel::Antennas],
            "document order, whatever order the list was in"
        );
        assert_eq!(
            ChannelMask::parse("body_yaw"),
            Ok(ChannelMask::of(Channel::BodyYaw))
        );
        assert_eq!(
            ChannelMask::parse("legs"),
            Err(MaskError::NotAChannel {
                word: "legs".to_owned()
            })
        );
        assert_eq!(
            ChannelMask::parse("head,head"),
            Err(MaskError::Twice {
                channel: Channel::Head
            })
        );
        assert_eq!(ChannelMask::parse(""), Err(MaskError::Nothing));
    }

    #[test]
    fn mask_insert_reports_a_fresh_channel() {
        let mut mask = ChannelMask::empty();
        assert!(mask.insert(Channel::Head), "first insert is the event");
        assert!(!mask.insert(Channel::Head), "the second is not");
    }

    /// A track of `frames` frames the antennas hold still through.
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

    /// `max_speed` is not a key of this format: a document stating one is
    /// refused as unknown rather than having the number silently ignored.
    #[test]
    fn a_document_stating_a_speed_ceiling_is_refused() {
        let json = r#"{
            "version": 1, "kind": "clip", "name": "pod/x",
            "channels": ["antennas"], "frame_hz": 50.0, "max_speed": 1.0,
            "frames": [{"antennas": [0.0, 0.0]}]
        }"#;
        let err = Clip::from_json(json, &limits()).expect_err("max_speed is not a key");
        match err {
            ClipError::Malformed { detail } => {
                assert!(detail.contains("max_speed"), "{detail}");
            }
            other => panic!("{other:?}"),
        }
    }
}
