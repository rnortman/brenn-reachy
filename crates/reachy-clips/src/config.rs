//! The clip library as the running system is handed it: a borrowed view of a
//! configuration message.
//!
//! This is the one storage of a playable track. A clip arrives as a read-only
//! [`ClipLibraryConfig`] — the validated form of the message a host is handed
//! for the duration of one call, which the host reaches through one
//! `validate()` at its own boundary — and nothing in that message may be copied
//! out: a host whose per-execution memory is a fixed-layout slot has nowhere to
//! put it. So a [`ClipView`] borrows the message and answers every question a
//! [`ClipPlayer`](crate::player::ClipPlayer) asks straight out of its bytes.
//!
//! It is also the one place the mapping between a loaded [`Clip`] and the
//! config schema is written, in both directions: [`write_clip`] is what the
//! emitter uses to turn a loaded clip into a message, and [`ClipView`] is what
//! plays one back. A second statement of the mapping would be a second answer
//! able to disagree. [`write_library`] carries the loaded
//! [`Motion`](crate::library::Motion)s across the same way, flattened at load
//! so that what crosses is a list of segments and never a reference graph. A
//! [`MotionView`] plays one back: it borrows that segment list and derives what
//! playing it is like — the channels it drives, how long it runs, how fast it
//! may be invoked, how it blends — from the clips the segments name, because a
//! motion carries the walk and nothing about it.
//!
//! **Every check the loader makes, this makes.** A `Clip` holds its invariants
//! by construction and a message holds whatever is in the file, so the view
//! establishes them at construction — frames present, mask known, sampled on
//! this system's grid, numbers finite, rotations unit, nothing driven that the
//! mask does not claim — and the player re-checks nothing per tick. What the
//! generated validation establishes first is a different set: the frame count
//! is a count the array can hold, and the bytes are the shape the schema
//! declares. Neither covers the other, and a float is a thing validation never
//! inspects.
//!
//! Nothing here is a safety gate: a clip this accepts still meets the envelope
//! check and the step bound on its composed result, every tick.

use crate::format::{
    Channel, ChannelMask, Clip, DeltaFrame, MAX_SPEED, MIN_SPEED, QUAT_NORM_TOL, clip_duration_ms,
};
use crate::library::{Motion, Segment, speed_in_bounds, within_ceiling};
use brenn_reachy__cogs__config_clk_rs::{
    ClipConfig, ClipFrame, ClipLibraryConfig, MotionConfig, MotionSegment,
};
use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointFlagsWire};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use reachy_motion::FLOOR_TICK_HZ;
use reachy_motion::joints::{JointGroup, flags};
use thiserror::Error;

/// How many clips the library message holds. The schema's own capacity, checked
/// against it below rather than restated: a bound stated twice is a bound that
/// can drift.
pub const MAX_CLIPS: usize = 16;

/// How many frames one clip may carry.
pub const MAX_CLIP_FRAMES: usize = 512;

/// How many motions the library message holds.
pub const MAX_MOTIONS: usize = 32;

/// How many segments one motion may carry.
pub const MAX_SEGMENTS: usize = 32;

/// A configured clip that cannot be played as it stands.
///
/// Every variant is a fact about an asset rather than about a run: the message
/// says something the player's invariants do not allow. Refusing is the whole
/// point — a clip is presence, never safety, so nothing is gained by playing
/// half of a broken one. What the emit direction refuses on is
/// [`LibraryWriteError`], so this type stays the list of what a configuration
/// already on the box can be wrong about.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum ClipViewError {
    /// The mask names bits that are not a whole channel's joints.
    ///
    /// A channel is a group of joints — the head is six cranks, the antennas are
    /// a pair — so only unions of whole groups spell a channel mask. Anything
    /// else is a mask written against a different convention.
    #[error("mask bits {bits:#x} are not a union of whole channel groups")]
    UnknownMaskBits {
        /// What the field held.
        bits: u16,
    },

    /// The mask is empty: a clip that drives nothing.
    #[error("clip drives no channels")]
    NoChannels,

    /// The frame track is empty.
    #[error("clip has no frames")]
    NoFrames,

    /// The clip was sampled on another grid.
    #[error("clip is sampled at {frame_rate_hz} Hz; clips are played at {FLOOR_TICK_HZ} Hz")]
    FrameRate {
        /// What the field held.
        frame_rate_hz: f64,
    },

    /// A frame carries a value that is not a finite number.
    #[error("frame {frame} field {key} is {value}, which is not a finite number")]
    NonFinite {
        /// Which frame.
        frame: usize,
        /// Which field of it.
        key: &'static str,
        /// What it held.
        value: f64,
    },

    /// A frame's head rotation is not a unit quaternion.
    #[error("frame {frame} head rotation has norm {norm}, which is not 1")]
    Quaternion {
        /// Which frame.
        frame: usize,
        /// The norm found.
        norm: f64,
    },

    /// A frame drives a channel the mask does not name.
    ///
    /// An unmasked channel's fields are zero, which is the neutral delta. A
    /// non-zero one is an asset wrong about itself, and playing it would move a
    /// joint no consumer of the mask expects to move.
    #[error("frame {frame} field {key} is {value}, but the mask does not name {channel}")]
    StrayChannel {
        /// Which frame.
        frame: usize,
        /// Which field of it.
        key: &'static str,
        /// The channel that field belongs to.
        channel: Channel,
        /// What it held.
        value: f64,
    },

    /// The clip's speed ceiling is not a rate anything can be played at.
    ///
    /// A derived number: the load takes the tightest frame pair against the
    /// machine's step bounds, so a clip that loaded has a positive one.
    #[error("clip speed ceiling {max_speed} is not finite and positive")]
    SpeedCeiling {
        /// What the field held.
        max_speed: f64,
    },

    /// A script named a clip the library does not have.
    #[error("clip id {clip_id} is not one of the library's {clips} clips")]
    UnknownClip {
        /// The id asked for.
        clip_id: usize,
        /// How many clips there are.
        clips: usize,
    },
}

/// Why a library will not be written into the message.
///
/// The emit direction's refusals, kept apart from [`ClipViewError`] because they
/// are facts about a table being assembled on a host and not about a
/// configuration a box was handed: nothing here can reach a player, and a tick
/// path asking "which refusals can reach me?" reads the other type.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum LibraryWriteError {
    /// More clips than the message holds.
    #[error("library has {clips} clips; the message holds {MAX_CLIPS}")]
    TooManyClips {
        /// How many the library has.
        clips: usize,
    },

    /// More frames than the message holds.
    #[error("clip has {frames} frames; the library message holds {MAX_CLIP_FRAMES}")]
    TooManyFrames {
        /// How many the clip has.
        frames: usize,
    },

    /// More motions than the message holds.
    #[error("library has {motions} motions; the message holds {MAX_MOTIONS}")]
    TooManyMotions {
        /// How many the library has.
        motions: usize,
    },

    /// A motion flattened to more segments than the message holds.
    #[error("motion {motion} has {segments} segments; the message holds {MAX_SEGMENTS}")]
    TooManySegments {
        /// Which motion.
        motion: usize,
        /// How many segments it flattened to.
        segments: usize,
    },

    /// A segment naming a clip that is not among the clips being written.
    ///
    /// A segment's clip is resolved by name against the very clips the message
    /// carries, so this is the two halves of one emit disagreeing about what
    /// the library holds — never something a document can say.
    #[error("segment {segment} of motion {motion} names a clip the library does not carry")]
    SegmentClipMissing {
        /// Which motion.
        motion: usize,
        /// Which of its segments.
        segment: usize,
    },

    /// A hold the message's millisecond field cannot hold.
    ///
    /// Only a gap that is not a finite non-negative number of seconds, or one
    /// longer than roughly forty-nine days, reaches this: a gap that merely
    /// falls between two milliseconds is rounded to the nearer one.
    #[error("motion {motion} holds a gap of {gap_s} s, which is not a duration the message holds")]
    GapOutOfRange {
        /// Which motion.
        motion: usize,
        /// The gap, seconds.
        gap_s: f64,
    },
}

/// Which joints a channel drives.
///
/// The one crossing between the clip crate's three channels and the motion
/// crate's nine joints. It is a bijection onto the joint groups by construction
/// — a clip's channels and the machine's joint groups are the same partition of
/// the same machine — and the conformance test below is what holds it that way.
#[must_use]
pub fn channel_group(channel: Channel) -> JointGroup {
    match channel {
        Channel::Head => JointGroup::Legs,
        Channel::BodyYaw => JointGroup::BodyYaw,
        Channel::Antennas => JointGroup::Antennas,
    }
}

/// A channel mask as joint-set bits: the union of the joints of every channel it
/// names.
///
/// Nine bits, one per bus row, which is the config field's convention and *not*
/// the clip crate's own packed three-bit channel form. The two are different
/// encodings for different purposes, so they do not share a name.
#[must_use]
pub fn joint_mask_bits(mask: ChannelMask) -> u16 {
    let set = mask.iter().fold(JointFlags::NONE, |set, channel| {
        set | channel_group(channel).joints()
    });
    JointFlagsWire::from(set).0
}

/// Joint-set bits back to a channel mask, refusing anything that is not a union
/// of whole channel groups.
pub fn mask_from_bits(bits: u16) -> Result<ChannelMask, ClipViewError> {
    let unknown = ClipViewError::UnknownMaskBits { bits };
    let set = JointFlagsWire(bits).to_known().ok_or(unknown)?;
    let mut mask = ChannelMask::empty();
    let mut covered = JointFlags::NONE;
    for channel in Channel::ALL {
        let joints = channel_group(channel).joints();
        if flags::covers(set, channel_group(channel)) {
            mask.insert(channel);
            covered |= joints;
        }
    }
    if covered != set {
        // Some bit named a joint without its group-mates: a partial channel,
        // which no clip can drive.
        return Err(unknown);
    }
    if mask.is_empty() {
        return Err(ClipViewError::NoChannels);
    }
    Ok(mask)
}

/// Write `clip` into `out`, replacing whatever it held.
///
/// The emitter's half of the mapping. Fallible only on the one thing a loaded
/// clip can still be wrong about from the message's point of view: being longer
/// than the message holds.
pub fn write_clip(clip: &Clip, out: &mut ClipConfig) -> Result<(), LibraryWriteError> {
    let frames = clip.frames();
    if frames.len() > MAX_CLIP_FRAMES {
        return Err(LibraryWriteError::TooManyFrames {
            frames: frames.len(),
        });
    }
    out.mask = joint_mask_bits(clip.mask());
    out.frame_rate_hz = FLOOR_TICK_HZ;
    out.blend_in_ms = clip.blend_in_ms();
    out.blend_out_ms = clip.blend_out_ms();
    out.max_speed = clip.max_speed();
    out.frames.clear();
    for frame in frames {
        let slot = out
            .frames
            .try_grow()
            .expect("frame count is within capacity");
        write_frame(frame, slot);
    }
    Ok(())
}

/// Write `clips` and `motions` into `out` in the orders given, replacing
/// whatever it held.
///
/// The orders are the two identities: a script names a clip by its index among
/// `clips`, a schedule names a motion by its index among `motions`. They are
/// separate numberings over the same assets, which is why both are written from
/// one call — the segments' clip references are resolved against these very
/// clips, so no caller can pair a motion table with the wrong clip table.
///
/// # Errors
///
/// [`LibraryWriteError`] for a library that does not fit the message, and for a
/// motion naming a clip that is not in `clips` or holding a hold the message
/// cannot state.
pub fn write_library(
    clips: &[&Clip],
    motions: &[&Motion],
    out: &mut ClipLibraryConfig,
) -> Result<(), LibraryWriteError> {
    // Everything either table can be refused on is decided before any slot is
    // grown, so a refusal leaves the message empty rather than half-written.
    if clips.len() > MAX_CLIPS {
        return Err(LibraryWriteError::TooManyClips { clips: clips.len() });
    }
    if motions.len() > MAX_MOTIONS {
        return Err(LibraryWriteError::TooManyMotions {
            motions: motions.len(),
        });
    }
    for clip in clips {
        if clip.frames().len() > MAX_CLIP_FRAMES {
            return Err(LibraryWriteError::TooManyFrames {
                frames: clip.frames().len(),
            });
        }
    }
    for (motion_id, motion) in motions.iter().enumerate() {
        check_motion(motion_id, motion, clips)?;
    }

    out.clips.clear();
    for clip in clips {
        let slot = out.clips.try_grow().expect("clip count is within capacity");
        write_clip(clip, slot)?;
    }
    out.motions.clear();
    for (motion_id, motion) in motions.iter().enumerate() {
        let slot = out
            .motions
            .try_grow()
            .expect("motion count is within capacity");
        write_motion(motion_id, motion, clips, slot)?;
    }
    Ok(())
}

/// Whether `motion` is one the message can carry: its segment count, its lead
/// gap, and every segment's clip and hold.
///
/// The pre-pass half of the write. It decides on exactly what [`write_motion`]
/// decides on, by calling the same resolution, so what passes here places.
fn check_motion(
    motion_id: usize,
    motion: &Motion,
    clips: &[&Clip],
) -> Result<(), LibraryWriteError> {
    if motion.segments().len() > MAX_SEGMENTS {
        return Err(LibraryWriteError::TooManySegments {
            motion: motion_id,
            segments: motion.segments().len(),
        });
    }
    lead_gap_ms(motion_id, motion)?;
    for (index, segment) in motion.segments().iter().enumerate() {
        placed_segment(motion_id, index, segment, clips)?;
    }
    Ok(())
}

/// Write one flattened motion into `out`, resolving its segments' clips against
/// `clips` by name.
///
/// `motion_id` names the motion in a refusal and nothing else: what is written
/// carries no id, because a motion's id is its position in the library. Every
/// refusal it could return has already been taken by [`check_motion`].
fn write_motion(
    motion_id: usize,
    motion: &Motion,
    clips: &[&Clip],
    out: &mut MotionConfig,
) -> Result<(), LibraryWriteError> {
    out.lead_gap_ms = lead_gap_ms(motion_id, motion)?;
    out.segments.clear();
    for (index, segment) in motion.segments().iter().enumerate() {
        let (clip_id, gap_after_ms) = placed_segment(motion_id, index, segment, clips)?;
        let slot = out
            .segments
            .try_grow()
            .expect("segment count is within capacity");
        slot.clip_id = clip_id;
        slot.speed = segment.speed();
        slot.gap_after_ms = gap_after_ms;
    }
    Ok(())
}

/// A motion's lead gap as the whole milliseconds the message carries.
fn lead_gap_ms(motion_id: usize, motion: &Motion) -> Result<u32, LibraryWriteError> {
    gap_ms(motion.lead_gap_s()).ok_or(LibraryWriteError::GapOutOfRange {
        motion: motion_id,
        gap_s: motion.lead_gap_s(),
    })
}

/// One segment as the message states it: the id of the clip it names among
/// `clips`, and its trailing hold in whole milliseconds.
///
/// The one place a segment is resolved, so the pre-pass and the write cannot
/// disagree about what places.
fn placed_segment(
    motion_id: usize,
    index: usize,
    segment: &Segment,
    clips: &[&Clip],
) -> Result<(u16, u32), LibraryWriteError> {
    let clip_id = clips
        .iter()
        .position(|clip| clip.name() == segment.clip())
        .ok_or(LibraryWriteError::SegmentClipMissing {
            motion: motion_id,
            segment: index,
        })?;
    let gap_after_ms = gap_ms(segment.gap_after_s()).ok_or(LibraryWriteError::GapOutOfRange {
        motion: motion_id,
        gap_s: segment.gap_after_s(),
    })?;
    Ok((
        u16::try_from(clip_id).expect("a clip id is below the clip capacity"),
        gap_after_ms,
    ))
}

/// A hold in seconds as the whole milliseconds the message carries.
///
/// Rounded to the nearer millisecond rather than refused for landing between
/// two: a flattened gap is a document's whole milliseconds divided by the
/// speeds it nests under, so thirds and sevenths of a millisecond are ordinary
/// arithmetic, and the clock a motion is played on advances in whole ticks —
/// ten times coarser than the rounding — so nothing downstream can tell. What
/// is refused is a gap the field cannot state at all.
fn gap_ms(gap_s: f64) -> Option<u32> {
    if !gap_s.is_finite() || gap_s < 0.0 {
        return None;
    }
    let ms = (gap_s * 1000.0).round();
    if ms > f64::from(u32::MAX) {
        return None;
    }
    Some(ms as u32)
}

/// One frame into its slot: every field, zero for a channel the frame does not
/// drive.
///
/// Writing the zeros rather than relying on the slot arriving cleared keeps the
/// invariant local — an unmasked channel reads zero because this wrote zero,
/// not because of what a re-grown message slot happens to hold — and it is what
/// the reader requires. Ten doubles; there is nothing to save.
fn write_frame(frame: &DeltaFrame, out: &mut ClipFrame) {
    for field in &FRAME_FIELDS {
        (field.set)(out, (field.of_delta)(frame).unwrap_or(0.0));
    }
}

/// One field of a frame: its key, the channel that drives it, and the three
/// ways the mapping touches it.
///
/// The mapping is one table, so a channel added to `ClipFrame` is one row here
/// rather than a coordinated edit of an emitter's printer, a writer and a
/// checker that no compiler pairs up.
pub struct FrameField {
    /// The schema's field name, which is also the protobuf text's key.
    pub key: &'static str,
    /// The channel whose mask bit says whether this field means anything.
    pub channel: Channel,
    /// Read it out of a message frame.
    pub get: fn(&ClipFrame) -> f64,
    /// Write it into a message frame.
    pub set: fn(&mut ClipFrame, f64),
    /// Read it out of a loaded frame, if that frame drives its channel.
    pub of_delta: fn(&DeltaFrame) -> Option<f64>,
}

/// Every field of a frame, in the schema's declared order.
pub const FRAME_FIELDS: [FrameField; 10] = [
    FrameField {
        key: "head_dx",
        channel: Channel::Head,
        get: |frame| frame.head_dx,
        set: |frame, value| frame.head_dx = value,
        of_delta: |frame| frame.head.map(|head| head.translation.vector.x),
    },
    FrameField {
        key: "head_dy",
        channel: Channel::Head,
        get: |frame| frame.head_dy,
        set: |frame, value| frame.head_dy = value,
        of_delta: |frame| frame.head.map(|head| head.translation.vector.y),
    },
    FrameField {
        key: "head_dz",
        channel: Channel::Head,
        get: |frame| frame.head_dz,
        set: |frame, value| frame.head_dz = value,
        of_delta: |frame| frame.head.map(|head| head.translation.vector.z),
    },
    FrameField {
        key: "quat_w",
        channel: Channel::Head,
        get: |frame| frame.quat_w,
        set: |frame, value| frame.quat_w = value,
        of_delta: |frame| frame.head.map(|head| head.rotation.w),
    },
    FrameField {
        key: "quat_x",
        channel: Channel::Head,
        get: |frame| frame.quat_x,
        set: |frame, value| frame.quat_x = value,
        of_delta: |frame| frame.head.map(|head| head.rotation.i),
    },
    FrameField {
        key: "quat_y",
        channel: Channel::Head,
        get: |frame| frame.quat_y,
        set: |frame, value| frame.quat_y = value,
        of_delta: |frame| frame.head.map(|head| head.rotation.j),
    },
    FrameField {
        key: "quat_z",
        channel: Channel::Head,
        get: |frame| frame.quat_z,
        set: |frame, value| frame.quat_z = value,
        of_delta: |frame| frame.head.map(|head| head.rotation.k),
    },
    FrameField {
        key: "body_yaw_d",
        channel: Channel::BodyYaw,
        get: |frame| frame.body_yaw_d,
        set: |frame, value| frame.body_yaw_d = value,
        of_delta: |frame| frame.body_yaw,
    },
    FrameField {
        key: "antenna_right_d",
        channel: Channel::Antennas,
        get: |frame| frame.antenna_right_d,
        set: |frame, value| frame.antenna_right_d = value,
        of_delta: |frame| frame.antennas.map(|[right, _left]| right),
    },
    FrameField {
        key: "antenna_left_d",
        channel: Channel::Antennas,
        get: |frame| frame.antenna_left_d,
        set: |frame, value| frame.antenna_left_d = value,
        of_delta: |frame| frame.antennas.map(|[_right, left]| left),
    },
];

/// A configured motion that cannot be played as it stands.
///
/// The motion-level counterpart of [`ClipViewError`], and the same kind of
/// fact: the segments say something the play path's arithmetic does not allow.
/// A motion's own clips are refused as clips, so nothing here restates a
/// per-clip fact — a segment names one and carries what the flattening spent on
/// it.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum MotionViewError {
    /// A script named a motion the library does not have.
    #[error("motion id {motion_id} is not one of the library's {motions} motions")]
    UnknownMotion {
        /// The id asked for.
        motion_id: usize,
        /// How many motions there are.
        motions: usize,
    },

    /// The motion has no segments: it would play nothing at all.
    ///
    /// A hold with no clip after it is not a motion, which is the same refusal
    /// the loader makes of a sequence flattening to no clips.
    #[error("motion has no segments")]
    NoSegments,

    /// A segment naming a clip that is not one, or is not there.
    #[error("segment {segment} names clip {clip_id}: {source}")]
    SegmentClip {
        /// Which segment.
        segment: usize,
        /// The clip it named.
        clip_id: usize,
        /// What the clip refused with.
        #[source]
        source: ClipViewError,
    },

    /// A flattened segment speed outside the bounds anything may be played at.
    ///
    /// The nesting's product, not an invocation's: this is what the motion
    /// costs before anyone asks for a speed at all.
    #[error(
        "segment {segment} plays at {speed}x before any invocation, \
         outside the {MIN_SPEED}-{MAX_SPEED} bounds"
    )]
    SegmentSpeed {
        /// Which segment.
        segment: usize,
        /// The flattened speed.
        speed: f64,
    },

    /// A flattened segment speed above what that segment's clip admits.
    #[error("segment {segment} plays at {speed}x, above its clip's ceiling of {ceiling}x")]
    SegmentPastCeiling {
        /// Which segment.
        segment: usize,
        /// The flattened speed.
        speed: f64,
        /// What the clip admits.
        ceiling: f64,
    },
}

/// Which asset of a library will not play, and why.
///
/// Two numberings, so two arms: a clip id indexes the clips and a motion id
/// indexes the motions, and a caller that renders a name has to know which
/// table to look in.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum UnplayableAsset {
    /// A clip the library carries that will not play.
    #[error("clip {clip_id} cannot be played: {source}")]
    Clip {
        /// The clip's id, which is its position among the clips.
        clip_id: usize,
        /// What the clip itself refused with.
        #[source]
        source: ClipViewError,
    },

    /// A motion the library carries that will not play.
    #[error("motion {motion_id} cannot be played: {source}")]
    Motion {
        /// The motion's id, which is its position among the motions.
        motion_id: usize,
        /// What the motion itself refused with.
        #[source]
        source: MotionViewError,
    },
}

/// The clip `clip_id` names, with every invariant a player relies on
/// established.
///
/// Walks the clip's frames, so it costs the clip's length. A caller doing this
/// every tick is paying for it every tick: [`ValidatedLibrary`] is the handle
/// that pays once.
///
/// # Errors
///
/// [`ClipViewError`] for a clip id the library does not have, or a clip that
/// will not play.
pub fn clip_at(library: &ClipLibraryConfig, clip_id: usize) -> Result<ClipView<'_>, ClipViewError> {
    ClipView::new(find(library, clip_id)?)
}

/// The clip message `clip_id` names.
fn find(library: &ClipLibraryConfig, clip_id: usize) -> Result<&ClipConfig, ClipViewError> {
    library
        .clips
        .get(clip_id)
        .ok_or(ClipViewError::UnknownClip {
            clip_id,
            clips: library.clips.len(),
        })
}

/// A library whose every clip has been walked and found playable.
///
/// The tick path's handle, and the only thing [`Self::playable`] hangs off: the
/// fast accessor takes the per-frame invariants as established, so it is sound
/// only over a library a walk has established them for, and reaching it
/// requires holding the result of that walk. What a doc comment could only ask
/// of a caller, this asks of the compiler.
///
/// Cheap to hold and to copy: it borrows the same message the view does and
/// carries no per-clip work.
#[derive(Clone, Copy, Debug)]
pub struct ValidatedLibrary<'a> {
    /// The library this walk passed over.
    library: &'a ClipLibraryConfig,
}

impl<'a> ValidatedLibrary<'a> {
    /// Establish every clip's invariants, over the whole library, once.
    ///
    /// A library is configuration: immutable for the life of the process, so
    /// what it holds is worth establishing once rather than per use. What comes
    /// back is the checked library, which is what a tick path takes its clips
    /// out of.
    ///
    /// The handle borrows the message, so a host whose memory between
    /// executions is a state slot cannot carry it across them: as it stands a
    /// control period that wants one pays the whole walk again.
    /// TODO(library-walk-per-execution)
    ///
    /// # Errors
    ///
    /// [`UnplayableAsset`]: whatever the first unplayable clip or motion
    /// refuses with, and which one it was — the frame or segment index alone
    /// does not say that.
    pub fn of(library: &'a ClipLibraryConfig) -> Result<Self, UnplayableAsset> {
        for clip_id in 0..library.clips.len() {
            clip_at(library, clip_id)
                .map_err(|source| UnplayableAsset::Clip { clip_id, source })?;
        }
        // The clips first, because a motion's own checks read its segments'
        // clips: what the segment spends of a clip's ceiling means nothing
        // until the clip is one.
        let checked = Self { library };
        for motion_id in 0..library.motions.len() {
            checked
                .playable_motion(motion_id)
                .map_err(|source| UnplayableAsset::Motion { motion_id, source })?;
        }
        Ok(checked)
    }

    /// How many clips there are. A clip id is an index below this.
    #[must_use]
    pub fn clips(&self) -> usize {
        self.library.clips.len()
    }

    /// How many motions there are. A motion id is an index below this.
    #[must_use]
    pub fn motions(&self) -> usize {
        self.library.motions.len()
    }

    /// The clip `clip_id` names, taking the frame-level invariants as already
    /// established by the walk this handle stands for.
    ///
    /// The tick path's accessor: costs nothing per frame, which is what a
    /// player handed a clip relies on. The clip-level facts are still checked
    /// here, because they cost one read each:
    /// a mask this stack does not know, a foreign frame grid, no frames at all.
    ///
    /// # Errors
    ///
    /// [`ClipViewError`] for a clip id the library does not have, or a
    /// clip-level fact that does not hold.
    pub fn playable(&self, clip_id: usize) -> Result<ClipView<'a>, ClipViewError> {
        ClipView::prevalidated(find(self.library, clip_id)?)
    }

    /// The clip `clip_id` names, taking every clip-level fact as established
    /// too.
    ///
    /// For a caller that has already been told this clip is playable and is
    /// coming back for it — a motion's segment, per tick, per channel. The
    /// checks [`Self::playable`] makes are the same answer every time over the
    /// same immutable message, and asking again per control period is work a
    /// fixed-rate loop pays for nothing.
    ///
    /// # Panics
    ///
    /// If the library does not answer for `clip_id`, or answers with a clip the
    /// walk this handle stands for would have refused — neither of which a
    /// segment of a constructed [`MotionView`] reaches.
    fn established(&self, clip_id: usize) -> ClipView<'a> {
        ClipView::established(find(self.library, clip_id).expect("a clip a checked motion names"))
    }

    /// The motion `motion_id` names, with what it is like to play derived from
    /// its segments' clips.
    ///
    /// Costs one pass over the segments, which is a handful of reads: the
    /// per-frame work is the clips', already established by the walk this
    /// handle stands for.
    ///
    /// # Errors
    ///
    /// [`MotionViewError`] for a motion id the library does not have, or a
    /// motion whose segments do not describe a playable walk.
    pub fn playable_motion(&self, motion_id: usize) -> Result<MotionView<'a>, MotionViewError> {
        let motion = self
            .library
            .motions
            .get(motion_id)
            .ok_or(MotionViewError::UnknownMotion {
                motion_id,
                motions: self.library.motions.len(),
            })?;
        MotionView::over(*self, motion)
    }
}

/// One segment of a motion: the clip it plays, the speed the flattening left on
/// it, and the hold that follows.
#[derive(Clone, Copy, Debug)]
pub struct SegmentView<'a> {
    /// The clip this segment plays.
    pub clip: ClipView<'a>,
    /// The multiplier the nesting spent on the clip's own clock. An
    /// invocation's speed multiplies this rather than replacing it.
    pub speed: f64,
    /// The hold following this clip's last frame, seconds, at 1.0x invocation
    /// speed. Zero when the next segment follows at once.
    pub gap_after_s: f64,
}

impl SegmentView<'_> {
    /// How long the clip's frames occupy on the motion's clock, seconds.
    #[must_use]
    pub fn play_span_s(&self) -> f64 {
        self.clip.duration_s() / self.speed
    }

    /// How long the segment occupies on the motion's clock, its hold included.
    #[must_use]
    pub fn span_s(&self) -> f64 {
        self.play_span_s() + self.gap_after_s
    }
}

/// One configured motion a player can play: its segments, and what it is like
/// to play them.
///
/// The derived facts — which channels it drives, how long it runs, how fast it
/// may be invoked, how it blends — are computed here from the segments' clips
/// and are stored nowhere: the asset carries the segments alone, so no second
/// opinion about a motion can disagree with the frames it is made of.
#[derive(Clone, Copy, Debug)]
pub struct MotionView<'a> {
    /// The library the segments' clips are taken from.
    library: ValidatedLibrary<'a>,
    /// The message, checked at construction.
    motion: &'a MotionConfig,
    /// Every channel any segment drives.
    mask: ChannelMask,
    /// How long the whole walk runs at 1.0x, seconds, holds included.
    duration_s: f64,
    /// The fastest invocation the segments admit together.
    max_speed: f64,
    /// The entry ramp, from the first segment's clip.
    blend_in_ms: u32,
    /// The exit ramp, from the last segment's clip.
    blend_out_ms: u32,
}

impl<'a> MotionView<'a> {
    /// A view over `motion`, with every invariant the play path relies on
    /// established and every derived fact computed.
    fn over(
        library: ValidatedLibrary<'a>,
        motion: &'a MotionConfig,
    ) -> Result<Self, MotionViewError> {
        if motion.segments.is_empty() {
            return Err(MotionViewError::NoSegments);
        }
        let mut mask = ChannelMask::empty();
        let mut duration_s = gap_s(motion.lead_gap_ms);
        // The tightest ceiling any segment leaves, never above the global one.
        let mut max_speed = MAX_SPEED;
        // The edges of the whole walk: a motion ramps in on the clip it starts
        // with and out of the one it ends on. The seams in between are stepped
        // across rather than blended, which is what the load-time seam check
        // bounds.
        let (mut blend_in_ms, mut blend_out_ms) = (0, 0);
        for (segment, config) in motion.segments.iter().enumerate() {
            let clip_id = usize::from(config.clip_id);
            let clip =
                library
                    .playable(clip_id)
                    .map_err(|source| MotionViewError::SegmentClip {
                        segment,
                        clip_id,
                        source,
                    })?;
            let speed = config.speed;
            // The flattening's own predicates, asked again of the message: the
            // same arithmetic, so the two cannot disagree about a product an ulp
            // either side of a bound.
            if !speed_in_bounds(speed) {
                return Err(MotionViewError::SegmentSpeed { segment, speed });
            }
            let ceiling = clip.max_speed();
            if !within_ceiling(speed, ceiling) {
                return Err(MotionViewError::SegmentPastCeiling {
                    segment,
                    speed,
                    ceiling,
                });
            }
            for channel in Channel::ALL {
                if clip.mask().contains(channel) {
                    mask.insert(channel);
                }
            }
            duration_s += clip.duration_s() / speed + gap_s(config.gap_after_ms);
            max_speed = max_speed.min(ceiling / speed);
            if segment == 0 {
                blend_in_ms = clip.blend_in_ms();
            }
            blend_out_ms = clip.blend_out_ms();
        }
        // Every segment has just been proven playable at 1.0x, so the motion is
        // too. Without the floor it need not say so: a nesting product an ulp
        // above a clip's own ceiling is admitted by `SPEED_EPS` and divides
        // back out an ulp below one, which would refuse the default invocation
        // of a motion the emitter accepted.
        let max_speed = max_speed.max(1.0);
        Ok(Self {
            library,
            motion,
            mask,
            duration_s,
            max_speed,
            blend_in_ms,
            blend_out_ms,
        })
    }

    /// How many segments the motion has. Never zero.
    #[must_use]
    pub fn segments(&self) -> usize {
        self.motion.segments.len()
    }

    /// Segment `segment` of the motion.
    ///
    /// # Panics
    ///
    /// If `segment` is not below [`MotionView::segments`], or if the library
    /// this view was built over no longer answers for the clip that segment
    /// names — neither of which a constructed view reaches.
    ///
    /// Costs a mask decode and three reads: what the clip is like was
    /// established when this view was built, and a play path walking the
    /// segments every control period is the caller.
    #[must_use]
    pub fn segment(&self, segment: usize) -> SegmentView<'a> {
        let config = segment_at(self.motion, segment);
        SegmentView {
            clip: self.library.established(usize::from(config.clip_id)),
            speed: config.speed,
            gap_after_s: gap_s(config.gap_after_ms),
        }
    }

    /// The hold before the first segment, seconds, at 1.0x.
    ///
    /// The base alone holds through it: there is no previous segment whose
    /// delta could be frozen.
    #[must_use]
    pub fn lead_gap_s(&self) -> f64 {
        gap_s(self.motion.lead_gap_ms)
    }

    /// Every channel any of this motion's clips drives.
    ///
    /// The union, not an intersection: a channel is driven by the motion even
    /// if only one segment touches it. Between such segments the channel
    /// contributes no delta, which is not the motion having nothing to say
    /// about it.
    #[must_use]
    pub fn mask(&self) -> ChannelMask {
        self.mask
    }

    /// How long the motion runs at 1.0x, seconds, holds included.
    #[must_use]
    pub fn duration_s(&self) -> f64 {
        self.duration_s
    }

    /// The highest invocation speed the segments admit together.
    ///
    /// The tightest clip's ceiling after the nesting has spent its share of it,
    /// never above the global bound and never below 1.0x.
    #[must_use]
    pub fn max_speed(&self) -> f64 {
        self.max_speed
    }

    /// The entry blend ramp, milliseconds, from the first segment's clip.
    #[must_use]
    pub fn blend_in_ms(&self) -> u32 {
        self.blend_in_ms
    }

    /// The exit blend ramp, milliseconds, from the last segment's clip.
    #[must_use]
    pub fn blend_out_ms(&self) -> u32 {
        self.blend_out_ms
    }
}

/// Segment `segment` of `motion`.
///
/// # Panics
///
/// If `segment` is not one of the motion's.
fn segment_at(motion: &MotionConfig, segment: usize) -> &MotionSegment {
    motion.segments.get(segment).expect("segment is in range")
}

/// A hold the message states in milliseconds, as the seconds the play path
/// counts in.
fn gap_s(gap_ms: u32) -> f64 {
    f64::from(gap_ms) / 1000.0
}

/// One configured clip a player can play.
///
/// A configured clip is a clip and nothing else: no lead gap, no trailing hold,
/// no invocation speed.
#[derive(Clone, Copy, Debug)]
pub struct ClipView<'a> {
    /// The message, checked at construction.
    clip: &'a ClipConfig,
    /// What it drives, decoded once.
    mask: ChannelMask,
}

impl<'a> ClipView<'a> {
    /// A view over `clip`, with every invariant a player relies on established.
    ///
    /// # Errors
    ///
    /// [`ClipViewError`] for a mask this stack does not know, a foreign frame
    /// grid, no frames at all, or a frame that carries what its mask does not
    /// allow.
    pub fn new(clip: &'a ClipConfig) -> Result<Self, ClipViewError> {
        let view = Self::opened(clip)?;
        for (index, frame) in clip.frames.iter().enumerate() {
            check_frame(index, frame, view.mask)?;
        }
        Ok(view)
    }

    /// A view over `clip`, taking the per-frame invariants as established.
    ///
    /// What establishes those invariants is a walk over the whole library, so
    /// the handle that walk hands back is the only way in.
    fn prevalidated(clip: &'a ClipConfig) -> Result<Self, ClipViewError> {
        Self::opened(clip)
    }

    /// A view over `clip`, taking every invariant as established.
    ///
    /// The mask is decoded because the view carries it; nothing else is asked,
    /// which is what makes this the accessor a per-tick walk uses. The
    /// established facts are established by the same library walk that admits
    /// the only handle reaching this.
    ///
    /// # Panics
    ///
    /// If the clip's mask is not one this stack knows — a clip the library walk
    /// refused, which no handle it hands back answers with.
    fn established(clip: &'a ClipConfig) -> Self {
        let mask = mask_from_bits(clip.mask).expect("a mask the library walk decoded");
        Self { clip, mask }
    }

    /// The clip-level checks, which cost one read each.
    fn opened(clip: &'a ClipConfig) -> Result<Self, ClipViewError> {
        let mask = mask_from_bits(clip.mask)?;
        if clip.frame_rate_hz != FLOOR_TICK_HZ {
            return Err(ClipViewError::FrameRate {
                frame_rate_hz: clip.frame_rate_hz,
            });
        }
        if clip.frames.is_empty() {
            return Err(ClipViewError::NoFrames);
        }
        if !clip.max_speed.is_finite() || clip.max_speed <= 0.0 {
            return Err(ClipViewError::SpeedCeiling {
                max_speed: clip.max_speed,
            });
        }
        Ok(Self { clip, mask })
    }

    /// The channels this clip drives.
    #[must_use]
    pub fn mask(&self) -> ChannelMask {
        self.mask
    }

    /// How many frames it has. Never zero.
    #[must_use]
    pub fn frames(&self) -> usize {
        self.clip.frames.len()
    }

    /// How long it runs at 1.0x, seconds, from its own frame count.
    #[must_use]
    pub fn duration_s(&self) -> f64 {
        clip_duration_ms(self.frames()) / 1000.0
    }

    /// The entry blend ramp, milliseconds.
    #[must_use]
    pub fn blend_in_ms(&self) -> u32 {
        self.clip.blend_in_ms
    }

    /// The exit blend ramp, milliseconds.
    #[must_use]
    pub fn blend_out_ms(&self) -> u32 {
        self.clip.blend_out_ms
    }

    /// The highest invocation speed the frames admit.
    ///
    /// Derived at load and carried, never re-derived: the derivation costs a
    /// kinematic solve per frame.
    #[must_use]
    pub fn max_speed(&self) -> f64 {
        self.clip.max_speed
    }

    /// Frame `frame` of the clip.
    ///
    /// # Panics
    ///
    /// If `frame` is not below [`ClipView::frames`].
    #[must_use]
    pub fn frame(&self, frame: usize) -> DeltaFrame {
        let row = self.clip.frames.get(frame).expect("frame is in range");
        DeltaFrame {
            head: self.mask.contains(Channel::Head).then(|| {
                Isometry3::from_parts(
                    Translation3::new(row.head_dx, row.head_dy, row.head_dz),
                    UnitQuaternion::from_quaternion(quaternion(row)),
                )
            }),
            antennas: self
                .mask
                .contains(Channel::Antennas)
                .then_some([row.antenna_right_d, row.antenna_left_d]),
            body_yaw: self
                .mask
                .contains(Channel::BodyYaw)
                .then_some(row.body_yaw_d),
        }
    }
}

/// A fingerprint of what `view` holds: enough of its shape that a player's
/// state can say which clip it was left by.
///
/// The frame count, the mask and the ramps, hashed. Not the duration, which is
/// the frame count over the one grid and so says nothing the count has not said
/// already, and not the frames themselves — a fingerprint is taken once per
/// pick-up, and two clips
/// that agree on all of this and differ only in a frame value are a mistake
/// nothing here can catch. What it does catch is the case a reused row actually
/// meets: a slot holding the state of a different clip, whose frames are
/// indexed by a clock that meant something else.
///
/// A free function rather than a method on the message, so nothing can answer
/// with a fingerprint of its own that agrees with nobody.
#[must_use]
pub fn track_fingerprint(view: &ClipView<'_>) -> u64 {
    let mut hash = Fnv1a::new();
    hash.eat(&(view.frames() as u64).to_le_bytes());
    hash.eat(&[mask_bits(view.mask())]);
    hash.eat(&view.blend_in_ms().to_le_bytes());
    hash.eat(&view.blend_out_ms().to_le_bytes());
    hash.finish()
}

/// A fingerprint of the whole walk `view` describes: enough of its shape that a
/// player's state can say which motion it was left by.
///
/// The lead gap, then each segment's clip fingerprint with what the flattening
/// spent on it — the speed and the hold after it — folded in order into the same
/// stream. The whole motion and not just its first clip: two motions that open
/// on the same clip and diverge after it are exactly the pair a reused row
/// meets, and a fingerprint that agreed on them would restore a player of one
/// walk against the other's frames.
///
/// A free function rather than a method on the view, for the same reason
/// [`track_fingerprint`] is one.
#[must_use]
pub fn motion_fingerprint(view: &MotionView<'_>) -> u64 {
    let mut hash = Fnv1a::new();
    hash.eat(&(view.segments() as u64).to_le_bytes());
    hash.eat(&view.lead_gap_s().to_bits().to_le_bytes());
    for index in 0..view.segments() {
        let segment = view.segment(index);
        hash.eat(&track_fingerprint(&segment.clip).to_le_bytes());
        hash.eat(&segment.speed.to_bits().to_le_bytes());
        hash.eat(&segment.gap_after_s.to_bits().to_le_bytes());
    }
    hash.finish()
}

/// FNV-1a over whatever the fingerprints above feed it, stated once.
///
/// Both of them are state-identity guards, so the two have to answer with the
/// same arithmetic: a widening or a salt landing in one and not the other is a
/// player restored over the wrong frames, and a hash written out twice is a
/// hash that half-lands such a change.
struct Fnv1a(u64);

impl Fnv1a {
    /// A hash of nothing yet: FNV-1a's offset basis.
    fn new() -> Self {
        Self(0xcbf2_9ce4_8422_2325)
    }

    /// Fold `bytes` in, in the order they arrive.
    fn eat(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.0 ^= u64::from(*byte);
            self.0 = self.0.wrapping_mul(0x100_0000_01b3);
        }
    }

    /// What it comes to.
    fn finish(self) -> u64 {
        self.0
    }
}

/// A mask as one byte, a bit per channel in declared order.
///
/// Only for the fingerprints above: this crate's masks are per-channel flags
/// everywhere else, and a packed form is a wire concern.
fn mask_bits(mask: ChannelMask) -> u8 {
    let mut bits = 0;
    for (index, channel) in Channel::ALL.into_iter().enumerate() {
        if mask.contains(channel) {
            bits |= 1 << index;
        }
    }
    bits
}

/// Every value of one frame, against the mask that says which of them mean
/// anything: finite where the mask names the channel, zero where it does not.
fn check_frame(index: usize, frame: &ClipFrame, mask: ChannelMask) -> Result<(), ClipViewError> {
    for field in &FRAME_FIELDS {
        let value = (field.get)(frame);
        if mask.contains(field.channel) {
            if !value.is_finite() {
                return Err(ClipViewError::NonFinite {
                    frame: index,
                    key: field.key,
                    value,
                });
            }
        } else if value != 0.0 {
            return Err(ClipViewError::StrayChannel {
                frame: index,
                key: field.key,
                channel: field.channel,
                value,
            });
        }
    }

    if mask.contains(Channel::Head) {
        // The norm is only meaningful once the coordinates are known finite,
        // which the loop above has established.
        let norm = quaternion(frame).norm();
        if (norm - 1.0).abs() > QUAT_NORM_TOL {
            return Err(ClipViewError::Quaternion { frame: index, norm });
        }
    }
    Ok(())
}

/// A frame's head rotation as written, before it is made a unit.
fn quaternion(frame: &ClipFrame) -> Quaternion<f64> {
    Quaternion::new(frame.quat_w, frame.quat_x, frame.quat_y, frame.quat_z)
}

#[cfg(test)]
mod tests {
    use super::*;

    use brenn_reachy__cogs__config_clk_rs::{ClipConfigWire, ClipLibraryConfigWire};

    use crate::format::{ClipDoc, FrameDoc};
    use crate::library::SPEED_EPS;
    use crate::speed::ClipLimits;
    use reachy_motion::joints::JointStep;

    /// Generous step bounds, so a fixture's round numbers load as written: what
    /// is under test here is the crossing, not the derivation.
    fn limits() -> ClipLimits {
        ClipLimits {
            max_step: JointStep {
                legs: 100.0,
                body_yaw: 100.0,
                antennas: 100.0,
            },
            ..ClipLimits::default()
        }
    }

    /// A clip driving all three channels, `frames` frames long, whose values
    /// walk so that a frame confused with its neighbour shows up.
    fn all_channels_doc(name: &str, frames: usize) -> ClipDoc {
        ClipDoc {
            version: 1,
            kind: "clip".to_owned(),
            name: name.to_owned(),
            description: None,
            channels: vec![Channel::Head, Channel::BodyYaw, Channel::Antennas],
            frame_hz: FLOOR_TICK_HZ,
            max_speed: 1.0,
            blend_in_ms: Some(0),
            blend_out_ms: Some(0),
            frames: (0..frames)
                .map(|index| {
                    let step = index as f64;
                    // A small rotation about z, written as a unit quaternion so
                    // the loader takes it as authored.
                    let angle = 0.001 * step;
                    FrameDoc {
                        dt: Some([0.0001 * step, 0.0002 * step, 0.0003 * step]),
                        dq: Some([(angle / 2.0).cos(), 0.0, 0.0, (angle / 2.0).sin()]),
                        body_yaw: Some(0.002 * step),
                        antennas: Some([0.003 * step, -0.003 * step]),
                    }
                })
                .collect(),
        }
    }

    /// An antennas-only clip, for the unmasked-channel cases.
    fn antenna_doc(name: &str, frames: usize) -> ClipDoc {
        ClipDoc {
            channels: vec![Channel::Antennas],
            frames: (0..frames)
                .map(|index| FrameDoc {
                    antennas: Some([0.01 * index as f64, -0.01 * index as f64]),
                    ..FrameDoc::default()
                })
                .collect(),
            ..all_channels_doc(name, 0)
        }
    }

    /// A view's refusal, or nothing where it was accepted.
    ///
    /// A `ClipView` borrows a message and is not comparable, so a refusal is
    /// asserted against this rather than against the `Result` the constructor
    /// hands back.
    fn refused<T>(result: Result<T, ClipViewError>) -> Result<(), ClipViewError> {
        result.map(|_| ())
    }

    fn load(doc: ClipDoc) -> Clip {
        Clip::from_doc(doc, &limits()).expect("fixture loads")
    }

    /// `clip` written into a fresh message.
    fn configured(clip: &Clip) -> Box<ClipConfigWire> {
        let mut out = Box::new(ClipConfigWire::new());
        write_clip(clip, out.clear_valid()).expect("fixture fits");
        out
    }

    /// The validated form of a message a case built.
    fn valid(message: &ClipConfigWire) -> &ClipConfig {
        message.validate().expect("a written clip validates")
    }

    /// The validated form of a library a case built.
    fn valid_library(message: &ClipLibraryConfigWire) -> &ClipLibraryConfig {
        message.validate().expect("a written library validates")
    }

    /// The channels and the joint groups are the same partition of the same
    /// machine, so the crossing is a bijection and every joint is covered once.
    #[test]
    fn every_channel_names_its_own_whole_joint_group() {
        let mut covered = JointFlags::NONE;
        for channel in Channel::ALL {
            let joints = channel_group(channel).joints();
            assert!(!flags::is_empty(joints), "{channel} drives no joints");
            assert!(
                flags::len(covered | joints) == flags::len(covered) + flags::len(joints),
                "{channel}'s joints overlap another channel's"
            );
            covered |= joints;
        }
        assert_eq!(covered, flags::all(), "the channels miss a joint");
        assert_eq!(
            Channel::ALL.len(),
            JointGroup::ALL.len(),
            "a joint group has no channel"
        );
    }

    /// The mask carriage is the message's only encoded field, so every mask a
    /// clip can have crosses and comes back.
    #[test]
    fn every_mask_crosses_and_comes_back() {
        for bits in 1u8..8 {
            let mut mask = ChannelMask::empty();
            for (index, channel) in Channel::ALL.into_iter().enumerate() {
                if bits & (1 << index) != 0 {
                    mask.insert(channel);
                }
            }
            let carried = joint_mask_bits(mask);
            assert_eq!(
                mask_from_bits(carried),
                Ok(mask),
                "mask {mask:?} carried as {carried:#x}"
            );
        }
    }

    #[test]
    fn an_empty_mask_drives_nothing_and_is_refused() {
        assert_eq!(mask_from_bits(0), Err(ClipViewError::NoChannels));
    }

    /// One crank without its five siblings is not a channel: the head moves as a
    /// linkage or not at all.
    #[test]
    fn a_partial_channel_group_is_refused() {
        let one_leg = flags::iter(JointGroup::Legs.joints())
            .next()
            .expect("legs are joints");
        let mut set = JointFlags::NONE;
        flags::insert(&mut set, one_leg);
        let bits = JointFlagsWire::from(set).0;
        assert_eq!(
            mask_from_bits(bits),
            Err(ClipViewError::UnknownMaskBits { bits })
        );
    }

    #[test]
    fn bits_beyond_the_joint_set_are_refused() {
        let bits = 1 << 15;
        assert_eq!(
            mask_from_bits(bits),
            Err(ClipViewError::UnknownMaskBits { bits })
        );
    }

    /// What was written is what the view answers: the clip's own numbers, field
    /// for field, over a track long enough to catch an off-by-one.
    #[test]
    fn a_written_clip_reads_back_as_the_clip() {
        let clip = load(all_channels_doc("walk", 12));
        let message = configured(&clip);
        let view = ClipView::new(valid(&message)).expect("a written clip is playable");

        assert_eq!(view.mask(), clip.mask());
        assert_eq!(view.frames(), clip.frames().len());
        assert_eq!(view.duration_s(), clip.duration_s());
        assert_eq!(view.blend_in_ms(), clip.blend_in_ms());
        assert_eq!(view.blend_out_ms(), clip.blend_out_ms());

        for (index, frame) in clip.frames().iter().enumerate() {
            let read = view.frame(index);
            let head = frame.head.expect("head is masked");
            let read_head = read.head.expect("head is masked");
            assert!(
                (read_head.translation.vector - head.translation.vector).norm() < 1e-15,
                "frame {index} translation"
            );
            assert!(
                read_head.rotation.angle_to(&head.rotation) < 1e-12,
                "frame {index} rotation"
            );
            assert_eq!(read.body_yaw, frame.body_yaw, "frame {index} body yaw");
            assert_eq!(read.antennas, frame.antennas, "frame {index} antennas");
        }
    }

    /// A clip whose mask names one channel answers `None` for the other two,
    /// rather than a zero delta they never asked for.
    #[test]
    fn an_unmasked_channel_is_absent_rather_than_zero() {
        let clip = load(antenna_doc("wiggle", 4));
        let message = configured(&clip);
        let view = ClipView::new(valid(&message)).expect("playable");
        let frame = view.frame(1);
        assert!(frame.head.is_none(), "head is not masked");
        assert!(frame.body_yaw.is_none(), "body yaw is not masked");
        assert_eq!(frame.antennas, clip.frames()[1].antennas);
    }

    /// A library is its clips in order, and an id is an index into it.
    #[test]
    fn a_library_answers_by_index_and_refuses_anything_else() {
        let first = load(all_channels_doc("first", 3));
        let second = load(antenna_doc("second", 5));
        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&[&first, &second], &[], message.clear_valid()).expect("two clips fit");

        let library = valid_library(&message);
        assert_eq!(library.clips.len(), 2);
        assert_eq!(
            clip_at(library, 0).expect("first is playable").mask(),
            first.mask()
        );
        assert_eq!(clip_at(library, 1).expect("second is playable").frames(), 5);
        assert_eq!(
            refused(clip_at(library, 2)),
            Err(ClipViewError::UnknownClip {
                clip_id: 2,
                clips: 2
            })
        );
        assert_eq!(
            library.clips.capacity(),
            MAX_CLIPS,
            "the constant and the schema disagree about how many clips fit"
        );
        assert_eq!(
            first_slot_capacity(library),
            MAX_CLIP_FRAMES,
            "the constant and the schema disagree about how many frames fit"
        );
    }

    /// A library is established once, and the tick path then takes its clips
    /// without re-walking their frames — same mask, same span, same samples.
    #[test]
    fn a_validated_library_plays_the_same_clips_it_was_checked_as() {
        let first = load(all_channels_doc("first", 4));
        let second = load(antenna_doc("second", 6));
        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&[&first, &second], &[], message.clear_valid()).expect("two clips fit");

        let library = valid_library(&message);
        let checked_library = ValidatedLibrary::of(library).expect("both clips are playable");
        assert_eq!(checked_library.clips(), library.clips.len());
        for clip_id in 0..library.clips.len() {
            let checked = clip_at(library, clip_id).expect("playable");
            let taken = checked_library.playable(clip_id).expect("playable");
            assert_eq!(taken.mask(), checked.mask());
            assert_eq!(taken.frames(), checked.frames());
            assert_eq!(track_fingerprint(&taken), track_fingerprint(&checked));
            for frame in 0..checked.frames() {
                assert_eq!(taken.frame(frame), checked.frame(frame));
            }
        }
    }

    /// Validating a library names the clip that will not play: a frame index on
    /// its own does not say which clip it is a frame of.
    #[test]
    fn validating_a_library_names_the_clip_that_will_not_play() {
        let good = load(antenna_doc("good", 2));
        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&[&good, &good], &[], message.clear_valid()).expect("two clips fit");
        // Second clip only: a value in a channel its mask does not name.
        message
            .validate_mut()
            .expect("a written library validates")
            .clips
            .get_mut(1)
            .expect("the library has two clips")
            .frames
            .get_mut(0)
            .expect("the clip has frames")
            .body_yaw_d = 0.5;

        let library = valid_library(&message);
        let refusal = ValidatedLibrary::of(library).expect_err("the second clip is wrong");
        assert_eq!(
            refusal,
            UnplayableAsset::Clip {
                clip_id: 1,
                source: ClipViewError::StrayChannel {
                    frame: 0,
                    key: "body_yaw_d",
                    channel: Channel::BodyYaw,
                    value: 0.5,
                }
            }
        );
        // The clip-level facts of the bad clip still hold; what does not is a
        // frame of it, which is why there is no handle to take it out of.
        clip_at(library, 1).expect_err("the frame is still wrong");
    }

    /// A clip written over a longer one carries nothing of it: every field is
    /// written, so what an unmasked channel reads does not depend on what the
    /// message slot happened to hold.
    #[test]
    fn a_clip_written_over_another_carries_none_of_it() {
        let head_and_more = load(all_channels_doc("first", 4));
        let antennas = load(antenna_doc("second", 4));
        let mut message = Box::new(ClipConfigWire::new());
        write_clip(&head_and_more, message.clear_valid()).expect("fits");
        write_clip(&antennas, message.clear_valid()).expect("fits");

        let clip = valid(&message);
        let view = ClipView::new(clip).expect("an antennas-only clip");
        assert_eq!(view.mask(), antennas.mask());
        for frame in clip.frames.iter() {
            for field in &FRAME_FIELDS {
                if field.channel == Channel::Antennas {
                    continue;
                }
                assert_eq!(
                    (field.get)(frame),
                    0.0,
                    "{} carries the clip that was here before",
                    field.key
                );
            }
        }
    }

    /// Every fact the fingerprint names, varied one at a time.
    ///
    /// The fingerprint is the whole of what tells a player's state apart from
    /// one left by another clip, so a fact dropped from the hash is a state
    /// resumed over the wrong frames. Duration is not among them: it is the
    /// frame count over the one grid, and the frame-count row covers it.
    #[test]
    fn a_fingerprint_covers_every_fact_of_the_shape() {
        let base_doc = ClipDoc {
            blend_in_ms: Some(40),
            blend_out_ms: Some(60),
            ..all_channels_doc("base", 6)
        };
        let base = load(base_doc.clone());
        assert_eq!(
            (base.blend_in_ms(), base.blend_out_ms()),
            (40, 60),
            "the case rests on the ramps loading as written"
        );

        let variants = [
            (
                "one fewer frame",
                ClipDoc {
                    frames: base_doc.frames[..5].to_vec(),
                    ..base_doc.clone()
                },
            ),
            (
                "another mask",
                ClipDoc {
                    blend_in_ms: Some(40),
                    blend_out_ms: Some(60),
                    ..antenna_doc("base", 6)
                },
            ),
            (
                "another entry ramp",
                ClipDoc {
                    blend_in_ms: Some(80),
                    ..base_doc.clone()
                },
            ),
            (
                "another exit ramp",
                ClipDoc {
                    blend_out_ms: Some(80),
                    ..base_doc.clone()
                },
            ),
        ];

        let message = configured(&base);
        let wanted = track_fingerprint(&ClipView::new(valid(&message)).expect("playable"));
        for (what, doc) in variants {
            let clip = load(doc);
            let message = configured(&clip);
            let view = ClipView::new(valid(&message)).expect("playable");
            assert_ne!(
                track_fingerprint(&view),
                wanted,
                "a clip differing in {what} hashes as the base"
            );
        }

        let again = load(base_doc);
        let message = configured(&again);
        let view = ClipView::new(valid(&message)).expect("playable");
        assert_eq!(
            track_fingerprint(&view),
            wanted,
            "the same clip written again hashes differently"
        );
    }

    /// The frame capacity of the library's first clip slot.
    fn first_slot_capacity(library: &ClipLibraryConfig) -> usize {
        library
            .clips
            .get(0)
            .expect("the library has a clip")
            .frames
            .capacity()
    }

    /// A library longer than the message holds is refused rather than truncated:
    /// a truncation would silently renumber every clip a script names.
    #[test]
    fn a_library_past_capacity_is_refused() {
        let clip = load(antenna_doc("one", 2));
        let clips: Vec<&Clip> = (0..MAX_CLIPS + 1).map(|_| &clip).collect();
        let mut message = Box::new(ClipLibraryConfigWire::new());
        assert_eq!(
            write_library(&clips, &[], message.clear_valid()),
            Err(LibraryWriteError::TooManyClips {
                clips: MAX_CLIPS + 1
            })
        );
        assert!(
            valid_library(&message).clips.is_empty(),
            "a refused write left clips behind"
        );
    }

    /// A library that fills the message exactly is accepted: the last slot is
    /// one the emitter may write, and it must not be the one that panics.
    #[test]
    fn a_library_that_fills_the_message_is_accepted() {
        let clip = load(antenna_doc("one", 2));
        let clips: Vec<&Clip> = (0..MAX_CLIPS).map(|_| &clip).collect();
        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&clips, &[], message.clear_valid()).expect("a full library fits");
        let library = valid_library(&message);
        assert_eq!(library.clips.len(), MAX_CLIPS);
        ValidatedLibrary::of(library).expect("every clip of a full library plays");
    }

    /// The library a zeroed or unbound config slot reads as. A cog asking it for
    /// a clip gets a typed refusal naming what it has, not a panic.
    #[test]
    fn an_empty_library_names_no_clip() {
        let message = Box::new(ClipLibraryConfigWire::new());
        let library = valid_library(&message);
        assert!(library.clips.is_empty());
        let checked = ValidatedLibrary::of(library).expect("an empty library holds no clip");
        assert_eq!(checked.clips(), 0);
        assert_eq!(checked.motions(), 0);
        assert_eq!(
            refused(clip_at(library, 0)),
            Err(ClipViewError::UnknownClip {
                clip_id: 0,
                clips: 0
            })
        );
        assert_eq!(
            refused(checked.playable(0)),
            Err(ClipViewError::UnknownClip {
                clip_id: 0,
                clips: 0
            })
        );
    }

    /// The field table is the schema: a `Float64` added to `ClipFrame` that is
    /// not a row here would be a channel nothing zeroes, nothing checks and
    /// nothing prints, and the asset the emitter wrote would be one the
    /// generated conversion refuses at process setup with nothing pointing here.
    #[test]
    fn the_field_table_accounts_for_every_field_of_a_frame() {
        assert_eq!(
            size_of::<ClipFrame>(),
            FRAME_FIELDS.len() * size_of::<f64>(),
            "a frame has a field the table does not name"
        );
    }

    /// The one path to the per-frame fast accessor is the walk that makes it
    /// sound, so a library holding a frame the walk refuses hands back no
    /// handle to take clips out of — which is why a non-finite value cannot
    /// reach a `DeltaFrame` through this crate at all.
    #[test]
    fn a_library_with_a_non_finite_frame_hands_back_no_tick_handle() {
        let clip = load(antenna_doc("wiggle", 3));
        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&[&clip], &[], message.clear_valid()).expect("one clip fits");
        message
            .validate_mut()
            .expect("a written library validates")
            .clips
            .get_mut(0)
            .expect("the library has a clip")
            .frames
            .get_mut(1)
            .expect("the clip has frames")
            .antenna_right_d = f64::NAN;

        let refusal =
            ValidatedLibrary::of(valid_library(&message)).expect_err("a NaN frame is refused");
        assert!(
            matches!(
                refusal,
                UnplayableAsset::Clip {
                    clip_id: 0,
                    source: ClipViewError::NonFinite {
                        frame: 1,
                        key: "antenna_right_d",
                        ..
                    }
                }
            ),
            "{refusal}"
        );
    }

    /// A clip longer than the message holds is refused, and refused before
    /// anything is written.
    #[test]
    fn a_clip_past_capacity_is_refused() {
        let long = load(antenna_doc("long", MAX_CLIP_FRAMES + 1));
        let mut out = Box::new(ClipConfigWire::new());
        assert_eq!(
            write_clip(&long, out.clear_valid()),
            Err(LibraryWriteError::TooManyFrames {
                frames: MAX_CLIP_FRAMES + 1
            })
        );
        assert!(
            valid(&out).frames.is_empty(),
            "a refused write left frames behind"
        );

        let short = load(antenna_doc("short", 2));
        let mut message = Box::new(ClipLibraryConfigWire::new());
        assert_eq!(
            write_library(&[&short, &long], &[], message.clear_valid()),
            Err(LibraryWriteError::TooManyFrames {
                frames: MAX_CLIP_FRAMES + 1
            })
        );
        assert!(
            valid_library(&message).clips.is_empty(),
            "a refused library write left a clip behind"
        );
    }

    /// A clip that fills the message exactly is accepted: the bound is a limit,
    /// not a margin.
    #[test]
    fn a_clip_that_fills_the_message_is_accepted() {
        let full = load(antenna_doc("full", MAX_CLIP_FRAMES));
        let message = configured(&full);
        let view = ClipView::new(valid(&message)).expect("a full clip is playable");
        assert_eq!(view.frames(), MAX_CLIP_FRAMES);
    }

    #[test]
    fn a_clip_with_no_frames_is_refused() {
        let mut message = Box::new(ClipConfigWire::new());
        let clip = message.clear_valid();
        clip.mask = joint_mask_bits(ChannelMask::of(Channel::Antennas));
        clip.frame_rate_hz = FLOOR_TICK_HZ;
        assert_eq!(refused(ClipView::new(clip)), Err(ClipViewError::NoFrames));
    }

    #[test]
    fn a_clip_sampled_on_another_grid_is_refused() {
        let mut message = configured(&load(antenna_doc("wiggle", 3)));
        let clip = message.validate_mut().expect("a written clip validates");
        clip.frame_rate_hz = FLOOR_TICK_HZ * 2.0;
        assert_eq!(
            refused(ClipView::new(clip)),
            Err(ClipViewError::FrameRate {
                frame_rate_hz: FLOOR_TICK_HZ * 2.0
            })
        );
    }

    /// A zeroed message is the case that has to be refused: a slot or a file
    /// nothing wrote must not read as a playable clip.
    #[test]
    fn a_zeroed_message_is_refused() {
        let message = Box::new(ClipConfigWire::new());
        assert_eq!(
            refused(ClipView::new(valid(&message))),
            Err(ClipViewError::NoChannels),
            "an all-zero mask is no channels, which is where a zeroed clip dies"
        );
    }

    #[test]
    fn a_non_finite_value_on_a_masked_channel_is_refused() {
        for (key, set) in [
            (
                "antenna_right_d",
                (|frame, value| frame.antenna_right_d = value) as fn(&mut ClipFrame, f64),
            ),
            ("antenna_left_d", |frame, value| {
                frame.antenna_left_d = value;
            }),
        ] {
            let mut message = configured(&load(antenna_doc("wiggle", 3)));
            let clip = message.validate_mut().expect("a written clip validates");
            set(clip.frames.get_mut(1).expect("three frames"), f64::NAN);
            // Compared by parts rather than as a whole: the payload worth
            // refusing here is a NaN, which is equal to nothing, itself
            // included.
            let error = ClipView::new(clip)
                .err()
                .unwrap_or_else(|| panic!("{key} was accepted as a non-finite number"));
            let ClipViewError::NonFinite {
                frame,
                key: named,
                value,
            } = error
            else {
                panic!("{key} was refused as {error} rather than as a non-finite number");
            };
            assert_eq!((frame, named), (1, key));
            assert!(value.is_nan(), "{key} refused with {value}");
        }
    }

    /// A rotation that is not a unit quaternion would be renormalised into some
    /// other rotation than the one written, so it is refused instead.
    #[test]
    fn a_non_unit_rotation_is_refused() {
        let mut message = configured(&load(all_channels_doc("walk", 3)));
        let clip = message.validate_mut().expect("a written clip validates");
        clip.frames.get_mut(2).expect("three frames").quat_w = 2.0;
        let error = ClipView::new(clip).expect_err("a non-unit rotation is refused");
        assert!(
            matches!(error, ClipViewError::Quaternion { frame: 2, .. }),
            "{error}"
        );
    }

    /// A non-finite coordinate is caught as one rather than as a rotation whose
    /// norm cannot be compared: `NaN.partial_cmp` answers nothing, so the
    /// finiteness check has to come first.
    #[test]
    fn a_non_finite_rotation_coordinate_is_refused_as_non_finite() {
        let mut message = configured(&load(all_channels_doc("walk", 3)));
        let clip = message.validate_mut().expect("a written clip validates");
        clip.frames.get_mut(0).expect("three frames").quat_x = f64::INFINITY;
        assert_eq!(
            refused(ClipView::new(clip)),
            Err(ClipViewError::NonFinite {
                frame: 0,
                key: "quat_x",
                value: f64::INFINITY,
            })
        );
    }

    /// An asset that drives a channel it does not claim is wrong about itself,
    /// and playing it would move a joint no consumer of the mask expects to
    /// move.
    #[test]
    fn a_value_on_an_unmasked_channel_is_refused() {
        let mut message = configured(&load(antenna_doc("wiggle", 3)));
        let clip = message.validate_mut().expect("a written clip validates");
        clip.frames.get_mut(1).expect("three frames").body_yaw_d = 0.5;
        assert_eq!(
            refused(ClipView::new(clip)),
            Err(ClipViewError::StrayChannel {
                frame: 1,
                key: "body_yaw_d",
                channel: Channel::BodyYaw,
                value: 0.5,
            })
        );
    }
    /// A clip document of `frames` antennas-only frames, as JSON, for the cases
    /// that need a whole library rather than one loaded clip.
    fn clip_json(name: &str, frames: usize) -> String {
        let track: Vec<String> = (0..frames)
            .map(|index| format!("{{\"antennas\": [{}, 0.0]}}", 0.01 * index as f64))
            .collect();
        format!(
            r#"{{"version": 1, "kind": "clip", "name": "{name}",
                 "channels": ["antennas"], "frame_hz": {FLOOR_TICK_HZ},
                 "max_speed": 2.0, "frames": [{}]}}"#,
            track.join(",")
        )
    }

    /// A sequence document from raw entry JSON.
    fn sequence_json(name: &str, entries: &str) -> String {
        format!(r#"{{"version": 1, "kind": "sequence", "name": "{name}", "entries": [{entries}]}}"#)
    }

    /// Load `documents` and require that nothing was skipped.
    fn loaded_library(documents: &[(&str, String)]) -> crate::library::Library {
        let (library, skips) = crate::library::Library::load(
            documents
                .iter()
                .map(|(source, text)| (*source, text.as_str())),
            &limits(),
        );
        assert!(skips.is_empty(), "unexpected skips: {skips:?}");
        library
    }

    /// The loaded clips and motions in the library's own read order, which is
    /// the numbering the message carries.
    fn numbered(library: &crate::library::Library) -> (Vec<&Clip>, Vec<&Motion>) {
        (
            library
                .loaded()
                .map(|asset| library.clip(&asset.name).expect("a loaded clip"))
                .collect(),
            library
                .motions_loaded()
                .iter()
                .map(|asset| library.motion(&asset.name).expect("a loaded motion"))
                .collect(),
        )
    }

    /// A flattened motion crosses as what it is: a lead gap, then a segment per
    /// clip carrying the clip's id, the speed the nesting produced, and the hold
    /// that follows. Nothing of the reference graph survives the crossing.
    #[test]
    fn a_motion_crosses_as_its_lead_gap_and_its_segments() {
        let library = loaded_library(&[
            ("a.json", clip_json("pod/a", 4)),
            ("b.json", clip_json("pod/b", 6)),
            (
                "z.json",
                sequence_json(
                    "pod/greeting",
                    r#"{"gap_ms": 250}, {"ref": "pod/a"}, {"gap_ms": 300}, {"ref": "pod/b", "speed": 2.0}"#,
                ),
            ),
        ]);
        let (clips, motions) = numbered(&library);
        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&clips, &motions, message.clear_valid()).expect("the library fits");

        let written = valid_library(&message);
        assert_eq!(
            written.motions.capacity(),
            MAX_MOTIONS,
            "the constant and the schema disagree about how many motions fit"
        );

        // A clip is a one-segment motion at 1.0x with no holds anywhere.
        let bare = written.motions.get(0).expect("the first motion");
        assert_eq!(bare.lead_gap_ms, 0);
        assert_eq!(bare.segments.len(), 1);
        assert_eq!(bare.segments.get(0).expect("one segment").clip_id, 0);
        assert_eq!(bare.segments.get(0).expect("one segment").speed, 1.0);
        assert_eq!(
            bare.segments.capacity(),
            MAX_SEGMENTS,
            "the constant and the schema disagree about how many segments fit"
        );

        let composed = written.motions.get(2).expect("the sequence's motion");
        assert_eq!(composed.lead_gap_ms, 250);
        assert_eq!(composed.segments.len(), 2);
        let first = composed.segments.get(0).expect("two segments");
        assert_eq!(
            (first.clip_id, first.speed, first.gap_after_ms),
            (0, 1.0, 300)
        );
        let second = composed.segments.get(1).expect("two segments");
        assert_eq!(
            (second.clip_id, second.speed, second.gap_after_ms),
            (1, 2.0, 0)
        );
    }

    /// A gap landing between two milliseconds rounds to the nearest one.
    #[test]
    fn a_hold_between_two_clips_lands_on_the_nearest_millisecond() {
        let library = loaded_library(&[
            ("a.json", clip_json("pod/a", 4)),
            (
                "i.json",
                sequence_json(
                    "pod/inner",
                    r#"{"ref": "pod/a"}, {"gap_ms": 100}, {"ref": "pod/a"}"#,
                ),
            ),
            (
                "z.json",
                sequence_json("pod/outer", r#"{"ref": "pod/inner", "speed": 1.5}"#),
            ),
        ]);
        let (clips, motions) = numbered(&library);
        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&clips, &motions, message.clear_valid()).expect("the library fits");

        // 100 ms under a 1.5x nesting is 66.66… ms, which rounds up.
        let outer = valid_library(&message)
            .motions
            .get(2)
            .expect("the outer motion");
        assert_eq!(
            outer.segments.get(0).expect("two segments").gap_after_ms,
            67
        );
    }

    /// A head-and-yaw clip, for the mask-union case.
    fn head_clip_json(name: &str) -> String {
        format!(
            r#"{{"version": 1, "kind": "clip", "name": "{name}",
                 "channels": ["head", "body_yaw"], "frame_hz": {FLOOR_TICK_HZ},
                 "max_speed": 2.0,
                 "frames": [{{"dt": [0.0, 0.0, 0.0], "dq": [1.0, 0.0, 0.0, 0.0],
                              "body_yaw": 0.1}}]}}"#
        )
    }

    /// The library of the crossing case above, as a message a motion can be
    /// taken out of: two antennas-only clips and a sequence composing them.
    fn composed_message() -> Box<ClipLibraryConfigWire> {
        let library = loaded_library(&[
            ("a.json", clip_json("pod/a", 4)),
            ("b.json", clip_json("pod/b", 6)),
            (
                "z.json",
                sequence_json(
                    "pod/greeting",
                    r#"{"gap_ms": 250}, {"ref": "pod/a"}, {"gap_ms": 300}, {"ref": "pod/b", "speed": 2.0}"#,
                ),
            ),
        ]);
        let (clips, motions) = numbered(&library);
        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&clips, &motions, message.clear_valid()).expect("the library fits");
        message
    }

    /// What a motion is like to play is derived from the clips its segments
    /// name, every time: the asset carries the walk and nothing about it.
    #[test]
    fn a_motion_says_what_playing_it_is_like() {
        let message = composed_message();
        let library = valid_library(&message);
        let checked = ValidatedLibrary::of(library).expect("the composed library plays");
        assert_eq!(checked.motions(), 3);

        let motion = checked.playable_motion(2).expect("the composed motion");
        assert_eq!(motion.segments(), 2);
        assert!((motion.lead_gap_s() - 0.25).abs() < 1e-12);
        // 250 ms of lead, four frames at 1.0x, 300 ms of hold, six frames at
        // 2.0x: the holds count and the clips scale.
        assert!(
            (motion.duration_s() - 0.69).abs() < 1e-12,
            "{}",
            motion.duration_s()
        );
        for channel in Channel::ALL {
            assert_eq!(
                motion.mask().contains(channel),
                channel == Channel::Antennas,
                "{channel}"
            );
        }

        let first = motion.segment(0);
        assert_eq!(first.clip.frames(), 4);
        assert_eq!(first.speed, 1.0);
        assert!((first.gap_after_s - 0.3).abs() < 1e-12);
        let second = motion.segment(1);
        assert_eq!(second.clip.frames(), 6);
        assert_eq!(second.speed, 2.0);
        assert_eq!(second.gap_after_s, 0.0);

        // The edges of the whole walk, not of one clip: the ramps come from the
        // first and last segments' clips, which here are different lengths and
        // so carry different default ramps.
        let (a, b) = (
            library.clips.get(0).expect("two clips"),
            library.clips.get(1).expect("two clips"),
        );
        assert_ne!(
            a.blend_in_ms, b.blend_out_ms,
            "the case does not discriminate"
        );
        assert_eq!(motion.blend_in_ms(), a.blend_in_ms);
        assert_eq!(motion.blend_out_ms(), b.blend_out_ms);
    }

    /// A channel is driven by the motion even if only one segment touches it:
    /// between such segments it contributes no delta, which is not the same as
    /// the motion having nothing to say about it.
    #[test]
    fn a_motions_mask_is_the_union_of_its_clips() {
        let library = loaded_library(&[
            ("a.json", clip_json("pod/a", 4)),
            ("h.json", head_clip_json("pod/h")),
            (
                "z.json",
                sequence_json("pod/both", r#"{"ref": "pod/a"}, {"ref": "pod/h"}"#),
            ),
        ]);
        let (clips, motions) = numbered(&library);
        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&clips, &motions, message.clear_valid()).expect("the library fits");

        let checked = ValidatedLibrary::of(valid_library(&message)).expect("both clips play");
        let mask = checked
            .playable_motion(2)
            .expect("the composed motion")
            .mask();
        for channel in Channel::ALL {
            assert!(mask.contains(channel), "{channel} is not in the union");
        }
    }

    /// A motion's ceiling is the tightest clip's after the nesting has already
    /// spent its share, and never below 1.0x — every segment was proven legal
    /// at 1.0x when the sequence resolved.
    #[test]
    fn a_motions_ceiling_is_the_tightest_clip_after_the_nesting_spends_it() {
        let message = composed_message();
        let checked = ValidatedLibrary::of(valid_library(&message)).expect("the library plays");

        // A bare clip is the clip's own ceiling.
        let bare = checked.playable_motion(0).expect("the first clip's motion");
        let clip = checked.playable(0).expect("the first clip");
        assert!((bare.max_speed() - clip.max_speed()).abs() < 1e-12);

        // The composed motion plays its second clip at 2.0x already, which is
        // all that clip admits, so the motion admits 1.0x and no more.
        let composed = checked.playable_motion(2).expect("the composed motion");
        assert!(
            (composed.max_speed() - 1.0).abs() < 1e-9,
            "{}",
            composed.max_speed()
        );
    }

    /// The cap: a segment the nesting left below 1.0x divides its clip's
    /// ceiling back up, and the global bound is what stops the motion
    /// advertising a speed no clip was authored for.
    #[test]
    fn a_motions_ceiling_never_rises_above_the_global_bound() {
        let library = loaded_library(&[
            ("a.json", clip_json("pod/a", 4)),
            (
                "z.json",
                sequence_json("pod/slow", r#"{"ref": "pod/a", "speed": 0.5}"#),
            ),
        ]);
        let (clips, motions) = numbered(&library);
        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&clips, &motions, message.clear_valid()).expect("the library fits");
        let checked = ValidatedLibrary::of(valid_library(&message)).expect("the library plays");

        // The clip admits 2.0x and the segment spends half of it, so the
        // quotient is 4.0x — twice what any clip may be invoked at.
        let clip = checked.playable(0).expect("the first clip");
        assert!((clip.max_speed() - MAX_SPEED).abs() < 1e-12);
        let motion = checked.playable_motion(1).expect("the slowed motion");
        assert!(
            (motion.max_speed() - MAX_SPEED).abs() < 1e-12,
            "{}",
            motion.max_speed()
        );
    }

    /// The floor: a nesting product an ulp above a clip's ceiling is admitted by
    /// `SPEED_EPS` and divides back out an ulp below one. Without the floor the
    /// default invocation of a motion the emitter accepted would be refused.
    #[test]
    fn a_motion_the_walk_accepted_is_always_playable_at_one_times() {
        let mut message = composed_message();
        let ceiling = valid_library(&message)
            .clips
            .get(1)
            .expect("two clips")
            .max_speed;
        message
            .validate_mut()
            .expect("a written library validates")
            .motions
            .get_mut(2)
            .expect("three motions")
            .segments
            .get_mut(1)
            .expect("two segments")
            .speed = ceiling + SPEED_EPS / 2.0;

        let checked = ValidatedLibrary::of(valid_library(&message))
            .expect("a segment an ulp over its clip's ceiling is arithmetic, not motion");
        let motion = checked.playable_motion(2).expect("the composed motion");
        assert!(motion.max_speed() >= 1.0, "{}", motion.max_speed());
    }

    /// A motion with no segments would play nothing at all, which is the same
    /// refusal the loader makes of a sequence that flattens to no clips.
    #[test]
    fn a_motion_with_no_segments_is_refused() {
        let mut message = composed_message();
        message
            .validate_mut()
            .expect("a written library validates")
            .motions
            .get_mut(2)
            .expect("three motions")
            .segments
            .clear();

        assert_eq!(
            ValidatedLibrary::of(valid_library(&message)).expect_err("an empty motion is refused"),
            UnplayableAsset::Motion {
                motion_id: 2,
                source: MotionViewError::NoSegments,
            }
        );
    }

    /// A segment naming a clip the message does not carry: the walk says which
    /// segment of which motion, because a clip id on its own does not.
    #[test]
    fn a_segment_naming_a_clip_the_library_does_not_have_is_refused() {
        let mut message = composed_message();
        message
            .validate_mut()
            .expect("a written library validates")
            .motions
            .get_mut(2)
            .expect("three motions")
            .segments
            .get_mut(1)
            .expect("two segments")
            .clip_id = 7;

        assert_eq!(
            ValidatedLibrary::of(valid_library(&message)).expect_err("clip 7 is not there"),
            UnplayableAsset::Motion {
                motion_id: 2,
                source: MotionViewError::SegmentClip {
                    segment: 1,
                    clip_id: 7,
                    source: ClipViewError::UnknownClip {
                        clip_id: 7,
                        clips: 2,
                    },
                },
            }
        );
    }

    /// A flattened speed past what its clip admits, by more than arithmetic.
    ///
    /// The clip's ceiling is lowered rather than the speed raised past the
    /// global bound, so what refuses is the clip's own limit and not the bound
    /// every clip shares.
    #[test]
    fn a_segment_past_its_clips_ceiling_is_refused() {
        let mut message = composed_message();
        let ceiling = 1.5;
        {
            let library = message.validate_mut().expect("a written library validates");
            library.clips.get_mut(0).expect("two clips").max_speed = ceiling;
            library
                .motions
                .get_mut(2)
                .expect("three motions")
                .segments
                .get_mut(0)
                .expect("two segments")
                .speed = ceiling + 0.001;
        }

        assert_eq!(
            ValidatedLibrary::of(valid_library(&message)).expect_err("past the clip's ceiling"),
            UnplayableAsset::Motion {
                motion_id: 2,
                source: MotionViewError::SegmentPastCeiling {
                    segment: 0,
                    speed: ceiling + 0.001,
                    ceiling,
                },
            }
        );
    }

    /// A flattened speed outside the bounds anything may be played at, before
    /// an invocation asks for anything.
    #[test]
    fn a_segment_speed_outside_the_global_bounds_is_refused() {
        for speed in [MIN_SPEED / 2.0, MAX_SPEED + 0.001, f64::NAN, 0.0] {
            let mut message = composed_message();
            message
                .validate_mut()
                .expect("a written library validates")
                .motions
                .get_mut(2)
                .expect("three motions")
                .segments
                .get_mut(0)
                .expect("two segments")
                .speed = speed;

            let refusal = ValidatedLibrary::of(valid_library(&message))
                .expect_err("a speed outside the bounds is refused");
            // Compared by parts rather than as a whole: one of the payloads is
            // a NaN, which is equal to nothing, itself included.
            let UnplayableAsset::Motion {
                motion_id,
                source: MotionViewError::SegmentSpeed { segment, .. },
            } = refusal
            else {
                panic!("{speed} was refused as {refusal}");
            };
            assert_eq!((motion_id, segment), (2, 0));
        }
    }

    /// The bounds' own endpoints are speeds, and a flattening that lands on one
    /// is a motion rather than a refusal.
    #[test]
    fn the_global_speed_bounds_admit_their_own_endpoints() {
        for speed in [MIN_SPEED, MAX_SPEED] {
            let mut message = composed_message();
            message
                .validate_mut()
                .expect("a written library validates")
                .motions
                .get_mut(2)
                .expect("three motions")
                .segments
                .get_mut(0)
                .expect("two segments")
                .speed = speed;

            let checked = ValidatedLibrary::of(valid_library(&message))
                .unwrap_or_else(|refusal| panic!("{speed}x is a speed: {refusal}"));
            assert_eq!(
                checked
                    .playable_motion(2)
                    .expect("the motion")
                    .segment(0)
                    .speed,
                speed
            );
        }
    }

    /// A motion id nothing carries is refused rather than answered with the
    /// first motion, which is the same refusal a clip id gets.
    #[test]
    fn a_motion_id_the_library_does_not_have_is_refused() {
        let message = composed_message();
        let checked = ValidatedLibrary::of(valid_library(&message)).expect("the library plays");
        assert_eq!(
            checked
                .playable_motion(3)
                .expect_err("there is no fourth motion"),
            MotionViewError::UnknownMotion {
                motion_id: 3,
                motions: 3,
            }
        );
    }

    /// A hold the field cannot state at all is refused rather than wrapped: a
    /// gap slowed past the range of a millisecond count is a motion the message
    /// has no way to carry.
    #[test]
    fn a_hold_past_the_range_of_the_field_is_refused() {
        let library = loaded_library(&[
            ("a.json", clip_json("pod/a", 4)),
            (
                "i.json",
                sequence_json(
                    "pod/inner",
                    &format!(r#"{{"gap_ms": {}}}, {{"ref": "pod/a"}}"#, u32::MAX),
                ),
            ),
            (
                "z.json",
                sequence_json("pod/outer", r#"{"ref": "pod/inner", "speed": 0.25}"#),
            ),
        ]);
        let (clips, motions) = numbered(&library);
        let mut message = Box::new(ClipLibraryConfigWire::new());
        let refusal = write_library(&clips, &motions, message.clear_valid())
            .expect_err("a gap four times the range of the field");
        assert!(
            matches!(refusal, LibraryWriteError::GapOutOfRange { motion: 2, .. }),
            "{refusal:?}"
        );
        let written = valid_library(&message);
        assert!(
            written.clips.is_empty() && written.motions.is_empty(),
            "a refused write left a table behind"
        );
    }

    /// A library carrying more motions than the message holds is refused by the
    /// bound that is actually about motions, and leaves the message empty. Both
    /// halves matter: a truncation would renumber every motion a schedule names.
    #[test]
    fn a_motion_table_past_capacity_is_refused() {
        let mut documents = vec![("a.json", clip_json("pod/a", 4))];
        let sequences: Vec<(String, String)> = (0..MAX_MOTIONS)
            .map(|index| {
                (
                    format!("s{index:02}.json"),
                    sequence_json(&format!("pod/s{index:02}"), r#"{"ref": "pod/a"}"#),
                )
            })
            .collect();
        documents.extend(sequences.iter().map(|(s, d)| (s.as_str(), d.clone())));
        let library = loaded_library(&documents);
        let (clips, motions) = numbered(&library);
        assert_eq!(motions.len(), MAX_MOTIONS + 1);

        let mut message = Box::new(ClipLibraryConfigWire::new());
        assert_eq!(
            write_library(&clips, &motions, message.clear_valid()),
            Err(LibraryWriteError::TooManyMotions {
                motions: MAX_MOTIONS + 1
            })
        );
        let written = valid_library(&message);
        assert!(
            written.clips.is_empty() && written.motions.is_empty(),
            "a refused write left a table behind"
        );
    }

    /// The motion table that fills the message exactly is accepted: the last
    /// slot is one the emitter may write, and it must not be the one that
    /// panics.
    #[test]
    fn a_motion_table_that_fills_the_message_is_accepted() {
        let mut documents = vec![("a.json", clip_json("pod/a", 4))];
        let sequences: Vec<(String, String)> = (0..MAX_MOTIONS - 1)
            .map(|index| {
                (
                    format!("s{index:02}.json"),
                    sequence_json(&format!("pod/s{index:02}"), r#"{"ref": "pod/a"}"#),
                )
            })
            .collect();
        documents.extend(sequences.iter().map(|(s, d)| (s.as_str(), d.clone())));
        let library = loaded_library(&documents);
        let (clips, motions) = numbered(&library);
        assert_eq!(motions.len(), MAX_MOTIONS);

        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&clips, &motions, message.clear_valid()).expect("a full motion table fits");
        assert_eq!(valid_library(&message).motions.len(), MAX_MOTIONS);
    }

    /// A motion of exactly as many segments as the message holds crosses whole:
    /// the last segment is one an author may write, and what it says about its
    /// clip and its hold survives the write.
    #[test]
    fn a_motion_that_fills_the_segment_array_crosses_intact() {
        let mut entries: Vec<String> = (0..MAX_SEGMENTS - 1)
            .map(|_| r#"{"ref": "pod/a"}"#.to_owned())
            .collect();
        entries.push(r#"{"gap_ms": 150}"#.to_owned());
        entries.push(r#"{"ref": "pod/b"}"#.to_owned());
        let library = loaded_library(&[
            ("a.json", clip_json("pod/a", 4)),
            ("b.json", clip_json("pod/b", 4)),
            ("z.json", sequence_json("pod/long", &entries.join(", "))),
        ]);
        let (clips, motions) = numbered(&library);
        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&clips, &motions, message.clear_valid()).expect("a full motion fits");

        let written = valid_library(&message);
        let long = written.motions.get(2).expect("the sequence's motion");
        assert_eq!(long.segments.len(), MAX_SEGMENTS);
        let held = long
            .segments
            .get(MAX_SEGMENTS - 2)
            .expect("the segment the hold follows");
        assert_eq!((held.clip_id, held.gap_after_ms), (0, 150));
        let last = long
            .segments
            .get(MAX_SEGMENTS - 1)
            .expect("the last segment");
        assert_eq!((last.clip_id, last.speed, last.gap_after_ms), (1, 1.0, 0));
    }

    /// A configuration an earlier build emitted has no speed ceiling, so the
    /// field it never wrote reads as zero — and a zero ceiling is refused for
    /// the whole library rather than played as "no speed at all". This is the
    /// loud, typed refusal of a stale asset the changelog promises; nothing
    /// about it is a partial load.
    #[test]
    fn a_clip_carrying_no_speed_ceiling_refuses_the_whole_library() {
        let clip = load(antenna_doc("stale", 2));
        let mut message = Box::new(ClipLibraryConfigWire::new());
        write_library(&[&clip, &clip], &[], message.clear_valid()).expect("two clips fit");
        message
            .validate_mut()
            .expect("a written library validates")
            .clips
            .get_mut(1)
            .expect("the library has two clips")
            .max_speed = 0.0;

        let refusal = ValidatedLibrary::of(valid_library(&message))
            .expect_err("a clip with no ceiling is not playable");
        assert_eq!(
            refusal,
            UnplayableAsset::Clip {
                clip_id: 1,
                source: ClipViewError::SpeedCeiling { max_speed: 0.0 }
            }
        );
    }

    /// The two halves of one emit have to agree about what the library holds: a
    /// segment naming a clip that is not being written is a caller pairing a
    /// motion table with the wrong clips.
    #[test]
    fn a_segment_naming_a_clip_the_message_does_not_carry_is_refused() {
        let library = loaded_library(&[("a.json", clip_json("pod/a", 4))]);
        let (_, motions) = numbered(&library);
        let mut message = Box::new(ClipLibraryConfigWire::new());
        assert_eq!(
            write_library(&[], &motions, message.clear_valid()),
            Err(LibraryWriteError::SegmentClipMissing {
                motion: 0,
                segment: 0
            })
        );
        let written = valid_library(&message);
        assert!(
            written.clips.is_empty() && written.motions.is_empty(),
            "a refused write left a table behind"
        );
    }
}
