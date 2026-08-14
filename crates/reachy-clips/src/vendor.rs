//! The vendor's recorded-move format, and what it takes to become one of ours.
//!
//! Pollen distributes emotes and dances as JSON recordings:
//! `{description, time[], set_target_data[]}`, where each frame carries a 4x4
//! head pose in the **world** frame with the neutral head height subtracted
//! from z, antenna angles `[right, left]` in radians, and an absolute
//! `body_yaw`. Timestamps are whatever the recording loop ran at, and nothing
//! in that format is versioned or validated.
//!
//! What comes out the other side is one of our clips: per-channel **deltas**
//! against the neutral reference, uniformly sampled at the tick rate, masked to
//! the channels the recording actually drives. The conversion is where the two
//! frame conventions are reconciled — the vendor's head pose is world-frame and
//! yaw-independent while ours rides on the yawing body — and where a file that
//! is not what it claims to be is refused rather than silently played.
//!
//! Everything here is pure: text in, document out. The reading, the writing and
//! the directory walk belong to the importer binary.

use std::collections::BTreeMap;

use nalgebra::{Isometry3, Matrix3, Rotation3, Translation3, UnitQuaternion};
use reachy_kin::baked::HEAD_Z_OFFSET;
use reachy_kin::{neutral_head_pose, world_to_body};
use reachy_motion::FLOOR_TICK_HZ;
use serde::Deserialize;
use serde_json::Value;
use thiserror::Error;

use crate::compose::{interpolate_pose, lerp};
use crate::format::{
    Channel, ChannelMask, Clip, ClipDoc, ClipError, DEFAULT_BLEND_MS, DeltaFrame, FORMAT_VERSION,
    FrameDoc, MAX_SPEED, validate_name,
};
use crate::speed::ClipLimits;

/// How far a recorded rotation block may sit from orthonormal and still be
/// read as a rotation.
///
/// JSON-serialised rotation matrices drift: the vendor writes what its solver
/// produced, through Python's float formatting, and nothing on either side
/// renormalises. What this tolerance separates is that drift — parts in a
/// billion — from a matrix that is not a rotation at all, which is a file we
/// have misread rather than a file that lost precision.
pub const ROTATION_TOL: f64 = 1e-6;

/// How far a channel's values may spread and still be called constant.
///
/// A recording that never moved its antennas should probably not pin them, and
/// the operator wants to hear about it. The threshold is loose enough that a
/// pair sitting still through a hundred frames of encoder noise reads as still,
/// and tight enough that a deliberate millimetre or milliradian does not.
pub const CONSTANT_TOL: f64 = 1e-4;

/// A recorded move as the vendor writes it.
///
/// `description` is a required field in the format — a file without one is not
/// a valid recording. Unknown top-level keys are kept rather than refused: we
/// want to know what else the datasets carry.
#[derive(Clone, Debug, Deserialize)]
pub struct VendorMove {
    /// Free text, carried through to the clip.
    pub description: String,
    /// One timestamp per frame, seconds, non-decreasing.
    pub time: Vec<f64>,
    /// One frame per timestamp.
    pub set_target_data: Vec<VendorFrame>,
    /// Every top-level key this reader does not know.
    #[serde(flatten)]
    pub extra: BTreeMap<String, Value>,
}

/// One recorded frame.
///
/// `head` and `antennas` are required fields; `body_yaw` is optional. The
/// difference is load-bearing: a file whose frames never state a yaw is a
/// recording that does not drive the channel, and our mask says so rather than
/// pinning it to zero.
#[derive(Clone, Debug, Deserialize)]
pub struct VendorFrame {
    /// The head pose, world frame, neutral height subtracted from z, as a 4x4
    /// row-major homogeneous transform.
    pub head: [[f64; 4]; 4],
    /// Antenna angles, right then left, radians, absolute.
    pub antennas: [f64; 2],
    /// Body yaw, radians, absolute. Absent in recordings that never turned.
    #[serde(default)]
    pub body_yaw: Option<f64>,
    /// Every per-frame key this reader does not know.
    #[serde(flatten)]
    pub extra: BTreeMap<String, Value>,
}

/// Why a vendor file could not become a clip.
///
/// Every one of these refuses the whole file and names what it was: nothing is
/// clamped, trimmed or dropped to make a recording load. A refused file is
/// content we do not have, which is a fact for the report, rather than content
/// we have quietly altered.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum ImportError {
    /// The file is not the JSON the vendor's own reader would take.
    #[error("not a recorded move: {detail}")]
    Malformed {
        /// What the parser said.
        detail: String,
    },

    /// The two parallel arrays disagree.
    #[error("{times} timestamps against {frames} frames")]
    LengthMismatch {
        /// How many timestamps.
        times: usize,
        /// How many frames.
        frames: usize,
    },

    /// A recording with no frames at all.
    #[error("the recording has no frames")]
    Empty,

    /// A timestamp that goes backwards, which would silently mis-order
    /// playback.
    #[error("timestamp {frame} goes backwards: {at} after {previous}")]
    NonMonotonic {
        /// The frame whose timestamp went back.
        frame: usize,
        /// What it said.
        at: f64,
        /// What the frame before it said.
        previous: f64,
    },

    /// A number no arithmetic can use.
    #[error("frame {frame} {key} is not finite")]
    NonFinite {
        /// Which frame, or the timestamp array's index.
        frame: usize,
        /// Which quantity.
        key: &'static str,
    },

    /// A head matrix whose rotation block is not a rotation.
    #[error("frame {frame} head rotation is {detail}, further than {ROTATION_TOL} from a rotation")]
    Rotation {
        /// Which frame.
        frame: usize,
        /// What was wrong with it.
        detail: String,
    },

    /// A head matrix whose last row is not an affine transform's.
    #[error("frame {frame} head is not an affine transform: last row {row:?}")]
    NotAffine {
        /// Which frame.
        frame: usize,
        /// What the file said.
        row: [f64; 4],
    },

    /// The operator asked for a mask the recording cannot fill.
    #[error("the recording states no {channel}, so it cannot be masked")]
    ChannelAbsent {
        /// The channel asked for.
        channel: Channel,
    },

    /// The operator's mask drops a channel the recording actually moves.
    ///
    /// The override exists for a channel a recording states and never touches;
    /// dropping one that moves is a different motion, and for body yaw it is a
    /// wrong one: the head deltas are expressed in the body frame at each
    /// frame's own yaw, so a clip that keeps the head and drops a turning yaw
    /// keeps the counter-rotation for a turn it no longer performs.
    #[error("the recording moves its {channel}, so dropping it would change the motion")]
    ChannelMoves {
        /// The channel the override dropped.
        channel: Channel,
    },

    /// The name the file's stem and the prefix make is not a library name.
    #[error("{name} is not a usable asset name: {detail}")]
    Name {
        /// The name that was built.
        name: String,
        /// Why it was refused.
        detail: String,
    },

    /// The converted clip does not load — it leaves the envelope over the
    /// neutral base it was recorded against, or its frames admit no speed.
    #[error("the converted clip does not load: {source}")]
    Unloadable {
        /// The loader's own refusal.
        source: ClipError,
    },

    /// A clip our stack cannot play at the speed it was recorded at.
    ///
    /// Not content: a recording whose own frames step further per tick than the
    /// machine's bounds allow at 1.0× would have to be slowed below its own
    /// pace to play at all, and a motion played slower than it was performed is
    /// a different motion.
    #[error("the clip admits at most {max_speed}x, below its own recorded pace")]
    TooFast {
        /// The derived ceiling.
        max_speed: f64,
    },
}

/// What the operator asked of one conversion.
#[derive(Clone, Debug, Default)]
pub struct ImportOptions {
    /// The mask to write, overriding the one the recording implies. `None`
    /// derives it.
    pub channels: Option<ChannelMask>,
}

/// A converted recording, and everything the report says about it.
#[derive(Clone, Debug)]
pub struct Import {
    /// The loaded clip: the conversion's whole result, and what the numbers
    /// below were derived from. The document to write comes off it through
    /// [`Import::doc`], so there is one copy of the frame track rather than two
    /// that could disagree.
    pub clip: Clip,
    /// Masked channels whose delta never changes across the whole track.
    pub constant: Vec<Channel>,
    /// Every key in the file this reader does not know, top-level keys bare and
    /// per-frame keys prefixed `set_target_data.`.
    pub unknown_keys: Vec<String>,
    /// How many frames the recording carried, before resampling.
    pub source_frames: usize,
    /// How long the recording ran, seconds, as its timestamps state it.
    pub source_duration_s: f64,
}

impl Import {
    /// The clip document to write: the derived ceiling and the floored ramps,
    /// as the loader settled them.
    pub fn doc(&self) -> ClipDoc {
        self.clip.to_doc()
    }
}

/// One frame's quantities in the vendor's own conventions, ready to resample.
#[derive(Clone, Copy, Debug)]
struct Sample {
    /// The head pose in the world frame, at its true height.
    head_world: Isometry3<f64>,
    /// Antenna angles, right then left.
    antennas: [f64; 2],
    /// Body yaw.
    body_yaw: f64,
}

/// Convert one vendor recording into a clip document named `name`.
///
/// Resampling happens in the vendor's frame, before the delta conversion:
/// interpolating deltas across a turning body would corrupt the motion.
///
/// The loader is the validator: it is the same code the daemon runs, so a file
/// this accepts is a file that loads, and import-time and load-time validation
/// cannot drift.
pub fn convert(
    json: &str,
    name: &str,
    limits: &ClipLimits,
    options: &ImportOptions,
) -> Result<Import, ImportError> {
    let move_doc: VendorMove =
        serde_json::from_str(json).map_err(|err| ImportError::Malformed {
            detail: err.to_string(),
        })?;
    validate_name(name).map_err(|source| ImportError::Name {
        name: name.to_owned(),
        detail: source.to_string(),
    })?;

    let times = rebased_times(&move_doc)?;
    let samples = samples(&move_doc)?;
    let mask = mask_for(&move_doc, &samples, options)?;
    let resampled = resample(&times, &samples);
    let frames: Vec<DeltaFrame> = resampled.iter().map(|s| delta(s, mask)).collect();
    let constant = constant_channels(&frames, mask);

    // Placeholders: the loader re-derives ceiling and ramps from the frames
    // alone, so computing them here would double the cost for numbers it
    // recomputes anyway. The written document comes from the loaded clip.
    let asked = ClipDoc {
        version: FORMAT_VERSION,
        kind: "clip".to_owned(),
        name: name.to_owned(),
        description: Some(move_doc.description.clone()),
        channels: Channel::ALL
            .into_iter()
            .filter(|channel| mask.contains(*channel))
            .collect(),
        frame_hz: FLOOR_TICK_HZ,
        max_speed: MAX_SPEED,
        blend_in_ms: Some(DEFAULT_BLEND_MS),
        blend_out_ms: Some(DEFAULT_BLEND_MS),
        frames: frames.iter().map(frame_doc).collect(),
    };
    let mut clip =
        Clip::from_doc(asked, limits).map_err(|source| ImportError::Unloadable { source })?;
    clip.forget_notes();
    if clip.max_speed() < 1.0 {
        return Err(ImportError::TooFast {
            max_speed: clip.max_speed(),
        });
    }

    Ok(Import {
        clip,
        constant,
        unknown_keys: unknown_keys(&move_doc),
        source_frames: samples.len(),
        source_duration_s: times.last().copied().unwrap_or(0.0),
    })
}

/// The recording's timestamps, shifted so the first is zero.
///
/// Published files are 0-based in practice, but capture writes epoch seconds,
/// so the rebase is done here rather than assumed.
fn rebased_times(doc: &VendorMove) -> Result<Vec<f64>, ImportError> {
    if doc.time.len() != doc.set_target_data.len() {
        return Err(ImportError::LengthMismatch {
            times: doc.time.len(),
            frames: doc.set_target_data.len(),
        });
    }
    let Some(first) = doc.time.first().copied() else {
        return Err(ImportError::Empty);
    };
    for (frame, at) in doc.time.iter().enumerate() {
        if !at.is_finite() {
            return Err(ImportError::NonFinite { frame, key: "time" });
        }
    }
    let mut times = Vec::with_capacity(doc.time.len());
    let mut previous = first;
    for (frame, at) in doc.time.iter().enumerate() {
        if *at < previous {
            return Err(ImportError::NonMonotonic {
                frame,
                at: *at,
                previous,
            });
        }
        previous = *at;
        times.push(at - first);
    }
    Ok(times)
}

/// Every frame in the vendor's conventions, with its head pose lifted back to
/// the world frame.
///
/// Their matrices carry the neutral head height subtracted from z and nothing
/// else — identity is the neutral pose — so recovering the pose is adding the
/// one constant back. Both stacks hold the same number, from the same machine.
fn samples(doc: &VendorMove) -> Result<Vec<Sample>, ImportError> {
    let mut samples = Vec::with_capacity(doc.set_target_data.len());
    for (index, frame) in doc.set_target_data.iter().enumerate() {
        let body_yaw = frame.body_yaw.unwrap_or(0.0);
        for (value, key) in [
            (body_yaw, "body_yaw"),
            (frame.antennas[0], "antennas"),
            (frame.antennas[1], "antennas"),
        ] {
            if !value.is_finite() {
                return Err(ImportError::NonFinite { frame: index, key });
            }
        }
        let mut head = rotation(index, &frame.head)?;
        head.translation.vector.z += HEAD_Z_OFFSET;
        samples.push(Sample {
            head_world: head,
            antennas: frame.antennas,
            body_yaw,
        });
    }
    Ok(samples)
}

/// One 4x4 as an isometry, refusing anything that is not one.
///
/// The rotation block is checked against orthonormality rather than trusted:
/// a matrix that has drifted is renormalised through a quaternion, and a matrix
/// that is not a rotation — a reflection, a scale, a transposed convention —
/// means we have misread the file, which is worth a refusal and not a silent
/// nearest fit.
fn rotation(index: usize, head: &[[f64; 4]; 4]) -> Result<Isometry3<f64>, ImportError> {
    for row in head {
        for value in row {
            if !value.is_finite() {
                return Err(ImportError::NonFinite {
                    frame: index,
                    key: "head",
                });
            }
        }
    }
    if head[3] != [0.0, 0.0, 0.0, 1.0] {
        return Err(ImportError::NotAffine {
            frame: index,
            row: head[3],
        });
    }
    let matrix = Matrix3::from_fn(|row, col| head[row][col]);
    let drift = (matrix.transpose() * matrix - Matrix3::identity())
        .abs()
        .max();
    if drift > ROTATION_TOL {
        return Err(ImportError::Rotation {
            frame: index,
            detail: format!("{drift} off orthonormal"),
        });
    }
    if (matrix.determinant() - 1.0).abs() > ROTATION_TOL {
        return Err(ImportError::Rotation {
            frame: index,
            detail: format!("a determinant of {}", matrix.determinant()),
        });
    }
    let quaternion =
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(matrix));
    Ok(Isometry3::from_parts(
        Translation3::new(head[0][3], head[1][3], head[2][3]),
        quaternion,
    ))
}

/// The channels the clip drives.
///
/// The recording's own answer is derived: head and antennas are required keys,
/// so they are always driven; body yaw is masked only if some frame states it.
/// An operator override may drop channels — an emote whose antennas never move
/// should not pin them — but never add one the recording does not carry, and
/// never one the recording moves: that is a silently different motion, which is
/// the one thing this importer refuses on principle.
fn mask_for(
    doc: &VendorMove,
    samples: &[Sample],
    options: &ImportOptions,
) -> Result<ChannelMask, ImportError> {
    let mut derived = ChannelMask::empty();
    derived.insert(Channel::Head);
    derived.insert(Channel::Antennas);
    if doc
        .set_target_data
        .iter()
        .any(|frame| frame.body_yaw.is_some())
    {
        derived.insert(Channel::BodyYaw);
    }
    let Some(asked) = options.channels else {
        return Ok(derived);
    };
    for channel in Channel::ALL {
        if asked.contains(channel) && !derived.contains(channel) {
            return Err(ImportError::ChannelAbsent { channel });
        }
        if derived.contains(channel) && !asked.contains(channel) && moves(samples, channel) {
            return Err(ImportError::ChannelMoves { channel });
        }
    }
    Ok(asked)
}

/// Whether a recording's own samples move a channel at all.
///
/// The same [`CONSTANT_TOL`] spread the report calls a channel still by, asked
/// of the vendor's quantities rather than of the converted deltas: a dropped
/// channel has no deltas to ask about.
fn moves(samples: &[Sample], channel: Channel) -> bool {
    let Some(first) = samples.first() else {
        return false;
    };
    samples.iter().any(|sample| match channel {
        Channel::Head => {
            (first.head_world.translation.vector - sample.head_world.translation.vector)
                .abs()
                .max()
                > CONSTANT_TOL
                || first
                    .head_world
                    .rotation
                    .angle_to(&sample.head_world.rotation)
                    > CONSTANT_TOL
        }
        Channel::Antennas => {
            (0..2).any(|side| (first.antennas[side] - sample.antennas[side]).abs() > CONSTANT_TOL)
        }
        Channel::BodyYaw => (first.body_yaw - sample.body_yaw).abs() > CONSTANT_TOL,
    })
}

/// The recording's quantities on our uniform tick grid.
///
/// One frame per tick, the first on the recording's own zero and the last on or
/// just past its end, so a track of `n` frames plays for `n` ticks. Between the
/// recording's samples the head pose walks the geodesic and everything else
/// lerps; the resampling changes when the motion is sampled, never what it is.
fn resample(times: &[f64], samples: &[Sample]) -> Vec<Sample> {
    let period = 1.0 / FLOOR_TICK_HZ;
    let span = times.last().copied().unwrap_or(0.0);
    let ticks = (span / period).round() as usize + 1;
    let mut out = Vec::with_capacity(ticks);
    // The window the previous sample time fell in, carried forward: the grid
    // only ever moves forward, so the search is a walk and never a scan.
    let mut window = 0usize;
    for tick in 0..ticks {
        let at = tick as f64 * period;
        while window + 2 < times.len() && times[window + 1] <= at {
            window += 1;
        }
        let (before, after) = (
            &samples[window],
            &samples[(window + 1).min(samples.len() - 1)],
        );
        let (start, end) = (times[window], times[(window + 1).min(times.len() - 1)]);
        let alpha = if end > start {
            ((at - start) / (end - start)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        out.push(Sample {
            head_world: interpolate_pose(&before.head_world, &after.head_world, alpha),
            antennas: [
                lerp(before.antennas[0], after.antennas[0], alpha),
                lerp(before.antennas[1], after.antennas[1], alpha),
            ],
            body_yaw: lerp(before.body_yaw, after.body_yaw, alpha),
        });
    }
    out
}

/// One resampled frame as a masked delta against the neutral reference.
///
/// The head takes two steps and both are forced. The vendor's pose is
/// world-frame and independent of the body's yaw, while ours rides on the
/// yawing body, so the pose is first re-expressed in the body frame at that
/// frame's own yaw — skip that and every recorded turn drags the head around
/// with it, which is not what the recording did. What is left is the delta
/// against neutral, so a recording that does nothing stores zeros.
fn delta(sample: &Sample, mask: ChannelMask) -> DeltaFrame {
    let head = mask.contains(Channel::Head).then(|| {
        let body = world_to_body(&sample.head_world, sample.body_yaw);
        neutral_head_pose().inverse() * body
    });
    DeltaFrame {
        head,
        antennas: mask.contains(Channel::Antennas).then_some(sample.antennas),
        body_yaw: mask.contains(Channel::BodyYaw).then_some(sample.body_yaw),
    }
}

/// [`DeltaFrame`]'s own document form is private to the format module, and
/// deliberately so — nothing but a load should be building frames from the
/// outside. The importer is the one exception, and it goes through the same
/// keys the loader reads back.
fn frame_doc(frame: &DeltaFrame) -> FrameDoc {
    let (dt, dq) = match frame.head {
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
        antennas: frame.antennas,
        body_yaw: frame.body_yaw,
    }
}

/// Masked channels that never move across the whole track.
///
/// Reported, never acted on. A clip that masks a channel it holds constant
/// pins that channel at its recorded value for the whole playback and forbids
/// anything else from driving it — which is occasionally what an author wants
/// and much more often a recording that simply never touched it.
fn constant_channels(frames: &[DeltaFrame], mask: ChannelMask) -> Vec<Channel> {
    let Some(first) = frames.first() else {
        return Vec::new();
    };
    let mut constant = Vec::new();
    for channel in Channel::ALL {
        if !mask.contains(channel) {
            continue;
        }
        let still = frames.iter().all(|frame| match channel {
            Channel::Head => {
                let (a, b) = (
                    first.head.unwrap_or_else(Isometry3::identity),
                    frame.head.unwrap_or_else(Isometry3::identity),
                );
                (a.translation.vector - b.translation.vector).abs().max() <= CONSTANT_TOL
                    && a.rotation.angle_to(&b.rotation) <= CONSTANT_TOL
            }
            Channel::Antennas => {
                let (a, b) = (
                    first.antennas.unwrap_or_default(),
                    frame.antennas.unwrap_or_default(),
                );
                (a[0] - b[0]).abs() <= CONSTANT_TOL && (a[1] - b[1]).abs() <= CONSTANT_TOL
            }
            Channel::BodyYaw => {
                (first.body_yaw.unwrap_or_default() - frame.body_yaw.unwrap_or_default()).abs()
                    <= CONSTANT_TOL
            }
        });
        if still {
            constant.push(channel);
        }
    }
    constant
}

/// Every key in the file this reader does not know.
///
/// An unknown key is news about the datasets, not a reason to refuse content.
fn unknown_keys(doc: &VendorMove) -> Vec<String> {
    let mut keys: Vec<String> = doc.extra.keys().cloned().collect();
    let mut per_frame: Vec<String> = doc
        .set_target_data
        .iter()
        .flat_map(|frame| frame.extra.keys())
        .map(|key| format!("set_target_data.{key}"))
        .collect();
    per_frame.sort();
    per_frame.dedup();
    keys.extend(per_frame);
    keys
}

#[cfg(test)]
mod tests {
    use super::*;

    use nalgebra::Vector3;
    use serde_json::json;

    /// A 4x4 identity, which is the vendor's spelling of the neutral head pose.
    fn identity() -> Vec<Vec<f64>> {
        (0..4)
            .map(|row| (0..4).map(|col| f64::from(u8::from(row == col))).collect())
            .collect()
    }

    /// The identity lifted by `dz` metres, still level.
    fn lifted(dz: f64) -> Vec<Vec<f64>> {
        let mut head = identity();
        head[2][3] = dz;
        head
    }

    /// A recording of `frames`, evenly spaced at `period` seconds from zero.
    fn recording(period: f64, frames: Vec<Value>) -> String {
        let times: Vec<f64> = (0..frames.len()).map(|i| i as f64 * period).collect();
        json!({
            "description": "a test recording",
            "time": times,
            "set_target_data": frames,
        })
        .to_string()
    }

    /// One frame in the vendor's shape.
    fn frame(head: Vec<Vec<f64>>, antennas: [f64; 2], body_yaw: Option<f64>) -> Value {
        let mut value = json!({ "head": head, "antennas": antennas });
        if let Some(yaw) = body_yaw {
            value["body_yaw"] = json!(yaw);
        }
        value
    }

    /// The vendor's own minimal fixture — two identity frames — converts, and
    /// what it produces is a clip of zero deltas.
    ///
    /// The one literal instance of the format anywhere in their repositories,
    /// and the anchor for the convention: identity is *neutral*, not a
    /// degenerate value, so it must come out as the delta that does nothing.
    #[test]
    fn the_vendors_minimal_fixture_converts_to_a_clip_of_no_motion() {
        let json = recording(
            0.1,
            vec![
                frame(identity(), [0.0, 0.0], Some(0.0)),
                frame(identity(), [0.0, 0.0], Some(0.0)),
            ],
        );
        let import = convert(
            &json,
            "pollen/test/minimal",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect("the vendor's own fixture converts");

        assert_eq!(import.doc().name, "pollen/test/minimal");
        assert_eq!(
            import.doc().description.as_deref(),
            Some("a test recording")
        );
        assert_eq!(import.doc().frame_hz, FLOOR_TICK_HZ);
        assert_eq!(import.source_frames, 2);
        // 0.1 s at 50 Hz is five periods, plus the frame that lands on the end.
        assert_eq!(import.doc().frames.len(), 6);
        for frame in import.clip.frames() {
            assert_eq!(frame.head, Some(Isometry3::identity()), "{frame:?}");
            assert_eq!(frame.antennas, Some([0.0, 0.0]));
            assert_eq!(frame.body_yaw, Some(0.0));
        }
        assert_eq!(
            import.constant,
            vec![Channel::Head, Channel::BodyYaw, Channel::Antennas],
            "nothing in it moves, and the report says so of every channel",
        );
    }

    /// A pure vertical lift in the vendor's frame becomes a pure translation
    /// delta, and nothing else.
    #[test]
    fn a_vendor_z_lift_becomes_a_pure_translation_delta() {
        let json = recording(
            0.1,
            vec![
                frame(identity(), [0.0, 0.0], None),
                frame(lifted(0.01), [0.0, 0.0], None),
            ],
        );
        let import = convert(
            &json,
            "pollen/test/lift",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect("a centimetre of lift is inside the envelope");

        let last = import.clip.frames().last().expect("frames");
        let head = last.head.expect("the head is masked");
        assert!(
            (head.translation.vector - Vector3::new(0.0, 0.0, 0.01))
                .abs()
                .max()
                < 1e-9,
            "{:?}",
            head.translation.vector
        );
        assert!(
            head.rotation.angle() < 1e-12,
            "no rotation: {:?}",
            head.rotation
        );
        assert!(
            !import.doc().channels.contains(&Channel::BodyYaw),
            "no frame states a yaw, so the channel is not driven: {:?}",
            import.doc().channels
        );
    }

    /// A recorded body turn under a world-stationary head yields a yaw delta
    /// and a head delta that counter-rotates by the same angle.
    ///
    /// The vendor's head pose is world-frame and yaw-independent, so their
    /// recording of "the body pivots under the head" carries an unchanged head
    /// matrix. Ours rides on the body: expressing the same instant in the body
    /// frame is what puts the counter-rotation in, and skipping it would make
    /// every recorded turn drag the head around with it.
    #[test]
    fn a_recorded_body_turn_counter_rotates_the_head_delta() {
        let turn = 0.2;
        let json = recording(
            0.5,
            vec![
                frame(identity(), [0.0, 0.0], Some(0.0)),
                frame(identity(), [0.0, 0.0], Some(turn)),
            ],
        );
        let import = convert(
            &json,
            "pollen/test/turn",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect("a fifth of a radian is well inside the yaw limits");

        let last = import.clip.frames().last().expect("frames");
        assert_eq!(last.body_yaw, Some(turn));
        let head = last.head.expect("the head is masked");
        let axis = head.rotation.scaled_axis();
        assert!(
            (axis - Vector3::new(0.0, 0.0, -turn)).abs().max() < 1e-9,
            "the head delta is the body's turn, negated: {axis:?}",
        );
        assert!(
            head.translation.vector.abs().max() < 1e-9,
            "a turn about the yaw axis moves the head origin nowhere: {:?}",
            head.translation.vector,
        );
    }

    /// Antenna angles pass through as deltas, since the neutral pair is zero.
    #[test]
    fn antenna_angles_pass_through_as_deltas_right_then_left() {
        let json = recording(
            0.2,
            vec![
                frame(identity(), [0.0, 0.0], None),
                frame(identity(), [0.3, -0.2], None),
            ],
        );
        let import = convert(
            &json,
            "pollen/test/wave",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect("the pair is inside its goal range");

        let last = import.clip.frames().last().expect("frames");
        let antennas = last.antennas.expect("the antennas are masked");
        assert!((antennas[0] - 0.3).abs() < 1e-12, "{antennas:?}");
        assert!((antennas[1] + 0.2).abs() < 1e-12, "{antennas:?}");
        assert_eq!(
            import.constant,
            vec![Channel::Head],
            "the head never moved and the antennas did",
        );
    }

    /// Non-uniform timestamps are resampled onto the tick grid, and what the
    /// grid reads between two recorded frames is the interpolation.
    #[test]
    fn a_non_uniform_recording_lands_on_the_tick_grid() {
        let json = json!({
            "description": "uneven",
            "time": [0.0, 0.02, 0.08],
            "set_target_data": [
                frame(identity(), [0.0, 0.0], None),
                frame(identity(), [0.1, 0.0], None),
                frame(identity(), [0.4, 0.0], None),
            ],
        })
        .to_string();
        let import = convert(
            &json,
            "pollen/test/uneven",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect("an uneven recording converts");

        assert_eq!(import.source_frames, 3);
        assert_eq!(
            import.doc().frames.len(),
            5,
            "0.08 s is four periods plus the end"
        );
        let right: Vec<f64> = import
            .clip
            .frames()
            .iter()
            .map(|frame| frame.antennas.expect("masked")[0])
            .collect();
        // 0, 0.02 (the recorded corner), then thirds of the way through the
        // 0.1 → 0.4 leg at 0.04, 0.06 and 0.08.
        let expected = [0.0, 0.1, 0.2, 0.3, 0.4];
        for (got, want) in right.iter().zip(expected) {
            assert!((got - want).abs() < 1e-9, "{right:?} against {expected:?}");
        }
    }

    /// Every structural refusal in the format, each named for what it was.
    #[test]
    fn a_recording_that_is_not_one_is_refused_by_name() {
        let convert = |json: &str| {
            convert(
                json,
                "pollen/test/x",
                &ClipLimits::default(),
                &ImportOptions::default(),
            )
            .expect_err("refused")
        };
        let mismatched = json!({
            "description": "d",
            "time": [0.0, 0.1, 0.2],
            "set_target_data": [frame(identity(), [0.0, 0.0], None)],
        })
        .to_string();
        assert_eq!(
            convert(&mismatched),
            ImportError::LengthMismatch {
                times: 3,
                frames: 1
            },
        );

        let empty = json!({ "description": "d", "time": [], "set_target_data": [] }).to_string();
        assert_eq!(convert(&empty), ImportError::Empty);

        let backwards = json!({
            "description": "d",
            "time": [0.0, 0.2, 0.1],
            "set_target_data": [
                frame(identity(), [0.0, 0.0], None),
                frame(identity(), [0.0, 0.0], None),
                frame(identity(), [0.0, 0.0], None),
            ],
        })
        .to_string();
        assert!(matches!(
            convert(&backwards),
            ImportError::NonMonotonic { frame: 2, .. }
        ));

        let mut scaled = identity();
        scaled[0][0] = 1.5;
        let stretched = recording(
            0.1,
            vec![
                frame(identity(), [0.0, 0.0], None),
                frame(scaled, [0.0, 0.0], None),
            ],
        );
        assert!(matches!(
            convert(&stretched),
            ImportError::Rotation { frame: 1, .. }
        ));

        // Orthonormal and still not a rotation: a reflection has determinant
        // −1 and passes the drift check untouched. Left in, it would convert a
        // dataset written under a mirrored convention into a motion that plays
        // the recording backwards on one axis — the head nodding the right way
        // and turning the wrong one.
        let mut mirrored = identity();
        mirrored[0][0] = -1.0;
        let reflected = recording(0.1, vec![frame(mirrored, [0.0, 0.0], None)]);
        let refused = convert(&reflected);
        let ImportError::Rotation {
            frame: at,
            detail: why,
        } = &refused
        else {
            panic!("expected a rotation refusal: {refused:?}");
        };
        assert_eq!(*at, 0);
        assert!(why.contains("determinant"), "{why}");

        let mut skewed = identity();
        skewed[3] = vec![0.0, 0.0, 1.0, 1.0];
        let projective = recording(0.1, vec![frame(skewed, [0.0, 0.0], None)]);
        assert!(matches!(
            convert(&projective),
            ImportError::NotAffine { frame: 0, .. }
        ));

        let missing = json!({ "description": "d", "time": [0.0] }).to_string();
        assert!(matches!(convert(&missing), ImportError::Malformed { .. }));

        let unnamed = json!({
            "time": [0.0],
            "set_target_data": [frame(identity(), [0.0, 0.0], None)],
        })
        .to_string();
        assert!(
            matches!(convert(&unnamed), ImportError::Malformed { .. }),
            "their own reader requires a description too",
        );
    }

    /// A drifted rotation — the parts-per-billion a JSON round trip costs —
    /// is renormalised rather than refused.
    #[test]
    fn a_rotation_that_only_drifted_is_renormalised() {
        let mut drifted = identity();
        drifted[0][0] = 1.0 + ROTATION_TOL / 4.0;
        let json = recording(0.1, vec![frame(drifted, [0.0, 0.0], None)]);
        let import = convert(
            &json,
            "pollen/test/drift",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect("drift is not a wrong matrix");
        let head = import.clip.frames()[0].head.expect("masked");
        assert!(head.rotation.angle() < 1e-6, "{:?}", head.rotation);
    }

    /// A name the library would not take is refused before any conversion work.
    #[test]
    fn a_name_the_library_would_refuse_is_refused_here() {
        let json = recording(0.1, vec![frame(identity(), [0.0, 0.0], None)]);
        let refused = convert(
            &json,
            "Pollen/Test",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect_err("upper case is not in the charset");
        assert!(matches!(refused, ImportError::Name { .. }), "{refused:?}");
    }

    /// The mask override drops a channel the recording carries and holds still,
    /// and refuses to invent one it does not.
    #[test]
    fn the_mask_override_may_drop_a_channel_but_never_add_one() {
        let json = recording(
            0.2,
            vec![
                frame(identity(), [0.0, 0.0], None),
                frame(identity(), [0.0, 0.0], None),
            ],
        );
        let head_only = ImportOptions {
            channels: Some(ChannelMask::of(Channel::Head)),
        };
        let import = convert(
            &json,
            "pollen/test/head",
            &ClipLimits::default(),
            &head_only,
        )
        .expect("dropping the antennas is the operator's call");
        assert_eq!(import.doc().channels, vec![Channel::Head]);
        for frame in &import.doc().frames {
            assert_eq!(
                frame.antennas, None,
                "a dropped channel is removed, not zeroed"
            );
            assert_eq!(frame.body_yaw, None);
        }

        let mut asked = ChannelMask::of(Channel::Head);
        asked.insert(Channel::BodyYaw);
        let refused = convert(
            &json,
            "pollen/test/head",
            &ClipLimits::default(),
            &ImportOptions {
                channels: Some(asked),
            },
        )
        .expect_err("the recording states no yaw");
        assert_eq!(
            refused,
            ImportError::ChannelAbsent {
                channel: Channel::BodyYaw
            }
        );
    }

    /// Dropping a channel that moves is not a mask choice, it is a different
    /// motion — and for body yaw a wrong one, since the head deltas carry the
    /// counter-rotation of a turn the clip would no longer perform.
    #[test]
    fn the_mask_override_will_not_drop_a_channel_that_moves() {
        let turning = recording(
            0.2,
            vec![
                frame(identity(), [0.0, 0.0], Some(0.0)),
                frame(identity(), [0.0, 0.0], Some(0.4)),
            ],
        );
        let mut asked = ChannelMask::of(Channel::Head);
        asked.insert(Channel::Antennas);
        let refused = convert(
            &turning,
            "pollen/test/turn",
            &ClipLimits::default(),
            &ImportOptions {
                channels: Some(asked),
            },
        )
        .expect_err("the recording turns");
        assert_eq!(
            refused,
            ImportError::ChannelMoves {
                channel: Channel::BodyYaw
            }
        );

        let waving = recording(
            0.2,
            vec![
                frame(identity(), [0.0, 0.0], None),
                frame(identity(), [0.3, -0.2], None),
            ],
        );
        let refused = convert(
            &waving,
            "pollen/test/wave",
            &ClipLimits::default(),
            &ImportOptions {
                channels: Some(ChannelMask::of(Channel::Head)),
            },
        )
        .expect_err("the recording waves");
        assert_eq!(
            refused,
            ImportError::ChannelMoves {
                channel: Channel::Antennas
            }
        );
    }

    /// Keys neither reader knows are reported and carried past — the opposite
    /// of the vendor's silence, which is how an inert `check_collision` flag
    /// came to sit in the data with nothing saying so.
    #[test]
    fn keys_we_do_not_read_are_reported_and_not_refused() {
        let mut first = frame(identity(), [0.0, 0.0], None);
        first["check_collision"] = json!(false);
        let times = [0.0, 0.1];
        let json = json!({
            "description": "d",
            "time": times,
            "set_target_data": [first, frame(identity(), [0.0, 0.0], None)],
            "recorded_by": "marionette",
        })
        .to_string();
        let import = convert(
            &json,
            "pollen/test/extra",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect("an unknown key is news, not a refusal");
        assert_eq!(
            import.unknown_keys,
            vec![
                "recorded_by".to_owned(),
                "set_target_data.check_collision".to_owned()
            ],
        );
    }

    /// A recording whose frames step further per tick than the machine allows
    /// is refused rather than written out unplayable.
    #[test]
    fn a_recording_too_fast_for_its_own_pace_is_refused() {
        // Six tenths of a radian of antenna in one recorded period, past the
        // per-tick step the machine allows once the derivation's margin is
        // taken off it.
        let json = recording(
            0.02,
            vec![
                frame(identity(), [0.0, 0.0], None),
                frame(identity(), [0.6, 0.0], None),
            ],
        );
        let refused = convert(
            &json,
            "pollen/test/snap",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect_err("nothing plays that");
        assert!(
            matches!(
                refused,
                ImportError::TooFast { .. } | ImportError::Unloadable { .. }
            ),
            "{refused:?}",
        );
    }

    /// A frame outside the envelope over the neutral base refuses the file,
    /// naming the frame and what it failed.
    #[test]
    fn a_frame_outside_the_envelope_refuses_the_file() {
        // A head pose a long way off the yaw axis: no linkage solution, let
        // alone one inside the cone.
        let json = recording(
            2.0,
            vec![
                frame(identity(), [0.0, 0.0], None),
                frame(lifted(0.5), [0.0, 0.0], None),
            ],
        );
        let refused = convert(
            &json,
            "pollen/test/moon",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect_err("half a metre of lift is not a pose this machine holds");
        assert!(
            matches!(refused, ImportError::Unloadable { .. }),
            "{refused:?}"
        );
    }

    /// The written document is the one the loader reads back: the max speed and
    /// the blend ramps in the file are the derived ones, not the placeholders.
    #[test]
    fn the_written_document_carries_what_the_loader_derived() {
        // Fast enough that the derivation lands under the global ceiling and
        // above the default ramps: a track the placeholders happened to fit
        // would prove nothing about which numbers were written.
        let json = recording(
            0.04,
            vec![
                frame(identity(), [0.0, 0.0], None),
                frame(identity(), [0.7, -0.7], None),
            ],
        );
        let import = convert(
            &json,
            "pollen/test/round",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect("converts");
        assert!(
            import.doc().max_speed < MAX_SPEED,
            "the fixture derives a ceiling below the placeholder, or this proves nothing",
        );
        assert_eq!(import.doc().max_speed, import.clip.max_speed());
        assert_eq!(import.doc().blend_in_ms, Some(import.clip.blend_in_ms()));
        assert_eq!(import.doc().blend_out_ms, Some(import.clip.blend_out_ms()));

        let text = serde_json::to_string(&import.doc()).expect("renders");
        let reloaded = Clip::from_json(&text, &ClipLimits::default()).expect("loads back");
        assert_eq!(reloaded, import.clip);
        assert!(
            reloaded.notes().is_empty(),
            "the file needs no correction on the way back in: {:?}",
            reloaded.notes(),
        );
    }
}
