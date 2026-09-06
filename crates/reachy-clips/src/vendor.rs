//! The vendor's recorded-move format, and what it takes to become one of ours.
//!
//! Pollen distributes emotes and dances as JSON recordings:
//! `{description, time[], set_target_data[]}`, where each frame carries a 4x4
//! head pose in the **world** frame with the neutral head height subtracted
//! from z, antenna angles `[right, left]` in radians, and an absolute
//! `body_yaw`. Timestamps are whatever the recording loop ran at, every number
//! is written to six decimal places, and nothing in that format is versioned or
//! validated.
//!
//! What comes out the other side is one of our clips: per-channel **deltas**
//! against the neutral reference, uniformly sampled at the tick rate, masked to
//! the channels the recording actually moves. The conversion is where the two
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
use crate::envelope::ClipLimits;
use crate::format::{
    Channel, ChannelMask, Clip, ClipDoc, ClipError, DeltaFrame, FORMAT_VERSION, FrameDoc,
    validate_name,
};

/// How far a recorded rotation block may sit from orthonormal and still be
/// read as a rotation.
///
/// Two populations sit below this, and both are drift rather than content.
/// Every number in the published recordings is written to **six decimal
/// places**, so each entry carries up to 5e-7 of rounding and `RᵀR − I` reaches
/// ~3e-6 on quantisation alone; the recordings measure 0.9–1.7e-6. A second
/// population, four files, measures 5.1–6.1e-4 — a uniform quarter-permille
/// stretch, as if one recorder composed rotations with an unnormalised
/// quaternion — whose polar factor is under 0.02° of attitude away, less than
/// the mechanism can express.
///
/// What the check is for is a matrix that is not a rotation at all, meaning a
/// file we have misread: a reflection sits at `|det − 1| = 2`, a translation
/// block read into the rotation is off by whole units, and a scale worth
/// noticing is ≥1e-2. Nothing real lives between 6.1e-4 and 1e-2.
pub const ROTATION_TOL: f64 = 1e-3;

/// The orthonormality error above which a conversion's drift is worth saying
/// out loud.
///
/// Above anything six-decimal rounding can produce, so the report names the few
/// recordings whose matrices were stretched by their recorder and stays quiet
/// about the hundred that merely lost decimals.
pub const ROTATION_DRIFT_NOTED: f64 = 1e-5;

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
    /// Present in vendor recordings but unused by this converter. Declared so
    /// it does not land in `extra` as an unknown key.
    #[serde(default)]
    pub check_collision: Option<bool>,
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
    /// A channel a recording states and never touches is already dropped by
    /// default, so an override that drops one is asking for something else:
    /// dropping a channel that moves is a different motion, and for body yaw it
    /// is a wrong one — the head deltas are expressed in the body frame at each
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
    /// The channels the recording states and never moves, whether or not they
    /// are masked: the dropped ones say why the clip drives less than the file
    /// mentions, and a masked one says the clip pins a channel that stood still.
    pub constant: Vec<Channel>,
    /// The largest orthonormality error any frame's rotation block carried,
    /// measured before renormalisation.
    pub rotation_drift: f64,
    /// Every key in the file this reader does not know, top-level keys bare and
    /// per-frame keys prefixed `set_target_data.`.
    pub unknown_keys: Vec<String>,
    /// How many frames the recording carried, before resampling.
    pub source_frames: usize,
    /// How long the recording ran, seconds, as its timestamps state it.
    pub source_duration_s: f64,
}

impl Import {
    /// The clip document to write, with the ramps as the loader settled them.
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
/// cannot drift. A recording is refused for a frame the envelope refuses over
/// the neutral base or an antenna angle no goal register holds, and for nothing
/// about its speed — how fast a recording moves is a property of the recording,
/// not a claim this machine judges.
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
    let (samples, rotation_drift) = samples(&move_doc)?;
    let stated = stated_channels(&move_doc);
    let moving = moving_channels(&samples);
    let mask = mask_for(stated, moving, options)?;
    let resampled = resample(&times, &samples);
    let frames: Vec<DeltaFrame> = resampled.iter().map(|s| delta(s, mask)).collect();
    let constant: Vec<Channel> = Channel::ALL
        .into_iter()
        .filter(|channel| stated.contains(*channel) && !moving.contains(*channel))
        .collect();

    // The ramps are left unsaid rather than set to the default, because a
    // recording states no blend intent and a stated ramp longer than the clip is
    // refused; omitted, the default is capped at the clip's own length instead.
    // The written document comes from the loaded clip.
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
        blend_in_ms: None,
        blend_out_ms: None,
        frames: frames.iter().map(frame_doc).collect(),
    };
    let mut clip =
        Clip::from_doc(asked, limits).map_err(|source| ImportError::Unloadable { source })?;
    clip.forget_notes();

    Ok(Import {
        clip,
        constant,
        rotation_drift,
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
/// the world frame, and the largest orthonormality error any of them carried.
///
/// Their matrices carry the neutral head height subtracted from z and nothing
/// else — identity is the neutral pose — so recovering the pose is adding the
/// one constant back. Both stacks hold the same number, from the same machine.
fn samples(doc: &VendorMove) -> Result<(Vec<Sample>, f64), ImportError> {
    let mut samples = Vec::with_capacity(doc.set_target_data.len());
    let mut worst_drift = 0.0f64;
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
        let (mut head, drift) = rotation(index, &frame.head)?;
        worst_drift = worst_drift.max(drift);
        head.translation.vector.z += HEAD_Z_OFFSET;
        samples.push(Sample {
            head_world: head,
            antennas: frame.antennas,
            body_yaw,
        });
    }
    Ok((samples, worst_drift))
}

/// One 4x4 as an isometry and how far its rotation block sat from orthonormal,
/// refusing anything that is not an isometry at all.
///
/// The rotation block is checked against orthonormality rather than trusted.
/// What survives the check is projected onto the nearest rotation — nalgebra's
/// iterative polar extraction, then a quaternion off that, so the `dq` written
/// out is unit to floating precision rather than to luck. Six-decimal
/// quantisation is what most of the drift is; the rest is one recorder's
/// unnormalised composition, and both are inside what the polar factor
/// answers. A matrix that is not a rotation — a reflection, a real scale, a
/// transposed convention — means we have misread the file, which is worth a
/// refusal and not a silent nearest fit.
fn rotation(index: usize, head: &[[f64; 4]; 4]) -> Result<(Isometry3<f64>, f64), ImportError> {
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
    let quaternion = UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix(&matrix));
    Ok((
        Isometry3::from_parts(
            Translation3::new(head[0][3], head[1][3], head[2][3]),
            quaternion,
        ),
        drift,
    ))
}

/// The channels the recording mentions at all.
///
/// Head and antennas are required keys, so every recording states them; body
/// yaw is stated only if some frame carries it. Stating a channel is what makes
/// it maskable — it says nothing about whether the recording drives it.
fn stated_channels(doc: &VendorMove) -> ChannelMask {
    let mut stated = ChannelMask::empty();
    stated.insert(Channel::Head);
    stated.insert(Channel::Antennas);
    if doc
        .set_target_data
        .iter()
        .any(|frame| frame.body_yaw.is_some())
    {
        stated.insert(Channel::BodyYaw);
    }
    stated
}

/// The channels the clip drives.
///
/// The default is what the recording states **and moves**. The recorder writes
/// every channel whether or not the puppeteer touched it, so presence carries
/// no intent, while a masked channel is pinned for the whole playback: a clip
/// holding the antennas at the vendor's rest angle for three seconds yanks the
/// live base's antenna posture to that angle and back. Pinning a still channel
/// on purpose is the rare case, and it has a spelling.
///
/// An operator override may name any stated channel, moving or still — that is
/// the spelling — but never one the recording does not carry, and never one it
/// moves: dropping a moving channel is a silently different motion, which is
/// the one thing this importer refuses on principle.
fn mask_for(
    stated: ChannelMask,
    moving: ChannelMask,
    options: &ImportOptions,
) -> Result<ChannelMask, ImportError> {
    let Some(asked) = options.channels else {
        return Ok(moving);
    };
    for channel in Channel::ALL {
        if asked.contains(channel) && !stated.contains(channel) {
            return Err(ImportError::ChannelAbsent { channel });
        }
        if moving.contains(channel) && !asked.contains(channel) {
            return Err(ImportError::ChannelMoves { channel });
        }
    }
    Ok(asked)
}

/// The channels a recording's own samples actually move, by the [`CONSTANT_TOL`]
/// spread the report calls a channel still by.
///
/// Asked of the vendor's quantities rather than of the converted deltas, since
/// a dropped channel has no deltas to ask about — with one correction. The head
/// is measured **in the body frame**, which is the frame the clip stores it in:
/// a recording whose head stands still in the world while the body turns under
/// it moves its head relative to the body every frame, and dropping that
/// channel would strand the counter-rotation and drag the head around with the
/// turn. A moving channel is a subset of the stated ones: an absent body yaw
/// reads as a constant zero and moves nothing.
fn moving_channels(samples: &[Sample]) -> ChannelMask {
    let mut moving = ChannelMask::empty();
    let Some(first) = samples.first() else {
        return moving;
    };
    let body = |sample: &Sample| world_to_body(&sample.head_world, sample.body_yaw);
    let first_head = body(first);
    for sample in samples {
        let head = body(sample);
        if (first_head.translation.vector - head.translation.vector)
            .abs()
            .max()
            > CONSTANT_TOL
            || first_head.rotation.angle_to(&head.rotation) > CONSTANT_TOL
        {
            moving.insert(Channel::Head);
        }
        if (0..2).any(|side| (first.antennas[side] - sample.antennas[side]).abs() > CONSTANT_TOL) {
            moving.insert(Channel::Antennas);
        }
        if (first.body_yaw - sample.body_yaw).abs() > CONSTANT_TOL {
            moving.insert(Channel::BodyYaw);
        }
    }
    moving
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

    /// The vendor's own minimal fixture — two identity frames — converts under
    /// an explicit mask, and what it produces is a clip of zero deltas.
    ///
    /// The one literal instance of the format anywhere in their repositories,
    /// and the anchor for the convention: identity is *neutral*, not a
    /// degenerate value, so it must come out as the delta that does nothing. It
    /// moves nothing at all, so every channel has to be asked for by name.
    #[test]
    fn the_vendors_minimal_fixture_converts_to_a_clip_of_no_motion() {
        let json = recording(
            0.1,
            vec![
                frame(identity(), [0.0, 0.0], Some(0.0)),
                frame(identity(), [0.0, 0.0], Some(0.0)),
            ],
        );
        let mut everything = ChannelMask::empty();
        for channel in Channel::ALL {
            everything.insert(channel);
        }
        let import = convert(
            &json,
            "pollen/test/minimal",
            &ClipLimits::default(),
            &ImportOptions {
                channels: Some(everything),
            },
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

    /// A recording that moves nothing at all drives nothing, and a clip that
    /// drives nothing is not a clip.
    ///
    /// The refusal comes from the loader rather than from here: an empty mask is
    /// what "states three channels and touches none of them" derives to, and
    /// what an operator wants from such a file — pin it anyway — is spelled with
    /// `--channels`.
    #[test]
    fn a_recording_that_moves_nothing_drives_nothing() {
        let json = recording(
            0.1,
            vec![
                frame(identity(), [0.0, 0.0], Some(0.0)),
                frame(identity(), [0.0, 0.0], Some(0.0)),
            ],
        );
        let refused = convert(
            &json,
            "pollen/test/inert",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect_err("nothing moves, so nothing is masked");
        assert!(
            matches!(
                refused,
                ImportError::Unloadable {
                    source: ClipError::NoChannels
                }
            ),
            "{refused:?}",
        );
    }

    /// The default mask is what the recording moves, not what it mentions: a
    /// stated channel that stands still is dropped rather than pinned, and named
    /// as constant so the report can say so.
    #[test]
    fn the_default_mask_drops_a_stated_channel_that_never_moves() {
        let json = recording(
            0.1,
            vec![
                frame(identity(), [0.0, 0.0], Some(0.0)),
                frame(identity(), [0.2, -0.1], Some(0.0)),
            ],
        );
        let import = convert(
            &json,
            "pollen/test/still",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect("the antennas move");
        assert_eq!(import.doc().channels, vec![Channel::Antennas]);
        assert_eq!(import.constant, vec![Channel::Head, Channel::BodyYaw]);
        for frame in &import.doc().frames {
            assert_eq!(frame.body_yaw, None, "a still yaw is not pinned");
            assert_eq!(frame.dt, None);
        }
    }

    /// A recording whose body yaw stands still somewhere other than zero: the
    /// clip drives no yaw, and the head keeps the pose relative to the body that
    /// the puppeteer set it at.
    ///
    /// What is dropped is the body's constant world-frame offset, which a clip
    /// has no claim to — it is a delta over whatever base the session is at.
    /// Pinning the yaw instead would swing the body to the recording's starting
    /// posture and back.
    #[test]
    fn a_still_body_yaw_is_dropped_and_the_head_keeps_its_pose_in_the_body() {
        let yaw = 0.15;
        let json = recording(
            0.1,
            vec![
                frame(identity(), [0.0, 0.0], Some(yaw)),
                frame(lifted(0.01), [0.0, 0.0], Some(yaw)),
            ],
        );
        let import = convert(
            &json,
            "pollen/test/offset",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect("a centimetre of lift is inside the envelope");

        assert_eq!(import.doc().channels, vec![Channel::Head]);
        assert!(import.constant.contains(&Channel::BodyYaw));
        let last = import.clip.frames().last().expect("frames");
        let head = last.head.expect("the head is masked");
        // The recording's head is level in the world while the body sits at
        // 0.15 rad, so relative to the body it is turned the other way by the
        // same angle — for every frame, including the first.
        let axis = head.rotation.scaled_axis();
        assert!(
            (axis - Vector3::new(0.0, 0.0, -yaw)).abs().max() < 1e-9,
            "{axis:?}",
        );
        assert!(
            (head.translation.vector - Vector3::new(0.0, 0.0, 0.01))
                .abs()
                .max()
                < 1e-9,
            "{:?}",
            head.translation.vector,
        );
        for frame in &import.doc().frames {
            assert_eq!(frame.body_yaw, None, "the clip drives no yaw");
        }
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

    /// A number no arithmetic can use, at each of the three places the file can
    /// carry one.
    ///
    /// The rule is that a non-finite number is never handed onward, and this is
    /// where one enters: an infinity in a timestamp divides the resampler, one
    /// in an antenna reaches a goal register, one in the head matrix reaches the
    /// polar extraction. Each guard is its own loop, so each gets its own input.
    #[test]
    fn a_non_finite_number_is_refused_at_the_frame_and_key_that_carried_it() {
        // Driven through the parsed document rather than through JSON text:
        // `1e400` is refused by the JSON parser itself as out of range, so no
        // text reaches these guards, and they exist for the value a future
        // reader hands them.
        let parsed = || {
            let json = recording(
                0.1,
                vec![
                    frame(identity(), [0.0, 0.0], Some(0.0)),
                    frame(identity(), [0.0, 0.0], Some(0.0)),
                ],
            );
            serde_json::from_str::<VendorMove>(&json).expect("a recording parses")
        };

        let mut times = parsed();
        times.time[1] = f64::INFINITY;
        assert_eq!(
            rebased_times(&times).expect_err("a timestamp nothing can subtract"),
            ImportError::NonFinite {
                frame: 1,
                key: "time"
            }
        );

        for (key, poison) in [
            ("body_yaw", 0usize),
            ("antennas", 1),
            ("antennas", 2),
            ("head", 3),
        ] {
            let mut doc = parsed();
            let frame = &mut doc.set_target_data[1];
            match poison {
                0 => frame.body_yaw = Some(f64::INFINITY),
                1 => frame.antennas[0] = f64::NAN,
                2 => frame.antennas[1] = f64::INFINITY,
                _ => frame.head[0][0] = f64::NAN,
            }
            assert_eq!(
                samples(&doc).expect_err("a number no arithmetic can use"),
                ImportError::NonFinite { frame: 1, key },
            );
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

    /// A drifted rotation — what six-decimal rounding and an unnormalised
    /// composition cost — is renormalised rather than refused, and the
    /// quaternion written out is a unit one.
    ///
    /// The drift here is a quarter of the tolerance, which is far past the
    /// loader's own norm check: nothing but real renormalisation gets this
    /// document back through a load.
    #[test]
    fn a_rotation_that_only_drifted_is_renormalised() {
        let mut drifted = identity();
        drifted[0][0] = 1.0 + ROTATION_TOL / 4.0;
        let json = recording(0.1, vec![frame(drifted, [0.0, 0.0], None)]);
        let import = convert(
            &json,
            "pollen/test/drift",
            &ClipLimits::default(),
            &ImportOptions {
                channels: Some(ChannelMask::of(Channel::Head)),
            },
        )
        .expect("drift is not a wrong matrix");
        let head = import.clip.frames()[0].head.expect("masked");
        assert!(head.rotation.angle() < 1e-6, "{:?}", head.rotation);
        let dq = import.doc().frames[0].dq.expect("a head frame carries one");
        let norm = dq.iter().map(|term| term * term).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-15, "{norm}");
        assert!(
            import.rotation_drift > ROTATION_DRIFT_NOTED,
            "{}",
            import.rotation_drift
        );
    }

    /// The stretched population: a rotation scaled by a quarter permille, which
    /// is what one of the vendor's recorders wrote, converts to the attitude it
    /// was trying to express.
    ///
    /// The polar factor is the answer, and it is under a hundredth of a degree
    /// from the matrix in the file — less than the mechanism resolves and less
    /// than the six decimals the vendor already quantised to.
    #[test]
    fn a_uniformly_stretched_rotation_converts_to_the_attitude_it_meant() {
        let stretch = 1.00025;
        let turned = |scale: f64| {
            let (sin, cos) = 0.3_f64.sin_cos();
            vec![
                vec![cos * scale, -sin * scale, 0.0, 0.0],
                vec![sin * scale, cos * scale, 0.0, 0.0],
                vec![0.0, 0.0, scale, 0.0],
                vec![0.0, 0.0, 0.0, 1.0],
            ]
        };
        let head_only = ImportOptions {
            channels: Some(ChannelMask::of(Channel::Head)),
        };
        let convert_one = |head: Vec<Vec<f64>>| {
            convert(
                &recording(0.1, vec![frame(head, [0.0, 0.0], None)]),
                "pollen/test/stretched",
                &ClipLimits::default(),
                &head_only,
            )
        };
        let stretched = convert_one(turned(stretch)).expect("a stretch is drift, not content");
        let exact = convert_one(turned(1.0)).expect("the same attitude, written exactly");

        assert!(
            (4e-4..1e-3).contains(&stretched.rotation_drift),
            "{}",
            stretched.rotation_drift,
        );
        let (got, want) = (
            stretched.clip.frames()[0].head.expect("masked").rotation,
            exact.clip.frames()[0].head.expect("masked").rotation,
        );
        assert!(got.angle_to(&want) < 1e-3, "{}", got.angle_to(&want));
        let dq = stretched.doc().frames[0]
            .dq
            .expect("a head frame carries one");
        let norm = dq.iter().map(|term| term * term).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-15, "{norm}");
    }

    /// A percent of scale is not drift: nothing rounds or composes its way to
    /// that, so the file is one we have misread.
    #[test]
    fn a_matrix_scaled_by_a_percent_is_refused_on_drift() {
        let mut scaled = identity();
        for (axis, row) in scaled.iter_mut().enumerate().take(3) {
            row[axis] = 1.01;
        }
        let refused = convert(
            &recording(0.1, vec![frame(scaled, [0.0, 0.0], None)]),
            "pollen/test/scaled",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect_err("a percent is content, not precision");
        let ImportError::Rotation { frame: at, detail } = &refused else {
            panic!("expected a rotation refusal: {refused:?}");
        };
        assert_eq!(*at, 0);
        assert!(detail.contains("orthonormal"), "{detail}");
    }

    /// A real recording from the published dataset converts, and what it says
    /// about itself is what the report will print.
    ///
    /// The drift here is the whole reason the first tolerance refused a hundred
    /// good files: it sits above 1e-6 and below anything worth mentioning.
    #[test]
    fn the_published_simple_nod_converts() {
        let json = include_str!("../tests/fixtures/vendor/simple_nod.json");
        let import = convert(
            json,
            "pollen/dances/simple_nod",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect("a published recording converts");

        assert_eq!(import.source_frames, 92);
        assert_eq!(import.clip.frames().len(), 92, "recorded at the tick rate");
        assert_eq!(
            import.doc().channels,
            vec![Channel::Head, Channel::Antennas],
            "the yaw is stated and never moves",
        );
        assert_eq!(import.constant, vec![Channel::BodyYaw]);
        assert!(import.unknown_keys.is_empty(), "{:?}", import.unknown_keys);
        assert!(
            (1e-6..ROTATION_DRIFT_NOTED).contains(&import.rotation_drift),
            "six-decimal quantisation, past the tolerance this used to carry: {}",
            import.rotation_drift,
        );
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

    /// The mask override force-pins a channel the recording carries and holds
    /// still, and refuses to invent one it does not.
    #[test]
    fn the_mask_override_may_pin_a_still_channel_but_never_add_an_absent_one() {
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
    /// of the vendor's silence. `check_collision` is not one of them: it is
    /// declared by name and parsed, not caught by `extra`.
    #[test]
    fn keys_we_do_not_read_are_reported_and_not_refused() {
        let mut first = frame(identity(), [0.0, 0.0], None);
        first["check_collision"] = json!(false);
        let mut second = frame(identity(), [0.2, -0.1], None);
        second["check_collision"] = Value::Null;
        let times = [0.0, 0.1];
        let json = json!({
            "description": "d",
            "time": times,
            "set_target_data": [first, second],
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
        assert_eq!(import.unknown_keys, vec!["recorded_by".to_owned()]);
    }

    /// A recording whose frames step further per tick than any move this stack
    /// plans converts and loads with nothing said about it: content is not
    /// judged on its speed.
    #[test]
    fn a_recording_stepping_further_per_tick_than_our_own_moves_converts() {
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
        let import = convert(
            &json,
            "pollen/test/snap",
            &ClipLimits::default(),
            &ImportOptions::default(),
        )
        .expect("a fast recording is content, not a malformed file");
        assert_eq!(import.clip.frames().len(), 2);
        assert!(import.clip.notes().is_empty(), "{:?}", import.clip.notes());
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

    /// The written document is the one the loader reads back: the blends in the
    /// file are the ones the load settled on, and a reload lands on the same
    /// clip.
    #[test]
    fn the_written_document_carries_what_the_loader_derived() {
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
        // Two frames at 50 Hz is 40 ms of clip, so the unstated default blend is
        // capped at the clip's own length and both ends are written as they
        // stand.
        assert_eq!(import.doc().blend_in_ms, Some(import.clip.blend_in_ms()));
        assert_eq!(import.doc().blend_out_ms, Some(import.clip.blend_out_ms()));

        let text = serde_json::to_string(&import.doc()).expect("renders");
        let reloaded = Clip::from_json(&text, &ClipLimits::default()).expect("loads back");
        assert_eq!(reloaded, import.clip);
    }
}
