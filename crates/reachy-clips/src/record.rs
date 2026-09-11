//! A recorded stretch of a limp machine, as a clip document.
//!
//! The other authoring path into the format. The importer converts somebody
//! else's recording; this converts *ours* — a stretch of the pose recorder's
//! stream, taken with the torque off and a pair of hands on the head, with the
//! head pose already solved per frame by whoever read the stream.
//!
//! What it does is the one conversion that stands between a reading and a
//! frame: a recording holds where the machine *was*, and a clip holds what a
//! motion *does*, so every channel comes out as a delta over the neutral base.
//! The head is `neutral⁻¹ · pose` in the base head's own frame, the antennas
//! are the measured angles less their neutral lean — so a hand that left them
//! where a lift puts them extracts to zeros rather than to a pair of tenths
//! nobody meant — and body yaw is measured less zero, which is where neutral
//! puts it. Get any of that wrong and the composer adds the recording to the
//! base a second time.
//!
//! Pure, and no wider than that: frames in, document out. Reading the stream,
//! choosing the stretch and writing the file belong to the caller.
//!
//! **Nothing here validates the motion.** A document this builds is raw — a
//! hand's path at the sampling rate, tremor and all — and a hand can hold the
//! head where the envelope will not. The refusals below are about frames that
//! carry no reading at all; whether the result is playable is
//! [`Clip::from_doc`](crate::format::Clip::from_doc)'s answer, and a recorded
//! frame past the toggle floor or the cone cap is a refused clip, which is
//! correct.

use nalgebra::Isometry3;
use reachy_kin::neutral_head_pose;
use reachy_motion::{FLOOR_TICK_HZ, NEUTRAL_ANTENNAS};
use thiserror::Error;

use crate::format::{
    CLIP_KIND, Channel, ChannelMask, ClipDoc, FORMAT_VERSION, FrameDoc, NameError, validate_name,
};

/// One frame of a recording: where the machine stood, as it was read.
///
/// Every quantity is optional and absent means *unread*, not zero. A crank the
/// forward solve did not close on has no pose; a servo that stopped answering
/// has no angle. Zero is a legal reading of all three, so a slot that carried
/// the missing case as a number would extract as a delta somebody could not
/// tell from a measurement.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RecordedFrame {
    /// The head pose in the body frame, absolute, as forward kinematics solved
    /// it; `None` where no seed closed the linkage.
    pub head: Option<Isometry3<f64>>,
    /// Antenna angles, right then left, radians, absolute.
    pub antennas: [Option<f64>; 2],
    /// Body yaw, radians, absolute.
    pub body_yaw: Option<f64>,
}

/// What the caller wants the document to say about itself.
#[derive(Clone, Debug)]
pub struct Draft<'a> {
    /// The library name the clip is invoked by. Checked here rather than at the
    /// load, so a name the library would refuse costs an argument and not a
    /// session's worth of extraction.
    pub name: &'a str,
    /// Free text: which session and which stretch of it this came off.
    pub description: Option<String>,
    /// The channels to drive. A channel outside it is left unsaid, which is how
    /// the format spells "this clip has no opinion about that".
    pub mask: ChannelMask,
}

/// Why a recorded stretch is not a clip document.
///
/// Each arm refuses the whole extraction. There is no partial document and no
/// interpolation across a hole: a frame track with a guessed frame in it is a
/// motion nobody recorded, and the caller can always drop the channel that
/// went unread instead.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum RecordError {
    /// The name is not one the library can file.
    #[error("clip name {name:?} is unusable: {source}")]
    Name {
        /// The name as given.
        name: String,
        /// What is wrong with it.
        source: NameError,
    },
    /// The mask names no channel, so the document would drive nothing.
    #[error("a clip drives at least one channel; this extraction names none")]
    NoChannels,
    /// The stretch holds no frames.
    #[error("a clip carries at least one frame; this stretch holds none")]
    NoFrames,
    /// A masked channel went unread in one of the frames.
    #[error(
        "frame {frame} carries no reading of {}: the recording cannot drive it",
        channel.as_str()
    )]
    Unread {
        /// Which frame, counted from zero within the stretch.
        frame: usize,
        /// Which masked channel had nothing in it.
        channel: Channel,
    },
}

/// The clip document a recorded stretch extracts to.
///
/// One frame per recorded frame, at the sampling rate, in the order given: the
/// recorder samples on the tick grid, which is the only rate a clip is read at,
/// so there is nothing to resample. The ramps are left unsaid — a recording
/// states no blend intent, and omitted means the loader's default capped at the
/// clip's own length rather than a ramp longer than the motion.
///
/// A one-frame stretch is a pose rather than a motion, and is the same document
/// in the same shape: a pose is a clip a blend reaches.
///
/// # Errors
///
/// [`RecordError`]: an unusable name, an empty mask, no frames, or a masked
/// channel unread in some frame, named with the frame it went missing in.
pub fn clip_doc(draft: &Draft, frames: &[RecordedFrame]) -> Result<ClipDoc, RecordError> {
    validate_name(draft.name).map_err(|source| RecordError::Name {
        name: draft.name.to_owned(),
        source,
    })?;
    if draft.mask.is_empty() {
        return Err(RecordError::NoChannels);
    }
    if frames.is_empty() {
        return Err(RecordError::NoFrames);
    }
    let track = frames
        .iter()
        .enumerate()
        .map(|(index, frame)| delta(index, frame, draft.mask))
        .collect::<Result<Vec<FrameDoc>, RecordError>>()?;
    Ok(ClipDoc {
        version: FORMAT_VERSION,
        kind: CLIP_KIND.to_owned(),
        name: draft.name.to_owned(),
        description: draft.description.clone(),
        channels: Channel::ALL
            .into_iter()
            .filter(|channel| draft.mask.contains(*channel))
            .collect(),
        frame_hz: FLOOR_TICK_HZ,
        blend_in_ms: None,
        blend_out_ms: None,
        frames: track,
    })
}

/// One recorded frame as the deltas the composer adds.
///
/// The head's delta is taken in neutral's own frame, which is the frame the
/// composition applies it in; taking it in the body frame instead would rotate
/// every recorded translation by neutral's attitude.
fn delta(index: usize, frame: &RecordedFrame, mask: ChannelMask) -> Result<FrameDoc, RecordError> {
    let unread = |channel: Channel| RecordError::Unread {
        frame: index,
        channel,
    };
    let (dt, dq) = if mask.contains(Channel::Head) {
        let pose = frame.head.ok_or_else(|| unread(Channel::Head))?;
        let head = neutral_head_pose().inverse() * pose;
        let q = head.rotation.quaternion();
        (
            Some([
                head.translation.vector.x,
                head.translation.vector.y,
                head.translation.vector.z,
            ]),
            Some([q.w, q.i, q.j, q.k]),
        )
    } else {
        (None, None)
    };
    let antennas = if mask.contains(Channel::Antennas) {
        let mut pair = [0.0; 2];
        for (side, angle) in pair.iter_mut().enumerate() {
            let measured = frame.antennas[side].ok_or_else(|| unread(Channel::Antennas))?;
            *angle = measured - NEUTRAL_ANTENNAS[side];
        }
        Some(pair)
    } else {
        None
    };
    let body_yaw = if mask.contains(Channel::BodyYaw) {
        // Neutral's yaw is zero, so the measured angle *is* the delta. Written
        // as the subtraction it is, because the day neutral yaws is the day
        // this line has to change and a bare pass-through would not say so.
        Some(frame.body_yaw.ok_or_else(|| unread(Channel::BodyYaw))? - 0.0)
    } else {
        None
    };
    Ok(FrameDoc {
        dt,
        dq,
        antennas,
        body_yaw,
    })
}

#[cfg(test)]
mod tests {
    use nalgebra::{Translation3, UnitQuaternion, Vector3};
    use reachy_motion::neutral_targets;

    use super::*;
    use crate::envelope::{ClipLimits, FrameError};
    use crate::format::Clip;

    /// The draft every case below varies from: all three channels, a name the
    /// library files.
    fn draft(name: &str) -> Draft<'_> {
        Draft {
            name,
            description: Some("session under test".to_owned()),
            mask: ChannelMask::all(),
        }
    }

    /// The frame a limp machine standing exactly at neutral would record.
    fn at_neutral() -> RecordedFrame {
        let targets = neutral_targets();
        RecordedFrame {
            head: Some(targets.head_pose_body),
            antennas: targets.antennas.map(Some),
            body_yaw: Some(targets.body_yaw),
        }
    }

    /// The whole point of the conversion: a machine left where a lift puts it
    /// records as a clip that does nothing, on every channel at once. An
    /// antenna delta of the neutral lean, or a head delta in the body frame,
    /// would each show up here as a motion nobody performed.
    #[test]
    fn a_frame_recorded_at_neutral_extracts_to_no_motion_at_all() {
        let doc = clip_doc(&draft("recorded/session/s001"), &[at_neutral()])
            .expect("a frame at neutral extracts");
        assert_eq!(doc.frames.len(), 1, "one frame in, one frame out");
        let frame = &doc.frames[0];
        assert_eq!(frame.antennas, Some([0.0, 0.0]));
        assert_eq!(frame.body_yaw, Some(0.0));
        let dt = frame.dt.expect("the head channel is masked");
        let dq = frame.dq.expect("the head channel is masked");
        for axis in dt {
            assert!(axis.abs() < 1e-9, "the translation delta is zero: {dt:?}");
        }
        assert!(
            (dq[0].abs() - 1.0).abs() < 1e-9 && dq[1..].iter().all(|term| term.abs() < 1e-9),
            "the rotation delta is the identity: {dq:?}"
        );
    }

    /// The document is the one the loader reads, and the extraction is not the
    /// validator: what comes out goes through `Clip::from_doc` unchanged.
    #[test]
    fn the_document_loads_as_the_clip_it_claims_to_be() {
        let frames = vec![at_neutral(); 5];
        let doc = clip_doc(&draft("recorded/session/s002"), &frames).expect("five neutral frames");
        let clip = Clip::from_doc(doc, &ClipLimits::default()).expect("a clip of no motion loads");
        assert_eq!(clip.name(), "recorded/session/s002");
        assert_eq!(clip.description(), Some("session under test"));
        assert_eq!(clip.frames().len(), 5);
        assert_eq!(clip.mask().iter().count(), Channel::COUNT);
    }

    /// A recorded lift is a translation delta of what the hand actually
    /// travelled, in metres, on the axis it travelled along.
    #[test]
    fn a_recorded_lift_extracts_as_the_translation_the_hand_made() {
        let lifted = Isometry3::from_parts(
            Translation3::from(Vector3::new(0.0, 0.0, 0.030)) * neutral_head_pose().translation,
            neutral_head_pose().rotation,
        );
        let frame = RecordedFrame {
            head: Some(lifted),
            ..at_neutral()
        };
        let doc = clip_doc(&draft("recorded/session/s003"), &[frame]).expect("a lifted frame");
        let dt = doc.frames[0].dt.expect("the head channel is masked");
        assert!(
            (dt[2] - 0.030).abs() < 1e-9 && dt[0].abs() < 1e-9 && dt[1].abs() < 1e-9,
            "30 mm up and nothing else: {dt:?}"
        );
    }

    /// An absolute antenna angle and an absolute yaw come out as what they add
    /// to the base, and the right antenna is the first of the pair.
    #[test]
    fn antenna_and_yaw_deltas_are_the_measurement_less_the_base() {
        let frame = RecordedFrame {
            antennas: [Some(0.5), Some(-0.25)],
            body_yaw: Some(0.2),
            ..at_neutral()
        };
        let doc = clip_doc(&draft("recorded/session/s004"), &[frame]).expect("a moved frame");
        let antennas = doc.frames[0].antennas.expect("the pair is masked");
        assert!((antennas[0] - (0.5 - NEUTRAL_ANTENNAS[0])).abs() < 1e-12);
        assert!((antennas[1] - (-0.25 - NEUTRAL_ANTENNAS[1])).abs() < 1e-12);
        assert_eq!(doc.frames[0].body_yaw, Some(0.2));
    }

    /// A channel outside the mask is absent from every frame, which is the
    /// format's own way of saying the clip has no opinion about it — and the
    /// unread reading of such a channel is then nobody's problem.
    #[test]
    fn an_unmasked_channel_is_absent_and_its_unread_reading_costs_nothing() {
        let mut draft = draft("recorded/session/s005");
        draft.mask = ChannelMask::of(Channel::Antennas);
        let frame = RecordedFrame {
            head: None,
            antennas: [Some(0.0), Some(0.0)],
            body_yaw: None,
        };
        let doc = clip_doc(&draft, &[frame]).expect("an antennas-only extraction");
        assert_eq!(doc.channels, vec![Channel::Antennas]);
        assert_eq!(doc.frames[0].dt, None);
        assert_eq!(doc.frames[0].dq, None);
        assert_eq!(doc.frames[0].body_yaw, None);
        assert!(doc.frames[0].antennas.is_some());
    }

    /// A hand that put the head where the linkage does not close leaves a
    /// frame with no pose in it, and the refusal names which frame — the
    /// operator's next move is to cut the stretch elsewhere or drop the
    /// channel, and both need the index.
    #[test]
    fn a_frame_the_solver_did_not_close_on_is_refused_by_index() {
        let frames = vec![
            at_neutral(),
            at_neutral(),
            RecordedFrame {
                head: None,
                ..at_neutral()
            },
        ];
        assert_eq!(
            clip_doc(&draft("recorded/session/s006"), &frames),
            Err(RecordError::Unread {
                frame: 2,
                channel: Channel::Head,
            })
        );
    }

    /// A servo that stopped answering is the same refusal on the channel it
    /// belongs to. Zero is a legal antenna angle, so extracting the missing
    /// case as one would write a delta of the neutral lean into the clip.
    #[test]
    fn a_silent_servo_is_refused_on_the_channel_it_drives() {
        let frames = vec![
            RecordedFrame {
                antennas: [Some(0.0), None],
                ..at_neutral()
            },
            at_neutral(),
        ];
        assert_eq!(
            clip_doc(&draft("recorded/session/s007"), &frames),
            Err(RecordError::Unread {
                frame: 0,
                channel: Channel::Antennas,
            })
        );
        let frames = vec![RecordedFrame {
            body_yaw: None,
            ..at_neutral()
        }];
        assert_eq!(
            clip_doc(&draft("recorded/session/s008"), &frames),
            Err(RecordError::Unread {
                frame: 0,
                channel: Channel::BodyYaw,
            })
        );
    }

    /// The three shape refusals. The name is checked here rather than at the
    /// load because the caller's next argument is the fix.
    #[test]
    fn a_document_nobody_could_load_is_refused_before_it_is_built() {
        assert_eq!(
            clip_doc(&draft("Recorded/Session"), &[at_neutral()]),
            Err(RecordError::Name {
                name: "Recorded/Session".to_owned(),
                source: NameError::BadChar { ch: 'R' },
            })
        );
        assert_eq!(
            clip_doc(&draft("recorded/session/s009"), &[]),
            Err(RecordError::NoFrames)
        );
        let mut nothing = draft("recorded/session/s010");
        nothing.mask = ChannelMask::empty();
        assert_eq!(
            clip_doc(&nothing, &[at_neutral()]),
            Err(RecordError::NoChannels)
        );
    }

    /// The raw draft is not judged here, and the loader is where a hand's
    /// reach becomes a refusal: a frame outside the envelope extracts fine and
    /// fails to load, which is the division of labour the header states.
    #[test]
    fn a_pose_outside_the_envelope_extracts_and_then_fails_to_load() {
        let far = Isometry3::from_parts(
            Translation3::from(Vector3::new(0.0, 0.0, 0.25)) * neutral_head_pose().translation,
            UnitQuaternion::identity(),
        );
        let doc = clip_doc(
            &draft("recorded/session/s011"),
            &[RecordedFrame {
                head: Some(far),
                ..at_neutral()
            }],
        )
        .expect("the extraction does not judge the pose");
        assert!(matches!(
            Clip::from_doc(doc, &ClipLimits::default()),
            Err(crate::format::ClipError::Frames {
                source: FrameError::Envelope { frame: 0, .. }
            })
        ));
    }
}
