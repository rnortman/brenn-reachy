//! The import-time envelope walk: what a clip's frames have to satisfy before
//! the clip loads at all.
//!
//! One pass over the frame track, every frame taken over a **static neutral
//! base**, which is the context a recording was made in. Two questions per
//! frame, both geometric:
//!
//! - does the frame's head pose, at the frame's body yaw, pass the envelope
//!   check — reachable, every crank inside its travel window, clear of the
//!   linkage's singular configurations, inside the yaw cap and the attitude
//!   cone;
//! - is every antenna angle one a goal register can represent.
//!
//! A frame that fails either is a pose this machine cannot hold standing still,
//! so the clip is refused. Nothing is projected, trimmed or slowed to make it
//! pass.
//!
//! A clip carries no speed ceiling, no derived blend floors and no seam check.
//! The machine's per-tick step bound (`JointStep`) is a planner-bug guard on the
//! moves this stack plans for itself — sized off two of our own recorded
//! gestures, checked against a sampled trajectory this crate authored. A library
//! clip is content: its frames are what somebody recorded, and how fast they move
//! is a property of the recording rather than a claim about the machine. A
//! composed setpoint is commanded as asked, and the envelope above is the only
//! screen content faces.
//!
//! What the envelope does not answer is play-time: a delta that is inside the
//! envelope over neutral can be outside it over a lifted or yawed base, and the
//! per-tick envelope check on the composed target is what holds there.

use nalgebra::Isometry3;
use reachy_kin::envelope::{EnvelopeConfig, EnvelopeReport, EnvelopeViolations, check_envelope};
use reachy_kin::geometry::{HeadGeometry, neutral_head_pose};
use reachy_motion::{ANTENNA_GOAL_MAX_RAD, ANTENNA_GOAL_MIN_RAD, MotionConfig};
use thiserror::Error;

use crate::compose::interpolate_pose;
use crate::format::DeltaFrame;

/// The bounds a clip is checked against.
///
/// A copy of the two parts of [`MotionConfig`] the walk reads, so a caller that
/// has a configured daemon checks against *its* geometry and envelope rather
/// than against defaults, and a caller that has none still gets the defaults the
/// machine ships with.
#[derive(Clone, Debug)]
pub struct ClipLimits {
    /// The head geometry the crank angles are solved through.
    pub geom: HeadGeometry,
    /// The envelope every frame's pose is checked against.
    pub env: EnvelopeConfig,
}

impl Default for ClipLimits {
    fn default() -> Self {
        Self::from_motion_config(&MotionConfig::default())
    }
}

impl ClipLimits {
    /// The limits a configured motion stack imposes.
    #[must_use]
    pub fn from_motion_config(cfg: &MotionConfig) -> Self {
        Self {
            geom: cfg.geom.clone(),
            env: cfg.env,
        }
    }
}

/// Why a clip's frame track is refused.
///
/// Each of these refuses the whole clip. They are content faults: the recording
/// asks for something this machine cannot hold even standing still, so no ramp
/// and no playback makes it reachable.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum FrameError {
    /// A frame's deltas, applied to the neutral base, leave the envelope.
    #[error("frame {frame} leaves the envelope over the neutral base: {violations}")]
    Envelope {
        /// Which frame.
        frame: usize,
        /// Everything that frame's pose failed.
        violations: EnvelopeViolations,
    },
    /// A frame asks an antenna for an angle no goal register represents.
    #[error("frame {frame} commands antenna {side} to {angle} rad, which has no goal count")]
    AntennaGoal {
        /// Which frame.
        frame: usize,
        /// Which antenna, right then left.
        side: usize,
        /// The commanded angle, radians.
        angle: f64,
    },
}

/// Walk a clip's frames over the neutral base, refusing the clip on the first
/// frame this machine could not hold there.
///
/// `frames` must be non-empty and must carry exactly the channels the clip masks
/// — the invariants [`crate::format::Clip`] establishes before it calls this.
///
/// # Panics
///
/// If `frames` is empty.
pub fn check_frames(frames: &[DeltaFrame], limits: &ClipLimits) -> Result<(), FrameError> {
    assert!(!frames.is_empty(), "a clip has frames");

    for (index, frame) in frames.iter().enumerate() {
        check_frame(index, frame, limits)?;
    }
    Ok(())
}

/// One frame over the neutral base, refusing a frame this machine could not hold
/// there.
fn check_frame(index: usize, frame: &DeltaFrame, limits: &ClipLimits) -> Result<(), FrameError> {
    let body_yaw = frame.body_yaw.unwrap_or(0.0);
    let antennas = frame.antennas.unwrap_or([0.0, 0.0]);
    for (side, angle) in antennas.iter().enumerate() {
        if !(ANTENNA_GOAL_MIN_RAD..=ANTENNA_GOAL_MAX_RAD).contains(angle) {
            return Err(FrameError::AntennaGoal {
                frame: index,
                side,
                angle: *angle,
            });
        }
    }

    let mut report = EnvelopeReport::default();
    let pose = pose_at(frame);
    match check_envelope(
        &limits.geom,
        &limits.env,
        &pose,
        body_yaw,
        None,
        &mut report,
    ) {
        Ok(()) => Ok(()),
        Err(error) => Err(FrameError::Envelope {
            frame: index,
            violations: error.violations,
        }),
    }
}

/// The head pose one frame's delta puts the head at over the neutral base.
///
/// The same right-multiplication [`crate::compose`] performs, against the one
/// base the walk knows about.
fn pose_at(frame: &DeltaFrame) -> Isometry3<f64> {
    match frame.head {
        Some(head) => neutral_head_pose() * interpolate_pose(&Isometry3::identity(), &head, 1.0),
        None => neutral_head_pose(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};

    /// An antennas-only frame.
    fn antennas(right: f64, left: f64) -> DeltaFrame {
        DeltaFrame {
            head: None,
            antennas: Some([right, left]),
            body_yaw: None,
        }
    }

    /// A head-only frame lifting the head by `dz` metres.
    fn lift(dz: f64) -> DeltaFrame {
        DeltaFrame {
            head: Some(Isometry3::translation(0.0, 0.0, dz)),
            antennas: None,
            body_yaw: None,
        }
    }

    /// A head-only frame pitching the head by `deg`.
    fn pitch(deg: f64) -> DeltaFrame {
        DeltaFrame {
            head: Some(Isometry3::from_parts(
                Translation3::identity(),
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), deg.to_radians()),
            )),
            antennas: None,
            body_yaw: None,
        }
    }

    /// A body-yaw-only frame.
    fn yaw(rad: f64) -> DeltaFrame {
        DeltaFrame {
            head: None,
            antennas: None,
            body_yaw: Some(rad),
        }
    }

    #[test]
    fn a_frame_outside_the_envelope_refuses_the_clip() {
        // Well past the 35° cone bound.
        let frames = [pitch(0.0), pitch(80.0)];
        let error = check_frames(&frames, &ClipLimits::default()).expect_err("outside the cone");
        match error {
            FrameError::Envelope { frame, violations } => {
                assert_eq!(frame, 1);
                assert!(violations.cone);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_body_yaw_past_its_bound_refuses_the_clip() {
        let frames = [yaw(0.0), yaw(3.0)];
        let error = check_frames(&frames, &ClipLimits::default()).expect_err("past the yaw bound");
        assert!(matches!(
            error,
            FrameError::Envelope { frame: 1, violations } if violations.body_yaw
        ));
    }

    #[test]
    fn an_antenna_angle_with_no_goal_count_refuses_the_clip() {
        let frames = [
            antennas(0.0, 0.0),
            antennas(ANTENNA_GOAL_MAX_RAD * 2.0, 0.0),
        ];
        let error = check_frames(&frames, &ClipLimits::default()).expect_err("no goal count");
        assert!(matches!(
            error,
            FrameError::AntennaGoal {
                frame: 1,
                side: 0,
                ..
            }
        ));
    }

    /// A track that steps further per frame than any move this stack plans is
    /// content, not a fault: the walk has nothing to say about how fast it moves.
    #[test]
    fn a_track_that_moves_faster_than_our_own_moves_do_still_walks() {
        let frames = [antennas(0.0, 0.0), antennas(1.2, -1.2), lift(0.01)];
        check_frames(&frames, &ClipLimits::default()).expect("inside the envelope");
    }

    /// A frame the linkage cannot reach at all is refused as such, whatever the
    /// rest of the track does.
    #[test]
    fn a_frame_with_no_crank_solution_refuses_the_clip() {
        let frames = [lift(0.5)];
        let error =
            check_frames(&frames, &ClipLimits::default()).expect_err("the linkage cannot reach it");
        assert!(matches!(error, FrameError::Envelope { frame: 0, .. }));
    }

    /// The limits are the caller's, not this module's: a daemon configured with
    /// a tighter envelope than the shipped one walks its clips against that.
    /// The same frame is taken by the defaults and refused by the tighter
    /// bound, so a walk that read the defaults internally would fail this.
    #[test]
    fn the_walk_answers_to_the_limits_it_is_handed() {
        let frames = [pitch(20.0)];
        check_frames(&frames, &ClipLimits::default()).expect("inside the shipped cone");

        let mut cfg = MotionConfig::default();
        cfg.env.head_cone_limit = 10.0_f64.to_radians();
        let tight = ClipLimits::from_motion_config(&cfg);
        let error = check_frames(&frames, &tight).expect_err("outside the configured cone");
        assert!(matches!(
            error,
            FrameError::Envelope { frame: 0, violations } if violations.cone
        ));
    }

    /// The documented panic: an empty track is the caller's invariant to
    /// establish, and `Clip` refuses a document with no frames before this is
    /// ever reached.
    #[test]
    #[should_panic(expected = "a clip has frames")]
    fn a_walk_over_no_frames_is_a_caller_bug() {
        let _ = check_frames(&[], &ClipLimits::default());
    }
}
