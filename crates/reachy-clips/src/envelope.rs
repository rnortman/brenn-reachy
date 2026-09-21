//! The import-time envelope walk: what a clip's frames have to satisfy before
//! the clip loads at all.
//!
//! One pass over the frame track. Unposed frames are screened over a **static
//! neutral base**; posed frames use their declared base only for the channels
//! they drive. Two questions per frame, both geometric:
//!
//! - does a frame that drives head or body yaw pass the coupled head envelope
//!   check — reachable, every crank inside its travel window, clear of the
//!   linkage's singular configurations, inside the yaw cap and the attitude
//!   cone;
//! - when present, is every antenna angle one a goal register can represent.
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
//! Full-weight screening is exact only when both coupled head/yaw channels are
//! driven. Blends and masked pairs remain protected by the per-tick check.
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
#[derive(Clone, Debug, Error, PartialEq)]
pub enum FrameError {
    /// A frame's deltas, applied to the neutral base, leave the envelope.
    #[error("frame {frame} leaves the envelope over the neutral base: {violations}")]
    Envelope {
        /// Which frame.
        frame: usize,
        /// Everything that frame's pose failed.
        violations: EnvelopeViolations,
    },
    /// A posed frame leaves the envelope over its authored base.
    #[error("frame {frame} leaves the envelope over base {base:?}: {violations}")]
    AnchoredEnvelope {
        frame: usize,
        base: String,
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
    /// A posed frame asks an antenna for an unrepresentable absolute angle.
    #[error(
        "frame {frame} over base {base:?} commands antenna {side} to {angle} rad, which has no goal count"
    )]
    AnchoredAntennaGoal {
        frame: usize,
        base: String,
        side: usize,
        angle: f64,
    },
}

/// Walk a clip's frames over their base, refusing the clip on the first
/// frame this machine could not hold there.
///
/// `frames` must be non-empty and must carry exactly the channels the clip masks
/// — the invariants [`crate::format::Clip`] establishes before it calls this.
///
/// # Panics
///
/// If `frames` is empty.
pub fn check_frames(
    frames: &[DeltaFrame],
    anchor: Option<&crate::format::ResolvedAnchor>,
    limits: &ClipLimits,
) -> Result<(), FrameError> {
    assert!(!frames.is_empty(), "a clip has frames");

    for (index, frame) in frames.iter().enumerate() {
        check_frame(index, frame, anchor, limits)?;
    }
    Ok(())
}

/// One frame over its base, refusing a frame this machine could not hold there.
fn check_frame(
    index: usize,
    frame: &DeltaFrame,
    anchor: Option<&crate::format::ResolvedAnchor>,
    limits: &ClipLimits,
) -> Result<(), FrameError> {
    let (base_name, base_head, base_yaw, base_antennas) = match anchor {
        Some(anchor) => (
            Some(anchor.name().to_owned()),
            anchor.targets().head_pose_body,
            anchor.targets().body_yaw,
            anchor.targets().antennas,
        ),
        None => (None, neutral_head_pose(), 0.0, [0.0, 0.0]),
    };
    if let Some(antennas) = frame.antennas {
        for (side, delta) in antennas.iter().enumerate() {
            let angle = base_antennas[side] + delta;
            if !(ANTENNA_GOAL_MIN_RAD..=ANTENNA_GOAL_MAX_RAD).contains(&angle) {
                return Err(match base_name {
                    Some(base) => FrameError::AnchoredAntennaGoal {
                        frame: index,
                        base,
                        side,
                        angle,
                    },
                    None => FrameError::AntennaGoal {
                        frame: index,
                        side,
                        angle,
                    },
                });
            }
        }
    }
    if frame.head.is_none() && frame.body_yaw.is_none() {
        return Ok(());
    }
    let pose = frame.head.map_or(base_head, |head| {
        base_head * interpolate_pose(&Isometry3::identity(), &head, 1.0)
    });
    let absolute_yaw = base_yaw + frame.body_yaw.unwrap_or(0.0);

    let mut report = EnvelopeReport::default();
    match check_envelope(
        &limits.geom,
        &limits.env,
        &pose,
        absolute_yaw,
        None,
        &mut report,
    ) {
        Ok(()) => Ok(()),
        Err(error) => Err(match base_name {
            Some(base) => FrameError::AnchoredEnvelope {
                frame: index,
                base,
                violations: error.violations,
            },
            None => FrameError::Envelope {
                frame: index,
                violations: error.violations,
            },
        }),
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
        let error =
            check_frames(&frames, None, &ClipLimits::default()).expect_err("outside the cone");
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
        let error =
            check_frames(&frames, None, &ClipLimits::default()).expect_err("past the yaw bound");
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
        let error = check_frames(&frames, None, &ClipLimits::default()).expect_err("no goal count");
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
        check_frames(&frames, None, &ClipLimits::default()).expect("inside the envelope");
    }

    /// A frame the linkage cannot reach at all is refused as such, whatever the
    /// rest of the track does.
    #[test]
    fn a_frame_with_no_crank_solution_refuses_the_clip() {
        let frames = [lift(0.5)];
        let error = check_frames(&frames, None, &ClipLimits::default())
            .expect_err("the linkage cannot reach it");
        assert!(matches!(error, FrameError::Envelope { frame: 0, .. }));
    }

    /// The limits are the caller's, not this module's: a daemon configured with
    /// a tighter envelope than the shipped one walks its clips against that.
    /// The same frame is taken by the defaults and refused by the tighter
    /// bound, so a walk that read the defaults internally would fail this.
    #[test]
    fn the_walk_answers_to_the_limits_it_is_handed() {
        let frames = [pitch(20.0)];
        check_frames(&frames, None, &ClipLimits::default()).expect("inside the shipped cone");

        let mut cfg = MotionConfig::default();
        cfg.env.head_cone_limit = 10.0_f64.to_radians();
        let tight = ClipLimits::from_motion_config(&cfg);
        let error = check_frames(&frames, None, &tight).expect_err("outside the configured cone");
        assert!(matches!(
            error,
            FrameError::Envelope { frame: 0, violations } if violations.cone
        ));
    }

    #[test]
    fn posed_screening_checks_only_driven_channels_and_their_coupled_base() {
        let invalid_head = reachy_kin::geometry::rest_head_pose();
        let antenna_anchor = crate::format::ResolvedAnchor {
            name: "antennas-only".to_owned(),
            targets: reachy_motion::JointTargets {
                head_pose_body: invalid_head,
                antennas: [0.0, 0.0],
                ..Default::default()
            },
        };
        check_frames(
            &[antennas(0.0, 0.0)],
            Some(&antenna_anchor),
            &ClipLimits::default(),
        )
        .expect("an antennas-only frame does not screen the anchor head");

        let head_anchor = crate::format::ResolvedAnchor {
            name: "head-only".to_owned(),
            targets: reachy_motion::JointTargets {
                antennas: [ANTENNA_GOAL_MAX_RAD, ANTENNA_GOAL_MIN_RAD],
                ..Default::default()
            },
        };
        check_frames(&[lift(0.0)], Some(&head_anchor), &ClipLimits::default())
            .expect("a head-only frame does not screen anchor antennas");

        let yaw_anchor = crate::format::ResolvedAnchor {
            name: "yaw-only".to_owned(),
            targets: reachy_motion::JointTargets {
                head_pose_body: invalid_head,
                ..Default::default()
            },
        };
        let error = check_frames(&[yaw(0.0)], Some(&yaw_anchor), &ClipLimits::default())
            .expect_err("yaw drives the coupled head check using its anchor head");
        assert!(matches!(
            error,
            FrameError::AnchoredEnvelope { frame: 0, base, .. } if base == "yaw-only"
        ));
    }

    /// The documented panic: an empty track is the caller's invariant to
    /// establish, and `Clip` refuses a document with no frames before this is
    /// ever reached.
    #[test]
    #[should_panic(expected = "a clip has frames")]
    fn a_walk_over_no_frames_is_a_caller_bug() {
        let _ = check_frames(&[], None, &ClipLimits::default());
    }
}
