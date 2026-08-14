//! What a clip's own frames say about how fast it may be played, and how
//! gently it has to be blended in and out.
//!
//! Three numbers come out of one pass over a clip's frame track, all three
//! derived from the same place — the per-tick step bounds the tick guard
//! applies to whatever is commanded (`JointStep`) — and all three computed
//! **over a static neutral base**, which is the context the recording was made
//! in:
//!
//! - `max_speed`, the highest invocation speed at which no adjacent frame pair
//!   asks for more than one tick's worth of movement.
//! - the blend-in floor, the shortest entry ramp that cannot trip a step bound
//!   *while the clip's own clock is advancing under it*, which is the only way
//!   an entry ramp ever runs.
//! - the blend-out floor, the same for the exit ramp, which runs alone.
//!
//! The first two share one tick's step, so they share one budget: a blend-in
//! commands its own travel and the frame-to-frame travel on the same period,
//! and two numbers each sized to spend the whole allowance would together spend
//! twice it.
//!
//! None of them is a safety gate and none of them is exact. The gate is the
//! per-tick envelope check and step bound applied to the *composed* target,
//! every tick, over whatever base is live; a delta that is gentle over neutral
//! can be anything at all over an aggressive base. What these numbers buy is
//! that a clip which would have been refused at its very first tick over a
//! resting robot never loads, and that the ordinary case leaves headroom for a
//! base that is moving underneath.
//!
//! ## Why the legs need the envelope pass
//!
//! The step bounds are per joint, and the head's joints are six cranks driven
//! through the inverse kinematics; a clip's head deltas are Cartesian and say
//! nothing directly about crank travel. So the derivation solves each frame's
//! pose — which it has to do anyway, to refuse a clip whose recorded motion
//! leaves our envelope — and takes the crank angles the envelope report already
//! carries. Adjacent frames' crank differences are what the leg bound is
//! checked against. That is conservative rather than exact: above 1.0× a tick's
//! two endpoints are inverse-kinematic solutions of poses interpolated
//! *between* frames, and the map is nonlinear, so the per-frame differences
//! bound the per-tick one from the safe side.

use nalgebra::Isometry3;
use reachy_kin::envelope::{EnvelopeConfig, EnvelopeReport, EnvelopeViolations, check_envelope};
use reachy_kin::geometry::{HeadGeometry, neutral_head_pose};
use reachy_motion::{
    ANTENNA_GOAL_MAX_RAD, ANTENNA_GOAL_MIN_RAD, FLOOR_TICK_HZ, JointStep, MotionConfig,
};
use thiserror::Error;

use crate::compose::interpolate_pose;
use crate::format::{Channel, ChannelMask, DeltaFrame, MAX_SPEED};

/// How much of each per-tick step bound the clip's own clock advance is allowed
/// to spend.
///
/// The remaining fifth is what a base moving underneath the overlay eats into.
/// It is a cushion, not a guarantee — no bound computed without knowing the
/// base could be one — and the runtime check is what actually holds. An entry
/// ramp spends that fifth rather than the cushion holding through it, since a
/// ramp and an advance land on the same tick.
pub const STEP_MARGIN: f64 = 0.8;

/// How many points along a blend ramp the leg derivation samples.
///
/// A ramp scales a head delta from identity to its full value, and the crank
/// angles along that path are not linear in the weight, so the fastest stretch
/// of it is found by looking rather than by differencing the endpoints. Eight
/// samples put the estimate within a few percent on the deltas real recordings
/// carry, and the whole derivation is offline.
pub const RAMP_SAMPLES: usize = 8;

/// The slop a ramp rate gets before it is rounded up to whole ticks.
///
/// A rate is a quotient of two products of bounds and shares, so a delta of
/// exactly sixteen ramp-shares lands a hair either side of sixteen depending on
/// which way the arithmetic went. Rounding that up buys a seventeenth tick of
/// ramp for nothing and makes the floor depend on the last bit of a division.
const RATE_EPS: f64 = 1e-9;

/// The bounds a clip is derived against.
///
/// A copy of the three parts of [`MotionConfig`] the derivation reads, so a
/// caller that has a configured daemon derives against *its* bounds rather than
/// against defaults, and a caller that has none still gets the defaults the
/// machine ships with.
#[derive(Clone, Debug)]
pub struct ClipLimits {
    /// The head geometry the crank angles are solved through.
    pub geom: HeadGeometry,
    /// The envelope every frame's pose is checked against.
    pub env: EnvelopeConfig,
    /// The per-tick step bounds every derived number is scaled off.
    pub max_step: JointStep,
    /// The tick rate, hertz. A frame is one tick.
    pub tick_hz: f64,
    /// The fraction of each step bound the derivation may spend.
    pub step_margin: f64,
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
            max_step: cfg.max_step,
            tick_hz: FLOOR_TICK_HZ,
            step_margin: STEP_MARGIN,
        }
    }

    /// The step bound a derivation may actually spend on `bound`.
    fn usable(&self, bound: f64) -> f64 {
        bound * self.step_margin
    }

    /// The share of a whole step bound an entry ramp may spend.
    ///
    /// Not [`Self::usable`], and the difference is the point: a blend-in runs
    /// while the clip's own clock is advancing, so the tick commands the ramp's
    /// travel *and* the frame-to-frame travel at once. The speed ceiling
    /// already spends [`Self::usable`] on the second term, so what is left for
    /// the first is everything else the bound holds. The two together come to
    /// the whole bound and no more, which is what the tick guard actually
    /// checks. During an entry ramp the margin is therefore spent rather than
    /// held in reserve — the reserve exists for a moving base, and a base that
    /// moves under a blend-in is what the runtime gate is for.
    fn blend_in_share(&self) -> f64 {
        1.0 - self.step_margin
    }

    /// The share of a whole step bound an exit ramp may spend.
    ///
    /// The whole of [`Self::usable`], because nothing advances beside it: a
    /// blend-out runs past a clip's final frame, holding that frame's delta
    /// while the weight falls, and a channel fading at a sequence seam is by
    /// definition one the incoming clip does not drive.
    fn blend_out_share(&self) -> f64 {
        self.step_margin
    }
}

/// Why a clip's frame track admits no derivation.
///
/// Each of these refuses the whole clip. They are content faults: the recording
/// asks for something this machine cannot hold even standing still, so no
/// invocation speed and no ramp makes it playable.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum DeriveError {
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
    /// A partly blended-in frame has no inverse-kinematic solution, so the ramp
    /// that reaches that frame passes through a pose the head cannot hold.
    #[error("frame {frame} has no solution at blend weight {weight}")]
    BlendPath {
        /// Which frame.
        frame: usize,
        /// The weight along the ramp where the solution was lost.
        weight: f64,
    },
}

/// What one frame's deltas come to in joint coordinates, over the neutral base.
///
/// The crank angles are the inverse-kinematic solution of the frame's head
/// pose; the other two are the deltas themselves, since the neutral base is
/// zero in both. Kept for a clip's first and last frames so a sequence can
/// measure the step at a seam between two clips without re-solving either.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct FrameMetrics {
    /// The six crank angles, radians.
    pub legs: [f64; 6],
    /// Body yaw, radians.
    pub body_yaw: f64,
    /// Antenna angles, right then left, radians.
    pub antennas: [f64; 2],
}

/// Everything one pass over a frame track derives.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Derivation {
    /// The highest invocation speed every adjacent frame pair stays inside the
    /// step bounds at, never above the global ceiling.
    pub max_speed: f64,
    /// The shortest entry ramp that cannot trip a step bound, milliseconds.
    pub blend_in_floor_ms: u32,
    /// The shortest exit ramp that cannot trip a step bound, milliseconds.
    pub blend_out_floor_ms: u32,
    /// The first frame in joint coordinates.
    pub first: FrameMetrics,
    /// The last frame in joint coordinates.
    pub last: FrameMetrics,
}

/// Derive a clip's speed ceiling and blend floors from its frames.
///
/// `frames` must be non-empty and must carry exactly the channels `mask` names
/// — the invariants [`crate::Clip`] establishes before it calls this.
///
/// # Panics
///
/// If `frames` is empty.
pub fn derive(
    frames: &[DeltaFrame],
    mask: ChannelMask,
    limits: &ClipLimits,
) -> Result<Derivation, DeriveError> {
    assert!(!frames.is_empty(), "a clip has frames");

    let mut metrics = Vec::with_capacity(frames.len());
    // The largest rate of change of any joint with respect to blend weight,
    // over every frame: what the entry ramp has to be slow enough to absorb,
    // since a player joining a script in progress ramps from zero against
    // whichever frame it joined at.
    let mut ramp_rate = 0.0_f64;
    // The last frame's own rate, which is the only one the exit ramp answers to:
    // a blend-out runs from the final frame and nowhere else. Carried out of the
    // loop rather than asked for again afterwards, since the answer is the same
    // one and the question costs nine inverse-kinematic solves.
    let mut out_rate = 0.0_f64;
    for (index, frame) in frames.iter().enumerate() {
        let solved = solve_frame(index, frame, limits)?;
        out_rate = frame_ramp_rate(index, frame, solved.legs, limits)?;
        ramp_rate = ramp_rate.max(out_rate);
        metrics.push(solved);
    }

    let max_speed = speed_ceiling(&metrics, mask, limits);
    let last_index = frames.len() - 1;

    Ok(Derivation {
        max_speed,
        blend_in_floor_ms: floor_ms(ramp_rate / limits.blend_in_share(), limits),
        blend_out_floor_ms: floor_ms(out_rate / limits.blend_out_share(), limits),
        first: metrics[0],
        last: metrics[last_index],
    })
}

/// The joint coordinates of one frame over the neutral base, refusing a frame
/// this machine could not hold there.
fn solve_frame(
    index: usize,
    frame: &DeltaFrame,
    limits: &ClipLimits,
) -> Result<FrameMetrics, DeriveError> {
    let body_yaw = frame.body_yaw.unwrap_or(0.0);
    let antennas = frame.antennas.unwrap_or([0.0, 0.0]);
    for (side, angle) in antennas.iter().enumerate() {
        if !(ANTENNA_GOAL_MIN_RAD..=ANTENNA_GOAL_MAX_RAD).contains(angle) {
            return Err(DeriveError::AntennaGoal {
                frame: index,
                side,
                angle: *angle,
            });
        }
    }

    let mut report = EnvelopeReport::default();
    let pose = pose_at(frame, 1.0);
    match check_envelope(
        &limits.geom,
        &limits.env,
        &pose,
        body_yaw,
        None,
        &mut report,
    ) {
        Ok(()) => Ok(FrameMetrics {
            legs: report.leg_angles.expect("an accepted pose solved").0,
            body_yaw,
            antennas,
        }),
        Err(error) => Err(DeriveError::Envelope {
            frame: index,
            violations: error.violations,
        }),
    }
}

/// The head pose one frame's delta puts the head at over the neutral base,
/// scaled to blend weight `w`.
///
/// The same right-multiplication [`crate::compose`] performs, against the one
/// base the derivation knows about.
fn pose_at(frame: &DeltaFrame, w: f64) -> Isometry3<f64> {
    match frame.head {
        Some(head) => neutral_head_pose() * interpolate_pose(&Isometry3::identity(), &head, w),
        None => neutral_head_pose(),
    }
}

/// How fast each joint moves per unit of blend weight, for one frame, as a
/// multiple of that joint's whole per-tick step bound.
///
/// Whole bounds rather than usable ones, because the two ends of a ramp are
/// allowed different shares of the bound: an entry ramp shares the tick with
/// the clip's own clock advance and an exit ramp does not. The caller divides
/// by the share its end gets.
///
/// Body yaw and the antennas are linear in the weight, so their rate is the
/// delta itself. The legs are not: the crank angles along the ramp are
/// inverse-kinematic solutions of scaled poses, so the path is sampled and the
/// steepest stretch of it is what the floor answers to. A sample with no
/// solution is a refusal rather than a gap in the estimate — the ramp would
/// pass through a pose the head cannot hold.
///
/// `legs` is the ramp's own endpoint at full weight, which the caller has
/// already solved for this frame: solving it a second time here would be a
/// seventh of the derivation spent re-answering a question. It is the crank
/// solution of the same pose because the cranks depend on the head pose alone,
/// which is what lets the intermediate samples be solved at zero body yaw.
fn frame_ramp_rate(
    index: usize,
    frame: &DeltaFrame,
    legs: [f64; 6],
    limits: &ClipLimits,
) -> Result<f64, DeriveError> {
    let mut rate: f64 = 0.0;
    if let Some(delta) = frame.body_yaw {
        rate = rate.max(delta.abs() / limits.max_step.body_yaw);
    }
    if let Some(antennas) = frame.antennas {
        for delta in antennas {
            rate = rate.max(delta.abs() / limits.max_step.antennas);
        }
    }
    if frame.head.is_some() {
        let bound = limits.max_step.legs;
        let mut previous =
            solve_legs(&pose_at(frame, 0.0), limits).ok_or(DeriveError::BlendPath {
                frame: index,
                weight: 0.0,
            })?;
        for sample in 1..=RAMP_SAMPLES {
            let w = sample as f64 / RAMP_SAMPLES as f64;
            let angles = if sample == RAMP_SAMPLES {
                legs
            } else {
                solve_legs(&pose_at(frame, w), limits).ok_or(DeriveError::BlendPath {
                    frame: index,
                    weight: w,
                })?
            };
            for leg in 0..6 {
                let per_weight = (angles[leg] - previous[leg]).abs() * RAMP_SAMPLES as f64;
                rate = rate.max(per_weight / bound);
            }
            previous = angles;
        }
    }
    Ok(rate)
}

/// The crank angles of a pose, or `None` where some leg has no solution.
///
/// The envelope's other bounds are deliberately not applied: this runs on
/// intermediate points of a blend ramp whose endpoints are already checked, and
/// what a ramp has to be slow enough for is the travel, not the verdict.
fn solve_legs(pose: &Isometry3<f64>, limits: &ClipLimits) -> Option<[f64; 6]> {
    let mut report = EnvelopeReport::default();
    let _ = check_envelope(&limits.geom, &limits.env, pose, 0.0, None, &mut report);
    report.leg_angles.map(|angles| angles.0)
}

/// The shortest ramp that keeps a rate of `rate` shares per unit weight inside
/// one tick, milliseconds, rounded up to whole ticks.
///
/// A ramp spends one unit of weight over its length and a tick may spend one
/// share, so the ramp needs `rate` ticks. At or below one tick it needs
/// none: the whole delta already fits inside a single tick's step, and a ramp
/// shorter than a period is not a ramp — it is the same jump, with a stretch
/// reported against it for nothing.
fn floor_ms(rate: f64, limits: &ClipLimits) -> u32 {
    if rate <= 1.0 + RATE_EPS {
        return 0;
    }
    let ms = ((rate - RATE_EPS).ceil() * 1000.0 / limits.tick_hz).ceil();
    if ms >= f64::from(u32::MAX) {
        u32::MAX
    } else {
        ms as u32
    }
}

/// The highest speed every adjacent frame pair stays inside the step bounds at.
///
/// At speed `s` a tick advances the clip clock by `s` frames, so a pair whose
/// joints differ by `d` asks for `s * d` in one tick.
fn speed_ceiling(metrics: &[FrameMetrics], mask: ChannelMask, limits: &ClipLimits) -> f64 {
    let mut ceiling = MAX_SPEED;
    let mut bound = |step: f64, usable: f64| {
        if step > 0.0 {
            ceiling = ceiling.min(usable / step);
        }
    };
    for pair in metrics.windows(2) {
        let (before, after) = (&pair[0], &pair[1]);
        if mask.contains(Channel::Head) {
            for leg in 0..6 {
                bound(
                    (after.legs[leg] - before.legs[leg]).abs(),
                    limits.usable(limits.max_step.legs),
                );
            }
        }
        if mask.contains(Channel::BodyYaw) {
            bound(
                (after.body_yaw - before.body_yaw).abs(),
                limits.usable(limits.max_step.body_yaw),
            );
        }
        if mask.contains(Channel::Antennas) {
            for side in 0..2 {
                bound(
                    (after.antennas[side] - before.antennas[side]).abs(),
                    limits.usable(limits.max_step.antennas),
                );
            }
        }
    }
    // A one-frame clip has no pair and no motion of its own; nothing about its
    // track bounds the speed it is played at.
    ceiling
}

/// The per-tick step a seam between two clips commands, in bound-widths.
///
/// A seam is the tick where one segment's last frame is replaced by the next
/// segment's first. Nothing interpolates across it and, for a channel both
/// clips drive, nothing blends across it either — the weight is already at one
/// on both sides — so the difference of the two deltas is commanded whole, in
/// one tick, at any invocation speed. A hold between them changes nothing: it
/// freezes the outgoing delta and the same jump happens when it ends.
///
/// The answer is a multiple of the usable step bound: at most 1.0 is a seam the
/// tick will accept over a static base.
#[must_use]
pub fn seam_step(
    out_metrics: &FrameMetrics,
    out_mask: ChannelMask,
    in_metrics: &FrameMetrics,
    in_mask: ChannelMask,
    limits: &ClipLimits,
) -> f64 {
    let shared = |channel: Channel| out_mask.contains(channel) && in_mask.contains(channel);
    let mut worst: f64 = 0.0;
    if shared(Channel::Head) {
        for leg in 0..6 {
            let step = (in_metrics.legs[leg] - out_metrics.legs[leg]).abs();
            worst = worst.max(step / limits.usable(limits.max_step.legs));
        }
    }
    if shared(Channel::BodyYaw) {
        let step = (in_metrics.body_yaw - out_metrics.body_yaw).abs();
        worst = worst.max(step / limits.usable(limits.max_step.body_yaw));
    }
    if shared(Channel::Antennas) {
        for side in 0..2 {
            let step = (in_metrics.antennas[side] - out_metrics.antennas[side]).abs();
            worst = worst.max(step / limits.usable(limits.max_step.antennas));
        }
    }
    worst
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
    fn a_still_clip_may_be_played_at_the_global_ceiling() {
        let frames = [antennas(0.2, -0.2), antennas(0.2, -0.2)];
        let derived = derive(
            &frames,
            ChannelMask::of(Channel::Antennas),
            &ClipLimits::default(),
        )
        .expect("inside the envelope");
        assert_eq!(derived.max_speed, MAX_SPEED);
    }

    #[test]
    fn an_antenna_step_bounds_the_speed_by_the_bound_it_spends() {
        let limits = ClipLimits::default();
        // Half the usable bound per frame: playable at exactly twice speed,
        // which the global ceiling happens to agree with, so take a third.
        let step = limits.usable(limits.max_step.antennas) / 3.0;
        let frames = [antennas(0.0, 0.0), antennas(step, 0.0)];
        let derived =
            derive(&frames, ChannelMask::of(Channel::Antennas), &limits).expect("in envelope");
        assert!(
            (derived.max_speed - 3.0_f64.min(MAX_SPEED)).abs() < 1e-9,
            "{}",
            derived.max_speed
        );
    }

    #[test]
    fn the_speed_ceiling_takes_the_tightest_pair_in_the_track() {
        let limits = ClipLimits::default();
        let gentle = limits.usable(limits.max_step.antennas) / 8.0;
        let harsh = limits.usable(limits.max_step.antennas) / 2.0;
        let frames = [
            antennas(0.0, 0.0),
            antennas(gentle, 0.0),
            antennas(gentle + harsh, 0.0),
        ];
        let derived =
            derive(&frames, ChannelMask::of(Channel::Antennas), &limits).expect("in envelope");
        assert!(
            (derived.max_speed - 2.0).abs() < 1e-9,
            "{}",
            derived.max_speed
        );
    }

    #[test]
    fn an_unmasked_channel_does_not_bound_the_speed() {
        // The frames carry only antennas, so a mask naming body yaw as well
        // must not read the absent channel as a hard-held zero.
        let limits = ClipLimits::default();
        let mut mask = ChannelMask::of(Channel::Antennas);
        mask.insert(Channel::BodyYaw);
        let frames = [antennas(0.0, 0.0), antennas(0.01, 0.0)];
        let derived = derive(&frames, mask, &limits).expect("in envelope");
        assert_eq!(derived.max_speed, MAX_SPEED);
    }

    #[test]
    fn the_leg_bound_comes_from_the_solved_crank_angles() {
        let limits = ClipLimits::default();
        // A centimetre of lift in one frame is a real crank excursion; the
        // derived ceiling has to be finite and the pair has to be what set it.
        let frames = [lift(0.0), lift(0.01)];
        let derived =
            derive(&frames, ChannelMask::of(Channel::Head), &limits).expect("in envelope");
        assert!(derived.max_speed < MAX_SPEED, "{}", derived.max_speed);
        let crank = (derived.last.legs[0] - derived.first.legs[0]).abs();
        assert!(crank > 0.0);
        assert!(derived.max_speed <= limits.usable(limits.max_step.legs) / crank + 1e-9);
    }

    #[test]
    fn a_frame_outside_the_envelope_refuses_the_clip() {
        // Well past the 35° cone bound.
        let frames = [pitch(0.0), pitch(80.0)];
        let error = derive(
            &frames,
            ChannelMask::of(Channel::Head),
            &ClipLimits::default(),
        )
        .expect_err("outside the cone");
        match error {
            DeriveError::Envelope { frame, violations } => {
                assert_eq!(frame, 1);
                assert!(violations.cone);
            }
            other => panic!("{other:?}"),
        }
    }

    #[test]
    fn a_body_yaw_past_its_bound_refuses_the_clip() {
        let frames = [yaw(0.0), yaw(3.0)];
        let error = derive(
            &frames,
            ChannelMask::of(Channel::BodyYaw),
            &ClipLimits::default(),
        )
        .expect_err("past the yaw bound");
        assert!(matches!(
            error,
            DeriveError::Envelope { frame: 1, violations } if violations.body_yaw
        ));
    }

    #[test]
    fn an_antenna_angle_with_no_goal_count_refuses_the_clip() {
        let frames = [
            antennas(0.0, 0.0),
            antennas(ANTENNA_GOAL_MAX_RAD * 2.0, 0.0),
        ];
        let error = derive(
            &frames,
            ChannelMask::of(Channel::Antennas),
            &ClipLimits::default(),
        )
        .expect_err("no goal count");
        assert!(matches!(
            error,
            DeriveError::AntennaGoal {
                frame: 1,
                side: 0,
                ..
            }
        ));
    }

    #[test]
    fn the_blend_in_floor_comes_from_the_largest_delta_not_the_first() {
        let limits = ClipLimits::default();
        let big = limits.usable(limits.max_step.antennas) * 4.0;
        let frames = [antennas(0.0, 0.0), antennas(big, 0.0)];
        let derived =
            derive(&frames, ChannelMask::of(Channel::Antennas), &limits).expect("in envelope");
        // Four usable-widths is 3.2 whole bounds, and an entry ramp gets the
        // fifth of the bound the clip's own advance does not spend: sixteen
        // ticks, 320 ms at 50 Hz. The first frame is the zero delta and would
        // have asked for none.
        assert_eq!(derived.blend_in_floor_ms, 320);
    }

    #[test]
    fn the_blend_out_floor_comes_from_the_final_frame_alone() {
        let limits = ClipLimits::default();
        let big = limits.usable(limits.max_step.antennas) * 4.0;
        // Large in the middle, back to nothing at the end: the exit ramp fades
        // from the final frame and needs no time at all.
        let frames = [antennas(0.0, 0.0), antennas(big, 0.0), antennas(0.0, 0.0)];
        let derived =
            derive(&frames, ChannelMask::of(Channel::Antennas), &limits).expect("in envelope");
        assert_eq!(derived.blend_in_floor_ms, 320);
        assert_eq!(derived.blend_out_floor_ms, 0);
    }

    /// An exit ramp runs alone — the clip is over and its final frame is frozen
    /// — so it keeps the whole usable step rather than the entry ramp's share,
    /// and is four times the shorter for it.
    #[test]
    fn the_two_ramps_get_different_shares_of_the_bound() {
        let limits = ClipLimits::default();
        let big = limits.usable(limits.max_step.antennas) * 4.0;
        let frames = [antennas(big, 0.0)];
        let derived =
            derive(&frames, ChannelMask::of(Channel::Antennas), &limits).expect("in envelope");
        assert_eq!(derived.blend_in_floor_ms, 320);
        assert_eq!(derived.blend_out_floor_ms, 80);
    }

    /// The ramp and the clip's own advance land on the same tick, so what has
    /// to stay inside the whole step bound is their sum — which is the thing
    /// two independently derived numbers would each have spent whole.
    #[test]
    fn a_ramp_at_its_floor_leaves_room_for_the_advance_beside_it() {
        let limits = ClipLimits::default();
        let usable = limits.usable(limits.max_step.antennas);
        // A track that sits far from zero *and* moves at its ceiling: the worst
        // tick of a blend-in is one where both terms are at their largest.
        let big = usable * 4.0;
        let frames = [
            antennas(big, 0.0),
            antennas(big + usable, 0.0),
            antennas(big, 0.0),
        ];
        let mask = ChannelMask::of(Channel::Antennas);
        let derived = derive(&frames, mask, &limits).expect("in envelope");

        let ticks = f64::from(derived.blend_in_floor_ms) / 1000.0 * limits.tick_hz;
        let ramp_term = (big + usable) / ticks;
        let advance_term = usable * derived.max_speed;
        assert!(
            ramp_term + advance_term <= limits.max_step.antennas + 1e-12,
            "ramp {ramp_term} + advance {advance_term} past {}",
            limits.max_step.antennas
        );
    }

    #[test]
    fn a_head_ramp_floor_is_derived_along_the_ramp() {
        let limits = ClipLimits::default();
        let frames = [lift(0.012)];
        let derived =
            derive(&frames, ChannelMask::of(Channel::Head), &limits).expect("in envelope");
        assert!(derived.blend_in_floor_ms > 0);
        // Sampling the ramp at the derived length must keep every tick inside
        // the crank bound.
        let ticks = (f64::from(derived.blend_in_floor_ms) / 1000.0 * limits.tick_hz).ceil() as u32;
        let mut previous = solve_legs(&pose_at(&frames[0], 0.0), &limits).expect("solves");
        for tick in 1..=ticks {
            let w = f64::from(tick) / f64::from(ticks);
            let angles = solve_legs(&pose_at(&frames[0], w), &limits).expect("solves");
            for leg in 0..6 {
                let step = (angles[leg] - previous[leg]).abs();
                assert!(
                    step <= limits.usable(limits.max_step.legs) + 1e-9,
                    "leg {leg} stepped {step}"
                );
            }
            previous = angles;
        }
    }

    #[test]
    fn a_clip_at_its_derived_max_speed_stays_inside_every_step_bound() {
        let limits = ClipLimits::default();
        // A track that moves on all three channels at once.
        let frames: Vec<DeltaFrame> = (0..25)
            .map(|index| {
                let phase = f64::from(index) * 0.25;
                DeltaFrame {
                    head: Some(Isometry3::translation(0.0, 0.0, 0.004 * phase.sin())),
                    antennas: Some([0.3 * phase.sin(), -0.2 * phase.cos()]),
                    body_yaw: Some(0.15 * phase.sin()),
                }
            })
            .collect();
        let mut mask = ChannelMask::of(Channel::Head);
        mask.insert(Channel::Antennas);
        mask.insert(Channel::BodyYaw);
        let derived = derive(&frames, mask, &limits).expect("in envelope");

        // Advancing the clip clock at the derived speed, no tick may ask for
        // more than a step bound in any joint. Endpoints are solved fresh, so
        // this exercises the interpolation the player actually performs rather
        // than the per-frame differences the derivation reasons about.
        let speed = derived.max_speed;
        let mut clock = 0.0;
        let mut previous = probe(&frames, 0.0, &limits);
        while clock < frames.len() as f64 - 1.0 {
            clock += speed;
            let at = clock.min(frames.len() as f64 - 1.0);
            let now = probe(&frames, at, &limits);
            for leg in 0..6 {
                assert!(
                    (now.legs[leg] - previous.legs[leg]).abs() <= limits.max_step.legs,
                    "leg {leg} at clock {at}"
                );
            }
            assert!((now.body_yaw - previous.body_yaw).abs() <= limits.max_step.body_yaw);
            for side in 0..2 {
                assert!(
                    (now.antennas[side] - previous.antennas[side]).abs()
                        <= limits.max_step.antennas
                );
            }
            previous = now;
        }
    }

    /// The joint coordinates of a track sampled at a fractional frame index.
    fn probe(frames: &[DeltaFrame], at: f64, limits: &ClipLimits) -> FrameMetrics {
        let lower = at.floor() as usize;
        let upper = (lower + 1).min(frames.len() - 1);
        let t = at - lower as f64;
        let head = match (frames[lower].head, frames[upper].head) {
            (Some(a), Some(b)) => Some(interpolate_pose(&a, &b, t)),
            _ => None,
        };
        let frame = DeltaFrame {
            head,
            antennas: match (frames[lower].antennas, frames[upper].antennas) {
                (Some(a), Some(b)) => Some([a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t]),
                _ => None,
            },
            body_yaw: match (frames[lower].body_yaw, frames[upper].body_yaw) {
                (Some(a), Some(b)) => Some(a + (b - a) * t),
                _ => None,
            },
        };
        solve_frame(0, &frame, limits).expect("in envelope")
    }

    #[test]
    fn a_seam_between_two_clips_measures_the_step_it_commands() {
        let limits = ClipLimits::default();
        let mask = ChannelMask::of(Channel::Antennas);
        let out = FrameMetrics {
            antennas: [0.0, 0.0],
            ..FrameMetrics::default()
        };
        let step = limits.usable(limits.max_step.antennas);
        let incoming = FrameMetrics {
            antennas: [step, 0.0],
            ..FrameMetrics::default()
        };
        assert!((seam_step(&out, mask, &incoming, mask, &limits) - 1.0).abs() < 1e-12);

        // A channel only one of the two drives is not a seam: the one leaving
        // the mask blends out and the one entering blends in.
        let head_only = ChannelMask::of(Channel::Head);
        assert_eq!(seam_step(&out, mask, &incoming, head_only, &limits), 0.0);
    }

    /// Head seams difference six solved crank angles and answer with the worst;
    /// body yaw is a scalar difference.
    #[test]
    fn a_head_or_yaw_seam_measures_the_worst_of_its_joints() {
        let limits = ClipLimits::default();
        let head = ChannelMask::of(Channel::Head);
        let usable = limits.usable(limits.max_step.legs);
        let out = FrameMetrics::default();
        let incoming = FrameMetrics {
            // One leg a half-width out, another two widths out: the worst leg
            // is the answer, not the first or the sum.
            legs: [usable * 0.5, 0.0, usable * 2.0, 0.0, 0.0, 0.0],
            ..FrameMetrics::default()
        };
        assert!((seam_step(&out, head, &incoming, head, &limits) - 2.0).abs() < 1e-12);

        let yaw = ChannelMask::of(Channel::BodyYaw);
        let turned = FrameMetrics {
            body_yaw: limits.usable(limits.max_step.body_yaw) * 1.5,
            ..FrameMetrics::default()
        };
        assert!((seam_step(&out, yaw, &turned, yaw, &limits) - 1.5).abs() < 1e-12);

        // Masks that share nothing measure nothing, whichever pair they are.
        assert_eq!(seam_step(&out, head, &turned, yaw, &limits), 0.0);
    }

    /// A frame the head can hold, on a ramp it cannot walk to.
    ///
    /// A ramp sample
    /// with no inverse-kinematic solution is a pose the head cannot pass
    /// through, not a hole in the estimate to be skipped over. Reached here
    /// with a shortened rod, which puts the *neutral* end of the ramp outside
    /// the linkage's reach while the frame's own pose stays inside it — with
    /// the shipped geometry the reachable set along a scaled translation has no
    /// gap in the middle, so the end of the ramp is where the arm is entered.
    #[test]
    fn a_ramp_through_a_pose_the_head_cannot_hold_refuses_the_clip() {
        let mut limits = ClipLimits::default();
        limits.geom.rod_len = 0.06;
        let frames = [lift(-0.02)];
        let error = derive(&frames, ChannelMask::of(Channel::Head), &limits)
            .expect_err("the ramp leaves the linkage");
        assert_eq!(
            error,
            DeriveError::BlendPath {
                frame: 0,
                weight: 0.0,
            }
        );

        // A hair longer and the whole ramp solves: the refusal is about the
        // path, not about the shortened rod alone.
        limits.geom.rod_len = 0.0615;
        derive(&frames, ChannelMask::of(Channel::Head), &limits).expect("the ramp is walkable");
    }
}
