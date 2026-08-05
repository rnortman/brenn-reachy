//! Trajectories: a shaped path between two command sets.
//!
//! The servos shape nothing. A goal position is applied as an immediate step,
//! so every gentle movement in this system is an interpolation computed here
//! and emitted as a bounded increment once per tick.
//!
//! A trajectory is fixed at construction and then sampled — no state, no
//! integration, no dependence on when it was last asked. Sampling the same time
//! twice gives the same answer bit for bit, which is what lets a tick that
//! overran its period simply ask for the time it actually is instead of
//! catching up.
//!
//! Shape: one scalar warp of normalised time drives all four components, so
//! translation, rotation, body yaw and the antennas start together, finish
//! together, and stay in the same phase throughout. Rotation follows the
//! geodesic between the two orientations; translation and the scalars are
//! straight lines.
//!
//! Two things this deliberately does differently from the vendor's open
//! implementation, both of which it gets wrong:
//!
//! - The endpoint is **sampled**. Interpolating over a half-open interval that
//!   never reaches 1 leaves the machine a hair short of its commanded pose
//!   forever, and the residue accumulates over chained moves. Here a sample at
//!   or past the duration returns the target's own bits.
//! - A non-positive duration is refused at construction rather than dividing by
//!   zero at the first sample.
//!
//! Large body-yaw moves in front of the machine would want the head yaw carried
//! as a scalar alongside the body's rather than folded into the pose geodesic,
//! which is what the vendor's implementation switches to for that case. For
//! pitch-and-height-only commands the plain geodesic is both correct and
//! simpler; the yaw-scalar case would arrive as another warp-adjacent variant.

use core::time::Duration;

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use thiserror::Error;

use crate::joints::JointTargets;

/// How normalised time maps onto normalised progress.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Warp {
    /// Minimum-jerk: starts and ends at zero velocity *and* zero acceleration.
    /// The default for anything the head does, because the linkage's own
    /// compliance rings on a velocity step.
    MinJerk,
    /// Constant rate. For test paths and for motion whose endpoints are
    /// already at rest by construction.
    Linear,
}

impl Warp {
    /// Progress at normalised time `u`, which the caller has already placed in
    /// `[0, 1]`.
    ///
    /// `MinJerk` is `10u³ − 15u⁴ + 6u⁵`: the quintic with `s(0) = 0`,
    /// `s(1) = 1` and both first and second derivatives zero at each end.
    #[must_use]
    fn progress(self, u: f64) -> f64 {
        match self {
            Warp::MinJerk => u * u * u * (10.0 + u * (6.0 * u - 15.0)),
            Warp::Linear => u,
        }
    }
}

/// A commanded move that cannot be shaped.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum TrajectoryError {
    /// A duration of zero. There is no path in no time, and the normalisation
    /// would divide by it.
    #[error("trajectory duration must be greater than zero")]
    NonPositiveDuration,
    /// Some endpoint carries a non-finite number. Refused here rather than
    /// carried to the envelope check as a violation on every tick of a move
    /// that was never going to arrive.
    #[error("trajectory endpoints must be finite")]
    NonFinite,
}

/// A shaped path between two command sets, sampled by elapsed time.
#[derive(Clone, Debug, PartialEq)]
pub struct Trajectory {
    start: JointTargets,
    target: JointTargets,
    duration_s: f64,
    warp: Warp,
    /// The geodesic from the start orientation to the target one, as a rotation
    /// vector in the *start's* frame. Precomputed: it costs a square root and
    /// an inverse trigonometric call, and the sample path runs every tick.
    rotvec_rel: Vector3<f64>,
}

impl Trajectory {
    /// Shape a move from `start` to `target` over `duration`.
    ///
    /// # Errors
    ///
    /// [`TrajectoryError::NonPositiveDuration`] for a zero duration, and
    /// [`TrajectoryError::NonFinite`] if either endpoint carries a non-finite
    /// number.
    pub fn new(
        start: &JointTargets,
        target: &JointTargets,
        duration: Duration,
        warp: Warp,
    ) -> Result<Self, TrajectoryError> {
        if duration.is_zero() {
            return Err(TrajectoryError::NonPositiveDuration);
        }
        if !start.is_finite() || !target.is_finite() {
            return Err(TrajectoryError::NonFinite);
        }
        // The relative rotation's scaled axis carries an angle in [0, π]: of
        // the two ways round the same pair of orientations, the shorter one.
        let relative = start.head_pose_body.rotation.inverse() * target.head_pose_body.rotation;
        Ok(Self {
            start: *start,
            target: *target,
            duration_s: duration.as_secs_f64(),
            warp,
            rotvec_rel: relative.scaled_axis(),
        })
    }

    /// The command set this path ends at.
    #[must_use]
    pub fn target(&self) -> &JointTargets {
        &self.target
    }

    /// The command set this path started from.
    #[must_use]
    pub fn start(&self) -> &JointTargets {
        &self.start
    }

    /// The path's duration, seconds.
    #[must_use]
    pub fn duration_s(&self) -> f64 {
        self.duration_s
    }

    /// The shape this path was built with.
    #[must_use]
    pub fn warp(&self) -> Warp {
        self.warp
    }

    /// Whether `t` is at or past the end.
    #[must_use]
    pub fn done(&self, t: Duration) -> bool {
        t.as_secs_f64() >= self.duration_s
    }

    /// The command set at elapsed time `t`, written into `out`.
    ///
    /// At or past the duration this is the target's own bits, so a move that
    /// ran to completion commands exactly what was asked for and a subsequent
    /// move chains from it without a step.
    pub fn sample(&self, t: Duration, out: &mut JointTargets) {
        if self.done(t) {
            *out = self.target;
            return;
        }
        // Normalised time. `done` above already excluded the upper end; the cap
        // covers the rounding in the division alone, and is on elapsed time,
        // never on a commanded quantity.
        let u = (t.as_secs_f64() / self.duration_s).clamp(0.0, 1.0);
        let s = self.warp.progress(u);

        let start_t = self.start.head_pose_body.translation.vector;
        let target_t = self.target.head_pose_body.translation.vector;
        out.head_pose_body = Isometry3::from_parts(
            Translation3::from(start_t + (target_t - start_t) * s),
            self.start.head_pose_body.rotation
                * UnitQuaternion::from_scaled_axis(self.rotvec_rel * s),
        );
        out.body_yaw = lerp(self.start.body_yaw, self.target.body_yaw, s);
        out.antennas = [
            lerp(self.start.antennas[0], self.target.antennas[0], s),
            lerp(self.start.antennas[1], self.target.antennas[1], s),
        ];
    }
}

/// `a` at `s = 0`, exactly; `b` at `s = 1`, to within a rounding.
///
/// The exact endpoint that matters is the target's, and `sample` takes that
/// from the target's own bits rather than from this.
fn lerp(a: f64, b: f64, s: f64) -> f64 {
    a + (b - a) * s
}

#[cfg(test)]
mod tests {
    use super::*;
    use nalgebra::{Quaternion, Vector3};
    use reachy_kin::stow_head_pose;

    fn secs(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    fn neutral() -> JointTargets {
        JointTargets::default()
    }

    fn stow() -> JointTargets {
        JointTargets {
            head_pose_body: stow_head_pose(),
            body_yaw: 0.3,
            antennas: [-3.05, 3.05],
        }
    }

    fn bits(t: &JointTargets) -> Vec<u64> {
        let mut out: Vec<u64> = t
            .head_pose_body
            .translation
            .vector
            .iter()
            .map(|c| c.to_bits())
            .collect();
        out.extend(t.head_pose_body.rotation.coords.iter().map(|c| c.to_bits()));
        out.push(t.body_yaw.to_bits());
        out.extend(t.antennas.iter().map(|a| a.to_bits()));
        out
    }

    /// The endpoint is not merely close: past the duration the sample is the
    /// target's own bit pattern, so a completed move commands what was asked.
    #[test]
    fn endpoint_is_bitwise_the_target() {
        let (a, b) = (neutral(), stow());
        let traj = Trajectory::new(&a, &b, secs(2.0), Warp::MinJerk).expect("valid move");
        for t in [secs(2.0), secs(2.000_000_001), secs(60.0)] {
            let mut out = JointTargets::default();
            traj.sample(t, &mut out);
            assert_eq!(bits(&out), bits(&b), "at {t:?}");
            assert!(traj.done(t));
        }
    }

    /// And the start is reproduced at zero, so nothing jumps on the first tick
    /// of a move.
    #[test]
    fn zero_time_reproduces_the_start() {
        let (a, b) = (neutral(), stow());
        for warp in [Warp::MinJerk, Warp::Linear] {
            let traj = Trajectory::new(&a, &b, secs(2.0), warp).expect("valid move");
            let mut out = JointTargets::default();
            traj.sample(Duration::ZERO, &mut out);
            assert_eq!(bits(&out), bits(&a), "{warp:?}");
            assert!(!traj.done(Duration::ZERO));
        }
    }

    /// Consecutive moves chain: the second starts at the first's endpoint
    /// exactly, so a sequence of moves accumulates no residue.
    #[test]
    fn consecutive_moves_chain_without_a_step() {
        let (a, b) = (neutral(), stow());
        let first = Trajectory::new(&a, &b, secs(2.0), Warp::MinJerk).expect("valid move");
        let mut landed = JointTargets::default();
        first.sample(secs(2.0), &mut landed);

        let second = Trajectory::new(&landed, &a, secs(1.5), Warp::MinJerk).expect("valid move");
        let mut resumed = JointTargets::default();
        second.sample(Duration::ZERO, &mut resumed);
        assert_eq!(bits(&resumed), bits(&landed));
    }

    #[test]
    fn zero_duration_is_refused() {
        let (a, b) = (neutral(), stow());
        assert_eq!(
            Trajectory::new(&a, &b, Duration::ZERO, Warp::MinJerk),
            Err(TrajectoryError::NonPositiveDuration)
        );
        // One nanosecond is a terrible move, but it is a well-defined one.
        assert!(Trajectory::new(&a, &b, Duration::from_nanos(1), Warp::MinJerk).is_ok());
    }

    /// A non-finite endpoint is refused at construction, whichever end it is
    /// on and whichever component carries it.
    #[test]
    fn non_finite_endpoints_are_refused() {
        let good = neutral();
        for bad_value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut translation = neutral();
            translation.head_pose_body.translation.vector.z = bad_value;
            let mut rotation = neutral();
            rotation.head_pose_body.rotation =
                UnitQuaternion::new_unchecked(Quaternion::new(1.0, bad_value, 0.0, 0.0));
            let mut yaw = neutral();
            yaw.body_yaw = bad_value;
            let mut antenna = neutral();
            antenna.antennas[0] = bad_value;

            for bad in [translation, rotation, yaw, antenna] {
                assert_eq!(
                    Trajectory::new(&bad, &good, secs(1.0), Warp::MinJerk),
                    Err(TrajectoryError::NonFinite),
                    "bad start with {bad_value}"
                );
                assert_eq!(
                    Trajectory::new(&good, &bad, secs(1.0), Warp::MinJerk),
                    Err(TrajectoryError::NonFinite),
                    "bad target with {bad_value}"
                );
            }
        }
    }

    /// The minimum-jerk quintic's defining properties: it spans exactly [0, 1]
    /// and leaves and arrives with zero velocity and zero acceleration.
    #[test]
    fn min_jerk_boundary_conditions() {
        assert_eq!(Warp::MinJerk.progress(0.0), 0.0);
        assert!((Warp::MinJerk.progress(1.0) - 1.0).abs() < 1e-15);
        assert!((Warp::MinJerk.progress(0.5) - 0.5).abs() < 1e-15);

        let h = 1e-4;
        let d1 =
            |u: f64| (Warp::MinJerk.progress(u + h) - Warp::MinJerk.progress(u - h)) / (2.0 * h);
        let d2 = |u: f64| {
            (Warp::MinJerk.progress(u + h) - 2.0 * Warp::MinJerk.progress(u)
                + Warp::MinJerk.progress(u - h))
                / (h * h)
        };
        for end in [0.0, 1.0] {
            assert!(d1(end).abs() < 1e-6, "velocity at {end}: {}", d1(end));
            assert!(d2(end).abs() < 1e-6, "acceleration at {end}: {}", d2(end));
        }
        // Peak rate is 1.875 — the quintic's cost for those boundary
        // conditions, and what the per-tick step guard is sized against.
        assert!((d1(0.5) - 1.875).abs() < 1e-6, "peak rate {}", d1(0.5));
    }

    /// Monotone and inside [0, 1] throughout, both warps: the path never
    /// overshoots its endpoints or doubles back.
    #[test]
    fn warps_are_monotone_and_bounded() {
        for warp in [Warp::MinJerk, Warp::Linear] {
            let mut previous = f64::NEG_INFINITY;
            for step in 0..=1000 {
                let u = f64::from(step) / 1000.0;
                let s = warp.progress(u);
                assert!((0.0..=1.0).contains(&s), "{warp:?} at {u}: {s}");
                assert!(s >= previous, "{warp:?} not monotone at {u}");
                previous = s;
            }
            assert!((warp.progress(1.0) - 1.0).abs() < 1e-15);
        }
    }

    #[test]
    fn linear_warp_is_the_identity() {
        for step in 0..=100 {
            let u = f64::from(step) / 100.0;
            assert_eq!(Warp::Linear.progress(u), u);
        }
    }

    /// One warp scalar drives all four components: at any sample, the fraction
    /// of the translation covered, of the yaw, of each antenna and of the
    /// rotation angle are the same number. A per-component phase difference
    /// would be invisible at the endpoints and wrong everywhere between.
    #[test]
    fn one_warp_scalar_drives_every_component() {
        let (a, b) = (neutral(), stow());
        let traj = Trajectory::new(&a, &b, secs(2.0), Warp::MinJerk).expect("valid move");
        let total_translation =
            (b.head_pose_body.translation.vector - a.head_pose_body.translation.vector).norm();
        let total_rotation =
            (a.head_pose_body.rotation.inverse() * b.head_pose_body.rotation).angle();
        assert!(total_translation > 0.04 && total_rotation > 0.4, "moves");

        let mut out = JointTargets::default();
        for step in 1..20 {
            let t = 2.0 * f64::from(step) / 20.0;
            traj.sample(secs(t), &mut out);
            let s = Warp::MinJerk.progress(t / 2.0);

            let done_translation = (out.head_pose_body.translation.vector
                - a.head_pose_body.translation.vector)
                .norm()
                / total_translation;
            let done_rotation = (a.head_pose_body.rotation.inverse() * out.head_pose_body.rotation)
                .angle()
                / total_rotation;
            let done_yaw = (out.body_yaw - a.body_yaw) / (b.body_yaw - a.body_yaw);
            let done_right = (out.antennas[0] - a.antennas[0]) / (b.antennas[0] - a.antennas[0]);
            let done_left = (out.antennas[1] - a.antennas[1]) / (b.antennas[1] - a.antennas[1]);

            for (name, fraction) in [
                ("translation", done_translation),
                ("rotation", done_rotation),
                ("yaw", done_yaw),
                ("right antenna", done_right),
                ("left antenna", done_left),
            ] {
                assert!(
                    (fraction - s).abs() < 1e-12,
                    "{name} at {t}s: {fraction} vs {s}"
                );
            }
        }
    }

    /// Rotation follows the geodesic: the interpolated orientation lies on the
    /// single-axis arc between the endpoints, not on a component-wise blend.
    #[test]
    fn rotation_follows_the_geodesic() {
        let mut a = neutral();
        a.head_pose_body.rotation = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.2_f64);
        let mut b = neutral();
        b.head_pose_body.rotation = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.5_f64)
            * UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.3_f64);

        let traj = Trajectory::new(&a, &b, secs(1.0), Warp::Linear).expect("valid move");
        let axis = (a.head_pose_body.rotation.inverse() * b.head_pose_body.rotation)
            .axis()
            .expect("the endpoints differ");
        let mut out = JointTargets::default();
        for step in 0..=10 {
            let s = f64::from(step) / 10.0;
            traj.sample(secs(s), &mut out);
            let relative = a.head_pose_body.rotation.inverse() * out.head_pose_body.rotation;
            if let Some(sample_axis) = relative.axis() {
                assert!(
                    (sample_axis.dot(&axis) - 1.0).abs() < 1e-12,
                    "axis wandered at s = {s}"
                );
            }
            let expected =
                s * (a.head_pose_body.rotation.inverse() * b.head_pose_body.rotation).angle();
            assert!(
                (relative.angle() - expected).abs() < 1e-12,
                "angle at s = {s}: {} vs {expected}",
                relative.angle()
            );
        }
    }

    /// The geodesic takes the short way round. Two orientations 350° apart on
    /// one axis interpolate through the 10° arc; taking the long way would
    /// sweep the head through most of a turn to reach a pose beside it.
    #[test]
    fn the_geodesic_takes_the_short_way() {
        let mut a = neutral();
        a.head_pose_body.rotation = UnitQuaternion::identity();
        let mut b = neutral();
        b.head_pose_body.rotation =
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 350.0_f64.to_radians());

        let traj = Trajectory::new(&a, &b, secs(1.0), Warp::Linear).expect("valid move");
        let mut out = JointTargets::default();
        traj.sample(secs(0.5), &mut out);
        let swept = out.head_pose_body.rotation.angle().to_degrees();
        assert!((swept - 5.0).abs() < 1e-9, "swept {swept}°");
    }

    /// Sampling is a total overwrite of the output: nothing of a previous
    /// sample survives in it, and the same time twice gives the same bits.
    #[test]
    fn sampling_overwrites_and_repeats_exactly() {
        let (a, b) = (neutral(), stow());
        let traj = Trajectory::new(&a, &b, secs(2.0), Warp::MinJerk).expect("valid move");

        let mut fresh = JointTargets::default();
        traj.sample(secs(0.7), &mut fresh);

        let mut dirty = stow();
        dirty.antennas = [9.0, -9.0];
        dirty.body_yaw = 42.0;
        traj.sample(secs(0.7), &mut dirty);
        assert_eq!(bits(&dirty), bits(&fresh));
    }

    /// `done` is the same comparison the sample path takes, so a caller told
    /// the move has finished is handed the target's own bits.
    ///
    /// Only that direction is asserted. The converse is false and would be a
    /// brittle thing to demand: the quintic's progress rounds to exactly 1.0 a
    /// few nanoseconds short of the end, and the interpolation there reproduces
    /// the target bit for bit without any help from the snap.
    #[test]
    fn done_agrees_with_the_endpoint_snap() {
        let (a, b) = (neutral(), stow());
        let traj = Trajectory::new(&a, &b, secs(1.25), Warp::MinJerk).expect("valid move");
        let mut out = JointTargets::default();
        for nanos in [0_u64, 1, 1_249_999_999, 1_250_000_000, 1_250_000_001] {
            let t = Duration::from_nanos(nanos);
            traj.sample(t, &mut out);
            assert_eq!(traj.done(t), nanos >= 1_250_000_000, "at {nanos} ns");
            if traj.done(t) {
                assert_eq!(bits(&out), bits(&b), "at {nanos} ns");
            }
        }
    }

    #[test]
    fn accessors_report_what_was_built() {
        let (a, b) = (neutral(), stow());
        let traj = Trajectory::new(&a, &b, secs(3.5), Warp::Linear).expect("valid move");
        assert_eq!(bits(traj.start()), bits(&a));
        assert_eq!(bits(traj.target()), bits(&b));
        assert_eq!(traj.duration_s(), 3.5);
        assert_eq!(traj.warp(), Warp::Linear);
    }
}
