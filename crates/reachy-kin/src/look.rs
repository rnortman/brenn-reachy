//! Turning a world direction into the targets a look commands.
//!
//! Pure, and it clamps nothing: the head carries the bearing up to its share
//! and the body carries the rest, so the two always sum to the bearing, and a
//! direction the pair cannot reach is the envelope's refusal rather than a
//! saturated command.

use nalgebra::{Isometry3, UnitQuaternion, Vector3};

use crate::geometry::neutral_head_pose;

/// How a look splits a bearing between the head and the body.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LookPolicy {
    /// The most of a bearing the head carries, radians, not negative. The
    /// body carries the rest. At most [`LOOK_HEAD_SHARE_LIMIT`]; the Mover
    /// refuses a configuration past it.
    pub head_share: f64,
}

/// Where a look sends the machine.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LookTargets {
    /// Head pose relative to the body at `body_yaw`.
    pub head_pose_body: Isometry3<f64>,
    /// Body yaw, radians.
    pub body_yaw: f64,
    /// Antenna angles, right then left, radians: always [`LOOK_ANTENNAS`].
    pub antennas: [f64; 2],
}

/// Right, then left; radians. Where a look holds the antennas: `neutral`'s
/// pair, ten degrees off vertical toward each side's outboard, so the rod's own
/// weight holds the gearbox play to one side. `cogs/committed_poses` holds this
/// literal to the committed `neutral` document.
pub const LOOK_ANTENNAS: [f64; 2] = [-0.1745, 0.1745];

/// The most of a bearing a look's head may carry, radians: 54°, one whole
/// degree inside the 55° head-relative yaw cap. A pose composed at the cap
/// itself reads back through the envelope within an ulp of it and lands on
/// either side, so a share there would pass or fail at the cap by rounding;
/// every share up to this figure is inside the grid the tests sweep clean.
pub const LOOK_HEAD_SHARE_LIMIT: f64 = 54.0_f64.to_radians();

/// The head's share of `bearing` and the body's remainder, which sum to it.
///
/// A split of one commanded quantity into two whose sum is that quantity, not
/// a bound on a commanded value: nothing of the bearing is lost.
fn split(bearing: f64, head_share: f64) -> (f64, f64) {
    let head = if bearing.abs() <= head_share {
        bearing
    } else {
        head_share.copysign(bearing)
    };
    (head, bearing - head)
}

/// The targets that face `bearing` (radians from the base's forward, positive
/// to the robot's left) at `elevation` (radians above level).
#[must_use]
pub fn target(bearing: f64, elevation: f64, policy: &LookPolicy) -> LookTargets {
    let (head, body_yaw) = split(bearing, policy.head_share);
    // A yaw about the body vertical, then a pitch about the head's own lateral
    // axis. The frame is right-handed with x forward and z up, so a positive
    // rotation about y drops the nose: a nose-up elevation is `Ry(-elevation)`.
    let rotation = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), head)
        * UnitQuaternion::from_axis_angle(&Vector3::y_axis(), -elevation);
    LookTargets {
        head_pose_body: Isometry3::from_parts(neutral_head_pose().translation, rotation),
        body_yaw,
        antennas: LOOK_ANTENNAS,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::envelope::{EnvelopeConfig, EnvelopeReport, check_envelope};
    use crate::geometry::default_geometry;

    fn deg(d: i32) -> f64 {
        f64::from(d).to_radians()
    }

    fn policy(share_deg: i32) -> LookPolicy {
        LookPolicy {
            head_share: deg(share_deg),
        }
    }

    #[test]
    fn the_head_carries_the_bearing_up_to_its_share_and_the_body_the_rest() {
        let share = deg(30);
        let cases = [
            (0, 0.0, 0.0),
            (20, deg(20), 0.0),
            (30, deg(30), 0.0),
            (50, deg(30), deg(20)),
            (120, deg(30), deg(90)),
            (-20, deg(-20), 0.0),
            (-30, deg(-30), 0.0),
            (-50, deg(-30), deg(-20)),
            (-120, deg(-30), deg(-90)),
        ];
        for (bearing_deg, want_head, want_body) in cases {
            let bearing = deg(bearing_deg);
            let (head, body) = split(bearing, share);
            assert!(
                (head - want_head).abs() < 1e-12,
                "bearing {bearing_deg}°: head {head}, want {want_head}"
            );
            assert!(
                (body - want_body).abs() < 1e-12,
                "bearing {bearing_deg}°: body {body}, want {want_body}"
            );
            assert!(
                (head + body - bearing).abs() < 1e-12,
                "bearing {bearing_deg}°: the split lost {}",
                head + body - bearing
            );
        }
        for bearing_deg in [-120, -50, 0, 20, 50, 120] {
            let bearing = deg(bearing_deg);
            let (head, body) = split(bearing, 0.0);
            assert_eq!(head.abs(), 0.0, "share 0, bearing {bearing_deg}°");
            assert!(
                (body - bearing).abs() < 1e-12,
                "share 0, bearing {bearing_deg}°"
            );
            assert!((head + body - bearing).abs() < 1e-12);
        }
    }

    #[test]
    fn a_look_holds_the_antennas_where_neutral_does() {
        for b in (-170..=170).step_by(10) {
            let t = target(deg(b), 0.471, &policy(30));
            assert_eq!(t.antennas, LOOK_ANTENNAS, "bearing {b}°");
        }
    }

    #[test]
    fn the_attitude_is_the_head_s_yaw_and_a_nose_up_pitch() {
        for (b, e) in [(0, 27), (20, 27), (50, 10), (-50, 30)] {
            let (bearing, elevation) = (deg(b), deg(e));
            let t = target(bearing, elevation, &policy(30));
            let (head, body) = split(bearing, deg(30));
            let (roll, pitch, yaw) = t.head_pose_body.rotation.euler_angles();
            assert!(roll.abs() < 1e-12, "({b}°, {e}°): roll {roll}");
            assert!(
                (pitch + elevation).abs() < 1e-12,
                "({b}°, {e}°): pitch {pitch}, want {}",
                -elevation
            );
            assert!(
                (yaw - head).abs() < 1e-12,
                "({b}°, {e}°): yaw {yaw}, want {head}"
            );
            assert_eq!(
                t.head_pose_body.translation,
                neutral_head_pose().translation,
                "({b}°, {e}°)"
            );
            if e > 0 {
                let nose = t.head_pose_body.rotation * Vector3::x();
                assert!(nose.z > 0.0, "({b}°, {e}°): the nose points {nose:?}");
            }
            assert_eq!(t.body_yaw, body, "({b}°, {e}°)");
        }
    }

    /// Sweeps the head's yaw rather than the share, because the head's yaw is
    /// `min(|bearing|, share)`. The Mover admits no share past
    /// [`LOOK_HEAD_SHARE_LIMIT`], so this covers every bearing at every share
    /// it can run with. The cap itself is left out: a pose composed exactly at
    /// it passes or fails by an ulp.
    #[test]
    fn every_look_in_the_launcher_s_range_passes_the_envelope() {
        const TOP_DEG: i32 = 54;
        let env = EnvelopeConfig::default();
        assert!(
            (deg(TOP_DEG) - LOOK_HEAD_SHARE_LIMIT).abs() < 1e-15,
            "the sweep's top is LOOK_HEAD_SHARE_LIMIT"
        );
        assert!(
            LOOK_HEAD_SHARE_LIMIT < env.relative_yaw_limit,
            "the share's top must stay inside the head-relative yaw cap"
        );
        let wide = policy(TOP_DEG);
        for h in -TOP_DEG..=TOP_DEG {
            for e in 0..=30 {
                let t = target(deg(h), deg(e), &wide);
                assert_eq!(t.body_yaw, 0.0, "({h}°, {e}°): the head carries it all");
                let mut report = EnvelopeReport::default();
                let verdict = check_envelope(
                    default_geometry(),
                    &env,
                    &t.head_pose_body,
                    t.body_yaw,
                    None,
                    &mut report,
                );
                assert!(
                    verdict.is_ok(),
                    "head yaw {h}°, elevation {e}°: {:?} ({report:?})",
                    report.violations
                );
            }
        }
    }

    #[test]
    fn a_look_past_the_launcher_s_range_is_refused() {
        let env = EnvelopeConfig::default();
        let wide = policy(54);
        // The cone alone: at ±55° the case would also carry the yaw cap's ulp.
        for h in [-54, -30, 0, 30, 54] {
            let t = target(deg(h), deg(40), &wide);
            let mut report = EnvelopeReport::default();
            let verdict = check_envelope(
                default_geometry(),
                &env,
                &t.head_pose_body,
                t.body_yaw,
                None,
                &mut report,
            );
            match verdict {
                Err(error) => assert!(error.violations.cone, "head yaw {h}°: {error:?}"),
                Ok(()) => panic!("head yaw {h}° at 40° passed the envelope: {report:?}"),
            }
        }
    }
}
