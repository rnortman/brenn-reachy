//! Composing body yaw with a head pose.
//!
//! The six legs ride on the yawing body: every crank hangs off the link the yaw
//! joint turns. So yawing the body carries the whole platform with it, and
//! holding the head still in the world while the body yaws requires the
//! platform to counter-rotate by the same angle. The solvers in this crate take
//! the head pose **relative to the body**; these two functions are the only
//! place the world frame and the body frame are related, and the sign of that
//! relation is forced by the topology rather than chosen.
//!
//! The yaw axis is the base-frame vertical to within a quarter of a micron, so
//! the composition is a pure rotation with no offset term, and no reference
//! point on the axis needs picking — sliding it along a vertical axis is a
//! no-op.
//!
//! Nothing commands a nonzero body yaw and a world-frame head pose at the same
//! time yet. Both directions exist and are tested anyway, because a sign error
//! here is invisible at zero yaw and grows with it: the wrong sign leaves the
//! head at twice the body's yaw instead of at rest.

use nalgebra::{Isometry3, Vector3};

/// Rotation about the base vertical.
fn rz(angle: f64) -> Isometry3<f64> {
    Isometry3::rotation(Vector3::z() * angle)
}

/// A world-frame head pose, expressed relative to a body at `body_yaw`.
///
/// This is what a world-frame command runs through before the legs are solved.
#[must_use]
pub fn world_to_body(pose_world: &Isometry3<f64>, body_yaw: f64) -> Isometry3<f64> {
    rz(-body_yaw) * pose_world
}

/// A body-relative head pose, expressed in the world frame.
///
/// The inverse of [`world_to_body`] at the same yaw; this is what turns a pose
/// recovered from measured crank angles back into a world-frame answer.
#[must_use]
pub fn body_to_world(pose_body: &Isometry3<f64>, body_yaw: f64) -> Isometry3<f64> {
    rz(body_yaw) * pose_body
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::fk::{FkOptions, forward_kinematics};
    use crate::geometry::HeadGeometry;
    use crate::ik::{LegAngles, inverse_kinematics};
    use nalgebra::{Translation3, UnitQuaternion};

    /// A head pose that is off the yaw axis and tilted, so that a rotation
    /// about the vertical actually moves it. A pose centred on the axis and
    /// level is fixed by every such rotation and would discriminate nothing.
    fn asymmetric_pose() -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(0.004, -0.002, 0.172),
            UnitQuaternion::from_euler_angles(0.02, 0.05, 0.0),
        )
    }

    fn crank_angles(geom: &HeadGeometry, pose: &Isometry3<f64>) -> [f64; 6] {
        let mut angles = LegAngles::default();
        inverse_kinematics(geom, pose, &mut angles).expect("pose is reachable");
        angles.0
    }

    /// The sign test. A head pose that **co-rotates** with the body is at rest
    /// relative to it: the correct wrapper returns one constant body-relative
    /// pose at every yaw, and the cranks do not move.
    ///
    /// The wrong sign is not a small error — it doubles. Applying the composition
    /// in the other direction leaves the head at twice the body's yaw, which is
    /// asserted here so that the failure has a signature rather than just a
    /// disagreement.
    ///
    /// A pose held *fixed* in the world under varying yaw does not discriminate:
    /// under the correct sign its body-relative form counter-rotates and the
    /// cranks change, which is exactly what a sign error also looks like.
    #[test]
    fn a_co_rotating_head_is_at_rest_relative_to_the_body() {
        let geom = HeadGeometry::default();
        let base = asymmetric_pose();
        let reference = crank_angles(&geom, &base);

        for deg in [-160.0, -90.0, -15.0, 15.0, 90.0, 160.0] {
            let yaw = f64::to_radians(deg);
            let pose_world = rz(yaw) * base;

            let body = world_to_body(&pose_world, yaw);
            assert!(
                (body.translation.vector - base.translation.vector).norm() < 1e-12,
                "translation drifted at {deg}°"
            );
            assert!(
                body.rotation.angle_to(&base.rotation) < 1e-12,
                "rotation drifted at {deg}°"
            );
            for (leg, (got, want)) in crank_angles(&geom, &body)
                .iter()
                .zip(reference.iter())
                .enumerate()
            {
                assert!((got - want).abs() < 1e-12, "leg {} at {deg}°", leg + 1);
            }

            // The wrong sign: twice the body yaw, and cranks that move.
            let wrong = body_to_world(&pose_world, yaw);
            let doubled = rz(2.0 * yaw) * base;
            assert!(wrong.rotation.angle_to(&doubled.rotation) < 1e-12);
            assert!(
                (wrong.translation.vector - doubled.translation.vector).norm() < 1e-12,
                "the wrong sign doubles the yaw at {deg}°"
            );
            let mut wrong_angles = LegAngles::default();
            let detected = match inverse_kinematics(&geom, &wrong, &mut wrong_angles) {
                // Past a big enough doubling the head is not reachable at all,
                // which is the same discovery arriving louder.
                Err(_) => true,
                Ok(()) => {
                    wrong_angles
                        .0
                        .iter()
                        .zip(reference.iter())
                        .map(|(got, want)| (got - want).abs())
                        .fold(0.0_f64, f64::max)
                        > 1e-4
                }
            };
            assert!(detected, "the wrong sign is detectable at {deg}°");
        }
    }

    /// The two directions invert each other at the same yaw, including past the
    /// half turn where the underlying rotation wraps.
    #[test]
    fn the_two_directions_invert_each_other() {
        let base = asymmetric_pose();
        for deg in [-200.0, -160.0, 0.0, 37.0, 160.0, 200.0] {
            let yaw = f64::to_radians(deg);
            let round_trip = body_to_world(&world_to_body(&base, yaw), yaw);
            assert!((round_trip.translation.vector - base.translation.vector).norm() < 1e-12);
            assert!(round_trip.rotation.angle_to(&base.rotation) < 1e-12);

            let other_way = world_to_body(&body_to_world(&base, yaw), yaw);
            assert!((other_way.translation.vector - base.translation.vector).norm() < 1e-12);
            assert!(other_way.rotation.angle_to(&base.rotation) < 1e-12);
        }
    }

    /// A world-frame pose survives the whole loop at nonzero yaw: compose in,
    /// solve the legs, recover the pose from the angles, compose back out.
    #[test]
    fn a_world_pose_round_trips_through_the_solvers_at_nonzero_yaw() {
        let geom = HeadGeometry::default();
        let opts = FkOptions::default();
        let base = asymmetric_pose();

        for deg in [-120.0, -45.0, 20.0, 90.0] {
            let yaw = f64::to_radians(deg);
            let pose_world = rz(yaw) * base;

            let body = world_to_body(&pose_world, yaw);
            let mut angles = LegAngles::default();
            inverse_kinematics(&geom, &body, &mut angles).expect("reachable");

            // Seeded a few millimetres and a degree away from the solution.
            let seed = Isometry3::from_parts(
                Translation3::new(0.0, 0.0, 0.177),
                UnitQuaternion::from_euler_angles(0.0, 0.02, 0.0),
            );
            let mut recovered = Isometry3::identity();
            forward_kinematics(&geom, &angles, &seed, &opts, &mut recovered)
                .expect("the pose is recoverable");

            let world = body_to_world(&recovered, yaw);
            assert!(
                (world.translation.vector - pose_world.translation.vector).norm() < 1e-9,
                "translation at {deg}°"
            );
            assert!(
                world.rotation.angle_to(&pose_world.rotation) < 1e-9,
                "rotation at {deg}°"
            );
        }
    }

    /// At zero yaw the body frame is the base frame, and both directions are
    /// the identity — bitwise, not approximately, because callers may compose
    /// on every cycle.
    #[test]
    fn zero_yaw_is_the_identity() {
        let base = asymmetric_pose();
        for pose in [world_to_body(&base, 0.0), body_to_world(&base, 0.0)] {
            assert_eq!(pose.translation.vector, base.translation.vector);
            assert!(pose.rotation.angle_to(&base.rotation) < 1e-15);
        }
    }
}
