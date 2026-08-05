//! The runtime geometry structs and the named head poses.
//!
//! All public poses are the **head pose in the base (foot) frame** — the fixed
//! foot below the yaw joint. The solvers take the pose relative to the body at
//! zero yaw; body frame and base frame coincide there, and nonzero yaw composes
//! before the solve.
//!
//! Geometry is a struct rather than a set of constants because an individual
//! unit's link dimensions ultimately have to be measured and swapped in. The
//! defaults are the vendor's nominal model.

use nalgebra::{Isometry3, Matrix3, Point3, Rotation3, Translation3, UnitQuaternion, Vector3};

use crate::baked;

/// Which of the two circle–sphere intersection roots a leg occupies.
///
/// The loop closure reduces to `A·cosθ + B·sinθ = C`, whose solutions are
/// `atan2(B, A) ± acos(C / √(A²+B²))`. This picks the sign.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BranchSign {
    /// The `+acos` root.
    Plus,
    /// The `−acos` root.
    Minus,
}

impl BranchSign {
    /// `+1.0` or `−1.0`, the multiplier on the `acos` term.
    #[must_use]
    pub fn as_f64(self) -> f64 {
        match self {
            BranchSign::Plus => 1.0,
            BranchSign::Minus => -1.0,
        }
    }
}

/// One leg: its crank frame, its platform anchor, and its root.
#[derive(Clone, Debug)]
pub struct LegGeometry {
    /// Base frame → crank frame. The crank rotates about this frame's local z
    /// axis and its arm lies in the local xy plane, so the crank tip at angle θ
    /// is `(crank_len·cosθ, crank_len·sinθ, 0)` in crank coordinates.
    pub t_base_to_crank: Isometry3<f64>,
    /// Rod anchor on the moving platform, in the head frame.
    pub anchor_head: Point3<f64>,
    /// Which root this leg occupies.
    pub branch: BranchSign,
}

/// The whole head linkage: two link lengths and six legs.
///
/// TODO(geometry-fit): the defaults are the vendor's nominal model, and a
/// second parameter set differing by a few millimetres has been written down for
/// this linkage. Millimetres are large against the clearance the crank stops
/// leave at the top of travel, so this unit's dimensions want fitting on the
/// bench and substituting here.
#[derive(Clone, Debug)]
pub struct HeadGeometry {
    /// Crank (motor arm) length, metres.
    pub crank_len: f64,
    /// Rod length, metres.
    pub rod_len: f64,
    /// The six legs, in servo order 1..=6.
    pub legs: [LegGeometry; 6],
}

impl Default for HeadGeometry {
    fn default() -> Self {
        Self {
            crank_len: baked::CRANK_LEN,
            rod_len: baked::ROD_LEN,
            legs: core::array::from_fn(|leg| LegGeometry {
                t_base_to_crank: isometry_from_rows(&baked::BASE_TO_CRANK[leg]),
                anchor_head: Point3::from(baked::ANCHOR_HEAD[leg]),
                branch: baked::BRANCH_SIGNS[leg],
            }),
        }
    }
}

/// The top three rows of a 4×4 homogeneous transform as an isometry.
///
/// The rotation block is taken as-is. It has to be a *proper* rotation —
/// orthonormal and of determinant +1 — which `baked::tests` asserts of the
/// constants and the assertion below rechecks of anything substituted for them.
/// A reflection would pass an orthonormality check and then convert silently
/// into some unrelated rotation, giving a mirrored mechanism that solves,
/// stays in window, and is wrong.
fn isometry_from_rows(rows: &[[f64; 4]; 3]) -> Isometry3<f64> {
    let rotation = Matrix3::new(
        rows[0][0], rows[0][1], rows[0][2], //
        rows[1][0], rows[1][1], rows[1][2], //
        rows[2][0], rows[2][1], rows[2][2],
    );
    debug_assert!(
        (rotation.determinant() - 1.0).abs() < 1e-9,
        "rotation block is not a proper rotation: det {}",
        rotation.determinant()
    );
    Isometry3::from_parts(
        Translation3::new(rows[0][3], rows[1][3], rows[2][3]),
        UnitQuaternion::from_rotation_matrix(&Rotation3::from_matrix_unchecked(rotation)),
    )
}

/// The neutral head pose: identity orientation, head origin on the yaw axis at
/// the nominal height.
#[must_use]
pub fn neutral_head_pose() -> Isometry3<f64> {
    Isometry3::translation(0.0, 0.0, baked::HEAD_Z_OFFSET)
}

/// The stow head pose: off-centre, low, and pitched up.
///
/// Adapted from the sleep head pose of the Apache-2.0 `reachy_mini`
/// distribution. It is **not** the bottom of pure vertical travel, and it is
/// not the resting configuration the machine settles into when limp — which
/// one it settles into is a measurement, not a derivation.
#[must_use]
pub fn stow_head_pose() -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(
            baked::STOW_TRANSLATION[0],
            baked::STOW_TRANSLATION[1],
            baked::STOW_TRANSLATION[2],
        ),
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), baked::STOW_PITCH),
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn branch_sign_multipliers() {
        assert_eq!(BranchSign::Plus.as_f64(), 1.0);
        assert_eq!(BranchSign::Minus.as_f64(), -1.0);
    }

    /// Inverting the baked transform recovers the crank origin in the base
    /// frame; the isometry conversion must not disturb that.
    #[test]
    fn default_geometry_crank_origins() {
        let geom = HeadGeometry::default();
        for (leg, l) in geom.legs.iter().enumerate() {
            let origin = l.t_base_to_crank.inverse() * Point3::origin();
            assert!(
                (origin.coords.xy().norm() - 0.038).abs() < 1e-6,
                "leg {}",
                leg + 1
            );
            assert!((origin.z - 0.076_633).abs() < 1e-6, "leg {}", leg + 1);
        }
    }

    /// Every anchor is 30 mm from the head origin, and the neutral pose lifts
    /// all six to the nominal height.
    #[test]
    fn neutral_pose_places_anchors() {
        let geom = HeadGeometry::default();
        let pose = neutral_head_pose();
        for l in &geom.legs {
            assert!((l.anchor_head.coords.norm() - 0.030).abs() < 1e-6);
            let anchor_base = pose * l.anchor_head;
            assert!((anchor_base.z - baked::HEAD_Z_OFFSET).abs() < 1e-6);
        }
    }

    /// Stow is low, off the yaw axis, and pitched — none of which the neutral
    /// pose is.
    #[test]
    fn stow_pose_is_low_and_pitched() {
        let pose = stow_head_pose();
        assert!((pose.translation.z - 0.133).abs() < 1e-12);
        assert!((pose.translation.x + 0.021).abs() < 1e-12);
        let head_z = pose.rotation * Vector3::z();
        let cone = head_z.z.acos();
        assert!((cone - baked::STOW_PITCH).abs() < 1e-9, "cone {cone}");
    }
}
