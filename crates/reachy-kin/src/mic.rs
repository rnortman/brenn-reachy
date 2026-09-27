//! The audio device's azimuth, turned into a world bearing of the talker.
//!
//! The frame is right-handed, z up, x forward, so +y is the robot's left and
//! positive yaw turns the head to its left.
//!
//! The array is four microphones in line on top of the head. Its axis runs
//! along the head's +y, and its 0° end is the robot's left.
//!
//! The chip reports an azimuth α ∈ [0, π]: the angle between the source
//! direction and the array axis. That is a cone about the axis, so front and
//! back fold together, and so do up and down; no elevation is reported. The
//! talker's elevation is therefore the caller's assumption, handed in.
//!
//! Nothing is clamped: every outcome is a bearing or a typed refusal. The frames
//! and the array's convention are recorded in `docs/frames.md`.
// TODO(frame-aware-motion): the head attitude handed in is whatever estimate the
// caller holds when it asks, not the one at the reading's own instant. Matching
// the two needs the audio device's host-time projection related to `SyncTime`.

use core::f64::consts::{FRAC_PI_2, PI};

use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use thiserror::Error;

use crate::ik::wrap_to_pi;
use crate::yaw::body_to_world;

/// How far the array axis may be from unit length and still be read as a direction.
const AXIS_UNIT_TOLERANCE: f64 = 1e-9;

/// An azimuth that yields no bearing.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum MicError {
    /// An input was NaN or infinite: the azimuth, the elevation, or a component of the axis.
    #[error("a non-finite input: azimuth, elevation or array axis")]
    NonFinite,
    /// The azimuth is outside [0, π], the elevation outside (−π/2, π/2), or the
    /// axis is not a unit vector.
    #[error(
        "an input outside its domain: azimuth in [0, π], elevation in (−π/2, π/2), a unit array axis"
    )]
    OutOfRange,
    /// The axis has no horizontal extent: every bearing reads the same azimuth.
    #[error("the array axis is vertical, so the azimuth carries no bearing")]
    ArrayVertical,
    /// The reading lies closer to end-fire than any source at the assumed
    /// elevation can. `edge` is the world bearing of the nearest consistent
    /// solution — the cone's end-fire edge on that side, radians in (−π, π]. A
    /// bound, not a bearing: the reading is refused as a solution and the edge
    /// reported.
    #[error(
        "the azimuth is past end-fire for the assumed elevation; the {} edge is at {edge} rad",
        side(*.left)
    )]
    BeyondEndfire {
        /// The reading is past the 0° end, the robot's left.
        left: bool,
        /// The world bearing of the cone's end-fire edge on that side, radians.
        edge: f64,
    },
}

/// The word for an end-fire side.
fn side(left: bool) -> &'static str {
    if left { "left" } else { "right" }
}

/// The array axis in the world frame: the head's +y under the body-frame head
/// attitude and the body yaw, composed by [`body_to_world`].
///
/// Unit by construction. A non-finite `body_yaw` yields a non-finite vector,
/// which [`bearing_from_azimuth`] refuses as [`MicError::NonFinite`].
#[must_use]
pub fn array_axis_world(head_quat_body: &UnitQuaternion<f64>, body_yaw: f64) -> Vector3<f64> {
    let head = Isometry3::from_parts(Translation3::identity(), *head_quat_body);
    body_to_world(&head, body_yaw).rotation * Vector3::y()
}

/// Whether `radians` is a talker elevation [`bearing_from_azimuth`] takes:
/// finite, and strictly between −π/2 and π/2.
///
/// The one statement of that domain, so a caller validating an elevation
/// once, ahead of every reading, refuses exactly what this module would.
#[must_use]
pub fn talker_elevation_in_domain(radians: f64) -> bool {
    radians.is_finite() && radians.abs() < FRAC_PI_2
}

/// The world bearing of the talker from the chip's azimuth on the head-mounted
/// array.
///
/// `azimuth` is the chip's α, radians in [0, π], the angle between the source
/// direction and the array axis. `talker_elevation` is the assumed elevation φ
/// of the talker above the horizontal, radians in (−π/2, π/2).
/// `array_axis_world` is the unit axis in the world frame, toward the array's 0°
/// end, as [`array_axis_world`] gives it.
///
/// The result is radians in (−π, π], from world x, positive to the robot's left.
/// It is exact for any head attitude: the intersection of the chip's cone with
/// the talker's elevation, on the array's front side. The cone meets that
/// elevation at two bearings symmetric about the axis — the chip's front/back
/// fold — and the front one is returned, the gaze being a quarter turn clockwise
/// of the 0° end.
///
/// # Errors
///
/// - [`MicError::NonFinite`]: any input, or any component of the axis, is NaN or
///   infinite.
/// - [`MicError::OutOfRange`]: the azimuth is outside [0, π], the elevation
///   outside (−π/2, π/2), or the axis more than 1e-9 off unit length.
/// - [`MicError::ArrayVertical`]: the axis has no horizontal extent.
/// - [`MicError::BeyondEndfire`]: the reading is closer to end-fire than any
///   source at the assumed elevation can be. `left` names the side and `edge` is
///   the cone's end-fire edge on it, which turns with the head and the body like
///   every other bearing.
pub fn bearing_from_azimuth(
    azimuth: f64,
    talker_elevation: f64,
    array_axis_world: &Vector3<f64>,
) -> Result<f64, MicError> {
    let axis = array_axis_world;
    if ![azimuth, talker_elevation, axis.x, axis.y, axis.z]
        .iter()
        .all(|value| value.is_finite())
    {
        return Err(MicError::NonFinite);
    }
    // Every value below is finite, so each comparison is total.
    if !(0.0..=PI).contains(&azimuth)
        || !talker_elevation_in_domain(talker_elevation)
        || (axis.norm() - 1.0).abs() > AXIS_UNIT_TOLERANCE
    {
        return Err(MicError::OutOfRange);
    }

    // `cos α = a · d` is `a cos θ + b sin θ = c` in the bearing θ.
    let (sin_phi, cos_phi) = talker_elevation.sin_cos();
    let a = cos_phi * axis.x;
    let b = cos_phi * axis.y;
    let c = azimuth.cos() - axis.z * sin_phi;
    let r = a.hypot(b);
    if r <= 0.0 {
        return Err(MicError::ArrayVertical);
    }

    // The world yaw of the array's 0° end.
    let psi = b.atan2(a);
    if c > r {
        return Err(MicError::BeyondEndfire {
            left: true,
            edge: wrap_to_pi(psi),
        });
    }
    if c < -r {
        return Err(MicError::BeyondEndfire {
            left: false,
            edge: wrap_to_pi(psi - PI),
        });
    }
    // `|c| ≤ r` here, and a correctly rounded division is monotone, so the ratio
    // is inside [−1, 1] and `acos` is finite.
    let delta = (c / r).acos();
    Ok(wrap_to_pi(psi - delta))
}

#[cfg(test)]
mod tests {
    use super::*;

    const PHI: f64 = 27.0_f64.to_radians();
    const TOL: f64 = 1e-12;

    fn deg(x: f64) -> f64 {
        x.to_radians()
    }

    /// The unit direction toward a talker at world bearing `theta` and elevation `phi`.
    fn toward(theta: f64, phi: f64) -> Vector3<f64> {
        Vector3::new(phi.cos() * theta.cos(), phi.cos() * theta.sin(), phi.sin())
    }

    /// The azimuth the chip reads for that talker on an array along `axis`.
    fn azimuth_of(axis: &Vector3<f64>, theta: f64, phi: f64) -> f64 {
        axis.dot(&toward(theta, phi)).acos()
    }

    fn level() -> Vector3<f64> {
        array_axis_world(&UnitQuaternion::identity(), 0.0)
    }

    fn close(got: f64, want: f64, tol: f64) {
        assert!((got - want).abs() < tol, "got {got}, want {want}");
    }

    fn close_vec(got: &Vector3<f64>, want: &Vector3<f64>, tol: f64) {
        assert!((got - want).norm() < tol, "got {got:?}, want {want:?}");
    }

    fn beyond(got: Result<f64, MicError>, left: bool, edge: f64) {
        match got {
            Err(MicError::BeyondEndfire {
                left: got_left,
                edge: got_edge,
            }) => {
                assert_eq!(got_left, left);
                close(got_edge, edge, TOL);
            }
            other => panic!("expected BeyondEndfire, got {other:?}"),
        }
    }

    fn yawed(head_yaw: f64) -> UnitQuaternion<f64> {
        UnitQuaternion::from_euler_angles(0.0, 0.0, head_yaw)
    }

    #[test]
    fn a_level_unyawed_head_puts_the_axis_to_its_left() {
        close_vec(&level(), &Vector3::new(0.0, 1.0, 0.0), TOL);
    }

    #[test]
    fn broadside_on_a_level_head_is_dead_ahead() {
        close(
            bearing_from_azimuth(FRAC_PI_2, PHI, &level()).unwrap(),
            0.0,
            TOL,
        );
    }

    #[test]
    fn end_fire_at_zero_elevation_is_a_quarter_turn() {
        close(
            bearing_from_azimuth(0.0, 0.0, &level()).unwrap(),
            FRAC_PI_2,
            TOL,
        );
        close(
            bearing_from_azimuth(PI, 0.0, &level()).unwrap(),
            -FRAC_PI_2,
            TOL,
        );
    }

    #[test]
    fn the_fold_is_symmetric() {
        for alpha in [deg(40.0), deg(60.0), deg(85.0)] {
            let near = bearing_from_azimuth(alpha, PHI, &level()).unwrap();
            let far = bearing_from_azimuth(PI - alpha, PHI, &level()).unwrap();
            close(far, -near, TOL);
        }
    }

    #[test]
    fn the_worked_figure() {
        let got = bearing_from_azimuth(deg(60.0), deg(27.0), &level()).unwrap();
        close(got, (0.5 / deg(27.0).cos()).asin(), TOL);
        close(got.to_degrees(), 34.1, 0.05);
    }

    #[test]
    fn a_rolled_head_reads_a_talker_where_it_is() {
        let head = UnitQuaternion::from_euler_angles(deg(26.0), 0.0, 0.0);
        let axis = array_axis_world(&head, 0.0);
        close_vec(
            &axis,
            &Vector3::new(0.0, deg(26.0).cos(), deg(26.0).sin()),
            TOL,
        );
        let ahead = bearing_from_azimuth(azimuth_of(&axis, 0.0, PHI), PHI, &axis).unwrap();
        close(ahead, 0.0, 1e-9);
        let off = bearing_from_azimuth(azimuth_of(&axis, deg(30.0), PHI), PHI, &axis).unwrap();
        close(off, deg(30.0), 1e-9);
    }

    #[test]
    fn a_pitched_head_reads_as_a_level_one() {
        let head = UnitQuaternion::from_euler_angles(0.0, deg(20.0), 0.0);
        let axis = array_axis_world(&head, 0.0);
        close_vec(&axis, &Vector3::new(0.0, 1.0, 0.0), TOL);
        close(
            bearing_from_azimuth(deg(60.0), PHI, &axis).unwrap(),
            bearing_from_azimuth(deg(60.0), PHI, &level()).unwrap(),
            TOL,
        );
    }

    #[test]
    fn head_and_body_yaw_compose() {
        let axis = array_axis_world(&yawed(deg(30.0)), deg(20.0));
        close_vec(
            &axis,
            &Vector3::new(-deg(50.0).sin(), deg(50.0).cos(), 0.0),
            TOL,
        );
        for theta in [deg(20.0), deg(50.0), deg(80.0)] {
            let got = bearing_from_azimuth(azimuth_of(&axis, theta, PHI), PHI, &axis).unwrap();
            close(got, theta, 1e-9);
        }
    }

    #[test]
    fn a_bearing_past_the_half_turn_wraps() {
        let axis = array_axis_world(&yawed(deg(45.0)), deg(150.0));
        for theta in [deg(170.0), deg(-150.0)] {
            let got = bearing_from_azimuth(azimuth_of(&axis, theta, PHI), PHI, &axis).unwrap();
            close(got, theta, 1e-9);
            assert!(got > -PI && got <= PI, "{got} is outside (−π, π]");
        }
    }

    #[test]
    fn beyond_end_fire_reports_the_side_and_the_edge() {
        beyond(bearing_from_azimuth(0.0, PHI, &level()), true, FRAC_PI_2);
        beyond(bearing_from_azimuth(PI, PHI, &level()), false, -FRAC_PI_2);

        let axis = array_axis_world(&yawed(deg(30.0)), deg(20.0));
        beyond(bearing_from_azimuth(PI, PHI, &axis), false, deg(-40.0));
        beyond(bearing_from_azimuth(0.0, PHI, &axis), true, deg(140.0));
    }

    #[test]
    fn a_vertical_axis_is_refused() {
        assert_eq!(
            bearing_from_azimuth(FRAC_PI_2, PHI, &Vector3::new(0.0, 0.0, 1.0)),
            Err(MicError::ArrayVertical)
        );
    }

    #[test]
    fn non_finite_inputs_are_refused() {
        let nan_z = Vector3::new(0.0, 1.0, f64::NAN);
        let nan_yaw = array_axis_world(&UnitQuaternion::identity(), f64::NAN);
        for got in [
            bearing_from_azimuth(f64::NAN, PHI, &level()),
            bearing_from_azimuth(FRAC_PI_2, f64::INFINITY, &level()),
            bearing_from_azimuth(FRAC_PI_2, PHI, &nan_z),
            bearing_from_azimuth(FRAC_PI_2, PHI, &nan_yaw),
        ] {
            assert_eq!(got, Err(MicError::NonFinite));
        }
    }

    #[test]
    fn inputs_outside_their_domain_are_refused() {
        for got in [
            bearing_from_azimuth(-1e-9, PHI, &level()),
            bearing_from_azimuth(PI + 1e-9, PHI, &level()),
            bearing_from_azimuth(FRAC_PI_2, FRAC_PI_2, &level()),
            bearing_from_azimuth(FRAC_PI_2, -FRAC_PI_2, &level()),
            bearing_from_azimuth(FRAC_PI_2, PHI, &Vector3::new(0.0, 2.0, 0.0)),
        ] {
            assert_eq!(got, Err(MicError::OutOfRange));
        }
    }

    #[test]
    fn the_elevation_domain_is_the_open_quarter_turn() {
        let below = f64::from_bits(FRAC_PI_2.to_bits() - 1);
        for radians in [0.0, PHI, -PHI, below, -below] {
            assert!(talker_elevation_in_domain(radians), "{radians}");
        }
        for radians in [FRAC_PI_2, -FRAC_PI_2, 2.0, f64::NAN, f64::INFINITY] {
            assert!(!talker_elevation_in_domain(radians), "{radians}");
        }
    }
}
