//! The envelope: one verdict per commanded pose, naming everything wrong with
//! it.
//!
//! This is the check every command path runs before anything reaches a servo.
//! It covers what no other layer covers:
//!
//! - **Per-leg travel windows.** The pose-to-crank-angle map is defined just as
//!   readily outside a leg's window as inside it — a pose demanding an
//!   out-of-travel angle simply yields that angle. Nothing supplies the limit
//!   implicitly, so it is enforced positively here, against each leg's *own*
//!   asymmetric window: the bottom of vertical travel rests on four of the six
//!   legs, and a symmetric window copied from the other two would drive those
//!   four straight past it.
//! - **Clearance from the singular configurations**, as a floor on the
//!   per-pose toggle margin rather than a tabulated number. The windows hold
//!   the linkage about a millimetre off a singular configuration along pure
//!   vertical travel and do not bound it away from zero anywhere else.
//! - **Body yaw, head-relative yaw and head attitude**, none of which any leg
//!   window expresses: the windows alone permit around 102° of relative head
//!   yaw at nominal height, some 47° past the tightest bound the vendor's own
//!   solvers enforce.
//!
//! The antennas are not here. They are free rotors turning a full circle with no
//! hard stop, nothing behind them for a bound to guard, and no linkage to place
//! them in a pose; what a commanded antenna angle has to be is representable,
//! which the layer that turns a direction into a goal decides.
//!
//! A violation is a typed error naming every failing check. Never a clamp,
//! never a saturated angle: a pose outside the envelope is refused, not
//! approximated.
//!
//! ## Where the windows come from
//!
//! The whole-degree crank limits are baked in from the vendor's simulation
//! model, which agrees with all three of its robot descriptions. They are
//! deliberately **not** taken from the geometry file this crate's transforms
//! come from: that file carries a per-leg limit field of ±180° on all six legs,
//! a generator artifact of the tool that wrote it. Reading geometry and limits
//! from the same file yields ±180° windows, which in this mechanism is no
//! protection at all.
//!
//! ## The margin baseline
//!
//! The clearance floor is a floor on *commanded* poses, but the machine can
//! come to rest below it — one documented resting configuration sits at 0.141 mm
//! of clearance, a twentieth of the floor. Refusing every command from there
//! would leave the head stuck at its tightest. So a caller that knows the
//! present pose's clearance passes it as `margin_baseline`, and a pose that
//! strictly increases clearance is admissible even below the floor. Motion
//! toward a singular configuration stays refused at every margin.

use nalgebra::Isometry3;
use thiserror::Error;

use crate::baked;
use crate::geometry::{HeadGeometry, cone_angle};
use crate::ik::{LegAngles, min_margin, solve_leg};

/// The bounds a commanded pose is checked against.
///
/// Every field is a bound on a commanded quantity, in radians or metres. They
/// are configuration rather than constants because the working caps are
/// operating choices — the bench tightens body yaw well inside the mechanical
/// figure — and because the mechanical figures themselves are open to
/// measurement.
///
/// TODO(collision-envelope): nothing here bounds the linkage against itself.
/// Inside a band of head heights around 13 mm below nominal the crank windows
/// stop binding on relative yaw entirely, and what limits it there is rod
/// touching rod — a distance that falls to zero at half a turn and is not
/// modelled by any check in this crate. Collision geometry is published in the
/// vendor's descriptions at three fidelities; until it is used, the relative
/// yaw cap is what keeps commands far away from that band's interior.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EnvelopeConfig {
    /// Per-leg crank travel windows, radians, `(lower, upper)`, in servo order
    /// 1..=6. Asymmetric per leg and not interchangeable between legs.
    pub crank_windows: [(f64, f64); 6],
    /// Body yaw bound, radians, symmetric about zero.
    pub body_yaw_limit: f64,
    /// Head-relative yaw bound, radians, symmetric about zero.
    pub relative_yaw_limit: f64,
    /// Bound on the angle between the head's own vertical and the base
    /// vertical, radians.
    pub head_cone_limit: f64,
    /// Floor on the per-pose toggle margin, metres.
    pub min_toggle_margin: f64,
}

impl Default for EnvelopeConfig {
    /// The vendor's published figures, with the tighter of the two relative-yaw
    /// candidates, and a clearance floor of 3 mm — comfortably above the
    /// millimetre the crank stops leave at the top of vertical travel, so the
    /// floor is what binds there rather than the stops.
    fn default() -> Self {
        Self {
            crank_windows: core::array::from_fn(|leg| {
                let (lo, hi) = baked::CRANK_WINDOWS_DEG[leg];
                (lo.to_radians(), hi.to_radians())
            }),
            // ±160°, the mechanical figure. The yaw servo's own provisioned
            // range is the full turn, so nothing below this enforces it.
            body_yaw_limit: 160.0_f64.to_radians(),
            // The vendor's two solvers disagree, at 55° and 65°; neither states
            // a derivation and nothing in the geometry binds near either. The
            // tighter one is the working cap until the axis is measured.
            relative_yaw_limit: 55.0_f64.to_radians(),
            head_cone_limit: 35.0_f64.to_radians(),
            min_toggle_margin: 0.003,
        }
    }
}

/// Which checks a pose failed. All-false is a pose inside the envelope.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct EnvelopeViolations {
    /// Per leg: the pose has no crank angle at all.
    pub unreachable: [bool; 6],
    /// Per leg: the crank angle exists but lies outside that leg's travel
    /// window. Always `false` where `unreachable` is set — an angle that does
    /// not exist is not outside a window.
    pub window: [bool; 6],
    /// The smallest toggle margin is below the floor, and the baseline does not
    /// excuse it.
    pub margin: bool,
    /// Body yaw past its bound.
    pub body_yaw: bool,
    /// Head-relative yaw past its bound.
    pub relative_yaw: bool,
    /// Head attitude past the cone bound.
    pub cone: bool,
}

impl EnvelopeViolations {
    /// Whether any check failed.
    #[must_use]
    pub fn any(&self) -> bool {
        self.unreachable.iter().any(|f| *f)
            || self.window.iter().any(|f| *f)
            || self.margin
            || self.body_yaw
            || self.relative_yaw
            || self.cone
    }
}

impl core::fmt::Display for EnvelopeViolations {
    /// Lists the failing checks, legs named. This is what a refused command
    /// prints, so it names every failure rather than the first.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let mut sep = "";
        let mut item = |f: &mut core::fmt::Formatter<'_>, text: &str| -> core::fmt::Result {
            write!(f, "{sep}{text}")?;
            sep = ", ";
            Ok(())
        };
        for (leg, _) in self.unreachable.iter().enumerate().filter(|(_, v)| **v) {
            item(f, &format!("leg {} unreachable", leg + 1))?;
        }
        for (leg, _) in self.window.iter().enumerate().filter(|(_, v)| **v) {
            item(f, &format!("leg {} outside its travel window", leg + 1))?;
        }
        if self.margin {
            item(f, "toggle margin below the floor")?;
        }
        if self.body_yaw {
            item(f, "body yaw out of range")?;
        }
        if self.relative_yaw {
            item(f, "head-relative yaw out of range")?;
        }
        if self.cone {
            item(f, "head attitude outside the cone")?;
        }
        if sep.is_empty() {
            write!(f, "none")?;
        }
        Ok(())
    }
}

/// Everything the check computed about a pose, filled whether it passed or not.
///
/// The angles are here because the check solves the legs itself: a caller that
/// passed the envelope has the goal angles in hand and never runs the inverse
/// kinematics a second time, so the checked pose and the commanded pose cannot
/// come apart.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EnvelopeReport {
    /// The crank angles, `None` exactly when some leg has no solution.
    pub leg_angles: Option<LegAngles>,
    /// Per-leg toggle margins, metres. Never a NaN; zero or less on a leg with
    /// no solution, and unbounded below for a pose placed infinitely far out.
    pub toggle_margins: [f64; 6],
    /// The smallest of the six margins.
    pub min_margin: f64,
    /// Head yaw relative to the body, radians.
    pub relative_yaw: f64,
    /// Angle between the head's vertical and the base vertical, radians; never
    /// negative.
    pub cone_angle: f64,
    /// Which checks failed.
    pub violations: EnvelopeViolations,
}

/// A commanded pose outside the envelope.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
#[error("pose is outside the envelope: {violations}")]
pub struct EnvelopeError {
    /// Every check the pose failed.
    pub violations: EnvelopeViolations,
}

/// Whether `value` is *not* known to be within `limit`.
///
/// Phrased as a failure to compare rather than as a comparison, because a
/// commanded or measured value can be non-finite: `NaN` is neither inside a bound
/// nor outside it, and every direct test of it comes back false. Requiring a
/// definite answer makes the incomparable case a violation, which is the only
/// safe reading of a quantity nobody can place.
///
/// Every bound test on the command path — envelope, tracking, and step guards —
/// must use this definition; two copies would drift on what counts as a
/// violation while both kept passing their own tests.
#[must_use]
pub fn outside_limit(value: f64, limit: f64) -> bool {
    !matches!(
        value.partial_cmp(&limit),
        Some(core::cmp::Ordering::Less | core::cmp::Ordering::Equal)
    )
}

/// Whether `value` is *not* known to reach `floor`.
///
/// [`outside_limit`] with the operands the other way round, named so that a test
/// against a lower bound reads in its own direction: a call site checking that a
/// supply rail is up or that a height clears a floor says so, instead of looking
/// like a ceiling test with its arguments swapped. Same rule for a value nobody
/// can place — a `NaN` reaches no floor, so it counts as below one.
#[must_use]
pub fn below_limit(value: f64, floor: f64) -> bool {
    outside_limit(floor, value)
}

/// Whether `value`'s magnitude is not known to be within `limit`, for the bounds
/// that are symmetric about zero.
fn magnitude_outside(value: f64, limit: f64) -> bool {
    outside_limit(value.abs(), limit)
}

/// Check one commanded configuration against the envelope.
///
/// `head_pose_body` is the head pose relative to the body at zero yaw; a
/// world-frame command composes through [`crate::yaw`] first. `margin_baseline`,
/// when supplied, is the toggle margin of the pose the machine is presently at:
/// a command whose margin strictly exceeds it is admitted even below the floor,
/// which is what lets the head lift off a rest tighter than the floor.
///
/// The antennas are not arguments: nothing here bounds them, and a check that
/// took them and said nothing about them would read like one that did.
///
/// `out` is filled either way — a refused pose reports its angles, margins and
/// yaw exactly as an accepted one does, because those numbers are what a
/// refusal has to be diagnosed from.
///
/// Bounds are tested as negated comparisons, so a non-finite commanded yaw is a
/// violation rather than something that quietly passes every test it is put to.
pub fn check_envelope(
    geom: &HeadGeometry,
    env: &EnvelopeConfig,
    head_pose_body: &Isometry3<f64>,
    body_yaw: f64,
    margin_baseline: Option<f64>,
    out: &mut EnvelopeReport,
) -> Result<(), EnvelopeError> {
    let rotation = head_pose_body.rotation.to_rotation_matrix();
    let m = rotation.matrix();

    // Head yaw about the body vertical, from the first column of the rotation.
    // Relative yaw is method-dependent, and this is the definition the caps are
    // enforced under: the ZYX yaw of the head frame in the body frame.
    out.relative_yaw = m[(1, 0)].atan2(m[(0, 0)]);

    out.cone_angle = cone_angle(&head_pose_body.rotation);

    let mut violations = EnvelopeViolations::default();
    let mut angles = [0.0; 6];
    let mut solved_all = true;

    for (leg, &(lo, hi)) in env.crank_windows.iter().enumerate() {
        let solve = solve_leg(geom, leg, head_pose_body);
        out.toggle_margins[leg] = solve.margin;
        match solve.angle {
            Some(angle) => {
                angles[leg] = angle;
                violations.window[leg] = !(angle >= lo && angle <= hi);
            }
            None => {
                violations.unreachable[leg] = true;
                solved_all = false;
            }
        }
    }

    // The same reduction the baseline a caller supplies was made under.
    out.min_margin = min_margin(&out.toggle_margins);
    out.leg_angles = solved_all.then_some(LegAngles(angles));

    // Below the floor is a violation unless the caller supplied the present
    // pose's clearance and this pose strictly improves on it. Strictly: a
    // command that merely holds the present clearance below the floor buys
    // nothing and is refused.
    violations.margin = out.min_margin < env.min_toggle_margin
        && !matches!(margin_baseline, Some(baseline) if out.min_margin > baseline);

    violations.body_yaw = magnitude_outside(body_yaw, env.body_yaw_limit);
    violations.relative_yaw = magnitude_outside(out.relative_yaw, env.relative_yaw_limit);
    violations.cone = magnitude_outside(out.cone_angle, env.head_cone_limit);

    out.violations = violations;
    if violations.any() {
        Err(EnvelopeError { violations })
    } else {
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geometry::{neutral_head_pose, rest_head_pose, stow_head_pose};
    use crate::ik::min_pose_margin;
    use nalgebra::{Translation3, UnitQuaternion, Vector3};

    /// A pose at the neutral attitude, `z` metres up the yaw axis.
    fn at_height(z: f64) -> Isometry3<f64> {
        Isometry3::translation(0.0, 0.0, z)
    }

    /// The neutral attitude pitched by `deg` about the head y axis, at the
    /// nominal height.
    fn pitched(deg: f64) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(0.0, 0.0, baked::HEAD_Z_OFFSET),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), deg.to_radians()),
        )
    }

    /// The neutral attitude yawed by `deg` about the vertical, at the nominal
    /// height.
    fn head_yawed(deg: f64) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(0.0, 0.0, baked::HEAD_Z_OFFSET),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), deg.to_radians()),
        )
    }

    /// The recorded tight resting configuration, raised or lowered by `dz`
    /// metres. At `dz = 0` it carries 0.141 mm of clearance, far below the
    /// floor.
    fn rest_shifted(dz: f64) -> Isometry3<f64> {
        let mut pose = rest_head_pose();
        pose.translation.z += dz;
        pose
    }

    fn check(
        pose: &Isometry3<f64>,
        body_yaw: f64,
        baseline: Option<f64>,
    ) -> (Result<(), EnvelopeError>, EnvelopeReport) {
        let mut report = EnvelopeReport::default();
        let verdict = check_envelope(
            &HeadGeometry::default(),
            &EnvelopeConfig::default(),
            pose,
            body_yaw,
            baseline,
            &mut report,
        );
        (verdict, report)
    }

    /// The neutral pose passes everything, and the report carries the numbers a
    /// caller needs without a second solve.
    #[test]
    fn neutral_pose_passes() {
        let (verdict, report) = check(&neutral_head_pose(), 0.0, None);
        assert!(verdict.is_ok(), "{:?}", report.violations);
        assert_eq!(report.violations, EnvelopeViolations::default());
        assert!(!report.violations.any());

        let angles = report.leg_angles.expect("neutral is reachable");
        for (leg, angle) in angles.0.iter().enumerate() {
            assert!((angle.abs() - 0.626_5).abs() < 1e-3, "leg {}", leg + 1);
        }
        assert!(report.min_margin > 0.024, "margin {}", report.min_margin);
        assert!(report.relative_yaw.abs() < 1e-15);
        assert!(report.cone_angle < 1e-15);
    }

    /// Stow is a pose the machine has to be able to command, so it has to pass
    /// its own envelope — including the cone, which it uses 24° of.
    #[test]
    fn the_stow_pose_passes() {
        let (verdict, report) = check(&stow_head_pose(), 0.0, None);
        assert!(verdict.is_ok(), "{:?}", report.violations);
        assert!(
            (report.cone_angle - baked::STOW_PITCH).abs() < 1e-9,
            "cone {}",
            report.cone_angle
        );
    }

    /// The two failure bands at the top of vertical travel are distinct, and
    /// this is the whole reason the window check exists: between the height
    /// where the cranks leave their stops and the height where the linkage runs
    /// out of geometry, every leg still solves. Without the window check those
    /// poses would be commanded.
    #[test]
    fn window_and_reach_are_separate_bands_at_the_top() {
        // Inside the band: solvable, but past the stops.
        let (verdict, report) = check(&at_height(0.2005), 0.0, None);
        assert!(verdict.is_err());
        assert!(report.leg_angles.is_some(), "the pose still solves");
        assert_eq!(report.violations.unreachable, [false; 6]);
        assert_eq!(report.violations.window, [true; 6]);

        // Above the band: no solution at all, and no angles reported.
        let (verdict, report) = check(&at_height(0.2015), 0.0, None);
        assert!(verdict.is_err());
        assert!(report.leg_angles.is_none());
        assert_eq!(report.violations.unreachable, [true; 6]);
        assert_eq!(
            report.violations.window, [false; 6],
            "an angle that does not exist is not out of window"
        );
        assert!(report.min_margin < 0.0, "margin {}", report.min_margin);
        assert!(
            report.toggle_margins.iter().all(|m| m.is_finite()),
            "margins stay finite where the angles do not exist"
        );
    }

    /// The bottom stop rests on four legs. A pose just past it names those four
    /// and leaves legs 2 and 5, which still have 22° in hand, alone.
    #[test]
    fn the_bottom_stop_names_the_four_legs_that_reach_it() {
        let (verdict, report) = check(&at_height(0.1255), 0.0, None);
        assert!(verdict.is_err());
        assert_eq!(
            report.violations.window,
            [true, false, true, true, false, true]
        );
        assert_eq!(report.violations.unreachable, [false; 6]);
    }

    /// The margin floor binds before the crank stops do on the way up: a height
    /// inside every window is still refused for clearance alone.
    #[test]
    fn the_clearance_floor_binds_before_the_crank_stops() {
        let (verdict, report) = check(&at_height(0.1995), 0.0, None);
        assert!(verdict.is_err());
        assert!(report.violations.margin);
        assert_eq!(report.violations.window, [false; 6]);
        assert_eq!(report.violations.unreachable, [false; 6]);
        assert!(
            report.min_margin > 0.0 && report.min_margin < 0.003,
            "margin {}",
            report.min_margin
        );
    }

    /// The baseline policy, on the configuration it exists for. From a rest at
    /// 0.141 mm of clearance every command is below the floor, and the machine
    /// would be stuck there without it.
    #[test]
    fn the_baseline_admits_a_lift_and_refuses_a_tightening() {
        let rest = rest_shifted(0.0);
        let baseline = min_pose_margin(&HeadGeometry::default(), &rest);
        assert!(
            (baseline - 0.000_141_133).abs() < 1e-9,
            "baseline {baseline}"
        );

        // Without a baseline the rest pose refuses its own clearance.
        let (verdict, report) = check(&rest, 0.0, None);
        assert!(verdict.is_err());
        assert!(report.violations.margin);

        // A pose 1 mm higher is still far below the floor, but improves on the
        // baseline, so it is admitted.
        let lift = rest_shifted(0.001);
        let (verdict, report) = check(&lift, 0.0, Some(baseline));
        assert!(verdict.is_ok(), "{:?}", report.violations);
        assert!(
            report.min_margin > baseline && report.min_margin < 0.003,
            "lifted margin {}",
            report.min_margin
        );

        // The same pose without the baseline is refused: the floor alone
        // governs a target validated against nothing.
        assert!(check(&lift, 0.0, None).0.is_err());

        // Holding exactly still does not improve on the baseline and buys
        // nothing, so it stays refused even with one.
        let (verdict, report) = check(&rest, 0.0, Some(baseline));
        assert!(verdict.is_err());
        assert!(report.violations.margin);
    }

    /// A baseline never excuses anything but the clearance floor: dropping the
    /// same distance the lift rose is refused, and a reach violation is refused
    /// with a baseline as readily as without one.
    #[test]
    fn the_baseline_excuses_only_the_clearance_floor() {
        let drop = rest_shifted(-0.001);
        let (verdict, report) = check(&drop, 0.0, Some(0.001));
        assert!(verdict.is_err());
        assert!(report.violations.margin);

        let (verdict, report) = check(&at_height(0.2015), 0.0, Some(1.0));
        assert!(verdict.is_err());
        assert_eq!(report.violations.unreachable, [true; 6]);
    }

    /// The relative-yaw cap is what protects this axis; the crank windows are
    /// nowhere near it. At nominal height the windows first bind around 102° of
    /// head yaw — some 47° past the cap — so at 54° nothing at all is violated,
    /// and only past 102° do the windows finally bind. Pins this module's yaw
    /// extraction convention against the height at which that figure was
    /// computed: an extraction that disagreed by a few degrees would move the
    /// bind off 102°.
    ///
    /// The cap itself is tested a degree either side rather than at it. A pose
    /// built from exactly the cap angle extracts back to within an ulp of it and
    /// lands on whichever side the rounding puts it — which is correct behaviour
    /// for a bound, and nothing worth pinning a test to.
    #[test]
    fn the_yaw_cap_binds_long_before_the_crank_windows() {
        let (verdict, report) = check(&head_yawed(54.0), 0.0, None);
        assert!(verdict.is_ok(), "{:?}", report.violations);
        assert!(
            (report.relative_yaw - 54.0_f64.to_radians()).abs() < 1e-12,
            "extracted {}",
            report.relative_yaw
        );

        let (verdict, report) = check(&head_yawed(56.0), 0.0, None);
        assert!(verdict.is_err());
        assert!(report.violations.relative_yaw);
        assert_eq!(
            report.violations.window, [false; 6],
            "nothing else binds at 56°"
        );

        // The windows themselves, with the cap widened out of the way.
        let env = EnvelopeConfig {
            relative_yaw_limit: core::f64::consts::PI,
            ..EnvelopeConfig::default()
        };
        let mut report = EnvelopeReport::default();
        let geom = HeadGeometry::default();
        let verdict = check_envelope(&geom, &env, &head_yawed(102.0), 0.0, None, &mut report);
        assert!(verdict.is_ok(), "at 102° {:?}", report.violations);

        let verdict = check_envelope(&geom, &env, &head_yawed(103.0), 0.0, None, &mut report);
        assert!(verdict.is_err());
        assert!(
            report.violations.window.iter().any(|w| *w),
            "the windows bind by 103°: {:?}",
            report.violations
        );
    }

    /// The head attitude cone, which no crank window expresses. Stow already
    /// uses 24° of it, so the bound is one the machine works close to.
    #[test]
    fn the_cone_bounds_head_attitude() {
        let (verdict, report) = check(&pitched(34.0), 0.0, None);
        assert!(verdict.is_ok(), "{:?}", report.violations);
        assert!(
            (report.cone_angle - 34.0_f64.to_radians()).abs() < 1e-12,
            "cone {}",
            report.cone_angle
        );

        // Two degrees further is refused, and by the cone alone: nothing about
        // the legs has run out at that attitude.
        for deg in [36.0, -36.0] {
            let (verdict, report) = check(&pitched(deg), 0.0, None);
            assert!(verdict.is_err(), "{deg}°");
            assert!(report.violations.cone, "{deg}° is outside the cone");
            assert_eq!(report.violations.window, [false; 6], "{deg}°");
            assert_eq!(report.violations.unreachable, [false; 6], "{deg}°");
            assert!(!report.violations.margin, "{deg}°");
        }
    }

    /// Body yaw is a scalar bound on a quantity no pose expresses, checked on
    /// the same verdict as everything else.
    #[test]
    fn the_body_yaw_bound_is_two_sided() {
        let pose = neutral_head_pose();
        let over = 161.0_f64.to_radians();
        for yaw in [over, -over] {
            let (verdict, report) = check(&pose, yaw, None);
            assert!(verdict.is_err());
            assert!(report.violations.body_yaw);
        }
        assert!(check(&pose, 159.0_f64.to_radians(), None).0.is_ok());
    }

    /// Inside and at the bound pass, past it fails, and a value nobody can
    /// compare fails.
    #[test]
    fn the_bound_test_admits_the_bound_and_refuses_the_incomparable() {
        assert!(!outside_limit(0.9, 1.0));
        assert!(!outside_limit(1.0, 1.0));
        assert!(outside_limit(1.000_000_000_000_1, 1.0));
        assert!(outside_limit(f64::NAN, 1.0));
        assert!(outside_limit(f64::INFINITY, 1.0));
        assert!(!outside_limit(f64::NEG_INFINITY, 1.0));
        assert!(outside_limit(0.5, f64::NAN));

        assert!(!magnitude_outside(-1.0, 1.0));
        assert!(magnitude_outside(-1.5, 1.0));
        assert!(magnitude_outside(f64::NEG_INFINITY, 1.0));
    }

    /// The floor test is the ceiling test read the other way: at the floor
    /// passes, under it fails, and a reading nobody can place reaches no floor.
    #[test]
    fn the_floor_test_is_the_bound_test_the_other_way_round() {
        assert!(!below_limit(1.0, 1.0));
        assert!(!below_limit(1.5, 1.0));
        assert!(below_limit(0.999_999_999_999_9, 1.0));
        assert!(below_limit(f64::NAN, 1.0));
        assert!(below_limit(f64::NEG_INFINITY, 1.0));
        assert!(!below_limit(f64::INFINITY, 1.0));

        for (value, bound) in [(0.9, 1.0), (1.0, 1.0), (f64::NAN, 1.0), (0.5, f64::NAN)] {
            assert_eq!(
                below_limit(value, bound),
                outside_limit(bound, value),
                "{value} against {bound}"
            );
        }
    }

    /// A non-finite commanded scalar is a violation, not something that passes
    /// every comparison it is put to. `NaN <= limit` is false; the checks are
    /// written negated so that falseness reads as a failure.
    #[test]
    fn non_finite_scalars_are_violations() {
        let pose = neutral_head_pose();
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let (verdict, report) = check(&pose, bad, None);
            assert!(verdict.is_err(), "body yaw {bad}");
            assert!(report.violations.body_yaw);
        }
    }

    /// The pose half of the same story, which the scalar bounds do not cover: a
    /// translation or quaternion component nobody can place makes every leg
    /// unsolvable, and the pose is refused on that.
    ///
    /// Pinned because the refusal comes out of the reach test demanding a
    /// definite answer rather than out of a check written for this case: a
    /// reordering of those guards that read as equivalent could turn an undefined
    /// pose into an accepted one. Pinned too is what the report says about the
    /// clearance of such a pose — never a positive number, and never a NaN, since
    /// the tick feeds that reduction straight back as the next check's baseline.
    /// A pose that cannot be placed at all reduces to exactly zero; one placed
    /// infinitely far out reduces to negative infinity.
    #[test]
    fn a_non_finite_pose_is_refused_on_every_leg() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for axis in 0..3 {
                let mut pose = neutral_head_pose();
                pose.translation.vector[axis] = bad;
                let (verdict, report) = check(&pose, 0.0, None);
                let case = format!("translation axis {axis} with {bad}");
                assert!(verdict.is_err(), "{case}");
                assert_eq!(report.violations.unreachable, [true; 6], "{case}");
                assert!(report.leg_angles.is_none(), "{case}");
                assert!(report.violations.margin, "{case}");
                assert!(
                    !report.min_margin.is_nan() && report.min_margin <= 0.0,
                    "{case}: no clearance is claimed, and the number is one: {}",
                    report.min_margin
                );
                assert_eq!(
                    min_pose_margin(&HeadGeometry::default(), &pose),
                    report.min_margin,
                    "{case}: the baseline reducer agrees with the report"
                );
            }

            // The undefined case reduces to exactly zero, which is what the
            // baseline reducer promises for a pose it cannot place at all.
            let mut pose = neutral_head_pose();
            pose.translation.vector.y = f64::NAN;
            assert_eq!(min_pose_margin(&HeadGeometry::default(), &pose), 0.0);

            let mut pose = neutral_head_pose();
            pose.rotation =
                UnitQuaternion::new_unchecked(nalgebra::Quaternion::new(bad, 0.0, 0.0, 0.0));
            let (verdict, report) = check(&pose, 0.0, None);
            assert!(verdict.is_err(), "rotation with {bad}");
            assert_eq!(report.violations.unreachable, [true; 6]);
            assert!(report.violations.relative_yaw && report.violations.cone);
            assert!(
                report.cone_angle.is_nan(),
                "a rotation nobody can place has no tilt to report: {}",
                report.cone_angle
            );
        }

        // A baseline excuses none of it, at any value.
        let mut pose = neutral_head_pose();
        pose.translation.vector.y = f64::NAN;
        assert!(check(&pose, 0.0, Some(-1.0)).0.is_err());
    }

    /// Every violation kind is producible and every one is named in the
    /// message. A kind that no input can set, or that the report can set but
    /// the message swallows, is a hole in the verdict.
    #[test]
    fn every_violation_kind_is_produced_and_named() {
        let cases: [(EnvelopeViolations, &str); 6] = [
            (
                check(&at_height(0.2015), 0.0, None).1.violations,
                "leg 1 unreachable",
            ),
            (
                check(&at_height(0.2005), 0.0, None).1.violations,
                "leg 1 outside its travel window",
            ),
            (
                check(&at_height(0.1995), 0.0, None).1.violations,
                "toggle margin below the floor",
            ),
            (
                check(&neutral_head_pose(), 3.0, None).1.violations,
                "body yaw out of range",
            ),
            (
                check(&head_yawed(56.0), 0.0, None).1.violations,
                "head-relative yaw out of range",
            ),
            (
                check(&pitched(36.0), 0.0, None).1.violations,
                "head attitude outside the cone",
            ),
        ];
        for (violations, expected) in cases {
            assert!(violations.any());
            let text = violations.to_string();
            assert!(text.contains(expected), "{text:?} lacks {expected:?}");
        }

        assert_eq!(EnvelopeViolations::default().to_string(), "none");
    }

    /// The error carries exactly what the report carries, so a caller that only
    /// keeps the error still knows everything that failed.
    #[test]
    fn the_error_carries_the_report_s_violations() {
        let (verdict, report) = check(&at_height(0.2015), 0.0, None);
        let error = verdict.expect_err("refused");
        assert_eq!(error.violations, report.violations);
        assert!(error.to_string().contains("leg 6 unreachable"));
        assert!(error.to_string().contains("toggle margin below the floor"));
    }
}
