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
//! - **Antenna range**, which has no servo-side envelope in any operating mode.
//!   This check is the only one there is, so it belongs on the same verdict
//!   path as the rest.
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
//! come to rest below it — one documented resting configuration sits at 0.19 mm
//! of clearance, a sixteenth of the floor. Refusing every command from there
//! would leave the head stuck at its tightest. So a caller that knows the
//! present pose's clearance passes it as `margin_baseline`, and a pose that
//! strictly increases clearance is admissible even below the floor. Motion
//! toward a singular configuration stays refused at every margin.

use nalgebra::Isometry3;
use thiserror::Error;

use crate::baked;
use crate::geometry::HeadGeometry;
use crate::ik::{LegAngles, solve_leg};

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
    /// Antenna bound, radians, symmetric about zero.
    pub antenna_limit: f64,
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
            // The magnitude the stow pose uses. The mechanical range is a full
            // half turn either way, but nothing enforces it in the servo.
            antenna_limit: 3.05,
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
    /// Per antenna, right then left.
    pub antenna: [bool; 2],
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
            || self.antenna.iter().any(|f| *f)
    }
}

impl core::fmt::Display for EnvelopeViolations {
    /// Lists the failing checks, legs and antennas named. This is what a
    /// refused command prints, so it names every failure rather than the first.
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
        if self.antenna[0] {
            item(f, "right antenna out of range")?;
        }
        if self.antenna[1] {
            item(f, "left antenna out of range")?;
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
    /// Per-leg toggle margins, metres. Always finite, negative where a leg has
    /// no solution.
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

/// Whether `value`'s magnitude is *not* known to be within `limit`.
///
/// Phrased as a failure to compare rather than as a comparison, because a
/// commanded value can be non-finite: `NaN` is neither inside a bound nor
/// outside it, and every direct test of it comes back false. Requiring a
/// definite answer makes the incomparable case a violation, which is the only
/// safe reading of a commanded quantity nobody can place.
fn magnitude_outside(value: f64, limit: f64) -> bool {
    !matches!(
        value.abs().partial_cmp(&limit),
        Some(core::cmp::Ordering::Less | core::cmp::Ordering::Equal)
    )
}

/// Check one commanded configuration against the envelope.
///
/// `head_pose_body` is the head pose relative to the body at zero yaw; a
/// world-frame command composes through [`crate::yaw`] first. `antennas` is
/// right then left. `margin_baseline`, when supplied, is the toggle margin of
/// the pose the machine is presently at: a command whose margin strictly
/// exceeds it is admitted even below the floor, which is what lets the head
/// lift off a rest tighter than the floor.
///
/// `out` is filled either way — a refused pose reports its angles, margins and
/// yaw exactly as an accepted one does, because those numbers are what a
/// refusal has to be diagnosed from.
///
/// Bounds are tested as negated comparisons, so a non-finite commanded yaw or
/// antenna angle is a violation rather than something that quietly passes every
/// test it is put to.
pub fn check_envelope(
    geom: &HeadGeometry,
    env: &EnvelopeConfig,
    head_pose_body: &Isometry3<f64>,
    body_yaw: f64,
    antennas: [f64; 2],
    margin_baseline: Option<f64>,
    out: &mut EnvelopeReport,
) -> Result<(), EnvelopeError> {
    let rotation = head_pose_body.rotation.to_rotation_matrix();
    let m = rotation.matrix();

    // Head yaw about the body vertical, from the first column of the rotation.
    // Relative yaw is method-dependent, and this is the definition the caps are
    // enforced under: the ZYX yaw of the head frame in the body frame.
    out.relative_yaw = m[(1, 0)].atan2(m[(0, 0)]);

    // The head's own vertical against the base vertical. The clamp is
    // numeric hygiene on a unit-vector component that fp error can push an ulp
    // past either end of the domain; the near-inverted end is reachable in
    // practice, so the full domain must be handled.
    out.cone_angle = m[(2, 2)].clamp(-1.0, 1.0).acos();

    let mut violations = EnvelopeViolations::default();
    let mut angles = [0.0; 6];
    let mut solved_all = true;
    let mut min_margin = f64::INFINITY;

    for (leg, &(lo, hi)) in env.crank_windows.iter().enumerate() {
        let solve = solve_leg(geom, leg, head_pose_body);
        out.toggle_margins[leg] = solve.margin;
        min_margin = min_margin.min(solve.margin);
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

    out.min_margin = min_margin;
    out.leg_angles = solved_all.then_some(LegAngles(angles));

    // Below the floor is a violation unless the caller supplied the present
    // pose's clearance and this pose strictly improves on it. Strictly: a
    // command that merely holds the present clearance below the floor buys
    // nothing and is refused.
    violations.margin = min_margin < env.min_toggle_margin
        && !matches!(margin_baseline, Some(baseline) if min_margin > baseline);

    violations.body_yaw = magnitude_outside(body_yaw, env.body_yaw_limit);
    violations.relative_yaw = magnitude_outside(out.relative_yaw, env.relative_yaw_limit);
    violations.cone = magnitude_outside(out.cone_angle, env.head_cone_limit);
    for (slot, angle) in violations.antenna.iter_mut().zip(antennas) {
        *slot = magnitude_outside(angle, env.antenna_limit);
    }

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
    use crate::geometry::{neutral_head_pose, stow_head_pose};
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

    /// The recovered head pose of the tight resting configuration the vendor's
    /// simulated backends start from: under 0.2 mm of clearance, far below the
    /// floor.
    fn tight_rest_pose() -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(-0.015_17, 0.001_03, 0.126_57),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 30.84_f64.to_radians()),
        )
    }

    fn check(
        pose: &Isometry3<f64>,
        body_yaw: f64,
        antennas: [f64; 2],
        baseline: Option<f64>,
    ) -> (Result<(), EnvelopeError>, EnvelopeReport) {
        let mut report = EnvelopeReport::default();
        let verdict = check_envelope(
            &HeadGeometry::default(),
            &EnvelopeConfig::default(),
            pose,
            body_yaw,
            antennas,
            baseline,
            &mut report,
        );
        (verdict, report)
    }

    /// The neutral pose passes everything, and the report carries the numbers a
    /// caller needs without a second solve.
    #[test]
    fn neutral_pose_passes() {
        let (verdict, report) = check(&neutral_head_pose(), 0.0, [0.0, 0.0], None);
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
    fn stow_pose_passes_with_stow_antennas() {
        let (verdict, report) = check(&stow_head_pose(), 0.0, [-3.05, 3.05], None);
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
        let (verdict, report) = check(&at_height(0.2005), 0.0, [0.0, 0.0], None);
        assert!(verdict.is_err());
        assert!(report.leg_angles.is_some(), "the pose still solves");
        assert_eq!(report.violations.unreachable, [false; 6]);
        assert_eq!(report.violations.window, [true; 6]);

        // Above the band: no solution at all, and no angles reported.
        let (verdict, report) = check(&at_height(0.2015), 0.0, [0.0, 0.0], None);
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
        let (verdict, report) = check(&at_height(0.1255), 0.0, [0.0, 0.0], None);
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
        let (verdict, report) = check(&at_height(0.1995), 0.0, [0.0, 0.0], None);
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
    /// 0.19 mm of clearance every command is below the floor, and the machine
    /// would be stuck there without it.
    #[test]
    fn the_baseline_admits_a_lift_and_refuses_a_tightening() {
        let rest = tight_rest_pose();
        let mut margins = [0.0; 6];
        crate::ik::pose_margins(&HeadGeometry::default(), &rest, &mut margins);
        let baseline = margins.iter().copied().fold(f64::INFINITY, f64::min);
        assert!(baseline > 0.0 && baseline < 0.0003, "baseline {baseline}");

        // Without a baseline the rest pose refuses its own clearance.
        let (verdict, report) = check(&rest, 0.0, [0.0, 0.0], None);
        assert!(verdict.is_err());
        assert!(report.violations.margin);

        // A pose 1 mm higher is still far below the floor, but improves on the
        // baseline, so it is admitted.
        let lift = Isometry3::from_parts(
            Translation3::new(-0.015_17, 0.001_03, 0.127_57),
            rest.rotation,
        );
        let (verdict, report) = check(&lift, 0.0, [0.0, 0.0], Some(baseline));
        assert!(verdict.is_ok(), "{:?}", report.violations);
        assert!(
            report.min_margin > baseline && report.min_margin < 0.003,
            "lifted margin {}",
            report.min_margin
        );

        // The same pose without the baseline is refused: the floor alone
        // governs a target validated against nothing.
        assert!(check(&lift, 0.0, [0.0, 0.0], None).0.is_err());

        // Holding exactly still does not improve on the baseline and buys
        // nothing, so it stays refused even with one.
        let (verdict, report) = check(&rest, 0.0, [0.0, 0.0], Some(baseline));
        assert!(verdict.is_err());
        assert!(report.violations.margin);
    }

    /// A baseline never excuses anything but the clearance floor: dropping the
    /// same distance the lift rose is refused, and a reach violation is refused
    /// with a baseline as readily as without one.
    #[test]
    fn the_baseline_excuses_only_the_clearance_floor() {
        let rest = tight_rest_pose();
        let drop = Isometry3::from_parts(
            Translation3::new(-0.015_17, 0.001_03, 0.125_57),
            rest.rotation,
        );
        let (verdict, report) = check(&drop, 0.0, [0.0, 0.0], Some(0.001));
        assert!(verdict.is_err());
        assert!(report.violations.margin);

        let (verdict, report) = check(&at_height(0.2015), 0.0, [0.0, 0.0], Some(1.0));
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
        let (verdict, report) = check(&head_yawed(54.0), 0.0, [0.0, 0.0], None);
        assert!(verdict.is_ok(), "{:?}", report.violations);
        assert!(
            (report.relative_yaw - 54.0_f64.to_radians()).abs() < 1e-12,
            "extracted {}",
            report.relative_yaw
        );

        let (verdict, report) = check(&head_yawed(56.0), 0.0, [0.0, 0.0], None);
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
        let verdict = check_envelope(
            &geom,
            &env,
            &head_yawed(102.0),
            0.0,
            [0.0, 0.0],
            None,
            &mut report,
        );
        assert!(verdict.is_ok(), "at 102° {:?}", report.violations);

        let verdict = check_envelope(
            &geom,
            &env,
            &head_yawed(103.0),
            0.0,
            [0.0, 0.0],
            None,
            &mut report,
        );
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
        let (verdict, report) = check(&pitched(34.0), 0.0, [0.0, 0.0], None);
        assert!(verdict.is_ok(), "{:?}", report.violations);
        assert!(
            (report.cone_angle - 34.0_f64.to_radians()).abs() < 1e-12,
            "cone {}",
            report.cone_angle
        );

        // Two degrees further is refused, and by the cone alone: nothing about
        // the legs has run out at that attitude.
        for deg in [36.0, -36.0] {
            let (verdict, report) = check(&pitched(deg), 0.0, [0.0, 0.0], None);
            assert!(verdict.is_err(), "{deg}°");
            assert!(report.violations.cone, "{deg}° is outside the cone");
            assert_eq!(report.violations.window, [false; 6], "{deg}°");
            assert_eq!(report.violations.unreachable, [false; 6], "{deg}°");
            assert!(!report.violations.margin, "{deg}°");
        }
    }

    /// Body yaw and the antennas are scalar bounds on quantities no pose
    /// expresses, checked on the same verdict as everything else. The antennas
    /// have no servo-side envelope in any mode, so this is their only check.
    #[test]
    fn the_scalar_bounds_are_two_sided_and_named() {
        let pose = neutral_head_pose();
        let over = 161.0_f64.to_radians();
        for yaw in [over, -over] {
            let (verdict, report) = check(&pose, yaw, [0.0, 0.0], None);
            assert!(verdict.is_err());
            assert!(report.violations.body_yaw);
        }
        assert!(
            check(&pose, 159.0_f64.to_radians(), [0.0, 0.0], None)
                .0
                .is_ok()
        );

        for (index, antennas) in [[3.06, 0.0], [0.0, 3.06]].into_iter().enumerate() {
            let (verdict, report) = check(&pose, 0.0, antennas, None);
            assert!(verdict.is_err());
            assert!(report.violations.antenna[index], "antenna {index} named");
            assert!(!report.violations.antenna[1 - index]);
        }
        assert!(check(&pose, 0.0, [-3.05, 3.05], None).0.is_ok());
    }

    /// A non-finite commanded scalar is a violation, not something that passes
    /// every comparison it is put to. `NaN <= limit` is false; the checks are
    /// written negated so that falseness reads as a failure.
    #[test]
    fn non_finite_scalars_are_violations() {
        let pose = neutral_head_pose();
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let (verdict, report) = check(&pose, bad, [0.0, 0.0], None);
            assert!(verdict.is_err(), "body yaw {bad}");
            assert!(report.violations.body_yaw);

            let (verdict, report) = check(&pose, 0.0, [bad, bad], None);
            assert!(verdict.is_err(), "antennas {bad}");
            assert_eq!(report.violations.antenna, [true, true]);
        }
    }

    /// Every violation kind is producible and every one is named in the
    /// message. A kind that no input can set, or that the report can set but
    /// the message swallows, is a hole in the verdict.
    #[test]
    fn every_violation_kind_is_produced_and_named() {
        let cases: [(EnvelopeViolations, &str); 7] = [
            (
                check(&at_height(0.2015), 0.0, [0.0, 0.0], None)
                    .1
                    .violations,
                "leg 1 unreachable",
            ),
            (
                check(&at_height(0.2005), 0.0, [0.0, 0.0], None)
                    .1
                    .violations,
                "leg 1 outside its travel window",
            ),
            (
                check(&at_height(0.1995), 0.0, [0.0, 0.0], None)
                    .1
                    .violations,
                "toggle margin below the floor",
            ),
            (
                check(&neutral_head_pose(), 3.0, [0.0, 0.0], None)
                    .1
                    .violations,
                "body yaw out of range",
            ),
            (
                check(&head_yawed(56.0), 0.0, [0.0, 0.0], None).1.violations,
                "head-relative yaw out of range",
            ),
            (
                check(&pitched(36.0), 0.0, [0.0, 0.0], None).1.violations,
                "head attitude outside the cone",
            ),
            (
                check(&neutral_head_pose(), 0.0, [3.2, 3.2], None)
                    .1
                    .violations,
                "right antenna out of range",
            ),
        ];
        for (violations, expected) in cases {
            assert!(violations.any());
            let text = violations.to_string();
            assert!(text.contains(expected), "{text:?} lacks {expected:?}");
        }

        // The left antenna is the one kind the cases above cannot reach on its
        // own, since the right one is checked first.
        let violations = check(&neutral_head_pose(), 0.0, [0.0, 3.2], None)
            .1
            .violations;
        assert_eq!(violations.to_string(), "left antenna out of range");
        assert_eq!(EnvelopeViolations::default().to_string(), "none");
    }

    /// The error carries exactly what the report carries, so a caller that only
    /// keeps the error still knows everything that failed.
    #[test]
    fn the_error_carries_the_report_s_violations() {
        let (verdict, report) = check(&at_height(0.2015), 0.0, [4.0, 0.0], None);
        let error = verdict.expect_err("refused");
        assert_eq!(error.violations, report.violations);
        assert!(error.to_string().contains("leg 6 unreachable"));
        assert!(error.to_string().contains("right antenna"));
    }
}
