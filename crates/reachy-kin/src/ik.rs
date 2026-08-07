//! Inverse kinematics: head pose in, six crank angles out, closed form per leg.
//!
//! Each leg is one loop-closure equation in one unknown, so there is nothing to
//! iterate. The crank tip is confined to the crank's swept plane; the platform
//! anchor is not, and sits a pose-dependent distance `h` out of it. Splitting
//! the rod at the anchor into its out-of-plane part `h` — fixed by the head pose
//! alone — and an in-plane part of length `r = √(rod_len² − h²)` turns the leg
//! into a planar circle-meets-circle problem that is *exact*, not a projected
//! approximation: the only part of the loop that leaves the plane is the rod,
//! and how far it leaves does not depend on the crank angle.
//!
//! ## Toggle margin
//!
//! A leg reaches a singular configuration exactly where its two roots merge, and
//! beyond it no real solution exists. With `ρ` the in-plane distance from crank
//! centre to anchor projection, the leg is solvable exactly on
//! `ρ ∈ [|crank_len − r|, crank_len + r]` and the roots merge at both ends, so
//!
//! ```text
//! margin = min( (crank_len + r) − ρ , ρ − |crank_len − r| )
//! ```
//!
//! is zero at a singular configuration and monotone away from it on both sides.
//! The familiar crank-and-rod-become-collinear picture is a planar idealisation
//! and is false here — at a merged root the crank–rod angle is `asin(h/rod_len)`,
//! which is pose-dependent and reaches zero at admissible poses, so it has no
//! fixed value to measure a distance from. This margin does.
//!
//! The margin is **not** a tabulated constant. The crank travel windows hold the
//! linkage about a millimetre off a singular configuration along pure vertical
//! travel; off that axis they do not bound the distance away from zero at all,
//! and the singular surface passes through the interior of the window box. Every
//! consumer computes it per pose.
//!
//! ## No non-finite results
//!
//! Every step below is guarded so that no `NaN` can be produced or returned. The
//! margin is a total function — defined at every pose, including poses with no
//! solution, where it is negative — so margin arrays are always finite even when
//! the angles do not exist.

use nalgebra::{Isometry3, Point3};
use thiserror::Error;

use crate::geometry::HeadGeometry;

/// The six crank angles, radians, in servo order 1..=6, on the model datum.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct LegAngles(pub [f64; 6]);

/// A head pose with no solution on at least one leg.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum IkError {
    /// Leg `leg` (0-based) has no real solution; `deficit` metres is how far
    /// past its solvable band the pose puts it, and is never negative.
    #[error("leg {leg} cannot reach the commanded pose, short by {deficit:.6} m")]
    Unreachable {
        /// 0-based leg index.
        leg: u8,
        /// Metres past the solvable band; ≥ 0.
        deficit: f64,
    },
}

/// Everything one leg's closed form produces, computed in a single pass so that
/// the angle and the margin can never disagree about the same pose.
#[derive(Clone, Copy, Debug)]
pub(crate) struct LegSolve {
    /// The crank angle, normalised into `(−π, π]`. `None` exactly when the leg
    /// has no real solution at this pose.
    pub angle: Option<f64>,
    /// Toggle margin, metres. Always finite; negative when there is no
    /// solution.
    pub margin: f64,
}

impl LegSolve {
    /// Metres past the solvable band, for a leg that has no solution.
    pub(crate) fn deficit(&self) -> f64 {
        -self.margin
    }
}

/// Normalise an angle into `(−π, π]`.
///
/// The one piece of circular arithmetic in this workspace: the leg solve folds
/// its crank angle here, and the motion layer resolves an antenna direction and
/// measures an antenna's distance from stow with it. One definition, so a
/// difference taken in one place and a bound applied in another cannot disagree
/// about where the half turn is.
#[must_use]
pub fn wrap_to_pi(angle: f64) -> f64 {
    let wrapped = angle.rem_euclid(core::f64::consts::TAU);
    if wrapped > core::f64::consts::PI {
        wrapped - core::f64::consts::TAU
    } else {
        wrapped
    }
}

/// Solve one leg for a head pose given in the body frame at zero yaw.
pub(crate) fn solve_leg(
    geom: &HeadGeometry,
    leg: usize,
    head_pose_body: &Isometry3<f64>,
) -> LegSolve {
    let l = &geom.legs[leg];
    let anchor_base: Point3<f64> = head_pose_body * l.anchor_head;
    let q: Point3<f64> = l.t_base_to_crank * anchor_base;

    let h = q.z;
    let in_plane_reach = q.x.hypot(q.y);
    let disc = geom.rod_len * geom.rod_len - h * h;

    // The anchor is farther from the swept plane than the whole rod: no crank
    // angle can close the loop, whatever ρ is. The margin stays defined and
    // negative so that margin arrays remain finite and monotone.
    if disc < 0.0 {
        return LegSolve {
            angle: None,
            margin: geom.rod_len - h.abs(),
        };
    }

    let r = disc.sqrt();
    let margin =
        ((geom.crank_len + r) - in_plane_reach).min(in_plane_reach - (geom.crank_len - r).abs());

    // |tip(θ) − q| = rod_len expands to A·cosθ + B·sinθ = C, with
    // A = 2·crank_len·q.x and B = 2·crank_len·q.y. Neither is formed: A and B
    // enter only as D = √(A² + B²) = 2·crank_len·ρ and as
    // atan2(B, A) = atan2(q.y, q.x), the scaling by 2·crank_len > 0 dropping out
    // of both. Taking D from ρ also keeps it and the margin on the same ρ.
    let c = geom.crank_len * geom.crank_len + in_plane_reach * in_plane_reach - r * r;
    let d = 2.0 * geom.crank_len * in_plane_reach;

    // Solvable iff |C| ≤ D, which is the same statement as margin ≥ 0: both say
    // ρ ∈ [|crank_len − r|, crank_len + r]. Tested against each other.
    //
    // D == 0 puts the anchor on the crank axis, where every crank angle solves
    // the leg. A whole circle of answers is not an answer, so it is refused —
    // and it is the one place the deficit can come out at zero.
    //
    // Both comparisons demand a definite answer, so a pose carrying a component
    // nobody can place leaves the leg unsolvable rather than solved: an
    // incomparable D fails the first test outright.
    let solvable = d > 0.0 && c.abs() <= d;
    let angle = if solvable {
        // |ratio| ≤ 1 by the strict test above, so acos never sees an argument
        // outside its domain and never returns NaN.
        let ratio = c / d;
        Some(wrap_to_pi(
            q.y.atan2(q.x) + l.branch.as_f64() * ratio.acos(),
        ))
    } else {
        None
    };

    LegSolve {
        angle,
        // A refused D == 0 must not report a non-negative margin, and a margin
        // nobody can place must not travel onward as one: an unsolvable leg
        // reports zero or less, always a number.
        margin: if angle.is_none() {
            if margin.is_nan() {
                0.0
            } else {
                margin.min(0.0)
            }
        } else {
            margin
        },
    }
}

/// Crank angles for a head pose given in the body frame at zero yaw.
///
/// Fails on the first leg with no real solution, naming it and how far past its
/// solvable band the pose is. There is no clamped or saturated answer: a pose
/// outside the workspace has no crank angles, and returning approximate ones
/// would be returning a different pose than the one commanded.
///
/// On failure `out` is left exactly as it arrived. Legs solved before the one
/// that failed are discarded rather than written: a caller reusing one
/// `LegAngles` across calls would otherwise be holding some new angles beside
/// some stale ones, which describe a pose nobody commanded.
///
/// This is public for tests and tools. The command path goes through the
/// envelope check, which additionally enforces the travel windows and the
/// clearance floor that this function knows nothing about.
pub fn inverse_kinematics(
    geom: &HeadGeometry,
    head_pose_body: &Isometry3<f64>,
    out: &mut LegAngles,
) -> Result<(), IkError> {
    let mut angles = [0.0; 6];
    for (leg, slot) in angles.iter_mut().enumerate() {
        let solved = solve_leg(geom, leg, head_pose_body);
        match solved.angle {
            Some(angle) => *slot = angle,
            None => {
                return Err(IkError::Unreachable {
                    leg: leg as u8,
                    deficit: solved.deficit(),
                });
            }
        }
    }
    out.0 = angles;
    Ok(())
}

/// Per-leg toggle margins for a head pose, metres.
///
/// Infallible: the margin is defined at every pose, and is zero or less where
/// the leg has no solution. Callers use it as the clearance baseline of a pose
/// they are already at, which may be a pose no command would be allowed to reach.
pub fn pose_margins(geom: &HeadGeometry, head_pose_body: &Isometry3<f64>, out: &mut [f64; 6]) {
    for (leg, slot) in out.iter_mut().enumerate() {
        *slot = solve_leg(geom, leg, head_pose_body).margin;
    }
}

/// The smallest of six per-leg toggle margins, metres.
///
/// The one reduction every clearance comparison is made under: the envelope's
/// minimum, the baseline an armed machine is started from, and any caller that
/// already holds a margin array all reduce here, so a change to what "smallest"
/// means cannot reach one of them and miss another. A caller holding only a pose
/// uses [`min_pose_margin`] instead.
///
/// Never a NaN: an unsolvable leg's margin is zero or less, and `f64::min`
/// carries a number past one, so the result is the least clearance any leg has.
#[must_use]
pub fn min_margin(margins: &[f64; 6]) -> f64 {
    margins.iter().copied().fold(f64::INFINITY, f64::min)
}

/// The smallest of a pose's six toggle margins, metres.
///
/// The clearance baseline the margin-baseline policy is defined over. The
/// baseline a caller measures and the minimum the envelope compares against must
/// be the same quantity, reduced the same way; callers that need only the
/// minimum use this rather than folding [`pose_margins`] themselves.
///
/// Allocates nothing and never returns a NaN, an undefined pose included: such a
/// pose has no solvable leg, and an unsolvable leg's margin is zero or less, so
/// it reduces to exactly `0.0` — no clearance, which is the least a caller can
/// act on. A pose placed infinitely far out reduces to negative infinity.
#[must_use]
pub fn min_pose_margin(geom: &HeadGeometry, head_pose_body: &Isometry3<f64>) -> f64 {
    let mut margins = [0.0; 6];
    pose_margins(geom, head_pose_body, &mut margins);
    min_margin(&margins)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::baked;
    use crate::geometry::{BranchSign, neutral_head_pose, rest_head_pose, stow_head_pose};
    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

    use crate::testutil::Rng;

    fn random_pose(rng: &mut Rng, reach: f64) -> Isometry3<f64> {
        let axis = Vector3::new(
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        );
        let rotation = match nalgebra::Unit::try_new(axis, 1e-9) {
            Some(unit) => UnitQuaternion::from_axis_angle(&unit, rng.range(-1.2, 1.2)),
            None => UnitQuaternion::identity(),
        };
        Isometry3::from_parts(
            Translation3::new(
                rng.range(-reach, reach),
                rng.range(-reach, reach),
                baked::HEAD_Z_OFFSET + rng.range(-reach, reach),
            ),
            rotation,
        )
    }

    /// The neutral pose: crank angles equal in magnitude, alternating in sign,
    /// and a comfortable 24.26 mm of clearance on all six legs.
    #[test]
    fn neutral_pose_golden() {
        let geom = HeadGeometry::default();
        let mut angles = LegAngles::default();
        inverse_kinematics(&geom, &neutral_head_pose(), &mut angles).expect("neutral is reachable");

        let mut margins = [0.0; 6];
        pose_margins(&geom, &neutral_head_pose(), &mut margins);

        for (leg, margin) in margins.iter().enumerate() {
            let deg = angles.0[leg].to_degrees();
            let expected = if leg % 2 == 0 { 35.898 } else { -35.898 };
            assert!(
                (deg - expected).abs() < 0.001,
                "leg {} angle {deg}",
                leg + 1
            );
            assert!(
                (margin - 0.024_258).abs() < 1e-6,
                "leg {} margin {margin}",
                leg + 1
            );
        }
    }

    /// Which algebraic root each leg occupies is settled by the travel windows,
    /// not assumed from the vendor's branch field: at the neutral pose exactly
    /// one root per leg lies inside that leg's own window, and the baked sign
    /// must select it.
    #[test]
    fn branch_signs_are_the_roots_the_windows_admit() {
        let mut geom = HeadGeometry::default();
        let pose = neutral_head_pose();

        for leg in 0..6 {
            let (lo, hi) = baked::CRANK_WINDOWS_DEG[leg];
            let mut admissible = None;
            for sign in [BranchSign::Plus, BranchSign::Minus] {
                geom.legs[leg].branch = sign;
                let angle = solve_leg(&geom, leg, &pose)
                    .angle
                    .expect("neutral is reachable")
                    .to_degrees();
                if angle >= lo && angle <= hi {
                    assert!(
                        admissible.is_none(),
                        "leg {} admits both roots in [{lo}, {hi}]",
                        leg + 1
                    );
                    admissible = Some(sign);
                }
            }
            assert_eq!(
                admissible,
                Some(baked::BRANCH_SIGNS[leg]),
                "leg {} branch",
                leg + 1
            );
            geom.legs[leg].branch = baked::BRANCH_SIGNS[leg];
        }
    }

    /// `|C| ≤ D` and `margin ≥ 0` are the same statement. Both are recomputed
    /// here from the geometry, independently of the solver's single pass, over
    /// poses ranging from well inside the workspace to well outside it.
    ///
    /// Random poses never land close enough to a tangency for the two sides to
    /// round apart; poses that do are constructed in `lemma_holds_at_tangency`.
    #[test]
    fn solvability_and_margin_sign_agree() {
        let geom = HeadGeometry::default();
        let mut rng = Rng::new(0x5eed_1357_9bdf_0246);

        for _ in 0..20_000 {
            let pose = random_pose(&mut rng, 0.09);
            for leg in 0..6 {
                let solved = solve_leg(&geom, leg, &pose);
                assert!(solved.margin.is_finite());

                let l = &geom.legs[leg];
                let q = l.t_base_to_crank * (pose * l.anchor_head);
                let disc = geom.rod_len * geom.rod_len - q.z * q.z;
                if disc < 0.0 {
                    assert!(solved.angle.is_none());
                    assert!(solved.margin < 0.0);
                    continue;
                }
                let r = disc.sqrt();
                let rho = q.x.hypot(q.y);
                let a = 2.0 * geom.crank_len * q.x;
                let b = 2.0 * geom.crank_len * q.y;
                let c = geom.crank_len * geom.crank_len + rho * rho - r * r;
                let d = a.hypot(b);

                if solved.margin.abs() < 1e-12 {
                    continue;
                }
                assert_eq!(
                    c.abs() <= d,
                    solved.margin >= 0.0,
                    "leg {} margin {} vs |C|={} D={}",
                    leg + 1,
                    solved.margin,
                    c.abs(),
                    d
                );
                assert_eq!(solved.angle.is_some(), solved.margin >= 0.0);
            }
        }
    }

    /// The reduction every clearance comparison shares: it is the least of the
    /// six, it agrees with solving the pose, and a leg with no clearance decides
    /// it however the others sit.
    #[test]
    fn the_minimum_is_one_reduction() {
        assert_eq!(
            min_margin(&[0.004, 0.003, 0.009, 0.005, 0.011, 0.007]),
            0.003
        );
        assert_eq!(min_margin(&[0.0; 6]), 0.0);
        assert_eq!(min_margin(&[-0.002, 0.5, 0.5, 0.5, 0.5, 0.5]), -0.002);

        let geom = HeadGeometry::default();
        for pose in [neutral_head_pose(), stow_head_pose(), rest_head_pose()] {
            let mut margins = [0.0; 6];
            pose_margins(&geom, &pose, &mut margins);
            assert_eq!(min_margin(&margins), min_pose_margin(&geom, &pose));
        }
    }

    /// Poses that walk out through a tangency: the margin crosses zero exactly
    /// where the solution disappears, from both the inner and the outer side.
    #[test]
    fn margin_crosses_zero_where_the_solution_disappears() {
        let geom = HeadGeometry::default();
        let mut last_solvable = None;
        let mut first_unsolvable = None;
        for step in 0..4000 {
            let z = 0.177 + f64::from(step) * 1e-5;
            let pose = Isometry3::translation(0.0, 0.0, z);
            let min = min_pose_margin(&geom, &pose);
            let mut angles = LegAngles::default();
            let solvable = inverse_kinematics(&geom, &pose, &mut angles).is_ok();
            assert_eq!(solvable, min >= 0.0, "z {z} min margin {min}");
            if solvable {
                last_solvable = Some(z);
            } else if first_unsolvable.is_none() {
                first_unsolvable = Some(z);
            }
        }
        let top = last_solvable.expect("some heights solve");
        let past = first_unsolvable.expect("the sweep leaves the workspace");
        assert!((0.2005..0.2020).contains(&top), "top of travel {top}");
        assert!(past > top);
    }

    /// An anchor farther from the swept plane than the whole rod: no solution,
    /// and the deficit is exactly how much rod is missing.
    #[test]
    fn out_of_plane_beyond_the_rod_is_unreachable() {
        let geom = HeadGeometry::default();
        // Translate hard along one pair's shared axis direction.
        let pose = Isometry3::translation(0.0, 0.30, baked::HEAD_Z_OFFSET);
        let err = inverse_kinematics(&geom, &pose, &mut LegAngles::default())
            .expect_err("0.3 m sideways is off the workspace");
        let IkError::Unreachable { leg, deficit } = err;
        assert!(deficit > 0.0, "deficit {deficit}");
        let solved = solve_leg(&geom, usize::from(leg), &pose);
        assert!((solved.deficit() - deficit).abs() < 1e-15);
    }

    /// Margins stay finite at every pose, including absurd ones. Nothing in
    /// this crate is allowed to hand a NaN onward.
    #[test]
    fn margins_are_finite_everywhere() {
        let geom = HeadGeometry::default();
        let mut rng = Rng::new(0x1234_5678_9abc_def1);
        for _ in 0..20_000 {
            let pose = random_pose(&mut rng, 2.0);
            let mut margins = [0.0; 6];
            pose_margins(&geom, &pose, &mut margins);
            for (leg, m) in margins.iter().enumerate() {
                assert!(m.is_finite(), "leg {} margin {m}", leg + 1);
            }
            let mut angles = LegAngles::default();
            if inverse_kinematics(&geom, &pose, &mut angles).is_ok() {
                for a in angles.0 {
                    assert!(a.is_finite());
                    assert!(a > -core::f64::consts::PI && a <= core::f64::consts::PI);
                }
            }
        }
    }

    /// The stow pose is solvable, well inside every travel window, and carries
    /// about 10 mm of clearance — twice what a bottom-of-vertical-travel
    /// assumption would predict for it.
    ///
    /// The per-leg window slack is pinned as a golden alongside the angles and
    /// margins, so a slip in one window bound names the leg it belongs to. The
    /// windows themselves are pinned by `vertical_travel_binds_where_the_windows_say`.
    #[test]
    fn stow_pose_golden() {
        let geom = HeadGeometry::default();
        let pose = stow_head_pose();
        let mut angles = LegAngles::default();
        inverse_kinematics(&geom, &pose, &mut angles).expect("stow is reachable");

        let mut margins = [0.0; 6];
        pose_margins(&geom, &pose, &mut margins);

        let expected_deg = [-9.498, 47.876, -6.958, 5.149, -46.576, 10.475];
        let expected_margin_mm = [10.143, 12.139, 24.144, 24.231, 12.235, 10.108];
        let expected_slack_deg = [38.502, 22.124, 41.042, 42.851, 23.424, 37.525];
        for leg in 0..6 {
            let deg = angles.0[leg].to_degrees();
            assert!(
                (deg - expected_deg[leg]).abs() < 0.001,
                "leg {} angle {deg}",
                leg + 1
            );
            assert!(
                (margins[leg] * 1000.0 - expected_margin_mm[leg]).abs() < 0.001,
                "leg {} margin {}",
                leg + 1,
                margins[leg]
            );
            let (lo, hi) = baked::CRANK_WINDOWS_DEG[leg];
            let slack = (deg - lo).min(hi - deg);
            assert!(
                (slack - expected_slack_deg[leg]).abs() < 0.001,
                "leg {} slack {slack}",
                leg + 1
            );
        }
    }

    /// The resting configuration the vendor's simulated backends start from sits
    /// far tighter than stow: 0.141 mm on the tightest leg, a twentieth of the
    /// clearance floor. Pinned as a golden rather than a band, because it is the
    /// number the whole clearance-baseline policy is sized against, and because
    /// the same configuration is on record a second way that yields 0.182 mm —
    /// a band loose enough to hold both would hide which record moved.
    #[test]
    fn candidate_resting_pose_is_much_tighter_than_stow() {
        let geom = HeadGeometry::default();
        let min = min_pose_margin(&geom, &rest_head_pose());
        assert!((min - 0.000_141_133).abs() < 1e-9, "min margin {min}");

        let stow_min = min_pose_margin(&geom, &stow_head_pose());
        assert!(stow_min > 40.0 * min, "stow {stow_min} vs rest {min}");
    }

    /// The angles this crate hands to the servos actually close the linkage:
    /// with the crank at the returned angle, the distance from the crank tip to
    /// the platform anchor is the rod length, to within a rounding error.
    ///
    /// Every other angle assertion here compares this code against numbers this
    /// code produced. This one compares it against the mechanism, so a swapped
    /// `atan2`, a sign slip on the branch term or a broken normalisation cannot
    /// hide behind re-baked goldens.
    #[test]
    fn crank_angles_close_the_kinematic_loop() {
        let geom = HeadGeometry::default();
        let mut rng = Rng::new(0x0bad_c0de_1234_5678);
        let mut checked = 0;
        let mut worst: f64 = 0.0;

        for _ in 0..5_000 {
            let pose = random_pose(&mut rng, 0.02);
            for (leg, l) in geom.legs.iter().enumerate() {
                let Some(angle) = solve_leg(&geom, leg, &pose).angle else {
                    continue;
                };
                let q = l.t_base_to_crank * (pose * l.anchor_head);
                let tip = Point3::new(
                    geom.crank_len * angle.cos(),
                    geom.crank_len * angle.sin(),
                    0.0,
                );
                let residual = (q - tip).norm() - geom.rod_len;
                worst = worst.max(residual.abs());
                assert!(
                    residual.abs() < 1e-12,
                    "leg {} residual {residual} at {pose:?}",
                    leg + 1
                );
                checked += 1;
            }
        }
        assert!(checked > 10_000, "only {checked} legs solved");
        assert!(worst > 0.0, "residuals are suspiciously exact");
    }

    /// What one direction of pure vertical travel runs into first.
    struct TravelSweep {
        /// Per leg, the height and angle at which it first leaves its window.
        left_window: [Option<(f64, f64)>; 6],
        /// Height at which the first leg leaves its window.
        first_bind_z: f64,
        /// Smallest toggle margin at that height.
        min_margin_at_bind: f64,
        /// Degrees of window each leg still has in hand at that height.
        slack_at_bind: [f64; 6],
        /// First height with no solution at all.
        first_unsolvable: f64,
    }

    /// Steps vertically from the neutral height in 10 µm increments, recording
    /// where each leg leaves its travel window and where the linkage runs out
    /// of solutions altogether.
    fn sweep_vertical_travel(geom: &HeadGeometry, direction: f64) -> TravelSweep {
        let mut sweep = TravelSweep {
            left_window: [None; 6],
            first_bind_z: f64::NAN,
            min_margin_at_bind: f64::NAN,
            slack_at_bind: [0.0; 6],
            first_unsolvable: f64::NAN,
        };
        for step in 0..12_000 {
            let z = baked::HEAD_Z_OFFSET + direction * f64::from(step) * 1e-5;
            let pose = Isometry3::translation(0.0, 0.0, z);
            let mut angles = LegAngles::default();
            if inverse_kinematics(geom, &pose, &mut angles).is_err() {
                if sweep.first_unsolvable.is_nan() {
                    sweep.first_unsolvable = z;
                }
                continue;
            }
            let mut slack = [0.0; 6];
            let mut newly_bound = false;
            for (leg, slot) in sweep.left_window.iter_mut().enumerate() {
                let deg = angles.0[leg].to_degrees();
                let (lo, hi) = baked::CRANK_WINDOWS_DEG[leg];
                slack[leg] = (deg - lo).min(hi - deg);
                if slack[leg] < 0.0 && slot.is_none() {
                    *slot = Some((z, deg));
                    newly_bound = true;
                }
            }
            if newly_bound && sweep.first_bind_z.is_nan() {
                sweep.first_bind_z = z;
                sweep.min_margin_at_bind = min_pose_margin(geom, &pose);
                sweep.slack_at_bind = slack;
            }
        }
        sweep
    }

    /// Where the travel windows actually bind, swept in both directions.
    ///
    /// Downward, legs 1, 3, 4 and 6 hit their 48° bound together while legs 2
    /// and 5 still hold 22° in reserve: the bottom stop rests on four legs, and
    /// a symmetric window copied from legs 2 and 5 would drive the other four
    /// straight past it. Upward all six bind together at 80°, a bit over a
    /// millimetre of height short of the tangency where solutions vanish — the
    /// clearance a change of a few millimetres in the link lengths would eat.
    #[test]
    fn vertical_travel_binds_where_the_windows_say() {
        let geom = HeadGeometry::default();

        let down = sweep_vertical_travel(&geom, -1.0);
        for leg in [0, 2, 3, 5] {
            let (z, deg) = down.left_window[leg].expect("the four-leg stop is reached");
            assert!((z - 0.126_17).abs() < 1.5e-5, "leg {} at {z}", leg + 1);
            assert!((deg.abs() - 48.0).abs() < 0.02, "leg {} at {deg}°", leg + 1);
        }
        for leg in [1, 4] {
            let (z, deg) = down.left_window[leg].expect("legs 2 and 5 bind lower down");
            assert!((z - 0.122_05).abs() < 1.5e-5, "leg {} at {z}", leg + 1);
            assert!((deg.abs() - 70.0).abs() < 0.06, "leg {} at {deg}°", leg + 1);
            assert!(
                (down.slack_at_bind[leg] - 21.99).abs() < 0.02,
                "leg {} reserve {}",
                leg + 1,
                down.slack_at_bind[leg]
            );
        }
        assert!(
            (down.min_margin_at_bind - 0.004_958).abs() < 1e-6,
            "bottom stop margin {}",
            down.min_margin_at_bind
        );
        assert!(
            (down.first_unsolvable - 0.121_20).abs() < 1.5e-5,
            "bottom tangency {}",
            down.first_unsolvable
        );

        let up = sweep_vertical_travel(&geom, 1.0);
        for (leg, bind) in up.left_window.iter().enumerate() {
            let (z, deg) = bind.expect("every leg binds on the way up");
            assert!((z - 0.200_11).abs() < 1.5e-5, "leg {} at {z}", leg + 1);
            assert!((deg.abs() - 80.0).abs() < 0.02, "leg {} at {deg}°", leg + 1);
        }
        assert!(
            (up.min_margin_at_bind - 0.001_156).abs() < 1e-6,
            "top stop margin {}",
            up.min_margin_at_bind
        );
        assert!(
            (up.first_unsolvable - 0.201_27).abs() < 1.5e-5,
            "top tangency {}",
            up.first_unsolvable
        );
        let headroom = up.first_unsolvable - up.first_bind_z;
        assert!(
            (headroom - 0.001_16).abs() < 3e-5,
            "the top stop leaves {headroom} m to the tangency"
        );
    }

    /// A pose with no solution leaves the caller's angles exactly as they were.
    /// Blending freshly solved legs with stale ones would describe a pose that
    /// was never commanded, and the failing leg is deliberately not the first
    /// one — where a partial write is invisible.
    #[test]
    fn unreachable_pose_leaves_the_output_untouched() {
        let geom = HeadGeometry::default();
        let pose = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, baked::HEAD_Z_OFFSET),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 1.2),
        );
        let mut out = LegAngles([7.0; 6]);
        let err = inverse_kinematics(&geom, &pose, &mut out).expect_err("1.2 rad of pitch is off");
        let IkError::Unreachable { leg, .. } = err;
        assert_eq!(leg, 2, "the fixture must fail on a leg after the first");
        assert_eq!(out.0, [7.0; 6]);
    }

    /// The float `z` shifted by `steps` representable values.
    fn ulp_step(z: f64, steps: i64) -> f64 {
        let bits = z.to_bits() as i64 + steps;
        f64::from_bits(bits as u64)
    }

    /// The lemma where its two sides can actually disagree.
    ///
    /// `solvability_and_margin_sign_agree` samples poses at random, which reach
    /// a tangency with probability zero. Here the boundary is bisected to
    /// adjacent floating-point heights and then stepped across in single
    /// representable increments, so every sample sits within nanometres of a
    /// merged root. The two sides are asserted to agree outside a stated band;
    /// inside it they are free to differ by rounding, and the solver's
    /// `margin.min(0.0)` on a refused leg keeps that difference from ever
    /// reporting positive clearance for a pose with no solution.
    #[test]
    fn lemma_holds_at_tangency() {
        let geom = HeadGeometry::default();
        let solves = |x: f64, y: f64, z: f64| {
            let pose = Isometry3::translation(x, y, z);
            inverse_kinematics(&geom, &pose, &mut LegAngles::default()).is_ok()
        };

        let mut reached = 0;
        for (x, y) in [(0.0, 0.0), (0.01, 0.0), (0.0, -0.012), (0.008, 0.006)] {
            for far in [0.2013, 0.1210] {
                assert!(solves(x, y, baked::HEAD_Z_OFFSET), "seed pose must solve");
                assert!(!solves(x, y, far), "far pose must not solve");

                let mut inside = baked::HEAD_Z_OFFSET;
                let mut outside = far;
                for _ in 0..80 {
                    let mid = 0.5 * (inside + outside);
                    if mid == inside || mid == outside {
                        break;
                    }
                    if solves(x, y, mid) {
                        inside = mid;
                    } else {
                        outside = mid;
                    }
                }

                for step in -4i64..=4 {
                    let z = ulp_step(inside, step);
                    let pose = Isometry3::translation(x, y, z);
                    for (leg, l) in geom.legs.iter().enumerate() {
                        let solved = solve_leg(&geom, leg, &pose);
                        assert!(solved.margin.is_finite());
                        if solved.margin.abs() < 1e-9 {
                            reached += 1;
                        }

                        let q = l.t_base_to_crank * (pose * l.anchor_head);
                        let disc = geom.rod_len * geom.rod_len - q.z * q.z;
                        if disc < 0.0 {
                            continue;
                        }
                        let r = disc.sqrt();
                        let rho = q.x.hypot(q.y);
                        let c = geom.crank_len * geom.crank_len + rho * rho - r * r;
                        let d = 2.0 * geom.crank_len * rho;

                        // A picometre: far below any disagreement rounding can
                        // produce here, far above zero.
                        if solved.margin.abs() > 1e-12 {
                            assert_eq!(
                                c.abs() <= d,
                                solved.margin >= 0.0,
                                "leg {} margin {} vs |C|={} D={}",
                                leg + 1,
                                solved.margin,
                                c.abs(),
                                d
                            );
                            assert_eq!(solved.angle.is_some(), solved.margin >= 0.0);
                        }
                        assert!(solved.angle.is_some() || solved.margin <= 0.0);
                    }
                }
            }
        }
        assert!(reached > 0, "no sample landed near a tangency");
    }

    #[test]
    fn angles_normalise_into_the_half_open_turn() {
        assert!((wrap_to_pi(core::f64::consts::PI) - core::f64::consts::PI).abs() < 1e-15);
        assert!((wrap_to_pi(-core::f64::consts::PI) - core::f64::consts::PI).abs() < 1e-15);
        assert!(wrap_to_pi(0.0).abs() < 1e-15);
        assert!(
            (wrap_to_pi(3.0 * core::f64::consts::FRAC_PI_2) + core::f64::consts::FRAC_PI_2).abs()
                < 1e-15
        );
    }
}
