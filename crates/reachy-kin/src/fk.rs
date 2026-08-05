//! Forward kinematics: six crank angles in, one head pose out, by damped Newton
//! on the loop closure.
//!
//! There is no closed form. Six rods of fixed length pin the platform, and the
//! system that says so is six coupled equations in the six degrees of freedom of
//! a rigid body. Worse, it is genuinely multi-valued: the same six crank angles
//! admit several assembly modes, poses the mechanism could physically be in with
//! the motors exactly where they are. Only one of them is the pose the platform
//! is actually holding, and no amount of arithmetic distinguishes them — the
//! caller's seed does.
//!
//! ## The seed is an argument, and the solver keeps no state
//!
//! The seed says which assembly mode to look for; a solver that remembered its
//! last answer instead would silently change what it means for every subsequent
//! call, and two identical calls could return different poses. So the seed comes
//! in through the signature, and a caller that wants the pose near where the
//! platform was last seen passes that pose.
//!
//! When the iteration fails, it fails. It never perturbs the inputs and tries
//! again: a retry loop over jittered angles turns "I cannot tell where the head
//! is" into a plausible-looking pose derived from numbers nobody measured, and
//! the commanded motion computed from it is real. A failure is a typed error and
//! the caller decides — re-seed, or fault.
//!
//! ## The iteration
//!
//! With the crank angles fixed, each crank tip is a fixed point in the base
//! frame, so leg *i* contributes one scalar residual in the unknown pose `T`:
//!
//! ```text
//! f_i(T) = |T·anchor_i − tip_i| − rod_len
//! ```
//!
//! Newton needs a derivative, and a pose has no useful vector-space structure to
//! differentiate in. What is parameterised instead is the *update*: a small
//! twist `δ = (δt, δw)` left-composed onto the current iterate, under which each
//! anchor point `a` moves by `δt + δw × a` to first order. That gives the exact
//! Jacobian rows below, and keeps every iterate an exact rigid transform rather
//! than a matrix drifting off the rotation group.
//!
//! Damping is step-halving under a residual-decrease test rather than a
//! Levenberg-style parameter: near a singular configuration the Jacobian is
//! ill-conditioned and the Newton step is enormous, and the honest response is a
//! shorter step that demonstrably improves the residual, or an admission of
//! defeat.
//!
//! ## The plausibility screen
//!
//! Convergence proves the pose closes the linkage — not that it is the one the
//! platform is in. A converged pose is therefore screened for gross
//! implausibility: an upside-down or wildly displaced head is reported as the
//! wrong assembly mode rather than handed on. The screen bounds are deliberately
//! far looser than the travel envelope; they separate assembly modes, which sit
//! most of a turn apart, and enforce nothing. Anything tighter is the envelope's
//! job, on the envelope's error type.

use nalgebra::{Isometry3, Matrix6, Point3, Vector3, Vector6};
use thiserror::Error;

use crate::geometry::HeadGeometry;
use crate::ik::LegAngles;

/// Per-iteration cap on the translational part of a step, metres. The whole
/// workspace is about 80 mm tall, so a step this size already crosses it.
const MAX_STEP_TRANSLATION: f64 = 0.05;

/// Per-iteration cap on the rotational part of a step, radians — roughly 29°,
/// against a head that tilts 35° before the envelope stops it.
const MAX_STEP_ROTATION: f64 = 0.5;

/// How many times a rejected step is halved before the solve gives up. Six
/// halvings take a capped step down to under a millimetre.
const MAX_HALVINGS: u32 = 6;

/// Rod separations below this have no trustworthy direction, metres. The rod is
/// 85 mm and the anchors never approach the crank tips, so this is unreachable
/// from any real configuration and exists to keep a divergent iterate from
/// normalising a vector that is almost zero.
const MIN_ROD_SEPARATION: f64 = 0.001;

/// Iteration budget, tolerance, and the assembly-mode screen.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FkOptions {
    /// Newton iterations before giving up.
    pub max_iters: u32,
    /// Convergence tolerance on the largest per-leg loop-closure residual,
    /// metres. A nanometre: the iteration converges quadratically, so this
    /// costs an iteration or two over a looser figure and leaves no doubt that
    /// the answer is the root rather than its neighbourhood.
    pub tol_m: f64,
    /// Largest head tilt from vertical a converged pose may have and still be
    /// believed, radians.
    pub screen_cone: f64,
    /// Band of head heights a converged pose may lie in and still be believed,
    /// metres, as `(low, high)`.
    pub screen_z: (f64, f64),
}

impl Default for FkOptions {
    fn default() -> Self {
        Self {
            max_iters: 50,
            tol_m: 1e-9,
            // The head tilts 35° at the envelope's limit and both candidate
            // resting configurations sit under 31°, while a flipped assembly
            // mode is most of a half turn away. 60° separates those without
            // coming near anything legitimate.
            screen_cone: 60_f64.to_radians(),
            // Physical travel is 121–201 mm; the band is opened at both ends so
            // that a pose just outside the workspace still reads as a pose.
            screen_z: (0.09, 0.24),
        }
    }
}

/// What the iteration cost and how well it converged.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FkStats {
    /// Newton iterations taken. Zero when the seed already satisfied the
    /// tolerance.
    pub iters: u32,
    /// The largest per-leg loop-closure residual at the returned pose, metres.
    pub residual: f64,
}

/// Why a pose could not be determined.
///
/// Both variants mean the caller gets no pose at all. Neither a partially
/// converged iterate nor the previous answer is a measurement of where the head
/// is, and anything commanded from one would be commanded from fiction.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum FkError {
    /// The iteration ran out of budget, or ran out of usable steps: a singular
    /// Jacobian, or no step short enough to reduce the residual.
    #[error("no pose found after {iters} iterations, residual {residual:.3e} m")]
    NoConvergence {
        /// Iterations taken before giving up.
        iters: u32,
        /// The largest per-leg residual reached, metres.
        residual: f64,
    },
    /// A pose was found that closes the linkage but is not one the platform
    /// could plausibly be holding — the iteration landed in a different
    /// assembly mode than the seed asked for.
    #[error("converged onto an implausible pose: tilt {cone_deg:.1}°, height {z:.4} m")]
    WrongAssemblyMode {
        /// Head tilt from vertical at the converged pose, degrees.
        cone_deg: f64,
        /// Head height at the converged pose, metres.
        z: f64,
    },
}

/// The loop-closure state at one iterate: residuals, and the geometry the
/// Jacobian needs, computed together so they describe the same pose.
struct Closure {
    /// Per-leg `|a_i − tip_i| − rod_len`, metres.
    residual: Vector6<f64>,
    /// Anchor positions in the base frame.
    anchors: [Point3<f64>; 6],
    /// Unit vectors from crank tip to anchor. Meaningless, and never read, when
    /// `degenerate` is set.
    dirs: [Vector3<f64>; 6],
    /// Some leg's tip and anchor are close enough that the direction between
    /// them carries no information.
    degenerate: bool,
}

impl Closure {
    /// The convergence measure: the largest per-leg residual, metres.
    fn worst(&self) -> f64 {
        self.residual.amax()
    }

    /// The 6×6 Jacobian of the residuals with respect to a left-composed twist
    /// `δ = (δt, δw)`.
    ///
    /// An anchor at `a` moves by `δt + δw × a`, and only the component along
    /// the rod changes its length, so `∂f_i/∂δt = u_iᵀ` and, by the triple
    /// product identity `u · (δw × a) = δw · (a × u)`, `∂f_i/∂δw = (a_i × u_i)ᵀ`.
    fn jacobian(&self) -> Matrix6<f64> {
        let mut j = Matrix6::zeros();
        for leg in 0..6 {
            let u = self.dirs[leg];
            let moment = self.anchors[leg].coords.cross(&u);
            for axis in 0..3 {
                j[(leg, axis)] = u[axis];
                j[(leg, 3 + axis)] = moment[axis];
            }
        }
        j
    }
}

/// The six crank tips in the base frame, fixed for the whole solve because the
/// crank angles are the inputs.
fn crank_tips(geom: &HeadGeometry, angles: &LegAngles) -> [Point3<f64>; 6] {
    core::array::from_fn(|leg| {
        let theta = angles.0[leg];
        let tip_crank = Point3::new(
            geom.crank_len * theta.cos(),
            geom.crank_len * theta.sin(),
            0.0,
        );
        geom.legs[leg]
            .t_base_to_crank
            .inverse_transform_point(&tip_crank)
    })
}

/// Residuals and rod directions at one candidate pose.
fn evaluate(geom: &HeadGeometry, tips: &[Point3<f64>; 6], pose: &Isometry3<f64>) -> Closure {
    let mut closure = Closure {
        residual: Vector6::zeros(),
        anchors: [Point3::origin(); 6],
        dirs: [Vector3::zeros(); 6],
        degenerate: false,
    };
    for (leg, tip) in tips.iter().enumerate() {
        let anchor = pose * geom.legs[leg].anchor_head;
        let separation = anchor - tip;
        let distance = separation.norm();
        closure.anchors[leg] = anchor;
        closure.residual[leg] = distance - geom.rod_len;
        if distance < MIN_ROD_SEPARATION {
            closure.degenerate = true;
        } else {
            closure.dirs[leg] = separation / distance;
        }
    }
    closure
}

/// The head pose that the given crank angles hold, sought near `seed`.
///
/// On success `out` is the pose and the return value says what it cost. On
/// failure `out` is left exactly as it arrived: there is no partial answer here
/// to salvage, and overwriting it with a half-converged iterate would hand the
/// caller a pose it had no way to recognise as fiction.
///
/// The seed selects the assembly mode. Calling this with a seed far from the
/// true pose is not an error, but it is a question about a different branch of
/// the mechanism, and it will be answered as one.
pub fn forward_kinematics(
    geom: &HeadGeometry,
    angles: &LegAngles,
    seed: &Isometry3<f64>,
    opts: &FkOptions,
    out: &mut Isometry3<f64>,
) -> Result<FkStats, FkError> {
    let tips = crank_tips(geom, angles);
    let mut pose = *seed;
    let mut closure = evaluate(geom, &tips, &pose);
    let mut residual = closure.worst();
    let mut iters = 0;

    while residual >= opts.tol_m {
        if iters >= opts.max_iters || closure.degenerate {
            return Err(FkError::NoConvergence { iters, residual });
        }
        iters += 1;

        // A singular Jacobian leaves no step to shorten, so the halving loop
        // below has nothing to work with and the solve is over.
        let Some(step) = closure.jacobian().lu().solve(&(-closure.residual)) else {
            return Err(FkError::NoConvergence { iters, residual });
        };

        // Nothing is clamped: an oversized step is retried shorter or
        // abandoned, never truncated to the cap and applied as if it were the
        // step Newton asked for.
        let mut accepted = None;
        let mut scale = 1.0;
        for _ in 0..=MAX_HALVINGS {
            let translation = step.fixed_rows::<3>(0) * scale;
            let rotation = step.fixed_rows::<3>(3) * scale;
            if translation.norm() <= MAX_STEP_TRANSLATION && rotation.norm() <= MAX_STEP_ROTATION {
                let candidate = Isometry3::new(translation, rotation) * pose;
                let evaluated = evaluate(geom, &tips, &candidate);
                let worst = evaluated.worst();
                if worst < residual {
                    accepted = Some((candidate, evaluated, worst));
                    break;
                }
            }
            scale *= 0.5;
        }

        let Some((next_pose, next_closure, next_residual)) = accepted else {
            return Err(FkError::NoConvergence { iters, residual });
        };
        pose = next_pose;
        closure = next_closure;
        residual = next_residual;
    }

    // The clamp is on a component of a unit vector, which rounding can push an
    // ulp outside [−1, 1] at either end — the near-inverted poses this screen
    // exists to catch sit at the −1 end. It is arithmetic hygiene before
    // `acos`, not a cap on anything commanded.
    let cone = (pose.rotation * Vector3::z()).z.clamp(-1.0, 1.0).acos();
    let z = pose.translation.z;
    if cone > opts.screen_cone || z < opts.screen_z.0 || z > opts.screen_z.1 {
        return Err(FkError::WrongAssemblyMode {
            cone_deg: cone.to_degrees(),
            z,
        });
    }

    *out = pose;
    Ok(FkStats { iters, residual })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::baked;
    use crate::geometry::{neutral_head_pose, stow_head_pose};
    use crate::ik::inverse_kinematics;
    use crate::testutil::Rng;
    use nalgebra::{Translation3, UnitQuaternion};

    /// The crank angles of a pose the platform can actually hold.
    fn angles_of(geom: &HeadGeometry, pose: &Isometry3<f64>) -> LegAngles {
        let mut angles = LegAngles::default();
        inverse_kinematics(geom, pose, &mut angles).expect("fixture pose must be reachable");
        angles
    }

    /// A pose near neutral, with `reach` metres of offset and `tilt` radians of
    /// rotation to play with.
    fn random_pose(rng: &mut Rng, reach: f64, tilt: f64) -> Isometry3<f64> {
        let axis = Vector3::new(
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
            rng.range(-1.0, 1.0),
        );
        let rotation = match nalgebra::Unit::try_new(axis, 1e-9) {
            Some(unit) => UnitQuaternion::from_axis_angle(&unit, rng.range(-tilt, tilt)),
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

    /// How far apart two poses are: metres, and radians of relative rotation.
    fn pose_gap(a: &Isometry3<f64>, b: &Isometry3<f64>) -> (f64, f64) {
        (
            (a.translation.vector - b.translation.vector).norm(),
            a.rotation.angle_to(&b.rotation),
        )
    }

    /// Crank angles of the vendor's simulated resting configuration, recorded
    /// independently.
    fn candidate_b_angles() -> LegAngles {
        LegAngles([-56.43, 72.33, -13.97, 11.78, -70.84, 57.48].map(f64::to_radians))
    }

    /// The neutral pose's own crank angles put the head back at neutral, from a
    /// seed deliberately displaced by centimetres and by a tenth of a radian so
    /// that returning the seed unchanged would fail.
    #[test]
    fn neutral_angles_recover_the_neutral_pose() {
        let geom = HeadGeometry::default();
        let angles = angles_of(&geom, &neutral_head_pose());
        let seed = Isometry3::from_parts(
            Translation3::new(0.01, -0.012, baked::HEAD_Z_OFFSET + 0.015),
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 0.1),
        );

        let mut out = Isometry3::identity();
        let stats = forward_kinematics(&geom, &angles, &seed, &FkOptions::default(), &mut out)
            .expect("neutral is a plausible pose");

        let (metres, radians) = pose_gap(&out, &neutral_head_pose());
        assert!(metres < 1e-9, "translation off by {metres} m");
        assert!(radians < 1e-8, "rotation off by {radians} rad");
        assert!(stats.iters > 0 && stats.iters < 20, "iters {}", stats.iters);
        assert!(stats.residual < 1e-9, "residual {}", stats.residual);
    }

    /// Inverse then forward over the workspace: the pose that comes back is the
    /// pose that went in, from a seed several millimetres and several degrees
    /// away from it.
    #[test]
    fn round_trips_from_random_admissible_poses() {
        let geom = HeadGeometry::default();
        let opts = FkOptions::default();
        let mut rng = Rng::new(0x00fb_1a5e_0011_2233);
        let mut solved = 0;
        let mut worst_iters = 0;

        for _ in 0..2_000 {
            let pose = random_pose(&mut rng, 0.012, 0.25);
            let mut angles = LegAngles::default();
            if inverse_kinematics(&geom, &pose, &mut angles).is_err() {
                continue;
            }
            let seed = Isometry3::from_parts(
                Translation3::new(
                    pose.translation.x + 0.005,
                    pose.translation.y - 0.004,
                    pose.translation.z + 0.006,
                ),
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.08) * pose.rotation,
            );

            let mut out = Isometry3::identity();
            let stats = forward_kinematics(&geom, &angles, &seed, &opts, &mut out)
                .expect("a pose reached by inverse kinematics is recoverable from a close seed");
            let (metres, radians) = pose_gap(&out, &pose);
            assert!(metres < 1e-8, "translation off by {metres} m at {pose:?}");
            assert!(radians < 1e-7, "rotation off by {radians} rad at {pose:?}");
            worst_iters = worst_iters.max(stats.iters);
            solved += 1;
        }
        assert!(solved > 1_000, "only {solved} poses solved");
        assert!(worst_iters <= 8, "worst case took {worst_iters} iterations");
    }

    /// The analytic Jacobian against central finite differences of the residual
    /// itself, at random poses and in all six twist directions.
    ///
    /// This is the derivation under test. Every convergence result in this file
    /// would survive a Jacobian that was merely close enough to still converge,
    /// just more slowly; only differencing the residual catches a transposed
    /// cross product or a swapped translation and rotation block.
    #[test]
    fn jacobian_matches_finite_differences() {
        let geom = HeadGeometry::default();
        let mut rng = Rng::new(0x1acb_0912_7fed_3344);
        let mut worst: f64 = 0.0;
        let mut checked = 0;

        for _ in 0..200 {
            let pose = random_pose(&mut rng, 0.012, 0.25);
            let mut angles = LegAngles::default();
            if inverse_kinematics(&geom, &pose, &mut angles).is_err() {
                continue;
            }
            // Evaluate away from the solution, where the residuals and their
            // derivatives are both nonzero.
            let at = Isometry3::from_parts(
                Translation3::new(
                    pose.translation.x + 0.003,
                    pose.translation.y + 0.002,
                    pose.translation.z - 0.004,
                ),
                UnitQuaternion::from_axis_angle(&Vector3::z_axis(), 0.05) * pose.rotation,
            );
            let tips = crank_tips(&geom, &angles);
            let analytic = evaluate(&geom, &tips, &at).jacobian();

            let h = 1e-7;
            for axis in 0..6 {
                let mut twist = Vector6::zeros();
                twist[axis] = h;
                let bump = |sign: f64| {
                    let step = twist * sign;
                    let moved = Isometry3::new(
                        Vector3::new(step[0], step[1], step[2]),
                        Vector3::new(step[3], step[4], step[5]),
                    ) * at;
                    evaluate(&geom, &tips, &moved).residual
                };
                let numeric = (bump(1.0) - bump(-1.0)) / (2.0 * h);

                for leg in 0..6 {
                    let scale = numeric[leg].abs().max(1e-3);
                    let relative = (analytic[(leg, axis)] - numeric[leg]).abs() / scale;
                    worst = worst.max(relative);
                    assert!(
                        relative < 1e-6,
                        "leg {} axis {axis}: analytic {} vs numeric {}",
                        leg + 1,
                        analytic[(leg, axis)],
                        numeric[leg]
                    );
                }
                checked += 1;
            }
        }
        assert!(checked > 600, "only {checked} directions differenced");
        assert!(worst > 0.0, "the two agreed exactly, which is suspicious");
    }

    /// The seeded-Newton acceptance test, and a cross-check between two numbers
    /// that were written down separately.
    ///
    /// The tight resting configuration is on record twice: as six crank angles,
    /// and as the head pose recovered from them. Starting from the *other*
    /// candidate resting configuration — centimetres and degrees away — the
    /// solve crosses to the pose that belongs to those angles, and it is the
    /// recorded one to within a couple of microns.
    #[test]
    fn candidate_b_angles_recover_their_recorded_pose() {
        let geom = HeadGeometry::default();
        let recorded = Isometry3::from_parts(
            Translation3::new(-0.015_17, 0.001_03, 0.126_57),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 30.84_f64.to_radians()),
        );

        let mut out = Isometry3::identity();
        forward_kinematics(
            &geom,
            &candidate_b_angles(),
            &stow_head_pose(),
            &FkOptions::default(),
            &mut out,
        )
        .expect("the resting configuration is a plausible pose");

        let (metres, radians) = pose_gap(&out, &recorded);
        assert!(metres < 5e-6, "translation off by {metres} m");
        assert!(radians.to_degrees() < 0.35, "rotation off by {radians} rad");
        let (seed_metres, _) = pose_gap(&stow_head_pose(), &recorded);
        assert!(
            seed_metres > 0.006,
            "seed was already there ({seed_metres} m)"
        );
    }

    /// The same six crank angles that hold the head at neutral also close the
    /// linkage with the platform 15 cm lower. That pose is a solution and not a
    /// measurement, and the screen refuses it rather than reporting the head is
    /// somewhere it visibly is not.
    #[test]
    fn a_low_assembly_mode_is_refused() {
        let geom = HeadGeometry::default();
        let angles = angles_of(&geom, &neutral_head_pose());

        let mut out = Isometry3::translation(1.0, 2.0, 3.0);
        let err = forward_kinematics(
            &geom,
            &angles,
            &Isometry3::translation(0.0, 0.0, -0.05),
            &FkOptions::default(),
            &mut out,
        )
        .expect_err("a pose 15 cm below the workspace is not believable");

        let FkError::WrongAssemblyMode { cone_deg, z } = err else {
            panic!("expected the assembly-mode screen, got {err:?}");
        };
        assert!((z - 0.0232).abs() < 1e-3, "second mode at z {z}");
        assert!(
            cone_deg < 1.0,
            "that mode is level, not tilted: {cone_deg}°"
        );
        assert_eq!(out, Isometry3::translation(1.0, 2.0, 3.0));
    }

    /// Another mode of the same angles, this one tilted past a right angle.
    /// Caught by the cone half of the screen rather than the height half.
    #[test]
    fn a_tilted_assembly_mode_is_refused() {
        let geom = HeadGeometry::default();
        let angles = angles_of(&geom, &neutral_head_pose());
        let seed = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, 0.05),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 2.8),
        );

        let mut out = Isometry3::identity();
        let err = forward_kinematics(&geom, &angles, &seed, &FkOptions::default(), &mut out)
            .expect_err("a head lying on its side is not believable");
        let FkError::WrongAssemblyMode { cone_deg, .. } = err else {
            panic!("expected the assembly-mode screen, got {err:?}");
        };
        assert!(cone_deg > 90.0, "tilt {cone_deg}°");
    }

    /// The screen is a coarse filter, not a proof, and this pins the gap: a
    /// half-turn seed reaches a mode that is neither neutral nor implausible
    /// enough for the bounds to reject, and it is returned.
    ///
    /// What keeps the caller on the right branch is the seed — the previous
    /// tick's pose, a few millimetres away — never the screen. Tightening the
    /// bounds until this case is caught would start rejecting real poses, so
    /// the case is recorded rather than defended against.
    #[test]
    fn the_screen_does_not_separate_every_mode() {
        let geom = HeadGeometry::default();
        let angles = angles_of(&geom, &neutral_head_pose());
        let seed = Isometry3::from_parts(
            Translation3::new(0.0, 0.0, baked::HEAD_Z_OFFSET),
            UnitQuaternion::from_axis_angle(&Vector3::x_axis(), core::f64::consts::PI),
        );

        let mut out = Isometry3::identity();
        forward_kinematics(&geom, &angles, &seed, &FkOptions::default(), &mut out)
            .expect("this mode passes the plausibility bounds");
        let (metres, radians) = pose_gap(&out, &neutral_head_pose());
        assert!(metres > 0.01, "reached a distinct mode, {metres} m away");
        assert!(radians > 1.0, "and {radians} rad away");
    }

    /// Six crank angles that no pose can satisfy: the solve runs out of steps
    /// that improve anything and says so, leaving the caller's pose alone. A
    /// half-converged iterate here would be a fabricated measurement.
    #[test]
    fn angles_that_close_no_loop_produce_no_pose() {
        let geom = HeadGeometry::default();
        let angles = LegAngles([core::f64::consts::FRAC_PI_2; 6]);
        let before = Isometry3::translation(0.4, 0.5, 0.6);

        let mut out = before;
        let err = forward_kinematics(
            &geom,
            &angles,
            &neutral_head_pose(),
            &FkOptions::default(),
            &mut out,
        )
        .expect_err("all six cranks at a quarter turn close nothing");
        let FkError::NoConvergence { residual, .. } = err else {
            panic!("expected non-convergence, got {err:?}");
        };
        assert!(
            residual.is_finite() && residual > 0.0,
            "residual {residual}"
        );
        assert_eq!(out, before);
    }

    /// A budget too small for the seed reports the budget it spent, not a pose.
    #[test]
    fn an_exhausted_iteration_budget_reports_what_it_spent() {
        let geom = HeadGeometry::default();
        let angles = angles_of(&geom, &neutral_head_pose());
        let opts = FkOptions {
            max_iters: 2,
            ..FkOptions::default()
        };
        let seed = Isometry3::from_parts(
            Translation3::new(0.05, 0.05, 0.30),
            UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.9),
        );

        let mut out = Isometry3::identity();
        let err = forward_kinematics(&geom, &angles, &seed, &opts, &mut out)
            .expect_err("two iterations from that seed is not enough");
        let FkError::NoConvergence { iters, residual } = err else {
            panic!("expected non-convergence, got {err:?}");
        };
        assert_eq!(iters, 2);
        assert!(residual.is_finite());
        assert_eq!(out, Isometry3::identity());

        // The same seed with the standard budget does converge, so the failure
        // above is the budget and nothing else.
        let mut recovered = Isometry3::identity();
        forward_kinematics(&geom, &angles, &seed, &FkOptions::default(), &mut recovered)
            .expect("fifty iterations reach it");
    }

    /// An anchor sitting on its crank tip leaves the rod with no direction, so
    /// the Jacobian row would be noise. The solve stops instead of stepping on
    /// a normalised near-zero vector.
    ///
    /// Unreachable on this mechanism — the rod is 85 mm and the anchors never
    /// come near the tips — so the geometry is doctored to produce it.
    #[test]
    fn a_degenerate_rod_direction_stops_the_solve() {
        let mut geom = HeadGeometry::default();
        let angles = angles_of(&geom, &neutral_head_pose());
        // Leg 1's anchor moved onto the head origin, and the head placed at
        // leg 1's crank tip.
        let tip = crank_tips(&geom, &angles)[0];
        geom.legs[0].anchor_head = Point3::origin();
        let seed =
            Isometry3::from_parts(Translation3::from(tip.coords), UnitQuaternion::identity());

        let mut out = Isometry3::identity();
        let err = forward_kinematics(&geom, &angles, &seed, &FkOptions::default(), &mut out)
            .expect_err("no rod direction, no step");
        assert!(
            matches!(err, FkError::NoConvergence { iters: 0, .. }),
            "got {err:?}"
        );
        assert_eq!(out, Isometry3::identity());
    }

    /// A seed that already satisfies the tolerance is the answer, at no cost.
    #[test]
    fn a_seed_at_the_solution_takes_no_iterations() {
        let geom = HeadGeometry::default();
        let angles = angles_of(&geom, &neutral_head_pose());

        let mut out = Isometry3::identity();
        let stats = forward_kinematics(
            &geom,
            &angles,
            &neutral_head_pose(),
            &FkOptions::default(),
            &mut out,
        )
        .expect("neutral solves neutral");
        assert_eq!(stats.iters, 0);
        assert_eq!(out, neutral_head_pose());
    }

    /// Two identical calls give identical answers, bit for bit. The solver
    /// carries no state between calls and there is nothing for a second call to
    /// remember.
    #[test]
    fn identical_calls_agree_exactly() {
        let geom = HeadGeometry::default();
        let angles = candidate_b_angles();
        let opts = FkOptions::default();

        let mut first = Isometry3::identity();
        let a = forward_kinematics(&geom, &angles, &stow_head_pose(), &opts, &mut first)
            .expect("converges");
        let mut second = Isometry3::identity();
        let b = forward_kinematics(&geom, &angles, &stow_head_pose(), &opts, &mut second)
            .expect("converges");

        assert_eq!(a, b);
        assert_eq!(first, second);
    }

    /// The reported residual is the one the returned pose actually has, not a
    /// number from some earlier iterate.
    #[test]
    fn reported_stats_describe_the_returned_pose() {
        let geom = HeadGeometry::default();
        let opts = FkOptions::default();
        let angles = candidate_b_angles();

        let mut out = Isometry3::identity();
        let stats = forward_kinematics(&geom, &angles, &stow_head_pose(), &opts, &mut out)
            .expect("converges");

        let recomputed = evaluate(&geom, &crank_tips(&geom, &angles), &out).worst();
        assert_eq!(recomputed, stats.residual);
        assert!(stats.residual < opts.tol_m);
    }
}
