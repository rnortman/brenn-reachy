//! Trajectories: a shaped path between two command sets.
//!
//! The host does the shaping. Every gentle movement in this system is an
//! interpolation computed here and emitted as a bounded increment once per
//! tick, so the servos' own profile registers — which do shape each written
//! goal's approach, which is why arming writes them — are left as the backstop
//! under a goal step the host got wrong rather than as the thing that makes
//! motion gentle.
//!
//! A trajectory is fixed at construction and then sampled — no state, no
//! integration, no dependence on when it was last asked. Sampling the same time
//! twice gives the same answer bit for bit, which is what lets a tick that
//! overran its period simply ask for the time it actually is instead of
//! catching up.
//!
//! Shape: the same warp runs over independent clocks. The head group —
//! translation, rotation and body yaw, which the legs follow through the IK —
//! shares one duration and stays in phase throughout; each antenna, a free rotor
//! bolted to the same skull and sharing nothing else with it, gets one of its
//! own. Every clock starts together and each finishes on its own, so the head's
//! lift is not floored by however long an antenna sweep takes, and the two
//! antennas need not arrive together — a pair sweeping inboard on one clock
//! reaches the point where their arcs cross in phase, and their tips can meet
//! there instead of passing. Rotation follows the geodesic between the two
//! orientations; translation and the scalars are straight lines.
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
use crate::record;
use crate::snap::{DurationError, PoseSnapshotError, duration_from_nanos, duration_nanos};
use clockwork_rs::Duration as SlotDuration;

/// How normalised time maps onto normalised progress.
///
/// The vocabulary's own enum, declared in `motion/tick_state.clk`. Zero is the
/// shape a move gets when nobody said otherwise, which is why this one
/// numbering starts there rather than at one: a move with no warp is not a
/// thing, and minimum-jerk is what the head actually moves on, because the
/// linkage's own compliance rings on a velocity step.
pub use brenn_reachy__motion__tick_state_clk_rs::WarpKind;

/// Progress at normalised time `u`, which the caller has already placed in
/// `[0, 1]`.
///
/// `MinJerk` is `10u³ − 15u⁴ + 6u⁵`: the quintic with `s(0) = 0`, `s(1) = 1` and
/// both first and second derivatives zero at each end. `Linear` is `u`.
fn progress(warp: WarpKind, u: f64) -> f64 {
    match warp {
        WarpKind::MinJerk => u * u * u * (10.0 + u * (6.0 * u - 15.0)),
        WarpKind::Linear => u,
    }
}

/// A commanded move that cannot be shaped.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum TrajectoryError {
    /// A duration of zero. There is no path in no time, and the normalisation
    /// would divide by it.
    #[error("trajectory duration must be greater than zero")]
    NonPositiveDuration,
    /// A clock longer than the count a state holds it in. Refused at
    /// construction because the state a move runs in is the schema: a path
    /// nobody could sit through is refused rather than stored short.
    #[error("a trajectory clock is longer than a state can hold: {0}")]
    UnstorableClock(#[from] DurationError),
    /// Some endpoint carries a non-finite number. Refused here rather than
    /// carried to the envelope check as a violation on every tick of a move
    /// that was never going to arrive.
    #[error("trajectory endpoints must be finite")]
    NonFinite,
}

/// How long each independently clocked part of a move takes to cover its span.
///
/// Three clocks rather than one because the head and the two antennas are
/// mechanically independent: they share a skull and nothing else. Tying the
/// head's lift to an antenna's sweep is an implementation detail's grip on the
/// machine's behaviour, not a property of it, and tying the two antennas to each
/// other puts their tips at the point where their inboard arcs cross at the same
/// instant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MoveDurations {
    /// The head pose and the body yaw, which the six legs follow through the
    /// IK.
    pub head: Duration,
    /// Each antenna's own clock, right then left.
    pub antennas: [Duration; 2],
}

impl MoveDurations {
    /// Everything on one clock — what a caller with nothing to say about the
    /// antennas asks for.
    #[must_use]
    pub fn uniform(duration: Duration) -> Self {
        Self {
            head: duration,
            antennas: [duration; 2],
        }
    }

    /// `head` for the head group and `antennas` for both antennas.
    ///
    /// Test-only. A configuration resolves antenna clocks through
    /// [`Self::resolved`], and a caller with nothing to say about the antennas
    /// asks for [`Self::uniform`]; what this shape is good for is a case that
    /// wants one clock for the head and another for the pair, said in one line.
    #[cfg(test)]
    #[must_use]
    pub(crate) fn split(head: Duration, antennas: Duration) -> Self {
        Self {
            head,
            antennas: [antennas; 2],
        }
    }

    /// `head` for the head group, with each antenna taking the first clock a
    /// configuration states for it: its own, then `shared`, then the head's.
    ///
    /// The fallback chain lives here because more than one configuration
    /// resolves it — the bench's file and the daemon's — and two hand-written
    /// copies of it would be free to disagree about what a file naming one
    /// antenna key means for the other side. The clocks this returns are what
    /// the pair's stagger is, so a divergence here is a divergence in how the
    /// two tips pass each other.
    #[must_use]
    pub fn resolved(
        head: Duration,
        shared: Option<Duration>,
        sides: [Option<Duration>; 2],
    ) -> Self {
        Self {
            head,
            antennas: sides.map(|side| side.or(shared).unwrap_or(head)),
        }
    }

    /// The clock that finishes last, which is when the move is over.
    #[must_use]
    pub fn longest(self) -> Duration {
        self.head.max(self.antennas[0]).max(self.antennas[1])
    }
}

/// A shaped path between two command sets, sampled by elapsed time.
#[derive(Clone, Debug, PartialEq)]
pub struct Trajectory {
    start: JointTargets,
    target: JointTargets,
    head_s: f64,
    antennas_s: [f64; 2],
    warp: WarpKind,
    /// The geodesic from the start orientation to the target one, as a rotation
    /// vector in the *start's* frame. Precomputed: it costs a square root and
    /// an inverse trigonometric call, and the sample path runs every tick.
    rotvec_rel: Vector3<f64>,
}

impl Trajectory {
    /// Shape a move from `start` to `target`, each clock over its own duration.
    ///
    /// # Errors
    ///
    /// [`TrajectoryError::NonPositiveDuration`] if any clock's duration is
    /// zero, and [`TrajectoryError::NonFinite`] if either endpoint carries a
    /// non-finite number.
    pub fn new(
        start: &JointTargets,
        target: &JointTargets,
        durations: MoveDurations,
        warp: WarpKind,
    ) -> Result<Self, TrajectoryError> {
        if durations.head.is_zero() || durations.antennas.iter().any(Duration::is_zero) {
            return Err(TrajectoryError::NonPositiveDuration);
        }
        if !start.is_finite() || !target.is_finite() {
            return Err(TrajectoryError::NonFinite);
        }
        // Every clock is one a state can hold. Checked on the way in, before
        // anything converts them, so a length of time no state could hold is
        // refused rather than turned into seconds that no longer describe one.
        duration_nanos(durations.head)?;
        duration_nanos(durations.antennas[0])?;
        duration_nanos(durations.antennas[1])?;
        // The relative rotation's scaled axis carries an angle in [0, π]: of
        // the two ways round the same pair of orientations, the shorter one.
        let relative = start.head_pose_body.rotation.inverse() * target.head_pose_body.rotation;
        let path = Self {
            start: *start,
            target: *target,
            head_s: durations.head.as_secs_f64(),
            antennas_s: durations.antennas.map(|d| d.as_secs_f64()),
            warp,
            rotvec_rel: relative.scaled_axis(),
        };
        // And again on the clocks the path answers with, seconds and back
        // again, so that writing a path into the state it runs in has nothing
        // left to refuse: a clock that rounds up over the count's ceiling on
        // the way through is one this path cannot be stored at.
        let stored = path.durations();
        duration_nanos(stored.head)?;
        duration_nanos(stored.antennas[0])?;
        duration_nanos(stored.antennas[1])?;
        Ok(path)
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

    /// The path's per-clock durations.
    #[must_use]
    pub fn durations(&self) -> MoveDurations {
        MoveDurations {
            head: Duration::from_secs_f64(self.head_s),
            antennas: self.antennas_s.map(Duration::from_secs_f64),
        }
    }

    /// The shape this path was built with.
    #[must_use]
    pub fn warp(&self) -> WarpKind {
        self.warp
    }

    /// Whether `t` is at or past the end of every clock.
    #[must_use]
    pub fn done(&self, t: Duration) -> bool {
        let secs = t.as_secs_f64();
        secs >= self.head_s && self.antennas_s.iter().all(|end| secs >= *end)
    }

    /// The command set at elapsed time `t`, written into `out`.
    ///
    /// At or past a clock's duration what that clock drives is the target's own
    /// bits, so a move that ran to completion commands exactly what was asked
    /// for and a subsequent move chains from it without a step. Whatever
    /// finishes first sits at its target while the rest carries on.
    pub fn sample(&self, t: Duration, out: &mut JointTargets) {
        let secs = t.as_secs_f64();

        if secs >= self.head_s {
            out.head_pose_body = self.target.head_pose_body;
            out.body_yaw = self.target.body_yaw;
        } else {
            let s = self.progress(secs, self.head_s);
            let start_t = self.start.head_pose_body.translation.vector;
            let target_t = self.target.head_pose_body.translation.vector;
            out.head_pose_body = Isometry3::from_parts(
                Translation3::from(start_t + (target_t - start_t) * s),
                self.start.head_pose_body.rotation
                    * UnitQuaternion::from_scaled_axis(self.rotvec_rel * s),
            );
            out.body_yaw = lerp(self.start.body_yaw, self.target.body_yaw, s);
        }

        for side in 0..out.antennas.len() {
            let end = self.antennas_s[side];
            out.antennas[side] = if secs >= end {
                self.target.antennas[side]
            } else {
                let s = self.progress(secs, end);
                lerp(self.start.antennas[side], self.target.antennas[side], s)
            };
        }
    }

    /// Warped progress at `secs` on a clock of `duration_s`.
    ///
    /// The caller has already excluded the upper end, so the cap covers the
    /// rounding in the division alone, and it is on elapsed time, never on a
    /// commanded quantity.
    fn progress(&self, secs: f64, duration_s: f64) -> f64 {
        progress(self.warp, (secs / duration_s).clamp(0.0, 1.0))
    }
}

/// `a` at `s = 0`, exactly; `b` at `s = 1`, to within a rounding.
///
/// The exact endpoint that matters is the target's, and `sample` takes that
/// from the target's own bits rather than from this.
fn lerp(a: f64, b: f64, s: f64) -> f64 {
    a + (b - a) * s
}

/// A command set as the schema holds one: a head pose and three scalars.
pub use brenn_reachy__motion__targets_clk_rs::Targets;

/// A running move as the schema holds one, and the boundary form of it.
pub use brenn_reachy__motion__tick_state_clk_rs::{TrajectorySeed, TrajectorySeedWire};

/// Why a seed's numbers describe no running move.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum SeedError {
    /// A head pose that is not one.
    #[error("a command set in the seed is not one: {0}")]
    Pose(#[from] PoseSnapshotError),
    /// A clock that is not a length of time.
    #[error("a clock in the seed is not one: {0}")]
    Duration(#[from] DurationError),
    /// Numbers that are a command set each and no path between them.
    #[error("the seed is no path: {0}")]
    Path(#[from] TrajectoryError),
}

/// Write a command set into the fields the schema holds one in.
pub fn write_targets(out: &mut Targets, targets: &JointTargets) {
    record::write_pose(
        &mut out.head_pos,
        &mut out.head_quat,
        &targets.head_pose_body,
    );
    out.body_yaw = targets.body_yaw;
    out.antenna_right = targets.antennas[0];
    out.antenna_left = targets.antennas[1];
}

/// The command set those fields describe.
///
/// # Errors
///
/// [`PoseSnapshotError::NotARotation`] for a head pose that is not one — which
/// is what a field nothing wrote holds, the zeroed quaternion being no rotation.
pub fn targets_of(slot: &Targets) -> Result<JointTargets, PoseSnapshotError> {
    Ok(JointTargets {
        head_pose_body: record::read_pose(&slot.head_pos, &slot.head_quat)?,
        body_yaw: slot.body_yaw,
        antennas: [slot.antenna_right, slot.antenna_left],
    })
}

/// Write the path a move runs, as the four values it is rebuilt from.
///
/// Total: a trajectory refuses a clock a state cannot hold at construction, so
/// there is nothing left to refuse on the way out.
pub fn write_seed(out: &mut TrajectorySeed, path: &Trajectory) {
    let durations = path.durations();
    write_targets(&mut out.start, path.start());
    write_targets(&mut out.target, path.target());
    out.dur_head = stored(durations.head);
    out.dur_antenna_right = stored(durations.antennas[0]);
    out.dur_antenna_left = stored(durations.antennas[1]);
    out.warp = path.warp();
    out.present = true.into();
}

/// A clock as the count a seed holds it in.
///
/// Structurally total: [`Trajectory::new`] refuses a clock past what the count
/// reaches, so a path in hand has three that fit.
fn stored(clock: Duration) -> SlotDuration {
    SlotDuration::from_nanos(
        duration_nanos(clock).expect("a trajectory's clocks are ones a state holds"),
    )
}

/// Leave the fields holding no move at all.
///
/// The whole seed is written, not just the flag: a path from a move that ended
/// is still a path, and a reader that trusted the flag alone would carry it.
/// The declared initial state, taken from the schema rather than restated here:
/// a fresh message cleared through the generated route, swapped in over what the
/// slot held.
pub fn clear_seed(out: &mut TrajectorySeed) {
    let mut fresh = TrajectorySeedWire::new();
    core::mem::swap(out, fresh.clear_valid());
}

/// The move those fields describe, or `None` where none is running.
///
/// The path is rebuilt rather than stored sample by sample: the seed fully
/// determines it, so a move picked up out of a slot carries on along the same
/// path it was on.
///
/// # Errors
///
/// [`SeedError`], for an endpoint that is not a command set, a clock that is
/// not a length of time, or numbers that are no path.
pub fn read_seed(slot: &TrajectorySeed) -> Result<Option<Trajectory>, SeedError> {
    if !bool::from(slot.present) {
        return Ok(None);
    }
    let durations = MoveDurations {
        head: duration_from_nanos(slot.dur_head.as_nanos())?,
        antennas: [
            duration_from_nanos(slot.dur_antenna_right.as_nanos())?,
            duration_from_nanos(slot.dur_antenna_left.as_nanos())?,
        ],
    };
    Ok(Some(Trajectory::new(
        &targets_of(&slot.start)?,
        &targets_of(&slot.target)?,
        durations,
        slot.warp,
    )?))
}

#[cfg(test)]
mod tests {
    use super::*;
    use clockwork_rs::blob_as_bytes;
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
        let traj = Trajectory::new(&a, &b, MoveDurations::uniform(secs(2.0)), WarpKind::MinJerk)
            .expect("valid move");
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
        for warp in WarpKind::VARIANTS {
            let traj = Trajectory::new(&a, &b, MoveDurations::uniform(secs(2.0)), warp)
                .expect("valid move");
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
        let first = Trajectory::new(&a, &b, MoveDurations::uniform(secs(2.0)), WarpKind::MinJerk)
            .expect("valid move");
        let mut landed = JointTargets::default();
        first.sample(secs(2.0), &mut landed);

        let second = Trajectory::new(
            &landed,
            &a,
            MoveDurations::uniform(secs(1.5)),
            WarpKind::MinJerk,
        )
        .expect("valid move");
        let mut resumed = JointTargets::default();
        second.sample(Duration::ZERO, &mut resumed);
        assert_eq!(bits(&resumed), bits(&landed));
    }

    /// Any one clock at zero is refused, not just all of them: a head duration
    /// of zero with live antenna clocks would divide by it on the first sample
    /// of the group nobody was thinking about, and so would one antenna's.
    #[test]
    fn zero_duration_is_refused() {
        let (a, b) = (neutral(), stow());
        for durations in [
            MoveDurations::uniform(Duration::ZERO),
            MoveDurations::split(Duration::ZERO, secs(1.0)),
            MoveDurations::split(secs(1.0), Duration::ZERO),
            MoveDurations {
                head: secs(1.0),
                antennas: [Duration::ZERO, secs(1.0)],
            },
            MoveDurations {
                head: secs(1.0),
                antennas: [secs(1.0), Duration::ZERO],
            },
        ] {
            assert_eq!(
                Trajectory::new(&a, &b, durations, WarpKind::MinJerk),
                Err(TrajectoryError::NonPositiveDuration),
                "{durations:?}"
            );
        }
        // One nanosecond is a terrible move, but it is a well-defined one.
        assert!(
            Trajectory::new(
                &a,
                &b,
                MoveDurations::uniform(Duration::from_nanos(1)),
                WarpKind::MinJerk
            )
            .is_ok()
        );
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
                    Trajectory::new(
                        &bad,
                        &good,
                        MoveDurations::uniform(secs(1.0)),
                        WarpKind::MinJerk
                    ),
                    Err(TrajectoryError::NonFinite),
                    "bad start with {bad_value}"
                );
                assert_eq!(
                    Trajectory::new(
                        &good,
                        &bad,
                        MoveDurations::uniform(secs(1.0)),
                        WarpKind::MinJerk
                    ),
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
        assert_eq!(progress(WarpKind::MinJerk, 0.0), 0.0);
        assert!((progress(WarpKind::MinJerk, 1.0) - 1.0).abs() < 1e-15);
        assert!((progress(WarpKind::MinJerk, 0.5) - 0.5).abs() < 1e-15);

        let h = 1e-4;
        let d1 = |u: f64| {
            (progress(WarpKind::MinJerk, u + h) - progress(WarpKind::MinJerk, u - h)) / (2.0 * h)
        };
        let d2 = |u: f64| {
            (progress(WarpKind::MinJerk, u + h) - 2.0 * progress(WarpKind::MinJerk, u)
                + progress(WarpKind::MinJerk, u - h))
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
        for warp in WarpKind::VARIANTS {
            let mut previous = f64::NEG_INFINITY;
            for step in 0..=1000 {
                let u = f64::from(step) / 1000.0;
                let s = progress(warp, u);
                assert!((0.0..=1.0).contains(&s), "{warp:?} at {u}: {s}");
                assert!(s >= previous, "{warp:?} not monotone at {u}");
                previous = s;
            }
            assert!((progress(warp, 1.0) - 1.0).abs() < 1e-15);
        }
    }

    #[test]
    fn linear_warp_is_the_identity() {
        for step in 0..=100 {
            let u = f64::from(step) / 100.0;
            assert_eq!(progress(WarpKind::Linear, u), u);
        }
    }

    /// On one clock the warp drives all four components together: at any
    /// sample, the fraction of the translation covered, of the yaw, of each
    /// antenna and of the rotation angle are the same number. A phase
    /// difference within a group would be invisible at the endpoints and wrong
    /// everywhere between.
    #[test]
    fn one_warp_scalar_drives_every_component() {
        let (a, b) = (neutral(), stow());
        let traj = Trajectory::new(&a, &b, MoveDurations::uniform(secs(2.0)), WarpKind::MinJerk)
            .expect("valid move");
        let total_translation =
            (b.head_pose_body.translation.vector - a.head_pose_body.translation.vector).norm();
        let total_rotation =
            (a.head_pose_body.rotation.inverse() * b.head_pose_body.rotation).angle();
        assert!(total_translation > 0.04 && total_rotation > 0.4, "moves");

        let mut out = JointTargets::default();
        for step in 1..20 {
            let t = 2.0 * f64::from(step) / 20.0;
            traj.sample(secs(t), &mut out);
            let s = progress(WarpKind::MinJerk, t / 2.0);

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

        let traj = Trajectory::new(&a, &b, MoveDurations::uniform(secs(1.0)), WarpKind::Linear)
            .expect("valid move");
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

        let traj = Trajectory::new(&a, &b, MoveDurations::uniform(secs(1.0)), WarpKind::Linear)
            .expect("valid move");
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
        let traj = Trajectory::new(&a, &b, MoveDurations::uniform(secs(2.0)), WarpKind::MinJerk)
            .expect("valid move");

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
        let traj = Trajectory::new(
            &a,
            &b,
            MoveDurations::uniform(secs(1.25)),
            WarpKind::MinJerk,
        )
        .expect("valid move");
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
        let durations = MoveDurations {
            head: secs(3.5),
            antennas: [secs(1.25), secs(0.75)],
        };
        let traj = Trajectory::new(&a, &b, durations, WarpKind::Linear).expect("valid move");
        assert_eq!(bits(traj.start()), bits(&a));
        assert_eq!(bits(traj.target()), bits(&b));
        assert_eq!(traj.durations(), durations);
        assert_eq!(traj.durations().longest(), secs(3.5));
        assert_eq!(traj.warp(), WarpKind::Linear);
    }

    /// The clocks are independent: at any sample the head group's progress is
    /// read off its own duration and the antennas' off theirs. This is the whole
    /// point of the split — a lift that no longer waits on a sweep.
    #[test]
    fn group_clocks_run_independently() {
        let (a, b) = (neutral(), stow());
        let durations = MoveDurations::split(secs(1.0), secs(4.0));
        let traj = Trajectory::new(&a, &b, durations, WarpKind::Linear).expect("valid move");

        let mut out = JointTargets::default();
        for step in 1..10 {
            let t = f64::from(step) / 10.0;
            traj.sample(secs(t), &mut out);
            let head = (out.head_pose_body.translation.vector.z
                - a.head_pose_body.translation.vector.z)
                / (b.head_pose_body.translation.vector.z - a.head_pose_body.translation.vector.z);
            let antenna = (out.antennas[0] - a.antennas[0]) / (b.antennas[0] - a.antennas[0]);
            assert!((head - t).abs() < 1e-12, "head at {t}s: {head}");
            assert!(
                (antenna - t / 4.0).abs() < 1e-12,
                "antenna at {t}s: {antenna}"
            );
        }
    }

    /// The group that finishes first sits on its target while the other carries
    /// on, and the move is over when the longer clock is — so a completed move
    /// still commands both groups' endpoints exactly.
    #[test]
    fn the_short_group_waits_at_its_target() {
        let (a, b) = (neutral(), stow());
        let durations = MoveDurations::split(secs(1.0), secs(4.0));
        let traj = Trajectory::new(&a, &b, durations, WarpKind::MinJerk).expect("valid move");

        let mut out = JointTargets::default();
        for t in [secs(1.0), secs(2.0), secs(3.9)] {
            traj.sample(t, &mut out);
            assert!(!traj.done(t), "at {t:?}");
            assert_eq!(
                out.head_pose_body.translation.vector.z.to_bits(),
                b.head_pose_body.translation.vector.z.to_bits(),
                "head at {t:?}"
            );
            assert_eq!(out.body_yaw.to_bits(), b.body_yaw.to_bits(), "yaw at {t:?}");
            assert!(
                (out.antennas[0] - b.antennas[0]).abs() > 1e-6,
                "antennas still moving at {t:?}"
            );
        }

        traj.sample(secs(4.0), &mut out);
        assert!(traj.done(secs(4.0)));
        assert_eq!(bits(&out), bits(&b));
    }

    /// The antenna group may be the shorter one just as well, and then it is the
    /// head still travelling after the sweep has landed.
    #[test]
    fn either_group_may_be_the_shorter_one() {
        let (a, b) = (neutral(), stow());
        let durations = MoveDurations::split(secs(3.0), secs(0.5));
        let traj = Trajectory::new(&a, &b, durations, WarpKind::MinJerk).expect("valid move");

        let mut out = JointTargets::default();
        traj.sample(secs(1.0), &mut out);
        assert!(!traj.done(secs(1.0)));
        assert_eq!(out.antennas[0].to_bits(), b.antennas[0].to_bits());
        assert_eq!(out.antennas[1].to_bits(), b.antennas[1].to_bits());
        assert!(
            (out.head_pose_body.translation.vector.z - b.head_pose_body.translation.vector.z).abs()
                > 1e-6
        );
    }

    /// Each antenna reads its progress off its own clock. Two inboard arcs on
    /// one clock put both tips at the crossing point at the same instant; two
    /// clocks a tenth of a second apart put one there first, which is the whole
    /// reason each side carries a duration.
    #[test]
    fn each_antenna_runs_on_its_own_clock() {
        let a = neutral();
        let mut b = neutral();
        b.antennas = [2.0, -2.0];
        let durations = MoveDurations {
            head: secs(1.0),
            antennas: [secs(0.8), secs(0.7)],
        };
        let traj = Trajectory::new(&a, &b, durations, WarpKind::Linear).expect("valid move");

        let mut out = JointTargets::default();
        for step in 1..7 {
            let t = f64::from(step) / 10.0;
            traj.sample(secs(t), &mut out);
            let right = out.antennas[0] / b.antennas[0];
            let left = out.antennas[1] / b.antennas[1];
            assert!((right - t / 0.8).abs() < 1e-12, "right at {t}s: {right}");
            assert!((left - t / 0.7).abs() < 1e-12, "left at {t}s: {left}");
        }

        // The faster side lands and waits while the slower one is still
        // travelling, and the move is over when the last clock is.
        traj.sample(secs(0.7), &mut out);
        assert_eq!(out.antennas[1].to_bits(), b.antennas[1].to_bits());
        assert!((out.antennas[0] - b.antennas[0]).abs() > 1e-6);
        assert!(!traj.done(secs(0.7)));
        assert!(!traj.done(secs(0.8)), "the head clock still has 0.2 s");
        assert!(traj.done(secs(1.0)));
        traj.sample(secs(1.0), &mut out);
        assert_eq!(bits(&out), bits(&b));
    }

    /// The longest clock is the move's, whichever of the three it is.
    #[test]
    fn the_longest_clock_is_any_of_the_three() {
        assert_eq!(
            MoveDurations {
                head: secs(2.0),
                antennas: [secs(0.8), secs(0.7)],
            }
            .longest(),
            secs(2.0)
        );
        assert_eq!(
            MoveDurations {
                head: secs(0.5),
                antennas: [secs(0.8), secs(0.7)],
            }
            .longest(),
            secs(0.8)
        );
        assert_eq!(
            MoveDurations {
                head: secs(0.5),
                antennas: [secs(0.7), secs(0.9)],
            }
            .longest(),
            secs(0.9)
        );
        assert_eq!(MoveDurations::uniform(secs(1.5)).longest(), secs(1.5));
        assert_eq!(
            MoveDurations::split(secs(1.5), secs(0.5)),
            MoveDurations {
                head: secs(1.5),
                antennas: [secs(0.5); 2],
            }
        );
    }

    /// A clock longer than the count a state holds it in is refused at
    /// construction, on whichever of the three carries it.
    ///
    /// This refusal is what makes writing a path into a state total: the seed's
    /// three counts are written without a check, on the strength of a path in
    /// hand having three clocks that fit.
    #[test]
    fn a_clock_no_state_can_hold_is_refused() {
        // Past the 292 years a signed nanosecond count reaches, and past what
        // the same length of time rounds to going through seconds and back.
        let unstorable = Duration::from_secs(1 << 40);
        let ordinary = secs(2.0);
        let cases = [
            MoveDurations {
                head: unstorable,
                antennas: [ordinary; 2],
            },
            MoveDurations {
                head: ordinary,
                antennas: [unstorable, ordinary],
            },
            MoveDurations {
                head: ordinary,
                antennas: [ordinary, unstorable],
            },
        ];
        for durations in cases {
            assert_eq!(
                Trajectory::new(&neutral(), &stow(), durations, WarpKind::MinJerk),
                Err(TrajectoryError::UnstorableClock(DurationError::TooLong(
                    unstorable
                ))),
                "{durations:?}"
            );
        }

        // A clock the count holds comfortably builds, and survives the trip
        // through the slot it is stored in.
        let long = MoveDurations::uniform(Duration::from_secs(1 << 32));
        let path = Trajectory::new(&neutral(), &stow(), long, WarpKind::MinJerk)
            .expect("a clock the count holds");
        let mut slot = TrajectorySeedWire::new();
        write_seed(slot.clear_valid(), &path);
        assert_eq!(
            read_seed(slot.validate().expect("the seed validates")),
            Ok(Some(path))
        );
    }

    /// A cleared seed is a fresh one, not a live one with its flag down: a path
    /// from a move that ended is still a path, and a reader that trusted the
    /// flag alone would carry it.
    #[test]
    fn a_cleared_seed_keeps_no_trace_of_the_move_that_ended() {
        let mut slot = TrajectorySeedWire::new();
        let path = Trajectory::new(
            &neutral(),
            &stow(),
            MoveDurations {
                head: secs(2.0),
                antennas: [secs(1.0), secs(3.0)],
            },
            WarpKind::Linear,
        )
        .expect("the fixture shapes");
        write_seed(slot.clear_valid(), &path);
        assert_ne!(
            blob_as_bytes(&slot),
            blob_as_bytes(&TrajectorySeedWire::new()),
            "the fixture never wrote a move"
        );

        clear_seed(slot.validate_mut().expect("the seed validates"));
        assert_eq!(
            blob_as_bytes(&slot),
            blob_as_bytes(&TrajectorySeedWire::new()),
            "the ended move is still in the slot"
        );
        assert_eq!(
            read_seed(slot.validate().expect("a cleared seed validates")),
            Ok(None)
        );
    }

    /// Each side resolves its clock independently, and a side that states
    /// nothing takes the shared clock, then the head's.
    ///
    /// All four combinations, because this is the one function every
    /// configuration in this workspace resolves antenna clocks through: a side
    /// that quietly took the other side's key would collapse a staggered pair
    /// to one clock, which is the arrangement that puts the two tips at their
    /// crossing point together.
    #[test]
    fn each_antenna_takes_its_own_clock_then_the_shared_one_then_the_head() {
        let head = secs(2.0);
        assert_eq!(
            MoveDurations::resolved(head, None, [None, None]),
            MoveDurations::uniform(head)
        );
        assert_eq!(
            MoveDurations::resolved(head, Some(secs(1.5)), [None, None]),
            MoveDurations::split(head, secs(1.5))
        );
        assert_eq!(
            MoveDurations::resolved(head, Some(secs(1.5)), [Some(secs(0.7)), None]),
            MoveDurations {
                head,
                antennas: [secs(0.7), secs(1.5)],
            }
        );
        assert_eq!(
            MoveDurations::resolved(head, None, [Some(secs(0.7)), Some(secs(0.3))]),
            MoveDurations {
                head,
                antennas: [secs(0.7), secs(0.3)],
            }
        );
        assert_eq!(
            MoveDurations::resolved(head, Some(secs(1.5)), [None, Some(secs(0.3))]),
            MoveDurations {
                head,
                antennas: [secs(1.5), secs(0.3)],
            }
        );
    }

    /// A command set is carried into the fields the schema holds one in and
    /// comes back whole: the head pose bit for bit, the yaw and both antennas
    /// on their own fields.
    #[test]
    fn a_command_set_survives_the_fields_it_is_held_in() {
        let targets = JointTargets {
            head_pose_body: Isometry3::from_parts(
                nalgebra::Translation3::new(0.011, -0.023, 0.157),
                nalgebra::UnitQuaternion::from_scaled_axis(nalgebra::Vector3::new(
                    0.31, -0.17, 0.09,
                )),
            ),
            body_yaw: -0.41,
            antennas: [1.23, -1.24],
        };
        let mut wire = brenn_reachy__motion__targets_clk_rs::TargetsWire::new();
        let slot = wire.clear_valid();
        write_targets(slot, &targets);

        let back = targets_of(slot).expect("a written command set is one");
        assert_eq!(back.body_yaw, targets.body_yaw);
        assert_eq!(back.antennas, targets.antennas);
        assert_eq!(
            back.head_pose_body.translation.vector.as_slice(),
            targets.head_pose_body.translation.vector.as_slice()
        );
        assert_eq!(
            back.head_pose_body.rotation.as_ref().coords.as_slice(),
            targets.head_pose_body.rotation.as_ref().coords.as_slice()
        );
    }
}
