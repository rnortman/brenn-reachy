//! The flat forms the numbers a slot holds are paired with, for the values that
//! are not fields of their own.
//!
//! The tick's state is the schema (`motion/tick_state.clk`, read and written
//! through [`crate::tick`]); a sequencer's is its own. What is left over is the
//! handful of library values that no schema field holds directly — a rigid pose
//! written as a translation and a quaternion, a length of time as a signed
//! nanosecond count, and a solve failure as a code and two numbers. Each is
//! paired with its fields here, once, so a mapping is not written again per
//! host.
//!
//! The refusals are the doctrine: a quaternion that is not a rotation is
//! refused rather than normalised, a negative nanosecond count is not a length
//! of time, and a solve failure that names none is not one. What a slot nothing
//! wrote holds is exactly what those checks catch.
//!
//! The enumerated kinds need no flat form: [`MotionMode`],
//! [`TrackingSideKind`], [`WarpKind`](crate::traj::WarpKind) and
//! [`BusSourceKind`] are the vocabulary's own enums, held in a slot as
//! themselves and narrowed by the one validation at the boundary. A set of
//! servos likewise — it is the vocabulary's `JointFlags` wherever it is held.

use core::time::Duration;

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use reachy_kin::FkError;
use thiserror::Error;

/// Which side of a tracking run's anchor its goal lies on.
///
/// The vocabulary's own enum, declared in `motion/tick_state.clk`. Signed,
/// because the numbers are the direction: the side below the anchor is the
/// negative one, and [`TrackingSideKind::Unplaced`] is neither side — the goal
/// sits on the anchor, or is a number nobody can place, so there is no
/// direction to close in and no side to cross to.
pub use brenn_reachy__motion__tick_state_clk_rs::TrackingSideKind;

/// Why a code and two numbers name no solve failure.
///
/// Its own type rather than variants of the two refusals that wrap it, because
/// two flat forms carry a solve failure — a standing fault's
/// ([`crate::fault::FaultError`]) and a sequencer's verdict
/// ([`crate::verdict::VerdictError`]) — and each keeps its own refusal
/// vocabulary over the one arithmetic.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum FkFieldError {
    /// The three fields say no solve failed, where one is wanted.
    #[error("no solve failure is named")]
    NoSolveFailure,
    /// An iteration count that is not a count: not whole, not finite, or
    /// outside what one counts to.
    #[error("{0} is not a number of iterations")]
    NotACount(f64),
}

/// A solve failure as the code and two numbers a flat form holds it in.
///
/// The pairing is stated at [`FkFailureKind`]: which number is which depends on
/// which failure it is.
#[must_use]
pub fn fk_fields(cause: FkError) -> (FkFailureKind, f64, f64) {
    match cause {
        FkError::NoConvergence { iters, residual } => {
            (FkFailureKind::NoConvergence, f64::from(iters), residual)
        }
        FkError::WrongAssemblyMode { cone_deg, z } => {
            (FkFailureKind::WrongAssemblyMode, cone_deg, z)
        }
    }
}

/// The solve failure `code` and its two numbers describe.
///
/// # Errors
///
/// [`FkFieldError`]: a code saying no solve failed, or an iteration count that
/// is not one.
pub fn fk_cause(code: FkFailureKind, a: f64, b: f64) -> Result<FkError, FkFieldError> {
    match code {
        FkFailureKind::NotApplicable => Err(FkFieldError::NoSolveFailure),
        FkFailureKind::NoConvergence => Ok(FkError::NoConvergence {
            iters: whole_count(a)?,
            residual: b,
        }),
        FkFailureKind::WrongAssemblyMode => Ok(FkError::WrongAssemblyMode { cone_deg: a, z: b }),
    }
}

/// The count `value` holds, or why it holds none.
///
/// A [`u32`] count crosses into an [`f64`] field losslessly, so the way back is
/// exact — for the numbers that came from a count. Anything else in the field
/// was written by something that was not counting.
fn whole_count(value: f64) -> Result<u32, FkFieldError> {
    let whole = value.is_finite() && value.fract() == 0.0;
    if whole && (0.0..=f64::from(u32::MAX)).contains(&value) {
        #[expect(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "checked to be a whole number in range on the line above"
        )]
        Ok(value as u32)
    } else {
        Err(FkFieldError::NotACount(value))
    }
}

/// Which way a pose solve failed, or that none did.
///
/// The vocabulary's own enum, declared in `motion/fk.clk`. [`FkError`]'s two
/// variants each carry two numbers, which a flat form holds in the pair of
/// fields this keys — `fk_a` and `fk_b` on a fault's slot, on a verdict's:
/// [`FkFailureKind::NoConvergence`] pairs iterations with the largest residual,
/// [`FkFailureKind::WrongAssemblyMode`] tilt in degrees with height in metres,
/// and [`FkFailureKind::NotApplicable`] is what a flat form that is not about a
/// solve holds, its two numbers meaning nothing.
pub use brenn_reachy__motion__fk_clk_rs::FkFailureKind;

/// Which layer judged the bus not carrying, or that none did.
///
/// The vocabulary's own enum, declared in `motion/tick_state.clk`. Keys what a
/// standing fault's `bus_failure_kind` field holds: a
/// [`WireFailure`](crate::tick::WireFailure) under
/// [`BusSourceKind::Transaction`], a [`SeqFailureKind`] under
/// [`BusSourceKind::Sequence`], and nothing at all under
/// [`BusSourceKind::NotApplicable`], which is a fault that is not about the
/// bus. What restores from a sequence is
/// [`BusFailureSource::RestoredSequence`], never a fabricated
/// [`crate::seq::SeqError`].
pub use brenn_reachy__motion__tick_state_clk_rs::BusSourceKind;

/// What the tick is doing, as the slot holds it.
///
/// The vocabulary's own enum, declared in `motion/tick_state.clk`. The mode's
/// payloads are not in it: a moving mode's elapsed time is a duration
/// ([`duration_nanos`]) and a faulted one's fault is a `FaultSnap`, both of
/// which a slot holds in fields of their own keyed by this. `None` is what a
/// slot nothing wrote holds and is no state the tick is ever in.
pub use brenn_reachy__motion__tick_state_clk_rs::MotionMode;

/// A length of time that a slot's nanosecond field cannot hold, or that its
/// contents do not describe.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum DurationError {
    /// Longer than a signed nanosecond count reaches — around 292 years, which
    /// is no move and no tick gap.
    #[error("{0:?} is longer than a slot's nanoseconds reach")]
    TooLong(Duration),
    /// A negative count. Every duration this crate carries is an elapsed or a
    /// remaining time, and neither runs backwards, so a negative slot was
    /// written by something that was not measuring one.
    #[error("{0} nanoseconds is not a length of time")]
    Negative(i64),
}

/// `duration` as the whole nanoseconds a slot holds it in.
///
/// Nanoseconds signed rather than unsigned because that is what the schema
/// layer's duration field is; the sign is refused on the way back rather than
/// reinterpreted.
///
/// # Errors
///
/// [`DurationError::TooLong`] for a duration past what the count reaches.
pub fn duration_nanos(duration: Duration) -> Result<i64, DurationError> {
    i64::try_from(duration.as_nanos()).map_err(|_| DurationError::TooLong(duration))
}

/// The length of time `nanos` describes.
///
/// # Errors
///
/// [`DurationError::Negative`] for a count below zero.
pub fn duration_from_nanos(nanos: i64) -> Result<Duration, DurationError> {
    u64::try_from(nanos)
        .map(Duration::from_nanos)
        .map_err(|_| DurationError::Negative(nanos))
}

/// A rigid pose as the seven numbers a slot holds it in: a translation and a
/// rotation quaternion.
///
/// The quaternion is written out in full rather than as three of its four
/// components or as angles, so the trip back is the bits that went in and not a
/// reconstruction that has to choose a branch.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PoseSnapshot {
    /// Translation along x, metres.
    pub pos_x: f64,
    /// Translation along y, metres.
    pub pos_y: f64,
    /// Translation along z, metres.
    pub pos_z: f64,
    /// The rotation quaternion's real part.
    pub quat_w: f64,
    /// The rotation quaternion's i component.
    pub quat_x: f64,
    /// The rotation quaternion's j component.
    pub quat_y: f64,
    /// The rotation quaternion's k component.
    pub quat_z: f64,
}

impl PoseSnapshot {
    /// How far a slot's quaternion may stand from unit length and still be read
    /// as the rotation it was written from.
    ///
    /// Wide enough for the drift a chain of solves and interpolations leaves
    /// behind — which is parts in `1e15` — and far narrower than any number
    /// that is not a rotation at all.
    pub const UNIT_TOLERANCE: f64 = 1e-9;

    /// The pose these numbers describe.
    ///
    /// The quaternion is taken as written rather than normalised: a pose read
    /// back from a slot is the pose that was put in it, and rescaling a
    /// quaternion that is already unit would move the pose by the rounding.
    /// What that costs is the check below, and what it buys is a trip that
    /// changes nothing.
    ///
    /// # Errors
    ///
    /// [`PoseSnapshotError::NotARotation`] for a quaternion that is not unit
    /// within [`Self::UNIT_TOLERANCE`] — a non-finite component included, whose
    /// norm is no distance from one. Such a slot was written by something that
    /// was not holding a rotation, and reading it as one puts the machine's
    /// idea of where its head is wherever the arithmetic lands.
    pub fn to_isometry(&self) -> Result<Isometry3<f64>, PoseSnapshotError> {
        let quaternion = Quaternion::new(self.quat_w, self.quat_x, self.quat_y, self.quat_z);
        let norm = quaternion.norm();
        let off_unit = (norm - 1.0).abs();
        if off_unit.is_nan() || off_unit > Self::UNIT_TOLERANCE {
            return Err(PoseSnapshotError::NotARotation(norm));
        }
        Ok(Isometry3::from_parts(
            Translation3::new(self.pos_x, self.pos_y, self.pos_z),
            UnitQuaternion::new_unchecked(quaternion),
        ))
    }
}

impl From<&Isometry3<f64>> for PoseSnapshot {
    fn from(pose: &Isometry3<f64>) -> Self {
        let translation = pose.translation.vector;
        let rotation = pose.rotation.as_ref();
        Self {
            pos_x: translation.x,
            pos_y: translation.y,
            pos_z: translation.z,
            quat_w: rotation.w,
            quat_x: rotation.i,
            quat_y: rotation.j,
            quat_z: rotation.k,
        }
    }
}

/// Numbers that describe no pose.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum PoseSnapshotError {
    /// A quaternion of this length, which is not one.
    #[error("a rotation quaternion of length {0} is not a rotation")]
    NotARotation(f64),
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::seq::{SeqError, SeqFailureKind, failure};
    use crate::testutil::every_sequencer_failure;
    use crate::tick::WireFailure;
    use brenn_reachy__motion__faults_clk_rs::WireFailureWire;
    use brenn_reachy__motion__seq_clk_rs::SeqFailureKindWire;

    crate::vocab_numbering! {
        /// The transaction-failure numbering is the one written down, and
        /// nothing outside it names a shape. A number here is what a fault a
        /// slot carries between two builds is read back as.
        the_transaction_failure_numbering_is_the_one_written_down:
            WireFailure as WireFailureWire, past the end 7 {
            WireFailure::None => 0,
            WireFailure::Silent => 1,
            WireFailure::Corrupt => 2,
            WireFailure::Refused => 3,
            WireFailure::NotWritten => 4,
            WireFailure::Port => 5,
            WireFailure::Unsendable => 6,
        }
    }

    #[test]
    fn zero_names_no_bus_failure_shape() {
        assert_eq!(WireFailureWire(0).to_known(), Some(WireFailure::None));
        assert_eq!(SeqFailureKindWire(0).to_known(), Some(SeqFailureKind::None));
    }

    #[test]
    fn every_sequencer_failure_has_a_name_of_its_own() {
        let mut named: Vec<SeqFailureKind> = every_sequencer_failure()
            .iter()
            .map(SeqError::kind)
            .collect();
        named.sort_unstable();
        named.dedup();
        let all: Vec<SeqFailureKind> = failure::raised().collect();
        assert_eq!(named, all);
    }

    /// A pose with nothing round about it, so a trip that renormalised or
    /// reordered a component would show.
    fn a_pose() -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(0.031, -0.147, 0.2215),
            UnitQuaternion::from_euler_angles(0.21, -0.34, 1.02),
        )
    }

    #[test]
    fn a_pose_restores_bit_for_bit() {
        for pose in [
            a_pose(),
            Isometry3::identity(),
            reachy_kin::neutral_head_pose(),
            reachy_kin::stow_head_pose(),
        ] {
            let restored = PoseSnapshot::from(&pose)
                .to_isometry()
                .expect("a pose's own numbers describe it");
            assert_eq!(restored, pose);
            // Not merely equal: the same bits, which is what "not renormalised"
            // means and what a comparison of two poses one solve apart needs.
            assert_eq!(
                restored.rotation.as_ref().coords.as_slice(),
                pose.rotation.as_ref().coords.as_slice()
            );
        }
    }

    #[test]
    fn the_seven_numbers_are_the_pose_in_the_order_they_are_named() {
        let pose = a_pose();
        let snap = PoseSnapshot::from(&pose);
        assert_eq!(snap.pos_x, pose.translation.vector.x);
        assert_eq!(snap.pos_y, pose.translation.vector.y);
        assert_eq!(snap.pos_z, pose.translation.vector.z);
        assert_eq!(snap.quat_w, pose.rotation.as_ref().w);
        assert_eq!(snap.quat_x, pose.rotation.as_ref().i);
        assert_eq!(snap.quat_y, pose.rotation.as_ref().j);
        assert_eq!(snap.quat_z, pose.rotation.as_ref().k);
    }

    #[test]
    fn a_quaternion_that_is_not_a_rotation_is_refused() {
        let mut snap = PoseSnapshot::from(&a_pose());
        for scale in [0.0, 0.5, 2.0, 1.0 + 1e-6] {
            let scaled = PoseSnapshot {
                quat_w: snap.quat_w * scale,
                quat_x: snap.quat_x * scale,
                quat_y: snap.quat_y * scale,
                quat_z: snap.quat_z * scale,
                ..snap
            };
            assert!(
                matches!(
                    scaled.to_isometry(),
                    Err(PoseSnapshotError::NotARotation(norm)) if (norm - scale).abs() < 1e-9
                ),
                "a quaternion {scale} times as long as a rotation"
            );
        }

        // A number nobody can place is no distance from unit length either.
        snap.quat_x = f64::NAN;
        assert!(matches!(
            snap.to_isometry(),
            Err(PoseSnapshotError::NotARotation(norm)) if norm.is_nan()
        ));
        snap.quat_x = f64::INFINITY;
        assert!(matches!(
            snap.to_isometry(),
            Err(PoseSnapshotError::NotARotation(_))
        ));
    }

    #[test]
    fn the_drift_a_chain_of_solves_leaves_is_carried_not_corrected() {
        let mut snap = PoseSnapshot::from(&a_pose());
        snap.quat_w += 1e-12;
        let restored = snap
            .to_isometry()
            .expect("drift inside the tolerance is still a rotation");
        assert_eq!(PoseSnapshot::from(&restored), snap);
    }

    #[test]
    fn a_clock_a_slot_cannot_hold_is_refused_rather_than_shortened() {
        let too_long = Duration::from_nanos(u64::MAX);
        assert_eq!(
            duration_nanos(too_long),
            Err(DurationError::TooLong(too_long))
        );
        assert_eq!(duration_nanos(Duration::from_nanos(7)), Ok(7));
    }

    #[test]
    fn a_length_of_time_that_runs_backwards_is_refused() {
        assert_eq!(duration_from_nanos(-1), Err(DurationError::Negative(-1)));
        assert_eq!(duration_from_nanos(0), Ok(Duration::ZERO));
    }

    /// The zero a slot nothing wrote holds names no mode, and a number this
    /// build does not know names nothing at all.
    #[test]
    fn the_unwritten_zero_names_no_mode() {
        use brenn_reachy__motion__tick_state_clk_rs::MotionModeWire;

        assert_eq!(
            MotionModeWire::default().to_known(),
            Some(MotionMode::None),
            "an unwritten slot is no mode"
        );
        assert_eq!(MotionModeWire(4).to_known(), None);
    }
}
