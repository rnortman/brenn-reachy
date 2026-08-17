//! Everything [`MotionState`](crate::tick::MotionState) carries between
//! periods, as plain copyable data.
//!
//! The tick's state is private on purpose: nothing outside it may set a mode or
//! a last goal by hand, because a caller that could would be able to put the
//! machine in a state no sequence of ticks produces. That privacy costs nothing
//! to a host that keeps the state in a local variable for the life of a
//! session, and everything to a host that cannot — one whose per-execution
//! memory is a fixed-layout slot it is handed, and which gets no `&mut` that
//! outlives the call.
//!
//! [`MotionSnapshot`] is the whole of that state written out as fields, and
//! [`MotionState::snapshot`](crate::tick::MotionState::snapshot) and
//! [`MotionState::from_snapshot`](crate::tick::MotionState::from_snapshot) are
//! the only two doors it goes through. What that gives up is bounded: the
//! fields are numbers and sets a live read could have written in any
//! combination, and the pairings that are not — a mode and a path that
//! disagree, a path that cannot be rebuilt — are refused rather than restored,
//! as [`SnapshotError`] describes. The case that needs refusing is a slot
//! holding bytes nothing wrote.
//!
//! # What is not in here
//!
//! The session's [`FaultTimeline`](crate::timeline::FaultTimeline) is
//! deliberately absent: it is neither copyable nor bounded-size, and a host
//! that persists its state in a fixed-size slot has nowhere to put it. That
//! is not a loss: the reporting rule is that a classification happens exactly
//! once and travels as that value, and a host of this shape consumes
//! `report.fault`, `report.degraded` and `report.aborted` as they are raised
//! and sends each on as a message. A restored state therefore starts with an
//! empty timeline, and a host that wants the session record keeps it where
//! the session lives.
//!
//! # The round-trip law
//!
//! For any state the tick can reach, restoring a snapshot of it yields a state
//! that ticks identically: same [`TickOutputs`](crate::tick::TickOutputs), same
//! successor snapshot, for any inputs. `reachy-motion`'s tests assert exactly
//! that, at every period of replayed command-and-sample sequences covering
//! moves, retargets, aborts, masks, tracking runs, a recovery out of the
//! envelope, and every fault the tick raises from evidence.
//!
//! A field added to `MotionState` and forgotten here never reaches those tests:
//! both directions of the round trip destructure their source with no rest
//! pattern, so the omission is a compile error at the two places that have to
//! change. The tests are what catch a field carried across but carried wrongly.
//!
//! # The second boundary
//!
//! The law above is stated over [`MotionSnapshot`] values. A host that keeps
//! the snapshot in a fixed-layout slot crosses a second boundary — snapshot to
//! slot fields and back — and every member that is not already a scalar has a
//! crate-owned flat form for it here rather than a mapping written again per
//! host: [`FaultSnapshot`] for a standing fault, [`PoseSnapshot`] for the pose
//! seed and the commanded head pose, [`JointsSnapshot`] and [`TargetsSnapshot`]
//! for the two command vectors, [`DurationsSnapshot`] for a move's three
//! clocks, and a stated numbering with a refusing decoder for each of the
//! remaining kinds — [`ModeCode`], [`TrackingSide`],
//! [`Warp::as_u8`](crate::traj::Warp::as_u8),
//! [`JointSet::bits`](crate::joints::JointSet::bits), and
//! [`duration_nanos`] for a duration.
//!
//! The law extends through all of it: the crate's replayed sequences cross
//! every snapshot they reach through those forms and back, and the crossing
//! destructures [`MotionSnapshot`] with no rest pattern, so a member added and
//! not given a flat form is a compile error rather than a slot field nobody
//! wrote.

use core::time::Duration;

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use reachy_kin::FkError;
use thiserror::Error;

use crate::joints::{JointId, JointSet, JointTargets, JointVector};
use crate::seq::SeqErrorKind;
use crate::slot_enum::slot_enum;
use crate::tick::{BusFailureSource, Fault, Mode, WireFailure};
use crate::traj::{MoveDurations, TrajectoryError, Warp};

/// The whole of [`MotionState`](crate::tick::MotionState), as fields.
///
/// Every member is `Copy` and fixed-size, so the struct mirrors onto a
/// fixed-layout slot without a heap and without a length that could grow.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MotionSnapshot {
    /// What the machine is doing, including a latched fault's own value.
    pub mode: Mode,
    /// The running move, as the four values it is built from, or `None` when
    /// none is running.
    pub trajectory: Option<TrajectorySeed>,
    /// The `now` of the previous tick, or `None` before the first one.
    pub prev_now: Option<Duration>,
    /// The goals last emitted, or the ones arming pinned if none have been.
    pub last_goal: JointVector,
    /// The Cartesian mirror of `last_goal`, which the next move starts from.
    pub last_targets: JointTargets,
    /// The pose the next present-pose solve is seeded from.
    pub fk_seed: Isometry3<f64>,
    /// The present pose's smallest toggle margin, as of the last live read.
    pub present_min_margin: f64,
    /// How far the running move's start pose stood outside the envelope.
    pub start_excursion: ExcursionSnapshot,
    /// Consecutive periods without a live read.
    pub miss_count: u32,
    /// Consecutive live reads whose pose would not solve.
    pub pose_failures: u32,
    /// Each joint's open tracking run, in bus order, or `None` where none is
    /// open.
    pub tracking: [Option<TrackingStreakSnapshot>; JointId::COUNT],
    /// The joints out of service.
    pub masked: JointSet,
}

/// The four values a running move is rebuilt from.
///
/// These fully determine the path: a [`Trajectory`](crate::traj::Trajectory)
/// built from the same arguments produces the same samples regardless of
/// when it is constructed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrajectorySeed {
    /// The command set the path starts from.
    pub start: JointTargets,
    /// The command set it ends at.
    pub target: JointTargets,
    /// How long each independently clocked part takes.
    pub durations: MoveDurations,
    /// How normalised time maps onto normalised progress.
    pub warp: Warp,
}

/// How far a pose stands outside each envelope bound it can travel back inside
/// of, radians, zero on a bound it is within.
///
/// The baseline a move's later samples are excused against: a sample the
/// envelope refuses is still admitted when it stands no further out than the
/// pose the move began at. The clearance floor is not among these, because it
/// carries a baseline of its own taken from every live read.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ExcursionSnapshot {
    /// Per leg, how far the crank angle reaches past its travel window.
    pub window: [f64; 6],
    /// How far the body yaw's magnitude reaches past its cap.
    pub body_yaw: f64,
    /// How far the head-relative yaw's magnitude reaches past its cap.
    pub relative_yaw: f64,
    /// How far the head attitude reaches past the cone bound.
    pub cone: f64,
}

/// One joint's open run of live ticks past the tracking threshold.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackingStreakSnapshot {
    /// Where the joint was when this run last restarted, radians.
    pub anchor: f64,
    /// Which side of `anchor` the goal lay on then.
    pub side: TrackingSide,
    /// Live ticks since, this one included.
    pub count: u32,
}

slot_enum! {
    /// Which side of a tracking run's anchor its goal lies on.
    ///
    /// Public data with a stated integer numbering, for a host writing the run
    /// into a slot with an integer field. Signed, because the numbers are the
    /// direction: the side below the anchor is the negative one.
    pub enum TrackingSide: i8 {
        encode: as_i8;
        decode: from_i8;
        refusal: "A number outside the three is not silently read as \
                  `Unplaced`: a slot holding one was written by something that \
                  does not agree with this type about what a side is, and \
                  reading it as \"no direction\" would restore a run measuring \
                  in a direction the state was never in.";

        /// Neither side — the goal sits on the anchor, or is a number nobody
        /// can place. There is no direction to close in and no side to cross
        /// to.
        Unplaced = 0,
        /// Above the anchor: closing means rising.
        Above = 1,
        /// Below it: closing means falling.
        Below = -1,
    }
}

/// A snapshot that describes no state the tick could be in.
///
/// Only reachable from a snapshot that was assembled rather than taken —
/// [`MotionState::snapshot`](crate::tick::MotionState::snapshot) cannot produce one — which in practice means a
/// slot holding bytes that no [`MotionState::snapshot`](crate::tick::MotionState::snapshot) ever wrote. Refused
/// here rather than restored, because each of these would leave the tick
/// holding a contradiction it can only resolve by dropping something.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum SnapshotError {
    /// The move's seed does not describe a path.
    #[error("the snapshot's trajectory cannot be rebuilt: {0}")]
    Trajectory(#[from] TrajectoryError),
    /// [`Mode::Moving`] with no trajectory to sample. A live tick would find
    /// nothing to advance and drop to holding; a restore says so instead.
    #[error("the snapshot is moving with no trajectory")]
    MovingWithoutTrajectory,
    /// [`Mode::Holding`] with a trajectory. Every way the tick reaches holding
    /// — a hold, an abort, a tracked setpoint, a completed move, a
    /// non-latching fault — drops the path in the same statement, so a holding
    /// state carrying one is a state no sequence of ticks produces.
    ///
    /// Refused rather than carried, because a path in a state nothing is
    /// sampling reads to anything that looks at it as a move in flight.
    ///
    /// [`Mode::Faulted`] with a trajectory is *not* an error: a latching fault
    /// leaves the path in place as it stops commanding, so a faulted state
    /// carrying a trajectory nobody will sample again is a state the tick
    /// reaches.
    #[error("the snapshot is holding with a trajectory")]
    HoldingWithTrajectory,
}

/// A [`Fault`] as the numbers a fixed-layout slot holds.
///
/// [`Fault`] is `Copy`, but two of its variants carry another error inside
/// them — a solver's cause, a sequencer's whole verdict — and a nested error is
/// not a set of scalar fields a slot has room for. What has to survive that
/// trip is settled by the reporting rule: a classification happens exactly
/// once and travels as that value, so the raise-time [`Fault`] is the report of
/// record and a restored one is not a report at all. It exists to keep the
/// machine parked under the same response and latch, to let a host tell a
/// standing fault from a fresh one, and to name itself truthfully if it is
/// surfaced again.
///
/// So this form keeps **identity and actionable evidence** — the slug, the
/// joint, the servo, the hardware-error byte, the counters and the magnitudes —
/// and drops **diagnostic error-chain structure**, which travelled once at
/// raise time and lives in the log.
///
/// # Which fields mean anything
///
/// Every field is present for every fault, because a slot has no room for a
/// variant. Which of them the fault actually said is [`Self::code`], and each
/// field below names the codes it belongs to; the rest carry
/// [`FaultSnapshot::NO_JOINT`] or zero. Nothing infers a fault from them, so an
/// unused zero says only "not part of this fault".
///
/// # The round trip
///
/// [`Fault`] → [`FaultSnapshot`] is total, and [`FaultSnapshot::to_fault`]
/// refuses any snapshot whose numbers name no fault it could have come from.
/// Six of the eight faults restore to a value equal to the original *when the
/// fault names a joint the bus has a row for*: a fault about a leg index no
/// machine carries flattens to [`FaultSnapshot::NO_JOINT`] — which is how the
/// outward direction stays total — and is then refused on the way back rather
/// than restored naming some other servo. The seventh,
/// [`Fault::TorqueOffUnconfirmed`], is already flat and so does the
/// eighth when its source is a transaction; a [`Fault::BusFailure`] whose
/// source was a *sequencer's* verdict restores as
/// [`BusFailureSource::RestoredSequence`] instead, which agrees with the
/// original on [`Fault::slug`], [`Fault::response`], [`Fault::latches`], the
/// servo and the failure's name, and drops the step context and the payload
/// the slot never held.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FaultSnapshot {
    /// Which fault this is. The slug, as a number.
    pub code: FaultCode,
    /// The joint concerned in bus order, or [`Self::NO_JOINT`] where the fault
    /// names none. Meaningful for the two obstructions and the two servo
    /// faults.
    pub joint: u8,
    /// The servo's bus ID, or zero where the fault names no servo. Meaningful
    /// for the two servo faults, [`FaultCode::TorqueOffUnconfirmed`], and
    /// [`FaultCode::BusFailure`].
    pub servo_id: u8,
    /// The servo's hardware-error byte as read, or zero. Meaningful for the two
    /// servo faults.
    pub error_bits: u8,
    /// How many consecutive periods the evidence ran for, or zero. Meaningful
    /// for [`FaultCode::PositionFeedbackLost`] (missed reads) and
    /// [`FaultCode::MeasuredPoseInvalid`] (failed solves).
    pub count: u32,
    /// How far the joint stood from its goal, radians, or zero. Meaningful for
    /// the two obstructions.
    pub error: f64,
    /// Which way the pose solve failed, or [`FkFailureCode::NotApplicable`].
    /// Meaningful for [`FaultCode::MeasuredPoseInvalid`].
    pub fk_code: FkFailureCode,
    /// The solve failure's first number: iterations taken for
    /// [`FkFailureCode::NoConvergence`], head tilt in degrees for
    /// [`FkFailureCode::WrongAssemblyMode`].
    pub fk_a: f64,
    /// The solve failure's second number: the largest residual in metres for
    /// [`FkFailureCode::NoConvergence`], head height in metres for
    /// [`FkFailureCode::WrongAssemblyMode`].
    pub fk_b: f64,
    /// Which layer judged the bus not carrying, or
    /// [`BusSourceCode::NotApplicable`]. Meaningful for
    /// [`FaultCode::BusFailure`].
    pub bus_source: BusSourceCode,
    /// What that layer called the failure, as the number its own type states:
    /// a [`WireFailure`] for [`BusSourceCode::Transaction`], a
    /// [`SeqErrorKind`] for [`BusSourceCode::Sequence`]. Zero where
    /// [`Self::bus_source`] is not applicable — the slot is keyed by the
    /// source, so it is never a number without an owner.
    pub bus_failure_kind: u8,
}

impl FaultSnapshot {
    /// [`Self::joint`] where the fault names no joint.
    ///
    /// Outside the nine bus rows on purpose: a fault that names no joint must
    /// not read as one that names the body yaw.
    pub const NO_JOINT: u8 = 255;

    /// The joint this fault names, or `None` where it names none or names a
    /// row no joint has.
    #[must_use]
    pub fn joint_id(&self) -> Option<JointId> {
        JointId::from_index(usize::from(self.joint))
    }

    /// The fault these numbers describe, or why they describe none.
    ///
    /// # Errors
    ///
    /// [`FaultSnapshotError`], one variant per way a slot's numbers can fail to
    /// name a fault: a joint row that is no joint, a discriminant this build
    /// does not know, a solver's iteration count that is not a count.
    pub fn to_fault(&self) -> Result<Fault, FaultSnapshotError> {
        let joint = || {
            self.joint_id()
                .ok_or(FaultSnapshotError::NoSuchJoint(self.joint))
        };
        Ok(match self.code {
            FaultCode::AntennaObstructed => Fault::AntennaObstructed {
                joint: joint()?,
                error: self.error,
            },
            FaultCode::AntennaServoFault => Fault::AntennaServoFault {
                joint: joint()?,
                id: self.servo_id,
                bits: self.error_bits,
            },
            FaultCode::HeadObstructed => Fault::HeadObstructed {
                joint: joint()?,
                error: self.error,
            },
            FaultCode::HeadServoFault => Fault::HeadServoFault {
                joint: joint()?,
                id: self.servo_id,
                bits: self.error_bits,
            },
            FaultCode::PositionFeedbackLost => Fault::PositionFeedbackLost { misses: self.count },
            FaultCode::MeasuredPoseInvalid => Fault::MeasuredPoseInvalid {
                failures: self.count,
                source: self.fk_error()?,
            },
            FaultCode::BusFailure => Fault::BusFailure {
                source: self.bus_source()?,
            },
            FaultCode::TorqueOffUnconfirmed => Fault::TorqueOffUnconfirmed { id: self.servo_id },
        })
    }

    /// The solve failure [`Self::fk_code`] and its two numbers describe.
    fn fk_error(&self) -> Result<FkError, FaultSnapshotError> {
        match self.fk_code {
            FkFailureCode::NotApplicable => Err(FaultSnapshotError::NoSolveFailure),
            FkFailureCode::NoConvergence => Ok(FkError::NoConvergence {
                iters: whole_count(self.fk_a)?,
                residual: self.fk_b,
            }),
            FkFailureCode::WrongAssemblyMode => Ok(FkError::WrongAssemblyMode {
                cone_deg: self.fk_a,
                z: self.fk_b,
            }),
        }
    }

    /// The bus failure [`Self::bus_source`] and [`Self::bus_failure_kind`]
    /// describe, as far as a slot can carry one.
    fn bus_source(&self) -> Result<BusFailureSource, FaultSnapshotError> {
        match self.bus_source {
            BusSourceCode::NotApplicable => Err(FaultSnapshotError::NoBusSource),
            BusSourceCode::Transaction => Ok(BusFailureSource::Transaction {
                id: self.servo_id,
                kind: WireFailure::from_u8(self.bus_failure_kind)
                    .ok_or(FaultSnapshotError::NoSuchWireFailure(self.bus_failure_kind))?,
            }),
            BusSourceCode::Sequence => Ok(BusFailureSource::RestoredSequence {
                id: self.servo_id,
                kind: SeqErrorKind::from_u8(self.bus_failure_kind)
                    .ok_or(FaultSnapshotError::NoSuchSeqError(self.bus_failure_kind))?,
            }),
        }
    }

    /// A snapshot with every field at its "not part of this fault" value.
    fn blank(code: FaultCode) -> Self {
        Self {
            code,
            joint: Self::NO_JOINT,
            servo_id: 0,
            error_bits: 0,
            count: 0,
            error: 0.0,
            fk_code: FkFailureCode::NotApplicable,
            fk_a: 0.0,
            fk_b: 0.0,
            bus_source: BusSourceCode::NotApplicable,
            bus_failure_kind: 0,
        }
    }
}

/// The count `value` holds, or why it holds none.
///
/// A [`u32`] count crosses into an [`f64`] field losslessly, so the way back is
/// exact — for the numbers that came from a count. Anything else in the field
/// was written by something that was not counting.
fn whole_count(value: f64) -> Result<u32, FaultSnapshotError> {
    let whole = value.is_finite() && value.fract() == 0.0;
    if whole && (0.0..=f64::from(u32::MAX)).contains(&value) {
        #[expect(
            clippy::cast_possible_truncation,
            clippy::cast_sign_loss,
            reason = "checked to be a whole number in range on the line above"
        )]
        Ok(value as u32)
    } else {
        Err(FaultSnapshotError::NotACount(value))
    }
}

impl From<&Fault> for FaultSnapshot {
    fn from(fault: &Fault) -> Self {
        match *fault {
            Fault::AntennaObstructed { joint, error } => Self {
                joint: joint_row(joint),
                error,
                ..Self::blank(FaultCode::AntennaObstructed)
            },
            Fault::AntennaServoFault { joint, id, bits } => Self {
                joint: joint_row(joint),
                servo_id: id,
                error_bits: bits,
                ..Self::blank(FaultCode::AntennaServoFault)
            },
            Fault::HeadObstructed { joint, error } => Self {
                joint: joint_row(joint),
                error,
                ..Self::blank(FaultCode::HeadObstructed)
            },
            Fault::HeadServoFault { joint, id, bits } => Self {
                joint: joint_row(joint),
                servo_id: id,
                error_bits: bits,
                ..Self::blank(FaultCode::HeadServoFault)
            },
            Fault::PositionFeedbackLost { misses } => Self {
                count: misses,
                ..Self::blank(FaultCode::PositionFeedbackLost)
            },
            Fault::MeasuredPoseInvalid { failures, source } => {
                let (fk_code, fk_a, fk_b) = match source {
                    FkError::NoConvergence { iters, residual } => {
                        (FkFailureCode::NoConvergence, f64::from(iters), residual)
                    }
                    FkError::WrongAssemblyMode { cone_deg, z } => {
                        (FkFailureCode::WrongAssemblyMode, cone_deg, z)
                    }
                };
                Self {
                    count: failures,
                    fk_code,
                    fk_a,
                    fk_b,
                    ..Self::blank(FaultCode::MeasuredPoseInvalid)
                }
            }
            Fault::BusFailure { source } => {
                let (bus_source, servo_id, bus_failure_kind) = match source {
                    BusFailureSource::Transaction { id, kind } => {
                        (BusSourceCode::Transaction, id, kind.as_u8())
                    }
                    BusFailureSource::Sequence(error) => (
                        BusSourceCode::Sequence,
                        error.context().id,
                        error.kind().as_u8(),
                    ),
                    BusFailureSource::RestoredSequence { id, kind } => {
                        (BusSourceCode::Sequence, id, kind.as_u8())
                    }
                };
                Self {
                    servo_id,
                    bus_source,
                    bus_failure_kind,
                    ..Self::blank(FaultCode::BusFailure)
                }
            }
            Fault::TorqueOffUnconfirmed { id } => Self {
                servo_id: id,
                ..Self::blank(FaultCode::TorqueOffUnconfirmed)
            },
        }
    }
}

/// `joint`'s bus row, or [`FaultSnapshot::NO_JOINT`] for a leg index past the
/// sixth — a joint no machine carries, which no fault can be about.
fn joint_row(joint: JointId) -> u8 {
    joint.index().map_or(FaultSnapshot::NO_JOINT, |index| {
        u8::try_from(index).unwrap_or(FaultSnapshot::NO_JOINT)
    })
}

slot_enum! {
    /// Which fault a [`FaultSnapshot`] is.
    ///
    /// The numbering is the fault vocabulary as integers. A new fault is a new
    /// number here and a classification decision at [`Fault::response`], as
    /// ever — and a new value on any schema enum that mirrors this one, which
    /// its host binds to these numbers by test.
    pub enum FaultCode: u8 {
        encode: as_u8;
        decode: from_u8;
        refusal: "Zero included, which is what an unwritten slot holds and is \
                  no fault at all.";

        /// [`Fault::AntennaObstructed`], slug `antenna_obstructed`.
        AntennaObstructed = 1,
        /// [`Fault::AntennaServoFault`], slug `antenna_servo_fault`.
        AntennaServoFault = 2,
        /// [`Fault::HeadObstructed`], slug `head_obstructed`.
        HeadObstructed = 3,
        /// [`Fault::HeadServoFault`], slug `head_servo_fault`.
        HeadServoFault = 4,
        /// [`Fault::PositionFeedbackLost`], slug `position_feedback_lost`.
        PositionFeedbackLost = 5,
        /// [`Fault::MeasuredPoseInvalid`], slug `measured_pose_invalid`.
        MeasuredPoseInvalid = 6,
        /// [`Fault::BusFailure`], slug `bus_failure`.
        BusFailure = 7,
        /// [`Fault::TorqueOffUnconfirmed`], slug `torque_off_unconfirmed`.
        TorqueOffUnconfirmed = 8,
    }
}

slot_enum! {
    /// Which way a pose solve failed, as a number, or that none did.
    ///
    /// Mirrors [`FkError`], whose two variants each carry two numbers that
    /// [`FaultSnapshot`] holds in [`FaultSnapshot::fk_a`] and
    /// [`FaultSnapshot::fk_b`].
    pub enum FkFailureCode: u8 {
        encode: as_u8;
        decode: from_u8;
        refusal: "A number outside the three names no way a solve has of \
                  failing.";

        /// The fault is not about a pose solve, and the two numbers mean
        /// nothing.
        NotApplicable = 0,
        /// [`FkError::NoConvergence`]: iterations, then the largest residual.
        NoConvergence = 1,
        /// [`FkError::WrongAssemblyMode`]: tilt in degrees, then height in
        /// metres.
        WrongAssemblyMode = 2,
    }
}

slot_enum! {
    /// Which layer judged the bus not carrying, as a number, or that none did.
    ///
    /// Mirrors [`BusFailureSource`], and keys what
    /// [`FaultSnapshot::bus_failure_kind`] holds.
    pub enum BusSourceCode: u8 {
        encode: as_u8;
        decode: from_u8;
        refusal: "A number outside the three names no layer, and the \
                  failure-kind slot beside it would have nothing keying it.";

        /// The fault is not about the bus, and the failure-kind slot means
        /// nothing.
        NotApplicable = 0,
        /// The move loop's own transaction: the slot holds a [`WireFailure`].
        Transaction = 1,
        /// A sequencer's verdict: the slot holds a [`SeqErrorKind`]. What
        /// restores from it is [`BusFailureSource::RestoredSequence`], never a
        /// fabricated [`crate::seq::SeqError`].
        Sequence = 2,
    }
}

/// Numbers that name no fault.
///
/// Only reachable from a snapshot that was assembled rather than taken — every
/// [`FaultSnapshot`] built from a [`Fault`] restores — which in practice means
/// a slot holding bytes that no fault ever wrote. Refused rather than repaired,
/// because every repair available here would be a report of something nobody
/// observed.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum FaultSnapshotError {
    /// A fault that names a joint, naming a bus row that is not one.
    #[error("{0} is not a bus row, and this fault is about a joint")]
    NoSuchJoint(u8),
    /// A pose-solve fault whose solve did not fail.
    #[error("the pose is invalid, but no solve failure is named")]
    NoSolveFailure,
    /// An iteration count that is not a count: not whole, not finite, or
    /// outside what one counts to.
    #[error("{0} is not a number of iterations")]
    NotACount(f64),
    /// A bus fault with no layer that judged it.
    #[error("the bus is not carrying, but no layer is named as having judged it")]
    NoBusSource,
    /// A transaction failure this build does not know the shape of.
    #[error("{0} is not a wire failure this build knows")]
    NoSuchWireFailure(u8),
    /// A sequencer failure this build does not know the name of.
    #[error("{0} is not a sequencer failure this build knows")]
    NoSuchSeqError(u8),
}

slot_enum! {
    /// Which of [`Mode`]'s three states a snapshot is in, as a number.
    ///
    /// The mode's payloads are not in here: a moving mode's elapsed time is a
    /// duration ([`duration_nanos`]) and a faulted one's fault is a
    /// [`FaultSnapshot`], both of which a slot holds in fields of their own
    /// keyed by this number.
    ///
    /// The numbering starts at one, so a slot nothing wrote names no mode
    /// rather than naming the one the machine happens to be safe in.
    pub enum ModeCode: u8 {
        encode: as_u8;
        decode: from_u8;
        refusal: "Zero included: a slot nothing wrote names no mode.";

        /// [`Mode::Holding`].
        Holding = 1,
        /// [`Mode::Moving`], whose elapsed time travels beside this.
        Moving = 2,
        /// [`Mode::Faulted`], whose fault travels beside this as a
        /// [`FaultSnapshot`].
        Faulted = 3,
    }
}

impl ModeCode {
    /// Which of these `mode` is.
    #[must_use]
    pub fn of(mode: Mode) -> Self {
        match mode {
            Mode::Holding => Self::Holding,
            Mode::Moving { .. } => Self::Moving,
            Mode::Faulted(_) => Self::Faulted,
        }
    }
}

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

/// A [`JointVector`] as one named field per servo, in [`JointId::ALL`]'s bus
/// order.
///
/// The same nine numbers, with the array indices spelled out: a slot's fields
/// are named, and a host mapping `legs[3]` onto the fourth of them by hand is a
/// place to put the fifth by mistake.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct JointsSnapshot {
    /// Body yaw, radians. Bus row 0.
    pub body_yaw: f64,
    /// The first crank, radians. Bus row 1.
    pub leg_0: f64,
    /// The second crank, radians. Bus row 2.
    pub leg_1: f64,
    /// The third crank, radians. Bus row 3.
    pub leg_2: f64,
    /// The fourth crank, radians. Bus row 4.
    pub leg_3: f64,
    /// The fifth crank, radians. Bus row 5.
    pub leg_4: f64,
    /// The sixth crank, radians. Bus row 6.
    pub leg_5: f64,
    /// The right antenna, radians. Bus row 7.
    pub antenna_right: f64,
    /// The left antenna, radians. Bus row 8.
    pub antenna_left: f64,
}

impl JointsSnapshot {
    /// The nine angles these fields hold, in bus order.
    ///
    /// Total, and its own inverse: these are numbers, and every combination of
    /// them is one some read could have produced.
    #[must_use]
    pub fn to_vector(&self) -> JointVector {
        JointVector {
            body_yaw: self.body_yaw,
            legs: [
                self.leg_0, self.leg_1, self.leg_2, self.leg_3, self.leg_4, self.leg_5,
            ],
            antennas: [self.antenna_right, self.antenna_left],
        }
    }
}

impl From<&JointVector> for JointsSnapshot {
    fn from(joints: &JointVector) -> Self {
        let JointVector {
            body_yaw,
            legs,
            antennas,
        } = *joints;
        Self {
            body_yaw,
            leg_0: legs[0],
            leg_1: legs[1],
            leg_2: legs[2],
            leg_3: legs[3],
            leg_4: legs[4],
            leg_5: legs[5],
            antenna_right: antennas[0],
            antenna_left: antennas[1],
        }
    }
}

/// A [`JointTargets`] as the numbers a slot holds it in: a head pose and three
/// angles.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TargetsSnapshot {
    /// The head pose relative to the body at the commanded yaw.
    pub head_pose: PoseSnapshot,
    /// Body yaw, radians.
    pub body_yaw: f64,
    /// The right antenna's angle, radians.
    pub antenna_right: f64,
    /// The left antenna's angle, radians.
    pub antenna_left: f64,
}

impl TargetsSnapshot {
    /// The command set these numbers describe.
    ///
    /// # Errors
    ///
    /// [`PoseSnapshotError`], for a head pose that is not one.
    pub fn to_targets(&self) -> Result<JointTargets, PoseSnapshotError> {
        Ok(JointTargets {
            head_pose_body: self.head_pose.to_isometry()?,
            body_yaw: self.body_yaw,
            antennas: [self.antenna_right, self.antenna_left],
        })
    }
}

impl From<&JointTargets> for TargetsSnapshot {
    fn from(targets: &JointTargets) -> Self {
        let JointTargets {
            head_pose_body,
            body_yaw,
            antennas,
        } = targets;
        Self {
            head_pose: PoseSnapshot::from(head_pose_body),
            body_yaw: *body_yaw,
            antenna_right: antennas[0],
            antenna_left: antennas[1],
        }
    }
}

/// A move's three clocks as the whole nanoseconds a slot holds them in.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DurationsSnapshot {
    /// The head group's clock, nanoseconds.
    pub head_ns: i64,
    /// The right antenna's clock, nanoseconds.
    pub antenna_right_ns: i64,
    /// The left antenna's clock, nanoseconds.
    pub antenna_left_ns: i64,
}

impl DurationsSnapshot {
    /// The three clocks these counts describe.
    ///
    /// # Errors
    ///
    /// [`DurationError::Negative`] for a count below zero, on any of the three.
    pub fn to_durations(&self) -> Result<MoveDurations, DurationError> {
        Ok(MoveDurations {
            head: duration_from_nanos(self.head_ns)?,
            antennas: [
                duration_from_nanos(self.antenna_right_ns)?,
                duration_from_nanos(self.antenna_left_ns)?,
            ],
        })
    }
}

impl TryFrom<MoveDurations> for DurationsSnapshot {
    type Error = DurationError;

    /// Fallible in this direction alone: a [`Duration`] reaches further than a
    /// nanosecond count does, and truncating one to fit would shorten a move
    /// nobody asked to shorten.
    fn try_from(durations: MoveDurations) -> Result<Self, Self::Error> {
        let MoveDurations { head, antennas } = durations;
        Ok(Self {
            head_ns: duration_nanos(head)?,
            antenna_right_ns: duration_nanos(antennas[0])?,
            antenna_left_ns: duration_nanos(antennas[1])?,
        })
    }
}

/// `snap` crossed through every flat form a fixed-layout slot would hold it in,
/// and read back.
///
/// The second boundary, exercised as one step: each member goes out to the
/// numbers a slot holds and comes back from them, and what returns is what a
/// host that stored the snapshot and restored it would be holding. Every state
/// the replayed sequences reach goes through here, and the result is compared
/// with the original.
///
/// Destructured with no rest pattern, in both [`MotionSnapshot`] and
/// [`TrajectorySeed`], so a member added without a flat form is a compile error
/// here rather than a field that silently arrives at zero on the far side.
///
/// Panics on a refusal, which is the assertion: a snapshot the tick reached
/// crosses, and a refusal means it did not.
#[cfg(test)]
pub(crate) fn crossed(snap: &MotionSnapshot) -> MotionSnapshot {
    let MotionSnapshot {
        mode,
        trajectory,
        prev_now,
        last_goal,
        last_targets,
        fk_seed,
        present_min_margin,
        start_excursion,
        miss_count,
        pose_failures,
        tracking,
        masked,
    } = snap;

    let code = ModeCode::of(*mode).as_u8();
    let elapsed_ns = match mode {
        Mode::Moving { elapsed } => duration_nanos(*elapsed).expect("a move's elapsed time fits"),
        Mode::Holding | Mode::Faulted(_) => 0,
    };
    let fault = match mode {
        Mode::Faulted(fault) => Some(FaultSnapshot::from(fault)),
        Mode::Holding | Mode::Moving { .. } => None,
    };
    let mode = match ModeCode::from_u8(code).expect("a mode's own code names it") {
        ModeCode::Holding => Mode::Holding,
        ModeCode::Moving => Mode::Moving {
            elapsed: duration_from_nanos(elapsed_ns).expect("an elapsed time is not negative"),
        },
        ModeCode::Faulted => Mode::Faulted(
            fault
                .expect("a faulted mode carries a fault")
                .to_fault()
                .expect("a fault the tick raised names itself"),
        ),
    };

    let trajectory = trajectory.map(|seed| {
        let TrajectorySeed {
            start,
            target,
            durations,
            warp,
        } = seed;
        let start = TargetsSnapshot::from(&start);
        let target = TargetsSnapshot::from(&target);
        let durations =
            DurationsSnapshot::try_from(durations).expect("a move's clocks fit a slot's counts");
        let warp = warp.as_u8();
        TrajectorySeed {
            start: start
                .to_targets()
                .expect("a running move's start is a pose"),
            target: target
                .to_targets()
                .expect("a running move's target is a pose"),
            durations: durations
                .to_durations()
                .expect("a move's clocks are not negative"),
            warp: Warp::from_u8(warp).expect("a warp's own number names it"),
        }
    });

    let prev_now = prev_now.map(|now| {
        duration_from_nanos(duration_nanos(now).expect("a tick's time fits"))
            .expect("a tick's time is not negative")
    });

    let ExcursionSnapshot {
        window,
        body_yaw,
        relative_yaw,
        cone,
    } = *start_excursion;

    let tracking = tracking.map(|streak| {
        streak.map(|run| {
            let TrackingStreakSnapshot {
                anchor,
                side,
                count,
            } = run;
            let side = side.as_i8();
            TrackingStreakSnapshot {
                anchor,
                side: TrackingSide::from_i8(side).expect("a side's own number names it"),
                count,
            }
        })
    });

    MotionSnapshot {
        mode,
        trajectory,
        prev_now,
        last_goal: JointsSnapshot::from(last_goal).to_vector(),
        last_targets: TargetsSnapshot::from(last_targets)
            .to_targets()
            .expect("the last commanded head pose is a pose"),
        fk_seed: PoseSnapshot::from(fk_seed)
            .to_isometry()
            .expect("the solver's seed is a pose"),
        present_min_margin: *present_min_margin,
        start_excursion: ExcursionSnapshot {
            window,
            body_yaw,
            relative_yaw,
            cone,
        },
        miss_count: *miss_count,
        pose_failures: *pose_failures,
        tracking,
        masked: JointSet::from_bits(masked.bits()).expect("a set's own bits name it"),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use crate::seq::{AbsentSet, AnswerKind, RegId, RegValue, SeqError, SeqStep, StepContext};

    /// A context to hang a sequencer failure on, for cases that are about the
    /// failure's name rather than about where it happened.
    fn context() -> StepContext {
        StepContext::reg(SeqStep::Provision, 4, RegId::GoalPosition)
    }

    /// One fault per [`FaultCode`], each carrying numbers a slot has to bring
    /// back. Covered as a set by `every_code_has_a_fault_here`, so a new fault
    /// cannot land without a fixture.
    fn one_of_each() -> Vec<Fault> {
        vec![
            Fault::AntennaObstructed {
                joint: JointId::AntennaLeft,
                error: -0.418,
            },
            Fault::AntennaServoFault {
                joint: JointId::AntennaRight,
                id: 71,
                bits: 0b0010_1001,
            },
            Fault::HeadObstructed {
                joint: JointId::Leg(3),
                error: 0.204,
            },
            Fault::HeadServoFault {
                joint: JointId::BodyYaw,
                id: 10,
                bits: 0b1000_0000,
            },
            Fault::PositionFeedbackLost { misses: 51 },
            Fault::MeasuredPoseInvalid {
                failures: 7,
                source: FkError::NoConvergence {
                    iters: u32::MAX,
                    residual: 3.75e-4,
                },
            },
            Fault::BusFailure {
                source: BusFailureSource::Transaction {
                    id: 12,
                    kind: WireFailure::NotWritten,
                },
            },
            Fault::TorqueOffUnconfirmed { id: 9 },
        ]
    }

    #[test]
    fn every_code_has_a_fault_here() {
        let mut covered: Vec<u8> = one_of_each()
            .iter()
            .map(|fault| FaultSnapshot::from(fault).code.as_u8())
            .collect();
        covered.sort_unstable();
        let all: Vec<u8> = FaultCode::ALL.iter().map(|code| code.as_u8()).collect();
        assert_eq!(covered, all);
    }

    #[test]
    fn a_fault_a_slot_can_hold_whole_restores_equal() {
        for fault in one_of_each() {
            let restored = FaultSnapshot::from(&fault)
                .to_fault()
                .expect("a snapshot taken of a fault names that fault");
            assert_eq!(restored, fault, "restoring {fault}");
        }
    }

    /// A snapshot with every field at the value that says "not part of this
    /// fault", written out as literals rather than taken from the blank the
    /// implementation builds: a test that asked the code what blank means could
    /// not see the code change its mind.
    fn nothing_but(code: FaultCode) -> FaultSnapshot {
        FaultSnapshot {
            code,
            joint: 255,
            servo_id: 0,
            error_bits: 0,
            count: 0,
            error: 0.0,
            fk_code: FkFailureCode::NotApplicable,
            fk_a: 0.0,
            fk_b: 0.0,
            bus_source: BusSourceCode::NotApplicable,
            bus_failure_kind: 0,
        }
    }

    /// Which number lands in which field, stated rather than round-tripped.
    ///
    /// The round trip is symmetric: a payload written into the wrong field and
    /// read back out of the same wrong field restores equal and says nothing.
    /// A host does not round-trip — it reads the named fields and writes them
    /// into slot fields — so what it needs is the mapping written down. Each
    /// expectation below is the whole snapshot, so the fields the fault does
    /// not name are pinned at their blank values too.
    ///
    /// The match names every code with no wildcard: a fault added without an
    /// expectation here fails to build.
    #[test]
    fn the_numbers_a_fault_carries_land_in_the_fields_that_name_them() {
        for fault in one_of_each() {
            let snap = FaultSnapshot::from(&fault);
            let expected = match snap.code {
                // The left antenna is the last bus row, and the error is how
                // far it stood from its goal.
                FaultCode::AntennaObstructed => FaultSnapshot {
                    joint: 8,
                    error: -0.418,
                    ..nothing_but(FaultCode::AntennaObstructed)
                },
                // The right antenna, its servo's ID, and the hardware-error
                // byte as the servo reported it.
                FaultCode::AntennaServoFault => FaultSnapshot {
                    joint: 7,
                    servo_id: 71,
                    error_bits: 0b0010_1001,
                    ..nothing_but(FaultCode::AntennaServoFault)
                },
                // The fourth crank: leg three is bus row four, the yaw being
                // row zero.
                FaultCode::HeadObstructed => FaultSnapshot {
                    joint: 4,
                    error: 0.204,
                    ..nothing_but(FaultCode::HeadObstructed)
                },
                FaultCode::HeadServoFault => FaultSnapshot {
                    joint: 0,
                    servo_id: 10,
                    error_bits: 0b1000_0000,
                    ..nothing_but(FaultCode::HeadServoFault)
                },
                // A streak length, in the counter — not in the magnitude.
                FaultCode::PositionFeedbackLost => FaultSnapshot {
                    count: 51,
                    ..nothing_but(FaultCode::PositionFeedbackLost)
                },
                // Failures counted, then the solve's own two numbers in the
                // order the field docs name: iterations, then residual.
                FaultCode::MeasuredPoseInvalid => FaultSnapshot {
                    count: 7,
                    fk_code: FkFailureCode::NoConvergence,
                    fk_a: f64::from(u32::MAX),
                    fk_b: 3.75e-4,
                    ..nothing_but(FaultCode::MeasuredPoseInvalid)
                },
                // The servo the transaction was with, and the wire failure's
                // own number in the slot the source keys.
                FaultCode::BusFailure => FaultSnapshot {
                    servo_id: 12,
                    bus_source: BusSourceCode::Transaction,
                    bus_failure_kind: 4,
                    ..nothing_but(FaultCode::BusFailure)
                },
                FaultCode::TorqueOffUnconfirmed => FaultSnapshot {
                    servo_id: 9,
                    ..nothing_but(FaultCode::TorqueOffUnconfirmed)
                },
            };
            assert_eq!(snap, expected, "flattening {fault}");
        }
    }

    /// The other solve failure's two numbers, which share `fk_a`/`fk_b` with
    /// the convergence case and mean something else there.
    #[test]
    fn a_wrong_assembly_mode_carries_its_cone_then_its_height() {
        let fault = Fault::MeasuredPoseInvalid {
            failures: 3,
            source: FkError::WrongAssemblyMode {
                cone_deg: 141.2,
                z: -0.031,
            },
        };
        assert_eq!(
            FaultSnapshot::from(&fault),
            FaultSnapshot {
                count: 3,
                fk_code: FkFailureCode::WrongAssemblyMode,
                fk_a: 141.2,
                fk_b: -0.031,
                ..nothing_but(FaultCode::MeasuredPoseInvalid)
            }
        );
    }

    /// A leg index past the sixth is a joint no machine carries. The outward
    /// direction stays total by writing the sentinel; the way back refuses,
    /// because a plausible row here would name the wrong servo in a fault an
    /// operator reads.
    #[test]
    fn a_fault_about_a_leg_no_machine_has_flattens_to_no_row() {
        let fault = Fault::HeadObstructed {
            joint: JointId::Leg(9),
            error: 0.11,
        };
        let snap = FaultSnapshot::from(&fault);
        assert_eq!(snap.joint, FaultSnapshot::NO_JOINT);
        assert_eq!(snap.error, 0.11, "the evidence still travelled");
        assert_eq!(
            snap.to_fault(),
            Err(FaultSnapshotError::NoSuchJoint(FaultSnapshot::NO_JOINT))
        );
    }

    #[test]
    fn the_other_way_a_pose_solve_fails_restores_equal() {
        let fault = Fault::MeasuredPoseInvalid {
            failures: 3,
            source: FkError::WrongAssemblyMode {
                cone_deg: 141.2,
                z: -0.031,
            },
        };
        assert_eq!(FaultSnapshot::from(&fault).to_fault().unwrap(), fault);
    }

    #[test]
    fn every_shape_of_transaction_failure_restores_equal() {
        for kind in WireFailure::ALL {
            let fault = Fault::BusFailure {
                source: BusFailureSource::Transaction { id: 3, kind },
            };
            assert_eq!(FaultSnapshot::from(&fault).to_fault().unwrap(), fault);
        }
    }

    #[test]
    fn a_sequencers_verdict_restores_as_what_is_left_of_it() {
        let error = SeqError::Refused {
            context: context(),
            code: 0x24,
        };
        let fault = Fault::BusFailure {
            source: BusFailureSource::Sequence(error),
        };
        let restored = FaultSnapshot::from(&fault).to_fault().unwrap();

        // Not equal — the step context and the status code have nowhere to go —
        // but the same condition, answered the same way, at the same servo.
        assert_ne!(restored, fault);
        assert_eq!(restored.slug(), fault.slug());
        assert_eq!(restored.response(), fault.response());
        assert_eq!(restored.latches(), fault.latches());
        assert_eq!(
            restored,
            Fault::BusFailure {
                source: BusFailureSource::RestoredSequence {
                    id: context().id,
                    kind: SeqErrorKind::Refused,
                },
            }
        );
    }

    #[test]
    fn a_restored_verdict_says_what_it_knows_and_no_more() {
        let restored = Fault::BusFailure {
            source: BusFailureSource::RestoredSequence {
                id: 6,
                kind: SeqErrorKind::VoltageLow,
            },
        };
        assert_eq!(
            restored.to_string(),
            "the bus is not carrying commands: restored: a supply that never \
             reached the arming floor at servo 6"
        );
    }

    #[test]
    fn restoring_a_restored_verdict_changes_nothing_further() {
        let once = FaultSnapshot::from(&Fault::BusFailure {
            source: BusFailureSource::Sequence(SeqError::NoAnswer { context: context() }),
        })
        .to_fault()
        .unwrap();
        let twice = FaultSnapshot::from(&once).to_fault().unwrap();
        assert_eq!(twice, once);
    }

    #[test]
    fn a_code_names_the_slug_the_fault_reports_under() {
        for fault in one_of_each() {
            let expected = match FaultSnapshot::from(&fault).code {
                FaultCode::AntennaObstructed => "antenna_obstructed",
                FaultCode::AntennaServoFault => "antenna_servo_fault",
                FaultCode::HeadObstructed => "head_obstructed",
                FaultCode::HeadServoFault => "head_servo_fault",
                FaultCode::PositionFeedbackLost => "position_feedback_lost",
                FaultCode::MeasuredPoseInvalid => "measured_pose_invalid",
                FaultCode::BusFailure => "bus_failure",
                FaultCode::TorqueOffUnconfirmed => "torque_off_unconfirmed",
            };
            assert_eq!(fault.slug(), expected);
        }
    }

    #[test]
    fn a_fault_about_a_joint_is_refused_when_it_names_no_bus_row() {
        for fault in one_of_each() {
            let mut snap = FaultSnapshot::from(&fault);
            if snap.joint == FaultSnapshot::NO_JOINT {
                continue;
            }
            for row in [9, 200, FaultSnapshot::NO_JOINT] {
                snap.joint = row;
                assert_eq!(snap.to_fault(), Err(FaultSnapshotError::NoSuchJoint(row)));
            }
        }
    }

    #[test]
    fn a_fault_about_no_joint_ignores_the_row_slot() {
        let mut snap = FaultSnapshot::from(&Fault::PositionFeedbackLost { misses: 2 });
        snap.joint = 4;
        assert_eq!(
            snap.to_fault().unwrap(),
            Fault::PositionFeedbackLost { misses: 2 }
        );
    }

    #[test]
    fn an_invalid_pose_with_no_solve_failure_is_refused() {
        let mut snap = FaultSnapshot::from(&Fault::MeasuredPoseInvalid {
            failures: 4,
            source: FkError::NoConvergence {
                iters: 12,
                residual: 1e-3,
            },
        });
        snap.fk_code = FkFailureCode::NotApplicable;
        assert_eq!(snap.to_fault(), Err(FaultSnapshotError::NoSolveFailure));
    }

    #[test]
    fn an_iteration_count_that_is_not_a_count_is_refused() {
        let mut snap = FaultSnapshot::from(&Fault::MeasuredPoseInvalid {
            failures: 4,
            source: FkError::NoConvergence {
                iters: 12,
                residual: 1e-3,
            },
        });
        for value in [f64::INFINITY, -1.0, 0.5, f64::from(u32::MAX) + 1.0] {
            snap.fk_a = value;
            assert_eq!(snap.to_fault(), Err(FaultSnapshotError::NotACount(value)));
        }

        // A refusal of a number that is not a number cannot be compared to one.
        snap.fk_a = f64::NAN;
        assert!(matches!(
            snap.to_fault(),
            Err(FaultSnapshotError::NotACount(value)) if value.is_nan()
        ));
    }

    #[test]
    fn a_bus_failure_naming_no_layer_is_refused() {
        let mut snap = FaultSnapshot::from(&Fault::BusFailure {
            source: BusFailureSource::Transaction {
                id: 2,
                kind: WireFailure::Silent,
            },
        });
        snap.bus_source = BusSourceCode::NotApplicable;
        assert_eq!(snap.to_fault(), Err(FaultSnapshotError::NoBusSource));
    }

    #[test]
    fn a_failure_name_this_build_does_not_know_is_refused() {
        let mut snap = FaultSnapshot::from(&Fault::BusFailure {
            source: BusFailureSource::Transaction {
                id: 2,
                kind: WireFailure::Silent,
            },
        });
        for value in 0..=u8::MAX {
            snap.bus_source = BusSourceCode::Transaction;
            snap.bus_failure_kind = value;
            let known = WireFailure::from_u8(value).is_some();
            assert_eq!(
                snap.to_fault().is_ok(),
                known,
                "{value} as a transaction failure"
            );

            snap.bus_source = BusSourceCode::Sequence;
            let known = SeqErrorKind::from_u8(value).is_some();
            assert_eq!(
                snap.to_fault().is_ok(),
                known,
                "{value} as a sequencer failure"
            );
        }
    }

    /// A numbering is the one written down here, and the decoder names those
    /// numbers and no others.
    ///
    /// `stated` pairs every variant with the literal number this crate promises
    /// for it, built at each call site by a match with no wildcard: a variant
    /// added without a number does not build, and a variant renumbered turns
    /// this red. Asserting the decoder against `ALL` instead would restate
    /// `slot_enum!`'s own definition of `from_u8` and could not fail — the
    /// numbers themselves are the part a slot written by one build and read by
    /// the next depends on, so the numbers are what is written out.
    ///
    /// That `ALL` lists every variant is not asserted here and does not need to
    /// be: `slot_enum!` emits the variants and the list from one source, so a
    /// variant missing from the list is not expressible.
    fn numbering_is<T: Copy + PartialEq + core::fmt::Debug>(
        stated: &[(T, u8)],
        encode: impl Fn(T) -> u8,
        decode: impl Fn(u8) -> Option<T>,
    ) {
        for (variant, number) in stated {
            assert_eq!(encode(*variant), *number, "{variant:?}");
        }
        for value in 0..=u8::MAX {
            let named = stated
                .iter()
                .find(|(_, number)| *number == value)
                .map(|(variant, _)| *variant);
            assert_eq!(decode(value), named, "the number {value}");
        }
    }

    #[test]
    fn the_fault_numbering_is_the_one_written_down() {
        let stated: Vec<(FaultCode, u8)> = FaultCode::ALL
            .into_iter()
            .map(|code| {
                let number = match code {
                    FaultCode::AntennaObstructed => 1,
                    FaultCode::AntennaServoFault => 2,
                    FaultCode::HeadObstructed => 3,
                    FaultCode::HeadServoFault => 4,
                    FaultCode::PositionFeedbackLost => 5,
                    FaultCode::MeasuredPoseInvalid => 6,
                    FaultCode::BusFailure => 7,
                    FaultCode::TorqueOffUnconfirmed => 8,
                };
                (code, number)
            })
            .collect();
        numbering_is(&stated, FaultCode::as_u8, FaultCode::from_u8);
    }

    #[test]
    fn the_solve_failure_numbering_is_the_one_written_down() {
        let stated: Vec<(FkFailureCode, u8)> = FkFailureCode::ALL
            .into_iter()
            .map(|code| {
                let number = match code {
                    FkFailureCode::NotApplicable => 0,
                    FkFailureCode::NoConvergence => 1,
                    FkFailureCode::WrongAssemblyMode => 2,
                };
                (code, number)
            })
            .collect();
        numbering_is(&stated, FkFailureCode::as_u8, FkFailureCode::from_u8);
    }

    #[test]
    fn the_bus_source_numbering_is_the_one_written_down() {
        let stated: Vec<(BusSourceCode, u8)> = BusSourceCode::ALL
            .into_iter()
            .map(|code| {
                let number = match code {
                    BusSourceCode::NotApplicable => 0,
                    BusSourceCode::Transaction => 1,
                    BusSourceCode::Sequence => 2,
                };
                (code, number)
            })
            .collect();
        numbering_is(&stated, BusSourceCode::as_u8, BusSourceCode::from_u8);
    }

    #[test]
    fn the_transaction_failure_numbering_is_the_one_written_down() {
        let stated: Vec<(WireFailure, u8)> = WireFailure::ALL
            .into_iter()
            .map(|kind| {
                let number = match kind {
                    WireFailure::Silent => 1,
                    WireFailure::Corrupt => 2,
                    WireFailure::Refused => 3,
                    WireFailure::NotWritten => 4,
                    WireFailure::Port => 5,
                    WireFailure::Unsendable => 6,
                };
                (kind, number)
            })
            .collect();
        numbering_is(&stated, WireFailure::as_u8, WireFailure::from_u8);
    }

    /// Fifteen adjacent values, and the number is the only thing that survives
    /// a sequencer's verdict through a slot: an insertion rather than an append
    /// would restore every later verdict under some other failure's name.
    #[test]
    fn the_sequencer_failure_numbering_is_the_one_written_down() {
        let stated: Vec<(SeqErrorKind, u8)> = SeqErrorKind::ALL
            .into_iter()
            .map(|kind| {
                let number = match kind {
                    SeqErrorKind::NoAnswer => 1,
                    SeqErrorKind::Refused => 2,
                    SeqErrorKind::WireCorrupt => 3,
                    SeqErrorKind::VerifyMismatch => 4,
                    SeqErrorKind::WrongAnswer => 5,
                    SeqErrorKind::WrongValue => 6,
                    SeqErrorKind::UnplaceableAngle => 7,
                    SeqErrorKind::AbsentServos => 8,
                    SeqErrorKind::IdentityMismatch => 9,
                    SeqErrorKind::ProvisionMismatch => 10,
                    SeqErrorKind::VoltageLow => 11,
                    SeqErrorKind::SupplyBelowFloor => 12,
                    SeqErrorKind::UnhealthyServo => 13,
                    SeqErrorKind::RestPoseImplausible => 14,
                    SeqErrorKind::PinnedPoseUnsolvable => 15,
                };
                (kind, number)
            })
            .collect();
        numbering_is(&stated, SeqErrorKind::as_u8, SeqErrorKind::from_u8);
    }

    /// The one signed numbering, and the one whose blank value is a variant
    /// rather than a refusal: a streak that has not been placed yet is zero.
    #[test]
    fn the_tracking_side_numbering_is_the_one_written_down() {
        for side in TrackingSide::ALL {
            let number: i8 = match side {
                TrackingSide::Unplaced => 0,
                TrackingSide::Above => 1,
                TrackingSide::Below => -1,
            };
            assert_eq!(side.as_i8(), number, "{side:?}");
        }
        for value in i8::MIN..=i8::MAX {
            let named = match value {
                0 => Some(TrackingSide::Unplaced),
                1 => Some(TrackingSide::Above),
                -1 => Some(TrackingSide::Below),
                _ => None,
            };
            assert_eq!(TrackingSide::from_i8(value), named, "the number {value}");
        }
    }

    #[test]
    fn zero_names_no_fault_and_no_bus_failure_shape() {
        assert_eq!(FaultCode::from_u8(0), None);
        assert_eq!(WireFailure::from_u8(0), None);
        assert_eq!(SeqErrorKind::from_u8(0), None);
    }

    /// Every [`SeqError`], so the naming below is exhaustive by count as well
    /// as by construction.
    fn every_sequencer_failure() -> Vec<SeqError> {
        let context = context();
        let readings = [11.4; JointId::COUNT];
        vec![
            SeqError::NoAnswer { context },
            SeqError::Refused { context, code: 1 },
            SeqError::WireCorrupt { context },
            SeqError::VerifyMismatch {
                context,
                expected: RegValue::U8(1),
                read_back: RegValue::U8(0),
            },
            SeqError::WrongAnswer {
                context,
                expected: AnswerKind::Value,
                observed: AnswerKind::Pinged,
            },
            SeqError::WrongValue {
                context,
                expected: crate::seq::ValueKind::U8,
                observed: crate::seq::ValueKind::U16,
            },
            SeqError::UnplaceableAngle {
                context,
                joint: JointId::Leg(0),
                angle: f64::NAN,
            },
            SeqError::AbsentServos {
                context,
                absent: AbsentSet::new(&[1; JointId::COUNT], &[true; JointId::COUNT]),
            },
            SeqError::IdentityMismatch {
                context,
                model: 1,
                expected: 2,
            },
            SeqError::ProvisionMismatch {
                context,
                expected: RegValue::U16(3),
                observed: RegValue::U16(4),
            },
            SeqError::VoltageLow {
                context,
                readings,
                lowest: 10.0,
                limit: 11.0,
                waited: Duration::from_secs(2),
            },
            SeqError::SupplyBelowFloor {
                context,
                readings,
                lowest: 10.0,
                limit: 11.0,
            },
            SeqError::UnhealthyServo { context, bits: 4 },
            SeqError::RestPoseImplausible {
                context,
                cause: FkError::WrongAssemblyMode {
                    cone_deg: 90.0,
                    z: 0.0,
                },
            },
            SeqError::PinnedPoseUnsolvable {
                context,
                cause: FkError::WrongAssemblyMode {
                    cone_deg: 90.0,
                    z: 0.0,
                },
            },
        ]
    }

    #[test]
    fn every_sequencer_failure_has_a_name_of_its_own() {
        let mut named: Vec<u8> = every_sequencer_failure()
            .iter()
            .map(|error| error.kind().as_u8())
            .collect();
        named.sort_unstable();
        named.dedup();
        let all: Vec<u8> = SeqErrorKind::ALL
            .into_iter()
            .map(SeqErrorKind::as_u8)
            .collect();
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
    fn the_nine_angles_land_in_bus_order() {
        let mut vector = JointVector::default();
        for (row, joint) in JointId::ALL.into_iter().enumerate() {
            #[expect(
                clippy::cast_precision_loss,
                reason = "nine small whole numbers, used only as distinct marks"
            )]
            vector.set(joint, row as f64 + 0.5);
        }
        let snap = JointsSnapshot::from(&vector);
        let named = [
            snap.body_yaw,
            snap.leg_0,
            snap.leg_1,
            snap.leg_2,
            snap.leg_3,
            snap.leg_4,
            snap.leg_5,
            snap.antenna_right,
            snap.antenna_left,
        ];
        for (row, joint) in JointId::ALL.into_iter().enumerate() {
            assert_eq!(
                Some(named[row]),
                vector.get(joint),
                "the field at bus row {row} is not {joint}'s"
            );
        }
        assert_eq!(snap.to_vector(), vector);
    }

    #[test]
    fn a_command_set_restores_equal() {
        let targets = JointTargets {
            head_pose_body: a_pose(),
            body_yaw: -0.72,
            antennas: [0.31, -1.4],
        };
        let snap = TargetsSnapshot::from(&targets);
        assert_eq!(snap.antenna_right, targets.antennas[0]);
        assert_eq!(snap.antenna_left, targets.antennas[1]);
        assert_eq!(snap.to_targets().unwrap(), targets);
    }

    #[test]
    fn a_command_set_whose_pose_is_not_one_is_refused() {
        let mut snap = TargetsSnapshot::from(&JointTargets::default());
        snap.head_pose.quat_w = 0.0;
        snap.head_pose.quat_x = 0.0;
        snap.head_pose.quat_y = 0.0;
        snap.head_pose.quat_z = 0.0;
        assert_eq!(snap.to_targets(), Err(PoseSnapshotError::NotARotation(0.0)));
    }

    #[test]
    fn the_three_clocks_restore_equal() {
        let durations = MoveDurations {
            head: Duration::from_nanos(2_000_000_001),
            antennas: [Duration::from_millis(800), Duration::ZERO],
        };
        let snap = DurationsSnapshot::try_from(durations).unwrap();
        assert_eq!(snap.head_ns, 2_000_000_001);
        assert_eq!(snap.antenna_right_ns, 800_000_000);
        assert_eq!(snap.antenna_left_ns, 0);
        assert_eq!(snap.to_durations().unwrap(), durations);
    }

    #[test]
    fn a_clock_a_slot_cannot_hold_is_refused_rather_than_shortened() {
        let too_long = Duration::from_nanos(u64::MAX);
        assert_eq!(
            DurationsSnapshot::try_from(MoveDurations::uniform(too_long)),
            Err(DurationError::TooLong(too_long))
        );
        assert_eq!(
            duration_nanos(too_long),
            Err(DurationError::TooLong(too_long))
        );
        assert_eq!(duration_nanos(Duration::from_nanos(7)), Ok(7));
    }

    #[test]
    fn a_length_of_time_that_runs_backwards_is_refused() {
        let snap = DurationsSnapshot {
            head_ns: 1,
            antenna_right_ns: -1,
            antenna_left_ns: 1,
        };
        assert_eq!(snap.to_durations(), Err(DurationError::Negative(-1)));
        assert_eq!(duration_from_nanos(-1), Err(DurationError::Negative(-1)));
        assert_eq!(duration_from_nanos(0), Ok(Duration::ZERO));
    }

    #[test]
    fn the_mode_numbering_is_the_one_written_down() {
        let stated: Vec<(ModeCode, u8)> = ModeCode::ALL
            .into_iter()
            .map(|code| {
                let number = match code {
                    ModeCode::Holding => 1,
                    ModeCode::Moving => 2,
                    ModeCode::Faulted => 3,
                };
                (code, number)
            })
            .collect();
        numbering_is(&stated, ModeCode::as_u8, ModeCode::from_u8);

        assert_eq!(ModeCode::from_u8(0), None, "an unwritten slot is no mode");
        assert_eq!(ModeCode::of(Mode::Holding), ModeCode::Holding);
        assert_eq!(
            ModeCode::of(Mode::Moving {
                elapsed: Duration::ZERO
            }),
            ModeCode::Moving
        );
        assert_eq!(
            ModeCode::of(Mode::Faulted(Fault::TorqueOffUnconfirmed { id: 3 })),
            ModeCode::Faulted
        );
    }

    /// A snapshot with something in every member that is not a plain number, so
    /// the crossing has all of them to carry.
    fn a_full_snapshot() -> MotionSnapshot {
        let mut masked = JointSet::EMPTY;
        masked.insert(JointId::Leg(2));
        masked.insert(JointId::AntennaLeft);
        let mut tracking = [None; JointId::COUNT];
        tracking[0] = Some(TrackingStreakSnapshot {
            anchor: 0.11,
            side: TrackingSide::Above,
            count: 4,
        });
        tracking[8] = Some(TrackingStreakSnapshot {
            anchor: -1.2,
            side: TrackingSide::Below,
            count: 1,
        });
        tracking[3] = Some(TrackingStreakSnapshot {
            anchor: 0.0,
            side: TrackingSide::Unplaced,
            count: 9,
        });
        MotionSnapshot {
            mode: Mode::Moving {
                elapsed: Duration::from_millis(320),
            },
            trajectory: Some(TrajectorySeed {
                start: JointTargets::default(),
                target: JointTargets {
                    head_pose_body: a_pose(),
                    body_yaw: 0.4,
                    antennas: [-0.2, 0.9],
                },
                durations: MoveDurations::split(
                    Duration::from_millis(900),
                    Duration::from_millis(450),
                ),
                warp: Warp::Linear,
            }),
            prev_now: Some(Duration::from_nanos(1_234_567_891)),
            last_goal: JointVector {
                body_yaw: 0.2,
                legs: [0.1, -0.2, 0.3, -0.4, 0.5, -0.6],
                antennas: [0.7, -0.8],
            },
            last_targets: JointTargets {
                head_pose_body: a_pose(),
                body_yaw: -0.05,
                antennas: [0.0, 0.02],
            },
            fk_seed: a_pose(),
            present_min_margin: 0.0134,
            start_excursion: ExcursionSnapshot {
                window: [0.0, 0.02, 0.0, 0.0, 0.11, 0.0],
                body_yaw: 0.3,
                relative_yaw: 0.0,
                cone: 0.007,
            },
            miss_count: 2,
            pose_failures: 1,
            tracking,
            masked,
        }
    }

    #[test]
    fn every_member_survives_the_flat_forms_a_slot_holds_it_in() {
        let snap = a_full_snapshot();
        assert_eq!(crossed(&snap), snap);
    }

    #[test]
    fn the_modes_a_move_is_not_in_cross_too() {
        let moving = a_full_snapshot();
        let holding = MotionSnapshot {
            mode: Mode::Holding,
            trajectory: None,
            prev_now: None,
            ..moving
        };
        assert_eq!(crossed(&holding), holding);

        for fault in one_of_each() {
            let faulted = MotionSnapshot {
                mode: Mode::Faulted(fault),
                ..moving
            };
            assert_eq!(crossed(&faulted), faulted, "faulted with {fault}");
        }
    }

    #[test]
    fn a_sequencers_verdict_keeps_the_servo_it_happened_at() {
        for error in every_sequencer_failure() {
            let snap = FaultSnapshot::from(&Fault::BusFailure {
                source: BusFailureSource::Sequence(error),
            });
            assert_eq!(snap.servo_id, error.context().id);
            assert_eq!(snap.bus_failure_kind, error.kind().as_u8());
            snap.to_fault().expect("every named failure restores");
        }
    }
}
