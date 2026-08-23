//! A standing fault, in the schema a host keeps it in, and the classification
//! every layer reads off it.
//!
//! [`Fault`] is one shape per condition, each carrying the evidence that
//! classified it, and a fixed-layout slot has no union to hold them in.
//! [`FaultSnap`] is that union written out — one field per number a fault can
//! carry, keyed by [`FaultKind`] — and this module is the only place the two
//! forms meet. [`write`] assigns the schema's own fields through the validated
//! view, [`read`] reads them back, and the host validates once at the boundary
//! and hands the view down.
//!
//! What the fields mean is stated at the declaration, in `motion/tick_state.clk`;
//! what this module adds is the two judgements a slot's numbers cannot make for
//! themselves:
//!
//! - **Which fields are meaningful is the kind's business.** Nothing infers a
//!   fault from the payload, so [`write`] blanks every field it is not filling
//!   and a blank field is never read back as an observation.
//! - **Whether the evidence suits the kind** is [`read`]'s, asked once here
//!   rather than again in every host of the tick. A pose fault whose solve did
//!   not fail, a bus fault with no layer that judged it, a fault about a joint
//!   that names none: refused rather than repaired, because every repair
//!   available would report something nobody observed.
//!
//! # The classification
//!
//! [`slug`], [`response`] and [`latches`] are functions over [`FaultKind`], the
//! vocabulary's own enum: the single classification point in the stack, made
//! once where a fault is raised and travelling as the value it was classified
//! as. They are total over the whole report vocabulary, which is wider than the
//! faults — a move abandoned and a command refused travel on the same channel
//! and are not conditions of the machine, so nothing is answered for them.
//!
//! Six of the eight faults [`read`] back equal to the value [`write`] was
//! given. [`Fault::TorqueOffUnconfirmed`] is already flat, and so is
//! [`Fault::BusFailure`] when its source is a transaction; a bus failure whose
//! source was a *sequencer's* verdict reads back as
//! [`BusFailureSource::RestoredSequence`], which agrees with the original on
//! the slug, the response, the latch and the servo, and drops the step context
//! and the payload the slot never held.

use reachy_kin::FkError;
use thiserror::Error;

use crate::joints::{self, JointGroup, JointRef, ServoHealth, group_of};
use crate::snap::{BusSourceKind, FkFieldError, fk_cause, fk_fields};
use crate::tick::{BusFailureSource, Fault, ResponseKind};
use crate::vocab::known_nonzero;
use brenn_reachy__motion__faults_clk_rs::WireFailureWire;
use brenn_reachy__motion__seq_clk_rs::SeqFailureKindWire;
use brenn_reachy__motion__tick_state_clk_rs::{FaultSnap, FaultSnapWire};

/// What a tick reported: the eight faults, and the three outcomes that are not
/// faults but travel on the same channel.
///
/// The vocabulary's own enum, declared in `motion/faults.clk`. The classification
/// functions in this module are written over it, so a condition added to the
/// vocabulary is a decision made here at compile time.
pub use brenn_reachy__motion__faults_clk_rs::FaultKind;

/// Numbers in a fault slot that name no standing fault.
///
/// Only reachable from fields that were assembled rather than written by
/// [`write`] — every fault the tick raises reads back — which in practice means
/// a slot holding bytes no fault ever wrote.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum FaultError {
    /// A number that names no fault a machine can stand parked on: the
    /// unwritten zero, or one of the three outcomes that are not conditions of
    /// the machine.
    #[error("{0} names no fault a machine can stand parked on")]
    NoStandingFault(FaultKind),
    /// A fault about a joint, naming none. The evidence says which servo was
    /// concerned; a fault that lost it reads to an operator as a report about
    /// the whole machine.
    #[error("this fault is about a joint, and the slot names none")]
    NoJoint,
    /// A pose fault whose three solve fields name no solve failure, or whose
    /// iteration count is not a count.
    #[error("the pose is invalid, but the solve failure is not one: {0}")]
    Solve(#[from] FkFieldError),
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

/// Which fault `fault` is, dropping its evidence.
///
/// A total function over the raised side: every fault the stack raises is one of
/// the eight, and the three report-only outcomes are not among them.
#[must_use]
pub fn kind(fault: &Fault) -> FaultKind {
    match fault {
        Fault::AntennaObstructed { .. } => FaultKind::AntennaObstructed,
        Fault::AntennaServoFault { .. } => FaultKind::AntennaServoFault,
        Fault::HeadObstructed { .. } => FaultKind::HeadObstructed,
        Fault::HeadServoFault { .. } => FaultKind::HeadServoFault,
        Fault::PositionFeedbackLost { .. } => FaultKind::PositionFeedbackLost,
        Fault::MeasuredPoseInvalid { .. } => FaultKind::MeasuredPoseInvalid,
        Fault::BusFailure { .. } => FaultKind::BusFailure,
        Fault::TorqueOffUnconfirmed { .. } => FaultKind::TorqueOffUnconfirmed,
    }
}

/// The servo `fault` is about, or [`JointRef::None`] where it is about none.
#[must_use]
pub fn joint(fault: &Fault) -> JointRef {
    match *fault {
        Fault::AntennaObstructed { joint, .. }
        | Fault::AntennaServoFault { joint, .. }
        | Fault::HeadObstructed { joint, .. }
        | Fault::HeadServoFault { joint, .. } => joint,
        Fault::PositionFeedbackLost { .. }
        | Fault::MeasuredPoseInvalid { .. }
        | Fault::BusFailure { .. }
        | Fault::TorqueOffUnconfirmed { .. } => JointRef::None,
    }
}

/// The magnitude that carried the classification, or zero where the evidence is
/// not a magnitude.
///
/// Radians from a goal for the two obstructions; nothing for the rest, whose
/// evidence is a count, a hardware byte or a failure's name.
#[must_use]
pub fn detail(fault: &Fault) -> f64 {
    match *fault {
        Fault::AntennaObstructed { error, .. } | Fault::HeadObstructed { error, .. } => error,
        Fault::AntennaServoFault { .. }
        | Fault::HeadServoFault { .. }
        | Fault::PositionFeedbackLost { .. }
        | Fault::MeasuredPoseInvalid { .. }
        | Fault::BusFailure { .. }
        | Fault::TorqueOffUnconfirmed { .. } => 0.0,
    }
}

/// How many consecutive periods the evidence ran for, or zero where it is not a
/// run.
#[must_use]
pub fn count(fault: &Fault) -> u32 {
    match *fault {
        Fault::PositionFeedbackLost { misses } => misses,
        Fault::MeasuredPoseInvalid { failures, .. } => failures,
        Fault::AntennaObstructed { .. }
        | Fault::AntennaServoFault { .. }
        | Fault::HeadObstructed { .. }
        | Fault::HeadServoFault { .. }
        | Fault::BusFailure { .. }
        | Fault::TorqueOffUnconfirmed { .. } => 0,
    }
}

/// The slug `kind` is reported, alerted and logged under.
///
/// One name per condition, everywhere: the session's timeline row, the operator
/// line and the daemon's fault cell all say this word, so an operator reading a
/// log and an operator reading a status file are reading about the same thing.
/// The vocabulary's own spelling, which is what a recorded log and a running
/// build have in common.
#[must_use]
pub fn slug(kind: FaultKind) -> &'static str {
    match kind {
        FaultKind::None => "none",
        FaultKind::AntennaObstructed => "antenna_obstructed",
        FaultKind::AntennaServoFault => "antenna_servo_fault",
        FaultKind::HeadObstructed => "head_obstructed",
        FaultKind::HeadServoFault => "head_servo_fault",
        FaultKind::PositionFeedbackLost => "position_feedback_lost",
        FaultKind::MeasuredPoseInvalid => "measured_pose_invalid",
        FaultKind::BusFailure => "bus_failure",
        FaultKind::TorqueOffUnconfirmed => "torque_off_unconfirmed",
        FaultKind::MoveAbortedEnvelope => "move_aborted_envelope",
        FaultKind::MoveAbortedStep => "move_aborted_step",
        FaultKind::CommandRejected => "command_rejected",
    }
}

/// The maneuver and post-state `kind` is answered with.
///
/// The single classification point in the stack. Exhaustive by construction: a
/// condition added to the vocabulary is a classification decision made here, at
/// compile time, and never a default anybody falls through to.
///
/// [`ResponseKind::None`] for the four values that are not conditions of the
/// machine — the unwritten zero, the two abandoned moves and the refused
/// command. Nothing is answered, because nothing about the machine is wrong: a
/// plan was.
#[must_use]
pub fn response(kind: FaultKind) -> ResponseKind {
    match kind {
        // Each antenna is its own non-load-bearing joint, so its trouble stays
        // its own: the pair goes limp and the head keeps its presence.
        FaultKind::AntennaObstructed | FaultKind::AntennaServoFault => {
            ResponseKind::DegradeAntennas
        }
        // The motors still command, so the machine yields under control rather
        // than dropping — which is also what a hand pushing the head down
        // wants.
        FaultKind::HeadObstructed => ResponseKind::SlowStowToRest,
        // Semi-controlled descent: the faulted servo is released on the spot and
        // the rest carry the head down, then everything releases and waits for
        // an operator.
        FaultKind::HeadServoFault => ResponseKind::MaskedSlowStowToPark,
        // Control is not trusted: without feedback, or against a mechanism
        // outside its own model, a stow is a maneuver commanded blind or a
        // maneuver that grinds.
        FaultKind::PositionFeedbackLost | FaultKind::MeasuredPoseInvalid => {
            ResponseKind::ImmediateAllTorqueOffToPark
        }
        // Nothing can be commanded, so nothing controlled can be attempted.
        FaultKind::BusFailure => ResponseKind::ImmediateAllTorqueOffToPark,
        // Degenerate: the torque-off already ran. What remains is the park and
        // the alert, because an unconfirmed release is never Resting.
        FaultKind::TorqueOffUnconfirmed => ResponseKind::ImmediateAllTorqueOffToPark,
        FaultKind::None
        | FaultKind::MoveAbortedEnvelope
        | FaultKind::MoveAbortedStep
        | FaultKind::CommandRejected => ResponseKind::None,
    }
}

/// Whether `kind` takes the tick out of service, rather than leaving it
/// commanding what remains.
///
/// True only where control itself is what stopped being trustworthy. A grabbed
/// head and a released servo both leave a machine that still takes goals, and
/// the wind-down that answers them is driven through this same tick. False for
/// the four values that are answered with nothing.
#[must_use]
pub fn latches(kind: FaultKind) -> bool {
    matches!(
        response(kind),
        ResponseKind::ImmediateAllTorqueOffToPark | ResponseKind::ImmediateAllTorqueOffToRest
    )
}

/// The condition a servo's hardware-error byte is evidence of, or `None` where
/// it is evidence of nothing.
///
/// The judgement the decision tick makes over its own health poll, at one site
/// so that a host reading the same bytes off a driver's rotating read reaches
/// the same verdict. Two things decide it and nothing else does: whether the
/// byte carries anything but the input-voltage bit, and which group the row
/// belongs to — an antenna's trouble is its own, and a head servo's is the
/// head's.
///
/// `None` for a clear byte, for the input-voltage bit alone — expected on this
/// platform, reported rather than acted on — and for a row the bus does not
/// have, which is evidence about no servo at all. Masking is the caller's: a
/// masked joint keeps flagging for the rest of the session, and what to do about
/// a condition already answered is a question about the machine's state rather
/// than about these bits.
#[must_use]
pub fn fault_of_health(row: usize, bits: u8) -> Option<FaultKind> {
    let joint = joints::joint_ref(row)?;
    // The reading's own judgement of its bits, asked through the type that owns
    // the rule rather than restated here. The bus id is not part of it: what
    // names the servo in the report is the row the caller read.
    if (ServoHealth { id: 0, bits }).healthy_or_voltage_only() {
        return None;
    }
    match group_of(joint) {
        Some(JointGroup::Antennas) => Some(FaultKind::AntennaServoFault),
        // The yaw and the six cranks all hold the head up, so any of them
        // reporting a hardware error is the head's condition.
        Some(JointGroup::BodyYaw | JointGroup::Legs) | None => Some(FaultKind::HeadServoFault),
    }
}

/// Write `fault` into the fields a slot holds a standing fault in.
///
/// Total: every field is assigned, the blanks included, so a slot carrying an
/// earlier fault is left describing this one and nothing else. A fault about a
/// joint writes the joint it is about, and one about the machine writes
/// [`JointRef::None`] — the vocabulary's own way of saying that, which the way
/// back refuses where a joint is wanted.
pub fn write(out: &mut FaultSnap, fault: &Fault) {
    blank(out);
    out.code = kind(fault);
    out.joint = joint(fault);
    out.error = detail(fault);
    out.count = count(fault);

    match *fault {
        Fault::AntennaObstructed { .. }
        | Fault::HeadObstructed { .. }
        | Fault::PositionFeedbackLost { .. } => {}
        Fault::AntennaServoFault { id, bits, .. } | Fault::HeadServoFault { id, bits, .. } => {
            out.servo_id = id;
            out.error_bits = bits;
        }
        Fault::MeasuredPoseInvalid { source, .. } => {
            let (code, a, b) = fk_fields(source);
            out.fk_code = code;
            out.fk_a = a;
            out.fk_b = b;
        }
        Fault::BusFailure { source } => {
            let (bus_source, servo_id, failure) = match source {
                BusFailureSource::Transaction { id, kind } => (
                    BusSourceKind::Transaction,
                    id,
                    WireFailureWire::from(kind).0,
                ),
                BusFailureSource::Sequence(error) => (
                    BusSourceKind::Sequence,
                    error.context().id,
                    SeqFailureKindWire::from(error.kind()).0,
                ),
                BusFailureSource::RestoredSequence { id, kind } => (
                    BusSourceKind::Sequence,
                    id,
                    SeqFailureKindWire::from(kind).0,
                ),
            };
            out.servo_id = servo_id;
            out.bus_source = bus_source;
            out.bus_failure_kind = failure;
        }
        Fault::TorqueOffUnconfirmed { id } => out.servo_id = id,
    }
}

/// Every field at its "not part of this fault" value.
///
/// The declared initial state, taken from the schema rather than restated here:
/// a fresh message cleared through the generated route, swapped in over what the
/// slot held. A field added to the schema is blanked by that without this
/// function naming it.
fn blank(out: &mut FaultSnap) {
    let mut fresh = FaultSnapWire::new();
    core::mem::swap(out, fresh.clear_valid());
}

/// The standing fault those fields describe.
///
/// # Errors
///
/// [`FaultError`], one variant per way a slot's numbers can fail to name a
/// fault: a kind no machine stands parked on, a fault about a joint that names
/// none, a solve that did not fail, a bus failure with no layer or a failure
/// name this build does not know.
pub fn read(slot: &FaultSnap) -> Result<Fault, FaultError> {
    let joint = || {
        (slot.joint != JointRef::None)
            .then_some(slot.joint)
            .ok_or(FaultError::NoJoint)
    };
    Ok(match slot.code {
        FaultKind::AntennaObstructed => Fault::AntennaObstructed {
            joint: joint()?,
            error: slot.error,
        },
        FaultKind::AntennaServoFault => Fault::AntennaServoFault {
            joint: joint()?,
            id: slot.servo_id,
            bits: slot.error_bits,
        },
        FaultKind::HeadObstructed => Fault::HeadObstructed {
            joint: joint()?,
            error: slot.error,
        },
        FaultKind::HeadServoFault => Fault::HeadServoFault {
            joint: joint()?,
            id: slot.servo_id,
            bits: slot.error_bits,
        },
        FaultKind::PositionFeedbackLost => Fault::PositionFeedbackLost { misses: slot.count },
        FaultKind::MeasuredPoseInvalid => Fault::MeasuredPoseInvalid {
            failures: slot.count,
            source: solve(slot)?,
        },
        FaultKind::BusFailure => Fault::BusFailure {
            source: bus_source(slot)?,
        },
        FaultKind::TorqueOffUnconfirmed => Fault::TorqueOffUnconfirmed { id: slot.servo_id },
        kind @ (FaultKind::None
        | FaultKind::MoveAbortedEnvelope
        | FaultKind::MoveAbortedStep
        | FaultKind::CommandRejected) => return Err(FaultError::NoStandingFault(kind)),
    })
}

/// The solve failure the three solve fields describe.
fn solve(slot: &FaultSnap) -> Result<FkError, FaultError> {
    Ok(fk_cause(slot.fk_code, slot.fk_a, slot.fk_b)?)
}

/// The bus failure the source and its number describe, as far as a slot can
/// carry one.
fn bus_source(slot: &FaultSnap) -> Result<BusFailureSource, FaultError> {
    match slot.bus_source {
        BusSourceKind::NotApplicable => Err(FaultError::NoBusSource),
        BusSourceKind::Transaction => Ok(BusFailureSource::Transaction {
            id: slot.servo_id,
            // The zero is refused with every number outside the six: a
            // transaction that failed failed in a shape, and a fault claiming a
            // transaction source without one is a slot nothing wrote.
            kind: known_nonzero(WireFailureWire(slot.bus_failure_kind).to_known())
                .ok_or(FaultError::NoSuchWireFailure(slot.bus_failure_kind))?,
        }),
        BusSourceKind::Sequence => Ok(BusFailureSource::RestoredSequence {
            id: slot.servo_id,
            // The zero is refused with every other number that names no
            // failure: a restored sequence failure has a name, and a fault about
            // one that does not is a slot nothing wrote.
            kind: known_nonzero(SeqFailureKindWire(slot.bus_failure_kind).to_known())
                .ok_or(FaultError::NoSuchSeqError(slot.bus_failure_kind))?,
        }),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::joints::ROWS;
    use crate::seq::{RegId, SeqError, SeqFailureKind, SeqStepKind, StepContext};
    use crate::snap::FkFailureKind;
    use crate::testutil::{every_fault, every_sequencer_failure};
    use crate::tick::WireFailure;
    use crate::vocab::without_zero;

    /// A context to hang a sequencer failure on, for cases that are about the
    /// failure's name rather than about where it happened.
    fn context() -> StepContext {
        StepContext::reg(SeqStepKind::Provision, 4, RegId::GoalPosition)
    }

    /// The kinds a machine can stand parked on: the vocabulary less its zero and
    /// less the three outcomes that are not conditions of the machine.
    fn standing() -> Vec<FaultKind> {
        without_zero(FaultKind::VARIANTS)
            .filter(|kind| {
                !matches!(
                    kind,
                    FaultKind::MoveAbortedEnvelope
                        | FaultKind::MoveAbortedStep
                        | FaultKind::CommandRejected
                )
            })
            .collect()
    }

    /// A slot holding `fault` and nothing else.
    fn written(fault: &Fault) -> FaultSnapWire {
        let mut wire = FaultSnapWire::new();
        write(wire.clear_valid(), fault);
        wire
    }

    /// Every standing kind has a fixture, so every sweep below is exhaustive by
    /// count as well as by construction.
    #[test]
    fn every_standing_kind_has_a_fault_here() {
        let mut covered: Vec<FaultKind> = every_fault().iter().map(kind).collect();
        covered.sort_unstable();
        assert_eq!(covered, standing());
    }

    /// Every byte, every row: which condition a health reading is evidence of.
    ///
    /// Swept over all 256 bytes rather than a handful, because what the rule
    /// says is that *one* bit is forgiven and every other one is not, and a
    /// case naming two or three bytes would pass over a build that forgave a
    /// second bit. The expectation is written from the group rather than from
    /// `fault_of_health`'s own answer, so the two are separate statements.
    #[test]
    fn a_health_reading_is_evidence_about_the_group_its_row_belongs_to() {
        for (row, joint) in ROWS.into_iter().enumerate() {
            let antenna = group_of(joint) == Some(JointGroup::Antennas);
            for bits in 0..=u8::MAX {
                let forgiven = bits & !ServoHealth::INPUT_VOLTAGE == 0;
                let expected = match (forgiven, antenna) {
                    (true, _) => None,
                    (false, true) => Some(FaultKind::AntennaServoFault),
                    (false, false) => Some(FaultKind::HeadServoFault),
                };
                assert_eq!(
                    fault_of_health(row, bits),
                    expected,
                    "row {row} ({joint}) reading {bits:#010b}"
                );
            }
        }
    }

    /// A row the bus does not have is evidence about no servo.
    ///
    /// The one arm that is not about the bits: a reading filed under a row past
    /// the ninth came from somewhere this build cannot place, and a verdict
    /// about it would name a servo nobody read.
    #[test]
    fn a_reading_from_no_row_this_bus_has_is_evidence_of_nothing() {
        for bits in [0x00, ServoHealth::INPUT_VOLTAGE, 0x20, 0xff] {
            assert_eq!(fault_of_health(ROWS.len(), bits), None);
            assert_eq!(fault_of_health(usize::MAX, bits), None);
        }
    }

    #[test]
    fn a_fault_a_slot_can_hold_whole_restores_equal() {
        for fault in every_fault() {
            let restored = read(
                written(&fault)
                    .validate()
                    .expect("a written fault validates"),
            )
            .expect("a slot written from a fault names that fault");
            assert_eq!(restored, fault, "restoring {fault}");
        }
    }

    /// Which number lands in which field, stated rather than round-tripped.
    ///
    /// The round trip is symmetric: a payload written into the wrong field and
    /// read back out of the same wrong field restores equal and says nothing. A
    /// host does not round-trip — it reads the named fields — so what it needs
    /// is the mapping written down. Each expectation below states every field,
    /// so the fields the fault does not name are pinned at their blank values
    /// too.
    ///
    /// The match names every standing kind with no wildcard: a fault added
    /// without an expectation here fails to build.
    #[test]
    fn the_numbers_a_fault_carries_land_in_the_fields_that_name_them() {
        for fault in every_fault() {
            let wire = written(&fault);
            let slot = wire.validate().expect("a written fault validates");
            let expected = match slot.code {
                // The left antenna, and the error is how far it stood from its
                // goal.
                FaultKind::AntennaObstructed => (JointRef::AntennaLeft, 0, 0, 0, -0.418),
                // The right antenna, its servo's ID, and the hardware-error
                // byte as the servo reported it.
                FaultKind::AntennaServoFault => (JointRef::AntennaRight, 71, 0b0010_1001, 0, 0.0),
                FaultKind::HeadObstructed => (JointRef::Leg3, 0, 0, 0, 0.204),
                FaultKind::HeadServoFault => (JointRef::BodyYaw, 10, 0b1000_0000, 0, 0.0),
                // A streak length, in the counter — not in the magnitude.
                FaultKind::PositionFeedbackLost => (JointRef::None, 0, 0, 51, 0.0),
                // Failures counted, and the solve's own two numbers asserted
                // below.
                FaultKind::MeasuredPoseInvalid => (JointRef::None, 0, 0, 7, 0.0),
                // The servo the transaction was with; the source and its number
                // are asserted below.
                FaultKind::BusFailure => (JointRef::None, 12, 0, 0, 0.0),
                FaultKind::TorqueOffUnconfirmed => (JointRef::None, 9, 0, 0, 0.0),
                FaultKind::None
                | FaultKind::MoveAbortedEnvelope
                | FaultKind::MoveAbortedStep
                | FaultKind::CommandRejected => panic!("{fault} is a standing fault"),
            };
            assert_eq!(
                (
                    slot.joint,
                    slot.servo_id,
                    slot.error_bits,
                    slot.count,
                    slot.error
                ),
                expected,
                "flattening {fault}"
            );

            // The solve fields and the bus fields belong to one kind each, and
            // are blank for every other.
            let solve = (slot.fk_code, slot.fk_a, slot.fk_b);
            let bus = (slot.bus_source, slot.bus_failure_kind);
            match slot.code {
                FaultKind::MeasuredPoseInvalid => {
                    assert_eq!(
                        solve,
                        (FkFailureKind::NoConvergence, f64::from(u32::MAX), 3.75e-4)
                    );
                    assert_eq!(bus, (BusSourceKind::NotApplicable, 0));
                }
                FaultKind::BusFailure => {
                    assert_eq!(solve, (FkFailureKind::NotApplicable, 0.0, 0.0));
                    assert_eq!(bus, (BusSourceKind::Transaction, 4));
                }
                _ => {
                    assert_eq!(solve, (FkFailureKind::NotApplicable, 0.0, 0.0));
                    assert_eq!(bus, (BusSourceKind::NotApplicable, 0));
                }
            }
        }
    }

    /// The other solve failure's two numbers, which share `fk_a`/`fk_b` with the
    /// convergence case and mean something else there.
    #[test]
    fn a_wrong_assembly_mode_carries_its_cone_then_its_height() {
        let fault = Fault::MeasuredPoseInvalid {
            failures: 3,
            source: FkError::WrongAssemblyMode {
                cone_deg: 141.2,
                z: -0.031,
            },
        };
        let wire = written(&fault);
        let slot = wire.validate().unwrap();
        assert_eq!(
            (slot.fk_code, slot.fk_a, slot.fk_b),
            (FkFailureKind::WrongAssemblyMode, 141.2, -0.031)
        );
        assert_eq!(read(slot).unwrap(), fault);
    }

    /// A slot describes the fault last written into it and nothing of the one
    /// before: every field is assigned on the way out, so evidence from an
    /// earlier condition cannot stand beside a kind that never carried it.
    ///
    /// Written over what the previous fault left, with nothing clearing the slot
    /// in between — the reuse a host does — and over every ordered pair of
    /// faults, so a field one condition fills and the next does not is caught
    /// whichever pair of conditions those are.
    #[test]
    fn a_slot_reused_carries_nothing_of_the_earlier_fault() {
        for earlier in every_fault() {
            for later in every_fault() {
                let mut reused = written(&earlier);
                let slot = reused
                    .validate_mut()
                    .expect("a written fault stays a written fault");
                write(slot, &later);

                let fresh = written(&later);
                assert_eq!(
                    &*slot,
                    fresh.validate().expect("a written fault validates"),
                    "{earlier} then {later}"
                );
                assert_eq!(read(slot).unwrap(), later, "{earlier} then {later}");
            }
        }
    }

    /// A fault about the machine rather than about one servo names no joint, and
    /// one about a joint that lost it is refused: a plausible servo here would
    /// name one in a report the fault was never about.
    #[test]
    fn a_fault_about_a_joint_is_refused_when_it_names_none() {
        for fault in every_fault() {
            let mut wire = written(&fault);
            let slot = wire.clear_valid();
            write(slot, &fault);
            if slot.joint == JointRef::None {
                continue;
            }
            slot.joint = JointRef::None;
            assert_eq!(read(slot), Err(FaultError::NoJoint), "{fault}");
        }
    }

    #[test]
    fn a_fault_about_no_joint_ignores_the_joint_field() {
        let mut wire = written(&Fault::PositionFeedbackLost { misses: 2 });
        let slot = wire.clear_valid();
        write(slot, &Fault::PositionFeedbackLost { misses: 2 });
        slot.joint = JointRef::Leg2;
        assert_eq!(
            read(slot).unwrap(),
            Fault::PositionFeedbackLost { misses: 2 }
        );
    }

    #[test]
    fn every_shape_of_transaction_failure_restores_equal() {
        for kind in crate::vocab::without_zero(WireFailure::VARIANTS) {
            let fault = Fault::BusFailure {
                source: BusFailureSource::Transaction { id: 3, kind },
            };
            assert_eq!(read(written(&fault).validate().unwrap()).unwrap(), fault);
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
        let restored = read(written(&fault).validate().unwrap()).unwrap();

        // Not equal — the step context and the status code have nowhere to go —
        // but the same condition, answered the same way, at the same servo.
        assert_ne!(restored, fault);
        assert_eq!(slug(kind(&restored)), slug(kind(&fault)));
        assert_eq!(response(kind(&restored)), response(kind(&fault)));
        assert_eq!(latches(kind(&restored)), latches(kind(&fault)));
        assert_eq!(
            restored,
            Fault::BusFailure {
                source: BusFailureSource::RestoredSequence {
                    id: context().id,
                    kind: SeqFailureKind::Refused,
                },
            }
        );
    }

    #[test]
    fn a_restored_verdict_says_what_it_knows_and_no_more() {
        let restored = Fault::BusFailure {
            source: BusFailureSource::RestoredSequence {
                id: 6,
                kind: SeqFailureKind::VoltageLow,
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
        let once = read(
            written(&Fault::BusFailure {
                source: BusFailureSource::Sequence(SeqError::NoAnswer { context: context() }),
            })
            .validate()
            .unwrap(),
        )
        .unwrap();
        let twice = read(written(&once).validate().unwrap()).unwrap();
        assert_eq!(twice, once);
    }

    #[test]
    fn an_invalid_pose_with_no_solve_failure_is_refused() {
        let mut wire = written(&Fault::MeasuredPoseInvalid {
            failures: 4,
            source: FkError::NoConvergence {
                iters: 12,
                residual: 1e-3,
            },
        });
        let slot = wire.validate_mut().unwrap();
        slot.fk_code = FkFailureKind::NotApplicable;
        assert_eq!(
            read(slot),
            Err(FaultError::Solve(FkFieldError::NoSolveFailure))
        );
    }

    #[test]
    fn an_iteration_count_that_is_not_a_count_is_refused() {
        let mut wire = written(&Fault::MeasuredPoseInvalid {
            failures: 4,
            source: FkError::NoConvergence {
                iters: 12,
                residual: 1e-3,
            },
        });
        let slot = wire.validate_mut().unwrap();
        for value in [f64::INFINITY, -1.0, 0.5, f64::from(u32::MAX) + 1.0] {
            slot.fk_a = value;
            assert_eq!(
                read(slot),
                Err(FaultError::Solve(FkFieldError::NotACount(value)))
            );
        }

        // A refusal of a number that is not a number cannot be compared to one.
        slot.fk_a = f64::NAN;
        assert!(matches!(
            read(slot),
            Err(FaultError::Solve(FkFieldError::NotACount(value))) if value.is_nan()
        ));
    }

    #[test]
    fn a_bus_failure_naming_no_layer_is_refused() {
        let mut wire = written(&Fault::BusFailure {
            source: BusFailureSource::Transaction {
                id: 2,
                kind: WireFailure::Silent,
            },
        });
        let slot = wire.validate_mut().unwrap();
        slot.bus_source = BusSourceKind::NotApplicable;
        assert_eq!(read(slot), Err(FaultError::NoBusSource));
    }

    #[test]
    fn a_failure_name_this_build_does_not_know_is_refused() {
        let mut wire = written(&Fault::BusFailure {
            source: BusFailureSource::Transaction {
                id: 2,
                kind: WireFailure::Silent,
            },
        });
        let slot = wire.validate_mut().unwrap();
        for value in 0..=u8::MAX {
            slot.bus_source = BusSourceKind::Transaction;
            slot.bus_failure_kind = value;
            let known = matches!(
                WireFailureWire(value).to_known(),
                Some(kind) if kind != WireFailure::None
            );
            assert_eq!(
                read(slot).is_ok(),
                known,
                "{value} as a transaction failure"
            );

            slot.bus_source = BusSourceKind::Sequence;
            let known = matches!(
                SeqFailureKindWire(value).to_known(),
                Some(kind) if kind != SeqFailureKind::None
            );
            assert_eq!(read(slot).is_ok(), known, "{value} as a sequencer failure");
        }
    }

    /// The zero and the three outcomes that are not conditions of the machine
    /// are refused: a machine does not stand parked on a refused command, and a
    /// slot nothing wrote holds the zero.
    #[test]
    fn a_kind_no_machine_stands_parked_on_is_refused() {
        let mut wire = written(&Fault::PositionFeedbackLost { misses: 2 });
        let slot = wire.validate_mut().unwrap();
        for kind in FaultKind::VARIANTS {
            slot.code = kind;
            if standing().contains(&kind) {
                // The evidence beside it is a missed-read count, so most of the
                // standing kinds refuse on their own evidence here; none of them
                // refuses for being no fault at all.
                assert_ne!(
                    read(slot),
                    Err(FaultError::NoStandingFault(kind)),
                    "{kind} is a fault a machine stands parked on"
                );
            } else {
                assert_eq!(read(slot), Err(FaultError::NoStandingFault(kind)));
            }
        }
    }

    /// Every kind reports under a word of its own, and the eight standing ones
    /// report under the slugs the doctrine names.
    #[test]
    fn a_kind_names_the_slug_it_is_reported_under() {
        let mut seen = std::collections::BTreeSet::new();
        for kind in FaultKind::VARIANTS {
            assert!(seen.insert(slug(kind)), "{kind} shares a slug");
        }
        for fault in every_fault() {
            let expected = match kind(&fault) {
                FaultKind::AntennaObstructed => "antenna_obstructed",
                FaultKind::AntennaServoFault => "antenna_servo_fault",
                FaultKind::HeadObstructed => "head_obstructed",
                FaultKind::HeadServoFault => "head_servo_fault",
                FaultKind::PositionFeedbackLost => "position_feedback_lost",
                FaultKind::MeasuredPoseInvalid => "measured_pose_invalid",
                FaultKind::BusFailure => "bus_failure",
                FaultKind::TorqueOffUnconfirmed => "torque_off_unconfirmed",
                FaultKind::None
                | FaultKind::MoveAbortedEnvelope
                | FaultKind::MoveAbortedStep
                | FaultKind::CommandRejected => panic!("{fault} is a standing fault"),
            };
            assert_eq!(slug(kind(&fault)), expected);
        }
    }

    /// Nothing is answered for the four values that are not conditions of the
    /// machine, and none of them takes the tick out of service. The eight
    /// faults' own classifications are pinned in `tick.rs`, beside the
    /// conditions they are decisions about.
    #[test]
    fn what_is_not_a_fault_is_answered_with_nothing() {
        for kind in [
            FaultKind::None,
            FaultKind::MoveAbortedEnvelope,
            FaultKind::MoveAbortedStep,
            FaultKind::CommandRejected,
        ] {
            assert_eq!(response(kind), ResponseKind::None, "{kind}");
            assert!(!latches(kind), "{kind}");
        }
    }

    /// A sequencer's verdict keeps the servo it happened at and the name of the
    /// failure, for every failure a sequencer raises.
    #[test]
    fn a_sequencers_verdict_keeps_the_servo_it_happened_at() {
        for error in every_sequencer_failure() {
            let wire = written(&Fault::BusFailure {
                source: BusFailureSource::Sequence(error),
            });
            let slot = wire.validate().unwrap();
            assert_eq!(slot.servo_id, error.context().id);
            assert_eq!(
                slot.bus_failure_kind,
                SeqFailureKindWire::from(error.kind()).0
            );
            read(slot).expect("every named failure restores");
        }
    }
}
