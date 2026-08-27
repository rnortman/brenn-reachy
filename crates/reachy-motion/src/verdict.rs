//! A stopped sequence's verdict, in the schema a host keeps it in.
//!
//! [`SeqError`] is one shape per way a sequence can stop, each carrying its own
//! evidence, and a fixed-layout slot has no union to hold them in.
//! [`SeqFailureSnap`] is that union written out — one field per thing a verdict
//! can carry, keyed by
//! [`SeqFailureKind`] — and this module is the only place the two forms meet.
//!
//! There is no third representation. [`write`] assigns the schema's own fields
//! through the validated view, [`read`] reads them back, and the host validates
//! once at the boundary and hands the view down. What the fields mean is stated
//! at the declaration, in `motion/seq.clk`; what this module adds is the two
//! judgements a slot's numbers cannot make for themselves:
//!
//! - **Which fields are meaningful is the kind's business.** Nothing infers a
//!   failure from the payload, so [`write`] blanks every field it is not filling
//!   and a blank field is never read back as an observation.
//! - **Whether the evidence suits the kind** is [`read`]'s, asked once here
//!   rather than again in every host of a sequencer. A verdict that names no
//!   failure, no phase, or a mismatch whose *wanted* side names no shape at all
//!   is refused rather than repaired: every repair available would report a
//!   failure of a register, a servo or a solve that nobody observed. The side
//!   the driver answered on is not held to that: a value of no shape is what
//!   the sequencers raise a shape refusal about in the first place, so it
//!   crosses as it stands rather than being read back as a corrupt slot.
//!
//! The no-failure zero is a number the vocabulary names, so it reads back
//! without complaint at the boundary and [`read`] refuses it as
//! [`VerdictError::NoFailure`] — a caller asking for the failure a slot holds has
//! already decided one is there.

use thiserror::Error;

use crate::joints::{JointRef, ROW_COUNT, rows_of, write_rows};
use crate::seq::{
    AbsentSet, AnswerShape, RegId, SeqError, SeqFailureKind, SeqStepKind, StepContext,
};
use crate::snap::{
    FkFailureKind, FkFieldError, duration_from_nanos, duration_nanos, fk_cause, fk_fields,
};
use crate::value::{self, ShapeName, Value, ValueShape};
use brenn_reachy__hardware__dynamixel__registers_clk_rs::ValueShapeWire;
use brenn_reachy__motion__seq_clk_rs::{AnswerShapeWire, SeqFailureSnap};
use clockwork_rs::Duration as SlotDuration;

/// Numbers in a verdict slot that name no sequencer failure.
///
/// Only reachable from fields that were assembled rather than written by
/// [`write`] — every verdict a sequencer produces reads back — which in practice
/// means a slot holding bytes no verdict ever wrote.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum VerdictError {
    /// An angle or a voltage that is not a number, which is the one value this
    /// stack never carries onward.
    #[error("a {} in the slot is not a number", ShapeName(*.0))]
    NonFiniteValue(ValueShape),
    /// More servos absent than the bus has.
    #[error("{0} servos did not answer a ping, on a bus of {count}", count = ROW_COUNT)]
    TooManyAbsent(u8),
    /// A verdict that names no failure, which is what a running phase and a slot
    /// nothing wrote hold.
    #[error("the fields name no failure at all")]
    NoFailure,
    /// A failure that names no phase. Every failure has a context, so a phase is
    /// always meaningful; a verdict filed under no phase sends somebody looking
    /// at no half of a bring-up.
    #[error("the failure names no phase it happened in")]
    NoStep,
    /// A mismatch of answer shapes with no shape on one side of it.
    #[error("the step and the driver disagreed about the answer, but one side names no shape")]
    NoAnswerShape,
    /// A mismatch whose wanted side names no shape, or holds no value to
    /// compare. What a step wanted is what it asked the bus for, so a mismatch
    /// against nothing on that side was never observed.
    #[error("the step and the driver disagreed, but the step itself wanted nothing")]
    NoValueShape,
    /// A solve failure whose code and numbers name none.
    #[error("the angles place no pose, but no solve failure is named: {0}")]
    Solve(#[from] FkFieldError),
    /// A wait past what the slot's nanosecond count reaches, or a count that is
    /// not a length of time.
    #[error("the wait in the slot is not one: {0}")]
    Wait(#[from] crate::snap::DurationError),
}

impl VerdictError {
    /// This refusal as one small number, for a row that has a slot for a number
    /// and none for a sentence.
    ///
    /// The narration of a verdict nobody can read is the case with the least
    /// evidence in it, so what little there is travels: a reader learns whether
    /// the slot held no failure at all, a failure under no phase, or evidence
    /// that does not suit the kind beside it. Zero is not among the answers --
    /// it is what a row carries when nothing wrote this number.
    ///
    /// Numbering is append-only, for the reason every vocabulary here is: a
    /// recorded row and a running build have to have the number in common.
    #[must_use]
    pub const fn code(self) -> u8 {
        match self {
            VerdictError::NoFailure => 1,
            VerdictError::NoStep => 2,
            VerdictError::NonFiniteValue(_) => 3,
            VerdictError::TooManyAbsent(_) => 4,
            VerdictError::NoAnswerShape => 5,
            VerdictError::NoValueShape => 6,
            VerdictError::Solve(_) => 7,
            VerdictError::Wait(_) => 8,
        }
    }
}

/// What a slot whose bytes are not a valid message at all reads as.
///
/// Not a [`VerdictError`] -- the refusal comes from the schema's own validation,
/// before anything here looks at a field -- but it shares the numbering, because
/// a row carrying one of these numbers carries no second field saying which
/// vocabulary it came from.
pub const BYTES_UNREADABLE: u8 = 9;

/// The one number a sequencer failure is worth stating beside its kind.
///
/// One per kind, and zero for the kinds whose whole evidence is that nothing
/// came back: there is no headline for a silence. Lossy on purpose where a
/// failure carries more than one number -- nine voltages become the lowest, a
/// set of silent servos becomes its size -- because a row of "which kind, which
/// servo, roughly how bad" is what a fixed-layout report has room for and the
/// verdict itself answers the rest.
///
/// Here rather than in a host: this module is where the two forms of a verdict
/// meet, and a projection of one of them living in a binary is a third form that
/// the next host to want it would copy.
#[must_use]
pub fn headline(failure: &SeqError) -> f64 {
    match failure {
        SeqError::Refused { code, .. } => f64::from(*code),
        SeqError::UnhealthyServo { bits, .. } => f64::from(*bits),
        SeqError::VerifyMismatch { read_back, .. } => value::headline(*read_back),
        SeqError::ProvisionMismatch { observed, .. } => value::headline(*observed),
        SeqError::WrongValue { observed, .. } => f64::from(ValueShapeWire::from(*observed).0),
        SeqError::WrongAnswer { observed, .. } => f64::from(AnswerShapeWire::from(*observed).0),
        // The evidence, non-finite included: what makes this failure the failure
        // it is, is that the number places no angle.
        SeqError::UnplaceableAngle { angle, .. } => *angle,
        SeqError::AbsentServos { absent, .. } => {
            f64::from(u8::try_from(absent.count()).unwrap_or(u8::MAX))
        }
        SeqError::IdentityMismatch { model, .. } => f64::from(*model),
        SeqError::VoltageLow { lowest, .. } | SeqError::SupplyBelowFloor { lowest, .. } => *lowest,
        SeqError::RestPoseImplausible { cause, .. }
        | SeqError::PinnedPoseUnsolvable { cause, .. } => {
            let (_, first, _) = fk_fields(*cause);
            first
        }
        SeqError::NoAnswer { .. }
        | SeqError::DriverRefused { .. }
        | SeqError::WireCorrupt { .. }
        | SeqError::PendingUnreadable { .. }
        | SeqError::VerdictUnreadable { .. }
        | SeqError::ClockOutOfRange { .. }
        | SeqError::RecordUnreadable { .. }
        | SeqError::RecordAbsent { .. } => 0.0,
    }
}

/// Write `error` into the fields a verdict slot holds one in.
///
/// Every field is assigned, the blanks included, so a slot carrying an earlier
/// verdict is left describing this one and nothing else.
///
/// # Errors
///
/// [`VerdictError::NonFiniteValue`] for a register value that will not cross,
/// which today means a non-finite angle or voltage, and
/// [`VerdictError::Wait`] for a wait past what the slot's count reaches. Both are
/// unreachable from a verdict a sequencer produced. On a refusal the slot is
/// blanked rather than left half-way between two verdicts: a kind with none of
/// its own evidence beside it would read back as a failure whose numbers nobody
/// observed.
pub fn write(out: &mut SeqFailureSnap, error: &SeqError) -> Result<(), VerdictError> {
    let refusal = fill(out, error);
    if refusal.is_err() {
        blank(out);
    }
    refusal
}

/// The fields, filled in, or the refusal that stopped part-way through them.
fn fill(out: &mut SeqFailureSnap, error: &SeqError) -> Result<(), VerdictError> {
    blank(out);

    let context = error.context();
    out.kind = error.kind();
    out.step = context.step;
    out.servo_id = context.id;
    out.reg = context.reg.unwrap_or(RegId::None);

    match *error {
        SeqError::NoAnswer { .. }
        | SeqError::DriverRefused { .. }
        | SeqError::WireCorrupt { .. }
        | SeqError::PendingUnreadable { .. }
        | SeqError::VerdictUnreadable { .. }
        | SeqError::ClockOutOfRange { .. }
        | SeqError::RecordUnreadable { .. }
        | SeqError::RecordAbsent { .. } => {}
        SeqError::Refused { code, .. } => out.status_code = code,
        SeqError::VerifyMismatch {
            expected,
            read_back,
            ..
        } => values(out, expected, read_back)?,
        SeqError::ProvisionMismatch {
            expected, observed, ..
        } => values(out, expected, observed)?,
        SeqError::WrongAnswer {
            expected, observed, ..
        } => {
            out.expected_answer = expected;
            out.observed_answer = observed;
        }
        SeqError::WrongValue {
            expected, observed, ..
        } => {
            out.expected_kind = expected;
            out.observed_kind = observed;
        }
        SeqError::UnplaceableAngle { joint, angle, .. } => {
            out.joint = joint;
            // Bit for bit, a non-finite reading included: the reading that
            // places no angle is the whole evidence of that failure.
            out.angle = angle;
        }
        SeqError::AbsentServos { absent, .. } => {
            out.absent_ids[..absent.count()].copy_from_slice(absent.ids());
            out.absent_count = absent_row(absent.count());
        }
        SeqError::IdentityMismatch {
            model, expected, ..
        } => {
            out.model = model;
            out.expected_model = expected;
        }
        SeqError::VoltageLow {
            readings,
            lowest,
            limit,
            waited,
            ..
        } => {
            write_rows(&mut out.readings, &readings);
            out.lowest = lowest;
            out.limit = limit;
            out.waited = SlotDuration::from_nanos(duration_nanos(waited)?);
        }
        SeqError::SupplyBelowFloor {
            readings,
            lowest,
            limit,
            ..
        } => {
            write_rows(&mut out.readings, &readings);
            out.lowest = lowest;
            out.limit = limit;
        }
        SeqError::UnhealthyServo { bits, .. } => out.error_bits = bits,
        SeqError::RestPoseImplausible { cause, .. }
        | SeqError::PinnedPoseUnsolvable { cause, .. } => {
            let (fk_code, fk_a, fk_b) = fk_fields(cause);
            out.fk_code = fk_code;
            out.fk_a = fk_a;
            out.fk_b = fk_b;
        }
    }
    Ok(())
}

/// The verdict those fields describe.
///
/// # Errors
///
/// [`VerdictError`], one variant per way a slot's numbers can fail to name a
/// failure: no failure at all, no phase, more absent servos than the bus has, a
/// mismatch whose wanted side names no shape, a value that is not a number, a
/// solve failure with no failure in it.
pub fn read(slot: &SeqFailureSnap) -> Result<SeqError, VerdictError> {
    // Ahead of `context`, so that fields naming neither a failure nor a phase --
    // a slot nobody wrote -- refuse as no failure rather than as no phase, which
    // is the diagnostic that fits the common case.
    if slot.kind == SeqFailureKind::None {
        return Err(VerdictError::NoFailure);
    }
    let context = context(slot)?;
    Ok(match slot.kind {
        SeqFailureKind::None => unreachable!("the guard above answers the zero"),
        SeqFailureKind::NoAnswer => SeqError::NoAnswer { context },
        SeqFailureKind::DriverRefused => SeqError::DriverRefused { context },
        SeqFailureKind::WireCorrupt => SeqError::WireCorrupt { context },
        SeqFailureKind::PendingUnreadable => SeqError::PendingUnreadable { context },
        SeqFailureKind::VerdictUnreadable => SeqError::VerdictUnreadable { context },
        SeqFailureKind::ClockOutOfRange => SeqError::ClockOutOfRange { context },
        SeqFailureKind::RecordUnreadable => SeqError::RecordUnreadable { context },
        SeqFailureKind::RecordAbsent => SeqError::RecordAbsent { context },
        SeqFailureKind::Refused => SeqError::Refused {
            context,
            code: slot.status_code,
        },
        SeqFailureKind::VerifyMismatch => SeqError::VerifyMismatch {
            context,
            expected: expected(slot)?,
            read_back: observed(slot)?,
        },
        SeqFailureKind::ProvisionMismatch => SeqError::ProvisionMismatch {
            context,
            expected: expected(slot)?,
            observed: observed(slot)?,
        },
        SeqFailureKind::WrongAnswer => SeqError::WrongAnswer {
            context,
            expected: answer_named(slot.expected_answer)?,
            observed: answer_named(slot.observed_answer)?,
        },
        SeqFailureKind::WrongValue => SeqError::WrongValue {
            context,
            expected: shape_named(slot.expected_kind)?,
            // Taken as it stands, the shapeless zero included: a driver that
            // answered a value of no shape is what this failure is raised
            // about, so refusing it here would lose the failure that named it.
            observed: slot.observed_kind,
        },
        SeqFailureKind::UnplaceableAngle => SeqError::UnplaceableAngle {
            context,
            joint: slot.joint,
            angle: slot.angle,
        },
        SeqFailureKind::AbsentServos => SeqError::AbsentServos {
            context,
            absent: absent_set(slot)?,
        },
        SeqFailureKind::IdentityMismatch => SeqError::IdentityMismatch {
            context,
            model: slot.model,
            expected: slot.expected_model,
        },
        SeqFailureKind::VoltageLow => SeqError::VoltageLow {
            context,
            readings: rows_of(&slot.readings),
            lowest: slot.lowest,
            limit: slot.limit,
            waited: duration_from_nanos(slot.waited.as_nanos())?,
        },
        SeqFailureKind::SupplyBelowFloor => SeqError::SupplyBelowFloor {
            context,
            readings: rows_of(&slot.readings),
            lowest: slot.lowest,
            limit: slot.limit,
        },
        SeqFailureKind::UnhealthyServo => SeqError::UnhealthyServo {
            context,
            bits: slot.error_bits,
        },
        SeqFailureKind::RestPoseImplausible => SeqError::RestPoseImplausible {
            context,
            cause: fk_cause(slot.fk_code, slot.fk_a, slot.fk_b)?,
        },
        SeqFailureKind::PinnedPoseUnsolvable => SeqError::PinnedPoseUnsolvable {
            context,
            cause: fk_cause(slot.fk_code, slot.fk_a, slot.fk_b)?,
        },
    })
}

/// Every field at its "not part of this failure" value, which is what a field the
/// kind does not name must read back as.
fn blank(out: &mut SeqFailureSnap) {
    out.kind = SeqFailureKind::None;
    out.step = SeqStepKind::None;
    out.servo_id = 0;
    out.reg = RegId::None;
    out.status_code = 0;
    out.error_bits = 0;
    out.joint = JointRef::None;
    out.angle = 0.0;
    out.absent_ids = [0; ROW_COUNT];
    out.absent_count = 0;
    out.model = 0;
    out.expected_model = 0;
    out.expected_kind = ValueShape::None;
    out.expected_value = 0;
    out.observed_kind = ValueShape::None;
    out.observed_value = 0;
    out.expected_answer = AnswerShape::NotApplicable;
    out.observed_answer = AnswerShape::NotApplicable;
    write_rows(&mut out.readings, &[0.0; ROW_COUNT]);
    out.lowest = 0.0;
    out.limit = 0.0;
    out.waited = SlotDuration::from_nanos(0);
    out.fk_code = FkFailureKind::NotApplicable;
    out.fk_a = 0.0;
    out.fk_b = 0.0;
}

/// Where the failure happened.
fn context(slot: &SeqFailureSnap) -> Result<StepContext, VerdictError> {
    if slot.step == SeqStepKind::None {
        return Err(VerdictError::NoStep);
    }
    if slot.reg == RegId::None {
        return Ok(StepContext::servo(slot.step, slot.servo_id));
    }
    Ok(StepContext::reg(slot.step, slot.servo_id, slot.reg))
}

/// The two values a mismatch compares, as the shape and the bits each is.
///
/// Nothing is converted — the pair a sequencer holds is the pair the slot holds —
/// so the only refusal is a value that never leaves this stack.
fn values(out: &mut SeqFailureSnap, expected: Value, observed: Value) -> Result<(), VerdictError> {
    for value in [expected, observed] {
        if !value.is_carriable() {
            return Err(VerdictError::NonFiniteValue(value.shape()));
        }
    }
    out.expected_kind = expected.shape();
    out.expected_value = expected.bits();
    out.observed_kind = observed.shape();
    out.observed_value = observed.bits();
    Ok(())
}

/// What was wanted, refusing a number that is not one.
fn expected(slot: &SeqFailureSnap) -> Result<Value, VerdictError> {
    checked(value::carried(slot.expected_kind, slot.expected_value))
}

/// What arrived, which may be no value at all.
///
/// A driver that answered a value of no shape is an observation a sequencer
/// makes and files, so the shapeless zero crosses here; a number that is not one
/// still does not.
fn observed(slot: &SeqFailureSnap) -> Result<Value, VerdictError> {
    let value = value::carried(slot.observed_kind, slot.observed_value);
    if !value.is_carriable() {
        return Err(VerdictError::NonFiniteValue(value.shape()));
    }
    Ok(value)
}

/// The servos that did not answer, rebuilt from the IDs and the count.
fn absent_set(slot: &SeqFailureSnap) -> Result<AbsentSet, VerdictError> {
    let count = usize::from(slot.absent_count);
    if count > ROW_COUNT {
        return Err(VerdictError::TooManyAbsent(slot.absent_count));
    }
    let mut flags = [false; ROW_COUNT];
    for flag in &mut flags[..count] {
        *flag = true;
    }
    Ok(AbsentSet::new(&slot.absent_ids, &flags))
}

/// The answer shape a field names, refusing the zero that names none: a mismatch
/// between shapes nobody can name says nothing about what the driver did wrong.
fn answer_named(shape: AnswerShape) -> Result<AnswerShape, VerdictError> {
    if shape == AnswerShape::NotApplicable {
        return Err(VerdictError::NoAnswerShape);
    }
    Ok(shape)
}

/// The shape a step wanted, for a failure that is about the shapes alone,
/// refusing the zero that names none: every such step asked for a shape.
fn shape_named(shape: ValueShape) -> Result<ValueShape, VerdictError> {
    if shape == ValueShape::None {
        return Err(VerdictError::NoValueShape);
    }
    Ok(shape)
}

/// The value a mismatch wanted, refused where the fields hold none to compare —
/// the failure is then about the shapes and is a different verdict — or a number
/// that is not one, which nothing here passes onward.
fn checked(value: Value) -> Result<Value, VerdictError> {
    if value.shape() == ValueShape::None {
        return Err(VerdictError::NoValueShape);
    }
    if !value.is_carriable() {
        return Err(VerdictError::NonFiniteValue(value.shape()));
    }
    Ok(value)
}

/// How many servos are in an absent set, as the field that holds it.
///
/// A set is built from the nine bus rows, so it cannot hold more, and the cast
/// is total for every count one can carry.
fn absent_row(count: usize) -> u8 {
    u8::try_from(count).unwrap_or(u8::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::time::Duration;

    use crate::joints::ROWS;
    use crate::seq::{answer, failure, reg};
    use crate::testutil::{every_sequencer_failure, failure_context};
    use crate::value;
    use brenn_reachy__motion__seq_clk_rs::SeqFailureSnapWire;
    use reachy_kin::FkError;

    /// `error` written into a slot of its own, as a host hands one down: cleared
    /// once, then filled by assignment.
    fn wrote(error: &SeqError) -> SeqFailureSnapWire {
        let mut wire = SeqFailureSnapWire::new();
        write(wire.clear_valid(), error).expect("a reachable verdict crosses");
        wire
    }

    /// The validated view of a slot a verdict was written into. Every field
    /// `write` assigns is a value the schema declares, so this cannot refuse.
    fn valid(wire: &SeqFailureSnapWire) -> &SeqFailureSnap {
        wire.validate().expect("a written verdict validates")
    }

    /// A row that says "unreadable" and nothing else is the least a parked
    /// machine can be told, so the refusal's own number travels: every one of
    /// them is distinct, none is the zero a field nobody wrote carries, and none
    /// collides with the bytes-unreadable number that shares the numbering.
    #[test]
    fn every_refusal_has_a_number_of_its_own() {
        let refusals = [
            VerdictError::NoFailure,
            VerdictError::NoStep,
            VerdictError::NonFiniteValue(ValueShape::Radians),
            VerdictError::TooManyAbsent(11),
            VerdictError::NoAnswerShape,
            VerdictError::NoValueShape,
            VerdictError::Solve(FkFieldError::NoSolveFailure),
            VerdictError::Wait(crate::snap::DurationError::Negative(-1)),
        ];
        let mut seen = Vec::new();
        for refusal in refusals {
            let code = refusal.code();
            assert_ne!(code, 0, "{refusal} reads as a field nobody wrote");
            assert_ne!(
                code, BYTES_UNREADABLE,
                "{refusal} reads as unreadable bytes"
            );
            assert!(!seen.contains(&code), "{refusal} shares a number");
            seen.push(code);
        }
    }

    /// The common case has a number too: a slot nobody ever wrote.
    #[test]
    fn a_slot_nobody_wrote_says_so_by_number() {
        let wire = SeqFailureSnapWire::new();
        let refusal = read(wire.validate().expect("zeroes validate"))
            .expect_err("a slot nobody wrote names no failure");
        assert_eq!(refusal, VerdictError::NoFailure);
        assert_eq!(
            refusal.code(),
            1,
            "the number a row carries for the commonest refusal of all"
        );
    }

    /// The one number each kind is worth stating beside itself, per kind.
    ///
    /// This is the whole of what a parked run's narration says about how bad it
    /// was, so a field picked from the wrong side of a failure -- a floor for a
    /// reading, a derivative term for a proportional one -- mis-narrates every
    /// parked run and nothing else in the tree has a view. The zeroes are
    /// asserted rather than defaulted: the kinds whose whole evidence is that
    /// nothing came back have no headline, and saying so here is what keeps
    /// "nothing to state" apart from "nobody wrote this field".
    #[test]
    fn every_failure_states_the_one_number_it_is_worth_stating() {
        let context = failure_context();
        let readings = [11.4, 11.5, 11.6, 11.7, 11.8, 11.9, 12.0, 12.1, 12.2];
        let table: Vec<(SeqError, f64)> = vec![
            (SeqError::Refused { context, code: 7 }, 7.0),
            (
                SeqError::UnhealthyServo {
                    context,
                    bits: 0b0010_0001,
                },
                33.0,
            ),
            (
                SeqError::VerifyMismatch {
                    context,
                    expected: value::radians(0.5),
                    read_back: value::radians(0.25),
                },
                0.25,
            ),
            (
                SeqError::ProvisionMismatch {
                    context,
                    expected: value::u16(3),
                    observed: value::u16(4),
                },
                4.0,
            ),
            (
                SeqError::WrongValue {
                    context,
                    expected: ValueShape::U8,
                    observed: ValueShape::U16,
                },
                f64::from(ValueShapeWire::from(ValueShape::U16).0),
            ),
            (
                SeqError::WrongAnswer {
                    context,
                    expected: answer::AnswerShape::Value,
                    observed: answer::AnswerShape::Pinged,
                },
                f64::from(AnswerShapeWire::from(answer::AnswerShape::Pinged).0),
            ),
            (
                SeqError::UnplaceableAngle {
                    context,
                    joint: ROWS[0],
                    angle: -3.5,
                },
                -3.5,
            ),
            (
                SeqError::AbsentServos {
                    context,
                    absent: AbsentSet::new(
                        &[10, 11, 12, 13, 14, 15, 16, 17, 18],
                        &[true, true, false, false, false, false, false, false, false],
                    ),
                },
                2.0,
            ),
            (
                SeqError::IdentityMismatch {
                    context,
                    model: 1_060,
                    expected: 1_020,
                },
                1_060.0,
            ),
            (
                SeqError::VoltageLow {
                    context,
                    readings,
                    lowest: 11.4,
                    limit: 11.9,
                    waited: Duration::from_secs(3),
                },
                11.4,
            ),
            (
                SeqError::SupplyBelowFloor {
                    context,
                    readings,
                    lowest: 10.2,
                    limit: 11.9,
                },
                10.2,
            ),
            (
                SeqError::RestPoseImplausible {
                    context,
                    cause: FkError::NoConvergence {
                        iters: 12,
                        residual: 0.004,
                    },
                },
                12.0,
            ),
            (
                SeqError::PinnedPoseUnsolvable {
                    context,
                    cause: FkError::WrongAssemblyMode {
                        cone_deg: 41.0,
                        z: 0.1,
                    },
                },
                41.0,
            ),
            (SeqError::NoAnswer { context }, 0.0),
            (SeqError::DriverRefused { context }, 0.0),
            (SeqError::WireCorrupt { context }, 0.0),
            (SeqError::PendingUnreadable { context }, 0.0),
            (SeqError::VerdictUnreadable { context }, 0.0),
            (SeqError::ClockOutOfRange { context }, 0.0),
            (SeqError::RecordUnreadable { context }, 0.0),
            (SeqError::RecordAbsent { context }, 0.0),
        ];
        assert_eq!(
            table.len(),
            failure::raised().count(),
            "one per kind, so this sweep is exhaustive by count as well"
        );
        for (error, expected) in &table {
            assert_eq!(headline(error), *expected, "{error}");
        }
    }

    /// A blank verdict of a kind, for the refusal cases to spoil one field of.
    fn blank(kind: SeqFailureKind) -> SeqFailureSnapWire {
        let mut wire = wrote(&SeqError::NoAnswer {
            context: failure_context(),
        });
        wire.validate_mut()
            .expect("a written verdict validates")
            .kind = kind;
        wire
    }

    /// Both directions of the verdict crossing, compared as written rather than
    /// by `PartialEq`: a verdict carrying a NaN reading is not equal to itself,
    /// and that verdict — an angle that is not an angle — is one of the seventeen.
    fn crosses(error: &SeqError) -> String {
        let wire = wrote(error);
        let back = read(valid(&wire)).expect("and comes back");
        format!("{back:?}")
    }

    /// Every failure the sequencers raise crosses whole. The table is one per
    /// kind, so this is exhaustive by count as well as by construction.
    #[test]
    fn every_failure_crosses_whole() {
        let table = every_sequencer_failure();
        assert_eq!(table.len(), failure::raised().count());
        for error in &table {
            assert_eq!(crosses(error), format!("{error:?}"), "{error}");
        }
    }

    /// A verdict keeps the kind it was written from, so the key the payload
    /// fields are read under is never the neighbouring failure's.
    #[test]
    fn a_verdict_names_the_failure_it_was_written_from() {
        for error in &every_sequencer_failure() {
            let wire = wrote(error);
            assert_eq!(valid(&wire).kind, error.kind(), "{error}");
        }
    }

    /// Every register a failure can name, and every shape a value comes in,
    /// through the two mismatches that carry values. The register numbering and
    /// the value carriage are `txn`'s and `value`'s, so what this pins is that a
    /// slot's two value fields are not swapped and that the shapes survive being
    /// stored beside each other.
    #[test]
    fn every_register_and_every_value_shape_crosses() {
        let shapes = [
            value::u8(0x5a),
            value::u16(0x1234),
            value::u32(0x1234_5678),
            value::i32(-1_234_567),
            value::radians(-1.234_567_890_123),
            value::volts(11.7),
            value::gains(900, 7, 33),
        ];
        for reg in reg::named() {
            let context = StepContext::reg(SeqStepKind::Provision, 14, reg);
            for expected in shapes {
                for observed in shapes {
                    for error in [
                        SeqError::VerifyMismatch {
                            context,
                            expected,
                            read_back: observed,
                        },
                        SeqError::ProvisionMismatch {
                            context,
                            expected,
                            observed,
                        },
                    ] {
                        assert_eq!(crosses(&error), format!("{error:?}"), "{error}");
                    }
                }
            }
        }
    }

    /// A failure in a phase that concerns no register crosses as one, rather
    /// than acquiring the first register in the table.
    #[test]
    fn a_failure_that_names_no_register_names_none_on_the_way_back() {
        let error = SeqError::NoAnswer {
            context: StepContext::servo(SeqStepKind::Presence, 17),
        };
        let wire = wrote(&error);
        assert_eq!(valid(&wire).reg, RegId::None);
        assert_eq!(read(valid(&wire)), Ok(error));
    }

    /// Every phase, on the failure that carries nothing but its context, so the
    /// phase is the only thing under test.
    #[test]
    fn every_phase_a_failure_can_happen_in_crosses() {
        for step in SeqStepKind::VARIANTS {
            let error = SeqError::WireCorrupt {
                context: StepContext::servo(step, 11),
            };
            let wire = wrote(&error);
            if step == SeqStepKind::None {
                // A context naming no phase is not a place a failure happened,
                // and the slot says so rather than restoring one.
                assert_eq!(read(valid(&wire)), Err(VerdictError::NoStep));
                continue;
            }
            assert_eq!(read(valid(&wire)), Ok(error));
        }
    }

    /// Every pairing of answer shapes a driver-wiring mistake can produce, both
    /// sides, so a crossing that read one field for both would fail on the
    /// fifty-six unequal pairs.
    #[test]
    fn every_pairing_of_answer_shapes_crosses() {
        for expected in every_answer_shape() {
            for observed in every_answer_shape() {
                let error = SeqError::WrongAnswer {
                    context: failure_context(),
                    expected,
                    observed,
                };
                let wire = wrote(&error);
                assert_eq!(read(valid(&wire)), Ok(error));
            }
        }
    }

    /// Every pairing of value shapes, likewise — the failure that is about the
    /// shapes alone, whose value bytes stay blank.
    #[test]
    fn every_pairing_of_value_shapes_crosses_with_no_value_beside_it() {
        for expected in every_value_shape() {
            for observed in every_value_shape() {
                let error = SeqError::WrongValue {
                    context: failure_context(),
                    expected,
                    observed,
                };
                let wire = wrote(&error);
                assert_eq!(valid(&wire).expected_value, 0);
                assert_eq!(valid(&wire).observed_value, 0);
                assert_eq!(read(valid(&wire)), Ok(error));
            }
        }
    }

    /// A presence sweep's absent set at every size it can be, including the one
    /// silent servo and the whole bus. The set is rebuilt from the IDs and the
    /// count, so a size the rebuild got wrong would name the wrong servos to
    /// whoever goes to look at them.
    #[test]
    fn an_absent_set_crosses_at_every_size() {
        let ids = [10, 11, 12, 13, 14, 15, 16, 17, 18];
        for count in 0..=ROW_COUNT {
            let mut flags = [false; ROW_COUNT];
            for flag in &mut flags[..count] {
                *flag = true;
            }
            let absent = AbsentSet::new(&ids, &flags);
            let error = SeqError::AbsentServos {
                context: failure_context(),
                absent,
            };
            let wire = wrote(&error);
            assert_eq!(usize::from(valid(&wire).absent_count), count);
            let back = read(valid(&wire)).expect("comes back");
            assert_eq!(back, error);
            assert_eq!(back.to_string(), error.to_string(), "{count} absent");
        }
    }

    /// A set built from servos scattered along the bus, so what is carried is
    /// the IDs rather than the first `count` of them.
    #[test]
    fn a_scattered_absent_set_keeps_the_servos_it_named() {
        let absent = AbsentSet::new(
            &[10, 11, 12, 13, 14, 15, 16, 17, 18],
            &[false, true, false, false, true, false, false, false, true],
        );
        let error = SeqError::AbsentServos {
            context: failure_context(),
            absent,
        };
        let wire = wrote(&error);
        let back = read(valid(&wire)).expect("comes back");
        assert_eq!(back, error);
        assert_eq!(
            back.to_string(),
            "provisioning of servo 12, operating mode: no answer from servos 11, 14, 18"
        );
    }

    /// Every joint a reading can be about, and the reading itself bit for bit —
    /// the one field here that carries a non-finite number on purpose, because
    /// what the failure reports *is* that the number is not one.
    #[test]
    fn an_unplaceable_reading_crosses_bit_for_bit() {
        for joint in ROWS {
            for angle in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY, -0.0, 1.5] {
                let error = SeqError::UnplaceableAngle {
                    context: failure_context(),
                    joint,
                    angle,
                };
                let wire = wrote(&error);
                assert_eq!(valid(&wire).angle.to_bits(), angle.to_bits());
                let Ok(SeqError::UnplaceableAngle {
                    joint: back_joint,
                    angle: back_angle,
                    ..
                }) = read(valid(&wire))
                else {
                    panic!("an unplaceable angle crosses as one");
                };
                assert_eq!(back_joint, joint);
                assert_eq!(back_angle.to_bits(), angle.to_bits());
            }
        }
    }

    /// Every supply reading, on its own servo. The nine readings are indexed by
    /// bus row and the slot holds them by servo name, so a crossing that got the
    /// order wrong would report the antennas' supply against a crank.
    #[test]
    fn every_supply_reading_crosses_on_its_own_servo() {
        let mut readings = [0.0; ROW_COUNT];
        for (row, reading) in readings.iter_mut().enumerate() {
            *reading = 11.0 + row as f64 / 10.0;
        }
        let error = SeqError::SupplyBelowFloor {
            context: failure_context(),
            readings,
            lowest: 11.0,
            limit: 11.5,
        };
        let wire = wrote(&error);
        assert_eq!(crate::joints::rows_of(&valid(&wire).readings), readings);
        assert_eq!(read(valid(&wire)), Ok(error));
    }

    /// Both solve failures, both numbers, through both of the failures that
    /// carry one.
    #[test]
    fn every_solve_failure_crosses_through_both_of_its_carriers() {
        for cause in [
            FkError::NoConvergence {
                iters: 40,
                residual: 0.004,
            },
            FkError::WrongAssemblyMode {
                cone_deg: 91.5,
                z: -0.021,
            },
        ] {
            for error in [
                SeqError::RestPoseImplausible {
                    context: failure_context(),
                    cause,
                },
                SeqError::PinnedPoseUnsolvable {
                    context: failure_context(),
                    cause,
                },
            ] {
                let wire = wrote(&error);
                assert_eq!(read(valid(&wire)), Ok(error));
            }
        }
    }

    /// A failure that is about none of the payload fields leaves every one of
    /// them at a value that is not an observation: no joint, no servos absent,
    /// no shapes, no solve failure. Nothing reads them without the kind saying
    /// to, and this is what they hold when it does not.
    #[test]
    fn a_failure_that_carries_nothing_leaves_every_payload_field_blank() {
        let wire = wrote(&SeqError::NoAnswer {
            context: failure_context(),
        });
        let snap = valid(&wire);
        assert_eq!(snap.joint, JointRef::None);
        assert_eq!(snap.absent_count, 0);
        assert_eq!(snap.status_code, 0);
        assert_eq!(snap.error_bits, 0);
        assert_eq!(snap.expected_kind, ValueShape::None);
        assert_eq!(snap.observed_kind, ValueShape::None);
        assert_eq!(snap.expected_answer, AnswerShape::NotApplicable);
        assert_eq!(snap.observed_answer, AnswerShape::NotApplicable);
        assert_eq!(snap.fk_code, FkFailureKind::NotApplicable);
        assert_eq!(snap.waited.as_nanos(), 0);
        assert_eq!(crate::joints::rows_of(&snap.readings), [0.0; ROW_COUNT]);
    }

    /// A slot carrying one verdict, written with the next: every field the new
    /// kind does not name is blank again, so nothing of the first is read as
    /// evidence for the second.
    #[test]
    fn a_reused_slot_carries_no_part_of_the_verdict_before_it() {
        let mut wire = wrote(&SeqError::Refused {
            context: StepContext::reg(SeqStepKind::Presence, 3, RegId::TorqueEnable),
            code: 7,
        });
        let plain = SeqError::NoAnswer {
            context: StepContext::servo(SeqStepKind::PoseAndDatum, 4),
        };
        let view = wire.validate_mut().expect("a written verdict validates");
        write(view, &plain).expect("the verdict after it crosses too");
        assert_eq!(view.status_code, 0);
        assert_eq!(view.reg, RegId::None);
        assert_eq!(read(view), Ok(plain));
    }

    /// The gate that does not wait carries no wait, and the gate that does
    /// carries its own. Two failures over the same three readings otherwise, so
    /// the wait is the only thing under test.
    #[test]
    fn only_the_gate_that_waits_carries_a_wait() {
        let readings = [11.4; ROW_COUNT];
        let waiting = SeqError::VoltageLow {
            context: failure_context(),
            readings,
            lowest: 10.0,
            limit: 11.0,
            waited: Duration::from_millis(4200),
        };
        let not = SeqError::SupplyBelowFloor {
            context: failure_context(),
            readings,
            lowest: 10.0,
            limit: 11.0,
        };
        let waited = wrote(&waiting);
        let never = wrote(&not);
        assert_eq!(
            valid(&waited).waited.as_nanos(),
            i64::try_from(Duration::from_millis(4200).as_nanos()).expect("fits"),
        );
        assert_eq!(valid(&never).waited.as_nanos(), 0);
        assert_eq!(read(valid(&waited)), Ok(waiting));
        assert_eq!(read(valid(&never)), Ok(not));
    }

    /// Fields naming no failure, and fields naming neither a failure nor a
    /// phase. Both refuse as no failure: the zero-kind guard runs ahead of the
    /// phase check, so a slot nobody wrote reads as the absence of a verdict
    /// rather than as a verdict filed under no phase.
    #[test]
    fn a_verdict_naming_no_failure_is_refused_as_that() {
        let mut wire = blank(SeqFailureKind::None);
        let view = wire.validate_mut().expect("a written verdict validates");
        assert_eq!(read(view), Err(VerdictError::NoFailure));
        assert_eq!(
            read(view).unwrap_err().to_string(),
            "the fields name no failure at all"
        );

        view.step = SeqStepKind::None;
        assert_eq!(
            read(view),
            Err(VerdictError::NoFailure),
            "the zero kind answers before the phase does"
        );

        let mut phaseless = blank(SeqFailureKind::NoAnswer);
        let view = phaseless
            .validate_mut()
            .expect("a written verdict validates");
        view.step = SeqStepKind::None;
        assert_eq!(read(view), Err(VerdictError::NoStep));
        assert_eq!(
            read(view).unwrap_err().to_string(),
            "the failure names no phase it happened in"
        );
    }

    /// More servos silent than the bus has. The nine are the whole bus, and a
    /// tenth would be a set naming a servo nobody asked.
    #[test]
    fn more_absent_servos_than_the_bus_has_is_refused() {
        let mut wire = blank(SeqFailureKind::AbsentServos);
        let view = wire.validate_mut().expect("a written verdict validates");
        view.absent_count = 10;
        assert_eq!(read(view), Err(VerdictError::TooManyAbsent(10)));
        assert_eq!(
            read(view).unwrap_err().to_string(),
            "10 servos did not answer a ping, on a bus of 9"
        );
    }

    /// A shape mismatch whose wanted side names no shape. The step asked the bus
    /// for a shape, so a verdict where it asked for nothing describes no step.
    #[test]
    fn a_shape_mismatch_wanting_no_shape_is_refused() {
        for (expected, observed) in [
            (AnswerShape::NotApplicable, AnswerShape::Value),
            (AnswerShape::Value, AnswerShape::NotApplicable),
        ] {
            let mut wire = blank(SeqFailureKind::WrongAnswer);
            let view = wire.validate_mut().expect("a written verdict validates");
            view.expected_answer = expected;
            view.observed_answer = observed;
            assert_eq!(read(view), Err(VerdictError::NoAnswerShape));
        }

        let mut wire = blank(SeqFailureKind::WrongValue);
        let view = wire.validate_mut().expect("a written verdict validates");
        view.expected_kind = ValueShape::None;
        view.observed_kind = ValueShape::U8;
        assert_eq!(read(view), Err(VerdictError::NoValueShape));
    }

    /// A value mismatch that wanted no value.
    #[test]
    fn a_value_mismatch_wanting_no_value_is_refused() {
        for kind in [
            SeqFailureKind::VerifyMismatch,
            SeqFailureKind::ProvisionMismatch,
        ] {
            let mut wire = blank(kind);
            let view = wire.validate_mut().expect("a written verdict validates");
            view.expected_kind = ValueShape::None;
            view.observed_kind = ValueShape::U8;
            assert_eq!(
                read(view),
                Err(VerdictError::NoValueShape),
                "a mismatch of values with nothing wanted is about the value, not about the two shapes"
            );
        }
    }

    /// A driver that answered a value of no shape at all crosses as that, in
    /// every failure that carries what arrived.
    ///
    /// This is the refusal the sequencers raise on every read sweep — a value
    /// carrying no shape is exactly what [`crate::value::as_radians`] and its
    /// neighbours refuse — so a verdict slot that would not carry it back would
    /// lose the evidence and leave the state unresumable.
    #[test]
    fn a_driver_answering_no_shape_at_all_crosses_as_that() {
        let context = failure_context();
        for error in [
            SeqError::WrongValue {
                context,
                expected: ValueShape::Radians,
                observed: ValueShape::None,
            },
            SeqError::VerifyMismatch {
                context,
                expected: value::u8(1),
                read_back: value::NONE,
            },
            SeqError::ProvisionMismatch {
                context,
                expected: value::u16(3),
                observed: value::NONE,
            },
        ] {
            let wire = wrote(&error);
            assert_eq!(read(valid(&wire)), Ok(error));
        }
    }

    /// A pose failure whose three fields name no solve failure, and one whose
    /// iteration count is not a count.
    #[test]
    fn a_pose_failure_with_no_solve_failure_in_it_is_refused() {
        for kind in [
            SeqFailureKind::RestPoseImplausible,
            SeqFailureKind::PinnedPoseUnsolvable,
        ] {
            let mut wire = blank(kind);
            let view = wire.validate_mut().expect("a written verdict validates");
            assert_eq!(
                read(view),
                Err(VerdictError::Solve(FkFieldError::NoSolveFailure))
            );
            view.fk_code = FkFailureKind::NoConvergence;
            view.fk_a = 0.5;
            assert_eq!(
                read(view),
                Err(VerdictError::Solve(FkFieldError::NotACount(0.5)))
            );
        }
    }

    /// A non-finite register value does not cross, in either mismatch. Nothing
    /// above the wire line passes one on, and a slot holding one would hand a
    /// driver a goal nobody meant to send.
    #[test]
    fn a_non_finite_register_value_does_not_cross() {
        for value in [value::radians(f64::NAN), value::volts(f64::INFINITY)] {
            let error = SeqError::VerifyMismatch {
                context: failure_context(),
                expected: value,
                read_back: value::u8(0),
            };
            let mut wire = SeqFailureSnapWire::new();
            let slot = wire.clear_valid();
            assert!(matches!(
                write(slot, &error),
                Err(VerdictError::NonFiniteValue(_))
            ));
            // Blank, not a kind with no evidence beside it: the failure this
            // slot could not hold is reported by the stop that wrote it, and a
            // half-written verdict would be read back as an observation.
            assert_eq!(slot.kind, SeqFailureKind::None);
            assert_eq!(read(slot), Err(VerdictError::NoFailure));
        }
    }

    /// Every answer shape a transaction has, the not-applicable zero excluded.
    fn every_answer_shape() -> Vec<AnswerShape> {
        answer::shapes().collect()
    }

    /// Every shape a register value has, the absence excluded.
    fn every_value_shape() -> Vec<ValueShape> {
        value::shapes().collect()
    }
}
