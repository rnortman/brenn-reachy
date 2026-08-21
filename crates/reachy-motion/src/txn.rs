//! One bus transaction, as the vocabulary declares it.
//!
//! A sequencer asks for one transaction at a time and is handed the previous
//! one's result. What it asks for is a [`BusTxnWire`] — the vocabulary's own
//! record, the same bytes a slot holds and the same bytes a datagram will carry
//! — built and read here and nowhere else. There is no second form: a
//! transaction a sequencer holds and a transaction in a slot are one value.
//!
//! # What the fields say, and in what units
//!
//! Engineering units, always: radians, volts, and the gain span as three
//! numbers, in the eight bytes [`crate::value`] describes. Counts never appear —
//! the count conversion is the bus layer's, below this — which is what lets this
//! crate hold no register address and no width.
//!
//! Which is also why a register rides as a [`RegId`] rather than as its address
//! on the servo. The address is the bus layer's table; the name is the
//! vocabulary's, one number per register stated in the schema, so a transaction
//! written by one build names the same register in the next.
//!
//! # Where a record is built, and where it is read
//!
//! Every sequencer builds into its own state: `set_ping`, `set_read_reg`,
//! `set_write_reg_verified` and `set_none` write a caller's `&mut BusTxn`
//! through the validated view, so nothing is built beside the record and copied
//! into it, and [`fields`] and [`held`] read it back the same way. [`none`] and
//! [`ping`] answer a fresh [`BusTxnWire`], and [`read`], [`active`] and [`op`]
//! read one off the wire type — the boundary shape, for a host or a fixture
//! holding a record outside any state. Both are one builder underneath.
//!
//! # Reading one back
//!
//! [`read`] is the one place a transaction's own fields are read. It takes the
//! record through its validated view, so an operation or a value shape outside
//! this build's vocabulary is a typed refusal
//! ([`SeqError::PendingUnreadable`]) at that one point rather than a guess
//! carried onward.

pub use brenn_reachy__motion__bus_txn_clk_rs::{AuxOpKind, BusTxn, BusTxnWire};

use crate::seq::{RegId, SeqError, SeqStepKind, StepContext};
use crate::value::{self, Value};

/// No transaction: what a sequencer holds on the step after a wait, and before
/// its first one.
#[must_use]
pub fn none() -> BusTxnWire {
    BusTxnWire::new()
}

/// Ask whether the servo at `id` answers at all.
#[must_use]
pub fn ping(id: u8) -> BusTxnWire {
    wire(AuxOpKind::Ping, id, RegId::None, value::NONE)
}

/// Ask whether the servo at `id` answers, in the record `out`.
///
/// The `set_*` family writes where a sequencer's outstanding transaction lives —
/// a field of its state — through the validated view, so nothing is built beside
/// the record and copied into it.
pub fn set_ping(out: &mut BusTxn, id: u8) {
    build(out, AuxOpKind::Ping, id, RegId::None, value::NONE);
}

/// Read one register, in the record `out`.
pub fn set_read_reg(out: &mut BusTxn, id: u8, reg: RegId) {
    build(out, AuxOpKind::ReadReg, id, reg, value::NONE);
}

/// Write one register and read it back, in the record `out`.
pub fn set_write_reg_verified(out: &mut BusTxn, id: u8, reg: RegId, value: Value) {
    build(out, AuxOpKind::WriteRegVerified, id, reg, value);
}

/// Nothing outstanding, in the record `out`.
pub fn set_none(out: &mut BusTxn) {
    build(out, AuxOpKind::None, 0, RegId::None, value::NONE);
    out.active = false.into();
}

/// Whether a transaction is outstanding, in a record held as its view.
#[must_use]
pub fn held(txn: &BusTxn) -> bool {
    txn.active.into()
}

/// The fields of a record held as its view: where it is addressed, and the value
/// it writes.
///
/// [`read`] over the open type is this plus the one validation at the boundary.
///
/// # Errors
///
/// [`SeqError::PendingUnreadable`] for a record that names no register where the
/// operation needs one.
pub fn fields(txn: &BusTxn, step: SeqStepKind) -> Result<(StepContext, Value), SeqError> {
    let reg = match txn.op {
        AuxOpKind::None | AuxOpKind::Ping => None,
        AuxOpKind::ReadReg | AuxOpKind::WriteRegVerified => match txn.reg {
            RegId::None => {
                return Err(SeqError::PendingUnreadable {
                    context: StepContext::servo(step, txn.id),
                });
            }
            named => Some(named),
        },
    };
    let context = StepContext {
        step,
        id: txn.id,
        reg,
    };
    Ok((context, value::carried(txn.value_kind, txn.value)))
}

/// Whether a transaction is outstanding.
#[must_use]
pub fn active(txn: &BusTxnWire) -> bool {
    txn.active()
}

/// The transaction's fields: where it is addressed, and the value it writes.
///
/// The context is what a failure about this step is reported under, and the
/// value is what a read-back is compared against — [`value::NONE`] where the
/// transaction writes nothing.
///
/// # Errors
///
/// [`SeqError::PendingUnreadable`] for a record this build cannot read: an
/// operation, a register or a value shape outside the vocabulary's declared
/// values. The servo it was addressed to is the one field a malformed record
/// still says plainly, so the refusal says which one.
pub fn read(txn: &BusTxnWire, step: SeqStepKind) -> Result<(StepContext, Value), SeqError> {
    let view = txn.validate().map_err(|_| SeqError::PendingUnreadable {
        context: StepContext::servo(step, txn.id()),
    })?;
    fields(view, step)
}

/// Which transaction this asks for, or `None` for a record this build cannot
/// read.
#[must_use]
pub fn op(txn: &BusTxnWire) -> Option<AuxOpKind> {
    txn.validate().ok().map(|view| view.op)
}

/// The record those fields make, on its own.
///
/// Through the validated view: the fields are assigned as themselves, and the
/// record a fresh `new()` starts from is the declared initial state, which is
/// what makes the view's guarantee hold without a check.
fn wire(op: AuxOpKind, id: u8, reg: RegId, value: Value) -> BusTxnWire {
    let mut out = BusTxnWire::new();
    build(out.clear_valid(), op, id, reg, value);
    out
}

/// Those fields, assigned into `out`.
///
/// Every field, so a record holding an earlier transaction describes this one and
/// nothing else.
fn build(out: &mut BusTxn, op: AuxOpKind, id: u8, reg: RegId, value: Value) {
    out.active = true.into();
    out.op = op;
    out.id = id;
    out.reg = reg;
    out.value_kind = value.shape();
    out.value = value.bits();
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A read, and a verified write, as a whole record.
    ///
    /// Every sequencer builds into its own state, so these two exist only here:
    /// the cases below are about the wire-reading half, which needs a record
    /// standing on its own to read.
    fn read_reg(id: u8, reg: RegId) -> BusTxnWire {
        wire(AuxOpKind::ReadReg, id, reg, value::NONE)
    }

    fn write_reg_verified(id: u8, reg: RegId, value: Value) -> BusTxnWire {
        wire(AuxOpKind::WriteRegVerified, id, reg, value)
    }

    #[test]
    fn the_three_transactions_say_what_they_are() {
        let ping = ping(4);
        let (context, value) = read(&ping, SeqStepKind::Presence).expect("a ping reads");
        assert_eq!(context, StepContext::servo(SeqStepKind::Presence, 4));
        assert_eq!(value, value::NONE);

        let read_reg = read_reg(7, RegId::ModelNumber);
        let (context, value) = read(&read_reg, SeqStepKind::Identity).expect("a read reads");
        assert_eq!(
            context,
            StepContext::reg(SeqStepKind::Identity, 7, RegId::ModelNumber)
        );
        assert_eq!(value, value::NONE);

        let write = write_reg_verified(9, RegId::TorqueEnable, value::u8(1));
        let (context, value) = read(&write, SeqStepKind::VerifyAtStow).expect("a write reads");
        assert_eq!(
            context,
            StepContext::reg(SeqStepKind::VerifyAtStow, 9, RegId::TorqueEnable)
        );
        assert_eq!(value, value::u8(1));
    }

    #[test]
    fn nothing_outstanding_is_the_record_nothing_wrote() {
        assert!(!active(&none()));
        assert!(active(&ping(1)));
        assert_eq!(none(), BusTxnWire::new());
    }

    #[test]
    fn a_clone_is_the_same_transaction() {
        let txn = write_reg_verified(3, RegId::GoalPosition, value::radians(0.25));
        assert_eq!(txn.clone(), txn);
    }

    #[test]
    fn a_register_this_build_does_not_name_is_refused() {
        let mut txn = read_reg(5, RegId::GoalPosition);
        txn.set_reg(brenn_reachy__hardware__dynamixel__registers_clk_rs::RegIdWire(u16::MAX));
        assert_eq!(
            read(&txn, SeqStepKind::Provision),
            Err(SeqError::PendingUnreadable {
                context: StepContext::servo(SeqStepKind::Provision, 5),
            })
        );
    }

    /// A read of no register is a record about nothing, and the sequencer that
    /// reads it back has nothing to ask the bus for.
    #[test]
    fn a_read_naming_no_register_is_refused() {
        let mut txn = read_reg(8, RegId::HomingOffset);
        txn.set_reg(RegId::None.into());
        assert_eq!(
            read(&txn, SeqStepKind::Provision),
            Err(SeqError::PendingUnreadable {
                context: StepContext::servo(SeqStepKind::Provision, 8),
            })
        );
    }

    #[test]
    fn an_operation_outside_the_vocabulary_is_refused() {
        let mut txn = ping(6);
        txn.set_op(brenn_reachy__motion__bus_txn_clk_rs::AuxOpKindWire(9));
        assert_eq!(
            read(&txn, SeqStepKind::Presence),
            Err(SeqError::PendingUnreadable {
                context: StepContext::servo(SeqStepKind::Presence, 6),
            })
        );
    }

    /// The view-writing family assigns every field, so a record holding an
    /// earlier transaction describes this one and nothing else.
    ///
    /// Each step is checked against the fields themselves rather than through
    /// [`read`]: the case this guards is a field the builder forgot, and a
    /// reader that only consults the fields the operation names would not look
    /// at it.
    #[test]
    fn a_record_written_over_carries_no_part_of_the_one_before_it() {
        let mut wire = BusTxnWire::new();
        let out = wire.clear_valid();

        set_write_reg_verified(out, 9, RegId::GoalPosition, value::radians(0.25));
        assert!(held(out));
        assert_eq!(out.op, AuxOpKind::WriteRegVerified);
        assert_eq!(out.id, 9);
        assert_eq!(out.reg, RegId::GoalPosition);
        assert_eq!(
            value::carried(out.value_kind, out.value),
            value::radians(0.25)
        );

        // A ping over it: no register, no value, and the ones before it gone.
        set_ping(out, 3);
        assert!(held(out));
        assert_eq!(out.op, AuxOpKind::Ping);
        assert_eq!(out.id, 3);
        assert_eq!(out.reg, RegId::None);
        assert_eq!(value::carried(out.value_kind, out.value), value::NONE);

        // A read of one register: the register is back, the value stays absent.
        set_read_reg(out, 7, RegId::ModelNumber);
        assert_eq!(out.op, AuxOpKind::ReadReg);
        assert_eq!(out.id, 7);
        assert_eq!(out.reg, RegId::ModelNumber);
        assert_eq!(value::carried(out.value_kind, out.value), value::NONE);

        // Nothing outstanding: every field blank, not merely the flag. A slot
        // that kept the servo and the register would report a transaction
        // against a servo nobody is talking to, and this is the state a
        // post-mortem reads.
        set_none(out);
        assert!(!held(out));
        assert_eq!(out.op, AuxOpKind::None);
        assert_eq!(out.id, 0);
        assert_eq!(out.reg, RegId::None);
        assert_eq!(value::carried(out.value_kind, out.value), value::NONE);
    }
}
