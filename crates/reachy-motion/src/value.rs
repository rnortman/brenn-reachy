//! A register's value: its shape, and the eight bytes it rides in.
//!
//! The shape is the vocabulary's [`ValueShape`] and the bytes are the same
//! eight a transaction carries, read as one little-endian number. Engineering
//! units wherever the register has any — radians and volts cross as radians and
//! volts, and counts are the bus layer's business — so what a shape names is how
//! to read the bits, never what they mean.
//!
//! There is no second representation. A [`Value`] here is the two fields of a
//! bus transaction, in the order the schema declares them, so a sequencer's
//! value and the one in a slot are the same bits with the same tag beside them;
//! the constructors and readers below are how those bits are made and read, and
//! nothing converts.
//!
//! The readers come in two heights. [`Value`]'s own are shape-checked and
//! answer `None`, for a caller with no sequencer step to name — the bus layer
//! and the bench. The free functions of the same names take a
//! [`StepContext`] and turn that `None` into the sequencer's own refusal. Both
//! read the same bits in the same place: no shift width leaves this module.

use core::fmt;

pub use brenn_reachy__hardware__dynamixel__registers_clk_rs::ValueShape;

use crate::seq::{SeqError, StepContext};

/// A value and the shape it is to be read in.
///
/// [`ValueShape::None`] is the absence of one: a ping's answer, a step that
/// wants nothing written, an empty cell in a readings grid. The bits beside it
/// are zero and mean nothing.
///
/// The two fields are private and the constructors below are the only way to
/// make one, so a shape and the bits beside it always agree: nothing can hand a
/// sequencer an angle whose bits are the integer three.
///
/// Equality is the shape and the bits: two values are equal when the same bytes
/// carry the same tag. For the float-shaped ones that is bit equality, not
/// numeric equality — `0.0` and `-0.0` are different values here, and two NaNs
/// are the same one — so a question that is numeric (a provisioned angle read
/// back, a rail voltage compared to a floor) is asked through
/// [`Value::as_radians`] / [`Value::as_volts`] and answered in `f64`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Value(ValueShape, u64);

impl Value {
    /// How the bits are to be read.
    #[must_use]
    pub const fn shape(self) -> ValueShape {
        self.0
    }

    /// The eight bytes, as one little-endian number.
    #[must_use]
    pub const fn bits(self) -> u64 {
        self.1
    }

    /// The byte, if that is the shape.
    #[must_use]
    pub fn as_u8(self) -> Option<u8> {
        self.narrow(ValueShape::U8).map(|bits| bits as u8)
    }

    /// The two-byte value, if that is the shape.
    #[must_use]
    pub fn as_u16(self) -> Option<u16> {
        self.narrow(ValueShape::U16).map(|bits| bits as u16)
    }

    /// The four-byte value, if that is the shape.
    #[must_use]
    pub fn as_u32(self) -> Option<u32> {
        self.narrow(ValueShape::U32).map(|bits| bits as u32)
    }

    /// The signed four-byte value, if that is the shape.
    #[must_use]
    pub fn as_i32(self) -> Option<i32> {
        self.narrow(ValueShape::I32)
            .map(|bits| (bits as u32).cast_signed())
    }

    /// The angle, if that is the shape.
    #[must_use]
    pub fn as_radians(self) -> Option<f64> {
        self.narrow(ValueShape::Radians).map(f64::from_bits)
    }

    /// The voltage, if that is the shape.
    #[must_use]
    pub fn as_volts(self) -> Option<f64> {
        self.narrow(ValueShape::Volts).map(f64::from_bits)
    }

    /// The three gains in `p, i, d` order, if that is the shape.
    #[must_use]
    pub fn as_gains(self) -> Option<(u16, u16, u16)> {
        self.narrow(ValueShape::Gains)
            .map(|bits| (bits as u16, (bits >> 16) as u16, (bits >> 32) as u16))
    }

    /// Whether this is a value this stack passes onward.
    ///
    /// An angle or a voltage that is not a number is refused rather than
    /// carried: the one thing nothing here hands on is a non-finite commanded
    /// or reported value, and a driver given one would write it to a servo.
    #[must_use]
    pub fn is_carriable(self) -> bool {
        match self.0 {
            ValueShape::Radians | ValueShape::Volts => f64::from_bits(self.1).is_finite(),
            _ => true,
        }
    }

    /// The bits, if the shape is the one wanted.
    fn narrow(self, wanted: ValueShape) -> Option<u64> {
        (self.0 == wanted).then_some(self.1)
    }
}

/// The absence of a value.
pub const NONE: Value = Value(ValueShape::None, 0);

/// The value a schema's two fields carry.
///
/// The one constructor that does not know what it is making: a shape and eight
/// bytes arriving together from a slot or a datagram, which is where the pairing
/// was decided. Everything inside this stack builds a value from a number
/// instead, through the constructors below.
#[must_use]
pub const fn carried(shape: ValueShape, bits: u64) -> Value {
    Value(shape, bits)
}

/// A one-byte register's value.
#[must_use]
pub fn u8(value: u8) -> Value {
    Value(ValueShape::U8, u64::from(value))
}

/// A two-byte register's value.
#[must_use]
pub fn u16(value: u16) -> Value {
    Value(ValueShape::U16, u64::from(value))
}

/// A four-byte register's value.
#[must_use]
pub fn u32(value: u32) -> Value {
    Value(ValueShape::U32, u64::from(value))
}

/// A signed four-byte register's value.
///
/// Carried as the four bytes' own pattern rather than sign-extended: the width
/// the shape names is what a reader takes, and the homing offset read back as
/// four billion is the bug this shape exists to prevent.
#[must_use]
pub fn i32(value: i32) -> Value {
    Value(ValueShape::I32, u64::from(value.cast_unsigned()))
}

/// An angle, in the model's own frame.
#[must_use]
pub fn radians(value: f64) -> Value {
    Value(ValueShape::Radians, value.to_bits())
}

/// A supply voltage.
#[must_use]
pub fn volts(value: f64) -> Value {
    Value(ValueShape::Volts, value.to_bits())
}

/// The position loop's three gains as one span, two bytes each in `p, i, d`
/// order. Their order on the wire is not this order, and that is the bus
/// layer's problem.
#[must_use]
pub fn gains(p: u16, i: u16, d: u16) -> Value {
    let bits = u64::from(p) | u64::from(i) << 16 | u64::from(d) << 32;
    Value(ValueShape::Gains, bits)
}

/// The byte, or a failure naming what arrived instead.
pub fn as_u8(value: Value, context: StepContext) -> Result<u8, SeqError> {
    value
        .as_u8()
        .ok_or_else(|| wrong(value, ValueShape::U8, context))
}

/// The two-byte value, or a failure naming what arrived instead.
pub fn as_u16(value: Value, context: StepContext) -> Result<u16, SeqError> {
    value
        .as_u16()
        .ok_or_else(|| wrong(value, ValueShape::U16, context))
}

/// The four-byte value, or a failure naming what arrived instead.
pub fn as_u32(value: Value, context: StepContext) -> Result<u32, SeqError> {
    value
        .as_u32()
        .ok_or_else(|| wrong(value, ValueShape::U32, context))
}

/// The signed four-byte value, or a failure naming what arrived instead.
pub fn as_i32(value: Value, context: StepContext) -> Result<i32, SeqError> {
    value
        .as_i32()
        .ok_or_else(|| wrong(value, ValueShape::I32, context))
}

/// The angle, or a failure naming what arrived instead.
pub fn as_radians(value: Value, context: StepContext) -> Result<f64, SeqError> {
    value
        .as_radians()
        .ok_or_else(|| wrong(value, ValueShape::Radians, context))
}

/// The voltage, or a failure naming what arrived instead.
pub fn as_volts(value: Value, context: StepContext) -> Result<f64, SeqError> {
    value
        .as_volts()
        .ok_or_else(|| wrong(value, ValueShape::Volts, context))
}

/// The three gains, or a failure naming what arrived instead.
pub fn as_gains(value: Value, context: StepContext) -> Result<(u16, u16, u16), SeqError> {
    value
        .as_gains()
        .ok_or_else(|| wrong(value, ValueShape::Gains, context))
}

/// The refusal a shape-checked read raises, naming what arrived instead.
fn wrong(value: Value, wanted: ValueShape, context: StepContext) -> SeqError {
    SeqError::WrongValue {
        context,
        expected: wanted,
        observed: value.shape(),
    }
}

vocab_name! {
    /// A shape, as an operator reads it.
    pub struct ShapeName(ValueShape) {
        ValueShape::None => "absent value",
        ValueShape::U8 => "one-byte value",
        ValueShape::U16 => "two-byte value",
        ValueShape::U32 => "four-byte value",
        ValueShape::I32 => "signed four-byte value",
        ValueShape::Radians => "angle",
        ValueShape::Volts => "voltage",
        ValueShape::Gains => "gain span",
    }
}

/// Every shape a register's value takes, the absent-value zero excluded: that one
/// is a transaction with no value, not a value of a shape.
pub fn shapes() -> impl Iterator<Item = ValueShape> {
    crate::vocab::without_zero(ValueShape::VARIANTS)
}

/// A value, as an operator reads it: the number in the units its shape names.
#[derive(Clone, Copy, Debug)]
pub struct Shown(pub Value);

impl fmt::Display for Shown {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let value = self.0;
        let read = |shape| match shape {
            ValueShape::None => None,
            ValueShape::U8 => value.as_u8().map(|v| v.to_string()),
            ValueShape::U16 => value.as_u16().map(|v| v.to_string()),
            ValueShape::U32 => value.as_u32().map(|v| v.to_string()),
            ValueShape::I32 => value.as_i32().map(|v| v.to_string()),
            ValueShape::Radians => value.as_radians().map(|v| format!("{v:.4} rad")),
            ValueShape::Volts => value.as_volts().map(|v| format!("{v:.1} V")),
            ValueShape::Gains => value
                .as_gains()
                .map(|(p, i, d)| format!("P {p} I {i} D {d}")),
        };
        match read(value.shape()) {
            Some(text) => f.write_str(&text),
            None => f.write_str("nothing"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::seq::SeqStepKind;

    fn context() -> StepContext {
        StepContext::servo(SeqStepKind::Presence, 10)
    }

    /// A number that is not one does not cross: the one value nothing above the
    /// wire line passes onward.
    #[test]
    fn a_non_finite_angle_is_not_carriable() {
        assert!(!radians(f64::NAN).is_carriable());
        assert!(!volts(f64::INFINITY).is_carriable());
        assert!(radians(0.0).is_carriable());
        assert!(u32(u32::MAX).is_carriable());
    }

    #[test]
    fn every_shape_reads_back_what_was_written() {
        assert_eq!(as_u8(u8(3), context()), Ok(3));
        assert_eq!(as_u16(u16(1750), context()), Ok(1750));
        assert_eq!(as_u32(u32(4096), context()), Ok(4096));
        assert_eq!(as_i32(i32(-1024), context()), Ok(-1024));
        assert_eq!(as_radians(radians(-0.5), context()), Ok(-0.5));
        assert_eq!(as_volts(volts(7.4), context()), Ok(7.4));
        assert_eq!(as_gains(gains(300, 1, 2), context()), Ok((300, 1, 2)));
    }

    #[test]
    fn a_reader_answers_only_on_the_shape_it_names() {
        let made = [
            u8(3),
            u16(1750),
            u32(4096),
            i32(-1024),
            radians(-0.5),
            volts(7.4),
            gains(300, 1, 2),
        ];
        for value in made {
            let answered = [
                value.as_u8().is_some(),
                value.as_u16().is_some(),
                value.as_u32().is_some(),
                value.as_i32().is_some(),
                value.as_radians().is_some(),
                value.as_volts().is_some(),
                value.as_gains().is_some(),
            ];
            assert_eq!(
                answered.iter().filter(|answered| **answered).count(),
                1,
                "{:?} was read by more than its own reader",
                value.shape()
            );
        }
        assert!(NONE.as_u8().is_none(), "the absence of a value reads as no");
    }

    #[test]
    fn a_value_out_of_a_slot_is_the_value_that_went_in() {
        let sent = gains(300, 1, 2);
        let crossed = carried(sent.shape(), sent.bits());
        assert_eq!(crossed, sent);
        assert_eq!(crossed.as_gains(), Some((300, 1, 2)));
    }

    #[test]
    fn a_value_of_another_shape_is_refused_naming_both() {
        assert_eq!(
            as_u8(u16(3), context()),
            Err(SeqError::WrongValue {
                context: context(),
                expected: ValueShape::U8,
                observed: ValueShape::U16,
            })
        );
    }

    #[test]
    fn a_signed_value_is_not_read_as_an_unsigned_one() {
        // The four bytes of -1,024 read unsigned are a number near four
        // billion, which is the confusion this shape tag exists to refuse, and
        // it is refused in both directions.
        assert_eq!(
            as_u32(i32(-1024), context()),
            Err(SeqError::WrongValue {
                context: context(),
                expected: ValueShape::U32,
                observed: ValueShape::I32,
            })
        );
        let confused = as_i32(u32(1024), context())
            .expect_err("an unsigned four bytes is not the signed register")
            .to_string();
        assert!(
            confused.contains("signed four-byte value") && confused.ends_with("four-byte value"),
            "{confused}"
        );
    }

    /// The refusal's text is what an operator reads, so it names both shapes in
    /// the words a data sheet uses.
    #[test]
    fn a_refusal_names_both_shapes_in_words() {
        assert_eq!(
            as_radians(u8(3), context())
                .expect_err("a byte is not an angle")
                .to_string(),
            "presence of servo 10: expected an angle and got a one-byte value"
        );
    }

    #[test]
    fn the_absence_of_a_value_is_a_shape_and_reads_as_none_of_them() {
        assert_eq!(NONE.0, ValueShape::None);
        assert!(as_u8(NONE, context()).is_err());
    }

    #[test]
    fn a_value_reads_in_the_units_its_shape_names() {
        assert_eq!(Shown(u8(3)).to_string(), "3");
        assert_eq!(Shown(i32(-1024)).to_string(), "-1024");
        assert_eq!(Shown(radians(-0.628_3)).to_string(), "-0.6283 rad");
        assert_eq!(Shown(volts(7.38)).to_string(), "7.4 V");
        assert_eq!(Shown(gains(300, 1, 2)).to_string(), "P 300 I 1 D 2");
        assert_eq!(Shown(NONE).to_string(), "nothing");
        assert_eq!(ShapeName(ValueShape::Gains).to_string(), "gain span");
    }

    #[test]
    fn the_gain_span_packs_three_numbers_without_them_meeting() {
        assert_eq!(
            as_gains(gains(u16::MAX, u16::MAX, u16::MAX), context()),
            Ok((u16::MAX, u16::MAX, u16::MAX))
        );
        assert_eq!(
            as_gains(gains(0, 0, u16::MAX), context()),
            Ok((0, 0, u16::MAX))
        );
    }
}
