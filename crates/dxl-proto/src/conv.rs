//! Raw register units to engineering units.
//!
//! The wire carries counts, tenths of a volt and raw current units; everything
//! above this crate works in radians, volts and amps. These are the only
//! functions that know both.

use std::f64::consts::PI;

use thiserror::Error;

/// Encoder counts in one turn.
pub const COUNTS_PER_REV: i32 = 4096;

/// Count that reads as zero radians.
pub const CENTRE_COUNTS: i32 = COUNTS_PER_REV / 2;

/// Volts per least-significant bit of the voltage registers.
pub const VOLTS_PER_LSB: f64 = 0.1;

/// Why a value could not be converted.
#[derive(Debug, Clone, Copy, PartialEq, Error)]
pub enum ConvError {
    /// A NaN or an infinity reached the wire boundary.
    #[error("value {value} is not finite")]
    NonFinite { value: f64 },
    /// A finite value that no count can represent.
    #[error("{radians} rad is {counts} counts, outside the representable range")]
    OutOfRange { radians: f64, counts: f64 },
}

/// Counts to radians. Linear and unwrapped, so a multi-turn Present Position
/// reading (which is what a servo reports while its torque is off) converts to
/// a multi-turn angle rather than folding back into one revolution.
#[must_use]
pub fn counts_to_rad(counts: i32) -> f64 {
    2.0 * PI * f64::from(counts) / f64::from(COUNTS_PER_REV) - PI
}

/// Radians to counts, rounded to nearest.
///
/// Rounding, not truncating: a half count is 0.044°, and truncation would bias
/// every goal in one direction on every servo.
///
/// A value that no count represents is an error, never a saturated count. The
/// envelope check upstream means this cannot happen in practice, but the
/// function does not depend on its callers for that.
pub fn rad_to_counts(radians: f64) -> Result<i32, ConvError> {
    if !radians.is_finite() {
        return Err(ConvError::NonFinite { value: radians });
    }
    let counts = (radians + PI) * f64::from(COUNTS_PER_REV) / (2.0 * PI);
    let rounded = counts.round();
    if rounded < f64::from(i32::MIN) || rounded > f64::from(i32::MAX) {
        return Err(ConvError::OutOfRange { radians, counts });
    }
    Ok(rounded as i32)
}

/// Voltage register to volts.
#[must_use]
pub fn volts_from_raw(raw: u16) -> f64 {
    f64::from(raw) * VOLTS_PER_LSB
}

/// Current register to milliamps at the *nominal* scale of 1 mA per unit.
///
/// Treat every figure this returns as an order-of-magnitude estimate. On these
/// parts one unit has been observed to correspond to roughly 3 mA, and the
/// torque constant is not linear, so nothing safety-bearing may be decided from
/// a current in milliamps until the scale is measured on our own hardware.
#[must_use]
pub fn milliamps_from_raw(raw: i16) -> f64 {
    f64::from(raw)
}

/// Latched hardware-error bits, as reported by Hardware Error Status.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct HardwareError(pub u8);

// Bit numbering is the one the eManual's Hardware Error Status and Shutdown bit
// tables give. Note that the Min/Max Voltage Limit text on the same page calls
// the input-voltage flag 0x10 twice; that contradicts the tables, and a reader
// who follows it decodes an overload as an under-voltage. The tables are right.
/// Supply voltage outside the configured window.
pub const HW_INPUT_VOLTAGE: u8 = 1 << 0;
/// Internal temperature above the configured limit.
pub const HW_OVERHEATING: u8 = 1 << 2;
/// Electrical shock, or insufficient power to drive the motor.
pub const HW_ELECTRICAL_SHOCK: u8 = 1 << 4;
/// Sustained load beyond the maximum output.
pub const HW_OVERLOAD: u8 = 1 << 5;

impl HardwareError {
    /// No bits, or the input-voltage bit alone.
    ///
    /// The input-voltage bit alone is not a fault on this machine: the servo
    /// bus rail is specified above the highest Max Voltage Limit the register
    /// accepts, so a perfectly healthy robot sets that bit by arithmetic. It is
    /// still reported, never filtered away — only never acted on by itself.
    #[must_use]
    pub fn healthy_or_voltage_only(&self) -> bool {
        self.bits_other_than_voltage() == 0
    }

    /// Everything except the input-voltage bit.
    #[must_use]
    pub fn bits_other_than_voltage(&self) -> u8 {
        self.0 & !HW_INPUT_VOLTAGE
    }

    /// True when the named bit is set.
    #[must_use]
    pub fn has(&self, bit: u8) -> bool {
        self.0 & bit != 0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn centre_count_is_zero_radians() {
        assert!(counts_to_rad(CENTRE_COUNTS).abs() < 1e-12);
        assert_eq!(rad_to_counts(0.0).unwrap(), CENTRE_COUNTS);
    }

    #[test]
    fn end_counts_span_one_turn() {
        assert!((counts_to_rad(0) + PI).abs() < 1e-12);
        assert!((counts_to_rad(COUNTS_PER_REV) - PI).abs() < 1e-12);
    }

    #[test]
    fn conversion_is_linear_past_one_turn() {
        let one_turn = 2.0 * PI;
        let a = counts_to_rad(CENTRE_COUNTS + COUNTS_PER_REV);
        assert!((a - one_turn).abs() < 1e-12);
        let b = counts_to_rad(CENTRE_COUNTS - 3 * COUNTS_PER_REV);
        assert!((b + 3.0 * one_turn).abs() < 1e-12);
    }

    #[test]
    fn round_trips_every_count_in_a_turn() {
        for counts in 0..=COUNTS_PER_REV {
            let radians = counts_to_rad(counts);
            assert_eq!(rad_to_counts(radians).unwrap(), counts, "{counts}");
        }
    }

    #[test]
    fn rounds_to_nearest_rather_than_truncating() {
        // Six tenths of a count above centre must land one count above, which
        // truncation would not do.
        let step = 2.0 * PI / f64::from(COUNTS_PER_REV);
        assert_eq!(rad_to_counts(0.6 * step).unwrap(), CENTRE_COUNTS + 1);
        assert_eq!(rad_to_counts(0.4 * step).unwrap(), CENTRE_COUNTS);
        assert_eq!(rad_to_counts(-0.6 * step).unwrap(), CENTRE_COUNTS - 1);
    }

    #[test]
    fn non_finite_is_an_error_not_a_count() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            // Compared by variant: a NaN payload is never equal to itself.
            let err = rad_to_counts(value).unwrap_err();
            assert!(matches!(err, ConvError::NonFinite { .. }), "{value}");
        }
    }

    #[test]
    fn out_of_range_is_an_error_not_a_saturated_count() {
        let err = rad_to_counts(1e18).unwrap_err();
        assert!(matches!(err, ConvError::OutOfRange { .. }));
        let err = rad_to_counts(-1e18).unwrap_err();
        assert!(matches!(err, ConvError::OutOfRange { .. }));
    }

    #[test]
    fn voltage_scale() {
        assert!((volts_from_raw(70) - 7.0).abs() < 1e-12);
        assert!((volts_from_raw(0) - 0.0).abs() < 1e-12);
    }

    #[test]
    fn current_scale_is_signed() {
        assert!((milliamps_from_raw(-250) + 250.0).abs() < 1e-12);
    }

    #[test]
    fn voltage_bit_alone_is_not_a_fault() {
        assert!(HardwareError(0).healthy_or_voltage_only());
        assert!(HardwareError(HW_INPUT_VOLTAGE).healthy_or_voltage_only());
        assert_eq!(HardwareError(HW_INPUT_VOLTAGE).bits_other_than_voltage(), 0);
    }

    #[test]
    fn any_other_bit_is_a_fault() {
        for bit in [HW_OVERHEATING, HW_ELECTRICAL_SHOCK, HW_OVERLOAD] {
            let err = HardwareError(bit | HW_INPUT_VOLTAGE);
            assert!(!err.healthy_or_voltage_only(), "{bit:#04x}");
            assert_eq!(err.bits_other_than_voltage(), bit);
            assert!(err.has(bit));
            assert!(err.has(HW_INPUT_VOLTAGE));
        }
    }

    /// The shutdown mask this project expects to read back is 52: overload,
    /// electrical shock and overheating, with input voltage deliberately out.
    #[test]
    fn the_expected_shutdown_mask_decomposes_as_documented() {
        assert_eq!(HW_OVERLOAD | HW_ELECTRICAL_SHOCK | HW_OVERHEATING, 52);
        assert_eq!(
            HW_OVERLOAD | HW_ELECTRICAL_SHOCK | HW_OVERHEATING | HW_INPUT_VOLTAGE,
            53
        );
    }
}
