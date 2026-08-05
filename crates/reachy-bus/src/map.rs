//! Joints, the datum, registers and counts — the one place all four meet.
//!
//! Above this module the machine is nine joints holding angles in radians.
//! Below it the machine is servo IDs, control-table addresses and encoder
//! counts. This map is the only translation between the two, and being the only
//! one is the point: a second copy of the datum shift, or a second register
//! table, could disagree with this one, and a disagreement about which crank a
//! goal belongs to puts the head on the desk.
//!
//! ## The datum
//!
//! The legs' position registers carry a provisioned homing offset, and which
//! offset this unit was given is not yet known. Under one reading a converted
//! count *is* the model's crank angle; under the other, alternate legs sit a
//! quarter turn either side of it. Both readings close the linkage exactly, so
//! nothing computed separates them — an externally measured head height does.
//! Until a human has resolved it and written it down, no motion command runs.
//!
//! ## Shapes
//!
//! Each register has exactly one engineering shape, and it is fixed here.
//! Position registers cross this boundary as radians, with the datum applied;
//! the supply-voltage reading crosses as volts. Everything else crosses as the
//! integer the control table holds, because the provisioning checks compare
//! those against a data sheet and a converted angle would be the wrong thing to
//! compare. The position-gain span is the one register whose byte order is not
//! its field order: the table runs D, I, P.

use std::f64::consts::FRAC_PI_2;

use dxl_proto::regs;
use dxl_proto::{ConvError, Reg, counts_to_rad, rad_to_counts, raw_from_volts, volts_from_raw};
use reachy_motion::{JointId, RegId, RegValue, ValueKind};
use thiserror::Error;

use crate::bus::{MAX_SYNC_IDS, RawValue};

// The map hands the grouped operations one entry per joint, and those refuse
// more IDs than a single frame carries. Tying the two counts here makes a roster
// that outgrew the frame a compile error rather than a refusal on hardware.
const _: () = assert!(JointId::COUNT <= MAX_SYNC_IDS);

/// How the legs' counts relate to the model's crank angles.
///
/// Absent from a bench configuration until a human has resolved it, and no
/// motion command runs without one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CrankDatum {
    /// A converted count is the crank angle as the model means it.
    Direct,
    /// Alternate legs are a quarter turn either side of the model's angle: the
    /// odd-numbered legs a quarter turn one way, the even-numbered legs the
    /// other.
    ParityShifted,
}

impl CrankDatum {
    /// The shift added to each of the six legs' converted counts to reach the
    /// model angle, in leg order.
    ///
    /// Total: under either datum every leg has a shift, so a caller working
    /// over the six legs never has an absent one to substitute for. The 0-based
    /// leg index: leg 1 on the bus is `0` here and takes the positive shift.
    #[must_use]
    pub fn leg_shifts(self) -> [f64; 6] {
        match self {
            Self::Direct => [0.0; 6],
            Self::ParityShifted => [
                FRAC_PI_2, -FRAC_PI_2, FRAC_PI_2, -FRAC_PI_2, FRAC_PI_2, -FRAC_PI_2,
            ],
        }
    }

    /// The shift added to `joint`'s converted count to reach the model angle,
    /// or `None` for a joint this bus has no place for.
    ///
    /// Takes the joint rather than a row number or a bare leg index, because
    /// the two numberings are one apart and the datum must never move the yaw
    /// or an antenna: a row handed in where a leg was meant would otherwise
    /// come back with a quarter turn on it. A leg past the sixth places
    /// nowhere and has no shift to give.
    #[must_use]
    pub fn shift_for(self, joint: JointId) -> Option<f64> {
        joint.index()?;
        match joint {
            JointId::Leg(leg) => self.leg_shifts().get(usize::from(leg)).copied(),
            // Yaw and the antennas are single-turn joints with no parity
            // pairing, so no reading of the datum moves them.
            _ => Some(0.0),
        }
    }
}

impl core::fmt::Display for CrankDatum {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Direct => f.write_str("direct"),
            Self::ParityShifted => f.write_str("parity shifted"),
        }
    }
}

/// Why a value could not cross the boundary.
#[derive(Clone, Copy, Debug, PartialEq, Error)]
pub enum MapError {
    /// A joint index past the nine.
    #[error("joint {joint} is not one of the nine")]
    UnknownJoint {
        /// The index asked for.
        joint: usize,
    },

    /// An angle no count represents. Never a saturated count: a goal that
    /// cannot be placed is a refusal, not a nearby goal.
    #[error("servo {id}: {source}")]
    Angle {
        /// Position in bus order.
        joint: usize,
        /// The servo's bus ID.
        id: u8,
        /// What the conversion refused on.
        source: ConvError,
    },

    /// A value of the wrong engineering shape for the register named.
    #[error("{reg} is {expected}, not {observed}")]
    WrongShape {
        /// The register.
        reg: RegId,
        /// The shape the register has.
        expected: ValueKind,
        /// The shape that arrived.
        observed: ValueKind,
    },

    /// Bytes that do not fill, or overfill, the register named.
    #[error("{reg} is {expected} bytes wide, these are {actual}")]
    Width {
        /// The register.
        reg: RegId,
        /// The register's width.
        expected: usize,
        /// The width that arrived.
        actual: usize,
    },

    /// A voltage no register unit represents. Never a saturated unit, for the
    /// same reason an unplaceable angle is not a nearby goal.
    #[error("{reg}: {source}")]
    Voltage {
        /// The register.
        reg: RegId,
        /// What the conversion refused on.
        source: ConvError,
    },
}

/// The control-table entry a register name refers to.
///
/// One table for all nine servos: they are three different parts, but the
/// X-series control table places these registers at the same addresses on all
/// of them. What differs between the groups is the *values* they are
/// provisioned with, which is bench configuration rather than addressing.
#[must_use]
pub fn reg_for(reg: RegId) -> Reg {
    match reg {
        RegId::TorqueEnable => regs::TORQUE_ENABLE,
        RegId::GoalPosition => regs::GOAL_POSITION,
        RegId::PresentPosition => regs::PRESENT_POSITION,
        RegId::OperatingMode => regs::OPERATING_MODE,
        RegId::HomingOffset => regs::HOMING_OFFSET,
        RegId::ReturnDelayTime => regs::RETURN_DELAY_TIME,
        RegId::MinPositionLimit => regs::MIN_POSITION_LIMIT,
        RegId::MaxPositionLimit => regs::MAX_POSITION_LIMIT,
        RegId::Shutdown => regs::SHUTDOWN,
        RegId::DriveMode => regs::DRIVE_MODE,
        RegId::MaxVoltageLimit => regs::MAX_VOLTAGE_LIMIT,
        RegId::MinVoltageLimit => regs::MIN_VOLTAGE_LIMIT,
        RegId::CurrentLimit => regs::CURRENT_LIMIT,
        RegId::VelocityLimit => regs::VELOCITY_LIMIT,
        RegId::TemperatureLimit => regs::TEMPERATURE_LIMIT,
        RegId::BusWatchdog => regs::BUS_WATCHDOG,
        RegId::ProfileAcceleration => regs::PROFILE_ACCELERATION,
        RegId::ProfileVelocity => regs::PROFILE_VELOCITY,
        RegId::PositionGains => regs::POSITION_GAINS,
        RegId::HardwareErrorStatus => regs::HARDWARE_ERROR_STATUS,
        RegId::PresentInputVoltage => regs::PRESENT_INPUT_VOLTAGE,
        RegId::ModelNumber => regs::MODEL_NUMBER,
    }
}

/// The engineering shape a register's value takes above this crate.
///
/// Only the two position registers a command path touches are angles. The
/// travel limits and the homing offset stay integers: they are compared
/// against how the unit was provisioned, and a datum-shifted angle is not that
/// comparison.
#[must_use]
pub fn value_kind(reg: RegId) -> ValueKind {
    match reg {
        RegId::GoalPosition | RegId::PresentPosition => ValueKind::Radians,
        RegId::PresentInputVoltage => ValueKind::Volts,
        RegId::PositionGains => ValueKind::Gains,
        RegId::TorqueEnable
        | RegId::OperatingMode
        | RegId::ReturnDelayTime
        | RegId::Shutdown
        | RegId::DriveMode
        | RegId::TemperatureLimit
        | RegId::BusWatchdog
        | RegId::HardwareErrorStatus => ValueKind::U8,
        RegId::MaxVoltageLimit
        | RegId::MinVoltageLimit
        | RegId::CurrentLimit
        | RegId::ModelNumber => ValueKind::U16,
        RegId::HomingOffset
        | RegId::MinPositionLimit
        | RegId::MaxPositionLimit
        | RegId::VelocityLimit
        | RegId::ProfileAcceleration
        | RegId::ProfileVelocity => ValueKind::U32,
    }
}

/// Why the shape accessors cannot fail once the bytes match the register's
/// width: every register's declared width is exactly the width its engineering
/// shape needs, which `every_register_name_has_an_address_and_a_shape` pins for
/// all of them.
const SHAPE_FITS_WIDTH: &str = "a register's width is the width its shape needs";

/// One joint's wire identity.
#[derive(Clone, Copy, Debug, PartialEq)]
struct JointServo {
    id: u8,
    datum_shift: f64,
}

/// Which servo each joint is, and what its counts mean.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoMap {
    joints: [JointServo; JointId::COUNT],
    datum: CrankDatum,
}

impl ServoMap {
    /// A map over `ids` in bus order — body yaw, legs 1..=6, right antenna,
    /// left antenna — with the legs' counts read under `datum`.
    #[must_use]
    pub fn new(ids: [u8; JointId::COUNT], datum: CrankDatum) -> Self {
        let mut joints = [JointServo {
            id: 0,
            datum_shift: 0.0,
        }; JointId::COUNT];
        for (slot, (id, joint)) in joints.iter_mut().zip(ids.iter().zip(JointId::ALL)) {
            // Every joint in bus order places, so every one has a shift.
            debug_assert!(datum.shift_for(joint).is_some());
            let shift = datum.shift_for(joint).unwrap_or(0.0);
            *slot = JointServo {
                id: *id,
                datum_shift: shift,
            };
        }
        Self { joints, datum }
    }

    /// The datum the legs are read under.
    #[must_use]
    pub fn datum(&self) -> CrankDatum {
        self.datum
    }

    /// The servo IDs, in bus order.
    #[must_use]
    pub fn ids(&self) -> [u8; JointId::COUNT] {
        let mut ids = [0; JointId::COUNT];
        for (slot, joint) in ids.iter_mut().zip(&self.joints) {
            *slot = joint.id;
        }
        ids
    }

    /// The servo at position `joint` in bus order.
    #[must_use]
    pub fn id_at(&self, joint: usize) -> Option<u8> {
        self.joints.get(joint).map(|servo| servo.id)
    }

    /// The shift added to `joint`'s converted count to reach the model angle.
    #[must_use]
    pub fn shift_at(&self, joint: usize) -> Option<f64> {
        self.joints.get(joint).map(|servo| servo.datum_shift)
    }

    /// A measured count as the model's angle.
    pub fn present_rad(&self, joint: usize, counts: i32) -> Result<f64, MapError> {
        let servo = self.servo(joint)?;
        Ok(counts_to_rad(counts) + servo.datum_shift)
    }

    /// A model angle as the count to command.
    ///
    /// Rounds to the nearest count and refuses anything no count represents.
    pub fn goal_counts(&self, joint: usize, rad: f64) -> Result<i32, MapError> {
        let servo = self.servo(joint)?;
        rad_to_counts(rad - servo.datum_shift).map_err(|source| MapError::Angle {
            joint,
            id: servo.id,
            source,
        })
    }

    /// A register's value as the bytes to put on the wire.
    pub fn encode_value(
        &self,
        joint: usize,
        reg: RegId,
        value: RegValue,
    ) -> Result<RawValue, MapError> {
        let expected = value_kind(reg);
        if value.kind() != expected {
            return Err(MapError::WrongShape {
                reg,
                expected,
                observed: value.kind(),
            });
        }
        match value {
            RegValue::U8(byte) => self.carry(reg, &[byte]),
            RegValue::U16(word) => self.carry(reg, &word.to_le_bytes()),
            RegValue::U32(long) => self.carry(reg, &long.to_le_bytes()),
            RegValue::Radians(rad) => {
                let counts = self.goal_counts(joint, rad)?;
                self.carry(reg, &counts.to_le_bytes())
            }
            RegValue::Volts(volts) => {
                let units =
                    raw_from_volts(volts).map_err(|source| MapError::Voltage { reg, source })?;
                self.carry(reg, &units.to_le_bytes())
            }
            RegValue::Gains { p, i, d } => {
                // Wire order is the table's: derivative lowest, proportional
                // highest, integral between them.
                let mut bytes = [0u8; 6];
                bytes[0..2].copy_from_slice(&d.to_le_bytes());
                bytes[2..4].copy_from_slice(&i.to_le_bytes());
                bytes[4..6].copy_from_slice(&p.to_le_bytes());
                self.carry(reg, &bytes)
            }
        }
    }

    /// Bytes off the wire as the register's value.
    pub fn decode_value(
        &self,
        joint: usize,
        reg: RegId,
        raw: &RawValue,
    ) -> Result<RegValue, MapError> {
        let width = usize::from(reg_for(reg).len);
        if raw.len() != width {
            return Err(MapError::Width {
                reg,
                expected: width,
                actual: raw.len(),
            });
        }
        match value_kind(reg) {
            ValueKind::U8 => Ok(RegValue::U8(raw.u8().expect(SHAPE_FITS_WIDTH))),
            ValueKind::U16 => Ok(RegValue::U16(raw.u16().expect(SHAPE_FITS_WIDTH))),
            ValueKind::U32 => Ok(RegValue::U32(raw.u32().expect(SHAPE_FITS_WIDTH))),
            ValueKind::Radians => {
                let counts = raw.i32().expect(SHAPE_FITS_WIDTH);
                Ok(RegValue::Radians(self.present_rad(joint, counts)?))
            }
            ValueKind::Volts => {
                let units = raw.u16().expect(SHAPE_FITS_WIDTH);
                Ok(RegValue::Volts(volts_from_raw(units)))
            }
            ValueKind::Gains => {
                let bytes = raw.as_slice();
                let word = |at: usize| u16::from_le_bytes([bytes[at], bytes[at + 1]]);
                Ok(RegValue::Gains {
                    d: word(0),
                    i: word(2),
                    p: word(4),
                })
            }
        }
    }

    fn servo(&self, joint: usize) -> Result<&JointServo, MapError> {
        self.joints
            .get(joint)
            .ok_or(MapError::UnknownJoint { joint })
    }

    /// Bytes as a value, checked against the register's own width so a shape
    /// and its register can never disagree about how many bytes go out.
    fn carry(&self, reg: RegId, bytes: &[u8]) -> Result<RawValue, MapError> {
        let width = usize::from(reg_for(reg).len);
        if bytes.len() != width {
            return Err(MapError::Width {
                reg,
                expected: width,
                actual: bytes.len(),
            });
        }
        RawValue::new(bytes).ok_or(MapError::Width {
            reg,
            expected: RawValue::MAX_LEN,
            actual: bytes.len(),
        })
    }
}

#[cfg(test)]
mod tests {
    use std::f64::consts::PI;

    use dxl_proto::conv::{CENTRE_COUNTS, COUNTS_PER_REV};

    use super::*;

    /// The nine servo IDs this machine uses, in bus order.
    const IDS: [u8; JointId::COUNT] = [10, 11, 12, 13, 14, 15, 16, 17, 18];

    fn direct() -> ServoMap {
        ServoMap::new(IDS, CrankDatum::Direct)
    }

    fn shifted() -> ServoMap {
        ServoMap::new(IDS, CrankDatum::ParityShifted)
    }

    #[test]
    fn every_joint_maps_to_its_own_servo_in_bus_order() {
        let map = direct();
        assert_eq!(map.ids(), IDS);
        for (index, id) in IDS.iter().enumerate() {
            assert_eq!(map.id_at(index), Some(*id));
        }
        assert_eq!(map.id_at(JointId::COUNT), None);
        assert_eq!(map.shift_at(JointId::COUNT), None);
        assert_eq!(
            map.present_rad(JointId::COUNT, 0),
            Err(MapError::UnknownJoint {
                joint: JointId::COUNT
            })
        );
        assert_eq!(
            map.goal_counts(JointId::COUNT, 0.0),
            Err(MapError::UnknownJoint {
                joint: JointId::COUNT
            })
        );
    }

    #[test]
    fn the_direct_datum_shifts_nothing() {
        let map = direct();
        assert_eq!(map.datum(), CrankDatum::Direct);
        for index in 0..JointId::COUNT {
            assert_eq!(map.shift_at(index), Some(0.0));
        }
        assert!(map.present_rad(1, CENTRE_COUNTS).unwrap().abs() < 1e-12);
        assert_eq!(map.goal_counts(1, 0.0), Ok(CENTRE_COUNTS));
    }

    #[test]
    fn the_parity_datum_moves_alternate_legs_a_quarter_turn_each_way() {
        let map = shifted();
        assert_eq!(map.datum(), CrankDatum::ParityShifted);

        // Yaw and the antennas are untouched under either datum.
        assert_eq!(map.shift_at(0), Some(0.0));
        assert_eq!(map.shift_at(7), Some(0.0));
        assert_eq!(map.shift_at(8), Some(0.0));

        // Legs 1, 3, 5 sit at bus positions 1, 3, 5 and take the positive
        // shift; legs 2, 4, 6 take the negative one.
        for (index, expected) in [
            (1, FRAC_PI_2),
            (2, -FRAC_PI_2),
            (3, FRAC_PI_2),
            (4, -FRAC_PI_2),
            (5, FRAC_PI_2),
            (6, -FRAC_PI_2),
        ] {
            assert_eq!(map.shift_at(index), Some(expected));
            let model = map.present_rad(index, CENTRE_COUNTS).unwrap();
            assert!((model - expected).abs() < 1e-12, "joint {index}");
        }
    }

    /// The shift is asked for by joint, not by a number: the bus rows and the
    /// leg indices are one apart, and a row handed in where a leg was meant
    /// would otherwise come back with a quarter turn on a joint the datum must
    /// never move. A leg past the sixth places nowhere and gets no answer.
    #[test]
    fn only_a_joint_this_bus_has_a_place_for_has_a_shift() {
        for datum in [CrankDatum::Direct, CrankDatum::ParityShifted] {
            for joint in JointId::ALL {
                assert!(datum.shift_for(joint).is_some(), "{joint} under {datum}");
            }
            for leg in [6u8, 7, 9, u8::MAX] {
                assert_eq!(
                    datum.shift_for(JointId::Leg(leg)),
                    None,
                    "there is no seventh crank"
                );
            }
        }
        // The right antenna sits at row 7, where leg 7 would be if rows were
        // legs. It takes no shift under either datum.
        assert_eq!(
            CrankDatum::ParityShifted.shift_for(JointId::AntennaRight),
            Some(0.0)
        );
        assert_eq!(
            CrankDatum::ParityShifted.shift_for(JointId::BodyYaw),
            Some(0.0)
        );
    }

    /// The six legs' shifts are one record, read the same way whether a caller
    /// asks joint by joint or takes the whole array. A caller working over the
    /// six legs therefore never meets an absent shift to substitute for.
    #[test]
    fn the_leg_shifts_are_the_array_and_the_lookup_alike() {
        for datum in [CrankDatum::Direct, CrankDatum::ParityShifted] {
            let shifts = datum.leg_shifts();
            for (leg, shift) in shifts.iter().enumerate() {
                let leg = u8::try_from(leg).expect("six legs fit a byte");
                assert_eq!(datum.shift_for(JointId::Leg(leg)), Some(*shift));
            }
        }
        assert_eq!(CrankDatum::Direct.leg_shifts(), [0.0; 6]);
        // Alternate legs a quarter turn each way, and no leg left where it was.
        for (leg, shift) in CrankDatum::ParityShifted.leg_shifts().iter().enumerate() {
            assert_eq!(shift.abs(), FRAC_PI_2, "leg {}", leg + 1);
            assert_eq!(shift.is_sign_positive(), leg.is_multiple_of(2));
        }
    }

    #[test]
    fn a_count_and_an_angle_round_trip_under_both_datums() {
        for map in [direct(), shifted()] {
            for joint in 0..JointId::COUNT {
                for counts in [0, 1000, CENTRE_COUNTS, 3000, COUNTS_PER_REV] {
                    let rad = map.present_rad(joint, counts).unwrap();
                    assert_eq!(
                        map.goal_counts(joint, rad),
                        Ok(counts),
                        "joint {joint} at {counts} counts under {}",
                        map.datum()
                    );
                }
            }
        }
    }

    #[test]
    fn an_angle_no_count_places_is_refused_rather_than_saturated() {
        let map = direct();
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let failure = map.goal_counts(3, bad).expect_err("nothing places that");
            assert!(matches!(
                failure,
                MapError::Angle {
                    joint: 3,
                    id: 13,
                    source: ConvError::NonFinite { .. }
                }
            ));
        }
        let far = 1e12;
        assert!(matches!(
            map.goal_counts(3, far),
            Err(MapError::Angle {
                source: ConvError::OutOfRange { .. },
                ..
            })
        ));
    }

    #[test]
    fn a_goal_is_the_measurement_it_reads_back_as() {
        // What the tick relies on: a goal written under a datum and read back
        // under the same datum is the angle that was commanded.
        let map = shifted();
        let commanded = 0.3;
        let counts = map.goal_counts(2, commanded).unwrap();
        let measured = map.present_rad(2, counts).unwrap();
        // One count is 0.088°; rounding to nearest bounds the difference at
        // half of that.
        assert!((measured - commanded).abs() < PI / f64::from(COUNTS_PER_REV));
    }

    #[test]
    fn every_register_name_has_an_address_and_a_shape() {
        for reg in RegId::ALL {
            let entry = reg_for(reg);
            let kind = value_kind(reg);
            let expected_width = match kind {
                ValueKind::U8 => 1,
                ValueKind::U16 | ValueKind::Volts => 2,
                ValueKind::U32 | ValueKind::Radians => 4,
                ValueKind::Gains => 6,
            };
            assert_eq!(
                usize::from(entry.len),
                expected_width,
                "{reg} is a {kind} at {} bytes",
                entry.len
            );
        }
    }

    /// Every register name against the address and width the X-series control
    /// table gives it. Widths and distinctness alone leave any swap between two
    /// registers of the same width invisible — goal for present would write
    /// every goal to a read-only address and read each measurement back off the
    /// goal, so tracking would compare the goal against itself and report zero
    /// error while the head never moved.
    #[test]
    fn every_register_name_sits_at_its_own_control_table_address() {
        let table = [
            (RegId::TorqueEnable, 64u16, 1u8),
            (RegId::GoalPosition, 116, 4),
            (RegId::PresentPosition, 132, 4),
            (RegId::OperatingMode, 11, 1),
            (RegId::HomingOffset, 20, 4),
            (RegId::ReturnDelayTime, 9, 1),
            (RegId::MinPositionLimit, 52, 4),
            (RegId::MaxPositionLimit, 48, 4),
            (RegId::Shutdown, 63, 1),
            (RegId::DriveMode, 10, 1),
            (RegId::MaxVoltageLimit, 32, 2),
            (RegId::MinVoltageLimit, 34, 2),
            (RegId::CurrentLimit, 38, 2),
            (RegId::VelocityLimit, 44, 4),
            (RegId::TemperatureLimit, 31, 1),
            (RegId::BusWatchdog, 98, 1),
            (RegId::ProfileAcceleration, 108, 4),
            (RegId::ProfileVelocity, 112, 4),
            (RegId::PositionGains, 80, 6),
            (RegId::HardwareErrorStatus, 70, 1),
            (RegId::PresentInputVoltage, 144, 2),
            (RegId::ModelNumber, 0, 2),
        ];
        assert_eq!(
            table.len(),
            RegId::ALL.len(),
            "a register name without a row here is a name nothing pins"
        );
        for (reg, addr, len) in table {
            let entry = reg_for(reg);
            assert_eq!(entry.addr, addr, "{reg} address");
            assert_eq!(entry.len, len, "{reg} width");
            // The area follows from the address, and which side of the boundary
            // a register sits on is what refuses a write to it.
            assert_eq!(entry.is_eeprom(), addr < 64, "{reg} area");
        }
    }

    #[test]
    fn no_two_register_names_share_an_address() {
        for a in RegId::ALL {
            for b in RegId::ALL {
                if a == b {
                    continue;
                }
                assert_ne!(reg_for(a).addr, reg_for(b).addr, "{a} and {b}");
            }
        }
    }

    #[test]
    fn the_gain_span_goes_out_in_the_tables_order_not_the_fields() {
        let map = direct();
        let value = RegValue::Gains {
            p: 0x0300,
            i: 0x0200,
            d: 0x0100,
        };
        let raw = map.encode_value(1, RegId::PositionGains, value).unwrap();
        assert_eq!(
            raw.as_slice(),
            &[0x00, 0x01, 0x00, 0x02, 0x00, 0x03],
            "derivative first, proportional last"
        );
        assert_eq!(
            map.decode_value(1, RegId::PositionGains, &raw),
            Ok(value),
            "the round trip puts the fields back where they came from"
        );
    }

    #[test]
    fn integer_registers_cross_as_integers() {
        let map = shifted();
        // The travel limits are what a provisioning check compares against the
        // way the unit was set up, so the datum must not touch them.
        let raw = map
            .encode_value(1, RegId::MinPositionLimit, RegValue::U32(1502))
            .unwrap();
        assert_eq!(raw.as_slice(), &1502u32.to_le_bytes());
        assert_eq!(
            map.decode_value(1, RegId::MinPositionLimit, &raw),
            Ok(RegValue::U32(1502))
        );

        let byte = map
            .encode_value(1, RegId::OperatingMode, RegValue::U8(3))
            .unwrap();
        assert_eq!(byte.as_slice(), &[3]);
        assert_eq!(
            map.decode_value(1, RegId::OperatingMode, &byte),
            Ok(RegValue::U8(3))
        );

        let word = map
            .encode_value(0, RegId::CurrentLimit, RegValue::U16(2352))
            .unwrap();
        assert_eq!(word.as_slice(), &2352u16.to_le_bytes());
    }

    #[test]
    fn a_position_register_crosses_as_an_angle_under_the_datum() {
        let map = shifted();
        let raw = map
            .encode_value(1, RegId::GoalPosition, RegValue::Radians(FRAC_PI_2))
            .unwrap();
        assert_eq!(
            raw.i32(),
            Some(CENTRE_COUNTS),
            "leg 1's quarter-turn shift lands the model's 90 degrees at centre"
        );
        assert_eq!(
            map.decode_value(1, RegId::PresentPosition, &raw),
            Ok(RegValue::Radians(FRAC_PI_2))
        );
        // The same bytes on the yaw servo mean a different angle.
        assert_eq!(
            map.decode_value(0, RegId::PresentPosition, &raw),
            Ok(RegValue::Radians(0.0))
        );
    }

    #[test]
    fn the_supply_reading_crosses_as_volts() {
        let map = direct();
        let raw = RawValue::new(&120u16.to_le_bytes()).unwrap();
        let RegValue::Volts(volts) = map
            .decode_value(0, RegId::PresentInputVoltage, &raw)
            .unwrap()
        else {
            panic!("the supply register is a voltage");
        };
        assert!((volts - 12.0).abs() < 1e-12);

        let back = map
            .encode_value(0, RegId::PresentInputVoltage, RegValue::Volts(12.0))
            .unwrap();
        assert_eq!(back, raw);

        for bad in [f64::NAN, f64::INFINITY, -1.0, 1e9] {
            assert!(matches!(
                map.encode_value(0, RegId::PresentInputVoltage, RegValue::Volts(bad)),
                Err(MapError::Voltage { .. })
            ));
        }
    }

    #[test]
    fn a_value_of_the_wrong_shape_for_its_register_is_refused() {
        let map = direct();
        let failure = map
            .encode_value(1, RegId::TorqueEnable, RegValue::U16(1))
            .expect_err("torque enable is one byte");
        assert_eq!(
            failure,
            MapError::WrongShape {
                reg: RegId::TorqueEnable,
                expected: ValueKind::U8,
                observed: ValueKind::U16,
            }
        );
        assert!(matches!(
            map.encode_value(1, RegId::GoalPosition, RegValue::U32(2048)),
            Err(MapError::WrongShape {
                expected: ValueKind::Radians,
                ..
            })
        ));
    }

    #[test]
    fn bytes_that_do_not_fill_a_register_decode_to_nothing() {
        let map = direct();
        let two = RawValue::new(&[1, 2]).unwrap();
        assert_eq!(
            map.decode_value(1, RegId::PresentPosition, &two),
            Err(MapError::Width {
                reg: RegId::PresentPosition,
                expected: 4,
                actual: 2,
            })
        );
    }

    #[test]
    fn a_datum_names_itself() {
        assert_eq!(format!("{}", CrankDatum::Direct), "direct");
        assert_eq!(format!("{}", CrankDatum::ParityShifted), "parity shifted");
    }
}
