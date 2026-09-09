//! Joints, registers and counts — the one place all three meet.
//!
//! Above this module the machine is nine joints holding angles in radians.
//! Below it the machine is servo IDs, control-table addresses and encoder
//! counts. This map is the only translation between the two, and being the only
//! one is the point: a second register table could disagree with this one, and a
//! disagreement about which crank a goal belongs to puts the head on the desk.
//!
//! ## The datum
//!
//! A converted count *is* the crank angle the kinematic model means, with no
//! host-side correction anywhere. The quarter turn that separates a crank's
//! mechanical zero from the model's is provisioned into each leg servo's homing
//! offset register, applied by servo firmware before Present Position is
//! reported, and the vendor's own per-leg count limits are derived from the
//! crank-angle limits under exactly that bare conversion. A servo missing its
//! offset is a per-servo provisioning fault to be refused by name, never
//! something to compensate for here: a host-side shift would move all six legs
//! for a fault that is one servo's.
//!
//! ## Shapes
//!
//! Each register has exactly one engineering shape, and it is fixed here.
//! Position registers cross this boundary as radians; the supply-voltage reading
//! crosses as volts. Everything else crosses as the integer the control table
//! holds, because the provisioning checks compare those against a data sheet and
//! a converted angle would be the wrong thing to compare. The position-gain span
//! is the one register whose byte order is not its field order: the table runs
//! D, I, P.

use dxl_proto::regs;
use dxl_proto::{ConvError, Reg, counts_to_rad, rad_to_counts, raw_from_volts, volts_from_raw};
use reachy_motion::joints::ROW_COUNT;
use reachy_motion::reg::Name as RegName;
use reachy_motion::value;
use reachy_motion::{RegId, ShapeName, Value, ValueShape};
use thiserror::Error;

use crate::bus::{MAX_SYNC_IDS, RawValue};

// The map hands the grouped operations one entry per joint, and those refuse
// more IDs than a single frame carries. Tying the two counts here makes a roster
// that outgrew the frame a compile error rather than a refusal on hardware.
const _: () = assert!(ROW_COUNT <= MAX_SYNC_IDS);

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

    /// No register at all, where one had to be named. A transaction carrying
    /// `none` reaches nothing on the control table, and a `none` can arrive
    /// from a slot, so this is a refusal rather than a default.
    #[error("no register is named")]
    NoRegister,

    /// A value of the wrong engineering shape for the register named.
    #[error("{} is {}, not {}", RegName(*.reg), ShapeName(*.expected), ShapeName(*.observed))]
    WrongShape {
        /// The register.
        reg: RegId,
        /// The shape the register has.
        expected: ValueShape,
        /// The shape that arrived.
        observed: ValueShape,
    },

    /// Bytes that do not fill, or overfill, the register named.
    #[error("{} is {expected} bytes wide, these are {actual}", RegName(*.reg))]
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
    #[error("{}: {source}", RegName(*.reg))]
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
///
/// # Errors
///
/// [`MapError::NoRegister`] for [`RegId::None`], which names no entry: the
/// vocabulary's zero is what a ping carries and what an unwritten slot holds,
/// and it can arrive here from either.
pub fn reg_for(reg: RegId) -> Result<Reg, MapError> {
    let entry = match reg {
        RegId::None => return Err(MapError::NoRegister),
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
        RegId::PresentTemperature => regs::PRESENT_TEMPERATURE,
    };
    Ok(entry)
}

/// The control-table entry a register named in the source sits at.
///
/// [`reg_for`]'s one refusal is [`RegId::None`], which a caller naming a register
/// at compile time cannot have written — so a constant-register lookup has no
/// failure to report, and reporting one anyway puts a line an operator cannot act
/// on into an incident. A register arriving from a slot, a datagram or a sweep
/// goes through [`reg_for`] and propagates.
///
/// # Panics
///
/// If handed [`RegId::None`].
#[must_use]
pub fn named_reg(reg: RegId) -> Reg {
    reg_for(reg).expect("a register named in the source is not the no-register zero")
}

/// The engineering shape a register's value takes above this crate.
///
/// Only the two position registers a command path touches are angles. The
/// travel limits and the homing offset stay integers: they are compared
/// against how the unit was provisioned, and an angle is not that comparison.
/// The homing offset is the one signed integer among them — this platform's
/// legs carry a quarter turn of either sign — and reading it unsigned renders a
/// negative offset as a number near four billion.
///
/// # Errors
///
/// [`MapError::NoRegister`] for [`RegId::None`]: the absence of a register has
/// no engineering shape, and answering [`ValueShape::None`] would let a
/// transaction about nothing carry a value.
pub fn value_kind(reg: RegId) -> Result<ValueShape, MapError> {
    let shape = match reg {
        RegId::None => return Err(MapError::NoRegister),
        RegId::GoalPosition | RegId::PresentPosition => ValueShape::Radians,
        RegId::PresentInputVoltage => ValueShape::Volts,
        RegId::PositionGains => ValueShape::Gains,
        RegId::TorqueEnable
        | RegId::OperatingMode
        | RegId::ReturnDelayTime
        | RegId::Shutdown
        | RegId::DriveMode
        | RegId::TemperatureLimit
        | RegId::BusWatchdog
        | RegId::HardwareErrorStatus
        | RegId::PresentTemperature => ValueShape::U8,
        RegId::MaxVoltageLimit
        | RegId::MinVoltageLimit
        | RegId::CurrentLimit
        | RegId::ModelNumber => ValueShape::U16,
        RegId::HomingOffset => ValueShape::I32,
        RegId::MinPositionLimit
        | RegId::MaxPositionLimit
        | RegId::VelocityLimit
        | RegId::ProfileAcceleration
        | RegId::ProfileVelocity => ValueShape::U32,
    };
    Ok(shape)
}

/// Why the shape accessors cannot fail once the bytes match the register's
/// width: every register's declared width is exactly the width its engineering
/// shape needs, which `every_register_name_has_an_address_and_a_shape` pins for
/// all of them.
const SHAPE_FITS_WIDTH: &str = "a register's width is the width its shape needs";

/// Why a reader on the arm its own shape names cannot fail.
const SHAPE_IS_CHECKED: &str = "the arm is the shape the value carries";

/// Why no register reaches the shapeless arm: every register in the table is
/// provisioned with a shape, which `every_register_name_has_an_address_and_a_shape`
/// asserts.
const TABLE_NAMES_A_SHAPE: &str = "every register the table names has a value shape";

/// One joint's wire identity.
#[derive(Clone, Copy, Debug, PartialEq)]
struct JointServo {
    id: u8,
}

/// Which servo each joint is, and what its counts mean.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ServoMap {
    joints: [JointServo; ROW_COUNT],
}

impl ServoMap {
    /// A map over `ids` in bus order — body yaw, legs 1..=6, right antenna,
    /// left antenna.
    #[must_use]
    pub fn new(ids: [u8; ROW_COUNT]) -> Self {
        let mut joints = [JointServo { id: 0 }; ROW_COUNT];
        for (slot, id) in joints.iter_mut().zip(ids.iter()) {
            *slot = JointServo { id: *id };
        }
        Self { joints }
    }

    /// The servo IDs, in bus order.
    #[must_use]
    pub fn ids(&self) -> [u8; ROW_COUNT] {
        let mut ids = [0; ROW_COUNT];
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

    /// A measured count as the model's angle.
    ///
    /// The bare conversion: the servo's provisioned homing offset has already
    /// been applied by its own firmware, so what arrives here is the model's
    /// angle in counts.
    pub fn present_rad(&self, joint: usize, counts: i32) -> Result<f64, MapError> {
        self.servo(joint)?;
        Ok(counts_to_rad(counts))
    }

    /// A model angle as the count to command.
    ///
    /// Rounds to the nearest count and refuses anything no count represents.
    pub fn goal_counts(&self, joint: usize, rad: f64) -> Result<i32, MapError> {
        let servo = self.servo(joint)?;
        rad_to_counts(rad).map_err(|source| MapError::Angle {
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
        value: Value,
    ) -> Result<RawValue, MapError> {
        let expected = value_kind(reg)?;
        let shape = value.shape();
        if shape != expected {
            return Err(MapError::WrongShape {
                reg,
                expected,
                observed: shape,
            });
        }
        match shape {
            ValueShape::None => unreachable!("{TABLE_NAMES_A_SHAPE}"),
            ValueShape::U8 => self.carry(reg, &[value.as_u8().expect(SHAPE_IS_CHECKED)]),
            ValueShape::U16 => {
                self.carry(reg, &value.as_u16().expect(SHAPE_IS_CHECKED).to_le_bytes())
            }
            ValueShape::U32 => {
                self.carry(reg, &value.as_u32().expect(SHAPE_IS_CHECKED).to_le_bytes())
            }
            ValueShape::I32 => {
                self.carry(reg, &value.as_i32().expect(SHAPE_IS_CHECKED).to_le_bytes())
            }
            ValueShape::Radians => {
                let counts =
                    self.goal_counts(joint, value.as_radians().expect(SHAPE_IS_CHECKED))?;
                self.carry(reg, &counts.to_le_bytes())
            }
            ValueShape::Volts => {
                let units = raw_from_volts(value.as_volts().expect(SHAPE_IS_CHECKED))
                    .map_err(|source| MapError::Voltage { reg, source })?;
                self.carry(reg, &units.to_le_bytes())
            }
            ValueShape::Gains => {
                let (p, i, d) = value.as_gains().expect(SHAPE_IS_CHECKED);
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
    ) -> Result<Value, MapError> {
        let width = usize::from(reg_for(reg)?.len);
        if raw.len() != width {
            return Err(MapError::Width {
                reg,
                expected: width,
                actual: raw.len(),
            });
        }
        match value_kind(reg)? {
            ValueShape::None => unreachable!("{TABLE_NAMES_A_SHAPE}"),
            ValueShape::U8 => Ok(value::u8(raw.u8().expect(SHAPE_FITS_WIDTH))),
            ValueShape::U16 => Ok(value::u16(raw.u16().expect(SHAPE_FITS_WIDTH))),
            ValueShape::U32 => Ok(value::u32(raw.u32().expect(SHAPE_FITS_WIDTH))),
            ValueShape::I32 => Ok(value::i32(raw.i32().expect(SHAPE_FITS_WIDTH))),
            ValueShape::Radians => {
                let counts = raw.i32().expect(SHAPE_FITS_WIDTH);
                Ok(value::radians(self.present_rad(joint, counts)?))
            }
            ValueShape::Volts => {
                let units = raw.u16().expect(SHAPE_FITS_WIDTH);
                Ok(value::volts(volts_from_raw(units)))
            }
            ValueShape::Gains => {
                let bytes = raw.as_slice();
                let word = |at: usize| u16::from_le_bytes([bytes[at], bytes[at + 1]]);
                Ok(value::gains(word(4), word(2), word(0)))
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
        let width = usize::from(reg_for(reg)?.len);
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
    use reachy_motion::Shown;
    use reachy_motion::reg;

    /// The nine servo IDs this machine uses, in bus order.
    const IDS: [u8; ROW_COUNT] = [10, 11, 12, 13, 14, 15, 16, 17, 18];

    fn direct() -> ServoMap {
        ServoMap::new(IDS)
    }

    #[test]
    fn every_joint_maps_to_its_own_servo_in_bus_order() {
        let map = direct();
        assert_eq!(map.ids(), IDS);
        for (index, id) in IDS.iter().enumerate() {
            assert_eq!(map.id_at(index), Some(*id));
        }
        assert_eq!(map.id_at(ROW_COUNT), None);
        assert_eq!(
            map.present_rad(ROW_COUNT, 0),
            Err(MapError::UnknownJoint { joint: ROW_COUNT })
        );
        assert_eq!(
            map.goal_counts(ROW_COUNT, 0.0),
            Err(MapError::UnknownJoint { joint: ROW_COUNT })
        );
    }

    /// Every joint's counts convert the same way: the offset that separates a
    /// crank's mechanical zero from the model's lives in the servo, so nothing
    /// here moves one joint's reading relative to another's.
    #[test]
    fn a_count_is_the_model_angle_on_every_joint_alike() {
        let map = direct();
        for joint in 0..ROW_COUNT {
            assert!(
                map.present_rad(joint, CENTRE_COUNTS).unwrap().abs() < 1e-12,
                "joint {joint}"
            );
            assert_eq!(
                map.goal_counts(joint, 0.0),
                Ok(CENTRE_COUNTS),
                "joint {joint}"
            );
        }
    }

    #[test]
    fn a_count_and_an_angle_round_trip() {
        let map = direct();
        for joint in 0..ROW_COUNT {
            for counts in [0, 1000, CENTRE_COUNTS, 3000, COUNTS_PER_REV] {
                let rad = map.present_rad(joint, counts).unwrap();
                assert_eq!(
                    map.goal_counts(joint, rad),
                    Ok(counts),
                    "joint {joint} at {counts} counts"
                );
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
        // What the tick relies on: a goal written and read back is the angle
        // that was commanded.
        let map = direct();
        let commanded = 0.3;
        let counts = map.goal_counts(2, commanded).unwrap();
        let measured = map.present_rad(2, counts).unwrap();
        // One count is 0.088°; rounding to nearest bounds the difference at
        // half of that.
        assert!((measured - commanded).abs() < PI / f64::from(COUNTS_PER_REV));
    }

    #[test]
    fn every_register_name_has_an_address_and_a_shape() {
        for reg in reg::named() {
            let entry = reg_for(reg).expect("a named register has an entry");
            let kind = value_kind(reg).expect("a named register has a shape");
            let expected_width = match kind {
                // Every register the table names has a value; the shape that is
                // the absence of one belongs to a transaction, not a register.
                ValueShape::None => panic!("{reg} is provisioned with no value shape"),
                ValueShape::U8 => 1,
                ValueShape::U16 | ValueShape::Volts => 2,
                ValueShape::U32 | ValueShape::I32 | ValueShape::Radians => 4,
                ValueShape::Gains => 6,
            };
            assert_eq!(
                usize::from(entry.len),
                expected_width,
                "{reg} is a {} at {} bytes",
                ShapeName(kind),
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
            (RegId::PresentTemperature, 146, 1),
        ];
        assert_eq!(
            table.len(),
            reg::named().count(),
            "a register name without a row here is a name nothing pins"
        );
        for (reg, addr, len) in table {
            let entry = reg_for(reg).expect("a named register has an entry");
            assert_eq!(entry.addr, addr, "{reg} address");
            assert_eq!(entry.len, len, "{reg} width");
            // The area follows from the address, and which side of the boundary
            // a register sits on is what refuses a write to it.
            assert_eq!(entry.is_eeprom(), addr < 64, "{reg} area");
        }
    }

    #[test]
    fn no_two_register_names_share_an_address() {
        for a in reg::named() {
            for b in reg::named() {
                if a == b {
                    continue;
                }
                let (a_entry, b_entry) = (reg_for(a).expect("named"), reg_for(b).expect("named"));
                assert_ne!(a_entry.addr, b_entry.addr, "{a} and {b}");
            }
        }
    }

    /// The vocabulary's zero names no control-table entry and no shape, and the
    /// refusal travels: a transaction about no register reaches no address.
    ///
    /// It can arrive from a slot, so the accessors refuse it rather than
    /// defaulting; without this the arms are deletable — a `RegId::None` row
    /// added to either table builds green, and the failure is a transaction about
    /// nothing addressing torque enable.
    #[test]
    fn the_no_register_zero_names_no_entry_and_no_shape() {
        assert!(matches!(reg_for(RegId::None), Err(MapError::NoRegister)));
        assert!(matches!(value_kind(RegId::None), Err(MapError::NoRegister)));
        assert_eq!(MapError::NoRegister.to_string(), "no register is named");

        let map = direct();
        assert!(
            matches!(
                map.encode_value(1, RegId::None, value::u8(1)),
                Err(MapError::NoRegister)
            ),
            "a write of no register is refused before any width is consulted"
        );
        let raw = RawValue::new(&[0]).expect("one byte is a raw value");
        assert!(
            matches!(
                map.decode_value(1, RegId::None, &raw),
                Err(MapError::NoRegister)
            ),
            "bytes read back for no register name no value"
        );
    }

    /// A register named in the source is never the zero, and the shortcut over
    /// [`reg_for`] says so by crashing rather than resolving one.
    #[test]
    #[should_panic(expected = "a register named in the source is not the no-register zero")]
    fn a_constant_register_lookup_of_the_zero_panics() {
        let _ = named_reg(RegId::None);
    }

    #[test]
    fn the_gain_span_goes_out_in_the_tables_order_not_the_fields() {
        let map = direct();
        let value = value::gains(0x0300, 0x0200, 0x0100);
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
        let map = direct();
        // The travel limits are what a provisioning check compares against the
        // way the unit was set up, so they cross as the counts the register
        // holds and never as angles.
        let raw = map
            .encode_value(1, RegId::MinPositionLimit, value::u32(1502))
            .unwrap();
        assert_eq!(raw.as_slice(), &1502u32.to_le_bytes());
        assert_eq!(
            map.decode_value(1, RegId::MinPositionLimit, &raw),
            Ok(value::u32(1502))
        );

        let byte = map
            .encode_value(1, RegId::OperatingMode, value::u8(3))
            .unwrap();
        assert_eq!(byte.as_slice(), &[3]);
        assert_eq!(
            map.decode_value(1, RegId::OperatingMode, &byte),
            Ok(value::u8(3))
        );

        let word = map
            .encode_value(0, RegId::CurrentLimit, value::u16(2352))
            .unwrap();
        assert_eq!(word.as_slice(), &2352u16.to_le_bytes());
    }

    /// The homing offset is the one signed register: this platform's legs carry
    /// a quarter turn of either sign, and read unsigned the negative ones report
    /// as a span near four billion.
    #[test]
    fn the_homing_offset_crosses_as_a_signed_integer() {
        let map = direct();
        let raw = map
            .encode_value(2, RegId::HomingOffset, value::i32(-1024))
            .unwrap();
        assert_eq!(raw.as_slice(), &(-1024i32).to_le_bytes());
        assert_eq!(raw.as_slice(), &0xFFFF_FC00u32.to_le_bytes());
        let decoded = map.decode_value(2, RegId::HomingOffset, &raw).unwrap();
        assert_eq!(decoded, value::i32(-1024));
        assert_eq!(Shown(decoded).to_string(), "-1024");

        assert_eq!(
            map.encode_value(2, RegId::HomingOffset, value::u32(1024)),
            Err(MapError::WrongShape {
                reg: RegId::HomingOffset,
                expected: ValueShape::I32,
                observed: ValueShape::U32,
            })
        );
    }

    #[test]
    fn a_position_register_crosses_as_an_angle() {
        let map = direct();
        let raw = map
            .encode_value(1, RegId::GoalPosition, value::radians(0.0))
            .unwrap();
        assert_eq!(
            raw.i32(),
            Some(CENTRE_COUNTS),
            "the model's zero is the register's centre count"
        );
        assert_eq!(
            map.decode_value(1, RegId::PresentPosition, &raw),
            Ok(value::radians(0.0))
        );
        // The same bytes mean the same angle on every joint: nothing here is
        // per-joint, which is what makes a provisioning fault a servo's own.
        assert_eq!(
            map.decode_value(0, RegId::PresentPosition, &raw),
            Ok(value::radians(0.0))
        );
    }

    #[test]
    fn the_supply_reading_crosses_as_volts() {
        let map = direct();
        let raw = RawValue::new(&120u16.to_le_bytes()).unwrap();
        let read = map
            .decode_value(0, RegId::PresentInputVoltage, &raw)
            .unwrap();
        let volts = read.as_volts().expect("the supply register is a voltage");
        assert!((volts - 12.0).abs() < 1e-12);

        let back = map
            .encode_value(0, RegId::PresentInputVoltage, value::volts(12.0))
            .unwrap();
        assert_eq!(back, raw);

        for bad in [f64::NAN, f64::INFINITY, -1.0, 1e9] {
            assert!(matches!(
                map.encode_value(0, RegId::PresentInputVoltage, value::volts(bad)),
                Err(MapError::Voltage { .. })
            ));
        }
    }

    #[test]
    fn a_value_of_the_wrong_shape_for_its_register_is_refused() {
        let map = direct();
        let failure = map
            .encode_value(1, RegId::TorqueEnable, value::u16(1))
            .expect_err("torque enable is one byte");
        assert_eq!(
            failure,
            MapError::WrongShape {
                reg: RegId::TorqueEnable,
                expected: ValueShape::U8,
                observed: ValueShape::U16,
            }
        );
        assert!(matches!(
            map.encode_value(1, RegId::GoalPosition, value::u32(2048)),
            Err(MapError::WrongShape {
                expected: ValueShape::Radians,
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
}
