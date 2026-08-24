//! The modelled servos' control tables: one cell per register per bus row.
//!
//! What a real driver reaches over the wire, the simulated one reaches here. A
//! cell holds the register's *engineering-unit* value in the shape-tagged
//! carriage a transaction uses, never a servo count: counts are the bus layer's
//! and nothing above it converts, so a simulator that stored counts would be
//! modelling the one layer it does not contain.
//!
//! Which shape a register's value takes, and whether writing it needs torque
//! off, are the bus table's answers and are asked of it rather than restated.
//! That is the whole reason this file is thin: the register vocabulary, the
//! shapes and the volatility classification all exist, and what is added here
//! is nine rows of storage and the two accessors that keep a cell's bits and
//! its shape in agreement.
//!
//! The live cells -- where a servo says it is, what it is holding, whether it is
//! energised -- are refreshed from the plant every cycle, so a read of them
//! answers about the modelled machine now rather than about the last thing
//! written. Every other cell is provisioning: written once at start-up to what a
//! healthy, correctly provisioned unit holds, and thereafter only by whatever
//! writes registers.

use brenn_reachy__hardware__dynamixel__registers_clk_rs::{RegId, ValueShape};
use reachy_bus::{reg_for, value_kind};
use reachy_motion::arm::{
    DEFAULT_GAINS, DEFAULT_MIN_ARM_VOLTAGE, EXPECTED_MODELS, EXPECTED_OPERATING_MODES,
    VENDOR_HOMING_OFFSETS,
};
use reachy_motion::joints::{ROW_COUNT, ROWS};
use reachy_motion::value::{self, Value};

/// How many registers one row's block of cells carries.
///
/// Every register the vocabulary names except its zero, which names none: a
/// ping carries that zero and an unwritten slot holds it, and neither is a
/// register with a value. Derived from the vocabulary rather than counted by
/// hand, so a register added there widens the file instead of falling off the
/// end of it.
pub const REG_COUNT: usize = RegId::VARIANTS.len() - 1;

/// How many cells the file holds: nine rows of registers.
pub const CELL_COUNT: usize = ROW_COUNT * REG_COUNT;

/// The cells the plant owns, in the order [`Regs::refresh`] writes them.
///
/// What a servo reports about itself: where it is, what it is holding, whether
/// it is energised. Every one of them is written from the modelled plant every
/// cycle, so a value put in one from anywhere else is gone before anything reads
/// it. That makes them the registers an injection cannot have -- a scenario
/// reaches these by moving the machine, and [`is_plant_owned`] is what lets a
/// caller refuse the write instead of losing it.
pub const PLANT_OWNED: [RegId; 3] = [
    RegId::PresentPosition,
    RegId::GoalPosition,
    RegId::TorqueEnable,
];

/// Whether the plant owns this register's cells.
#[must_use]
pub fn is_plant_owned(reg: RegId) -> bool {
    PLANT_OWNED.contains(&reg)
}

/// The supply the modelled rail sits at, volts.
///
/// Above the floor torque is switched on at, with room: what commissioning
/// waits for is a rail that is up, and a simulator whose rail sat on the
/// threshold would make the wait's outcome a rounding question. Not a scenario
/// parameter -- a scenario that wants a sagging rail writes the voltage
/// register.
const NOMINAL_VOLTS: f64 = 7.4;

const _: () = assert!(NOMINAL_VOLTS > DEFAULT_MIN_ARM_VOLTAGE);

/// Why the register file refused.
///
/// A refusal and never a nearby cell: a caller naming a register that does not
/// exist, or a value of the wrong shape for the one it does, has a bug that a
/// silently substituted cell would hide until a real bus answered differently.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegRefusal {
    /// The no-register zero, which names no cell. A ping carries it and a slot
    /// nothing wrote holds it, so it arrives here and is answered rather than
    /// indexed with.
    NoRegister,
    /// A row past the nine the bus carries.
    RowNotOnBus {
        /// The row asked for.
        row: usize,
    },
    /// A value whose shape is not the one the register's value takes.
    WrongShape {
        /// The register.
        reg: RegId,
        /// The shape the register has.
        expected: ValueShape,
        /// The shape that arrived.
        observed: ValueShape,
    },
    /// An angle or a voltage that is not a number. Refused at the write rather
    /// than carried, so every finite-shaped cell in the file is finite and the
    /// readers of one do not each need their own check.
    NotCarriable {
        /// The register.
        reg: RegId,
    },
    /// A write to a non-volatile register on a row that is holding torque. The
    /// real bus refuses this outright because a servo ignores such a write and
    /// acknowledges it anyway, which is the worst available outcome.
    NonVolatileUnderTorque {
        /// The register.
        reg: RegId,
        /// The row.
        row: usize,
    },
}

/// Which cell a row's register is.
///
/// # Errors
///
/// [`RegRefusal::NoRegister`] for the no-register zero, which names no cell and
/// arrives here from a ping or from a slot nothing wrote;
/// [`RegRefusal::RowNotOnBus`] for a row the bus does not have.
fn cell(row: usize, reg: RegId) -> Result<usize, RegRefusal> {
    if reg == RegId::None {
        return Err(RegRefusal::NoRegister);
    }
    if row >= ROW_COUNT {
        return Err(RegRefusal::RowNotOnBus { row });
    }
    Ok(row * REG_COUNT + reg as usize - 1)
}

/// What a row's register holds.
///
/// A reader rather than a method, because the cells are read where only the
/// state is in hand -- the pose sample is a read of these registers -- and
/// taking up the writable view to answer a question is a borrow nothing needs.
///
/// # Errors
///
/// [`cell`]'s, and [`RegRefusal::NoRegister`] for a register with no engineering
/// shape, which is the same zero.
pub fn read(cells: &[u64; CELL_COUNT], row: usize, reg: RegId) -> Result<Value, RegRefusal> {
    let cell = cell(row, reg)?;
    let shape = value_kind(reg).map_err(|_| RegRefusal::NoRegister)?;
    Ok(value::carried(shape, cells[cell]))
}

/// The nine present-position cells, radians in bus order.
///
/// The reading a cycle's pose sample carries: the sample *is* a read of these
/// registers, and taking it from anywhere else would be a second route from the
/// plant to what the machine reports about itself.
///
/// Every refusal [`read`] has is about a register or a row named from outside,
/// and both are named in the source here; a cell that somehow answered nothing
/// reads as zero rather than stopping the one publication a cycle owes.
#[must_use]
pub fn present_rows(cells: &[u64; CELL_COUNT]) -> [f64; ROW_COUNT] {
    let mut rows = [0.0; ROW_COUNT];
    for (row, out) in rows.iter_mut().enumerate() {
        *out = read(cells, row, RegId::PresentPosition)
            .ok()
            .and_then(Value::as_radians)
            .unwrap_or(0.0);
    }
    rows
}

/// The nine control tables, decided over the cells they live in.
///
/// Borrows the cells and holds nothing of its own, so the file a cycle reads and
/// the file it writes are the same field of the same state.
pub struct Regs<'a> {
    /// The cells, row-major.
    cells: &'a mut [u64; CELL_COUNT],
}

impl<'a> Regs<'a> {
    /// Take up the cells a state carries.
    pub fn over(cells: &'a mut [u64; CELL_COUNT]) -> Self {
        Self { cells }
    }

    /// Write the provisioning every healthy, correctly provisioned unit holds.
    ///
    /// The expectations are the motion library's own constants, so the
    /// commissioning sequence verifies here against the same numbers it verifies
    /// against on hardware -- a simulator carrying its own idea of a provisioned
    /// servo would pass a commission the machine fails.
    ///
    /// Only the cells named below are written. Every other cell stays at zero,
    /// which reads back as its register's own shape carrying zero, and the live
    /// cells are [`Self::refresh`]'s.
    ///
    /// The temperature cell carries the same constant the health report does, so
    /// a host reading that register over the seam and a host reading the report
    /// get one answer about one servo.
    pub fn init(&mut self) {
        for (row, joint) in ROWS.into_iter().enumerate() {
            let gains = DEFAULT_GAINS.for_joint(joint);
            for (reg, held) in [
                (RegId::ModelNumber, value::u16(EXPECTED_MODELS[row])),
                (
                    RegId::OperatingMode,
                    value::u8(EXPECTED_OPERATING_MODES[row]),
                ),
                (RegId::HomingOffset, value::i32(VENDOR_HOMING_OFFSETS[row])),
                (RegId::PositionGains, gains.value()),
                (RegId::PresentInputVoltage, value::volts(NOMINAL_VOLTS)),
                (RegId::HardwareErrorStatus, value::u8(0)),
                (
                    RegId::PresentTemperature,
                    value::u8(crate::sim_aux::SIM_TEMP_C.cast_unsigned()),
                ),
            ] {
                self.put(row, reg, held);
            }
        }
    }

    /// Bring the live cells up to date with the plant.
    ///
    /// Where the servos are, what they are holding, and which of them are
    /// energised: the three readings that answer about the machine now rather
    /// than about the last thing written to it. Run every cycle before anything
    /// reads a register, so a verification of a de-torquing or of a stow reads
    /// where the plant actually is.
    ///
    /// Angles arrive from the plant's own vector, which nothing guarantees is
    /// finite; a non-finite one is left out rather than stored, so the file's
    /// invariant holds and the row reads as the last number the plant had.
    /// Answers with how many readings were left out that way, which is the only
    /// evidence a caller has that a cell stopped tracking the plant.
    pub fn refresh(
        &mut self,
        positions: &[f64; ROW_COUNT],
        targets: &[f64; ROW_COUNT],
        torqued: &[bool; ROW_COUNT],
    ) -> u32 {
        let mut refused = 0;
        for row in 0..ROW_COUNT {
            // Paired with [`PLANT_OWNED`] positionally, so the registers this
            // writes and the registers an injection is refused for are one list.
            for (reg, held) in PLANT_OWNED.into_iter().zip([
                value::radians(positions[row]),
                value::radians(targets[row]),
                value::u8(u8::from(torqued[row])),
            ]) {
                if held.is_carriable() {
                    self.put(row, reg, held);
                } else {
                    refused += 1;
                }
            }
        }
        refused
    }

    /// What a row's register holds, over the view a cycle already took up.
    ///
    /// # Errors
    ///
    /// [`read`]'s.
    pub fn read(&self, row: usize, reg: RegId) -> Result<Value, RegRefusal> {
        read(self.cells, row, reg)
    }

    /// Write a row's register from bits whose shape the bus table decides.
    ///
    /// What a caller holding a register and eight bytes has: an injection out of
    /// an input log, a transaction off a channel. The shape is the register's
    /// own, so the pairing cannot be wrong -- which leaves the finiteness and
    /// volatility refusals of [`Self::write`] as the ones this can report.
    ///
    /// # Errors
    ///
    /// [`Self::write`]'s, less [`RegRefusal::WrongShape`].
    pub fn write_bits(
        &mut self,
        row: usize,
        reg: RegId,
        bits: u64,
        torqued: bool,
    ) -> Result<Value, RegRefusal> {
        let shape = value_kind(reg).map_err(|_| RegRefusal::NoRegister)?;
        self.write(row, reg, value::carried(shape, bits), torqued)
    }

    /// Write a row's register, and answer with what it holds afterwards.
    ///
    /// `torqued` is whether the row is energised, which decides the
    /// non-volatile refusal: the real bus reads torque rather than assuming it,
    /// and this mirrors that rule rather than restating which registers it
    /// covers.
    ///
    /// # Errors
    ///
    /// [`RegRefusal::NoRegister`], [`RegRefusal::RowNotOnBus`],
    /// [`RegRefusal::WrongShape`] for a value that is not the register's shape,
    /// [`RegRefusal::NotCarriable`] for a non-finite angle or voltage, or
    /// [`RegRefusal::NonVolatileUnderTorque`].
    pub fn write(
        &mut self,
        row: usize,
        reg: RegId,
        held: Value,
        torqued: bool,
    ) -> Result<Value, RegRefusal> {
        let cell = cell(row, reg)?;
        let expected = value_kind(reg).map_err(|_| RegRefusal::NoRegister)?;
        if held.shape() != expected {
            return Err(RegRefusal::WrongShape {
                reg,
                expected,
                observed: held.shape(),
            });
        }
        if !held.is_carriable() {
            return Err(RegRefusal::NotCarriable { reg });
        }
        let entry = reg_for(reg).map_err(|_| RegRefusal::NoRegister)?;
        if entry.is_eeprom() && torqued {
            return Err(RegRefusal::NonVolatileUnderTorque { reg, row });
        }
        self.cells[cell] = held.bits();
        Ok(value::carried(expected, self.cells[cell]))
    }

    /// Set a cell whose register and shape the caller named in the source.
    ///
    /// The refusals [`Self::write`] reports are all about a register or a value
    /// that arrived from outside; a constant register with a constructed value
    /// has none of them, and the provisioning writes below are all of that kind.
    fn put(&mut self, row: usize, reg: RegId, held: Value) {
        if let Ok(cell) = cell(row, reg) {
            self.cells[cell] = held.bits();
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A file of cells a case can hand out the borrow of.
    fn cells() -> [u64; CELL_COUNT] {
        [0; CELL_COUNT]
    }

    /// The nine angles a case refreshes with: a distinct number per row, so a
    /// reading that came out of the wrong cell is a number no other row holds.
    fn ramp(base: f64) -> [f64; ROW_COUNT] {
        let mut rows = [0.0; ROW_COUNT];
        for (row, out) in rows.iter_mut().enumerate() {
            *out = base + row as f64 / 100.0;
        }
        rows
    }

    #[test]
    fn every_register_of_every_row_has_a_cell_of_its_own() {
        let mut seen = [false; CELL_COUNT];
        for row in 0..ROW_COUNT {
            for reg in RegId::VARIANTS
                .into_iter()
                .filter(|reg| *reg != RegId::None)
            {
                let index = cell(row, reg).expect("a row and a register both name one");
                assert!(!seen[index], "{reg:?} on row {row} shares a cell");
                seen[index] = true;
            }
        }
        assert!(seen.into_iter().all(|used| used), "every cell is reachable");
    }

    #[test]
    fn the_no_register_zero_names_no_cell() {
        let mut held = cells();
        assert_eq!(read(&held, 0, RegId::None), Err(RegRefusal::NoRegister));
        assert_eq!(
            Regs::over(&mut held).write_bits(0, RegId::None, 1, false),
            Err(RegRefusal::NoRegister),
        );
    }

    #[test]
    fn a_row_past_the_bus_names_no_cell() {
        let mut held = cells();
        assert_eq!(
            read(&held, ROW_COUNT, RegId::TorqueEnable),
            Err(RegRefusal::RowNotOnBus { row: ROW_COUNT }),
        );
        assert_eq!(
            Regs::over(&mut held).write_bits(ROW_COUNT, RegId::TorqueEnable, 1, false),
            Err(RegRefusal::RowNotOnBus { row: ROW_COUNT }),
        );
    }

    #[test]
    fn an_unwritten_cell_reads_as_its_own_shape_carrying_zero() {
        let held = cells();
        assert_eq!(
            read(&held, 3, RegId::HomingOffset),
            Ok(value::i32(0)),
            "the shape is the register's whatever the bits are",
        );
        assert_eq!(
            read(&held, 3, RegId::PresentPosition),
            Ok(value::radians(0.0))
        );
    }

    #[test]
    fn provisioning_reads_back_what_a_healthy_unit_holds() {
        let mut held = cells();
        Regs::over(&mut held).init();
        for (row, joint) in ROWS.into_iter().enumerate() {
            assert_eq!(
                read(&held, row, RegId::ModelNumber),
                Ok(value::u16(EXPECTED_MODELS[row])),
            );
            assert_eq!(
                read(&held, row, RegId::OperatingMode),
                Ok(value::u8(EXPECTED_OPERATING_MODES[row])),
            );
            assert_eq!(
                read(&held, row, RegId::HomingOffset),
                Ok(value::i32(VENDOR_HOMING_OFFSETS[row])),
            );
            assert_eq!(
                read(&held, row, RegId::PositionGains),
                Ok(DEFAULT_GAINS.for_joint(joint).value()),
            );
            assert_eq!(
                read(&held, row, RegId::PresentInputVoltage)
                    .expect("a rail reading")
                    .as_volts(),
                Some(NOMINAL_VOLTS),
                "the modelled rail is up",
            );
            assert_eq!(
                read(&held, row, RegId::HardwareErrorStatus),
                Ok(value::u8(0)),
                "nothing is complaining",
            );
        }
    }

    #[test]
    fn the_live_cells_follow_the_plant() {
        let mut held = cells();
        let positions = ramp(0.5);
        let targets = ramp(1.5);
        let mut torqued = [false; ROW_COUNT];
        torqued[2] = true;

        Regs::over(&mut held).refresh(&positions, &targets, &torqued);

        assert_eq!(present_rows(&held), positions);
        for row in 0..ROW_COUNT {
            assert_eq!(
                read(&held, row, RegId::GoalPosition),
                Ok(value::radians(targets[row])),
            );
            assert_eq!(
                read(&held, row, RegId::TorqueEnable),
                Ok(value::u8(u8::from(torqued[row]))),
                "row {row} says whether it is energised",
            );
        }
    }

    #[test]
    fn a_non_finite_angle_never_reaches_a_cell() {
        let mut held = cells();
        let mut positions = ramp(0.5);
        assert_eq!(
            Regs::over(&mut held).refresh(&positions, &positions, &[false; ROW_COUNT]),
            0,
            "a finite plant is a plant every reading of which lands",
        );

        positions[4] = f64::NAN;
        assert_eq!(
            Regs::over(&mut held).refresh(&positions, &positions, &[false; ROW_COUNT]),
            2,
            "the row's present and goal readings are both counted, not stored",
        );
        assert_eq!(
            present_rows(&held)[4],
            ramp(0.5)[4],
            "the row keeps the last number the plant had",
        );

        assert_eq!(
            Regs::over(&mut held).write(
                4,
                RegId::PresentPosition,
                value::radians(f64::INFINITY),
                false,
            ),
            Err(RegRefusal::NotCarriable {
                reg: RegId::PresentPosition,
            }),
        );
    }

    #[test]
    fn a_value_of_the_wrong_shape_is_refused() {
        let mut held = cells();
        assert_eq!(
            Regs::over(&mut held).write(0, RegId::HardwareErrorStatus, value::u16(3), false),
            Err(RegRefusal::WrongShape {
                reg: RegId::HardwareErrorStatus,
                expected: ValueShape::U8,
                observed: ValueShape::U16,
            }),
        );
    }

    #[test]
    fn a_write_answers_with_what_the_cell_holds() {
        let mut held = cells();
        assert_eq!(
            Regs::over(&mut held).write_bits(1, RegId::HardwareErrorStatus, 0x20, false),
            Ok(value::u8(0x20)),
        );
        assert_eq!(
            read(&held, 1, RegId::HardwareErrorStatus),
            Ok(value::u8(0x20))
        );
        assert_eq!(
            read(&held, 0, RegId::HardwareErrorStatus),
            Ok(value::u8(0)),
            "the write reached one row",
        );
    }

    #[test]
    fn a_non_volatile_register_takes_a_write_only_with_torque_off() {
        let mut held = cells();
        Regs::over(&mut held).init();
        assert_eq!(
            Regs::over(&mut held).write(0, RegId::OperatingMode, value::u8(4), true),
            Err(RegRefusal::NonVolatileUnderTorque {
                reg: RegId::OperatingMode,
                row: 0,
            }),
        );
        assert_eq!(
            read(&held, 0, RegId::OperatingMode),
            Ok(value::u8(EXPECTED_OPERATING_MODES[0])),
            "a refused write changes nothing",
        );
        assert_eq!(
            Regs::over(&mut held).write(0, RegId::OperatingMode, value::u8(4), false),
            Ok(value::u8(4)),
        );
    }

    #[test]
    fn a_volatile_register_takes_a_write_under_torque() {
        let mut held = cells();
        assert_eq!(
            Regs::over(&mut held).write(0, RegId::HardwareErrorStatus, value::u8(1), true),
            Ok(value::u8(1)),
        );
    }
}
