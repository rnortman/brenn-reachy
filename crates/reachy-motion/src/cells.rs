//! The schemas that hold one field per servo, addressed by joint.
//!
//! A survey of the machine is nine readings, and every schema that holds one
//! spells the nine out as named fields rather than as an array — a row misplaced
//! in an array is a silent swap of two servos, and a row misplaced across named
//! fields does not compile. What that costs is a mapping from [`JointRef`] to the
//! field one servo's reading lives in. That pairing is the joint vocabulary's
//! own — [`rows_by_joint`] writes it once for every schema of this shape — and
//! what this module adds is the surveys themselves: the sequencers, the reports
//! and the hosts all address one by joint.
//!
//! A verdict is a reading like any other: the grid of nine failure records a
//! release fills in — one per servo it could not read — is the same shape and
//! addressed the same way.
//!
//! The provisioning grid is the same argument in two dimensions: a row per servo,
//! a cell per register the sweep provisions. Its column mapping — which register
//! occupies which cell — is here too, wildcard-free over [`RegId`], so a register
//! this build names and the grid has no cell for is refused rather than dropped.
//!
//! Values in a cell carry bit for bit, a reading that is not a number included:
//! the grid is the record an arm report is written from, and what a servo
//! answered is the evidence. A reading only has to be a number where something
//! is decided from it, and those places refuse one that is not
//! ([`crate::arm::placeable`], the supply gate).

use crate::arm::Rail;
use crate::joints::{self, JointRef, ROW_COUNT, ROWS, ServoHealth, joint_ref, rows_by_joint};
use crate::seq::RegId;
use crate::value::{self, Value, ValueShape};

pub use brenn_reachy__motion__commission_clk_rs::{
    ProvisionCell, ProvisionGrid, ProvisionRow, ServoModels,
};
pub use brenn_reachy__motion__disarm_clk_rs::SeqFailures;
pub use brenn_reachy__motion__seq_clk_rs::SeqFailureSnap;
pub use brenn_reachy__motion__servo_health_clk_rs::{
    RailRecord, RailRecordRow, RailSnap, ServoHealthRow, ServoHealths,
};

rows_by_joint!(pub ServoHealths, ServoHealthRow, health_row, health_row_mut);
rows_by_joint!(pub RailRecord, RailRecordRow, rail_row, rail_row_mut);
rows_by_joint!(pub SeqFailures, SeqFailureSnap, failure_row, failure_row_mut);
rows_by_joint!(pub ProvisionGrid, ProvisionRow, grid_row, grid_row_mut);
rows_by_joint!(pub ServoModels, u16, model, model_mut);

/// The nine model numbers those fields hold, in bus-row order.
///
/// The rows and the servos are matched through [`joint_ref`] rather than by
/// writing the field order out a second time.
#[must_use]
pub fn models_of(models: &ServoModels) -> [u16; ROW_COUNT] {
    let mut rows = [0; ROW_COUNT];
    for (row, out) in rows.iter_mut().enumerate() {
        if let Some(model) = joint_ref(row).and_then(|joint| model(models, joint)) {
            *out = *model;
        }
    }
    rows
}

/// The nine health readings those fields hold, in bus-row order.
#[must_use]
pub fn healths_of(healths: &ServoHealths) -> [ServoHealth; ROW_COUNT] {
    let mut rows = [ServoHealth { id: 0, bits: 0 }; ROW_COUNT];
    for (row, out) in rows.iter_mut().enumerate() {
        if let Some(read) = joint_ref(row).and_then(|joint| health_row(healths, joint)) {
            *out = ServoHealth {
                id: read.id,
                bits: read.bits,
            };
        }
    }
    rows
}

/// One servo's latched error byte and the id that answered it, written into the
/// field that servo names.
pub fn set_health(healths: &mut ServoHealths, joint: JointRef, health: ServoHealth) {
    if let Some(row) = health_row_mut(healths, joint) {
        row.id = health.id;
        row.bits = health.bits;
    }
}

/// The supply and error-bit readings a sweep took, as the readings the torque-on
/// gates judge.
#[must_use]
pub fn rail_of(rail: &RailSnap) -> Rail {
    Rail {
        voltages: joints::rows_of(&rail.voltages),
        health: healths_of(&rail.health),
    }
}

/// The same readings, written into a sweep's own record.
pub fn write_rail(out: &mut RailSnap, rail: &Rail) {
    joints::write_rows(&mut out.voltages, &rail.voltages);
    for (row, health) in rail.health.iter().enumerate() {
        if let Some(joint) = joint_ref(row) {
            set_health(&mut out.health, joint, *health);
        }
    }
}

/// One servo's supply reading, written into the field that servo names.
///
/// The rail's voltages are the schema's nine-number vector — the same shape the
/// angles use, because nine numbers per servo is one shape — so the write goes
/// through the vector's own indexing.
pub fn set_voltage(rail: &mut RailSnap, joint: JointRef, volts: f64) {
    joints::set_angle(&mut rail.voltages, joint, volts);
}

/// The nine supply readings, in bus-row order.
#[must_use]
pub fn voltages_of(rail: &RailSnap) -> [f64; ROW_COUNT] {
    joints::rows_of(&rail.voltages)
}

/// Pair each provisioned register with the cell of a row that holds it.
///
/// Wildcard-free over [`RegId`], so a register this build knows and this list
/// forgets is a compile error. That is not the same guarantee as pairing every
/// register the sweep walks: the sweep's list can gain a register that lands in
/// the unprovisioned arm below, which is why the pairing answers `None` there
/// rather than panicking, and why [`PROVISION_CELL_FIELDS`] is asserted against
/// the sweep's own list where that list lives.
macro_rules! provision_cells {
    ($($reg:ident => $field:ident),+ $(,)?; unprovisioned: $($bare:ident),+ $(,)?) => {
        /// The cell `reg` occupies in a row, or `None` for a register this row
        /// has no cell for.
        #[must_use]
        pub fn cell(row: &ProvisionRow, reg: RegId) -> Option<&ProvisionCell> {
            match reg {
                $(RegId::$reg => Some(&row.$field),)+
                RegId::None | $(RegId::$bare)|+ => None,
            }
        }

        /// The same cell, to write.
        pub fn cell_mut(row: &mut ProvisionRow, reg: RegId) -> Option<&mut ProvisionCell> {
            match reg {
                $(RegId::$reg => Some(&mut row.$field),)+
                RegId::None | $(RegId::$bare)|+ => None,
            }
        }

        /// How many registers a row has a cell for.
        pub const PROVISION_CELL_FIELDS: usize = [$(stringify!($reg)),+].len();
    };
}

provision_cells! {
    ReturnDelayTime => return_delay_time,
    OperatingMode => operating_mode,
    DriveMode => drive_mode,
    HomingOffset => homing_offset,
    MinPositionLimit => min_position_limit,
    MaxPositionLimit => max_position_limit,
    Shutdown => shutdown,
    MaxVoltageLimit => max_voltage_limit,
    MinVoltageLimit => min_voltage_limit,
    TemperatureLimit => temperature_limit,
    CurrentLimit => current_limit,
    VelocityLimit => velocity_limit,
    BusWatchdog => bus_watchdog,
    ProfileAcceleration => profile_acceleration,
    ProfileVelocity => profile_velocity;
    unprovisioned:
        ModelNumber,
        TorqueEnable,
        GoalPosition,
        PresentPosition,
        PresentInputVoltage,
        PresentTemperature,
        HardwareErrorStatus,
        PositionGains,
}

/// What `reg` held on `joint`, or `None` for a cell the sweep never reached and
/// for a register or a servo the grid has no cell for.
#[must_use]
pub fn provisioned(grid: &ProvisionGrid, joint: JointRef, reg: RegId) -> Option<Value> {
    let cell = cell(grid_row(grid, joint)?, reg)?;
    if cell.shape == ValueShape::None {
        return None;
    }
    Some(value::carried(cell.shape, cell.value))
}

/// Record what `reg` held on `joint`, answering `false` — and changing nothing —
/// where the grid has no cell for it.
pub fn record(grid: &mut ProvisionGrid, joint: JointRef, reg: RegId, held: Value) -> bool {
    let Some(row) = grid_row_mut(grid, joint) else {
        return false;
    };
    let Some(cell) = cell_mut(row, reg) else {
        return false;
    };
    cell.shape = held.shape();
    cell.value = held.bits();
    true
}

/// How many cells the sweep read a value into.
///
/// What an arm report counts: the cells nobody has established a correct value
/// for are the point of the grid, and how many were read says how far the sweep
/// got.
#[must_use]
pub fn recorded(grid: &ProvisionGrid) -> usize {
    let mut count = 0;
    for joint in ROWS {
        for reg in crate::arm::PROVISION_REGS {
            if provisioned(grid, joint, reg).is_some() {
                count += 1;
            }
        }
    }
    count
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::seq::{SeqError, SeqFailureKind, SeqStepKind, StepContext};
    use crate::verdict;
    use brenn_reachy__motion__commission_clk_rs::{ProvisionGridWire, ServoModelsWire};
    use brenn_reachy__motion__disarm_clk_rs::SeqFailuresWire;
    use brenn_reachy__motion__servo_health_clk_rs::RailSnapWire;

    /// The nine verdict rows read by name, in bus order.
    ///
    /// The second, independent statement of the row mapping: a test that asked
    /// [`failure_row`] which field to look in would pass under a transposition of
    /// two servos, because the write went through the same answer.
    fn named_failures(failures: &SeqFailures) -> [u8; ROW_COUNT] {
        [
            failures.body_yaw.servo_id,
            failures.leg_0.servo_id,
            failures.leg_1.servo_id,
            failures.leg_2.servo_id,
            failures.leg_3.servo_id,
            failures.leg_4.servo_id,
            failures.leg_5.servo_id,
            failures.antenna_right.servo_id,
            failures.antenna_left.servo_id,
        ]
    }

    /// A verdict about one servo is filed in that servo's own field: a release
    /// reports "unmeasured: leg 3" out of these rows, and a row misplaced sends
    /// somebody to the wrong servo.
    #[test]
    fn a_verdict_is_filed_under_the_servo_it_is_about() {
        let mut slot = SeqFailuresWire::new();
        let failures = slot.clear_valid();
        for (row, joint) in ROWS.into_iter().enumerate() {
            let id = 20 + u8::try_from(row).expect("nine rows");
            verdict::write(
                failure_row_mut(failures, joint).expect("every servo has a row"),
                &SeqError::NoAnswer {
                    context: StepContext::servo(SeqStepKind::VerifyAtStow, id),
                },
            )
            .expect("silence crosses");
        }

        assert_eq!(
            named_failures(failures),
            [20, 21, 22, 23, 24, 25, 26, 27, 28]
        );
        for (row, joint) in ROWS.into_iter().enumerate() {
            let filed = failure_row(failures, joint).expect("every servo has a row");
            assert_eq!(
                filed.servo_id,
                20 + u8::try_from(row).expect("nine rows"),
                "{joint:?}"
            );
        }

        // A joint no field answers reaches none, in either direction.
        assert!(failure_row(failures, JointRef::None).is_none());
        assert!(failure_row_mut(failures, JointRef::None).is_none());

        // A cleared grid holds no verdict at all, which is the ordinary case: a
        // release that could read every servo files nothing here.
        let mut fresh = SeqFailuresWire::new();
        let empty = fresh.clear_valid();
        for joint in ROWS {
            assert_eq!(
                failure_row(empty, joint)
                    .expect("every servo has a row")
                    .kind,
                SeqFailureKind::None
            );
        }
    }

    /// Every servo's model number reads back off the field that servo names, and
    /// off the bus row it stands on.
    #[test]
    fn a_model_number_is_read_off_the_servo_that_answered_it() {
        let mut slot = ServoModelsWire::new();
        let models = slot.clear_valid();
        for (row, joint) in ROWS.into_iter().enumerate() {
            *model_mut(models, joint).expect("every servo has a field") =
                u16::try_from(100 + row).expect("a small number");
        }
        assert_eq!(
            models_of(models),
            [100, 101, 102, 103, 104, 105, 106, 107, 108]
        );
        assert_eq!(model(models, JointRef::None), None);
    }

    /// A rail reading crosses into the schema and back as itself, health ids
    /// included.
    #[test]
    fn a_rail_reading_is_the_reading_that_went_in() {
        let mut slot = RailSnapWire::new();
        let rail = slot.clear_valid();
        let read = Rail {
            voltages: [11.0, 11.1, 11.2, 11.3, 11.4, 11.5, 11.6, 11.7, 11.8],
            health: core::array::from_fn(|row| ServoHealth {
                id: u8::try_from(10 + row).expect("a small id"),
                bits: u8::try_from(row).expect("a small byte"),
            }),
        };
        write_rail(rail, &read);
        assert_eq!(rail_of(rail), read);
        assert_eq!(voltages_of(rail), read.voltages);
    }

    /// A recorded cell reads back on the servo and the register it was recorded
    /// under, and nowhere else.
    #[test]
    fn a_recorded_cell_belongs_to_one_servo_and_one_register() {
        let mut slot = ProvisionGridWire::new();
        let grid = slot.clear_valid();
        assert!(record(
            grid,
            JointRef::Leg3,
            RegId::HomingOffset,
            value::radians(0.25)
        ));
        assert_eq!(
            provisioned(grid, JointRef::Leg3, RegId::HomingOffset),
            Some(value::radians(0.25))
        );
        assert_eq!(provisioned(grid, JointRef::Leg2, RegId::HomingOffset), None);
        assert_eq!(provisioned(grid, JointRef::Leg3, RegId::Shutdown), None);
        assert_eq!(recorded(grid), 1);

        // A register the sweep does not provision has no cell, and a joint the
        // bus has no row for has no row.
        assert!(!record(
            grid,
            JointRef::Leg3,
            RegId::GoalPosition,
            value::u8(1)
        ));
        assert!(!record(grid, JointRef::None, RegId::Shutdown, value::u8(1)));

        // A fresh sweep starts from the schema's own initial state: the grid a
        // cleared slot holds has nothing recorded in it.
        let mut fresh = ProvisionGridWire::new();
        assert_eq!(recorded(fresh.clear_valid()), 0);
    }

    /// A register and the field of a row its cell is, read by name.
    type NamedCell = (RegId, fn(&ProvisionRow) -> &ProvisionCell);

    /// Each provisioned register paired with the row field its cell is, by name.
    ///
    /// The second, independent statement of the column mapping: a test that
    /// asked [`cell`] which field to look in would pass under a transposition of
    /// two columns, because the write went through the same answer.
    fn cells_by_name() -> [NamedCell; PROVISION_CELL_FIELDS] {
        [
            (RegId::ReturnDelayTime, |row| &row.return_delay_time),
            (RegId::OperatingMode, |row| &row.operating_mode),
            (RegId::DriveMode, |row| &row.drive_mode),
            (RegId::HomingOffset, |row| &row.homing_offset),
            (RegId::MinPositionLimit, |row| &row.min_position_limit),
            (RegId::MaxPositionLimit, |row| &row.max_position_limit),
            (RegId::Shutdown, |row| &row.shutdown),
            (RegId::MaxVoltageLimit, |row| &row.max_voltage_limit),
            (RegId::MinVoltageLimit, |row| &row.min_voltage_limit),
            (RegId::TemperatureLimit, |row| &row.temperature_limit),
            (RegId::CurrentLimit, |row| &row.current_limit),
            (RegId::VelocityLimit, |row| &row.velocity_limit),
            (RegId::BusWatchdog, |row| &row.bus_watchdog),
            (RegId::ProfileAcceleration, |row| &row.profile_acceleration),
            (RegId::ProfileVelocity, |row| &row.profile_velocity),
        ]
    }

    /// A recorded reading lands in the cell its own register names.
    ///
    /// Every register gets a value of its own and every one is read off the
    /// field it belongs in: a column mapping that pairs two registers with each
    /// other's cells puts a shutdown mask where an arm report reads a homing
    /// offset, and a round trip through one mapping cannot see it.
    #[test]
    fn a_reading_lands_in_the_cell_its_own_register_names() {
        let mut slot = ProvisionGridWire::new();
        let grid = slot.clear_valid();
        for (index, (reg, _)) in cells_by_name().into_iter().enumerate() {
            let held = u8::try_from(index + 1).expect("a small count");
            assert!(
                record(grid, JointRef::BodyYaw, reg, value::u8(held)),
                "{reg} has a cell"
            );
        }
        for (index, (reg, field)) in cells_by_name().into_iter().enumerate() {
            let cell = field(&grid.body_yaw);
            let held = u8::try_from(index + 1).expect("a small count");
            assert_eq!(cell.shape, ValueShape::U8, "{reg}");
            assert_eq!(cell.value, u64::from(held), "{reg}");
        }
    }

    /// One servo's reading lands on the field that servo names, in each of the
    /// per-servo schemas: the nine fields are listed here in bus order rather
    /// than asked for through the mapping that wrote them.
    #[test]
    fn a_reading_lands_on_the_field_its_own_servo_names() {
        let mut models_slot = ServoModelsWire::new();
        let models = models_slot.clear_valid();
        let mut rail_slot = RailSnapWire::new();
        let rail = rail_slot.clear_valid();
        for (row, joint) in ROWS.into_iter().enumerate() {
            let row = u8::try_from(row).expect("a small row");
            *model_mut(models, joint).expect("every servo has a field") = 100 + u16::from(row);
            set_voltage(rail, joint, f64::from(row) + 1.0);
            set_health(
                &mut rail.health,
                joint,
                ServoHealth {
                    id: 20 + row,
                    bits: 30 + row,
                },
            );
        }
        assert_eq!(
            [
                models.body_yaw,
                models.leg_0,
                models.leg_1,
                models.leg_2,
                models.leg_3,
                models.leg_4,
                models.leg_5,
                models.antenna_right,
                models.antenna_left,
            ],
            [100, 101, 102, 103, 104, 105, 106, 107, 108]
        );
        assert_eq!(
            [
                rail.voltages.body_yaw,
                rail.voltages.leg_0,
                rail.voltages.leg_1,
                rail.voltages.leg_2,
                rail.voltages.leg_3,
                rail.voltages.leg_4,
                rail.voltages.leg_5,
                rail.voltages.antenna_right,
                rail.voltages.antenna_left,
            ],
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
        );
        assert_eq!(
            [
                (rail.health.body_yaw.id, rail.health.body_yaw.bits),
                (rail.health.leg_0.id, rail.health.leg_0.bits),
                (rail.health.leg_1.id, rail.health.leg_1.bits),
                (rail.health.leg_2.id, rail.health.leg_2.bits),
                (rail.health.leg_3.id, rail.health.leg_3.bits),
                (rail.health.leg_4.id, rail.health.leg_4.bits),
                (rail.health.leg_5.id, rail.health.leg_5.bits),
                (rail.health.antenna_right.id, rail.health.antenna_right.bits),
                (rail.health.antenna_left.id, rail.health.antenna_left.bits),
            ],
            [
                (20, 30),
                (21, 31),
                (22, 32),
                (23, 33),
                (24, 34),
                (25, 35),
                (26, 36),
                (27, 37),
                (28, 38)
            ]
        );
    }

    /// A reading that is not a number is recorded as that, in a cell and on the
    /// rail alike: the grid is the evidence an arm report is written from, and a
    /// servo that answered nonsense answered nonsense.
    #[test]
    fn a_reading_that_is_no_number_is_recorded_as_itself() {
        let mut slot = ProvisionGridWire::new();
        let grid = slot.clear_valid();
        assert!(record(
            grid,
            JointRef::Leg5,
            RegId::HomingOffset,
            value::radians(f64::NAN)
        ));
        let held = provisioned(grid, JointRef::Leg5, RegId::HomingOffset)
            .expect("a cell the sweep reached");
        assert_eq!(held.shape(), ValueShape::Radians);
        assert_eq!(
            held.bits(),
            f64::NAN.to_bits(),
            "the reading crosses bit for bit"
        );
        assert_eq!(recorded(grid), 1, "a cell holding nonsense was still read");

        let mut rail_slot = RailSnapWire::new();
        let rail = rail_slot.clear_valid();
        set_voltage(rail, JointRef::Leg5, f64::NAN);
        assert_eq!(voltages_of(rail)[6].to_bits(), f64::NAN.to_bits());
        assert_eq!(rail_of(rail).voltages[6].to_bits(), f64::NAN.to_bits());
    }

    /// The sweep's register list and a row's cells are one set said twice; a
    /// count that disagrees is a register with nowhere to be recorded.
    #[test]
    fn every_provisioned_register_has_a_cell() {
        assert_eq!(PROVISION_CELL_FIELDS, crate::arm::PROVISION_REGS.len());
        for reg in crate::arm::PROVISION_REGS {
            let mut slot = ProvisionGridWire::new();
            let grid = slot.clear_valid();
            assert!(
                record(grid, JointRef::BodyYaw, reg, value::u8(1)),
                "{reg:?} has no cell"
            );
        }
    }
}
