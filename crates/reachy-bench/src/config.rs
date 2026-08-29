//! The bench's configuration file, and the checks that decide whether it may be
//! used.
//!
//! One TOML file maps onto what the bench's two halves need: the bus device and
//! its timing, the roster, what the read-only registry verifies and the two
//! thresholds it judges against. Every default here carries the provenance of
//! its number, because a bench configuration is the place where a measured
//! figure eventually replaces a guess and the two have to be told apart.
//!
//! Nothing here configures motion. Coordinated motion is the cog system's, and
//! it is configured by its own schema-typed parameter files; a key here with no
//! consumer in this crate is a key an operator would read as a bound that
//! something enforces, so there are none.
//!
//! The datum is not configured here. That a converted count is the model's
//! crank angle rests on every leg servo carrying its provisioned homing offset,
//! and the record of it is the self-test's `datum` case: a hardware comparison
//! against [`VENDOR_HOMING_OFFSETS`], written into the self-test state with its
//! provenance.

use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::Context as _;
use serde::Deserialize;
use thiserror::Error;

#[cfg(test)]
use reachy_bus::ServoMap;
use reachy_bus::{BusTiming, DEFAULT_BAUD, MapError};
use reachy_motion::joints::{ROW_COUNT, ROWS, leg_ref};
use reachy_motion::value;
#[cfg(test)]
use reachy_motion::{
    ANTENNA_GOAL_MAX_RAD, ANTENNA_GOAL_MIN_RAD, YAW_GOAL_COUNT_MAX, yaw_goal_counts,
};
use reachy_motion::{
    EXPECTED_OPERATING_MODES, ProvisionExpect, ProvisionTable, RegId, VENDOR_HOMING_OFFSETS,
};

/// The highest servo ID a unicast request may name: the address one below the
/// broadcast address.
const MAX_SERVO_ID: u8 = dxl_proto::BROADCAST_ID - 1;

/// Why a configuration cannot be used.
///
/// Every arm is a refusal, never a substitution: a configuration that says
/// something impossible is not quietly replaced with something possible.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum ConfigError {
    /// A quantity that must be a positive real number is not one.
    #[error("{key} must be a positive, finite number; it is {value}")]
    NotPositive {
        /// The configuration key, in `table.key` form.
        key: &'static str,
        /// What the file said.
        value: f64,
    },

    /// A count or rate that must be at least one is zero.
    #[error("{key} must be at least 1; it is 0")]
    NotPositiveInt {
        /// The configuration key, in `table.key` form.
        key: &'static str,
    },

    /// A servo ID that is not a unicast address.
    #[error("servo id {id} at bus position {row} is not an addressable servo")]
    ServoIdReserved {
        /// Position in bus order.
        row: usize,
        /// The configured ID.
        id: u8,
    },

    /// One servo ID configured for two joints.
    #[error("servo id {id} appears twice in the bus roster")]
    ServoIdDuplicate {
        /// The repeated ID.
        id: u8,
    },

    /// The two readings the sweep accepts for one register at rest are the
    /// same value, so nothing could tell the two rest states apart.
    #[error("{alternate} must differ from {baseline}; both are {value}")]
    RestingStatesCollide {
        /// The key holding the second accepted reading.
        alternate: &'static str,
        /// The key holding the provisioned baseline.
        baseline: &'static str,
        /// The value both keys hold.
        value: u8,
    },

    /// A second accepted reading is the value the register holds once it has
    /// tripped, which is a reading the sweep exists to refuse.
    #[error("{key} must not be {value}: that is what the register reads once it has tripped")]
    RestingStateIsATrip {
        /// The configuration key, in `table.key` form.
        key: &'static str,
        /// What the file said.
        value: u8,
    },

    /// A count could not cross the joint/wire boundary.
    #[error(transparent)]
    Map(#[from] MapError),
}

/// The configuration file as written.
///
/// Every table may be omitted, in which case its defaults apply.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct BenchConfig {
    /// The port and its timing.
    #[serde(default)]
    pub bus: BusSection,
    /// The thresholds the registry judges against.
    #[serde(default)]
    pub arm: ArmSection,
    /// What the provisioned registers must hold.
    #[serde(default)]
    pub provision: ProvisionSection,
}

/// `[bus]` — the port and the transaction timing.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(default, deny_unknown_fields)]
pub struct BusSection {
    /// The serial device the servos are on.
    pub device: String,
    /// Wire rate, bits per second.
    pub baud: u32,
    /// Fixed slack over the wire time, milliseconds. At least one: a zero
    /// allowance collapses every transaction deadline to the bare wire time,
    /// leaving the servo's own turnaround no room and timing out every read.
    pub host_allowance_ms: u64,
    /// Attempts a retried operation makes in total, the first included. Zero is
    /// read as one — the transaction layer always makes the first attempt — so
    /// it disables retrying rather than disabling the operation.
    pub retry_attempts: u32,
    /// Pause between attempts, milliseconds. Zero is meaningful: the attempt
    /// that failed already spent its whole deadline waiting, so retrying with no
    /// further pause is a choice rather than a hot loop.
    pub retry_spacing_ms: u64,
    /// The nine servo IDs, in bus order: body yaw, legs 1..=6, right antenna,
    /// left antenna.
    pub servo_ids: [u8; ROW_COUNT],
}

impl Default for BusSection {
    fn default() -> Self {
        Self {
            // The node the platform's own documentation names. The first
            // read-only run confirms it; nothing here discovers ports.
            device: "/dev/ttyAMA3".to_string(),
            baud: DEFAULT_BAUD,
            host_allowance_ms: 10,
            retry_attempts: 5,
            retry_spacing_ms: 20,
            servo_ids: reachy_motion::arm::SERVO_IDS,
        }
    }
}

/// `[arm]` — the two thresholds the read-only registry judges a reading
/// against.
///
/// Named for the arm gate they were sized for: the rail floor a machine has to
/// clear before its servos could hold anything up, and how far a limp servo's
/// goal register may sit from its measured position. Nothing in this crate arms
/// anything, and neither figure gates anything — the registry reports a case
/// against them and writes the verdict into its record.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(default, deny_unknown_fields)]
pub struct ArmSection {
    /// The supply floor a rail reading is judged against, volts.
    pub min_arm_voltage: f64,
    /// How far a limp servo's goal register may sit from its measured position,
    /// degrees.
    pub goal_shadow_tolerance_deg: f64,
}

impl Default for ArmSection {
    /// Whole degrees rather than the library's radians converted back, so a
    /// value written in the file and a value written here are the same decimal
    /// and a round trip through the file changes nothing.
    fn default() -> Self {
        Self {
            min_arm_voltage: reachy_motion::arm::DEFAULT_MIN_ARM_VOLTAGE,
            goal_shadow_tolerance_deg: 2.0,
        }
    }
}

/// `[provision]` — what the servos must already hold for this project to command
/// them.
///
/// These are the register's own contents as a data sheet states them, not
/// engineering units: integers a person compares against how the unit was set
/// up. Two register families are deliberately absent — the velocity limits and
/// the shutdown masks, whose correct values nobody has established. The sweep
/// reads and reports those rather than judging them, and a reading in a record
/// is what establishes them.
///
/// The profile registers are absent for a different reason: whoever arms this
/// machine writes them, verified, at every arm, and they live in RAM that holds
/// until power comes off. What a table here could state is the power-on value,
/// which stops being true the moment something has armed once — so they are
/// read and reported rather than checked. The gains are absent for the same
/// reason.
///
/// The operating mode is absent for a third: it differs per servo and is not an
/// operator's to choose, so it is baked in
/// [`reachy_motion::EXPECTED_OPERATING_MODES`] and checked from there.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(default, deny_unknown_fields)]
pub struct ProvisionSection {
    /// Return Delay Time, all nine servos.
    pub return_delay_time: u8,
    /// Drive Mode, all nine.
    pub drive_mode: u8,
    /// Bus Watchdog, all nine. The provisioned baseline of 0, the disabled
    /// register: what a unit power-cycled since its last session reads.
    pub bus_watchdog: u8,
    /// Bus Watchdog, all nine, as a motion session leaves it: the timeout the
    /// session's commissioning sweep arms. It must track
    /// `cogs/session_params.textproto`'s `bus_watchdog`, because this is the
    /// second reading the sweep accepts and a figure that drifted from the
    /// session's would accept a register nothing wrote.
    ///
    /// The register is RAM-resident and resets to 0 at power-on, so a sweep run
    /// after a session reads this and a sweep run after a power cycle reads the
    /// baseline. Both are the machine at rest and both pass; anything else,
    /// including a latched trip, fails.
    ///
    /// The default is asserted equal to `bare::WATCHDOG_COUNTS`, the figure the
    /// bench's own `watchdog` command arms, so the two in-crate copies cannot
    /// drift. The cog system's `bus_watchdog` is in another process's
    /// configuration and nothing here can read it: that coupling is checked by
    /// a person changing one and grepping for the other, and by the sweep going
    /// red on a machine whose session armed something else.
    ///
    /// A figure equal to `bus_watchdog`, or the tripped `0xFF`, refuses the
    /// registry: see [`BenchConfig::resting_bus_watchdog`].
    pub bus_watchdog_armed: u8,
    /// Temperature Limit, all nine.
    pub temperature_limit: u8,
    /// Maximum Voltage Limit, all nine, in tenths of a volt.
    pub max_voltage_limit: u16,
    /// Minimum Voltage Limit, all nine, in tenths of a volt.
    pub min_voltage_limit: u16,
    /// Current Limit, per servo in bus order. Per servo rather than one value
    /// because the roster is not all one part; on this unit all nine read the
    /// same untouched factory ceiling.
    pub current_limit: [u16; ROW_COUNT],
    /// The per-leg position window the servo itself refuses to be commanded
    /// past, counts, lower then upper, legs 1..=6.
    ///
    /// What this deployment means to provision, not what the fence agreement
    /// is judged from: the `leg-fence` registry case reads the windows back
    /// off the servos and compares those against the motion envelope.
    pub leg_position_limits: [[u32; 2]; 6],
}

impl Default for ProvisionSection {
    /// The factory table this platform is documented as being provisioned with.
    fn default() -> Self {
        Self {
            return_delay_time: 0,
            drive_mode: 0,
            bus_watchdog: 0,
            bus_watchdog_armed: 10,
            temperature_limit: 70,
            max_voltage_limit: 70,
            min_voltage_limit: 35,
            current_limit: [1750; ROW_COUNT],
            leg_position_limits: [
                [1502, 2958],
                [1138, 2844],
                [1502, 2958],
                [1138, 2594],
                [1252, 2958],
                [1138, 2594],
            ],
        }
    }
}

/// Read and parse a configuration file.
///
/// Parsing only: a file that parses can still be refused by
/// [`BenchConfig::servo_ids`].
pub fn load(path: &Path) -> anyhow::Result<BenchConfig> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("reading the bench configuration at {}", path.display()))?;
    parse(&text).with_context(|| format!("parsing the bench configuration at {}", path.display()))
}

/// Parse a configuration from TOML text.
pub fn parse(text: &str) -> anyhow::Result<BenchConfig> {
    toml::from_str(text).map_err(Into::into)
}

/// The self-test record's file name, written beside the configuration it
/// describes a run of.
pub const RECORD_NAME: &str = "selftest-state.toml";

/// Where the record for `config` lives unless a caller names another path.
///
/// Beside the configuration, because the pair belongs to one machine: a record
/// says a particular unit's servos answered as the file next to it says they
/// should.
pub fn record_path_beside(config: &Path) -> PathBuf {
    config.parent().unwrap_or(Path::new("")).join(RECORD_NAME)
}

impl BenchConfig {
    /// The bus timing.
    pub fn bus_timing(&self) -> Result<BusTiming, ConfigError> {
        let section = &self.bus;
        Ok(BusTiming {
            host_allowance: positive_millis("bus.host_allowance_ms", section.host_allowance_ms)?,
            baud: positive_int("bus.baud", section.baud)?,
            retry_attempts: section.retry_attempts,
            retry_spacing: Duration::from_millis(section.retry_spacing_ms),
        })
    }

    /// The Bus Watchdog reading a motion session leaves behind, checked for
    /// being a reading the sweep can act on.
    ///
    /// Two configurations say something impossible and are refused rather than
    /// worked around: a value equal to the provisioned baseline, which would
    /// make every servo's reading ambiguous and turn the sweep's record of
    /// whether a session ran into a coin toss; and the tripped value, which the
    /// sweep is there to fail on and cannot also accept.
    pub fn resting_bus_watchdog(&self) -> Result<u8, ConfigError> {
        let armed = self.provision.bus_watchdog_armed;
        if armed == self.provision.bus_watchdog {
            return Err(ConfigError::RestingStatesCollide {
                alternate: "provision.bus_watchdog_armed",
                baseline: "provision.bus_watchdog",
                value: armed,
            });
        }
        if armed == crate::bare::WATCHDOG_LATCHED {
            return Err(ConfigError::RestingStateIsATrip {
                key: "provision.bus_watchdog_armed",
                value: armed,
            });
        }
        Ok(armed)
    }

    /// The nine servo IDs, checked for being addressable and distinct.
    pub fn servo_ids(&self) -> Result<[u8; ROW_COUNT], ConfigError> {
        let ids = self.bus.servo_ids;
        for (row, id) in ids.iter().enumerate() {
            if *id > MAX_SERVO_ID {
                return Err(ConfigError::ServoIdReserved { row, id: *id });
            }
            if ids[..row].contains(id) {
                return Err(ConfigError::ServoIdDuplicate { id: *id });
            }
        }
        Ok(ids)
    }

    /// What arming verifies, per servo and register.
    ///
    /// Position limits, the homing offset and the current limit differ per
    /// servo; everything else is one value on all nine.
    #[must_use]
    pub fn provision_table(&self) -> ProvisionTable {
        let section = &self.provision;
        let mut table = ProvisionTable::new();
        table.set_all(
            RegId::ReturnDelayTime,
            ProvisionExpect::Check(value::u8(section.return_delay_time)),
        );
        table.set_all(
            RegId::DriveMode,
            ProvisionExpect::Check(value::u8(section.drive_mode)),
        );
        table.set_all(
            RegId::BusWatchdog,
            ProvisionExpect::Check(value::u8(section.bus_watchdog)),
        );
        table.set_all(
            RegId::TemperatureLimit,
            ProvisionExpect::Check(value::u8(section.temperature_limit)),
        );
        table.set_all(
            RegId::MaxVoltageLimit,
            ProvisionExpect::Check(value::u16(section.max_voltage_limit)),
        );
        table.set_all(
            RegId::MinVoltageLimit,
            ProvisionExpect::Check(value::u16(section.min_voltage_limit)),
        );
        // Read and reported, never judged: the gains-and-profiles phase writes
        // these RAM registers at every arm and verifies its own write, so what
        // they hold is arming's property rather than the platform's setup. A
        // nonzero reading here is useful evidence — it says the machine has not
        // been power-cycled since the last arm — and it is not a disagreement.
        table.set_all(RegId::ProfileAcceleration, ProvisionExpect::Record);
        table.set_all(RegId::ProfileVelocity, ProvisionExpect::Record);
        // Recorded rather than checked: nobody has established what these hold,
        // and a reading in the arm report is what establishes them. The three
        // single-turn joints' position limits are here for the same reason —
        // their provisioned range is the whole turn and no window is claimed.
        table.set_all(RegId::VelocityLimit, ProvisionExpect::Record);
        table.set_all(RegId::Shutdown, ProvisionExpect::Record);
        table.set_all(RegId::MinPositionLimit, ProvisionExpect::Record);
        table.set_all(RegId::MaxPositionLimit, ProvisionExpect::Record);
        // Checked against the workspace's one record of the datum and of the
        // per-servo modes, neither of them a per-unit setting.
        for (row, joint) in ROWS.iter().enumerate() {
            table.set(
                *joint,
                RegId::CurrentLimit,
                ProvisionExpect::Check(value::u16(section.current_limit[row])),
            );
            table.set(
                *joint,
                RegId::HomingOffset,
                ProvisionExpect::Check(value::i32(VENDOR_HOMING_OFFSETS[row])),
            );
            table.set(
                *joint,
                RegId::OperatingMode,
                ProvisionExpect::Check(value::u8(EXPECTED_OPERATING_MODES[row])),
            );
        }
        for leg in 0..6u8 {
            let index = usize::from(leg);
            let [lower, upper] = section.leg_position_limits[index];
            table.set(
                leg_ref(leg),
                RegId::MinPositionLimit,
                ProvisionExpect::Check(value::u32(lower)),
            );
            table.set(
                leg_ref(leg),
                RegId::MaxPositionLimit,
                ProvisionExpect::Check(value::u32(upper)),
            );
        }
        table
    }
}

/// A quantity that has to be a positive real number.
pub(crate) fn positive(key: &'static str, value: f64) -> Result<f64, ConfigError> {
    if value.is_finite() && value > 0.0 {
        Ok(value)
    } else {
        Err(ConfigError::NotPositive { key, value })
    }
}

/// A count or rate that has to be at least one.
fn positive_int(key: &'static str, value: u32) -> Result<u32, ConfigError> {
    if value == 0 {
        Err(ConfigError::NotPositiveInt { key })
    } else {
        Ok(value)
    }
}

/// A period in milliseconds that has to be at least one.
fn positive_millis(key: &'static str, millis: u64) -> Result<Duration, ConfigError> {
    if millis == 0 {
        Err(ConfigError::NotPositiveInt { key })
    } else {
        Ok(Duration::from_millis(millis))
    }
}

#[cfg(test)]
mod tests {

    use super::*;

    /// The example shipped beside the crate, which an operator copies.
    const EXAMPLE: &str = include_str!("../reachy-bench.example.toml");

    /// The bounds the motion layer refuses a goal or a stored pin on are count
    /// bounds, and this is the one place the two layers that hold them meet:
    /// the motion layer states them over its own copy of the count frame, and
    /// the map is what turns a radian into the count the goal register takes.
    ///
    /// Yaw is checked count for count across the turn rather than at its ends
    /// alone, because its bound is a rounding as much as a range: the last half
    /// count below +π rounds to a count the single-turn register refuses, and
    /// the motion layer only refuses it if it rounds the way the map does. Each
    /// antenna end is exactly the last count extended position mode accepts —
    /// asymmetric, because zero radians sits at count 2048.
    #[test]
    fn the_goal_bounds_are_the_last_counts_the_registers_hold() {
        const YAW_ROW: usize = 0;
        const ANTENNA_ROWS: [usize; 2] = [7, 8];
        const REGISTER_LIMIT: i32 = 1_048_575;
        let map = ServoMap::new(reachy_motion::arm::SERVO_IDS);

        let count = core::f64::consts::TAU / (YAW_GOAL_COUNT_MAX + 1.0);
        let probes = [
            -core::f64::consts::PI,
            -core::f64::consts::PI + count / 4.0,
            -1.0,
            0.0,
            1.0,
            core::f64::consts::PI - count,
            core::f64::consts::PI - count / 4.0,
        ];
        for radians in probes {
            assert_eq!(
                yaw_goal_counts(radians),
                f64::from(
                    map.goal_counts(YAW_ROW, radians)
                        .expect("a yaw goal counts")
                ),
                "{radians} rad"
            );
        }
        assert_eq!(yaw_goal_counts(-core::f64::consts::PI), 0.0);
        assert_eq!(
            yaw_goal_counts(core::f64::consts::PI - count),
            YAW_GOAL_COUNT_MAX
        );
        assert_eq!(
            yaw_goal_counts(core::f64::consts::PI - count / 4.0),
            YAW_GOAL_COUNT_MAX + 1.0,
            "the half-count sliver the register refuses"
        );

        // Both ends on both rows: a sign or an offset wrong on one antenna only
        // is invisible to a row exercised in the one direction it is right in.
        let ends = [
            (ANTENNA_GOAL_MAX_RAD, REGISTER_LIMIT, 1.0),
            (ANTENNA_GOAL_MIN_RAD, -REGISTER_LIMIT, -1.0),
        ];
        for row in ANTENNA_ROWS {
            for (edge, limit, past) in ends {
                assert_eq!(
                    map.goal_counts(row, edge).expect("the bound places"),
                    limit,
                    "row {row} at {edge} rad"
                );
                let over = map
                    .goal_counts(row, edge + past)
                    .expect("a count outside the register's range is still an i32");
                assert!(
                    over.abs() > REGISTER_LIMIT,
                    "row {row}: a radian past the bound is {over} counts"
                );
            }
        }
    }

    /// The smallest file there is: empty, every table defaulted.
    const MINIMAL: &str = "";

    fn minimal() -> BenchConfig {
        parse(MINIMAL).expect("the minimal configuration parses")
    }

    #[test]
    fn the_example_file_parses_as_shipped() {
        let cfg = parse(EXAMPLE).expect("the shipped example parses");
        assert_eq!(
            cfg.servo_ids().expect("the roster is nine servos"),
            reachy_motion::arm::SERVO_IDS
        );
    }

    /// A per-unit file left over from when the schema had a `[datum]` table
    /// fails parse naming the table, which is the migration prompt: the copy is
    /// gitignored and per-machine, so the parse error is the only place an
    /// operator learns the table is gone.
    #[test]
    fn a_file_still_carrying_a_datum_table_is_refused_by_name() {
        let stale = parse(&format!(
            "{MINIMAL}\n[datum]\ncrank_datum = \"direct\"\nprovenance = \"an old copy\"\n"
        ));
        let message = format!("{:#}", stale.expect_err("the table is no longer a key"));
        assert!(message.contains("datum"), "{message}");
    }

    #[test]
    fn the_example_file_says_what_the_defaults_say() {
        // The example spells out every key. If it drifts from the defaults, one
        // of the two is a stale transcription and an operator reading the file
        // would be misled about what omitting a key does.
        let example = parse(EXAMPLE).expect("the shipped example parses");
        let defaults = minimal();
        assert_eq!(example.bus, defaults.bus);
        assert_eq!(example.provision, defaults.provision);
        assert_eq!(example.arm, defaults.arm);
    }

    /// The figures this file defaults to are the libraries' own, not a second
    /// copy free to drift from them.
    #[test]
    fn the_defaults_are_the_libraries_own() {
        let cfg = minimal();
        assert_eq!(
            cfg.arm.min_arm_voltage,
            reachy_motion::arm::DEFAULT_MIN_ARM_VOLTAGE
        );
        let timing = cfg.bus_timing().expect("the default timing resolves");
        assert_eq!(timing.baud, DEFAULT_BAUD);
        assert_eq!(timing, BusTiming::default());
    }

    /// An unknown key is refused by name rather than ignored.
    #[test]
    fn an_unknown_key_is_refused() {
        let typo = parse(&format!("{MINIMAL}\n[bus]\nbaud_rate = 50\n"));
        let message = format!("{:#}", typo.expect_err("an unknown key is refused"));
        assert!(message.contains("baud_rate"), "{message}");
    }

    /// The datum has one record, the self-test's, and this file is not it.
    ///
    /// A homing offset is what makes a converted count the model's crank angle;
    /// a file that could set it would be a second record of the same truth,
    /// free to disagree. There is no such key, so a file carrying one is
    /// refused rather than quietly ignored.
    #[test]
    fn the_homing_offsets_are_not_a_configuration_key() {
        let typo = parse(&format!(
            "{MINIMAL}\n[provision]\nleg_homing_offset = [1024, -1024, 1024, -1024, 1024, -1024]\n"
        ));
        let message = format!("{:#}", typo.expect_err("there is no such key"));
        assert!(message.contains("leg_homing_offset"), "{message}");
    }

    #[test]
    fn a_roster_that_is_not_nine_distinct_addressable_servos_refuses() {
        let reserved = parse(&format!(
            "{MINIMAL}\n[bus]\nservo_ids = [10, 11, 12, 13, 14, 15, 16, 17, 254]\n"
        ))
        .expect("parses");
        assert_eq!(
            reserved.servo_ids().unwrap_err(),
            ConfigError::ServoIdReserved { row: 8, id: 254 }
        );

        let duplicate = parse(&format!(
            "{MINIMAL}\n[bus]\nservo_ids = [10, 11, 12, 13, 14, 15, 16, 17, 11]\n"
        ))
        .expect("parses");
        assert_eq!(
            duplicate.servo_ids().unwrap_err(),
            ConfigError::ServoIdDuplicate { id: 11 }
        );
    }

    #[test]
    fn the_provisioning_table_checks_the_setup_and_records_the_rest() {
        let cfg = minimal();
        let table = cfg.provision_table();
        // Eight registers on all nine servos — the homing offset among them —
        // the current limit on all nine, and two more per-leg registers on six
        // legs.
        assert_eq!(table.checks(), 8 * 9 + 9 + 2 * 6);
        // Everything checked, plus six recorded families on all nine, less the
        // two per-leg position limits that are checked rather than recorded.
        assert_eq!(table.reads(), table.checks() + 6 * 9 - 2 * 6);

        // The mode is per servo and not a key: single-turn position on the body
        // and the legs, extended position on the two free-turning antennas.
        let column = ProvisionTable::column(RegId::OperatingMode).expect("provisioned");
        for (row, mode) in EXPECTED_OPERATING_MODES.into_iter().enumerate() {
            assert_eq!(
                table.at(row, column),
                Some(ProvisionExpect::Check(value::u8(mode))),
                "row {row}"
            );
        }
        assert_eq!(EXPECTED_OPERATING_MODES, [3, 3, 3, 3, 3, 3, 3, 4, 4]);

        // The offset register is signed, so a negative quarter turn is checked
        // and reported as one rather than as a span near four billion.
        let column = ProvisionTable::column(RegId::HomingOffset).expect("provisioned");
        for (row, offset) in VENDOR_HOMING_OFFSETS.into_iter().enumerate() {
            assert_eq!(
                table.at(row, column),
                Some(ProvisionExpect::Check(value::i32(offset))),
                "row {row}"
            );
        }
        assert_eq!(reachy_motion::Shown(value::i32(-1024)).to_string(), "-1024");

        // Per servo, and on this unit every servo holds the same ceiling.
        let column = ProvisionTable::column(RegId::CurrentLimit).expect("provisioned");
        for row in 0..ROW_COUNT {
            assert_eq!(
                table.at(row, column),
                Some(ProvisionExpect::Check(value::u16(1750))),
                "row {row}"
            );
        }
    }

    /// The armed watchdog figure is a key with a default rather than something
    /// the sweep infers: it has to track what the session arms, and a
    /// deployment whose session is configured otherwise says so in its own
    /// file.
    ///
    /// The default is pinned against `bare::WATCHDOG_COUNTS` and not against a
    /// fresh literal: the bench's `watchdog` command arms that constant, so the
    /// one in-tree copy of this figure the crate can reach is the one this
    /// default must not drift from. The cog system's copy is in another
    /// process's configuration and is out of a unit test's reach.
    #[test]
    fn the_armed_watchdog_value_defaults_to_what_the_session_arms() {
        assert_eq!(
            minimal().provision.bus_watchdog_armed,
            crate::bare::WATCHDOG_COUNTS,
            "the sweep's second rest state is the value the bench itself arms"
        );
        assert_eq!(
            parse(EXAMPLE)
                .expect("the shipped example parses")
                .provision
                .bus_watchdog_armed,
            crate::bare::WATCHDOG_COUNTS,
            "and the example ships the same figure the session's parameters hold"
        );

        let other =
            parse(&format!("{MINIMAL}\n[provision]\nbus_watchdog_armed = 4\n")).expect("parses");
        assert_eq!(other.provision.bus_watchdog_armed, 4);
        // The baseline is untouched by it: the two readings are separate keys,
        // and the table still checks the provisioned zero.
        assert_eq!(other.provision.bus_watchdog, 0);
        let column = ProvisionTable::column(RegId::BusWatchdog).expect("provisioned");
        assert_eq!(
            other.provision_table().at(0, column),
            Some(ProvisionExpect::Check(value::u8(0)))
        );
    }

    /// The two profile registers are read and reported, never compared.
    ///
    /// Arming writes them at every arm and they persist in RAM until power comes
    /// off, so a checked value here holds only until this process has armed
    /// once — after which every arm and every self-test in the same power cycle
    /// would fail the sweep on a machine that is behaving exactly as designed.
    /// The lifecycle re-arms in every process, so that is not an edge case.
    #[test]
    fn the_registers_arming_rewrites_are_recorded_and_not_checked() {
        let table = minimal().provision_table();
        for reg in [RegId::ProfileAcceleration, RegId::ProfileVelocity] {
            let column = ProvisionTable::column(reg).expect("read by the sweep");
            for row in 0..ROW_COUNT {
                assert_eq!(
                    table.at(row, column),
                    Some(ProvisionExpect::Record),
                    "{reg} row {row}"
                );
            }
        }
    }

    /// Retired keys are refused by name rather than ignored.
    ///
    /// Every section is `deny_unknown_fields`, so a configuration carrying a
    /// key that no longer exists says so loudly instead of silently accepting
    /// it.
    #[test]
    fn a_configuration_carrying_a_retired_key_is_refused_by_name() {
        // Motion tables this crate has no consumer for. A file carrying
        // one would leave an operator believing a bound is configured that
        // nothing reads.
        for section in ["motion", "envelope", "disarm", "settle"] {
            let stale = parse(&format!("{MINIMAL}\n[{section}]\n"));
            let message = format!("{:#}", stale.expect_err("there is no such table"));
            assert!(message.contains(section), "{message}");
        }

        // The `[arm]` keys this crate does not consume: the profile and
        // gains a commanding host writes for itself, and the rail-wait
        // budget nothing here waits out.
        for (key, value) in [
            ("profile_acceleration", "400"),
            ("profile_velocity", "600"),
            ("leg_gains", "{ p = 800, i = 100, d = 300 }"),
            ("yaw_gains", "{ p = 200, i = 0, d = 0 }"),
            ("antenna_gains", "{ p = 500, i = 0, d = 100 }"),
            ("voltage_poll_period_ms", "100"),
            ("voltage_budget_s", "30.0"),
            ("repin_tolerance_deg", "0.5"),
            ("max_pin_pull_in_deg", "12.0"),
            ("recheck_tolerance_deg", "0.5"),
        ] {
            let stale = parse(&format!("{MINIMAL}\n[arm]\n{key} = {value}\n"));
            let message = format!("{:#}", stale.expect_err("there is no such key"));
            assert!(message.contains(key), "{message}");
        }

        for key in ["profile_acceleration", "profile_velocity"] {
            let stale = parse(&format!("{MINIMAL}\n[provision]\n{key} = 0\n"));
            let message = format!("{:#}", stale.expect_err("there is no such key"));
            assert!(message.contains(key), "{message}");
        }

        // `operating_mode` is a per-servo record in the library, not a
        // configuration key. A file setting it is refused rather than silently
        // accepted with wrong antenna expectations.
        let stale = parse(&format!("{MINIMAL}\n[provision]\noperating_mode = 3\n"));
        let message = format!("{:#}", stale.expect_err("there is no such key"));
        assert!(message.contains("operating_mode"), "{message}");
    }

    #[test]
    fn the_provisioned_set_contains_no_command_path_register() {
        // Arming reads these before it writes anything, and a goal or a torque
        // flag among them would be a command-path register verified as setup.
        for reg in [
            RegId::GoalPosition,
            RegId::PresentPosition,
            RegId::TorqueEnable,
            RegId::PositionGains,
        ] {
            assert_eq!(ProvisionTable::column(reg), None, "{reg}");
        }
    }
}
