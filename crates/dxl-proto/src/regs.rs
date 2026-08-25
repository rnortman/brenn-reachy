//! The X-series control-table entries this project reads or writes.
//!
//! Addresses and widths are the published Robotis X-series table. The set is
//! closed rather than complete: an entry is a licence to touch that register,
//! nothing outside the list is touched at all, and a register is added here
//! only when there is a reason to reach for it. Some entries have no caller
//! yet.
//!
//! The `area` field is what lets the bus layer refuse an EEPROM write
//! mechanically. EEPROM writes are only accepted by a servo while its torque is
//! off, and on this machine the head is held up by a closed linkage, so a
//! mistaken EEPROM write is either a silent no-op or a controlled head drop.

/// Which half of the control table an address lives in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Area {
    /// Non-volatile, writable only while torque is off.
    Eeprom,
    /// Volatile, writable at any time, reset to defaults at power-on.
    Ram,
}

/// First RAM address; everything below it is EEPROM.
pub const RAM_BOUNDARY: u16 = 64;

/// One control-table entry.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Reg {
    /// Control-table address.
    pub addr: u16,
    /// Width in bytes.
    pub len: u8,
    /// Which half of the table the address falls in.
    pub area: Area,
}

impl Reg {
    const fn new(addr: u16, len: u8) -> Self {
        let area = if addr < RAM_BOUNDARY {
            Area::Eeprom
        } else {
            Area::Ram
        };
        Self { addr, len, area }
    }

    /// True when writing this register requires torque to be off.
    #[must_use]
    pub fn is_eeprom(&self) -> bool {
        matches!(self.area, Area::Eeprom)
    }
}

/// Model number. Identifies the servo part.
pub const MODEL_NUMBER: Reg = Reg::new(0, 2);
/// Servo ID.
pub const ID: Reg = Reg::new(7, 1);
/// Baud-rate index.
pub const BAUD_RATE: Reg = Reg::new(8, 1);
/// Delay before a status packet is sent, in 2 µs units.
pub const RETURN_DELAY_TIME: Reg = Reg::new(9, 1);
/// Direction and profile configuration bits.
pub const DRIVE_MODE: Reg = Reg::new(10, 1);
/// Control mode. 3 is single-turn position control (the servo enforces a
/// travel window); 4 is extended position (no window, continuous counting).
/// Anything else voids the servo-side position envelope.
pub const OPERATING_MODE: Reg = Reg::new(11, 1);
/// Offset added to the raw encoder position, in counts.
pub const HOMING_OFFSET: Reg = Reg::new(20, 4);
/// Shutdown temperature, in degrees Celsius.
pub const TEMPERATURE_LIMIT: Reg = Reg::new(31, 1);
/// Upper supply-voltage bound, in 0.1 V units.
pub const MAX_VOLTAGE_LIMIT: Reg = Reg::new(32, 2);
/// Lower supply-voltage bound, in 0.1 V units.
pub const MIN_VOLTAGE_LIMIT: Reg = Reg::new(34, 2);
/// Current ceiling, in raw register units.
pub const CURRENT_LIMIT: Reg = Reg::new(38, 2);
/// Velocity ceiling, in raw register units.
pub const VELOCITY_LIMIT: Reg = Reg::new(44, 4);
/// Upper end of the servo-side position window, in counts.
pub const MAX_POSITION_LIMIT: Reg = Reg::new(48, 4);
/// Lower end of the servo-side position window, in counts.
pub const MIN_POSITION_LIMIT: Reg = Reg::new(52, 4);
/// Which hardware-error bits latch torque off.
pub const SHUTDOWN: Reg = Reg::new(63, 1);

/// Torque on/off. Turning it off drops whatever the servo is holding.
pub const TORQUE_ENABLE: Reg = Reg::new(64, 1);
/// Indicator LED.
pub const LED: Reg = Reg::new(65, 1);
/// Which instructions get a status reply.
pub const STATUS_RETURN_LEVEL: Reg = Reg::new(68, 1);
/// Latched hardware-error bits.
pub const HARDWARE_ERROR_STATUS: Reg = Reg::new(70, 1);

// Position gain addresses run D, then I, then P — the table is not in P-I-D
// order, and reading it as if it were swaps the derivative and proportional
// terms on a load-bearing linkage.
/// Position derivative gain, at the *lowest* of the three gain addresses.
pub const POSITION_D_GAIN: Reg = Reg::new(80, 2);
/// Position integral gain.
pub const POSITION_I_GAIN: Reg = Reg::new(82, 2);
/// Position proportional gain, at the *highest* of the three gain addresses.
pub const POSITION_P_GAIN: Reg = Reg::new(84, 2);
/// All three position gains as one contiguous span, written in one transaction.
/// The byte order within it is D, I, P.
pub const POSITION_GAINS: Reg = Reg::new(80, 6);

/// Bus inactivity timeout, in 20 ms units. 0 disables it.
///
/// Armed, per session, at the value the session's configuration carries: a servo
/// whose bus has been silent that long stops holding its goal, which is what
/// answers a driver that was killed, crashed or unplugged while the machine was
/// under torque. The register lives in RAM and resets to 0 at power-on, so the
/// arming write is part of every commissioning sweep.
///
/// Written as a pair, clear then arm: a latched watchdog answers ordinary writes
/// with a Data Range error -- the same status a servo sends for an out-of-range
/// goal -- and 0 is the vendor's documented clear.
pub const BUS_WATCHDOG: Reg = Reg::new(98, 1);
/// Goal current, in raw register units. Meaningful only outside position mode.
pub const GOAL_CURRENT: Reg = Reg::new(102, 2);
/// Profile acceleration; 0 disables the profile generator.
pub const PROFILE_ACCELERATION: Reg = Reg::new(108, 4);
/// Profile velocity; 0 disables the profile generator.
pub const PROFILE_VELOCITY: Reg = Reg::new(112, 4);
/// Commanded position, in counts.
pub const GOAL_POSITION: Reg = Reg::new(116, 4);
/// Measured current, in raw register units.
pub const PRESENT_CURRENT: Reg = Reg::new(126, 2);
/// Measured position, in counts. Multi-turn while torque is off; reset into a
/// single turn when torque is enabled in single-turn position mode, when the
/// operating mode is changed to it, and at power-on or reboot. In extended
/// position mode the reading stays continuous across those transitions.
pub const PRESENT_POSITION: Reg = Reg::new(132, 4);
/// Measured supply voltage, in 0.1 V units.
pub const PRESENT_INPUT_VOLTAGE: Reg = Reg::new(144, 2);
/// Measured temperature at the servo, whole degrees Celsius.
pub const PRESENT_TEMPERATURE: Reg = Reg::new(146, 1);

#[cfg(test)]
mod tests {
    use super::*;

    const ALL: &[(&str, Reg)] = &[
        ("model_number", MODEL_NUMBER),
        ("id", ID),
        ("baud_rate", BAUD_RATE),
        ("return_delay_time", RETURN_DELAY_TIME),
        ("drive_mode", DRIVE_MODE),
        ("operating_mode", OPERATING_MODE),
        ("homing_offset", HOMING_OFFSET),
        ("temperature_limit", TEMPERATURE_LIMIT),
        ("max_voltage_limit", MAX_VOLTAGE_LIMIT),
        ("min_voltage_limit", MIN_VOLTAGE_LIMIT),
        ("current_limit", CURRENT_LIMIT),
        ("velocity_limit", VELOCITY_LIMIT),
        ("max_position_limit", MAX_POSITION_LIMIT),
        ("min_position_limit", MIN_POSITION_LIMIT),
        ("shutdown", SHUTDOWN),
        ("torque_enable", TORQUE_ENABLE),
        ("led", LED),
        ("status_return_level", STATUS_RETURN_LEVEL),
        ("hardware_error_status", HARDWARE_ERROR_STATUS),
        ("position_d_gain", POSITION_D_GAIN),
        ("position_i_gain", POSITION_I_GAIN),
        ("position_p_gain", POSITION_P_GAIN),
        ("position_gains", POSITION_GAINS),
        ("bus_watchdog", BUS_WATCHDOG),
        ("goal_current", GOAL_CURRENT),
        ("profile_acceleration", PROFILE_ACCELERATION),
        ("profile_velocity", PROFILE_VELOCITY),
        ("goal_position", GOAL_POSITION),
        ("present_current", PRESENT_CURRENT),
        ("present_position", PRESENT_POSITION),
        ("present_input_voltage", PRESENT_INPUT_VOLTAGE),
        ("present_temperature", PRESENT_TEMPERATURE),
    ];

    #[test]
    fn area_follows_the_address_boundary() {
        for (name, reg) in ALL {
            let expected = if reg.addr < RAM_BOUNDARY {
                Area::Eeprom
            } else {
                Area::Ram
            };
            assert_eq!(reg.area, expected, "{name}");
            assert_eq!(reg.is_eeprom(), expected == Area::Eeprom, "{name}");
        }
    }

    #[test]
    fn gain_addresses_are_in_d_i_p_order() {
        assert_eq!(POSITION_D_GAIN.addr, 80);
        assert_eq!(POSITION_I_GAIN.addr, 82);
        assert_eq!(POSITION_P_GAIN.addr, 84);
        const { assert!(POSITION_D_GAIN.addr < POSITION_P_GAIN.addr) };
    }

    #[test]
    fn the_gain_span_covers_all_three_gains() {
        assert_eq!(POSITION_GAINS.addr, POSITION_D_GAIN.addr);
        assert_eq!(
            u16::from(POSITION_GAINS.len),
            POSITION_P_GAIN.addr + u16::from(POSITION_P_GAIN.len) - POSITION_D_GAIN.addr
        );
    }

    #[test]
    fn every_register_has_a_plausible_width() {
        for (name, reg) in ALL {
            assert!(matches!(reg.len, 1 | 2 | 4 | 6), "{name}");
        }
    }

    /// Two registers may share bytes only when one is the gain span and the
    /// other is a gain inside it; any other overlap is a transcription slip.
    #[test]
    fn addresses_do_not_overlap_apart_from_the_gain_span() {
        let inside_the_gain_span = |name: &str| {
            matches!(
                name,
                "position_d_gain" | "position_i_gain" | "position_p_gain"
            )
        };

        for (name_a, a) in ALL {
            for (name_b, b) in ALL {
                if name_a == name_b {
                    continue;
                }
                if (*name_a == "position_gains" && inside_the_gain_span(name_b))
                    || (*name_b == "position_gains" && inside_the_gain_span(name_a))
                {
                    continue;
                }
                let (end_a, end_b) = (a.addr + u16::from(a.len), b.addr + u16::from(b.len));
                assert!(
                    end_a <= b.addr || end_b <= a.addr,
                    "{name_a} ({}..{end_a}) overlaps {name_b} ({}..{end_b})",
                    a.addr,
                    b.addr
                );
            }
        }
    }
}
