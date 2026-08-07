//! The bench's configuration file, and the checks that decide whether it may be
//! used.
//!
//! One TOML file maps onto the library configuration structs: the bus device and
//! its timing, the envelope's caps, the tick's parameters, what arming verifies
//! and writes, what disarming compares against, and the resolved crank datum.
//! Every default here carries the provenance of its number, because a bench
//! configuration is the place where a measured figure eventually replaces a
//! guess and the two have to be told apart.
//!
//! ## The datum gate
//!
//! There is no default crank datum and there never will be. That a converted
//! count is the model's crank angle rests on every leg servo carrying its
//! provisioned homing offset, which the self-test reads back and the arm
//! sequence re-verifies; the `[datum]` table is the record that a person checked
//! that evidence and stands behind it. A configuration without the table, or
//! with one carrying no provenance line, resolves to a typed refusal and no
//! command that moves anything runs.
//!
//! That gate and the self-test record's gate are one function here,
//! [`arm_gates`], rather than a step in a binary: the bench is not the only
//! host that commands this machine, and a second host has to run the gate
//! itself and not a copy of it.
//!
//! ## Two fences, one region
//!
//! The servos enforce a per-leg position window of their own, in counts, and the
//! envelope enforces a per-leg crank window, in radians. They are separate
//! mechanisms guarding what must be one region. Mapping the count window through
//! the servo map and comparing it against the envelope window is the only place
//! that correspondence is ever established, so it is established here, before
//! any sequencer runs: each count bound must land inside its envelope bound and
//! within one count of it. A leg whose two windows describe different regions is
//! refused by name rather than left to diverge silently — which of the two
//! records is wrong is not something code should pick.

use std::fmt;
use std::path::{Path, PathBuf};
use std::time::Duration;

use anyhow::Context as _;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::selftest::SelftestRecord;
use reachy_bus::{BusTiming, DEFAULT_BAUD, MapError, ServoMap};
use reachy_kin::{EnvelopeConfig, FkOptions, HeadGeometry, IkError, baked};
use reachy_motion::{
    ArmConfig, DisarmConfig, EXPECTED_OPERATING_MODES, Gains, GroupGains, JointId, JointStep,
    MotionConfig, ProfileConfig, ProvisionExpect, ProvisionTable, RegId, RegValue,
    TrackingFaultConfig, VENDOR_HOMING_OFFSETS, stow_targets,
};

/// One encoder count, radians — the finest distinction a position register can
/// make, and the slack the two per-leg fences are allowed to differ by.
///
/// Derived from the encoder resolution the conversions themselves are built on,
/// so the fence tolerance and the counts-to-radians conversion can never
/// disagree about how wide a count is.
const ONE_COUNT_RAD: f64 = core::f64::consts::TAU / dxl_proto::conv::COUNTS_PER_REV as f64;

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

    /// A per-tick step bound wider than the tracking threshold, which would let
    /// a goal move past an open tracking run's anchor in one period and fault a
    /// joint that is following.
    #[error(
        "{key} is {step} rad, wider than motion.tracking_threshold_rad ({threshold} rad): a goal \
         stepping that far in one period can be read as a joint running away from it"
    )]
    StepPastTrackingThreshold {
        /// The step bound's configuration key, in `table.key` form.
        key: &'static str,
        /// What the file said the step bound is, radians.
        step: f64,
        /// The threshold it must not exceed, radians.
        threshold: f64,
    },

    /// A positive number of seconds that is longer than a duration can hold.
    #[error("{key} is {secs} seconds, which is longer than a duration can hold")]
    DurationOutOfRange {
        /// The configuration key, in `table.key` form.
        key: &'static str,
        /// What the file said.
        secs: f64,
    },

    /// An envelope crank window whose lower bound is not below its upper bound.
    #[error("the envelope crank window for leg {} is empty: {lower_deg}° to {upper_deg}°", leg + 1)]
    EmptyEnvelopeWindow {
        /// 0-based leg index.
        leg: usize,
        /// The configured lower bound, degrees.
        lower_deg: f64,
        /// The configured upper bound, degrees.
        upper_deg: f64,
    },

    /// A provisioned count window whose lower bound is not below its upper
    /// bound.
    #[error("the provisioned count window for leg {} is empty: {lower} to {upper}", leg + 1)]
    EmptyCountWindow {
        /// 0-based leg index.
        leg: usize,
        /// The configured lower bound, counts.
        lower: u32,
        /// The configured upper bound, counts.
        upper: u32,
    },

    /// A provisioned count no position register can hold.
    #[error("the provisioned count {count} for leg {} is past what a position register holds", leg + 1)]
    CountOutOfRange {
        /// 0-based leg index.
        leg: usize,
        /// The configured count.
        count: u32,
    },

    /// The servo-side and host-side per-leg fences describe different regions.
    #[error("{0}")]
    FenceMismatch(FenceMismatch),

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

    /// No crank datum has been resolved.
    #[error(
        "no crank datum is configured: add a [datum] table once a human has read a self-test record and stands behind what its datum case says"
    )]
    MissingDatum,

    /// A datum recorded with nothing saying where it came from.
    #[error(
        "the [datum] table has no provenance: record who resolved the datum, when, and from which self-test record"
    )]
    DatumProvenanceEmpty,

    /// A provisioning table that verifies nothing.
    #[error(
        "the provisioning table checks nothing: arming would enable torque on a machine whose setup was never verified"
    )]
    NothingChecked,

    /// The configured geometry cannot reach the stow pose, so disarming has
    /// nothing to compare against.
    #[error("the configured geometry cannot reach the stow pose: {0}")]
    StowUnreachable(#[from] IkError),

    /// A count could not cross the joint/wire boundary.
    #[error(transparent)]
    Map(#[from] MapError),
}

/// A leg whose provisioned count window and envelope crank window do not
/// describe the same region.
///
/// Carries both windows in both domains, because the fault is in exactly one of
/// the two records and the message has to give a person enough to say which.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FenceMismatch {
    /// 0-based leg index.
    pub leg: usize,
    /// The provisioned window, counts.
    pub servo_counts: [u32; 2],
    /// The same window mapped to model angles, degrees.
    pub servo_deg: [f64; 2],
    /// The envelope window, degrees.
    pub envelope_deg: [f64; 2],
    /// The envelope window mapped back to counts, where a count places it.
    pub envelope_counts: [Option<i32>; 2],
}

impl fmt::Display for FenceMismatch {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "leg {}: the provisioned window {}..{} counts reads {:.3}°..{:.3}°, \
             which is not inside the envelope window {:.3}°..{:.3}° ({}..{} counts) to within one count",
            self.leg + 1,
            self.servo_counts[0],
            self.servo_counts[1],
            self.servo_deg[0],
            self.servo_deg[1],
            self.envelope_deg[0],
            self.envelope_deg[1],
            render_count(self.envelope_counts[0]),
            render_count(self.envelope_counts[1]),
        )
    }
}

/// A count bound, or a placeholder where no count places the angle.
fn render_count(count: Option<i32>) -> String {
    count.map_or_else(|| "unplaceable".to_string(), |c| c.to_string())
}

/// The configuration file as written.
///
/// Every table but `[arm]` may be omitted, in which case its defaults apply.
/// `[arm]` carries the servo-side motion profile, which has no defensible
/// default — it is the backstop under host-side shaping, and a profile invented
/// here would be a limit nobody sized.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct BenchConfig {
    /// The port and its timing.
    #[serde(default)]
    pub bus: BusSection,
    /// The bounds every commanded pose is checked against.
    #[serde(default)]
    pub envelope: EnvelopeSection,
    /// The tick's rates and bounds.
    #[serde(default)]
    pub motion: MotionSection,
    /// What arming verifies and writes.
    pub arm: ArmSection,
    /// What disarming compares against.
    #[serde(default)]
    pub disarm: DisarmSection,
    /// What the provisioned registers must hold.
    #[serde(default)]
    pub provision: ProvisionSection,
    /// The resolved crank datum. Absent until a human has read a self-test
    /// record and written it in.
    #[serde(default)]
    pub datum: Option<DatumSection>,
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
    pub servo_ids: [u8; JointId::COUNT],
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

/// `[envelope]` — the caps every commanded pose is checked against.
///
/// Angles are degrees and the clearance floor is millimetres, because that is
/// how a person reasons about them; the library takes radians and metres.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(default, deny_unknown_fields)]
pub struct EnvelopeSection {
    /// Per-leg crank travel windows, degrees, lower then upper, legs 1..=6.
    pub crank_windows_deg: [[f64; 2]; 6],
    /// Body yaw cap, degrees, symmetric about zero.
    pub body_yaw_limit_deg: f64,
    /// Head-relative yaw cap, degrees, symmetric about zero.
    pub relative_yaw_limit_deg: f64,
    /// Head attitude cap against base vertical, degrees.
    pub head_cone_limit_deg: f64,
    /// Floor on the per-pose toggle margin, millimetres.
    pub min_toggle_margin_mm: f64,
}

impl Default for EnvelopeSection {
    /// Whole numbers rather than the library's radians converted back, so a
    /// value written here and a value written there are the same decimal and a
    /// round trip through the file changes nothing.
    /// `the_defaults_are_the_libraries_own` is what keeps the two in step.
    fn default() -> Self {
        Self {
            crank_windows_deg: core::array::from_fn(|leg| {
                let (lower, upper) = baked::CRANK_WINDOWS_DEG[leg];
                [lower, upper]
            }),
            // Well inside the ±160° mechanical figure; the cable routing at
            // large yaw has not been checked.
            body_yaw_limit_deg: 60.0,
            relative_yaw_limit_deg: 55.0,
            head_cone_limit_deg: 35.0,
            min_toggle_margin_mm: 3.0,
        }
    }
}

/// `[motion]` — the tick's rates and bounds, and the move durations the bench
/// commands with.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(default, deny_unknown_fields)]
pub struct MotionSection {
    /// Control periods per second.
    pub tick_hz: u32,
    /// Hardware-health sweeps per second.
    pub health_poll_hz: u32,
    /// Bound on any one crank's change per tick, radians.
    pub max_step_legs_rad: f64,
    /// Bound on the body yaw's change per tick, radians.
    pub max_step_body_yaw_rad: f64,
    /// Bound on either antenna's change per tick, radians.
    pub max_step_antennas_rad: f64,
    /// How far a joint may lag its goal before the lag is examined at all,
    /// radians.
    pub tracking_threshold_rad: f64,
    /// How far a lagging joint must close on its goal, over a window of
    /// `tracking_ticks`, to count as following it, radians.
    pub tracking_progress_min_rad: f64,
    /// How many consecutive lagging, non-closing ticks are a tracking fault.
    pub tracking_ticks: u32,
    /// Consecutive ticks without a position read before the read-loss fault.
    /// Absent means one second at the tick rate.
    pub read_loss_ticks: Option<u32>,
    /// How long the stow move takes, seconds.
    pub stow_duration_s: f64,
    /// How long the move from stow to the neutral pose takes, seconds.
    pub up_duration_s: f64,
    /// How long a joint-space move — a yaw or an antenna command — takes,
    /// seconds.
    pub move_duration_s: f64,
    /// How long a `hold` command watches the machine before returning, seconds.
    pub hold_duration_s: f64,
}

impl Default for MotionSection {
    fn default() -> Self {
        let defaults = MotionConfig::default();
        Self {
            tick_hz: 50,
            // The servos update their own hardware-error byte far slower than
            // they update position; once a second is enough to catch a latch
            // and cheap enough to sit inside the tick's budget.
            health_poll_hz: 1,
            max_step_legs_rad: defaults.max_step.legs,
            max_step_body_yaw_rad: defaults.max_step.body_yaw,
            max_step_antennas_rad: defaults.max_step.antennas,
            tracking_threshold_rad: defaults.tracking.threshold_rad,
            tracking_progress_min_rad: defaults.tracking.progress_min_rad,
            tracking_ticks: defaults.tracking.ticks,
            read_loss_ticks: None,
            stow_duration_s: 2.0,
            // Longer than the stow move: the lift is the one motion that starts
            // from a configuration close to a singular one, and there is no
            // reason for it to be quick.
            up_duration_s: 3.0,
            // A yaw or an antenna command moves joints that carry no load and
            // pass through no near-singular configuration, so the length is a
            // matter of what is comfortable to watch rather than of mechanics.
            move_duration_s: 3.0,
            // Long enough that a tracking fault or a health latch has periods
            // to appear in, short enough that a supervised operator is not left
            // waiting on a command that does nothing by design.
            hold_duration_s: 2.0,
        }
    }
}

/// `[arm]` — the thresholds arming refuses on and the values it writes.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(deny_unknown_fields)]
pub struct ArmSection {
    /// The supply floor arming refuses to proceed below, volts.
    #[serde(default = "default_min_arm_voltage")]
    pub min_arm_voltage: f64,
    /// How often the supply is re-read while waiting for the rail,
    /// milliseconds. At least one: a zero period turns the paced rail wait into
    /// an unpaced re-read of the voltage register for the whole budget.
    #[serde(default = "default_voltage_poll_period_ms")]
    pub voltage_poll_period_ms: u64,
    /// How long to wait for the rail before failing, seconds.
    #[serde(default = "default_voltage_budget_s")]
    pub voltage_budget_s: f64,
    /// The largest distance a pin may pull a joint, degrees.
    #[serde(default = "default_max_pin_pull_in_deg")]
    pub max_pin_pull_in_deg: f64,
    /// How far a joint may be from where the arrival check expects it, degrees.
    #[serde(default = "default_recheck_tolerance_deg")]
    pub recheck_tolerance_deg: f64,
    /// How far a limp servo's goal register may sit from its measured position,
    /// degrees.
    #[serde(default = "default_goal_shadow_tolerance_deg")]
    pub goal_shadow_tolerance_deg: f64,
    /// Position gains for the six crank servos.
    #[serde(default = "default_leg_gains")]
    pub leg_gains: GainsSection,
    /// Position gains for the body yaw servo.
    #[serde(default = "default_yaw_gains")]
    pub yaw_gains: GainsSection,
    /// Position gains for the two antenna servos.
    #[serde(default = "default_antenna_gains")]
    pub antenna_gains: GainsSection,
    /// The servo-side profile's acceleration limit, register units. No default:
    /// this is the backstop under host-side shaping, and it has to be sized
    /// against the moves this bench actually makes.
    pub profile_acceleration: u32,
    /// The servo-side profile's velocity limit, register units. No default, for
    /// the same reason.
    pub profile_velocity: u32,
}

/// One servo group's position-loop gains.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct GainsSection {
    /// Proportional gain.
    pub p: u16,
    /// Integral gain.
    pub i: u16,
    /// Derivative gain.
    pub d: u16,
}

impl From<GainsSection> for Gains {
    fn from(section: GainsSection) -> Self {
        Self {
            p: section.p,
            i: section.i,
            d: section.d,
        }
    }
}

fn default_min_arm_voltage() -> f64 {
    reachy_motion::arm::DEFAULT_MIN_ARM_VOLTAGE
}

fn default_voltage_poll_period_ms() -> u64 {
    reachy_motion::arm::DEFAULT_VOLTAGE_POLL_PERIOD.as_millis() as u64
}

fn default_voltage_budget_s() -> f64 {
    reachy_motion::arm::DEFAULT_VOLTAGE_BUDGET.as_secs_f64()
}

/// Whole degrees rather than the library's radians converted back; see
/// [`EnvelopeSection::default`] for why.
fn default_max_pin_pull_in_deg() -> f64 {
    12.0
}

fn default_recheck_tolerance_deg() -> f64 {
    0.5
}

fn default_goal_shadow_tolerance_deg() -> f64 {
    2.0
}

fn default_leg_gains() -> GainsSection {
    gains_section(reachy_motion::arm::DEFAULT_GAINS.legs)
}

fn default_yaw_gains() -> GainsSection {
    gains_section(reachy_motion::arm::DEFAULT_GAINS.yaw)
}

fn default_antenna_gains() -> GainsSection {
    gains_section(reachy_motion::arm::DEFAULT_GAINS.antennas)
}

fn gains_section(gains: Gains) -> GainsSection {
    GainsSection {
        p: gains.p,
        i: gains.i,
        d: gains.d,
    }
}

/// `[disarm]` — what the verified torque-off tail compares against.
#[derive(Clone, Debug, Deserialize, PartialEq)]
#[serde(default, deny_unknown_fields)]
pub struct DisarmSection {
    /// How far a joint may be from its stow angle and still count as at stow,
    /// degrees.
    pub tolerance_deg: f64,
    /// How long the platform is left to settle before it is measured against
    /// stow, seconds.
    pub dwell_s: f64,
}

impl Default for DisarmSection {
    fn default() -> Self {
        Self {
            tolerance_deg: 2.0,
            dwell_s: reachy_motion::disarm::DEFAULT_STOW_DWELL.as_secs_f64(),
        }
    }
}

/// `[provision]` — what the servos must already hold for this project to command
/// them.
///
/// These are the register's own contents as a data sheet states them, not
/// engineering units: integers a person compares against how the unit was set
/// up. Two register families are deliberately absent — the velocity limits and
/// the shutdown masks, whose correct values nobody has established. Arming reads
/// and reports those rather than judging them, and a reading in an arm report is
/// what establishes them.
///
/// The profile registers are absent for a different reason: arming writes them
/// itself, verified, at every arm, and they live in RAM that holds until power
/// comes off. What a table here could state is the power-on value, which stops
/// being true the moment this process has armed once — so they are read and
/// reported rather than checked, and arming's own read-back is the sole
/// authority on what they hold. The gains are absent for the same reason.
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
    /// Bus Watchdog, all nine. Disabled: on this linkage a servo that stops
    /// holding its goal drops the head.
    pub bus_watchdog: u8,
    /// Temperature Limit, all nine.
    pub temperature_limit: u8,
    /// Maximum Voltage Limit, all nine, in tenths of a volt.
    pub max_voltage_limit: u16,
    /// Minimum Voltage Limit, all nine, in tenths of a volt.
    pub min_voltage_limit: u16,
    /// Current Limit, per servo in bus order. Per servo rather than one value
    /// because the roster is not all one part; on this unit all nine read the
    /// same untouched factory ceiling.
    pub current_limit: [u16; JointId::COUNT],
    /// The per-leg position window the servo itself refuses to be commanded
    /// past, counts, lower then upper, legs 1..=6. Mapped through the servo map
    /// under the configured datum, this must be the envelope's crank window.
    pub leg_position_limits: [[u32; 2]; 6],
}

impl Default for ProvisionSection {
    /// The factory table this platform is documented as being provisioned with.
    fn default() -> Self {
        Self {
            return_delay_time: 0,
            drive_mode: 0,
            bus_watchdog: 0,
            temperature_limit: 70,
            max_voltage_limit: 70,
            min_voltage_limit: 35,
            current_limit: [1750; JointId::COUNT],
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

/// `[datum]` — how the legs' counts relate to the model's crank angles.
///
/// Both keys are required. A datum without a provenance line is a claim nobody
/// can trace back to the reading that produced it, and the whole point of this
/// table is that a person read the evidence.
#[derive(Clone, Debug, Deserialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct DatumSection {
    /// Which reading of the counts is the model's crank angle.
    pub crank_datum: DatumSetting,
    /// Who resolved it, when, and from which self-test record.
    pub provenance: String,
}

/// The datum as a file spells it — the bench configuration and the self-test
/// record alike, which is why one spelling serves both.
///
/// One variant, deliberately. A host-side correction is never the answer to a
/// servo that lacks its provisioned offset: that is one servo's fault, refused
/// by name, and a compensating shift would move all six legs for it. The enum
/// exists so that a file saying anything else is refused by serde rather than
/// read past.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DatumSetting {
    /// A converted count is the crank angle as the model means it, because each
    /// leg servo applies its provisioned homing offset before reporting.
    Direct,
}

impl fmt::Display for DatumSetting {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Direct => f.write_str("direct"),
        }
    }
}

/// A configuration that has passed every check, as the library structs.
///
/// Holding one of these is the evidence that the datum is resolved, the two
/// per-leg fences agree, and the geometry can reach stow.
#[derive(Clone, Debug)]
pub struct Resolved {
    /// The serial device to open.
    pub device: String,
    /// Deadline and retry policy.
    pub timing: BusTiming,
    /// Joints, IDs, registers and counts.
    pub map: ServoMap,
    /// Where the configured datum came from.
    pub datum_provenance: String,
    /// The tick's configuration, geometry and envelope included.
    pub motion: MotionConfig,
    /// What arming verifies and writes.
    pub arm: ArmConfig,
    /// What disarming compares against, with the drop flag clear — that flag is
    /// the operator's, given per invocation and never stored.
    pub disarm: DisarmConfig,
    /// Control periods per second.
    pub tick_hz: u32,
    /// Hardware-health sweeps per second.
    pub health_poll_hz: u32,
    /// How long the stow move takes.
    pub stow_duration: Duration,
    /// How long the move from stow to neutral takes.
    pub up_duration: Duration,
    /// How long a yaw or antenna move takes.
    pub move_duration: Duration,
    /// How long a `hold` command watches the machine.
    pub hold_duration: Duration,
}

/// Read and parse a configuration file.
///
/// Parsing only: what the file says still has to survive [`BenchConfig::resolve`].
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

/// The two standing gates, and the configuration that survived them.
///
/// Every host that commands this machine runs exactly this, before it opens the
/// port: the configuration must resolve — which is where the recorded crank
/// datum is required — and the self-test record at `record` must be one in
/// which every case passed. It lives here rather than in a binary so a second
/// host runs the gate itself and not a copy of it.
///
/// Separate from the run, so the refusals are testable without a port: they are
/// the whole reason a motion command exists behind a read-only one.
pub fn arm_gates(cfg: &BenchConfig, record: &Path) -> anyhow::Result<Resolved> {
    let resolved = cfg.resolve()?;
    let record = SelftestRecord::load(record).with_context(|| {
        format!(
            "reading the self-test record at {}; run `reachy-bench selftest` first",
            record.display()
        )
    })?;
    record
        .admits_arm()
        .context("the self-test record does not admit arming")?;
    Ok(resolved)
}

impl BenchConfig {
    /// The envelope, in the library's units.
    pub fn envelope(&self) -> Result<EnvelopeConfig, ConfigError> {
        let section = &self.envelope;
        let mut crank_windows = [(0.0, 0.0); 6];
        for (leg, window) in section.crank_windows_deg.iter().enumerate() {
            let [lower_deg, upper_deg] = *window;
            if !(lower_deg.is_finite() && upper_deg.is_finite() && lower_deg < upper_deg) {
                return Err(ConfigError::EmptyEnvelopeWindow {
                    leg,
                    lower_deg,
                    upper_deg,
                });
            }
            crank_windows[leg] = (lower_deg.to_radians(), upper_deg.to_radians());
        }
        Ok(EnvelopeConfig {
            crank_windows,
            body_yaw_limit: positive("envelope.body_yaw_limit_deg", section.body_yaw_limit_deg)?
                .to_radians(),
            relative_yaw_limit: positive(
                "envelope.relative_yaw_limit_deg",
                section.relative_yaw_limit_deg,
            )?
            .to_radians(),
            head_cone_limit: positive("envelope.head_cone_limit_deg", section.head_cone_limit_deg)?
                .to_radians(),
            min_toggle_margin: positive(
                "envelope.min_toggle_margin_mm",
                section.min_toggle_margin_mm,
            )? / 1000.0,
        })
    }

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

    /// The nine servo IDs, checked for being addressable and distinct.
    pub fn servo_ids(&self) -> Result<[u8; JointId::COUNT], ConfigError> {
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

    /// Where the recorded datum came from.
    ///
    /// This is the gate that keeps every motion command off a machine nobody
    /// has reviewed the provisioning evidence for.
    pub fn datum(&self) -> Result<&str, ConfigError> {
        let section = self.datum.as_ref().ok_or(ConfigError::MissingDatum)?;
        if section.provenance.trim().is_empty() {
            return Err(ConfigError::DatumProvenanceEmpty);
        }
        Ok(section.provenance.as_str())
    }

    /// The tick's configuration.
    pub fn motion(
        &self,
        geom: HeadGeometry,
        env: EnvelopeConfig,
    ) -> Result<MotionConfig, ConfigError> {
        let section = &self.motion;
        let tick_hz = positive_int("motion.tick_hz", section.tick_hz)?;
        let max_step = JointStep {
            legs: positive("motion.max_step_legs_rad", section.max_step_legs_rad)?,
            body_yaw: positive(
                "motion.max_step_body_yaw_rad",
                section.max_step_body_yaw_rad,
            )?,
            antennas: positive(
                "motion.max_step_antennas_rad",
                section.max_step_antennas_rad,
            )?,
        };
        let tracking = TrackingFaultConfig {
            threshold_rad: positive(
                "motion.tracking_threshold_rad",
                section.tracking_threshold_rad,
            )?,
            progress_min_rad: positive(
                "motion.tracking_progress_min_rad",
                section.tracking_progress_min_rad,
            )?,
            ticks: positive_int("motion.tracking_ticks", section.tracking_ticks)?,
        };
        // The tracking monitor measures a joint's progress from where it stood
        // when its run opened, signed toward the goal, so a goal that moves
        // past that anchor in one period reverses the sign on a joint that is
        // following healthily. What keeps a goal from getting there is the step
        // guard: a per-tick step no wider than the threshold cannot cross the
        // band of tolerated error around the joint, and a run inside that band
        // has already cleared. The two bounds are therefore checked against
        // each other rather than only for positivity.
        for (key, step) in [
            ("motion.max_step_legs_rad", max_step.legs),
            ("motion.max_step_body_yaw_rad", max_step.body_yaw),
            ("motion.max_step_antennas_rad", max_step.antennas),
        ] {
            if step > tracking.threshold_rad {
                return Err(ConfigError::StepPastTrackingThreshold {
                    key,
                    step,
                    threshold: tracking.threshold_rad,
                });
            }
        }
        Ok(MotionConfig {
            geom,
            env,
            fk: FkOptions::default(),
            max_step,
            tracking,
            // One second of silence at the configured rate, unless the file says
            // otherwise.
            read_loss_ticks: match section.read_loss_ticks {
                Some(ticks) => positive_int("motion.read_loss_ticks", ticks)?,
                None => tick_hz,
            },
        })
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
            ProvisionExpect::Check(RegValue::U8(section.return_delay_time)),
        );
        table.set_all(
            RegId::DriveMode,
            ProvisionExpect::Check(RegValue::U8(section.drive_mode)),
        );
        table.set_all(
            RegId::BusWatchdog,
            ProvisionExpect::Check(RegValue::U8(section.bus_watchdog)),
        );
        table.set_all(
            RegId::TemperatureLimit,
            ProvisionExpect::Check(RegValue::U8(section.temperature_limit)),
        );
        table.set_all(
            RegId::MaxVoltageLimit,
            ProvisionExpect::Check(RegValue::U16(section.max_voltage_limit)),
        );
        table.set_all(
            RegId::MinVoltageLimit,
            ProvisionExpect::Check(RegValue::U16(section.min_voltage_limit)),
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
        for (row, joint) in JointId::ALL.iter().enumerate() {
            table.set(
                *joint,
                RegId::CurrentLimit,
                ProvisionExpect::Check(RegValue::U16(section.current_limit[row])),
            );
            table.set(
                *joint,
                RegId::HomingOffset,
                ProvisionExpect::Check(RegValue::I32(VENDOR_HOMING_OFFSETS[row])),
            );
            table.set(
                *joint,
                RegId::OperatingMode,
                ProvisionExpect::Check(RegValue::U8(EXPECTED_OPERATING_MODES[row])),
            );
        }
        for leg in 0..6u8 {
            let index = usize::from(leg);
            let [lower, upper] = section.leg_position_limits[index];
            table.set(
                JointId::Leg(leg),
                RegId::MinPositionLimit,
                ProvisionExpect::Check(RegValue::U32(lower)),
            );
            table.set(
                JointId::Leg(leg),
                RegId::MaxPositionLimit,
                ProvisionExpect::Check(RegValue::U32(upper)),
            );
        }
        table
    }

    /// Everything the libraries need, with every check run.
    ///
    /// The datum gate runs first: without a reviewed datum record nothing that
    /// moves the machine may run, so there is no point converting the rest.
    pub fn resolve(&self) -> Result<Resolved, ConfigError> {
        let provenance = self.datum()?;
        let ids = self.servo_ids()?;
        let map = ServoMap::new(ids);
        let env = self.envelope()?;
        let geom = HeadGeometry::default();
        let motion = self.motion(geom.clone(), env)?;
        let leg_windows = leg_windows_from_counts(&env, &map, &self.provision.leg_position_limits)?;

        let expected = self.provision_table();
        check_table(&expected)?;

        let arm_section = &self.arm;
        let arm = ArmConfig {
            ids,
            expected,
            min_arm_voltage: positive("arm.min_arm_voltage", arm_section.min_arm_voltage)?,
            voltage_poll_period: positive_millis(
                "arm.voltage_poll_period_ms",
                arm_section.voltage_poll_period_ms,
            )?,
            voltage_budget: duration_from_secs(
                "arm.voltage_budget_s",
                arm_section.voltage_budget_s,
            )?,
            gains: GroupGains {
                legs: arm_section.leg_gains.into(),
                yaw: arm_section.yaw_gains.into(),
                antennas: arm_section.antenna_gains.into(),
            },
            profile: ProfileConfig {
                acceleration: arm_section.profile_acceleration,
                velocity: arm_section.profile_velocity,
            },
            max_pin_pull_in: positive("arm.max_pin_pull_in_deg", arm_section.max_pin_pull_in_deg)?
                .to_radians(),
            recheck_tolerance: positive(
                "arm.recheck_tolerance_deg",
                arm_section.recheck_tolerance_deg,
            )?
            .to_radians(),
            goal_shadow_tolerance: positive(
                "arm.goal_shadow_tolerance_deg",
                arm_section.goal_shadow_tolerance_deg,
            )?
            .to_radians(),
            leg_windows,
        };

        let disarm = DisarmConfig {
            ids,
            stow_targets: stow_targets(&geom)?,
            tolerance: positive("disarm.tolerance_deg", self.disarm.tolerance_deg)?.to_radians(),
            dwell: duration_from_secs_non_negative("disarm.dwell_s", self.disarm.dwell_s)?,
            force_drop: false,
        };

        Ok(Resolved {
            device: self.bus.device.clone(),
            timing: self.bus_timing()?,
            map,
            datum_provenance: provenance.to_string(),
            motion,
            arm,
            disarm,
            tick_hz: self.motion.tick_hz,
            health_poll_hz: positive_int("motion.health_poll_hz", self.motion.health_poll_hz)?,
            stow_duration: duration_from_secs(
                "motion.stow_duration_s",
                self.motion.stow_duration_s,
            )?,
            up_duration: duration_from_secs("motion.up_duration_s", self.motion.up_duration_s)?,
            move_duration: duration_from_secs(
                "motion.move_duration_s",
                self.motion.move_duration_s,
            )?,
            hold_duration: duration_from_secs(
                "motion.hold_duration_s",
                self.motion.hold_duration_s,
            )?,
        })
    }
}

/// The six legs' servo-side windows in model radians, refusing any that is not
/// the envelope's own window.
///
/// This is the whole of the fence correspondence. A count bound must land inside
/// its envelope bound and no further inside than one count: further out and the
/// servo would accept a crank angle the envelope refuses, further in and the
/// servo would refuse angles the envelope permits, and either way the two
/// mechanisms are guarding different regions with nothing on the command path
/// noticing. The windows in radians are returned because they are what a pin
/// pins into, so the fence that was checked is the fence that is used.
pub fn leg_windows_from_counts(
    env: &EnvelopeConfig,
    map: &ServoMap,
    counts: &[[u32; 2]; 6],
) -> Result<[(f64, f64); 6], ConfigError> {
    let mut windows = [(0.0, 0.0); 6];
    // Walked in bus order and filtered to the legs, so which row a leg's servo
    // sits at comes from the joint layout rather than from arithmetic here.
    for (row, id) in JointId::ALL.into_iter().enumerate() {
        let JointId::Leg(index) = id else { continue };
        let leg = usize::from(index);
        let [lower_counts, upper_counts] = counts[leg];
        if lower_counts >= upper_counts {
            return Err(ConfigError::EmptyCountWindow {
                leg,
                lower: lower_counts,
                upper: upper_counts,
            });
        }
        let lower = map.present_rad(row, count_to_i32(leg, lower_counts)?)?;
        let upper = map.present_rad(row, count_to_i32(leg, upper_counts)?)?;
        let (env_lower, env_upper) = env.crank_windows[leg];
        let low_slack = lower - env_lower;
        let high_slack = env_upper - upper;
        let agrees = (0.0..=ONE_COUNT_RAD).contains(&low_slack)
            && (0.0..=ONE_COUNT_RAD).contains(&high_slack);
        if !agrees {
            return Err(ConfigError::FenceMismatch(FenceMismatch {
                leg,
                servo_counts: [lower_counts, upper_counts],
                servo_deg: [lower.to_degrees(), upper.to_degrees()],
                envelope_deg: [env_lower.to_degrees(), env_upper.to_degrees()],
                envelope_counts: [
                    map.goal_counts(row, env_lower).ok(),
                    map.goal_counts(row, env_upper).ok(),
                ],
            }));
        }
        windows[leg] = (lower, upper);
    }
    Ok(windows)
}

/// A configured count as the signed count the conversion takes.
fn count_to_i32(leg: usize, count: u32) -> Result<i32, ConfigError> {
    i32::try_from(count).map_err(|_| ConfigError::CountOutOfRange { leg, count })
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

/// A number of seconds as a duration, refusing anything a duration cannot hold.
///
/// `Duration::from_secs_f64` panics on a finite value past its range, so the
/// conversion is the fallible one: a mistyped exponent in the file comes back as
/// a refusal naming the key, like every other bad value.
fn duration_from_secs_checked(key: &'static str, secs: f64) -> Result<Duration, ConfigError> {
    Duration::try_from_secs_f64(secs).map_err(|_| ConfigError::DurationOutOfRange { key, secs })
}

/// A duration that has to be a positive number of seconds.
fn duration_from_secs(key: &'static str, secs: f64) -> Result<Duration, ConfigError> {
    duration_from_secs_checked(key, positive(key, secs)?)
}

/// A duration that may be zero but not negative or unplaceable.
fn duration_from_secs_non_negative(key: &'static str, secs: f64) -> Result<Duration, ConfigError> {
    if secs.is_finite() && secs >= 0.0 {
        duration_from_secs_checked(key, secs)
    } else {
        Err(ConfigError::NotPositive { key, value: secs })
    }
}

/// The provisioning table arming verifies, refusing one that verifies nothing.
///
/// A table with no checked cell would let arming enable torque on a machine
/// whose setup was never established. No file can express one today — the table
/// is built from sections that always contribute — so this stands between a
/// future one and a torque enable rather than guarding a shape the parser
/// currently admits.
pub(crate) fn check_table(table: &ProvisionTable) -> Result<(), ConfigError> {
    if table.checks() == 0 {
        return Err(ConfigError::NothingChecked);
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The example shipped beside the crate, which an operator copies.
    const EXAMPLE: &str = include_str!("../reachy-bench.example.toml");

    /// The `[datum]` table the example deliberately does not ship, appended by
    /// the tests that need a configuration which resolves. A real one is written
    /// by a human from a self-test record; this one stands for that step.
    const RESOLVED_DATUM: &str =
        "\n[datum]\ncrank_datum = \"direct\"\nprovenance = \"test fixture\"\n";

    /// The shipped example with a datum written in, which is the file an
    /// operator ends up with after reviewing a run.
    fn example_resolved() -> BenchConfig {
        parse(&format!("{EXAMPLE}{RESOLVED_DATUM}")).expect("the example plus a datum parses")
    }

    /// A key and the edit that makes it invalid.
    type Mutation = (&'static str, fn(&mut BenchConfig));

    /// The smallest file that resolves: the two profile figures, and a datum.
    const MINIMAL: &str = "\
[arm]
profile_acceleration = 50
profile_velocity = 300

[datum]
crank_datum = \"direct\"
provenance = \"test fixture\"
";

    fn minimal() -> BenchConfig {
        parse(MINIMAL).expect("the minimal configuration parses")
    }

    #[test]
    fn the_example_file_parses_and_resolves_once_a_datum_is_written_in() {
        let cfg = example_resolved();
        let resolved = cfg.resolve().expect("the example plus a datum resolves");
        assert_eq!(resolved.map.ids(), reachy_motion::arm::SERVO_IDS);
        assert!(!resolved.datum_provenance.trim().is_empty());
    }

    /// The shipped example carries no datum, and therefore refuses every command
    /// that moves something exactly as it stands.
    ///
    /// A filled-in example would be a datum nobody reviewed sitting on the gate
    /// that exists to force a human to review one: the copied file would
    /// resolve, and nothing downstream re-asks the question the table answers.
    #[test]
    fn the_shipped_example_resolves_no_datum() {
        let cfg = parse(EXAMPLE).expect("the shipped example parses");
        assert_eq!(cfg.datum, None);
        assert_eq!(cfg.datum().unwrap_err(), ConfigError::MissingDatum);
        assert_eq!(cfg.resolve().unwrap_err(), ConfigError::MissingDatum);
    }

    #[test]
    fn the_example_file_says_what_the_defaults_say() {
        // The example spells out every key. If it drifts from the defaults, one
        // of the two is a stale transcription and an operator reading the file
        // would be misled about what omitting a key does.
        let example = parse(EXAMPLE).expect("the shipped example parses");
        let defaults = minimal();
        assert_eq!(example.bus, defaults.bus);
        assert_eq!(example.envelope, defaults.envelope);
        assert_eq!(example.disarm, defaults.disarm);
        assert_eq!(example.provision, defaults.provision);
        // The one key the example states outright rather than leaving to its
        // fallback, because the fallback is derived from another key and a file
        // that showed nothing there would hide the relationship.
        assert_eq!(example.motion.read_loss_ticks, Some(example.motion.tick_hz));
        assert_eq!(
            MotionSection {
                read_loss_ticks: None,
                ..example.motion
            },
            defaults.motion
        );
        // `[arm]` too, excluding only the two keys that have no default: the
        // servo-side profile has to be sized against the moves this bench makes
        // rather than inherited. Everything else in the section — the arming
        // voltage floor, the pull-in bound, the gains — is a figure an operator
        // reads off this file and would be misled by if it drifted.
        assert_eq!(
            ArmSection {
                profile_acceleration: defaults.arm.profile_acceleration,
                profile_velocity: defaults.arm.profile_velocity,
                ..example.arm
            },
            defaults.arm
        );
    }

    #[test]
    fn the_defaults_are_the_libraries_own() {
        let cfg = minimal();
        let env = cfg.envelope().expect("the default envelope resolves");
        let library = EnvelopeConfig::default();
        assert_eq!(env.crank_windows, library.crank_windows);
        assert_eq!(env.relative_yaw_limit, library.relative_yaw_limit);
        assert_eq!(env.head_cone_limit, library.head_cone_limit);
        assert!((env.min_toggle_margin - library.min_toggle_margin).abs() < 1e-15);
        // The one deliberate departure: the bench works well inside the
        // mechanical yaw figure.
        assert!(env.body_yaw_limit < library.body_yaw_limit);
        assert_eq!(env.body_yaw_limit, 60.0_f64.to_radians());

        let resolved = cfg.resolve().expect("the minimal configuration resolves");
        assert_eq!(
            resolved.arm.min_arm_voltage,
            reachy_motion::arm::DEFAULT_MIN_ARM_VOLTAGE
        );
        assert_eq!(
            resolved.arm.voltage_budget,
            reachy_motion::arm::DEFAULT_VOLTAGE_BUDGET
        );
        assert_eq!(
            resolved.arm.voltage_poll_period,
            reachy_motion::arm::DEFAULT_VOLTAGE_POLL_PERIOD
        );
        assert_eq!(resolved.arm.gains, reachy_motion::arm::DEFAULT_GAINS);
        assert!(
            (resolved.arm.max_pin_pull_in - reachy_motion::arm::DEFAULT_MAX_PIN_PULL_IN).abs()
                < 1e-15
        );
        assert!(
            (resolved.arm.recheck_tolerance - reachy_motion::arm::DEFAULT_RECHECK_TOLERANCE).abs()
                < 1e-15
        );
        assert!(
            (resolved.arm.goal_shadow_tolerance
                - reachy_motion::arm::DEFAULT_GOAL_SHADOW_TOLERANCE)
                .abs()
                < 1e-15
        );
        assert!(
            (resolved.disarm.tolerance - reachy_motion::disarm::DEFAULT_STOW_TOLERANCE).abs()
                < 1e-15
        );
        assert_eq!(
            resolved.disarm.dwell,
            reachy_motion::disarm::DEFAULT_STOW_DWELL
        );
        assert!(!resolved.disarm.force_drop);
        assert_eq!(resolved.timing.baud, DEFAULT_BAUD);
        assert_eq!(resolved.timing, BusTiming::default());
    }

    #[test]
    fn the_read_loss_budget_defaults_to_a_second_at_the_tick_rate() {
        let cfg = minimal();
        let resolved = cfg.resolve().expect("resolves");
        assert_eq!(resolved.motion.read_loss_ticks, resolved.tick_hz);

        let stated = parse(&format!(
            "{MINIMAL}\n[motion]\ntick_hz = 100\nread_loss_ticks = 7\n"
        ))
        .expect("parses");
        let resolved = stated.resolve().expect("resolves");
        assert_eq!(resolved.tick_hz, 100);
        assert_eq!(resolved.motion.read_loss_ticks, 7);

        let derived = parse(&format!("{MINIMAL}\n[motion]\ntick_hz = 100\n")).expect("parses");
        assert_eq!(
            derived.resolve().expect("resolves").motion.read_loss_ticks,
            100
        );
    }

    #[test]
    fn a_file_without_a_datum_refuses() {
        let text = "[arm]\nprofile_acceleration = 50\nprofile_velocity = 300\n";
        let cfg = parse(text).expect("parses");
        assert_eq!(cfg.datum().unwrap_err(), ConfigError::MissingDatum);
        assert_eq!(cfg.resolve().unwrap_err(), ConfigError::MissingDatum);
    }

    #[test]
    fn a_datum_without_provenance_refuses() {
        let text = "[arm]\nprofile_acceleration = 50\nprofile_velocity = 300\n\
                    [datum]\ncrank_datum = \"direct\"\nprovenance = \"   \"\n";
        let cfg = parse(text).expect("parses");
        assert_eq!(
            cfg.resolve().unwrap_err(),
            ConfigError::DatumProvenanceEmpty
        );
    }

    #[test]
    fn the_profile_has_no_default_and_an_unknown_key_is_refused() {
        // Omitting the profile is not a configuration with a default profile;
        // it is a configuration that has not said what the backstop is.
        let missing = parse("[datum]\ncrank_datum = \"direct\"\nprovenance = \"x\"\n");
        assert!(missing.is_err());
        let message = format!("{:#}", missing.unwrap_err());
        assert!(message.contains("arm"), "{message}");

        let typo = parse(&format!("{MINIMAL}\n[motion]\ntick_hertz = 50\n"));
        let message = format!("{:#}", typo.expect_err("an unknown key is refused"));
        assert!(message.contains("tick_hertz"), "{message}");
    }

    /// The datum has one record, and this file is not it.
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
            reserved.resolve().unwrap_err(),
            ConfigError::ServoIdReserved { row: 8, id: 254 }
        );

        let duplicate = parse(&format!(
            "{MINIMAL}\n[bus]\nservo_ids = [10, 11, 12, 13, 14, 15, 16, 17, 11]\n"
        ))
        .expect("parses");
        assert_eq!(
            duplicate.resolve().unwrap_err(),
            ConfigError::ServoIdDuplicate { id: 11 }
        );
    }

    #[test]
    fn the_two_fences_agree_on_the_shipped_windows() {
        let cfg = minimal();
        let resolved = cfg.resolve().expect("resolves");
        let env = cfg.envelope().expect("resolves");
        for (leg, (lower, upper)) in resolved.arm.leg_windows.iter().enumerate() {
            let (env_lower, env_upper) = env.crank_windows[leg];
            assert!(
                *lower >= env_lower && *upper <= env_upper,
                "leg {leg}: {lower}..{upper} outside {env_lower}..{env_upper}"
            );
            assert!(lower - env_lower <= ONE_COUNT_RAD);
            assert!(env_upper - upper <= ONE_COUNT_RAD);
        }
        // The tightest and loosest of the twelve bounds, so a change to either
        // record shows up as a number rather than as a passing inequality.
        let interior: Vec<f64> = resolved
            .arm
            .leg_windows
            .iter()
            .enumerate()
            .flat_map(|(leg, (lower, upper))| {
                let (env_lower, env_upper) = env.crank_windows[leg];
                [
                    (lower - env_lower).to_degrees(),
                    (env_upper - upper).to_degrees(),
                ]
            })
            .collect();
        let tightest = interior.iter().copied().fold(f64::INFINITY, f64::min);
        let loosest = interior.iter().copied().fold(f64::NEG_INFINITY, f64::max);
        assert!(
            (tightest - 0.011_718_75).abs() < 1e-9,
            "tightest {tightest}"
        );
        // An eighth and three eighths of the 0.087_890_625° count.
        assert!((loosest - 0.039_062_5).abs() < 1e-9, "loosest {loosest}");
    }

    /// A refused fence names the leg and both windows in both domains.
    ///
    /// The two records come from different places — the servos' provisioned
    /// limits and this file's envelope — and the refusal exists because which of
    /// them is wrong is a person's call. A message that named only the leg would
    /// send that person back to read both tables themselves.
    #[test]
    fn a_refused_fence_names_the_leg_and_both_windows_in_both_domains() {
        let mut cfg = minimal();
        // Two counts out on leg 1's low bound: past the one count of slack the
        // whole-degrees-against-whole-counts derivation leaves.
        cfg.provision.leg_position_limits[0][0] -= 2;
        let error = cfg.resolve().unwrap_err();
        let ConfigError::FenceMismatch(mismatch) = error else {
            panic!("expected a fence mismatch, got {error}");
        };
        assert_eq!(mismatch.leg, 0);
        assert_eq!(mismatch.servo_counts, [1500, 2958]);
        assert!((mismatch.servo_deg[0] + 48.164).abs() < 0.01, "{mismatch}");
        assert!((mismatch.servo_deg[1] - 79.980).abs() < 0.01, "{mismatch}");
        assert!((mismatch.envelope_deg[0] + 48.0).abs() < 1e-9);
        assert!((mismatch.envelope_deg[1] - 80.0).abs() < 1e-9);
        assert_eq!(mismatch.envelope_counts, [Some(1502), Some(2958)]);

        let message = mismatch.to_string();
        assert!(message.contains("leg 1"), "{message}");
        assert!(message.contains("1500..2958"), "{message}");
        assert!(message.contains("1502..2958 counts"), "{message}");
        assert!(message.contains("-48.000°..80.000°"), "{message}");
    }

    #[test]
    fn a_count_window_that_is_not_the_envelope_window_refuses() {
        let mut cfg = minimal();
        // One count wider than the envelope on the low side.
        cfg.provision.leg_position_limits[2][0] -= 1;
        let error = cfg.resolve().unwrap_err();
        let ConfigError::FenceMismatch(mismatch) = error else {
            panic!("expected a fence mismatch, got {error}");
        };
        assert_eq!(mismatch.leg, 2);

        // Two counts narrower than the envelope on the high side: inside it, but
        // no longer the same fence.
        let mut cfg = minimal();
        cfg.provision.leg_position_limits[4][1] -= 2;
        let error = cfg.resolve().unwrap_err();
        let ConfigError::FenceMismatch(mismatch) = error else {
            panic!("expected a fence mismatch, got {error}");
        };
        assert_eq!(mismatch.leg, 4);

        // An empty window is refused before it is mapped at all.
        let mut cfg = minimal();
        cfg.provision.leg_position_limits[1] = [2844, 1138];
        assert_eq!(
            cfg.resolve().unwrap_err(),
            ConfigError::EmptyCountWindow {
                leg: 1,
                lower: 2844,
                upper: 1138
            }
        );
    }

    #[test]
    fn a_count_no_register_holds_refuses() {
        let mut cfg = minimal();
        cfg.provision.leg_position_limits[3] = [1138, u32::MAX];
        assert_eq!(
            cfg.resolve().unwrap_err(),
            ConfigError::CountOutOfRange {
                leg: 3,
                count: u32::MAX
            }
        );
    }

    #[test]
    fn an_envelope_window_that_is_not_a_window_refuses() {
        let mut cfg = minimal();
        cfg.envelope.crank_windows_deg[5] = [48.0, -80.0];
        assert_eq!(
            cfg.envelope().unwrap_err(),
            ConfigError::EmptyEnvelopeWindow {
                leg: 5,
                lower_deg: 48.0,
                upper_deg: -80.0
            }
        );
    }

    #[test]
    fn every_scalar_that_must_be_positive_is_checked() {
        // Each key, mutated to something that is not a positive real number,
        // must come back naming itself. A key that silently accepted a zero
        // would put a zero step bound or a zero tick rate into the pump.
        let cases: [Mutation; 15] = [
            ("motion.stow_duration_s", |c| {
                c.motion.stow_duration_s = 0.0;
            }),
            ("motion.up_duration_s", |c| c.motion.up_duration_s = -1.0),
            ("motion.move_duration_s", |c| {
                c.motion.move_duration_s = f64::NAN;
            }),
            ("motion.hold_duration_s", |c| {
                c.motion.hold_duration_s = 0.0;
            }),
            ("envelope.body_yaw_limit_deg", |c| {
                c.envelope.body_yaw_limit_deg = 0.0;
            }),
            ("envelope.relative_yaw_limit_deg", |c| {
                c.envelope.relative_yaw_limit_deg = f64::NAN;
            }),
            ("envelope.head_cone_limit_deg", |c| {
                c.envelope.head_cone_limit_deg = -1.0;
            }),
            ("envelope.min_toggle_margin_mm", |c| {
                c.envelope.min_toggle_margin_mm = 0.0;
            }),
            ("motion.max_step_legs_rad", |c| {
                c.motion.max_step_legs_rad = 0.0;
            }),
            ("motion.max_step_body_yaw_rad", |c| {
                c.motion.max_step_body_yaw_rad = -0.1;
            }),
            ("motion.max_step_antennas_rad", |c| {
                c.motion.max_step_antennas_rad = f64::NAN;
            }),
            ("motion.tracking_threshold_rad", |c| {
                c.motion.tracking_threshold_rad = 0.0;
            }),
            ("motion.tracking_progress_min_rad", |c| {
                c.motion.tracking_progress_min_rad = 0.0;
            }),
            ("arm.min_arm_voltage", |c| {
                c.arm.min_arm_voltage = 0.0;
            }),
            ("disarm.tolerance_deg", |c| {
                c.disarm.tolerance_deg = 0.0;
            }),
        ];
        for (key, mutate) in cases {
            let mut cfg = minimal();
            mutate(&mut cfg);
            let error = cfg.resolve().unwrap_err();
            let ConfigError::NotPositive { key: named, .. } = error else {
                panic!("{key}: expected a positivity refusal, got {error}");
            };
            assert_eq!(named, key);
        }

        let integers: [Mutation; 7] = [
            ("motion.tick_hz", |c| c.motion.tick_hz = 0),
            ("motion.tracking_ticks", |c| c.motion.tracking_ticks = 0),
            ("motion.read_loss_ticks", |c| {
                c.motion.read_loss_ticks = Some(0);
            }),
            ("motion.health_poll_hz", |c| c.motion.health_poll_hz = 0),
            ("bus.baud", |c| c.bus.baud = 0),
            ("bus.host_allowance_ms", |c| c.bus.host_allowance_ms = 0),
            ("arm.voltage_poll_period_ms", |c| {
                c.arm.voltage_poll_period_ms = 0;
            }),
        ];
        for (key, mutate) in integers {
            let mut cfg = minimal();
            mutate(&mut cfg);
            assert_eq!(
                cfg.resolve().unwrap_err(),
                ConfigError::NotPositiveInt { key }
            );
        }

        // The two integers where zero is a choice rather than a mistake, so
        // nobody later adds them to the sweep above by symmetry: the transaction
        // layer makes the first attempt whatever the count says, and a retry
        // after a spent deadline needs no further pause.
        let mut cfg = minimal();
        cfg.bus.retry_attempts = 0;
        cfg.bus.retry_spacing_ms = 0;
        let timing = cfg.resolve().expect("resolves").timing;
        assert_eq!(timing.retry_attempts, 0);
        assert_eq!(timing.retry_spacing, Duration::ZERO);
    }

    /// A step bound wider than the tracking threshold refuses, naming both
    /// numbers.
    ///
    /// Nothing in the file's own numbers says the two are related, so
    /// lowering the threshold has to be refused rather than silently arming
    /// a spurious fault.
    #[test]
    fn a_step_bound_wider_than_the_tracking_band_refuses() {
        let shipped = minimal();
        shipped
            .resolve()
            .expect("the shipped numbers hold the invariant");

        let cases: [Mutation; 3] = [
            ("motion.max_step_legs_rad", |c| {
                c.motion.max_step_legs_rad = 0.2;
            }),
            ("motion.max_step_body_yaw_rad", |c| {
                c.motion.max_step_body_yaw_rad = 0.2;
            }),
            ("motion.max_step_antennas_rad", |c| {
                c.motion.max_step_antennas_rad = 0.2;
            }),
        ];
        for (key, mutate) in cases {
            let mut cfg = minimal();
            mutate(&mut cfg);
            cfg.motion.tracking_threshold_rad = 0.1;
            let error = cfg.resolve().unwrap_err();
            assert_eq!(
                error,
                ConfigError::StepPastTrackingThreshold {
                    key,
                    step: 0.2,
                    threshold: 0.1,
                },
                "{key}"
            );
            let printed = error.to_string();
            assert!(printed.contains(key), "{printed}");
            assert!(
                printed.contains("motion.tracking_threshold_rad"),
                "{printed}"
            );
        }

        // Equal is the shipped relationship and is admitted: a goal step of
        // exactly the threshold reaches the edge of the band, not past it.
        let mut cfg = minimal();
        cfg.motion.max_step_legs_rad = 0.15;
        cfg.motion.tracking_threshold_rad = 0.15;
        cfg.resolve()
            .expect("a step of exactly the band is admitted");
    }

    /// A number of seconds no duration can hold is a refusal naming the key, not
    /// a panic. `Duration::from_secs_f64` panics on a finite value past its
    /// range, and every one of these keys is operator-typed.
    #[test]
    fn a_duration_longer_than_a_duration_refuses() {
        let cases: [Mutation; 6] = [
            ("motion.stow_duration_s", |c| {
                c.motion.stow_duration_s = 1e30;
            }),
            ("motion.up_duration_s", |c| c.motion.up_duration_s = 1e30),
            ("motion.move_duration_s", |c| {
                c.motion.move_duration_s = 1e30;
            }),
            ("motion.hold_duration_s", |c| {
                c.motion.hold_duration_s = 1e30;
            }),
            ("arm.voltage_budget_s", |c| c.arm.voltage_budget_s = 30e30),
            ("disarm.dwell_s", |c| c.disarm.dwell_s = f64::MAX),
        ];
        for (key, mutate) in cases {
            let mut cfg = minimal();
            mutate(&mut cfg);
            let error = cfg.resolve().unwrap_err();
            let ConfigError::DurationOutOfRange { key: named, .. } = error else {
                panic!("{key}: expected a range refusal, got {error}");
            };
            assert_eq!(named, key);
            assert!(error.to_string().contains(key), "{error}");
        }
    }

    #[test]
    fn a_zero_dwell_is_allowed_and_a_negative_one_is_not() {
        let mut cfg = minimal();
        cfg.disarm.dwell_s = 0.0;
        assert_eq!(
            cfg.resolve().expect("resolves").disarm.dwell,
            Duration::ZERO
        );

        cfg.disarm.dwell_s = -1.0;
        assert_eq!(
            cfg.resolve().unwrap_err(),
            ConfigError::NotPositive {
                key: "disarm.dwell_s",
                value: -1.0
            }
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
                Some(ProvisionExpect::Check(RegValue::U8(mode))),
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
                Some(ProvisionExpect::Check(RegValue::I32(offset))),
                "row {row}"
            );
        }
        assert_eq!(RegValue::I32(-1024).to_string(), "-1024");

        // Per servo, and on this unit every servo holds the same ceiling.
        let column = ProvisionTable::column(RegId::CurrentLimit).expect("provisioned");
        for row in 0..JointId::COUNT {
            assert_eq!(
                table.at(row, column),
                Some(ProvisionExpect::Check(RegValue::U16(1750))),
                "row {row}"
            );
        }
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
            for row in 0..JointId::COUNT {
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
        for key in ["profile_acceleration", "profile_velocity"] {
            let stale = parse(&format!("{MINIMAL}\n[provision]\n{key} = 0\n"));
            let message = format!("{:#}", stale.expect_err("there is no such key"));
            assert!(message.contains(key), "{message}");
        }

        let stale = parse(
            "[arm]\nprofile_acceleration = 50\nprofile_velocity = 300\n\
             repin_tolerance_deg = 0.5\n",
        );
        let message = format!("{:#}", stale.expect_err("there is no such key"));
        assert!(message.contains("repin_tolerance_deg"), "{message}");

        // `antenna_limit_rad` is not a recognised key: a file carrying it
        // says so rather than setting nothing.
        let stale = parse(&format!(
            "{MINIMAL}\n[envelope]\nantenna_limit_rad = 3.05\n"
        ));
        let message = format!("{:#}", stale.expect_err("there is no such key"));
        assert!(message.contains("antenna_limit_rad"), "{message}");

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

    #[test]
    fn stow_is_a_pose_the_configured_geometry_reaches() {
        let cfg = minimal();
        let resolved = cfg.resolve().expect("resolves");
        let stow = stow_targets(&HeadGeometry::default()).expect("stow solves");
        assert_eq!(resolved.disarm.stow_targets, stow);
        assert_eq!(resolved.disarm.ids, resolved.arm.ids);
    }

    /// Every duration key lands in its own resolved field.
    ///
    /// Four distinct values, because the resolve arms are a copy-paste block
    /// and equal values would let two of them read the same key: a `hold` that
    /// watched for the move duration and a `yaw` that ran for the hold duration
    /// would be a move carried over the wrong length of time with torque on.
    #[test]
    fn the_durations_are_what_the_file_said() {
        let cfg = parse(&format!(
            "{MINIMAL}\n[motion]\nstow_duration_s = 2.5\nup_duration_s = 4.0\n\
             move_duration_s = 1.5\nhold_duration_s = 0.5\n"
        ))
        .expect("parses");
        let resolved = cfg.resolve().expect("resolves");
        assert_eq!(resolved.stow_duration, Duration::from_millis(2500));
        assert_eq!(resolved.up_duration, Duration::from_secs(4));
        assert_eq!(resolved.move_duration, Duration::from_millis(1500));
        assert_eq!(resolved.hold_duration, Duration::from_millis(500));
        assert_eq!(resolved.device, "/dev/ttyAMA3");
    }

    #[test]
    fn a_configuration_that_verifies_nothing_refuses() {
        // No file can express an empty table, so the guard is called directly
        // here to reach both branches.
        assert_eq!(
            check_table(&ProvisionTable::new()),
            Err(ConfigError::NothingChecked)
        );
        assert_eq!(check_table(&minimal().provision_table()), Ok(()));
        assert!(
            ConfigError::NothingChecked
                .to_string()
                .contains("never verified")
        );
    }

    #[test]
    fn an_unplaceable_envelope_bound_renders_rather_than_hiding() {
        assert_eq!(render_count(Some(-7)), "-7");
        assert_eq!(render_count(None), "unplaceable");
    }
}
