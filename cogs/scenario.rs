//! What every scenario of the motion system is made of.
//!
//! A scenario is three programs sharing one statement of what the run is: an
//! author that turns the statement into an input log, the deterministic runner
//! that plays the log through the system, and a checker that joins the output
//! log back to the same statement. A checker whose expectations were restated in
//! its own source would pass by agreeing with itself; this crate is where the
//! agreement lives.
//!
//! Three parts, and the split is the one the harness has: [`author`] writes an
//! input log, [`read`] takes an output log apart into typed streams, and
//! [`check`] holds the assertions every scenario of this system makes about the
//! result. The facts they all rest on -- the epoch, the grid, the channel names,
//! and the configured numbers the cogs run on -- are here at the top.
//!
//! Times are absolute simulated nanoseconds since the Unix epoch. The
//! deterministic runner jumps its clock to each logged message's transmit time,
//! so a scenario's schedule is the times it writes; nothing rebases them.

// The crate root's own directory is where rustc looks for a submodule, and this
// crate's root is a file in a package of many; the two halves live in a
// directory of their own so the package's file names stay about their subjects.
#[path = "scenario/author.rs"]
pub mod author;
#[path = "scenario/check.rs"]
pub mod check;
#[path = "scenario/read.rs"]
pub mod read;

/// The scenario epoch: an arbitrary round Unix time, far enough from zero that
/// a dropped or defaulted timestamp reads as obviously wrong rather than as a
/// plausible small number.
pub const T0_NS: i64 = 1_700_000_000_000_000_000;

/// The bus cycle. Every sample sits on this grid, the plant advances one step
/// per cycle, and the decision tick dates its goals in multiples of it.
pub const PERIOD_NS: i64 = 20_000_000;

/// How many cycles ahead of the sample that decided it a goal is dated.
pub const LAG_K: i64 = 2;

/// How long a move to the upright posture is given.
pub const UP_DURATION_NS: i64 = 800_000_000;

/// How long a move to stow is given.
pub const STOW_DURATION_NS: i64 = 2_000_000_000;

/// How long the goal stream may be silent before the gate de-torques.
pub const HOLD_TIMEOUT_NS: i64 = 200_000_000;

/// How far a crank moves in one cycle, radians.
pub const SLEW_LEGS_RAD: f64 = 0.15;

/// How far the body yaw moves in one cycle, radians. Its own number rather than
/// the cranks': the plant configures the three groups separately, and a scenario
/// that could not say they differ could not run one where they do.
pub const SLEW_BODY_YAW_RAD: f64 = 0.15;

/// Whether the modelled machine starts energised. Every scenario so far starts
/// it cold and torques it on with an injection, which is what an arming
/// sequencer does on the real machine.
pub const START_TORQUED: bool = false;

/// How far an antenna moves in one cycle, radians.
pub const SLEW_ANTENNAS_RAD: f64 = 0.65;

/// How long an execution is modelled to take, which is the gap between the
/// instant a cog runs at and the log time of what it published.
///
/// Every cog in this system declares the same duration, so a message's log time
/// is always its cog's start time plus this. It is not jitter and not a
/// tolerance: the run is exact, and a checker that conflated a message's log
/// time with the instant its contents are about would be reading two clocks as
/// one.
///
/// The number this system's cog modules declare, not a repo-wide one: the
/// harness proof next door declares its own, and a single constant behind both
/// would let one system's `.clk` change break the other system's assertions.
pub const EXECUTION_DURATION_NS: i64 = 1_000_000;

/// How long after the cycle it is about a control-rate cog's message is logged.
///
/// Two execution durations, not one: the driver runs at the cycle's nominal
/// instant and publishes its sample a duration later, and the cogs that run on
/// that sample publish a duration after *that*. So a goal or an estimate lands
/// in the log two milliseconds after the instant its contents are about, and
/// the instants themselves -- a goal's `execute_at`, an estimate's
/// `time_of_validity` -- are arithmetic off the cycle rather than off any
/// publish.
pub const CONTROL_DELAY_NS: i64 = 2 * EXECUTION_DURATION_NS;

/// The channel the session's schedule arrives on, fed from the input log.
pub const SCHEDULE_CHANNEL: &str = "ScheduleChan";

/// The channel the scenario's injections arrive on, fed from the input log.
pub const SIM_CMD_CHANNEL: &str = "SimCmdChan";

/// The driver's sample stream.
pub const POSE_CHANNEL: &str = "DriverPose";

/// The goal stream.
pub const CMD_CHANNEL: &str = "DriverCmd";

/// The driver's events.
pub const EVENT_CHANNEL: &str = "DriverEvt";

/// What the decision tick raised.
pub const FAULT_CHANNEL: &str = "TickFaults";

/// Where the head was.
pub const ESTIMATE_CHANNEL: &str = "Estimates";

/// What a signal report group's channel is called, up to the cog that owns it.
///
/// A group's channel name is composed by the framework rather than declared, so
/// the shape is stated once here and the cog names below complete it. What
/// follows the group's own name is a digest of its generated schema, which is
/// why a scenario matches the prefix rather than the whole name.
pub const REPORT_GROUP_PREFIX: &str = "/_clockwork/report-groups/";

/// The group every cog of this system declares its counters in.
pub const REPORT_GROUP: &str = "stats";

/// The cogs of this system, each of which owns one report group.
pub const COGS: [&str; 3] = ["Mover", "Pose", "MotorSim"];

/// The cycle the simulated driver first executes on.
///
/// One period in rather than at the epoch: the driver runs on a periodic timer,
/// and the first firing of a timer started at the run's beginning is a period
/// later. Stated rather than observed, because a scenario that took the run's
/// first cycle from the run would move its whole expectation along with a
/// regression that delayed the driver -- and one of them, S3, has nothing else
/// pinning when its clock started.
pub const FIRST_CYCLE: i64 = 1;

/// How many cycles a stretch of `duration_ns` covers, rounded up.
///
/// Rounded up rather than down because the durations a scenario states are the
/// time a move is *given*: a move that runs into the fraction of a cycle at the
/// end of its budget has not overrun, and a scenario that rounded the other way
/// would assert the machine had arrived while it was still travelling.
#[must_use]
pub fn cycles_for(duration_ns: i64) -> i64 {
    (duration_ns + PERIOD_NS - 1) / PERIOD_NS
}

/// How many cycles the goal stream may be silent before the gate de-torques.
#[must_use]
pub fn hold_timeout_cycles() -> i64 {
    HOLD_TIMEOUT_NS / PERIOD_NS
}

/// The cycle the driver's gate latches its torque-off on, given the cycle the
/// silence it measures began on.
///
/// The gate compares the silence it has measured against the configured timeout
/// and latches once it is *past* it, so the latch lands one cycle further out
/// than the timeout itself.
#[must_use]
pub fn dead_man_latch_cycle(window_opened_at: i64) -> i64 {
    window_opened_at + hold_timeout_cycles() + 1
}

/// How long a stretch of cycles is, microseconds: what the gate reports as the
/// silence it measured.
///
/// Saturating the way the gate's own conversion does, at both ends: a stretch
/// that ran backwards is nothing, and one past seventy minutes is as long as the
/// field says. Two spellings of one conversion that clamped the low end
/// differently would have a scenario asserting seventy minutes of silence
/// against a driver reporting none, and send whoever read the failure to the
/// driver rather than to the arithmetic that asked for it.
#[must_use]
pub fn silence_us(from_cycle: i64, to_cycle: i64) -> u32 {
    let quiet = (to_cycle - from_cycle).max(0) * PERIOD_NS / 1_000;
    u32::try_from(quiet).unwrap_or(u32::MAX)
}

/// The cycle `nominal` sits on, counted from the epoch.
///
/// # Errors
///
/// How far off the grid `nominal` sits. Every instant in a deterministic run is
/// on it, so an off-grid one is a run that drifted -- and this is fed log data,
/// so it is the run under test that says so, not the caller. Returned rather
/// than thrown because a checker collects every failure: a drifted run is the
/// one whose other complaints explain why it drifted, and a panic here would
/// throw them away.
pub fn cycle_of(nominal_ns: i64) -> Result<i64, String> {
    let elapsed = nominal_ns - T0_NS;
    let off = elapsed % PERIOD_NS;
    if off != 0 {
        return Err(format!(
            "{nominal_ns} is {off}ns off the {PERIOD_NS}ns grid the run is on"
        ));
    }
    Ok(elapsed / PERIOD_NS)
}

/// The instant cycle `n` begins.
#[must_use]
pub fn cycle_at(n: i64) -> i64 {
    T0_NS + n * PERIOD_NS
}

/// The configured numbers above, as they are written in the textprotos the box
/// binds.
///
/// The constants in this module and the files the process reads are two
/// statements of the same numbers, and a scenario asserting "the goal is due two
/// cycles out" against a build configured for three would pass while describing
/// a machine nobody ran. Every checker calls this with the paths the test target
/// hands it, so a change to either side fails the scenario rather than shifting
/// what it means.
///
/// The parse is deliberately literal -- `key: value` lines, comments and blanks
/// skipped -- because it is checking a handful of scalars in a file this repo
/// writes, not implementing protobuf text. The values are compared as the
/// numbers they are rather than as the characters they were written with: what
/// the process reads is `0.15`, whether the file spells it `0.15` or `1.5e-1`,
/// and a check that failed over the spelling would send its next reader to edit
/// the constant.
///
/// # Errors
///
/// One line per number the file states differently, or per number it does not
/// state at all, or the reason the file could not be read.
pub fn check_params(mover_textproto: &str, sim_textproto: &str) -> Vec<String> {
    let mut failures = Vec::new();
    expect(
        mover_textproto,
        &[
            ("lag_k", Value::Int(LAG_K)),
            ("period_ns", Value::Int(PERIOD_NS)),
            ("up_duration_ns", Value::Int(UP_DURATION_NS)),
            ("stow_duration_ns", Value::Int(STOW_DURATION_NS)),
        ],
        &mut failures,
    );
    expect(
        sim_textproto,
        &[
            ("period_ns", Value::Int(PERIOD_NS)),
            ("hold_timeout_ns", Value::Int(HOLD_TIMEOUT_NS)),
            ("start_torqued", Value::Bool(START_TORQUED)),
            ("slew_legs_rad", Value::Float(SLEW_LEGS_RAD)),
            ("slew_body_yaw_rad", Value::Float(SLEW_BODY_YAW_RAD)),
            ("slew_antennas_rad", Value::Float(SLEW_ANTENNAS_RAD)),
        ],
        &mut failures,
    );
    failures
}

/// One configured scalar, as the kind of value its field holds.
#[derive(Clone, Copy, PartialEq)]
enum Value {
    /// A whole number: a count of cycles or of nanoseconds.
    Int(i64),
    /// An angle, radians.
    Float(f64),
    /// A choice.
    Bool(bool),
}

impl Value {
    /// The same value, read out of the text a file states it as, or `None` if
    /// those characters are not one of these at all.
    fn parse(self, text: &str) -> Option<Self> {
        match self {
            Self::Int(_) => text.parse().ok().map(Self::Int),
            // Exact equality on the parsed number, which is what "the file
            // states this number" means: the process gets the parse, not the
            // characters, and any rounding is the same rounding on both sides.
            Self::Float(_) => text.parse().ok().map(Self::Float),
            Self::Bool(_) => text.parse().ok().map(Self::Bool),
        }
    }
}

impl core::fmt::Display for Value {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Int(value) => write!(f, "{value}"),
            Self::Float(value) => write!(f, "{value}"),
            Self::Bool(value) => write!(f, "{value}"),
        }
    }
}

/// Assert one textproto states exactly these values for these keys.
fn expect(path: &str, wanted: &[(&str, Value)], failures: &mut Vec<String>) {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(err) => {
            failures.push(format!("reading {path}: {err}"));
            return;
        }
    };
    let stated: Vec<(&str, &str)> = text
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .filter_map(|line| line.split_once(':'))
        .map(|(key, value)| (key.trim(), value.trim()))
        .collect();
    for (key, value) in wanted {
        let Some((_, found)) = stated.iter().find(|(name, _)| name == key) else {
            failures.push(format!(
                "{path} states no {key}; the scenario needs {value}"
            ));
            continue;
        };
        match value.parse(found) {
            None => failures.push(format!(
                "{path} states {key}: {found}, which is not a value of that field's kind; the \
                 scenario is written for {value}"
            )),
            Some(parsed) if parsed != *value => failures.push(format!(
                "{path} states {key}: {found}, but the scenario is written for {value}"
            )),
            Some(_) => {}
        }
    }
}
