//! What the motion system's channels are called.
//!
//! A channel name is the join key between three artifacts: the `channel`
//! declarations the `.clk` modules make, the logging policies each system
//! declares over them, and every Rust reader of a log. A rename that lands in
//! one Rust spelling and not another surfaces as a channel reported missing at
//! the top of a report, which is the worst place to meet one. So the names are
//! stated once, here, and every reader takes them from this module.
//!
//! Its own crate, holding nothing but strings: an operator's tool must be able
//! to name a channel without depending on the scenario harness, and the harness
//! must be able to name one without depending on anything of an operator's.

/// The channel the scripts arrive on, fed from the input log in a scenario and
/// published by the wake cog on a machine.
pub const SCRIPT_CHANNEL: &str = "ScriptsIn";

/// The channel the session publishes what it accepted on.
pub const SCHEDULE_CHANNEL: &str = "ScheduleChan";

/// The channel the scenario's injections arrive on, fed from the input log.
///
/// Only the simulated system has one: there is no plant to inject into on a
/// machine, so a reader of a hardware log must not bind it.
pub const SIM_CMD_CHANNEL: &str = "SimCmdChan";

/// The driver's sample stream.
pub const POSE_CHANNEL: &str = "DriverPose";

/// The goal stream.
pub const CMD_CHANNEL: &str = "DriverCmd";

/// The driver's events.
pub const EVENT_CHANNEL: &str = "DriverEvt";

/// What the decision tick raised.
pub const FAULT_CHANNEL: &str = "TickFaults";

/// What the session asks of the driver: one datagram per wake at most.
pub const SESSION_CMD_CHANNEL: &str = "SessionCmdChan";

/// What the session said about the session: one report per wake, oldest first.
pub const REPORT_CHANNEL: &str = "ReportsOut";

/// Where the head was.
pub const ESTIMATE_CHANNEL: &str = "Estimates";

/// How each out-of-band transaction the driver ran turned out.
pub const AUX_OUT_CHANNEL: &str = "DriverAuxOut";

/// What the driver's health rotation read.
pub const HEALTH_CHANNEL: &str = "DriverHealth";

/// Everything the driver knows about its own run, republished on a cadence.
///
/// Cumulative: the newest message is the whole account, so a reader that saw
/// only one of them has read the run.
pub const STATUS_CHANNEL: &str = "DriverStatus";
