//! S3, dead-man: the scenario that says an energised machine nobody is
//! commanding de-torques itself.
//!
//! Nothing engages. The machine is energised, as an arming sequencer would
//! leave it, and then no session ever asks it for anything -- so the decision
//! tick publishes no goals, the driver's gate sees a goal stream that never
//! starts, and after the hold timeout it writes the torque off and says so.
//!
//! This is the shortest scenario in the suite and the one that covers the most
//! ground per cycle: it is the machine's last line of defence, the one that
//! holds whatever else broke. Everything else in the suite is written so the
//! dead-man never fires; this is the one that makes sure it can.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once.

use scenario::{cycle_at, hold_timeout_cycles};

/// The cycle the machine is energised at.
pub const TORQUE_ON_CYCLE: i64 = 0;

/// The epoch of the one schedule the run carries.
pub const IDLE_EPOCH: u32 = 1;

/// The last cycle of the run.
///
/// Comfortably past the latch, because half of what the scenario asserts is
/// about what happens *after* it: the machine stays de-torqued, stays where it
/// stood, and the gate does not go on raising the same event.
#[must_use]
pub fn end_cycle() -> i64 {
    TORQUE_ON_CYCLE + 4 * hold_timeout_cycles()
}

/// The simulated time the run ends at.
#[must_use]
pub fn end_time_ns() -> i64 {
    cycle_at(end_cycle())
}
