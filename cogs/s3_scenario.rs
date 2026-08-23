//! S3, dead-man: the scenario that says an energised machine nobody is
//! commanding de-torques itself.
//!
//! Nothing engages. The machine is energised, as an arming sequencer would
//! leave it, and then no session ever asks it for anything -- so the decision
//! tick publishes no goals, the driver's gate sees a goal stream that never
//! starts, and after the hold timeout it writes the torque off and says so.
//!
//! The session is not idle for the whole of it: it commissions the machine
//! first, the way it does at the start of every run, and every datagram that
//! costs is liveness the gate counts. The silence therefore begins where the
//! survey ends, which is why the run is long enough to hold both and why the
//! checker measures the dead-man from the last thing the session said rather
//! than from the run's first cycle.
//!
//! This is the shortest scenario in the suite and the one that covers the most
//! ground per cycle: it is the machine's last line of defence, the one that
//! holds whatever else broke. Everything else in the suite is written so the
//! dead-man never fires; this is the one that makes sure it can.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once.

use scenario::{commission_allowance_cycles, hold_timeout_cycles};

/// The cycle the machine is energised at.
pub const TORQUE_ON_CYCLE: i64 = 0;

/// The last cycle of the run.
///
/// The survey first: the session commissions the machine before it does anything
/// else, and every datagram it sends to do that is liveness the driver's gate
/// counts. So the silence this scenario is about cannot begin until the survey
/// is over, and the run allows for it.
///
/// Then comfortably past the latch, because half of what the scenario asserts is
/// about what happens *after* it: the machine stays de-torqued, stays where it
/// stood, and the gate does not go on raising the same event.
#[must_use]
pub fn end_cycle() -> i64 {
    TORQUE_ON_CYCLE + commission_allowance_cycles() + 4 * hold_timeout_cycles()
}
