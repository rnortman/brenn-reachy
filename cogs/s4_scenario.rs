//! S4, feedback loss: the scenario that says a machine the loop can no longer
//! see ends up de-torqued.
//!
//! S1's session, held at the upright posture, and then the bus stops answering
//! for a while. What that exercises is the path no other scenario reaches: the
//! decision tick counting reads that did not arrive, latching once the run of
//! them is long enough to mean something, reporting that exactly once, and
//! then commanding nothing ever again -- which stops the keep-alive, which
//! leaves the driver's gate measuring a silence, which de-torques the machine.
//!
//! That last step is the point. A latching fault in this slice has no
//! sequencer to run a stow ladder, and it does not need one: the goal stream
//! ceasing *is* the response, and the dead-man behind it is what puts the
//! machine in the minimum-risk condition. The scenario asserts the whole chain,
//! link by link, because each link is a different cog and any one of them
//! quietly not doing its part would leave a machine energised with nobody
//! commanding it.
//!
//! The reads come back before the gate's window runs out, deliberately: nothing
//! recovers. A latched fault dies with the engagement, and a scenario where the
//! bus returned and the machine carried on would be describing a flag that
//! cleared itself.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use scenario::author::Step;
use scenario::{UP_DURATION_NS, cycle_at, cycles_for, dead_man_latch_cycle, silence_ns};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;
use reachy_motion::default_motion_config;

/// The cycle the session is engaged and the machine energised at.
pub const ENGAGE_CYCLE: i64 = 0;

/// The cycle the bus stops answering on.
///
/// Well after the machine has arrived upright and settled into its hold, so
/// what the run is about is a loop that lost its measurements rather than one
/// that lost them mid-move. A move interrupted by an outage is a different
/// question, and mixing the two would leave every assertion here with two
/// possible causes.
pub const OUTAGE_CYCLE: i64 = 60;

/// How many cycles of position replies are lost.
///
/// Longer than the tick's tolerance, so the outage is long enough to mean
/// something, and long enough afterwards that the reads are back well before
/// the gate de-torques the machine -- which is what makes "nothing recovers" an
/// assertion about the loop rather than about the timing.
pub const OUTAGE_CYCLES: u32 = 60;

/// How long the run continues past the de-torquing, in cycles.
pub const TAIL_CYCLES: i64 = 20;

/// The epoch of the one schedule the run carries.
pub const ENGAGED_EPOCH: u32 = 1;

/// How many missed reads in a row the decision tick tolerates before it says
/// the machine's position feedback is gone.
#[must_use]
pub fn read_loss_cycles() -> i64 {
    i64::from(default_motion_config().read_loss_ticks)
}

/// The cycle the decision tick reports the loss on.
///
/// The tick raises once the run of misses is *past* what it tolerates, so the
/// report lands on the read after the last one it forgave: the first blind
/// sample is one miss, and the raise is on the miss numbered one past the
/// tolerance.
#[must_use]
pub fn fault_cycle() -> i64 {
    OUTAGE_CYCLE + read_loss_cycles()
}

/// How many missed reads the report should name.
#[must_use]
pub fn reported_misses() -> u32 {
    u32::try_from(read_loss_cycles() + 1).unwrap_or(u32::MAX)
}

/// The cycle the gate de-torques the machine on.
///
/// The last goal the tick published was decided on the cycle before it faulted,
/// and the driver drains it on the cycle after that -- which is the fault cycle
/// itself. So the gate's window opens there, and when it latches is the gate's
/// own arithmetic rather than this scenario's.
#[must_use]
pub fn latch_cycle() -> i64 {
    dead_man_latch_cycle(fault_cycle())
}

/// How long the gate should say the goal stream was silent for, nanoseconds.
#[must_use]
pub fn reported_silence_ns() -> i64 {
    silence_ns(fault_cycle(), latch_cycle())
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    latch_cycle() + TAIL_CYCLES
}

/// The simulated time the run ends at.
#[must_use]
pub fn end_time_ns() -> i64 {
    cycle_at(end_cycle())
}

/// The session's one step: upright, for the whole run.
///
/// One step rather than S1's two, because a schedule that moved on to another
/// posture part way through would be commanding a machine that had already
/// stopped taking commands -- which says nothing, and would make the run's
/// silence ambiguous between a fault and a session that ran out of steps.
#[must_use]
pub fn steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(ENGAGE_CYCLE),
        end_ns: cycle_at(end_cycle()),
        posture: Some(PostureWire::UP),
    }]
}

/// How many cycles a move to the upright posture is given, rounded up.
#[must_use]
pub fn up_cycles() -> i64 {
    cycles_for(UP_DURATION_NS)
}
