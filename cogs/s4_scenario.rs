//! S4, feedback loss: the scenario that says a machine the loop can no longer
//! see ends up de-torqued, by command.
//!
//! S1's run, held at the upright posture, and then the bus stops answering for
//! a while. The driver notices: twenty-five cycles of hearing nothing at all is
//! a bus it declares gone, which is the one fault it can raise about itself. The
//! session answers that evidence with the doctrine's response for a machine
//! nothing can be commanded through -- an immediate best-effort torque-off, and
//! a park an operator has to clear -- publishes a schedule nobody is engaged on,
//! and the driver reads the release back and confirms it once it can read
//! anything at all.
//!
//! The order is the point, and it is arithmetic rather than luck: the driver's
//! evidence is a run of blind cycles and the decision tick's would be a longer
//! run of missed reads, so the machine is released and the session over before
//! the tick's own tolerance runs out. What the tick therefore reports is
//! nothing, and the goal stream ends because the session let go rather than
//! because the loop gave up -- the one window in which goals reach a gate that
//! has already let go is the cycle between the release and the disengagement
//! reaching the mover.
//!
//! Nothing recovers. The reads come back after the machine has been released and
//! parked, and the run continues past them: a session that took a script or a
//! stream that started again would be describing a latch that cleared itself.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use scenario::author::Step;
use scenario::{BLIND_CYCLES_BEFORE_BUS_FAILURE, cycle_at, up_cycles};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;
use reachy_motion::default_motion_config;
use reachy_motion::joints::ROW_COUNT;

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The script's number.
pub const SCRIPT_ID: u32 = 4;

/// How long the machine holds upright before the bus goes away, in cycles: the
/// move plus room to arrive and settle into its hold.
pub const SETTLE_CYCLES: i64 = 20;

/// The cycle the bus stops answering on.
///
/// Past the start-up survey and past the arming, because a bus that goes away
/// carries their transactions off with it: this run is about a session that had
/// established the machine and taken hold of it and then lost the wire, and one
/// that never got through either would fail from a different phase and prove a
/// different thing. And well after the machine has arrived upright and settled
/// into its hold, so what the run is about is a loop that lost its measurements
/// rather than one that lost them mid-move -- a move interrupted by an outage is
/// a different question, and mixing the two would leave every assertion here
/// with two possible causes.
#[must_use]
pub fn outage_cycle() -> i64 {
    up_start_cycle() + up_cycles() + SETTLE_CYCLES
}

/// How many cycles of replies are lost.
///
/// Longer than the run of blind cycles the driver calls a dead bus, so its
/// evidence is on the record; and long enough after the release for the driver's
/// own confirmation budget to run out on a read-back that can read nothing,
/// which is the half of the handshake this run carries that no other does.
pub const OUTAGE_CYCLES: u32 = 60;

/// How long the run continues past the reads coming back, in cycles.
///
/// Long enough to carry the confirmation the driver owes once it can read again
/// -- one row a cycle over the whole bus -- and a stretch past that in which
/// nothing recovers.
pub const TAIL_CYCLES: i64 = 20;

/// The script's one step: upright, for the whole run.
///
/// One step rather than S1's two, because a schedule that moved on to another
/// posture part way through would be commanding a machine that had already
/// stopped taking commands -- and one that ran out would end the session at rest,
/// which is the opposite of what this run is about.
#[must_use]
pub fn steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(up_start_cycle()),
        end_ns: cycle_at(end_cycle()),
        posture: Some(PostureWire::UP),
    }]
}

/// How many missed reads in a row the decision tick tolerates before it says
/// the machine's position feedback is gone.
#[must_use]
pub fn read_loss_cycles() -> i64 {
    i64::from(default_motion_config().read_loss_ticks)
}

/// The cycle the decision tick would report the loss on, if it were still
/// running by then.
///
/// It is not: the session parks the machine and publishes a schedule nobody is
/// engaged on well before this, and a disengaged mover neither ticks nor
/// reports. Kept because the ordering is what says so -- the driver's own
/// evidence arrives first, which is why the tick's tolerance never runs out.
#[must_use]
pub fn fault_cycle() -> i64 {
    outage_cycle() + read_loss_cycles()
}

/// The cycle the driver says its own bus is gone on.
///
/// The first observer, and the only one in this run: the tick reports a control
/// loop that has lost its measurements, and the driver reports a bus that is
/// answering nothing at all. The first blind cycle counts, so the report lands
/// on the cycle the run reaches its length.
#[must_use]
pub fn bus_failure_cycle() -> i64 {
    outage_cycle() + i64::from(BLIND_CYCLES_BEFORE_BUS_FAILURE) - 1
}

/// The cycle the machine is released on.
///
/// The driver publishes its bus-failure evidence inside the cycle it declares
/// it on, the session is woken by it there -- an edge waits for no floor -- and
/// the datagram it answers with is drained by the next cycle. So the release
/// lands one cycle after the evidence, and the arithmetic is the assertion: a
/// session that answered a cycle later would be one that waited for its own
/// wake floor.
#[must_use]
pub fn release_cycle() -> i64 {
    bus_failure_cycle() + 1
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    reads_back_cycle() + ROW_COUNT as i64 + TAIL_CYCLES
}

/// The cycle the replies come back on.
#[must_use]
pub fn reads_back_cycle() -> i64 {
    outage_cycle() + i64::from(OUTAGE_CYCLES)
}
