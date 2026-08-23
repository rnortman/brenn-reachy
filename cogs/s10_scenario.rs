//! S10, a bus that never comes back: the scenario that says nothing gates
//! de-torquing.
//!
//! S4's outage, made permanent. The machine is holding its posture when the bus
//! stops answering; twenty-five blind cycles later the driver says so on its own
//! account, and the session answers the doctrine's response for a machine
//! nothing can be commanded through -- an immediate best-effort torque-off and a
//! park an operator has to clear. So far this is S4.
//!
//! What is S10's is what happens next, and it is the half S4 cannot show because
//! its bus comes back: nothing ever acknowledges the release. A read-back is a
//! round trip like any other, so the driver's confirmation pass reads nothing
//! and credits nothing, and says once that it cannot confirm the de-torquing it
//! wrote. The session's own budget then runs out and it reports that it has had
//! no acknowledgement either -- and it goes on commanding the release, every
//! wake, to the end of the run. That is the whole point: a de-torquing nobody
//! can confirm is the operator's problem and never a reason to stop asking for
//! it, and a machine that may still be holding is asked again rather than
//! written off.
//!
//! Nothing recovers, and nothing here can: the machine stays parked, the goal
//! stream stays stopped, and no script is ever taken again.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. The instants are all cycle counts from the epoch, because the
//! deterministic runner puts every sample on the grid exactly and a scenario
//! written in milliseconds would be asserting against arithmetic it did not do.

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;
use scenario::author::Step;
use scenario::{
    BLIND_CYCLES_BEFORE_BUS_FAILURE, SESSION_CONFIRM_BUDGET_NS, cycle_at, cycles_for, up_cycles,
};

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The script's number.
pub const SCRIPT_ID: u32 = 10;

/// How long the machine holds upright before the bus goes away, in cycles: the
/// move plus room to arrive and settle into its hold.
///
/// S4's number and S4's reason: a move interrupted by an outage is a different
/// question, and mixing the two would leave every assertion here with two
/// possible causes.
pub const SETTLE_CYCLES: i64 = 20;

/// How many of the session's own confirmation budgets the run carries past the
/// release.
///
/// The budget is what bounds the *reporting* of an unacknowledged release, and
/// the commanding is what this run is about, so the run has to outlast several
/// of them: one would show a session that said so once and stopped, which is
/// indistinguishable from a session that gave up.
pub const BUDGETS_AFTER_RELEASE: i64 = 4;

/// The cycle the bus stops answering on.
#[must_use]
pub fn outage_cycle() -> i64 {
    up_start_cycle() + up_cycles() + SETTLE_CYCLES
}

/// The cycle the driver says its own bus is gone on.
///
/// The first blind cycle counts, so the report lands on the cycle the run of
/// them reaches its length.
#[must_use]
pub fn bus_failure_cycle() -> i64 {
    outage_cycle() + i64::from(BLIND_CYCLES_BEFORE_BUS_FAILURE) - 1
}

/// The cycle the machine is released on.
///
/// The driver publishes its evidence inside the cycle it declares it on, the
/// session is woken by it there -- an edge waits for no floor -- and the datagram
/// it answers with is drained by the next cycle.
#[must_use]
pub fn release_cycle() -> i64 {
    bus_failure_cycle() + 1
}

/// How many cycles the session's budget for an acknowledged release runs for.
#[must_use]
pub fn confirm_budget_cycles() -> i64 {
    cycles_for(SESSION_CONFIRM_BUDGET_NS)
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    release_cycle() + BUDGETS_AFTER_RELEASE * confirm_budget_cycles()
}

/// How many cycles of replies are lost: the rest of the run, from the outage on.
///
/// The bus never comes back, which is the whole difference between this run and
/// S4's. Counted to one cycle past the end so nothing about the ending depends
/// on the outage running out exactly there.
///
/// # Panics
///
/// If the run is longer than an injection can count, which is a scenario stating
/// something it cannot ask for.
#[must_use]
pub fn outage_cycles() -> u32 {
    u32::try_from(end_cycle() - outage_cycle() + 1).expect("an outage an injection can count")
}

/// The script's one step: upright, for the whole run.
///
/// S4's shape and S4's reason: a schedule that ran out would end the session at
/// rest, which is the opposite of what this run is about.
#[must_use]
pub fn steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(up_start_cycle()),
        end_ns: cycle_at(end_cycle()),
        posture: Some(PostureWire::UP),
    }]
}
