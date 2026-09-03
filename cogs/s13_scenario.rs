//! S13, a wake at the wrong moments: the scenario that says a script arriving
//! mid-maneuver is the next command rather than an error.
//!
//! A wake word can land at any instant, and three of them here land while a
//! maneuver is under way: one while the engagement it opened is
//! still in flight, one while the machine is being let go of at the end of the
//! schedule, and one that is a duplicate of a script already waiting. What the
//! run says is that each of the first two is *held* -- answered on the phase the
//! maneuver ends in -- and that the duplicate is refused for its number and
//! nothing else.
//!
//! The two holds are drained the two ways there are. The one held while the
//! engagement is in flight becomes a replacement the wake the machine goes
//! active, so the schedule the session runs is the held script's and no torque
//! is touched for it. The one held through the release becomes an acceptance the
//! wake the machine reaches rest, which opens a second engagement on that same
//! wake -- after the release's last write, which is the ordering that matters:
//! torque comes back on only once it has fully come off.
//!
//! Nothing is done to the plant. Every hold, the one refusal and both
//! engagements are the system's own answer to four messages, so every assertion
//! in the checker is a statement about the session rather than about the
//! scenario's hand.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. Every instant is a cycle count from the epoch.

use scenario::author::Step;
use scenario::{cycle_at, release_allowance_cycles, run_end_cycle};

use brenn_reachy__cogs__schedule_clk_rs::PostureWire;

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The number of the script that opens the engagement.
pub const OPENING_SCRIPT_ID: u32 = 30;

/// The number of the script that arrives while that engagement is in flight.
///
/// Strictly greater than the one before it, which is what the ordering screen a
/// hold is put through demands.
pub const HELD_SCRIPT_ID: u32 = 31;

/// The number of the script that arrives while the machine is being let go of.
pub const CLOSING_SCRIPT_ID: u32 = 32;

/// How long the opening script asks for the machine to be up, in cycles.
///
/// Never run: the script held beside it replaces it the wake the machine goes
/// active, so what this span buys the run is that the opening script is
/// well-formed and would have held the machine had nothing superseded it.
pub const OPENING_CYCLES: i64 = 100;

/// How long the held script's own upright step lasts, in cycles.
pub const UP_CYCLES: i64 = 60;

/// How long its fold lasts, in cycles: the longer move plus room to arrive and
/// hold.
pub const STOW_CYCLES: i64 = 150;

/// How many cycles into the release the closing script arrives.
///
/// Well inside it: the release is a settle under held torque and then torque
/// written off one servo at a time with each write read back, and what this
/// scenario is about is a script arriving while that is under way rather than
/// one racing its last write.
pub const CLOSING_AFTER_RELEASE: i64 = 20;

/// How many cycles after the closing script its own number is sent again.
///
/// Still inside the release, so the duplicate arrives at a session with the
/// first one held: what it is refused for is its number, which is the ordering
/// screen a hold applies to the script already waiting.
pub const DUPLICATE_AFTER_CLOSING: i64 = 20;

/// How long the second session holds the machine up, in cycles.
pub const SECOND_UP_CYCLES: i64 = 60;

/// The cycle the held script's fold begins on.
#[must_use]
pub fn stow_start_cycle() -> i64 {
    up_start_cycle() + UP_CYCLES
}

/// The cycle the first schedule runs out on, which is what ends the first
/// session.
#[must_use]
pub fn disengage_cycle() -> i64 {
    stow_start_cycle() + STOW_CYCLES
}

/// The cycle the closing script is sent on: inside the release the first
/// session ends at.
#[must_use]
pub fn closing_cycle() -> i64 {
    disengage_cycle() + CLOSING_AFTER_RELEASE
}

/// The cycle the closing script's own number is sent again on.
#[must_use]
pub fn duplicate_cycle() -> i64 {
    closing_cycle() + DUPLICATE_AFTER_CLOSING
}

/// The cycle the second session's upright step begins on.
///
/// A whole release allowance past the schedule that ended the first session, so
/// the step opens on a cycle this scenario named rather than on whichever cycle
/// the held script happened to be drained on -- which is what makes every
/// instant placed after it exact. What the machine does between the drain and
/// this cycle is nothing: it is engaged and holding where it stands, and no step
/// covers those instants yet.
#[must_use]
pub fn second_up_start_cycle() -> i64 {
    disengage_cycle() + release_allowance_cycles()
}

/// The cycle the second session's fold begins on.
#[must_use]
pub fn second_stow_start_cycle() -> i64 {
    second_up_start_cycle() + SECOND_UP_CYCLES
}

/// The cycle the second schedule runs out on, which is what ends the second
/// session.
#[must_use]
pub fn second_disengage_cycle() -> i64 {
    second_stow_start_cycle() + STOW_CYCLES
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(second_disengage_cycle())
}

/// The one step of the opening script: up, and bounded.
#[must_use]
pub fn opening_steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(up_start_cycle()),
        end_ns: cycle_at(up_start_cycle() + OPENING_CYCLES),
        posture: Some(PostureWire::UP),
    }]
}

/// The two steps of the script held through the engagement: up, and then the
/// fold that ends the session.
#[must_use]
pub fn held_steps() -> [Step; 2] {
    [
        Step {
            start_ns: cycle_at(up_start_cycle()),
            end_ns: cycle_at(stow_start_cycle()),
            posture: Some(PostureWire::UP),
        },
        Step {
            start_ns: cycle_at(stow_start_cycle()),
            end_ns: cycle_at(disengage_cycle()),
            posture: Some(PostureWire::STOW),
        },
    ]
}

/// The two steps of the script held through the release: the second session's
/// whole schedule, which starts once the machine has been engaged again.
#[must_use]
pub fn closing_steps() -> [Step; 2] {
    [
        Step {
            start_ns: cycle_at(second_up_start_cycle()),
            end_ns: cycle_at(second_stow_start_cycle()),
            posture: Some(PostureWire::UP),
        },
        Step {
            start_ns: cycle_at(second_stow_start_cycle()),
            end_ns: cycle_at(second_disengage_cycle()),
            posture: Some(PostureWire::STOW),
        },
    ]
}

/// The steps of the duplicate: the closing script's own, sent again.
///
/// Never read. The ordering screen answers on the number the script carries,
/// before anything in it is looked at, so what these say is beside the point --
/// and they are the well-formed schedule the first copy carried so that the
/// refusal cannot be the times being wrong.
#[must_use]
pub fn duplicate_steps() -> [Step; 2] {
    closing_steps()
}
