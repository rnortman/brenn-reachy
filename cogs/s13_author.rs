//! S13's input log.
//!
//! Five messages: the world the run begins in and four scripts. Two of them
//! share the instant the survey's allowance runs out -- the opening script and
//! the one that arrives while the engagement it opens is still in flight, which
//! is how a script lands in `engaging` on a grid where an engagement is one
//! driver cycle. The other two arrive inside the release the first session ends
//! at, the second of them carrying the first's number.
//!
//! The plant is never touched: every hold, the refusal and both engagements are
//! the system's own answers.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s13_scenario::{
    CLOSING_SCRIPT_ID, HELD_SCRIPT_ID, OPENING_SCRIPT_ID, START_CYCLE, closing_cycle,
    closing_steps, duplicate_cycle, duplicate_steps, held_steps, opening_steps, script_sent_cycle,
};

fn main() -> ExitCode {
    author::main("s13_author", s13_scenario::end_cycle(), write)
}

/// Write S13's input log into `dir`.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.script(
        cycle_at(script_sent_cycle()),
        OPENING_SCRIPT_ID,
        &opening_steps(),
    )?;
    // The same instant as the one above, and the session's script view is sized
    // to see every message of a wake rather than the latest: the first is taken
    // and opens the engagement, and this one is screened against the phase that
    // acceptance left the machine in.
    log.script(cycle_at(script_sent_cycle()), HELD_SCRIPT_ID, &held_steps())?;
    log.script(
        cycle_at(closing_cycle()),
        CLOSING_SCRIPT_ID,
        &closing_steps(),
    )?;
    // The number already waiting, sent again: what a duplicated delivery looks
    // like to a session with a script held.
    log.script(
        cycle_at(duplicate_cycle()),
        CLOSING_SCRIPT_ID,
        &duplicate_steps(),
    )?;
    log.close()
}
