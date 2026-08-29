//! S7's input log.
//!
//! Five messages: the world the run begins in, the script that raises the
//! machine, the refresh that replaces it while it is up, that refresh's own
//! number sent again, and the closing script that folds the machine. Nothing
//! touches the plant -- every replacement, the one refusal and the ending are
//! the system's own answers to what a presence sender asked of it.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s7_scenario::{
    CLOSING_SCRIPT_ID, HOLD_SCRIPT_ID, REFRESH_SCRIPT_ID, START_CYCLE, closing_cycle,
    closing_steps, duplicate_cycle, duplicate_steps, hold_steps, refresh_cycle, refresh_overlays,
    refresh_steps, script_sent_cycle,
};

fn main() -> ExitCode {
    author::main("s7_author", s7_scenario::end_cycle(), write)
}

/// Write S7's input log into `dir`.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.script(cycle_at(script_sent_cycle()), HOLD_SCRIPT_ID, &hold_steps())?;
    log.playing(
        cycle_at(refresh_cycle()),
        REFRESH_SCRIPT_ID,
        &refresh_steps(),
        &refresh_overlays(),
    )?;
    // The same number again: what a duplicated delivery looks like from the
    // session's side, which is the case the strictly-greater rule exists for.
    log.script(
        cycle_at(duplicate_cycle()),
        REFRESH_SCRIPT_ID,
        &duplicate_steps(),
    )?;
    log.script(
        cycle_at(closing_cycle()),
        CLOSING_SCRIPT_ID,
        &closing_steps(),
    )?;
    log.close()
}
