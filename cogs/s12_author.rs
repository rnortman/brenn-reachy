//! S12's input log.
//!
//! Three messages: the world the run begins in, the script that raises and
//! folds the machine, and the replacement that arrives late in that fold.
//! Nothing is done to the plant -- the antennas trail the fold under their own
//! profile -- so the reversal, the answer to it and the arrival are the
//! system's own.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s12_scenario::{
    OPENING_SCRIPT_ID, REVERSAL_SCRIPT_ID, START_CYCLE, opening_steps, reversal_cycle,
    reversal_steps, script_sent_cycle,
};

fn main() -> ExitCode {
    author::main("s12_author", s12_scenario::end_cycle(), write)
}

/// Write S12's input log into `dir`.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.script(
        cycle_at(script_sent_cycle()),
        OPENING_SCRIPT_ID,
        &opening_steps(),
    )?;
    log.script(
        cycle_at(reversal_cycle()),
        REVERSAL_SCRIPT_ID,
        &reversal_steps(),
    )?;
    log.close()
}
