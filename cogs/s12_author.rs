//! S12's input log.
//!
//! Four messages: the world the run begins in, the script that raises and folds
//! the machine, the response delay the antennas run the fold at, and the
//! replacement that arrives a second into that fold.
//!
//! The delay is laid on a machine standing upright and still, for the reason
//! `s12_scenario::LAG_LAID_BEFORE_STOW` gives. It is the whole of the
//! scenario's hand: the reversal, the answer to it and the arrival are the
//! system's own.

use std::path::Path;
use std::process::ExitCode;

use brenn_reachy__motion__joints_clk_rs::JointFlagsWire;
use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s12_scenario::{
    ANTENNA_LAG_CYCLES, OPENING_SCRIPT_ID, REVERSAL_SCRIPT_ID, START_CYCLE, lag_cycle, lagged_rows,
    opening_steps, reversal_cycle, reversal_steps, script_sent_cycle,
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
    // Both antennas, because both of them unwind three radians at the fold and
    // a run about one of them would leave the other saying the reversal is
    // instant.
    log.set_lag(
        cycle_at(lag_cycle()),
        JointFlagsWire::from(lagged_rows()),
        ANTENNA_LAG_CYCLES,
    )?;
    log.script(
        cycle_at(reversal_cycle()),
        REVERSAL_SCRIPT_ID,
        &reversal_steps(),
    )?;
    log.close()
}
