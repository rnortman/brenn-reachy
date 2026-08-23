//! S6's input log.
//!
//! Three messages: the world the run begins in, the script that asks for two
//! postures with a motion played over the first of them, and the script the
//! machine is asked for once it has let go. Nothing touches the plant -- the
//! composition, the truncation and the hand-back are all the system's own answer
//! to the first of those requests.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s6_scenario::{
    SCRIPT_ID, SECOND_SCRIPT_ID, START_CYCLE, overlays, script_sent_cycle, second_script_cycle,
    second_steps, steps,
};

fn main() -> ExitCode {
    author::main("s6_author", s6_scenario::end_cycle(), write)
}

/// Write S6's input log into `dir`.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.playing(
        cycle_at(script_sent_cycle()),
        SCRIPT_ID,
        &steps(),
        &overlays(),
    )?;
    log.script(
        cycle_at(second_script_cycle()),
        SECOND_SCRIPT_ID,
        &second_steps(),
    )?;
    log.close()
}
