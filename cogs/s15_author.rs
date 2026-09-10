//! S15's input log.
//!
//! Two messages: the world the run begins in, and the one script -- the upright
//! posture held for the whole run, with the antenna step probe played over it at
//! full gain. Nothing is done to the plant: the antennas fall behind the
//! document's steps under their own commissioned profile, and every answer the
//! run gives to that is the system's own.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s15_scenario::{SCRIPT_ID, START_CYCLE, overlays, script_sent_cycle, steps};

fn main() -> ExitCode {
    author::main("s15_author", s15_scenario::end_cycle(), write)
}

/// Write S15's input log into `dir`.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.playing(
        cycle_at(script_sent_cycle()),
        SCRIPT_ID,
        &steps(),
        &overlays(),
    )?;
    log.close()
}
