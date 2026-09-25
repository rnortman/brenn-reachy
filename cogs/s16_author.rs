//! S16's input log.
//!
//! Six messages: the world the run begins in, the opening script with the nod
//! played over the upright posture, the three replacements each sent half way
//! through a cycle while a window is still playing, and the closing script.
//! Nothing touches the plant -- every seam is the system's own answer to a
//! replacement.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s16_scenario::{
    CLOSING_SCRIPT_ID, OPENING_SCRIPT_ID, REPLACEMENT_SCRIPT_IDS, START_CYCLE, closing_cycle,
    closing_steps, opening_overlays, opening_steps, replacement_overlays, replacement_sent_ns,
    replacement_steps, script_sent_cycle,
};

fn main() -> ExitCode {
    author::main("s16_author", s16_scenario::end_cycle(), write)
}

/// Write S16's input log into `dir`.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.playing(
        cycle_at(script_sent_cycle()),
        OPENING_SCRIPT_ID,
        &opening_steps(),
        &opening_overlays(),
    )?;
    for (n, script_id) in REPLACEMENT_SCRIPT_IDS.into_iter().enumerate() {
        log.playing(
            replacement_sent_ns(n),
            script_id,
            &replacement_steps(n),
            &replacement_overlays(n),
        )?;
    }
    log.script(
        cycle_at(closing_cycle()),
        CLOSING_SCRIPT_ID,
        &closing_steps(),
    )?;
    log.close()
}
