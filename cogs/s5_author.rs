//! S5's input log.
//!
//! One argument, the directory the log is written into. The simulated end time
//! goes to standard output, as every scenario's author does, so the schedule
//! stays stated in one place.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s5_scenario::{SCRIPT_ID, START_CYCLE, script_sent_cycle, steps};

fn main() -> ExitCode {
    author::main("s5_author", s5_scenario::end_cycle(), write)
}

/// Write S5's input log into `dir`.
///
/// Two messages, and the second one carries the subject: a script whose second
/// step asks for a different posture from an instant the first step's move has
/// not finished by. Nothing else about the run differs from a healthy session,
/// which is what makes every other assertion in the checker a statement about
/// the turnaround.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.script(cycle_at(script_sent_cycle()), SCRIPT_ID, &steps())?;
    log.close()
}
