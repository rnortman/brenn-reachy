//! S10's input log.
//!
//! One argument, the directory the log is written into. The simulated end time
//! goes to standard output, as every scenario's author does, so the schedule
//! stays stated in one place.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s10_scenario::{SCRIPT_ID, START_CYCLE, outage_cycle, outage_cycles, script_sent_cycle, steps};

fn main() -> ExitCode {
    author::main("s10_author", s10_scenario::end_cycle(), write)
}

/// Write S10's input log into `dir`.
///
/// Three messages: the world the run begins in, the script, and the bus asked to
/// stop answering for longer than the run lasts. The servos keep holding
/// whatever they were last written -- this takes away the replies, not the torque
/// -- so the machine nobody can read is a machine that may still be energised,
/// which is what makes an unacknowledged release worth going on commanding.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.script(cycle_at(script_sent_cycle()), SCRIPT_ID, &steps())?;
    log.drop_replies(cycle_at(outage_cycle()), outage_cycles())?;
    log.close()
}
