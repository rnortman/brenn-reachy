//! S4's input log.
//!
//! One argument, the directory the log is written into. The simulated end time
//! goes to standard output, as every scenario's author does, so the schedule
//! stays stated in one place.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s4_scenario::{OUTAGE_CYCLES, SCRIPT_ID, START_CYCLE, outage_cycle, script_sent_cycle, steps};

fn main() -> ExitCode {
    author::main("s4_author", s4_scenario::end_cycle(), write)
}

/// Write S4's input log into `dir`.
///
/// Three messages: the world the run begins in, the script, and the bus asked to
/// stop answering for a while. Nothing ends the session -- what ends this run is
/// the driver's own evidence and the session's answer to it, which is the
/// subject.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.script(cycle_at(script_sent_cycle()), SCRIPT_ID, &steps())?;

    // The outage. The servos keep holding whatever they were last written --
    // this takes away the replies, not the torque -- so the machine the loop
    // stops seeing is a machine that is still there and still energised, which
    // is exactly the case the session's park exists for.
    log.drop_replies(cycle_at(outage_cycle()), OUTAGE_CYCLES)?;

    log.close()
}
