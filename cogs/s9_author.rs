//! S9's input log.
//!
//! One argument, the directory the log is written into. The simulated end time
//! goes to standard output, as every scenario's author does, so the schedule
//! stays stated in one place.

use std::path::Path;
use std::process::ExitCode;

use brenn_reachy__motion__joints_clk_rs::JointFlagsWire;
use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s9_scenario::{SCRIPT_ID, START_CYCLE, absent_rows, script_sent_cycle, steps};

fn main() -> ExitCode {
    author::main("s9_author", s9_scenario::end_cycle(), write)
}

/// Write S9's input log into `dir`.
///
/// Three messages: the world the run begins in, the servo that is not in it, and
/// the script that arrives once the survey has refused the machine. The absence
/// is stated at the epoch, before the session has said anything at all, because
/// what this run is about is a machine that was never established -- one that
/// went away mid-survey would be a bus failing rather than a machine being
/// wrong about itself.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.absent(cycle_at(START_CYCLE), JointFlagsWire::from(absent_rows()))?;
    log.script(cycle_at(script_sent_cycle()), SCRIPT_ID, &steps())?;
    log.close()
}
