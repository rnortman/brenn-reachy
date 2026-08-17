//! S3's input log.
//!
//! One argument, the directory the log is written into. The simulated end time
//! goes to standard output, as every scenario's author does, so the schedule
//! stays stated in one place.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, ALL_ROWS, InputLog};
use scenario::cycle_at;

use s3_scenario::{IDLE_EPOCH, TORQUE_ON_CYCLE};

fn main() -> ExitCode {
    author::main("s3_author", s3_scenario::end_time_ns(), write)
}

/// Write S3's input log into `dir`.
///
/// Two messages, and the second one is the interesting half: a session that
/// says it is not engaged. The machine is energised anyway, which is exactly
/// the window an arming sequencer leaves open between the torque going on and
/// the first command arriving -- and this run is what happens when the command
/// never comes.
///
/// The schedule is published rather than left out so that the run distinguishes
/// a session saying "not engaged" from a channel nobody ever wrote to. They
/// mean the same thing to the decision tick, and a scenario that only covered
/// the second would say nothing about the first.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;

    let at = cycle_at(TORQUE_ON_CYCLE);
    log.torque_on(at, ALL_ROWS)?;
    log.schedule(at, false, IDLE_EPOCH, &[])?;

    log.close()
}
