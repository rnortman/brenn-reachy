//! S3's input log.
//!
//! One argument, the directory the log is written into. The simulated end time
//! goes to standard output, as every scenario's author does, so the schedule
//! stays stated in one place.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s3_scenario::TORQUE_ON_CYCLE;

fn main() -> ExitCode {
    author::main("s3_author", s3_scenario::end_cycle(), write)
}

/// Write S3's input log into `dir`.
///
/// One message, and it is the whole scenario: the machine is energised behind
/// the session's back. That is exactly the window an arming sequencer leaves
/// open between the torque going on and the first command arriving -- and this
/// run is what happens when the command never comes, because nobody sends a
/// script and so nothing is ever engaged.
///
/// It also anchors the run at the epoch, which every other scenario's opening
/// statement about the world does: the runner starts its clock at the first
/// message the log carries.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.torque_on(cycle_at(TORQUE_ON_CYCLE), author::all_rows())?;
    log.close()
}
