//! S4's input log.
//!
//! One argument, the directory the log is written into. The simulated end time
//! goes to standard output, as every scenario's author does, so the schedule
//! stays stated in one place.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, ALL_ROWS, InputLog};
use scenario::cycle_at;

use s4_scenario::{ENGAGE_CYCLE, ENGAGED_EPOCH, OUTAGE_CYCLE, OUTAGE_CYCLES, steps};

fn main() -> ExitCode {
    author::main("s4_author", s4_scenario::end_time_ns(), write)
}

/// Write S4's input log into `dir`.
///
/// Three messages: the machine energised, the session engaged, and the bus
/// asked to stop answering for a while. Nothing ends the session -- what ends
/// this run is the machine's own last line of defence, which is the subject.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;

    let engage = cycle_at(ENGAGE_CYCLE);
    log.torque_on(engage, ALL_ROWS)?;
    log.schedule(engage, true, ENGAGED_EPOCH, &steps())?;

    // The outage. The servos keep holding whatever they were last written --
    // this takes away the reads, not the torque -- so the machine the loop
    // stops seeing is a machine that is still there and still energised, which
    // is exactly the case the position-feedback fault exists for.
    log.drop_replies(cycle_at(OUTAGE_CYCLE), OUTAGE_CYCLES)?;

    log.close()
}
