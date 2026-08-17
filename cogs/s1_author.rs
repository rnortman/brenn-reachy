//! S1's input log.
//!
//! One argument, the directory the log is written into. The directory is the
//! whole log -- the offboard format is a directory of `.slog` files -- and the
//! deterministic runner takes that directory as its `--input-log-uri`.
//!
//! The simulated end time goes to standard output, because it is a fact about
//! the scenario rather than about the harness: the shell script that runs the
//! three phases passes it to the runner without knowing what it is, so the
//! scenario's schedule is stated in exactly one place.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, ALL_ROWS, InputLog};
use scenario::cycle_at;

use s1_scenario::{DISENGAGE_CYCLE, DISENGAGED_EPOCH, ENGAGE_CYCLE, ENGAGED_EPOCH, steps};

fn main() -> ExitCode {
    author::main("s1_author", s1_scenario::end_time_ns(), write)
}

/// Write S1's input log into `dir`.
///
/// Four messages: the machine energised and the session engaged at the start,
/// and the session ended and the machine de-energised at the finish. Everything
/// between them is what the loop does about it.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;

    // The arming sequencer's job, done by the scenario: the machine is
    // energised before anything commands it. A schedule that engaged a cold
    // machine would have the decision tick commanding servos that answer
    // nothing, which is a different scenario.
    let engage = cycle_at(ENGAGE_CYCLE);
    log.torque_on(engage, ALL_ROWS)?;
    log.schedule(engage, true, ENGAGED_EPOCH, &steps())?;

    // The session ends. Disengaging stops the goal stream, so the de-energising
    // has to come with it: the alternative is a machine holding torque with
    // nobody feeding the dead-man, which is the driver's job to end and not
    // this scenario's subject.
    let disengage = cycle_at(DISENGAGE_CYCLE);
    log.schedule(disengage, false, DISENGAGED_EPOCH, &[])?;
    log.torque_off(disengage, ALL_ROWS)?;

    log.close()
}
