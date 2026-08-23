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

use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s1_scenario::{SCRIPT_ID, START_CYCLE, script_sent_cycle, steps};

fn main() -> ExitCode {
    author::main("s1_author", s1_scenario::end_cycle(), write)
}

/// Write S1's input log into `dir`.
///
/// Two messages: the world the run begins in, and the script. Everything else in
/// the run is the system's own answer to that script -- the survey it takes
/// before it will accept one, the arming it drives over the bus, the goals it
/// streams, and the release it ends at. The machine itself is never touched from
/// outside, which is what makes every assertion in the checker a statement about
/// the loop rather than about the scenario's hand.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.script(cycle_at(script_sent_cycle()), SCRIPT_ID, &steps())?;
    log.close()
}
