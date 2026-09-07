//! S14's input log.
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

use brenn_reachy__motion__joints_clk_rs::JointFlagsWire;
use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s14_scenario::{SCRIPT_ID, START_CYCLE, jammed_rows, obstruct_cycle, script_sent_cycle, steps};

fn main() -> ExitCode {
    author::main("s14_author", s14_scenario::end_cycle(), write)
}

/// Write S14's input log into `dir`.
///
/// S2's log with one message taken out. The hand goes on the head cranks part
/// way through the raise and there is no release: everything this run is about
/// follows from an obstruction that is still there a window later, which is the
/// evidence the doctrine answers by letting the machine go.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.script(cycle_at(script_sent_cycle()), SCRIPT_ID, &steps())?;
    log.obstruct(
        cycle_at(obstruct_cycle()),
        JointFlagsWire::from(jammed_rows()),
    )?;
    log.close()
}
