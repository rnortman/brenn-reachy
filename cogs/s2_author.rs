//! S2's input log.
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

use s2_scenario::{
    DISENGAGE_CYCLE, DISENGAGED_EPOCH, ENGAGE_CYCLE, ENGAGED_EPOCH, OBSTRUCT_CYCLE, RELEASE_CYCLE,
    jammed_rows, steps,
};

fn main() -> ExitCode {
    author::main("s2_author", s2_scenario::end_time_ns(), write)
}

/// Write S2's input log into `dir`.
///
/// S1's four messages, with two more between them: the hand that jams the head
/// cranks and the hand that takes them back off. Everything the scenario is
/// about follows from those two.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;

    let engage = cycle_at(ENGAGE_CYCLE);
    log.torque_on(engage, author::all_rows())?;
    log.schedule(engage, true, ENGAGED_EPOCH, &steps())?;

    // The jam. A jammed servo on this machine holds where it stands, which is
    // what the modelled plant does with an obstructed row: the scenario is not
    // asking what a servo does when something pushes back, it is asking what
    // the control loop does about a joint that stopped arriving.
    let jammed = JointFlagsWire::from(jammed_rows());
    log.obstruct(cycle_at(OBSTRUCT_CYCLE), jammed)?;
    log.release(cycle_at(RELEASE_CYCLE), jammed)?;

    let disengage = cycle_at(DISENGAGE_CYCLE);
    log.schedule(disengage, false, DISENGAGED_EPOCH, &[])?;
    log.torque_off(disengage, author::all_rows())?;

    log.close()
}
