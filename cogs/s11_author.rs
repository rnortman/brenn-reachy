//! S11's input log.
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

use s11_scenario::{
    ERROR_BITS, SCRIPT_ID, START_CYCLE, fault_cycle, faulted_rows, script_sent_cycle, steps,
};

fn main() -> ExitCode {
    author::main("s11_author", s11_scenario::end_cycle(), write)
}

/// Write S11's input log into `dir`.
///
/// S1's two messages and one more: the byte one antenna starts holding about
/// itself. Nothing in the modelled machine changes for it and no hand is laid on
/// the plant at all -- the pair goes limp because the *session* takes its torque
/// off, which is the whole of what this run is about, and a scenario that
/// de-torqued the antennas itself would be asserting its own hand.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.script(cycle_at(script_sent_cycle()), SCRIPT_ID, &steps())?;
    log.set_error_bits(
        cycle_at(fault_cycle()),
        JointFlagsWire::from(faulted_rows()),
        ERROR_BITS,
    )?;
    log.close()
}
