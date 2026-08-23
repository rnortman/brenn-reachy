//! S8's input log.
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

use s8_scenario::{
    ERROR_BITS, REFUSED_SCRIPT_ID, SCRIPT_ID, START_CYCLE, fault_cycle, faulted_rows, jam_cycle,
    jam_release_cycle, jammed_rows, refused_script_cycle, refused_steps, script_sent_cycle, steps,
};

fn main() -> ExitCode {
    author::main("s8_author", s8_scenario::end_cycle(), write)
}

/// Write S8's input log into `dir`.
///
/// S1's two messages, and then the four this scenario is about: the error byte
/// the servo starts holding, the hand that jams the cranks while the head is
/// being carried down, the hand that takes it back off, and the script that
/// finds the machine parked.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.script(cycle_at(script_sent_cycle()), SCRIPT_ID, &steps())?;

    // What a servo says about itself. Nothing in the modelled machine changes
    // for it: the run is about what the loop does with a servo's own account of
    // its condition, which reaches the session over the driver's rotating read
    // and nowhere else.
    log.set_error_bits(
        cycle_at(fault_cycle()),
        JointFlagsWire::from(faulted_rows()),
        ERROR_BITS,
    )?;

    // The jam, placed inside the maneuver the error byte causes: a machine that
    // stops closing on the fold it was commanded to is what the tick raises
    // about, and the maneuver is what answers it.
    let jammed = JointFlagsWire::from(jammed_rows());
    log.obstruct(cycle_at(jam_cycle()), jammed)?;
    log.release(cycle_at(jam_release_cycle()), jammed)?;

    log.script(
        cycle_at(refused_script_cycle()),
        REFUSED_SCRIPT_ID,
        &refused_steps(),
    )?;

    log.close()
}
