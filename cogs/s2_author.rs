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
    SCRIPT_ID, SECOND_SCRIPT_ID, START_CYCLE, jammed_rows, obstruct_cycle, release_cycle,
    script_sent_cycle, second_script_cycle, second_steps, steps,
};

fn main() -> ExitCode {
    author::main("s2_author", s2_scenario::end_cycle(), write)
}

/// Write S2's input log into `dir`.
///
/// S1's two messages, with three more after them: the hand that jams the head
/// cranks, the hand that takes it back off, and the script a machine the
/// maneuver let go of is asked to run. Everything the scenario is about follows
/// from those three.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;
    log.begin(cycle_at(START_CYCLE))?;
    log.script(cycle_at(script_sent_cycle()), SCRIPT_ID, &steps())?;

    // The jam. A jammed servo on this machine holds where it stands, which is
    // what the modelled plant does with an obstructed row: the scenario is not
    // asking what a servo does when something pushes back, it is asking what
    // the control loop does about a joint that stopped arriving.
    let jammed = JointFlagsWire::from(jammed_rows());
    log.obstruct(cycle_at(obstruct_cycle()), jammed)?;
    log.release(cycle_at(release_cycle()), jammed)?;

    // The second engagement. A rest-class response ends the session and leaves
    // the machine unlatched, so a script sent after it is taken -- which is the
    // half of the park/rest split no other run in the suite says anything about.
    log.script(
        cycle_at(second_script_cycle()),
        SECOND_SCRIPT_ID,
        &second_steps(),
    )?;

    log.close()
}
