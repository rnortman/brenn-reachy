//! S5's input log.
//!
//! One argument, the directory the log is written into. The simulated end time
//! goes to standard output, as every scenario's author does, so the schedule
//! stays stated in one place.

use std::path::Path;
use std::process::ExitCode;

use scenario::author::{self, InputLog};
use scenario::cycle_at;

use s5_scenario::{
    DISENGAGE_CYCLE, DISENGAGED_EPOCH, ENGAGE_CYCLE, ENGAGED_EPOCH, RETARGET_CYCLE, RETARGET_EPOCH,
    stow_steps, up_steps,
};

fn main() -> ExitCode {
    author::main("s5_author", s5_scenario::end_time_ns(), write)
}

/// Write S5's input log into `dir`.
///
/// Five messages, and the middle one is the subject: a second schedule, under a
/// second epoch, asking for a different posture while the machine is still on
/// its way to the first. Nothing else about the run differs from a healthy
/// session, which is what makes every other assertion in the checker a
/// statement about the retarget.
fn write(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut log = InputLog::create(dir)?;

    let engage = cycle_at(ENGAGE_CYCLE);
    log.torque_on(engage, author::all_rows())?;
    log.schedule(engage, true, ENGAGED_EPOCH, &up_steps())?;

    // The session changes its mind. The machine stays engaged and stays
    // energised across it: a retarget is a new command inside one session, and
    // a run that de-energised in the middle would be asserting about an arming
    // path instead.
    log.schedule(
        cycle_at(RETARGET_CYCLE),
        true,
        RETARGET_EPOCH,
        &stow_steps(),
    )?;

    let disengage = cycle_at(DISENGAGE_CYCLE);
    log.schedule(disengage, false, DISENGAGED_EPOCH, &[])?;
    log.torque_off(disengage, author::all_rows())?;

    log.close()
}
