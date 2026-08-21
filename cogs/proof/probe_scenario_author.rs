//! Phase one of the harness: write the scenario's input log.
//!
//! One argument, the directory the log is written into. The directory is the
//! whole log -- the offboard format is a directory of `.slog` files -- and the
//! deterministic runner takes that directory as its `--input-log-uri`.
//!
//! The simulated end time goes to standard output, because it is a fact about
//! the scenario rather than about the harness: the shell script that runs the
//! three phases passes it to the runner without knowing what it is, so the
//! scenario's schedule is stated in exactly one place.
//!
//! The channel is created typed, from the same generated schema the cog reads,
//! so the log carries the schema name and definition the runner's publisher
//! config was generated from. A channel created untyped would replay as a
//! silence the run could not distinguish from an empty scenario.

use std::path::{Path, PathBuf};
use std::process::ExitCode;

use clockwork_logs::offboard::{OffboardWriter, OffboardWriterConfig};
use clockwork_rs::SyncTime;

use brenn_reachy__cogs__proof__probe_msgs_clk_rs::ProbeCmdWire;
use probe_scenario::{CMD_CHANNEL, POSITIONS, command, command_time_ns};

fn main() -> ExitCode {
    let mut args = std::env::args_os().skip(1);
    let (Some(dir), None) = (args.next(), args.next()) else {
        eprintln!("usage: probe_scenario_author <input-log-dir>");
        return ExitCode::FAILURE;
    };
    let dir = PathBuf::from(dir);
    match write_input_log(&dir) {
        Ok(()) => {
            println!("{}", probe_scenario::end_time_ns());
            ExitCode::SUCCESS
        }
        Err(err) => {
            eprintln!("writing the input log under {}: {err}", dir.display());
            ExitCode::FAILURE
        }
    }
}

/// Write the scenario's input log into `dir`.
///
/// The log time and the transmit time are the same instant. They differ on a
/// real recording -- one is when the message was sent, the other when the
/// logger wrote it -- but an authored scenario has no such gap to represent,
/// and the runner schedules on the transmit time.
fn write_input_log(dir: &Path) -> Result<(), clockwork_logs::LogError> {
    let mut writer = OffboardWriter::create(dir, OffboardWriterConfig::default())?;
    let channel = writer.create_channel_typed::<ProbeCmdWire>(CMD_CHANNEL)?;
    for index in 0..POSITIONS.len() {
        let at = SyncTime::from_nanos(command_time_ns(index));
        // The sequence number is the publisher's own count, from zero. The
        // runner does not read it; the checker does, on the way back out.
        let seq = u32::try_from(index).expect("the scenario has few enough commands to count");
        writer.write_typed(channel, seq, at, at, &command(index))?;
    }
    writer.close()
}
