//! Phase three of the harness: read the output log back and assert on it.
//!
//! One argument, the directory the deterministic runner wrote its output log
//! into. Everything asserted here is read out of that log -- nothing is taken
//! from the process's console, and nothing is timed against this machine's
//! clock.
//!
//! Three things are under test at once, and each is a design proof point rather
//! than a property of the probe:
//!
//! * that a log authored by the Rust writer drives the runner at all, which
//!   shows up here as the commands having reached the cog in the right order at
//!   the right simulated times;
//! * that a typed channel of ours round-trips through the log -- written by the
//!   framework's C++ logger, read by the Rust reader, decoded into the same
//!   generated type the scenario author wrote with, `SyncTime` and `Duration`
//!   fields included;
//! * that a `VarPacket` channel's logged payload yields the datagram the cog
//!   put in it, byte for byte.
//!
//! The reading is `log_read`'s, which is what the motion scenarios read their
//! logs with too: a break in it fails here, where the failure is about the
//! harness, rather than there, where it would look like a motion bug.

use std::path::PathBuf;
use std::process::ExitCode;

use clockwork_logs::ChannelMetadata;
use clockwork_logs::offboard::OffboardReader;

use brenn_reachy__cogs__proof__probe_msgs_clk_rs::ProbeCmd;
use clockwork__clockwork__io__var_packet_clk_rs::VarPacket__288;
use log_read::{Complaints, Datagram, Logged, binding, datagram, typed};
use probe_scenario::{
    CMD_CHANNEL, PACKET_CHANNEL, POSITIONS, PROBE_ROW, command, command_time_ns, packet_time_ns,
};
use reachy_wire::golden::to_hex as hex;
use reachy_wire::{GoalSetpoint, JOINT_COUNT};

/// Everything the run put in the log.
#[derive(Default)]
struct Run {
    /// The commands the runner replayed out of the input log.
    commands: Vec<Logged<ProbeCmd>>,
    /// The datagrams the cog published, out of their carriers.
    packets: Vec<Logged<Datagram<GoalSetpoint>>>,
    /// Every message the reader yielded, in the order it yielded them: which
    /// channel it was on and when it was published. What the ordering assertion
    /// reads, and the only thing that needs the two streams merged.
    order: Vec<(String, i64)>,
    /// What could not be read, which is a failure of the run.
    complaints: Complaints,
}

/// A failure of the run, as the checker reports it: a line naming what was
/// expected and what the log held instead.
type Failures = Vec<String>;

fn main() -> ExitCode {
    let mut args = std::env::args_os().skip(1);
    let (Some(dir), None) = (args.next(), args.next()) else {
        eprintln!("usage: probe_scenario_checker <output-log-dir>");
        return ExitCode::FAILURE;
    };
    let dir = PathBuf::from(dir);

    let failures = match check(&dir) {
        Ok(failures) => failures,
        Err(err) => {
            eprintln!("reading the output log under {}: {err}", dir.display());
            return ExitCode::FAILURE;
        }
    };
    if failures.is_empty() {
        return ExitCode::SUCCESS;
    }
    for failure in &failures {
        eprintln!("probe_scenario_checker: {failure}");
    }
    ExitCode::FAILURE
}

fn check(dir: &std::path::Path) -> Result<Failures, clockwork_logs::LogError> {
    let mut reader = OffboardReader::open(dir)?;
    let mut run = Run::default();

    // Before a single message is decoded: `to_message` is size-checked only, so
    // what makes the decodes below trustworthy is the log's own record of each
    // channel's schema agreeing with the type it is read as.
    let channels: Vec<ChannelMetadata> = reader.channels().to_vec();
    check_channel_types(&channels, &mut run.complaints);

    while let Some(message) = reader.read_next()? {
        run.order.push((
            message.metadata.channel_name.clone(),
            message.message_time.as_nanos(),
        ));
        match message.metadata.channel_name.as_str() {
            CMD_CHANNEL => typed(&message, &mut run.commands, &mut run.complaints),
            PACKET_CHANNEL => datagram::<VarPacket__288, _>(
                &message,
                GoalSetpoint::decode,
                &mut run.packets,
                &mut run.complaints,
            ),
            other => run
                .complaints
                .push(format!("no type is bound to channel {other}")),
        }
    }
    if !reader.error_counters().is_clean() {
        run.complaints.push(format!(
            "the reader recorded errors over the output log: {:?}",
            reader.error_counters()
        ));
    }

    let mut failures = run.complaints.clone();
    check_commands(&run, &mut failures);
    check_packets(&run, &mut failures);
    check_interleaving(&run, &mut failures);
    Ok(failures)
}

/// Both channels are in the log, and each carries the schema the checker is
/// about to decode it as.
fn check_channel_types(channels: &[ChannelMetadata], complaints: &mut Complaints) {
    binding::<ProbeCmd>(channels, CMD_CHANNEL, complaints);
    binding::<VarPacket__288>(channels, PACKET_CHANNEL, complaints);
}

/// The commands the runner published, read back as the type they were authored
/// as. The input log's own messages reach the output log because the system
/// logs that channel too, so this leg asserts the whole round trip: authored in
/// Rust, replayed by the runner, logged by the framework, decoded in Rust.
fn check_commands(run: &Run, failures: &mut Failures) {
    if run.commands.len() != POSITIONS.len() {
        failures.push(format!(
            "expected {} messages on {CMD_CHANNEL}, found {}",
            POSITIONS.len(),
            run.commands.len()
        ));
        return;
    }
    for (index, received) in run.commands.iter().enumerate() {
        let expected = command(index);
        if received.message != expected {
            failures.push(format!(
                "command {index} on {CMD_CHANNEL} decoded as {:?}, expected {expected:?}",
                received.message
            ));
        }
        let at = command_time_ns(index);
        if received.at_ns != at {
            failures.push(format!(
                "command {index} on {CMD_CHANNEL} was replayed at {}, expected {at}",
                received.at_ns
            ));
        }
    }
}

/// The datagrams the cog emitted, recovered from the logged `VarPacket`
/// payloads. One per command, in order, carrying that command's instant and
/// position.
fn check_packets(run: &Run, failures: &mut Failures) {
    if run.packets.len() != POSITIONS.len() {
        failures.push(format!(
            "expected {} messages on {PACKET_CHANNEL}, found {}",
            POSITIONS.len(),
            run.packets.len()
        ));
        return;
    }
    for (index, received) in run.packets.iter().enumerate() {
        let datagram = &received.message.bytes;
        let mut expected_targets = [0.0; JOINT_COUNT];
        expected_targets[PROBE_ROW] = POSITIONS[index];
        let expected = GoalSetpoint {
            execute_at_ns: command_time_ns(index),
            mask: 1 << PROBE_ROW,
            targets: expected_targets,
        };
        // The datagram is compared as bytes rather than field by field: the
        // carrier is meant to be transparent, so what is under test is that
        // exactly the bytes the cog encoded came back, and encoding the
        // expectation is how that is stated.
        let expected_seq = u32::try_from(index).expect("the scenario has few enough commands");
        let expected_bytes = expected.encode(expected_seq);
        if *datagram != expected_bytes {
            failures.push(format!(
                "packet {index} on {PACKET_CHANNEL} is {}, expected {} (decoded: {:?})",
                hex(datagram),
                hex(&expected_bytes),
                received.message.message
            ));
        }
    }
}

/// The runner's ordering, as the reader presents it: every datagram lands at
/// the instant of the command that caused it, and the log's transmit times do
/// not go backwards. The motion scenarios' ordering assertions all read through
/// this same merge, so a merge that reordered anything would show here first.
fn check_interleaving(run: &Run, failures: &mut Failures) {
    let mut previous = i64::MIN;
    for (channel, at_ns) in &run.order {
        if *at_ns < previous {
            failures.push(format!(
                "the reader yielded {channel} at {at_ns} after {previous}: transmit order is not \
                 monotonic"
            ));
        }
        previous = *at_ns;
    }
    for (index, received) in run.packets.iter().enumerate() {
        let at = packet_time_ns(index);
        if received.at_ns != at {
            failures.push(format!(
                "packet {index} was published at {}, expected {at} -- its command's instant \
                 plus the cog's declared execution duration",
                received.at_ns
            ));
        }
        if received.sequence_number != u32::try_from(index).unwrap_or(u32::MAX) {
            failures.push(format!(
                "packet {index} carries publisher sequence {}, expected {index}",
                received.sequence_number
            ));
        }
    }
}
