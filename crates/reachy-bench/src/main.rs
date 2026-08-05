//! The bench binary's entry point: argument parsing and command dispatch.
//!
//! What a command decides lives in the library beside this file, so the
//! decisions — configuration, the registry's verdicts, the datum classification
//! — are reachable from tests that need no port and no machine. This file owns
//! only the argument shape, the port, the printing and the exit code.
//!
//! One command is dispatched: `selftest`, which is read-only. Every command that
//! moves something refuses, and keeps refusing until a self-test has run on the
//! machine and its record has been reviewed.

#![forbid(unsafe_code)]

use std::path::{Path, PathBuf};

use anyhow::{Context as _, bail};

use reachy_bench::config;
use reachy_bench::selftest::{Case, Registry, Report, now_unix};
use reachy_bus::SerialBusPort;

/// Where the configuration is read from unless `--config` says otherwise.
const DEFAULT_CONFIG: &str = "reachy-bench.toml";

/// The record file's name, written beside the configuration.
const RECORD_NAME: &str = "selftest-state.toml";

/// What the operator asked for.
#[derive(Debug)]
struct Args {
    config: PathBuf,
    record: Option<PathBuf>,
    measured_height_m: Option<f64>,
}

/// How to invoke this, for a refusal to print.
fn usage() -> String {
    format!(
        "usage: reachy-bench selftest [--config PATH] [--record PATH] \
         [--measured-height-m METRES]\n\
         \n\
         `selftest` is read-only: pings and register reads, no torque and no motion.\n\
         The measured head height is a rule reading from the base to the head; \
         without it the crank datum cannot be resolved, and the run records that \
         it was not measured.\n\
         \n\
         Configuration defaults to {DEFAULT_CONFIG}; the record is written to \
         {RECORD_NAME} beside it."
    )
}

fn main() -> anyhow::Result<()> {
    dispatch(std::env::args().skip(1))
}

fn dispatch(argv: impl Iterator<Item = String>) -> anyhow::Result<()> {
    let mut argv = argv;
    let Some(command) = argv.next() else {
        bail!("reachy-bench: no command given\n\n{}", usage());
    };

    match command.as_str() {
        "selftest" => selftest(&parse_args(argv)?),
        "arm" | "up" | "hold" | "stow" | "off" | "yaw" | "antennas" | "demo" => bail!(
            "reachy-bench: `{command}` moves the machine and is not implemented. Nothing that \
             commands a servo runs until a read-only self-test has passed on this unit and its \
             crank datum has been reviewed and written into the configuration."
        ),
        other => bail!("reachy-bench: unknown command `{other}`\n\n{}", usage()),
    }
}

/// The flags, in whatever order they were given.
fn parse_args(argv: impl Iterator<Item = String>) -> anyhow::Result<Args> {
    let mut args = Args {
        config: PathBuf::from(DEFAULT_CONFIG),
        record: None,
        measured_height_m: None,
    };
    let mut argv = argv;
    while let Some(flag) = argv.next() {
        // Every flag takes a value, so a missing one is an operator typo rather
        // than shorthand for anything.
        let value = argv
            .next()
            .with_context(|| format!("`{flag}` needs a value\n\n{}", usage()))?;
        match flag.as_str() {
            "--config" => args.config = PathBuf::from(value),
            "--record" => args.record = Some(PathBuf::from(value)),
            "--measured-height-m" => {
                let height: f64 = value
                    .parse()
                    .with_context(|| format!("`--measured-height-m {value}` is not a number"))?;
                if !height.is_finite() || height <= 0.0 {
                    bail!("`--measured-height-m {value}` is not a height above the base");
                }
                args.measured_height_m = Some(height);
            }
            other => bail!("reachy-bench: unknown option `{other}`\n\n{}", usage()),
        }
    }
    Ok(args)
}

fn record_path(args: &Args) -> PathBuf {
    args.record.clone().unwrap_or_else(|| {
        args.config
            .parent()
            .unwrap_or(Path::new(""))
            .join(RECORD_NAME)
    })
}

/// Run the read-only registry and write down what it saw.
///
/// The record is written whether the run passed or not — a failing run is
/// exactly the evidence a bring-up wants kept — and the exit code is the
/// verdict.
fn selftest(args: &Args) -> anyhow::Result<()> {
    let cfg = config::load(&args.config)?;
    let registry = Registry::from_config(&cfg, args.measured_height_m)?;

    let port = SerialBusPort::open(&cfg.bus.device, cfg.bus.baud);
    run_and_record(&registry, port, &record_path(args), now_unix())
}

/// Run the registry, print the run, write the record, and refuse if anything
/// fell short.
///
/// The record is saved before the refusal, which is the property that matters
/// on a bring-up: with no reviewed clearance
/// floor baked in yet every run fails, so the failing path is the only path
/// there is, and an early return between the run and the save would throw away
/// the whole product of a hardware round trip.
fn run_and_record<P, E>(
    registry: &Registry,
    port: Result<P, E>,
    path: &Path,
    taken_at_unix: u64,
) -> anyhow::Result<()>
where
    P: reachy_bus::BusPort,
    E: std::fmt::Display,
{
    let mut report = Report::new();
    registry.run(port, &mut report);

    print!("{report}");
    let passed = report.all_passed();
    let record = report.into_record(taken_at_unix);
    record.save(path)?;
    println!("record written to {}", path.display());

    if !passed {
        let short: Vec<String> = Case::ALL
            .iter()
            .filter(|case| !record.outcome(**case).passed())
            .map(|case| format!("{case} ({})", record.outcome(*case)))
            .collect();
        bail!(
            "reachy-bench: the self-test did not pass — {}. Nothing that moves the machine may \
             be run against this record.",
            short.join(", ")
        );
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::io;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::time::Instant;

    use reachy_bench::selftest::{Outcome, SelftestRecord};
    use reachy_bus::BusPort;

    use super::*;

    /// A port that opens and then says nothing. Every exchange reaches its
    /// deadline, so the presence case fails and the run stops there — which is
    /// the shape of a run against a machine that is not powered.
    struct SilentPort;

    impl BusPort for SilentPort {
        fn write_all(&mut self, _buf: &[u8]) -> io::Result<()> {
            Ok(())
        }

        fn read_some(&mut self, _buf: &mut [u8], _deadline: Instant) -> io::Result<usize> {
            Ok(0)
        }

        fn discard_input(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    /// A path in the system temporary directory nothing else is using.
    fn scratch_path() -> PathBuf {
        static NEXT: AtomicU32 = AtomicU32::new(0);
        std::env::temp_dir().join(format!(
            "reachy-bench-record-{}-{}.toml",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ))
    }

    /// The registry the shipped example describes, with the bus timing wound
    /// down so a run against a silent port does not spend its retry budget in
    /// real time.
    fn quick_registry() -> Registry {
        let mut cfg = reachy_bench::config::parse(include_str!("../reachy-bench.example.toml"))
            .expect("the shipped example parses");
        cfg.bus.host_allowance_ms = 1;
        cfg.bus.retry_attempts = 1;
        cfg.bus.retry_spacing_ms = 0;
        Registry::from_config(&cfg, None).expect("the configuration converts")
    }

    /// A failing run still leaves its record, and the record names what failed.
    ///
    /// This is the property the whole bring-up rests on: with no reviewed
    /// clearance floor baked in, every run fails, so the failing path is the
    /// only path there is. A record saved after the refusal instead of before
    /// it would throw away the entire product of a hardware round trip.
    #[test]
    fn a_failing_run_writes_its_record_before_it_refuses() {
        let path = scratch_path();
        let refused = run_and_record(
            &quick_registry(),
            Ok::<_, io::Error>(SilentPort),
            &path,
            1_754_000_000,
        )
        .expect_err("a silent machine does not pass");
        assert!(refused.to_string().contains("presence"), "{refused}");

        let record = SelftestRecord::load(&path).expect("the record was written and parses");
        std::fs::remove_file(&path).expect("the scratch record is removed");

        assert_eq!(record.taken_at_unix, 1_754_000_000);
        assert_eq!(record.outcome(Case::PortOpen), Outcome::Pass);
        assert_eq!(record.outcome(Case::Presence), Outcome::Fail);
        // The cases after the one that stopped the run are absent from the file
        // and read back as failures rather than as silence.
        for case in Case::ALL.iter().skip(2) {
            assert_eq!(record.outcome(*case), Outcome::NotRun, "{case}");
        }
    }

    /// A port that will not open is one of the nine cases, not something that
    /// happened before the run began — so it too leaves a record.
    #[test]
    fn a_port_that_will_not_open_still_leaves_a_record() {
        let path = scratch_path();
        let refused = run_and_record(
            &quick_registry(),
            Err::<SilentPort, _>("no such device"),
            &path,
            7,
        )
        .expect_err("a run that opened nothing does not pass");
        assert!(refused.to_string().contains("port-open"), "{refused}");

        let record = SelftestRecord::load(&path).expect("the record was written and parses");
        std::fs::remove_file(&path).expect("the scratch record is removed");

        assert_eq!(record.outcome(Case::PortOpen), Outcome::Fail);
        assert_eq!(record.cases.len(), 1, "nothing after it ran");
        for case in Case::ALL.iter().skip(1) {
            assert_eq!(record.outcome(*case), Outcome::NotRun, "{case}");
        }
        assert!(
            record.cases[0].detail.contains("no such device"),
            "{:?}",
            record.cases[0]
        );
    }

    /// Arguments as the shell would hand them over.
    fn argv(words: &[&str]) -> std::vec::IntoIter<String> {
        words
            .iter()
            .map(ToString::to_string)
            .collect::<Vec<_>>()
            .into_iter()
    }

    /// With no flags, the defaults are the file beside the working directory
    /// and no measured height — which is the run that records that nobody
    /// measured one.
    #[test]
    fn the_defaults_are_the_file_beside_the_working_directory() {
        let args = parse_args(argv(&[])).expect("no flags is a valid invocation");
        assert_eq!(args.config, PathBuf::from(DEFAULT_CONFIG));
        assert_eq!(args.measured_height_m, None);
        assert_eq!(record_path(&args), PathBuf::from(RECORD_NAME));
    }

    /// The record lands beside the configuration wherever that is, and an
    /// explicit path wins.
    #[test]
    fn the_record_lands_beside_the_configuration() {
        let args = parse_args(argv(&["--config", "/etc/bench/reachy.toml"])).expect("it parses");
        assert_eq!(
            record_path(&args),
            PathBuf::from("/etc/bench").join(RECORD_NAME)
        );

        let args = parse_args(argv(&[
            "--config",
            "/etc/bench/reachy.toml",
            "--record",
            "/tmp/r",
        ]))
        .expect("it parses");
        assert_eq!(record_path(&args), PathBuf::from("/tmp/r"));
    }

    /// A measured height is a positive number of metres, and nothing else.
    #[test]
    fn a_measured_height_is_a_positive_number_of_metres() {
        let args = parse_args(argv(&["--measured-height-m", "0.1266"])).expect("it parses");
        assert_eq!(args.measured_height_m, Some(0.1266));

        for bad in ["hand-width", "0", "-0.1", "NaN", "inf"] {
            let refused =
                parse_args(argv(&["--measured-height-m", bad])).expect_err("{bad} is not a height");
            assert!(
                refused.to_string().contains(bad),
                "the refusal repeats what was typed: {refused}"
            );
        }
    }

    /// A flag with no value, and a flag nobody defined, are both refused with
    /// the usage rather than assumed away.
    #[test]
    fn a_malformed_flag_is_refused_with_the_usage() {
        let refused = parse_args(argv(&["--config"])).expect_err("a flag needs its value");
        assert!(refused.to_string().contains("--config"), "{refused}");

        let refused = parse_args(argv(&["--verbose", "yes"])).expect_err("nobody defined that");
        assert!(refused.to_string().contains("--verbose"), "{refused}");
        assert!(refused.to_string().contains("usage:"), "{refused}");
    }

    /// Every command that moves the machine refuses, and says what has to
    /// happen before it will not.
    #[test]
    fn every_command_that_moves_the_machine_refuses() {
        for command in [
            "arm", "up", "hold", "stow", "off", "yaw", "antennas", "demo",
        ] {
            let refused = dispatch(argv(&[command])).expect_err("{command} moves the machine");
            let printed = refused.to_string();
            assert!(printed.contains(command), "{printed}");
            assert!(printed.contains("self-test"), "{printed}");
            assert!(printed.contains("crank datum"), "{printed}");
        }
    }

    /// No command, and an unknown one, are refused rather than treated as a
    /// default. A bench tool that exits zero having done nothing reads as a
    /// pass.
    #[test]
    fn no_command_and_an_unknown_command_both_refuse() {
        let refused = dispatch(argv(&[])).expect_err("nothing was asked for");
        assert!(refused.to_string().contains("usage:"), "{refused}");

        let refused = dispatch(argv(&["wiggle"])).expect_err("nobody defined that");
        assert!(refused.to_string().contains("wiggle"), "{refused}");
    }
}
