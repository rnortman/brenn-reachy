//! The bench binary's entry point: argument parsing and command dispatch.
//!
//! What a command decides lives in the library beside this file, so the
//! decisions — configuration and the registry's verdicts — are reachable from
//! tests that need no port and no machine. This file owns only the argument
//! shape, the port, the printing and the exit code.
//!
//! `selftest` is read-only. Everything else writes to a servo, so everything
//! else is behind the standing gate: a crank datum a human wrote into the
//! configuration, checked before the port is opened.

#![forbid(unsafe_code)]

use std::path::{Path, PathBuf};

use anyhow::{Context as _, bail};

use reachy_bench::commands;
use reachy_bench::config::{self, RECORD_NAME, Resolved, resolve_for_commanding};
use reachy_bench::pump::{MonotonicClock, PumpError};
use reachy_bench::selftest::{Case, Registry, Report, now_unix};
use reachy_bus::{SerialBusPort, ServoMap};

/// Where the configuration is read from unless `--config` says otherwise.
const DEFAULT_CONFIG: &str = "reachy-bench.toml";

/// What the operator asked for.
#[derive(Debug)]
struct Args {
    config: PathBuf,
    record: Option<PathBuf>,
    /// Where every run of this invocation appends its per-period trace.
    trace: Option<PathBuf>,
    /// The words that were not flags: a yaw in degrees, or two antenna angles.
    operands: Vec<String>,
}

/// How to invoke this, for a refusal to print.
fn usage() -> String {
    format!(
        "usage: reachy-bench <command> [operands] [--config PATH] [--record PATH] \
         [--trace PATH]\n\
         \n\
         commands:\n\
         \x20 selftest              read-only: pings and register reads, no torque, no motion\n\
         \x20 provision             write the antennas' operating mode; no torque, no motion\n\
         \x20 reboot [id]           restart every servo, or one; clears a latched error and \
         drops torque\n\
         \x20 arm                   verify, pin every joint where it stands, enable torque\n\
         \x20 up                    lift the head to the neutral configuration\n\
         \x20 hold                  command nothing and measure the machine holding\n\
         \x20 stow                  move to the stow configuration; torque stays on\n\
         \x20 off                   settle, measure against stow, release torque\n\
         \x20 yaw <deg>             rotate the body\n\
         \x20 antennas <right> <left>   move the antennas, radians\n\
         \x20 demo                  up, hold, antennas, yaw, stow, off\n\
         \n\
         Every command but `selftest`, `provision`, `reboot` and `off` commissions the\n\
         machine, polls it and takes hold of it first: nothing is remembered between\n\
         invocations.\n\
         \n\
         `reboot` restarts the servos, which is how a latched hardware error — an\n\
         overload above all — is cleared without cutting power. A restart drops torque,\n\
         so the head settles: take its weight if it is up. It gates on nothing.\n\
         \n\
         `off` always releases: wherever the machine is, torque comes off and where it\n\
         was is reported. The head settles as it goes, so take its weight if it is up.\n\
         That is the way out of any session, at any moment. A move that *faults* also\n\
         releases, immediately and without measuring, and the head settles then too.\n\
         \n\
         `--trace PATH` writes one CSV row per control period — every joint's measured\n\
         angle against the goal it was being held to, which is the move's velocity\n\
         profile at the rate it was sampled. Each run of the invocation appends. Give it\n\
         a path on a memory filesystem the account this runs as can write, \
         `/var/lib/brenn-app/reachy-trace.csv` on the machine itself:\n\
         it is written once per run rather than once per period, and nothing this\n\
         produces belongs on the device's flash.\n\
         \n\
         Configuration defaults to {DEFAULT_CONFIG}; the record is written to \
         {RECORD_NAME} beside it."
    )
}

fn main() -> anyhow::Result<()> {
    dispatch(std::env::args().skip(1))
}

/// The commands this binary has.
///
/// A name becomes one of these once, at the top of the dispatch, and everything
/// downstream matches on the variant: what a command accepts and what it runs
/// are then two exhaustive matches over the same enum, so a command cannot be
/// dispatched without a shape to check its invocation against.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Command {
    Selftest,
    Provision,
    Reboot,
    Arm,
    Up,
    Hold,
    Stow,
    Off,
    Yaw,
    Antennas,
    Demo,
}

impl Command {
    /// Every command, for the tests that walk them.
    #[cfg(test)]
    const ALL: [Command; 11] = [
        Self::Selftest,
        Self::Provision,
        Self::Reboot,
        Self::Arm,
        Self::Up,
        Self::Hold,
        Self::Stow,
        Self::Off,
        Self::Yaw,
        Self::Antennas,
        Self::Demo,
    ];

    /// The command `word` names, or nothing.
    fn parse(word: &str) -> Option<Self> {
        Some(match word {
            "selftest" => Self::Selftest,
            "provision" => Self::Provision,
            "reboot" => Self::Reboot,
            "arm" => Self::Arm,
            "up" => Self::Up,
            "hold" => Self::Hold,
            "stow" => Self::Stow,
            "off" => Self::Off,
            "yaw" => Self::Yaw,
            "antennas" => Self::Antennas,
            "demo" => Self::Demo,
            _ => return None,
        })
    }

    /// The word an operator types for this command.
    fn name(self) -> &'static str {
        match self {
            Self::Selftest => "selftest",
            Self::Provision => "provision",
            Self::Reboot => "reboot",
            Self::Arm => "arm",
            Self::Up => "up",
            Self::Hold => "hold",
            Self::Stow => "stow",
            Self::Off => "off",
            Self::Yaw => "yaw",
            Self::Antennas => "antennas",
            Self::Demo => "demo",
        }
    }

    /// How many operands this command takes beyond its own name, at most.
    ///
    /// The bound is what a stray word is refused against. A command that needs
    /// every one of them says so where it reads them — `yaw` without its angle
    /// is a refusal there, while `reboot` without its servo means all of them.
    fn operands(self) -> usize {
        match self {
            Self::Selftest
            | Self::Provision
            | Self::Arm
            | Self::Up
            | Self::Hold
            | Self::Stow
            | Self::Off
            | Self::Demo => 0,
            Self::Reboot | Self::Yaw => 1,
            Self::Antennas => 2,
        }
    }
}

fn dispatch(argv: impl Iterator<Item = String>) -> anyhow::Result<()> {
    let mut argv = argv;
    let Some(word) = argv.next() else {
        bail!("reachy-bench: no command given\n\n{}", usage());
    };
    let args = parse_args(argv)?;
    let Some(command) = Command::parse(&word) else {
        bail!("reachy-bench: unknown command `{word}`\n\n{}", usage());
    };
    check_invocation(&args, command)?;
    let name = command.name();

    match command {
        Command::Selftest => selftest(&args),
        Command::Provision => provision(&args),
        Command::Reboot => reboot(&args, optional_id(&args)?),
        Command::Arm => moving(&args, name, |resolved, port, clock, line| {
            commands::arm(resolved, port, clock, line)
        }),
        Command::Up => moving(&args, name, |resolved, port, clock, line| {
            commands::up(resolved, port, clock, line)
        }),
        Command::Hold => moving(&args, name, |resolved, port, clock, line| {
            commands::hold(resolved, port, clock, line)
        }),
        Command::Stow => moving(&args, name, |resolved, port, clock, line| {
            commands::stow(resolved, port, clock, line)
        }),
        Command::Off => moving(&args, name, |resolved, port, clock, line| {
            commands::off(resolved, port, clock, line)
        }),
        Command::Yaw => {
            let degrees = one_number(&args, "yaw <deg>")?;
            moving(&args, name, move |resolved, port, clock, line| {
                commands::yaw(resolved, port, degrees, clock, line)
            })
        }
        Command::Antennas => {
            let [right, left] = two_numbers(&args, "antennas <right> <left>")?;
            moving(&args, name, move |resolved, port, clock, line| {
                commands::antennas(resolved, port, right, left, clock, line)
            })
        }
        Command::Demo => moving(&args, name, |resolved, port, clock, line| {
            commands::demo(resolved, port, clock, line)
        }),
    }
}

/// The flags and the operands, in whatever order they were given.
fn parse_args(argv: impl Iterator<Item = String>) -> anyhow::Result<Args> {
    let mut args = Args {
        config: PathBuf::from(DEFAULT_CONFIG),
        record: None,
        trace: None,
        operands: Vec::new(),
    };
    let mut argv = argv;
    while let Some(word) = argv.next() {
        if !word.starts_with("--") {
            args.operands.push(word);
            continue;
        }
        // Every flag takes a value, so a missing one is an operator typo rather
        // than shorthand for anything.
        let value = argv
            .next()
            .with_context(|| format!("`{word}` needs a value\n\n{}", usage()))?;
        match word.as_str() {
            "--config" => args.config = PathBuf::from(value),
            "--record" => args.record = Some(PathBuf::from(value)),
            "--trace" => args.trace = Some(PathBuf::from(value)),
            other => bail!("reachy-bench: unknown option `{other}`\n\n{}", usage()),
        }
    }
    Ok(args)
}

/// Refuse an invocation carrying words this command has no use for.
///
/// Unknown flags are already refused by name, and a stray word is the same kind
/// of typo: `reachy-bench stow off` would otherwise run `stow`, discard `off`,
/// and exit success with torque still on — an operator who believes the head is
/// released ends the session by cutting power.
fn check_invocation(args: &Args, command: Command) -> anyhow::Result<()> {
    let operands = command.operands();
    let name = command.name();
    if args.operands.len() > operands {
        bail!(
            "reachy-bench: `{name}` takes {operands} operand(s), {given} given\n\n{}",
            usage(),
            given = args.operands.len(),
        );
    }
    Ok(())
}

/// The one number a command like `yaw` takes.
fn one_number(args: &Args, shape: &str) -> anyhow::Result<f64> {
    let [only] = args.operands.as_slice() else {
        bail!("reachy-bench: {shape} takes one number\n\n{}", usage());
    };
    number(only)
}

/// The two numbers a command like `antennas` takes.
fn two_numbers(args: &Args, shape: &str) -> anyhow::Result<[f64; 2]> {
    let [first, second] = args.operands.as_slice() else {
        bail!("reachy-bench: {shape} takes two numbers\n\n{}", usage());
    };
    Ok([number(first)?, number(second)?])
}

/// The servo a command was pointed at, or nothing for all of them.
///
/// A servo ID is a whole number the protocol can address, so a word that is not
/// one is refused here rather than reaching the roster check as an ID nobody
/// configured — the two refusals say different things, and an operator who
/// typed `reboot leg3` needs the first one.
fn optional_id(args: &Args) -> anyhow::Result<Option<u8>> {
    let [only] = args.operands.as_slice() else {
        return Ok(None);
    };
    let id: u8 = only
        .parse()
        .with_context(|| format!("`{only}` is not a servo id\n\n{}", usage()))?;
    Ok(Some(id))
}

/// An operand as the number it has to be.
///
/// Finite, because everything downstream of a commanded angle refuses a value
/// nothing can place — and refusing it here says which word on the command line
/// was wrong.
fn number(word: &str) -> anyhow::Result<f64> {
    let value: f64 = word
        .parse()
        .with_context(|| format!("`{word}` is not a number\n\n{}", usage()))?;
    if !value.is_finite() {
        bail!(
            "reachy-bench: `{word}` is not a finite number\n\n{}",
            usage()
        );
    }
    Ok(value)
}

fn record_path(args: &Args) -> PathBuf {
    args.record
        .clone()
        .unwrap_or_else(|| config::record_path_beside(&args.config))
}

/// Run the read-only registry and write down what it saw.
///
/// The record is written whether the run passed or not — a failing run is
/// exactly the evidence a bring-up wants kept — and the exit code is the
/// verdict. Nothing that moves the machine runs against a record short of a
/// pass.
fn selftest(args: &Args) -> anyhow::Result<()> {
    let cfg = config::load(&args.config)?;
    let registry = Registry::from_config(&cfg)?;

    let port = SerialBusPort::open(&cfg.bus.device, cfg.bus.baud);
    run_and_record(&registry, port, &record_path(args), now_unix())
}

/// Run the registry, print the run, write the record, and refuse if anything
/// fell short.
///
/// The record is saved before the refusal: a failing run is the reading a
/// bring-up most wants kept, and an early return between the run and the save
/// would throw away the whole product of a hardware round trip.
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

/// Write the antennas' operating mode.
///
/// Not behind the datum gate, and deliberately: this moves nothing, so it needs
/// no envelope, no kinematics and no torque, and reads the roster and the bus
/// timing straight out of the file.
fn provision(args: &Args) -> anyhow::Result<()> {
    let cfg = config::load(&args.config)?;
    let map = ServoMap::new(cfg.servo_ids()?);
    let timing = cfg.bus_timing()?;

    println!(
        "provision over {} at {} baud. This writes one non-volatile register on the two \
         antenna servos and moves nothing.\n\
         Torque must be off on both — release it with `off`, which releases wherever \
         the machine is, and take the head's weight first.",
        cfg.bus.device, timing.baud
    );

    let port = SerialBusPort::open(&cfg.bus.device, timing.baud)
        .with_context(|| format!("opening {}", cfg.bus.device))?;

    commands::provision(&map, timing, port, &mut |line| println!("{line}"))
        .map_err(|error| anyhow::Error::new(error).context("`provision`"))
}

/// Restart the servos and report what they come back holding.
///
/// Not behind the datum gate, and deliberately: a reboot commands no angle, so
/// it needs no conversion, no envelope and no kinematics — only the roster and
/// the bus timing. It is also a de-torque, and nothing gates a de-torque.
fn reboot(args: &Args, target: Option<u8>) -> anyhow::Result<()> {
    let cfg = config::load(&args.config)?;
    let map = ServoMap::new(cfg.servo_ids()?);
    let timing = cfg.bus_timing()?;

    // The header only: what a reboot costs is said once, by the command itself,
    // where every caller of it hears the same words.
    println!("reboot over {} at {} baud.", cfg.bus.device, timing.baud);

    let port = SerialBusPort::open(&cfg.bus.device, timing.baud)
        .with_context(|| format!("opening {}", cfg.bus.device))?;
    let mut clock = MonotonicClock::new();

    commands::reboot(&map, timing, port, target, &mut clock, &mut |line| {
        println!("{line}")
    })
    .map_err(|error| anyhow::Error::new(error).context("`reboot`"))
}

/// What every command that touches a servo says before it does.
///
/// It describes the machine that ships, the way out included: an operator
/// reading this in the middle of a session is entitled to find the release that
/// works from anywhere, rather than concluding that cutting power is the only
/// way to end it.
fn preamble(command: &str, device: &str, baud: u32) -> String {
    format!(
        "{command} over {device} at {baud} baud. Every command but `selftest`, `provision`, \
         `reboot` and `off` verifies the nine servos, measures where each one is standing, \
         pins it there and enables torque — which holds it where it stands — before it moves anything; a leg \
         outside its travel window is pulled to the nearer bound.\n\
         `off` always releases: wherever the machine is, torque comes off and where it was is \
         reported. A move that faults releases too, immediately and without measuring. The head \
         settles as it goes, so take its weight if it is up. That is the way out of any session, \
         at any point — no session needs to end by cutting power."
    )
}

/// Run a command that writes to a servo: resolve the configuration, open the
/// port, and hand both to the command.
///
/// The configuration is resolved before the port is opened, so a refusal costs
/// the machine nothing. That is where the recorded crank datum is required —
/// without it every converted angle is a guess, so there is nothing to command
/// with.
fn moving<F>(args: &Args, command: &str, run: F) -> anyhow::Result<()>
where
    F: FnOnce(
        &Resolved,
        SerialBusPort,
        &mut MonotonicClock,
        &mut dyn FnMut(&str),
    ) -> Result<(), PumpError>,
{
    let cfg = config::load(&args.config)?;
    let mut resolved = resolve_for_commanding(&cfg)?;
    // The one thing about a commanding run that comes from the command line
    // rather than the file: which run an operator wants the periods of.
    resolved.trace = args.trace.clone();

    println!(
        "{}",
        preamble(command, &resolved.device, resolved.timing.baud)
    );

    let port = SerialBusPort::open(&resolved.device, resolved.timing.baud)
        .with_context(|| format!("opening {}", resolved.device))?;
    let mut clock = MonotonicClock::new();

    run(&resolved, port, &mut clock, &mut |line| println!("{line}")).map_err(|error| match error {
        // A sequence that refused has already said which phase, which servo,
        // which register and both values. Wrapping that in "the command failed"
        // would bury the only part worth reading.
        PumpError::Sequence(refusal) => anyhow::anyhow!("`{command}` stopped at {refusal}"),
        other => anyhow::Error::new(other).context(format!("`{command}`")),
    })
}

#[cfg(test)]
mod tests {
    use std::io;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::time::Instant;

    use reachy_bench::selftest::{Case, Outcome, SelftestRecord};
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
    ///
    /// The library's test fixtures wind the same three knobs down in one place,
    /// but this is the binary: it is a separate crate and cannot reach a
    /// `#[cfg(test)]` module of the library it links.
    fn quick_registry() -> Registry {
        let mut cfg = reachy_bench::config::parse(include_str!("../reachy-bench.example.toml"))
            .expect("the shipped example parses");
        cfg.bus.host_allowance_ms = 1;
        cfg.bus.retry_attempts = 1;
        cfg.bus.retry_spacing_ms = 0;
        Registry::from_config(&cfg).expect("the configuration converts")
    }

    /// A failing run still leaves its record, and the record names what failed.
    ///
    /// A record saved after the refusal instead of before it would throw away
    /// the entire product of a hardware round trip, and a run against a
    /// machine that answers nothing is exactly the one whose record is worth
    /// reading.
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

    /// A port that will not open is one of the registry's own cases, not
    /// something that happened before the run began — so it too leaves a record.
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

    /// With no flags, the defaults are the file beside the working directory.
    #[test]
    fn the_defaults_are_the_file_beside_the_working_directory() {
        let args = parse_args(argv(&[])).expect("no flags is a valid invocation");
        assert_eq!(args.config, PathBuf::from(DEFAULT_CONFIG));
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

    /// Nothing is traced unless a run was asked for it, and the flag is what
    /// asks.
    ///
    /// A trace is diagnostic output an operator wants on a particular run and
    /// on a filesystem of their choosing, which is why it is a flag and not a
    /// key in the configuration file the device carries.
    #[test]
    fn a_trace_is_written_only_where_a_run_was_told_to_write_one() {
        let args = parse_args(argv(&[])).expect("no flags is a valid invocation");
        assert_eq!(args.trace, None);

        let args = parse_args(argv(&["--trace", "/run/reachy-trace.csv"])).expect("it parses");
        assert_eq!(args.trace, Some(PathBuf::from("/run/reachy-trace.csv")));
        assert!(usage().contains("--trace"), "{}", usage());
    }

    /// `reboot` on its own means every servo, and `reboot <id>` means that one.
    ///
    /// The two are one command and not two, so the absent operand has to mean
    /// the whole roster somewhere: here, before any configuration is read.
    #[test]
    fn a_reboot_with_no_servo_named_means_every_servo() {
        let args = parse_args(argv(&[])).expect("no operand is a valid invocation");
        assert_eq!(optional_id(&args).expect("nothing to parse"), None);

        let args = parse_args(argv(&["11"])).expect("one operand is a valid invocation");
        assert_eq!(optional_id(&args).expect("11 is an id"), Some(11));
    }

    /// A word that is not a servo ID is refused as one, rather than reaching
    /// the roster check as an ID nobody configured.
    ///
    /// `256` is the case that matters: it is a number, and it is not an ID, so
    /// a refusal that only checked for digits would send it to the roster and
    /// report it as a servo this machine does not carry — which is true of
    /// every number and tells the operator nothing about what they typed.
    #[test]
    fn a_word_that_is_not_a_servo_id_is_refused_as_one() {
        for word in ["leg3", "256", "-1", "1.5", ""] {
            let args = parse_args(argv(&[word])).expect("it is an operand, whatever it says");
            let refused = optional_id(&args).expect_err("that is not a servo id");
            let printed = format!("{refused:#}");
            assert!(printed.contains("not a servo id"), "{word}: {printed}");
            assert!(printed.contains("usage:"), "{word}: {printed}");
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

    /// A command's numbers are the words that are not flags, in the order they
    /// were given, and they survive being written either side of a flag.
    #[test]
    fn the_operands_are_the_words_that_are_not_flags() {
        let args = parse_args(argv(&["1.5", "--config", "/etc/r.toml", "-2.5"]))
            .expect("operands may sit either side of a flag");
        assert_eq!(args.operands, vec!["1.5".to_string(), "-2.5".to_string()]);
        assert_eq!(
            two_numbers(&args, "antennas <right> <left>").expect("two numbers"),
            [1.5, -2.5]
        );
    }

    /// A command given the wrong number of operands, a word that is not a
    /// number, or one nothing can place, refuses before it reads any
    /// configuration — so the refusal is about the command line and says so.
    ///
    /// Each case carries the words its own refusal has to contain: every
    /// refusal this file prints ends in the usage banner, so the banner alone
    /// cannot tell an arity refusal from a parse refusal from a refusal raised
    /// somewhere the case was never meant to reach.
    #[test]
    fn a_bad_operand_is_refused_with_the_shape_the_command_takes() {
        for (words, expected) in [
            (vec!["yaw"], "takes one number"),
            (vec!["yaw", "30", "40"], "operand"),
            (vec!["yaw", "sideways"], "is not a number"),
            (vec!["yaw", "nan"], "not a finite number"),
            (vec!["antennas", "1.0"], "takes two numbers"),
            (vec!["antennas", "1.0", "inf"], "not a finite number"),
        ] {
            let refused = dispatch(argv(&words)).expect_err("that is not a valid invocation");
            let printed = refused.to_string();
            assert!(printed.contains(expected), "{words:?}: {printed}");
            assert!(printed.contains("usage:"), "{words:?}: {printed}");
        }
    }

    /// Every command answers to the word it prints, and the usage banner lists
    /// all of them.
    ///
    /// The shape check and the dispatch are two matches over one enum, so a
    /// command cannot reach the machine without a shape; this is the other half
    /// — a command the operator cannot find out about.
    #[test]
    fn every_command_answers_to_its_own_name_and_is_documented() {
        let banner = usage();
        for command in Command::ALL {
            let name = command.name();
            assert_eq!(Command::parse(name), Some(command), "{name}");
            assert!(banner.contains(&format!("\x20 {name}")), "{name}");
        }
    }

    /// A word a command has no use for is refused rather than discarded.
    ///
    /// `stow off` is the case that matters: run as `stow`, it would leave
    /// torque on and exit success, and an operator who read that as a release
    /// ends the session by cutting power with the head up.
    #[test]
    fn a_stray_operand_is_refused_rather_than_discarded() {
        for words in [
            vec!["selftest", "extra"],
            vec!["provision", "now"],
            vec!["reboot", "11", "12"],
            vec!["arm", "up"],
            vec!["up", "now"],
            vec!["hold", "10"],
            vec!["stow", "off"],
            vec!["off", "please"],
            vec!["demo", "twice"],
            vec!["yaw", "10", "20"],
            vec!["antennas", "0.5", "-0.5", "0.0"],
        ] {
            let refused = dispatch(argv(&words)).expect_err("that word means nothing here");
            let printed = refused.to_string();
            assert!(printed.contains("operand"), "{words:?}: {printed}");
            assert!(printed.contains("usage:"), "{words:?}: {printed}");
        }
    }

    /// There is no flag to authorise a release: `off` releases wherever the
    /// machine is, so an operator typing one gets it refused by name rather
    /// than silently accepted.
    #[test]
    fn there_is_no_flag_authorising_a_release() {
        for command in [
            "selftest",
            "provision",
            "reboot",
            "arm",
            "up",
            "hold",
            "stow",
            "off",
            "demo",
        ] {
            let refused =
                dispatch(argv(&[command, "--drop-head"])).expect_err("no such flag exists");
            let printed = refused.to_string();
            assert!(printed.contains("--drop-head"), "{command}: {printed}");
            assert!(printed.contains("usage:"), "{command}: {printed}");
        }
    }

    /// An unknown command is still refused as one, whatever else is on the
    /// line: there is no shape to check it against.
    #[test]
    fn an_unknown_command_is_refused_by_name_whatever_follows_it() {
        let refused = dispatch(argv(&["wiggle", "3", "--config", "/etc/r.toml"]))
            .expect_err("nobody defined that command");
        assert!(refused.to_string().contains("wiggle"), "{refused}");
    }

    /// The way out of a session is in the text an operator has in front of them
    /// when they need it.
    ///
    /// An operator holding a head up with one hand does not read the source:
    /// they read the banner the last command printed, and if it omits the
    /// release that works from anywhere, that is a release they do not know
    /// about.
    #[test]
    fn the_operator_text_names_the_release_that_works_from_anywhere() {
        for text in [usage(), preamble("stow", "/dev/ttyAMA3", 1_000_000)] {
            assert!(text.contains("`off`"), "{text}");
            assert!(
                text.contains("always releases") && text.contains("wherever the machine is"),
                "the release is named, but not that it works from anywhere: {text}"
            );
            assert!(
                text.contains("weight"),
                "the release is named, but not what it costs: {text}"
            );
            assert!(
                !text.contains("--drop-head"),
                "a flag that no longer exists: {text}"
            );
        }

        let printed = preamble("up", "/dev/ttyAMA3", 1_000_000);
        assert!(
            !printed.contains("only at stow"),
            "the banner still says stow is the only release: {printed}"
        );
    }

    /// The commands that enable torque are the ones the operator text says
    /// enable torque.
    ///
    /// Both banners say which commands arm, and an operator reads that as what
    /// a command is about to do to a machine they are standing next to. A
    /// read-only command listed among the arming ones is the same defect as a
    /// missing release: text that describes a different machine from the one
    /// that ships.
    #[test]
    fn the_operator_text_excepts_every_command_that_does_not_arm() {
        // `selftest` reads registers, `provision` writes one non-volatile
        // register, `reboot` restarts the servos and `off` releases; none of
        // the four arms.
        let excepted = "Every command but `selftest`, `provision`, `reboot` and `off`";
        for text in [usage(), preamble("up", "/dev/ttyAMA3", 1_000_000)] {
            assert!(
                text.contains(excepted),
                "a command that arms nothing is claimed to arm: {text}"
            );
        }
    }

    /// Every command that writes to a servo goes through the same gates, so a
    /// missing configuration stops each of them in the same place — before any
    /// port is opened.
    #[test]
    fn every_writing_command_is_gated_before_the_port() {
        for words in [
            vec!["provision"],
            vec!["reboot"],
            vec!["reboot", "11"],
            vec!["arm"],
            vec!["up"],
            vec!["hold"],
            vec!["stow"],
            vec!["off"],
            vec!["yaw", "10"],
            vec!["antennas", "0.5", "-0.5"],
            vec!["demo"],
        ] {
            let refused = dispatch(argv(
                &[&words[..], &["--config", "/nonexistent/reachy-bench.toml"]].concat(),
            ))
            .expect_err("there is no configuration there");
            let printed = format!("{refused:#}");
            assert!(printed.contains("configuration"), "{words:?}: {printed}");
        }
    }

    /// The shipped example, which carries no datum table, and the same file with
    /// one recorded.
    fn example_config(datum: bool) -> reachy_bench::config::BenchConfig {
        let mut cfg = reachy_bench::config::parse(include_str!("../reachy-bench.example.toml"))
            .expect("the shipped example parses");
        if datum {
            cfg.datum = Some(reachy_bench::config::DatumSection {
                crank_datum: reachy_bench::config::DatumSetting::Direct,
                provenance: "a test, not a unit".to_string(),
            });
        }
        cfg
    }

    /// Without a crank datum written into the configuration, arming refuses —
    /// before it opens anything. Every converted angle rests on it, so there is
    /// nothing to command with.
    #[test]
    fn arming_refuses_without_a_recorded_datum() {
        let refused = resolve_for_commanding(&example_config(false))
            .expect_err("the shipped example resolves no datum");
        assert!(refused.to_string().contains("datum"), "{refused}");
    }

    /// A self-test record is not consulted, present or absent, passing or not.
    /// The registry is a diagnostic and a regression guard; the arm sequence
    /// re-establishes on its own everything a record could assert.
    #[test]
    fn no_self_test_record_is_needed_to_command_the_machine() {
        // Nothing at this path, and nothing looks for it.
        let empty = scratch_path();
        assert!(!empty.exists());

        // A record that passed nothing, sitting where one would be looked for,
        // changes nothing either.
        let path = scratch_path();
        Report::new()
            .into_record(1_754_000_000)
            .save(&path)
            .expect("the scratch record is written");
        let cfg = example_config(true);
        let resolved = resolve_for_commanding(&cfg);
        std::fs::remove_file(&path).expect("the scratch record is removed");

        let resolved = resolved.expect("a resolved datum is the whole of it");
        assert_eq!(resolved.device, cfg.bus.device);
        assert_eq!(resolved.timing.baud, cfg.bus.baud);
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
