//! The bench binary's entry point: argument parsing and command dispatch.
//!
//! What a command decides lives in the library beside this file, so the
//! decisions — configuration and the registry's verdicts — are reachable from
//! tests that need no port and no machine. This file owns only the argument
//! shape, the port, the printing and the exit code.
//!
//! `selftest` is read-only. Everything else writes to a servo, so everything
//! else is behind both standing gates: a self-test record in which every case
//! passed, and a crank datum a human wrote into the configuration. Neither is
//! remembered state — the record is a file and the datum is a config table —
//! and both are checked before the port is opened.

#![forbid(unsafe_code)]

use std::path::{Path, PathBuf};

use anyhow::{Context as _, bail};

use reachy_bench::commands;
use reachy_bench::config::{self, BenchConfig, Resolved};
use reachy_bench::pump::{MonotonicClock, PumpError};
use reachy_bench::selftest::{Case, Registry, Report, SelftestRecord, now_unix};
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
    /// The explicit acceptance that the head may fall, for `off` away from
    /// stow. Given per invocation and never stored.
    drop_head: bool,
    /// The words that were not flags: a yaw in degrees, or two antenna angles.
    operands: Vec<String>,
}

/// How to invoke this, for a refusal to print.
fn usage() -> String {
    format!(
        "usage: reachy-bench <command> [operands] [--config PATH] [--record PATH] \
         [--drop-head]\n\
         \n\
         commands:\n\
         \x20 selftest              read-only: pings and register reads, no torque, no motion\n\
         \x20 arm                   verify, pin every joint where it stands, enable torque\n\
         \x20 up                    lift the head to the neutral configuration\n\
         \x20 hold                  command nothing and measure the machine holding\n\
         \x20 stow                  move to the stow configuration; torque stays on\n\
         \x20 off                   verify at stow, settle, release torque\n\
         \x20 yaw <deg>             rotate the body\n\
         \x20 antennas <right> <left>   move the antennas, radians\n\
         \x20 demo                  up, hold, antennas, yaw, stow, off\n\
         \n\
         Every command but `selftest` and `off` re-drives the whole arm sequence first:\n\
         nothing is remembered between invocations. Only `off` releases torque, and it\n\
         refuses away from stow unless `--drop-head` says the head may fall.\n\
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
    const ALL: [Command; 9] = [
        Self::Selftest,
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

    /// What this command accepts beyond its own name: how many operands, and
    /// whether `--drop-head` authorises anything for it.
    fn accepts(self) -> (usize, bool) {
        match self {
            Self::Selftest | Self::Arm | Self::Up | Self::Hold | Self::Stow | Self::Demo => {
                (0, false)
            }
            Self::Off => (0, true),
            Self::Yaw => (1, false),
            Self::Antennas => (2, false),
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
        Command::Off => {
            let drop_head = args.drop_head;
            moving(&args, name, move |resolved, port, clock, line| {
                commands::off(resolved, port, drop_head, clock, line)
            })
        }
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
        drop_head: false,
        operands: Vec::new(),
    };
    let mut argv = argv;
    while let Some(word) = argv.next() {
        if !word.starts_with("--") {
            args.operands.push(word);
            continue;
        }
        // `--drop-head` is an authorisation rather than a setting, so it takes
        // no value: there is nothing to say about it but that it was given.
        if word == "--drop-head" {
            args.drop_head = true;
            continue;
        }
        // Every other flag takes a value, so a missing one is an operator typo
        // rather than shorthand for anything.
        let value = argv
            .next()
            .with_context(|| format!("`{word}` needs a value\n\n{}", usage()))?;
        match word.as_str() {
            "--config" => args.config = PathBuf::from(value),
            "--record" => args.record = Some(PathBuf::from(value)),
            other => bail!("reachy-bench: unknown option `{other}`\n\n{}", usage()),
        }
    }
    Ok(args)
}

/// Refuse an invocation carrying words or authorisations this command has no
/// use for.
///
/// Unknown flags are already refused by name, and a stray word is the same kind
/// of typo: `reachy-bench stow off` would otherwise run `stow`, discard `off`,
/// and exit success with torque still on — an operator who believes the head is
/// released ends the session by cutting power. A discarded `--drop-head` is an
/// authorisation the operator believes they gave.
fn check_invocation(args: &Args, command: Command) -> anyhow::Result<()> {
    let (operands, drop_head) = command.accepts();
    let name = command.name();
    if args.operands.len() > operands {
        bail!(
            "reachy-bench: `{name}` takes {operands} operand(s), {given} given\n\n{}",
            usage(),
            given = args.operands.len(),
        );
    }
    if args.drop_head && !drop_head {
        bail!(
            "reachy-bench: `--drop-head` authorises nothing for `{name}`; only `off` \
             releases torque, and only it can be told the head may fall\n\n{}",
            usage()
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
    let registry = Registry::from_config(&cfg)?;

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

/// Run a command that writes to a servo: check the gates, open the port, and
/// hand both to the command.
///
/// Both gates are checked before the port is opened, so a refusal costs the
/// machine nothing: the configuration must resolve — which is where the recorded
/// crank datum is required — and the self-test record beside it must be one in
/// which every case passed.
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
    let resolved = arm_gates(&cfg, &record_path(args))?;

    println!(
        "{command} over {} at {} baud. Every command but `off` verifies the nine servos, pins \
         every joint where it stands and enables torque before it moves anything; a joint \
         outside its travel window is pulled to the nearer bound.\n\
         Only `off` releases torque, and only at stow. A session ended any other way ends by \
         cutting power, and the head falls when it goes — so be ready to take its weight.",
        resolved.device, resolved.timing.baud
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

/// The two standing gates, and the configuration that survived them.
///
/// Separate from the run so the refusals are testable without a port: they are
/// the whole reason a motion command exists behind a read-only one.
fn arm_gates(cfg: &BenchConfig, record: &Path) -> anyhow::Result<Resolved> {
    let resolved = cfg.resolve()?;
    let record = SelftestRecord::load(record).with_context(|| {
        format!(
            "reading the self-test record at {}; run `reachy-bench selftest` first",
            record.display()
        )
    })?;
    record
        .admits_arm()
        .context("the self-test record does not admit arming")?;
    Ok(resolved)
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

    /// The head-may-fall flag is an authorisation, not a setting: it takes no
    /// value, and it is off unless it was given.
    #[test]
    fn the_drop_head_flag_takes_no_value() {
        let args = parse_args(argv(&["--drop-head", "--config", "/etc/r.toml"]))
            .expect("a valueless flag beside a valued one parses");
        assert!(args.drop_head);
        assert_eq!(args.config, PathBuf::from("/etc/r.toml"));
        assert!(args.operands.is_empty());

        let args = parse_args(argv(&[])).expect("no flags is a valid invocation");
        assert!(!args.drop_head, "the head falls only when it was asked to");
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

    /// `--drop-head` authorises a falling head during `off` and nothing else,
    /// so every other command refuses it instead of throwing it away.
    #[test]
    fn drop_head_is_refused_by_every_command_but_off() {
        for command in ["selftest", "arm", "up", "hold", "stow", "demo"] {
            let refused = dispatch(argv(&[command, "--drop-head"]))
                .expect_err("that flag authorises nothing here");
            let printed = refused.to_string();
            assert!(printed.contains("--drop-head"), "{command}: {printed}");
            assert!(printed.contains("usage:"), "{command}: {printed}");
        }

        // The flag is not the reason `off` stops here: it got past the shape
        // check and refused on the configuration it could not read.
        let refused = dispatch(argv(&[
            "off",
            "--drop-head",
            "--config",
            "/nonexistent/reachy-bench.toml",
        ]))
        .expect_err("there is no configuration there");
        let printed = format!("{refused:#}");
        assert!(printed.contains("configuration"), "{printed}");
    }

    /// An unknown command is still refused as one, whatever else is on the
    /// line: there is no shape to check it against.
    #[test]
    fn an_unknown_command_is_refused_by_name_whatever_follows_it() {
        let refused = dispatch(argv(&["wiggle", "3", "--drop-head"]))
            .expect_err("nobody defined that command");
        assert!(refused.to_string().contains("wiggle"), "{refused}");
    }

    /// Every command that writes to a servo goes through the same gates, so a
    /// missing configuration stops each of them in the same place — before any
    /// port is opened.
    #[test]
    fn every_writing_command_is_gated_before_the_port() {
        for words in [
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
    /// before it opens anything.
    #[test]
    fn arming_refuses_without_a_recorded_datum() {
        let refused = arm_gates(&example_config(false), Path::new("/nonexistent"))
            .expect_err("the shipped example resolves no datum");
        assert!(refused.to_string().contains("datum"), "{refused}");
    }

    /// With a datum but no self-test record, arming refuses and says which
    /// command produces one.
    #[test]
    fn arming_refuses_without_a_record() {
        let refused = arm_gates(&example_config(true), &scratch_path())
            .expect_err("there is no record at a scratch path");
        assert!(refused.to_string().contains("selftest"), "{refused}");
    }

    /// A record with a case short of a pass refuses by name. Every real record
    /// is one of these until the clearance floor has been reviewed, so this is
    /// the gate as a bring-up meets it.
    #[test]
    fn arming_refuses_on_a_record_that_did_not_pass() {
        let path = scratch_path();
        Report::new()
            .into_record(1_754_000_000)
            .save(&path)
            .expect("the scratch record is written");

        let refused =
            arm_gates(&example_config(true), &path).expect_err("an empty record passed nothing");
        std::fs::remove_file(&path).expect("the scratch record is removed");
        // The whole chain: the outer context says the record refused, and the
        // refusal itself names the case.
        let printed = format!("{refused:#}");
        assert!(printed.contains("port-open"), "{printed}");
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
