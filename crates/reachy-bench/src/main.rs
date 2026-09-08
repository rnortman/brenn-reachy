//! The bench binary's entry point: argument parsing and command dispatch.
//!
//! What a command decides lives in the library beside this file, so the
//! decisions — configuration and the registry's verdicts — are reachable from
//! tests that need no port and no machine. This file owns only the argument
//! shape, the port, the printing and the exit code.
//!
//! `selftest` is read-only. The other five write to a servo, and none of them
//! commands an angle: `provision` writes one non-volatile register on a limp
//! machine, `reboot` and `off` are de-torques, which nothing gates, `watchdog`
//! torques one servo at the position it is already standing at and watches what
//! its bus watchdog does to it — a stop, with torque held — and `hold-probe`
//! torques one servo at that same position and reads it as fast as the bus
//! answers, to see whether a joint told to stand still does.

#![forbid(unsafe_code)]

use std::io::Write as _;
use std::path::{Path, PathBuf};

use anyhow::{Context as _, bail};

use reachy_bench::bare::{
    self, BareError, HOLD_PROBE_SECONDS, HOLD_PROBE_SERIES_PREFIX, MonotonicClock, ProbeRequest,
    ProbeRun,
};
use reachy_bench::config::{self, RECORD_NAME};
use reachy_bench::selftest::{Case, Registry, Report, now_unix};
use reachy_bus::{SerialBusPort, ServoMap};
use reachy_motion::Gains;

/// Where the configuration is read from unless `--config` says otherwise.
const DEFAULT_CONFIG: &str = "reachy-bench.toml";

/// The flags, named once so the parser, the per-command contract and the
/// refusals all say the same word.
const CONFIG_FLAG: &str = "--config";
/// See [`CONFIG_FLAG`].
const RECORD_FLAG: &str = "--record";
/// See [`CONFIG_FLAG`].
const GAINS_FLAG: &str = "--gains";
/// See [`CONFIG_FLAG`].
const SECONDS_FLAG: &str = "--seconds";

/// What the operator asked for.
#[derive(Debug)]
struct Args {
    config: PathBuf,
    record: Option<PathBuf>,
    /// The position gains a hold probe swaps in for its run, if the operator
    /// named a triple.
    gains: Option<Gains>,
    /// How long each of a hold probe's phases runs, if the operator named a
    /// figure.
    seconds: Option<u64>,
    /// The words that were not flags: the one servo a reboot or a watchdog
    /// self-test addresses.
    operands: Vec<String>,
    /// The flags that were given, by name and in the order they were typed.
    ///
    /// Kept because a flag only some commands read is a flag the rest have to
    /// refuse: a run that silently ignored `--gains` is an operator who
    /// believes gains were swapped when nothing was written.
    given: Vec<&'static str>,
}

/// How to invoke this, for a refusal to print.
fn usage() -> String {
    format!(
        "usage: reachy-bench <command> [operands] [--config PATH] [--record PATH] \
         [--gains P,I,D] [--seconds N]\n\
         \n\
         commands:\n\
         \x20 selftest              read-only: pings and register reads, no torque, no motion\n\
         \x20 provision             write the antennas' operating mode; no torque, no motion\n\
         \x20 reboot [id]           restart every servo, or one; clears a latched error and \
         drops torque\n\
         \x20 off                   write torque off on every servo\n\
         \x20 watchdog [id]         torque one servo and watch what its bus watchdog does to \
         it\n\
         \x20 hold-probe [id]       torque one servo and read it at bus speed to see whether \
         it holds still\n\
         \n\
         Nothing here commands an angle: this tool reads the machine, provisions it and \
         releases it.\n\
         The one goal `watchdog` writes is the count the servo reports for itself, which \
         moves it nowhere.\n\
         Coordinated motion is the cog path's, and there is no command for it here.\n\
         \n\
         `reboot` restarts the servos, which is how a latched hardware error — an\n\
         overload above all — is cleared without cutting power. A restart drops torque,\n\
         so the head settles: take its weight if it is up. It gates on nothing.\n\
         \n\
         `off` always releases: wherever the machine is, torque comes off. Every servo on\n\
         the roster is asked whatever the ones before it answered. The head settles as it\n\
         goes, so take its weight if it is up. That is the way out of any session, at any\n\
         moment.\n\
         \n\
         `watchdog` is a supervised bring-up assertion, not a routine command: it arms the\n\
         servos' own bus inactivity timeout on one servo — an antenna unless you name\n\
         another — holds torque there, and then goes quiet and reads what the trip did.\n\
         It runs for a few seconds. On this hardware the trip stops the servo and leaves it\n\
         holding torque, so the assertion that it releases fails: that failure is the\n\
         standing record of an unresolved policy defect and is not to be made green. The\n\
         command's own make-safe disarms the register and releases torque on the way out,\n\
         whichever way it ends. Run it at rest, never with the head up. No power cycle is\n\
         needed before the next read-only sweep: the sweep accepts both readings a machine\n\
         at rest has, the provisioned zero and the value a session arms. Check that the\n\
         disarm took, though — this command arms the same value a session does, so one\n\
         servo left armed among eight at zero splits the roster across the two and the\n\
         sweep fails on the mix, naming the servo.\n\
         \n\
         `hold-probe` is the other supervised assertion: it holds one servo — an antenna\n\
         unless you name another — at the count it reports for itself and reads that count as\n\
         fast as the bus answers, for {HOLD_PROBE_SECONDS} s with the goal rewritten at the\n\
         driver's cadence and {HOLD_PROBE_SECONDS} s with reads alone. `--gains P,I,D` swaps\n\
         the position loop's terms for the run and puts the kept ones back on the way out;\n\
         `--seconds N` sets each phase's length. It commands no angle and holds torque for both\n\
         phases: run it at rest, never with the head up. The two series are written beside the\n\
         record as hold-probe-<stamp>-<id>.csv, whether the run passed or not.\n\
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
    Off,
    Watchdog,
    HoldProbe,
}

impl Command {
    /// Every command, for the tests that walk them.
    #[cfg(test)]
    const ALL: [Command; 6] = [
        Self::Selftest,
        Self::Provision,
        Self::Reboot,
        Self::Off,
        Self::Watchdog,
        Self::HoldProbe,
    ];

    /// The command `word` names, or nothing.
    fn parse(word: &str) -> Option<Self> {
        Some(match word {
            "selftest" => Self::Selftest,
            "provision" => Self::Provision,
            "reboot" => Self::Reboot,
            "off" => Self::Off,
            "watchdog" => Self::Watchdog,
            "hold-probe" => Self::HoldProbe,
            _ => return None,
        })
    }

    /// The word an operator types for this command.
    fn name(self) -> &'static str {
        match self {
            Self::Selftest => "selftest",
            Self::Provision => "provision",
            Self::Reboot => "reboot",
            Self::Off => "off",
            Self::Watchdog => "watchdog",
            Self::HoldProbe => "hold-probe",
        }
    }

    /// How many operands this command takes beyond its own name, at most.
    ///
    /// The bound is what a stray word is refused against. A command that needs
    /// every one of them says so where it reads them — `reboot` without its
    /// servo means all of them.
    fn operands(self) -> usize {
        match self {
            Self::Selftest | Self::Provision | Self::Off => 0,
            Self::Reboot | Self::Watchdog | Self::HoldProbe => 1,
        }
    }

    /// The flags this command reads, beyond the one every command reads.
    ///
    /// The other half of the invocation contract `operands` is: a flag a
    /// command does not read is refused rather than dropped, because a dropped
    /// one is an operator told nothing while the machine did something else.
    /// `--config` is every command's: all six load the bus out of it.
    fn flags(self) -> &'static [&'static str] {
        match self {
            Self::Selftest => &[RECORD_FLAG],
            Self::Provision | Self::Reboot | Self::Off | Self::Watchdog => &[],
            Self::HoldProbe => &[RECORD_FLAG, GAINS_FLAG, SECONDS_FLAG],
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

    match command {
        Command::Selftest => selftest(&args),
        Command::Provision => provision(&args),
        Command::Reboot => reboot(&args, optional_id(&args)?),
        Command::Off => off(&args),
        Command::Watchdog => watchdog(&args, optional_id(&args)?),
        Command::HoldProbe => hold_probe(&args, optional_id(&args)?),
    }
}

/// The flags and the operands, in whatever order they were given.
fn parse_args(argv: impl Iterator<Item = String>) -> anyhow::Result<Args> {
    let mut args = Args {
        config: PathBuf::from(DEFAULT_CONFIG),
        record: None,
        gains: None,
        seconds: None,
        operands: Vec::new(),
        given: Vec::new(),
    };
    let mut argv = argv;
    while let Some(word) = argv.next() {
        if !word.starts_with("--") {
            args.operands.push(word);
            continue;
        }
        // The name is matched before its value is taken, so a flag nobody
        // defined is refused by name whether or not a word follows it: an
        // option that used to exist is what an operator most often types, and
        // "needs a value" would send them looking for the value instead of for
        // the flag. Every flag this program does define takes a value, so a
        // missing one is an operator typo rather than shorthand for anything.
        match word.as_str() {
            "--config" => {
                args.config = PathBuf::from(value_for(&word, &mut argv)?);
                args.given.push(CONFIG_FLAG);
            }
            "--record" => {
                args.record = Some(PathBuf::from(value_for(&word, &mut argv)?));
                args.given.push(RECORD_FLAG);
            }
            "--gains" => {
                args.gains = Some(parse_gains(&value_for(&word, &mut argv)?)?);
                args.given.push(GAINS_FLAG);
            }
            "--seconds" => {
                args.seconds = Some(parse_seconds(&value_for(&word, &mut argv)?)?);
                args.given.push(SECONDS_FLAG);
            }
            other => bail!("reachy-bench: unknown option `{other}`\n\n{}", usage()),
        }
    }
    Ok(args)
}

/// The three position gains a `--gains` flag carries, as `P,I,D`.
///
/// All three or none: the register is one six-byte span written in one
/// transaction, and a flag that set the proportional term alone would be
/// writing the other two anyway — from whatever the operator did not say.
fn parse_gains(word: &str) -> anyhow::Result<Gains> {
    let terms: Vec<&str> = word.split(',').collect();
    let [p, i, d] = terms.as_slice() else {
        bail!(
            "`--gains` takes three terms, `P,I,D`; got `{word}`\n\n{}",
            usage()
        );
    };
    let term = |text: &str, which: &str| -> anyhow::Result<u16> {
        text.trim()
            .parse()
            .with_context(|| format!("`--gains`: `{text}` is not a {which} gain\n\n{}", usage()))
    };
    Ok(Gains {
        p: term(p, "proportional")?,
        i: term(i, "integral")?,
        d: term(d, "derivative")?,
    })
}

/// The seconds a `--seconds` flag carries. Zero is refused here: a phase of no
/// length is a phase that measures nothing, and the command would print a rate
/// over no readings rather than say so.
fn parse_seconds(word: &str) -> anyhow::Result<u64> {
    let seconds: u64 = word.parse().with_context(|| {
        format!(
            "`--seconds`: `{word}` is not a number of seconds\n\n{}",
            usage()
        )
    })?;
    if seconds == 0 {
        bail!("`--seconds` must be at least one second\n\n{}", usage());
    }
    Ok(seconds)
}

/// The word after a flag, or a refusal naming the flag that wanted it.
fn value_for(flag: &str, argv: &mut impl Iterator<Item = String>) -> anyhow::Result<String> {
    argv.next()
        .with_context(|| format!("`{flag}` needs a value\n\n{}", usage()))
}

/// Refuse an invocation carrying words this command has no use for.
///
/// Unknown flags are already refused by name, and a stray word is the same kind
/// of typo: `reachy-bench selftest off` would otherwise run the registry,
/// discard `off`, and exit success with torque wherever it was — an operator who
/// believes the head is released ends the session by cutting power.
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
    let taken = command.flags();
    for flag in &args.given {
        if *flag != CONFIG_FLAG && !taken.contains(flag) {
            bail!(
                "reachy-bench: `{name}` does not take `{flag}`\n\n{}",
                usage()
            );
        }
    }
    Ok(())
}

/// The servo a command was pointed at, or nothing for the command's own default
/// — every servo for `reboot`, an antenna for `watchdog`.
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

fn record_path(args: &Args) -> PathBuf {
    args.record
        .clone()
        .unwrap_or_else(|| config::record_path_beside(&args.config))
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
            "reachy-bench: the self-test did not pass — {}.",
            short.join(", ")
        );
    }
    Ok(())
}

/// The roster and the bus timing a bare-bus command runs on.
///
/// All three of them need exactly this and nothing more: no envelope, no
/// geometry, no datum — none of them converts an angle. Read before any port is
/// opened, so a refusal costs the machine nothing.
fn bare_config(args: &Args) -> anyhow::Result<(ServoMap, reachy_bus::BusTiming, String)> {
    let cfg = config::load(&args.config)?;
    let map = ServoMap::new(cfg.servo_ids()?);
    let timing = cfg.bus_timing()?;
    Ok((map, timing, cfg.bus.device))
}

/// Open the port a bare-bus command runs over.
fn bare_port(device: &str, baud: u32) -> anyhow::Result<SerialBusPort> {
    SerialBusPort::open(device, baud).with_context(|| format!("opening {device}"))
}

/// What a bare-bus command's refusal reads as.
fn refused(command: &str, error: BareError) -> anyhow::Error {
    anyhow::Error::new(error).context(format!("`{command}`"))
}

/// Write the antennas' operating mode.
///
/// Not gated on anything: this moves nothing, so it needs no envelope, no
/// kinematics and no torque, and reads the roster and the bus timing straight
/// out of the file.
fn provision(args: &Args) -> anyhow::Result<()> {
    let (map, timing, device) = bare_config(args)?;

    println!(
        "provision over {device} at {} baud. This writes one non-volatile register on the two \
         antenna servos and moves nothing.\n\
         Torque must be off on both — release it with `off`, which releases wherever \
         the machine is, and take the head's weight first.",
        timing.baud
    );

    let port = bare_port(&device, timing.baud)?;
    bare::provision(&map, timing, port, &mut |line| println!("{line}"))
        .map_err(|error| refused("provision", error))
}

/// Restart the servos and report what they come back holding.
///
/// Not gated on anything: a reboot commands no angle, so it needs no conversion,
/// no envelope and no kinematics — only the roster and the bus timing. It is
/// also a de-torque, and nothing gates a de-torque.
fn reboot(args: &Args, target: Option<u8>) -> anyhow::Result<()> {
    let (map, timing, device) = bare_config(args)?;

    // The header only: what a reboot costs is said once, by the command itself,
    // where every caller of it hears the same words.
    println!("reboot over {device} at {} baud.", timing.baud);

    let port = bare_port(&device, timing.baud)?;
    let mut clock = MonotonicClock::new();

    bare::reboot(&map, timing, port, target, &mut clock, &mut |line| {
        println!("{line}")
    })
    .map_err(|error| refused("reboot", error))
}

/// Write torque off on every servo.
///
/// Not gated on anything, and that is the whole point: this is a de-torque, so
/// where the machine is standing, what a registry said about it, and whether a
/// datum was ever recorded all decide nothing here.
fn off(args: &Args) -> anyhow::Result<()> {
    let (map, timing, device) = bare_config(args)?;

    println!("off over {device} at {} baud.", timing.baud);

    let port = bare_port(&device, timing.baud)?;
    bare::off(&map, timing, port, &mut |line| println!("{line}"))
        .map_err(|error| refused("off", error))
}

/// Establish what an armed bus watchdog does to one servo.
///
/// Not gated on anything the machine says, and it needs no envelope: the only
/// goal it writes is the count the servo just reported for itself, so the
/// conversion this tool has no business doing never comes up. What it does need
/// is an operator standing there — it holds torque on one servo for a few
/// seconds, and the trip does not release it: the command's own make-safe is
/// what does, on the way out — so the warning is printed by the command itself,
/// before the port is opened.
fn watchdog(args: &Args, target: Option<u8>) -> anyhow::Result<()> {
    let (map, timing, device) = bare_config(args)?;

    println!("watchdog over {device} at {} baud.", timing.baud);

    let port = bare_port(&device, timing.baud)?;
    let mut clock = MonotonicClock::new();

    bare::watchdog(&map, timing, port, target, &mut clock, &mut |line| {
        println!("{line}")
    })
    .map_err(|error| refused("watchdog", error))
}

/// Hold one servo where it stands and read it at bus speed.
///
/// Not gated on anything the machine says, and it needs no envelope: the only
/// goal it writes is the count the servo just reported for itself. What it does
/// need is an operator standing there — it holds torque on one servo for both
/// phases — so the warning is the command's own, printed before the port is
/// opened.
///
/// The series is written before the verdict is acted on, for the reason the
/// self-test's record is: a refused run is the reading a bring-up most wants
/// kept, and an early return between the run and the save would throw away a
/// hardware round trip.
fn hold_probe(args: &Args, target: Option<u8>) -> anyhow::Result<()> {
    let (map, timing, device) = bare_config(args)?;
    let seconds = std::time::Duration::from_secs(args.seconds.unwrap_or(HOLD_PROBE_SECONDS));

    println!("hold-probe over {device} at {} baud.", timing.baud);

    let port = bare_port(&device, timing.baud)?;
    let mut clock = MonotonicClock::new();

    let request = ProbeRequest {
        target,
        gains: args.gains,
        seconds,
    };
    let run = bare::hold_probe(&map, timing, port, request, &mut clock, &mut |line| {
        println!("{line}")
    })
    .map_err(|error| refused("hold-probe", error))?;

    // The verdict outlives a failed write. A series that could not be written
    // down is a host problem — a full tmpfs, most likely, which is where these
    // land — and reporting it in place of the finding the run made would throw
    // away the hardware round trip that found it. So the write's failure is
    // said out loud and kept, and it becomes the exit status only when the
    // probe itself had nothing to report.
    let saved = save_probe(&run, &record_path(args), now_unix());
    if let Err(failed) = &saved {
        eprintln!("hold-probe: the series could not be written: {failed:#}");
    }
    verdict_before_write(run.outcome, saved)
}

/// What a probe run exits with: its own verdict, or the series write's failure
/// when the probe had nothing to report.
///
/// A separate step because the order is the whole point and an inversion of it
/// is invisible at the call site: a hardware finding reported as "the series
/// could not be written" throws away the round trip that found it.
fn verdict_before_write(
    outcome: Result<(), BareError>,
    saved: anyhow::Result<()>,
) -> anyhow::Result<()> {
    outcome.map_err(|error| refused("hold-probe", error))?;
    saved
}

/// Write a probe's two series beside the record, and say where they went.
///
/// Created, never overwritten. The name carries the moment the run was taken,
/// off a device whose clock is RAM and whose home is a tmpfs: a boot without a
/// network, or a step once one arrives, gives a stamp that repeats. A second
/// run landing on an already-used name is refused loudly rather than writing
/// over a hardware reading that cost a round trip.
fn save_probe(run: &ProbeRun, record: &Path, taken_at_unix: u64) -> anyhow::Result<()> {
    let beside = record.parent().unwrap_or(Path::new(""));
    let path = beside.join(format!(
        "{HOLD_PROBE_SERIES_PREFIX}{taken_at_unix}-{id}.csv",
        id = run.id
    ));
    let mut file = std::fs::OpenOptions::new()
        .write(true)
        .create_new(true)
        .open(&path)
        .with_context(|| format!("writing {}", path.display()))?;
    file.write_all(run.csv().as_bytes())
        .with_context(|| format!("writing {}", path.display()))?;
    println!("series written to {}", path.display());
    Ok(())
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

    /// An invocation carrying `--no-drain` is refused by name rather than
    /// quietly ignored: an operator running the timing case with it would
    /// otherwise believe they had measured two different ports.
    #[test]
    fn the_drain_switch_is_no_longer_an_option() {
        // Both shapes: the flag with a word after it, and the flag last, which
        // is what an operator running the timing case actually types. A parser
        // that took the value before matching the name refused the second with
        // "needs a value", which reads as a flag that still exists.
        for invocation in [&["--no-drain", "11"][..], &["--no-drain"][..]] {
            let refusal = parse_args(argv(invocation)).expect_err("it is not an option");
            assert!(
                refusal.to_string().contains("unknown option `--no-drain`"),
                "the refusal names the flag: {refusal}"
            );
        }
        assert!(
            !usage().contains("--no-drain"),
            "and the usage text does not offer it"
        );
    }

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
    /// `selftest off` is the case that matters: run as `selftest`, it would
    /// leave torque wherever it was and exit success, and an operator who read
    /// that as a release ends the session by cutting power with the head up.
    #[test]
    fn a_stray_operand_is_refused_rather_than_discarded() {
        for words in [
            vec!["selftest", "extra"],
            vec!["selftest", "off"],
            vec!["provision", "now"],
            vec!["reboot", "11", "12"],
            vec!["off", "please"],
            vec!["watchdog", "11", "12"],
        ] {
            let refused = dispatch(argv(&words)).expect_err("that word means nothing here");
            let printed = refused.to_string();
            assert!(printed.contains("operand"), "{words:?}: {printed}");
            assert!(printed.contains("usage:"), "{words:?}: {printed}");
        }
    }

    /// A flag a command does not read is refused rather than dropped.
    ///
    /// `off --gains 300,0,50` parses, and every word of it is a flag this
    /// program defines; run, it would write no gain and say nothing. An
    /// operator who believes a triple was swapped reads the next run as the
    /// machine's answer to it.
    #[test]
    fn a_flag_a_command_does_not_read_is_refused() {
        for words in [
            vec!["off", "--gains", "300,0,50"],
            vec!["provision", "--seconds", "30"],
            vec!["reboot", "--record", "/tmp/r"],
            vec!["watchdog", "--gains", "200,0,0"],
            vec!["selftest", "--seconds", "5"],
        ] {
            let refused = dispatch(argv(&words)).expect_err("that command does not read it");
            let printed = refused.to_string();
            assert!(printed.contains(words[0]), "{words:?}: {printed}");
            assert!(printed.contains(words[1]), "{words:?}: {printed}");
            assert!(printed.contains("usage:"), "{words:?}: {printed}");
        }
    }

    /// Every command reads `--config`, and the ones that take the rest say so
    /// in the same place the parser reads them.
    #[test]
    fn the_flags_a_command_takes_are_the_flags_it_reads() {
        assert_eq!(
            Command::HoldProbe.flags(),
            &[RECORD_FLAG, GAINS_FLAG, SECONDS_FLAG]
        );
        assert_eq!(Command::Selftest.flags(), &[RECORD_FLAG]);
        for command in Command::ALL {
            assert!(
                !command.flags().contains(&CONFIG_FLAG),
                "{}: every command reads the configuration, so it is not listed",
                command.name()
            );
            let args = parse_args(argv(&["--config", "/tmp/c.toml"]))
                .expect("the configuration is every command's");
            assert!(
                check_invocation(&args, command).is_ok(),
                "{}",
                command.name()
            );
        }
    }

    /// The name the probe writes its series under is the name the fetch globs
    /// for.
    ///
    /// Nothing joins a Rust `format!` to a shell glob but the text itself, so
    /// the prefix is one constant and this is the half of the join the binary
    /// can assert; `tools/deploy-bench.test.sh` compares that constant against
    /// the script.
    #[test]
    fn a_series_is_written_under_the_name_the_fetch_looks_for() {
        // `TEST_TMPDIR` under bazel: private per test target and per run, so
        // nothing here writes into the source tree or races another case.
        let dir = std::env::var_os("TEST_TMPDIR")
            .map_or_else(std::env::temp_dir, PathBuf::from)
            .join("probe-series");
        std::fs::create_dir_all(&dir).expect("a writable temporary directory");
        let run = ProbeRun {
            id: 18,
            phases: Vec::new(),
            outcome: Ok(()),
        };
        save_probe(&run, &dir.join(RECORD_NAME), 1_750_000_000).expect("the directory is writable");
        let written: Vec<String> = std::fs::read_dir(&dir)
            .expect("the directory is there")
            .map(|entry| {
                entry
                    .expect("a readable entry")
                    .file_name()
                    .to_string_lossy()
                    .into_owned()
            })
            .collect();
        assert_eq!(written, vec!["hold-probe-1750000000-18.csv".to_string()]);
        assert!(
            written[0].starts_with(HOLD_PROBE_SERIES_PREFIX),
            "{written:?}"
        );
        assert!(written[0].ends_with(".csv"), "{written:?}");
    }

    /// The gains a `--gains` flag carries reach the register in the order they
    /// were typed, and every malformed one is refused by name.
    ///
    /// A transposed term here writes an operator's derivative gain into the
    /// proportional slot on a servo about to hold torque for six seconds, which
    /// is a reading of a machine nobody configured.
    #[test]
    fn the_gains_flag_parses_p_i_d_in_that_order() {
        let args = parse_args(argv(&["hold-probe", "--gains", "1,2,3"]))
            .expect("three terms are a triple");
        assert_eq!(args.gains, Some(Gains { p: 1, i: 2, d: 3 }));
        // Spaces around a term are an operator's, not a typo.
        let args = parse_args(argv(&["hold-probe", "--gains", "500, 0, 100"]))
            .expect("three terms are a triple");
        assert_eq!(
            args.gains,
            Some(Gains {
                p: 500,
                i: 0,
                d: 100
            })
        );

        for (word, says) in [
            ("200,0", "three terms"),
            ("200,0,0,0", "three terms"),
            ("", "three terms"),
            ("x,0,0", "proportional"),
            ("200,x,0", "integral"),
            ("200,0,x", "derivative"),
            ("200,0,70000", "derivative"),
        ] {
            let refused = parse_args(argv(&["hold-probe", "--gains", word]))
                .expect_err("that is not a triple");
            let printed = format!("{refused:#}");
            assert!(printed.contains("--gains"), "`{word}`: {printed}");
            assert!(printed.contains(says), "`{word}`: {printed}");
            assert!(printed.contains("usage:"), "`{word}`: {printed}");
        }
    }

    /// A phase of no length is refused rather than run: it would measure
    /// nothing and print a rate over no readings.
    #[test]
    fn the_seconds_flag_takes_a_length_and_refuses_no_length() {
        let args =
            parse_args(argv(&["hold-probe", "--seconds", "12"])).expect("twelve is a length");
        assert_eq!(args.seconds, Some(12));

        for (word, says) in [
            ("0", "at least one second"),
            ("x", "not a number of seconds"),
            ("-3", "not a number of seconds"),
            ("2.5", "not a number of seconds"),
        ] {
            let refused = parse_args(argv(&["hold-probe", "--seconds", word]))
                .expect_err("that is not a phase length");
            let printed = format!("{refused:#}");
            assert!(printed.contains("--seconds"), "`{word}`: {printed}");
            assert!(printed.contains(says), "`{word}`: {printed}");
            assert!(printed.contains("usage:"), "`{word}`: {printed}");
        }
    }

    /// The probe's own verdict is what a run exits with; the series write's
    /// failure is what it exits with only when the probe had nothing to say.
    #[test]
    fn a_hardware_finding_outranks_a_series_that_could_not_be_written() {
        let finding = || Err(BareError::HoldProbeTorqueHeld { id: 18 });
        let unwritable = || Err(anyhow::anyhow!("writing /nowhere/series.csv"));

        verdict_before_write(Ok(()), Ok(())).expect("a quiet hold that was written down");

        let refused = verdict_before_write(finding(), Ok(())).expect_err("the probe refused");
        assert!(format!("{refused:#}").contains("holding"), "{refused:#}");

        let refused =
            verdict_before_write(finding(), unwritable()).expect_err("the probe still refused");
        assert!(
            format!("{refused:#}").contains("holding"),
            "the write's failure displaced the finding: {refused:#}"
        );

        let refused =
            verdict_before_write(Ok(()), unwritable()).expect_err("the series went nowhere");
        assert!(format!("{refused:#}").contains("series.csv"), "{refused:#}");
    }

    /// A series is created, never written over, and a directory that will not
    /// take it says which path it was.
    ///
    /// The device's clock is RAM: a boot without a network repeats a stamp, and
    /// a second run under a name already taken would otherwise truncate the
    /// first run's readings.
    #[test]
    fn a_series_never_writes_over_one_already_there() {
        let dir = std::env::var_os("TEST_TMPDIR")
            .map_or_else(std::env::temp_dir, PathBuf::from)
            .join("probe-series-twice");
        std::fs::create_dir_all(&dir).expect("a writable temporary directory");
        let run = ProbeRun {
            id: 17,
            phases: Vec::new(),
            outcome: Ok(()),
        };
        let record = dir.join(RECORD_NAME);
        save_probe(&run, &record, 1_750_000_000).expect("the name is free");
        let refused = save_probe(&run, &record, 1_750_000_000)
            .expect_err("that name is a series already taken");
        let printed = format!("{refused:#}");
        assert!(
            printed.contains("hold-probe-1750000000-17.csv"),
            "{printed}"
        );
        assert_eq!(
            std::fs::read_to_string(dir.join("hold-probe-1750000000-17.csv"))
                .expect("the first series is still there"),
            run.csv(),
        );

        let refused = save_probe(&run, &dir.join("nowhere").join(RECORD_NAME), 1_750_000_001)
            .expect_err("there is no such directory");
        assert!(
            format!("{refused:#}").contains("hold-probe-1750000001-17.csv"),
            "{refused:#}"
        );
    }

    /// There is no flag to authorise a release: `off` releases wherever the
    /// machine is, so an operator typing one gets it refused by name rather
    /// than silently accepted.
    #[test]
    fn there_is_no_flag_authorising_a_release() {
        for command in ["selftest", "provision", "reboot", "off", "watchdog"] {
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

    /// The commands this tool no longer has are refused by name rather than
    /// silently reinterpreted.
    ///
    /// An operator with a script from the motion era, or a habit from it, must
    /// be told the command is gone. A `stow` that fell through to anything at
    /// all would be a command they did not ask for on a machine they are
    /// standing next to.
    #[test]
    fn the_retired_motion_commands_are_gone_by_name() {
        for word in [
            "arm", "up", "hold", "stow", "yaw", "antennas", "demo", "play",
        ] {
            let refused = dispatch(argv(&[word])).expect_err("that command is retired");
            let printed = refused.to_string();
            assert!(printed.contains(word), "{word}: {printed}");
            assert!(printed.contains("unknown command"), "{word}: {printed}");
        }
    }

    /// The way out of a session is in the text an operator has in front of them
    /// when they need it.
    ///
    /// An operator holding a head up with one hand does not read the source:
    /// they read the banner, and if it omits the release that works from
    /// anywhere, that is a release they do not know about.
    #[test]
    fn the_operator_text_names_the_release_that_works_from_anywhere() {
        let text = usage();
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

    /// The banner does not offer a coordinated move, because this tool has
    /// none: a text describing commands that left the build is a text
    /// describing a different machine from the one that ships.
    #[test]
    fn the_operator_text_offers_no_coordinated_motion() {
        let text = usage();
        // The listed word itself, at the boundary the listing gives it: a
        // command's entry is its name, indented, and whatever follows on that
        // line. Matching the whole word rather than a prefix is what keeps
        // `hold-probe` — a read of one servo standing where it already is —
        // from reading as an offer to command a hold, while a bare `stow`
        // with nothing after it on the line is still caught.
        let listed: Vec<&str> = text
            .lines()
            .filter(|line| line.starts_with("  ") && !line.starts_with("   "))
            .filter_map(|line| line.split_whitespace().next())
            .collect();
        assert!(
            listed.contains(&"hold-probe"),
            "the listing this reads is not the listing: {text}"
        );
        for gone in [
            "arm", "up", "hold", "stow", "yaw", "antennas", "demo", "play",
        ] {
            assert!(!listed.contains(&gone), "`{gone}` is still offered: {text}");
        }
        assert!(text.contains("Nothing here commands an angle"), "{text}");
    }

    /// Every command that writes to a servo reads its configuration first, so a
    /// missing one stops each of them in the same place — before any port is
    /// opened.
    #[test]
    fn every_writing_command_reads_its_configuration_before_the_port() {
        for words in [
            vec!["provision"],
            vec!["reboot"],
            vec!["reboot", "11"],
            vec!["off"],
            vec!["watchdog"],
            vec!["watchdog", "11"],
            vec!["hold-probe"],
            vec!["hold-probe", "11"],
        ] {
            let refused = dispatch(argv(
                &[&words[..], &["--config", "/nonexistent/reachy-bench.toml"]].concat(),
            ))
            .expect_err("there is no configuration there");
            let printed = format!("{refused:#}");
            assert!(printed.contains("configuration"), "{words:?}: {printed}");
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
