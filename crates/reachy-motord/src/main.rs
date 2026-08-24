//! The driver process's entry point: configuration, the port, and the loop.
//!
//! Four steps, in this order and for a reason. Read the configuration, because
//! everything below is a number out of it. Bind the two inbound ports, because
//! a second driver already reading them is the one failure that must stop this
//! one before it touches the bus. Open the serial port exclusively, because
//! that is what makes this process the machine's only speaker. Then run the
//! grid, forever.
//!
//! Every one of those failures is an exit and none of them is retried: a
//! configuration that does not parse, a port somebody else holds, a device that
//! is not there. Restart policy belongs to whatever supervises this process,
//! and a restarted driver's own default is to let the machine go — so a crash
//! loop is a limp machine and a loud log, not a machine being commanded by a
//! process that half-started.
//!
//! What this file prints is the driver's whole voice for everything the event
//! vocabulary has no kind for: one line of counts per reporting interval. It is
//! printed here rather than deeper down because a library that printed would be
//! a library a test cannot run quietly.
//!
//! Nothing on the loop thread writes to stdout. A write to a pipe or a socket
//! whose reader has stopped consuming blocks until it does, and the loop thread
//! is the only thing in this process that can write a torque-off sweep — so a
//! wedged log collector would be a machine holding torque while the dead-man
//! never ran. The line goes to a printing thread through a queue that is never
//! waited on: when the reader is behind, lines are dropped and the cycle
//! carries on. Diagnostics do not gate de-torquing.

#![forbid(unsafe_code)]

use std::io;
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::mpsc::{SyncSender, TrySendError, sync_channel};
use std::thread;

use reachy_bus::{Bus, SerialBusPort};
use reachy_motord::grid::Grid;
use reachy_motord::inbound::Inbox;
use reachy_motord::loop_ctl::{Destinations, Driver, Outbound, RealTime};
use reachy_motord::params;
use reachy_motord::tick::{Tick, TickConfig, cycle_timing, now_ns};

/// Where the configuration is read from unless `--config` says otherwise.
///
/// The path a Bazel-built binary's runfiles put it at, which is also what the
/// deploy step copies: the file travels with the binary, and a driver run from
/// somewhere else is a driver told where to look.
const DEFAULT_CONFIG: &str = "driver/motord_params.textproto";

/// How many cycles between report lines: five seconds of a 20 ms grid.
///
/// Long enough that the log is readable by a human watching a run, short enough
/// that a run cut short by a signal has still said what it counted. A process
/// killed between two lines prints nothing more — there is no handler here,
/// because installing one needs a syscall this tree has no way to make, and
/// because a driver stopped last in the shutdown order is a driver whose
/// machine is already at rest.
const REPORT_CYCLES: u64 = 250;

/// Lines the printing thread may be behind by before they are dropped.
///
/// A handful: the interval is five seconds, so a queue deeper than this is a
/// reader that has been gone long enough that the old lines say nothing anybody
/// still wants. It exists to be shallow — the point is that the loop thread
/// never waits, not that no line is ever lost.
const REPORT_BACKLOG: usize = 4;

/// Somewhere to say things that never makes the caller wait.
///
/// The whole of this process's stdout. [`Reporting::say`] hands a line to the
/// printing thread if it has room and drops it if it does not: a line dropped is
/// counters an operator does not see for five seconds, and a line waited on is a
/// cycle that did not run.
struct Reporting {
    lines: SyncSender<String>,
}

impl Reporting {
    /// Start the printing thread.
    fn start() -> Self {
        let (lines, waiting) = sync_channel::<String>(REPORT_BACKLOG);
        thread::spawn(move || {
            for line in waiting {
                println!("reachy-motord: {line}");
            }
        });
        Self::over(lines)
    }

    /// Say things down `lines`, whoever is reading them.
    fn over(lines: SyncSender<String>) -> Self {
        Self { lines }
    }

    /// Say `line`, or drop it. Never blocks, and answers whether it was taken.
    fn say(&self, line: String) -> bool {
        match self.lines.try_send(line) {
            Ok(()) => true,
            // A full queue is a reader that is not reading; a disconnected one is
            // a printing thread that has gone. Neither is a reason to stop
            // running cycles.
            Err(TrySendError::Full(_) | TrySendError::Disconnected(_)) => false,
        }
    }
}

/// How to invoke this, for a refusal to print.
fn usage() -> String {
    format!(
        "usage: reachy-motord [--config PATH]\n\
         \n\
         Holds the servo bus and runs the 20 ms command cycle, taking goals and session\n\
         commands over loopback UDP and publishing pose, events, outcomes and health back.\n\
         There is no command here and no interactive mode: it runs until it is stopped.\n\
         \n\
         Nothing is retried. A configuration that does not parse, a port already held and a\n\
         device that is not there are each an exit. A driver nobody talks to de-torques the\n\
         machine and keeps running.\n\
         \n\
         Configuration defaults to {DEFAULT_CONFIG}, relative to the working directory."
    )
}

fn main() -> ExitCode {
    match parse(std::env::args().skip(1)) {
        Ok(config) => match run(&config) {
            Ok(()) => ExitCode::SUCCESS,
            Err(message) => {
                eprintln!("reachy-motord: {message}");
                ExitCode::FAILURE
            }
        },
        Err(message) => {
            eprintln!("reachy-motord: {message}\n\n{}", usage());
            ExitCode::FAILURE
        }
    }
}

/// Which configuration file the invocation names.
///
/// The whole of the argument grammar: one optional flag with one value. A word
/// this does not know is a refusal rather than something ignored — an operator
/// who misspelled a flag would otherwise get a driver running on the shipped
/// numbers and no sign that the ones they meant were not read.
fn parse(args: impl Iterator<Item = String>) -> Result<PathBuf, String> {
    let mut config: Option<PathBuf> = None;
    let mut args = args.peekable();
    while let Some(word) = args.next() {
        match word.as_str() {
            "--config" => {
                let value = args.next().ok_or("--config needs a path")?;
                if config.replace(PathBuf::from(value)).is_some() {
                    return Err("--config was given twice".to_string());
                }
            }
            other => return Err(format!("`{other}` is not an option this takes")),
        }
    }
    Ok(config.unwrap_or_else(|| PathBuf::from(DEFAULT_CONFIG)))
}

/// Build the driver out of `config` and run it.
///
/// Never returns of its own accord: the loop is endless, and this process ends
/// by being stopped or by one of the failures above.
fn run(config: &Path) -> Result<(), String> {
    let message = params::load(config).map_err(|error| error.to_string())?;
    let params = message
        .validate()
        .map_err(|invalid| format!("{} carries {invalid:?}", config.display()))?;

    // The seam before the bus: two drivers reading one command stream is worse
    // than none, and finding that out after taking the port would mean taking a
    // port this process is about to give back.
    let inbox = Inbox::bind().map_err(|error| {
        // Only the address-in-use case names a second driver. A permission
        // refusal, an exhausted descriptor table or a loopback interface that
        // is not there are the other ways this fails, and sending an operator
        // hunting for a process that does not exist is worse than saying only
        // what the operating system said.
        let cause = if error.kind() == io::ErrorKind::AddrInUse {
            "; another driver already holds them"
        } else {
            ""
        };
        format!("binding the driver's inbound ports on loopback: {error}{cause}")
    })?;
    let out = Outbound::open(Destinations::SEAM)
        .map_err(|error| format!("opening the outbound socket: {error}"))?;

    let device = params.bus_device.as_str();
    let port = SerialBusPort::open(device, params.bus_baud).map_err(|error| error.to_string())?;
    let bus = Bus::new(port, cycle_timing(params.bus_baud));
    let tick = Tick::new(
        bus,
        TickConfig {
            period_ns: params.period_ns,
            hold_timeout_ns: params.hold_timeout_ns,
            health_poll_period_ns: params.health_poll_period_ns,
        },
    );

    let grid = Grid::new(Grid::top_of_second_at(now_ns()), params.period_ns)
        .map_err(|error| error.to_string())?;
    let mut driver = Driver::new(tick, inbox, out, grid, RealTime, params.startup_window_ns);
    let reporting = Reporting::start();
    reporting.say(format!(
        "{device} at {} baud, {}ms cycle, grid starts at {}",
        params.bus_baud,
        params.period_ns / 1_000_000,
        grid.instant(0)
    ));
    loop {
        driver.run_cycles(REPORT_CYCLES);
        reporting.say(driver.report());
    }
}

#[cfg(test)]
mod tests {
    use super::{
        DEFAULT_CONFIG, REPORT_BACKLOG, REPORT_CYCLES, Reporting, parse, sync_channel, usage,
    };
    use std::path::PathBuf;

    fn invoked(words: &[&str]) -> Result<PathBuf, String> {
        parse(words.iter().map(|word| (*word).to_string()))
    }

    #[test]
    fn no_arguments_reads_the_configuration_that_travels_with_the_binary() {
        assert_eq!(invoked(&[]), Ok(PathBuf::from(DEFAULT_CONFIG)));
    }

    #[test]
    fn a_named_configuration_is_the_one_read() {
        assert_eq!(
            invoked(&["--config", "/run/motord.textproto"]),
            Ok(PathBuf::from("/run/motord.textproto"))
        );
    }

    #[test]
    fn an_invocation_this_does_not_understand_is_refused_rather_than_ignored() {
        for words in [
            vec!["--config"],
            vec!["--config", "a", "--config", "b"],
            vec!["selftest"],
            vec!["--deterministic-runner"],
            vec!["-c", "a"],
        ] {
            assert!(
                invoked(&words).is_err(),
                "{words:?} names something this driver does not take"
            );
        }
    }

    #[test]
    fn the_usage_says_what_the_reporting_interval_costs_nobody() {
        // A driver stopped by a signal prints no final line, so the interval is
        // the only thing that ever prints: five seconds of the shipped grid,
        // measured against the cycle the grid is built on rather than a restated
        // twenty.
        let interval_ns = REPORT_CYCLES as i64 * reachy_driver::NOMINAL_CYCLE_NS;
        assert_eq!(interval_ns, 5_000_000_000, "nanoseconds between lines");
        assert!(usage().contains(DEFAULT_CONFIG));
    }

    #[test]
    fn a_report_nobody_is_reading_is_dropped_rather_than_waited_on() {
        // A reader that never reads: what a stopped log collector, a full pipe
        // or a `kill -STOP`'d `tee` looks like from this side.
        let (lines, wedged) = sync_channel::<String>(REPORT_BACKLOG);
        let reporting = Reporting::over(lines);

        let taken = (0..1_000)
            .filter(|cycle| reporting.say(format!("cycle={cycle}")))
            .count();

        // The point is that this returned at all: a blocking write here would be
        // a loop thread that stopped running cycles — and stopped sweeping
        // torque off — because something stopped reading its stdout.
        assert_eq!(
            taken, REPORT_BACKLOG,
            "the queue's depth and not one line more"
        );
        drop(wedged);
        assert!(
            !reporting.say("after the reader is gone".to_string()),
            "a printer that has gone is not a reason to wait either"
        );
    }
}
