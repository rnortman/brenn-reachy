//! `reachy-ask` — the harness's intent source: one gesture, through the real
//! edge.
//!
//! A motion run's verdict is the analyzer's reading of the log the control
//! process wrote, and the log says nothing unless something asked the machine
//! to move. On a unit and on a workstation the thing that asks is this binary,
//! standing exactly where the voice host stands in production: outside the
//! composition, holding the two loopback ports of the intent edge, sending a
//! compiled `Script` to 7409 and following the session's narration off 7410.
//!
//! It links `reachy-edge`, and that is the point. The decode, the four screens,
//! the compile, the timeline diff, the narration and the loop that drives them
//! are not re-written here: this binary holds the same `HostEdge` the voice
//! host holds, so a run proves the edge rather than a rehearsal of it. What it
//! does not carry is the host's other halves — no bus attachment, no audio, no
//! speech services — because a motion run must need none of them. What is its
//! own is the trigger, the two deadlines, and the one line it adds.
//!
//! The order of operations is the run's whole start-up story: bind 7410 first,
//! then let the launcher start the composition. The bind precedes the control
//! process's first narration by construction, so no start-ordering race exists
//! to tolerate, and the harness scripts start this binary before the launcher
//! for that reason.
//!
//! Nothing here retries and nothing here is configurable about the gesture. A
//! refusal — from the edge's own screens or from the session — is a red run:
//! the harness asks once, and a run that got something other than what it asked
//! for is a finding, not something to have another go at.

#![forbid(unsafe_code)]

mod gesture;
mod watch;

use std::io;
use std::net::UdpSocket;
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use clockwork_rs::SyncTime;
use reachy_edge::{
    Alert, DATAGRAM_CAP, EdgeConfig, HostEdge, LOOPBACK, MotionTable, Origin, POLL,
    REPORTS_OUT_PORT, SCRIPTS_IN_PORT, Surface, alert_line, now,
};
use signal_hook::consts::{SIGINT, SIGTERM};
use signal_hook::flag;

use gesture::{ASK_POD, body};
use watch::Watch;

/// How long the run waits for the session to say it commissioned, unless the
/// invocation says otherwise.
///
/// Commissioning is about five seconds of bus transactions on a machine with a
/// bus, and less on the simulated plant. Thirty seconds is room for a loaded
/// workstation and for a unit whose survey retries, and it is a red timeout
/// rather than a budget: a run that spends it has a machine that never
/// commissioned, which is the finding.
const RESTING_TIMEOUT: Duration = Duration::from_secs(30);

/// How long the run keeps following the story after the gesture goes out,
/// unless the invocation says otherwise.
///
/// The gesture itself runs thirteen seconds and the release that follows it a
/// few more. Twenty-five leaves the whole of it inside the window with margin,
/// and nothing about the verdict is read off this clock — the analyzer reads
/// the log — so it only has to be long enough.
const RUN_WINDOW: Duration = Duration::from_secs(25);

/// What the invocation asked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Options {
    /// How long to wait for the commissioning row before calling the run red.
    resting_timeout: Duration,
    /// How long to keep following the story once the gesture has gone out.
    run_window: Duration,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            resting_timeout: RESTING_TIMEOUT,
            run_window: RUN_WINDOW,
        }
    }
}

/// How to invoke this, for a refusal to print.
fn usage() -> String {
    format!(
        "usage: reachy-ask [--resting-timeout SECONDS] [--run-window SECONDS]\n\
         \n\
         Binds {REPORTS_OUT_PORT} on loopback, waits for the session to narrate that it\n\
         commissioned, sends one compiled gesture to {SCRIPTS_IN_PORT}, and narrates the story\n\
         until the run window ends. One line of JSON per row, on stdout.\n\
         \n\
         Start it before the launcher: the bind has to precede the control process's first\n\
         narration, and this is what makes that true rather than likely.\n\
         \n\
         Nothing is retried. A commissioning row that never arrives, a gesture the edge\n\
         refuses and a port already held are each a nonzero exit.\n\
         \n\
         Defaults: --resting-timeout {}, --run-window {}.",
        RESTING_TIMEOUT.as_secs(),
        RUN_WINDOW.as_secs(),
    )
}

fn main() -> ExitCode {
    match parse(std::env::args().skip(1)) {
        Ok(options) => match run(&options) {
            Ok(()) => ExitCode::SUCCESS,
            Err(message) => {
                eprintln!("reachy-ask: {message}");
                ExitCode::FAILURE
            }
        },
        Err(message) => {
            eprintln!("reachy-ask: {message}\n\n{}", usage());
            ExitCode::FAILURE
        }
    }
}

/// What the invocation asks for.
///
/// Two optional flags, each with a value in whole seconds. A word this does not
/// know is a refusal rather than something ignored: a harness run on the
/// shipped numbers, when an operator meant others, is a run whose timing means
/// something other than what they read.
///
/// The word loop, the once-only bool per flag and the refusal wording are a
/// third copy of a shape the driver and the host each carry their own of.
/// TODO(cli-argv-shared)
fn parse(args: impl Iterator<Item = String>) -> Result<Options, String> {
    let mut options = Options::default();
    let mut resting_given = false;
    let mut window_given = false;
    let mut args = args;
    while let Some(word) = args.next() {
        match word.as_str() {
            "--resting-timeout" => {
                let value = seconds(&word, args.next())?;
                if resting_given {
                    return Err(format!("{word} was given twice"));
                }
                resting_given = true;
                options.resting_timeout = value;
            }
            "--run-window" => {
                let value = seconds(&word, args.next())?;
                if window_given {
                    return Err(format!("{word} was given twice"));
                }
                window_given = true;
                options.run_window = value;
            }
            other => return Err(format!("`{other}` is not an option this takes")),
        }
    }
    Ok(options)
}

/// A flag's value, in whole seconds.
///
/// Zero is refused: a window of no time is a run that binds the port and exits
/// before the composition has started, and the analyzer would then judge a log
/// nothing asked for.
fn seconds(flag: &str, value: Option<String>) -> Result<Duration, String> {
    let value = value.ok_or_else(|| format!("{flag} needs a whole number of seconds"))?;
    let count: u64 = value
        .parse()
        .map_err(|_| format!("{flag} takes a whole number of seconds, not `{value}`"))?;
    if count == 0 {
        return Err(format!(
            "{flag} of zero seconds leaves the run no time at all"
        ));
    }
    Ok(Duration::from_secs(count))
}

/// Where the harness's narration goes: stdout, one JSON object per line, alerts
/// among them.
///
/// A run has nobody to interrupt — the analyzer over the log is the verdict —
/// so an alert the edge's table raised is one more line, said where it happened
/// rather than swallowed.
#[derive(Clone, Copy, Debug, Default)]
struct Console;

impl Surface for Console {
    fn say(&mut self, line: String) {
        println!("{line}");
    }

    fn alert(&mut self, alert: &Alert) {
        println!("{}", alert_line(alert, now()));
    }
}

/// Hold the ports, ask once, narrate until the window ends.
fn run(options: &Options) -> Result<(), String> {
    let reports = UdpSocket::bind((LOOPBACK, REPORTS_OUT_PORT)).map_err(|error| {
        // Only the address-in-use case names another reader. A permission
        // refusal or an exhausted descriptor table are the other ways this
        // fails, and naming a process that does not exist is worse than saying
        // what the operating system said.
        let cause = if error.kind() == io::ErrorKind::AddrInUse {
            "; something else already holds it — a second harness run, or a voice host on this \
             machine"
        } else {
            ""
        };
        format!("binding the reports port {REPORTS_OUT_PORT} on loopback: {error}{cause}")
    })?;
    reports
        .set_read_timeout(Some(POLL))
        .map_err(|error| format!("setting the read timeout on the reports port: {error}"))?;
    // An ephemeral source port: the seam identifies a datagram by the port it
    // arrived on, never by where it came from.
    let scripts = UdpSocket::bind((LOOPBACK, 0))
        .map_err(|error| format!("opening the socket the gesture goes out on: {error}"))?;

    // The stop the harness sends when the launcher is done. A background child
    // of a non-interactive shell inherits SIGINT ignored, so a handler is what
    // makes the stop act at all -- without one the scripts' `kill -INT` is a
    // no-op and the run holds the narration port until its own deadline.
    let stop = Arc::new(AtomicBool::new(false));
    for signal in [SIGINT, SIGTERM] {
        flag::register(signal, Arc::clone(&stop))
            .map_err(|error| format!("installing the stop flag for signal {signal}: {error}"))?;
    }

    let mut host = HostEdge::new(EdgeConfig::for_pod(ASK_POD), MotionTable::default());
    let mut surface = Console;
    let mut watch = Watch::new();
    let mut buffer = vec![0u8; DATAGRAM_CAP];
    let mut deadline = Instant::now() + options.resting_timeout;

    while !stop.load(Ordering::Relaxed) {
        if Instant::now() >= deadline {
            return verdict(
                watch.asked(),
                &format!(
                    "no commissioning row in {}s",
                    options.resting_timeout.as_secs()
                ),
            );
        }
        let read = match reports.recv_from(&mut buffer) {
            Ok((bytes, _)) => bytes,
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock
                        | io::ErrorKind::TimedOut
                        | io::ErrorKind::Interrupted
                ) =>
            {
                // A signal arriving mid-read is the stop this loop is watching
                // for, not a failed run: the flag above is what says so.
                continue;
            }
            Err(error) => return Err(format!("reading the reports port: {error}")),
        };

        // A datagram that is not a story is narrated and dropped by the edge
        // itself — the same line the host writes for it, on the same stream.
        let Some(update) = host.follow(&buffer[..read], now(), &mut surface) else {
            continue;
        };

        if watch.should_ask(&update.rows) {
            let arrival = now();
            let accepted = host
                .offer(
                    body(ASK_POD).as_bytes(),
                    Origin::Local,
                    arrival,
                    &mut surface,
                )
                .ok_or_else(|| {
                    "the edge refused the harness gesture; the line above names the screen it \
                     stopped at"
                        .to_owned()
                })?;
            scripts
                .send_to(accepted.bytes(), (LOOPBACK, SCRIPTS_IN_PORT))
                .map_err(|error| {
                    format!(
                        "sending the gesture to the control process on {SCRIPTS_IN_PORT}: {error}"
                    )
                })?;
            surface.say(asked_line(
                accepted.script_id,
                options.run_window.as_secs(),
                arrival,
            ));
            deadline = Instant::now() + options.run_window;
        }
    }
    verdict(watch.asked(), "stopped before the session commissioned")
}

/// What the run says when the following ends: green if the gesture went out,
/// red if nothing was ever asked for.
///
/// The whole point of the harness is that the analyzer judges a log of a machine
/// that was asked to move. A run that ends without having asked is red here
/// rather than a green exit over a log with no gesture in it -- which is the one
/// failure this binary exists to catch and the one an inverted predicate would
/// let through.
fn verdict(asked: bool, ending: &str) -> Result<(), String> {
    if asked {
        return Ok(());
    }
    Err(format!(
        "{ending}: the session never narrated entering `resting` from `starting`, so nothing was \
         ever asked for. Either the control process did not start, or it did not finish its \
         survey."
    ))
}

/// The gesture went out, as a line on the same stream the story is narrated on.
///
/// The edge's own narration has no kind for this — it renders what arrives and
/// what it refused, and a script it accepted is neither — so the one line the
/// harness adds is written here, in the same shape, so one reader parses the
/// whole stream.
fn asked_line(script_id: u32, window_secs: u64, at: SyncTime) -> String {
    serde_json::json!({
        "stream": "edge",
        "at_ns": at.as_nanos(),
        "kind": "asked",
        "script_id": script_id,
        "says": format!(
            "the harness gesture went out as script {script_id}; following the story for \
             {window_secs}s"
        ),
    })
    .to_string()
}

#[cfg(test)]
mod tests {
    use std::time::Duration;

    use super::{Options, parse, verdict};

    /// The invocation, as words.
    fn parsed(words: &[&str]) -> Result<Options, String> {
        parse(words.iter().map(|word| (*word).to_string()))
    }

    #[test]
    fn no_arguments_is_the_shipped_run() {
        assert_eq!(parsed(&[]), Ok(Options::default()));
    }

    #[test]
    fn both_windows_can_be_named() {
        assert_eq!(
            parsed(&["--resting-timeout", "5", "--run-window", "9"]),
            Ok(Options {
                resting_timeout: Duration::from_secs(5),
                run_window: Duration::from_secs(9),
            }),
        );
    }

    #[test]
    fn a_flag_it_does_not_know_is_a_refusal() {
        let refused = parsed(&["--lead-ms", "8000"]).expect_err("an unknown flag");
        assert!(refused.contains("--lead-ms"), "{refused}");
    }

    #[test]
    fn a_flag_needs_a_value_and_takes_whole_seconds() {
        assert!(parsed(&["--run-window"]).is_err());
        assert!(parsed(&["--run-window", "9.5"]).is_err());
        assert!(parsed(&["--run-window", "later"]).is_err());
    }

    #[test]
    fn a_window_of_no_time_is_refused() {
        let refused = parsed(&["--run-window", "0"]).expect_err("a run with no time in it");
        assert!(refused.contains("no time at all"), "{refused}");
    }

    #[test]
    fn a_run_that_asked_is_green_however_it_ended() {
        assert_eq!(verdict(true, "no commissioning row in 30s"), Ok(()));
        assert_eq!(
            verdict(true, "stopped before the session commissioned"),
            Ok(())
        );
    }

    #[test]
    fn a_run_that_never_asked_is_red_and_names_the_row() {
        let red = verdict(false, "no commissioning row in 30s").expect_err("a run with no gesture");
        assert!(red.contains("no commissioning row in 30s"), "{red}");
        assert!(red.contains("`resting`"), "{red}");
        assert!(red.contains("`starting`"), "{red}");
    }

    #[test]
    fn a_flag_given_twice_is_a_refusal() {
        let refused =
            parsed(&["--run-window", "9", "--run-window", "9"]).expect_err("one window, once");
        assert!(refused.contains("twice"), "{refused}");
    }
}
