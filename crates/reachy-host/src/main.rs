//! The voice host's entry point: configuration, the two loopback ports, and the
//! loop that owns the edge.
//!
//! Four steps, in this order and for a reason. Read the configuration, because
//! the name this machine answers to and every screen below is a value out of
//! it. Read the clip name table, because an overlay names a motion and only
//! that file says which index a name has. Bind the reports port, because a
//! second host already reading it is the one failure that has to stop this one
//! before anything is asked of the machine. Install the stop flag, and then run
//! the loop.
//!
//! Every one of those failures is an exit and none of them is retried: a
//! configuration that does not parse, a name table that is not there, a port
//! somebody else holds. What supervises this process decides what happens next,
//! and a host that never comes back is a robot that is deaf and mute — never an
//! unsafe one. The motion stack owes the host nothing: the schedule already
//! running concludes at its own horizon, and the machine stows and rests there.
//!
//! What this prints is one JSON object per line on stdout: every row of the
//! session's story as it is narrated, every body the edge dropped, and every
//! alert the table raised. With no bus attachment configured the alerts are
//! those lines and nothing else.

#![forbid(unsafe_code)]

use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

use clockwork_rs::SyncTime;
use reachy_edge::{
    DATAGRAM_CAP, HostEdge, LOOPBACK, MotionTable, POLL, REPORTS_OUT_PORT, SCRIPTS_IN_PORT,
    Surface, now,
};
use reachy_host::edge::Console;
use reachy_host::intents::{Waiting, waking_queue};
use reachy_host::params::{self, HostSettings};
use signal_hook::consts::{SIGINT, SIGTERM};
use signal_hook::flag;

/// Where the configuration is read from unless `--config` says otherwise.
///
/// The path a Bazel-built binary's runfiles put it at, which is also what the
/// deploy step copies: the file travels with the binary, and a host run from
/// somewhere else is a host told where to look.
const DEFAULT_CONFIG: &str = "host/host_params.textproto";

/// What the invocation asked for.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Options {
    /// Which configuration to run on.
    config: PathBuf,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            config: PathBuf::from(DEFAULT_CONFIG),
        }
    }
}

/// How to invoke this, for a refusal to print.
fn usage() -> String {
    format!(
        "usage: reachy-host [--config PATH]\n\
         \n\
         Runs the robot's voice host: binds {REPORTS_OUT_PORT} on loopback, follows the\n\
         session's story, and sends compiled scripts to {SCRIPTS_IN_PORT}. One line of JSON\n\
         per row, on stdout.\n\
         \n\
         Nothing is retried and nothing is persisted. A configuration that does not parse,\n\
         a clip name table that is not there and a port already held are each a nonzero\n\
         exit.\n\
         \n\
         Default: --config {DEFAULT_CONFIG}."
    )
}

fn main() -> ExitCode {
    match parse(std::env::args().skip(1)) {
        Ok(options) => match run(&options) {
            Ok(()) => ExitCode::SUCCESS,
            Err(message) => {
                eprintln!("reachy-host: {message}");
                ExitCode::FAILURE
            }
        },
        Err(message) => {
            eprintln!("reachy-host: {message}\n\n{}", usage());
            ExitCode::FAILURE
        }
    }
}

/// What the invocation asks for.
///
/// One optional flag. A word this does not know is a refusal rather than
/// something ignored: a host run on the shipped configuration when an operator
/// meant a unit's own would answer to the wrong pod name.
fn parse(args: impl Iterator<Item = String>) -> Result<Options, String> {
    let mut options = Options::default();
    let mut given = false;
    let mut args = args;
    while let Some(word) = args.next() {
        match word.as_str() {
            "--config" => {
                let value = args
                    .next()
                    .ok_or_else(|| format!("{word} needs the path of a configuration"))?;
                if given {
                    return Err(format!("{word} was given twice"));
                }
                given = true;
                options.config = PathBuf::from(value);
            }
            other => return Err(format!("`{other}` is not an option this takes")),
        }
    }
    Ok(options)
}

/// Read the configuration and the name table, hold the ports, run.
fn run(options: &Options) -> Result<(), String> {
    let settings = params::load(&options.config).map_err(|error| error.to_string())?;
    let table = names(&settings)?;

    let reports = UdpSocket::bind((LOOPBACK, REPORTS_OUT_PORT)).map_err(|error| {
        // Only the address-in-use case names another reader. A permission
        // refusal or an exhausted descriptor table are the other ways this
        // fails, and naming a process that does not exist is worse than saying
        // what the operating system said.
        let cause = if error.kind() == io::ErrorKind::AddrInUse {
            "; something else already holds it — a second host, or a motion-run harness on this \
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
        .map_err(|error| format!("opening the socket scripts go out on: {error}"))?;
    // What an offer wakes this loop through: an empty datagram to the port the
    // loop is asleep in, so a body authored in-process is compiled and sent now
    // rather than at the next read timeout. Connected, so the queue's handle
    // holds a destination and not a port number.
    let nudge = UdpSocket::bind((LOOPBACK, 0))
        .map_err(|error| format!("opening the socket an offer wakes the loop through: {error}"))?;
    nudge
        .connect((LOOPBACK, REPORTS_OUT_PORT))
        .map_err(|error| format!("pointing the wake-up socket at {REPORTS_OUT_PORT}: {error}"))?;

    let stop = Arc::new(AtomicBool::new(false));
    for signal in [SIGINT, SIGTERM] {
        flag::register(signal, Arc::clone(&stop))
            .map_err(|error| format!("installing the stop flag for signal {signal}: {error}"))?;
    }

    // The sending half is what an intent-authoring task holds. No task holds one
    // yet: the two that will — the scripter's sink and the bus subscription —
    // are the wiring this process's other half brings, and until it lands the
    // loop below narrates the session's story and asks for nothing.
    // TODO(host-intent-producers)
    let (_intents, waiting) = waking_queue(Arc::new(nudge));
    let mut host = HostEdge::new(settings.edge.clone(), table);
    let mut surface = Console;
    surface.say(started_line(&settings, &options.config, now()));

    follow(
        &reports,
        &scripts,
        (LOOPBACK, SCRIPTS_IN_PORT).into(),
        &waiting,
        &mut host,
        &mut surface,
        &stop,
    )
}

/// The loop: a story datagram, then whatever intent is waiting, until stopped.
///
/// The story comes first because the read is what the loop sleeps in; the queue
/// is drained on every pass, whatever woke it. An offer wakes it immediately —
/// a body's own handle sends an empty datagram to the reports port for exactly
/// that — so the wake word's path to the motors does not wait on the read
/// timeout. The timeout is then a liveness tick: it is what makes a stop signal
/// act within a quarter second on a machine narrating nothing.
///
/// Where the compiled scripts go is a parameter rather than the constant the
/// caller passes, because the destination is the one thing about this loop that
/// a test cannot otherwise see: a regression sending a script to the reports
/// port, or dropping the send error, looks exactly like a robot that heard the
/// wake word and did not move.
///
/// An empty datagram is the nudge and carries nothing else. It is not offered
/// to the story follower, which would refuse it as a wrong-sized blob and
/// narrate a drop that nothing dropped; a stray one from elsewhere on loopback
/// costs one pass over an empty queue.
fn follow(
    reports: &UdpSocket,
    scripts: &UdpSocket,
    scripts_to: SocketAddr,
    waiting: &Waiting,
    host: &mut HostEdge,
    surface: &mut impl Surface,
    stop: &AtomicBool,
) -> Result<(), String> {
    let mut buffer = vec![0u8; DATAGRAM_CAP];
    while !stop.load(Ordering::Relaxed) {
        match reports.recv_from(&mut buffer) {
            Ok((0, _)) => {}
            Ok((read, _)) => {
                host.follow(&buffer[..read], now(), surface);
            }
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock
                        | io::ErrorKind::TimedOut
                        | io::ErrorKind::Interrupted
                ) => {}
            Err(error) => return Err(format!("reading the reports port: {error}")),
        }

        while let Some(body) = waiting.next() {
            let arrival = now();
            if let Some(accepted) = host.offer(&body, arrival, surface) {
                // Fire and forget: the session narrates what it decided with
                // the script, and a send that failed is said here because
                // nothing else would ever mention it.
                if let Err(error) = scripts.send_to(accepted.bytes(), scripts_to) {
                    surface.say(unsent_line(accepted.script_id, &error.to_string(), arrival));
                }
            }
        }
    }
    Ok(())
}

/// The clip library's name table, read once at startup.
fn names(settings: &HostSettings) -> Result<MotionTable, String> {
    let path: &Path = &settings.clip_names;
    let text = std::fs::read_to_string(path)
        .map_err(|error| format!("reading the clip name table at {}: {error}", path.display()))?;
    MotionTable::from_sidecar(&text).map_err(|error| {
        format!(
            "the clip name table at {} is not one this build can resolve names through: {error}",
            path.display()
        )
    })
}

/// What this host is, as the first line of its stream.
fn started_line(settings: &HostSettings, config: &Path, at: SyncTime) -> String {
    serde_json::json!({
        "stream": "edge",
        "at_ns": at.as_nanos(),
        "kind": "started",
        "pod": settings.edge.pod(),
        "says": format!(
            "the voice host answers for `{}`, configured by {}; reports on {REPORTS_OUT_PORT}, \
             scripts to {SCRIPTS_IN_PORT}",
            settings.edge.pod(),
            config.display(),
        ),
    })
    .to_string()
}

/// A compiled script that never left this machine, as a line.
fn unsent_line(script_id: u32, detail: &str, at: SyncTime) -> String {
    serde_json::json!({
        "stream": "edge",
        "at_ns": at.as_nanos(),
        "kind": "unsent",
        "script_id": script_id,
        "says": format!(
            "script {script_id} compiled and could not be sent to the control process: {detail}. \
             nothing is retried; the sender's next refresh is what recovers"
        ),
    })
    .to_string()
}

#[cfg(test)]
mod tests {
    use std::net::{Ipv6Addr, SocketAddr, UdpSocket};
    use std::path::PathBuf;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::thread;
    use std::time::Duration;

    use clockwork_rs::{SyncTime, blob_from_bytes};
    use motion_proto::{MotionScript, Posture, Step};
    use reachy_edge::{Alert, EdgeConfig, HostEdge, LOOPBACK, MotionTable, Surface};
    use reachy_host::intents::queue;

    use brenn_reachy__cogs__script_clk_rs::ScriptWire;

    use super::{DEFAULT_CONFIG, Options, follow, parse, unsent_line};

    /// The pod the fixture bodies are addressed to.
    const POD: &str = "fixture-reachy";

    /// How long a read waits in the cases below. Short, because every datagram
    /// they read was already queued before the loop started: the timeout is
    /// only what the loop wakes on to notice the stop.
    const READ: Duration = Duration::from_millis(20);

    /// Everything said, kept rather than printed.
    #[derive(Clone, Debug, Default)]
    struct Recorded {
        lines: Vec<String>,
    }

    impl Surface for Recorded {
        fn say(&mut self, line: String) {
            self.lines.push(line);
        }

        fn alert(&mut self, _alert: &Alert) {}
    }

    /// A script body for `POD`, as the wire contract encodes one: a raise now,
    /// with the closing stow left to the compile.
    fn body(seq: u64) -> Vec<u8> {
        MotionScript::new(POD, seq, vec![Step::new(0, Posture::Up)], 13_000)
            .expect("a lawful script")
            .encode()
            .into_bytes()
    }

    /// A reports port of this machine's own, with the short read.
    fn reports_port() -> UdpSocket {
        let socket = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
        socket
            .set_read_timeout(Some(READ))
            .expect("a read timeout on a bound socket");
        socket
    }

    /// Run the loop over what is already queued, until a stop arrives.
    ///
    /// Every input is in the socket's buffer or in the queue before the loop
    /// starts, so what the cases assert does not depend on when the stop lands
    /// — only that it lands.
    fn drive(reports: &UdpSocket, scripts_to: SocketAddr, bodies: &[Vec<u8>]) -> Recorded {
        let (intents, waiting) = queue();
        for body in bodies {
            intents.offer(body.clone()).expect("a queue with room");
        }
        let scripts = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
        let mut host = HostEdge::new(EdgeConfig::for_pod(POD), MotionTable::default());
        let mut surface = Recorded::default();
        let stop = Arc::new(AtomicBool::new(false));
        let raise = Arc::clone(&stop);
        let stopper = thread::spawn(move || {
            thread::sleep(READ * 10);
            raise.store(true, Ordering::Relaxed);
        });
        follow(
            reports,
            &scripts,
            scripts_to,
            &waiting,
            &mut host,
            &mut surface,
            &stop,
        )
        .expect("a loop that was stopped rather than broken");
        stopper.join().expect("the stopping thread");
        surface
    }

    #[test]
    fn a_nudge_is_not_offered_to_the_story_follower() {
        // The empty datagram an offer wakes the loop with. Read as a story it
        // would be a wrong-sized blob and narrate a drop that dropped nothing.
        let reports = reports_port();
        let sender = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
        sender
            .send_to(&[], reports.local_addr().expect("a bound port"))
            .expect("a datagram to a bound port");

        let said = drive(&reports, unreachable_destination(), &[]);
        assert!(said.lines.is_empty(), "{:?}", said.lines);
    }

    #[test]
    fn an_offered_body_leaves_as_a_script_on_the_scripts_port() {
        let reports = reports_port();
        let control = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
        control
            .set_read_timeout(Some(READ * 10))
            .expect("a read timeout on a bound socket");
        let destination = control.local_addr().expect("a bound port");

        let said = drive(&reports, destination, &[body(1)]);
        assert!(said.lines.is_empty(), "{:?}", said.lines);

        let mut buffer = vec![0u8; 4096];
        let (read, _) = control.recv_from(&mut buffer).expect("one script datagram");
        let script: ScriptWire =
            blob_from_bytes(&buffer[..read]).expect("the bytes of a `Script` and nothing else");
        assert_eq!(script.script_id(), 1);
        assert_eq!(
            script.steps().len(),
            2,
            "the raise and the synthesized stow"
        );
    }

    #[test]
    fn a_script_that_could_not_be_sent_is_said() {
        let reports = reports_port();
        let said = drive(&reports, unreachable_destination(), &[body(1)]);
        assert_eq!(said.lines.len(), 1, "{:?}", said.lines);
        let line: serde_json::Value =
            serde_json::from_str(&said.lines[0]).expect("one JSON object");
        assert_eq!(line["kind"], "unsent");
        assert_eq!(line["script_id"], 1);
    }

    #[test]
    fn a_stop_ends_the_loop() {
        let reports = reports_port();
        let scripts = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
        let (_intents, waiting) = queue();
        let mut host = HostEdge::new(EdgeConfig::for_pod(POD), MotionTable::default());
        let mut surface = Recorded::default();
        let stop = AtomicBool::new(true);
        follow(
            &reports,
            &scripts,
            unreachable_destination(),
            &waiting,
            &mut host,
            &mut surface,
            &stop,
        )
        .expect("a stopped loop is not a failed one");
        assert!(surface.lines.is_empty(), "{:?}", surface.lines);
    }

    /// A destination a send to cannot reach: an IPv6 address from a socket the
    /// operating system opened for IPv4. The refusal is the kernel's and is
    /// immediate, which is what makes the unsent line's case deterministic.
    fn unreachable_destination() -> SocketAddr {
        SocketAddr::from((Ipv6Addr::LOCALHOST, 9))
    }

    /// The invocation, as words.
    fn parsed(words: &[&str]) -> Result<Options, String> {
        parse(words.iter().map(|word| (*word).to_string()))
    }

    #[test]
    fn no_arguments_is_the_configuration_the_payload_carries() {
        assert_eq!(
            parsed(&[]),
            Ok(Options {
                config: PathBuf::from(DEFAULT_CONFIG),
            }),
        );
    }

    #[test]
    fn a_configuration_can_be_named() {
        assert_eq!(
            parsed(&["--config", "/run/reachy/host_params.textproto"]),
            Ok(Options {
                config: PathBuf::from("/run/reachy/host_params.textproto"),
            }),
        );
    }

    #[test]
    fn a_flag_it_does_not_know_is_a_refusal() {
        let refused = parsed(&["--pod", "kitchen-reachy"]).expect_err("an unknown flag");
        assert!(refused.contains("--pod"), "{refused}");
    }

    #[test]
    fn a_configuration_flag_needs_a_path_and_is_given_once() {
        assert!(parsed(&["--config"]).is_err());
        assert!(parsed(&["--config", "a", "--config", "b"]).is_err());
    }

    #[test]
    fn a_script_that_could_not_be_sent_says_which_one() {
        let at = SyncTime::from_nanos(1_700_000_000_000_000_000);
        let line = unsent_line(7, "no route to host", at);
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("one JSON object");
        assert_eq!(parsed["kind"], "unsent");
        assert_eq!(parsed["at_ns"], at.as_nanos());
        assert_eq!(parsed["script_id"], 7);
        assert!(
            parsed["says"]
                .as_str()
                .expect("a sentence")
                .contains("no route"),
            "{line}"
        );
    }
}
