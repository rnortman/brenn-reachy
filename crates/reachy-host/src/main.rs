//! The voice host's entry point: configuration, the two loopback ports, and the
//! loop that owns the edge.
//!
//! Five steps, in this order and for a reason. Read the configuration, because
//! the name this machine answers to and every screen below is a value out of
//! it. Read the clip name table, because an overlay names a motion and only
//! that file says which index a name has. Bind the reports port, because a
//! second host already reading it is the one failure that has to stop this one
//! before anything is asked of the machine. Install the stop flag. Start the
//! voice pipeline, where one was configured, with its motion seams pointed at
//! the gate this loop owns. Then run the loop.
//!
//! Every one of those failures is an exit and none of them is retried: a
//! configuration that does not parse, a name table that is not there, a port
//! somebody else holds, a speech configuration the pipeline will not run on.
//! A speech configuration that is not on the machine at all is not among them —
//! that is how a unit starts, and it runs the edge half and says so.
//! What supervises this process decides what happens next,
//! and a host that never comes back is a robot that is deaf and mute — never an
//! unsafe one. The motion stack owes the host nothing: the schedule already
//! running concludes at its own horizon, and the machine stows and rests there.
//!
//! What this prints is one JSON object per line on stdout: every row of the
//! session's story as it is narrated, every body the edge dropped, and every
//! alert the table raised. An alert also goes to the bus, through the seam the
//! composed pipeline drains onto the robot's one attachment; a host that
//! composed no pipeline has no attachment, and its alerts are those lines and
//! nothing else.

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
    Surface, edge_line_with, now,
};
use reachy_host::edge::{Console, Publishing};
use reachy_host::intents::{Intents, Waiting, waking_queue};
use reachy_host::params::{self, HostSettings};
use reachy_host::sinks::Stdout;
use reachy_host::voice::{NotRunning, Voice, absent_line, composed_line, silent_line};
use signal_hook::consts::{SIGINT, SIGTERM};
use signal_hook::flag;
use speech_surface::{ALERT_QUEUE_DEPTH, AlertRaiser, alert_seam};

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
    /// The speech pipeline's own configuration, where the voice half is to run.
    ///
    /// Optional, and so is the file it names: a host asked for neither, and a
    /// host asked for one that is not on the machine, both run the edge half
    /// alone, which is what narrates the session's story. The second is how a
    /// unit starts — the launcher entry names the path and the operator's own
    /// file is a push into RAM. Named separately from `--config` because the
    /// two are different formats owned by different repositories — this repo's
    /// textproto for the edge, the pod platform's TOML for the pipeline.
    speech_config: Option<PathBuf>,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            config: PathBuf::from(DEFAULT_CONFIG),
            speech_config: None,
        }
    }
}

/// How to invoke this, for a refusal to print.
fn usage() -> String {
    format!(
        "usage: reachy-host [--config PATH] [--speech-config PATH]\n\
         \n\
         Runs the robot's voice host: binds {REPORTS_OUT_PORT} on loopback, follows the\n\
         session's story, and sends compiled scripts to {SCRIPTS_IN_PORT}. One line of JSON\n\
         per row, on stdout.\n\
         \n\
         With --speech-config naming a file that is there, the voice pipeline runs in\n\
         this process too, and the scripter's decisions and the bus's motion channel both\n\
         meet the gate above. Without the flag, or with a path nothing has been pushed to\n\
         yet, the host runs its edge half alone and says which of the two it is.\n\
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
/// Two optional flags, each naming a path. A word this does not know is a
/// refusal rather than something ignored: a host run on the shipped
/// configuration when an operator meant a unit's own would answer to the wrong
/// pod name.
///
/// One of three copies of this argument-parsing shape; the driver and the
/// harness carry the others.
/// TODO(cli-argv-shared)
fn parse(args: impl Iterator<Item = String>) -> Result<Options, String> {
    let mut options = Options::default();
    let mut given = false;
    let mut speech_given = false;
    let mut args = args;
    while let Some(word) = args.next() {
        match word.as_str() {
            "--config" => {
                options.config = PathBuf::from(path_once(&word, args.next(), &mut given)?);
            }
            "--speech-config" => {
                let value = path_once(&word, args.next(), &mut speech_given)?;
                options.speech_config = Some(PathBuf::from(value));
            }
            other => return Err(format!("`{other}` is not an option this takes")),
        }
    }
    Ok(options)
}

/// A flag's path value, refused if the flag carried none or was given before.
///
/// The value is taken before the repeat is refused, so a repeated flag and a
/// repeated flag with no value read the same way to whoever wrote them.
fn path_once(flag: &str, value: Option<String>, given: &mut bool) -> Result<String, String> {
    let value = value.ok_or_else(|| format!("{flag} needs the path of a configuration"))?;
    if *given {
        return Err(format!("{flag} was given twice"));
    }
    *given = true;
    Ok(value)
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

    // The sending half is what an intent-authoring task holds: the scripter's
    // sink and the bus's motion subscription, both inside the pipeline the
    // voice half composes. With no speech configuration named nothing holds one
    // and the loop below narrates the session's story and asks for nothing.
    let (intents, waiting) = waking_queue(Arc::new(nudge));
    let mut host = HostEdge::new(settings.edge.clone(), table);
    let mut surface = Publishing::new(Console);
    surface.say(started_line(&settings, &options.config, now()));

    // Only a run that drains the seam gives the alerts anywhere to go, so the
    // raising end comes back only from one that does. Installed after the fact
    // because the surface exists before it: the host's first line is written
    // before there is anything to publish through.
    let started = start_voice(options, &intents, &mut surface)?;
    let voice = install_alerts(&mut surface, started);
    // The queue's other sending handle. Dropped here so that the only holders
    // are the pipeline's two sinks: the loop reads a disconnected queue the same
    // as an empty one, so a handle kept here would hide nothing, but a queue
    // whose senders are exactly the two authoring tasks says what this process
    // is by its own shape.
    drop(intents);

    let followed = follow(
        &reports,
        &scripts,
        (LOOPBACK, SCRIPTS_IN_PORT).into(),
        &waiting,
        &mut host,
        &mut surface,
        &stop,
    );

    // The pipeline is stopped after the loop returns, whichever way it went: a
    // loop that broke is still a process on its way out, and the pod link's
    // open segments finalize either way.
    let stopped = voice.and_then(Voice::stop);
    outcome(followed, stopped)
}

/// Put the raising end on the surface, and hand back the pipeline to stop.
///
/// The one place the two halves of the alert path are joined: the raiser a
/// composed run handed back is the raiser the surface publishes through. A
/// separate function because that join is otherwise unobservable — a host that
/// composed a pipeline and installed nothing looks exactly like one whose
/// deployment carries no alerts, and both of them run.
fn install_alerts<S: Surface>(
    surface: &mut Publishing<S>,
    started: Option<Started>,
) -> Option<Voice> {
    let started = started?;
    if let Some(raiser) = started.alerts {
        surface.publish_through(raiser);
    }
    Some(started.voice)
}

/// What the process exits on, from the loop's answer and the pipeline's.
///
/// The loop's failure leads when both failed: it is what ended the run, and the
/// pipeline's is the tidy-up behind it. But it is carried in the same sentence
/// rather than dropped, because a loop that broke because the machine itself is
/// unhealthy — descriptors exhausted, memory gone — is exactly the case where
/// the voice half's own death is part of the diagnosis, and nothing else in
/// this process would ever mention it.
fn outcome(followed: Result<(), String>, stopped: Option<String>) -> Result<(), String> {
    match (followed, stopped) {
        (Err(loop_error), Some(voice_error)) => Err(format!(
            "{loop_error} — and stopping the voice pipeline also failed: {voice_error}"
        )),
        (Err(loop_error), None) => Err(loop_error),
        (Ok(()), Some(voice_error)) => Err(voice_error),
        (Ok(()), None) => Ok(()),
    }
}

/// What a composed voice half left this process holding.
///
/// Two things rather than one, because they are held by different halves: the
/// pipeline is stopped by whoever ends the run, and the raiser belongs to the
/// surface the edge's loop narrates on. A host that composed no pipeline holds
/// neither, which is the absence of this whole value rather than a state
/// inside it.
struct Started {
    /// The running pipeline.
    voice: Voice,
    /// The raising end of the alert seam, where this run drains one. Absent on
    /// a pipeline that composed no bus attachment: the run drops its end, so
    /// every raise against it would refuse and say so for nothing.
    alerts: Option<AlertRaiser>,
}

/// Start the voice pipeline where one was configured, and say which it is.
///
/// The sinks it is composed with are handed the queue the loop takes from, so
/// the scripter's decision and a body off the bus reach one gate. The alert
/// seam runs the other way: the pipeline drains it onto the attachment it
/// holds. Nothing about the machine waits on any of this: a host with no
/// pipeline is deaf and mute, and the motion stack owes it nothing either way.
fn start_voice(
    options: &Options,
    intents: &Intents,
    surface: &mut impl Surface,
) -> Result<Option<Started>, String> {
    let Some(path) = &options.speech_config else {
        surface.say(silent_line(now()));
        return Ok(None);
    };
    // Minted here and handed on only once the pipeline is serving and drains
    // it: a raiser whose far end never ran, or whose run composed no bus
    // attachment to drain onto, would refuse every alert — a line per alert
    // saying nothing a host with no pipeline does not already say.
    let (raiser, inbox) = alert_seam(ALERT_QUEUE_DEPTH);
    // A named configuration that is not on the machine is the shipped state of
    // a unit: the launcher entry names the path unconditionally and an
    // operator's own file is a push into RAM, so until that push the file is
    // simply absent. Said and survived, unlike every other startup failure
    // here — a file that is there and will not run, or will not even read, is
    // still an exit, because then somebody pushed something this host cannot
    // answer for. Absence is the loader's own answer and not a `stat` asked
    // ahead of it, which cannot tell a file that is not there from one this
    // process may not look at.
    match Voice::start(path, intents.clone(), Arc::new(Stdout), inbox) {
        Ok(voice) => {
            let carries = voice.carries_alerts();
            surface.say(composed_line(path, &voice.listening(), carries, now()));
            Ok(Some(Started {
                voice,
                alerts: carries.then_some(raiser),
            }))
        }
        Err(NotRunning::Absent) => {
            surface.say(absent_line(path, now()));
            Ok(None)
        }
        Err(NotRunning::Refused(message)) => Err(message),
    }
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
    edge_line_with(
        "started",
        at,
        &format!(
            "the voice host answers for `{}`, configured by {}; reports on {REPORTS_OUT_PORT}, \
             scripts to {SCRIPTS_IN_PORT}",
            settings.edge.pod(),
            config.display(),
        ),
        &[("pod", serde_json::json!(settings.edge.pod()))],
    )
}

/// A compiled script that never left this machine, as a line.
fn unsent_line(script_id: u32, detail: &str, at: SyncTime) -> String {
    edge_line_with(
        "unsent",
        at,
        &format!(
            "script {script_id} compiled and could not be sent to the control process: {detail}. \
             nothing is retried; the sender's next refresh is what recovers"
        ),
        &[("script_id", serde_json::json!(script_id))],
    )
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
    use reachy_scratch::scratch_dir;

    use brenn_reachy__cogs__script_clk_rs::ScriptWire;

    use super::{
        DEFAULT_CONFIG, Options, Publishing, Started, alert_seam, follow, install_alerts, outcome,
        parse, start_voice, unsent_line,
    };

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

    /// Everything said, readable while something else holds the surface.
    ///
    /// The cases that install the alert seam hand their surface to
    /// [`Publishing`], which owns what it wraps and shows nobody, so what they
    /// read the lines through is a handle onto the same vector.
    #[derive(Clone, Debug, Default)]
    struct Shared {
        lines: Arc<std::sync::Mutex<Vec<String>>>,
    }

    impl Shared {
        /// Every line said so far.
        fn lines(&self) -> Vec<String> {
            self.lines.lock().expect("the recorded lines").clone()
        }
    }

    impl Surface for Shared {
        fn say(&mut self, line: String) {
            self.lines.lock().expect("the recorded lines").push(line);
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
                speech_config: None,
            }),
        );
    }

    #[test]
    fn a_configuration_can_be_named() {
        assert_eq!(
            parsed(&["--config", "/run/reachy/host_params.textproto"]),
            Ok(Options {
                config: PathBuf::from("/run/reachy/host_params.textproto"),
                speech_config: None,
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
        assert!(parsed(&["--speech-config"]).is_err());
        assert!(parsed(&["--speech-config", "a", "--speech-config", "b"]).is_err());
    }

    #[test]
    fn a_speech_configuration_is_what_makes_this_host_a_voice() {
        // Its absence is a host, not a refusal: the edge half runs on a unit
        // whose payload does not yet carry the pipeline's own inputs.
        assert_eq!(parsed(&[]).expect("no arguments").speech_config, None);
        assert_eq!(
            parsed(&["--speech-config", "/run/reachy/speech.toml"])
                .expect("a named speech configuration")
                .speech_config,
            Some(PathBuf::from("/run/reachy/speech.toml")),
        );
    }

    #[test]
    fn a_speech_configuration_nothing_has_been_pushed_to_is_a_running_host() {
        // A directory this case owns, so the absence it asserts on is one it
        // established rather than one it assumed of the machine it runs on.
        let dir = scratch_dir("reachy-host-start-voice");
        let path = dir.join("speech.toml");
        let (intents, _waiting) = queue();
        let mut said = Recorded::default();
        let options = Options {
            speech_config: Some(path.clone()),
            ..Options::default()
        };
        // `is_none()` rather than a match on the value: a running `Voice` owns
        // a runtime and is deliberately not `Debug`.
        let started = start_voice(&options, &intents, &mut said).expect("a running host");
        assert!(
            started.is_none(),
            "no pipeline was composed, so there is nothing to hold and nowhere to publish",
        );
        let line = said.lines.last().expect("a line saying which host this is");
        let parsed: serde_json::Value = serde_json::from_str(line).expect("one JSON object");
        assert_eq!(parsed["kind"], "awaiting_speech_config");
        assert!(
            parsed["says"]
                .as_str()
                .expect("a sentence")
                .contains(path.to_str().expect("a path this case wrote")),
            "{line}",
        );
    }

    #[test]
    fn a_speech_configuration_this_host_can_run_composes_the_pipeline() {
        let dir = scratch_dir("reachy-host-composed-arm");
        let path = runnable_speech_config(dir.as_ref());
        let (intents, _waiting) = queue();
        let mut said = Recorded::default();
        let options = Options {
            speech_config: Some(path.clone()),
            ..Options::default()
        };
        let started = start_voice(&options, &intents, &mut said)
            .expect("a running host")
            .expect("a composed pipeline");
        // The fixture configuration names no bus, so the composed run drops its
        // end of the alert seam and this host keeps no raising end.
        assert!(started.alerts.is_none(), "nowhere to publish, so no raiser");
        let voice = started.voice;
        let line = said.lines.last().expect("a line saying what was composed");
        let parsed: serde_json::Value = serde_json::from_str(line).expect("one JSON object");
        assert_eq!(parsed["kind"], "composed");
        assert_eq!(parsed["alerts"], false);
        let says = parsed["says"].as_str().expect("a sentence");
        let listening = voice.listening();
        assert!(says.contains(&listening), "{line}");
        assert!(!listening.ends_with(":0"), "the bound port, not `:0`");
        assert!(voice.stop().is_none(), "a pipeline asked to stop stops");
    }

    #[test]
    fn a_pipeline_that_carries_alerts_is_the_one_the_surface_publishes_through() {
        // The whole path in one case: a deployment whose run drains the seam,
        // the raiser it hands back, the install onto the surface the loop
        // narrates on, and an alert raised afterwards leaving without a word
        // saying it did not.
        let dir = scratch_dir("reachy-host-alert-install");
        let path = carrying_speech_config(dir.as_ref());
        let (intents, _waiting) = queue();
        let said = Shared::default();
        let mut surface = Publishing::new(said.clone());
        let options = Options {
            speech_config: Some(path),
            ..Options::default()
        };
        let started = start_voice(&options, &intents, &mut surface)
            .expect("a running host")
            .expect("a composed pipeline");
        assert!(
            started.alerts.is_some(),
            "a bus attachment is somewhere to publish, so the raiser comes back",
        );
        let composed: serde_json::Value =
            serde_json::from_str(&said.lines().pop().expect("a composed line"))
                .expect("one JSON object");
        assert_eq!(composed["kind"], "composed");
        assert_eq!(composed["alerts"], true);

        let voice = install_alerts(&mut surface, Some(started)).expect("the composed pipeline");
        surface.alert(&Alert {
            severity: reachy_edge::Severity::Critical,
            title: "the head is parked".to_owned(),
            body: "a fault row ended the session".to_owned(),
        });
        let after = said.lines();
        assert!(
            after.iter().all(|line| !line.contains("unpublished")),
            "the installed raiser took it: {after:?}",
        );
        assert!(voice.stop().is_none(), "a pipeline asked to stop stops");
    }

    #[test]
    fn a_pipeline_with_nowhere_to_publish_installs_nothing() {
        // The other arm of the same join: `start_voice` hands back no raiser,
        // so the surface stays narration-only and an alert raised on it is not
        // reported as one that failed to travel either.
        let dir = scratch_dir("reachy-host-alert-narration");
        let path = runnable_speech_config(dir.as_ref());
        let (intents, _waiting) = queue();
        let said = Shared::default();
        let mut surface = Publishing::new(said.clone());
        let options = Options {
            speech_config: Some(path),
            ..Options::default()
        };
        let started = start_voice(&options, &intents, &mut surface)
            .expect("a running host")
            .expect("a composed pipeline");
        assert!(started.alerts.is_none(), "nowhere to publish, so no raiser");

        let voice = install_alerts(&mut surface, Some(started)).expect("the composed pipeline");
        surface.alert(&Alert {
            severity: reachy_edge::Severity::Warning,
            title: "a script was refused".to_owned(),
            body: "the session declined it".to_owned(),
        });
        let after = said.lines();
        assert!(
            after.iter().all(|line| !line.contains("unpublished")),
            "an alert nobody could publish is not an unpublished one: {after:?}",
        );
        assert!(voice.stop().is_none(), "a pipeline asked to stop stops");
    }

    #[test]
    fn an_installed_raiser_is_the_one_an_alert_goes_through() {
        // Dropping the inbox before install means a raised alert is refused —
        // the "unpublished" line proves install_alerts wired this raiser.
        let dir = scratch_dir("reachy-host-alert-installed-raiser");
        let path = runnable_speech_config(dir.as_ref());
        let (intents, _waiting) = queue();
        let said = Shared::default();
        let mut surface = Publishing::new(said.clone());
        let options = Options {
            speech_config: Some(path),
            ..Options::default()
        };
        let voice = start_voice(&options, &intents, &mut surface)
            .expect("a running host")
            .expect("a composed pipeline")
            .voice;
        let (raiser, inbox) = alert_seam(1);
        drop(inbox);

        let voice = install_alerts(
            &mut surface,
            Some(Started {
                voice,
                alerts: Some(raiser),
            }),
        )
        .expect("the composed pipeline");
        surface.alert(&Alert {
            severity: reachy_edge::Severity::Critical,
            title: "the head is parked".to_owned(),
            body: "a fault row ended the session".to_owned(),
        });
        let lines = said.lines();
        let unpublished: Vec<serde_json::Value> = lines
            .iter()
            .map(|line| serde_json::from_str(line).expect("one JSON object"))
            .filter(|line: &serde_json::Value| line["kind"] == "unpublished")
            .collect();
        assert_eq!(unpublished.len(), 1, "{lines:?}");
        assert_eq!(unpublished[0]["title"], "the head is parked");
        assert_eq!(unpublished[0]["reason"], "gone");
        assert!(voice.stop().is_none(), "a pipeline asked to stop stops");
    }

    /// A speech configuration the pod platform will run, written into `dir`.
    ///
    /// Loopback on an ephemeral port, one key, no recording, no wake or
    /// endpointer table, so nothing here loads a model or touches a network.
    fn runnable_speech_config(dir: &std::path::Path) -> PathBuf {
        let keys = dir.join("psk.toml");
        speech_surface::psk::write_secret_file(
            &keys,
            "fixture-pod = \"00112233445566778899aabbccddeeff00112233445566778899aabbccddeeff\"\n",
        )
        .expect("a key table");
        let path = dir.join("speech.toml");
        std::fs::write(
            &path,
            format!(
                "listen_addr = \"127.0.0.1:0\"\n\
                 pod_psk_file = {:?}\n\
                 [record]\nenabled = false\n\
                 [jsonl]\nsink = \"none\"\n",
                keys.to_str().expect("a path this case wrote"),
            ),
        )
        .expect("a file");
        path
    }

    /// The same, for a deployment whose brain is on the bus.
    ///
    /// Everything `mode = "brenn"` requires, pointed at nobody: speech service
    /// URLs a composition that runs no turn never calls, and a bridge onto a
    /// closed loopback port with a token file this case wrote. What it buys is
    /// the run that drains the alert seam, which is the branch under test.
    fn carrying_speech_config(dir: &std::path::Path) -> PathBuf {
        let token = dir.join("bus.token");
        speech_surface::psk::write_secret_file(&token, "a-bearer-token\n").expect("a token file");
        let path = runnable_speech_config(dir);
        let text = std::fs::read_to_string(&path).expect("the fixture");
        std::fs::write(
            &path,
            format!(
                "{text}\
                 [brain]\nmode = \"brenn\"\n\
                 [stt]\nbackend = \"http\"\nurl = \"http://127.0.0.1:8000\"\nmodel = \"m\"\n\
                 [tts]\nbackend = \"http\"\nurl = \"http://127.0.0.1:8000\"\n\
                 model = \"m\"\nvoice = \"v\"\n\
                 [brenn]\n\
                 publish_channel = \"brenn:pod.utterance\"\n\
                 response_channel = \"brenn:pod.speak\"\n\
                 [brenn.bridge]\n\
                 server_url = \"wss://127.0.0.1:1/ws\"\n\
                 token_file = {:?}\n",
                token.to_str().expect("a path this case wrote"),
            ),
        )
        .expect("a file");
        path
    }

    #[test]
    fn the_loop_s_failure_is_the_one_the_process_exits_on() {
        let both = outcome(
            Err("reading the reports port: bad file descriptor".to_owned()),
            Some("the voice pipeline's task did not finish: panic".to_owned()),
        )
        .expect_err("two failures are a failure");
        assert!(both.starts_with("reading the reports port"), "{both}");
        assert!(both.contains("did not finish: panic"), "{both}");
    }

    #[test]
    fn a_clean_loop_still_exits_on_a_pipeline_that_would_not_stop() {
        assert_eq!(
            outcome(
                Ok(()),
                Some("the voice pipeline stopped on an error: x".to_owned())
            ),
            Err("the voice pipeline stopped on an error: x".to_owned()),
        );
        assert_eq!(outcome(Ok(()), None), Ok(()));
        assert_eq!(
            outcome(Err("reading the reports port: x".to_owned()), None),
            Err("reading the reports port: x".to_owned()),
        );
    }

    #[test]
    fn a_host_asked_for_no_speech_configuration_says_that_instead() {
        let (intents, _waiting) = queue();
        let mut said = Recorded::default();
        let started =
            start_voice(&Options::default(), &intents, &mut said).expect("a running host");
        assert!(
            started.is_none(),
            "no pipeline was composed, and nothing to publish through",
        );
        let line = said.lines.last().expect("a line");
        let parsed: serde_json::Value = serde_json::from_str(line).expect("one JSON object");
        assert_eq!(parsed["kind"], "voiceless");
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
