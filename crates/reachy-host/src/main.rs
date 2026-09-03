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
    DATAGRAM_CAP, HostEdge, LOOPBACK, MotionTable, Origin, POLL, REPORTS_OUT_PORT, SCRIPTS_IN_PORT,
    Surface, edge_line_with, now, origin_word,
};
use reachy_host::check;
use reachy_host::edge::{Console, Publishing, Speaker};
use reachy_host::intents::{Intents, Waiting, waking_queue};
use reachy_host::params::{self, HostSettings};
use reachy_host::sinks::Stdout;
use reachy_host::voice::{NotRunning, Voice, absent_line, composed_line, silent_line};
use signal_hook::consts::{SIGINT, SIGTERM};
use signal_hook::flag;
use speech_surface::{ALERT_QUEUE_DEPTH, AlertRaiser, alert_seam};

/// Where the configuration is read from unless `--config` says otherwise.
///
/// The path the payload lays the operator's configuration at, relative to the
/// working directory the launcher starts this process in. The file does not
/// travel with the binary: it is a per-unit file pushed into RAM, so a host
/// started anywhere else is a host told where to look.
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
    /// Load both configurations, look for every file they name, and exit —
    /// without binding a port, starting a pipeline or touching a robot.
    ///
    /// What the deploy step runs over a staged payload, with that payload's
    /// root as the working directory, so the relative paths the unit will
    /// resolve are the ones resolved here.
    check: bool,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            config: PathBuf::from(DEFAULT_CONFIG),
            speech_config: None,
            check: false,
        }
    }
}

/// How to invoke this, for a refusal to print.
fn usage() -> String {
    format!(
        "usage: reachy-host [--config PATH] [--speech-config PATH] [--check]\n\
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
         With --check the process instead loads both configurations, looks for every file\n\
         they name relative to the working directory, prints one line of JSON per\n\
         conclusion and exits: zero when everything loaded and every file is there. It\n\
         binds nothing and prints no file's contents.\n\
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
        Ok(options) if options.check => checked(&options),
        Ok(options) => match run(&options) {
            Ok(()) => ExitCode::SUCCESS,
            Err(message) => {
                eprintln!("{}{message}", reachy_host::REFUSAL_PREFIX);
                ExitCode::FAILURE
            }
        },
        Err(message) => {
            eprintln!("{}{message}\n\n{}", reachy_host::REFUSAL_PREFIX, usage());
            ExitCode::FAILURE
        }
    }
}

/// What the invocation asks for.
///
/// Three optional flags, two of them naming a path. A word this does not know is a
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
            "--check" => {
                if options.check {
                    return Err("--check was given twice".to_owned());
                }
                options.check = true;
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

/// Say what both configurations would load, and exit on whether they would.
///
/// The empty base is what makes this the run's own question: a relative path
/// joins to itself and so resolves against this process's working directory,
/// which the deploy step sets to the staged payload's root — the directory the
/// launcher will run the host from on the unit.
fn checked(options: &Options) -> ExitCode {
    if write_check(&mut io::stdout().lock(), options, Path::new("")) {
        ExitCode::SUCCESS
    } else {
        ExitCode::FAILURE
    }
}

/// The conclusions onto `out`, and whether the run could start on them.
///
/// Written to a handed-in sink and given the base as an argument, because what
/// this decides is a gate: the deploy step refuses a speech run on this exit
/// status, so an inverted answer is a host that exits at start on a unit with a
/// person in front of it. A case can hold both halves of that here — the status
/// and the one line of JSON per conclusion an operator and the fetched log both
/// read — without a process and without a working directory of its own.
///
/// # Panics
///
/// If the sink cannot be written, which for the caller's stdout is a console
/// that has gone away mid-preflight.
fn write_check(out: &mut impl io::Write, options: &Options, base: &Path) -> bool {
    let found = check::inspect(&options.config, options.speech_config.as_deref(), base);
    let at = now();
    for conclusion in &found {
        writeln!(out, "{}", check::conclusion_line(conclusion, at)).expect("a writable stream");
    }
    check::settled(&found)
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
        &until(&stop, &voice),
    );

    // The pipeline is stopped after the loop returns, whichever way it went: a
    // loop that broke is still a process on its way out, and the pod link's
    // open segments finalize either way.
    let stopped = voice.and_then(Voice::stop);
    outcome(followed, stopped)
}

/// Put the raising and speaking ends on the surface, and hand back the pipeline
/// to stop.
///
/// The one place the alert path's halves are joined: the raiser a composed run
/// handed back is the raiser the surface publishes through, and the seam that
/// run speaks on is what the surface says a Critical's sentence through. A
/// separate function because that join is otherwise unobservable — a host that
/// composed a pipeline and installed nothing looks exactly like one whose
/// deployment carries no alerts and cannot speak, and both of them run.
fn install_alerts<S: Surface>(
    surface: &mut Publishing<S>,
    started: Option<Started>,
) -> Option<Voice> {
    let started = started?;
    if let Some(raiser) = started.alerts {
        surface.publish_through(raiser);
    }
    if let Some(speaker) = started.speaker {
        surface.speak_through(speaker);
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
    /// What the robot says a critical alert's sentence through, where this run
    /// can speak. Absent on a pipeline that composed no brain or no voice, on
    /// the same grounds the raiser is.
    speaker: Option<Box<dyn Speaker>>,
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
            let speaker = voice.speaker();
            surface.say(composed_line(
                path,
                &voice.listening(),
                voice.composition(),
                now(),
            ));
            Ok(Some(Started {
                voice,
                alerts: carries.then_some(raiser),
                speaker,
            }))
        }
        Err(NotRunning::Absent) => {
            surface.say(absent_line(path, now()));
            Ok(None)
        }
        Err(NotRunning::Refused(message)) => Err(message),
    }
}

/// Whether this run's voice half has ended without being asked to.
///
/// A host that composed no pipeline has nothing that can end, so this answers
/// false for its whole run and the edge half serves on. `map_or(true, …)`
/// would end every voiceless host's loop on its first pass.
fn voice_ended(voice: &Option<Voice>) -> bool {
    voice.as_ref().is_some_and(Voice::pipeline_ended)
}

/// Why the loop below would end.
///
/// Two conditions, asked together every pass, because they are one question:
/// whether this process still has anything to do.
struct Until<'a> {
    /// The flag the signal handlers set.
    stop: &'a AtomicBool,
    /// Whether the voice pipeline has ended without being asked to.
    ///
    /// A predicate rather than a `Voice` read here, because the case that
    /// matters is not otherwise observable: a loop that never asks, and a loop
    /// that asks a host with no pipeline and reads its absence as a death, both
    /// look exactly like the working one from the outside.
    ended: Box<dyn Fn() -> bool + 'a>,
}

/// The pair of conditions this run's loop ends on, as `run` builds them.
///
/// A loop and a predicate that are each right separately still leave a host
/// narrating around a dead voice half if the two are not joined.
fn until<'a>(stop: &'a AtomicBool, voice: &'a Option<Voice>) -> Until<'a> {
    Until {
        stop,
        ended: Box::new(move || voice_ended(voice)),
    }
}

impl Until<'_> {
    /// Whether either of them has come about.
    fn reached(&self) -> bool {
        self.stop.load(Ordering::Relaxed) || (self.ended)()
    }
}

/// The loop: a story datagram, then whatever intent is waiting, until stopped.
///
/// Two ways out, and the second is why this process ends when its voice half
/// does. A stop signal is one. The other is the pipeline having ended without
/// being asked to: a host that went on narrating around a dead voice half would
/// be a robot that hears nothing and looks alive — the shape that costs an
/// operator a whole session before anybody reads the log. Ending here puts the
/// pipeline's own sentence on the caller's exit status within one read timeout
/// of the death.
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
    until: &Until,
) -> Result<(), String> {
    let mut buffer = vec![0u8; DATAGRAM_CAP];
    while !until.reached() {
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

        while let Some(offered) = waiting.next() {
            let arrival = now();
            if let Some(accepted) = host.offer(&offered.body, offered.origin, arrival, surface) {
                // Fire and forget: the session narrates what it decided with
                // the script, and a send that failed is said here because
                // nothing else would ever mention it.
                if let Err(error) = scripts.send_to(accepted.bytes(), scripts_to) {
                    surface.say(unsent_line(
                        accepted.script_id,
                        offered.origin,
                        &error.to_string(),
                        arrival,
                    ));
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
        reachy_host::STARTED,
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
///
/// Carries the origin the body was offered under, because this host sends every
/// script its gate accepted and a send that failed does not say whose gesture
/// was lost: a reader counting what this machine did to its own scripts needs
/// the same word a refusal carries.
fn unsent_line(script_id: u32, origin: Origin, detail: &str, at: SyncTime) -> String {
    edge_line_with(
        reachy_host::UNSENT,
        at,
        &format!(
            "script {script_id} compiled and could not be sent to the control process: {detail}. \
             nothing is retried; the sender's next refresh is what recovers"
        ),
        &[
            ("script_id", serde_json::json!(script_id)),
            ("origin", serde_json::json!(origin_word(origin))),
        ],
    )
}

#[cfg(test)]
mod tests {
    use std::net::{Ipv6Addr, SocketAddr, UdpSocket};
    use std::path::PathBuf;
    use std::sync::Arc;
    use std::sync::atomic::{AtomicBool, Ordering};
    use std::thread;
    use std::time::{Duration, Instant};

    use clockwork_rs::{SyncTime, blob_from_bytes};
    use motion_proto::{MotionScript, Posture, Step};
    use reachy_edge::{Alert, EdgeConfig, HostEdge, LOOPBACK, MotionTable, Origin, Surface};
    use reachy_host::intents::queue;
    use reachy_scratch::scratch_dir;

    use brenn_reachy__cogs__script_clk_rs::ScriptWire;

    use super::{
        DEFAULT_CONFIG, Options, Publishing, Started, Until, Voice, alert_seam, follow,
        install_alerts, outcome, parse, start_voice, unsent_line, until, voice_ended,
    };

    /// The pod the fixture bodies are addressed to.
    const POD: &str = "fixture-reachy";

    /// How long a read waits in the cases below. Short, because every datagram
    /// they read was already queued before the loop started: the timeout is
    /// only what the loop wakes on to notice the stop.
    const READ: Duration = Duration::from_millis(20);

    /// How long a loop whose only exit condition already holds is given to
    /// return. Orders of magnitude past one read timeout, because what it
    /// bounds is a case that would otherwise never end rather than a duration
    /// worth measuring.
    const ENDS_WITHIN: Duration = Duration::from_secs(10);

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

    /// Every sentence a surface was asked to say out loud.
    ///
    /// The room's end of the spoken path, stood in for: the composed pipeline's
    /// own seam is what production installs, and what these cases have to see
    /// is that something was installed at all.
    #[derive(Clone, Debug, Default)]
    struct Heard {
        said: Arc<std::sync::Mutex<Vec<String>>>,
    }

    impl reachy_host::Speaker for Heard {
        fn speak(&self, sentence: &str) -> Result<(), reachy_host::Unspoken> {
            self.said
                .lock()
                .expect("the recorded sentences")
                .push(sentence.to_owned());
            Ok(())
        }
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
        driven(reports, scripts_to, bodies, Box::new(|| false))
    }

    /// The same, with the pipeline-ended predicate the loop also exits on.
    fn driven(
        reports: &UdpSocket,
        scripts_to: SocketAddr,
        bodies: &[Vec<u8>],
        ended: Box<dyn Fn() -> bool + '_>,
    ) -> Recorded {
        let (intents, waiting) = queue();
        for body in bodies {
            intents
                .offer(body.clone(), Origin::Local)
                .expect("a queue with room");
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
            &Until { stop: &stop, ended },
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
        assert_eq!(
            line["origin"], "local",
            "the loop offered this body itself, so the line says whose script was lost",
        );
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
            &Until {
                stop: &stop,
                ended: Box::new(|| false),
            },
        )
        .expect("a stopped loop is not a failed one");
        assert!(surface.lines.is_empty(), "{:?}", surface.lines);
    }

    #[test]
    fn a_pipeline_that_ended_ends_the_loop_with_no_stop() {
        // The condition this exists for: nothing signalled the host, and the
        // voice half is gone. A loop that stayed here is the host sitting
        // alive-and-deaf until somebody presses ^C.
        //
        // On its own thread and joined against a deadline, because *returning*
        // is the behaviour: nothing ever stops this loop, so a regression in
        // the exit condition runs forever, and a case that proved itself by not
        // hanging would spend that regression as a wedged test binary with no
        // sentence in it.
        let reports = reports_port();
        let following = thread::spawn(move || {
            let scripts = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
            let (_intents, waiting) = queue();
            let mut host = HostEdge::new(EdgeConfig::for_pod(POD), MotionTable::default());
            let mut surface = Recorded::default();
            let stop = AtomicBool::new(false);
            follow(
                &reports,
                &scripts,
                unreachable_destination(),
                &waiting,
                &mut host,
                &mut surface,
                &Until {
                    stop: &stop,
                    ended: Box::new(|| true),
                },
            )
            .expect("a loop that ended on the pipeline is not a broken one");
            surface
        });
        let deadline = Instant::now() + ENDS_WITHIN;
        while !following.is_finished() {
            assert!(
                Instant::now() < deadline,
                "the loop did not end within {ENDS_WITHIN:?} on a pipeline that is gone, and \
                 nothing else will ever end it",
            );
            thread::sleep(Duration::from_millis(1));
        }
        let surface = following.join().expect("the following thread");
        assert!(surface.lines.is_empty(), "{:?}", surface.lines);
    }

    #[test]
    fn a_host_with_no_pipeline_keeps_serving_the_edge() {
        // The conditions `run` builds, against the host that composed no voice
        // half: a regression here takes the narrating edge away from a unit
        // whose speech configuration has simply not been pushed yet.
        let voiceless: Option<Voice> = None;
        let quiet = AtomicBool::new(false);
        assert!(
            !until(&quiet, &voiceless).reached(),
            "no pipeline is not an ended pipeline",
        );

        let reports = reports_port();
        let control = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
        control
            .set_read_timeout(Some(READ * 10))
            .expect("a read timeout on a bound socket");
        let destination = control.local_addr().expect("a bound port");

        // The loop ran a pass and did the seam's work before the stop reached
        // it, which is what makes this the voiceless host still serving rather
        // than a loop that exited early and said nothing.
        let said = driven(
            &reports,
            destination,
            &[body(1)],
            Box::new(|| voice_ended(&voiceless)),
        );
        assert!(said.lines.is_empty(), "{:?}", said.lines);
        let mut buffer = vec![0u8; 4096];
        control
            .recv_from(&mut buffer)
            .expect("the body left as a script, so the loop iterated");
    }

    #[test]
    fn a_serving_pipeline_and_a_stop_are_a_clean_exit() {
        // The orderly path: a pipeline that is actually serving answers false
        // throughout, the stop ends the loop, and the stop of the pipeline
        // itself adds nothing to the outcome.
        let dir = scratch_dir("reachy-host-follow-serving");
        let path = runnable_speech_config(dir.as_ref());
        let (intents, _waiting) = queue();
        let mut said = Recorded::default();
        let options = Options {
            speech_config: Some(path),
            ..Options::default()
        };
        let voice = Some(
            start_voice(&options, &intents, &mut said)
                .expect("a running host")
                .expect("a composed pipeline")
                .voice,
        );
        let stop = AtomicBool::new(false);
        assert!(
            !until(&stop, &voice).reached(),
            "a pipeline that is serving has not ended",
        );
        stop.store(true, Ordering::Relaxed);
        assert!(
            until(&stop, &voice).reached(),
            "the same conditions still end on a stop",
        );

        let reports = reports_port();
        let followed = driven(
            &reports,
            unreachable_destination(),
            &[],
            Box::new(|| voice_ended(&voice)),
        );
        assert!(followed.lines.is_empty(), "{:?}", followed.lines);
        assert_eq!(outcome(Ok(()), voice.and_then(Voice::stop)), Ok(()));
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
                check: false,
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
                check: false,
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
    fn the_check_flag_is_asked_for_and_asked_for_once() {
        // The flag that makes this a preflight instead of a host: no path, and
        // a run that never binds a port.
        assert!(!parsed(&[]).expect("no arguments").check);
        assert!(
            parsed(&["--check", "--speech-config", "host/speech.toml"])
                .expect("a preflight over a named speech configuration")
                .check
        );
        let refused = parsed(&["--check", "--check"]).expect_err("a repeated flag");
        assert!(refused.contains("--check"), "{refused}");
    }

    /// A host configuration this build reads, in `dir`, with its clip table.
    ///
    /// Named relative to `dir`, which is the base the cases below hand the
    /// check: what a payload's own configuration names, resolved the way the
    /// unit resolves it.
    fn checkable(dir: &std::path::Path) -> PathBuf {
        let path = dir.join("host_params.textproto");
        std::fs::write(
            &path,
            "pod: \"fixture-reachy\"\n\
             stow_duration_ms: 3000\n\
             body_cap_bytes: 8192\n\
             clip_names_path: \"clip_library.names.json\"\n",
        )
        .expect("a file");
        std::fs::write(dir.join("clip_library.names.json"), "{\"names\": []}\n").expect("a file");
        path
    }

    /// Every line `write_check` printed for these options, over this base.
    fn checked_lines(options: &Options, base: &std::path::Path) -> (bool, Vec<String>) {
        let mut out: Vec<u8> = Vec::new();
        let settled = super::write_check(&mut out, options, base);
        let text = String::from_utf8(out).expect("the conclusions are text");
        (
            settled,
            text.lines().map(std::borrow::ToOwned::to_owned).collect(),
        )
    }

    #[test]
    fn a_payload_carrying_everything_its_configurations_name_checks_out() {
        // The gate the deploy step refuses a speech run on. An inverted answer
        // here is a host that exits at start on a unit with a person in front
        // of it, so the status and what it printed are both the case's.
        let dir = scratch_dir("reachy-host-checked-clean");
        let config = checkable(dir.as_ref());
        let speech = speech_fixture::carrying_named(
            dir.as_ref(),
            speech_fixture::Events::Dropped,
            speech_fixture::Naming::PayloadRelative,
        );
        let options = Options {
            config,
            speech_config: Some(speech),
            check: true,
        };

        let (settled, lines) = checked_lines(&options, dir.as_ref());
        assert!(settled, "{lines:?}");
        assert!(lines.len() > 1, "{lines:?}");
        for line in &lines {
            let object: serde_json::Value =
                serde_json::from_str(line).unwrap_or_else(|_| panic!("one line of JSON: {line}"));
            assert_eq!(object["stream"], "check", "{line}");
            assert_eq!(object["held"], true, "{line}");
        }
        let verdict: serde_json::Value =
            serde_json::from_str(lines.last().expect("a verdict")).expect("JSON");
        assert_eq!(verdict["kind"], "checked");
    }

    #[test]
    fn a_payload_missing_a_file_its_configuration_names_does_not() {
        let dir = scratch_dir("reachy-host-checked-missing");
        let config = checkable(dir.as_ref());
        let speech = speech_fixture::carrying_named(
            dir.as_ref(),
            speech_fixture::Events::Dropped,
            speech_fixture::Naming::PayloadRelative,
        );
        std::fs::remove_file(dir.join("bus.token")).expect("the fixture's token file");
        let options = Options {
            config,
            speech_config: Some(speech),
            check: true,
        };

        let (settled, lines) = checked_lines(&options, dir.as_ref());
        assert!(!settled, "{lines:?}");
        let verdict: serde_json::Value =
            serde_json::from_str(lines.last().expect("a verdict")).expect("JSON");
        assert_eq!(verdict["kind"], "checked");
        assert_eq!(verdict["held"], false);
        assert!(
            verdict["says"]
                .as_str()
                .expect("a sentence")
                .contains("brenn.bridge.token_file"),
            "{verdict}",
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
        assert!(
            started.speaker.is_none(),
            "no brain and no voice, so nothing to speak through",
        );
        let voice = started.voice;
        let line = said.lines.last().expect("a line saying what was composed");
        let parsed: serde_json::Value = serde_json::from_str(line).expect("one JSON object");
        assert_eq!(parsed["kind"], "composed");
        assert_eq!(parsed["alerts"], false);
        assert_eq!(parsed["speaks"], false);
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
        assert_eq!(composed["speaks"], true);
        assert!(
            started.speaker.is_some(),
            "a brain and a voice, so the seam to speak through comes back",
        );

        let voice = install_alerts(&mut surface, Some(started)).expect("the composed pipeline");
        surface.alert(&Alert {
            severity: reachy_edge::Severity::Critical,
            title: "the head is parked".to_owned(),
            body: "a fault row ended the session".to_owned(),
            spoken: Some("My head motion has stopped.".to_owned()),
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
            spoken: None,
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
                speaker: None,
            }),
        )
        .expect("the composed pipeline");
        surface.alert(&Alert {
            severity: reachy_edge::Severity::Critical,
            title: "the head is parked".to_owned(),
            body: "a fault row ended the session".to_owned(),
            spoken: Some("My head motion has stopped.".to_owned()),
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

    #[test]
    fn an_installed_speaker_is_the_one_a_sentence_goes_through() {
        // The speaking half of the same join, and the only thing that can see
        // it: a host that composed a pipeline and installed no speaker says
        // nothing out loud and writes no line saying so, so the sentence
        // arriving at a double is the whole of the evidence.
        let dir = scratch_dir("reachy-host-alert-installed-speaker");
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
        let heard = Heard::default();

        let voice = install_alerts(
            &mut surface,
            Some(Started {
                voice,
                alerts: None,
                speaker: Some(Box::new(heard.clone())),
            }),
        )
        .expect("the composed pipeline");
        surface.alert(&Alert {
            severity: reachy_edge::Severity::Critical,
            title: "the head is parked".to_owned(),
            body: "a fault row ended the session".to_owned(),
            spoken: Some("My head motion has stopped.".to_owned()),
        });
        surface.alert(&Alert {
            severity: reachy_edge::Severity::Warning,
            title: "a script was refused".to_owned(),
            body: "the session declined it".to_owned(),
            spoken: None,
        });

        assert_eq!(
            heard.said.lock().expect("the recorded sentences").clone(),
            vec!["My head motion has stopped.".to_owned()],
            "the critical's sentence reached the installed speaker, and the warning's silence \
             stayed silent",
        );
        let lines = said.lines();
        assert!(
            lines.iter().all(|line| !line.contains("unspoken")),
            "the installed speaker took it: {lines:?}",
        );
        assert!(voice.stop().is_none(), "a pipeline asked to stop stops");
    }

    /// The fixture, with the event stream dropped: nothing here reads it back.
    fn runnable_speech_config(dir: &std::path::Path) -> PathBuf {
        speech_fixture::runnable(dir, speech_fixture::Events::Dropped)
    }

    /// The same, for a deployment whose brain is on the bus.
    fn carrying_speech_config(dir: &std::path::Path) -> PathBuf {
        speech_fixture::carrying(dir, speech_fixture::Events::Dropped)
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
        let line = unsent_line(7, Origin::Remote, "no route to host", at);
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("one JSON object");
        assert_eq!(parsed["kind"], "unsent");
        assert_eq!(parsed["at_ns"], at.as_nanos());
        assert_eq!(parsed["script_id"], 7);
        assert_eq!(
            parsed["origin"], "remote",
            "this host sends a bus sender's accepted script too, and says so",
        );
        assert!(
            parsed["says"]
                .as_str()
                .expect("a sentence")
                .contains("no route"),
            "{line}"
        );
    }
}
