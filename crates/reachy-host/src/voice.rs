//! The voice pipeline, composed: the pod platform's server running in this
//! process with its two motion seams filled by the edge.
//!
//! What is composed here is a library of the fleet's platform, not a fork of
//! it. Every pod runs this same server — wake word, endpointing, the turn
//! lifecycle, the STT and TTS clients, the scripter, and the one Brenn-bus
//! attachment. What the robot adds is where it runs and what its motion seams
//! are pointed at: the scripter's decisions go to the gate on the other side of
//! this module instead of onto a bus, and a body a remote sender put on the
//! bus's motion channel arrives at the same gate.
//!
//! Two schedulers share this process on purpose. The pipeline is network I/O
//! and belongs on an async runtime; the edge's loop owns two loopback sockets
//! and a story it narrates in order, and it stays a plain blocking loop on the
//! thread that started the process. They meet at two bounded queues and nowhere
//! else — bodies inbound to the gate, alerts outbound to the attachment — so
//! neither can park the other.
//!
//! Everything here is optional at run time. A unit whose payload does not yet
//! carry a speech configuration runs the edge half alone: it follows the
//! session's story, narrates it, and accepts intent from anything already in
//! the process — which is nothing, so the robot is silent and still rather than
//! broken.

use std::path::Path;
use std::sync::Arc;
use std::time::Duration;

use clockwork_rs::SyncTime;
use reachy_edge::{edge_line, edge_line_with};
use serde_json::json;
use speech_surface::server::Server;
use speech_surface::{AlertInbox, Config, ConfigError, Sinks, jsonl};
use tokio::runtime::Runtime;
use tokio::sync::oneshot;

use crate::intents::Intents;
use crate::sinks::{BusIntents, Lines, ScripterIntents};
use crate::words;

/// The composed pipeline, running.
///
/// Held by the thread that owns the edge loop. Dropping it without
/// [`Voice::stop`] leaves the runtime to shut its tasks down abruptly, which is
/// what a panicking host wants and not what a stopping one does.
pub struct Voice {
    runtime: Runtime,
    stop: Option<oneshot::Sender<()>>,
    serving: Option<tokio::task::JoinHandle<std::io::Result<()>>>,
    listening: String,
    carries_alerts: bool,
}

/// How long a stop waits for the composed server to drain before it stops
/// waiting.
///
/// The drain is graceful by the platform's definition — open segments finalize,
/// turns in flight end, logs flush — and every one of those steps can be held
/// up by something off this machine: an HTTP call to a speech service that
/// never answers, a pod that never closes its connection. Unbounded, this wait
/// is a host that ignores every further signal, because the stop flag the
/// handlers set has already been read; the operator's only remaining tool is
/// `SIGKILL`. Nothing about the machine rides on the drain finishing, so a
/// deadline costs tidiness and buys a process that always ends.
const STOP_DEADLINE: Duration = Duration::from_secs(10);

/// How long the runtime is then given to put its own threads down.
///
/// Asked of the runtime rather than left to its drop for the same reason the
/// wait above is bounded: dropping a runtime waits for blocking tasks with no
/// deadline at all.
const SHUTDOWN_GRACE: Duration = Duration::from_secs(1);

/// Why a speech configuration did not become a running pipeline.
///
/// Two outcomes rather than one sentence, because the caller answers them
/// differently: a file that is not on the machine is how every unit starts and
/// is survivable, and everything else — including a file that is there and
/// cannot be read — is a host that must not pretend to have a voice.
#[derive(Debug)]
pub enum NotRunning {
    /// Nothing is at that path.
    Absent,
    /// Anything else, as the sentence an operator reads.
    Refused(String),
}

impl std::fmt::Display for NotRunning {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Absent => f.write_str("no file is at that path"),
            Self::Refused(message) => f.write_str(message),
        }
    }
}

impl Voice {
    /// Load `config`, compose the server over `intents` and `alerts`, and start
    /// serving.
    ///
    /// `alerts` is the server's end of the operator-alert seam: the robot's one
    /// bus attachment is held inside this run, and this is how something the
    /// edge's table raised on the blocking loop's thread reaches it. A start
    /// that fails drops it, so the raiser its caller kept refuses every alert
    /// rather than appearing to carry one.
    ///
    /// The runtime is this process's own and is built here rather than wrapped
    /// around `main`: the thread that calls this goes on to run the edge's
    /// blocking loop, and a blocking loop on a runtime's own thread is the one
    /// shape that deadlocks a runtime.
    ///
    /// # Errors
    ///
    /// [`NotRunning::Absent`] where no file is at `config`, which is what the
    /// caller survives. Otherwise [`NotRunning::Refused`] with the sentence: a
    /// configuration that cannot be read for a reason that is not absence, one
    /// that does not parse or does not validate, a JSONL sink that cannot be
    /// opened, and a listener or key table the server refuses. None of them is
    /// retried: a voice host that came up without keys would refuse its own
    /// audio device while looking healthy.
    ///
    /// Absence is read off the load's own error rather than asked separately:
    /// a `stat` in front of the read answers `false` for a file that is there
    /// under an ownership this process cannot follow, and telling an operator
    /// to push the file they just pushed is the expensive misdiagnosis.
    pub fn start(
        config: &Path,
        intents: Intents,
        lines: Arc<dyn Lines>,
        alerts: AlertInbox,
    ) -> Result<Self, NotRunning> {
        let settings = Config::load(config).map_err(|error| match &error {
            ConfigError::Read { source, .. } if source.kind() == std::io::ErrorKind::NotFound => {
                NotRunning::Absent
            }
            // Apart from the catch-all: this is the one an operator can act on
            // without a second question — the file is theirs and the message
            // names what about it this host refuses.
            ConfigError::Invalid { message, .. } => NotRunning::Refused(format!(
                "the speech configuration {} is not one this host can run: {message}",
                config.display()
            )),
            _ => NotRunning::Refused(format!(
                "loading the speech configuration {}: {error}",
                config.display()
            )),
        })?;
        // The server's own drain condition, asked of the crate that spawns the
        // drain rather than re-spelled here: a pinned revision that narrows or
        // widens it moves this answer with it.
        let carries_alerts = settings.carries_alerts();
        let settings = Arc::new(settings);

        let runtime = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .map_err(|error| {
                NotRunning::Refused(format!(
                    "building the runtime the voice pipeline runs on: {error}"
                ))
            })?;

        let sinks = Sinks {
            scripts: Some(Arc::new(ScripterIntents::new(intents.clone(), lines))),
            intents: Some(Arc::new(BusIntents::new(intents))),
        };

        let (stop, stopped) = oneshot::channel();
        let (serving, listening) = runtime
            .block_on(async move {
                let (handle, tasks) = jsonl::spawn(
                    &settings.jsonl.sink,
                    tokio::io::stderr(),
                    std::io::IsTerminal::is_terminal(&std::io::stderr()),
                )
                .await
                .map_err(|error| format!("opening the speech pipeline's event sink: {error}"))?;

                let server = Server::bind(Arc::clone(&settings), handle.clone())
                    .await
                    .map_err(|error| {
                        format!("binding the pod link at {}: {error}", settings.listen_addr)
                    })?
                    .with_sinks(sinks)
                    .with_alerts(alerts);
                // Asked of the listener rather than read off the configuration: an
                // ephemeral port is resolved only once it is bound, and a line
                // naming `:0` would answer nobody's question.
                let listening = server
                    .local_addr()
                    .map_err(|error| {
                        format!("asking the pod link which address it bound: {error}")
                    })?
                    .to_string();

                // What this task answers with is the server's run, and the flush
                // that follows it is inside the task rather than at the call site:
                // the JSONL sinks must drain after the server stops and before the
                // runtime goes away, and this body is the only place that ordering
                // can be stated.
                let serving = tokio::spawn(async move {
                    // A stop that never arrives because the sender was dropped is a
                    // stop: the thread holding it has gone, and this process is on
                    // its way out either way.
                    let served = server.run(async { stopped.await.unwrap_or(()) }).await;
                    // The emit handle has to go before the writers see their
                    // channels close, and the writers have to drain before the
                    // runtime does.
                    drop(handle);
                    tasks.join().await;
                    served
                });
                Ok::<_, String>((serving, listening))
            })
            .map_err(NotRunning::Refused)?;

        Ok(Self {
            runtime,
            stop: Some(stop),
            serving: Some(serving),
            listening,
            carries_alerts,
        })
    }

    /// Whether an alert raised on this run's seam reaches the bus.
    ///
    /// False on every configuration that composed no attachment, where the run
    /// drops its end of the seam: a raiser kept against one refuses each alert,
    /// which is a line per alert saying nothing about the machine. The caller
    /// keeps the raising end only when this is true.
    #[must_use]
    pub const fn carries_alerts(&self) -> bool {
        self.carries_alerts
    }

    /// Whether the composed pipeline's task has ended.
    ///
    /// A pipeline that ended without being asked to — a fault it latched, a
    /// task that panicked, a stage that died under it — is a process that has
    /// stopped listening, and a host that goes on narrating the motion edge
    /// around it is a robot nobody watching can tell is deaf. Answering true is
    /// how the loop is ended so that the stop below reaps the sentence and the
    /// process exits on it.
    ///
    /// The handle is an `Option` only because [`Voice::stop`] takes it, and
    /// `stop` consumes the whole value, so every `Voice` a caller can still ask
    /// this of holds one.
    #[must_use]
    pub fn pipeline_ended(&self) -> bool {
        self.serving
            .as_ref()
            .is_some_and(tokio::task::JoinHandle::is_finished)
    }

    /// The address the pod link actually bound.
    ///
    /// The listener's answer and not the configuration's, so an ephemeral port
    /// reads as the port a device would dial.
    #[must_use]
    pub fn listening(&self) -> String {
        self.listening.clone()
    }

    /// Ask the server to stop, and wait for it to have stopped.
    ///
    /// Graceful by the platform's own definition: open segments finalize, logs
    /// flush, the pipeline drains. Nothing about the machine depends on it — a
    /// host that died instead leaves a schedule that concludes at its own
    /// horizon — so this is tidiness, not safety.
    ///
    /// Bounded by [`STOP_DEADLINE`] for exactly that reason: a drain that does
    /// not finish is reported as the stop's own failure and the runtime is put
    /// down under [`SHUTDOWN_GRACE`], so the process ends whatever the pipeline
    /// is waiting on. The alternative is a host whose signal handlers have
    /// already fired and that nothing short of `SIGKILL` can end.
    #[must_use]
    pub fn stop(mut self) -> Option<String> {
        if let Some(stop) = self.stop.take() {
            let _ = stop.send(());
        }
        let serving = self.serving.take()?;
        let Self { runtime, .. } = self;
        let drained =
            runtime.block_on(async move { tokio::time::timeout(STOP_DEADLINE, serving).await });
        let said = match drained {
            Ok(Ok(Ok(()))) => None,
            Ok(Ok(Err(error))) => Some(format!("the voice pipeline stopped on an error: {error}")),
            Ok(Err(error)) => Some(format!("the voice pipeline's task did not finish: {error}")),
            Err(_) => Some(format!(
                "the voice pipeline did not finish draining within {} s, so it was left where it \
                 was; open segments and logs may be short",
                STOP_DEADLINE.as_secs()
            )),
        };
        runtime.shutdown_timeout(SHUTDOWN_GRACE);
        said
    }
}

/// What the pipeline this host composed is, as a line.
///
/// On the edge's stream rather than the pipeline's own: a reader following the
/// robot's narration should see that the voice half came up without having to
/// join two streams to find out.
///
/// `alerts` says whether this pipeline can interrupt anybody: a configuration
/// that composed no bus attachment runs every other part of the voice half and
/// carries no alert off the machine, and that is a standing property of the
/// deployment worth reading once at startup rather than inferring from a
/// silence.
#[must_use]
pub fn composed_line(config: &Path, listen: &str, alerts: bool, at: SyncTime) -> String {
    edge_line_with(
        words::COMPOSED,
        at,
        &format!(
            "the voice pipeline is running from {}, pod link on {listen}; the scripter's \
             decisions and the bus's motion channel both meet this host's gate, and alerts {}",
            config.display(),
            if alerts {
                "ride its attachment"
            } else {
                "are narration only"
            },
        ),
        &[("alerts", json!(alerts))],
    )
}

/// What a host with no speech configuration is, as a line.
///
/// Said rather than left silent: a robot that hears nothing looks exactly like
/// a robot whose wake word is broken, and the difference is one line.
#[must_use]
pub fn silent_line(at: SyncTime) -> String {
    edge_line(
        words::VOICELESS,
        at,
        "no speech configuration was named, so this host runs its edge half alone: it follows \
         the session's story and narrates it, and nothing in this process authors intent",
    )
}

/// What a host whose speech configuration is not on the machine is, as a line.
///
/// Distinct from [`silent_line`] because the two are different situations for
/// whoever is reading: nobody asked for a voice half, against a unit that was
/// told where its speech configuration is and does not have it yet. The second
/// is the shipped state of every unit until an operator pushes one, so it is a
/// running host with a sentence rather than an exit — a launcher app that died
/// on every unit until somebody pushed a file would take the narrating edge
/// half away with it.
#[must_use]
pub fn absent_line(config: &Path, at: SyncTime) -> String {
    edge_line(
        words::AWAITING_SPEECH_CONFIG,
        at,
        &format!(
            "no speech configuration at {}, so this host runs its edge half alone; push one \
             there and restart the payload's launcher to give this unit a voice",
            config.display(),
        ),
    )
}

/// How long a task that returns immediately is given to be polled once.
///
/// Orders of magnitude past what a spawn costs, because what it bounds is a
/// case that would otherwise hang rather than a duration worth measuring.
#[cfg(test)]
const ENDED_DEADLINE: Duration = Duration::from_secs(10);

#[cfg(test)]
impl Voice {
    /// A pipeline whose serving task has already ended on `message`.
    ///
    /// Built without a listener, an audio device, or a bus.
    fn ended_on(message: &str) -> Self {
        let runtime = tokio::runtime::Builder::new_multi_thread()
            .enable_all()
            .build()
            .expect("a runtime this case can spawn on");
        let message = message.to_owned();
        let serving = runtime.spawn(async move { Err(std::io::Error::other(message)) });
        // The handle answers `is_finished` only once the task has been polled
        // to completion, so the case waits for the state it is about rather
        // than racing the runtime's first poll. Bounded, and loudly: a wait
        // that never ends costs whoever hits it a harness timeout with no
        // sentence in it, on a suite that gates every commit.
        let deadline = std::time::Instant::now() + ENDED_DEADLINE;
        while !serving.is_finished() {
            assert!(
                std::time::Instant::now() < deadline,
                "the serving task did not finish within {ENDED_DEADLINE:?}",
            );
            std::thread::sleep(Duration::from_millis(1));
        }
        let (stop, _stopped) = oneshot::channel();
        Self {
            runtime,
            stop: Some(stop),
            serving: Some(serving),
            listening: "127.0.0.1:0".to_owned(),
            carries_alerts: false,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use reachy_scratch::scratch_dir;

    use super::*;
    use crate::intents::queue;
    use crate::sinks::Stdout;

    /// A configuration naming a key table that is not there.
    const NO_KEYS: &str = "listen_addr = \"127.0.0.1:0\"\npod_psk_file = \"/nowhere/psk.toml\"\n";

    /// The server's end of an alert seam whose raising end nothing keeps.
    ///
    /// What the cases below are about is the composition, not the reporting: a
    /// dropped raiser is a seam nothing raises on, which is what a host with no
    /// alerts to raise looks like.
    fn inbox() -> speech_surface::AlertInbox {
        let (_raiser, inbox) = speech_surface::alert_seam(speech_surface::ALERT_QUEUE_DEPTH);
        inbox
    }

    /// The fixture, with an event sink one case reads back.
    ///
    /// A file rather than `none` because the composition's own proof that the
    /// sink was opened and flushed is that the file is there.
    fn runnable(dir: &Path) -> std::path::PathBuf {
        let events = dir.join("events.jsonl");
        speech_fixture::runnable(dir, speech_fixture::Events::File(&events))
    }

    /// The bus-brain fixture, same sink: the deployment whose alerts travel.
    fn carrying(dir: &Path) -> std::path::PathBuf {
        let events = dir.join("events.jsonl");
        speech_fixture::carrying(dir, speech_fixture::Events::File(&events))
    }

    #[test]
    fn a_configuration_this_host_can_run_becomes_a_listening_pipeline() {
        let dir = scratch_dir("reachy-host-composed");
        let path = runnable(dir.as_ref());
        let (intents, _waiting) = queue();
        let voice = match Voice::start(&path, intents, Arc::new(Stdout), inbox()) {
            Ok(voice) => voice,
            Err(refused) => panic!("a configuration this host can run: {refused}"),
        };

        // The listener's own answer: the configuration says `:0`, so a port
        // here at all is the bind having happened and the address having been
        // asked of the socket rather than read back off the file.
        let listening: std::net::SocketAddr =
            voice.listening().parse().expect("an address, not a string");
        assert_eq!(listening.ip(), std::net::Ipv4Addr::LOCALHOST);
        assert_ne!(listening.port(), 0, "an ephemeral port, resolved");

        assert!(
            voice.stop().is_none(),
            "a pipeline asked to stop stops cleanly"
        );
        // The sink must drain before the runtime goes away.
        assert!(
            dir.join("events.jsonl").exists(),
            "the event sink was opened and flushed"
        );
    }

    #[test]
    fn a_serving_pipeline_has_not_ended() {
        // The healthy answer, from a pipeline that is actually serving: a true
        // here would end the edge's loop on a robot that is working.
        let dir = scratch_dir("reachy-host-serving");
        let path = runnable(dir.as_ref());
        let (intents, _waiting) = queue();
        let voice = match Voice::start(&path, intents, Arc::new(Stdout), inbox()) {
            Ok(voice) => voice,
            Err(refused) => panic!("a configuration this host can run: {refused}"),
        };
        assert!(
            !voice.pipeline_ended(),
            "a pipeline nobody has asked to stop is still serving"
        );
        assert!(voice.stop().is_none(), "a pipeline asked to stop stops");
    }

    #[test]
    fn a_pipeline_that_ended_on_its_own_says_so_and_stopping_reaps_its_sentence() {
        // The path from a dead pipeline to the process's exit status: the
        // predicate the loop breaks on, and the message `run` fails with.
        let voice = Voice::ended_on(
            "brenn bridge exited: no wire version in common: this bridge speaks 3..=3, the \
             server speaks 4..=4",
        );
        assert!(
            voice.pipeline_ended(),
            "a serving task that has finished is a pipeline that ended"
        );
        let said = voice.stop().expect("the sentence the pipeline ended on");
        assert!(
            said.contains("the voice pipeline stopped on an error"),
            "{said}"
        );
        assert!(said.contains("no wire version in common"), "{said}");
    }

    #[test]
    fn a_configuration_the_platform_will_not_validate_says_which() {
        // Binding every interface is what the pod platform's own validation
        // refuses, and the loader runs that validation, so this is the sentence
        // an operator gets for a file that reads and parses and still will not
        // run.
        let dir = scratch_dir("reachy-host-invalid");
        let path = runnable(dir.as_ref());
        let text = std::fs::read_to_string(&path).expect("the fixture");
        std::fs::write(&path, text.replace("127.0.0.1:0", "0.0.0.0:7380")).expect("a file");
        let (intents, _waiting) = queue();
        let refused = Voice::start(&path, intents, Arc::new(Stdout), inbox())
            .err()
            .expect("a configuration this host will not run");
        assert!(
            matches!(refused, NotRunning::Refused(ref message) if message.contains("is not one this host can run")),
            "{refused}"
        );
    }

    #[test]
    fn a_configuration_that_does_not_parse_is_a_sentence() {
        let dir = scratch_dir("reachy-host-voice");
        let path = dir.join("bad.toml");
        std::fs::write(&path, "listen_addr = 7").expect("a file");
        let (intents, _waiting) = queue();
        // `err()` rather than `expect_err`: a running `Voice` owns a runtime and
        // is deliberately not `Debug`, and the failure is the whole assertion.
        let refused = Voice::start(&path, intents, Arc::new(Stdout), inbox())
            .err()
            .expect("a configuration this host will not run on");
        assert!(
            matches!(refused, NotRunning::Refused(ref message) if message.contains("speech configuration")),
            "{refused}"
        );
    }

    #[test]
    fn a_key_table_that_is_not_there_stops_startup() {
        let dir = scratch_dir("reachy-host-keys");
        let path = dir.join("nokeys.toml");
        std::fs::write(&path, NO_KEYS).expect("a file");
        let (intents, _waiting) = queue();
        let refused = Voice::start(&path, intents, Arc::new(Stdout), inbox())
            .err()
            .expect("a key table that is not there");
        assert!(
            matches!(refused, NotRunning::Refused(ref message) if message.contains("pod link")),
            "{refused}"
        );
    }

    #[test]
    fn a_configuration_that_is_not_on_the_machine_is_absent() {
        let dir = scratch_dir("reachy-host-absent");
        let (intents, _waiting) = queue();
        let refused = Voice::start(&dir.join("speech.toml"), intents, Arc::new(Stdout), inbox())
            .err()
            .expect("a path with no file at it");
        assert!(matches!(refused, NotRunning::Absent), "{refused}");
    }

    #[test]
    fn a_configuration_that_will_not_read_is_not_read_as_absent() {
        // A directory where the file should be: the read fails for a reason
        // that is not absence, which is the shape a wrong ownership or a
        // permission-denied lookup has, and the one an operator must not be
        // told to fix by pushing the file again.
        let dir = scratch_dir("reachy-host-unreadable");
        let path = dir.join("speech.toml");
        std::fs::create_dir_all(&path).expect("a directory where a file should be");
        let (intents, _waiting) = queue();
        let refused = Voice::start(&path, intents, Arc::new(Stdout), inbox())
            .err()
            .expect("a configuration this host cannot read");
        assert!(
            matches!(refused, NotRunning::Refused(ref message) if message.contains("speech.toml")),
            "an operator has to be told which file and that it was not readable: {refused}"
        );
    }

    #[test]
    fn a_composed_pipeline_says_where_it_came_from() {
        let at = SyncTime::from_nanos(1_700_000_000_000_000_000);
        let line = composed_line(
            Path::new("/run/reachy/speech.toml"),
            "127.0.0.1:7380",
            true,
            at,
        );
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("one JSON object");
        assert_eq!(parsed["stream"], "edge");
        assert_eq!(parsed["kind"], "composed");
        assert_eq!(parsed["at_ns"], at.as_nanos());
        assert_eq!(parsed["alerts"], true);
        assert!(
            parsed["says"]
                .as_str()
                .expect("a sentence")
                .contains("7380"),
            "{line}",
        );
    }

    #[test]
    fn a_composed_pipeline_says_whether_it_can_interrupt_anybody() {
        let at = SyncTime::from_nanos(1_700_000_000_000_000_000);
        let line = composed_line(
            Path::new("/run/reachy/speech.toml"),
            "127.0.0.1:7380",
            false,
            at,
        );
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("one JSON object");
        assert_eq!(parsed["alerts"], false);
        assert!(
            parsed["says"]
                .as_str()
                .expect("a sentence")
                .contains("narration only"),
            "{line}",
        );
    }

    #[test]
    fn a_pipeline_with_no_bus_attachment_carries_no_alert() {
        // The fixture names no `[brain]` table, so the composed run drops its
        // end of the alert seam: the host must not keep the raising end.
        let dir = scratch_dir("reachy-host-alertless");
        let path = runnable(dir.as_ref());
        let (intents, _waiting) = queue();
        let voice = match Voice::start(&path, intents, Arc::new(Stdout), inbox()) {
            Ok(voice) => voice,
            Err(refused) => panic!("a configuration this host can run: {refused}"),
        };
        assert!(
            !voice.carries_alerts(),
            "no bus attachment is nowhere to publish"
        );
        assert!(voice.stop().is_none(), "a pipeline asked to stop stops");
    }

    #[test]
    fn a_pipeline_whose_brain_is_on_the_bus_carries_its_alerts() {
        // The branch the seam exists for, and the only one where the composed
        // run drains it: a raiser kept against this one reaches an attachment.
        let dir = scratch_dir("reachy-host-alerting");
        let path = carrying(dir.as_ref());
        let (intents, _waiting) = queue();
        let voice = match Voice::start(&path, intents, Arc::new(Stdout), inbox()) {
            Ok(voice) => voice,
            Err(refused) => panic!("a configuration this host can run: {refused}"),
        };
        assert!(
            voice.carries_alerts(),
            "a bus brain is an attachment to publish through"
        );
        assert!(voice.stop().is_none(), "a pipeline asked to stop stops");
    }

    #[test]
    fn a_brain_that_is_not_on_the_bus_carries_no_alert() {
        // A brain, and not the one that builds an attachment: the predicate is
        // about which mode this is and not about whether a brain was named at
        // all, which is the reading a `[brain]`-less fixture cannot tell apart.
        let dir = scratch_dir("reachy-host-echoing");
        let path = runnable(dir.as_ref());
        let text = std::fs::read_to_string(&path).expect("the fixture");
        std::fs::write(
            &path,
            format!(
                "{text}\
                 [brain]\nmode = \"echo\"\n\
                 [stt]\nbackend = \"http\"\nurl = \"http://127.0.0.1:8000\"\nmodel = \"m\"\n\
                 [tts]\nbackend = \"http\"\nurl = \"http://127.0.0.1:8000\"\n\
                 model = \"m\"\nvoice = \"v\"\n",
            ),
        )
        .expect("a file");
        let (intents, _waiting) = queue();
        let voice = match Voice::start(&path, intents, Arc::new(Stdout), inbox()) {
            Ok(voice) => voice,
            Err(refused) => panic!("a configuration this host can run: {refused}"),
        };
        assert!(
            !voice.carries_alerts(),
            "no bridge is built, so nothing drains the seam"
        );
        assert!(voice.stop().is_none(), "a pipeline asked to stop stops");
    }

    #[test]
    fn a_host_with_no_speech_configuration_says_so() {
        let at = SyncTime::from_nanos(7);
        let parsed: serde_json::Value =
            serde_json::from_str(&silent_line(at)).expect("one JSON object");
        assert_eq!(parsed["kind"], "voiceless");
        assert_eq!(parsed["at_ns"], 7);
    }

    #[test]
    fn a_host_whose_speech_configuration_is_not_there_names_the_path() {
        let at = SyncTime::from_nanos(11);
        let line = absent_line(Path::new("host/speech.toml"), at);
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("one JSON object");
        assert_eq!(parsed["kind"], "awaiting_speech_config");
        assert_eq!(parsed["at_ns"], 11);
        assert!(
            parsed["says"]
                .as_str()
                .expect("a sentence")
                .contains("host/speech.toml"),
            "{line}",
        );
    }
}
