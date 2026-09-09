//! `reachy-ask` — the harness's intent source: the wake gesture, or the
//! library tour, through the real intent edge.
//!
//! A run's verdict is the analyzer's reading of the log the control process
//! wrote, and the log says nothing unless something asked the machine to move.
//! On a unit and on a workstation the thing that asks is this binary,
//! standing exactly where the voice host stands in production: outside the
//! composition, holding the two loopback ports of the intent edge, sending a
//! compiled `Script` to 7409 and following the session's narration off 7410.
//!
//! Two modes, one edge. The **gesture** asks once, on the commissioning row,
//! and follows the story for a fixed window the harness ends. The **tour**
//! (`--tour`) asks for every motion the deployed library holds, one script per
//! motion on the sender's own clock, and **ends the run itself**: a library's
//! worth of motion is too long to sit under a budget that would then be spent
//! idling, so when the story is over the sender quits the launcher and the
//! harness's `timeout` is only a backstop. `--tour-budget` prints that
//! backstop, from the same plan, so the two cannot disagree.
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
mod tour;
mod watch;

use std::io;
use std::net::{SocketAddr, UdpSocket};
use std::path::{Path, PathBuf};
use std::process::ExitCode;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use clockwork_rs::SyncTime;
use reachy_edge::{
    Alert, DATAGRAM_CAP, EdgeConfig, HostEdge, LOOPBACK, MotionTable, Origin, POLL,
    REPORTS_OUT_PORT, SCRIPTS_IN_PORT, Surface, Update, alert_line, now,
};
use signal_hook::consts::{SIGINT, SIGTERM};
use signal_hook::flag;

use gesture::{ASK_POD, body};
use tour::{
    END_MARGIN_MS, LAUNCHER_CONTROL, Leg, PROBE_PREFIX, QUIT_CONNECT_WINDOW, RELEASE_ALLOWANCE_MS,
    Tour, quit_launcher, select,
};
use watch::{Watch, is_released};

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
/// The gesture itself runs nineteen seconds — a raise at eight, a fold at
/// sixteen and the three-second stow that ends it — and the release that
/// follows it a few more. Twenty-five leaves the whole of it inside the window
/// with margin, and nothing about the verdict is read off this clock — the
/// analyzer reads the log — so it only has to be long enough. The device
/// harness overrides it in any case.
///
/// It belongs to the gesture alone. The tour knows its own end and is refused
/// this flag.
const RUN_WINDOW: Duration = Duration::from_secs(25);

/// Which run this invocation is.
#[derive(Clone, Debug, PartialEq, Eq)]
enum Mode {
    /// The wake gesture: ask once, follow for the run window.
    Gesture,
    /// The library tour over the names sidecar at this path.
    Tour(PathBuf),
    /// Print the tour's backstop budget for that sidecar, in whole seconds,
    /// and exit. Runs on the workstation, where there is no machine.
    Budget(PathBuf),
    /// Print the names sidecar of the motions this run's selection over that
    /// sidecar holds, and exit. Runs on the workstation, where there is no
    /// machine.
    Table(PathBuf),
}

/// What the invocation asked for.
#[derive(Clone, Debug, PartialEq, Eq)]
struct Options {
    /// How long to wait for the commissioning row before calling the run red.
    ///
    /// The tour reads it the same way: it is the red timeout on commissioning,
    /// never the tour's own budget.
    resting_timeout: Duration,
    /// How long to keep following the story once the gesture has gone out.
    run_window: Duration,
    /// The one motion of the library to play, where the invocation names one.
    ///
    /// `None` is the whole selection the mode implies: for a tour, the library
    /// minus its probes. A name here is a probe run — one motion, played once,
    /// judged on its own.
    motion: Option<String>,
    /// Which run this is.
    mode: Mode,
}

impl Default for Options {
    fn default() -> Self {
        Self {
            resting_timeout: RESTING_TIMEOUT,
            run_window: RUN_WINDOW,
            motion: None,
            mode: Mode::Gesture,
        }
    }
}

/// How to invoke this, for a refusal to print.
fn usage() -> String {
    format!(
        "usage: reachy-ask [--resting-timeout SECONDS] [--run-window SECONDS]\n\
         \x20      reachy-ask --tour SIDECAR [--motion NAME] [--resting-timeout SECONDS]\n\
         \x20      reachy-ask --tour-budget SIDECAR [--motion NAME]\n\
         \x20      reachy-ask --tour-table SIDECAR [--motion NAME]\n\
         \n\
         Binds {REPORTS_OUT_PORT} on loopback, waits for the session to narrate that it\n\
         commissioned, sends compiled scripts to {SCRIPTS_IN_PORT}, and narrates the story.\n\
         One line of JSON per row, on stdout.\n\
         \n\
         Without --tour: one wake gesture, followed until the run window ends.\n\
         \n\
         With --tour SIDECAR: one script per motion of the names sidecar, in library\n\
         order, each at recorded pace, the next going out when the previous motion's\n\
         window has closed. The run ends itself — on the session's release row, or on\n\
         the plan's own clock — and asks the launcher to quit at {LAUNCHER_CONTROL}. Every\n\
         terminal ending the sender reaches by itself quits the launcher, red as well as\n\
         green; only a stop signal does not, because then the launcher is already gone.\n\
         --run-window is refused with --tour: the tour knows its own end.\n\
         A tour plays the library minus its `{PROBE_PREFIX}` instruments: a probe is a step\n\
         goal held for a judged hold, and it is played one at a time.\n\
         \n\
         With --motion NAME: that one motion, and nothing else, whatever the mode. It is\n\
         how a probe is played, and the name is a motion of the sidecar.\n\
         \n\
         With --tour-budget SIDECAR: print the backstop the harness should wrap the\n\
         launcher in, in whole seconds, and exit. Same plan, so the two agree.\n\
         \n\
         With --tour-table SIDECAR: print the names sidecar of the motions this\n\
         selection holds, and exit. It is what the analyzer judges the run against and\n\
         what a fetched run directory carries, so the plan and the verdict are one\n\
         selection made once.\n\
         \n\
         Start it before the launcher: the bind has to precede the control process's first\n\
         narration, and this is what makes that true rather than likely.\n\
         \n\
         Nothing is retried. A commissioning row that never arrives, a script the edge\n\
         refuses, a quit the launcher does not take and a port already held are each a\n\
         nonzero exit.\n\
         \n\
         Defaults: --resting-timeout {}, --run-window {}.",
        RESTING_TIMEOUT.as_secs(),
        RUN_WINDOW.as_secs(),
    )
}

fn main() -> ExitCode {
    match parse(std::env::args().skip(1)) {
        Ok(options) => {
            let ended = match &options.mode {
                Mode::Gesture => run(&options),
                Mode::Tour(sidecar) => tour_run(&options, sidecar),
                Mode::Budget(sidecar) => budget(sidecar, options.motion.as_deref()),
                Mode::Table(sidecar) => emit_table(sidecar, options.motion.as_deref()),
            };
            match ended {
                Ok(()) => ExitCode::SUCCESS,
                Err(message) => {
                    eprintln!("reachy-ask: {message}");
                    ExitCode::FAILURE
                }
            }
        }
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
    let mut mode_given = false;
    let mut args = args;
    while let Some(word) = args.next() {
        match word.as_str() {
            "--tour" | "--tour-budget" | "--tour-table" => {
                let path = args
                    .next()
                    .ok_or_else(|| format!("{word} needs the path of a names sidecar"))?;
                if mode_given {
                    return Err(
                        "--tour, --tour-budget and --tour-table each name the one run this is"
                            .to_owned(),
                    );
                }
                mode_given = true;
                let path = PathBuf::from(path);
                options.mode = match word.as_str() {
                    "--tour" => Mode::Tour(path),
                    "--tour-budget" => Mode::Budget(path),
                    _ => Mode::Table(path),
                };
            }
            "--resting-timeout" => {
                let value = seconds(&word, args.next())?;
                if resting_given {
                    return Err(format!("{word} was given twice"));
                }
                resting_given = true;
                options.resting_timeout = value;
            }
            "--motion" => {
                let name = args
                    .next()
                    .ok_or_else(|| format!("{word} needs the name of a motion"))?;
                if options.motion.is_some() {
                    return Err(format!("{word} was given twice"));
                }
                if name.trim().is_empty() {
                    return Err(format!("{word} takes a motion's name, not an empty one"));
                }
                options.motion = Some(name);
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
    if options.motion.is_some() && options.mode == Mode::Gesture {
        return Err(
            "--motion names a motion of the library, and the wake gesture plays none; it goes \
             with --tour, --tour-budget or --tour-table"
                .to_owned(),
        );
    }
    if window_given && options.mode != Mode::Gesture {
        return Err(
            "--run-window is the gesture's; the tour follows the story until its own plan is \
             over and then ends the run itself"
                .to_owned(),
        );
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

/// The two sockets a run holds: the narration it follows, and the one scripts
/// go out on.
///
/// One struct because both modes hold exactly these two, and because a case
/// that drives a run needs them somewhere other than the well-known port.
struct Ports {
    reports: UdpSocket,
    scripts: UdpSocket,
    /// Where a script goes: the control process's own port in production, and
    /// wherever a case is listening when one drives a run.
    scripts_to: u16,
}

impl Ports {
    /// The production pair: the narration port this seam names, and an
    /// ephemeral source port for the scripts, addressed to the control
    /// process.
    fn bind() -> Result<Self, String> {
        Self::on(REPORTS_OUT_PORT, SCRIPTS_IN_PORT)
    }

    /// The pair, with the narration socket on `reports_port` and the scripts
    /// addressed to `scripts_to`.
    fn on(reports_port: u16, scripts_to: u16) -> Result<Self, String> {
        let reports = UdpSocket::bind((LOOPBACK, reports_port)).map_err(|error| {
            // Only the address-in-use case names another reader. A permission
            // refusal or an exhausted descriptor table are the other ways this
            // fails, and naming a process that does not exist is worse than
            // saying what the operating system said.
            let cause = if error.kind() == io::ErrorKind::AddrInUse {
                "; something else already holds it — a second harness run, or a voice host on \
                 this machine"
            } else {
                ""
            };
            format!("binding the reports port {reports_port} on loopback: {error}{cause}")
        })?;
        reports
            .set_read_timeout(Some(POLL))
            .map_err(|error| format!("setting the read timeout on the reports port: {error}"))?;
        // An ephemeral source port: the seam identifies a datagram by the port
        // it arrived on, never by where it came from.
        let scripts = UdpSocket::bind((LOOPBACK, 0))
            .map_err(|error| format!("opening the socket the scripts go out on: {error}"))?;
        Ok(Self {
            reports,
            scripts,
            scripts_to,
        })
    }
}

/// The stop the harness sends when the launcher is done.
///
/// A background child of a non-interactive shell inherits SIGINT ignored, so a
/// handler is what makes the stop act at all — without one the scripts'
/// `kill -INT` is a no-op and the run holds the narration port until its own
/// deadline.
fn stop_flag() -> Result<Arc<AtomicBool>, String> {
    let stop = Arc::new(AtomicBool::new(false));
    for signal in [SIGINT, SIGTERM] {
        flag::register(signal, Arc::clone(&stop))
            .map_err(|error| format!("installing the stop flag for signal {signal}: {error}"))?;
    }
    Ok(stop)
}

/// One datagram off the narration port, followed.
///
/// The shape both runs poll in: a read that timed out or was interrupted is the
/// poll doing its job rather than a failure, and a datagram that is not a story
/// is narrated and dropped by the edge itself. `None` either way, and the
/// caller goes round again.
///
/// One implementation because a change here — an `io::ErrorKind` worth
/// tolerating, a different answer to a datagram nobody can read — is a change
/// to how both runs follow the same session, and a copy of it would be a change
/// made in one run and not the other with nothing to say so.
///
/// # Errors
///
/// A read the operating system refused for any other reason, which is the
/// socket having gone away under the run.
fn heard(
    ports: &Ports,
    host: &mut HostEdge,
    buffer: &mut [u8],
    surface: &mut impl Surface,
) -> Result<Option<Update>, String> {
    let read = match ports.reports.recv_from(buffer) {
        Ok((bytes, _)) => bytes,
        Err(error)
            if matches!(
                error.kind(),
                io::ErrorKind::WouldBlock | io::ErrorKind::TimedOut | io::ErrorKind::Interrupted
            ) =>
        {
            // A signal arriving mid-read is the stop the callers are watching
            // for, not a failed run: their own flag is what says so.
            return Ok(None);
        }
        Err(error) => return Err(format!("reading the reports port: {error}")),
    };
    Ok(host.follow(&buffer[..read], now(), surface))
}

/// One script through the sender's own edge and onto the wire.
///
/// `what` names the thing being asked for, in both refusals: the edge's screens
/// and the socket are the same two ways either run fails to ask, and what
/// differs between a gesture and a tour leg is only which one it was.
///
/// # Errors
///
/// A screen of the edge's own that refused the script, or a socket that would
/// not take it.
fn offer_and_post(
    host: &mut HostEdge,
    ports: &Ports,
    surface: &mut impl Surface,
    script: &[u8],
    what: &str,
) -> Result<(u32, SyncTime), String> {
    let arrival = now();
    let accepted = host
        .offer(script, Origin::Local, arrival, surface)
        .ok_or_else(|| {
            format!("the edge refused {what}; the line above names the screen it stopped at")
        })?;
    ports
        .scripts
        .send_to(accepted.bytes(), (LOOPBACK, ports.scripts_to))
        .map_err(|error| {
            format!(
                "sending {what} to the control process on {}: {error}",
                ports.scripts_to
            )
        })?;
    Ok((accepted.script_id, arrival))
}

/// Hold the ports, ask once, narrate until the window ends.
fn run(options: &Options) -> Result<(), String> {
    let ports = Ports::bind()?;
    let stop = stop_flag()?;

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
        let Some(update) = heard(&ports, &mut host, &mut buffer, &mut surface)? else {
            continue;
        };

        if watch.should_ask(&update.rows) {
            let (script_id, arrival) = offer_and_post(
                &mut host,
                &ports,
                &mut surface,
                body(ASK_POD).as_bytes(),
                "the harness gesture",
            )?;
            surface.say(asked_line(script_id, options.run_window.as_secs(), arrival));
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

/// The tour's plan, out of the sidecar at `path`, over the motions `motion`
/// selects.
///
/// The table that comes back is the selection and not the library: it is what
/// the edge resolves a script's name against, so a run cannot play a motion it
/// was not asked to.
fn plan(path: &Path, motion: Option<&str>) -> Result<(MotionTable, Tour), String> {
    let text = std::fs::read_to_string(path)
        .map_err(|error| format!("reading the names sidecar `{}`: {error}", path.display()))?;
    let library = MotionTable::from_sidecar(&text)
        .map_err(|error| format!("reading the names sidecar `{}`: {error}", path.display()))?;
    let table = select(&library, motion)?;
    let tour = Tour::of(&table)?;
    Ok((table, tour))
}

/// Print the backstop budget the harness should wrap the launcher in, in whole
/// seconds.
///
/// The workstation side of the tour: no ports, no machine, no launcher — the
/// deploy script asks this before it starts anything, and the number comes off
/// the same plan the run itself follows.
fn budget(path: &Path, motion: Option<&str>) -> Result<(), String> {
    let (_, tour) = plan(path, motion)?;
    println!("{}", tour.budget().as_secs());
    Ok(())
}

/// Print the names sidecar of the motions this selection holds.
///
/// The other workstation side of a run: the harness writes this beside the
/// records it fetched and hands it to the analyzer, so what a run is judged
/// against is the table its plan came off rather than a second reading of the
/// library. A selection that holds nothing is the refusal `plan` gives, before
/// anything is pushed.
fn emit_table(path: &Path, motion: Option<&str>) -> Result<(), String> {
    let (table, _) = plan(path, motion)?;
    println!("{}", table.to_sidecar());
    Ok(())
}

/// How a tour stopped.
///
/// The distinction is the whole of the quit rule: an ending the sender reached
/// by itself leaves a launcher still running, which must be asked to quit
/// however red the ending was, and a stop signal means the launcher has already
/// returned and there is nothing left to ask.
enum Ending {
    /// The sender reached this ending itself, green or red.
    Told(Result<(), String>),
    /// A stop signal arrived. Red, and no quit.
    Stopped(String),
}

/// Run the tour, then settle up with the launcher.
///
/// The stop flag is installed here rather than inside the run, because the quit
/// watches it too: a stop that arrives while the control port is not listening
/// says the launcher has gone, and waiting the connect window out for it would
/// keep the shell waiting on this process for nothing.
fn tour_run(options: &Options, path: &Path) -> Result<(), String> {
    let control: SocketAddr = LAUNCHER_CONTROL
        .parse()
        .expect("the launcher's control address is a constant of this file");
    let installed = stop_flag();
    // A flag that would not install never sets, so the quit and the loop both
    // behave as they do on a run nobody signals.
    let stop = installed
        .clone()
        .unwrap_or_else(|_| Arc::new(AtomicBool::new(false)));
    let ending = match installed {
        Err(message) => Ending::Told(Err(message)),
        Ok(_) => tour_ending(options, path, &stop),
    };
    settle(ending, control, QUIT_CONNECT_WINDOW, &stop)
}

/// The tour's own ending: the plan, the ports, the loop.
///
/// Every failure here is `Told` — a sidecar that will not read is as much the
/// sender's own ending as a commissioning timeout, and leaving the launcher
/// running for a plan that never started would spend the whole backstop.
fn tour_ending(options: &Options, path: &Path, stop: &AtomicBool) -> Ending {
    let started = (|| {
        let (table, tour) = plan(path, options.motion.as_deref())?;
        let ports = Ports::bind()?;
        Ok((table, tour, ports))
    })();
    match started {
        Err(message) => Ending::Told(Err(message)),
        Ok((table, tour, ports)) => conduct(options, &tour, table, &ports, stop, &mut Console),
    }
}

/// Quit the launcher, unless the stop already did, and answer with the run's
/// own status.
///
/// A quit the launcher did not take is a red of its own whatever the ending
/// was: the run is then about to ride its backstop, which is the one ending
/// nothing here chose. `window` is how long a launcher that has not bound its
/// control port yet is waited for, which is what keeps an ending reached in the
/// sender's first moments from spending the whole backstop.
fn settle(
    ending: Ending,
    control: SocketAddr,
    window: Duration,
    stop: &AtomicBool,
) -> Result<(), String> {
    match ending {
        Ending::Stopped(message) => Err(message),
        Ending::Told(ending) => match (ending, quit_launcher(control, window, stop)) {
            (Ok(()), Ok(())) => Ok(()),
            (Ok(()), Err(quit)) => Err(quit),
            (Err(red), Ok(())) => Err(red),
            (Err(red), Err(quit)) => Err(format!("{red}; and then {quit}")),
        },
    }
}

/// Follow the story, sending each leg when the previous one's window has
/// closed, until the tour is over one way or another.
fn conduct(
    options: &Options,
    tour: &Tour,
    table: MotionTable,
    ports: &Ports,
    stop: &AtomicBool,
    surface: &mut impl Surface,
) -> Ending {
    let legs = tour.legs();
    let mut host = HostEdge::new(EdgeConfig::for_pod(ASK_POD), table);
    let mut buffer = vec![0u8; DATAGRAM_CAP];
    let mut sent = 0usize;
    let mut watch = Watch::new();
    let mut next_send = Instant::now();
    // Taken when the watch fires: until then it is the red timeout on
    // commissioning, and afterwards there is nothing it would mean.
    let mut resting_deadline = Some(Instant::now() + options.resting_timeout);
    let mut clock_end: Option<Instant> = None;

    loop {
        if stop.load(Ordering::Relaxed) {
            return Ending::Stopped(format!(
                "stopped after {sent} of the tour's {} scripts; the launcher had already gone, \
                 so nothing was asked to quit",
                legs.len()
            ));
        }
        let at = Instant::now();
        if let Some(deadline) = resting_deadline {
            if at >= deadline {
                return Ending::Told(Err(format!(
                    "no commissioning row in {}s: the session never narrated entering `resting` \
                     from `starting`, so the tour never started",
                    options.resting_timeout.as_secs()
                )));
            }
        } else if sent < legs.len() {
            if at >= next_send {
                let leg = &legs[sent];
                if let Err(message) = send(&mut host, ports, surface, leg) {
                    return Ending::Told(Err(message));
                }
                sent += 1;
                next_send = at + Duration::from_millis(leg.window_close_ms);
                if sent == legs.len() {
                    clock_end = Some(
                        at + Duration::from_millis(
                            leg.stow_end_ms + RELEASE_ALLOWANCE_MS + END_MARGIN_MS,
                        ),
                    );
                }
                continue;
            }
        } else if clock_end.is_some_and(|end| at >= end) {
            surface.say(ending_line(
                sent,
                "the tour's whole plan ran out with no release row; the session may have parked \
                 part-way, and the analyzer names every motion that never moved",
            ));
            return Ending::Told(Ok(()));
        }

        let update = match heard(ports, &mut host, &mut buffer, surface) {
            Ok(Some(update)) => update,
            Ok(None) => continue,
            Err(message) => return Ending::Told(Err(message)),
        };

        // The same latch the gesture asks on, and for the reason its doc
        // states: only the first entry into `resting` out of `starting` is the
        // session having commissioned, and a later one is a session that ended.
        if watch.should_ask(&update.rows) {
            // The first leg goes out on the row itself; every later one on the
            // sender's own clock.
            next_send = at;
            resting_deadline = None;
            continue;
        }
        if resting_deadline.is_some() {
            continue;
        }
        // The release row counts only once every script has gone out. An
        // earlier one is a session that ended under the tour — a park — and
        // that ending is the clock's, so that the sender keeps to its plan and
        // the analyzer gets the whole list of what never moved.
        if sent == legs.len() && update.rows.iter().any(is_released) {
            surface.say(ending_line(
                sent,
                "every motion in the library went out and the session released",
            ));
            return Ending::Told(Ok(()));
        }
    }
}

/// One leg out through the sender's own edge and onto the wire.
fn send(
    host: &mut HostEdge,
    ports: &Ports,
    surface: &mut impl Surface,
    leg: &Leg,
) -> Result<(), String> {
    let (script_id, arrival) = offer_and_post(
        host,
        ports,
        surface,
        leg.script.encode().as_bytes(),
        &format!("the tour's script for `{}`", leg.name),
    )?;
    surface.say(played_line(script_id, leg, arrival));
    Ok(())
}

/// A leg went out, as a line on the stream the story is narrated on.
fn played_line(script_id: u32, leg: &Leg, at: SyncTime) -> String {
    serde_json::json!({
        "stream": "edge",
        "at_ns": at.as_nanos(),
        "kind": "asked",
        "script_id": script_id,
        "motion_id": leg.motion_id,
        "motion": leg.name,
        "says": format!(
            "script {script_id} plays `{}` at recorded pace; its window closes {} ms from now",
            leg.name, leg.window_close_ms
        ),
    })
    .to_string()
}

/// The tour is over, and why.
///
/// Said before the quit goes out, so the console carries the sender's own last
/// word whatever the launcher then does with the request.
fn ending_line(sent: usize, why: &str) -> String {
    serde_json::json!({
        "stream": "edge",
        "at_ns": now().as_nanos(),
        "kind": "ending",
        "scripts": sent,
        "says": why,
    })
    .to_string()
}

#[cfg(test)]
mod tests {
    use std::io::{Read, Write};
    use std::net::{Ipv4Addr, TcpListener, UdpSocket};
    use std::path::PathBuf;
    use std::sync::atomic::AtomicBool;
    use std::thread;
    use std::time::Duration;

    use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
    use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
    use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, TimelineWire};
    use clockwork_rs::blob_as_bytes;
    use reachy_edge::{Alert, LOOPBACK, Surface};

    use super::{
        Console, Ending, Mode, Options, Ports, Tour, conduct, parse, plan, settle, verdict,
    };

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
                motion: None,
                mode: Mode::Gesture,
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

    #[test]
    fn the_tour_names_its_sidecar() {
        let options = parsed(&["--tour", "cogs/clip_library.names.json"]).expect("the tour");
        assert_eq!(
            options.mode,
            Mode::Tour(PathBuf::from("cogs/clip_library.names.json")),
        );
        assert_eq!(
            options.resting_timeout,
            Options::default().resting_timeout,
            "the tour is launched on the shipped commissioning timeout",
        );
        let budget = parsed(&["--tour-budget", "names.json"]).expect("the budget");
        assert_eq!(budget.mode, Mode::Budget(PathBuf::from("names.json")));
    }

    #[test]
    fn the_tour_takes_the_commissioning_timeout_and_refuses_the_run_window() {
        assert_eq!(
            parsed(&["--tour", "names.json", "--resting-timeout", "5"])
                .expect("the tour's own red timeout on commissioning")
                .resting_timeout,
            Duration::from_secs(5),
        );
        let refused = parsed(&["--tour", "names.json", "--run-window", "9"])
            .expect_err("the tour knows its own end");
        assert!(refused.contains("--run-window"), "{refused}");
        let refused =
            parsed(&["--run-window", "9", "--tour", "names.json"]).expect_err("in either order");
        assert!(refused.contains("--run-window"), "{refused}");
    }

    #[test]
    fn a_probe_run_names_the_one_motion_it_plays() {
        let options = parsed(&[
            "--tour",
            "cogs/clip_library.names.json",
            "--motion",
            "probe/antenna-step-a",
        ])
        .expect("a probe run");
        assert_eq!(
            options.mode,
            Mode::Tour(PathBuf::from("cogs/clip_library.names.json")),
        );
        assert_eq!(options.motion.as_deref(), Some("probe/antenna-step-a"));
        let table = parsed(&["--tour-table", "names.json", "--motion", "bench/nod"])
            .expect("the table a run is judged against");
        assert_eq!(table.mode, Mode::Table(PathBuf::from("names.json")));
        assert_eq!(table.motion.as_deref(), Some("bench/nod"));
        assert_eq!(
            parsed(&["--tour-budget", "names.json", "--motion", "bench/nod"])
                .expect("the probe run's backstop")
                .motion
                .as_deref(),
            Some("bench/nod"),
            "the backstop comes off the plan the selection makes, not the library's",
        );
    }

    #[test]
    fn a_motion_is_named_once_and_never_for_the_gesture() {
        let refused = parsed(&["--motion", "bench/nod"])
            .expect_err("the gesture plays no motion of the library");
        assert!(refused.contains("--motion"), "{refused}");
        let refused = parsed(&[
            "--tour",
            "names.json",
            "--motion",
            "bench/nod",
            "--motion",
            "bench/perk",
        ])
        .expect_err("one motion, once");
        assert!(refused.contains("twice"), "{refused}");
        assert!(parsed(&["--tour", "names.json", "--motion"]).is_err());
        let refused =
            parsed(&["--tour", "names.json", "--motion", "  "]).expect_err("a name of nothing");
        assert!(refused.contains("empty"), "{refused}");
    }

    #[test]
    fn one_run_per_invocation_and_each_mode_needs_its_sidecar() {
        let refused = parsed(&["--tour", "a.json", "--tour-budget", "a.json"])
            .expect_err("two runs in one invocation");
        assert!(refused.contains("the one run this is"), "{refused}");
        let refused = parsed(&["--tour", "a.json", "--tour-table", "a.json"])
            .expect_err("a run is not also a table to print");
        assert!(refused.contains("the one run this is"), "{refused}");
        assert!(
            parsed(&["--tour-table"]).is_err(),
            "a table needs a sidecar"
        );
        let refused = parsed(&["--tour"]).expect_err("a mode with no sidecar");
        assert!(refused.contains("names sidecar"), "{refused}");
    }

    #[test]
    fn a_sidecar_that_will_not_read_is_the_senders_own_ending() {
        let refused =
            plan(&PathBuf::from("no/such/names.json"), None).expect_err("a missing sidecar");
        assert!(refused.contains("no/such/names.json"), "{refused}");
    }

    /// A listener standing in for the launcher's control API, answering one
    /// request with 200 and handing back what it was sent.
    fn launcher() -> (std::net::SocketAddr, thread::JoinHandle<String>) {
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).expect("an ephemeral port");
        let address = listener
            .local_addr()
            .expect("a bound listener has an address");
        let handle = thread::spawn(move || {
            let (mut stream, _) = listener.accept().expect("the one request");
            let mut request = [0u8; 512];
            let read = stream.read(&mut request).expect("the request's bytes");
            stream
                .write_all(b"HTTP/1.1 200 OK\r\nContent-Length: 0\r\n\r\n")
                .expect("the launcher answers");
            String::from_utf8_lossy(&request[..read]).into_owned()
        });
        (address, handle)
    }

    /// A one-motion table, so a plan exists to run.
    fn table() -> reachy_edge::MotionTable {
        reachy_edge::MotionTable::of([(
            "pollen/dances/nod".to_owned(),
            reachy_edge::MotionEntry {
                motion_id: 0,
                window: motion_proto::PlayWindow {
                    duration_ms: 1000,
                    blend_out_ms: 60,
                },
            },
        )])
    }

    /// Everything the run said, kept rather than printed.
    #[derive(Default)]
    struct Recorded {
        lines: Vec<String>,
    }

    impl Surface for Recorded {
        fn say(&mut self, line: String) {
            self.lines.push(line);
        }

        fn alert(&mut self, _alert: &Alert) {}
    }

    impl Recorded {
        /// The motions the run said it played, in the order it said so.
        fn played(&self) -> Vec<String> {
            self.lines
                .iter()
                .filter_map(|line| serde_json::from_str::<serde_json::Value>(line).ok())
                .filter(|line| line["kind"] == "asked")
                .filter_map(|line| line["motion"].as_str().map(str::to_owned))
                .collect()
        }

        /// What the run said about how it ended, if it said anything.
        fn ending(&self) -> Option<String> {
            self.lines
                .iter()
                .filter_map(|line| serde_json::from_str::<serde_json::Value>(line).ok())
                .find(|line| line["kind"] == "ending")
                .and_then(|line| line["says"].as_str().map(str::to_owned))
        }
    }

    /// A two-motion table, numbered as the emitter numbers one.
    fn two_motions() -> reachy_edge::MotionTable {
        reachy_edge::MotionTable::of([
            (
                "pollen/dances/nod".to_owned(),
                reachy_edge::MotionEntry {
                    motion_id: 0,
                    window: motion_proto::PlayWindow {
                        duration_ms: 1000,
                        blend_out_ms: 60,
                    },
                },
            ),
            (
                "pollen/emotions/oops".to_owned(),
                reachy_edge::MotionEntry {
                    motion_id: 1,
                    window: motion_proto::PlayWindow {
                        duration_ms: 800,
                        blend_out_ms: 60,
                    },
                },
            ),
        ])
    }

    /// The same plan on a clock a case can wait out: the scripts and their
    /// order are the tour's own, and only the milliseconds between them are
    /// the case's.
    fn brisk(table: &reachy_edge::MotionTable) -> Tour {
        let mut legs = Tour::of(table).expect("a plan").legs().to_vec();
        for leg in &mut legs {
            leg.window_close_ms = 50;
            leg.stow_end_ms = 50;
        }
        Tour::of_legs(legs)
    }

    /// One narration row: a phase change from `b` into `a`.
    fn phase(a: SessionPhaseWire, b: SessionPhaseWire) -> TimelineEntryWire {
        let mut entry = TimelineEntryWire::new();
        entry.set_kind(ReportKindWire::PHASE_CHANGED);
        entry.set_a(u32::from(a.0));
        entry.set_b(u32::from(b.0));
        entry
    }

    /// A story datagram carrying `rows`, as the session publishes one: the
    /// whole story every time, so a follower takes what is new off the end.
    fn story(rows: &[TimelineEntryWire]) -> Vec<u8> {
        let mut message = TimelineWire::new();
        {
            let mut entries = message.entries_mut();
            entries.clear();
            for row in rows {
                *entries.try_grow().expect("a short story") = row.clone();
            }
        }
        blob_as_bytes(&message).to_vec()
    }

    /// The session having commissioned.
    fn commissioned() -> TimelineEntryWire {
        phase(SessionPhaseWire::RESTING, SessionPhaseWire::STARTING)
    }

    /// The session having released.
    fn released() -> TimelineEntryWire {
        phase(SessionPhaseWire::RESTING, SessionPhaseWire::STOPPING)
    }

    /// The two sockets a case drives a run through: where the scripts land, and
    /// where the narration is written from.
    fn wiring() -> (Ports, UdpSocket, UdpSocket, std::net::SocketAddr) {
        let control = UdpSocket::bind((LOOPBACK, 0)).expect("a port for the control process");
        control
            .set_read_timeout(Some(Duration::from_secs(10)))
            .expect("a case does not wait for a script forever");
        let scripts_to = control
            .local_addr()
            .expect("a bound socket has an address")
            .port();
        let ports = Ports::on(0, scripts_to).expect("an ephemeral narration port");
        let narrator = UdpSocket::bind((LOOPBACK, 0)).expect("a port to narrate from");
        let reports = ports
            .reports
            .local_addr()
            .expect("a bound socket has an address");
        (ports, control, narrator, reports)
    }

    #[test]
    fn the_commissioning_row_releases_every_leg_and_the_release_row_ends_the_run() {
        let table = two_motions();
        let tour = brisk(&table);
        let (ports, control, narrator, reports) = wiring();
        let told = thread::spawn(move || {
            narrator
                .send_to(&story(&[commissioned()]), reports)
                .expect("the commissioning row");
            let mut buffer = vec![0u8; super::DATAGRAM_CAP];
            let mut scripts = 0;
            while scripts < 2 {
                control.recv_from(&mut buffer).expect("a leg of the tour");
                scripts += 1;
            }
            // The story is over only once every script has gone out, so this
            // row is the ending only because it comes after both.
            narrator
                .send_to(&story(&[commissioned(), released()]), reports)
                .expect("the release row");
            scripts
        });
        let mut recorded = Recorded::default();
        let ending = conduct(
            &Options::default(),
            &tour,
            table,
            &ports,
            &AtomicBool::new(false),
            &mut recorded,
        );
        assert_eq!(told.join().expect("the daemon's side"), 2);
        assert!(
            matches!(ending, Ending::Told(Ok(()))),
            "a tour that ran its plan out and saw the session release is green",
        );
        assert_eq!(
            recorded.played(),
            vec!["pollen/dances/nod", "pollen/emotions/oops"],
            "one script per motion, in the order the library numbers them",
        );
        let said = recorded.ending().expect("the run says how it ended");
        assert!(said.contains("the session released"), "{said}");
    }

    #[test]
    fn a_release_row_before_the_last_leg_leaves_the_tour_on_its_own_clock() {
        // A session that ended part-way through the plan: the sender keeps to
        // its plan regardless, so that the analyzer is handed the whole list of
        // what never moved, and the run ends on the clock instead.
        let table = two_motions();
        let tour = brisk(&table);
        let (ports, control, narrator, reports) = wiring();
        narrator
            .send_to(&story(&[commissioned(), released()]), reports)
            .expect("both rows before a single leg has gone out");
        let heard = thread::spawn(move || {
            let mut buffer = vec![0u8; super::DATAGRAM_CAP];
            let mut scripts = 0;
            while scripts < 2 {
                control.recv_from(&mut buffer).expect("a leg of the tour");
                scripts += 1;
            }
            scripts
        });
        let mut recorded = Recorded::default();
        let ending = conduct(
            &Options::default(),
            &tour,
            table,
            &ports,
            &AtomicBool::new(false),
            &mut recorded,
        );
        assert_eq!(heard.join().expect("the daemon's side"), 2);
        assert!(
            matches!(ending, Ending::Told(Ok(()))),
            "the clock ending is a green ending: what the machine did is the analyzer's",
        );
        let said = recorded.ending().expect("the run says how it ended");
        assert!(said.contains("whole plan ran out"), "{said}");
    }

    #[test]
    fn a_commissioning_timeout_is_red_and_still_quits_the_launcher() {
        let table = table();
        let tour = Tour::of(&table).expect("a one-motion library");
        let options = Options {
            resting_timeout: Duration::from_millis(1),
            mode: Mode::Tour(PathBuf::from("names.json")),
            ..Options::default()
        };
        // An ephemeral narration port and no control process: nothing ever
        // narrates, so the run ends on its commissioning timeout.
        let ports = Ports::on(0, 0).expect("an ephemeral narration port");
        let stopped = AtomicBool::new(false);
        let ending = conduct(&options, &tour, table, &ports, &stopped, &mut Console);
        let (address, handle) = launcher();
        let red = settle(ending, address, Duration::ZERO, &AtomicBool::new(false))
            .expect_err("a run with no machine under it");
        assert!(red.contains("no commissioning row in 0s"), "{red}");
        let request = handle.join().expect("the listener thread");
        assert!(
            request.starts_with("POST /quit"),
            "a red ending quits too, or the run rides its backstop for nothing: {request}",
        );
    }

    #[test]
    fn a_stop_signal_issues_no_quit() {
        let table = table();
        let tour = Tour::of(&table).expect("a one-motion library");
        let ports = Ports::on(0, 0).expect("an ephemeral narration port");
        let stopped = AtomicBool::new(true);
        let ending = conduct(
            &Options::default(),
            &tour,
            table,
            &ports,
            &stopped,
            &mut Console,
        );
        assert!(
            matches!(ending, Ending::Stopped(_)),
            "a stop signal is the one ending that does not quit",
        );
        // An address nothing is listening on: if `settle` tried the quit, it
        // would fail there and say so instead of saying what stopped the run.
        let address = {
            let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).expect("an ephemeral port");
            listener
                .local_addr()
                .expect("a bound listener has an address")
        };
        let red = settle(ending, address, Duration::ZERO, &AtomicBool::new(false))
            .expect_err("a stop is a red run");
        assert!(red.contains("stopped after 0"), "{red}");
        assert!(
            !red.contains("connecting to the launcher"),
            "the launcher is already gone when a stop arrives: {red}",
        );
    }

    #[test]
    fn a_green_ending_the_launcher_will_not_take_is_red() {
        let listener = TcpListener::bind((Ipv4Addr::LOCALHOST, 0)).expect("an ephemeral port");
        let address = listener
            .local_addr()
            .expect("a bound listener has an address");
        drop(listener);
        let red = settle(
            Ending::Told(Ok(())),
            address,
            Duration::ZERO,
            &AtomicBool::new(false),
        )
        .expect_err("a quit nobody took");
        assert!(red.contains("connecting to the launcher"), "{red}");
    }
}
