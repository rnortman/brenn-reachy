//! The loop: the grid, the seam's sending half, and the cycle's own order.
//!
//! What a cycle does against the wire is [`crate::tick`]; this is what runs it.
//! Sleep to the next grid point, take whatever arrived on the two inbound
//! ports, run the cycle, put its reports on the outbound ports. Nothing here
//! decides what reaches a servo, and nothing here holds driver state: the loop
//! owns time, sockets and counters, which is the whole of what a simulation of
//! this driver does not need.
//!
//! Four things the loop answers for that a cycle cannot see:
//!
//! - **A cycle that did not run.** [`Grid::next_after`] skips rather than
//!   catches up, and the count it hands back is published as `cycle_skipped`.
//!   The cycle that ran is never told; a skipped grid point is a fact about the
//!   schedule.
//! - **The release this process already wrote.** A driver that came up next to
//!   a torqued machine has already let it go: the minimum risk condition is
//!   written before this loop exists, because a driver cannot know what its
//!   predecessor left behind. What is left here is saying so — the first cycle
//!   publishes the sweep's `startup_mrc_write`, dated at the instant the sweep
//!   ran.
//! - **A port with no reader.** A reader thread that gave up is a port whose
//!   datagrams reach nothing, `TorqueOffNow` among them, so the loop de-torques
//!   rather than keeping a machine energised behind a seam it can no longer be
//!   released through. Nothing gates that, and the release goes out again on
//!   every cycle for as long as the port has no reader: a wire failure is not
//!   recovered from, so nothing arriving on the port that still has a reader
//!   may leave the state it forces.
//! - **A stop nobody on the bus asked for.** A signal sets a flag the loop
//!   reads once per cycle; the loop then writes the torque-off sweep, keeps
//!   cycling until the confirmation pass says what it read back, and returns.
//!   An operator's stop is the one case where control is still trusted while
//!   the process is ending, so the release is commanded rather than left to the
//!   servos' own watchdog — which, on every stop this cannot answer, stops the
//!   machine with torque still held.
//!
//! And one thing the clock does that the grid cannot express: a
//! `CLOCK_REALTIME` step backwards. The grid is drawn on that clock, so a step
//! puts every remaining grid point that much further away in real time; the
//! loop de-torques the machine and re-draws the grid from where the clock now
//! is, rather than sleeping out the difference. Sleeping it out is a torqued
//! machine with nothing watching it for the size of the step; re-drawing alone
//! is no better, because every timer that could de-torque it — the dead-man
//! above all — measures a difference against the clock that moved, and is
//! suspended for the same span. A step backwards is loss of that time base, so
//! the machine goes to the minimum risk condition.
//!
//! Publication is one unconnected socket sending to four loopback destinations.
//! A datagram is the schema's own bytes at the schema's own size, because that
//! is what the socket on the other end reads. A send that fails is counted and
//! the cycle goes on: a report nobody received is a report nobody received, and
//! re-sending it later would put a stale sample on a stream whose consumer
//! dates everything it reads.

use std::fmt::Write as _;
use std::io;
use std::net::{SocketAddr, SocketAddrV4, UdpSocket};
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::Duration as StdDuration;

use brenn_reachy__driver__health_clk_rs::{DriverEventWire, DriverStatusWire, EventKind};
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use clockwork_rs::{Blob, SyncTime, blob_as_bytes};
use reachy_bus::BusPort;
use reachy_driver::{ConfirmReport, Event, TORQUE_OFF_CONFIRM_BUDGET_NS};

use crate::grid::{Cycle, Grid, NANOS_PER_SECOND};
use crate::inbound::{Counts, Inbound, Inbox};
use crate::ports::{AUX_OUT_PORT, EVENT_PORT, HEALTH_PORT, LOOPBACK, POSE_PORT, STATUS_PORT};
use crate::tick::{CycleReport, Tick, TickCounts, now_ns};

/// Whether somebody has asked this process to stop.
///
/// A trait rather than the flag itself so that the loop's stop rules are stated
/// over something a case can set at a chosen cycle. The driver runs it over an
/// `AtomicBool` a signal handler stores into; a signal handler doing nothing but
/// a store is what lets every consequence of a stop run on the loop thread,
/// which is the only thread here allowed to write a torque-off sweep.
pub trait Stop {
    /// Whether the stop has been asked for.
    fn asked(&self) -> bool;
}

impl Stop for AtomicBool {
    fn asked(&self) -> bool {
        self.load(Ordering::Relaxed)
    }
}

/// Nothing ever asks: the loop runs its cycles out.
impl Stop for () {
    fn asked(&self) -> bool {
        false
    }
}

/// What one turn of a loop over the grid did.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Stepped {
    /// The cycle ran and the grid advanced past it.
    Advanced,
    /// The clock stepped backwards, so the grid was re-drawn and no cycle ran.
    Reanchored,
    /// A stop was asked for during the wait, so no cycle ran and the caller
    /// winds down instead of waiting the grid point out.
    Stopped,
}

/// How a bounded run of cycles ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Ran {
    /// Every cycle asked for ran, and nobody asked the process to stop.
    Cycles,
    /// A stop was asked for and the wind-down below it has finished. The loop is
    /// done: the caller's next act is to say what happened and exit.
    WoundDown(WoundDown),
}

/// What a wind-down found and what it managed to establish.
///
/// The whole of the vocabulary, because the event schema has none for it: a
/// wind-down is a thing an operator's stop gesture does, and what a stopping
/// driver has to say about torque it says in the line it prints on the way out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WoundDown {
    /// Nothing was believed torqued and no de-torquing was outstanding, so the
    /// stop cost no bus work.
    ///
    /// A belief and not a reading: a process that has just started believes
    /// nothing because it has asked the machine nothing, so this says what this
    /// driver commanded rather than what a servo answered.
    AlreadyReleased,
    /// A torque-off sweep went out and a whole pass read every row back
    /// released.
    Confirmed,
    /// A torque-off sweep went out and the confirmation budget ran out without a
    /// clean pass. The sweep was written on every cycle of it regardless — this
    /// says what was read back, not what was commanded.
    Unconfirmed,
}

impl WoundDown {
    /// How every one of these lines starts.
    ///
    /// A driver that answered its stop prints its last counter summary and one
    /// of these lines together, so this prefix is what a reader of the console
    /// can find the end of the run by. Named here rather than spelled in the
    /// offline report, so a rewording of a line stops that build instead of
    /// quietly turning the check off.
    pub const STOPPING: &str = "stopping:";

    /// What to print about it.
    #[must_use]
    pub fn line(self) -> &'static str {
        match self {
            Self::AlreadyReleased => {
                "stopping: nothing believed torqued and nothing outstanding, so no torque-off was \
                 written and nothing was read back; nothing automatic releases torque this process \
                 never commanded"
            }
            Self::Confirmed => "stopping: torque off, read back released on every row",
            Self::Unconfirmed => {
                "stopping: torque off written on every cycle of the confirmation budget and not \
                 read back released; the servos' own bus watchdog stops motion but does not \
                 release torque on this hardware, so treat the machine as holding torque and \
                 power it down before reaching in"
            }
        }
    }
}

/// The longest a single sleep runs before the clock is read again.
///
/// One nominal cycle: a wait to the next grid point is never longer than that in
/// ordinary running, so the chunking costs nothing there, and the one place a
/// longer wait is legitimate — the run-up to the grid's first point, which is
/// the next period boundary — is where a clock step or a stop would otherwise
/// go unnoticed for the longest. Both are read once per chunk, so neither waits
/// out a wait of any length.
const SLEEP_CHUNK_NS: i64 = reachy_driver::NOMINAL_CYCLE_NS;

/// Where the loop meets real time.
///
/// A trait rather than two calls into the clock, so that every rule this module
/// has about time — what a late cycle costs, when a window is due — is
/// stated over instants a case can name. The driver runs on [`RealTime`]; a
/// case runs on a schedule it advances itself, and neither one has a different
/// loop body.
pub trait Schedule {
    /// Now, on the clock the grid is drawn on.
    fn now_ns(&self) -> i64;

    /// Wait until `target_ns`. Returns at once if it has already passed.
    ///
    /// Answers how the wait ended, because the clock this grid is drawn on can
    /// be stepped and a wait that ended in a step is not a wait that arrived,
    /// and because a stop asked for during the wait is answered by ending it:
    /// the wait to the grid's first point is up to a whole period, and a process
    /// asked to stop owes its supervisor an answer sooner than a grid point.
    fn sleep_until(&mut self, target_ns: i64, stop: &dyn Stop) -> Waited;
}

/// How a wait ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Waited {
    /// The target instant arrived, and this is what the clock read when it did.
    ///
    /// The instant is answered rather than read again by the caller: the wait
    /// has the reading in hand at the moment it decides the target has passed,
    /// so a cycle's start is measured with a clock read that already happens
    /// instead of one added for the measurement.
    Arrived(i64),
    /// The clock went backwards while waiting, so the target is now further
    /// away in real time than the grid ever put it. The grid has to be re-drawn
    /// from where the clock says it is; waiting for the old target would leave
    /// the loop asleep for the size of the step.
    ClockSteppedBack,
    /// A stop was asked for while waiting. No cycle ran at the target; the
    /// caller winds down from where it stands.
    Stopped,
}

/// The clock and the sleep the driver actually runs on.
#[derive(Clone, Copy, Debug, Default)]
pub struct RealTime;

impl Schedule for RealTime {
    fn now_ns(&self) -> i64 {
        now_ns()
    }

    /// Sleep the distance from now to `target_ns`, in steps no longer than
    /// [`SLEEP_CHUNK_NS`].
    ///
    /// The target is absolute and is recomputed from the grid every cycle, so
    /// however long a sleep actually took, the next one is measured to a grid
    /// point and not to where the last one landed: the phase is the grid's and
    /// error does not accumulate. The clock read and the sleep are two steps,
    /// so a preemption between them lands the cycle late by that much; a cycle
    /// that lands late is already a case the loop handles, and it handles it
    /// the same way whichever step was slow.
    ///
    /// The sleep is relative, and chunked, because the grid's clock can step
    /// backwards. A relative sleep measures elapsed time, which a clock step
    /// does not change, so re-reading the clock between bounded chunks catches
    /// the step within one cycle: time going backwards between two reads is not
    /// something elapsed time does, and the wait ends there. This is the only
    /// thread that can write a torque-off sweep, so how long a backward step
    /// takes to notice is how long a machine can hold torque with nothing
    /// watching it. An absolute sleep on this clock would be worse rather than
    /// better: the kernel completes it when the clock reaches the requested
    /// instant, so a target drawn before a backward step is on the far side of
    /// it and the loop waits the step out.
    fn sleep_until(&mut self, target_ns: i64, stop: &dyn Stop) -> Waited {
        loop {
            let now = self.now_ns();
            let Some(wait) = chunk_to(now, target_ns) else {
                return Waited::Arrived(now);
            };
            if stop.asked() {
                return Waited::Stopped;
            }
            std::thread::sleep(StdDuration::from_nanos(
                u64::try_from(wait).unwrap_or_default(),
            ));
            if self.now_ns() < now {
                return Waited::ClockSteppedBack;
            }
        }
    }
}

/// How long to sleep now, waiting for `target_ns`, or `None` where it has
/// arrived.
///
/// Never more than [`SLEEP_CHUNK_NS`], so that however far away a target is the
/// clock is read again within one cycle of the loop's own period.
const fn chunk_to(now_ns: i64, target_ns: i64) -> Option<i64> {
    if target_ns <= now_ns {
        return None;
    }
    let remaining = target_ns.saturating_sub(now_ns);
    Some(if remaining < SLEEP_CHUNK_NS {
        remaining
    } else {
        SLEEP_CHUNK_NS
    })
}

/// The four loopback ports a cycle's reports go to.
///
/// Named as a value rather than read from [`crate::ports`] at each send, so
/// that a case points a driver at sockets it bound itself and the loop body it
/// exercises is the shipped one.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Destinations {
    /// Where pose samples go.
    pub pose: u16,
    /// Where driver events go.
    pub event: u16,
    /// Where auxiliary outcomes go.
    pub aux_out: u16,
    /// Where health reports go.
    pub health: u16,
    /// Where the cumulative status record goes.
    pub status: u16,
}

impl Destinations {
    /// The seam as it ships: the four ports the control process binds.
    pub const SEAM: Self = Self {
        pose: POSE_PORT,
        event: EVENT_PORT,
        aux_out: AUX_OUT_PORT,
        health: HEALTH_PORT,
        status: STATUS_PORT,
    };

    /// `port` as the loopback address a datagram is sent to.
    fn addr(port: u16) -> SocketAddr {
        SocketAddr::V4(SocketAddrV4::new(LOOPBACK, port))
    }
}

/// The sending half of the seam: one socket, four destinations.
///
/// One socket because a datagram's destination is named per send and its source
/// port is nobody's business on this seam — the receiving end binds the port
/// that says which subject it is, and nothing reads where a datagram came from.
pub struct Outbound {
    socket: UdpSocket,
    dest: Destinations,
    sent: u64,
    failures: u64,
}

impl Outbound {
    /// A sending socket on loopback, addressing `dest`.
    ///
    /// # Errors
    ///
    /// Whatever the operating system said about binding an ephemeral loopback
    /// port. A driver that cannot send its reports still de-torques on its own
    /// dead-man, but it is not a driver anybody can command, so this failure is
    /// an exit rather than a count.
    pub fn open(dest: Destinations) -> io::Result<Self> {
        let socket = UdpSocket::bind((LOOPBACK, 0))?;
        Ok(Self {
            socket,
            dest,
            sent: 0,
            failures: 0,
        })
    }

    /// Put one message on the wire as the bare bytes of its schema.
    ///
    /// No header, no length, no sequence number: the socket at the other end
    /// reads a datagram of exactly the schema's size and nothing else, so
    /// anything this added would be a datagram that end refuses.
    fn send<T: Blob>(&mut self, port: u16, message: &T) {
        match self
            .socket
            .send_to(blob_as_bytes(message), Destinations::addr(port))
        {
            Ok(_) => self.sent += 1,
            Err(_) => self.failures += 1,
        }
    }

    /// Publish everything one cycle produced.
    ///
    /// The sample first, because it is the stream a consumer dates everything
    /// else against.
    pub fn publish(&mut self, report: &CycleReport) {
        self.send(self.dest.pose, &report.sample);
        if let Some(event) = report.event.as_ref() {
            self.send(self.dest.event, event);
        }
        if let Some(outcome) = report.outcome.as_ref() {
            self.send(self.dest.aux_out, outcome);
        }
        // The served transaction's answer first: a cycle that also turned a
        // request away owes the host both, and the one it is waiting on is the
        // one it asked for.
        if let Some(turned_away) = report.turned_away.as_ref() {
            self.send(self.dest.aux_out, turned_away);
        }
        if let Some(health) = report.health.as_ref() {
            self.send(self.dest.health, health);
        }
    }

    /// Publish one event the loop raised rather than the cycle.
    pub fn publish_event(&mut self, event: &DriverEventWire) {
        self.send(self.dest.event, event);
    }

    /// Publish the driver's account of its whole run so far.
    ///
    /// Fire and forget like every other send here, and unlike the rest nothing
    /// is lost when one does not arrive: the next copy carries everything this
    /// one did.
    fn publish_status(&mut self, status: &DriverStatusWire) {
        self.send(self.dest.status, status);
    }

    /// Datagrams sent, and sends the operating system refused.
    #[must_use]
    pub fn counts(&self) -> (u64, u64) {
        (self.sent, self.failures)
    }
}

/// What the loop counted, as plain numbers since the process started.
///
/// Process-local, like every other count here: this process is not a cog and
/// has no signals. They reach an operator through [`Driver::report`].
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct LoopCounts {
    /// Cycles run.
    pub cycles: u64,
    /// Grid points passed over without running, summed. Never made up for.
    pub skipped: u64,
    /// Times the process-start release was reported, which is once on every
    /// run: the sweep is this process's first act on the bus, and the first
    /// cycle is the earliest it can be said. Zero exactly until that cycle, and
    /// what makes the publish happen once.
    pub startup_mrc: u64,
    /// Cycles on which a stopped reader was answered with a release. Every
    /// cycle for as long as one stands: a wire failure is not recovered from, so
    /// the state it forces must not be one a datagram on the surviving port can
    /// leave.
    pub wire_failures: u64,
    /// Datagrams taken off the seam and handed to the cycle.
    pub taken: u64,
    /// Times the clock stepped backwards, the machine was let go, and the grid
    /// was re-drawn from where the clock then was. Grid instants either side of
    /// one of these do not line up.
    pub clock_steps: u64,
}

impl LoopCounts {
    /// Every count, as labelled numbers for a log line.
    #[must_use]
    pub fn read(&self) -> [(&'static str, u64); 6] {
        [
            ("cycles", self.cycles),
            ("skipped", self.skipped),
            ("startup_mrc", self.startup_mrc),
            ("wire_failures", self.wire_failures),
            ("taken", self.taken),
            ("clock_steps", self.clock_steps),
        ]
    }
}

/// One event the loop raises about itself, as the datagram carrying it.
///
/// Built through the driver layer's own event rather than by assigning the
/// schema's fields here: a field the vocabulary grows is then a field every
/// raiser in this tree gets, instead of one more place that has to remember it.
fn raised(event: &Event) -> DriverEventWire {
    let mut message = DriverEventWire::new();
    event.write(message.clear_valid());
    message
}

/// What the process-start release established, carried into the loop.
///
/// The instant and the rows together, because the two are read as one fact: a
/// sweep that ran at this instant and left these rows unverified. The loop
/// publishes it as an event on the first cycle and in every status record
/// afterwards, so a reader that missed the event still holds it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StartupMrc {
    /// When the sweep ran.
    pub at_ns: i64,
    /// The rows whose verified torque-off write did not read back. Empty on a
    /// healthy start.
    pub failed: JointFlags,
}

/// The driver's loop: a cycle, a grid, a seam and a clock.
pub struct Driver<P: BusPort, S: Schedule> {
    tick: Tick<P>,
    inbox: Inbox,
    out: Outbound,
    grid: Grid,
    schedule: S,
    /// The cycle the next call will run.
    next: Cycle,
    /// What this process established about torque before the loop was built.
    /// Published on the first cycle and carried in every status record; never
    /// acted on, because the release has already happened.
    startup: StartupMrc,
    /// When the first pose sample was published, and nothing until one has
    /// been.
    first_pose_ns: Option<i64>,
    /// When the first session command was taken off the seam, and nothing until
    /// one has been.
    first_session_cmd_ns: Option<i64>,
    /// What the cycle before this one spent, where one has been measured. It is
    /// the previous cycle's number because the skip it explains is reported by
    /// the cycle after it: the grid points that went by were the ones the
    /// overlong cycle ran through.
    last_work_ns: Option<i64>,
    /// What the cycles since the last statistics event cost.
    window: CycleWindow,
    counts: LoopCounts,
}

/// The worst of what a run of cycles cost, until it is published and started
/// again.
///
/// A window rather than a per-cycle stream: fifty events a second on a channel
/// a whole run is read off would be the measurement drowning what it measures,
/// and the question the measurement exists for -- whether cycles are running
/// past their slot, and whether the out-of-band work is why -- is one the worst
/// case of a second answers.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct CycleWindow {
    /// Cycles counted since the last publication.
    cycles: u32,
    /// How many of them ran an out-of-band transaction.
    aux_cycles: u32,
    /// The worst a cycle in the window measured: the longest of them, or a
    /// negative span where one came out negative.
    worst_work_ns: i64,
    /// The worst an out-of-band exchange in the window measured, read the same
    /// way.
    worst_aux_ns: i64,
    /// The worst single write call a cycle in the window spent. A wall-clock
    /// span of one call and never negative, so the longest of them is the worst
    /// and there is nothing here for `worst_of`'s backwards-clock reading to
    /// answer.
    worst_drain_ns: i64,
}

impl CycleWindow {
    /// Fold one cycle in.
    fn note(&mut self, work_ns: i64, aux_span_ns: Option<i64>, drain_ns: i64) {
        self.worst_work_ns = worst_of(self.worst_work_ns, work_ns, self.cycles == 0);
        self.worst_drain_ns = self.worst_drain_ns.max(drain_ns);
        self.cycles = self.cycles.saturating_add(1);
        if let Some(aux_ns) = aux_span_ns {
            self.worst_aux_ns = worst_of(self.worst_aux_ns, aux_ns, self.aux_cycles == 0);
            self.aux_cycles = self.aux_cycles.saturating_add(1);
        }
    }
}

/// What a cycle's bus work cost, as the loop folds it into a window.
///
/// Two spans of two different shapes: the out-of-band exchange is optional
/// because most cycles run none, and the write span is not because every cycle
/// puts at least the grouped read's request on the wire.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Spans {
    /// How long this cycle's out-of-band exchange took, where it ran one.
    aux_span_ns: Option<i64>,
    /// The longest single write call this cycle spent.
    drain_ns: i64,
}

/// The worse of two measured spans, `first` saying that `held` is the zero a
/// window starts at rather than a reading.
///
/// Longer is worse among spans a clock that moved forwards produced, and a
/// negative span is worse than any of them: it says the clock the driver
/// measures on went backwards between the two reads that bracket the span,
/// which is loss of the time base every de-torquing timer runs on and the one
/// reading a window must not round away. So a negative span holds the window
/// against every ordinary cycle after it, and the most negative of several
/// holds it against the rest.
fn worst_of(held: i64, span: i64, first: bool) -> i64 {
    if first {
        span
    } else if held < 0 || span < 0 {
        held.min(span)
    } else {
        held.max(span)
    }
}

impl<P: BusPort, S: Schedule> Driver<P, S> {
    /// A loop running `tick` on `grid`, reading `inbox` and publishing on `out`.
    ///
    /// `startup` is what the caller established when it wrote the minimum risk
    /// condition on the bus, which every start does before a loop is built. It
    /// is carried rather than re-derived because the first cycle publishes it,
    /// and a record dated at the grid point would say the release happened later
    /// than it did.
    pub fn new(
        tick: Tick<P>,
        inbox: Inbox,
        out: Outbound,
        grid: Grid,
        schedule: S,
        startup: StartupMrc,
    ) -> Self {
        let next = grid.first_from(schedule.now_ns());
        Self {
            tick,
            inbox,
            out,
            grid,
            schedule,
            next,
            startup,
            first_pose_ns: None,
            first_session_cmd_ns: None,
            last_work_ns: None,
            window: CycleWindow::default(),
            counts: LoopCounts::default(),
        }
    }

    /// Run `cycles` grid cycles, sleeping to each one.
    ///
    /// Bounded rather than endless so that the caller owns the reporting
    /// interval and a case owns how far it runs. The grid position carries
    /// across calls: a driver that ran two hundred cycles and then two hundred
    /// more has run four hundred cycles of one grid, not two grids of two
    /// hundred.
    pub fn run_cycles(&mut self, cycles: u64) {
        let _ = self.run_until(cycles, &());
    }

    /// Run `cycles` grid cycles, or wind the machine down if `stop` is asked for.
    ///
    /// The flag is read once per cycle, on this thread. Nothing about a stop
    /// happens anywhere else: a handler stores, and every consequence — the
    /// sweep, the confirmation, the reading of what came back — runs here,
    /// because this is the only thread in this process that may write to the
    /// bus.
    ///
    /// It is read before the cycle rather than after, so a stop asked for during
    /// a sleep is answered without the intervening cycle publishing a sample
    /// nobody will read. The wait itself reads it too, once per sleep chunk, so
    /// a stop is answered inside a cycle however far off the next grid point
    /// is rather than leaving a supervisor waiting for it.
    pub fn run_until(&mut self, cycles: u64, stop: &impl Stop) -> Ran {
        for _ in 0..cycles {
            if stop.asked() {
                return Ran::WoundDown(self.wind_down());
            }
            if self.cycle_once(stop) == Stepped::Stopped {
                return Ran::WoundDown(self.wind_down());
            }
        }
        Ran::Cycles
    }

    /// Sleep to the next grid point and run the cycle there, or re-draw the grid
    /// if the clock stepped backwards instead.
    ///
    /// Both `run_until` and `wind_down` run this; the wind-down path is the one
    /// that writes the torque-off sweep. `stop` is what the wait watches: the
    /// running loop hands its flag over, and the wind-down hands over the flag
    /// that never asks, because the stop it is already answering must not cut
    /// short the cycles that de-torque the machine.
    fn cycle_once(&mut self, stop: &dyn Stop) -> Stepped {
        let cycle = self.next;
        let started_ns = match self.schedule.sleep_until(cycle.nominal_ns, stop) {
            Waited::ClockSteppedBack => {
                self.reanchor();
                return Stepped::Reanchored;
            }
            Waited::Stopped => return Stepped::Stopped,
            Waited::Arrived(now_ns) => now_ns,
        };
        let stepped = self.step(cycle);
        // The read that picks the next grid point is also the cycle's end: what
        // the loop spent is the distance between the two reads the cycle
        // already takes, and neither of them is here for the measurement.
        let ended_ns = self.schedule.now_ns();
        self.note_cycle(cycle, ended_ns.saturating_sub(started_ns), stepped);
        self.next = self.grid.next_after(&cycle, ended_ns);
        Stepped::Advanced
    }

    /// De-torque the machine, read it back, and hand the loop over.
    ///
    /// What an operator's stop gesture deserves: control is still trusted, so
    /// the release is a commanded one and the driver stays on the bus long
    /// enough to see it take. Nothing here gates the sweep — the write goes out
    /// on the first cycle of the wind-down and on every cycle after it — and
    /// nothing recovers anything: a wind-down that cannot confirm says so and
    /// ends, and what is left is a servo-side watchdog that stops motion
    /// without releasing torque, so the machine is to be treated as holding.
    ///
    /// The bound is the confirmation's own budget plus a cycle of margin, so a
    /// wind-down ends in about four hundred milliseconds at worst — well inside
    /// the escalation any supervisor gives a process it has asked to stop.
    fn wind_down(&mut self) -> WoundDown {
        let how = self.release();
        // The one copy that says the run finished, and the last thing this loop
        // publishes. Every fact in it is in the periodic copies too, bar the
        // flag: a run whose final copy is lost reads as a run that stopped
        // mid-window, which is what a run that was killed also reads as.
        //
        // Dated off the clock rather than off the grid. The grid point in hand
        // is the next one, which no cycle will now run, and a record stamped
        // there postdates every message the run published; the reason the
        // periodic copies take no clock read of their own -- that the read would
        // land inside the span a cycle is measured over -- does not apply out
        // here, where no cycle is being measured.
        let at_ns = self.schedule.now_ns();
        self.publish_status(at_ns, true);
        how
    }

    /// De-torque and read back, leaving the record of it to the caller.
    fn release(&mut self) -> WoundDown {
        if !self.tick.torque_outstanding() {
            return WoundDown::AlreadyReleased;
        }
        let cycle = self.next;
        self.tick.request_torque_off(cycle.nominal_ns);
        let period_ns = self.grid.period_ns().max(1);
        let bound = TORQUE_OFF_CONFIRM_BUDGET_NS / period_ns + 2;
        for _ in 0..bound {
            if self.cycle_once(&()) == Stepped::Reanchored {
                continue;
            }
            match self.tick.confirm_said() {
                Some(ConfirmReport::Confirmed) => return WoundDown::Confirmed,
                Some(ConfirmReport::Unconfirmed) => return WoundDown::Unconfirmed,
                _ => {}
            }
        }
        // The pass reports as soon as it is overdue, so the bound is reached
        // only by a clock that stepped often enough to keep re-opening it. A
        // machine whose time base is being lost is not one this process can say
        // anything confirmed about.
        WoundDown::Unconfirmed
    }

    /// Re-draw the grid from where the clock now says it is.
    ///
    /// What a backwards clock step leaves to do. The old grid's remaining points
    /// are all in the future by the size of the step, so keeping it would stop
    /// the loop for that long — and this is the only thread that can de-torque
    /// the machine. A new grid at the next period boundary is the same
    /// construction the process started with, so a log can be read against it
    /// the same way, and the step is counted because a run whose grid was
    /// re-drawn has instants in it that do not line up with the ones before.
    ///
    /// And the machine is let go, first and unconditionally. Every timer that can
    /// de-torque this machine is a difference against the clock that just moved —
    /// the dead-man's silence, the confirmation pass's budget — so instants stored
    /// before the step sit in the future of every instant after it, and a
    /// difference that has gone negative expires nothing. A backwards step is
    /// therefore loss of the time base the safety timers are measured on, and the
    /// answer to losing it is the minimum risk condition rather than a suspended
    /// dead-man: no timer that can de-torque this machine is left suspended for
    /// the size of a step, because the release does not wait for one. Nothing
    /// recovers it; a host that arms again re-stamps its own liveness on the
    /// re-drawn grid, which is a fresh engagement and not a recovery.
    fn reanchor(&mut self) {
        let period_ns = self.grid.period_ns();
        let now = self.schedule.now_ns();
        self.counts.clock_steps += 1;
        self.tick.request_torque_off(now);
        let Ok(grid) = Grid::new(Grid::top_of_period_at(now, period_ns), period_ns) else {
            // The period came from a grid, so it is positive. Nothing to do
            // rather than a panic: this thread's job is to keep de-torquing
            // available.
            return;
        };
        // Neither the part-built window nor the last cycle's span survives a
        // re-drawn grid: both are spans on a time base that has just moved, and
        // a worst case measured across a step is a measurement of the step.
        self.window = CycleWindow::default();
        self.last_work_ns = None;
        self.grid = grid;
        self.next = grid.first_from(now);
    }

    /// One cycle, in the order its parts have to happen in.
    ///
    /// The skip report first, because it is about the cycles between this one
    /// and the last; then the seam, so the cycle acts on everything that has
    /// arrived; then the two conditions the cycle cannot see for itself; then
    /// the cycle, and its reports.
    fn step(&mut self, cycle: Cycle) -> Spans {
        self.counts.cycles += 1;
        if cycle.skipped > 0 {
            self.counts.skipped += u64::from(cycle.skipped);
            let late = i64::from(cycle.skipped) * self.grid.period_ns();
            self.out.publish_event(&raised(&Event {
                kind: EventKind::CycleSkipped,
                silence_ns: late,
                // What the cycle that ran through those grid points spent. Zero
                // where no cycle has been measured on this grid yet, which is
                // the same zero every unmeasured field on this record carries;
                // the first cycle of a grid and the first after one was
                // re-drawn both report no skip, so a run leaves that state
                // before it can say this.
                work_ns: self.last_work_ns.unwrap_or_default(),
                count: cycle.skipped,
                ..Event::at(cycle.nominal_ns)
            }));
        }
        self.take_inbound(cycle.nominal_ns);
        self.answer_wire_failure(cycle.nominal_ns);
        self.report_startup_mrc();
        let report = self.tick.run(cycle.nominal_ns);
        self.out.publish(&report);
        if self.first_pose_ns.is_none() {
            self.first_pose_ns = Some(cycle.nominal_ns);
            // The first status goes out here rather than waiting for the first
            // window, so the startup record and the two first-contact instants
            // exist on the channel from the run's first instant.
            self.publish_status(cycle.nominal_ns, false);
        }
        Spans {
            aux_span_ns: report.aux_span_ns,
            drain_ns: report.drain_ns,
        }
    }

    /// Fold what a cycle cost into the window, and publish the window when it
    /// is a second old.
    ///
    /// The cadence is counted in cycles rather than measured against the clock:
    /// the grid is what a window of cycles is a window of, so a run whose cycles
    /// are running late publishes on the same count of cycles and says they
    /// were late, instead of publishing more often the worse it gets.
    fn note_cycle(&mut self, cycle: Cycle, work_ns: i64, stepped: Spans) {
        self.last_work_ns = Some(work_ns);
        self.window
            .note(work_ns, stepped.aux_span_ns, stepped.drain_ns);
        let per_second = (NANOS_PER_SECOND / self.grid.period_ns().max(1)).max(1);
        if i64::from(self.window.cycles) < per_second {
            return;
        }
        let window = std::mem::take(&mut self.window);
        self.out.publish_event(&raised(&Event {
            kind: EventKind::CycleStats,
            work_ns: window.worst_work_ns,
            exchange_ns: window.worst_aux_ns,
            drain_ns: window.worst_drain_ns,
            count: window.cycles,
            out_of_band: window.aux_cycles,
            ..Event::at(cycle.nominal_ns)
        }));
        // On the window's own cadence rather than a timer of its own: the
        // record is republished so that a reader who arrived late has it, and a
        // second of staleness is what that costs.
        self.publish_status(cycle.nominal_ns, false);
    }

    /// Hand everything waiting on the seam to the cycle.
    ///
    /// Both kinds are already known to be the right size and to validate — the
    /// reader thread refused anything that was not — so the view taken here
    /// cannot fail. It is taken rather than assumed anyway, and a message that
    /// somehow failed it is dropped rather than raised: this process's job is to
    /// keep de-torquing available, not to be right about its own queue.
    fn take_inbound(&mut self, nominal_ns: i64) {
        let Self {
            tick,
            inbox,
            counts,
            first_session_cmd_ns,
            ..
        } = self;
        let taken = inbox.drain(|message| match message {
            Inbound::Goal(wire) => {
                if let Ok(goal) = wire.validate() {
                    tick.offer_goal(goal, nominal_ns);
                }
            }
            Inbound::Session(wire) => {
                if let Ok(cmd) = wire.validate() {
                    // Stamped at the drain rather than at whatever the host
                    // said, and on every datagram that decodes including one the
                    // cycle turns away: what this records is when the driver
                    // first heard from the host, which is the instant the
                    // survey-order check is read against. One that does not
                    // decode never reaches here -- the reader refuses it at the
                    // seam and counts it there.
                    first_session_cmd_ns.get_or_insert(nominal_ns);
                    tick.offer_session_cmd(cmd, nominal_ns);
                }
            }
        });
        counts.taken += taken as u64;
    }

    /// De-torque the machine if a port has stopped being read.
    ///
    /// The event vocabulary has no kind for this and none is invented: it
    /// surfaces as a count and a log line, which is what the driver's own
    /// numbers do when the schema has no word for them. What it must not do is
    /// nothing — a session port with no reader is a machine that cannot be
    /// released by being asked.
    ///
    /// Every cycle the condition stands, and not once. The two ports have
    /// separate readers, so a stopped goal reader leaves a live session port
    /// through which the machine can be armed — a verified torque-enable write
    /// releases the torque-off latch, and keep-alives on that same port then
    /// hold the dead-man off, leaving a machine energised behind a seam whose
    /// goals reach nobody. Re-asking on every cycle is what makes the state a
    /// wire failure forces one that cannot be left, which is what "not recovered
    /// from" has to mean if it means anything.
    fn answer_wire_failure(&mut self, nominal_ns: i64) {
        if !self.inbox.reader_stopped() {
            return;
        }
        self.counts.wire_failures += 1;
        self.tick.request_torque_off(nominal_ns);
    }

    /// Say that this process wrote the minimum risk condition before it started
    /// cycling.
    ///
    /// A record, not an action: a restarted driver does not know what state it
    /// found the machine in, so it released the machine before this loop
    /// existed, and the first cycle is the earliest anything can be published.
    /// Dated at the sweep rather than at the grid point, and carrying no silence
    /// figure — nothing was waited for.
    ///
    /// The counter is what says it has gone out: the release happens once a run
    /// and this publishes it once, so a count of zero is the only state in which
    /// there is anything to publish. A flag beside it would be a second piece of
    /// state saying the same thing, able to disagree with the number the record
    /// carries.
    fn report_startup_mrc(&mut self) {
        if self.counts.startup_mrc > 0 {
            return;
        }
        self.counts.startup_mrc += 1;
        self.out.publish_event(&raised(&Event {
            kind: EventKind::StartupMrcWrite,
            ..Event::at(self.startup.at_ns)
        }));
    }

    /// Compose and publish the driver's account of its run so far.
    ///
    /// Every number this process has, cumulative, in one record: what the seam
    /// counted, what the cycles counted, what the loop counted, what the sending
    /// half sent, when the machine was released and which rows would not read
    /// back, and when the driver first published a sample and first heard from
    /// the host. The whole point is that it is cumulative — a reader that has
    /// seen exactly one copy has read the run, so it does not matter which copy
    /// it saw or how many it missed.
    ///
    /// `at_ns` is when the record was composed. On a periodic copy that is the
    /// cycle's grid instant -- the same clock every event carries, and no read
    /// of its own, because a read here would land inside the span the cycle
    /// around it is measured over. The wind-down's copy is composed outside any
    /// cycle and carries the clock instead. `wound_down` is true on that one
    /// copy and false on every periodic one.
    fn publish_status(&mut self, at_ns: i64, wound_down: bool) {
        let mut message = DriverStatusWire::new();
        {
            let status = message.clear_valid();
            status.time = SyncTime::from_nanos(at_ns);
            status.sweep_time = SyncTime::from_nanos(self.startup.at_ns);
            status.sweep_failed_rows = self.startup.failed;
            status.torque_latched = self.tick.torque_latched().into();
            // Zero for an instant that has not happened yet, which is what the
            // schema says an unset one carries: a driver that has published no
            // sample has no instant to report, and a made-up one would be read
            // as an ordering.
            status.first_pose = SyncTime::from_nanos(self.first_pose_ns.unwrap_or(0));
            status.first_session_cmd = SyncTime::from_nanos(self.first_session_cmd_ns.unwrap_or(0));
            status.wound_down = wound_down.into();
            // Read one at a time: the readers keep counting while this
            // runs, so the record is an account of the run and not an
            // instant's snapshot of it.
            let counts = self.inbox.counts();
            let seam = &mut status.seam;
            seam.queued = counts.queued.load(Ordering::Relaxed);
            seam.goals = counts.goals.load(Ordering::Relaxed);
            seam.session_cmds = counts.session_cmds.load(Ordering::Relaxed);
            seam.wrong_size = counts.wrong_size.load(Ordering::Relaxed);
            seam.invalid = counts.invalid.load(Ordering::Relaxed);
            seam.overflowed = counts.overflowed.load(Ordering::Relaxed);
            seam.undelivered = counts.undelivered.load(Ordering::Relaxed);
            seam.recv_errors = counts.recv_errors.load(Ordering::Relaxed);
            seam.readers_stopped = counts.readers_stopped.load(Ordering::Relaxed);
            let ticked = self.tick.counts();
            let cycle = &mut status.cycle;
            cycle.goals_executed = ticked.goals_executed;
            cycle.goals_dropped = ticked.goals_dropped;
            cycle.hold_timeouts = ticked.hold_timeouts;
            cycle.read_misses = ticked.read_misses;
            cycle.write_failures = ticked.write_failures;
            cycle.blind_cycles = ticked.blind_cycles;
            cycle.events_dropped = ticked.events_dropped;
            cycle.aux_refused = ticked.aux_refused;
            cycle.aux_duplicates = ticked.aux_duplicates;
            cycle.aux_deferred = ticked.aux_deferred;
            cycle.health_reports = ticked.health_reports;
            cycle.health_misses = ticked.health_misses;
            cycle.confirm_misses = ticked.confirm_misses;
            let looped = &mut status.loop_counts;
            looped.cycles = self.counts.cycles;
            looped.skipped = self.counts.skipped;
            looped.startup_mrc = self.counts.startup_mrc;
            looped.wire_failures = self.counts.wire_failures;
            looped.taken = self.counts.taken;
            looped.clock_steps = self.counts.clock_steps;
            // The record's own send is not in these two: it has not happened
            // yet. Every copy is behind by the sends made since the one before
            // it, which is what "cumulative up to this instant" means.
            let (sent, failures) = self.out.counts();
            status.published = sent;
            status.publish_failures = failures;
        }
        self.out.publish_status(&message);
    }

    /// What the loop has counted.
    #[must_use]
    pub fn counts(&self) -> LoopCounts {
        self.counts
    }

    /// What the cycle has counted.
    #[must_use]
    pub fn tick_counts(&self) -> TickCounts {
        self.tick.counts()
    }

    /// The seam's own counts, as a handle that outlives this loop.
    #[must_use]
    pub fn inbound_counts(&self) -> Arc<Counts> {
        self.inbox.counts_handle()
    }

    /// Every number this process has, as one line.
    ///
    /// The whole of the driver's voice for everything the event vocabulary has
    /// no kind for: the loop's own counts, the cycle's, the seam's, the sending
    /// half's, and what was wrong with the last datagram refused. Printed by
    /// whoever runs the loop, on whatever interval it chooses.
    ///
    /// It opens with where on the grid the loop has got to — the next cycle's
    /// index and its instant — because every other number here is cumulative
    /// since the process started. Without them a line can only be placed in time
    /// by counting lines, and a run's counters cannot be lined up against the
    /// samples and events published on the seam, which carry the grid instant
    /// they were raised at.
    #[must_use]
    pub fn report(&self) -> String {
        let mut line = String::new();
        let _ = write!(
            line,
            "cycle={} nominal={} ",
            self.next.index, self.next.nominal_ns
        );
        for (label, value) in self.counts.read() {
            let _ = write!(line, "{label}={value} ");
        }
        for (label, value) in self.tick_counts().read() {
            let _ = write!(line, "{label}={value} ");
        }
        for (label, value) in self.inbox.counts().read() {
            let _ = write!(line, "{label}={value} ");
        }
        let (sent, failures) = self.out.counts();
        let _ = write!(line, "published={sent} publish_failures={failures}");
        if let Some(rejected) = self.inbox.counts().last_refusal() {
            let _ = write!(line, " last_refusal=[{rejected}]");
        }
        line
    }
}

#[cfg(test)]
mod tests {
    use super::{
        AtomicBool, CycleWindow, Destinations, Driver, LoopCounts, Outbound, Ran, SLEEP_CHUNK_NS,
        Schedule, StartupMrc, Stop, TORQUE_OFF_CONFIRM_BUDGET_NS, Waited, WoundDown, chunk_to,
    };
    use crate::grid::Grid;
    use crate::inbound::{Counts, Inbox};
    use crate::tick::{CycleReport, Tick, TickConfig, TickCounts, cycle_timing};
    use brenn_reachy__cogs__session_cmd_clk_rs::{SessionCmdKind, SessionCmdWire};
    use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
    use brenn_reachy__driver__health_clk_rs::{
        AuxOutcomeWire, DriverEventWire, DriverStatusWire, EventKind, HealthReportWire,
    };
    use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
    use brenn_reachy__hardware__dynamixel__registers_clk_rs::RegId;
    use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKind;
    use brenn_reachy__motion__joints_clk_rs::JointFlags;
    use clockwork_rs::{Blob, SyncTime, blob_as_bytes, blob_from_bytes};
    use dxl_proto::crc16;
    use dxl_proto::frame::{
        HEADER, INST_READ, INST_STATUS, INST_SYNC_READ, INST_SYNC_WRITE, INST_WRITE,
    };
    use dxl_proto::regs::TORQUE_ENABLE;
    use reachy_bus::{Bus, BusPort, DEFAULT_BAUD};
    use reachy_motion::joints::ROW_COUNT;
    use reachy_motion::value;
    use std::cell::{Cell, RefCell};
    use std::collections::{HashMap, VecDeque};
    use std::io;
    use std::net::UdpSocket;
    use std::rc::Rc;
    use std::time::{Duration, Instant};

    /// A round instant a second before the grid's start, so a number read out of
    /// the wrong side of the startup arithmetic is visible.
    const T0: i64 = 1_700_000_000_000_000_000;

    /// The grid these cases run on: the shipped cycle.
    const PERIOD: i64 = reachy_driver::NOMINAL_CYCLE_NS;

    /// How long before the grid's first point these cases' driver wrote the
    /// minimum risk condition: half a period, so a record dated at the sweep is
    /// distinguishable from one dated at a grid point.
    const SWEPT_BEFORE: i64 = PERIOD / 2;

    /// A bus nothing answers on.
    ///
    /// Every exchange is a miss, which is the state a driver starting up next to
    /// an unpowered machine is in, and the state these cases want: they are
    /// about the loop, and a cycle whose bus work is entirely misses still
    /// produces the sample and the reports the loop is here to publish.
    struct Silent;

    impl BusPort for Silent {
        fn write_all(&mut self, _buf: &[u8]) -> io::Result<()> {
            Ok(())
        }

        fn read_some(&mut self, _buf: &mut [u8], _deadline: Instant) -> io::Result<usize> {
            // The one and only way this seam reports silence.
            Ok(0)
        }

        fn discard_input(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    /// A bus with a control table: a write is stored and a read answers what is
    /// stored, zero for anything never written.
    ///
    /// The little a wind-down needs of a machine. Reads of a register nothing
    /// wrote come back zero, which is a de-torqued machine, so a confirmation
    /// pass over an untouched table completes; and a verified torque-enable
    /// write reads back what it wrote, which is how a case gets the driver to
    /// believe a row is holding. The table is shared with the case that built it
    /// — the driver owns the port — so what reached the machine is readable
    /// after a run.
    ///
    /// TODO(shared-servo-fixture): the third scripted servo model in this tree,
    /// and the two others can disagree with it about what a write does.
    /// One servo's registers, by address.
    type Table = HashMap<u16, Vec<u8>>;

    /// One write as it reached a control table: the servo, the address, the
    /// bytes.
    type Wrote = (u8, u16, Vec<u8>);

    #[derive(Clone, Default)]
    struct Limp {
        out: VecDeque<u8>,
        /// The control tables, by servo.
        regs: Rc<RefCell<HashMap<u8, Table>>>,
        /// Every write that reached one, in order.
        wrote: Rc<RefCell<Vec<Wrote>>>,
    }

    impl Limp {
        /// How many writes of `byte` to `reg` this machine has taken.
        fn writes_of(&self, reg: dxl_proto::Reg, byte: u8) -> usize {
            self.wrote
                .borrow()
                .iter()
                .filter(|(_, addr, bytes)| *addr == reg.addr && bytes.as_slice() == [byte])
                .count()
        }

        /// Store one write, both in the table and on the record of what arrived.
        fn store(&mut self, id: u8, addr: u16, bytes: &[u8]) {
            self.wrote.borrow_mut().push((id, addr, bytes.to_vec()));
            self.regs
                .borrow_mut()
                .entry(id)
                .or_default()
                .insert(addr, bytes.to_vec());
        }

        /// What a read of one register answers: what is stored, zero-extended,
        /// or zeros for a register nothing has written.
        fn stored(&self, id: u8, addr: u16, width: usize) -> Vec<u8> {
            let mut value = self
                .regs
                .borrow()
                .get(&id)
                .and_then(|table| table.get(&addr))
                .cloned()
                .unwrap_or_default();
            value.resize(width, 0);
            value
        }

        /// A status frame carrying `params`, as a servo puts one on the wire.
        fn reply(&mut self, id: u8, params: &[u8]) {
            let mut frame = Vec::from(HEADER);
            frame.push(id);
            let len = u16::try_from(params.len() + 4).expect("a fixture reply is short");
            frame.extend_from_slice(&len.to_le_bytes());
            frame.push(INST_STATUS);
            frame.push(0);
            frame.extend_from_slice(params);
            frame.extend_from_slice(&crc16(&frame).to_le_bytes());
            self.out.extend(frame);
        }
    }

    impl BusPort for Limp {
        fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
            let id = buf[4];
            let len = usize::from(u16::from_le_bytes([buf[5], buf[6]]));
            let instruction = buf[7];
            let params = &buf[8..8 + len - 3];
            let addr = u16::from_le_bytes([params[0], params[1]]);
            match instruction {
                INST_READ => {
                    let width = usize::from(u16::from_le_bytes([params[2], params[3]]));
                    let value = self.stored(id, addr, width);
                    self.reply(id, &value);
                }
                INST_SYNC_READ => {
                    let width = usize::from(u16::from_le_bytes([params[2], params[3]]));
                    for asked in params[4..].iter().copied() {
                        let value = self.stored(asked, addr, width);
                        self.reply(asked, &value);
                    }
                }
                // A unicast write is acknowledged with no parameters; a grouped
                // one is acknowledged by nothing at all.
                INST_WRITE => {
                    self.store(id, addr, &params[2..]);
                    self.reply(id, &[]);
                }
                INST_SYNC_WRITE => {
                    let width = usize::from(u16::from_le_bytes([params[2], params[3]]));
                    for entry in params[4..].chunks_exact(1 + width) {
                        self.store(entry[0], addr, &entry[1..]);
                    }
                }
                _ => {}
            }
            Ok(())
        }

        fn read_some(&mut self, buf: &mut [u8], _deadline: Instant) -> io::Result<usize> {
            let mut taken = 0;
            while taken < buf.len() {
                match self.out.pop_front() {
                    Some(byte) => {
                        buf[taken] = byte;
                        taken += 1;
                    }
                    None => break,
                }
            }
            Ok(taken)
        }

        fn discard_input(&mut self) -> io::Result<()> {
            self.out.clear();
            Ok(())
        }
    }

    /// A clock a case advances: a sleep arrives exactly on its target, plus
    /// however long the case says a cycle overran by.
    #[derive(Clone)]
    struct Fake {
        now: Rc<Cell<i64>>,
        overrun_ns: Rc<Cell<i64>>,
        /// How far the clock goes backwards on the next wait, once.
        step_back_ns: Rc<Cell<Option<i64>>>,
        /// How far it goes backwards on every wait, for as long as it is set:
        /// a time base that keeps being lost rather than one that slipped once.
        back_every_ns: Rc<Cell<Option<i64>>>,
        /// How far the clock moves on each read of it. Zero by default, which
        /// is a case that spends no time inside a cycle; a case measuring what a
        /// cycle cost sets it, because the loop reads this clock once per cycle
        /// after the cycle's work.
        per_read_ns: Rc<Cell<i64>>,
    }

    impl Fake {
        fn at(now: i64) -> Self {
            Self {
                now: Rc::new(Cell::new(now)),
                overrun_ns: Rc::new(Cell::new(0)),
                step_back_ns: Rc::new(Cell::new(None)),
                back_every_ns: Rc::new(Cell::new(None)),
                per_read_ns: Rc::new(Cell::new(0)),
            }
        }
    }

    impl Schedule for Fake {
        fn now_ns(&self) -> i64 {
            self.now.set(self.now.get() + self.per_read_ns.get());
            self.now.get()
        }

        fn sleep_until(&mut self, target_ns: i64, stop: &dyn Stop) -> Waited {
            // Answered before the clock moves, so a case can see that the
            // wait did not run the target out.
            if stop.asked() {
                return Waited::Stopped;
            }
            if let Some(back_ns) = self
                .step_back_ns
                .take()
                .or_else(|| self.back_every_ns.get())
            {
                // What the shipped schedule answers when the clock it reads goes
                // backwards while it is waiting.
                self.now.set(self.now.get() - back_ns);
                return Waited::ClockSteppedBack;
            }
            self.now
                .set(target_ns.max(self.now.get()) + self.overrun_ns.get());
            Waited::Arrived(self.now.get())
        }
    }

    /// A socket bound to a free loopback port, and the port it got.
    fn listener() -> (UdpSocket, u16) {
        let socket = UdpSocket::bind(("127.0.0.1", 0)).expect("a free port");
        socket
            .set_read_timeout(Some(Duration::from_millis(200)))
            .expect("a read timeout");
        let port = socket.local_addr().expect("bound").port();
        (socket, port)
    }

    /// Whatever the loop published on `socket`, as whole datagrams.
    fn published(socket: &UdpSocket) -> Vec<Vec<u8>> {
        let mut seen = Vec::new();
        let mut buffer = [0u8; 4096];
        while let Ok((read, _from)) = socket.recv_from(&mut buffer) {
            seen.push(buffer[..read].to_vec());
        }
        seen
    }

    /// One decoded sample out of a published datagram.
    fn as_sample(bytes: &[u8]) -> PoseSampleWire {
        assert_eq!(bytes.len(), PoseSampleWire::SIZE, "the schema's own size");
        blob_from_bytes::<PoseSampleWire>(bytes).expect("the schema's own size")
    }

    /// Whether the last sample published on `socket` says the machine has been
    /// let go: the consequence of a release, as the seam reports it.
    fn latched_in_last_sample(socket: &UdpSocket) -> bool {
        let samples = published(socket);
        let last = samples.last().expect("a sample every cycle");
        bool::from(
            as_sample(last)
                .validate()
                .expect("a published sample validates")
                .torque_off_latched,
        )
    }

    /// One decoded event out of a published datagram.
    fn as_event(bytes: &[u8]) -> DriverEventWire {
        assert_eq!(bytes.len(), DriverEventWire::SIZE, "the schema's own size");
        blob_from_bytes::<DriverEventWire>(bytes).expect("the schema's own size")
    }

    /// The four listeners a driver publishes to, and the two ports it is
    /// commanded on.
    struct Seam {
        pose: UdpSocket,
        event: UdpSocket,
        status: UdpSocket,
        goal_port: u16,
        session_port: u16,
    }

    /// Every status record published on `socket`.
    fn statuses(socket: &UdpSocket) -> Vec<DriverStatusWire> {
        published(socket)
            .iter()
            .map(|bytes| {
                assert_eq!(bytes.len(), DriverStatusWire::SIZE, "the schema's own size");
                blob_from_bytes::<DriverStatusWire>(bytes).expect("the schema's own size")
            })
            .collect()
    }

    /// A driver over a silent bus, publishing to sockets this case holds and
    /// reading two it bound.
    ///
    /// Every port is one the operating system chose: a case that asked for the
    /// shipped numbers would be a case that fails when another one runs beside
    /// it, and the seam's layout is [`crate::ports`]'s claim rather than this
    /// module's.
    fn driver(start_ns: i64, now_ns: i64) -> (Driver<Silent, Fake>, Seam, Fake) {
        driver_over(Silent, start_ns, now_ns)
    }

    /// As [`driver`], over a bus a case names — for the cases about what the loop
    /// concludes from what came back.
    ///
    /// The process-start release is not run here: what a caller's sweep left
    /// behind is the case's to choose, and a loop built over a machine that
    /// verified every row is what a healthy start hands this constructor.
    fn driver_over<P: BusPort>(
        port: P,
        start_ns: i64,
        now_ns: i64,
    ) -> (Driver<P, Fake>, Seam, Fake) {
        driver_built(port, start_ns, now_ns, false)
    }

    /// As [`driver_over`], with the process-start release actually run against
    /// the case's bus — for the cases about a start that could not verify one.
    fn driver_swept_over<P: BusPort>(
        port: P,
        start_ns: i64,
        now_ns: i64,
    ) -> (Driver<P, Fake>, Seam, Fake) {
        driver_built(port, start_ns, now_ns, true)
    }

    fn driver_built<P: BusPort>(
        port: P,
        start_ns: i64,
        now_ns: i64,
        swept: bool,
    ) -> (Driver<P, Fake>, Seam, Fake) {
        let (pose, pose_port) = listener();
        let (event, event_port) = listener();
        let (aux_out, aux_out_port) = listener();
        let (health, health_port) = listener();
        let (status, status_port) = listener();
        drop(aux_out);
        drop(health);
        let out = Outbound::open(Destinations {
            pose: pose_port,
            event: event_port,
            aux_out: aux_out_port,
            health: health_port,
            status: status_port,
        })
        .expect("a sending socket");

        let goals = UdpSocket::bind(("127.0.0.1", 0)).expect("a free port");
        let sessions = UdpSocket::bind(("127.0.0.1", 0)).expect("a free port");
        let goal_port = goals.local_addr().expect("bound").port();
        let session_port = sessions.local_addr().expect("bound").port();
        let inbox = Inbox::from_sockets(goals, sessions);

        let mut tick = Tick::new(
            Bus::new(port, cycle_timing(DEFAULT_BAUD)),
            TickConfig {
                period_ns: PERIOD,
                hold_timeout_ns: 10 * PERIOD,
                health_poll_period_ns: 5 * PERIOD,
            },
        );
        let failed = if swept {
            tick.startup_mrc_sweep(start_ns - SWEPT_BEFORE)
        } else {
            JointFlags::NONE
        };
        let schedule = Fake::at(now_ns);
        let grid = Grid::new(start_ns, PERIOD).expect("the shipped cycle is a period");
        let driver = Driver::new(
            tick,
            inbox,
            out,
            grid,
            schedule.clone(),
            StartupMrc {
                at_ns: start_ns - SWEPT_BEFORE,
                failed,
            },
        );
        (
            driver,
            Seam {
                pose,
                event,
                status,
                goal_port,
                session_port,
            },
            schedule,
        )
    }

    /// Send `bytes` to a loopback port and wait until the reader has accounted
    /// for it.
    ///
    /// The reader thread decodes and queues; the loop drains without blocking,
    /// so a case that ran a cycle immediately would be a case about scheduling
    /// rather than about the loop. What is waited on is the seam's own counters
    /// moving rather than a fixed pause: a pause long enough to be safe on a
    /// loaded runner is a pause every case pays, and one short enough to be
    /// cheap is a flake nobody can see the cause of.
    fn command(counts: &Counts, port: u16, bytes: &[u8]) {
        let before = accounted(counts);
        let sender = UdpSocket::bind(("127.0.0.1", 0)).expect("a free port");
        sender
            .send_to(bytes, ("127.0.0.1", port))
            .expect("a datagram to loopback");
        let deadline = Instant::now() + Duration::from_secs(10);
        while accounted(counts) == before {
            assert!(
                Instant::now() < deadline,
                "the reader thread never accounted for the datagram"
            );
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    /// Every datagram the seam has taken or refused: what moves exactly once per
    /// datagram, whichever way the reader decided about it.
    fn accounted(counts: &Counts) -> u64 {
        counts.read().iter().map(|(_, value)| value).sum()
    }

    /// A setpoint a control process would send, due on the cycle at `at_ns`.
    fn goal(at_ns: i64) -> GoalSetpointWire {
        let mut message = GoalSetpointWire::new();
        let goal = message.clear_valid();
        goal.execute_at = SyncTime::from_nanos(at_ns);
        goal.mask = JointFlags::BODY_YAW;
        goal.targets.body_yaw = 0.25;
        message
    }

    /// A keep-alive: the cheapest datagram that is traffic.
    fn keep_alive() -> SessionCmdWire {
        let mut message = SessionCmdWire::new();
        message.clear_valid().kind = SessionCmdKind::KeepAlive;
        message
    }

    #[test]
    fn every_cycle_publishes_its_sample_at_the_schemas_own_size() {
        let (mut driver, seam, _clock) = driver(T0, T0);

        driver.run_cycles(3);

        let samples = published(&seam.pose);
        assert_eq!(samples.len(), 3, "one sample per cycle, without exception");
        for datagram in &samples {
            assert_eq!(
                datagram.len(),
                PoseSampleWire::SIZE,
                "a datagram on this seam is the schema's bytes and nothing else"
            );
        }
        assert_eq!(
            driver.counts(),
            LoopCounts {
                cycles: 3,
                startup_mrc: 1,
                ..LoopCounts::default()
            }
        );
    }

    #[test]
    fn a_cycle_that_ran_long_skips_the_grid_points_it_missed_and_says_how_many() {
        let (mut driver, seam, clock) = driver(T0, T0);

        // The first cycle runs on time; from then on every cycle overruns into
        // the third grid point after its own, so two points go by unattended
        // each time. The skip is reported by the cycle that ran late, not by the
        // one that overran, so three more cycles produce two reports.
        driver.run_cycles(1);
        clock.overrun_ns.set(2 * PERIOD + PERIOD / 2);
        driver.run_cycles(3);

        assert_eq!(
            driver.counts().skipped,
            4,
            "two grid points passed over per overrun, counted and not made up for"
        );
        let events: Vec<_> = published(&seam.event)
            .iter()
            .map(|bytes| as_event(bytes))
            .collect();
        let skips: Vec<_> = events
            .iter()
            .filter_map(|message| {
                let event = message.validate().expect("a published event validates");
                (event.kind == EventKind::CycleSkipped)
                    .then_some((event.count, event.silence.as_nanos()))
            })
            .collect();
        assert_eq!(
            skips,
            vec![(2, 2 * PERIOD), (2, 2 * PERIOD)],
            "one event per late cycle, naming how many points it passed and how late it is"
        );
    }

    /// Every event of `kind` the loop published on `socket`.
    fn events_of(socket: &UdpSocket, kind: EventKind) -> Vec<reachy_driver::Event> {
        published(socket)
            .iter()
            .map(|bytes| as_event(bytes))
            .filter_map(|message| {
                let event = message.validate().expect("a published event validates");
                (event.kind == kind).then_some(reachy_driver::Event {
                    kind: event.kind,
                    time_ns: event.time.as_nanos(),
                    silence_ns: event.silence.as_nanos(),
                    work_ns: event.work.as_nanos(),
                    exchange_ns: event.exchange.as_nanos(),
                    drain_ns: event.drain.as_nanos(),
                    count: event.count,
                    out_of_band: event.out_of_band,
                    rows: event.rows,
                    id: event.id,
                })
            })
            .collect()
    }

    #[test]
    fn a_skip_says_what_the_cycle_that_caused_it_actually_spent() {
        let (mut driver, seam, clock) = driver(T0, T0);

        // A cycle that takes two and a half periods: the grid points it runs
        // through are the ones the next cycle reports as skipped, and what it
        // spent is the number that says why.
        clock.per_read_ns.set(2 * PERIOD + PERIOD / 2);
        driver.run_cycles(3);

        let skips = events_of(&seam.event, EventKind::CycleSkipped);
        assert_eq!(skips.len(), 2, "one report per late cycle");
        for skip in &skips {
            assert_eq!(skip.count, 2, "two grid points passed over");
            assert_eq!(
                skip.silence_ns,
                2 * PERIOD,
                "the lateness the grid says, which is arithmetic"
            );
            assert_eq!(
                skip.work_ns,
                2 * PERIOD + PERIOD / 2,
                "and what the cycle before it measured, which is not"
            );
        }
    }

    #[test]
    fn the_loop_publishes_what_a_second_of_cycles_cost() {
        let (mut driver, seam, clock) = driver(T0, T0);
        let spend_ns = PERIOD / 4;
        clock.per_read_ns.set(spend_ns);

        // A second of a twenty-millisecond grid is fifty cycles, and the window
        // is counted in cycles of the grid it measures.
        driver.run_cycles(49);
        assert!(
            events_of(&seam.event, EventKind::CycleStats).is_empty(),
            "a window is published when it is full and not before"
        );
        driver.run_cycles(51);
        let stats = events_of(&seam.event, EventKind::CycleStats);
        assert_eq!(stats.len(), 2, "one per second of cycles");
        for window in &stats {
            assert_eq!(window.count, 50, "the cycles the window held");
            assert_eq!(window.work_ns, spend_ns, "the longest of them, as measured");
            // The health rotation alone reads on one cycle in five here, and
            // the torque-off confirmation this case's silent bus leaves open
            // takes the slot on most of the rest: what the count must never be
            // is zero, which is what a cycle report that stopped saying it ran
            // a transaction would leave on the wire forever.
            assert!(
                window.out_of_band >= window.count / 5 && window.out_of_band <= window.count,
                "the out-of-band cycles are some of the cycles, and the rotation ran on some: \
                 {window:?}"
            );
            assert!(
                window.exchange_ns > 0,
                "an exchange that ran took time, and the window carries what it took: {window:?}"
            );
            assert_eq!(
                (window.silence_ns, window.id),
                (0, 0),
                "a window names no silence and no servo, so both carry the zero that says so"
            );
        }
    }

    /// A span that came out negative is the worst thing in its window, and it
    /// stays the worst however many ordinary cycles stand beside it.
    ///
    /// A backwards clock step landing inside one cycle's span makes that cycle
    /// look instant, so nothing else about the run marks it: a window that took
    /// the longest span would drop the reading, and the analyzer's finding
    /// about the time base could never fire for the one step that matters.
    #[test]
    fn one_span_the_clock_made_negative_is_the_worst_of_its_window() {
        let mut window = CycleWindow::default();
        window.note(PERIOD / 4, Some(2_000), 100);
        window.note(-PERIOD, Some(-500), 900);
        for _ in 0..10 {
            window.note(PERIOD / 2, Some(3_000), 100);
        }
        assert_eq!(window.cycles, 12);
        assert_eq!(
            window.worst_work_ns, -PERIOD,
            "the negative span was rounded away by its neighbours"
        );
        assert_eq!(window.worst_aux_ns, -500, "and the exchange's, likewise");
        assert_eq!(
            window.worst_drain_ns, 900,
            "a write span is a wall-clock span of one call, so the longest is the worst"
        );

        // And the most negative of several holds it against the rest.
        let mut window = CycleWindow::default();
        window.note(-1_000, None, 0);
        window.note(-9_000, None, 0);
        window.note(PERIOD, None, 0);
        assert_eq!(window.worst_work_ns, -9_000);
    }

    /// A window is thrown away when the grid it measured is re-drawn.
    ///
    /// The spans in a part-built window were measured on a time base that has
    /// just moved, and a window carrying them forward would publish the step's
    /// own size as what a cycle cost -- the number a skip budget is to be sized
    /// from, laundered out of a clock event.
    #[test]
    fn a_window_measured_before_a_clock_step_is_not_published_after_it() {
        let (mut driver, seam, clock) = driver(T0, T0);

        // Expensive cycles, and not enough of them to fill a window.
        clock.per_read_ns.set(PERIOD / 2);
        driver.run_cycles(20);
        assert!(
            events_of(&seam.event, EventKind::CycleStats).is_empty(),
            "a window is published when it is full and not before"
        );

        // An hour backwards, and cheap cycles from here on.
        clock.per_read_ns.set(PERIOD / 8);
        clock.step_back_ns.set(Some(3_600 * 1_000_000_000));
        driver.run_cycles(51);

        let stats = events_of(&seam.event, EventKind::CycleStats);
        assert_eq!(stats.len(), 1, "one window, built after the step");
        assert_eq!(stats[0].count, 50, "the cycles of the re-drawn grid");
        assert_eq!(
            stats[0].work_ns,
            PERIOD / 8,
            "the expensive cycles from before the step are no part of this window"
        );
    }

    /// And neither is the last span, which is what a skip is explained by.
    #[test]
    fn a_skip_after_a_clock_step_is_explained_by_a_cycle_measured_after_it() {
        let (mut driver, seam, clock) = driver(T0, T0);

        clock.per_read_ns.set(PERIOD / 2);
        driver.run_cycles(4);

        clock.per_read_ns.set(0);
        clock.step_back_ns.set(Some(3_600 * 1_000_000_000));
        driver.run_cycles(1);
        // The first cycle of the re-drawn grid runs on time; the ones after it
        // overrun, so the skip they report is explained by a span measured on
        // this grid.
        clock.overrun_ns.set(2 * PERIOD);
        driver.run_cycles(3);

        let skips = events_of(&seam.event, EventKind::CycleSkipped);
        let after: Vec<_> = skips
            .iter()
            .filter(|skip| skip.time_ns < T0 - 3_000 * 1_000_000_000)
            .collect();
        assert!(!after.is_empty(), "the cycles after the step reported one");
        for skip in after {
            assert_ne!(
                skip.work_ns,
                PERIOD / 2,
                "a span from before the step explained a skip after it"
            );
        }
    }

    #[test]
    fn a_cycle_measured_across_a_clock_that_moved_publishes_the_span_it_measured() {
        let (mut driver, seam, clock) = driver(T0, T0);
        clock.per_read_ns.set(-PERIOD / 4);

        driver.run_cycles(50);

        let stats = events_of(&seam.event, EventKind::CycleStats);
        assert_eq!(stats.len(), 1);
        assert_eq!(
            stats[0].work_ns,
            -PERIOD / 4,
            "a span the clock made negative is published as it was measured: the negative \
             number is the reading"
        );
    }

    #[test]
    fn the_first_cycle_says_this_process_released_the_machine_before_it_started() {
        let (mut driver, seam, _clock) = driver(T0, T0);

        driver.run_cycles(1);
        assert_eq!(driver.counts().startup_mrc, 1, "on the very first cycle");
        driver.run_cycles(10);
        assert_eq!(
            driver.counts().startup_mrc,
            1,
            "a release written once is reported once"
        );

        let announced = events_of(&seam.event, EventKind::StartupMrcWrite);
        assert_eq!(announced.len(), 1, "reported exactly once");
        assert_eq!(
            announced[0].time_ns,
            T0 - SWEPT_BEFORE,
            "dated at the sweep, which ran before the grid this loop is on"
        );
        assert_eq!(
            announced[0].silence_ns, 0,
            "nothing was waited for: the release is this process's first act"
        );
    }

    #[test]
    fn a_start_whose_sweep_verified_every_row_latches_nothing_and_sweeps_no_more() {
        // The healthy start: the caller's sweep read every row back released, so
        // the belief holds nothing torqued, the gate is in its never-commanded
        // state, and no cycle carries goal-path traffic at all.
        let (mut driver, seam, _clock) = driver_over(Limp::default(), T0, T0);

        driver.run_cycles(5);

        assert!(
            !latched_in_last_sample(&seam.pose),
            "a start with nothing to re-reach does not stand in a latched gate"
        );
        assert!(
            !event_kinds(&seam.event).contains(&EventKind::TorqueOffUnconfirmed),
            "no confirmation pass was opened, so none can run out of budget"
        );
    }

    #[test]
    fn the_shipped_destinations_are_the_ports_the_seam_declares() {
        // With no header on this seam the port is the type, so a transposed pair
        // here is a health report arriving on the aux-outcome port -- a
        // wrong-size datagram, which the control process dies of rather than
        // counting.
        assert_eq!(
            Destinations::SEAM,
            Destinations {
                pose: crate::ports::POSE_PORT,
                event: crate::ports::EVENT_PORT,
                aux_out: crate::ports::AUX_OUT_PORT,
                health: crate::ports::HEALTH_PORT,
                status: crate::ports::STATUS_PORT,
            }
        );
    }

    #[test]
    fn each_of_a_cycles_reports_lands_on_its_own_port() {
        let (pose, pose_port) = listener();
        let (event, event_port) = listener();
        let (aux_out, aux_out_port) = listener();
        let (health, health_port) = listener();
        let (status, status_port) = listener();
        let mut out = Outbound::open(Destinations {
            pose: pose_port,
            event: event_port,
            aux_out: aux_out_port,
            health: health_port,
            status: status_port,
        })
        .expect("a sending socket");

        let report = CycleReport {
            sample: PoseSampleWire::new(),
            event: Some(DriverEventWire::new()),
            outcome: Some(AuxOutcomeWire::new()),
            turned_away: None,
            health: Some(HealthReportWire::new()),
            aux_span_ns: None,
            drain_ns: 0,
        };
        out.publish(&report);

        // Each subject on its own port, at its own schema's size, and exactly
        // one datagram each: a crossed pair shows up here as a datagram of the
        // wrong length on the wrong socket.
        for (socket, size, subject) in [
            (&pose, PoseSampleWire::SIZE, "sample"),
            (&event, DriverEventWire::SIZE, "event"),
            (&aux_out, AuxOutcomeWire::SIZE, "outcome"),
            (&health, HealthReportWire::SIZE, "health report"),
        ] {
            let seen = published(socket);
            assert_eq!(seen.len(), 1, "one {subject} on the {subject} port");
            assert_eq!(seen[0].len(), size, "the {subject}'s own schema size");
        }
        assert!(
            published(&status).is_empty(),
            "the status record is not a cycle's report: it goes out on its own cadence"
        );
        assert_eq!(out.counts(), (4, 0));
    }

    /// A cycle that served one request and turned another away publishes both
    /// answers, on the one port outcomes go out on: the host is waiting on an
    /// answer under each number, and a driver that sent one would leave the
    /// other to a delivery timeout.
    #[test]
    fn a_cycle_that_turned_a_request_away_publishes_that_answer_too() {
        let (pose, pose_port) = listener();
        let (event, event_port) = listener();
        let (aux_out, aux_out_port) = listener();
        let (health, health_port) = listener();
        let (_status_socket, status_port) = listener();
        let mut out = Outbound::open(Destinations {
            pose: pose_port,
            event: event_port,
            aux_out: aux_out_port,
            health: health_port,
            status: status_port,
        })
        .expect("a sending socket");

        let report = CycleReport {
            sample: PoseSampleWire::new(),
            event: None,
            outcome: Some(AuxOutcomeWire::new()),
            turned_away: Some(AuxOutcomeWire::new()),
            health: None,
            aux_span_ns: None,
            drain_ns: 0,
        };
        out.publish(&report);

        assert_eq!(published(&aux_out).len(), 2, "one answer per request");
        assert!(published(&pose).len() == 1 && published(&event).is_empty());
        assert!(published(&health).is_empty());
        assert_eq!(out.counts(), (3, 0));
    }

    #[test]
    fn a_datagram_arriving_changes_nothing_about_the_release_already_written() {
        let (mut driver, seam, _clock) = driver(T0, T0);
        command(
            &driver.inbound_counts(),
            seam.session_port,
            blob_as_bytes(&keep_alive()),
        );

        driver.run_cycles(8);

        assert_eq!(driver.counts().taken, 1, "the case rests on this");
        assert_eq!(
            driver.counts().startup_mrc,
            1,
            "the record is of what this process did on the bus, not of what it heard"
        );
        assert_eq!(
            events_of(&seam.event, EventKind::StartupMrcWrite).len(),
            1,
            "and it is still said once"
        );
    }

    #[test]
    fn a_port_that_stopped_being_read_de_torques_the_machine_once() {
        // A goal socket whose reads fail immediately and permanently: its reader
        // gives up, which is the wire failure this case is about.
        let goals = UdpSocket::bind(("127.0.0.1", 0)).expect("a free port");
        goals
            .set_read_timeout(Some(Duration::from_millis(1)))
            .expect("a read timeout");
        let sessions = UdpSocket::bind(("127.0.0.1", 0)).expect("a free port");
        let inbox = Inbox::from_sockets(goals, sessions);
        let deadline = Instant::now() + Duration::from_secs(10);
        while !inbox.reader_stopped() {
            assert!(Instant::now() < deadline, "the reader gave up");
            std::thread::sleep(Duration::from_millis(1));
        }

        let (pose, pose_port) = listener();
        let (event, event_port) = listener();
        let out = Outbound::open(Destinations {
            pose: pose_port,
            event: event_port,
            aux_out: event_port,
            health: event_port,
            status: event_port,
        })
        .expect("a sending socket");
        let tick = Tick::new(
            Bus::new(Silent, cycle_timing(DEFAULT_BAUD)),
            TickConfig {
                period_ns: PERIOD,
                hold_timeout_ns: 10 * PERIOD,
                health_poll_period_ns: 5 * PERIOD,
            },
        );
        let grid = Grid::new(T0, PERIOD).expect("the shipped cycle is a period");
        let mut driver = Driver::new(
            tick,
            inbox,
            out,
            grid,
            Fake::at(T0),
            StartupMrc {
                at_ns: T0 - SWEPT_BEFORE,
                failed: JointFlags::NONE,
            },
        );

        driver.run_cycles(3);

        assert_eq!(
            driver.counts().wire_failures,
            3,
            "re-asked on every cycle: nothing arriving on the port that still has              a reader may leave the state a wire failure forces"
        );
        let samples = published(&pose);
        assert_eq!(
            samples.len(),
            3,
            "the samples keep going out — a driver that de-torqued still reports"
        );
        for (index, datagram) in samples.iter().enumerate() {
            assert!(
                bool::from(
                    as_sample(datagram)
                        .validate()
                        .expect("a published sample validates")
                        .torque_off_latched
                ),
                "cycle {index} published a sample about a machine still being held"
            );
        }
        drop(event);
    }

    #[test]
    fn a_goal_reaches_the_gate_and_is_traffic_like_any_other_datagram() {
        let (mut driver, seam, _clock) = driver(T0, T0);
        // Dated for the second grid point, so the cycle that runs it is one this
        // case runs rather than one it has already passed.
        command(
            &driver.inbound_counts(),
            seam.goal_port,
            blob_as_bytes(&goal(T0 + PERIOD)),
        );

        driver.run_cycles(8);

        assert_eq!(driver.counts().taken, 1);
        assert_eq!(
            driver.tick_counts().goals_executed,
            1,
            "the setpoint reached the gate and was written"
        );
        assert_eq!(
            driver.counts().startup_mrc,
            1,
            "the process-start release is a fact about this process, not about its traffic"
        );
    }

    #[test]
    fn a_wait_is_never_longer_than_one_cycle() {
        // A target already past is nothing to wait for.
        assert_eq!(chunk_to(T0, T0), None);
        assert_eq!(chunk_to(T0, T0 - PERIOD), None);
        // Inside a chunk: the whole remainder, so an ordinary cycle sleeps once
        // and the chunking costs it nothing.
        assert_eq!(chunk_to(T0, T0 + PERIOD / 2), Some(PERIOD / 2));
        // Further away than a chunk — the run-up to the grid's first point, or a
        // clock that has stepped backwards — is capped, so the clock is read
        // again within one cycle either way.
        assert_eq!(chunk_to(T0, T0 + 1_000_000_000), Some(SLEEP_CHUNK_NS));
        assert_eq!(chunk_to(T0, T0 + 3_600_000_000_000), Some(SLEEP_CHUNK_NS));
    }

    #[test]
    fn a_clock_that_stepped_backwards_lets_the_machine_go() {
        let (mut driver, seam, clock) = driver(T0, T0);

        // Something is talking to this driver, so the dead-man has a silence to
        // measure and what latches is the clock step under it.
        command(
            &driver.inbound_counts(),
            seam.session_port,
            blob_as_bytes(&keep_alive()),
        );
        driver.run_cycles(2);
        assert!(
            !latched_in_last_sample(&seam.pose),
            "nothing has let this machine go yet: the case's subject would not be decisive"
        );

        // An hour backwards. Every instant the cycle stored — the last accepted
        // command above all — is now an hour in the future of every nominal
        // instant that follows, so `nominal - stored` is negative and no timer
        // measured that way can expire. A commander that dies now would leave a
        // torqued machine with a dead-man that cannot fire for an hour, which is
        // the stall this answers.
        let hour_ns = 3_600 * 1_000_000_000;
        clock.step_back_ns.set(Some(hour_ns));
        driver.run_cycles(2);

        assert_eq!(driver.counts().clock_steps, 1);
        assert!(
            latched_in_last_sample(&seam.pose),
            "a step backwards is loss of the time base every de-torquing timer runs on, \
             so the machine goes to the minimum risk condition instead"
        );

        // And traffic does not undo it: the latch is left only by a verified
        // torque-enable write, which is a fresh engagement.
        command(
            &driver.inbound_counts(),
            seam.session_port,
            blob_as_bytes(&keep_alive()),
        );
        driver.run_cycles(2);
        assert!(
            latched_in_last_sample(&seam.pose),
            "a keep-alive is not an arming"
        );
    }

    #[test]
    fn a_clock_that_stepped_backwards_re_draws_the_grid_instead_of_waiting_it_out() {
        let (mut driver, seam, clock) = driver(T0, T0);
        driver.run_cycles(2);
        assert_eq!(driver.counts().cycles, 2);

        // An hour backwards, the size of a summer-time correction or an operator
        // setting the clock. Every remaining point of the old grid is now an hour
        // away in real time; waiting one out is an hour in which nothing runs,
        // and this is the only thread that can de-torque the machine.
        let hour_ns = 3_600 * 1_000_000_000;
        clock.step_back_ns.set(Some(hour_ns));
        driver.run_cycles(4);

        assert_eq!(driver.counts().clock_steps, 1, "caught, and counted once");
        assert_eq!(
            driver.counts().cycles,
            5,
            "the wait the step interrupted ran no cycle; the loop kept cycling after it"
        );
        let samples = published(&seam.pose);
        let last = as_sample(samples.last().expect("cycles kept running"))
            .validate()
            .expect("a published sample validates")
            .nominal_time
            .as_nanos();
        assert!(
            last < T0 - hour_ns + 2 * 1_000_000_000,
            "the cycles after the step are stamped on the re-drawn grid,              within a second of where the clock now is: {last}"
        );
    }

    /// A stop that has already been asked for.
    fn asked() -> AtomicBool {
        AtomicBool::new(true)
    }

    /// A stop asked for part way through a run: false for the first `after`
    /// readings and true from then on.
    ///
    /// Models a signal arriving mid-run. The loop reads the flag twice a cycle
    /// — once before the cycle and once inside the wait that leads to it — so a
    /// stop landing after whole cycles is counted in pairs, and a case that
    /// wants the wait to be the reader that sees it asks for an odd count.
    struct AskedAfter {
        reads: Cell<u64>,
        after: u64,
    }

    impl AskedAfter {
        /// How many times the loop reads the flag on a cycle it runs.
        const READS_PER_CYCLE: u64 = 2;

        fn cycles(after: u64) -> Self {
            Self::readings(after * Self::READS_PER_CYCLE)
        }

        fn readings(after: u64) -> Self {
            Self {
                reads: Cell::new(0),
                after,
            }
        }
    }

    impl Stop for AskedAfter {
        fn asked(&self) -> bool {
            let read = self.reads.get();
            self.reads.set(read + 1);
            read >= self.after
        }
    }

    /// Every event kind published on `socket`, in order.
    fn event_kinds(socket: &UdpSocket) -> Vec<EventKind> {
        published(socket)
            .iter()
            .map(|datagram| {
                as_event(datagram)
                    .validate()
                    .expect("a published event validates")
                    .kind
            })
            .collect()
    }

    /// One published status record, flattened out of the three groups it is
    /// composed of.
    ///
    /// Every field of the record and not the subset a case happens to read: the
    /// whole value is asserted at once by
    /// [`the_record_this_driver_composes_is_this_one_whole`], which is what
    /// makes a field added to the schema and left unwritten here visible.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    struct Whole {
        time_ns: i64,
        sweep_time_ns: i64,
        sweep_failed_rows: JointFlags,
        torque_latched: bool,
        first_pose_ns: i64,
        first_session_cmd_ns: i64,
        queued: u64,
        goals: u64,
        session_cmds: u64,
        invalid: u64,
        wrong_size: u64,
        overflowed: u64,
        undelivered: u64,
        recv_errors: u64,
        readers_stopped: u64,
        goals_executed: u64,
        goals_dropped: u64,
        hold_timeouts: u64,
        read_misses: u64,
        write_failures: u64,
        events_dropped: u64,
        aux_refused: u64,
        aux_duplicates: u64,
        aux_deferred: u64,
        health_reports: u64,
        health_misses: u64,
        confirm_misses: u64,
        blind_cycles: u64,
        cycles: u64,
        skipped: u64,
        startup_mrc: u64,
        wire_failures: u64,
        taken: u64,
        clock_steps: u64,
        published: u64,
        publish_failures: u64,
        wound_down: bool,
    }

    impl Whole {
        /// The record as it came off the wire.
        fn of(record: &DriverStatusWire) -> Self {
            let status = record.validate().expect("a published status validates");
            Self {
                time_ns: status.time.as_nanos(),
                sweep_time_ns: status.sweep_time.as_nanos(),
                sweep_failed_rows: status.sweep_failed_rows,
                torque_latched: bool::from(status.torque_latched),
                first_pose_ns: status.first_pose.as_nanos(),
                first_session_cmd_ns: status.first_session_cmd.as_nanos(),
                queued: status.seam.queued,
                goals: status.seam.goals,
                session_cmds: status.seam.session_cmds,
                invalid: status.seam.invalid,
                wrong_size: status.seam.wrong_size,
                overflowed: status.seam.overflowed,
                undelivered: status.seam.undelivered,
                recv_errors: status.seam.recv_errors,
                readers_stopped: status.seam.readers_stopped,
                goals_executed: status.cycle.goals_executed,
                goals_dropped: status.cycle.goals_dropped,
                hold_timeouts: status.cycle.hold_timeouts,
                read_misses: status.cycle.read_misses,
                write_failures: status.cycle.write_failures,
                events_dropped: status.cycle.events_dropped,
                aux_refused: status.cycle.aux_refused,
                aux_duplicates: status.cycle.aux_duplicates,
                aux_deferred: status.cycle.aux_deferred,
                health_reports: status.cycle.health_reports,
                health_misses: status.cycle.health_misses,
                confirm_misses: status.cycle.confirm_misses,
                blind_cycles: status.cycle.blind_cycles,
                cycles: status.loop_counts.cycles,
                skipped: status.loop_counts.skipped,
                startup_mrc: status.loop_counts.startup_mrc,
                wire_failures: status.loop_counts.wire_failures,
                taken: status.loop_counts.taken,
                clock_steps: status.loop_counts.clock_steps,
                published: status.published,
                publish_failures: status.publish_failures,
                wound_down: bool::from(status.wound_down),
            }
        }
    }

    /// The whole record, field for field, over one known run.
    ///
    /// `DriverStatus` is composed twice -- once here and once by the simulated
    /// driver's `write_status` -- as two hand-written mappings out of different
    /// state, and the analyzer that verifies a run reads whichever the log
    /// carries. Roughly forty assignments, and nothing in either language joins
    /// them to the schema. So the value is asserted whole rather than field by
    /// field: a field written from its neighbour, or one a schema revision added
    /// and nobody filled in, changes this literal and has to be answered here.
    ///
    /// The record this driver composes is the one a unit run is verified from,
    /// which is why it is this driver's turn to have the case the simulated one
    /// already had.
    ///
    /// The run: a first cycle, then a datagram of each kind, then a window of
    /// cycles over a bus that answers nothing. The zeros are the justified set
    /// and each says why it is one -- either a fact this fixture's seam and bus
    /// cannot produce, or a state this run did not reach.
    #[test]
    fn the_record_this_driver_composes_is_this_one_whole() {
        let per_second = super::NANOS_PER_SECOND / PERIOD;
        let (mut driver, seam, _clock) = driver(T0, T0);

        driver.run_cycles(1);
        command(
            &driver.inbound_counts(),
            seam.session_port,
            blob_as_bytes(&keep_alive()),
        );
        command(
            &driver.inbound_counts(),
            seam.goal_port,
            blob_as_bytes(&goal(T0 + 4 * PERIOD)),
        );
        driver.run_cycles(u64::try_from(per_second).expect("a positive count"));

        let records = statuses(&seam.status);
        let last = Whole::of(records.last().expect("a window closed"));
        assert_eq!(
            last,
            Whole {
                time_ns: T0 + (per_second - 1) * PERIOD,
                // Half a period before the grid, which is where this fixture's
                // process wrote the minimum risk condition.
                sweep_time_ns: T0 - SWEPT_BEFORE,
                // Nothing swept it here, so nothing was left outstanding and
                // the gate stands in no latch.
                sweep_failed_rows: JointFlags::NONE,
                torque_latched: false,
                first_pose_ns: T0,
                first_session_cmd_ns: T0 + PERIOD,
                queued: 2,
                goals: 1,
                session_cmds: 1,
                // Both datagrams were the right length and read as themselves.
                invalid: 0,
                wrong_size: 0,
                // A queue two datagrams deep does not overflow, and this seam
                // does not send.
                overflowed: 0,
                undelivered: 0,
                recv_errors: 0,
                readers_stopped: 0,
                // The one setpoint, written at its instant, and no second one
                // to be dropped for want of room in the queue.
                goals_executed: 1,
                goals_dropped: 0,
                // Nothing was ever torqued, so the dead-man has nothing to
                // expire on and no read-back to miss.
                hold_timeouts: 0,
                // Nine rows a cycle over fifty cycles, none of them answered.
                read_misses: 450,
                // Writes go out; it is the replies that do not come back.
                write_failures: 0,
                events_dropped: 0,
                // The host asked for no transaction at all.
                aux_refused: 0,
                aux_duplicates: 0,
                aux_deferred: 0,
                // The rotation asks on its own period and nothing answers.
                health_reports: 0,
                // The rotation asks once every five cycles.
                health_misses: 10,
                confirm_misses: 0,
                blind_cycles: 50,
                cycles: 50,
                // The clock is the case's and never runs a grid point out.
                skipped: 0,
                startup_mrc: 1,
                wire_failures: 0,
                taken: 2,
                clock_steps: 0,
                // Fifty samples, the first cycle's copy of this record, and
                // the three events below. This copy's own send is not in it.
                published: 54,
                publish_failures: 0,
                // A wind-down is the only copy that says one happened.
                wound_down: false,
            },
        );
        assert_eq!(
            event_kinds(&seam.event),
            vec![
                EventKind::StartupMrcWrite,
                EventKind::BusFailure,
                EventKind::CycleStats
            ],
            "the three events the published count above includes",
        );
    }

    /// The first cycle publishes the record the whole run is verified from, so a
    /// logger that attached at any point after it has the startup facts.
    #[test]
    fn the_first_cycle_publishes_a_status_carrying_the_startup_record() {
        let (mut driver, seam, _clock) = driver(T0, T0);

        driver.run_cycles(1);

        let records = statuses(&seam.status);
        assert_eq!(
            records.len(),
            1,
            "one copy, published with the first sample"
        );
        let status = records[0].validate().expect("a published status validates");
        assert_eq!(
            status.sweep_time.as_nanos(),
            T0 - SWEPT_BEFORE,
            "dated at the sweep and not at the grid point that published it"
        );
        assert_eq!(status.sweep_failed_rows, JointFlags::NONE);
        assert!(!bool::from(status.wound_down));
        assert_eq!(
            status.first_pose.as_nanos(),
            T0,
            "the record goes out with the first sample, so it names it"
        );
        assert_eq!(
            status.first_session_cmd.as_nanos(),
            0,
            "nothing has been heard from the host, which is an instant that has not happened"
        );
        assert_eq!(status.loop_counts.cycles, 1);
    }

    /// A start that could not verify a row says so in every copy, whatever the
    /// logger saw of the event stream.
    #[test]
    fn a_start_that_left_a_row_unverified_says_so_in_every_status() {
        let (mut driver, seam, _clock) = driver_swept_over(Silent, T0, T0);

        driver.run_cycles(1);

        let records = statuses(&seam.status);
        let status = records[0].validate().expect("a published status validates");
        assert_eq!(
            status.sweep_failed_rows,
            reachy_driver::every_row(),
            "a silent bus verifies nothing, so every row is outstanding"
        );
        assert!(
            bool::from(status.torque_latched),
            "the gate stands in the latch the failed sweep set"
        );
    }

    /// The record rides the window the loop already keeps, and every copy is
    /// the run so far rather than the window's own slice of it.
    #[test]
    fn a_status_goes_out_on_every_window_and_carries_the_run_so_far() {
        let per_second = super::NANOS_PER_SECOND / PERIOD;
        let (mut driver, seam, _clock) = driver(T0, T0);

        driver.run_cycles(u64::try_from(2 * per_second).expect("a positive count"));

        let records = statuses(&seam.status);
        assert_eq!(
            records.len(),
            3,
            "the first cycle's copy, and one per window of cycles after it"
        );
        let cycles: Vec<u64> = records
            .iter()
            .map(|record| {
                record
                    .validate()
                    .expect("a published status validates")
                    .loop_counts
                    .cycles
            })
            .collect();
        assert_eq!(
            cycles,
            vec![
                1,
                u64::try_from(per_second).expect("a positive count"),
                u64::try_from(2 * per_second).expect("a positive count"),
            ],
            "cumulative since the process started, so each copy is a superset of the last"
        );
    }

    /// The first datagram from the host is stamped when the driver takes it,
    /// which is what an ordering against the first sample is read from.
    #[test]
    fn the_first_session_command_is_stamped_at_the_cycle_that_took_it() {
        let (mut driver, seam, _clock) = driver(T0, T0);
        driver.run_cycles(1);
        command(
            &driver.inbound_counts(),
            seam.session_port,
            blob_as_bytes(&keep_alive()),
        );

        driver.run_cycles(1);

        let records = statuses(&seam.status);
        let last = records
            .last()
            .expect("a status a cycle")
            .validate()
            .expect("a published status validates");
        assert_eq!(
            last.first_session_cmd.as_nanos(),
            0,
            "no window has closed since it arrived, so no new copy has gone out"
        );
        driver
            .run_cycles(u64::try_from(super::NANOS_PER_SECOND / PERIOD).expect("a positive count"));
        let records = statuses(&seam.status);
        let last = records
            .last()
            .expect("a window closed")
            .validate()
            .expect("a published status validates");
        assert_eq!(
            last.first_session_cmd.as_nanos(),
            T0 + PERIOD,
            "the cycle that drained it, not the one that first published a sample"
        );
        assert_eq!(last.seam.session_cmds, 1);
        assert_eq!(last.first_pose.as_nanos(), T0);
    }

    /// The last thing the loop publishes says the run finished, which is what
    /// tells a reader that the counters beside it are the whole run's.
    #[test]
    fn a_wind_down_publishes_one_final_status_saying_so() {
        let (mut driver, seam, _clock) = driver(T0, T0);

        let ran = driver.run_until(4, &asked());

        assert_eq!(ran, Ran::WoundDown(WoundDown::AlreadyReleased));
        let records = statuses(&seam.status);
        assert_eq!(
            records.len(),
            1,
            "no cycle ran, so the final copy is the only one"
        );
        assert!(
            bool::from(
                records[0]
                    .validate()
                    .expect("a published status validates")
                    .wound_down
            ),
            "the flag is what a periodic copy does not carry"
        );
    }

    /// The final copy says when it was composed, which is an instant the process
    /// actually reached.
    ///
    /// A periodic copy is dated at its cycle's grid point, and out here there is
    /// no cycle: the grid point in hand is the next one, which the wind-down has
    /// just decided nothing will run. Dating the run's last word there puts it
    /// after every message the run published, and up to a period into the future
    /// of a stop answered before the first cycle.
    #[test]
    fn the_final_status_is_dated_at_an_instant_the_run_reached() {
        // A clock a period behind the grid's first point, so the two readings
        // are different numbers.
        let (mut driver, seam, clock) = driver(T0 + PERIOD, T0);

        let ran = driver.run_until(4, &asked());

        assert_eq!(ran, Ran::WoundDown(WoundDown::AlreadyReleased));
        let records = statuses(&seam.status);
        let status = records
            .last()
            .expect("the final copy")
            .validate()
            .expect("a published status validates");
        assert!(bool::from(status.wound_down));
        assert_eq!(
            status.time.as_nanos(),
            clock.now_ns(),
            "composed when it was composed, and not at the grid point no cycle ran",
        );
    }

    #[test]
    fn a_stop_asked_for_with_nothing_torqued_costs_no_bus_work_at_all() {
        let (mut driver, seam, _clock) = driver(T0, T0);

        let ran = driver.run_until(4, &asked());

        assert_eq!(ran, Ran::WoundDown(WoundDown::AlreadyReleased));
        assert_eq!(
            driver.counts().cycles,
            0,
            "the flag is read before the cycle, so a stop asked for during a sleep does not \
             buy one more cycle of bus traffic"
        );
        assert!(
            published(&seam.pose).is_empty(),
            "a machine already at the minimum risk condition is one a stop has nothing to do about"
        );
    }

    #[test]
    fn a_stop_asked_for_part_way_through_is_answered_at_the_next_cycle_boundary() {
        let (mut driver, seam, _clock) = driver(T0, T0);

        let ran = driver.run_until(20, &AskedAfter::cycles(3));

        assert_eq!(
            ran,
            Ran::WoundDown(WoundDown::AlreadyReleased),
            "the run ends in a wind-down rather than running the cycles out"
        );
        assert_eq!(
            driver.counts().cycles,
            3,
            "the cycles before the stop ran and the ones after it did not"
        );
        assert_eq!(
            published(&seam.pose).len(),
            3,
            "the wind-down starts on a cycle boundary: no cycle is cut in half \
             and none is published twice"
        );
    }

    #[test]
    fn a_stop_asked_for_during_the_run_up_to_the_first_grid_point_does_not_wait_it_out() {
        // A first grid point seconds away rather than a period, which no
        // shipped configuration produces -- the driver anchors at the next
        // period boundary. The case constructs it so the run-up is many sleep
        // chunks long: what is asserted is that a stop is answered inside one
        // of them and not at the far end of the wait.
        const FIRST_POINT_NS: i64 = 3_000_000_000;
        let (mut driver, seam, clock) = driver(T0 + FIRST_POINT_NS, T0);

        // False on the run's own reading, true on the wait's.
        let ran = driver.run_until(20, &AskedAfter::readings(1));

        assert_eq!(
            ran,
            Ran::WoundDown(WoundDown::AlreadyReleased),
            "a stop during the run-up is answered by winding down, not by holding on"
        );
        assert_eq!(
            clock.now_ns(),
            T0,
            "the wait ended where it began: a process asked to stop does not sit out the run-up"
        );
        assert_eq!(driver.counts().cycles, 0, "no cycle ran");
        assert!(
            published(&seam.pose).is_empty(),
            "nothing was published from a grid the loop never reached"
        );
    }

    #[test]
    fn a_stop_with_a_de_torquing_outstanding_sweeps_until_the_budget_says_it_cannot_confirm() {
        // A start whose sweep could not verify a single row: the gate is
        // latched, a confirmation pass is open, and this bus answers nothing, so
        // no row is ever read back.
        let (mut driver, seam, _clock) = driver_swept_over(Silent, T0, T0);
        let before = driver.counts().cycles;

        let ran = driver.run_until(1, &asked());

        assert_eq!(
            ran,
            Ran::WoundDown(WoundDown::Unconfirmed),
            "a de-torquing nobody read back is reported as one, not waited on further"
        );
        let spent = driver.counts().cycles - before;
        let budget_cycles = TORQUE_OFF_CONFIRM_BUDGET_NS / PERIOD;
        assert!(
            (budget_cycles..=budget_cycles + 3).contains(&i64::try_from(spent).expect("cycles")),
            "the wind-down ran {spent} cycles and the confirmation budget is {budget_cycles} \
             of them plus a cycle of margin"
        );
        assert!(
            latched_in_last_sample(&seam.pose),
            "the sweep is written on every cycle of the wind-down, budget or no budget"
        );
        assert!(
            event_kinds(&seam.event).contains(&EventKind::TorqueOffUnconfirmed),
            "what a wind-down could not establish goes on the record it is establishing it on"
        );
    }

    #[test]
    fn a_stop_ends_when_every_row_has_been_read_back_released() {
        // A machine an arming torqued, over registers that answer: the
        // wind-down's sweep is written and the confirmation pass has rows to
        // credit, one per cycle.
        let (mut driver, seam, _machine, _clock) = armed_driver();
        let before = driver.counts().cycles;

        let ran = driver.run_until(1, &asked());

        assert_eq!(ran, Ran::WoundDown(WoundDown::Confirmed));
        let spent = driver.counts().cycles - before;
        assert!(
            (ROW_COUNT as u64..=ROW_COUNT as u64 + 2).contains(&spent),
            "a pass reads one row back per cycle, so nine rows cost {ROW_COUNT} cycles and \
             this spent {spent}"
        );
        assert!(
            event_kinds(&seam.event).contains(&EventKind::TorqueOffConfirmed),
            "a wind-down that read the machine back released says so"
        );
    }

    /// Arm one row: the verified torque-enable write a session runs, as the
    /// datagram it arrives in.
    ///
    /// The one transaction a host can run that moves what the driver believes
    /// about torque, which is what a wind-down over a live gesture reads.
    fn arming(corr: u32, id: u8) -> SessionCmdWire {
        let mut message = SessionCmdWire::new();
        let cmd = message.clear_valid();
        cmd.kind = SessionCmdKind::Aux;
        cmd.corr = corr;
        cmd.txn.active = true.into();
        cmd.txn.op = AuxOpKind::WriteRegVerified;
        cmd.txn.id = id;
        cmd.txn.reg = RegId::TorqueEnable;
        cmd.txn.value_kind = value::u8(1).shape();
        cmd.txn.value = value::u8(1).bits();
        message
    }

    /// A driver over a control table with one row believed torqued, and the
    /// table it is holding.
    ///
    /// The state a stop mid-gesture arrives in: a verified torque-enable write
    /// was run and read back, so the belief is not empty. The process-start
    /// release is behind it and latched nothing — this loop's driver was handed
    /// a machine whose rows all read back released — so what is torqued here is
    /// what the arming torqued.
    fn armed_driver() -> (Driver<Limp, Fake>, Seam, Limp, Fake) {
        let machine = Limp::default();
        let (mut driver, seam, clock) = driver_over(machine.clone(), T0, T0);
        command(
            &driver.inbound_counts(),
            seam.session_port,
            blob_as_bytes(&arming(1, reachy_motion::arm::SERVO_IDS[0])),
        );
        driver.run_cycles(3);
        assert_eq!(
            machine.writes_of(TORQUE_ENABLE, 1),
            1,
            "the case rests on the machine having been armed"
        );
        assert_eq!(
            driver.counts().startup_mrc,
            1,
            "the process-start release ran and is reported; what this case is about is the \
             arming after it"
        );
        assert!(
            !latched_in_last_sample(&seam.pose),
            "a machine believed to be holding is what this case is about"
        );
        (driver, seam, machine, clock)
    }

    #[test]
    fn a_stop_while_the_machine_is_believed_torqued_sweeps_it_and_reads_it_back() {
        let (mut driver, seam, machine, _clock) = armed_driver();
        let swept_before = machine.writes_of(TORQUE_ENABLE, 0);
        assert_eq!(
            swept_before, 0,
            "nothing has released this machine yet: the sweep below is the wind-down's own"
        );

        let ran = driver.run_until(1, &asked());

        assert_eq!(
            ran,
            Ran::WoundDown(WoundDown::Confirmed),
            "a stop over a machine this driver believes is holding has work to do, and this \
             machine answers the read-back"
        );
        assert!(
            machine.writes_of(TORQUE_ENABLE, 0) >= ROW_COUNT,
            "every row was written released, in the wind-down's own cycles: {} writes",
            machine.writes_of(TORQUE_ENABLE, 0)
        );
        assert!(
            latched_in_last_sample(&seam.pose),
            "and the samples say the machine has been let go"
        );
        assert!(
            event_kinds(&seam.event).contains(&EventKind::TorqueOffConfirmed),
            "a wind-down that read every row back released says so on the record"
        );
    }

    #[test]
    fn a_clock_that_steps_back_inside_a_wind_down_keeps_writing_the_release() {
        let (mut driver, seam, machine, clock) = armed_driver();
        // The step lands on the wind-down's first wait, so the cycle that would
        // have written the sweep does not run and the grid is re-drawn instead.
        clock.step_back_ns.set(Some(3_600 * 1_000_000_000));

        let ran = driver.run_until(1, &asked());

        assert_eq!(
            driver.counts().clock_steps,
            1,
            "the step is what this case is about"
        );
        assert_eq!(
            ran,
            Ran::WoundDown(WoundDown::Confirmed),
            "a re-drawn grid costs the wind-down an iteration and not its verdict"
        );
        assert!(
            machine.writes_of(TORQUE_ENABLE, 0) >= ROW_COUNT,
            "the release was written on the cycles after the step: {} writes",
            machine.writes_of(TORQUE_ENABLE, 0)
        );
        assert!(
            event_kinds(&seam.event).contains(&EventKind::TorqueOffConfirmed),
            "and it was read back"
        );
    }

    #[test]
    fn a_wind_down_whose_clock_keeps_stepping_back_ends_on_its_bound_unconfirmed() {
        let (mut driver, _seam, machine, clock) = armed_driver();
        // Every wait from here is a backwards step, so no cycle of the wind-down
        // ever runs and the confirmation pass is never fed.
        clock.back_every_ns.set(Some(1_000_000_000));
        let cycles_before = driver.counts().cycles;

        let ran = driver.run_until(1, &asked());

        assert_eq!(
            ran,
            Ran::WoundDown(WoundDown::Unconfirmed),
            "a wind-down that ran out of iterations says it could not confirm rather than \
             looping on a clock it cannot trust"
        );
        assert_eq!(
            driver.counts().cycles,
            cycles_before,
            "no cycle ran: every iteration of the bound was a re-drawn grid"
        );
        let bound = TORQUE_OFF_CONFIRM_BUDGET_NS / PERIOD + 2;
        assert_eq!(
            i64::try_from(driver.counts().clock_steps).expect("steps"),
            bound,
            "one step per iteration of the bound, and then the loop gave up"
        );
        // And what a wind-down that ran no cycle did *not* do: a re-anchoring
        // asks the cycle for the release rather than writing it, so a grid that
        // was re-drawn on every iteration leaves nothing on the wire. The
        // machine this driver can no longer schedule against is left to the
        // servos' own bus watchdog, which stops it without releasing it — which
        // is what the verdict's line names.
        assert_eq!(
            machine.writes_of(TORQUE_ENABLE, 0),
            0,
            "no cycle ran, so nothing reached the bus"
        );
    }

    #[test]
    fn the_report_names_every_number_the_process_has() {
        let (mut driver, _seam, _clock) = driver(T0, T0);
        driver.run_cycles(1);

        let line = driver.report();
        assert!(
            line.starts_with(&format!("cycle=1 nominal={} ", T0 + PERIOD)),
            "a line has to be placeable in time: `{line}`"
        );
        for label in [
            "cycles",
            "skipped",
            "startup_mrc",
            "wire_failures",
            "taken",
            "goals_executed",
            "blind_cycles",
            "health_misses",
            "goals",
            "wrong_size",
            "readers_stopped",
            "published",
            "publish_failures",
        ] {
            assert!(
                line.contains(&format!("{label}=")),
                "{label} is missing from `{line}`"
            );
        }
        assert!(
            !line.contains("last_refusal"),
            "nothing has been refused, so there is nothing to name"
        );
    }

    /// The two numbers the offline report reads back out of this line, in the
    /// shape it reads them in: one line, `label=value` pairs separated by
    /// spaces. The report imports the labels rather than spelling them, so what
    /// this pins is the shape around them.
    #[test]
    fn the_summary_line_carries_the_two_counts_the_report_cross_checks_against() {
        let (mut driver, _seam, _clock) = driver(T0, T0);
        driver.run_cycles(1);

        let line = driver.report();
        assert_eq!(line.lines().count(), 1, "the summary is one line: `{line}`");
        for label in [Counts::SESSION_CMDS, TickCounts::AUX_REFUSED] {
            let value = line
                .split_whitespace()
                .filter_map(|pair| pair.split_once('='))
                .find(|(key, _)| *key == label)
                .map(|(_, value)| value.parse::<u64>());
            assert_eq!(
                value,
                Some(Ok(0)),
                "{label}=<number> is missing from `{line}`"
            );
        }
        assert!(
            !line.contains(WoundDown::STOPPING),
            "a cadence summary is not the end of a run: `{line}`"
        );
    }

    /// Every wind-down line carries the prefix the offline report finds the end
    /// of a run by. A reworded line that dropped it would make every hard-killed
    /// run's counters read as the run's last word, which is what the prefix is
    /// checked for.
    #[test]
    fn every_wind_down_line_says_it_is_stopping() {
        for how in [
            WoundDown::AlreadyReleased,
            WoundDown::Confirmed,
            WoundDown::Unconfirmed,
        ] {
            assert!(
                how.line().starts_with(WoundDown::STOPPING),
                "`{}` does not start with `{}`",
                how.line(),
                WoundDown::STOPPING
            );
        }
    }
}
