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
//! - **A driver nobody is talking to.** Nothing on either inbound port within
//!   the startup window and the machine is de-torqued once, reported as
//!   `startup_mrc_write`. A driver that came up next to a torqued machine
//!   nobody is commanding is the one case where the dead-man has no silence to
//!   measure — it has never heard anything to be silent after.
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

use brenn_reachy__driver__health_clk_rs::{DriverEventWire, EventKind};
use clockwork_rs::{Blob, blob_as_bytes};
use reachy_bus::BusPort;
use reachy_driver::{ConfirmReport, Event, TORQUE_OFF_CONFIRM_BUDGET_NS};

use crate::grid::{Cycle, Grid};
use crate::inbound::{Counts, Inbound, Inbox};
use crate::ports::{AUX_OUT_PORT, EVENT_PORT, HEALTH_PORT, LOOPBACK, POSE_PORT};
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
/// longer wait is legitimate — the run-up to the grid's first point, which is up
/// to a second — is where a clock step would otherwise go unnoticed for the
/// longest.
const SLEEP_CHUNK_NS: i64 = reachy_driver::NOMINAL_CYCLE_NS;

/// Where the loop meets real time.
///
/// A trait rather than two calls into the clock, so that every rule this module
/// has about time — when a startup window closes, what a late cycle costs — is
/// stated over instants a case can name. The driver runs on [`RealTime`]; a
/// case runs on a schedule it advances itself, and neither one has a different
/// loop body.
pub trait Schedule {
    /// Now, on the clock the grid is drawn on.
    fn now_ns(&self) -> i64;

    /// Wait until `target_ns`. Returns at once if it has already passed.
    ///
    /// Answers how the wait ended, because the clock this grid is drawn on can
    /// be stepped and a wait that ended in a step is not a wait that arrived.
    fn sleep_until(&mut self, target_ns: i64) -> Waited;
}

/// How a wait ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Waited {
    /// The target instant arrived.
    Arrived,
    /// The clock went backwards while waiting, so the target is now further
    /// away in real time than the grid ever put it. The grid has to be re-drawn
    /// from where the clock says it is; waiting for the old target would leave
    /// the loop asleep for the size of the step.
    ClockSteppedBack,
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
    fn sleep_until(&mut self, target_ns: i64) -> Waited {
        loop {
            let now = self.now_ns();
            let Some(wait) = chunk_to(now, target_ns) else {
                return Waited::Arrived;
            };
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
}

impl Destinations {
    /// The seam as it ships: the four ports the control process binds.
    pub const SEAM: Self = Self {
        pose: POSE_PORT,
        event: EVENT_PORT,
        aux_out: AUX_OUT_PORT,
        health: HEALTH_PORT,
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
        if let Some(health) = report.health.as_ref() {
            self.send(self.dest.health, health);
        }
    }

    /// Publish one event the loop raised rather than the cycle.
    pub fn publish_event(&mut self, event: &DriverEventWire) {
        self.send(self.dest.event, event);
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
    /// Times the startup window closed with nothing having arrived, which is
    /// once at most.
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

/// The driver's loop: a cycle, a grid, a seam and a clock.
pub struct Driver<P: BusPort, S: Schedule> {
    tick: Tick<P>,
    inbox: Inbox,
    out: Outbound,
    grid: Grid,
    schedule: S,
    /// The cycle the next call will run.
    next: Cycle,
    /// When the startup window closes: the instant after which a driver that
    /// has heard nothing lets the machine go.
    startup_closes_ns: i64,
    /// Whether anything has ever arrived on either inbound port.
    heard: bool,
    /// Whether the startup release has already been written. Once only: a
    /// window that closed does not keep closing.
    startup_written: bool,
    counts: LoopCounts,
}

impl<P: BusPort, S: Schedule> Driver<P, S> {
    /// A loop running `tick` on `grid`, reading `inbox` and publishing on `out`.
    ///
    /// The startup window is measured from the grid's own start rather than from
    /// whenever this was called: the grid begins at a top of a second, which is
    /// an instant a log can be read against, and a window measured from process
    /// setup would be a window nobody can name afterwards.
    pub fn new(
        tick: Tick<P>,
        inbox: Inbox,
        out: Outbound,
        grid: Grid,
        schedule: S,
        startup_window_ns: i64,
    ) -> Self {
        let next = grid.first_from(schedule.now_ns());
        Self {
            tick,
            inbox,
            out,
            grid,
            schedule,
            next,
            startup_closes_ns: grid.instant(0) + startup_window_ns,
            heard: false,
            startup_written: false,
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
    /// nobody will read.
    pub fn run_until(&mut self, cycles: u64, stop: &impl Stop) -> Ran {
        for _ in 0..cycles {
            if stop.asked() {
                return Ran::WoundDown(self.wind_down());
            }
            self.cycle_once();
        }
        Ran::Cycles
    }

    /// Sleep to the next grid point and run the cycle there, or re-draw the grid
    /// if the clock stepped backwards instead.
    ///
    /// Both `run_until` and `wind_down` run this; the wind-down path is the one
    /// that writes the torque-off sweep.
    fn cycle_once(&mut self) -> Stepped {
        let cycle = self.next;
        if self.schedule.sleep_until(cycle.nominal_ns) == Waited::ClockSteppedBack {
            self.reanchor();
            return Stepped::Reanchored;
        }
        self.step(cycle);
        self.next = self.grid.next_after(&cycle, self.schedule.now_ns());
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
        if !self.tick.torque_outstanding() {
            return WoundDown::AlreadyReleased;
        }
        let cycle = self.next;
        self.tick.request_torque_off(cycle.nominal_ns);
        let period_ns = self.grid.period_ns().max(1);
        let bound = TORQUE_OFF_CONFIRM_BUDGET_NS / period_ns + 2;
        for _ in 0..bound {
            if self.cycle_once() == Stepped::Reanchored {
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
    /// the machine. A new grid at the next top of a second is the same
    /// construction the process started with, so a log can be read against it
    /// the same way, and the step is counted because a run whose grid was
    /// re-drawn has instants in it that do not line up with the ones before.
    ///
    /// The startup window travels with the grid: a driver still waiting to hear
    /// from anybody keeps waiting the same span it was given, measured from the
    /// grid it is now on.
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
        let Ok(grid) = Grid::new(Grid::top_of_second_at(now), period_ns) else {
            // The period came from a grid, so it is positive. Nothing to do
            // rather than a panic: this thread's job is to keep de-torquing
            // available.
            return;
        };
        let window_ns = self.startup_closes_ns - self.grid.instant(0);
        self.grid = grid;
        self.next = grid.first_from(now);
        self.startup_closes_ns = grid.instant(0) + window_ns;
    }

    /// One cycle, in the order its parts have to happen in.
    ///
    /// The skip report first, because it is about the cycles between this one
    /// and the last; then the seam, so the cycle acts on everything that has
    /// arrived; then the two conditions the cycle cannot see for itself; then
    /// the cycle, and its reports.
    fn step(&mut self, cycle: Cycle) {
        self.counts.cycles += 1;
        if cycle.skipped > 0 {
            self.counts.skipped += u64::from(cycle.skipped);
            let late = i64::from(cycle.skipped) * self.grid.period_ns();
            self.out.publish_event(&raised(&Event {
                kind: EventKind::CycleSkipped,
                silence_ns: late,
                count: cycle.skipped,
                ..Event::at(cycle.nominal_ns)
            }));
        }
        self.take_inbound(cycle.nominal_ns);
        self.answer_wire_failure(cycle.nominal_ns);
        self.answer_startup_window(cycle.nominal_ns);
        let report = self.tick.run(cycle.nominal_ns);
        self.out.publish(&report);
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
                    tick.offer_session_cmd(cmd, nominal_ns);
                }
            }
        });
        counts.taken += taken as u64;
        if taken > 0 {
            self.heard = true;
        }
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

    /// De-torque the machine if the startup window closed in silence.
    ///
    /// A restarted driver's default is the minimum risk condition: it does not
    /// know what state it found the machine in, and the machine cannot tell it.
    /// Once, and never again — after this the dead-man has the silence to
    /// measure that it did not have before.
    fn answer_startup_window(&mut self, nominal_ns: i64) {
        if self.startup_written || self.heard || nominal_ns < self.startup_closes_ns {
            return;
        }
        self.startup_written = true;
        self.counts.startup_mrc += 1;
        self.tick.request_torque_off(nominal_ns);
        self.out.publish_event(&raised(&Event {
            kind: EventKind::StartupMrcWrite,
            silence_ns: nominal_ns - self.grid.instant(0),
            ..Event::at(nominal_ns)
        }));
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
        AtomicBool, Destinations, Driver, LoopCounts, Outbound, Ran, SLEEP_CHUNK_NS, Schedule,
        Stop, TORQUE_OFF_CONFIRM_BUDGET_NS, Waited, WoundDown, chunk_to,
    };
    use crate::grid::Grid;
    use crate::inbound::{Counts, Inbox};
    use crate::tick::{CycleReport, Tick, TickConfig, cycle_timing};
    use brenn_reachy__cogs__session_cmd_clk_rs::{SessionCmdKind, SessionCmdWire};
    use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
    use brenn_reachy__driver__health_clk_rs::{
        AuxOutcomeWire, DriverEventWire, EventKind, HealthReportWire,
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

    /// The startup window in these cases: five cycles, so a case can run inside
    /// it and past it without waiting on the shipped two seconds.
    const WINDOW: i64 = 5 * PERIOD;

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
    }

    impl Fake {
        fn at(now: i64) -> Self {
            Self {
                now: Rc::new(Cell::new(now)),
                overrun_ns: Rc::new(Cell::new(0)),
                step_back_ns: Rc::new(Cell::new(None)),
                back_every_ns: Rc::new(Cell::new(None)),
            }
        }
    }

    impl Schedule for Fake {
        fn now_ns(&self) -> i64 {
            self.now.get()
        }

        fn sleep_until(&mut self, target_ns: i64) -> Waited {
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
            Waited::Arrived
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
        goal_port: u16,
        session_port: u16,
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
    fn driver_over<P: BusPort>(
        port: P,
        start_ns: i64,
        now_ns: i64,
    ) -> (Driver<P, Fake>, Seam, Fake) {
        let (pose, pose_port) = listener();
        let (event, event_port) = listener();
        let (aux_out, aux_out_port) = listener();
        let (health, health_port) = listener();
        drop(aux_out);
        drop(health);
        let out = Outbound::open(Destinations {
            pose: pose_port,
            event: event_port,
            aux_out: aux_out_port,
            health: health_port,
        })
        .expect("a sending socket");

        let goals = UdpSocket::bind(("127.0.0.1", 0)).expect("a free port");
        let sessions = UdpSocket::bind(("127.0.0.1", 0)).expect("a free port");
        let goal_port = goals.local_addr().expect("bound").port();
        let session_port = sessions.local_addr().expect("bound").port();
        let inbox = Inbox::from_sockets(goals, sessions);

        let tick = Tick::new(
            Bus::new(port, cycle_timing(DEFAULT_BAUD)),
            TickConfig {
                period_ns: PERIOD,
                hold_timeout_ns: 10 * PERIOD,
                health_poll_period_ns: 5 * PERIOD,
            },
        );
        let schedule = Fake::at(now_ns);
        let grid = Grid::new(start_ns, PERIOD).expect("the shipped cycle is a period");
        let driver = Driver::new(tick, inbox, out, grid, schedule.clone(), WINDOW);
        (
            driver,
            Seam {
                pose,
                event,
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

    #[test]
    fn a_driver_nobody_talks_to_lets_the_machine_go_once_when_the_window_closes() {
        let (mut driver, seam, _clock) = driver(T0, T0);

        // Inside the window: nothing has been heard, and nothing is written.
        driver.run_cycles(5);
        assert_eq!(driver.counts().startup_mrc, 0, "the window is still open");

        // The sixth cycle is the first at or past the window's close.
        driver.run_cycles(1);
        assert_eq!(driver.counts().startup_mrc, 1);
        driver.run_cycles(10);
        assert_eq!(
            driver.counts().startup_mrc,
            1,
            "a window that closed does not keep closing"
        );

        let announced: Vec<_> = published(&seam.event)
            .iter()
            .map(|bytes| as_event(bytes))
            .filter(|message| {
                message
                    .validate()
                    .expect("a published event validates")
                    .kind
                    == EventKind::StartupMrcWrite
            })
            .collect();
        assert_eq!(announced.len(), 1, "reported exactly once");
        assert_eq!(
            announced[0]
                .validate()
                .expect("a published event validates")
                .silence
                .as_nanos(),
            WINDOW,
            "how long it waited before letting go"
        );
        // The bookkeeping is not the point: the machine is the point. The sample
        // the same cycle published says the release is latched, and later cycles
        // keep saying it.
        assert!(
            latched_in_last_sample(&seam.pose),
            "the window closing let the machine go, and it stays let go"
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
            }
        );
    }

    #[test]
    fn each_of_a_cycles_four_reports_lands_on_its_own_port() {
        let (pose, pose_port) = listener();
        let (event, event_port) = listener();
        let (aux_out, aux_out_port) = listener();
        let (health, health_port) = listener();
        let mut out = Outbound::open(Destinations {
            pose: pose_port,
            event: event_port,
            aux_out: aux_out_port,
            health: health_port,
        })
        .expect("a sending socket");

        let report = CycleReport {
            sample: PoseSampleWire::new(),
            event: Some(DriverEventWire::new()),
            outcome: Some(AuxOutcomeWire::new()),
            health: Some(HealthReportWire::new()),
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
        assert_eq!(out.counts(), (4, 0));
    }

    #[test]
    fn a_driver_that_heard_one_datagram_leaves_the_startup_window_alone() {
        let (mut driver, seam, _clock) = driver(T0, T0);
        command(
            &driver.inbound_counts(),
            seam.session_port,
            blob_as_bytes(&keep_alive()),
        );

        // Well past the window's close: the dead-man owns the silence from here,
        // and it measures from the datagram that arrived rather than from a
        // driver that had never heard anything.
        driver.run_cycles(8);

        assert_eq!(driver.counts().taken, 1, "the case rests on this");
        assert_eq!(
            driver.counts().startup_mrc,
            0,
            "something is talking to this driver, so its startup default is not the one for silence"
        );
        assert!(
            !published(&seam.event)
                .iter()
                .map(|bytes| as_event(bytes))
                .any(
                    |message| message.validate().expect("valid").kind == EventKind::StartupMrcWrite
                )
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
        let mut driver = Driver::new(tick, inbox, out, grid, Fake::at(T0), WINDOW);

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
            0,
            "a goal is a host talking to this driver, whichever port it came in on"
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

        // Something is talking to this driver, so the startup window is not what
        // latches here and the dead-man has a silence to measure.
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
    /// Models a signal arriving mid-run — the loop reads the flag once per
    /// cycle, so `after` is the cycle count at which the stop lands.
    struct AskedAfter {
        reads: Cell<u64>,
        after: u64,
    }

    impl AskedAfter {
        fn cycles(after: u64) -> Self {
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
    fn a_stop_with_a_de_torquing_outstanding_sweeps_until_the_budget_says_it_cannot_confirm() {
        let (mut driver, seam, _clock) = driver(T0, T0);

        // Past the startup window with nothing having arrived: the loop has
        // written the release and opened a confirmation pass, and this bus
        // answers nothing, so no row is ever read back.
        driver.run_cycles(6);
        assert_eq!(driver.counts().startup_mrc, 1);
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
        let (mut driver, seam, _clock) = driver_over(Limp::default(), T0, T0);

        // The same startup release, over a machine whose registers answer: the
        // confirmation pass has rows to credit, one per cycle.
        driver.run_cycles(6);
        assert_eq!(driver.counts().startup_mrc, 1);
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
    /// was run and read back, so the belief is not empty. Nothing about the
    /// startup window is in play — a datagram arrived, so the window is not what
    /// de-torques anything here, and no startup sweep has been written.
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
            0,
            "something talked to this driver, so nothing here is the startup default"
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
}
