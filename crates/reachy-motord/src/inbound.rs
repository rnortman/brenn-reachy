//! The reading half of the seam: two bound sockets and the queue behind them.
//!
//! A datagram here is a schema's blob bytes and nothing else — no header, no
//! length, no sequence number — so the port it arrived on is what says which
//! schema it is. Two ports, two schemas: goals on one, session commands on the
//! other.
//!
//! Two things are refused and counted, never raised: a datagram whose length is
//! not the schema's size, and one whose bytes do not validate. Both mean a
//! sender this process cannot do anything about, and a driver that died of one
//! would be a driver a stray packet can stop from de-torquing a machine. The
//! counts are what a run is read against afterwards.
//!
//! Where the work happens matters as much as what it is. One thread per socket
//! blocks in `recv_from`, decodes, and pushes onto that port's queue; the loop
//! thread drains both without blocking at the top of each cycle. Nothing that
//! decides anything runs on a reader thread, and the loop thread never waits on
//! a socket — a cycle's timing is the grid's, not the network's.
//!
//! **A queue per port, not one shared.** Both are bounded and a drain takes at
//! most their depth, which is what makes that last sentence true of a machine
//! and not just of a design: a sender in a loop can neither grow this process's
//! memory nor lengthen a cycle, it can only have its own datagrams refused and
//! counted. Separate queues are what makes the refusal *its own*: goals arrive
//! every cycle and session commands rarely, so one shared queue full of goals
//! is a queue that drops the de-torquing that arrives next — a thing standing
//! between a torque-off and the loop, which this seam does not get to have.
//! Session commands are drained first for the same reason.
//!
//! The reader threads live as long as the process, with two ends. The queue's
//! receiving end going away is one, noticed on the next datagram to arrive — a
//! caller that drops the [`Inbox`] and expects the threads to have stopped
//! already is expecting something this does not promise. A run of errors from
//! the socket itself is the other: a socket that fails immediately every time
//! would make a reader a spin loop against the thread that owns the bus, so the
//! reader stops and is counted as stopped.
//!
//! A stopped reader is a wire failure and not a statistic, and the two ports are
//! not symmetric in what it costs. The goal port going deaf is goal silence, and
//! silence is what the dead-man answers. The session port going deaf is a driver
//! that keeps commanding motion while `TorqueOffNow` can no longer reach it —
//! answered by nothing at all unless something reads it. So
//! [`Inbox::reader_stopped`] is the loop's condition to act on, not
//! [`Counts::readers_stopped`] to log.

use brenn_reachy__cogs__session_cmd_clk_rs::{SessionCmd, SessionCmdWire};
use brenn_reachy__driver__goal_clk_rs::{GoalSetpoint, GoalSetpointWire};
use clockwork_rs::{Blob, Invalid, ValidView, blob_from_bytes, validate};
use std::fmt;
use std::io;
use std::net::UdpSocket;
use std::sync::Mutex;
use std::sync::atomic::{AtomicU64, Ordering};
use std::sync::mpsc::{Receiver, SyncSender, TryRecvError, TrySendError, sync_channel};
use std::sync::{Arc, MutexGuard};
use std::thread;

use crate::ports::{GOAL_PORT, LOOPBACK, SESSION_PORT};

/// The read buffer, comfortably larger than either schema.
///
/// Sized so that an oversized datagram is *seen* to be oversized: a buffer that
/// happened to be exactly the schema's size would silently truncate a longer
/// datagram into one that decodes, which is the one failure mode this seam
/// cannot have.
const BUFFER_BYTES: usize = 4096;

const _: () = assert!(BUFFER_BYTES > GoalSetpointWire::SIZE);
const _: () = assert!(BUFFER_BYTES > SessionCmdWire::SIZE);

/// How many decoded datagrams may wait for the loop thread, per port.
///
/// Ordinary traffic is one goal and one session command per cycle, so this is
/// dozens of cycles of backlog — deep enough that a long bus transaction never
/// costs a datagram, shallow enough that a sender in a loop grows nothing. Past
/// it a datagram is refused and counted, which is the same answer this seam
/// gives every other datagram it cannot take. Per port, so a flood on one is
/// refused on that one: the depth a session command finds available never
/// depends on how many goals arrived.
const QUEUE_DEPTH: usize = 64;

/// How many errors in a row from one socket end its reader thread.
///
/// A transient error is answered by looping round and reading again. A socket in
/// a permanent error state returns one immediately every time, and a reader
/// looping on that is a spin loop taking a core from the thread that owns the
/// bus — on this board, the more expensive failure by far. So a run this long is
/// taken as permanent: the reader stops, the stop is counted, and the seam goes
/// silent rather than busy.
const MAX_CONSECUTIVE_RECV_ERRORS: u32 = 16;

/// One decoded datagram, as the loop thread receives it.
#[derive(Clone, Debug)]
pub enum Inbound {
    /// A goal setpoint, from the control process's motion tick.
    Goal(GoalSetpointWire),
    /// A session command: a keep-alive, a de-torquing, or a transaction to run.
    Session(SessionCmdWire),
}

/// Which schema a port carries, for the record a refusal leaves.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Subject {
    /// The goal port.
    Goal,
    /// The session-command port.
    Session,
}

impl fmt::Display for Subject {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Goal => f.write_str("goal"),
            Self::Session => f.write_str("session command"),
        }
    }
}

/// Why a datagram was not taken.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal {
    /// Its length was not the schema's size. Nothing is read out of it: the
    /// bytes of a message of another shape are not this message's fields.
    WrongSize {
        /// How many bytes arrived.
        got: usize,
        /// How many the schema is.
        want: usize,
    },
    /// The right number of bytes, but not a valid message of the schema — an
    /// undeclared enumerator, a count past a capacity. The first invariant it
    /// failed, with its offset.
    Invalid(Invalid),
}

impl fmt::Display for Refusal {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WrongSize { got, want } => write!(f, "{got} bytes where the schema is {want}"),
            Self::Invalid(invalid) => write!(
                f,
                "{:?} at byte {} of a datagram of the right size",
                invalid.kind, invalid.offset
            ),
        }
    }
}

/// The last datagram refused, kept as the detail a count cannot carry.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Rejected {
    /// Which port it arrived on, which on this seam is which schema it claimed
    /// to be.
    pub subject: Subject,
    /// What was wrong with it.
    pub refusal: Refusal,
}

impl fmt::Display for Rejected {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}: {}", self.subject, self.refusal)
    }
}

/// What the reading half of the seam counted.
///
/// Process-local numbers, read by the loop thread and reported in the driver's
/// own log lines: nothing here is a Clockwork signal, because this process is
/// not a cog.
#[derive(Debug, Default)]
pub struct Counts {
    /// Datagrams a reader thread decoded and put on a queue. Counted where the
    /// reader accepts one rather than where the loop takes it, so the difference
    /// against `goals` + `session_cmds` is what is waiting on the seam right
    /// now.
    pub queued: AtomicU64,
    /// Goals taken.
    pub goals: AtomicU64,
    /// Session commands taken.
    pub session_cmds: AtomicU64,
    /// Datagrams refused for their length.
    pub wrong_size: AtomicU64,
    /// Datagrams refused by validation.
    pub invalid: AtomicU64,
    /// Datagrams decoded but dropped because the queue was full: the loop
    /// thread is behind by more than [`QUEUE_DEPTH`], or a sender is pushing
    /// faster than the seam carries.
    pub overflowed: AtomicU64,
    /// Datagrams decoded but dropped because the loop thread is gone. Counted
    /// for completeness; a run that has any of these has already ended.
    pub undelivered: AtomicU64,
    /// Errors from the socket itself.
    pub recv_errors: AtomicU64,
    /// Reader threads that gave up after a run of socket errors. Any of these
    /// is a port nothing is reading any more, which is a wire failure the loop
    /// asks about through [`Inbox::reader_stopped`] rather than reading here
    /// after the fact.
    pub readers_stopped: AtomicU64,
    /// The last datagram refused, with which port refused it. A count says how
    /// many; this says what, which for a seam whose whole type check is a length
    /// is the detail worth keeping.
    last_refusal: Mutex<Option<Rejected>>,
}

impl Counts {
    /// Note a refusal, keeping the detail of the most recent one.
    fn refused(&self, subject: Subject, refusal: Refusal) {
        let counter = match refusal {
            Refusal::WrongSize { .. } => &self.wrong_size,
            Refusal::Invalid(_) => &self.invalid,
        };
        counter.fetch_add(1, Ordering::Relaxed);
        *self.held() = Some(Rejected { subject, refusal });
    }

    /// The last datagram refused, or nothing when none has been.
    #[must_use]
    pub fn last_refusal(&self) -> Option<Rejected> {
        *self.held()
    }

    /// The held cell, taken even if a holder panicked: a poisoned record of the
    /// last refusal is still the last refusal, and this seam does not fail a
    /// driver over its own bookkeeping.
    fn held(&self) -> MutexGuard<'_, Option<Rejected>> {
        self.last_refusal
            .lock()
            .unwrap_or_else(std::sync::PoisonError::into_inner)
    }

    /// The label the datagram count is printed under.
    ///
    /// A `pub const` rather than a literal at the one place it is written,
    /// because the driver's summary line is read back by the offline report as
    /// the independent witness to what the log kept: a rename here would
    /// otherwise be a refactor in this crate that silently stops that
    /// cross-check working.
    pub const SESSION_CMDS: &str = "session_cmds";

    /// Every count, as plain numbers taken one at a time. Not an instant's
    /// snapshot — the readers keep counting while this reads — which is all a
    /// log line at the end of a cycle needs. What was wrong with the refused
    /// ones is [`Counts::last_refusal`].
    #[must_use]
    pub fn read(&self) -> [(&'static str, u64); 9] {
        [
            ("queued", self.queued.load(Ordering::Relaxed)),
            ("goals", self.goals.load(Ordering::Relaxed)),
            (
                Self::SESSION_CMDS,
                self.session_cmds.load(Ordering::Relaxed),
            ),
            ("wrong_size", self.wrong_size.load(Ordering::Relaxed)),
            ("invalid", self.invalid.load(Ordering::Relaxed)),
            ("overflowed", self.overflowed.load(Ordering::Relaxed)),
            ("undelivered", self.undelivered.load(Ordering::Relaxed)),
            ("recv_errors", self.recv_errors.load(Ordering::Relaxed)),
            (
                "readers_stopped",
                self.readers_stopped.load(Ordering::Relaxed),
            ),
        ]
    }
}

/// One message out of a datagram's bytes, named by the validated view `V` the
/// schema generates over it.
///
/// The length is the type on this seam, so it is checked before anything is read
/// out: the bytes of a message of another shape are not this message's fields.
/// Stated once over every subject rather than once per subject — the rule
/// belongs to the seam, and a copy of it per schema is a copy that drifts while
/// each one still looks whole.
///
/// # Errors
///
/// [`Refusal`] when the length is not the schema's, or the bytes are not a valid
/// message of it.
pub fn decode<V: ValidView>(bytes: &[u8]) -> Result<V::Raw, Refusal> {
    let message = blob_from_bytes::<V::Raw>(bytes).ok_or(Refusal::WrongSize {
        got: bytes.len(),
        want: <V::Raw as Blob>::SIZE,
    })?;
    validate::<V>(&message).map_err(Refusal::Invalid)?;
    Ok(message)
}

/// The two queues the loop thread drains, and the sockets feeding them.
pub struct Inbox {
    goals: Receiver<Inbound>,
    sessions: Receiver<Inbound>,
    counts: Arc<Counts>,
}

impl Inbox {
    /// Bind the driver's two ports on loopback and start reading them.
    ///
    /// # Errors
    ///
    /// Whatever the operating system said about binding a port. A driver that
    /// cannot bind exits: the port it wanted is held by something else, and two
    /// processes reading one command stream is worse than none.
    pub fn bind() -> io::Result<Self> {
        let (goals, sessions) = open(GOAL_PORT, SESSION_PORT)?;
        Ok(Self::from_sockets(goals, sessions))
    }

    /// As [`Inbox::bind`], reading two sockets the caller bound.
    ///
    /// The sockets rather than their port numbers, so that a test binds port
    /// zero, keeps what the operating system gave it, and sends to that — a test
    /// that asked for a free port, dropped it, and hoped to bind the same number
    /// again would be racing everything else on the machine.
    #[must_use]
    pub fn from_sockets(goals: UdpSocket, sessions: UdpSocket) -> Self {
        let counts = Arc::new(Counts::default());
        let (goals_in, goals_out) = sync_channel(QUEUE_DEPTH);
        let (sessions_in, sessions_out) = sync_channel(QUEUE_DEPTH);

        reader(
            goals,
            Subject::Goal,
            Arc::clone(&counts),
            goals_in,
            |bytes| decode::<GoalSetpoint>(bytes).map(Inbound::Goal),
        );
        reader(
            sessions,
            Subject::Session,
            Arc::clone(&counts),
            sessions_in,
            |bytes| decode::<SessionCmd>(bytes).map(Inbound::Session),
        );

        Self {
            goals: goals_out,
            sessions: sessions_out,
            counts,
        }
    }

    /// Hand every datagram waiting to `take`, and answer how many there were.
    ///
    /// Non-blocking: what has arrived is taken, what has not is next cycle's.
    /// The loop thread calls this once at the top of a cycle and never waits
    /// here — a cycle whose length depended on how much traffic arrived would be
    /// a cycle off the grid.
    ///
    /// Session commands first, then goals, each capped at [`QUEUE_DEPTH`] —
    /// which is that queue's whole contents, so the work a cycle does here is
    /// bounded by the seam and not by the sender. The order is deliberate: a
    /// cycle that took a de-torquing and a goal in the same drain applies the
    /// de-torquing having seen it, not next cycle.
    pub fn drain(&self, mut take: impl FnMut(Inbound)) -> usize {
        self.drain_one(&self.sessions, &mut take) + self.drain_one(&self.goals, &mut take)
    }

    /// Take one queue's whole contents, at most [`QUEUE_DEPTH`] of them.
    fn drain_one(&self, queue: &Receiver<Inbound>, take: &mut impl FnMut(Inbound)) -> usize {
        let mut taken = 0;
        while taken < QUEUE_DEPTH {
            match queue.try_recv() {
                Ok(message) => {
                    match &message {
                        Inbound::Goal(_) => &self.counts.goals,
                        Inbound::Session(_) => &self.counts.session_cmds,
                    }
                    .fetch_add(1, Ordering::Relaxed);
                    take(message);
                    taken += 1;
                }
                // Both ends of a reader thread dying leave the loop with an
                // empty queue. For the goal port that is silence, which the
                // dead-man answers; for the session port it is deafness, which
                // [`Inbox::reader_stopped`] notices.
                Err(TryRecvError::Empty | TryRecvError::Disconnected) => return taken,
            }
        }
        taken
    }

    /// Whether any reader thread has given up on its socket.
    ///
    /// A wire-failure condition, not a statistic: a port with no reader is a
    /// port whose datagrams — a `TorqueOffNow` among them — reach nothing. The
    /// loop treats this as the wire failure it is rather than letting the
    /// counter sit in a log line nobody reads until afterwards.
    #[must_use]
    pub fn reader_stopped(&self) -> bool {
        self.counts.readers_stopped.load(Ordering::Relaxed) > 0
    }

    /// What the seam has counted so far.
    #[must_use]
    pub fn counts(&self) -> &Counts {
        &self.counts
    }

    /// The counts as a handle that outlives this inbox.
    ///
    /// The reader threads count for as long as they run, which is past the point
    /// the loop stopped draining them, so the site that prints a run's final
    /// numbers holds this rather than borrowing from something it has dropped.
    #[must_use]
    pub fn counts_handle(&self) -> Arc<Counts> {
        Arc::clone(&self.counts)
    }
}

/// Bind the two ports of this seam on loopback.
///
/// Separated from [`Inbox::from_sockets`] because it is the step that can
/// fail, and the one a test about a port already held needs to exercise.
///
/// # Errors
///
/// The operating system's bind error.
pub fn open(goal_port: u16, session_port: u16) -> io::Result<(UdpSocket, UdpSocket)> {
    let goals = UdpSocket::bind((LOOPBACK, goal_port))?;
    let sessions = UdpSocket::bind((LOOPBACK, session_port))?;
    Ok((goals, sessions))
}

/// A run of consecutive socket errors, and when it is long enough to give up on.
///
/// Its own value rather than a counter inline in the loop below, because the
/// rule it carries — a good read resets the run, so only an *unbroken* run ends
/// a reader — is the difference between a transient and a dead socket, and a
/// timed test over a real socket cannot state it decisively.
#[derive(Clone, Copy, Debug, Default)]
struct ErrorRun {
    consecutive: u32,
}

impl ErrorRun {
    /// A read that worked: the run is over.
    fn read(&mut self) {
        self.consecutive = 0;
    }

    /// A read that failed. Answers whether the reader should give up.
    fn errored(&mut self) -> bool {
        self.consecutive += 1;
        self.consecutive >= MAX_CONSECUTIVE_RECV_ERRORS
    }
}

/// One reader thread: block, decode, push, repeat.
fn reader(
    socket: UdpSocket,
    subject: Subject,
    counts: Arc<Counts>,
    queue: SyncSender<Inbound>,
    decode: impl Fn(&[u8]) -> Result<Inbound, Refusal> + Send + 'static,
) {
    thread::spawn(move || {
        let mut buffer = [0u8; BUFFER_BYTES];
        let mut failures = ErrorRun::default();
        loop {
            let read = match socket.recv_from(&mut buffer) {
                Ok((read, _from)) => {
                    failures.read();
                    read
                }
                Err(error) if error.kind() == io::ErrorKind::Interrupted => continue,
                Err(_) => {
                    counts.recv_errors.fetch_add(1, Ordering::Relaxed);
                    if failures.errored() {
                        counts.readers_stopped.fetch_add(1, Ordering::Relaxed);
                        return;
                    }
                    continue;
                }
            };
            match decode(&buffer[..read]) {
                Ok(message) => match queue.try_send(message) {
                    Ok(()) => {
                        counts.queued.fetch_add(1, Ordering::Relaxed);
                    }
                    Err(TrySendError::Full(_)) => {
                        counts.overflowed.fetch_add(1, Ordering::Relaxed);
                    }
                    Err(TrySendError::Disconnected(_)) => {
                        counts.undelivered.fetch_add(1, Ordering::Relaxed);
                        return;
                    }
                },
                Err(refusal) => counts.refused(subject, refusal),
            }
        }
    });
}

#[cfg(test)]
mod tests {
    use super::{
        BUFFER_BYTES, ErrorRun, Inbound, Inbox, MAX_CONSECUTIVE_RECV_ERRORS, QUEUE_DEPTH, Refusal,
        Subject, decode, open,
    };
    use brenn_reachy__cogs__session_cmd_clk_rs::{SessionCmd, SessionCmdKind, SessionCmdWire};
    use brenn_reachy__driver__goal_clk_rs::{GoalSetpoint, GoalSetpointWire};
    use brenn_reachy__motion__joints_clk_rs::JointFlags;
    use clockwork_rs::{Blob, SyncTime, blob_as_bytes};
    use std::net::UdpSocket;
    use std::sync::atomic::Ordering;
    use std::time::{Duration, Instant};

    /// A round instant, so a number read out of the wrong field is visible.
    const T0: i64 = 1_700_000_000_000_000_000;

    /// A setpoint a control process would send.
    fn goal() -> GoalSetpointWire {
        let mut message = GoalSetpointWire::new();
        let goal = message.clear_valid();
        goal.execute_at = SyncTime::from_nanos(T0);
        goal.mask = JointFlags::BODY_YAW;
        goal.targets.body_yaw = 0.25;
        message
    }

    /// A keep-alive, which is the smallest thing the session says.
    fn keep_alive() -> SessionCmdWire {
        let mut message = SessionCmdWire::new();
        let cmd = message.clear_valid();
        cmd.kind = SessionCmdKind::KeepAlive;
        cmd.corr = 7;
        message
    }

    /// A command whose `kind` byte holds an enumerator the schema never
    /// declared: the right number of bytes, and the one thing on this seam that
    /// gets past the length check.
    ///
    /// Where that byte is, is found rather than stated: the generator lays
    /// fields out by alignment, so the offset is wherever setting `kind` to a
    /// second value moves a byte.
    fn undeclared_kind() -> Vec<u8> {
        let mut other = keep_alive();
        other
            .validate_mut()
            .expect("a written command validates")
            .kind = SessionCmdKind::TorqueOffNow;
        let mut bytes = blob_as_bytes(&keep_alive()).to_vec();
        let at = bytes
            .iter()
            .zip(blob_as_bytes(&other))
            .position(|(had, other)| had != other)
            .expect("two kinds are two byte patterns");
        bytes[at] = 0x7f;
        bytes
    }

    /// An inbox reading two sockets the operating system chose, with the ports
    /// it chose them on.
    ///
    /// The sockets are kept from the moment they are bound: asking for a free
    /// port, dropping it and binding the number again is a race against every
    /// other process on the machine, and the flake it produces is
    /// unattributable.
    fn reading() -> (Inbox, u16, u16) {
        let goals = UdpSocket::bind(("127.0.0.1", 0)).expect("a free port");
        let sessions = UdpSocket::bind(("127.0.0.1", 0)).expect("a second free port");
        let goal_port = goals.local_addr().expect("bound").port();
        let session_port = sessions.local_addr().expect("bound").port();
        (
            Inbox::from_sockets(goals, sessions),
            goal_port,
            session_port,
        )
    }

    /// Send `bytes` to a port on loopback.
    fn send(port: u16, bytes: &[u8]) {
        let socket = UdpSocket::bind(("127.0.0.1", 0)).expect("a sending socket");
        socket
            .send_to(bytes, ("127.0.0.1", port))
            .expect("loopback takes the datagram");
    }

    /// Drain until `want` messages have arrived, or give up. UDP on loopback
    /// does not reorder or lose, but it does arrive whenever the reader thread
    /// is scheduled, so a case waits rather than asserting on the first look.
    fn collect(inbox: &Inbox, want: usize) -> Vec<Inbound> {
        let deadline = Instant::now() + Duration::from_secs(5);
        let mut taken = Vec::new();
        while taken.len() < want && Instant::now() < deadline {
            inbox.drain(|message| taken.push(message));
            if taken.len() < want {
                std::thread::sleep(Duration::from_millis(1));
            }
        }
        taken
    }

    #[test]
    fn a_goal_and_a_session_command_arrive_typed_by_their_port() {
        let (inbox, goal_port, session_port) = reading();
        send(goal_port, blob_as_bytes(&goal()));
        send(session_port, blob_as_bytes(&keep_alive()));

        let taken = collect(&inbox, 2);
        assert_eq!(taken.len(), 2, "both datagrams arrived");
        let mut goals = 0;
        let mut sessions = 0;
        for message in taken {
            match message {
                Inbound::Goal(had) => {
                    goals += 1;
                    let had = had.validate().expect("a valid goal");
                    assert_eq!(had.execute_at.as_nanos(), T0);
                    assert_eq!(had.targets.body_yaw, 0.25);
                }
                Inbound::Session(had) => {
                    sessions += 1;
                    let had = had.validate().expect("a valid command");
                    assert_eq!(had.kind, SessionCmdKind::KeepAlive);
                    assert_eq!(had.corr, 7);
                }
            }
        }
        assert_eq!((goals, sessions), (1, 1));
        assert_eq!(inbox.counts().goals.load(Ordering::Relaxed), 1);
        assert_eq!(inbox.counts().session_cmds.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn a_datagram_of_the_wrong_length_is_counted_and_the_reader_keeps_reading() {
        let (inbox, goal_port, _session_port) = reading();
        let whole = blob_as_bytes(&goal()).to_vec();

        send(goal_port, &whole[..whole.len() - 1]);
        send(goal_port, &[]);
        send(goal_port, &[0u8; BUFFER_BYTES]);
        // A session command on the goal port is a datagram of the wrong length,
        // which is the whole of the type check on this seam.
        send(goal_port, blob_as_bytes(&keep_alive()));
        send(goal_port, &whole);

        let taken = collect(&inbox, 1);
        assert_eq!(taken.len(), 1, "the good datagram after the bad ones");
        assert!(matches!(taken[0], Inbound::Goal(_)));
        assert_eq!(inbox.counts().wrong_size.load(Ordering::Relaxed), 4);
        assert_eq!(inbox.counts().invalid.load(Ordering::Relaxed), 0);
        // Read as a log line would read it: the names, their order and which
        // counter each one names, pinned in one comparison. On a seam whose
        // whole type check is a length, a transposed pair here is a run record
        // that says the wrong thing about why datagrams were refused.
        assert_eq!(
            inbox.counts().read(),
            [
                ("queued", 1),
                ("goals", 1),
                ("session_cmds", 0),
                ("wrong_size", 4),
                ("invalid", 0),
                ("overflowed", 0),
                ("undelivered", 0),
                ("recv_errors", 0),
                ("readers_stopped", 0),
            ]
        );
    }

    #[test]
    fn a_datagram_that_does_not_validate_is_counted_and_never_taken() {
        let (inbox, _goal_port, session_port) = reading();
        send(session_port, &undeclared_kind());
        send(session_port, blob_as_bytes(&keep_alive()));

        let taken = collect(&inbox, 1);
        assert_eq!(taken.len(), 1, "only the valid one");
        assert!(matches!(taken[0], Inbound::Session(_)));
        assert_eq!(inbox.counts().invalid.load(Ordering::Relaxed), 1);
        assert_eq!(inbox.counts().wrong_size.load(Ordering::Relaxed), 0);
    }

    #[test]
    fn a_decode_names_the_size_it_wanted() {
        assert_eq!(
            decode::<GoalSetpoint>(&[]),
            Err(Refusal::WrongSize {
                got: 0,
                want: GoalSetpointWire::SIZE
            })
        );
        assert_eq!(
            decode::<SessionCmd>(&[0u8; 3]),
            Err(Refusal::WrongSize {
                got: 3,
                want: SessionCmdWire::SIZE
            })
        );
        assert!(matches!(
            decode::<SessionCmd>(&undeclared_kind()),
            Err(Refusal::Invalid(_))
        ));
    }

    #[test]
    fn draining_an_empty_queue_takes_nothing_and_does_not_wait() {
        let (inbox, _goal_port, _session_port) = reading();
        let started = Instant::now();
        assert_eq!(inbox.drain(|_| panic!("nothing was sent")), 0);
        assert!(
            started.elapsed() < Duration::from_millis(100),
            "a drain of an empty queue returns immediately"
        );
    }

    #[test]
    fn a_port_already_held_is_a_refusal_and_not_a_second_reader() {
        let held = UdpSocket::bind(("127.0.0.1", 0)).expect("holding a port");
        let taken = held.local_addr().expect("bound").port();
        assert!(
            open(taken, 0).is_err(),
            "two readers of one command stream is worse than none"
        );
        drop(held);
    }

    /// Send goals to `port` until the queue behind it has refused at least
    /// `want` of them, so the queue is known to be full and the surplus known
    /// to be gone rather than pending.
    fn flood_until_overflowed(inbox: &Inbox, port: u16, want: u64) {
        let whole = blob_as_bytes(&goal()).to_vec();
        let deadline = Instant::now() + Duration::from_secs(5);
        while inbox.counts().overflowed.load(Ordering::Relaxed) < want {
            assert!(
                Instant::now() < deadline,
                "the queue is bounded, so a sender in a loop reaches its refusals"
            );
            send(port, &whole);
        }
    }

    #[test]
    fn a_queue_the_loop_thread_is_not_draining_holds_its_depth_and_refuses_the_surplus() {
        let (inbox, goal_port, _session_port) = reading();
        flood_until_overflowed(&inbox, goal_port, 8);
        // Let the reader finish with what the socket still holds, so what is
        // measured below is a queue at rest rather than one being refilled.
        std::thread::sleep(Duration::from_millis(250));

        // The queue held exactly its depth: one drain hands the loop thread that
        // many and no more, and the surplus was refused rather than deferred to
        // the next cycle. An unbounded queue, or a drain that ran until empty,
        // fails one of these two.
        assert_eq!(inbox.drain(|_| {}), QUEUE_DEPTH);
        assert_eq!(
            inbox.drain(|_| panic!("the surplus was refused, not queued")),
            0
        );
    }

    #[test]
    fn a_flood_of_goals_does_not_cost_a_session_command_its_place() {
        let (inbox, goal_port, session_port) = reading();
        flood_until_overflowed(&inbox, goal_port, 8);
        // The goal queue is full and refusing. A de-torquing arriving now has
        // its own queue's whole depth available, and a shared queue is the one
        // arrangement where it would not: nothing gates de-torquing, a full
        // queue of goals included.
        send(session_port, blob_as_bytes(&keep_alive()));
        // Let it be queued before the first drain, so the drain this case is
        // about is one with traffic waiting on both ports.
        std::thread::sleep(Duration::from_millis(250));

        let deadline = Instant::now() + Duration::from_secs(5);
        let mut taken = Vec::new();
        while !taken.iter().any(|m| matches!(m, Inbound::Session(_))) {
            assert!(Instant::now() < deadline, "the session command arrived");
            inbox.drain(|message| taken.push(message));
            std::thread::sleep(Duration::from_millis(1));
        }
        assert!(
            matches!(taken[0], Inbound::Session(_)),
            "session commands are drained first, so a de-torquing is seen before the cycle's goals"
        );
    }

    /// An inbox whose goal socket answers a read with an error after `patience`,
    /// and a healthy session socket beside it.
    fn reading_a_failing_socket(patience: Duration) -> (Inbox, u16) {
        let goals = UdpSocket::bind(("127.0.0.1", 0)).expect("a free port");
        let sessions = UdpSocket::bind(("127.0.0.1", 0)).expect("a second free port");
        goals
            .set_read_timeout(Some(patience))
            .expect("a read timeout");
        let goal_port = goals.local_addr().expect("bound").port();
        (Inbox::from_sockets(goals, sessions), goal_port)
    }

    #[test]
    fn a_reader_whose_socket_only_fails_stops_instead_of_spinning() {
        let (inbox, _goal_port) = reading_a_failing_socket(Duration::from_millis(1));

        let deadline = Instant::now() + Duration::from_secs(10);
        while inbox.counts().readers_stopped.load(Ordering::Relaxed) == 0 {
            assert!(
                Instant::now() < deadline,
                "a socket in a permanent error state ends its reader rather than spinning on it"
            );
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(inbox.counts().readers_stopped.load(Ordering::Relaxed), 1);
        assert_eq!(
            inbox.counts().recv_errors.load(Ordering::Relaxed),
            u64::from(MAX_CONSECUTIVE_RECV_ERRORS),
            "it stopped on the run's length and not before or after it"
        );
        assert!(
            inbox.reader_stopped(),
            "a port with no reader is the wire failure the loop acts on"
        );
    }

    #[test]
    fn only_an_unbroken_run_of_errors_ends_a_reader() {
        // The rule, stated against known counts rather than against however many
        // timeouts a loaded machine happened to produce: a good read resets the
        // run, so a reader gives up only on an unbroken one.
        let mut run = ErrorRun::default();
        for error in 1..MAX_CONSECUTIVE_RECV_ERRORS {
            assert!(!run.errored(), "error {error} of the run is not the last");
        }
        assert!(
            run.errored(),
            "the run reaching its length is where a reader gives up"
        );

        let mut run = ErrorRun::default();
        for _lap in 0..3 {
            for error in 1..MAX_CONSECUTIVE_RECV_ERRORS {
                assert!(!run.errored(), "error {error} of a run that gets broken");
            }
            run.read();
        }
        assert!(
            !run.errored(),
            "runs broken by a good read never accumulate into one"
        );
    }

    #[test]
    fn a_run_of_errors_broken_by_a_good_datagram_does_not_end_the_reader() {
        // Over a real socket, where the count is whatever the scheduler makes it:
        // the margin is the point. Each gap is a little longer than the read
        // timeout, so it produces a timeout or two, and it would take roughly ten
        // times that to reach MAX_CONSECUTIVE_RECV_ERRORS inside one gap. The
        // decisive form of the reset rule is the case above.
        let (inbox, goal_port) = reading_a_failing_socket(Duration::from_millis(10));
        let whole = blob_as_bytes(&goal()).to_vec();

        for _ in 0..2 {
            std::thread::sleep(Duration::from_millis(15));
            send(goal_port, &whole);
        }

        let taken = collect(&inbox, 2);
        assert_eq!(taken.len(), 2, "both datagrams, either side of the errors");
        assert!(
            inbox.counts().recv_errors.load(Ordering::Relaxed) > 0,
            "the gaps produced the errors this case is about"
        );
        assert_eq!(
            inbox.counts().readers_stopped.load(Ordering::Relaxed),
            0,
            "a good read resets the run, so transient errors never end a reader"
        );
        assert!(!inbox.reader_stopped());
    }

    #[test]
    fn a_datagram_arriving_after_the_loop_is_gone_is_counted_undelivered() {
        let (inbox, goal_port, _session_port) = reading();
        // The counts outlive the inbox, which is why they are held by a handle:
        // the reader thread is still reading its socket after the loop that
        // drained it has gone.
        let counts = inbox.counts_handle();
        drop(inbox);
        send(goal_port, blob_as_bytes(&goal()));

        let deadline = Instant::now() + Duration::from_secs(5);
        while counts.undelivered.load(Ordering::Relaxed) == 0 {
            assert!(
                Instant::now() < deadline,
                "a decoded datagram with nowhere to go is counted, not dropped in silence"
            );
            std::thread::sleep(Duration::from_millis(1));
        }
        assert_eq!(counts.undelivered.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn the_last_refusal_says_which_port_and_what_was_wrong() {
        let (inbox, goal_port, session_port) = reading();
        send(goal_port, &[0u8; 3]);
        let deadline = Instant::now() + Duration::from_secs(5);
        while inbox.counts().wrong_size.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(1));
        }
        let kept = inbox.counts().last_refusal().expect("a refusal was kept");
        assert_eq!(kept.subject, Subject::Goal);
        assert_eq!(
            kept.refusal,
            Refusal::WrongSize {
                got: 3,
                want: GoalSetpointWire::SIZE
            }
        );

        // The most recent one, and it names the other port when that is where
        // the last bad datagram arrived.
        send(session_port, &undeclared_kind());
        let deadline = Instant::now() + Duration::from_secs(5);
        while inbox.counts().invalid.load(Ordering::Relaxed) == 0 && Instant::now() < deadline {
            std::thread::sleep(Duration::from_millis(1));
        }
        let kept = inbox.counts().last_refusal().expect("a refusal was kept");
        assert_eq!(kept.subject, Subject::Session);
        assert!(matches!(kept.refusal, Refusal::Invalid(_)));
        // What a log line would say, which is the point of keeping it.
        assert!(kept.to_string().contains("session command"));
    }
}
