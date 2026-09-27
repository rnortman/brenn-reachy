//! The head's pose as the control process publishes it: the newest usable
//! estimate, and the thread that keeps it.
//!
//! Every estimate the pose solver makes arrives on [`ESTIMATES_OUT_PORT`], one
//! per 20 ms sample, as the schema's bytes. [`LastPose`] keeps the newest
//! usable one; [`PoseFeed`] is the thread that reads the port into one, and
//! [`PoseReader`] is the handle a reader asks through.
//!
//! A thread of its own because the edge loop sleeps up to [`POLL`] in the
//! reports port, and a slot refreshed at that pace would be a quarter second
//! stale for a reader asking where the head is now.
//!
//! A datagram of the wrong size is kept out, and the first one is said. A feed
//! that cannot run is said when it stops. Neither is fatal: this process is not
//! the one whose death de-torques the machine, and a datagram it cannot read
//! costs a reader of the slot a pose, which the reader already has to handle.
//!
//! Loopback is the whole defence on this port. A local sender can put any pose
//! in the slot, and what that buys it is a different library pose chosen for a
//! look: a name the edge and the mover screen like any other.
//!
//! Nothing here decides anything about motion.
//!
//! [`ESTIMATES_OUT_PORT`]: reachy_edge::ESTIMATES_OUT_PORT

use std::io;
use std::net::UdpSocket;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex, MutexGuard, PoisonError};
use std::thread::JoinHandle;
use std::time::Duration;

use brenn_reachy__driver__pose_clk_rs::PoseEstimateWire;
use clockwork_rs::{Blob, SyncTime, blob_from_bytes};
use nalgebra::UnitQuaternion;
use reachy_edge::{POLL, edge_line_with, now};
use reachy_motion::record;
use serde_json::json;

use crate::sinks::Lines;
use crate::words::{POSE_UNFED, POSE_WRONG_SIZE};

/// How many socket errors in a row end the feed.
///
/// A transient socket error is answered by reading again. A socket in a
/// permanent error state answers every read at once, and a thread looping on
/// that takes a core on the board the control process runs on. So a run this
/// long is taken as permanent, and the thread stops and says so.
const MAX_CONSECUTIVE_RECV_ERRORS: u32 = 16;

/// Where the head was at one instant.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HeadAttitude {
    /// The estimate's `time_of_validity`: when the sample it came from was read.
    pub at: SyncTime,
    /// Head orientation in the body frame.
    pub head_quat_body: UnitQuaternion<f64>,
    /// The body yaw servo's measured angle, radians.
    pub body_yaw: f64,
}

/// A datagram that was not one estimate's size.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct WrongSize {
    /// How many bytes arrived.
    pub got: usize,
    /// How many an estimate is.
    pub want: usize,
}

/// The newest estimate that said something usable.
#[derive(Debug, Default)]
pub struct LastPose {
    usable: Option<HeadAttitude>,
}

impl LastPose {
    /// A slot holding nothing.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Take one datagram off the feed.
    ///
    /// An estimate that says something usable replaces the held one, and one
    /// that says nothing usable leaves it standing, so [`LastPose::latest`]
    /// answers with the newest usable estimate whatever arrived after it.
    ///
    /// # Errors
    ///
    /// [`WrongSize`] for a datagram that is not exactly one estimate's size,
    /// which is kept out.
    pub fn datagram(&mut self, bytes: &[u8]) -> Result<(), WrongSize> {
        let wire = blob_from_bytes::<PoseEstimateWire>(bytes).ok_or(WrongSize {
            got: bytes.len(),
            want: PoseEstimateWire::SIZE,
        })?;
        if let Some(attitude) = attitude_of(&wire) {
            self.usable = Some(attitude);
        }
        Ok(())
    }

    /// The newest usable estimate, if it is no older than `max_age` at `now`.
    ///
    /// The bound is inclusive. An estimate stamped after `now` is not used: both
    /// stamps are on one realtime clock, so a future stamp is a clock step, and
    /// nothing about it says where the head is now.
    #[must_use]
    pub fn latest(&self, now: SyncTime, max_age: Duration) -> Option<HeadAttitude> {
        let newest = self.usable?;
        let age = now.as_nanos().checked_sub(newest.at.as_nanos())?;
        let max = i64::try_from(max_age.as_nanos()).unwrap_or(i64::MAX);
        (0 <= age && age <= max).then_some(newest)
    }
}

/// What an estimate says about the head, or `None` when it says nothing usable:
/// bytes that do not validate, an estimate its solver marked invalid, a
/// non-finite body yaw, or a quaternion off unit length.
fn attitude_of(wire: &PoseEstimateWire) -> Option<HeadAttitude> {
    let estimate = wire.validate().ok()?;
    if !estimate.valid.get() {
        return None;
    }
    let body_yaw = estimate.joints.body_yaw;
    if !body_yaw.is_finite() {
        return None;
    }
    let pose = record::read_pose(&estimate.head_pos, &estimate.head_quat).ok()?;
    Some(HeadAttitude {
        at: estimate.time_of_validity,
        head_quat_body: pose.rotation,
        body_yaw,
    })
}

/// The slot, locked. A poisoned lock is taken as it is: [`LastPose::datagram`]
/// finishes each edit before it returns, so the slot is consistent after any
/// panic.
fn lock(last: &Mutex<LastPose>) -> MutexGuard<'_, LastPose> {
    last.lock().unwrap_or_else(PoisonError::into_inner)
}

/// A read handle on a feed's slot, cheap to clone and safe to hold on any thread.
#[derive(Clone, Debug)]
pub struct PoseReader(Arc<Mutex<LastPose>>);

impl PoseReader {
    /// As [`LastPose::latest`], on the feed's slot as it stands.
    #[must_use]
    pub fn latest(&self, now: SyncTime, max_age: Duration) -> Option<HeadAttitude> {
        lock(&self.0).latest(now, max_age)
    }

    /// A reader over a slot already holding `attitude`, with no feed behind it.
    #[cfg(test)]
    pub(crate) fn holding(attitude: HeadAttitude) -> Self {
        Self(Arc::new(Mutex::new(LastPose {
            usable: Some(attitude),
        })))
    }
}

/// The line saying the pose feed is not running, and `detail` for why.
#[must_use]
pub fn unfed_line(detail: &str, at: SyncTime) -> String {
    edge_line_with(
        POSE_UNFED,
        at,
        &format!(
            "the head's pose estimates are not being read: {detail}; until this host restarts \
             nothing here knows where the head is"
        ),
        &[("detail", json!(detail))],
    )
}

/// The line saying the first datagram of the wrong size arrived.
fn wrong_size_line(wrong: WrongSize, at: SyncTime) -> String {
    let WrongSize { got, want } = wrong;
    edge_line_with(
        POSE_WRONG_SIZE,
        at,
        &format!(
            "a datagram on the pose feed port was {got} bytes where an estimate is {want}; it and \
             every later one of the wrong size are kept out, and this is said once"
        ),
        &[("got", json!(got)), ("want", json!(want))],
    )
}

/// What one receive on the port asks of the reading thread.
#[derive(Debug, PartialEq, Eq)]
enum Received {
    /// A datagram of this many bytes is in the buffer.
    Datagram(usize),
    /// Nothing to take: a timeout, an interrupted read, or a socket error
    /// short of a run that ends the feed. Read again.
    Again,
    /// A run of socket errors long enough to end the feed, and why.
    Ended(String),
}

/// The run of socket errors the reading thread is in.
#[derive(Debug, Default)]
struct ErrorRun {
    consecutive: u32,
}

impl ErrorRun {
    /// Count one receive, and say what the thread does next. A datagram, a
    /// timeout or an interrupted read ends the run; any other error extends it,
    /// and the [`MAX_CONSECUTIVE_RECV_ERRORS`]th in a row ends the feed.
    fn received(&mut self, result: io::Result<usize>) -> Received {
        match result {
            Ok(read) => {
                self.consecutive = 0;
                Received::Datagram(read)
            }
            Err(error)
                if matches!(
                    error.kind(),
                    io::ErrorKind::WouldBlock
                        | io::ErrorKind::TimedOut
                        | io::ErrorKind::Interrupted
                ) =>
            {
                self.consecutive = 0;
                Received::Again
            }
            Err(error) => {
                self.consecutive += 1;
                if self.consecutive >= MAX_CONSECUTIVE_RECV_ERRORS {
                    Received::Ended(format!(
                        "reading the pose feed port: {MAX_CONSECUTIVE_RECV_ERRORS} errors in a \
                         row, the last {error}"
                    ))
                } else {
                    Received::Again
                }
            }
        }
    }
}

/// The thread reading the pose feed port into a slot.
pub struct PoseFeed {
    last: Arc<Mutex<LastPose>>,
    stop: Arc<AtomicBool>,
    lines: Arc<dyn Lines>,
    thread: JoinHandle<()>,
}

impl PoseFeed {
    /// Start reading `socket` into a fresh slot, saying on `lines` the first
    /// datagram of the wrong size and why the thread stopped if it stops on its
    /// own or panics.
    ///
    /// Sets the socket's read timeout to [`POLL`], which is how the thread sees
    /// a stop, so a caller hands over the bound socket and nothing else.
    ///
    /// # Errors
    ///
    /// The read timeout could not be set, or the thread could not be spawned.
    pub fn start(socket: UdpSocket, lines: Arc<dyn Lines>) -> io::Result<Self> {
        socket.set_read_timeout(Some(POLL))?;
        let last = Arc::new(Mutex::new(LastPose::new()));
        let stop = Arc::new(AtomicBool::new(false));
        let thread = {
            let last = Arc::clone(&last);
            let stop = Arc::clone(&stop);
            let lines = Arc::clone(&lines);
            std::thread::Builder::new()
                .name("pose-feed".to_owned())
                .spawn(move || {
                    if let Err(detail) = read(&socket, &last, &stop, lines.as_ref()) {
                        lines.say(unfed_line(&detail, now()));
                    }
                })?
        };
        Ok(Self {
            last,
            stop,
            lines,
            thread,
        })
    }

    /// A read handle on this feed's slot.
    #[must_use]
    pub fn reader(&self) -> PoseReader {
        PoseReader(Arc::clone(&self.last))
    }

    /// Stop the thread and wait for it, which takes at most one [`POLL`]. A
    /// thread that had stopped on its own said why when it did; one that
    /// panicked could not, so it is said here.
    pub fn stop(self) {
        self.stop.store(true, Ordering::Relaxed);
        if self.thread.join().is_err() {
            self.lines
                .say(unfed_line("the pose feed's thread panicked", now()));
        }
    }
}

/// Read datagrams into the slot until `stop` is set, or until
/// [`MAX_CONSECUTIVE_RECV_ERRORS`] socket errors in a row.
///
/// The buffer has one byte to spare, so an oversize datagram reads as one byte
/// too long and is refused wrong-size rather than truncated into a false fit.
fn read(
    socket: &UdpSocket,
    last: &Mutex<LastPose>,
    stop: &AtomicBool,
    lines: &dyn Lines,
) -> Result<(), String> {
    let mut buffer = vec![0u8; PoseEstimateWire::SIZE + 1];
    let mut errors = ErrorRun::default();
    let mut told_wrong_size = false;
    while !stop.load(Ordering::Relaxed) {
        match errors.received(socket.recv_from(&mut buffer).map(|(read, _)| read)) {
            Received::Datagram(read) => {
                let took = lock(last).datagram(&buffer[..read]);
                if let Err(wrong) = took
                    && !told_wrong_size
                {
                    told_wrong_size = true;
                    lines.say(wrong_size_line(wrong, now()));
                }
            }
            Received::Again => {}
            Received::Ended(detail) => return Err(detail),
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use clockwork_rs::blob_as_bytes;
    use nalgebra::{Isometry3, Translation3};
    use reachy_edge::LOOPBACK;
    use serde_json::Value;
    use std::time::Instant;

    /// Every line said, in order.
    #[derive(Default)]
    struct Said(Mutex<Vec<String>>);

    impl Lines for Said {
        fn say(&self, line: String) {
            self.0.lock().expect("an unpoisoned recorder").push(line);
        }
    }

    impl Said {
        fn lines(&self) -> Vec<Value> {
            self.0
                .lock()
                .expect("an unpoisoned recorder")
                .iter()
                .map(|line| serde_json::from_str(line).expect("a JSON line"))
                .collect()
        }
    }

    const T0: i64 = 1_700_000_000_000_000_000;
    const MS: i64 = 1_000_000;

    fn estimate(at_ns: i64, head: &UnitQuaternion<f64>, body_yaw: f64) -> PoseEstimateWire {
        let mut wire = PoseEstimateWire::new();
        let v = wire.clear_valid();
        record::write_pose(
            &mut v.head_pos,
            &mut v.head_quat,
            &Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.17), *head),
        );
        v.joints.body_yaw = body_yaw;
        v.valid = true.into();
        v.time_of_validity = SyncTime::from_nanos(at_ns);
        wire
    }

    fn invalid(at_ns: i64) -> PoseEstimateWire {
        let mut wire = estimate(at_ns, &UnitQuaternion::identity(), 0.0);
        wire.validate_mut()
            .expect("a written estimate validates")
            .valid = false.into();
        wire
    }

    fn bytes(wire: &PoseEstimateWire) -> Vec<u8> {
        blob_as_bytes(wire).to_vec()
    }

    fn yawed(yaw: f64) -> UnitQuaternion<f64> {
        UnitQuaternion::from_euler_angles(0.0, 0.0, yaw)
    }

    fn at(ns: i64) -> SyncTime {
        SyncTime::from_nanos(ns)
    }

    #[test]
    fn the_newest_usable_estimate_is_the_head() {
        let mut last = LastPose::new();
        last.datagram(&bytes(&estimate(T0, &yawed(0.1), 0.1)))
            .expect("one estimate's size");
        last.datagram(&bytes(&estimate(T0 + 20 * MS, &yawed(0.2), 0.2)))
            .expect("one estimate's size");
        let want = yawed(0.3);
        last.datagram(&bytes(&estimate(T0 + 40 * MS, &want, 0.3)))
            .expect("one estimate's size");

        let got = last
            .latest(at(T0 + 50 * MS), Duration::from_millis(200))
            .expect("a fresh estimate");
        assert_eq!(got.at, at(T0 + 40 * MS));
        assert_eq!(got.body_yaw, 0.3);
        assert!(got.head_quat_body.angle_to(&want) < 1e-12);
    }

    #[test]
    fn an_estimate_that_says_nothing_usable_is_skipped() {
        let mut last = LastPose::new();
        last.datagram(&bytes(&estimate(T0, &yawed(0.1), 0.1)))
            .expect("one estimate's size");
        last.datagram(&bytes(&invalid(T0 + 20 * MS)))
            .expect("one estimate's size");
        let mut off_unit = estimate(T0 + 40 * MS, &yawed(0.2), 0.2);
        off_unit
            .validate_mut()
            .expect("a written estimate validates")
            .head_quat
            .w = 2.0;
        last.datagram(&bytes(&off_unit))
            .expect("one estimate's size");
        last.datagram(&bytes(&estimate(T0 + 60 * MS, &yawed(0.3), f64::NAN)))
            .expect("one estimate's size");

        let got = last
            .latest(at(T0 + 70 * MS), Duration::from_millis(200))
            .expect("the first estimate");
        assert_eq!(got.at, at(T0));
        assert_eq!(got.body_yaw, 0.1);
    }

    #[test]
    fn a_stale_or_future_estimate_is_no_pose() {
        let mut last = LastPose::new();
        last.datagram(&bytes(&estimate(T0, &yawed(0.1), 0.1)))
            .expect("one estimate's size");
        let bound = Duration::from_millis(200);
        assert!(last.latest(at(T0 + 200 * MS), bound).is_some());
        assert!(last.latest(at(T0 + 200 * MS + 1), bound).is_none());
        assert!(last.latest(at(T0 - 1), bound).is_none());
    }

    #[test]
    fn unusable_estimates_leave_the_last_usable_one_standing() {
        let mut last = LastPose::new();
        last.datagram(&bytes(&estimate(T0, &yawed(0.1), 0.1)))
            .expect("one estimate's size");
        for _ in 0..100 {
            last.datagram(&bytes(&invalid(T0)))
                .expect("one estimate's size");
        }
        let got = last
            .latest(at(T0 + 10 * MS), Duration::from_secs(1))
            .expect("the usable estimate");
        assert_eq!(got.at, at(T0));
    }

    #[test]
    fn a_datagram_of_the_wrong_size_is_refused_and_kept_out() {
        let mut last = LastPose::new();
        let size = PoseEstimateWire::SIZE;
        let good = bytes(&estimate(T0, &yawed(0.1), 0.1));
        assert_eq!(
            last.datagram(&good[..good.len() - 1]),
            Err(WrongSize {
                got: size - 1,
                want: size
            })
        );
        assert_eq!(last.datagram(&[]), Err(WrongSize { got: 0, want: size }));
        let mut long = good.clone();
        long.push(0);
        assert_eq!(
            last.datagram(&long),
            Err(WrongSize {
                got: size + 1,
                want: size
            })
        );
        let bound = Duration::from_millis(200);
        assert!(last.latest(at(T0), bound).is_none());

        assert_eq!(last.datagram(&good), Ok(()));
        assert_eq!(last.latest(at(T0), bound).map(|got| got.at), Some(at(T0)));
        assert!(last.datagram(&long).is_err());
        assert_eq!(last.latest(at(T0), bound).map(|got| got.at), Some(at(T0)));
    }

    #[test]
    fn the_feed_reads_its_port_until_stopped() {
        let socket = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
        let port = socket.local_addr().expect("a bound address");
        let said = Arc::new(Said::default());
        let feed =
            PoseFeed::start(socket, Arc::clone(&said) as Arc<dyn Lines>).expect("the feed starts");

        let sender = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
        let head = yawed(0.2);
        sender
            .send_to(&bytes(&estimate(now().as_nanos(), &head, 0.3)), port)
            .expect("a sent datagram");

        let reader = feed.reader();
        let deadline = Instant::now() + Duration::from_secs(5);
        let got = loop {
            if let Some(got) = reader.latest(now(), Duration::from_secs(1)) {
                break got;
            }
            assert!(Instant::now() < deadline, "the feed read nothing in 5 s");
            std::thread::sleep(Duration::from_millis(5));
        };
        assert_eq!(got.body_yaw, 0.3);

        let asked = Instant::now();
        feed.stop();
        assert!(asked.elapsed() < 2 * POLL);
        assert!(said.lines().is_empty(), "nothing is said");
    }

    #[test]
    fn the_first_wrong_size_datagram_is_said_once() {
        let socket = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
        let port = socket.local_addr().expect("a bound address");
        let said = Arc::new(Said::default());
        let feed =
            PoseFeed::start(socket, Arc::clone(&said) as Arc<dyn Lines>).expect("the feed starts");

        let sender = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
        let good = bytes(&estimate(now().as_nanos(), &yawed(0.2), 0.3));
        sender.send_to(&[], port).expect("a sent datagram");
        sender
            .send_to(&good[..good.len() - 1], port)
            .expect("a sent datagram");
        sender.send_to(&good, port).expect("a sent datagram");

        let reader = feed.reader();
        let deadline = Instant::now() + Duration::from_secs(5);
        while reader.latest(now(), Duration::from_secs(1)).is_none() {
            assert!(Instant::now() < deadline, "the feed read nothing in 5 s");
            std::thread::sleep(Duration::from_millis(5));
        }
        feed.stop();

        let lines = said.lines();
        assert_eq!(lines.len(), 1, "{lines:?}");
        assert_eq!(lines[0]["kind"], "pose_wrong_size");
        assert_eq!(lines[0]["got"], 0);
        assert_eq!(lines[0]["want"], PoseEstimateWire::SIZE);
    }

    #[test]
    fn an_oversize_datagram_on_the_port_is_refused_not_truncated() {
        let socket = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
        let port = socket.local_addr().expect("a bound address");
        let said = Arc::new(Said::default());
        let feed =
            PoseFeed::start(socket, Arc::clone(&said) as Arc<dyn Lines>).expect("the feed starts");
        let sender = UdpSocket::bind((LOOPBACK, 0)).expect("an ephemeral port");
        let reader = feed.reader();

        sender
            .send_to(&bytes(&estimate(now().as_nanos(), &yawed(0.2), 0.3)), port)
            .expect("a sent datagram");
        let deadline = Instant::now() + Duration::from_secs(5);
        while reader.latest(now(), Duration::from_secs(60)).is_none() {
            assert!(Instant::now() < deadline, "the feed read nothing in 5 s");
            std::thread::sleep(Duration::from_millis(5));
        }

        let mut oversize = bytes(&estimate(now().as_nanos(), &yawed(0.2), 0.9));
        oversize.push(0u8);
        sender.send_to(&oversize, port).expect("a sent datagram");
        let deadline = Instant::now() + Duration::from_secs(5);
        while said.lines().is_empty() {
            assert!(
                Instant::now() < deadline,
                "no wrong-size line in 5 s: an oversize datagram was taken as an estimate"
            );
            std::thread::sleep(Duration::from_millis(5));
        }

        let lines = said.lines();
        assert_eq!(lines.len(), 1, "{lines:?}");
        assert_eq!(lines[0]["kind"], "pose_wrong_size");
        assert_eq!(lines[0]["got"], PoseEstimateWire::SIZE + 1);
        assert_eq!(lines[0]["want"], PoseEstimateWire::SIZE);
        let got = reader
            .latest(now(), Duration::from_secs(60))
            .expect("the estimate before it");
        assert_eq!(got.body_yaw, 0.3);
        feed.stop();
    }

    fn refused() -> io::Result<usize> {
        Err(io::Error::from(io::ErrorKind::ConnectionRefused))
    }

    #[test]
    fn sixteen_socket_errors_in_a_row_end_the_feed() {
        let mut errors = ErrorRun::default();
        for _ in 0..15 {
            assert_eq!(errors.received(refused()), Received::Again);
        }
        match errors.received(refused()) {
            Received::Ended(detail) => {
                assert!(detail.contains("16 errors in a row"), "{detail}");
            }
            other => panic!("the 16th error in a row ends the feed, not {other:?}"),
        }
    }

    #[test]
    fn timeouts_and_datagrams_end_a_run_of_errors_and_never_the_feed() {
        for kind in [
            io::ErrorKind::WouldBlock,
            io::ErrorKind::TimedOut,
            io::ErrorKind::Interrupted,
        ] {
            let quiet = || Err(io::Error::from(kind));
            let mut errors = ErrorRun::default();
            for _ in 0..1_000 {
                assert_eq!(errors.received(quiet()), Received::Again, "{kind:?}");
            }
            for _ in 0..15 {
                assert_eq!(errors.received(refused()), Received::Again, "{kind:?}");
            }
            assert_eq!(errors.received(quiet()), Received::Again, "{kind:?}");
            for _ in 0..15 {
                assert_eq!(errors.received(refused()), Received::Again, "{kind:?}");
            }
            assert!(
                matches!(errors.received(refused()), Received::Ended(_)),
                "{kind:?}"
            );
        }

        let mut errors = ErrorRun::default();
        for _ in 0..15 {
            assert_eq!(errors.received(refused()), Received::Again);
        }
        assert_eq!(errors.received(Ok(7)), Received::Datagram(7));
        for _ in 0..15 {
            assert_eq!(errors.received(refused()), Received::Again);
        }
        assert!(matches!(errors.received(refused()), Received::Ended(_)));
    }

    #[test]
    fn the_unfed_line_names_what_stopped_the_feed() {
        let line: Value = serde_json::from_str(&unfed_line("x", at(T0))).expect("a JSON line");
        assert_eq!(line["kind"], "pose_unfed");
        assert_eq!(line["detail"], "x");
        assert_eq!(line["at_ns"], T0);
    }
}
