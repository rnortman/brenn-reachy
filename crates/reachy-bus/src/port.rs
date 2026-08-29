//! The port seam: three operations and one deadline contract.
//!
//! `read_some` returning `Ok(0)` means the deadline passed with nothing more to
//! hand back. It never means an error the caller has to interpret. A serial
//! read that runs out of time surfaces as `TimedOut` on one platform, as
//! `WouldBlock` on another and as a zero-length read on a third, and all three
//! mean the same thing to a transaction. The mapping happens once, here, so no
//! code above this module ever matches on an [`io::ErrorKind`].
//!
//! The trait is three lines wide rather than the serial crate's full interface
//! because three lines is all a transaction needs, and it is what makes the
//! whole transaction layer testable against a scripted port with no hardware
//! and no timing luck.
//!
//! The device is opened for exclusive use. Nine servos on one half-duplex line
//! tolerate exactly one speaker: a second process writing frames into the same
//! bus turns every reply into somebody's corrupt frame. The second opener is
//! refused by name, immediately — a typed [`OpenError::PortBusy`], never a wait
//! and never a shared line.

use std::fs::OpenOptions;
use std::io::{self, Read, Write};
use std::os::unix::fs::OpenOptionsExt as _;
use std::time::Instant;

use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits, TTYPort};
use thiserror::Error;

/// The servo chain's baud rate. Fixed by how the servos are provisioned, not a
/// tuning knob.
pub const DEFAULT_BAUD: u32 = 1_000_000;

/// A byte pipe with a deadline.
pub trait BusPort {
    /// Sends every byte of `buf`.
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()>;

    /// Reads whatever has arrived, blocking until at least one byte is there or
    /// `deadline` passes. `Ok(0)` means the deadline passed — the one and only
    /// way this reports silence.
    fn read_some(&mut self, buf: &mut [u8], deadline: Instant) -> io::Result<usize>;

    /// Drops anything already waiting in the receive buffer, so the tail of an
    /// abandoned exchange cannot be read as the head of the next one.
    fn discard_input(&mut self) -> io::Result<()>;
}

/// Why a device could not be opened for exclusive use.
///
/// Three failures that read differently to an operator: somebody else has the
/// bus, the device is not there or not usable, or the lock call itself failed.
/// Each names the device, because a unit has more than one serial node and the
/// one in the message is the one to go and look at.
#[derive(Debug, Error)]
pub enum OpenError {
    /// Another process holds the advisory lock on this device. Reported at
    /// once: a bus with two speakers is not a bus that waits its turn.
    #[error("{path} is already held by another process")]
    PortBusy {
        /// The device node named in the configuration.
        path: String,
    },

    /// The serial device could not be opened or configured.
    #[error("opening {path}")]
    Port {
        /// The device node named in the configuration.
        path: String,
        /// The serial crate's refusal.
        source: serialport::Error,
    },
}

/// The real serial port: 8 data bits, no parity, one stop bit, no flow control
/// — the framing the servos speak, and the framing the wire-time arithmetic in
/// [`crate::bus::BusTiming`] assumes at ten bits per byte.
#[derive(Debug)]
pub struct SerialBusPort {
    port: TTYPort,
}

impl SerialBusPort {
    /// Opens `path` at `baud`, exclusively.
    ///
    /// Exclusivity is the serial crate's exclusive open (`TIOCEXCL` + `flock`,
    /// held for the port's lifetime). No second lock is added here: `flock` is
    /// per open file description, so one taken on our own descriptor for the
    /// same node would contend with the port's and refuse every open.
    ///
    /// What this layer adds is the typed refusal. A device somebody else holds
    /// comes back as [`OpenError::PortBusy`] rather than as an open failure that
    /// reads like a missing device — the two send an operator to different
    /// places, and underneath they are the same `NoDevice` prose.
    ///
    /// The initial timeout is nominal: every read replaces it with the time
    /// left on that transaction's own deadline.
    pub fn open(path: &str, baud: u32) -> Result<Self, OpenError> {
        let port = serialport::new(path, baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            // Stated rather than inherited: the whole refusal below rests on
            // this being on, and a default is not a guarantee.
            .exclusive(true)
            .timeout(std::time::Duration::from_millis(10))
            .open_native()
            .map_err(|source| {
                if device_is_held(path) {
                    OpenError::PortBusy {
                        path: path.to_owned(),
                    }
                } else {
                    OpenError::Port {
                        path: path.to_owned(),
                        source,
                    }
                }
            })?;
        Ok(Self { port })
    }
}

/// Whether some other open descriptor holds this device against us.
///
/// Run only after an open has already failed, to say which failure it was. An
/// exclusive holder leaves two signatures and which one shows up depends on the
/// caller's privilege: an ordinary process meets `EBUSY` from the open itself,
/// which is what `TIOCEXCL` does to every later opener, while a process with
/// `CAP_SYS_ADMIN` opens straight past `TIOCEXCL` and instead meets a `flock`
/// the kernel will not hand over. Both are checked, because the daemon and the
/// bench may well both run as root on the same unit. Anything else — including
/// a device that is not there — is not contention, and says so by returning
/// false so the opener's own error survives.
///
/// The probe's own lock is dropped immediately; it is asked, never kept. It is
/// still an acquire: `flock` offers no way to ask without taking, so for the
/// width of the `try_lock` call a device nobody holds is a device this process
/// holds. Another opener landing inside that window is refused by the probe
/// rather than by a real holder, and — finding the lock free by the time it runs
/// its own probe — reports the refusal as a device error instead of contention.
/// The window is inherent to `flock` and the outcome is a misclassified refusal,
/// never a second speaker on the line.
///
/// `O_NONBLOCK` so the open cannot hang on a device asserting no carrier, and
/// `O_NOCTTY` so a bus device never becomes the process's controlling terminal.
fn device_is_held(path: &str) -> bool {
    let probe = match OpenOptions::new()
        .read(true)
        .custom_flags(libc::O_NONBLOCK | libc::O_NOCTTY)
        .open(path)
    {
        Ok(probe) => probe,
        Err(error) => return error.raw_os_error() == Some(libc::EBUSY),
    };
    matches!(probe.try_lock(), Err(std::fs::TryLockError::WouldBlock))
}

#[cfg(test)]
impl SerialBusPort {
    /// Wraps an already-open tty. Lets the deadline contract be exercised over
    /// a pty pair, which is the same kernel path as a serial device without a
    /// serial device.
    fn over(port: TTYPort) -> Self {
        Self { port }
    }
}

impl BusPort for SerialBusPort {
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        // The write returns once the kernel has the bytes; nothing here waits
        // for them to leave the UART. The reply is still waited for over a
        // window that covers them, because the exchange's deadline is taken
        // before the write and `BusTiming::worst_exchange` includes the
        // request's own wire time. The one write that can start behind bytes
        // still draining is the one after a broadcast, which nothing answers:
        // at most a single frame, sub-millisecond at this baud, well inside the
        // host allowance the next exchange runs on.
        Write::write_all(&mut self.port, buf)
    }

    fn read_some(&mut self, buf: &mut [u8], deadline: Instant) -> io::Result<usize> {
        let remaining = deadline.saturating_duration_since(Instant::now());
        if remaining.is_zero() {
            return Ok(0);
        }
        self.port.set_timeout(remaining)?;
        match self.port.read(buf) {
            Ok(n) => Ok(n),
            Err(e)
                if matches!(
                    e.kind(),
                    io::ErrorKind::TimedOut | io::ErrorKind::WouldBlock
                ) =>
            {
                Ok(0)
            }
            Err(e) => Err(e),
        }
    }

    fn discard_input(&mut self) -> io::Result<()> {
        self.port.clear(ClearBuffer::Input)?;
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use std::fs::File;
    use std::path::PathBuf;
    use std::sync::atomic::{AtomicU32, Ordering};
    use std::time::Duration;

    use super::*;

    /// Two ends of a pty, as `SerialBusPort`s. The kernel path is the one a
    /// real serial device takes; what differs is only what is on the far end.
    fn pair() -> (SerialBusPort, SerialBusPort) {
        let (near, far) = TTYPort::pair().expect("a pty pair");
        (SerialBusPort::over(near), SerialBusPort::over(far))
    }

    /// Bytes written on one end arrive on the other, and `read_some` hands back
    /// what it has rather than waiting for the deadline it was given.
    #[test]
    fn what_is_written_on_one_end_is_read_on_the_other() {
        let (mut near, mut far) = pair();
        near.write_all(&[0xFF, 0xFF, 0xFD, 0x00])
            .expect("the write");

        let mut buf = [0u8; 16];
        let deadline = Instant::now() + Duration::from_secs(2);
        let mut got = Vec::new();
        while got.len() < 4 {
            let n = far.read_some(&mut buf, deadline).expect("the read");
            assert_ne!(n, 0, "the bytes were sent well inside the deadline");
            got.extend_from_slice(&buf[..n]);
        }
        assert_eq!(got, [0xFF, 0xFF, 0xFD, 0x00]);
    }

    /// The contract the whole transaction layer rests on: silence is `Ok(0)`,
    /// never an error. A timed-out read surfaces as `TimedOut` on one platform
    /// and `WouldBlock` on another, and if either ever reached the transaction
    /// layer it would arrive as `XactError::Io` — which is not retryable, so
    /// the entire retry budget would silently stop applying.
    #[test]
    fn a_deadline_that_passes_with_nothing_on_the_line_is_not_an_error() {
        let (_near, mut far) = pair();
        let mut buf = [0u8; 16];

        let waited_from = Instant::now();
        let n = far
            .read_some(&mut buf, waited_from + Duration::from_millis(50))
            .expect("silence is not a failure");
        assert_eq!(n, 0);
        assert!(
            waited_from.elapsed() >= Duration::from_millis(25),
            "the read waits for the deadline rather than returning at once"
        );

        // A deadline already behind us needs no port call at all.
        let past = Instant::now() - Duration::from_millis(1);
        let started = Instant::now();
        assert_eq!(
            far.read_some(&mut buf, past).expect("still not a failure"),
            0
        );
        assert!(started.elapsed() < Duration::from_millis(20));
    }

    /// A pty whose slave node can be opened by path, and the master that keeps
    /// it alive. The slave is dropped: the node stays while the master is open,
    /// which is what lets `open` be called on it the way it is called on a
    /// serial device.
    fn openable_node() -> (TTYPort, String) {
        let (master, slave) = TTYPort::pair().expect("a pty pair");
        let name = slave.name().expect("the slave's path");
        drop(slave);
        (master, name)
    }

    /// Nine servos on one half-duplex line tolerate one speaker. The second
    /// opener is refused by name rather than joining in, and the refusal is the
    /// contention one — not a device error an operator would go chasing.
    #[test]
    fn a_second_open_of_the_same_device_is_refused() {
        let (_master, node) = openable_node();

        let held = SerialBusPort::open(&node, DEFAULT_BAUD).expect("the first open");
        let refused = SerialBusPort::open(&node, DEFAULT_BAUD).expect_err("the second open");
        match &refused {
            OpenError::PortBusy { path } => assert_eq!(*path, node),
            other => panic!("expected a busy refusal, got {other}"),
        }
        assert!(
            refused.to_string().contains(&node),
            "the refusal names the device: {refused}"
        );
        drop(held);
    }

    /// The lock is the port's, so it goes when the port goes. Without this a
    /// bench session that ended cleanly would leave the daemon unable to open
    /// the bus until the process exited.
    #[test]
    fn dropping_the_port_releases_the_device() {
        let (_master, node) = openable_node();

        let held = SerialBusPort::open(&node, DEFAULT_BAUD).expect("the first open");
        drop(held);

        let reopened = SerialBusPort::open(&node, DEFAULT_BAUD);
        assert!(
            reopened.is_ok(),
            "the device is free once the port that held it is gone"
        );
    }

    /// A device that is not there is not a device somebody else is using. The
    /// two refusals send an operator to different places, so they stay apart.
    #[test]
    fn a_device_that_is_not_there_is_not_reported_as_busy() {
        let missing = "/dev/null/reachy-bus-no-such-device";
        match SerialBusPort::open(missing, DEFAULT_BAUD).expect_err("no such device") {
            OpenError::Port { path, .. } => assert_eq!(path, missing),
            other => panic!("expected an open failure, got {other}"),
        }
    }

    /// A scratch path in the temp dir, unique per process and per call.
    fn scratch_path() -> PathBuf {
        static NEXT: AtomicU32 = AtomicU32::new(0);
        std::env::temp_dir().join(format!(
            "reachy-bus-probe-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ))
    }

    /// The `flock` half of the contention check, exercised without a privilege.
    ///
    /// The two signatures an exclusive holder leaves are chosen by the opener's
    /// privilege, so a test against a real tty only ever runs whichever one the
    /// runner happens to have — and the `flock` arm is the one that matters on
    /// the unit, where the bench and the daemon may both be root and
    /// `TIOCEXCL` refuses neither. `flock` is per open file description, so a
    /// second descriptor on the same path contends even inside one process:
    /// that is the same refusal the kernel hands a privileged second opener,
    /// on any file.
    #[test]
    fn a_locked_node_reads_as_held_without_a_privileged_opener() {
        let path = scratch_path();
        let name = path.to_str().expect("the scratch path is utf-8").to_owned();
        let holder = File::create(&path).expect("the scratch file");
        holder.lock().expect("nothing else holds the scratch file");

        assert!(
            device_is_held(&name),
            "a lock the kernel will not hand over is contention"
        );

        drop(holder);
        std::fs::remove_file(&path).expect("the scratch file is removed");
    }

    /// The other direction, and the one that keeps the probe from swallowing
    /// every open failure: a node nobody holds is not contended, so the
    /// opener's own error survives.
    #[test]
    fn an_unlocked_node_is_not_reported_as_held() {
        let path = scratch_path();
        let name = path.to_str().expect("the scratch path is utf-8").to_owned();
        File::create(&path).expect("the scratch file");

        assert!(!device_is_held(&name), "nothing holds it");

        std::fs::remove_file(&path).expect("the scratch file is removed");
    }

    /// Stale bytes on the line are gone after a discard, so nothing from a
    /// prior exchange contaminates the next read.
    #[test]
    fn discarding_the_input_leaves_nothing_to_read() {
        let (mut near, mut far) = pair();
        near.write_all(b"stale").expect("the write");
        // Give the bytes time to cross before flushing them, or the flush is
        // testing nothing.
        std::thread::sleep(Duration::from_millis(50));
        far.discard_input().expect("the flush");

        let mut buf = [0u8; 16];
        let n = far
            .read_some(&mut buf, Instant::now() + Duration::from_millis(50))
            .expect("silence is not a failure");
        assert_eq!(n, 0, "the residue is gone");
    }
}
