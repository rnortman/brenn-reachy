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

use std::io::{self, Read, Write};
use std::time::Instant;

use serialport::{ClearBuffer, DataBits, FlowControl, Parity, SerialPort, StopBits, TTYPort};

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

/// The real serial port: 8 data bits, no parity, one stop bit, no flow control
/// — the framing the servos speak, and the framing the wire-time arithmetic in
/// [`crate::bus::BusTiming`] assumes at ten bits per byte.
#[derive(Debug)]
pub struct SerialBusPort {
    port: TTYPort,
}

impl SerialBusPort {
    /// Opens `path` at `baud`.
    ///
    /// The initial timeout is nominal: every read replaces it with the time
    /// left on that transaction's own deadline.
    pub fn open(path: &str, baud: u32) -> serialport::Result<Self> {
        let port = serialport::new(path, baud)
            .data_bits(DataBits::Eight)
            .parity(Parity::None)
            .stop_bits(StopBits::One)
            .flow_control(FlowControl::None)
            .timeout(std::time::Duration::from_millis(10))
            .open_native()?;
        Ok(Self { port })
    }
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
        Write::write_all(&mut self.port, buf)?;
        // The frame has to be on the wire before the reply can be waited for;
        // a buffered write would put the transaction's deadline in front of
        // bytes that have not left yet.
        self.port.flush()
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
