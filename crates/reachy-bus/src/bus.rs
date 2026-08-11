//! Unicast transactions: one request out, one attributed reply back.
//!
//! Every exchange runs the same core — drop any residue from an abandoned
//! exchange, encode, write, then feed arriving bytes to the decoder until one
//! of four things happens: the addressed servo's reply decodes, a frame from
//! some *other* servo turns up (counted and skipped, never misattributed), the
//! decoder rejects a candidate (immediate failure, no retry), or the deadline
//! passes with nothing (a timeout, which the retry wrapper may repeat).
//!
//! The deadline is computed per transaction from the bytes actually on the
//! wire at the configured baud plus a fixed host allowance, so a wide read is
//! given more time than a ping without anyone tuning a constant per operation.
//!
//! The grouped operations are the tick's own traffic and follow different
//! rules. A grouped read asks every servo in one frame and collects a verdict
//! per servo: a refusal, a short answer or silence lands in that servo's slot
//! and nowhere else, and a damaged frame — which carries no ID worth believing
//! — is counted against the read rather than blamed on anyone. A grouped write
//! is acknowledged by nothing at all, so there is nothing to read back and
//! nothing to verify; it exists because streaming nine goals as nine verified
//! writes would not fit in a control period.
//!
//! Non-volatile registers are refused on every write path but one. The
//! exception reads the servo's Torque Enable first and refuses unless it is
//! off, because a servo holding torque ignores such a write and acknowledges it
//! anyway.

use std::cmp::Ordering;
use std::time::{Duration, Instant};

use dxl_proto::frame::{CRC_LEN, MAX_STATUS_PARAMS, PREAMBLE_LEN};
use dxl_proto::regs::TORQUE_ENABLE;
use dxl_proto::{
    BROADCAST_ID, DecodeStep, MAX_FRAME_BUF, MAX_INSTR_FRAME, Reg, StatusDecoder, StatusError,
    encode_ping, encode_read, encode_reboot, encode_sync_read, encode_sync_write, encode_write,
};

use crate::error::{IdOutcome, SyncReadOutcome, XactError};
use crate::port::{BusPort, DEFAULT_BAUD};

/// Bytes of a status frame that are not parameters: header, ID, length field,
/// instruction, error field and CRC.
const STATUS_OVERHEAD: usize = PREAMBLE_LEN + 1 + 1 + CRC_LEN;

/// Parameters in a ping reply: model number (2) and firmware version (1).
const PING_PARAMS: usize = 3;

/// Bits on the wire per byte at 8N1: a start bit, eight data bits, a stop bit.
const BITS_PER_BYTE: u64 = 10;

/// Servos one grouped request may name. Nine is the machine: body yaw, six
/// legs, two antennas.
pub const MAX_SYNC_IDS: usize = 9;

/// Deadline and retry policy.
///
/// Every figure here is provisional and says so. The host allowance is the
/// shape of the open budget for a request-response exchange on this kind of
/// bus, not a measurement of this host; the retry count and spacing are round
/// numbers pending a bench run that shows what the loss actually looks like.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BusTiming {
    /// Fixed slack over the wire time: the servo's own turnaround plus the
    /// host's scheduling jitter.
    pub host_allowance: Duration,
    /// Wire rate, in bits per second.
    pub baud: u32,
    /// Attempts a retried operation makes in total, the first one included.
    pub retry_attempts: u32,
    /// Pause between attempts.
    pub retry_spacing: Duration,
}

impl Default for BusTiming {
    fn default() -> Self {
        Self {
            host_allowance: Duration::from_millis(10),
            baud: DEFAULT_BAUD,
            retry_attempts: 5,
            retry_spacing: Duration::from_millis(20),
        }
    }
}

impl BusTiming {
    /// Time `bytes` take to cross the wire at this baud.
    #[must_use]
    pub fn wire_time(&self, bytes: usize) -> Duration {
        // A rate of zero is not a rate; the floor keeps the division defined
        // rather than bounding anything commanded.
        let baud = u64::from(self.baud).max(1);
        let nanos = bytes as u64 * BITS_PER_BYTE * 1_000_000_000 / baud;
        Duration::from_nanos(nanos)
    }

    /// When an exchange sending `tx` bytes and expecting `rx` back gives up.
    #[must_use]
    pub fn deadline(&self, from: Instant, tx: usize, rx: usize) -> Instant {
        from + self.host_allowance + self.wire_time(tx + rx)
    }
}

/// Things that happened on the bus that no single transaction failed on.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct BusCounters {
    /// Well-formed replies carrying an ID nobody had asked for — a late answer
    /// to a previous exchange. Skipped, never attributed.
    pub unexpected_id_frames: u64,
    /// Exchanges that ended at their deadline.
    pub timeouts: u64,
    /// Attempts the retry wrapper made beyond the first.
    pub retries: u64,
    /// Replies that carried the alert bit: the servo processed the instruction
    /// and has a hardware error latched. The bit is a standing condition rather
    /// than a verdict on the exchange, so it does not fail one; what it points
    /// at is the Hardware Error Status register.
    pub alerts: u64,
}

/// A register value as bytes, in control-table order.
///
/// Wide enough for the position-gain span, which is the widest entry in the
/// table at six bytes. Carries the bytes without interpretation.
///
/// The default is the zero-width value, which no register produces: it is what
/// a caller assembling a batch seeds an array with before filling it.
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub struct RawValue {
    bytes: [u8; Self::MAX_LEN],
    len: u8,
}

impl RawValue {
    /// Widest register this carries.
    pub const MAX_LEN: usize = 6;

    /// A value from `bytes`, or `None` if it is wider than [`Self::MAX_LEN`].
    #[must_use]
    pub fn new(bytes: &[u8]) -> Option<Self> {
        if bytes.len() > Self::MAX_LEN {
            return None;
        }
        let mut value = Self {
            bytes: [0; Self::MAX_LEN],
            len: bytes.len() as u8,
        };
        value.bytes[..bytes.len()].copy_from_slice(bytes);
        Some(value)
    }

    /// The bytes, in the order they sit in the control table.
    #[must_use]
    pub fn as_slice(&self) -> &[u8] {
        &self.bytes[..usize::from(self.len)]
    }

    /// Width in bytes.
    #[must_use]
    pub fn len(&self) -> usize {
        usize::from(self.len)
    }

    /// True for a zero-width value, which only a zero-width register produces.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    /// The value as an unsigned byte, if it is one byte wide.
    #[must_use]
    pub fn u8(&self) -> Option<u8> {
        match self.as_slice() {
            [b] => Some(*b),
            _ => None,
        }
    }

    /// The value as a little-endian `u16`, if it is two bytes wide.
    #[must_use]
    pub fn u16(&self) -> Option<u16> {
        match self.as_slice() {
            [a, b] => Some(u16::from_le_bytes([*a, *b])),
            _ => None,
        }
    }

    /// The value as a little-endian `u32`, if it is four bytes wide.
    #[must_use]
    pub fn u32(&self) -> Option<u32> {
        match self.as_slice() {
            [a, b, c, d] => Some(u32::from_le_bytes([*a, *b, *c, *d])),
            _ => None,
        }
    }

    /// The value as a little-endian `i32`, if it is four bytes wide. Position
    /// registers are signed and multi-turn, so this is not the same reading as
    /// [`Self::u32`].
    #[must_use]
    pub fn i32(&self) -> Option<i32> {
        self.u32().map(|raw| raw as i32)
    }
}

impl core::fmt::Debug for RawValue {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "RawValue({self})")
    }
}

impl core::fmt::Display for RawValue {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for (n, byte) in self.as_slice().iter().enumerate() {
            if n > 0 {
                f.write_str(" ")?;
            }
            write!(f, "{byte:02x}")?;
        }
        Ok(())
    }
}

/// What a ping reply carries.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PingInfo {
    /// Model number. The three servo groups on this machine answer with three
    /// distinct values.
    pub model: u16,
    /// Firmware version.
    pub firmware: u8,
}

/// One accepted reply, copied out of the decoder's buffer.
struct Reply {
    error: StatusError,
    /// Parameter bytes the frame actually carried, which is what the caller
    /// checks its request against. It can exceed what was copied out; a reply
    /// that wide is never this request's answer, so it never becomes a value.
    len: usize,
}

/// The transaction layer over a port.
#[derive(Debug)]
pub struct Bus<P: BusPort> {
    port: P,
    timing: BusTiming,
    decoder: StatusDecoder,
    tx: [u8; MAX_INSTR_FRAME],
    counters: BusCounters,
}

impl<P: BusPort> Bus<P> {
    /// A bus over `port`.
    pub fn new(port: P, timing: BusTiming) -> Self {
        Self {
            port,
            timing,
            decoder: StatusDecoder::new(),
            tx: [0; MAX_INSTR_FRAME],
            counters: BusCounters::default(),
        }
    }

    /// The deadline and retry policy in force.
    #[must_use]
    pub fn timing(&self) -> &BusTiming {
        &self.timing
    }

    /// Anomalies counted since the bus was opened.
    #[must_use]
    pub fn counters(&self) -> BusCounters {
        self.counters
    }

    /// Pings `id`.
    pub fn ping(&mut self, id: u8) -> Result<PingInfo, XactError> {
        let tx_len =
            encode_ping(id, &mut self.tx).map_err(|source| XactError::Encode { id, source })?;
        let mut params = [0u8; MAX_STATUS_PARAMS];
        let reply = self.exchange(id, tx_len, PING_PARAMS, &mut params)?;
        self.check_status(id, reply.error)?;
        check_length(id, PING_PARAMS, reply.len)?;
        Ok(PingInfo {
            model: u16::from_le_bytes([params[0], params[1]]),
            firmware: params[2],
        })
    }

    /// Reboots `id`, and takes the acknowledgement it answers with before it
    /// goes.
    ///
    /// A restart clears Torque Enable, so a servo that was holding stops
    /// holding. Nothing in this crate decides to send one: the caller owns that
    /// decision and whatever the machine is resting on.
    ///
    /// The acknowledgement says the instruction was taken, not that the servo is
    /// back. It answers nothing while it restarts, so a caller that needs to
    /// know it has returned pings it until it does.
    pub fn reboot(&mut self, id: u8) -> Result<(), XactError> {
        let tx_len =
            encode_reboot(id, &mut self.tx).map_err(|source| XactError::Encode { id, source })?;
        let mut params = [0u8; MAX_STATUS_PARAMS];
        let reply = self.exchange(id, tx_len, 0, &mut params)?;
        self.check_status(id, reply.error)?;
        // A reboot is acknowledged with no parameters, as a write is. Anything
        // else answers a different question, and taking it would report an
        // instruction as accepted that no servo has answered for.
        check_length(id, 0, reply.len)
    }

    /// Reads `reg` from `id`.
    pub fn read_reg(&mut self, id: u8, reg: Reg) -> Result<RawValue, XactError> {
        let width = usize::from(reg.len);
        if width > RawValue::MAX_LEN {
            return Err(XactError::RegisterTooWide {
                id,
                addr: reg.addr,
                len: width,
                max: RawValue::MAX_LEN,
            });
        }
        let tx_len = encode_read(id, reg.addr, u16::from(reg.len), &mut self.tx)
            .map_err(|source| XactError::Encode { id, source })?;
        let mut params = [0u8; MAX_STATUS_PARAMS];
        let reply = self.exchange(id, tx_len, width, &mut params)?;
        self.check_status(id, reply.error)?;
        check_length(id, width, reply.len)?;
        // The reply is exactly the register's width, and the width was bounded
        // above, so this only fails if the two bounds have drifted apart.
        RawValue::new(&params[..width]).ok_or(XactError::RegisterTooWide {
            id,
            addr: reg.addr,
            len: width,
            max: RawValue::MAX_LEN,
        })
    }

    /// Writes `value` to `reg` on `id`, then reads it back and compares.
    ///
    /// Refuses non-volatile registers outright: [`Self::write_eeprom_verified`]
    /// is the only path that writes one, and it establishes torque-off first.
    pub fn write_reg_verified(
        &mut self,
        id: u8,
        reg: Reg,
        value: &RawValue,
    ) -> Result<(), XactError> {
        // TODO(provisioning-repair): the guarded path below writes the one
        // non-volatile register this project provisions itself. Repairing a
        // vendor-provisioned register — a homing offset, a travel limit — is
        // still refused here and still undecided.
        if reg.is_eeprom() {
            return Err(XactError::EepromRefused { id, addr: reg.addr });
        }
        self.write_and_verify(id, reg, value)
    }

    /// Writes `value` to a non-volatile `reg` on `id`, having established that
    /// the servo is not holding torque, then reads it back and compares.
    ///
    /// Both halves are load-bearing. A servo accepts a non-volatile write only
    /// while its torque is off and acknowledges one it ignored, so torque is
    /// read here rather than assumed, and the read-back is what says the write
    /// took. Either half alone reports a no-op as a success.
    ///
    /// The window between the read and the write is the host's own: nothing
    /// else on this bus enables torque, and a servo that gained it in between
    /// fails the read-back rather than passing quietly.
    pub fn write_eeprom_verified(
        &mut self,
        id: u8,
        reg: Reg,
        value: &RawValue,
    ) -> Result<(), XactError> {
        let torque = self.read_reg(id, TORQUE_ENABLE)?;
        // A reply of another width cannot happen — the read checks its own — and
        // if it did, "not demonstrably off" is the only safe reading of it.
        if torque.u8().is_none_or(|held| held != 0) {
            return Err(XactError::TorqueHeld { id, addr: reg.addr });
        }
        self.write_and_verify(id, reg, value)
    }

    /// The write itself: put it on the wire, take the acknowledgement, read the
    /// register back and compare it count-exact.
    fn write_and_verify(&mut self, id: u8, reg: Reg, value: &RawValue) -> Result<(), XactError> {
        if value.len() != usize::from(reg.len) {
            return Err(XactError::ValueWidth {
                id,
                addr: reg.addr,
                expected: usize::from(reg.len),
                actual: value.len(),
            });
        }
        let tx_len = encode_write(id, reg.addr, value.as_slice(), &mut self.tx)
            .map_err(|source| XactError::Encode { id, source })?;
        let mut params = [0u8; MAX_STATUS_PARAMS];
        let reply = self.exchange(id, tx_len, 0, &mut params)?;
        self.check_status(id, reply.error)?;
        // A write is acknowledged with no parameters at all. Anything else is
        // some other exchange's reply, and accepting it would count a write as
        // acknowledged that no servo has answered for.
        check_length(id, 0, reply.len)?;

        let read_back = self.read_reg(id, reg)?;
        if read_back != *value {
            return Err(XactError::VerifyMismatch {
                id,
                addr: reg.addr,
                wrote: *value,
                read_back,
            });
        }
        Ok(())
    }

    /// Reads `reg` from every servo in `ids` with one broadcast request.
    ///
    /// Fills `out` with one verdict per servo. Nothing a servo does fails the
    /// call: a refusal, a short answer and silence are all recorded against
    /// that servo alone, and a frame too damaged to attribute is counted
    /// against the read. The call itself fails only when the request could not
    /// be built or the port did.
    pub fn sync_read(
        &mut self,
        ids: &[u8],
        reg: Reg,
        out: &mut SyncReadOutcome,
    ) -> Result<(), XactError> {
        let width = usize::from(reg.len);
        if width > RawValue::MAX_LEN {
            return Err(XactError::RegisterTooWide {
                id: BROADCAST_ID,
                addr: reg.addr,
                len: width,
                max: RawValue::MAX_LEN,
            });
        }
        if ids.len() > MAX_SYNC_IDS {
            return Err(XactError::TooManyIds {
                count: ids.len(),
                max: MAX_SYNC_IDS,
            });
        }
        let tx_len = encode_sync_read(ids, reg.addr, u16::from(reg.len), &mut self.tx).map_err(
            |source| XactError::Encode {
                id: BROADCAST_ID,
                source,
            },
        )?;
        out.begin(ids);
        self.send(BROADCAST_ID, tx_len)?;

        let Self {
            port,
            timing,
            decoder,
            counters,
            ..
        } = self;

        let expected_rx = out.len() * (STATUS_OVERHEAD + width);
        let deadline = timing.deadline(Instant::now(), tx_len, expected_rx);
        let mut chunk = [0u8; MAX_FRAME_BUF];
        let mut answered = 0;
        while answered < out.len() {
            let read = port
                .read_some(&mut chunk, deadline)
                .map_err(|source| XactError::Io {
                    id: BROADCAST_ID,
                    source,
                })?;
            if read == 0 {
                // At least one servo is still silent, or the loop would have
                // ended; the slots left waiting stay `Timeout`.
                counters.timeouts += 1;
                break;
            }
            let mut fed = 0;
            while fed < read {
                let (used, step) = decoder.feed(&chunk[fed..read]);
                fed += used;
                match step {
                    DecodeStep::NeedMore => {}
                    DecodeStep::Packet(view) => {
                        if view.error.alert() {
                            counters.alerts += 1;
                        }
                        let outcome = if view.error.code().is_some() {
                            IdOutcome::ServoError(view.error)
                        } else if view.params.len() < width {
                            IdOutcome::ShortReply {
                                expected: width,
                                actual: view.params.len(),
                            }
                        } else if view.params.len() > width {
                            // Every servo was asked for the same register, so a
                            // wider frame answers a different question and its
                            // head is not this register's value.
                            IdOutcome::LongReply {
                                expected: width,
                                actual: view.params.len(),
                            }
                        } else {
                            match RawValue::new(&view.params[..width]) {
                                Some(value) => IdOutcome::Ok(value),
                                // The width was refused at the top of this call
                                // if it exceeded what a value holds, so this
                                // arm needs the two bounds to have drifted
                                // apart. It fails the read for what it is: a
                                // register too wide to carry, not a servo that
                                // answered short.
                                None => {
                                    return Err(XactError::RegisterTooWide {
                                        id: BROADCAST_ID,
                                        addr: reg.addr,
                                        len: width,
                                        max: RawValue::MAX_LEN,
                                    });
                                }
                            }
                        };
                        if out.record(view.id, outcome) {
                            answered += 1;
                        } else {
                            // A servo nobody asked, or one that has already
                            // answered. Either way the reading in hand stands.
                            counters.unexpected_id_frames += 1;
                        }
                    }
                    // Damaged bytes carry no ID worth believing, so they are
                    // counted rather than blamed on whichever servo they might
                    // have come from — and reading continues, because eight
                    // good answers are worth more than one bad frame.
                    DecodeStep::Corrupt(_) => out.count_corrupt(),
                }
            }
        }
        Ok(())
    }

    /// Writes one register on many servos with a single broadcast frame.
    ///
    /// The protocol acknowledges nothing here, so nothing is read back and
    /// nothing can be verified. The caller must detect goals that did not take
    /// by other means. Refuses non-volatile registers, as every write on this
    /// bus does.
    pub fn sync_write(&mut self, reg: Reg, entries: &[(u8, RawValue)]) -> Result<(), XactError> {
        if reg.is_eeprom() {
            return Err(XactError::EepromRefused {
                id: BROADCAST_ID,
                addr: reg.addr,
            });
        }
        if entries.len() > MAX_SYNC_IDS {
            return Err(XactError::TooManyIds {
                count: entries.len(),
                max: MAX_SYNC_IDS,
            });
        }
        let width = usize::from(reg.len);
        for (id, value) in entries {
            if value.len() != width {
                return Err(XactError::ValueWidth {
                    id: *id,
                    addr: reg.addr,
                    expected: width,
                    actual: value.len(),
                });
            }
        }
        let mut payload: [(u8, &[u8]); MAX_SYNC_IDS] = [(0, &[]); MAX_SYNC_IDS];
        for (slot, (id, value)) in payload.iter_mut().zip(entries) {
            *slot = (*id, value.as_slice());
        }
        let tx_len = encode_sync_write(
            reg.addr,
            u16::from(reg.len),
            &payload[..entries.len()],
            &mut self.tx,
        )
        .map_err(|source| XactError::Encode {
            id: BROADCAST_ID,
            source,
        })?;
        self.port
            .write_all(&self.tx[..tx_len])
            .map_err(|source| XactError::Io {
                id: BROADCAST_ID,
                source,
            })
    }

    /// Clear the line, then put `tx[..tx_len]` on it.
    ///
    /// Every request starts here: residue from an abandoned exchange is dropped
    /// from the port and from the decoder's half-read frame before anything new
    /// goes out, so a stale reply can never be read as this request's answer.
    /// `id` names whoever the failure is reported against.
    fn send(&mut self, id: u8, tx_len: usize) -> Result<(), XactError> {
        self.port
            .discard_input()
            .map_err(|source| XactError::Io { id, source })?;
        self.decoder.reset();
        self.port
            .write_all(&self.tx[..tx_len])
            .map_err(|source| XactError::Io { id, source })
    }

    /// The common core: send `tx[..tx_len]`, wait for `id` to answer.
    fn exchange(
        &mut self,
        id: u8,
        tx_len: usize,
        expect_params: usize,
        params: &mut [u8; MAX_STATUS_PARAMS],
    ) -> Result<Reply, XactError> {
        self.send(id, tx_len)?;

        let Self {
            port,
            timing,
            decoder,
            counters,
            ..
        } = self;

        let started = Instant::now();
        let deadline = timing.deadline(started, tx_len, STATUS_OVERHEAD + expect_params);
        let mut chunk = [0u8; MAX_FRAME_BUF];
        loop {
            let read = port
                .read_some(&mut chunk, deadline)
                .map_err(|source| XactError::Io { id, source })?;
            if read == 0 {
                counters.timeouts += 1;
                return Err(XactError::Timeout {
                    id,
                    waited: started.elapsed(),
                });
            }
            let mut fed = 0;
            while fed < read {
                let (used, step) = decoder.feed(&chunk[fed..read]);
                fed += used;
                match step {
                    DecodeStep::NeedMore => {}
                    DecodeStep::Packet(view) => {
                        if view.id != id {
                            counters.unexpected_id_frames += 1;
                            continue;
                        }
                        // The decoder's frames can be wider than this buffer, so
                        // the copy is bounded by the buffer while the reported
                        // length is what arrived. Every caller compares that
                        // length against its own request and refuses a
                        // disagreement, so a truncated copy is never read.
                        let copied = view.params.len().min(MAX_STATUS_PARAMS);
                        params[..copied].copy_from_slice(&view.params[..copied]);
                        return Ok(Reply {
                            error: view.error,
                            len: view.params.len(),
                        });
                    }
                    // A well-formed frame that is not a status packet arrives
                    // here too, and fails the exchange with everything else
                    // this arm refuses. That is the right verdict on this path:
                    // it does not reflect the host's own transmission — four
                    // read-only runs over nine servos saw every exchange clean
                    // from the first ping — so a non-status frame here is a
                    // genuine anomaly and not traffic to be skipped past.
                    DecodeStep::Corrupt(cause) => return Err(XactError::Corrupt { id, cause }),
                }
            }
        }
    }

    /// Turns a reply's error field into a failure, or lets it through.
    ///
    /// The error *number* is the servo's verdict on the instruction just sent,
    /// and it fails the transaction with the byte intact. The alert bit is not
    /// a verdict on anything: it says a hardware error is latched, which is a
    /// standing condition that would otherwise fail every exchange with the
    /// one servo whose Hardware Error Status most needs reading. It is counted
    /// rather than acted on.
    fn check_status(&mut self, id: u8, error: StatusError) -> Result<(), XactError> {
        if error.alert() {
            self.counters.alerts += 1;
        }
        if error.code().is_some() {
            return Err(XactError::ServoError { id, error });
        }
        Ok(())
    }

    /// Counts an attempt beyond the first.
    fn record_retry(&mut self) {
        self.counters.retries += 1;
    }
}

/// Checks a reply's parameter count against the request that earned it.
///
/// A status packet's width is fixed by the instruction it answers — three bytes
/// for a ping, the register's width for a read, none at all for a write — so a
/// reply of any other width is an answer to a different question, whatever ID
/// it carries. Both directions are refused and each says which it was: short
/// means the servo answered a narrower request, long means a frame from a wider
/// exchange arrived late enough to survive the line being cleared.
fn check_length(id: u8, expected: usize, actual: usize) -> Result<(), XactError> {
    match actual.cmp(&expected) {
        Ordering::Less => Err(XactError::ShortReply {
            id,
            expected,
            actual,
        }),
        Ordering::Greater => Err(XactError::LongReply {
            id,
            expected,
            actual,
        }),
        Ordering::Equal => Ok(()),
    }
}

/// Runs `op` again on a timeout, up to the configured attempt budget.
///
/// A free function rather than a method so the rule is the shape of one match:
/// exactly one variant is repeated and every other failure returns on the spot.
/// A corrupt frame retried into an eventual success would be a wire fault
/// reported as a healthy exchange, and there is no way back from that.
pub fn with_retry<P, T, F>(bus: &mut Bus<P>, mut op: F) -> Result<T, XactError>
where
    P: BusPort,
    F: FnMut(&mut Bus<P>) -> Result<T, XactError>,
{
    let attempts = bus.timing.retry_attempts.max(1);
    let spacing = bus.timing.retry_spacing;
    let mut made = 1;
    loop {
        match op(bus) {
            Err(XactError::Timeout { .. }) if made < attempts => {
                made += 1;
                bus.record_retry();
                if !spacing.is_zero() {
                    std::thread::sleep(spacing);
                }
            }
            outcome => return outcome,
        }
    }
}

#[cfg(test)]
mod tests {
    use std::collections::VecDeque;
    use std::io;

    use dxl_proto::frame::{
        HEADER, IDX_ID, IDX_INSTRUCTION, INST_READ, INST_REBOOT, INST_STATUS, INST_SYNC_WRITE,
        INST_WRITE, MIN_STATUS_LEN,
    };
    use dxl_proto::regs::{
        GOAL_POSITION, HARDWARE_ERROR_STATUS, HOMING_OFFSET, OPERATING_MODE, PRESENT_POSITION,
    };
    use dxl_proto::{Area, FrameError, StatusCode, crc16};

    use super::*;

    /// A status frame as a servo would put it on the wire. No stuffing: no
    /// fixture here carries the header pattern in its parameters, and the
    /// stuffing round trip is `dxl-proto`'s own property test.
    fn status(id: u8, error: u8, params: &[u8]) -> Vec<u8> {
        let mut frame = HEADER.to_vec();
        frame.push(id);
        let len = (params.len() + MIN_STATUS_LEN) as u16;
        frame.extend_from_slice(&len.to_le_bytes());
        frame.push(INST_STATUS);
        frame.push(error);
        frame.extend_from_slice(params);
        let crc = crc16(&frame);
        frame.extend_from_slice(&crc.to_le_bytes());
        frame
    }

    /// A frame whose CRC does not match its contents.
    fn corrupted(id: u8) -> Vec<u8> {
        let mut frame = status(id, 0, &[1, 2, 3]);
        let last = frame.len() - 1;
        frame[last] ^= 0xFF;
        frame
    }

    /// A port whose replies are scripted: one entry per request written, each
    /// entry the whole byte stream that arrives before the next request goes
    /// out. An empty entry is a servo that says nothing.
    struct FakePort {
        replies: VecDeque<Vec<u8>>,
        pending: VecDeque<u8>,
        writes: Vec<Vec<u8>>,
        chunk: usize,
        discards: usize,
        fail_write: Option<io::ErrorKind>,
        fail_read: Option<io::ErrorKind>,
        fail_discard: Option<io::ErrorKind>,
    }

    impl FakePort {
        fn new(replies: &[Vec<u8>]) -> Self {
            Self {
                replies: replies.iter().cloned().collect(),
                pending: VecDeque::new(),
                writes: Vec::new(),
                chunk: usize::MAX,
                discards: 0,
                fail_write: None,
                fail_read: None,
                fail_discard: None,
            }
        }

        /// Hands back at most `n` bytes per read, so a reply arrives split the
        /// way a UART would deliver it.
        fn in_chunks(mut self, n: usize) -> Self {
            self.chunk = n;
            self
        }

        /// Bytes already waiting in the receive buffer before anything is sent
        /// — the tail of an exchange somebody else abandoned.
        fn with_residue(mut self, bytes: &[u8]) -> Self {
            self.pending.extend(bytes.iter().copied());
            self
        }

        /// Fails the next call to the named operation, the way a port whose
        /// adapter has been unplugged does.
        fn failing_write(mut self, kind: io::ErrorKind) -> Self {
            self.fail_write = Some(kind);
            self
        }

        fn failing_read(mut self, kind: io::ErrorKind) -> Self {
            self.fail_read = Some(kind);
            self
        }

        fn failing_discard(mut self, kind: io::ErrorKind) -> Self {
            self.fail_discard = Some(kind);
            self
        }
    }

    impl BusPort for FakePort {
        fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
            if let Some(kind) = self.fail_write.take() {
                return Err(io::Error::from(kind));
            }
            self.writes.push(buf.to_vec());
            if let Some(reply) = self.replies.pop_front() {
                self.pending.extend(reply);
            }
            Ok(())
        }

        fn read_some(&mut self, buf: &mut [u8], _deadline: Instant) -> io::Result<usize> {
            if let Some(kind) = self.fail_read.take() {
                return Err(io::Error::from(kind));
            }
            let n = self.pending.len().min(buf.len()).min(self.chunk);
            for slot in buf.iter_mut().take(n) {
                *slot = self.pending.pop_front().unwrap_or(0);
            }
            Ok(n)
        }

        fn discard_input(&mut self) -> io::Result<()> {
            if let Some(kind) = self.fail_discard.take() {
                return Err(io::Error::from(kind));
            }
            self.discards += 1;
            self.pending.clear();
            Ok(())
        }
    }

    /// Default timing with the retry pause removed, so a retry test costs no
    /// wall-clock time. Nothing else about the policy changes.
    fn quick() -> BusTiming {
        BusTiming {
            retry_spacing: Duration::ZERO,
            ..BusTiming::default()
        }
    }

    fn bus_over(replies: &[Vec<u8>]) -> Bus<FakePort> {
        Bus::new(FakePort::new(replies), quick())
    }

    #[test]
    fn a_ping_reports_the_model_and_firmware() {
        let mut bus = bus_over(&[status(11, 0, &[0x0A, 0x04, 0x2C])]);
        let info = bus.ping(11).expect("the servo answered");
        assert_eq!(
            info,
            PingInfo {
                model: 0x040A,
                firmware: 0x2C
            }
        );
        assert_eq!(bus.counters(), BusCounters::default());
    }

    /// A reboot goes out as the reboot instruction and comes back acknowledged
    /// with nothing in it.
    #[test]
    fn a_reboot_is_the_reboot_instruction_and_an_empty_acknowledgement() {
        let mut bus = bus_over(&[status(11, 0, &[])]);
        bus.reboot(11).expect("the servo took the instruction");
        let frame = &bus.port.writes[0];
        assert_eq!(frame[IDX_ID], 11);
        assert_eq!(frame[IDX_INSTRUCTION], INST_REBOOT);
    }

    /// A servo with a hardware error latched sets the alert bit on every reply,
    /// the reboot's acknowledgement included — and that servo is precisely the
    /// one a reboot is sent to. The bit is counted, not acted on.
    #[test]
    fn a_latched_alert_does_not_fail_the_reboot_that_clears_it() {
        let mut bus = bus_over(&[status(11, 0x80, &[])]);
        bus.reboot(11).expect("the alert bit is not a refusal");
        assert_eq!(bus.counters().alerts, 1);
    }

    /// Silence fails the reboot rather than reporting one that no servo
    /// answered for.
    #[test]
    fn a_silent_servo_fails_its_reboot() {
        let mut bus = bus_over(&[Vec::new()]);
        let failed = bus.reboot(11).expect_err("nothing answered");
        assert!(
            matches!(failed, XactError::Timeout { id: 11, .. }),
            "{failed}"
        );
    }

    /// A reply carrying parameters is some other exchange's, and a reboot that
    /// accepted it would say an instruction was taken on that evidence.
    #[test]
    fn a_wide_reply_is_not_a_reboot_acknowledgement() {
        let mut bus = bus_over(&[status(11, 0, &[0x0A, 0x04, 0x2C])]);
        let failed = bus.reboot(11).expect_err("that is a ping's reply");
        assert!(
            matches!(
                failed,
                XactError::LongReply {
                    id: 11,
                    expected: 0,
                    actual: 3
                }
            ),
            "{failed}"
        );
    }

    #[test]
    fn a_read_returns_the_registers_bytes_in_wire_order() {
        let counts: i32 = -1234;
        let mut bus = bus_over(&[status(13, 0, &counts.to_le_bytes())]);
        let value = bus
            .read_reg(13, PRESENT_POSITION)
            .expect("the servo answered");
        assert_eq!(value.len(), 4);
        assert_eq!(value.i32(), Some(counts));
        assert_eq!(value.u16(), None, "a four-byte value is not a u16");
    }

    #[test]
    fn a_reply_split_one_byte_at_a_time_is_assembled() {
        let port = FakePort::new(&[status(11, 0, &[0x0A, 0x04, 0x2C])]).in_chunks(1);
        let mut bus = Bus::new(port, quick());
        assert_eq!(bus.ping(11).expect("the servo answered").model, 0x040A);
    }

    #[test]
    fn a_timeout_is_retried_and_the_next_attempt_is_taken() {
        let mut bus = bus_over(&[Vec::new(), Vec::new(), status(11, 0, &[1, 0, 2])]);
        let info = with_retry(&mut bus, |bus| bus.ping(11)).expect("the third attempt answered");
        assert_eq!(info.model, 1);
        assert_eq!(bus.counters().retries, 2);
        assert_eq!(bus.counters().timeouts, 2);
        assert_eq!(bus.port.writes.len(), 3, "one request per attempt");
    }

    #[test]
    fn a_timeout_that_outlives_the_budget_is_reported_as_one() {
        let silence = vec![Vec::new(); 8];
        let mut bus = bus_over(&silence);
        let failure = with_retry(&mut bus, |bus| bus.ping(11)).expect_err("nobody answered");
        assert!(matches!(failure, XactError::Timeout { id: 11, .. }));
        assert!(failure.is_retryable());
        assert_eq!(
            bus.port.writes.len(),
            quick().retry_attempts as usize,
            "the budget bounds the attempts"
        );
    }

    #[test]
    fn a_corrupt_frame_fails_at_once_and_is_never_retried() {
        let mut bus = bus_over(&[corrupted(11), status(11, 0, &[1, 0, 2])]);
        let failure = with_retry(&mut bus, |bus| bus.ping(11)).expect_err("the frame was corrupt");
        assert!(matches!(
            failure,
            XactError::Corrupt {
                id: 11,
                cause: FrameError::BadCrc
            }
        ));
        assert!(!failure.is_retryable());
        assert_eq!(
            bus.port.writes.len(),
            1,
            "a second request would have found the good frame waiting"
        );
    }

    #[test]
    fn a_servo_error_reaches_the_caller_with_its_byte_intact() {
        let mut bus = bus_over(&[status(14, 0x04, &[])]);
        let failure = bus
            .read_reg(14, PRESENT_POSITION)
            .expect_err("the servo refused");
        let XactError::ServoError { id, error } = failure else {
            panic!("expected a servo error, got {failure:?}");
        };
        assert_eq!(id, 14);
        assert_eq!(error.0, 0x04);
        assert_eq!(
            error.code(),
            Some(StatusCode::DataRange),
            "the one code that means two things must survive this layer"
        );
    }

    #[test]
    fn the_alert_bit_is_counted_rather_than_failing_the_read_that_diagnoses_it() {
        let mut bus = bus_over(&[status(12, 0x80, &[0x20])]);
        let value = bus
            .read_reg(12, HARDWARE_ERROR_STATUS)
            .expect("an alerting servo still answers");
        assert_eq!(value.u8(), Some(0x20));
        assert_eq!(bus.counters().alerts, 1);
    }

    #[test]
    fn a_late_reply_from_another_servo_is_counted_and_skipped() {
        let mut stream = status(12, 0, &[9, 9, 9]);
        stream.extend(status(11, 0, &[0x0A, 0x04, 0x2C]));
        let mut bus = bus_over(&[stream]);
        let info = bus.ping(11).expect("the addressed servo answered");
        assert_eq!(info.model, 0x040A, "never attributed to the wrong servo");
        assert_eq!(bus.counters().unexpected_id_frames, 1);
    }

    #[test]
    fn a_reply_shorter_than_the_register_is_not_a_value() {
        let mut bus = bus_over(&[status(13, 0, &[0x01, 0x02])]);
        let failure = bus
            .read_reg(13, PRESENT_POSITION)
            .expect_err("two bytes are not a position");
        assert!(matches!(
            failure,
            XactError::ShortReply {
                id: 13,
                expected: 4,
                actual: 2
            }
        ));
    }

    #[test]
    fn a_register_wider_than_a_raw_value_is_refused_before_the_wire() {
        let too_wide = Reg {
            addr: 200,
            len: (RawValue::MAX_LEN + 1) as u8,
            area: Area::Ram,
        };
        let mut bus = bus_over(&[]);
        let failure = bus
            .read_reg(11, too_wide)
            .expect_err("nothing carries that");
        assert!(matches!(failure, XactError::RegisterTooWide { id: 11, .. }));
        assert!(bus.port.writes.is_empty());
    }

    #[test]
    fn a_write_is_read_back_and_compared() {
        let mut bus = bus_over(&[status(11, 0, &[]), status(11, 0, &[1])]);
        let value = RawValue::new(&[1]).expect("one byte");
        bus.write_reg_verified(11, TORQUE_ENABLE, &value)
            .expect("the read-back matched");
        assert_eq!(bus.port.writes.len(), 2, "a write and its read-back");
    }

    #[test]
    fn a_write_that_reads_back_different_fails_verification() {
        let mut bus = bus_over(&[status(11, 0, &[]), status(11, 0, &[0])]);
        let value = RawValue::new(&[1]).expect("one byte");
        let failure = bus
            .write_reg_verified(11, TORQUE_ENABLE, &value)
            .expect_err("the servo kept its old value");
        let XactError::VerifyMismatch {
            wrote, read_back, ..
        } = failure
        else {
            panic!("expected a verify mismatch, got {failure:?}");
        };
        assert_eq!(wrote.u8(), Some(1));
        assert_eq!(read_back.u8(), Some(0));
    }

    /// The guarded path reads torque before it writes, and a servo holding it
    /// gets nothing: a non-volatile write under torque is ignored by the servo
    /// and acknowledged anyway, so an unguarded "success" here is the worst
    /// available outcome.
    #[test]
    fn a_guarded_write_to_a_servo_holding_torque_writes_nothing() {
        let mut bus = bus_over(&[status(17, 0, &[1])]);
        let value = RawValue::new(&[4]).expect("one byte");
        let failure = bus
            .write_eeprom_verified(17, OPERATING_MODE, &value)
            .expect_err("the servo is holding torque");
        assert!(matches!(
            failure,
            XactError::TorqueHeld { id: 17, addr: 11 }
        ));
        // What the operator reads: which servo, which register, and that the
        // register takes the write once the torque is off.
        assert_eq!(
            failure.to_string(),
            "servo 17 is holding torque; register 11 takes a write only once it is released"
        );
        assert_eq!(
            bus.port.writes.len(),
            1,
            "the torque read went out and nothing else"
        );
        let frame = &bus.port.writes[0];
        assert_eq!(frame[IDX_INSTRUCTION], INST_READ);
        let params = &frame[IDX_INSTRUCTION + 1..frame.len() - CRC_LEN];
        assert_eq!(
            u16::from_le_bytes([params[0], params[1]]),
            TORQUE_ENABLE.addr
        );
    }

    /// With torque off the write goes out and is read back count-exact.
    #[test]
    fn a_guarded_write_with_torque_off_is_read_back_and_compared() {
        let mut bus = bus_over(&[status(17, 0, &[0]), status(17, 0, &[]), status(17, 0, &[4])]);
        let value = RawValue::new(&[4]).expect("one byte");
        bus.write_eeprom_verified(17, OPERATING_MODE, &value)
            .expect("the read-back matched");
        assert_eq!(
            bus.port.writes.len(),
            3,
            "a torque read, the write, and its read-back"
        );
        let frame = &bus.port.writes[1];
        assert_eq!(frame[IDX_INSTRUCTION], INST_WRITE);
        let params = &frame[IDX_INSTRUCTION + 1..frame.len() - CRC_LEN];
        assert_eq!(
            u16::from_le_bytes([params[0], params[1]]),
            OPERATING_MODE.addr
        );
        assert_eq!(params[2], 4);
    }

    /// A servo that acknowledged the write and kept its old value fails, with
    /// both values named. This is what an EEPROM write the firmware dropped
    /// looks like from the host, and it is the reason the read-back exists.
    #[test]
    fn a_guarded_write_the_servo_ignored_is_a_typed_failure() {
        let mut bus = bus_over(&[status(17, 0, &[0]), status(17, 0, &[]), status(17, 0, &[3])]);
        let value = RawValue::new(&[4]).expect("one byte");
        let failure = bus
            .write_eeprom_verified(17, OPERATING_MODE, &value)
            .expect_err("the servo kept mode 3");
        let XactError::VerifyMismatch {
            id,
            addr,
            wrote,
            read_back,
        } = failure
        else {
            panic!("expected a verify mismatch, got {failure:?}");
        };
        assert_eq!((id, addr), (17, OPERATING_MODE.addr));
        assert_eq!(wrote.u8(), Some(4));
        assert_eq!(read_back.u8(), Some(3));
    }

    /// The guarded path is the only one: everything else still refuses a
    /// non-volatile register with nothing on the wire.
    #[test]
    fn every_other_path_still_refuses_a_non_volatile_write() {
        let value = RawValue::new(&[4]).expect("one byte");

        let mut bus = bus_over(&[]);
        assert!(matches!(
            bus.write_reg_verified(17, OPERATING_MODE, &value),
            Err(XactError::EepromRefused { id: 17, addr: 11 })
        ));
        assert!(bus.port.writes.is_empty());

        let mut bus = bus_over(&[]);
        assert!(matches!(
            bus.sync_write(OPERATING_MODE, &[(17, value), (18, value)]),
            Err(XactError::EepromRefused { addr: 11, .. })
        ));
        assert!(bus.port.writes.is_empty());
    }

    #[test]
    fn a_write_to_a_non_volatile_register_never_reaches_the_wire() {
        let mut bus = bus_over(&[]);
        let value = RawValue::new(&1024i32.to_le_bytes()).expect("four bytes");
        let failure = bus
            .write_reg_verified(11, HOMING_OFFSET, &value)
            .expect_err("eeprom is refused");
        assert!(matches!(
            failure,
            XactError::EepromRefused { id: 11, addr: 20 }
        ));
        assert!(bus.port.writes.is_empty());
    }

    #[test]
    fn a_value_of_the_wrong_width_is_refused_before_the_wire() {
        let mut bus = bus_over(&[]);
        let value = RawValue::new(&[1, 0]).expect("two bytes");
        let failure = bus
            .write_reg_verified(11, TORQUE_ENABLE, &value)
            .expect_err("torque enable is one byte");
        assert!(matches!(
            failure,
            XactError::ValueWidth {
                id: 11,
                expected: 1,
                actual: 2,
                ..
            }
        ));
        assert!(bus.port.writes.is_empty());
    }

    /// The prologue every request runs: whatever is already on the line is
    /// dropped before the request goes out. Without it the tail of an abandoned
    /// exchange — the common case after a timeout, which is exactly when the
    /// retry wrapper re-sends — is read as this request's answer, and a stale
    /// hardware-error byte is a fault raised on a healthy machine.
    #[test]
    fn a_stale_reply_waiting_on_the_line_is_dropped_before_the_request_goes_out() {
        let stale = status(11, 0, &[0x20]);
        let port = FakePort::new(&[status(11, 0, &[0x00])]).with_residue(&stale);
        let mut bus = Bus::new(port, quick());
        let value = bus
            .read_reg(11, HARDWARE_ERROR_STATUS)
            .expect("the servo answered");
        assert_eq!(
            value.u8(),
            Some(0x00),
            "the answer is this exchange's, not the one the line was still holding"
        );
        assert_eq!(bus.port.discards, 1, "one request, one line clear");
    }

    /// The other half of the prologue: the decoder is reset too, so a frame
    /// left half-read at a deadline cannot be spliced onto the next reply.
    #[test]
    fn a_half_read_frame_does_not_survive_into_the_next_exchange() {
        let mut half = status(11, 0, &[1, 2, 3]);
        half.truncate(8);
        let mut bus = bus_over(&[half, status(11, 0, &[0x00])]);
        let value = with_retry(&mut bus, |bus| bus.read_reg(11, HARDWARE_ERROR_STATUS))
            .expect("the retry reads a whole frame, not a splice of two");
        assert_eq!(value.u8(), Some(0x00));
        assert_eq!(bus.port.writes.len(), 2, "the first attempt timed out");
        assert_eq!(bus.port.discards, 2);
    }

    /// A reply is exactly as wide as the request that earned it. A wider one is
    /// a frame from some other exchange wearing this servo's ID, and its head
    /// is not this register's value.
    #[test]
    fn a_reply_wider_than_the_request_is_not_this_exchanges_answer() {
        let mut bus = bus_over(&[status(11, 0, &(-1234i32).to_le_bytes())]);
        let failure = bus
            .read_reg(11, HARDWARE_ERROR_STATUS)
            .expect_err("four position bytes are not an error byte");
        assert!(matches!(
            failure,
            XactError::LongReply {
                id: 11,
                expected: 1,
                actual: 4
            }
        ));
        assert!(!failure.is_retryable());

        let mut bus = bus_over(&[status(11, 0, &[1, 2, 3, 4])]);
        assert!(matches!(
            bus.ping(11).expect_err("a ping reply is three bytes"),
            XactError::LongReply {
                id: 11,
                expected: 3,
                actual: 4
            }
        ));

        // A write is acknowledged with no parameters at all.
        let mut bus = bus_over(&[status(11, 0, &[1]), status(11, 0, &[1])]);
        let value = RawValue::new(&[1]).expect("one byte");
        assert!(matches!(
            bus.write_reg_verified(11, TORQUE_ENABLE, &value)
                .expect_err("that acknowledged nothing"),
            XactError::LongReply {
                id: 11,
                expected: 0,
                actual: 1
            }
        ));
        assert_eq!(
            bus.port.writes.len(),
            1,
            "an unacknowledged write is not read back as if it landed"
        );
    }

    /// The same rule inside a grouped read: the slot takes a typed verdict, not
    /// the head of a frame that answers a wider question.
    #[test]
    fn a_wider_frame_in_a_group_fills_no_slot_with_a_value() {
        let ids = [11, 12];
        let wire = stream(&[position(11, 100), status(12, 0, &[0x00])]);
        let mut bus = bus_over(&[wire]);
        let mut out = SyncReadOutcome::new();
        bus.sync_read(&ids, HARDWARE_ERROR_STATUS, &mut out)
            .expect("a wide frame never fails the call");
        assert_eq!(
            out.get(11),
            Some(IdOutcome::LongReply {
                expected: 1,
                actual: 4
            })
        );
        assert_eq!(
            out.get(12).and_then(|outcome| outcome.value()),
            RawValue::new(&[0x00]),
            "the servo that answered the question asked still reads"
        );
        assert_eq!(out.corrupt_frames(), 0, "both frames passed their own CRC");
        assert!(!out.all_ok());
    }

    /// A port that fails is the port failing, not a servo going quiet: `Io`
    /// carries who was addressed and is not retryable, so a dead adapter is
    /// diagnosed as a dead adapter rather than as nine silent servos.
    #[test]
    fn a_port_that_fails_is_reported_as_the_port_rather_than_the_servo() {
        let broken = io::ErrorKind::BrokenPipe;

        for port in [
            FakePort::new(&[]).failing_write(broken),
            FakePort::new(&[status(11, 0, &[1, 0, 2])]).failing_read(broken),
            FakePort::new(&[]).failing_discard(broken),
        ] {
            let mut bus = Bus::new(port, quick());
            let failure = bus.ping(11).expect_err("the port is gone");
            assert!(matches!(failure, XactError::Io { id: 11, .. }));
            assert_eq!(failure.id(), 11);
            assert!(!failure.is_retryable(), "a dead port is not a timeout");
        }

        let mut bus = Bus::new(FakePort::new(&[]).failing_write(broken), quick());
        let mut out = SyncReadOutcome::new();
        let failure = bus
            .sync_read(&[11, 12], PRESENT_POSITION, &mut out)
            .expect_err("the port is gone");
        assert!(matches!(failure, XactError::Io { .. }));
        assert_eq!(failure.id(), BROADCAST_ID, "nobody in particular was asked");
        assert!(!out.all_ok(), "a failed read reports no readings");
        assert_eq!(out.get(11), Some(IdOutcome::Timeout));

        let mut bus = Bus::new(FakePort::new(&[]).failing_write(broken), quick());
        let value = RawValue::new(&0i32.to_le_bytes()).expect("four bytes");
        let failure = bus
            .sync_write(GOAL_POSITION, &[(11, value)])
            .expect_err("the port is gone");
        assert!(matches!(failure, XactError::Io { .. }));
        assert_eq!(failure.id(), BROADCAST_ID);
    }

    #[test]
    fn the_deadline_is_the_frames_own_wire_time_plus_the_allowance() {
        let timing = BusTiming::default();
        // Ten bits per byte at 1 Mbaud is ten microseconds a byte.
        assert_eq!(timing.wire_time(100), Duration::from_millis(1));
        assert_eq!(timing.wire_time(0), Duration::ZERO);

        // A configured baud of zero is not a rate, and the floor keeps the
        // division defined rather than panicking inside the control loop. What
        // it degrades to is a deadline nothing reaches, which is the honest
        // reading of a wire that carries no bits per second.
        let stopped = BusTiming {
            baud: 0,
            ..BusTiming::default()
        };
        assert_eq!(
            stopped.wire_time(100),
            Duration::from_secs(1000),
            "a hundred bytes at one bit per second"
        );

        // A ping: a ten-byte request and a fourteen-byte reply.
        let from = Instant::now();
        assert_eq!(
            timing.deadline(from, 10, 14),
            from + timing.host_allowance + Duration::from_micros(240)
        );

        let slow = BusTiming {
            baud: 57_600,
            ..BusTiming::default()
        };
        assert!(
            slow.wire_time(100) > timing.wire_time(100),
            "a slower wire buys more time for the same frame"
        );
    }

    /// A four-byte position reply from `id`.
    fn position(id: u8, counts: i32) -> Vec<u8> {
        status(id, 0, &counts.to_le_bytes())
    }

    /// One stream out of several frames, as they would arrive back to back.
    fn stream(frames: &[Vec<u8>]) -> Vec<u8> {
        frames.concat()
    }

    #[test]
    fn a_grouped_read_reports_every_servo_separately() {
        // Five servos asked: one answers, one's frame arrives damaged, one
        // refuses, one answers after the damaged frame, one says nothing at
        // all. The damaged frame sits in the middle deliberately — the arm that
        // counts it has to keep reading, or one bad byte costs every servo
        // still to be heard from.
        let ids = [11, 12, 13, 14, 15];
        let wire = stream(&[
            position(11, 100),
            corrupted(13),
            status(12, 0x04, &[]),
            position(14, 400),
            // 15 is silent.
        ]);
        let mut bus = bus_over(&[wire]);
        let mut out = SyncReadOutcome::new();
        bus.sync_read(&ids, PRESENT_POSITION, &mut out)
            .expect("a bad responder never fails the call");

        assert_eq!(out.ids(), &ids);
        assert_eq!(
            out.get(11).and_then(|outcome| outcome.value()),
            RawValue::new(&100i32.to_le_bytes())
        );
        let Some(IdOutcome::ServoError(error)) = out.get(12) else {
            panic!("servo 12 refused, and that is not silence");
        };
        assert_eq!(error.code(), Some(StatusCode::DataRange));
        assert_eq!(
            out.get(13),
            Some(IdOutcome::Timeout),
            "a frame nobody can attribute leaves its servo unheard from"
        );
        assert_eq!(
            out.get(14).and_then(|outcome| outcome.value()),
            RawValue::new(&400i32.to_le_bytes()),
            "reading continued past the damaged frame"
        );
        assert_eq!(out.get(15), Some(IdOutcome::Timeout));
        assert_eq!(out.corrupt_frames(), 1);
        assert!(!out.all_ok());
        assert_eq!(out.get(17), None, "a servo the read never asked");
        assert_eq!(
            bus.counters().timeouts,
            1,
            "one incomplete read is one timeout, however many servos went unheard"
        );
    }

    #[test]
    fn every_servo_answering_is_the_reading_the_tick_needs() {
        let ids = [11, 12, 13];
        let wire = stream(&[position(11, 1), position(12, 2), position(13, 3)]);
        let mut bus = bus_over(&[wire]);
        let mut out = SyncReadOutcome::new();
        bus.sync_read(&ids, PRESENT_POSITION, &mut out)
            .expect("everyone answered");
        assert!(out.all_ok());
        assert_eq!(out.len(), 3);
        assert_eq!(out.corrupt_frames(), 0);
        for (index, counts) in [1i32, 2, 3].iter().enumerate() {
            let (id, outcome) = out.at(index).expect("three slots");
            assert_eq!(id, ids[index]);
            assert_eq!(outcome.value().and_then(|value| value.i32()), Some(*counts));
        }
        assert_eq!(out.at(3), None);
        assert_eq!(bus.counters().timeouts, 0, "nobody had to be waited out");
    }

    #[test]
    fn replies_out_of_order_are_attributed_by_their_id() {
        let ids = [11, 12, 13];
        let wire = stream(&[position(13, 300), position(11, 100), position(12, 200)]);
        let mut bus = bus_over(&[wire]);
        let mut out = SyncReadOutcome::new();
        bus.sync_read(&ids, PRESENT_POSITION, &mut out)
            .expect("everyone answered");
        for (id, counts) in [(11i32, 100i32), (12, 200), (13, 300)] {
            assert_eq!(
                out.get(id as u8)
                    .and_then(|outcome| outcome.value())
                    .and_then(|value| value.i32()),
                Some(counts),
                "servo {id} read by its own ID field, not by arrival order"
            );
        }
        assert_eq!(bus.counters().unexpected_id_frames, 0);
    }

    #[test]
    fn a_frame_from_a_servo_nobody_asked_is_counted_and_ignored() {
        let ids = [11, 12];
        let wire = stream(&[
            position(17, 999),
            position(11, 100),
            position(11, 111),
            position(12, 200),
        ]);
        let mut bus = bus_over(&[wire]);
        let mut out = SyncReadOutcome::new();
        bus.sync_read(&ids, PRESENT_POSITION, &mut out)
            .expect("the two asked both answered");
        assert_eq!(
            out.get(11)
                .and_then(|outcome| outcome.value())
                .and_then(|value| value.i32()),
            Some(100),
            "the first answer stands; a second is not an update"
        );
        assert_eq!(
            bus.counters().unexpected_id_frames,
            2,
            "one servo nobody asked, one answering twice"
        );
    }

    #[test]
    fn a_short_reply_in_a_group_is_neither_silence_nor_a_refusal() {
        let ids = [11];
        let mut bus = bus_over(&[status(11, 0, &[1, 2])]);
        let mut out = SyncReadOutcome::new();
        bus.sync_read(&ids, PRESENT_POSITION, &mut out)
            .expect("the servo answered, just not with a position");
        assert_eq!(
            out.get(11),
            Some(IdOutcome::ShortReply {
                expected: 4,
                actual: 2
            })
        );
        assert_eq!(out.corrupt_frames(), 0, "the frame passed its own CRC");
    }

    #[test]
    fn a_grouped_read_the_bus_cannot_carry_is_refused_before_the_wire() {
        let mut bus = bus_over(&[]);
        let mut out = SyncReadOutcome::new();
        let too_many: [u8; MAX_SYNC_IDS + 1] = [10, 11, 12, 13, 14, 15, 16, 17, 18, 19];
        assert!(matches!(
            bus.sync_read(&too_many, PRESENT_POSITION, &mut out),
            Err(XactError::TooManyIds {
                count: 10,
                max: MAX_SYNC_IDS
            })
        ));

        let too_wide = Reg {
            addr: 200,
            len: (RawValue::MAX_LEN + 1) as u8,
            area: Area::Ram,
        };
        assert!(matches!(
            bus.sync_read(&[11], too_wide, &mut out),
            Err(XactError::RegisterTooWide { .. })
        ));
        assert!(bus.port.writes.is_empty());
    }

    #[test]
    fn a_grouped_write_puts_one_frame_on_the_wire_and_waits_for_nothing() {
        let mut bus = bus_over(&[]);
        let entries: Vec<(u8, RawValue)> = (11u8..=13)
            .map(|id| {
                (
                    id,
                    RawValue::new(&(i32::from(id) * 10).to_le_bytes()).expect("four bytes"),
                )
            })
            .collect();
        bus.sync_write(GOAL_POSITION, &entries)
            .expect("nothing acknowledges this");
        assert_eq!(bus.port.writes.len(), 1, "one frame for all three servos");

        let frame = &bus.port.writes[0];
        assert_eq!(frame[IDX_ID], BROADCAST_ID);
        assert_eq!(frame[IDX_INSTRUCTION], INST_SYNC_WRITE);
        let params = &frame[IDX_INSTRUCTION + 1..frame.len() - CRC_LEN];
        assert_eq!(
            u16::from_le_bytes([params[0], params[1]]),
            GOAL_POSITION.addr
        );
        assert_eq!(u16::from_le_bytes([params[2], params[3]]), 4);
        for (index, (id, value)) in entries.iter().enumerate() {
            let at = 4 + index * 5;
            assert_eq!(params[at], *id);
            assert_eq!(&params[at + 1..at + 5], value.as_slice());
        }
    }

    #[test]
    fn a_grouped_write_to_a_non_volatile_register_never_reaches_the_wire() {
        let mut bus = bus_over(&[]);
        let value = RawValue::new(&1024i32.to_le_bytes()).expect("four bytes");
        assert!(matches!(
            bus.sync_write(HOMING_OFFSET, &[(11, value)]),
            Err(XactError::EepromRefused { addr: 20, .. })
        ));
        assert!(bus.port.writes.is_empty());
    }

    #[test]
    fn a_grouped_write_entry_of_the_wrong_width_is_refused_before_the_wire() {
        let mut bus = bus_over(&[]);
        let good = RawValue::new(&0i32.to_le_bytes()).expect("four bytes");
        let short = RawValue::new(&[0, 0]).expect("two bytes");
        assert!(matches!(
            bus.sync_write(GOAL_POSITION, &[(11, good), (12, short)]),
            Err(XactError::ValueWidth {
                id: 12,
                expected: 4,
                actual: 2,
                ..
            })
        ));
        assert!(
            bus.port.writes.is_empty(),
            "one bad entry stops the whole frame; a partial goal sweep is worse"
        );

        let too_many = vec![(11u8, good); MAX_SYNC_IDS + 1];
        assert!(matches!(
            bus.sync_write(GOAL_POSITION, &too_many),
            Err(XactError::TooManyIds { .. })
        ));
    }

    #[test]
    fn a_raw_value_takes_itself_apart_only_at_its_own_width() {
        let one = RawValue::new(&[0xAB]).expect("one byte");
        assert_eq!(one.u8(), Some(0xAB));
        assert_eq!(one.u16(), None);
        assert_eq!(format!("{one}"), "ab");

        let two = RawValue::new(&[0x34, 0x12]).expect("two bytes");
        assert_eq!(two.u16(), Some(0x1234));
        assert_eq!(two.u8(), None);

        let four = RawValue::new(&(-2i32).to_le_bytes()).expect("four bytes");
        assert_eq!(four.i32(), Some(-2));
        assert_eq!(four.u32(), Some(0xFFFF_FFFE));

        let six = RawValue::new(&[0; RawValue::MAX_LEN]).expect("the widest register");
        assert_eq!(six.len(), RawValue::MAX_LEN);
        assert_eq!(RawValue::new(&[0; RawValue::MAX_LEN + 1]), None);

        let empty = RawValue::new(&[]).expect("no bytes");
        assert!(empty.is_empty());
        assert_eq!(format!("{empty}"), "");
    }
}
