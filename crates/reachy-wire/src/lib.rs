//! `reachy-wire` — the motion bus datagram layout, with no bus attached.
//!
//! Four messages travel between the process that owns the servo bus and the
//! processes that decide what the machine does: a pose sample out of the
//! driver every cycle, a driver event at every boundary condition, a goal
//! setpoint in, and a control datagram in. This crate is the only statement of
//! how those four look as bytes.
//!
//! It exists as its own crate, with no dependencies at all, because the same
//! layout has to be spoken by three unrelated things: the driver process, a
//! Clockwork cog body carrying the bytes as a channel payload, and a Python
//! test checker reading them back out of a log. Anything this crate depends on
//! is something all three inherit.
//!
//! Properties the rest of the stack is written against:
//!
//! - **Packed little-endian, fields in declaration order, no padding.** The
//!   layout is stated by the encoders here and by the length constants beside
//!   them; nothing is derived from a Rust struct's in-memory shape, which is
//!   not a promise the language makes.
//! - **Arbitrary bytes never panic.** Every decode is total: a truncated,
//!   over-long, garbage or future-versioned datagram is a [`DecodeError`]
//!   value, which the receiver drops and counts.
//! - **A datagram is exactly as long as its type says.** Length is not a hint
//!   to be trusted from the sender; a message of the wrong size is refused
//!   rather than parsed out of a prefix.
//! - **No allocation on the codec path.** Encoding returns a fixed-size array,
//!   decoding borrows the caller's slice. (The golden-vector helpers in
//!   [`golden`] do allocate; they are test and tooling surface, not codec.)
//!
//! Unknown reserved bits and unknown enum values are refused, not ignored. A
//! sender that means to extend the format bumps the [`VERSION`] byte, and a
//! receiver that does not speak that version drops the datagram at the header
//! — which is a decision the receiver can count, unlike a silently discarded
//! flag bit.
//!
//! Joint order is the bus order shared with `reachy-motion`'s `JointId::ALL`:
//! body yaw, legs 0 through 5, right antenna, left antenna. Positions are
//! `f64` radians about the model datum. Joint masks are one bit per bus row,
//! matching that crate's `JointSet` convention.

#![forbid(unsafe_code)]

pub mod golden;
mod msgs;

pub use msgs::{
    CONTROL_LEN, Control, ControlOp, DRIVER_EVENT_LEN, DriverEvent, EventKind, GOAL_SETPOINT_LEN,
    GoalSetpoint, Message, POSE_SAMPLE_LEN, PoseSample, decode,
};

/// Header discriminator, `0x5257` — the ASCII of `RW` read as a number, so it
/// is legible in a hex dump. Encoded little-endian like every other field, it
/// puts the bytes `WR` at the front of every datagram.
pub const MAGIC: u16 = 0x5257;

/// The layout revision this crate speaks. A receiver drops anything else.
pub const VERSION: u8 = 1;

/// Bytes of header in front of every message body.
pub const HEADER_LEN: usize = 8;

/// Servo rows on the bus, and so the length of every position array here.
pub const JOINT_COUNT: usize = 9;

/// Every joint in a mask: the low [`JOINT_COUNT`] bits.
pub const JOINT_MASK_ALL: u16 = (1 << JOINT_COUNT) - 1;

/// The fixed part of every datagram.
///
/// `seq` is per-sender and monotonic, and it wraps: it exists so a receiver can
/// see a gap or a duplicate, not to order messages, which is what the time
/// fields in the bodies are for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Header {
    /// Which message follows. See the per-type `MSG_TYPE` constants.
    pub msg_type: u8,
    /// Per-sender counter, wrapping.
    pub seq: u32,
}

/// Why a datagram was refused.
///
/// Every variant is a thing a receiver counts and drops. None of them is
/// recoverable by reading further into the same datagram, which is why decoding
/// never returns a partial message.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DecodeError {
    /// Fewer bytes than a header.
    TooShort {
        /// What arrived.
        len: usize,
    },
    /// Leading two bytes are not [`MAGIC`].
    BadMagic {
        /// What arrived.
        magic: u16,
    },
    /// A layout revision this crate does not speak.
    BadVersion {
        /// What arrived.
        version: u8,
    },
    /// A message type this crate does not speak.
    UnknownType {
        /// What arrived.
        msg_type: u8,
    },
    /// A well-formed header for a different message than the caller asked for.
    WrongType {
        /// What the caller's decoder speaks.
        expected: u8,
        /// What the header said.
        actual: u8,
    },
    /// The datagram is not the exact length its type calls for.
    WrongLength {
        /// The type in the header.
        msg_type: u8,
        /// The only acceptable length for it.
        expected: usize,
        /// What arrived.
        actual: usize,
    },
    /// Bits set in a field that this layout revision leaves reserved.
    ReservedBits {
        /// Which field.
        field: &'static str,
        /// The offending bits, in place.
        bits: u16,
    },
    /// An enumerated field carrying a value this crate does not speak.
    UnknownEnum {
        /// Which field.
        field: &'static str,
        /// What arrived.
        value: u8,
    },
}

impl core::fmt::Display for DecodeError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::TooShort { len } => {
                write!(
                    f,
                    "datagram of {len} bytes is shorter than the {HEADER_LEN}-byte header"
                )
            }
            Self::BadMagic { magic } => write!(f, "magic {magic:#06x}, expected {MAGIC:#06x}"),
            Self::BadVersion { version } => {
                write!(f, "wire version {version}, this build speaks {VERSION}")
            }
            Self::UnknownType { msg_type } => write!(f, "unknown message type {msg_type:#04x}"),
            Self::WrongType { expected, actual } => {
                write!(
                    f,
                    "message type {actual:#04x}, decoder expects {expected:#04x}"
                )
            }
            Self::WrongLength {
                msg_type,
                expected,
                actual,
            } => write!(
                f,
                "message type {msg_type:#04x} is {expected} bytes, got {actual}"
            ),
            Self::ReservedBits { field, bits } => {
                write!(f, "reserved bits {bits:#06x} set in field `{field}`")
            }
            Self::UnknownEnum { field, value } => {
                write!(f, "field `{field}` carries unknown value {value}")
            }
        }
    }
}

impl core::error::Error for DecodeError {}

/// Read and validate the header without committing to a body type.
///
/// What a dispatching receiver calls first: it answers "is this ours, and what
/// is it" in one step, and everything it refuses is refused before any body
/// interpretation happens.
///
/// # Errors
///
/// [`DecodeError::TooShort`], [`DecodeError::BadMagic`] or
/// [`DecodeError::BadVersion`].
pub fn peek_header(bytes: &[u8]) -> Result<Header, DecodeError> {
    let mut r = Reader::new(bytes);
    let magic = r
        .u16()
        .map_err(|_| DecodeError::TooShort { len: bytes.len() })?;
    if magic != MAGIC {
        return Err(DecodeError::BadMagic { magic });
    }
    let version = r
        .u8()
        .map_err(|_| DecodeError::TooShort { len: bytes.len() })?;
    if version != VERSION {
        return Err(DecodeError::BadVersion { version });
    }
    let msg_type = r
        .u8()
        .map_err(|_| DecodeError::TooShort { len: bytes.len() })?;
    let seq = r
        .u32()
        .map_err(|_| DecodeError::TooShort { len: bytes.len() })?;
    Ok(Header { msg_type, seq })
}

/// Cursor over a datagram, total in the face of any input.
///
/// Bodies are decoded only after the datagram's exact length has been checked,
/// so no read here can actually run out; it still returns a `Result` rather
/// than indexing, because "cannot happen" is not a reason to leave a panic in
/// the path that parses bytes off a wire.
pub(crate) struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    pub(crate) fn new(buf: &'a [u8]) -> Self {
        Self { buf, pos: 0 }
    }

    pub(crate) fn body(buf: &'a [u8]) -> Self {
        Self {
            buf,
            pos: HEADER_LEN,
        }
    }

    fn take<const N: usize>(&mut self) -> Result<[u8; N], ()> {
        let slice = self.buf.get(self.pos..self.pos + N).ok_or(())?;
        let mut out = [0u8; N];
        out.copy_from_slice(slice);
        self.pos += N;
        Ok(out)
    }

    pub(crate) fn u8(&mut self) -> Result<u8, ()> {
        Ok(self.take::<1>()?[0])
    }

    pub(crate) fn u16(&mut self) -> Result<u16, ()> {
        Ok(u16::from_le_bytes(self.take::<2>()?))
    }

    pub(crate) fn u32(&mut self) -> Result<u32, ()> {
        Ok(u32::from_le_bytes(self.take::<4>()?))
    }

    pub(crate) fn i64(&mut self) -> Result<i64, ()> {
        Ok(i64::from_le_bytes(self.take::<8>()?))
    }

    pub(crate) fn f64(&mut self) -> Result<f64, ()> {
        Ok(f64::from_le_bytes(self.take::<8>()?))
    }

    pub(crate) fn joints(&mut self) -> Result<[f64; JOINT_COUNT], ()> {
        let mut out = [0.0; JOINT_COUNT];
        for slot in &mut out {
            *slot = self.f64()?;
        }
        Ok(out)
    }
}

/// Fixed-size datagram builder.
///
/// `N` is the message's total length constant, so the buffer is exactly the
/// datagram and a layout that does not add up trips the assertion in
/// [`Writer::finish`] in the crate's own tests rather than on a wire.
pub(crate) struct Writer<const N: usize> {
    buf: [u8; N],
    pos: usize,
}

impl<const N: usize> Writer<N> {
    pub(crate) fn new(msg_type: u8, seq: u32) -> Self {
        let mut w = Self {
            buf: [0u8; N],
            pos: 0,
        };
        w.put(&MAGIC.to_le_bytes());
        w.put(&[VERSION, msg_type]);
        w.put(&seq.to_le_bytes());
        w
    }

    fn put(&mut self, bytes: &[u8]) {
        let end = self.pos + bytes.len();
        self.buf[self.pos..end].copy_from_slice(bytes);
        self.pos = end;
    }

    pub(crate) fn u8(&mut self, v: u8) {
        self.put(&[v]);
    }

    pub(crate) fn u16(&mut self, v: u16) {
        self.put(&v.to_le_bytes());
    }

    pub(crate) fn u32(&mut self, v: u32) {
        self.put(&v.to_le_bytes());
    }

    pub(crate) fn i64(&mut self, v: i64) {
        self.put(&v.to_le_bytes());
    }

    pub(crate) fn f64(&mut self, v: f64) {
        self.put(&v.to_le_bytes());
    }

    pub(crate) fn joints(&mut self, v: &[f64; JOINT_COUNT]) {
        for angle in v {
            self.f64(*angle);
        }
    }

    pub(crate) fn finish(self) -> [u8; N] {
        debug_assert_eq!(
            self.pos, N,
            "encoder wrote a different length than its layout constant"
        );
        self.buf
    }
}

/// A mask reduced to the nine bus rows.
///
/// The encoders' half of the domain [`check_mask`] enforces on the way in. A
/// sender is not a trusted source of a mask either: `u16::MAX` written where
/// "every row" was meant produces a datagram every conforming receiver drops,
/// and on the sample stream that is the whole control loop losing its clock
/// with a drop counter as the only trace. Bits outside the rows are a caller
/// bug, so a debug build says so at the encoder; a release build sends the nine
/// rows the caller meant rather than a datagram nobody can read.
pub(crate) fn bus_mask(mask: u16) -> u16 {
    mask & JOINT_MASK_ALL
}

/// Refuse a mask with bits set outside the nine bus rows.
pub(crate) fn check_mask(field: &'static str, mask: u16) -> Result<u16, DecodeError> {
    let stray = mask & !JOINT_MASK_ALL;
    if stray != 0 {
        return Err(DecodeError::ReservedBits { field, bits: stray });
    }
    Ok(mask)
}

/// Validate the header of a datagram whose type and length are both known.
pub(crate) fn expect(bytes: &[u8], msg_type: u8, len: usize) -> Result<Header, DecodeError> {
    let header = peek_header(bytes)?;
    if header.msg_type != msg_type {
        return Err(DecodeError::WrongType {
            expected: msg_type,
            actual: header.msg_type,
        });
    }
    if bytes.len() != len {
        return Err(DecodeError::WrongLength {
            msg_type,
            expected: len,
            actual: bytes.len(),
        });
    }
    Ok(header)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn magic_is_ascii_rw_and_leads_with_w_on_the_wire() {
        assert_eq!(MAGIC.to_be_bytes(), *b"RW");
        assert_eq!(MAGIC.to_le_bytes(), *b"WR");
    }

    #[test]
    fn joint_mask_all_is_nine_bits() {
        assert_eq!(JOINT_MASK_ALL, 0x01ff);
        assert_eq!(JOINT_MASK_ALL.count_ones() as usize, JOINT_COUNT);
    }

    #[test]
    fn header_round_trips_through_a_writer() {
        let w = Writer::<HEADER_LEN>::new(0x2a, 0xdead_beef);
        let header = peek_header(&w.finish()).expect("well-formed header");
        assert_eq!(
            header,
            Header {
                msg_type: 0x2a,
                seq: 0xdead_beef
            }
        );
    }

    #[test]
    fn a_short_datagram_is_too_short_at_every_length() {
        let full = Writer::<HEADER_LEN>::new(0x01, 1).finish();
        for len in 0..HEADER_LEN {
            assert_eq!(
                peek_header(&full[..len]),
                Err(DecodeError::TooShort { len })
            );
        }
    }

    #[test]
    fn a_foreign_magic_is_refused_before_anything_else() {
        let mut bytes = Writer::<HEADER_LEN>::new(0x01, 1).finish();
        bytes[0] = 0x00;
        assert_eq!(
            peek_header(&bytes),
            Err(DecodeError::BadMagic { magic: 0x5200 })
        );
    }

    #[test]
    fn a_future_version_is_refused() {
        let mut bytes = Writer::<HEADER_LEN>::new(0x01, 1).finish();
        bytes[2] = VERSION + 1;
        assert_eq!(
            peek_header(&bytes),
            Err(DecodeError::BadVersion {
                version: VERSION + 1
            })
        );
    }

    #[test]
    fn errors_print_something_a_log_reader_can_use() {
        let text = DecodeError::BadVersion { version: 9 }.to_string();
        assert!(text.contains('9'), "{text}");
        let text = DecodeError::ReservedBits {
            field: "flags",
            bits: 0x08,
        }
        .to_string();
        assert!(text.contains("flags"), "{text}");
    }

    #[test]
    fn a_mask_is_reduced_to_the_bus_rows_on_the_way_out() {
        assert_eq!(bus_mask(JOINT_MASK_ALL), JOINT_MASK_ALL);
        assert_eq!(bus_mask(0), 0);
        assert_eq!(bus_mask(u16::MAX), JOINT_MASK_ALL);
        assert_eq!(bus_mask(0x0400 | 0b1_0101), 0b1_0101);
    }

    #[test]
    fn a_mask_outside_the_bus_rows_is_refused() {
        assert_eq!(check_mask("mask", JOINT_MASK_ALL), Ok(JOINT_MASK_ALL));
        assert_eq!(
            check_mask("mask", 0x0400),
            Err(DecodeError::ReservedBits {
                field: "mask",
                bits: 0x0400
            })
        );
    }

    /// The bus layout this crate states, checked against the crate that owns
    /// it.
    ///
    /// `reachy-motion` is a test-only dependency and stays one: nothing in the
    /// codec path may inherit it. It is here because the row count and the mask
    /// bit positions above are a restatement of `JointId`, and a restatement
    /// nothing compares is a copy waiting to disagree — a bus that grows a row
    /// would otherwise present as goals whose top rows are quietly never
    /// written.
    #[test]
    fn the_bus_rows_are_the_ones_reachy_motion_names() {
        use reachy_motion::JointId;

        assert_eq!(JOINT_COUNT, JointId::COUNT);
        let mask = JointId::ALL
            .iter()
            .map(|joint| 1u16 << joint.index().expect("every JointId::ALL row has an index"))
            .fold(0u16, |mask, bit| mask | bit);
        assert_eq!(JOINT_MASK_ALL, mask);
    }
}
