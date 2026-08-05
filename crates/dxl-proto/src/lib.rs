//! `dxl-proto` — Dynamixel Protocol 2.0 on the wire, with no wire attached.
//!
//! Frame encoding, incremental status-packet decoding, the X-series control
//! table, and the count/radian/volt/amp conversions. Pure functions and plain
//! structs: no serial port, no clock, no allocation on the decode path beyond
//! the caller's buffers. The crate that owns the port is `reachy-bus`, and it is
//! the only consumer that needs one.
//!
//! Three properties this crate exists to guarantee, because the servos hold up a
//! head and the code above cannot compensate for a codec that lies:
//!
//! - **Arbitrary bytes never panic.** A corrupted or truncated frame is a value,
//!   never an assertion failure. The decoder is fed bytes as they arrive, scans
//!   for a header, and resynchronizes after garbage.
//! - **A corrupt frame is a distinct outcome from a timeout.** They demand
//!   opposite policies — a timeout may be retried, a corrupt frame never is — so
//!   they are separate variants rather than one I/O error.
//! - **The servo's own error field is surfaced, not swallowed.** A reply that
//!   arrives with an error bit set is the signal that something is wrong with
//!   that servo, and it is exactly the signal a codec is tempted to drop because
//!   the payload parsed fine.
//!
//! Responses are attributed by the ID field in the packet, never by arrival
//! order, so one silent servo cannot misalign the readings of the others.

#![forbid(unsafe_code)]

pub mod conv;
pub mod crc;
pub mod decode;
pub mod encode;
pub mod frame;
pub mod regs;

pub use conv::{
    ConvError, HardwareError, counts_to_rad, milliamps_from_raw, rad_to_counts, volts_from_raw,
};
pub use crc::{crc_matches, crc16};
pub use decode::{DecodeStep, FrameError, StatusCode, StatusDecoder, StatusError, StatusView};
pub use encode::{
    EncodeError, encode_ping, encode_read, encode_reboot, encode_sync_read, encode_sync_write,
    encode_write,
};
pub use frame::{BROADCAST_ID, MAX_FRAME_BUF, MAX_INSTR_FRAME, MAX_STATUS_FRAME};
pub use regs::{Area, Reg};
