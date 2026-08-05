//! Protocol 2.0 frame layout: the header, the instruction bytes, the field
//! offsets shared by the encoder and the decoder, and the size caps every
//! buffer in this crate is dimensioned from.
//!
//! Every cap here is arithmetic on the largest payload this project actually
//! moves, written out so a future payload that outgrows one fails loudly at the
//! constant rather than quietly in a buffer.

/// Frame preamble. A data occurrence of this sequence is impossible: the byte
/// stuffing of [`crate::encode`] turns `FF FF FD` inside a payload into
/// `FF FF FD FD`, which is what makes the decoder's header scan sound.
pub const HEADER: [u8; 4] = [0xFF, 0xFF, 0xFD, 0x00];

/// The byte inserted after a payload `FF FF FD`, and removed again on decode.
pub const STUFF_BYTE: u8 = 0xFD;

/// Addresses every servo at once. Legal only for the sync instructions here.
pub const BROADCAST_ID: u8 = 0xFE;

/// Largest legal ID value; `0xFF` is reserved and never addresses anything.
pub const MAX_ID: u8 = 0xFE;

/// Instruction byte of a status (reply) packet.
pub const INST_STATUS: u8 = 0x55;

/// Instruction bytes used by this project. Bulk operations and factory reset
/// are deliberately absent: nothing here ever mass-erases a servo.
pub const INST_PING: u8 = 0x01;
/// See [`INST_PING`].
pub const INST_READ: u8 = 0x02;
/// See [`INST_PING`].
pub const INST_WRITE: u8 = 0x03;
/// See [`INST_PING`].
pub const INST_REBOOT: u8 = 0x08;
/// See [`INST_PING`].
pub const INST_SYNC_READ: u8 = 0x82;
/// See [`INST_PING`].
pub const INST_SYNC_WRITE: u8 = 0x83;

/// Offset of the packet ID byte.
pub const IDX_ID: usize = 4;
/// Offset of the little-endian 2-byte length field.
pub const IDX_LEN: usize = 5;
/// Offset of the instruction byte, and the first byte of the stuffing region.
pub const IDX_INSTRUCTION: usize = 7;
/// Offset of a status packet's error field.
pub const IDX_STATUS_ERROR: usize = 8;
/// Offset of a status packet's first parameter.
pub const IDX_STATUS_PARAMS: usize = 9;

/// Bytes preceding the length-counted region: header + ID + length field.
pub const PREAMBLE_LEN: usize = 7;

/// Trailing CRC width.
pub const CRC_LEN: usize = 2;

/// Length-field value of a status packet carrying no parameters: the
/// instruction byte, the error byte and the two CRC bytes.
pub const MIN_STATUS_LEN: usize = 4;

/// Largest status payload this project reads. The biggest real one is a 4-byte
/// register read; a ping reply carries 3. 32 leaves room for a future
/// multi-register read without re-dimensioning anything.
pub const MAX_STATUS_PARAMS: usize = 32;

/// Worst-case stuffing growth of a status packet's stuffing region. The region
/// is the error byte plus the parameters, 33 bytes, and stuffing inserts at
/// most one byte per three, so 11; 16 is that rounded up with slack.
pub const STATUS_STUFF_HEADROOM: usize = 16;

/// Largest length-field value a status packet may claim. Anything above it is a
/// corrupt length, not a large packet.
pub const MAX_STATUS_LEN: usize = MAX_STATUS_PARAMS + STATUS_STUFF_HEADROOM + 1 + 1 + CRC_LEN;

/// Largest whole status frame: `4 + 1 + 2 + 1 + 1 + 32 + 16 + 2 = 59`.
pub const MAX_STATUS_FRAME: usize = PREAMBLE_LEN + MAX_STATUS_LEN;

/// Decoder buffer size: the largest status frame, rounded up.
pub const MAX_FRAME_BUF: usize = 64;

/// Largest instruction payload this project sends, pre-stuffing. The worst case
/// is a nine-servo sync write of a 4-byte register: address 2 + data length 2 +
/// 9 × (1 ID + 4 data) = 49.
pub const MAX_INSTR_PARAMS: usize = 64;

/// Worst-case stuffing growth of an instruction packet's stuffing region: the
/// instruction byte plus the parameters, 65 bytes, one insert per three.
pub const INSTR_STUFF_HEADROOM: usize = (MAX_INSTR_PARAMS + 1) / 3;

/// Buffer size that can hold any frame this crate encodes:
/// `4 + 1 + 2 + 1 + 64 + 21 + 2 = 95`.
pub const MAX_INSTR_FRAME: usize =
    PREAMBLE_LEN + 1 + MAX_INSTR_PARAMS + INSTR_STUFF_HEADROOM + CRC_LEN;

/// Reads the little-endian length field out of a buffer holding at least
/// [`PREAMBLE_LEN`] bytes. Crate-internal: it panics on a shorter slice, and
/// the crate's public surface never panics on arbitrary bytes.
#[must_use]
pub(crate) fn length_field(buf: &[u8]) -> u16 {
    u16::from_le_bytes([buf[IDX_LEN], buf[IDX_LEN + 1]])
}

/// True when `a, b, c` is the `FF FF FD` sequence that byte stuffing breaks up.
///
/// The encoder's size calculation, the encoder's copy and the decoder's
/// destuffing all recognise the pattern through this one predicate, so they
/// cannot drift apart: a mis-sized frame and an out-of-bounds copy are the two
/// ways that divergence would show up.
#[must_use]
pub(crate) fn completes_stuff_pattern(a: u8, b: u8, c: u8) -> bool {
    a == 0xFF && b == 0xFF && c == STUFF_BYTE
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn status_caps_are_consistent() {
        assert_eq!(MAX_STATUS_LEN, 52);
        assert_eq!(MAX_STATUS_FRAME, 59);
        const { assert!(MAX_STATUS_FRAME <= MAX_FRAME_BUF) };
    }

    #[test]
    fn instruction_caps_cover_a_nine_servo_sync_write() {
        let nine_servo_sync_write = 2 + 2 + 9 * (1 + 4);
        assert!(nine_servo_sync_write <= MAX_INSTR_PARAMS);
        assert_eq!(INSTR_STUFF_HEADROOM, 21);
        assert_eq!(MAX_INSTR_FRAME, 95);
    }

    #[test]
    fn the_stuff_pattern_is_the_header_prefix() {
        assert!(completes_stuff_pattern(0xFF, 0xFF, STUFF_BYTE));
        assert_eq!(&HEADER[..3], &[0xFF, 0xFF, STUFF_BYTE]);
        assert!(!completes_stuff_pattern(0xFF, STUFF_BYTE, STUFF_BYTE));
        assert!(!completes_stuff_pattern(0xFF, 0xFF, 0x00));
        assert!(!completes_stuff_pattern(0x00, 0xFF, STUFF_BYTE));
    }

    #[test]
    fn length_field_is_little_endian() {
        let buf = [0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x34, 0x12];
        assert_eq!(length_field(&buf), 0x1234);
    }
}
