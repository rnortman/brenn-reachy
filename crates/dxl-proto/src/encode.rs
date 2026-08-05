//! Instruction-packet encoders.
//!
//! Every function writes one complete frame into a caller-owned buffer and
//! returns its length. Nothing allocates, nothing is truncated: arguments are
//! validated in full before the first byte is written, so a rejected call
//! leaves the buffer untouched and returns a named reason.

use thiserror::Error;

use crate::crc::crc16;
use crate::frame::{
    BROADCAST_ID, CRC_LEN, HEADER, IDX_ID, IDX_INSTRUCTION, IDX_LEN, INST_PING, INST_READ,
    INST_REBOOT, INST_SYNC_READ, INST_SYNC_WRITE, INST_WRITE, MAX_ID, MAX_INSTR_PARAMS,
    MAX_STATUS_PARAMS, PREAMBLE_LEN, STUFF_BYTE, completes_stuff_pattern,
};

/// Why a frame was not encoded.
#[derive(Debug, Clone, PartialEq, Eq, Error)]
pub enum EncodeError {
    /// `0xFF` addresses nothing.
    #[error("servo id {id:#04x} is not addressable (max {max:#04x})")]
    InvalidId { id: u8, max: u8 },
    /// The broadcast ID was given to an instruction that must be unicast.
    #[error("instruction requires a unicast id, got the broadcast id")]
    BroadcastNotAllowed,
    /// Payload larger than any frame this crate is dimensioned for.
    #[error("payload of {len} bytes exceeds the {max}-byte instruction limit")]
    ParamsTooLong { len: usize, max: usize },
    /// A sync-write entry's payload disagreed with the declared width. Silently
    /// truncating here would write a partial register value to a live servo.
    #[error("sync-write entry for id {id} carries {actual} bytes, expected {expected}")]
    EntryLengthMismatch {
        id: u8,
        expected: usize,
        actual: usize,
    },
    /// A sync operation was given no servos to talk to.
    #[error("sync instruction needs at least one entry")]
    NoEntries,
    /// More register bytes were requested than a reply to this request can
    /// carry. Encoding it anyway would put the failure on the decoder, which
    /// sees only an over-long length field and can report nothing better than
    /// a corrupt frame — blaming the wire for a caller's arithmetic.
    #[error("read of {len} bytes exceeds the {max}-byte reply this crate accepts")]
    ReadTooWide { len: u16, max: u16 },
    /// The output buffer cannot hold the encoded frame.
    #[error("buffer holds {given} bytes, frame needs {needed}")]
    BufferTooSmall { needed: usize, given: usize },
}

/// Encodes a Ping. The reply carries model number and firmware version.
pub fn encode_ping(id: u8, buf: &mut [u8]) -> Result<usize, EncodeError> {
    check_unicast(id)?;
    build(id, INST_PING, &[], buf)
}

/// Encodes a Read of `len` bytes at `addr`.
pub fn encode_read(id: u8, addr: u16, len: u16, buf: &mut [u8]) -> Result<usize, EncodeError> {
    check_unicast(id)?;
    check_read_width(len)?;
    let mut params = [0u8; 4];
    params[0..2].copy_from_slice(&addr.to_le_bytes());
    params[2..4].copy_from_slice(&len.to_le_bytes());
    build(id, INST_READ, &params, buf)
}

/// Encodes a Write of `data` at `addr`.
pub fn encode_write(id: u8, addr: u16, data: &[u8], buf: &mut [u8]) -> Result<usize, EncodeError> {
    check_unicast(id)?;
    let mut params = [0u8; MAX_INSTR_PARAMS];
    let total = 2 + data.len();
    if total > MAX_INSTR_PARAMS {
        return Err(EncodeError::ParamsTooLong {
            len: total,
            max: MAX_INSTR_PARAMS,
        });
    }
    params[0..2].copy_from_slice(&addr.to_le_bytes());
    params[2..total].copy_from_slice(data);
    build(id, INST_WRITE, &params[..total], buf)
}

/// Encodes a Reboot. A reboot clears Torque Enable, so on this machine it drops
/// whatever the servo was holding; nothing in this crate decides to send one.
pub fn encode_reboot(id: u8, buf: &mut [u8]) -> Result<usize, EncodeError> {
    check_unicast(id)?;
    build(id, INST_REBOOT, &[], buf)
}

/// Encodes a Sync Read: one broadcast request, one status reply per listed ID.
pub fn encode_sync_read(
    ids: &[u8],
    addr: u16,
    len: u16,
    buf: &mut [u8],
) -> Result<usize, EncodeError> {
    if ids.is_empty() {
        return Err(EncodeError::NoEntries);
    }
    for &id in ids {
        check_unicast(id)?;
    }
    check_read_width(len)?;
    let mut params = [0u8; MAX_INSTR_PARAMS];
    let total = 4 + ids.len();
    if total > MAX_INSTR_PARAMS {
        return Err(EncodeError::ParamsTooLong {
            len: total,
            max: MAX_INSTR_PARAMS,
        });
    }
    params[0..2].copy_from_slice(&addr.to_le_bytes());
    params[2..4].copy_from_slice(&len.to_le_bytes());
    params[4..total].copy_from_slice(ids);
    build(BROADCAST_ID, INST_SYNC_READ, &params[..total], buf)
}

/// Encodes a Sync Write: one broadcast frame carrying a per-servo payload, all
/// of width `data_len`. The protocol acknowledges nothing here, which is why an
/// entry whose payload does not match `data_len` is refused rather than padded.
pub fn encode_sync_write(
    addr: u16,
    data_len: u16,
    entries: &[(u8, &[u8])],
    buf: &mut [u8],
) -> Result<usize, EncodeError> {
    if entries.is_empty() {
        return Err(EncodeError::NoEntries);
    }
    let width = usize::from(data_len);
    for &(id, data) in entries {
        check_unicast(id)?;
        if data.len() != width {
            return Err(EncodeError::EntryLengthMismatch {
                id,
                expected: width,
                actual: data.len(),
            });
        }
    }
    let mut params = [0u8; MAX_INSTR_PARAMS];
    let total = 4 + entries.len() * (1 + width);
    if total > MAX_INSTR_PARAMS {
        return Err(EncodeError::ParamsTooLong {
            len: total,
            max: MAX_INSTR_PARAMS,
        });
    }
    params[0..2].copy_from_slice(&addr.to_le_bytes());
    params[2..4].copy_from_slice(&data_len.to_le_bytes());
    let mut at = 4;
    for &(id, data) in entries {
        params[at] = id;
        at += 1;
        params[at..at + width].copy_from_slice(data);
        at += width;
    }
    build(BROADCAST_ID, INST_SYNC_WRITE, &params[..total], buf)
}

/// Refuses a read whose reply the status decoder is not dimensioned to accept.
/// The bound is the parameter cap the receive buffer is sized from.
fn check_read_width(len: u16) -> Result<(), EncodeError> {
    let max = MAX_STATUS_PARAMS as u16;
    if len > max {
        return Err(EncodeError::ReadTooWide { len, max });
    }
    Ok(())
}

fn check_unicast(id: u8) -> Result<(), EncodeError> {
    if id > MAX_ID {
        return Err(EncodeError::InvalidId { id, max: MAX_ID });
    }
    if id == BROADCAST_ID {
        return Err(EncodeError::BroadcastNotAllowed);
    }
    Ok(())
}

/// Length the stuffing region grows to: one `FD` inserted after each `FF FF FD`.
fn stuffed_len(region: &[u8]) -> usize {
    let mut len = region.len();
    for i in 2..region.len() {
        if completes_stuff_pattern(region[i - 2], region[i - 1], region[i]) {
            len += 1;
        }
    }
    len
}

/// Writes `region` into `out` with stuffing applied; returns the bytes written.
/// `out` must be at least `stuffed_len(region)` long.
fn write_stuffed(region: &[u8], out: &mut [u8]) -> usize {
    let mut at = 0;
    for i in 0..region.len() {
        out[at] = region[i];
        at += 1;
        if i >= 2 && completes_stuff_pattern(region[i - 2], region[i - 1], region[i]) {
            out[at] = STUFF_BYTE;
            at += 1;
        }
    }
    at
}

/// Assembles a whole frame. The stuffing region is the instruction byte through
/// the last parameter; the length field counts that region after stuffing, plus
/// the CRC; the CRC covers every byte before it, stuffing included.
fn build(id: u8, instruction: u8, params: &[u8], buf: &mut [u8]) -> Result<usize, EncodeError> {
    if params.len() > MAX_INSTR_PARAMS {
        return Err(EncodeError::ParamsTooLong {
            len: params.len(),
            max: MAX_INSTR_PARAMS,
        });
    }

    let mut region = [0u8; MAX_INSTR_PARAMS + 1];
    region[0] = instruction;
    region[1..1 + params.len()].copy_from_slice(params);
    let region = &region[..1 + params.len()];

    let needed = PREAMBLE_LEN + stuffed_len(region) + CRC_LEN;
    if buf.len() < needed {
        return Err(EncodeError::BufferTooSmall {
            needed,
            given: buf.len(),
        });
    }

    buf[0..HEADER.len()].copy_from_slice(&HEADER);
    buf[IDX_ID] = id;
    let written = write_stuffed(region, &mut buf[IDX_INSTRUCTION..]);
    let len_field =
        u16::try_from(written + CRC_LEN).expect("frame length is capped well below u16");
    buf[IDX_LEN..IDX_LEN + 2].copy_from_slice(&len_field.to_le_bytes());

    let crc_at = PREAMBLE_LEN + written;
    let crc = crc16(&buf[..crc_at]);
    buf[crc_at..crc_at + CRC_LEN].copy_from_slice(&crc.to_le_bytes());
    Ok(crc_at + CRC_LEN)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::frame::{INSTR_STUFF_HEADROOM, MAX_INSTR_FRAME};

    // Byte-exact expectations below are adapted from the Apache-2.0 rustypot
    // crate's Protocol 2.0 codec tests (see NOTICE), which assert whole frames
    // including CRC for exactly the XL330 position path this project drives.

    #[test]
    fn ping_frame() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let n = encode_ping(2, &mut buf).unwrap();
        assert_eq!(
            &buf[..n],
            &[0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x03, 0x00, 0x01, 0x19, 0x72]
        );
    }

    #[test]
    fn reboot_frame() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let n = encode_reboot(2, &mut buf).unwrap();
        assert_eq!(
            &buf[..n],
            &[0xFF, 0xFF, 0xFD, 0x00, 0x02, 0x03, 0x00, 0x08, 0x2F, 0x72]
        );
    }

    #[test]
    fn read_frame() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let n = encode_read(1, 0x2B, 2, &mut buf).unwrap();
        assert_eq!(
            &buf[..n],
            &[
                0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x02, 0x2B, 0x00, 0x02, 0x00, 0x2E, 0xCD
            ]
        );
    }

    #[test]
    fn write_frame() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let n = encode_write(1, 116, &512u32.to_le_bytes(), &mut buf).unwrap();
        assert_eq!(
            &buf[..n],
            &[
                0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x09, 0x00, 0x03, 0x74, 0x00, 0x00, 0x02, 0x00, 0x00,
                0xCA, 0x89
            ]
        );
    }

    #[test]
    fn sync_read_frame() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let n = encode_sync_read(&[1, 2], 132, 4, &mut buf).unwrap();
        assert_eq!(
            &buf[..n],
            &[
                0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x09, 0x00, 0x82, 0x84, 0x00, 0x04, 0x00, 0x01, 0x02,
                0xCE, 0xFA
            ]
        );
    }

    #[test]
    fn sync_write_frame() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let entries: [(u8, &[u8]); 2] = [(1, &150u32.to_le_bytes()), (2, &170u32.to_le_bytes())];
        let n = encode_sync_write(116, 4, &entries, &mut buf).unwrap();
        assert_eq!(
            &buf[..n],
            &[
                0xFF, 0xFF, 0xFD, 0x00, 0xFE, 0x11, 0x00, 0x83, 0x74, 0x00, 0x04, 0x00, 0x01, 0x96,
                0x00, 0x00, 0x00, 0x02, 0xAA, 0x00, 0x00, 0x00, 0x82, 0x87
            ]
        );
    }

    #[test]
    fn sync_write_rejects_a_short_entry() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let entries: [(u8, &[u8]); 2] = [(1, &[0, 0, 0, 0]), (2, &[0, 0])];
        assert_eq!(
            encode_sync_write(116, 4, &entries, &mut buf),
            Err(EncodeError::EntryLengthMismatch {
                id: 2,
                expected: 4,
                actual: 2
            })
        );
    }

    #[test]
    fn sync_ops_reject_an_empty_list() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        assert_eq!(
            encode_sync_read(&[], 132, 4, &mut buf),
            Err(EncodeError::NoEntries)
        );
        assert_eq!(
            encode_sync_write(116, 4, &[], &mut buf),
            Err(EncodeError::NoEntries)
        );
    }

    #[test]
    fn unicast_ops_reject_the_broadcast_id() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        assert_eq!(
            encode_ping(BROADCAST_ID, &mut buf),
            Err(EncodeError::BroadcastNotAllowed)
        );
        assert_eq!(
            encode_read(BROADCAST_ID, 132, 4, &mut buf),
            Err(EncodeError::BroadcastNotAllowed)
        );
        assert_eq!(
            encode_write(BROADCAST_ID, 116, &[0], &mut buf),
            Err(EncodeError::BroadcastNotAllowed)
        );
        assert_eq!(
            encode_reboot(BROADCAST_ID, &mut buf),
            Err(EncodeError::BroadcastNotAllowed)
        );
    }

    #[test]
    fn reserved_id_is_refused() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        assert_eq!(
            encode_ping(0xFF, &mut buf),
            Err(EncodeError::InvalidId {
                id: 0xFF,
                max: MAX_ID
            })
        );
    }

    #[test]
    fn short_buffer_is_refused_without_writing() {
        let mut buf = [0xAAu8; 8];
        let err = encode_ping(1, &mut buf).unwrap_err();
        assert_eq!(
            err,
            EncodeError::BufferTooSmall {
                needed: 10,
                given: 8
            }
        );
        assert!(buf.iter().all(|&b| b == 0xAA));
    }

    #[test]
    fn oversized_write_payload_is_refused() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let data = [0u8; MAX_INSTR_PARAMS];
        assert_eq!(
            encode_write(1, 116, &data, &mut buf),
            Err(EncodeError::ParamsTooLong {
                len: MAX_INSTR_PARAMS + 2,
                max: MAX_INSTR_PARAMS
            })
        );
    }

    #[test]
    fn payload_header_pattern_is_stuffed() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let n = encode_write(1, 116, &[0xFF, 0xFF, 0xFD, 0x00], &mut buf).unwrap();
        // The payload's FF FF FD gains an FD, so no data run can imitate a header.
        assert_eq!(
            &buf[7..n - 2],
            &[0x03, 0x74, 0x00, 0xFF, 0xFF, 0xFD, 0xFD, 0x00]
        );
        // The length field counts the stuffed region plus the CRC.
        assert_eq!(crate::frame::length_field(&buf), 10);
        assert!(crate::crc::crc_matches(&buf[..n]));
    }

    #[test]
    fn reads_wider_than_a_reply_are_refused() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let max = MAX_STATUS_PARAMS as u16;
        assert!(encode_read(1, 132, max, &mut buf).is_ok());
        assert_eq!(
            encode_read(1, 132, max + 1, &mut buf),
            Err(EncodeError::ReadTooWide { len: max + 1, max })
        );
        assert!(encode_sync_read(&[1, 2], 132, max, &mut buf).is_ok());
        assert_eq!(
            encode_sync_read(&[1, 2], 132, 200, &mut buf),
            Err(EncodeError::ReadTooWide { len: 200, max })
        );
    }

    #[test]
    fn oversized_sync_write_is_refused() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let payload = [0u8; 4];
        let entries: Vec<(u8, &[u8])> = (1..=13u8).map(|id| (id, &payload[..])).collect();
        let total = 4 + 13 * (1 + 4);
        assert_eq!(
            encode_sync_write(116, 4, &entries, &mut buf),
            Err(EncodeError::ParamsTooLong {
                len: total,
                max: MAX_INSTR_PARAMS
            })
        );
    }

    #[test]
    fn oversized_sync_read_is_refused() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let ids: Vec<u8> = (1..=61u8).collect();
        assert_eq!(
            encode_sync_read(&ids, 132, 4, &mut buf),
            Err(EncodeError::ParamsTooLong {
                len: 4 + 61,
                max: MAX_INSTR_PARAMS
            })
        );
    }

    #[test]
    fn sync_ops_reject_the_broadcast_id_in_their_lists() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        assert_eq!(
            encode_sync_read(&[1, BROADCAST_ID], 132, 4, &mut buf),
            Err(EncodeError::BroadcastNotAllowed)
        );
        let payload = [0u8; 4];
        let entries: [(u8, &[u8]); 2] = [(1, &payload), (BROADCAST_ID, &payload)];
        assert_eq!(
            encode_sync_write(116, 4, &entries, &mut buf),
            Err(EncodeError::BroadcastNotAllowed)
        );
    }

    /// The largest payload this crate accepts, seeded so that every three bytes
    /// of it need a stuffing byte, fills the frame buffer to the byte. This is
    /// what the headroom constant claims and the only thing that demonstrates
    /// it: an off-by-one there turns a legal frame into a refused one.
    #[test]
    fn worst_case_stuffing_fills_the_frame_cap_exactly() {
        // Address 0xFFFF supplies the first FF FF, so the pattern repeats from
        // the first data byte onward with no break.
        let data: Vec<u8> = (0..MAX_INSTR_PARAMS - 2)
            .map(|i| if i % 3 == 0 { 0xFD } else { 0xFF })
            .collect();
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let n = encode_write(1, 0xFFFF, &data, &mut buf).expect("the cap payload must encode");
        assert_eq!(n, MAX_INSTR_FRAME);
        assert!(crate::crc::crc_matches(&buf[..n]));
        // Every stuffing byte the headroom constant budgets for was used.
        assert_eq!(
            usize::from(crate::frame::length_field(&buf)),
            1 + MAX_INSTR_PARAMS + INSTR_STUFF_HEADROOM + CRC_LEN
        );
    }

    #[test]
    fn stuffed_len_agrees_with_write_stuffed() {
        let mut out = [0u8; 256];
        for pattern in [
            vec![0xFF, 0xFF, 0xFD],
            vec![0xFF, 0xFF, 0xFD, 0xFD],
            vec![0xFF, 0xFF, 0xFD, 0xFF, 0xFF, 0xFD],
            vec![0x00, 0xFF, 0xFF, 0xFD, 0x00],
            vec![0xFF, 0xFF, 0xFF, 0xFD],
        ] {
            assert_eq!(stuffed_len(&pattern), write_stuffed(&pattern, &mut out));
        }
    }
}
