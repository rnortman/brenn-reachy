//! Incremental status-packet decoder.
//!
//! Bytes arrive from a half-duplex bus in whatever chunks the UART hands over
//! and, after a fault, mixed with garbage. The decoder is fed those bytes as
//! they come and reports at most one event per call: a complete packet, a
//! corrupt candidate, or "keep feeding". It never panics, never allocates, and
//! never blocks on more input than one frame's worth.
//!
//! A well-formed Protocol 2.0 frame that is not a status packet — what a
//! transceiver that reflects the host's own transmission would deliver — is one
//! `NotStatus` event, after which the scan resumes a byte later and finds
//! whatever follows. The decoder does not decide what such a frame means:
//! whether the receive path reflects anything is a property of the port, and
//! the layer that owns the port decides whether to skip it or to call it a wire
//! fault.
//!
//! A corrupt frame and a silent bus are different events here by construction:
//! corruption is a verdict this decoder reaches about bytes it has, and silence
//! is a verdict the caller reaches about bytes that never came. They demand
//! opposite retry policies, so they can never be the same value.

use thiserror::Error;

use crate::crc::crc_matches;
use crate::frame::{
    CRC_LEN, HEADER, IDX_ID, IDX_INSTRUCTION, IDX_STATUS_ERROR, IDX_STATUS_PARAMS, INST_STATUS,
    MAX_FRAME_BUF, MAX_STATUS_LEN, MIN_STATUS_LEN, PREAMBLE_LEN, STUFF_BYTE,
    completes_stuff_pattern, length_field,
};

/// Why a candidate frame was rejected. Each variant is a decoder verdict about
/// bytes that arrived, never about bytes that did not.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum FrameError {
    /// Length field outside the range any real status packet can claim.
    #[error("status length field {len} outside {min}..={max}")]
    BadLength { len: u16, min: u16, max: u16 },
    /// Checksum mismatch over an otherwise well-shaped frame.
    #[error("status frame failed its crc")]
    BadCrc,
    /// A well-formed Protocol 2.0 frame that is not a status packet.
    #[error("frame carries instruction {instruction:#04x}, not a status packet")]
    NotStatus { instruction: u8 },
}

/// The error field of a status packet: the servo's own verdict on the
/// instruction it just processed.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StatusError(pub u8);

/// Error numbers of Protocol 2.0's status error field.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StatusCode {
    /// The instruction could not be processed.
    ResultFail,
    /// Undefined instruction, or an Action with no registered instruction.
    Instruction,
    /// The instruction packet's own CRC did not match.
    Crc,
    /// A written value fell outside the register's range. Also the signature a
    /// latched bus watchdog returns, so callers must surface it verbatim rather
    /// than deciding which of the two it means.
    DataRange,
    /// Data length shorter or longer than the register.
    DataLength,
    /// A written value fell outside the configured limit.
    DataLimit,
    /// Write to a read-only register, read of a write-only one, or a write to
    /// EEPROM while torque is enabled.
    Access,
}

impl StatusError {
    /// No error number and no alert bit.
    #[must_use]
    pub fn is_ok(&self) -> bool {
        self.0 == 0
    }

    /// Bit 7: a hardware error is latched in Hardware Error Status. It is a
    /// standing condition, not a verdict on this instruction.
    #[must_use]
    pub fn alert(&self) -> bool {
        self.0 & 0x80 != 0
    }

    /// The error number in bits 0..6, if any.
    #[must_use]
    pub fn code(&self) -> Option<StatusCode> {
        match self.0 & 0x7F {
            1 => Some(StatusCode::ResultFail),
            2 => Some(StatusCode::Instruction),
            3 => Some(StatusCode::Crc),
            4 => Some(StatusCode::DataRange),
            5 => Some(StatusCode::DataLength),
            6 => Some(StatusCode::DataLimit),
            7 => Some(StatusCode::Access),
            _ => None,
        }
    }
}

/// A decoded status packet, borrowing the decoder's buffer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StatusView<'a> {
    /// The replying servo, taken from the packet itself. Attribution is by this
    /// field and never by arrival order.
    pub id: u8,
    /// The servo's error field.
    pub error: StatusError,
    /// Parameters, destuffed.
    pub params: &'a [u8],
}

/// One decoder event.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DecodeStep<'a> {
    /// Nothing decidable yet; feed more bytes.
    NeedMore,
    /// A complete, CRC-valid status packet.
    Packet(StatusView<'a>),
    /// A rejected candidate. One byte has been discarded and the scan resumes
    /// from the next; call again to look for a frame behind the garbage.
    Corrupt(FrameError),
}

/// What the buffered bytes alone can decide, with no borrows outstanding.
#[derive(Debug, Clone, Copy)]
enum Progress {
    Need,
    Frame {
        id: u8,
        error: u8,
        params_len: usize,
    },
    Bad(FrameError),
}

/// Incremental decoder over one bus's receive stream.
#[derive(Debug)]
pub struct StatusDecoder {
    buf: [u8; MAX_FRAME_BUF],
    fill: usize,
    /// Bytes of an already-reported frame still sitting at the front, dropped
    /// at the start of the next call so the reported parameters stay borrowable
    /// until then.
    pending: usize,
}

impl Default for StatusDecoder {
    fn default() -> Self {
        Self::new()
    }
}

impl StatusDecoder {
    /// A decoder with an empty buffer.
    #[must_use]
    pub fn new() -> Self {
        Self {
            buf: [0; MAX_FRAME_BUF],
            fill: 0,
            pending: 0,
        }
    }

    /// Discards every buffered byte. Used when a transaction is abandoned, so
    /// the tail of one exchange cannot be read as the head of the next.
    pub fn reset(&mut self) {
        self.fill = 0;
        self.pending = 0;
    }

    /// Feeds bytes and reports at most one event. The returned count is how
    /// many input bytes were taken; the caller repeats with the remainder until
    /// it is exhausted, which is what drains several frames out of one read.
    pub fn feed(&mut self, input: &[u8]) -> (usize, DecodeStep<'_>) {
        if self.pending > 0 {
            let pending = self.pending;
            self.drain_front(pending);
            self.pending = 0;
        }

        let mut consumed = 0;
        let outcome = loop {
            match self.advance() {
                Progress::Need => {
                    if consumed == input.len() {
                        break Progress::Need;
                    }
                    // A candidate frame is at most MAX_STATUS_FRAME bytes and
                    // the header scan leaves at most a three-byte prefix, so
                    // there is always room here; discarding keeps the loop
                    // making progress if that ever stops holding.
                    debug_assert!(self.fill < MAX_FRAME_BUF);
                    if self.fill == MAX_FRAME_BUF {
                        self.drain_front(1);
                        continue;
                    }
                    self.buf[self.fill] = input[consumed];
                    self.fill += 1;
                    consumed += 1;
                }
                other => break other,
            }
        };

        let step = match outcome {
            Progress::Need => DecodeStep::NeedMore,
            Progress::Bad(err) => DecodeStep::Corrupt(err),
            Progress::Frame {
                id,
                error,
                params_len,
            } => DecodeStep::Packet(StatusView {
                id,
                error: StatusError(error),
                params: &self.buf[IDX_STATUS_PARAMS..IDX_STATUS_PARAMS + params_len],
            }),
        };
        (consumed, step)
    }

    /// Header scan, length check, CRC, instruction check, destuff — the whole
    /// verdict on the buffered bytes, without producing a borrow.
    fn advance(&mut self) -> Progress {
        self.scan_header();
        if self.fill < PREAMBLE_LEN {
            return Progress::Need;
        }

        let len = usize::from(length_field(&self.buf));
        if !(MIN_STATUS_LEN..=MAX_STATUS_LEN).contains(&len) {
            self.drain_front(1);
            return Progress::Bad(FrameError::BadLength {
                len: len as u16,
                min: MIN_STATUS_LEN as u16,
                max: MAX_STATUS_LEN as u16,
            });
        }

        let total = PREAMBLE_LEN + len;
        if self.fill < total {
            return Progress::Need;
        }

        if !crc_matches(&self.buf[..total]) {
            self.drain_front(1);
            return Progress::Bad(FrameError::BadCrc);
        }

        let instruction = self.buf[IDX_INSTRUCTION];
        if instruction != INST_STATUS {
            self.drain_front(1);
            return Progress::Bad(FrameError::NotStatus { instruction });
        }

        let id = self.buf[IDX_ID];
        let region_end = total - CRC_LEN;
        let destuffed_end = self.destuff(IDX_INSTRUCTION, region_end);
        let error = self.buf[IDX_STATUS_ERROR];
        self.pending = total;
        Progress::Frame {
            id,
            error,
            params_len: destuffed_end - IDX_STATUS_PARAMS,
        }
    }

    /// Removes the `FD` inserted after each `FF FF FD` in `start..end`,
    /// compacting in place. Returns the end of the destuffed region.
    ///
    /// The pattern is recognised in the bytes already written out, which is the
    /// same sequence the sender examined before inserting, so encode and decode
    /// stay exact inverses.
    fn destuff(&mut self, start: usize, end: usize) -> usize {
        let mut write = start;
        let mut read = start;
        while read < end {
            self.buf[write] = self.buf[read];
            let stuffed = write >= start + 2
                && completes_stuff_pattern(
                    self.buf[write - 2],
                    self.buf[write - 1],
                    self.buf[write],
                )
                && read + 1 < end
                && self.buf[read + 1] == STUFF_BYTE;
            read += if stuffed { 2 } else { 1 };
            write += 1;
        }
        write
    }

    /// Drops leading bytes until the buffer starts with the header, or with a
    /// partial header prefix that runs to the end of the buffered bytes.
    fn scan_header(&mut self) {
        let mut at = 0;
        while at < self.fill {
            let n = (self.fill - at).min(HEADER.len());
            if self.buf[at..at + n] == HEADER[..n] && (n == HEADER.len() || at + n == self.fill) {
                break;
            }
            at += 1;
        }
        if at > 0 {
            self.drain_front(at);
        }
    }

    fn drain_front(&mut self, n: usize) {
        let n = n.min(self.fill);
        self.buf.copy_within(n..self.fill, 0);
        self.fill -= n;
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::encode::encode_write;
    use crate::frame::{MAX_INSTR_FRAME, MAX_STATUS_PARAMS};

    /// A ping reply from ID 1: model 0x0406, firmware 0x26. Adapted from the
    /// Robotis Protocol 2.0 documented example.
    const PING_REPLY: [u8; 14] = [
        0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x07, 0x00, 0x55, 0x00, 0x06, 0x04, 0x26, 0x65, 0x5D,
    ];

    /// A four-parameter read reply from ID 1. Adapted from the Apache-2.0
    /// rustypot crate's Protocol 2.0 codec tests (see NOTICE).
    const READ_REPLY: [u8; 15] = [
        0xFF, 0xFF, 0xFD, 0x00, 0x01, 0x08, 0x00, 0x55, 0x00, 0xA6, 0x00, 0x00, 0x00, 0x8C, 0xC0,
    ];

    /// The reply to a write from ID 5: no parameters at all, which is the
    /// shortest status packet the protocol defines and the acknowledgement
    /// every command write is judged on.
    const WRITE_ACK: [u8; 11] = [
        0xFF, 0xFF, 0xFD, 0x00, 0x05, 0x04, 0x00, 0x55, 0x00, 0x42, 0x8D,
    ];

    /// A status frame with the given length field, filled so that it is
    /// otherwise well formed. Used to probe the accept/reject edge of the
    /// length gate, which is the only check that runs before this much of a
    /// frame has even arrived.
    fn status_frame_with_len(len: usize) -> Vec<u8> {
        let mut frame = vec![0xFF, 0xFF, 0xFD, 0x00, 0x07];
        frame.extend_from_slice(&(len as u16).to_le_bytes());
        frame.push(INST_STATUS);
        frame.push(0x00);
        frame.resize(PREAMBLE_LEN + len - CRC_LEN, 0x11);
        let crc = crate::crc::crc16(&frame);
        frame.extend_from_slice(&crc.to_le_bytes());
        frame
    }

    /// Feeds everything, collecting every event the decoder reports.
    fn drain(decoder: &mut StatusDecoder, mut input: &[u8]) -> Vec<Event> {
        let mut events = Vec::new();
        loop {
            let (used, step) = decoder.feed(input);
            match step {
                DecodeStep::NeedMore => {
                    events.push(Event::NeedMore);
                    return events;
                }
                DecodeStep::Packet(view) => events.push(Event::Packet {
                    id: view.id,
                    error: view.error,
                    params: view.params.to_vec(),
                }),
                DecodeStep::Corrupt(err) => events.push(Event::Corrupt(err)),
            }
            input = &input[used..];
        }
    }

    #[derive(Debug, PartialEq, Eq)]
    enum Event {
        NeedMore,
        Packet {
            id: u8,
            error: StatusError,
            params: Vec<u8>,
        },
        Corrupt(FrameError),
    }

    fn packet(id: u8, params: &[u8]) -> Event {
        Event::Packet {
            id,
            error: StatusError(0),
            params: params.to_vec(),
        }
    }

    #[test]
    fn decodes_a_ping_reply() {
        let mut decoder = StatusDecoder::new();
        assert_eq!(
            drain(&mut decoder, &PING_REPLY),
            vec![packet(1, &[0x06, 0x04, 0x26]), Event::NeedMore]
        );
    }

    #[test]
    fn decodes_a_read_reply() {
        let mut decoder = StatusDecoder::new();
        assert_eq!(
            drain(&mut decoder, &READ_REPLY),
            vec![packet(1, &[0xA6, 0x00, 0x00, 0x00]), Event::NeedMore]
        );
    }

    #[test]
    fn decodes_across_arbitrary_chunk_boundaries() {
        for chunk in 1..=READ_REPLY.len() {
            let mut decoder = StatusDecoder::new();
            let mut events = Vec::new();
            for piece in READ_REPLY.chunks(chunk) {
                events.extend(drain(&mut decoder, piece));
            }
            events.retain(|e| !matches!(e, Event::NeedMore));
            assert_eq!(
                events,
                vec![packet(1, &[0xA6, 0x00, 0x00, 0x00])],
                "{chunk}"
            );
        }
    }

    #[test]
    fn decodes_two_frames_from_one_feed() {
        let mut stream = Vec::new();
        stream.extend_from_slice(&PING_REPLY);
        stream.extend_from_slice(&READ_REPLY);
        let mut decoder = StatusDecoder::new();
        assert_eq!(
            drain(&mut decoder, &stream),
            vec![
                packet(1, &[0x06, 0x04, 0x26]),
                packet(1, &[0xA6, 0x00, 0x00, 0x00]),
                Event::NeedMore
            ]
        );
    }

    #[test]
    fn resyncs_onto_a_frame_behind_garbage() {
        let mut stream = vec![0x00, 0xFF, 0xFF, 0x12, 0xFD, 0x00, 0xAB];
        stream.extend_from_slice(&READ_REPLY);
        let mut decoder = StatusDecoder::new();
        let events = drain(&mut decoder, &stream);
        assert_eq!(events.last(), Some(&Event::NeedMore));
        assert!(events.contains(&packet(1, &[0xA6, 0x00, 0x00, 0x00])));
    }

    #[test]
    fn bad_crc_is_corrupt_then_recovers() {
        let mut broken = READ_REPLY;
        broken[9] ^= 0xFF;
        let mut stream = broken.to_vec();
        stream.extend_from_slice(&READ_REPLY);
        let mut decoder = StatusDecoder::new();
        let events = drain(&mut decoder, &stream);
        assert_eq!(events[0], Event::Corrupt(FrameError::BadCrc));
        assert!(events.contains(&packet(1, &[0xA6, 0x00, 0x00, 0x00])));
    }

    /// A write acknowledgement carries nothing but the error field. The
    /// destuffing arithmetic has to come out at an empty parameter slice
    /// rather than underflowing past the start of one.
    #[test]
    fn decodes_a_zero_parameter_reply() {
        let mut decoder = StatusDecoder::new();
        assert_eq!(
            drain(&mut decoder, &WRITE_ACK),
            vec![packet(5, &[]), Event::NeedMore]
        );
    }

    /// The length gate's upper edge, from both sides: the largest frame this
    /// crate is dimensioned for decodes, and one byte more is a corrupt length
    /// rather than a large packet.
    #[test]
    fn largest_accepted_length_decodes_and_one_more_does_not() {
        let accepted = status_frame_with_len(MAX_STATUS_LEN);
        let mut decoder = StatusDecoder::new();
        let events = drain(&mut decoder, &accepted);
        assert_eq!(events[0], packet(7, &[0x11; MAX_STATUS_LEN - CRC_LEN - 2]));

        let rejected = status_frame_with_len(MAX_STATUS_LEN + 1);
        let mut decoder = StatusDecoder::new();
        let events = drain(&mut decoder, &rejected);
        assert_eq!(
            events[0],
            Event::Corrupt(FrameError::BadLength {
                len: MAX_STATUS_LEN as u16 + 1,
                min: MIN_STATUS_LEN as u16,
                max: MAX_STATUS_LEN as u16,
            })
        );
    }

    /// Garbage that happens to spell a header and claim a near-maximal length
    /// holds a whole buffer's worth of bytes hostage. Once those bytes arrive
    /// the candidate fails its CRC, and the frame hiding behind it comes out.
    #[test]
    fn a_false_header_does_not_swallow_the_frame_behind_it() {
        let mut stream = vec![0xFF, 0xFF, 0xFD, 0x00, 0x09];
        stream.extend_from_slice(&(MAX_STATUS_LEN as u16).to_le_bytes());
        stream.extend_from_slice(&[0x55; MAX_STATUS_LEN]);
        stream.extend_from_slice(&READ_REPLY);
        let mut decoder = StatusDecoder::new();
        let events = drain(&mut decoder, &stream);
        assert_eq!(events[0], Event::Corrupt(FrameError::BadCrc));
        assert!(events.contains(&packet(1, &[0xA6, 0x00, 0x00, 0x00])));
    }

    /// Garbage ending in a partial header prefix: the scan keeps the prefix,
    /// and the real header arriving right behind it must still be found.
    #[test]
    fn garbage_ending_in_a_header_prefix_still_resyncs() {
        for prefix in [
            &[0x22u8, 0xFF][..],
            &[0x22, 0xFF, 0xFF],
            &[0xFF, 0xFF, 0xFD],
        ] {
            let mut stream = prefix.to_vec();
            stream.extend_from_slice(&READ_REPLY);
            let mut decoder = StatusDecoder::new();
            let mut events = Vec::new();
            for piece in stream.chunks(3) {
                events.extend(drain(&mut decoder, piece));
            }
            assert!(
                events.contains(&packet(1, &[0xA6, 0x00, 0x00, 0x00])),
                "prefix {prefix:?} gave {events:?}"
            );
        }
    }

    #[test]
    fn bad_length_is_named() {
        let mut broken = READ_REPLY;
        broken[5] = 0x01;
        broken[6] = 0x00;
        let mut decoder = StatusDecoder::new();
        let events = drain(&mut decoder, &broken);
        assert_eq!(
            events[0],
            Event::Corrupt(FrameError::BadLength {
                len: 1,
                min: MIN_STATUS_LEN as u16,
                max: MAX_STATUS_LEN as u16,
            })
        );
    }

    #[test]
    fn an_instruction_frame_is_not_a_status_frame() {
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let n = encode_write(1, 116, &512u32.to_le_bytes(), &mut buf).unwrap();
        let mut decoder = StatusDecoder::new();
        let events = drain(&mut decoder, &buf[..n]);
        assert_eq!(
            events[0],
            Event::Corrupt(FrameError::NotStatus { instruction: 0x03 })
        );
    }

    #[test]
    fn error_field_is_surfaced_not_swallowed() {
        // Same shape as the ping reply, with the servo reporting Data Range and
        // the alert bit set; the CRC is recomputed over the altered frame.
        let mut frame = PING_REPLY;
        frame[8] = 0x84;
        let crc = crate::crc::crc16(&frame[..12]);
        frame[12..14].copy_from_slice(&crc.to_le_bytes());

        let mut decoder = StatusDecoder::new();
        let (_, step) = decoder.feed(&frame);
        match step {
            DecodeStep::Packet(view) => {
                assert!(!view.error.is_ok());
                assert!(view.error.alert());
                assert_eq!(view.error.code(), Some(StatusCode::DataRange));
            }
            other => panic!("expected a packet, got {other:?}"),
        }
    }

    #[test]
    fn reset_drops_a_partial_frame() {
        let mut decoder = StatusDecoder::new();
        let (_, step) = decoder.feed(&READ_REPLY[..6]);
        assert_eq!(step, DecodeStep::NeedMore);
        decoder.reset();
        assert_eq!(
            drain(&mut decoder, &READ_REPLY[6..]),
            vec![Event::NeedMore],
            "a frame's tail alone must not decode"
        );
    }

    /// Round-trips a payload seeded with the header pattern at every offset:
    /// encode stuffs, decode destuffs, parameters come back identical.
    #[test]
    fn stuffing_round_trips_at_every_offset() {
        for offset in 0..8usize {
            let mut params = [0x11u8; 12];
            params[offset] = 0xFF;
            params[offset + 1] = 0xFF;
            params[offset + 2] = 0xFD;
            params[offset + 3] = 0x00;

            let mut buf = [0u8; MAX_INSTR_FRAME];
            let n = encode_write(1, 116, &params, &mut buf).unwrap();
            assert!(crate::crc::crc_matches(&buf[..n]), "offset {offset}");

            // Re-badge the encoded frame as a status packet so the decoder,
            // which only accepts 0x55, exercises the same destuffing path.
            let mut frame = buf[..n].to_vec();
            frame[7] = INST_STATUS;
            let crc = crate::crc::crc16(&frame[..n - 2]);
            frame[n - 2..].copy_from_slice(&crc.to_le_bytes());

            let mut decoder = StatusDecoder::new();
            let events = drain(&mut decoder, &frame);
            // Re-badging shifts the fields by one: the write instruction's low
            // address byte lands where a status packet's error field is, and
            // the parameters are the high address byte plus the payload.
            let mut expected = vec![0x00];
            expected.extend_from_slice(&params);
            assert_eq!(
                events[0],
                Event::Packet {
                    id: 1,
                    error: StatusError(116),
                    params: expected,
                },
                "offset {offset}"
            );
        }
    }

    /// The widest status payload this crate accepts, seeded so that every three
    /// bytes of the stuffed region need a stuffing byte. It has to stay inside
    /// the receive buffer and come back byte-identical.
    #[test]
    fn worst_case_stuffed_status_frame_fits_the_buffer() {
        // Address 0xFFFF supplies the first FF FF, so the pattern repeats from
        // the first payload byte onward with no break. Re-badged as a status
        // packet the fields shift by one: the low address byte becomes the
        // error field and the high one becomes the first parameter.
        let payload: Vec<u8> = (0..MAX_STATUS_PARAMS - 1)
            .map(|i| if i % 3 == 0 { 0xFD } else { 0xFF })
            .collect();
        let mut buf = [0u8; MAX_INSTR_FRAME];
        let n = encode_write(3, 0xFFFF, &payload, &mut buf).unwrap();
        let mut frame = buf[..n].to_vec();
        frame[IDX_INSTRUCTION] = INST_STATUS;
        let crc = crate::crc::crc16(&frame[..n - CRC_LEN]);
        frame[n - CRC_LEN..].copy_from_slice(&crc.to_le_bytes());

        assert!(n <= MAX_FRAME_BUF, "stuffed status frame is {n} bytes");
        assert!(usize::from(length_field(&frame)) <= MAX_STATUS_LEN);

        let mut expected = vec![0xFF];
        expected.extend_from_slice(&payload);
        assert_eq!(expected.len(), MAX_STATUS_PARAMS);
        let mut decoder = StatusDecoder::new();
        let events = drain(&mut decoder, &frame);
        assert_eq!(
            events[0],
            Event::Packet {
                id: 3,
                error: StatusError(0xFF),
                params: expected,
            }
        );
    }

    /// Arbitrary bytes in arbitrary chunkings: no panic, always terminates,
    /// every feed either consumes input or drains a buffered byte, and a good
    /// frame behind the garbage is always recovered.
    #[test]
    fn arbitrary_input_never_panics_and_always_resyncs() {
        let mut state: u32 = 0x1234_5678;
        let mut next = move || {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            (state >> 24) as u8
        };

        for round in 0..2_000 {
            let len = usize::from(next()) % 200 + 1;
            let mut stream: Vec<u8> = (0..len)
                .map(|_| match next() % 4 {
                    0 => 0xFF,
                    1 => 0xFD,
                    2 => 0x00,
                    _ => next(),
                })
                .collect();
            stream.extend_from_slice(&READ_REPLY);
            // Noise that spells a header can claim a length holding a whole
            // buffer of bytes hostage. The trailing filler — bytes that can
            // never begin a header — is what lets such a candidate reach its
            // verdict, after which the scan walks onto the real frame.
            stream.extend_from_slice(&[0x11; MAX_FRAME_BUF]);

            let mut decoder = StatusDecoder::new();
            let chunk = usize::from(next()) % 7 + 1;
            let mut recovered = false;
            for piece in stream.chunks(chunk) {
                let mut input: &[u8] = piece;
                let mut steps = 0;
                loop {
                    let (used, step) = decoder.feed(input);
                    steps += 1;
                    // Every step consumes input or drops at least one buffered
                    // byte, and the buffer holds at most MAX_FRAME_BUF.
                    assert!(
                        steps <= 2 * piece.len() + MAX_FRAME_BUF + 2,
                        "round {round} made no progress"
                    );
                    match step {
                        DecodeStep::NeedMore => {
                            assert_eq!(used, input.len());
                            break;
                        }
                        DecodeStep::Packet(view) => {
                            recovered |= view.id == 1
                                && view.params == [0xA6u8, 0x00, 0x00, 0x00].as_slice();
                        }
                        DecodeStep::Corrupt(_) => {}
                    }
                    input = &input[used..];
                }
            }
            assert!(recovered, "round {round} lost the frame behind the garbage");
        }
    }
}
