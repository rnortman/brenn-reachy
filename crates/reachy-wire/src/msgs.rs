//! The four messages, their byte layouts, and the dispatching decoder.
//!
//! Each type states its own `MSG_TYPE` and total length as constants, encodes
//! into a fixed array of exactly that length, and decodes from a slice of
//! exactly that length. The length constants are the layout: they are asserted
//! against the field sizes in this module's tests and against the golden
//! vectors in [`crate::golden`], so a field added without a length change
//! cannot compile-and-ship.

use crate::{
    DecodeError, HEADER_LEN, Header, JOINT_COUNT, JOINT_MASK_ALL, Reader, Writer, bus_mask,
    check_mask, expect,
};

/// Total bytes of a [`PoseSample`] datagram.
pub const POSE_SAMPLE_LEN: usize = HEADER_LEN + 163;

/// Total bytes of a [`DriverEvent`] datagram.
pub const DRIVER_EVENT_LEN: usize = HEADER_LEN + 13;

/// Total bytes of a [`GoalSetpoint`] datagram.
pub const GOAL_SETPOINT_LEN: usize = HEADER_LEN + 82;

/// Total bytes of a [`Control`] datagram.
pub const CONTROL_LEN: usize = HEADER_LEN + 1;

/// The driver's per-cycle report: what it read, what it is holding, and how
/// well the read went.
///
/// One of these is published every cycle without exception, whatever else
/// happened — it is the heartbeat the control loop runs on, so a cycle in which
/// the bus went silent is a sample carrying `present_valid: false`, never an
/// absent sample.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PoseSample {
    /// The cycle's grid instant, in nanoseconds since the Unix epoch.
    pub nominal_time_ns: i64,
    /// When the bus read actually completed. Equal to the nominal instant only
    /// where there is no read jitter to report, as in simulation.
    pub sample_time_ns: i64,
    /// Whether `present` is a complete reading of every row.
    pub present_valid: bool,
    /// Whether `commanded` reflects a setpoint the driver is holding.
    pub commanded_valid: bool,
    /// Whether the driver has latched torque off and is refusing to write.
    pub torque_off_latched: bool,
    /// Rows that did not answer this cycle, one bit per bus row.
    pub miss_mask: u16,
    /// Measured positions in bus order, radians.
    pub present: [f64; JOINT_COUNT],
    /// The setpoint currently held, in bus order, radians.
    pub commanded: [f64; JOINT_COUNT],
}

impl PoseSample {
    /// Header type byte.
    pub const MSG_TYPE: u8 = 0x01;
    /// Total datagram length.
    pub const LEN: usize = POSE_SAMPLE_LEN;

    const FLAG_PRESENT_VALID: u8 = 0x01;
    const FLAG_COMMANDED_VALID: u8 = 0x02;
    const FLAG_TORQUE_OFF_LATCHED: u8 = 0x04;
    const FLAG_KNOWN: u8 =
        Self::FLAG_PRESENT_VALID | Self::FLAG_COMMANDED_VALID | Self::FLAG_TORQUE_OFF_LATCHED;

    /// The flags byte as it goes on the wire.
    #[must_use]
    pub fn flags(&self) -> u8 {
        let mut flags = 0;
        if self.present_valid {
            flags |= Self::FLAG_PRESENT_VALID;
        }
        if self.commanded_valid {
            flags |= Self::FLAG_COMMANDED_VALID;
        }
        if self.torque_off_latched {
            flags |= Self::FLAG_TORQUE_OFF_LATCHED;
        }
        flags
    }

    /// Encode a whole datagram, header included.
    ///
    /// `miss_mask` is written as the nine bus rows it names and nothing else,
    /// so what goes out is always a datagram [`Self::decode`] accepts. Bits
    /// outside the rows are a caller bug and a debug build panics on them.
    ///
    /// # Panics
    ///
    /// In a debug build, if `miss_mask` has bits set outside the bus rows.
    #[must_use]
    pub fn encode(&self, seq: u32) -> [u8; POSE_SAMPLE_LEN] {
        debug_assert_eq!(
            self.miss_mask & !JOINT_MASK_ALL,
            0,
            "miss_mask has bits outside the nine bus rows"
        );
        let mut w = Writer::<POSE_SAMPLE_LEN>::new(Self::MSG_TYPE, seq);
        w.i64(self.nominal_time_ns);
        w.i64(self.sample_time_ns);
        w.u8(self.flags());
        w.u16(bus_mask(self.miss_mask));
        w.joints(&self.present);
        w.joints(&self.commanded);
        w.finish()
    }

    /// Decode a whole datagram, header included.
    ///
    /// # Errors
    ///
    /// Any [`DecodeError`]: a header this crate does not speak, a length other
    /// than [`POSE_SAMPLE_LEN`], or reserved bits set in the flags byte or the
    /// miss mask.
    pub fn decode(bytes: &[u8]) -> Result<(Header, Self), DecodeError> {
        let header = expect(bytes, Self::MSG_TYPE, Self::LEN)?;
        let mut r = Reader::body(bytes);
        let sample = (|| {
            let nominal_time_ns = r.i64()?;
            let sample_time_ns = r.i64()?;
            let flags = r.u8()?;
            let miss_mask = r.u16()?;
            let present = r.joints()?;
            let commanded = r.joints()?;
            Ok((
                nominal_time_ns,
                sample_time_ns,
                flags,
                miss_mask,
                present,
                commanded,
            ))
        })()
        .map_err(|()| short(Self::MSG_TYPE, Self::LEN, bytes.len()))?;
        let (nominal_time_ns, sample_time_ns, flags, miss_mask, present, commanded) = sample;
        let stray = flags & !Self::FLAG_KNOWN;
        if stray != 0 {
            return Err(DecodeError::ReservedBits {
                field: "flags",
                bits: u16::from(stray),
            });
        }
        Ok((
            header,
            Self {
                nominal_time_ns,
                sample_time_ns,
                present_valid: flags & Self::FLAG_PRESENT_VALID != 0,
                commanded_valid: flags & Self::FLAG_COMMANDED_VALID != 0,
                torque_off_latched: flags & Self::FLAG_TORQUE_OFF_LATCHED != 0,
                miss_mask: check_mask("miss_mask", miss_mask)?,
                present,
                commanded,
            },
        ))
    }
}

/// Something the driver did, or refused to do, that is not visible in the
/// sample stream.
///
/// Events are per-occurrence, never per-cycle: a standing condition is visible
/// in [`PoseSample`] flags, and an event marks the edge that got it there.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DriverEvent {
    /// What happened.
    pub kind: EventKind,
    /// The number that makes the event diagnosable, whose meaning is stated
    /// per kind on [`EventKind`]. Zero where the kind names none.
    pub detail: u32,
    /// When it happened, nanoseconds since the Unix epoch.
    pub time_ns: i64,
}

/// The events a driver reports.
///
/// Each kind says what [`DriverEvent::detail`] carries for it, because a number
/// whose meaning is not stated at its one wire home is a number every sender
/// fills in differently.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum EventKind {
    /// No goal arrived within the hold timeout; the driver latched torque off.
    /// `detail` is how long the goal stream was silent, microseconds.
    HoldTimeoutTorqueOff = 1,
    /// The driver wrote the minimum-risk condition at startup. `detail` is
    /// unused.
    StartupMrcWrite = 2,
    /// A torque-off write was confirmed by every row it named. `detail` is
    /// unused.
    TorqueOffConfirmed = 3,
    /// A torque-off write went unconfirmed by at least one row. `detail` is the
    /// rows that did not confirm, one bit per bus row.
    TorqueOffUnconfirmed = 4,
    /// A bus cycle did not run in its slot. `detail` is how late the cycle
    /// was, microseconds.
    CycleSkipped = 5,
    /// A goal arrived with the queue full and was dropped. `detail` is how many
    /// goals were queued when it arrived.
    GoalDroppedQueueFull = 6,
    /// A goal arrived stale or out of order. It is executed anyway, in arrival
    /// order; this says so. `detail` is how far past its instant it arrived,
    /// microseconds, and zero for one that is merely out of order.
    GoalStaleOrOutOfOrder = 7,
    /// The bus itself failed. `detail` is the failing servo's bus id.
    BusFailure = 8,
}

impl EventKind {
    /// Every kind, in wire-value order.
    pub const ALL: [Self; 8] = [
        Self::HoldTimeoutTorqueOff,
        Self::StartupMrcWrite,
        Self::TorqueOffConfirmed,
        Self::TorqueOffUnconfirmed,
        Self::CycleSkipped,
        Self::GoalDroppedQueueFull,
        Self::GoalStaleOrOutOfOrder,
        Self::BusFailure,
    ];

    /// The wire value.
    #[must_use]
    pub fn as_u8(self) -> u8 {
        self as u8
    }

    /// The kind a wire value names, or `None` for one this crate does not
    /// speak.
    #[must_use]
    pub fn from_u8(value: u8) -> Option<Self> {
        Self::ALL.iter().copied().find(|kind| kind.as_u8() == value)
    }
}

impl DriverEvent {
    /// Header type byte.
    pub const MSG_TYPE: u8 = 0x03;
    /// Total datagram length.
    pub const LEN: usize = DRIVER_EVENT_LEN;

    /// Encode a whole datagram, header included.
    #[must_use]
    pub fn encode(&self, seq: u32) -> [u8; DRIVER_EVENT_LEN] {
        let mut w = Writer::<DRIVER_EVENT_LEN>::new(Self::MSG_TYPE, seq);
        w.u8(self.kind.as_u8());
        w.u32(self.detail);
        w.i64(self.time_ns);
        w.finish()
    }

    /// Decode a whole datagram, header included.
    ///
    /// # Errors
    ///
    /// Any [`DecodeError`]: a header this crate does not speak, a length other
    /// than [`DRIVER_EVENT_LEN`], or an unknown event kind.
    pub fn decode(bytes: &[u8]) -> Result<(Header, Self), DecodeError> {
        let header = expect(bytes, Self::MSG_TYPE, Self::LEN)?;
        let mut r = Reader::body(bytes);
        let fields = (|| Ok((r.u8()?, r.u32()?, r.i64()?)))()
            .map_err(|()| short(Self::MSG_TYPE, Self::LEN, bytes.len()))?;
        let (kind, detail, time_ns) = fields;
        let kind = EventKind::from_u8(kind).ok_or(DecodeError::UnknownEnum {
            field: "kind",
            value: kind,
        })?;
        Ok((
            header,
            Self {
                kind,
                detail,
                time_ns,
            },
        ))
    }
}

/// A commanded setpoint and the grid instant it is for.
///
/// `mask` is write-side filtering and nothing else: a row outside the mask
/// keeps whatever it was already holding, and the target carried for it in
/// `targets` is not written. That is the whole meaning of the mask to a
/// driver, and every host of the goal gate honours it identically.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GoalSetpoint {
    /// The grid instant this setpoint is to be written at, nanoseconds since
    /// the Unix epoch.
    pub execute_at_ns: i64,
    /// Rows this setpoint writes, one bit per bus row.
    pub mask: u16,
    /// Targets in bus order, radians. Rows outside `mask` are not written.
    pub targets: [f64; JOINT_COUNT],
}

impl GoalSetpoint {
    /// Header type byte.
    pub const MSG_TYPE: u8 = 0x10;
    /// Total datagram length.
    pub const LEN: usize = GOAL_SETPOINT_LEN;

    /// Encode a whole datagram, header included.
    ///
    /// `mask` is written as the nine bus rows it names and nothing else, so
    /// what goes out is always a datagram [`Self::decode`] accepts. Bits
    /// outside the rows are a caller bug and a debug build panics on them.
    ///
    /// # Panics
    ///
    /// In a debug build, if `mask` has bits set outside the bus rows.
    #[must_use]
    pub fn encode(&self, seq: u32) -> [u8; GOAL_SETPOINT_LEN] {
        debug_assert_eq!(
            self.mask & !JOINT_MASK_ALL,
            0,
            "mask has bits outside the nine bus rows"
        );
        let mut w = Writer::<GOAL_SETPOINT_LEN>::new(Self::MSG_TYPE, seq);
        w.i64(self.execute_at_ns);
        w.u16(bus_mask(self.mask));
        w.joints(&self.targets);
        w.finish()
    }

    /// Decode a whole datagram, header included.
    ///
    /// # Errors
    ///
    /// Any [`DecodeError`]: a header this crate does not speak, a length other
    /// than [`GOAL_SETPOINT_LEN`], or mask bits outside the bus rows.
    pub fn decode(bytes: &[u8]) -> Result<(Header, Self), DecodeError> {
        let header = expect(bytes, Self::MSG_TYPE, Self::LEN)?;
        let mut r = Reader::body(bytes);
        let fields = (|| Ok((r.i64()?, r.u16()?, r.joints()?)))()
            .map_err(|()| short(Self::MSG_TYPE, Self::LEN, bytes.len()))?;
        let (execute_at_ns, mask, targets) = fields;
        Ok((
            header,
            Self {
                execute_at_ns,
                mask: check_mask("mask", mask)?,
                targets,
            },
        ))
    }
}

/// An out-of-band instruction to the driver.
///
/// Deliberately tiny and deliberately not a setpoint: the only thing anything
/// may tell the driver out of band is to stop holding torque, because that is
/// the one action nothing is ever allowed to gate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Control {
    /// What to do.
    pub op: ControlOp,
}

/// The control operations a driver accepts.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[repr(u8)]
pub enum ControlOp {
    /// De-torque every row now, whatever else is in flight.
    TorqueOffNow = 1,
}

impl ControlOp {
    /// Every operation, in wire-value order.
    pub const ALL: [Self; 1] = [Self::TorqueOffNow];

    /// The wire value.
    #[must_use]
    pub fn as_u8(self) -> u8 {
        self as u8
    }

    /// The operation a wire value names, or `None` for one this crate does not
    /// speak.
    #[must_use]
    pub fn from_u8(value: u8) -> Option<Self> {
        Self::ALL.iter().copied().find(|op| op.as_u8() == value)
    }
}

impl Control {
    /// Header type byte.
    pub const MSG_TYPE: u8 = 0x11;
    /// Total datagram length.
    pub const LEN: usize = CONTROL_LEN;

    /// Encode a whole datagram, header included.
    #[must_use]
    pub fn encode(&self, seq: u32) -> [u8; CONTROL_LEN] {
        let mut w = Writer::<CONTROL_LEN>::new(Self::MSG_TYPE, seq);
        w.u8(self.op.as_u8());
        w.finish()
    }

    /// Decode a whole datagram, header included.
    ///
    /// # Errors
    ///
    /// Any [`DecodeError`]: a header this crate does not speak, a length other
    /// than [`CONTROL_LEN`], or an unknown operation.
    pub fn decode(bytes: &[u8]) -> Result<(Header, Self), DecodeError> {
        let header = expect(bytes, Self::MSG_TYPE, Self::LEN)?;
        let mut r = Reader::body(bytes);
        let op = r
            .u8()
            .map_err(|()| short(Self::MSG_TYPE, Self::LEN, bytes.len()))?;
        let op = ControlOp::from_u8(op).ok_or(DecodeError::UnknownEnum {
            field: "op",
            value: op,
        })?;
        Ok((header, Self { op }))
    }
}

/// Whichever message a datagram turned out to carry.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Message {
    /// A driver's per-cycle report.
    PoseSample(PoseSample),
    /// A driver boundary event.
    DriverEvent(DriverEvent),
    /// A commanded setpoint.
    GoalSetpoint(GoalSetpoint),
    /// An out-of-band instruction.
    Control(Control),
}

/// Decode any datagram this crate speaks.
///
/// The dispatching entry point: a receiver on a channel that carries more than
/// one message type calls this and matches, instead of guessing a type from a
/// length.
///
/// # Errors
///
/// Any [`DecodeError`], including [`DecodeError::UnknownType`] for a
/// well-formed header naming a message this crate does not speak.
pub fn decode(bytes: &[u8]) -> Result<(Header, Message), DecodeError> {
    let header = crate::peek_header(bytes)?;
    match header.msg_type {
        PoseSample::MSG_TYPE => PoseSample::decode(bytes).map(|(h, m)| (h, Message::PoseSample(m))),
        DriverEvent::MSG_TYPE => {
            DriverEvent::decode(bytes).map(|(h, m)| (h, Message::DriverEvent(m)))
        }
        GoalSetpoint::MSG_TYPE => {
            GoalSetpoint::decode(bytes).map(|(h, m)| (h, Message::GoalSetpoint(m)))
        }
        Control::MSG_TYPE => Control::decode(bytes).map(|(h, m)| (h, Message::Control(m))),
        msg_type => Err(DecodeError::UnknownType { msg_type }),
    }
}

/// The error a body read that ran out of bytes reports.
///
/// Unreachable in practice — every body decode runs behind an exact-length
/// check — but the decoders are written so that removing that check would
/// produce a refusal rather than a panic.
fn short(msg_type: u8, expected: usize, actual: usize) -> DecodeError {
    DecodeError::WrongLength {
        msg_type,
        expected,
        actual,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{JOINT_MASK_ALL, MAGIC, VERSION};

    fn joints(base: f64) -> [f64; JOINT_COUNT] {
        core::array::from_fn(|i| base + i as f64 * 0.125)
    }

    fn sample() -> PoseSample {
        PoseSample {
            nominal_time_ns: 1_700_000_000_000_000_000,
            sample_time_ns: 1_700_000_000_000_400_000,
            present_valid: true,
            commanded_valid: true,
            torque_off_latched: false,
            miss_mask: 0b0000_0000_1000_0010,
            present: joints(-1.0),
            commanded: joints(0.5),
        }
    }

    fn goal() -> GoalSetpoint {
        GoalSetpoint {
            execute_at_ns: 1_700_000_000_040_000_000,
            mask: JOINT_MASK_ALL,
            targets: joints(0.25),
        }
    }

    #[test]
    fn lengths_match_the_documented_layout() {
        assert_eq!(POSE_SAMPLE_LEN, 171);
        assert_eq!(DRIVER_EVENT_LEN, 21);
        assert_eq!(GOAL_SETPOINT_LEN, 90);
        assert_eq!(CONTROL_LEN, 9);
    }

    #[test]
    fn every_encoder_produces_its_own_length_and_header() {
        for (bytes, msg_type, len) in [
            (
                sample().encode(1).to_vec(),
                PoseSample::MSG_TYPE,
                POSE_SAMPLE_LEN,
            ),
            (
                DriverEvent {
                    kind: EventKind::BusFailure,
                    detail: 3,
                    time_ns: 5,
                }
                .encode(2)
                .to_vec(),
                DriverEvent::MSG_TYPE,
                DRIVER_EVENT_LEN,
            ),
            (
                goal().encode(3).to_vec(),
                GoalSetpoint::MSG_TYPE,
                GOAL_SETPOINT_LEN,
            ),
            (
                Control {
                    op: ControlOp::TorqueOffNow,
                }
                .encode(4)
                .to_vec(),
                Control::MSG_TYPE,
                CONTROL_LEN,
            ),
        ] {
            assert_eq!(bytes.len(), len);
            assert_eq!(u16::from_le_bytes([bytes[0], bytes[1]]), MAGIC);
            assert_eq!(bytes[2], VERSION);
            assert_eq!(bytes[3], msg_type);
        }
    }

    #[test]
    fn pose_sample_round_trips_every_flag_combination() {
        for bits in 0..8u8 {
            let mut s = sample();
            s.present_valid = bits & 1 != 0;
            s.commanded_valid = bits & 2 != 0;
            s.torque_off_latched = bits & 4 != 0;
            let bytes = s.encode(bits.into());
            let (header, back) = PoseSample::decode(&bytes).expect("own output decodes");
            assert_eq!(
                header,
                Header {
                    msg_type: PoseSample::MSG_TYPE,
                    seq: bits.into()
                }
            );
            assert_eq!(back, s);
            assert_eq!(back.flags(), bits);
        }
    }

    #[test]
    fn pose_sample_round_trips_every_mask() {
        for bit in 0..JOINT_COUNT {
            let mut s = sample();
            s.miss_mask = 1 << bit;
            let (_, back) = PoseSample::decode(&s.encode(0)).expect("own output decodes");
            assert_eq!(back.miss_mask, 1 << bit);
        }
    }

    #[test]
    fn driver_event_round_trips_every_kind() {
        for kind in EventKind::ALL {
            let event = DriverEvent {
                kind,
                detail: u32::MAX,
                time_ns: i64::MIN,
            };
            let (_, back) = DriverEvent::decode(&event.encode(7)).expect("own output decodes");
            assert_eq!(back, event);
        }
    }

    #[test]
    fn goal_setpoint_round_trips_every_mask() {
        for mask in 0..=JOINT_MASK_ALL {
            let g = GoalSetpoint { mask, ..goal() };
            let (_, back) = GoalSetpoint::decode(&g.encode(0)).expect("own output decodes");
            assert_eq!(back, g);
        }
    }

    #[test]
    fn control_round_trips_every_op() {
        for op in ControlOp::ALL {
            let (_, back) = Control::decode(&Control { op }.encode(0)).expect("own output decodes");
            assert_eq!(back.op, op);
        }
    }

    #[test]
    fn a_wrapped_sequence_number_survives() {
        let (header, _) = PoseSample::decode(&sample().encode(u32::MAX)).expect("decodes");
        assert_eq!(header.seq, u32::MAX);
        let next = header.seq.wrapping_add(1);
        let (header, _) = PoseSample::decode(&sample().encode(next)).expect("decodes");
        assert_eq!(header.seq, 0);
    }

    #[test]
    fn non_finite_positions_survive_the_round_trip_bit_for_bit() {
        // The codec is not the layer that decides a NaN is unacceptable. It has
        // to carry one faithfully so the layer that does can see it.
        let mut s = sample();
        s.present[0] = f64::NAN;
        s.present[1] = f64::INFINITY;
        s.present[2] = -0.0;
        let (_, back) = PoseSample::decode(&s.encode(0)).expect("own output decodes");
        assert!(back.present[0].is_nan());
        assert_eq!(back.present[1], f64::INFINITY);
        assert!(back.present[2].is_sign_negative());
    }

    #[test]
    fn the_dispatcher_returns_the_right_variant_for_each_type() {
        let s = sample();
        assert_eq!(
            decode(&s.encode(0)).expect("decodes").1,
            Message::PoseSample(s)
        );
        let e = DriverEvent {
            kind: EventKind::CycleSkipped,
            detail: 1,
            time_ns: 2,
        };
        assert_eq!(
            decode(&e.encode(0)).expect("decodes").1,
            Message::DriverEvent(e)
        );
        let g = goal();
        assert_eq!(
            decode(&g.encode(0)).expect("decodes").1,
            Message::GoalSetpoint(g)
        );
        let c = Control {
            op: ControlOp::TorqueOffNow,
        };
        assert_eq!(
            decode(&c.encode(0)).expect("decodes").1,
            Message::Control(c)
        );
    }

    #[test]
    fn the_dispatcher_refuses_a_type_it_does_not_speak() {
        let mut bytes = sample().encode(0);
        bytes[3] = 0x7f;
        assert_eq!(
            decode(&bytes),
            Err(DecodeError::UnknownType { msg_type: 0x7f })
        );
    }

    #[test]
    fn a_decoder_refuses_a_datagram_of_another_type() {
        let bytes = goal().encode(0);
        assert_eq!(
            PoseSample::decode(&bytes),
            Err(DecodeError::WrongType {
                expected: PoseSample::MSG_TYPE,
                actual: GoalSetpoint::MSG_TYPE
            })
        );
    }

    #[test]
    fn every_truncation_of_every_message_is_refused_and_none_panics() {
        let cases: [(Vec<u8>, u8, usize); 4] = [
            (
                sample().encode(0).to_vec(),
                PoseSample::MSG_TYPE,
                POSE_SAMPLE_LEN,
            ),
            (
                DriverEvent {
                    kind: EventKind::BusFailure,
                    detail: 0,
                    time_ns: 0,
                }
                .encode(0)
                .to_vec(),
                DriverEvent::MSG_TYPE,
                DRIVER_EVENT_LEN,
            ),
            (
                goal().encode(0).to_vec(),
                GoalSetpoint::MSG_TYPE,
                GOAL_SETPOINT_LEN,
            ),
            (
                Control {
                    op: ControlOp::TorqueOffNow,
                }
                .encode(0)
                .to_vec(),
                Control::MSG_TYPE,
                CONTROL_LEN,
            ),
        ];
        for (bytes, msg_type, len) in cases {
            for cut in 0..len {
                let err = decode(&bytes[..cut]).expect_err("a truncated datagram is refused");
                let expected = if cut < HEADER_LEN {
                    DecodeError::TooShort { len: cut }
                } else {
                    DecodeError::WrongLength {
                        msg_type,
                        expected: len,
                        actual: cut,
                    }
                };
                assert_eq!(err, expected, "type {msg_type:#04x} cut at {cut}");
            }
        }
    }

    #[test]
    fn a_datagram_with_trailing_bytes_is_refused() {
        let mut bytes = goal().encode(0).to_vec();
        bytes.push(0);
        assert_eq!(
            decode(&bytes),
            Err(DecodeError::WrongLength {
                msg_type: GoalSetpoint::MSG_TYPE,
                expected: GOAL_SETPOINT_LEN,
                actual: GOAL_SETPOINT_LEN + 1,
            })
        );
    }

    #[test]
    fn reserved_flag_bits_are_refused() {
        let mut bytes = sample().encode(0);
        bytes[HEADER_LEN + 16] |= 0x80;
        assert_eq!(
            PoseSample::decode(&bytes),
            Err(DecodeError::ReservedBits {
                field: "flags",
                bits: 0x80
            })
        );
    }

    #[test]
    fn mask_bits_outside_the_bus_rows_are_refused() {
        let mut bytes = sample().encode(0);
        bytes[HEADER_LEN + 18] = 0x08;
        assert_eq!(
            PoseSample::decode(&bytes),
            Err(DecodeError::ReservedBits {
                field: "miss_mask",
                bits: 0x0800
            })
        );

        let mut bytes = goal().encode(0);
        bytes[HEADER_LEN + 9] = 0x02;
        assert_eq!(
            GoalSetpoint::decode(&bytes),
            Err(DecodeError::ReservedBits {
                field: "mask",
                bits: 0x0200
            })
        );
    }

    #[test]
    fn unknown_enum_values_are_refused() {
        let mut bytes = DriverEvent {
            kind: EventKind::BusFailure,
            detail: 0,
            time_ns: 0,
        }
        .encode(0);
        bytes[HEADER_LEN] = 0;
        assert_eq!(
            DriverEvent::decode(&bytes),
            Err(DecodeError::UnknownEnum {
                field: "kind",
                value: 0
            })
        );
        bytes[HEADER_LEN] = 9;
        assert_eq!(
            DriverEvent::decode(&bytes),
            Err(DecodeError::UnknownEnum {
                field: "kind",
                value: 9
            })
        );

        let mut bytes = Control {
            op: ControlOp::TorqueOffNow,
        }
        .encode(0);
        bytes[HEADER_LEN] = 2;
        assert_eq!(
            Control::decode(&bytes),
            Err(DecodeError::UnknownEnum {
                field: "op",
                value: 2
            })
        );
    }

    #[test]
    fn arbitrary_bytes_never_panic() {
        // A cheap deterministic sweep over the byte space in front of the
        // decoder: no seed, no dependency, and it covers every length class
        // including the exact ones.
        //
        // Most iterations wear a header the dispatcher accepts, because random
        // bytes carry the magic once in 65536 tries and a sweep that never
        // passes `peek_header` is a sweep of two bytes. The remainder are
        // random from byte zero, which is the header path itself. What each
        // per-type decoder was reached with is counted, and the counts are
        // asserted: a decoder this sweep cannot reach is a decoder with no
        // fuzz coverage, and that has to fail here rather than pass silently.
        const TYPES: [u8; 5] = [
            PoseSample::MSG_TYPE,
            DriverEvent::MSG_TYPE,
            GoalSetpoint::MSG_TYPE,
            Control::MSG_TYPE,
            0x7f,
        ];
        let exact = [
            (PoseSample::MSG_TYPE, POSE_SAMPLE_LEN),
            (DriverEvent::MSG_TYPE, DRIVER_EVENT_LEN),
            (GoalSetpoint::MSG_TYPE, GOAL_SETPOINT_LEN),
            (Control::MSG_TYPE, CONTROL_LEN),
        ];
        let mut reached = [0usize; 4];
        let mut state = 0x2545_f491_4f6c_dd1du64;
        let mut buf = [0u8; POSE_SAMPLE_LEN + 4];
        for iteration in 0..2000usize {
            for slot in &mut buf {
                state ^= state << 13;
                state ^= state >> 7;
                state ^= state << 17;
                *slot = (state >> 24) as u8;
            }
            // One iteration in eight stays random all the way to byte zero.
            if iteration % 8 != 0 {
                buf[..2].copy_from_slice(&MAGIC.to_le_bytes());
                buf[2] = VERSION;
                buf[3] = TYPES[iteration % TYPES.len()];
            }
            for len in [
                0,
                1,
                7,
                8,
                9,
                CONTROL_LEN,
                DRIVER_EVENT_LEN,
                GOAL_SETPOINT_LEN,
                POSE_SAMPLE_LEN,
                POSE_SAMPLE_LEN + 4,
            ] {
                let outcome = decode(&buf[..len]);
                let header_refused = matches!(
                    outcome,
                    Err(DecodeError::TooShort { .. }
                        | DecodeError::BadMagic { .. }
                        | DecodeError::BadVersion { .. }
                        | DecodeError::UnknownType { .. })
                );
                if header_refused {
                    continue;
                }
                for (slot, (msg_type, exact_len)) in reached.iter_mut().zip(exact) {
                    if buf[3] == msg_type && len == exact_len {
                        *slot += 1;
                    }
                }
            }
        }
        for (count, (msg_type, _)) in reached.iter().zip(exact) {
            assert!(
                *count > 0,
                "no fuzz input reached the body decoder for type {msg_type:#04x}"
            );
        }
    }

    #[test]
    fn a_valid_datagram_with_one_byte_flipped_is_still_never_a_panic() {
        // Every type, not only the longest one: each body decoder reads its own
        // fields with its own lengths and enums.
        let mut datagrams: Vec<Vec<u8>> = vec![
            sample().encode(3).to_vec(),
            goal().encode(3).to_vec(),
            Control {
                op: ControlOp::TorqueOffNow,
            }
            .encode(3)
            .to_vec(),
        ];
        for kind in EventKind::ALL {
            datagrams.push(
                DriverEvent {
                    kind,
                    detail: 7,
                    time_ns: -1,
                }
                .encode(3)
                .to_vec(),
            );
        }
        for bytes in datagrams {
            for i in 0..bytes.len() {
                for bit in 0..8 {
                    let mut corrupt = bytes.clone();
                    corrupt[i] ^= 1 << bit;
                    let _ = decode(&corrupt);
                }
            }
        }
    }

    #[test]
    fn event_kind_values_are_the_documented_ones() {
        assert_eq!(EventKind::HoldTimeoutTorqueOff.as_u8(), 1);
        assert_eq!(EventKind::BusFailure.as_u8(), 8);
        for (i, kind) in EventKind::ALL.iter().enumerate() {
            assert_eq!(kind.as_u8() as usize, i + 1);
            assert_eq!(EventKind::from_u8(kind.as_u8()), Some(*kind));
        }
        assert_eq!(EventKind::from_u8(0), None);
        assert_eq!(EventKind::from_u8(9), None);
        assert_eq!(ControlOp::from_u8(1), Some(ControlOp::TorqueOffNow));
        assert_eq!(ControlOp::from_u8(0), None);
    }

    /// What a sender encodes, a receiver decodes — including at the edges of
    /// every field's domain. An encoder that accepted a value its own decoder
    /// refuses would put undecodable datagrams on the sample stream the whole
    /// control loop clocks off.
    #[test]
    fn every_extreme_a_field_can_hold_survives_its_own_decoder() {
        let sample = PoseSample {
            nominal_time_ns: i64::MIN,
            sample_time_ns: i64::MAX,
            present_valid: true,
            commanded_valid: true,
            torque_off_latched: true,
            miss_mask: JOINT_MASK_ALL,
            present: [f64::MIN; JOINT_COUNT],
            commanded: [f64::MAX; JOINT_COUNT],
        };
        assert_eq!(
            PoseSample::decode(&sample.encode(u32::MAX)),
            Ok((
                Header {
                    msg_type: PoseSample::MSG_TYPE,
                    seq: u32::MAX
                },
                sample
            ))
        );

        let goal = GoalSetpoint {
            execute_at_ns: i64::MAX,
            mask: JOINT_MASK_ALL,
            targets: [f64::MIN; JOINT_COUNT],
        };
        assert_eq!(
            GoalSetpoint::decode(&goal.encode(u32::MAX)),
            Ok((
                Header {
                    msg_type: GoalSetpoint::MSG_TYPE,
                    seq: u32::MAX
                },
                goal
            ))
        );
    }

    /// "Every row" written as `u16::MAX` rather than the nine rows: a debug
    /// build stops the caller, and the datagram a release build sends is still
    /// one a receiver accepts, rather than a sample silently dropped for the
    /// whole window the mistake covers.
    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "miss_mask has bits outside the nine bus rows")]
    fn a_miss_mask_outside_the_bus_rows_stops_the_sender() {
        let _ = PoseSample {
            miss_mask: u16::MAX,
            ..sample()
        }
        .encode(1);
    }

    #[test]
    #[cfg(debug_assertions)]
    #[should_panic(expected = "mask has bits outside the nine bus rows")]
    fn a_goal_mask_outside_the_bus_rows_stops_the_sender() {
        let _ = GoalSetpoint {
            mask: 0x0400,
            ..goal()
        }
        .encode(1);
    }
}
