//! The out-of-band half of a cycle: one transaction against the real wire.
//!
//! A driver cycle spends whatever bus time is left over from the grouped read
//! and the goal write on exactly one of these, and [`reachy_driver::AuxSlot`]
//! is what picks which — a host request first, then the torque-off
//! confirmation's read-back, then the health rotation. What is here is the
//! other half: running the one it picked, and saying what came back in the same
//! outcome record the simulated driver publishes on its channel.
//!
//! This module is the wire and nothing else. Which transaction runs, what a
//! confirmation pass makes of a read-back and what the driver believes about
//! torque are all [`reachy_driver`]'s, hosted by [`crate::tick`] — the same
//! decisions the simulated driver hosts next to its own plant, so the two
//! cannot disagree about what a de-torquing that read back as still holding
//! means.
//!
//! Every function below is one round trip and no retries: a transaction that
//! did not complete is an outcome saying how it failed, and the host's own
//! re-issue is the retry mechanism this seam has. Nothing here sleeps and
//! nothing here reads a clock.

use brenn_reachy__driver__health_clk_rs::{AuxOutcome, AuxStatus, HealthReport};
use brenn_reachy__hardware__dynamixel__registers_clk_rs::{RegId, ValueShape};
use brenn_reachy__motion__bus_txn_clk_rs::{AuxOpKind, BusTxn};
use clockwork_rs::SyncTime;
use dxl_proto::regs::{
    GOAL_POSITION, HARDWARE_ERROR_STATUS, PRESENT_INPUT_VOLTAGE, PRESENT_POSITION,
    PRESENT_TEMPERATURE, TORQUE_ENABLE,
};
use reachy_bus::{Bus, BusPort, RawValue, ServoMap, XactError, named_reg, value_kind};
use reachy_motion::arm::row_of_id;
use reachy_motion::joints::ROW_COUNT;
use reachy_motion::value::{self, Value};

/// One transaction, copied out of the record the slot holds.
///
/// Copied rather than borrowed for the reason the simulated driver copies it:
/// the fields are five numbers, and holding the slot's record open across the
/// bus transaction it describes would borrow the state the answer has to be
/// written into.
#[derive(Clone, Copy, Debug)]
pub struct Request {
    /// Which transaction.
    pub op: AuxOpKind,
    /// Which servo, as its bus id.
    pub id: u8,
    /// Which register, or the no-register zero.
    pub reg: RegId,
    /// What shape the value is, as the host built it.
    pub value_kind: ValueShape,
    /// The value, as eight little-endian bytes.
    pub value: u64,
}

impl Request {
    /// The transaction a slot handed over.
    #[must_use]
    pub fn of(txn: &BusTxn) -> Self {
        Self {
            op: txn.op,
            id: txn.id,
            reg: txn.reg,
            value_kind: txn.value_kind,
            value: txn.value,
        }
    }
}

/// What the machine answered, before it reaches the outcome record.
///
/// Held as ordinary Rust for the reason a cycle's event is: the cycle decides
/// what to publish and then publishes it once, rather than writing into a
/// message as it goes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Answer {
    /// The correlation number of the request this answers.
    pub corr: u32,
    /// How it went.
    pub status: AuxStatus,
    /// What shape the value is, and `none` where there is no register value to
    /// carry.
    pub value_kind: ValueShape,
    /// The value, as eight little-endian bytes.
    pub value: u64,
    /// The model number a ping answered with, and zero for everything else.
    pub model: u16,
}

impl Answer {
    /// An answer carrying no register value: a ping's, a refusal's, a
    /// silence's.
    fn bare(corr: u32, status: AuxStatus) -> Self {
        Self {
            corr,
            status,
            value_kind: ValueShape::None,
            value: 0,
            model: 0,
        }
    }

    /// The driver declining to run a transaction, so nothing reached the bus.
    ///
    /// What a malformed request gets: a transaction naming no register, a value
    /// built in a shape the register does not take, a value the register cannot
    /// carry, or an id this machine does not have. Nothing about the machine
    /// changed, which is the whole difference between this and a servo that
    /// answered badly.
    fn refused(corr: u32) -> Self {
        Self::bare(corr, AuxStatus::Refused)
    }

    /// The driver turning a request away because one is already pending.
    ///
    /// Against the turned-away request's own correlation number, so the host
    /// learns which of its two requests was not run rather than having to work
    /// it out from a silence.
    #[must_use]
    pub fn busy(corr: u32) -> Self {
        Self::bare(corr, AuxStatus::Refused)
    }

    /// A value read off a register.
    fn value(corr: u32, held: Value) -> Self {
        Self {
            corr,
            status: AuxStatus::Ok,
            value_kind: held.shape(),
            value: held.bits(),
            model: 0,
        }
    }

    /// Write this answer into the outcome record that carries it.
    pub fn write(&self, out: &mut AuxOutcome) {
        out.corr = self.corr;
        out.status = self.status;
        out.value_kind = self.value_kind;
        out.value = self.value;
        out.model = self.model;
    }
}

/// Run one host transaction against the machine.
///
/// The three transactions the vocabulary has: does something at this id answer,
/// read one register, write one register and read it back. The last is the only
/// one that changes anything, and it is verified by the bus layer itself — an
/// unverified write says nothing about a machine, and a belief built out of one
/// would make the dead-man measure the driver's own sends.
pub fn answer<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    corr: u32,
    request: &Request,
) -> Answer {
    let Some(row) = row_of_id(request.id) else {
        // An id this driver does not address. Refused rather than put on the
        // wire: the map is the machine's own wiring, and a transaction naming
        // something else is a request about a different machine — nothing
        // reached the bus, which is what a refusal says.
        return Answer::refused(corr);
    };
    match request.op {
        // A slot nothing wrote asks for no transaction, and putting a datagram
        // on the bus for it would be commanding a machine on the strength of
        // unwritten memory.
        AuxOpKind::None => Answer::refused(corr),
        AuxOpKind::Ping => match bus.ping(request.id) {
            Ok(info) => Answer {
                model: info.model,
                ..Answer::bare(corr, AuxStatus::Ok)
            },
            Err(failure) => failed(corr, map, row, request.reg, &failure),
        },
        AuxOpKind::ReadReg => read_reg(bus, map, corr, row, request),
        AuxOpKind::WriteRegVerified => write_verified(bus, map, corr, row, request),
    }
}

/// Read one register and answer with what it holds.
fn read_reg<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    corr: u32,
    row: usize,
    request: &Request,
) -> Answer {
    let Ok(reg) = reachy_bus::reg_for(request.reg) else {
        return Answer::refused(corr);
    };
    match bus.read_reg(request.id, reg) {
        Ok(raw) => match map.decode_value(row, request.reg, &raw) {
            Ok(held) => Answer::value(corr, held),
            // Bytes of the register's own width that do not read as its shape.
            // Nothing this build produces, and a decode failure is the honest
            // reading of it rather than a value nobody can trust.
            Err(_) => Answer::bare(corr, AuxStatus::DecodeError),
        },
        Err(failure) => failed(corr, map, row, request.reg, &failure),
    }
}

/// Write one register and read it back, so the answer is what the servo holds
/// rather than what was sent.
fn write_verified<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    corr: u32,
    row: usize,
    request: &Request,
) -> Answer {
    let Ok(shape) = value_kind(request.reg) else {
        return Answer::refused(corr);
    };
    if shape != request.value_kind {
        // The host built the value in a shape the register does not take, which
        // is a request that cannot be run rather than a servo that answered
        // badly.
        return Answer::refused(corr);
    }
    let held = value::carried(shape, request.value);
    let (Ok(reg), Ok(raw)) = (
        reachy_bus::reg_for(request.reg),
        map.encode_value(row, request.reg, held),
    ) else {
        // A value no wire representation covers: an angle no count reaches, a
        // voltage no register unit reaches. Refused rather than encoded to the
        // nearest thing the wire has — the nearest thing is a different
        // command.
        return Answer::refused(corr);
    };
    match bus.write_reg_verified(request.id, reg, &raw) {
        // The bus layer's own read-back matched what went out, so what the
        // servo holds is the value the request carried.
        Ok(()) => Answer::value(corr, held),
        Err(failure) => failed(corr, map, row, request.reg, &failure),
    }
}

/// The outcome a failed transaction carries.
///
/// Every arm of the bus layer's failure vocabulary reaches exactly one status,
/// and the two that carry evidence carry it: a servo's own error code, and what
/// a verified write read back instead. The rest are the driver declining to run
/// a request — nothing reached the wire — which is a refusal and not a machine
/// that misbehaved.
fn failed(corr: u32, map: &ServoMap, row: usize, reg: RegId, failure: &XactError) -> Answer {
    match failure {
        XactError::Timeout { .. } => Answer::bare(corr, AuxStatus::Timeout),
        XactError::Corrupt { .. } | XactError::ShortReply { .. } | XactError::LongReply { .. } => {
            Answer::bare(corr, AuxStatus::DecodeError)
        }
        XactError::ServoError { error, .. } => Answer {
            // The code the servo gave, which is a number and not a register
            // value — so the shape stays `none` and the bits are the code.
            value: u64::from(error.0),
            ..Answer::bare(corr, AuxStatus::ServoError)
        },
        XactError::VerifyMismatch { read_back, .. } => {
            let status = AuxStatus::VerifyMismatch;
            match map.decode_value(row, reg, read_back) {
                Ok(held) => Answer {
                    status,
                    ..Answer::value(corr, held)
                },
                Err(_) => Answer::bare(corr, status),
            }
        }
        XactError::Io { .. } => Answer::bare(corr, AuxStatus::WireError),
        // The driver declining: a non-volatile register, a servo holding torque
        // where the write needed it released, a value of the wrong width, a
        // register wider than the wire carries, a frame that would not encode,
        // more ids than one frame holds. None of them put anything on the bus.
        XactError::EepromRefused { .. }
        | XactError::TorqueHeld { .. }
        | XactError::ValueWidth { .. }
        | XactError::RegisterTooWide { .. }
        | XactError::Encode { .. }
        | XactError::TooManyIds { .. } => Answer::refused(corr),
    }
}

/// Whether this row's torque-enable register still reads as energised.
///
/// What the confirmation pass is fed. Every failure counts as still holding:
/// a servo that did not answer has not been *seen* to go limp, and a
/// de-torquing credited to silence is the one report this driver must never
/// make. A register that answered something other than a byte reads the same
/// way and for the same reason.
///
/// The second half of the pair says whether the exchange completed, which is
/// what a caller counts — the reading is deliberately identical for a servo
/// that said "still on" and one that said nothing at all.
pub fn reads_torqued<P: BusPort>(bus: &mut Bus<P>, id: u8) -> (bool, bool) {
    match bus.read_reg(id, named_reg(RegId::TorqueEnable)) {
        Ok(raw) => (raw.u8().is_none_or(|held| held != 0), true),
        Err(_) => (true, false),
    }
}

/// One servo's health, as the rotation's read of its status registers.
///
/// Three reads, all of registers a host could read itself, so what a report says
/// is what an operator reading that control table would find. Answers whether
/// the report is worth publishing: a servo that did not answer all three gets no
/// report at all rather than a report of zeroes about a machine nobody heard
/// from — the rotation walks on either way, because the cadence was stamped when
/// the read was named.
///
/// TODO(health-read-budget): a read the machine did not answer costs the
/// rotation one report and nothing else. Whether a run of them should stop the
/// loop the way a run of missed position reads does, and after how many, is
/// undecided — the one detection of a latched overload is here, and the cost of
/// a wrong answer is a session ended over a bus that stutters.
///
/// The temperature is the servo's own present-temperature cell, whole degrees
/// Celsius. The record carries it as an `Int8`, so a byte above 127 is not a
/// reading this report can state and fails the task exactly as an unanswered
/// read does: the standing evidence of an overheating servo is the latched
/// error byte, not a number bent to fit.
pub fn health<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    sample_time_ns: i64,
    out: &mut HealthReport,
) -> bool {
    let Some(id) = map.id_at(row) else {
        return false;
    };
    let Ok(bits) = bus
        .read_reg(id, named_reg(RegId::HardwareErrorStatus))
        .map(|raw| raw.u8().unwrap_or(0))
    else {
        return false;
    };
    let Ok(volts) = bus
        .read_reg(id, named_reg(RegId::PresentInputVoltage))
        .map(|raw| raw.u16().map_or(0.0, dxl_proto::volts_from_raw))
    else {
        return false;
    };
    let Ok(Some(temp_c)) = bus
        .read_reg(id, named_reg(RegId::PresentTemperature))
        .map(|raw| raw.u8().and_then(|degrees| i8::try_from(degrees).ok()))
    else {
        return false;
    };
    out.id = id;
    out.bits = bits;
    out.volts = volts;
    out.temp_c = temp_c;
    out.sample_time = SyncTime::from_nanos(sample_time_ns);
    true
}

/// What each part of a cycle's bus work can cost at worst, nanoseconds.
///
/// Bounds and not measurements, and they do not need to be measurements: every
/// exchange in [`reachy_bus`] runs under a deadline that crate computes, and
/// each figure here is that same deadline asked for ahead of time. They come off
/// [`reachy_bus::BusTiming`] rather than being re-derived from a guessed frame
/// size, so a change to the framing, the status overhead or the servo count
/// moves the budget and the enforced deadline together.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CycleBounds {
    /// The grouped read of the nine present positions: one instruction frame
    /// out, nine status frames back.
    pub read_ns: i64,
    /// The grouped write of the nine goal positions, which the protocol
    /// acknowledges with nothing at all.
    pub write_ns: i64,
    /// The torque-off sweep: a verified write per row, which is a write and a
    /// read-back each.
    pub sweep_ns: i64,
    /// The most expensive out-of-band transaction: a verified write of the
    /// widest value the wire carries, or the health rotation's three reads.
    ///
    /// The host's transaction is charged the widest value because the register
    /// it names is the host's choice; the rotation is charged the three cells it
    /// actually reads, whose widths are fixed here.
    pub aux_ns: i64,
}

impl CycleBounds {
    /// The bounds of a cycle on `bus_timing`.
    #[must_use]
    pub fn of(bus_timing: &reachy_bus::BusTiming) -> Self {
        let widest = RawValue::MAX_LEN;
        let verified = bus_timing.verified_write_bound(widest);
        let health_reads = bus_timing.read_reg_bound(HARDWARE_ERROR_STATUS.len.into())
            + bus_timing.read_reg_bound(PRESENT_INPUT_VOLTAGE.len.into())
            + bus_timing.read_reg_bound(PRESENT_TEMPERATURE.len.into());
        Self {
            read_ns: nanos(bus_timing.sync_read_bound(ROW_COUNT, PRESENT_POSITION.len.into())),
            write_ns: nanos(bus_timing.sync_write_bound(ROW_COUNT, GOAL_POSITION.len.into())),
            sweep_ns: nanos(bus_timing.verified_write_bound(TORQUE_ENABLE.len.into()))
                .saturating_mul(ROW_COUNT as i64),
            aux_ns: nanos(verified.max(health_reads)),
        }
    }
}

/// A duration as nanoseconds, saturating rather than wrapping: a bound that
/// overflowed would be a budget that admits everything.
fn nanos(span: std::time::Duration) -> i64 {
    i64::try_from(span.as_nanos()).unwrap_or(i64::MAX)
}

#[cfg(test)]
mod tests {
    use super::{Answer, CycleBounds, Request, answer, failed};
    use brenn_reachy__driver__health_clk_rs::AuxStatus;
    use brenn_reachy__hardware__dynamixel__registers_clk_rs::{RegId, ValueShape};
    use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKind;
    use dxl_proto::{EncodeError, FrameError, StatusError};
    use reachy_bus::{Bus, BusPort, BusTiming, RawValue, ServoMap, XactError};
    use reachy_motion::arm::SERVO_IDS;
    use reachy_motion::value;
    use std::cell::Cell;
    use std::io;
    use std::rc::Rc;
    use std::time::Instant;

    /// The correlation number every case answers against, so an answer built for
    /// a different request is visible.
    const CORR: u32 = 77;

    /// The row and the id these cases classify a failure against.
    const ROW: usize = 0;

    fn map() -> ServoMap {
        ServoMap::new(SERVO_IDS)
    }

    /// How `failure` is classified, against a four-byte position register.
    fn status_of(failure: &XactError) -> AuxStatus {
        failed(CORR, &map(), ROW, RegId::PresentPosition, failure).status
    }

    fn raw(bytes: &[u8]) -> RawValue {
        RawValue::new(bytes).expect("a value inside the wire's width")
    }

    /// A port that answers nothing and remembers everything sent to it, so a
    /// case about a refusal can say that nothing reached the wire — which is the
    /// whole difference between a refusal and a servo that misbehaved.
    #[derive(Clone, Default)]
    struct Recorder {
        sent: Rc<Cell<usize>>,
    }

    impl BusPort for Recorder {
        fn write_all(&mut self, _buf: &[u8]) -> io::Result<()> {
            self.sent.set(self.sent.get() + 1);
            Ok(())
        }

        fn read_some(&mut self, _buf: &mut [u8], _deadline: Instant) -> io::Result<usize> {
            Ok(0)
        }

        fn discard_input(&mut self) -> io::Result<()> {
            Ok(())
        }
    }

    /// A request as a host would have written it into the slot.
    fn asking(op: AuxOpKind, id: u8, reg: RegId, held: value::Value) -> Request {
        Request {
            op,
            id,
            reg,
            value_kind: held.shape(),
            value: held.bits(),
        }
    }

    /// What [`answer`] makes of `request`, and how many frames it put on the
    /// wire doing it.
    fn ran(request: &Request) -> (Answer, usize) {
        // A timing with no host allowance, so a case that does reach the wire
        // fails on an answer rather than on a wall-clock deadline.
        let timing = BusTiming {
            host_allowance: std::time::Duration::ZERO,
            retry_spacing: std::time::Duration::ZERO,
            ..BusTiming::default()
        };
        let port = Recorder::default();
        let sent = Rc::clone(&port.sent);
        let mut bus = Bus::new(port, timing);
        let answer = answer(&mut bus, &map(), CORR, request);
        (answer, sent.get())
    }

    #[test]
    fn a_request_the_driver_cannot_run_reaches_no_servo() {
        // Every arm that refuses before the wire: a slot nobody wrote, an id
        // this machine does not address, a transaction naming no register, a
        // value built in a shape the register does not take, and a value no wire
        // representation covers.
        let cases = [
            (
                "a slot nobody wrote",
                asking(AuxOpKind::None, SERVO_IDS[0], RegId::None, value::NONE),
            ),
            (
                "an id this machine does not have",
                asking(AuxOpKind::ReadReg, 200, RegId::PresentPosition, value::NONE),
            ),
            (
                "a read naming no register",
                asking(AuxOpKind::ReadReg, SERVO_IDS[0], RegId::None, value::NONE),
            ),
            (
                "a write naming no register",
                asking(
                    AuxOpKind::WriteRegVerified,
                    SERVO_IDS[0],
                    RegId::None,
                    value::u8(1),
                ),
            ),
            (
                "a value in a shape the register does not take",
                asking(
                    AuxOpKind::WriteRegVerified,
                    SERVO_IDS[0],
                    RegId::TorqueEnable,
                    value::radians(0.5),
                ),
            ),
            (
                "an angle no servo count reaches",
                asking(
                    AuxOpKind::WriteRegVerified,
                    SERVO_IDS[0],
                    RegId::GoalPosition,
                    value::radians(f64::from(i32::MAX)),
                ),
            ),
        ];

        for (what, request) in cases {
            let (answer, sent) = ran(&request);
            assert_eq!(answer.corr, CORR, "{what}");
            assert_eq!(answer.status, AuxStatus::Refused, "{what}");
            assert_eq!(answer.value_kind, ValueShape::None, "{what}");
            assert_eq!(answer.value, 0, "{what}");
            assert_eq!(sent, 0, "{what}: nothing reached the wire");
        }
    }

    #[test]
    fn a_silence_is_a_timeout_and_not_a_refusal() {
        assert_eq!(
            status_of(&XactError::Timeout {
                id: SERVO_IDS[ROW],
                waited: std::time::Duration::from_millis(3),
            }),
            AuxStatus::Timeout,
            "the frame went out, so the host cannot assume the machine is untouched"
        );
    }

    #[test]
    fn bytes_that_did_not_read_as_a_frame_are_a_decode_error() {
        for failure in [
            XactError::Corrupt {
                id: SERVO_IDS[ROW],
                cause: FrameError::BadCrc,
            },
            XactError::ShortReply {
                id: SERVO_IDS[ROW],
                expected: 4,
                actual: 2,
            },
            XactError::LongReply {
                id: SERVO_IDS[ROW],
                expected: 4,
                actual: 6,
            },
        ] {
            assert_eq!(
                status_of(&failure),
                AuxStatus::DecodeError,
                "{failure:?} is bytes nobody can trust, not a refusal"
            );
        }
    }

    #[test]
    fn a_servos_own_error_code_travels_in_the_value_with_no_shape() {
        let answer = failed(
            CORR,
            &map(),
            ROW,
            RegId::PresentPosition,
            &XactError::ServoError {
                id: SERVO_IDS[ROW],
                error: StatusError(0x87),
            },
        );

        assert_eq!(answer.status, AuxStatus::ServoError);
        // The code is a number and not a register value, so the shape stays
        // `none` and a host reading `value` gets the byte the servo sent.
        assert_eq!(answer.value_kind, ValueShape::None);
        assert_eq!(answer.value, 0x87);
        assert_eq!(answer.corr, CORR);
    }

    #[test]
    fn a_read_back_that_decoded_carries_what_the_servo_still_holds() {
        let answer = failed(
            CORR,
            &map(),
            ROW,
            RegId::PresentPosition,
            &XactError::VerifyMismatch {
                id: SERVO_IDS[ROW],
                addr: 116,
                wrote: raw(&2100i32.to_le_bytes()),
                read_back: raw(&2048i32.to_le_bytes()),
            },
        );

        assert_eq!(answer.status, AuxStatus::VerifyMismatch);
        assert_eq!(
            answer.value_kind,
            ValueShape::Radians,
            "the register's own shape, so the host reads an angle and not counts"
        );
        let expected = map()
            .present_rad(ROW, 2048)
            .expect("row 0 is a servo on this bus");
        assert_eq!(f64::from_bits(answer.value), expected);
    }

    #[test]
    fn a_read_back_that_did_not_decode_is_still_a_mismatch_carrying_nothing() {
        let answer = failed(
            CORR,
            &map(),
            ROW,
            RegId::PresentPosition,
            &XactError::VerifyMismatch {
                id: SERVO_IDS[ROW],
                addr: 116,
                wrote: raw(&2100i32.to_le_bytes()),
                // One byte where the register is four: bytes of the wrong width
                // for the shape the register takes.
                read_back: raw(&[9]),
            },
        );

        assert_eq!(
            answer.status,
            AuxStatus::VerifyMismatch,
            "what the write did is still known: it did not take"
        );
        assert_eq!(
            answer.value_kind,
            ValueShape::None,
            "and there is no value to carry rather than one nobody can trust"
        );
        assert_eq!(answer.value, 0);
    }

    #[test]
    fn the_port_itself_failing_is_a_wire_error() {
        assert_eq!(
            status_of(&XactError::Io {
                id: SERVO_IDS[ROW],
                source: io::Error::other("the port went away"),
            }),
            AuxStatus::WireError,
        );
    }

    #[test]
    fn everything_the_bus_layer_declined_to_send_is_a_refusal() {
        // Nothing in this group reached the wire, which is the whole difference
        // between a refusal and a machine that answered badly: a host reading
        // one of these knows the servo was not written.
        for failure in [
            XactError::EepromRefused {
                id: SERVO_IDS[ROW],
                addr: 8,
            },
            XactError::TorqueHeld {
                id: SERVO_IDS[ROW],
                addr: 8,
            },
            XactError::ValueWidth {
                id: SERVO_IDS[ROW],
                addr: 116,
                expected: 4,
                actual: 2,
            },
            XactError::RegisterTooWide {
                id: SERVO_IDS[ROW],
                addr: 116,
                len: 8,
                max: RawValue::MAX_LEN,
            },
            XactError::Encode {
                id: SERVO_IDS[ROW],
                source: EncodeError::BroadcastNotAllowed,
            },
            XactError::TooManyIds { count: 10, max: 9 },
        ] {
            let answer = failed(CORR, &map(), ROW, RegId::PresentPosition, &failure);
            assert_eq!(
                answer.status,
                AuxStatus::Refused,
                "{failure:?} put nothing on the bus"
            );
            assert_eq!(answer.value_kind, ValueShape::None);
            assert_eq!(answer.value, 0);
        }
    }

    #[test]
    fn a_turned_away_request_answers_against_its_own_number() {
        let busy = Answer::busy(CORR);

        assert_eq!(busy.status, AuxStatus::Refused);
        assert_eq!(busy.corr, CORR, "which of the two requests was not run");
        assert_eq!(busy.value_kind, ValueShape::None);
        assert_eq!(busy.model, 0);
    }

    #[test]
    fn a_cycle_of_the_shipped_bus_has_room_for_one_out_of_band_transaction() {
        let timing = crate::tick::cycle_timing(reachy_bus::DEFAULT_BAUD);
        let bounds = CycleBounds::of(&timing);

        // The grouped read is the expensive exchange of a cycle: nine status
        // frames back against one instruction frame out, which is what makes it
        // dearer than the unacknowledged grouped write.
        assert!(
            bounds.read_ns > bounds.write_ns,
            "nine replies cost more than none: {bounds:?}"
        );
        // The sweep is a verified write per row and overruns the period on its
        // own, which is why a swept cycle has no room for surveillance.
        assert!(
            bounds.sweep_ns > reachy_driver::NOMINAL_CYCLE_NS,
            "{bounds:?}"
        );
        assert!(
            bounds.read_ns + bounds.write_ns + bounds.aux_ns < reachy_driver::NOMINAL_CYCLE_NS,
            "an ordinary cycle has room for one transaction: {bounds:?}"
        );
    }

    #[test]
    fn the_bounds_are_the_deadlines_the_bus_layer_enforces() {
        let timing = BusTiming::default();

        // Not a restatement of the arithmetic: the point is that a budget asks
        // the timing that computes the deadline, so a change to the framing or
        // the status overhead moves both together.
        assert_eq!(
            timing.verified_write_bound(4),
            timing.write_reg_bound(4) + timing.read_reg_bound(4),
        );
        assert!(
            timing.sync_read_bound(9, 4) > timing.read_reg_bound(4),
            "nine rows answered cost more than one"
        );
        assert!(
            timing.sync_write_bound(9, 4) < timing.sync_read_bound(9, 4),
            "a grouped write is acknowledged by nothing at all"
        );
    }
}
