//! Sequencer framework: the multi-transaction procedures, as state machines that
//! never touch a port.
//!
//! Arming and disarming are not one read and one write. Arming verifies nine
//! servos' provisioning register by register, waits for the supply rail, checks
//! that every limp servo's goal register is mirroring where the joint already
//! is, and only then enables torque and writes goals — dozens of
//! transactions whose order is the safety property. A state machine that yields
//! one abstract request at a time keeps that order in one readable place and lets
//! the whole procedure be tested against scripted replies, with no hardware and
//! no port.
//!
//! ## The driver shape
//!
//! A sequencer is driven by whoever owns the port: hand it the previous
//! transaction's result, take the next action, execute it, repeat. [`SeqAction`]
//! is what comes back — one request to run, a deadline to wait until, a summary
//! on success, or a typed failure. Nothing here sleeps, retries, or decides how
//! long a transaction may take; the driver owns all of that.
//!
//! ## Unicast only
//!
//! Every transaction a sequencer emits names one servo. Arming's writes must each be
//! acknowledged and read back, because a broadcast write that silently fails to
//! apply is exactly the failure arming exists to catch, and its reads are
//! per-servo verdicts rather than one aggregate. The grouped traffic — reading
//! nine positions, writing nine goals — belongs to the tick's own path and never
//! passes through a sequencer.
//!
//! ## What a failure has to say
//!
//! This is the machine's bring-up output, so a failure names the step, the servo,
//! the register, and both the expected and the observed value. A sequencer that
//! reported only "arming failed" would send a person back to the bus with an
//! oscilloscope.

use core::fmt;
use core::time::Duration;

use reachy_kin::FkError;
use thiserror::Error;

use crate::joints::{JointRef, Name, ROW_COUNT};
use crate::txn::BusTxnWire;
use crate::value::{ShapeName, Shown, Value, ValueShape};

/// A register, and the operator's word for it.
///
/// The register is the vocabulary's: one enum, declared in
/// `hardware/dynamixel/registers.clk`, so the number that crosses a slot and the
/// name this crate matches on are one thing. Addresses, widths and byte order
/// belong to the layer that owns the wire, and the mapping is its table.
pub mod reg {
    pub use brenn_reachy__hardware__dynamixel__registers_clk_rs::RegId;

    vocab_name! {
        /// A register as a bring-up report says it.
        ///
        /// The generated `Display` renders the variant's name, which is a
        /// diagnostic spelling and not what an operator reads off a failure line.
        pub struct Name(RegId) {
            RegId::None => "no register",
            RegId::TorqueEnable => "torque enable",
            RegId::GoalPosition => "goal position",
            RegId::PresentPosition => "present position",
            RegId::OperatingMode => "operating mode",
            RegId::HomingOffset => "homing offset",
            RegId::ReturnDelayTime => "return delay time",
            RegId::MinPositionLimit => "minimum position limit",
            RegId::MaxPositionLimit => "maximum position limit",
            RegId::Shutdown => "shutdown mask",
            RegId::DriveMode => "drive mode",
            RegId::MaxVoltageLimit => "maximum voltage limit",
            RegId::MinVoltageLimit => "minimum voltage limit",
            RegId::CurrentLimit => "current limit",
            RegId::VelocityLimit => "velocity limit",
            RegId::TemperatureLimit => "temperature limit",
            RegId::BusWatchdog => "bus watchdog",
            RegId::ProfileAcceleration => "profile acceleration",
            RegId::ProfileVelocity => "profile velocity",
            RegId::PositionGains => "position gains",
            RegId::HardwareErrorStatus => "hardware error status",
            RegId::PresentInputVoltage => "present input voltage",
            RegId::ModelNumber => "model number",
        }
    }

    /// Every register the vocabulary names, the no-register zero excluded: that
    /// one names no control-table entry, and a sweep over registers does not mean
    /// it.
    pub fn named() -> impl Iterator<Item = RegId> {
        crate::vocab::without_zero(RegId::VARIANTS)
    }
}

pub use reg::RegId;

/// Which servos did not answer.
///
/// A set rather than one ID: nine servos are pinged before anything is decided,
/// so a report names every silent one. Two silent servos and nine silent servos
/// are different observations and read differently.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AbsentSet {
    ids: [u8; ROW_COUNT],
    count: usize,
}

impl AbsentSet {
    /// The set of IDs among `ids` whose corresponding `absent` flag is set.
    #[must_use]
    pub fn new(ids: &[u8; ROW_COUNT], absent: &[bool; ROW_COUNT]) -> Self {
        let mut set = Self {
            ids: [0; ROW_COUNT],
            count: 0,
        };
        for (id, missing) in ids.iter().zip(absent) {
            if *missing {
                set.ids[set.count] = *id;
                set.count += 1;
            }
        }
        set
    }

    /// How many servos are in the set.
    #[must_use]
    pub fn count(&self) -> usize {
        self.count
    }

    /// The IDs, in bus order.
    #[must_use]
    pub fn ids(&self) -> &[u8] {
        &self.ids[..self.count]
    }
}

impl fmt::Display for AbsentSet {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        // The whole bus being silent is stated as itself. It is one observation
        // with many possible causes — supply, wiring, adapter, baud, the whole
        // bus held by something else — and naming a cause here would be a guess
        // printed as a finding.
        if self.count == ROW_COUNT {
            return f.write_str("all nine servos");
        }
        if self.count == 1 {
            return write!(f, "servo {}", self.ids[0]);
        }
        if self.count == 0 {
            return f.write_str("no servos");
        }
        f.write_str("servos ")?;
        for (position, id) in self.ids().iter().enumerate() {
            if position > 0 {
                f.write_str(", ")?;
            }
            write!(f, "{id}")?;
        }
        Ok(())
    }
}

/// How a transaction came out.
///
/// A silent servo, a servo's refusal, a driver that would not run the
/// transaction, a mismatched read-back and a corrupt frame are five different
/// things, and every one of them is reported as itself. The driver has already
/// spent whatever retry budget applies to the ones worth retrying.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BusResult {
    /// The servo answered a ping, and said what it is.
    Pinged {
        /// Its model number.
        model: u16,
    },
    /// The read returned this.
    Value(Value),
    /// The write was acknowledged and read back as written.
    Written,
    /// Nothing came back within the deadline, and the retries are spent.
    NoAnswer,
    /// The driver would not attempt the transaction: it was asked while another
    /// was outstanding, or asked for something it does not allow in the state it
    /// is in. Distinct from [`Self::NoAnswer`] because it is evidence about the
    /// host and the driver rather than about the servo — nothing was put on the
    /// bus, so a servo that answers perfectly well produces this too.
    DriverRefused,
    /// The servo answered with its error field set.
    ServoError {
        /// The status error field, verbatim. Never reinterpreted here: one code
        /// covers both a refused out-of-range goal and a latched bus watchdog,
        /// and a sequencer that guessed between them would guess wrong.
        code: u8,
    },
    /// The write was acknowledged, and the register does not hold what was
    /// written.
    VerifyMismatch {
        /// What the register held afterwards.
        read_back: Value,
    },
    /// A frame came back that the wire mangled. Never retried: a corrupted answer
    /// carries no evidence about what the servo actually did.
    WireCorrupt,
}

/// The shape of a [`BusResult`], and the operator's word for it.
///
/// The shape is the vocabulary's: one enum, declared in `motion/seq.clk`, so the
/// number a verdict crosses a slot in and the name this crate matches on are one
/// thing. [`AnswerShape::NotApplicable`] is the value a failure that is not about
/// a mismatch of answer shapes leaves in those fields; nothing a bus answers is
/// ever that.
pub mod answer {
    pub use brenn_reachy__motion__seq_clk_rs::AnswerShape;

    vocab_name! {
        /// An answer shape as a failure line says it.
        pub struct Name(AnswerShape) {
            AnswerShape::NotApplicable => "no answer shape",
            AnswerShape::Pinged => "ping reply",
            AnswerShape::Value => "register value",
            AnswerShape::Written => "verified write",
            AnswerShape::Missing => "silence",
            AnswerShape::DriverRefused => "a driver that would not run it",
            AnswerShape::Refused => "refusal",
            AnswerShape::Mismatched => "read-back mismatch",
            AnswerShape::Corrupt => "corrupt frame",
        }
    }

    /// Every shape a transaction is answered in, the not-applicable zero
    /// excluded: nothing a bus answers is ever that.
    pub fn shapes() -> impl Iterator<Item = AnswerShape> {
        crate::vocab::without_zero(AnswerShape::VARIANTS)
    }
}

pub use answer::AnswerShape;

impl BusResult {
    /// Which shape this is.
    #[must_use]
    pub fn kind(&self) -> AnswerShape {
        match self {
            Self::Pinged { .. } => AnswerShape::Pinged,
            Self::Value(_) => AnswerShape::Value,
            Self::Written => AnswerShape::Written,
            Self::NoAnswer => AnswerShape::Missing,
            Self::DriverRefused => AnswerShape::DriverRefused,
            Self::ServoError { .. } => AnswerShape::Refused,
            Self::VerifyMismatch { .. } => AnswerShape::Mismatched,
            Self::WireCorrupt => AnswerShape::Corrupt,
        }
    }

    /// The model number a ping returned, or the typed failure to report.
    pub fn pinged(&self, context: StepContext) -> Result<u16, SeqError> {
        match self {
            Self::Pinged { model } => Ok(*model),
            other => Err(other.failure(context, AnswerShape::Pinged, None)),
        }
    }

    /// The value a read returned, or the typed failure to report.
    pub fn value(&self, context: StepContext) -> Result<Value, SeqError> {
        match self {
            Self::Value(value) => Ok(*value),
            other => Err(other.failure(context, AnswerShape::Value, None)),
        }
    }

    /// Confirmation that `wrote` landed, or the typed failure to report. `wrote`
    /// is what the sequencer asked for, so a mismatch can say both halves.
    pub fn written(&self, context: StepContext, wrote: Value) -> Result<(), SeqError> {
        match self {
            Self::Written => Ok(()),
            other => Err(other.failure(context, AnswerShape::Written, Some(wrote))),
        }
    }

    /// The failure this result amounts to, given what the step wanted.
    fn failure(&self, context: StepContext, wanted: AnswerShape, wrote: Option<Value>) -> SeqError {
        match self {
            Self::NoAnswer => SeqError::NoAnswer { context },
            Self::DriverRefused => SeqError::DriverRefused { context },
            Self::ServoError { code } => SeqError::Refused {
                context,
                code: *code,
            },
            Self::WireCorrupt => SeqError::WireCorrupt { context },
            // A read-back mismatch answering anything but a write is a driver
            // answering a question nobody asked: there is no written value for it
            // to be a mismatch against.
            Self::VerifyMismatch { read_back } => match wrote {
                Some(expected) => SeqError::VerifyMismatch {
                    context,
                    expected,
                    read_back: *read_back,
                },
                None => SeqError::WrongAnswer {
                    context,
                    expected: wanted,
                    observed: AnswerShape::Mismatched,
                },
            },
            other => SeqError::WrongAnswer {
                context,
                expected: wanted,
                observed: other.kind(),
            },
        }
    }
}

/// Which part of a sequence is running, and the operator's word for it.
///
/// The phase is the vocabulary's: one enum, declared in `motion/seq.clk`, holding
/// both sequencers' phases, because a failure is reported the same way whichever
/// sequence raised it. [`SeqStepKind::None`] is the value a slot nothing wrote
/// holds; no running sequence is ever in it.
pub mod step {
    pub use brenn_reachy__motion__seq_clk_rs::SeqStepKind;

    vocab_name! {
        /// A phase as a bring-up report says it.
        pub struct Name(SeqStepKind) {
            SeqStepKind::None => "no phase",
            SeqStepKind::Presence => "presence",
            SeqStepKind::Identity => "identity",
            SeqStepKind::Provision => "provisioning",
            SeqStepKind::VoltageGate => "voltage gate",
            SeqStepKind::Health => "health",
            SeqStepKind::PoseAndDatum => "measured pose and datum",
            SeqStepKind::GainsProfiles => "gains and profiles",
            SeqStepKind::PinAndEnable => "pin and enable",
            SeqStepKind::VerifyAtStow => "stow verification",
            SeqStepKind::Dwell => "settle dwell",
            SeqStepKind::TorqueOff => "torque off",
        }
    }
}

pub use step::SeqStepKind;

/// What a sequence was doing when something went wrong.
///
/// Carried by every failure, so a report never has to be read next to the code
/// to work out which of nine servos and which of two dozen registers it meant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StepContext {
    /// The phase that was running.
    pub step: SeqStepKind,
    /// The servo being addressed.
    pub id: u8,
    /// The register concerned, where one was.
    pub reg: Option<RegId>,
}

impl StepContext {
    /// A context naming a register.
    #[must_use]
    pub fn reg(step: SeqStepKind, id: u8, reg: RegId) -> Self {
        Self {
            step,
            id,
            reg: Some(reg),
        }
    }

    /// A context for a step that concerns no particular register.
    #[must_use]
    pub fn servo(step: SeqStepKind, id: u8) -> Self {
        Self {
            step,
            id,
            reg: None,
        }
    }
}

impl fmt::Display for StepContext {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} of servo {}", step::Name(self.step), self.id)?;
        if let Some(reg) = self.reg {
            write!(f, ", {}", reg::Name(reg))?;
        }
        Ok(())
    }
}

/// Why a sequence stopped.
///
/// The transaction-level failures live here, and each sequencer adds the
/// conditions only it can detect.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum SeqError {
    /// A servo did not answer, and the retries are spent.
    #[error("{context}: no answer")]
    NoAnswer {
        /// Where this happened.
        context: StepContext,
    },
    /// The driver would not run the transaction. Nothing reached the bus, so this
    /// says nothing about the servo: it is a request made while another was
    /// outstanding, or one the driver does not allow in the state it is in.
    #[error("{context}: the driver would not run it")]
    DriverRefused {
        /// Where this happened.
        context: StepContext,
    },
    /// A servo answered with its error field set. The code travels verbatim.
    #[error("{context}: refused with status code {code:#04x}")]
    Refused {
        /// Where this happened.
        context: StepContext,
        /// The status error field as received.
        code: u8,
    },
    /// A reply came back mangled. Never retried.
    #[error("{context}: the reply was corrupt on the wire")]
    WireCorrupt {
        /// Where this happened.
        context: StepContext,
    },
    /// A register does not hold what was just written to it.
    #[error("{context}: wrote {} and read back {}", Shown(*.expected), Shown(*.read_back))]
    VerifyMismatch {
        /// Where this happened.
        context: StepContext,
        /// What the sequencer asked for.
        expected: Value,
        /// What the register held afterwards.
        read_back: Value,
    },
    /// The driver answered a question the step did not ask — a wiring mistake in
    /// whatever is executing the requests, not a fault of the machine.
    #[error("{context}: expected a {} and got a {}", answer::Name(*.expected), answer::Name(*.observed))]
    WrongAnswer {
        /// Where this happened.
        context: StepContext,
        /// What the step needed.
        expected: AnswerShape,
        /// What arrived.
        observed: AnswerShape,
    },
    /// A register's value arrived in the wrong shape, which likewise means the
    /// register table and the step disagree about what this register is.
    #[error("{context}: expected an {} and got a {}", ShapeName(*.expected), ShapeName(*.observed))]
    WrongValue {
        /// Where this happened.
        context: StepContext,
        /// The shape the step needed.
        expected: ValueShape,
        /// The shape that arrived.
        observed: ValueShape,
    },
    /// A measured angle is not a number, so nothing can be decided from it: it
    /// is inside no window, closes no linkage, and would become a meaningless
    /// goal.
    #[error("{context}: {} measured {angle} rad, which is not an angle", Name(*.joint))]
    UnplaceableAngle {
        /// Where this happened.
        context: StepContext,
        /// The joint whose reading it was.
        joint: JointRef,
        /// The reading, as it arrived.
        angle: f64,
    },
    /// Servos did not answer a ping. All nine are asked before this is raised, so
    /// the set is complete: a report that stopped at the first silence would send
    /// somebody looking at one servo when three are unplugged.
    #[error("{context}: no answer from {absent}")]
    AbsentServos {
        /// Where this happened; the first silent servo.
        context: StepContext,
        /// Every servo that did not answer.
        absent: AbsentSet,
    },
    /// A servo reports a model number this platform's servos do not, which means
    /// the bus is not the platform this code was written for.
    #[error("{context}: model {model}, where this platform reports {expected}")]
    IdentityMismatch {
        /// Where this happened.
        context: StepContext,
        /// What this servo reports.
        model: u16,
        /// What a servo at this address reports on this platform.
        expected: u16,
    },
    /// A provisioned register does not hold what this platform was set up with.
    /// Arming verifies provisioning and never repairs it: a register that is
    /// wrong is a question about how the machine was configured.
    #[error("{context}: provisioned as {}, holds {}", Shown(*.expected), Shown(*.observed))]
    ProvisionMismatch {
        /// Where this happened.
        context: StepContext,
        /// What the configuration says it should hold.
        expected: Value,
        /// What it holds.
        observed: Value,
    },
    /// The supply never reached the arming floor within the budget — the
    /// commissioning gate, which polls and waits. A rail found low by a gate
    /// that does neither is [`SeqError::SupplyBelowFloor`]. Every reading of the
    /// last sweep travels with it, because one low servo and nine low servos
    /// point at different halves of the wiring.
    #[error(
        "{context}: {lowest:.1} V is below the {limit:.1} V arming floor after {:.1} s",
        waited.as_secs_f64()
    )]
    VoltageLow {
        /// Where this happened; the servo reporting `lowest`.
        context: StepContext,
        /// The last sweep's readings, in bus order, volts.
        readings: [f64; ROW_COUNT],
        /// The lowest of them, volts.
        lowest: f64,
        /// The floor, volts.
        limit: f64,
        /// How long the gate waited.
        waited: Duration,
    },
    /// The last sweep of the rail read below the arming floor, so torque was not
    /// enabled. Nothing waited and nothing was written: the reading is whatever
    /// the resting watch brought back, and the next request judges a fresh one.
    /// Every reading travels with it, because one low servo and nine low servos
    /// point at different halves of the wiring.
    #[error("{context}: {lowest:.1} V is below the {limit:.1} V arming floor; torque stays off")]
    SupplyBelowFloor {
        /// Where this happened; the servo reporting `lowest`.
        context: StepContext,
        /// The sweep's readings, in bus order, volts.
        readings: [f64; ROW_COUNT],
        /// The lowest of them, volts.
        lowest: f64,
        /// The floor, volts.
        limit: f64,
    },
    /// A servo has latched a hardware error beyond a supply dip it rode out.
    /// Never followed by a reboot: rebooting a servo holding this head drops it.
    #[error("{context}: hardware error bits {bits:#010b}")]
    UnhealthyServo {
        /// Where this happened.
        context: StepContext,
        /// The byte as read.
        bits: u8,
    },
    /// The measured resting angles place no pose the platform could be holding.
    /// The angles are what they are, so what is in question is whether the model
    /// and the machine are the same machine — geometry, assembly, or a servo
    /// answering for a joint it is not.
    #[error("{context}: the resting angles place no plausible pose ({cause})")]
    RestPoseImplausible {
        /// Where this happened.
        context: StepContext,
        /// What the solver said.
        cause: FkError,
    },
    /// The angles the machine reported once its torque was on place no pose.
    /// The trajectory the next move starts from would have no start.
    #[error("{context}: the angles place no pose ({cause})")]
    PinnedPoseUnsolvable {
        /// Where this happened.
        context: StepContext,
        /// What the solver said.
        cause: FkError,
    },
    /// The transaction awaiting an answer is a record this build cannot read:
    /// an operation or a value shape outside the vocabulary, or a register
    /// number nothing here names. Whatever wrote it disagrees with this build
    /// about what a transaction is, so nothing is guessed from it and the
    /// sequence stops.
    #[error("{context}: the transaction it is waiting on cannot be read")]
    PendingUnreadable {
        /// Where this happened; the servo the record still names.
        context: StepContext,
    },
    /// The verdict the state slot holds does not read as a failure: the fields
    /// name no failure, no phase, or evidence that does not suit the kind. The
    /// sequence stopped — the phase says so — and what stopped it is not
    /// recoverable, so that is what is reported rather than a verdict assembled
    /// out of the numbers that are there.
    #[error("{context}: the verdict it stopped on cannot be read")]
    VerdictUnreadable {
        /// Where this happened, as far as the fields still say.
        context: StepContext,
    },
    /// A moment this phase has to record is past what the state slot's
    /// nanosecond count reaches. Where the sequence stands could not be written
    /// down, so it stops rather than carrying on with a clock the slot disagrees
    /// with.
    #[error("{context}: the clock is past what the state slot can hold")]
    ClockOutOfRange {
        /// Where this happened.
        context: StepContext,
    },
    /// A solved record the state slot holds does not read as a pose -- a
    /// quaternion that is no rotation. The sequence plans its next writes from
    /// that record, so it stops rather than planning from a pose nobody solved.
    #[error("{context}: a record in the state slot is not a pose")]
    RecordUnreadable {
        /// Where this happened.
        context: StepContext,
    },
    /// A step needs a solved record the state slot holds none of. Where the
    /// unreadable record above says the bytes are damaged, this says a sequence
    /// reached a step no sequence of steps reaches a record-less one at, so the
    /// two are reported apart: the causes have nothing in common.
    #[error("{context}: the state slot holds no record for this step")]
    RecordAbsent {
        /// Where this happened.
        context: StepContext,
    },
}

impl SeqError {
    /// Where the failure happened.
    #[must_use]
    pub fn context(&self) -> StepContext {
        match self {
            Self::NoAnswer { context }
            | Self::DriverRefused { context }
            | Self::Refused { context, .. }
            | Self::WireCorrupt { context }
            | Self::VerifyMismatch { context, .. }
            | Self::WrongAnswer { context, .. }
            | Self::WrongValue { context, .. }
            | Self::UnplaceableAngle { context, .. }
            | Self::AbsentServos { context, .. }
            | Self::IdentityMismatch { context, .. }
            | Self::ProvisionMismatch { context, .. }
            | Self::VoltageLow { context, .. }
            | Self::SupplyBelowFloor { context, .. }
            | Self::UnhealthyServo { context, .. }
            | Self::RestPoseImplausible { context, .. }
            | Self::PinnedPoseUnsolvable { context, .. }
            | Self::PendingUnreadable { context }
            | Self::VerdictUnreadable { context }
            | Self::ClockOutOfRange { context }
            | Self::RecordUnreadable { context }
            | Self::RecordAbsent { context } => *context,
        }
    }

    /// Which failure this is, without any of what it saw.
    ///
    /// What survives a trip through a fixed-layout slot, where the context and
    /// the readings have nowhere to go. See [`SeqFailureKind`].
    #[must_use]
    pub fn kind(&self) -> SeqFailureKind {
        match self {
            Self::NoAnswer { .. } => SeqFailureKind::NoAnswer,
            Self::PendingUnreadable { .. } => SeqFailureKind::PendingUnreadable,
            Self::VerdictUnreadable { .. } => SeqFailureKind::VerdictUnreadable,
            Self::ClockOutOfRange { .. } => SeqFailureKind::ClockOutOfRange,
            Self::RecordUnreadable { .. } => SeqFailureKind::RecordUnreadable,
            Self::RecordAbsent { .. } => SeqFailureKind::RecordAbsent,
            Self::DriverRefused { .. } => SeqFailureKind::DriverRefused,
            Self::Refused { .. } => SeqFailureKind::Refused,
            Self::WireCorrupt { .. } => SeqFailureKind::WireCorrupt,
            Self::VerifyMismatch { .. } => SeqFailureKind::VerifyMismatch,
            Self::WrongAnswer { .. } => SeqFailureKind::WrongAnswer,
            Self::WrongValue { .. } => SeqFailureKind::WrongValue,
            Self::UnplaceableAngle { .. } => SeqFailureKind::UnplaceableAngle,
            Self::AbsentServos { .. } => SeqFailureKind::AbsentServos,
            Self::IdentityMismatch { .. } => SeqFailureKind::IdentityMismatch,
            Self::ProvisionMismatch { .. } => SeqFailureKind::ProvisionMismatch,
            Self::VoltageLow { .. } => SeqFailureKind::VoltageLow,
            Self::SupplyBelowFloor { .. } => SeqFailureKind::SupplyBelowFloor,
            Self::UnhealthyServo { .. } => SeqFailureKind::UnhealthyServo,
            Self::RestPoseImplausible { .. } => SeqFailureKind::RestPoseImplausible,
            Self::PinnedPoseUnsolvable { .. } => SeqFailureKind::PinnedPoseUnsolvable,
        }
    }
}

/// Which [`SeqError`] a failure was, and the operator's word for it.
///
/// The name is the vocabulary's: one enum, declared in `motion/seq.clk`, so the
/// number a slot holds a stopped sequence's verdict in and the name this crate
/// matches on are one thing. [`SeqFailureKind::None`] is no failure, which is
/// what a running phase and an unwritten slot hold.
///
/// [`SeqError`] carries a [`StepContext`] and, variant by variant, registers,
/// readings and solver causes — none of which fit a fixed-layout enum field, and
/// all of which have already been reported by the time anything reads a slot
/// back. The name of the failure does fit, and the name is what still tells an
/// operator which half of the machine to look at.
pub mod failure {
    pub use brenn_reachy__motion__seq_clk_rs::SeqFailureKind;

    vocab_name! {
        /// A failure as a bring-up report says it.
        pub struct Name(SeqFailureKind) {
            SeqFailureKind::None => "no failure",
            SeqFailureKind::NoAnswer => "no answer",
            SeqFailureKind::PendingUnreadable => "a transaction this build cannot read",
            SeqFailureKind::VerdictUnreadable => "a verdict this build cannot read",
            SeqFailureKind::ClockOutOfRange => "a clock the state slot cannot hold",
            SeqFailureKind::RecordUnreadable => "a solved record this build cannot read",
            SeqFailureKind::RecordAbsent => "a step with no solved record to plan from",
            SeqFailureKind::Refused => "a refusal",
            SeqFailureKind::WireCorrupt => "a corrupt reply",
            SeqFailureKind::VerifyMismatch => "a write that did not read back",
            SeqFailureKind::WrongAnswer => "an answer of the wrong shape",
            SeqFailureKind::WrongValue => "a value of the wrong shape",
            SeqFailureKind::UnplaceableAngle => "an angle that is not a number",
            SeqFailureKind::AbsentServos => "servos that did not answer a ping",
            SeqFailureKind::IdentityMismatch => "a servo of another model",
            SeqFailureKind::ProvisionMismatch => "a register provisioned otherwise",
            SeqFailureKind::VoltageLow => "a supply that never reached the arming floor",
            SeqFailureKind::SupplyBelowFloor => "a supply below the arming floor",
            SeqFailureKind::UnhealthyServo => "latched hardware error bits",
            SeqFailureKind::RestPoseImplausible => "resting angles that place no pose",
            SeqFailureKind::PinnedPoseUnsolvable => "pinned angles that place no pose",
            SeqFailureKind::DriverRefused => "a driver that would not run a transaction",
        }
    }

    /// Every failure a sequence stops on, the no-failure zero excluded: that one
    /// is what a running phase and an unwritten slot hold.
    pub fn raised() -> impl Iterator<Item = SeqFailureKind> {
        crate::vocab::without_zero(SeqFailureKind::VARIANTS)
    }
}

pub use failure::SeqFailureKind;

/// What a sequencer wants to happen next.
///
/// `S` is the sequence's own summary — what arming or disarming has to hand back
/// on success. The four arms are the whole protocol between a sequencer and its
/// driver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SeqAction<S> {
    /// Run the transaction the sequencer is holding — [`Sequencer::pending`] —
    /// and bring the result back.
    ///
    /// Carried by reference rather than in the arm: the transaction is the
    /// vocabulary's own record, which lives in the sequencer's state and is
    /// handed to a slot or a datagram as itself.
    Transact,
    /// Come back no earlier than this time on the driver's own clock. Used where
    /// a value needs re-reading at a spacing, not where a delay would hide a
    /// failure.
    Wait {
        /// The time to come back at.
        until: Duration,
    },
    /// The sequence finished, and this is what it found.
    Done(S),
    /// The sequence stopped here, for this reason.
    Fail(SeqError),
}

impl<S> SeqAction<S> {
    /// Whether this action ends the sequence, either way.
    #[must_use]
    pub fn is_terminal(&self) -> bool {
        matches!(self, Self::Done(_) | Self::Fail(_))
    }
}

/// The shape every sequencer has, so one driver can run any of them.
///
/// The driver's whole loop is: take an action, execute it, hand the result back.
/// A sequencer sees the previous result and nothing else about the port.
pub trait Sequencer {
    /// What this sequence hands back when it finishes.
    type Summary;

    /// Take the previous transaction's result — `None` on the first call, and on
    /// the call after a [`SeqAction::Wait`] — and decide what happens next.
    fn next(&mut self, now: Duration, prior: Option<&BusResult>) -> SeqAction<Self::Summary>;

    /// The transaction awaiting an answer, which a [`SeqAction::Transact`] is
    /// about. Inactive on the step after a wait, before the first step, and
    /// once the sequence has ended.
    fn pending(&self) -> &BusTxnWire;

    /// Which phase the sequence is in, which is the phase the action just
    /// handed out belongs to. The driver logs against it.
    fn step(&self) -> SeqStepKind;
}

#[cfg(test)]
mod tests {
    use super::*;
    use brenn_reachy__hardware__dynamixel__registers_clk_rs::RegIdWire;

    use crate::value;

    fn context() -> StepContext {
        StepContext::reg(SeqStepKind::Provision, 12, RegId::OperatingMode)
    }

    /// The absent set at every size that reads differently: none, one, a few,
    /// and the whole bus. The rendering is what a person acts on — they go and
    /// look at the servos it names — so each branch is checked as a whole string,
    /// including that one silent servo reads as one servo.
    #[test]
    fn an_absent_set_reads_at_every_size() {
        let ids = [10, 11, 12, 13, 14, 15, 16, 17, 18];
        let of = |absent: [bool; ROW_COUNT]| AbsentSet::new(&ids, &absent);

        let none = of([false; ROW_COUNT]);
        assert_eq!(none.count(), 0);
        assert_eq!(none.ids(), [] as [u8; 0]);
        assert_eq!(none.to_string(), "no servos");

        let one = of([false, false, false, true, false, false, false, false, false]);
        assert_eq!(one.count(), 1);
        assert_eq!(one.ids(), [13]);
        assert_eq!(one.to_string(), "servo 13");

        let few = of([true, false, false, true, false, false, false, false, true]);
        assert_eq!(few.count(), 3);
        assert_eq!(few.ids(), [10, 13, 18], "the set is in bus order");
        assert_eq!(few.to_string(), "servos 10, 13, 18");

        let all = of([true; ROW_COUNT]);
        assert_eq!(all.count(), ROW_COUNT);
        assert_eq!(all.ids(), ids);
        assert_eq!(all.to_string(), "all nine servos");
    }

    /// A failure names the phase, the servo and the register, in that order,
    /// whether or not there is a register to name.
    #[test]
    fn a_context_names_everything_it_has() {
        assert_eq!(
            context().to_string(),
            "provisioning of servo 12, operating mode"
        );
        assert_eq!(
            StepContext::servo(SeqStepKind::Presence, 17).to_string(),
            "presence of servo 17"
        );
    }

    /// Each way a transaction can come out is its own shape, and the four failure
    /// shapes become four different errors rather than one flattened one.
    #[test]
    fn every_outcome_keeps_its_own_shape() {
        assert_eq!(BusResult::Pinged { model: 1 }.kind(), AnswerShape::Pinged);
        assert_eq!(BusResult::Value(value::u8(3)).kind(), AnswerShape::Value);
        assert_eq!(BusResult::Written.kind(), AnswerShape::Written);
        assert_eq!(BusResult::NoAnswer.kind(), AnswerShape::Missing);
        assert_eq!(BusResult::DriverRefused.kind(), AnswerShape::DriverRefused);
        assert_eq!(
            BusResult::ServoError { code: 7 }.kind(),
            AnswerShape::Refused
        );
        assert_eq!(
            BusResult::VerifyMismatch {
                read_back: value::u8(0)
            }
            .kind(),
            AnswerShape::Mismatched
        );
        assert_eq!(BusResult::WireCorrupt.kind(), AnswerShape::Corrupt);

        let wrote = value::u8(3);
        let cases = [
            (
                BusResult::NoAnswer,
                SeqError::NoAnswer { context: context() },
            ),
            (
                BusResult::DriverRefused,
                SeqError::DriverRefused { context: context() },
            ),
            (
                BusResult::ServoError { code: 0x07 },
                SeqError::Refused {
                    context: context(),
                    code: 0x07,
                },
            ),
            (
                BusResult::WireCorrupt,
                SeqError::WireCorrupt { context: context() },
            ),
            (
                BusResult::VerifyMismatch {
                    read_back: value::u8(0),
                },
                SeqError::VerifyMismatch {
                    context: context(),
                    expected: wrote,
                    read_back: value::u8(0),
                },
            ),
        ];
        for (result, expected) in cases {
            assert_eq!(
                result.written(context(), wrote),
                Err(expected),
                "writing under {result:?}"
            );
            assert_eq!(expected.context(), context());
        }

        // A read has no written value for a mismatch to be measured against, so a
        // driver reporting one to a read is reporting a wiring mistake.
        assert_eq!(
            BusResult::VerifyMismatch {
                read_back: value::u8(0)
            }
            .value(context()),
            Err(SeqError::WrongAnswer {
                context: context(),
                expected: AnswerShape::Value,
                observed: AnswerShape::Mismatched,
            })
        );
    }

    /// A read that succeeded hands back its value; a ping hands back the model.
    #[test]
    fn a_good_answer_is_taken_apart() {
        assert_eq!(
            BusResult::Value(value::radians(0.5)).value(context()),
            Ok(value::radians(0.5))
        );
        assert_eq!(
            BusResult::Pinged { model: 1200 }.pinged(context()),
            Ok(1200)
        );
        assert_eq!(BusResult::Written.written(context(), value::u8(3)), Ok(()));
    }

    /// A mismatched read-back reports both halves: what was asked for and what
    /// the register actually holds. Either one alone is unactionable.
    #[test]
    fn a_mismatch_reports_both_halves() {
        let failure = BusResult::VerifyMismatch {
            read_back: value::u8(0),
        }
        .written(context(), value::u8(3))
        .expect_err("a mismatch is a failure");
        assert_eq!(
            failure,
            SeqError::VerifyMismatch {
                context: context(),
                expected: value::u8(3),
                read_back: value::u8(0),
            }
        );
        assert_eq!(
            failure.to_string(),
            "provisioning of servo 12, operating mode: wrote 3 and read back 0"
        );
    }

    /// An answer of the wrong shape is a mistake in whatever is driving the
    /// sequencer, and it is reported as one rather than being mistaken for a
    /// failure of the machine.
    #[test]
    fn an_answer_to_the_wrong_question_says_so() {
        assert_eq!(
            BusResult::Written.value(context()),
            Err(SeqError::WrongAnswer {
                context: context(),
                expected: AnswerShape::Value,
                observed: AnswerShape::Written,
            })
        );
        assert_eq!(
            BusResult::Value(value::u8(1)).pinged(context()),
            Err(SeqError::WrongAnswer {
                context: context(),
                expected: AnswerShape::Pinged,
                observed: AnswerShape::Value,
            })
        );
        assert_eq!(
            BusResult::Pinged { model: 1 }
                .value(context())
                .expect_err("a ping is not a value")
                .to_string(),
            "provisioning of servo 12, operating mode: expected a register value and got a ping reply"
        );
    }

    /// A three-step stand-in for the real sequencers: ping one servo, wait out a
    /// poll interval, report what it said. Enough to exercise the framework's
    /// whole contract, and small enough to read.
    struct Toy {
        step: u8,
        model: u16,
        pending: BusTxnWire,
    }

    impl Toy {
        const CONTEXT: StepContext = StepContext {
            step: SeqStepKind::Presence,
            id: 10,
            reg: None,
        };

        fn new() -> Self {
            Self {
                step: 0,
                model: 0,
                pending: crate::txn::none(),
            }
        }
    }

    impl Sequencer for Toy {
        type Summary = u16;

        fn next(&mut self, now: Duration, prior: Option<&BusResult>) -> SeqAction<u16> {
            match self.step {
                0 => {
                    self.step = 1;
                    self.pending = crate::txn::ping(10);
                    SeqAction::Transact
                }
                1 => match prior.map(|result| result.pinged(Self::CONTEXT)) {
                    Some(Ok(model)) => {
                        self.model = model;
                        self.step = 2;
                        SeqAction::Wait {
                            until: now + Duration::from_millis(100),
                        }
                    }
                    Some(Err(error)) => SeqAction::Fail(error),
                    None => SeqAction::Fail(SeqError::NoAnswer {
                        context: Self::CONTEXT,
                    }),
                },
                _ => SeqAction::Done(self.model),
            }
        }

        fn step(&self) -> SeqStepKind {
            SeqStepKind::Presence
        }

        fn pending(&self) -> &BusTxnWire {
            &self.pending
        }
    }

    /// The pump's driver loop: take an action, execute it, hand the result
    /// back. A wait consumes no transaction and carries no result forward.
    #[test]
    fn a_sequencer_is_driven_by_handing_results_back() {
        let mut toy = Toy::new();
        let mut now = Duration::ZERO;
        let mut prior = None;
        let mut transactions = 0;
        let mut waited = false;

        let summary = loop {
            let action = toy.next(now, prior.as_ref());
            assert_eq!(action.is_terminal(), matches!(action, SeqAction::Done(_)));
            match action {
                SeqAction::Transact => {
                    assert_eq!(*toy.pending(), crate::txn::ping(10));
                    transactions += 1;
                    prior = Some(BusResult::Pinged { model: 1200 });
                }
                SeqAction::Wait { until } => {
                    assert!(until > now);
                    now = until;
                    waited = true;
                    prior = None;
                }
                SeqAction::Done(summary) => break summary,
                SeqAction::Fail(error) => panic!("{error}"),
            }
        };
        assert_eq!(summary, 1200, "the summary carries what was learned");
        assert_eq!(transactions, 1);
        assert!(waited);
        assert_eq!(now, Duration::from_millis(100));
    }

    /// A failing transaction stops the sequence with the typed cause, which is how
    /// every real failure will surface.
    #[test]
    fn a_failed_transaction_stops_the_sequence() {
        let mut toy = Toy::new();
        assert!(matches!(
            toy.next(Duration::ZERO, None),
            SeqAction::Transact
        ));

        let action = toy.next(Duration::ZERO, Some(&BusResult::NoAnswer));
        assert!(action.is_terminal());
        let SeqAction::Fail(error) = action else {
            panic!("expected a failure, got {action:?}");
        };
        assert_eq!(error.to_string(), "presence of servo 10: no answer");
    }

    /// The two symmetric register pairs, and one phase, render as themselves.
    ///
    /// The adapters' generated guard proves every word is distinct and says
    /// something, which a swapped pair of arms passes: "minimum" and "maximum"
    /// stay distinct when they trade places, and the operator reading a
    /// provisioning mismatch is then sent to the wrong limit. These are the
    /// renderings where a swap is both plausible and invisible.
    #[test]
    fn the_symmetric_names_are_not_each_other() {
        assert_eq!(
            reg::Name(RegId::MinPositionLimit).to_string(),
            "minimum position limit"
        );
        assert_eq!(
            reg::Name(RegId::MaxPositionLimit).to_string(),
            "maximum position limit"
        );
        assert_eq!(
            reg::Name(RegId::MinVoltageLimit).to_string(),
            "minimum voltage limit"
        );
        assert_eq!(
            reg::Name(RegId::MaxVoltageLimit).to_string(),
            "maximum voltage limit"
        );
        assert_eq!(
            step::Name(SeqStepKind::VerifyAtStow).to_string(),
            "stow verification"
        );
    }

    crate::vocab_numbering! {
        /// The register numbering is the one written down here.
        ///
        /// It keys what a transaction asks a servo to do, in a slot and at the
        /// process edge, so the list is appended to and never renumbered: a
        /// register inserted among these turns a goal write into a shutdown
        /// write in a peer built at the other revision.
        the_register_numbering_is_the_one_written_down:
            RegId as RegIdWire, past the end 23 {
            RegId::None => 0,
            RegId::TorqueEnable => 1,
            RegId::GoalPosition => 2,
            RegId::PresentPosition => 3,
            RegId::OperatingMode => 4,
            RegId::HomingOffset => 5,
            RegId::ReturnDelayTime => 6,
            RegId::MinPositionLimit => 7,
            RegId::MaxPositionLimit => 8,
            RegId::Shutdown => 9,
            RegId::DriveMode => 10,
            RegId::MaxVoltageLimit => 11,
            RegId::MinVoltageLimit => 12,
            RegId::CurrentLimit => 13,
            RegId::VelocityLimit => 14,
            RegId::TemperatureLimit => 15,
            RegId::BusWatchdog => 16,
            RegId::ProfileAcceleration => 17,
            RegId::ProfileVelocity => 18,
            RegId::PositionGains => 19,
            RegId::HardwareErrorStatus => 20,
            RegId::PresentInputVoltage => 21,
            RegId::ModelNumber => 22,
        }
    }
}
