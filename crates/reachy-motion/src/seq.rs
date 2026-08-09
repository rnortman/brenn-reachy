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
//! Every request in [`BusRequest`] names one servo. Arming's writes must each be
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

use crate::joints::JointId;

/// A register, named the way this crate thinks of them.
///
/// Addresses, widths and byte order belong to the layer that owns the wire; the
/// closed set here is the vocabulary a sequencer may ask for, and the mapping is
/// somebody else's table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RegId {
    /// Whether the servo holds its goal.
    TorqueEnable,
    /// Where the servo is being commanded to.
    GoalPosition,
    /// Where the servo says it is.
    PresentPosition,
    /// Which control mode the servo is in. Position mode is the only one this
    /// project ever runs, and the only one whose position limits apply.
    OperatingMode,
    /// The offset added to the raw position, which is how this platform's legs
    /// are datum-shifted in the servo rather than in the host.
    HomingOffset,
    /// How long the servo waits before answering.
    ReturnDelayTime,
    /// The bottom of the range the servo itself refuses to be commanded past.
    MinPositionLimit,
    /// The top of that range.
    MaxPositionLimit,
    /// Which hardware errors make the servo shut its own torque off.
    Shutdown,
    /// Direction and profile flags.
    DriveMode,
    /// The supply ceiling the servo alarms on.
    MaxVoltageLimit,
    /// The supply floor it alarms on.
    MinVoltageLimit,
    /// The current ceiling.
    CurrentLimit,
    /// The velocity ceiling.
    VelocityLimit,
    /// The temperature ceiling.
    TemperatureLimit,
    /// The timeout after which the servo stops holding its goal because the host
    /// went quiet. Disabled on this platform: on this linkage a servo that stops
    /// holding drops the head.
    BusWatchdog,
    /// The acceleration limit of the servo's own profile, the backstop under
    /// host-side shaping.
    ProfileAcceleration,
    /// The velocity limit of that profile.
    ProfileVelocity,
    /// The position loop's three gains, read and written as one span.
    PositionGains,
    /// The latched hardware-error bits.
    HardwareErrorStatus,
    /// The measured supply voltage.
    PresentInputVoltage,
    /// What kind of servo this is.
    ModelNumber,
}

impl RegId {
    /// Every register, for sweeps and tables.
    pub const ALL: [Self; 22] = [
        Self::TorqueEnable,
        Self::GoalPosition,
        Self::PresentPosition,
        Self::OperatingMode,
        Self::HomingOffset,
        Self::ReturnDelayTime,
        Self::MinPositionLimit,
        Self::MaxPositionLimit,
        Self::Shutdown,
        Self::DriveMode,
        Self::MaxVoltageLimit,
        Self::MinVoltageLimit,
        Self::CurrentLimit,
        Self::VelocityLimit,
        Self::TemperatureLimit,
        Self::BusWatchdog,
        Self::ProfileAcceleration,
        Self::ProfileVelocity,
        Self::PositionGains,
        Self::HardwareErrorStatus,
        Self::PresentInputVoltage,
        Self::ModelNumber,
    ];
}

impl fmt::Display for RegId {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            Self::TorqueEnable => "torque enable",
            Self::GoalPosition => "goal position",
            Self::PresentPosition => "present position",
            Self::OperatingMode => "operating mode",
            Self::HomingOffset => "homing offset",
            Self::ReturnDelayTime => "return delay time",
            Self::MinPositionLimit => "minimum position limit",
            Self::MaxPositionLimit => "maximum position limit",
            Self::Shutdown => "shutdown mask",
            Self::DriveMode => "drive mode",
            Self::MaxVoltageLimit => "maximum voltage limit",
            Self::MinVoltageLimit => "minimum voltage limit",
            Self::CurrentLimit => "current limit",
            Self::VelocityLimit => "velocity limit",
            Self::TemperatureLimit => "temperature limit",
            Self::BusWatchdog => "bus watchdog",
            Self::ProfileAcceleration => "profile acceleration",
            Self::ProfileVelocity => "profile velocity",
            Self::PositionGains => "position gains",
            Self::HardwareErrorStatus => "hardware error status",
            Self::PresentInputVoltage => "present input voltage",
            Self::ModelNumber => "model number",
        };
        f.write_str(name)
    }
}

/// A register's value, in engineering units wherever the register has any.
///
/// Radians and volts cross this boundary as radians and volts. Counts and raw
/// bytes are the wire layer's business, and nothing above this line may depend
/// on them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum RegValue {
    /// A one-byte register.
    U8(u8),
    /// A two-byte register.
    U16(u16),
    /// A four-byte register.
    U32(u32),
    /// A four-byte register whose contents are signed. The homing offset is
    /// one: it is a quarter turn either side of zero on this platform, and read
    /// unsigned it reports a negative offset as a number near four billion.
    I32(i32),
    /// An angle: a position register, in the model's own frame.
    Radians(f64),
    /// A supply voltage.
    Volts(f64),
    /// The position loop's three gains as one span. Their order on the wire is
    /// not this order, and that is the wire layer's problem.
    Gains {
        /// Proportional gain.
        p: u16,
        /// Integral gain.
        i: u16,
        /// Derivative gain.
        d: u16,
    },
}

/// The shape of a [`RegValue`], for saying what was wanted and what arrived.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ValueKind {
    /// [`RegValue::U8`].
    U8,
    /// [`RegValue::U16`].
    U16,
    /// [`RegValue::U32`].
    U32,
    /// [`RegValue::I32`].
    I32,
    /// [`RegValue::Radians`].
    Radians,
    /// [`RegValue::Volts`].
    Volts,
    /// [`RegValue::Gains`].
    Gains,
}

impl fmt::Display for ValueKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            Self::U8 => "one-byte value",
            Self::U16 => "two-byte value",
            Self::U32 => "four-byte value",
            Self::I32 => "signed four-byte value",
            Self::Radians => "angle",
            Self::Volts => "voltage",
            Self::Gains => "gain span",
        };
        f.write_str(name)
    }
}

impl RegValue {
    /// Which shape this is.
    #[must_use]
    pub fn kind(&self) -> ValueKind {
        match self {
            Self::U8(_) => ValueKind::U8,
            Self::U16(_) => ValueKind::U16,
            Self::U32(_) => ValueKind::U32,
            Self::I32(_) => ValueKind::I32,
            Self::Radians(_) => ValueKind::Radians,
            Self::Volts(_) => ValueKind::Volts,
            Self::Gains { .. } => ValueKind::Gains,
        }
    }

    /// The byte, or a failure naming what arrived instead.
    pub fn u8(&self, context: StepContext) -> Result<u8, SeqError> {
        match self {
            Self::U8(value) => Ok(*value),
            other => Err(other.wrong_shape(context, ValueKind::U8)),
        }
    }

    /// The two-byte value, or a failure naming what arrived instead.
    pub fn u16(&self, context: StepContext) -> Result<u16, SeqError> {
        match self {
            Self::U16(value) => Ok(*value),
            other => Err(other.wrong_shape(context, ValueKind::U16)),
        }
    }

    /// The four-byte value, or a failure naming what arrived instead.
    pub fn u32(&self, context: StepContext) -> Result<u32, SeqError> {
        match self {
            Self::U32(value) => Ok(*value),
            other => Err(other.wrong_shape(context, ValueKind::U32)),
        }
    }

    /// The signed four-byte value, or a failure naming what arrived instead.
    pub fn i32(&self, context: StepContext) -> Result<i32, SeqError> {
        match self {
            Self::I32(value) => Ok(*value),
            other => Err(other.wrong_shape(context, ValueKind::I32)),
        }
    }

    /// The angle, or a failure naming what arrived instead.
    pub fn radians(&self, context: StepContext) -> Result<f64, SeqError> {
        match self {
            Self::Radians(value) => Ok(*value),
            other => Err(other.wrong_shape(context, ValueKind::Radians)),
        }
    }

    /// The voltage, or a failure naming what arrived instead.
    pub fn volts(&self, context: StepContext) -> Result<f64, SeqError> {
        match self {
            Self::Volts(value) => Ok(*value),
            other => Err(other.wrong_shape(context, ValueKind::Volts)),
        }
    }

    /// The three gains, or a failure naming what arrived instead.
    pub fn gains(&self, context: StepContext) -> Result<(u16, u16, u16), SeqError> {
        match self {
            Self::Gains { p, i, d } => Ok((*p, *i, *d)),
            other => Err(other.wrong_shape(context, ValueKind::Gains)),
        }
    }

    fn wrong_shape(&self, context: StepContext, wanted: ValueKind) -> SeqError {
        SeqError::WrongValue {
            context,
            expected: wanted,
            observed: self.kind(),
        }
    }
}

impl fmt::Display for RegValue {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::U8(value) => write!(f, "{value}"),
            Self::U16(value) => write!(f, "{value}"),
            Self::U32(value) => write!(f, "{value}"),
            Self::I32(value) => write!(f, "{value}"),
            Self::Radians(value) => write!(f, "{value:.4} rad"),
            Self::Volts(value) => write!(f, "{value:.1} V"),
            Self::Gains { p, i, d } => write!(f, "P {p} I {i} D {d}"),
        }
    }
}

/// One transaction a sequencer wants run. Always addressed to a single servo.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BusRequest {
    /// Ask whether this servo is there.
    Ping {
        /// Its bus ID.
        id: u8,
    },
    /// Read one register.
    ReadReg {
        /// The servo's bus ID.
        id: u8,
        /// Which register.
        reg: RegId,
    },
    /// Write one register, check the acknowledgement, and read it back.
    WriteRegVerified {
        /// The servo's bus ID.
        id: u8,
        /// Which register.
        reg: RegId,
        /// What to write.
        value: RegValue,
    },
}

impl BusRequest {
    /// The servo this request is addressed to.
    #[must_use]
    pub fn id(&self) -> u8 {
        match self {
            Self::Ping { id } | Self::ReadReg { id, .. } | Self::WriteRegVerified { id, .. } => *id,
        }
    }

    /// The register it concerns, if it concerns one.
    #[must_use]
    pub fn reg(&self) -> Option<RegId> {
        match self {
            Self::Ping { .. } => None,
            Self::ReadReg { reg, .. } | Self::WriteRegVerified { reg, .. } => Some(*reg),
        }
    }

    /// The value it writes, if it writes one.
    ///
    /// What a read-back is compared against, so a sequencer confirming a write
    /// takes the expected value from the request it made rather than keeping a
    /// second copy that could disagree with it.
    #[must_use]
    pub fn value(&self) -> Option<RegValue> {
        match self {
            Self::Ping { .. } | Self::ReadReg { .. } => None,
            Self::WriteRegVerified { value, .. } => Some(*value),
        }
    }
}

/// Which servos did not answer.
///
/// A set rather than one ID: nine servos are pinged before anything is decided,
/// so a report names every silent one. Two silent servos and nine silent servos
/// are different observations and read differently.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AbsentSet {
    ids: [u8; JointId::COUNT],
    count: usize,
}

impl AbsentSet {
    /// The set of IDs among `ids` whose corresponding `absent` flag is set.
    #[must_use]
    pub fn new(ids: &[u8; JointId::COUNT], absent: &[bool; JointId::COUNT]) -> Self {
        let mut set = Self {
            ids: [0; JointId::COUNT],
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
        if self.count == JointId::COUNT {
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
/// A silent servo, a refusal, a mismatched read-back and a corrupt frame are four
/// different things, and every one of them is reported as itself. The driver has
/// already spent whatever retry budget applies to the ones worth retrying.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum BusResult {
    /// The servo answered a ping, and said what it is.
    Pinged {
        /// Its model number.
        model: u16,
    },
    /// The read returned this.
    Value(RegValue),
    /// The write was acknowledged and read back as written.
    Written,
    /// Nothing came back within the deadline, and the retries are spent.
    NoAnswer,
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
        read_back: RegValue,
    },
    /// A frame came back that the wire mangled. Never retried: a corrupted answer
    /// carries no evidence about what the servo actually did.
    WireCorrupt,
}

/// The shape of a [`BusResult`], for saying what was wanted and what arrived.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AnswerKind {
    /// [`BusResult::Pinged`].
    Pinged,
    /// [`BusResult::Value`].
    Value,
    /// [`BusResult::Written`].
    Written,
    /// [`BusResult::NoAnswer`].
    Missing,
    /// [`BusResult::ServoError`].
    Refused,
    /// [`BusResult::VerifyMismatch`].
    Mismatched,
    /// [`BusResult::WireCorrupt`].
    Corrupt,
}

impl fmt::Display for AnswerKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            Self::Pinged => "ping reply",
            Self::Value => "register value",
            Self::Written => "verified write",
            Self::Missing => "silence",
            Self::Refused => "refusal",
            Self::Mismatched => "read-back mismatch",
            Self::Corrupt => "corrupt frame",
        };
        f.write_str(name)
    }
}

impl BusResult {
    /// Which shape this is.
    #[must_use]
    pub fn kind(&self) -> AnswerKind {
        match self {
            Self::Pinged { .. } => AnswerKind::Pinged,
            Self::Value(_) => AnswerKind::Value,
            Self::Written => AnswerKind::Written,
            Self::NoAnswer => AnswerKind::Missing,
            Self::ServoError { .. } => AnswerKind::Refused,
            Self::VerifyMismatch { .. } => AnswerKind::Mismatched,
            Self::WireCorrupt => AnswerKind::Corrupt,
        }
    }

    /// The model number a ping returned, or the typed failure to report.
    pub fn pinged(&self, context: StepContext) -> Result<u16, SeqError> {
        match self {
            Self::Pinged { model } => Ok(*model),
            other => Err(other.failure(context, AnswerKind::Pinged, None)),
        }
    }

    /// The value a read returned, or the typed failure to report.
    pub fn value(&self, context: StepContext) -> Result<RegValue, SeqError> {
        match self {
            Self::Value(value) => Ok(*value),
            other => Err(other.failure(context, AnswerKind::Value, None)),
        }
    }

    /// Confirmation that `wrote` landed, or the typed failure to report. `wrote`
    /// is what the sequencer asked for, so a mismatch can say both halves.
    pub fn written(&self, context: StepContext, wrote: RegValue) -> Result<(), SeqError> {
        match self {
            Self::Written => Ok(()),
            other => Err(other.failure(context, AnswerKind::Written, Some(wrote))),
        }
    }

    /// The failure this result amounts to, given what the step wanted.
    fn failure(
        &self,
        context: StepContext,
        wanted: AnswerKind,
        wrote: Option<RegValue>,
    ) -> SeqError {
        match self {
            Self::NoAnswer => SeqError::NoAnswer { context },
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
                    observed: AnswerKind::Mismatched,
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

/// Which part of a sequence is running.
///
/// Both sequencers' phases, in one vocabulary, because a failure is reported the
/// same way whichever sequence raised it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SeqStep {
    /// Every servo answers a ping.
    Presence,
    /// Every servo says what it is.
    Identity,
    /// Every provisioned register holds what it should.
    Provision,
    /// The supply rail, read: a floor commissioning waits for and engaging
    /// checks against.
    VoltageGate,
    /// The latched hardware-error bytes, read.
    Health,
    /// Where the platform is standing, and whether those angles place a pose at
    /// all. What the datum itself rests on — the provisioned homing offsets — is
    /// verified in the provisioning phase, before anything here is read.
    PoseAndDatum,
    /// The position gains and motion profiles, written fresh.
    GainsProfiles,
    /// Goals pinned where the joints stand, then torque enabled — which holds
    /// every joint where it is — then the pose read back.
    PinAndEnable,
    /// Waiting out the settle, before the stow pose is measured at all.
    Dwell,
    /// The settled platform is measured to be at the stow pose.
    VerifyAtStow,
    /// Torque released, servo by servo.
    TorqueOff,
}

impl SeqStep {
    /// Every phase any sequencer has, the torque-on ones in order and then
    /// disarming's.
    ///
    /// Exhaustive: a phase added without a name is caught by the name guard
    /// rather than escaping it.
    pub const ALL: [Self; 11] = [
        Self::Presence,
        Self::Identity,
        Self::Provision,
        Self::VoltageGate,
        Self::Health,
        Self::PoseAndDatum,
        Self::GainsProfiles,
        Self::PinAndEnable,
        Self::Dwell,
        Self::VerifyAtStow,
        Self::TorqueOff,
    ];
}

impl fmt::Display for SeqStep {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let name = match self {
            Self::Presence => "presence",
            Self::Identity => "identity",
            Self::Provision => "provisioning",
            Self::VoltageGate => "voltage gate",
            Self::Health => "health",
            Self::PoseAndDatum => "measured pose and datum",
            Self::GainsProfiles => "gains and profiles",
            Self::PinAndEnable => "pin and enable",
            Self::VerifyAtStow => "stow verification",
            Self::Dwell => "settle dwell",
            Self::TorqueOff => "torque off",
        };
        f.write_str(name)
    }
}

/// What a sequence was doing when something went wrong.
///
/// Carried by every failure, so a report never has to be read next to the code
/// to work out which of nine servos and which of two dozen registers it meant.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct StepContext {
    /// The phase that was running.
    pub step: SeqStep,
    /// The servo being addressed.
    pub id: u8,
    /// The register concerned, where one was.
    pub reg: Option<RegId>,
}

impl StepContext {
    /// A context naming a register.
    #[must_use]
    pub fn reg(step: SeqStep, id: u8, reg: RegId) -> Self {
        Self {
            step,
            id,
            reg: Some(reg),
        }
    }

    /// A context for a step that concerns no particular register.
    #[must_use]
    pub fn servo(step: SeqStep, id: u8) -> Self {
        Self {
            step,
            id,
            reg: None,
        }
    }
}

impl fmt::Display for StepContext {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{} of servo {}", self.step, self.id)?;
        if let Some(reg) = self.reg {
            write!(f, ", {reg}")?;
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
    #[error("{context}: wrote {expected} and read back {read_back}")]
    VerifyMismatch {
        /// Where this happened.
        context: StepContext,
        /// What the sequencer asked for.
        expected: RegValue,
        /// What the register held afterwards.
        read_back: RegValue,
    },
    /// The driver answered a question the step did not ask — a wiring mistake in
    /// whatever is executing the requests, not a fault of the machine.
    #[error("{context}: expected a {expected} and got a {observed}")]
    WrongAnswer {
        /// Where this happened.
        context: StepContext,
        /// What the step needed.
        expected: AnswerKind,
        /// What arrived.
        observed: AnswerKind,
    },
    /// A register's value arrived in the wrong shape, which likewise means the
    /// register table and the step disagree about what this register is.
    #[error("{context}: expected an {expected} and got a {observed}")]
    WrongValue {
        /// Where this happened.
        context: StepContext,
        /// The shape the step needed.
        expected: ValueKind,
        /// The shape that arrived.
        observed: ValueKind,
    },
    /// A measured angle is not a number, so nothing can be decided from it: it
    /// is inside no window, closes no linkage, and would become a meaningless
    /// goal.
    #[error("{context}: {joint} measured {angle} rad, which is not an angle")]
    UnplaceableAngle {
        /// Where this happened.
        context: StepContext,
        /// The joint whose reading it was.
        joint: JointId,
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
    #[error("{context}: provisioned as {expected}, holds {observed}")]
    ProvisionMismatch {
        /// Where this happened.
        context: StepContext,
        /// What the configuration says it should hold.
        expected: RegValue,
        /// What it holds.
        observed: RegValue,
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
        readings: [f64; JointId::COUNT],
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
        readings: [f64; JointId::COUNT],
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
}

impl SeqError {
    /// Where the failure happened.
    #[must_use]
    pub fn context(&self) -> StepContext {
        match self {
            Self::NoAnswer { context }
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
            | Self::PinnedPoseUnsolvable { context, .. } => *context,
        }
    }
}

/// What a sequencer wants to happen next.
///
/// `S` is the sequence's own summary — what arming or disarming has to hand back
/// on success. The four arms are the whole protocol between a sequencer and its
/// driver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SeqAction<S> {
    /// Run this transaction and bring the result back.
    Transact(BusRequest),
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

    /// Which phase the sequence is in, which is the phase the action just
    /// handed out belongs to. The driver logs against it.
    fn step(&self) -> SeqStep;
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::collections::BTreeSet;

    fn context() -> StepContext {
        StepContext::reg(SeqStep::Provision, 12, RegId::OperatingMode)
    }

    /// Every register and every phase has its own name. These names are the whole
    /// content of a bring-up failure report, and two registers sharing one, or a
    /// name left empty, would be invisible until the day it mattered.
    #[test]
    fn every_name_is_distinct_and_says_something() {
        let mut names = BTreeSet::new();
        for reg in RegId::ALL {
            let name = reg.to_string();
            assert!(name.len() > 3, "{reg:?} renders as {name:?}");
            assert!(names.insert(name), "{reg:?} shares a name");
        }
        assert_eq!(names.len(), 22);

        let mut names = BTreeSet::new();
        for step in SeqStep::ALL {
            let name = step.to_string();
            assert!(name.len() > 3, "{step:?} renders as {name:?}");
            assert!(names.insert(name), "{step:?} shares a name");
        }
        assert_eq!(names.len(), SeqStep::ALL.len());
    }

    /// The absent set at every size that reads differently: none, one, a few,
    /// and the whole bus. The rendering is what a person acts on — they go and
    /// look at the servos it names — so each branch is checked as a whole string,
    /// including that one silent servo reads as one servo.
    #[test]
    fn an_absent_set_reads_at_every_size() {
        let ids = [10, 11, 12, 13, 14, 15, 16, 17, 18];
        let of = |absent: [bool; JointId::COUNT]| AbsentSet::new(&ids, &absent);

        let none = of([false; JointId::COUNT]);
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

        let all = of([true; JointId::COUNT]);
        assert_eq!(all.count(), JointId::COUNT);
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
            StepContext::servo(SeqStep::Presence, 17).to_string(),
            "presence of servo 17"
        );
    }

    /// Each way a transaction can come out is its own shape, and the four failure
    /// shapes become four different errors rather than one flattened one.
    #[test]
    fn every_outcome_keeps_its_own_shape() {
        assert_eq!(BusResult::Pinged { model: 1 }.kind(), AnswerKind::Pinged);
        assert_eq!(BusResult::Value(RegValue::U8(3)).kind(), AnswerKind::Value);
        assert_eq!(BusResult::Written.kind(), AnswerKind::Written);
        assert_eq!(BusResult::NoAnswer.kind(), AnswerKind::Missing);
        assert_eq!(
            BusResult::ServoError { code: 7 }.kind(),
            AnswerKind::Refused
        );
        assert_eq!(
            BusResult::VerifyMismatch {
                read_back: RegValue::U8(0)
            }
            .kind(),
            AnswerKind::Mismatched
        );
        assert_eq!(BusResult::WireCorrupt.kind(), AnswerKind::Corrupt);

        let wrote = RegValue::U8(3);
        let cases = [
            (
                BusResult::NoAnswer,
                SeqError::NoAnswer { context: context() },
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
                    read_back: RegValue::U8(0),
                },
                SeqError::VerifyMismatch {
                    context: context(),
                    expected: wrote,
                    read_back: RegValue::U8(0),
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
                read_back: RegValue::U8(0)
            }
            .value(context()),
            Err(SeqError::WrongAnswer {
                context: context(),
                expected: AnswerKind::Value,
                observed: AnswerKind::Mismatched,
            })
        );
    }

    /// A read that succeeded hands back its value; a ping hands back the model.
    #[test]
    fn a_good_answer_is_taken_apart() {
        assert_eq!(
            BusResult::Value(RegValue::Radians(0.5)).value(context()),
            Ok(RegValue::Radians(0.5))
        );
        assert_eq!(
            BusResult::Pinged { model: 1200 }.pinged(context()),
            Ok(1200)
        );
        assert_eq!(
            BusResult::Written.written(context(), RegValue::U8(3)),
            Ok(())
        );
    }

    /// A mismatched read-back reports both halves: what was asked for and what
    /// the register actually holds. Either one alone is unactionable.
    #[test]
    fn a_mismatch_reports_both_halves() {
        let failure = BusResult::VerifyMismatch {
            read_back: RegValue::U8(0),
        }
        .written(context(), RegValue::U8(3))
        .expect_err("a mismatch is a failure");
        assert_eq!(
            failure,
            SeqError::VerifyMismatch {
                context: context(),
                expected: RegValue::U8(3),
                read_back: RegValue::U8(0),
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
                expected: AnswerKind::Value,
                observed: AnswerKind::Written,
            })
        );
        assert_eq!(
            BusResult::Value(RegValue::U8(1)).pinged(context()),
            Err(SeqError::WrongAnswer {
                context: context(),
                expected: AnswerKind::Pinged,
                observed: AnswerKind::Value,
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

    /// Values are taken apart by shape, and a value of the wrong shape names both
    /// shapes rather than being silently coerced.
    #[test]
    fn values_are_taken_apart_by_shape() {
        assert_eq!(RegValue::U8(3).u8(context()), Ok(3));
        assert_eq!(RegValue::U16(1750).u16(context()), Ok(1750));
        assert_eq!(RegValue::U32(4096).u32(context()), Ok(4096));
        assert_eq!(RegValue::I32(-1024).i32(context()), Ok(-1024));
        assert_eq!(RegValue::Radians(-0.5).radians(context()), Ok(-0.5));
        assert_eq!(RegValue::Volts(7.4).volts(context()), Ok(7.4));
        assert_eq!(
            RegValue::Gains { p: 300, i: 0, d: 0 }.gains(context()),
            Ok((300, 0, 0))
        );

        assert_eq!(
            RegValue::U16(3).u8(context()),
            Err(SeqError::WrongValue {
                context: context(),
                expected: ValueKind::U8,
                observed: ValueKind::U16,
            })
        );
        assert_eq!(
            RegValue::U8(3)
                .radians(context())
                .expect_err("a byte is not an angle")
                .to_string(),
            "provisioning of servo 12, operating mode: expected an angle and got a one-byte value"
        );
        // The confusion the signed shape exists to prevent, named in both
        // directions: the same four bytes read unsigned are a homing offset of
        // about four billion.
        let confused = RegValue::U32(1024)
            .i32(context())
            .expect_err("an unsigned four bytes is not the signed register")
            .to_string();
        assert!(
            confused.contains("signed four-byte value") && confused.ends_with("four-byte value"),
            "{confused}"
        );
        assert_eq!(
            RegValue::I32(-1024).u32(context()),
            Err(SeqError::WrongValue {
                context: context(),
                expected: ValueKind::U32,
                observed: ValueKind::I32,
            })
        );
    }

    /// Values render as what they are, units included, because these strings are
    /// what an operator compares against a data sheet.
    #[test]
    fn values_render_with_their_units() {
        assert_eq!(RegValue::U8(3).to_string(), "3");
        assert_eq!(RegValue::Radians(-0.628_3).to_string(), "-0.6283 rad");
        assert_eq!(RegValue::Volts(7.38).to_string(), "7.4 V");
        // A tenth is the register's own resolution, so nothing is lost here; a
        // value sitting on the half rounds down, because the nearest double to
        // 7.35 is below it.
        assert_eq!(RegValue::Volts(7.35).to_string(), "7.3 V");
        assert_eq!(
            RegValue::Gains { p: 300, i: 1, d: 2 }.to_string(),
            "P 300 I 1 D 2"
        );
    }

    /// Every request names exactly one servo, and says which register it is
    /// about when it is about one. Nothing here can address the whole bus.
    #[test]
    fn every_request_is_addressed_to_one_servo() {
        let requests = [
            BusRequest::Ping { id: 10 },
            BusRequest::ReadReg {
                id: 10,
                reg: RegId::PresentPosition,
            },
            BusRequest::WriteRegVerified {
                id: 10,
                reg: RegId::TorqueEnable,
                value: RegValue::U8(1),
            },
        ];
        for request in requests {
            assert_eq!(request.id(), 10);
        }
        assert_eq!(requests[0].reg(), None);
        assert_eq!(requests[1].reg(), Some(RegId::PresentPosition));
        assert_eq!(requests[2].reg(), Some(RegId::TorqueEnable));
    }

    /// A three-step stand-in for the real sequencers: ping one servo, wait out a
    /// poll interval, report what it said. Enough to exercise the framework's
    /// whole contract, and small enough to read.
    struct Toy {
        step: u8,
        model: u16,
    }

    impl Toy {
        const CONTEXT: StepContext = StepContext {
            step: SeqStep::Presence,
            id: 10,
            reg: None,
        };

        fn new() -> Self {
            Self { step: 0, model: 0 }
        }
    }

    impl Sequencer for Toy {
        type Summary = u16;

        fn next(&mut self, now: Duration, prior: Option<&BusResult>) -> SeqAction<u16> {
            match self.step {
                0 => {
                    self.step = 1;
                    SeqAction::Transact(BusRequest::Ping { id: 10 })
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

        fn step(&self) -> SeqStep {
            SeqStep::Presence
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
                SeqAction::Transact(request) => {
                    assert_eq!(request, BusRequest::Ping { id: 10 });
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
            SeqAction::Transact(_)
        ));

        let action = toy.next(Duration::ZERO, Some(&BusResult::NoAnswer));
        assert!(action.is_terminal());
        let SeqAction::Fail(error) = action else {
            panic!("expected a failure, got {action:?}");
        };
        assert_eq!(error.to_string(), "presence of servo 10: no answer");
    }
}
