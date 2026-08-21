//! Fixtures shared by this crate's tests.
//!
//! The servo-side travel windows and the arming configuration drawn from them,
//! and the loop that drives a sequencer against a scripted machine. Two copies
//! of any of it would let one sequencer's tests and another's model different
//! fences, or diverge on what the driver contract is, while both stayed green —
//! the one failure a shared scenario exists to prevent.

use std::time::Duration;

use reachy_kin::EnvelopeConfig;

use crate::arm::{
    ArmConfig, DEFAULT_GAINS, DEFAULT_MIN_ARM_VOLTAGE, DEFAULT_VOLTAGE_BUDGET,
    DEFAULT_VOLTAGE_POLL_PERIOD, ProfileConfig, ProvisionTable, SERVO_IDS,
};
use crate::joints::{JointRef, ROW_COUNT};
use crate::seq::{
    AbsentSet, AnswerShape, BusResult, RegId, SeqAction, SeqError, SeqStepKind, Sequencer,
    StepContext,
};
use crate::tick::{BusFailureSource, Fault, WireFailure};
use crate::txn::{self, AuxOpKind, BusTxnWire};
use crate::value::{self, Value, ValueShape};

/// The context every failure in [`every_sequencer_failure`] is filed under: a
/// phase, a servo, and a register.
pub(crate) fn failure_context() -> StepContext {
    StepContext::reg(SeqStepKind::Provision, 12, RegId::OperatingMode)
}

/// One [`SeqError`] of every kind, so a sweep over them is exhaustive by count
/// as well as by construction.
///
/// Shared rather than written per test module, because two tables would let one
/// module's sweep miss a failure the other's covers while both stayed green.
/// Every payload field carries a number distinct from its neighbours, so a
/// crossing that swaps two of them fails rather than passing on symmetry.
pub(crate) fn every_sequencer_failure() -> Vec<SeqError> {
    let context = failure_context();
    let readings = [11.4, 11.5, 11.6, 11.7, 11.8, 11.9, 12.0, 12.1, 12.2];
    vec![
        SeqError::NoAnswer { context },
        SeqError::DriverRefused { context },
        SeqError::Refused { context, code: 1 },
        SeqError::WireCorrupt { context },
        SeqError::PendingUnreadable { context },
        SeqError::VerdictUnreadable { context },
        SeqError::ClockOutOfRange { context },
        SeqError::RecordUnreadable { context },
        SeqError::RecordAbsent { context },
        SeqError::VerifyMismatch {
            context,
            expected: value::u8(1),
            read_back: value::u8(0),
        },
        SeqError::WrongAnswer {
            context,
            expected: AnswerShape::Value,
            observed: AnswerShape::Pinged,
        },
        SeqError::WrongValue {
            context,
            expected: ValueShape::U8,
            observed: ValueShape::U16,
        },
        SeqError::UnplaceableAngle {
            context,
            joint: JointRef::Leg0,
            angle: f64::NAN,
        },
        SeqError::AbsentServos {
            context,
            absent: AbsentSet::new(&[10, 11, 12, 13, 14, 15, 16, 17, 18], &[true; ROW_COUNT]),
        },
        SeqError::IdentityMismatch {
            context,
            model: 1,
            expected: 2,
        },
        SeqError::ProvisionMismatch {
            context,
            expected: value::u16(3),
            observed: value::u16(4),
        },
        SeqError::VoltageLow {
            context,
            readings,
            lowest: 10.0,
            limit: 11.0,
            waited: Duration::from_secs(2),
        },
        SeqError::SupplyBelowFloor {
            context,
            readings,
            lowest: 10.0,
            limit: 11.0,
        },
        SeqError::UnhealthyServo { context, bits: 4 },
        SeqError::RestPoseImplausible {
            context,
            cause: reachy_kin::FkError::WrongAssemblyMode {
                cone_deg: 90.0,
                z: 0.0,
            },
        },
        SeqError::PinnedPoseUnsolvable {
            context,
            cause: reachy_kin::FkError::NoConvergence {
                iters: 40,
                residual: 0.004,
            },
        },
    ]
}

/// One [`Fault`] of every kind a machine can stand parked on, each carrying
/// numbers a slot has to bring back.
///
/// Shared rather than written per test module, because two tables would let one
/// module's sweep miss a fault the other's covers while both stayed green. Every
/// payload field carries a number distinct from its neighbours, so a flattening
/// that swaps two of them fails rather than passing on symmetry.
pub(crate) fn every_fault() -> Vec<Fault> {
    vec![
        Fault::AntennaObstructed {
            joint: JointRef::AntennaLeft,
            error: -0.418,
        },
        Fault::AntennaServoFault {
            joint: JointRef::AntennaRight,
            id: 71,
            bits: 0b0010_1001,
        },
        Fault::HeadObstructed {
            joint: JointRef::Leg3,
            error: 0.204,
        },
        Fault::HeadServoFault {
            joint: JointRef::BodyYaw,
            id: 10,
            bits: 0b1000_0000,
        },
        Fault::PositionFeedbackLost { misses: 51 },
        Fault::MeasuredPoseInvalid {
            failures: 7,
            source: reachy_kin::FkError::NoConvergence {
                iters: u32::MAX,
                residual: 3.75e-4,
            },
        },
        Fault::BusFailure {
            source: BusFailureSource::Transaction {
                id: 12,
                kind: WireFailure::NotWritten,
            },
        },
        Fault::TorqueOffUnconfirmed { id: 9 },
    ]
}

/// How far inside its travel window a pinned leg lands, degrees.
///
/// Arming pins a leg the measured pose puts outside its travel window at the
/// nearer bound of the *provisioned* window the servo itself enforces, and those
/// bounds sit between 0.012° and 0.039° inside the corresponding envelope bound.
/// The tightest of them is the case worth modelling, and a plain angle is all
/// that is needed for it — nothing in this crate knows what a count is.
pub(crate) const WINDOW_INSET_DEG: f64 = 0.012;

/// The servo-side travel windows: `env`'s own windows, drawn in by that inset.
pub(crate) fn leg_windows(env: &EnvelopeConfig) -> [(f64, f64); 6] {
    let inset = WINDOW_INSET_DEG.to_radians();
    let mut windows = env.crank_windows;
    for (low, high) in &mut windows {
        *low += inset;
        *high -= inset;
    }
    windows
}

/// The torque-on path's configuration against the fences `env` implies.
pub(crate) fn arm_config(env: &EnvelopeConfig) -> ArmConfig {
    ArmConfig {
        ids: SERVO_IDS,
        expected: ProvisionTable::new(),
        min_arm_voltage: DEFAULT_MIN_ARM_VOLTAGE,
        voltage_poll_period: DEFAULT_VOLTAGE_POLL_PERIOD,
        voltage_budget: DEFAULT_VOLTAGE_BUDGET,
        gains: DEFAULT_GAINS,
        profile: ProfileConfig {
            acceleration: 20,
            velocity: 50,
        },
        leg_windows: leg_windows(env),
    }
}

/// Transactions and waits one scripted sequence may take before the driver
/// gives up. Far above what any sequence here needs, so an exhausted budget
/// means a sequencer that never terminates rather than a fixture that grew.
pub(crate) const STEP_BUDGET: usize = 8192;

/// The bus a sequencer talks to, scripted.
///
/// Each sequencer's tests keep their own machine — arming's knobs are
/// arm-shaped — and implement this to be driven by the loop below.
pub(crate) trait ScriptedBus {
    /// What the machine answers `txn`, issued during `step`.
    fn answer(&mut self, step: SeqStepKind, txn: &BusTxnWire) -> BusResult;

    /// The clock jumping from `now` to `until`. Recorded by the fixtures that
    /// assert how a sequence waits, ignored by the rest.
    fn waited(&mut self, _now: Duration, _until: Duration) {}
}

/// A transaction as a scripted machine reads it: what it asks for, where it is
/// addressed, and the value it carries.
///
/// The fixtures' log entry, and the one reading of a record any of them makes,
/// so a machine and an assertion cannot disagree about what one says.
#[derive(Clone, Copy, Debug, PartialEq)]
pub(crate) struct Asked {
    /// Which transaction it is.
    pub op: AuxOpKind,
    /// The phase, the servo, and the register it names.
    pub context: StepContext,
    /// What it writes, or [`value::NONE`].
    pub value: Value,
}

impl Asked {
    /// The servo it is addressed to.
    pub(crate) fn id(&self) -> u8 {
        self.context.id
    }

    /// The register it names, where it names one.
    pub(crate) fn reg(&self) -> Option<RegId> {
        self.context.reg
    }
}

/// What `txn`, issued during `step`, asks for.
///
/// A transaction a sequencer emitted always reads; the panic is a fixture
/// handing over a record no sequencer built.
pub(crate) fn asked(txn: &BusTxnWire, step: SeqStepKind) -> Asked {
    let op = txn::op(txn).expect("a transaction a sequencer emitted names an operation");
    let (context, value) = txn::read(txn, step).expect("and reads");
    Asked { op, context, value }
}

/// The pump's loop, against a scripted machine instead of a port.
///
/// One copy of the driver contract: a transaction is answered and handed back,
/// a wait advances the clock and clears the prior result, and either terminal
/// action ends the run. A second copy could drift from the real pump — and from
/// the other sequencer's copy — while both stayed green.
pub(crate) fn drive<S: Sequencer, M: ScriptedBus>(
    seq: &mut S,
    machine: &mut M,
) -> Result<S::Summary, SeqError> {
    let mut now = Duration::ZERO;
    let mut prior = None;
    for _ in 0..STEP_BUDGET {
        match seq.next(now, prior.as_ref()) {
            SeqAction::Transact => {
                // The action says only that there is one; the request itself is
                // the sequencer's pending record, so the two have to agree or
                // the fixture is answering a datagram nobody sent.
                assert!(
                    txn::active(seq.pending()),
                    "a transaction with nothing outstanding"
                );
                assert_ne!(
                    txn::op(seq.pending()),
                    Some(AuxOpKind::None),
                    "a transaction that asks for nothing"
                );
                let phase = seq.step();
                prior = Some(machine.answer(phase, seq.pending()));
            }
            SeqAction::Wait { until } => {
                assert!(
                    !txn::active(seq.pending()),
                    "a wait with a transaction still outstanding"
                );
                assert!(until > now, "a wait that does not advance the clock");
                machine.waited(now, until);
                now = until;
                prior = None;
            }
            SeqAction::Done(summary) => {
                assert!(
                    !txn::active(seq.pending()),
                    "a finished sequence with a transaction outstanding"
                );
                return Ok(summary);
            }
            SeqAction::Fail(error) => return Err(error),
        }
    }
    panic!("the sequence did not terminate");
}

/// A sequencer whose state is the slot its host hands it, with the configuration
/// its `resume` needs.
///
/// Implemented beside each sequencer, because the configuration a sequence is
/// resumed against is its own module's business. What this buys is one statement
/// of the resume law for all four sequencers rather than a pump apiece.
pub(crate) trait Resumed {
    /// The slot type the state lives in, as a host holds it.
    type Slot;

    /// What the sequence hands back when it finishes.
    type Summary;

    /// The sequencer, borrowing that slot for one step.
    type Seq<'a>: Sequencer<Summary = Self::Summary>
    where
        Self: 'a;

    /// The sequence `slot` holds, resumed against this host's configuration.
    ///
    /// # Panics
    ///
    /// If the slot does not validate, or holds a state no sequence of steps
    /// reaches — which is exactly what the law below is asserting cannot happen
    /// between two steps of a real sequence.
    fn resume<'a>(&'a self, slot: &'a mut Self::Slot) -> Self::Seq<'a>;
}

/// One sequencer's [`Resumed`] host, declared where the sequencer is.
///
/// The four hosts differ in three things: what their `resume` is handed besides
/// the slot, which slot and summary types they name, and how the sequencer is
/// built. Everything else — the associated types, the boundary validation, and
/// the two expectations the law rests on (a sequencer writes what its schema
/// declares, and a state a sequence reached resumes) — is the same in all four
/// and is stated here once, so a fifth sequencer's host is one call.
///
/// The config fields are borrowed for the host's own lifetime and named in the
/// resume expression as `$cfg.<field>`.
macro_rules! resumed {
    (
        $(#[$meta:meta])*
        struct $name:ident { $($field:ident: $fty:ty),+ $(,)? }
        slot = $slot:ty, summary = $summary:ty, seq = $seq:ident,
        resume($cfg:ident, $state:ident) = $build:expr;
    ) => {
        $(#[$meta])*
        struct $name<'c> {
            $($field: &'c $fty,)+
        }

        impl $crate::testutil::Resumed for $name<'_> {
            type Slot = $slot;
            type Summary = $summary;
            type Seq<'a>
                = $seq<'a>
            where
                Self: 'a;

            fn resume<'a>(&'a self, slot: &'a mut $slot) -> $seq<'a> {
                let $cfg = self;
                let $state = slot
                    .validate_mut()
                    .expect("a sequencer writes what its schema declares");
                $build.expect("a state a sequence reached resumes")
            }
        }
    };
}

/// The resume law: the same pump as [`drive`], with the sequencer built afresh
/// out of the slot before every step.
///
/// One copy of what a slot-crossing host does. The state between two steps is
/// the slot and nothing else, so a step that leaves a state its own `resume`
/// refuses — a phase-and-cursor pairing, a stray failure, a field a phase owns
/// and left behind — stops the run at the step that reached it rather than on
/// the first host that puts the sequence down between two transactions.
///
/// The clock starts at `since_boot` rather than at zero: a phase that measures
/// from the moment it was entered writes a non-zero moment on any host with
/// uptime, and that is the value a crossing has to carry.
pub(crate) fn drive_from_slot<R: Resumed, M: ScriptedBus>(
    host: &R,
    slot: &mut R::Slot,
    machine: &mut M,
    since_boot: Duration,
) -> Result<R::Summary, SeqError> {
    let mut now = since_boot;
    let mut prior = None;
    for _ in 0..STEP_BUDGET {
        let mut seq = host.resume(slot);
        match seq.next(now, prior.as_ref()) {
            SeqAction::Transact => {
                let step = seq.step();
                prior = Some(machine.answer(step, seq.pending()));
            }
            SeqAction::Wait { until } => {
                machine.waited(now, until);
                now = until;
                prior = None;
            }
            SeqAction::Done(summary) => return Ok(summary),
            SeqAction::Fail(error) => return Err(error),
        }
    }
    panic!("the sequence did not terminate");
}
