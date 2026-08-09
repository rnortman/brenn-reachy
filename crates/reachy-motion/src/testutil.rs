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
use crate::seq::{BusRequest, BusResult, SeqAction, SeqError, SeqStep, Sequencer};

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
const STEP_BUDGET: usize = 8192;

/// The bus a sequencer talks to, scripted.
///
/// Each sequencer's tests keep their own machine — arming's knobs are
/// arm-shaped — and implement this to be driven by the loop below.
pub(crate) trait ScriptedBus {
    /// What the machine answers `request`, issued during `step`.
    fn answer(&mut self, step: SeqStep, request: BusRequest) -> BusResult;

    /// The clock jumping from `now` to `until`. Recorded by the fixtures that
    /// assert how a sequence waits, ignored by the rest.
    fn waited(&mut self, _now: Duration, _until: Duration) {}
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
            SeqAction::Transact(request) => {
                let step = seq.step();
                prior = Some(machine.answer(step, request));
            }
            SeqAction::Wait { until } => {
                assert!(until > now, "a wait that does not advance the clock");
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
