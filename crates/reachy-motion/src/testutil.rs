//! Fixtures shared by this crate's tests.
//!
//! The servo-side travel windows and the arming configuration drawn from them,
//! in one place. Two copies would let the tick's tests and the sequencer's model
//! different fences while both stayed green, which is the one failure a shared
//! scenario exists to prevent.

use reachy_kin::EnvelopeConfig;

use crate::arm::{
    ArmConfig, DEFAULT_GAINS, DEFAULT_MAX_PIN_PULL_IN, DEFAULT_MIN_ARM_VOLTAGE,
    DEFAULT_REPIN_TOLERANCE, DEFAULT_VOLTAGE_BUDGET, DEFAULT_VOLTAGE_POLL_PERIOD, ProfileConfig,
    ProvisionTable, SERVO_IDS,
};

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

/// Arming's configuration against the fences `env` implies.
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
        max_pin_pull_in: DEFAULT_MAX_PIN_PULL_IN,
        repin_tolerance: DEFAULT_REPIN_TOLERANCE,
        leg_windows: leg_windows(env),
    }
}
