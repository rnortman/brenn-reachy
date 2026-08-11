//! `reachy-motion` — what to command next, decided without touching anything.
//!
//! Trajectory shaping, the per-tick control step, and the arm and disarm
//! sequences. This is the crate a host harness calls once per tick, and it is
//! sans-I/O in the strong sense: time arrives as a parameter, present positions
//! arrive as a parameter, state lives in a struct the caller allocates, and the
//! result is a set of goals written into a caller-provided output. Nothing here
//! blocks, sleeps, allocates per tick, or reads a clock.
//!
//! It depends on `reachy-kin` and on nothing else in the workspace. Registers,
//! counts and frames belong to the bus; this crate speaks joint angles and
//! abstract requests, and something lower down maps those onto the wire. That
//! boundary is the reason this code can be hosted by a middleware harness with no
//! serial port in sight, and tested end to end against scripted replies.
//!
//! Two shapes, because the work has two shapes:
//!
//! - **The tick** is one pure function per control period: ingest the present
//!   positions, check health, advance the active trajectory, run the envelope
//!   check on the sampled target, guard the per-tick step size, and emit goals.
//!   Failure at any stage stops commanding and reports a typed cause.
//! - **The sequencers** are state machines for the multi-transaction procedures
//!   that do not fit one read and one write — commissioning touches many
//!   registers with read-backs. They yield one abstract request at a time and
//!   are driven to completion by whatever owns the port.
//!
//! The failure policy is the fault doctrine in `docs/fault-management.md`: the
//! minimum risk condition is head stowed, motors unpowered, and a fault reaches
//! it by writing torque off immediately, best-effort, per servo. Holding torque
//! is never a fault response, and nothing anywhere may refuse or condition a
//! torque-off write. Torque *on* is gated, minimally: the supply floor and the
//! latched error bits, both in `arm::engage_gates`, and nothing else — where the
//! machine happens to be standing is never among them. The bits gate by group:
//! a servo that carries the head refuses the engagement, an antenna is left out
//! of service for it.
//!
//! What was found and what was done about it are reported through one channel,
//! `timeline`: the session keeps an append-only record of every fault raised
//! and every maneuver that answered one, typed, readable while it runs and
//! deliverable to a subscriber as it grows. An operator line, a status file and
//! an alert are all renderings of those entries; none of them is the record.
//!
//! Motion is shaped host-side because the servos have none of their own: a goal
//! position is applied as an immediate step. Every gentle movement in this system
//! is an interpolation computed here, checked against the envelope on each tick,
//! and emitted as a bounded increment.

#![forbid(unsafe_code)]

pub mod arm;
pub mod disarm;
pub mod joints;
pub mod seq;
#[cfg(test)]
mod testutil;
pub mod tick;
pub mod timeline;
pub mod traj;

pub use arm::{
    ArmConfig, ArmRecord, CommissionSequencer, CommissionSummary, EXPECTED_MODELS,
    EXPECTED_OPERATING_MODES, EngageSequencer, EngageSummary, Gains, GroupGains, PinOutcome,
    PollCadence, PollSequencer, Posture, ProfileConfig, ProvisionExpect, ProvisionReadings,
    ProvisionTable, Rail, VENDOR_HOMING_OFFSETS, engage_gates, pin_goals,
};
pub use disarm::{
    DisarmConfig, DisarmSequencer, DisarmSummary, ReleaseForm, at_stow, stow_targets,
};
pub use joints::{
    JointGroup, JointId, JointSet, JointStep, JointTargets, JointVector, ServoHealth,
};
pub use seq::{
    AbsentSet, AnswerKind, BusRequest, BusResult, RegId, RegValue, SeqAction, SeqError, SeqStep,
    Sequencer, StepContext, ValueKind,
};
pub use tick::{
    ANTENNA_GOAL_MAX_RAD, ANTENNA_GOAL_MIN_RAD, ANTENNA_OUTBOARD, ClockStretch, CommandDisposition,
    CommandRejection, FLOOR_TICK_HZ, Fault, HEAD_GROUP_FLOOR_S, MIN_JERK_PEAK_RATE, Mode,
    MotionCommand, MotionConfig, MotionState, MoveAbort, Response, TickInputs, TickOutputs,
    TickReport, TrackingFaultConfig, duration_floor_s, floor_move_clock, motion_tick,
};
pub use timeline::{Entry, FaultTimeline, Maneuver, Outcome};
pub use traj::{MoveDurations, Trajectory, TrajectoryError, Warp};
