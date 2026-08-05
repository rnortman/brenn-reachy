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
//!   that do not fit one read and one write — arming touches many registers with
//!   read-backs. They yield one abstract request at a time and are driven to
//!   completion by whatever owns the port.
//!
//! The failure policy is deliberate and is not the platform's usual one. Elsewhere
//! "better dead than wrong" means stop; here stopping the wrong way means the head
//! falls. So a fault **stops commanding and holds torque**, reports loudly, and
//! stays faulted until an operator says otherwise. It never auto-recovers and never
//! auto-releases. Releasing is always an explicit command, and always documented as
//! dropping the head unless it is already stowed.
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
pub mod traj;

pub use arm::{
    ArmConfig, ArmRecord, ArmSequencer, ArmSummary, Gains, GroupGains, PinOutcome, ProfileConfig,
    ProvisionExpect, ProvisionReadings, ProvisionTable, pin_goals,
};
pub use disarm::{DisarmConfig, DisarmSequencer, DisarmSummary, stow_targets};
pub use joints::{JointId, JointStep, JointTargets, JointVector, ServoHealth};
pub use seq::{
    AbsentSet, AnswerKind, BusRequest, BusResult, RegId, RegValue, SeqAction, SeqError, SeqStep,
    Sequencer, StepContext, ValueKind,
};
pub use tick::{
    CommandDisposition, CommandRejection, Fault, Mode, MotionCommand, MotionConfig, MotionState,
    TickInputs, TickOutputs, TickReport, TrackingFaultConfig, motion_tick,
};
pub use traj::{Trajectory, TrajectoryError, Warp};
