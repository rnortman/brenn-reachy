//! `reachy-motion` — what to command next, decided without touching anything.
//!
//! Trajectory shaping, the per-tick control step, and the arm and disarm
//! sequences. This is the crate a host harness calls once per tick, and it is
//! sans-I/O in the strong sense: time arrives as a parameter, present positions
//! arrive as a parameter, state lives in a struct the caller allocates, and the
//! result is a set of goals written into a caller-provided output. Nothing here
//! blocks, sleeps, allocates per tick, or reads a clock.
//!
//! It depends on `reachy-kin` and on the vocabulary crates the schemas generate
//! — plain data over `clockwork-rs`, so the purity above holds. Counts,
//! register addresses and frames belong to the bus; this crate speaks joint
//! angles, engineering units and abstract requests, and something lower down
//! maps those onto the servo wire. That boundary is the reason this code can be
//! hosted by a middleware harness with no serial port in sight, and tested end to
//! end against scripted replies.
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
//! What a sequencer asks for is the vocabulary's own transaction record
//! ([`txn`]), the same bytes a slot holds and a datagram will carry, so a
//! simulated driver and a real one cannot read one differently.
//!
//! [`winddown`] is the third shape: the decision core of the controlled
//! responses, one action per call, so a blocking bench loop and a cog woken
//! every hundred milliseconds carry out the same maneuver.
//!
//! The failure policy is the fault doctrine in `docs/fault-management.md`: the
//! minimum risk condition is head stowed, motors unpowered, and a fault reaches
//! it by the maneuver its response names — a stow under control where the
//! motors still command, an immediate best-effort torque-off where they cannot
//! be trusted to, and either of those scoped to the group that failed where the
//! rest of the machine is sound. Holding torque
//! is never a fault response, and nothing anywhere may refuse or condition a
//! torque-off write. Torque *on* is gated, minimally: the supply floor and the
//! latched error bits, both in `arm::engage_gates`, and nothing else — where the
//! machine happens to be standing is never among them. The bits gate by group:
//! a servo that carries the head refuses the engagement, an antenna is left out
//! of service for it.
//!
//! What was found and what was done about it is reported once, at the raise, and
//! travels as the value it was classified as: a fault on the tick's report, a
//! maneuver's ending on the wind-down's. The session's own narration -- one row
//! per thing worth saying -- is the timeline it keeps in its slot
//! (`motion/timeline.clk`). An operator line, a status file and an alert are all
//! renderings of those values; none of them is the record.
//!
//! Motion is shaped host-side because the servos have none of their own: a goal
//! position is applied as an immediate step. Every gentle movement in this system
//! is an interpolation computed here, checked against the envelope on each tick,
//! and emitted as a bounded increment.

#![forbid(unsafe_code)]

// Ahead of the modules that use it, out of alphabetical order on purpose:
// `macro_rules!` scoping is textual, so `vocab_name!` is in scope only for
// modules declared after this one.
#[macro_use]
pub mod vocab;

// `#[macro_use]`, and ahead of the modules that use it: `phase_state!` is the
// scaffolding a schema-resident sequencer carries, and `macro_rules!` scoping is
// textual, so it is in scope only for modules declared after this one.
#[macro_use]
pub mod resume;

// Likewise, and for the same reason: `resumed!` declares a test's slot-resuming
// host beside the sequencer it drives.
#[cfg(test)]
#[macro_use]
mod testutil;

pub mod arm;
pub mod cells;
pub mod disarm;
pub mod fault;
pub mod joints;
pub mod phase;
pub mod postures;
pub mod record;
pub mod seq;
pub mod snap;
pub mod tick;
pub mod traj;
pub mod txn;
pub mod value;
pub mod verdict;
pub mod winddown;

pub use arm::{
    ArmConfig, ArmRecord, CommissionSequencer, CommissionSummary, EXPECTED_MODELS,
    EXPECTED_OPERATING_MODES, EngageSequencer, EngageSummary, Gains, GroupGains, PinOutcome,
    PollCadence, PollSequencer, Posture, ProfileConfig, ProvisionExpect, ProvisionTable, Rail,
    VENDOR_HOMING_OFFSETS, engage_gates, pin_goals, rest_pose_seeds,
};
pub use disarm::{
    DisarmConfig, DisarmSequencer, DisarmSummary, ReleaseForm, at_stow, stow_targets,
};
pub use fault::{FaultError, FaultKind};
pub use joints::{JointGroup, JointRef, JointStep, JointTargets, JointVector, ServoHealth};
pub use phase::{
    ANTENNA_CONTACT_BAND_RAD, ANTENNA_PHASE_SEPARATION_RAD, AntennaPhaseConfig, PhaseSeparation,
    PhaseWatch, mirror_offset,
};
pub use postures::{neutral_targets, stow_pose_targets};
pub use resume::{GAINS_PROFILE_WRITES, PROVISION_CELLS, ResumeError};
pub use seq::{
    AbsentSet, AnswerShape, BusResult, RegId, SeqAction, SeqError, SeqFailureKind, SeqStepKind,
    Sequencer, StepContext, answer, failure, reg, step,
};
pub use snap::{
    BusSourceKind, DurationError, FkFailureKind, FkFieldError, MotionMode, PoseSnapshot,
    PoseSnapshotError, TrackingSideKind, duration_from_nanos, duration_nanos, fk_cause, fk_fields,
};
pub use tick::{
    ANTENNA_GOAL_MAX_RAD, ANTENNA_GOAL_MIN_RAD, ANTENNA_OUTBOARD, BusFailureSource, ClockStretch,
    CommandDisposition, CommandRejection, DryPassPeaks, FLOOR_TICK_HZ, Fault, HEAD_GROUP_FLOOR_S,
    MIN_JERK_PEAK_RATE, MotionCommand, MotionConfig, MotionSnap, MotionSnapWire, MoveAbort,
    ResponseKind, StateError, TickInputs, TickOutputs, TickReport, TrackingFaultConfig,
    TrackingLook, WireFailure, YAW_GOAL_COUNT_MAX, arm, default_motion_config, dry_pass_peaks,
    dry_pass_separation, duration_floor_s, floor_move_clock, last_goal, last_targets, motion_tick,
    plan_move, resume, standing_fault, tracking, yaw_goal_counts,
};
pub use traj::{MoveDurations, SeedError, Trajectory, TrajectoryError, WarpKind};
pub use txn::{AuxOpKind, BusTxn, BusTxnWire};
pub use value::{ShapeName, Shown, Value, ValueShape};
pub use verdict::VerdictError;
pub use winddown::{
    Disposition, EndingKind, Evidence, Maneuver, StowEnding, WindDown, WindDownAction,
    WindDownError, WindDownOutcome, ending, within,
};
