//! The tick: one control step, decided and handed back, never executed here.
//!
//! [`motion_tick`] is the whole control law. It takes the configuration, the
//! state, and what this period measured, and it writes what to command into a
//! caller-provided output. It reads no clock, owns no port, allocates nothing
//! and blocks on nothing; the caller owns the loop, the timing and the wire.
//!
//! ## What one tick does
//!
//! Ingest the present positions and solve the head pose from them; check the
//! servos' health when the slower health poll ran; take at most one command;
//! advance the active trajectory by one sample; check that sample against the
//! envelope; guard the per-joint step size; emit goals if they changed.
//!
//! ## The failure policy
//!
//! Any stage can raise a [`Fault`], and a fault is absorbing: the tick stops
//! commanding and every subsequent tick emits nothing. It never cuts torque —
//! that drops the head — and it never recovers on its own. The servos hold their
//! last goal indefinitely, which is exactly what a stopped tick wants from them,
//! and releasing is an operator action taken with full knowledge of what it
//! does.
//!
//! Two things are deliberately *not* faults. A command whose target fails the
//! envelope is **rejected** and reported: an armed, holding machine must not be
//! bricked by someone typing a pose it cannot reach. A command that arrives
//! while a move is running is rejected the same way. A sampled *path* pose that
//! fails the envelope after its target passed is a different matter — the
//! checker and the interpolation have disagreed about a pose already accepted —
//! and that is a fault.
//!
//! ## Freshness
//!
//! A position read that did not arrive is `None`, never the previous tick's
//! numbers, and a read carrying a value nobody can place is a fault naming the
//! joint it arrived on rather than an input to the solvers. A stale tick is
//! marked stale in the report and counts toward the read-loss fault; nothing
//! downstream ever sees a stale reading presented as a live one. The same
//! all-or-nothing rule governs the health poll.
//!
//! A stale tick does **not** stop an active move. It skips the tracking
//! comparison — the one thing that needs a live reading — and otherwise advances,
//! checks and emits as usual, so a validated move rides out a brief read outage
//! instead of stalling halfway. What bounds that is `read_loss_ticks`: it is the
//! length of the outage the machine may keep moving blind through, and sizing it
//! is sizing exactly that.

use core::time::Duration;

use nalgebra::Isometry3;
use reachy_kin::{
    EnvelopeConfig, EnvelopeReport, EnvelopeViolations, FkError, FkOptions, FkStats, HeadGeometry,
    LegAngles, check_envelope, forward_kinematics, min_pose_margin, outside_limit,
};
use thiserror::Error;

use crate::joints::{JointId, JointStep, JointTargets, JointVector};
use crate::traj::{Trajectory, TrajectoryError, Warp};

/// One servo's hardware-error byte, paired with the bus ID it was read from.
///
/// The tick names the offending servo by its bus ID, so the ID travels with the
/// bits rather than being inferred from a position in an array. Whatever owns
/// the port fills these in; this crate never learns what an ID means.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ServoHealth {
    /// The servo's bus ID.
    pub id: u8,
    /// The hardware-error byte as read.
    pub bits: u8,
}

impl ServoHealth {
    /// Bit 0: the input voltage left its configured range at some point.
    pub const INPUT_VOLTAGE: u8 = 1;

    /// Whether the byte is clear, or carries the input-voltage bit and nothing
    /// else.
    ///
    /// The voltage bit alone is expected on this platform and is reported
    /// rather than acted on: it latches on a supply dip that the servo rode out,
    /// and every other bit means something is wrong with the motor.
    #[must_use]
    pub fn healthy_or_voltage_only(self) -> bool {
        self.bits & !Self::INPUT_VOLTAGE == 0
    }

    /// Whether the input-voltage bit is set and nothing else is — the
    /// informational case.
    #[must_use]
    pub fn voltage_only(self) -> bool {
        self.bits == Self::INPUT_VOLTAGE
    }
}

/// When a joint is far enough from its goal, for long enough, to conclude the
/// servo is not tracking it.
///
/// Goal writes to the whole group are unacknowledged by the protocol, so a
/// write that never applied leaves no trace on the bus. This comparison is the
/// compensating detection: a goal that is not being followed shows up as a
/// standing position error whether the write landed or the motor stalled.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackingFaultConfig {
    /// How far a joint may sit from its goal without counting, radians.
    pub threshold_rad: f64,
    /// How many consecutive ticks of that before the fault.
    pub ticks: u32,
}

impl Default for TrackingFaultConfig {
    /// Provisional bench figures, to be replaced by measured ones: no lag this
    /// large has been observed on this linkage, because nothing has yet
    /// measured one.
    fn default() -> Self {
        Self {
            // 8.6° of crank, well past anything the position gains should
            // permit under the head's own weight.
            threshold_rad: 0.15,
            // A fifth of a second at the bench's 50 Hz tick, so a single
            // transient on one read cannot raise it.
            ticks: 10,
        }
    }
}

/// Everything the tick needs that does not change between ticks.
#[derive(Clone, Debug)]
pub struct MotionConfig {
    /// Link lengths and per-leg frames.
    pub geom: HeadGeometry,
    /// The bounds every commanded pose is checked against.
    pub env: EnvelopeConfig,
    /// Budget and screen for the present-pose solve.
    pub fk: FkOptions,
    /// Per-tick step bounds. Exceeding one is a fault, never a clamp.
    pub max_step: JointStep,
    /// When to call tracking lost.
    pub tracking: TrackingFaultConfig,
    /// Consecutive ticks without a position read before the read-loss fault.
    pub read_loss_ticks: u32,
}

impl Default for MotionConfig {
    /// Defaults sized against the bench's 50 Hz tick and the moves this
    /// milestone makes; the bench's configuration file overrides all of them.
    fn default() -> Self {
        Self {
            geom: HeadGeometry::default(),
            env: EnvelopeConfig::default(),
            fk: FkOptions::default(),
            // The shaped move's peak rate is 1.875 times its average, so a two
            // second move spanning a crank's whole 2.2 rad window peaks at
            // about 0.041 rad per tick at 50 Hz. The milestone's moves are a
            // fraction of that; 0.05 leaves the fastest legitimate one room and
            // still catches a step the linkage would take as a slam.
            max_step: JointStep {
                legs: 0.05,
                body_yaw: 0.05,
                // The antennas sweep the full 6.1 rad between their stow
                // positions, which over two seconds peaks near 0.114.
                antennas: 0.15,
            },
            tracking: TrackingFaultConfig::default(),
            // One second of silence at 50 Hz.
            read_loss_ticks: 50,
        }
    }
}

/// What the tick is doing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mode {
    /// No trajectory running; the servos hold their last goal.
    Holding,
    /// A trajectory is being sampled, having started at this time on the
    /// caller's clock.
    Moving {
        /// The `now` of the tick that accepted the move.
        started: Duration,
    },
    /// Stopped commanding, for this reason, until an operator intervenes.
    Faulted(Fault),
}

/// Why the tick stopped commanding.
///
/// Every variant means the same thing operationally: no more goals go out, the
/// servos hold what they have, and nothing here will change that. They differ
/// in what they tell the operator went wrong.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum Fault {
    /// A sampled path pose failed the envelope after its target had passed it.
    #[error("the commanded path left the envelope: {0}")]
    Envelope(EnvelopeViolations),
    /// One joint's goal would have moved further in one tick than the step
    /// bound allows — an interpolator or a seed is wrong, and the servo would
    /// take the difference as an immediate jump.
    #[error("{joint} would step {delta:.4} rad in one tick")]
    StepTooLarge {
        /// The joint whose step was too large.
        joint: JointId,
        /// How far it would have moved, radians.
        delta: f64,
    },
    /// Too many consecutive ticks with no position read.
    #[error("no position read for {misses} consecutive ticks")]
    ReadLoss {
        /// Consecutive missed reads.
        misses: u32,
    },
    /// A joint sat too far from its goal for too long.
    #[error("{joint} is {error:.4} rad from its goal and not closing")]
    TrackingLost {
        /// The joint furthest from its goal when the count ran out.
        joint: JointId,
        /// How far, radians.
        error: f64,
    },
    /// A servo reported a hardware error beyond the input-voltage bit. Never
    /// rebooted automatically: a reboot drops the head.
    #[error("servo {id} reports hardware error bits {bits:#04x}")]
    HardwareError {
        /// The reporting servo's bus ID.
        id: u8,
        /// Its hardware-error byte.
        bits: u8,
    },
    /// The measured crank angles yielded no believable head pose, so there is
    /// nothing to command from. Tried once, from the previous tick's pose; the
    /// solver is never re-run on perturbed inputs.
    #[error("present pose unknown: {0}")]
    PresentPoseLost(FkError),
    /// A position read carried a value that is not a number. Nothing is
    /// commanded from it: an angle nobody can place is a corrupt read, and the
    /// layer that produced it is the thing to look at.
    #[error("the position read for {joint} is not a number")]
    PresentNotFinite {
        /// The first joint in bus order whose reading could not be placed.
        joint: JointId,
    },
}

/// What a caller asks the tick to do.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MotionCommand {
    /// Move to `target` over `duration`, shaped by `warp`.
    MoveTo {
        /// Where to end up.
        target: JointTargets,
        /// How long to take.
        duration: Duration,
        /// How to shape it.
        warp: Warp,
    },
    /// Abandon any active move and hold where the last goal put things.
    Hold,
}

/// Why a command was refused. A refusal changes nothing: the machine stays in
/// whatever mode it was already in.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum CommandRejection {
    /// A move was already running. One motion at a time; nothing here queues or
    /// blends.
    #[error("a move is already running")]
    AlreadyMoving,
    /// The commanded target is not a pose this machine may hold.
    #[error("the commanded target is outside the envelope: {0}")]
    Envelope(EnvelopeViolations),
    /// The commanded move cannot be shaped into a path.
    #[error("the commanded move cannot be shaped: {0}")]
    Trajectory(TrajectoryError),
}

/// What became of this tick's command.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum CommandDisposition {
    /// There was no command.
    #[default]
    None,
    /// A move was accepted and started on this tick.
    Started,
    /// A hold was taken.
    Held,
    /// A command was refused, and nothing changed.
    Rejected(CommandRejection),
}

/// Everything one tick decided, for the operator and for tests.
///
/// Filled from scratch on every tick, including the ticks that emit no goal:
/// what a tick did *not* do is as much of the record as what it did.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TickReport {
    /// The mode the tick left the machine in.
    pub mode: Mode,
    /// Whether this tick had a live position read. False means the numbers
    /// behind every other field in here are the previous tick's.
    pub present_fresh: bool,
    /// Consecutive ticks without a position read, this one included.
    pub misses: u32,
    /// Consecutive live ticks with some joint past the tracking threshold,
    /// including the tick that raised [`Fault::TrackingLost`]. A tick without a
    /// live read measures nothing and repeats the standing count.
    pub tracking_count: u32,
    /// The joint furthest from its goal, and by how much, when the read was
    /// live.
    pub tracking_worst: Option<(JointId, f64)>,
    /// What the present-pose solve cost, when it ran and succeeded.
    pub fk: Option<FkStats>,
    /// The present pose's smallest toggle margin — the baseline every envelope
    /// check on this tick ran against.
    pub present_min_margin: f64,
    /// The hardware-error bytes, when the health poll ran this tick. Reported
    /// verbatim, including the input-voltage bit that raises no fault.
    pub health: Option<[ServoHealth; JointId::COUNT]>,
    /// What became of the command, if there was one.
    pub command: CommandDisposition,
    /// The envelope check of the sampled path pose, when the tick advanced one.
    pub envelope: Option<EnvelopeReport>,
    /// Whether goals were emitted. Holding writes nothing; the servos hold.
    pub emitted: bool,
    /// Whether an active move reached its endpoint on this tick.
    pub completed: bool,
    /// The fault raised on this tick, or the standing one on the ticks after.
    pub fault: Option<Fault>,
}

impl Default for TickReport {
    /// An empty record. Every field is overwritten at the top of each tick, so
    /// this is what an unused output buffer looks like, not a claim about any
    /// machine.
    fn default() -> Self {
        Self {
            mode: Mode::Holding,
            present_fresh: false,
            misses: 0,
            tracking_count: 0,
            tracking_worst: None,
            fk: None,
            present_min_margin: 0.0,
            health: None,
            command: CommandDisposition::None,
            envelope: None,
            emitted: false,
            completed: false,
            fault: None,
        }
    }
}

/// What this period measured, and what it was asked to do.
#[derive(Clone, Copy, Debug)]
pub struct TickInputs<'a> {
    /// Elapsed time on the caller's own epoch, required to be non-decreasing
    /// across ticks.
    ///
    /// Nothing here reads a clock, so nothing here can enforce that. A `now` that
    /// went backwards during a move resamples the path at the earlier time and
    /// walks the head back the way it came, bounded only by the per-tick step
    /// guard; before the move's own start it resamples the start.
    pub now: Duration,
    /// The measured joint angles, or `None` when this tick's read failed.
    pub present: Option<&'a JointVector>,
    /// A command, at most one per tick.
    pub command: Option<&'a MotionCommand>,
    /// The hardware-error bytes, when the slower health poll ran this tick.
    pub health: Option<&'a [ServoHealth; JointId::COUNT]>,
}

/// What to command, and the record of how that was decided.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TickOutputs {
    /// The goals to write, or `None` to write nothing at all.
    pub goal: Option<JointVector>,
    /// What the tick did.
    pub report: TickReport,
}

/// Everything the tick carries between periods.
///
/// Private throughout: the tick is the only thing that may change any of it,
/// and a caller that could set the mode or the last goal by hand could put the
/// machine in a state no sequence of ticks produces.
#[derive(Clone, Debug)]
pub struct MotionState {
    mode: Mode,
    trajectory: Option<Trajectory>,
    last_goal: JointVector,
    last_targets: JointTargets,
    fk_seed: Isometry3<f64>,
    present_min_margin: f64,
    miss_count: u32,
    tracking_count: u32,
}

impl MotionState {
    /// The state of a machine that has just finished arming.
    ///
    /// `pinned` is what arming left in the servos' goal registers,
    /// `head_pose_body` the pose solved from the resting angles, and
    /// `min_margin` that pose's smallest toggle margin — the baseline that lets
    /// the first move lift off a rest tighter than the clearance floor.
    ///
    /// There is no other way to build one, and that is the point: a tick can
    /// only run on a machine somebody armed.
    #[must_use]
    pub fn new_armed(
        pinned: &JointVector,
        head_pose_body: &Isometry3<f64>,
        min_margin: f64,
    ) -> Self {
        Self {
            mode: Mode::Holding,
            trajectory: None,
            last_goal: *pinned,
            last_targets: JointTargets {
                head_pose_body: *head_pose_body,
                body_yaw: pinned.body_yaw,
                antennas: pinned.antennas,
            },
            fk_seed: *head_pose_body,
            present_min_margin: min_margin,
            miss_count: 0,
            tracking_count: 0,
        }
    }

    /// What the machine is doing.
    #[must_use]
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// Whether the tick has stopped commanding.
    #[must_use]
    pub fn is_faulted(&self) -> bool {
        matches!(self.mode, Mode::Faulted(_))
    }

    /// The goals last emitted, or pinned at arm time if none have been.
    #[must_use]
    pub fn last_goal(&self) -> &JointVector {
        &self.last_goal
    }

    /// The Cartesian mirror of [`Self::last_goal`], which the next move starts
    /// from.
    #[must_use]
    pub fn last_targets(&self) -> &JointTargets {
        &self.last_targets
    }

    /// The present pose's smallest toggle margin, as of the last live read.
    #[must_use]
    pub fn present_min_margin(&self) -> f64 {
        self.present_min_margin
    }

    /// The pose the next present-pose solve will be seeded from.
    #[must_use]
    pub fn fk_seed(&self) -> &Isometry3<f64> {
        &self.fk_seed
    }

    /// Stop commanding, and record why.
    fn raise(&mut self, fault: Fault, out: &mut TickOutputs) {
        self.mode = Mode::Faulted(fault);
        out.goal = None;
        out.report.mode = self.mode;
        out.report.fault = Some(fault);
    }
}

/// One control step.
///
/// `out` is overwritten in full, so a caller may reuse one buffer forever and
/// never see a field left over from a previous period.
pub fn motion_tick(
    cfg: &MotionConfig,
    state: &mut MotionState,
    inp: &TickInputs<'_>,
    out: &mut TickOutputs,
) {
    *out = TickOutputs::default();
    out.report.mode = state.mode;
    out.report.present_min_margin = state.present_min_margin;

    // A fault is absorbing. Commands are ignored and nothing is emitted; the
    // servos hold their last goal, which is the only stopped state that does
    // not drop the head.
    //
    // TODO(fault-recovery): an explicit clear-fault command belongs here, once
    // there is an operator surface to issue one from.
    if let Mode::Faulted(fault) = state.mode {
        out.report.fault = Some(fault);
        return;
    }

    // A live read gives the head pose, and with it the clearance baseline
    // every envelope check on this tick uses.
    let fresh = match inp.present {
        Some(present) => {
            state.miss_count = 0;
            out.report.present_fresh = true;
            if let Some(joint) = present.first_non_finite() {
                state.raise(Fault::PresentNotFinite { joint }, out);
                return;
            }
            let mut pose = Isometry3::identity();
            match forward_kinematics(
                &cfg.geom,
                &LegAngles(present.legs),
                &state.fk_seed,
                &cfg.fk,
                &mut pose,
            ) {
                Ok(stats) => {
                    out.report.fk = Some(stats);
                    state.fk_seed = pose;
                    state.present_min_margin = min_pose_margin(&cfg.geom, &pose);
                    out.report.present_min_margin = state.present_min_margin;
                    Some(present)
                }
                Err(error) => {
                    state.raise(Fault::PresentPoseLost(error), out);
                    return;
                }
            }
        }
        None => {
            state.miss_count += 1;
            out.report.misses = state.miss_count;
            if state.miss_count > cfg.read_loss_ticks {
                state.raise(
                    Fault::ReadLoss {
                        misses: state.miss_count,
                    },
                    out,
                );
                return;
            }
            None
        }
    };
    out.report.misses = state.miss_count;
    out.report.tracking_count = state.tracking_count;

    // Tracking, on live reads only: a stale reading compared against a fresh
    // goal is a difference nobody measured.
    if let Some(present) = fresh {
        let mut worst = (JointId::BodyYaw, f64::NEG_INFINITY);
        let mut over = false;
        for ((id, angle), (_, goal)) in present.joints().into_iter().zip(state.last_goal.joints()) {
            let error = (angle - goal).abs();
            over |= outside_limit(error, cfg.tracking.threshold_rad);
            if worse_error(error, worst.1) {
                worst = (id, error);
            }
        }
        out.report.tracking_worst = Some(worst);
        if over {
            state.tracking_count += 1;
        } else {
            state.tracking_count = 0;
        }
        // Recorded before the fault check: the tick that runs the budget out is
        // the one whose count matters most, and a report that shipped zero there
        // would read as a single-tick trip rather than a sustained breach.
        out.report.tracking_count = state.tracking_count;
        if over && state.tracking_count >= cfg.tracking.ticks {
            state.raise(
                Fault::TrackingLost {
                    joint: worst.0,
                    error: worst.1,
                },
                out,
            );
            return;
        }
    }

    // Health, when the slower poll ran. Reported in full either way; the
    // input-voltage bit alone raises nothing and is never filtered out.
    if let Some(health) = inp.health {
        out.report.health = Some(*health);
        if let Some(bad) = health.iter().find(|h| !h.healthy_or_voltage_only()) {
            state.raise(
                Fault::HardwareError {
                    id: bad.id,
                    bits: bad.bits,
                },
                out,
            );
            return;
        }
    }

    // At most one command. Refusals report and change nothing.
    if let Some(command) = inp.command {
        out.report.command = take_command(cfg, state, inp.now, command);
    }
    out.report.mode = state.mode;

    // Advance the active trajectory by one sample. Everything is copied out of
    // the trajectory here so the borrow ends before anything can fault.
    let Mode::Moving { started } = state.mode else {
        return;
    };
    let Some((sampled, endpoint, done)) = state.trajectory.as_ref().map(|trajectory| {
        let t = inp.now.saturating_sub(started);
        let mut sampled = JointTargets::default();
        trajectory.sample(t, &mut sampled);
        (sampled, *trajectory.target(), trajectory.done(t))
    }) else {
        // Moving with nothing to sample is a state no sequence of ticks
        // produces: the mode and the trajectory are set and cleared together,
        // and both are private. If one ever appears anyway, the machine drops to
        // a named state instead of staying Moving forever — where it would emit
        // nothing, refuse every command as already-moving, and report no reason.
        debug_assert!(false, "Moving with no trajectory to sample");
        state.mode = Mode::Holding;
        out.report.mode = state.mode;
        return;
    };

    let mut envelope = EnvelopeReport::default();
    let verdict = check_envelope(
        &cfg.geom,
        &cfg.env,
        &sampled.head_pose_body,
        sampled.body_yaw,
        sampled.antennas,
        Some(state.present_min_margin),
        &mut envelope,
    );
    out.report.envelope = Some(envelope);
    // A path pose that fails after its target passed means the checker and the
    // interpolation disagree about a pose already accepted. The second arm is
    // that same disagreement in its other form: a verdict that passed without
    // producing the angles it passed on.
    let (Ok(()), Some(angles)) = (verdict, envelope.leg_angles) else {
        state.raise(Fault::Envelope(envelope.violations), out);
        return;
    };
    let candidate = JointVector {
        body_yaw: sampled.body_yaw,
        legs: angles.0,
        antennas: sampled.antennas,
    };

    // Step guard. An oversized step is a slam, and the bug that produced it is
    // the thing worth reporting; it is never trimmed and sent.
    for ((id, angle), (_, last)) in candidate.joints().into_iter().zip(state.last_goal.joints()) {
        let delta = (angle - last).abs();
        if outside_limit(delta, cfg.max_step.for_joint(id)) {
            state.raise(Fault::StepTooLarge { joint: id, delta }, out);
            return;
        }
    }

    // Emit, but only what changed: holding writes nothing and the servos hold.
    out.report.emitted = candidate != state.last_goal;
    if out.report.emitted {
        out.goal = Some(candidate);
    }
    state.last_goal = candidate;
    state.last_targets = sampled;

    if done {
        // The endpoint's own bits, so the next move chains from exactly what
        // was commanded rather than from a sample near it.
        state.last_targets = endpoint;
        state.trajectory = None;
        state.mode = Mode::Holding;
        out.report.completed = true;
    }
    out.report.mode = state.mode;
}

/// Whether `candidate` is a worse tracking error than `incumbent`.
///
/// An ordering, deliberately not the bound test the threshold beside it uses: a
/// bound test treats an incomparable value as a violation, which is right for
/// "is this joint past the threshold" and wrong for "which joint is furthest
/// out" — an unplaceable error would both beat the incumbent and be beaten by
/// whatever came next, so the fault would name the joint *after* the bad one and
/// report its zero error.
///
/// An error nobody can place is the worst thing this comparison can see, so it
/// wins outright and keeps winning. Ties keep the joint found first.
fn worse_error(candidate: f64, incumbent: f64) -> bool {
    if candidate.is_nan() {
        return !incumbent.is_nan();
    }
    candidate > incumbent
}

/// Take at most one command, returning what became of it. Mutates the state
/// only when the command is accepted.
fn take_command(
    cfg: &MotionConfig,
    state: &mut MotionState,
    now: Duration,
    command: &MotionCommand,
) -> CommandDisposition {
    let MotionCommand::MoveTo {
        target,
        duration,
        warp,
    } = command
    else {
        state.trajectory = None;
        state.mode = Mode::Holding;
        return CommandDisposition::Held;
    };

    if matches!(state.mode, Mode::Moving { .. }) {
        return CommandDisposition::Rejected(CommandRejection::AlreadyMoving);
    }

    let mut report = EnvelopeReport::default();
    if let Err(error) = check_envelope(
        &cfg.geom,
        &cfg.env,
        &target.head_pose_body,
        target.body_yaw,
        target.antennas,
        Some(state.present_min_margin),
        &mut report,
    ) {
        return CommandDisposition::Rejected(CommandRejection::Envelope(error.violations));
    }

    // Start from the last commanded targets rather than the measured pose, so
    // consecutive moves chain without a step and a tracking lag is not written
    // into the path.
    match Trajectory::new(&state.last_targets, target, *duration, *warp) {
        Ok(trajectory) => {
            state.trajectory = Some(trajectory);
            state.mode = Mode::Moving { started: now };
            CommandDisposition::Started
        }
        Err(error) => CommandDisposition::Rejected(CommandRejection::Trajectory(error)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use reachy_kin::{inverse_kinematics, rest_head_pose};

    fn secs(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    /// The joint vector that holds `targets`, and that pose's clearance.
    fn joints_for(cfg: &MotionConfig, targets: &JointTargets) -> (JointVector, f64) {
        let mut angles = LegAngles::default();
        inverse_kinematics(&cfg.geom, &targets.head_pose_body, &mut angles)
            .expect("the test pose is reachable");
        (
            JointVector {
                body_yaw: targets.body_yaw,
                legs: angles.0,
                antennas: targets.antennas,
            },
            min_pose_margin(&cfg.geom, &targets.head_pose_body),
        )
    }

    /// The recorded tight resting configuration, raised or lowered by `dz`
    /// metres. At `dz = 0` its clearance is a fraction of the floor, which is
    /// the configuration the baseline policy exists for.
    fn rest_targets(dz: f64) -> JointTargets {
        let mut head_pose_body = rest_head_pose();
        head_pose_body.translation.z += dz;
        JointTargets {
            head_pose_body,
            ..JointTargets::default()
        }
    }

    /// A machine armed and holding at `targets`.
    fn armed_at(cfg: &MotionConfig, targets: &JointTargets) -> (MotionState, JointVector) {
        let (pinned, margin) = joints_for(cfg, targets);
        (
            MotionState::new_armed(&pinned, &targets.head_pose_body, margin),
            pinned,
        )
    }

    /// One tick with a live read.
    fn tick_with(
        cfg: &MotionConfig,
        state: &mut MotionState,
        now: Duration,
        present: &JointVector,
        command: Option<&MotionCommand>,
    ) -> TickOutputs {
        let mut out = TickOutputs::default();
        motion_tick(
            cfg,
            state,
            &TickInputs {
                now,
                present: Some(present),
                command,
                health: None,
            },
            &mut out,
        );
        out
    }

    fn pose_at(z: f64) -> JointTargets {
        JointTargets {
            head_pose_body: Isometry3::translation(0.0, 0.0, z),
            ..JointTargets::default()
        }
    }

    /// Run a move to completion with a perfectly tracking machine: every tick's
    /// present positions are the previous tick's goals. Returns the ticks it
    /// took and the last output.
    fn run_move(
        cfg: &MotionConfig,
        state: &mut MotionState,
        start: &JointVector,
        target: &JointTargets,
        duration: Duration,
        period: f64,
    ) -> (u32, TickOutputs) {
        let command = MotionCommand::MoveTo {
            target: *target,
            duration,
            warp: Warp::MinJerk,
        };
        let mut present = *start;
        let mut ticks = 0;
        let mut out = TickOutputs::default();
        for n in 0..1000 {
            let command = (n == 0).then_some(&command);
            out = tick_with(cfg, state, secs(f64::from(n) * period), &present, command);
            ticks += 1;
            if let Some(goal) = out.goal {
                present = goal;
            }
            if out.report.completed || out.report.fault.is_some() {
                break;
            }
        }
        (ticks, out)
    }

    /// An armed machine holding still commands nothing at all. The servos hold
    /// their pinned goals; writing them again every tick would be noise on the
    /// bus and would say nothing new.
    #[test]
    fn holding_emits_nothing() {
        let cfg = MotionConfig::default();
        let targets = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &targets);

        for n in 0..5 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &pinned, None);
            assert_eq!(out.goal, None);
            assert_eq!(out.report.mode, Mode::Holding);
            assert!(out.report.present_fresh);
            assert!(!out.report.emitted);
            assert_eq!(out.report.command, CommandDisposition::None);
            assert_eq!(out.report.fault, None);
            assert!(out.report.fk.is_some(), "the present pose was solved");
        }
        assert!(state.present_min_margin() > 0.024, "neutral clearance");
    }

    /// A move runs, emits goals, and lands on its endpoint exactly — the
    /// Cartesian target's own bits, so a following move starts where this one
    /// finished.
    #[test]
    fn a_move_runs_to_its_endpoint_and_holds_there() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let target = pose_at(0.19);

        let (ticks, out) = run_move(&cfg, &mut state, &pinned, &target, secs(2.0), 0.02);
        assert!(out.report.completed, "the move finished");
        assert_eq!(out.report.mode, Mode::Holding);
        assert_eq!(state.mode(), Mode::Holding);
        assert!((99..=102).contains(&ticks), "took {ticks} ticks");
        assert_eq!(
            state.last_targets().head_pose_body.translation.vector.z,
            0.19,
            "the endpoint is the target's own number"
        );

        let (expected, _) = joints_for(&cfg, &target);
        for leg in 0..6 {
            assert!(
                (state.last_goal().legs[leg] - expected.legs[leg]).abs() < 1e-12,
                "leg {leg}"
            );
        }
    }

    /// A target the machine may not hold is refused, and refusing it changes
    /// nothing: the machine is still armed and still holding, and the next
    /// command is taken normally.
    #[test]
    fn a_bad_target_is_rejected_and_a_bad_path_faults() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let too_high = MotionCommand::MoveTo {
            target: pose_at(0.25),
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };
        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&too_high));
        let CommandDisposition::Rejected(CommandRejection::Envelope(violations)) =
            out.report.command
        else {
            panic!(
                "expected an envelope rejection, got {:?}",
                out.report.command
            );
        };
        assert_eq!(violations.unreachable, [true; 6]);
        assert_eq!(out.goal, None);
        assert_eq!(out.report.fault, None);
        assert_eq!(state.mode(), Mode::Holding, "a bad command does not fault");

        let good = MotionCommand::MoveTo {
            target: pose_at(0.19),
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };
        let out = tick_with(&cfg, &mut state, secs(0.02), &pinned, Some(&good));
        assert_eq!(out.report.command, CommandDisposition::Started);
        assert_eq!(out.report.fault, None, "and starting it faulted nothing");

        // Now tighten the envelope under the running move, which is the
        // disagreement the path check exists to catch: a pose that passed
        // validation no longer passes on the way there. The machine stops
        // commanding rather than guessing which verdict to believe.
        let tight = MotionConfig {
            env: EnvelopeConfig {
                // Above the 24 mm the neutral pose has, so a path that rises
                // out of it — and so loses clearance — is below the floor and
                // improves on nothing.
                min_toggle_margin: 0.030,
                ..EnvelopeConfig::default()
            },
            ..MotionConfig::default()
        };
        let out = tick_with(&tight, &mut state, secs(0.04), &pinned, None);
        assert_eq!(out.goal, None);
        let Some(Fault::Envelope(violations)) = out.report.fault else {
            panic!("expected an envelope fault, got {:?}", out.report.fault);
        };
        assert!(violations.margin);
        assert!(state.is_faulted());
    }

    /// A fault absorbs everything after it: no goals, no commands taken, and
    /// the standing cause repeated on every tick so an operator reading one
    /// line of output sees it.
    #[test]
    fn faulted_is_absorbing() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let bad_health = [ServoHealth { id: 10, bits: 0x20 }; 9];
        let mut out = TickOutputs::default();
        motion_tick(
            &cfg,
            &mut state,
            &TickInputs {
                now: secs(0.0),
                present: Some(&pinned),
                command: None,
                health: Some(&bad_health),
            },
            &mut out,
        );
        let standing = out.report.fault.expect("hardware error faults");
        assert_eq!(standing, Fault::HardwareError { id: 10, bits: 0x20 });

        let command = MotionCommand::MoveTo {
            target: pose_at(0.19),
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };
        for n in 1..5 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &pinned,
                Some(&command),
            );
            assert_eq!(out.goal, None);
            assert_eq!(out.report.fault, Some(standing));
            assert_eq!(out.report.mode, Mode::Faulted(standing));
            assert_eq!(
                out.report.command,
                CommandDisposition::None,
                "commands are not even looked at"
            );
        }
    }

    /// The input-voltage bit alone is reported and not acted on; any other bit
    /// faults and names the servo that set it.
    #[test]
    fn only_bits_beyond_input_voltage_fault() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();

        for (bits, faults) in [(0x00, false), (0x01, false), (0x04, true), (0x21, true)] {
            let (mut state, pinned) = armed_at(&cfg, &start);
            let mut health = [ServoHealth::default(); 9];
            for (slot, id) in health.iter_mut().zip(10u8..) {
                slot.id = id;
            }
            health[3].bits = bits;

            let mut out = TickOutputs::default();
            motion_tick(
                &cfg,
                &mut state,
                &TickInputs {
                    now: secs(0.0),
                    present: Some(&pinned),
                    command: None,
                    health: Some(&health),
                },
                &mut out,
            );
            assert_eq!(
                out.report.health.expect("health is reported verbatim")[3].bits,
                bits,
                "bits {bits:#04x} survive into the report"
            );
            assert_eq!(
                out.report.fault,
                faults.then_some(Fault::HardwareError { id: 13, bits }),
                "bits {bits:#04x}"
            );
        }
    }

    /// Missed reads count, are reported as stale, and fault once the run
    /// exceeds the configured budget. A live read in between clears the count.
    #[test]
    fn read_loss_counts_and_a_live_read_clears_it() {
        let cfg = MotionConfig {
            read_loss_ticks: 3,
            ..MotionConfig::default()
        };
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let stale = |state: &mut MotionState, n: u32| {
            let mut out = TickOutputs::default();
            motion_tick(
                &cfg,
                state,
                &TickInputs {
                    now: secs(f64::from(n) * 0.02),
                    present: None,
                    command: None,
                    health: None,
                },
                &mut out,
            );
            out
        };

        for n in 1..=3 {
            let out = stale(&mut state, n);
            assert!(!out.report.present_fresh);
            assert_eq!(out.report.misses, n);
            assert_eq!(out.report.fault, None);
            assert_eq!(out.goal, None);
            assert_eq!(out.report.tracking_worst, None, "nothing was measured");
        }

        let out = tick_with(&cfg, &mut state, secs(0.08), &pinned, None);
        assert_eq!(out.report.misses, 0);
        for n in 5..=7 {
            assert_eq!(stale(&mut state, n).report.fault, None);
        }
        let out = stale(&mut state, 8);
        assert_eq!(out.report.fault, Some(Fault::ReadLoss { misses: 4 }));
        assert!(state.is_faulted());
    }

    /// The tracking counter needs the breach sustained: it resets on the first
    /// tick that is back inside the threshold, and only a run as long as the
    /// configured one faults. The joint named is the one furthest out.
    #[test]
    fn the_tracking_counter_resets_on_a_good_tick() {
        let cfg = MotionConfig {
            tracking: TrackingFaultConfig {
                threshold_rad: 0.1,
                ticks: 3,
            },
            ..MotionConfig::default()
        };
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let mut lagging = pinned;
        lagging.antennas[1] = 0.4;
        lagging.body_yaw = 0.15;

        for n in 0..2 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &lagging, None);
            assert_eq!(out.report.tracking_count, n + 1);
            assert_eq!(out.report.fault, None);
        }
        let out = tick_with(&cfg, &mut state, secs(0.04), &pinned, None);
        assert_eq!(out.report.tracking_count, 0);
        assert!(
            out.report.tracking_worst.expect("measured").1 < 1e-12,
            "a tracking machine has no error"
        );

        for n in 3..5 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &lagging, None);
            assert_eq!(out.report.tracking_count, n - 2);
            assert_eq!(out.report.fault, None);
        }

        // A tick that measured nothing repeats the standing count rather than
        // reporting a run that has not ended.
        let mut out = TickOutputs::default();
        motion_tick(
            &cfg,
            &mut state,
            &TickInputs {
                now: secs(0.08),
                present: None,
                command: None,
                health: None,
            },
            &mut out,
        );
        assert!(!out.report.present_fresh);
        assert_eq!(out.report.tracking_count, 2);

        let out = tick_with(&cfg, &mut state, secs(0.1), &lagging, None);
        assert_eq!(
            out.report.fault,
            Some(Fault::TrackingLost {
                joint: JointId::AntennaLeft,
                error: 0.4
            }),
            "the joint furthest from its goal is the one named"
        );
        assert_eq!(
            out.report.tracking_count, 3,
            "the tick that ran the budget out reports the full run, not zero"
        );
    }

    /// A second move while one is running is refused, and the running move is
    /// untouched by the refusal.
    #[test]
    fn a_move_while_moving_is_rejected() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let command = MotionCommand::MoveTo {
            target: pose_at(0.19),
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };

        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&command));
        assert_eq!(out.report.command, CommandDisposition::Started);
        assert_eq!(out.report.fault, None, "accepting it did not also fault");
        let started = state.mode();

        let other = MotionCommand::MoveTo {
            target: pose_at(0.15),
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };
        let out = tick_with(&cfg, &mut state, secs(0.02), &pinned, Some(&other));
        assert_eq!(
            out.report.command,
            CommandDisposition::Rejected(CommandRejection::AlreadyMoving)
        );
        assert_eq!(out.report.fault, None, "refusing it did not fault either");
        assert_eq!(state.mode(), started, "the running move is untouched");
        assert!(out.report.envelope.is_some(), "and it still advanced");
    }

    /// A move that cannot be shaped is refused the same way a bad target is: a
    /// caller's typo does not fault an armed machine.
    #[test]
    fn an_unshapeable_move_is_rejected() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let instant = MotionCommand::MoveTo {
            target: pose_at(0.19),
            duration: Duration::ZERO,
            warp: Warp::MinJerk,
        };
        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&instant));
        assert_eq!(
            out.report.command,
            CommandDisposition::Rejected(CommandRejection::Trajectory(
                TrajectoryError::NonPositiveDuration
            ))
        );
        assert_eq!(out.report.fault, None);
        assert_eq!(state.mode(), Mode::Holding);
    }

    /// A hold abandons the active move where it stands, and the machine holds
    /// the last goal it emitted rather than the target it was heading for.
    #[test]
    fn hold_abandons_the_active_move() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let command = MotionCommand::MoveTo {
            target: pose_at(0.19),
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };

        let mut present = pinned;
        tick_with(&cfg, &mut state, secs(0.0), &present, Some(&command));
        for n in 1..30 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &present, None);
            if let Some(goal) = out.goal {
                present = goal;
            }
        }
        let mid = *state.last_goal();
        assert!(mid.legs[0] != pinned.legs[0], "the move went somewhere");

        let out = tick_with(
            &cfg,
            &mut state,
            secs(0.6),
            &present,
            Some(&MotionCommand::Hold),
        );
        assert_eq!(out.report.command, CommandDisposition::Held);
        assert_eq!(out.report.fault, None);
        assert_eq!(state.mode(), Mode::Holding);
        assert_eq!(*state.last_goal(), mid);

        let out = tick_with(&cfg, &mut state, secs(0.62), &present, None);
        assert_eq!(out.goal, None);
        assert_eq!(*state.last_goal(), mid);
    }

    /// A step larger than the bound is a fault, not a trimmed goal: the servo
    /// would take the difference as an immediate jump, and the interpolator or
    /// seed that produced it is the thing worth reporting.
    #[test]
    fn an_oversized_step_faults() {
        let cfg = MotionConfig {
            max_step: JointStep {
                legs: 1e-4,
                body_yaw: 1e-4,
                antennas: 1e-4,
            },
            ..MotionConfig::default()
        };
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let command = MotionCommand::MoveTo {
            target: pose_at(0.19),
            duration: secs(0.5),
            warp: Warp::Linear,
        };

        let mut present = pinned;
        let mut fault = None;
        for n in 0..10 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                (n == 0).then_some(&command),
            );
            if let Some(goal) = out.goal {
                present = goal;
            }
            if let Some(raised) = out.report.fault {
                fault = Some(raised);
                assert_eq!(out.goal, None, "nothing is emitted on the faulting tick");
                break;
            }
        }
        let Some(Fault::StepTooLarge { joint, delta }) = fault else {
            panic!("expected a step fault, got {fault:?}");
        };
        assert!(matches!(joint, JointId::Leg(_)), "{joint} stepped");
        assert!(delta > 1e-4, "delta {delta}");
    }

    /// Crank angles that close no loop leave the tick with no idea where the
    /// head is, and it stops rather than commanding from a guess.
    #[test]
    fn an_unsolvable_present_pose_faults() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, _) = armed_at(&cfg, &start);
        let impossible = JointVector {
            legs: [
                0.0,
                core::f64::consts::PI,
                0.0,
                core::f64::consts::PI,
                0.0,
                0.0,
            ],
            ..JointVector::default()
        };
        let out = tick_with(&cfg, &mut state, secs(0.0), &impossible, None);
        assert!(
            matches!(out.report.fault, Some(Fault::PresentPoseLost(_))),
            "got {:?}",
            out.report.fault
        );
        assert_eq!(out.goal, None);
    }

    /// The baseline the tick passes to the envelope is the *present* pose's
    /// clearance, which is what lets an armed machine lift off a rest tighter
    /// than the clearance floor. The same command from a machine that has not
    /// measured that rest is refused.
    #[test]
    fn the_present_clearance_is_the_baseline() {
        let cfg = MotionConfig::default();
        let rest = rest_targets(0.0);
        let (mut state, pinned) = armed_at(&cfg, &rest);
        assert!(
            state.present_min_margin() < 0.000_3,
            "the rest is tighter than the floor: {}",
            state.present_min_margin()
        );

        // One millimetre up: still far below the floor, admitted because it
        // improves on the measured clearance.
        let lift = rest_targets(0.001);
        let command = MotionCommand::MoveTo {
            target: lift,
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };
        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&command));
        assert_eq!(out.report.command, CommandDisposition::Started);

        // Downward from the same rest is a tightening, and no baseline excuses
        // that.
        let (mut state, pinned) = armed_at(&cfg, &rest);
        let drop = rest_targets(-0.001);
        let command = MotionCommand::MoveTo {
            target: drop,
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };
        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&command));
        assert!(
            matches!(
                out.report.command,
                CommandDisposition::Rejected(CommandRejection::Envelope(_))
            ),
            "got {:?}",
            out.report.command
        );
    }

    /// A reading that is not a number faults on the tick it arrives, naming the
    /// joint it arrived on — not nine ticks later as a tracking failure blaming
    /// some other joint, and never as an input to the pose solve.
    #[test]
    fn a_non_finite_present_read_faults_at_once() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();

        for (index, expected) in JointId::ALL.iter().enumerate() {
            for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let (mut state, pinned) = armed_at(&cfg, &start);
                let mut present = pinned;
                match index {
                    0 => present.body_yaw = bad,
                    1..=6 => present.legs[index - 1] = bad,
                    _ => present.antennas[index - 7] = bad,
                }

                let out = tick_with(&cfg, &mut state, secs(0.0), &present, None);
                assert_eq!(
                    out.report.fault,
                    Some(Fault::PresentNotFinite { joint: *expected }),
                    "slot {index} with {bad}"
                );
                assert_eq!(out.goal, None);
                assert_eq!(out.report.fk, None, "the solve never ran");
                assert!(state.is_faulted());
            }
        }
    }

    /// The worst-error selection is an ordering, and an error nobody can place
    /// wins it. Reusing the bound test here would let the unplaceable joint be
    /// displaced by the next joint in the sweep, and the fault whose whole job is
    /// naming a joint would name the wrong one with a zero error beside it.
    #[test]
    fn the_worst_error_selection_is_an_ordering() {
        assert!(worse_error(0.2, 0.1));
        assert!(!worse_error(0.1, 0.2));
        assert!(!worse_error(0.1, 0.1), "a tie keeps the first joint");
        assert!(
            worse_error(0.0, f64::NEG_INFINITY),
            "the sweep's starting point"
        );
        assert!(worse_error(f64::NAN, 0.5), "unplaceable beats any number");
        assert!(!worse_error(0.5, f64::NAN), "and is not displaced by one");
        assert!(
            !worse_error(f64::NAN, f64::NAN),
            "a tie between two of them"
        );
    }

    /// The tracking fault names the joint whose error could not be placed, with
    /// that error, rather than the joint after it with zero.
    ///
    /// An arming summary is the only way a goal nobody can place gets into the
    /// state — a present read carrying one faults before tracking runs — so that
    /// is how this is reached.
    #[test]
    fn the_tracking_fault_names_the_joint_nobody_could_place() {
        let cfg = MotionConfig {
            tracking: TrackingFaultConfig {
                threshold_rad: 0.1,
                ticks: 1,
            },
            ..MotionConfig::default()
        };
        let targets = JointTargets::default();
        let (present, margin) = joints_for(&cfg, &targets);

        let mut pinned = present;
        pinned.legs[3] = f64::NAN;
        let mut state = MotionState::new_armed(&pinned, &targets.head_pose_body, margin);

        let out = tick_with(&cfg, &mut state, secs(0.0), &present, None);
        let Some(Fault::TrackingLost { joint, error }) = out.report.fault else {
            panic!("expected a tracking fault, got {:?}", out.report.fault);
        };
        assert_eq!(joint, JointId::Leg(3));
        assert!(
            error.is_nan(),
            "the error travels as it was measured: {error}"
        );
        assert_eq!(
            out.report.fault.unwrap().to_string(),
            "leg 4 is NaN rad from its goal and not closing"
        );
    }

    /// Goals are written only when they change. A move to the pose the machine is
    /// already holding runs its whole duration emitting nothing at all, which is
    /// an ordinary thing for an operator or a script to ask for, and the tick is
    /// what keeps it off the bus.
    #[test]
    fn a_move_to_where_the_machine_is_emits_nothing() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let command = MotionCommand::MoveTo {
            target: start,
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };
        let mut completed = false;
        for n in 0..=100 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &pinned,
                (n == 0).then_some(&command),
            );
            assert_eq!(out.goal, None, "tick {n}");
            assert!(!out.report.emitted, "tick {n}");
            assert_eq!(out.report.fault, None, "tick {n}");
            completed |= out.report.completed;
        }
        assert!(completed, "the move still finished");
        assert_eq!(state.mode(), Mode::Holding);
    }

    /// The other half of the same rule: in a move that does go somewhere, every
    /// tick that emits a goal emits a *different* goal from the one before it.
    #[test]
    fn every_emitted_goal_differs_from_the_last() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let command = MotionCommand::MoveTo {
            target: pose_at(0.19),
            duration: secs(1.0),
            warp: Warp::MinJerk,
        };

        let mut present = pinned;
        let mut emitted = 0;
        for n in 0..60 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                (n == 0).then_some(&command),
            );
            assert_eq!(
                out.goal.is_some(),
                out.report.emitted,
                "tick {n}: the report and the goal agree"
            );
            if let Some(goal) = out.goal {
                assert_ne!(goal, present, "tick {n} re-sent what was already commanded");
                emitted += 1;
                present = goal;
            }
            if out.report.completed {
                break;
            }
        }
        assert!(emitted > 40, "only {emitted} goals over a 1 s move");
    }

    /// The seed advances with the head. The pose solve is a Newton iteration
    /// seeded from where the platform was last seen, and that — not the
    /// plausibility screen — is what keeps it on the assembly mode the machine is
    /// actually in. A seed pinned at the arming pose would still converge across
    /// this whole workspace, so nothing but this notices.
    #[test]
    fn the_fk_seed_follows_the_head() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let arming_seed = *state.fk_seed();
        let command = MotionCommand::MoveTo {
            target: pose_at(0.195),
            duration: secs(1.0),
            warp: Warp::MinJerk,
        };

        let mut present = pinned;
        for n in 0..60 {
            let seed_before = *state.fk_seed();
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                (n == 0).then_some(&command),
            );

            // What the solve returned this tick, recomputed from the same seed
            // and the same reading, must be what the state now carries.
            let mut expected = Isometry3::identity();
            forward_kinematics(
                &cfg.geom,
                &LegAngles(present.legs),
                &seed_before,
                &cfg.fk,
                &mut expected,
            )
            .expect("the present angles solve");
            assert_eq!(*state.fk_seed(), expected, "tick {n}");

            if let Some(goal) = out.goal {
                present = goal;
            }
            if out.report.completed {
                break;
            }
        }
        let travelled =
            (state.fk_seed().translation.vector - arming_seed.translation.vector).norm();
        assert!(
            travelled > 0.015,
            "the seed stayed at the arming pose ({travelled} m)"
        );
    }

    /// The step guard is wired per group, and each group's bound is checked
    /// end to end: a bound attached to the wrong group would show up on hardware
    /// as a slam or a spurious fault, and the lookup's own unit test would not
    /// move.
    #[test]
    fn every_step_group_is_guarded() {
        let start = JointTargets::default();
        let antennas_target = JointTargets {
            antennas: [0.4, -0.4],
            ..JointTargets::default()
        };
        let yaw_target = JointTargets {
            body_yaw: 0.5,
            ..JointTargets::default()
        };

        let cases: [(&str, JointStep, JointTargets); 3] = [
            (
                "legs",
                JointStep {
                    legs: 1e-5,
                    ..MotionConfig::default().max_step
                },
                pose_at(0.19),
            ),
            (
                "body yaw",
                JointStep {
                    body_yaw: 1e-5,
                    ..MotionConfig::default().max_step
                },
                yaw_target,
            ),
            (
                "antennas",
                JointStep {
                    antennas: 1e-5,
                    ..MotionConfig::default().max_step
                },
                antennas_target,
            ),
        ];

        for (group, max_step, target) in cases {
            let cfg = MotionConfig {
                max_step,
                ..MotionConfig::default()
            };
            let (mut state, pinned) = armed_at(&cfg, &start);
            let (_, out) = run_move(&cfg, &mut state, &pinned, &target, secs(0.5), 0.02);

            let Some(Fault::StepTooLarge { joint, delta }) = out.report.fault else {
                panic!("{group}: expected a step fault, got {:?}", out.report.fault);
            };
            let named = match joint {
                JointId::Leg(_) => "legs",
                JointId::BodyYaw => "body yaw",
                JointId::AntennaRight | JointId::AntennaLeft => "antennas",
            };
            assert_eq!(named, group, "{group}: the fault named {joint}");
            assert!(delta > 1e-5, "{group}: delta {delta}");
            assert_eq!(out.goal, None);
        }
    }

    /// A read outage does not stop a move that was already validated: the tick
    /// skips the tracking comparison, which is the only thing needing a live
    /// reading, and goes on advancing and emitting. `read_loss_ticks` is the
    /// length of outage that may be ridden out, and the fault at the end of it is
    /// the backstop.
    #[test]
    fn a_stale_tick_keeps_a_move_going() {
        let cfg = MotionConfig {
            read_loss_ticks: 4,
            ..MotionConfig::default()
        };
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let command = MotionCommand::MoveTo {
            target: pose_at(0.19),
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };

        let mut present = pinned;
        for n in 0..10 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                (n == 0).then_some(&command),
            );
            if let Some(goal) = out.goal {
                present = goal;
            }
        }
        let before_outage = *state.last_goal();

        for n in 10..14 {
            let mut out = TickOutputs::default();
            motion_tick(
                &cfg,
                &mut state,
                &TickInputs {
                    now: secs(f64::from(n) * 0.02),
                    present: None,
                    command: None,
                    health: None,
                },
                &mut out,
            );
            assert!(!out.report.present_fresh, "tick {n}");
            assert_eq!(out.report.fault, None, "tick {n}");
            assert!(out.report.emitted, "tick {n} kept the move going");
            assert_eq!(out.report.tracking_worst, None, "nothing was measured");
        }
        assert_ne!(
            *state.last_goal(),
            before_outage,
            "the move advanced through the outage"
        );

        // One tick past the budget and the outage is the fault, with the move
        // abandoned where it stands.
        let mut out = TickOutputs::default();
        motion_tick(
            &cfg,
            &mut state,
            &TickInputs {
                now: secs(0.28),
                present: None,
                command: None,
                health: None,
            },
            &mut out,
        );
        assert_eq!(out.report.fault, Some(Fault::ReadLoss { misses: 5 }));
        assert_eq!(out.goal, None);
    }

    /// `now` is required to be non-decreasing, and this is what a caller that
    /// breaks that gets: the path is resampled at the earlier time and the head
    /// walks back the way it came, with only the step guard between a rewind and a
    /// slam. Pinned rather than left to `saturating_sub` to imply, because a
    /// rewind smaller than the step bound raises nothing at all.
    #[test]
    fn a_clock_that_runs_backwards_rewinds_the_path() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let command = MotionCommand::MoveTo {
            target: pose_at(0.19),
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };

        let mut present = pinned;
        for n in 0..20 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                (n == 0).then_some(&command),
            );
            if let Some(goal) = out.goal {
                present = goal;
            }
        }
        let advanced = *state.last_goal();

        // A `now` before the move's own start floors to zero elapsed, so the
        // sample is the start pose again. The whole move's leg travel is under
        // the step bound here, so nothing catches it.
        let out = tick_with(&cfg, &mut state, secs(0.0), &present, None);
        assert_eq!(out.report.fault, None);
        assert!(out.report.emitted, "a goal went out for the earlier time");
        assert_eq!(
            *state.last_goal(),
            pinned,
            "the rewound goal is the move's own start"
        );
        assert_ne!(*state.last_goal(), advanced);

        // A rewind that undoes more than one tick's worth of travel is caught,
        // and that is the whole protection: the same move on an axis with real
        // travel in it faults on the step rather than walking back.
        let (mut state, pinned) = armed_at(&cfg, &start);
        let yaw_move = MotionCommand::MoveTo {
            target: JointTargets {
                body_yaw: 0.5,
                ..JointTargets::default()
            },
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };
        let mut present = pinned;
        for n in 0..60 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                (n == 0).then_some(&yaw_move),
            );
            assert_eq!(out.report.fault, None, "tick {n} of the yaw move");
            if let Some(goal) = out.goal {
                present = goal;
            }
        }
        let out = tick_with(&cfg, &mut state, secs(0.0), &present, None);
        let Some(Fault::StepTooLarge { joint, .. }) = out.report.fault else {
            panic!("expected a step fault, got {:?}", out.report.fault);
        };
        assert_eq!(joint, JointId::BodyYaw);
        assert_eq!(out.goal, None);
    }

    /// Every fault and every refusal renders, and this is what it says. The
    /// messages are the operator's whole view of a stopped machine, and the
    /// formatting in them can go wrong silently.
    #[test]
    fn every_fault_and_refusal_names_itself() {
        let mut violations = EnvelopeViolations::default();
        violations.unreachable[0] = true;
        violations.margin = true;

        let faults: [(Fault, &str); 7] = [
            (
                Fault::Envelope(violations),
                "the commanded path left the envelope: leg 1 unreachable, toggle margin below the floor",
            ),
            (
                Fault::StepTooLarge {
                    joint: JointId::Leg(3),
                    delta: 0.5,
                },
                "leg 4 would step 0.5000 rad in one tick",
            ),
            (
                Fault::ReadLoss { misses: 50 },
                "no position read for 50 consecutive ticks",
            ),
            (
                Fault::TrackingLost {
                    joint: JointId::BodyYaw,
                    error: 0.123_456,
                },
                "body yaw is 0.1235 rad from its goal and not closing",
            ),
            (
                Fault::HardwareError { id: 13, bits: 0x20 },
                "servo 13 reports hardware error bits 0x20",
            ),
            (
                Fault::PresentPoseLost(FkError::NoConvergence {
                    iters: 7,
                    residual: 1.5e-4,
                }),
                "present pose unknown: no pose found after 7 iterations, residual 1.500e-4 m",
            ),
            (
                Fault::PresentNotFinite {
                    joint: JointId::AntennaRight,
                },
                "the position read for right antenna is not a number",
            ),
        ];
        for (fault, expected) in faults {
            assert_eq!(fault.to_string(), expected);
        }

        let refusals: [(CommandRejection, &str); 3] = [
            (CommandRejection::AlreadyMoving, "a move is already running"),
            (
                CommandRejection::Envelope(violations),
                "the commanded target is outside the envelope: leg 1 unreachable, toggle margin below the floor",
            ),
            (
                CommandRejection::Trajectory(TrajectoryError::NonPositiveDuration),
                "the commanded move cannot be shaped: trajectory duration must be greater than zero",
            ),
        ];
        for (refusal, expected) in refusals {
            assert_eq!(refusal.to_string(), expected);
        }
    }

    /// Two identical calls answer identically: the tick reads no clock and
    /// keeps nothing outside the state it is handed.
    #[test]
    fn the_tick_is_a_function_of_its_inputs() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut first, pinned) = armed_at(&cfg, &start);
        let mut second = first.clone();
        let command = MotionCommand::MoveTo {
            target: pose_at(0.19),
            duration: secs(2.0),
            warp: Warp::MinJerk,
        };

        for n in 0..20 {
            let now = secs(f64::from(n) * 0.02);
            let command = (n == 0).then_some(&command);
            let a = tick_with(&cfg, &mut first, now, &pinned, command);
            let b = tick_with(&cfg, &mut second, now, &pinned, command);
            assert_eq!(a, b, "tick {n}");
        }
    }
}
