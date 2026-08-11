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
//! commanding and every subsequent tick emits nothing, and it never recovers on
//! its own. This layer owns no wire, so it cuts no torque either; what it hands
//! back is the verdict that the machine must be brought to the minimum risk
//! condition, and the caller driving the bus is what writes torque off.
//!
//! A command whose target fails the envelope is deliberately *not* a fault: it
//! is **rejected** and reported, because an armed, holding machine must not be
//! bricked by someone typing a pose it cannot reach. A sampled *path* pose that
//! fails the envelope after its target passed is a different matter — the
//! checker and the interpolation have disagreed about a pose already accepted —
//! and that is a fault.
//!
//! ## Replacing a move in flight
//!
//! A command that arrives mid-move **retargets**: the running trajectory is
//! abandoned and a new one is shaped from the setpoint the last tick commanded,
//! at rest. Nothing queues and nothing blends — the machine follows the latest
//! intent it was given, which is what an interaction that changes its mind
//! partway needs. The splice assumes zero velocity, so the servo and the
//! gearbox absorb whatever the old path was carrying at the moment it was
//! dropped.
//!
//! ## The move's clock
//!
//! A caller's durations are a nominal policy, not a promise the machine can
//! keep from anywhere: they are sized for the spans an ordinary command covers,
//! and a move out of a pose a hand or a crash left the machine in can span far
//! more. [`floor_move_clock`] right-sizes such a move before it is commanded —
//! it dry-samples the candidate path at the control rate and stretches
//! whichever clock cannot carry its own span inside the per-tick step bounds.
//! The head group, the right antenna and the left antenna are each judged and
//! stretched on their own, so a pair of antennas deliberately staggered onto
//! different clocks stays staggered. Clocks are only ever lengthened, so the
//! path is preserved exactly and merely traversed more slowly, which is the
//! right degradation for a move whose only sin is starting further away than
//! the knob assumed. The step
//! guard below is untouched by any of it and stays the backstop for genuine
//! runaway bugs.
//!
//! ## The move's own start
//!
//! A trajectory sampled at zero elapsed time reproduces its start, which is the
//! pose the machine is already commanded to hold. Such a tick commands the
//! machine towards nothing, so it checks nothing and emits nothing, and stays in
//! the move. That is the tick that accepts a move, and any later tick whose clock
//! has not advanced past the move's start.
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
    LegAngles, below_limit, check_envelope, forward_kinematics, min_pose_margin, outside_limit,
    wrap_to_pi,
};
use thiserror::Error;

use crate::arm::ArmRecord;
use crate::joints::{JointId, JointStep, JointTargets, JointVector, ServoHealth, worst_joint};
use crate::traj::{MoveDurations, Trajectory, TrajectoryError, Warp};

/// When a joint is far enough from its goal, for long enough, without closing
/// on it, to conclude the servo is not tracking it.
///
/// Goal writes to the whole group are unacknowledged by the protocol, so a
/// write that never applied leaves no trace on the bus. This comparison is the
/// compensating detection: a goal that is not being followed shows up as a
/// position error that does not close, whether the write landed or the motor
/// stalled.
///
/// Distance alone cannot say that. The servos run a proportional position loop
/// with no integral term, so a joint chasing a streamed goal sits behind it by
/// roughly the commanded velocity times the loop's own time constant. At the
/// bench, leg 2 ran 0.246 rad behind a goal moving near 0.71 rad/s, and the
/// right antenna 0.430 rad behind one moving near 4.91 rad/s, both while
/// following perfectly well. Lag scales with commanded speed, so no constant
/// distance separates a chase from a stall; what separates them is whether the
/// joint is closing on where its goal lies, which is what `progress_min_rad`
/// measures.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackingFaultConfig {
    /// How far a joint may sit from its goal without being examined at all,
    /// radians.
    ///
    /// At least as wide as the widest per-tick goal step the step guard admits.
    /// Progress is measured from where the joint stood when its run opened, so
    /// a goal able to step past that anchor in one period would read a joint
    /// that is following as one running away from it.
    pub threshold_rad: f64,
    /// How far a joint past that threshold must travel toward its goal, within
    /// a window of `ticks`, to count as closing on it, radians.
    pub progress_min_rad: f64,
    /// How many consecutive live ticks past the threshold without that
    /// progress before the fault.
    pub ticks: u32,
}

impl Default for TrackingFaultConfig {
    /// Provisional bench figures. The threshold and the window are sized rather
    /// than measured; every move records its per-joint worst lag, and those are
    /// the numbers that will ground them.
    fn default() -> Self {
        Self {
            // 8.6° of crank. Only a screen for which joints are worth
            // examining, not a verdict: the bench has seen 0.43 rad of healthy
            // lag on an antenna at speed.
            threshold_rad: 0.15,
            // About 6.5 of the servos' 0.088° counts, comfortably above the
            // half-count quantisation floor of 7.7e-4 rad. Over the ten-tick
            // window at the bench's 50 Hz that asks a joint sitting past the
            // threshold for 0.05 rad/s of closing speed — an order of magnitude
            // under the 0.71 rad/s the legs were commanded at on the fastest
            // move so far, and two under the antennas' 4.91 rad/s.
            progress_min_rad: 0.01,
            // A fifth of a second at the bench's 50 Hz tick, so a single
            // transient on one read cannot raise it.
            ticks: 10,
        }
    }
}

/// One joint's open run of live ticks past the tracking threshold.
///
/// Progress is measured from `anchor` — where the joint stood when the run
/// opened, or when it last closed a window's worth of distance — rather than
/// between consecutive ticks, so a joint creeping less than one count per tick
/// still shows its motion over the window.
#[derive(Clone, Copy, Debug, PartialEq)]
struct TrackingStreak {
    /// Where the joint was when this run last restarted, radians.
    anchor: f64,
    /// Live ticks since, this one included.
    count: u32,
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
                // An antenna sweep takes the arc that misses its outboard
                // direction, so a commandable sweep runs to just under a full
                // turn — 6.28 rad, peaking near 0.15 at 50 Hz over a 1.57 s
                // move. That is this bound exactly, and it is why an antenna
                // duration has a floor: the presence sweep of 3.23 rad needs
                // 0.81 s, and re-stowing an antenna left inboard of sideways
                // (4.80 rad) needs 1.21 s.
                antennas: 0.15,
            },
            tracking: TrackingFaultConfig::default(),
            // One second of silence at 50 Hz.
            read_loss_ticks: 50,
        }
    }
}

/// The tick rate the duration floors below are derived at, hertz.
///
/// The bench and the daemon both run the loop at fifty. A deployment that
/// changed it would move every floor in proportion: what a step bound limits is
/// the distance between two consecutive periods, so halving the rate doubles
/// each step and doubles the shortest duration that fits.
pub const FLOOR_TICK_HZ: f64 = 50.0;

/// A min-jerk path's peak rate as a multiple of its average — fifteen eighths.
///
/// What makes a duration floor closed-form: a move covering `span` in
/// `duration` peaks at this times `span / duration`, and the per-tick step at
/// that peak is the largest one the guard will see.
pub const MIN_JERK_PEAK_RATE: f64 = 1.875;

/// The shortest a min-jerk move covering `span` radians can take without a
/// single tick's step passing `max_step`, seconds.
///
/// Exact for a joint the path moves linearly in its own coordinate — the body
/// yaw and the antennas. The legs move through the inverse kinematics and have
/// no closed form; theirs is [`HEAD_GROUP_FLOOR_S`], derived by search.
#[must_use]
pub fn duration_floor_s(span: f64, max_step: f64, tick_hz: f64) -> f64 {
    MIN_JERK_PEAK_RATE * span / (max_step * tick_hz)
}

/// The head group's duration floor: the shortest stow-to-neutral move whose
/// every leg step at [`FLOOR_TICK_HZ`] stays inside the default
/// `max_step.legs`, seconds, rounded up to the 10 ms its search steps in.
///
/// Derived rather than guessed — a test re-derives it through the inverse
/// kinematics and fails when the geometry, the tick rate or the bound moves it
/// — and public because it is the number both example configurations quote
/// beside the durations an operator tunes, and the number a configuration test
/// checks the shipped values against. Under it the guard faults rather than
/// clamping, which is a stopped move and a de-torqued machine: a shipped
/// duration below this is presence that never works.
pub const HEAD_GROUP_FLOOR_S: f64 = 1.07;

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
    /// A joint sat past the tracking threshold for a whole window without
    /// closing on its goal.
    #[error("{joint} is {error:.4} rad from its goal and not closing")]
    TrackingLost {
        /// The joint whose window ran out, or the furthest out of those whose
        /// windows ran out together.
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

/// How far a pose stands outside each envelope bound it can travel back inside
/// of, radians, zero on a bound it is within.
///
/// The clearance floor is not among them. It has a baseline of its own, taken
/// from the present pose on every live read, and the question it asks is
/// different: an interpolated pose that merely holds a clearance already below
/// the floor buys nothing, where a pose merely holding a yaw already past its
/// cap is the machine standing still on its way out.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct Excursion {
    /// Per leg, how far the crank angle reaches past its travel window.
    window: [f64; 6],
    /// How far the body yaw's magnitude reaches past its cap.
    body_yaw: f64,
    /// How far the head-relative yaw's magnitude reaches past its cap.
    relative_yaw: f64,
    /// How far the head attitude reaches past the cone bound.
    cone: f64,
}

/// How far `value` reaches past `limit`, zero when it does not reach past it.
///
/// A value nobody can place is infinitely far out, for the same reason
/// [`outside_limit`] treats it as a violation: nothing can be decided from it,
/// least of all that it is no worse than where the machine started.
fn over(value: f64, limit: f64) -> f64 {
    if value.is_nan() {
        return f64::INFINITY;
    }
    (value - limit).max(0.0)
}

impl Excursion {
    /// How far the pose `report` describes stands outside each bound.
    ///
    /// `report` is [`check_envelope`]'s own record, which is filled whether the
    /// pose passed or not, so the numbers here and the verdict they qualify come
    /// from one solve of one pose.
    fn of(env: &EnvelopeConfig, report: &EnvelopeReport, body_yaw: f64) -> Self {
        Self {
            window: core::array::from_fn(|leg| {
                let (lo, hi) = env.crank_windows[leg];
                // A pose with no crank angle for this leg stands outside no
                // window: there is no angle to be outside one. Such a sample is
                // refused before any of this is consulted, because the tick has
                // no six angles to command.
                report.leg_angles.map_or(0.0, |LegAngles(angles)| {
                    let angle = angles[leg];
                    over(angle, hi).max(over(-angle, -lo))
                })
            }),
            body_yaw: over(body_yaw.abs(), env.body_yaw_limit),
            relative_yaw: over(report.relative_yaw.abs(), env.relative_yaw_limit),
            cone: over(report.cone_angle.abs(), env.head_cone_limit),
        }
    }

    /// Whether nothing here stands further out than it did at `start`.
    fn no_further_out_than(&self, start: &Self) -> bool {
        !self
            .window
            .iter()
            .zip(start.window)
            .any(|(now, then)| outside_limit(*now, then))
            && !outside_limit(self.body_yaw, start.body_yaw)
            && !outside_limit(self.relative_yaw, start.relative_yaw)
            && !outside_limit(self.cone, start.cone)
    }
}

/// How far the pose a move begins at stands outside the envelope.
///
/// The verdict is not the question here — a move is accepted or refused on its
/// *target*, which [`take_command`] checks — so the error is dropped and only
/// the distances are kept.
fn start_excursion(cfg: &MotionConfig, start: &JointTargets) -> Excursion {
    let mut report = EnvelopeReport::default();
    let _ = check_envelope(
        &cfg.geom,
        &cfg.env,
        &start.head_pose_body,
        start.body_yaw,
        None,
        &mut report,
    );
    Excursion::of(&cfg.env, &report, start.body_yaw)
}

/// Counts the antennas' goal register reaches either side of its own zero, and
/// the count that reads as zero radians.
///
/// The wire layer owns the conversion; these two figures are repeated here
/// because the bound below is a count bound and the two layers share no crate.
/// A bench test pins them against the conversion itself.
const ANTENNA_GOAL_COUNTS: f64 = 1_048_575.0;
const COUNTS_PER_TURN: f64 = 4096.0;

/// The highest angle either antenna's goal may hold, radians.
///
/// The servos run the antennas in extended position mode, whose goal register
/// reaches ±1_048_575 counts. That span is not symmetric about zero *radians*:
/// the count frame's zero sits half a turn below it, so the reachable angles
/// run from half a turn below −256 turns to half a turn below +256. A goal
/// outside them has no count to write. Nothing this machine commands
/// approaches either bound — every antenna target resolves to within half a
/// turn of the frame the last command left — but a value no count represents
/// is a refusal rather than something to saturate.
pub const ANTENNA_GOAL_MAX_RAD: f64 =
    core::f64::consts::TAU * (ANTENNA_GOAL_COUNTS / COUNTS_PER_TURN) - core::f64::consts::PI;

/// The lowest angle either antenna's goal may hold, radians. See
/// [`ANTENNA_GOAL_MAX_RAD`] for why the two are not mirror images.
pub const ANTENNA_GOAL_MIN_RAD: f64 =
    -core::f64::consts::TAU * (ANTENNA_GOAL_COUNTS / COUNTS_PER_TURN) - core::f64::consts::PI;

/// The physically sideways direction each antenna is kept from sweeping
/// through, radians: right, then left.
///
/// Horizontal — a quarter turn either side of straight up — and signed by the
/// side the antenna is mounted on, so each constant names its own antenna's
/// outboard direction. That arc is the maximal-interference one: it sweeps the
/// widest envelope around the machine exactly where objects sit beside it,
/// while the inboard arc crosses the antennas harmlessly over the head at their
/// different heights and disturbs almost nothing outside the head's footprint.
///
/// Deliberately not derived from the stow angles. The midpoint of the
/// stow-to-neutral arc is ±1.525 rad, about 2.6° off horizontal; these are the
/// physical direction and stay put if the stow angles ever move.
pub const ANTENNA_OUTBOARD: [f64; 2] =
    [-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2];

/// What a caller asks the tick to do.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum MotionCommand {
    /// Move to `target`, each group over its own duration, shaped by `warp`.
    MoveTo {
        /// Where to end up.
        target: JointTargets,
        /// How long each mechanical group takes.
        durations: MoveDurations,
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
    /// The commanded target is not a pose this machine may hold.
    #[error("the commanded target is outside the envelope: {0}")]
    Envelope(EnvelopeViolations),
    /// The commanded move cannot be shaped into a path.
    #[error("the commanded move cannot be shaped: {0}")]
    Trajectory(TrajectoryError),
    /// An antenna direction resolved to a goal no servo count represents.
    #[error("the commanded {joint} angle {angle:.4} rad is not commandable")]
    AntennaUnreachable {
        /// Which antenna.
        joint: JointId,
        /// The goal that has no count, radians.
        angle: f64,
    },
}

/// What became of this tick's command.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum CommandDisposition {
    /// There was no command.
    #[default]
    None,
    /// A move was accepted on this tick, from a machine that was holding.
    Started,
    /// A move was accepted on this tick over one already running, which it
    /// replaced. The new path starts at the setpoint the previous tick
    /// commanded.
    Retargeted,
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
    /// The longest open run of live ticks with one joint past the tracking
    /// threshold and not closing on its goal, including the tick that raised
    /// [`Fault::TrackingLost`]. A tick without a live read measures nothing and
    /// repeats the standing figure.
    pub tracking_count: u32,
    /// How far each joint was from the goal it was last written, in bus order,
    /// radians, when the read was live.
    ///
    /// The raw per-joint lag, not a verdict: what a proportional loop runs
    /// behind a moving goal is the number this project has so far only guessed
    /// at, and every move — clean or faulted — is a measurement of it.
    pub tracking_errors: Option<[f64; JointId::COUNT]>,
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
    /// The envelope check of the sampled path pose, when the tick checked one.
    /// A tick that sampled a move's own start checked nothing.
    pub envelope: Option<EnvelopeReport>,
    /// Whether this tick's sample was the active move's own start — the pose
    /// already held — in which case nothing was checked and nothing emitted.
    pub start_sample: bool,
    /// Whether the sampled pose failed an envelope bound and was commanded
    /// anyway, standing no further outside it than the move's own start did.
    /// A move travelling back inside the envelope, in other words.
    pub recovering: bool,
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
            tracking_errors: None,
            fk: None,
            present_min_margin: 0.0,
            health: None,
            command: CommandDisposition::None,
            envelope: None,
            start_sample: false,
            recovering: false,
            emitted: false,
            completed: false,
            fault: None,
        }
    }
}

impl TickReport {
    /// The joint furthest from its goal, and by how much, when the read was
    /// live.
    ///
    /// Derived from `tracking_errors` rather than carried beside it, so the
    /// worst named here and the nine figures a summary accumulates cannot
    /// disagree.
    #[must_use]
    pub fn tracking_worst(&self) -> Option<(JointId, f64)> {
        self.tracking_errors.as_ref().map(worst_joint)
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
    /// guard. At or before the move's own start there is no elapsed time at all,
    /// and such a tick commands nothing.
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
    start_excursion: Excursion,
    miss_count: u32,
    tracking: [Option<TrackingStreak>; JointId::COUNT],
}

impl MotionState {
    /// The state of a machine that has just finished arming.
    ///
    /// `armed` is arming's record of what it left the machine holding: the goals
    /// in the servos' registers, the pose those angles hold, and that pose's
    /// smallest toggle margin — the baseline that lets a first move lift off a
    /// rest tighter than the clearance floor.
    ///
    /// The **armed** record, specifically, and not the record of where the
    /// platform was found. The two are different poses whenever arming had to
    /// pull a joint into its travel window, and a state whose goals came from one
    /// and whose Cartesian mirror came from the other would hand the first
    /// trajectory a start the machine is not at — outside the travel windows
    /// every later sample is checked against.
    ///
    /// There is no other way to build one, and that is the point: a tick can
    /// only run on a machine somebody armed.
    #[must_use]
    pub fn new_armed(armed: &ArmRecord) -> Self {
        Self {
            mode: Mode::Holding,
            trajectory: None,
            last_goal: armed.joints,
            last_targets: JointTargets {
                head_pose_body: armed.head_pose_body,
                body_yaw: armed.joints.body_yaw,
                antennas: armed.joints.antennas,
            },
            fk_seed: armed.head_pose_body,
            present_min_margin: armed.min_margin,
            // No move is running, so no sample is being excused anything. The
            // first command computes the allowance from the pose it starts at,
            // which is this one.
            start_excursion: Excursion::default(),
            miss_count: 0,
            tracking: [None; JointId::COUNT],
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

/// The longest open run across the nine joints, or zero when none is open.
///
/// The single number the report carries about nine independent runs: the one
/// closest to raising the fault.
fn longest_streak(streaks: &[Option<TrackingStreak>; JointId::COUNT]) -> u32 {
    streaks
        .iter()
        .filter_map(|streak| streak.map(|open| open.count))
        .max()
        .unwrap_or(0)
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

    // A fault is absorbing. Commands are ignored and nothing is emitted, and
    // that is the whole of what this layer can do about one: it holds no wire,
    // so torque is left exactly where the caller had it, and bringing the
    // machine to the minimum risk condition — torque off — is the caller's job.
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
    out.report.tracking_count = longest_streak(&state.tracking);

    // Tracking, on live reads only: a stale reading compared against a fresh
    // goal is a difference nobody measured, so a stale tick freezes every
    // joint's run where it stands rather than growing or clearing it.
    if let Some(present) = fresh {
        let mut errors = [0.0; JointId::COUNT];
        // The rows whose window ran out on this tick, by how far out they are.
        // Every other row holds a value no measurement can beat, so the same
        // worst-of sweep names the joint among them.
        let mut exhausted = [f64::NEG_INFINITY; JointId::COUNT];
        let mut any_exhausted = false;
        for (row, ((_, angle), (_, goal))) in present
            .joints()
            .into_iter()
            .zip(state.last_goal.joints())
            .enumerate()
        {
            errors[row] = (angle - goal).abs();
            let streak = &mut state.tracking[row];
            if !outside_limit(errors[row], cfg.tracking.threshold_rad) {
                // Within the threshold is healthy, whatever came before it.
                *streak = None;
                continue;
            }
            let (count, closing) = match streak {
                // Progress is measured from where the joint stands now.
                None => {
                    *streak = Some(TrackingStreak {
                        anchor: angle,
                        count: 1,
                    });
                    (1, false)
                }
                Some(open) => {
                    // Ground covered since the anchor, signed toward the goal:
                    // positive is closing, negative is running away, and a goal
                    // that has arrived at the anchor leaves no direction to
                    // close in. An unplaceable number closes nothing.
                    let toward = goal - open.anchor;
                    let advance = if toward == 0.0 {
                        0.0
                    } else {
                        (angle - open.anchor) * toward.signum()
                    };
                    if advance >= cfg.tracking.progress_min_rad {
                        // Sitting behind a moving goal is what a proportional
                        // loop does, not what this fault is for.
                        open.anchor = angle;
                        open.count = 1;
                        (1, true)
                    } else {
                        open.count += 1;
                        (open.count, false)
                    }
                }
            };
            if !closing && count >= cfg.tracking.ticks {
                exhausted[row] = errors[row];
                any_exhausted = true;
            }
        }
        out.report.tracking_errors = Some(errors);
        // Recorded before the fault check: the tick that runs a window out is
        // the one whose figure matters most, and a report that shipped zero
        // there would read as a single-tick trip rather than a sustained one.
        out.report.tracking_count = longest_streak(&state.tracking);
        if any_exhausted {
            let (joint, error) = worst_joint(&exhausted);
            state.raise(Fault::TrackingLost { joint, error }, out);
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
    let t = inp.now.saturating_sub(started);
    let Some((sampled, endpoint, done)) = state.trajectory.as_ref().map(|trajectory| {
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

    // Zero elapsed time samples the move's own start, which is the pose already
    // commanded and held. Nothing is being asked of the machine, so nothing is
    // checked and nothing goes out: a refusal here would stop the machine at the
    // very pose the refusal was protecting, and a goal here would rewrite bits
    // the servos already have. Exact time arithmetic, not a comparison of poses —
    // a freshly armed machine's start is the solved mirror of its pinned goals
    // and sits a rounding error away from them.
    if t.is_zero() {
        out.report.start_sample = true;
        return;
    }

    let mut envelope = EnvelopeReport::default();
    let verdict = check_envelope(
        &cfg.geom,
        &cfg.env,
        &sampled.head_pose_body,
        sampled.body_yaw,
        Some(state.present_min_margin),
        &mut envelope,
    );
    out.report.envelope = Some(envelope);
    // A move out of a pose the envelope refuses has to be able to leave it, and
    // where the machine physically stands is not refusable — a hand or a crash
    // can leave the body past its yaw cap or a crank outside its window, and
    // stowing from there is the recovery. So a failing sample is judged against
    // the move's own start rather than against the bounds alone: one standing no
    // further out asks for nothing the machine is not already doing, and the
    // target it travels to passed the bounds outright when the command was
    // taken. Anything further out is the fault it always was.
    //
    // The second arm is a verdict that passed without producing the angles it
    // passed on, and a pose with no angles is nothing this can command.
    let recovering = verdict.is_err();
    let admitted = match verdict {
        Ok(()) => true,
        Err(error) => {
            !error.violations.margin
                && Excursion::of(&cfg.env, &envelope, sampled.body_yaw)
                    .no_further_out_than(&state.start_excursion)
        }
    };
    let (true, Some(angles)) = (admitted, envelope.leg_angles) else {
        state.raise(Fault::Envelope(envelope.violations), out);
        return;
    };
    out.report.recovering = recovering;
    let candidate = JointVector {
        body_yaw: sampled.body_yaw,
        legs: angles.0,
        antennas: sampled.antennas,
    };

    // Step guard. An oversized step is a slam, and the bug that produced it is
    // the thing worth reporting; it is never trimmed and sent. A move whose
    // clock is too short for its span does not reach here:
    // `floor_move_clock` right-sizes it before it is commanded, so what
    // remains for this guard to catch is an interpolator or a seed that is
    // wrong.
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
        durations,
        warp,
    } = command
    else {
        state.trajectory = None;
        state.mode = Mode::Holding;
        return CommandDisposition::Held;
    };

    let target = match resolve_antennas(&state.last_targets, target) {
        Ok(target) => target,
        Err(rejection) => return CommandDisposition::Rejected(rejection),
    };

    let mut report = EnvelopeReport::default();
    if let Err(error) = check_envelope(
        &cfg.geom,
        &cfg.env,
        &target.head_pose_body,
        target.body_yaw,
        Some(state.present_min_margin),
        &mut report,
    ) {
        return CommandDisposition::Rejected(CommandRejection::Envelope(error.violations));
    }

    // Start from the last commanded targets rather than the measured pose, so
    // consecutive moves chain without a step and a tracking lag is not written
    // into the path. Mid-move that is the setpoint the previous tick commanded,
    // which is what makes a retarget a splice rather than a jump.
    let retarget = matches!(state.mode, Mode::Moving { .. });
    match Trajectory::new(&state.last_targets, &target, *durations, *warp) {
        Ok(trajectory) => {
            // How far outside the envelope this move begins, which is the
            // allowance every sample of it is judged against. Recomputed per
            // command, so a retarget mid-recovery tightens it to wherever the
            // machine has got to.
            state.start_excursion = start_excursion(cfg, &state.last_targets);
            state.trajectory = Some(trajectory);
            state.mode = Mode::Moving { started: now };
            if retarget {
                CommandDisposition::Retargeted
            } else {
                CommandDisposition::Started
            }
        }
        Err(error) => CommandDisposition::Rejected(CommandRejection::Trajectory(error)),
    }
}

/// How much slack past the measured ratio a stretched clock is given.
///
/// The correction itself is first-order exact for a joint the path moves
/// linearly in its own coordinate: min-jerk is time-scale invariant, so twice
/// the clock is half the per-tick step. What the slack is for is everything the
/// dry pass cannot see, and the largest of those by far is the clock the live
/// loop keeps. This pass samples an ideal grid; the loop samples the trajectory
/// at whatever time each period *begins*, and the sleep that paces it wakes at
/// or after its deadline. A period that starts late by some fraction of a
/// period covers that much more of the path, and the step it commands grows in
/// proportion — so a clock sized to land exactly on the bound faults on
/// ordinary scheduler jitter, on the recovery move a stretch exists to save.
/// Half a period is the lateness the control loop treats as ordinary, so half
/// again is what a stretched clock has to cover before anything else; the rest
/// is room over that, and over the two small terms — the inverse kinematics the
/// legs go through, and the shift of the sampling grid itself under a stretch.
/// A stretch is already the slow path, and a recovery span taking this much
/// longer costs nothing next to a machine de-torqued partway through it.
const STRETCH_MARGIN: f64 = 0.75;

/// How many times a clock may be measured and stretched before the move runs on
/// whatever the last pass produced.
///
/// The worst per-tick step shrinks monotonically as the same path is sampled
/// over a longer clock, so the loop terminates on its own; this bound is what
/// keeps a pathological geometry from spending an unbounded number of dry
/// passes on the period that accepts a command. Exhausting it is not expected,
/// and the step guard is the backstop if it ever happens.
const STRETCH_PASSES: usize = 4;

/// The most samples one dry pass takes.
///
/// Two thousand seconds at fifty hertz. Any span this machine can travel fits
/// inside the step bounds on a clock far shorter than that, so a duration past
/// this is one there is nothing left to measure about.
const MAX_DRY_SAMPLES: u32 = 100_000;

/// A move's clock as it was asked for and as it will run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClockStretch {
    /// The durations the caller commanded.
    pub requested: MoveDurations,
    /// The durations the move runs on instead: long enough, per clock, to carry
    /// the move's own span inside the per-tick step bounds.
    pub effective: MoveDurations,
}

/// The worst per-tick step one dry pass saw, per clock, as a fraction of the
/// bound the joints on that clock are judged against.
///
/// At or under one is a clock that fits. The head clock drives the body yaw and
/// the six legs, which are bounded separately and reduced together here because
/// one duration governs them both; each antenna is its own clock and its own
/// ratio, so a side asked for a clock too short for its arc stretches without
/// dragging the other side out with it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct StepRatios {
    head: f64,
    antennas: [f64; 2],
}

impl StepRatios {
    /// Whether every clock carries its span.
    fn fit(self) -> bool {
        self.head <= 1.0 && self.antennas.iter().all(|ratio| *ratio <= 1.0)
    }
}

/// `command` on a clock long enough to carry the span it actually covers within
/// the per-tick step bounds, and what the stretch was when there was one.
///
/// A duration is configuration, sized for the spans an ordinary command covers.
/// Where the machine physically *stands* is not configuration: a hand or a crash
/// can leave the body most of a turn from where a stow expects to find it, and
/// a fixed clock over that span steps past the guard partway through and
/// faults, which de-torques the machine at the exact moment it was recovering.
/// This measures the candidate path before it is commanded — dry-sampling it at
/// `tick_hz` through the same envelope and inverse kinematics the live tick
/// runs, over the same trajectory [`take_command`] will build — and hands back a
/// clock that fits.
///
/// Only ever longer, and only where a fixed clock was never sized: a feasible
/// move comes back untouched with `None`, so the configured durations remain
/// the policy for every move they are feasible for. A command this cannot
/// measure — a move that cannot be shaped, a path some sample has no inverse
/// kinematics for, a step bound of zero — also comes back untouched. Nothing
/// here refuses anything; [`take_command`] still judges the command it is
/// handed.
///
/// `start` is what the accepted trajectory will chain from — the state's last
/// commanded targets — so a retarget mid-recovery is right-sized for the span
/// still ahead of it rather than for the one the original command had.
#[must_use]
pub fn floor_move_clock(
    cfg: &MotionConfig,
    start: &JointTargets,
    command: &MotionCommand,
    tick_hz: f64,
) -> (MotionCommand, Option<ClockStretch>) {
    let MotionCommand::MoveTo {
        target,
        durations,
        warp,
    } = command
    else {
        return (*command, None);
    };
    let Ok(resolved) = resolve_antennas(start, target) else {
        return (*command, None);
    };

    let requested = *durations;
    let mut effective = requested;
    for _ in 0..STRETCH_PASSES {
        let Some(ratios) = worst_step_ratios(cfg, start, &resolved, effective, *warp, tick_hz)
        else {
            break;
        };
        if ratios.fit() {
            break;
        }
        let Some(stretched) = stretched(effective, ratios) else {
            break;
        };
        effective = stretched;
    }

    if effective == requested {
        return (*command, None);
    }
    (
        // The target as it was handed in, unresolved: the resolution above is
        // this pass's own arithmetic, and re-resolving an already-resolved
        // antenna direction is not the identity.
        MotionCommand::MoveTo {
            target: *target,
            durations: effective,
            warp: *warp,
        },
        Some(ClockStretch {
            requested,
            effective,
        }),
    )
}

/// `durations` with each clock that overran its bound scaled past it, and the
/// clocks that did not left alone. `None` when the arithmetic leaves the range a
/// duration can hold.
fn stretched(durations: MoveDurations, ratios: StepRatios) -> Option<MoveDurations> {
    Some(MoveDurations {
        head: scale_past(durations.head, ratios.head)?,
        antennas: [
            scale_past(durations.antennas[0], ratios.antennas[0])?,
            scale_past(durations.antennas[1], ratios.antennas[1])?,
        ],
    })
}

/// `duration` scaled by `ratio` and the margin, or itself when the ratio is
/// already inside the bound. Never shortened.
fn scale_past(duration: Duration, ratio: f64) -> Option<Duration> {
    if ratio <= 1.0 {
        return Some(duration);
    }
    Duration::try_from_secs_f64(duration.as_secs_f64() * ratio * (1.0 + STRETCH_MARGIN)).ok()
}

/// The worst per-tick step the move from `start` to `target` would emit on
/// `durations`, per clock, as a fraction of the bound its joints are judged
/// against.
///
/// `None` when the path cannot be measured at all: an unshapeable move, a
/// sample the inverse kinematics has no answer for, a bound of zero, or a clock
/// too long to walk. Every one of those is a question for the tick rather than
/// something a longer clock fixes.
fn worst_step_ratios(
    cfg: &MotionConfig,
    start: &JointTargets,
    target: &JointTargets,
    durations: MoveDurations,
    warp: Warp,
    tick_hz: f64,
) -> Option<StepRatios> {
    if !(tick_hz.is_finite() && tick_hz > 0.0) {
        return None;
    }
    let trajectory = Trajectory::new(start, target, durations, warp).ok()?;

    let samples = (durations.longest().as_secs_f64() * tick_hz).ceil();
    if !samples.is_finite() || samples > f64::from(MAX_DRY_SAMPLES) {
        return None;
    }
    // The grid the live loop samples on, from the first period that commands
    // anything through the one that lands the endpoint. The move's own start is
    // the pose already held and is the baseline rather than a step.
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "bounded above by MAX_DRY_SAMPLES and below by the ceil of a non-negative span"
    )]
    let samples = samples as u32;

    let mut previous = joints_of(cfg, start)?;
    let mut sampled = JointTargets::default();
    let mut ratios = StepRatios::default();
    for step in 1..=samples {
        let t = Duration::try_from_secs_f64(f64::from(step) / tick_hz).ok()?;
        trajectory.sample(t, &mut sampled);
        let candidate = joints_of(cfg, &sampled)?;
        for ((id, angle), (_, last)) in candidate.joints().into_iter().zip(previous.joints()) {
            let ratio = (angle - last).abs() / cfg.max_step.for_joint(id);
            if !ratio.is_finite() {
                return None;
            }
            // Which clock this joint rides, not which group it belongs to: the
            // two antennas share a bound and a group and nothing else here.
            let clock = match id {
                JointId::AntennaRight => &mut ratios.antennas[0],
                JointId::AntennaLeft => &mut ratios.antennas[1],
                JointId::BodyYaw | JointId::Leg(_) => &mut ratios.head,
            };
            *clock = clock.max(ratio);
        }
        previous = candidate;
    }
    Some(ratios)
}

/// The nine joint angles that hold `targets`, or `None` when some leg has no
/// solution.
///
/// The same solve the tick commands from — [`check_envelope`]'s own crank
/// angles — so what this measures is what the guard will see. The verdict is
/// dropped: a recovering move is admitted outside the envelope, and its steps
/// are exactly the ones worth sizing a clock for.
fn joints_of(cfg: &MotionConfig, targets: &JointTargets) -> Option<JointVector> {
    let mut report = EnvelopeReport::default();
    let _ = check_envelope(
        &cfg.geom,
        &cfg.env,
        &targets.head_pose_body,
        targets.body_yaw,
        None,
        &mut report,
    );
    report.leg_angles.map(|LegAngles(legs)| JointVector {
        body_yaw: targets.body_yaw,
        legs,
        antennas: targets.antennas,
    })
}

/// `target` with each antenna direction resolved to the representative the
/// machine will sweep to from `start`.
///
/// An antenna target is a direction — a physical angle mod 2π — and the
/// machine's frame for it is continuous and unbounded. Each direction resolves
/// to a representative within a turn of where the last command left that
/// antenna, chosen to miss the outboard sideways point. This is the only wrap
/// arithmetic on the command path, and the one place it lives: the
/// interpolation, the step guard, the tracking comparison and the dry pass that
/// right-sizes a move's clock all take plain linear differences in the frame it
/// produces.
fn resolve_antennas(
    start: &JointTargets,
    target: &JointTargets,
) -> Result<JointTargets, CommandRejection> {
    let mut resolved = *target;
    for (side, joint) in [JointId::AntennaRight, JointId::AntennaLeft]
        .into_iter()
        .enumerate()
    {
        let last = start.antennas[side];
        match resolve_antenna(last, resolved.antennas[side], ANTENNA_OUTBOARD[side]) {
            Some(angle) => resolved.antennas[side] = angle,
            None => {
                // Both arcs land where no servo count reaches. The preferred one
                // is what the operator asked for, so it is the number reported.
                let angle = short_arc(last, resolved.antennas[side]);
                return Err(CommandRejection::AntennaUnreachable { joint, angle });
            }
        }
    }
    Ok(resolved)
}

/// The representative of direction `target` nearest `last`: the arc no longer
/// than half a turn.
fn short_arc(last: f64, target: f64) -> f64 {
    last + wrap_to_pi(target - last)
}

/// Which representative of antenna direction `target` to sweep to from `last`,
/// or `None` when no servo count reaches either candidate.
///
/// The short arc unless it would carry the antenna through `outboard` — the
/// direction it must not sweep past — in which case the long way round, which
/// costs at most a turn and keeps the antenna over the head instead of out to
/// the side. Endpoints count as not crossing, so an antenna already standing at
/// sideways takes the shortest path away from it, and one commanded exactly
/// there arrives the short way.
fn resolve_antenna(last: f64, target: f64, outboard: f64) -> Option<f64> {
    let short = short_arc(last, target);
    let sweep = short - last;
    // Ground from `last` to `outboard` in the direction of travel, in
    // `[0, 2π)`. Strictly inside the sweep is a crossing; zero is the antenna
    // standing on the point, and `sweep` itself is it arriving there.
    let to_outboard = if sweep >= 0.0 {
        (outboard - last).rem_euclid(core::f64::consts::TAU)
    } else {
        (last - outboard).rem_euclid(core::f64::consts::TAU)
    };
    let crosses = to_outboard > 0.0 && to_outboard < sweep.abs();

    let long = if sweep >= 0.0 {
        short - core::f64::consts::TAU
    } else {
        short + core::f64::consts::TAU
    };
    let (preferred, fallback) = if crosses {
        (long, short)
    } else {
        (short, long)
    };
    [preferred, fallback].into_iter().find(|angle| {
        !(outside_limit(*angle, ANTENNA_GOAL_MAX_RAD) || below_limit(*angle, ANTENNA_GOAL_MIN_RAD))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arm::{ArmConfig, pin_goals};
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

    /// Arming's configuration, against the fences this tick's envelope implies.
    fn arm_config(cfg: &MotionConfig) -> ArmConfig {
        crate::testutil::arm_config(&cfg.env)
    }

    /// An arming record for angles known to hold `head_pose_body`.
    fn record_at(
        cfg: &MotionConfig,
        joints: &JointVector,
        head_pose_body: &Isometry3<f64>,
    ) -> ArmRecord {
        let mut margins = [0.0; 6];
        reachy_kin::pose_margins(&cfg.geom, head_pose_body, &mut margins);
        ArmRecord {
            joints: *joints,
            head_pose_body: *head_pose_body,
            margins,
            min_margin: min_pose_margin(&cfg.geom, head_pose_body),
        }
    }

    /// A machine armed and holding at `targets`, through arming's own pin phase.
    ///
    /// A leg that `targets` puts outside its travel window is pinned at the
    /// nearer bound, and the state's Cartesian mirror is then the pose those
    /// pinned angles hold rather than the pose that was asked for. That matters
    /// for a rest tighter than the linkage's own limits: arming leaves such a
    /// machine in the window, and every trajectory therefore starts in it.
    ///
    /// With nothing to pin, the pose asked for *is* the pose the pinned angles
    /// hold, and it is used as the solve's exact answer rather than a value a
    /// rounding error away from it.
    fn armed_at(cfg: &MotionConfig, targets: &JointTargets) -> (MotionState, JointVector) {
        let (present, _) = joints_for(cfg, targets);
        let outcome =
            pin_goals(&arm_config(cfg), &present).expect("the pull-in is inside the gate");
        let record = if outcome.pinned.legs == present.legs {
            record_at(cfg, &outcome.pinned, &targets.head_pose_body)
        } else {
            ArmRecord::solve(
                &cfg.geom,
                &cfg.fk,
                &outcome.pinned,
                &[targets.head_pose_body],
            )
            .expect("the pinned angles close the linkage")
        };
        (MotionState::new_armed(&record), outcome.pinned)
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
        run_command(
            cfg,
            state,
            start,
            &MotionCommand::MoveTo {
                target: *target,
                durations: MoveDurations::uniform(duration),
                warp: Warp::MinJerk,
            },
            period,
        )
    }

    /// The same, for a command whose two group clocks differ.
    fn run_command(
        cfg: &MotionConfig,
        state: &mut MotionState,
        start: &JointVector,
        command: &MotionCommand,
        period: f64,
    ) -> (u32, TickOutputs) {
        let command = *command;
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

    /// The largest single-tick leg-goal step a stow-to-neutral move of
    /// `duration` emits, sampled through the IK at the tick rate.
    fn worst_leg_step(duration: f64) -> f64 {
        // A step bound wide enough to never fire: this measures the steps
        // rather than asking the guard about them.
        let mut cfg = MotionConfig::default();
        cfg.max_step.legs = f64::INFINITY;
        cfg.max_step.body_yaw = f64::INFINITY;
        cfg.max_step.antennas = f64::INFINITY;

        let stow = JointTargets {
            head_pose_body: reachy_kin::stow_head_pose(),
            ..JointTargets::default()
        };
        let (mut state, pinned) = armed_at(&cfg, &stow);
        let neutral = JointTargets::default();

        let command = MotionCommand::MoveTo {
            target: neutral,
            durations: MoveDurations::uniform(secs(duration)),
            warp: Warp::MinJerk,
        };
        let period = 1.0 / FLOOR_TICK_HZ;
        let mut present = pinned;
        let mut worst: f64 = 0.0;
        for n in 0..1000 {
            let command = (n == 0).then_some(&command);
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * period),
                &present,
                command,
            );
            assert_eq!(out.report.fault, None, "at {duration}s, tick {n}");
            if let Some(goal) = out.goal {
                for (leg, angle) in goal.legs.iter().enumerate() {
                    worst = worst.max((angle - present.legs[leg]).abs());
                }
                present = goal;
            }
            if out.report.completed {
                return worst;
            }
        }
        panic!("the move never completed at {duration}s");
    }

    /// The head group's floor, derived by running the move the daemon and the
    /// bench both make — stow to neutral, through the IK, at the tick rate — at
    /// decreasing durations until a leg step passes the bound.
    ///
    /// The number this pins is what the example TOMLs' comments cite. It is a
    /// property of the geometry and the tick rate, so it moves only when one of
    /// those does, and then this test says so.
    #[test]
    fn the_head_group_duration_floor_is_derived() {
        let bound = MotionConfig::default().max_step.legs;
        // Hundredths of a second, counted down as integers so the search grid
        // is exact and the number it lands on is the number quoted.
        let step = 0.01;
        let mut floor = None;
        for hundredths in (1..=200).rev() {
            let candidate = f64::from(hundredths) / 100.0;
            if worst_leg_step(candidate) > bound {
                break;
            }
            floor = Some(candidate);
        }
        let floor = floor.expect("two seconds is inside the bound");
        assert!(
            (floor - HEAD_GROUP_FLOOR_S).abs() < 1e-9,
            "the derived head-group floor is {floor}s, not the {HEAD_GROUP_FLOOR_S}s the example \
             configurations quote"
        );

        // And the guard is what enforces it: a move one search step under the
        // floor faults rather than being trimmed and sent.
        let cfg = MotionConfig::default();
        let stow = JointTargets {
            head_pose_body: reachy_kin::stow_head_pose(),
            ..JointTargets::default()
        };
        let (mut state, pinned) = armed_at(&cfg, &stow);
        let (_, out) = run_move(
            &cfg,
            &mut state,
            &pinned,
            &JointTargets::default(),
            secs(floor - step),
            1.0 / FLOOR_TICK_HZ,
        );
        assert!(
            matches!(out.report.fault, Some(Fault::StepTooLarge { .. })),
            "under the floor: {:?}",
            out.report.fault
        );
    }

    /// A stow-to-neutral command at a duration the span fits inside is handed
    /// back exactly as it came, with nothing to report.
    ///
    /// This is the ordinary case and it is the one that matters most: the
    /// configured durations are the policy, and a right-sizing pass that
    /// nudged a feasible move would have quietly taken that policy over.
    #[test]
    fn a_feasible_move_keeps_the_clock_it_was_given() {
        let cfg = MotionConfig::default();
        let stow = JointTargets {
            head_pose_body: reachy_kin::stow_head_pose(),
            ..JointTargets::default()
        };
        let (state, _) = armed_at(&cfg, &stow);
        let command = MotionCommand::MoveTo {
            target: JointTargets::default(),
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };

        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        assert_eq!(stretch, None);
        assert_eq!(floored, command);
    }

    /// A stow-to-neutral command under the derived head-group floor comes back
    /// on a clock at or above it, and that clock carries the move through.
    ///
    /// The requested clock is the one the derivation test pins as faulting, so
    /// the two halves are the same move: the original clock faults partway
    /// through; the stretched one completes.
    #[test]
    fn a_move_under_the_head_floor_is_stretched_past_it() {
        let cfg = MotionConfig::default();
        let stow = JointTargets {
            head_pose_body: reachy_kin::stow_head_pose(),
            ..JointTargets::default()
        };
        let requested = secs(0.5);
        let command = MotionCommand::MoveTo {
            target: JointTargets::default(),
            durations: MoveDurations::uniform(requested),
            warp: Warp::MinJerk,
        };

        let (mut state, pinned) = armed_at(&cfg, &stow);
        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("half a second cannot carry the fold");
        assert_eq!(stretch.requested, MoveDurations::uniform(requested));
        assert_eq!(stretch.effective, durations_of(&floored));
        assert!(
            stretch.effective.head.as_secs_f64() >= HEAD_GROUP_FLOOR_S,
            "stretched to {:?}, under the {HEAD_GROUP_FLOOR_S} s floor",
            stretch.effective.head,
        );

        let (_, out) = run_command(&cfg, &mut state, &pinned, &floored, 1.0 / FLOOR_TICK_HZ);
        assert_eq!(out.report.fault, None);
        assert!(out.report.completed, "{:?}", out.report);

        // And the clock it was asked for is the one that faults, so the
        // stretch is what stands between this move and a de-torqued machine.
        let (mut state, pinned) = armed_at(&cfg, &stow);
        let (_, out) = run_command(&cfg, &mut state, &pinned, &command, 1.0 / FLOOR_TICK_HZ);
        assert!(
            matches!(out.report.fault, Some(Fault::StepTooLarge { .. })),
            "{:?}",
            out.report.fault
        );
    }

    /// The body yaw and the antennas move linearly in their own coordinates, so
    /// what a stretch lands on is [`duration_floor_s`] of the span it covers —
    /// within the margin the stretch adds and never under the floor itself.
    ///
    /// Three independent clocks, checked one at a time: a yaw sweep leaves both
    /// antenna clocks alone, and one antenna's sweep leaves the head clock and
    /// the other antenna's alone, which is the whole point of the split.
    #[test]
    fn the_yaw_and_antenna_stretches_land_on_their_closed_forms() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &JointTargets::default());
        let requested = MoveDurations {
            head: secs(0.5),
            antennas: [secs(0.3), secs(0.35)],
        };

        let yaw_span = 1.0;
        let antenna_span = 3.0;
        // `None` is the head clock; `Some(side)` is that antenna's own.
        for (name, target, span, bound, clock) in [
            (
                "yaw",
                JointTargets {
                    body_yaw: yaw_span,
                    ..JointTargets::default()
                },
                yaw_span,
                cfg.max_step.body_yaw,
                None,
            ),
            (
                "right antenna",
                JointTargets {
                    antennas: [antenna_span, 0.0],
                    ..JointTargets::default()
                },
                antenna_span,
                cfg.max_step.antennas,
                Some(0),
            ),
            (
                // Mirrored, so the arc is the direct one: the left antenna's
                // outboard point sits where the right's does reflected, and a
                // positive sweep this wide would be sent the long way round it.
                "left antenna",
                JointTargets {
                    antennas: [0.0, -antenna_span],
                    ..JointTargets::default()
                },
                antenna_span,
                cfg.max_step.antennas,
                Some(1),
            ),
        ] {
            let command = MotionCommand::MoveTo {
                target,
                durations: requested,
                warp: Warp::MinJerk,
            };
            let (_, stretch) =
                floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
            let stretch = stretch.expect("the span does not fit the clock it was given");

            let floor = duration_floor_s(span, bound, FLOOR_TICK_HZ);
            let (moved, asked) = match clock {
                None => (stretch.effective.head, requested.head),
                Some(side) => (stretch.effective.antennas[side], requested.antennas[side]),
            };
            assert!(
                moved.as_secs_f64() >= floor,
                "{name}: stretched to {moved:?}, under its {floor:.4} s floor"
            );
            assert!(
                moved.as_secs_f64() <= floor * (1.0 + STRETCH_MARGIN),
                "{name}: stretched to {moved:?}, past the {floor:.4} s floor by more than the \
                 margin"
            );
            assert!(
                moved > asked,
                "{name}: {moved:?} is not longer than {asked:?}"
            );
            if clock.is_some() {
                assert_eq!(
                    stretch.effective.head, requested.head,
                    "{name}: the head clock moved"
                );
            }
            for side in 0..requested.antennas.len() {
                if clock != Some(side) {
                    assert_eq!(
                        stretch.effective.antennas[side], requested.antennas[side],
                        "{name}: the other antenna's clock moved"
                    );
                }
            }
        }
    }

    /// A move giving the two antennas different clocks is judged per side: the
    /// side whose clock cannot carry its own arc is stretched, and the side
    /// whose clock can is left exactly as asked. Staggering the pair — which is
    /// what stops two inboard sweeps meeting at the crossing point — must not
    /// collapse back onto one clock the first time one side is right-sized.
    #[test]
    fn one_antennas_short_clock_stretches_only_that_side() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &JointTargets::default());
        // The right sweeps far on a short clock; the left barely moves on a
        // shorter one and still fits.
        let requested = MoveDurations {
            head: secs(2.0),
            antennas: [secs(0.3), secs(0.2)],
        };
        let command = MotionCommand::MoveTo {
            target: JointTargets {
                antennas: [3.0, 0.05],
                ..JointTargets::default()
            },
            durations: requested,
            warp: Warp::MinJerk,
        };

        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("the right antenna's arc does not fit its clock");
        assert_eq!(stretch.requested, requested);
        assert!(
            stretch.effective.antennas[0] > requested.antennas[0],
            "the right antenna's clock is {:?}",
            stretch.effective.antennas[0]
        );
        assert_eq!(stretch.effective.antennas[1], requested.antennas[1]);
        assert_eq!(stretch.effective.head, requested.head);
        assert!(
            stretch.effective.antennas[0] > stretch.effective.antennas[1],
            "the two sides are still on different clocks"
        );

        // And the command handed on is the stretched one, unresolved target and
        // all.
        let MotionCommand::MoveTo { durations, .. } = floored else {
            panic!("a stretched move is still a move");
        };
        assert_eq!(durations, stretch.effective);
    }

    /// The recovery this exists for: a body a hand spun to the half turn while
    /// the machine lay limp folds to stow on a clock sized for the half turn,
    /// instead of faulting partway and dropping the head.
    ///
    /// Half a turn exceeds the ~153° a two-second clock covers at this step
    /// bound, and it is the unattended-at-boot case: nobody is there to catch
    /// the head or to restart the daemon.
    #[test]
    fn a_body_spun_to_the_half_turn_folds_on_a_stretched_clock() {
        let cfg = MotionConfig::default();
        let crooked = JointTargets {
            body_yaw: core::f64::consts::PI,
            ..JointTargets::default()
        };
        let stow = JointTargets {
            head_pose_body: reachy_kin::stow_head_pose(),
            ..JointTargets::default()
        };
        let shipped = secs(2.0);
        let command = MotionCommand::MoveTo {
            target: stow,
            durations: MoveDurations::uniform(shipped),
            warp: Warp::MinJerk,
        };

        // Unfloored, the shipped fold is the regression: it steps past the
        // bound partway round and stops the machine wherever that was.
        let (mut state, pinned) = armed_at(&cfg, &crooked);
        let (_, out) = run_command(&cfg, &mut state, &pinned, &command, 1.0 / FLOOR_TICK_HZ);
        assert!(
            matches!(
                out.report.fault,
                Some(Fault::StepTooLarge {
                    joint: JointId::BodyYaw,
                    ..
                })
            ),
            "{:?}",
            out.report.fault
        );

        let (mut state, pinned) = armed_at(&cfg, &crooked);
        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("a half turn does not fit a two second clock");
        let floor = duration_floor_s(core::f64::consts::PI, cfg.max_step.body_yaw, FLOOR_TICK_HZ);
        assert!(
            stretch.effective.head.as_secs_f64() >= floor,
            "stretched to {:?}, under the {floor:.4} s the half turn needs",
            stretch.effective.head,
        );

        let (_, out) = run_command(&cfg, &mut state, &pinned, &floored, 1.0 / FLOOR_TICK_HZ);
        assert_eq!(out.report.fault, None, "{:?}", out.report);
        assert!(out.report.completed, "{:?}", out.report);
        // The fold travelled the whole way: the recovery ends at stow, not
        // partway to it.
        assert!(
            state.last_targets().body_yaw.abs() < 1e-12,
            "left at {} rad",
            state.last_targets().body_yaw
        );
    }

    /// A stretched clock carries its move through a loop that runs late.
    ///
    /// Injects the worst alternation of the lateness the control loop treats
    /// as ordinary — half a period on every second tick — and expects the
    /// guard to stay quiet. A clock sized exactly to the bound would fault
    /// here on timing noise, not on span.
    #[test]
    fn a_stretched_move_survives_a_loop_running_late() {
        let cfg = MotionConfig::default();
        let crooked = JointTargets {
            body_yaw: core::f64::consts::PI,
            ..JointTargets::default()
        };
        let stow = JointTargets {
            head_pose_body: reachy_kin::stow_head_pose(),
            ..JointTargets::default()
        };
        let command = MotionCommand::MoveTo {
            target: stow,
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };

        let (mut state, pinned) = armed_at(&cfg, &crooked);
        let (floored, _) = floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);

        let period = 1.0 / FLOOR_TICK_HZ;
        let mut present = pinned;
        let mut out = TickOutputs::default();
        for n in 0..2000 {
            let late = if n % 2 == 1 { period / 2.0 } else { 0.0 };
            let now = secs(f64::from(n) * period + late);
            let command = (n == 0).then_some(&floored);
            out = tick_with(&cfg, &mut state, now, &present, command);
            if let Some(goal) = out.goal {
                present = goal;
            }
            if out.report.completed || out.report.fault.is_some() {
                break;
            }
        }
        assert_eq!(out.report.fault, None, "{:?}", out.report);
        assert!(out.report.completed, "{:?}", out.report);
    }

    /// A stretch is sized from where the move starts, so a replacement issued
    /// partway through a recovery is right-sized for the span still ahead of it
    /// rather than for the one the original command had.
    #[test]
    fn a_stretch_follows_the_span_still_ahead() {
        let cfg = MotionConfig::default();
        let requested = MoveDurations::uniform(secs(0.5));
        let stow = JointTargets {
            head_pose_body: reachy_kin::stow_head_pose(),
            ..JointTargets::default()
        };

        let effective_from = |yaw: f64| {
            let (state, _) = armed_at(
                &cfg,
                &JointTargets {
                    body_yaw: yaw,
                    ..JointTargets::default()
                },
            );
            let command = MotionCommand::MoveTo {
                target: stow,
                durations: requested,
                warp: Warp::MinJerk,
            };
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ)
                .1
                .expect("half a second carries neither span")
                .effective
                .head
                .as_secs_f64()
        };

        let far = effective_from(core::f64::consts::PI);
        let near = effective_from(core::f64::consts::FRAC_PI_2);
        assert!(
            far > near,
            "the half turn asked for {far:.4} s, the quarter for {near:.4} s"
        );
    }

    /// Nothing a right-sizing pass cannot measure is changed by it, and nothing
    /// it is not asked about either: a hold has no clock, and a move that
    /// cannot be shaped is the tick's refusal to make, not this pass's.
    ///
    /// Every bail-out — every way the pass hands the command back unchanged —
    /// is enumerated here. Losing one takes the pass somewhere it has no
    /// answer for.
    #[test]
    fn what_cannot_be_measured_is_handed_straight_back() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &JointTargets::default());

        for (command, tick_hz) in [
            (MotionCommand::Hold, FLOOR_TICK_HZ),
            // Zero on either clock: unshapeable, and refused by name at the
            // tick rather than stretched into something shapeable here.
            (
                MotionCommand::MoveTo {
                    target: JointTargets::default(),
                    durations: MoveDurations::uniform(Duration::ZERO),
                    warp: Warp::MinJerk,
                },
                FLOOR_TICK_HZ,
            ),
            // A head pose the linkage cannot close on: the path has samples
            // with no crank angles at all, so there is nothing to measure and
            // no clock that would help.
            (
                MotionCommand::MoveTo {
                    target: pose_at(10.0),
                    durations: MoveDurations::uniform(secs(1.0)),
                    warp: Warp::MinJerk,
                },
                FLOOR_TICK_HZ,
            ),
            // No control rate: there is no grid to sample the path on, and a
            // dry pass with no periods in it measures nothing.
            (
                MotionCommand::MoveTo {
                    target: pose_at(0.19),
                    durations: MoveDurations::uniform(secs(1.0)),
                    warp: Warp::MinJerk,
                },
                0.0,
            ),
            // A clock past what one pass may walk: forty minutes at the control
            // rate is more samples than `MAX_DRY_SAMPLES` allows, and a span
            // this machine can travel fits its bounds on a clock nothing like
            // that long anyway.
            (
                MotionCommand::MoveTo {
                    target: pose_at(0.19),
                    durations: MoveDurations::uniform(secs(
                        f64::from(MAX_DRY_SAMPLES) / FLOOR_TICK_HZ + 1.0,
                    )),
                    warp: Warp::MinJerk,
                },
                FLOOR_TICK_HZ,
            ),
        ] {
            let (floored, stretch) =
                floor_move_clock(&cfg, state.last_targets(), &command, tick_hz);
            assert_eq!(stretch, None, "{command:?} at {tick_hz} Hz");
            assert_eq!(floored, command, "{command:?} at {tick_hz} Hz");
        }
    }

    /// An antenna whose short arc would sweep through the outboard point goes
    /// the long way round, and the clock is sized for the arc the machine
    /// actually travels — not for the short one nobody commanded.
    ///
    /// Both halves of the one place this pass does wrap arithmetic. The dry
    /// sample runs over the resolved representative, so the span it measures is
    /// the long one; and what comes back carries the target exactly as it was
    /// handed in, because resolving an already-resolved direction a second time
    /// at the tick would flip it back to the short arc and run a path whose
    /// clock was never measured.
    #[test]
    fn an_antenna_that_wraps_is_floored_for_the_arc_it_sweeps() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &JointTargets::default());

        // The right antenna, from straight up: the short way to -2 rad crosses
        // its outboard direction at -π/2, so the sweep is the other way round.
        let short_arc = -2.0;
        let long_arc = short_arc + core::f64::consts::TAU;
        let command = MotionCommand::MoveTo {
            target: JointTargets {
                antennas: [short_arc, 0.0],
                ..JointTargets::default()
            },
            durations: MoveDurations::uniform(secs(0.5)),
            warp: Warp::MinJerk,
        };

        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("half a second cannot carry a turn of antenna");

        let long_floor = duration_floor_s(long_arc, cfg.max_step.antennas, FLOOR_TICK_HZ);
        let short_floor = duration_floor_s(short_arc.abs(), cfg.max_step.antennas, FLOOR_TICK_HZ);
        let moved = stretch.effective.antennas[0].as_secs_f64();
        assert!(
            moved >= long_floor,
            "stretched to {moved:.4} s, under the {long_floor:.4} s the long way round needs"
        );
        assert!(
            moved <= long_floor * (1.0 + STRETCH_MARGIN),
            "stretched to {moved:.4} s, past the {long_floor:.4} s floor by more than the margin"
        );
        assert!(
            moved > short_floor * (1.0 + STRETCH_MARGIN),
            "stretched to {moved:.4} s, which the {short_floor:.4} s short arc would have covered"
        );

        let MotionCommand::MoveTo { target, .. } = floored else {
            panic!("a move came back as {floored:?}");
        };
        assert_eq!(
            target.antennas,
            [short_arc, 0.0],
            "the target came back resolved, and the tick resolves it again"
        );
    }

    /// The durations a command carries, or the zero clock a hold has.
    fn durations_of(command: &MotionCommand) -> MoveDurations {
        match command {
            MotionCommand::MoveTo { durations, .. } => *durations,
            MotionCommand::Hold => MoveDurations::uniform(Duration::ZERO),
        }
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
            durations: MoveDurations::uniform(secs(2.0)),
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
            durations: MoveDurations::uniform(secs(2.0)),
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
            durations: MoveDurations::uniform(secs(2.0)),
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
            assert_eq!(out.report.tracking_worst(), None, "nothing was measured");
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

    /// A run needs the breach sustained: it clears on the first tick back
    /// inside the threshold, and only a run as long as the configured one
    /// faults. Two joints run out together here, and the one named is the
    /// furthest out.
    #[test]
    fn a_tracking_run_clears_on_a_good_tick() {
        let cfg = MotionConfig {
            tracking: TrackingFaultConfig {
                threshold_rad: 0.1,
                progress_min_rad: 0.01,
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
            out.report.tracking_worst().expect("measured").1 < 1e-12,
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
            "of the two runs that ran out together, the furthest out is named"
        );
        assert_eq!(
            out.report.tracking_count, 3,
            "the tick that ran the budget out reports the full run, not zero"
        );
    }

    /// Every joint's own distance from its goal rides the report, in bus order.
    ///
    /// The worst of them is what a fault names, but the nine are what a move
    /// accumulates into the lag it reports: which joint ran behind, and by how
    /// far, is the measurement the threshold, its window and the stow tolerance
    /// are all still provisional against. A single worst figure per tick would
    /// hide every joint but one.
    #[test]
    fn the_report_carries_each_joint_s_own_distance_from_its_goal() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        // Nine distinct offsets, each inside the threshold, so what is being
        // read back is the per-joint figure and not a fault's aftermath.
        let mut off = pinned;
        off.body_yaw += 0.01;
        for (n, leg) in off.legs.iter_mut().enumerate() {
            *leg += 0.02 * (n as f64 + 1.0);
        }
        off.antennas[0] -= 0.03;
        off.antennas[1] += 0.05;

        let out = tick_with(&cfg, &mut state, secs(0.0), &off, None);
        assert_eq!(out.report.fault, None);
        let errors = out.report.tracking_errors.expect("a live tick measures");
        for (row, ((joint, angle), (_, goal))) in
            off.joints().into_iter().zip(pinned.joints()).enumerate()
        {
            assert!(
                (errors[row] - (angle - goal).abs()).abs() < 1e-12,
                "{joint} reports {} against {}",
                errors[row],
                (angle - goal).abs()
            );
        }
        assert_eq!(
            out.report.tracking_worst(),
            Some((JointId::Leg(5), 0.12)),
            "the worst is the sweep of the nine, not a figure kept beside them"
        );

        // A tick with no read measured nothing, and says so rather than
        // repeating the last tick's nine numbers as if they were fresh.
        let mut stale = TickOutputs::default();
        motion_tick(
            &cfg,
            &mut state,
            &TickInputs {
                now: secs(0.02),
                present: None,
                command: None,
                health: None,
            },
            &mut stale,
        );
        assert_eq!(stale.report.tracking_errors, None);
    }

    /// A tracking configuration with everything else left at its default.
    fn tracking_cfg(threshold_rad: f64, progress_min_rad: f64, ticks: u32) -> MotionConfig {
        MotionConfig {
            tracking: TrackingFaultConfig {
                threshold_rad,
                progress_min_rad,
                ticks,
            },
            ..MotionConfig::default()
        }
    }

    /// A machine that reaches each goal `lag` ticks after it was written.
    ///
    /// The shape a proportional position loop's error actually has: distance
    /// behind the goal proportional to how fast the goal is moving, largest in
    /// the middle of a shaped move and zero at its ends. The fixture that
    /// hands the tick its own last goals cannot see this fault working at all,
    /// and the fixture that freezes the machine cannot see it staying quiet.
    struct Follower {
        /// Every goal written, oldest first.
        goals: Vec<JointVector>,
        /// How many ticks behind the machine runs.
        lag: usize,
        /// The worst tracking error any tick measured.
        worst: f64,
        /// The longest run any tick reported.
        longest: u32,
        /// Ticks issued so far, at 50 Hz.
        tick: u32,
    }

    impl Follower {
        fn new(start: &JointVector, lag: usize) -> Self {
            Self {
                goals: vec![*start],
                lag,
                worst: 0.0,
                longest: 0,
                tick: 0,
            }
        }

        /// Where the machine reports itself: the goal `lag` writes ago, or the
        /// pose it started from while the move is younger than that.
        fn present(&self) -> JointVector {
            self.goals[self.goals.len().saturating_sub(1 + self.lag)]
        }

        /// One tick, taking `command` if there is one.
        fn step(
            &mut self,
            cfg: &MotionConfig,
            state: &mut MotionState,
            command: Option<&MotionCommand>,
        ) -> TickOutputs {
            let present = self.present();
            let now = secs(f64::from(self.tick) * 0.02);
            let out = tick_with(cfg, state, now, &present, command);
            self.tick += 1;
            if let Some((_, error)) = out.report.tracking_worst() {
                self.worst = self.worst.max(error);
            }
            self.longest = self.longest.max(out.report.tracking_count);
            self.goals.push(*state.last_goal());
            out
        }

        /// Command a move and run it out, or until it faults.
        fn run(
            &mut self,
            cfg: &MotionConfig,
            state: &mut MotionState,
            command: &MotionCommand,
        ) -> TickOutputs {
            let mut out = TickOutputs::default();
            for n in 0..500 {
                out = self.step(cfg, state, (n == 0).then_some(command));
                if out.report.completed || out.report.fault.is_some() {
                    break;
                }
            }
            out
        }
    }

    fn move_to(target: JointTargets, duration: Duration) -> MotionCommand {
        MotionCommand::MoveTo {
            target,
            durations: MoveDurations::uniform(duration),
            warp: Warp::MinJerk,
        }
    }

    /// A lagging chase, and the move that produces one, in each of the two
    /// configurations this fault has to hold in.
    ///
    /// The first is tuned by hand — a tight threshold and a slow move, which
    /// separates the arithmetic from the numbers the machine happens to ship
    /// with. The second is what the bench ships and its resolver admits: every
    /// per-tick goal step bounded by a step guard no wider than the threshold,
    /// which is the relationship the reversal case below rests on. A joint
    /// following at a distance has to survive both.
    fn regimes() -> [(MotionConfig, JointTargets, Duration, usize); 2] {
        [
            (tracking_cfg(0.02, 0.002, 10), pose_at(0.19), secs(2.0), 8),
            (
                MotionConfig::default(),
                antennas_at([1.5, -1.5]),
                secs(0.8),
                8,
            ),
        ]
    }

    /// The step guard is no wider than the threshold in the shipped
    /// configuration, which is what stops a goal stepping over the band that
    /// clears a run. The bench's own resolver refuses a file that breaks it;
    /// this is the same relationship at the layer that depends on it.
    #[test]
    fn the_shipped_step_guard_is_inside_the_shipped_threshold() {
        let cfg = MotionConfig::default();
        for (joint, step) in [
            (JointId::Leg(0), cfg.max_step.legs),
            (JointId::BodyYaw, cfg.max_step.body_yaw),
            (JointId::AntennaRight, cfg.max_step.antennas),
        ] {
            assert!(
                step <= cfg.tracking.threshold_rad,
                "{joint} steps {step} against a {} threshold",
                cfg.tracking.threshold_rad
            );
        }
    }

    /// The fault this machine's own gains guarantee, and must not raise: a
    /// joint sitting well past the threshold for the whole middle of a move,
    /// closing on its goal the entire time, is following it.
    #[test]
    fn a_lagging_but_closing_joint_never_faults() {
        for (cfg, target, duration, lag) in regimes() {
            let start = JointTargets::default();
            let (mut state, pinned) = armed_at(&cfg, &start);

            let mut machine = Follower::new(&pinned, lag);
            let out = machine.run(&cfg, &mut state, &move_to(target, duration));

            assert_eq!(
                out.report.fault, None,
                "a closing joint is a tracking joint"
            );
            assert!(out.report.completed, "and the move finished");
            assert!(
                machine.worst > 2.0 * cfg.tracking.threshold_rad,
                "the lag has to clear the threshold for this to test anything: {}",
                machine.worst
            );
            assert!(
                machine.longest > 0 && machine.longest < cfg.tracking.ticks,
                "runs opened and were closed by progress, not by luck: {}",
                machine.longest
            );
        }
    }

    /// The detection this fault exists for: a goal that
    /// never applied, or a motor that cannot move, leaves the joint standing
    /// still with its goal beyond the threshold. Nothing closes, so the window
    /// runs out — on the tick the count reaches it, not one either side.
    #[test]
    fn a_stalled_joint_faults_at_exactly_the_window() {
        let cfg = tracking_cfg(0.1, 0.01, 5);
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());

        let mut stuck = pinned;
        stuck.body_yaw += 0.3;

        for n in 1..=5u32 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &stuck, None);
            assert_eq!(out.report.tracking_count, n, "tick {n}");
            if n < cfg.tracking.ticks {
                assert_eq!(out.report.fault, None, "tick {n}");
            } else {
                assert_eq!(
                    out.report.fault,
                    Some(Fault::TrackingLost {
                        joint: JointId::BodyYaw,
                        error: 0.3,
                    }),
                    "tick {n}"
                );
            }
        }
        assert!(state.is_faulted());
    }

    /// Progress buys one window, not immunity. A joint that closes on its goal
    /// and then stops dead — the arrival that never completes, a servo losing
    /// the last of a move — faults a window after it stopped, because progress
    /// is measured from where the closing left it rather than from where the
    /// run first opened.
    #[test]
    fn a_joint_that_stops_after_closing_still_faults() {
        let cfg = tracking_cfg(0.1, 0.01, 5);
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());

        let mut present = pinned;
        present.body_yaw = 0.5;
        let mut tick = 0u32;
        // Closing at twice the minimum every tick: the run restarts each time.
        for _ in 0..5 {
            tick += 1;
            present.body_yaw -= 0.02;
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(tick) * 0.02),
                &present,
                None,
            );
            assert_eq!(out.report.tracking_count, 1, "tick {tick}");
            assert_eq!(out.report.fault, None, "tick {tick}");
        }

        // Then stopped, still well past the threshold.
        let mut last = None;
        for expected in 2..=5u32 {
            tick += 1;
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(tick) * 0.02),
                &present,
                None,
            );
            assert_eq!(out.report.tracking_count, expected, "tick {tick}");
            last = out.report.fault;
        }
        assert!(
            matches!(
                last,
                Some(Fault::TrackingLost {
                    joint: JointId::BodyYaw,
                    ..
                })
            ),
            "expected a tracking fault, got {last:?}"
        );
    }

    /// Motion away from the goal is not progress. A sign error somewhere below
    /// this crate drives a joint the wrong way at full speed, and distance
    /// alone would call that a lag; the signed advance calls it what it is.
    #[test]
    fn a_joint_running_away_from_its_goal_faults() {
        let cfg = tracking_cfg(0.1, 0.01, 5);
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());

        let mut present = pinned;
        present.body_yaw = 0.15;
        let mut last = None;
        for n in 1..=5u32 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &present, None);
            assert_eq!(out.report.tracking_count, n, "tick {n}");
            last = out.report.fault;
            // Away from the goal at zero, and faster than the progress
            // minimum, so nothing here could be mistaken for a slow chase.
            present.body_yaw += 0.05;
        }
        let Some(Fault::TrackingLost { joint, .. }) = last else {
            panic!("expected a tracking fault, got {last:?}");
        };
        assert_eq!(joint, JointId::BodyYaw);
    }

    /// Motion with no net progress is not progress either: a joint hunting
    /// around a standing offset covers ground every tick and closes none of it.
    #[test]
    fn oscillation_without_net_progress_faults() {
        let cfg = tracking_cfg(0.1, 0.01, 5);
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());

        let mut present = pinned;
        let mut last = None;
        for n in 1..=5u32 {
            present.body_yaw = if n.is_multiple_of(2) { 0.25 } else { 0.2 };
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &present, None);
            assert_eq!(out.report.tracking_count, n, "tick {n}");
            last = out.report.fault;
        }
        let Some(Fault::TrackingLost { joint, .. }) = last else {
            panic!("expected a tracking fault, got {last:?}");
        };
        assert_eq!(joint, JointId::BodyYaw);
    }

    /// A goal that comes to rest exactly on an open run's anchor leaves no
    /// direction to close in, and a joint walking away from it is closing on
    /// nothing.
    ///
    /// `f64::signum` answers `+1` for a zero, so the signed advance has to name
    /// this case rather than multiply through it: fold the three lines into the
    /// single product and motion away from a stopped goal reads as progress,
    /// the run re-anchors on every tick, and the arrival failure never faults.
    #[test]
    fn a_goal_at_rest_on_the_anchor_closes_nothing() {
        let cfg = tracking_cfg(0.15, 0.01, 25);
        // A move that ends where this joint is already reading, so the run that
        // opens on the first tick anchors exactly where the goal comes to rest.
        let start = antennas_at([-0.4, 0.0]);
        let (mut state, pinned) = armed_at(&cfg, &start);
        let command = move_to(antennas_at([0.0, 0.0]), secs(0.3));

        let mut present = pinned;
        let mut arrived = None;
        let mut faulted = None;
        for n in 0..cfg.tracking.ticks {
            // Walking away from the goal, in the direction the anchor lies from
            // it, and faster every tick than the progress minimum.
            present.antennas[0] = 0.05 * f64::from(n);
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                (n == 0).then_some(&command),
            );
            if let Some(fault) = out.report.fault {
                let Fault::TrackingLost { joint, .. } = fault else {
                    panic!("expected a tracking fault at tick {n}, got {fault}");
                };
                assert_eq!(joint, JointId::AntennaRight);
                faulted = Some(n);
                break;
            }
            assert_eq!(
                out.report.tracking_count,
                n + 1,
                "the run re-anchored at tick {n}"
            );
            if arrived.is_none() && state.last_goal().antennas[0] == 0.0 {
                arrived = Some(n);
            }
        }

        let arrived = arrived.expect("the goal comes to rest on the anchor");
        let faulted = faulted.expect("a joint walking away from a stopped goal faults");
        assert_eq!(faulted, cfg.tracking.ticks - 1);
        assert!(
            faulted >= arrived + 5,
            "the run stood on a goal at rest for {} ticks",
            faulted - arrived
        );
    }

    /// The progress minimum is a rate, and it is the line between the two: a
    /// joint closing less than it over a whole window faults, and the same
    /// crawl a shade faster runs indefinitely. Both sit far past the threshold
    /// throughout, so only the closing rate separates them.
    #[test]
    fn a_crawl_below_the_progress_minimum_faults_and_one_above_it_does_not() {
        let cfg = tracking_cfg(0.1, 0.01, 10);

        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        let mut present = pinned;
        present.body_yaw = 0.3;
        let mut last = None;
        for n in 1..=10u32 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &present, None);
            assert_eq!(out.report.tracking_count, n, "tick {n}");
            last = out.report.fault;
            // 0.0009 a tick is 0.0081 over the window, just under the minimum.
            present.body_yaw -= 0.0009;
        }
        assert!(
            matches!(
                last,
                Some(Fault::TrackingLost {
                    joint: JointId::BodyYaw,
                    ..
                })
            ),
            "expected a tracking fault, got {last:?}"
        );

        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        let mut present = pinned;
        present.body_yaw = 0.3;
        for n in 1..=40u32 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &present, None);
            assert_eq!(out.report.fault, None, "tick {n}");
            assert!(out.report.tracking_count <= cfg.tracking.ticks, "tick {n}");
            // 0.0012 a tick is 0.0108 over the window, just over it.
            present.body_yaw -= 0.0012;
        }
        assert!(
            present.body_yaw > cfg.tracking.threshold_rad,
            "still past the threshold the whole way: {}",
            present.body_yaw
        );
    }

    /// A tick that measured nothing freezes every run where it stands. Counting
    /// stale ticks would run the window out on a read outage — during which the
    /// move goes on, by design — and blame a joint nobody looked at.
    #[test]
    fn stale_ticks_freeze_every_run() {
        let cfg = tracking_cfg(0.1, 0.01, 5);
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());

        let mut stuck = pinned;
        stuck.body_yaw += 0.3;

        for n in 1..=3u32 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &stuck, None);
            assert_eq!(out.report.tracking_count, n);
        }

        for n in 4..=13u32 {
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
            assert_eq!(
                out.report.tracking_worst(),
                None,
                "tick {n} measured nothing"
            );
            assert_eq!(out.report.tracking_count, 3, "tick {n} froze the run");
            assert_eq!(out.report.fault, None, "tick {n}");
        }

        let out = tick_with(&cfg, &mut state, secs(0.28), &stuck, None);
        assert_eq!(out.report.tracking_count, 4, "the run resumes where it was");
        assert_eq!(out.report.fault, None);
        let out = tick_with(&cfg, &mut state, secs(0.30), &stuck, None);
        assert_eq!(
            out.report.fault,
            Some(Fault::TrackingLost {
                joint: JointId::BodyYaw,
                error: 0.3,
            })
        );
    }

    /// A lagging joint through a goal that reverses under it, in both the ways
    /// this system produces one: a move abandoned and reversed mid-flight, and
    /// two moves issued back to back the way `demo` does.
    ///
    /// The reversal is the case the signed advance could get wrong — a goal
    /// crossing the anchor flips the sign of progress. It cannot get there: a
    /// goal on its way to the anchor passes through the threshold band around
    /// the joint first, which clears the run.
    #[test]
    fn a_lagging_joint_survives_a_goal_reversal() {
        for (cfg, target, duration, lag) in regimes() {
            let start = JointTargets::default();

            // Back to back: the second move starts on the tick after the first
            // finished, with the machine still that many goals behind.
            let (mut state, pinned) = armed_at(&cfg, &start);
            let mut machine = Follower::new(&pinned, lag);
            let out = machine.run(&cfg, &mut state, &move_to(target, duration));
            assert!(out.report.completed && out.report.fault.is_none());
            let out = machine.run(&cfg, &mut state, &move_to(start, duration));
            assert_eq!(out.report.fault, None, "the reversal faulted nothing");
            assert!(out.report.completed);
            assert!(
                machine.worst > cfg.tracking.threshold_rad,
                "the machine lagged through both moves: {}",
                machine.worst
            );

            // Mid-flight: hold, then reverse, while the lag is at its largest.
            let (mut state, pinned) = armed_at(&cfg, &start);
            let mut machine = Follower::new(&pinned, lag);
            // Halfway through the move at 50 Hz, where a min-jerk goal is
            // moving fastest and the chase is furthest behind.
            let half = (duration.as_secs_f64() * 25.0).round() as u32;
            for n in 0..half {
                let command = move_to(target, duration);
                let out = machine.step(&cfg, &mut state, (n == 0).then_some(&command));
                assert_eq!(out.report.fault, None, "tick {n}");
            }
            let held = machine.step(&cfg, &mut state, Some(&MotionCommand::Hold));
            assert_eq!(held.report.command, CommandDisposition::Held);
            let out = machine.run(&cfg, &mut state, &move_to(start, duration));
            assert_eq!(
                out.report.fault, None,
                "the mid-flight reversal faulted nothing"
            );
            assert!(out.report.completed);
            assert!(
                machine.worst > cfg.tracking.threshold_rad,
                "and it was lagging when the goal turned round: {}",
                machine.worst
            );
        }
    }

    /// The nine runs are nine runs. A joint closing on its goal, and a joint
    /// inside the threshold, neither hold up nor reset the run of the joint
    /// that is stuck — and the fault names the joint whose window ran out, not
    /// the one furthest from its goal.
    #[test]
    fn the_window_that_ran_out_names_its_own_joint() {
        let cfg = tracking_cfg(0.1, 0.01, 5);
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());

        let mut present = pinned;
        // Stuck, just past the threshold.
        present.body_yaw = 0.15;
        // Four times as far out, and closing on its goal every tick.
        present.antennas[0] = 0.6;
        // Comfortably inside the threshold, and staying there.
        present.antennas[1] = 0.05;

        let mut last = None;
        for n in 1..=5u32 {
            present.antennas[0] -= 0.03;
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &present, None);
            assert_eq!(
                out.report.tracking_worst().expect("measured").0,
                JointId::AntennaRight,
                "tick {n}: the antenna is the furthest from its goal throughout"
            );
            assert_eq!(
                out.report.tracking_count, n,
                "tick {n}: the stuck joint's run is untouched by the other two"
            );
            last = out.report.fault;
        }
        assert_eq!(
            last,
            Some(Fault::TrackingLost {
                joint: JointId::BodyYaw,
                error: 0.15,
            })
        );
    }

    /// A second move while one is running replaces it, from the setpoint the
    /// last tick commanded. The machine follows the latest intent it was given;
    /// nothing queues and nothing is refused for arriving mid-move.
    #[test]
    fn a_move_while_moving_retargets() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let command = MotionCommand::MoveTo {
            target: pose_at(0.19),
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };

        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&command));
        assert_eq!(out.report.command, CommandDisposition::Started);
        assert_eq!(out.report.fault, None, "accepting it did not also fault");

        // Far enough in for the first path to be somewhere between its
        // endpoints, so the splice has a setpoint that is neither.
        let mut present = pinned;
        for n in 1..30 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &present, None);
            if let Some(goal) = out.goal {
                present = goal;
            }
        }
        let spliced = *state.last_targets();
        let travelled = spliced.head_pose_body.translation.z;
        assert!(
            travelled > 0.001 && travelled < 0.18,
            "mid-flight at {travelled}"
        );

        let other = MotionCommand::MoveTo {
            target: pose_at(0.15),
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };
        let out = tick_with(&cfg, &mut state, secs(0.6), &present, Some(&other));
        assert_eq!(out.report.command, CommandDisposition::Retargeted);
        assert_eq!(out.report.fault, None, "and replacing it did not fault");
        assert_eq!(
            state.mode(),
            Mode::Moving { started: secs(0.6) },
            "the clock restarts with the new path"
        );
        // Zero elapsed time on the new path: it starts where the old one had
        // got to, so this tick asks for nothing and emits nothing.
        assert!(out.report.start_sample);
        assert!(out.goal.is_none());
        assert_eq!(
            state.last_targets().head_pose_body.translation.z,
            travelled,
            "the splice is the setpoint, not a jump"
        );

        // And it carries to the new target, not the abandoned one.
        let mut completed = false;
        for n in 31..=131 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &present, None);
            if let Some(goal) = out.goal {
                present = goal;
            }
            completed |= out.report.completed;
        }
        assert!(completed, "the replacement path reached its endpoint");
        assert_eq!(state.mode(), Mode::Holding);
        assert!(
            (state.last_targets().head_pose_body.translation.z - 0.15).abs() < 1e-12,
            "ended at {}",
            state.last_targets().head_pose_body.translation.z
        );
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
            durations: MoveDurations::uniform(Duration::ZERO),
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
            durations: MoveDurations::uniform(secs(2.0)),
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
            durations: MoveDurations::uniform(secs(0.5)),
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

    /// The recorded resting configuration puts four legs past their travel
    /// windows, which is the case arming's pin phase exists for, and pinning
    /// seats all six inside — including under the round trip through the pose
    /// those pinned angles hold, which is what every later trajectory starts
    /// from.
    #[test]
    fn pinning_seats_the_rest_configuration_in_its_windows() {
        let cfg = MotionConfig::default();
        let rest = rest_targets(0.0);
        let (raw, _) = joints_for(&cfg, &rest);

        let overrun = |legs: [f64; 6]| -> [f64; 6] {
            core::array::from_fn(|leg| {
                let (lo, hi) = cfg.env.crank_windows[leg];
                (lo - legs[leg]).max(legs[leg] - hi).max(0.0).to_degrees()
            })
        };
        assert_eq!(
            format!("{:.3?}", overrun(raw.legs)),
            "[7.531, 2.442, 0.000, 0.000, 0.755, 10.563]",
            "how far the raw rest IK overruns each window, degrees"
        );

        let (state, pinned) = armed_at(&cfg, &rest);
        assert_eq!(
            format!("{:.3?}", overrun(pinned.legs)),
            "[0.000, 0.000, 0.000, 0.000, 0.000, 0.000]",
            "the pinned goals are inside every window"
        );

        // The pose those pinned angles hold, solved back to angles by the
        // envelope, must also be inside: the trajectory starts from the pose,
        // not from the goals.
        let mut report = EnvelopeReport::default();
        let held = state.last_targets();
        let verdict = check_envelope(
            &cfg.geom,
            &cfg.env,
            &held.head_pose_body,
            held.body_yaw,
            None,
            &mut report,
        );
        assert_eq!(report.violations.window, [false; 6], "{verdict:?}");
        assert_eq!(report.violations.unreachable, [false; 6]);
        assert!(
            report.violations.margin,
            "and it is still tighter than the floor"
        );
        assert_eq!(
            format!("{:.9}", state.present_min_margin()),
            "0.000841568",
            "the armed clearance, metres"
        );
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
            state.present_min_margin() < cfg.env.min_toggle_margin,
            "the armed rest is tighter than the floor: {}",
            state.present_min_margin()
        );

        // One millimetre up: still far below the floor, admitted because it
        // improves on the measured clearance.
        let lift = rest_targets(0.001);
        let command = MotionCommand::MoveTo {
            target: lift,
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };
        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&command));
        assert_eq!(out.report.command, CommandDisposition::Started);
        assert_eq!(out.report.fault, None);

        // Downward from the same rest is a tightening, and no baseline excuses
        // that.
        let (mut state, pinned) = armed_at(&cfg, &rest);
        let drop = rest_targets(-0.001);
        let command = MotionCommand::MoveTo {
            target: drop,
            durations: MoveDurations::uniform(secs(2.0)),
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
        assert_eq!(out.report.fault, None);
    }

    /// The whole point of the two rules above, composed: the milestone's first
    /// commanded motion is the lift off the rest, and it runs from end to end
    /// without a single fault.
    ///
    /// It exercises the pinned start, the start-sample exemption on the accepting
    /// tick, and the margin baseline carrying a path that spends its first
    /// samples below the clearance floor. Any one of the three missing and this
    /// stops on tick 0 or shortly after.
    #[test]
    fn the_lift_off_the_rest_never_faults() {
        let cfg = MotionConfig::default();
        let (mut state, pinned) = armed_at(&cfg, &rest_targets(0.0));
        let neutral = JointTargets::default();
        let command = MotionCommand::MoveTo {
            target: neutral,
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };

        let mut present = pinned;
        let mut emitted = 0;
        let mut ticks = 0;
        let mut completed = false;
        for n in 0..200 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                (n == 0).then_some(&command),
            );
            ticks += 1;
            assert_eq!(out.report.fault, None, "tick {n}");
            if n == 0 {
                assert_eq!(out.report.command, CommandDisposition::Started);
            }
            if let Some(goal) = out.goal {
                emitted += 1;
                present = goal;
            }
            if out.report.completed {
                completed = true;
                break;
            }
        }
        assert!(completed, "the lift finished in {ticks} ticks");
        assert_eq!(state.mode(), Mode::Holding);
        assert!(emitted > 90, "only {emitted} goals over a 2 s lift");

        // It arrived, and it arrived somewhere with real clearance.
        assert_eq!(
            state.last_targets().head_pose_body,
            neutral.head_pose_body,
            "the endpoint is the target's own bits"
        );
        assert!(
            state.present_min_margin() > cfg.env.min_toggle_margin,
            "clearance at the top of the lift: {}",
            state.present_min_margin()
        );
    }

    /// A machine found outside the envelope can be commanded back inside it.
    ///
    /// A hand or a crash can leave the body turned past its yaw cap, and taking
    /// hold no longer refuses that — where the machine stands is not refusable.
    /// What has to follow is the recovery: a stow whose every sample fails the
    /// same cap the start fails, run to completion, never further out than where
    /// it began.
    #[test]
    fn a_move_out_of_a_pose_the_envelope_refuses_runs() {
        let cfg = MotionConfig::default();
        let turned = cfg.env.body_yaw_limit + 0.05;
        let crooked = JointTargets {
            body_yaw: turned,
            ..JointTargets::default()
        };
        let (mut state, pinned) = armed_at(&cfg, &crooked);
        let square = JointTargets::default();

        // Long enough that the yaw sweep back inside the cap clears the
        // per-tick step bound: a recovery is a move like any other and its
        // duration is floored by how far it has to travel.
        let (ticks, out) = run_move(&cfg, &mut state, &pinned, &square, secs(4.0), 0.02);
        assert_eq!(out.report.fault, None, "the recovery ran to its end");
        assert!(out.report.completed, "it completed in {ticks} ticks");
        assert_eq!(state.last_targets().body_yaw, 0.0, "square at the end");
    }

    /// The recovery is an allowance to travel back in, not a licence to stay
    /// out: every sample it commands is checked against where the move began,
    /// and the ticks that spend the allowance say so.
    #[test]
    fn a_recovering_move_never_commands_further_out_than_it_started() {
        let cfg = MotionConfig::default();
        let turned = cfg.env.body_yaw_limit + 0.05;
        let crooked = JointTargets {
            body_yaw: turned,
            ..JointTargets::default()
        };
        let (mut state, pinned) = armed_at(&cfg, &crooked);
        let command = MotionCommand::MoveTo {
            target: JointTargets::default(),
            durations: MoveDurations::uniform(secs(4.0)),
            warp: Warp::MinJerk,
        };

        let mut present = pinned;
        let mut recovering = 0;
        for n in 0..200 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                (n == 0).then_some(&command),
            );
            assert_eq!(out.report.fault, None, "tick {n}");
            if out.report.recovering {
                recovering += 1;
                let envelope = out.report.envelope.expect("a checked sample");
                assert!(envelope.violations.body_yaw, "tick {n} was out of the cap");
            }
            if let Some(goal) = out.goal {
                assert!(
                    goal.body_yaw <= turned,
                    "tick {n} commanded {} rad, past the {turned} rad it started at",
                    goal.body_yaw
                );
                present = goal;
            }
            if out.report.completed {
                break;
            }
        }
        assert!(
            recovering > 0,
            "the cap was outstanding for part of the move"
        );
        assert!(!state.is_faulted());

        // And the allowance is spent by the end: with the machine square again,
        // a move that would leave the cap is the fault it always was.
        let over = JointTargets {
            body_yaw: turned,
            ..JointTargets::default()
        };
        let out = tick_with(
            &cfg,
            &mut state,
            secs(10.0),
            &present,
            Some(&MotionCommand::MoveTo {
                target: over,
                durations: MoveDurations::uniform(secs(2.0)),
                warp: Warp::MinJerk,
            }),
        );
        assert!(
            matches!(
                out.report.command,
                CommandDisposition::Rejected(CommandRejection::Envelope(_))
            ),
            "{:?}",
            out.report.command
        );
    }

    /// The excursion arithmetic, on the bounds a recovery is judged against.
    #[test]
    fn an_excursion_measures_how_far_outside_each_bound_a_pose_is() {
        let env = EnvelopeConfig::default();
        let (lo, hi) = env.crank_windows[2];
        let mut report = EnvelopeReport {
            leg_angles: Some(LegAngles(core::array::from_fn(|leg| {
                let (lo, hi) = env.crank_windows[leg];
                (lo + hi) / 2.0
            }))),
            relative_yaw: env.relative_yaw_limit + 0.2,
            cone_angle: env.head_cone_limit - 0.1,
            ..EnvelopeReport::default()
        };

        let inside = Excursion::of(&env, &report, 0.0);
        assert_eq!(inside.window, [0.0; 6], "every crank is mid-window");
        assert_eq!(inside.body_yaw, 0.0);
        assert!((inside.relative_yaw - 0.2).abs() < 1e-12);
        assert_eq!(inside.cone, 0.0, "inside the cone is no excursion at all");

        // Past a window in either direction, and past the yaw cap.
        let mut angles = report.leg_angles.expect("set above");
        angles.0[2] = hi + 0.3;
        report.leg_angles = Some(angles);
        let above = Excursion::of(&env, &report, -env.body_yaw_limit - 0.4);
        assert!((above.window[2] - 0.3).abs() < 1e-12);
        assert!(
            (above.body_yaw - 0.4).abs() < 1e-12,
            "the cap is on magnitude"
        );
        angles.0[2] = lo - 0.5;
        report.leg_angles = Some(angles);
        let below = Excursion::of(&env, &report, 0.0);
        assert!((below.window[2] - 0.5).abs() < 1e-12);

        // A pose no further out than the start is admitted; one further out on
        // any single bound is not, and a reading nobody can place never is.
        assert!(above.no_further_out_than(&above));
        assert!(inside.no_further_out_than(&above));
        assert!(!above.no_further_out_than(&inside));
        assert!(!below.no_further_out_than(&above));
        let unplaceable = Excursion::of(&env, &report, f64::NAN);
        assert_eq!(unplaceable.body_yaw, f64::INFINITY);
        assert!(!unplaceable.no_further_out_than(&below));
    }

    /// The tick that accepts a move samples the move's own start — the pose
    /// already held — so it checks nothing and commands nothing, and says so. The
    /// next tick is the first that asks anything of the machine.
    ///
    /// Run from a rest below the clearance floor, which is where it matters: the
    /// start's clearance equals the baseline it would be compared against, so
    /// checking it would refuse the pose the machine is standing in.
    #[test]
    fn a_moves_first_tick_samples_its_own_start() {
        let cfg = MotionConfig::default();
        let (mut state, pinned) = armed_at(&cfg, &rest_targets(0.0));
        let held = *state.last_targets();
        let command = MotionCommand::MoveTo {
            target: JointTargets::default(),
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };

        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&command));
        assert_eq!(out.report.command, CommandDisposition::Started);
        assert!(out.report.start_sample);
        assert_eq!(out.report.envelope, None, "nothing was checked");
        assert_eq!(out.goal, None, "and nothing went out");
        assert!(!out.report.emitted);
        assert_eq!(out.report.fault, None);
        assert!(matches!(out.report.mode, Mode::Moving { .. }));
        assert_eq!(*state.last_goal(), pinned, "the goals are untouched");
        assert_eq!(*state.last_targets(), held, "and so is their mirror");

        // The next tick asks for a pose the machine is not in, and that one is
        // checked and emitted.
        let out = tick_with(&cfg, &mut state, secs(0.02), &pinned, None);
        assert!(!out.report.start_sample);
        assert!(out.report.envelope.is_some(), "this one was checked");
        assert!(out.report.emitted);
        assert_eq!(out.report.fault, None);
        assert_ne!(*state.last_goal(), pinned);
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
                progress_min_rad: 0.01,
                ticks: 1,
            },
            ..MotionConfig::default()
        };
        let targets = JointTargets::default();
        let (present, _) = joints_for(&cfg, &targets);

        let mut pinned = present;
        pinned.legs[3] = f64::NAN;
        let mut state = MotionState::new_armed(&record_at(&cfg, &pinned, &targets.head_pose_body));

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
            durations: MoveDurations::uniform(secs(2.0)),
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
            durations: MoveDurations::uniform(secs(1.0)),
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
            durations: MoveDurations::uniform(secs(1.0)),
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
            durations: MoveDurations::uniform(secs(2.0)),
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
            assert_eq!(out.report.tracking_worst(), None, "nothing was measured");
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
            durations: MoveDurations::uniform(secs(2.0)),
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

        // One tick's worth of elapsed time, long after twenty ticks have passed:
        // the path is resampled near its start and the head walks back there.
        // The whole move's leg travel is under the step bound here, so nothing
        // catches it.
        let out = tick_with(&cfg, &mut state, secs(0.02), &present, None);
        assert_eq!(out.report.fault, None);
        assert!(out.report.emitted, "a goal went out for the earlier time");
        let rewound = *state.last_goal();
        assert_ne!(rewound, advanced);
        for (leg, angle) in rewound.legs.iter().enumerate() {
            assert!(
                (angle - pinned.legs[leg]).abs() < 1e-4,
                "leg {leg} went back to within a tick of the start"
            );
        }

        // Rewound to at or before the move's own start, there is no elapsed time
        // at all, and that is the start-sample exemption: nothing is checked and
        // nothing goes out, so this particular clock fault cannot walk the head
        // anywhere.
        let out = tick_with(&cfg, &mut state, secs(0.0), &present, None);
        assert!(out.report.start_sample);
        assert_eq!(out.goal, None);
        assert_eq!(out.report.fault, None);
        assert_eq!(
            *state.last_goal(),
            rewound,
            "still where the rewind left it"
        );

        // A rewind that undoes more than one tick's worth of travel is caught,
        // and that is the whole protection: the same move on an axis with real
        // travel in it faults on the step rather than walking back.
        let (mut state, pinned) = armed_at(&cfg, &start);
        let yaw_move = MotionCommand::MoveTo {
            target: JointTargets {
                body_yaw: 0.5,
                ..JointTargets::default()
            },
            durations: MoveDurations::uniform(secs(2.0)),
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
        let out = tick_with(&cfg, &mut state, secs(0.02), &present, None);
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
            (
                CommandRejection::Envelope(violations),
                "the commanded target is outside the envelope: leg 1 unreachable, toggle margin below the floor",
            ),
            (
                CommandRejection::Trajectory(TrajectoryError::NonPositiveDuration),
                "the commanded move cannot be shaped: trajectory duration must be greater than zero",
            ),
            (
                CommandRejection::AntennaUnreachable {
                    joint: JointId::AntennaLeft,
                    angle: 2000.0,
                },
                "the commanded left antenna angle 2000.0000 rad is not commandable",
            ),
        ];
        for (refusal, expected) in refusals {
            assert_eq!(refusal.to_string(), expected);
        }
    }

    /// The nine angles a set of antenna directions asks for, at the neutral
    /// head pose.
    fn antennas_at(antennas: [f64; 2]) -> JointTargets {
        JointTargets {
            antennas,
            ..JointTargets::default()
        }
    }

    /// An antenna target is a direction, resolved against the frame the machine
    /// is already in. Run 4's geometry: the right antenna reads +162.334° after
    /// torque-on and stow's fold is −174.752°, which is 22.9° away the near way
    /// and 337° away the way a raw difference asks for. The near way is what
    /// gets commanded.
    #[test]
    fn an_antenna_direction_resolves_to_the_nearest_representative() {
        let cfg = MotionConfig::default();
        let start = antennas_at([162.334_f64.to_radians(), 0.0]);
        let (mut state, pinned) = armed_at(&cfg, &start);

        let (_, out) = run_move(
            &cfg,
            &mut state,
            &pinned,
            &antennas_at([-3.05, 0.0]),
            secs(2.0),
            0.02,
        );
        assert!(out.report.completed, "{:?}", out.report.fault);

        // It ended pointing at the fold, measured around the circle.
        let landed = state.last_targets().antennas[0];
        assert!(
            wrap_to_pi(landed - (-3.05)).abs() < 1e-12,
            "landed {landed}"
        );

        // And it got there the short way: 22.9° of sweep, not 337°.
        let swept = landed - start.antennas[0];
        assert!(
            swept > 0.0 && swept < core::f64::consts::PI,
            "swept {} rad",
            swept
        );
        assert!(
            (swept.to_degrees() - 22.914).abs() < 1e-3,
            "swept {}°",
            swept.to_degrees()
        );
    }

    /// Every antenna goal a move emits, in order, for one side.
    fn antenna_series(
        cfg: &MotionConfig,
        state: &mut MotionState,
        start: &JointVector,
        target: &JointTargets,
        side: usize,
    ) -> Vec<f64> {
        let command = MotionCommand::MoveTo {
            target: *target,
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };
        let mut present = *start;
        let mut series = Vec::new();
        for n in 0..200 {
            let command = (n == 0).then_some(&command);
            let out = tick_with(cfg, state, secs(f64::from(n) * 0.02), &present, command);
            assert_eq!(out.report.fault, None, "tick {n}");
            if let Some(goal) = out.goal {
                series.push(goal.antennas[side]);
                present = goal;
            }
        }
        series
    }

    /// The arc policy: a sweep takes the way round that misses the antenna's own
    /// outboard sideways point, because that is the direction that sweeps the
    /// widest envelope past whatever is standing beside the machine. Stow to
    /// neutral and back is the move this exists for — the short way is 3.05 rad
    /// straight through sideways, the way taken is 3.23 rad over the head.
    #[test]
    fn an_antenna_sweep_misses_its_outboard_point() {
        let cfg = MotionConfig::default();
        for (side, outboard) in ANTENNA_OUTBOARD.into_iter().enumerate() {
            let stow = if side == 0 { -3.05 } else { 3.05 };
            for (from, to) in [(stow, 0.0), (0.0, stow)] {
                let mut antennas = [0.0; 2];
                antennas[side] = from;
                let start = antennas_at(antennas);
                let (mut state, pinned) = armed_at(&cfg, &start);
                antennas[side] = to;

                let series =
                    antenna_series(&cfg, &mut state, &pinned, &antennas_at(antennas), side);
                assert!(!series.is_empty(), "antenna {side} was commanded");
                for goal in &series {
                    assert!(
                        wrap_to_pi(goal - outboard).abs() > 0.5,
                        "antenna {side} passed its outboard point at {goal} going {from} -> {to}"
                    );
                }
                let landed = *series.last().expect("a last goal");
                assert!(wrap_to_pi(landed - to).abs() < 1e-9, "landed at {landed}");
                let swept = (landed - from).abs();
                assert!(
                    (swept - (core::f64::consts::TAU - 3.05)).abs() < 1e-9,
                    "antenna {side} swept {swept} rad going {from} -> {to}"
                );
            }
        }
    }

    /// An antenna already standing at its outboard point takes the shortest
    /// path away from it: the endpoint counts as not crossing, so nothing sends
    /// a sideways antenna the long way round to come down.
    #[test]
    fn an_antenna_at_sideways_takes_the_short_way() {
        let cfg = MotionConfig::default();
        for (side, outboard) in ANTENNA_OUTBOARD.into_iter().enumerate() {
            let stow = if side == 0 { -3.05 } else { 3.05 };
            let mut antennas = [0.0; 2];
            antennas[side] = outboard;
            let start = antennas_at(antennas);
            let (mut state, pinned) = armed_at(&cfg, &start);

            // Down to stow, which is the near side from sideways.
            antennas[side] = stow;
            let (_, out) = run_move(
                &cfg,
                &mut state,
                &pinned,
                &antennas_at(antennas),
                secs(2.0),
                0.02,
            );
            assert!(out.report.completed, "{:?}", out.report.fault);
            let landed = state.last_targets().antennas[side];
            let swept = (landed - outboard).abs();
            assert!(
                (swept - (3.05 - core::f64::consts::FRAC_PI_2)).abs() < 1e-9,
                "antenna {side} swept {swept} rad from sideways"
            );
        }
    }

    /// And an antenna commanded *to* its outboard point arrives the short way,
    /// for the same reason: arriving at the point is not passing through it.
    #[test]
    fn an_antenna_commanded_to_sideways_arrives_the_short_way() {
        let cfg = MotionConfig::default();
        for (side, outboard) in ANTENNA_OUTBOARD.into_iter().enumerate() {
            let stow = if side == 0 { -3.05 } else { 3.05 };
            let mut antennas = [0.0; 2];
            antennas[side] = stow;
            let start = antennas_at(antennas);
            let (mut state, pinned) = armed_at(&cfg, &start);

            antennas[side] = outboard;
            let (_, out) = run_move(
                &cfg,
                &mut state,
                &pinned,
                &antennas_at(antennas),
                secs(2.0),
                0.02,
            );
            assert!(out.report.completed, "{:?}", out.report.fault);
            let landed = state.last_targets().antennas[side];
            let swept = (landed - stow).abs();
            assert!(
                (swept - (3.05 - core::f64::consts::FRAC_PI_2)).abs() < 1e-9,
                "antenna {side} swept {swept} rad to sideways"
            );
        }
    }

    /// The outboard constants are the physical sideways direction — a quarter
    /// turn either side of straight up — and not the midpoint of the arc the
    /// stow angles happen to span, which is 2.6° off it. A machine whose stow
    /// fold moved would keep the same two constants.
    #[test]
    fn the_outboard_directions_are_horizontal() {
        assert_eq!(
            ANTENNA_OUTBOARD,
            [-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2]
        );
        let stow_midpoint = 3.05 / 2.0;
        assert!(
            (stow_midpoint - core::f64::consts::FRAC_PI_2).abs() > 0.04,
            "the stow arc's midpoint is {stow_midpoint}, which is not horizontal"
        );
    }

    /// A direction a whole turn from the frame is the direction the machine is
    /// already pointing, so it commands nothing at all.
    #[test]
    fn a_direction_a_whole_turn_from_the_frame_commands_no_motion() {
        let cfg = MotionConfig::default();
        let frame = 0.6;
        let start = antennas_at([frame, frame]);
        let (mut state, pinned) = armed_at(&cfg, &start);

        let turns = core::f64::consts::TAU;
        let (_, out) = run_move(
            &cfg,
            &mut state,
            &pinned,
            &antennas_at([frame + turns, frame - 3.0 * turns]),
            secs(2.0),
            0.02,
        );
        assert!(out.report.completed, "{:?}", out.report.fault);
        for side in 0..2 {
            let landed = state.last_targets().antennas[side];
            assert!((landed - frame).abs() < 1e-12, "antenna {side} at {landed}");
        }
    }

    /// Consecutive moves chain in the continuous frame: a machine found ten
    /// turns out from zero takes stow, then neutral, then stow again, each a
    /// sweep under a whole turn, with no step and no fault. The frame it ends in
    /// is ten turns from zero still — nothing renormalises it — and every one of
    /// those poses points where it was asked to.
    #[test]
    fn antenna_moves_chain_in_the_continuous_frame() {
        let cfg = MotionConfig::default();
        let turns = 10.0 * core::f64::consts::TAU;
        let start = antennas_at([turns, -turns]);
        let (mut state, mut pinned) = armed_at(&cfg, &start);

        let mut previous = start.antennas;
        for directions in [[-3.05, 3.05], [0.0, 0.0], [-3.05, 3.05]] {
            let (_, out) = run_move(
                &cfg,
                &mut state,
                &pinned,
                &antennas_at(directions),
                secs(2.0),
                0.02,
            );
            assert!(out.report.completed, "{:?}", out.report.fault);
            pinned = out.goal.unwrap_or(pinned);

            let landed = state.last_targets().antennas;
            for side in 0..2 {
                assert!(
                    wrap_to_pi(landed[side] - directions[side]).abs() < 1e-12,
                    "antenna {side} landed at {} for {}",
                    landed[side],
                    directions[side]
                );
                let swept = (landed[side] - previous[side]).abs();
                assert!(
                    swept < core::f64::consts::TAU,
                    "antenna {side} swept {swept} rad"
                );
                assert!(
                    (landed[side].abs() - turns).abs() < core::f64::consts::TAU,
                    "antenna {side} stayed in the frame it was found in: {}",
                    landed[side]
                );
            }
            previous = landed;
        }
    }

    /// The one limit an antenna has: extended position mode's goal register
    /// reaches ±1_048_575 counts and no further. The two ends are not mirror
    /// images — zero radians sits half a turn up the count frame — so each is
    /// pinned in its own direction. A direction nobody can place is a typed
    /// refusal; a direction whose preferred arc has no count takes the other
    /// arc, which is a turn back inside the range. Neither is saturated, and
    /// the refusal does not disturb a holding machine.
    #[test]
    fn an_antenna_goal_no_count_represents_is_refused() {
        let cfg = MotionConfig::default();
        let edges = [
            (ANTENNA_GOAL_MAX_RAD, ANTENNA_GOAL_MAX_RAD + 1.0),
            (ANTENNA_GOAL_MIN_RAD, ANTENNA_GOAL_MIN_RAD - 1.0),
        ];
        for (edge, past) in edges {
            let start = antennas_at([edge, 0.0]);
            let (mut state, pinned) = armed_at(&cfg, &start);

            for direction in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let command = MotionCommand::MoveTo {
                    target: antennas_at([direction, 0.0]),
                    durations: MoveDurations::uniform(secs(2.0)),
                    warp: Warp::MinJerk,
                };
                let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&command));
                let CommandDisposition::Rejected(CommandRejection::AntennaUnreachable {
                    joint,
                    angle,
                }) = out.report.command
                else {
                    panic!(
                        "expected a refusal for {direction}, got {:?}",
                        out.report.command
                    );
                };
                assert_eq!(joint, JointId::AntennaRight);
                assert!(!angle.is_finite(), "{direction} resolved to {angle}");
                // Refused, so nothing moved and nothing went out.
                assert_eq!(out.report.mode, Mode::Holding);
                assert!(out.goal.is_none());
                assert_eq!(state.last_targets().antennas, start.antennas);
            }

            // A placeable direction off the end of the register is not a
            // refusal: the two candidate arcs are a turn apart and the range is
            // 512 turns wide, so one of them is always inside it.
            let (_, out) = run_move(
                &cfg,
                &mut state,
                &pinned,
                &antennas_at([past, 0.0]),
                secs(2.0),
                0.02,
            );
            assert!(out.report.completed, "{:?}", out.report.fault);
            let goal = state.last_targets().antennas[0];
            assert!(
                (ANTENNA_GOAL_MIN_RAD..=ANTENNA_GOAL_MAX_RAD).contains(&goal),
                "{past} resolved to {goal}"
            );
            assert!(
                wrap_to_pi(goal - past).abs() < 1e-9,
                "{past} resolved to {goal}, which points elsewhere"
            );

            // The range's own edge is inside it: a bound admits its bound.
            let out = tick_with(
                &cfg,
                &mut state,
                secs(0.0),
                &pinned,
                Some(&MotionCommand::MoveTo {
                    target: antennas_at([edge, 0.0]),
                    durations: MoveDurations::uniform(secs(2.0)),
                    warp: Warp::MinJerk,
                }),
            );
            assert_eq!(out.report.command, CommandDisposition::Started);
        }

        // Half a turn apart from symmetric, and each end a whole turn clear of
        // the other's magnitude: the asymmetry is the count frame's offset, not
        // a rounding artefact.
        let offset = ANTENNA_GOAL_MAX_RAD + ANTENNA_GOAL_MIN_RAD;
        assert!(
            (offset + core::f64::consts::TAU).abs() < 1e-9,
            "the two ends sit {offset} rad apart from mirror images"
        );
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
            durations: MoveDurations::uniform(secs(2.0)),
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
