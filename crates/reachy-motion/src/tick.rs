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
//! Any stage can raise a [`Fault`], and every fault carries the [`Response`]
//! that answers it. This layer owns no wire, so it performs none of them: what
//! it hands back is the verdict, and the caller driving the bus is what writes
//! torque.
//!
//! What the tick does with its own state depends on which answer that is. A
//! fault saying control itself cannot be trusted — the feedback gone, the
//! mechanism outside its model — is absorbing: the tick stops commanding, every
//! subsequent tick emits nothing, and nothing here recovers on its own, because
//! all that is left is for the caller to cut torque. Every other fault leaves
//! the tick **holding**, the move abandoned and the last commanded goal
//! standing, because the answer to it is a stow that has to be driven through
//! this same state.
//!
//! A fault confined to a joint group takes only that group out of service: its
//! joints go into the **mask**, and the move carries on without them. A masked
//! joint is commanded nothing and checked for nothing, the raise checks
//! included — entry into the mask *is* the raise, so error bits that latch in a
//! servo report forever and raise once. The caller torques a newly masked servo
//! off; masked means released, not merely unspoken to.
//!
//! A command whose target fails the envelope is deliberately *not* a fault: it
//! is **rejected** and reported, because an armed, holding machine must not be
//! bricked by someone typing a pose it cannot reach.
//!
//! Between the two sits the [`MoveAbort`]: the running move is abandoned and
//! the tick goes back to holding the last goal it commanded, healthy and
//! commandable. A sampled *path* pose that fails the envelope after its target
//! passed, and a goal stepping further in one period than the step guard
//! admits, are both of these — the interpolator and the checker disagreeing
//! about a pose already accepted, or a planner producing a discontinuity. The
//! sample is never emitted, and what the caller does about it is a wind-down
//! under control, not a de-torqued machine.
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
//! different clocks stays staggered — and a pair that is *not* staggered enough
//! to keep its tips from meeting at their crossing is staggered there, on the
//! clocks the move ends up with. Clocks are only ever lengthened, so the
//! path is preserved exactly and merely traversed more slowly, which is the
//! right degradation for a move whose only sin is starting further away than
//! the knob assumed. The step
//! guard below is untouched by any of it and stays the backstop for genuine
//! runaway bugs.
//!
//! ## The move's clock is the tick's, not the wall's
//!
//! The trajectory is not sampled at wall time. Each executed tick credits the
//! running move with the time since the previous tick, capped at one nominal
//! period, so the grid the live loop samples on is the grid the dry pass
//! measured. A period that begins late advances the path by one nominal step
//! and no more: lateness delays arrival by exactly the lateness and can never
//! inflate a commanded step. What the step guard bounds is therefore what the
//! planner produced, which is the only thing it can say anything useful about.
//! A clock that goes backwards credits nothing, and the move stands still until
//! it catches up.
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
//! numbers, and a read carrying a value nobody can place is discarded and
//! counted as one that did not arrive rather than fed to the solvers. A read
//! that arrives and solves to no believable pose is skipped too, on a run of
//! its own. A stale tick is marked stale in the report and counts toward the
//! read-loss fault; nothing downstream ever sees a stale reading presented as a
//! live one. The same all-or-nothing rule governs the health poll.
//!
//! One bad frame is never a verdict about the machine. Both runs are bounded by
//! `read_loss_ticks`, and reaching either of those bounds is what says the
//! feedback path or the mechanism itself has gone.
//!
//! A stale tick does **not** stop an active move. It skips the tracking
//! comparison — the one thing that needs a live reading — and otherwise advances,
//! checks and emits as usual, so a validated move rides out a brief read outage
//! instead of stalling halfway. What bounds that is `read_loss_ticks`: it is the
//! length of the outage the machine may keep moving blind through, and sizing it
//! is sizing exactly that.

use core::time::Duration;
use std::sync::mpsc::Receiver;

use nalgebra::Isometry3;
use reachy_kin::{
    EnvelopeConfig, EnvelopeReport, EnvelopeViolations, FkError, FkOptions, FkStats, HeadGeometry,
    LegAngles, below_limit, check_envelope, forward_kinematics, min_pose_margin, outside_limit,
    wrap_to_pi,
};
use thiserror::Error;

use crate::arm::ArmRecord;
use crate::joints::{
    JointGroup, JointId, JointSet, JointStep, JointTargets, JointVector, ServoHealth, worst_joint,
};
use crate::phase::{AntennaPhaseConfig, PhaseSeparation, PhaseWatch};
use crate::seq::SeqError;
use crate::timeline::{Entry, FaultTimeline, Maneuver, Outcome};
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
/// Distance alone cannot say that. What a servo's integral term closes is a
/// standing error and not a moving one, so a joint chasing a streamed goal sits
/// behind it by roughly the commanded velocity times the loop's own time
/// constant, whatever the gains. At the
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
    /// A screen for which joints are worth measuring, independent of the step
    /// bounds. Progress is measured from where the joint stood when its run
    /// opened and signed toward the goal, so a goal that moves to the far side
    /// of that anchor would read a joint that is following as one running away
    /// from it — which is why a run whose goal crosses its anchor re-anchors
    /// where the joint now stands and opens a fresh window instead. Whatever
    /// distance one period's goal covers, it cannot turn a chase into a fault.
    pub threshold_rad: f64,
    /// How far a joint past that threshold must travel toward its goal, within
    /// a window of `ticks`, to count as closing on it, radians.
    pub progress_min_rad: f64,
    /// How many consecutive live ticks past the threshold without that
    /// progress before the fault.
    pub ticks: u32,
}

impl Default for TrackingFaultConfig {
    /// The threshold is measured off the recorded runs this repo keeps as
    /// fixtures; the window and the progress minimum are sized rather than
    /// measured.
    fn default() -> Self {
        Self {
            // 28.6° of crank, better than twice the worst lag any head joint
            // ran at on a healthy recorded gesture (0.245 rad on a loaded leg
            // at 3.3 rad/s of commanded speed). Only a screen for which joints
            // are worth examining, not a verdict: the antennas cross it while
            // following perfectly — 1.38 rad behind an 855°/s sweep — and
            // re-anchor rather than fault, because what decides a stall is
            // whether the joint is closing.
            threshold_rad: 0.5,
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
/// opened, or when it last restarted — rather than between consecutive ticks,
/// so a joint creeping less than one count per tick still shows its motion over
/// the window.
#[derive(Clone, Copy, Debug, PartialEq)]
struct TrackingStreak {
    /// Where the joint was when this run last restarted, radians.
    anchor: f64,
    /// Which side of `anchor` the goal lay on then.
    ///
    /// The direction progress is signed in. A goal that moves to the other side
    /// is measuring the joint against a distance that no longer exists, so the
    /// run restarts rather than reading the joint as running away.
    side: Side,
    /// Live ticks since, this one included.
    count: u32,
}

/// Which side of a tracking run's anchor its goal lies on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Side {
    /// Above the anchor: closing means rising.
    Above,
    /// Below it: closing means falling.
    Below,
    /// Neither — the goal sits on the anchor, or is a number nobody can place.
    /// There is no direction to close in and no side to cross to.
    Unplaced,
}

impl Side {
    /// Which side of `anchor` `goal` lies on.
    ///
    /// The one constructor, so the sign the run is measured in and the distance
    /// it is measured over are read off the same subtraction.
    fn of(goal: f64, anchor: f64) -> Self {
        let toward = goal - anchor;
        if toward > 0.0 {
            Self::Above
        } else if toward < 0.0 {
            Self::Below
        } else {
            Self::Unplaced
        }
    }

    /// How far a joint that has moved `travelled` from the anchor got toward a
    /// goal on this side: positive is closing, negative is running away.
    ///
    /// A goal with no side leaves nothing to close, so nothing counts as
    /// progress toward it.
    fn advance(self, travelled: f64) -> f64 {
        match self {
            Self::Above => travelled,
            Self::Below => -travelled,
            Self::Unplaced => 0.0,
        }
    }

    /// Whether the goal has crossed the anchor: both sides placed, and opposite.
    fn crossed_from(self, was: Self) -> bool {
        matches!(
            (was, self),
            (Self::Above, Self::Below) | (Self::Below, Self::Above)
        )
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
    /// Per-tick step bounds on the goals the planner produces. Exceeding one
    /// abandons the move, and is never a clamp.
    pub max_step: JointStep,
    /// When to call tracking lost.
    pub tracking: TrackingFaultConfig,
    /// Where the antennas' tips can meet, and how far apart in phase a
    /// commanded pair has to cross there.
    pub phase: AntennaPhaseConfig,
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
            // What these bound is the plan, so they are sized off the fastest
            // motion the machine is known to make well and left wide enough
            // that only a discontinuity reaches them.
            max_step: JointStep {
                // The validated 0.8 s presence gesture peaks at 0.067 rad per
                // leg per period at 50 Hz, dry-sampled through the inverse
                // kinematics. Better than twice that.
                legs: 0.15,
                // Analogy rather than measurement: no yaw speed trial has been
                // run. The body yaw is the legs' servo model, unloaded and
                // moving linearly in its own coordinate, so it carries their
                // figure until a supervised fold measures its own.
                body_yaw: 0.15,
                // The fastest sweep on record — an antenna crossing 3.22 rad in
                // 0.3 s, 855°/s — plans a peak of 0.403 rad per period at
                // 50 Hz. Better than half as much again. A sweep takes the arc
                // that misses its outboard direction, so the widest one
                // commandable is just under a full turn: 6.28 rad, which needs
                // 0.36 s to stay inside this.
                antennas: 0.65,
            },
            tracking: TrackingFaultConfig::default(),
            phase: AntennaPhaseConfig::default(),
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
/// checks the shipped values against. Under it the guard abandons the move
/// rather than clamping it: a shipped duration below this is presence that
/// never works.
pub const HEAD_GROUP_FLOOR_S: f64 = 0.36;

/// What the tick is doing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Mode {
    /// No trajectory running; the servos hold their last goal.
    Holding,
    /// A trajectory is being sampled, this far into its own clock.
    Moving {
        /// The move's own elapsed time: one nominal period per executed tick,
        /// or the real gap when that was shorter. Zero on the tick that
        /// accepted the move.
        elapsed: Duration,
    },
    /// Stopped commanding, for this reason, until an operator intervenes.
    ///
    /// Only the faults that say control itself cannot be trusted land here
    /// ([`Fault::latches`]). A grabbed head and a released servo leave the tick
    /// holding instead, because the maneuver that answers them is a stow this
    /// same tick has to drive.
    Faulted(Fault),
}

/// What a fault is answered with: a maneuver and the state it leaves behind.
///
/// One vocabulary for the whole stack, so the bench and the daemon act on the
/// same six answers and neither re-derives one from a message. The
/// classification happens once, at [`Fault::response`], and travels as this
/// value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Response {
    /// The ask is declined and nothing changes. Nothing was energized, or
    /// nothing needs to be.
    Refuse,
    /// Abandon the move and stow every commanded joint under control, full
    /// checks live, then release. The machine is trusted to command.
    SlowStowToRest,
    /// Torque off the antenna group and stop commanding it; the head is
    /// unaffected and the move carries on. The session continues degraded until
    /// the next engage retries them.
    DegradeAntennas,
    /// Torque off the faulted servo on the spot and stow on what still
    /// commands, then release everything and wait for an operator.
    MaskedSlowStowToPark,
    /// Immediate best-effort torque-off of all nine, then rest: the next wake
    /// re-engages normally.
    ImmediateAllTorqueOffToRest,
    /// Immediate best-effort torque-off of all nine, then wait for an operator.
    ImmediateAllTorqueOffToPark,
}

impl Response {
    /// What this response does to the machine, less where it leaves it.
    ///
    /// The half of a response that is worth watching happen, and so the half
    /// the timeline records. A refusal has none: nothing is done to a machine
    /// whose ask was declined.
    #[must_use]
    pub fn maneuver(self) -> Option<Maneuver> {
        match self {
            Self::Refuse => None,
            Self::SlowStowToRest => Some(Maneuver::SlowStow),
            Self::DegradeAntennas => Some(Maneuver::AntennaTorqueOff),
            Self::MaskedSlowStowToPark => Some(Maneuver::MaskedSlowStow),
            Self::ImmediateAllTorqueOffToRest | Self::ImmediateAllTorqueOffToPark => {
                Some(Maneuver::ImmediateAllTorqueOff)
            }
        }
    }
}

/// A condition of the machine that a maneuver has to answer.
///
/// One variant per named condition, and the name is the vocabulary the whole
/// stack reports in. The criterion for being here at all is the doctrine's: a
/// faulted motor is one that can no longer be commanded, or a mechanism that
/// can no longer be commanded safely. A software defect with a healthy machine
/// is not one of these — that is a [`MoveAbort`] — and neither is our own
/// accounting running out.
///
/// The tick raises the first six. The caller driving the bus raises
/// [`Self::BusFailure`] and [`Self::TorqueOffUnconfirmed`], which are verdicts
/// about transactions rather than about anything a control step can see.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum Fault {
    /// An antenna sat past the tracking threshold for a whole window without
    /// closing on its goal: interference, a snag, or a hand.
    #[error("{joint} is {error:.4} rad from its goal and not closing")]
    AntennaObstructed {
        /// The antenna whose window ran out.
        joint: JointId,
        /// How far it was from its goal, radians.
        error: f64,
    },
    /// An antenna servo reported a hardware error beyond the input-voltage bit.
    /// Never rebooted automatically.
    #[error("{joint} (servo {id}) reports hardware error bits {bits:#04x}")]
    AntennaServoFault {
        /// The antenna concerned.
        joint: JointId,
        /// The reporting servo's bus ID.
        id: u8,
        /// Its hardware-error byte.
        bits: u8,
    },
    /// A leg or the body yaw sat past the tracking threshold for a whole window
    /// without closing on its goal: a grab, a snag, or a jam.
    ///
    /// Not a motor failure — the servo still commands — so the answer is a
    /// controlled stow, which is also what helps a hand pushing the head down.
    #[error("{joint} is {error:.4} rad from its goal and not closing")]
    HeadObstructed {
        /// The joint whose window ran out, or the furthest out of those whose
        /// windows ran out together.
        joint: JointId,
        /// How far, radians.
        error: f64,
    },
    /// A leg or body-yaw servo reported a hardware error beyond the
    /// input-voltage bit. Never rebooted automatically: a reboot drops the
    /// head.
    #[error("{joint} (servo {id}) reports hardware error bits {bits:#04x}")]
    HeadServoFault {
        /// The joint whose servo it is.
        joint: JointId,
        /// The reporting servo's bus ID.
        id: u8,
        /// Its hardware-error byte.
        bits: u8,
    },
    /// Too many consecutive ticks with no usable position read. A read carrying
    /// a number nobody can place is one of them.
    ///
    /// Commanding blind is commanding a machine nothing is watching, and
    /// whatever took the reads away — a bus, a connector — makes the writes
    /// equally suspect.
    #[error("no position read for {misses} consecutive ticks")]
    PositionFeedbackLost {
        /// Consecutive missed reads.
        misses: u32,
    },
    /// The measured crank angles yielded no believable head pose for a whole
    /// run of live reads, so there is nothing to command from. Each solve is
    /// tried once, from the previous tick's pose; the solver is never re-run on
    /// perturbed inputs.
    ///
    /// A run and not a single frame: one unsolvable read is a frame the control
    /// path skips, and only a mechanism that stays outside its own model — a
    /// linkage forced, dislocated, or taken apart — keeps producing them.
    #[error("present pose unknown for {failures} consecutive live reads: {source}")]
    MeasuredPoseInvalid {
        /// Consecutive live reads whose pose solve failed.
        failures: u32,
        /// What the last of them failed with.
        source: FkError,
    },
    /// Transactions are failing under torque, so the machine can no longer be
    /// commanded — and a machine that cannot be commanded cannot be
    /// manoeuvred.
    #[error("the bus is not carrying commands: {source}")]
    BusFailure {
        /// What the failing transaction reported.
        source: SeqError,
    },
    /// A torque-off write went unacknowledged after every attempt, so the
    /// minimum risk condition is believed rather than known.
    #[error("servo {id} did not acknowledge torque off and may still be holding")]
    TorqueOffUnconfirmed {
        /// The servo that did not answer.
        id: u8,
    },
}

impl Fault {
    /// The fault's slug — the name it is reported, alerted and logged under.
    ///
    /// One name per condition, everywhere: the timeline entry, the operator
    /// line and the daemon's fault cell all say this word, so an operator
    /// reading a log and an operator reading a status file are reading about
    /// the same thing.
    #[must_use]
    pub fn slug(&self) -> &'static str {
        match self {
            Self::AntennaObstructed { .. } => "antenna_obstructed",
            Self::AntennaServoFault { .. } => "antenna_servo_fault",
            Self::HeadObstructed { .. } => "head_obstructed",
            Self::HeadServoFault { .. } => "head_servo_fault",
            Self::PositionFeedbackLost { .. } => "position_feedback_lost",
            Self::MeasuredPoseInvalid { .. } => "measured_pose_invalid",
            Self::BusFailure { .. } => "bus_failure",
            Self::TorqueOffUnconfirmed { .. } => "torque_off_unconfirmed",
        }
    }

    /// The maneuver and post-state this fault is answered with.
    ///
    /// The single classification point in the stack. Exhaustive by
    /// construction: a new fault is a classification decision made here, at
    /// compile time, and never a default anybody falls through to.
    #[must_use]
    pub fn response(&self) -> Response {
        match self {
            // Each antenna is its own non-load-bearing joint, so its trouble
            // stays its own: the pair goes limp and the head keeps its
            // presence.
            Self::AntennaObstructed { .. } | Self::AntennaServoFault { .. } => {
                Response::DegradeAntennas
            }
            // The motors still command, so the machine yields under control
            // rather than dropping — which is also what a hand pushing the head
            // down wants.
            Self::HeadObstructed { .. } => Response::SlowStowToRest,
            // Semi-controlled descent: the faulted servo is released on the
            // spot and the rest carry the head down, then everything releases
            // and waits for an operator.
            Self::HeadServoFault { .. } => Response::MaskedSlowStowToPark,
            // Control is not trusted: without feedback, or against a mechanism
            // outside its own model, a stow is a maneuver commanded blind or a
            // maneuver that grinds.
            Self::PositionFeedbackLost { .. } | Self::MeasuredPoseInvalid { .. } => {
                Response::ImmediateAllTorqueOffToPark
            }
            // Nothing can be commanded, so nothing controlled can be attempted.
            Self::BusFailure { .. } => Response::ImmediateAllTorqueOffToPark,
            // Degenerate: the torque-off already ran. What remains is the park
            // and the alert, because an unconfirmed release is never Resting.
            Self::TorqueOffUnconfirmed { .. } => Response::ImmediateAllTorqueOffToPark,
        }
    }

    /// Whether this fault takes the tick out of service, rather than leaving it
    /// commanding what remains.
    ///
    /// True only where control itself is what stopped being trustworthy. A
    /// grabbed head and a released servo both leave a machine that still takes
    /// goals, and the wind-down that answers them is driven through this same
    /// tick.
    #[must_use]
    pub fn latches(&self) -> bool {
        matches!(
            self.response(),
            Response::ImmediateAllTorqueOffToPark | Response::ImmediateAllTorqueOffToRest
        )
    }
}

/// Why a move stopped, with the machine still healthy and still commandable.
///
/// Not a fault: both of these say the plan was wrong, not the platform. The
/// offending sample is never emitted, the trajectory is dropped, and the tick
/// goes back to [`Mode::Holding`] at the last goal it commanded — so a caller
/// can wind the machine down under control, which is the whole difference
/// between a planner bug and a motor that no longer answers.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum MoveAbort {
    /// A sampled path pose failed the envelope after its target had passed it.
    #[error("the commanded path left the envelope: {0}")]
    EnvelopePath(EnvelopeViolations),
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
    /// Consecutive ticks without a position read, this one included. A read
    /// carrying a value nobody can place is one of them.
    pub misses: u32,
    /// Consecutive live reads whose pose solve failed, this one included.
    ///
    /// Its own run, counted separately from `misses`: a read that arrives and
    /// cannot be solved says something different about the machine from one
    /// that never arrives, and each has to reach its own fault.
    pub pose_failures: u32,
    /// The longest open run of live ticks with one joint past the tracking
    /// threshold and not closing on its goal, including the tick that raised
    /// [`Fault::HeadObstructed`]. A tick without a live read measures nothing and
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
    ///
    /// The move is over when this is set: either the tick has stopped
    /// commanding, or it is holding and waiting for the caller to wind the
    /// machine down. A fault confined to a joint group is not here — it is
    /// `degraded`, and the move carries on.
    pub fault: Option<Fault>,
    /// A fault confined to a joint group, raised on this tick. Its joints are
    /// in `newly_masked`; the move carries on without them.
    pub degraded: Option<Fault>,
    /// Every joint out of service, this tick's entries included.
    ///
    /// Masked joints are commanded nothing and checked for nothing. The caller
    /// owns the wire, so it is the caller that torques them off; what this says
    /// is which joints the tick has stopped speaking for.
    pub masked: JointSet,
    /// The joints that entered the mask on this tick — the ones the caller has
    /// to release before it writes another goal.
    pub newly_masked: JointSet,
    /// The move abandoned on this tick, and why. Stamped once, on the tick that
    /// dropped the trajectory; the machine is holding afterwards, not faulted,
    /// so nothing repeats it.
    pub aborted: Option<MoveAbort>,
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
            pose_failures: 0,
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
            degraded: None,
            masked: JointSet::EMPTY,
            newly_masked: JointSet::EMPTY,
            aborted: None,
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
    /// Read only as the gap since the previous tick, which is what advances a
    /// move's own clock. Nothing here reads a clock, so nothing here can enforce
    /// the ordering; a `now` that went backwards advances the move by nothing
    /// and leaves the machine holding where the last tick put it, rather than
    /// walking the path back the way it came.
    pub now: Duration,
    /// The control period the loop is paced at.
    ///
    /// The most one tick may advance a move by, whatever the wall clock did:
    /// the cap is what keeps the live sampling grid identical to the one the
    /// clock was sized against.
    pub period: Duration,
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
    ///
    /// All nine, including any joint the report says is masked: the plan is
    /// left whole and the mask says which rows of it may reach the wire. The
    /// caller writes the unmasked ones — a masked servo has been torqued off
    /// and is never commanded again.
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
    /// The `now` of the previous tick, or `None` before the first one.
    prev_now: Option<Duration>,
    last_goal: JointVector,
    last_targets: JointTargets,
    fk_seed: Isometry3<f64>,
    present_min_margin: f64,
    start_excursion: Excursion,
    miss_count: u32,
    pose_failures: u32,
    tracking: TrackingMonitor,
    masked: JointSet,
    /// What this session has raised and what was done about it.
    ///
    /// Here because the session is the story's scope and this struct is the
    /// session: a state built by arming and dropped by the release that ends
    /// the engagement bounds the record exactly.
    timeline: FaultTimeline,
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
    /// `degraded` is what arming left out of service — the antennas whose
    /// latched error bits the health gate found and engaged around, limp and
    /// never enabled. They start in the mask, so nothing here ever commands
    /// them, checks them or raises on their standing bits.
    ///
    /// There is no other way to build one, and that is the point: a tick can
    /// only run on a machine somebody armed.
    #[must_use]
    pub fn new_armed(armed: &ArmRecord, degraded: JointSet) -> Self {
        Self {
            mode: Mode::Holding,
            trajectory: None,
            prev_now: None,
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
            pose_failures: 0,
            tracking: TrackingMonitor::new(),
            masked: degraded,
            timeline: FaultTimeline::new(),
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

    /// The joints out of service.
    #[must_use]
    pub fn masked(&self) -> JointSet {
        self.masked
    }

    /// What this session has raised and what was done about it, so far.
    ///
    /// The pull half of the reporting rule: readable at any point while the
    /// session runs, and read as data — the entries are typed, and nothing
    /// downstream is asked to parse a sentence.
    #[must_use]
    pub fn timeline(&self) -> &FaultTimeline {
        &self.timeline
    }

    /// Receive each timeline entry as it appends.
    ///
    /// The push half. A daemon that alerts on faults subscribes once, at the
    /// top of the session, and never polls.
    pub fn subscribe_timeline(&mut self) -> Receiver<Entry> {
        self.timeline.subscribe()
    }

    /// The record, kept past the session that made it.
    #[must_use]
    pub fn into_timeline(self) -> FaultTimeline {
        self.timeline
    }

    /// Record a fault nothing in here could see.
    ///
    /// For conditions only the wire-holding layer can see — bus failures,
    /// unconfirmed torque-off — that belong in the same record as the faults
    /// the tick records itself via [`Self::raise`] and [`Self::degrade`].
    pub fn record_fault(&mut self, fault: Fault, at: Duration) {
        self.timeline.fault(fault, at);
    }

    /// Record how far the maneuver answering a fault has got.
    ///
    /// Called by the layer driving the wire — the only one that knows whether
    /// a stow reached stow.
    pub fn record_response(&mut self, maneuver: Maneuver, outcome: Outcome, at: Duration) {
        self.timeline.response(maneuver, outcome, at);
    }

    /// When the tick now executing began, on the caller's own epoch.
    ///
    /// Stamped at the top of every tick, so within one this is that tick's own
    /// `now` — the instant a fault raised here is recorded at — and between
    /// ticks it is the previous one's, which is what the move clock advances
    /// from. Zero before the first tick has run.
    fn now(&self) -> Duration {
        self.prev_now.unwrap_or(Duration::ZERO)
    }

    /// Raise `fault`, leaving the machine in whatever state its response has to
    /// be driven from. Records the fault in the timeline.
    ///
    /// Two shapes, chosen by the fault itself. One stops commanding for good:
    /// control is not trusted, so the only thing left is for the caller to cut
    /// torque. The other abandons the move and holds the last goal that went
    /// out — a live state the caller stows from, which is the whole difference
    /// between a machine that cannot be commanded and one that must not be
    /// commanded *further*.
    fn raise(&mut self, fault: Fault, out: &mut TickOutputs) {
        if fault.latches() {
            self.mode = Mode::Faulted(fault);
        } else {
            self.trajectory = None;
            self.mode = Mode::Holding;
            // The wind-down starts measuring from where the machine now
            // stands: the runs open against a goal that is about to stop
            // moving, and carrying them into the stow would spend a window
            // that was already half gone.
            self.tracking.clear();
        }
        out.goal = None;
        out.report.mode = self.mode;
        out.report.fault = Some(fault);
        self.timeline.fault(fault, self.now());
    }

    /// Take `joints` out of service over `fault`, and carry on. Records the
    /// fault in the timeline when joints are newly masked.
    ///
    /// The move keeps running on what remains. Masked joints are commanded
    /// nothing and checked for nothing from here on, including by the raise
    /// checks: entry into the mask is the raise, so a servo already masked
    /// raises nothing however long its error bits stay latched.
    fn degrade(&mut self, fault: Fault, joints: JointSet, out: &mut TickOutputs) {
        for joint in joints.iter() {
            if self.masked.insert(joint) {
                out.report.newly_masked.insert(joint);
            }
            self.tracking.forget(joint);
        }
        out.report.masked = self.masked;
        out.report.degraded = Some(fault);
        // On the mask entry and not on the condition: a group already out of
        // service is a fault already recorded, and a timeline that grew every
        // time a latched servo answered a health poll would bury the story in
        // its own repetitions.
        if !out.report.newly_masked.is_empty() {
            self.timeline.fault(fault, self.now());
        }
    }

    /// Take one servo out of service over `fault`, and abandon the move.
    ///
    /// The mask entry and the torque-off it obliges the caller to write are the
    /// same rule as a degrade; what differs is that a head servo leaving the
    /// group is not something a move carries on through.
    fn mask_and_raise(&mut self, fault: Fault, joint: JointId, out: &mut TickOutputs) {
        if self.masked.insert(joint) {
            out.report.newly_masked.insert(joint);
        }
        out.report.masked = self.masked;
        self.raise(fault, out);
    }

    /// Abandon the running move, and record why.
    ///
    /// The offending sample goes nowhere and the trajectory is dropped, leaving
    /// the machine holding the last goal that was commanded — a live state the
    /// next command drives, which is what lets the caller stow under control.
    fn abort(&mut self, abort: MoveAbort, out: &mut TickOutputs) {
        self.trajectory = None;
        self.mode = Mode::Holding;
        // The open runs go with the move, for the same reason a raise clears
        // them: the maneuver that answers this measures from where the machine
        // now stands, and a run already half spent would fault it early.
        self.tracking.clear();
        out.goal = None;
        out.report.mode = self.mode;
        out.report.aborted = Some(abort);
    }
}

/// What one period of the tracking comparison found.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackingLook {
    /// How far each joint stands from the goal it was last written, radians, in
    /// bus order.
    pub errors: [f64; JointId::COUNT],
    /// The joints whose window ran out on this period without closing.
    pub exhausted: JointSet,
    /// The longest open run afterwards, periods.
    pub longest: u32,
}

/// The tracking comparison, carried across periods.
///
/// One open run per joint, advanced by [`TrackingMonitor::look`] once per live
/// read. It is public because it is the guard: a recorded run replayed through
/// this type is judged by exactly what the tick judges a live one by, so a
/// bound or a threshold that would false-trip a gesture the machine is known to
/// make well can be caught away from the machine.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct TrackingMonitor {
    runs: [Option<TrackingStreak>; JointId::COUNT],
}

impl TrackingMonitor {
    /// A monitor with nothing open.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Forget every open run.
    ///
    /// What a joint was doing against a goal that has been abandoned says
    /// nothing about what it does against the next one.
    pub fn clear(&mut self) {
        self.runs = [None; JointId::COUNT];
    }

    /// Forget one joint's open run, for a joint leaving service.
    pub fn forget(&mut self, joint: JointId) {
        if let Some(row) = joint.index() {
            self.runs[row] = None;
        }
    }

    /// The longest open run across the nine joints, or zero when none is open.
    ///
    /// The single number a report carries about nine independent runs: the one
    /// closest to raising the fault.
    #[must_use]
    pub fn longest(&self) -> u32 {
        self.runs
            .iter()
            .filter_map(|run| run.map(|open| open.count))
            .max()
            .unwrap_or(0)
    }

    /// Advance every joint's run by one live read of `present` against the
    /// `goal` those joints were last written, and say what ran out.
    ///
    /// Only for live reads: a stale reading compared against a fresh goal is a
    /// difference nobody measured, and a caller that skips the period leaves
    /// every run where it stands rather than growing or clearing it. A masked
    /// joint is commanded nothing, so the distance between where it stands and
    /// the last goal it was written measures the machine's own drift and closes
    /// its run.
    pub fn look(
        &mut self,
        cfg: &TrackingFaultConfig,
        masked: JointSet,
        present: &JointVector,
        goal: &JointVector,
    ) -> TrackingLook {
        let mut errors = [0.0; JointId::COUNT];
        let mut exhausted = JointSet::default();
        for (row, ((id, angle), (_, goal))) in
            present.joints().into_iter().zip(goal.joints()).enumerate()
        {
            errors[row] = (angle - goal).abs();
            if masked.contains(id) {
                self.runs[row] = None;
                continue;
            }
            let run = &mut self.runs[row];
            if !outside_limit(errors[row], cfg.threshold_rad) {
                // Within the threshold is healthy, whatever came before it.
                *run = None;
                continue;
            }
            let (count, closing) = match run {
                // Progress is measured from where the joint stands now.
                None => {
                    *run = Some(TrackingStreak {
                        anchor: angle,
                        side: Side::of(goal, angle),
                        count: 1,
                    });
                    (1, false)
                }
                Some(open) => {
                    // Ground covered since the anchor, signed toward the goal:
                    // positive is closing, negative is running away, and a goal
                    // that has arrived at the anchor leaves no direction to
                    // close in. An unplaceable number closes nothing.
                    let side = Side::of(goal, open.anchor);
                    let advance = side.advance(angle - open.anchor);
                    if side.crossed_from(open.side) {
                        // The goal has moved to the far side of the anchor, so
                        // the distance this run was measuring no longer exists
                        // and every step the joint takes toward the new goal
                        // reads as a step away from the old one. The run
                        // restarts where the joint stands: a stalled joint
                        // under a goal that keeps going faults one window
                        // later, and a following joint is never blamed for the
                        // goal turning round under it.
                        open.anchor = angle;
                        open.side = Side::of(goal, angle);
                        open.count = 1;
                        (1, true)
                    } else if advance >= cfg.progress_min_rad {
                        // Sitting behind a moving goal is what a proportional
                        // loop does, not what this fault is for.
                        open.anchor = angle;
                        open.side = Side::of(goal, angle);
                        open.count = 1;
                        (1, true)
                    } else {
                        // A goal that started on the anchor takes its side from
                        // wherever it first lands off it.
                        if open.side == Side::Unplaced {
                            open.side = side;
                        }
                        open.count += 1;
                        (open.count, false)
                    }
                }
            };
            if !closing && count >= cfg.ticks {
                exhausted.insert(id);
            }
        }
        TrackingLook {
            errors,
            exhausted,
            longest: self.longest(),
        }
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

    // The move's own clock, advanced before anything reads it and capped at one
    // nominal period. The cap is the whole point: the loop's lateness lands on
    // when the machine arrives, never on how far one commanded step reaches.
    // Advanced on every tick a move is running — including the ones that go on
    // to fault — because it is the record of how much of the path has been
    // travelled, not of how much of it went well.
    let advance = state
        .prev_now
        .map_or(Duration::ZERO, |prev| inp.now.saturating_sub(prev))
        .min(inp.period);
    state.prev_now = Some(inp.now);
    if let Mode::Moving { elapsed } = &mut state.mode {
        *elapsed = elapsed.saturating_add(advance);
    }

    out.report.mode = state.mode;
    out.report.present_min_margin = state.present_min_margin;
    out.report.masked = state.masked;

    // A fault is absorbing. Commands are ignored and nothing is emitted, and
    // that is the whole of what this layer can do about one: it holds no wire,
    // so torque is left exactly where the caller had it, and bringing the
    // machine to the minimum risk condition — torque off — is the caller's job.
    //
    // There is no clearing it, by design. A state is built by arming and dies
    // with the engagement, so recovery is either the next wake building a fresh
    // one — which is what every rest-disposition response leaves the machine
    // ready for — or a person, for the ones that park. Nothing resumes
    // commanding on the state that stopped.
    //
    // Recorded once, at the raise: the standing fault is repeated in every
    // report from here on, and repeating it in the timeline would grow the
    // record at tick rate.
    if let Mode::Faulted(fault) = state.mode {
        out.report.fault = Some(fault);
        return;
    }

    // A live read gives the head pose, and with it the clearance baseline
    // every envelope check on this tick uses. A read carrying a number nobody
    // can place is not one: it is a corrupt frame, discarded and counted as a
    // read that did not arrive, because a single bad frame says nothing about
    // the machine and everything about the layer that produced it. What says
    // something is a run of them, and that is what `read_loss_ticks` bounds.
    let arrived = inp
        .present
        .filter(|present| present.first_non_finite().is_none());
    let fresh = match arrived {
        Some(present) => {
            state.miss_count = 0;
            let mut pose = Isometry3::identity();
            match forward_kinematics(
                &cfg.geom,
                &LegAngles(present.legs),
                &state.fk_seed,
                &cfg.fk,
                &mut pose,
            ) {
                Ok(stats) => {
                    state.pose_failures = 0;
                    out.report.present_fresh = true;
                    out.report.fk = Some(stats);
                    state.fk_seed = pose;
                    state.present_min_margin = min_pose_margin(&cfg.geom, &pose);
                    out.report.present_min_margin = state.present_min_margin;
                    Some(present)
                }
                // Angles that close no loop are angles the control path cannot
                // use, so this frame updates nothing and is skipped exactly as
                // a missing one is. Its own run, though: silence is a bus
                // going, and live-but-unsolvable is a mechanism outside the
                // model it is commanded through — different causes, different
                // faults, and neither may hide behind the other.
                Err(source) => {
                    state.pose_failures += 1;
                    out.report.pose_failures = state.pose_failures;
                    if state.pose_failures > cfg.read_loss_ticks {
                        state.raise(
                            Fault::MeasuredPoseInvalid {
                                failures: state.pose_failures,
                                source,
                            },
                            out,
                        );
                        return;
                    }
                    None
                }
            }
        }
        None => {
            state.miss_count += 1;
            out.report.misses = state.miss_count;
            if state.miss_count > cfg.read_loss_ticks {
                state.raise(
                    Fault::PositionFeedbackLost {
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
    out.report.pose_failures = state.pose_failures;
    out.report.tracking_count = state.tracking.longest();

    // Tracking, on live reads only: a stale reading compared against a fresh
    // goal is a difference nobody measured, so a stale tick freezes every
    // joint's run where it stands rather than growing or clearing it.
    if let Some(present) = fresh {
        let look = state
            .tracking
            .look(&cfg.tracking, state.masked, present, &state.last_goal);
        out.report.tracking_errors = Some(look.errors);
        // Recorded before the fault check: the tick that runs a window out is
        // the one whose figure matters most, and a report that shipped zero
        // there would read as a single-tick trip rather than a sustained one.
        out.report.tracking_count = look.longest;

        // The rows whose window ran out, by how far out they are, kept per
        // group because the two groups are answered differently. Every other
        // row holds a value no measurement can beat, so the same worst-of sweep
        // names the joint among them.
        let mut head_out = [f64::NEG_INFINITY; JointId::COUNT];
        let mut antennas_out = [f64::NEG_INFINITY; JointId::COUNT];
        let mut head_exhausted = false;
        let mut antennas_exhausted = false;
        for (row, id) in JointId::ALL.into_iter().enumerate() {
            if !look.exhausted.contains(id) {
                continue;
            }
            if id.group() == JointGroup::Antennas {
                antennas_out[row] = look.errors[row];
                antennas_exhausted = true;
            } else {
                head_out[row] = look.errors[row];
                head_exhausted = true;
            }
        }
        // The head decides the tick when both groups run out together: its
        // answer winds the whole machine down, which subsumes taking the
        // antennas out of service.
        if head_exhausted {
            let (joint, error) = worst_joint(&head_out);
            state.raise(Fault::HeadObstructed { joint, error }, out);
            return;
        }
        if antennas_exhausted {
            let (joint, error) = worst_joint(&antennas_out);
            state.degrade(
                Fault::AntennaObstructed { joint, error },
                JointGroup::Antennas.joints(),
                out,
            );
        }
    }

    // Health, when the slower poll ran. Reported in full either way; the
    // input-voltage bit alone raises nothing and is never filtered out.
    //
    // Every unmasked servo is examined, not just the first unhealthy one in bus
    // order: hardware error bits latch in the servo, so a masked servo keeps
    // flagging on every poll for the rest of the session, and a sweep that
    // stopped at it would never see the second servo to go.
    if let Some(health) = inp.health {
        out.report.health = Some(*health);
        let mut head_bad = None;
        let mut antenna_bad = None;
        for (row, servo) in health.iter().enumerate() {
            let joint = JointId::ALL[row];
            if state.masked.contains(joint) || servo.healthy_or_voltage_only() {
                continue;
            }
            let worst = if joint.group() == JointGroup::Antennas {
                &mut antenna_bad
            } else {
                &mut head_bad
            };
            worst.get_or_insert((joint, *servo));
        }
        if let Some((joint, servo)) = head_bad {
            state.mask_and_raise(
                Fault::HeadServoFault {
                    joint,
                    id: servo.id,
                    bits: servo.bits,
                },
                joint,
                out,
            );
            return;
        }
        if let Some((joint, servo)) = antenna_bad {
            state.degrade(
                Fault::AntennaServoFault {
                    joint,
                    id: servo.id,
                    bits: servo.bits,
                },
                JointGroup::Antennas.joints(),
                out,
            );
        }
    }

    // At most one command. Refusals report and change nothing.
    if let Some(command) = inp.command {
        out.report.command = take_command(cfg, state, command);
    }
    out.report.mode = state.mode;

    // Sample the active trajectory at the move's own elapsed time. Everything is
    // copied out of the trajectory here so the borrow ends before anything can
    // fault.
    let Mode::Moving { elapsed: t } = state.mode else {
        return;
    };
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
        state.abort(MoveAbort::EnvelopePath(envelope.violations), out);
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
    // wrong. What the bound bounds is therefore the plan, which is why passing
    // it kills the move and not the machine.
    //
    // A masked joint is skipped: nothing of what the planner produced for it
    // goes anywhere, so there is no step to bound. The plan itself is left
    // whole — the mask decides what reaches the wire, not what the trajectory
    // says.
    let mut changed = false;
    for ((id, angle), (_, last)) in candidate.joints().into_iter().zip(state.last_goal.joints()) {
        if state.masked.contains(id) {
            continue;
        }
        let delta = (angle - last).abs();
        if outside_limit(delta, cfg.max_step.for_joint(id)) {
            state.abort(MoveAbort::StepTooLarge { joint: id, delta }, out);
            return;
        }
        changed |= angle != last;
    }

    // Emit, but only what changed: holding writes nothing and the servos hold.
    // A period whose only movement is on a masked joint has nothing to say.
    out.report.emitted = changed;
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
            // At zero, so the accepting tick samples the move's own start; a
            // replacement's clock starts there too, from the setpoint the last
            // tick commanded.
            state.mode = Mode::Moving {
                elapsed: Duration::ZERO,
            };
            if retarget {
                CommandDisposition::Retargeted
            } else {
                CommandDisposition::Started
            }
        }
        Err(error) => CommandDisposition::Rejected(CommandRejection::Trajectory(error)),
    }
}

/// The headroom a clock carries past the ratio its dry pass measured, as a
/// multiple of one over the square of the periods the clock spans.
///
/// What the dry pass measures is the largest step between two points of *one*
/// sampling grid, and that is a little under the largest step of one period's
/// width the path holds anywhere: the peak of a min-jerk rate falls between two
/// samples, and the grid reads it low by the curvature there. The deficit is
/// second order in the grid spacing — `f'` loses fifteen times the square of
/// the distance from its peak, which is at most half a period — so it is
/// fifteen quarters over 1.875 of one over the periods squared, and this
/// numerator is that with a factor of two over it.
///
/// Two things need exactly that headroom. A loop running late resumes its
/// periods at a shifted phase, and a shifted grid can land nearer the peak than
/// the measured one did — so a clock accepted at the measured maximum alone,
/// whether it was stretched to there or asked for there, leaves the guard to
/// fault on a healthy move at an unlucky wake phase. And the
/// stretch iteration itself does not otherwise finish: scaling a clock by the
/// measured ratio refines its grid in the same motion, so each pass removes
/// about fifteen sixteenths of the excess and the sequence approaches the bound
/// from above without reaching it — four exact passes over the antenna sweep
/// still leave a step five parts in ten million over.
///
/// A term of this shape is the opposite of a jitter allowance: it does not grow
/// with how late a loop runs, it grows with how coarsely the path is sampled,
/// and it vanishes as the clock lengthens. On the fold's floor it is three
/// parts in a thousand — a few milliseconds nobody can see.
const STRETCH_GRID_HEADROOM: f64 = 4.0;

/// The most of it any one clock may take, for a clock spanning so few periods
/// that the second-order argument above says nothing useful. A move of under
/// four periods is not something a longer clock is going to characterise; the
/// re-measuring passes and the guard are what stand behind it.
const MAX_GRID_HEADROOM: f64 = 0.25;

/// The headroom a clock spanning `periods` periods needs over the step its dry
/// pass measured.
///
/// The one place the term is computed, so what a clock is judged against and
/// what a stretch lands on are the same number: a clock accepted with less
/// headroom than a stretch would have given it is a clock the live grid's own
/// phase can walk past the guard.
fn grid_headroom(periods: f64) -> f64 {
    if periods > 0.0 {
        (STRETCH_GRID_HEADROOM / (periods * periods)).min(MAX_GRID_HEADROOM)
    } else {
        MAX_GRID_HEADROOM
    }
}

/// Whether a clock of `duration` carries a path whose dry pass measured
/// `ratio` of the bound, with the grid headroom its own length calls for.
fn carries(ratio: f64, duration: Duration, tick_hz: f64) -> bool {
    let periods = duration.as_secs_f64() * tick_hz;
    ratio * (1.0 + grid_headroom(periods)) <= 1.0
}

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

/// The most a de-phasing stretch may lengthen one antenna's clock, as a
/// multiple of what was asked for.
///
/// A side held four times as long as it was commanded is not a stagger any
/// more, and the geometry that asks for it is one no clock separates: a leader
/// creeping through the crossing leaves the follower nothing to be late for.
/// The move runs on the last clocks and says the separation is unmet, which is
/// the one thing this must never turn into a refusal — the maneuvers that
/// recover this machine go down the same path.
const MAX_PHASE_STRETCH: f64 = 4.0;

/// How far past the separation a de-phasing stretch aims, as a fraction of it.
///
/// Each pass estimates the delay the pair still needs from the rate the leader
/// is travelling at, and the leader is shaped, so it is slowing: the pass
/// closes most of the remaining gap and the sequence approaches the separation
/// from below without arriving at it — four passes over a floored pair landed
/// four parts in ten million short. Aiming a twentieth past it is what makes a
/// pass land, and a twentieth of the separation is a few hundredths of a second
/// on the clock it comes out of.
const PHASE_TARGET_OVERSHOOT: f64 = 1.05;

/// A move's clock as it was asked for and as it will run, and what the antenna
/// pair's phase came to.
///
/// Reported whenever there is something to say: a clock this pass lengthened, a
/// pair it de-phased, or a pair whose separation it could not reach.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ClockStretch {
    /// The durations the caller commanded.
    pub requested: MoveDurations,
    /// The durations the move runs on instead: per clock, the longer of what
    /// was asked for and what carries that clock's own span inside the per-tick
    /// step bounds. Nothing sits between the two — a clock at its floor is a
    /// clock that runs.
    pub effective: MoveDurations,
    /// How far from mirrored the antennas stand when the second of them reaches
    /// the contact band, on the effective clocks. `None` for a move that does
    /// not carry both tips across the band's edge.
    pub separation: Option<PhaseSeparation>,
    /// How far apart the pass was holding that pair to, radians — the
    /// configured [`AntennaPhaseConfig::separation_rad`]. Carried here so a
    /// report of the measurement carries what it was judged against.
    pub separation_required: f64,
    /// Whether an antenna clock here was lengthened to de-phase the pair, as
    /// against to fit its own span inside the step bounds.
    pub dephased: bool,
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
    /// Whether every clock carries its span, headroom included.
    ///
    /// The same test a stretch is sized to satisfy. The dry pass walks one
    /// phase of the sampling grid and the live loop walks whichever phase it
    /// wakes on, so a clock is accepted only with the room its own length says
    /// the other phases need.
    fn fit(self, durations: MoveDurations, tick_hz: f64) -> bool {
        carries(self.head, durations.head, tick_hz)
            && carries(self.antennas[0], durations.antennas[0], tick_hz)
            && carries(self.antennas[1], durations.antennas[1], tick_hz)
    }
}

/// `command` on clocks long enough to carry the span it actually covers within
/// the per-tick step bounds and to part the antennas at their crossing, and
/// what the pass did when it did anything.
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
/// The second thing a clock has to carry is the pair's phase. Two antennas
/// sweeping inboard on clocks near enough alike reach the point their arcs
/// cross together and their tips meet there — the one collision on record. So
/// the sampled pair is measured at the contact band on the clocks the move will
/// run on, floors included, and a pair that arrives too near mirrored has the
/// later side delayed until they clear each other. A separation no delay
/// reaches is reported and run anyway.
///
/// Only ever longer, and only where a fixed clock was never sized: a feasible
/// move comes back untouched with `None`, so the configured durations remain
/// the policy for every move they are feasible for. A command this cannot
/// measure — a move that cannot be shaped, a path some sample has no inverse
/// kinematics for, a step bound of zero — also comes back untouched. Nothing
/// here refuses anything, phase geometry included: the maneuvers that recover
/// this machine come down this path, and a stow must always be commandable.
/// [`take_command`] still judges the command it is handed.
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
    // The spans first. A clock that cannot carry its own path is not a clock
    // whose phase means anything, and the fit reached here survives what comes
    // next: the worst step shrinks monotonically as a clock lengthens, and
    // de-phasing only ever lengthens one.
    let mut measured = false;
    for _ in 0..STRETCH_PASSES {
        let Some(ratios) = worst_step_ratios(cfg, start, &resolved, effective, *warp, tick_hz)
        else {
            break;
        };
        measured = true;
        if ratios.fit(effective, tick_hz) {
            break;
        }
        let Some(stretched) = stretched(effective, ratios, tick_hz) else {
            break;
        };
        effective = stretched;
    }
    if !measured {
        // Nothing about this command could be measured on the clocks it came
        // with, so nothing about it is this pass's to change. The phase walk
        // would still have an answer — it solves no legs, so a path nobody can
        // place does not stop it — and a clock lengthened on that answer would
        // be a stretch reported for a move the tick is about to refuse for a
        // reason no clock addresses.
        return (*command, None);
    }

    // Then the pair's phase, on the clocks the move will run on — measured
    // again after every adjustment, so what is reported is what runs.
    let sized = effective;
    let mut separation;
    let mut dephased = false;
    let mut adjustments = 0;
    loop {
        separation = phase_of(cfg, start, &resolved, effective, *warp, tick_hz);
        let Some(pair) = separation.filter(|pair| !pair.met(cfg.phase.separation_rad)) else {
            break;
        };
        if adjustments == STRETCH_PASSES {
            break;
        }
        let Some(next) = de_phased(cfg, sized, effective, &pair) else {
            break;
        };
        effective = next;
        adjustments += 1;
        dephased = true;
    }

    if effective == requested && separation.is_none_or(|pair| pair.met(cfg.phase.separation_rad)) {
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
            separation,
            separation_required: cfg.phase.separation_rad,
            dephased,
        }),
    )
}

/// `effective` with the later-arriving antenna's clock lengthened enough to
/// carry the pair to the configured separation, or `None` when no clock gets
/// there.
///
/// Delaying the side that reaches the crossing second is the whole mechanism:
/// the other one is still travelling away from the crossing, so every extra
/// second of delay is `leader_rate` more radians between the tips, and the
/// estimate is that division. It is first-order — the leader is shaped and its
/// rate is falling — and the re-measuring passes above are what close the rest,
/// exactly as they do for a step-bound stretch.
///
/// `None` for the three cases no delay answers: a leader that has stopped, a
/// crossing at the very start of the path, and a side already lengthened to
/// [`MAX_PHASE_STRETCH`]. Each of them leaves the move on the clocks it has,
/// saying the separation is unmet.
///
/// The cap is a multiple of `sized` — the clock the side runs on with its own
/// span carried, before any de-phasing — and not of whatever was asked for. A
/// caller who asks for a clock far under the floor has already had it replaced;
/// capping against the number they typed would cap a side below the span it has
/// to cover.
fn de_phased(
    cfg: &MotionConfig,
    sized: MoveDurations,
    effective: MoveDurations,
    pair: &PhaseSeparation,
) -> Option<MoveDurations> {
    let side = match pair.later {
        JointId::AntennaRight => 0,
        JointId::AntennaLeft => 1,
        JointId::BodyYaw | JointId::Leg(_) => return None,
    };
    let arrival = pair.at.as_secs_f64();
    if !(pair.leader_rate > 0.0 && arrival > 0.0) {
        return None;
    }
    // Scaling a clock scales every time along its path, the crossing included,
    // so the delay is asked for as a factor rather than added on.
    let target = cfg.phase.separation_rad * PHASE_TARGET_OVERSHOOT;
    let delay = (target - pair.offset) / pair.leader_rate;
    let scaled = effective.antennas[side].as_secs_f64() * (arrival + delay) / arrival;
    let capped = scaled.min(sized.antennas[side].as_secs_f64() * MAX_PHASE_STRETCH);
    let longer = Duration::try_from_secs_f64(capped).ok()?;
    (longer > effective.antennas[side]).then(|| {
        let mut durations = effective;
        durations.antennas[side] = longer;
        durations
    })
}

/// `durations` with each clock that overran its bound scaled past it, and the
/// clocks that did not left alone. `None` when the arithmetic leaves the range a
/// duration can hold.
fn stretched(durations: MoveDurations, ratios: StepRatios, tick_hz: f64) -> Option<MoveDurations> {
    Some(MoveDurations {
        head: scale_past(durations.head, ratios.head, tick_hz)?,
        antennas: [
            scale_past(durations.antennas[0], ratios.antennas[0], tick_hz)?,
            scale_past(durations.antennas[1], ratios.antennas[1], tick_hz)?,
        ],
    })
}

/// `duration` scaled past `ratio` by [`STRETCH_GRID_HEADROOM`], or itself when
/// it already carries its span with that headroom. Never shortened.
///
/// The scaling is first-order exact for a joint the path moves linearly in its
/// own coordinate — min-jerk is time-scale invariant, so twice the clock is
/// half the per-tick step — and the term it is not exact for, the inverse
/// kinematics the legs go through, is what the re-measuring passes above are
/// for. The headroom is the sampling grid's own error and nothing else; no
/// allowance is made for how the live loop is paced, because the loop advances
/// a move by one period of its clock however late it wakes.
///
/// A clock inside the bound on the grid the dry pass walked but short of the
/// headroom is stretched by the headroom alone: its ratio is already one or
/// under, and what it lacks is the room the other phases of the grid need.
fn scale_past(duration: Duration, ratio: f64, tick_hz: f64) -> Option<Duration> {
    if carries(ratio, duration, tick_hz) {
        return Some(duration);
    }
    let scaled = duration.as_secs_f64() * ratio.max(1.0);
    let periods = scaled * tick_hz;
    Duration::try_from_secs_f64(scaled * (1.0 + grid_headroom(periods))).ok()
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
    let peaks = peaks_of(cfg, start, target, durations, warp, tick_hz)?;
    // Per joint and not per group: the legs and the body yaw ride one clock and
    // are judged against bounds of their own, so the clock's ratio is the worse
    // of the two.
    let ratio = |step: f64, bound: f64| {
        let ratio = step / bound;
        ratio.is_finite().then_some(ratio)
    };
    Some(StepRatios {
        head: ratio(peaks.legs, cfg.max_step.legs)?
            .max(ratio(peaks.body_yaw, cfg.max_step.body_yaw)?),
        antennas: [
            ratio(peaks.antennas[0], cfg.max_step.antennas)?,
            ratio(peaks.antennas[1], cfg.max_step.antennas)?,
        ],
    })
}

/// The worst per-tick step a move plans, per joint group, radians.
///
/// What a step bound is sized against: the largest distance one period of the
/// plan asks a joint to cover, which is the peak of the shaped path and nothing
/// to do with how the loop is paced.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DryPassPeaks {
    /// The worst step any of the six cranks takes.
    pub legs: f64,
    /// The body yaw's worst step.
    pub body_yaw: f64,
    /// Each antenna's worst step: right, then left.
    pub antennas: [f64; 2],
}

/// The largest per-tick step `command` would plan from `start`, sampled at
/// `tick_hz` through the same envelope and inverse kinematics the live tick
/// runs.
///
/// The measurement [`floor_move_clock`] decides on, handed back in radians
/// rather than as a fraction of a bound. What it is for is sizing the bounds
/// themselves: a recorded run says what the machine did, and this says what the
/// planner asked of it, which is the series `max_step` has to admit. `None`
/// for a command with no clock, and for every path the dry pass cannot measure.
#[must_use]
pub fn dry_pass_peaks(
    cfg: &MotionConfig,
    start: &JointTargets,
    command: &MotionCommand,
    tick_hz: f64,
) -> Option<DryPassPeaks> {
    let MotionCommand::MoveTo {
        target,
        durations,
        warp,
    } = command
    else {
        return None;
    };
    let resolved = resolve_antennas(start, target).ok()?;
    peaks_of(cfg, start, &resolved, *durations, *warp, tick_hz)
}

/// How far from mirrored `command` plans to leave the antennas when the second
/// of them reaches the contact band, sampled at `tick_hz`.
///
/// The measurement [`floor_move_clock`] holds a pair to, handed back on its own
/// so a caller can ask what a command's phase comes to without commanding it —
/// which is how the recorded runs and the shipped configuration are compared
/// against the same figure. `None` for a command with no clock, for a path that
/// cannot be shaped or walked, and for one that does not carry both tips across
/// the band's edge.
#[must_use]
pub fn dry_pass_separation(
    cfg: &MotionConfig,
    start: &JointTargets,
    command: &MotionCommand,
    tick_hz: f64,
) -> Option<PhaseSeparation> {
    let MotionCommand::MoveTo {
        target,
        durations,
        warp,
    } = command
    else {
        return None;
    };
    let resolved = resolve_antennas(start, target).ok()?;
    phase_of(cfg, start, &resolved, *durations, *warp, tick_hz)
}

/// One walk of the antennas' planned path from `start` to an already-resolved
/// `target`, for the pair's phase at the contact band.
///
/// The legs are not solved for. This walk runs once per de-phasing pass on the
/// period that accepts a command, and what it needs is two scalars per sample;
/// putting the inverse kinematics through that many more passes to reach them
/// would cost the accepting period a great deal for nothing.
///
/// Two things keep it to the question it asks. A side that is not going
/// anywhere cannot cross the band's edge, so a command that moves neither
/// antenna or only one of them is answered without a walk at all. And the walk
/// spans the antenna clocks rather than the move's — a pair settled by period
/// fifteen says nothing further over the remaining eighty-five of a calm stow.
fn phase_of(
    cfg: &MotionConfig,
    start: &JointTargets,
    target: &JointTargets,
    durations: MoveDurations,
    warp: Warp,
    tick_hz: f64,
) -> Option<PhaseSeparation> {
    if start.antennas[0] == target.antennas[0] || start.antennas[1] == target.antennas[1] {
        return None;
    }
    let span = durations.antennas[0].max(durations.antennas[1]);
    let samples = dry_samples(span, tick_hz)?;
    let trajectory = Trajectory::new(start, target, durations, warp).ok()?;
    let mut sampled = JointTargets::default();
    // Seeded with the pose the machine already holds, so a pair that starts
    // inside the contact band is read as standing there rather than as arriving.
    let mut watch = PhaseWatch::new(cfg.phase.contact_band_rad);
    watch.look(Duration::ZERO, start.antennas);
    for step in 1..=samples {
        let t = Duration::try_from_secs_f64(f64::from(step) / tick_hz).ok()?;
        trajectory.sample(t, &mut sampled);
        watch.look(t, sampled.antennas);
    }
    watch.separation()
}

/// How many periods a dry walk over `span` takes, or `None` for a tick rate or
/// a clock there is nothing to measure about.
///
/// The grid the live loop samples on, from the first period that commands
/// anything through the one that lands the endpoint. The move's own start is
/// the pose already held and is the baseline rather than a step. `span` is
/// whichever of the move's clocks the walk has a question about: every one of
/// them for the step bounds, the two antennas for the pair's phase.
fn dry_samples(span: Duration, tick_hz: f64) -> Option<u32> {
    if !(tick_hz.is_finite() && tick_hz > 0.0) {
        return None;
    }
    let samples = (span.as_secs_f64() * tick_hz).ceil();
    if !samples.is_finite() || samples > f64::from(MAX_DRY_SAMPLES) {
        return None;
    }
    #[expect(
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss,
        reason = "bounded above by MAX_DRY_SAMPLES and below by the ceil of a non-negative span"
    )]
    Some(samples as u32)
}

/// [`dry_pass_peaks`] over an already-resolved target.
fn peaks_of(
    cfg: &MotionConfig,
    start: &JointTargets,
    target: &JointTargets,
    durations: MoveDurations,
    warp: Warp,
    tick_hz: f64,
) -> Option<DryPassPeaks> {
    let samples = dry_samples(durations.longest(), tick_hz)?;
    let trajectory = Trajectory::new(start, target, durations, warp).ok()?;

    let mut previous = joints_of(cfg, start)?;
    let mut sampled = JointTargets::default();
    let mut peaks = DryPassPeaks::default();
    for step in 1..=samples {
        let t = Duration::try_from_secs_f64(f64::from(step) / tick_hz).ok()?;
        trajectory.sample(t, &mut sampled);
        let candidate = joints_of(cfg, &sampled)?;
        for ((id, angle), (_, last)) in candidate.joints().into_iter().zip(previous.joints()) {
            let step = (angle - last).abs();
            if !step.is_finite() {
                return None;
            }
            // The six cranks are one figure and the yaw another, because they
            // are bounded separately even though one clock drives them both.
            let peak = match id {
                JointId::AntennaRight => &mut peaks.antennas[0],
                JointId::AntennaLeft => &mut peaks.antennas[1],
                JointId::BodyYaw => &mut peaks.body_yaw,
                JointId::Leg(_) => &mut peaks.legs,
            };
            *peak = peak.max(step);
        }
        previous = candidate;
    }
    Some(peaks)
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
    use crate::seq::{RegId, SeqStep, StepContext};
    use reachy_kin::{inverse_kinematics, rest_head_pose};

    fn secs(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    /// The control period every test here drives, and the one the floors are
    /// derived at.
    const PERIOD: Duration = Duration::from_millis(20);

    /// The grid the head group's floor is searched on, seconds: the derived
    /// number is the largest candidate on it that carries the fold, so the
    /// exact threshold lies within one step below.
    const FLOOR_SEARCH_STEP_S: f64 = 0.01;

    /// How far from the clock its span calls for an exactly stretched clock may
    /// land.
    ///
    /// The slack a stretch carries is the grid headroom, which is arithmetic
    /// and is accounted for by [`floored_clock`] rather than tolerated here.
    /// What is left is the sampling itself: the dry pass measures the largest
    /// step between two sample points, and a min-jerk peak that falls between
    /// them reads a fraction of a percent low. A clock landing a whole percent
    /// off is something other than sampling.
    const GRID_TOLERANCE: f64 = 0.01;

    /// Whether an effective clock landed on `wanted` — the larger of what was
    /// asked for and the closed form its span needs — with no band between the
    /// two.
    ///
    /// Never shortened, and over by no more than the grid headroom that clock
    /// carries. The headroom is the ceiling because it is also the size of the
    /// dry pass's own underestimate of the peak: a stretch starting from a very
    /// short clock measures a ratio a little low, so it converges to within one
    /// headroom of the floor rather than onto it. Both terms fall off as the
    /// square of the periods the clock spans, so a long clock is held to the
    /// sampling tolerance alone.
    fn lands_on(effective: f64, wanted: f64, tick_hz: f64) -> bool {
        let allowance = 2.0 * grid_headroom(wanted * tick_hz) + GRID_TOLERANCE;
        effective >= wanted * (1.0 - GRID_TOLERANCE) && effective <= wanted * (1.0 + allowance)
    }

    /// The period the tests grid on is the rate the floors are derived at, so
    /// what they measure is what a shipped configuration runs.
    #[test]
    fn the_test_period_is_the_floor_tick_rate() {
        assert_eq!(PERIOD.as_secs_f64(), 1.0 / FLOOR_TICK_HZ);
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
        (
            MotionState::new_armed(&record, JointSet::EMPTY),
            outcome.pinned,
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
                period: PERIOD,
                present: Some(present),
                command,
                health: None,
            },
            &mut out,
        );
        out
    }

    /// One tick with a live read and a health sweep.
    fn tick_with_health(
        cfg: &MotionConfig,
        state: &mut MotionState,
        now: Duration,
        present: &JointVector,
        health: &[ServoHealth; JointId::COUNT],
        command: Option<&MotionCommand>,
    ) -> TickOutputs {
        let mut out = TickOutputs::default();
        motion_tick(
            cfg,
            state,
            &TickInputs {
                now,
                period: PERIOD,
                present: Some(present),
                command,
                health: Some(health),
            },
            &mut out,
        );
        out
    }

    /// Nine healthy servos, numbered from ten as the bench's roster is.
    fn healthy_servos() -> [ServoHealth; JointId::COUNT] {
        let mut health = [ServoHealth::default(); JointId::COUNT];
        for (slot, id) in health.iter_mut().zip(10u8..) {
            slot.id = id;
        }
        health
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
            if out.report.completed || out.report.fault.is_some() || out.report.aborted.is_some() {
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
        let step = FLOOR_SEARCH_STEP_S;
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
            matches!(out.report.aborted, Some(MoveAbort::StepTooLarge { .. })),
            "under the floor: {:?}",
            out.report.aborted
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
    /// on a clock at the floor, and that clock carries the move through.
    ///
    /// The requested clock is the one the derivation test pins as faulting, so
    /// the two halves are the same move: the original clock faults partway
    /// through; the stretched one completes.
    ///
    /// "At the floor" is the exact threshold, which sits inside the last
    /// hundredth of a second the derivation's search grid stepped in:
    /// [`HEAD_GROUP_FLOOR_S`] is the largest hundredth that carries the fold,
    /// and the true threshold is somewhere in the hundredth below it.
    #[test]
    fn a_move_under_the_head_floor_is_stretched_past_it() {
        let cfg = MotionConfig::default();
        let stow = JointTargets {
            head_pose_body: reachy_kin::stow_head_pose(),
            ..JointTargets::default()
        };
        let requested = secs(0.2);
        let command = MotionCommand::MoveTo {
            target: JointTargets::default(),
            durations: MoveDurations::uniform(requested),
            warp: Warp::MinJerk,
        };

        let (mut state, pinned) = armed_at(&cfg, &stow);
        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("a fifth of a second cannot carry the fold");
        assert_eq!(stretch.requested, MoveDurations::uniform(requested));
        assert_eq!(stretch.effective, durations_of(&floored));
        assert!(
            stretch.effective.head.as_secs_f64() > HEAD_GROUP_FLOOR_S - FLOOR_SEARCH_STEP_S
                && stretch.effective.head.as_secs_f64() < HEAD_GROUP_FLOOR_S + FLOOR_SEARCH_STEP_S,
            "stretched to {:?}, not the {HEAD_GROUP_FLOOR_S} s floor",
            stretch.effective.head,
        );

        let (_, out) = run_command(&cfg, &mut state, &pinned, &floored, 1.0 / FLOOR_TICK_HZ);
        assert_eq!(out.report.fault, None);
        assert!(out.report.completed, "{:?}", out.report);

        // And the clock it was asked for is the one the guard stops, so the
        // stretch is what stands between this move and an abandoned one.
        let (mut state, pinned) = armed_at(&cfg, &stow);
        let (_, out) = run_command(&cfg, &mut state, &pinned, &command, 1.0 / FLOOR_TICK_HZ);
        assert!(
            matches!(out.report.aborted, Some(MoveAbort::StepTooLarge { .. })),
            "{:?}",
            out.report.aborted
        );
    }

    /// The body yaw and the antennas move linearly in their own coordinates, so
    /// what a stretch lands on is [`duration_floor_s`] of the span it covers,
    /// to within the grid tolerance and from either side.
    ///
    /// Three independent clocks, checked one at a time: a yaw sweep leaves both
    /// antenna clocks alone, and one antenna's sweep leaves the head clock and
    /// the other antenna's alone, which is the whole point of the split.
    #[test]
    fn the_yaw_and_antenna_stretches_land_on_their_closed_forms() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &JointTargets::default());
        let requested = MoveDurations {
            head: secs(0.2),
            antennas: [secs(0.1), secs(0.15)],
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
                lands_on(moved.as_secs_f64(), floor, FLOOR_TICK_HZ),
                "{name}: stretched to {moved:?}, not the {floor:.4} s its span needs"
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
            antennas: [secs(0.1), secs(0.05)],
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
    /// instead of faulting partway and dropping the head. It is the
    /// unattended-at-boot case: nobody is there to catch the head or to restart
    /// the daemon.
    ///
    /// Half a turn needs 0.79 s at the measured yaw bound, which a calm
    /// two-second stow carries outright — asserted here, because it is why the
    /// fold below is driven at a quick gesture's clock instead: that is the
    /// clock a half turn does not fit inside.
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
        let calm = MotionCommand::MoveTo {
            target: stow,
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };
        let (state, _) = armed_at(&cfg, &crooked);
        assert_eq!(
            floor_move_clock(&cfg, state.last_targets(), &calm, FLOOR_TICK_HZ).1,
            None,
            "a calm stow carries the half turn as asked"
        );

        let quick = secs(0.5);
        let command = MotionCommand::MoveTo {
            target: stow,
            durations: MoveDurations::uniform(quick),
            warp: Warp::MinJerk,
        };

        // Unfloored, the quick fold is the regression: it steps past the
        // bound partway round and the move is abandoned wherever that was.
        let (mut state, pinned) = armed_at(&cfg, &crooked);
        let (_, out) = run_command(&cfg, &mut state, &pinned, &command, 1.0 / FLOOR_TICK_HZ);
        assert!(
            matches!(
                out.report.aborted,
                Some(MoveAbort::StepTooLarge {
                    joint: JointId::BodyYaw,
                    ..
                })
            ),
            "{:?}",
            out.report.aborted
        );

        let (mut state, pinned) = armed_at(&cfg, &crooked);
        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("a half turn does not fit a half second clock");
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

    /// Every goal a move emits, driven at the times `at` hands out, with a
    /// perfectly tracking machine: every period's present positions are the
    /// previous period's goals.
    ///
    /// Panics on a fault, because a run that stops early is not a goal sequence
    /// any caller of this can compare against another.
    fn goals_driven_at(
        cfg: &MotionConfig,
        start: &JointTargets,
        command: &MotionCommand,
        at: &dyn Fn(u32) -> Duration,
    ) -> (JointVector, Vec<JointVector>) {
        let (mut state, pinned) = armed_at(cfg, start);
        let mut present = pinned;
        let mut goals = Vec::new();
        for n in 0..4000 {
            let command = (n == 0).then_some(command);
            let out = tick_with(cfg, &mut state, at(n), &present, command);
            assert_eq!(out.report.fault, None, "tick {n}");
            assert_eq!(out.report.aborted, None, "tick {n}");
            if let Some(goal) = out.goal {
                present = goal;
                goals.push(goal);
            }
            if out.report.completed {
                return (pinned, goals);
            }
        }
        panic!("the move never completed");
    }

    /// The largest single-period step in a goal sequence, over all nine joints.
    fn worst_step_of(start: &JointVector, goals: &[JointVector]) -> f64 {
        let mut worst: f64 = 0.0;
        let mut previous = *start;
        for goal in goals {
            for ((_, angle), (_, last)) in goal.joints().into_iter().zip(previous.joints()) {
                worst = worst.max((angle - last).abs());
            }
            previous = *goal;
        }
        worst
    }

    /// A loop running late emits the steps of a loop running on time; it takes
    /// longer, and that is all.
    ///
    /// Injects the worst alternation of the lateness the control loop treats as
    /// ordinary — half a period on every second period — over a move whose
    /// clock is at its floor, which is where an inflated step would go straight
    /// past the guard. Wall-clock sampling would have stepped half again as far
    /// on every late period. What is left is the sampling grid landing at a
    /// different phase, which is worth parts in a thousand and is what the
    /// stretch's headroom is sized for; the binding assertion is that the steps
    /// stay inside the bound the guard enforces.
    #[test]
    fn a_loop_running_late_steps_no_further_than_one_running_on_time() {
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

        let (state, _) = armed_at(&cfg, &crooked);
        let (floored, _) = floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);

        let (pinned, on_time) = goals_driven_at(&cfg, &crooked, &floored, &|n| PERIOD * n);
        let (_, jittery) = goals_driven_at(&cfg, &crooked, &floored, &|n| {
            PERIOD * n
                + if n % 2 == 1 {
                    PERIOD / 2
                } else {
                    Duration::ZERO
                }
        });

        let peak = worst_step_of(&pinned, &on_time);
        let late_peak = worst_step_of(&pinned, &jittery);
        assert!(
            late_peak <= cfg.max_step.body_yaw,
            "the late loop stepped {late_peak:.6} rad, past the guard's bound"
        );
        assert!(
            late_peak <= peak * (1.0 + GRID_TOLERANCE),
            "the late loop stepped {late_peak:.6} rad against the on-time {peak:.6} rad"
        );
        // Half the periods advance the clock by half a period, so the move
        // takes about a third again as many of them, and the goals it emits are
        // the same path sampled finer.
        assert!(
            jittery.len() > on_time.len(),
            "{} goals late against {} on time",
            jittery.len(),
            on_time.len()
        );
    }

    /// A move at its floor survives whatever phase the loop wakes at.
    ///
    /// Real lateness is not a tidy alternation: periods begin at arbitrary
    /// fractions of a period late, and each one puts the sampling grid
    /// somewhere new against the path's own peak — which is the whole of what
    /// the stretch's headroom has to cover once the clock itself can no longer
    /// be inflated. Walked over a cycle of offsets on the recovery move whose
    /// clock sits exactly at its floor.
    #[test]
    fn a_floored_move_survives_whatever_phase_the_loop_wakes_at() {
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
            durations: MoveDurations::uniform(secs(0.5)),
            warp: Warp::MinJerk,
        };
        let (state, _) = armed_at(&cfg, &crooked);
        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        assert!(stretch.is_some(), "the fixture move is the stretched one");

        let offsets = [0.0, 0.17, 0.41, 0.63, 0.88];
        let (pinned, goals) = goals_driven_at(&cfg, &crooked, &floored, &|n| {
            PERIOD * n + PERIOD.mul_f64(offsets[n as usize % offsets.len()])
        });
        let worst = worst_step_of(&pinned, &goals);
        assert!(
            worst <= cfg.max_step.body_yaw,
            "stepped {worst:.6} rad at some phase, past the guard's bound"
        );
    }

    /// A clock the right-sizing pass hands back carries its move at whatever
    /// phase the loop wakes at — the clocks it accepts as asked included.
    ///
    /// A stretch takes headroom over the step it measured, because the dry pass
    /// walks one phase of the sampling grid and the live loop walks whichever
    /// phase it wakes on. A clock a little under its closed-form floor can
    /// measure inside the bound on that one grid — the grid reads the min-jerk
    /// peak low by the curvature there — and so is accepted as asked. Without
    /// the same headroom on that decision it steps past the guard the moment
    /// the phase moves, abandoning a healthy move over ordinary scheduler
    /// noise.
    ///
    /// The phase moves on a *short* gap: a late wake is credited at most one
    /// period, so what shifts the grid is the period that recovers the
    /// lateness, not the one that runs late. Walked across the band a fifth of
    /// a millisecond at a time, each clock driven at a spread of shifts.
    #[test]
    fn a_clock_the_pass_accepts_carries_its_move_at_every_phase() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (state, _) = armed_at(&cfg, &start);
        let span = 1.0;
        let floor = duration_floor_s(span, cfg.max_step.body_yaw, FLOOR_TICK_HZ);

        for fifths in -50..=20 {
            let requested = floor + f64::from(fifths) / 5000.0;
            let command = MotionCommand::MoveTo {
                target: JointTargets {
                    body_yaw: span,
                    ..JointTargets::default()
                },
                durations: MoveDurations::uniform(secs(requested)),
                warp: Warp::MinJerk,
            };
            let (floored, _) =
                floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
            for eighths in 1..8 {
                // One short period at the start and the grid from there on:
                // every later sample sits this far before where the dry pass
                // took it.
                let shift = PERIOD.mul_f64(f64::from(eighths) / 8.0);
                let (pinned, goals) = goals_driven_at(&cfg, &start, &floored, &|n| {
                    if n == 0 {
                        Duration::ZERO
                    } else {
                        PERIOD * n - shift
                    }
                });
                let worst = worst_step_of(&pinned, &goals);
                assert!(
                    worst <= cfg.max_step.body_yaw,
                    "{requested:.4} s stepped {worst:.7} rad {eighths} eighths of a period out of \
                     phase, past the guard's bound"
                );
            }
        }
    }

    /// A loop that is late by the same amount every period emits exactly the
    /// goals an on-time loop does.
    ///
    /// Nothing about a move's path depends on where the wall clock's zero is —
    /// only on the gaps between periods — so a constant offset is invisible to
    /// it. Pinned on the emitted bits rather than on a tolerance, because the
    /// two sequences are meant to be the same arithmetic.
    #[test]
    fn a_constant_offset_emits_the_goals_an_on_time_loop_does() {
        let cfg = MotionConfig::default();
        let stow = JointTargets {
            head_pose_body: reachy_kin::stow_head_pose(),
            ..JointTargets::default()
        };
        let command = MotionCommand::MoveTo {
            target: JointTargets::default(),
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };
        let (_, on_time) = goals_driven_at(&cfg, &stow, &command, &|n| PERIOD * n);
        let (_, offset) = goals_driven_at(&cfg, &stow, &command, &|n| PERIOD * n + PERIOD / 3);
        assert_eq!(on_time, offset);
    }

    /// The clock a move runs on is the one it asked for, or its floor, and
    /// there is no band between the two.
    ///
    /// Walked across the boundary a hundredth of a second at a time: every
    /// clock lands on the larger of what was asked and what the span needs,
    /// none is ever shortened, and the sequence never steps backwards by more
    /// than the convergence residual. A margin on top of the floor would show
    /// up here as a jump at the crossing, and a stretch that undershot would
    /// show up as a clock under the floor.
    ///
    /// The residual is why the backwards check is not exact: two requests below
    /// the floor land on it from different starting grids, and the shorter
    /// start measures its ratio a shade lower, so it arrives a shade higher.
    #[test]
    fn the_effective_clock_is_the_requested_one_or_the_floor_and_nothing_between() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &JointTargets::default());
        let span = 1.0;
        let floor = duration_floor_s(span, cfg.max_step.body_yaw, FLOOR_TICK_HZ);

        let mut previous = 0.0;
        // Straddling the floor, which the yaw's measured bound puts at a
        // quarter of a second: a band that sat entirely above it would pass
        // without ever crossing the thing under test.
        for hundredths in 10..=60 {
            let requested = f64::from(hundredths) / 100.0;
            let command = MotionCommand::MoveTo {
                target: JointTargets {
                    body_yaw: span,
                    ..JointTargets::default()
                },
                durations: MoveDurations::uniform(secs(requested)),
                warp: Warp::MinJerk,
            };
            let (_, stretch) =
                floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
            let effective =
                stretch.map_or(requested, |stretch| stretch.effective.head.as_secs_f64());

            let wanted = requested.max(floor);
            assert!(
                lands_on(effective, wanted, FLOOR_TICK_HZ),
                "{requested:.2} s ran on {effective:.4} s, not the {wanted:.4} s it should"
            );
            assert!(
                effective >= requested,
                "{requested:.2} s was shortened to {effective:.4} s"
            );
            let residual = 2.0 * grid_headroom(previous * FLOOR_TICK_HZ) + GRID_TOLERANCE;
            assert!(
                effective >= previous * (1.0 - residual),
                "{requested:.2} s ran on {effective:.4} s, shorter than the clock before it"
            );
            previous = effective;
        }
    }

    /// A stretch is sized from where the move starts, so a replacement issued
    /// partway through a recovery is right-sized for the span still ahead of it
    /// rather than for the one the original command had.
    #[test]
    fn a_stretch_follows_the_span_still_ahead() {
        let cfg = MotionConfig::default();
        let requested = MoveDurations::uniform(secs(0.3));
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
                .expect("three tenths of a second carries neither span")
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
            durations: MoveDurations::uniform(secs(0.15)),
            warp: Warp::MinJerk,
        };

        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("a seventh of a second cannot carry a turn of antenna");

        let long_floor = duration_floor_s(long_arc, cfg.max_step.antennas, FLOOR_TICK_HZ);
        let short_floor = duration_floor_s(short_arc.abs(), cfg.max_step.antennas, FLOOR_TICK_HZ);
        let moved = stretch.effective.antennas[0].as_secs_f64();
        assert!(
            lands_on(moved, long_floor, FLOOR_TICK_HZ),
            "stretched to {moved:.4} s, not the {long_floor:.4} s the long way round needs"
        );
        assert!(
            moved > short_floor * 1.5,
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

    /// Where the machine stands with its antennas at `antennas` and the head
    /// stowed: the pose every recorded antenna sweep started from.
    fn stowed_with(antennas: [f64; 2]) -> JointTargets {
        JointTargets {
            head_pose_body: reachy_kin::stow_head_pose(),
            antennas,
            ..JointTargets::default()
        }
    }

    /// The stow representative the recordings hold the pair at, a hair over half
    /// a turn from straight up on each side.
    const SWEPT_FROM: [f64; 2] = [3.2336, -3.2336];

    /// The gesture the pair is judged on: both antennas inboard to straight up,
    /// on `antennas` and a head clock of `head`.
    fn inboard_sweep(head: f64, antennas: [f64; 2]) -> MotionCommand {
        MotionCommand::MoveTo {
            target: JointTargets::default(),
            durations: MoveDurations {
                head: secs(head),
                antennas: [secs(antennas[0]), secs(antennas[1])],
            },
            warp: Warp::MinJerk,
        }
    }

    /// The validated pair — one side quick, the other slow — is what the
    /// separation was sized to admit, and it comes back untouched.
    #[test]
    fn the_staggered_pair_sweeps_as_it_was_asked_to() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &stowed_with(SWEPT_FROM));
        let command = inboard_sweep(0.8, [0.7, 0.3]);

        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        assert_eq!(stretch, None, "{stretch:?}");
        assert_eq!(floored, command);

        let pair = dry_pass_separation(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ)
            .expect("both antennas cross the band");
        assert!(pair.met(cfg.phase.separation_rad), "{pair:?}");
    }

    /// Both numbers are configuration, and a configuration that moves them
    /// moves what the resolver makes of the same command.
    ///
    /// The validated pair clears the shipped separation and comes back with
    /// nothing said; a configuration asking for more says the pair falls short
    /// and names the figure it fell short of. And the band is where the pair is
    /// measured: narrow it and the arrival the verdict is taken at moves later
    /// into the sweep, on tips nearer the crossing.
    #[test]
    fn the_pair_is_judged_against_the_configured_geometry() {
        let shipped = MotionConfig::default();
        let (state, _) = armed_at(&shipped, &stowed_with(SWEPT_FROM));
        let command = inboard_sweep(0.8, [0.7, 0.3]);

        let mut demanding = shipped.clone();
        demanding.phase.separation_rad = 1.2;
        let (_, stretch) =
            floor_move_clock(&demanding, state.last_targets(), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("the pair no longer clears the separation asked for");
        assert!(
            (stretch.separation_required - 1.2).abs() < 1e-12,
            "the report carries what it judged against: {stretch:?}"
        );
        assert!(
            !stretch
                .separation
                .expect("the pair was measured")
                .met(demanding.phase.separation_rad),
            "{stretch:?}"
        );
        // The quick side has arrived and stopped by the time the slow one
        // reaches the edge, so no delay parts them further and the pass says so
        // rather than stretching for nothing.
        assert!(!stretch.dephased, "{stretch:?}");

        // And the de-phasing aims at the configured figure rather than at a
        // number of its own: a pair in step is parted to a little past what
        // was asked for, whatever was asked for.
        let in_step = inboard_sweep(0.8, [0.8, 0.8]);
        let mut lax = shipped.clone();
        lax.phase.separation_rad = 0.3;
        let parted = |cfg: &MotionConfig| {
            let (floored, stretch) =
                floor_move_clock(cfg, state.last_targets(), &in_step, FLOOR_TICK_HZ);
            let stretch = stretch.expect("a pair in step is de-phased");
            assert!(stretch.dephased, "{stretch:?}");
            let pair = dry_pass_separation(cfg, state.last_targets(), &floored, FLOOR_TICK_HZ)
                .expect("the de-phased pair still crosses the band");
            (stretch.effective.antennas[0], pair.offset)
        };
        let (lax_clock, lax_offset) = parted(&lax);
        let (shipped_clock, shipped_offset) = parted(&shipped);
        assert!(
            lax_clock < shipped_clock,
            "a laxer separation asked for the same delay: {lax_clock:?} against {shipped_clock:?}"
        );
        assert!(
            (0.30..0.45).contains(&lax_offset),
            "the laxer pass parted the pair to {lax_offset:.4} rad, not to a little past 0.30"
        );
        assert!(
            (0.60..0.75).contains(&shipped_offset),
            "the shipped pass parted the pair to {shipped_offset:.4} rad"
        );

        let mut narrow = shipped.clone();
        narrow.phase.contact_band_rad = 0.4;
        let wide_band =
            dry_pass_separation(&shipped, state.last_targets(), &command, FLOOR_TICK_HZ)
                .expect("both antennas cross the band");
        let narrow_band =
            dry_pass_separation(&narrow, state.last_targets(), &command, FLOOR_TICK_HZ)
                .expect("both antennas cross the narrower band too");
        assert!(
            narrow_band.at > wide_band.at,
            "the narrower band is reached later: {narrow_band:?} against {wide_band:?}"
        );
    }

    /// A pair needs two sides going somewhere. A command that leaves one
    /// antenna where it stands has no second arrival to be late for, so there
    /// is nothing to measure and no walk taken to find that out.
    #[test]
    fn a_command_that_moves_one_antenna_is_not_a_pair() {
        let cfg = MotionConfig::default();
        let held = stowed_with(SWEPT_FROM);
        let (state, _) = armed_at(&cfg, &held);
        let command = MotionCommand::MoveTo {
            target: JointTargets {
                antennas: [0.0, held.antennas[1]],
                ..JointTargets::default()
            },
            durations: MoveDurations {
                head: secs(0.8),
                antennas: [secs(0.7), secs(0.7)],
            },
            warp: Warp::MinJerk,
        };

        assert_eq!(
            dry_pass_separation(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ),
            None
        );
        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        assert_eq!(stretch, None, "{stretch:?}");
        assert_eq!(floored, command);
    }

    /// The phase question spans the antennas' own clocks. A head clock ten
    /// times longer is ten times the walk to reach the same verdict: the pair
    /// has arrived, and where it crossed is where it crossed.
    #[test]
    fn the_pair_is_measured_over_its_own_clocks_and_not_the_moves() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &stowed_with(SWEPT_FROM));

        let quick = dry_pass_separation(
            &cfg,
            state.last_targets(),
            &inboard_sweep(0.8, [0.7, 0.3]),
            FLOOR_TICK_HZ,
        )
        .expect("both antennas cross the band");
        let calm = dry_pass_separation(
            &cfg,
            state.last_targets(),
            &inboard_sweep(8.0, [0.7, 0.3]),
            FLOOR_TICK_HZ,
        )
        .expect("both antennas cross the band");
        assert_eq!(quick, calm);
    }

    /// A pair on one clock reaches the crossing mirror-symmetric, which is the
    /// configuration the tips meet in. The pass delays the side that gets there
    /// second until they clear each other, lengthening nothing else and
    /// shortening nothing at all.
    #[test]
    fn a_pair_sweeping_in_step_is_de_phased() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &stowed_with(SWEPT_FROM));
        let command = inboard_sweep(0.8, [0.8, 0.8]);

        let in_step = dry_pass_separation(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ)
            .expect("both antennas cross the band");
        assert!(
            in_step.offset < 1e-6,
            "one clock leaves the pair {:.4} rad apart",
            in_step.offset
        );

        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("a pair in step is de-phased");
        assert!(stretch.dephased, "{stretch:?}");
        assert_eq!(stretch.effective.head, stretch.requested.head);
        assert_eq!(stretch.effective.antennas[1], stretch.requested.antennas[1]);
        assert!(
            stretch.effective.antennas[0] > stretch.requested.antennas[0],
            "{stretch:?}"
        );

        let separated = dry_pass_separation(&cfg, state.last_targets(), &floored, FLOOR_TICK_HZ)
            .expect("both antennas still cross the band");
        assert!(
            separated.met(cfg.phase.separation_rad),
            "de-phased to {:.4} rad, which is still under the bound",
            separated.offset
        );
    }

    /// A stagger a floor collapses is caught by the same check, because the
    /// check runs on the clocks the move will actually run on.
    ///
    /// The quick side here is quicker than its own span allows, so it is
    /// right-sized up towards the slow side's clock — and a stagger that
    /// survives only as long as nothing else moves the clocks is folklore.
    #[test]
    fn a_floor_that_collapses_the_stagger_is_caught_on_the_effective_clocks() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &stowed_with(SWEPT_FROM));
        let asked = inboard_sweep(0.8, [0.05, 0.06]);

        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &asked, FLOOR_TICK_HZ);
        let stretch = stretch.expect("neither side carries its sweep in a twentieth of a second");
        assert!(stretch.dephased, "{stretch:?}");
        let floor = duration_floor_s(SWEPT_FROM[0], cfg.max_step.antennas, FLOOR_TICK_HZ);
        for side in 0..2 {
            assert!(
                stretch.effective.antennas[side].as_secs_f64() >= floor,
                "side {side} runs on {:?}, under the {floor:.4} s its sweep needs",
                stretch.effective.antennas[side]
            );
        }
        let separated = dry_pass_separation(&cfg, state.last_targets(), &floored, FLOOR_TICK_HZ)
            .expect("both antennas cross the band");
        assert!(separated.met(cfg.phase.separation_rad), "{separated:?}");
    }

    /// A leader that has stopped is one no delay separates: the move runs on
    /// the clocks it has and the report says the separation is unmet. Refusing
    /// is not available — the maneuvers that recover this machine come down
    /// this same path.
    #[test]
    fn a_pair_no_delay_parts_runs_and_says_so() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &stowed_with(SWEPT_FROM));
        // The left antenna stops just inside the band, early; the right arrives
        // at the far edge long afterwards, mirroring it.
        let command = MotionCommand::MoveTo {
            target: JointTargets {
                antennas: [0.0, -0.95],
                ..JointTargets::default()
            },
            durations: MoveDurations {
                head: secs(0.8),
                antennas: [secs(1.0), secs(0.2)],
            },
            warp: Warp::MinJerk,
        };

        let (floored, stretch) =
            floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("an unmet separation is reported");
        assert!(!stretch.dephased, "{stretch:?}");
        assert_eq!(stretch.effective, stretch.requested);
        assert_eq!(durations_of(&floored), stretch.requested);
        let pair = stretch.separation.expect("the pair was measured");
        assert!(!pair.met(cfg.phase.separation_rad), "{pair:?}");
        assert_eq!(pair.later, JointId::AntennaRight);
        assert!(pair.leader_rate < 1e-6, "{pair:?}");
    }

    /// And a leader creeping through the crossing is the same answer arrived at
    /// the long way: the delay the arithmetic asks for is past what a clock may
    /// be stretched to, so the move runs on the last clocks with the separation
    /// unmet.
    #[test]
    fn a_de_phasing_that_would_never_end_stops_at_the_cap() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &stowed_with([SWEPT_FROM[0], -1.06]));
        let command = MotionCommand::MoveTo {
            target: JointTargets {
                antennas: [0.0, -0.2],
                ..JointTargets::default()
            },
            durations: MoveDurations {
                head: secs(0.8),
                antennas: [secs(2.0), secs(8.0)],
            },
            // Constant rate, so the left antenna is still crawling out of the
            // band whenever the right one arrives at it.
            warp: Warp::Linear,
        };

        let (_, stretch) = floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("the pair is measured and cannot be parted");
        assert!(stretch.dephased, "{stretch:?}");
        assert!(
            !stretch
                .separation
                .expect("the pair was measured")
                .met(cfg.phase.separation_rad)
        );
        assert_eq!(
            stretch.effective.antennas[0],
            secs(2.0 * MAX_PHASE_STRETCH),
            "the right antenna was carried past its cap"
        );
    }

    /// A pair already standing inside the band has no arrival to be late for,
    /// and one side parked in there while the other sweeps through has only its
    /// own. Both are left alone: a stationary tip is not somewhere a delay puts
    /// the other one.
    #[test]
    fn a_pair_with_no_crossing_between_them_is_left_alone() {
        let cfg = MotionConfig::default();
        for (name, start, target) in [
            ("both already inside", [0.5, -0.5], [0.2, -0.2]),
            ("one parked inside", [SWEPT_FROM[0], -0.5], [0.0, -0.5]),
        ] {
            let (state, _) = armed_at(&cfg, &stowed_with(start));
            let command = MotionCommand::MoveTo {
                target: JointTargets {
                    antennas: target,
                    ..JointTargets::default()
                },
                durations: MoveDurations::uniform(secs(1.0)),
                warp: Warp::MinJerk,
            };
            assert_eq!(
                dry_pass_separation(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ),
                None,
                "{name}"
            );
            assert_eq!(
                floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ).1,
                None,
                "{name}"
            );
        }
    }

    /// A command whose spans the dry pass cannot measure comes back untouched,
    /// pair included.
    ///
    /// The phase walk solves no legs, so it still has an answer for a path the
    /// step walk gave up on — and acting on that answer would report a stretch
    /// for a move the tick is about to refuse for a reason no clock addresses.
    /// The pair here is the mirrored one the resolver de-phases whenever it can
    /// measure anything at all.
    #[test]
    fn a_command_the_spans_cannot_be_measured_on_is_left_alone() {
        let mut cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &stowed_with(SWEPT_FROM));
        let command = inboard_sweep(0.8, [0.8, 0.8]);

        let measurable = floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ)
            .1
            .expect("the mirrored pair is de-phased while the spans can be measured");
        assert!(measurable.dephased);

        // A bound of zero: every ratio against it is a number the pass refuses
        // to reason from, which is one of the three cases the contract names.
        cfg.max_step.legs = 0.0;
        let unmeasurable = floor_move_clock(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ);
        assert_eq!(
            durations_of(&unmeasurable.0),
            durations_of(&command),
            "a command nothing could be measured about was re-clocked"
        );
        assert_eq!(unmeasurable.1, None);
        // And the pair really is the one that would have been parted, so this
        // case is about the early return and not about a pair with nothing to
        // say.
        assert!(
            dry_pass_separation(&cfg, state.last_targets(), &command, FLOOR_TICK_HZ)
                .is_some_and(|pair| !pair.met(cfg.phase.separation_rad))
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
    /// command is taken normally. A sampled path pose that fails the envelope
    /// after its target passed abandons the move instead, which is a different
    /// thing again — the checker and the interpolation have disagreed — and
    /// leaves the machine holding, not faulted.
    #[test]
    fn a_bad_target_is_rejected_and_a_bad_path_is_abandoned() {
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
        // validation no longer passes on the way there. The move dies rather
        // than the tick guessing which verdict to believe.
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
        let Some(MoveAbort::EnvelopePath(violations)) = out.report.aborted else {
            panic!("expected an envelope abort, got {:?}", out.report.aborted);
        };
        assert!(violations.margin);
        assert_eq!(out.report.fault, None, "the machine is healthy");
        assert!(!state.is_faulted());
        assert_eq!(state.mode(), Mode::Holding, "and still commandable");

        // Which the next command proves: the abandoned move left a live state
        // behind, so a wind-down can be driven through the same tick. Against
        // the original bounds, because the tightened ones refuse every target
        // there is — that is what made the path fail in the first place.
        let back = MotionCommand::MoveTo {
            target: start,
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };
        let out = tick_with(&cfg, &mut state, secs(0.06), &pinned, Some(&back));
        assert_eq!(out.report.command, CommandDisposition::Started);
    }

    /// A fault that says control cannot be trusted absorbs everything after it:
    /// no goals, no commands taken, and the standing cause repeated on every
    /// tick so an operator reading one line of output sees it.
    #[test]
    fn a_control_not_trusted_fault_is_absorbing() {
        let cfg = MotionConfig {
            read_loss_ticks: 2,
            ..MotionConfig::default()
        };
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let mut out = TickOutputs::default();
        for n in 0..=cfg.read_loss_ticks {
            motion_tick(
                &cfg,
                &mut state,
                &TickInputs {
                    now: secs(f64::from(n) * 0.02),
                    period: PERIOD,
                    present: None,
                    command: None,
                    health: None,
                },
                &mut out,
            );
        }
        let standing = out.report.fault.expect("a run of missed reads faults");
        assert_eq!(standing, Fault::PositionFeedbackLost { misses: 3 });
        assert_eq!(standing.response(), Response::ImmediateAllTorqueOffToPark);

        let command = MotionCommand::MoveTo {
            target: pose_at(0.19),
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };
        for n in 4..8 {
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
                    period: PERIOD,
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
                faults.then_some(Fault::HeadServoFault {
                    joint: JointId::Leg(2),
                    id: 13,
                    bits
                }),
                "bits {bits:#04x}"
            );
            assert_eq!(
                out.report.masked.contains(JointId::Leg(2)),
                faults,
                "the servo that flagged is out of service, and only it"
            );
        }
    }

    /// Every fault's answer, pinned one variant at a time.
    ///
    /// The table is the whole classification the stack acts on, and a match
    /// with no wildcard is what makes adding a fault a decision somebody has to
    /// make rather than a default they fall into. This is the other half of
    /// that: the arms are also *correct*, and nothing quietly re-shuffles which
    /// condition costs the machine its torque.
    #[test]
    fn every_fault_names_the_maneuver_that_answers_it() {
        let bits = 0x20;
        let table: [(Fault, Response, bool); 8] = [
            (
                Fault::AntennaObstructed {
                    joint: JointId::AntennaRight,
                    error: 0.5,
                },
                Response::DegradeAntennas,
                false,
            ),
            (
                Fault::AntennaServoFault {
                    joint: JointId::AntennaLeft,
                    id: 18,
                    bits,
                },
                Response::DegradeAntennas,
                false,
            ),
            (
                Fault::HeadObstructed {
                    joint: JointId::BodyYaw,
                    error: 0.5,
                },
                Response::SlowStowToRest,
                false,
            ),
            (
                Fault::HeadServoFault {
                    joint: JointId::Leg(2),
                    id: 13,
                    bits,
                },
                Response::MaskedSlowStowToPark,
                false,
            ),
            (
                Fault::PositionFeedbackLost { misses: 51 },
                Response::ImmediateAllTorqueOffToPark,
                true,
            ),
            (
                Fault::MeasuredPoseInvalid {
                    failures: 51,
                    source: FkError::NoConvergence {
                        iters: 7,
                        residual: 1.5e-4,
                    },
                },
                Response::ImmediateAllTorqueOffToPark,
                true,
            ),
            (
                Fault::BusFailure {
                    source: SeqError::NoAnswer {
                        context: StepContext::reg(SeqStep::PinAndEnable, 13, RegId::GoalPosition),
                    },
                },
                Response::ImmediateAllTorqueOffToPark,
                true,
            ),
            (
                Fault::TorqueOffUnconfirmed { id: 14 },
                Response::ImmediateAllTorqueOffToPark,
                true,
            ),
        ];
        for (fault, response, latches) in table {
            assert_eq!(fault.response(), response, "{fault}");
            assert_eq!(
                fault.latches(),
                latches,
                "{fault} leaves the tick commanding or it does not"
            );
        }
    }

    /// Every condition says one word, and no two say the same one.
    ///
    /// The slug is the join: the timeline entry, the operator line and the
    /// daemon's fault cell all carry it, so a typo makes a condition nobody can
    /// alert on and a copy-pasted duplicate conflates two of them. Neither is
    /// visible by reading. Driven off a wildcard-free slot function, so a fault
    /// added to the doctrine cannot be left out of the table.
    #[test]
    fn every_fault_names_itself_and_no_other() {
        let table = [
            (
                Fault::AntennaObstructed {
                    joint: JointId::AntennaRight,
                    error: 0.5,
                },
                "antenna_obstructed",
            ),
            (
                Fault::AntennaServoFault {
                    joint: JointId::AntennaLeft,
                    id: 18,
                    bits: 0x20,
                },
                "antenna_servo_fault",
            ),
            (
                Fault::HeadObstructed {
                    joint: JointId::BodyYaw,
                    error: 0.5,
                },
                "head_obstructed",
            ),
            (
                Fault::HeadServoFault {
                    joint: JointId::Leg(2),
                    id: 13,
                    bits: 0x20,
                },
                "head_servo_fault",
            ),
            (
                Fault::PositionFeedbackLost { misses: 51 },
                "position_feedback_lost",
            ),
            (
                Fault::MeasuredPoseInvalid {
                    failures: 51,
                    source: FkError::NoConvergence {
                        iters: 7,
                        residual: 1.5e-4,
                    },
                },
                "measured_pose_invalid",
            ),
            (
                Fault::BusFailure {
                    source: SeqError::NoAnswer {
                        context: StepContext::reg(SeqStep::PinAndEnable, 13, RegId::GoalPosition),
                    },
                },
                "bus_failure",
            ),
            (
                Fault::TorqueOffUnconfirmed { id: 14 },
                "torque_off_unconfirmed",
            ),
        ];

        let mut seen = [false; 8];
        let mut slugs: Vec<&str> = Vec::new();
        for (fault, slug) in &table {
            seen[fault_slot(fault)] = true;
            assert_eq!(fault.slug(), *slug, "{fault}");
            slugs.push(slug);
        }
        assert!(seen.iter().all(|named| *named), "a fault went unnamed");
        slugs.sort_unstable();
        let distinct = slugs.len();
        slugs.dedup();
        assert_eq!(slugs.len(), distinct, "two faults share a slug: {slugs:?}");
    }

    /// Which condition this is, as a slot in the table above.
    ///
    /// Wildcard-free, so a fault added to the enum leaves the slug table one
    /// slot short rather than silently unasserted.
    fn fault_slot(fault: &Fault) -> usize {
        match fault {
            Fault::AntennaObstructed { .. } => 0,
            Fault::AntennaServoFault { .. } => 1,
            Fault::HeadObstructed { .. } => 2,
            Fault::HeadServoFault { .. } => 3,
            Fault::PositionFeedbackLost { .. } => 4,
            Fault::MeasuredPoseInvalid { .. } => 5,
            Fault::BusFailure { .. } => 6,
            Fault::TorqueOffUnconfirmed { .. } => 7,
        }
    }

    /// The bench incident, pinned: the antennas interfere mid-move, and the
    /// head keeps its presence.
    ///
    /// An antenna snagging on its neighbour must not take the machine's torque
    /// off and drop the head.
    #[test]
    fn an_obstructed_antenna_degrades_the_pair_and_the_head_move_finishes() {
        let cfg = tracking_cfg(0.1, 0.01, 3);
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let target = JointTargets {
            head_pose_body: Isometry3::translation(0.0, 0.0, 0.19),
            antennas: [1.0, 1.0],
            ..JointTargets::default()
        };
        let command = move_to(target, secs(1.0));

        // A machine that follows every goal exactly, except for a right
        // antenna held where it stands.
        let mut present = pinned;
        let mut degraded_at = None;
        let mut out = TickOutputs::default();
        for n in 0..1000 {
            out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                (n == 0).then_some(&command),
            );
            assert_eq!(out.report.fault, None, "tick {n}: no head fault, ever");
            if let Some(fault) = out.report.degraded {
                let Fault::AntennaObstructed { joint, error } = fault else {
                    panic!("tick {n}: expected the antenna's obstruction, got {fault}");
                };
                assert_eq!(joint, JointId::AntennaRight);
                assert!(error > cfg.tracking.threshold_rad, "{error} rad out");
                assert!(
                    out.report.newly_masked.covers(JointGroup::Antennas),
                    "the pair goes out of service together"
                );
                assert_eq!(out.report.newly_masked.len(), 2, "and nothing else does");
                degraded_at = Some(n);
            }
            if let Some(goal) = out.goal {
                present = goal;
                present.antennas[0] = pinned.antennas[0];
            }
            if out.report.completed {
                break;
            }
        }

        let degraded_at = degraded_at.expect("a stuck antenna runs its window out");
        assert!(out.report.completed, "the head move finished");
        assert_eq!(state.mode(), Mode::Holding);
        assert!(!state.is_faulted());
        assert!(state.masked().covers(JointGroup::Antennas));
        assert_eq!(state.masked().len(), 2);
        assert!(
            u32::try_from(degraded_at).expect("a short run") > cfg.tracking.ticks,
            "the run had to be sustained to trip"
        );
    }

    /// One period can carry both a degrade and a raise, and reports both.
    ///
    /// The tracking sweep runs before the health poll and only a *head*
    /// exhaustion returns out of it, so an antenna running its window out on
    /// the period a leg's error bits are read leaves `degraded` and `fault`
    /// both set and the mask carrying all three joints. The layer that turns a
    /// mask entry into a record has to choose between them, so the period it
    /// chooses on is pinned here as a reachable one.
    #[test]
    fn an_antenna_running_out_as_a_leg_flags_reports_both() {
        let cfg = tracking_cfg(0.1, 0.01, 3);
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        let command = move_to(
            JointTargets {
                head_pose_body: Isometry3::translation(0.0, 0.0, 0.19),
                antennas: [1.0, 1.0],
                ..JointTargets::default()
            },
            secs(1.0),
        );

        // The right antenna held where it stands, so its window runs out part
        // way up; the leg's bits are set for the period that window closes on.
        let mut present = pinned;
        let mut flag_now = false;
        let mut both = None;
        for n in 0..1000 {
            let mut health = healthy_servos();
            if flag_now {
                health[4].bits = 0x20;
            }
            let out = tick_with_health(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                &health,
                (n == 0).then_some(&command),
            );
            if out.report.degraded.is_some() && out.report.fault.is_some() {
                both = Some(out.report);
                break;
            }
            flag_now = out.report.tracking_count + 1 == cfg.tracking.ticks;
            if let Some(goal) = out.goal {
                present = goal;
                present.antennas[0] = pinned.antennas[0];
            }
        }

        let report = both.expect("the window closes on the period the leg flags");
        assert!(
            matches!(report.degraded, Some(Fault::AntennaObstructed { .. })),
            "{:?}",
            report.degraded
        );
        assert!(
            matches!(report.fault, Some(Fault::HeadServoFault { .. })),
            "{:?}",
            report.fault
        );
        assert!(
            report.newly_masked.covers(JointGroup::Antennas),
            "the pair went with the leg: {}",
            report.newly_masked
        );
        assert!(report.newly_masked.contains(JointId::Leg(3)));
        assert_eq!(report.newly_masked.len(), 3);
    }

    /// A head servo flagging takes that servo out of service and abandons the
    /// move — and leaves the tick commanding, because the semi-controlled
    /// descent that answers it is a stow driven through this same state.
    #[test]
    fn a_head_servo_fault_masks_the_servo_and_leaves_the_tick_holding() {
        let cfg = MotionConfig::default();
        // Raised, so the move a servo flags during is the stow itself, and the
        // one commanded after it has somewhere to go.
        let (mut state, pinned) = armed_at(&cfg, &pose_at(0.19));
        let stow = move_to(JointTargets::default(), secs(2.0));

        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&stow));
        assert_eq!(out.report.command, CommandDisposition::Started);

        let mut health = healthy_servos();
        health[4].bits = 0x20;
        let out = tick_with_health(&cfg, &mut state, secs(0.02), &pinned, &health, None);
        assert_eq!(
            out.report.fault,
            Some(Fault::HeadServoFault {
                joint: JointId::Leg(3),
                id: 14,
                bits: 0x20,
            })
        );
        assert_eq!(out.goal, None, "the offending period commands nothing");
        assert_eq!(out.report.mode, Mode::Holding, "no absorbing state");
        assert_eq!(
            out.report.newly_masked.iter().collect::<Vec<_>>(),
            vec![JointId::Leg(3)]
        );
        assert_eq!(out.report.masked, out.report.newly_masked);

        // The stow the fault is answered with is commanded again on the next
        // period, and runs on the eight joints that remain.
        let out = tick_with_health(&cfg, &mut state, secs(0.04), &pinned, &health, Some(&stow));
        assert_eq!(out.report.command, CommandDisposition::Started);
        assert_eq!(
            out.report.fault, None,
            "the masked servo's bits raise nothing the second time"
        );
        assert!(out.report.newly_masked.is_empty());
        let (_, out) = run_command(&cfg, &mut state, &pinned, &stow, 0.02);
        assert!(out.report.completed, "the masked stow reaches its endpoint");
    }

    /// The mask covers its own trigger. Hardware error bits latch in the servo,
    /// so a masked one keeps flagging on every poll for the rest of the
    /// session; the raise checks skip it exactly as the goal and step checks
    /// do. What must still be seen is the *next* servo to go — which a sweep
    /// stopping at the first unhealthy servo in bus order never would.
    #[test]
    fn a_masked_servo_keeps_flagging_and_the_next_one_is_still_named() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let mut health = healthy_servos();
        health[2].bits = 0x20;
        let out = tick_with_health(&cfg, &mut state, secs(0.0), &pinned, &health, None);
        assert_eq!(
            out.report.fault,
            Some(Fault::HeadServoFault {
                joint: JointId::Leg(1),
                id: 12,
                bits: 0x20,
            })
        );

        // The same bits, poll after poll: reported, and nothing more.
        for n in 1..4 {
            let out = tick_with_health(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &pinned,
                &health,
                None,
            );
            assert_eq!(out.report.fault, None, "poll {n}");
            assert!(out.report.newly_masked.is_empty(), "poll {n}");
            assert_eq!(
                out.report.health.expect("the sweep is reported")[2].bits,
                0x20,
                "poll {n}: the bits are still visible"
            );
        }

        // A second servo, later in bus order than the one still flagging.
        health[5].bits = 0x04;
        let out = tick_with_health(&cfg, &mut state, secs(0.08), &pinned, &health, None);
        assert_eq!(
            out.report.fault,
            Some(Fault::HeadServoFault {
                joint: JointId::Leg(4),
                id: 15,
                bits: 0x04,
            }),
            "the mask expands to the new servo, not back onto the old one"
        );
        assert_eq!(
            out.report.newly_masked.iter().collect::<Vec<_>>(),
            vec![JointId::Leg(4)]
        );
        assert_eq!(state.masked().len(), 2);
    }

    /// Every fault the tick raises is in the session's record, stamped with the
    /// period it was raised on — and each of them once.
    #[test]
    fn every_fault_the_tick_raises_is_recorded_where_it_happened() {
        let cfg = MotionConfig::default();
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        assert!(state.timeline().is_empty(), "nothing has gone wrong yet");

        // An antenna flagging: the pair goes out of service, and that is one
        // entry.
        let mut health = healthy_servos();
        health[7].bits = 0x20;
        let out = tick_with_health(&cfg, &mut state, secs(0.02), &pinned, &health, None);
        let degraded = out.report.degraded.expect("the pair is degraded");
        assert_eq!(
            state.timeline().entries(),
            [Entry::Fault {
                fault: degraded,
                at: secs(0.02),
            }]
        );

        // A leg flagging on a later poll: a second entry, at its own instant.
        health[2].bits = 0x04;
        let out = tick_with_health(&cfg, &mut state, secs(0.06), &pinned, &health, None);
        let raised = out.report.fault.expect("a head servo dropping out");
        assert_eq!(state.timeline().last_fault(), Some(raised));
        assert_eq!(state.timeline().entries().len(), 2);
        assert_eq!(state.timeline().entries()[1].at(), secs(0.06));
        assert_eq!(
            state.timeline().open_maneuver(),
            None,
            "nothing answers yet"
        );
    }

    /// The bits stay latched for the rest of the session; the record does not
    /// grow with them.
    ///
    /// The mask-scope rule as the reporting channel sees it: a timeline that
    /// took an entry per poll would bury the story of an incident under the
    /// standing condition that started it.
    #[test]
    fn a_standing_fault_is_recorded_once_and_not_at_poll_rate() {
        let cfg = MotionConfig::default();
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        let mut health = healthy_servos();
        health[7].bits = 0x20;

        for n in 0..8 {
            tick_with_health(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &pinned,
                &health,
                None,
            );
        }
        assert_eq!(state.timeline().entries().len(), 1, "{}", state.timeline());

        // And the same for a fault that stops the tick commanding: the report
        // repeats it every period, the record does not.
        let mut blind = MotionState::new_armed(
            &record_at(&cfg, &pinned, &Isometry3::translation(0.0, 0.0, 0.15)),
            JointSet::EMPTY,
        );
        let mut out = TickOutputs::default();
        for n in 0..u64::from(cfg.read_loss_ticks) + 4 {
            #[expect(
                clippy::cast_precision_loss,
                reason = "a tick count this small is exact in f64"
            )]
            let now = secs(n as f64 * 0.02);
            motion_tick(
                &cfg,
                &mut blind,
                &TickInputs {
                    now,
                    period: PERIOD,
                    present: None,
                    command: None,
                    health: None,
                },
                &mut out,
            );
        }
        assert!(matches!(
            out.report.fault,
            Some(Fault::PositionFeedbackLost { .. })
        ));
        assert_eq!(blind.timeline().entries().len(), 1, "{}", blind.timeline());
    }

    /// A subscriber hears the entries as they are made.
    #[test]
    fn a_subscriber_hears_the_faults_as_they_are_raised() {
        let cfg = MotionConfig::default();
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        let entries = state.subscribe_timeline();
        let mut health = healthy_servos();
        health[2].bits = 0x20;
        let out = tick_with_health(&cfg, &mut state, secs(0.04), &pinned, &health, None);

        assert_eq!(
            entries.try_iter().collect::<Vec<_>>(),
            vec![Entry::Fault {
                fault: out.report.fault.expect("a servo dropped out"),
                at: secs(0.04),
            }]
        );
    }

    /// An antenna servo flagging degrades the pair and the move carries on;
    /// its bits then raise nothing however long they stay latched.
    #[test]
    fn an_antenna_servo_fault_degrades_the_pair_and_raises_once() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let command = move_to(pose_at(0.19), secs(2.0));
        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&command));
        assert_eq!(out.report.command, CommandDisposition::Started);

        let mut health = healthy_servos();
        health[8].bits = 0x04;
        let out = tick_with_health(&cfg, &mut state, secs(0.02), &pinned, &health, None);
        assert_eq!(
            out.report.degraded,
            Some(Fault::AntennaServoFault {
                joint: JointId::AntennaLeft,
                id: 18,
                bits: 0x04,
            })
        );
        assert_eq!(out.report.fault, None, "the head is not in this");
        assert!(out.report.newly_masked.covers(JointGroup::Antennas));
        assert!(
            matches!(out.report.mode, Mode::Moving { .. }),
            "still moving"
        );

        let out = tick_with_health(&cfg, &mut state, secs(0.04), &pinned, &health, None);
        assert_eq!(
            out.report.degraded, None,
            "raised on entry to the mask only"
        );
        assert!(out.report.newly_masked.is_empty());
        assert!(out.report.masked.covers(JointGroup::Antennas));
    }

    /// A masked joint is checked for nothing. It stands still under a goal that
    /// walks away from it and no tracking run opens; a step that would abandon
    /// the move on any other joint passes, because nothing of what the planner
    /// produced for it reaches the wire.
    #[test]
    fn a_masked_joint_is_checked_for_nothing() {
        let cfg = tracking_cfg(0.1, 0.01, 3);
        let start = antennas_at([0.0, 0.0]);
        let (mut state, pinned) = armed_at(&cfg, &start);

        let mut health = healthy_servos();
        health[7].bits = 0x20;
        let out = tick_with_health(&cfg, &mut state, secs(0.0), &pinned, &health, None);
        assert!(out.report.masked.covers(JointGroup::Antennas));

        // A sweep far faster than the antenna step bound admits, run against a
        // pair that never moves: neither guard has anything to say.
        let sweep = move_to(antennas_at([2.0, 2.0]), secs(0.1));
        let mut present = pinned;
        let mut out = TickOutputs::default();
        for n in 1..12 {
            out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                (n == 1).then_some(&sweep),
            );
            assert_eq!(out.report.fault, None, "tick {n}");
            assert_eq!(out.report.aborted, None, "tick {n}");
            assert_eq!(out.report.tracking_count, 0, "tick {n}: no run opens");
            if let Some(goal) = out.goal {
                present = goal;
                present.antennas = pinned.antennas;
            }
            if out.report.completed {
                break;
            }
        }
        assert!(out.report.completed);
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
                    period: PERIOD,
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
        assert_eq!(
            out.report.fault,
            Some(Fault::PositionFeedbackLost { misses: 4 })
        );
        assert!(state.is_faulted());
    }

    /// A run needs the breach sustained: it clears on the first tick back
    /// inside the threshold, and only a run as long as the configured one
    /// faults. Three joints run out together here — two head joints and an
    /// antenna — so the two selections both show: the head decides the tick,
    /// and the furthest-out head joint is the one it names.
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
        lagging.legs[3] += 0.2;

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
                period: PERIOD,
                present: None,
                command: None,
                health: None,
            },
            &mut out,
        );
        assert!(!out.report.present_fresh);
        assert_eq!(out.report.tracking_count, 2);

        let out = tick_with(&cfg, &mut state, secs(0.1), &lagging, None);
        let Some(Fault::HeadObstructed { joint, error }) = out.report.fault else {
            panic!(
                "expected the head's obstruction, got {:?}",
                out.report.fault
            );
        };
        assert_eq!(
            joint,
            JointId::Leg(3),
            "of the head runs that ran out together, the furthest out is named"
        );
        assert!((error - 0.2).abs() < 1e-9, "{error} rad out");
        assert_eq!(
            out.report.degraded, None,
            "the antenna's own run is subsumed: the head's answer stows the machine"
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
                period: PERIOD,
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
                if out.report.completed
                    || out.report.fault.is_some()
                    || out.report.aborted.is_some()
                {
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
    /// with. The second is the shipped configuration driven at the fastest
    /// gesture it admits: a mirrored antenna sweep whose lag runs to several
    /// times the threshold, which is the regime the shipped numbers have to
    /// hold in. A joint following at a distance has to survive both.
    fn regimes() -> [(MotionConfig, JointTargets, Duration, usize); 2] {
        [
            (tracking_cfg(0.02, 0.002, 10), pose_at(0.19), secs(2.0), 8),
            (
                MotionConfig::default(),
                antennas_at([3.0, -3.0]),
                secs(0.5),
                8,
            ),
        ]
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
                    Some(Fault::HeadObstructed {
                        joint: JointId::BodyYaw,
                        error: 0.3,
                    }),
                    "tick {n}"
                );
            }
        }
        assert!(
            !state.is_faulted() && state.mode() == Mode::Holding,
            "the motors still command, so the stow that answers this is driven \
             from right here"
        );
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
                Some(Fault::HeadObstructed {
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
        let Some(Fault::HeadObstructed { joint, .. }) = last else {
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
        let Some(Fault::HeadObstructed { joint, .. }) = last else {
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
            if let Some(fault) = out.report.degraded {
                let Fault::AntennaObstructed { joint, .. } = fault else {
                    panic!("expected an antenna's obstruction at tick {n}, got {fault}");
                };
                assert_eq!(joint, JointId::AntennaRight);
                assert_eq!(out.report.fault, None, "an antenna stops no head");
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
                Some(Fault::HeadObstructed {
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
                    period: PERIOD,
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
            Some(Fault::HeadObstructed {
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
    /// crossing the anchor flips the sign of progress. At these step sizes it
    /// never gets there: a goal on its way to the anchor passes through the
    /// threshold band around the joint first, which clears the run. The
    /// crossing itself, which wider steps do reach, is
    /// [`a_goal_crossing_its_anchor_re_anchors_the_run`].
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

    /// The configuration the re-anchor exists for: a step bound wide enough
    /// that one period's goal jumps clean over the threshold band, which is
    /// every fast move this machine has been measured making.
    fn crossing_cfg() -> MotionConfig {
        MotionConfig {
            max_step: JointStep {
                legs: 2.0,
                body_yaw: 2.0,
                antennas: 2.0,
            },
            tracking: TrackingFaultConfig {
                threshold_rad: 0.05,
                progress_min_rad: 0.01,
                ticks: 5,
            },
            ..MotionConfig::default()
        }
    }

    /// A body-yaw move that lands its whole span on the next period, so the
    /// goal reaches the far side of the joint without ever passing through the
    /// band around it.
    fn yaw_jump(body_yaw: f64) -> MotionCommand {
        MotionCommand::MoveTo {
            target: JointTargets {
                body_yaw,
                ..JointTargets::default()
            },
            durations: MoveDurations::uniform(PERIOD),
            warp: Warp::Linear,
        }
    }

    /// A goal that steps to the far side of an open run's anchor restarts the
    /// run where the joint stands, rather than reading the joint as running
    /// away from a distance that no longer exists.
    ///
    /// The stalled joint is the discriminating case: nothing about the machine
    /// changes across the crossing, so the only question is which window the
    /// fault comes out of. It comes out of the fresh one — detection of a real
    /// stall is delayed by one window per crossing, and never lost.
    #[test]
    fn a_goal_crossing_its_anchor_re_anchors_the_run() {
        let cfg = crossing_cfg();
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());

        // Fully stalled: every period reports the pose the machine was armed
        // at, whatever it is asked for.
        let mut counts = Vec::new();
        let mut faulted_at = None;
        for n in 0..12u32 {
            let command = match n {
                0 => Some(yaw_jump(0.4)),
                3 => Some(yaw_jump(-0.4)),
                _ => None,
            };
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &pinned,
                command.as_ref(),
            );
            counts.push(out.report.tracking_count);
            if out.report.fault.is_some() && faulted_at.is_none() {
                faulted_at = Some((n, out.report.fault));
            }
        }

        // Period 1 emits the outbound goal, so period 2 is the first to measure
        // against it and opens the run; period 4 emits the reversed goal, and
        // period 5 is the first to measure against that one.
        assert_eq!(
            counts[2..=4],
            [1, 2, 3],
            "the outbound run grew: {counts:?}"
        );
        assert_eq!(counts[5], 1, "the crossing restarted it: {counts:?}");
        assert_eq!(
            counts[6..=9],
            [2, 3, 4, 5],
            "and the fresh window ran out on its own: {counts:?}"
        );
        assert_eq!(
            faulted_at,
            Some((
                9,
                Some(Fault::HeadObstructed {
                    joint: JointId::BodyYaw,
                    error: 0.4,
                })
            )),
            "a window after the crossing, not on it: {counts:?}"
        );
    }

    /// A joint that is following, through the same crossing. The run restarts
    /// on the period the goal turns round, so the periods the joint spends
    /// still travelling the old way are measured from where it actually is —
    /// which is the false positive a narrow step bound would prevent, and does
    /// not need to.
    #[test]
    fn a_following_joint_survives_a_goal_that_jumps_past_its_anchor() {
        let cfg = crossing_cfg();
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());

        // Chasing the outbound goal, coasting two periods past the reversal,
        // then turning round and chasing the other way.
        let chase = [
            0.0, 0.0, 0.10, 0.20, 0.28, 0.33, 0.35, 0.30, 0.20, 0.10, 0.0, -0.15,
        ];
        let mut crossing = None;
        for (n, yaw) in chase.iter().enumerate() {
            let command = match n {
                0 => Some(yaw_jump(0.4)),
                4 => Some(yaw_jump(-0.4)),
                _ => None,
            };
            let present = JointVector {
                body_yaw: *yaw,
                ..pinned
            };
            let out = tick_with(
                &cfg,
                &mut state,
                secs(n as f64 * 0.02),
                &present,
                command.as_ref(),
            );
            assert_eq!(out.report.fault, None, "period {n}");
            if n == 6 {
                crossing = Some(out.report.tracking_count);
            }
            if n >= 6 {
                assert!(
                    out.report.tracking_count <= 1,
                    "period {n}: the run never grew, {}",
                    out.report.tracking_count
                );
            }
        }
        assert_eq!(
            crossing,
            Some(1),
            "the period that first measured against the reversed goal restarted the run"
        );
    }

    /// The residual this mechanism accepts, pinned so it is a decision and not
    /// a surprise: goals that re-cross an anchor faster than the window is long
    /// keep restarting the run, and a stalled joint under them is never
    /// detected. Reaching this regime takes goal steps well past the threshold
    /// at every crossing, and the servos' own overload protection is what
    /// stands behind a joint driven into a stop.
    #[test]
    fn goals_re_crossing_faster_than_the_window_never_detect_a_stall() {
        let cfg = crossing_cfg();
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());

        let mut worst_count = 0;
        let mut worst_error: f64 = 0.0;
        for n in 0..60u32 {
            let command = (n.is_multiple_of(3)).then(|| {
                let sign = if n.is_multiple_of(6) { 1.0 } else { -1.0 };
                yaw_jump(sign * 0.4)
            });
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &pinned,
                command.as_ref(),
            );
            assert_eq!(out.report.fault, None, "period {n}");
            worst_count = worst_count.max(out.report.tracking_count);
            if let Some((_, error)) = out.report.tracking_worst() {
                worst_error = worst_error.max(error);
            }
        }
        assert!(
            worst_count < cfg.tracking.ticks,
            "no window ever ran out: {worst_count}"
        );
        assert!(
            worst_error > cfg.tracking.threshold_rad,
            "and the joint sat well past the threshold throughout: {worst_error}"
        );
        assert!(!state.is_faulted());
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
            Some(Fault::HeadObstructed {
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
            Mode::Moving {
                elapsed: Duration::ZERO
            },
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

    /// A step larger than the bound abandons the move, and is never a trimmed
    /// goal: the servo would take the difference as an immediate jump, and the
    /// interpolator or seed that produced it is the thing worth reporting. The
    /// machine itself is fine, so it keeps its torque and its next command.
    #[test]
    fn an_oversized_step_abandons_the_move() {
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
        let mut abort = None;
        let mut last_goal = pinned;
        for n in 0..10 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &present,
                (n == 0).then_some(&command),
            );
            assert_eq!(out.report.fault, None, "tick {n}: nothing faulted");
            if let Some(goal) = out.goal {
                present = goal;
                last_goal = goal;
            }
            if let Some(raised) = out.report.aborted {
                abort = Some(raised);
                assert_eq!(out.goal, None, "nothing is emitted on the abandoning tick");
                break;
            }
        }
        let Some(MoveAbort::StepTooLarge { joint, delta }) = abort else {
            panic!("expected a step abort, got {abort:?}");
        };
        assert!(matches!(joint, JointId::Leg(_)), "{joint} stepped");
        assert!(delta > 1e-4, "delta {delta}");
        assert_eq!(state.mode(), Mode::Holding, "holding, not faulted");
        assert_eq!(
            *state.last_goal(),
            last_goal,
            "at the last goal it commanded"
        );
    }

    /// The open tracking runs go with the move an abort abandons.
    ///
    /// The same asymmetry a non-latching raise avoids: the maneuver that
    /// answers an abandoned move is commanded on this state, and it measures
    /// tracking from where the machine stands *now*. A run banked against the
    /// goal the move was dropped at would spend a window that was already half
    /// gone, and raise an obstruction out of a healthy machine on the recovery
    /// itself.
    #[test]
    fn an_abort_leaves_no_tracking_run_open() {
        let cfg = MotionConfig {
            max_step: JointStep {
                body_yaw: 0.008,
                ..MotionConfig::default().max_step
            },
            tracking: TrackingFaultConfig {
                threshold_rad: 5e-4,
                ticks: 20,
                ..MotionConfig::default().tracking
            },
            ..MotionConfig::default()
        };
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let command = MotionCommand::MoveTo {
            target: JointTargets {
                body_yaw: 0.19,
                ..JointTargets::default()
            },
            durations: MoveDurations::uniform(secs(0.5)),
            warp: Warp::MinJerk,
        };

        // The machine does not follow, so a run opens on the yaw and grows
        // while the min-jerk ramp builds toward the step the guard stops.
        let mut open = 0;
        let mut aborted = None;
        for n in 0..20 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &pinned,
                (n == 0).then_some(&command),
            );
            assert_eq!(out.report.fault, None, "tick {n}");
            if let Some(abort) = out.report.aborted {
                aborted = Some(abort);
                open = out.report.tracking_count;
                break;
            }
        }
        assert!(
            matches!(
                aborted,
                Some(MoveAbort::StepTooLarge {
                    joint: JointId::BodyYaw,
                    ..
                })
            ),
            "{aborted:?}"
        );
        assert!(open >= 2, "the run to be cleared was {open} period(s) old");

        // The next live read is the recovery's first measurement, and it starts
        // its own run rather than continuing the abandoned move's.
        let out = tick_with(&cfg, &mut state, secs(0.5), &pinned, None);
        assert_eq!(out.report.fault, None);
        assert_eq!(
            out.report.tracking_count, 1,
            "the abandoned move's run was carried into the recovery"
        );
    }

    /// Crank angles that close no loop leave the tick with no idea where the
    /// head is, so the frame is skipped — and a run of them says the mechanism
    /// itself is outside the model it is commanded through, which is where the
    /// fault is. One such frame is not that: it is a bad read, and a machine
    /// that de-torqued on one would drop its head on a single corrupt packet.
    #[test]
    fn an_unsolvable_present_pose_faults_only_when_it_persists() {
        let cfg = MotionConfig {
            read_loss_ticks: 3,
            ..MotionConfig::default()
        };
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
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

        for n in 1..=3u32 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &impossible,
                None,
            );
            assert_eq!(out.report.fault, None, "read {n}");
            assert_eq!(out.report.pose_failures, n, "read {n}");
            assert!(!out.report.present_fresh, "read {n}: nothing was placed");
            assert_eq!(out.report.misses, 0, "read {n}: the frame did arrive");
            assert_eq!(out.goal, None, "read {n}");
        }

        // A solvable read closes the run, and the count starts over.
        let out = tick_with(&cfg, &mut state, secs(0.08), &pinned, None);
        assert_eq!(out.report.pose_failures, 0);
        assert!(out.report.present_fresh);

        for n in 1..=4u32 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(0.08 + f64::from(n) * 0.02),
                &impossible,
                None,
            );
            if n <= 3 {
                assert_eq!(out.report.fault, None, "read {n}");
            } else {
                let Some(Fault::MeasuredPoseInvalid { failures, .. }) = out.report.fault else {
                    panic!("expected a pose fault, got {:?}", out.report.fault);
                };
                assert_eq!(failures, 4);
                assert!(state.is_faulted());
            }
        }
    }

    /// The two runs are two runs, and each has its own budget. A read that
    /// never arrived does not count toward the pose run, and an unsolvable read
    /// does not count toward the miss run: silence says the feedback path has
    /// gone, live-but-unsolvable says the mechanism is outside its model, and a
    /// single shared counter would fault on a sequence that is neither.
    #[test]
    fn a_lost_read_and_an_unsolvable_one_count_separately() {
        let cfg = MotionConfig {
            read_loss_ticks: 3,
            ..MotionConfig::default()
        };
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
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

        // Three unsolvable reads, then three that never arrive: six bad
        // periods in a row, and neither budget of three is passed.
        let mut tick = 0u32;
        for n in 1..=3u32 {
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(tick) * 0.02),
                &impossible,
                None,
            );
            tick += 1;
            assert_eq!(out.report.pose_failures, n);
            assert_eq!(out.report.misses, 0, "an unsolvable read did arrive");
            assert_eq!(out.report.fault, None);
        }
        for n in 1..=3u32 {
            let mut out = TickOutputs::default();
            motion_tick(
                &cfg,
                &mut state,
                &TickInputs {
                    now: secs(f64::from(tick) * 0.02),
                    period: PERIOD,
                    present: None,
                    command: None,
                    health: None,
                },
                &mut out,
            );
            tick += 1;
            assert_eq!(out.report.misses, n);
            assert_eq!(
                out.report.pose_failures, 3,
                "a read that never came solved nothing and failed nothing"
            );
            assert_eq!(out.report.fault, None);
        }

        // And one live, solvable read closes both.
        let out = tick_with(
            &cfg,
            &mut state,
            secs(f64::from(tick) * 0.02),
            &pinned,
            None,
        );
        assert_eq!(out.report.misses, 0);
        assert_eq!(out.report.pose_failures, 0);
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

    /// A reading that is not a number is a read that did not arrive: it never
    /// reaches the pose solve, it never reaches the tracking comparison as an
    /// error nobody can place, and it costs the machine one period, not its
    /// torque. Whichever joint carries it, and whichever way it is unplaceable.
    #[test]
    fn a_non_finite_present_read_is_a_missed_read() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();

        for index in 0..JointId::COUNT {
            for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let (mut state, pinned) = armed_at(&cfg, &start);
                let mut present = pinned;
                match index {
                    0 => present.body_yaw = bad,
                    1..=6 => present.legs[index - 1] = bad,
                    _ => present.antennas[index - 7] = bad,
                }

                let out = tick_with(&cfg, &mut state, secs(0.0), &present, None);
                assert_eq!(out.report.fault, None, "slot {index} with {bad}");
                assert_eq!(out.report.misses, 1, "slot {index} with {bad}");
                assert!(!out.report.present_fresh, "slot {index} with {bad}");
                assert_eq!(out.report.tracking_errors, None, "slot {index} with {bad}");
                assert_eq!(out.goal, None);
                assert_eq!(out.report.fk, None, "the solve never ran");
                assert!(!state.is_faulted());
            }
        }
    }

    /// A run of unplaceable reads is a read outage, and ends in the same fault
    /// one of silence does — the reason a corrupt frame is tolerated at all is
    /// that it is one frame, not that a broken feedback path is acceptable.
    #[test]
    fn a_run_of_non_finite_reads_ends_in_read_loss() {
        let cfg = MotionConfig {
            read_loss_ticks: 3,
            ..MotionConfig::default()
        };
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        let mut garbage = pinned;
        garbage.legs[2] = f64::NAN;

        for n in 1..=4u32 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &garbage, None);
            assert_eq!(out.report.misses, n);
            if n <= 3 {
                assert_eq!(out.report.fault, None, "period {n}");
            } else {
                assert_eq!(
                    out.report.fault,
                    Some(Fault::PositionFeedbackLost { misses: 4 })
                );
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
        let mut state = MotionState::new_armed(
            &record_at(&cfg, &pinned, &targets.head_pose_body),
            JointSet::EMPTY,
        );

        let out = tick_with(&cfg, &mut state, secs(0.0), &present, None);
        let Some(Fault::HeadObstructed { joint, error }) = out.report.fault else {
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

            let Some(MoveAbort::StepTooLarge { joint, delta }) = out.report.aborted else {
                panic!(
                    "{group}: expected a step abort, got {:?}",
                    out.report.aborted
                );
            };
            let named = match joint {
                JointId::Leg(_) => "legs",
                JointId::BodyYaw => "body yaw",
                JointId::AntennaRight | JointId::AntennaLeft => "antennas",
            };
            assert_eq!(named, group, "{group}: the abort named {joint}");
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
                    period: PERIOD,
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
                period: PERIOD,
                present: None,
                command: None,
                health: None,
            },
            &mut out,
        );
        assert_eq!(
            out.report.fault,
            Some(Fault::PositionFeedbackLost { misses: 5 })
        );
        assert_eq!(out.goal, None);
    }

    /// `now` is required to be non-decreasing, and this is what a caller that
    /// breaks that gets: nothing at all. A period timed before the one ahead of
    /// it credits the move with no elapsed time, so the machine stands where
    /// the last period put it rather than walking the path back the way it
    /// came — and the periods after it carry on from there, one step at a time,
    /// exactly as they would have without the disturbance.
    ///
    /// Run on the body yaw, which is the axis with enough travel per period for
    /// a rewind to be a slam rather than a wobble.
    #[test]
    fn a_clock_that_runs_backwards_holds_the_path_where_it_stands() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let yaw_move = MotionCommand::MoveTo {
            target: JointTargets {
                body_yaw: 0.5,
                ..JointTargets::default()
            },
            durations: MoveDurations::uniform(secs(2.0)),
            warp: Warp::MinJerk,
        };

        // Two machines given the same move: one is handed a period out of
        // order, the other is not.
        let (mut state, pinned) = armed_at(&cfg, &start);
        let (mut undisturbed, _) = armed_at(&cfg, &start);
        let mut present = pinned;
        let mut clean = pinned;
        for n in 0..20 {
            let now = secs(f64::from(n) * PERIOD.as_secs_f64());
            let command = (n == 0).then_some(&yaw_move);
            let out = tick_with(&cfg, &mut state, now, &present, command);
            assert_eq!(out.report.fault, None, "tick {n}");
            if let Some(goal) = out.goal {
                present = goal;
            }
            let out = tick_with(&cfg, &mut undisturbed, now, &clean, command);
            if let Some(goal) = out.goal {
                clean = goal;
            }
        }
        let advanced = *state.last_goal();

        // One period's worth of elapsed time, long after twenty periods have
        // passed. The sample lands where the move already stood, so the goal is
        // the one the servos are holding and nothing goes out.
        let out = tick_with(&cfg, &mut state, PERIOD, &present, None);
        assert_eq!(out.report.fault, None);
        assert!(!out.report.emitted, "a goal went out for the earlier time");
        assert_eq!(out.goal, None);
        assert_eq!(*state.last_goal(), advanced, "the machine stood still");
        assert!(matches!(state.mode(), Mode::Moving { .. }), "still moving");

        // And the next ordinary period picks the path up where it stood: the
        // disturbed machine's goals are the undisturbed one's, a period behind.
        let out = tick_with(&cfg, &mut state, PERIOD * 2, &present, None);
        assert_eq!(out.report.fault, None);
        assert!(out.report.emitted);
        let out = tick_with(
            &cfg,
            &mut undisturbed,
            secs(20.0 * PERIOD.as_secs_f64()),
            &clean,
            None,
        );
        assert!(out.report.emitted);
        assert_eq!(
            state.last_goal(),
            undisturbed.last_goal(),
            "the out-of-order period cost the move a period and nothing else"
        );
    }

    /// Every fault, abort and refusal renders, and this is what it says. The
    /// messages are the operator's whole view of a stopped machine, and the
    /// formatting in them can go wrong silently.
    #[test]
    fn every_fault_abort_and_refusal_names_itself() {
        let mut violations = EnvelopeViolations::default();
        violations.unreachable[0] = true;
        violations.margin = true;

        let faults: [(Fault, &str); 8] = [
            (
                Fault::PositionFeedbackLost { misses: 50 },
                "no position read for 50 consecutive ticks",
            ),
            (
                Fault::HeadObstructed {
                    joint: JointId::BodyYaw,
                    error: 0.123_456,
                },
                "body yaw is 0.1235 rad from its goal and not closing",
            ),
            (
                Fault::AntennaObstructed {
                    joint: JointId::AntennaRight,
                    error: 0.5,
                },
                "right antenna is 0.5000 rad from its goal and not closing",
            ),
            (
                Fault::HeadServoFault {
                    joint: JointId::Leg(2),
                    id: 13,
                    bits: 0x20,
                },
                "leg 3 (servo 13) reports hardware error bits 0x20",
            ),
            (
                Fault::AntennaServoFault {
                    joint: JointId::AntennaLeft,
                    id: 18,
                    bits: 0x04,
                },
                "left antenna (servo 18) reports hardware error bits 0x04",
            ),
            (
                Fault::BusFailure {
                    source: SeqError::NoAnswer {
                        context: StepContext::reg(SeqStep::PinAndEnable, 13, RegId::GoalPosition),
                    },
                },
                "the bus is not carrying commands: pin and enable of servo 13, goal position: no \
                 answer",
            ),
            (
                Fault::TorqueOffUnconfirmed { id: 14 },
                "servo 14 did not acknowledge torque off and may still be holding",
            ),
            (
                Fault::MeasuredPoseInvalid {
                    failures: 50,
                    source: FkError::NoConvergence {
                        iters: 7,
                        residual: 1.5e-4,
                    },
                },
                "present pose unknown for 50 consecutive live reads: no pose found after 7 \
                 iterations, residual 1.500e-4 m",
            ),
        ];
        for (fault, expected) in faults {
            assert_eq!(fault.to_string(), expected);
        }

        let aborts: [(MoveAbort, &str); 2] = [
            (
                MoveAbort::EnvelopePath(violations),
                "the commanded path left the envelope: leg 1 unreachable, toggle margin below the floor",
            ),
            (
                MoveAbort::StepTooLarge {
                    joint: JointId::Leg(3),
                    delta: 0.5,
                },
                "leg 4 would step 0.5000 rad in one tick",
            ),
        ];
        for (abort, expected) in aborts {
            assert_eq!(abort.to_string(), expected);
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
