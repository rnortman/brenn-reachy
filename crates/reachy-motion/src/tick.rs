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
//! Any stage can raise a [`Fault`], and every fault carries the [`ResponseKind`]
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

use nalgebra::Isometry3;
use reachy_kin::{
    EnvelopeConfig, EnvelopeReport, EnvelopeViolations, FkError, FkOptions, FkStats, HeadGeometry,
    LegAngles, below_limit, check_envelope, forward_kinematics, min_pose_margin, outside_limit,
    wrap_to_pi,
};
use thiserror::Error;

use crate::arm::ArmRecord;
use crate::fault;
use crate::fault::FaultError;
use crate::joints;
use crate::joints::{
    JointGroup, JointRef, JointStep, JointTargets, JointVector, Name, ROW_COUNT, ROWS, ServoHealth,
    flags, group_of, row, worst_joint, worst_row,
};
use crate::phase::{AntennaPhaseConfig, PhaseSeparation, PhaseWatch};
use crate::plant::{GroupPlants, MAX_GAP_PERIODS, Predicted, RESPONSE_DEAD_SAMPLES};
use crate::record;
use crate::seq::{SeqError, SeqFailureKind, failure};
use crate::snap::{DurationError, PoseSnapshotError, duration_from_nanos, duration_nanos};
use crate::traj::{self, MoveDurations, SeedError, Trajectory, TrajectoryError, WarpKind};
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use brenn_reachy__motion__tick_state_clk_rs::TrackingRowSnap;
use clockwork_rs::Duration as SlotDuration;

/// The worst lag any head joint ran at on a healthy recorded gesture, radians —
/// a loaded leg 0.245 rad behind a goal moving near 3.3 rad/s.
///
/// A hardware measurement of how far behind its goal this content runs the
/// machine, which is a figure about the speed of the content against the
/// servos' own profile and not a figure about health: the screen below is sized
/// on the residual against the modelled trajectory instead, and a joint may sit
/// several times this far behind a fast goal while following perfectly. It is
/// pinned against the recordings themselves by the replay suite beside this
/// crate (`tests/replay_test.rs`, over the recordings in `fixtures/traces`),
/// and a report over a live run prints its own worst lag beside it, so both
/// readers of the figure read one number.
pub const RECORDED_WORST_HEAD_LAG_RAD: f64 = 0.245;

/// The worst lag an antenna ran at on the fastest recorded sweep, radians —
/// 1.38 rad behind a goal peaking near 1120°/s, on the sweep whose 855°/s of
/// travel is this machine's speed record, while following perfectly.
///
/// Pinned and read the same way as [`RECORDED_WORST_HEAD_LAG_RAD`], and the
/// clearest case for why a lag says nothing about health: this joint was doing
/// everything its generator could while a radian and a third behind.
pub const RECORDED_WORST_ANTENNA_LAG_RAD: f64 = 1.38;

/// The worst residual any head joint ran at on the recorded clip library,
/// radians — a body yaw crank 0.3884 rad off its modelled trajectory as a goal
/// turned round through it.
///
/// The figure the tracking screen is sized on, which
/// [`RECORDED_WORST_HEAD_LAG_RAD`] is not: distance from where the servo's own
/// generator had got to, over the whole vendor clip library played on the unit.
/// What it is made of is what the model leaves out — the stiction and backlash
/// a crank shows when the goal reverses through it, and a load-dependent steady
/// offset — so it is the measurement `threshold_rad` carries its margin over.
///
/// One of the three recorded tours of that library ran the body yaw to 0.4024
/// rad, so this figure sits 0.014 rad under the worst the shipped pair has been
/// seen to produce, and that reading is the one the sizing rule does not clear:
/// half again over it is 0.6036 rad, past the 0.6 rad threshold, a headroom of
/// 1.491 rather than 1.5. Nothing here moves on that reading — the threshold is
/// not widened and the recorded worsts are re-baked only from a confirmation
/// tour — but a re-bake that carries a body-yaw worst above 0.4 rad puts the
/// replay suite's margin assertion past the shipped threshold, and which of the
/// two gives way is a decision for whoever reads that tour, not an edit made to
/// get the suite green.
///
/// The figure is a body yaw's, at the pair the class still runs and the gains
/// it still runs — neither moved when the legs were commissioned — so the
/// confirmation tour cut no head window and this figure is the recorded tours'
/// as it was. What would move it is a tour that commissions the yaw.
///
/// The same tours are also recorded per joint group as p99.9 residuals, in the
/// offline analyzer (`RECORDED_P999_*_RESIDUAL_RAD`, `cogs/pose_reading.rs`),
/// which is the noise floor a candidate profile's p99.9 is read against, and as
/// the capability pairs beside them (`RECORDED_CAPABILITY_*`). Each class's
/// figures are one configuration's reading, at the pair and the gains the tree
/// ships that class at, and are re-baked together for that class or not at all:
/// a fresh worst printed against a noise floor measured under some other
/// configuration is a comparison of two machines. Every figure is at its class's
/// shipping configuration; where one is not read over three tours of the
/// library, its own comment says so — the antennas' p99.9 array, which is a tour
/// and two sweeps. TODO(session-servo-profile) is the re-bake, and lists the
/// set.
pub const RECORDED_WORST_HEAD_RESIDUAL_RAD: f64 = 0.3884;

/// The worst residual an antenna ran at on the recorded clip library at the
/// configuration the class ships, radians — an antenna 0.2536 rad off its
/// modelled trajectory through a goal reversal.
///
/// The antennas run the library's fastest content and its longest travel, and
/// their residual is nonetheless the smallest of the three classes': what this
/// figure is about is the reversal, not the speed, and the speed is the
/// motor's own.
///
/// The class at its shipping configuration and nothing else: the `522 / 640`
/// pair its own capability reads and it is commissioned at, the `200 / 0 / 0`
/// gains it ships, and the following lag measured on that loop. Which matters
/// here and not on the head's figure, because the antennas have been recorded
/// at two other configurations that read higher. The 2026-09-06 tours ran
/// `500 / 0 / 100` — a proportional term two and a half times this one, and the
/// proportional term is what sets how far a joint follows behind its own
/// generator — and no lag has been read on that stiffer loop, so those
/// recordings are replayed at the generator alone and one of them, `proud1`,
/// stands at 0.3782 rad. The confirmation tour ran this loop at `20 / 50` and
/// stands at 0.3625 rad, the reversal excursion being a fixed number of radians
/// while the lag the model now carries is not. Both are configurations the
/// class has left, which is why the aggregate assertion in the replay suite
/// takes the largest of the three and this constant stays the shipping
/// configuration's reading. Half again over any of them is under the 0.6 rad
/// screen. The class that stands past 0.4 rad on these recordings is the body
/// yaw, whose comment above says what that costs the sizing rule.
///
/// Read off the one library tour flown at this pair, and confirmed on the
/// machine by six armed runs at it. The band is the four that chase content --
/// two sweeps and two one-frame step probes -- whose antenna worsts read
/// 0.2051–0.2535 rad behind and 0.1913–0.2152 rad ahead; the two wake runs of
/// the same confirmation stand far inside it, at 0.0326 rad behind and 0.0818
/// ahead at their worst. The widest of the four is a kept fixture beside the
/// tour's own window.
///
/// Re-baked with [`RECORDED_WORST_HEAD_RESIDUAL_RAD`], and beside the same
/// per-group p99.9 arrays that comment names.
pub const RECORDED_WORST_ANTENNA_RESIDUAL_RAD: f64 = 0.2536;

/// When a joint stands far enough from where a healthy servo following its
/// generator would stand, for long enough, without closing on it or keeping
/// pace with it, to conclude the servo is not tracking.
///
/// Goal writes to the whole group are unacknowledged by the protocol, so a
/// write that never applied leaves no trace on the bus. This comparison is the
/// compensating detection: a goal that is not being followed shows up as a
/// position error that does not close, whether the write landed or the motor
/// stalled.
///
/// Distance from the *goal* cannot say that. Every servo here runs a velocity
/// profile — [`crate::plant`] is that profile as arithmetic — and content
/// asking for more speed than the profile allows leaves every healthy joint far
/// behind its goal: on the recorded clip library the healthy machine runs
/// 1.5-3 rad behind, and [`RECORDED_WORST_ANTENNA_LAG_RAD`] is a joint
/// following perfectly at a radian and a third out. What this judges instead is
/// the residual against the modelled shaft — the generator's trajectory with
/// the position loop's own following lag on it
/// ([`crate::plant::ClassProfile::following_lag_us`], as
/// [`crate::plant::PlantModel`] steps it) — which over those same recordings
/// stays under half a radian, the worst reading of the shipped pair on record
/// being a body yaw at 0.4024, a class whose lag is zero. Two ways past the
/// residual threshold are still healthy and both are checked before anything is
/// raised: a joint closing on the prediction, and a joint behind it but still
/// moving with it.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackingFaultConfig {
    /// Whether an expired window raises anything.
    ///
    /// Shipped `true`. What the comparison is pinned against is the machine
    /// itself: the recordings the threshold below was sized on are fixtures
    /// here, replayed under the profile the deployment commissions, and the
    /// worst residual at each profile is a constant the replay suite holds them
    /// to. What a raise leads to is pinned too, end to end, by the deterministic
    /// suite. The measurement runs either way — `tracking_residuals` and
    /// `tracking_count` are reported whatever this says — so a run's record is
    /// collected even from a build that disarms it.
    pub armed: bool,
    /// How far a joint may stand from the prediction without being examined at
    /// all, radians.
    ///
    /// The screen, and the whole of the margin for what the model leaves out:
    /// the stiction-and-backlash excursion a real joint shows when a goal turns
    /// round through it, a load-dependent steady offset, and count
    /// quantisation. Sized well over the worst of those on record, so what
    /// reaches the two rules below is a joint substantially worse than the
    /// recordings rather than a joint on a bad period.
    pub threshold_rad: f64,
    /// How far a joint past that threshold must close on the prediction, within
    /// a window of `ticks`, to count as catching up on it, radians.
    ///
    /// A gain over the whole window and not a rate: it is measured from the
    /// residual the run opened or last restarted at, which is up to `ticks - 1`
    /// ticks old, so a joint creeping toward a stopped generator by this much
    /// over a window restarts its window on the tick it gets there. What it
    /// separates is a joint that is going nowhere from one that is going
    /// somewhere slowly; a rate floor would be a different rule.
    ///
    /// Also the floor on the generator's own motion below which the pace rule
    /// asks nothing: a generator that has moved less than this since the anchor
    /// has set no pace to keep.
    pub progress_min_rad: f64,
    /// What fraction of the generator's own motion a joint that is falling
    /// further behind must still be covering to count as slow rather than
    /// obstructed.
    ///
    /// The tolerance for a servo worse than its own profile: voltage sag, a
    /// worn gearbox under load. Such a servo does not produce a bounded
    /// excursion — it accumulates `(1 - f) · L` of residual over a move of
    /// length `L` at speed fraction `f`, and the library's content runs the
    /// profile saturated, so on a 2.9 rad antenna raise a servo at four fifths
    /// of the profile ends better than half a radian behind. No distance
    /// threshold sized on the recordings admits that without also admitting a
    /// hand. This rule admits it by asking the only question that separates
    /// them: is the joint still moving with its generator. A jammed joint paces
    /// nothing, whatever its distance.
    pub pace_min: f64,
    /// How many consecutive live ticks past the threshold without either
    /// closing or pacing before the fault.
    pub ticks: u32,
}

impl Default for TrackingFaultConfig {
    /// The threshold is measured off the recorded runs this repo keeps as
    /// fixtures; the pace fraction is judged against what those runs show a
    /// healthy servo doing; the window and the progress minimum are sized
    /// rather than measured.
    fn default() -> Self {
        Self {
            armed: true,
            // Half again over the larger pinned worst residual and a little
            // more: [`RECORDED_WORST_ANTENNA_RESIDUAL_RAD`], an antenna
            // reversal on the recorded clip library, gives 0.5870 rad, and the
            // head's 0.5826 rad. The pinned worsts are not the largest samples
            // those recordings hold — the largest is a body yaw at 0.4024 rad,
            // and the screen stands 1.49 times that reading rather than the
            // 1.53 times the larger pinned worst gives. The replay suite
            // asserts the margin against the recordings themselves, so the two
            // move together. A displacement of a third of a radian from where
            // the generator has got to is what a hand does to a crank, not
            // what a servo does to itself.
            threshold_rad: 0.6,
            // About 6.5 of the servos' 0.088° counts, comfortably above the
            // half-count quantisation floor of 7.7e-4 rad, and under a third
            // of what a servo catching up after a reversal stick moves in a
            // single period. Sized so that a window's worth of it is progress
            // no quantisation and no jitter can fake, rather than as a rate:
            // the closing rule measures the gain over the run.
            progress_min_rad: 0.01,
            // Half the profile. The recordings' healthy joints run the
            // configured velocity cap to within 2 %, so half of it is far
            // outside both a good day and a bad one. A ratio rather than a
            // distance, so it asks the same question of a fast move and a slow
            // one.
            pace_min: 0.5,
            // A fifth of a second at the shipped 50 Hz tick, so a single
            // transient on one read cannot raise it.
            ticks: 10,
        }
    }
}

/// The worst residual any row held for a whole window of `periods` judged
/// periods of one recording, radians.
///
/// The minimum within a window, maximised over the rows and the windows: a
/// residual a run never came back under for that long is the shape the detector
/// answers, and an excursion that closes inside the window is the shape it must
/// not. So this is the figure a run is judged by where the worst single sample
/// is the figure a screen is *sized* by, and the two are printed side by side —
/// the screen has to stand over a sustained reading or the detector fires on
/// content, and it has to stand half again over the worst sample or it is a
/// hair trigger for the next servo that ages.
///
/// A window is `periods` *judged* periods and not `periods` grid slots, because
/// that is the window the detector counts: a period whose grouped read fell
/// short is stepped and never judged, and a stale tick leaves every run exactly
/// where it stood rather than growing or clearing it. So a run spans a hole and
/// runs out on its tenth live reading whatever grid distance the ten covered,
/// and a window formed only from adjacent slots would drop the stretches that
/// straddle a dropped read — the direction that hides margin the machine has
/// already lost.
///
/// Only within one recording, though: the caller passes one run's judged
/// periods, since a window straddling two of them would be a figure about two
/// machines.
///
/// Every row rather than a class at a time: the figure is the worst any joint
/// held, and the head and the antennas are all nine of them. A caller wanting
/// the split takes [`sustained_per_row`] and folds the rows it cares about.
///
/// Zero for a run shorter than a window, which held nothing for one.
#[must_use]
pub fn sustained_residual(judged: &[[f64; ROW_COUNT]], periods: usize) -> f64 {
    sustained_per_row(judged, periods)
        .into_iter()
        .fold(0.0_f64, f64::max)
}

/// The same figure per row: what each joint held across a whole window of
/// `periods` judged periods without coming back under it, radians.
///
/// A caller reading a class off this folds that class's rows rather than
/// handing in a copy of the run with the other rows zeroed.
///
/// Zero for a row over a run shorter than a window, which held nothing for one.
#[must_use]
pub fn sustained_per_row(judged: &[[f64; ROW_COUNT]], periods: usize) -> [f64; ROW_COUNT] {
    let mut worst = [0.0_f64; ROW_COUNT];
    for window in judged.windows(periods.max(1)) {
        for (row, held_worst) in worst.iter_mut().enumerate() {
            let held = window
                .iter()
                .map(|residuals| residuals[row])
                .fold(f64::INFINITY, f64::min);
            *held_worst = held_worst.max(held);
        }
    }
    worst
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
    /// The servos' own trajectory generators, as commissioned, per class: what
    /// the tracking comparison predicts a healthy joint against.
    ///
    /// A model of the machine and not a choice about it — a host reads the
    /// registers it wrote per class and the control period it runs on, and
    /// builds these from them.
    pub plant: GroupPlants,
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
            // The profiles this deployment commissions, at the period it ticks
            // on. A host on another period, or a machine commissioned with
            // other pairs, builds its own.
            plant: GroupPlants::default(),
            phase: AntennaPhaseConfig::default(),
            // One second of silence at 50 Hz.
            read_loss_ticks: 50,
        }
    }
}

/// The default configuration, built once.
///
/// [`MotionConfig::default`] is not a struct literal: its geometry converts six
/// rotation matrices into quaternions and checks each is a proper rotation, and
/// a host that ticks at the control rate would pay for that every period. This
/// is the same value, built on first use and shared — the same arrangement
/// `reachy_kin::default_geometry` makes for the geometry alone, for the same
/// reason.
///
/// It is not a configuration seam and does not become one: a host with anything
/// to say about the envelope, the step bounds or the tracking windows builds a
/// [`MotionConfig`] of its own and passes it to the tick.
#[must_use]
pub fn default_motion_config() -> &'static MotionConfig {
    static NOMINAL: std::sync::OnceLock<MotionConfig> = std::sync::OnceLock::new();
    NOMINAL.get_or_init(MotionConfig::default)
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

/// What a fault is answered with: a maneuver and the state it leaves behind.
///
/// One vocabulary for the whole stack, so the bench and the daemon act on the
/// same six answers and neither re-derives one from a message. The
/// classification happens once, at [`Fault::response`], and travels as this
/// value.
///
/// The vocabulary's own enum, declared in `motion/faults.clk`: the variants
/// this code matches on and the numbers a slot holds an answer in are one
/// thing. [`ResponseKind::None`] is what a slot that has answered nothing
/// holds and no answer the library ever gives; what an ending makes of it is
/// [`crate::winddown::ending::answering`].
pub use brenn_reachy__motion__faults_clk_rs::ResponseKind;

/// What a failed transaction looked like, and the operator's word for each.
///
/// The vocabulary's own enum, declared in `motion/faults.clk` beside the
/// conditions it is read with. A driver's own error type is not what travels
/// here: a [`Fault`] is `Copy` and rides on the report that raises it, and the
/// whole detail of the failure is on the ending the same transaction returned.
/// [`WireFailure::None`] is what a slot holding no transaction failure holds,
/// and no shape a transaction ever failed in.
pub mod wire_failure {
    pub use brenn_reachy__motion__faults_clk_rs::WireFailure;

    vocab_name! {
        /// A transaction failure as an operator line says it.
        pub struct Name(WireFailure) {
            WireFailure::None => "no transaction failure",
            WireFailure::Silent => "silence",
            WireFailure::Corrupt => "a corrupt reply",
            WireFailure::Refused => "a refusal",
            WireFailure::NotWritten => "a write that did not read back",
            WireFailure::Port => "the port itself",
            WireFailure::Unsendable => "a request that could not be sent",
        }
    }
}

pub use wire_failure::WireFailure;

/// What found the bus not carrying, and what it saw.
///
/// One condition reached from two layers. A sequencer judges its own transaction
/// and has the whole step context to say so; the loop driving a move sees one
/// transaction fail and has the servo and the shape of the failure. "The wire
/// stopped carrying commands under torque" is the same condition either way, it
/// takes the same maneuver, and an alert rule keys on one word for it — so both
/// are [`Fault::BusFailure`], and which layer noticed is a detail of the
/// payload rather than a second name.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum BusFailureSource {
    /// A sequencer's verdict, whole.
    #[error("{0}")]
    Sequence(SeqError),
    /// One of the move loop's own transactions, summarised.
    #[error("servo {id}: {}", wire_failure::Name(*.kind))]
    Transaction {
        /// The servo the failed transaction addressed.
        id: u8,
        /// What the failure was.
        kind: WireFailure,
    },
    /// A sequencer's verdict as it comes back out of a fixed-layout slot: the
    /// name of the failure and the servo it happened at, and nothing else.
    ///
    /// [`Self::Sequence`] carries a whole [`SeqError`], which carries a
    /// [`StepContext`](crate::seq::StepContext) and per-variant payloads that
    /// no slot holds. Restoring
    /// one as a fabricated `SeqError` would put a register and a reading in
    /// front of an operator that nothing ever observed, so a restore says what
    /// it knows and stops there. Nothing raises this: it exists only on the way
    /// back in.
    #[error("restored: {} at servo {id}", failure::Name(*.kind))]
    RestoredSequence {
        /// The servo the sequence was addressing.
        id: u8,
        /// Which failure it was.
        kind: SeqFailureKind,
    },
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
    /// closing on where its servo's profile puts it, or keeping pace with it:
    /// interference, a snag, or a hand.
    #[error("{} is {error:.4} rad off its predicted position and not closing", Name(*.joint))]
    AntennaObstructed {
        /// The antenna whose window ran out.
        joint: JointRef,
        /// How far it stood from the prediction, radians.
        error: f64,
        /// The count the run reached on the tick it ran out.
        count: u32,
    },
    /// An antenna servo reported a hardware error beyond the input-voltage bit.
    /// Never rebooted automatically.
    #[error("{} (servo {id}) reports hardware error bits {bits:#04x}", Name(*.joint))]
    AntennaServoFault {
        /// The antenna concerned.
        joint: JointRef,
        /// The reporting servo's bus ID.
        id: u8,
        /// Its hardware-error byte.
        bits: u8,
    },
    /// A leg or the body yaw sat past the tracking threshold for a whole window
    /// without closing on where its servo's profile puts it, or keeping pace
    /// with it: a grab, a snag, or a jam.
    ///
    /// Not a motor failure — the servo still commands — so the answer is a
    /// controlled stow, which is also what helps a hand pushing the head down.
    #[error("{} is {error:.4} rad off its predicted position and not closing", Name(*.joint))]
    HeadObstructed {
        /// The joint whose window ran out, or the furthest out of those whose
        /// windows ran out together.
        joint: JointRef,
        /// How far, radians.
        error: f64,
        /// The count that joint's run reached on the tick it ran out.
        count: u32,
    },
    /// A leg or body-yaw servo reported a hardware error beyond the
    /// input-voltage bit. Never rebooted automatically: a reboot drops the
    /// head.
    #[error("{} (servo {id}) reports hardware error bits {bits:#04x}", Name(*.joint))]
    HeadServoFault {
        /// The joint whose servo it is.
        joint: JointRef,
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
        /// What the failing transaction reported, as the layer that judged it
        /// can carry.
        source: BusFailureSource,
    },
    /// A torque-off write went unacknowledged after every attempt, so the
    /// minimum risk condition is believed rather than known.
    #[error("servo {id} did not acknowledge torque off and may still be holding")]
    TorqueOffUnconfirmed {
        /// The servo that did not answer.
        id: u8,
    },
}

/// Why a move stopped, with the machine still healthy and still commandable.
///
/// Not a fault: every one of these says the plan, or the state the plan was
/// kept in, was wrong — not the platform. The offending sample is never
/// emitted, the trajectory is dropped, and the tick goes back to holding at the
/// last goal it commanded — so a caller can wind the machine down under
/// control, which is the whole difference between a planner bug and a motor
/// that no longer answers.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum MoveAbort {
    /// A sampled path pose failed the envelope after its target had passed it.
    #[error("the commanded path left the envelope: {0}")]
    EnvelopePath(EnvelopeViolations),
    /// One joint's goal would have moved further in one tick than the step
    /// bound allows — an interpolator or a seed is wrong, and the servo would
    /// take the difference as an immediate jump.
    #[error("{} would step {delta:.4} rad in one tick", Name(*.joint))]
    StepTooLarge {
        /// The joint whose step was too large.
        joint: JointRef,
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

    /// Whether nothing here stands further out than the allowance `state`
    /// records — the excursion the running move began at.
    fn no_further_out_than(&self, state: &MotionSnap) -> bool {
        !self
            .window
            .iter()
            .zip(state.excursion_cranks)
            .any(|(now, then)| outside_limit(*now, then))
            && !outside_limit(self.body_yaw, state.excursion_body_yaw)
            && !outside_limit(self.relative_yaw, state.excursion_relative_yaw)
            && !outside_limit(self.cone, state.excursion_cone)
    }

    /// Record these distances as the allowance every sample of the move about
    /// to start is judged against.
    fn store(&self, state: &mut MotionSnap) {
        state.excursion_cranks = self.window;
        state.excursion_body_yaw = self.body_yaw;
        state.excursion_relative_yaw = self.relative_yaw;
        state.excursion_cone = self.cone;
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

/// Counts the antennas' goal register reaches either side of its own zero.
///
/// The wire layer owns the conversion; this figure is repeated here because the
/// bound below is a count bound and the two layers share no crate. A bench test
/// pins it against the conversion itself.
const ANTENNA_GOAL_COUNTS: f64 = 1_048_575.0;

/// Encoder counts in one turn.
///
/// The crate's one statement of it, for everything here that reasons in counts:
/// the goal spans below, and the excursion bound in `stillness`. The wire
/// layer's `dxl_proto::conv::COUNTS_PER_REV` is the same number where the
/// conversion itself lives, and the two layers share no crate.
pub(crate) const COUNTS_PER_TURN: f64 = 4096.0;

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

/// The highest count body yaw's goal register holds.
///
/// The yaw servo is provisioned in single-turn mode — commissioning stops
/// arming unless it reads that mode — and in it the goal register holds one
/// turn, count 0 through this. A goal past it is a goal the register cannot
/// take.
pub const YAW_GOAL_COUNT_MAX: f64 = COUNTS_PER_TURN - 1.0;

/// The count body yaw's goal register would hold for `radians`, rounded to
/// nearest; NaN for a value that is not finite.
///
/// Count-wise rather than an interval in radians, because the register's set is
/// discrete: the last half count below +π rounds to a count one past
/// [`YAW_GOAL_COUNT_MAX`], and an interval in radians would admit it and leave
/// the failure to the bus write. Zero counts sits half a turn below zero
/// radians, the same frame the antenna span above is stated in. The wire layer
/// owns the conversion; the rounding is repeated here because the two layers
/// share no crate.
#[must_use]
pub fn yaw_goal_counts(radians: f64) -> f64 {
    ((radians + core::f64::consts::PI) * COUNTS_PER_TURN / core::f64::consts::TAU).round()
}

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
/// Deliberately not derived from the stow angles. Halfway between straight up
/// and the fold is ±1.66 rad, about 5.1° off horizontal; these are the physical
/// direction and stay put if the stow angles ever move.
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
        warp: WarpKind,
    },
    /// Abandon any active move and hold where the last goal put things.
    Hold,
    /// Command exactly this target on this tick, and nothing beyond it.
    ///
    /// One setpoint, checked and emitted on the period it arrives on: the
    /// caller owns the shaping, and what it hands over each period is the pose
    /// it wants held until the next one. The checks are the ones a sampled path
    /// pose gets — the envelope, and the per-tick step against the last goal —
    /// so a caller that shapes badly is refused rather than obeyed, and a
    /// refusal changes nothing at all. The antennas are taken as they are
    /// given: an absolute angle in the frame the last command left them in, not
    /// a direction to resolve, because a per-tick setpoint names a position
    /// rather than an arc to sweep.
    ///
    /// Abandons an active move, for the same reason [`MotionCommand::Hold`]
    /// does: two things cannot be commanding the same servos.
    Track(JointTargets),
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
    #[error("the commanded {} angle {angle:.4} rad is not commandable", Name(*.joint))]
    AntennaUnreachable {
        /// Which antenna.
        joint: JointRef,
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
    /// A tracked setpoint was accepted, checked and commanded on this tick.
    /// Whether it put goals on the wire is `emitted`, exactly as for a sampled
    /// path pose: a setpoint identical to the last goal writes nothing.
    Tracked,
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
    pub mode: MotionMode,
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
    /// threshold and neither closing on the prediction nor pacing it, including
    /// the tick that raised [`Fault::HeadObstructed`]. A tick without a live
    /// read measures nothing and repeats the standing figure.
    pub tracking_count: u32,
    /// How far each joint stood from where the modelled shaft has got to — the
    /// generator's trajectory with the loop's following lag on it — in bus
    /// order, radians, when the read was live.
    ///
    /// The per-joint residual, not a verdict: a row past the threshold may be
    /// closing on the prediction or pacing it, and a row whose prediction is
    /// not yet seeded reports zero because nothing was measured.
    pub tracking_residuals: Option<[f64; ROW_COUNT]>,
    /// What the present-pose solve cost, when it ran and succeeded.
    pub fk: Option<FkStats>,
    /// The present pose's smallest toggle margin — the baseline every envelope
    /// check on this tick ran against.
    pub present_min_margin: f64,
    /// The hardware-error bytes, when the health poll ran this tick. Reported
    /// verbatim, including the input-voltage bit that raises no fault.
    pub health: Option<[ServoHealth; ROW_COUNT]>,
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
    pub masked: JointFlags,
    /// The joints that entered the mask on this tick — the ones the caller has
    /// to release before it writes another goal.
    pub newly_masked: JointFlags,
    /// The move abandoned on this tick, and why. Stamped once, on the tick that
    /// dropped the trajectory; the machine is holding afterwards, not faulted,
    /// so nothing repeats it.
    pub aborted: Option<MoveAbort>,
    /// The move dropped because the state said one was running and held nothing
    /// to sample it from, and which way the state disagreed with itself.
    ///
    /// Not an abandoned move in the sense `aborted` carries: nothing was wrong
    /// with the plan, and there was no plan to be wrong. Something wrote this
    /// slot that was not the tick, which is a refusal of the state and counted
    /// as one — and named here, so a machine that stopped mid-move leaves more
    /// trace than a mode change.
    pub unsampleable: Option<StateError>,
}

impl Default for TickReport {
    /// An empty record. Every field is overwritten at the top of each tick, so
    /// this is what an unused output buffer looks like, not a claim about any
    /// machine.
    fn default() -> Self {
        Self {
            mode: MotionMode::Holding,
            present_fresh: false,
            misses: 0,
            pose_failures: 0,
            tracking_count: 0,
            tracking_residuals: None,
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
            masked: JointFlags::NONE,
            newly_masked: JointFlags::NONE,
            aborted: None,
            unsampleable: None,
        }
    }
}

impl TickReport {
    /// The joint furthest from its own prediction, and by how much, when the
    /// read was live.
    ///
    /// Derived from `tracking_residuals` rather than carried beside it, so the
    /// worst named here and the nine figures a summary accumulates cannot
    /// disagree.
    #[must_use]
    pub fn tracking_worst(&self) -> Option<(JointRef, f64)> {
        self.tracking_residuals.as_ref().map(worst_joint)
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
    /// The setpoint the driver reported holding when this sample was taken, or
    /// `None` where it held none — before the first goal reached it, after a
    /// release, after a torque-off.
    ///
    /// The driver's held setpoint and never the tick's own last goal: what the
    /// servos are chasing is what actually reached them, including the
    /// keep-alive rewrites and anything the driver refused, and it is what the
    /// modelled generator is stepped against. A tick handed `None` steps
    /// nothing and re-seeds the prediction from the next reading.
    pub commanded: Option<&'a JointVector>,
    /// A command, at most one per tick.
    pub command: Option<&'a MotionCommand>,
    /// The hardware-error bytes, when the slower health poll ran this tick.
    pub health: Option<&'a [ServoHealth; ROW_COUNT]>,
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

/// The tick's state, as the slot holds it.
///
/// The schema *is* the state. What survives from one execution to the next is
/// what a declared slot holds, so the tick reads and writes the slot's own
/// fields through the validated view and keeps nothing of its own: there is no
/// second form of the mode, the running move, the accumulators or the standing
/// fault, and therefore nothing that can be carried across wrongly.
///
/// A host validates the slot once at the boundary, asks [`resume`] whether the
/// numbers describe a state a tick can run on, and hands the view to
/// [`motion_tick`]. A machine that has just armed starts from [`arm`].
pub use brenn_reachy__motion__tick_state_clk_rs::{MotionMode, MotionSnap, MotionSnapWire};

/// Why a slot's numbers describe no state the motion tick could be in.
///
/// Every variant is a refusal rather than a repair. This state is the fault
/// detectors themselves — the tracking runs, the miss count, the step-bound
/// baseline — and a field read wrongly is a machine that has forgotten how far
/// it is allowed to move next.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum StateError {
    /// A mode this build does not know, the zero included — which is what a slot
    /// nothing has written holds, and is refused rather than read as the state
    /// the machine happens to be safe in.
    #[error("the state names no mode the tick is ever in")]
    NoMode,
    /// [`MotionMode::Moving`] with no move to sample. A live tick would find
    /// nothing to advance and drop to holding; a resume says so instead.
    #[error("the state is moving with no trajectory")]
    MovingWithoutTrajectory,
    /// [`MotionMode::Holding`] with a move. Every way the tick reaches holding
    /// — a hold, an abort, a tracked setpoint, a completed move, a non-latching
    /// fault — drops the path in the same statement, so a holding state carrying
    /// one is a state no sequence of ticks produces, and it reads to anything
    /// that looks at it as a move in flight.
    ///
    /// [`MotionMode::Faulted`] with a move is *not* an error: a latching fault
    /// stops commanding without clearing the path it stopped on.
    #[error("the state is holding with a trajectory")]
    HoldingWithTrajectory,
    /// More held setpoints counted than the ring can hold. The count says how
    /// many of the ring's entries the driver really held, so a count past its
    /// depth counts entries that are not there, and it reads as a full ring:
    /// the model would be stepped toward whatever sits in the oldest slot as
    /// though a driver had held it. `push_held` saturates at the depth, so no
    /// sequence of ticks writes one.
    #[error("the state counts more held setpoints than the ring holds")]
    HeldCountPastTheRing,
    /// The move's seed describes no path.
    #[error("the state's move cannot be rebuilt: {0}")]
    Seed(#[from] SeedError),
    /// A clock that is not a length of time.
    #[error("a clock in the state is not one: {0}")]
    Duration(#[from] DurationError),
    /// A pose field holding numbers that are not a pose.
    #[error("a pose in the state is not one: {0}")]
    Pose(#[from] PoseSnapshotError),
    /// A standing fault whose numbers name no fault.
    #[error("the standing fault in the state is not one: {0}")]
    Fault(#[from] FaultError),
    /// A field holding a number nobody can place, named here by the field it
    /// sat in. Every scalar in this state is a measurement or a commanded
    /// angle, and a machine carrying a NaN in one of them is one whose step
    /// bound, envelope allowance or next move start cannot be computed: it
    /// refuses every command it is given afterwards and says nothing about why.
    #[error("the state's {0} is not a number")]
    NonFinite(&'static str),
}

/// Write the state of a machine that has just finished arming.
///
/// `armed` is arming's record of what it left the machine holding: the goals in
/// the servos' registers, the pose those angles hold, and that pose's smallest
/// toggle margin — the baseline that lets a first move lift off a rest tighter
/// than the clearance floor.
///
/// The **armed** record, specifically, and not the record of where the platform
/// was found. The two are different poses whenever arming had to pull a joint
/// into its travel window, and a state whose goals came from one and whose
/// Cartesian mirror came from the other would hand the first trajectory a start
/// the machine is not at — outside the travel windows every later sample is
/// checked against.
///
/// `degraded` is what arming left out of service — the antennas whose latched
/// error bits the health gate found and engaged around, limp and never enabled.
/// They start in the mask, so nothing here ever commands them, checks them or
/// raises on their standing bits.
///
/// The state is cleared first — through the generated route, so the schema's own
/// declared zeroes are what everything not written here holds: no move running,
/// no tick yet, no excursion allowance, no runs open, no fault. There is no
/// other way to build a state, and that is the point: a tick can only run on a
/// machine somebody armed.
pub fn arm(state: &mut MotionSnap, armed: &ArmRecord, degraded: JointFlags) {
    let mut fresh = MotionSnapWire::new();
    core::mem::swap(state, fresh.clear_valid());
    state.mode = MotionMode::Holding;
    joints::write_vector(&mut state.last_goal, &armed.joints);
    traj::write_targets(
        &mut state.last_targets,
        &JointTargets {
            head_pose_body: armed.head_pose_body,
            body_yaw: armed.joints.body_yaw,
            antennas: armed.joints.antennas,
        },
    );
    record::write_pose(
        &mut state.fk_seed_pos,
        &mut state.fk_seed_quat,
        &armed.head_pose_body,
    );
    state.present_min_margin = armed.min_margin;
    state.masked = degraded;
}

/// Whether `state` describes a state a tick can be run on, which is what a host
/// picking a slot up asks before it runs one.
///
/// Called once per period, at the boundary, and never from inside the tick: what
/// it checks is everything the tick then reads as given — a mode, a move that
/// rebuilds, poses that are rotations, clocks that are lengths of time, and a
/// standing fault that names one.
///
/// # Errors
///
/// [`StateError`], one variant per way a slot's numbers can fail to describe a
/// state — including the case this exists for, a slot nothing has written,
/// whose zeroed mode names no mode.
// The path is rebuilt here to be judged and dropped, and `motion_tick` rebuilds
// it from the same bytes.
// TODO(resume-hands-back-the-path)
pub fn resume(state: &MotionSnap) -> Result<(), StateError> {
    match state.mode {
        MotionMode::None => return Err(StateError::NoMode),
        MotionMode::Holding => {}
        MotionMode::Moving => {
            duration_from_nanos(state.moving_elapsed.as_nanos())?;
        }
        MotionMode::Faulted => {
            fault::read(&state.fault)?;
        }
    }
    let path = traj::read_seed(&state.trajectory)?;
    // The mode and the path are set and cleared together, so the two ways they
    // can disagree are both states no tick produced.
    if path.is_none() && state.mode == MotionMode::Moving {
        return Err(StateError::MovingWithoutTrajectory);
    }
    if path.is_some() && state.mode == MotionMode::Holding {
        return Err(StateError::HoldingWithTrajectory);
    }
    // The count and the ring are one number and one buffer: the count is how
    // many of the ring's entries came off a driver, so it cannot exceed the
    // depth. Refused rather than trimmed, for the reason the pair above is.
    if usize::from(state.held_count) > RESPONSE_DEAD_SAMPLES {
        return Err(StateError::HeldCountPastTheRing);
    }
    if bool::from(state.prev_now_valid) {
        duration_from_nanos(state.prev_now.as_nanos())?;
    }
    let targets = traj::targets_of(&state.last_targets)?;
    let seed = record::read_pose(&state.fk_seed_pos, &state.fk_seed_quat)?;
    // The rotations are refused above by their own length, which no non-finite
    // component survives. The rest of the numbers are checked here: a
    // structurally valid slot carrying a NaN in one of them reads as a state
    // and behaves as a wedge.
    for (field, finite) in [
        (
            "last goal",
            joints::vector_of(&state.last_goal)
                .first_non_finite()
                .is_none(),
        ),
        ("last command set", targets.is_finite()),
        (
            "solver seed",
            seed.translation.vector.iter().all(|n| n.is_finite()),
        ),
        ("present margin", state.present_min_margin.is_finite()),
        (
            "modelled trajectory",
            state
                .tracking
                .iter()
                .flat_map(|run| {
                    [
                        run.generator,
                        run.predicted,
                        run.velocity,
                        run.anchor_present,
                        run.anchor_predicted,
                    ]
                })
                .all(f64::is_finite),
        ),
        (
            "held setpoint",
            state
                .held
                .iter()
                .all(|held| joints::vector_of(held).first_non_finite().is_none()),
        ),
        (
            "excursion allowance",
            state
                .excursion_cranks
                .iter()
                .chain([
                    &state.excursion_body_yaw,
                    &state.excursion_relative_yaw,
                    &state.excursion_cone,
                ])
                .all(|n| n.is_finite()),
        ),
    ] {
        if !finite {
            return Err(StateError::NonFinite(field));
        }
    }
    Ok(())
}

/// The goals last emitted, or the ones arming pinned if none have been.
#[must_use]
pub fn last_goal(state: &MotionSnap) -> JointVector {
    joints::vector_of(&state.last_goal)
}

/// The Cartesian mirror of [`last_goal`], which the next move starts from.
///
/// Total against a state [`resume`] has accepted: the only writers are arming's
/// solved record and a target the envelope check passed, both of which carry a
/// rotation, and a slot holding anything else is refused before a tick runs.
#[must_use]
pub fn last_targets(state: &MotionSnap) -> JointTargets {
    traj::targets_of(&state.last_targets).expect("a resumed state's command set is one")
}

/// The pose the next present-pose solve is seeded from. Total for the same
/// reason [`last_targets`] is.
#[must_use]
pub fn fk_seed(state: &MotionSnap) -> Isometry3<f64> {
    record::read_pose(&state.fk_seed_pos, &state.fk_seed_quat)
        .expect("a resumed state's solver seed is a pose")
}

/// The fault the machine stands parked on, or `None` where it stands on none.
///
/// Total for the same reason [`last_targets`] is: a faulted state whose numbers
/// name no fault is refused before a tick runs.
#[must_use]
pub fn standing_fault(state: &MotionSnap) -> Option<Fault> {
    (state.mode == MotionMode::Faulted)
        .then(|| fault::read(&state.fault).expect("a resumed state's standing fault is one"))
}

/// The instant a slot's count holds, for a state [`resume`] has accepted.
fn instant(count: SlotDuration) -> Duration {
    duration_from_nanos(count.as_nanos()).expect("a resumed state's clocks are lengths of time")
}

/// The count a slot holds `elapsed` in.
///
/// A length of time past what the count reaches is stored at the count's own
/// ceiling rather than refusing the tick: this is the loop's own clock, an
/// instant 292 years past its epoch is not one this machine runs on, and a
/// ceiling reached advances no move — the machine holds where it stands.
fn counted(elapsed: Duration) -> SlotDuration {
    SlotDuration::from_nanos(duration_nanos(elapsed).unwrap_or(i64::MAX))
}

/// Leave the machine holding the last goal that went out: no move, no clock.
fn hold(state: &mut MotionSnap) {
    state.mode = MotionMode::Holding;
    state.moving_elapsed = SlotDuration::from_nanos(0);
    traj::clear_seed(&mut state.trajectory);
}

/// Send the machine along `path`, from the start of the path's own clock.
///
/// At zero, so the accepting tick samples the move's own start; a replacement's
/// clock starts there too, from the setpoint the last tick commanded.
fn start_move(state: &mut MotionSnap, path: &Trajectory) {
    traj::write_seed(&mut state.trajectory, path);
    state.mode = MotionMode::Moving;
    state.moving_elapsed = SlotDuration::from_nanos(0);
}

/// Raise `fault`, leaving the machine in whatever state its response has to be
/// driven from. The fault itself travels on the report.
///
/// Two shapes, chosen by the fault itself. One stops commanding for good:
/// control is not trusted, so the only thing left is for the caller to cut
/// torque, and the path it stopped on is left exactly where it was. The other
/// abandons the move and holds the last goal that went out — a live state the
/// caller stows from, which is the whole difference between a machine that
/// cannot be commanded and one that must not be commanded *further*.
fn raise(state: &mut MotionSnap, fault: Fault, out: &mut TickOutputs) {
    if fault::latches(fault::kind(&fault)) {
        state.mode = MotionMode::Faulted;
        fault::write(&mut state.fault, &fault);
    } else {
        hold(state);
        // The wind-down starts measuring from where the machine now stands: the
        // runs open against a goal that is about to stop moving, and carrying
        // them into the stow would spend a window that was already half gone.
        tracking::clear(&mut state.tracking);
    }
    out.goal = None;
    out.report.mode = state.mode;
    out.report.fault = Some(fault);
}

/// Take `joints` out of service over `fault`, and carry on. Reports the fault
/// when joints are newly masked.
///
/// The move keeps running on what remains. Masked joints are commanded nothing
/// and checked for nothing from here on, including by the raise checks: entry
/// into the mask is the raise, so a servo already masked raises nothing however
/// long its error bits stay latched.
fn degrade(state: &mut MotionSnap, fault: Fault, joints: JointFlags, out: &mut TickOutputs) {
    for joint in flags::iter(joints) {
        if flags::insert(&mut state.masked, joint) {
            flags::insert(&mut out.report.newly_masked, joint);
        }
        tracking::forget(&mut state.tracking, joint);
    }
    out.report.masked = state.masked;
    out.report.degraded = Some(fault);
}

/// Take one servo out of service over `fault`, and abandon the move.
///
/// The mask entry and the torque-off it obliges the caller to write are the same
/// rule as a degrade; what differs is that a head servo leaving the group is not
/// something a move carries on through.
fn mask_and_raise(state: &mut MotionSnap, fault: Fault, joint: JointRef, out: &mut TickOutputs) {
    if flags::insert(&mut state.masked, joint) {
        flags::insert(&mut out.report.newly_masked, joint);
    }
    out.report.masked = state.masked;
    raise(state, fault, out);
}

/// Abandon the running move, and record why.
///
/// The offending sample goes nowhere and the trajectory is dropped, leaving the
/// machine holding the last goal that was commanded — a live state the next
/// command drives, which is what lets the caller stow under control.
fn abort(state: &mut MotionSnap, abort: MoveAbort, out: &mut TickOutputs) {
    hold(state);
    // The open runs go with the move, for the same reason a raise clears them:
    // the maneuver that answers this measures from where the machine now stands,
    // and a run already half spent would fault it early.
    tracking::clear(&mut state.tracking);
    out.goal = None;
    out.report.mode = state.mode;
    out.report.aborted = Some(abort);
}

/// What one period of the tracking comparison found.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TrackingLook {
    /// How far each joint stands from its own prediction, radians, in bus
    /// order. A row whose prediction is not seeded reports zero: nothing was
    /// measured about it.
    pub residuals: [f64; ROW_COUNT],
    /// The joints whose window ran out on this period, neither closing on the
    /// prediction nor pacing it.
    pub exhausted: JointFlags,
    /// The longest open run afterwards, periods.
    pub longest: u32,
}

/// Each joint's modelled trajectory and open run, as the state holds them.
///
/// The tracking comparison, carried across periods: one prediction and one open
/// run per joint, stepped by [`predict`](tracking::predict) every period and
/// judged by [`look`](tracking::look) on every live read. Public because it is
/// the guard — a recorded run replayed through these functions is judged by
/// exactly what the tick judges a live one by, so a threshold or a pace
/// fraction that would false-trip a gesture the machine is known to make well
/// can be caught away from the machine.
///
/// The two halves of an entry have different lifetimes. The prediction is plant
/// state: it is seeded once from a reading, stepped from the setpoints the
/// driver held whether the joint is masked or judged or neither, and survives
/// every raise, degrade and abort, because none of those stop the servo running
/// its own generator. The run is evidence about one stretch of one goal, and
/// [`clear`](tracking::clear) or [`forget`](tracking::forget) drops it whole:
/// what a joint was doing against a goal that has been abandoned says nothing
/// about what it does against the next one.
///
/// A run's two anchors — the reading and the prediction as they stood when it
/// opened or last restarted — are what the judgement is made from, rather than
/// the difference between consecutive ticks, so a joint creeping less than one
/// count per tick still shows its motion over the window.
pub mod tracking {
    use super::{
        GroupPlants, JointFlags, JointRef, JointVector, Predicted, ROW_COUNT, TrackingFaultConfig,
        TrackingLook, TrackingRowSnap, flags, outside_limit, row,
    };

    /// The nine entries, in bus order.
    pub type Runs = [TrackingRowSnap; ROW_COUNT];

    /// Drop the open run, keeping the prediction.
    ///
    /// Every caller means the same thing by it: the evidence is about a goal
    /// that no longer applies. The joint's generator is unaffected, so
    /// `predicted`, `velocity` and `seeded` are left exactly where the last
    /// step put them.
    fn forget_run(run: &mut TrackingRowSnap) {
        run.active = false.into();
        run.count = 0;
        run.anchor_present = 0.0;
        run.anchor_predicted = 0.0;
    }

    /// Forget every open run.
    pub fn clear(runs: &mut Runs) {
        for run in runs {
            forget_run(run);
        }
    }

    /// Forget one joint's open run, for a joint leaving service.
    pub fn forget(runs: &mut Runs, joint: JointRef) {
        if let Some(index) = row(joint) {
            forget_run(&mut runs[index]);
        }
    }

    /// The longest open run across the nine joints, or zero when none is open.
    ///
    /// The single number a report carries about nine independent runs: the one
    /// closest to raising the fault.
    #[must_use]
    pub fn longest(runs: &Runs) -> u32 {
        runs.iter()
            .filter(|run| bool::from(run.active))
            .map(|run| run.count)
            .max()
            .unwrap_or(0)
    }

    /// Advance every seeded row's modelled trajectory by one period, toward the
    /// setpoint `held` the driver was holding [`RESPONSE_DEAD_SAMPLES`] periods
    /// before the sample being predicted.
    ///
    /// Masked rows included. The model is the plant's and not the plan's: a
    /// joint nobody is judging is still a joint whose servo is running its
    /// generator, and a row that re-enters service mid-session is re-seeded
    /// from a reading rather than trusted to have kept up.
    ///
    /// An unseeded row is not stepped. There is nothing to step: a prediction
    /// starts from where the joint was measured, never from where a goal says
    /// it should be.
    pub fn predict(plant: &GroupPlants, held: &JointVector, runs: &mut Runs) {
        for (run, (joint, target)) in runs.iter_mut().zip(held.joints()) {
            if !bool::from(run.seeded) {
                continue;
            }
            let mut state = Predicted {
                generator: run.generator,
                velocity: run.velocity,
                position: run.predicted,
            };
            // Keyed by the joint the iterator is already yielding rather than
            // by its position: the class a joint is judged against is a fact
            // about the joint, and a positional index would rest on an
            // unstated agreement about the order they come in.
            plant.for_joint(joint).step(&mut state, target);
            run.generator = state.generator;
            run.predicted = state.position;
            run.velocity = state.velocity;
        }
    }

    /// Seed the rows that have no prediction yet from a live reading: the joint
    /// is where it is, and its generator's trajectory starts there at rest with
    /// the loop standing on it.
    ///
    /// Called on every live read, so a row that has just been told to start
    /// over takes this tick's own measurement. Rows already seeded are left
    /// alone.
    pub fn seed(runs: &mut Runs, present: &JointVector) {
        for (run, (_, angle)) in runs.iter_mut().zip(present.joints()) {
            if bool::from(run.seeded) {
                continue;
            }
            run.generator = angle;
            run.predicted = angle;
            run.velocity = 0.0;
            run.seeded = true.into();
        }
    }

    /// Throw every prediction away, so that the next live read seeds them all
    /// afresh.
    ///
    /// For the stretches where the model has nothing to chase — the driver
    /// holding no setpoint, or a gap of samples too long to step through — after
    /// which where the generator stands is not something arithmetic knows.
    ///
    /// The open runs are left alone here, and [`look`] drops the ones it can no
    /// longer judge on the next live read. A tick that measured nothing may not
    /// end a run: a run is a claim about live reads, and one already standing
    /// keeps standing until a reading either closes it or extends it.
    pub fn reseed(runs: &mut Runs) {
        for run in runs {
            run.seeded = false.into();
            run.generator = 0.0;
            run.predicted = 0.0;
            run.velocity = 0.0;
        }
    }

    /// Judge every row's live reading against its own prediction, and say what
    /// ran out.
    ///
    /// Only for live reads: a stale reading judged against a prediction that
    /// has moved on is a difference nobody measured, and a caller that skips
    /// the period leaves every run exactly where it was. The predictions are
    /// stepped either way, by [`predict`].
    pub fn look(
        cfg: &TrackingFaultConfig,
        masked: JointFlags,
        present: &JointVector,
        runs: &mut Runs,
    ) -> TrackingLook {
        let mut residuals = [0.0; ROW_COUNT];
        let mut exhausted = JointFlags::NONE;
        for (index, (id, angle)) in present.joints().into_iter().enumerate() {
            let run = &mut runs[index];
            if flags::contains(masked, id) || !bool::from(run.seeded) {
                // A masked joint is judged about nothing, and a row with no
                // prediction has nothing to be judged against. Neither is a
                // measurement of zero: `residuals` carries zero because there
                // is no figure.
                forget_run(run);
                continue;
            }
            let residual = (angle - run.predicted).abs();
            residuals[index] = residual;
            if !outside_limit(residual, cfg.threshold_rad) {
                // Inside the threshold is healthy, whatever came before it.
                forget_run(run);
                continue;
            }
            let count = if bool::from(run.active) {
                // The gap the run opened on, and the two ways it is still a
                // healthy joint. Closing is measured against the prediction: a
                // joint behind its generator but catching up on it — a tired
                // servo finishing a move the generator has already finished —
                // regains ground it will not have to regain twice. Pacing is
                // measured against the modelled shaft's own motion -- the
                // lagged output the residual is read against, and not the
                // trajectory generator standing ahead of it: a joint that is
                // behind and falling further behind, but still covering
                // `pace_min` of the ground the model covers, is a slow servo
                // and not an obstructed one. A jammed joint moves nothing and
                // paces nothing, however far the model goes.
                let opened = (run.anchor_predicted - run.anchor_present).abs();
                let closing = opened - residual >= cfg.progress_min_rad;
                let shaft = run.predicted - run.anchor_predicted;
                let joint = angle - run.anchor_present;
                let pacing = shaft.abs() >= cfg.progress_min_rad
                    && joint * shaft.signum() >= cfg.pace_min * shaft.abs();
                if closing || pacing {
                    restart(run, angle);
                } else {
                    run.count += 1;
                }
                run.count
            } else {
                restart(run, angle);
                run.count
            };
            if count >= cfg.ticks {
                flags::insert(&mut exhausted, id);
            }
        }
        TrackingLook {
            residuals,
            exhausted,
            longest: longest(runs),
        }
    }

    /// Open a run at where the joint stands and where its prediction stands.
    ///
    /// The pair is the whole record of the run: how far apart the two were when
    /// it opened, and where each has got to since.
    fn restart(run: &mut TrackingRowSnap, angle: f64) {
        run.active = true.into();
        run.count = 1;
        run.anchor_present = angle;
        run.anchor_predicted = run.predicted;
    }
}

/// How many control periods a sample covers: the gap since the previous one,
/// on the grid, floored at one.
///
/// A sample that arrived early or on time covers its own period and nothing
/// more; one that arrived two periods late covers the slot nobody attended as
/// well. Uncapped here — the caller decides what to do with a gap longer than
/// the model will be walked through.
fn grid_periods(elapsed: Duration, period: Duration) -> usize {
    let period = period.as_nanos();
    if period == 0 {
        return 1;
    }
    usize::try_from(elapsed.as_nanos() / period)
        .unwrap_or(usize::MAX)
        .max(1)
}

/// The oldest setpoint in the ring: the one the driver was holding
/// [`RESPONSE_DEAD_SAMPLES`] periods before the sample being predicted.
fn oldest_held(state: &MotionSnap) -> JointVector {
    joints::vector_of(&state.held[0])
}

/// The newest, which is what the servos went on chasing through a period no
/// sample attended: the driver wrote nothing in a slot the loop missed.
fn newest_held(state: &MotionSnap) -> JointVector {
    joints::vector_of(&state.held[RESPONSE_DEAD_SAMPLES - 1])
}

/// Push one period's held setpoint into the ring, dropping the oldest.
///
/// Once per period stepped, and always after the step that read the ring, which
/// is what keeps the oldest entry `RESPONSE_DEAD_SAMPLES` periods old.
fn push_held(state: &mut MotionSnap, setpoint: &JointVector) {
    for slot in 1..RESPONSE_DEAD_SAMPLES {
        state.held.swap(slot - 1, slot);
    }
    joints::write_vector(&mut state.held[RESPONSE_DEAD_SAMPLES - 1], setpoint);
    state.held_count = u8::try_from(RESPONSE_DEAD_SAMPLES)
        .unwrap_or(u8::MAX)
        .min(state.held_count.saturating_add(1));
}

/// Step every row's modelled trajectory through the periods this sample covers,
/// and take up the setpoint it reports being held.
///
/// One period is one step-then-push: the step reads the setpoint of
/// `RESPONSE_DEAD_SAMPLES` periods ago and the push takes up this period's,
/// which is the dead time the model is measured at. A period no sample attended
/// is stepped on the setpoint the driver was already holding, because that is
/// what the servos went on chasing.
///
/// Three things put a row back to being seeded from the next reading instead:
/// a ring that does not yet hold a full dead time of setpoints, a driver
/// holding no setpoint at all, and a gap longer than the model is walked
/// through. Each is a stretch where the generator's position is not something
/// arithmetic knows, and a gap that long is the read-loss fault's business
/// rather than this comparison's.
///
/// A setpoint carrying a number nobody can place is a driver holding nothing,
/// for the reason a reading carrying one is not a reading: it says something
/// about the layer that produced it and nothing about the machine, and a single
/// one of them costs a period of prediction rather than poisoning the slot the
/// next execution has to resume from.
fn advance_prediction(
    cfg: &MotionConfig,
    state: &mut MotionSnap,
    inp: &TickInputs<'_>,
    periods: usize,
) {
    let gap = periods > MAX_GAP_PERIODS;
    if gap || usize::from(state.held_count) < RESPONSE_DEAD_SAMPLES {
        tracking::reseed(&mut state.tracking);
        if gap {
            // The ring goes with the prediction, and only on a gap: its
            // setpoints are from before it, and a prediction seeded from this
            // sample's reading has no business being stepped toward a setpoint
            // of arbitrary age. The next periods fill it again, and every row
            // re-seeds until they have. A ring that is merely not full yet is
            // filling, and clearing that one would stop it ever filling.
            state.held_count = 0;
        }
    } else {
        for period in 0..periods {
            let held = oldest_held(state);
            tracking::predict(&cfg.plant, &held, &mut state.tracking);
            if period + 1 < periods {
                let carried = newest_held(state);
                push_held(state, &carried);
            }
        }
    }
    match inp
        .commanded
        .filter(|setpoint| setpoint.first_non_finite().is_none())
    {
        Some(setpoint) => push_held(state, setpoint),
        None => {
            state.held_count = 0;
            tracking::reseed(&mut state.tracking);
        }
    }
}

/// The joint furthest out, how far, and the count its run had reached.
fn worst_run(deviations: &[f64; ROW_COUNT], runs: &tracking::Runs) -> (JointRef, f64, u32) {
    let row = worst_row(deviations);
    (ROWS[row], deviations[row], runs[row].count)
}

/// One control step.
///
/// `out` is overwritten in full, so a caller may reuse one buffer forever and
/// never see a field left over from a previous period.
pub fn motion_tick(
    cfg: &MotionConfig,
    state: &mut MotionSnap,
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
    let elapsed = if bool::from(state.prev_now_valid) {
        inp.now.saturating_sub(instant(state.prev_now))
    } else {
        Duration::ZERO
    };
    let advance = elapsed.min(inp.period);
    // How many periods this sample covers, off the same subtraction. One is a
    // sample on the grid; more is a loop that lost slots, and the prediction
    // has to be walked through the ones nobody attended. A first tick covers
    // one period, because there is no previous instant to have missed any.
    let periods = if bool::from(state.prev_now_valid) {
        grid_periods(elapsed, inp.period)
    } else {
        1
    };
    state.prev_now = counted(inp.now);
    state.prev_now_valid = true.into();
    if state.mode == MotionMode::Moving {
        state.moving_elapsed = counted(instant(state.moving_elapsed).saturating_add(advance));
    }

    out.report.mode = state.mode;
    out.report.present_min_margin = state.present_min_margin;
    out.report.masked = state.masked;

    // The modelled plant, stepped before anything is judged and on every tick
    // there is: a read that failed does not stop the servo running its own
    // generator, and neither does a fault this layer is holding on. What the
    // comparison below needs is where the generator stands now, whether or not
    // this sample said where the joint is.
    advance_prediction(cfg, state, inp, periods);

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
    // Classified once, at the raise: the standing fault is repeated in every
    // report from here on, and a reader that narrates every report would grow
    // its record at tick rate.
    if let Some(fault) = standing_fault(state) {
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
                &fk_seed(state),
                &cfg.fk,
                &mut pose,
            ) {
                Ok(stats) => {
                    state.pose_failures = 0;
                    out.report.present_fresh = true;
                    out.report.fk = Some(stats);
                    record::write_pose(&mut state.fk_seed_pos, &mut state.fk_seed_quat, &pose);
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
                        raise(
                            state,
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
                raise(
                    state,
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
    out.report.tracking_count = tracking::longest(&state.tracking);

    // Tracking, on live reads only: a stale reading judged against a prediction
    // that has moved on is a difference nobody measured, so a stale tick
    // freezes every joint's run where it stands rather than growing or
    // clearing it. The predictions themselves were stepped above, on this tick
    // like every other.
    if let Some(present) = fresh {
        // Rows with no prediction yet take this reading as theirs. A live read
        // is the only thing a prediction is ever seeded from — a goal says
        // where the joint was asked to be, which is the question, not the
        // baseline.
        tracking::seed(&mut state.tracking, present);
        let masked = state.masked;
        let look = tracking::look(&cfg.tracking, masked, present, &mut state.tracking);
        out.report.tracking_residuals = Some(look.residuals);
        // Recorded before the fault check: the tick that runs a window out is
        // the one whose figure matters most, and a report that shipped zero
        // there would read as a single-tick trip rather than a sustained one.
        out.report.tracking_count = look.longest;

        // The rows whose window ran out, by how far out they are, kept per
        // group because the two groups are answered differently. Every other
        // row holds a value no measurement can beat, so the same worst-of sweep
        // names the joint among them.
        //
        // Only when the detector is armed. Disarmed, the look above still ran
        // and its figures are still reported; an expired window then buys a
        // figure in the record and nothing else.
        if cfg.tracking.armed {
            let mut head_out = [f64::NEG_INFINITY; ROW_COUNT];
            let mut antennas_out = [f64::NEG_INFINITY; ROW_COUNT];
            let mut head_exhausted = false;
            let mut antennas_exhausted = false;
            for (row, id) in ROWS.into_iter().enumerate() {
                if !flags::contains(look.exhausted, id) {
                    continue;
                }
                if group_of(id) == Some(JointGroup::Antennas) {
                    antennas_out[row] = look.residuals[row];
                    antennas_exhausted = true;
                } else {
                    head_out[row] = look.residuals[row];
                    head_exhausted = true;
                }
            }
            // The head decides the tick when both groups run out together: its
            // answer winds the whole machine down, which subsumes taking the
            // antennas out of service.
            //
            // The count comes off the run itself, read here while it still
            // stands: `raise` and `degrade` both forget the run, and the count
            // is the window that judged it.
            if head_exhausted {
                let (joint, error, count) = worst_run(&head_out, &state.tracking);
                raise(
                    state,
                    Fault::HeadObstructed {
                        joint,
                        error,
                        count,
                    },
                    out,
                );
                return;
            }
            if antennas_exhausted {
                let (joint, error, count) = worst_run(&antennas_out, &state.tracking);
                degrade(
                    state,
                    Fault::AntennaObstructed {
                        joint,
                        error,
                        count,
                    },
                    JointGroup::Antennas.joints(),
                    out,
                );
            }
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
            let joint = ROWS[row];
            if flags::contains(state.masked, joint) || servo.healthy_or_voltage_only() {
                continue;
            }
            let worst = if group_of(joint) == Some(JointGroup::Antennas) {
                &mut antenna_bad
            } else {
                &mut head_bad
            };
            worst.get_or_insert((joint, *servo));
        }
        if let Some((joint, servo)) = head_bad {
            mask_and_raise(
                state,
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
            degrade(
                state,
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
        let disposition = take_command(cfg, state, command, out);
        out.report.command = disposition;
    }
    out.report.mode = state.mode;

    // Sample the active trajectory at the move's own elapsed time. Everything is
    // copied out of the trajectory here so the borrow ends before anything can
    // fault.
    if state.mode != MotionMode::Moving {
        return;
    }
    let t = instant(state.moving_elapsed);
    let path = match traj::read_seed(&state.trajectory) {
        Ok(Some(path)) => path,
        // Moving with nothing to sample is a state no sequence of ticks
        // produces: the mode and the seed are written and cleared together, and
        // a seed that is no path is refused before a tick runs. If one ever
        // appears anyway, the machine drops to a named state instead of staying
        // Moving forever — where it would emit nothing, refuse every command as
        // already-moving, and report no reason. Which way the state disagreed
        // with itself is reported, so the mode change is not the whole trace.
        unsampleable => {
            out.report.unsampleable = Some(match unsampleable {
                Err(seed) => StateError::Seed(seed),
                _ => StateError::MovingWithoutTrajectory,
            });
            hold(state);
            out.goal = None;
            out.report.mode = state.mode;
            return;
        }
    };
    let mut sampled = JointTargets::default();
    path.sample(t, &mut sampled);
    let (endpoint, done) = (*path.target(), path.done(t));

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

    match stage_target(cfg, state, &sampled, Judged::AgainstTheStart, out) {
        Staged::Envelope(violations) => {
            abort(state, MoveAbort::EnvelopePath(violations), out);
            return;
        }
        Staged::Admitted(candidate) => match step_over(cfg, state, &candidate) {
            Some(StepTooFar { joint, delta }) => {
                abort(state, MoveAbort::StepTooLarge { joint, delta }, out);
                return;
            }
            None => command(state, out, &sampled, &candidate),
        },
    }

    if done {
        // The endpoint's own bits, so the next move chains from exactly what
        // was commanded rather than from a sample near it.
        traj::write_targets(&mut state.last_targets, &endpoint);
        hold(state);
        out.report.completed = true;
    }
    out.report.mode = state.mode;
}

/// Which of the two command paths a candidate pose arrived on.
///
/// The split decides one thing: a pose the envelope refuses is judged on the
/// bounds alone for a tracked setpoint and against the excursion the move
/// started from for a sample, which is how a machine standing outside the
/// envelope travels back inside. The per-tick step guard is not on this axis —
/// it is a question the sampled path asks of an admitted pose ([`step_over`])
/// and the tracked path never asks, so there is no step outcome for a tracked
/// setpoint to have to answer for.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Judged {
    /// A tracked setpoint: judged on the bounds alone.
    OnTheBounds,
    /// A sample of a move this crate planned, judged against the excursion the
    /// machine started that move from.
    AgainstTheStart,
}

/// What became of one candidate pose at the envelope gate.
enum Staged {
    /// It passed, and these are the joint angles it means.
    Admitted(JointVector),
    /// The envelope refused it.
    Envelope(EnvelopeViolations),
}

/// A joint asked to move further in one period than it may.
struct StepTooFar {
    /// Which joint.
    joint: JointRef,
    /// How far it was asked to travel, radians.
    delta: f64,
}

/// Put one pose the machine is about to be commanded to through the envelope
/// gate, and hand back the joint angles it solved to if it passes.
///
/// The envelope gate is one rule for both paths: a sampled move and a tracked
/// setpoint ask the same question of the same machine, and a geometry check that
/// meant one thing on one path and another on the other is the divergence the
/// single gate exists to prevent. What differs on a failure is the answer, which
/// is the caller's: a sample aborts the move it came from, a setpoint is refused
/// to whoever sent it. What a caller does with an admitted pose is the caller's
/// too — the sampled path bounds its step ([`step_over`]) before commanding it,
/// and the tracked path commands it as asked.
fn stage_target(
    cfg: &MotionConfig,
    state: &mut MotionSnap,
    target: &JointTargets,
    judged: Judged,
    out: &mut TickOutputs,
) -> Staged {
    let mut envelope = EnvelopeReport::default();
    let verdict = check_envelope(
        &cfg.geom,
        &cfg.env,
        &target.head_pose_body,
        target.body_yaw,
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
    // A tracked setpoint gets no such allowance: it is a fresh ask every period
    // with no start of its own to be judged against, and a machine standing
    // outside the envelope is recovered by commanding a move.
    //
    // The second arm is a verdict that passed without producing the angles it
    // passed on, and a pose with no angles is nothing this can command.
    let recovering = verdict.is_err();
    let admitted = match verdict {
        Ok(()) => true,
        Err(error) => {
            judged == Judged::AgainstTheStart
                && !error.violations.margin
                && Excursion::of(&cfg.env, &envelope, target.body_yaw).no_further_out_than(state)
        }
    };
    let (true, Some(angles)) = (admitted, envelope.leg_angles) else {
        return Staged::Envelope(envelope.violations);
    };
    out.report.recovering = recovering;
    let candidate = JointVector {
        body_yaw: target.body_yaw,
        legs: angles.0,
        antennas: target.antennas,
    };

    Staged::Admitted(candidate)
}

/// The joint an admitted pose would move further than a period allows, if there
/// is one.
///
/// An oversized step is a slam, and the bug that produced it is the thing worth
/// reporting; it is never trimmed and sent. A move whose clock is too short for
/// its span does not reach here: `floor_move_clock` right-sizes it before it is
/// commanded, so what remains for this to catch is an interpolator or a seed
/// that is wrong on a path this crate planned. Only the sampled path asks, which
/// is why the question is a call of its own: a tracked setpoint is content, sent
/// as asked, and has no answer to give.
///
/// A masked joint is skipped: nothing of what the plan says about it goes
/// anywhere, so there is no step to bound. The plan itself is left whole — the
/// mask decides what reaches the wire, not what the trajectory says.
fn step_over(
    cfg: &MotionConfig,
    state: &MotionSnap,
    candidate: &JointVector,
) -> Option<StepTooFar> {
    for ((id, angle), (_, last)) in candidate
        .joints()
        .into_iter()
        .zip(last_goal(state).joints())
    {
        if flags::contains(state.masked, id) {
            continue;
        }
        let delta = (angle - last).abs();
        if outside_limit(delta, cfg.max_step.for_joint(id)) {
            return Some(StepTooFar { joint: id, delta });
        }
    }
    None
}

/// Command an admitted pose: this period's goal, and the state it leaves behind.
///
/// `target` is the pose as it was asked for, which is what the next move chains
/// from; `candidate` is the joint angles it solved to, which is what goes to the
/// servos.
fn command(
    state: &mut MotionSnap,
    out: &mut TickOutputs,
    target: &JointTargets,
    candidate: &JointVector,
) {
    let mut changed = false;
    for ((id, angle), (_, last)) in candidate
        .joints()
        .into_iter()
        .zip(last_goal(state).joints())
    {
        if flags::contains(state.masked, id) {
            continue;
        }
        changed |= angle != last;
    }

    // Emit, but only what changed: holding writes nothing and the servos hold.
    // A period whose only movement is on a masked joint has nothing to say.
    out.report.emitted = changed;
    if changed {
        out.goal = Some(*candidate);
    }
    joints::write_vector(&mut state.last_goal, candidate);
    traj::write_targets(&mut state.last_targets, target);
}

/// Take at most one command, returning what became of it. Mutates the state
/// only when the command is accepted.
///
/// `out` is written by the tracked-setpoint arm alone, which is the one command
/// that both decides and emits within the period it arrives on; every other arm
/// leaves the goal to the sampling below.
fn take_command(
    cfg: &MotionConfig,
    state: &mut MotionSnap,
    command: &MotionCommand,
    out: &mut TickOutputs,
) -> CommandDisposition {
    let (target, durations, warp) = match command {
        MotionCommand::MoveTo {
            target,
            durations,
            warp,
        } => (target, *durations, *warp),
        MotionCommand::Track(target) => return take_track(cfg, state, target, out),
        MotionCommand::Hold => {
            hold(state);
            return CommandDisposition::Held;
        }
    };

    // Start from the last commanded targets rather than the measured pose, so
    // consecutive moves chain without a step and a tracking lag is not written
    // into the path. Mid-move that is the setpoint the previous tick commanded,
    // which is what makes a retarget a splice rather than a jump.
    let retarget = state.mode == MotionMode::Moving;
    let start = last_targets(state);
    match shape_move(
        cfg,
        &start,
        target,
        durations,
        warp,
        Some(state.present_min_margin),
    ) {
        Ok(trajectory) => {
            // How far outside the envelope this move begins, which is the
            // allowance every sample of it is judged against. Recomputed per
            // command, so a retarget mid-recovery tightens it to wherever the
            // machine has got to.
            start_excursion(cfg, &start).store(state);
            start_move(state, &trajectory);
            if retarget {
                CommandDisposition::Retargeted
            } else {
                CommandDisposition::Started
            }
        }
        Err(rejection) => CommandDisposition::Rejected(rejection),
    }
}

/// The trajectory a move from `start` to `target` runs, or why it is refused.
///
/// The construction order the whole stack shapes a move through, in one place:
/// resolve the antenna directions against where the last command left them,
/// judge the *resolved* target against the envelope, and shape the path to it.
/// Nothing here touches state, so a caller planning a move it will drive
/// itself gets exactly the path [`take_command`] would have built.
///
/// `margin_baseline` is the toggle margin of the pose the machine presently
/// holds, which is what admits a lift off a rest tighter than the clearance
/// floor. A caller with no live read passes `None` and is held to the floor
/// itself.
fn shape_move(
    cfg: &MotionConfig,
    start: &JointTargets,
    target: &JointTargets,
    durations: MoveDurations,
    warp: WarpKind,
    margin_baseline: Option<f64>,
) -> Result<Trajectory, CommandRejection> {
    let target = resolve_antennas(start, target)?;

    let mut report = EnvelopeReport::default();
    check_envelope(
        &cfg.geom,
        &cfg.env,
        &target.head_pose_body,
        target.body_yaw,
        margin_baseline,
        &mut report,
    )
    .map_err(|error| CommandRejection::Envelope(error.violations))?;

    Trajectory::new(start, &target, durations, warp).map_err(CommandRejection::Trajectory)
}

/// The path a move from `start` would run, on a clock long enough to carry it,
/// without commanding anything.
///
/// What a caller driving its own tick-by-tick composition needs and cannot
/// assemble itself: the antenna resolution that routes each side away from its
/// outboard direction is private to this module, and a trajectory built from
/// [`floor_move_clock`]'s deliberately *unresolved* target would sweep the
/// short arc straight through the point that resolution exists to miss. So the
/// whole construction lives here — resolve, floor the clock, shape the path to
/// the resolved target — and a caller that samples the result and hands each
/// sample back as [`MotionCommand::Track`] moves exactly as the same command
/// through [`take_command`] would have.
///
/// The returned [`ClockStretch`] is the caller's to report, as it is for a
/// commanded move. A refusal is a plan this machine will not run — an antenna
/// direction no servo count reaches, a target outside the envelope, a move
/// nothing can shape — and, per the fault doctrine, a refused plan is not a
/// fault: nothing here parks, latches, or touches torque.
///
/// `margin_baseline` is the state's own `present_min_margin` for a caller with
/// a live read of the machine it is planning for.
pub fn plan_move(
    cfg: &MotionConfig,
    start: &JointTargets,
    target: &JointTargets,
    durations: MoveDurations,
    warp: WarpKind,
    tick_hz: f64,
    margin_baseline: Option<f64>,
) -> Result<(Trajectory, Option<ClockStretch>), CommandRejection> {
    let asked = MotionCommand::MoveTo {
        target: *target,
        durations,
        warp,
    };
    let (floored, stretch) = floor_move_clock(cfg, start, &asked, tick_hz);
    let MotionCommand::MoveTo {
        durations: effective,
        ..
    } = floored
    else {
        unreachable!("flooring a move returns a move")
    };
    let trajectory = shape_move(cfg, start, target, effective, warp, margin_baseline)?;
    Ok((trajectory, stretch))
}

/// Take one tracked setpoint: check it as a sampled path pose is checked, and
/// command it on this tick.
///
/// The refusals are geometry, answered to the caller rather than written into
/// the machine's mode: an antenna angle no goal register holds, and a pose the
/// envelope will not have. How far the setpoint moves in one period is not one
/// of them — that bounds a plan, and this is the caller's. Neither refusal is a
/// fault and neither changes anything — the mask, the mode and the last goal are
/// exactly what they were, so a caller whose composition went out of bounds
/// drops it and carries on from a machine that never moved.
fn take_track(
    cfg: &MotionConfig,
    state: &mut MotionSnap,
    target: &JointTargets,
    out: &mut TickOutputs,
) -> CommandDisposition {
    // The antennas are absolute here rather than directions to resolve, so what
    // is left to check is that each angle is one the goal register can hold.
    //
    // A masked antenna is skipped, on the same rule the step guard keeps: what
    // is masked never reaches the wire, so no register has to hold it. Checking
    // one anyway would let a degraded antenna refuse every composed target a
    // clip produces — and a refused composition drops every overlay — over a
    // count nothing was ever going to write.
    for (side, joint) in [JointRef::AntennaRight, JointRef::AntennaLeft]
        .into_iter()
        .enumerate()
    {
        if flags::contains(state.masked, joint) {
            continue;
        }
        let angle = target.antennas[side];
        if outside_limit(angle, ANTENNA_GOAL_MAX_RAD) || below_limit(angle, ANTENNA_GOAL_MIN_RAD) {
            return CommandDisposition::Rejected(CommandRejection::AntennaUnreachable {
                joint,
                angle,
            });
        }
    }

    match stage_target(cfg, state, target, Judged::OnTheBounds, out) {
        Staged::Envelope(violations) => {
            CommandDisposition::Rejected(CommandRejection::Envelope(violations))
        }
        // Accepted, and commanded without asking the step question: a tracked
        // setpoint is content, and there is no plan of ours to bound. Any move
        // in flight is over — its samples and this setpoint cannot both be what
        // the servos are holding — and the machine is holding wherever this
        // period puts it.
        Staged::Admitted(candidate) => {
            command(state, out, target, &candidate);
            hold(state);
            CommandDisposition::Tracked
        }
    }
}

/// How many samples the dry pass takes per nominal control period.
///
/// The live loop steps a move one period at a time from whatever phase it woke
/// on, so the step that has to stay inside the bound is the largest over *every*
/// window of one period's width the path holds — not the largest between two
/// points of the one grid that starts where the move does. Sampling this much
/// finer and maximising over every period-wide window of the fine series
/// measures that directly, at every phase the fine grid can express, the
/// inverse kinematics included: the fine samples go through the same solve the
/// coarse ones did.
///
/// Eight puts the residual — the phases *between* two fine samples — at a
/// sixteenth of a period, and what that residual is worth is
/// [`grid_headroom`]. The cost is eight times the solves per pass, off the
/// control loop and inside [`MAX_DRY_SAMPLES`].
///
/// The same factor at every clock length, deliberately, and not only where the
/// second-order term is loosest. Two reasons, one of them measured. The
/// allowance is derived from the grid that was actually walked, so a factor that
/// varied with the clock would step the allowance by its square at the length it
/// changed at — granting a *longer* clock sixty-four times more slack than the
/// one just below it, which makes the predicate non-monotone in duration exactly
/// where the stretch iteration is walking. And the saving is not worth buying
/// that with: one pass over the shipped 0.8 s gesture at 50 Hz — 320 fine
/// samples, each through the leg solve — measures 86 µs on the bench host, so
/// the four passes a stretch may take are under a fiftieth of one control
/// period, on a path that runs between periods rather than inside one.
const DRY_OVERSAMPLE: u32 = 8;

/// The fine samples one nominal period is worth, as a walk's extension past the
/// sample that lands the endpoint.
const ONE_PERIOD_BEYOND: u32 = DRY_OVERSAMPLE;

/// No samples past the sample that lands the endpoint.
///
/// What a walk measuring a crossing offset takes: the pair's phase is a time the
/// tips pass each other on the way, and nothing about the pose held afterwards
/// says anything further about it.
const NO_EXTENSION: u32 = 0;

/// The headroom a clock carries past the step its dry pass measured, as a
/// multiple of one over the square of the fine samples the clock spans.
///
/// What is left for a headroom to cover, once the pass maximises over every
/// window the fine grid expresses, is the wake phases that fall between two
/// fine samples — at most half a fine step from the window the pass measured.
/// The deficit there is second order in that distance: `f'` loses fifteen times
/// the square of the distance from its peak, so it is fifteen quarters over
/// 1.875 of one over the fine samples squared, and this numerator is that with a
/// factor of two over it.
///
/// Two things need exactly that headroom. A loop running late resumes its
/// periods at a shifted phase, and a shifted grid can land nearer the peak than
/// any fine sample did — so a clock accepted at the measured maximum alone,
/// whether it was stretched to there or asked for there, leaves the guard to
/// fault on a healthy move at an unlucky wake phase. And the stretch iteration
/// needs somewhere to terminate: scaling a clock by the measured ratio refines
/// its grid in the same motion, so the sequence approaches the bound from above,
/// and a target exactly at the bound is one it reaches only in the limit.
///
/// A term of this shape is the opposite of a jitter allowance: it does not grow
/// with how late a loop runs, it grows with how coarsely the path is sampled,
/// and it vanishes as the clock lengthens. It is derived at every clock length
/// — nothing caps it, so no clock is accepted on an allowance that was asserted
/// rather than measured. On the fold's floor it is two parts in ten thousand,
/// and over a clock spanning as little as two periods it is under two parts in
/// a hundred.
const STRETCH_GRID_HEADROOM: f64 = 4.0;

/// The headroom a clock spanning `periods` periods needs over the step its dry
/// pass measured.
///
/// The one place the term is computed, so what a clock is judged against and
/// what a stretch lands on are the same number: a clock accepted with less
/// headroom than a stretch would have given it is a clock the live grid's own
/// phase can walk past the guard.
///
/// Infinite for a clock spanning no periods at all: nothing about such a clock
/// was sampled, so nothing about it is acceptable. No caller reaches it — a
/// non-positive duration has no trajectory to measure and is answered long
/// before this.
fn grid_headroom(periods: f64) -> f64 {
    let samples = periods * f64::from(DRY_OVERSAMPLE);
    if samples > 0.0 {
        STRETCH_GRID_HEADROOM / (samples * samples)
    } else {
        f64::INFINITY
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
/// Two hundred and fifty seconds of move at fifty hertz, [`DRY_OVERSAMPLE`]
/// samples to the period. Any span this machine can travel fits inside the step
/// bounds on a clock far shorter than that, so a duration past this is one there
/// is nothing left to measure about.
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
    /// Whether any clock here was lengthened to carry its own span inside the
    /// per-tick step bounds. Distinct from `dephased`, and the two combine: a
    /// clock lengthened for its span and then de-phased says both, and a caller
    /// that reads only `dephased` cannot tell such a move from a pair the pass
    /// merely parted.
    pub span_stretched: bool,
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
    /// The same test a stretch is sized to satisfy. The dry pass walks every
    /// wake phase its fine grid expresses and the live loop wakes wherever it
    /// wakes, so a clock is accepted only with the room its own length says the
    /// phases between two fine samples need.
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
/// This measures the candidate path before it is commanded — dry-sampling it
/// through the same envelope and inverse kinematics the live tick runs, over the
/// same trajectory [`take_command`] will build, for the worst step of one
/// `tick_hz` period at any phase the loop could wake on — and hands back a clock
/// that fits.
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
            span_stretched: sized != requested,
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
        JointRef::AntennaRight => 0,
        JointRef::AntennaLeft => 1,
        _ => return None,
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
/// A clock spanning fewer than [`MIN_JERK_PEAK_RATE`] periods is scaled from
/// that many instead of from itself, because the proportionality it is scaled by
/// is not true down there: a move that finishes inside one period is one step of
/// its whole span whatever clock it was asked for, so its ratio says nothing
/// about how much longer the clock must be. What that ratio says instead is the
/// closed form — [`duration_floor_s`] of the span it measured is exactly
/// `MIN_JERK_PEAK_RATE` periods times the ratio — so scaling from there lands on
/// the span's own floor in one pass rather than creeping up on it over many. A
/// wind-down re-commanding a stow with a nanosecond of deadline left is the case
/// that reaches this.
///
/// A clock inside the bound on the grid the dry pass walked but short of the
/// headroom is stretched by the headroom alone: its ratio is already one or
/// under, and what it lacks is the room the phases between two fine samples
/// need.
fn scale_past(duration: Duration, ratio: f64, tick_hz: f64) -> Option<Duration> {
    if carries(ratio, duration, tick_hz) {
        return Some(duration);
    }
    let from = duration.as_secs_f64().max(MIN_JERK_PEAK_RATE / tick_hz);
    let scaled = from * ratio.max(1.0);
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
    warp: WarpKind,
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
/// plan asks a joint to cover at the worst phase a loop could wake on, which is
/// the peak of the shaped path and nothing to do with how the loop is paced.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DryPassPeaks {
    /// The worst step any of the six cranks takes.
    pub legs: f64,
    /// The body yaw's worst step.
    pub body_yaw: f64,
    /// Each antenna's worst step: right, then left.
    pub antennas: [f64; 2],
}

impl DryPassPeaks {
    /// The figure a joint's steps belong to.
    ///
    /// The six cranks are one figure and the yaw another, because they are
    /// bounded separately even though one clock drives them both.
    fn slot(&mut self, id: JointRef) -> Option<&mut f64> {
        match id {
            JointRef::AntennaRight => Some(&mut self.antennas[0]),
            JointRef::AntennaLeft => Some(&mut self.antennas[1]),
            JointRef::BodyYaw => Some(&mut self.body_yaw),
            // The six cranks share one peak.
            JointRef::Leg0
            | JointRef::Leg1
            | JointRef::Leg2
            | JointRef::Leg3
            | JointRef::Leg4
            | JointRef::Leg5 => Some(&mut self.legs),
            // A ref naming no servo carries no change, so there is no figure
            // for it to inflate.
            JointRef::None => None,
        }
    }

    /// Take each joint's change from `from` to `to` into these peaks, or `None`
    /// when some joint's change is not a number.
    fn fold(&mut self, from: &JointVector, to: &JointVector) -> Option<()> {
        for ((id, angle), (_, last)) in to.joints().into_iter().zip(from.joints()) {
            let step = (angle - last).abs();
            if !step.is_finite() {
                return None;
            }
            if let Some(peak) = self.slot(id) {
                *peak = peak.max(step);
            }
        }
        Some(())
    }
}

/// The largest step of one `tick_hz` period `command` would plan from `start` at
/// any phase a loop could wake on, sampled through the same envelope and inverse
/// kinematics the live tick runs.
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
    warp: WarpKind,
    tick_hz: f64,
) -> Option<PhaseSeparation> {
    if start.antennas[0] == target.antennas[0] || start.antennas[1] == target.antennas[1] {
        return None;
    }
    let span = durations.antennas[0].max(durations.antennas[1]);
    let samples = dry_samples(span, tick_hz, NO_EXTENSION)?;
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

/// How many samples a dry walk over `span` takes at `sample_hz`, `beyond`
/// samples past the endpoint included, or `None` for a rate or a clock there is
/// nothing to measure about.
///
/// From the first sample that moves anything through the one that lands the
/// endpoint, and then `beyond` further. The move's own start is the pose already
/// held and is the baseline rather than a step. `span` is whichever of the
/// move's clocks the walk has a question about: every one of them for the step
/// bounds, the two antennas for the pair's phase. `sample_hz` is the walk's own
/// rate — the control rate for a walk that measures a crossing offset, and
/// [`DRY_OVERSAMPLE`] times it for one that measures a step, which is why the
/// budget below counts samples and not periods. The extension counts against
/// that budget too: it is samples the pass walks and solves like any other.
fn dry_samples(span: Duration, sample_hz: f64, beyond: u32) -> Option<u32> {
    if !(sample_hz.is_finite() && sample_hz > 0.0) {
        return None;
    }
    let samples = (span.as_secs_f64() * sample_hz).ceil() + f64::from(beyond);
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

/// The fine samples [`peaks_of`] walks for a move on `durations`, at `fine_hz`.
///
/// The step walk's whole extent in one named place: the move's longest clock,
/// and then the nominal period past its endpoint where the trajectory holds the
/// target. Named rather than written at the call site because how far the walk
/// runs is what decides whether the landing step is measured at all, and on the
/// geometry this machine has the landing windows sit under the interior peak —
/// so an extension dropped here changes no measured figure and no assertion
/// about one. What can hold it is a test of this function.
fn walk_samples(durations: MoveDurations, fine_hz: f64) -> Option<u32> {
    dry_samples(durations.longest(), fine_hz, ONE_PERIOD_BEYOND)
}

/// [`dry_pass_peaks`] over an already-resolved target.
///
/// Walked at [`DRY_OVERSAMPLE`] samples per nominal period, taking each joint's
/// largest change across a window a whole period wide — every such window the
/// fine grid holds, not only the ones a grid starting at the move's own zero
/// would have visited. That is the step the live loop emits at whichever phase
/// it wakes on, to within the half fine step [`grid_headroom`] covers.
///
/// The windows that reach back before the move's start read the pose the machine
/// is already holding, which is what a first period shortened by a late wake
/// actually steps from.
///
/// The walk runs one nominal period *past* the sample that lands the endpoint,
/// for the same reason at the other end: past a clock's duration the trajectory
/// holds that clock's joints on the target's own bits, so the windows reaching
/// beyond the end read the pose the machine holds after landing — and the step
/// the loop's last period emits is exactly one of them, from wherever inside the
/// final period it woke to the target. Without the extension those windows go
/// unmeasured, and a path whose joint angle is not monotone along its last
/// period can hold a landing step larger than any window the walk did visit.
fn peaks_of(
    cfg: &MotionConfig,
    start: &JointTargets,
    target: &JointTargets,
    durations: MoveDurations,
    warp: WarpKind,
    tick_hz: f64,
) -> Option<DryPassPeaks> {
    let fine_hz = tick_hz * f64::from(DRY_OVERSAMPLE);
    let samples = walk_samples(durations, fine_hz)?;
    let trajectory = Trajectory::new(start, target, durations, warp).ok()?;

    // One nominal period of fine samples, as a ring: the slot a new sample lands
    // in holds the sample from a whole period earlier, which is exactly what it
    // has to be compared against.
    let mut window = [joints_of(cfg, start)?; DRY_OVERSAMPLE as usize];
    let mut sampled = JointTargets::default();
    let mut peaks = DryPassPeaks::default();
    for step in 1..=samples {
        let t = Duration::try_from_secs_f64(f64::from(step) / fine_hz).ok()?;
        trajectory.sample(t, &mut sampled);
        let candidate = joints_of(cfg, &sampled)?;
        let slot = (step % DRY_OVERSAMPLE) as usize;
        peaks.fold(&window[slot], &candidate)?;
        window[slot] = candidate;
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
///
/// Public because a caller outside the loop deriving what the machine does with
/// a posture — how far each joint travels, how long that takes — has to compose
/// the same nine angles the tick would command, or it is measuring a machine
/// this one is not.
#[must_use]
pub fn joints_of(cfg: &MotionConfig, targets: &JointTargets) -> Option<JointVector> {
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
    for (side, joint) in [JointRef::AntennaRight, JointRef::AntennaLeft]
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
    // The sweep is the wrapped difference itself, not `short - last`: adding it
    // to `last` and taking it back off loses a bit at some angles, and the
    // endpoint case below compares it against a distance measured from `last`
    // for equality.
    let sweep = wrap_to_pi(target - last);
    let short = last + sweep;
    // Ground from `last` to `outboard` in the direction of travel, up to a
    // turn. Strictly inside the sweep is a crossing; nothing else is — an
    // antenna standing on the point measures no ground or a whole turn of it,
    // and one arriving there measures the sweep itself.
    //
    // Measured through the same wrap the sweep is, and turned round the same
    // way, so that an antenna commanded exactly to the point reaches the
    // comparison with the two figures bit-equal rather than a rounding apart.
    let forward = (outboard - last).rem_euclid(core::f64::consts::TAU);
    let to_outboard = if sweep >= 0.0 {
        forward
    } else {
        core::f64::consts::TAU - forward
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
    use crate::plant::{
        ClassProfile, GroupProfiles, PlantModel, SHIPPED_PERIOD_NS, SHIPPED_PROFILES,
    };
    use crate::seq::{RegId, SeqStepKind, StepContext};
    use reachy_kin::rest_head_pose;

    fn secs(s: f64) -> Duration {
        Duration::from_secs_f64(s)
    }

    /// The shared default configuration is the built one: a host that takes the
    /// reference and one that builds its own tick against the same numbers.
    ///
    /// Compared as text because [`MotionConfig`] carries a [`HeadGeometry`],
    /// which has no equality — the same reason `reachy-kin`'s own shared-value
    /// test compares its legs that way.
    #[test]
    fn the_shared_default_configuration_is_the_built_one() {
        let built = MotionConfig::default();
        let shared = default_motion_config();
        assert_eq!(format!("{shared:?}"), format!("{built:?}"));
        assert!(
            core::ptr::eq(default_motion_config(), shared),
            "the reference is built once and shared"
        );
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
    /// The two terms a stretch lands within — the grid headroom it aims past
    /// the bound by, and the closed form's own overestimate of a period's
    /// travel — are arithmetic, and [`lands_on`] accounts for each rather than
    /// tolerating either here. What is left is the fine grid's residual: the
    /// phase between two fine samples, worth a fraction of a percent. A clock
    /// landing a whole percent off is something other than sampling.
    const GRID_TOLERANCE: f64 = 0.01;

    /// How far a one-period step falls short of the instantaneous peak rate the
    /// closed form is written from, as a fraction, over a clock spanning
    /// `periods` periods.
    ///
    /// [`duration_floor_s`] sizes a clock from `f'` at its peak, and what the
    /// guard measures is a whole period's travel — the mean of `f'` across a
    /// window centred there, which the curvature reads under the peak. The mean
    /// over a window of half-width `h` is `f'` plus a sixth of `h²` times
    /// `f'''`, and min-jerk's `f'''` at the peak is minus sixteen times its
    /// `f'` there, so the shortfall is two thirds of one over the periods
    /// squared. A clock the pass measures honestly therefore lands a shade
    /// *under* the closed form, and that shade is arithmetic rather than
    /// tolerance.
    fn window_shortfall(periods: f64) -> f64 {
        2.0 / (3.0 * periods * periods)
    }

    /// Whether an effective clock landed on `wanted` — the larger of what was
    /// asked for and the closed form its span needs — with no band between the
    /// two.
    ///
    /// Over by no more than the grid headroom that clock carries, which is what
    /// a stretch aims past the bound by. Under by no more than
    /// [`window_shortfall`], which is the closed form's own overestimate of a
    /// period's travel. Both terms fall off as the square of the periods the
    /// clock spans, so a long clock is held to the sampling tolerance alone.
    fn lands_on(effective: f64, wanted: f64, tick_hz: f64) -> bool {
        let periods = wanted * tick_hz;
        let over = grid_headroom(periods) + GRID_TOLERANCE;
        let under = window_shortfall(periods) + GRID_TOLERANCE;
        effective >= wanted * (1.0 - under) && effective <= wanted * (1.0 + over)
    }

    /// The period the tests grid on is the rate the floors are derived at, so
    /// what they measure is what a shipped configuration runs.
    #[test]
    fn the_test_period_is_the_floor_tick_rate() {
        assert_eq!(PERIOD.as_secs_f64(), 1.0 / FLOOR_TICK_HZ);
    }

    /// The joint vector that holds `targets`, and that pose's clearance.
    fn joints_for(cfg: &MotionConfig, targets: &JointTargets) -> (JointVector, f64) {
        (
            joints_of(cfg, targets).expect("the test pose is reachable"),
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
    fn armed_at(cfg: &MotionConfig, targets: &JointTargets) -> (Armed, JointVector) {
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
        (Armed::new(&record, JointFlags::NONE), outcome.pinned)
    }

    /// A state in a fixture's own slot.
    ///
    /// The case owns the bytes for the life of the run; every access validates
    /// them. There is no second form of the state to compare against or to
    /// forget a field of.
    struct Armed {
        slot: MotionSnapWire,
    }

    impl Armed {
        /// A machine that has just finished arming.
        fn new(record: &ArmRecord, degraded: JointFlags) -> Self {
            let mut slot = MotionSnapWire::new();
            arm(slot.clear_valid(), record, degraded);
            Self { slot }
        }

        /// A state written by something other than [`arm`].
        fn of(slot: MotionSnapWire) -> Self {
            Self { slot }
        }

        /// What [`resume`] makes of these bytes.
        fn resumes(&self) -> Result<(), StateError> {
            resume(self.slot.validate().expect("the fixture's slot validates"))
        }

        /// The bytes themselves, for a case that damages them or compares them.
        fn bytes(&self) -> &MotionSnapWire {
            &self.slot
        }
    }

    impl core::ops::Deref for Armed {
        type Target = MotionSnap;

        fn deref(&self) -> &MotionSnap {
            self.slot.validate().expect("the fixture's slot validates")
        }
    }

    impl core::ops::DerefMut for Armed {
        fn deref_mut(&mut self) -> &mut MotionSnap {
            self.slot
                .validate_mut()
                .expect("the fixture's slot validates")
        }
    }

    /// One tick with a live read.
    fn tick_with(
        cfg: &MotionConfig,
        state: &mut MotionSnap,
        now: Duration,
        present: &JointVector,
        command: Option<&MotionCommand>,
    ) -> TickOutputs {
        let mut out = TickOutputs::default();
        // A driver holding the goal the tick last emitted, which is what one
        // does between writes. The dead time the model chases the setpoint at
        // is the tick's own; this is only who is holding it.
        let held = last_goal(state);
        motion_tick(
            cfg,
            state,
            &TickInputs {
                now,
                period: PERIOD,
                present: Some(present),
                commanded: Some(&held),
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
        state: &mut MotionSnap,
        now: Duration,
        present: &JointVector,
        health: &[ServoHealth; ROW_COUNT],
        command: Option<&MotionCommand>,
    ) -> TickOutputs {
        let mut out = TickOutputs::default();
        let held = last_goal(state);
        motion_tick(
            cfg,
            state,
            &TickInputs {
                now,
                period: PERIOD,
                present: Some(present),
                commanded: Some(&held),
                command,
                health: Some(health),
            },
            &mut out,
        );
        out
    }

    /// Nine healthy servos, numbered from ten as the bench's roster is.
    fn healthy_servos() -> [ServoHealth; ROW_COUNT] {
        let mut health = [ServoHealth::default(); ROW_COUNT];
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
        state: &mut MotionSnap,
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
                warp: WarpKind::MinJerk,
            },
            period,
        )
    }

    /// The same, for a command whose two group clocks differ.
    fn run_command(
        cfg: &MotionConfig,
        state: &mut MotionSnap,
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
            warp: WarpKind::MinJerk,
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
            warp: WarpKind::MinJerk,
        };

        let (floored, stretch) =
            floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
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
            warp: WarpKind::MinJerk,
        };

        let (mut state, pinned) = armed_at(&cfg, &stow);
        let (floored, stretch) =
            floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
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
                warp: WarpKind::MinJerk,
            };
            let (_, stretch) =
                floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
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
            warp: WarpKind::MinJerk,
        };

        let (floored, stretch) =
            floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
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
            warp: WarpKind::MinJerk,
        };
        let (state, _) = armed_at(&cfg, &crooked);
        assert_eq!(
            floor_move_clock(&cfg, &last_targets(&state), &calm, FLOOR_TICK_HZ).1,
            None,
            "a calm stow carries the half turn as asked"
        );

        let quick = secs(0.5);
        let command = MotionCommand::MoveTo {
            target: stow,
            durations: MoveDurations::uniform(quick),
            warp: WarpKind::MinJerk,
        };

        // Unfloored, the quick fold is the regression: it steps past the
        // bound partway round and the move is abandoned wherever that was.
        let (mut state, pinned) = armed_at(&cfg, &crooked);
        let (_, out) = run_command(&cfg, &mut state, &pinned, &command, 1.0 / FLOOR_TICK_HZ);
        assert!(
            matches!(
                out.report.aborted,
                Some(MoveAbort::StepTooLarge {
                    joint: JointRef::BodyYaw,
                    ..
                })
            ),
            "{:?}",
            out.report.aborted
        );

        let (mut state, pinned) = armed_at(&cfg, &crooked);
        let (floored, stretch) =
            floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("a half turn does not fit a half second clock");
        let floor = duration_floor_s(core::f64::consts::PI, cfg.max_step.body_yaw, FLOOR_TICK_HZ);
        // The closed form is written from the peak rate, and a whole period's
        // travel is the mean across a window centred there, so a clock measured
        // honestly lands [`window_shortfall`] under it — four parts in ten
        // thousand over a clock this long.
        let carried = floor * (1.0 - window_shortfall(floor * FLOOR_TICK_HZ));
        assert!(
            stretch.effective.head.as_secs_f64() >= carried,
            "stretched to {:?}, under the {carried:.4} s the half turn needs",
            stretch.effective.head,
        );

        let (_, out) = run_command(&cfg, &mut state, &pinned, &floored, 1.0 / FLOOR_TICK_HZ);
        assert_eq!(out.report.fault, None, "{:?}", out.report);
        assert!(out.report.completed, "{:?}", out.report);
        // The fold travelled the whole way: the recovery ends at stow, not
        // partway to it.
        assert!(
            last_targets(&state).body_yaw.abs() < 1e-12,
            "left at {} rad",
            last_targets(&state).body_yaw
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
            // A refused command is not a shorter goal sequence, it is none at
            // all — and a caller comparing sequences would otherwise wait out
            // the loop below for a move that never started.
            if n == 0 {
                assert_eq!(out.report.command, CommandDisposition::Started, "tick {n}");
            }
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

    /// The largest single-period step in a goal sequence, per joint group.
    fn peaks_of_goals(start: &JointVector, goals: &[JointVector]) -> DryPassPeaks {
        let mut peaks = DryPassPeaks::default();
        let mut previous = *start;
        for goal in goals {
            peaks
                .fold(&previous, goal)
                .expect("a goal the loop emitted is a number");
            previous = *goal;
        }
        peaks
    }

    /// The largest single-period step in a goal sequence, over all nine joints.
    fn worst_step_of(start: &JointVector, goals: &[JointVector]) -> f64 {
        let peaks = peaks_of_goals(start, goals);
        peaks
            .legs
            .max(peaks.body_yaw)
            .max(peaks.antennas[0])
            .max(peaks.antennas[1])
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
            warp: WarpKind::MinJerk,
        };

        let (state, _) = armed_at(&cfg, &crooked);
        let (floored, _) = floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);

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
            warp: WarpKind::MinJerk,
        };
        let (state, _) = armed_at(&cfg, &crooked);
        let (floored, stretch) =
            floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
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
                warp: WarpKind::MinJerk,
            };
            let (floored, _) =
                floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
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

    /// A move over a span whose clock is a couple of periods long carries at
    /// every wake phase, whether the clock was stretched to there or asked for
    /// there, and the dry pass's own number bounds what the loop emits.
    ///
    /// This is the band where one grid phase says least. Two periods is the worst
    /// of it: that grid's middle sample lands on the peak of the min-jerk rate and
    /// splits it, so it reads five eighths of the step a grid a quarter of a
    /// period over emits. A clock accepted on that reading alone steps a quarter
    /// past the bound at an unlucky wake phase and abandons a healthy move — and
    /// clocks this short are reachable twice over: a wind-down re-commands the
    /// remainder of a stow, and floors are proportional to the span, so a small
    /// span never forces a long clock.
    ///
    /// Walked over spans whose closed-form floors run from a period and a half to
    /// four, each asked for on clocks from a quarter of that floor to a tenth
    /// past it, each driven at seven wake phases. Of every one: the loop never
    /// steps past the bound, and never further than the dry pass measured with
    /// its headroom. And a clock asked for at the closed form is accepted as
    /// asked — the form is written from the peak rate, which a clock this short
    /// never sustains for a whole period, so it forbids nothing the physics does.
    #[test]
    fn a_clock_of_a_couple_of_periods_carries_its_move_at_every_phase() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (state, _) = armed_at(&cfg, &start);

        for tenths in [15, 20, 25, 30, 40] {
            let periods = f64::from(tenths) / 10.0;
            let floor = periods / FLOOR_TICK_HZ;
            // The yaw sweep whose closed-form floor is exactly that clock.
            let span = periods * cfg.max_step.body_yaw / MIN_JERK_PEAK_RATE;
            for tenths_of_floor in [2.5, 5.0, 7.5, 9.0, 10.0, 11.0] {
                let requested = floor * tenths_of_floor / 10.0;
                let command = MotionCommand::MoveTo {
                    target: JointTargets {
                        body_yaw: span,
                        ..JointTargets::default()
                    },
                    durations: MoveDurations::uniform(secs(requested)),
                    warp: WarpKind::MinJerk,
                };
                let (floored, stretch) =
                    floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
                if tenths_of_floor >= 10.0 {
                    assert_eq!(
                        stretch, None,
                        "{periods} periods over {span:.4} rad was right-sized at {requested:.4} s, \
                         and it does not need to be"
                    );
                }
                let planned = dry_pass_peaks(&cfg, &last_targets(&state), &floored, FLOOR_TICK_HZ)
                    .expect("the sweep is measurable");
                let effective = durations_of(&floored).head.as_secs_f64();
                let admitted = planned.body_yaw * (1.0 + grid_headroom(effective * FLOOR_TICK_HZ));

                for eighths in 1..8 {
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
                        "{span:.4} rad asked for in {requested:.4} s ran on {effective:.4} s and \
                         stepped {worst:.7} rad {eighths} eighths out of phase, past the guard's \
                         bound"
                    );
                    assert!(
                        worst <= admitted,
                        "{span:.4} rad on {effective:.4} s stepped {worst:.7} rad {eighths} eighths \
                         out of phase, past the {:.7} rad the dry pass measured",
                        planned.body_yaw
                    );
                }
            }
        }
    }

    /// A stow re-commanded with a nanosecond of deadline left is re-floored to a
    /// clock that carries the span still ahead of it, and the pass converges
    /// there rather than exhausting its passes.
    ///
    /// The wind-down clamps the stow it re-commands to the deadline's remainder,
    /// which is a clock as short as a nanosecond — a move that finishes between
    /// two ticks, and whose measured step is its whole span whatever clock it was
    /// asked for. Nothing about the clock's length can be read off that ratio;
    /// what can be read off it is the closed form, and the pass lands there.
    #[test]
    fn a_stow_re_commanded_with_a_nanosecond_left_is_floored_to_a_clock_that_carries_it() {
        let cfg = MotionConfig::default();
        // A fold interrupted near its end: the head is already stowed and a
        // third of a radian of yaw is what remains.
        let remaining = 0.3;
        let stowed = JointTargets {
            head_pose_body: reachy_kin::stow_head_pose(),
            ..JointTargets::default()
        };
        let part_way = JointTargets {
            body_yaw: remaining,
            ..stowed
        };
        let command = MotionCommand::MoveTo {
            target: stowed,
            durations: MoveDurations::uniform(Duration::from_nanos(1)),
            warp: WarpKind::MinJerk,
        };

        let (mut state, pinned) = armed_at(&cfg, &part_way);
        let (floored, stretch) =
            floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("a nanosecond carries nothing");

        // Converged, and not exhausted: the clock the last pass produced is one
        // the acceptance predicate takes, measured again from scratch.
        let ratios = worst_step_ratios(
            &cfg,
            &last_targets(&state),
            &stowed,
            stretch.effective,
            WarpKind::MinJerk,
            FLOOR_TICK_HZ,
        )
        .expect("the remainder is measurable");
        assert!(
            ratios.fit(stretch.effective, FLOOR_TICK_HZ),
            "the pass handed back {:?}, which its own measurement does not accept: {ratios:?}",
            stretch.effective
        );

        // And that clock is the remaining span's own floor, less the shortfall a
        // whole period's travel has against the peak rate the form is written
        // from.
        let floor = duration_floor_s(remaining, cfg.max_step.body_yaw, FLOOR_TICK_HZ);
        assert!(
            lands_on(stretch.effective.head.as_secs_f64(), floor, FLOOR_TICK_HZ),
            "re-floored to {:?}, not the {floor:.4} s the remainder needs",
            stretch.effective.head
        );

        // It runs, in bounds, at every phase the loop could wake on.
        for eighths in 1..8 {
            let shift = PERIOD.mul_f64(f64::from(eighths) / 8.0);
            let (start, goals) = goals_driven_at(&cfg, &part_way, &floored, &|n| {
                if n == 0 {
                    Duration::ZERO
                } else {
                    PERIOD * n - shift
                }
            });
            let worst = worst_step_of(&start, &goals);
            assert!(
                worst <= cfg.max_step.body_yaw,
                "the re-floored remainder stepped {worst:.7} rad {eighths} eighths out of phase"
            );
        }

        let (_, out) = run_command(&cfg, &mut state, &pinned, &floored, 1.0 / FLOOR_TICK_HZ);
        assert_eq!(out.report.fault, None, "{:?}", out.report);
        assert!(out.report.completed, "{:?}", out.report);
        assert!(
            last_targets(&state).body_yaw.abs() < 1e-12,
            "left at {} rad",
            last_targets(&state).body_yaw
        );
    }

    /// Every goal a move emits at each of `phases` wake phases across one
    /// period, with the goals of each run kept in order.
    ///
    /// Driven through the real tick by [`goals_driven_at`], so the sequences are
    /// what the loop emits — the antenna resolution the accept does, the
    /// envelope, the step guard and the completion rule included — rather than a
    /// second model of them built from the same two primitives the dry pass
    /// uses. Each run's last goal is the one that lands the move, so the landing
    /// step is in the sweep by construction.
    ///
    /// The phase moves on a *short* gap, as everywhere else in this module: a
    /// late wake is credited at most one period, so what shifts the grid is the
    /// period that recovers the lateness.
    fn goals_at_every_phase(
        cfg: &MotionConfig,
        start: &JointTargets,
        command: &MotionCommand,
        phases: u32,
    ) -> Vec<(JointVector, Vec<JointVector>)> {
        (0..phases)
            .map(|phase| {
                let shift = PERIOD.mul_f64(f64::from(phase) / f64::from(phases));
                goals_driven_at(cfg, start, command, &|n| {
                    if n == 0 {
                        Duration::ZERO
                    } else {
                        PERIOD * n - shift
                    }
                })
            })
            .collect()
    }

    /// Every step a wake phase can make the loop emit over a whole move, the
    /// step that lands it included, per joint group.
    fn peaks_emitted(
        cfg: &MotionConfig,
        start: &JointTargets,
        command: &MotionCommand,
        phases: u32,
    ) -> DryPassPeaks {
        let mut peaks = DryPassPeaks::default();
        for (pinned, goals) in goals_at_every_phase(cfg, start, command, phases) {
            let mut previous = pinned;
            for goal in &goals {
                peaks
                    .fold(&previous, goal)
                    .expect("a goal the loop emitted is a number");
                previous = *goal;
            }
        }
        peaks
    }

    /// The largest step the loop emits *into* the pose a move lands on, per
    /// joint group.
    ///
    /// The step the walk's extension exists for: one end wherever inside the
    /// final period the loop last woke, the other on the target itself. Taken
    /// per joint as the last step of the run that moved that joint at all — its
    /// own clock's landing, which on a move whose groups run on different clocks
    /// is not the run's last goal — so what this reports is the landing step the
    /// loop actually emitted at that phase and not a reconstruction of one.
    fn peaks_landing(
        cfg: &MotionConfig,
        start: &JointTargets,
        command: &MotionCommand,
        phases: u32,
    ) -> DryPassPeaks {
        let mut peaks = DryPassPeaks::default();
        for (pinned, goals) in goals_at_every_phase(cfg, start, command, phases) {
            let mut previous = pinned;
            let mut landing = [0.0; ROW_COUNT];
            for goal in &goals {
                for (slot, ((_, angle), (_, last))) in
                    goal.joints().into_iter().zip(previous.joints()).enumerate()
                {
                    let step = (angle - last).abs();
                    if step > 0.0 {
                        landing[slot] = step;
                    }
                }
                previous = *goal;
            }
            for (slot, (id, _)) in previous.joints().into_iter().enumerate() {
                if let Some(peak) = peaks.slot(id) {
                    *peak = peak.max(landing[slot]);
                }
            }
        }
        peaks
    }

    /// The windows the planned path holds into the pose a move lands on, per
    /// joint group.
    ///
    /// The landing step the trajectory itself explains: one end on the target
    /// the clock holds from its duration onward, the other at each fine sample
    /// of the final period, which is where a loop's last wake before the clock
    /// runs out can fall. Built from [`Trajectory`] and [`joints_of`] — the two
    /// primitives the dry pass walks with — so what it says is what the plan
    /// holds, independent of anything the tick does with it. Per group on that
    /// group's own clock: a move whose antennas outlast its head group lands
    /// each of them at its own moment.
    fn landing_windows(
        cfg: &MotionConfig,
        start: &JointTargets,
        command: &MotionCommand,
        tick_hz: f64,
    ) -> DryPassPeaks {
        let MotionCommand::MoveTo {
            target,
            durations,
            warp,
        } = command
        else {
            panic!("a landing window is a question about a move");
        };
        let resolved = resolve_antennas(start, target).expect("the target resolves");
        let trajectory =
            Trajectory::new(start, &resolved, *durations, *warp).expect("the move is plannable");
        let held = joints_of(cfg, &resolved).expect("the target solves");
        let fine_hz = tick_hz * f64::from(DRY_OVERSAMPLE);

        let final_period = |clock: Duration| {
            let mut peaks = DryPassPeaks::default();
            let mut sampled = JointTargets::default();
            for back in 1..=DRY_OVERSAMPLE {
                let t = clock.saturating_sub(secs(f64::from(back) / fine_hz));
                trajectory.sample(t, &mut sampled);
                let at = joints_of(cfg, &sampled).expect("the path solves");
                peaks.fold(&at, &held).expect("a window is a number");
            }
            peaks
        };

        let head = final_period(durations.head);
        let right = final_period(durations.antennas[0]);
        let left = final_period(durations.antennas[1]);
        DryPassPeaks {
            legs: head.legs,
            body_yaw: head.body_yaw,
            antennas: [right.antennas[0], left.antennas[1]],
        }
    }

    /// The fixtures the landing walk is asserted over: the gestures this
    /// machine actually runs, each named with the clock it runs on.
    ///
    /// The stow pose carries the antennas the machine actually folds them to,
    /// so its antenna leg is a landing and a length: the sweep from neutral is
    /// most of a turn inboard over the head, and the fixture asserts the walk
    /// lands on it and takes that long. Which branch of the arc policy produces
    /// the inboard sweep is `an_antenna_sweep_misses_its_outboard_point`'s
    /// question, not this one's.
    fn landing_fixtures() -> [(&'static str, JointTargets, JointTargets, f64); 4] {
        let rest = JointTargets {
            head_pose_body: rest_head_pose(),
            ..JointTargets::default()
        };
        let stow = JointTargets {
            head_pose_body: reachy_kin::stow_head_pose(),
            antennas: crate::disarm::STOW_ANTENNAS,
            ..JointTargets::default()
        };
        let raised = JointTargets {
            antennas: [1.0, -1.0],
            ..pose_at(0.19)
        };
        let spun = JointTargets {
            body_yaw: core::f64::consts::PI,
            ..rest
        };
        [
            ("the presence gesture", rest, raised, 0.8),
            ("the fold to stow", raised, stow, HEAD_GROUP_FLOOR_S),
            // To neutral and not back to the rest pose: the rest a machine is
            // armed from sits outside the legs' travel window and arming pins
            // it, so it is a pose to start from and not one to command.
            ("the lift to neutral", stow, JointTargets::default(), 0.5),
            ("the fold from a half turn", spun, stow, 2.0),
        ]
    }

    /// The binding rule holds through the step that lands a move, and that step
    /// is one the plan holds.
    ///
    /// What the loop emits are period-spaced differences of the *clamped*
    /// trajectory, so every step but the last is a full period of path and is
    /// measured by a window the walk visits whatever the path does in between.
    /// The last one is not: it runs from wherever the loop woke inside the final
    /// period to the target being held, which is a *sub*-period difference — and
    /// a crank whose angle is not monotone over that stretch can carry it
    /// further than any window ending on the endpoint. The walk therefore runs
    /// one period past the endpoint, where the trajectory holds the target.
    ///
    /// Three claims, per joint group and at the clock as asked, over this
    /// machine's own gestures. The binding rule: what the pass measured, with
    /// the headroom that clock carries, covers every step a sweep of wake phases
    /// through the real tick can make the loop emit. The landing step is real
    /// and is the one the plan holds: wherever a group moved at all it lands
    /// somewhere, and what it lands from is a fine sample of the final period,
    /// so a landing step the trajectory does not explain fails here. And the
    /// pass covers those windows — the extension's own claim, and the one bound
    /// of the three this machine's geometry leaves slack, because the landing
    /// windows sit a hundredth to two of the interior peak. That the walk runs
    /// far enough to have measured them is pinned at the seam instead, by
    /// [`the_step_walk_runs_a_period_past_the_endpoint`].
    #[test]
    fn the_binding_rule_holds_through_the_step_that_lands_the_move() {
        let cfg = MotionConfig::default();

        for (name, start, target, duration) in landing_fixtures() {
            let command = MotionCommand::MoveTo {
                target,
                durations: MoveDurations::uniform(secs(duration)),
                warp: WarpKind::MinJerk,
            };
            let (state, _) = armed_at(&cfg, &start);
            let pinned = last_targets(&state);
            // The clock the command would run on, which is the one the pass is
            // asked about: a gesture's own duration where that carries it, and
            // the floor for its span where it does not.
            let (floored, _) = floor_move_clock(&cfg, &pinned, &command, FLOOR_TICK_HZ);
            let measured = dry_pass_peaks(&cfg, &pinned, &floored, FLOOR_TICK_HZ)
                .expect("the path is walkable");
            let emitted = peaks_emitted(&cfg, &start, &floored, 64);
            let landing = peaks_landing(&cfg, &start, &floored, 64);
            let windows = landing_windows(&cfg, &pinned, &floored, FLOOR_TICK_HZ);
            // Per group and not per move: each group's headroom is the residual
            // of the grid *its own* clock was walked on.
            let clocks = durations_of(&floored);
            for (group, clock, measured, emitted, landing, windows) in [
                (
                    "the legs",
                    clocks.head,
                    measured.legs,
                    emitted.legs,
                    landing.legs,
                    windows.legs,
                ),
                (
                    "the body yaw",
                    clocks.head,
                    measured.body_yaw,
                    emitted.body_yaw,
                    landing.body_yaw,
                    windows.body_yaw,
                ),
                (
                    "the right antenna",
                    clocks.antennas[0],
                    measured.antennas[0],
                    emitted.antennas[0],
                    landing.antennas[0],
                    windows.antennas[0],
                ),
                (
                    "the left antenna",
                    clocks.antennas[1],
                    measured.antennas[1],
                    emitted.antennas[1],
                    landing.antennas[1],
                    windows.antennas[1],
                ),
            ] {
                let admitted = 1.0 + grid_headroom(clock.as_secs_f64() * FLOOR_TICK_HZ);
                assert!(
                    emitted <= measured * admitted,
                    "{name}: {group} step {emitted:.6} rad at some wake phase, past the \
                     {measured:.6} rad the dry pass measured with its headroom"
                );
                // A group the move does not carry anywhere has no landing step
                // to speak of; one it does carry must have measured a real one,
                // or the comparison below is satisfied by nothing.
                assert!(
                    measured == 0.0 || landing > 0.0,
                    "{name}: {group} moved {measured:.6} rad per period and yet lands nowhere, so \
                     its landing step measures nothing"
                );
                // The landing step the loop emits is one the plan holds: its far
                // end is the target, and its near end is a fine sample of the
                // final period the walk's extension covers.
                assert!(
                    landing <= windows * admitted,
                    "{name}: {group} lands {landing:.6} rad, past the {windows:.6} rad the \
                     planned path holds into the target over its final period"
                );
                // And the pass measured those windows: this is the extension's
                // own claim, and the only one of the three that a walk stopping
                // at the endpoint could fail.
                assert!(
                    windows <= measured * admitted,
                    "{name}: {group} holds {windows:.6} rad into the target over its final \
                     period, past the {measured:.6} rad the dry pass measured with its headroom"
                );
            }
        }
    }

    /// A clock the pass hands back carries its move at every wake phase, the
    /// step that lands it included.
    ///
    /// The same gestures asked for on a quarter of the clock they need, so every
    /// one of them is stretched, and then swept at sixty-four wake phases
    /// through the landing. The sweep starts from the pose arming leaves the
    /// machine holding, which is the pose the floored clock was measured from.
    #[test]
    fn a_floored_clock_carries_its_landing_step_too() {
        let cfg = MotionConfig::default();

        for (name, start, target, duration) in landing_fixtures() {
            let command = MotionCommand::MoveTo {
                target,
                durations: MoveDurations::uniform(secs(duration / 4.0)),
                warp: WarpKind::MinJerk,
            };
            let (state, _) = armed_at(&cfg, &start);
            let pinned = last_targets(&state);
            let (floored, stretch) = floor_move_clock(&cfg, &pinned, &command, FLOOR_TICK_HZ);
            assert!(
                stretch.is_some(),
                "{name}: a quarter of its clock is not enough for it"
            );

            let emitted = peaks_emitted(&cfg, &start, &floored, 64);
            for (group, emitted, bound) in [
                ("a crank", emitted.legs, cfg.max_step.legs),
                ("the yaw", emitted.body_yaw, cfg.max_step.body_yaw),
                (
                    "the right antenna",
                    emitted.antennas[0],
                    cfg.max_step.antennas,
                ),
                (
                    "the left antenna",
                    emitted.antennas[1],
                    cfg.max_step.antennas,
                ),
            ] {
                assert!(
                    emitted <= bound,
                    "{name}: {group} stepped {emitted:.6} rad at some wake phase, past the \
                     {bound:.6} rad bound"
                );
            }
        }
    }

    /// The step walk runs one nominal period past the sample that lands the
    /// move.
    ///
    /// Pinned at the seam rather than through a measured figure, because on this
    /// machine's geometry there is no reachable move whose landing windows
    /// exceed the windows inside it: the extension is coverage of windows that
    /// happen to sit under the interior peak, so dropping it moves nothing any
    /// other assertion in this module reads. What it would move is the guarantee
    /// — a path not monotone over its last period can put its landing step above
    /// every window a walk stopping at the endpoint visits — so what holds the
    /// extension in place is this.
    #[test]
    fn the_step_walk_runs_a_period_past_the_endpoint() {
        let fine_hz = FLOOR_TICK_HZ * f64::from(DRY_OVERSAMPLE);
        for periods in [1.0, 2.5, 40.0] {
            let durations = MoveDurations::uniform(secs(periods / FLOOR_TICK_HZ));
            #[expect(
                clippy::cast_possible_truncation,
                clippy::cast_sign_loss,
                reason = "a handful of periods at the fine rate, far under MAX_DRY_SAMPLES"
            )]
            let lands = (periods * f64::from(DRY_OVERSAMPLE)).ceil() as u32;
            assert_eq!(
                walk_samples(durations, fine_hz),
                Some(lands + DRY_OVERSAMPLE),
                "a {periods}-period clock is walked to its endpoint and no further"
            );
        }
    }

    /// The samples the walk takes past the endpoint are samples, and the budget
    /// counts them.
    ///
    /// Each is a solve like any other, so a span that fits inside
    /// [`MAX_DRY_SAMPLES`] only without its extension is a span the pass hands
    /// back rather than one it walks short. The crossing walk takes no
    /// extension and reaches the budget on its own.
    #[test]
    fn the_walks_budget_counts_what_it_takes_past_the_endpoint() {
        let fine_hz = FLOOR_TICK_HZ * f64::from(DRY_OVERSAMPLE);
        let whole = f64::from(MAX_DRY_SAMPLES - ONE_PERIOD_BEYOND) / fine_hz;

        assert_eq!(
            dry_samples(secs(whole), fine_hz, ONE_PERIOD_BEYOND),
            Some(MAX_DRY_SAMPLES)
        );
        assert_eq!(
            dry_samples(secs(whole + 1.0 / fine_hz), fine_hz, ONE_PERIOD_BEYOND),
            None
        );
        assert_eq!(
            dry_samples(
                secs(f64::from(MAX_DRY_SAMPLES) / FLOOR_TICK_HZ),
                FLOOR_TICK_HZ,
                NO_EXTENSION
            ),
            Some(MAX_DRY_SAMPLES)
        );
    }

    /// The headroom is derived at every clock length, and nothing caps it.
    ///
    /// Four over the square of the fine samples the clock spans, which is the
    /// second-order deficit of a wake phase falling between two of them. A clock
    /// spanning no periods at all was not measured, and no headroom makes it
    /// acceptable.
    ///
    /// Pinned as the numbers themselves rather than as the expression, because
    /// what an operator is promised is a *width*: the sentences in
    /// [`STRETCH_GRID_HEADROOM`]'s own documentation and in the bench example
    /// quote these figures, and a tuning of either constant that left them stale
    /// would pass a test written as the implementation's own arithmetic.
    #[test]
    fn the_grid_headroom_is_the_fine_grids_own_residual() {
        // The three lengths the prose quotes: a one-period clock, the two
        // periods the adversarial case is written at, and the fold's own floor.
        let fold_floor_periods = HEAD_GROUP_FLOOR_S * FLOOR_TICK_HZ;
        assert!(
            (fold_floor_periods - 18.0).abs() < 1e-9,
            "the fold's floor spans {fold_floor_periods} periods, not the 18 the figures below \
             were read off"
        );
        for (periods, expected) in [
            // 4 / 8², the whole of a one-period clock's own span.
            (1.0, 0.062_5),
            // 4 / 16²: under two parts in a hundred.
            (2.0, 0.015_625),
            // 4 / 144²: under two parts in ten thousand.
            (fold_floor_periods, 0.000_192_901_234_567_901_2),
            // 4 / 320²: four parts in a hundred thousand at the shipped gesture.
            (40.0, 0.000_039_062_5),
        ] {
            let headroom = grid_headroom(periods);
            assert!(
                (headroom - expected).abs() < 1e-15,
                "at {periods} periods: {headroom} against {expected}"
            );
        }
        // Falling off monotonically is what makes a longer clock never harder to
        // accept than a shorter one, which is what the stretch walks on.
        let mut previous = f64::INFINITY;
        for periods in [0.5, 1.0, 1.875, 2.0, 4.0, 18.0, 40.0, 1000.0] {
            let headroom = grid_headroom(periods);
            assert!(
                headroom < previous,
                "at {periods} periods the term rose to {headroom} from {previous}"
            );
            previous = headroom;
        }
        assert_eq!(grid_headroom(0.0), f64::INFINITY);
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
            warp: WarpKind::MinJerk,
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

        // `None` until there is a clock before this one to compare against;
        // nothing is monotone about the first entry of a sequence.
        let mut previous: Option<f64> = None;
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
                warp: WarpKind::MinJerk,
            };
            let (_, stretch) =
                floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
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
            if let Some(previous) = previous {
                let residual = grid_headroom(previous * FLOOR_TICK_HZ) + GRID_TOLERANCE;
                assert!(
                    effective >= previous * (1.0 - residual),
                    "{requested:.2} s ran on {effective:.4} s, shorter than the clock before it"
                );
            }
            previous = Some(effective);
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
                warp: WarpKind::MinJerk,
            };
            floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ)
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
                    warp: WarpKind::MinJerk,
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
                    warp: WarpKind::MinJerk,
                },
                FLOOR_TICK_HZ,
            ),
            // No control rate: there is no grid to sample the path on, and a
            // dry pass with no periods in it measures nothing.
            (
                MotionCommand::MoveTo {
                    target: pose_at(0.19),
                    durations: MoveDurations::uniform(secs(1.0)),
                    warp: WarpKind::MinJerk,
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
                    warp: WarpKind::MinJerk,
                },
                FLOOR_TICK_HZ,
            ),
        ] {
            let (floored, stretch) =
                floor_move_clock(&cfg, &last_targets(&state), &command, tick_hz);
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
            warp: WarpKind::MinJerk,
        };

        let (floored, stretch) =
            floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
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
            warp: WarpKind::MinJerk,
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
            floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
        assert_eq!(stretch, None, "{stretch:?}");
        assert_eq!(floored, command);

        let pair = dry_pass_separation(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ)
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
            floor_move_clock(&demanding, &last_targets(&state), &command, FLOOR_TICK_HZ);
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
                floor_move_clock(cfg, &last_targets(&state), &in_step, FLOOR_TICK_HZ);
            let stretch = stretch.expect("a pair in step is de-phased");
            assert!(stretch.dephased, "{stretch:?}");
            let pair = dry_pass_separation(cfg, &last_targets(&state), &floored, FLOOR_TICK_HZ)
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
            dry_pass_separation(&shipped, &last_targets(&state), &command, FLOOR_TICK_HZ)
                .expect("both antennas cross the band");
        let narrow_band =
            dry_pass_separation(&narrow, &last_targets(&state), &command, FLOOR_TICK_HZ)
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
            warp: WarpKind::MinJerk,
        };

        assert_eq!(
            dry_pass_separation(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ),
            None
        );
        let (floored, stretch) =
            floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
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
            &last_targets(&state),
            &inboard_sweep(0.8, [0.7, 0.3]),
            FLOOR_TICK_HZ,
        )
        .expect("both antennas cross the band");
        let calm = dry_pass_separation(
            &cfg,
            &last_targets(&state),
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

        let in_step = dry_pass_separation(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ)
            .expect("both antennas cross the band");
        assert!(
            in_step.offset < 1e-6,
            "one clock leaves the pair {:.4} rad apart",
            in_step.offset
        );

        let (floored, stretch) =
            floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("a pair in step is de-phased");
        assert!(stretch.dephased, "{stretch:?}");
        assert_eq!(stretch.effective.head, stretch.requested.head);
        assert_eq!(stretch.effective.antennas[1], stretch.requested.antennas[1]);
        assert!(
            stretch.effective.antennas[0] > stretch.requested.antennas[0],
            "{stretch:?}"
        );

        let separated = dry_pass_separation(&cfg, &last_targets(&state), &floored, FLOOR_TICK_HZ)
            .expect("both antennas still cross the band");
        assert!(
            separated.met(cfg.phase.separation_rad),
            "de-phased to {:.4} rad, which is still under the bound",
            separated.offset
        );
    }

    /// The report says which of the two things happened to the clocks, and a
    /// caller that must tell a clock too short for its path from a pair merely
    /// parted has both answers.
    ///
    /// Three shapes: a yaw sweep on a clock that cannot carry it, with no pair
    /// to judge; a pair in step on clocks that carry their own arcs, parted and
    /// nothing else; and a pair whose clocks carry neither, which says both.
    #[test]
    fn the_report_says_whether_a_span_was_stretched_or_only_a_pair_parted() {
        let cfg = MotionConfig::default();
        let (state, _) = armed_at(&cfg, &JointTargets::default());
        let yaw = MotionCommand::MoveTo {
            target: JointTargets {
                body_yaw: 1.0,
                ..JointTargets::default()
            },
            durations: MoveDurations::uniform(secs(0.2)),
            warp: WarpKind::MinJerk,
        };
        let (_, stretch) = floor_move_clock(&cfg, &last_targets(&state), &yaw, FLOOR_TICK_HZ);
        let stretch = stretch.expect("a fifth of a second cannot carry a radian of yaw");
        assert!(stretch.span_stretched, "{stretch:?}");
        assert!(!stretch.dephased, "{stretch:?}");

        let (swept, _) = armed_at(&cfg, &stowed_with(SWEPT_FROM));
        let (_, stretch) = floor_move_clock(
            &cfg,
            &last_targets(&swept),
            &inboard_sweep(0.8, [0.8, 0.8]),
            FLOOR_TICK_HZ,
        );
        let stretch = stretch.expect("a pair in step is de-phased");
        assert!(!stretch.span_stretched, "{stretch:?}");
        assert!(stretch.dephased, "{stretch:?}");

        let (_, stretch) = floor_move_clock(
            &cfg,
            &last_targets(&swept),
            &inboard_sweep(0.8, [0.05, 0.06]),
            FLOOR_TICK_HZ,
        );
        let stretch = stretch.expect("neither side carries its sweep in a twentieth of a second");
        assert!(stretch.span_stretched, "{stretch:?}");
        assert!(stretch.dephased, "{stretch:?}");
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
            floor_move_clock(&cfg, &last_targets(&state), &asked, FLOOR_TICK_HZ);
        let stretch = stretch.expect("neither side carries its sweep in a twentieth of a second");
        assert!(stretch.dephased, "{stretch:?}");
        let floor = duration_floor_s(SWEPT_FROM[0], cfg.max_step.antennas, FLOOR_TICK_HZ);
        // Under the closed form by the shortfall a period's travel has against
        // the peak rate it is written from, and no further.
        let carried = floor * (1.0 - window_shortfall(floor * FLOOR_TICK_HZ));
        for side in 0..2 {
            assert!(
                stretch.effective.antennas[side].as_secs_f64() >= carried,
                "side {side} runs on {:?}, under the {carried:.4} s its sweep needs",
                stretch.effective.antennas[side]
            );
        }
        let separated = dry_pass_separation(&cfg, &last_targets(&state), &floored, FLOOR_TICK_HZ)
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
            warp: WarpKind::MinJerk,
        };

        let (floored, stretch) =
            floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
        let stretch = stretch.expect("an unmet separation is reported");
        assert!(!stretch.dephased, "{stretch:?}");
        assert_eq!(stretch.effective, stretch.requested);
        assert_eq!(durations_of(&floored), stretch.requested);
        let pair = stretch.separation.expect("the pair was measured");
        assert!(!pair.met(cfg.phase.separation_rad), "{pair:?}");
        assert_eq!(pair.later, JointRef::AntennaRight);
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
            warp: WarpKind::Linear,
        };

        let (_, stretch) = floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
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
                warp: WarpKind::MinJerk,
            };
            assert_eq!(
                dry_pass_separation(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ),
                None,
                "{name}"
            );
            assert_eq!(
                floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ).1,
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

        let measurable = floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ)
            .1
            .expect("the mirrored pair is de-phased while the spans can be measured");
        assert!(measurable.dephased);

        // A bound of zero: every ratio against it is a number the pass refuses
        // to reason from, which is one of the three cases the contract names.
        cfg.max_step.legs = 0.0;
        let unmeasurable = floor_move_clock(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ);
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
            dry_pass_separation(&cfg, &last_targets(&state), &command, FLOOR_TICK_HZ)
                .is_some_and(|pair| !pair.met(cfg.phase.separation_rad))
        );
    }

    /// The durations a command carries, or the zero clock the commands that
    /// travel no path have.
    fn durations_of(command: &MotionCommand) -> MoveDurations {
        match command {
            MotionCommand::MoveTo { durations, .. } => *durations,
            MotionCommand::Hold | MotionCommand::Track(_) => MoveDurations::uniform(Duration::ZERO),
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
            assert_eq!(out.report.mode, MotionMode::Holding);
            assert!(out.report.present_fresh);
            assert!(!out.report.emitted);
            assert_eq!(out.report.command, CommandDisposition::None);
            assert_eq!(out.report.fault, None);
            assert!(out.report.fk.is_some(), "the present pose was solved");
        }
        assert!(state.present_min_margin > 0.024, "neutral clearance");
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
        assert_eq!(out.report.mode, MotionMode::Holding);
        assert_eq!(state.mode, MotionMode::Holding);
        assert!((99..=102).contains(&ticks), "took {ticks} ticks");
        assert_eq!(
            last_targets(&state).head_pose_body.translation.vector.z,
            0.19,
            "the endpoint is the target's own number"
        );

        let (expected, _) = joints_for(&cfg, &target);
        for leg in 0..6 {
            assert!(
                (last_goal(&state).legs[leg] - expected.legs[leg]).abs() < 1e-12,
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
            warp: WarpKind::MinJerk,
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
        assert_eq!(
            state.mode,
            MotionMode::Holding,
            "a bad command does not fault"
        );

        let good = MotionCommand::MoveTo {
            target: pose_at(0.19),
            durations: MoveDurations::uniform(secs(2.0)),
            warp: WarpKind::MinJerk,
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
        assert!(!(state.mode == MotionMode::Faulted));
        assert_eq!(state.mode, MotionMode::Holding, "and still commandable");

        // Which the next command proves: the abandoned move left a live state
        // behind, so a wind-down can be driven through the same tick. Against
        // the original bounds, because the tightened ones refuse every target
        // there is — that is what made the path fail in the first place.
        let back = MotionCommand::MoveTo {
            target: start,
            durations: MoveDurations::uniform(secs(2.0)),
            warp: WarpKind::MinJerk,
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
                    commanded: None,
                    command: None,
                    health: None,
                },
                &mut out,
            );
        }
        let standing = out.report.fault.expect("a run of missed reads faults");
        assert_eq!(standing, Fault::PositionFeedbackLost { misses: 3 });
        assert_eq!(
            fault::response(fault::kind(&standing)),
            ResponseKind::ImmediateAllTorqueOffToPark
        );

        let command = MotionCommand::MoveTo {
            target: pose_at(0.19),
            durations: MoveDurations::uniform(secs(2.0)),
            warp: WarpKind::MinJerk,
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
            assert_eq!(out.report.mode, MotionMode::Faulted);
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
                    commanded: None,
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
                    joint: JointRef::Leg2,
                    id: 13,
                    bits
                }),
                "bits {bits:#04x}"
            );
            assert_eq!(
                flags::contains(out.report.masked, JointRef::Leg2),
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
        let table: [(Fault, ResponseKind, bool); 9] = [
            (
                Fault::AntennaObstructed {
                    joint: JointRef::AntennaRight,
                    error: 0.5,
                    count: 30,
                },
                ResponseKind::DegradeAntennas,
                false,
            ),
            (
                Fault::AntennaServoFault {
                    joint: JointRef::AntennaLeft,
                    id: 18,
                    bits,
                },
                ResponseKind::DegradeAntennas,
                false,
            ),
            (
                Fault::HeadObstructed {
                    joint: JointRef::BodyYaw,
                    error: 0.5,
                    count: 10,
                },
                ResponseKind::SlowStowToRest,
                false,
            ),
            (
                Fault::HeadServoFault {
                    joint: JointRef::Leg2,
                    id: 13,
                    bits,
                },
                ResponseKind::MaskedSlowStowToPark,
                false,
            ),
            (
                Fault::PositionFeedbackLost { misses: 51 },
                ResponseKind::ImmediateAllTorqueOffToPark,
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
                ResponseKind::ImmediateAllTorqueOffToPark,
                true,
            ),
            (
                Fault::BusFailure {
                    source: BusFailureSource::Sequence(SeqError::NoAnswer {
                        context: StepContext::reg(
                            SeqStepKind::PinAndEnable,
                            13,
                            RegId::GoalPosition,
                        ),
                    }),
                },
                ResponseKind::ImmediateAllTorqueOffToPark,
                true,
            ),
            // The same condition, found by the loop driving a move rather than
            // by a sequencer. One response, whichever layer noticed.
            (
                Fault::BusFailure {
                    source: BusFailureSource::Transaction {
                        id: 13,
                        kind: WireFailure::Silent,
                    },
                },
                ResponseKind::ImmediateAllTorqueOffToPark,
                true,
            ),
            (
                Fault::TorqueOffUnconfirmed { id: 14 },
                ResponseKind::ImmediateAllTorqueOffToPark,
                true,
            ),
        ];
        for (fault, response, latches) in table {
            assert_eq!(fault::response(fault::kind(&fault)), response, "{fault}");
            assert_eq!(
                fault::latches(fault::kind(&fault)),
                latches,
                "{fault} leaves the tick commanding or it does not"
            );
        }
    }

    /// Every condition says one word, and no two say the same one.
    ///
    /// The slug is the join: the session's timeline row, the operator line and the
    /// daemon's fault cell all carry it, so a typo makes a condition nobody can
    /// alert on and a copy-pasted duplicate conflates two of them. Neither is
    /// visible by reading. Driven off a wildcard-free slot function, so a fault
    /// added to the doctrine cannot be left out of the table.
    #[test]
    fn every_fault_names_itself_and_no_other() {
        let table = [
            (
                Fault::AntennaObstructed {
                    joint: JointRef::AntennaRight,
                    error: 0.5,
                    count: 30,
                },
                "antenna_obstructed",
            ),
            (
                Fault::AntennaServoFault {
                    joint: JointRef::AntennaLeft,
                    id: 18,
                    bits: 0x20,
                },
                "antenna_servo_fault",
            ),
            (
                Fault::HeadObstructed {
                    joint: JointRef::BodyYaw,
                    error: 0.5,
                    count: 10,
                },
                "head_obstructed",
            ),
            (
                Fault::HeadServoFault {
                    joint: JointRef::Leg2,
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
                    source: BusFailureSource::Sequence(SeqError::NoAnswer {
                        context: StepContext::reg(
                            SeqStepKind::PinAndEnable,
                            13,
                            RegId::GoalPosition,
                        ),
                    }),
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
            assert_eq!(fault::slug(fault::kind(fault)), *slug, "{fault}");
            slugs.push(slug);
        }
        assert!(seen.iter().all(|named| *named), "a fault went unnamed");
        slugs.sort_unstable();
        let distinct = slugs.len();
        slugs.dedup();
        assert_eq!(slugs.len(), distinct, "two faults share a slug: {slugs:?}");

        // A wire failure the move loop found says the same word as one a
        // sequencer found. One condition, one name, so one alert rule covers
        // the machine however the trouble was noticed.
        assert_eq!(
            fault::slug(fault::kind(&Fault::BusFailure {
                source: BusFailureSource::Transaction {
                    id: 13,
                    kind: WireFailure::Silent,
                },
            })),
            "bus_failure"
        );
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
                let Fault::AntennaObstructed {
                    joint,
                    error,
                    count,
                } = fault
                else {
                    panic!("tick {n}: expected the antenna's obstruction, got {fault}");
                };
                assert_eq!(joint, JointRef::AntennaRight);
                assert!(error > cfg.tracking.threshold_rad, "{error} rad out");
                // The window that judged the held antenna's run, and the row it
                // was read from: nothing crossed here, and the antenna that
                // followed has no run at all.
                assert_eq!(count, cfg.tracking.ticks, "the ordinary window ran out");
                assert!(
                    flags::covers(out.report.newly_masked, JointGroup::Antennas),
                    "the pair goes out of service together"
                );
                assert_eq!(
                    flags::len(out.report.newly_masked),
                    2,
                    "and nothing else does"
                );
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
        assert_eq!(state.mode, MotionMode::Holding);
        assert!(!(state.mode == MotionMode::Faulted));
        assert!(flags::covers(state.masked, JointGroup::Antennas));
        assert_eq!(flags::len(state.masked), 2);
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
            flags::covers(report.newly_masked, JointGroup::Antennas),
            "the pair went with the leg: {}",
            flags::Names(report.newly_masked)
        );
        assert!(flags::contains(report.newly_masked, JointRef::Leg3));
        assert_eq!(flags::len(report.newly_masked), 3);
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
                joint: JointRef::Leg3,
                id: 14,
                bits: 0x20,
            })
        );
        assert_eq!(out.goal, None, "the offending period commands nothing");
        assert_eq!(out.report.mode, MotionMode::Holding, "no absorbing state");
        assert_eq!(
            flags::iter(out.report.newly_masked).collect::<Vec<_>>(),
            vec![JointRef::Leg3]
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
        assert!(flags::is_empty(out.report.newly_masked));
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
                joint: JointRef::Leg1,
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
            assert!(flags::is_empty(out.report.newly_masked), "poll {n}");
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
                joint: JointRef::Leg4,
                id: 15,
                bits: 0x04,
            }),
            "the mask expands to the new servo, not back onto the old one"
        );
        assert_eq!(
            flags::iter(out.report.newly_masked).collect::<Vec<_>>(),
            vec![JointRef::Leg4]
        );
        assert_eq!(flags::len(state.masked), 2);
    }

    /// The bits stay latched for the rest of the session; the raise happens
    /// once.
    ///
    /// The mask-scope rule as a reader of the reports sees it: entry into the
    /// mask is the raise, so a narration keyed on `newly_masked` takes one row
    /// per incident rather than one per poll.
    #[test]
    fn a_standing_fault_is_raised_once_and_not_at_poll_rate() {
        let cfg = MotionConfig::default();
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        let mut health = healthy_servos();
        health[7].bits = 0x20;

        let mut entries = 0;
        for n in 0..8 {
            let out = tick_with_health(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &pinned,
                &health,
                None,
            );
            if !flags::is_empty(out.report.newly_masked) {
                entries += 1;
            }
        }
        assert_eq!(entries, 1, "the pair enters the mask once");

        // And the same for a fault that stops the tick commanding: the report
        // repeats it every period, the raise happened once.
        let mut blind = Armed::new(
            &record_at(&cfg, &pinned, &Isometry3::translation(0.0, 0.0, 0.15)),
            JointFlags::NONE,
        );
        let mut out = TickOutputs::default();
        let mut raises = 0;
        let mut first_raised = None;
        for n in 0..u64::from(cfg.read_loss_ticks) + 4 {
            #[expect(
                clippy::cast_precision_loss,
                reason = "a tick count this small is exact in f64"
            )]
            let now = secs(n as f64 * 0.02);
            let stood_faulted = blind.mode == MotionMode::Faulted;
            motion_tick(
                &cfg,
                &mut blind,
                &TickInputs {
                    now,
                    period: PERIOD,
                    present: None,
                    commanded: None,
                    command: None,
                    health: None,
                },
                &mut out,
            );
            if !stood_faulted && blind.mode == MotionMode::Faulted {
                raises += 1;
                first_raised = Some(blind.mode);
            }
        }
        assert!(matches!(
            out.report.fault,
            Some(Fault::PositionFeedbackLost { .. })
        ));
        assert_eq!(
            raises, 1,
            "the tick entered the fault once, however long the reads stay lost"
        );
        assert!(
            blind.mode == MotionMode::Faulted,
            "the tick stopped commanding on the first raise and stays stopped"
        );
        assert_eq!(
            Some(blind.mode),
            first_raised,
            "the fault it stands on is the one it was raised with, evidence and all"
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
                joint: JointRef::AntennaLeft,
                id: 18,
                bits: 0x04,
            })
        );
        assert_eq!(out.report.fault, None, "the head is not in this");
        assert!(flags::covers(out.report.newly_masked, JointGroup::Antennas));
        assert!(out.report.mode == MotionMode::Moving, "still moving");

        let out = tick_with_health(&cfg, &mut state, secs(0.04), &pinned, &health, None);
        assert_eq!(
            out.report.degraded, None,
            "raised on entry to the mask only"
        );
        assert!(flags::is_empty(out.report.newly_masked));
        assert!(flags::covers(out.report.masked, JointGroup::Antennas));
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
        assert!(flags::covers(out.report.masked, JointGroup::Antennas));

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

        let stale = |state: &mut MotionSnap, n: u32| {
            let mut out = TickOutputs::default();
            motion_tick(
                &cfg,
                state,
                &TickInputs {
                    now: secs(f64::from(n) * 0.02),
                    period: PERIOD,
                    present: None,
                    commanded: None,
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
        assert!((state.mode == MotionMode::Faulted));
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
                armed: true,
                threshold_rad: 0.1,
                progress_min_rad: 0.01,
                ticks: 3,
                ..MotionConfig::default().tracking
            },
            ..MotionConfig::default()
        };
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        settle(&cfg, &mut state, &pinned);

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
        let held = last_goal(&state);
        motion_tick(
            &cfg,
            &mut state,
            &TickInputs {
                now: secs(0.08),
                period: PERIOD,
                present: None,
                commanded: Some(&held),
                command: None,
                health: None,
            },
            &mut out,
        );
        assert!(!out.report.present_fresh);
        assert_eq!(out.report.tracking_count, 2);

        let out = tick_with(&cfg, &mut state, secs(0.1), &lagging, None);
        let Some(Fault::HeadObstructed { joint, error, .. }) = out.report.fault else {
            panic!(
                "expected the head's obstruction, got {:?}",
                out.report.fault
            );
        };
        assert_eq!(
            joint,
            JointRef::Leg3,
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

    /// Every joint's own residual rides the report, in bus order.
    ///
    /// The worst of them is what a fault names, but the nine are what a run is
    /// judged by offline: which joint stood off its prediction, and by how far,
    /// is the figure a report prints against the threshold. A single worst
    /// figure per tick would hide every joint but one.
    #[test]
    fn the_report_carries_each_joint_s_own_residual() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);
        // The machine standing on its goal, so each prediction is where the
        // joint is and the offsets below are the whole of every residual.
        settle(&cfg, &mut state, &pinned);

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
        let errors = out.report.tracking_residuals.expect("a live tick measures");
        for (row, ((joint, angle), (_, goal))) in
            off.joints().into_iter().zip(pinned.joints()).enumerate()
        {
            assert!(
                (errors[row] - (angle - goal).abs()).abs() < 1e-12,
                "{} reports {} against {}",
                Name(joint),
                errors[row],
                (angle - goal).abs()
            );
        }
        assert_eq!(
            out.report.tracking_worst(),
            Some((JointRef::Leg5, 0.12)),
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
                commanded: None,
                command: None,
                health: None,
            },
            &mut stale,
        );
        assert_eq!(stale.report.tracking_residuals, None);
    }

    /// Run the prediction up to where the machine is standing, on a machine
    /// standing exactly where it was pinned.
    ///
    /// Every case below that displaces a joint by hand needs this first. A
    /// prediction is seeded from a reading and then stepped from the setpoints
    /// the driver held, so it takes a full dead time of setpoints before it is
    /// running at all — and until it is, every row is re-seeded to wherever the
    /// joint reads and no residual can exist. These ticks are all at instant
    /// zero, so a case that follows them starts its own clock where it would
    /// have anyway.
    fn settle(cfg: &MotionConfig, state: &mut MotionSnap, pinned: &JointVector) {
        for _ in 0..=RESPONSE_DEAD_SAMPLES {
            let out = tick_with(cfg, state, secs(0.0), pinned, None);
            assert_eq!(out.report.fault, None, "a machine on its goal is healthy");
            assert_eq!(out.report.tracking_count, 0, "and has no run open");
        }
    }

    /// A tracking configuration with everything else left at its default.
    fn tracking_cfg(threshold_rad: f64, progress_min_rad: f64, ticks: u32) -> MotionConfig {
        MotionConfig {
            tracking: TrackingFaultConfig {
                armed: true,
                threshold_rad,
                progress_min_rad,
                ticks,
                ..MotionConfig::default().tracking
            },
            ..MotionConfig::default()
        }
    }

    /// A machine that runs the same profile the tick predicts, at a fraction of
    /// its speed.
    ///
    /// The shape a servo's error actually has: its generator ramps toward each
    /// setpoint at the commissioned acceleration and cap, so a joint chasing
    /// content faster than the profile falls behind the *goal* by however far
    /// the content outran it — and stays with the *prediction* to whatever
    /// extent it is doing its own generator's work. `pace` is that extent: 1.0
    /// is a healthy servo and leaves no residual at all, and anything less is
    /// the servo this fault has to tolerate rather than raise on — voltage sag,
    /// a worn gearbox under load — which accumulates residual on a saturated
    /// move and closes it once the generator stops.
    ///
    /// The machine is driven from the prediction rather than from the goal
    /// because that is what a servo is: the generator's trajectory is the thing
    /// the shaft is chasing, and the goal is only what the generator was told.
    struct Plant {
        /// Every goal written, oldest first.
        goals: Vec<JointVector>,
        /// Where the machine's own generator stands, per row.
        modelled: [Predicted; ROW_COUNT],
        /// Where the machine reads, per row.
        position: JointVector,
        /// The plant this machine is: the tick's own, with both limits taken
        /// down to the fraction of the profile these servos manage.
        plant: GroupPlants,
        /// The worst residual any tick measured.
        worst: f64,
        /// The longest run any tick reported.
        longest: u32,
        /// The first degrade any tick reported, if one did.
        ///
        /// An antenna whose window runs out never reaches `report.fault` — the
        /// answer is the pair going out of service — so a chase judged on the
        /// fault alone would pass over a machine that had lost its antennas.
        degraded: Option<Fault>,
        /// Whether the plan ever reached its endpoint. Kept here because the
        /// machine goes on moving after the clock ends — a servo slower than
        /// its profile arrives late — so the tick that reports the completion
        /// is not the last one of the run.
        completed: bool,
        /// Rows a hand is holding: they stand where they are whatever the
        /// setpoint says.
        jammed: JointFlags,
        /// Ticks issued so far, at 50 Hz.
        tick: u32,
    }

    /// The three configured generators with both limits taken down to `pace` of
    /// what the registers say, which is how a scripted machine is made slower
    /// than its own profile.
    fn paced(plant: &GroupPlants, pace: f64) -> GroupPlants {
        let slow = |model: PlantModel| PlantModel {
            v_max: pace * model.v_max,
            a_max: pace * model.a_max,
            lag_alpha: model.lag_alpha,
        };
        GroupPlants {
            legs: slow(plant.legs),
            yaw: slow(plant.yaw),
            antennas: slow(plant.antennas),
        }
    }

    impl Plant {
        fn new(cfg: &MotionConfig, start: &JointVector, pace: f64) -> Self {
            let mut modelled = [Predicted::default(); ROW_COUNT];
            for (row, (_, angle)) in modelled.iter_mut().zip(start.joints()) {
                row.generator = angle;
                row.position = angle;
            }
            Self {
                goals: vec![*start],
                modelled,
                position: *start,
                plant: paced(&cfg.plant, pace),
                worst: 0.0,
                longest: 0,
                degraded: None,
                completed: false,
                jammed: JointFlags::NONE,
                tick: 0,
            }
        }

        /// Put a hand on these rows: a jammed row holds where it stands,
        /// whatever it is asked for, and its generator is at rest for as long
        /// as the hand is on it.
        ///
        /// The machine's own state and not a held reading: a row jammed here
        /// stops where the plant had got to, which is what an obstruction is to
        /// a position loop, and a held reading would instead snap back to the
        /// modelled position the moment the hand came off.
        fn jam(&mut self, rows: JointFlags) {
            self.jammed |= rows;
            for joint in flags::iter(rows) {
                if let Some(row) = row(joint) {
                    self.modelled[row].velocity = 0.0;
                }
            }
        }

        /// Take the hand off these rows: they resume toward the setpoint the
        /// driver is holding, from rest.
        ///
        /// From rest, not from the speed they had: a held row's generator
        /// resumes from a standstill, so the first `progress_min_rad` costs
        /// three periods of ramp — and nothing else, since the setpoint stood
        /// still through the hold. Nothing is zeroed here because the jam
        /// already did it and a held row is not stepped.
        fn release(&mut self, rows: JointFlags) {
            self.jammed = flags::without(self.jammed, rows);
        }

        /// The goal this machine is holding: the last one the tick decided.
        ///
        /// The one reading of it, because the setpoint the plant steps toward,
        /// the setpoint a streamed sweep advances from and the arrival the
        /// tail waits on are the same vector, and three indexings of the same
        /// vector are three chances to disagree about which one it is.
        fn held(&self) -> JointVector {
            *self.goals.last().expect("a plant always holds a goal")
        }

        /// One period of the machine: every row's generator steps toward the
        /// setpoint the driver is holding, at whatever fraction of the profile
        /// this machine manages.
        ///
        /// The setpoint is the goal written on the previous period — a driver
        /// holds what it was last given — and the dead time on top of that is
        /// the tick's own, so this machine answers a goal one period after it
        /// was emitted and the tick predicts it a dead time later still.
        ///
        /// A move the profile can carry leaves no residual whatever the
        /// fraction, because the cap is not what is binding. It is a saturated
        /// move that separates a slow servo from a healthy one: the slow one
        /// ends `(1 - pace)` of the distance behind, and closes that once the
        /// generator it is chasing stops.
        fn advance(&mut self) -> JointVector {
            let held = self.held();
            for ((id, target), modelled) in held.joints().into_iter().zip(self.modelled.iter_mut())
            {
                if flags::contains(self.jammed, id) {
                    continue;
                }
                self.plant.for_joint(id).step(modelled, target);
                self.position.set(id, modelled.position);
            }
            self.position
        }

        /// One tick, taking `command` if there is one.
        fn step(
            &mut self,
            cfg: &MotionConfig,
            state: &mut MotionSnap,
            command: Option<&MotionCommand>,
        ) -> TickOutputs {
            let present = self.advance();
            let now = secs(f64::from(self.tick) * 0.02);
            let out = tick_with(cfg, state, now, &present, command);
            self.tick += 1;
            if let Some((_, error)) = out.report.tracking_worst() {
                self.worst = self.worst.max(error);
            }
            self.longest = self.longest.max(out.report.tracking_count);
            if self.degraded.is_none() {
                self.degraded = out.report.degraded;
            }
            self.completed |= out.report.completed;
            self.goals.push(last_goal(state));
            out
        }

        /// One period of a mirrored antenna sweep: a `Track` setpoint `step`
        /// radians further round than the goal the machine is holding, the
        /// right antenna one way and the left the other.
        ///
        /// The one statement of what a swept period is. [`Plant::sweep`]
        /// streams it, and the two cases that watch the ticks either side of a
        /// threshold crossing take it a period at a time, because they need
        /// the content to go on arriving while they read the report. A second
        /// spelling of the mirroring would let a case that wanted a different
        /// gesture change one of them and test another.
        fn swept(&self, step: f64) -> MotionCommand {
            let goal = self.held();
            let read = |id| goal.get(id).expect("an antenna is a bus row");
            MotionCommand::Track(antennas_at([
                read(JointRef::AntennaRight) + step,
                read(JointRef::AntennaLeft) - step,
            ]))
        }

        /// Stream a mirrored antenna sweep as content: one `Track` setpoint a
        /// period, each `step` radians further round than the last, `periods`
        /// of them, stopping early if anything is raised.
        ///
        /// Content and not a planned move, because no move this crate plans
        /// outruns the antennas' commissioned generator: `floor_move_clock`
        /// sizes a plan to the per-tick step bound and min-jerk shaping leaves
        /// its peak inside what the generator carries, so a plan is followed
        /// rather than chased. A streamed setpoint is bounded by the step guard
        /// alone, at better than twice that cap, which is what the clip library
        /// sends and what leaves a generator saturated for as long as the
        /// content runs.
        ///
        /// A `Track` target names an angle rather than a direction to resolve,
        /// so the sweep goes on round for as far as it is streamed.
        fn sweep(
            &mut self,
            cfg: &MotionConfig,
            state: &mut MotionSnap,
            periods: u32,
            step: f64,
        ) -> TickOutputs {
            let mut out = TickOutputs::default();
            for _ in 0..periods {
                let command = self.swept(step);
                out = self.step(cfg, state, Some(&command));
                if out.report.fault.is_some() || out.report.degraded.is_some() {
                    break;
                }
                // The premise of every case this streams for is that the
                // content outran the servo. A setpoint the tick refused leaves
                // the goal where it was, so the sweep would go on looping,
                // command nothing and report nothing wrong -- vacuous before
                // it went red, and the residual assertion after the loop would
                // name the plant for what a rejection did.
                assert_eq!(
                    out.report.command,
                    CommandDisposition::Tracked,
                    "the sweep's setpoint has to be taken for this content to outrun anything"
                );
            }
            out
        }

        /// Go on ticking, commanding nothing, until the machine has arrived on
        /// the setpoint the stream left — or until something is raised.
        ///
        /// The tail is the point for a servo slower than its profile: the
        /// residual is largest as the generator arrives and the shaft has not,
        /// and what closes it is the periods after the content stops.
        fn settle(&mut self, cfg: &MotionConfig, state: &mut MotionSnap) -> TickOutputs {
            let mut out = TickOutputs::default();
            for _ in 0..2000 {
                out = self.step(cfg, state, None);
                if out.report.fault.is_some() || out.report.degraded.is_some() {
                    break;
                }
                // Nothing is commanded here, so anything else is a tick that
                // decided something this tail was not asking it to.
                assert_eq!(
                    out.report.command,
                    CommandDisposition::None,
                    "the tail commands nothing"
                );
                let goal = self.held();
                let arrived = goal
                    .joints()
                    .into_iter()
                    .zip(self.position.joints())
                    .all(|((_, goal), (_, angle))| (goal - angle).abs() < 1e-9);
                if arrived {
                    break;
                }
            }
            out
        }
    }

    /// A tick, and the run up to it, that found nothing wrong with the machine.
    ///
    /// Both halves are needed: `report.fault` is the head's answer and the
    /// antennas' is a degrade, so a chase asserted on the fault alone stays
    /// green over a pair taken out of service.
    fn nothing_wrong(machine: &Plant, out: &TickOutputs, what: &str) {
        assert_eq!(out.report.fault, None, "{what}");
        assert_eq!(out.report.degraded, None, "{what}");
        assert_eq!(out.report.newly_masked, JointFlags::NONE, "{what}");
        assert_eq!(
            machine.degraded, None,
            "{what}: nothing was taken out of service on the way, either"
        );
    }

    fn move_to(target: JointTargets, duration: Duration) -> MotionCommand {
        MotionCommand::MoveTo {
            target,
            durations: MoveDurations::uniform(duration),
            warp: WarpKind::MinJerk,
        }
    }

    /// The step a streamed antenna sweep advances by each period, radians.
    ///
    /// Just inside the shipped per-tick step bound, and better than twice what
    /// the antennas' commissioned generator carries in a period, which is what
    /// leaves that generator saturated for as long as the sweep runs.
    const SWEEP_STEP_RAD: f64 = 0.6;

    /// How many periods of it every pace case streams.
    ///
    /// Long enough that the shaft of a servo at 0.55 of its profile ends
    /// better than twice the shipped threshold behind its own generator, and
    /// short enough to leave the tail inside the tick budget.
    const SWEEP_PERIODS: u32 = 40;

    /// A slow servo, in each of the two configurations this fault has to hold
    /// in.
    ///
    /// The first is tuned by hand — a threshold and a progress minimum well
    /// under the shipped ones, which separates the arithmetic from the numbers
    /// the machine happens to ship with. The second is the shipped numbers,
    /// armed. Both drive the same gesture, the streamed sweep above, because it
    /// is a saturated move that makes a slow servo visible at all: this one
    /// ends `(1 - pace)` of the generator's own travel behind it, which is
    /// better than twice either threshold, and closes that once the content
    /// stops. A servo running at 0.55 of its profile has to survive both.
    fn regimes() -> [(MotionConfig, f64); 2] {
        [
            (tracking_cfg(0.02, 0.002, 10), 0.55),
            (
                MotionConfig {
                    tracking: TrackingFaultConfig {
                        armed: true,
                        ..MotionConfig::default().tracking
                    },
                    ..MotionConfig::default()
                },
                0.55,
            ),
        ]
    }

    /// The fault a slow servo must not raise: a joint sitting well past the
    /// threshold for the whole of a move, pacing its own generator the entire
    /// time and closing on it once the generator stops, is a servo doing its
    /// best rather than an obstructed one.
    #[test]
    fn a_lagging_but_closing_joint_never_faults() {
        for (cfg, pace) in regimes() {
            let start = JointTargets::default();
            let (mut state, pinned) = armed_at(&cfg, &start);

            let mut machine = Plant::new(&cfg, &pinned, pace);
            let out = machine.sweep(&cfg, &mut state, SWEEP_PERIODS, SWEEP_STEP_RAD);
            nothing_wrong(&machine, &out, "a pacing joint is a tracking joint");
            let out = machine.settle(&cfg, &mut state);

            nothing_wrong(&machine, &out, "and it closes on the generator it chased");
            assert!(
                machine.worst > 2.0 * cfg.tracking.threshold_rad,
                "the residual has to clear the threshold for this to test anything: {}",
                machine.worst
            );
            assert!(
                machine.longest > 0 && machine.longest < cfg.tracking.ticks,
                "runs opened and were closed by progress, not by luck: {}",
                machine.longest
            );
        }
    }

    /// The shipped configuration, armed: the starting point for every pace-rule
    /// case.
    fn armed_shipped() -> MotionConfig {
        MotionConfig {
            tracking: TrackingFaultConfig {
                armed: true,
                ..MotionConfig::default().tracking
            },
            ..MotionConfig::default()
        }
    }

    /// Whether any row this tick measured stands further off its prediction
    /// than the screen.
    fn past_threshold(cfg: &MotionConfig, out: &TickOutputs) -> bool {
        out.report.tracking_residuals.is_some_and(|residuals| {
            residuals
                .iter()
                .any(|residual| *residual > cfg.tracking.threshold_rad)
        })
    }

    /// The other side of the pace rule, and the reason it is a fraction of the
    /// generator's motion rather than a distance: a servo running at less than
    /// `pace_min` of its own profile does fault, a window after it crosses the
    /// threshold.
    ///
    /// Without this the rule has no failing direction. Every case that raises
    /// on a jam has a joint whose generator is not moving either, so `pacing`
    /// is false in them whatever `pace_min` says; a `pace_min` set to a
    /// twentieth, or a pace term weakened to "moved at all in the right
    /// direction", would leave all of them green. This is the case that fails
    /// when the rule stops discriminating: the joint moves the whole time, in
    /// the direction of its generator, and is faulted for moving too slowly.
    #[test]
    fn a_servo_under_the_pace_floor_faults_a_window_after_it_crosses() {
        let cfg = armed_shipped();
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        let mut machine = Plant::new(&cfg, &pinned, 0.3);

        let mut crossed = None;
        let mut raised = None;
        for n in 0..2000u32 {
            let command = machine.swept(SWEEP_STEP_RAD);
            let out = machine.step(&cfg, &mut state, Some(&command));
            if crossed.is_none() && past_threshold(&cfg, &out) {
                crossed = Some(n);
            }
            if out.report.degraded.is_some() || out.report.fault.is_some() {
                raised = Some((n, out.report.degraded, out.report.fault));
                break;
            }
        }

        let crossed = crossed.expect("a servo at 0.3 of the profile falls past the threshold");
        let (at, degraded, fault) = raised.expect("and is not tolerated there");
        assert_eq!(
            at,
            crossed + cfg.tracking.ticks - 1,
            "the window opened on the tick the residual crossed and ran out {} ticks later, \
             neither early nor late",
            cfg.tracking.ticks
        );
        assert_eq!(fault, None, "an antenna's answer is the pair, not a fault");
        assert!(
            matches!(degraded, Some(Fault::AntennaObstructed { .. })),
            "the pair went out of service over the antenna that could not keep up: {degraded:?}"
        );
    }

    /// And pacing stops when the joint does. A servo at 0.6 of its profile
    /// carries the whole sweep, and the same servo with a hand on it faults —
    /// so what keeps a slow joint in service is its motion and not its speed
    /// setting.
    ///
    /// The pair of runs is the point: one fixture, one sweep, one difference.
    #[test]
    fn a_pacing_joint_that_is_stopped_faults() {
        let cfg = armed_shipped();

        // Ahead of the jam: the same servo, unobstructed, is tolerated for the
        // whole sweep and the settle after it.
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        let mut machine = Plant::new(&cfg, &pinned, 0.6);
        let out = machine.sweep(&cfg, &mut state, SWEEP_PERIODS, SWEEP_STEP_RAD);
        nothing_wrong(&machine, &out, "0.6 of the profile is a servo, not a hand");
        let out = machine.settle(&cfg, &mut state);
        nothing_wrong(&machine, &out, "and it closes once the content stops");
        assert!(
            machine.worst > cfg.tracking.threshold_rad,
            "and it was past the screen while it ran: {}",
            machine.worst
        );

        // The same run with a hand on one antenna from the tick its residual
        // first crosses the screen.
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        let mut machine = Plant::new(&cfg, &pinned, 0.6);
        let mut crossed = None;
        let mut raised = None;
        for n in 0..2000u32 {
            let command = machine.swept(SWEEP_STEP_RAD);
            let out = machine.step(&cfg, &mut state, Some(&command));
            if crossed.is_none() && past_threshold(&cfg, &out) {
                crossed = Some(n);
                machine.jam(flags::bit(JointRef::AntennaRight));
            }
            if out.report.degraded.is_some() || out.report.fault.is_some() {
                raised = Some((n, out.report.degraded));
                break;
            }
        }
        let crossed = crossed.expect("the servo falls past the threshold on the way out");
        let (at, degraded) = raised.expect("a jammed joint paces nothing, whatever its speed");
        assert!(
            matches!(
                degraded,
                Some(Fault::AntennaObstructed {
                    joint: JointRef::AntennaRight,
                    ..
                })
            ),
            "the hand's own antenna is the one named: {degraded:?}"
        );
        assert!(
            at <= crossed + 2 * cfg.tracking.ticks,
            "and it was answered within two windows of the hand going on: crossed at {crossed}, \
             raised at {at}"
        );
    }

    /// A move that turns the body a stated fraction of the way to the profile
    /// velocity at its fastest.
    ///
    /// The body yaw rather than a leg or an antenna, for two reasons: it is the
    /// one row a target names directly, so the rate below is arithmetic rather
    /// than the output of an inverse solve, and its answer is a head
    /// obstruction — which holds the tick and goes on judging. An antenna's
    /// answer is the pair going out of service, and a masked row is judged
    /// against nothing, so no cadence could be measured on one.
    fn yaw_move(cfg: &MotionConfig, distance_rad: f64, fraction: f64) -> MotionCommand {
        // A min-jerk path peaks at fifteen eighths of its average rate.
        let periods = MIN_JERK_PEAK_RATE * distance_rad / (fraction * cfg.plant.yaw.v_max);
        move_to(
            JointTargets {
                body_yaw: distance_rad,
                ..JointTargets::default()
            },
            secs(periods * 0.02),
        )
    }

    /// One run of the re-raise fixture: a hand on the yaw crank under `command`,
    /// let go of `release_after` ticks past the first raise if it is ever let go
    /// of at all.
    ///
    /// Answers every tick a head obstruction was raised on with the run length
    /// it carried, and where the jammed row's modelled trajectory and its
    /// setpoint stood on each tick. The machine paces its profile exactly, so
    /// nothing here is about a slow servo: what is being measured is what the
    /// tick does with a joint that has stopped and a driver that goes on
    /// holding the goal.
    fn re_raises(
        cfg: &MotionConfig,
        command: &MotionCommand,
        jam_at: u32,
        release_after: Option<u32>,
        budget: u32,
    ) -> Cadence {
        let (mut state, pinned) = armed_at(cfg, &JointTargets::default());
        let mut machine = Plant::new(cfg, &pinned, 1.0);
        let yaw = flags::bit(JointRef::BodyYaw);
        let mut raises: Vec<(u32, u32)> = Vec::new();
        let mut trajectory = Vec::new();
        let mut reading = Vec::new();
        let mut held = Vec::new();
        for n in 0..budget {
            if n == jam_at {
                machine.jam(yaw);
            }
            if let (Some(after), Some((first, _))) = (release_after, raises.first().copied())
                && n == first + after
            {
                machine.release(yaw);
            }
            let out = machine.step(cfg, &mut state, (n == 0).then_some(command));
            trajectory.push(prediction_of(&state, JointRef::BodyYaw));
            reading.push(machine.position.body_yaw);
            held.push(last_goal(&state).body_yaw);
            if let Some(Fault::HeadObstructed {
                joint: JointRef::BodyYaw,
                count,
                ..
            }) = out.report.fault
            {
                raises.push((n, count));
            }
        }
        Cadence {
            raises,
            trajectory,
            reading,
            held,
        }
    }

    /// What one run of [`re_raises`] leaves behind.
    struct Cadence {
        /// Every tick a head obstruction was raised on, with the run length the
        /// report carried.
        raises: Vec<(u32, u32)>,
        /// Where the jammed row's modelled trajectory stood after each tick.
        trajectory: Vec<Predicted>,
        /// Where the jammed row itself read on each tick.
        reading: Vec<f64>,
        /// The yaw setpoint the driver was left holding after each tick.
        held: Vec<f64>,
    }

    /// A hand that stays on the head is answered every window, out of the hold,
    /// for as long as it stays there.
    ///
    /// The raise leaves the tick holding rather than faulted, and holding is not
    /// silence: the keep-alive goes on publishing the last goal that went out,
    /// so the driver goes on writing it and the joint the hand is on stays
    /// past the threshold with nothing closing and nothing pacing. The run that
    /// reopens on the tick after the raise therefore runs out exactly `ticks`
    /// later, and the next, and the next. That cadence is what the session's
    /// second raise comes out of, and it is asserted here rather than argued.
    ///
    /// The goal runs at half the profile velocity so that the prediction is
    /// never saturated and lands on the held setpoint within a few periods of
    /// the hold — pinned below, because a prediction still running would make
    /// the cadence a fact about the goal rather than about the window.
    #[test]
    fn a_persisting_obstruction_re_raises_every_window() {
        let cfg = armed_shipped();
        let ticks = cfg.tracking.ticks;
        let run = re_raises(&cfg, &yaw_move(&cfg, 1.5, 0.5), 20, None, 260);
        let raises = run.raises;

        assert!(
            raises.len() >= 3,
            "a hand that stays on is answered again and again: {raises:?}"
        );
        for pair in raises.windows(2) {
            let [(previous, _), (at, count)] = [pair[0], pair[1]];
            assert_eq!(
                at - previous,
                ticks,
                "a window opened on the tick after the raise at {previous} and ran out {ticks} \
                 ticks later, not on some cadence of the goal's"
            );
            assert_eq!(
                count, ticks,
                "and it is a whole window's worth of evidence each time"
            );
        }
        let (first, count) = raises[0];
        assert_eq!(count, ticks, "the first raise is a full window too");
        let reopened = run.trajectory[(first + ticks) as usize];
        assert_eq!(
            reopened.velocity, 0.0,
            "by the tick the second window runs out the prediction has landed on the setpoint \
             the hold goes on holding: the re-raise is the joint's, not a generator still running"
        );
    }

    /// And the hand coming off inside the window saves the engagement.
    ///
    /// A released joint sets off from rest and regains its first
    /// `progress_min_rad` after `pass_cycles` of that distance — the ramp
    /// alone, since the setpoint stood still through the hold and a release
    /// waits on nothing. The run that reopened on the tick after the raise
    /// counts that tick as one, so the restart has to land no later than a
    /// whole window past it: a release inside
    /// `raise + ticks - pass_cycles(progress_min_rad)` is answered with
    /// nothing further, and a release as late as the tick the window runs out
    /// on is answered with that window's raise and nothing after it.
    ///
    /// What restarts the run here is *pacing*, not closing, and that is a fact
    /// about this fixture rather than about the bound. The anchor is set on the
    /// reopen tick, where the prediction is still braking onto the held goal, so
    /// closing would need the joint to regain the progress minimum *and* make
    /// up whatever the prediction moved on afterwards. It paces instead: the
    /// ramp covers `pace_min` of that remaining travel well inside the slack
    /// the release offset leaves, which the case measures rather than assumes.
    ///
    /// The bound is an expression and not an instant on purpose. It moves with
    /// the profile and the progress minimum, and a case that stated the period
    /// it currently works out to would go on passing after either moved. The
    /// period on which recovery actually stops being possible is one further
    /// out and sits on a coincidence of the shipped pair; nothing here pins it.
    #[test]
    fn a_release_inside_the_window_ends_the_re_raising() {
        let cfg = armed_shipped();
        let ticks = cfg.tracking.ticks;
        let command = yaw_move(&cfg, 1.5, 0.5);
        let recovery = cfg.plant.yaw.pass_cycles(cfg.tracking.progress_min_rad) as u32;
        let bound = ticks - recovery;

        let inside = re_raises(&cfg, &command, 20, Some(bound - 1), 260);
        assert_eq!(
            inside.raises.len(),
            1,
            "let go of {} ticks after the raise — a period inside the bound — the joint regains \
             the progress minimum in time and nothing else is raised: {:?}",
            bound - 1,
            inside.raises
        );

        // The premise this fixture recovers under, measured: the prediction is
        // still braking onto the held goal at the anchor, and what it has left
        // to travel is what the released joint has to pace.
        let first = inside.raises[0].0;
        let anchor = inside.trajectory[(first + 1) as usize];
        assert!(
            anchor.velocity.abs() > 0.0,
            "the anchor tick's prediction is still moving, so this is the braking regime"
        );
        let remaining =
            (inside.trajectory[(first + ticks) as usize].position - anchor.position).abs();
        assert!(
            cfg.plant.yaw.pass_cycles(cfg.tracking.pace_min * remaining) <= recovery as usize + 2,
            "and pacing {} rad of it takes the ramp no longer than the two periods of slack the \
             release offset leaves",
            cfg.tracking.pace_min * remaining
        );

        let late = re_raises(&cfg, &command, 20, Some(ticks), 260).raises;
        assert_eq!(
            late.len(),
            2,
            "let go of on the tick the window runs out on, the hand is one raise too late — and \
             only one: {late:?}"
        );
        assert_eq!(
            late[1].0 - late[0].0,
            ticks,
            "that second raise is the reopened window's own, not a third thing"
        );
    }

    /// A hand that came off a joint the goal had left behind at the profile
    /// velocity is answered anyway, and this is the trade that costs.
    ///
    /// A released joint sets off from rest against a prediction still running at
    /// the cap. It cannot close — the residual is still growing — and over the
    /// window its ramp covers less than `pace_min` of what the generator covers
    /// at the cap, so it cannot pace either. The second raise lands regardless
    /// of the hand, and the engagement ends. Only once the joint is at the cap
    /// itself does pacing hold, which is why nothing is raised after it.
    ///
    /// What this pins is the width of the doctrine's trade: a grab of a window
    /// and a raise on a saturated move ends the session unless the hand comes
    /// off within a period or two of the raise — in effect, whether or not it
    /// then lets go. The recovery the case above measures needs a generator that
    /// had already stopped, or all but. This release is placed on that case's
    /// own offset, well clear of the couple of periods of grace a saturated move
    /// does leave, so what separates the two runs is the regime and not the
    /// instant.
    #[test]
    fn a_release_after_a_raise_on_a_saturated_move_still_re_raises() {
        let cfg = armed_shipped();
        let ticks = cfg.tracking.ticks;
        // Three times the profile velocity at its peak: the goal is far past
        // the prediction and staying there.
        let command = yaw_move(&cfg, 2.0, 3.0);
        let recovery = cfg.plant.yaw.pass_cycles(cfg.tracking.progress_min_rad) as u32;
        let run = re_raises(&cfg, &command, 5, Some(ticks - recovery - 1), 300);
        let raises = &run.raises;

        assert_eq!(
            raises.len(),
            2,
            "the release buys nothing and the second window is the last: {raises:?}"
        );
        assert_eq!(
            raises[1].0 - raises[0].0,
            ticks,
            "the second raise is the reopened window running out, on the tick it reaches {ticks}"
        );
        let first = raises[0].0 as usize;
        let at_first = run.trajectory[first];
        let braking = cfg.plant.yaw.v_max * cfg.plant.yaw.v_max / (2.0 * cfg.plant.yaw.a_max);
        assert!(
            at_first.velocity >= cfg.plant.yaw.v_max - 1e-12,
            "the premise: the generator is still running at the cap when the hand comes off, at \
             {} rad per period",
            at_first.velocity
        );
        assert!(
            (run.held[first] - at_first.position).abs() > braking,
            "and the setpoint it is chasing is more than the {braking} rad it needs to brake in"
        );
        // What silences the third window is the joint, at the cap by now,
        // pacing a generator that is still at the cap itself — not the
        // prediction landing, which with the setpoint that far ahead it does
        // not do for another forty periods.
        let anchor = (raises[1].0 + 1) as usize;
        let out = (raises[1].0 + ticks) as usize;
        let generator = run.trajectory[out].position - run.trajectory[anchor].position;
        let joint = run.reading[out] - run.reading[anchor];
        assert!(
            run.trajectory[out].velocity >= cfg.plant.yaw.v_max - 1e-12,
            "the generator has not stopped by the end of the third window"
        );
        assert!(
            joint * generator.signum() >= cfg.tracking.pace_min * generator.abs(),
            "and the joint covered {joint} rad of the generator's {generator} — at or over the \
             pace floor, which is what restarts the window"
        );
    }

    /// The detection this fault exists for: a goal that
    /// never applied, or a motor that cannot move, leaves the joint standing
    /// still with its goal beyond the threshold. Nothing closes, so the window
    /// runs out — on the tick the count reaches it, not one either side.
    #[test]
    fn a_stalled_joint_faults_at_exactly_the_window() {
        let cfg = tracking_cfg(0.1, 0.01, 5);
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        settle(&cfg, &mut state, &pinned);

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
                        joint: JointRef::BodyYaw,
                        error: 0.3,
                        count: cfg.tracking.ticks,
                    }),
                    "tick {n}"
                );
            }
        }
        assert!(
            state.mode == MotionMode::Holding,
            "the motors still command, so the stow that answers this is driven \
             from right here"
        );
    }

    /// The shipped configuration answers that same stall.
    ///
    /// The detector ships armed, so a joint standing still far outside every
    /// window is a head obstruction at exactly the shipped window -- and the
    /// machine keeps commanding, because the raise is non-latching: what it
    /// hands the session is the evidence for the stow that answers it, driven
    /// from right here. The measurement is reported either way, which is what a
    /// run is judged by offline.
    #[test]
    fn the_shipped_detector_raises_on_a_stall() {
        let cfg = MotionConfig::default();
        assert!(cfg.tracking.armed, "the shipped detector is armed");
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        settle(&cfg, &mut state, &pinned);

        let mut stuck = pinned;
        stuck.body_yaw += 2.0;

        for n in 1..=cfg.tracking.ticks {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &stuck, None);
            assert_eq!(out.report.tracking_count, n, "tick {n}");
            assert!(
                out.report
                    .tracking_residuals
                    .is_some_and(|residuals| residuals
                        .iter()
                        .any(|residual| *residual > cfg.tracking.threshold_rad)),
                "tick {n} measures the residual"
            );
            if n < cfg.tracking.ticks {
                assert_eq!(out.report.fault, None, "tick {n}");
            } else {
                assert_eq!(
                    out.report.fault,
                    Some(Fault::HeadObstructed {
                        joint: JointRef::BodyYaw,
                        error: 2.0,
                        count: cfg.tracking.ticks,
                    }),
                    "tick {n}"
                );
            }
        }
        assert!(
            state.mode == MotionMode::Holding,
            "and the machine is still under command"
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
        settle(&cfg, &mut state, &pinned);

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
                    joint: JointRef::BodyYaw,
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
        settle(&cfg, &mut state, &pinned);

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
        assert_eq!(joint, JointRef::BodyYaw);
    }

    /// Motion with no net progress is not progress either: a joint hunting
    /// around a standing offset covers ground every tick and closes none of it.
    #[test]
    fn oscillation_without_net_progress_faults() {
        let cfg = tracking_cfg(0.1, 0.01, 5);
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        settle(&cfg, &mut state, &pinned);

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
        assert_eq!(joint, JointRef::BodyYaw);
    }

    /// The progress minimum is a rate, and it is the line between the two: a
    /// joint closing less than it over a whole window faults, and the same
    /// crawl a shade faster runs indefinitely. Both sit far past the threshold
    /// throughout, so only the closing rate separates them.
    #[test]
    fn a_crawl_below_the_progress_minimum_faults_and_one_above_it_does_not() {
        let cfg = tracking_cfg(0.1, 0.01, 10);

        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        settle(&cfg, &mut state, &pinned);
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
                    joint: JointRef::BodyYaw,
                    ..
                })
            ),
            "expected a tracking fault, got {last:?}"
        );

        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        settle(&cfg, &mut state, &pinned);
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
        settle(&cfg, &mut state, &pinned);

        let mut stuck = pinned;
        stuck.body_yaw += 0.3;

        for n in 1..=3u32 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &stuck, None);
            assert_eq!(out.report.tracking_count, n);
        }

        // Ten periods of silence, with the driver still holding the setpoint it
        // had. The model is stepped through every one of them — the servo did
        // not stop moving because a read failed — and the goal here is where
        // the prediction already stands, so it is the run and not the
        // prediction that this pins as frozen.
        for n in 4..=13u32 {
            let mut out = TickOutputs::default();
            let held = last_goal(&state);
            motion_tick(
                &cfg,
                &mut state,
                &TickInputs {
                    now: secs(f64::from(n) * 0.02),
                    period: PERIOD,
                    present: None,
                    commanded: Some(&held),
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
                joint: JointRef::BodyYaw,
                error: 0.3,
                count: cfg.tracking.ticks,
            })
        );
    }

    /// One tick, with the reading and the held setpoint each stated: the two
    /// inputs the prediction is a walk over.
    ///
    /// The cases below drive them apart on purpose — a stale stretch is a
    /// setpoint with no reading, a gap is neither for several periods, and a
    /// driver holding nothing is a reading with no setpoint — so neither can
    /// come from the other the way [`tick_with`] has it.
    fn tick_over(
        cfg: &MotionConfig,
        state: &mut MotionSnap,
        now: Duration,
        present: Option<&JointVector>,
        commanded: Option<&JointVector>,
    ) -> TickOutputs {
        let mut out = TickOutputs::default();
        motion_tick(
            cfg,
            state,
            &TickInputs {
                now,
                period: PERIOD,
                present,
                commanded,
                command: None,
                health: None,
            },
            &mut out,
        );
        out
    }

    /// Where a row's modelled trajectory stands, as the state holds it.
    fn prediction_of(state: &MotionSnap, joint: JointRef) -> Predicted {
        let run = &state.tracking[row(joint).expect("a bus row")];
        Predicted {
            generator: run.generator,
            velocity: run.velocity,
            position: run.predicted,
        }
    }

    /// The same walk the tick makes, done here: one step-then-push per period,
    /// the step against the setpoint of `RESPONSE_DEAD_SAMPLES` periods ago.
    ///
    /// A second statement of the arithmetic, on purpose. What the cases below
    /// pin is that the tick's own walk through periods no sample attended
    /// arrives where stepping the plant directly does, so the comparison has to
    /// be against something other than the loop under test.
    fn walked(plant: &PlantModel, start: f64, held: f64, setpoints: &[f64]) -> Predicted {
        let mut ring = vec![held; RESPONSE_DEAD_SAMPLES];
        let mut state = Predicted {
            generator: start,
            velocity: 0.0,
            position: start,
        };
        for setpoint in setpoints {
            let target = ring.remove(0);
            plant.step(&mut state, target);
            ring.push(*setpoint);
        }
        state
    }

    /// A seeded row starts with its generator and its shaft both on the
    /// reading, and a class carrying a following lag is then predicted with the
    /// lag: the residual is read against a shaft that trails its generator, not
    /// against the generator.
    ///
    /// The whole of what the lag term buys sits in the difference between those
    /// two numbers, and the state carries both because the step needs where the
    /// generator got to. A row whose class has no lag reads one number twice.
    #[test]
    fn a_lagged_class_is_predicted_at_its_shaft_and_not_at_its_generator() {
        let lagged = GroupPlants::from_profiles(
            &GroupProfiles {
                antennas: ClassProfile {
                    following_lag_us: 30_000,
                    ..SHIPPED_PROFILES.antennas
                },
                ..SHIPPED_PROFILES
            },
            SHIPPED_PERIOD_NS,
        )
        .expect("a lagged antenna profile is three models");
        let mut slot = MotionSnapWire::new();
        let runs = &mut slot.clear_valid().tracking;
        let mut present = JointVector::default();
        present.set(JointRef::AntennaLeft, 0.25);
        tracking::seed(runs, &present);
        let index = row(JointRef::AntennaLeft).expect("a bus row");
        assert_eq!(runs[index].generator, 0.25, "the reading is the seed");
        assert_eq!(runs[index].predicted, 0.25, "for both of them");

        // Far enough that the generator is still at cruise eighty periods in:
        // what is measured is the steady-state lag, and an arrival would snap
        // the shaft onto the generator and measure nothing. Eighty because the
        // lag closes on its steady state geometrically and the assertion below
        // is to a nanoradian.
        let mut held = JointVector::default();
        held.set(JointRef::AntennaLeft, 40.25);
        for _ in 0..80 {
            tracking::predict(&lagged, &held, runs);
        }
        let run = &runs[index];
        assert!(
            run.generator - run.predicted > 0.0,
            "the shaft has to trail the generator under a lag: {} against {}",
            run.predicted,
            run.generator
        );
        // A period and a half of travel at the antennas' cap, which is what
        // 30 000 us is on the 20 ms grid.
        let behind = 1.5 * lagged.antennas.v_max;
        assert!(
            (run.generator - run.predicted - behind).abs() < 1e-9,
            "{} rad behind, against {behind} rad of travel",
            run.generator - run.predicted
        );

        // The same walk on a class with no lag leaves the two on each other.
        // Stated as a profile rather than taken from the shipped one, which
        // carries the antennas' measured lag.
        let plain_plants = GroupPlants::from_profiles(
            &GroupProfiles {
                antennas: ClassProfile {
                    following_lag_us: 0,
                    ..SHIPPED_PROFILES.antennas
                },
                ..SHIPPED_PROFILES
            },
            SHIPPED_PERIOD_NS,
        )
        .expect("an unlagged antenna profile is three models");
        let mut plain_slot = MotionSnapWire::new();
        let plain = &mut plain_slot.clear_valid().tracking;
        tracking::seed(plain, &present);
        for _ in 0..80 {
            tracking::predict(&plain_plants, &held, plain);
        }
        assert_eq!(plain[index].generator, plain[index].predicted);
    }

    /// The sustained figure is the worst a row never came back under for a
    /// whole window, so an excursion that closes inside one does not raise it
    /// and a plateau does.
    ///
    /// The two shapes side by side because they are the difference between the
    /// figure a screen is sized by and the figure the detector answers: a
    /// single period at 0.5 rad is a sample the sizing rule reads and the
    /// window rule does not, and five periods at 0.2 rad in a window of three
    /// are the other way round.
    #[test]
    fn a_sustained_residual_is_the_worst_a_row_never_came_back_under() {
        let row = |antenna: f64| {
            let mut residuals = [0.0; ROW_COUNT];
            residuals[row(JointRef::AntennaLeft).expect("a bus row")] = antenna;
            residuals
        };
        let spike: Vec<[f64; ROW_COUNT]> = [0.0, 0.0, 0.5, 0.0, 0.0].map(row).into();
        assert_eq!(
            sustained_residual(&spike, 3),
            0.0,
            "one period past the screen is held for no window"
        );
        let plateau: Vec<[f64; ROW_COUNT]> = [0.0, 0.2, 0.25, 0.2, 0.0].map(row).into();
        assert!(
            (sustained_residual(&plateau, 3) - 0.2).abs() < 1e-12,
            "three periods never under 0.2 rad is a sustained 0.2 rad"
        );
        // The floor is the window's own minimum, so a window is worth its
        // weakest period and not its strongest.
        assert!(
            (sustained_residual(&plateau, 2) - 0.2).abs() < 1e-12,
            "the pair at 0.2 and 0.25 is worth the 0.2"
        );
        // A recording shorter than a window held nothing for one.
        assert_eq!(sustained_residual(&spike, 9), 0.0);
    }

    /// A read outage advances the prediction on what the driver was told, so the
    /// comparison resumes against where the servo should now be.
    ///
    /// The servo did not stop moving because a read failed. A prediction frozen
    /// through the outage would judge the first live read against a trajectory
    /// several periods stale and could fault a healthy joint for having
    /// followed its setpoint; one that advanced is the whole reason the outage
    /// costs the comparison nothing. Both directions are here: the joint that
    /// moved with the plant opens no run, and the one that stayed where it was
    /// opens one.
    #[test]
    fn a_stale_stretch_advances_the_prediction_on_the_setpoint_the_driver_held() {
        let cfg = tracking_cfg(0.01, 0.002, 10);
        // Long enough that the walk gets past the dead time and then creeps
        // three periods clear of the threshold below.
        let stale = i32::try_from(RESPONSE_DEAD_SAMPLES).expect("a few samples") + 3;
        // The setpoint moves through the outage, by less than the profile
        // carries in a period, so a joint following it is following exactly.
        let creep = 0.01;

        for follows in [true, false] {
            let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
            settle(&cfg, &mut state, &pinned);

            let mut held = pinned;
            let mut setpoints = Vec::new();
            for period in 1..=stale {
                held.body_yaw = pinned.body_yaw + creep * f64::from(period);
                setpoints.push(held.body_yaw);
                let out = tick_over(
                    &cfg,
                    &mut state,
                    secs(f64::from(period) * 0.02),
                    None,
                    Some(&held),
                );
                assert_eq!(
                    out.report.tracking_residuals, None,
                    "period {period} measured nothing"
                );
            }

            let expected = walked(
                &cfg.plant.yaw,
                pinned.body_yaw,
                pinned.body_yaw,
                &setpoints[..usize::try_from(stale).expect("a few periods")],
            );
            let reached = prediction_of(&state, JointRef::BodyYaw);
            assert!(
                (reached.position - expected.position).abs() < 1e-12,
                "the outage left the prediction at {}, not the {} the plant reaches",
                reached.position,
                expected.position
            );
            assert!(
                reached.position - pinned.body_yaw > cfg.tracking.threshold_rad,
                "and it moved far enough for the read below to be a question: {}",
                reached.position - pinned.body_yaw
            );

            // Where the prediction stands after the live tick's own step: that
            // tick steps the model before it judges anything, so a joint that
            // followed its setpoint reads where the walk gets to with this
            // period included.
            let mut walk = setpoints.clone();
            walk.push(held.body_yaw);
            let after = walked(&cfg.plant.yaw, pinned.body_yaw, pinned.body_yaw, &walk);
            let mut present = pinned;
            if follows {
                present.body_yaw = after.position;
            }
            let out = tick_over(
                &cfg,
                &mut state,
                secs(f64::from(stale + 1) * 0.02),
                Some(&present),
                Some(&held),
            );
            assert_eq!(
                out.report.tracking_count > 0,
                !follows,
                "a joint that {} its setpoint through the outage",
                if follows {
                    "followed"
                } else {
                    "did not follow"
                }
            );
            assert_eq!(out.report.fault, None, "one live read is not a window");
        }
    }

    /// A slot the loop never attended is a period the servo went on moving
    /// through, up to the gap the model is walked through; past it, the
    /// prediction is re-seeded rather than guessed at.
    ///
    /// Both bounds, because both directions are wrong in their own way: a
    /// prediction that stopped at the first missed slot resumes against a stale
    /// trajectory, and one walked through an arbitrarily long gap judges a
    /// joint against a guess whose error grows with the gap. The ring goes with
    /// the prediction on the long gap, so the periods after it re-seed too
    /// rather than stepping a fresh seed toward a setpoint from before it.
    #[test]
    fn a_gap_is_walked_to_the_bound_and_re_seeded_past_it() {
        for gap in [MAX_GAP_PERIODS, MAX_GAP_PERIODS + 1] {
            let cfg = MotionConfig::default();
            let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
            settle(&cfg, &mut state, &pinned);

            // A setpoint a long way off, held for a full dead time, so the ring
            // the gap is walked over carries it and nothing else.
            let mut far = pinned;
            far.body_yaw += 1.0;
            for period in 1..=RESPONSE_DEAD_SAMPLES {
                tick_over(
                    &cfg,
                    &mut state,
                    secs(period as f64 * 0.02),
                    Some(&pinned),
                    Some(&far),
                );
            }
            let before = prediction_of(&state, JointRef::BodyYaw);

            let at = (RESPONSE_DEAD_SAMPLES + gap) as f64 * 0.02;
            let out = tick_over(&cfg, &mut state, secs(at), Some(&pinned), Some(&far));
            let reached = prediction_of(&state, JointRef::BodyYaw);
            let residual = out
                .report
                .tracking_residuals
                .expect("a live read measures the machine")
                [row(JointRef::BodyYaw).expect("a bus row")];

            if gap > MAX_GAP_PERIODS {
                assert_eq!(
                    reached.position, pinned.body_yaw,
                    "a gap past the bound re-seeds from the reading"
                );
                assert_eq!(reached.velocity, 0.0, "at rest, as a seed is");
                assert_eq!(residual, 0.0, "so nothing is measured against it");
                assert_eq!(
                    state.held_count, 1,
                    "and the ring went with it: all it holds is this period's setpoint, so the \
                     periods after the gap re-seed rather than stepping toward one from before it"
                );
            } else {
                let expected = walked(
                    &cfg.plant.yaw,
                    before.position,
                    far.body_yaw,
                    &vec![far.body_yaw; gap],
                );
                assert!(
                    (reached.position - expected.position).abs() < 1e-12,
                    "the gap of {gap} left the prediction at {}, not the {} the plant reaches",
                    reached.position,
                    expected.position
                );
                assert!(
                    residual > 0.1,
                    "and the joint that did not move with it is measured against it: {residual}"
                );
            }
        }
    }

    /// A driver holding nothing leaves the model nothing to chase: the
    /// prediction is re-seeded from the reading, and the reading is what it is
    /// then compared against.
    ///
    /// Before the first goal, after a release, behind a latched torque-off.
    /// Nothing is commanded, so a hand moving the joint is not an obstruction
    /// and a joint standing anywhere is not a fault — and the ring is emptied,
    /// so the periods until it is full again re-seed rather than stepping
    /// toward a setpoint nobody is holding any more.
    #[test]
    fn a_driver_holding_nothing_re_seeds_the_prediction() {
        let cfg = MotionConfig::default();
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        settle(&cfg, &mut state, &pinned);

        let mut far = pinned;
        far.body_yaw += 1.0;
        for period in 1..=10 {
            tick_over(
                &cfg,
                &mut state,
                secs(f64::from(period) * 0.02),
                Some(&pinned),
                Some(&far),
            );
        }
        let running = prediction_of(&state, JointRef::BodyYaw);
        assert!(
            (running.position - pinned.body_yaw).abs() > 0.05,
            "the prediction has to be away from the joint for this to say anything: {}",
            running.position
        );

        let yaw = row(JointRef::BodyYaw).expect("a bus row");
        let out = tick_over(&cfg, &mut state, secs(0.22), Some(&pinned), None);
        assert_eq!(
            prediction_of(&state, JointRef::BodyYaw).position,
            pinned.body_yaw,
            "the reading is the seed"
        );
        assert_eq!(
            out.report.tracking_residuals.expect("a live read")[yaw],
            0.0,
            "and a re-seeded row stands nowhere"
        );
        assert_eq!(state.held_count, 0, "with nothing left in the ring");

        // And it stays re-seeded until the ring is a dead time deep again.
        let dead = i32::try_from(RESPONSE_DEAD_SAMPLES).expect("a few samples");
        for period in 12..=11 + dead {
            let out = tick_over(
                &cfg,
                &mut state,
                secs(f64::from(period) * 0.02),
                Some(&pinned),
                Some(&far),
            );
            assert_eq!(
                out.report.tracking_residuals.expect("a live read")[yaw],
                0.0,
                "period {period} is still filling the ring"
            );
        }
        let out = tick_over(
            &cfg,
            &mut state,
            secs(f64::from(12 + dead) * 0.02),
            Some(&pinned),
            Some(&far),
        );
        assert!(
            out.report.tracking_residuals.expect("a live read")[yaw] > 0.0,
            "and then the comparison is running again"
        );
    }

    /// A held setpoint carrying a number nobody can place is a driver holding
    /// nothing, and costs a period of prediction rather than the engagement.
    ///
    /// The same trust boundary as the reading beside it, answered the same way.
    /// The alternative is what the slot screen exists to catch: one such
    /// setpoint stepped into the model makes the row's trajectory unplaceable,
    /// which is persisted, and the next execution's `resume` refuses the state
    /// -- so a single bad frame from the layer below would drop the engagement
    /// and leave the machine to the dead-man, with nothing said about it beyond
    /// a counter.
    #[test]
    fn a_held_setpoint_nobody_can_place_is_a_driver_holding_nothing() {
        let cfg = MotionConfig::default();
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        settle(&cfg, &mut state, &pinned);

        let mut far = pinned;
        far.body_yaw += 1.0;
        for period in 1..=6 {
            tick_over(
                &cfg,
                &mut state,
                secs(f64::from(period) * 0.02),
                Some(&pinned),
                Some(&far),
            );
        }
        state.resumes().expect("a running prediction is a state");

        let mut garbage = far;
        garbage.antennas[0] = f64::NAN;
        let out = tick_over(&cfg, &mut state, secs(0.14), Some(&pinned), Some(&garbage));
        assert_eq!(out.report.fault, None, "a bad frame is not a fault");
        assert_eq!(
            state.held_count, 0,
            "the ring is dropped, as it is for a driver holding nothing"
        );
        for run in state.tracking.iter() {
            assert!(
                run.predicted.is_finite() && run.velocity.is_finite(),
                "no trajectory was stepped toward it"
            );
        }
        state
            .resumes()
            .expect("and the slot the next execution reads is still a state");
    }

    /// A slow servo through a goal that reverses under it, in both the ways
    /// this system produces one: a move abandoned and reversed mid-flight, and
    /// two moves issued back to back the way `demo` does.
    ///
    /// The reversal is the case a residual could get wrong. The generator
    /// decelerates for the eight periods its acceleration needs and then comes
    /// back through the joint, so the residual falls to nothing and grows again
    /// on the other side, with the joint pacing it in the new direction the
    /// whole time — and a rule that read the second half as a joint running
    /// away would fault a machine turning round.
    #[test]
    fn a_lagging_joint_survives_a_goal_reversal() {
        for (cfg, pace) in regimes() {
            let start = JointTargets::default();

            // Back to back: the sweep back starts on the tick after the sweep
            // out stopped, with the machine still that many periods behind.
            let (mut state, pinned) = armed_at(&cfg, &start);
            let mut machine = Plant::new(&cfg, &pinned, pace);
            let out = machine.sweep(&cfg, &mut state, SWEEP_PERIODS, SWEEP_STEP_RAD);
            nothing_wrong(&machine, &out, "the sweep out faulted nothing");
            let out = machine.sweep(&cfg, &mut state, SWEEP_PERIODS, -SWEEP_STEP_RAD);
            nothing_wrong(&machine, &out, "the reversal faulted nothing");
            let out = machine.settle(&cfg, &mut state);
            nothing_wrong(&machine, &out, "and the settle after it faulted nothing");
            assert!(
                machine.worst > cfg.tracking.threshold_rad,
                "the machine stood off its prediction through both sweeps: {}",
                machine.worst
            );

            // Mid-flight: hold, then reverse, while the joint is furthest
            // behind its generator.
            let (mut state, pinned) = armed_at(&cfg, &start);
            let mut machine = Plant::new(&cfg, &pinned, pace);
            // Turned round once the residual is past the threshold, which is
            // where a reversal is hardest to tell from a joint that stopped:
            // the generator is about to come back through a joint that is
            // still carrying the old direction.
            let mut n = 0;
            while machine.worst <= cfg.tracking.threshold_rad {
                let command = machine.swept(SWEEP_STEP_RAD);
                let out = machine.step(&cfg, &mut state, Some(&command));
                nothing_wrong(&machine, &out, &format!("tick {n}"));
                n += 1;
                assert!(n < 500, "the residual never reached the threshold");
            }
            let held = machine.step(&cfg, &mut state, Some(&MotionCommand::Hold));
            assert_eq!(held.report.command, CommandDisposition::Held);
            let out = machine.sweep(&cfg, &mut state, SWEEP_PERIODS, -SWEEP_STEP_RAD);
            nothing_wrong(&machine, &out, "the mid-flight reversal faulted nothing");
            let out = machine.settle(&cfg, &mut state);
            nothing_wrong(&machine, &out, "and the settle after it faulted nothing");
            assert!(
                machine.worst > cfg.tracking.threshold_rad,
                "and it was behind when the goal turned round: {}",
                machine.worst
            );
        }
    }

    /// The configuration the residual's own blind spot needs: a step bound wide
    /// enough that one period's goal jumps clean over the joint, and a
    /// threshold tight enough that a whole radian of goal distance is not
    /// needed to reach it.
    fn crossing_cfg() -> MotionConfig {
        MotionConfig {
            max_step: JointStep {
                legs: 2.0,
                body_yaw: 2.0,
                antennas: 2.0,
            },
            tracking: TrackingFaultConfig {
                armed: true,
                threshold_rad: 0.01,
                progress_min_rad: 0.002,
                pace_min: 0.5,
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
            warp: WarpKind::Linear,
        }
    }

    /// A stall under goals that re-cross it faster than the window is long.
    ///
    /// A trajectory has momentum: a square-wave target reverses before the
    /// generator's velocity has come back through zero, so the trajectory
    /// ratchets away from the stalled joint period after period, and the
    /// residual grows until the window runs out.
    #[test]
    fn goals_re_crossing_a_stalled_joint_no_longer_hide_it() {
        let cfg = crossing_cfg();
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        settle(&cfg, &mut state, &pinned);

        let mut raised = None;
        let mut worst_gap: f64 = 0.0;
        for n in 0..60u32 {
            // A fresh one-period move every other period, the other way each
            // time: a move retargeted on every single period would never get
            // past its own start sample and emit no goal at all.
            let command = n.is_multiple_of(2).then(|| {
                let sign = if n % 4 == 0 { 1.0 } else { -1.0 };
                yaw_jump(sign * 0.4)
            });
            let out = tick_with(
                &cfg,
                &mut state,
                secs(f64::from(n) * 0.02),
                &pinned,
                command.as_ref(),
            );
            worst_gap = worst_gap.max((last_goal(&state).body_yaw - pinned.body_yaw).abs());
            if let Some(fault) = out.report.fault {
                raised = Some((n, fault));
                break;
            }
        }

        let (at, fault) = raised.expect("the stalled joint's window ran out");
        let Fault::HeadObstructed {
            joint,
            error,
            count,
        } = fault
        else {
            panic!("expected the head's obstruction, got {fault}");
        };
        assert_eq!(joint, JointRef::BodyYaw);
        assert_eq!(count, cfg.tracking.ticks, "the ordinary window ran out");
        assert!(
            error > cfg.tracking.threshold_rad,
            "and it was past the threshold when it did: {error}"
        );
        assert!(
            at < 30,
            "the ratchet took {at} periods to carry the trajectory clear of the joint"
        );
        assert!(
            worst_gap > 10.0 * cfg.tracking.threshold_rad,
            "the goal was reversing right across the joint: {worst_gap}"
        );
    }

    /// The nine runs are nine runs. A joint closing on its goal, and a joint
    /// inside the threshold, neither hold up nor reset the run of the joint
    /// that is stuck — and the fault names the joint whose window ran out, not
    /// the one furthest from its goal.
    #[test]
    fn the_window_that_ran_out_names_its_own_joint() {
        let cfg = tracking_cfg(0.1, 0.01, 5);
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        settle(&cfg, &mut state, &pinned);

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
                JointRef::AntennaRight,
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
                joint: JointRef::BodyYaw,
                error: 0.15,
                count: cfg.tracking.ticks,
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
            warp: WarpKind::MinJerk,
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
        let spliced = last_targets(&state);
        let travelled = spliced.head_pose_body.translation.z;
        assert!(
            travelled > 0.001 && travelled < 0.18,
            "mid-flight at {travelled}"
        );

        let other = MotionCommand::MoveTo {
            target: pose_at(0.15),
            durations: MoveDurations::uniform(secs(2.0)),
            warp: WarpKind::MinJerk,
        };
        let out = tick_with(&cfg, &mut state, secs(0.6), &present, Some(&other));
        assert_eq!(out.report.command, CommandDisposition::Retargeted);
        assert_eq!(out.report.fault, None, "and replacing it did not fault");
        assert_eq!(state.mode, MotionMode::Moving);
        assert_eq!(
            state.moving_elapsed,
            SlotDuration::from_nanos(0),
            "the clock restarts with the new path"
        );
        // Zero elapsed time on the new path: it starts where the old one had
        // got to, so this tick asks for nothing and emits nothing.
        assert!(out.report.start_sample);
        assert!(out.goal.is_none());
        assert_eq!(
            last_targets(&state).head_pose_body.translation.z,
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
        assert_eq!(state.mode, MotionMode::Holding);
        assert!(
            (last_targets(&state).head_pose_body.translation.z - 0.15).abs() < 1e-12,
            "ended at {}",
            last_targets(&state).head_pose_body.translation.z
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
            warp: WarpKind::MinJerk,
        };
        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&instant));
        assert_eq!(
            out.report.command,
            CommandDisposition::Rejected(CommandRejection::Trajectory(
                TrajectoryError::NonPositiveDuration
            ))
        );
        assert_eq!(out.report.fault, None);
        assert_eq!(state.mode, MotionMode::Holding);
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
            warp: WarpKind::MinJerk,
        };

        let mut present = pinned;
        tick_with(&cfg, &mut state, secs(0.0), &present, Some(&command));
        for n in 1..30 {
            let out = tick_with(&cfg, &mut state, secs(f64::from(n) * 0.02), &present, None);
            if let Some(goal) = out.goal {
                present = goal;
            }
        }
        let mid = last_goal(&state);
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
        assert_eq!(state.mode, MotionMode::Holding);
        assert_eq!(last_goal(&state), mid);

        let out = tick_with(&cfg, &mut state, secs(0.62), &present, None);
        assert_eq!(out.goal, None);
        assert_eq!(last_goal(&state), mid);
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
            warp: WarpKind::Linear,
        };

        let mut present = pinned;
        let mut abort = None;
        let mut commanded = pinned;
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
                commanded = goal;
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
        assert_eq!(
            group_of(joint),
            Some(JointGroup::Legs),
            "{} stepped",
            Name(joint)
        );
        assert!(delta > 1e-4, "delta {delta}");
        assert_eq!(state.mode, MotionMode::Holding, "holding, not faulted");
        assert_eq!(
            last_goal(&state),
            commanded,
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
                armed: true,
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
            warp: WarpKind::MinJerk,
        };

        // The machine does not follow, so a run opens on the yaw and grows
        // while the min-jerk ramp builds toward the step the guard stops.
        let mut open = 0;
        let mut aborted = None;
        let mut at = 0;
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
                at = n;
                break;
            }
        }
        assert!(
            matches!(
                aborted,
                Some(MoveAbort::StepTooLarge {
                    joint: JointRef::BodyYaw,
                    ..
                })
            ),
            "{aborted:?}"
        );
        assert!(open >= 2, "the run to be cleared was {open} period(s) old");

        // The next live read is the recovery's first measurement, and it starts
        // its own run rather than continuing the abandoned move's. The very
        // next period, so the prediction is stepped rather than re-seeded.
        let out = tick_with(
            &cfg,
            &mut state,
            secs(f64::from(at + 1) * 0.02),
            &pinned,
            None,
        );
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
                assert!((state.mode == MotionMode::Faulted));
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
                    commanded: None,
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
        let held = &last_targets(&state);
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
            format!("{:.9}", state.present_min_margin),
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
            state.present_min_margin < cfg.env.min_toggle_margin,
            "the armed rest is tighter than the floor: {}",
            state.present_min_margin
        );

        // One millimetre up: still far below the floor, admitted because it
        // improves on the measured clearance.
        let lift = rest_targets(0.001);
        let command = MotionCommand::MoveTo {
            target: lift,
            durations: MoveDurations::uniform(secs(2.0)),
            warp: WarpKind::MinJerk,
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
            warp: WarpKind::MinJerk,
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
            warp: WarpKind::MinJerk,
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
        assert_eq!(state.mode, MotionMode::Holding);
        assert!(emitted > 90, "only {emitted} goals over a 2 s lift");

        // It arrived, and it arrived somewhere with real clearance.
        assert_eq!(
            last_targets(&state).head_pose_body,
            neutral.head_pose_body,
            "the endpoint is the target's own bits"
        );
        assert!(
            state.present_min_margin > cfg.env.min_toggle_margin,
            "clearance at the top of the lift: {}",
            state.present_min_margin
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
        assert_eq!(last_targets(&state).body_yaw, 0.0, "square at the end");
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
            warp: WarpKind::MinJerk,
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
        assert!(!(state.mode == MotionMode::Faulted));

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
                warp: WarpKind::MinJerk,
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
        // any single bound is not, and a reading nobody can place never is. The
        // allowance is read off the state the move began in, so each comparison
        // goes through the fields that hold it.
        let allowance = |excursion: &Excursion| {
            let mut state = Armed::of(MotionSnapWire::new());
            excursion.store(&mut state);
            state
        };
        assert!(above.no_further_out_than(&allowance(&above)));
        assert!(inside.no_further_out_than(&allowance(&above)));
        assert!(!above.no_further_out_than(&allowance(&inside)));
        assert!(!below.no_further_out_than(&allowance(&above)));
        let unplaceable = Excursion::of(&env, &report, f64::NAN);
        assert_eq!(unplaceable.body_yaw, f64::INFINITY);
        assert!(!unplaceable.no_further_out_than(&allowance(&below)));
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
        let held = last_targets(&state);
        let command = MotionCommand::MoveTo {
            target: JointTargets::default(),
            durations: MoveDurations::uniform(secs(2.0)),
            warp: WarpKind::MinJerk,
        };

        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&command));
        assert_eq!(out.report.command, CommandDisposition::Started);
        assert!(out.report.start_sample);
        assert_eq!(out.report.envelope, None, "nothing was checked");
        assert_eq!(out.goal, None, "and nothing went out");
        assert!(!out.report.emitted);
        assert_eq!(out.report.fault, None);
        assert!(out.report.mode == MotionMode::Moving);
        assert_eq!(last_goal(&state), pinned, "the goals are untouched");
        assert_eq!(last_targets(&state), held, "and so is their mirror");

        // The next tick asks for a pose the machine is not in, and that one is
        // checked and emitted.
        let out = tick_with(&cfg, &mut state, secs(0.02), &pinned, None);
        assert!(!out.report.start_sample);
        assert!(out.report.envelope.is_some(), "this one was checked");
        assert!(out.report.emitted);
        assert_eq!(out.report.fault, None);
        assert_ne!(last_goal(&state), pinned);
    }

    /// A reading that is not a number is a read that did not arrive: it never
    /// reaches the pose solve, it never reaches the tracking comparison as an
    /// error nobody can place, and it costs the machine one period, not its
    /// torque. Whichever joint carries it, and whichever way it is unplaceable.
    #[test]
    fn a_non_finite_present_read_is_a_missed_read() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();

        for index in 0..ROW_COUNT {
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
                assert_eq!(
                    out.report.tracking_residuals, None,
                    "slot {index} with {bad}"
                );
                assert_eq!(out.goal, None);
                assert_eq!(out.report.fk, None, "the solve never ran");
                assert!(!(state.mode == MotionMode::Faulted));
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

    /// The tracking fault names the joint whose residual could not be placed,
    /// with that residual, rather than the joint after it with zero.
    ///
    /// Neither a reading nor a setpoint can put such a number into the
    /// comparison — both are screened at the boundary, and a corrupt frame is
    /// counted rather than modelled — so the route in is a modelled trajectory
    /// that is not a number, which is exactly what `resume` refuses a state
    /// for. This pins what the tick makes of one that reached it anyway: the
    /// fault names the row, and the residual travels as it was measured.
    #[test]
    fn the_tracking_fault_names_the_joint_nobody_could_place() {
        let cfg = MotionConfig {
            tracking: TrackingFaultConfig {
                armed: true,
                threshold_rad: 0.1,
                progress_min_rad: 0.01,
                ticks: 1,
                ..MotionConfig::default().tracking
            },
            ..MotionConfig::default()
        };
        let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());
        settle(&cfg, &mut state, &pinned);
        state.tracking[row(JointRef::Leg3).expect("a bus row")].predicted = f64::NAN;

        let raised = tick_with(&cfg, &mut state, secs(0.02), &pinned, None)
            .report
            .fault;
        let Some(Fault::HeadObstructed { joint, error, .. }) = raised else {
            panic!("expected a tracking fault, got {raised:?}");
        };
        assert_eq!(joint, JointRef::Leg3);
        assert!(
            error.is_nan(),
            "the residual travels as it was measured: {error}"
        );
        assert_eq!(
            raised.unwrap().to_string(),
            "leg 4 is NaN rad off its predicted position and not closing"
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
            warp: WarpKind::MinJerk,
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
        assert_eq!(state.mode, MotionMode::Holding);
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
            warp: WarpKind::MinJerk,
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
        let arming_seed = fk_seed(&state);
        let command = MotionCommand::MoveTo {
            target: pose_at(0.195),
            durations: MoveDurations::uniform(secs(1.0)),
            warp: WarpKind::MinJerk,
        };

        let mut present = pinned;
        for n in 0..60 {
            let seed_before = fk_seed(&state);
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
            assert_eq!(fk_seed(&state), expected, "tick {n}");

            if let Some(goal) = out.goal {
                present = goal;
            }
            if out.report.completed {
                break;
            }
        }
        let travelled =
            (fk_seed(&state).translation.vector - arming_seed.translation.vector).norm();
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
                JointRef::BodyYaw => "body yaw",
                JointRef::AntennaRight | JointRef::AntennaLeft => "antennas",
                _ => "legs",
            };
            assert_eq!(named, group, "{group}: the abort named {}", Name(joint));
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
            warp: WarpKind::MinJerk,
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
        let before_outage = last_goal(&state);

        for n in 10..14 {
            let mut out = TickOutputs::default();
            motion_tick(
                &cfg,
                &mut state,
                &TickInputs {
                    now: secs(f64::from(n) * 0.02),
                    period: PERIOD,
                    present: None,
                    commanded: None,
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
            last_goal(&state),
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
                commanded: None,
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
            warp: WarpKind::MinJerk,
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
        let advanced = last_goal(&state);

        // One period's worth of elapsed time, long after twenty periods have
        // passed. The sample lands where the move already stood, so the goal is
        // the one the servos are holding and nothing goes out.
        let out = tick_with(&cfg, &mut state, PERIOD, &present, None);
        assert_eq!(out.report.fault, None);
        assert!(!out.report.emitted, "a goal went out for the earlier time");
        assert_eq!(out.goal, None);
        assert_eq!(last_goal(&state), advanced, "the machine stood still");
        assert!(state.mode == MotionMode::Moving, "still moving");

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
            last_goal(&state),
            last_goal(&undisturbed),
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

        let faults: [(Fault, &str); 9] = [
            (
                Fault::PositionFeedbackLost { misses: 50 },
                "no position read for 50 consecutive ticks",
            ),
            (
                Fault::HeadObstructed {
                    joint: JointRef::BodyYaw,
                    error: 0.123_456,
                    count: 10,
                },
                "body yaw is 0.1235 rad off its predicted position and not closing",
            ),
            (
                Fault::AntennaObstructed {
                    joint: JointRef::AntennaRight,
                    error: 0.5,
                    count: 30,
                },
                "right antenna is 0.5000 rad off its predicted position and not closing",
            ),
            (
                Fault::HeadServoFault {
                    joint: JointRef::Leg2,
                    id: 13,
                    bits: 0x20,
                },
                "leg 3 (servo 13) reports hardware error bits 0x20",
            ),
            (
                Fault::AntennaServoFault {
                    joint: JointRef::AntennaLeft,
                    id: 18,
                    bits: 0x04,
                },
                "left antenna (servo 18) reports hardware error bits 0x04",
            ),
            (
                Fault::BusFailure {
                    source: BusFailureSource::Sequence(SeqError::NoAnswer {
                        context: StepContext::reg(
                            SeqStepKind::PinAndEnable,
                            13,
                            RegId::GoalPosition,
                        ),
                    }),
                },
                "the bus is not carrying commands: pin and enable of servo 13, goal position: no \
                 answer",
            ),
            (
                Fault::BusFailure {
                    source: BusFailureSource::Transaction {
                        id: 13,
                        kind: WireFailure::Corrupt,
                    },
                },
                "the bus is not carrying commands: servo 13: a corrupt reply",
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
                    joint: JointRef::Leg3,
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
                    joint: JointRef::AntennaLeft,
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
    /// torque-on and the target is −174.752°, which is 22.9° away the near way
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
        let landed = last_targets(&state).antennas[0];
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
        state: &mut MotionSnap,
        start: &JointVector,
        target: &JointTargets,
        side: usize,
    ) -> Vec<f64> {
        let command = MotionCommand::MoveTo {
            target: *target,
            durations: MoveDurations::uniform(secs(2.0)),
            warp: WarpKind::MinJerk,
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
    /// widest envelope past whatever is standing beside the machine.
    ///
    /// Two legs per side, and they prove different halves. The stow fold leans
    /// inboard of straight down, so its *short* arc to upright already misses
    /// the outboard point and the policy never fires: that leg is the sweep the
    /// machine actually makes, asserted to miss the point, land, and be a whole
    /// turn less the difference. The synthetic fold short of half a turn is the
    /// leg whose short arc does cross, so it is the one the long-way branch
    /// carries; the same three assertions hold over it.
    #[test]
    fn an_antenna_sweep_misses_its_outboard_point() {
        let cfg = MotionConfig::default();
        for (side, outboard) in ANTENNA_OUTBOARD.into_iter().enumerate() {
            let stow = crate::disarm::STOW_ANTENNAS[side];
            // Inboard of the outboard point, so its short arc crosses it.
            let crossing = if side == 0 { -2.5 } else { 2.5 };
            for (from, to) in [(stow, 0.0), (0.0, stow), (crossing, 0.0), (0.0, crossing)] {
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
                let fold = if from == 0.0 { to } else { from };
                assert!(
                    (swept - (core::f64::consts::TAU - fold.abs())).abs() < 1e-9,
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
            let stow = crate::disarm::STOW_ANTENNAS[side];
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
            let landed = last_targets(&state).antennas[side];
            let swept = (landed - outboard).abs();
            assert!(
                (swept - (stow.abs() - core::f64::consts::FRAC_PI_2)).abs() < 1e-9,
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
            let stow = crate::disarm::STOW_ANTENNAS[side];
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
            let landed = last_targets(&state).antennas[side];
            let swept = (landed - stow).abs();
            assert!(
                (swept - (stow.abs() - core::f64::consts::FRAC_PI_2)).abs() < 1e-9,
                "antenna {side} swept {swept} rad to sideways"
            );
        }
    }

    /// The outboard constants are the physical sideways direction — a quarter
    /// turn either side of straight up — and not the halfway point between
    /// straight up and the fold, which is 5.1° off it. A machine whose stow
    /// fold moved would keep the same two constants.
    #[test]
    fn the_outboard_directions_are_horizontal() {
        assert_eq!(
            ANTENNA_OUTBOARD,
            [-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2]
        );
        let stow_midpoint = crate::disarm::STOW_ANTENNAS[1] / 2.0;
        assert!(
            (stow_midpoint - core::f64::consts::FRAC_PI_2).abs() > 0.04,
            "halfway to the fold is {stow_midpoint}, which is not horizontal"
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
            let landed = last_targets(&state).antennas[side];
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
        for directions in [
            crate::disarm::STOW_ANTENNAS,
            [0.0, 0.0],
            crate::disarm::STOW_ANTENNAS,
        ] {
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

            let landed = last_targets(&state).antennas;
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
                    warp: WarpKind::MinJerk,
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
                assert_eq!(joint, JointRef::AntennaRight);
                assert!(!angle.is_finite(), "{direction} resolved to {angle}");
                // Refused, so nothing moved and nothing went out.
                assert_eq!(out.report.mode, MotionMode::Holding);
                assert!(out.goal.is_none());
                assert_eq!(last_targets(&state).antennas, start.antennas);
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
            let goal = last_targets(&state).antennas[0];
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
                    warp: WarpKind::MinJerk,
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
        // The same bytes, in a second slot: what a host that picked this state
        // up somewhere else would be ticking.
        let mut second = Armed::of(
            clockwork_rs::blob_from_bytes(clockwork_rs::blob_as_bytes(first.bytes()))
                .expect("a state is its own size"),
        );
        let command = MotionCommand::MoveTo {
            target: pose_at(0.19),
            durations: MoveDurations::uniform(secs(2.0)),
            warp: WarpKind::MinJerk,
        };

        for n in 0..20 {
            let now = secs(f64::from(n) * 0.02);
            let command = (n == 0).then_some(&command);
            let a = tick_with(&cfg, &mut first, now, &pinned, command);
            let b = tick_with(&cfg, &mut second, now, &pinned, command);
            assert_eq!(a, b, "tick {n}");
        }
    }

    /// The neutral pose raised by `dz` metres: a setpoint a period away from
    /// where an armed machine stands.
    fn lifted(dz: f64) -> JointTargets {
        let mut targets = JointTargets::default();
        targets.head_pose_body.translation.z += dz;
        targets
    }

    /// A tracked setpoint is decided and commanded within the period it
    /// arrives on: no trajectory, no mode to leave, and the goals out on the
    /// same tick.
    #[test]
    fn a_tracked_setpoint_is_commanded_on_its_own_tick() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let setpoint = lifted(0.002);
        let out = tick_with(
            &cfg,
            &mut state,
            secs(0.0),
            &pinned,
            Some(&MotionCommand::Track(setpoint)),
        );

        assert_eq!(out.report.command, CommandDisposition::Tracked);
        assert_eq!(
            out.report.mode,
            MotionMode::Holding,
            "nothing is left running"
        );
        assert!(out.report.emitted, "the setpoint went out");
        let goal = out.goal.expect("a setpoint that moves a joint emits");
        assert_ne!(goal.legs, pinned.legs, "the legs hold the new pose");
        assert_eq!(&last_goal(&state), &goal);
        assert_eq!(
            &last_targets(&state),
            &setpoint,
            "the next move chains from what was tracked, not from a solve of it"
        );
        assert_eq!(out.report.fault, None);
        assert!(out.report.envelope.is_some(), "the pose was checked");
    }

    /// A setpoint identical to the goal the servos already hold is accepted and
    /// writes nothing — the same rule a sampled path pose is emitted under.
    #[test]
    fn a_tracked_setpoint_that_moves_nothing_writes_nothing() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let held = MotionCommand::Track(last_targets(&state));
        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&held));

        assert_eq!(out.report.command, CommandDisposition::Tracked);
        assert!(!out.report.emitted);
        assert!(out.goal.is_none());
        assert_eq!(&last_goal(&state), &pinned);
    }

    /// A setpoint outside the envelope is refused, and a refusal is nothing
    /// happening: no goal, no fault, no mask, and the machine holding exactly
    /// where it was.
    #[test]
    fn a_tracked_setpoint_outside_the_envelope_is_refused() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let past_the_cap = JointTargets {
            body_yaw: cfg.env.body_yaw_limit + 0.1,
            ..JointTargets::default()
        };
        let out = tick_with(
            &cfg,
            &mut state,
            secs(0.0),
            &pinned,
            Some(&MotionCommand::Track(past_the_cap)),
        );

        let CommandDisposition::Rejected(CommandRejection::Envelope(violations)) =
            out.report.command
        else {
            panic!("expected an envelope refusal, got {:?}", out.report.command);
        };
        assert!(violations.body_yaw);
        assert!(out.goal.is_none());
        assert!(!out.report.emitted);
        assert_eq!(out.report.fault, None, "a bad plan is not a fault");
        assert!(flags::is_empty(out.report.masked), "nothing was taken out");
        assert_eq!(out.report.mode, MotionMode::Holding);
        assert_eq!(&last_goal(&state), &pinned, "the machine did not move");
    }

    /// The recovery allowance is a sampled move's alone.
    ///
    /// The pair is the assertion, and neither half proves anything by itself: a
    /// machine standing outside the envelope has a move's samples admitted no
    /// further out than where it started — that is how it travels back inside —
    /// and the *same* pose handed in as a setpoint is refused. Every other
    /// tracked test arms at neutral, where the excursion is zero and the two
    /// verdicts are identical, so this is the only place the parameter means
    /// anything. Without it a composed overlay would inherit the allowance and
    /// the daemon would command out-of-envelope poses every tick, for as long
    /// as the machine started out there, with nothing refused and nothing said.
    #[test]
    fn a_tracked_setpoint_gets_no_share_of_a_moves_recovery_allowance() {
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
            warp: WarpKind::MinJerk,
        };
        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&command));
        assert_eq!(out.report.command, CommandDisposition::Started);
        let out = tick_with(&cfg, &mut state, secs(0.02), &pinned, None);
        assert!(
            out.report.recovering,
            "the move's samples are admitted under the allowance"
        );
        assert!(
            out.report
                .envelope
                .expect("a checked sample")
                .violations
                .body_yaw,
            "and they are still outside the cap"
        );

        // The pose that sample just commanded, handed straight back in as a
        // setpoint: identical to something the move was allowed to command, and
        // no further out than where the move began, so the allowance is the
        // only thing that could admit it.
        let admitted_as_a_sample = last_targets(&state);
        let out = tick_with(
            &cfg,
            &mut state,
            secs(0.04),
            &pinned,
            Some(&MotionCommand::Track(admitted_as_a_sample)),
        );
        let CommandDisposition::Rejected(CommandRejection::Envelope(violations)) =
            out.report.command
        else {
            panic!(
                "a setpoint is judged on the bounds alone, got {:?}",
                out.report.command
            );
        };
        assert!(violations.body_yaw);
        assert_eq!(out.report.fault, None, "a bad plan is not a fault");
        // The refusal changed nothing: the move it landed on is still the thing
        // driving the machine, which is why its samples keep the allowance and
        // the setpoint does not.
        assert!(out.report.mode == MotionMode::Moving);
    }

    /// A setpoint further from the last goal than one tick's bound allows is
    /// emitted. The step bound bounds a plan of this crate's own, and a tracked
    /// setpoint is the caller's content, sent as asked.
    #[test]
    fn a_tracked_setpoint_past_the_step_bound_is_emitted() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        // A pose the envelope has no objection to — it is a target moves are
        // commanded to elsewhere here — reached in one period.
        let target = pose_at(0.19);
        let out = tick_with(
            &cfg,
            &mut state,
            secs(0.0),
            &pinned,
            Some(&MotionCommand::Track(target)),
        );

        assert_eq!(out.report.command, CommandDisposition::Tracked);
        let goal = out.goal.expect("the setpoint went out");
        // Some leg moved further in one period than the guard would have
        // allowed on a sampled path, or this proves nothing.
        assert!(
            (0..6).any(|leg| (goal.legs[leg] - pinned.legs[leg]).abs() > cfg.max_step.legs),
            "no leg stepped past the bound: {goal:?}"
        );
        assert_eq!(out.report.aborted, None, "there was no move to abandon");
        assert_eq!(out.report.fault, None);
        assert_eq!(&last_goal(&state), &goal);

        // And the machine still takes the next setpoint from where this one put
        // it.
        let out = tick_with(
            &cfg,
            &mut state,
            secs(0.02),
            &pinned,
            Some(&MotionCommand::Track(lifted(0.002))),
        );
        assert_eq!(out.report.command, CommandDisposition::Tracked);
    }

    /// An antenna angle no goal register can hold is refused, on the same
    /// reachability rule a commanded move is refused by.
    #[test]
    fn a_tracked_antenna_angle_with_no_count_is_refused() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let out = tick_with(
            &cfg,
            &mut state,
            secs(0.0),
            &pinned,
            Some(&MotionCommand::Track(antennas_at([
                ANTENNA_GOAL_MAX_RAD + 1.0,
                0.0,
            ]))),
        );

        let CommandDisposition::Rejected(CommandRejection::AntennaUnreachable { joint, angle }) =
            out.report.command
        else {
            panic!(
                "expected a reachability refusal, got {:?}",
                out.report.command
            );
        };
        assert_eq!(joint, JointRef::AntennaRight);
        assert!(angle > ANTENNA_GOAL_MAX_RAD);
        assert!(out.goal.is_none());
    }

    /// A degraded antenna is never commanded and never bounded: the pair is out
    /// of service, so what a setpoint says about it neither reaches the wire nor
    /// refuses the setpoint the rest of the machine is asked for.
    #[test]
    fn a_tracked_setpoint_honours_the_antenna_mask() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let mut health = healthy_servos();
        health[8].bits = 0x04;
        let out = tick_with_health(&cfg, &mut state, secs(0.0), &pinned, &health, None);
        assert!(flags::covers(out.report.newly_masked, JointGroup::Antennas));

        // An antenna step no bound would pass, beside a head step that fits.
        let wild = JointTargets {
            antennas: [pinned.antennas[0] + 3.0, pinned.antennas[1]],
            ..lifted(0.002)
        };
        let out = tick_with(
            &cfg,
            &mut state,
            secs(0.02),
            &pinned,
            Some(&MotionCommand::Track(wild)),
        );

        assert_eq!(
            out.report.command,
            CommandDisposition::Tracked,
            "a masked joint has no step to bound"
        );
        let goal = out.goal.expect("the head still moves");
        assert!(
            (goal.antennas[0] - wild.antennas[0]).abs() < 1e-12,
            "the plan is left whole; the mask decides what reaches the wire"
        );
        assert!(flags::covers(out.report.masked, JointGroup::Antennas));

        // Past the goal register's range, not merely past a step: the same skip
        // has to hold there, or a degraded antenna would refuse every composed
        // target a clip produces — and a refused composition drops every
        // overlay on the machine — over a count nothing writes.
        let unreachable = JointTargets {
            antennas: [ANTENNA_GOAL_MAX_RAD + 1.0, pinned.antennas[1]],
            ..lifted(0.004)
        };
        let out = tick_with(
            &cfg,
            &mut state,
            secs(0.04),
            &pinned,
            Some(&MotionCommand::Track(unreachable)),
        );
        assert_eq!(
            out.report.command,
            CommandDisposition::Tracked,
            "a masked antenna has no register to reach"
        );
        assert!(out.goal.is_some(), "the head still moves");
    }

    /// A setpoint ends a move in flight. Two things cannot be commanding the
    /// same servos, and the caller handing over setpoints is the one that is.
    #[test]
    fn a_tracked_setpoint_ends_the_move_it_lands_on() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (mut state, pinned) = armed_at(&cfg, &start);

        let command = move_to(pose_at(0.19), secs(2.0));
        let out = tick_with(&cfg, &mut state, secs(0.0), &pinned, Some(&command));
        assert_eq!(out.report.command, CommandDisposition::Started);
        let out = tick_with(&cfg, &mut state, secs(0.02), &pinned, None);
        assert!(out.report.mode == MotionMode::Moving);

        let held = last_targets(&state);
        let out = tick_with(
            &cfg,
            &mut state,
            secs(0.04),
            &pinned,
            Some(&MotionCommand::Track(held)),
        );
        assert_eq!(out.report.command, CommandDisposition::Tracked);
        assert_eq!(out.report.mode, MotionMode::Holding);

        // And the move really is gone: the ticks after it emit nothing.
        let out = tick_with(&cfg, &mut state, secs(0.06), &pinned, None);
        assert!(out.goal.is_none());
        assert_eq!(out.report.mode, MotionMode::Holding);
    }

    /// A planned move driven setpoint by setpoint is the same motion as the
    /// same move commanded: the plan resolves the antenna directions, floors
    /// the clock, and shapes the path exactly as the tick would have, so the
    /// goals that go on the wire are goal-for-goal identical.
    ///
    /// The antenna target is deliberately one whose short arc crosses its
    /// outboard direction, which is the whole reason a caller cannot assemble
    /// this trajectory itself: the resolution that routes it the long way round
    /// is private to the tick.
    #[test]
    fn a_planned_move_driven_as_setpoints_is_the_move_it_planned() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let target = JointTargets {
            body_yaw: 0.2,
            antennas: [-1.7, 0.3],
            ..lifted(0.005)
        };
        // Short enough that the pass has to lengthen it, so the equivalence
        // covers the flooring too.
        let asked = MoveDurations::uniform(secs(0.2));

        let (mut commanded, pinned) = armed_at(&cfg, &start);
        let (floored, stretch) = floor_move_clock(
            &cfg,
            &last_targets(&commanded),
            &MotionCommand::MoveTo {
                target,
                durations: asked,
                warp: WarpKind::MinJerk,
            },
            FLOOR_TICK_HZ,
        );
        assert!(stretch.is_some(), "the asked-for clock is under its floor");

        let mut wired = Vec::new();
        let mut present = pinned;
        for n in 0..2000 {
            let command = (n == 0).then_some(&floored);
            let out = tick_with(
                &cfg,
                &mut commanded,
                secs(f64::from(n) * 0.02),
                &present,
                command,
            );
            assert_eq!(out.report.fault, None, "tick {n}");
            assert_eq!(out.report.aborted, None, "tick {n}");
            if let Some(goal) = out.goal {
                wired.push(goal);
                present = goal;
            }
            if out.report.completed {
                break;
            }
        }
        assert!(!wired.is_empty(), "the commanded move went out");

        let (mut driven, pinned) = armed_at(&cfg, &start);
        let (trajectory, planned_stretch) = plan_move(
            &cfg,
            &last_targets(&driven),
            &target,
            asked,
            WarpKind::MinJerk,
            FLOOR_TICK_HZ,
            Some(driven.present_min_margin),
        )
        .expect("the move is one this machine runs");
        assert_eq!(
            planned_stretch, stretch,
            "the same clock, reported the same"
        );
        assert!(
            trajectory.target().antennas[0] > 0.0,
            "the right antenna goes the long way round, not through its outboard \
             direction: {}",
            trajectory.target().antennas[0]
        );

        let mut streamed = Vec::new();
        let mut present = pinned;
        for n in 1..2000 {
            let at = secs(f64::from(n) * 0.02);
            let mut setpoint = JointTargets::default();
            trajectory.sample(at, &mut setpoint);
            let out = tick_with(
                &cfg,
                &mut driven,
                at,
                &present,
                Some(&MotionCommand::Track(setpoint)),
            );
            assert_eq!(
                out.report.command,
                CommandDisposition::Tracked,
                "tick {n}: a planned path is one the tick takes"
            );
            if let Some(goal) = out.goal {
                streamed.push(goal);
                present = goal;
            }
            if trajectory.done(at) {
                break;
            }
        }

        assert_eq!(streamed, wired, "the same goals, in the same order");
    }

    /// A plan is refused for the reasons a command is, and a refused plan is a
    /// plan: nothing was touched, so there is nothing to wind down.
    #[test]
    fn a_plan_is_refused_for_what_a_command_is_refused_for() {
        let cfg = MotionConfig::default();
        let start = JointTargets::default();
        let (state, _) = armed_at(&cfg, &start);
        let baseline = Some(state.present_min_margin);

        let past_the_cap = JointTargets {
            body_yaw: cfg.env.body_yaw_limit + 0.1,
            ..JointTargets::default()
        };
        let refused = plan_move(
            &cfg,
            &last_targets(&state),
            &past_the_cap,
            MoveDurations::uniform(secs(2.0)),
            WarpKind::MinJerk,
            FLOOR_TICK_HZ,
            baseline,
        );
        let Err(CommandRejection::Envelope(violations)) = refused else {
            panic!("expected an envelope refusal, got {refused:?}");
        };
        assert!(violations.body_yaw);

        let unreachable = plan_move(
            &cfg,
            &last_targets(&state),
            &antennas_at([f64::NAN, 0.0]),
            MoveDurations::uniform(secs(2.0)),
            WarpKind::MinJerk,
            FLOOR_TICK_HZ,
            baseline,
        );
        assert!(matches!(
            unreachable,
            Err(CommandRejection::AntennaUnreachable {
                joint: JointRef::AntennaRight,
                ..
            })
        ));
    }

    /// The snapshot round-trip law: for any state the tick can reach and any
    /// inputs, a state restored from a snapshot of it ticks identically.
    ///
    /// Asserted at *every* period of each replayed sequence rather than at a
    /// chosen one, because the states worth doubting are the ones a sequence
    /// passes through: a move part way along its clock, a tracking run part way
    /// through its window, a mask half filled. Each sequence also says what it
    /// exercised, so a scenario that stops reaching the state it was written
    /// for fails rather than passing vacuously.
    mod snapshot {
        use super::*;
        use clockwork_rs::{blob_as_bytes, blob_from_bytes};

        /// One way of damaging a state's bytes, for a case that runs several.
        type Damage = fn(&mut MotionSnap);

        /// One way of pushing an excursion a hair further out than it was.
        type Nudge = fn(&mut Excursion);

        /// What a scripted machine offers the tick on one period.
        #[derive(Default)]
        struct Offered {
            present: Option<JointVector>,
            health: Option<[ServoHealth; ROW_COUNT]>,
            command: Option<MotionCommand>,
        }

        impl Offered {
            /// A live read and nothing else.
            fn read(present: JointVector) -> Self {
                Self {
                    present: Some(present),
                    ..Self::default()
                }
            }

            /// A live read, with a command on this period.
            fn commanded(present: JointVector, command: MotionCommand) -> Self {
                Self {
                    present: Some(present),
                    command: Some(command),
                    ..Self::default()
                }
            }

            /// A live read, with a health sweep on this period.
            fn swept(present: JointVector, health: [ServoHealth; ROW_COUNT]) -> Self {
                Self {
                    present: Some(present),
                    health: Some(health),
                    ..Self::default()
                }
            }
        }

        /// What a replayed sequence turned out to reach.
        ///
        /// The guard against a vacuous law: every scenario below asserts on
        /// this, so a fixture that quietly stops faulting, aborting or moving
        /// fails instead of confirming the law over a state machine standing
        /// still.
        #[derive(Debug)]
        struct Seen {
            moved: bool,
            completed: bool,
            faults: Vec<Fault>,
            degraded: Vec<Fault>,
            aborted: Vec<MoveAbort>,
            rejected: u32,
            longest_run: u32,
            most_misses: u32,
            most_pose_failures: u32,
            /// The furthest any of the ten excursion distances stood outside
            /// its bound, over the whole run. Zero says every scenario started
            /// a move from inside the envelope, which is a mirror crossing the
            /// round trip as ten zeroes.
            furthest_out: f64,
            masked: JointFlags,
        }

        impl Default for Seen {
            /// Nothing seen yet, and no servo masked.
            ///
            /// Written out rather than derived: the vocabulary's set of servos
            /// has no `Default`, because the empty set is a value it names
            /// (`NONE`) rather than a state of not having been filled in.
            fn default() -> Self {
                Self {
                    moved: false,
                    completed: false,
                    faults: Vec::new(),
                    degraded: Vec::new(),
                    aborted: Vec::new(),
                    rejected: 0,
                    longest_run: 0,
                    most_misses: 0,
                    most_pose_failures: 0,
                    furthest_out: 0.0,
                    masked: JointFlags::NONE,
                }
            }
        }

        impl Seen {
            fn record(&mut self, report: &TickReport) {
                self.moved |= report.mode == MotionMode::Moving;
                self.completed |= report.completed;
                if let Some(fault) = report.fault {
                    self.faults.push(fault);
                }
                if let Some(fault) = report.degraded {
                    self.degraded.push(fault);
                }
                if let Some(abort) = report.aborted {
                    self.aborted.push(abort);
                }
                if matches!(report.command, CommandDisposition::Rejected(_)) {
                    self.rejected += 1;
                }
                self.longest_run = self.longest_run.max(report.tracking_count);
                self.most_misses = self.most_misses.max(report.misses);
                self.most_pose_failures = self.most_pose_failures.max(report.pose_failures);
                self.masked = report.masked;
            }

            /// Note how far outside the envelope the state's running move
            /// began, so a fixture can prove the allowance the state carries
            /// held something other than zeroes.
            fn note_excursion(&mut self, state: &MotionSnap) {
                for out in state
                    .excursion_cranks
                    .iter()
                    .chain([
                        &state.excursion_body_yaw,
                        &state.excursion_relative_yaw,
                        &state.excursion_cone,
                    ])
                    .filter(|out| out.is_finite())
                {
                    self.furthest_out = self.furthest_out.max(*out);
                }
            }

            /// The distinct fault slugs raised — a re-raise of the same
            /// standing condition is one slug, not two.
            fn fault_slugs(&self) -> Vec<&'static str> {
                let mut slugs: Vec<&'static str> = self
                    .faults
                    .iter()
                    .map(|raised| fault::slug(fault::kind(raised)))
                    .collect();
                slugs.dedup();
                slugs
            }
        }

        /// Drive `state` through `periods` periods of `script`, asserting the
        /// law at each one, and report what the run reached.
        ///
        /// `script` is handed the period index and where a perfectly following
        /// machine would be standing — the previous period's goal — and answers
        /// with what its own machine offers, which is how a fixture freezes a
        /// joint or drops a read.
        fn law_holds<F>(cfg: &MotionConfig, state: &mut Armed, periods: u32, mut script: F) -> Seen
        where
            F: FnMut(u32, &JointVector) -> Offered,
        {
            let mut tracking_machine = last_goal(state);
            let mut seen = Seen::default();
            for n in 0..periods {
                let offered = script(n, &tracking_machine);
                // The driver holds the goal the last period emitted, so the
                // prediction the run is judged against crosses the round trip
                // with everything else.
                let held = last_goal(state);
                let inputs = TickInputs {
                    now: secs(f64::from(n) * PERIOD.as_secs_f64()),
                    period: PERIOD,
                    present: offered.present.as_ref(),
                    commanded: Some(&held),
                    command: offered.command.as_ref(),
                    health: offered.health.as_ref(),
                };

                // What a second host picks up is these very bytes -- the state
                // is the slot -- so the state it carries is a copy of them,
                // resumed the way a host resumes one.
                let carried: MotionSnapWire =
                    blob_from_bytes(blob_as_bytes(state.bytes())).expect("a state is its own size");
                let mut resumed = Armed::of(carried);
                resumed
                    .resumes()
                    .unwrap_or_else(|err| panic!("period {n}: the state does not resume: {err}"));

                let mut out = TickOutputs::default();
                let mut shadow = TickOutputs::default();
                motion_tick(cfg, state, &inputs, &mut out);
                motion_tick(cfg, &mut resumed, &inputs, &mut shadow);
                assert_eq!(
                    out, shadow,
                    "period {n}: a resumed state ticked differently"
                );
                assert_eq!(
                    blob_as_bytes(state.bytes()),
                    blob_as_bytes(resumed.bytes()),
                    "period {n}: a resumed state landed in a different successor"
                );

                seen.record(&out.report);
                seen.note_excursion(state);
                if let Some(goal) = out.goal {
                    tracking_machine = goal;
                }
            }
            seen
        }

        /// A machine armed and holding at the recorded stow pose.
        fn armed_at_stow(cfg: &MotionConfig) -> (Armed, JointVector) {
            armed_at(
                cfg,
                &JointTargets {
                    head_pose_body: reachy_kin::stow_head_pose(),
                    ..JointTargets::default()
                },
            )
        }

        /// A whole move, sampled every period by a machine that follows
        /// perfectly, then held past its completion.
        #[test]
        fn a_move_survives_the_round_trip_at_every_period() {
            let cfg = MotionConfig::default();
            let (mut state, _) = armed_at_stow(&cfg);
            let command = move_to(JointTargets::default(), secs(1.0));

            let seen = law_holds(&cfg, &mut state, 80, |n, tracked| {
                if n == 0 {
                    Offered::commanded(*tracked, command)
                } else {
                    Offered::read(*tracked)
                }
            });

            assert!(seen.moved, "the fixture never moved");
            assert!(seen.completed, "the move never finished: {seen:?}");
            assert!(seen.faults.is_empty(), "{seen:?}");
        }

        /// A move retargeted part way along its own clock, which rebuilds the
        /// trajectory from a start that is neither endpoint of the first one.
        #[test]
        fn a_retarget_survives_the_round_trip() {
            let cfg = MotionConfig::default();
            let (mut state, _) = armed_at_stow(&cfg);
            let up = move_to(JointTargets::default(), secs(1.0));
            let back = move_to(
                JointTargets {
                    head_pose_body: reachy_kin::stow_head_pose(),
                    ..JointTargets::default()
                },
                secs(1.0),
            );

            let seen = law_holds(&cfg, &mut state, 140, |n, tracked| match n {
                0 => Offered::commanded(*tracked, up),
                25 => Offered::commanded(*tracked, back),
                _ => Offered::read(*tracked),
            });

            assert!(seen.completed, "the retargeted move never finished");
            assert!(seen.faults.is_empty(), "{seen:?}");
        }

        /// A move abandoned part way by a hold, which clears the trajectory
        /// without ending at either of its endpoints.
        #[test]
        fn a_hold_survives_the_round_trip() {
            let cfg = MotionConfig::default();
            let (mut state, _) = armed_at_stow(&cfg);
            let up = move_to(JointTargets::default(), secs(2.0));

            let seen = law_holds(&cfg, &mut state, 40, |n, tracked| match n {
                0 => Offered::commanded(*tracked, up),
                20 => Offered::commanded(*tracked, MotionCommand::Hold),
                _ => Offered::read(*tracked),
            });

            assert!(seen.moved, "the fixture never moved");
            assert!(!seen.completed, "the hold should have abandoned the move");
            assert!(seen.faults.is_empty(), "{seen:?}");
        }

        /// Single setpoints tracked one period at a time — the command that
        /// never builds a trajectory, and whose state is nothing but the last
        /// goal and its mirror.
        #[test]
        fn tracked_setpoints_survive_the_round_trip() {
            let cfg = MotionConfig::default();
            let neutral = JointTargets::default();
            let (mut state, _) = armed_at(&cfg, &neutral);

            let seen = law_holds(&cfg, &mut state, 12, |n, tracked| {
                // Creep the right antenna a hundredth of a radian a period,
                // well inside the step bound, so each period asks the tick for
                // one setpoint and nothing shapes it.
                let mut target = neutral;
                target.antennas[0] += 0.01 * f64::from(n + 1);
                Offered::commanded(*tracked, MotionCommand::Track(target))
            });

            assert!(!seen.moved, "a tracked setpoint is not a move");
            assert_eq!(seen.rejected, 0, "{seen:?}");
            assert!(seen.faults.is_empty(), "{seen:?}");
        }

        /// A run of dropped reads long enough to raise the latching
        /// position-feedback fault, and periods past it: the tick re-reports a
        /// standing fault forever, and the restored state has to re-report the
        /// same one.
        #[test]
        fn a_latching_fault_and_its_re_reports_survive_the_round_trip() {
            let cfg = MotionConfig {
                read_loss_ticks: 4,
                ..MotionConfig::default()
            };
            let (mut state, _) = armed_at_stow(&cfg);

            let seen = law_holds(&cfg, &mut state, 12, |n, tracked| {
                if (2..8).contains(&n) {
                    Offered::default()
                } else {
                    Offered::read(*tracked)
                }
            });

            assert!(
                (state.mode == MotionMode::Faulted),
                "the fixture never latched"
            );
            assert_eq!(
                seen.fault_slugs(),
                vec!["position_feedback_lost"],
                "{seen:?}"
            );
            assert!(seen.most_misses >= 4, "{seen:?}");
        }

        /// A joint held away from its goal long enough to run its tracking
        /// window out, twice: the prediction, the run's two anchors and its
        /// count all have to cross the round trip, and the fault is
        /// non-latching so the machine carries on and rebuilds one.
        #[test]
        fn a_tracking_run_and_its_fault_survive_the_round_trip() {
            let cfg = tracking_cfg(0.1, 0.01, 3);
            let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());

            let mut stuck = pinned;
            stuck.legs[3] += 0.2;
            stuck.antennas[1] = 0.4;

            // Long enough for the prediction to be seeded, stepped out to the
            // threshold and run its window out twice over.
            let seen = law_holds(&cfg, &mut state, 40, |_, _| Offered::read(stuck));

            assert!(seen.longest_run >= 3, "{seen:?}");
            assert_eq!(seen.fault_slugs(), vec!["head_obstructed"], "{seen:?}");
            assert!(
                seen.faults.len() >= 2,
                "a standing obstruction should rebuild its run and raise again: {seen:?}"
            );
            assert!(
                !(state.mode == MotionMode::Faulted),
                "the head's obstruction does not latch"
            );
        }

        /// An antenna's latched error bits, which take it out of service: the
        /// mask and the forgotten run both have to cross the round trip.
        #[test]
        fn a_degrade_survives_the_round_trip() {
            let cfg = MotionConfig::default();
            let (mut state, _) = armed_at_stow(&cfg);
            let mut flagged = healthy_servos();
            flagged[7].bits = 0x20;

            let seen = law_holds(&cfg, &mut state, 8, |n, tracked| {
                if n >= 3 {
                    Offered::swept(*tracked, flagged)
                } else {
                    Offered::swept(*tracked, healthy_servos())
                }
            });

            assert!(
                seen.degraded
                    .iter()
                    .any(|fault| matches!(fault, Fault::AntennaServoFault { .. })),
                "{seen:?}"
            );
            assert!(
                flags::contains(seen.masked, JointRef::AntennaRight),
                "the flagged antenna was not taken out of service: {seen:?}"
            );
        }

        /// A move whose sampled path steps further in a period than the bound
        /// allows, which aborts it and drops the trajectory mid-flight.
        #[test]
        fn an_abort_survives_the_round_trip() {
            let cfg = MotionConfig {
                max_step: JointStep {
                    legs: 0.001,
                    body_yaw: 0.15,
                    antennas: 0.65,
                },
                ..MotionConfig::default()
            };
            let (mut state, _) = armed_at_stow(&cfg);
            let command = move_to(JointTargets::default(), secs(1.0));

            let seen = law_holds(&cfg, &mut state, 20, |n, tracked| {
                if n == 0 {
                    Offered::commanded(*tracked, command)
                } else {
                    Offered::read(*tracked)
                }
            });

            assert!(
                seen.aborted
                    .iter()
                    .any(|abort| matches!(abort, MoveAbort::StepTooLarge { .. })),
                "{seen:?}"
            );
            assert!(!seen.completed, "an aborted move does not complete");
        }

        /// A refused command, which changes nothing but is reported: the state
        /// either side of it is the same one, and the report has to match.
        #[test]
        fn a_rejection_survives_the_round_trip() {
            let cfg = MotionConfig::default();
            let (mut state, _) = armed_at_stow(&cfg);
            let unreachable = move_to(pose_at(10.0), secs(1.0));

            let seen = law_holds(&cfg, &mut state, 4, |n, tracked| {
                if n == 1 {
                    Offered::commanded(*tracked, unreachable)
                } else {
                    Offered::read(*tracked)
                }
            });

            assert_eq!(seen.rejected, 1, "{seen:?}");
            assert!(seen.faults.is_empty(), "a refusal is not a fault: {seen:?}");
        }

        /// A slot holding a mode with no path to sample is refused rather than
        /// picked up by a machine that would drop to holding on its next tick
        /// without saying why.
        #[test]
        fn a_move_with_no_path_is_refused() {
            let cfg = MotionConfig::default();
            let (mut state, _) = armed_at_stow(&cfg);
            state.mode = MotionMode::Moving;
            state.moving_elapsed = SlotDuration::from_nanos(500_000_000);
            assert_eq!(
                state.resumes().err(),
                Some(StateError::MovingWithoutTrajectory)
            );
        }

        /// A slot holding a path nobody is sampling is refused too: a mode and a
        /// path are written and cleared together.
        #[test]
        fn a_path_nothing_is_sampling_is_refused() {
            let cfg = MotionConfig::default();
            let (mut state, pinned) = armed_at_stow(&cfg);
            tick_with(
                &cfg,
                &mut state,
                secs(0.0),
                &pinned,
                Some(&move_to(JointTargets::default(), secs(2.0))),
            );
            state.mode = MotionMode::Holding;
            assert_eq!(
                state.resumes().err(),
                Some(StateError::HoldingWithTrajectory)
            );
        }

        /// A count of held setpoints the ring cannot hold is refused. Nothing
        /// live writes one — the push saturates — so the exposure is a
        /// truncated or foreign slot, and read as a full ring it would step
        /// every joint's prediction toward a setpoint no driver held.
        #[test]
        fn a_held_count_past_the_ring_is_refused() {
            let cfg = MotionConfig::default();
            let (mut state, _) = armed_at_stow(&cfg);
            state.held_count = u8::try_from(RESPONSE_DEAD_SAMPLES + 1).expect("a small depth");
            assert_eq!(
                state.resumes().err(),
                Some(StateError::HeldCountPastTheRing)
            );
            state.held_count = u8::try_from(RESPONSE_DEAD_SAMPLES).expect("a small depth");
            assert_eq!(
                state.resumes(),
                Ok(()),
                "a full ring is a state ticks reach"
            );
        }

        /// A seed that is no path is refused with the constructor's own reason.
        #[test]
        fn a_path_that_cannot_be_built_is_refused() {
            let cfg = MotionConfig::default();
            let (mut state, pinned) = armed_at_stow(&cfg);
            tick_with(
                &cfg,
                &mut state,
                secs(0.0),
                &pinned,
                Some(&move_to(JointTargets::default(), secs(2.0))),
            );
            // A running move with its head clock taken out from under it: two
            // command sets and no time to travel between them.
            state.trajectory.dur_head = SlotDuration::from_nanos(0);
            assert_eq!(
                state.resumes().err(),
                Some(StateError::Seed(SeedError::Path(
                    TrajectoryError::NonPositiveDuration
                )))
            );
        }

        /// A seed whose endpoints are not command sets is refused before the
        /// path is built: a slot nothing wrote holds a quaternion that is no
        /// rotation, which is the case that has to be caught.
        #[test]
        fn a_seed_whose_endpoints_are_not_poses_is_refused() {
            let cfg = MotionConfig::default();
            let (mut state, _) = armed_at_stow(&cfg);
            state.trajectory.present = true.into();
            assert_eq!(
                state.resumes().err(),
                Some(StateError::Seed(SeedError::Pose(
                    PoseSnapshotError::NotARotation(0.0)
                )))
            );
        }

        /// A parked machine's fault is one it can stand parked on. The three
        /// outcomes the tick reports on the same channel — a refused command, a
        /// move abandoned — are things that happened, not conditions the
        /// machine is in, and a slot claiming to be parked on one describes no
        /// state.
        #[test]
        fn an_outcome_is_not_a_fault_a_machine_can_stand_parked_on() {
            use crate::fault::FaultKind;

            let cfg = MotionConfig::default();
            for code in [
                FaultKind::CommandRejected,
                FaultKind::MoveAbortedEnvelope,
                FaultKind::MoveAbortedStep,
                FaultKind::None,
            ] {
                let (mut state, _) = armed_at_stow(&cfg);
                state.mode = MotionMode::Faulted;
                state.fault.code = code;
                assert_eq!(
                    state.resumes().err(),
                    Some(StateError::Fault(FaultError::NoStandingFault(code))),
                    "{code}"
                );
            }
        }

        /// A clock that runs backwards is no length of time, on either of the
        /// two the state carries: the previous tick's reading, and how far into
        /// a running move.
        #[test]
        fn a_clock_that_runs_backwards_is_refused() {
            let cfg = MotionConfig::default();

            let (mut state, _) = armed_at_stow(&cfg);
            state.prev_now_valid = true.into();
            state.prev_now = SlotDuration::from_nanos(-1);
            assert_eq!(
                state.resumes().err(),
                Some(StateError::Duration(DurationError::Negative(-1)))
            );

            // The same count on a state that never ticked means nothing and is
            // not read.
            state.prev_now_valid = false.into();
            state.resumes().expect("an unset clock is not read");

            let (mut moving, pinned) = armed_at_stow(&cfg);
            tick_with(
                &cfg,
                &mut moving,
                secs(0.0),
                &pinned,
                Some(&move_to(JointTargets::default(), secs(2.0))),
            );
            assert_eq!(moving.mode, MotionMode::Moving);
            moving.moving_elapsed = SlotDuration::from_nanos(-1);
            assert_eq!(
                moving.resumes().err(),
                Some(StateError::Duration(DurationError::Negative(-1)))
            );
        }

        /// The solver seed is a pose in its own right, and a slot whose
        /// quaternion is not a rotation is refused for it — the field the next
        /// present-pose solve starts from, not the endpoints of a move.
        #[test]
        fn a_solver_seed_that_is_no_pose_is_refused() {
            let cfg = MotionConfig::default();
            let (mut state, _) = armed_at_stow(&cfg);
            state.fk_seed_quat.w = 0.0;
            state.fk_seed_quat.x = 0.0;
            state.fk_seed_quat.y = 0.0;
            state.fk_seed_quat.z = 0.0;
            assert_eq!(
                state.resumes().err(),
                Some(StateError::Pose(PoseSnapshotError::NotARotation(0.0)))
            );
        }

        /// A number nobody can place, in any of the state's scalar fields, is
        /// refused by the field's own name. A rotation is caught by its length;
        /// these are the fields no length covers, and a state carrying one
        /// resumes into a machine that refuses every command afterwards and
        /// raises nothing.
        #[test]
        fn a_number_nobody_can_place_is_refused_by_field() {
            let cfg = MotionConfig::default();
            let damage: [(&str, Damage); 14] = [
                ("last goal", |state| state.last_goal.body_yaw = f64::NAN),
                ("last goal", |state| state.last_goal.antenna_left = f64::NAN),
                ("last command set", |state| {
                    state.last_targets.body_yaw = f64::INFINITY;
                }),
                ("last command set", |state| {
                    state.last_targets.head_pos.z = f64::NAN;
                }),
                ("solver seed", |state| state.fk_seed_pos.x = f64::NAN),
                ("present margin", |state| {
                    state.present_min_margin = f64::NAN;
                }),
                ("excursion allowance", |state| {
                    state.excursion_cranks[3] = f64::NAN;
                }),
                ("excursion allowance", |state| {
                    state.excursion_cone = f64::NEG_INFINITY;
                }),
                // The plant's own state, which the comparison is arithmetic
                // over: a trajectory nobody can place makes every residual
                // after it one too, and `outside_limit` would then decide a
                // fault on a number that is not one. All five fields of a row,
                // because the sweep reads them through one iterator and a later
                // edit dropping one of them would leave that row unscreened.
                ("modelled trajectory", |state| {
                    state.tracking[3].predicted = f64::NAN;
                }),
                ("modelled trajectory", |state| {
                    state.tracking[6].generator = f64::NAN;
                }),
                ("modelled trajectory", |state| {
                    state.tracking[0].velocity = f64::INFINITY;
                }),
                ("modelled trajectory", |state| {
                    state.tracking[8].anchor_predicted = f64::NAN;
                }),
                ("modelled trajectory", |state| {
                    state.tracking[5].anchor_present = f64::NEG_INFINITY;
                }),
                ("held setpoint", |state| {
                    state.held[0].antenna_left = f64::NAN;
                }),
            ];
            for (field, damage) in damage {
                let (mut state, _) = armed_at_stow(&cfg);
                state.resumes().expect("the fixture resumes undamaged");
                damage(&mut state);
                assert_eq!(
                    state.resumes().err(),
                    Some(StateError::NonFinite(field)),
                    "{field}"
                );
            }
        }

        /// A length of time past what the state's count reaches is stored at
        /// the count's own ceiling. Both clocks the tick keeps go through this
        /// one conversion, and a ceiling reached advances no move: the pair
        /// stops growing together rather than wrapping or restarting.
        #[test]
        fn a_clock_past_what_the_count_reaches_stands_at_its_ceiling() {
            // Beyond the 292 years a signed nanosecond count reaches.
            let unreachable = Duration::from_secs(1 << 40);
            assert_eq!(counted(unreachable).as_nanos(), i64::MAX);
            assert_eq!(counted(Duration::from_nanos(7)).as_nanos(), 7);
            assert_eq!(instant(counted(PERIOD)), PERIOD, "below it, the bits");

            // And the tick's own reading with it: an instant nobody's clock
            // reaches is not a refusal, not a fault, and leaves a state that
            // still resumes.
            let cfg = MotionConfig::default();
            let (mut state, pinned) = armed_at_stow(&cfg);
            let mut out = TickOutputs::default();
            motion_tick(
                &cfg,
                &mut state,
                &TickInputs {
                    now: unreachable,
                    period: PERIOD,
                    present: Some(&pinned),
                    commanded: None,
                    command: None,
                    health: None,
                },
                &mut out,
            );
            assert_eq!(state.prev_now.as_nanos(), i64::MAX, "the clock's ceiling");
            assert_eq!(out.report.fault, None, "a clock is not a fault");
            assert_eq!(out.report.aborted, None, "nor is it an abandoned move");
            assert_eq!(state.mode, MotionMode::Holding, "the machine holds");
            state
                .resumes()
                .expect("a state at the ceiling is still a state");

            // The move's clock is the same conversion over an elapsed time
            // already at the ceiling: another period on top of it stays there.
            assert_eq!(
                counted(instant(SlotDuration::from_nanos(i64::MAX)).saturating_add(PERIOD))
                    .as_nanos(),
                i64::MAX,
            );
        }

        /// A slot claiming a move it holds nothing to sample drops to holding
        /// and says which way the state disagreed with itself, rather than
        /// stopping the machine with a mode change and no reason.
        #[test]
        fn a_move_with_nothing_to_sample_is_abandoned_out_loud() {
            let cfg = MotionConfig::default();
            let (mut state, pinned) = armed_at_stow(&cfg);
            tick_with(
                &cfg,
                &mut state,
                secs(0.0),
                &pinned,
                Some(&move_to(JointTargets::default(), secs(2.0))),
            );

            // The seed withdrawn from under a running move, and then a seed
            // that is there and rebuilds no path: the two ways the pair the
            // tick writes together can be found apart.
            let cases: [(Damage, StateError); 2] = [
                (
                    |state| traj::clear_seed(&mut state.trajectory),
                    StateError::MovingWithoutTrajectory,
                ),
                (
                    |state| state.trajectory.dur_head = SlotDuration::from_nanos(0),
                    StateError::Seed(SeedError::Path(TrajectoryError::NonPositiveDuration)),
                ),
            ];
            for (damage, why) in cases {
                let mut damaged = Armed::of(state.bytes().clone());
                damage(&mut damaged);
                let mut out = TickOutputs::default();
                motion_tick(
                    &cfg,
                    &mut damaged,
                    &TickInputs {
                        now: secs(0.02),
                        period: PERIOD,
                        present: Some(&pinned),
                        commanded: None,
                        command: None,
                        health: None,
                    },
                    &mut out,
                );
                assert_eq!(damaged.mode, MotionMode::Holding, "{why}");
                assert_eq!(out.report.mode, MotionMode::Holding, "{why}");
                assert_eq!(out.report.unsampleable, Some(why), "{why}");
                assert_eq!(out.report.aborted, None, "no plan was abandoned");
                assert!(out.goal.is_none(), "{why}");
                assert_eq!(out.report.fault, None, "a wrong state is not a fault");
            }
        }

        /// A slot nothing has written is no state at all: the mode's zero names
        /// no mode, which is the refusal a host picking up a fresh slot gets.
        #[test]
        fn a_slot_nothing_wrote_is_no_state() {
            let state = Armed::of(MotionSnapWire::new());
            assert_eq!(state.resumes().err(), Some(StateError::NoMode));
        }

        /// A faulted state keeps the path it stopped on. Nothing samples it
        /// again, but it is half of a pair the tick sets and clears together,
        /// and a restore that refused it would refuse a state the tick reaches.
        #[test]
        fn a_path_left_by_a_latching_fault_restores() {
            let cfg = MotionConfig {
                read_loss_ticks: 2,
                ..MotionConfig::default()
            };
            let (mut state, pinned) = armed_at_stow(&cfg);
            tick_with(
                &cfg,
                &mut state,
                secs(0.0),
                &pinned,
                Some(&move_to(JointTargets::default(), secs(2.0))),
            );
            for n in 1..=3 {
                let mut out = TickOutputs::default();
                motion_tick(
                    &cfg,
                    &mut state,
                    &TickInputs {
                        now: secs(f64::from(n) * 0.02),
                        period: PERIOD,
                        present: None,
                        commanded: None,
                        command: None,
                        health: None,
                    },
                    &mut out,
                );
            }
            assert_eq!(state.mode, MotionMode::Faulted);
            assert!(
                bool::from(state.trajectory.present),
                "the path was cleared after all"
            );
            state.resumes().expect("the state resumes");
        }

        /// A machine standing outside the envelope, commanded back inside:
        /// the only state in which the excursion mirror carries anything but
        /// zeroes, and the allowance every sample of the recovery is judged
        /// against.
        #[test]
        fn a_recovery_out_of_the_envelope_survives_the_round_trip() {
            let cfg = MotionConfig::default();
            let turned = cfg.env.body_yaw_limit + 0.05;
            let crooked = JointTargets {
                body_yaw: turned,
                ..JointTargets::default()
            };
            let (mut state, _) = armed_at(&cfg, &crooked);
            let square = move_to(JointTargets::default(), secs(4.0));

            let seen = law_holds(&cfg, &mut state, 120, |n, tracked| {
                if n == 0 {
                    Offered::commanded(*tracked, square)
                } else {
                    Offered::read(*tracked)
                }
            });

            assert!(seen.moved, "the recovery never started");
            assert!(seen.faults.is_empty(), "{seen:?}");
            assert!(
                seen.furthest_out > 0.0,
                "the excursion mirror crossed the round trip as zeroes: {seen:?}"
            );
        }

        /// Live reads whose crank angles close no linkage, for longer than
        /// their run is allowed: the pose-failure counter is the detector, and
        /// it has to cross the round trip carrying a value.
        #[test]
        fn an_unsolvable_pose_and_its_fault_survive_the_round_trip() {
            let cfg = MotionConfig {
                read_loss_ticks: 3,
                ..MotionConfig::default()
            };
            let (mut state, _) = armed_at(&cfg, &JointTargets::default());
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

            let seen = law_holds(&cfg, &mut state, 10, |n, tracked| {
                if n < 2 {
                    Offered::read(*tracked)
                } else {
                    Offered::read(impossible)
                }
            });

            assert!(
                seen.most_pose_failures >= 4,
                "the pose-failure run never reached its bound: {seen:?}"
            );
            assert_eq!(
                seen.fault_slugs(),
                vec!["measured_pose_invalid"],
                "{seen:?}"
            );
            assert!(
                (state.mode == MotionMode::Faulted),
                "an unsolvable pose latches"
            );
            assert_eq!(seen.most_misses, 0, "the reads did arrive: {seen:?}");
        }

        /// An antenna held away from its goal while the head follows: the
        /// degrade path, which takes the antennas out of service instead of
        /// winding the machine down, and leaves a mask and a cleared run to
        /// carry across.
        #[test]
        fn an_antenna_obstruction_survives_the_round_trip() {
            let cfg = tracking_cfg(0.1, 0.01, 3);
            let (mut state, pinned) = armed_at(&cfg, &JointTargets::default());

            let mut stuck = pinned;
            stuck.antennas[1] = 0.4;

            let seen = law_holds(&cfg, &mut state, 40, |_, _| Offered::read(stuck));

            assert!(
                seen.degraded
                    .iter()
                    .any(|fault| matches!(fault, Fault::AntennaObstructed { .. })),
                "{seen:?}"
            );
            assert!(seen.longest_run >= 3, "{seen:?}");
            assert!(
                flags::contains(seen.masked, JointRef::AntennaLeft),
                "the obstructed antenna was not taken out of service: {seen:?}"
            );
            assert!(
                !(state.mode == MotionMode::Faulted),
                "an antenna does not wind the machine down"
            );
        }

        /// A leg servo's own error bits: the fault that masks the joint it
        /// names and hands the machine back holding, so both the mask and the
        /// raise have to cross the round trip.
        #[test]
        fn a_head_servo_fault_survives_the_round_trip() {
            let cfg = MotionConfig::default();
            let (mut state, _) = armed_at_stow(&cfg);
            let mut flagged = healthy_servos();
            flagged[2].bits = 0x20;

            let seen = law_holds(&cfg, &mut state, 8, |n, tracked| {
                if n >= 2 {
                    Offered::swept(*tracked, flagged)
                } else {
                    Offered::swept(*tracked, healthy_servos())
                }
            });

            assert_eq!(seen.fault_slugs(), vec!["head_servo_fault"], "{seen:?}");
            assert!(
                flags::contains(seen.masked, JointRef::Leg1),
                "the flagged leg was not taken out of service: {seen:?}"
            );
        }

        /// The nine distances an allowance is made of, each a different number
        /// and each on the field that holds it: a swap of two of them, or an
        /// index shifted along the cranks, is a different value in the state.
        #[test]
        fn an_allowance_is_nine_separate_numbers_in_the_state() {
            let excursion = Excursion {
                window: [0.11, 0.22, 0.33, 0.44, 0.55, 0.66],
                body_yaw: 0.77,
                relative_yaw: 0.88,
                cone: 0.99,
            };
            let mut state = Armed::of(MotionSnapWire::new());
            excursion.store(&mut state);
            assert_eq!(state.excursion_cranks, excursion.window);
            assert_eq!(state.excursion_body_yaw, excursion.body_yaw);
            assert_eq!(state.excursion_relative_yaw, excursion.relative_yaw);
            assert_eq!(state.excursion_cone, excursion.cone);

            // And what the allowance admits is read off those fields: the same
            // distances stand no further out, a hair more anywhere does not.
            assert!(excursion.no_further_out_than(&state));
            let nudges: [(&str, Nudge); 9] = [
                ("crank 0", |worse| worse.window[0] += 1e-3),
                ("crank 1", |worse| worse.window[1] += 1e-3),
                ("crank 2", |worse| worse.window[2] += 1e-3),
                ("crank 3", |worse| worse.window[3] += 1e-3),
                ("crank 4", |worse| worse.window[4] += 1e-3),
                ("crank 5", |worse| worse.window[5] += 1e-3),
                ("body yaw", |worse| worse.body_yaw += 1e-3),
                ("relative yaw", |worse| worse.relative_yaw += 1e-3),
                ("cone", |worse| worse.cone += 1e-3),
            ];
            for (further, nudge) in nudges {
                let mut worse = excursion;
                nudge(&mut worse);
                assert!(
                    !worse.no_further_out_than(&state),
                    "{further} stood further out"
                );
            }
        }

        /// A slot holding a path in a mode that never keeps one is refused.
        ///
        /// Every way the tick reaches holding drops the path in the same
        /// statement, so this pairing is not a state it produces — and a path
        /// left standing in it reads to anything that looks as a move in
        /// flight.
        #[test]
        fn a_hold_carrying_a_path_is_refused() {
            let cfg = MotionConfig::default();
            let (mut state, pinned) = armed_at_stow(&cfg);
            tick_with(
                &cfg,
                &mut state,
                secs(0.0),
                &pinned,
                Some(&move_to(JointTargets::default(), secs(2.0))),
            );
            assert!(
                bool::from(state.trajectory.present),
                "the fixture never moved"
            );

            state.mode = MotionMode::Holding;
            assert_eq!(
                state.resumes().err(),
                Some(StateError::HoldingWithTrajectory)
            );
        }
    }
}
