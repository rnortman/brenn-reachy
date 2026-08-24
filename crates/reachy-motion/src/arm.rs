//! Taking hold of the platform: commission once, poll while resting, engage in
//! milliseconds.
//!
//! Three sequencers, in the order a process runs them.
//!
//! - [`CommissionSequencer`] runs once, before any torque: every servo present,
//!   every servo the kind it should be, every provisioned register as
//!   configured, the supply rail up, nothing latched, and the gains and profiles
//!   written. Around two hundred transactions, not one of which touches torque.
//! - [`PollSequencer`] is the resting watch: nine position reads, and on a
//!   slower cadence the supply and error-bit sweeps too. It keeps a [`Posture`]
//!   fresh — where the machine physically is, and what its rail reads — so that
//!   engaging has nothing left to find out.
//! - [`EngageSequencer`] is the fast path: check the posture against the two
//!   torque-on gates, write each goal register to the position its joint was
//!   measured at, enable torque servo by servo, and read the nine positions back
//!   to seed the trajectory. Twenty-seven transactions — tens of milliseconds at
//!   1 Mbaud, which is what lets a wake word raise the head.
//!
//! The order of those transactions is the safety property and belongs to the
//! state machines that yield them; what lives here beside them is what they
//! decide *with* — the configuration they read, the records they produce, and
//! the decisions pure enough to be settled and tested on their own.
//!
//! ## The pin, and why it may clamp
//!
//! Nothing else in this project clamps a commanded value: a value outside its
//! bound is a typed refusal, because a silently saturated command is a command
//! nobody asked for. Pinning is the exception, and it is one because of what it
//! is guarding against. The pose the platform actually rests in can lie outside
//! the travel window each servo enforces for itself, and what a servo does with
//! an out-of-window goal is not settled — it may refuse the write with a
//! data-range error, or take it and clamp, and those two look nothing alike from
//! the host. So the goal written at arm time is pinned to the nearer bound of
//! the window by us, deliberately, with the distance recorded. The alternative
//! is not "no clamp"; it is a write whose effect is unknown. The distance is a
//! report and nothing more: a machine found somewhere unexpected is measured
//! and moved out of there, never refused, because refusing it leaves the head
//! exactly where it was with nothing able to recover it but a hand.
//!
//! The antennas are not pinned into anything. They are free rotors with no
//! travel window in the servo, no linkage behind them, and no bound on the
//! command path to bring a reading inside, so their goal is the basis reading
//! itself however many turns from zero it sits — and the first move resolves its
//! direction against that frame. Body yaw is pinned where it is: a body turned
//! past its cap is a gross, visible state somebody put it in by hand, and the
//! recovery is that same hand.
//!
//! ## Two records, not one blur
//!
//! Engaging produces two descriptions of the machine and they are not the same
//! pose. The **rest** record is where the poll found the platform — the report's
//! evidence, and the reason a strange rest does not refuse anything. The
//! **armed** record is where the nine servos reported themselves once their
//! torque was on. The tick starts from the armed record alone: a tick whose
//! goals came from one pose and whose Cartesian mirror came from the other would
//! hand its first trajectory a start the machine is not at.

use core::fmt;
use core::time::Duration;

use nalgebra::Isometry3;
use reachy_kin::{
    EnvelopeConfig, FkError, FkOptions, HeadGeometry, LegAngles, below_limit, forward_kinematics,
    min_margin, neutral_head_pose, pose_margins, rest_head_pose, stow_head_pose,
};

use crate::joints::{
    self, JointGroup, JointRef, JointVector, ROW_COUNT, ROWS, ServoHealth, flags, group_of,
    joint_ref, leg_index, leg_ref, row,
};
use crate::resume::{
    GAINS_PROFILE_WRITES, PROVISION_CELLS, ResumeError, checked_cursor, no_phase, no_stray_failure,
    no_stray_field,
};
use crate::seq::{
    AbsentSet, AnswerShape, BusResult, RegId, SeqAction, SeqError, SeqStepKind, Sequencer,
    StepContext,
};
use crate::snap::{duration_from_nanos, duration_nanos};
use crate::tick::{
    ANTENNA_GOAL_MAX_RAD, ANTENNA_GOAL_MIN_RAD, YAW_GOAL_COUNT_MAX, yaw_goal_counts,
};
use crate::txn::{self, BusTxnWire};
use crate::value::{self, Value};
use crate::{cells, record, verdict};
use brenn_reachy__motion__commission_clk_rs::{
    CommissionPhaseKind, CommissionSnap, CommissionSnapWire,
};
use brenn_reachy__motion__engage_clk_rs::{EngagePhaseKind, EngageSnap, EngageSnapWire};
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use brenn_reachy__motion__poll_clk_rs::{PollPhaseKind, PollSnap, PollSnapWire};
use brenn_reachy__motion__seq_clk_rs::SeqFailureKind;
use clockwork_rs::SyncTime;

/// The nine servo IDs in bus order: body yaw, legs 1..=6, right antenna, left
/// antenna.
///
/// The platform's own numbering, and the order every nine-slot array in this
/// crate is in.
pub const SERVO_IDS: [u8; ROW_COUNT] = [10, 11, 12, 13, 14, 15, 16, 17, 18];

/// The model number each servo must report, in bus order.
///
/// Read off this platform and identical across four runs: the body-yaw servo and
/// the six legs report one number, the two antennas another. Two groups rather
/// than three — the base's gearbox is a custom part but carries no model number
/// of its own, so it answers as the same servo the legs are.
///
/// Baked rather than left as a structural check: a number a real machine
/// answered with is a regression guard, and "these six agree with each other"
/// passes on a bus of six wrong servos.
pub const EXPECTED_MODELS: [u16; ROW_COUNT] =
    [1200, 1200, 1200, 1200, 1200, 1200, 1200, 1190, 1190];

/// The Operating Mode each servo must be provisioned with, in bus order.
///
/// Two modes, for two kinds of joint. The body and the six legs run single-turn
/// position control (3): each is a bounded joint held inside a travel window the
/// servo itself enforces. The antennas run extended position (4): they are free
/// rotors with no hard stop and no window, and single-turn mode's count boundary
/// at ±180° makes the two physically adjacent angles either side of it 4094
/// counts apart — a fold parked near it becomes reachable only the long way
/// round.
///
/// The antennas' mode is this project's own provisioning rather than the
/// vendor's, written by the `provision` command and verified byte-for-byte
/// thereafter. One record, deliberately, and not a configuration key: the mode
/// is a fact about how the machine is set up, not an operator preference.
pub const EXPECTED_OPERATING_MODES: [u8; ROW_COUNT] = [3, 3, 3, 3, 3, 3, 3, 4, 4];

/// The homing offset each servo must be provisioned with, counts, in bus order.
///
/// This is the datum. Each servo applies its own offset before reporting a
/// position, so a converted count is the model's crank angle exactly when these
/// nine registers hold these nine values; the legs' alternating quarter turns
/// are what the per-leg count limits were derived from in the first place. A
/// servo answering otherwise is a provisioning fault to be repaired with the
/// vendor's tool, and no host-side correction for it exists anywhere in this
/// workspace.
///
/// One record, deliberately: two copies of the expectation would let a run
/// render two verdicts on one truth. It is not a per-unit setting and so is
/// not a configuration key.
pub const VENDOR_HOMING_OFFSETS: [i32; ROW_COUNT] =
    [0, 1024, -1024, 1024, -1024, 1024, -1024, 0, 0];

/// The supply floor torque is not switched on below, volts.
///
/// An accepted guard: a round number chosen with margin above the point where
/// the servos' own minimum-voltage alarm sits, and below anything a healthy
/// supply should sag to. It refuses to arm a machine whose rail is already low
/// rather than characterising the rail.
pub const DEFAULT_MIN_ARM_VOLTAGE: f64 = 6.0;

/// How often the supply voltage is re-read while commissioning waits for the
/// rail.
///
/// The servos update their own voltage reading about ten times a second, so
/// polling faster would return the same number twice.
pub const DEFAULT_VOLTAGE_POLL_PERIOD: Duration = Duration::from_millis(100);

/// How long commissioning waits for the rail before giving up.
pub const DEFAULT_VOLTAGE_BUDGET: Duration = Duration::from_secs(30);

/// The position loop's three gains for one servo.
///
/// Integral and derivative terms are zero on this platform: the load is a
/// linkage holding a head up against gravity, and the proportional term plus
/// the servo's own profile is what shapes it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Gains {
    /// Proportional gain.
    pub p: u16,
    /// Integral gain.
    pub i: u16,
    /// Derivative gain.
    pub d: u16,
}

impl Gains {
    /// The three gains as the one span a register write carries.
    #[must_use]
    pub fn value(self) -> Value {
        value::gains(self.p, self.i, self.d)
    }
}

impl fmt::Display for Gains {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "P {} I {} D {}", self.p, self.i, self.d)
    }
}

/// Position gains per group of joints.
///
/// The legs carry the head's weight through a six-bar linkage and the other
/// three carry almost nothing, so they are not tuned alike.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct GroupGains {
    /// The six crank servos.
    pub legs: Gains,
    /// The body yaw servo.
    pub yaw: Gains,
    /// The two antenna servos.
    pub antennas: Gains,
}

impl GroupGains {
    /// The gains for one joint's group.
    #[must_use]
    pub fn for_joint(&self, joint: JointRef) -> Gains {
        match group_of(joint) {
            Some(JointGroup::BodyYaw) => self.yaw,
            Some(JointGroup::Antennas) => self.antennas,
            // Every other ref is a crank. A ref naming no servo reaches no
            // servo, so the gains read for it are never written anywhere.
            _ => self.legs,
        }
    }
}

/// The gains this platform is armed with.
///
/// Tuned on the bench against recorded step responses, except the yaw, which
/// still carries the value the vendor's own stack writes at startup — nothing
/// has ever implicated it.
pub const DEFAULT_GAINS: GroupGains = GroupGains {
    // A proportional term alone cannot hold the head's weight up this linkage:
    // at the vendor's P-only 300 the two most loaded cranks park 3.9–4.3° short
    // of their goal and stay there. The integral term is what closes that, and
    // the derivative term is what stops it hunting; measured, the pair settles
    // inside 1.3° with no oscillation.
    legs: Gains {
        p: 800,
        i: 100,
        d: 300,
    },
    yaw: Gains { p: 200, i: 0, d: 0 },
    // Stiff enough to carry a 0.3 s sweep, which the vendor's 200 lags badly
    // enough to overshoot the crossing. No integral term: an antenna holds
    // nothing up, so there is no standing error to integrate away.
    antennas: Gains {
        p: 500,
        i: 0,
        d: 100,
    },
};

/// The servo-side motion profile, the backstop under host-side shaping.
///
/// Register units, written to every servo at arm time. Host-side interpolation
/// is what makes motion gentle; this is what bounds a goal step the host got
/// wrong, so it is deliberately modest rather than generous. No default: the
/// numbers are bench configuration, with the provenance recorded beside them
/// there.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ProfileConfig {
    /// Profile acceleration, register units.
    pub acceleration: u32,
    /// Profile velocity, register units.
    pub velocity: u32,
}

/// The registers arming verifies before it writes anything, in read order.
///
/// The closed set of provisioning: what a servo must already hold for this
/// project to command it at all. Expectations are the register's own contents in
/// whatever shape the wire layer decodes it to, not engineering units — these
/// are integers a person compares against a data sheet, and an angle converted
/// from them would be the wrong thing to compare.
pub const PROVISION_REGS: [RegId; 15] = [
    RegId::ReturnDelayTime,
    RegId::OperatingMode,
    RegId::DriveMode,
    RegId::HomingOffset,
    RegId::MinPositionLimit,
    RegId::MaxPositionLimit,
    RegId::Shutdown,
    RegId::MaxVoltageLimit,
    RegId::MinVoltageLimit,
    RegId::TemperatureLimit,
    RegId::CurrentLimit,
    RegId::VelocityLimit,
    RegId::BusWatchdog,
    RegId::ProfileAcceleration,
    RegId::ProfileVelocity,
];

/// The motion-profile registers the gains-and-profiles sweep writes per servo,
/// after the one position-gains write.
///
/// The sweep's arithmetic and the cursor bound
/// [`GAINS_PROFILE_WRITES`](crate::resume::GAINS_PROFILE_WRITES) both derive
/// from this list, so a register added here widens both rather than leaving a
/// snapshot refused at cursors the sweep legitimately reaches.
pub const PROFILE_REGS: [RegId; 2] = [RegId::ProfileAcceleration, RegId::ProfileVelocity];

/// What arming does about one servo's one provisioned register.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub enum ProvisionExpect {
    /// Do not read it.
    #[default]
    Skip,
    /// Read it and report what it holds, without judging it. For the registers
    /// whose correct value nobody has established yet: a reading in the arm
    /// report is what establishes it.
    Record,
    /// Read it and stop arming unless it holds exactly this.
    Check(Value),
}

/// Which registers arming checks, per servo.
///
/// Rows are the nine joints in bus order, columns are [`PROVISION_REGS`]. The
/// values are configuration, not code: what the servos should hold is a property
/// of how this platform was provisioned, and the sequence's job is comparing,
/// not knowing. A table with nothing to check therefore verifies nothing, and
/// whoever builds one is what stands between that and the hardware —
/// [`Self::checks`] is there to be asserted on.
#[derive(Clone, Debug, PartialEq)]
pub struct ProvisionTable {
    cells: [[ProvisionExpect; PROVISION_REGS.len()]; ROW_COUNT],
}

impl Default for ProvisionTable {
    fn default() -> Self {
        Self {
            cells: [[ProvisionExpect::Skip; PROVISION_REGS.len()]; ROW_COUNT],
        }
    }
}

impl ProvisionTable {
    /// An empty table: nothing read, nothing checked.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// The column `reg` occupies, or `None` if arming does not provision it.
    #[must_use]
    pub fn column(reg: RegId) -> Option<usize> {
        PROVISION_REGS.iter().position(|entry| *entry == reg)
    }

    /// What to do about `joint`'s `reg`, by row and column.
    ///
    /// `None` for indices outside the table, so a cursor cannot walk off it.
    #[must_use]
    pub fn at(&self, row: usize, column: usize) -> Option<ProvisionExpect> {
        self.cells.get(row)?.get(column).copied()
    }

    /// Set one cell. `false` if the register is not one arming provisions, or
    /// the joint is not one of the nine.
    pub fn set(&mut self, joint: JointRef, reg: RegId, expect: ProvisionExpect) -> bool {
        let Some(row) = row(joint) else {
            return false;
        };
        let Some(column) = Self::column(reg) else {
            return false;
        };
        self.cells[row][column] = expect;
        true
    }

    /// Set one register's cell on all nine servos.
    pub fn set_all(&mut self, reg: RegId, expect: ProvisionExpect) -> bool {
        ROWS.iter().all(|joint| self.set(*joint, reg, expect))
    }

    /// Set one register's cell on the six legs, leaving yaw and the antennas
    /// alone. The travel windows are the case: they exist per leg and mean
    /// nothing for the three single-turn joints.
    pub fn set_legs(&mut self, reg: RegId, expect: ProvisionExpect) -> bool {
        (0..6).all(|leg| self.set(leg_ref(leg), reg, expect))
    }

    /// How many cells hold an expectation arming will fail on.
    #[must_use]
    pub fn checks(&self) -> usize {
        self.cells
            .iter()
            .flatten()
            .filter(|cell| matches!(cell, ProvisionExpect::Check(_)))
            .count()
    }

    /// How many cells arming reads at all.
    #[must_use]
    pub fn reads(&self) -> usize {
        self.cells
            .iter()
            .flatten()
            .filter(|cell| !matches!(cell, ProvisionExpect::Skip))
            .count()
    }
}

/// Everything arming needs to know that is not the machine itself.
///
/// No `Default`: two of these fields have no defensible default. The
/// provisioning table's contents are a property of how this unit was set up, and
/// the travel windows are the fence the servos themselves enforce — inventing
/// either would produce a configuration that arms a machine while checking
/// nothing about it.
#[derive(Clone, Debug, PartialEq)]
pub struct ArmConfig {
    /// The nine servo IDs in bus order.
    pub ids: [u8; ROW_COUNT],
    /// What the provisioned registers must hold.
    pub expected: ProvisionTable,
    /// The supply floor, volts.
    pub min_arm_voltage: f64,
    /// How often to re-read the supply while waiting for it.
    pub voltage_poll_period: Duration,
    /// How long to wait for the supply before failing.
    pub voltage_budget: Duration,
    /// Position gains, written fresh at every arm because they live in RAM and
    /// power-on clears them.
    pub gains: GroupGains,
    /// The servo-side profile written alongside the gains.
    pub profile: ProfileConfig,
    /// Each leg's travel window in model radians: the window the servo itself
    /// refuses to be commanded past, which is what a pin pins into.
    ///
    /// Not the envelope's crank windows. These are the servo-side fence, and
    /// they map strictly inside the host-side one; the two agreeing is a
    /// property of configuration, checked where the configuration is built,
    /// because a host envelope and a servo envelope guarding different regions
    /// is a divergence nothing on the command path would notice.
    pub leg_windows: [(f64, f64); 6],
}

impl ArmConfig {
    /// The servo ID for one joint, or `None` for a leg index past the sixth.
    #[must_use]
    pub fn id_for(&self, joint: JointRef) -> Option<u8> {
        row(joint).and_then(|row| self.ids.get(row).copied())
    }
}

/// Which bus row answers at `id`, or `None` where nothing on this bus does.
///
/// The reverse of [`ArmConfig::id_for`], over [`SERVO_IDS`]: a host reading an id
/// off a bus message needs the row to index its nine-slot arrays with, and the
/// ids are the platform's own numbering rather than a table a host may restate.
#[must_use]
pub fn row_of_id(id: u8) -> Option<usize> {
    SERVO_IDS.iter().position(|configured| *configured == id)
}

/// How far inside the host envelope's crank windows the servo-side fence sits,
/// degrees.
///
/// A hair, and its direction is the whole point: the servo refuses a goal past
/// its own window, so a fence *outside* the host's would let a pin command an
/// angle the servo declines, and a fence far inside it would refuse poses the
/// envelope allows. The provisioned windows this platform's units answered with
/// sit between 0.012° and 0.039° inside the corresponding envelope bound, and
/// the tightest of them is the one worth modelling.
pub const WINDOW_INSET_DEG: f64 = 0.012;

/// The servo-side travel windows `env`'s own windows imply: those windows, drawn
/// in by [`WINDOW_INSET_DEG`].
#[must_use]
pub fn leg_windows(env: &EnvelopeConfig) -> [(f64, f64); 6] {
    let inset = WINDOW_INSET_DEG.to_radians();
    let mut windows = env.crank_windows;
    for (low, high) in &mut windows {
        *low += inset;
        *high -= inset;
    }
    windows
}

/// Arming's configuration over this platform's hardware facts.
///
/// One record of which fields a host fills and where each comes from: the nine
/// ids, the supply floor and its polling, the gains and the servo-side fences
/// are facts this crate states once, and two copies of any of them would let one
/// host commission a machine against fences another host's tests never see.
///
/// The two arguments are the two things that are not hardware facts. `expected`
/// is the provisioning grid — what this deployment bakes an expectation for, and
/// empty where a caller checks nothing. `profile` is the servo-side
/// velocity/acceleration backstop, which this crate deliberately has no default
/// for: what it should be is a property of the machine a host drives and of the
/// shaping that host does, not of the motion arithmetic here.
#[must_use]
pub fn arm_config(
    env: &EnvelopeConfig,
    expected: ProvisionTable,
    profile: ProfileConfig,
) -> ArmConfig {
    ArmConfig {
        ids: SERVO_IDS,
        expected,
        min_arm_voltage: DEFAULT_MIN_ARM_VOLTAGE,
        voltage_poll_period: DEFAULT_VOLTAGE_POLL_PERIOD,
        voltage_budget: DEFAULT_VOLTAGE_BUDGET,
        gains: DEFAULT_GAINS,
        profile,
        leg_windows: leg_windows(env),
    }
}

/// Where a pin put each joint, and how far it had to pull.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PinOutcome {
    /// The goals to write: the measured angles, with each leg brought inside its
    /// travel window.
    pub pinned: JointVector,
    /// Per leg, how far the pin moved it, radians. Zero for a leg already
    /// inside its window, which is the ordinary case on an already-armed
    /// machine.
    pub pull_in: [f64; 6],
}

impl PinOutcome {
    /// The largest pull applied to a leg, radians.
    ///
    /// The legs alone, because they are the only joints a pin moves.
    #[must_use]
    pub fn worst_pull_in(&self) -> f64 {
        self.pull_in.iter().copied().fold(0.0, f64::max)
    }
}

/// The goals to pin the platform at, given where it says it is.
///
/// Each leg is brought inside its own travel window. Body yaw and the antennas
/// are pinned where they stand: the yaw servo's provisioned range is the whole
/// turn and a body outside its cap is a state a hand made and a hand undoes, and
/// an antenna is a free rotor whose reading is a direction in a continuous frame
/// with no bound anywhere to bring it inside. How far each leg was pulled is
/// recorded and never judged. What does stop arming is a measured angle nobody
/// can place: pinning a goal to a value that is not a number would send a
/// meaningless write to a servo about to take the head's weight.
pub fn pin_goals(cfg: &ArmConfig, present: &JointVector) -> Result<PinOutcome, SeqError> {
    pin_goals_from(cfg, present, present)
}

/// [`pin_goals`] with the angles the goals are computed from separated from the
/// angles the pull is measured against.
///
/// The two are the same reading wherever a servo was found limp: the pin is its
/// present position, brought into range. They differ for a servo found already
/// holding torque, whose honest pin is the target it is holding rather than the
/// position it has sagged to under load — pinning at the sag would lower the
/// target by that sag on every re-arm. The pull stays measured against where the
/// joint actually is, so the figure recorded for a leg says how far the write is
/// about to move it. The legs alone carry one: body yaw and the antennas are
/// pinned at their basis untouched.
pub fn pin_goals_from(
    cfg: &ArmConfig,
    basis: &JointVector,
    measured: &JointVector,
) -> Result<PinOutcome, SeqError> {
    all_placeable(cfg, measured, RegId::PresentPosition)?;
    all_placeable(cfg, basis, RegId::GoalPosition)?;

    let mut pinned = *basis;
    let mut pull_in = [0.0; 6];
    // Walked in bus order, so the joint named and the leg pinned both come from
    // the one ordering table rather than from an offset restated here.
    for (joint, target) in basis.joints() {
        let Some(leg) = leg_index(joint) else {
            continue;
        };
        let leg = usize::from(leg);
        let (low, high) = cfg.leg_windows[leg];

        let angle = if target < low {
            low
        } else if target > high {
            high
        } else {
            target
        };
        pinned.legs[leg] = angle;
        pull_in[leg] = (angle - measured.legs[leg]).abs();
    }

    Ok(PinOutcome { pinned, pull_in })
}

/// Refuse a set of angles holding one nobody can place.
///
/// Split out of the placement so a caller that must not place — a resume,
/// reading pins a slot already holds — can ask the refusal on its own. `reg` is
/// the register the angles belong to, which is what the refusal names.
fn all_placeable(cfg: &ArmConfig, vector: &JointVector, reg: RegId) -> Result<(), SeqError> {
    for (row, (joint, angle)) in vector.joints().into_iter().enumerate() {
        if !angle.is_finite() {
            return Err(SeqError::UnplaceableAngle {
                context: StepContext::reg(SeqStepKind::PinAndEnable, cfg.ids[row], reg),
                joint,
                angle,
            });
        }
    }
    Ok(())
}

/// Whether `angle` is inside the travel window crank `leg` is placed in.
///
/// The membership question alone, with no placement: [`pin_goals_from`] pulls an
/// out-of-window basis angle to the window edge and cannot fail, which is the
/// right answer for a measured position and the wrong one for a goal a slot
/// already holds. A reader of stored pins asks this and refuses, because pulling
/// a commanded value to an edge is the clamp this repo does not do.
#[must_use]
pub fn in_leg_window(cfg: &ArmConfig, leg: usize, angle: f64) -> bool {
    let (low, high) = cfg.leg_windows[leg];
    low <= angle && angle <= high
}

/// A record a resumed engagement plans from, read before the sequencer exists.
///
/// Asked only where the phase says the record was written: the resting one
/// always has been by the time a sequence runs, and the armed one only once the
/// settle sweep has solved it. The pose read refuses a quaternion that is no
/// rotation and nothing else, so the rest of the numbers — the nine angles, the
/// translation, the six margins and the smallest of them — are walked here.
fn checked_record(
    record_name: &'static str,
    phase: &'static str,
    slot: &record::ArmRecordSnap,
) -> Result<(), ResumeError> {
    let Some(held) = record::read(slot).map_err(|source| ResumeError::RecordNotAPose {
        record: record_name,
        source,
    })?
    else {
        return Err(ResumeError::RecordMissing {
            record: record_name,
            phase,
        });
    };
    let margins = held.margins.iter().copied().chain([held.min_margin]);
    for (field, finite) in [
        ("joints", held.joints.first_non_finite().is_none()),
        (
            "head translation",
            held.head_pose_body
                .translation
                .vector
                .iter()
                .all(|n| n.is_finite()),
        ),
        ("margins", margins.clone().all(f64::is_finite)),
    ] {
        if !finite {
            return Err(ResumeError::NonFinite {
                record: record_name,
                field,
            });
        }
    }
    Ok(())
}

/// The pinned goals a resumed engagement writes to nine servos, read before the
/// sequencer exists.
///
/// The goal sweep hands each of these straight to a `GoalPosition` write, so
/// this is the whole of what stands between the slot bytes and a servo about to
/// take the head's weight. Out-of-window is a refusal rather than a placement:
/// see [`in_leg_window`].
///
/// The six cranks are bounded by their travel windows. Body yaw and the two
/// antennas are bounded by what their goal registers represent — the yaw
/// servo's provisioned single turn, the antennas' extended-position span —
/// which is a hardware fact rather than a rule about how far a pin may sit
/// from the posture. The task-space yaw cap is deliberately not the bound:
/// pinning legitimately exceeds it, and refusing on it would release a session
/// a hand could still recover. Each of these three refuses a pin the servo
/// would refuse at the write, so the engagement was stopping either way and the
/// refusal only moves the stop ahead of torque-on.
///
/// The pull-in figures are checked finite and no further: they are a report of
/// how far placement moved, never written to a servo.
fn checked_pins(cfg: &ArmConfig, slot: &record::PinSnap) -> Result<(), ResumeError> {
    let pins = record::pins_of(slot);
    for (field, finite) in [
        ("pinned goals", pins.pinned.first_non_finite().is_none()),
        (
            "pull-in figures",
            pins.pull_in.iter().all(|n| n.is_finite()),
        ),
    ] {
        if !finite {
            return Err(ResumeError::NonFinite {
                record: "pins",
                field,
            });
        }
    }
    for leg in 0..joints::LEG_COUNT {
        let pin = pins.pinned.legs[leg];
        if !in_leg_window(cfg, leg, pin) {
            let (low, high) = cfg.leg_windows[leg];
            return Err(ResumeError::PinOutOfWindow {
                leg,
                pin,
                low,
                high,
            });
        }
    }
    let yaw = pins.pinned.body_yaw;
    let counts = yaw_goal_counts(yaw);
    if !(0.0..=YAW_GOAL_COUNT_MAX).contains(&counts) {
        return Err(ResumeError::YawPinNoCount {
            pin: yaw,
            counts,
            bound: YAW_GOAL_COUNT_MAX,
        });
    }
    for (side, joint) in ["right", "left"].into_iter().enumerate() {
        let pin = pins.pinned.antennas[side];
        if !(ANTENNA_GOAL_MIN_RAD..=ANTENNA_GOAL_MAX_RAD).contains(&pin) {
            return Err(ResumeError::AntennaPinNoCount {
                joint,
                pin,
                low: ANTENNA_GOAL_MIN_RAD,
                high: ANTENNA_GOAL_MAX_RAD,
            });
        }
    }
    Ok(())
}

/// One description of the machine: nine angles and the head pose they hold.
///
/// Produced twice by arming — once for where the platform was found, once for
/// what it was left holding — and the second one is what a tick is started from.
/// The pose is solved from the angles rather than assumed, so a record cannot
/// claim a pose the legs do not put the head at.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmRecord {
    /// The nine angles: measured, for the rest record; pinned, for the armed
    /// one.
    pub joints: JointVector,
    /// The head pose those leg angles hold, relative to the body.
    pub head_pose_body: Isometry3<f64>,
    /// Per-leg toggle margins at that pose, metres.
    pub margins: [f64; 6],
    /// The smallest of them, metres: the clearance a first move is measured
    /// against.
    pub min_margin: f64,
}

impl ArmRecord {
    /// Solve the pose `joints`' legs hold, trying each seed in turn.
    ///
    /// The solver is seeded rather than global, and which assembly mode it lands
    /// in is the seed's doing, so a caller with two candidate poses for a resting
    /// platform hands both over and takes the first that closes the linkage
    /// plausibly. No seed working is a refusal, never a guess: the last solver
    /// failure is returned as it came, because an unsolvable set of angles at
    /// arm time says the model and the machine disagree, and reads as such.
    pub fn solve(
        geom: &HeadGeometry,
        opts: &FkOptions,
        joints: &JointVector,
        seeds: &[Isometry3<f64>],
    ) -> Result<Self, FkError> {
        let angles = LegAngles(joints.legs);
        let mut last = FkError::NoConvergence {
            iters: 0,
            residual: f64::INFINITY,
        };
        for seed in seeds {
            let mut head_pose_body = Isometry3::identity();
            match forward_kinematics(geom, &angles, seed, opts, &mut head_pose_body) {
                Ok(_) => {
                    let mut margins = [0.0; 6];
                    pose_margins(geom, &head_pose_body, &mut margins);
                    return Ok(Self {
                        joints: *joints,
                        head_pose_body,
                        margins,
                        min_margin: min_margin(&margins),
                    });
                }
                Err(error) => last = error,
            }
        }
        Err(last)
    }
}

/// What commissioning established about the machine.
///
/// Every register of record a bring-up wants written down, and nothing about
/// where the platform is standing: the pose is the poll's subject and changes
/// under a hand, while these are facts about how the unit was set up. Held by
/// value so a report can be printed after the sequencer is gone.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CommissionSummary {
    /// Each servo's model number, in bus order.
    pub models: [u16; ROW_COUNT],
    /// The supply and error-bit readings commissioning finished on.
    pub rail: Rail,
    /// How many sweeps the voltage gate took.
    pub voltage_polls: u32,
}

/// The supply and the latched error bits, as one sweep read them.
///
/// The two torque-on gates read from here rather than from the wire, which is
/// what makes engaging fast: the readings are already in hand from the resting
/// watch, so the gate is arithmetic instead of eighteen transactions.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rail {
    /// Each servo's supply reading, volts.
    pub voltages: [f64; ROW_COUNT],
    /// Each servo's hardware-error byte, as read.
    pub health: [ServoHealth; ROW_COUNT],
}

/// Where the machine is, and what its rail reads: one sweep of the resting
/// watch.
///
/// Kept fresh while the platform rests limp, because a hand can move the head
/// and the next engage plans from wherever it actually is. Never a verdict —
/// nothing in a posture refuses anything.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Posture {
    /// The nine measured angles, in bus order.
    pub present: JointVector,
    /// The supply and error bits: re-read by this sweep on the slow cadence,
    /// carried over from the last one otherwise.
    pub rail: Rail,
    /// Whether this sweep read the rail itself rather than carrying it.
    pub rail_read: bool,
}

/// What engaging wrote, and where it left the machine standing.
///
/// Two records of the platform, because they are different poses whenever a pin
/// had to pull a joint or torque coming on moved one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EngageSummary {
    /// Where the poll found the platform, before anything was written.
    pub rest: ArmRecord,
    /// Where the nine servos reported themselves once torque was on, and what a
    /// tick starts from.
    pub armed: ArmRecord,
    /// The goals written to the servos, and how far each pin pulled a leg.
    pub pins: PinOutcome,
    /// Per joint, its position after torque came on less the position it was
    /// resting at, radians.
    ///
    /// Enabling torque in position mode can renormalise a servo's reported
    /// position onto a single turn, which shows up here as a jump of about a
    /// whole turn on a joint that had settled past the half. Recorded rather
    /// than refused: the armed record is read back after the enables, so
    /// whatever this says is already in the pose the tick starts from.
    pub post_enable_shift: [f64; ROW_COUNT],
    /// The joints this engagement left out of service: never enabled, never
    /// commanded, still measured.
    ///
    /// Only ever antennas — bits on a servo that carries the head refuse the
    /// engagement outright ([`engage_gates`]) — and the set a tick starts its
    /// mask from.
    pub degraded: JointFlags,
}

impl EngageSummary {
    /// The largest distance the pin pulled any leg, radians. The legs alone:
    /// nothing else is pulled.
    #[must_use]
    pub fn worst_pull_in(&self) -> f64 {
        self.pins.worst_pull_in()
    }
}

/// The two torque-on gates, against the readings a poll already has in hand.
///
/// The whole enumeration of what may stand between a request to move and torque
/// coming on: a rail too low to lift the head without browning out, and a servo
/// that has latched a hardware error. The machine's position is deliberately not
/// among them — where it stands is measured and planned from, never refused —
/// and nothing whatsoever gates torque going *off*.
///
/// A refusal here is an expected error, not a fault: nothing has been written,
/// the machine is exactly where it was, and the next request tries again.
///
/// The error-bit gate answers by group, and what comes back on the passing side
/// is the set of joints the engagement leaves out of service. Bits on a servo
/// that carries the head refuse outright: it is holding the head up, there is no
/// rebooting it, and lifting on a servo already flagging is exactly the hazard
/// the gate is here for. Bits on an antenna alone refuse nothing. The pair
/// carries no load and is its own mechanism, so one latched overload — the
/// residue an antenna interference incident leaves behind — must not cost every
/// wake until somebody walks over and cycles the power. That antenna is named
/// here, never enabled, and never commanded for the rest of the engagement.
pub fn engage_gates(cfg: &ArmConfig, rail: &Rail) -> Result<JointFlags, SeqError> {
    let limit = cfg.min_arm_voltage;
    if let Some((row, lowest)) = worst_below(&rail.voltages, limit) {
        return Err(SeqError::SupplyBelowFloor {
            context: StepContext::reg(
                SeqStepKind::VoltageGate,
                cfg.ids[row],
                RegId::PresentInputVoltage,
            ),
            readings: rail.voltages,
            lowest,
            limit,
        });
    }

    let mut degraded = JointFlags::NONE;
    for (row, health) in rail.health.iter().enumerate() {
        if health.healthy_or_voltage_only() {
            continue;
        }
        let joint = ROWS[row];
        if group_of(joint) == Some(JointGroup::Antennas) {
            flags::insert(&mut degraded, joint);
            continue;
        }
        // No reboot, ever, and no clearing of the latch: a servo holding this
        // head that is rebooted drops it.
        return Err(SeqError::UnhealthyServo {
            context: StepContext::reg(
                SeqStepKind::Health,
                cfg.ids[row],
                RegId::HardwareErrorStatus,
            ),
            bits: health.bits,
        });
    }
    Ok(degraded)
}

/// The worst reading of a sweep that is under `limit`, and the row it came from.
///
/// The worst, not the first one under: the nine servos sit on different lengths
/// of wiring and read differently, and the servo a report names has to be the
/// one worst off. A reading nobody can place wins outright — `below_limit` is
/// the bound test the whole project uses, so a NaN counts as "not up" rather
/// than passing every comparison by default.
///
/// One home for the ordering, because both places that judge the rail — the
/// supply gate commissioning polls against a budget, and the torque-on gate that
/// judges the last resting sweep — have to agree about what "worst" means.
fn worst_below(readings: &[f64; ROW_COUNT], limit: f64) -> Option<(usize, f64)> {
    let mut low: Option<(usize, f64)> = None;
    for (row, value) in readings.iter().copied().enumerate() {
        if !below_limit(value, limit) {
            continue;
        }
        let worse = match low {
            None => true,
            Some((_, lowest)) => lowest.is_finite() && (!value.is_finite() || value < lowest),
        };
        if worse {
            low = Some((row, value));
        }
    }
    low
}

/// The servo standing at bus row `row`.
///
/// Every cursor in this file is bounded by [`ROW_COUNT`] by the phase
/// transitions, so a row a sweep is walking names a servo wherever this is
/// called — and a reading that named none would otherwise be recorded nowhere
/// while the sweep reported success.
fn servo_at(row: usize) -> JointRef {
    joint_ref(row).expect("bus rows are bounded by the phase cursors")
}

/// Decode one servo's supply reading into its row of a rail sweep.
///
/// The rail's shape is a fact about the machine and not about which sequencer is
/// asking, so commissioning's sweep and the resting watch's read it through the
/// same pair of functions.
fn absorb_volts(
    row: usize,
    into: &mut cells::RailSnap,
    context: StepContext,
    result: &BusResult,
) -> Result<(), SeqError> {
    let volts = value::as_volts(result.value(context)?, context)?;
    cells::set_voltage(into, servo_at(row), volts);
    Ok(())
}

/// Decode one servo's latched error byte into its row of a rail sweep.
fn absorb_health_bits(
    row: usize,
    id: u8,
    into: &mut cells::RailSnap,
    context: StepContext,
    result: &BusResult,
) -> Result<(), SeqError> {
    let bits = value::as_u8(result.value(context)?, context)?;
    cells::set_health(&mut into.health, servo_at(row), ServoHealth { id, bits });
    Ok(())
}

/// How many elements a commissioning phase's sweep walks, or `None` for the two
/// endings and the phase that names nothing, which have no cursor.
fn commission_sweep(phase: CommissionPhaseKind) -> Option<usize> {
    match phase {
        CommissionPhaseKind::Presence
        | CommissionPhaseKind::Identity
        | CommissionPhaseKind::Voltage
        | CommissionPhaseKind::Health => Some(ROW_COUNT),
        CommissionPhaseKind::Provision => Some(PROVISION_CELLS),
        CommissionPhaseKind::GainsProfiles => Some(GAINS_PROFILE_WRITES),
        CommissionPhaseKind::None | CommissionPhaseKind::Complete | CommissionPhaseKind::Failed => {
            None
        }
    }
}

/// The name a refusal about a commissioning phase reads under.
///
/// Operator prose rather than the generated `Display`, for the reason every
/// display adapter in this crate exists: what a person reads off a report is
/// this layer's opinion of a vocabulary it does not own.
fn commission_phase_name(phase: CommissionPhaseKind) -> &'static str {
    match phase {
        CommissionPhaseKind::None => "no",
        CommissionPhaseKind::Presence => "presence",
        CommissionPhaseKind::Identity => "identity",
        CommissionPhaseKind::Provision => "provision",
        CommissionPhaseKind::Voltage => "voltage",
        CommissionPhaseKind::Health => "health",
        CommissionPhaseKind::GainsProfiles => "gains-and-profiles",
        CommissionPhaseKind::Complete => "complete",
        CommissionPhaseKind::Failed => "failed",
    }
}

/// The angle at one row of the bus order.
///
/// The bus order lives once, in `JointRef`; this is a lookup through it rather
/// than a second statement of it. Every cursor in this file is bounded by
/// `ROW_COUNT` by the phase transitions, so the row is in range wherever
/// this is called.
pub(crate) fn angle_at(joints: &JointVector, row: usize) -> f64 {
    joint_ref(row)
        .and_then(|joint| joints.get(joint))
        .expect("bus rows are bounded by the phase cursors")
}

/// The poses the resting solve is seeded from: the two the platform comes to
/// rest at limp, and neutral, which is where a command that ran before this one
/// left it holding.
///
/// Every pose a command targets is one of the three, or differs from neutral
/// only in yaw and antennas, which do not move the head pose being solved for.
/// Named here rather than written inline so the list is a thing a test can read.
///
/// Public because it is the list *any* solve of a resting machine's measured
/// angles wants, and a caller that arms from a pose sample rather than from the
/// engage sequencer would otherwise write its own three and get to disagree
/// with this one about the poses a session can start from.
#[must_use]
pub fn rest_pose_seeds() -> [Isometry3<f64>; 3] {
    [stow_head_pose(), rest_head_pose(), neutral_head_pose()]
}

/// The poses the post-enable solve is seeded from: the pose the machine was
/// resting at, and then the standing list.
///
/// The resting record first, because the machine has ordinarily moved by a
/// settle at most and that is a close solve. The standing seeds follow it rather
/// than being replaced by it: torque coming on can renormalise a reported frame
/// by a whole turn, and a solve that only ever tried the pose from before that
/// would refuse a machine that is fine. Built from [`rest_pose_seeds`] so the
/// two solves cannot drift apart — a pose the resting solve reaches and the
/// armed one does not is a machine that engages and then refuses its own
/// read-back.
fn armed_pose_seeds(rest: Isometry3<f64>) -> [Isometry3<f64>; 4] {
    let [stow, resting, neutral] = rest_pose_seeds();
    [rest, stow, resting, neutral]
}

/// Commissioning, as a state machine that touches no port.
///
/// Six phases in a fixed order, one transaction at a time: every servo present,
/// every servo the kind it should be, every provisioned register as configured,
/// the supply up, nothing latched, and the gains and profiles written.
///
/// **Torque is never touched here**, in either direction, and no position is
/// read: this establishes that the machine on the bus is the machine this
/// process was configured for, and stops there. It runs once per process, so its
/// two hundred transactions are paid at startup rather than on every wake — the
/// whole reason engaging is fast.
///
/// The one order property is that no write of any kind happens before the supply
/// gate: the gains and profiles go into servo RAM, and writing them across a rail
/// that is browning out is how a servo ends up holding half a configuration.
///
/// It lives here, in one readable sequence, testable against scripted replies.
pub struct CommissionSequencer<'a> {
    cfg: &'a ArmConfig,
    state: &'a mut CommissionSnap,
}

impl<'a> CommissionSequencer<'a> {
    /// A fresh sequence, in `slot`.
    ///
    /// The slot is cleared to the schema's declared initial state and then put in
    /// the first sweep, so a slot an earlier sequence left its readings in
    /// describes this sequence and nothing else.
    pub fn start(cfg: &'a ArmConfig, slot: &'a mut CommissionSnapWire) -> Self {
        let state = slot.clear_valid();
        state.phase = CommissionPhaseKind::Presence;
        Self { cfg, state }
    }

    /// The sequence `state` holds, run against `cfg`.
    ///
    /// The configuration comes from the caller rather than the state: it is what
    /// the host was configured with, not what the sequence found out, and a
    /// second copy in the slot could disagree with the configured one.
    ///
    /// # Errors
    ///
    /// [`ResumeError`] for a phase-and-cursor pairing no sequence of steps
    /// reaches. Which numbers name a phase at all is the validated view's
    /// question, answered before this is called.
    pub fn resume(cfg: &'a ArmConfig, state: &'a mut CommissionSnap) -> Result<Self, ResumeError> {
        let phase = state.phase;
        let name = commission_phase_name(phase);
        no_phase(name, phase == CommissionPhaseKind::None)?;
        checked_cursor(name, commission_sweep(phase), state.cursor)?;
        if phase == CommissionPhaseKind::Failed {
            // The verdict is the whole content of that phase, and it is read
            // back rather than merely counted: a failed phase whose fields do
            // not amount to a failure would hand the next step an invented one.
            verdict::read(&state.failure).map_err(ResumeError::Verdict)?;
        } else {
            no_stray_failure(name, state.failure.kind != SeqFailureKind::None)?;
            // The supply gate's two fields likewise: it is the only phase that
            // writes them, and a slot carrying one anywhere else was written by
            // something that does not agree with this type about what a phase
            // holds.
            if phase != CommissionPhaseKind::Voltage {
                let owner = commission_phase_name(CommissionPhaseKind::Voltage);
                no_stray_field(
                    "voltage_started",
                    owner,
                    name,
                    state.voltage_started.as_nanos() != 0,
                )?;
                no_stray_field("voltage_waiting", owner, name, state.voltage_waiting.into())?;
            }
        }
        Ok(Self { cfg, state })
    }

    /// The fields a phase owns, blanked as it is left.
    ///
    /// The supply gate is the only phase here with any: the moment its budget is
    /// measured from, and whether the next step spaces its sweep out. A slot
    /// carrying either one in another phase is what [`Self::resume`] refuses, so
    /// a gate that passes has to leave them behind on the way out.
    fn blank_phase_fields(&mut self) {
        self.state.voltage_started = SyncTime::from_nanos(0);
        self.state.voltage_waiting = false.into();
    }

    /// Which phase a failure raised here is reported under.
    fn phase_step(&self) -> SeqStepKind {
        match self.state.phase {
            CommissionPhaseKind::Presence => SeqStepKind::Presence,
            CommissionPhaseKind::Identity => SeqStepKind::Identity,
            CommissionPhaseKind::Provision => SeqStepKind::Provision,
            CommissionPhaseKind::Voltage => SeqStepKind::VoltageGate,
            CommissionPhaseKind::Health => SeqStepKind::Health,
            CommissionPhaseKind::GainsProfiles | CommissionPhaseKind::Complete => {
                SeqStepKind::GainsProfiles
            }
            // A failure already carries the phase it happened in; taking the
            // name from anywhere else would report a supply gate that refused
            // as a write that never happened. A phase that names nothing is
            // refused on the way in, so it never reports a step.
            CommissionPhaseKind::Failed | CommissionPhaseKind::None => self.state.failure.step,
        }
    }

    /// Ask the servo at bus row `row` for `reg`.
    fn read(&mut self, row: usize, reg: RegId) {
        txn::set_read_reg(&mut self.state.pending, self.cfg.ids[row], reg);
    }

    /// Write `value` to `reg` on the servo at bus row `row`, and read it back.
    fn write(&mut self, row: usize, reg: RegId, value: Value) {
        txn::set_write_reg_verified(&mut self.state.pending, self.cfg.ids[row], reg, value);
    }

    /// The next action, given the previous transaction's result.
    fn emit(&mut self, now: Duration) -> SeqAction<CommissionSummary> {
        let cursor = self.cursor();
        match self.state.phase {
            CommissionPhaseKind::Presence => {
                txn::set_ping(&mut self.state.pending, self.cfg.ids[cursor]);
            }
            CommissionPhaseKind::Identity => self.read(cursor, RegId::ModelNumber),
            CommissionPhaseKind::Provision => {
                let (row, column) = (cursor / PROVISION_REGS.len(), cursor % PROVISION_REGS.len());
                self.read(row, PROVISION_REGS[column]);
            }
            CommissionPhaseKind::Voltage => {
                if self.state.voltage_waiting.into() {
                    // Space the sweeps out: the servos refresh their own voltage
                    // reading about ten times a second, so a faster poll reads
                    // the same number twice.
                    self.state.voltage_waiting = false.into();
                    return SeqAction::Wait {
                        until: now + self.cfg.voltage_poll_period,
                    };
                }
                self.read(cursor, RegId::PresentInputVoltage);
            }
            CommissionPhaseKind::Health => self.read(cursor, RegId::HardwareErrorStatus),
            CommissionPhaseKind::GainsProfiles => {
                if cursor < ROW_COUNT {
                    let gains = self.cfg.gains.for_joint(ROWS[cursor]);
                    self.write(cursor, RegId::PositionGains, gains.value());
                } else {
                    // The profile registers walk servo-major after the gains:
                    // one row per servo, one column per register.
                    let index = cursor - ROW_COUNT;
                    let reg = PROFILE_REGS[index % PROFILE_REGS.len()];
                    let value = match reg {
                        RegId::ProfileAcceleration => value::u32(self.cfg.profile.acceleration),
                        RegId::ProfileVelocity => value::u32(self.cfg.profile.velocity),
                        other => unreachable!("{other:?} is not a profile register"),
                    };
                    self.write(index / PROFILE_REGS.len(), reg, value);
                }
            }
            CommissionPhaseKind::Complete => return SeqAction::Done(self.summary()),
            // A phase that names nothing is refused on the way in, so a sequence
            // in one has just been failed by the arm below and hands that back.
            CommissionPhaseKind::Failed | CommissionPhaseKind::None => {
                return SeqAction::Fail(self.verdict());
            }
        }
        SeqAction::Transact
    }

    /// What the sequence established, as a report reads it.
    fn summary(&self) -> CommissionSummary {
        CommissionSummary {
            models: cells::models_of(&self.state.models),
            rail: cells::rail_of(&self.state.rail),
            voltage_polls: self.state.voltage_polls,
        }
    }

    /// Take the previous transaction's result and move the cursor on.
    fn absorb(&mut self, now: Duration, prior: Option<&BusResult>) -> Result<(), SeqError> {
        if !txn::held(&self.state.pending) {
            // Nothing was outstanding — the first call, or the call after a
            // wait. A result handed back here answers no request, so there is
            // nothing to validate it against and nothing to report it under.
            return Ok(());
        }
        let (context, wrote) = txn::fields(&self.state.pending, self.phase_step())?;
        txn::set_none(&mut self.state.pending);
        let Some(result) = prior else {
            // A transaction ran and nothing came back. From here that is
            // indistinguishable from silence on the wire, and it is treated as
            // silence rather than quietly retried.
            return Err(SeqError::NoAnswer { context });
        };
        let cursor = self.cursor();
        match self.state.phase {
            CommissionPhaseKind::Presence => self.absorb_presence(cursor, context, result),
            CommissionPhaseKind::Identity => self.absorb_identity(now, cursor, context, result),
            CommissionPhaseKind::Provision => self.absorb_provision(now, cursor, context, result),
            CommissionPhaseKind::Voltage => self.absorb_voltage(now, cursor, context, result),
            CommissionPhaseKind::Health => self.absorb_health(cursor, context, result),
            CommissionPhaseKind::GainsProfiles => {
                confirm_write(result, wrote, context)?;
                if cursor + 1 < GAINS_PROFILE_WRITES {
                    self.seek(cursor + 1);
                } else {
                    self.enter(CommissionPhaseKind::Complete);
                }
                Ok(())
            }
            // Terminal: nothing is ever outstanding, so this is unreachable.
            CommissionPhaseKind::Complete
            | CommissionPhaseKind::Failed
            | CommissionPhaseKind::None => Ok(()),
        }
    }

    fn absorb_presence(
        &mut self,
        cursor: usize,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        match result.pinged(context) {
            // Presence is about whether an answer came back at all. What kind of
            // servo answered is the identity phase's register read, which is
            // evidence a ping's own model field is not: it comes from the
            // control table.
            Ok(_) => {}
            Err(SeqError::NoAnswer { .. }) => {
                if let Some(joint) = joint_ref(cursor) {
                    flags::insert(&mut self.state.absent, joint);
                }
            }
            Err(other) => return Err(other),
        }
        if cursor + 1 < ROW_COUNT {
            self.seek(cursor + 1);
            return Ok(());
        }
        let absent = AbsentSet::new(&self.cfg.ids, &flags::rows(self.state.absent));
        if let Some(&first) = absent.ids().first() {
            return Err(SeqError::AbsentServos {
                context: StepContext::servo(SeqStepKind::Presence, first),
                absent,
            });
        }
        self.enter(CommissionPhaseKind::Identity);
        Ok(())
    }

    fn absorb_identity(
        &mut self,
        now: Duration,
        cursor: usize,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        let model = value::as_u16(result.value(context)?, context)?;
        if let Some(joint) = joint_ref(cursor)
            && let Some(cell) = cells::model_mut(&mut self.state.models, joint)
        {
            *cell = model;
        }
        if cursor + 1 < ROW_COUNT {
            self.seek(cursor + 1);
            return Ok(());
        }
        self.check_identity()?;
        self.enter_provision(now, 0)
    }

    /// Each servo against the model number this platform's servos report.
    ///
    /// A servo of the wrong kind at a roster address is a bus wired up wrong, a
    /// replaced part, or a reply attributed to the wrong servo, and every one of
    /// those has to stop arming before anything is written.
    fn check_identity(&self) -> Result<(), SeqError> {
        for (row, (model, expected)) in cells::models_of(&self.state.models)
            .into_iter()
            .zip(EXPECTED_MODELS)
            .enumerate()
        {
            if model != expected {
                return Err(SeqError::IdentityMismatch {
                    context: StepContext::reg(
                        SeqStepKind::Identity,
                        self.cfg.ids[row],
                        RegId::ModelNumber,
                    ),
                    model,
                    expected,
                });
            }
        }
        Ok(())
    }

    /// Enter the provisioning sweep at the first cell at or after `from` that the
    /// table asks for, skipping to the voltage gate if there is none.
    ///
    /// A table that asks for nothing verifies nothing and reads nothing; whoever
    /// built it is what stands between that and the hardware.
    fn enter_provision(&mut self, now: Duration, from: usize) -> Result<(), SeqError> {
        let columns = PROVISION_REGS.len();
        let cell = (from..columns * ROW_COUNT).find(|flat| {
            !matches!(
                self.cfg.expected.at(flat / columns, flat % columns),
                Some(ProvisionExpect::Skip) | None
            )
        });
        match cell {
            Some(cursor) => {
                // The sweep may start mid-grid; a direct assignment would
                // skip the blanking of the fields the phase being left owns.
                self.enter(CommissionPhaseKind::Provision);
                self.seek(cursor);
            }
            None => self.enter_voltage(now)?,
        }
        Ok(())
    }

    /// Enter the supply gate, whose budget is measured from now.
    fn enter_voltage(&mut self, now: Duration) -> Result<(), SeqError> {
        self.enter(CommissionPhaseKind::Voltage);
        self.state.voltage_started = SyncTime::from_nanos(self.clock(now)?);
        self.state.voltage_waiting = false.into();
        Ok(())
    }

    /// `now` as the nanosecond count the state holds a moment in.
    ///
    /// # Errors
    ///
    /// [`SeqError::ClockOutOfRange`] past what the count reaches, which is around
    /// 292 years of uptime.
    fn clock(&self, now: Duration) -> Result<i64, SeqError> {
        duration_nanos(now).map_err(|_| SeqError::ClockOutOfRange {
            context: StepContext::servo(self.phase_step(), 0),
        })
    }

    /// How long the supply gate has been polling.
    fn waited(&self, now: Duration) -> Result<Duration, SeqError> {
        let started = duration_from_nanos(self.state.voltage_started.as_nanos()).map_err(|_| {
            SeqError::ClockOutOfRange {
                context: StepContext::servo(self.phase_step(), 0),
            }
        })?;
        Ok(now.saturating_sub(started))
    }

    fn absorb_provision(
        &mut self,
        now: Duration,
        cursor: usize,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        let columns = PROVISION_REGS.len();
        let (row, column) = (cursor / columns, cursor % columns);
        let observed = result.value(context)?;
        assert!(
            cells::record(
                &mut self.state.provisioned,
                servo_at(row),
                PROVISION_REGS[column],
                observed,
            ),
            "every provisioned register has a cell on every servo"
        );
        // Exact comparison: every provisioned register is an integer a person
        // reads off a data sheet, never an engineering-unit value with rounding
        // in it.
        if let Some(ProvisionExpect::Check(expected)) = self.cfg.expected.at(row, column)
            && observed != expected
        {
            return Err(SeqError::ProvisionMismatch {
                context,
                expected,
                observed,
            });
        }
        self.enter_provision(now, cursor + 1)
    }

    fn absorb_voltage(
        &mut self,
        now: Duration,
        cursor: usize,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        absorb_volts(cursor, &mut self.state.rail, context, result)?;
        if cursor + 1 < ROW_COUNT {
            self.seek(cursor + 1);
            self.state.voltage_waiting = false.into();
            return Ok(());
        }
        self.state.voltage_polls += 1;
        let limit = self.cfg.min_arm_voltage;
        let readings = cells::voltages_of(&self.state.rail);
        let Some((row, lowest)) = worst_below(&readings, limit) else {
            self.enter(CommissionPhaseKind::Health);
            return Ok(());
        };
        let waited = self.waited(now)?;
        if waited >= self.cfg.voltage_budget {
            return Err(SeqError::VoltageLow {
                context: StepContext::reg(
                    SeqStepKind::VoltageGate,
                    self.cfg.ids[row],
                    RegId::PresentInputVoltage,
                ),
                readings,
                lowest,
                limit,
                waited,
            });
        }
        // The sweep starts over, spaced out by the wait the next step hands back.
        self.state.cursor = 0;
        self.state.voltage_waiting = true.into();
        Ok(())
    }

    /// One servo's latched error byte, recorded and not judged.
    ///
    /// A latch refuses torque coming on, and that refusal lives in
    /// [`engage_gates`] where the rest of the torque-on gates are. Refusing here
    /// as well would stop a process from commissioning at all over a servo that
    /// is hurting nothing while the machine rests limp — and would give one gate
    /// two sites to be inconsistent at.
    fn absorb_health(
        &mut self,
        cursor: usize,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        absorb_health_bits(
            cursor,
            self.cfg.ids[cursor],
            &mut self.state.rail,
            context,
            result,
        )?;
        if cursor + 1 < ROW_COUNT {
            self.seek(cursor + 1);
        } else {
            self.enter(CommissionPhaseKind::GainsProfiles);
        }
        Ok(())
    }
}

phase_state!(
    CommissionSequencer,
    CommissionPhaseKind,
    CommissionPhaseKind::Failed
);

impl Sequencer for CommissionSequencer<'_> {
    fn pending(&self) -> &BusTxnWire {
        clockwork_rs::as_raw(&self.state.pending)
    }

    type Summary = CommissionSummary;

    fn next(&mut self, now: Duration, prior: Option<&BusResult>) -> SeqAction<CommissionSummary> {
        if let Err(error) = self.absorb(now, prior) {
            return SeqAction::Fail(self.fail(error));
        }
        self.emit(now)
    }

    fn step(&self) -> SeqStepKind {
        self.phase_step()
    }
}

/// An angle reading that is a number, or the refusal that says it is not.
///
/// A reading nobody can place decides nothing: it closes no linkage, sits inside
/// no window, and would become a goal that means nothing. Every sweep of
/// positions passes through here, so the refusal has one shape and names the
/// joint that carried it.
pub(crate) fn placeable(
    row: usize,
    context: StepContext,
    result: &BusResult,
) -> Result<f64, SeqError> {
    let angle = value::as_radians(result.value(context)?, context)?;
    if angle.is_finite() {
        return Ok(angle);
    }
    Err(SeqError::UnplaceableAngle {
        context,
        joint: ROWS[row],
        angle,
    })
}

/// How many times a sweep reads a position again after one came back
/// unplaceable, before it refuses.
///
/// Two, which is the difference between a corrupt frame and a servo that has
/// stopped reporting where it is. The cost of being wrong either way is small:
/// two extra reads on a bus that answers in tens of microseconds, against a
/// wake refused over one bad byte.
pub(crate) const PLACE_REREADS: u32 = 2;

/// A position reading, or the instruction to read it again.
///
/// The bounded retry [`placeable`] does not do on its own, because the counter
/// belongs to the sweep that is walking the servos and not to the judgement
/// about one reading. `Ok(None)` means the same register is read again;
/// `attempts` counts the re-reads already spent on this joint and is reset by
/// the reading that lands.
///
/// Only an unplaceable *number* is retried. Silence and a malformed answer come
/// back from the bus with its own retry policy already spent on them, and a
/// sweep re-reading those would be a second, quieter policy over the top.
pub(crate) fn placeable_or_again(
    row: usize,
    context: StepContext,
    result: &BusResult,
    attempts: &mut u32,
) -> Result<Option<f64>, SeqError> {
    match placeable(row, context, result) {
        Ok(angle) => {
            *attempts = 0;
            Ok(Some(angle))
        }
        Err(SeqError::UnplaceableAngle { .. }) if *attempts < PLACE_REREADS => {
            *attempts += 1;
            Ok(None)
        }
        Err(error) => Err(error),
    }
}

/// Confirm that a write landed, comparing against the value the request carried.
pub(crate) fn confirm_write(
    result: &BusResult,
    wrote: Value,
    context: StepContext,
) -> Result<(), SeqError> {
    if wrote.shape() == crate::value::ValueShape::None {
        // Only a write phase confirms a write, and a write transaction carries
        // its value; a read-back with nothing to compare against is the driver
        // answering a question nobody asked.
        return Err(SeqError::WrongAnswer {
            context,
            expected: AnswerShape::Written,
            observed: result.kind(),
        });
    }
    result.written(context, wrote)
}

/// How much of the machine one poll sweep asks about.
///
/// The positions move under a hand and are read every sweep; the rail and the
/// error bits change on the timescale of a power supply and are read on a slower
/// cadence the caller sets. Both end in a complete [`Posture`] — a sweep that
/// did not re-read the rail carries the last reading forward, so what the
/// torque-on gates see is never half a picture.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum PollCadence {
    /// Nine position reads.
    Positions,
    /// Nine position reads, then the supply and the error bits.
    PositionsAndRail,
}

/// How many servos a poll phase's sweep walks, or `None` for the ending and the
/// phase that names nothing.
fn poll_sweep(phase: PollPhaseKind) -> Option<usize> {
    match phase {
        PollPhaseKind::Position | PollPhaseKind::Voltage | PollPhaseKind::Health => Some(ROW_COUNT),
        PollPhaseKind::None | PollPhaseKind::Complete | PollPhaseKind::Failed => None,
    }
}

/// The name a refusal about a poll phase reads under.
fn poll_phase_name(phase: PollPhaseKind) -> &'static str {
    match phase {
        PollPhaseKind::None => "no",
        PollPhaseKind::Position => "position",
        PollPhaseKind::Voltage => "voltage",
        PollPhaseKind::Health => "health",
        PollPhaseKind::Complete => "complete",
        PollPhaseKind::Failed => "failed",
    }
}

/// The resting watch: one sweep of where the machine is, and what its rail
/// reads.
///
/// Runs against a limp machine and writes nothing, in either direction. It
/// exists so that engaging has nothing left to find out: the pose it plans from
/// and the readings its two gates judge are both already in hand, which is the
/// difference between a wake word raising the head in tens of milliseconds and
/// in several seconds.
///
/// Nothing here refuses anything about the machine's *state* — a head somebody
/// turned by hand is a measurement, not a fault. What does stop a sweep is the
/// bus failing to answer or answering with something that is not an angle,
/// because a posture nobody can place is not a posture.
pub struct PollSequencer<'a> {
    cfg: &'a ArmConfig,
    state: &'a mut PollSnap,
}

impl<'a> PollSequencer<'a> {
    /// A fresh sweep, in `slot`, carrying `rail` forward for a
    /// [`PollCadence::Positions`] sweep that does not re-read it.
    pub fn start(
        cfg: &'a ArmConfig,
        slot: &'a mut PollSnapWire,
        rail: Rail,
        cadence: PollCadence,
    ) -> Self {
        let state = slot.clear_valid();
        state.phase = PollPhaseKind::Position;
        state.rail_read = matches!(cadence, PollCadence::PositionsAndRail).into();
        cells::write_rail(&mut state.rail, &rail);
        Self { cfg, state }
    }

    /// The sweep `state` holds, run against `cfg`.
    ///
    /// # Errors
    ///
    /// [`ResumeError`] for a phase-and-cursor pairing no sequence of steps
    /// reaches.
    pub fn resume(cfg: &'a ArmConfig, state: &'a mut PollSnap) -> Result<Self, ResumeError> {
        let phase = state.phase;
        let name = poll_phase_name(phase);
        no_phase(name, phase == PollPhaseKind::None)?;
        checked_cursor(name, poll_sweep(phase), state.cursor)?;
        if phase == PollPhaseKind::Failed {
            verdict::read(&state.failure).map_err(ResumeError::Verdict)?;
        } else {
            no_stray_failure(name, state.failure.kind != SeqFailureKind::None)?;
        }
        Ok(Self { cfg, state })
    }

    /// The fields a phase owns, blanked as it is left.
    ///
    /// The re-read count belongs to the sweep that is reading positions: it
    /// counts the attempts spent on the joint at the cursor, so it means nothing
    /// once another phase is running.
    fn blank_phase_fields(&mut self) {
        self.state.rereads = 0;
    }

    /// Which phase a failure raised here is reported under.
    fn phase_step(&self) -> SeqStepKind {
        match self.state.phase {
            PollPhaseKind::Position | PollPhaseKind::Complete => SeqStepKind::PoseAndDatum,
            PollPhaseKind::Voltage => SeqStepKind::VoltageGate,
            PollPhaseKind::Health => SeqStepKind::Health,
            PollPhaseKind::Failed | PollPhaseKind::None => self.state.failure.step,
        }
    }

    /// What the sweep found, as the next engagement plans from it.
    fn posture(&self) -> Posture {
        Posture {
            present: joints::vector_of(&self.state.present),
            rail: cells::rail_of(&self.state.rail),
            rail_read: self.state.rail_read.into(),
        }
    }

    fn read(&mut self, row: usize, reg: RegId) {
        txn::set_read_reg(&mut self.state.pending, self.cfg.ids[row], reg);
    }

    fn emit(&mut self) -> SeqAction<Posture> {
        let cursor = self.cursor();
        match self.state.phase {
            PollPhaseKind::Position => self.read(cursor, RegId::PresentPosition),
            PollPhaseKind::Voltage => self.read(cursor, RegId::PresentInputVoltage),
            PollPhaseKind::Health => self.read(cursor, RegId::HardwareErrorStatus),
            PollPhaseKind::Complete => return SeqAction::Done(self.posture()),
            PollPhaseKind::Failed | PollPhaseKind::None => return SeqAction::Fail(self.verdict()),
        }
        SeqAction::Transact
    }

    fn absorb(&mut self, prior: Option<&BusResult>) -> Result<(), SeqError> {
        if !txn::held(&self.state.pending) {
            return Ok(());
        }
        let (context, _) = txn::fields(&self.state.pending, self.phase_step())?;
        txn::set_none(&mut self.state.pending);
        let Some(result) = prior else {
            return Err(SeqError::NoAnswer { context });
        };
        let cursor = self.cursor();
        match self.state.phase {
            PollPhaseKind::Position => {
                let placed = placeable_or_again(cursor, context, result, &mut self.state.rereads);
                let Some(angle) = placed? else {
                    // The phase is left where it is, so the next action reads
                    // the same register again.
                    return Ok(());
                };
                joints::set_angle(&mut self.state.present, ROWS[cursor], angle);
                if cursor + 1 < ROW_COUNT {
                    self.seek(cursor + 1);
                } else if self.state.rail_read.into() {
                    self.enter(PollPhaseKind::Voltage);
                } else {
                    self.enter(PollPhaseKind::Complete);
                }
            }
            PollPhaseKind::Voltage => {
                absorb_volts(cursor, &mut self.state.rail, context, result)?;
                if cursor + 1 < ROW_COUNT {
                    self.seek(cursor + 1);
                } else {
                    self.enter(PollPhaseKind::Health);
                }
            }
            PollPhaseKind::Health => {
                absorb_health_bits(
                    cursor,
                    self.cfg.ids[cursor],
                    &mut self.state.rail,
                    context,
                    result,
                )?;
                if cursor + 1 < ROW_COUNT {
                    self.seek(cursor + 1);
                } else {
                    self.enter(PollPhaseKind::Complete);
                }
            }
            PollPhaseKind::Complete | PollPhaseKind::Failed | PollPhaseKind::None => {}
        }
        Ok(())
    }
}

phase_state!(PollSequencer, PollPhaseKind, PollPhaseKind::Failed);

impl Sequencer for PollSequencer<'_> {
    fn pending(&self) -> &BusTxnWire {
        clockwork_rs::as_raw(&self.state.pending)
    }

    type Summary = Posture;

    fn next(&mut self, _now: Duration, prior: Option<&BusResult>) -> SeqAction<Posture> {
        if let Err(error) = self.absorb(prior) {
            return SeqAction::Fail(self.fail(error));
        }
        self.emit()
    }

    fn step(&self) -> SeqStepKind {
        self.phase_step()
    }
}

/// How many elements an engagement phase's sweep walks, or `None` for the two
/// endings and the phase that names nothing, which have no cursor.
fn engage_sweep(phase: EngagePhaseKind) -> Option<usize> {
    match phase {
        EngagePhaseKind::Pin | EngagePhaseKind::Enable | EngagePhaseKind::Settle => Some(ROW_COUNT),
        EngagePhaseKind::None | EngagePhaseKind::Complete | EngagePhaseKind::Failed => None,
    }
}

/// The name a refusal about an engagement phase reads under.
fn engage_phase_name(phase: EngagePhaseKind) -> &'static str {
    match phase {
        EngagePhaseKind::None => "no",
        EngagePhaseKind::Pin => "pin",
        EngagePhaseKind::Enable => "enable",
        EngagePhaseKind::Settle => "settle",
        EngagePhaseKind::Complete => "complete",
        EngagePhaseKind::Failed => "failed",
    }
}

/// Whether `phase` writes to the joint its cursor stands on.
///
/// The read sweep and the two endings write nothing: the settle reads all nine
/// whatever the health gate said, and a finished or failed sequence has no
/// cursor at all. Only a write sweep steps past a degraded joint.
fn engage_writes(phase: EngagePhaseKind) -> bool {
    match phase {
        EngagePhaseKind::Pin | EngagePhaseKind::Enable => true,
        EngagePhaseKind::Settle
        | EngagePhaseKind::Complete
        | EngagePhaseKind::Failed
        | EngagePhaseKind::None => false,
    }
}

/// Taking hold of the machine, in twenty-seven transactions.
///
/// Three sweeps of nine, and nothing else: each goal register written to the
/// position its joint was just measured at, each servo's torque enabled — which
/// holds the joint where it stands — and each position read back to seed the
/// trajectory the next move starts from.
///
/// **The goal write comes first and is belt-and-braces.** With torque off this
/// platform's servos report Goal Position as their present position and keep
/// nothing written to it, so the write ordinarily lands nowhere and the register
/// the enable picks up is the mirror it already was. It is issued anyway, and
/// its answer is deliberately not judged: a firmware that *does* keep a stale
/// goal would slam the joint at the enable, and the cost of guarding against
/// that is one sweep of writes nobody has to believe in. A mismatch there is the
/// register mirroring, which is the expected case — treating it as a refusal
/// would gate torque-on behind the very behaviour that makes torque-on safe.
///
/// **The measured pose is never refused.** Where the machine physically stands
/// is the one fact nothing can argue with; a refusal would leave it standing
/// there limp with nothing but a hand able to move it. The gates that do apply —
/// the rail and the latched error bits — are checked in [`engage_gates`] before
/// a single transaction is issued, so a refusal costs the bus nothing and leaves
/// the machine exactly as it was.
///
/// **A degraded antenna costs three of the twenty-seven.** The health gate hands
/// back the joints it left out of service, and those take neither the goal write
/// nor the enable — they stay limp for the whole engagement — while the settle
/// sweep still reads all nine, because where a limp joint sits is a measurement
/// like any other and the record the tick starts from wants it.
pub struct EngageSequencer<'a> {
    cfg: &'a ArmConfig,
    geom: &'a HeadGeometry,
    fk: FkOptions,
    state: &'a mut EngageSnap,
}

impl<'a> EngageSequencer<'a> {
    /// A fresh engagement, in `slot`, taking hold of the machine `posture`
    /// describes.
    ///
    /// Everything that can refuse is settled here, before any transaction: the
    /// two torque-on gates, an angle nobody can place, and a set of angles that
    /// closes no linkage. A sequence started over a refusal fails on its first
    /// action having put nothing on the wire.
    ///
    /// The geometry and the solver options come separately because the records
    /// this produces are solved poses, not angles, and both have to be the ones
    /// the tick uses — a record solved against another geometry would hand the
    /// first trajectory a start the machine is not at.
    ///
    /// The slot is cleared to the schema's declared initial state before
    /// anything is written into it, so a slot an earlier engagement left its
    /// records in describes this one and nothing else.
    pub fn start(
        cfg: &'a ArmConfig,
        geom: &'a HeadGeometry,
        fk: &FkOptions,
        slot: &'a mut EngageSnapWire,
        posture: &Posture,
    ) -> Self {
        let state = slot.clear_valid();
        state.phase = EngagePhaseKind::Pin;
        joints::write_vector(&mut state.measured, &posture.present);
        joints::write_vector(&mut state.post_enable, &posture.present);
        joints::write_vector(&mut state.pins.pinned, &posture.present);
        let mut seq = Self {
            cfg,
            geom,
            fk: *fk,
            state,
        };
        if let Err(error) = seq.prepare(posture) {
            seq.fail(error);
        }
        seq
    }

    /// The engagement `state` holds, run against `cfg` with `geom` and `fk`.
    ///
    /// The geometry and the solver options have to be the ones the tick uses,
    /// for the same reason [`EngageSequencer::start`] takes them: the records
    /// this produces are solved poses.
    ///
    /// Everything the remaining sequence hands to the bus or plans writes from
    /// is read here, before a sequencer that can write exists: the two records,
    /// the floats inside them, and the pinned goals — which the goal sweep
    /// writes to nine servos with nothing between the slot bytes and the wire,
    /// generated validation covering enums and counts and never a float. A
    /// refused resume means the slot is not trusted; the caller's answer is the
    /// immediate release, which is always constructible, consults nothing from
    /// this slot, and reaches the minimum risk condition without it.
    ///
    /// The failed phase is exempt from all of it. A prepare-time refusal writes
    /// that phase with no resting record ever written, and — after a pin
    /// refusal — with the raw measured angles still sitting in the pins, so a
    /// failed engagement carrying neither is a state this sequencer produces.
    /// It hands nothing to the bus; its verdict reading back is its whole gate.
    ///
    /// # Errors
    ///
    /// [`ResumeError`] for a phase-and-cursor pairing no sequence of steps
    /// reaches, an engagement that says it finished with no record of where it
    /// left the machine, a record carried before it could have been solved, a
    /// record that is absent or is no pose in a phase that plans from it, a
    /// number in one that is not a number, or a pinned goal outside its leg's
    /// travel window.
    pub fn resume(
        cfg: &'a ArmConfig,
        geom: &'a HeadGeometry,
        fk: &FkOptions,
        state: &'a mut EngageSnap,
    ) -> Result<Self, ResumeError> {
        let phase = state.phase;
        let name = engage_phase_name(phase);
        no_phase(name, phase == EngagePhaseKind::None)?;
        checked_cursor(name, engage_sweep(phase), state.cursor)?;
        let armed = bool::from(state.armed.present);
        if armed && phase != EngagePhaseKind::Complete {
            return Err(ResumeError::RecordWithoutCompletePhase { phase: name });
        }
        if phase == EngagePhaseKind::Complete && !armed {
            return Err(ResumeError::CompleteWithoutRecord);
        }
        if phase == EngagePhaseKind::Failed {
            verdict::read(&state.failure).map_err(ResumeError::Verdict)?;
        } else {
            no_stray_failure(name, state.failure.kind != SeqFailureKind::None)?;
            // Every phase left is one a successful `prepare` precedes, so every
            // one of them has the resting record and the pins.
            checked_record("rest", name, &state.rest)?;
            if armed {
                checked_record("armed", name, &state.armed)?;
            }
            checked_pins(cfg, &state.pins)?;
        }
        Ok(Self {
            cfg,
            geom,
            fk: *fk,
            state,
        })
    }

    /// Whether an enable write has gone out, so the machine may be holding
    /// torque with nobody controlling it.
    ///
    /// What a caller does with a *failed* engage turns on this. A sequence that
    /// refused before its first enable left the machine limp and there is
    /// nothing to undo; one that stopped after it has to be taken to the
    /// minimum risk condition, which means writing torque off. Answered from
    /// the enable that was issued rather than from the acknowledgement it got,
    /// because a write nothing answered is exactly the case where the servo may
    /// have taken it.
    #[must_use]
    pub fn torque_written(&self) -> bool {
        self.state.torque_written.into()
    }

    /// The gates, the pins and the resting record, all before the first write.
    fn prepare(&mut self, posture: &Posture) -> Result<(), SeqError> {
        self.state.degraded = engage_gates(self.cfg, &posture.rail)?;
        let pins = pin_goals(self.cfg, &posture.present)?;
        record::write_pins(&mut self.state.pins, &pins);
        // Failure is not a solver problem to retry with a perturbed seed: the
        // angles are what nine servos reported, and angles that place no pose
        // say the model and the machine disagree.
        let rest = ArmRecord::solve(self.geom, &self.fk, &posture.present, &rest_pose_seeds())
            .map_err(|cause| SeqError::RestPoseImplausible {
                context: self.pose_context(),
                cause,
            })?;
        record::write(&mut self.state.rest, &rest);
        Ok(())
    }

    /// Where a solve failure or an unreadable record is reported from.
    ///
    /// Named against the first crank: the failure belongs to the six of them
    /// together, and a context names one servo.
    fn pose_context(&self) -> StepContext {
        StepContext::reg(
            SeqStepKind::PinAndEnable,
            self.cfg.ids[1],
            RegId::PresentPosition,
        )
    }

    /// The fields a phase owns, blanked as it is left.
    ///
    /// The re-read count belongs to the settle sweep: it counts the attempts
    /// spent on the joint at the cursor, so it means nothing once another phase
    /// is running.
    fn blank_phase_fields(&mut self) {
        self.state.rereads = 0;
    }

    /// Which phase a failure raised here is reported under.
    fn phase_step(&self) -> SeqStepKind {
        match self.state.phase {
            EngagePhaseKind::Pin
            | EngagePhaseKind::Enable
            | EngagePhaseKind::Settle
            | EngagePhaseKind::Complete => SeqStepKind::PinAndEnable,
            EngagePhaseKind::Failed | EngagePhaseKind::None => self.state.failure.step,
        }
    }

    /// Where a write sweep stands once it is done with the joint at its cursor.
    ///
    /// The order of the write sweeps, in one place: the goal writes run the nine
    /// and hand over to the enables, the enables run the nine and hand over to
    /// the settle reads. Two callers step it — the walk itself, as each answer
    /// comes back, and the skip that steps past a joint nothing is written to —
    /// and a second copy of the order is a copy that can be left behind by a
    /// sweep added to only one of them.
    ///
    /// A cursor never reaches [`ROW_COUNT`]: the last joint of a sweep hands
    /// over rather than counting past the end.
    fn advance(&mut self) {
        let cursor = self.cursor();
        match self.state.phase {
            EngagePhaseKind::Pin if cursor + 1 < ROW_COUNT => self.seek(cursor + 1),
            EngagePhaseKind::Pin => self.enter(EngagePhaseKind::Enable),
            EngagePhaseKind::Enable if cursor + 1 < ROW_COUNT => self.seek(cursor + 1),
            EngagePhaseKind::Enable => self.enter(EngagePhaseKind::Settle),
            EngagePhaseKind::Settle
            | EngagePhaseKind::Complete
            | EngagePhaseKind::Failed
            | EngagePhaseKind::None => {}
        }
    }

    /// Step a write sweep's cursor past the joints nothing is written to.
    ///
    /// A degraded joint is written neither a goal nor an enable: it is out of
    /// service for this engagement, and a goal register belonging to a servo
    /// that will never hold torque is a register nothing reads. Stepping over
    /// it here is what makes "never commanded" true on the wire rather than
    /// only in the tick.
    fn skip_degraded(&mut self) {
        while engage_writes(self.state.phase)
            && flags::contains(self.state.degraded, ROWS[self.cursor()])
        {
            self.advance();
        }
    }

    /// The angle the slot's vector holds for the joint at bus row `row`.
    ///
    /// Every cursor here is bounded by [`ROW_COUNT`] by the phase transitions,
    /// so the row names a servo wherever this is called.
    fn angle_at(vector: &joints::Joints, row: usize) -> f64 {
        joint_ref(row)
            .and_then(|joint| joints::angle_of(vector, joint))
            .expect("bus rows are bounded by the phase cursors")
    }

    /// A record the slot holds, or the refusal that says why it holds none.
    ///
    /// The two refusals are kept apart because their causes are unrelated: a
    /// step that needs a record the slot never had is a phase reached out of
    /// order, and a record whose quaternion is no rotation is a slot written by
    /// something else. Collapsing them would leave a post-mortem unable to tell
    /// which happened, and this crate has no logger to say it anywhere else.
    ///
    /// # Errors
    ///
    /// [`SeqError::RecordAbsent`] where the slot holds no record at all, and
    /// [`SeqError::RecordUnreadable`] where the one it holds is no pose. The
    /// engagement plans its next writes from these records, so it stops rather
    /// than planning from a pose nobody solved.
    fn record(&self, slot: &record::ArmRecordSnap) -> Result<ArmRecord, SeqError> {
        let context = self.pose_context();
        match record::read(slot) {
            Ok(Some(record)) => Ok(record),
            Ok(None) => Err(SeqError::RecordAbsent { context }),
            Err(_) => Err(SeqError::RecordUnreadable { context }),
        }
    }

    /// What the engagement wrote and where it left the machine standing.
    ///
    /// # Errors
    ///
    /// [`SeqError::RecordAbsent`] or [`SeqError::RecordUnreadable`], as
    /// [`Self::record`] says.
    fn summary(&self) -> Result<EngageSummary, SeqError> {
        Ok(EngageSummary {
            rest: self.record(&self.state.rest)?,
            armed: self.record(&self.state.armed)?,
            pins: record::pins_of(&self.state.pins),
            post_enable_shift: joints::rows_of(&self.state.post_enable_shift),
            degraded: self.state.degraded,
        })
    }

    /// Ask the servo at bus row `row` for `reg`.
    fn read(&mut self, row: usize, reg: RegId) {
        txn::set_read_reg(&mut self.state.pending, self.cfg.ids[row], reg);
    }

    /// Write `value` to `reg` on the servo at bus row `row`, and read it back.
    fn write(&mut self, row: usize, reg: RegId, value: Value) {
        txn::set_write_reg_verified(&mut self.state.pending, self.cfg.ids[row], reg, value);
    }

    fn emit(&mut self) -> SeqAction<EngageSummary> {
        self.skip_degraded();
        let cursor = self.cursor();
        match self.state.phase {
            EngagePhaseKind::Pin => {
                let goal = value::radians(Self::angle_at(&self.state.pins.pinned, cursor));
                self.write(cursor, RegId::GoalPosition, goal);
            }
            EngagePhaseKind::Enable => {
                self.state.torque_written = true.into();
                self.write(cursor, RegId::TorqueEnable, value::u8(1));
            }
            EngagePhaseKind::Settle => self.read(cursor, RegId::PresentPosition),
            EngagePhaseKind::Complete => {
                return match self.summary() {
                    Ok(summary) => SeqAction::Done(summary),
                    Err(error) => {
                        // Latched, so a sequence stepped again stops the same
                        // way rather than reading the record a second time.
                        SeqAction::Fail(self.fail(error))
                    }
                };
            }
            EngagePhaseKind::Failed | EngagePhaseKind::None => {
                return SeqAction::Fail(self.verdict());
            }
        }
        SeqAction::Transact
    }

    fn absorb(&mut self, prior: Option<&BusResult>) -> Result<(), SeqError> {
        if !txn::held(&self.state.pending) {
            return Ok(());
        }
        let (context, wrote) = txn::fields(&self.state.pending, self.phase_step())?;
        txn::set_none(&mut self.state.pending);
        let cursor = self.cursor();
        match self.state.phase {
            EngagePhaseKind::Pin => {
                // Whatever came back — the mirrored present, silence, a refusal
                // — the walk carries on. See the type's docs: this sweep is
                // insurance against a stale goal register, not a check on one.
                self.advance();
                Ok(())
            }
            EngagePhaseKind::Enable => {
                let Some(result) = prior else {
                    return Err(SeqError::NoAnswer { context });
                };
                confirm_write(result, wrote, context)?;
                self.advance();
                Ok(())
            }
            EngagePhaseKind::Settle => {
                let Some(result) = prior else {
                    return Err(SeqError::NoAnswer { context });
                };
                let placed = placeable_or_again(cursor, context, result, &mut self.state.rereads);
                let Some(angle) = placed? else {
                    // The phase is left where it is, so the next action reads
                    // the same register again.
                    return Ok(());
                };
                let shift = angle - Self::angle_at(&self.state.measured, cursor);
                joints::set_angle(&mut self.state.post_enable, ROWS[cursor], angle);
                joints::set_angle(&mut self.state.post_enable_shift, ROWS[cursor], shift);
                if cursor + 1 < ROW_COUNT {
                    self.seek(cursor + 1);
                    return Ok(());
                }
                let seeds = armed_pose_seeds(self.record(&self.state.rest)?.head_pose_body);
                let post_enable = joints::vector_of(&self.state.post_enable);
                let armed = ArmRecord::solve(self.geom, &self.fk, &post_enable, &seeds).map_err(
                    |cause| SeqError::PinnedPoseUnsolvable {
                        context: self.pose_context(),
                        cause,
                    },
                )?;
                record::write(&mut self.state.armed, &armed);
                self.enter(EngagePhaseKind::Complete);
                Ok(())
            }
            EngagePhaseKind::Complete | EngagePhaseKind::Failed | EngagePhaseKind::None => Ok(()),
        }
    }
}

phase_state!(EngageSequencer, EngagePhaseKind, EngagePhaseKind::Failed);

impl Sequencer for EngageSequencer<'_> {
    fn pending(&self) -> &BusTxnWire {
        clockwork_rs::as_raw(&self.state.pending)
    }

    type Summary = EngageSummary;

    fn next(&mut self, _now: Duration, prior: Option<&BusResult>) -> SeqAction<EngageSummary> {
        if let Err(error) = self.absorb(prior) {
            return SeqAction::Fail(self.fail(error));
        }
        self.emit()
    }

    fn step(&self) -> SeqStepKind {
        self.phase_step()
    }
}

#[cfg(test)]
mod tests {
    use core::f64::consts::PI;

    use super::*;
    use crate::testutil::{Asked, ScriptedBus, asked};
    use crate::txn::AuxOpKind;
    use nalgebra::{Translation3, UnitQuaternion};
    use reachy_kin::{
        EnvelopeConfig, LegAngles, inverse_kinematics, min_pose_margin, rest_head_pose,
        stow_head_pose,
    };

    /// Arming's configuration against the envelope's own fences, drawn in by
    /// the inset the servo-side windows really sit at.
    fn config() -> ArmConfig {
        crate::testutil::arm_config(&EnvelopeConfig::default())
    }

    /// The nine angles a pose puts the machine at, yaw and antennas at zero.
    fn joints_at(pose: &Isometry3<f64>) -> JointVector {
        let geom = HeadGeometry::default();
        let mut angles = LegAngles([0.0; 6]);
        inverse_kinematics(&geom, pose, &mut angles).expect("the pose is reachable");
        JointVector {
            body_yaw: 0.0,
            legs: angles.0,
            antennas: [0.0; 2],
        }
    }

    /// A leg already inside its window is pinned exactly where it is, bit for
    /// bit, and nothing is recorded as pulled.
    #[test]
    fn a_joint_inside_its_window_is_pinned_where_it_is() {
        let cfg = config();
        let present = joints_at(&reachy_kin::neutral_head_pose());
        let outcome = pin_goals(&cfg, &present).expect("nothing to pull");
        assert_eq!(outcome.pinned, present);
        assert_eq!(outcome.pull_in, [0.0; 6]);
        assert_eq!(outcome.worst_pull_in(), 0.0);
    }

    /// The recorded resting pose is the case the pin exists for: four legs sit
    /// outside their travel windows, and pinning seats every one of them inside
    /// while recording how far it pulled.
    #[test]
    fn the_resting_pose_is_pulled_into_every_window() {
        let cfg = config();
        let present = joints_at(&rest_head_pose());
        let outcome = pin_goals(&cfg, &present).expect("the pulls are inside the gate");

        for (leg, (angle, (low, high))) in
            outcome.pinned.legs.iter().zip(cfg.leg_windows).enumerate()
        {
            assert!(
                *angle >= low && *angle <= high,
                "leg {} pinned at {:.4}° outside [{:.4}°, {:.4}°]",
                leg + 1,
                angle.to_degrees(),
                low.to_degrees(),
                high.to_degrees()
            );
        }

        // Four legs pulled, two untouched, worst around ten and a half degrees —
        // inside the gate, and the reason the gate is set where it is.
        let pulled: Vec<usize> = outcome
            .pull_in
            .iter()
            .enumerate()
            .filter(|(_, pull)| **pull > 0.0)
            .map(|(leg, _)| leg + 1)
            .collect();
        assert_eq!(pulled, vec![1, 2, 5, 6]);
        let degrees: Vec<f64> = outcome
            .pull_in
            .iter()
            .map(|pull| (pull.to_degrees() * 1e3).round() / 1e3)
            .collect();
        assert_eq!(degrees, vec![7.543, 2.454, 0.0, 0.0, 0.767, 10.575]);
        assert!((outcome.worst_pull_in().to_degrees() - 10.575).abs() < 1e-3);
        // The antennas were at zero and stayed there: the leg pull touches
        // nothing else.
        assert_eq!(outcome.pinned.antennas, present.antennas);
    }

    /// The model angle a servo count denotes: a whole turn spread over 4096
    /// counts, zero at the middle of the range. Spelled out here because nothing
    /// in this crate knows what a count is, and the antenna readings below are
    /// what a real machine reported.
    fn rad_from_counts(counts: i32) -> f64 {
        core::f64::consts::TAU * f64::from(counts) / 4096.0 - PI
    }

    /// The antennas wherever a machine is found with them: parked past the half
    /// turn by the platform's own shutdown, spun several turns out by a hand, or
    /// standing anywhere in between. Every one of them is pinned exactly where
    /// it is measured, and nothing about the legs changes.
    ///
    /// The first two rows are readings real machines gave: 38 and 4051 counts at
    /// rest on run 1, and -202 counts on run 4. There is no bound to bring any of
    /// them inside and no pull to record, so the goal written is the reading.
    #[test]
    fn the_antennas_are_pinned_where_they_are_found() {
        let cfg = config();
        let rest = joints_at(&rest_head_pose());
        let plain = pin_goals(&cfg, &rest).expect("the pulls are inside the gate");

        for antennas in [
            [rad_from_counts(38), rad_from_counts(4051)],
            [rad_from_counts(-202), rad_from_counts(4051)],
            [3.6, -4.2],
            [0.2, -0.15],
            [-3.05, 3.05],
            [
                10.0 * core::f64::consts::TAU,
                -10.0 * core::f64::consts::TAU,
            ],
        ] {
            let mut present = rest;
            present.antennas = antennas;
            let outcome = pin_goals(&cfg, &present).expect("no gate stands in front of an antenna");
            assert_eq!(outcome.pinned.antennas, antennas, "{antennas:?}");

            // The legs are pinned exactly as they are without any of this: the
            // leg gate never sees an antenna.
            assert_eq!(outcome.pinned.legs, plain.pinned.legs, "{antennas:?}");
            assert_eq!(outcome.pull_in, plain.pull_in, "{antennas:?}");
            assert_eq!(
                outcome.worst_pull_in(),
                plain.worst_pull_in(),
                "{antennas:?}"
            );
        }
    }

    /// However far a pin has to pull, it pulls: the distance is recorded and
    /// nothing judges it. A machine found in a pose nobody predicted is a
    /// machine to take hold of and move, and refusing it leaves the head exactly
    /// where it was with nothing but a hand able to shift it.
    #[test]
    fn a_pull_of_any_size_is_recorded_and_never_refused() {
        let narrow = 1.0_f64.to_radians();
        let cfg = ArmConfig {
            leg_windows: [(-narrow, narrow); 6],
            ..config()
        };
        let present = joints_at(&rest_head_pose());
        let outcome = pin_goals(&cfg, &present).expect("nothing here refuses");

        for (leg, angle) in outcome.pinned.legs.into_iter().enumerate() {
            assert!(
                angle.abs() <= narrow + 1e-12,
                "leg {} pinned at {:.3}°",
                leg + 1,
                angle.to_degrees()
            );
            assert!(
                (outcome.pull_in[leg] - (angle - present.legs[leg]).abs()).abs() < 1e-12,
                "the recorded pull is the distance the write moves it"
            );
        }
        assert!(
            outcome.worst_pull_in().to_degrees() > 30.0,
            "a pull far past anything the old gate admitted: {:.3}°",
            outcome.worst_pull_in().to_degrees()
        );
    }

    /// An angle nobody can place is refused before anything is pinned, on
    /// whichever of the nine it arrives on — including the three the windows say
    /// nothing about, which is exactly where a clamp would have hidden it.
    #[test]
    fn an_unplaceable_angle_refuses_on_every_joint() {
        let cfg = config();
        let good = joints_at(&reachy_kin::neutral_head_pose());
        for (row, joint) in ROWS.into_iter().enumerate() {
            for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut present = good;
                match row {
                    0 => present.body_yaw = bad,
                    1..=6 => present.legs[row - 1] = bad,
                    _ => present.antennas[row - 7] = bad,
                }
                let error = pin_goals(&cfg, &present).expect_err("an unplaceable angle refuses");
                let SeqError::UnplaceableAngle {
                    context,
                    joint: named,
                    ..
                } = error
                else {
                    panic!("expected an unplaceable-angle refusal, got {error}");
                };
                assert_eq!(named, joint);
                assert_eq!(context.id, SERVO_IDS[row]);
                assert_eq!(context.reg, Some(RegId::PresentPosition));
            }
        }
        assert_eq!(
            SeqError::UnplaceableAngle {
                context: StepContext::reg(SeqStepKind::PinAndEnable, 17, RegId::PresentPosition),
                joint: JointRef::AntennaRight,
                angle: f64::NAN,
            }
            .to_string(),
            "pin and enable of servo 17, present position: right antenna measured NaN rad, \
             which is not an angle"
        );
    }

    /// A basis nobody can place is refused too, and named against the register
    /// it came from rather than against the positions.
    ///
    /// The sequence cannot reach this — every reading it puts in a basis has
    /// been through the non-finite guard already — but the two-vector entry
    /// point is public, and a caller handed a goal register full of nothing gets
    /// a refusal that says which of the two readings was bad.
    #[test]
    fn an_unplaceable_basis_is_refused_against_the_goal_register() {
        let cfg = config();
        let measured = joints_at(&reachy_kin::neutral_head_pose());
        for (row, joint) in ROWS.into_iter().enumerate() {
            let mut basis = measured;
            match row {
                0 => basis.body_yaw = f64::NAN,
                1..=6 => basis.legs[row - 1] = f64::NEG_INFINITY,
                _ => basis.antennas[row - 7] = f64::INFINITY,
            }
            let error =
                pin_goals_from(&cfg, &basis, &measured).expect_err("an unplaceable basis refuses");
            let SeqError::UnplaceableAngle {
                context,
                joint: named,
                ..
            } = error
            else {
                panic!("expected an unplaceable-angle refusal, got {error}");
            };
            assert_eq!(named, joint);
            assert_eq!(context.id, SERVO_IDS[row]);
            assert_eq!(context.reg, Some(RegId::GoalPosition));
        }
    }

    /// The armed record is the pose the pinned angles hold, not the pose that
    /// was measured — and that pose's own solution is inside every travel
    /// window, which is the property every trajectory afterwards depends on.
    #[test]
    fn the_armed_record_is_the_pose_the_pins_hold() {
        let cfg = config();
        let geom = HeadGeometry::default();
        let opts = FkOptions::default();
        let rest = rest_head_pose();
        let present = joints_at(&rest);
        let outcome = pin_goals(&cfg, &present).expect("the pulls are inside the gate");

        let record = ArmRecord::solve(&geom, &opts, &outcome.pinned, &[rest])
            .expect("the pinned angles close the linkage");
        assert_eq!(record.joints, outcome.pinned);

        let moved = (record.head_pose_body.translation.vector - rest.translation.vector).norm();
        assert!(moved > 1e-4, "the pinned pose is the rest pose: {moved} m");

        let mut margins = [0.0; 6];
        pose_margins(&geom, &record.head_pose_body, &mut margins);
        assert_eq!(record.margins, margins);
        assert_eq!(
            record.min_margin,
            min_pose_margin(&geom, &record.head_pose_body)
        );
        // Still far below the clearance floor: the pin buys windows, not
        // clearance, and the margin baseline is what carries the first lift.
        assert!(record.min_margin > 0.0);
        assert!(record.min_margin < EnvelopeConfig::default().min_toggle_margin);

        // The round trip: the pose the pins hold solves back to angles inside
        // every window the command path checks. Against the servo-side fence the
        // pinned legs sit exactly *on* their bound, and the solve reproduces that
        // to within a float's width either side — so the fence the tick actually
        // enforces, a hundredth of a degree looser, is the one this asserts.
        let mut angles = LegAngles([0.0; 6]);
        inverse_kinematics(&geom, &record.head_pose_body, &mut angles)
            .expect("the held pose is reachable");
        let windows = EnvelopeConfig::default().crank_windows;
        for (leg, (angle, (low, high))) in angles.0.iter().zip(windows).enumerate() {
            assert!(
                *angle >= low && *angle <= high,
                "leg {} solves to {:.4}° outside [{:.4}°, {:.4}°]",
                leg + 1,
                angle.to_degrees(),
                low.to_degrees(),
                high.to_degrees()
            );
        }
    }

    /// Seeds are tried in order and the first that works wins, so a caller with
    /// two candidate resting poses needs no policy of its own.
    #[test]
    fn the_first_seed_that_solves_is_the_one_used() {
        let geom = HeadGeometry::default();
        let opts = FkOptions::default();
        let rest = rest_head_pose();
        let joints = joints_at(&rest);

        let hopeless =
            Isometry3::from_parts(Translation3::new(0.0, 0.0, 5.0), UnitQuaternion::identity());
        assert!(
            ArmRecord::solve(&geom, &opts, &joints, &[hopeless]).is_err(),
            "the hopeless seed has to fail for this test to say anything"
        );

        let from_second = ArmRecord::solve(&geom, &opts, &joints, &[hopeless, rest])
            .expect("the second seed solves");
        let from_first =
            ArmRecord::solve(&geom, &opts, &joints, &[rest]).expect("the first seed solves");
        assert_eq!(from_second, from_first);
    }

    /// The resting pose solves from the stow candidate too — the two open
    /// candidates are what arming seeds with — and both reach the same record.
    #[test]
    fn either_candidate_seed_reaches_the_resting_record() {
        let geom = HeadGeometry::default();
        let opts = FkOptions::default();
        let joints = joints_at(&rest_head_pose());
        let from_stow = ArmRecord::solve(&geom, &opts, &joints, &[stow_head_pose()])
            .expect("the stow candidate seeds it");
        let from_rest = ArmRecord::solve(&geom, &opts, &joints, &[rest_head_pose()])
            .expect("the rest candidate seeds it");
        let gap = (from_stow.head_pose_body.translation.vector
            - from_rest.head_pose_body.translation.vector)
            .norm();
        assert!(gap < 1e-9, "the two seeds disagree by {gap} m");
    }

    /// Neutral is one of the poses the resting solve is seeded from.
    ///
    /// Structural, and deliberately so: at the shipped geometry the solver
    /// reaches neutral from the resting seeds anyway, so no behavioural test can
    /// tell the seed's presence from its absence. What the seed insures against
    /// is a basin of attraction nobody has bounded, on the pose every re-arm in
    /// a session actually starts from, and that is a property of the list rather
    /// than of an outcome.
    #[test]
    fn the_pose_a_command_leaves_the_machine_holding_is_a_seed() {
        let seeds = rest_pose_seeds();
        let neutral = reachy_kin::neutral_head_pose();
        assert!(
            seeds.contains(&neutral),
            "the resting solve is seeded from {seeds:?}, none of them neutral"
        );
        // And the seed is not one of the resting candidates under another name:
        // the pose a lifted machine holds is a long way from either.
        for resting in [stow_head_pose(), rest_head_pose()] {
            let apart = (resting.translation.vector - neutral.translation.vector).norm();
            assert!(apart > 0.04, "the resting seed is {apart} m from neutral");
        }

        // The post-enable solve is the same list with the resting record in
        // front of it. A pose the resting solve reaches and the armed one does
        // not would be a machine that engages and then refuses its own
        // read-back, so the two lists cannot be maintained apart.
        let measured = rest_head_pose();
        let armed = armed_pose_seeds(measured);
        assert_eq!(armed[0], measured);
        assert_eq!(armed[1..], seeds);
    }

    /// A machine standing where a command left it arms, and the resting record
    /// lands where it is standing.
    ///
    /// Every command in this bench re-arms, so the pose phase six solves is
    /// whatever the previous command left the machine holding — neutral, after
    /// the lift, which is 44 mm and a pitch away from either resting candidate.
    /// Driven through the sequence, so it is the solve engaging really runs that
    /// lands there; which of the seeds carried it is not observable here and the
    /// test above is what pins the list.
    #[test]
    fn the_pose_a_command_leaves_the_machine_at_is_solved_when_engaging() {
        let neutral = reachy_kin::neutral_head_pose();
        let mut machine = bus();
        machine.present = joints_at(&neutral);
        machine.present.body_yaw = 0.35;
        machine.present.antennas = [0.20, -0.15];

        let summary = drive(&provisioned_config(), &mut machine)
            .expect("a machine standing at neutral engages");
        let gap = (summary.engage.rest.head_pose_body.translation.vector
            - neutral.translation.vector)
            .norm();
        assert!(gap < 1e-6, "the resting record is {gap} m from neutral");
    }

    /// No seed working is a refusal carrying the last solver failure, not a
    /// pose. Six angles no rigid head can hold at once are the case: five of
    /// them place the head, and the sixth then has a rod length it cannot have.
    #[test]
    fn angles_that_close_no_loop_have_no_record() {
        let geom = HeadGeometry::default();
        let opts = FkOptions::default();
        let mut joints = joints_at(&rest_head_pose());
        joints.legs[5] += 1.0;
        let error = ArmRecord::solve(&geom, &opts, &joints, &[rest_head_pose(), stow_head_pose()])
            .expect_err("the sixth leg closes nothing");
        assert!(matches!(
            error,
            FkError::NoConvergence { .. } | FkError::WrongAssemblyMode { .. }
        ));

        // No seeds at all is the same refusal rather than a pose from nowhere.
        assert!(matches!(
            ArmRecord::solve(&geom, &opts, &joints, &[]),
            Err(FkError::NoConvergence { .. })
        ));

        // Worth knowing, and asserted so nobody assumes otherwise: six cranks at
        // zero *do* close the linkage, at a near-level head 150 mm up. An
        // all-zero reading is not evidence of anything being wrong.
        let zeros = JointVector::default();
        let record = ArmRecord::solve(&geom, &opts, &zeros, &[rest_head_pose()])
            .expect("all six cranks at zero is a real configuration");
        assert!((record.head_pose_body.translation.z - 0.149_571).abs() < 1e-6);
    }

    /// A servo reporting a quarter turn from where the model puts it is caught
    /// by neither the solver nor the travel windows at this resting
    /// configuration, and that is why the provisioned homing offsets are read
    /// back and compared rather than inferred from the angles.
    ///
    /// The legs' offsets are what put a crank's mechanical zero at the model's,
    /// so a servo that lost its offset reports the same measurement shifted ±90°
    /// by parity. Those angles close the linkage perfectly well — a head tilted
    /// 55°, 176 mm up, inside the solver's plausibility band — and they sit
    /// inside every travel window, which the *correct* reading of this rest does
    /// not. Nothing on the command path would notice; the head would simply not
    /// be where the model thinks it is, by five centimetres.
    #[test]
    fn a_quarter_turn_offset_error_looks_admissible_at_this_rest() {
        let geom = HeadGeometry::default();
        let opts = FkOptions::default();
        let mut joints = joints_at(&rest_head_pose());
        for (leg, angle) in joints.legs.iter_mut().enumerate() {
            *angle += if leg % 2 == 0 { PI / 2.0 } else { -PI / 2.0 };
        }
        let record = ArmRecord::solve(&geom, &opts, &joints, &[rest_head_pose(), stow_head_pose()])
            .expect("the shifted reading closes the linkage");
        let tilt = reachy_kin::cone_angle(&record.head_pose_body.rotation).to_degrees();
        assert!((tilt - 55.2).abs() < 0.5, "tilt {tilt}°");
        assert!((record.head_pose_body.translation.z - 0.175_602).abs() < 1e-6);

        let windows = EnvelopeConfig::default().crank_windows;
        let outside = |legs: &[f64; 6]| {
            legs.iter()
                .zip(windows)
                .filter(|(angle, (low, high))| **angle < *low || **angle > *high)
                .count()
        };
        // Every shifted angle is inside its window, while four of the six correct
        // ones are not: at this rest the windows admit the wrong reading and
        // refuse the right one.
        assert_eq!(outside(&joints.legs), 0);
        assert_eq!(outside(&joints_at(&rest_head_pose()).legs), 4);

        // What the two readings actually differ by: five centimetres of head.
        let gap = record.head_pose_body.translation.z - rest_head_pose().translation.z;
        assert!(gap > 0.04, "the two poses differ by {gap} m in height");
    }

    /// A record's angles travel as measured. An arming report that quietly
    /// tidied them would be the one place the machine's own account of itself
    /// was edited.
    #[test]
    fn a_record_carries_the_angles_it_was_given() {
        let geom = HeadGeometry::default();
        let opts = FkOptions::default();
        let rest = rest_head_pose();
        let mut joints = joints_at(&rest);
        joints.body_yaw = 0.25;
        joints.antennas = [-1.5, 1.5];
        let record = ArmRecord::solve(&geom, &opts, &joints, &[rest]).expect("the legs solve");
        assert_eq!(record.joints, joints);
        assert_eq!(record.joints.body_yaw, 0.25);
        assert_eq!(record.joints.antennas, [-1.5, 1.5]);
    }

    /// The provisioning table addresses one cell per servo per register, and
    /// says how many of them arming will actually fail on.
    #[test]
    fn the_provision_table_addresses_a_cell_per_servo_and_register() {
        let mut table = ProvisionTable::new();
        assert_eq!(table.checks(), 0);
        assert_eq!(table.reads(), 0);

        assert!(table.set_all(RegId::OperatingMode, ProvisionExpect::Check(value::u8(3))));
        assert!(table.set_all(RegId::Shutdown, ProvisionExpect::Record));
        assert!(table.set_legs(
            RegId::MinPositionLimit,
            ProvisionExpect::Check(value::u32(1502))
        ));
        assert_eq!(table.checks(), ROW_COUNT + 6);
        assert_eq!(table.reads(), 2 * ROW_COUNT + 6);

        let mode = ProvisionTable::column(RegId::OperatingMode).expect("a provisioned register");
        let limit =
            ProvisionTable::column(RegId::MinPositionLimit).expect("a provisioned register");
        assert_eq!(
            table.at(0, mode),
            Some(ProvisionExpect::Check(value::u8(3)))
        );
        // Yaw is row 0 and has no travel window of its own.
        assert_eq!(table.at(0, limit), Some(ProvisionExpect::Skip));
        assert_eq!(
            table.at(1, limit),
            Some(ProvisionExpect::Check(value::u32(1502)))
        );

        assert_eq!(ProvisionTable::column(RegId::PresentPosition), None);
        assert!(!table.set(leg_ref(9), RegId::OperatingMode, ProvisionExpect::Record));
        assert!(!table.set(
            JointRef::BodyYaw,
            RegId::GoalPosition,
            ProvisionExpect::Record
        ));
        assert_eq!(table.at(ROW_COUNT, mode), None);
        assert_eq!(table.at(0, PROVISION_REGS.len()), None);
    }

    /// Every provisioned register appears once, and none of them is one the
    /// command path writes: arming verifies what the servo was set up with, and
    /// a goal or a torque flag is not that.
    #[test]
    fn the_provisioned_registers_are_a_set_of_setup_registers() {
        for (index, reg) in PROVISION_REGS.iter().enumerate() {
            assert_eq!(
                PROVISION_REGS.iter().position(|entry| entry == reg),
                Some(index),
                "{reg} appears twice"
            );
            assert!(
                !matches!(
                    reg,
                    RegId::GoalPosition
                        | RegId::PresentPosition
                        | RegId::TorqueEnable
                        | RegId::PositionGains
                ),
                "{reg} is written or read on the command path, not provisioned"
            );
        }
    }

    /// The IDs are the platform's own numbering in bus order, and each joint
    /// finds its own.
    #[test]
    fn every_joint_maps_to_its_servo() {
        let cfg = config();
        assert_eq!(cfg.id_for(JointRef::BodyYaw), Some(10));
        assert_eq!(cfg.id_for(JointRef::Leg0), Some(11));
        assert_eq!(cfg.id_for(JointRef::Leg5), Some(16));
        assert_eq!(cfg.id_for(JointRef::AntennaRight), Some(17));
        assert_eq!(cfg.id_for(JointRef::AntennaLeft), Some(18));
        assert_eq!(cfg.id_for(leg_ref(6)), None);
        for (row, joint) in ROWS.into_iter().enumerate() {
            assert_eq!(cfg.id_for(joint), Some(SERVO_IDS[row]));
        }
    }

    /// The legs are tuned harder than the joints carrying nothing, and all
    /// three cross the boundary as the gain span the wire layer writes.
    ///
    /// The legs are the only group with an integral term, and they are the only
    /// group holding a weight up: the measured droop a proportional term alone
    /// leaves is what that term is there to close, so a leg gain set without one
    /// is the configuration that droop was measured on.
    #[test]
    fn gains_are_per_group() {
        let gains = DEFAULT_GAINS;
        assert_eq!(gains.for_joint(JointRef::Leg3), gains.legs);
        assert_eq!(gains.for_joint(JointRef::BodyYaw), gains.yaw);
        assert_eq!(gains.for_joint(JointRef::AntennaLeft), gains.antennas);
        assert!(gains.legs.p > gains.antennas.p);
        assert!(gains.antennas.p > gains.yaw.p);
        assert!(gains.legs.i > 0, "the loaded group integrates its error");
        assert_eq!((gains.yaw.i, gains.antennas.i), (0, 0));
        for group in [gains.legs, gains.yaw, gains.antennas] {
            assert_eq!(group.value(), value::gains(group.p, group.i, group.d));
        }
        assert_eq!(DEFAULT_GAINS.legs.to_string(), "P 800 I 100 D 300");
    }

    /// The provisional thresholds are the values the comments say they are. A
    /// figure nobody has measured is worth pinning, so changing one is a
    /// deliberate act rather than a drift.
    #[test]
    fn the_provisional_thresholds_are_what_they_claim() {
        assert_eq!(DEFAULT_MIN_ARM_VOLTAGE, 6.0);
        assert_eq!(DEFAULT_VOLTAGE_POLL_PERIOD, Duration::from_millis(100));
        assert_eq!(DEFAULT_VOLTAGE_BUDGET, Duration::from_secs(30));
    }

    /// A window narrower than the platform's own does not silently take over: a
    /// pin is against the servo-side fence, so a caller handing over the wrong
    /// fence pulls the legs to it, and the recorded pull is what says so.
    #[test]
    fn the_pin_is_against_the_window_it_was_handed() {
        let narrow = 5.0_f64.to_radians();
        let cfg = ArmConfig {
            leg_windows: [(-narrow, narrow); 6],
            ..config()
        };
        let present = joints_at(&reachy_kin::neutral_head_pose());
        // The neutral pose sits near ±36°, so every leg is pulled to the fence.
        let pulled = pin_goals(&cfg, &present).expect("nothing here refuses");
        assert_eq!(
            pulled.pinned.legs,
            [narrow, -narrow, narrow, -narrow, narrow, -narrow]
        );
        assert!(pulled.worst_pull_in().to_degrees() > 30.0);

        let wide = ArmConfig {
            leg_windows: [(-PI, PI); 6],
            ..cfg
        };
        let outcome = pin_goals(&wide, &present).expect("nothing to pull");
        assert_eq!(outcome.pinned, present);
    }

    /// A leg exactly on its bound is inside it. The bound is where the pin puts
    /// a leg it pulled, so a bound treated as outside would re-pin the same leg
    /// for ever.
    #[test]
    fn a_leg_on_its_bound_is_not_pulled() {
        let cfg = config();
        let mut present = joints_at(&reachy_kin::neutral_head_pose());
        for (angle, (low, _)) in present.legs.iter_mut().zip(cfg.leg_windows) {
            *angle = low;
        }
        let outcome = pin_goals(&cfg, &present).expect("nothing to pull");
        assert_eq!(outcome.pull_in, [0.0; 6]);
        assert_eq!(outcome.pinned.legs, present.legs);
    }

    /// Nine servos answering out of their own state, with a knob per thing a test
    /// wants to vary. The transaction log is what the phase-order assertions
    /// read: it is the whole content of "arming did these things in this order".
    ///
    /// The goal register is modelled as this platform's: with torque off it
    /// mirrors the present position and keeps nothing written to it, and once
    /// torque is on it stores what is written and the servo goes there.
    #[derive(Clone, Debug)]
    struct Machine {
        models: [u16; ROW_COUNT],
        silent: [bool; ROW_COUNT],
        /// Supply readings, one per poll; the last one repeats for ever.
        sweeps: Vec<f64>,
        /// Per servo, how far below the sweep's reading that servo reports.
        sag: [f64; ROW_COUNT],
        health: [u8; ROW_COUNT],
        /// Whether each servo holds torque, as one fact rather than two: an
        /// enable that lands sets it, and everything torque decides — the goal
        /// register storing writes, the reported frame taking its enable shift,
        /// a goal becoming a motion — reads it.
        torque: [u8; ROW_COUNT],
        /// Where each joint reads while the machine is as arming found it.
        present: JointVector,
        /// What the goal register of a servo *found holding torque* holds: the
        /// target it is holding. A limp servo's goal mirrors its present
        /// position instead and this says nothing about it.
        held: JointVector,
        /// Per servo, how far its goal register reads from its present position
        /// while it is limp. Zero is the mirroring this platform does.
        shadow_gap: [f64; ROW_COUNT],
        /// Per joint, what its reported position jumps by when its torque comes
        /// on — the single-turn renormalisation a servo may do there.
        enable_shift: [f64; ROW_COUNT],
        /// Per joint, how far below its goal a servo holding torque settles: the
        /// standing error of a loaded position loop with no integral term.
        load: [f64; ROW_COUNT],
        /// What a provisioned register holds.
        provision: Vec<(RegId, Value)>,
        /// One write to answer with something other than success. Separate from
        /// the read knob because several registers are both read and written in
        /// one sequence, and which of the two fails is the whole question.
        fail_write: Option<(u8, RegId, BusResult)>,
        /// One read to answer with something other than success.
        fail_read: Option<(u8, RegId, BusResult)>,
        /// Per servo, how many further position reads answer with something
        /// nobody can place, counted down as they arrive: the corrupt frame a
        /// sweep re-reads past, rather than a joint that has stopped reporting.
        nan_reads: [usize; ROW_COUNT],
        /// The goals the servos are holding, once each has been written.
        goals: JointVector,
        written: [bool; ROW_COUNT],
        poll: usize,
        waits: usize,
        log: Vec<(SeqStepKind, Asked)>,
    }

    /// A platform resting where the record says it rests, provisioned as
    /// configured, on a healthy 7.4 V rail with torque off.
    ///
    /// The body and the antennas rest at three angles of their own rather than
    /// at zero: three zeros make a round trip through them agree with itself
    /// whichever slot it went through, and those three slots have no window to
    /// catch a swap the way the legs do.
    fn bus() -> Machine {
        let mut present = joints_at(&rest_head_pose());
        present.body_yaw = 0.35;
        present.antennas = [0.20, -0.15];
        Machine {
            models: EXPECTED_MODELS,
            silent: [false; ROW_COUNT],
            sweeps: vec![7.4],
            sag: [0.0; ROW_COUNT],
            health: [0; ROW_COUNT],
            torque: [0; ROW_COUNT],
            present,
            held: JointVector::default(),
            shadow_gap: [0.0; ROW_COUNT],
            enable_shift: [0.0; ROW_COUNT],
            load: [0.0; ROW_COUNT],
            provision: vec![
                (RegId::OperatingMode, value::u8(3)),
                (RegId::HomingOffset, value::i32(1024)),
                (RegId::Shutdown, value::u8(0x34)),
            ],
            fail_write: None,
            fail_read: None,
            nan_reads: [0; ROW_COUNT],
            goals: JointVector::default(),
            written: [false; ROW_COUNT],
            poll: 0,
            waits: 0,
            log: Vec::new(),
        }
    }

    impl Machine {
        fn provisioned_as(mut self, reg: RegId, value: Value) -> Self {
            for cell in &mut self.provision {
                if cell.0 == reg {
                    cell.1 = value;
                }
            }
            self
        }

        fn value(&mut self, row: usize, reg: RegId) -> Value {
            match reg {
                RegId::ModelNumber => value::u16(self.models[row]),
                RegId::PresentInputVoltage => {
                    let volts = self.sweeps[self.poll.min(self.sweeps.len() - 1)] - self.sag[row];
                    if row + 1 == ROW_COUNT {
                        self.poll += 1;
                    }
                    value::volts(volts)
                }
                RegId::HardwareErrorStatus => value::u8(self.health[row]),
                RegId::TorqueEnable => value::u8(self.torque[row]),
                RegId::PresentPosition => {
                    if self.nan_reads[row] > 0 {
                        self.nan_reads[row] -= 1;
                        return value::radians(f64::NAN);
                    }
                    value::radians(self.position(row))
                }
                RegId::GoalPosition => value::radians(self.goal(row)),
                other => self
                    .provision
                    .iter()
                    .find(|(reg, _)| *reg == other)
                    .map_or(value::u8(0), |(_, value)| *value),
            }
        }

        /// Which servos are holding torque now, in the shape the enable
        /// assertions compare against.
        fn enabled(&self) -> [bool; ROW_COUNT] {
            self.torque.map(|state| state != 0)
        }

        /// What the goal register reads: what was last written to it, the held
        /// target on a servo found holding torque, and otherwise the present
        /// position it is mirroring, off by whatever gap the test asked for.
        fn goal(&self, row: usize) -> f64 {
            if self.written[row] {
                return angle_at(&self.goals, row);
            }
            if self.torque[row] != 0 {
                return angle_at(&self.held, row);
            }
            self.position(row) + self.shadow_gap[row]
        }

        /// Where a joint reads: at rest, plus whatever its torque coming on did
        /// to the reported frame, and a load below its written goal once one has
        /// been written.
        fn position(&self, row: usize) -> f64 {
            let mut angle = angle_at(&self.present, row);
            if self.torque[row] != 0 {
                angle += self.enable_shift[row];
            }
            if !self.written[row] {
                return angle;
            }
            angle_at(&self.goals, row) - self.load[row]
        }
    }

    impl ScriptedBus for Machine {
        fn answer(&mut self, step: SeqStepKind, request: &BusTxnWire) -> BusResult {
            let Asked { op, context, value } = asked(request, step);
            self.log.push((step, Asked { op, context, value }));
            let row = SERVO_IDS
                .iter()
                .position(|id| *id == context.id)
                .expect("addressed to a servo on this bus");
            if self.silent[row] {
                return BusResult::NoAnswer;
            }
            let scripted = match op {
                AuxOpKind::WriteRegVerified => self.fail_write,
                _ => self.fail_read,
            };
            if let Some((id, reg, result)) = scripted
                && context.id == id
                && context.reg == Some(reg)
            {
                return result;
            }
            match op {
                AuxOpKind::Ping => BusResult::Pinged {
                    model: self.models[row],
                },
                AuxOpKind::ReadReg => {
                    BusResult::Value(self.value(row, context.reg.expect("a read names a register")))
                }
                AuxOpKind::None => panic!("a sequencer emitted no transaction"),
                AuxOpKind::WriteRegVerified => {
                    let reg = context.reg.expect("a write names a register");
                    match reg {
                        // A goal written to a limp servo is dropped on the
                        // floor, which is the whole reason the sequence enables
                        // torque first.
                        RegId::GoalPosition => {
                            if let Some(rad) = value.as_radians()
                                && self.torque[row] != 0
                            {
                                self.goals.set(ROWS[row], rad);
                                self.written[row] = true;
                            }
                        }
                        RegId::TorqueEnable if value.as_u8() == Some(1) => {
                            self.torque[row] = 1;
                        }
                        _ => {}
                    }
                    BusResult::Written
                }
            }
        }

        fn waited(&mut self, _now: Duration, _until: Duration) {
            self.waits += 1;
        }
    }

    /// The same platform, found already holding torque on every servo at the
    /// goals a previous command left it, sagging `droop` below each of them.
    ///
    /// The state a re-arm within one session actually meets: every command but
    /// `off` exits without releasing torque, so the next one finds the machine
    /// standing at whatever it was last sent — the neutral configuration here,
    /// which is what `up` leaves and which sits inside every travel window, so
    /// nothing about this machine needs pulling anywhere.
    fn holding(droop: f64) -> Machine {
        let mut held = joints_at(&reachy_kin::neutral_head_pose());
        held.body_yaw = 0.35;
        held.antennas = [0.20, -0.15];
        let mut present = held;
        for (row, joint) in ROWS.into_iter().enumerate() {
            present.set(joint, angle_at(&held, row) - droop);
        }
        Machine {
            present,
            held,
            load: [droop; ROW_COUNT],
            torque: [1; ROW_COUNT],
            ..bus()
        }
    }

    /// What a whole torque-on path handed back, in the order it ran.
    #[derive(Clone, Copy, Debug)]
    struct Armed {
        commission: CommissionSummary,
        posture: Posture,
        engage: EngageSummary,
    }

    /// Commission, poll and engage `machine`, as a process does: once, then a
    /// resting sweep, then the fast path.
    fn drive(cfg: &ArmConfig, machine: &mut Machine) -> Result<Armed, SeqError> {
        let mut slot = CommissionSnapWire::new();
        drive_from(cfg, machine, &mut slot)
    }

    /// The same, with the commissioning's state left in `slot` for a test that
    /// reads the provisioning grid off it — the grid is the sequence's own state
    /// and not part of the summary.
    fn drive_from(
        cfg: &ArmConfig,
        machine: &mut Machine,
        slot: &mut CommissionSnapWire,
    ) -> Result<Armed, SeqError> {
        let commission =
            crate::testutil::drive(&mut CommissionSequencer::start(cfg, slot), machine)?;
        let posture = poll(cfg, machine, commission.rail, PollCadence::Positions)?;
        let engage = engage(cfg, machine, &posture)?;
        Ok(Armed {
            commission,
            posture,
            engage,
        })
    }

    fn commission(cfg: &ArmConfig, machine: &mut Machine) -> Result<CommissionSummary, SeqError> {
        let mut slot = CommissionSnapWire::new();
        crate::testutil::drive(&mut CommissionSequencer::start(cfg, &mut slot), machine)
    }

    fn poll(
        cfg: &ArmConfig,
        machine: &mut Machine,
        rail: Rail,
        cadence: PollCadence,
    ) -> Result<Posture, SeqError> {
        let mut slot = PollSnapWire::new();
        crate::testutil::drive(
            &mut PollSequencer::start(cfg, &mut slot, rail, cadence),
            machine,
        )
    }

    fn engage(
        cfg: &ArmConfig,
        machine: &mut Machine,
        posture: &Posture,
    ) -> Result<EngageSummary, SeqError> {
        let geom = HeadGeometry::default();
        let fk = FkOptions::default();
        let mut slot = EngageSnapWire::new();
        let mut seq = EngageSequencer::start(cfg, &geom, &fk, &mut slot, posture);
        crate::testutil::drive(&mut seq, machine)
    }

    /// An arming sequence restored with an outstanding record this build cannot
    /// read stops, under the phase and the servo the record still names. Arming
    /// is the half that may stop: nothing has torque on yet, so a step nobody
    /// can describe is a refusal rather than something to walk past.
    #[test]
    fn a_pending_record_that_cannot_be_read_stops_the_commissioning() {
        let cfg = config();
        let mut slot = CommissionSnapWire::new();
        let mut seq = CommissionSequencer::start(&cfg, &mut slot);
        assert!(matches!(
            seq.next(Duration::ZERO, None),
            SeqAction::Transact
        ));
        let addressed = asked(seq.pending(), SeqStepKind::Presence).id();

        // The record it is waiting on, left naming a register where the
        // operation needs one — the shape of a slot written by something that
        // does not agree with this build about what a transaction is.
        let state = slot.validate_mut().expect("the state was written here");
        state.pending.op = crate::txn::AuxOpKind::ReadReg;
        state.pending.reg = RegId::None;
        let mut seq =
            CommissionSequencer::resume(&cfg, state).expect("a commissioning mid-sweep resumes");

        // No answer is handed back: the record is read before anything that
        // came back is looked at, which is why the refusal is about the record.
        let SeqAction::Fail(error) = seq.next(Duration::ZERO, None) else {
            panic!("an unreadable record is a refusal");
        };
        let SeqError::PendingUnreadable { context } = error else {
            panic!("the refusal names the record: {error}");
        };
        assert_eq!(context.step, SeqStepKind::Presence);
        assert_eq!(context.id, addressed);
    }

    /// A configuration that actually checks something: position mode on all nine,
    /// with the two registers nobody has established a correct value for recorded
    /// rather than judged.
    fn provisioned_config() -> ArmConfig {
        let mut expected = ProvisionTable::new();
        assert!(expected.set_all(RegId::OperatingMode, ProvisionExpect::Check(value::u8(3))));
        assert!(expected.set_all(RegId::HomingOffset, ProvisionExpect::Record));
        assert!(expected.set_all(RegId::Shutdown, ProvisionExpect::Record));
        ArmConfig {
            expected,
            ..config()
        }
    }

    /// How many times `id`'s position was read, which is what a re-read shows
    /// up as.
    fn reads_of(log: &[(SeqStepKind, Asked)], id: u8) -> usize {
        log.iter()
            .filter(|(_, request)| {
                request.op == AuxOpKind::ReadReg
                    && request.context.id == id
                    && request.context.reg == Some(RegId::PresentPosition)
            })
            .count()
    }

    fn writes(log: &[(SeqStepKind, Asked)], reg: RegId) -> usize {
        log.iter()
            .filter(|(_, request)| {
                request.op == AuxOpKind::WriteRegVerified && request.context.reg == Some(reg)
            })
            .count()
    }

    /// The whole path against a machine that engages: the phase order, the
    /// transaction counts, and the records each half hands back.
    #[test]
    fn the_torque_on_path_runs_its_phases_in_order_and_records_what_it_found() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let mut commissioned = CommissionSnapWire::new();
        let summary =
            drive_from(&cfg, &mut machine, &mut commissioned).expect("this machine engages");

        let mut phases: Vec<SeqStepKind> = Vec::new();
        for (step, _) in &machine.log {
            if phases.last() != Some(step) {
                phases.push(*step);
            }
        }
        assert_eq!(
            phases,
            vec![
                SeqStepKind::Presence,
                SeqStepKind::Identity,
                SeqStepKind::Provision,
                SeqStepKind::VoltageGate,
                SeqStepKind::Health,
                SeqStepKind::GainsProfiles,
                SeqStepKind::PoseAndDatum,
                SeqStepKind::PinAndEnable,
            ]
        );

        // No write of any kind precedes the supply gate: the gains and profiles
        // go into servo RAM, and a rail browning out is how a servo ends up
        // holding half a configuration.
        let first_write = machine
            .log
            .iter()
            .position(|(_, request)| request.op == AuxOpKind::WriteRegVerified)
            .expect("commissioning writes");
        let last_voltage = machine
            .log
            .iter()
            .rposition(|(step, _)| *step == SeqStepKind::VoltageGate)
            .expect("the gate ran");
        assert!(first_write > last_voltage);

        // Commissioning touches torque in neither direction, and reads no
        // position: the machine it hands on is exactly the machine it met.
        let commissioning = |step: &SeqStepKind| {
            !matches!(step, SeqStepKind::PoseAndDatum | SeqStepKind::PinAndEnable)
        };
        assert!(
            machine
                .log
                .iter()
                .filter(|(s, _)| commissioning(s))
                .all(|(_, request)| request.reg() != Some(RegId::TorqueEnable)
                    && request.reg() != Some(RegId::PresentPosition))
        );

        // Every goal is pinned before any torque is enabled, and every enable
        // before the pose is read back. The pin sweep is belt-and-braces against
        // a stale goal register; the read-back is what the tick starts from, so
        // it has to see the machine with its torque already on.
        let last_pin = machine
            .log
            .iter()
            .rposition(|(_, request)| request.reg() == Some(RegId::GoalPosition))
            .expect("engaging pins");
        let first_enable = machine
            .log
            .iter()
            .position(|(_, request)| request.reg() == Some(RegId::TorqueEnable))
            .expect("engaging enables torque");
        let last_enable = machine
            .log
            .iter()
            .rposition(|(_, request)| request.reg() == Some(RegId::TorqueEnable))
            .expect("engaging enables torque");
        let first_settle = machine
            .log
            .iter()
            .position(|(step, request)| {
                *step == SeqStepKind::PinAndEnable && request.reg() == Some(RegId::PresentPosition)
            })
            .expect("engaging reads back");
        assert!(last_pin < first_enable);
        assert!(last_enable < first_settle);

        let count = |step| machine.log.iter().filter(|(s, _)| *s == step).count();
        assert_eq!(count(SeqStepKind::Presence), ROW_COUNT);
        assert_eq!(count(SeqStepKind::Identity), ROW_COUNT);
        assert_eq!(count(SeqStepKind::Provision), 3 * ROW_COUNT);
        assert_eq!(count(SeqStepKind::VoltageGate), ROW_COUNT);
        assert_eq!(count(SeqStepKind::Health), ROW_COUNT);
        // Gains, then every profile register, per servo — the same count the
        // cursor bound derives, so the sweep and the bound cannot disagree.
        assert_eq!(count(SeqStepKind::GainsProfiles), GAINS_PROFILE_WRITES);
        // And the two profile registers carry the configured pair, on every
        // row: the profile is the one number in this record a deployment
        // chooses, so a sweep writing anything else — a swapped pair, a
        // dropped field, a zero — is a machine running a limiter nobody asked
        // for.
        for (reg, wanted) in [
            (
                RegId::ProfileAcceleration,
                value::u32(cfg.profile.acceleration),
            ),
            (RegId::ProfileVelocity, value::u32(cfg.profile.velocity)),
        ] {
            let written: Vec<(u8, Value)> = machine
                .log
                .iter()
                .filter(|(step, request)| {
                    *step == SeqStepKind::GainsProfiles && request.reg() == Some(reg)
                })
                .map(|(_, request)| (request.id(), request.value))
                .collect();
            assert_eq!(
                written,
                cfg.ids.map(|id| (id, wanted)).to_vec(),
                "{reg:?} over the nine rows",
            );
        }
        // The resting sweep: nine positions and nothing else.
        assert_eq!(count(SeqStepKind::PoseAndDatum), ROW_COUNT);
        // A pin, an enable and a read-back, per servo. Twenty-seven
        // transactions is the whole cost of a wake word reaching the head.
        assert_eq!(count(SeqStepKind::PinAndEnable), 3 * ROW_COUNT);

        let pins = pin_goals(&cfg, &machine.present).expect("inside the window");
        assert_eq!(summary.posture.present, machine.present);
        assert_eq!(summary.engage.rest.joints, machine.present);
        assert_eq!(summary.engage.pins.pinned, pins.pinned);
        assert_eq!(summary.engage.pins.pull_in, pins.pull_in);
        // This fixture's torque-on moves nothing, so the pose read back is the
        // pose that was measured.
        assert_eq!(summary.engage.armed.joints, machine.present);
        assert_eq!(summary.engage.post_enable_shift, [0.0; ROW_COUNT]);
        assert_eq!(machine.enabled(), [true; ROW_COUNT]);

        assert_eq!(summary.commission.voltage_polls, 1);
        assert_eq!(machine.waits, 0);
        assert_eq!(summary.commission.models, machine.models);
        assert_eq!(summary.commission.rail.voltages, [7.4; ROW_COUNT]);
        assert_eq!(
            summary.commission.rail.health[0],
            ServoHealth { id: 10, bits: 0 }
        );
        // A positions-only sweep carries the rail forward rather than leaving a
        // gate to judge half a picture.
        assert!(!summary.posture.rail_read);
        assert_eq!(summary.posture.rail, summary.commission.rail);
        let state = commissioned
            .validate()
            .expect("the sequence wrote its state");
        let grid = &state.provisioned;
        assert_eq!(
            cells::provisioned(grid, JointRef::Leg5, RegId::OperatingMode),
            Some(value::u8(3))
        );
        assert_eq!(
            cells::provisioned(grid, JointRef::BodyYaw, RegId::Shutdown),
            Some(value::u8(0x34))
        );
        assert_eq!(cells::recorded(grid), 3 * ROW_COUNT);
        assert_eq!(
            cells::provisioned(grid, JointRef::BodyYaw, RegId::CurrentLimit),
            None
        );

        // The armed record is what a tick is started from, and it is below the
        // clearance floor at this rest: the margin baseline is what carries the
        // first lift, exactly as it does off the fixture in `tick.rs`.
        let mut slot = crate::tick::MotionSnapWire::new();
        let state = slot.clear_valid();
        crate::tick::arm(state, &summary.engage.armed, summary.engage.degraded);
        assert_eq!(crate::tick::last_goal(state), machine.present);
        assert!(summary.engage.armed.min_margin > 0.0);
        assert!(summary.engage.armed.min_margin < EnvelopeConfig::default().min_toggle_margin);
    }

    /// The pins go out before the enables, carrying the angles each joint was
    /// measured at, and a machine that drops them all the same engages.
    ///
    /// With torque off this platform's goal register mirrors the present
    /// position and keeps nothing written to it — the fixture models exactly
    /// that — so every one of these writes lands nowhere. That is the expected
    /// case, and the sequence neither depends on the write sticking nor treats
    /// the mirrored read-back as a refusal.
    #[test]
    fn the_pin_sweep_is_issued_and_its_answers_are_not_judged() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let summary = drive(&cfg, &mut machine).expect("a mirroring register engages");

        let pinned: Vec<f64> = machine
            .log
            .iter()
            .filter_map(|(_, request)| match (request.op, request.reg()) {
                (AuxOpKind::WriteRegVerified, Some(RegId::GoalPosition)) => {
                    request.value.as_radians()
                }
                _ => None,
            })
            .collect();
        let expected: Vec<f64> = (0..ROW_COUNT)
            .map(|row| angle_at(&summary.engage.pins.pinned, row))
            .collect();
        assert_eq!(pinned, expected);
        assert!(
            !machine.written.iter().any(|kept| *kept),
            "the fixture kept a goal written to a limp servo"
        );
    }

    /// A servo that refuses the pin write, or says nothing to it at all, does
    /// not stop the machine being taken hold of.
    ///
    /// Nothing about the pin sweep is load-bearing, and a refusal there would
    /// gate torque coming on behind the very register mirroring that makes
    /// torque coming on safe.
    #[test]
    fn a_pin_write_that_goes_nowhere_does_not_stop_the_engage() {
        let cfg = provisioned_config();
        for answer in [
            BusResult::NoAnswer,
            BusResult::ServoError { code: 0x08 },
            BusResult::VerifyMismatch {
                read_back: value::radians(1.0),
            },
        ] {
            let mut machine = Machine {
                fail_write: Some((14, RegId::GoalPosition, answer)),
                ..bus()
            };
            drive(&cfg, &mut machine).expect("a pin that goes nowhere is not a refusal");
            assert_eq!(machine.enabled(), [true; ROW_COUNT]);
        }
    }

    /// An enable that does not land stops the sequence where it stood: a servo
    /// that did not take torque is not holding anything up.
    #[test]
    fn an_enable_that_does_not_land_stops_the_engage() {
        let cfg = provisioned_config();
        let mut machine = Machine {
            fail_write: Some((14, RegId::TorqueEnable, BusResult::NoAnswer)),
            ..bus()
        };
        let error = drive(&cfg, &mut machine).expect_err("servo 14 never took torque");
        let SeqError::NoAnswer { context } = error else {
            panic!("expected silence, got {error}");
        };
        assert_eq!(context.id, 14);
        assert_eq!(context.step, SeqStepKind::PinAndEnable);
    }

    /// A failed engage says whether it may have left torque on, and that is
    /// what decides whether the caller has a machine to take to the minimum
    /// risk condition.
    ///
    /// Three endings, one question: a gate refusal put nothing on the wire; a
    /// pin sweep that died mid-walk wrote goals to a limp machine; an enable
    /// that went unanswered may have been taken. Only the third leaves torque
    /// to write off, and it is the unacknowledged write — the one nobody can
    /// say landed — that has to count as written.
    #[test]
    fn a_failed_engage_says_whether_torque_may_be_on() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let commission = commission(&cfg, &mut machine).expect("commissioning passes");
        let posture = poll(&cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect("the sweep reads");

        let geom = HeadGeometry::default();
        let fk = FkOptions::default();

        let mut sagging = posture;
        sagging.rail.voltages[4] = 5.5;
        let mut sag_slot = EngageSnapWire::new();
        let mut seq = EngageSequencer::start(&cfg, &geom, &fk, &mut sag_slot, &sagging);
        crate::testutil::drive(&mut seq, &mut machine).expect_err("a low rail does not torque on");
        assert!(!seq.torque_written(), "the gate refused before any write");

        // The pin sweep is not judged, so nothing stops there on the machine's
        // answers — the port itself going away is what ends a walk mid-pin, and
        // the driver reports that rather than the sequencer. Standing in for it:
        // a sequence stopped after one pin write has still written no torque.
        let mut pin_slot = EngageSnapWire::new();
        let mut seq = EngageSequencer::start(&cfg, &geom, &fk, &mut pin_slot, &posture);
        assert!(matches!(
            seq.next(Duration::ZERO, None),
            SeqAction::Transact
        ));
        let request = asked(seq.pending(), SeqStepKind::PinAndEnable);
        assert_eq!(request.op, AuxOpKind::WriteRegVerified);
        assert_eq!(request.context.reg, Some(RegId::GoalPosition));
        assert!(!seq.torque_written(), "a pin is not an enable");

        let mut machine = Machine {
            fail_write: Some((14, RegId::TorqueEnable, BusResult::NoAnswer)),
            ..bus()
        };
        let mut enable_slot = EngageSnapWire::new();
        let mut seq = EngageSequencer::start(&cfg, &geom, &fk, &mut enable_slot, &posture);
        crate::testutil::drive(&mut seq, &mut machine).expect_err("servo 14 never answered");
        assert!(
            seq.torque_written(),
            "an enable nobody answered may have landed"
        );
    }

    /// The two torque-on gates are the whole enumeration, and they refuse
    /// before a single transaction crosses the wire.
    #[test]
    fn the_torque_on_gates_refuse_without_touching_the_machine() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let commission = commission(&cfg, &mut machine).expect("commissioning passes");
        let posture = poll(&cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect("the sweep reads");
        machine.log.clear();

        let mut sagging = posture;
        sagging.rail.voltages[4] = 5.5;
        let error =
            engage(&cfg, &mut machine, &sagging).expect_err("a low rail does not torque on");
        let SeqError::SupplyBelowFloor {
            context, lowest, ..
        } = error
        else {
            panic!("expected a supply refusal, got {error}");
        };
        assert_eq!(context.id, 14);
        assert!((lowest - 5.5).abs() < 1e-12);
        // The gate that waits and the gate that does not are different errors:
        // this one never had a budget, so it does not report expiring one.
        assert!(
            !error.to_string().contains("after"),
            "the engage refusal reads as a wait that expired: {error}"
        );

        let mut latched = posture;
        latched.rail.health[6] = ServoHealth { id: 16, bits: 0x20 };
        let error = engage(&cfg, &mut machine, &latched).expect_err("a latch does not torque on");
        let SeqError::UnhealthyServo { context, bits } = error else {
            panic!("expected a health refusal, got {error}");
        };
        assert_eq!(context.id, 16);
        assert_eq!(bits, 0x20);

        assert!(machine.log.is_empty(), "a refused engage wrote nothing");
        assert_eq!(machine.enabled(), [false; ROW_COUNT]);

        // An input-voltage bit on its own is not a latch: it is what a rail
        // recovering from a dip leaves behind, and the floor above is what
        // judges the rail.
        let mut dipped = posture;
        dipped.rail.health[6] = ServoHealth { id: 16, bits: 0x01 };
        engage(&cfg, &mut machine, &dipped).expect("an input-voltage bit alone engages");
    }

    /// A latched servo does not stop the process commissioning.
    ///
    /// The latch refuses torque coming on, once, where the rest of the torque-on
    /// gates live. Refusing here as well would stop a daemon from starting at
    /// all over a servo hurting nothing while the machine rests limp.
    #[test]
    fn commissioning_records_a_latched_servo_without_refusing_it() {
        let cfg = provisioned_config();
        let mut health = [0; ROW_COUNT];
        health[3] = 0x20;
        let mut machine = Machine { health, ..bus() };
        let summary = commission(&cfg, &mut machine).expect("a latch does not stop commissioning");
        assert_eq!(summary.rail.health[3], ServoHealth { id: 13, bits: 0x20 });
        assert_eq!(machine.enabled(), [false; ROW_COUNT]);
        assert!(engage_gates(&cfg, &summary.rail).is_err());
    }

    /// The health gate answers by group: a head servo refuses, an antenna is
    /// named and engaged around.
    ///
    /// A servo carrying the head that has flagged is the one thing torque must
    /// not come on over. An antenna is not carrying anything, and one latched
    /// overload — what an interference incident leaves behind — must not make
    /// the head un-armable until somebody cycles the power.
    #[test]
    fn the_health_gate_refuses_a_head_servo_and_names_an_antenna() {
        let cfg = config();
        let healthy = Rail {
            voltages: [7.4; ROW_COUNT],
            health: [ServoHealth::default(); ROW_COUNT],
        };
        assert_eq!(
            engage_gates(&cfg, &healthy).expect("nothing flags"),
            JointFlags::NONE
        );

        // A leg and an antenna together: the head decides, and it refuses.
        let mut both = healthy;
        both.health[3] = ServoHealth { id: 13, bits: 0x20 };
        both.health[8] = ServoHealth { id: 18, bits: 0x20 };
        let error = engage_gates(&cfg, &both).expect_err("a crank that has flagged refuses");
        let SeqError::UnhealthyServo { context, bits } = error else {
            panic!("expected a health refusal, got {error}");
        };
        assert_eq!(context.id, 13);
        assert_eq!(bits, 0x20);

        // Body yaw is a head joint like the six cranks.
        let mut yaw = healthy;
        yaw.health[0] = ServoHealth { id: 10, bits: 0x20 };
        assert!(
            engage_gates(&cfg, &yaw).is_err(),
            "the body yaw carries too"
        );

        // Both antennas flagging is still an engagement, with both named.
        let mut pair = healthy;
        pair.health[7] = ServoHealth { id: 17, bits: 0x20 };
        pair.health[8] = ServoHealth { id: 18, bits: 0x10 };
        let degraded = engage_gates(&cfg, &pair).expect("the antennas refuse nothing");
        assert!(flags::covers(degraded, JointGroup::Antennas));
        assert_eq!(flags::len(degraded), 2);
    }

    /// A latched antenna is engaged around: no goal, no enable, still measured,
    /// and masked in the state the tick starts from.
    ///
    /// Masked here means limp rather than merely uncommanded, and it means it
    /// by construction: the servo is never enabled in the first place, so
    /// nothing has to be written to take it back out of service.
    #[test]
    fn a_latched_antenna_engages_the_head_and_stays_limp() {
        let cfg = provisioned_config();
        let mut health = [0; ROW_COUNT];
        health[7] = 0x20;
        let mut machine = Machine { health, ..bus() };
        let summary = drive(&cfg, &mut machine).expect("an antenna latch does not refuse the head");

        assert_eq!(
            flags::iter(summary.engage.degraded).collect::<Vec<_>>(),
            vec![JointRef::AntennaRight]
        );
        let mut expected = [true; ROW_COUNT];
        expected[7] = false;
        assert_eq!(machine.enabled(), expected);

        let engaging: Vec<Asked> = machine
            .log
            .iter()
            .filter(|(step, request)| {
                *step == SeqStepKind::PinAndEnable && request.context.id == SERVO_IDS[7]
            })
            .map(|(_, request)| *request)
            .collect();
        assert!(
            engaging
                .iter()
                .all(|request| request.op == AuxOpKind::ReadReg),
            "a joint out of service is written nothing: {engaging:?}"
        );
        // Read exactly once, by the settle sweep: where a limp joint sits is a
        // measurement like any other, and the armed record wants it.
        assert_eq!(engaging.len(), 1);
        assert_eq!(
            summary.engage.armed.joints.antennas[0],
            machine.present.antennas[0]
        );

        // Nothing commands or checks it from here: the tick starts masked.
        let mut slot = crate::tick::MotionSnapWire::new();
        let state = slot.clear_valid();
        crate::tick::arm(state, &summary.engage.armed, summary.engage.degraded);
        assert!(flags::contains(state.masked, JointRef::AntennaRight));
        assert!(!flags::contains(state.masked, JointRef::AntennaLeft));

        // Both of them flagging leaves seven servos holding the head up.
        let mut health = [0; ROW_COUNT];
        health[7] = 0x20;
        health[8] = 0x20;
        let mut machine = Machine { health, ..bus() };
        let summary = drive(&cfg, &mut machine).expect("a flagging pair does not refuse the head");
        assert!(flags::covers(summary.engage.degraded, JointGroup::Antennas));
        assert_eq!(machine.enabled()[..7], [true; 7]);
        assert_eq!(machine.enabled()[7..], [false; 2]);
    }

    /// A degradation lasts the engagement, and the next one judges the bits
    /// again.
    ///
    /// Nothing clears one in session — there is no clear-fault command in this
    /// stack by design — and nothing needs to: the gate reads the latched bits
    /// every time it runs, so an antenna whose latch a power cycle or a REBOOT
    /// cleared is back in service at the next wake, and one whose latch stands
    /// is not.
    #[test]
    fn a_degradation_is_judged_again_at_the_next_engagement() {
        let cfg = provisioned_config();
        let mut health = [0; ROW_COUNT];
        health[8] = 0x20;
        let mut machine = Machine { health, ..bus() };
        let first = drive(&cfg, &mut machine).expect("the head engages");
        assert!(flags::contains(
            first.engage.degraded,
            JointRef::AntennaLeft
        ));

        // A latch that still stands degrades the same joint again.
        machine.torque = [0; ROW_COUNT];
        let again = drive(&cfg, &mut machine).expect("the head engages again");
        assert_eq!(again.engage.degraded, first.engage.degraded);

        // What a REBOOT leaves behind: no latch, and the joint back in service.
        machine.health[8] = 0;
        machine.torque = [0; ROW_COUNT];
        let cleared = drive(&cfg, &mut machine).expect("the head engages once more");
        assert_eq!(cleared.engage.degraded, JointFlags::NONE);
        assert_eq!(machine.enabled(), [true; ROW_COUNT]);
    }

    /// A position that comes back unplaceable is read again before it refuses.
    ///
    /// One corrupt frame is not a servo that has stopped reporting where it is,
    /// and this sweep is the one standing between a wake word and the head
    /// moving. Persistence still refuses: the pose is what everything else is
    /// planned from.
    #[test]
    fn a_position_nobody_can_place_is_read_again_before_it_refuses() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let commission = commission(&cfg, &mut machine).expect("commissioning passes");
        let spent = usize::try_from(PLACE_REREADS).expect("a small count");

        // One bad frame each on three joints, counted rather than derived from
        // the bound: a sweep that re-read nothing refuses on the first of them,
        // and one whose count the landing reading does not reset refuses on the
        // third. The allowance belongs to the joint being read.
        for row in [2, 4, 6] {
            machine.nan_reads[row] = 1;
        }
        machine.log.clear();
        let posture = poll(&cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect("a corrupt frame does not cost the sweep");
        assert_eq!(posture.present.legs[3], machine.present.legs[3]);
        for row in [2, 4, 6] {
            assert_eq!(reads_of(&machine.log, SERVO_IDS[row]), 2, "servo row {row}");
        }
        assert_eq!(reads_of(&machine.log, SERVO_IDS[5]), 1, "a clean joint");

        // One frame more than the sweep will re-read, and it refuses: a joint
        // answering nothing placeable is not a pose to plan from.
        machine.nan_reads[4] = spent + 1;
        machine.log.clear();
        let error = poll(&cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect_err("a joint that keeps answering nothing placeable is refused");
        let SeqError::UnplaceableAngle { joint, .. } = error else {
            panic!("expected an unplaceable reading, got {error}");
        };
        assert_eq!(joint, JointRef::Leg3);
        assert_eq!(reads_of(&machine.log, SERVO_IDS[4]), 1 + spent);
    }

    /// The settle sweep re-reads on the same terms, and for a sharper reason:
    /// a refusal there is a machine already holding torque, taken straight back
    /// off again over one bad frame.
    #[test]
    fn a_settle_frame_nobody_can_place_is_read_again() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let commission = commission(&cfg, &mut machine).expect("commissioning passes");
        let posture = poll(&cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect("the sweep reads");
        machine.nan_reads[8] = 1;
        machine.log.clear();
        let summary = engage(&cfg, &mut machine, &posture)
            .expect("a corrupt settle frame does not cost the engagement");
        assert_eq!(
            summary.armed.joints.antennas[1],
            machine.present.antennas[1]
        );
        assert_eq!(reads_of(&machine.log, SERVO_IDS[8]), 2);
        assert_eq!(machine.enabled(), [true; ROW_COUNT]);

        // And persistence there still refuses, with the machine holding torque
        // — the caller's release is what answers that, not the sweep.
        machine.nan_reads[8] = usize::try_from(PLACE_REREADS).expect("a small count") + 1;
        let error = engage(&cfg, &mut machine, &posture)
            .expect_err("a joint that keeps answering nothing placeable is refused");
        assert!(
            matches!(
                error,
                SeqError::UnplaceableAngle {
                    joint: JointRef::AntennaLeft,
                    ..
                }
            ),
            "{error}"
        );
    }

    /// The slow cadence reads the rail itself; the fast one carries the last
    /// reading forward. Both produce a posture a gate can judge.
    #[test]
    fn a_rail_sweep_reads_what_a_positions_sweep_carries() {
        let cfg = config();
        let mut machine = Machine {
            sweeps: vec![7.1],
            ..bus()
        };
        let carried = Rail {
            voltages: [6.9; ROW_COUNT],
            health: [ServoHealth { id: 0, bits: 0 }; ROW_COUNT],
        };

        let fast = poll(&cfg, &mut machine, carried, PollCadence::Positions)
            .expect("a positions sweep reads");
        assert_eq!(fast.present, machine.present);
        assert_eq!(fast.rail, carried);
        assert!(!fast.rail_read);
        assert_eq!(machine.log.len(), ROW_COUNT);

        machine.log.clear();
        let slow = poll(&cfg, &mut machine, carried, PollCadence::PositionsAndRail)
            .expect("a rail sweep reads");
        assert_eq!(slow.present, machine.present);
        assert_eq!(slow.rail.voltages, [7.1; ROW_COUNT]);
        assert_eq!(slow.rail.health[8], ServoHealth { id: 18, bits: 0 });
        assert!(slow.rail_read);
        assert_eq!(machine.log.len(), 3 * ROW_COUNT);
    }

    /// A hand moves the head while the machine rests, and the next sweep sees
    /// it: the pose engaging plans from is the one that was last measured, not
    /// the one commissioning met.
    #[test]
    fn a_poll_after_the_head_moves_is_what_engaging_plans_from() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let commission = commission(&cfg, &mut machine).expect("commissioning passes");

        let moved = joints_at(&reachy_kin::neutral_head_pose());
        machine.present = moved;
        let posture = poll(&cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect("the sweep reads");
        let summary = engage(&cfg, &mut machine, &posture).expect("a moved head is not a refusal");

        assert_eq!(summary.rest.joints, moved);
        assert_eq!(summary.armed.joints, moved);
    }

    /// Torque coming on can renormalise a servo's reported position onto a
    /// single turn. That is absorbed rather than refused: the pose the tick
    /// starts from is read back after the enables, and the shift is recorded.
    #[test]
    fn a_position_that_jumps_when_torque_comes_on_is_absorbed_and_recorded() {
        let cfg = provisioned_config();
        let mut present = joints_at(&rest_head_pose());
        present.body_yaw = 0.35;
        // An antenna settled past the half turn, which is where this platform's
        // own park leaves them, and which is the reading a renormalisation moves
        // by a whole turn.
        present.antennas = [0.20, -0.15 - core::f64::consts::TAU];
        let mut enable_shift = [0.0; ROW_COUNT];
        enable_shift[8] = core::f64::consts::TAU;
        let mut machine = Machine {
            present,
            enable_shift,
            ..bus()
        };
        let engage = drive(&cfg, &mut machine)
            .expect("a renormalised antenna engages")
            .engage;

        assert!((engage.post_enable_shift[8] - core::f64::consts::TAU).abs() < 1e-12);
        assert_eq!(engage.post_enable_shift[..8], [0.0; 8]);
        // Where the poll found it, what was pinned there, and where it reported
        // itself once torque was on. The pin went out in the frame the joint was
        // measured in; the armed record — what the tick starts from — is the
        // frame the servo renormalised onto, which is the whole reason the pose
        // is read back after the enables rather than assumed.
        assert_eq!(engage.rest.joints.antennas[1], present.antennas[1]);
        assert_eq!(engage.pins.pinned.antennas[1], present.antennas[1]);
        assert!((engage.armed.joints.antennas[1] - (-0.15)).abs() < 1e-12);
    }

    /// A machine found still holding torque is engaged at where it stands.
    ///
    /// Nothing discovers the torque state any more: the resting state is torque
    /// off, so engaging a machine that is already holding is not the ordinary
    /// case, and paying nine reads on every wake to detect it would be. The pins
    /// are the positions the poll measured — a servo sagging under load pins at
    /// its sag, which walks the target down by that sag each time it happens —
    /// and nothing about that is refused.
    #[test]
    fn a_machine_found_holding_torque_engages_at_the_positions_it_reports() {
        let cfg = provisioned_config();
        let sag = 0.5_f64.to_radians();
        let mut machine = holding(sag);
        let measured = machine.present;
        let summary = drive(&cfg, &mut machine).expect("a machine already holding engages");

        assert_eq!(summary.posture.present, measured);
        assert_eq!(summary.engage.rest.joints, measured);
        assert_eq!(summary.engage.pins.pinned, measured);
        assert_eq!(machine.enabled(), [true; ROW_COUNT]);
        // The walk-down, measured: the goal that went out is the sag below where
        // the machine was already being held.
        assert!((angle_at(&machine.held, 1) - angle_at(&machine.goals, 1) - sag).abs() < 1e-12);
    }

    /// Every servo is pinged before anything is decided, and the refusal names
    /// all of the silent ones. Nine silent servos are reported as exactly that.
    #[test]
    fn silent_servos_are_all_named_after_every_ping() {
        let cfg = provisioned_config();
        let mut machine = bus();
        machine.silent[3] = true;
        machine.silent[6] = true;
        let error = drive(&cfg, &mut machine).expect_err("two servos are silent");
        let SeqError::AbsentServos { absent, .. } = error else {
            panic!("expected an absence refusal, got {error}");
        };
        assert_eq!(absent.ids(), [13, 16]);
        assert_eq!(absent.count(), 2);
        assert_eq!(machine.log.len(), ROW_COUNT);
        assert_eq!(
            error.to_string(),
            "presence of servo 13: no answer from servos 13, 16"
        );

        let mut machine = Machine {
            silent: [true; ROW_COUNT],
            ..bus()
        };
        let error = drive(&cfg, &mut machine).expect_err("nothing answers");
        assert_eq!(
            error.to_string(),
            "presence of servo 10: no answer from all nine servos"
        );

        // One servo unplugged is the ordinary bring-up observation, and it reads
        // as one servo rather than as a list of one.
        let mut machine = bus();
        machine.silent[3] = true;
        let error = drive(&cfg, &mut machine).expect_err("one servo is silent");
        let SeqError::AbsentServos { absent, .. } = error else {
            panic!("expected an absence refusal, got {error}");
        };
        assert_eq!(absent.ids(), [13]);
        assert_eq!(absent.count(), 1);
        assert_eq!(
            error.to_string(),
            "presence of servo 13: no answer from servo 13"
        );
    }

    /// A commissioning that has passed the supply gate resumes.
    ///
    /// The gate's two fields belong to the gate, and a slot carrying either one
    /// in a later phase is a slot [`CommissionSequencer::resume`] refuses. A
    /// sequence that polled the rail on a machine whose clock is not zero is the
    /// case that catches a gate that does not leave its fields behind.
    #[test]
    fn a_commissioning_past_the_supply_gate_resumes() {
        let cfg = provisioned_config();
        let mut machine = Machine {
            sweeps: vec![5.0, 5.9, 7.4],
            ..bus()
        };
        let mut slot = CommissionSnapWire::new();
        crate::testutil::drive(
            &mut CommissionSequencer::start(&cfg, &mut slot),
            &mut machine,
        )
        .expect("the rail comes up");
        assert_eq!(machine.waits, 2, "the clock moved off zero");

        let state = slot.validate_mut().expect("a commissioning is written");
        assert_eq!(state.phase, CommissionPhaseKind::Complete);
        assert_eq!(state.voltage_started.as_nanos(), 0);
        assert!(!bool::from(state.voltage_waiting));
        assert!(
            CommissionSequencer::resume(&cfg, state).is_ok(),
            "a state a sequence reached resumes"
        );
    }

    /// A moment the slot cannot hold stops the commissioning at the gate that
    /// measures from it.
    ///
    /// The supply gate's budget is measured from a nanosecond count, and a
    /// process whose clock is past what that count reaches has no moment to
    /// measure from. Recording a saturated zero instead would measure the budget
    /// from the epoch — a gate that either expires instantly or never does.
    #[test]
    fn a_clock_past_what_the_slot_holds_stops_the_commissioning() {
        let cfg = config();
        let mut machine = bus();
        let mut slot = CommissionSnapWire::new();
        let mut seq = CommissionSequencer::start(&cfg, &mut slot);
        // Past the count's reach, which is around 292 years of uptime.
        let now = Duration::from_secs(1 << 40);
        let mut prior = None;
        for _ in 0..STEPS_TO_THE_GATE {
            match seq.next(now, prior.as_ref()) {
                SeqAction::Transact => {
                    let step = seq.step();
                    prior = Some(machine.answer(step, seq.pending()));
                }
                SeqAction::Fail(SeqError::ClockOutOfRange { context }) => {
                    assert_eq!(context.step, SeqStepKind::VoltageGate);
                    assert_eq!(context.id, 0, "the clock belongs to no servo");
                    assert_eq!(context.reg, None);
                    return;
                }
                other => panic!("expected the gate to refuse the clock, got {other:?}"),
            }
        }
        panic!("the supply gate is reached inside the presence and identity sweeps");
    }

    /// A gate resumed from a slot whose start moment is not a length of time
    /// stops rather than waiting from the epoch.
    ///
    /// The count is the state, so this is the one clock refusal a slot written
    /// by something else reaches: the gate reads the moment back before it
    /// judges the budget.
    #[test]
    fn a_gate_started_at_no_moment_stops_the_commissioning() {
        let cfg = provisioned_config();
        let mut machine = Machine {
            sweeps: vec![5.0],
            ..bus()
        };
        let mut slot = CommissionSnapWire::new();
        let state = slot.clear_valid();
        state.phase = CommissionPhaseKind::Voltage;
        state.voltage_started = SyncTime::from_nanos(-1);
        let mut seq =
            CommissionSequencer::resume(&cfg, state).expect("the gate owns those two fields");
        let mut prior = None;
        for _ in 0..STEPS_TO_THE_GATE {
            match seq.next(Duration::ZERO, prior.as_ref()) {
                SeqAction::Transact => {
                    let step = seq.step();
                    prior = Some(machine.answer(step, seq.pending()));
                }
                SeqAction::Fail(SeqError::ClockOutOfRange { context }) => {
                    assert_eq!(context.step, SeqStepKind::VoltageGate);
                    assert_eq!(context.id, 0, "the clock belongs to no servo");
                    return;
                }
                other => panic!("expected the gate to refuse the moment, got {other:?}"),
            }
        }
        panic!("the gate judges the rail inside one sweep of it");
    }

    /// A supply reading that is not a number does not pass the gate.
    ///
    /// The commissioning and the poll hold no float a resumed sequence hands to
    /// the bus or plans a write from — every value they write comes from the
    /// configuration — so their `resume` reads none. The one float either of
    /// them judges is the rail, and the bound test counts a reading nobody can
    /// place as "not up" rather than letting it pass every comparison by
    /// default. That is the property the audit rests on, so it is pinned here.
    #[test]
    fn a_supply_reading_that_is_no_number_does_not_pass_the_gate() {
        let cfg = provisioned_config();
        let mut machine = Machine {
            sweeps: vec![f64::NAN],
            ..bus()
        };
        let error = commission(&cfg, &mut machine).expect_err("a reading nobody can place is low");
        let SeqError::VoltageLow { lowest, .. } = error else {
            panic!("expected the supply gate to refuse, got {error}");
        };
        assert!(lowest.is_nan());
    }

    /// Steps enough for the sweeps that stand between a fresh commissioning and
    /// its supply gate, with room to spare: a loop that runs out of them has not
    /// reached the gate at all, which is a different failure from the one under
    /// test.
    const STEPS_TO_THE_GATE: usize = 64;

    /// Entering a phase leaves the re-read count behind, in both sweeps that
    /// keep one.
    ///
    /// The count belongs to the phase spending it — the attempts made on the
    /// joint at the cursor — so a phase entered with a spent budget would refuse
    /// the first joint it re-reads. Asserted at the seam rather than end to end:
    /// the count is reset on every successful read today, which is exactly why a
    /// change to that rule could break this quietly.
    #[test]
    fn entering_a_phase_leaves_no_re_read_count_behind() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let commission = commission(&cfg, &mut machine).expect("this machine commissions");

        let mut slot = PollSnapWire::new();
        let mut sweep = PollSequencer::start(
            &cfg,
            &mut slot,
            commission.rail,
            PollCadence::PositionsAndRail,
        );
        sweep.state.rereads = PLACE_REREADS;
        sweep.enter(PollPhaseKind::Voltage);
        assert_eq!(sweep.state.rereads, 0);
        let state = slot.validate_mut().expect("the sweep wrote its state");
        assert_eq!(state.phase, PollPhaseKind::Voltage);
        assert!(
            PollSequencer::resume(&cfg, state).is_ok(),
            "a state a sweep reached resumes"
        );

        let posture = poll(&cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect("the sweep reads");
        let geom = HeadGeometry::default();
        let fk = FkOptions::default();
        let mut slot = EngageSnapWire::new();
        let mut engagement = EngageSequencer::start(&cfg, &geom, &fk, &mut slot, &posture);
        engagement.state.rereads = PLACE_REREADS;
        engagement.enter(EngagePhaseKind::Settle);
        assert_eq!(engagement.state.rereads, 0);
        let state = slot.validate_mut().expect("the engagement wrote its state");
        assert_eq!(state.phase, EngagePhaseKind::Settle);
        assert!(
            EngageSequencer::resume(&cfg, &geom, &fk, state).is_ok(),
            "a state an engagement reached resumes"
        );
    }

    /// The one failure whose evidence will not cross is still reported as
    /// itself.
    ///
    /// A verdict that leaves the slot blank is the only case where the slot and
    /// the failure disagree, so the error is answered by the stop rather than
    /// read back out of the fields — what the fields would answer is the
    /// substitute, which names neither the failure nor what it saw.
    #[test]
    fn a_verdict_that_will_not_cross_is_still_reported_as_itself() {
        let cfg = provisioned_config();
        let mut slot = CommissionSnapWire::new();
        let mut seq = CommissionSequencer::start(&cfg, &mut slot);
        let context = StepContext::reg(SeqStepKind::Provision, 11, RegId::HomingOffset);
        let reported = seq.fail(SeqError::VerifyMismatch {
            context,
            expected: value::radians(f64::NAN),
            read_back: value::radians(0.0),
        });
        assert_eq!(reported.kind(), SeqFailureKind::VerifyMismatch);
        assert_eq!(reported.context(), context);

        // What the slot can answer, for contrast: a verdict this build cannot
        // read, at no phase and on no servo.
        assert!(
            matches!(seq.verdict(), SeqError::VerdictUnreadable { .. }),
            "a blank slot answers no failure"
        );
        let state = slot.validate().expect("the stop is written down");
        assert_eq!(state.phase, CommissionPhaseKind::Failed);
        assert_eq!(state.failure.kind, SeqFailureKind::None);
    }

    /// The gate re-reads the supply at its poll spacing until every servo is
    /// above the floor, and gives up when the budget runs out with every reading
    /// in hand.
    #[test]
    fn the_supply_gate_polls_until_the_rail_is_up() {
        let cfg = provisioned_config();
        let mut machine = Machine {
            sweeps: vec![5.0, 5.9, 7.4],
            ..bus()
        };
        let summary = drive(&cfg, &mut machine).expect("the rail comes up");
        assert_eq!(summary.commission.voltage_polls, 3);
        assert_eq!(machine.waits, 2);
        assert_eq!(summary.commission.rail.voltages, [7.4; ROW_COUNT]);

        let mut machine = Machine {
            sweeps: vec![5.0],
            ..bus()
        };
        let error = drive(&cfg, &mut machine).expect_err("the rail never comes up");
        let SeqError::VoltageLow {
            readings,
            lowest,
            waited,
            ..
        } = error
        else {
            panic!("expected a supply refusal, got {error}");
        };
        assert_eq!(readings, [5.0; ROW_COUNT]);
        assert_eq!(lowest, 5.0);
        assert!(waited >= cfg.voltage_budget);
        assert_eq!(
            machine.waits,
            (cfg.voltage_budget.as_millis() / cfg.voltage_poll_period.as_millis()) as usize
        );
        assert_eq!(writes(&machine.log, RegId::PositionGains), 0);
    }

    /// A measured position that is not a number stops the sweep where it is
    /// read.
    ///
    /// It closes no linkage, sits inside no travel window and would become a
    /// goal that means nothing, so it is refused at the sweep rather than
    /// carried into the solver — which would report the same machine as a pose
    /// nobody can place and name the wrong servo.
    #[test]
    fn a_reading_nobody_can_place_stops_the_poll() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let commission = commission(&cfg, &mut machine).expect("commissioning passes");
        machine.present.legs[3] = f64::NAN;
        machine.log.clear();

        let error = poll(&cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect_err("a leg that reads as not-a-number is not a pose");
        let SeqError::UnplaceableAngle {
            context,
            joint,
            angle,
        } = error
        else {
            panic!("expected an unplaceable reading, got {error}");
        };
        assert_eq!(joint, JointRef::Leg3);
        assert_eq!(context.step, SeqStepKind::PoseAndDatum);
        assert_eq!(context.id, SERVO_IDS[4]);
        assert_eq!(context.reg, Some(RegId::PresentPosition));
        assert!(angle.is_nan(), "{angle}");
        assert!(
            !machine
                .log
                .iter()
                .any(|(_, request)| request.op == AuxOpKind::WriteRegVerified),
            "the refusal came after writing to the machine"
        );
    }

    /// Each commissioning check's own refusal, and the property they share: a
    /// machine that fails one is never written to and never has its torque
    /// enabled.
    #[test]
    fn a_failed_check_stops_commissioning_with_nothing_written() {
        let cfg = provisioned_config();
        let cases = [
            bus().provisioned_as(RegId::OperatingMode, value::u8(1)),
            Machine {
                models: [1200, 1190, 1190, 1191, 1190, 1190, 1190, 1180, 1180],
                ..bus()
            },
        ];
        for mut machine in cases {
            let error = drive(&cfg, &mut machine).expect_err("this machine does not commission");
            assert!(
                !machine
                    .log
                    .iter()
                    .any(|(_, request)| request.op == AuxOpKind::WriteRegVerified),
                "{error} was raised after writing to the machine"
            );
            assert_eq!(machine.enabled(), [false; ROW_COUNT]);
        }

        let mut machine = bus().provisioned_as(RegId::OperatingMode, value::u8(1));
        assert_eq!(
            drive(&cfg, &mut machine)
                .expect_err("position mode is not optional")
                .to_string(),
            "provisioning of servo 10, operating mode: provisioned as 3, holds 1"
        );

        // A supply dip the servo rode out is recorded and engaged through: it is
        // the one bit that means the platform is fine.
        let mut machine = Machine {
            health: [1; ROW_COUNT],
            ..bus()
        };
        let summary = drive(&cfg, &mut machine).expect("a voltage latch is not a fault");
        assert_eq!(
            summary.commission.rail.health[4],
            ServoHealth { id: 14, bits: 1 }
        );
    }

    /// Angles nobody can place a pose from stop the engage, named against the
    /// legs they came from.
    #[test]
    fn a_measured_pose_that_closes_no_loop_stops_the_engage() {
        let cfg = provisioned_config();
        let mut machine = bus();
        machine.present.legs[5] += 1.0;
        let error = drive(&cfg, &mut machine).expect_err("those angles close no loop");
        assert!(
            matches!(error, SeqError::RestPoseImplausible { .. }),
            "{error}"
        );
        assert_eq!(error.context().step, SeqStepKind::PinAndEnable);
        // The gates and the solve both run before the first write, so a machine
        // whose pose places nothing is left exactly as it was found.
        assert_eq!(writes(&machine.log, RegId::GoalPosition), 0);
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), 0);
        assert_eq!(machine.enabled(), [false; ROW_COUNT]);
    }

    /// The pose the record holds is the one the angles put the head at, to the
    /// solver's own tolerance. Checked against an independent measure: the
    /// residual of the loop closure the solver was minimising.
    #[test]
    fn the_record_pose_closes_the_linkage() {
        let geom = HeadGeometry::default();
        let opts = FkOptions::default();
        let pose = rest_head_pose();
        let joints = joints_at(&pose);
        let record = ArmRecord::solve(&geom, &opts, &joints, &[pose]).expect("it solves");
        let mut angles = LegAngles([0.0; 6]);
        inverse_kinematics(&geom, &record.head_pose_body, &mut angles).expect("reachable");
        for (leg, (solved, given)) in angles.0.iter().zip(joints.legs).enumerate() {
            assert!(
                (solved - given).abs() < 1e-9,
                "leg {} round trips to {solved} from {given}",
                leg + 1
            );
        }
        let axis_gap = (record.head_pose_body.rotation.inverse() * pose.rotation)
            .angle()
            .abs();
        assert!(axis_gap < 1e-9, "the orientation moved by {axis_gap} rad");
    }

    /// A body a hand has turned past its cap is armed where it stands.
    ///
    /// The measured pose is physical reality; the envelope fences commanded
    /// targets, which is the tick's job on every move afterwards. Refusing here
    /// would leave the head limp in the very pose somebody needs it moved out
    /// of, so arming measures it, takes hold of it, and hands the caller a
    /// record saying where it is.
    #[test]
    fn a_body_turned_past_its_cap_is_armed_where_it_stands() {
        let cfg = provisioned_config();
        let env = EnvelopeConfig::default();

        let mut machine = bus();
        let turned = env.body_yaw_limit + 0.05;
        machine.present.body_yaw = turned;
        let engage = drive(&cfg, &mut machine)
            .expect("reality is not refusable")
            .engage;

        assert_eq!(engage.rest.joints.body_yaw, turned);
        assert_eq!(engage.armed.joints.body_yaw, turned);
        assert_eq!(engage.pins.pinned.body_yaw, turned);
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), ROW_COUNT);
        assert_eq!(writes(&machine.log, RegId::GoalPosition), ROW_COUNT);

        // A rest tighter than the clearance floor engages too, and the record
        // carries the margin it was found at.
        let engage = drive(&cfg, &mut bus())
            .expect("a tight rest is not a refusal")
            .engage;
        assert!(engage.armed.min_margin < env.min_toggle_margin);
    }

    /// The whole sequence over a machine found parked: both antennas resting
    /// past the half turn, which is where the platform's own stow leaves them,
    /// and one of them a whole turn out. Arming records them, writes those very
    /// angles as goals, and completes — nothing is brought inside anything.
    #[test]
    fn a_machine_parked_past_the_half_turn_arms_at_its_readings() {
        let cfg = provisioned_config();

        for antennas in [
            [rad_from_counts(38), rad_from_counts(4051)],
            [
                rad_from_counts(-202),
                rad_from_counts(4051) + core::f64::consts::TAU,
            ],
        ] {
            let mut machine = bus();
            machine.present.antennas = antennas;
            let engage = drive(&cfg, &mut machine)
                .expect("a parked antenna is not a refusal")
                .engage;

            // Found, pinned and armed are all the same angles: an antenna has
            // nowhere to be pulled to.
            assert_eq!(engage.rest.joints.antennas, antennas);
            assert_eq!(engage.pins.pinned.antennas, antennas);
            assert_eq!(engage.armed.joints.antennas, antennas);
            assert_eq!(writes(&machine.log, RegId::GoalPosition), ROW_COUNT);
            assert_eq!(writes(&machine.log, RegId::TorqueEnable), ROW_COUNT);
        }
    }

    /// A machine that reports angles closing no linkage once its torque is on
    /// is refused there — the trajectory the next move starts from would have
    /// no start.
    ///
    /// The torque stays on. Reaching the minimum risk condition from here is a
    /// caller's decision and a caller's write; nothing in a read-back sequence
    /// is entitled to decide it.
    #[test]
    fn a_pose_read_back_that_closes_no_loop_is_refused() {
        let cfg = provisioned_config();
        let mut machine = bus();
        machine.enable_shift[6] = 1.0;

        let error = drive(&cfg, &mut machine).expect_err("those angles place no pose");
        let SeqError::PinnedPoseUnsolvable { context, .. } = error else {
            panic!("expected an unsolvable-pose refusal, got {error}");
        };
        assert_eq!(context.step, SeqStepKind::PinAndEnable);
        assert_eq!(context.reg, Some(RegId::PresentPosition));
        // Every sweep before it ran in full, which is what makes this the
        // read-back's own refusal rather than an earlier check stopping short.
        assert_eq!(writes(&machine.log, RegId::GoalPosition), ROW_COUNT);
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), ROW_COUNT);
        assert_eq!(machine.enabled(), [true; ROW_COUNT]);
    }

    /// A wrong servo at a roster address is named with what it answered and what
    /// this platform answers, on whichever of the nine it sits.
    #[test]
    fn identity_names_a_servo_of_the_wrong_kind() {
        let cfg = provisioned_config();

        let mut machine = Machine {
            models: [1200, 1200, 1200, 1191, 1200, 1200, 1200, 1190, 1190],
            ..bus()
        };
        let error = drive(&cfg, &mut machine).expect_err("that leg is not a leg servo");
        assert_eq!(
            error,
            SeqError::IdentityMismatch {
                context: StepContext::reg(SeqStepKind::Identity, 13, RegId::ModelNumber),
                model: 1191,
                expected: 1200,
            }
        );
        assert_eq!(
            error.to_string(),
            "identity of servo 13, model number: model 1191, where this platform reports 1200"
        );
        // Stopped at the identity phase: nine pings and nine model reads.
        assert_eq!(machine.log.len(), 2 * ROW_COUNT);

        // An antenna is a different servo from a leg, and its own expectation is
        // what it is held to.
        let mut machine = Machine {
            models: [1200, 1200, 1200, 1200, 1200, 1200, 1200, 1190, 1200],
            ..bus()
        };
        let error = drive(&cfg, &mut machine).expect_err("that antenna is a leg servo");
        assert_eq!(
            error,
            SeqError::IdentityMismatch {
                context: StepContext::reg(SeqStepKind::Identity, 18, RegId::ModelNumber),
                model: 1200,
                expected: 1190,
            }
        );

        // A bus of nine servos that agree with each other but not with this
        // platform fails on the first one.
        let mut machine = Machine {
            models: [1180, 1190, 1190, 1190, 1190, 1190, 1190, 1170, 1170],
            ..bus()
        };
        let error = drive(&cfg, &mut machine).expect_err("none of those is this platform");
        assert_eq!(
            error,
            SeqError::IdentityMismatch {
                context: StepContext::reg(SeqStepKind::Identity, 10, RegId::ModelNumber),
                model: 1180,
                expected: 1200,
            }
        );
    }

    /// A write refused partway through the enables stops there: the servos after
    /// it are never enabled, and the refusal names the one that refused and the
    /// register it refused.
    #[test]
    fn a_write_that_does_not_land_stops_the_sequence_where_it_stood() {
        let cfg = provisioned_config();
        let mut machine = Machine {
            fail_write: Some((
                14,
                RegId::TorqueEnable,
                BusResult::VerifyMismatch {
                    read_back: value::u8(0),
                },
            )),
            ..bus()
        };
        let error = drive(&cfg, &mut machine).expect_err("servo 14 will not take torque");
        let SeqError::VerifyMismatch {
            context,
            expected,
            read_back,
        } = error
        else {
            panic!("expected a verify mismatch, got {error}");
        };
        assert_eq!(context.step, SeqStepKind::PinAndEnable);
        assert_eq!(context.id, 14);
        assert_eq!(context.reg, Some(RegId::TorqueEnable));
        assert_eq!(expected, value::u8(1));
        assert_eq!(read_back, value::u8(0));
        // Torque reached the four servos before it and no further; the ones that
        // took it keep it, because nothing here turns torque off.
        assert_eq!(
            machine.enabled(),
            [true, true, true, true, false, false, false, false, false]
        );

        // The same knob on a read: a servo refusing the resting position read
        // stops the sweep with the status code it sent, before any torque.
        let mut machine = Machine {
            fail_read: Some((
                15,
                RegId::PresentPosition,
                BusResult::ServoError { code: 7 },
            )),
            ..bus()
        };
        let error = drive(&cfg, &mut machine).expect_err("servo 15 refuses the read");
        assert_eq!(
            error,
            SeqError::Refused {
                context: StepContext::reg(SeqStepKind::PoseAndDatum, 15, RegId::PresentPosition),
                code: 7,
            }
        );
        assert_eq!(machine.enabled(), [false; ROW_COUNT]);

        // And a corrupt reply, which is never retried by anything below.
        let mut machine = Machine {
            fail_write: Some((12, RegId::PositionGains, BusResult::WireCorrupt)),
            ..bus()
        };
        let error = drive(&cfg, &mut machine).expect_err("that reply came back mangled");
        assert_eq!(
            error,
            SeqError::WireCorrupt {
                context: StepContext::reg(SeqStepKind::GainsProfiles, 12, RegId::PositionGains),
            }
        );
        assert_eq!(machine.enabled(), [false; ROW_COUNT]);
    }

    /// The armed record is the reading taken after the enables, so a joint whose
    /// reported frame moved when torque came on is recorded where it now says it
    /// is — wherever that lands. Nothing judges it against the envelope: the
    /// fence is on commanded targets, and this is a measurement.
    #[test]
    fn a_pose_that_leaves_the_cap_once_torque_is_on_is_recorded_where_it_lands() {
        let cfg = provisioned_config();
        let env = EnvelopeConfig::default();

        let mut machine = bus();
        // Inside the cap where arming finds it, past the cap once torque is on.
        let found = env.body_yaw_limit - 0.05;
        machine.present.body_yaw = found;
        machine.enable_shift[0] = 0.1;
        let engage = drive(&cfg, &mut machine)
            .expect("the post-enable reading is not judged")
            .engage;

        assert_eq!(engage.rest.joints.body_yaw, found);
        assert_eq!(engage.pins.pinned.body_yaw, found);
        assert!((engage.armed.joints.body_yaw - (found + 0.1)).abs() < 1e-12);
        assert!(engage.armed.joints.body_yaw > env.body_yaw_limit);
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), ROW_COUNT);
        assert_eq!(writes(&machine.log, RegId::GoalPosition), ROW_COUNT);
        assert_eq!(machine.enabled(), [true; ROW_COUNT]);
    }

    /// Each servo is enabled once and pinned once, and the pose is read back
    /// once: twenty-seven transactions, no servo asked twice.
    #[test]
    fn every_servo_is_enabled_once_and_pinned_once() {
        let cfg = provisioned_config();
        let mut machine = bus();
        drive(&cfg, &mut machine).expect("this machine engages");

        assert_eq!(writes(&machine.log, RegId::TorqueEnable), ROW_COUNT);
        assert_eq!(writes(&machine.log, RegId::GoalPosition), ROW_COUNT);
        // One position sweep in this phase: the read-back after the enables,
        // which is what the trajectory starts from. Nothing is read after it.
        let positions = machine
            .log
            .iter()
            .filter(|(step, request)| {
                *step == SeqStepKind::PinAndEnable
                    && request.op == AuxOpKind::ReadReg
                    && request.reg() == Some(RegId::PresentPosition)
            })
            .count();
        assert_eq!(positions, ROW_COUNT);
    }

    /// A provisioning table that asks for nothing reads nothing and goes straight
    /// to the supply gate — which still runs, because every write is ordered
    /// behind it.
    #[test]
    fn a_table_that_asks_for_nothing_skips_the_sweep() {
        let cfg = config();
        assert_eq!(cfg.expected.checks(), 0);
        assert_eq!(cfg.expected.reads(), 0);
        let mut machine = bus();
        let mut commissioned = CommissionSnapWire::new();
        drive_from(&cfg, &mut machine, &mut commissioned).expect("this machine engages");

        assert_eq!(
            cells::recorded(
                &commissioned
                    .validate()
                    .expect("the sequence wrote its state")
                    .provisioned
            ),
            0
        );
        assert!(
            !machine
                .log
                .iter()
                .any(|(step, _)| *step == SeqStepKind::Provision)
        );
        let mut phases: Vec<SeqStepKind> = Vec::new();
        for (step, _) in &machine.log {
            if phases.last() != Some(step) {
                phases.push(*step);
            }
        }
        assert_eq!(
            phases,
            vec![
                SeqStepKind::Presence,
                SeqStepKind::Identity,
                SeqStepKind::VoltageGate,
                SeqStepKind::Health,
                SeqStepKind::GainsProfiles,
                SeqStepKind::PoseAndDatum,
                SeqStepKind::PinAndEnable,
            ]
        );
    }

    /// A driver that runs a transaction and brings nothing back is reporting
    /// silence, and the refusal names the servo and register that were
    /// outstanding rather than wherever a cursor had got to.
    #[test]
    fn a_driver_that_brings_nothing_back_is_silence() {
        let cfg = config();
        let mut slot = CommissionSnapWire::new();
        let mut seq = CommissionSequencer::start(&cfg, &mut slot);
        assert_eq!(seq.next(Duration::ZERO, None), SeqAction::Transact);
        assert_eq!(*seq.pending(), txn::ping(10));
        let SeqAction::Fail(error) = seq.next(Duration::ZERO, None) else {
            panic!("a transaction with no result is not an answer");
        };
        assert_eq!(
            error,
            SeqError::NoAnswer {
                context: StepContext::servo(SeqStepKind::Presence, 10),
            }
        );
        assert_eq!(error.context().reg, None);
    }

    /// The supply refusal names the worst-off servo, not the first one under the
    /// floor: nine servos on different lengths of wiring read differently, and
    /// the whole point of carrying every reading is knowing which one is which.
    #[test]
    fn the_supply_refusal_names_the_lowest_servo() {
        let cfg = provisioned_config();
        let mut sag = [0.0; ROW_COUNT];
        sag[1] = 1.5;
        sag[3] = 2.2;
        let mut machine = Machine { sag, ..bus() };
        let error = drive(&cfg, &mut machine).expect_err("the rail never comes up");
        let SeqError::VoltageLow {
            context,
            readings,
            lowest,
            ..
        } = error
        else {
            panic!("expected a supply refusal, got {error}");
        };
        assert!(
            (readings[1] - 5.9).abs() < 1e-9,
            "servo 11 reads {}",
            readings[1]
        );
        assert_eq!(lowest, readings[3]);
        assert_eq!(context.id, 13);
        assert_eq!(context.step, SeqStepKind::VoltageGate);

        // A reading nobody can place is the lowest there is, even beside a
        // number below the floor.
        let mut sag = [0.0; ROW_COUNT];
        sag[3] = 2.2;
        sag[6] = f64::NAN;
        let mut machine = Machine { sag, ..bus() };
        let error = drive(&cfg, &mut machine).expect_err("one servo reports nothing placeable");
        let SeqError::VoltageLow {
            context, lowest, ..
        } = error
        else {
            panic!("expected a supply refusal, got {error}");
        };
        assert!(lowest.is_nan());
        assert_eq!(context.id, 16);
    }

    // ---- The schema-resident sequencers resumed from their slots ----

    resumed! {
        /// The commissioning, resumed from its slot before every step: `cfg` is
        /// all its `resume` needs.
        struct ResumingCommission { cfg: ArmConfig }
        slot = CommissionSnapWire, summary = CommissionSummary, seq = CommissionSequencer,
        resume(host, state) = CommissionSequencer::resume(host.cfg, state);
    }

    resumed! {
        struct ResumingPoll { cfg: ArmConfig }
        slot = PollSnapWire, summary = Posture, seq = PollSequencer,
        resume(host, state) = PollSequencer::resume(host.cfg, state);
    }

    resumed! {
        /// The engagement, which is resumed against the geometry and the solver
        /// options as well.
        struct ResumingEngage { cfg: ArmConfig, geom: HeadGeometry, fk: FkOptions }
        slot = EngageSnapWire, summary = EngageSummary, seq = EngageSequencer,
        resume(host, state) = EngageSequencer::resume(host.cfg, host.geom, host.fk, state);
    }

    /// Step the commissioning `slot` holds to its end, resuming a sequencer from
    /// the slot before every step.
    ///
    /// The clock starts at `since_boot`: the moment the supply gate measures from
    /// is the moment it was entered, so a host that commissions after any uptime
    /// at all writes a non-zero one, and that is the value a phase which keeps it
    /// leaves behind.
    fn commission_from_slot(
        cfg: &ArmConfig,
        slot: &mut CommissionSnapWire,
        machine: &mut Machine,
        since_boot: Duration,
    ) -> Result<CommissionSummary, SeqError> {
        crate::testutil::drive_from_slot(&ResumingCommission { cfg }, slot, machine, since_boot)
    }

    /// Step the sweep `slot` holds to its end, likewise.
    fn poll_from_slot(
        cfg: &ArmConfig,
        slot: &mut PollSnapWire,
        machine: &mut Machine,
    ) -> Result<Posture, SeqError> {
        crate::testutil::drive_from_slot(&ResumingPoll { cfg }, slot, machine, Duration::ZERO)
    }

    /// Step the engagement `slot` holds to its end, likewise.
    fn drive_from_slot(
        cfg: &ArmConfig,
        geom: &HeadGeometry,
        fk: &FkOptions,
        slot: &mut EngageSnapWire,
        machine: &mut Machine,
    ) -> Result<EngageSummary, SeqError> {
        crate::testutil::drive_from_slot(
            &ResumingEngage { cfg, geom, fk },
            slot,
            machine,
            Duration::ZERO,
        )
    }

    /// Commission, poll and engage `machine` as `drive` does, with all three
    /// sequencers resumed from their state slots before every step.
    ///
    /// A crossing costs each of them the same thing: every step has to leave a
    /// state its own `resume` accepts, so a phase that keeps a field it owns, or
    /// a cursor no sequence reaches, stops the run here rather than on the first
    /// host that puts the sequence down between two transactions.
    fn drive_resumed(cfg: &ArmConfig, machine: &mut Machine) -> Result<Armed, SeqError> {
        let mut commission_slot = CommissionSnapWire::new();
        CommissionSequencer::start(cfg, &mut commission_slot);
        let commission = commission_from_slot(cfg, &mut commission_slot, machine, Duration::ZERO)?;

        let mut poll_slot = PollSnapWire::new();
        PollSequencer::start(cfg, &mut poll_slot, commission.rail, PollCadence::Positions);
        let posture = poll_from_slot(cfg, &mut poll_slot, machine)?;

        let geom = HeadGeometry::default();
        let fk = FkOptions::default();
        let mut slot = EngageSnapWire::new();
        EngageSequencer::start(cfg, &geom, &fk, &mut slot, &posture);
        let engage = drive_from_slot(cfg, &geom, &fk, &mut slot, machine)?;
        Ok(Armed {
            commission,
            posture,
            engage,
        })
    }

    /// The whole torque-on path, crossed at every step, against a machine that
    /// engages: the summaries a slot-crossing host gets are the summaries a host
    /// that held the sequencers in local variables gets.
    #[test]
    fn a_crossed_torque_on_path_reaches_the_same_records() {
        let cfg = provisioned_config();

        let (mut direct, mut crossed) = pair(bus());
        let held = drive(&cfg, &mut direct).expect("this machine engages");
        let restored = drive_resumed(&cfg, &mut crossed).expect("this machine engages");

        assert_eq!(restored.commission, held.commission);
        assert_eq!(restored.posture, held.posture);
        assert_eq!(restored.engage, held.engage);
        assert_eq!(crossed.log, direct.log);
        assert_eq!(crossed.waits, direct.waits);
        assert_eq!(crossed.enabled(), direct.enabled());
    }

    /// A commissioning crossed at every step on a machine that has been up for
    /// an hour: the supply gate's own fields are the ones a crossing loses.
    ///
    /// The gate measures its budget from the moment it was entered, so a host
    /// with any uptime writes a non-zero moment there. A phase that keeps that
    /// moment on the way out leaves a slot `resume` refuses as written by
    /// something else, two thirds of the way through a bring-up — and only a
    /// crossed run from a clock that is not zero sees it.
    #[test]
    fn a_crossed_commissioning_from_a_clock_past_boot_reaches_the_same_records() {
        let cfg = provisioned_config();
        // A rail that comes up over two waits, so the gate is entered, waited
        // in, and left.
        let sag = || Machine {
            sweeps: vec![5.0, 5.9, 7.4],
            ..bus()
        };
        let mut direct = sag();
        let held = crate::testutil::drive(
            &mut CommissionSequencer::start(&cfg, &mut CommissionSnapWire::new()),
            &mut direct,
        )
        .expect("the rail comes up");

        let mut crossed = sag();
        let mut slot = CommissionSnapWire::new();
        CommissionSequencer::start(&cfg, &mut slot);
        let restored =
            commission_from_slot(&cfg, &mut slot, &mut crossed, Duration::from_secs(60 * 60))
                .expect("the rail comes up");

        assert_eq!(restored, held);
        assert_eq!(crossed.log, direct.log);
        assert_eq!(crossed.waits, direct.waits, "the gate waited");
        let state = slot.validate_mut().expect("a commissioning is written");
        assert_eq!(state.phase, CommissionPhaseKind::Complete);
        assert_eq!(state.voltage_started.as_nanos(), 0);
        assert!(!bool::from(state.voltage_waiting));
    }

    /// A sweep crossed at every step reads what one held in a local variable
    /// reads.
    #[test]
    fn a_crossed_sweep_reaches_the_same_posture() {
        let cfg = provisioned_config();

        let carried = Rail {
            voltages: [6.9; ROW_COUNT],
            health: [ServoHealth { id: 0, bits: 0 }; ROW_COUNT],
        };

        let (mut direct, mut crossed) = pair(bus());
        let held = poll(&cfg, &mut direct, carried, PollCadence::PositionsAndRail)
            .expect("this machine answers");

        let mut slot = PollSnapWire::new();
        PollSequencer::start(&cfg, &mut slot, carried, PollCadence::PositionsAndRail);
        let restored = poll_from_slot(&cfg, &mut slot, &mut crossed).expect("this machine answers");

        assert_eq!(restored, held);
        assert_eq!(crossed.log, direct.log);
    }

    /// A machine found already holding torque, and one whose reported frame
    /// jumps a whole turn when torque comes on: the engage's records are
    /// solved poses, and a crossing that lost the resting record would solve
    /// the read-back against the wrong seed.
    #[test]
    fn a_crossed_engage_carries_the_records_the_solve_needs() {
        let cfg = provisioned_config();

        let (mut direct, mut crossed) = pair(holding(0.02));
        let held = drive(&cfg, &mut direct).expect("a machine holding torque engages");
        let restored = drive_resumed(&cfg, &mut crossed).expect("a machine holding torque engages");
        assert_eq!(restored.engage, held.engage);

        let mut shift = [0.0; ROW_COUNT];
        shift[7] = core::f64::consts::TAU;
        let (mut direct, mut crossed) = pair(Machine {
            enable_shift: shift,
            ..bus()
        });
        let held = drive(&cfg, &mut direct).expect("a renormalised frame is recorded");
        let restored = drive_resumed(&cfg, &mut crossed).expect("a renormalised frame is recorded");
        assert_eq!(restored.engage, held.engage);
        assert_eq!(
            restored.engage.post_enable_shift,
            held.engage.post_enable_shift
        );
    }

    /// Every failure the path can stop at, crossed: the verdict a restored
    /// sequence hands back is the same verdict, named against the same servo and
    /// the same phase.
    #[test]
    fn a_crossed_path_fails_with_the_same_verdict() {
        let cfg = provisioned_config();

        // Silence at the presence sweep.
        let (mut direct, mut machine) = pair(bus_with_silence(3));
        let held = drive(&cfg, &mut direct).expect_err("that servo is silent");
        let crossed = drive_resumed(&cfg, &mut machine).expect_err("that servo is silent");
        assert_eq!(crossed, held);

        // A servo of the wrong kind at the identity sweep.
        let models = [1200, 1200, 1200, 1191, 1200, 1200, 1200, 1190, 1190];
        let (mut direct, mut machine) = pair(Machine { models, ..bus() });
        let held = drive(&cfg, &mut direct).expect_err("that leg is not a leg servo");
        let crossed = drive_resumed(&cfg, &mut machine).expect_err("that leg is not a leg servo");
        assert_eq!(crossed, held);

        // A rail that never comes up: the supply gate's polls and its budget are
        // the one piece of clock arithmetic a commission carries across a
        // crossing.
        let (mut direct, mut machine) = pair(Machine {
            sweeps: vec![5.0],
            ..bus()
        });
        let held = drive(&cfg, &mut direct).expect_err("the rail never comes up");
        let crossed = drive_resumed(&cfg, &mut machine).expect_err("the rail never comes up");
        assert_eq!(crossed, held);
        assert_eq!(machine.waits, direct.waits);

        // A gains write that does not land.
        let (mut direct, mut machine) = pair(Machine {
            fail_write: Some((13, RegId::PositionGains, BusResult::NoAnswer)),
            ..bus()
        });
        let held = drive(&cfg, &mut direct).expect_err("that write does not land");
        let crossed = drive_resumed(&cfg, &mut machine).expect_err("that write does not land");
        assert_eq!(crossed, held);

        // An enable that does not land, which is the failure a caller asks
        // `torque_written` about.
        let (mut direct, mut machine) = pair(Machine {
            fail_write: Some((13, RegId::TorqueEnable, BusResult::NoAnswer)),
            ..bus()
        });
        let held = drive(&cfg, &mut direct).expect_err("that enable does not land");
        let crossed = drive_resumed(&cfg, &mut machine).expect_err("that enable does not land");
        assert_eq!(crossed, held);
    }

    /// A machine with one servo silent.
    fn bus_with_silence(row: usize) -> Machine {
        let mut machine = bus();
        machine.silent[row] = true;
        machine
    }

    /// Two copies of one machine: the direct run and the crossed run.
    ///
    /// Built once and cloned rather than written twice, because the comparison
    /// between the two runs means nothing unless both faced the same bus, and
    /// two literals are two things to keep in step.
    fn pair(machine: Machine) -> (Machine, Machine) {
        (machine.clone(), machine)
    }

    /// The supply gate polled across a rail that comes up, and a re-read of a
    /// frame nobody can place: the two states with a counter that only a
    /// crossing can lose.
    #[test]
    fn a_crossed_sweep_keeps_its_counters() {
        let cfg = provisioned_config();

        let (mut direct, mut machine) = pair(Machine {
            sweeps: vec![5.0, 5.9, 7.4],
            ..bus()
        });
        let held = drive(&cfg, &mut direct).expect("the rail comes up");
        let crossed = drive_resumed(&cfg, &mut machine).expect("the rail comes up");
        assert_eq!(
            crossed.commission.voltage_polls,
            held.commission.voltage_polls
        );
        assert_eq!(machine.waits, direct.waits);

        // A position read answered with something nobody can place is re-read
        // before it refuses; the count of re-reads spent is state a crossing has
        // to carry, or the sweep re-reads forever.
        let mut nan_reads = [0; ROW_COUNT];
        nan_reads[4] = 1;
        let mut machine = Machine { nan_reads, ..bus() };
        let commission = commission(&cfg, &mut machine).expect("commissioning reads no positions");
        let posture = poll(&cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect("the re-read lands");
        assert_eq!(reads_of(&machine.log, 14), 2);
        assert!(posture.present.legs[3].is_finite());
    }

    /// A rail sweep and a positions sweep: which of the two ran is what the
    /// finished posture reports, and the sweep that did not re-read the rail
    /// carries the last reading forward rather than reporting half a picture.
    #[test]
    fn a_poll_reports_which_sweep_it_was() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let commission = commission(&cfg, &mut machine).expect("this machine commissions");

        let carried = poll(&cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect("the sweep reads nine positions");
        assert!(!carried.rail_read);
        assert_eq!(carried.rail, commission.rail);

        let read = poll(
            &cfg,
            &mut machine,
            commission.rail,
            PollCadence::PositionsAndRail,
        )
        .expect("the sweep reads the rail too");
        assert!(read.rail_read);
    }

    /// A sweep that stopped hands the same verdict back on the next execution.
    ///
    /// The failed phase's whole content is its verdict, and the verdict lives in
    /// the state slot: a sweep resumed from that slot has to hand back what
    /// stopped it rather than a sweep claiming to have finished on partial
    /// readings.
    #[test]
    fn a_stopped_sweep_hands_its_verdict_back_when_it_is_resumed() {
        let cfg = provisioned_config();
        let mut commissioned = bus();
        let rail = commission(&cfg, &mut commissioned)
            .expect("this machine commissions")
            .rail;

        // A servo that stops answering after commissioning.
        let mut machine = bus_with_silence(3);
        let mut slot = PollSnapWire::new();
        let held = crate::testutil::drive(
            &mut PollSequencer::start(&cfg, &mut slot, rail, PollCadence::Positions),
            &mut machine,
        )
        .expect_err("that servo is silent");

        let state = slot.validate_mut().expect("the sweep wrote its state");
        assert_eq!(state.phase, PollPhaseKind::Failed);
        assert_eq!(state.failure.kind, held.kind());
        let mut resumed =
            PollSequencer::resume(&cfg, state).expect("a failed sweep is a reachable state");
        assert_eq!(
            resumed.next(Duration::ZERO, None),
            SeqAction::Fail(held),
            "the resumed sweep hands back the verdict it stopped on"
        );
    }

    /// A joint answering nothing placeable past the re-read budget stops the
    /// sweep, and the re-read count it spent is state the slot carries.
    #[test]
    fn a_reading_nobody_can_place_stops_the_sweep_after_its_re_reads() {
        let cfg = provisioned_config();
        let mut commissioned = bus();
        let rail = commission(&cfg, &mut commissioned)
            .expect("this machine commissions")
            .rail;

        let mut nan_reads = [0; ROW_COUNT];
        nan_reads[4] = usize::try_from(PLACE_REREADS).expect("a small count") + 1;
        let mut machine = Machine { nan_reads, ..bus() };
        let mut slot = PollSnapWire::new();
        let stopped = crate::testutil::drive(
            &mut PollSequencer::start(&cfg, &mut slot, rail, PollCadence::Positions),
            &mut machine,
        )
        .expect_err("that joint answers nothing placeable");
        assert!(
            matches!(stopped, SeqError::UnplaceableAngle { .. }),
            "{stopped}"
        );
        // Every re-read the budget allows was spent on that joint, and one more
        // read than the budget was issued.
        assert_eq!(
            reads_of(&machine.log, 14),
            usize::try_from(PLACE_REREADS).expect("a small count") + 1
        );
    }

    /// An antenna the health gate left out of service: the degraded set is what
    /// the engagement skips its writes for, and a crossing that lost it would
    /// enable a servo the gate refused.
    #[test]
    fn a_crossed_engage_keeps_the_joints_it_left_limp() {
        let cfg = provisioned_config();
        let mut health = [0; ROW_COUNT];
        health[7] = 0x04;

        let (mut direct, mut machine) = pair(Machine { health, ..bus() });
        let held = drive(&cfg, &mut direct).expect("the head engages");
        let crossed = drive_resumed(&cfg, &mut machine).expect("the head engages");
        assert_eq!(crossed.engage.degraded, held.engage.degraded);
        assert!(!flags::is_empty(crossed.engage.degraded));
        assert_eq!(machine.enabled(), direct.enabled());
        assert_eq!(machine.log, direct.log);
    }

    /// A sequence that refused before its first transaction, crossed: the
    /// failure is in the sequencer from the moment it is built, and it survives
    /// a crossing that happens before anything is on the wire.
    #[test]
    fn a_crossed_engage_that_refused_before_transacting_still_refuses() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let commission = commission(&cfg, &mut machine).expect("this machine commissions");
        let mut posture = poll(
            &cfg,
            &mut machine,
            commission.rail,
            PollCadence::PositionsAndRail,
        )
        .expect("the sweep reads");
        // A head servo with bits latched refuses the engagement outright.
        posture.rail.health[3] = ServoHealth { id: 13, bits: 0x04 };

        let geom = HeadGeometry::default();
        let fk = FkOptions::default();
        let mut slot = EngageSnapWire::new();
        {
            let seq = EngageSequencer::start(&cfg, &geom, &fk, &mut slot, &posture);
            assert!(!seq.torque_written());
        }
        let state = slot.validate().expect("a refusal is written down whole");
        assert_eq!(state.phase, EngagePhaseKind::Failed);
        assert_ne!(state.failure.kind, SeqFailureKind::None);

        let error = drive_from_slot(&cfg, &geom, &fk, &mut slot, &mut machine)
            .expect_err("that servo has bits latched");
        assert!(matches!(error, SeqError::UnhealthyServo { .. }), "{error}");
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), 0);
    }

    /// The refusals: a state slot holding a phase and a cursor that no sequence
    /// of steps produces is not resumed into a sequencer that pretends
    /// otherwise.
    #[test]
    fn a_state_no_sequence_reaches_is_refused() {
        let cfg = provisioned_config();

        // A cursor past the sweep it indexes.
        let mut slot = CommissionSnapWire::new();
        let state = slot.clear_valid();
        state.phase = CommissionPhaseKind::Presence;
        state.cursor = 9;
        assert_eq!(
            CommissionSequencer::resume(&cfg, state).err(),
            Some(ResumeError::CursorOutOfRange {
                phase: "presence",
                cursor: 9,
                bound: 9,
            })
        );

        // The zero an unwritten slot holds names no phase, and is refused rather
        // than read as the first sweep.
        let state = slot.clear_valid();
        assert_eq!(
            CommissionSequencer::resume(&cfg, state).err(),
            Some(ResumeError::NoPhase { phase: "no" })
        );

        // A failed phase with nothing in it to fail with.
        let state = slot.clear_valid();
        state.phase = CommissionPhaseKind::Failed;
        assert!(matches!(
            CommissionSequencer::resume(&cfg, state).err(),
            Some(ResumeError::Verdict(_))
        ));

        // A verdict recorded in a phase that is still running.
        let state = slot.clear_valid();
        state.phase = CommissionPhaseKind::Presence;
        verdict::write(
            &mut state.failure,
            &SeqError::NoAnswer {
                context: StepContext::servo(SeqStepKind::Presence, 10),
            },
        )
        .expect("the verdict crosses");
        assert_eq!(
            CommissionSequencer::resume(&cfg, state).err(),
            Some(ResumeError::ErrorWithoutFailedPhase { phase: "presence" })
        );

        // The supply gate's clock and its wait flag, carried by a phase that
        // never writes them. That phase does not hold them, so accepting these
        // would canonicalize a corrupt slot into a clean state instead of
        // reporting it. The two the loop leaves out answer earlier refusals of
        // their own: the zero names no phase, and the failed phase is judged by
        // its verdict.
        for phase in CommissionPhaseKind::VARIANTS {
            if matches!(
                phase,
                CommissionPhaseKind::None
                    | CommissionPhaseKind::Voltage
                    | CommissionPhaseKind::Failed
            ) {
                continue;
            }
            let state = slot.clear_valid();
            state.phase = phase;
            state.voltage_started = SyncTime::from_nanos(1_000_000);
            assert_eq!(
                CommissionSequencer::resume(&cfg, state).err(),
                Some(ResumeError::StrayPhaseField {
                    field: "voltage_started",
                    owner: "voltage",
                    phase: commission_phase_name(phase),
                }),
                "{phase:?}"
            );
            let state = slot.clear_valid();
            state.phase = phase;
            state.voltage_waiting = true.into();
            assert_eq!(
                CommissionSequencer::resume(&cfg, state).err(),
                Some(ResumeError::StrayPhaseField {
                    field: "voltage_waiting",
                    owner: "voltage",
                    phase: commission_phase_name(phase),
                }),
                "{phase:?}"
            );
        }
        // The gate itself takes both, as the phase that writes them.
        let state = slot.clear_valid();
        state.phase = CommissionPhaseKind::Voltage;
        state.voltage_started = SyncTime::from_nanos(1_000_000);
        state.voltage_waiting = true.into();
        assert!(CommissionSequencer::resume(&cfg, state).is_ok());

        // The poll sweep's own refusals: it walks nine servos per phase, its
        // verdict belongs to the failed phase, and that phase needs one.
        let mut slot = PollSnapWire::new();
        let state = slot.clear_valid();
        state.phase = PollPhaseKind::Position;
        state.cursor = 9;
        assert_eq!(
            PollSequencer::resume(&cfg, state).err(),
            Some(ResumeError::CursorOutOfRange {
                phase: "position",
                cursor: 9,
                bound: 9,
            })
        );
        let state = slot.clear_valid();
        state.phase = PollPhaseKind::Position;
        verdict::write(
            &mut state.failure,
            &SeqError::NoAnswer {
                context: StepContext::servo(SeqStepKind::PoseAndDatum, 10),
            },
        )
        .expect("the verdict crosses");
        assert_eq!(
            PollSequencer::resume(&cfg, state).err(),
            Some(ResumeError::ErrorWithoutFailedPhase { phase: "position" })
        );
        let state = slot.clear_valid();
        state.phase = PollPhaseKind::Failed;
        assert!(matches!(
            PollSequencer::resume(&cfg, state).err(),
            Some(ResumeError::Verdict(_))
        ));
        let state = slot.clear_valid();
        assert_eq!(
            PollSequencer::resume(&cfg, state).err(),
            Some(ResumeError::NoPhase { phase: "no" })
        );
    }

    /// The engagement's refusals, over the fields its slot holds.
    #[test]
    fn an_engagement_no_sequence_reaches_is_refused() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let commission = commission(&cfg, &mut machine).expect("this machine commissions");
        let posture = poll(&cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect("the sweep reads");

        let geom = HeadGeometry::default();
        let fk = FkOptions::default();
        let mut slot = EngageSnapWire::new();
        EngageSequencer::start(&cfg, &geom, &fk, &mut slot, &posture);

        // An engagement that says it finished with no record of where it left
        // the machine.
        let state = slot
            .validate_mut()
            .expect("a started engagement is written");
        state.phase = EngagePhaseKind::Complete;
        assert_eq!(
            EngageSequencer::resume(&cfg, &geom, &fk, state).err(),
            Some(ResumeError::CompleteWithoutRecord)
        );

        // A record carried before the settle sweep could have solved one.
        state.phase = EngagePhaseKind::Pin;
        state.armed.present = true.into();
        assert_eq!(
            EngageSequencer::resume(&cfg, &geom, &fk, state).err(),
            Some(ResumeError::RecordWithoutCompletePhase { phase: "pin" })
        );

        // A phase nothing wrote is not read as the goal sweep.
        let state = slot.clear_valid();
        assert_eq!(
            EngageSequencer::resume(&cfg, &geom, &fk, state).err(),
            Some(ResumeError::NoPhase { phase: "no" })
        );
    }

    /// A slot holding an engagement started against a healthy machine.
    ///
    /// The resume-time refusals each spoil one field of this and ask
    /// [`EngageSequencer::resume`], which is the boundary a host crosses.
    fn started_engagement(cfg: &ArmConfig, geom: &HeadGeometry, fk: &FkOptions) -> EngageSnapWire {
        let mut machine = bus();
        let commission = commission(cfg, &mut machine).expect("this machine commissions");
        let posture = poll(cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect("the sweep reads");
        let mut slot = EngageSnapWire::new();
        EngageSequencer::start(cfg, geom, fk, &mut slot, &posture);
        slot
    }

    /// A resume refuses a slot holding no resting record in a phase that plans
    /// from one, and says that rather than saying the record is damaged.
    ///
    /// The two are unrelated causes — a phase reached out of order against a
    /// slot written by something else — and the typed refusal is the only
    /// channel this crate has to tell them apart. The refusal is at the
    /// boundary, before a sequencer that can write exists: a caller holding one
    /// of these slots answers with the immediate release, which takes nothing
    /// from it.
    #[test]
    fn a_missing_resting_record_is_refused_before_the_engagement_runs() {
        let cfg = provisioned_config();
        let geom = HeadGeometry::default();
        let fk = FkOptions::default();
        let mut slot = started_engagement(&cfg, &geom, &fk);
        let state = slot
            .validate_mut()
            .expect("a started engagement is written");
        state.rest.present = false.into();

        assert_eq!(
            EngageSequencer::resume(&cfg, &geom, &fk, state).err(),
            Some(ResumeError::RecordMissing {
                record: "rest",
                phase: "pin",
            })
        );
    }

    /// A record whose quaternion is no rotation is refused at the boundary, in
    /// either of the two slots that hold one: nothing plans a pose out of a
    /// record nobody solved, and a resumed engagement plans every remaining
    /// write from these.
    #[test]
    fn a_record_that_is_no_pose_is_refused_before_the_engagement_runs() {
        let cfg = provisioned_config();
        let geom = HeadGeometry::default();
        let fk = FkOptions::default();
        let mut slot = started_engagement(&cfg, &geom, &fk);
        let state = slot
            .validate_mut()
            .expect("a started engagement is written");
        // The rotation is scaled to something no solve produced, which is the
        // shape a slot nothing wrote has.
        state.rest.head_quat.w *= 2.0;
        assert!(matches!(
            EngageSequencer::resume(&cfg, &geom, &fk, state).err(),
            Some(ResumeError::RecordNotAPose { record: "rest", .. })
        ));

        // The armed record the same way, in the one phase that says it exists:
        // a zeroed quaternion under a set flag is no rotation either.
        let state = slot
            .validate_mut()
            .expect("a started engagement is written");
        state.rest.head_quat.w /= 2.0;
        state.phase = EngagePhaseKind::Complete;
        state.armed.present = true.into();
        assert!(matches!(
            EngageSequencer::resume(&cfg, &geom, &fk, state).err(),
            Some(ResumeError::RecordNotAPose {
                record: "armed",
                ..
            })
        ));
    }

    /// A number that is not a number, in each of the five fields a resumed
    /// engagement plans from.
    ///
    /// The pose read judges the quaternion by its length and nothing else, and
    /// generated validation never inspects a float, so every one of these would
    /// otherwise reach a `GoalPosition` write or a first move's clearance
    /// measurement as a NaN. One row per field, so a check that inspects the
    /// wrong one is a red test rather than a hole.
    #[test]
    fn a_record_or_pin_carrying_no_number_is_refused_before_the_engagement_runs() {
        /// One field of a resumed engagement's numbers: the record it belongs
        /// to, the name the refusal gives it, and the way to spoil it.
        type Spoiler = (&'static str, &'static str, fn(&mut EngageSnap));

        let spoilers: [Spoiler; 5] = [
            ("rest", "joints", |state| {
                state.rest.joints.leg_0 = f64::NAN;
            }),
            ("rest", "head translation", |state| {
                state.rest.head_pos.x = f64::INFINITY;
            }),
            ("rest", "margins", |state| {
                state.rest.margins.leg_4 = f64::NAN;
            }),
            ("pins", "pinned goals", |state| {
                state.pins.pinned.leg_2 = f64::NAN;
            }),
            ("pins", "pull-in figures", |state| {
                state.pins.pull_in.leg_0 = f64::INFINITY;
            }),
        ];
        for (record, field, spoil) in spoilers {
            let cfg = provisioned_config();
            let geom = HeadGeometry::default();
            let fk = FkOptions::default();
            let mut slot = started_engagement(&cfg, &geom, &fk);
            let state = slot
                .validate_mut()
                .expect("a started engagement is written");
            spoil(state);

            assert_eq!(
                EngageSequencer::resume(&cfg, &geom, &fk, state).err(),
                Some(ResumeError::NonFinite { record, field }),
                "{record} {field}"
            );
        }
    }

    /// A pinned goal outside its leg's travel window is refused rather than
    /// placed again: placement only ever produces an in-window pin, so this is
    /// a slot written by something else, and pulling a commanded value to the
    /// window edge is the clamp this repo does not do.
    ///
    /// Both edges are in the window, and that is not a detail: placement pulls
    /// an out-of-window basis angle *to* an edge, so a pin sitting exactly on
    /// one is a value an ordinary pull-in produces and stores. Refusing it
    /// would release a session for no reason.
    #[test]
    fn a_pinned_goal_outside_its_window_is_refused_rather_than_placed_again() {
        let cfg = provisioned_config();
        let geom = HeadGeometry::default();
        let fk = FkOptions::default();

        for leg in 0..joints::LEG_COUNT {
            let (low, high) = cfg.leg_windows[leg];
            for pin in [low - 0.5, high + 0.5] {
                let mut slot = started_engagement(&cfg, &geom, &fk);
                let state = slot
                    .validate_mut()
                    .expect("a started engagement is written");
                pin_leg(&mut state.pins.pinned, leg, pin);

                assert_eq!(
                    EngageSequencer::resume(&cfg, &geom, &fk, state).err(),
                    Some(ResumeError::PinOutOfWindow {
                        leg,
                        pin,
                        low,
                        high,
                    })
                );
            }

            for pin in [low, high] {
                let mut slot = started_engagement(&cfg, &geom, &fk);
                let state = slot
                    .validate_mut()
                    .expect("a started engagement is written");
                pin_leg(&mut state.pins.pinned, leg, pin);

                assert!(
                    EngageSequencer::resume(&cfg, &geom, &fk, state).is_ok(),
                    "leg {leg} pinned on its window edge at {pin}"
                );
            }
        }
    }

    /// Put one crank's pin at `angle`, leaving the other eight rows alone.
    fn pin_leg(slot: &mut joints::Joints, leg: usize, angle: f64) {
        let mut pinned = joints::vector_of(slot);
        pinned.legs[leg] = angle;
        joints::write_vector(slot, &pinned);
    }

    /// A pinned goal for body yaw or an antenna outside what its goal register
    /// represents is refused before the sequencer exists.
    ///
    /// Yaw's is a count bound rather than an interval in radians, and the last
    /// row is why: the half count below +π rounds to a count the register does
    /// not hold, and an interval in radians would let it through to the bus.
    #[test]
    fn a_pin_no_goal_register_holds_is_refused_before_torque_goes_on() {
        let cfg = provisioned_config();
        let geom = HeadGeometry::default();
        let fk = FkOptions::default();
        let count = core::f64::consts::TAU / (YAW_GOAL_COUNT_MAX + 1.0);
        let bottom = -core::f64::consts::PI;
        let top = core::f64::consts::PI - count;
        // Inside the last half count below +pi, which rounds up past the top.
        let sliver = core::f64::consts::PI - count / 4.0;

        for (whose, pin, expected) in [
            (
                "yaw a count below its frame's zero",
                bottom - count,
                Some(ResumeError::YawPinNoCount {
                    pin: bottom - count,
                    counts: -1.0,
                    bound: YAW_GOAL_COUNT_MAX,
                }),
            ),
            (
                "yaw in the half count above its last one",
                sliver,
                Some(ResumeError::YawPinNoCount {
                    pin: sliver,
                    counts: YAW_GOAL_COUNT_MAX + 1.0,
                    bound: YAW_GOAL_COUNT_MAX,
                }),
            ),
            ("yaw on count zero", bottom, None),
            ("yaw on its last count", top, None),
        ] {
            let mut slot = started_engagement(&cfg, &geom, &fk);
            let state = slot
                .validate_mut()
                .expect("a started engagement is written");
            let mut pinned = joints::vector_of(&state.pins.pinned);
            pinned.body_yaw = pin;
            joints::write_vector(&mut state.pins.pinned, &pinned);

            assert_eq!(
                EngageSequencer::resume(&cfg, &geom, &fk, state).err(),
                expected,
                "{whose}"
            );
        }

        for (side, joint) in ["right", "left"].into_iter().enumerate() {
            for (pin, expected) in [
                (
                    ANTENNA_GOAL_MAX_RAD + 1.0,
                    Some(ResumeError::AntennaPinNoCount {
                        joint,
                        pin: ANTENNA_GOAL_MAX_RAD + 1.0,
                        low: ANTENNA_GOAL_MIN_RAD,
                        high: ANTENNA_GOAL_MAX_RAD,
                    }),
                ),
                (
                    ANTENNA_GOAL_MIN_RAD - 1.0,
                    Some(ResumeError::AntennaPinNoCount {
                        joint,
                        pin: ANTENNA_GOAL_MIN_RAD - 1.0,
                        low: ANTENNA_GOAL_MIN_RAD,
                        high: ANTENNA_GOAL_MAX_RAD,
                    }),
                ),
                (ANTENNA_GOAL_MAX_RAD, None),
                (ANTENNA_GOAL_MIN_RAD, None),
            ] {
                let mut slot = started_engagement(&cfg, &geom, &fk);
                let state = slot
                    .validate_mut()
                    .expect("a started engagement is written");
                let mut pinned = joints::vector_of(&state.pins.pinned);
                pinned.antennas[side] = pin;
                joints::write_vector(&mut state.pins.pinned, &pinned);

                assert_eq!(
                    EngageSequencer::resume(&cfg, &geom, &fk, state).err(),
                    expected,
                    "{joint} antenna at {pin}"
                );
            }
        }
    }

    /// A prepare-time failure resumes and reports its verdict.
    ///
    /// That state has no resting record — the refusal happened before one was
    /// solved — and, after a pin refusal, the raw measured angles the start
    /// seeded are still sitting in the pins. It hands nothing to the bus, so
    /// none of the record and pin checks apply to it, and a failed sweep
    /// reaching its caller is the whole point of keeping it resumable.
    #[test]
    fn a_failed_engagement_resumes_and_reports_what_stopped_it() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let commission = commission(&cfg, &mut machine).expect("this machine commissions");
        let mut posture = poll(&cfg, &mut machine, commission.rail, PollCadence::Positions)
            .expect("the sweep reads");
        posture.present.legs[1] = f64::NAN;

        let geom = HeadGeometry::default();
        let fk = FkOptions::default();
        let mut slot = EngageSnapWire::new();
        EngageSequencer::start(&cfg, &geom, &fk, &mut slot, &posture);
        let state = slot
            .validate_mut()
            .expect("a refused engagement is written");
        assert_eq!(state.phase, EngagePhaseKind::Failed);
        assert!(!bool::from(state.rest.present));

        let mut seq = EngageSequencer::resume(&cfg, &geom, &fk, state)
            .expect("a prepare-time failure is a state this sequencer produces");
        let SeqAction::Fail(error) = seq.next(Duration::ZERO, None) else {
            panic!("a failed engagement reports its verdict");
        };
        assert!(
            matches!(error, SeqError::UnplaceableAngle { .. }),
            "{error}"
        );
    }
}
