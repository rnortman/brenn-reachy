//! Taking hold of the platform: commission once, keep the rail picture fresh,
//! engage in one driver cycle.
//!
//! Two sequencers, in the order a process runs them.
//!
//! - [`CommissionSequencer`] runs once, before any torque: every servo present,
//!   every servo the kind it should be, every provisioned register as
//!   configured, the supply rail up, nothing latched, and the gains and profiles
//!   written. Around two hundred transactions, not one of which touches torque.
//! - [`PollSequencer`] is the resting watch, which is the fallback under the
//!   torque-on gate: the supply and the error byte of the rows the driver's
//!   health rotation has left stale, two transactions each, merged into the
//!   picture [`engage_gates`] judges.
//!
//! Taking hold of the machine is neither. [`engage_gates`] is arithmetic over a
//! picture the host keeps, and the writes it clears — the goal registers pinned,
//! torque enabled, the enable read back — are one grouped exchange each in one
//! driver cycle, which is what lets a wake word raise the head.
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
    JointGroup, JointRef, JointVector, ROW_COUNT, ROWS, ServoHealth, flags, group_of, joint_ref,
    leg_index, leg_ref, row,
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
use crate::txn::{self, BusTxnWire};
use crate::value::{self, Value};
use crate::{cells, verdict};
use brenn_reachy__motion__commission_clk_rs::{
    CommissionPhaseKind, CommissionSnap, CommissionSnapWire,
};
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
    /// Bus Watchdog timeout, in the register's 20 ms units. Armed, because a
    /// servo that has stopped beats one chasing a stale goal once its host has
    /// gone quiet; the trip does not release torque on this hardware, which is
    /// observed rather than assumed. The register is RAM-resident and resets at
    /// power-on, which is why it is written per session.
    pub bus_watchdog: u8,
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

/// One entry of the gains-and-profiles sweep's write table: a register, and the
/// value the configured profile gives it.
///
/// A function rather than a value so one register can appear twice with
/// different contents, which is what the watchdog's clear-then-arm pair needs.
pub type ProfileWrite = (RegId, fn(&ProfileConfig) -> Value);

/// The registers the gains-and-profiles sweep writes per servo, in write order,
/// each paired with where its value comes from.
///
/// After the one position-gains write. The Bus Watchdog appears twice: zero
/// first, then the configured timeout. Zero is the vendor's documented clear for
/// a latched watchdog, which otherwise refuses ordinary writes, so the pair arms
/// the register from either state a fresh engagement can find it in. Which error
/// a refusal carries is nothing this sweep reads -- the vendor's manual and the
/// hardware disagree about it, and the byte travels verbatim.
///
/// The sweep's arithmetic and the cursor bound
/// [`GAINS_PROFILE_WRITES`](crate::resume::GAINS_PROFILE_WRITES) both derive
/// from this list, so an entry added here widens both rather than leaving a
/// snapshot refused at cursors the sweep legitimately reaches.
pub const PROFILE_REGS: [ProfileWrite; 4] = [
    (RegId::BusWatchdog, |_| value::u8(0)),
    (RegId::BusWatchdog, |cfg| value::u8(cfg.bus_watchdog)),
    (RegId::ProfileAcceleration, |cfg| {
        value::u32(cfg.acceleration)
    }),
    (RegId::ProfileVelocity, |cfg| value::u32(cfg.velocity)),
];

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
/// what makes engaging fast: the host keeps a picture of these two readings per
/// servo from the driver's health rotation, so the gate is arithmetic instead of
/// eighteen transactions. A row the rotation has left older than the host's
/// staleness window is re-read by the resting watch first, and merged in here.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Rail {
    /// Each servo's supply reading, volts.
    pub voltages: [f64; ROW_COUNT],
    /// Each servo's hardware-error byte, as read.
    pub health: [ServoHealth; ROW_COUNT],
}

/// The two torque-on gates, against the readings the host already has in hand.
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
                    // one row per servo, one column per table entry, in the
                    // table's order — which is what makes the watchdog's clear
                    // land before its arm.
                    let index = cursor - ROW_COUNT;
                    let (reg, value_of) = PROFILE_REGS[index % PROFILE_REGS.len()];
                    let value = value_of(&self.cfg.profile);
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

/// How many servos a poll phase's sweep walks, or `None` for the ending and the
/// phase that names nothing.
///
/// The bound is the whole bus even though a sweep reads only the rows it was
/// handed: the cursor is a bus row and steps over the rows nobody named, so what
/// bounds it is the row count and not the size of the mask.
fn poll_sweep(phase: PollPhaseKind) -> Option<usize> {
    match phase {
        PollPhaseKind::Voltage | PollPhaseKind::Health => Some(ROW_COUNT),
        PollPhaseKind::None | PollPhaseKind::Complete | PollPhaseKind::Failed => None,
    }
}

/// The name a refusal about a poll phase reads under.
fn poll_phase_name(phase: PollPhaseKind) -> &'static str {
    match phase {
        PollPhaseKind::None => "no",
        PollPhaseKind::Voltage => "voltage",
        PollPhaseKind::Health => "health",
        PollPhaseKind::Complete => "complete",
        PollPhaseKind::Failed => "failed",
    }
}

/// The resting watch: a re-read of the supply and the error byte of the rows it
/// was handed.
///
/// The fallback under the torque-on gate, and nothing else. The gate judges the
/// picture the session keeps from the driver's health rotation; a row that
/// rotation has left older than the staleness window, or has never reached, is
/// read here — two transactions, tens of milliseconds — and the gate is judged
/// again. Nothing is written in either direction, and no pose is taken: the
/// engagement pins at the angles the driver's own grouped read returns in the
/// cycle it engages on.
///
/// Nothing here refuses anything about the machine's *state* — a servo with a
/// latched bit is a reading, and what to do about it is the gate's. What does
/// stop a sweep is the bus failing to answer or answering with something that is
/// not a reading.
pub struct PollSequencer<'a> {
    cfg: &'a ArmConfig,
    state: &'a mut PollSnap,
}

impl<'a> PollSequencer<'a> {
    /// A fresh sweep, in `slot`, re-reading `rows` and nothing else.
    ///
    /// The rail it hands back carries a reading for every row it was named for
    /// and zero for the rest, so the caller merges by row rather than taking the
    /// whole picture: what this sweep establishes is the rows it read.
    ///
    /// A mask naming no row ends on the first step with nothing asked for, which
    /// is the honest answer to being asked about nothing.
    pub fn start(cfg: &'a ArmConfig, slot: &'a mut PollSnapWire, rows: JointFlags) -> Self {
        let state = slot.clear_valid();
        state.phase = PollPhaseKind::Voltage;
        state.rows = rows;
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
    /// Neither phase owns one: both read one register of each named row and
    /// carry nothing between them but the readings, which belong to the sweep.
    fn blank_phase_fields(&mut self) {}

    /// Which phase a failure raised here is reported under.
    fn phase_step(&self) -> SeqStepKind {
        match self.state.phase {
            PollPhaseKind::Voltage => SeqStepKind::VoltageGate,
            PollPhaseKind::Health | PollPhaseKind::Complete => SeqStepKind::Health,
            PollPhaseKind::Failed | PollPhaseKind::None => self.state.failure.step,
        }
    }

    /// The readings this sweep took, as the rows the caller merges.
    fn rail(&self) -> Rail {
        cells::rail_of(&self.state.rail)
    }

    /// Where the sweep stands once it is done with the row at its cursor.
    ///
    /// The order of the two reads, in one place: the supply of every named row,
    /// then the error byte of every named row. Two callers step it — the walk
    /// itself, as each answer comes back, and the skip that steps past a row
    /// nobody named — so a second copy of the order is one that could be left
    /// behind by a change to only one of them.
    fn advance(&mut self) {
        let cursor = self.cursor();
        match self.state.phase {
            PollPhaseKind::Voltage if cursor + 1 < ROW_COUNT => self.seek(cursor + 1),
            PollPhaseKind::Voltage => self.enter(PollPhaseKind::Health),
            PollPhaseKind::Health if cursor + 1 < ROW_COUNT => self.seek(cursor + 1),
            PollPhaseKind::Health => self.enter(PollPhaseKind::Complete),
            PollPhaseKind::Complete | PollPhaseKind::Failed | PollPhaseKind::None => {}
        }
    }

    /// Step the cursor past the rows this sweep was not named for.
    ///
    /// A row the rotation's picture is fresh on is not read again: the whole
    /// point of the mask is that the sweep costs two transactions per stale row
    /// rather than eighteen. The walk ends at the completing phase, which is
    /// what stops this for a mask naming no row at all.
    fn skip_unnamed(&mut self) {
        while poll_sweep(self.state.phase).is_some()
            && !flags::contains(self.state.rows, ROWS[self.cursor()])
        {
            self.advance();
        }
    }

    fn read(&mut self, row: usize, reg: RegId) {
        txn::set_read_reg(&mut self.state.pending, self.cfg.ids[row], reg);
    }

    fn emit(&mut self) -> SeqAction<Rail> {
        self.skip_unnamed();
        let cursor = self.cursor();
        match self.state.phase {
            PollPhaseKind::Voltage => self.read(cursor, RegId::PresentInputVoltage),
            PollPhaseKind::Health => self.read(cursor, RegId::HardwareErrorStatus),
            PollPhaseKind::Complete => return SeqAction::Done(self.rail()),
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
            PollPhaseKind::Voltage => {
                absorb_volts(cursor, &mut self.state.rail, context, result)?;
                self.advance();
            }
            PollPhaseKind::Health => {
                absorb_health_bits(
                    cursor,
                    self.cfg.ids[cursor],
                    &mut self.state.rail,
                    context,
                    result,
                )?;
                self.advance();
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

    type Summary = Rail;

    fn next(&mut self, _now: Duration, prior: Option<&BusResult>) -> SeqAction<Rail> {
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
                AuxOpKind::WriteRegVerified | AuxOpKind::WriteReg => self.fail_write,
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
                AuxOpKind::WriteRegVerified | AuxOpKind::WriteReg => {
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

    fn commission(cfg: &ArmConfig, machine: &mut Machine) -> Result<CommissionSummary, SeqError> {
        let mut slot = CommissionSnapWire::new();
        crate::testutil::drive(&mut CommissionSequencer::start(cfg, &mut slot), machine)
    }

    /// Re-read `rows` against `machine`, as the fallback under the gate does.
    fn poll(cfg: &ArmConfig, machine: &mut Machine, rows: JointFlags) -> Result<Rail, SeqError> {
        let mut slot = PollSnapWire::new();
        crate::testutil::drive(&mut PollSequencer::start(cfg, &mut slot, rows), machine)
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

    /// Whether a transaction put a value on the wire, verified or not.
    fn wrote(op: AuxOpKind) -> bool {
        matches!(op, AuxOpKind::WriteRegVerified | AuxOpKind::WriteReg)
    }

    fn writes(log: &[(SeqStepKind, Asked)], reg: RegId) -> usize {
        log.iter()
            .filter(|(_, request)| wrote(request.op) && request.context.reg == Some(reg))
            .count()
    }

    /// The two torque-on gates are the whole enumeration, and they refuse
    /// before a single transaction crosses the wire.
    ///
    /// Judged over the picture the host keeps rather than over a sweep it just
    /// took: the gate is arithmetic, so a refusal costs the bus nothing and
    /// leaves the machine exactly where it was standing.
    #[test]
    fn the_torque_on_gates_refuse_without_touching_the_machine() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let commission = commission(&cfg, &mut machine).expect("commissioning passes");
        machine.log.clear();

        let mut sagging = commission.rail;
        sagging.voltages[4] = 5.5;
        let error = engage_gates(&cfg, &sagging).expect_err("a low rail does not torque on");
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
            "the gate's refusal reads as a wait that expired: {error}"
        );

        let mut latched = commission.rail;
        latched.health[6] = ServoHealth { id: 16, bits: 0x20 };
        let error = engage_gates(&cfg, &latched).expect_err("a latch does not torque on");
        let SeqError::UnhealthyServo { context, bits } = error else {
            panic!("expected a health refusal, got {error}");
        };
        assert_eq!(context.id, 16);
        assert_eq!(bits, 0x20);

        assert!(machine.log.is_empty(), "a refused gate read nothing");
        assert_eq!(machine.enabled(), [false; ROW_COUNT]);

        // An input-voltage bit on its own is not a latch: it is what a rail
        // recovering from a dip leaves behind, and the floor above is what
        // judges the rail.
        let mut dipped = commission.rail;
        dipped.health[6] = ServoHealth { id: 16, bits: 0x01 };
        assert_eq!(
            engage_gates(&cfg, &dipped).expect("an input-voltage bit alone passes"),
            JointFlags::NONE
        );
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

    /// Every servo is pinged before anything is decided, and the refusal names
    /// all of the silent ones. Nine silent servos are reported as exactly that.
    #[test]
    fn silent_servos_are_all_named_after_every_ping() {
        let cfg = provisioned_config();
        let mut machine = bus();
        machine.silent[3] = true;
        machine.silent[6] = true;
        let error = commission(&cfg, &mut machine).expect_err("two servos are silent");
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
        let error = commission(&cfg, &mut machine).expect_err("nothing answers");
        assert_eq!(
            error.to_string(),
            "presence of servo 10: no answer from all nine servos"
        );

        // One servo unplugged is the ordinary bring-up observation, and it reads
        // as one servo rather than as a list of one.
        let mut machine = bus();
        machine.silent[3] = true;
        let error = commission(&cfg, &mut machine).expect_err("one servo is silent");
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
        let summary = commission(&cfg, &mut machine).expect("the rail comes up");
        assert_eq!(summary.voltage_polls, 3);
        assert_eq!(machine.waits, 2);
        assert_eq!(summary.rail.voltages, [7.4; ROW_COUNT]);

        let mut machine = Machine {
            sweeps: vec![5.0],
            ..bus()
        };
        let error = commission(&cfg, &mut machine).expect_err("the rail never comes up");
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

    /// A masked sweep reads two registers of each named row and touches nothing
    /// else.
    ///
    /// Two transactions per stale row is the whole cost of the fallback, and it
    /// is what makes re-reading a row cheaper than keeping every wake honest
    /// with a sweep of nine. The rows nobody named come back zero and unread:
    /// the caller merges by row, so what this sweep establishes is exactly the
    /// rows it was asked about.
    #[test]
    fn a_masked_sweep_reads_two_transactions_per_named_row() {
        let cfg = provisioned_config();
        let mut machine = Machine {
            sweeps: vec![7.1],
            ..bus()
        };
        commission(&cfg, &mut machine).expect("commissioning passes");
        machine.log.clear();

        let mut rows = JointFlags::NONE;
        flags::insert(&mut rows, JointRef::Leg2);
        flags::insert(&mut rows, JointRef::AntennaLeft);
        let rail = poll(&cfg, &mut machine, rows).expect("both rows answer");

        assert_eq!(
            machine.log.len(),
            2 * usize::try_from(flags::len(rows)).expect("two rows"),
        );
        let asked: Vec<(u8, Option<RegId>)> = machine
            .log
            .iter()
            .map(|(_, request)| (request.context.id, request.context.reg))
            .collect();
        assert_eq!(
            asked,
            vec![
                (SERVO_IDS[3], Some(RegId::PresentInputVoltage)),
                (SERVO_IDS[8], Some(RegId::PresentInputVoltage)),
                (SERVO_IDS[3], Some(RegId::HardwareErrorStatus)),
                (SERVO_IDS[8], Some(RegId::HardwareErrorStatus)),
            ],
            "the supply of every named row, then the error byte of every named row",
        );
        assert!((rail.voltages[3] - 7.1).abs() < 1e-12);
        assert!((rail.voltages[8] - 7.1).abs() < 1e-12);
        assert_eq!(rail.health[8], ServoHealth { id: 18, bits: 0 });
        // A row nobody named is left at nothing rather than at a reading this
        // sweep did not take.
        assert_eq!(rail.voltages[4], 0.0);
        assert_eq!(rail.health[4], ServoHealth { id: 0, bits: 0 });
    }

    /// A mask naming no row asks for nothing and finishes.
    ///
    /// The honest answer to being asked about nothing: the gate that reached for
    /// this fallback found every row fresh, and a sweep that walked the bus
    /// anyway would be the eighteen transactions this design deleted.
    #[test]
    fn a_sweep_named_for_no_row_transacts_nothing() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let rail = poll(&cfg, &mut machine, JointFlags::NONE).expect("nothing was asked");
        assert!(machine.log.is_empty());
        assert_eq!(rail.voltages, [0.0; ROW_COUNT]);
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
            let error =
                commission(&cfg, &mut machine).expect_err("this machine does not commission");
            assert!(
                !machine.log.iter().any(|(_, request)| wrote(request.op)),
                "{error} was raised after writing to the machine"
            );
            assert_eq!(machine.enabled(), [false; ROW_COUNT]);
        }

        let mut machine = bus().provisioned_as(RegId::OperatingMode, value::u8(1));
        assert_eq!(
            commission(&cfg, &mut machine)
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
        let summary = commission(&cfg, &mut machine).expect("a voltage latch is not a fault");
        assert_eq!(summary.rail.health[4], ServoHealth { id: 14, bits: 1 });
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

    /// A wrong servo at a roster address is named with what it answered and what
    /// this platform answers, on whichever of the nine it sits.
    #[test]
    fn identity_names_a_servo_of_the_wrong_kind() {
        let cfg = provisioned_config();

        let mut machine = Machine {
            models: [1200, 1200, 1200, 1191, 1200, 1200, 1200, 1190, 1190],
            ..bus()
        };
        let error = commission(&cfg, &mut machine).expect_err("that leg is not a leg servo");
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
        let error = commission(&cfg, &mut machine).expect_err("that antenna is a leg servo");
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
        let error = commission(&cfg, &mut machine).expect_err("none of those is this platform");
        assert_eq!(
            error,
            SeqError::IdentityMismatch {
                context: StepContext::reg(SeqStepKind::Identity, 10, RegId::ModelNumber),
                model: 1180,
                expected: 1200,
            }
        );
    }

    /// A write refused partway through a sweep stops there: the servos after it
    /// are never written to, and the refusal names the one that refused and the
    /// register it refused.
    #[test]
    fn a_write_that_does_not_land_stops_the_sequence_where_it_stood() {
        let cfg = provisioned_config();
        let mut machine = Machine {
            fail_write: Some((
                14,
                RegId::PositionGains,
                BusResult::VerifyMismatch {
                    read_back: value::u8(0),
                },
            )),
            ..bus()
        };
        let error = commission(&cfg, &mut machine).expect_err("servo 14 will not take its gains");
        let SeqError::VerifyMismatch {
            context,
            expected,
            read_back,
        } = error
        else {
            panic!("expected a verify mismatch, got {error}");
        };
        assert_eq!(context.step, SeqStepKind::GainsProfiles);
        assert_eq!(context.id, 14);
        assert_eq!(context.reg, Some(RegId::PositionGains));
        assert_ne!(expected, value::u8(0));
        assert_eq!(read_back, value::u8(0));
        // Nothing is torqued to get here, and a refused sweep torques nothing.
        assert_eq!(machine.enabled(), [false; ROW_COUNT]);

        // The same knob on a read: a servo refusing its identity read stops the
        // sweep with the status code it sent, before any write at all.
        let mut machine = Machine {
            fail_read: Some((15, RegId::ModelNumber, BusResult::ServoError { code: 7 })),
            ..bus()
        };
        let error = commission(&cfg, &mut machine).expect_err("servo 15 refuses the read");
        assert_eq!(
            error,
            SeqError::Refused {
                context: StepContext::reg(SeqStepKind::Identity, 15, RegId::ModelNumber),
                code: 7,
            }
        );
        assert_eq!(machine.enabled(), [false; ROW_COUNT]);

        // And a corrupt reply, which is never retried by anything below.
        let mut machine = Machine {
            fail_write: Some((12, RegId::PositionGains, BusResult::WireCorrupt)),
            ..bus()
        };
        let error = commission(&cfg, &mut machine).expect_err("that reply came back mangled");
        assert_eq!(
            error,
            SeqError::WireCorrupt {
                context: StepContext::reg(SeqStepKind::GainsProfiles, 12, RegId::PositionGains),
            }
        );
        assert_eq!(machine.enabled(), [false; ROW_COUNT]);

        // A servo refusing the watchdog write — 0x87, the refusal a latched
        // watchdog on this hardware answers with, alert bit and all — stops the
        // sweep naming that servo and that register. The byte reaches the error
        // whole; nothing here reads what it means.
        let mut machine = Machine {
            fail_write: Some((13, RegId::BusWatchdog, BusResult::ServoError { code: 0x87 })),
            ..bus()
        };
        let error =
            commission(&cfg, &mut machine).expect_err("servo 13 refuses the watchdog write");
        assert_eq!(
            error,
            SeqError::Refused {
                context: StepContext::reg(SeqStepKind::GainsProfiles, 13, RegId::BusWatchdog),
                code: 0x87,
            }
        );
        assert_eq!(machine.enabled(), [false; ROW_COUNT]);
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
        crate::testutil::drive(
            &mut CommissionSequencer::start(&cfg, &mut commissioned),
            &mut machine,
        )
        .expect("this machine commissions");

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
        let error = commission(&cfg, &mut machine).expect_err("the rail never comes up");
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
        let error =
            commission(&cfg, &mut machine).expect_err("one servo reports nothing placeable");
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
        slot = PollSnapWire, summary = Rail, seq = PollSequencer,
        resume(host, state) = PollSequencer::resume(host.cfg, state);
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
    ) -> Result<Rail, SeqError> {
        crate::testutil::drive_from_slot(&ResumingPoll { cfg }, slot, machine, Duration::ZERO)
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
    fn a_crossed_sweep_reaches_the_same_rail() {
        let cfg = provisioned_config();
        let mut rows = JointFlags::NONE;
        flags::insert(&mut rows, JointRef::Leg2);
        flags::insert(&mut rows, JointRef::AntennaLeft);

        let (mut direct, mut crossed) = pair(bus());
        let held = poll(&cfg, &mut direct, rows).expect("this machine answers");

        let mut slot = PollSnapWire::new();
        PollSequencer::start(&cfg, &mut slot, rows);
        let restored = poll_from_slot(&cfg, &mut slot, &mut crossed).expect("this machine answers");

        assert_eq!(restored, held);
        assert_eq!(crossed.log, direct.log);
    }

    /// Every failure a commissioning can stop at, crossed: the verdict a
    /// restored sequence hands back is the same verdict, named against the same
    /// servo and the same phase.
    #[test]
    fn a_crossed_path_fails_with_the_same_verdict() {
        let cfg = provisioned_config();

        // Silence at the presence sweep.
        let (mut direct, mut machine) = pair(bus_with_silence(3));
        let held = commission(&cfg, &mut direct).expect_err("that servo is silent");
        assert_eq!(commission_crossed(&cfg, &mut machine), held);

        // A servo of the wrong kind at the identity sweep.
        let models = [1200, 1200, 1200, 1191, 1200, 1200, 1200, 1190, 1190];
        let (mut direct, mut machine) = pair(Machine { models, ..bus() });
        let held = commission(&cfg, &mut direct).expect_err("that leg is not a leg servo");
        assert_eq!(commission_crossed(&cfg, &mut machine), held);

        // A rail that never comes up: the supply gate's polls and its budget are
        // the one piece of clock arithmetic a commission carries across a
        // crossing.
        let (mut direct, mut machine) = pair(Machine {
            sweeps: vec![5.0],
            ..bus()
        });
        let held = commission(&cfg, &mut direct).expect_err("the rail never comes up");
        assert_eq!(commission_crossed(&cfg, &mut machine), held);
        assert_eq!(machine.waits, direct.waits);

        // A gains write that does not land.
        let (mut direct, mut machine) = pair(Machine {
            fail_write: Some((13, RegId::PositionGains, BusResult::NoAnswer)),
            ..bus()
        });
        let held = commission(&cfg, &mut direct).expect_err("that write does not land");
        assert_eq!(commission_crossed(&cfg, &mut machine), held);
    }

    /// The verdict a commissioning crossed at every step stops on.
    fn commission_crossed(cfg: &ArmConfig, machine: &mut Machine) -> SeqError {
        let mut slot = CommissionSnapWire::new();
        CommissionSequencer::start(cfg, &mut slot);
        commission_from_slot(cfg, &mut slot, machine, Duration::ZERO)
            .expect_err("this machine does not commission")
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

    /// The supply gate polled across a rail that comes up: the poll count and
    /// the budget it measures against are the state only a crossing can lose.
    #[test]
    fn a_crossed_sweep_keeps_its_counters() {
        let cfg = provisioned_config();

        let (mut direct, mut machine) = pair(Machine {
            sweeps: vec![5.0, 5.9, 7.4],
            ..bus()
        });
        let held = commission(&cfg, &mut direct).expect("the rail comes up");
        let mut slot = CommissionSnapWire::new();
        CommissionSequencer::start(&cfg, &mut slot);
        let crossed = commission_from_slot(&cfg, &mut slot, &mut machine, Duration::ZERO)
            .expect("the rail comes up");
        assert_eq!(crossed.voltage_polls, held.voltage_polls);
        assert_eq!(machine.waits, direct.waits);
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

        // A servo that stops answering after commissioning.
        let mut machine = bus_with_silence(3);
        let mut rows = JointFlags::NONE;
        flags::insert(&mut rows, JointRef::Leg2);
        let mut slot = PollSnapWire::new();
        let held = crate::testutil::drive(
            &mut PollSequencer::start(&cfg, &mut slot, rows),
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

        // The poll sweep's own refusals: its cursor is a bus row whatever the
        // mask says, its verdict belongs to the failed phase, and that phase
        // needs one.
        let mut slot = PollSnapWire::new();
        let state = slot.clear_valid();
        state.phase = PollPhaseKind::Voltage;
        state.cursor = 9;
        assert_eq!(
            PollSequencer::resume(&cfg, state).err(),
            Some(ResumeError::CursorOutOfRange {
                phase: "voltage",
                cursor: 9,
                bound: 9,
            })
        );
        let state = slot.clear_valid();
        state.phase = PollPhaseKind::Voltage;
        verdict::write(
            &mut state.failure,
            &SeqError::NoAnswer {
                context: StepContext::servo(SeqStepKind::PoseAndDatum, 10),
            },
        )
        .expect("the verdict crosses");
        assert_eq!(
            PollSequencer::resume(&cfg, state).err(),
            Some(ResumeError::ErrorWithoutFailedPhase { phase: "voltage" })
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
}
