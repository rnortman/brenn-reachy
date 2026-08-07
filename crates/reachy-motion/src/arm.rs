//! Arming's configuration, its two records, and the one clamp in this repo.
//!
//! Arming takes a limp platform to a platform holding itself up: nine servos
//! verified register by register, the supply rail confirmed, the resting pose
//! solved and found plausible, every limp servo's goal register confirmed to be
//! mirroring its present position, and only then torque enabled — which holds
//! each joint where it stands — and goals written there. The order of those
//! transactions is the safety property and belongs to the state machine that
//! yields them; what lives here is what that machine decides *with* — the
//! configuration it reads, the records it produces, and the two decisions pure
//! enough to be settled and tested on their own.
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
//! the window by us, deliberately, with the distance recorded and a gate above
//! which arming stops for a human. The alternative is not "no clamp"; it is a
//! write whose effect is unknown.
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
//! Arming produces two descriptions of the machine and they are not the same
//! pose. The **rest** record is where the platform was found — the report's
//! evidence, and the reason a tight rest does not refuse arming. The **armed**
//! record is what arming left it holding, which is the rest pulled into every
//! travel window. The tick starts from the armed record alone: a tick whose
//! goals came from one pose and whose Cartesian mirror came from the other would
//! hand its first trajectory a start the machine is not at.

use core::f64::consts::PI;
use core::fmt;
use core::time::Duration;

use nalgebra::Isometry3;
use reachy_kin::{
    EnvelopeConfig, EnvelopeReport, EnvelopeViolations, FkError, FkOptions, HeadGeometry,
    LegAngles, below_limit, check_envelope, forward_kinematics, min_margin, neutral_head_pose,
    outside_limit, pose_margins, rest_head_pose, stow_head_pose,
};

use crate::joints::{JointId, JointVector, ServoHealth};
use crate::seq::{
    AbsentSet, AnswerKind, BusRequest, BusResult, RegId, RegValue, SeqAction, SeqError, SeqStep,
    Sequencer, StepContext,
};

/// The nine servo IDs in bus order: body yaw, legs 1..=6, right antenna, left
/// antenna.
///
/// The platform's own numbering, and the order every nine-slot array in this
/// crate is in.
pub const SERVO_IDS: [u8; JointId::COUNT] = [10, 11, 12, 13, 14, 15, 16, 17, 18];

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
pub const EXPECTED_MODELS: [u16; JointId::COUNT] =
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
pub const EXPECTED_OPERATING_MODES: [u8; JointId::COUNT] = [3, 3, 3, 3, 3, 3, 3, 4, 4];

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
pub const VENDOR_HOMING_OFFSETS: [i32; JointId::COUNT] =
    [0, 1024, -1024, 1024, -1024, 1024, -1024, 0, 0];

/// The supply floor arming refuses to proceed below, volts.
///
/// Provisional. It is a round number above the point where the servos' own
/// minimum-voltage alarm sits, not a measurement: what the rail does under the
/// current draw of nine servos taking up the head's weight has not been
/// recorded, and until it has, this figure is a guess with a margin.
/// TODO(rail-curve)
pub const DEFAULT_MIN_ARM_VOLTAGE: f64 = 6.0;

/// How often the supply voltage is re-read while waiting for the rail.
///
/// The servos update their own voltage reading about ten times a second, so
/// polling faster would return the same number twice.
pub const DEFAULT_VOLTAGE_POLL_PERIOD: Duration = Duration::from_millis(100);

/// How long arming waits for the rail before giving up.
pub const DEFAULT_VOLTAGE_BUDGET: Duration = Duration::from_secs(30);

/// The largest distance a pin may pull a joint, radians (12°).
///
/// Sized above the worst per-leg overrun in the recorded resting pose, which is
/// around 10°. A pull beyond this is not the documented case any more, and
/// arming stops rather than dragging the head somewhere nobody predicted.
pub const DEFAULT_MAX_PIN_PULL_IN: f64 = 12.0 * PI / 180.0;

/// How far a joint may be from where the arrival check expects it, radians
/// (0.5°).
///
/// It bounds two things: how far an unpulled joint may have moved between the
/// read that follows torque coming on and the read that follows its goal being
/// written — two readings taken under the same load, so the standing position
/// error of a loaded proportional loop cancels out of the comparison and this
/// figure does not have to cover it — and how far outside the corridor between
/// those two readings a pulled joint may sit. The far end of that corridor is
/// the goal, so on a pulled joint alone this figure does bound the standing
/// error, in the direction the pull went. The figure is provisional: several
/// counts of the servo's own 0.088° resolution, wide enough not to chase
/// quantisation and narrow enough that real motion cannot hide inside it.
pub const DEFAULT_RECHECK_TOLERANCE: f64 = 0.5 * PI / 180.0;

/// How far an untorqued servo's goal register may sit from its measured
/// position before arming refuses, radians (2°).
///
/// With torque off this platform's servos report Goal Position as their present
/// position rather than as a stored target: a goal written before torque came on
/// read back one count — 0.088° — from what was written, which is read wobble on
/// a mirrored present and not a value the register kept. Arming depends on that
/// mirroring, because it is what makes enabling torque safe: at the instant of
/// enable the goal is where the joint already is, so no servo can slam. This is
/// the gate that check runs against. Twenty-odd times the observed one-count
/// wobble, and still far under the motion a hand would put into a joint between
/// the position read and the enable.
pub const DEFAULT_GOAL_SHADOW_TOLERANCE: f64 = 2.0 * PI / 180.0;

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

impl From<Gains> for RegValue {
    fn from(gains: Gains) -> Self {
        RegValue::Gains {
            p: gains.p,
            i: gains.i,
            d: gains.d,
        }
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
    pub fn for_joint(&self, joint: JointId) -> Gains {
        match joint {
            JointId::Leg(_) => self.legs,
            JointId::BodyYaw => self.yaw,
            JointId::AntennaRight | JointId::AntennaLeft => self.antennas,
        }
    }
}

/// The gains this platform is armed with.
///
/// The values the vendor's own stack writes at startup, which is the only
/// evidence anyone has about what this linkage wants.
pub const DEFAULT_GAINS: GroupGains = GroupGains {
    legs: Gains { p: 300, i: 0, d: 0 },
    yaw: Gains { p: 200, i: 0, d: 0 },
    antennas: Gains { p: 200, i: 0, d: 0 },
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
    Check(RegValue),
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
    cells: [[ProvisionExpect; PROVISION_REGS.len()]; JointId::COUNT],
}

impl Default for ProvisionTable {
    fn default() -> Self {
        Self {
            cells: [[ProvisionExpect::Skip; PROVISION_REGS.len()]; JointId::COUNT],
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
    pub fn set(&mut self, joint: JointId, reg: RegId, expect: ProvisionExpect) -> bool {
        let Some(row) = joint.index() else {
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
        JointId::ALL
            .iter()
            .all(|joint| self.set(*joint, reg, expect))
    }

    /// Set one register's cell on the six legs, leaving yaw and the antennas
    /// alone. The travel windows are the case: they exist per leg and mean
    /// nothing for the three single-turn joints.
    pub fn set_legs(&mut self, reg: RegId, expect: ProvisionExpect) -> bool {
        (0..6).all(|leg| self.set(JointId::Leg(leg), reg, expect))
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
    pub ids: [u8; JointId::COUNT],
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
    /// The largest pull a pin may apply, radians.
    pub max_pin_pull_in: f64,
    /// How far the arrival check tolerates a joint being from where it expects
    /// it, radians.
    pub recheck_tolerance: f64,
    /// How far an untorqued servo's goal register may sit from its measured
    /// position, radians.
    pub goal_shadow_tolerance: f64,
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
    pub fn id_for(&self, joint: JointId) -> Option<u8> {
        joint.index().and_then(|row| self.ids.get(row).copied())
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
/// with no bound anywhere to bring it inside. A leg pulled beyond
/// `cfg.max_pin_pull_in` stops arming, and so does a measured angle nobody can
/// place: pinning a goal to a value that is not a number would send a
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
/// target by that sag on every re-arm. The pull, and the gate on it, stay
/// measured against where the joint actually is, so a *leg* whose held goal sits
/// implausibly far from its measured position is refused rather than written.
/// The gate is the legs' alone: body yaw and the antennas are pinned at their
/// basis untouched, so on those three a held goal is bounded by nothing nearer
/// than the envelope's own body-yaw cap.
pub fn pin_goals_from(
    cfg: &ArmConfig,
    basis: &JointVector,
    measured: &JointVector,
) -> Result<PinOutcome, SeqError> {
    for (vector, reg) in [
        (measured, RegId::PresentPosition),
        (basis, RegId::GoalPosition),
    ] {
        for (row, (joint, angle)) in vector.joints().into_iter().enumerate() {
            if !angle.is_finite() {
                return Err(SeqError::UnplaceableAngle {
                    context: StepContext::reg(SeqStep::PinAndEnable, cfg.ids[row], reg),
                    joint,
                    angle,
                });
            }
        }
    }

    let mut pinned = *basis;
    let mut pull_in = [0.0; 6];
    // Walked in bus order, so the joint named, the servo addressed and the leg
    // pinned all come from the one ordering table rather than from an offset
    // restated here.
    for (row, (joint, target)) in basis.joints().into_iter().enumerate() {
        let JointId::Leg(leg) = joint else { continue };
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

        if outside_limit(pull_in[leg], cfg.max_pin_pull_in) {
            return Err(SeqError::PullInTooLarge {
                context: StepContext::reg(SeqStep::PinAndEnable, cfg.ids[row], RegId::GoalPosition),
                joint,
                pull_in: pull_in[leg],
                limit: cfg.max_pin_pull_in,
            });
        }
    }

    Ok(PinOutcome { pinned, pull_in })
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

/// The provisioned registers as they were found.
///
/// Every cell arming read, whether it was checked against an expectation or only
/// recorded. The recorded ones are the point: the homing offsets, shutdown masks
/// and model numbers nobody has established a correct value for yet, whose
/// readings in an arm report are what establishes it.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ProvisionReadings {
    cells: [[Option<RegValue>; PROVISION_REGS.len()]; JointId::COUNT],
}

impl ProvisionReadings {
    /// Nothing read yet.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// What `joint`'s `reg` held, or `None` if arming did not read it.
    #[must_use]
    pub fn at(&self, joint: JointId, reg: RegId) -> Option<RegValue> {
        let row = joint.index()?;
        let column = ProvisionTable::column(reg)?;
        self.cells[row][column]
    }

    /// How many cells were read.
    #[must_use]
    pub fn count(&self) -> usize {
        self.cells.iter().flatten().filter(|c| c.is_some()).count()
    }
}

/// What arming found, and what it left the machine holding.
///
/// The bring-up output of a whole arm sequence: two records of the platform, the
/// distances the pin had to pull, and every register-of-record read on the way.
/// Held by value so a report can be printed after the sequencer is gone.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ArmSummary {
    /// Where the platform was found, before anything was pinned.
    pub rest: ArmRecord,
    /// What it was left holding, and what a tick starts from.
    pub armed: ArmRecord,
    /// Per leg, how far the first pin pulled it, radians.
    pub pull_in: [f64; 6],
    /// Per servo found already holding torque, its goal less its measured
    /// position, radians; `None` for a servo found limp, whose goal register is
    /// a mirror of its present position and says nothing.
    ///
    /// The standing position error of a loaded servo running a proportional
    /// term alone. Nothing acts on it — it is the first measurement of a
    /// quantity the tracking threshold is currently a guess about.
    pub droop: [Option<f64>; JointId::COUNT],
    /// Per joint, its position after torque came on less the position it was
    /// resting at, radians.
    ///
    /// Enabling torque in position mode can renormalise a servo's reported
    /// position onto a single turn, which shows up here as a jump of about a
    /// whole turn on a joint that had settled past the half. Recorded rather
    /// than refused: the pins are computed from the post-enable reading, so
    /// whatever this says has already been absorbed.
    pub post_enable_shift: [f64; JointId::COUNT],
    /// Each servo's model number, in bus order.
    pub models: [u16; JointId::COUNT],
    /// Each servo's supply reading from the sweep that passed the gate, volts.
    pub voltages: [f64; JointId::COUNT],
    /// Each servo's hardware-error byte, as read.
    pub health: [ServoHealth; JointId::COUNT],
    /// Whether each servo was already holding torque when arming found it.
    pub torque_before: [bool; JointId::COUNT],
    /// The provisioned registers as they were found.
    pub provisioned: ProvisionReadings,
    /// How many sweeps the voltage gate took.
    pub voltage_polls: u32,
}

impl ArmSummary {
    /// The largest distance the pin pulled any leg, radians. The legs alone:
    /// nothing else is pulled.
    #[must_use]
    pub fn worst_pull_in(&self) -> f64 {
        self.pull_in.iter().copied().fold(0.0, f64::max)
    }
}

/// What the pin phase decided, carried by the phases that act on it.
///
/// In the phase rather than beside it so that being in a pinning phase without
/// pins to write is not a state this type can express.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Pinning {
    /// The resting record the pins were computed from, and the solver seed.
    rest: ArmRecord,
    /// The angles the pins were clamped from: each servo's measured position,
    /// or the goal a servo found holding torque is already holding.
    basis: JointVector,
    /// The goals currently written, or about to be.
    pins: PinOutcome,
    /// The pose those goals hold.
    armed: ArmRecord,
    /// Per leg, how far the pins will pull it off the position it was measured
    /// at, radians.
    pull_in: [f64; 6],
}

/// Which part of the sequence is running, and the cursor within it.
///
/// Read phases carry a cursor over the nine servos; the provisioning sweep's
/// cursor walks servos × registers; the write phases' cursors walk servos ×
/// registers likewise. What each phase has learned travels in the phase where a
/// later phase depends on it, and in the records otherwise.
#[derive(Clone, Copy, Debug)]
enum Phase {
    Presence {
        cursor: usize,
    },
    Identity {
        cursor: usize,
    },
    Provision {
        cursor: usize,
    },
    Voltage {
        cursor: usize,
        started: Duration,
        waiting: bool,
    },
    Health {
        cursor: usize,
    },
    Pose {
        cursor: usize,
    },
    Torque {
        cursor: usize,
        rest: ArmRecord,
    },
    GoalShadow {
        cursor: usize,
        rest: ArmRecord,
    },
    GainsProfiles {
        cursor: usize,
        rest: ArmRecord,
    },
    Enable {
        cursor: usize,
        pinning: Pinning,
    },
    PostEnable {
        cursor: usize,
        pinning: Pinning,
    },
    PinGoals {
        cursor: usize,
        pinning: Pinning,
    },
    Arrival {
        cursor: usize,
        pinning: Pinning,
    },
    Complete(Pinning),
    Failed(SeqError),
}

impl Phase {
    /// The phase name a failure here is reported under.
    fn step(self) -> SeqStep {
        match self {
            Self::Presence { .. } => SeqStep::Presence,
            Self::Identity { .. } => SeqStep::Identity,
            Self::Provision { .. } => SeqStep::Provision,
            Self::Voltage { .. } => SeqStep::VoltageGate,
            Self::Health { .. } => SeqStep::Health,
            Self::Pose { .. } => SeqStep::PoseAndDatum,
            Self::Torque { .. } => SeqStep::StateDiscovery,
            Self::GoalShadow { .. } => SeqStep::GoalShadow,
            Self::GainsProfiles { .. } => SeqStep::GainsProfiles,
            // The enable, the read that follows it, the goals and the arrival
            // check are one phase as far as a report is concerned: they are the
            // same decision, and the register named in the context is what
            // separates them.
            Self::Enable { .. }
            | Self::PostEnable { .. }
            | Self::PinGoals { .. }
            | Self::Arrival { .. }
            | Self::Complete(_) => SeqStep::PinAndEnable,
            // A failure already carries the phase it happened in; taking the
            // name from anywhere else would report a supply gate that refused
            // as a pin that never happened.
            Self::Failed(error) => error.context().step,
        }
    }
}

/// Everything the phases accumulate that no later phase needs in hand.
#[derive(Clone, Copy, Debug, Default)]
struct Records {
    absent: [bool; JointId::COUNT],
    models: [u16; JointId::COUNT],
    voltages: [f64; JointId::COUNT],
    health: [ServoHealth; JointId::COUNT],
    torque: [bool; JointId::COUNT],
    provisioned: ProvisionReadings,
    present: JointVector,
    goal: JointVector,
    droop: [Option<f64>; JointId::COUNT],
    post_enable: JointVector,
    post_enable_shift: [f64; JointId::COUNT],
    arrival: JointVector,
    polls: u32,
}

/// The bus row a set of envelope violations is best reported against.
///
/// The verdict names every failing check itself; this picks the one servo a
/// report has room to address, in the order a reader would want it: the leg or
/// the body whose own bound failed first, and the whole-pose checks — attitude
/// and head-relative yaw — against the first crank, since they belong to the six
/// legs together and a context names one servo.
fn first_violated_row(violations: &EnvelopeViolations) -> usize {
    for leg in 0..6 {
        if violations.unreachable[leg] || violations.window[leg] {
            return 1 + leg;
        }
    }
    if violations.body_yaw {
        return 0;
    }
    1
}

/// The angle at one row of the bus order.
///
/// The bus order lives once, in `JointId`; this is a lookup through it rather
/// than a second statement of it. Every cursor in this file is bounded by
/// `JointId::COUNT` by the phase transitions, so the row is in range wherever
/// this is called.
pub(crate) fn angle_at(joints: &JointVector, row: usize) -> f64 {
    JointId::from_index(row)
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
fn rest_pose_seeds() -> [Isometry3<f64>; 3] {
    [stow_head_pose(), rest_head_pose(), neutral_head_pose()]
}

/// Arming, as a state machine that touches no port.
///
/// Ten phases in a fixed order, one transaction at a time: every servo present,
/// every servo the kind it should be, every provisioned register as configured,
/// the supply up, nothing latched, the resting pose solved, the torque states
/// recorded, every limp servo's goal register found mirroring its present
/// position, the gains and profiles written, and only then torque enabled and
/// the goals written.
///
/// The order is the safety property, and it is three properties: no write of any
/// kind happens before the supply gate; every goal-shadow read happens before
/// any torque is enabled; and no goal is written until torque is on. The first
/// two are what make the third safe. A servo whose goal register mirrors its
/// present position cannot slam when its torque comes on, because the target it
/// picks up is where it already stands — and a goal written before that would
/// not be stored anyway, since the register only keeps writes once torque is on.
/// So the write that means anything is the one after the enable, and it is
/// verified count-exact.
///
/// It lives here, in one readable sequence, testable against scripted replies.
pub struct ArmSequencer {
    cfg: ArmConfig,
    geom: HeadGeometry,
    env: EnvelopeConfig,
    fk: FkOptions,
    phase: Phase,
    pending: Option<BusRequest>,
    records: Records,
}

impl ArmSequencer {
    /// A sequence ready to run against `cfg`.
    ///
    /// The geometry, the envelope and the solver options come separately because
    /// the two records arming produces are solved poses, not angles, and because
    /// the pose arming leaves the machine holding is checked against the same
    /// fence the tick will enforce on every command afterwards. All three have to
    /// be the ones the tick uses.
    #[must_use]
    pub fn new(cfg: &ArmConfig, geom: &HeadGeometry, env: &EnvelopeConfig, fk: &FkOptions) -> Self {
        Self {
            cfg: cfg.clone(),
            geom: geom.clone(),
            env: *env,
            fk: *fk,
            phase: Phase::Presence { cursor: 0 },
            pending: None,
            records: Records::default(),
        }
    }

    fn read(&self, row: usize, reg: RegId) -> BusRequest {
        BusRequest::ReadReg {
            id: self.cfg.ids[row],
            reg,
        }
    }

    fn write(&self, row: usize, reg: RegId, value: RegValue) -> BusRequest {
        BusRequest::WriteRegVerified {
            id: self.cfg.ids[row],
            reg,
            value,
        }
    }

    fn context(&self, row: usize, reg: RegId) -> StepContext {
        StepContext::reg(self.phase.step(), self.cfg.ids[row], reg)
    }

    /// The next action, given the previous transaction's result.
    fn emit(&mut self, now: Duration) -> SeqAction<ArmSummary> {
        let request = match self.phase {
            Phase::Presence { cursor } => BusRequest::Ping {
                id: self.cfg.ids[cursor],
            },
            Phase::Identity { cursor } => self.read(cursor, RegId::ModelNumber),
            Phase::Provision { cursor } => {
                let (row, column) = (cursor / PROVISION_REGS.len(), cursor % PROVISION_REGS.len());
                self.read(row, PROVISION_REGS[column])
            }
            Phase::Voltage {
                cursor, waiting, ..
            } => {
                if waiting {
                    // Space the sweeps out: the servos refresh their own voltage
                    // reading about ten times a second, so a faster poll reads
                    // the same number twice.
                    if let Phase::Voltage { waiting, .. } = &mut self.phase {
                        *waiting = false;
                    }
                    return SeqAction::Wait {
                        until: now + self.cfg.voltage_poll_period,
                    };
                }
                self.read(cursor, RegId::PresentInputVoltage)
            }
            Phase::Health { cursor } => self.read(cursor, RegId::HardwareErrorStatus),
            Phase::Pose { cursor } => self.read(cursor, RegId::PresentPosition),
            Phase::Torque { cursor, .. } => self.read(cursor, RegId::TorqueEnable),
            Phase::GoalShadow { cursor, .. } => self.read(cursor, RegId::GoalPosition),
            Phase::GainsProfiles { cursor, .. } => {
                if cursor < JointId::COUNT {
                    let gains = self.cfg.gains.for_joint(JointId::ALL[cursor]);
                    self.write(cursor, RegId::PositionGains, gains.into())
                } else {
                    let index = cursor - JointId::COUNT;
                    let (reg, value) = if index.is_multiple_of(2) {
                        (
                            RegId::ProfileAcceleration,
                            RegValue::U32(self.cfg.profile.acceleration),
                        )
                    } else {
                        (
                            RegId::ProfileVelocity,
                            RegValue::U32(self.cfg.profile.velocity),
                        )
                    };
                    self.write(index / 2, reg, value)
                }
            }
            Phase::Enable { cursor, .. } => {
                self.write(cursor, RegId::TorqueEnable, RegValue::U8(1))
            }
            Phase::PostEnable { cursor, .. } | Phase::Arrival { cursor, .. } => {
                self.read(cursor, RegId::PresentPosition)
            }
            Phase::PinGoals { cursor, pinning } => {
                let goal = RegValue::Radians(angle_at(&pinning.pins.pinned, cursor));
                self.write(cursor, RegId::GoalPosition, goal)
            }
            Phase::Complete(pinning) => return SeqAction::Done(self.summary(&pinning)),
            Phase::Failed(error) => return SeqAction::Fail(error),
        };
        self.pending = Some(request);
        SeqAction::Transact(request)
    }

    fn summary(&self, pinning: &Pinning) -> ArmSummary {
        ArmSummary {
            rest: pinning.rest,
            armed: pinning.armed,
            pull_in: pinning.pull_in,
            droop: self.records.droop,
            post_enable_shift: self.records.post_enable_shift,
            models: self.records.models,
            voltages: self.records.voltages,
            health: self.records.health,
            torque_before: self.records.torque,
            provisioned: self.records.provisioned,
            voltage_polls: self.records.polls,
        }
    }

    /// Take the previous transaction's result and move the cursor on.
    fn absorb(&mut self, now: Duration, prior: Option<&BusResult>) -> Result<(), SeqError> {
        let Some(request) = self.pending.take() else {
            // Nothing was outstanding — the first call, or the call after a
            // wait. A result handed back here answers no request, so there is
            // nothing to validate it against and nothing to report it under.
            return Ok(());
        };
        let context = StepContext {
            step: self.phase.step(),
            id: request.id(),
            reg: request.reg(),
        };
        let Some(result) = prior else {
            // A transaction ran and nothing came back. From here that is
            // indistinguishable from silence on the wire, and it is treated as
            // silence rather than quietly retried.
            return Err(SeqError::NoAnswer { context });
        };
        match self.phase {
            Phase::Presence { cursor } => self.absorb_presence(cursor, context, result),
            Phase::Identity { cursor } => self.absorb_identity(now, cursor, context, result),
            Phase::Provision { cursor } => self.absorb_provision(now, cursor, context, result),
            Phase::Voltage {
                cursor, started, ..
            } => self.absorb_voltage(now, cursor, started, context, result),
            Phase::Health { cursor } => self.absorb_health(cursor, context, result),
            Phase::Pose { cursor } => self.absorb_pose(cursor, context, result),
            Phase::Torque { cursor, rest } => self.absorb_torque(cursor, rest, context, result),
            Phase::GoalShadow { cursor, rest } => {
                self.absorb_goal_shadow(cursor, rest, context, result)
            }
            Phase::GainsProfiles { cursor, rest } => {
                confirm_write(result, &request, context)?;
                self.phase = if cursor + 1 < 3 * JointId::COUNT {
                    Phase::GainsProfiles {
                        cursor: cursor + 1,
                        rest,
                    }
                } else {
                    return self.enter_enable(rest);
                };
                Ok(())
            }
            Phase::Enable { cursor, pinning } => {
                confirm_write(result, &request, context)?;
                self.phase = if cursor + 1 < JointId::COUNT {
                    Phase::Enable {
                        cursor: cursor + 1,
                        pinning,
                    }
                } else {
                    Phase::PostEnable { cursor: 0, pinning }
                };
                Ok(())
            }
            Phase::PostEnable { cursor, pinning } => {
                self.absorb_post_enable(cursor, pinning, context, result)
            }
            Phase::PinGoals { cursor, pinning } => {
                confirm_write(result, &request, context)?;
                self.phase = if cursor + 1 < JointId::COUNT {
                    Phase::PinGoals {
                        cursor: cursor + 1,
                        pinning,
                    }
                } else {
                    Phase::Arrival { cursor: 0, pinning }
                };
                Ok(())
            }
            Phase::Arrival { cursor, pinning } => {
                self.absorb_arrival(cursor, pinning, context, result)
            }
            // Terminal: nothing is ever outstanding, so this is unreachable.
            Phase::Complete(_) | Phase::Failed(_) => Ok(()),
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
            Err(SeqError::NoAnswer { .. }) => self.records.absent[cursor] = true,
            Err(other) => return Err(other),
        }
        if cursor + 1 < JointId::COUNT {
            self.phase = Phase::Presence { cursor: cursor + 1 };
            return Ok(());
        }
        let absent = AbsentSet::new(&self.cfg.ids, &self.records.absent);
        if let Some(&first) = absent.ids().first() {
            return Err(SeqError::AbsentServos {
                context: StepContext::servo(SeqStep::Presence, first),
                absent,
            });
        }
        self.phase = Phase::Identity { cursor: 0 };
        Ok(())
    }

    fn absorb_identity(
        &mut self,
        now: Duration,
        cursor: usize,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        self.records.models[cursor] = result.value(context)?.u16(context)?;
        if cursor + 1 < JointId::COUNT {
            self.phase = Phase::Identity { cursor: cursor + 1 };
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
        for (row, (model, expected)) in self
            .records
            .models
            .into_iter()
            .zip(EXPECTED_MODELS)
            .enumerate()
        {
            if model != expected {
                return Err(SeqError::IdentityMismatch {
                    context: StepContext::reg(
                        SeqStep::Identity,
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
        let cell = (from..columns * JointId::COUNT).find(|flat| {
            !matches!(
                self.cfg.expected.at(flat / columns, flat % columns),
                Some(ProvisionExpect::Skip) | None
            )
        });
        self.phase = match cell {
            Some(cursor) => Phase::Provision { cursor },
            None => Phase::Voltage {
                cursor: 0,
                started: now,
                waiting: false,
            },
        };
        Ok(())
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
        self.records.provisioned.cells[row][column] = Some(observed);
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
        started: Duration,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        self.records.voltages[cursor] = result.value(context)?.volts(context)?;
        if cursor + 1 < JointId::COUNT {
            self.phase = Phase::Voltage {
                cursor: cursor + 1,
                started,
                waiting: false,
            };
            return Ok(());
        }
        self.records.polls += 1;
        let limit = self.cfg.min_arm_voltage;
        // The worst reading of the sweep, not the first one under the floor: the
        // nine servos sit on different lengths of wiring and read differently, and
        // the servo a report names has to be the one worst off. A reading nobody
        // can place wins outright — `below_limit` is the bound test the whole
        // project uses, so a NaN counts as "not up" rather than passing every
        // comparison by default.
        let mut low: Option<(usize, f64)> = None;
        for (row, volts) in self.records.voltages.iter().copied().enumerate() {
            if !below_limit(volts, limit) {
                continue;
            }
            let worse = match low {
                None => true,
                Some((_, lowest)) => lowest.is_finite() && (!volts.is_finite() || volts < lowest),
            };
            if worse {
                low = Some((row, volts));
            }
        }
        let Some((row, lowest)) = low else {
            self.phase = Phase::Health { cursor: 0 };
            return Ok(());
        };
        let waited = now.saturating_sub(started);
        if waited >= self.cfg.voltage_budget {
            return Err(SeqError::VoltageLow {
                context: StepContext::reg(
                    SeqStep::VoltageGate,
                    self.cfg.ids[row],
                    RegId::PresentInputVoltage,
                ),
                readings: self.records.voltages,
                lowest,
                limit,
                waited,
            });
        }
        self.phase = Phase::Voltage {
            cursor: 0,
            started,
            waiting: true,
        };
        Ok(())
    }

    fn absorb_health(
        &mut self,
        cursor: usize,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        let bits = result.value(context)?.u8(context)?;
        let health = ServoHealth {
            id: self.cfg.ids[cursor],
            bits,
        };
        self.records.health[cursor] = health;
        // No reboot, ever, and no clearing of the latch: a servo holding this
        // head that is rebooted drops it.
        if !health.healthy_or_voltage_only() {
            return Err(SeqError::UnhealthyServo { context, bits });
        }
        self.phase = if cursor + 1 < JointId::COUNT {
            Phase::Health { cursor: cursor + 1 }
        } else {
            Phase::Pose { cursor: 0 }
        };
        Ok(())
    }

    fn absorb_pose(
        &mut self,
        cursor: usize,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        let joint = JointId::ALL[cursor];
        let angle = self.placeable(cursor, context, result)?;
        self.records.present.set(joint, angle);
        if cursor + 1 < JointId::COUNT {
            self.phase = Phase::Pose { cursor: cursor + 1 };
            return Ok(());
        }
        // Failure is not a solver problem to retry with a perturbed seed: the
        // angles are what nine servos reported, and angles that place no pose
        // say the model and the machine disagree.
        let rest = ArmRecord::solve(
            &self.geom,
            &self.fk,
            &self.records.present,
            &rest_pose_seeds(),
        )
        .map_err(|cause| SeqError::RestPoseImplausible {
            // Named against the first crank: the failure belongs to the six of
            // them together, and a context names one servo.
            context: self.context(1, RegId::PresentPosition),
            cause,
        })?;
        self.phase = Phase::Torque { cursor: 0, rest };
        Ok(())
    }

    fn absorb_torque(
        &mut self,
        cursor: usize,
        rest: ArmRecord,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        // Recorded per servo, never reduced to one boolean: a bus where three of
        // nine hold torque is a state worth seeing, and "some do" is not.
        self.records.torque[cursor] = result.value(context)?.u8(context)? != 0;
        self.phase = if cursor + 1 < JointId::COUNT {
            Phase::Torque {
                cursor: cursor + 1,
                rest,
            }
        } else {
            Phase::GoalShadow { cursor: 0, rest }
        };
        Ok(())
    }

    /// Every limp servo's goal register against the position it was measured at,
    /// before a byte is written to the machine.
    ///
    /// With torque off this platform's servos report Goal Position as their
    /// present position, so a limp servo whose goal reads anywhere else is
    /// either a machine that has moved since the position sweep or a firmware
    /// that does not mirror — and the enable-then-pin order this sequence uses
    /// is safe only because it does. Either way arming stops here, with every
    /// servo's torque exactly as it was found.
    ///
    /// A servo already holding torque is exempt: its goal is a target it is
    /// really holding, and the gap between that and its measured position is
    /// the sag of a loaded servo, recorded rather than judged.
    ///
    /// TODO(held-goal-bound): that gap becomes the pin basis for the servo, and
    /// on body yaw nothing bounds how far from the measured position it may sit
    /// — the pull-in gate that bounds it on a leg is the legs' alone. What it
    /// costs is an armed record claiming a pose the machine is not at, and — if
    /// the torque register is what is wrong — an enable against a goal this
    /// exemption is the reason nobody checked. What a plausible bound is worth
    /// setting at wants the sag measured first.
    fn absorb_goal_shadow(
        &mut self,
        cursor: usize,
        rest: ArmRecord,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        let joint = JointId::ALL[cursor];
        let goal = self.placeable(cursor, context, result)?;
        let present = angle_at(&self.records.present, cursor);
        self.records.goal.set(joint, goal);
        if self.records.torque[cursor] {
            self.records.droop[cursor] = Some(goal - present);
        } else if outside_limit((goal - present).abs(), self.cfg.goal_shadow_tolerance) {
            return Err(SeqError::GoalShadowMismatch {
                context,
                joint,
                goal,
                present,
                tolerance: self.cfg.goal_shadow_tolerance,
            });
        }
        self.phase = if cursor + 1 < JointId::COUNT {
            Phase::GoalShadow {
                cursor: cursor + 1,
                rest,
            }
        } else {
            Phase::GainsProfiles { cursor: 0, rest }
        };
        Ok(())
    }

    /// The angles the pins are clamped from, given a sweep of measured
    /// positions.
    ///
    /// A servo found limp pins at where it is. A servo found holding torque pins
    /// at the goal it is already holding: its measured position sits a sag below
    /// that goal, and pinning at the sag would lower the target by that sag
    /// every time the machine is re-armed, which over a session is a ratchet
    /// nobody commanded.
    fn pin_basis(&self, measured: &JointVector) -> JointVector {
        let mut basis = *measured;
        for (row, holding) in self.records.torque.into_iter().enumerate() {
            if holding {
                let joint = JointId::ALL[row];
                basis.set(joint, angle_at(&self.records.goal, row));
            }
        }
        basis
    }

    /// The pose a set of pins holds, and the envelope's verdict on it.
    ///
    /// The seed is the last pose this sequence solved: the pull is at most the
    /// pull-in gate, which is millimetres of head motion, so it is a close one.
    fn solve_pins(&self, pins: &PinOutcome, seed: &Isometry3<f64>) -> Result<ArmRecord, SeqError> {
        // The context names the first crank and the register about to be
        // written: the pins are what failed, and everything before them landed.
        let armed =
            ArmRecord::solve(&self.geom, &self.fk, &pins.pinned, &[*seed]).map_err(|cause| {
                SeqError::PinnedPoseUnsolvable {
                    context: StepContext::reg(
                        SeqStep::PinAndEnable,
                        self.cfg.ids[1],
                        RegId::GoalPosition,
                    ),
                    cause,
                }
            })?;
        self.check_armed_pose(&armed)?;
        Ok(armed)
    }

    /// Compute provisional pins and check them, then start enabling torque.
    ///
    /// Nothing here is written and nothing is enabled: a pin that would pull too
    /// far, an angle nobody can place, or a goal set that closes no linkage all
    /// stop the sequence with the machine's torque exactly as it was found,
    /// which is the state a hand can still put right. The pins are recomputed
    /// after the enables against the positions measured then, because torque
    /// coming on can move what a servo reports; these are what stands between a
    /// grossly mishandled machine and having torque put on it at all.
    fn enter_enable(&mut self, rest: ArmRecord) -> Result<(), SeqError> {
        let basis = self.pin_basis(&self.records.present);
        let pins = pin_goals_from(&self.cfg, &basis, &self.records.present)?;
        let armed = self.solve_pins(&pins, &rest.head_pose_body)?;
        self.phase = Phase::Enable {
            cursor: 0,
            pinning: Pinning {
                rest,
                basis,
                pins,
                armed,
                pull_in: pins.pull_in,
            },
        };
        Ok(())
    }

    /// Refuse a pinned pose the tick's own envelope would refuse.
    ///
    /// Every trajectory starts at the pose arming left the machine holding, so a
    /// start outside the envelope is a move that faults on its second tick, at a
    /// pose the machine is already standing in and with torque already on. The
    /// legs are inside their windows by the pin; this is what covers the rest of
    /// the verdict — body yaw, head attitude and head-relative yaw — none of
    /// which the pin has any fence for.
    /// It runs the whole envelope rather than the remainder, so a pin that ever
    /// stopped establishing what it establishes is caught here rather than
    /// passing unchecked.
    ///
    /// The clearance floor is excepted, and only it: the platform is known to
    /// come to rest tighter than the floor, and what governs the move off such a
    /// rest is the present clearance as a baseline, not the floor.
    fn check_armed_pose(&self, armed: &ArmRecord) -> Result<(), SeqError> {
        let mut report = EnvelopeReport::default();
        let verdict = check_envelope(
            &self.geom,
            &self.env,
            &armed.head_pose_body,
            armed.joints.body_yaw,
            None,
            &mut report,
        );
        let Err(error) = verdict else { return Ok(()) };
        let mut violations = error.violations;
        violations.margin = false;
        if !violations.any() {
            return Ok(());
        }
        Err(SeqError::PinnedPoseOutsideEnvelope {
            context: StepContext::reg(
                SeqStep::PinAndEnable,
                self.cfg.ids[first_violated_row(&violations)],
                RegId::GoalPosition,
            ),
            violations,
        })
    }

    /// One position reading from the sweep that follows the enables, and — on
    /// the last of them — the final pins computed from that sweep.
    ///
    /// This read is where a position renormalisation at torque-on is absorbed:
    /// the goals about to be written come from what the servos report now, not
    /// from what they reported limp, so a servo that has moved its reported
    /// frame is pinned where it now says it is. The shift is recorded per joint,
    /// and the pull-in gate, the solver and the envelope judge the result — a
    /// renormalisation is hypothesised firmware behaviour rather than an
    /// anomaly, and refusing it outright would refuse an ordinary parked
    /// antenna.
    ///
    /// A refusal from here on arrives with torque on and held. The machine keeps
    /// standing where it stands and recovery is the operator's; the alternative,
    /// cutting torque, drops the head.
    fn absorb_post_enable(
        &mut self,
        cursor: usize,
        pinning: Pinning,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        let joint = JointId::ALL[cursor];
        let angle = self.placeable(cursor, context, result)?;
        self.records.post_enable.set(joint, angle);
        self.records.post_enable_shift[cursor] = angle - angle_at(&self.records.present, cursor);
        if cursor + 1 < JointId::COUNT {
            self.phase = Phase::PostEnable {
                cursor: cursor + 1,
                pinning,
            };
            return Ok(());
        }
        let basis = self.pin_basis(&self.records.post_enable);
        let pins = pin_goals_from(&self.cfg, &basis, &self.records.post_enable)?;
        let armed = self.solve_pins(&pins, &pinning.armed.head_pose_body)?;
        self.phase = Phase::PinGoals {
            cursor: 0,
            pinning: Pinning {
                basis,
                pins,
                armed,
                pull_in: pins.pull_in,
                ..pinning
            },
        };
        Ok(())
    }

    /// One position reading from the sweep that follows the goal writes, and —
    /// on the last of them — the verdict on the nine.
    fn absorb_arrival(
        &mut self,
        cursor: usize,
        pinning: Pinning,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        let joint = JointId::ALL[cursor];
        let angle = self.placeable(cursor, context, result)?;
        self.records.arrival.set(joint, angle);
        if cursor + 1 < JointId::COUNT {
            self.phase = Phase::Arrival {
                cursor: cursor + 1,
                pinning,
            };
            return Ok(());
        }
        self.check_arrival(&pinning)?;
        self.phase = Phase::Complete(pinning);
        Ok(())
    }

    /// Every joint against where the goals should have left it.
    ///
    /// A joint whose goal is where it already was may not have moved at all: it
    /// is compared against the reading the sweep before, and any motion is
    /// motion nothing commanded. That comparison is between two readings taken
    /// under the same load, so the standing offset a loaded proportional term
    /// holds from its target cancels out of it rather than having to fit inside
    /// the tolerance.
    ///
    /// A joint whose goal pulled it somewhere else has to be between the two, no
    /// further out than the tolerance at either end — arrived, or on its way,
    /// and going the right way. The far end of that corridor is the goal, so for
    /// a pulled joint the tolerance does bound the standing offset in the
    /// direction of the pull.
    ///
    /// TODO(arrival-far-corridor): a joint the load pushes past its goal settles
    /// outside the far end once its standing offset exceeds the tolerance, and
    /// is refused for behaving the way a proportional loop behaves. How large
    /// that offset really is on this unit is unmeasured, so whether the far end
    /// wants widening — and by what, drawn from what — waits on the droop
    /// figures a supervised arm brings back.
    ///
    /// TODO(pin-settle-dwell): nothing waits between the last goal write and
    /// this sweep, so a pulled joint is read while it is still travelling. The
    /// corridor admits that. What it cannot tell is a joint that has stopped
    /// short of its goal inside the corridor from one still moving towards it,
    /// and how long this unit's pulls actually take is the measurement that
    /// would settle whether a dwell belongs here.
    fn check_arrival(&self, pinning: &Pinning) -> Result<(), SeqError> {
        let tolerance = self.cfg.recheck_tolerance;
        for (row, (joint, present)) in self.records.arrival.joints().into_iter().enumerate() {
            let pin = angle_at(&pinning.pins.pinned, row);
            let before = angle_at(&self.records.post_enable, row);
            // The clamp either replaced the basis angle or left it untouched, so
            // an unpulled joint's pin is that angle bit for bit.
            let settled = if pin == angle_at(&pinning.basis, row) {
                !outside_limit((present - before).abs(), tolerance)
            } else {
                present >= before.min(pin) - tolerance && present <= before.max(pin) + tolerance
            };
            if !settled {
                return Err(SeqError::PinUnstable {
                    context: self.context(row, RegId::PresentPosition),
                    joint,
                    pinned: pin,
                    before,
                    present,
                });
            }
        }
        Ok(())
    }

    /// An angle reading that is a number, or the refusal that says it is not.
    ///
    /// A reading nobody can place decides nothing: it closes no linkage, sits
    /// inside no window, and would become a goal that means nothing. Every
    /// sweep of positions and goals passes through here, so the refusal has one
    /// shape and names the joint that carried it.
    fn placeable(
        &self,
        row: usize,
        context: StepContext,
        result: &BusResult,
    ) -> Result<f64, SeqError> {
        let angle = result.value(context)?.radians(context)?;
        if angle.is_finite() {
            return Ok(angle);
        }
        Err(SeqError::UnplaceableAngle {
            context,
            joint: JointId::ALL[row],
            angle,
        })
    }
}

/// Confirm that a write landed, comparing against the value the request carried.
pub(crate) fn confirm_write(
    result: &BusResult,
    request: &BusRequest,
    context: StepContext,
) -> Result<(), SeqError> {
    match request.value() {
        Some(wrote) => result.written(context, wrote),
        // Only a write phase confirms a write, and a write request carries its
        // value; a read-back with nothing to compare against is the driver
        // answering a question nobody asked.
        None => Err(SeqError::WrongAnswer {
            context,
            expected: AnswerKind::Written,
            observed: result.kind(),
        }),
    }
}

impl Sequencer for ArmSequencer {
    type Summary = ArmSummary;

    fn next(&mut self, now: Duration, prior: Option<&BusResult>) -> SeqAction<ArmSummary> {
        if let Err(error) = self.absorb(now, prior) {
            self.phase = Phase::Failed(error);
        }
        self.emit(now)
    }

    fn step(&self) -> SeqStep {
        self.phase.step()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::testutil::ScriptedBus;
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
        assert!(outcome.worst_pull_in() < cfg.max_pin_pull_in);
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

    /// A pull beyond the gate stops arming, naming the leg, the servo, the
    /// register about to be written, and both figures.
    #[test]
    fn a_pull_beyond_the_gate_refuses() {
        let cfg = ArmConfig {
            max_pin_pull_in: 1.0_f64.to_radians(),
            ..config()
        };
        let present = joints_at(&rest_head_pose());
        let error = pin_goals(&cfg, &present).expect_err("the sixth leg is ten degrees out");
        let SeqError::PullInTooLarge {
            context,
            joint,
            pull_in,
            limit,
        } = error
        else {
            panic!("expected a pull-in refusal, got {error}");
        };
        // Leg 1 is the first one out, so it is the one reported.
        assert_eq!(joint, JointId::Leg(0));
        assert_eq!(context.id, 11);
        assert_eq!(context.reg, Some(RegId::GoalPosition));
        assert_eq!(context.step, SeqStep::PinAndEnable);
        assert!((pull_in.to_degrees() - 7.543).abs() < 1e-3);
        assert_eq!(limit, cfg.max_pin_pull_in);
        assert_eq!(
            error.to_string(),
            "pin and enable of servo 11, goal position: pinning leg 1 would pull it 7.54°, \
             past the 1.00° gate"
        );
    }

    /// An angle nobody can place is refused before anything is pinned, on
    /// whichever of the nine it arrives on — including the three the windows say
    /// nothing about, which is exactly where a clamp would have hidden it.
    #[test]
    fn an_unplaceable_angle_refuses_on_every_joint() {
        let cfg = config();
        let good = joints_at(&reachy_kin::neutral_head_pose());
        for (row, joint) in JointId::ALL.into_iter().enumerate() {
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
                context: StepContext::reg(SeqStep::PinAndEnable, 17, RegId::PresentPosition),
                joint: JointId::AntennaRight,
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
        for (row, joint) in JointId::ALL.into_iter().enumerate() {
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

    /// A machine standing where a command left it arms, and the resting record
    /// lands where it is standing.
    ///
    /// Every command in this bench re-arms, so the pose phase six solves is
    /// whatever the previous command left the machine holding — neutral, after
    /// the lift, which is 44 mm and a pitch away from either resting candidate.
    /// Driven through the sequence, so it is the solve phase six really runs
    /// that lands there; which of the seeds carried it is not observable here
    /// and the test above is what pins the list.
    #[test]
    fn the_pose_a_command_leaves_the_machine_at_is_solved_at_phase_six() {
        let neutral = reachy_kin::neutral_head_pose();
        let mut machine = bus();
        machine.present = joints_at(&neutral);
        machine.present.body_yaw = 0.35;
        machine.present.antennas = [0.20, -0.15];

        let summary =
            drive(&provisioned_config(), &mut machine).expect("a machine standing at neutral arms");
        let gap =
            (summary.rest.head_pose_body.translation.vector - neutral.translation.vector).norm();
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

        assert!(table.set_all(
            RegId::OperatingMode,
            ProvisionExpect::Check(RegValue::U8(3))
        ));
        assert!(table.set_all(RegId::Shutdown, ProvisionExpect::Record));
        assert!(table.set_legs(
            RegId::MinPositionLimit,
            ProvisionExpect::Check(RegValue::U32(1502))
        ));
        assert_eq!(table.checks(), JointId::COUNT + 6);
        assert_eq!(table.reads(), 2 * JointId::COUNT + 6);

        let mode = ProvisionTable::column(RegId::OperatingMode).expect("a provisioned register");
        let limit =
            ProvisionTable::column(RegId::MinPositionLimit).expect("a provisioned register");
        assert_eq!(
            table.at(0, mode),
            Some(ProvisionExpect::Check(RegValue::U8(3)))
        );
        // Yaw is row 0 and has no travel window of its own.
        assert_eq!(table.at(0, limit), Some(ProvisionExpect::Skip));
        assert_eq!(
            table.at(1, limit),
            Some(ProvisionExpect::Check(RegValue::U32(1502)))
        );

        assert_eq!(ProvisionTable::column(RegId::PresentPosition), None);
        assert!(!table.set(
            JointId::Leg(9),
            RegId::OperatingMode,
            ProvisionExpect::Record
        ));
        assert!(!table.set(
            JointId::BodyYaw,
            RegId::GoalPosition,
            ProvisionExpect::Record
        ));
        assert_eq!(table.at(JointId::COUNT, mode), None);
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
        assert_eq!(cfg.id_for(JointId::BodyYaw), Some(10));
        assert_eq!(cfg.id_for(JointId::Leg(0)), Some(11));
        assert_eq!(cfg.id_for(JointId::Leg(5)), Some(16));
        assert_eq!(cfg.id_for(JointId::AntennaRight), Some(17));
        assert_eq!(cfg.id_for(JointId::AntennaLeft), Some(18));
        assert_eq!(cfg.id_for(JointId::Leg(6)), None);
        for (row, joint) in JointId::ALL.into_iter().enumerate() {
            assert_eq!(cfg.id_for(joint), Some(SERVO_IDS[row]));
        }
    }

    /// The legs are tuned harder than the joints carrying nothing, and both
    /// cross the boundary as the gain span the wire layer writes.
    #[test]
    fn gains_are_per_group() {
        let gains = DEFAULT_GAINS;
        assert_eq!(gains.for_joint(JointId::Leg(3)), gains.legs);
        assert_eq!(gains.for_joint(JointId::BodyYaw), gains.yaw);
        assert_eq!(gains.for_joint(JointId::AntennaLeft), gains.antennas);
        assert!(gains.legs.p > gains.yaw.p);
        for group in [gains.legs, gains.yaw, gains.antennas] {
            assert_eq!((group.i, group.d), (0, 0));
            assert_eq!(
                RegValue::from(group),
                RegValue::Gains {
                    p: group.p,
                    i: 0,
                    d: 0
                }
            );
        }
        assert_eq!(DEFAULT_GAINS.legs.to_string(), "P 300 I 0 D 0");
    }

    /// The provisional thresholds are the values the comments say they are. A
    /// figure nobody has measured is worth pinning, so changing one is a
    /// deliberate act rather than a drift.
    #[test]
    fn the_provisional_thresholds_are_what_they_claim() {
        assert_eq!(DEFAULT_MIN_ARM_VOLTAGE, 6.0);
        assert_eq!(DEFAULT_VOLTAGE_POLL_PERIOD, Duration::from_millis(100));
        assert_eq!(DEFAULT_VOLTAGE_BUDGET, Duration::from_secs(30));
        assert!((DEFAULT_MAX_PIN_PULL_IN.to_degrees() - 12.0).abs() < 1e-12);
        assert!((DEFAULT_RECHECK_TOLERANCE.to_degrees() - 0.5).abs() < 1e-12);
        assert!((DEFAULT_GOAL_SHADOW_TOLERANCE.to_degrees() - 2.0).abs() < 1e-12);
        // The shadow gate is well above the servo's own resolution, so a count
        // of read wobble on a mirrored register cannot trip it.
        assert!(DEFAULT_GOAL_SHADOW_TOLERANCE > 20.0 * (0.088_f64).to_radians());
        // And it is not the arrival tolerance under another name: swapping the
        // two configuration keys would change both gates.
        assert_ne!(
            DEFAULT_GOAL_SHADOW_TOLERANCE.to_bits(),
            DEFAULT_RECHECK_TOLERANCE.to_bits()
        );
        // The pull-in gate is above the worst overrun the recorded rest has, or
        // arming would refuse the case it was sized for.
        let cfg = config();
        let present = joints_at(&rest_head_pose());
        let outcome = pin_goals(&cfg, &present).expect("inside the gate");
        assert!(outcome.worst_pull_in() < DEFAULT_MAX_PIN_PULL_IN);
    }

    /// A window narrower than the platform's own does not silently take over: a
    /// pin is against the servo-side fence, so a caller handing over the wrong
    /// fence pulls the legs to it and the gate is what notices.
    #[test]
    fn the_pin_is_against_the_window_it_was_handed() {
        let narrow = 5.0_f64.to_radians();
        let cfg = ArmConfig {
            leg_windows: [(-narrow, narrow); 6],
            ..config()
        };
        let present = joints_at(&reachy_kin::neutral_head_pose());
        // The neutral pose sits near ±36°, so every leg is pulled to the fence
        // and the first one trips the gate.
        let error = pin_goals(&cfg, &present).expect_err("thirty degrees is past the gate");
        assert!(matches!(error, SeqError::PullInTooLarge { .. }));

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

    /// How far a joint the goals pulled is short of its goal while it is still
    /// travelling, in the machine below. Twice the arrival tolerance, so a
    /// reading this far out is inside no tolerance and only the corridor admits
    /// it.
    const TRAVEL: f64 = 1.0 * PI / 180.0;

    /// Nine servos answering out of their own state, with a knob per thing a test
    /// wants to vary. The transaction log is what the phase-order assertions
    /// read: it is the whole content of "arming did these things in this order".
    ///
    /// The goal register is modelled as this platform's: with torque off it
    /// mirrors the present position and keeps nothing written to it, and once
    /// torque is on it stores what is written and the servo goes there.
    #[derive(Clone, Debug)]
    struct Machine {
        models: [u16; JointId::COUNT],
        silent: [bool; JointId::COUNT],
        /// Supply readings, one per poll; the last one repeats for ever.
        sweeps: Vec<f64>,
        /// Per servo, how far below the sweep's reading that servo reports.
        sag: [f64; JointId::COUNT],
        health: [u8; JointId::COUNT],
        /// Whether each servo holds torque, as one fact rather than two: an
        /// enable that lands sets it, and everything torque decides — the goal
        /// register storing writes, the reported frame taking its enable shift,
        /// a goal becoming a motion — reads it.
        torque: [u8; JointId::COUNT],
        /// Where each joint reads while the machine is as arming found it.
        present: JointVector,
        /// What the goal register of a servo *found holding torque* holds: the
        /// target it is holding. A limp servo's goal mirrors its present
        /// position instead and this says nothing about it.
        held: JointVector,
        /// Per servo, how far its goal register reads from its present position
        /// while it is limp. Zero is the mirroring this platform does.
        shadow_gap: [f64; JointId::COUNT],
        /// Per joint, what its reported position jumps by when its torque comes
        /// on — the single-turn renormalisation a servo may do there.
        enable_shift: [f64; JointId::COUNT],
        /// Per joint, how far below its goal a servo holding torque settles: the
        /// standing error of a loaded position loop with no integral term.
        load: [f64; JointId::COUNT],
        /// Per joint, how far it reads from where it should once the goals are
        /// written: motion nothing commanded.
        stray: [f64; JointId::COUNT],
        /// What a provisioned register holds.
        provision: Vec<(RegId, RegValue)>,
        /// Whether a joint the goals pulled is still travelling when it is read
        /// back, rather than having arrived.
        travelling: bool,
        /// One write to answer with something other than success. Separate from
        /// the read knob because several registers are both read and written in
        /// one sequence, and which of the two fails is the whole question.
        fail_write: Option<(u8, RegId, BusResult)>,
        /// One read to answer with something other than success.
        fail_read: Option<(u8, RegId, BusResult)>,
        /// The goals the servos are holding, once each has been written.
        goals: JointVector,
        written: [bool; JointId::COUNT],
        poll: usize,
        waits: usize,
        log: Vec<(SeqStep, BusRequest)>,
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
            silent: [false; JointId::COUNT],
            sweeps: vec![7.4],
            sag: [0.0; JointId::COUNT],
            health: [0; JointId::COUNT],
            torque: [0; JointId::COUNT],
            present,
            held: JointVector::default(),
            shadow_gap: [0.0; JointId::COUNT],
            enable_shift: [0.0; JointId::COUNT],
            load: [0.0; JointId::COUNT],
            stray: [0.0; JointId::COUNT],
            provision: vec![
                (RegId::OperatingMode, RegValue::U8(3)),
                (RegId::HomingOffset, RegValue::I32(1024)),
                (RegId::Shutdown, RegValue::U8(0x34)),
            ],
            travelling: false,
            fail_write: None,
            fail_read: None,
            goals: JointVector::default(),
            written: [false; JointId::COUNT],
            poll: 0,
            waits: 0,
            log: Vec::new(),
        }
    }

    impl Machine {
        fn provisioned_as(mut self, reg: RegId, value: RegValue) -> Self {
            for cell in &mut self.provision {
                if cell.0 == reg {
                    cell.1 = value;
                }
            }
            self
        }

        fn value(&mut self, row: usize, reg: RegId) -> RegValue {
            match reg {
                RegId::ModelNumber => RegValue::U16(self.models[row]),
                RegId::PresentInputVoltage => {
                    let volts = self.sweeps[self.poll.min(self.sweeps.len() - 1)] - self.sag[row];
                    if row + 1 == JointId::COUNT {
                        self.poll += 1;
                    }
                    RegValue::Volts(volts)
                }
                RegId::HardwareErrorStatus => RegValue::U8(self.health[row]),
                RegId::TorqueEnable => RegValue::U8(self.torque[row]),
                RegId::PresentPosition => RegValue::Radians(self.position(row)),
                RegId::GoalPosition => RegValue::Radians(self.goal(row)),
                other => self
                    .provision
                    .iter()
                    .find(|(reg, _)| *reg == other)
                    .map_or(RegValue::U8(0), |(_, value)| *value),
            }
        }

        /// Which servos are holding torque now, in the shape the enable
        /// assertions compare against.
        fn enabled(&self) -> [bool; JointId::COUNT] {
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
        /// been written — short of that while it is still travelling there, and
        /// off it by any stray motion the test asked for.
        fn position(&self, row: usize) -> f64 {
            let mut angle = angle_at(&self.present, row);
            if self.torque[row] != 0 {
                angle += self.enable_shift[row];
            }
            if !self.written[row] {
                return angle;
            }
            let settled = angle_at(&self.goals, row) - self.load[row];
            if self.travelling && (settled - angle).abs() > TRAVEL {
                // Short of where it is going, on the side it was pulled from.
                return settled - TRAVEL.copysign(settled - angle) + self.stray[row];
            }
            settled + self.stray[row]
        }
    }

    impl ScriptedBus for Machine {
        fn answer(&mut self, step: SeqStep, request: BusRequest) -> BusResult {
            self.log.push((step, request));
            let row = SERVO_IDS
                .iter()
                .position(|id| *id == request.id())
                .expect("addressed to a servo on this bus");
            if self.silent[row] {
                return BusResult::NoAnswer;
            }
            let scripted = match request {
                BusRequest::WriteRegVerified { .. } => self.fail_write,
                _ => self.fail_read,
            };
            if let Some((id, reg, result)) = scripted
                && request.id() == id
                && request.reg() == Some(reg)
            {
                return result;
            }
            match request {
                BusRequest::Ping { .. } => BusResult::Pinged {
                    model: self.models[row],
                },
                BusRequest::ReadReg { reg, .. } => BusResult::Value(self.value(row, reg)),
                BusRequest::WriteRegVerified { reg, value, .. } => {
                    match (reg, value) {
                        // A goal written to a limp servo is dropped on the
                        // floor, which is the whole reason the sequence enables
                        // torque first.
                        (RegId::GoalPosition, RegValue::Radians(angle)) => {
                            if self.torque[row] != 0 {
                                self.goals.set(JointId::ALL[row], angle);
                                self.written[row] = true;
                            }
                        }
                        (RegId::TorqueEnable, RegValue::U8(1)) => {
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
        for (row, joint) in JointId::ALL.into_iter().enumerate() {
            present.set(joint, angle_at(&held, row) - droop);
        }
        Machine {
            present,
            held,
            load: [droop; JointId::COUNT],
            torque: [1; JointId::COUNT],
            ..bus()
        }
    }

    /// The shared driver, against this crate's arming sequencer.
    fn drive(cfg: &ArmConfig, machine: &mut Machine) -> Result<ArmSummary, SeqError> {
        let mut seq = ArmSequencer::new(
            cfg,
            &HeadGeometry::default(),
            &EnvelopeConfig::default(),
            &FkOptions::default(),
        );
        crate::testutil::drive(&mut seq, machine)
    }

    /// A configuration that actually checks something: position mode on all nine,
    /// with the two registers nobody has established a correct value for recorded
    /// rather than judged.
    fn provisioned_config() -> ArmConfig {
        let mut expected = ProvisionTable::new();
        assert!(expected.set_all(
            RegId::OperatingMode,
            ProvisionExpect::Check(RegValue::U8(3))
        ));
        assert!(expected.set_all(RegId::HomingOffset, ProvisionExpect::Record));
        assert!(expected.set_all(RegId::Shutdown, ProvisionExpect::Record));
        ArmConfig {
            expected,
            ..config()
        }
    }

    fn writes(log: &[(SeqStep, BusRequest)], reg: RegId) -> usize {
        log.iter()
            .filter(|(_, request)| {
                matches!(request, BusRequest::WriteRegVerified { reg: written, .. } if *written == reg)
            })
            .count()
    }

    /// The whole sequence against a machine that arms: the phase order, the
    /// three order properties that are the safety content of it, and the records
    /// it hands back.
    #[test]
    fn arming_runs_its_phases_in_order_and_records_what_it_found() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let summary = drive(&cfg, &mut machine).expect("this machine arms");

        let mut phases: Vec<SeqStep> = Vec::new();
        for (step, _) in &machine.log {
            if phases.last() != Some(step) {
                phases.push(*step);
            }
        }
        assert_eq!(
            phases,
            vec![
                SeqStep::Presence,
                SeqStep::Identity,
                SeqStep::Provision,
                SeqStep::VoltageGate,
                SeqStep::Health,
                SeqStep::PoseAndDatum,
                SeqStep::StateDiscovery,
                SeqStep::GoalShadow,
                SeqStep::GainsProfiles,
                SeqStep::PinAndEnable,
            ]
        );

        // No write of any kind precedes the supply gate.
        let first_write = machine
            .log
            .iter()
            .position(|(_, request)| matches!(request, BusRequest::WriteRegVerified { .. }))
            .expect("arming writes");
        let last_voltage = machine
            .log
            .iter()
            .rposition(|(step, _)| *step == SeqStep::VoltageGate)
            .expect("the gate ran");
        assert!(first_write > last_voltage);

        // Every goal-shadow read precedes every enable: the check that says a
        // servo cannot slam when its torque comes on is complete on all nine
        // before any of them takes torque.
        let last_shadow = machine
            .log
            .iter()
            .rposition(|(step, _)| *step == SeqStep::GoalShadow)
            .expect("the shadow reads ran");
        let first_enable = machine
            .log
            .iter()
            .position(|(_, request)| {
                matches!(
                    request,
                    BusRequest::WriteRegVerified {
                        reg: RegId::TorqueEnable,
                        ..
                    }
                )
            })
            .expect("arming enables torque");
        assert!(last_shadow < first_enable);

        // Every servo's torque is enabled before any servo's goal is written:
        // with torque off the register does not keep a write, so a goal written
        // there would be a write nobody could verify and nothing would store.
        let last_enable = machine
            .log
            .iter()
            .rposition(|(_, request)| {
                matches!(
                    request,
                    BusRequest::WriteRegVerified {
                        reg: RegId::TorqueEnable,
                        ..
                    }
                )
            })
            .expect("arming enables torque");
        let first_goal = machine
            .log
            .iter()
            .position(|(_, request)| {
                matches!(
                    request,
                    BusRequest::WriteRegVerified {
                        reg: RegId::GoalPosition,
                        ..
                    }
                )
            })
            .expect("arming pins");
        assert!(last_enable < first_goal);

        let count = |step| machine.log.iter().filter(|(s, _)| *s == step).count();
        assert_eq!(count(SeqStep::Presence), JointId::COUNT);
        assert_eq!(count(SeqStep::Identity), JointId::COUNT);
        assert_eq!(count(SeqStep::Provision), 3 * JointId::COUNT);
        assert_eq!(count(SeqStep::VoltageGate), JointId::COUNT);
        assert_eq!(count(SeqStep::Health), JointId::COUNT);
        assert_eq!(count(SeqStep::PoseAndDatum), JointId::COUNT);
        assert_eq!(count(SeqStep::StateDiscovery), JointId::COUNT);
        assert_eq!(count(SeqStep::GoalShadow), JointId::COUNT);
        // Gains, then acceleration and velocity, per servo.
        assert_eq!(count(SeqStep::GainsProfiles), 3 * JointId::COUNT);
        // An enable, a position read, a goal and the arrival read, per servo.
        assert_eq!(count(SeqStep::PinAndEnable), 4 * JointId::COUNT);

        let pins = pin_goals(&cfg, &machine.present).expect("inside the gate");
        assert_eq!(summary.rest.joints, machine.present);
        assert_eq!(summary.armed.joints, pins.pinned);
        assert_eq!(
            machine.goals, pins.pinned,
            "the goals in the servos are the pins"
        );
        assert_eq!(summary.pull_in, pins.pull_in);
        assert!(summary.worst_pull_in() < cfg.max_pin_pull_in);
        // Nothing was found holding torque, so there is no droop to record, and
        // this fixture's torque-on moves nothing.
        assert_eq!(summary.droop, [None; JointId::COUNT]);
        assert_eq!(summary.post_enable_shift, [0.0; JointId::COUNT]);
        assert_eq!(summary.voltage_polls, 1);
        assert_eq!(machine.waits, 0);
        assert_eq!(summary.models, machine.models);
        assert_eq!(summary.voltages, [7.4; JointId::COUNT]);
        assert_eq!(summary.torque_before, [false; JointId::COUNT]);
        assert_eq!(summary.health[0], ServoHealth { id: 10, bits: 0 });
        assert_eq!(
            summary
                .provisioned
                .at(JointId::Leg(5), RegId::OperatingMode),
            Some(RegValue::U8(3))
        );
        assert_eq!(
            summary.provisioned.at(JointId::BodyYaw, RegId::Shutdown),
            Some(RegValue::U8(0x34))
        );
        assert_eq!(summary.provisioned.count(), 3 * JointId::COUNT);
        assert_eq!(
            summary
                .provisioned
                .at(JointId::BodyYaw, RegId::CurrentLimit),
            None
        );

        // The armed record is what a tick is started from, and it is below the
        // clearance floor at this rest: the margin baseline is what carries the
        // first lift, exactly as it does off the fixture in `tick.rs`.
        let state = crate::tick::MotionState::new_armed(&summary.armed);
        assert_eq!(*state.last_goal(), pins.pinned);
        assert!(summary.armed.min_margin > 0.0);
        assert!(summary.armed.min_margin < EnvelopeConfig::default().min_toggle_margin);
    }

    /// A limp servo whose goal register does not mirror its measured position
    /// stops arming before any torque is enabled, naming the servo and both
    /// values.
    ///
    /// This is the property the whole enable-first order rests on: torque is put
    /// on nine servos without a goal having been written, which is safe only
    /// because each one's goal is where the joint already stands.
    #[test]
    fn a_goal_that_does_not_shadow_its_present_stops_arming_before_any_torque() {
        let cfg = provisioned_config();
        let mut shadow_gap = [0.0; JointId::COUNT];
        shadow_gap[4] = 3.0_f64.to_radians();
        let mut machine = Machine {
            shadow_gap,
            ..bus()
        };
        let error = drive(&cfg, &mut machine).expect_err("servo 14's goal is not its present");
        let SeqError::GoalShadowMismatch {
            context,
            joint,
            goal,
            present,
            tolerance,
        } = error
        else {
            panic!("expected a goal-shadow refusal, got {error}");
        };
        assert_eq!(joint, JointId::Leg(3));
        assert_eq!(context.step, SeqStep::GoalShadow);
        assert_eq!(context.id, 14);
        assert_eq!(context.reg, Some(RegId::GoalPosition));
        assert!((goal - present - shadow_gap[4]).abs() < 1e-12);
        assert_eq!(tolerance, cfg.goal_shadow_tolerance);

        // Not one servo took torque, and not one goal was written: the machine
        // is exactly as it was found, which is the state a hand can still put
        // right.
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), 0);
        assert_eq!(writes(&machine.log, RegId::GoalPosition), 0);
        assert_eq!(machine.enabled(), [false; JointId::COUNT]);
        // And the gains phase never ran either: the shadow reads come first.
        assert_eq!(writes(&machine.log, RegId::PositionGains), 0);

        // A gap inside the gate is ordinary read wobble and arms.
        let mut shadow_gap = [0.0; JointId::COUNT];
        shadow_gap[4] = 1.0_f64.to_radians();
        let mut machine = Machine {
            shadow_gap,
            ..bus()
        };
        drive(&cfg, &mut machine).expect("a gap inside the gate is wobble");
    }

    /// A servo found already holding torque is exempt from the shadow check —
    /// its goal is a real target, not a mirror — pins at that goal rather than
    /// at the position it has sagged to, and has the sag recorded.
    ///
    /// Pinning at the sag would lower the target by the sag on every re-arm, and
    /// every command in this bench re-arms.
    #[test]
    fn a_servo_found_holding_torque_pins_at_its_goal_and_records_its_droop() {
        let cfg = provisioned_config();
        let droop = 0.6_f64.to_radians();
        let mut machine = holding(droop);
        let summary = drive(&cfg, &mut machine).expect("a holding machine re-arms");

        assert_eq!(summary.torque_before, [true; JointId::COUNT]);
        for (row, recorded) in summary.droop.into_iter().enumerate() {
            let gap = recorded.expect("every servo was found holding torque");
            assert!((gap - droop).abs() < 1e-12, "servo {row} droops {gap}");
        }

        // The pins are the held goals brought into their windows — not the
        // sagged positions, which sit a droop below them.
        let held = machine.held;
        let expected = pin_goals_from(&cfg, &held, &machine.present).expect("inside the gate");
        assert_eq!(summary.armed.joints, expected.pinned);
        assert_eq!(machine.goals, expected.pinned);
        // Re-arming twice does not ratchet the target down: the second arm pins
        // at the same angles the first one left.
        let first = summary.armed.joints;
        let mut again = machine.clone();
        again.log.clear();
        let second = drive(&cfg, &mut again).expect("a second re-arm");
        assert_eq!(second.armed.joints, first);
    }

    /// Torque coming on can renormalise a servo's reported position onto a
    /// single turn. That is absorbed rather than refused: the pins are computed
    /// from the reading taken after the enables, and the shift is recorded.
    #[test]
    fn a_position_that_jumps_when_torque_comes_on_is_absorbed_and_recorded() {
        let cfg = provisioned_config();
        let mut present = joints_at(&rest_head_pose());
        present.body_yaw = 0.35;
        // An antenna settled past the half turn, which is where this platform's
        // own park leaves them, and which is the reading a renormalisation moves
        // by a whole turn.
        present.antennas = [0.20, -0.15 - core::f64::consts::TAU];
        let mut enable_shift = [0.0; JointId::COUNT];
        enable_shift[8] = core::f64::consts::TAU;
        let mut machine = Machine {
            present,
            enable_shift,
            ..bus()
        };
        let summary = drive(&cfg, &mut machine).expect("a renormalised antenna arms");

        assert!((summary.post_enable_shift[8] - core::f64::consts::TAU).abs() < 1e-12);
        assert_eq!(summary.post_enable_shift[..8], [0.0; 8]);
        // Where it was found, and where the pins put it: the record keeps the
        // reading as it came, and the pin is off the post-enable frame.
        assert_eq!(summary.rest.joints.antennas[1], present.antennas[1]);
        assert!((summary.armed.joints.antennas[1] - (-0.15)).abs() < 1e-12);
        assert_eq!(machine.goals.antennas[1], summary.armed.joints.antennas[1]);
    }

    /// The arrival check, three ways: a joint the pins did not move that moves
    /// anyway is a fault; a pulled joint still travelling is inside the corridor
    /// and passes; a pulled joint outside the corridor is a fault.
    #[test]
    fn the_arrival_check_admits_travel_and_refuses_uncommanded_motion() {
        let cfg = provisioned_config();

        // Nothing pulled this joint — its window is not what put it where it is
        // — so any motion at all is motion nothing commanded.
        let mut stray = [0.0; JointId::COUNT];
        stray[0] = 1.0_f64.to_radians();
        let mut machine = Machine { stray, ..bus() };
        let error = drive(&cfg, &mut machine).expect_err("the body moved on its own");
        let SeqError::PinUnstable {
            context,
            joint,
            pinned,
            before,
            present,
        } = error
        else {
            panic!("expected a pin-unstable refusal, got {error}");
        };
        assert_eq!(joint, JointId::BodyYaw);
        assert_eq!(context.step, SeqStep::PinAndEnable);
        assert_eq!(context.id, 10);
        assert_eq!(pinned, before, "the body yaw pin is where it stood");
        assert!((present - before - stray[0]).abs() < 1e-12);
        // The goals were all written first: this is the read that follows them.
        assert_eq!(writes(&machine.log, RegId::GoalPosition), JointId::COUNT);

        // The legs this rest puts outside their windows *are* pulled, and a
        // reading between where they started and where they are going passes:
        // being mid-travel is not a fault.
        let mut machine = Machine {
            travelling: true,
            ..bus()
        };
        let summary = drive(&cfg, &mut machine).expect("mid-travel is inside the corridor");
        let pins = pin_goals(&cfg, &machine.present).expect("inside the gate");
        assert_eq!(summary.armed.joints, pins.pinned);
        assert!(pins.worst_pull_in() > 0.0, "this rest pulls four legs");

        // Past the far end of the corridor by more than the tolerance is a
        // fault: overshoot is not arrival. This rest pulls the first leg
        // upwards, so a stray above its pin is outside the corridor's far end.
        let mut stray = [0.0; JointId::COUNT];
        stray[1] = 2.0 * cfg.recheck_tolerance;
        let mut machine = Machine { stray, ..bus() };
        let error = drive(&cfg, &mut machine).expect_err("that leg overshot its goal");
        let SeqError::PinUnstable { joint, .. } = error else {
            panic!("expected a pin-unstable refusal, got {error}");
        };
        assert_eq!(joint, JointId::Leg(0));

        // And past the near end is a fault too: a pulled joint that ends up
        // further from its goal than where it started is going the wrong way,
        // which is a servo fighting its pin rather than travelling to it.
        let mut stray = [0.0; JointId::COUNT];
        stray[1] = -10.0_f64.to_radians();
        let mut machine = Machine { stray, ..bus() };
        let error = drive(&cfg, &mut machine).expect_err("that leg went the wrong way");
        let SeqError::PinUnstable {
            joint,
            pinned,
            before,
            present,
            ..
        } = error
        else {
            panic!("expected a pin-unstable refusal, got {error}");
        };
        assert_eq!(joint, JointId::Leg(0));
        assert!(pinned > before, "this rest pulls that leg upwards");
        assert!(
            present < before,
            "the reading has to be below where it started for this test to say \
             anything: {present} against {before}"
        );
    }

    /// The pull-in gate is measured against where the joint is, not against the
    /// goal the pin came from.
    ///
    /// A machine found holding torque pins at the goal it holds, so on a servo
    /// that has been dragged well away from that goal — or is failing to reach
    /// it — the pin and the basis agree while the position does not. The gate
    /// bounds that gap, and it refuses before a single enable, which is what
    /// keeps the machine hand-recoverable. Thirteen degrees of sag: past the
    /// twelve-degree gate, and short of the sag that makes the resting pose
    /// itself implausible.
    #[test]
    fn a_held_goal_far_from_the_position_it_is_measured_at_stops_arming() {
        let cfg = provisioned_config();
        let sag = 13.0_f64.to_radians();
        let mut machine = holding(sag);
        let error = drive(&cfg, &mut machine).expect_err("that goal is nowhere near that leg");
        let SeqError::PullInTooLarge {
            context,
            joint,
            pull_in,
            limit,
        } = error
        else {
            panic!("expected a pull-in refusal, got {error}");
        };
        assert_eq!(joint, JointId::Leg(0));
        assert_eq!(context.step, SeqStep::PinAndEnable);
        assert_eq!(context.reg, Some(RegId::GoalPosition));
        assert!((pull_in - sag).abs() < 1e-9, "the pull is the whole sag");
        assert_eq!(limit, cfg.max_pin_pull_in);
        // Before any enable: the servos are still exactly as they were found,
        // which is the state a hand can put right.
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), 0);
        assert_eq!(writes(&machine.log, RegId::GoalPosition), 0);
    }

    /// An antenna found holding torque is pinned at the goal it holds, not at
    /// the position it has sagged to — the same rule as every other joint, and
    /// the reason re-arming does not ratchet a target down by the sag each time.
    #[test]
    fn a_held_antenna_is_pinned_at_its_held_goal() {
        let cfg = provisioned_config();
        let reach = 0.25;
        let mut machine = holding(2.0_f64.to_radians());
        // One antenna sagging much further than the rest of the machine: its
        // goal is where it is held, its position a quarter radian below.
        machine.present.antennas[0] = machine.held.antennas[0] - reach;
        machine.load[7] = reach;
        let summary = drive(&cfg, &mut machine).expect("a sagging antenna is not a refusal");
        assert!((summary.armed.joints.antennas[0] - machine.held.antennas[0]).abs() < 1e-12);
        assert!((summary.rest.joints.antennas[0] - machine.present.antennas[0]).abs() < 1e-12);
    }

    /// A joint the pins did not move is judged against its own reading a sweep
    /// earlier, not against the corridor a pulled joint gets.
    ///
    /// The two branches only differ on a machine found holding torque: there the
    /// pin is the goal it holds and the reading before is a sag below it, so the
    /// corridor is as wide as the sag while the unpulled comparison is the
    /// arrival tolerance wide. A stray a fifth of the sag is inside that
    /// corridor and outside the tolerance, so it separates them.
    #[test]
    fn an_unpulled_joint_is_not_judged_by_the_pulled_joint_corridor() {
        let cfg = provisioned_config();
        let droop = 5.0_f64.to_radians();
        let mut stray = [0.0; JointId::COUNT];
        stray[1] = 1.0_f64.to_radians();
        let mut machine = Machine {
            stray,
            ..holding(droop)
        };
        let error = drive(&cfg, &mut machine).expect_err("that leg moved on its own");
        let SeqError::PinUnstable {
            joint,
            pinned,
            before,
            present,
            ..
        } = error
        else {
            panic!("expected a pin-unstable refusal, got {error}");
        };
        assert_eq!(joint, JointId::Leg(0));
        assert!((present - before - stray[1]).abs() < 1e-9);
        assert!(
            (pinned - before).abs() > 4.0 * cfg.recheck_tolerance,
            "the sag has to be wide against the tolerance for this test to say \
             anything: pin {pinned}, reading before {before}"
        );
        // Inside the corridor the pulled branch would draw, which is what makes
        // the two branches distinguishable here.
        assert!(present > before.min(pinned) && present < before.max(pinned));
    }

    /// The arrival check compares an unpulled joint against a reading taken
    /// under the same load, so a servo holding a standing offset from its target
    /// arms without the offset having to fit inside a tolerance nobody has
    /// measured.
    #[test]
    fn a_droop_far_past_the_arrival_tolerance_is_not_a_refusal() {
        let cfg = provisioned_config();
        // Ten times the arrival tolerance and still inside the pull-in gate,
        // which is what bounds a held goal sitting far from its position.
        let droop = 10.0 * cfg.recheck_tolerance;
        let mut machine = holding(droop);
        let summary = drive(&cfg, &mut machine).expect("a drooping servo is not unstable");
        assert!(
            summary.droop[0].expect("found holding torque").abs() > cfg.recheck_tolerance,
            "the droop has to exceed the tolerance for this test to say anything"
        );
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
        assert_eq!(machine.log.len(), JointId::COUNT);
        assert_eq!(
            error.to_string(),
            "presence of servo 13: no answer from servos 13, 16"
        );

        let mut machine = Machine {
            silent: [true; JointId::COUNT],
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
        assert_eq!(summary.voltage_polls, 3);
        assert_eq!(machine.waits, 2);
        assert_eq!(summary.voltages, [7.4; JointId::COUNT]);

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
        assert_eq!(readings, [5.0; JointId::COUNT]);
        assert_eq!(lowest, 5.0);
        assert!(waited >= cfg.voltage_budget);
        assert_eq!(
            machine.waits,
            (cfg.voltage_budget.as_millis() / cfg.voltage_poll_period.as_millis()) as usize
        );
        assert_eq!(writes(&machine.log, RegId::PositionGains), 0);
    }

    /// A resting position that is not a number stops arming where it is read.
    ///
    /// It closes no linkage, sits inside no travel window and would become a
    /// goal that means nothing, so it is refused at the sweep rather than
    /// carried into the solver — which would report the same machine as a rest
    /// pose nobody can place and name the wrong servo.
    #[test]
    fn a_resting_reading_nobody_can_place_stops_arming() {
        let mut machine = bus();
        machine.present.legs[3] = f64::NAN;

        let error = drive(&provisioned_config(), &mut machine)
            .expect_err("a leg that reads as not-a-number is not a pose");
        let SeqError::UnplaceableAngle {
            context,
            joint,
            angle,
        } = error
        else {
            panic!("expected an unplaceable reading, got {error}");
        };
        assert_eq!(joint, JointId::Leg(3));
        assert_eq!(context.step, SeqStep::PoseAndDatum);
        assert_eq!(context.id, SERVO_IDS[4]);
        assert_eq!(context.reg, Some(RegId::PresentPosition));
        assert!(angle.is_nan(), "{angle}");
        assert!(
            !machine
                .log
                .iter()
                .any(|(_, request)| matches!(request, BusRequest::WriteRegVerified { .. })),
            "the refusal came after writing to the machine"
        );
    }

    /// Each read phase's own refusal, and the property they share: a machine that
    /// fails a check is never written to and never has its torque enabled.
    #[test]
    fn a_failed_check_stops_arming_with_nothing_written() {
        let cfg = provisioned_config();
        let mut unsolvable = bus();
        unsolvable.present.legs[5] += 1.0;
        let cases = [
            bus().provisioned_as(RegId::OperatingMode, RegValue::U8(1)),
            Machine {
                health: [0, 0, 0, 0x20, 0, 0, 0, 0, 0],
                ..bus()
            },
            unsolvable,
            Machine {
                models: [1200, 1190, 1190, 1191, 1190, 1190, 1190, 1180, 1180],
                ..bus()
            },
        ];
        for mut machine in cases {
            let error = drive(&cfg, &mut machine).expect_err("this machine does not arm");
            assert!(
                !machine
                    .log
                    .iter()
                    .any(|(_, request)| matches!(request, BusRequest::WriteRegVerified { .. })),
                "{error} was raised after writing to the machine"
            );
        }

        let mut machine = bus().provisioned_as(RegId::OperatingMode, RegValue::U8(1));
        assert_eq!(
            drive(&cfg, &mut machine)
                .expect_err("position mode is not optional")
                .to_string(),
            "provisioning of servo 10, operating mode: provisioned as 3, holds 1"
        );

        let mut machine = Machine {
            health: [0, 0, 0, 0x20, 0, 0, 0, 0, 0],
            ..bus()
        };
        assert_eq!(
            drive(&cfg, &mut machine)
                .expect_err("an overload latch is not a warning")
                .to_string(),
            "health of servo 13, hardware error status: hardware error bits 0b00100000"
        );

        let mut machine = bus();
        machine.present.legs[5] += 1.0;
        let error = drive(&cfg, &mut machine).expect_err("those angles close no loop");
        assert!(
            matches!(error, SeqError::RestPoseImplausible { .. }),
            "{error}"
        );
        assert_eq!(error.context().step, SeqStep::PoseAndDatum);

        // A supply dip the servo rode out is recorded and armed through: it is
        // the one bit that means the platform is fine.
        let mut machine = Machine {
            health: [1; JointId::COUNT],
            ..bus()
        };
        let summary = drive(&cfg, &mut machine).expect("a voltage latch is not a fault");
        assert_eq!(summary.health[4], ServoHealth { id: 14, bits: 1 });
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

    /// The pins are what the tick's first trajectory starts from, so a pinned
    /// pose the envelope refuses stops arming — on what the pin has no fence of
    /// its own for, which is the body yaw a hand can turn anywhere in its cap.
    #[test]
    fn a_pinned_pose_outside_the_envelope_stops_arming() {
        let cfg = provisioned_config();
        let env = EnvelopeConfig::default();

        let mut machine = bus();
        machine.present.body_yaw = env.body_yaw_limit + 0.05;
        let error = drive(&cfg, &mut machine).expect_err("the body is turned past its cap");
        let SeqError::PinnedPoseOutsideEnvelope {
            context,
            violations,
        } = error
        else {
            panic!("expected an envelope refusal, got {error}");
        };
        assert!(violations.body_yaw);
        assert_eq!(context.step, SeqStep::PinAndEnable);
        assert_eq!(context.id, 10);
        assert_eq!(
            error.to_string(),
            "pin and enable of servo 10, goal position: the pinned pose is outside the \
             envelope (body yaw out of range)"
        );
        // Refused before the pin was written and before any torque went on.
        assert_eq!(writes(&machine.log, RegId::GoalPosition), 0);
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), 0);

        // The rest the pin exists for still arms: below the clearance floor is
        // the one verdict arming ignores.
        let summary = drive(&cfg, &mut bus()).expect("a tight rest is not a refusal");
        assert!(summary.armed.min_margin < env.min_toggle_margin);
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
            let summary = drive(&cfg, &mut machine).expect("a parked antenna is not a refusal");

            // Found and armed are the same angles: an antenna has nowhere to be
            // pulled to.
            assert_eq!(summary.rest.joints.antennas, antennas);
            assert_eq!(summary.armed.joints.antennas, antennas);

            // And those are the goals that reached the machine, on every servo.
            assert_eq!(machine.goals.antennas, antennas);
            assert_eq!(writes(&machine.log, RegId::GoalPosition), JointId::COUNT);
            assert_eq!(writes(&machine.log, RegId::TorqueEnable), JointId::COUNT);
        }
    }

    /// Pins that close no linkage are refused before a byte of them is written,
    /// and the refusal is reported against the pin, not against the writes that
    /// came before it and all landed.
    #[test]
    fn pins_that_place_no_pose_are_refused_before_any_write() {
        let mut cfg = provisioned_config();
        cfg.max_pin_pull_in = PI;
        let mut machine = bus();
        // A window a radian off where the sixth leg is resting: the pin will pull
        // it there, and those six angles close no loop.
        let rest = angle_at(&machine.present, 6);
        cfg.leg_windows[5] = (rest + 1.0, rest + 1.2);

        let error = drive(&cfg, &mut machine).expect_err("those pins place no pose");
        let SeqError::PinnedPoseUnsolvable { context, .. } = error else {
            panic!("expected an unsolvable-pins refusal, got {error}");
        };
        assert_eq!(context.step, SeqStep::PinAndEnable);
        assert_eq!(context.reg, Some(RegId::GoalPosition));
        assert_eq!(writes(&machine.log, RegId::GoalPosition), 0);
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), 0);
        // The phase before it ran in full, which is what makes this the pin's own
        // refusal rather than an earlier check stopping the sequence.
        assert_eq!(
            writes(&machine.log, RegId::PositionGains),
            JointId::COUNT,
            "the gains phase completed before the pins were computed"
        );
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
                context: StepContext::reg(SeqStep::Identity, 13, RegId::ModelNumber),
                model: 1191,
                expected: 1200,
            }
        );
        assert_eq!(
            error.to_string(),
            "identity of servo 13, model number: model 1191, where this platform reports 1200"
        );
        // Stopped at the identity phase: nine pings and nine model reads.
        assert_eq!(machine.log.len(), 2 * JointId::COUNT);

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
                context: StepContext::reg(SeqStep::Identity, 18, RegId::ModelNumber),
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
                context: StepContext::reg(SeqStep::Identity, 10, RegId::ModelNumber),
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
                    read_back: RegValue::U8(0),
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
        assert_eq!(context.step, SeqStep::PinAndEnable);
        assert_eq!(context.id, 14);
        assert_eq!(context.reg, Some(RegId::TorqueEnable));
        assert_eq!(expected, RegValue::U8(1));
        assert_eq!(read_back, RegValue::U8(0));
        // Torque reached the four servos before it and no further; the ones that
        // took it keep it, because nothing here turns torque off.
        assert_eq!(
            machine.enabled(),
            [true, true, true, true, false, false, false, false, false]
        );

        // The same knob on a read: a servo refusing the resting position read
        // stops arming with the status code it sent, before any write at all.
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
                context: StepContext::reg(SeqStep::PoseAndDatum, 15, RegId::PresentPosition),
                code: 7,
            }
        );
        assert_eq!(writes(&machine.log, RegId::PositionGains), 0);

        // And a corrupt reply, which is never retried by anything below.
        let mut machine = Machine {
            fail_write: Some((12, RegId::PositionGains, BusResult::WireCorrupt)),
            ..bus()
        };
        let error = drive(&cfg, &mut machine).expect_err("that reply came back mangled");
        assert_eq!(
            error,
            SeqError::WireCorrupt {
                context: StepContext::reg(SeqStep::GainsProfiles, 12, RegId::PositionGains),
            }
        );
        assert_eq!(machine.enabled(), [false; JointId::COUNT]);
    }

    /// A pose the envelope refuses only once torque is on is refused there, with
    /// the machine holding what it holds and no goal written.
    ///
    /// The pins are recomputed from the reading taken after the enables, so the
    /// envelope's verdict is on the state the machine is actually in — and a
    /// refusal at that point cannot be un-refused by cutting torque, which drops
    /// the head. It stands where it stands and recovery is the operator's.
    #[test]
    fn a_pose_the_envelope_refuses_after_the_enables_stops_there_holding() {
        let cfg = provisioned_config();
        let env = EnvelopeConfig::default();

        let mut machine = bus();
        // Inside the cap where arming finds it, past the cap once torque is on.
        machine.present.body_yaw = env.body_yaw_limit - 0.05;
        machine.enable_shift[0] = 0.1;
        let error = drive(&cfg, &mut machine).expect_err("the body ends up past its cap");
        let SeqError::PinnedPoseOutsideEnvelope { violations, .. } = error else {
            panic!("expected an envelope refusal, got {error}");
        };
        assert!(violations.body_yaw);
        // Every servo took torque and keeps it; not one goal was written.
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), JointId::COUNT);
        assert_eq!(writes(&machine.log, RegId::GoalPosition), 0);
        assert_eq!(machine.enabled(), [true; JointId::COUNT]);
    }

    /// Each servo is enabled once and pinned once, and the pin is a verified
    /// write with torque already on — which is the only condition under which
    /// this platform's goal register keeps what it is given.
    #[test]
    fn every_servo_is_enabled_once_and_pinned_once() {
        let cfg = provisioned_config();
        let mut machine = bus();
        drive(&cfg, &mut machine).expect("this machine arms");

        assert_eq!(writes(&machine.log, RegId::TorqueEnable), JointId::COUNT);
        assert_eq!(writes(&machine.log, RegId::GoalPosition), JointId::COUNT);
        assert_eq!(
            machine.written,
            [true; JointId::COUNT],
            "every goal was written with torque on, so every one was stored"
        );
        // Two position sweeps: the one after the enables that the pins are
        // computed from, and the one after the goals that checks arrival.
        let positions = machine
            .log
            .iter()
            .filter(|(step, request)| {
                *step == SeqStep::PinAndEnable
                    && matches!(
                        request,
                        BusRequest::ReadReg {
                            reg: RegId::PresentPosition,
                            ..
                        }
                    )
            })
            .count();
        assert_eq!(positions, 2 * JointId::COUNT);
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
        let summary = drive(&cfg, &mut machine).expect("this machine arms");

        assert_eq!(summary.provisioned.count(), 0);
        assert!(
            !machine
                .log
                .iter()
                .any(|(step, _)| *step == SeqStep::Provision)
        );
        let mut phases: Vec<SeqStep> = Vec::new();
        for (step, _) in &machine.log {
            if phases.last() != Some(step) {
                phases.push(*step);
            }
        }
        assert_eq!(
            phases,
            vec![
                SeqStep::Presence,
                SeqStep::Identity,
                SeqStep::VoltageGate,
                SeqStep::Health,
                SeqStep::PoseAndDatum,
                SeqStep::StateDiscovery,
                SeqStep::GoalShadow,
                SeqStep::GainsProfiles,
                SeqStep::PinAndEnable,
            ]
        );
    }

    /// A driver that runs a transaction and brings nothing back is reporting
    /// silence, and the refusal names the servo and register that were
    /// outstanding rather than wherever a cursor had got to.
    #[test]
    fn a_driver_that_brings_nothing_back_is_silence() {
        let mut seq = ArmSequencer::new(
            &config(),
            &HeadGeometry::default(),
            &EnvelopeConfig::default(),
            &FkOptions::default(),
        );
        assert_eq!(
            seq.next(Duration::ZERO, None),
            SeqAction::Transact(BusRequest::Ping { id: 10 })
        );
        let SeqAction::Fail(error) = seq.next(Duration::ZERO, None) else {
            panic!("a transaction with no result is not an answer");
        };
        assert_eq!(
            error,
            SeqError::NoAnswer {
                context: StepContext::servo(SeqStep::Presence, 10),
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
        let mut sag = [0.0; JointId::COUNT];
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
        assert_eq!(context.step, SeqStep::VoltageGate);

        // A reading nobody can place is the lowest there is, even beside a
        // number below the floor.
        let mut sag = [0.0; JointId::COUNT];
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
}
