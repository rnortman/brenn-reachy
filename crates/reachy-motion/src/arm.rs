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
    FkError, FkOptions, HeadGeometry, LegAngles, below_limit, forward_kinematics, min_margin,
    neutral_head_pose, pose_margins, rest_head_pose, stow_head_pose,
};

use crate::joints::{JointGroup, JointId, JointSet, JointVector, ServoHealth};
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

/// The supply floor torque is not switched on below, volts.
///
/// Provisional. It is a round number above the point where the servos' own
/// minimum-voltage alarm sits, not a measurement: what the rail does under the
/// current draw of nine servos taking up the head's weight has not been
/// recorded, and until it has, this figure is a guess with a margin.
/// TODO(rail-curve)
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
    // Walked in bus order, so the joint named and the leg pinned both come from
    // the one ordering table rather than from an offset restated here.
    for (joint, target) in basis.joints() {
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

/// What commissioning established about the machine.
///
/// Every register of record a bring-up wants written down, and nothing about
/// where the platform is standing: the pose is the poll's subject and changes
/// under a hand, while these are facts about how the unit was set up. Held by
/// value so a report can be printed after the sequencer is gone.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CommissionSummary {
    /// Each servo's model number, in bus order.
    pub models: [u16; JointId::COUNT],
    /// The supply and error-bit readings commissioning finished on.
    pub rail: Rail,
    /// The provisioned registers as they were found.
    pub provisioned: ProvisionReadings,
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
    pub voltages: [f64; JointId::COUNT],
    /// Each servo's hardware-error byte, as read.
    pub health: [ServoHealth; JointId::COUNT],
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
    pub post_enable_shift: [f64; JointId::COUNT],
    /// The joints this engagement left out of service: never enabled, never
    /// commanded, still measured.
    ///
    /// Only ever antennas — bits on a servo that carries the head refuse the
    /// engagement outright ([`engage_gates`]) — and the set a tick starts its
    /// mask from.
    pub degraded: JointSet,
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
pub fn engage_gates(cfg: &ArmConfig, rail: &Rail) -> Result<JointSet, SeqError> {
    let limit = cfg.min_arm_voltage;
    if let Some((row, lowest)) = worst_below(&rail.voltages, limit) {
        return Err(SeqError::SupplyBelowFloor {
            context: StepContext::reg(
                SeqStep::VoltageGate,
                cfg.ids[row],
                RegId::PresentInputVoltage,
            ),
            readings: rail.voltages,
            lowest,
            limit,
        });
    }

    let mut degraded = JointSet::EMPTY;
    for (row, health) in rail.health.iter().enumerate() {
        if health.healthy_or_voltage_only() {
            continue;
        }
        let joint = JointId::ALL[row];
        if joint.group() == JointGroup::Antennas {
            degraded.insert(joint);
            continue;
        }
        // No reboot, ever, and no clearing of the latch: a servo holding this
        // head that is rebooted drops it.
        return Err(SeqError::UnhealthyServo {
            context: StepContext::reg(SeqStep::Health, cfg.ids[row], RegId::HardwareErrorStatus),
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
fn worst_below(readings: &[f64; JointId::COUNT], limit: f64) -> Option<(usize, f64)> {
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

/// Decode one servo's supply reading into its row of a rail sweep.
///
/// The rail's shape is a fact about the machine and not about which sequencer is
/// asking, so commissioning's sweep and the resting watch's read it through the
/// same pair of functions.
fn absorb_volts(
    row: usize,
    into: &mut [f64; JointId::COUNT],
    context: StepContext,
    result: &BusResult,
) -> Result<(), SeqError> {
    into[row] = result.value(context)?.volts(context)?;
    Ok(())
}

/// Decode one servo's latched error byte into its row of a rail sweep.
fn absorb_health_bits(
    row: usize,
    id: u8,
    into: &mut [ServoHealth; JointId::COUNT],
    context: StepContext,
    result: &BusResult,
) -> Result<(), SeqError> {
    into[row] = ServoHealth {
        id,
        bits: result.value(context)?.u8(context)?,
    };
    Ok(())
}

/// Which part of commissioning is running, and the cursor within it.
///
/// Read phases carry a cursor over the nine servos; the provisioning sweep's
/// cursor walks servos × registers; the write phase's cursor walks servos ×
/// registers likewise.
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
    GainsProfiles {
        cursor: usize,
    },
    Complete,
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
            Self::GainsProfiles { .. } | Self::Complete => SeqStep::GainsProfiles,
            // A failure already carries the phase it happened in; taking the
            // name from anywhere else would report a supply gate that refused
            // as a write that never happened.
            Self::Failed(error) => error.context().step,
        }
    }
}

/// Everything commissioning's phases accumulate that no later phase needs in
/// hand.
#[derive(Clone, Copy, Debug, Default)]
struct Records {
    absent: [bool; JointId::COUNT],
    models: [u16; JointId::COUNT],
    voltages: [f64; JointId::COUNT],
    health: [ServoHealth; JointId::COUNT],
    provisioned: ProvisionReadings,
    polls: u32,
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
pub struct CommissionSequencer {
    cfg: ArmConfig,
    phase: Phase,
    pending: Option<BusRequest>,
    records: Records,
}

impl CommissionSequencer {
    /// A sequence ready to run against `cfg`.
    #[must_use]
    pub fn new(cfg: &ArmConfig) -> Self {
        Self {
            cfg: cfg.clone(),
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

    /// The next action, given the previous transaction's result.
    fn emit(&mut self, now: Duration) -> SeqAction<CommissionSummary> {
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
            Phase::GainsProfiles { cursor } => {
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
            Phase::Complete => return SeqAction::Done(self.summary()),
            Phase::Failed(error) => return SeqAction::Fail(error),
        };
        self.pending = Some(request);
        SeqAction::Transact(request)
    }

    fn summary(&self) -> CommissionSummary {
        CommissionSummary {
            models: self.records.models,
            rail: Rail {
                voltages: self.records.voltages,
                health: self.records.health,
            },
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
            Phase::GainsProfiles { cursor } => {
                confirm_write(result, &request, context)?;
                self.phase = if cursor + 1 < 3 * JointId::COUNT {
                    Phase::GainsProfiles { cursor: cursor + 1 }
                } else {
                    Phase::Complete
                };
                Ok(())
            }
            // Terminal: nothing is ever outstanding, so this is unreachable.
            Phase::Complete | Phase::Failed(_) => Ok(()),
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
        absorb_volts(cursor, &mut self.records.voltages, context, result)?;
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
        let Some((row, lowest)) = worst_below(&self.records.voltages, limit) else {
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
            &mut self.records.health,
            context,
            result,
        )?;
        self.phase = if cursor + 1 < JointId::COUNT {
            Phase::Health { cursor: cursor + 1 }
        } else {
            Phase::GainsProfiles { cursor: 0 }
        };
        Ok(())
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

impl Sequencer for CommissionSequencer {
    type Summary = CommissionSummary;

    fn next(&mut self, now: Duration, prior: Option<&BusResult>) -> SeqAction<CommissionSummary> {
        if let Err(error) = self.absorb(now, prior) {
            self.phase = Phase::Failed(error);
        }
        self.emit(now)
    }

    fn step(&self) -> SeqStep {
        self.phase.step()
    }
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

/// Which part of a poll sweep is running.
#[derive(Clone, Copy, Debug)]
enum PollPhase {
    Position { cursor: usize },
    Voltage { cursor: usize },
    Health { cursor: usize },
    Complete,
    Failed(SeqError),
}

impl PollPhase {
    fn step(self) -> SeqStep {
        match self {
            Self::Position { .. } | Self::Complete => SeqStep::PoseAndDatum,
            Self::Voltage { .. } => SeqStep::VoltageGate,
            Self::Health { .. } => SeqStep::Health,
            Self::Failed(error) => error.context().step,
        }
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
pub struct PollSequencer {
    cfg: ArmConfig,
    phase: PollPhase,
    pending: Option<BusRequest>,
    present: JointVector,
    rail: Rail,
    rail_read: bool,
    /// Re-reads already spent on the joint the position sweep is at.
    rereads: u32,
}

impl PollSequencer {
    /// A sweep ready to run, carrying `rail` forward for a
    /// [`PollCadence::Positions`] sweep that does not re-read it.
    #[must_use]
    pub fn new(cfg: &ArmConfig, rail: Rail, cadence: PollCadence) -> Self {
        Self {
            cfg: cfg.clone(),
            phase: PollPhase::Position { cursor: 0 },
            pending: None,
            present: JointVector::default(),
            rail,
            rail_read: matches!(cadence, PollCadence::PositionsAndRail),
            rereads: 0,
        }
    }

    fn read(&self, row: usize, reg: RegId) -> BusRequest {
        BusRequest::ReadReg {
            id: self.cfg.ids[row],
            reg,
        }
    }

    fn emit(&mut self) -> SeqAction<Posture> {
        let request = match self.phase {
            PollPhase::Position { cursor } => self.read(cursor, RegId::PresentPosition),
            PollPhase::Voltage { cursor } => self.read(cursor, RegId::PresentInputVoltage),
            PollPhase::Health { cursor } => self.read(cursor, RegId::HardwareErrorStatus),
            PollPhase::Complete => {
                return SeqAction::Done(Posture {
                    present: self.present,
                    rail: self.rail,
                    rail_read: self.rail_read,
                });
            }
            PollPhase::Failed(error) => return SeqAction::Fail(error),
        };
        self.pending = Some(request);
        SeqAction::Transact(request)
    }

    fn absorb(&mut self, prior: Option<&BusResult>) -> Result<(), SeqError> {
        let Some(request) = self.pending.take() else {
            return Ok(());
        };
        let context = StepContext {
            step: self.phase.step(),
            id: request.id(),
            reg: request.reg(),
        };
        let Some(result) = prior else {
            return Err(SeqError::NoAnswer { context });
        };
        match self.phase {
            PollPhase::Position { cursor } => {
                let Some(angle) = placeable_or_again(cursor, context, result, &mut self.rereads)?
                else {
                    // The phase is left where it is, so the next action reads
                    // the same register again.
                    return Ok(());
                };
                self.present.set(JointId::ALL[cursor], angle);
                self.phase = if cursor + 1 < JointId::COUNT {
                    PollPhase::Position { cursor: cursor + 1 }
                } else if self.rail_read {
                    PollPhase::Voltage { cursor: 0 }
                } else {
                    PollPhase::Complete
                };
            }
            PollPhase::Voltage { cursor } => {
                absorb_volts(cursor, &mut self.rail.voltages, context, result)?;
                self.phase = if cursor + 1 < JointId::COUNT {
                    PollPhase::Voltage { cursor: cursor + 1 }
                } else {
                    PollPhase::Health { cursor: 0 }
                };
            }
            PollPhase::Health { cursor } => {
                absorb_health_bits(
                    cursor,
                    self.cfg.ids[cursor],
                    &mut self.rail.health,
                    context,
                    result,
                )?;
                self.phase = if cursor + 1 < JointId::COUNT {
                    PollPhase::Health { cursor: cursor + 1 }
                } else {
                    PollPhase::Complete
                };
            }
            PollPhase::Complete | PollPhase::Failed(_) => {}
        }
        Ok(())
    }
}

impl Sequencer for PollSequencer {
    type Summary = Posture;

    fn next(&mut self, _now: Duration, prior: Option<&BusResult>) -> SeqAction<Posture> {
        if let Err(error) = self.absorb(prior) {
            self.phase = PollPhase::Failed(error);
        }
        self.emit()
    }

    fn step(&self) -> SeqStep {
        self.phase.step()
    }
}

/// Which part of engaging is running.
#[derive(Clone, Copy, Debug)]
enum EngagePhase {
    Pin { cursor: usize },
    Enable { cursor: usize },
    Settle { cursor: usize },
    Complete(ArmRecord),
    Failed(SeqError),
}

impl EngagePhase {
    fn step(self) -> SeqStep {
        match self {
            Self::Pin { .. } | Self::Enable { .. } | Self::Settle { .. } | Self::Complete(_) => {
                SeqStep::PinAndEnable
            }
            Self::Failed(error) => error.context().step,
        }
    }
}

/// Which joint a write sweep is at, or `None` where nothing is being written.
///
/// The read sweep and the two endings are not write sweeps: the settle reads
/// all nine whatever the health gate said, and a finished or failed sequence
/// has no cursor at all.
fn write_cursor(phase: EngagePhase) -> Option<usize> {
    match phase {
        EngagePhase::Pin { cursor } | EngagePhase::Enable { cursor } => Some(cursor),
        EngagePhase::Settle { .. } | EngagePhase::Complete(_) | EngagePhase::Failed(_) => None,
    }
}

/// Where a write sweep stands once it is done with the joint at its cursor.
///
/// The order of the write sweeps, in one place: the goal writes run the nine
/// and hand over to the enables, the enables run the nine and hand over to the
/// settle reads. Two callers step it — the walk itself, as each answer comes
/// back, and the skip that steps past a joint nothing is written to — and a
/// second copy of the order is a copy that can be left behind by a sweep added
/// to only one of them.
///
/// A cursor never reaches [`JointId::COUNT`]: the last joint of a sweep hands
/// over rather than counting past the end.
fn advanced(phase: EngagePhase) -> EngagePhase {
    match phase {
        EngagePhase::Pin { cursor } if cursor + 1 < JointId::COUNT => {
            EngagePhase::Pin { cursor: cursor + 1 }
        }
        EngagePhase::Pin { .. } => EngagePhase::Enable { cursor: 0 },
        EngagePhase::Enable { cursor } if cursor + 1 < JointId::COUNT => {
            EngagePhase::Enable { cursor: cursor + 1 }
        }
        EngagePhase::Enable { .. } => EngagePhase::Settle { cursor: 0 },
        EngagePhase::Settle { .. } | EngagePhase::Complete(_) | EngagePhase::Failed(_) => phase,
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
pub struct EngageSequencer {
    cfg: ArmConfig,
    geom: HeadGeometry,
    fk: FkOptions,
    phase: EngagePhase,
    pending: Option<BusRequest>,
    rest: ArmRecord,
    pins: PinOutcome,
    measured: JointVector,
    post_enable: JointVector,
    post_enable_shift: [f64; JointId::COUNT],
    torque_written: bool,
    /// The joints the health gate left out of service: their torque is never
    /// enabled, so masked here means limp rather than merely uncommanded.
    degraded: JointSet,
    /// Re-reads already spent on the joint the settle sweep is at.
    rereads: u32,
}

impl EngageSequencer {
    /// A sequence ready to take hold of the machine `posture` describes.
    ///
    /// Everything that can refuse is settled here, before any transaction: the
    /// two torque-on gates, an angle nobody can place, and a set of angles that
    /// closes no linkage. A sequence built over a refusal fails on its first
    /// action having put nothing on the wire.
    ///
    /// The geometry and the solver options come separately because the records
    /// this produces are solved poses, not angles, and both have to be the ones
    /// the tick uses — a record solved against another geometry would hand the
    /// first trajectory a start the machine is not at.
    #[must_use]
    pub fn new(cfg: &ArmConfig, geom: &HeadGeometry, fk: &FkOptions, posture: &Posture) -> Self {
        let mut seq = Self {
            cfg: cfg.clone(),
            geom: geom.clone(),
            fk: *fk,
            phase: EngagePhase::Pin { cursor: 0 },
            pending: None,
            rest: ArmRecord {
                joints: posture.present,
                head_pose_body: Isometry3::identity(),
                margins: [0.0; 6],
                min_margin: 0.0,
            },
            pins: PinOutcome {
                pinned: posture.present,
                pull_in: [0.0; 6],
            },
            measured: posture.present,
            post_enable: posture.present,
            post_enable_shift: [0.0; JointId::COUNT],
            torque_written: false,
            degraded: JointSet::EMPTY,
            rereads: 0,
        };
        if let Err(error) = seq.prepare(posture) {
            seq.phase = EngagePhase::Failed(error);
        }
        seq
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
        self.torque_written
    }

    /// The gates, the pins and the resting record, all before the first write.
    fn prepare(&mut self, posture: &Posture) -> Result<(), SeqError> {
        self.degraded = engage_gates(&self.cfg, &posture.rail)?;
        self.pins = pin_goals(&self.cfg, &posture.present)?;
        // Failure is not a solver problem to retry with a perturbed seed: the
        // angles are what nine servos reported, and angles that place no pose
        // say the model and the machine disagree.
        self.rest = ArmRecord::solve(&self.geom, &self.fk, &posture.present, &rest_pose_seeds())
            .map_err(|cause| SeqError::RestPoseImplausible {
                // Named against the first crank: the failure belongs to the six
                // of them together, and a context names one servo.
                context: StepContext::reg(
                    SeqStep::PinAndEnable,
                    self.cfg.ids[1],
                    RegId::PresentPosition,
                ),
                cause,
            })?;
        Ok(())
    }

    fn write(&self, row: usize, reg: RegId, value: RegValue) -> BusRequest {
        BusRequest::WriteRegVerified {
            id: self.cfg.ids[row],
            reg,
            value,
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
        while let Some(cursor) = write_cursor(self.phase) {
            if !self.degraded.contains(JointId::ALL[cursor]) {
                return;
            }
            self.phase = advanced(self.phase);
        }
    }

    fn emit(&mut self) -> SeqAction<EngageSummary> {
        self.skip_degraded();
        let request = match self.phase {
            EngagePhase::Pin { cursor } => {
                let goal = RegValue::Radians(angle_at(&self.pins.pinned, cursor));
                self.write(cursor, RegId::GoalPosition, goal)
            }
            EngagePhase::Enable { cursor } => {
                self.torque_written = true;
                self.write(cursor, RegId::TorqueEnable, RegValue::U8(1))
            }
            EngagePhase::Settle { cursor } => BusRequest::ReadReg {
                id: self.cfg.ids[cursor],
                reg: RegId::PresentPosition,
            },
            EngagePhase::Complete(armed) => {
                return SeqAction::Done(EngageSummary {
                    rest: self.rest,
                    armed,
                    pins: self.pins,
                    post_enable_shift: self.post_enable_shift,
                    degraded: self.degraded,
                });
            }
            EngagePhase::Failed(error) => return SeqAction::Fail(error),
        };
        self.pending = Some(request);
        SeqAction::Transact(request)
    }

    fn absorb(&mut self, prior: Option<&BusResult>) -> Result<(), SeqError> {
        let Some(request) = self.pending.take() else {
            return Ok(());
        };
        let context = StepContext {
            step: self.phase.step(),
            id: request.id(),
            reg: request.reg(),
        };
        match self.phase {
            EngagePhase::Pin { .. } => {
                // Whatever came back — the mirrored present, silence, a refusal
                // — the walk carries on. See the type's docs: this sweep is
                // insurance against a stale goal register, not a check on one.
                self.phase = advanced(self.phase);
                Ok(())
            }
            EngagePhase::Enable { .. } => {
                let Some(result) = prior else {
                    return Err(SeqError::NoAnswer { context });
                };
                confirm_write(result, &request, context)?;
                self.phase = advanced(self.phase);
                Ok(())
            }
            EngagePhase::Settle { cursor } => {
                let Some(result) = prior else {
                    return Err(SeqError::NoAnswer { context });
                };
                let Some(angle) = placeable_or_again(cursor, context, result, &mut self.rereads)?
                else {
                    // The phase is left where it is, so the next action reads
                    // the same register again.
                    return Ok(());
                };
                self.post_enable.set(JointId::ALL[cursor], angle);
                self.post_enable_shift[cursor] = angle - angle_at(&self.measured, cursor);
                if cursor + 1 < JointId::COUNT {
                    self.phase = EngagePhase::Settle { cursor: cursor + 1 };
                    return Ok(());
                }
                let seeds = armed_pose_seeds(self.rest.head_pose_body);
                let armed = ArmRecord::solve(&self.geom, &self.fk, &self.post_enable, &seeds)
                    .map_err(|cause| SeqError::PinnedPoseUnsolvable {
                        context: StepContext::reg(
                            SeqStep::PinAndEnable,
                            self.cfg.ids[1],
                            RegId::PresentPosition,
                        ),
                        cause,
                    })?;
                self.phase = EngagePhase::Complete(armed);
                Ok(())
            }
            EngagePhase::Complete(_) | EngagePhase::Failed(_) => Ok(()),
        }
    }
}

impl Sequencer for EngageSequencer {
    type Summary = EngageSummary;

    fn next(&mut self, _now: Duration, prior: Option<&BusResult>) -> SeqAction<EngageSummary> {
        if let Err(error) = self.absorb(prior) {
            self.phase = EngagePhase::Failed(error);
        }
        self.emit()
    }

    fn step(&self) -> SeqStep {
        self.phase.step()
    }
}

#[cfg(test)]
mod tests {
    use core::f64::consts::PI;

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
        /// What a provisioned register holds.
        provision: Vec<(RegId, RegValue)>,
        /// One write to answer with something other than success. Separate from
        /// the read knob because several registers are both read and written in
        /// one sequence, and which of the two fails is the whole question.
        fail_write: Option<(u8, RegId, BusResult)>,
        /// One read to answer with something other than success.
        fail_read: Option<(u8, RegId, BusResult)>,
        /// Per servo, how many further position reads answer with something
        /// nobody can place, counted down as they arrive: the corrupt frame a
        /// sweep re-reads past, rather than a joint that has stopped reporting.
        nan_reads: [usize; JointId::COUNT],
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
            provision: vec![
                (RegId::OperatingMode, RegValue::U8(3)),
                (RegId::HomingOffset, RegValue::I32(1024)),
                (RegId::Shutdown, RegValue::U8(0x34)),
            ],
            fail_write: None,
            fail_read: None,
            nan_reads: [0; JointId::COUNT],
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
                RegId::PresentPosition => {
                    if self.nan_reads[row] > 0 {
                        self.nan_reads[row] -= 1;
                        return RegValue::Radians(f64::NAN);
                    }
                    RegValue::Radians(self.position(row))
                }
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
        let commission = commission(cfg, machine)?;
        let posture = poll(cfg, machine, commission.rail, PollCadence::Positions)?;
        let engage = engage(cfg, machine, &posture)?;
        Ok(Armed {
            commission,
            posture,
            engage,
        })
    }

    fn commission(cfg: &ArmConfig, machine: &mut Machine) -> Result<CommissionSummary, SeqError> {
        crate::testutil::drive(&mut CommissionSequencer::new(cfg), machine)
    }

    fn poll(
        cfg: &ArmConfig,
        machine: &mut Machine,
        rail: Rail,
        cadence: PollCadence,
    ) -> Result<Posture, SeqError> {
        crate::testutil::drive(&mut PollSequencer::new(cfg, rail, cadence), machine)
    }

    fn engage(
        cfg: &ArmConfig,
        machine: &mut Machine,
        posture: &Posture,
    ) -> Result<EngageSummary, SeqError> {
        let mut seq = EngageSequencer::new(
            cfg,
            &HeadGeometry::default(),
            &FkOptions::default(),
            posture,
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

    /// How many times `id`'s position was read, which is what a re-read shows
    /// up as.
    fn reads_of(log: &[(SeqStep, BusRequest)], id: u8) -> usize {
        log.iter()
            .filter(|(_, request)| {
                matches!(
                    request,
                    BusRequest::ReadReg {
                        id: read,
                        reg: RegId::PresentPosition,
                    } if *read == id
                )
            })
            .count()
    }

    fn writes(log: &[(SeqStep, BusRequest)], reg: RegId) -> usize {
        log.iter()
            .filter(|(_, request)| {
                matches!(request, BusRequest::WriteRegVerified { reg: written, .. } if *written == reg)
            })
            .count()
    }

    /// The whole path against a machine that engages: the phase order, the
    /// transaction counts, and the records each half hands back.
    #[test]
    fn the_torque_on_path_runs_its_phases_in_order_and_records_what_it_found() {
        let cfg = provisioned_config();
        let mut machine = bus();
        let summary = drive(&cfg, &mut machine).expect("this machine engages");

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
                SeqStep::GainsProfiles,
                SeqStep::PoseAndDatum,
                SeqStep::PinAndEnable,
            ]
        );

        // No write of any kind precedes the supply gate: the gains and profiles
        // go into servo RAM, and a rail browning out is how a servo ends up
        // holding half a configuration.
        let first_write = machine
            .log
            .iter()
            .position(|(_, request)| matches!(request, BusRequest::WriteRegVerified { .. }))
            .expect("commissioning writes");
        let last_voltage = machine
            .log
            .iter()
            .rposition(|(step, _)| *step == SeqStep::VoltageGate)
            .expect("the gate ran");
        assert!(first_write > last_voltage);

        // Commissioning touches torque in neither direction, and reads no
        // position: the machine it hands on is exactly the machine it met.
        let commissioning =
            |step: &SeqStep| !matches!(step, SeqStep::PoseAndDatum | SeqStep::PinAndEnable);
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
                *step == SeqStep::PinAndEnable && request.reg() == Some(RegId::PresentPosition)
            })
            .expect("engaging reads back");
        assert!(last_pin < first_enable);
        assert!(last_enable < first_settle);

        let count = |step| machine.log.iter().filter(|(s, _)| *s == step).count();
        assert_eq!(count(SeqStep::Presence), JointId::COUNT);
        assert_eq!(count(SeqStep::Identity), JointId::COUNT);
        assert_eq!(count(SeqStep::Provision), 3 * JointId::COUNT);
        assert_eq!(count(SeqStep::VoltageGate), JointId::COUNT);
        assert_eq!(count(SeqStep::Health), JointId::COUNT);
        // Gains, then acceleration and velocity, per servo.
        assert_eq!(count(SeqStep::GainsProfiles), 3 * JointId::COUNT);
        // The resting sweep: nine positions and nothing else.
        assert_eq!(count(SeqStep::PoseAndDatum), JointId::COUNT);
        // A pin, an enable and a read-back, per servo. Twenty-seven
        // transactions is the whole cost of a wake word reaching the head.
        assert_eq!(count(SeqStep::PinAndEnable), 3 * JointId::COUNT);

        let pins = pin_goals(&cfg, &machine.present).expect("inside the window");
        assert_eq!(summary.posture.present, machine.present);
        assert_eq!(summary.engage.rest.joints, machine.present);
        assert_eq!(summary.engage.pins.pinned, pins.pinned);
        assert_eq!(summary.engage.pins.pull_in, pins.pull_in);
        // This fixture's torque-on moves nothing, so the pose read back is the
        // pose that was measured.
        assert_eq!(summary.engage.armed.joints, machine.present);
        assert_eq!(summary.engage.post_enable_shift, [0.0; JointId::COUNT]);
        assert_eq!(machine.enabled(), [true; JointId::COUNT]);

        assert_eq!(summary.commission.voltage_polls, 1);
        assert_eq!(machine.waits, 0);
        assert_eq!(summary.commission.models, machine.models);
        assert_eq!(summary.commission.rail.voltages, [7.4; JointId::COUNT]);
        assert_eq!(
            summary.commission.rail.health[0],
            ServoHealth { id: 10, bits: 0 }
        );
        // A positions-only sweep carries the rail forward rather than leaving a
        // gate to judge half a picture.
        assert!(!summary.posture.rail_read);
        assert_eq!(summary.posture.rail, summary.commission.rail);
        assert_eq!(
            summary
                .commission
                .provisioned
                .at(JointId::Leg(5), RegId::OperatingMode),
            Some(RegValue::U8(3))
        );
        assert_eq!(
            summary
                .commission
                .provisioned
                .at(JointId::BodyYaw, RegId::Shutdown),
            Some(RegValue::U8(0x34))
        );
        assert_eq!(summary.commission.provisioned.count(), 3 * JointId::COUNT);
        assert_eq!(
            summary
                .commission
                .provisioned
                .at(JointId::BodyYaw, RegId::CurrentLimit),
            None
        );

        // The armed record is what a tick is started from, and it is below the
        // clearance floor at this rest: the margin baseline is what carries the
        // first lift, exactly as it does off the fixture in `tick.rs`.
        let state =
            crate::tick::MotionState::new_armed(&summary.engage.armed, summary.engage.degraded);
        assert_eq!(*state.last_goal(), machine.present);
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
            .filter_map(|(_, request)| match request {
                BusRequest::WriteRegVerified {
                    reg: RegId::GoalPosition,
                    value: RegValue::Radians(angle),
                    ..
                } => Some(*angle),
                _ => None,
            })
            .collect();
        let expected: Vec<f64> = (0..JointId::COUNT)
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
                read_back: RegValue::Radians(1.0),
            },
        ] {
            let mut machine = Machine {
                fail_write: Some((14, RegId::GoalPosition, answer)),
                ..bus()
            };
            drive(&cfg, &mut machine).expect("a pin that goes nowhere is not a refusal");
            assert_eq!(machine.enabled(), [true; JointId::COUNT]);
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
        assert_eq!(context.step, SeqStep::PinAndEnable);
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
        let mut seq = EngageSequencer::new(&cfg, &geom, &fk, &sagging);
        crate::testutil::drive(&mut seq, &mut machine).expect_err("a low rail does not torque on");
        assert!(!seq.torque_written(), "the gate refused before any write");

        // The pin sweep is not judged, so nothing stops there on the machine's
        // answers — the port itself going away is what ends a walk mid-pin, and
        // the driver reports that rather than the sequencer. Standing in for it:
        // a sequence stopped after one pin write has still written no torque.
        let mut seq = EngageSequencer::new(&cfg, &geom, &fk, &posture);
        assert!(matches!(
            seq.next(Duration::ZERO, None),
            SeqAction::Transact(BusRequest::WriteRegVerified {
                reg: RegId::GoalPosition,
                ..
            })
        ));
        assert!(!seq.torque_written(), "a pin is not an enable");

        let mut machine = Machine {
            fail_write: Some((14, RegId::TorqueEnable, BusResult::NoAnswer)),
            ..bus()
        };
        let mut seq = EngageSequencer::new(&cfg, &geom, &fk, &posture);
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
        assert_eq!(machine.enabled(), [false; JointId::COUNT]);

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
        let mut health = [0; JointId::COUNT];
        health[3] = 0x20;
        let mut machine = Machine { health, ..bus() };
        let summary = commission(&cfg, &mut machine).expect("a latch does not stop commissioning");
        assert_eq!(summary.rail.health[3], ServoHealth { id: 13, bits: 0x20 });
        assert_eq!(machine.enabled(), [false; JointId::COUNT]);
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
            voltages: [7.4; JointId::COUNT],
            health: [ServoHealth::default(); JointId::COUNT],
        };
        assert_eq!(
            engage_gates(&cfg, &healthy).expect("nothing flags"),
            JointSet::EMPTY
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
        assert!(degraded.covers(JointGroup::Antennas));
        assert_eq!(degraded.len(), 2);
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
        let mut health = [0; JointId::COUNT];
        health[7] = 0x20;
        let mut machine = Machine { health, ..bus() };
        let summary = drive(&cfg, &mut machine).expect("an antenna latch does not refuse the head");

        assert_eq!(
            summary.engage.degraded.iter().collect::<Vec<_>>(),
            vec![JointId::AntennaRight]
        );
        let mut expected = [true; JointId::COUNT];
        expected[7] = false;
        assert_eq!(machine.enabled(), expected);

        let engaging: Vec<&BusRequest> = machine
            .log
            .iter()
            .filter(|(step, request)| {
                *step == SeqStep::PinAndEnable && request.id() == SERVO_IDS[7]
            })
            .map(|(_, request)| request)
            .collect();
        assert!(
            engaging
                .iter()
                .all(|request| matches!(request, BusRequest::ReadReg { .. })),
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
        let state =
            crate::tick::MotionState::new_armed(&summary.engage.armed, summary.engage.degraded);
        assert!(state.masked().contains(JointId::AntennaRight));
        assert!(!state.masked().contains(JointId::AntennaLeft));

        // Both of them flagging leaves seven servos holding the head up.
        let mut health = [0; JointId::COUNT];
        health[7] = 0x20;
        health[8] = 0x20;
        let mut machine = Machine { health, ..bus() };
        let summary = drive(&cfg, &mut machine).expect("a flagging pair does not refuse the head");
        assert!(summary.engage.degraded.covers(JointGroup::Antennas));
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
        let mut health = [0; JointId::COUNT];
        health[8] = 0x20;
        let mut machine = Machine { health, ..bus() };
        let first = drive(&cfg, &mut machine).expect("the head engages");
        assert!(first.engage.degraded.contains(JointId::AntennaLeft));

        // A latch that still stands degrades the same joint again.
        machine.torque = [0; JointId::COUNT];
        let again = drive(&cfg, &mut machine).expect("the head engages again");
        assert_eq!(again.engage.degraded, first.engage.degraded);

        // What a REBOOT leaves behind: no latch, and the joint back in service.
        machine.health[8] = 0;
        machine.torque = [0; JointId::COUNT];
        let cleared = drive(&cfg, &mut machine).expect("the head engages once more");
        assert_eq!(cleared.engage.degraded, JointSet::EMPTY);
        assert_eq!(machine.enabled(), [true; JointId::COUNT]);
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
        assert_eq!(joint, JointId::Leg(3));
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
        assert_eq!(machine.enabled(), [true; JointId::COUNT]);

        // And persistence there still refuses, with the machine holding torque
        // — the caller's release is what answers that, not the sweep.
        machine.nan_reads[8] = usize::try_from(PLACE_REREADS).expect("a small count") + 1;
        let error = engage(&cfg, &mut machine, &posture)
            .expect_err("a joint that keeps answering nothing placeable is refused");
        assert!(
            matches!(
                error,
                SeqError::UnplaceableAngle {
                    joint: JointId::AntennaLeft,
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
            voltages: [6.9; JointId::COUNT],
            health: [ServoHealth { id: 0, bits: 0 }; JointId::COUNT],
        };

        let fast = poll(&cfg, &mut machine, carried, PollCadence::Positions)
            .expect("a positions sweep reads");
        assert_eq!(fast.present, machine.present);
        assert_eq!(fast.rail, carried);
        assert!(!fast.rail_read);
        assert_eq!(machine.log.len(), JointId::COUNT);

        machine.log.clear();
        let slow = poll(&cfg, &mut machine, carried, PollCadence::PositionsAndRail)
            .expect("a rail sweep reads");
        assert_eq!(slow.present, machine.present);
        assert_eq!(slow.rail.voltages, [7.1; JointId::COUNT]);
        assert_eq!(slow.rail.health[8], ServoHealth { id: 18, bits: 0 });
        assert!(slow.rail_read);
        assert_eq!(machine.log.len(), 3 * JointId::COUNT);
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
        let mut enable_shift = [0.0; JointId::COUNT];
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
        assert_eq!(machine.enabled(), [true; JointId::COUNT]);
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
        assert_eq!(summary.commission.voltage_polls, 3);
        assert_eq!(machine.waits, 2);
        assert_eq!(summary.commission.rail.voltages, [7.4; JointId::COUNT]);

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

    /// Each commissioning check's own refusal, and the property they share: a
    /// machine that fails one is never written to and never has its torque
    /// enabled.
    #[test]
    fn a_failed_check_stops_commissioning_with_nothing_written() {
        let cfg = provisioned_config();
        let cases = [
            bus().provisioned_as(RegId::OperatingMode, RegValue::U8(1)),
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
                    .any(|(_, request)| matches!(request, BusRequest::WriteRegVerified { .. })),
                "{error} was raised after writing to the machine"
            );
            assert_eq!(machine.enabled(), [false; JointId::COUNT]);
        }

        let mut machine = bus().provisioned_as(RegId::OperatingMode, RegValue::U8(1));
        assert_eq!(
            drive(&cfg, &mut machine)
                .expect_err("position mode is not optional")
                .to_string(),
            "provisioning of servo 10, operating mode: provisioned as 3, holds 1"
        );

        // A supply dip the servo rode out is recorded and engaged through: it is
        // the one bit that means the platform is fine.
        let mut machine = Machine {
            health: [1; JointId::COUNT],
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
        assert_eq!(error.context().step, SeqStep::PinAndEnable);
        // The gates and the solve both run before the first write, so a machine
        // whose pose places nothing is left exactly as it was found.
        assert_eq!(writes(&machine.log, RegId::GoalPosition), 0);
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), 0);
        assert_eq!(machine.enabled(), [false; JointId::COUNT]);
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
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), JointId::COUNT);
        assert_eq!(writes(&machine.log, RegId::GoalPosition), JointId::COUNT);

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
            assert_eq!(writes(&machine.log, RegId::GoalPosition), JointId::COUNT);
            assert_eq!(writes(&machine.log, RegId::TorqueEnable), JointId::COUNT);
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
        assert_eq!(context.step, SeqStep::PinAndEnable);
        assert_eq!(context.reg, Some(RegId::PresentPosition));
        // Every sweep before it ran in full, which is what makes this the
        // read-back's own refusal rather than an earlier check stopping short.
        assert_eq!(writes(&machine.log, RegId::GoalPosition), JointId::COUNT);
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), JointId::COUNT);
        assert_eq!(machine.enabled(), [true; JointId::COUNT]);
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
                context: StepContext::reg(SeqStep::PoseAndDatum, 15, RegId::PresentPosition),
                code: 7,
            }
        );
        assert_eq!(machine.enabled(), [false; JointId::COUNT]);

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
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), JointId::COUNT);
        assert_eq!(writes(&machine.log, RegId::GoalPosition), JointId::COUNT);
        assert_eq!(machine.enabled(), [true; JointId::COUNT]);
    }

    /// Each servo is enabled once and pinned once, and the pose is read back
    /// once: twenty-seven transactions, no servo asked twice.
    #[test]
    fn every_servo_is_enabled_once_and_pinned_once() {
        let cfg = provisioned_config();
        let mut machine = bus();
        drive(&cfg, &mut machine).expect("this machine engages");

        assert_eq!(writes(&machine.log, RegId::TorqueEnable), JointId::COUNT);
        assert_eq!(writes(&machine.log, RegId::GoalPosition), JointId::COUNT);
        // One position sweep in this phase: the read-back after the enables,
        // which is what the trajectory starts from. Nothing is read after it.
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
        assert_eq!(positions, JointId::COUNT);
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
        let summary = drive(&cfg, &mut machine).expect("this machine engages");

        assert_eq!(summary.commission.provisioned.count(), 0);
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
                SeqStep::GainsProfiles,
                SeqStep::PoseAndDatum,
                SeqStep::PinAndEnable,
            ]
        );
    }

    /// A driver that runs a transaction and brings nothing back is reporting
    /// silence, and the refusal names the servo and register that were
    /// outstanding rather than wherever a cursor had got to.
    #[test]
    fn a_driver_that_brings_nothing_back_is_silence() {
        let mut seq = CommissionSequencer::new(&config());
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
