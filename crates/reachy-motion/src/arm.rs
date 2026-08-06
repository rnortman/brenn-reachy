//! Arming's configuration, its two records, and the one clamp in this repo.
//!
//! Arming takes a limp platform to a platform holding itself up: nine servos
//! verified register by register, the supply rail confirmed, the resting pose
//! solved and found plausible, and only then goals pinned
//! where the joints already are and torque enabled. The order of those
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
//! The antennas are pinned the same way, against the envelope's own antenna
//! bound rather than a servo-side window: they have no provisioned window at
//! all, and the platform parks them right at the command bound with torque off,
//! from where they settle a little past it. Pinning them is what makes a machine
//! found in its own parked state armable without moving the bound that the
//! command path enforces. There is deliberately **no pull-in gate on an
//! antenna**: it is a free rotor with no linkage behind it and no mass that
//! matters, its reading is unbounded with torque off, and a gate there would
//! refuse an ordinary resting state with no hazard behind the refusal. Body yaw
//! is pinned where it is: a body turned past its cap is a gross, visible state
//! somebody put it in by hand, and the recovery is that same hand.
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
    LegAngles, below_limit, check_envelope, forward_kinematics, min_margin, outside_limit,
    pose_margins, rest_head_pose, stow_head_pose,
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

/// How far a joint may have moved between its pin and the read after torque
/// came on before arming re-pins it, radians (0.5°).
///
/// Enabling torque in position mode can reset a servo's reported position, so
/// the read after enabling is compared against the pin rather than trusted. The
/// figure is provisional: several counts of the servo's own 0.088° resolution,
/// wide enough not to chase quantisation and narrow enough that a real reset
/// cannot hide inside it. Nothing has measured what the reset actually looks
/// like on this platform.
pub const DEFAULT_REPIN_TOLERANCE: f64 = 0.5 * PI / 180.0;

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
    /// How far a joint may drift between pin and post-enable read, radians.
    pub repin_tolerance: f64,
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
    /// travel window and each antenna inside the envelope's antenna bound.
    pub pinned: JointVector,
    /// Per leg, how far the pin moved it, radians. Zero for a leg already
    /// inside its window, which is the ordinary case on an already-armed
    /// machine.
    pub pull_in: [f64; 6],
    /// Per antenna, how far the pin moved it, radians. Recorded, never gated —
    /// an antenna resting past its bound is an ordinary limp state and the pull
    /// is a free rotor turning a fraction of a turn.
    pub antenna_pull_in: [f64; 2],
}

impl PinOutcome {
    /// The largest pull applied to a leg, radians.
    ///
    /// The legs alone, because they are what the gate is about: the antennas'
    /// pulls are recorded beside them and bounded by nothing.
    #[must_use]
    pub fn worst_pull_in(&self) -> f64 {
        self.pull_in.iter().copied().fold(0.0, f64::max)
    }
}

/// The goals to pin the platform at, given where it says it is.
///
/// Each leg is brought inside its own travel window and each antenna inside
/// `env.antenna_limit`; body yaw is pinned where it is, its provisioned range
/// being the whole turn and a body outside its cap being a state a hand made and
/// a hand undoes. A leg pulled beyond `cfg.max_pin_pull_in` stops arming — the
/// antennas have no such gate — and so does a measured angle nobody can place:
/// pinning a goal to a value that is not a number would send a meaningless write
/// to a servo about to take the head's weight.
pub fn pin_goals(
    cfg: &ArmConfig,
    env: &EnvelopeConfig,
    present: &JointVector,
) -> Result<PinOutcome, SeqError> {
    for (row, (joint, angle)) in present.joints().into_iter().enumerate() {
        if !angle.is_finite() {
            return Err(SeqError::UnplaceableAngle {
                context: StepContext::reg(
                    SeqStep::PinAndEnable,
                    cfg.ids[row],
                    RegId::PresentPosition,
                ),
                joint,
                angle,
            });
        }
    }

    let mut pinned = *present;
    let mut pull_in = [0.0; 6];
    // Walked in bus order, so the joint named, the servo addressed and the leg
    // pinned all come from the one ordering table rather than from an offset
    // restated here.
    for (row, (joint, measured)) in present.joints().into_iter().enumerate() {
        let JointId::Leg(leg) = joint else { continue };
        let leg = usize::from(leg);
        let (low, high) = cfg.leg_windows[leg];

        let angle = if measured < low {
            low
        } else if measured > high {
            high
        } else {
            measured
        };
        pinned.legs[leg] = angle;
        pull_in[leg] = (angle - measured).abs();

        if outside_limit(pull_in[leg], cfg.max_pin_pull_in) {
            return Err(SeqError::PullInTooLarge {
                context: StepContext::reg(SeqStep::PinAndEnable, cfg.ids[row], RegId::GoalPosition),
                joint,
                pull_in: pull_in[leg],
                limit: cfg.max_pin_pull_in,
            });
        }
    }

    let mut antenna_pull_in = [0.0; 2];
    for (side, measured) in present.antennas.into_iter().enumerate() {
        // The bound is symmetric, so the nearer end of it is the one the reading
        // is already on. A reading exactly at the bound is inside it and is
        // pinned untouched, which is where the platform parks them.
        let angle = if outside_limit(measured.abs(), env.antenna_limit) {
            measured.signum() * env.antenna_limit
        } else {
            measured
        };
        pinned.antennas[side] = angle;
        antenna_pull_in[side] = (angle - measured).abs();
    }

    Ok(PinOutcome {
        pinned,
        pull_in,
        antenna_pull_in,
    })
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
    /// Per antenna, how far the first pin pulled it, radians. Nothing gates
    /// this: it is here to be read, since arming a machine found in its parked
    /// state pulls both antennas in a little and the pull is worth seeing.
    pub antenna_pull_in: [f64; 2],
    /// Whether a joint had to be pinned a second time after torque came on.
    pub repinned: bool,
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
    /// the antennas' pulls are gated by nothing and compared against nothing.
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
    /// The goals currently written, or about to be.
    pins: PinOutcome,
    /// The pose those goals hold.
    armed: ArmRecord,
    /// The first pin's per-leg pull, which is the one worth reporting: a re-pin's
    /// pulls are quantisation, and the arm-time pull is the measurement.
    pull_in: [f64; 6],
    /// The first pin's per-antenna pull, carried for the same reason.
    antenna_pull_in: [f64; 2],
    /// Whether a re-pin pass has run.
    repinned: bool,
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
    GainsProfiles {
        cursor: usize,
        rest: ArmRecord,
    },
    PinAndEnable {
        cursor: usize,
        pinning: Pinning,
    },
    Recheck {
        cursor: usize,
        second: bool,
        pinning: Pinning,
    },
    Repin {
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
            Self::GainsProfiles { .. } => SeqStep::GainsProfiles,
            // The pin, its post-enable re-check and its one re-pin are one phase
            // as far as a report is concerned: they are the same decision, and
            // the register named in the context is what separates them.
            Self::PinAndEnable { .. }
            | Self::Recheck { .. }
            | Self::Repin { .. }
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
    recheck: JointVector,
    polls: u32,
}

/// The bus row a set of envelope violations is best reported against.
///
/// The verdict names every failing check itself; this picks the one servo a
/// report has room to address, in the order a reader would want it: the joint
/// with a bound of its own first, and the whole-pose checks — attitude and
/// head-relative yaw — against the first crank, since they belong to the six legs
/// together and a context names one servo.
fn first_violated_row(violations: &EnvelopeViolations) -> usize {
    for leg in 0..6 {
        if violations.unreachable[leg] || violations.window[leg] {
            return 1 + leg;
        }
    }
    if violations.body_yaw {
        return 0;
    }
    if violations.antenna[0] {
        return 7;
    }
    if violations.antenna[1] {
        return 8;
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

/// Arming, as a state machine that touches no port.
///
/// Nine phases in a fixed order, one transaction at a time: every servo present,
/// every servo the kind it should be, every provisioned register as configured,
/// the supply up, nothing latched, the resting pose solved, the torque states
/// recorded, the gains and profiles written, and only then goals pinned and
/// torque enabled. The order is the safety property — no write of any kind
/// happens before the supply gate, and every servo's goal is written before its
/// torque is enabled — and it lives here, in one readable sequence, testable
/// against scripted replies.
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
            Phase::PinAndEnable { cursor, pinning } => {
                let row = cursor / 2;
                if cursor.is_multiple_of(2) {
                    let goal = RegValue::Radians(angle_at(&pinning.pins.pinned, row));
                    self.write(row, RegId::GoalPosition, goal)
                } else {
                    self.write(row, RegId::TorqueEnable, RegValue::U8(1))
                }
            }
            Phase::Recheck { cursor, .. } => self.read(cursor, RegId::PresentPosition),
            Phase::Repin { cursor, pinning } => {
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
            antenna_pull_in: pinning.antenna_pull_in,
            repinned: pinning.repinned,
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
            Phase::GainsProfiles { cursor, rest } => {
                confirm_write(result, &request, context)?;
                self.phase = if cursor + 1 < 3 * JointId::COUNT {
                    Phase::GainsProfiles {
                        cursor: cursor + 1,
                        rest,
                    }
                } else {
                    return self.enter_pinning(rest);
                };
                Ok(())
            }
            Phase::PinAndEnable { cursor, pinning } => {
                confirm_write(result, &request, context)?;
                self.phase = if cursor + 1 < 2 * JointId::COUNT {
                    Phase::PinAndEnable {
                        cursor: cursor + 1,
                        pinning,
                    }
                } else {
                    Phase::Recheck {
                        cursor: 0,
                        second: false,
                        pinning,
                    }
                };
                Ok(())
            }
            Phase::Recheck {
                cursor,
                second,
                pinning,
            } => self.absorb_recheck(cursor, second, pinning, context, result),
            Phase::Repin { cursor, pinning } => {
                confirm_write(result, &request, context)?;
                self.phase = if cursor + 1 < JointId::COUNT {
                    Phase::Repin {
                        cursor: cursor + 1,
                        pinning,
                    }
                } else {
                    Phase::Recheck {
                        cursor: 0,
                        second: true,
                        pinning,
                    }
                };
                Ok(())
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
        let angle = result.value(context)?.radians(context)?;
        if !angle.is_finite() {
            return Err(SeqError::UnplaceableAngle {
                context,
                joint,
                angle,
            });
        }
        self.records.present.set(joint, angle);
        if cursor + 1 < JointId::COUNT {
            self.phase = Phase::Pose { cursor: cursor + 1 };
            return Ok(());
        }
        // The resting pose, solved from the two candidates the platform is known
        // to come to rest at. Failure is not a solver problem to retry with a
        // perturbed seed: the angles are what nine servos reported, and angles
        // that place no pose say the model and the machine disagree.
        let rest = ArmRecord::solve(
            &self.geom,
            &self.fk,
            &self.records.present,
            &[stow_head_pose(), rest_head_pose()],
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
            Phase::GainsProfiles { cursor: 0, rest }
        };
        Ok(())
    }

    /// Compute the goals to pin at, and the pose they hold, before any of them is
    /// written.
    ///
    /// The order matters: a pin that would pull too far, an angle nobody can
    /// place, or a goal set that closes no linkage all stop the sequence here,
    /// with nothing written and no torque enabled.
    fn enter_pinning(&mut self, rest: ArmRecord) -> Result<(), SeqError> {
        let pins = pin_goals(&self.cfg, &self.env, &self.records.present)?;
        // The pull is at most the pin-in gate, which is millimetres of head
        // motion, so the resting pose is a close seed. The context is built for
        // the phase this is entering, not the one it is leaving: the pins are
        // what failed, and the gains and profiles that preceded them all landed.
        let armed = ArmRecord::solve(&self.geom, &self.fk, &pins.pinned, &[rest.head_pose_body])
            .map_err(|cause| SeqError::PinnedPoseUnsolvable {
                context: StepContext::reg(
                    SeqStep::PinAndEnable,
                    self.cfg.ids[1],
                    RegId::GoalPosition,
                ),
                cause,
            })?;
        self.check_armed_pose(&armed)?;
        self.phase = Phase::PinAndEnable {
            cursor: 0,
            pinning: Pinning {
                rest,
                pins,
                armed,
                pull_in: pins.pull_in,
                antenna_pull_in: pins.antenna_pull_in,
                repinned: false,
            },
        };
        Ok(())
    }

    /// Refuse a pinned pose the tick's own envelope would refuse.
    ///
    /// Every trajectory starts at the pose arming left the machine holding, so a
    /// start outside the envelope is a move that faults on its second tick, at a
    /// pose the machine is already standing in and with torque already on. The
    /// legs are inside their windows and the antennas inside their bound by the
    /// pin; this is what covers the rest of the verdict — body yaw, head
    /// attitude and head-relative yaw — none of which the pin has any fence for.
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
            armed.joints.antennas,
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

    fn absorb_recheck(
        &mut self,
        cursor: usize,
        second: bool,
        pinning: Pinning,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        let joint = JointId::ALL[cursor];
        let angle = result.value(context)?.radians(context)?;
        if !angle.is_finite() {
            return Err(SeqError::UnplaceableAngle {
                context,
                joint,
                angle,
            });
        }
        self.records.recheck.set(joint, angle);
        if cursor + 1 < JointId::COUNT {
            self.phase = Phase::Recheck {
                cursor: cursor + 1,
                second,
                pinning,
            };
            return Ok(());
        }
        // Torque coming on in position mode can reset a servo's reported
        // position, so the read after enabling is compared against the pin
        // rather than trusted.
        //
        // TODO(pin-settle-dwell): nothing waits between the last enable and this
        // read, so a joint the pin is still pulling reads short of its goal
        // through both sweeps and arming gives up on a machine that is merely
        // mid-travel. An antenna pulled in from a limp rest is in that same
        // position, and its pull is bounded by nothing.
        let drifted =
            self.records
                .recheck
                .joints()
                .into_iter()
                .enumerate()
                .find(|(row, (_, present))| {
                    outside_limit(
                        (present - angle_at(&pinning.pins.pinned, *row)).abs(),
                        self.cfg.repin_tolerance,
                    )
                });
        let Some((row, (joint, present))) = drifted else {
            self.phase = Phase::Complete(pinning);
            return Ok(());
        };
        if second {
            return Err(SeqError::PinUnstable {
                context: self.context(row, RegId::PresentPosition),
                joint,
                pinned: angle_at(&pinning.pins.pinned, row),
                present,
            });
        }
        let pins = pin_goals(&self.cfg, &self.env, &self.records.recheck)?;
        let mut pinning = Pinning {
            repinned: true,
            ..pinning
        };
        // The re-pin arrives with torque already on and held, so a refusal here
        // leaves the machine holding what it holds; recovery is the operator's.
        if pins.pinned != pinning.pins.pinned {
            pinning.armed = ArmRecord::solve(
                &self.geom,
                &self.fk,
                &pins.pinned,
                &[pinning.armed.head_pose_body],
            )
            .map_err(|cause| SeqError::PinnedPoseUnsolvable {
                context: self.context(1, RegId::GoalPosition),
                cause,
            })?;
            self.check_armed_pose(&pinning.armed)?;
        }
        pinning.pins = pins;
        self.phase = Phase::Repin { cursor: 0, pinning };
        Ok(())
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

    /// The envelope the pin reads its antenna bound from, and the one the
    /// windows in [`config`] are drawn from.
    fn default_env() -> EnvelopeConfig {
        EnvelopeConfig::default()
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
        let outcome = pin_goals(&cfg, &default_env(), &present).expect("nothing to pull");
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
        let outcome =
            pin_goals(&cfg, &default_env(), &present).expect("the pulls are inside the gate");

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
        assert_eq!(outcome.antenna_pull_in, [0.0; 2]);
    }

    /// The model angle a servo count denotes: a whole turn spread over 4096
    /// counts, zero at the middle of the range. Spelled out here because nothing
    /// in this crate knows what a count is, and the antenna readings below are
    /// what a real machine reported.
    fn rad_from_counts(counts: i32) -> f64 {
        core::f64::consts::TAU * f64::from(counts) / 4096.0 - PI
    }

    /// The antennas as the platform parks them: driven to the command bound,
    /// torque cut, settled a little past it. Both are pinned back to the bound
    /// and both pulls are recorded, with nothing refused — which is what makes a
    /// machine found in its own parked state armable at all.
    #[test]
    fn the_parked_antennas_are_pinned_to_their_bound() {
        let cfg = config();
        let env = default_env();
        let rest = joints_at(&rest_head_pose());

        let mut present = rest;
        // The two readings this unit reported at rest: 38 and 4051 counts.
        present.antennas = [rad_from_counts(38), rad_from_counts(4051)];
        assert!(present.antennas[0] < -env.antenna_limit);
        assert!(present.antennas[1] > env.antenna_limit);

        let outcome = pin_goals(&cfg, &env, &present).expect("no gate stands in front of this");
        assert_eq!(
            outcome.pinned.antennas,
            [-env.antenna_limit, env.antenna_limit]
        );
        let degrees: Vec<f64> = outcome
            .antenna_pull_in
            .iter()
            .map(|pull| (pull.to_degrees() * 1e3).round() / 1e3)
            .collect();
        assert_eq!(degrees, vec![1.908, 1.293]);

        // The legs are pinned exactly as they are without any of this: the two
        // fences are independent, and the leg gate never sees an antenna.
        let plain = pin_goals(&cfg, &env, &rest).expect("the pulls are inside the gate");
        assert_eq!(outcome.pinned.legs, plain.pinned.legs);
        assert_eq!(outcome.pull_in, plain.pull_in);
        assert_eq!(outcome.worst_pull_in(), plain.worst_pull_in());
    }

    /// An antenna is a free rotor and its reading runs past the half turn with
    /// torque off, so the pin is by sign rather than by wrapping — and however
    /// far out the reading is, there is no gate to refuse it.
    #[test]
    fn an_antenna_past_the_half_turn_pins_by_sign() {
        let cfg = config();
        let env = default_env();
        let mut present = joints_at(&rest_head_pose());
        present.antennas = [3.6, -4.2];

        let outcome = pin_goals(&cfg, &env, &present).expect("no gate stands in front of this");
        assert_eq!(
            outcome.pinned.antennas,
            [env.antenna_limit, -env.antenna_limit]
        );
        let degrees: Vec<f64> = outcome
            .antenna_pull_in
            .iter()
            .map(|pull| (pull.to_degrees() * 1e3).round() / 1e3)
            .collect();
        assert_eq!(degrees, vec![31.513, 65.890]);
    }

    /// An antenna inside the bound is pinned where it is, bit for bit — and the
    /// bound itself is inside it, which is where the stow pose leaves them.
    #[test]
    fn an_antenna_inside_its_bound_is_pinned_where_it_is() {
        let cfg = config();
        let env = default_env();
        let mut present = joints_at(&rest_head_pose());

        for antennas in [[0.2, -0.15], [-env.antenna_limit, env.antenna_limit]] {
            present.antennas = antennas;
            let outcome = pin_goals(&cfg, &env, &present).expect("nothing to pull");
            assert_eq!(outcome.pinned.antennas, antennas);
            assert_eq!(outcome.antenna_pull_in, [0.0; 2]);
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
        let error = pin_goals(&cfg, &default_env(), &present)
            .expect_err("the sixth leg is ten degrees out");
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
                let error = pin_goals(&cfg, &default_env(), &present)
                    .expect_err("an unplaceable angle refuses");
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
        let outcome =
            pin_goals(&cfg, &default_env(), &present).expect("the pulls are inside the gate");

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
        assert!((DEFAULT_REPIN_TOLERANCE.to_degrees() - 0.5).abs() < 1e-12);
        // The pull-in gate is above the worst overrun the recorded rest has, or
        // arming would refuse the case it was sized for.
        let cfg = config();
        let present = joints_at(&rest_head_pose());
        let outcome = pin_goals(&cfg, &default_env(), &present).expect("inside the gate");
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
        let error =
            pin_goals(&cfg, &default_env(), &present).expect_err("thirty degrees is past the gate");
        assert!(matches!(error, SeqError::PullInTooLarge { .. }));

        let wide = ArmConfig {
            leg_windows: [(-PI, PI); 6],
            ..cfg
        };
        let outcome = pin_goals(&wide, &default_env(), &present).expect("nothing to pull");
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
        let outcome = pin_goals(&cfg, &default_env(), &present).expect("nothing to pull");
        assert_eq!(outcome.pull_in, [0.0; 6]);
        assert_eq!(outcome.pinned.legs, present.legs);
    }

    /// How far a joint appears to have moved between its pin and the read after
    /// torque came on, in the machine below. Twice the re-pin tolerance.
    const DRIFT: f64 = 1.0 * PI / 180.0;

    /// Nine servos answering out of their own state, with a knob per thing a test
    /// wants to vary. The transaction log is what the phase-order assertions
    /// read: it is the whole content of "arming did these things in this order".
    #[derive(Clone, Debug)]
    struct Machine {
        models: [u16; JointId::COUNT],
        silent: [bool; JointId::COUNT],
        /// Supply readings, one per poll; the last one repeats for ever.
        sweeps: Vec<f64>,
        /// Per servo, how far below the sweep's reading that servo reports.
        sag: [f64; JointId::COUNT],
        health: [u8; JointId::COUNT],
        torque: [u8; JointId::COUNT],
        /// What a position read returns before torque is on.
        present: JointVector,
        /// What a provisioned register holds.
        provision: Vec<(RegId, RegValue)>,
        /// How many post-enable sweeps report the right antenna away from its
        /// goal.
        drift_sweeps: u32,
        /// How many post-enable sweeps report every pulled leg still short of
        /// its goal, which is what a pin that is still travelling looks like.
        travelling_sweeps: u32,
        /// One write to answer with something other than success. Separate from
        /// the read knob because several registers are both read and written in
        /// one sequence, and which of the two fails is the whole question.
        fail_write: Option<(u8, RegId, BusResult)>,
        /// One read to answer with something other than success.
        fail_read: Option<(u8, RegId, BusResult)>,
        goals: JointVector,
        enabled: [bool; JointId::COUNT],
        reads_after_enable: usize,
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
            provision: vec![
                (RegId::OperatingMode, RegValue::U8(3)),
                (RegId::HomingOffset, RegValue::I32(1024)),
                (RegId::Shutdown, RegValue::U8(0x34)),
            ],
            drift_sweeps: 0,
            travelling_sweeps: 0,
            fail_write: None,
            fail_read: None,
            goals: JointVector::default(),
            enabled: [false; JointId::COUNT],
            reads_after_enable: 0,
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
                other => self
                    .provision
                    .iter()
                    .find(|(reg, _)| *reg == other)
                    .map_or(RegValue::U8(0), |(_, value)| *value),
            }
        }

        /// Where the platform rests until torque is on, and where it was
        /// commanded once it is — with the first `drift_sweeps` post-enable
        /// sweeps reporting the right antenna short of its goal, which is what a
        /// position reset at torque-on looks like from the host. The antenna
        /// rather than a leg because a leg reporting a value of its own would be
        /// six legs describing no pose at all.
        fn position(&mut self, row: usize) -> f64 {
            if !self.enabled[row] {
                return angle_at(&self.present, row);
            }
            let sweep = u32::try_from(self.reads_after_enable / JointId::COUNT).unwrap_or(u32::MAX);
            self.reads_after_enable += 1;
            let goal = angle_at(&self.goals, row);
            if sweep < self.drift_sweeps && JointId::ALL[row] == JointId::AntennaRight {
                return goal - DRIFT;
            }
            let rest = angle_at(&self.present, row);
            if sweep < self.travelling_sweeps
                && matches!(JointId::ALL[row], JointId::Leg(_))
                && (goal - rest).abs() > DRIFT
            {
                // Short of the goal, on the side it was pulled from.
                return goal - DRIFT.copysign(goal - rest);
            }
            goal
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
                        (RegId::GoalPosition, RegValue::Radians(angle)) => {
                            self.goals.set(JointId::ALL[row], angle);
                        }
                        (RegId::TorqueEnable, RegValue::U8(1)) => {
                            self.enabled[row] = true;
                            self.reads_after_enable = 0;
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

    /// The whole sequence against a machine that arms: the phase order, the two
    /// order properties that are the safety content of it, and the records it
    /// hands back.
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

        // Every servo's goal is written before its own torque is enabled.
        for id in SERVO_IDS {
            let at = |reg| {
                machine.log.iter().position(|(_, request)| {
                    matches!(request, BusRequest::WriteRegVerified { id: to, reg: written, .. }
                        if *to == id && *written == reg)
                })
            };
            let goal = at(RegId::GoalPosition).expect("a goal per servo");
            let enable = at(RegId::TorqueEnable).expect("an enable per servo");
            assert!(goal < enable, "servo {id} was enabled before it was pinned");
        }

        let count = |step| machine.log.iter().filter(|(s, _)| *s == step).count();
        assert_eq!(count(SeqStep::Presence), JointId::COUNT);
        assert_eq!(count(SeqStep::Identity), JointId::COUNT);
        assert_eq!(count(SeqStep::Provision), 3 * JointId::COUNT);
        assert_eq!(count(SeqStep::VoltageGate), JointId::COUNT);
        assert_eq!(count(SeqStep::Health), JointId::COUNT);
        assert_eq!(count(SeqStep::PoseAndDatum), JointId::COUNT);
        assert_eq!(count(SeqStep::StateDiscovery), JointId::COUNT);
        // Gains, then acceleration and velocity, per servo.
        assert_eq!(count(SeqStep::GainsProfiles), 3 * JointId::COUNT);
        // A goal and an enable per servo, then the nine-servo re-check.
        assert_eq!(count(SeqStep::PinAndEnable), 3 * JointId::COUNT);

        let pins = pin_goals(&cfg, &default_env(), &machine.present).expect("inside the gate");
        assert_eq!(summary.rest.joints, machine.present);
        assert_eq!(summary.armed.joints, pins.pinned);
        assert_eq!(
            machine.goals, pins.pinned,
            "the goals in the servos are the pins"
        );
        assert_eq!(summary.pull_in, pins.pull_in);
        assert!(summary.worst_pull_in() < cfg.max_pin_pull_in);
        assert!(!summary.repinned);
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

    /// Enabling torque can reset a servo's reported position. That is re-pinned
    /// once — goals rewritten, torque left alone — and the arm-time pull stays
    /// the reported one.
    #[test]
    fn a_joint_that_moved_under_torque_is_repinned_exactly_once() {
        let cfg = provisioned_config();
        let mut machine = Machine {
            drift_sweeps: 1,
            ..bus()
        };
        let summary = drive(&cfg, &mut machine).expect("one re-pin settles it");
        assert!(summary.repinned);
        assert_eq!(
            writes(&machine.log, RegId::GoalPosition),
            2 * JointId::COUNT
        );
        assert_eq!(
            writes(&machine.log, RegId::TorqueEnable),
            JointId::COUNT,
            "a re-pin rewrites goals, never torque"
        );
        // The resting read, and two re-check sweeps: exactly one re-pin.
        let positions = machine
            .log
            .iter()
            .filter(|(_, request)| {
                matches!(
                    request,
                    BusRequest::ReadReg {
                        reg: RegId::PresentPosition,
                        ..
                    }
                )
            })
            .count();
        assert_eq!(positions, 3 * JointId::COUNT);
        // The pull recorded is the pull off the rest, not the re-pin's own.
        let pins = pin_goals(&cfg, &default_env(), &machine.present).expect("inside the gate");
        assert_eq!(summary.pull_in, pins.pull_in);
        // The armed record describes the goals actually left in the servos.
        assert_eq!(summary.armed.joints, machine.goals);
        assert!(
            (summary.armed.joints.antennas[0] - (pins.pinned.antennas[0] - DRIFT)).abs() < 1e-12
        );
    }

    /// A joint that will not settle stops arming rather than being pinned for
    /// ever. Torque is already on and stays on; the head is held.
    #[test]
    fn a_joint_that_will_not_settle_stops_arming() {
        let mut machine = Machine {
            drift_sweeps: 99,
            ..bus()
        };
        let error = drive(&provisioned_config(), &mut machine).expect_err("it never settles");
        let SeqError::PinUnstable {
            context,
            joint,
            pinned,
            present,
        } = error
        else {
            panic!("expected a pin-unstable refusal, got {error}");
        };
        assert_eq!(joint, JointId::AntennaRight);
        assert_eq!(context.step, SeqStep::PinAndEnable);
        assert_eq!(context.id, 17);
        assert!((pinned - present - DRIFT).abs() < 1e-12);
        assert_eq!(
            writes(&machine.log, RegId::GoalPosition),
            2 * JointId::COUNT
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
    /// past the command bound, which is where the platform's own stow leaves
    /// them. Arming pins them in, writes those goals, and completes — the state
    /// it would refuse if the pin had no fence for an antenna.
    #[test]
    fn a_machine_parked_past_the_antenna_bound_arms() {
        let cfg = provisioned_config();
        let env = EnvelopeConfig::default();

        let mut machine = bus();
        machine.present.antennas = [rad_from_counts(38), rad_from_counts(4051)];
        let summary = drive(&cfg, &mut machine).expect("a parked antenna is not a refusal");

        // Recorded as found, left at the bound, and the pull reported.
        assert_eq!(summary.rest.joints.antennas, machine.present.antennas);
        assert_eq!(
            summary.armed.joints.antennas,
            [-env.antenna_limit, env.antenna_limit]
        );
        let degrees: Vec<f64> = summary
            .antenna_pull_in
            .iter()
            .map(|pull| (pull.to_degrees() * 1e3).round() / 1e3)
            .collect();
        assert_eq!(degrees, vec![1.908, 1.293]);
        assert!(!summary.repinned);

        // The goals that reached the machine are the pinned values, not the
        // measured ones, and every servo took one.
        assert_eq!(
            machine.goals.antennas,
            [-env.antenna_limit, env.antenna_limit]
        );
        assert_eq!(writes(&machine.log, RegId::GoalPosition), JointId::COUNT);
        assert_eq!(writes(&machine.log, RegId::TorqueEnable), JointId::COUNT);
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
            machine.enabled,
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
        assert_eq!(machine.enabled, [false; JointId::COUNT]);
    }

    /// Legs the pin is still pulling when the post-enable read happens. One
    /// re-pin absorbs a pull that has finished by the second sweep; a pull still
    /// running at both sweeps is refused as an unsettled joint.
    ///
    /// The second half is the behaviour of a sequence with no wait between the
    /// last torque enable and the first position read, against a fixture that can
    /// model either arrival. Which of the two the hardware does — how far this
    /// unit really rests outside its windows, and how long the pull takes — is
    /// `TODO(pin-settle-dwell)`.
    #[test]
    fn legs_still_travelling_at_the_post_enable_read() {
        let cfg = provisioned_config();

        let mut machine = Machine {
            travelling_sweeps: 1,
            ..bus()
        };
        let summary = drive(&cfg, &mut machine).expect("they arrive by the second sweep");
        assert!(summary.repinned);
        // The pins were rewritten, and to the same angles: a leg short of a bound
        // it is being pulled to is still outside the window, so it pins there
        // again.
        let pins = pin_goals(&cfg, &default_env(), &machine.present).expect("inside the gate");
        assert_eq!(machine.goals, pins.pinned);
        assert_eq!(
            writes(&machine.log, RegId::GoalPosition),
            2 * JointId::COUNT
        );
        assert_eq!(
            writes(&machine.log, RegId::TorqueEnable),
            JointId::COUNT,
            "a re-pin rewrites goals, never torque"
        );
        assert_eq!(summary.armed.joints, pins.pinned);

        let mut machine = Machine {
            travelling_sweeps: 99,
            ..bus()
        };
        let error = drive(&cfg, &mut machine).expect_err("they are still moving at both reads");
        let SeqError::PinUnstable {
            context,
            joint,
            pinned,
            present,
        } = error
        else {
            panic!("expected a pin-unstable refusal, got {error}");
        };
        // The first leg the pin pulled, in bus order.
        assert_eq!(joint, JointId::Leg(0));
        assert_eq!(context.id, 11);
        assert!((pinned - present).abs() > cfg.repin_tolerance);
        // Torque went on and stays on: the head is held by what it was given.
        assert_eq!(machine.enabled, [true; JointId::COUNT]);
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
