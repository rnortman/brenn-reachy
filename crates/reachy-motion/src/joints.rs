//! The two command vectors, the helpers over the joint vocabulary, and a
//! servo's health byte.
//!
//! Two views of the same machine, and the tick's whole job is turning one into
//! the other:
//!
//! - [`JointTargets`] is what a caller commands — a head pose, a body yaw and
//!   two antenna angles. It is the set the envelope check takes and the set the
//!   trajectories interpolate, because a head pose is the thing that has a
//!   meaningful straight line through it; six crank angles do not.
//! - [`JointVector`] is what the servos speak — nine angles, in the order the
//!   bus reports them. It is what comes back from a position read and what goes
//!   out as goals.
//!
//! The map from targets to joints runs through the envelope check, in that
//! direction only; the reverse needs the iterative solver and belongs to the
//! tick's ingest path, not here.
//!
//! Angles are radians about the model datum: what the servo's own registers
//! mean once the configured datum has been applied. Counts, registers and the
//! datum itself live below this crate.
//!
//! [`ServoHealth`] is here for the same reason the joint helpers are: it is
//! shared between tick and arming rather than owned by either.
//!
//! One servo is the vocabulary's [`JointRef`] and a set of them is its
//! `JointFlags`, and nothing else. What this module adds is what the generator
//! does not emit: [`row`] and [`joint_ref`], which carry the one offset between
//! the vocabulary's numbering and the bus rows the arrays here are indexed by,
//! [`group_of`] and [`Name`], the [`flags`] module's membership
//! operations — the only code that knows which bit is which bus row — and
//! [`angle_of`]/[`set_angle`] and their row-array pair, which address the
//! schema's own nine-angle vector by joint rather than by field order.

use brenn_reachy__motion__joints_clk_rs::JointFlags;
/// Which servo a report, a fault or a slot names — the vocabulary's own enum,
/// re-exported so a consumer takes the name and the helpers below from one
/// path.
pub use brenn_reachy__motion__joints_clk_rs::JointRef;
use nalgebra::Isometry3;
use reachy_kin::neutral_head_pose;

/// One angle per joint, in bus order: body yaw, legs 1..=6, right antenna, left
/// antenna. Radians about the model datum.
///
/// Used for both measurement (present positions) and command (goals), because
/// they are the same nine numbers and a type that distinguished them would have
/// to be converted at every comparison the tick makes between them.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct JointVector {
    /// Body yaw, radians.
    pub body_yaw: f64,
    /// The six crank angles, in servo order 1..=6, radians.
    pub legs: [f64; 6],
    /// Antenna angles, right then left, radians.
    pub antennas: [f64; 2],
}

impl JointVector {
    /// The angle at `id`, or `None` if `id` names no servo.
    #[must_use]
    pub fn get(&self, id: JointRef) -> Option<f64> {
        match id {
            JointRef::None => None,
            JointRef::BodyYaw => Some(self.body_yaw),
            JointRef::AntennaRight => Some(self.antennas[0]),
            JointRef::AntennaLeft => Some(self.antennas[1]),
            crank => self.legs.get(usize::from(leg_index(crank)?)).copied(),
        }
    }

    /// Set the angle at `id`, reporting `false` — and changing nothing — if
    /// `id` names no servo.
    ///
    /// The mirror of [`Self::get`], for filling a vector one servo's answer at a
    /// time as the reads come back.
    pub fn set(&mut self, id: JointRef, angle: f64) -> bool {
        match id {
            JointRef::None => return false,
            JointRef::BodyYaw => self.body_yaw = angle,
            JointRef::AntennaRight => self.antennas[0] = angle,
            JointRef::AntennaLeft => self.antennas[1] = angle,
            JointRef::Leg0 => self.legs[0] = angle,
            JointRef::Leg1 => self.legs[1] = angle,
            JointRef::Leg2 => self.legs[2] = angle,
            JointRef::Leg3 => self.legs[3] = angle,
            JointRef::Leg4 => self.legs[4] = angle,
            JointRef::Leg5 => self.legs[5] = angle,
        }
        true
    }

    /// Every joint paired with its angle, in bus order.
    ///
    /// A single pass over all nine joints ensures no joint is checked by one
    /// guard and missed by another. Fixed size, so nothing allocates.
    #[must_use]
    pub fn joints(&self) -> [(JointRef, f64); ROW_COUNT] {
        [
            (JointRef::BodyYaw, self.body_yaw),
            (JointRef::Leg0, self.legs[0]),
            (JointRef::Leg1, self.legs[1]),
            (JointRef::Leg2, self.legs[2]),
            (JointRef::Leg3, self.legs[3]),
            (JointRef::Leg4, self.legs[4]),
            (JointRef::Leg5, self.legs[5]),
            (JointRef::AntennaRight, self.antennas[0]),
            (JointRef::AntennaLeft, self.antennas[1]),
        ]
    }

    /// The first joint in bus order whose angle is not a number, if any.
    ///
    /// Named rather than counted, so a fault raised from this can name the
    /// joint the bad number arrived on.
    #[must_use]
    pub fn first_non_finite(&self) -> Option<JointRef> {
        self.joints()
            .into_iter()
            .find(|(_, angle)| !angle.is_finite())
            .map(|(id, _)| id)
    }
}

/// The Cartesian command set: what a caller asks for and what a trajectory
/// interpolates.
///
/// The head pose is expressed **in the body frame** — relative to the yawing
/// body, not the fixed foot — so `body_yaw` and `head_pose_body` are
/// independent commands rather than two descriptions of one motion.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointTargets {
    /// Head pose relative to the body at the commanded yaw.
    pub head_pose_body: Isometry3<f64>,
    /// Body yaw, radians.
    pub body_yaw: f64,
    /// Antenna angles, right then left, radians.
    pub antennas: [f64; 2],
}

impl Default for JointTargets {
    /// The neutral head pose, zero yaw, antennas at zero.
    ///
    /// Deliberately a configuration the machine can actually hold: these are
    /// out-parameter buffers, and a default of all-zeros would put the head
    /// origin at the floor — a pose no envelope admits and no reader would
    /// recognise as uninitialised.
    fn default() -> Self {
        Self {
            head_pose_body: neutral_head_pose(),
            body_yaw: 0.0,
            antennas: [0.0, 0.0],
        }
    }
}

impl JointTargets {
    /// Whether every commanded number is finite, the pose included.
    ///
    /// A non-finite pose cannot be interpolated toward or checked against a
    /// bound, so the trajectory constructor refuses one rather than carrying it
    /// to the envelope check as a violation on every tick of a doomed move.
    #[must_use]
    pub fn is_finite(&self) -> bool {
        self.head_pose_body
            .translation
            .vector
            .iter()
            .all(|c| c.is_finite())
            && self
                .head_pose_body
                .rotation
                .coords
                .iter()
                .all(|c| c.is_finite())
            && self.body_yaw.is_finite()
            && self.antennas.iter().all(|a| a.is_finite())
    }
}

/// Per-tick step bounds, radians, one per group.
///
/// Exceeding one of these is a fault, never a clamp: an oversized step is a
/// goal the servo applies as an immediate jump, and the interpolator or the
/// seed being wrong is the thing worth reporting, not the jump being trimmed.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointStep {
    /// Bound on any one crank's change per tick.
    pub legs: f64,
    /// Bound on the body yaw's change per tick.
    pub body_yaw: f64,
    /// Bound on either antenna's change per tick.
    pub antennas: f64,
}

impl JointStep {
    /// The bound that applies to `id`, and the tightest of the three for
    /// [`JointRef::None`], which names no group and must not read as the
    /// loosest one.
    #[must_use]
    pub fn for_joint(&self, id: JointRef) -> f64 {
        match group_of(id) {
            Some(JointGroup::BodyYaw) => self.body_yaw,
            Some(JointGroup::Legs) => self.legs,
            Some(JointGroup::Antennas) => self.antennas,
            None => self.legs.min(self.body_yaw).min(self.antennas),
        }
    }
}

/// One servo's hardware-error byte, paired with the bus ID it was read from.
///
/// A fault or a refusal names the offending servo by its bus ID, so the ID
/// travels with the bits rather than being inferred from a position in an array.
/// Whatever owns the port fills these in; this crate never learns what an ID
/// means.
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

/// Whether `candidate` is a worse deviation than `incumbent`.
///
/// An ordering, deliberately not the bound test that sits beside it wherever it
/// is used: a bound test treats an incomparable value as a violation, which is
/// right for "is this joint past the threshold" and wrong for "which joint is
/// furthest out" — an unplaceable deviation would both beat the incumbent and be
/// beaten by whatever came next, so the report whose whole job is naming a joint
/// would name the one *after* the bad one and print its zero beside it.
///
/// A deviation nobody can place is the worst thing this comparison can see, so
/// it wins outright and keeps winning. Ties keep the joint found first.
fn worse_error(candidate: f64, incumbent: f64) -> bool {
    if candidate.is_nan() {
        return !incumbent.is_nan();
    }
    candidate > incumbent
}

/// Which row of `deviations` — one per joint, in bus order — is furthest out.
///
/// The sweep every report that names a joint runs, written once so the seed is
/// decided once: the incumbent starts at row 0's own value, which is a real
/// deviation, rather than at a floor no measurement can be worse than. A sweep
/// seeded below every value can leave a joint named because nothing displaced
/// the seed; seeded at a floor of zero it can name a joint with no deviation at
/// all. Ties keep the earlier row, so the report is the same on every run.
pub(crate) fn worst_row(deviations: &[f64; ROW_COUNT]) -> usize {
    let mut worst = 0;
    for (row, deviation) in deviations.iter().enumerate() {
        if worse_error(*deviation, deviations[worst]) {
            worst = row;
        }
    }
    worst
}

/// The joint furthest out and how far, from deviations in bus order.
pub(crate) fn worst_joint(deviations: &[f64; ROW_COUNT]) -> (JointRef, f64) {
    let row = worst_row(deviations);
    (ROWS[row], deviations[row])
}

/// How many servos the bus carries.
pub const ROW_COUNT: usize = 9;

/// How many cranks carry the head.
///
/// The legs alone, in leg order: how the kinematics reports a per-leg number
/// and how a record of an arming holds one.
pub const LEG_COUNT: usize = 6;

/// Every servo, in bus order.
///
/// The vocabulary's [`JointRef`] also names `None` — a report about the machine
/// as a whole — which is not a servo and is not here.
pub const ROWS: [JointRef; ROW_COUNT] = [
    JointRef::BodyYaw,
    JointRef::Leg0,
    JointRef::Leg1,
    JointRef::Leg2,
    JointRef::Leg3,
    JointRef::Leg4,
    JointRef::Leg5,
    JointRef::AntennaRight,
    JointRef::AntennaLeft,
];

/// Position in bus order, or `None` for [`JointRef::None`], which names no row.
///
/// The vocabulary numbers the nine servos from one, because zero is the value a
/// slot nothing wrote carries; the arrays in this crate number them from zero.
/// This function and its inverse [`joint_ref`] are the only places that offset
/// is applied; a crossing that needs it goes through them.
#[must_use]
pub fn row(joint: JointRef) -> Option<usize> {
    match joint {
        JointRef::None => None,
        other => Some(other as usize - 1),
    }
}

/// The servo at `row` in bus order, or `None` past the ninth.
#[must_use]
pub fn joint_ref(row: usize) -> Option<JointRef> {
    ROWS.get(row).copied()
}

/// The crank at `leg`, 0-based in servo order, or [`JointRef::None`] past the
/// sixth.
///
/// 0-based because that is the leg index the kinematics reports an unreachable
/// pose against; the rendering is 1-based, matching the servo numbering on the
/// bus and the way the envelope names a leg.
#[must_use]
pub fn leg_ref(index: u8) -> JointRef {
    match index {
        0..=5 => joint_ref(1 + usize::from(index)).unwrap_or(JointRef::None),
        _ => JointRef::None,
    }
}

/// The 0-based crank index of `joint`, or `None` if it is not a crank.
#[must_use]
pub fn leg_index(joint: JointRef) -> Option<u8> {
    match joint {
        JointRef::Leg0 => Some(0),
        JointRef::Leg1 => Some(1),
        JointRef::Leg2 => Some(2),
        JointRef::Leg3 => Some(3),
        JointRef::Leg4 => Some(4),
        JointRef::Leg5 => Some(5),
        _ => None,
    }
}

/// Which group `joint` belongs to, or `None` for [`JointRef::None`].
#[must_use]
pub fn group_of(joint: JointRef) -> Option<JointGroup> {
    match joint {
        JointRef::None => None,
        JointRef::BodyYaw => Some(JointGroup::BodyYaw),
        JointRef::AntennaRight | JointRef::AntennaLeft => Some(JointGroup::Antennas),
        JointRef::Leg0
        | JointRef::Leg1
        | JointRef::Leg2
        | JointRef::Leg3
        | JointRef::Leg4
        | JointRef::Leg5 => Some(JointGroup::Legs),
    }
}

vocab_name! {
    /// A servo rendered for a message an operator reads.
    ///
    /// Legs render 1-based, as the servos and the envelope's own messages number
    /// them.
    pub struct Name(JointRef) {
        JointRef::None => "no joint",
        JointRef::BodyYaw => "body yaw",
        JointRef::AntennaRight => "right antenna",
        JointRef::AntennaLeft => "left antenna",
        JointRef::Leg0 => "leg 1",
        JointRef::Leg1 => "leg 2",
        JointRef::Leg2 => "leg 3",
        JointRef::Leg3 => "leg 4",
        JointRef::Leg4 => "leg 5",
        JointRef::Leg5 => "leg 6",
    }
}

/// Pair each of a schema's nine servo-named fields with the servo whose value
/// lives in it.
///
/// Every schema that holds one value per servo spells the nine out as named
/// fields rather than as an array — a row misplaced in an array is a silent swap
/// of two servos, and a row misplaced across named fields does not compile — and
/// every one of them uses the same nine field names. So the pairing is written
/// here once, and a schema of that shape asks for it by naming its type: a joint
/// and the field its value lives in cannot be paired differently in two places.
///
/// The nine patterns are counted against [`ROW_COUNT`], so a bus row the machine
/// grows is a build failure here rather than a servo whose value goes nowhere.
/// A joint no field answers — a leg index past the sixth, which [`JointRef`] can
/// spell and no machine carries, and the `none` that names no servo — is the one
/// answered with `None`.
macro_rules! rows_by_joint {
    ($vis:vis $type:ty, $row:ty, $of:ident, $of_mut:ident) => {
        const _: () = assert!(9 == ROW_COUNT);

        /// The field `joint`'s value lives in, or `None` for a joint no field
        /// answers.
        #[must_use]
        $vis fn $of(rows: &$type, joint: JointRef) -> Option<&$row> {
            match joint {
                JointRef::BodyYaw => Some(&rows.body_yaw),
                JointRef::Leg0 => Some(&rows.leg_0),
                JointRef::Leg1 => Some(&rows.leg_1),
                JointRef::Leg2 => Some(&rows.leg_2),
                JointRef::Leg3 => Some(&rows.leg_3),
                JointRef::Leg4 => Some(&rows.leg_4),
                JointRef::Leg5 => Some(&rows.leg_5),
                JointRef::AntennaRight => Some(&rows.antenna_right),
                JointRef::AntennaLeft => Some(&rows.antenna_left),
                JointRef::None => None,
            }
        }

        /// The same field, to write.
        $vis fn $of_mut(rows: &mut $type, joint: JointRef) -> Option<&mut $row> {
            match joint {
                JointRef::BodyYaw => Some(&mut rows.body_yaw),
                JointRef::Leg0 => Some(&mut rows.leg_0),
                JointRef::Leg1 => Some(&mut rows.leg_1),
                JointRef::Leg2 => Some(&mut rows.leg_2),
                JointRef::Leg3 => Some(&mut rows.leg_3),
                JointRef::Leg4 => Some(&mut rows.leg_4),
                JointRef::Leg5 => Some(&mut rows.leg_5),
                JointRef::AntennaRight => Some(&mut rows.antenna_right),
                JointRef::AntennaLeft => Some(&mut rows.antenna_left),
                JointRef::None => None,
            }
        }
    };
}

pub(crate) use rows_by_joint;

/// A nine-angle vector as the schema declares it: one field per bus row.
///
/// The vocabulary's own vector, re-exported beside the helpers that index it, so
/// a consumer writing one into a slot takes both from one path.
pub use brenn_reachy__motion__joints_clk_rs::Joints;

rows_by_joint!(Joints, f64, angle_ref, angle_ref_mut);

/// The angle at `id`, or `None` if `id` names no servo.
///
/// The schema's vector by field name, addressed by joint the way
/// [`JointVector::get`] addresses the library's own: a servo's angle can only be
/// read off the field that servo names.
#[must_use]
pub fn angle_of(joints: &Joints, id: JointRef) -> Option<f64> {
    angle_ref(joints, id).copied()
}

/// Set the angle at `id`, reporting `false` — and changing nothing — if `id`
/// names no servo. The mirror of [`angle_of`].
pub fn set_angle(joints: &mut Joints, id: JointRef, angle: f64) -> bool {
    if let Some(field) = angle_ref_mut(joints, id) {
        *field = angle;
        return true;
    }
    false
}

/// The library's own vector from the nine numbers the schema's vector holds.
///
/// Addressed by joint in both directions, so neither this nor [`write_vector`]
/// can put one servo's angle on another servo's name.
#[must_use]
pub fn vector_of(joints: &Joints) -> JointVector {
    let mut angles = JointVector::default();
    for joint in ROWS {
        if let Some(angle) = angle_of(joints, joint) {
            angles.set(joint, angle);
        }
    }
    angles
}

/// The same nine angles, written into the fields the schema holds them in.
pub fn write_vector(out: &mut Joints, angles: &JointVector) {
    for joint in ROWS {
        if let Some(angle) = angles.get(joint) {
            set_angle(out, joint, angle);
        }
    }
}

/// Write nine angles in bus-row order into the fields the schema holds them in.
///
/// The rows and the servos are matched through [`joint_ref`] rather than by
/// writing the field order out a second time, so neither this function nor its
/// inverse can put one servo's angle on another servo's name.
pub fn write_rows(out: &mut Joints, rows: &[f64; ROW_COUNT]) {
    for (row, angle) in rows.iter().enumerate() {
        if let Some(id) = joint_ref(row) {
            set_angle(out, id, *angle);
        }
    }
}

/// The inverse of [`write_rows`]: the nine angles those fields hold, in bus-row
/// order.
#[must_use]
pub fn rows_of(joints: &Joints) -> [f64; ROW_COUNT] {
    let mut rows = [0.0; ROW_COUNT];
    for (row, slot) in rows.iter_mut().enumerate() {
        if let Some(angle) = joint_ref(row).and_then(|id| angle_of(joints, id)) {
            *slot = angle;
        }
    }
    rows
}

/// The joints that move as one.
///
/// The six cranks carry the head together and are bounded together; the two
/// antennas are their own pair; body yaw is one servo. Anything that treats the
/// nine joints as three sets — a per-group step bound, a grouped goal write —
/// asks for the grouping here rather than restating which bus rows are which,
/// so the bus layout has one owner.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointGroup {
    /// The body yaw servo alone.
    BodyYaw,
    /// The six cranks.
    Legs,
    /// The two antennas.
    Antennas,
}

impl JointGroup {
    /// Every group, in bus order of its first joint.
    pub const ALL: [JointGroup; 3] = [Self::BodyYaw, Self::Legs, Self::Antennas];

    /// The joints this group covers, in bus order.
    #[must_use]
    pub fn joints(self) -> JointFlags {
        let mut set = JointFlags::NONE;
        for joint in ROWS {
            if group_of(joint) == Some(self) {
                flags::insert(&mut set, joint);
            }
        }
        set
    }
}

/// Membership operations on [`JointFlags`] by [`JointRef`].
///
/// The only code that knows which bit is which bus row. Bit for bus row `n`
/// is `1 << n`, matching the vocabulary's declared constants, the wire masks
/// and the sequencers' per-row arrays. A leg index past the sixth names no
/// bit: it is in no set and cannot be put in one.
pub mod flags {
    use super::{JointGroup, JointRef, Name, ROW_COUNT, ROWS, row};
    use brenn_reachy__motion__joints_clk_rs::JointFlags;

    /// Every servo on the bus.
    ///
    /// The canonical source for "all of them"; that this is the same nine
    /// [`ROWS`] names is a test, not a comment. A function and not a
    /// constant because the vocabulary's union operator is not `const fn`.
    #[must_use]
    pub fn all() -> JointFlags {
        from_rows(&[true; ROW_COUNT])
    }

    /// The single-servo set `joint` names, or the empty set if it names no
    /// servo.
    #[must_use]
    pub fn bit(joint: JointRef) -> JointFlags {
        match row(joint) {
            Some(row) => BITS[row],
            None => JointFlags::NONE,
        }
    }

    /// One bit per bus row, in bus order.
    const BITS: [JointFlags; ROW_COUNT] = [
        JointFlags::BODY_YAW,
        JointFlags::LEG_0,
        JointFlags::LEG_1,
        JointFlags::LEG_2,
        JointFlags::LEG_3,
        JointFlags::LEG_4,
        JointFlags::LEG_5,
        JointFlags::ANTENNA_RIGHT,
        JointFlags::ANTENNA_LEFT,
    ];

    /// Add `joint`, reporting whether it was not already there.
    ///
    /// The return is what makes an entry an event: a fault that names a servo
    /// already in the set is the same fault standing, not a new one. A ref that
    /// names no servo adds nothing and reports nothing added.
    pub fn insert(set: &mut JointFlags, joint: JointRef) -> bool {
        let bit = bit(joint);
        if bit == JointFlags::NONE || set.contains(bit) {
            return false;
        }
        *set |= bit;
        true
    }

    /// Whether `joint` is in `set`.
    #[must_use]
    pub fn contains(set: JointFlags, joint: JointRef) -> bool {
        let bit = bit(joint);
        bit != JointFlags::NONE && set.contains(bit)
    }

    /// Whether the set is empty.
    #[must_use]
    pub fn is_empty(set: JointFlags) -> bool {
        set == JointFlags::NONE
    }

    /// How many servos are in it.
    #[must_use]
    pub fn len(set: JointFlags) -> u32 {
        u32::try_from(iter(set).count()).unwrap_or(0)
    }

    /// The servos in `set` and not in `other`.
    ///
    /// The vocabulary type has no complement operator — the bits above the ninth
    /// belong to no servo and a complement would hand them out — so a difference
    /// is taken over the servos rather than over the word.
    #[must_use]
    pub fn without(set: JointFlags, other: JointFlags) -> JointFlags {
        let mut kept = JointFlags::NONE;
        for joint in iter(set) {
            if !contains(other, joint) {
                insert(&mut kept, joint);
            }
        }
        kept
    }

    /// Whether every servo of `group` is in `set`.
    #[must_use]
    pub fn covers(set: JointFlags, group: JointGroup) -> bool {
        set.contains(group.joints())
    }

    /// The set those nine flags name, one flag per bus row.
    ///
    /// `[bool; ROW_COUNT]` is how the sequencers carry a per-servo answer
    /// and a set is how a slot, a mask and a wire field carry one, so the two
    /// shapes meet in several hosts and the convention is written down once.
    #[must_use]
    pub fn from_rows(rows: &[bool; ROW_COUNT]) -> JointFlags {
        let mut set = JointFlags::NONE;
        for (row, present) in rows.iter().enumerate() {
            if *present {
                set |= BITS[row];
            }
        }
        set
    }

    /// The inverse of [`from_rows`]: one flag per bus row.
    #[must_use]
    pub fn rows(set: JointFlags) -> [bool; ROW_COUNT] {
        let mut rows = [false; ROW_COUNT];
        for (row, flag) in rows.iter_mut().enumerate() {
            *flag = set.contains(BITS[row]);
        }
        rows
    }

    /// Everything in the set, in bus order.
    pub fn iter(set: JointFlags) -> impl Iterator<Item = JointRef> {
        ROWS.into_iter().filter(move |joint| contains(set, *joint))
    }

    /// A set rendered as the servos it holds, for a message an operator reads.
    ///
    /// An adapter and not a `Display` impl on the set itself, because the type
    /// belongs to the vocabulary crate and a rendering is this layer's opinion
    /// rather than the vocabulary's.
    pub struct Names(pub JointFlags);

    impl core::fmt::Display for Names {
        /// The servos by name, comma-separated, or `nothing` for an empty set.
        fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
            if is_empty(self.0) {
                return f.write_str("nothing");
            }
            for (written, joint) in iter(self.0).enumerate() {
                if written > 0 {
                    f.write_str(", ")?;
                }
                write!(f, "{}", Name(joint))?;
            }
            Ok(())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use brenn_reachy__motion__joints_clk_rs::JointsWire;
    use reachy_kin::{
        EnvelopeConfig, EnvelopeReport, EnvelopeViolations, HeadGeometry, check_envelope,
    };

    /// The constant and the nine servos are one thing said twice, which is only
    /// safe while they agree.
    #[test]
    fn the_whole_bus_is_every_joint_the_bus_has() {
        let folded = ROWS.into_iter().fold(JointFlags::NONE, |set, joint| {
            let mut set = set;
            assert!(
                flags::insert(&mut set, joint),
                "{} is on the bus once",
                Name(joint)
            );
            set
        });
        assert_eq!(flags::all(), folded);
        assert_eq!(
            usize::try_from(flags::len(flags::all())).expect("nine servos is a small number"),
            ROW_COUNT,
        );
        for joint in ROWS {
            assert!(
                flags::contains(flags::all(), joint),
                "{} is on the bus",
                Name(joint)
            );
        }
    }

    /// The bit for a bus row is the vocabulary's own bit for that row, in both
    /// directions, and the nine rows are the whole of the type.
    ///
    /// The set a slot holds and the set this crate operates on are one value, so
    /// what is worth pinning is that a row's name, its bit and its index agree —
    /// a disagreement would put one servo's mask on another servo's name.
    #[test]
    fn a_row_and_its_bit_name_the_same_servo() {
        let declared = [
            (JointRef::BodyYaw, JointFlags::BODY_YAW),
            (JointRef::Leg0, JointFlags::LEG_0),
            (JointRef::Leg1, JointFlags::LEG_1),
            (JointRef::Leg2, JointFlags::LEG_2),
            (JointRef::Leg3, JointFlags::LEG_3),
            (JointRef::Leg4, JointFlags::LEG_4),
            (JointRef::Leg5, JointFlags::LEG_5),
            (JointRef::AntennaRight, JointFlags::ANTENNA_RIGHT),
            (JointRef::AntennaLeft, JointFlags::ANTENNA_LEFT),
        ];
        for (index, (joint, bit)) in declared.into_iter().enumerate() {
            assert_eq!(
                row(joint),
                Some(index),
                "{} is bus row {index}",
                Name(joint)
            );
            assert_eq!(
                flags::bit(joint),
                bit,
                "{} carries its own bit",
                Name(joint)
            );
            let mut one = JointFlags::NONE;
            assert!(flags::insert(&mut one, joint));
            assert_eq!(one, bit, "a set of one is that servo's bit");
            assert!(flags::rows(one)[index], "{} is row {index}", Name(joint));
        }
    }

    /// A leg index past the sixth names no bit, so it is in no set and cannot be
    /// put in one.
    #[test]
    fn a_leg_the_machine_does_not_have_is_in_no_set() {
        let mut set = flags::all();
        assert!(!flags::insert(&mut set, leg_ref(6)));
        assert_eq!(set, flags::all(), "and the set it was offered to stands");
        assert!(!flags::contains(flags::all(), leg_ref(9)));
        assert_eq!(flags::bit(leg_ref(6)), JointFlags::NONE);
    }

    /// The worst-deviation selection is an ordering, and a value nobody can
    /// place wins it. Reusing the bound test here would let the unplaceable
    /// joint be displaced by the next joint in the sweep, and the report whose
    /// whole job is naming a joint would name the wrong one with a zero beside
    /// it.
    #[test]
    fn the_worst_error_selection_is_an_ordering() {
        assert!(worse_error(0.2, 0.1));
        assert!(!worse_error(0.1, 0.2));
        assert!(!worse_error(0.1, 0.1), "a tie keeps the first joint");
        assert!(worse_error(f64::NAN, 0.5), "unplaceable beats any number");
        assert!(!worse_error(0.5, f64::NAN), "and is not displaced by one");
        assert!(
            !worse_error(f64::NAN, f64::NAN),
            "a tie between two of them"
        );
    }

    /// The sweep around that ordering, seed included: every joint is a
    /// candidate including the first, a tie keeps the earlier one, and an
    /// unplaceable deviation is named wherever it sits in the order.
    #[test]
    fn the_worst_joint_sweep_covers_its_own_seed() {
        let mut deviations = [0.0; ROW_COUNT];
        assert_eq!(
            worst_joint(&deviations),
            (JointRef::BodyYaw, 0.0),
            "all equal keeps the first row"
        );

        deviations[0] = 0.4;
        assert_eq!(
            worst_joint(&deviations),
            (JointRef::BodyYaw, 0.4),
            "the seed row wins when it is the worst"
        );

        deviations[8] = 0.5;
        assert_eq!(worst_joint(&deviations), (JointRef::AntennaLeft, 0.5));

        deviations[3] = 0.5;
        assert_eq!(
            worst_row(&deviations),
            3,
            "a tie between two rows keeps the earlier"
        );

        deviations[6] = f64::NAN;
        let (joint, deviation) = worst_joint(&deviations);
        assert_eq!(joint, JointRef::Leg5);
        assert!(
            deviation.is_nan(),
            "unplaceable, and named as its own joint"
        );
    }

    #[test]
    fn bus_order_is_one_order() {
        let v = JointVector {
            body_yaw: 0.5,
            legs: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            antennas: [7.0, 8.0],
        };
        let joints = v.joints();
        for (index, (id, angle)) in joints.iter().enumerate() {
            assert_eq!(joint_ref(index), Some(*id), "index {index}");
            assert_eq!(row(*id), Some(index), "index {index}");
            assert_eq!(v.get(*id), Some(*angle), "index {index}");
        }
        assert_eq!(joint_ref(ROW_COUNT), None);
    }

    /// Writing a joint writes that joint and leaves the other eight alone.
    ///
    /// A per-slot sweep because this is how a position sweep is assembled, one
    /// servo's answer at a time: a `set` that routed the left antenna's reading
    /// into the right antenna's slot would arm the machine with each antenna
    /// pinned at the other's measured angle and drag both there under torque, and
    /// no test of a whole armed sequence would see it.
    #[test]
    fn every_slot_is_written_where_it_is_named() {
        for (index, id) in ROWS.into_iter().enumerate() {
            let before = JointVector {
                body_yaw: 0.5,
                legs: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
                antennas: [7.0, 8.0],
            };
            let mut after = before;
            assert!(after.set(id, -1.0), "slot {index}");
            assert_eq!(after.get(id), Some(-1.0), "slot {index}");
            for (other, angle) in before.joints() {
                if other != id {
                    assert_eq!(
                        after.get(other),
                        Some(angle),
                        "{} changed with {}",
                        Name(other),
                        Name(id)
                    );
                }
            }
        }

        // A ref naming no servo writes nothing anywhere.
        let mut v = JointVector::default();
        assert!(!v.set(JointRef::None, 1.0));
        assert_eq!(v, JointVector::default());
    }

    /// A leg index past the sixth names no servo, and says so rather than
    /// naming another one.
    #[test]
    fn out_of_range_leg_has_no_slot() {
        let v = JointVector::default();
        assert_eq!(leg_ref(6), JointRef::None);
        assert_eq!(leg_ref(200), JointRef::None);
        assert_eq!(row(JointRef::None), None);
        assert_eq!(v.get(JointRef::None), None);
    }

    #[test]
    fn joint_names() {
        assert_eq!(Name(JointRef::None).to_string(), "no joint");
        assert_eq!(Name(JointRef::BodyYaw).to_string(), "body yaw");
        assert_eq!(Name(JointRef::Leg0).to_string(), "leg 1");
        assert_eq!(Name(JointRef::Leg3).to_string(), "leg 4");
        assert_eq!(Name(JointRef::Leg5).to_string(), "leg 6");
        assert_eq!(Name(JointRef::AntennaRight).to_string(), "right antenna");
        assert_eq!(Name(JointRef::AntennaLeft).to_string(), "left antenna");
    }

    /// The two crates must name the same physical crank the same way. Two
    /// numberings in one log is a wrong-part diagnosis on a mechanism with six
    /// identical-looking legs, and each crate's own test would pass either way.
    #[test]
    fn both_crates_number_the_legs_alike() {
        for leg in 0..6usize {
            let mut violations = EnvelopeViolations::default();
            violations.unreachable[leg] = true;
            let envelope_says = violations.to_string();
            let motion_says = Name(leg_ref(leg as u8)).to_string();
            assert!(
                envelope_says.starts_with(&format!("{motion_says} ")),
                "the envelope says {envelope_says:?}, the tick says {motion_says:?}"
            );
        }
    }

    /// Every one of the nine slots is covered by the finiteness check, and the
    /// slot it finds is the one it names — a per-slot sweep, because a
    /// hand-written conjunction that skipped one would still pass an all-finite
    /// test, and a check that named the wrong joint would still refuse.
    #[test]
    fn every_slot_is_checked_for_finiteness() {
        assert_eq!(JointVector::default().first_non_finite(), None);
        for (index, expected) in ROWS.iter().enumerate() {
            for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
                let mut v = JointVector::default();
                match index {
                    0 => v.body_yaw = bad,
                    1..=6 => v.legs[index - 1] = bad,
                    _ => v.antennas[index - 7] = bad,
                }
                assert_eq!(
                    v.first_non_finite(),
                    Some(*expected),
                    "slot {index} with {bad}"
                );
            }
        }
    }

    /// The same sweep over the target set, pose components included.
    #[test]
    fn targets_check_the_pose_too() {
        assert!(JointTargets::default().is_finite());
        for bad in [f64::NAN, f64::INFINITY] {
            let mut t = JointTargets::default();
            t.head_pose_body.translation.vector.x = bad;
            assert!(!t.is_finite(), "translation with {bad}");

            let mut t = JointTargets::default();
            t.head_pose_body.rotation = nalgebra::UnitQuaternion::new_unchecked(
                nalgebra::Quaternion::new(bad, 0.0, 0.0, 0.0),
            );
            assert!(!t.is_finite(), "rotation with {bad}");

            let t = JointTargets {
                body_yaw: bad,
                ..Default::default()
            };
            assert!(!t.is_finite(), "yaw with {bad}");

            let mut t = JointTargets::default();
            t.antennas[1] = bad;
            assert!(!t.is_finite(), "antenna with {bad}");
        }
    }

    /// The voltage bit alone is the informational case; any other bit is not,
    /// and the two predicates agree on which is which.
    #[test]
    fn only_the_voltage_bit_is_tolerated() {
        for bits in 0..=u8::MAX {
            let health = ServoHealth { id: 11, bits };
            let voltage_only = bits == ServoHealth::INPUT_VOLTAGE;
            assert_eq!(health.voltage_only(), voltage_only, "bits {bits:#04x}");
            assert_eq!(
                health.healthy_or_voltage_only(),
                bits == 0 || voltage_only,
                "bits {bits:#04x}"
            );
        }
    }

    #[test]
    fn step_bounds_by_group() {
        let step = JointStep {
            legs: 0.05,
            body_yaw: 0.02,
            antennas: 0.1,
        };
        assert_eq!(step.for_joint(JointRef::BodyYaw), 0.02);
        assert_eq!(step.for_joint(JointRef::Leg4), 0.05);
        assert_eq!(step.for_joint(JointRef::AntennaRight), 0.1);
        assert_eq!(step.for_joint(JointRef::AntennaLeft), 0.1);
    }

    /// The three groups partition the nine joints, and each one is a run of
    /// consecutive bus rows — which is what lets a grouped write ask for a
    /// group rather than restate a row range.
    #[test]
    fn the_groups_partition_bus_order_in_runs() {
        let mut seen = 0;
        let mut first_row = Vec::new();
        for group in JointGroup::ALL {
            let rows: Vec<usize> = ROWS
                .into_iter()
                .enumerate()
                .filter(|(_, joint)| group_of(*joint) == Some(group))
                .map(|(row, _)| row)
                .collect();
            assert!(!rows.is_empty(), "{group:?} names no joint");
            for pair in rows.windows(2) {
                assert_eq!(pair[1], pair[0] + 1, "{group:?} is not consecutive");
            }
            first_row.push(rows[0]);
            seen += rows.len();
        }
        assert_eq!(seen, ROW_COUNT);
        assert_eq!(first_row, vec![0, 1, 7]);
    }

    /// Membership, in bus order, with the second entry of a joint reported as
    /// the no-op it is — the distinction a fault raise turns on.
    #[test]
    fn a_joint_set_admits_each_joint_once() {
        let mut set = JointFlags::NONE;
        assert!(flags::is_empty(set));
        assert_eq!(flags::len(set), 0);
        assert!(
            flags::insert(&mut set, JointRef::AntennaLeft),
            "the first entry is news"
        );
        assert!(
            !flags::insert(&mut set, JointRef::AntennaLeft),
            "the second is not"
        );
        assert!(flags::insert(&mut set, JointRef::Leg2));
        assert_eq!(flags::len(set), 2);
        assert!(flags::contains(set, JointRef::Leg2) && !flags::contains(set, JointRef::Leg3));
        assert_eq!(
            flags::iter(set).collect::<Vec<JointRef>>(),
            vec![JointRef::Leg2, JointRef::AntennaLeft],
            "bus order, whatever order they went in"
        );
        assert_eq!(format!("{}", flags::Names(set)), "leg 3, left antenna");
        assert_eq!(format!("{}", flags::Names(JointFlags::NONE)), "nothing");
    }

    /// A leg index no machine carries is in no set: it has no bus row to be a
    /// bit of, and a set that swallowed it would answer `contains` false for
    /// something it claimed to hold.
    #[test]
    fn a_joint_set_holds_only_joints_that_exist() {
        let mut set = JointFlags::NONE;
        assert!(!flags::insert(&mut set, leg_ref(9)));
        assert!(flags::is_empty(set));
        assert!(!flags::contains(set, leg_ref(9)));
    }

    /// Union and difference, which is how a caller holding a set adds servos to
    /// it or takes them out without ever writing a shift.
    #[test]
    fn joints_go_into_a_set_and_come_out_of_it_by_name() {
        let cranks = JointGroup::Legs.joints();
        let antennas = JointGroup::Antennas.joints();

        let both = cranks | antennas;
        assert_eq!(flags::len(both), flags::len(cranks) + flags::len(antennas));
        assert!(flags::covers(both, JointGroup::Legs) && flags::covers(both, JointGroup::Antennas));
        assert!(!flags::contains(both, JointRef::BodyYaw));
        assert_eq!(both | cranks, both, "a union with a subset is itself");

        let left = flags::without(both, antennas);
        assert_eq!(left, cranks);
        assert!(flags::is_empty(flags::without(left, cranks)));
        assert_eq!(
            flags::without(left, JointFlags::NONE),
            cranks,
            "taking nothing out changes nothing"
        );
    }

    /// Group coverage, which is what says a mask has taken a whole group out of
    /// service.
    #[test]
    fn a_joint_set_covers_a_group_only_when_it_holds_all_of_it() {
        let mut set = JointFlags::NONE;
        flags::insert(&mut set, JointRef::AntennaRight);
        assert!(!flags::covers(set, JointGroup::Antennas));
        flags::insert(&mut set, JointRef::AntennaLeft);
        assert!(flags::covers(set, JointGroup::Antennas));
        assert!(!flags::covers(set, JointGroup::Legs));
        assert!(!flags::covers(set, JointGroup::BodyYaw));
        for group in JointGroup::ALL {
            let whole = group.joints();
            assert!(flags::covers(whole, group));
            assert_eq!(
                flags::len(whole),
                ROWS.into_iter()
                    .filter(|joint| group_of(*joint) == Some(group))
                    .count()
                    .try_into()
                    .expect("nine joints fit in a u32"),
            );
        }
    }

    /// The default target set is a configuration the machine can hold, not a
    /// zeroed struct: it passes the envelope with no baseline.
    #[test]
    fn default_targets_pass_the_envelope() {
        let targets = JointTargets::default();
        let mut report = EnvelopeReport::default();
        check_envelope(
            &HeadGeometry::default(),
            &EnvelopeConfig::default(),
            &targets.head_pose_body,
            targets.body_yaw,
            None,
            &mut report,
        )
        .expect("the neutral pose is inside the envelope");
    }

    /// Every joint's flag lands on its own bus row and comes back off it, one
    /// joint at a time — which is the transposition the row form exists to make
    /// impossible.
    #[test]
    fn one_flag_per_row_crosses_both_ways() {
        for joint in ROWS {
            let row = row(joint).expect("every joint has a bus row");
            let mut rows = [false; ROW_COUNT];
            rows[row] = true;

            let set = flags::from_rows(&rows);
            assert_eq!(flags::len(set), 1, "{} alone", Name(joint));
            assert!(
                flags::contains(set, joint),
                "{} is the one in it",
                Name(joint)
            );
            assert_eq!(
                flags::rows(set),
                rows,
                "{} back on its own row",
                Name(joint)
            );
        }
    }

    /// The empty set and the whole bus, the two ends of the crossing.
    #[test]
    fn the_empty_set_and_the_whole_bus_cross() {
        assert_eq!(flags::from_rows(&[false; ROW_COUNT]), JointFlags::NONE);
        assert_eq!(flags::rows(JointFlags::NONE), [false; ROW_COUNT]);
        assert_eq!(flags::from_rows(&[true; ROW_COUNT]), flags::all());
        assert_eq!(flags::rows(flags::all()), [true; ROW_COUNT]);
    }

    /// A servo's angle lands on the field that servo names.
    ///
    /// Written through [`set_angle`] and read off the schema's own nine fields,
    /// listed here in bus order rather than asked for through the same mapping
    /// that wrote them: a round trip through one mapping says only that the
    /// mapping is a bijection, and the swap named fields exist to prevent — one
    /// servo's reading under another servo's name — is a bijection too. This is
    /// the second, independent statement of the pairing, and it covers every
    /// schema [`rows_by_joint`] is instantiated for, since all of them get the
    /// nine field names from the one macro body.
    #[test]
    fn an_angle_lands_on_the_field_its_own_servo_names() {
        let mut wire = JointsWire::new();
        let joints = wire.clear_valid();
        for (row, joint) in ROWS.into_iter().enumerate() {
            assert!(
                set_angle(
                    joints,
                    joint,
                    f64::from(u8::try_from(row).expect("a small row")) + 1.0
                ),
                "{} has a field",
                Name(joint)
            );
        }
        assert_eq!(
            [
                joints.body_yaw,
                joints.leg_0,
                joints.leg_1,
                joints.leg_2,
                joints.leg_3,
                joints.leg_4,
                joints.leg_5,
                joints.antenna_right,
                joints.antenna_left,
            ],
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0]
        );
        assert!(
            !set_angle(joints, JointRef::None, 9.9),
            "no servo, no field"
        );
    }

    /// A mixed set survives the round trip whole.
    #[test]
    fn a_mixed_set_is_the_same_set_after_a_round_trip() {
        let mut set = JointFlags::NONE;
        flags::insert(&mut set, JointRef::AntennaLeft);
        flags::insert(&mut set, JointRef::Leg2);
        flags::insert(&mut set, JointRef::BodyYaw);
        assert_eq!(flags::from_rows(&flags::rows(set)), set);
    }

    /// Every angle of the library's vector lands on the field its own servo
    /// names, and comes back on the same one.
    #[test]
    fn every_joint_lands_on_its_own_named_field() {
        let angles = JointVector {
            body_yaw: 0.101,
            legs: [0.201, 0.202, 0.203, 0.204, 0.205, 0.206],
            antennas: [0.301, -0.302],
        };
        let mut wire = JointsWire::new();
        let joints = wire.clear_valid();
        write_vector(joints, &angles);

        assert_eq!(joints.body_yaw, angles.body_yaw);
        assert_eq!(
            [
                joints.leg_0,
                joints.leg_1,
                joints.leg_2,
                joints.leg_3,
                joints.leg_4,
                joints.leg_5
            ],
            angles.legs
        );
        assert_eq!([joints.antenna_right, joints.antenna_left], angles.antennas);
        assert_eq!(vector_of(joints), angles);
    }

    /// The seam where a row of angles meets a name: every row has to arrive at
    /// the joint whose bus row it is, and come back on the same row.
    #[test]
    fn a_row_of_angles_keeps_its_bus_order_both_ways() {
        let rows: [f64; ROW_COUNT] =
            core::array::from_fn(|row| f64::from(u8::try_from(row).expect("nine rows")) + 0.5);
        let mut wire = JointsWire::new();
        let joints = wire.clear_valid();
        write_rows(joints, &rows);

        for id in ROWS {
            let at = row(id).expect("every servo has a row");
            assert_eq!(
                angle_of(joints, id).expect("every servo has an angle"),
                rows[at],
                "{} took row {at}",
                Name(id)
            );
        }
        assert_eq!(rows_of(joints), rows);
    }
}
