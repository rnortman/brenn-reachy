//! The two command vectors, the names of the nine joints, and a servo's health
//! byte.
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
//! [`ServoHealth`] is here for the same reason the joint names are: it is
//! shared between tick and arming rather than owned by either.

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
    /// The angle at `id`, or `None` for a leg index past the sixth.
    #[must_use]
    pub fn get(&self, id: JointId) -> Option<f64> {
        match id {
            JointId::BodyYaw => Some(self.body_yaw),
            JointId::Leg(leg) => self.legs.get(usize::from(leg)).copied(),
            JointId::AntennaRight => Some(self.antennas[0]),
            JointId::AntennaLeft => Some(self.antennas[1]),
        }
    }

    /// Set the angle at `id`, reporting `false` — and changing nothing — for a
    /// leg index past the sixth.
    ///
    /// The mirror of [`Self::get`], for filling a vector one servo's answer at a
    /// time as the reads come back.
    pub fn set(&mut self, id: JointId, angle: f64) -> bool {
        match id {
            JointId::BodyYaw => self.body_yaw = angle,
            JointId::Leg(leg) => match self.legs.get_mut(usize::from(leg)) {
                Some(slot) => *slot = angle,
                None => return false,
            },
            JointId::AntennaRight => self.antennas[0] = angle,
            JointId::AntennaLeft => self.antennas[1] = angle,
        }
        true
    }

    /// Every joint paired with its angle, in bus order.
    ///
    /// A single pass over all nine joints ensures no joint is checked by one
    /// guard and missed by another. Fixed size, so nothing allocates.
    #[must_use]
    pub fn joints(&self) -> [(JointId, f64); JointId::COUNT] {
        [
            (JointId::BodyYaw, self.body_yaw),
            (JointId::Leg(0), self.legs[0]),
            (JointId::Leg(1), self.legs[1]),
            (JointId::Leg(2), self.legs[2]),
            (JointId::Leg(3), self.legs[3]),
            (JointId::Leg(4), self.legs[4]),
            (JointId::Leg(5), self.legs[5]),
            (JointId::AntennaRight, self.antennas[0]),
            (JointId::AntennaLeft, self.antennas[1]),
        ]
    }

    /// The first joint in bus order whose angle is not a number, if any.
    ///
    /// Named rather than counted, so a fault raised from this can name the
    /// joint the bad number arrived on.
    #[must_use]
    pub fn first_non_finite(&self) -> Option<JointId> {
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
    /// The bound that applies to `id`.
    #[must_use]
    pub fn for_joint(&self, id: JointId) -> f64 {
        match id.group() {
            JointGroup::BodyYaw => self.body_yaw,
            JointGroup::Legs => self.legs,
            JointGroup::Antennas => self.antennas,
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
pub(crate) fn worst_row(deviations: &[f64; JointId::COUNT]) -> usize {
    let mut worst = 0;
    for (row, deviation) in deviations.iter().enumerate() {
        if worse_error(*deviation, deviations[worst]) {
            worst = row;
        }
    }
    worst
}

/// The joint furthest out and how far, from deviations in bus order.
pub(crate) fn worst_joint(deviations: &[f64; JointId::COUNT]) -> (JointId, f64) {
    let row = worst_row(deviations);
    (JointId::ALL[row], deviations[row])
}

/// Names one joint, for fault causes and reports.
///
/// The `Leg` payload is 0-based, matching the leg index the kinematics reports an
/// unreachable pose against. It **renders** 1-based, matching the servo numbering
/// on the bus and the way the envelope names a leg: an operator reading a fault
/// and an envelope refusal in the same log must be reading about the same crank.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum JointId {
    /// The body yaw servo.
    BodyYaw,
    /// A crank, 0-based in servo order.
    Leg(u8),
    /// The right antenna.
    AntennaRight,
    /// The left antenna.
    AntennaLeft,
}

impl JointId {
    /// How many joints there are.
    pub const COUNT: usize = 9;

    /// Every joint, in bus order.
    pub const ALL: [JointId; Self::COUNT] = [
        JointId::BodyYaw,
        JointId::Leg(0),
        JointId::Leg(1),
        JointId::Leg(2),
        JointId::Leg(3),
        JointId::Leg(4),
        JointId::Leg(5),
        JointId::AntennaRight,
        JointId::AntennaLeft,
    ];

    /// Position in bus order, or `None` for a leg index past the sixth.
    #[must_use]
    pub fn index(self) -> Option<usize> {
        match self {
            JointId::BodyYaw => Some(0),
            JointId::Leg(leg) if leg < 6 => Some(1 + usize::from(leg)),
            JointId::Leg(_) => None,
            JointId::AntennaRight => Some(7),
            JointId::AntennaLeft => Some(8),
        }
    }

    /// The joint at `index` in bus order.
    #[must_use]
    pub fn from_index(index: usize) -> Option<Self> {
        Self::ALL.get(index).copied()
    }

    /// Which group this joint belongs to.
    #[must_use]
    pub fn group(self) -> JointGroup {
        match self {
            JointId::BodyYaw => JointGroup::BodyYaw,
            JointId::Leg(_) => JointGroup::Legs,
            JointId::AntennaRight | JointId::AntennaLeft => JointGroup::Antennas,
        }
    }
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
    pub fn joints(self) -> JointSet {
        let mut set = JointSet::EMPTY;
        for joint in JointId::ALL {
            if joint.group() == self {
                set.insert(joint);
            }
        }
        set
    }
}

/// A set of joints, one bit per bus row.
///
/// What names the joints a decision covers without allocating or fixing an
/// order on the caller: the servos a fault took out of service, the rows a
/// write skips. Membership is by bus row, so a leg index past the sixth — a
/// [`JointId`] no machine carries — is in no set and cannot be put in one.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct JointSet(u16);

impl JointSet {
    /// The set with nothing in it.
    pub const EMPTY: Self = Self(0);

    /// Add `joint`, reporting whether it was not already there.
    ///
    /// The return is what makes an entry an event: a fault that names a servo
    /// already in the set is the same fault standing, not a new one.
    pub fn insert(&mut self, joint: JointId) -> bool {
        let Some(index) = joint.index() else {
            return false;
        };
        let bit = 1 << index;
        let fresh = self.0 & bit == 0;
        self.0 |= bit;
        fresh
    }

    /// Whether `joint` is in the set.
    #[must_use]
    pub fn contains(self, joint: JointId) -> bool {
        joint
            .index()
            .is_some_and(|index| self.0 & (1 << index) != 0)
    }

    /// Whether the set is empty.
    #[must_use]
    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// How many joints are in it.
    #[must_use]
    pub fn len(self) -> u32 {
        self.0.count_ones()
    }

    /// Whether every joint of `group` is in the set.
    #[must_use]
    pub fn covers(self, group: JointGroup) -> bool {
        JointId::ALL
            .into_iter()
            .filter(|joint| joint.group() == group)
            .all(|joint| self.contains(joint))
    }

    /// Everything in the set, in bus order.
    pub fn iter(self) -> impl Iterator<Item = JointId> {
        JointId::ALL
            .into_iter()
            .filter(move |joint| self.contains(*joint))
    }
}

impl core::fmt::Display for JointSet {
    /// The joints by name, comma-separated, or `nothing` for an empty set.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        if self.is_empty() {
            return f.write_str("nothing");
        }
        for (written, joint) in self.iter().enumerate() {
            if written > 0 {
                f.write_str(", ")?;
            }
            write!(f, "{joint}")?;
        }
        Ok(())
    }
}

impl core::fmt::Display for JointId {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            JointId::BodyYaw => write!(f, "body yaw"),
            // 1-based, as the servos and the envelope's own messages number the
            // legs; the payload stays the 0-based index.
            JointId::Leg(leg) => write!(f, "leg {}", u16::from(*leg) + 1),
            JointId::AntennaRight => write!(f, "right antenna"),
            JointId::AntennaLeft => write!(f, "left antenna"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use reachy_kin::{
        EnvelopeConfig, EnvelopeReport, EnvelopeViolations, HeadGeometry, check_envelope,
    };

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
        let mut deviations = [0.0; JointId::COUNT];
        assert_eq!(
            worst_joint(&deviations),
            (JointId::BodyYaw, 0.0),
            "all equal keeps the first row"
        );

        deviations[0] = 0.4;
        assert_eq!(
            worst_joint(&deviations),
            (JointId::BodyYaw, 0.4),
            "the seed row wins when it is the worst"
        );

        deviations[8] = 0.5;
        assert_eq!(worst_joint(&deviations), (JointId::AntennaLeft, 0.5));

        deviations[3] = 0.5;
        assert_eq!(
            worst_row(&deviations),
            3,
            "a tie between two rows keeps the earlier"
        );

        deviations[6] = f64::NAN;
        let (joint, deviation) = worst_joint(&deviations);
        assert_eq!(joint, JointId::Leg(5));
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
            assert_eq!(JointId::from_index(index), Some(*id), "index {index}");
            assert_eq!(id.index(), Some(index), "index {index}");
            assert_eq!(v.get(*id), Some(*angle), "index {index}");
        }
        assert_eq!(JointId::from_index(JointId::COUNT), None);
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
        for (index, id) in JointId::ALL.into_iter().enumerate() {
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
                    assert_eq!(after.get(other), Some(angle), "{other} changed with {id}");
                }
            }
        }

        // A leg index past the sixth writes nothing anywhere.
        let mut v = JointVector::default();
        assert!(!v.set(JointId::Leg(6), 1.0));
        assert!(!v.set(JointId::Leg(200), 1.0));
        assert_eq!(v, JointVector::default());
    }

    /// A leg index past the sixth names nothing, and says so rather than
    /// indexing something else.
    #[test]
    fn out_of_range_leg_has_no_slot() {
        let v = JointVector::default();
        assert_eq!(JointId::Leg(6).index(), None);
        assert_eq!(JointId::Leg(200).index(), None);
        assert_eq!(v.get(JointId::Leg(6)), None);
    }

    #[test]
    fn joint_names() {
        assert_eq!(JointId::BodyYaw.to_string(), "body yaw");
        assert_eq!(JointId::Leg(0).to_string(), "leg 1");
        assert_eq!(JointId::Leg(3).to_string(), "leg 4");
        assert_eq!(JointId::Leg(5).to_string(), "leg 6");
        assert_eq!(JointId::AntennaRight.to_string(), "right antenna");
        assert_eq!(JointId::AntennaLeft.to_string(), "left antenna");
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
            let motion_says = JointId::Leg(leg as u8).to_string();
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
        for (index, expected) in JointId::ALL.iter().enumerate() {
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
        assert_eq!(step.for_joint(JointId::BodyYaw), 0.02);
        assert_eq!(step.for_joint(JointId::Leg(4)), 0.05);
        assert_eq!(step.for_joint(JointId::AntennaRight), 0.1);
        assert_eq!(step.for_joint(JointId::AntennaLeft), 0.1);
    }

    /// The three groups partition the nine joints, and each one is a run of
    /// consecutive bus rows — which is what lets a grouped write ask for a
    /// group rather than restate a row range.
    #[test]
    fn the_groups_partition_bus_order_in_runs() {
        let mut seen = 0;
        let mut first_row = Vec::new();
        for group in JointGroup::ALL {
            let rows: Vec<usize> = JointId::ALL
                .into_iter()
                .enumerate()
                .filter(|(_, joint)| joint.group() == group)
                .map(|(row, _)| row)
                .collect();
            assert!(!rows.is_empty(), "{group:?} names no joint");
            for pair in rows.windows(2) {
                assert_eq!(pair[1], pair[0] + 1, "{group:?} is not consecutive");
            }
            first_row.push(rows[0]);
            seen += rows.len();
        }
        assert_eq!(seen, JointId::COUNT);
        assert_eq!(first_row, vec![0, 1, 7]);
    }

    /// Membership, in bus order, with the second entry of a joint reported as
    /// the no-op it is — the distinction a fault raise turns on.
    #[test]
    fn a_joint_set_admits_each_joint_once() {
        let mut set = JointSet::EMPTY;
        assert!(set.is_empty());
        assert_eq!(set.len(), 0);
        assert!(set.insert(JointId::AntennaLeft), "the first entry is news");
        assert!(!set.insert(JointId::AntennaLeft), "the second is not");
        assert!(set.insert(JointId::Leg(2)));
        assert_eq!(set.len(), 2);
        assert!(set.contains(JointId::Leg(2)) && !set.contains(JointId::Leg(3)));
        assert_eq!(
            set.iter().collect::<Vec<JointId>>(),
            vec![JointId::Leg(2), JointId::AntennaLeft],
            "bus order, whatever order they went in"
        );
        assert_eq!(format!("{set}"), "leg 3, left antenna");
        assert_eq!(format!("{}", JointSet::EMPTY), "nothing");
    }

    /// A leg index no machine carries is in no set: it has no bus row to be a
    /// bit of, and a set that swallowed it would answer `contains` false for
    /// something it claimed to hold.
    #[test]
    fn a_joint_set_holds_only_joints_that_exist() {
        let mut set = JointSet::EMPTY;
        assert!(!set.insert(JointId::Leg(9)));
        assert!(set.is_empty());
        assert!(!set.contains(JointId::Leg(9)));
    }

    /// Group coverage, which is what says a mask has taken a whole group out of
    /// service.
    #[test]
    fn a_joint_set_covers_a_group_only_when_it_holds_all_of_it() {
        let mut set = JointSet::EMPTY;
        set.insert(JointId::AntennaRight);
        assert!(!set.covers(JointGroup::Antennas));
        set.insert(JointId::AntennaLeft);
        assert!(set.covers(JointGroup::Antennas));
        assert!(!set.covers(JointGroup::Legs));
        assert!(!set.covers(JointGroup::BodyYaw));
        for group in JointGroup::ALL {
            let whole = group.joints();
            assert!(whole.covers(group));
            assert_eq!(
                whole.len(),
                JointId::ALL
                    .into_iter()
                    .filter(|joint| joint.group() == group)
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
}
