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
        match id {
            JointId::BodyYaw => self.body_yaw,
            JointId::Leg(_) => self.legs,
            JointId::AntennaRight | JointId::AntennaLeft => self.antennas,
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
            targets.antennas,
            None,
            &mut report,
        )
        .expect("the neutral pose is inside the envelope");
    }
}
