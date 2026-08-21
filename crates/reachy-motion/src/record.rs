//! The record of an arming, as the slot holds it.
//!
//! Two of these bracket an engagement — where the poll found the platform
//! resting, and where the nine servos reported themselves once torque was on —
//! and a release reads the armed one back. The schema is the form they live in;
//! this module is the one place its fields are paired with the library's own
//! [`ArmRecord`], which carries a solved [`Isometry3`] the schema holds as a
//! translation and a quaternion.
//!
//! The pose is taken as written in both directions. A quaternion that is not a
//! rotation is refused rather than normalised, for the reason
//! [`crate::snap::PoseSnapshot`] states: a record is where the platform
//! actually is, and making a plausible rotation out of one that is not puts the
//! machine's idea of its own head wherever the arithmetic lands. Zeroes — what a
//! slot nothing wrote holds — are refused by the same check, which is what makes
//! `present` the only flag that says a record exists.
//!
//! The margins cross rather than being re-solved: they are what a first move's
//! clearance is measured against, and the seeds a solve would need are not in
//! the slot.

use nalgebra::Isometry3;

use crate::arm::{ArmRecord, PinOutcome};
use crate::joints;
use crate::snap::{PoseSnapshot, PoseSnapshotError};

pub use brenn_reachy__geometry__primitives_clk_rs::{Quat, Vec3};
pub use brenn_reachy__motion__arm_record_clk_rs::{ArmRecordSnap, Legs, PinSnap};

/// How many cranks a leg array holds one number for — the one collection in a
/// sequencer's state that is not indexed by bus row.
pub use crate::joints::LEG_COUNT;

/// Write a solved set of angles and the pose they hold.
///
/// `present` is set, which is what makes the record readable: a caller writing
/// no record at all uses [`clear`] instead.
pub fn write(out: &mut ArmRecordSnap, record: &ArmRecord) {
    out.present = true.into();
    joints::write_vector(&mut out.joints, &record.joints);
    write_pose(
        &mut out.head_pos,
        &mut out.head_quat,
        &record.head_pose_body,
    );
    write_legs(&mut out.margins, &record.margins);
    out.min_margin = record.min_margin;
}

/// The record those fields describe, or `None` where they hold none.
///
/// # Errors
///
/// [`PoseSnapshotError::NotARotation`] for a quaternion that is no rotation,
/// which is what a record nothing wrote holds and what a `present` flag set over
/// one is refused by.
pub fn read(slot: &ArmRecordSnap) -> Result<Option<ArmRecord>, PoseSnapshotError> {
    if !bool::from(slot.present) {
        return Ok(None);
    }
    Ok(Some(ArmRecord {
        joints: joints::vector_of(&slot.joints),
        head_pose_body: read_pose(&slot.head_pos, &slot.head_quat)?,
        margins: legs_of(&slot.margins),
        min_margin: slot.min_margin,
    }))
}

/// Leave the fields holding no record at all.
///
/// The quaternion is zeroed with the flag, so a slot whose `present` is set over
/// a record nothing wrote is refused by [`read`] rather than answering with the
/// identity rotation.
pub fn clear(out: &mut ArmRecordSnap) {
    out.present = false.into();
    out.min_margin = 0.0;
    joints::write_vector(&mut out.joints, &joints::JointVector::default());
    clear_pose(&mut out.head_pos, &mut out.head_quat);
    write_legs(&mut out.margins, &[0.0; LEG_COUNT]);
}

/// Write the goals an engagement pinned and how far each pin pulled a leg.
pub fn write_pins(out: &mut PinSnap, pins: &PinOutcome) {
    joints::write_vector(&mut out.pinned, &pins.pinned);
    write_legs(&mut out.pull_in, &pins.pull_in);
}

/// The pins those fields hold.
#[must_use]
pub fn pins_of(slot: &PinSnap) -> PinOutcome {
    PinOutcome {
        pinned: joints::vector_of(&slot.pinned),
        pull_in: legs_of(&slot.pull_in),
    }
}

/// Write one number per crank, in leg order.
///
/// Indexed directly, so the six constants are checked against the array's own
/// length where they are written.
pub fn write_legs(out: &mut Legs, legs: &[f64; LEG_COUNT]) {
    out.leg_0 = legs[0];
    out.leg_1 = legs[1];
    out.leg_2 = legs[2];
    out.leg_3 = legs[3];
    out.leg_4 = legs[4];
    out.leg_5 = legs[5];
}

/// The six numbers those fields hold.
#[must_use]
pub fn legs_of(slot: &Legs) -> [f64; LEG_COUNT] {
    [
        slot.leg_0, slot.leg_1, slot.leg_2, slot.leg_3, slot.leg_4, slot.leg_5,
    ]
}

/// Write a pose into the translation and the quaternion that hold it.
pub fn write_pose(pos: &mut Vec3, quat: &mut Quat, pose: &Isometry3<f64>) {
    let flat = PoseSnapshot::from(pose);
    pos.x = flat.pos_x;
    pos.y = flat.pos_y;
    pos.z = flat.pos_z;
    quat.w = flat.quat_w;
    quat.x = flat.quat_x;
    quat.y = flat.quat_y;
    quat.z = flat.quat_z;
}

/// Leave the two fields holding no pose at all.
///
/// Zeroed, and the zero quaternion is the one [`read_pose`] refuses: a presence
/// flag flipped over these bytes names no pose either, so an absent pose cannot
/// come back as the identity rotation. Stated once, because every pair of pose
/// fields in the tree means absence the same way.
pub fn clear_pose(pos: &mut Vec3, quat: &mut Quat) {
    pos.x = 0.0;
    pos.y = 0.0;
    pos.z = 0.0;
    quat.w = 0.0;
    quat.x = 0.0;
    quat.y = 0.0;
    quat.z = 0.0;
}

/// The pose those two fields describe.
///
/// # Errors
///
/// [`PoseSnapshotError::NotARotation`] for a quaternion that is not one.
pub fn read_pose(pos: &Vec3, quat: &Quat) -> Result<Isometry3<f64>, PoseSnapshotError> {
    PoseSnapshot {
        pos_x: pos.x,
        pos_y: pos.y,
        pos_z: pos.z,
        quat_w: quat.w,
        quat_x: quat.x,
        quat_y: quat.y,
        quat_z: quat.z,
    }
    .to_isometry()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::joints::JointVector;
    use brenn_reachy__motion__arm_record_clk_rs::{ArmRecordSnapWire, PinSnapWire};
    use nalgebra::{Translation3, UnitQuaternion};

    fn a_pose() -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::new(0.01, -0.02, 0.19),
            UnitQuaternion::from_euler_angles(0.05, -0.03, 0.02),
        )
    }

    fn a_record() -> ArmRecord {
        ArmRecord {
            joints: JointVector {
                body_yaw: 0.1,
                legs: [0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
                antennas: [0.8, 0.9],
            },
            head_pose_body: a_pose(),
            margins: [1.1, 1.2, 1.3, 1.4, 1.5, 1.6],
            min_margin: 1.1,
        }
    }

    /// Every number of a record is in the slot, on its own field: a written
    /// record reads back as itself, the pose included.
    #[test]
    fn a_record_written_into_a_slot_is_the_record_that_comes_out() {
        let mut slot = ArmRecordSnapWire::new();
        let state = slot.clear_valid();
        let record = a_record();
        write(state, &record);
        let read_back = read(state)
            .expect("a written pose is a rotation")
            .expect("a written record is present");
        assert_eq!(read_back.joints, record.joints);
        assert_eq!(read_back.margins, record.margins);
        assert_eq!(read_back.min_margin, record.min_margin);
        assert!(
            (read_back.head_pose_body.translation.vector
                - record.head_pose_body.translation.vector)
                .norm()
                < 1e-12
        );
        assert!(
            read_back
                .head_pose_body
                .rotation
                .angle_to(&record.head_pose_body.rotation)
                < 1e-12
        );
    }

    /// A slot nothing wrote holds no record, and a slot whose flag is set over
    /// zeroes is refused rather than read as the identity pose.
    #[test]
    fn a_slot_nothing_wrote_holds_no_record_and_a_flagged_one_is_refused() {
        let mut slot = ArmRecordSnapWire::new();
        let state = slot.clear_valid();
        assert_eq!(read(state).expect("no record is not a refusal"), None);

        state.present = true.into();
        let error = read(state).expect_err("zeroes are no rotation");
        assert!(matches!(error, PoseSnapshotError::NotARotation(_)));
    }

    /// A record cleared out of a reused slot leaves neither the flag nor the
    /// rotation of the one before it.
    #[test]
    fn clearing_a_record_leaves_no_part_of_the_one_before_it() {
        let mut slot = ArmRecordSnapWire::new();
        let state = slot.clear_valid();
        write(state, &a_record());
        clear(state);
        assert_eq!(read(state).expect("a cleared record is no refusal"), None);
        state.present = true.into();
        assert!(read(state).is_err(), "the rotation went with the flag");
    }

    /// A per-leg number lands on the field its own crank names.
    ///
    /// [`write_legs`] and [`legs_of`] list the six fields independently, so a
    /// round trip through both passes under a matched transposition. This reads
    /// the fields themselves: a margin attributed to the wrong crank is what a
    /// first move's clearance would then be measured against.
    #[test]
    fn a_per_leg_number_lands_on_the_field_its_own_crank_names() {
        let mut slot = PinSnapWire::new();
        let state = slot.clear_valid();
        write_legs(&mut state.pull_in, &[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let legs = &state.pull_in;
        assert_eq!(
            [
                legs.leg_0, legs.leg_1, legs.leg_2, legs.leg_3, legs.leg_4, legs.leg_5,
            ],
            [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        );
    }

    /// The pins carry by servo and by leg: nine goals and six pulls, each on the
    /// field its own joint names.
    #[test]
    fn the_pins_carry_every_goal_and_every_pull() {
        let mut slot = PinSnapWire::new();
        let state = slot.clear_valid();
        let pins = PinOutcome {
            pinned: JointVector {
                body_yaw: 0.1,
                legs: [0.2, 0.3, 0.4, 0.5, 0.6, 0.7],
                antennas: [0.8, 0.9],
            },
            pull_in: [0.01, 0.02, 0.03, 0.04, 0.05, 0.06],
        };
        write_pins(state, &pins);
        assert_eq!(pins_of(state), pins);
    }

    /// A pose is carried into its two fields and back exactly: the quaternion
    /// is the one that was written, never a renormalised approximation of it,
    /// because a seed picks which configuration of the mechanism a solve lands
    /// in.
    #[test]
    fn a_pose_written_into_two_fields_comes_back_bit_for_bit() {
        let pose = Isometry3::from_parts(
            Translation3::new(0.011, -0.023, 0.157),
            UnitQuaternion::from_scaled_axis(nalgebra::Vector3::new(0.31, -0.17, 0.09)),
        );
        let mut slot = ArmRecordSnapWire::new();
        let state = slot.clear_valid();
        write_pose(&mut state.head_pos, &mut state.head_quat, &pose);

        let back = read_pose(&state.head_pos, &state.head_quat).expect("a written pose is a pose");
        assert_eq!(
            back.translation.vector.as_slice(),
            pose.translation.vector.as_slice()
        );
        assert_eq!(
            back.rotation.as_ref().coords.as_slice(),
            pose.rotation.as_ref().coords.as_slice(),
            "the quaternion is carried, not renormalised"
        );
    }

    /// Zeroes are not a rotation, so neither a slot nobody wrote nor a cleared
    /// one reads as the pose that was there.
    #[test]
    fn fields_nobody_wrote_hold_no_pose() {
        let mut slot = ArmRecordSnapWire::new();
        let state = slot.clear_valid();
        assert!(
            read_pose(&state.head_pos, &state.head_quat).is_err(),
            "four zeroes are not a rotation"
        );

        write_pose(&mut state.head_pos, &mut state.head_quat, &a_pose());
        clear_pose(&mut state.head_pos, &mut state.head_quat);
        assert!(
            read_pose(&state.head_pos, &state.head_quat).is_err(),
            "a cleared pose does not read as the one that was there"
        );
    }
}
