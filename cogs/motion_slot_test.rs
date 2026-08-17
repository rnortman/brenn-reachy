//! What the slot mapping has to get right, and the one binding between the two
//! places the fault vocabulary is written down.

use core::time::Duration as StdDuration;

use brenn_reachy__cogs__msgs_clk_rs::{
    BusSourceKind, FaultKind, FaultSnap, FkFailureKind, JointFlags, JointRef, Joints, MotionMode,
    MotionSnap, PoseEstimate, Targets, TrackingSideKind, WarpKind,
};
use clockwork_rs::Duration;
use motion_slots::{
    JointSlotError, MotionSlotError, clear_pose, joint_flags, joint_ref_from_row, joint_ref_of,
    joint_set, joints_from_rows, read_fault, read_joints, read_motion_snap, read_pose,
    read_targets, row_from_joint_ref, rows_from_joints, write_fault, write_joints,
    write_motion_snap, write_pose, write_targets,
};
use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};
use reachy_motion::joints::{JointId, JointSet, JointTargets, JointVector};
use reachy_motion::snap::{
    BusSourceCode, ExcursionSnapshot, FaultCode, FaultSnapshot, FkFailureCode, ModeCode,
    MotionSnapshot, TrackingSide, TrackingStreakSnapshot, TrajectorySeed,
};
use reachy_motion::tick::{BusFailureSource, Fault, Mode, WireFailure};
use reachy_motion::traj::{MoveDurations, Warp};

/// A pose with nothing round about it, so a mapping that dropped or transposed
/// a component shows up as a different number rather than a coincidence.
fn awkward_pose() -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(0.011, -0.023, 0.157),
        UnitQuaternion::from_scaled_axis(Vector3::new(0.31, -0.17, 0.09)),
    )
}

fn awkward_joints() -> JointVector {
    JointVector {
        body_yaw: 0.101,
        legs: [0.201, 0.202, 0.203, 0.204, 0.205, 0.206],
        antennas: [0.301, -0.302],
    }
}

#[test]
fn a_pose_written_into_a_slot_comes_back_bit_for_bit() {
    let pose = awkward_pose();
    let mut fields = PoseEstimate::new();
    write_pose(&mut fields, &pose);

    let back = read_pose(&fields).expect("a pose this mapping wrote is a pose");
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

#[test]
fn a_slot_nobody_wrote_is_not_a_pose() {
    let fields = PoseEstimate::new();
    assert!(
        read_pose(&fields).is_err(),
        "four zeroes are not a rotation"
    );

    let mut fields = PoseEstimate::new();
    write_pose(&mut fields, &awkward_pose());
    clear_pose(&mut fields);
    assert!(
        read_pose(&fields).is_err(),
        "a cleared pose does not read as the one that was there"
    );
}

#[test]
fn every_joint_lands_on_its_own_named_field() {
    let joints = awkward_joints();
    let mut fields = Joints::new();
    write_joints(&mut fields, &joints);

    assert_eq!(fields.body_yaw(), joints.body_yaw);
    assert_eq!(fields.leg_0(), joints.legs[0]);
    assert_eq!(fields.leg_1(), joints.legs[1]);
    assert_eq!(fields.leg_2(), joints.legs[2]);
    assert_eq!(fields.leg_3(), joints.legs[3]);
    assert_eq!(fields.leg_4(), joints.legs[4]);
    assert_eq!(fields.leg_5(), joints.legs[5]);
    assert_eq!(fields.antenna_right(), joints.antennas[0]);
    assert_eq!(fields.antenna_left(), joints.antennas[1]);
    assert_eq!(read_joints(&fields), joints);
}

/// The seam where a wire datagram's array meets a name: every row has to arrive
/// at the joint whose bus row it is.
#[test]
fn a_row_of_angles_keeps_its_bus_order_both_ways() {
    let rows: [f64; JointId::COUNT] =
        core::array::from_fn(|row| f64::from(u8::try_from(row).expect("nine rows")) + 0.5);
    let joints = joints_from_rows(&rows);

    for id in JointId::ALL {
        let row = id.index().expect("every joint of ALL has a row");
        assert_eq!(
            joints.get(id).expect("every joint of ALL has an angle"),
            rows[row],
            "{id:?} took row {row}"
        );
    }
    assert_eq!(rows_from_joints(&joints), rows);
}

#[test]
fn a_command_set_survives_the_fields_it_is_held_in() {
    let targets = JointTargets {
        head_pose_body: awkward_pose(),
        body_yaw: -0.41,
        antennas: [1.23, -1.24],
    };
    let mut fields = Targets::new();
    write_targets(&mut fields, &targets);

    let back = read_targets(&fields).expect("a command set this mapping wrote is one");
    assert_eq!(back.body_yaw, targets.body_yaw);
    assert_eq!(back.antennas, targets.antennas);
    assert_eq!(
        back.head_pose_body.rotation.as_ref().coords.as_slice(),
        targets.head_pose_body.rotation.as_ref().coords.as_slice()
    );
    assert_eq!(
        back.head_pose_body.translation.vector.as_slice(),
        targets.head_pose_body.translation.vector.as_slice()
    );
}

/// The schema constant for each servo. Every variant of [`JointId`] is named,
/// so a tenth servo on either side fails to build; the arm under `Leg` covers
/// the leg indices no machine has, which are the ones [`JointId::index`] itself
/// answers `None` for.
///
/// Spelled out here rather than called from `motion_slots`: this is what the
/// answer should be, written a different way from how the mapping computes it,
/// and the two are asserted equal below.
fn expected_joint_ref_of(joint: JointId) -> JointRef {
    match joint {
        JointId::BodyYaw => JointRef::BODY_YAW,
        JointId::Leg(0) => JointRef::LEG_0,
        JointId::Leg(1) => JointRef::LEG_1,
        JointId::Leg(2) => JointRef::LEG_2,
        JointId::Leg(3) => JointRef::LEG_3,
        JointId::Leg(4) => JointRef::LEG_4,
        JointId::Leg(5) => JointRef::LEG_5,
        JointId::Leg(leg) => panic!("no machine here carries a seventh crank, and this is {leg}"),
        JointId::AntennaRight => JointRef::ANTENNA_RIGHT,
        JointId::AntennaLeft => JointRef::ANTENNA_LEFT,
    }
}

/// The schema flag for each servo, named the same way and for the same reason.
fn joint_flag_of(joint: JointId) -> JointFlags {
    match joint {
        JointId::BodyYaw => JointFlags::BODY_YAW,
        JointId::Leg(0) => JointFlags::LEG_0,
        JointId::Leg(1) => JointFlags::LEG_1,
        JointId::Leg(2) => JointFlags::LEG_2,
        JointId::Leg(3) => JointFlags::LEG_3,
        JointId::Leg(4) => JointFlags::LEG_4,
        JointId::Leg(5) => JointFlags::LEG_5,
        JointId::Leg(leg) => panic!("no machine here carries a seventh crank, and this is {leg}"),
        JointId::AntennaRight => JointFlags::ANTENNA_RIGHT,
        JointId::AntennaLeft => JointFlags::ANTENNA_LEFT,
    }
}

/// The joint vocabulary is written down twice as well -- once as `JointId`'s bus
/// rows, for the motion library, and once as the schema's `JointRef`, for the
/// channels -- and the two numberings are one apart, because a Clockwork default
/// carries zero and "no joint" is what a default must mean. That offset is
/// exactly the kind of thing a comment cannot hold still, so this holds it.
#[test]
fn every_servo_is_one_past_its_bus_row() {
    for joint in JointId::ALL {
        let row = joint.index().expect("every joint of ALL has a bus row");
        let row = u8::try_from(row).expect("nine rows");
        let reference = expected_joint_ref_of(joint);
        assert_eq!(
            joint_ref_of(joint),
            reference,
            "{joint:?} through the mapping the cogs call"
        );

        assert_eq!(
            u32::from(reference.0),
            u32::from(row) + 1,
            "{joint:?} sits one past its bus row"
        );
        assert_eq!(
            joint_ref_from_row(row),
            Ok(reference),
            "{joint:?} both ways"
        );
        assert_eq!(
            row_from_joint_ref(reference),
            Ok(row),
            "{joint:?} back again"
        );
        assert!(reference.is_known(), "{joint:?} is a declared value");
    }
}

/// A joint the bus has no row for names no servo, rather than naming another
/// one. [`JointId`] can spell a seventh crank and no machine here carries one,
/// so the mapping's answer for it is the same as a fault's: none.
#[test]
fn a_joint_with_no_bus_row_names_no_servo() {
    for leg in 6..=u8::MAX {
        assert_eq!(
            joint_ref_of(JointId::Leg(leg)),
            JointRef::NONE,
            "leg {leg} sits on no bus row, so it names no servo",
        );
    }
}

/// The sentinel and the default are the same value read from two sides: a fault
/// that names no joint, and a slot nobody wrote.
#[test]
fn naming_no_servo_is_a_value_and_not_a_gap() {
    assert_eq!(JointRef::NONE.0, 0, "the default carries zero");
    assert_eq!(
        joint_ref_from_row(FaultSnapshot::NO_JOINT),
        Ok(JointRef::NONE)
    );
    assert_eq!(
        row_from_joint_ref(JointRef::NONE),
        Ok(FaultSnapshot::NO_JOINT)
    );
    assert!(
        JointId::from_index(usize::from(FaultSnapshot::NO_JOINT)).is_none(),
        "the sentinel is no bus row on the library's side either"
    );
}

/// A generated enum is open: a publisher may write any number into the slot, and
/// the newtype carries it rather than refusing it. Refusing it is this
/// boundary's job, over the whole domain of the field.
#[test]
fn a_number_naming_no_servo_is_refused_at_the_boundary() {
    for value in u8::MIN..=u8::MAX {
        let declared = value <= 9;
        assert_eq!(
            JointRef(value).is_known(),
            declared,
            "{value} against the declared vocabulary"
        );
        assert_eq!(
            row_from_joint_ref(JointRef(value)).is_ok(),
            declared,
            "{value} decoded at the boundary"
        );
        if !declared {
            assert_eq!(
                row_from_joint_ref(JointRef(value)),
                Err(JointSlotError::NoSuchJointRef(value))
            );
        }

        // The other direction takes a bus row, whose domain is the nine rows
        // plus the sentinel and nothing between.
        let row = value < 9 || value == FaultSnapshot::NO_JOINT;
        assert_eq!(
            joint_ref_from_row(value).is_ok(),
            row,
            "{value} as a bus row"
        );
        if !row {
            assert_eq!(
                joint_ref_from_row(value),
                Err(JointSlotError::NoSuchRow(value))
            );
        }
    }
}

/// The flags are the joint set's own bits, not a second numbering of them: bit
/// `n` is bus row `n` on both sides, which is also the convention the wire
/// format's masks use.
#[test]
fn every_servo_flag_is_its_bus_row_bit() {
    let mut all = JointSet::EMPTY;
    let mut union = JointFlags::NONE;

    for joint in JointId::ALL {
        let row = joint.index().expect("every joint of ALL has a bus row");
        let flag = joint_flag_of(joint);
        let mut alone = JointSet::EMPTY;
        alone.insert(joint);

        assert_eq!(u32::from(flag.0), 1 << row, "{joint:?} holds bit {row}");
        assert_eq!(flag.0, alone.bits(), "{joint:?} against the library's bit");
        assert_eq!(joint_flags(alone), flag, "{joint:?} crossing over");
        assert_eq!(joint_set(flag), Ok(alone), "{joint:?} crossing back");

        all.insert(joint);
        union |= flag;
    }

    assert_eq!(
        union.0,
        all.bits(),
        "the nine together are the whole machine"
    );
    assert_eq!(JointFlags::NONE.0, 0, "the empty set carries zero");
    assert_eq!(joint_flags(JointSet::EMPTY), JointFlags::NONE);
}

/// Every value the field can hold, and what the boundary makes of it: the 512
/// sets of nine servos cross whole, and everything else is refused rather than
/// masked down to something plausible.
#[test]
fn a_set_of_servos_crosses_whole_and_nothing_else_crosses() {
    for value in u16::MIN..=u16::MAX {
        let flags = JointFlags(value);
        match joint_set(flags) {
            Ok(set) => {
                assert!(value < 512, "{value} is a set of the nine bus rows");
                assert_eq!(set.bits(), value, "carried, not repaired");
                assert_eq!(joint_flags(set), flags, "and back again");
            }
            Err(err) => {
                assert!(value >= 512, "{value} names servos this machine has");
                assert_eq!(err, JointSlotError::NoSuchJointSet(value));
            }
        }
    }
}

/// The fault vocabulary is written down twice -- once as `FaultCode`, for the
/// state slot, and once as the schema's `FaultKind`, for the channel -- and
/// nothing in either language holds the two numberings together. This does.
///
/// The match names every code with no wildcard, so a fault added to one side
/// and not the other fails to build rather than publishing fault A's number
/// under fault B's name.
#[test]
fn the_two_fault_numberings_are_one_numbering() {
    for code in FaultCode::ALL {
        let kind = match code {
            FaultCode::AntennaObstructed => FaultKind::ANTENNA_OBSTRUCTED,
            FaultCode::AntennaServoFault => FaultKind::ANTENNA_SERVO_FAULT,
            FaultCode::HeadObstructed => FaultKind::HEAD_OBSTRUCTED,
            FaultCode::HeadServoFault => FaultKind::HEAD_SERVO_FAULT,
            FaultCode::PositionFeedbackLost => FaultKind::POSITION_FEEDBACK_LOST,
            FaultCode::MeasuredPoseInvalid => FaultKind::MEASURED_POSE_INVALID,
            FaultCode::BusFailure => FaultKind::BUS_FAILURE,
            FaultCode::TorqueOffUnconfirmed => FaultKind::TORQUE_OFF_UNCONFIRMED,
        };
        assert_eq!(code.as_u8(), kind.0, "{code:?} against {kind:?}");
    }

    assert_eq!(FaultKind::NONE.0, 0, "no report is no number");
    assert!(
        FaultCode::from_u8(FaultKind::NONE.0).is_none(),
        "the value a never-written slot holds names no fault"
    );
}

/// The three non-fault outcomes share the fault channel's enum, and nothing
/// publishes them yet -- which is why their numbers are cheap to state now and
/// expensive to discover later, from a checker asserting on a number the
/// emitter never produced.
///
/// The numbers are the ones the schema compiler emits, which are the
/// declaration order rather than the `#N` tags. They are written out here as
/// literals so that a value inserted among the faults above turns this red
/// instead of quietly moving the whole family.
#[test]
fn the_abort_family_has_the_numbers_that_follow_the_faults() {
    assert_eq!(FaultKind::MOVE_ABORTED_ENVELOPE.0, 9);
    assert_eq!(FaultKind::MOVE_ABORTED_STEP.0, 10);
    assert_eq!(FaultKind::COMMAND_REJECTED.0, 11);

    // The other half of the binding: an abort is not a fault, so none of these
    // numbers names one on the state-slot side.
    for kind in [
        FaultKind::MOVE_ABORTED_ENVELOPE,
        FaultKind::MOVE_ABORTED_STEP,
        FaultKind::COMMAND_REJECTED,
    ] {
        assert!(
            FaultCode::from_u8(kind.0).is_none(),
            "{kind:?} is an outcome, not a fault",
        );
    }
}

/// The schema value for each mode, named exhaustively so a fourth on either side
/// fails to build rather than arriving as a number nobody mapped.
fn mode_of(code: ModeCode) -> MotionMode {
    match code {
        ModeCode::Holding => MotionMode::HOLDING,
        ModeCode::Moving => MotionMode::MOVING,
        ModeCode::Faulted => MotionMode::FAULTED,
    }
}

/// The schema value for each way a move spends its time.
fn warp_of(warp: Warp) -> WarpKind {
    match warp {
        Warp::MinJerk => WarpKind::MIN_JERK,
        Warp::Linear => WarpKind::LINEAR,
    }
}

/// The schema value for each side of a tracking run's anchor.
fn side_of(side: TrackingSide) -> TrackingSideKind {
    match side {
        TrackingSide::Unplaced => TrackingSideKind::UNPLACED,
        TrackingSide::Above => TrackingSideKind::ABOVE,
        TrackingSide::Below => TrackingSideKind::BELOW,
    }
}

/// The schema value for each way a pose solve fails.
fn fk_failure_of(code: FkFailureCode) -> FkFailureKind {
    match code {
        FkFailureCode::NotApplicable => FkFailureKind::NOT_APPLICABLE,
        FkFailureCode::NoConvergence => FkFailureKind::NO_CONVERGENCE,
        FkFailureCode::WrongAssemblyMode => FkFailureKind::WRONG_ASSEMBLY_MODE,
    }
}

/// The schema value for each layer that can judge the bus.
fn bus_source_of(code: BusSourceCode) -> BusSourceKind {
    match code {
        BusSourceCode::NotApplicable => BusSourceKind::NOT_APPLICABLE,
        BusSourceCode::Transaction => BusSourceKind::TRANSACTION,
        BusSourceCode::Sequence => BusSourceKind::SEQUENCE,
    }
}

/// Every vocabulary this crossing carries is written down twice -- once in the
/// motion library, once in the schema -- and the crossing assumes the two agree
/// number for number. Nothing in either file can say so, so this does: a value
/// renumbered on either side fails here rather than becoming a state restored as
/// the wrong one.
#[test]
fn the_slot_vocabularies_carry_the_librarys_numbers() {
    for code in ModeCode::ALL {
        assert_eq!(mode_of(code).0, code.as_u8(), "{code:?}");
        assert_eq!(ModeCode::from_u8(mode_of(code).0), Some(code), "{code:?}");
    }
    for warp in Warp::ALL {
        assert_eq!(warp_of(warp).0, warp.as_u8(), "{warp:?}");
    }
    for side in TrackingSide::ALL {
        assert_eq!(side_of(side).0, side.as_i8(), "{side:?}");
    }
    for code in FkFailureCode::ALL {
        assert_eq!(fk_failure_of(code).0, code.as_u8(), "{code:?}");
    }
    for code in BusSourceCode::ALL {
        assert_eq!(bus_source_of(code).0, code.as_u8(), "{code:?}");
    }

    // The one number an unwritten slot holds, checked against the one mode it
    // must not name: a machine that has never ticked is not a machine holding.
    assert_eq!(MotionMode::NONE.0, 0);
    assert!(ModeCode::from_u8(MotionMode::NONE.0).is_none());
}

/// A tick state with nothing round about it: a running move, an open tracking
/// run on one joint, a mask, and an excursion baseline of nine distinct numbers.
/// A crossing that dropped or transposed a field shows up as a different number
/// rather than as a coincidence of zeroes.
fn awkward_snapshot() -> MotionSnapshot {
    let mut tracking = [None; JointId::COUNT];
    tracking[3] = Some(TrackingStreakSnapshot {
        anchor: -0.337,
        side: TrackingSide::Below,
        count: 12,
    });
    tracking[8] = Some(TrackingStreakSnapshot {
        anchor: 0.771,
        side: TrackingSide::Above,
        count: 3,
    });
    MotionSnapshot {
        mode: Mode::Moving {
            elapsed: StdDuration::from_nanos(123_456_789),
        },
        trajectory: Some(TrajectorySeed {
            start: awkward_targets(0.0),
            target: awkward_targets(0.05),
            durations: MoveDurations {
                head: StdDuration::from_millis(800),
                antennas: [StdDuration::from_millis(650), StdDuration::from_millis(700)],
            },
            warp: Warp::Linear,
        }),
        prev_now: Some(StdDuration::from_nanos(9_876_543_210)),
        last_goal: awkward_joints(),
        last_targets: awkward_targets(-0.02),
        fk_seed: awkward_pose(),
        present_min_margin: 0.0417,
        start_excursion: ExcursionSnapshot {
            window: [0.11, 0.12, 0.13, 0.14, 0.15, 0.16],
            body_yaw: 0.17,
            relative_yaw: 0.18,
            cone: 0.19,
        },
        miss_count: 4,
        pose_failures: 6,
        tracking,
        masked: out_of_service(),
    }
}

/// Two joints out of service, which is a set with holes in it rather than one
/// contiguous run.
fn out_of_service() -> JointSet {
    let mut masked = JointSet::EMPTY;
    masked.insert(JointId::AntennaRight);
    masked.insert(JointId::Leg(2));
    masked
}

/// A command set offset by `nudge`, so two of them are never the same numbers.
fn awkward_targets(nudge: f64) -> JointTargets {
    JointTargets {
        head_pose_body: Isometry3::from_parts(
            Translation3::new(0.007 + nudge, -0.019, 0.161 + nudge),
            UnitQuaternion::from_scaled_axis(Vector3::new(0.21, -0.13 + nudge, 0.05)),
        ),
        body_yaw: -0.31 + nudge,
        antennas: [1.11 + nudge, -1.12],
    }
}

/// The whole of the tick's state through the slot and back. This is the law the
/// cog rests on: a `mover` execution restores from these fields, ticks, and
/// writes them again, so a field that does not survive is an accumulator reset
/// every cycle -- which is to say a detector that can never fire.
#[test]
fn the_whole_of_a_tick_state_survives_the_slot() {
    let snap = awkward_snapshot();
    let mut slot = MotionSnap::new();
    write_motion_snap(&mut slot, &snap).expect("a state the tick reached crosses");

    let back = read_motion_snap(&slot).expect("what this crossing wrote, it reads");
    assert_eq!(back, snap);
}

/// The two modes that carry a payload beside them, and the one that does not.
#[test]
fn every_mode_crosses_with_whatever_travels_beside_it() {
    let holding = MotionSnapshot {
        mode: Mode::Holding,
        trajectory: None,
        prev_now: None,
        ..awkward_snapshot()
    };
    let faulted = MotionSnapshot {
        mode: Mode::Faulted(Fault::HeadObstructed {
            joint: JointId::Leg(4),
            error: 0.238,
        }),
        ..awkward_snapshot()
    };

    for snap in [holding, awkward_snapshot(), faulted] {
        let mut slot = MotionSnap::new();
        write_motion_snap(&mut slot, &snap).expect("a state the tick reached crosses");
        assert_eq!(
            read_motion_snap(&slot).expect("what this crossing wrote, it reads"),
            snap,
            "{:?}",
            snap.mode
        );
    }
}

/// A latching fault stops the tick commanding without clearing the path it was
/// on, so faulted-with-a-trajectory is a state the tick reaches and the slot has
/// to hold both at once.
#[test]
fn a_path_left_by_a_fault_crosses_with_the_fault() {
    let snap = MotionSnapshot {
        mode: Mode::Faulted(Fault::PositionFeedbackLost { misses: 51 }),
        ..awkward_snapshot()
    };
    let mut slot = MotionSnap::new();
    write_motion_snap(&mut slot, &snap).expect("a state the tick reached crosses");

    assert!(slot.trajectory().present(), "the path is still in the slot");
    assert_eq!(
        read_motion_snap(&slot).expect("what this crossing wrote, it reads"),
        snap
    );
}

/// A slot's fields outlive the state that wrote them, so a mode that says there
/// is no fault must not sit beside the evidence of the last one.
#[test]
fn a_state_that_stopped_faulting_leaves_no_evidence_behind() {
    let mut slot = MotionSnap::new();
    write_motion_snap(
        &mut slot,
        &MotionSnapshot {
            mode: Mode::Faulted(Fault::TorqueOffUnconfirmed { id: 42 }),
            ..awkward_snapshot()
        },
    )
    .expect("a state the tick reached crosses");
    assert_eq!(slot.fault().servo_id(), 42);

    write_motion_snap(
        &mut slot,
        &MotionSnapshot {
            mode: Mode::Holding,
            trajectory: None,
            ..awkward_snapshot()
        },
    )
    .expect("a state the tick reached crosses");
    assert_eq!(slot.fault().servo_id(), 0, "the servo is not still named");
    assert_eq!(slot.fault().code(), FaultKind::NONE);
    assert!(
        !slot.trajectory().present(),
        "the path of the move that ended is not still flagged"
    );
}

/// Where a run is open matters as much as that one is: the state is per joint,
/// and a crossing that shifted the array would put one joint's evidence on
/// another's name.
#[test]
fn a_tracking_run_stays_on_the_joint_it_is_about() {
    let snap = awkward_snapshot();
    let mut slot = MotionSnap::new();
    write_motion_snap(&mut slot, &snap).expect("a state the tick reached crosses");

    for (row, entry) in slot.tracking().iter().enumerate() {
        assert_eq!(
            entry.active(),
            snap.tracking[row].is_some(),
            "row {row} agrees about whether a run is open"
        );
    }
    assert_eq!(slot.tracking()[3].count(), 12);
    assert_eq!(slot.tracking()[3].side(), TrackingSideKind::BELOW);
    assert_eq!(slot.tracking()[8].side(), TrackingSideKind::ABOVE);
}

/// The case the crossing exists to refuse: a cog's first execution, where the
/// slot is memory nobody has written and every number in it is zero.
#[test]
fn a_slot_nobody_wrote_is_no_state_at_all() {
    let slot = MotionSnap::new();
    assert_eq!(
        read_motion_snap(&slot),
        Err(MotionSlotError::NoSuchMode(0)),
        "zero names no mode, and is refused before anything else is read"
    );
}

/// Every other way a slot's numbers can fail to be a state. Each is reached by
/// writing a state that is one and then damaging the one field under test, so a
/// refusal that stopped working shows up as a state restored from numbers that
/// do not describe one.
#[test]
fn numbers_that_describe_no_state_are_refused_one_by_one() {
    let good = awkward_snapshot();

    let mut slot = MotionSnap::new();
    write_motion_snap(&mut slot, &good).expect("a state the tick reached crosses");
    slot.trajectory_mut().set_warp(WarpKind(7));
    assert_eq!(read_motion_snap(&slot), Err(MotionSlotError::NoSuchWarp(7)));

    let mut slot = MotionSnap::new();
    write_motion_snap(&mut slot, &good).expect("a state the tick reached crosses");
    slot.tracking_mut()[3].set_side(TrackingSideKind(-3));
    assert_eq!(
        read_motion_snap(&slot),
        Err(MotionSlotError::NoSuchTrackingSide(-3))
    );

    let mut slot = MotionSnap::new();
    write_motion_snap(&mut slot, &good).expect("a state the tick reached crosses");
    slot.fk_seed_quat_mut().set_w(0.5);
    assert!(
        matches!(read_motion_snap(&slot), Err(MotionSlotError::Pose(_))),
        "a quaternion of the wrong length is not the seed of anything"
    );

    let mut slot = MotionSnap::new();
    write_motion_snap(&mut slot, &good).expect("a state the tick reached crosses");
    slot.set_masked(JointFlags(1 << 9));
    assert_eq!(
        read_motion_snap(&slot),
        Err(MotionSlotError::Joint(JointSlotError::NoSuchJointSet(
            1 << 9
        )))
    );

    let mut slot = MotionSnap::new();
    write_motion_snap(&mut slot, &good).expect("a state the tick reached crosses");
    slot.set_prev_now(Duration::from_nanos(-1));
    assert!(matches!(
        read_motion_snap(&slot),
        Err(MotionSlotError::Duration(_))
    ));

    // The three refusals only a standing fault reaches: the fault fields are
    // read at all only when the mode says the tick is parked on one.
    let parked = MotionSnapshot {
        mode: Mode::Faulted(Fault::AntennaObstructed {
            joint: JointId::AntennaLeft,
            error: 0.4,
        }),
        ..good
    };

    let mut slot = MotionSnap::new();
    write_motion_snap(&mut slot, &parked).expect("a state the tick reached crosses");
    slot.fault_mut().set_fk_code(FkFailureKind(9));
    assert_eq!(
        read_motion_snap(&slot),
        Err(MotionSlotError::NoSuchFkFailure(9))
    );

    let mut slot = MotionSnap::new();
    write_motion_snap(&mut slot, &parked).expect("a state the tick reached crosses");
    slot.fault_mut().set_bus_source(BusSourceKind(9));
    assert_eq!(
        read_motion_snap(&slot),
        Err(MotionSlotError::NoSuchBusSource(9))
    );

    // Every field a number this build knows, and evidence that does not suit
    // the code: an obstruction is raised against a servo, so one naming none
    // has lost the thing it was about. The crossing puts that question to the
    // crate that owns the fault vocabulary rather than answering it itself.
    let mut slot = MotionSnap::new();
    write_motion_snap(&mut slot, &parked).expect("a state the tick reached crosses");
    slot.fault_mut().set_joint(JointRef::NONE);
    assert!(
        matches!(read_motion_snap(&slot), Err(MotionSlotError::Fault(_))),
        "a machine cannot stand parked on a fault it cannot describe"
    );
}

/// The write direction refuses a length of time the slot cannot hold, and
/// refuses it whole. A move nobody could sit through must not be shortened into
/// a plausible one on the way out, and the refusal must not leave the slot
/// holding half of one state and half of another.
#[test]
fn a_length_of_time_the_slot_cannot_hold_is_refused_on_the_way_out() {
    // Past what a signed nanosecond count reaches, which is around 292 years.
    let unreachable = StdDuration::from_secs(1 << 40);

    let mut slot = MotionSnap::new();
    write_motion_snap(&mut slot, &awkward_snapshot()).expect("a state the tick reached crosses");
    let mut written = MotionSnap::new();
    write_motion_snap(&mut written, &awkward_snapshot()).expect("the same state again");

    let elapsed = MotionSnapshot {
        mode: Mode::Moving {
            elapsed: unreachable,
        },
        ..awkward_snapshot()
    };
    assert!(matches!(
        write_motion_snap(&mut slot, &elapsed),
        Err(MotionSlotError::Duration(_))
    ));
    assert_eq!(slot, written, "the slot still holds the state it had");

    let durations = MotionSnapshot {
        trajectory: Some(TrajectorySeed {
            durations: MoveDurations {
                head: unreachable,
                antennas: [StdDuration::from_millis(650), StdDuration::from_millis(700)],
            },
            ..awkward_snapshot()
                .trajectory
                .expect("the fixture is moving")
        }),
        ..awkward_snapshot()
    };
    assert!(matches!(
        write_motion_snap(&mut slot, &durations),
        Err(MotionSlotError::Duration(_))
    ));
    assert_eq!(slot, written, "and still after the second refusal");
}

/// An abort is reported on the same channel as a fault and is not one: nothing
/// stands parked on a refused command, so the three outcome numbers name no
/// standing fault when they turn up in this field.
#[test]
fn an_outcome_is_not_a_fault_a_machine_can_stand_parked_on() {
    let mut slot = MotionSnap::new();
    write_motion_snap(
        &mut slot,
        &MotionSnapshot {
            mode: Mode::Faulted(Fault::TorqueOffUnconfirmed { id: 9 }),
            ..awkward_snapshot()
        },
    )
    .expect("a state the tick reached crosses");

    for kind in [
        FaultKind::NONE,
        FaultKind::MOVE_ABORTED_ENVELOPE,
        FaultKind::MOVE_ABORTED_STEP,
        FaultKind::COMMAND_REJECTED,
    ] {
        slot.fault_mut().set_code(kind);
        assert_eq!(
            read_motion_snap(&slot),
            Err(MotionSlotError::NoSuchFaultCode(kind.0)),
            "{kind:?}"
        );
    }
}

/// A fault's own numbers, through the fields alone. The whole-state tests above
/// carry one fault each; this one carries every kind of evidence a fault has,
/// which is what the fault fields exist to hold.
#[test]
fn a_faults_evidence_crosses_field_for_field() {
    let fault = FaultSnapshot::from(&Fault::MeasuredPoseInvalid {
        failures: 7,
        source: reachy_kin::FkError::WrongAssemblyMode {
            cone_deg: 41.5,
            z: -0.083,
        },
    });
    let mut slot = FaultSnap::new();
    write_fault(&mut slot, &fault).expect("a fault the tick raised names a servo or none");

    assert_eq!(slot.code(), FaultKind::MEASURED_POSE_INVALID);
    assert_eq!(slot.joint(), JointRef::NONE);
    assert_eq!(slot.count(), 7);
    assert_eq!(slot.fk_code(), FkFailureKind::WRONG_ASSEMBLY_MODE);
    assert_eq!(slot.fk_a(), 41.5);
    assert_eq!(slot.fk_b(), -0.083);
    assert_eq!(read_fault(&slot), Ok(fault));

    let bus = FaultSnapshot::from(&Fault::BusFailure {
        source: BusFailureSource::Transaction {
            id: 12,
            kind: WireFailure::NotWritten,
        },
    });
    let mut slot = FaultSnap::new();
    write_fault(&mut slot, &bus).expect("a bus fault names a servo or none");
    assert_eq!(slot.bus_source(), BusSourceKind::TRANSACTION);
    assert_eq!(slot.servo_id(), 12);
    assert_eq!(slot.bus_failure_kind(), WireFailure::NotWritten.as_u8());
    assert_eq!(read_fault(&slot), Ok(bus));
}

/// The write direction refuses a servo it cannot name, rather than writing a
/// report about no servo in particular. An operator reading a fault that names
/// no joint would take it for one about the whole machine, and nothing would
/// say a joint had been named and lost.
#[test]
fn a_fault_naming_a_row_no_servo_has_is_refused_on_the_way_out() {
    let mut fault = FaultSnapshot::from(&Fault::AntennaObstructed {
        joint: JointId::AntennaLeft,
        error: 0.4,
    });
    let mut slot = FaultSnap::new();
    write_fault(&mut slot, &fault).expect("a servo on the bus crosses");
    assert_eq!(slot.joint(), JointRef::ANTENNA_LEFT);

    for row in [9_u8, 10, 200, 254] {
        fault.joint = row;
        let mut slot = FaultSnap::new();
        assert_eq!(
            write_fault(&mut slot, &fault),
            Err(MotionSlotError::Joint(JointSlotError::NoSuchRow(row))),
        );
        assert_eq!(
            slot,
            FaultSnap::new(),
            "and no field of it was written: a slot carrying a code beside no \
             joint reads as a report about the whole machine",
        );
    }

    fault.joint = FaultSnapshot::NO_JOINT;
    let mut slot = FaultSnap::new();
    write_fault(&mut slot, &fault).expect("a fault that names no servo crosses as none");
    assert_eq!(slot.joint(), JointRef::NONE);
}
