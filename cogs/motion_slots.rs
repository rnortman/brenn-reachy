//! The one mapping between this repo's motion types and the schema types the
//! cogs' slots and messages are made of.
//!
//! A Clockwork message is generated code with a named setter per field, and a
//! `reachy-motion` value is a struct of arrays and nalgebra types: somewhere a
//! line has to say which field holds which number. `reachy-motion` says so once
//! already, in its own flat forms — [`PoseSnapshot`], [`JointsSnapshot`],
//! [`TargetsSnapshot`] — which exist precisely so the mapping is not written
//! again per host, and which carry the crate's refusal doctrine with them: a
//! quaternion that is not a rotation is refused rather than repaired.
//!
//! So this module is where a cog's fields meet those forms, and every cog in
//! this system goes through it. The alternative is a copy per cog, each free to
//! drift in which row it calls `leg_3` and in what it does with a slot holding
//! numbers nothing wrote.
//!
//! Nothing here holds state or allocates, and none of it looks at a clock.
//!
//! The [`counters!`] macro at the foot is the same argument about a different
//! part of a slot: every cog keeps its run's totals in state fields and reports
//! them on signals of the same names, and a cog writing that bookkeeping out by
//! hand is a cog where a counter can be added to the struct and forgotten in the
//! change guard.

use brenn_reachy__cogs__msgs_clk_rs::{
    BusSourceKind, FaultKind, FaultSnap, FkFailureKind, JointFlags, JointRef, Joints, MotionMode,
    MotionSnap, PoseEstimate, PoseState, Quat, Targets, TrackingSideKind, TrackingStreakSnap,
    TrajectorySeed as TrajectorySeedSlot, Vec3, WarpKind,
};
use clockwork_rs::Duration;
use nalgebra::Isometry3;
use reachy_motion::joints::{JointId, JointSet, JointTargets, JointVector};
use reachy_motion::snap::{
    BusSourceCode, DurationError, DurationsSnapshot, ExcursionSnapshot, FaultCode, FaultSnapshot,
    FaultSnapshotError, FkFailureCode, JointsSnapshot, ModeCode, MotionSnapshot, PoseSnapshot,
    PoseSnapshotError, TargetsSnapshot, TrackingSide, TrackingStreakSnapshot, TrajectorySeed,
    duration_from_nanos, duration_nanos,
};
use reachy_motion::tick::Mode;
use reachy_motion::traj::Warp;
use thiserror::Error;

/// Why a slot's joint vocabulary names no joint.
///
/// A generated schema enum is open — a value the schema does not declare is
/// carried rather than refused, because a publisher can write any bit pattern
/// into a shared slot. Refusing it is therefore this boundary's job, and these
/// are its three refusals.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum JointSlotError {
    /// A bus row that is no bus row, and is not the one value that names no
    /// joint.
    #[error("{0} is not a bus row, and is not how a fault says it names no joint")]
    NoSuchRow(u8),
    /// A joint reference this build does not know: past the ninth servo, or a
    /// number the vocabulary skips.
    #[error("{0} names no servo, and is not the value that names none")]
    NoSuchJointRef(u8),
    /// A set with a bit above the ninth bus row set. Refused rather than masked
    /// off: the bit means something to whoever wrote it, and this build does not
    /// know what.
    #[error("{0:#x} is not a set of servos -- it has bits above the ninth bus row")]
    NoSuchJointSet(u16),
}

/// The joint a fault's bus row names.
///
/// The two vocabularies are one apart: [`FaultSnapshot`] numbers the nine servos
/// from zero and spells "no joint" [`FaultSnapshot::NO_JOINT`], while the schema
/// puts `none` at zero — a Clockwork default must be zero, and a report about
/// the whole machine must not decode as one about the body yaw. This function
/// and [`row_from_joint_ref`] are the only two places that offset is applied.
///
/// # Errors
///
/// [`JointSlotError::NoSuchRow`] for a row past the ninth servo that is not the
/// sentinel.
pub fn joint_ref_from_row(row: u8) -> Result<JointRef, JointSlotError> {
    if row == FaultSnapshot::NO_JOINT {
        return Ok(JointRef::NONE);
    }
    JointId::from_index(usize::from(row))
        .and_then(|_| row.checked_add(1))
        .map(JointRef)
        .ok_or(JointSlotError::NoSuchRow(row))
}

/// The bus row a joint reference names, in the numbering
/// [`FaultSnapshot::joint`] uses.
///
/// The inverse of [`joint_ref_from_row`], sentinel included.
///
/// # Errors
///
/// [`JointSlotError::NoSuchJointRef`] for a value this build's vocabulary does
/// not declare.
pub fn row_from_joint_ref(joint: JointRef) -> Result<u8, JointSlotError> {
    if joint == JointRef::NONE {
        return Ok(FaultSnapshot::NO_JOINT);
    }
    joint
        .0
        .checked_sub(1)
        .filter(|row| JointId::from_index(usize::from(*row)).is_some())
        .ok_or(JointSlotError::NoSuchJointRef(joint.0))
}

/// The vocabulary's name for one servo.
///
/// [`JointRef::NONE`] for a joint the bus has no row for — a leg index past the
/// sixth, which [`JointId`] can spell and no machine carries. That is the same
/// answer [`FaultSnapshot`] gives such a joint, so a report about a servo that
/// does not exist names no servo here either, rather than naming another one.
#[must_use]
pub fn joint_ref_of(joint: JointId) -> JointRef {
    joint
        .index()
        .and_then(|row| u8::try_from(row).ok())
        .and_then(|row| joint_ref_from_row(row).ok())
        .unwrap_or(JointRef::NONE)
}

/// A set of joints as the schema's flags.
///
/// One bit per bus row in both, so this is the set's own number and nothing
/// derived from it.
#[must_use]
pub fn joint_flags(set: JointSet) -> JointFlags {
    JointFlags(set.bits())
}

/// The set of joints those flags name.
///
/// Not [`JointFlags::is_known`]: that answers for the declared bits, so a
/// genuine combination of two joints answers false. The membership question a
/// set actually has is whether every bit set is a bus row, which is what
/// [`JointSet::from_bits`] asks.
///
/// # Errors
///
/// [`JointSlotError::NoSuchJointSet`] for a value with a bit above the ninth bus
/// row.
pub fn joint_set(flags: JointFlags) -> Result<JointSet, JointSlotError> {
    JointSet::from_bits(flags.0).ok_or(JointSlotError::NoSuchJointSet(flags.0))
}

/// A schema type that holds a rigid pose, in whatever it calls its two fields.
///
/// A pose is two fields of one message, and a caller cannot borrow both at once;
/// the pose functions therefore take the message and ask it for one at a time.
/// Implementing this is the only place a schema's own spelling of "where the
/// head is" appears.
pub trait PoseRead {
    /// The translation field.
    fn pos(&self) -> &Vec3;
    /// The rotation field.
    fn quat(&self) -> &Quat;
}

/// The same, for a schema type a caller may write.
///
/// Split from [`PoseRead`] because a message can hold more than one pose: a view
/// naming one of them has a shared borrow of the message and nothing to write
/// through, and a single trait would have forced such a view to carry two
/// methods it can never answer.
pub trait PoseFields: PoseRead {
    /// The translation field, to write.
    fn pos_mut(&mut self) -> &mut Vec3;
    /// The rotation field, to write.
    fn quat_mut(&mut self) -> &mut Quat;
}

macro_rules! pose_fields {
    ($type:ty, $pos:ident, $quat:ident, $pos_mut:ident, $quat_mut:ident) => {
        impl PoseRead for $type {
            fn pos(&self) -> &Vec3 {
                self.$pos()
            }

            fn quat(&self) -> &Quat {
                self.$quat()
            }
        }

        impl PoseFields for $type {
            fn pos_mut(&mut self) -> &mut Vec3 {
                self.$pos_mut()
            }

            fn quat_mut(&mut self) -> &mut Quat {
                self.$quat_mut()
            }
        }
    };
}

pose_fields!(
    PoseEstimate,
    head_pos,
    head_quat,
    head_pos_mut,
    head_quat_mut
);
pose_fields!(PoseState, seed_pos, seed_quat, seed_pos_mut, seed_quat_mut);
pose_fields!(Targets, head_pos, head_quat, head_pos_mut, head_quat_mut);

/// Write a rigid pose into the two fields a schema holds one in.
pub fn write_pose<T: PoseFields + ?Sized>(out: &mut T, pose: &Isometry3<f64>) {
    write_flat_pose(out, &PoseSnapshot::from(pose));
}

fn write_flat_pose<T: PoseFields + ?Sized>(out: &mut T, flat: &PoseSnapshot) {
    let pos = out.pos_mut();
    pos.set_x(flat.pos_x);
    pos.set_y(flat.pos_y);
    pos.set_z(flat.pos_z);
    let quat = out.quat_mut();
    quat.set_w(flat.quat_w);
    quat.set_x(flat.quat_x);
    quat.set_y(flat.quat_y);
    quat.set_z(flat.quat_z);
}

/// The pose those two fields describe.
///
/// # Errors
///
/// [`PoseSnapshotError::NotARotation`] for a quaternion that is not one, which
/// is what a slot nothing wrote holds. The quaternion is taken as written rather
/// than normalised: normalising would turn a slot that holds no rotation into a
/// plausible one, and it is the seed of a solve that picks which configuration
/// of the mechanism the answer lands in.
pub fn read_pose<T: PoseRead + ?Sized>(fields: &T) -> Result<Isometry3<f64>, PoseSnapshotError> {
    let (pos, quat) = (fields.pos(), fields.quat());
    PoseSnapshot {
        pos_x: pos.x(),
        pos_y: pos.y(),
        pos_z: pos.z(),
        quat_w: quat.w(),
        quat_x: quat.x(),
        quat_y: quat.y(),
        quat_z: quat.z(),
    }
    .to_isometry()
}

/// Leave the two fields holding no pose at all.
///
/// An output slot is reused memory, so a message that carries no pose has to say
/// so in the fields as well as in whatever flag names them meaningless. Zeroes
/// are not a pose: the quaternion they make is refused by [`read_pose`].
pub fn clear_pose<T: PoseFields + ?Sized>(out: &mut T) {
    *out.pos_mut() = Vec3::new();
    *out.quat_mut() = Quat::new();
}

/// Write nine angles into the named joint fields.
pub fn write_joints(out: &mut Joints, joints: &JointVector) {
    let flat = JointsSnapshot::from(joints);
    out.set_body_yaw(flat.body_yaw);
    out.set_leg_0(flat.leg_0);
    out.set_leg_1(flat.leg_1);
    out.set_leg_2(flat.leg_2);
    out.set_leg_3(flat.leg_3);
    out.set_leg_4(flat.leg_4);
    out.set_leg_5(flat.leg_5);
    out.set_antenna_right(flat.antenna_right);
    out.set_antenna_left(flat.antenna_left);
}

/// The nine angles those fields hold.
#[must_use]
pub fn read_joints(joints: &Joints) -> JointVector {
    JointsSnapshot {
        body_yaw: joints.body_yaw(),
        leg_0: joints.leg_0(),
        leg_1: joints.leg_1(),
        leg_2: joints.leg_2(),
        leg_3: joints.leg_3(),
        leg_4: joints.leg_4(),
        leg_5: joints.leg_5(),
        antenna_right: joints.antenna_right(),
        antenna_left: joints.antenna_left(),
    }
    .to_vector()
}

/// A wire datagram's row of angles as the vector the motion library speaks.
///
/// The rows and the joints are matched through [`JointId::index`] rather than by
/// writing the array indices out, so neither this function nor its inverse can
/// put a servo's angle on another servo's name.
#[must_use]
pub fn joints_from_rows(rows: &[f64; JointId::COUNT]) -> JointVector {
    let mut joints = JointVector::default();
    for id in JointId::ALL {
        if let Some(row) = id.index()
            && let Some(angle) = rows.get(row)
        {
            joints.set(id, *angle);
        }
    }
    joints
}

/// The inverse of [`joints_from_rows`]: nine angles in bus-row order.
#[must_use]
pub fn rows_from_joints(joints: &JointVector) -> [f64; JointId::COUNT] {
    let mut rows = [0.0; JointId::COUNT];
    for (id, angle) in joints.joints() {
        if let Some(row) = id.index()
            && let Some(slot) = rows.get_mut(row)
        {
            *slot = angle;
        }
    }
    rows
}

/// Write a command set into the fields a schema holds one in.
pub fn write_targets(out: &mut Targets, targets: &JointTargets) {
    let flat = TargetsSnapshot::from(targets);
    write_flat_pose(out, &flat.head_pose);
    out.set_body_yaw(flat.body_yaw);
    out.set_antenna_right(flat.antenna_right);
    out.set_antenna_left(flat.antenna_left);
}

/// The command set those fields describe.
///
/// # Errors
///
/// [`PoseSnapshotError::NotARotation`], for a head pose that is not one.
pub fn read_targets(targets: &Targets) -> Result<JointTargets, PoseSnapshotError> {
    Ok(JointTargets {
        head_pose_body: read_pose(targets)?,
        body_yaw: targets.body_yaw(),
        antennas: [targets.antenna_right(), targets.antenna_left()],
    })
}

/// Why a slot's numbers describe no state the motion tick could be in.
///
/// Every variant here is a refusal rather than a repair. The state this crossing
/// carries is the fault detectors themselves — the tracking runs, the miss
/// count, the step-bound baseline — and a field read wrongly is not a cosmetic
/// loss: it is a machine that has forgotten how far it is allowed to move next.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum MotionSlotError {
    /// A duration a slot's count cannot hold, or a count that is not one.
    #[error("a length of time in the slot is not one: {0}")]
    Duration(#[from] DurationError),
    /// A pose field holding numbers that are not a pose.
    #[error("a pose in the slot is not one: {0}")]
    Pose(#[from] PoseSnapshotError),
    /// A fault whose numbers name no fault.
    #[error("the standing fault in the slot is not one: {0}")]
    Fault(#[from] FaultSnapshotError),
    /// A set of joints, or a joint reference, this build does not know.
    #[error("a joint in the slot is not one: {0}")]
    Joint(#[from] JointSlotError),
    /// A mode number this build does not know, zero included — which is what a
    /// slot nothing wrote holds.
    #[error("{0} names no mode the tick is ever in")]
    NoSuchMode(u8),
    /// A standing fault named by a number that is no standing fault: zero, or
    /// one of the three non-fault outcomes the tick reports on the same
    /// channel. A machine does not stand parked on a refused command.
    #[error("{0} names no fault a machine can stand parked on")]
    NoSuchFaultCode(u8),
    /// A solve-failure number this build does not know.
    #[error("{0} names no way a pose solve has of failing")]
    NoSuchFkFailure(u8),
    /// A bus-failure source this build does not know.
    #[error("{0} names no layer that could have judged the bus")]
    NoSuchBusSource(u8),
    /// A warp number this build does not know.
    #[error("{0} names no way for a move to spend its time")]
    NoSuchWarp(u8),
    /// A tracking-run side this build does not know.
    #[error("{0} names no side of a tracking run's anchor")]
    NoSuchTrackingSide(i8),
}

/// Write a standing fault into the fields a slot holds one in.
///
/// The numbering is `reachy-motion`'s own: the schema's fault vocabulary and
/// [`FaultCode`] carry the same numbers for the same eight slugs, which is a
/// fact a test pins rather than one this line may assume quietly.
///
/// # Errors
///
/// [`JointSlotError::NoSuchJointRef`] for a fault naming a row no servo has.
/// Refused rather than written as a fault about no servo in particular: a
/// report that lost the servo it was raised against reads to an operator as a
/// report about the whole machine. On a refusal `out` is left exactly as it
/// was — a caller that carries on past the error is not left holding a slot
/// half-way between two reports.
pub fn write_fault(out: &mut FaultSnap, fault: &FaultSnapshot) -> Result<(), MotionSlotError> {
    // Before the first write, so a refusal costs the slot nothing.
    let joint = joint_ref_from_row(fault.joint)?;
    out.set_code(FaultKind(fault.code.as_u8()));
    out.set_joint(joint);
    out.set_servo_id(fault.servo_id);
    out.set_error_bits(fault.error_bits);
    out.set_count(fault.count);
    out.set_error(fault.error);
    out.set_fk_code(FkFailureKind(fault.fk_code.as_u8()));
    out.set_fk_a(fault.fk_a);
    out.set_fk_b(fault.fk_b);
    out.set_bus_source(BusSourceKind(fault.bus_source.as_u8()));
    out.set_bus_failure_kind(fault.bus_failure_kind);
    Ok(())
}

/// The standing fault those fields describe.
///
/// # Errors
///
/// [`MotionSlotError`] for a code, a solve failure, a bus source or a joint this
/// build does not know. Note what is *not* checked here: whether the evidence
/// fields suit the code. That question belongs to
/// [`FaultSnapshot::to_fault`](reachy_motion::FaultSnapshot::to_fault), which
/// asks it once for every host.
pub fn read_fault(slot: &FaultSnap) -> Result<FaultSnapshot, MotionSlotError> {
    Ok(FaultSnapshot {
        code: FaultCode::from_u8(slot.code().0)
            .ok_or(MotionSlotError::NoSuchFaultCode(slot.code().0))?,
        joint: row_from_joint_ref(slot.joint())?,
        servo_id: slot.servo_id(),
        error_bits: slot.error_bits(),
        count: slot.count(),
        error: slot.error(),
        fk_code: FkFailureCode::from_u8(slot.fk_code().0)
            .ok_or(MotionSlotError::NoSuchFkFailure(slot.fk_code().0))?,
        fk_a: slot.fk_a(),
        fk_b: slot.fk_b(),
        bus_source: BusSourceCode::from_u8(slot.bus_source().0)
            .ok_or(MotionSlotError::NoSuchBusSource(slot.bus_source().0))?,
        bus_failure_kind: slot.bus_failure_kind(),
    })
}

/// Write the whole of the motion tick's state into the slot that holds it.
///
/// Total but for the two lengths of time a move is clocked on and the tick's own
/// clock, which reach further as a [`core::time::Duration`] than as the whole
/// nanoseconds a field holds. A move nobody could sit through is refused on the
/// way out rather than shortened on the way in.
///
/// # Errors
///
/// [`MotionSlotError::Duration`] for a length of time past what the slot's
/// counts reach, and [`JointSlotError::NoSuchJointRef`] for a standing fault
/// naming a row no servo has. On a refusal `out` is left exactly as it was:
/// every part of the state that can be refused is built aside first, so a
/// caller never ends up with a slot holding half of one state and half of
/// another — a faulted mode beside no fault would read as a report about the
/// whole machine.
pub fn write_motion_snap(
    out: &mut MotionSnap,
    snap: &MotionSnapshot,
) -> Result<(), MotionSlotError> {
    let MotionSnapshot {
        mode,
        trajectory,
        prev_now,
        last_goal,
        last_targets,
        fk_seed,
        present_min_margin,
        start_excursion,
        miss_count,
        pose_failures,
        tracking,
        masked,
    } = snap;

    // The four fallible parts, worked out before the slot is touched.
    let moving_elapsed = match mode {
        Mode::Moving { elapsed } => duration_nanos(*elapsed)?,
        Mode::Holding | Mode::Faulted(_) => 0,
    };
    // A blank rather than whatever the last fault left: a slot's fields outlive
    // the state that wrote them, and stale evidence beside a mode that says
    // there is no fault is the one shape of this slot a reader could mistake
    // for a report.
    let mut fault_out = FaultSnap::new();
    if let Mode::Faulted(fault) = mode {
        write_fault(&mut fault_out, &FaultSnapshot::from(fault))?;
    }
    let mut trajectory_out = TrajectorySeedSlot::new();
    write_trajectory(&mut trajectory_out, trajectory.as_ref())?;
    let prev_now_ns = prev_now.map(duration_nanos).transpose()?.unwrap_or(0);

    out.set_mode(MotionMode(ModeCode::of(*mode).as_u8()));
    out.set_moving_elapsed(Duration::from_nanos(moving_elapsed));
    *out.fault_mut() = fault_out;
    *out.trajectory_mut() = trajectory_out;

    out.set_prev_now_valid(prev_now.is_some());
    out.set_prev_now(Duration::from_nanos(prev_now_ns));

    write_joints(out.last_goal_mut(), last_goal);
    write_targets(out.last_targets_mut(), last_targets);
    write_pose(&mut SeedFields(out), fk_seed);
    out.set_present_min_margin(*present_min_margin);

    let ExcursionSnapshot {
        window,
        body_yaw,
        relative_yaw,
        cone,
    } = start_excursion;
    out.set_excursion_cranks(window);
    out.set_excursion_body_yaw(*body_yaw);
    out.set_excursion_relative_yaw(*relative_yaw);
    out.set_excursion_cone(*cone);

    out.set_miss_count(*miss_count);
    out.set_pose_failures(*pose_failures);

    for (slot, run) in out.tracking_mut().iter_mut().zip(tracking.iter()) {
        write_streak(slot, run.as_ref());
    }
    out.set_masked(joint_flags(*masked));
    Ok(())
}

/// The whole of the motion tick's state, as the slot holds it.
///
/// # Errors
///
/// [`MotionSlotError`], one variant per way a slot's numbers can fail to
/// describe a state — including the case this crossing exists for, a slot
/// nothing has written yet, whose zeroed mode names no mode.
pub fn read_motion_snap(slot: &MotionSnap) -> Result<MotionSnapshot, MotionSlotError> {
    let mode =
        match ModeCode::from_u8(slot.mode().0).ok_or(MotionSlotError::NoSuchMode(slot.mode().0))? {
            ModeCode::Holding => Mode::Holding,
            ModeCode::Moving => Mode::Moving {
                elapsed: duration_from_nanos(slot.moving_elapsed().as_nanos())?,
            },
            ModeCode::Faulted => Mode::Faulted(read_fault(slot.fault())?.to_fault()?),
        };

    let prev_now = slot
        .prev_now_valid()
        .then(|| duration_from_nanos(slot.prev_now().as_nanos()))
        .transpose()?;

    let mut tracking = [None; JointId::COUNT];
    for (run, entry) in tracking.iter_mut().zip(slot.tracking().iter()) {
        *run = read_streak(entry)?;
    }

    Ok(MotionSnapshot {
        mode,
        trajectory: read_trajectory(slot.trajectory())?,
        prev_now,
        last_goal: read_joints(slot.last_goal()),
        last_targets: read_targets(slot.last_targets())?,
        fk_seed: read_pose(&SeedRef(slot))?,
        present_min_margin: slot.present_min_margin(),
        start_excursion: ExcursionSnapshot {
            window: *slot.excursion_cranks(),
            body_yaw: slot.excursion_body_yaw(),
            relative_yaw: slot.excursion_relative_yaw(),
            cone: slot.excursion_cone(),
        },
        miss_count: slot.miss_count(),
        pose_failures: slot.pose_failures(),
        tracking,
        masked: joint_set(slot.masked())?,
    })
}

/// Write a running move's seed, or the absence of one.
fn write_trajectory(
    out: &mut TrajectorySeedSlot,
    seed: Option<&TrajectorySeed>,
) -> Result<(), MotionSlotError> {
    let Some(seed) = seed else {
        // Cleared rather than left: a path from a move that ended is still a
        // path, and the flag is the only thing saying it is over.
        *out = TrajectorySeedSlot::new();
        return Ok(());
    };
    let TrajectorySeed {
        start,
        target,
        durations,
        warp,
    } = seed;
    let flat = DurationsSnapshot::try_from(*durations)?;
    write_targets(out.start_mut(), start);
    write_targets(out.target_mut(), target);
    out.set_dur_head(Duration::from_nanos(flat.head_ns));
    out.set_dur_antenna_right(Duration::from_nanos(flat.antenna_right_ns));
    out.set_dur_antenna_left(Duration::from_nanos(flat.antenna_left_ns));
    out.set_warp(WarpKind(warp.as_u8()));
    out.set_present(true);
    Ok(())
}

/// The running move those fields describe, or `None` where none is running.
///
/// # Errors
///
/// [`MotionSlotError`] for a start or target that is not a command set, a clock
/// that is not a length of time, or a warp this build does not know.
fn read_trajectory(slot: &TrajectorySeedSlot) -> Result<Option<TrajectorySeed>, MotionSlotError> {
    if !slot.present() {
        return Ok(None);
    }
    let durations = DurationsSnapshot {
        head_ns: slot.dur_head().as_nanos(),
        antenna_right_ns: slot.dur_antenna_right().as_nanos(),
        antenna_left_ns: slot.dur_antenna_left().as_nanos(),
    }
    .to_durations()?;
    Ok(Some(TrajectorySeed {
        start: read_targets(slot.start())?,
        target: read_targets(slot.target())?,
        durations,
        warp: Warp::from_u8(slot.warp().0).ok_or(MotionSlotError::NoSuchWarp(slot.warp().0))?,
    }))
}

/// Write one joint's open tracking run, or the absence of one.
fn write_streak(out: &mut TrackingStreakSnap, run: Option<&TrackingStreakSnapshot>) {
    let Some(run) = run else {
        *out = TrackingStreakSnap::new();
        return;
    };
    out.set_active(true);
    out.set_anchor(run.anchor);
    out.set_side(TrackingSideKind(run.side.as_i8()));
    out.set_count(run.count);
}

/// The tracking run those fields describe, or `None` where none is open.
///
/// # Errors
///
/// [`MotionSlotError::NoSuchTrackingSide`] for a side this build does not know.
/// Read as "no direction" instead, an open run would be measuring toward a place
/// the state was never closing on.
fn read_streak(
    slot: &TrackingStreakSnap,
) -> Result<Option<TrackingStreakSnapshot>, MotionSlotError> {
    if !slot.active() {
        return Ok(None);
    }
    Ok(Some(TrackingStreakSnapshot {
        anchor: slot.anchor(),
        side: TrackingSide::from_i8(slot.side().0)
            .ok_or(MotionSlotError::NoSuchTrackingSide(slot.side().0))?,
        count: slot.count(),
    }))
}

/// The solver's seed, as the two fields [`MotionSnap`] spells it in.
///
/// A newtype rather than an `impl` on the message itself: [`MotionSnap`] holds
/// two poses — the seed and the head pose inside `last_targets` — and a trait
/// implemented on the message could only name one of them.
struct SeedFields<'a>(&'a mut MotionSnap);

/// The read-only half of the same, for a slot a caller only has a shared
/// borrow of.
struct SeedRef<'a>(&'a MotionSnap);

impl PoseRead for SeedFields<'_> {
    fn pos(&self) -> &Vec3 {
        self.0.fk_seed_pos()
    }

    fn quat(&self) -> &Quat {
        self.0.fk_seed_quat()
    }
}

impl PoseFields for SeedFields<'_> {
    fn pos_mut(&mut self) -> &mut Vec3 {
        self.0.fk_seed_pos_mut()
    }

    fn quat_mut(&mut self) -> &mut Quat {
        self.0.fk_seed_quat_mut()
    }
}

impl PoseRead for SeedRef<'_> {
    fn pos(&self) -> &Vec3 {
        self.0.fk_seed_pos()
    }

    fn quat(&self) -> &Quat {
        self.0.fk_seed_quat()
    }
}

/// Declare a cog's run totals: the struct, the two slot crossings, and the
/// change-guarded report.
///
/// One line per total, naming the field and the setter both the state slot and
/// the signal group spell for it, so a counter is added in one place instead of
/// four. The change guard is the load-bearing part: a total written on every
/// execution would put an observation in the report group at the control rate
/// and roll the group's window in seconds, and a total is an absolute count, so
/// whichever window it lands in carries the whole run.
///
/// The slot and signals types are parameters because they are generated per cog
/// and nothing here can name them. The form without a signals type is for a
/// crate the generated cog crate depends on, which therefore cannot name it: the
/// totals cross the slot here and the report is written where the type is
/// reachable.
///
/// The `crossing` clause names the round-trip case this emits into the calling
/// crate's test build, and is required so that a totals type cannot be declared
/// without one. The case is generated rather than written because the field list
/// is here: a pair declared with each other's setters compiles, and only a
/// distinct value in every field shows what it corrupts -- so the values are
/// counted out over the same repetition that declares the fields, and no caller
/// can hand two fields the same one.
#[macro_export]
macro_rules! counters {
    (
        $(#[$totals_doc:meta])*
        $name:ident of $slot:ty, crossing $crossing:ident {
            $($(#[$field_doc:meta])* $field:ident / $set:ident),+ $(,)?
        }
    ) => {
        $(#[$totals_doc])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
        pub struct $name {
            $($(#[$field_doc])* pub $field: u64,)+
        }

        impl $name {
            /// The totals the slot holds.
            #[must_use]
            pub fn read(state: &$slot) -> Self {
                Self { $($field: state.$field(),)+ }
            }

            /// Record them for the next execution.
            pub fn store(&self, state: &mut $slot) {
                $(state.$set(self.$field);)+
            }
        }

        /// Every total crosses the slot as itself: a distinct value in each
        /// field, stored, and read back field for field.
        #[cfg(test)]
        #[test]
        fn $crossing() {
            let mut totals = $name::default();
            let mut nth = 0u64;
            $(
                nth += 1;
                totals.$field = nth;
            )+
            let mut state = <$slot>::new();
            totals.store(&mut state);
            assert_eq!($name::read(&state), totals);
        }
    };

    (
        $(#[$totals_doc:meta])*
        $name:ident of $slot:ty, $signals:ty, crossing $crossing:ident {
            $($(#[$field_doc:meta])* $field:ident / $set:ident),+ $(,)?
        }
    ) => {
        $crate::counters! {
            $(#[$totals_doc])*
            $name of $slot, crossing $crossing {
                $($(#[$field_doc])* $field / $set),+
            }
        }

        impl $name {
            /// Report the ones that moved since `before`.
            pub fn report(&self, before: &Self, signals: &mut $signals) {
                $(
                    if self.$field != before.$field {
                        signals.$set(self.$field);
                    }
                )+
            }
        }
    };
}
