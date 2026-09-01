//! Unit tests for the host cogs, against the generated test wrappers.
//!
//! The wrappers wire no channels: each case publishes the samples it wants
//! seen, runs one execution, and reads back what was published and what the
//! slot holds. That is the whole harness -- no box, no process, no clock.
//!
//! Time is passed per call rather than simulated, so a case says at what instant
//! each execution happens. The samples carry their own instants in the datagram,
//! which is the time the estimator reports on; the execution's start time is the
//! carrier's, and the two are deliberately different numbers in these cases so
//! that one standing in for the other fails.

use std::cell::{Cell, RefCell};
use std::collections::VecDeque;

use brenn_reachy__cogs__config_clk_rs::{
    ClipLibraryConfigWire, MoverParamsWire, SessionParamsWire,
};
use brenn_reachy__cogs__motion_clk_rs_test::{
    MoverTestWrapper, PoseTestWrapper, SessionTestWrapper,
};
use brenn_reachy__cogs__schedule_clk_rs::{
    OverlayWindowWire, PostureWire, ScheduledStepWire, SessionScheduleWire, StepKindWire,
};
use brenn_reachy__cogs__script_clk_rs::{ScriptOverlayWire, ScriptStepWire, ScriptWire};
use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__cogs__session_cmd_clk_rs::{SessionCmdKindWire, SessionCmdWire};
use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
use brenn_reachy__driver__health_clk_rs::{
    AuxOutcomeWire, AuxStatusWire, DriverEventWire, EventKindWire, HealthReportWire,
};
use brenn_reachy__driver__pose_clk_rs::{PoseEstimateWire, PoseSampleWire};
use brenn_reachy__hardware__dynamixel__registers_clk_rs::{RegIdWire, ValueShapeWire};
use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKindWire;
use brenn_reachy__motion__faults_clk_rs::{FaultKindWire, ResponseKindWire, TickFaultWire};
use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointFlagsWire, JointRefWire, JointsWire};
use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKindWire};
use brenn_reachy__motion__seq_clk_rs::SeqKindWire;
use brenn_reachy__motion__tick_state_clk_rs::{MotionMode, MotionSnap, MotionSnapWire};
use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, WindDownOutcomeWire};
use clockwork_rs::{Clear as _, Duration as SlotDuration, SyncTime};
use nalgebra::Isometry3;
use reachy_clips::config::write_clip;
use reachy_clips::format::{Channel as ClipChannel, Clip, ClipDoc, FrameDoc};
use reachy_clips::speed::ClipLimits;
use reachy_driver::{NOMINAL_CYCLE_NS, STARTUP_INIT_BUDGET_NS};
use reachy_kin::{
    HeadGeometry, LegAngles, default_geometry, inverse_kinematics, neutral_head_pose,
    rest_head_pose, stow_head_pose, wrap_to_pi,
};
use reachy_motion::arm::{SERVO_IDS, row_of_id};
use reachy_motion::default_motion_config;
use reachy_motion::disarm::{
    DEFAULT_STOW_DWELL, DEFAULT_STOW_TOLERANCE, STOW_ANTENNAS, stow_targets,
};
use reachy_motion::fault;
use reachy_motion::joints::ROW_COUNT as JOINT_COUNT;
use reachy_motion::joints::{
    self, JointRef, Name, ROWS, flags, group_of, row, rows_of, write_rows,
};
use reachy_motion::record;
use reachy_motion::snap::PoseSnapshotError;
use reachy_motion::tick::ResponseKind;
use reachy_motion::value;
use reachy_motion::winddown::{Disposition, ending};
use session_slots::TIMELINE_LEN;

/// The validated tick state, panicking if the slot's bytes are not one.
fn state_of(slot: &MotionSnapWire) -> &MotionSnap {
    slot.validate().expect("a state a tick produced")
}

/// The instant every case starts from. Round rather than zero, so a time that
/// travelled through the wrong field is a number nothing else in the case is.
const T0: i64 = 1_700_000_000_000_000_000;

/// The control period, which is the grid the samples sit on.
const PERIOD: i64 = 20_000_000;

/// How far apart two poses may be and still be called the same one, metres and
/// radians. Two orders of magnitude looser than the solver's own tolerance
/// (1e-9 m), which is the band a round trip through inverse kinematics and back
/// lands in.
const CLOSE: f64 = 1e-7;

/// A wrapper with its input sized and not yet primed.
///
/// Unprimed matters: an input can only be sized before `initialize`, so this is
/// the state every case starts from.
fn pose_cog() -> PoseTestWrapper {
    let mut cog = PoseTestWrapper::new();
    cog.input_sample_set_num_slots(8);
    cog
}

/// The crank angles that hold `head` -- what the servos would be reading with
/// the head there.
fn cranks(head: &Isometry3<f64>) -> [f64; 6] {
    let mut angles = LegAngles([0.0; 6]);
    inverse_kinematics(&HeadGeometry::default(), head, &mut angles)
        .expect("a pose inside the workspace");
    angles.0
}

/// One sample as a case states it, before it is written into a message.
///
/// A plain struct rather than the schema type, so a case can build one and then
/// damage a single field of it; [`Sample::message`] is where it becomes the
/// message the channel carries.
struct Sample {
    /// The cycle's grid instant.
    nominal_time_ns: i64,
    /// When the read completed.
    sample_time_ns: i64,
    /// Whether the reading is a complete one.
    present_valid: bool,
    /// Whether a setpoint is being held.
    commanded_valid: bool,
    torque_off_latched: bool,
    /// The rows that did not answer, as the bits the schema holds: a case that
    /// wants a set this build does not know sets a bit above the ninth.
    missing: u16,
    /// Measured positions, radians in bus order.
    present: [f64; JOINT_COUNT],
    /// The setpoint held, radians in bus order.
    commanded: [f64; JOINT_COUNT],
}

impl Sample {
    fn message(&self) -> PoseSampleWire {
        let mut msg = PoseSampleWire::new();
        let view = msg.clear_valid();
        view.nominal_time = SyncTime::from_nanos(self.nominal_time_ns);
        view.sample_time = SyncTime::from_nanos(self.sample_time_ns);
        view.present_valid = self.present_valid.into();
        view.commanded_valid = self.commanded_valid.into();
        view.torque_off_latched = self.torque_off_latched.into();
        write_rows(&mut view.present, &self.present);
        write_rows(&mut view.commanded, &self.commanded);
        // Last, and through the open type: a case damages this field on purpose
        // to drive the boundary refusal, so the set is written as bits rather
        // than narrowed here.
        msg.set_missing(JointFlagsWire(self.missing));
        msg
    }
}

/// The nine angles the machine rests at, in bus-row order.
///
/// Through the schema's own vector, which is where the library states the
/// mapping between a servo's name and its bus row in both directions.
fn stow_rows() -> [f64; JOINT_COUNT] {
    let mut slot = JointsWire::new();
    joints::write_vector(
        slot.clear_valid(),
        &stow_targets(default_geometry()).expect("the baked geometry reaches stow"),
    );
    rows_of(
        slot.validate()
            .expect("a cleared vector of angles reads back"),
    )
}

/// A complete sample reading the crank angles that hold `head`.
fn sample_at(head: &Isometry3<f64>, at_ns: i64) -> Sample {
    let mut present = [0.0; JOINT_COUNT];
    present[1..7].copy_from_slice(&cranks(head));
    Sample {
        nominal_time_ns: at_ns,
        sample_time_ns: at_ns,
        present_valid: true,
        commanded_valid: true,
        torque_off_latched: false,
        missing: 0,
        present,
        commanded: [0.0; JOINT_COUNT],
    }
}

/// Hand a sample to the cog, as the driver's channel would.
fn publish(cog: &mut PoseTestWrapper, sample: &Sample, at_ns: i64) {
    cog.publish_sample(&sample.message(), SyncTime::from_nanos(at_ns));
}

/// Hand the cog a message validation will refuse.
///
/// A well-formed sample whose missing-rows field names a servo above the ninth
/// bus row, which is the failure a driver from a build that knows more servos
/// than this one produces: the bytes are the right size and every other field
/// reads, and the receiver's only answer is to drop the message.
fn publish_unreadable(cog: &mut PoseTestWrapper, at_ns: i64) {
    let mut sample = sample_at(&neutral_head_pose(), at_ns);
    sample.missing = 1 << 15;
    let msg = sample.message();
    assert!(
        msg.validate().is_err(),
        "the case rests on this message not validating",
    );
    cog.publish_sample(&msg, SyncTime::from_nanos(at_ns));
}

/// One published estimate, copied out of the message.
///
/// Copied because the wrapper returns a borrow into cog memory; further cog
/// calls would invalidate it.
struct Reading {
    /// The instant the estimate is about.
    time_of_validity_ns: i64,
    /// The measured positions it carried, radians in bus order.
    joints: [f64; JOINT_COUNT],
    /// Head position, metres.
    head_pos: [f64; 3],
    /// Head orientation, scalar first.
    head_quat: [f64; 4],
    /// Whether a pose was found.
    valid: bool,
    /// Newton iterations the solve took.
    fk_iters: u32,
    /// The worst per-leg residual at the answer, metres.
    fk_residual: f64,
    /// The pose those two fields describe, read through the same mapping the
    /// cog wrote them through.
    pose: Result<Isometry3<f64>, PoseSnapshotError>,
}

impl Reading {
    /// The pose it names, refused the way every reader of these fields refuses.
    fn pose(&self) -> Isometry3<f64> {
        self.pose.expect("a valid estimate names a pose")
    }
}

/// Copy an estimate out of the cog's memory.
fn read(estimate: &PoseEstimateWire) -> Reading {
    let estimate = estimate.validate().expect("the cog publishes an estimate");
    let (pos, quat) = (&estimate.head_pos, &estimate.head_quat);
    Reading {
        time_of_validity_ns: estimate.time_of_validity.as_nanos(),
        joints: rows_of(&estimate.joints),
        head_pos: [pos.x, pos.y, pos.z],
        head_quat: [quat.w, quat.x, quat.y, quat.z],
        pose: record::read_pose(pos, quat),
        valid: bool::from(estimate.valid),
        fk_iters: estimate.fk_iters,
        fk_residual: estimate.fk_residual,
    }
}

/// What one execution published, or `None` if it published nothing.
fn published(cog: &mut PoseTestWrapper) -> Option<Reading> {
    cog.try_next_estimate().map(read)
}

/// Publish one sample and run one execution, returning the estimate.
fn one_sample(cog: &mut PoseTestWrapper, sample: &Sample, at_ns: i64) -> Reading {
    publish(cog, sample, at_ns);
    assert!(cog.execute(SyncTime::from_nanos(at_ns)));
    published(cog).expect("every sample produces one estimate")
}

/// Assert two poses are the same to within the band above.
fn assert_close(found: &Isometry3<f64>, wanted: &Isometry3<f64>) {
    let offset = (found.translation.vector - wanted.translation.vector).norm();
    assert!(
        offset < CLOSE,
        "position off by {offset} m: {found} {wanted}"
    );
    let turn = found.rotation.angle_to(&wanted.rotation);
    assert!(turn < CLOSE, "orientation off by {turn} rad");
}

#[test]
fn a_sample_becomes_the_pose_its_cranks_hold() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    assert!(!cog.execute(SyncTime::from_nanos(T0 + PERIOD)));
    assert!(cog.try_next_estimate().is_none());

    let wanted = neutral_head_pose();
    let estimate = one_sample(&mut cog, &sample_at(&wanted, T0 + PERIOD), T0);

    assert!(estimate.valid, "a complete sample of a reachable pose");
    assert_eq!(
        estimate.time_of_validity_ns,
        T0 + PERIOD,
        "the sample's own instant, not the execution's",
    );
    assert_close(&estimate.pose(), &wanted);
    assert!(estimate.fk_residual < 1e-9, "the answer closes the linkage",);
}

#[test]
fn the_measured_joints_travel_whole_and_in_bus_order() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    // Every row a number nothing else in the case is, so a row that arrived in
    // the wrong field is visible rather than plausible.
    let mut sample = sample_at(&neutral_head_pose(), T0);
    sample.present[0] = 0.125;
    sample.present[7] = -0.25;
    sample.present[8] = 0.375;

    let estimate = one_sample(&mut cog, &sample, T0);
    assert_eq!(
        estimate.joints, sample.present,
        "every row, where it belongs"
    );
    assert_eq!(estimate.joints[0], 0.125);
    assert_eq!(estimate.joints[7], -0.25);
    assert_eq!(estimate.joints[8], 0.375);
}

#[test]
fn an_incomplete_sample_is_reported_as_such_rather_than_skipped() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    // A row that did not answer. Everything else about the sample is solvable,
    // so what makes this invalid is the mask and nothing else.
    let mut sample = sample_at(&neutral_head_pose(), T0);
    sample.missing = 1 << 3;

    let estimate = one_sample(&mut cog, &sample, T0);
    assert!(!estimate.valid, "a row is missing, so there is no reading");
    assert_eq!(estimate.time_of_validity_ns, T0);
    assert_eq!(estimate.joints, sample.present, "what was read is evidence");
    assert_eq!(estimate.fk_iters, 0, "no solve was attempted");

    assert!(!cog.state_est().have_seed());
}

#[test]
fn a_sample_the_driver_calls_stale_is_not_solved() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    let mut sample = sample_at(&neutral_head_pose(), T0);
    sample.present_valid = false;

    let estimate = one_sample(&mut cog, &sample, T0);
    assert!(!estimate.valid);
    assert!(!cog.state_est().have_seed());
}

/// The seed slot decides which configuration of the mechanism a solve lands in,
/// so numbers that are not a rotation are refused rather than normalised into a
/// plausible-looking one. Refusing falls back to the pose the cog starts from
/// before it has ever solved, and the next answer replaces it.
#[test]
fn a_seed_slot_holding_no_rotation_is_refused_and_not_repaired() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    let state = cog.state_est_mut();
    state.set_have_seed(true);
    // Twice unit length: four numbers that describe no rotation, which is what
    // memory nothing wrote looks like once a flag says it was written.
    let quat = state.seed_quat_mut();
    quat.set_w(2.0);
    quat.set_x(0.0);
    quat.set_y(0.0);
    quat.set_z(0.0);
    let quat = cog.state_est().seed_quat();
    let written = [quat.w(), quat.x(), quat.y(), quat.z()];

    let wanted = rest_head_pose();
    let estimate = one_sample(&mut cog, &sample_at(&wanted, T0), T0);
    assert!(
        estimate.valid,
        "the solve ran from the neutral pose instead"
    );
    assert_close(&estimate.pose(), &wanted);

    let quat = cog.state_est().seed_quat();
    assert_ne!(
        [quat.w(), quat.x(), quat.y(), quat.z()],
        written,
        "the answer replaced the slot the refusal came from",
    );
    assert_eq!(
        cog.state_est().refused_seeds(),
        1,
        "a refusal is counted, so a slot nobody wrote is visible rather than silent",
    );
}

#[test]
fn a_pose_that_will_not_converge_leaves_the_seed_alone() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    // One good sample first, so there is a seed to preserve.
    let wanted = rest_head_pose();
    let good = one_sample(&mut cog, &sample_at(&wanted, T0), T0);
    assert!(good.valid);
    assert!(cog.state_est().have_seed());
    let seed_pos = cog.state_est().seed_pos().z();

    // A crank angle nobody can place. The solver answers no pose at all for it,
    // which is the contract this rests on: there is no half-converged iterate
    // to leak into the seed.
    let mut broken = sample_at(&wanted, T0 + PERIOD);
    broken.present[2] = f64::NAN;

    let estimate = one_sample(&mut cog, &broken, T0 + PERIOD);
    assert!(!estimate.valid, "no pose was found");
    assert_eq!(estimate.time_of_validity_ns, T0 + PERIOD);
    assert_eq!(
        cog.state_est().seed_pos().z(),
        seed_pos,
        "the seed is the last pose found, not the last sample seen",
    );
    assert!(cog.state_est().have_seed());
    assert_eq!(
        cog.state_est().fk_failures(),
        1,
        "a complete sample the solver could not answer is a solve failure",
    );
}

#[test]
fn the_seed_is_the_last_pose_found_and_carries_across_executions() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    assert!(!cog.state_est().have_seed());

    // A run down to the stow pose, which is off-centre and pitched: far enough
    // from neutral that a solve seeded from neutral every time would be
    // answering a different question than one seeded from the pose before.
    let route = [neutral_head_pose(), rest_head_pose(), stow_head_pose()];
    for (step, wanted) in route.iter().enumerate() {
        let at = T0 + (step as i64) * PERIOD;
        let estimate = one_sample(&mut cog, &sample_at(wanted, at), at);
        assert!(estimate.valid, "step {step} is a reachable pose");
        assert_close(&estimate.pose(), wanted);

        assert!(cog.state_est().have_seed());
        let seed = cog.state_est().seed_pos();
        assert_eq!([seed.x(), seed.y(), seed.z()], estimate.head_pos);
    }
}

#[test]
fn a_burst_folds_to_the_last_sample_and_the_seed_follows_the_whole_run() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    // Three samples in one window, which is what a scheduling stall looks like
    // online. An output slot carries one message per execution, so the estimate
    // published is the newest -- an older one is superseded, not lost, because
    // the state it left behind is in the seed.
    let route = [neutral_head_pose(), rest_head_pose(), stow_head_pose()];
    for (step, wanted) in route.iter().enumerate() {
        let at = T0 + (step as i64) * PERIOD;
        publish(&mut cog, &sample_at(wanted, at), T0);
    }
    assert!(cog.execute(SyncTime::from_nanos(T0)));

    let estimate =
        published(&mut cog).expect("one estimate, not three: an output is a single slot");
    assert_eq!(estimate.time_of_validity_ns, T0 + 2 * PERIOD);
    assert_close(&estimate.pose(), &stow_head_pose());
    assert!(
        cog.try_next_estimate().is_none(),
        "an output slot carries one message per execution",
    );
}

#[test]
fn a_window_of_refused_samples_is_reported_as_an_outage_and_leaves_the_seed() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    // One good sample first, so there is a seed a dropped datagram could
    // disturb.
    let good = one_sample(&mut cog, &sample_at(&rest_head_pose(), T0), T0);
    assert!(good.valid);
    let seed_pos = cog.state_est().seed_pos().z();

    // The datagram carries an instant of its own -- it is a well-formed sample
    // this build will not read -- and the estimate must not report that one:
    // nothing decoded it, so the only instant the cog knows is its own.
    publish_unreadable(&mut cog, T0 + PERIOD);
    assert!(cog.execute(SyncTime::from_nanos(T0 + 2 * PERIOD)));

    let estimate = published(&mut cog).expect(
        "a window that decoded nothing is still an outage a consumer must see, not silence",
    );
    assert!(!estimate.valid);
    assert_eq!(
        estimate.time_of_validity_ns,
        T0 + 2 * PERIOD,
        "the execution's instant, there being no sample's",
    );
    assert_eq!(
        estimate.joints, [0.0; JOINT_COUNT],
        "no bytes were read, so no positions are claimed",
    );

    assert_eq!(cog.state_est().refused_samples(), 1);
    assert!(cog.state_est().have_seed());
    assert_eq!(
        cog.state_est().seed_pos().z(),
        seed_pos,
        "a dropped datagram is not a pose and does not move the seed",
    );
}

/// The three totals are the run's, not a window's: they live in the slot the
/// cog carries between executions, so a long run counts up rather than round.
#[test]
fn the_totals_count_the_run_and_stand_still_when_nothing_happens() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    for step in 0..3 {
        let at = T0 + step * PERIOD;
        publish_unreadable(&mut cog, at);
        assert!(cog.execute(SyncTime::from_nanos(at)));
    }
    assert_eq!(cog.state_est().refused_samples(), 3);
    assert_eq!(cog.state_est().fk_failures(), 0);
    assert_eq!(cog.state_est().refused_seeds(), 0);

    // A sample nobody can place: a solve that answered nothing, counted apart
    // from a datagram that was never a sample.
    let mut broken = sample_at(&rest_head_pose(), T0 + 3 * PERIOD);
    broken.present[2] = f64::NAN;
    let estimate = one_sample(&mut cog, &broken, T0 + 3 * PERIOD);
    assert!(!estimate.valid);
    assert_eq!(cog.state_est().fk_failures(), 1);
    assert_eq!(cog.state_est().refused_samples(), 3, "still the run's");

    // A sample that solves moves neither counter.
    let good = one_sample(&mut cog, &sample_at(&rest_head_pose(), T0 + 4 * PERIOD), T0);
    assert!(good.valid);
    assert_eq!(cog.state_est().fk_failures(), 1);
    assert_eq!(cog.state_est().refused_samples(), 3);
    assert_eq!(cog.state_est().refused_seeds(), 0);
}

#[test]
fn a_refused_sample_does_not_hide_the_samples_around_it() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    // The drop is per message; it does not poison the rest of the window.
    publish(&mut cog, &sample_at(&neutral_head_pose(), T0), T0);
    publish_unreadable(&mut cog, T0);
    publish(&mut cog, &sample_at(&stow_head_pose(), T0 + PERIOD), T0);
    assert!(cog.execute(SyncTime::from_nanos(T0)));

    let estimate = published(&mut cog).expect("the last decodable sample in the window");
    assert!(estimate.valid);
    assert_eq!(estimate.time_of_validity_ns, T0 + PERIOD);
    assert_close(&estimate.pose(), &stow_head_pose());
}

#[test]
fn an_invalid_estimate_carries_no_pose_from_the_one_before_it() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    let solved = one_sample(&mut cog, &sample_at(&rest_head_pose(), T0), T0);
    assert!(solved.valid);
    assert_ne!(solved.head_pos[2], 0.0);

    let mut missing = sample_at(&rest_head_pose(), T0 + PERIOD);
    missing.missing = 1;

    // The output slot is reused memory. What it held is not an estimate of
    // where the head is now, and a consumer reading the pose fields of an
    // invalid estimate should find nothing rather than something stale.
    let estimate = one_sample(&mut cog, &missing, T0 + PERIOD);
    assert!(!estimate.valid);
    assert_eq!(estimate.head_pos, [0.0; 3]);
    assert_eq!(estimate.head_quat, [0.0; 4]);
    assert_eq!(estimate.fk_residual, 0.0);
}

// -- the decision tick --------------------------------------------------------
//
// The mover's cases drive a machine as well as a cog: each one publishes a
// sample, runs an execution, and feeds the goal that came out back in as the
// next sample's measured positions -- a servo that arrives instantly, which is
// the plant that makes tracking error zero and therefore makes a case that sees
// a tracking fault mean it. Rows a case freezes stay where they are, which is
// what an obstruction is to a position loop. The cog's own goal channel is
// looped back by hand between executions, exactly as a box's `connect` would.

/// How many cycles ahead of its sample a goal is dated.
const LAG: i64 = 2;

/// How long the move to the upright posture is given.
const UP_NS: i64 = 800_000_000;

/// How long the move to stow is given.
const STOW_NS: i64 = 2_000_000_000;

/// One published setpoint, copied out of the message.
#[derive(Clone, Copy)]
struct Goal {
    /// The grid instant it is due at.
    execute_at_ns: i64,
    /// The rows it speaks for.
    mask: JointFlags,
    /// The angles asked for, in bus-row order.
    targets: [f64; JOINT_COUNT],
}

impl Goal {
    /// The same setpoint as the channel carries it, for the self-loop the
    /// wrappers do not wire.
    fn message(&self) -> GoalSetpointWire {
        let mut msg = GoalSetpointWire::new();
        let view = msg.clear_valid();
        view.execute_at = SyncTime::from_nanos(self.execute_at_ns);
        view.mask = self.mask;
        write_rows(&mut view.targets, &self.targets);
        msg
    }
}

/// One published report, copied out of the message.
#[derive(Clone, Copy, PartialEq, Debug)]
struct Report {
    /// The instant it is about.
    time_ns: i64,
    /// What was raised.
    kind: FaultKindWire,
    /// The servo concerned, or none.
    joint: JointRefWire,
    /// The magnitude that carried the classification.
    detail: f64,
    /// The count that carried it.
    count: u32,
}

/// What one execution published.
struct Cycle {
    /// The nominal instant of the sample that drove it.
    nominal: i64,
    /// The goal, where the machine was under command.
    goal: Option<Goal>,
    /// The report, where the tick had something to say.
    report: Option<Report>,
}

fn params(period_ns: i64, up_ns: i64, stow_ns: i64) -> MoverParamsWire {
    let mut message = MoverParamsWire::new();
    let params = message.clear_valid();
    params.lag_k = u32::try_from(LAG).expect("a small lag");
    params.period_ns = period_ns;
    params.up_duration_ns = up_ns;
    params.stow_duration_ns = stow_ns;
    message
}

/// A decision tick under test, with a machine in front of it.
struct Mover {
    /// The wrapper.
    cog: MoverTestWrapper,
    /// The nominal instant of the last sample published.
    now: i64,
    /// Where the modelled servos are, radians in bus order.
    present: [f64; JOINT_COUNT],
    /// Rows that do not follow their goal: an obstruction, to a position loop.
    frozen: JointFlags,
    /// Whether the samples carry a reading at all.
    blind: bool,
}

impl Mover {
    /// A cog at stow, disengaged, on the default parameters.
    fn new() -> Self {
        Self::on(&params(PERIOD, UP_NS, STOW_NS))
    }

    /// The same, with a clip library its overlay windows can name.
    fn playing(params: &MoverParamsWire, library: &ClipLibraryConfigWire) -> Self {
        let mut mover = Self::on(params);
        mover.cog.set_config_clips(library);
        mover
    }

    /// The same, configured as written -- including numbers the cog refuses.
    fn on(params: &MoverParamsWire) -> Self {
        let mut cog = MoverTestWrapper::new();
        cog.input_sample_set_num_slots(8);
        cog.input_sched_set_num_slots(1);
        cog.input_own_cmd_set_num_slots(1);

        // Seeded after `initialize`, and before the first execution: a config
        // record is not reachable until the wrapper has stood the cog up.
        cog.initialize(SyncTime::from_nanos(T0));
        cog.set_config_params(params);

        Self {
            cog,
            now: T0,
            present: stow_rows(),
            frozen: JointFlags::NONE,
            blind: false,
        }
    }

    /// Publish a schedule, as the session cog's channel would.
    ///
    /// The steps are half-open intervals from `T0`, one per posture named, each
    /// as long as `spans` says in cycles.
    fn schedule(&mut self, engaged: bool, epoch: u32, spans: &[(i64, Option<PostureWire>)]) {
        let spans: Vec<(i64, StepKindWire, PostureWire)> = spans
            .iter()
            .map(|(cycles, posture)| match posture {
                Some(posture) => (*cycles, StepKindWire::BASE_POSTURE, *posture),
                None => (*cycles, StepKindWire::BASE_KEEP, PostureWire::STOW),
            })
            .collect();
        self.schedule_raw(engaged, epoch, &spans);
    }

    /// The same, with each step's kind and posture written as given -- including
    /// values this build's vocabulary does not declare, which is what a schedule
    /// from a newer session cog carries.
    fn schedule_raw(
        &mut self,
        engaged: bool,
        epoch: u32,
        spans: &[(i64, StepKindWire, PostureWire)],
    ) {
        let mut schedule = SessionScheduleWire::new();
        schedule.set_engaged(engaged);
        schedule.set_epoch(epoch);
        {
            let mut steps = schedule.steps_mut();
            steps.clear();
            let mut start = T0;
            for (cycles, kind, posture) in spans {
                let end = start + cycles * PERIOD;
                let step: &mut ScheduledStepWire =
                    steps.try_grow().expect("sixteen steps is plenty");
                step.set_start(SyncTime::from_nanos(start));
                step.set_end(SyncTime::from_nanos(end));
                step.set_kind(*kind);
                step.set_posture(*posture);
                start = end;
            }
        }
        self.cog
            .publish_sched(&schedule, SyncTime::from_nanos(self.now));
    }

    /// Publish a schedule carrying overlay windows over one posture step.
    ///
    /// The step spans the whole run; each window is a motion, the cycles from
    /// `T0` it opens and closes on, its gain and its speed. Times are in cycles
    /// for the same reason the steps' are: a case says when something happens in
    /// the units its samples arrive in.
    fn schedule_playing(&mut self, epoch: u32, windows: &[(u16, i64, i64, f64, f64)]) {
        self.schedule_playing_engaged(true, epoch, windows);
    }

    /// The same, engaged or not: a schedule nobody is engaged on is what the
    /// interval between a script being accepted and an engagement concluding
    /// looks like to this cog.
    fn schedule_playing_engaged(
        &mut self,
        engaged: bool,
        epoch: u32,
        windows: &[(u16, i64, i64, f64, f64)],
    ) {
        let mut schedule = SessionScheduleWire::new();
        schedule.set_engaged(engaged);
        schedule.set_epoch(epoch);
        {
            let mut steps = schedule.steps_mut();
            steps.clear();
            let step: &mut ScheduledStepWire = steps.try_grow().expect("one step fits");
            step.set_start(SyncTime::from_nanos(T0));
            step.set_end(SyncTime::from_nanos(T0 + 1000 * PERIOD));
            step.set_kind(StepKindWire::BASE_POSTURE);
            step.set_posture(PostureWire::UP);
        }
        {
            let mut rows = schedule.overlays_mut();
            rows.clear();
            for (motion_id, opens, closes, gain, speed) in windows {
                let row: &mut OverlayWindowWire = rows.try_grow().expect("four windows fit");
                row.set_motion_id(*motion_id);
                row.set_start(SyncTime::from_nanos(T0 + opens * PERIOD));
                row.set_end(SyncTime::from_nanos(T0 + closes * PERIOD));
                row.set_gain(*gain);
                row.set_speed(*speed);
            }
        }
        self.cog
            .publish_sched(&schedule, SyncTime::from_nanos(self.now));
    }

    /// Hand the cog one sample, as the driver's channel would.
    fn publish_sample(&mut self, at_ns: i64) {
        let sample = Sample {
            nominal_time_ns: at_ns,
            sample_time_ns: at_ns,
            present_valid: !self.blind,
            commanded_valid: true,
            torque_off_latched: false,
            missing: if self.blind { u16::MAX >> 7 } else { 0 },
            present: self.present,
            commanded: [0.0; JOINT_COUNT],
        };
        self.cog
            .publish_sample(&sample.message(), SyncTime::from_nanos(at_ns));
    }

    /// Run one cycle: one sample in, whatever came out, and the machine moved.
    fn step(&mut self) -> Cycle {
        self.now += PERIOD;
        let nominal = self.now;
        self.publish_sample(nominal);
        assert!(
            self.cog.execute(SyncTime::from_nanos(nominal)),
            "a sample is what wakes this cog",
        );
        let cycle = self.collect(nominal);
        self.follow(cycle.goal.as_ref());
        cycle
    }

    /// Read this execution's outputs, and close the goal channel's self-loop.
    fn collect(&mut self, nominal: i64) -> Cycle {
        let goal = self.cog.try_next_goal().map(|msg| Goal {
            execute_at_ns: msg.execute_at().as_nanos(),
            mask: msg.mask().to_known().expect("a goal names bus rows"),
            targets: rows_of(
                &msg.validate()
                    .expect("the cog publishes a setpoint")
                    .targets,
            ),
        });
        let report = self.cog.try_next_fault().map(read_report);
        if let Some(goal) = &goal {
            self.cog
                .publish_own_cmd(&goal.message(), SyncTime::from_nanos(nominal));
        }
        Cycle {
            nominal,
            goal,
            report,
        }
    }

    /// Move the modelled servos to what they were last asked for.
    ///
    /// Instantly, and only the rows the goal names and the case has not frozen:
    /// a servo that arrives is a tracking error of zero, so a case that sees an
    /// obstruction fault has one.
    fn follow(&mut self, goal: Option<&Goal>) {
        let Some(goal) = goal else {
            return;
        };
        for joint in flags::iter(goal.mask) {
            let Some(row) = row(joint) else {
                continue;
            };
            if !flags::contains(self.frozen, joint) {
                self.present[row] = goal.targets[row];
            }
        }
    }

    /// Publish `count` samples into one window and run a single execution on
    /// all of them, which is what a scheduling stall produces online.
    fn burst(&mut self, count: i64) -> Cycle {
        let base = self.now;
        for step in 1..=count {
            self.publish_sample(base + step * PERIOD);
        }
        self.now = base + count * PERIOD;
        assert!(self.cog.execute(SyncTime::from_nanos(self.now)));
        let cycle = self.collect(self.now);
        self.follow(cycle.goal.as_ref());
        cycle
    }

    /// Run `cycles` of them, returning every one.
    fn run(&mut self, cycles: usize) -> Vec<Cycle> {
        (0..cycles).map(|_| self.step()).collect()
    }

    /// Which cycle from `T0` the last sample published was.
    ///
    /// The spans a schedule carries are counted from `T0`, so this is what a
    /// case saying "the next few samples" has to say it in.
    fn cycles_from_start(&self) -> i64 {
        (self.now - T0) / PERIOD
    }

    /// The angles the machine is at.
    fn at(&self, joint: JointRef) -> f64 {
        self.present[row(joint).expect("a bus row")]
    }
}

/// Copy a report out of the cog's memory.
fn read_report(fault: &TickFaultWire) -> Report {
    Report {
        time_ns: fault.time().as_nanos(),
        kind: fault.kind(),
        joint: fault.joint(),
        detail: fault.detail(),
        count: fault.count(),
    }
}

/// Which way an antenna points, wrapped into a half turn either side of
/// upright.
///
/// An antenna is a free rotor and a sweep takes the arc that misses its
/// outboard direction, so a fold at -3.05 rad reaches upright by continuing to
/// a whole turn rather than by turning back through the outboard side. The
/// reading is the turns as well as the direction; what a case about posture
/// means is the direction.
fn direction(angle: f64) -> f64 {
    let turn = angle.rem_euclid(core::f64::consts::TAU);
    if turn > core::f64::consts::PI {
        turn - core::f64::consts::TAU
    } else {
        turn
    }
}

/// Everything the cycles reported, in order.
fn reports(cycles: &[Cycle]) -> Vec<Report> {
    cycles.iter().filter_map(|cycle| cycle.report).collect()
}

/// A machine engaged and told to stand up, ticked until it is up.
fn standing_up() -> Mover {
    let mut mover = Mover::new();
    // One long step, so the whole run is inside it and a case that wants a
    // retarget publishes a fresh schedule rather than falling off the end.
    mover.schedule(true, 1, &[(1000, Some(PostureWire::UP))]);
    mover
}

#[test]
fn engaging_arms_from_the_sample_and_commands_the_posture_the_schedule_names() {
    let mut mover = standing_up();
    assert!(!mover.cog.state_ctrl().armed());

    let first = mover.step();
    assert!(
        mover.cog.state_ctrl().armed(),
        "engaged and readable is the whole arming condition",
    );
    let goal = first
        .goal
        .expect("an armed machine is commanded every sample");
    assert_eq!(
        goal.execute_at_ns,
        first.nominal + LAG * PERIOD,
        "a goal names the grid instant the lag puts it at",
    );
    assert_eq!(goal.mask, flags::all(), "nothing is out of service",);
    assert!(first.report.is_none(), "arming is not news");

    // The antennas unfold and the head rises: the run ends somewhere other than
    // where it started, which is what makes the assertions below about *how* it
    // got there worth making.
    let cycles = mover.run(60);
    assert!(cycles.iter().all(|cycle| cycle.goal.is_some()));
    assert!(reports(&cycles).is_empty(), "a clean move raises nothing");
    for antenna in [JointRef::AntennaRight, JointRef::AntennaLeft] {
        let angle = mover.at(antenna);
        assert!(
            direction(angle).abs() < 1e-9,
            "{} points upright, at {angle} rad",
            Name(antenna),
        );
    }
    assert_eq!(
        mover.cog.state_ctrl().samples_seen(),
        61,
        "every sample was ticked on",
    );
    assert_eq!(mover.cog.state_ctrl().goals_published(), 61);
}

#[test]
fn every_goal_names_a_later_instant_and_no_step_is_larger_than_a_servo_may_take() {
    let mut mover = standing_up();
    let cycles = mover.run(60);
    let step = default_motion_config().max_step;

    let mut previous: Option<Goal> = None;
    for cycle in &cycles {
        let goal = cycle.goal.expect("commanded on every sample");
        assert_eq!(goal.execute_at_ns, cycle.nominal + LAG * PERIOD);
        if let Some(previous) = previous {
            assert!(
                goal.execute_at_ns > previous.execute_at_ns,
                "the stream is monotonic",
            );
            for joint in ROWS {
                let row = row(joint).expect("a bus row");
                let delta = (goal.targets[row] - previous.targets[row]).abs();
                let bound = match group_of(joint) {
                    Some(reachy_motion::joints::JointGroup::Legs) => step.legs,
                    Some(reachy_motion::joints::JointGroup::BodyYaw) => step.body_yaw,
                    Some(reachy_motion::joints::JointGroup::Antennas) => step.antennas,
                    None => continue,
                };
                assert!(
                    delta <= bound,
                    "{} steps {delta} rad, past {bound}",
                    Name(joint)
                );
            }
        }
        previous = Some(goal);
    }
}

/// A machine that has arrived is still a machine under command: the goal stream
/// is what holds the driver's dead-man off, so a hold re-publishes rather than
/// falling silent.
#[test]
fn a_holding_machine_keeps_the_goal_stream_alive_with_the_setpoint_it_is_on() {
    let mut mover = standing_up();
    let moving = mover.run(60);
    let arrived = moving
        .last()
        .expect("a run of cycles")
        .goal
        .expect("a goal");

    let holding = mover.run(10);
    let mut previous = arrived;
    for cycle in &holding {
        let goal = cycle.goal.expect("a holding session is not a stopped one");
        assert_eq!(
            goal.targets, arrived.targets,
            "the setpoint it is already on, republished",
        );
        assert!(goal.execute_at_ns > previous.execute_at_ns);
        previous = goal;
    }
    assert!(reports(&holding).is_empty());
}

#[test]
fn a_session_nobody_engaged_is_never_armed_and_never_commands() {
    let mut mover = Mover::new();
    mover.schedule(false, 1, &[(1000, Some(PostureWire::UP))]);

    let cycles = mover.run(5);
    assert!(cycles.iter().all(|cycle| cycle.goal.is_none()));
    assert!(reports(&cycles).is_empty());
    assert!(!mover.cog.state_ctrl().armed());
    assert_eq!(
        mover.cog.state_ctrl().samples_seen(),
        5,
        "the samples were seen; there was nothing to do about them",
    );

    // A schedule with no message at all is the same thing said by silence.
    let mut quiet = Mover::new();
    let cycles = quiet.run(2);
    assert!(cycles.iter().all(|cycle| cycle.goal.is_none()));
    assert!(!quiet.cog.state_ctrl().armed());
}

#[test]
fn disengaging_ends_the_session_and_stops_the_stream() {
    let mut mover = standing_up();
    mover.run(20);
    assert!(mover.cog.state_ctrl().armed());

    mover.schedule(false, 2, &[(1000, Some(PostureWire::UP))]);
    let cycles = mover.run(3);
    assert!(
        cycles.iter().all(|cycle| cycle.goal.is_none()),
        "nothing is commanded once the session is over",
    );
    assert!(
        !mover.cog.state_ctrl().armed(),
        "the state dies with the engagement",
    );
    assert_eq!(
        mover.cog.state_ctrl().schedule_epoch_seen(),
        1,
        "a disengaged sample answers nothing, so it spends no epoch either",
    );
    assert_eq!(mover.cog.state_ctrl().epochs_answered(), 1);

    // Engaging again builds a fresh state from where the machine now stands,
    // rather than resuming the one that ended.
    mover.schedule(true, 3, &[(1000, Some(PostureWire::STOW))]);
    let fresh = mover.step();
    assert!(mover.cog.state_ctrl().armed());
    assert!(fresh.goal.is_some());
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert!(
        snap.mode == MotionMode::Moving,
        "a fresh engagement dispatches the posture its schedule names",
    );
    assert_eq!(
        mover.cog.state_ctrl().schedule_epoch_seen(),
        3,
        "and that dispatch is what consumes the epoch it came under",
    );
    assert_eq!(
        mover.cog.state_ctrl().epochs_answered(),
        2,
        "the engagement's epoch, answered once and not again by the re-arm",
    );
}

/// Arming solves the pose the cranks hold, and a sample that carries no reading
/// cannot be solved from. That is not a fault: nothing is under command yet, and
/// a pre-torque problem never faults -- the cog simply tries again on the next
/// sample.
#[test]
fn a_sample_with_no_reading_arms_nothing_and_raises_nothing() {
    let mut mover = standing_up();
    mover.blind = true;

    let cycles = mover.run(3);
    assert!(cycles.iter().all(|cycle| cycle.goal.is_none()));
    assert!(reports(&cycles).is_empty(), "arming is not a fault path");
    assert!(!mover.cog.state_ctrl().armed());

    mover.blind = false;
    let armed = mover.step();
    assert!(mover.cog.state_ctrl().armed(), "retried, with no edge kept");
    assert!(armed.goal.is_some());
}

/// A slot claiming a machine it does not describe is refused and counted, and
/// the next sample arms a fresh one. This is the loop's own commander: a cog
/// that aborted over its own memory would take the machine's only source of
/// goals down with it, saying nothing about which sample or which field.
#[test]
fn a_slot_that_describes_no_machine_re_arms_rather_than_aborting() {
    let mut mover = standing_up();
    mover.run(3);
    assert!(mover.cog.state_ctrl().armed());
    let goals = mover.cog.state_ctrl().goals_published();

    // Armed over a snapshot nobody wrote: every field reads as zero, and zero
    // is a mode the vocabulary does not have.
    mover.cog.state_ctrl_mut().snap_mut().clear();

    let cycle = mover.step();
    assert_eq!(
        mover.cog.state_ctrl().refused_state(),
        1,
        "the refusal is counted where a reader can see it",
    );
    assert!(
        mover.cog.state_ctrl().armed(),
        "the sample that met the refusal armed a fresh state from the pose it carried",
    );
    assert!(
        cycle.goal.is_some(),
        "a fresh engagement commands the posture its schedule names",
    );
    assert_eq!(
        mover.cog.state_ctrl().goals_published(),
        goals + 1,
        "the stream carried on across the refusal",
    );
    assert!(
        reports(&[cycle]).is_empty(),
        "a slot nobody wrote is not something the machine did",
    );
}

/// A ctrl slot whose bytes this build cannot read at all is cleared, counted
/// and re-armed on the same cycle, and the goal stream carries on across it.
///
/// Distinct from a slot that reads as a state and describes none: this one
/// fails validation, which is the branch a peer built against another schema
/// reaches. The clear is what keeps it to one cycle -- without it the cog
/// returns before ever running the arming path, and the stream stops for good
/// rather than for a cycle.
#[test]
fn a_slot_this_build_cannot_read_is_cleared_and_re_armed() {
    use brenn_reachy__motion__tick_state_clk_rs::MotionModeWire;

    let mut mover = standing_up();
    mover.run(3);
    assert!(mover.cog.state_ctrl().armed());
    let goals = mover.cog.state_ctrl().goals_published();

    // A discriminant the schema does not declare, written through the open
    // surface -- the only way one reaches a slot at all.
    mover
        .cog
        .state_ctrl_mut()
        .snap_mut()
        .set_mode(MotionModeWire(7));
    assert!(
        mover.cog.state_ctrl().snap().validate().is_err(),
        "the slot is damaged"
    );

    let cycle = mover.step();
    assert_eq!(
        mover.cog.state_ctrl().refused_state(),
        1,
        "the refusal is counted once, where a reader can see it",
    );
    assert!(
        mover.cog.state_ctrl().armed(),
        "the cleared slot armed a fresh state from the pose the sample carried",
    );
    assert!(cycle.goal.is_some(), "the same cycle commanded the machine");
    assert_eq!(
        mover.cog.state_ctrl().goals_published(),
        goals + 1,
        "the stream carried on across the refusal",
    );
    assert!(
        reports(&[cycle]).is_empty(),
        "bytes nobody in this build wrote are not something the machine did",
    );

    // And damaged a second time, the same cycle's worth of refusal: the count
    // rises and the stream does not latch off.
    mover
        .cog
        .state_ctrl_mut()
        .snap_mut()
        .set_mode(MotionModeWire(7));
    let again = mover.step();
    assert_eq!(mover.cog.state_ctrl().refused_state(), 2);
    assert!(mover.cog.state_ctrl().armed());
    assert!(again.goal.is_some());
}

/// The tick counts a sample with no reading as a miss and keeps holding, so the
/// goal stream carries on: the machine is still under command, and the report
/// that eventually comes is about the reads, not about this cycle.
#[test]
fn a_missing_reading_mid_session_is_a_miss_and_not_a_silence() {
    let mut mover = standing_up();
    mover.run(20);

    mover.blind = true;
    let blind = mover.run(10);
    assert!(
        blind.iter().all(|cycle| cycle.goal.is_some()),
        "a machine nobody can see is still a machine being commanded",
    );
    assert!(reports(&blind).is_empty(), "ten misses is not the fault");
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert_eq!(snap.miss_count, 10, "the run of misses is being counted");
}

/// The read-loss fault latches, so the tick parks and re-reports it on every
/// cycle after. One raise is one message: the standing re-reports are
/// suppressed, and the goal stream stops with the latch -- which is how the
/// driver's dead-man comes to de-torque the machine.
#[test]
fn losing_the_reads_reports_once_and_stops_the_stream() {
    let mut mover = standing_up();
    mover.run(20);

    mover.blind = true;
    let blind = mover.run(60);
    let raised = reports(&blind);
    assert_eq!(raised.len(), 1, "one raise is one message");
    let report = raised[0];
    assert_eq!(report.kind, FaultKindWire::POSITION_FEEDBACK_LOST);
    assert_eq!(report.joint, JointRefWire::NONE, "the reads, not a servo");
    assert_eq!(
        report.count,
        default_motion_config().read_loss_ticks + 1,
        "the run of misses that carried it: the bound is passed, not reached",
    );

    let latched = blind
        .iter()
        .position(|cycle| cycle.report.is_some())
        .expect("the fault was raised");
    assert!(
        blind[..latched].iter().all(|cycle| cycle.goal.is_some()),
        "commanded until the latch",
    );
    assert!(
        blind[latched..].iter().all(|cycle| cycle.goal.is_none()),
        "and never after it: the dead-man takes the machine down from here",
    );
    assert_eq!(
        mover.cog.state_ctrl().faults_raised(),
        1,
        "the standing re-reports are the same fault, not further ones",
    );
    assert_eq!(mover.cog.state_ctrl().reports_dropped(), 0);
}

/// A latching fault ends the session's ability to command, and the way out is a
/// fresh engagement rather than a flag being cleared.
#[test]
fn a_fresh_engagement_is_the_way_out_of_a_latched_fault() {
    let mut mover = standing_up();
    mover.run(20);
    mover.blind = true;
    mover.run(60);
    assert!(mover.step().goal.is_none(), "parked");

    mover.blind = false;
    mover.schedule(false, 2, &[(1000, Some(PostureWire::UP))]);
    mover.step();
    mover.schedule(true, 3, &[(1000, Some(PostureWire::UP))]);
    let fresh = mover.step();
    assert!(
        fresh.goal.is_some(),
        "a new engagement is a new state, which commands again",
    );
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert!(snap.mode != MotionMode::Faulted);
}

/// A jammed antenna is a fault confined to a group: the pair goes out of
/// service together -- one antenna limp and the other posed is a machine
/// pulling a face -- the move carries on without them, and every goal after
/// says so in its mask.
#[test]
fn a_jammed_antenna_takes_the_pair_out_of_service_and_the_goals_stop_naming_them() {
    let mut mover = standing_up();
    mover.frozen = {
        let mut set = JointFlags::NONE;
        flags::insert(&mut set, JointRef::AntennaRight);
        set
    };

    let cycles = mover.run(60);
    let raised = reports(&cycles);
    let first = raised.first().expect("a jammed servo is reported");
    assert_eq!(first.kind, FaultKindWire::ANTENNA_OBSTRUCTED);
    assert_eq!(
        first.joint,
        JointRefWire::ANTENNA_RIGHT,
        "the servo it is about",
    );
    assert!(first.detail.abs() > 0.5, "how far it stood from its goal");

    let after = cycles
        .iter()
        .position(|cycle| cycle.report.is_some())
        .expect("it was raised");
    for cycle in &cycles[after + 1..] {
        let goal = cycle.goal.expect("the move carries on without it");
        let mask = goal.mask;
        for antenna in [JointRef::AntennaRight, JointRef::AntennaLeft] {
            assert!(
                !flags::contains(mask, antenna),
                "a servo out of service is never written again",
            );
        }
        assert!(
            flags::contains(mask, JointRef::BodyYaw) && flags::contains(mask, JointRef::Leg0),
            "the head is sound and still being commanded",
        );
    }
    assert_eq!(
        raised.len(),
        1,
        "entry into the mask is the raise; the standing condition is not news",
    );
}

/// A jammed head joint is not confined to a group: the move is abandoned and
/// the tick holds. It does not latch, so the machine stays under command --
/// which is the whole difference between a machine that cannot be commanded and
/// one that must not be commanded further.
#[test]
fn a_jammed_crank_abandons_the_move_and_the_machine_holds_under_command() {
    let mut mover = standing_up();
    mover.frozen = {
        let mut set = JointFlags::NONE;
        for joint in ROWS {
            if group_of(joint) == Some(reachy_motion::joints::JointGroup::Legs) {
                flags::insert(&mut set, joint);
            }
        }
        set
    };

    let cycles = mover.run(60);
    let raised = reports(&cycles);
    let first = raised.first().expect("a jammed crank is reported");
    assert_eq!(first.kind, FaultKindWire::HEAD_OBSTRUCTED);
    assert_ne!(first.joint, JointRefWire::NONE, "the crank it is about");

    let after = cycles
        .iter()
        .position(|cycle| cycle.report.is_some())
        .expect("it was raised");
    let held = cycles[after].goal.expect("holding is still commanding");
    for cycle in &cycles[after + 1..] {
        let goal = cycle.goal.expect("the keep-alive outlives a hold");
        assert_eq!(
            goal.targets, held.targets,
            "a hold re-publishes what it is on",
        );
    }
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert_eq!(snap.mode, MotionMode::Holding, "it holds, it does not park");
}

#[test]
fn a_schedule_that_retargets_is_dispatched_and_the_stream_does_not_break() {
    let mut mover = standing_up();
    mover.run(20);

    // Mid-move, the session asks for stow instead. The epoch is what says the
    // schedule changed; the posture is what says where to.
    mover.schedule(true, 2, &[(1000, Some(PostureWire::STOW))]);
    let cycles = mover.run(120);
    assert!(
        cycles.iter().all(|cycle| cycle.goal.is_some()),
        "a retarget is a new path, not a gap in the stream",
    );
    assert!(reports(&cycles).is_empty(), "a retarget refuses nothing");

    let stow = stow_targets(default_geometry()).expect("the baked geometry reaches stow");
    for joint in ROWS {
        let at = mover.at(joint);
        let wanted = stow.get(joint).expect("a bus row");
        assert!(
            (at - wanted).abs() < 1e-6,
            "{} settled at {at} rather than {wanted}",
            Name(joint),
        );
    }
    assert_eq!(
        mover.at(JointRef::AntennaRight),
        STOW_ANTENNAS[0],
        "the antennas folded back",
    );
}

/// A schedule that says the same thing again asks for nothing new: the epoch is
/// what marks a change, and a step that keeps the base is not a retarget.
#[test]
fn a_repeated_schedule_dispatches_nothing_new() {
    let mut mover = standing_up();
    mover.run(60);
    let arrived = mover.step().goal.expect("a goal").targets;

    mover.schedule(true, 1, &[(1000, Some(PostureWire::UP))]);
    let same = mover.run(3);
    for cycle in &same {
        assert_eq!(
            cycle.goal.expect("still commanded").targets,
            arrived,
            "the same schedule moves nothing",
        );
    }
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert_eq!(snap.mode, MotionMode::Holding, "no move was started");
    assert_eq!(
        mover.cog.state_ctrl().epochs_answered(),
        1,
        "only the schedule that engaged the machine; a sample that is not a \
         retarget counts nothing",
    );
}

/// Two posture steps under one epoch are two dispatches and one epoch answered:
/// the step boundary sends the machine somewhere with the epoch unchanged, which
/// is what says the total counts epochs rather than dispatches.
#[test]
fn two_posture_steps_under_one_epoch_answer_it_once() {
    // The first step ends at the cycle from `T0` that the second one starts on,
    // a step's end being exclusive, and it is longer than the up move so the
    // machine is holding when the boundary arrives. The up move runs on the
    // floored clock rather than the configured 0.8 s: the pair sweeps between
    // the stow and the working posture mirrored, so the later antenna's clock is
    // lengthened by about a fifth to part their tips, which puts the end of the
    // move a little under fifty cycles in.
    const BOUNDARY: i64 = 60;
    let mut mover = Mover::new();
    mover.schedule(
        true,
        1,
        &[
            (BOUNDARY, Some(PostureWire::UP)),
            (1000, Some(PostureWire::STOW)),
        ],
    );

    let up = mover.run(usize::try_from(BOUNDARY - 1).expect("cycles inside the first step"));
    assert!(reports(&up).is_empty(), "a plain stand-up refuses nothing");
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert_eq!(snap.mode, MotionMode::Holding, "the up move is over");
    assert_eq!(mover.cog.state_ctrl().epochs_answered(), 1);
    assert_eq!(mover.cog.state_ctrl().schedule_epoch_seen(), 1);

    // Across the boundary, on the same epoch: the posture differs, so the step
    // is dispatched by the posture rather than by a retarget.
    let stowing = mover.run(3);
    assert!(
        stowing.iter().all(|cycle| cycle.goal.is_some()),
        "a step boundary is a new path, not a gap in the stream",
    );
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert!(
        snap.mode == MotionMode::Moving,
        "the second step was dispatched",
    );
    assert_eq!(
        mover.cog.state_ctrl().epochs_answered(),
        1,
        "a second dispatch under an epoch already answered counts nothing",
    );
    assert_eq!(mover.cog.state_ctrl().schedule_epoch_seen(), 1);
}

/// Two bumps that both land inside one gap are one answer: an execution sees
/// only the latest schedule, so the first epoch is never observed at all. What
/// the total says is which epoch changes this cog saw answered -- not how many a
/// session published, which is why subtracting it from a publish count is not a
/// count of outstanding retargets.
#[test]
fn two_bumps_in_one_gap_are_answered_once() {
    let mut mover = standing_up();
    mover.run(60);
    let arrived = mover.step().goal.expect("a goal").targets;

    // A keep span over the samples that follow, then the posture the machine is
    // already on -- the shape of the surviving-bump case, with a second bump
    // published inside the same span.
    const KEEP: usize = 6;
    let keep_until = mover.cycles_from_start() + i64::try_from(KEEP).expect("six cycles") + 1;
    let spans = [(keep_until, None), (1000, Some(PostureWire::UP))];
    mover.schedule(true, 2, &spans);
    mover.run(3);
    assert_eq!(
        mover.cog.state_ctrl().schedule_epoch_seen(),
        1,
        "the first bump is outstanding",
    );

    mover.schedule(true, 3, &spans);
    let keeping = mover.run(KEEP - 3);
    for cycle in &keeping {
        assert_eq!(
            cycle.goal.expect("still commanded").targets,
            arrived,
            "a keep span dispatches nothing, whatever the epoch says",
        );
    }
    assert_eq!(
        mover.cog.state_ctrl().epochs_answered(),
        1,
        "two bumps published, neither answered yet",
    );

    let moving = mover.run(2);
    assert!(
        moving.iter().all(|cycle| cycle.goal.is_some()),
        "the stream never broke",
    );
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert!(
        snap.mode == MotionMode::Moving,
        "the surviving bump was answered",
    );
    assert_eq!(
        mover.cog.state_ctrl().schedule_epoch_seen(),
        3,
        "the epoch the step answered is the latest one, the only one it saw",
    );
    assert_eq!(
        mover.cog.state_ctrl().epochs_answered(),
        2,
        "one answer for the two bumps: the superseded one was never observed",
    );
}

/// A schedule republished under a bumped epoch is the session saying "go again",
/// and it is dispatched even when it names the posture the machine is already
/// on: the epoch is the only thing that differs, so nothing else could carry it.
#[test]
fn an_epoch_bump_alone_dispatches_a_fresh_move() {
    let mut mover = standing_up();
    mover.run(60);
    let arrived = mover.step().goal.expect("a goal").targets;
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert_eq!(
        snap.mode,
        MotionMode::Holding,
        "the move is over before the bump"
    );
    assert_eq!(mover.cog.state_ctrl().schedule_epoch_seen(), 1);
    assert_eq!(
        mover.cog.state_ctrl().epochs_answered(),
        1,
        "the schedule that engaged the machine is the first epoch answered",
    );

    // The same step and the same posture at one epoch higher, covering the
    // instants the samples that follow name.
    mover.schedule(true, 2, &[(1000, Some(PostureWire::UP))]);
    let again = mover.run(3);
    assert!(reports(&again).is_empty(), "a retarget refuses nothing");

    // The setpoints a same-posture retarget plans are the ones the machine was
    // already holding, so what these cycles say is that the stream carried them
    // without a break: a retarget that dropped a cycle would trip the driver's
    // dead-man mid-session.
    for cycle in &again {
        assert_eq!(
            cycle
                .goal
                .expect("a retarget is a new path, not a gap")
                .targets,
            arrived,
            "the machine is asked for where it already is, every cycle",
        );
    }

    // A fresh path, not the finished move's remains: the first sample after the
    // bump started one, so the elapsed time is the samples since. The machine is
    // already at UP, so the setpoints a same-posture retarget produces are the
    // ones it was already holding -- the move's own clock is what says a path
    // exists at all.
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert_eq!(snap.mode, MotionMode::Moving);
    assert_eq!(
        snap.moving_elapsed,
        SlotDuration::from_nanos(2 * PERIOD),
        "the bumped epoch is the whole reason a move was started",
    );
    assert_eq!(
        mover.cog.state_ctrl().schedule_epoch_seen(),
        2,
        "and the dispatch consumed it",
    );
    assert_eq!(
        mover.cog.state_ctrl().epochs_answered(),
        2,
        "and the epoch a step answered is counted",
    );

    // And the path is the configured length, so a session watching the mode
    // sees a whole move rather than a mode word that clears on the next tick.
    let length = usize::try_from(UP_NS / PERIOD).expect("a move of whole cycles");
    mover.run(length - 3);
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert!(
        snap.mode == MotionMode::Moving,
        "one cycle short of the move's length is still moving",
    );
    mover.step();
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert_eq!(
        snap.mode,
        MotionMode::Holding,
        "and the move is over on its length"
    );
}

/// A bumped epoch that arrives during a gap is not spent there: the schedule
/// arriving is not the schedule answering, so the retarget stands until a
/// posture step covers an instant. Otherwise whether a session's "go again" took
/// effect would turn on how its publication happened to line up with a step
/// boundary.
#[test]
fn an_epoch_bump_in_a_gap_survives_until_a_step_answers() {
    let mut mover = standing_up();
    mover.run(60);
    let arrived = mover.step().goal.expect("a goal").targets;

    // A keep span over the samples that follow, then the posture the machine is
    // already on. The bump is published under the keep span, which asks for
    // nothing. The span is stated from where the machine now stands rather than
    // hand-counted from `T0`, and it ends one cycle past the last sample it is
    // to cover, a step's end being exclusive.
    const KEEP: usize = 4;
    let keep_cycles = i64::try_from(KEEP).expect("four cycles");
    let keep_until = mover.cycles_from_start() + keep_cycles + 1;
    mover.schedule(
        true,
        2,
        &[(keep_until, None), (1000, Some(PostureWire::UP))],
    );
    let keeping = mover.run(KEEP);
    for cycle in &keeping {
        assert_eq!(
            cycle.goal.expect("still commanded").targets,
            arrived,
            "a keep span dispatches nothing, whatever the epoch says",
        );
    }
    assert_eq!(
        mover.cog.state_ctrl().schedule_epoch_seen(),
        1,
        "the bump is outstanding, not consumed by the gap",
    );
    assert_eq!(
        mover.cog.state_ctrl().epochs_answered(),
        1,
        "one epoch answered so far, which is what says the bump is outstanding",
    );
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert_eq!(snap.mode, MotionMode::Holding, "nothing was dispatched");

    let moving = mover.run(2);
    assert!(
        moving.iter().all(|cycle| cycle.goal.is_some()),
        "the stream never broke",
    );
    assert_eq!(
        mover.cog.state_ctrl().schedule_epoch_seen(),
        2,
        "the step that answered is what consumed it",
    );
    assert_eq!(
        mover.cog.state_ctrl().epochs_answered(),
        2,
        "and the answer is what counted it",
    );
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert!(
        snap.mode == MotionMode::Moving,
        "the retarget outlived the gap it landed in",
    );
}

/// A step that keeps the base holds whatever posture is already commanded, and
/// an instant no step covers does the same: neither is a reason to send the
/// machine anywhere.
#[test]
fn a_step_that_keeps_the_base_and_a_gap_both_hold() {
    let mut mover = Mover::new();
    mover.schedule(true, 1, &[(5, None), (1000, Some(PostureWire::UP))]);

    let keeping = mover.run(4);
    for cycle in &keeping {
        assert!(cycle.goal.is_some(), "engaged and armed is commanded");
    }
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert_eq!(snap.mode, MotionMode::Holding, "nothing was dispatched");

    let moving = mover.run(2);
    assert!(moving.iter().all(|cycle| cycle.goal.is_some()));
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert!(
        snap.mode == MotionMode::Moving,
        "the posture step that follows is dispatched",
    );
}

/// The burst rule, which only a scheduling stall produces online: the goals are
/// superseded, so the last of them wins the one output slot, and the state
/// effects of the rest are already folded into the snapshot.
#[test]
fn a_burst_folds_to_the_last_goal() {
    let mut mover = standing_up();
    mover.run(10);

    let base = mover.now;
    for step in 1..=3 {
        mover.publish_sample(base + step * PERIOD);
    }
    mover.now = base + 3 * PERIOD;
    assert!(mover.cog.execute(SyncTime::from_nanos(mover.now)));

    let cycle = mover.collect(mover.now);
    let goal = cycle.goal.expect("one goal, not three");
    assert_eq!(
        goal.execute_at_ns,
        base + 3 * PERIOD + LAG * PERIOD,
        "the last sample of the window decided it",
    );
    assert!(
        mover.cog.try_next_goal().is_none(),
        "an output slot carries one message per execution",
    );
    assert_eq!(
        mover.cog.state_ctrl().samples_seen(),
        13,
        "every sample was ticked on, whatever the slot could carry",
    );
    assert_eq!(
        mover.cog.state_ctrl().goals_published(),
        11,
        "one datagram per execution, not per sample",
    );
}

#[test]
fn a_sample_validation_refuses_is_counted_and_ticks_nothing() {
    let mut mover = standing_up();
    mover.run(5);
    let before = mover.cog.state_ctrl().samples_seen();

    // A well-formed sample whose missing-rows field names a servo above the
    // ninth bus row: every other field reads, which is what a driver from a
    // build that knows more servos than this one looks like from here.
    let mut sample = Sample {
        nominal_time_ns: mover.now + PERIOD,
        sample_time_ns: mover.now + PERIOD,
        present_valid: true,
        commanded_valid: true,
        torque_off_latched: false,
        missing: 0,
        present: mover.present,
        commanded: [0.0; JOINT_COUNT],
    };
    sample.missing = 1 << 15;
    let msg = sample.message();
    assert!(msg.validate().is_err());

    mover.now += PERIOD;
    mover
        .cog
        .publish_sample(&msg, SyncTime::from_nanos(mover.now));
    assert!(mover.cog.execute(SyncTime::from_nanos(mover.now)));

    let cycle = mover.collect(mover.now);
    assert!(
        cycle.goal.is_none(),
        "no sample was read, so no cycle was decided",
    );
    assert_eq!(mover.cog.state_ctrl().refused_samples(), 1);
    assert_eq!(
        mover.cog.state_ctrl().samples_seen(),
        before,
        "bytes that are not a sample are not a sample",
    );
}

/// The state slot is the whole of what this cog carries between executions, so
/// what it holds after a run has to be the machine that run produced.
#[test]
fn the_tick_state_survives_the_slot_it_is_kept_in() {
    let mut mover = standing_up();
    let cycles = mover.run(30);
    let last = cycles.last().expect("a run").goal.expect("a goal");

    let state = mover.cog.state_ctrl();
    assert!(state.armed());
    assert_eq!(state.schedule_epoch_seen(), 1);
    assert_eq!(state.desired_kind(), StepKindWire::BASE_POSTURE);
    assert_eq!(state.desired_posture(), PostureWire::UP);

    let snap = state_of(state.snap());
    assert!(snap.mode == MotionMode::Moving);
    assert_eq!(
        joints::rows_of(&snap.last_goal),
        last.targets,
        "the goal in the slot is the goal that went out",
    );
    assert!(
        bool::from(snap.trajectory.present),
        "the path the move is running on outlived the execution",
    );
    assert_eq!(snap.masked, JointFlags::NONE);
}

/// A window that latches a fault part way through commands nothing at all. The
/// goal an earlier sample of the same window decided is superseded by the latch,
/// not by a later goal: publishing it would feed the driver's dead-man once more
/// and command the machine past the point the loop decided it must not be.
#[test]
fn a_fault_latching_mid_burst_leaves_the_window_commanding_nothing() {
    let mut mover = standing_up();
    mover.run(20);

    // Blind up to the cycle before the read-loss bound: the misses are counted,
    // nothing is raised, and the machine is still commanded.
    let ticks = i64::from(default_motion_config().read_loss_ticks);
    mover.blind = true;
    let quiet = mover.run(usize::try_from(ticks - 1).expect("a small bound"));
    assert!(quiet.iter().all(|cycle| cycle.goal.is_some()));
    assert!(reports(&quiet).is_empty(), "the bound is not passed yet");
    let published = mover.cog.state_ctrl().goals_published();

    // Three more misses in one window. The first passes nothing, the second
    // passes the bound and latches, and the third is a machine already parked.
    let cycle = mover.burst(3);
    assert!(
        cycle.goal.is_none(),
        "a goal decided before the latch is not published after it",
    );
    assert_eq!(
        mover.cog.state_ctrl().goals_published(),
        published,
        "the stream stopped with the latch, whatever the window held",
    );
    let report = cycle.report.expect("the latch is news");
    assert_eq!(report.kind, FaultKindWire::POSITION_FEEDBACK_LOST);
    assert_eq!(
        mover.cog.state_ctrl().samples_seen(),
        u64::try_from(20 + ticks + 2).expect("a small run"),
        "every sample of the window was ticked on",
    );
}

/// A command the tick refuses changes nothing: it is reported, the machine goes
/// nowhere, and the goal stream carries on holding what it was already on.
///
/// The refusal is reached the way one is reached on the machine -- an antenna
/// standing where no servo count can be commanded back from, which is what a
/// reading far outside the goal band means to the sweep that has to resolve it.
#[test]
fn a_command_the_tick_refuses_is_reported_and_moves_nothing() {
    let mut mover = standing_up();
    // Far enough round that neither arc back to upright lands inside the
    // antenna's goal band.
    let stranded = 2000.0;
    mover.present[row(JointRef::AntennaRight).expect("a bus row")] = stranded;

    let first = mover.step();
    assert!(mover.cog.state_ctrl().armed(), "the refusal is not arming");
    let report = first.report.expect("a refused command is reported");
    assert_eq!(report.kind, FaultKindWire::COMMAND_REJECTED);
    assert_eq!(
        report.joint,
        JointRefWire::ANTENNA_RIGHT,
        "the servo the refusal is about",
    );
    assert!(
        (report.detail - stranded).abs() <= core::f64::consts::PI,
        "the arc it was asked for, from where the antenna stands: {}",
        report.detail,
    );
    assert_eq!(report.count, 0, "a refusal has no count of failed checks");

    assert!(
        first.goal.is_some(),
        "a refusal changes nothing, and holding is still commanding",
    );
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert_eq!(snap.mode, MotionMode::Holding, "no move was started");
    assert_eq!(
        mover.cog.state_ctrl().faults_raised(),
        1,
        "one refusal, one raise",
    );
}

/// A move given less time than a servo can travel in is floored rather than
/// abandoned: this cog right-sizes the clock before it commands it, which is
/// what the tick's own step guard states it expects of a caller.
///
/// What this case pins is that this caller does not produce the abandonment:
/// the machine goes where the schedule asked, over the span a servo can
/// actually travel in, and the anomaly is counted as one rather than being
/// absorbed silently.
#[test]
fn a_move_no_servo_could_step_through_is_floored_and_counted() {
    // The whole stand-up in one cycle, which every crank would have to cross in
    // one bus period.
    let mut mover = Mover::on(&params(PERIOD, PERIOD, STOW_NS));
    mover.schedule(true, 1, &[(1000, Some(PostureWire::UP))]);

    let cycles = mover.run(80);
    assert!(
        reports(&cycles).is_empty(),
        "a floored move is not abandoned: {:?}",
        reports(&cycles),
    );
    assert!(
        cycles.iter().all(|cycle| cycle.goal.is_some()),
        "the machine is commanded throughout",
    );
    assert_eq!(
        mover.cog.state_ctrl().base_stretched(),
        1,
        "a clock no span fits inside is the anomaly, counted once for the plan",
    );
    assert_eq!(
        mover.cog.state_ctrl().base_dephased(),
        0,
        "a span stretch is not routine, however the pair was parted with it",
    );

    let snap = state_of(mover.cog.state_ctrl().snap());
    assert_eq!(snap.mode, MotionMode::Holding, "the floored move finished");
    for joint in [JointRef::AntennaRight, JointRef::AntennaLeft] {
        assert!(
            wrap_to_pi(mover.at(joint)).abs() < 1e-6,
            "{joint:?} stands at {} rather than upright",
            mover.at(joint),
        );
    }
}

/// A posture this build's vocabulary does not declare is not a reason to stand
/// up: stow is where the machine rests and where the minimum risk condition is,
/// so an unrecognised value goes there.
#[test]
fn a_posture_this_build_does_not_know_goes_to_stow() {
    let mut mover = standing_up();
    mover.run(60);
    assert!(
        (mover.at(JointRef::AntennaRight) - STOW_ANTENNAS[0]).abs() > 1.0,
        "the machine is up, so stow is somewhere else",
    );

    // A number no enumerator of this build's vocabulary carries, which is what a
    // schedule from a newer session cog would hold.
    mover.schedule(true, 2, &[(1000, Some(PostureWire(200)))]);
    let cycles = mover.run(140);
    assert!(
        cycles.iter().all(|cycle| cycle.goal.is_some()),
        "an unknown posture is a move, not a gap in the stream",
    );

    let stow = stow_targets(default_geometry()).expect("the baked geometry reaches stow");
    for joint in ROWS {
        let at = mover.at(joint);
        let wanted = stow.get(joint).expect("a bus row");
        assert!(
            (at - wanted).abs() < 1e-6,
            "{} settled at {at} rather than the {wanted} rad stow asks for",
            Name(joint),
        );
    }
}

/// A step kind this build cannot read holds whatever posture is already
/// commanded, the same as a step that keeps the base: a schedule this build only
/// half understands is not a reason to send the machine anywhere.
#[test]
fn a_step_kind_this_build_cannot_read_holds() {
    let mut mover = Mover::new();
    mover.schedule_raw(
        true,
        1,
        &[
            (5, StepKindWire(200), PostureWire::UP),
            (1000, StepKindWire::BASE_POSTURE, PostureWire::UP),
        ],
    );

    let unread = mover.run(4);
    assert!(
        unread.iter().all(|cycle| cycle.goal.is_some()),
        "engaged and armed is commanded",
    );
    assert!(
        reports(&unread).is_empty(),
        "an unread step refuses nothing"
    );
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert_eq!(snap.mode, MotionMode::Holding, "nothing was dispatched");

    let moving = mover.run(2);
    assert!(moving.iter().all(|cycle| cycle.goal.is_some()));
    let snap = state_of(mover.cog.state_ctrl().snap());
    assert!(
        snap.mode == MotionMode::Moving,
        "the posture step that follows is dispatched",
    );
}

/// An execution has one report slot, so a window that raises twice publishes
/// the first and counts the rest -- which is what lets a reader of the log know
/// whether it is reading all of them.
#[test]
fn a_second_raise_in_one_execution_is_counted_rather_than_quietly_lost() {
    // A machine whose cranks are jammed from the start and whose antennas jam
    // eleven cycles in: the antennas' window runs out one cycle before the
    // cranks' does, so the two raises are one cycle apart.
    let mut mover = standing_up();
    mover.frozen = {
        let mut set = JointFlags::NONE;
        for joint in ROWS {
            if group_of(joint) == Some(reachy_motion::joints::JointGroup::Legs) {
                flags::insert(&mut set, joint);
            }
        }
        set
    };
    mover.run(11);
    mover.frozen = flags::all();

    // Up to the cycle before the first of them, one sample per execution.
    let quiet = mover.run(14);
    assert!(reports(&quiet).is_empty(), "neither window has run out yet");
    assert_eq!(mover.cog.state_ctrl().faults_raised(), 0);

    // Both raises now fall in one window, which is what a scheduling stall
    // online does to them.
    let cycle = mover.burst(2);
    let report = cycle.report.expect("the window raised something");
    assert_eq!(
        report.kind,
        FaultKindWire::ANTENNA_OBSTRUCTED,
        "the first raise of the execution keeps the slot",
    );
    assert_eq!(
        mover.cog.state_ctrl().faults_raised(),
        2,
        "both were raised, whatever the slot could carry",
    );
    assert_eq!(
        mover.cog.state_ctrl().reports_dropped(),
        1,
        "the one that lost the slot is counted, not silent",
    );
}

/// Every configured length of time is checked at every execution, because a
/// scenario that asked for a cycle of no length, or a move of none, would
/// otherwise produce a plausible-looking run of a machine nobody meant.
///
/// One bad field per case, so a check dropped for that field is a case that
/// stops refusing. The refusal names the field it is about, but the message
/// does not reach here: the panic crosses the generated wrapper's C++ shim and
/// arrives as the shim's own, with the cog's on stderr. Which duration is which
/// is held instead by the moves themselves -- the up and stow steps are given
/// different lengths, and the cases that time them fail if the pair is swapped.
#[test]
#[should_panic(expected = "execute() failed")]
fn a_control_period_of_no_length_is_refused() {
    let mut mover = Mover::on(&params(0, UP_NS, STOW_NS));
    mover.schedule(true, 1, &[(1000, Some(PostureWire::UP))]);
    mover.step();
}

#[test]
#[should_panic(expected = "execute() failed")]
fn a_control_period_running_backwards_is_refused() {
    let mut mover = Mover::on(&params(-PERIOD, UP_NS, STOW_NS));
    mover.schedule(true, 1, &[(1000, Some(PostureWire::UP))]);
    mover.step();
}

#[test]
#[should_panic(expected = "execute() failed")]
fn a_move_to_the_up_posture_given_no_time_is_refused() {
    let mut mover = Mover::on(&params(PERIOD, 0, STOW_NS));
    mover.schedule(true, 1, &[(1000, Some(PostureWire::UP))]);
    mover.step();
}

#[test]
#[should_panic(expected = "execute() failed")]
fn a_move_to_stow_given_no_time_is_refused() {
    let mut mover = Mover::on(&params(PERIOD, UP_NS, 0));
    mover.schedule(true, 1, &[(1000, Some(PostureWire::UP))]);
    mover.step();
}

// The overlay layer's cases. A machine driven through the wrapper with a clip
// library bound, so what is under test is the whole of what a period does: the
// screen, the handover, the composition, the re-anchor that hands the base back,
// and the latch a refused composition leaves behind.

/// A clip driving the antennas alone, whose frames walk `step` radians further
/// out each frame, so a frame confused with its neighbour shows up in a goal.
///
/// Loaded through the emitter's own path with step bounds wide enough that the
/// round numbers survive it: what is under test is the layer, not the speed
/// derivation.
fn antenna_clip(step: f64, frames: usize) -> Clip {
    let doc = ClipDoc {
        version: 1,
        kind: "clip".to_owned(),
        name: "wiggle".to_owned(),
        description: None,
        channels: vec![ClipChannel::Antennas],
        frame_hz: reachy_motion::FLOOR_TICK_HZ,
        max_speed: 2.0,
        blend_in_ms: Some(40),
        blend_out_ms: Some(60),
        frames: (0..frames)
            .map(|index| FrameDoc {
                antennas: Some([step * index as f64, -step * index as f64]),
                ..FrameDoc::default()
            })
            .collect(),
    };
    let limits = ClipLimits {
        max_step: reachy_motion::joints::JointStep {
            legs: 100.0,
            body_yaw: 100.0,
            antennas: 100.0,
        },
        ..ClipLimits::default()
    };
    Clip::from_doc(doc, &limits).expect("the fixture loads")
}

/// A library of one clip and the one-segment motion that plays it, which is
/// motion 0 -- the shape the emitter writes for a clip nothing composes.
fn one_motion(step: f64, frames: usize) -> Box<ClipLibraryConfigWire> {
    library_naming(0, step, frames)
}

/// The same library, with the motion's one segment naming `clip_id`.
///
/// A clip id the library does not hold is what a library that will not establish
/// looks like: a structural fault, found by every establishment of it and by the
/// cheap one as much as by the frame walk.
fn library_naming(clip_id: u16, step: f64, frames: usize) -> Box<ClipLibraryConfigWire> {
    let clip = antenna_clip(step, frames);
    let mut out = Box::new(ClipLibraryConfigWire::new());
    {
        let message = out.clear_valid();
        write_clip(&clip, message.clips.try_grow().expect("one clip fits"))
            .expect("the fixture fits");
        let motion = message.motions.try_grow().expect("one motion fits");
        motion.lead_gap_ms = 0;
        let segment = motion.segments.try_grow().expect("one segment fits");
        segment.clip_id = clip_id;
        segment.speed = 1.0;
        segment.gap_after_ms = 0;
    }
    out
}

/// How far each servo travelled between consecutive goals, cycle by cycle.
fn travel(cycles: &[Cycle]) -> Vec<(JointRef, f64)> {
    let goals: Vec<&Goal> = cycles
        .iter()
        .map(|cycle| cycle.goal.as_ref().expect("an engaged machine commands"))
        .collect();
    goals
        .windows(2)
        .flat_map(|pair| {
            ROWS.iter().map(move |joint| {
                let index = row(*joint).expect("a bus row");
                (*joint, pair[1].targets[index] - pair[0].targets[index])
            })
        })
        .collect()
}

/// Every commanded step is one the machine may take in a period.
///
/// The continuity invariant at the unit level: a window closing while its
/// player still carries weight must not put the whole weighted delta into one
/// period, and this is what would see it. Asserted here rather than left to the
/// tick's own guard because the tick answers an oversized tracked setpoint by
/// refusing it, which is a hole in the composition rather than a slam -- and a
/// hole is what a case looking only for faults would miss.
fn assert_steps_fit(cycles: &[Cycle], what: &str) {
    let bounds = default_motion_config().max_step;
    for (joint, delta) in travel(cycles) {
        let bound = match group_of(joint).expect("a servo") {
            reachy_motion::joints::JointGroup::Legs => bounds.legs,
            reachy_motion::joints::JointGroup::BodyYaw => bounds.body_yaw,
            reachy_motion::joints::JointGroup::Antennas => bounds.antennas,
        };
        assert!(
            delta.abs() <= bound,
            "{what}: {} stepped {delta} rad in one period, past the {bound} rad bound",
            Name(joint)
        );
    }
}

/// How far one servo's commanded setpoint moved between consecutive periods.
///
/// One number per pair of cycles, so `deltas[i]` is the travel from `cycles[i]`
/// to `cycles[i + 1]`.
fn deltas(cycles: &[Cycle], joint: JointRef) -> Vec<f64> {
    let index = row(joint).expect("a bus row");
    let goals: Vec<f64> = cycles
        .iter()
        .map(|cycle| {
            cycle
                .goal
                .as_ref()
                .expect("an engaged machine commands")
                .targets[index]
        })
        .collect();
    goals.windows(2).map(|pair| pair[1] - pair[0]).collect()
}

/// The commanded stream is continuous into the cycle at `close`.
///
/// The continuity invariant at the unit level, and the assertion that does not
/// rest on a configured bound: what a window closing while its player still
/// carries weight would do is put that whole contribution into one period, and
/// this measures the travel across the close against the travel the periods just
/// before it were already taking. The per-period step bound passes any jolt
/// smaller than itself, which is most of them; this passes none.
fn assert_continuous_across(cycles: &[Cycle], close: usize, what: &str) {
    /// How many periods before the close the run-up is read over. Enough to
    /// cover a clip's own frame-to-frame variation, short enough to still be the
    /// same part of the motion.
    const RUN_UP: usize = 4;
    /// The arithmetic slack: a period's travel either side of the close is the
    /// same arithmetic on the same numbers, so nothing but rounding separates
    /// them.
    const SLACK: f64 = 1e-6;

    assert!(
        close > RUN_UP && close < cycles.len(),
        "{what}: cycle {close} is not a close with a run-up in this run of {}",
        cycles.len(),
    );
    for joint in ROWS {
        let travel = deltas(cycles, joint);
        let across = travel[close - 1].abs();
        let run_up = travel[close - 1 - RUN_UP..close - 1]
            .iter()
            .map(|delta| delta.abs())
            .fold(0.0, f64::max);
        assert!(
            across <= run_up + SLACK,
            "{what}: {} travelled {across} rad into cycle {close}, against {run_up} rad in the \
             periods before it -- the contribution the window was carrying went out in one step",
            Name(joint),
        );
    }
}

/// The load-bearing case: an overlay window composes over the base the schedule
/// is carrying, the player is picked up again on every execution rather than
/// restarted, and the window closing hands the base back without a step no
/// servo could take.
#[test]
fn an_overlay_window_composes_over_the_base_and_hands_it_back() {
    const OPENS: i64 = 5;
    const CLOSES: i64 = 15;
    let library = one_motion(0.02, 40);
    let mut mover = Mover::playing(&params(PERIOD, UP_NS, STOW_NS), &library);
    mover.schedule_playing(1, &[(0, OPENS, CLOSES, 1.0, 1.0)]);

    // The same schedule with no window, driven in step, which is what "the
    // composition is visible" is visible against.
    let mut bare = Mover::new();
    bare.schedule(true, 1, &[(1000, Some(PostureWire::UP))]);

    let mut composed = Vec::new();
    let mut plain = Vec::new();
    let mut clocks = Vec::new();
    // Long enough that the base finishes the move it was handed back on. That
    // clock is longer than the configured one: the hand-back is planned from the
    // composed setpoint, over the span still ahead of it, and the floor sizes it
    // for that span rather than for the one a stand-up from stow covers.
    for _ in 0..100 {
        composed.push(mover.step());
        plain.push(bare.step());
        clocks.push(mover.cog.state_ctrl().players()[0].clock_s());
    }

    assert!(
        mover.cog.state_ctrl().library_walked(),
        "the library was established and the fact recorded",
    );
    assert_eq!(mover.cog.state_ctrl().overlays_refused(), 0);
    assert_eq!(mover.cog.state_ctrl().players_refused(), 0);

    // Before the window, the two machines are commanded the same thing; inside
    // it, the antennas carry the clip's delta and nothing else does.
    let antenna = row(JointRef::AntennaRight).expect("a bus row");
    let body = row(JointRef::BodyYaw).expect("a bus row");
    let at = |cycles: &[Cycle], cycle: usize, index: usize| {
        cycles[cycle]
            .goal
            .as_ref()
            .expect("an engaged machine commands")
            .targets[index]
    };
    for cycle in 0..usize::try_from(OPENS).expect("a few cycles") - 1 {
        assert!(
            (at(&composed, cycle, antenna) - at(&plain, cycle, antenna)).abs() < CLOSE,
            "cycle {cycle} differed before any window opened",
        );
    }
    let inside = usize::try_from(CLOSES).expect("a few cycles") - 2;
    assert!(
        (at(&composed, inside, antenna) - at(&plain, inside, antenna)).abs() > 0.05,
        "the clip's delta is not visible in the goal stream",
    );
    assert!(
        (at(&composed, inside, body) - at(&plain, inside, body)).abs() < CLOSE,
        "an antennas-only clip moved the body",
    );

    // The player is the row, picked up again every execution: its clock runs on
    // through the window and stops when the window closes.
    let opening = usize::try_from(OPENS).expect("a few cycles");
    assert!(
        clocks[opening - 2] == 0.0,
        "nothing played before the window"
    );
    for cycle in opening..usize::try_from(CLOSES).expect("a few cycles") - 1 {
        assert!(
            clocks[cycle] > clocks[cycle - 1],
            "the player restarted at cycle {cycle}",
        );
    }
    assert_eq!(
        clocks[99], 0.0,
        "a closed window leaves no player in the row",
    );

    // The whole run, the close included, stays inside the per-period bound; and
    // the close itself is continuous, which is the re-anchor's own assertion --
    // the player was carrying about 0.2 rad when its window shut, well inside
    // the step bound, so nothing but this would see that contribution go out in
    // one period.
    assert_steps_fit(&composed, "a window opening and closing mid-move");
    assert_continuous_across(
        &composed,
        usize::try_from(CLOSES).expect("a few cycles") - 1,
        "the window closing",
    );
    assert!(reports(&composed).is_empty(), "nothing was refused");
    assert!(
        (mover.at(JointRef::AntennaRight) - bare.at(JointRef::AntennaRight)).abs() < 1e-3,
        "the base ended at {} against the plain run's {}",
        mover.at(JointRef::AntennaRight),
        bare.at(JointRef::AntennaRight),
    );
}

/// A window naming a motion the library does not have is refused once for the
/// schedule that carried it, not once for every period it spans -- and the base
/// carries on, because an overlay is presence and never safety.
#[test]
fn a_window_naming_no_motion_is_refused_once_and_the_base_carries_on() {
    let library = one_motion(0.02, 10);
    let mut mover = Mover::playing(&params(PERIOD, UP_NS, STOW_NS), &library);
    mover.schedule_playing(1, &[(3, 2, 40, 1.0, 1.0)]);

    let cycles = mover.run(20);
    assert_eq!(
        mover.cog.state_ctrl().overlays_refused(),
        1,
        "a refusal per schedule, not per period",
    );
    assert!(mover.cog.state_ctrl().library_walked());
    assert!(reports(&cycles).is_empty(), "a refused window is no fault");
    assert_steps_fit(&cycles, "a base streaming alone");

    // The same windows under a new epoch are screened again -- and the library
    // is not walked again, which is what the recorded walk is for.
    mover.schedule_playing(2, &[(3, 2, 40, 1.0, 1.0)]);
    mover.run(3);
    assert_eq!(mover.cog.state_ctrl().overlays_refused(), 2);
    assert!(mover.cog.state_ctrl().library_walked());
}

/// A composed setpoint the tick will not have drops this schedule's overlays
/// whole: the same clip over the same base composes the same setpoint, so the
/// layer is latched rather than offering the refusal again every period. The
/// refusal travels once, the base streams on, and a new epoch clears it.
#[test]
fn a_refused_composition_latches_the_layer_for_that_schedule() {
    // A clip stepping further in one frame than any antenna may travel in a
    // period, which the loader accepts under the fixture's wide bounds and the
    // tick refuses on the wire's behalf.
    let library = one_motion(1.0, 20);
    let mut mover = Mover::playing(&params(PERIOD, UP_NS, STOW_NS), &library);
    mover.schedule_playing(1, &[(0, 2, 60, 1.0, 1.0)]);

    let cycles = mover.run(20);
    let refused = reports(&cycles);
    assert_eq!(refused.len(), 1, "the refusal is reported once");
    assert_eq!(refused[0].kind, FaultKindWire::COMMAND_REJECTED);
    assert!(
        mover.cog.state_ctrl().overlay_latch(),
        "the layer is latched for this schedule",
    );
    assert_eq!(mover.cog.state_ctrl().overlays_refused(), 1);
    assert_eq!(mover.cog.state_ctrl().latch_epoch(), 1);
    assert!(
        cycles.iter().all(|cycle| cycle.goal.is_some()),
        "a refused composition does not stop the base",
    );
    assert_steps_fit(&cycles, "a base streaming under a latched layer");

    // A new epoch is a fresh set of windows: the latch is cleared, the layer
    // takes them up again, and this one is refused again.
    mover.schedule_playing(2, &[(0, mover.cycles_from_start() + 2, 60, 1.0, 1.0)]);
    let again = mover.run(10);
    assert_eq!(reports(&again).len(), 1, "the fresh schedule was tried");
    assert!(mover.cog.state_ctrl().overlay_latch());
    assert_eq!(mover.cog.state_ctrl().latch_epoch(), 2);
    assert_eq!(mover.cog.state_ctrl().overlays_refused(), 2);
}

/// A handover onto a clock too short for the span still ahead of the base runs
/// on a longer one, and says so.
///
/// The base is this cog's to plan while a window is open, and the floor is
/// applied to every plan this cog makes -- the handover here, and the posture
/// step of the case below it.
#[test]
fn a_handover_onto_a_clock_too_short_for_its_span_is_stretched() {
    /// The plans this run makes: the posture step it opens with, and the
    /// handover the window takes the base over on.
    const EXPECTED_STRETCHES: u64 = 2;

    let library = one_motion(0.02, 40);
    let mut mover = Mover::playing(&params(PERIOD, PERIOD, STOW_NS), &library);
    mover.schedule_playing(1, &[(0, 3, 60, 1.0, 1.0)]);

    let cycles = mover.run(70);
    // Exact on both totals, as the posture case is: every plan this run makes
    // is asked for on one period, so each of them is a span the machine cannot
    // step through, and the routine total staying at zero is what says the two
    // buckets are exclusive on this path as well.
    assert_eq!(
        mover.cog.state_ctrl().base_stretched(),
        EXPECTED_STRETCHES,
        "the whole stand-up in one period was planned as asked for",
    );
    assert_eq!(
        mover.cog.state_ctrl().base_dephased(),
        0,
        "a clock that could not carry its span is not a routine parting",
    );
    assert_eq!(mover.cog.state_ctrl().refused_base(), 0);
    assert_steps_fit(&cycles[3..], "a base on a stretched clock");
    // The run carries on past the window, because a stretched clock is only
    // half of what this path owes: the hand-back off a base whose move was
    // lengthened is still continuous.
    assert_continuous_across(&cycles, 59, "the window closing off a stretched clock");
}

/// A healthy posture move is counted as the de-phasing it is, and leaves the
/// anomaly total alone.
///
/// The two totals are what makes either of them readable: a machine doing what
/// it was built to do reports a de-phasing per move and no stretch at all, so
/// `base_stretched` is the number that says the configured clocks no longer
/// cover the spans they are being asked for.
#[test]
fn a_healthy_posture_move_is_a_de_phasing_and_not_a_stretch() {
    let mut mover = standing_up();
    let cycles = mover.run(60);
    assert!(
        reports(&cycles).is_empty(),
        "a plain stand-up refuses nothing"
    );
    assert_eq!(
        mover.cog.state_ctrl().base_dephased(),
        1,
        "the pair was parted for the one step this schedule carries",
    );
    assert_eq!(
        mover.cog.state_ctrl().base_stretched(),
        0,
        "nothing was asked for that the machine could not step through",
    );

    // And the fold back is the same move mirrored, so it is the same reading.
    mover.schedule(true, 2, &[(1000, Some(PostureWire::STOW))]);
    mover.run(140);
    assert_eq!(mover.cog.state_ctrl().base_dephased(), 2);
    assert_eq!(mover.cog.state_ctrl().base_stretched(), 0);
}

/// Two windows over one base, staggered: when the first closes with the second
/// still playing, the base is re-anchored under the surviving contribution and
/// the composed stream does not move.
///
/// The multi-row path, which is the one where continuity is this cog's own: with
/// a single window the close is a hand-back and the tick re-plans from the
/// setpoint it last commanded, so the tick absorbs it. Here the stream is still
/// a composed `Track` on the period after the close, so what the vacated
/// contribution is subtracted out of is this cog's arithmetic and nothing else.
#[test]
fn a_window_closing_under_another_leaves_the_composed_stream_continuous() {
    const FIRST_CLOSES: i64 = 20;
    let library = one_motion(0.02, 60);
    let mut mover = Mover::playing(&params(PERIOD, UP_NS, STOW_NS), &library);
    // Both windows name the same motion; the rows are what separate them, so the
    // two contributions are added and the first one closing leaves the second
    // riding.
    mover.schedule_playing(1, &[(0, 5, FIRST_CLOSES, 0.5, 1.0), (0, 10, 60, 0.5, 1.0)]);

    // Read the surviving row's clock a few cycles before the close, so what
    // comes after it can be compared against a clock that was already running:
    // a survivor restarted at the vacate reads a small positive number too.
    const BEFORE_CLOSE: usize = 17;
    const TOTAL: usize = 30;
    let mut cycles = mover.run(BEFORE_CLOSE);
    let clock_before = mover.cog.state_ctrl().players()[1].clock_s();
    cycles.extend(mover.run(TOTAL - BEFORE_CLOSE));
    assert_eq!(mover.cog.state_ctrl().overlays_refused(), 0);
    assert_eq!(mover.cog.state_ctrl().players_refused(), 0);

    // Past the first close and inside the second window: the row the closed
    // window held is emptied, and the surviving row's clock carried on from
    // where it was rather than being restarted with the vacate. The elapsed
    // periods are the measure, with one period of slack for where in the cycle
    // the reading lands.
    {
        let players = mover.cog.state_ctrl().players();
        assert!(
            !players[0].active(),
            "the closed window left its player behind",
        );
        assert!(players[1].active(), "the second window is still playing");
        let elapsed_s = (TOTAL - BEFORE_CLOSE - 1) as f64 * (PERIOD as f64 / 1_000_000_000.0);
        assert!(
            players[1].clock_s() >= clock_before + elapsed_s,
            "the surviving player was restarted at the close: {} s, from {} s \
             with {} s elapsed",
            players[1].clock_s(),
            clock_before,
            elapsed_s,
        );
    }

    let close = usize::try_from(FIRST_CLOSES).expect("a few cycles") - 1;
    assert_continuous_across(&cycles, close, "one window closing under another");
    assert_steps_fit(&cycles, "two windows over one base");
    assert!(reports(&cycles).is_empty(), "nothing was refused");
}

/// A disengagement takes the base and the players with it: nothing is left in
/// the slot for the next engagement's first open window to carry on from.
///
/// A player left behind would be picked up by whatever window next took its row,
/// and an ownership record left behind would have that window sampling a base
/// belonging to an engagement that ended -- a physical discontinuity on the one
/// path this cog most wants clean.
#[test]
fn a_disengagement_releases_the_base_and_the_players() {
    let library = one_motion(0.02, 60);
    let mut mover = Mover::playing(&params(PERIOD, UP_NS, STOW_NS), &library);
    mover.schedule_playing(1, &[(0, 3, 60, 1.0, 1.0)]);
    mover.run(10);
    assert!(
        mover.cog.state_ctrl().base().owned(),
        "this cog has the base while the window is open",
    );
    assert!(mover.cog.state_ctrl().players()[0].active());

    // Mid-window, the session ends the engagement.
    mover.schedule_playing_engaged(false, 2, &[(0, 3, 60, 1.0, 1.0)]);
    let quiet = mover.run(3);
    assert!(
        quiet.iter().all(|cycle| cycle.goal.is_none()),
        "a disengaged machine is commanded nothing",
    );
    assert!(
        !mover.cog.state_ctrl().base().owned(),
        "the ownership record went with the engagement",
    );
    assert!(
        !mover.cog.state_ctrl().players()[0].active(),
        "and so did the player",
    );

    // A fresh engagement over a window that is already open: the base is taken
    // over from the setpoint this engagement's arming established, and the
    // player starts afresh at the window's own offset.
    mover.schedule_playing(3, &[(0, 3, 60, 1.0, 1.0)]);
    let again = mover.run(10);
    assert!(
        again.iter().all(|cycle| cycle.goal.is_some()),
        "a fresh engagement commands again",
    );
    assert!(mover.cog.state_ctrl().base().owned());
    assert_steps_fit(&again[1..], "a base taken over by a fresh engagement");
    assert!(
        reports(&again).is_empty(),
        "a fresh engagement's first composed setpoint is one the tick will have",
    );
}

/// The same, through a fresh arming rather than a disengagement: a latching
/// fault ends the engagement, and the engagement after it starts its base from
/// the arming's own setpoint.
#[test]
fn a_fresh_arming_releases_the_base_and_the_players() {
    let library = one_motion(0.02, 60);
    let mut mover = Mover::playing(&params(PERIOD, UP_NS, STOW_NS), &library);
    mover.schedule_playing(1, &[(0, 3, 400, 1.0, 1.0)]);
    mover.run(10);
    assert!(mover.cog.state_ctrl().base().owned());

    // The reads stop for long enough that the tick gives up on them, which
    // latches: the stream stops and the state is spent.
    mover.blind = true;
    mover.run(60);
    assert!(mover.step().goal.is_none(), "the tick latched");
    mover.blind = false;

    // A fresh engagement, with the window still open. The arming is what wrote
    // the state the base is handed over from, so the record left by the
    // engagement that latched must be gone before it.
    mover.schedule_playing_engaged(false, 2, &[(0, 3, 400, 1.0, 1.0)]);
    mover.run(1);
    assert!(!mover.cog.state_ctrl().base().owned());
    mover.schedule_playing(3, &[(0, 3, 400, 1.0, 1.0)]);
    let again = mover.run(10);
    assert!(
        again.iter().all(|cycle| cycle.goal.is_some()),
        "the fresh engagement commands again",
    );
    assert!(
        mover.cog.state_ctrl().base().owned(),
        "the base is this engagement's, taken over from the arming's own setpoint",
    );
    assert_steps_fit(&again[1..], "a base taken over after a latched fault");
    assert!(
        reports(&again).is_empty(),
        "and its first composed setpoint is one the tick will have",
    );
}

/// A schedule nobody is engaged on costs the layer nothing: the library is not
/// established and the windows are not screened, because no sample would take
/// one up.
///
/// The interval between a script being accepted and an engagement concluding is
/// seconds of aux traffic long, so this is not a rare wake -- and the epoch is
/// not spent either, which is what lets the engaged wakes count the schedule's
/// refusals once each.
#[test]
fn nothing_is_screened_or_established_while_the_machine_is_disengaged() {
    let library = one_motion(0.02, 10);
    let mut mover = Mover::playing(&params(PERIOD, UP_NS, STOW_NS), &library);
    // One window naming a motion no library holds, so there is a refusal to
    // count when the screen does run.
    mover.schedule_playing_engaged(false, 7, &[(3, 2, 40, 1.0, 1.0)]);

    mover.run(5);
    assert!(
        !mover.cog.state_ctrl().library_walked(),
        "a schedule nobody is engaged on never touches the library",
    );
    assert_eq!(
        mover.cog.state_ctrl().overlays_refused(),
        0,
        "and its windows are not screened",
    );
    assert_eq!(
        mover.cog.state_ctrl().latch_epoch(),
        0,
        "nor is the schedule's one screening spent on those wakes",
    );

    // The same schedule, engaged, under the same epoch: this is the wake that
    // screens it, so this is the wake that counts what it refuses.
    mover.schedule_playing_engaged(true, 7, &[(3, 2, 40, 1.0, 1.0)]);
    mover.run(3);
    assert!(mover.cog.state_ctrl().library_walked());
    assert_eq!(
        mover.cog.state_ctrl().overlays_refused(),
        1,
        "the window naming no motion is refused once, on the wake that screened it",
    );
    assert_eq!(mover.cog.state_ctrl().latch_epoch(), 7);
}

/// A library that will not establish refuses every window the schedule carries
/// -- once, and without walking its frames again every period.
#[test]
fn a_library_that_will_not_establish_refuses_the_schedules_windows_once() {
    // A motion whose one segment names a clip the library does not have, which
    // is a structural fault every establishment of it finds.
    let library = library_naming(3, 0.02, 10);
    let mut mover = Mover::playing(&params(PERIOD, UP_NS, STOW_NS), &library);
    mover.schedule_playing(1, &[(0, 2, 40, 1.0, 1.0), (0, 5, 40, 1.0, 1.0)]);

    let cycles = mover.run(20);
    assert_eq!(
        mover.cog.state_ctrl().overlays_refused(),
        2,
        "every window the schedule carried, counted once for the schedule",
    );
    assert!(
        !mover.cog.state_ctrl().library_walked(),
        "a walk that failed established nothing",
    );
    assert!(
        mover.cog.state_ctrl().overlay_latch(),
        "and the layer is latched, so the walk is not repeated every period",
    );
    assert!(
        cycles.iter().all(|cycle| cycle.goal.is_some()),
        "the base streams alone: an overlay is presence, never safety",
    );
    assert!(reports(&cycles).is_empty(), "and nothing was raised");
}

/// A base record this build cannot read is counted and cleared, and the next
/// window opening takes the base over afresh from the tick's own setpoint.
#[test]
fn a_base_record_that_is_no_base_is_counted_and_taken_over_afresh() {
    let library = one_motion(0.02, 60);
    let mut mover = Mover::playing(&params(PERIOD, UP_NS, STOW_NS), &library);
    mover.schedule_playing(1, &[(0, 3, 60, 1.0, 1.0)]);
    mover.run(8);
    assert!(mover.cog.state_ctrl().base().owned());

    // An elapsed time that is no length of time: the record reads as bytes and
    // not as a base, which is the arm a peer built against another schema or a
    // slot nobody wrote reaches.
    mover
        .cog
        .state_ctrl_mut()
        .base_mut()
        .set_elapsed(SlotDuration::from_nanos(-1));

    let cycles = mover.run(6);
    assert_eq!(
        mover.cog.state_ctrl().refused_base(),
        1,
        "the refusal is counted where a reader can see it",
    );
    assert!(
        mover.cog.state_ctrl().base().owned(),
        "the window still covers the instant, so the base was taken over again",
    );
    assert!(
        cycles.iter().all(|cycle| cycle.goal.is_some()),
        "the stream carried on across the refusal",
    );
    assert!(reports(&cycles).is_empty(), "damaged memory is not a fault");
    assert_steps_fit(&cycles[1..], "a base taken over after a refused record");
}

/// An overlay row that does not read back as a player of the motion its window
/// names is counted, and the window starts a fresh player in it.
#[test]
fn a_player_row_that_will_not_resume_is_counted_and_started_afresh() {
    let library = one_motion(0.02, 60);
    let mut mover = Mover::playing(&params(PERIOD, UP_NS, STOW_NS), &library);
    mover.schedule_playing(1, &[(0, 3, 60, 1.0, 1.0)]);
    mover.run(8);
    assert!(mover.cog.state_ctrl().players()[0].active());

    // A player of a motion this library does not hold: the fingerprint is what
    // says the row and the window are about the same thing, and a row that
    // cannot be resumed is one the window replaces.
    mover.cog.state_ctrl_mut().players_mut()[0].set_track(7);

    let cycles = mover.run(4);
    assert_eq!(
        mover.cog.state_ctrl().players_refused(),
        1,
        "the row is counted where a reader can see it",
    );
    assert!(
        mover.cog.state_ctrl().players()[0].active(),
        "and the window plays a fresh player in it",
    );
    assert!(
        cycles.iter().all(|cycle| cycle.goal.is_some()),
        "the base streams throughout",
    );
}

// The session's cases. They drive no machine: the session decides what the
// machine is doing rather than what it holds next, so a case here publishes
// scripts and raises, runs executions at instants it chooses, and reads back the
// schedule the slot holds and the reports that went out.
//
// Two of them are about the framework rather than about the session: which
// message classes wake it, and that a wake happens with nothing arriving at all.
// Those facts are what the phase machine is built on, so they are asserted
// rather than assumed.

/// The wake floor, nanoseconds: the lapse the session's execution condition
/// states.
const LAPSE_NS: i64 = 100_000_000;

/// How long a datagram has before it goes out again, nanoseconds, and how many
/// times it does.
///
/// The numbers `cogs/session_params.textproto` states, restated here because a
/// case asserting a re-issue has to say which wake it expects one on. The
/// scenario harness checks the two statements against each other; what a case
/// here asserts is what the cog does with the numbers it was given.
const AUX_TIMEOUT_NS: i64 = 200_000_000;
const AUX_RETRIES: u32 = 3;

/// How long a stow maneuver has from the instant it opens, nanoseconds. The
/// number `cogs/session_params.textproto` states, restated here for the reason
/// the two above it are: a case asserting where a stow step ends has to say
/// which clock it was cut from.
const STOW_BUDGET_NS: i64 = 4_000_000_000;

/// How long the session waits for the driver's first sample before declaring the
/// bus failed, nanoseconds. The number `cogs/session_params.textproto` states,
/// restated here for the same reason: a case asserting when the declaration
/// lands has to say which budget it was cut from. The shipped figure covers the
/// skew between two process starts the supervisor orders in no particular way,
/// the driver's port open and its nine-write release, and its first cycle.
const STARTUP_GRACE_NS: i64 = 2_000_000_000;

/// The budget for the skew between the two process starts the supervisor orders
/// in no particular way, nanoseconds.
///
/// A budget and not a measurement: the adverse ordering it exists for -- the
/// session up first, the driver late -- has not been observed. Three
/// consecutive `make motion-run`s on `reachy00`, 2026-08-29, show the opposite
/// ordering: the control process last, by 268-276 ms behind the driver's first
/// logged line.
/// One second is ~3.6x that, deliberately wide because the term is a scheduler
/// property with no derivable ceiling and the observation bounds a healthy unit
/// in the benign direction only.
const START_SKEW_ALLOWANCE_NS: i64 = 1_000_000_000;

/// The servo-side profile the commissioning sweep writes, register units: the
/// pair `cogs/session_params.textproto` ships.
///
/// Restated here for a different reason from the three above it. Zero in either
/// register is a servo running unlimited, which is what a configuration missing
/// the lines parses to and what the session refuses to commission on, so a case
/// running on any other pair would be a case running a machine no deployment
/// ships. The scenario harness is what checks these two numbers against the
/// file.
const PROFILE_ACCELERATION: u32 = 20;
const PROFILE_VELOCITY: u32 = 50;

/// The watchdog timeout the sweep arms, in the register's 20 ms units, and for
/// the same reason as the pair above: zero is the register disabled, which the
/// session refuses to commission on.
const BUS_WATCHDOG: u32 = 10;

/// How far ahead a script may schedule anything, milliseconds: the ceiling
/// `cogs/session_params.textproto` ships.
///
/// Restated for the reason the timing figures above are: a case asserting that
/// a horizon exactly at the ceiling is taken and one millisecond past it is
/// refused has to name the ceiling it cut both scripts from.
const SCRIPT_SPAN_CAP_MS: u32 = 600_000;

/// The environment variable naming the shipped session configuration, relative
/// to the runfiles root, which is a test's working directory.
const SESSION_PARAMS_ENV: &str = "SESSION_PARAMS";

/// What the shipped configuration states for `field`.
///
/// Panics on a missing file or a missing field — either is a broken test
/// target or a name that has moved, not a case.
fn shipped_session_figure(field: &str) -> String {
    let path = std::env::var(SESSION_PARAMS_ENV).unwrap_or_else(|_| {
        panic!(
            "{SESSION_PARAMS_ENV} is unset: the test target has to name the file beside the data \
             attribute that supplies it"
        )
    });
    let text = std::fs::read_to_string(&path).unwrap_or_else(|error| {
        panic!("{SESSION_PARAMS_ENV} names {path}, which does not read: {error}")
    });
    text.lines()
        .filter_map(|line| line.split_once(':'))
        .find(|(name, _)| name.trim() == field)
        .map(|(_, value)| value.trim().to_string())
        .unwrap_or_else(|| panic!("{path} states no {field}: the name has moved"))
}

/// Every figure this file restates is the figure the deployment ships.
///
/// Without it the whole suite can stay green against a session no unit runs.
/// One figure is load-bearing across two processes: the grace has to outlast
/// everything the driver does before its first sample.
#[test]
fn the_restated_session_figures_are_the_ones_the_shipped_configuration_states() {
    for (field, restated) in [
        ("aux_timeout_ns", AUX_TIMEOUT_NS.to_string()),
        ("aux_retries", AUX_RETRIES.to_string()),
        ("sample_stale_after", "5".to_string()),
        ("startup_grace_ns", STARTUP_GRACE_NS.to_string()),
        ("stow_budget_ns", STOW_BUDGET_NS.to_string()),
        ("torque_off_confirm_budget_ns", 500_000_000_i64.to_string()),
        ("profile_acceleration", PROFILE_ACCELERATION.to_string()),
        ("profile_velocity", PROFILE_VELOCITY.to_string()),
        ("bus_watchdog", BUS_WATCHDOG.to_string()),
        ("script_span_cap_ms", SCRIPT_SPAN_CAP_MS.to_string()),
    ] {
        assert_eq!(
            shipped_session_figure(field),
            restated,
            "cogs/session_params.textproto states another {field} than the cases here run on"
        );
    }
}

/// The grace outlasts every term the driver spends before its first sample.
///
/// A relation between stated budgets, not a live measurement. Fails on a grace
/// cut below the sum or any allowance forced up past it -- most usefully, a hold
/// put back in front of the driver's first cycle, which is spent out of the
/// driver's own `STARTUP_INIT_BUDGET_NS` and cannot be accounted for honestly
/// without growing it. Both driver-side figures are the driver crate's own,
/// imported rather than restated, so the editor who grows either is in the file
/// this relation reads -- and the two periods are charged at the driver's grid
/// rather than the host's, because the alignment and the first cycle are the
/// driver's to spend.
#[test]
fn the_startup_grace_covers_the_start_skew_the_driver_init_and_the_first_cycle() {
    let covered = START_SKEW_ALLOWANCE_NS + STARTUP_INIT_BUDGET_NS + 2 * NOMINAL_CYCLE_NS;
    assert!(
        STARTUP_GRACE_NS >= covered,
        "the session's {STARTUP_GRACE_NS} ns grace is under the {covered} ns it has to cover: \
         {START_SKEW_ALLOWANCE_NS} ns of start skew, {STARTUP_INIT_BUDGET_NS} ns of driver \
         init, and two {NOMINAL_CYCLE_NS} ns cycles for grid alignment and the first cycle"
    );
}

/// The session's timing, as the config slot carries it.
fn session_params() -> SessionParamsWire {
    let mut params = SessionParamsWire::new();
    params.set_aux_timeout_ns(AUX_TIMEOUT_NS);
    params.set_aux_retries(AUX_RETRIES);
    params.set_sample_stale_after(5);
    params.set_startup_grace_ns(STARTUP_GRACE_NS);
    params.set_stow_budget_ns(STOW_BUDGET_NS);
    params.set_torque_off_confirm_budget_ns(500_000_000);
    params.set_profile_acceleration(PROFILE_ACCELERATION);
    params.set_profile_velocity(PROFILE_VELOCITY);
    params.set_bus_watchdog(u8::try_from(BUS_WATCHDOG).expect("the watchdog count is one byte"));
    params.set_script_span_cap_ms(SCRIPT_SPAN_CAP_MS);
    params
}

/// A session with its three input views sized, its timing bound, and stood up at
/// [`T0`].
///
/// The configuration is seeded after `initialize` and before the first
/// execution, the way the mover's is: a config slot is read at the top of every
/// execution, and the first one is where a session decides to start commissioning
/// the machine.
fn session() -> SessionTestWrapper {
    story_reset();
    let mut cog = SessionTestWrapper::new();
    cog.input_script_set_num_slots(4);
    cog.input_fault_set_num_slots(4);
    cog.input_aux_out_set_num_slots(4);
    cog.input_evt_set_num_slots(8);
    cog.input_readings_set_num_slots(8);
    cog.initialize(SyncTime::from_nanos(T0));
    cog.set_config_params(&session_params());
    cog
}

/// One datagram the session published, copied out of the message.
#[derive(Clone, Copy, PartialEq, Debug)]
struct Asked {
    kind: SessionCmdKindWire,
    corr: u32,
    op: AuxOpKindWire,
    id: u8,
    reg: RegIdWire,
    value_kind: ValueShapeWire,
    value: u64,
}

/// What one execution asked the driver for, or `None` where it asked nothing.
fn asked(cog: &mut SessionTestWrapper) -> Option<Asked> {
    cog.try_next_cmd().map(|msg: &SessionCmdWire| Asked {
        kind: msg.kind(),
        corr: msg.corr(),
        op: msg.txn().op(),
        id: msg.txn().id(),
        reg: msg.txn().reg(),
        value_kind: msg.txn().value_kind(),
        value: msg.txn().value(),
    })
}

/// The instant a fresh session first runs: nothing has arrived, so the wake it
/// commissions the machine on is the floor's.
const FIRST_WAKE: i64 = T0 + LAPSE_NS;

/// Run one execution at `at_ns` and answer with the datagram it published.
///
/// The driver's heartbeat is fed first, because a real one publishes one every
/// cycle and a session that has not heard from its driver declares the bus dead.
/// A case about that declaration is a case that stops calling this.
fn drive(cog: &mut SessionTestWrapper, at_ns: i64) -> Option<Asked> {
    heartbeat(cog, at_ns);
    assert!(stepped(cog, at_ns), "the session was expected to wake",);
    asked(cog)
}

/// One sample, carrying nothing the session reads but its instant.
///
/// The freshness watchdog is what reads it: what a sample proves to the session
/// is that the driver is still producing cycles, and the angles in it are the
/// two control-rate cogs' business.
fn heartbeat(cog: &mut SessionTestWrapper, at_ns: i64) {
    let mut msg = PoseSampleWire::new();
    msg.set_nominal_time(SyncTime::from_nanos(at_ns));
    msg.set_sample_time(SyncTime::from_nanos(at_ns));
    cog.publish_sample(&msg, SyncTime::from_nanos(at_ns));
}

/// An edge the driver reported, as its channel carries one.
fn event(kind: EventKindWire, at_ns: i64) -> DriverEventWire {
    let mut msg = DriverEventWire::new();
    msg.set_kind(kind);
    msg.set_time(SyncTime::from_nanos(at_ns));
    msg
}

/// One servo's health, as the driver's rotation reports it.
fn reading(id: u8, bits: u8, at_ns: i64) -> HealthReportWire {
    let mut msg = HealthReportWire::new();
    msg.set_id(id);
    msg.set_bits(bits);
    msg.set_sample_time(SyncTime::from_nanos(at_ns));
    msg
}

/// An outcome as the driver's channel carries it: a ping answered by a servo
/// that says what it is.
fn pinged(corr: u32, model: u16) -> AuxOutcomeWire {
    let mut msg = AuxOutcomeWire::new();
    msg.set_corr(corr);
    msg.set_status(AuxStatusWire::OK);
    msg.set_value_kind(ValueShapeWire::NONE);
    msg.set_model(model);
    msg
}

/// The same, resting: the phase a script is accepted in.
///
/// Written into the slot rather than reached by commissioning, which the case
/// would have to drive a bus for.
fn resting_session() -> SessionTestWrapper {
    let mut cog = session();
    cog.state_sess_mut().set_phase(SessionPhaseWire::RESTING);
    cog
}

/// One step of a script as a case states it: an offset, a length, and what it
/// asks for.
struct Step {
    after_ms: u32,
    duration_ms: u32,
    kind: StepKindWire,
    posture: PostureWire,
}

/// One overlay window of a script, likewise.
struct Overlay {
    motion_id: u16,
    after_ms: u32,
    duration_ms: u32,
}

/// A script as the channel carries it.
fn script_msg(script_id: u32, arrival_ns: i64, steps: &[Step], overlays: &[Overlay]) -> ScriptWire {
    let mut msg = ScriptWire::new();
    msg.set_script_id(script_id);
    msg.set_arrival(SyncTime::from_nanos(arrival_ns));
    {
        let mut rows = msg.steps_mut();
        rows.clear();
        for step in steps {
            let row: &mut ScriptStepWire = rows.try_grow().expect("sixteen steps is plenty");
            row.set_after_ms(step.after_ms);
            row.set_duration_ms(step.duration_ms);
            row.set_kind(step.kind);
            row.set_posture(step.posture);
        }
    }
    let mut rows = msg.overlays_mut();
    rows.clear();
    for overlay in overlays {
        let row: &mut ScriptOverlayWire = rows.try_grow().expect("four windows is plenty");
        row.set_motion_id(overlay.motion_id);
        row.set_after_ms(overlay.after_ms);
        row.set_duration_ms(overlay.duration_ms);
        row.set_gain(1.0);
        row.set_speed(1.0);
    }
    msg
}

/// A script that is no timeline: one step of no length, refused at the screen.
///
/// What a case wants when it is asserting the ring rather than the schedule: a
/// refusal leaves exactly one row and moves no phase, so a burst of them is a
/// burst of rows and nothing else.
fn no_timeline_script(script_id: u32, arrival_ns: i64) -> ScriptWire {
    script_msg(
        script_id,
        arrival_ns,
        &[Step {
            after_ms: 0,
            duration_ms: 0,
            kind: StepKindWire::BASE_KEEP,
            posture: PostureWire::STOW,
        }],
        &[],
    )
}

/// The one-step script every case that does not care about the steps uses: stand
/// up half a second after arrival, for two seconds.
fn one_step_script(script_id: u32, arrival_ns: i64) -> ScriptWire {
    script_msg(
        script_id,
        arrival_ns,
        &[Step {
            after_ms: 500,
            duration_ms: 2_000,
            kind: StepKindWire::BASE_POSTURE,
            posture: PostureWire::UP,
        }],
        &[],
    )
}

/// One published report, copied out of the message.
#[derive(Clone, Copy, PartialEq, Debug)]
struct Said {
    time_ns: i64,
    kind: ReportKindWire,
    a: u32,
    b: u32,
    detail: f64,
}

/// One row of a published story, copied out of the message.
fn read_said(row: &TimelineEntryWire) -> Said {
    Said {
        time_ns: row.time().as_nanos(),
        kind: row.kind(),
        a: row.a(),
        b: row.b(),
        detail: row.detail(),
    }
}

// The rows of the story a case has been handed already, the ones it has not, and
// the newest story whole.
//
// The session publishes its whole story every time it adds a row, so each
// message repeats what the case has already read. These hold the new rows only,
// oldest first -- which is the shape every case below is written in: one report
// a call, and nothing where there is nothing new. A case's thread is its own,
// and `session` resets them, so a second session in one case is a second story.
thread_local! {
    static UNREAD: RefCell<VecDeque<Said>> = const { RefCell::new(VecDeque::new()) };
    static STORY_READ: Cell<u64> = const { Cell::new(0) };
    static NEWEST: RefCell<Option<(u32, Vec<Said>)>> = const { RefCell::new(None) };
}

/// Forget whatever story was being read: a fresh session tells its own.
fn story_reset() {
    UNREAD.with(|unread| unread.borrow_mut().clear());
    STORY_READ.with(|read| read.set(0));
    NEWEST.with(|newest| *newest.borrow_mut() = None);
}

/// Take the rows of `message` the case has not been handed yet.
///
/// A message says how many rows the ring dropped off the front, so a row's place
/// in the whole story is its position plus that count -- which is what says
/// where the reading left off, across a ring that wrapped. A total shorter than
/// what has been read is a ring that was cleared under the reader, and the story
/// starts again from what the message holds.
fn take_unread(dropped: u32, rows: &[Said]) {
    let dropped = u64::from(dropped);
    let total = dropped + rows.len() as u64;
    STORY_READ.with(|read| {
        if total < read.get() {
            read.set(dropped);
        }
        let first = read.get().max(dropped);
        if total > first {
            let from = usize::try_from(first - dropped).expect("a story fits a ring");
            UNREAD.with(|unread| unread.borrow_mut().extend(rows[from..].iter().copied()));
            read.set(total);
        }
    });
}

/// Run one execution of the session at `at_ns`, and keep up with its story.
///
/// The report output carries what the execution that ran last put there and
/// nothing else, so an execution nobody looked at is a story nobody can read
/// afterwards -- and the story is published only by the executions that added
/// to it.
fn stepped(cog: &mut SessionTestWrapper, at_ns: i64) -> bool {
    let ran = cog.execute(SyncTime::from_nanos(at_ns));
    if let Some(msg) = cog.try_next_report() {
        let dropped = msg.dropped();
        let rows: Vec<Said> = msg.entries().iter().map(read_said).collect();
        take_unread(dropped, &rows);
        NEWEST.with(|newest| *newest.borrow_mut() = Some((dropped, rows)));
    }
    ran
}

/// The next thing the session said that this case has not read, or `None` where
/// it has read the whole story.
///
/// Does not read the output itself: [`stepped`] takes the story off the output
/// at the execution that published it, because that is the only instant it is
/// there to be taken.
fn said(_cog: &mut SessionTestWrapper) -> Option<Said> {
    UNREAD.with(|unread| unread.borrow_mut().pop_front())
}

/// Read past whatever the session has said up to here.
///
/// The story is cumulative and nothing drains it, so a case that drove the
/// session through a stretch it is not asserting about would otherwise be handed
/// those rows first. This says the case has read them: what [`said`] answers
/// next is what the session says after this call.
fn caught_up(_cog: &mut SessionTestWrapper) {
    UNREAD.with(|unread| unread.borrow_mut().clear());
}

/// The newest story the session has published, whole: what it says was dropped
/// and every row it holds.
///
/// For the cases that are about the story itself rather than about what one
/// execution said. Kept by [`stepped`] as it goes past, for the same reason the
/// unread rows are.
fn whole_story(_cog: &mut SessionTestWrapper) -> Option<(u32, Vec<Said>)> {
    NEWEST.with(|newest| newest.borrow().clone())
}

/// Run one execution at `at_ns`, asserting that it happened, and answer with
/// what it said.
///
/// The heartbeat is fed here too, for the reason [`drive`] states.
fn wake(cog: &mut SessionTestWrapper, at_ns: i64) -> Option<Said> {
    heartbeat(cog, at_ns);
    assert!(stepped(cog, at_ns), "the session was expected to wake",);
    said(cog)
}

/// Everything the session has left to say, in the order it said it.
fn everything(cog: &mut SessionTestWrapper, from_ns: i64) -> Vec<Said> {
    let mut told = Vec::new();
    let mut at = from_ns;
    while let Some(report) = wake(cog, at) {
        told.push(report);
        at += LAPSE_NS;
    }
    told
}

/// A raise as the tick's channel carries it.
fn raise(kind: FaultKindWire, joint: JointRefWire, at_ns: i64, detail: f64) -> TickFaultWire {
    let mut msg = TickFaultWire::new();
    msg.set_time(SyncTime::from_nanos(at_ns));
    msg.set_kind(kind);
    msg.set_joint(joint);
    msg.set_detail(detail);
    msg.set_count(1);
    msg
}

/// Nothing arriving still wakes the session, and only once the floor has passed.
///
/// The floor is what a decision about time having passed is made in, so a build
/// where a silent session never runs is a build where no deadline is ever
/// reached. Asserted rather than assumed: it is a fact about the framework this
/// cog's whole shape rests on.
#[test]
fn a_lapse_wakes_a_session_nothing_arrived_at() {
    let mut cog = session();

    assert!(
        !stepped(&mut cog, T0 + LAPSE_NS - 1),
        "a wake before the floor is not owed",
    );
    assert!(stepped(&mut cog, T0 + LAPSE_NS));
    assert!(
        !stepped(&mut cog, T0 + LAPSE_NS + 1),
        "and the floor is measured from the execution, not from start-up",
    );
    assert!(stepped(&mut cog, T0 + 2 * LAPSE_NS));
}

/// Either message class wakes it inside the floor, and a raise nobody has to
/// answer wakes it as surely as a script does: what the session does with a
/// message is not what decides whether it runs.
#[test]
fn a_script_and_a_raise_each_wake_it_inside_the_floor() {
    let mut cog = session();

    cog.publish_script(&one_step_script(1, T0), SyncTime::from_nanos(T0));
    assert!(stepped(&mut cog, T0 + 1));

    cog.publish_fault(
        &raise(
            FaultKindWire::COMMAND_REJECTED,
            JointRefWire::NONE,
            T0 + 2,
            0.0,
        ),
        SyncTime::from_nanos(T0 + 2),
    );
    assert!(stepped(&mut cog, T0 + 3));
}

/// A script accepted while resting becomes the schedule the session holds:
/// absolute instants off the sender's own stamp, the epoch bumped, and nothing
/// engaged -- an accepted script says what the machine is to do, not that it is
/// under command.
#[test]
fn an_accepted_script_becomes_the_schedule_the_session_holds() {
    let mut cog = resting_session();
    let arrival = T0 + LAPSE_NS;

    cog.publish_script(&one_step_script(7, arrival), SyncTime::from_nanos(arrival));
    let report = wake(&mut cog, arrival + 1).expect("an acceptance is narrated");

    assert_eq!(report.kind, ReportKindWire::SCRIPT_ACCEPTED);
    assert_eq!(report.a, 7, "the script the sender numbered");
    assert_eq!(report.b, 1, "and how many steps it asked for");
    assert_eq!(report.time_ns, arrival + 1, "reported at the wake");

    let state = cog.state_sess();
    assert_eq!(state.script_id(), 7);
    assert_eq!(state.scripts_accepted(), 1);
    assert_eq!(state.scripts_refused(), 0);
    let schedule = state.schedule();
    assert!(!schedule.engaged(), "acceptance is not engagement");
    assert_eq!(schedule.epoch(), 1, "and every change bumps the epoch");
    assert_eq!(schedule.steps().len(), 1);
    let step = schedule.steps().get(0).expect("the one step");
    assert_eq!(step.start().as_nanos(), arrival + 500_000_000);
    assert_eq!(step.end().as_nanos(), arrival + 2_500_000_000);
    assert_eq!(step.kind(), StepKindWire::BASE_POSTURE);
    assert_eq!(step.posture(), PostureWire::UP);
}

/// Two scripts in one window are answered one at a time, and the first one
/// accepted is the schedule the session holds: accepting it is the machine
/// beginning to arm, and a script arriving mid-engagement is refused rather than
/// queued or swapped in under a sequence that has started.
#[test]
fn the_first_script_accepted_in_a_window_is_the_schedule() {
    let mut cog = resting_session();

    cog.publish_script(
        &script_msg(
            1,
            T0,
            &[
                Step {
                    after_ms: 0,
                    duration_ms: 1_000,
                    kind: StepKindWire::BASE_POSTURE,
                    posture: PostureWire::UP,
                },
                Step {
                    after_ms: 1_000,
                    duration_ms: 1_000,
                    kind: StepKindWire::BASE_POSTURE,
                    posture: PostureWire::STOW,
                },
            ],
            &[Overlay {
                motion_id: 2,
                after_ms: 100,
                duration_ms: 100,
            }],
        ),
        SyncTime::from_nanos(T0),
    );
    cog.publish_script(&one_step_script(2, T0), SyncTime::from_nanos(T0));

    let first = wake(&mut cog, T0 + 1).expect("the first acceptance is narrated");
    assert_eq!(first.kind, ReportKindWire::SCRIPT_ACCEPTED);
    assert_eq!(first.a, 1);
    assert_eq!(first.b, 2, "two steps");
    let entered = wake(&mut cog, T0 + 1 + LAPSE_NS).expect("the phase it moved to");
    assert_eq!(entered.kind, ReportKindWire::PHASE_CHANGED);
    assert_eq!(entered.a, u32::from(SessionPhaseWire::ENGAGING.0));
    let second = wake(&mut cog, T0 + 1 + 2 * LAPSE_NS).expect("and the second script's answer");
    assert_eq!(second.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(second.a, 2);
    assert_eq!(second.b, u32::from(RefusalReasonWire::NOT_RESTING.0));

    let state = cog.state_sess();
    assert_eq!(state.scripts_accepted(), 1);
    assert_eq!(state.scripts_refused(), 1);
    assert_eq!(state.script_id(), 1, "the one that was taken");
    let schedule = state.schedule();
    assert_eq!(schedule.epoch(), 1, "the one acceptance bumped it");
    assert_eq!(schedule.steps().len(), 2, "the whole of what it asked for");
    assert_eq!(schedule.overlays().len(), 1);
}

/// The overlay windows cross the screen the same way, and a window is
/// independent of the steps: it may open before one and close after another.
#[test]
fn an_accepted_script_carries_its_overlay_windows() {
    let mut cog = resting_session();

    cog.publish_script(
        &script_msg(
            1,
            T0,
            &[Step {
                after_ms: 0,
                duration_ms: 1_000,
                kind: StepKindWire::BASE_POSTURE,
                posture: PostureWire::UP,
            }],
            &[Overlay {
                motion_id: 3,
                after_ms: 200,
                duration_ms: 400,
            }],
        ),
        SyncTime::from_nanos(T0),
    );
    wake(&mut cog, T0 + 1).expect("an acceptance is narrated");

    let state = cog.state_sess();
    let overlays = state.schedule().overlays();
    assert_eq!(overlays.len(), 1);
    let window = overlays.get(0).expect("the one window");
    assert_eq!(window.motion_id(), 3);
    assert_eq!(window.start().as_nanos(), T0 + 200_000_000);
    assert_eq!(window.end().as_nanos(), T0 + 600_000_000);
    assert_eq!(window.gain(), 1.0);
    assert_eq!(window.speed(), 1.0);
}

/// The answer a script gets at a machine that is doing something else: the
/// default non-resting one, which is what every phase but `active` and `parked`
/// gives. The phase table those two make is
/// `the_phases_between_the_two_that_take_a_script_still_refuse`'s.
///
/// The reason is the whole of what a sender is told, so a screen that answered
/// the wrong one would send somebody to fix a script that was fine. Each reason
/// wants its own case for that.
#[test]
fn a_script_outside_resting_is_refused_with_the_phase_it_found() {
    let mut cog = session();

    cog.publish_script(&one_step_script(1, T0), SyncTime::from_nanos(T0));
    let report = wake(&mut cog, T0 + 1).expect("a refusal is narrated");
    assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(report.b, u32::from(RefusalReasonWire::NOT_RESTING.0));
    assert_eq!(cog.state_sess().scripts_refused(), 1);
    assert_eq!(
        cog.state_sess().schedule().epoch(),
        0,
        "and nothing was written",
    );
}

#[test]
fn a_script_arriving_at_a_parked_machine_is_refused_as_parked() {
    let mut cog = session();
    cog.state_sess_mut().set_phase(SessionPhaseWire::PARKED);

    cog.publish_script(&one_step_script(1, T0), SyncTime::from_nanos(T0));
    let report = wake(&mut cog, T0 + 1).expect("a refusal is narrated");
    assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(report.b, u32::from(RefusalReasonWire::PARKED.0));
}

#[test]
fn a_script_whose_steps_go_backwards_is_refused() {
    let mut cog = resting_session();

    cog.publish_script(
        &script_msg(
            1,
            T0,
            &[
                Step {
                    after_ms: 500,
                    duration_ms: 100,
                    kind: StepKindWire::BASE_KEEP,
                    posture: PostureWire::STOW,
                },
                Step {
                    after_ms: 400,
                    duration_ms: 100,
                    kind: StepKindWire::BASE_KEEP,
                    posture: PostureWire::STOW,
                },
            ],
            &[],
        ),
        SyncTime::from_nanos(T0),
    );
    let report = wake(&mut cog, T0 + 1).expect("a refusal is narrated");
    assert_eq!(report.b, u32::from(RefusalReasonWire::BAD_TIMES.0));
    assert_eq!(
        cog.state_sess().schedule().steps().len(),
        0,
        "all-or-nothing: the first step is not kept either",
    );
}

#[test]
fn a_step_of_no_length_is_refused() {
    let mut cog = resting_session();

    cog.publish_script(
        &script_msg(
            1,
            T0,
            &[Step {
                after_ms: 0,
                duration_ms: 0,
                kind: StepKindWire::BASE_KEEP,
                posture: PostureWire::STOW,
            }],
            &[],
        ),
        SyncTime::from_nanos(T0),
    );
    let report = wake(&mut cog, T0 + 1).expect("a refusal is narrated");
    assert_eq!(report.b, u32::from(RefusalReasonWire::BAD_TIMES.0));
}

/// An index no library could hold is refused at the screen, and the largest
/// index one could hold is not. Whether *this* library holds the motion is the
/// mover's question at play time, which is why the screen's bound is the
/// capacity and not the library.
///
/// Both sides, because a bound is two claims: a screen that refused the last
/// legal motion would send somebody to fix a script that was fine.
#[test]
fn an_overlay_naming_a_motion_no_library_could_hold_is_refused() {
    let mut cog = resting_session();
    let legal = |motion_id| {
        script_msg(
            u32::from(motion_id),
            T0,
            &[Step {
                after_ms: 0,
                duration_ms: 1_000,
                kind: StepKindWire::BASE_KEEP,
                posture: PostureWire::STOW,
            }],
            &[Overlay {
                motion_id,
                after_ms: 0,
                duration_ms: 100,
            }],
        )
    };

    // The refused one first: accepting a script is the machine beginning to arm,
    // and every script after that is refused for the phase rather than for its
    // motions, which would tell this case nothing about the bound.
    cog.publish_script(&legal(32), SyncTime::from_nanos(T0));
    cog.publish_script(&legal(31), SyncTime::from_nanos(T0));

    let refused = wake(&mut cog, T0 + 1).expect("the first index no library could hold");
    assert_eq!(refused.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(refused.a, 32);
    assert_eq!(refused.b, u32::from(RefusalReasonWire::UNKNOWN_MOTION.0));
    let accepted = wake(&mut cog, T0 + 1 + LAPSE_NS).expect("and the acceptance after it");
    assert_eq!(accepted.kind, ReportKindWire::SCRIPT_ACCEPTED);
    assert_eq!(accepted.a, 31, "the last index a library could hold");

    let state = cog.state_sess();
    assert_eq!(state.scripts_accepted(), 1);
    assert_eq!(state.scripts_refused(), 1);
    let overlays = state.schedule().overlays();
    assert_eq!(overlays.len(), 1, "and the accepted script is what stands");
    assert_eq!(
        overlays.get(0).expect("the one window").motion_id(),
        31,
        "the refusal wrote nothing over it",
    );
}

/// A datagram that is no script at all is refused and narrated: a sender whose
/// bytes this build cannot read has asked for something and is owed an answer.
#[test]
fn a_datagram_that_is_no_script_is_refused_as_undecodable() {
    let mut cog = resting_session();

    let mut msg = one_step_script(9, T0);
    msg.steps_mut()
        .get_mut(0)
        .expect("the one step")
        .set_kind(StepKindWire(99));
    cog.publish_script(&msg, SyncTime::from_nanos(T0));

    let report = wake(&mut cog, T0 + 1).expect("a refusal is narrated");
    assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(report.a, 0, "the id is in the bytes that would not read");
    assert_eq!(report.b, u32::from(RefusalReasonWire::UNDECODABLE.0));
    assert_eq!(cog.state_sess().undecodable_inbound(), 1);
    assert_eq!(cog.state_sess().scripts_refused(), 1);
}

/// A burst of reports goes out in one story, oldest first: the ring is what
/// carries the order, and the message carries the ring.
///
/// A burst of refusals rather than of acceptances, because accepting one is the
/// machine beginning to arm and every script after it is answered against that.
/// What is asserted is the order the story holds, which is the same whatever the
/// rows say.
#[test]
fn a_burst_of_reports_goes_out_in_one_story_oldest_first() {
    let mut cog = resting_session();

    for nth in 1..=3u32 {
        cog.publish_script(&no_timeline_script(nth, T0), SyncTime::from_nanos(T0));
    }
    let first = wake(&mut cog, T0 + 1).expect("the first of the three");
    assert_eq!(first.a, 1);
    assert_eq!(cog.state_sess().scripts_refused(), 3, "all three screened");
    assert_eq!(
        cog.state_sess().reports_published(),
        1,
        "and one story carried all three away",
    );

    let second = said(&mut cog).expect("the second");
    assert_eq!(second.a, 2);
    let third = said(&mut cog).expect("the third");
    assert_eq!(third.a, 3);
    assert!(said(&mut cog).is_none(), "and the story is read out");
    assert!(
        wake(&mut cog, T0 + 1 + LAPSE_NS).is_none(),
        "a wake with nothing to add publishes nothing",
    );
    assert_eq!(cog.state_sess().reports_published(), 1);
    assert_eq!(cog.state_sess().reports_narrated(), 3);
    assert_eq!(cog.state_sess().reports_dropped(), 0);
}

/// A raise the classifier answers with something is recorded as a fault, at the
/// instant the raise names rather than the wake that read it.
#[test]
fn a_raise_that_asks_for_a_response_is_recorded() {
    let mut cog = session();

    cog.publish_fault(
        &raise(
            FaultKindWire::HEAD_OBSTRUCTED,
            JointRefWire::LEG_2,
            T0 + 5,
            0.25,
        ),
        SyncTime::from_nanos(T0 + 5),
    );
    let report = wake(&mut cog, T0 + 6).expect("a fault is narrated");

    assert_eq!(report.kind, ReportKindWire::FAULT_RECORDED);
    assert_eq!(report.a, u32::from(FaultKindWire::HEAD_OBSTRUCTED.0));
    assert_eq!(report.b, u32::from(JointRefWire::LEG_2.0));
    assert_eq!(report.detail, 0.25);
    assert_eq!(
        report.time_ns,
        T0 + 5,
        "the raise's instant, not the wake's"
    );
    assert_eq!(cog.state_sess().faults_recorded(), 1);
}

/// A raise nothing is answered with is not a fault of the machine and is not
/// narrated: it already stands on the channel that carried it, and a timeline
/// repeating it would be a second copy of that channel.
#[test]
fn a_raise_about_a_plan_is_not_recorded() {
    let mut cog = session();

    for kind in [
        FaultKindWire::COMMAND_REJECTED,
        FaultKindWire::MOVE_ABORTED_STEP,
        FaultKindWire::MOVE_ABORTED_ENVELOPE,
    ] {
        cog.publish_fault(
            &raise(kind, JointRefWire::NONE, T0 + 5, 0.0),
            SyncTime::from_nanos(T0 + 5),
        );
    }
    assert!(
        wake(&mut cog, T0 + 6).is_none(),
        "nothing to say about a refused plan",
    );
    assert_eq!(cog.state_sess().faults_recorded(), 0);
}

/// Bytes that describe no raise are counted where every other unreadable
/// datagram is: a tick built against a newer vocabulary than this binary must
/// not read as a tick that raised nothing.
#[test]
fn a_datagram_that_is_no_raise_is_counted() {
    let mut cog = session();

    let mut msg = raise(
        FaultKindWire::HEAD_OBSTRUCTED,
        JointRefWire::NONE,
        T0 + 5,
        0.0,
    );
    msg.set_kind(FaultKindWire(99));
    cog.publish_fault(&msg, SyncTime::from_nanos(T0 + 5));

    assert!(wake(&mut cog, T0 + 6).is_none());
    assert_eq!(cog.state_sess().undecodable_inbound(), 1);
    assert_eq!(cog.state_sess().faults_recorded(), 0);
}

/// A slot this build cannot read is counted, and the machine it described is let
/// go of and latched.
///
/// The slot is the whole record of a machine that may be under command: which
/// script is running, whether torque was written, what release is still owed,
/// what maneuver is being carried out. A reading that came back as nothing says
/// none of it, and starting a fresh session over the top would arm a machine
/// that may already be armed and streamed to. So the machine is commanded limp,
/// the tick is told nobody is engaged, and the phase latches -- which is what a
/// script arriving afterwards is refused as.
#[test]
fn a_slot_that_is_no_session_is_answered_by_letting_go_and_parking() {
    let mut cog = resting_session();
    cog.state_sess_mut().set_phase(SessionPhaseWire(99));

    cog.publish_script(&one_step_script(4, T0), SyncTime::from_nanos(T0));
    heartbeat(&mut cog, T0 + 1);
    assert!(stepped(&mut cog, T0 + 1));

    assert_eq!(cog.state_sess().refused_state(), 1);
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::PARKED,
        "nothing engages a machine an operator has not seen",
    );
    assert!(
        cog.state_sess().torque_off_pending(),
        "and the release stands until the driver confirms it",
    );
    let asked = asked(&mut cog).expect("the wake commanded the release");
    assert_eq!(asked.kind, SessionCmdKindWire::TORQUE_OFF_NOW);
    assert_eq!(
        publishes(&mut cog),
        vec![Published {
            engaged: false,
            epoch: 1,
            steps: 0,
        }],
        "the tick is told nobody is running a schedule, under a fresh epoch",
    );

    let mut told: Vec<Said> = said(&mut cog).into_iter().collect();
    told.extend(everything(&mut cog, T0 + 1 + LAPSE_NS));
    assert_eq!(
        kinds(&told),
        vec![
            ReportKindWire::PHASE_CHANGED,
            ReportKindWire::SCHEDULE_PUBLISHED,
            ReportKindWire::SCRIPT_REFUSED,
        ],
        "the park, the schedule it published, and the script it could not take: \
         {told:?}",
    );
    assert_eq!(told[0].a, u32::from(SessionPhaseWire::PARKED.0));
    assert_eq!(told[2].b, u32::from(RefusalReasonWire::PARKED.0));
}

/// An instant this system's clock does not have is refused, not clamped: a
/// script asking to run past the end of the clock is a request nobody can carry
/// out, and moving it to an instant the clock does have would run a script
/// nobody asked for. Both ends of a span are checked, because a start that fits
/// says nothing about the end after it.
#[test]
fn an_instant_the_clock_does_not_have_is_refused() {
    let mut cog = resting_session();
    let one_step = |script_id, arrival_ns, after_ms, duration_ms| {
        script_msg(
            script_id,
            arrival_ns,
            &[Step {
                after_ms,
                duration_ms,
                kind: StepKindWire::BASE_POSTURE,
                posture: PostureWire::UP,
            }],
            &[],
        )
    };

    // The offset runs off the end of the clock.
    cog.publish_script(&one_step(1, i64::MAX, 1, 1_000), SyncTime::from_nanos(T0));
    // And here the start lands on the last instant there is, so the duration is
    // what does not fit.
    cog.publish_script(
        &one_step(2, i64::MAX - 1_000_000, 1, 1),
        SyncTime::from_nanos(T0),
    );

    for nth in 1..=2u32 {
        let report =
            wake(&mut cog, T0 + 1 + i64::from(nth) * LAPSE_NS).expect("a refusal is narrated");
        assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED);
        assert_eq!(report.a, nth);
        assert_eq!(
            report.b,
            u32::from(RefusalReasonWire::BAD_TIMES.0),
            "an instant the clock does not have is no timeline",
        );
    }
    let state = cog.state_sess();
    assert_eq!(state.scripts_refused(), 2);
    assert_eq!(state.schedule().epoch(), 0, "and nothing was written");
    assert_eq!(state.schedule().steps().len(), 0);
}

/// A script asking for nothing is accepted, and what it asks for is a schedule
/// of nothing: the screen's business is whether a request can be carried out,
/// and a request for no steps and no windows can.
///
/// Pinned because the consequence lands elsewhere -- a session engaged against
/// an empty schedule has nothing to wait for and ends as soon as it begins --
/// and the phase machine that has to answer for that should find the choice
/// recorded rather than infer it.
#[test]
fn a_script_asking_for_nothing_is_accepted_as_a_schedule_of_nothing() {
    let mut cog = resting_session();

    cog.publish_script(&script_msg(5, T0, &[], &[]), SyncTime::from_nanos(T0));
    let report = wake(&mut cog, T0 + 1).expect("an acceptance is narrated");

    assert_eq!(report.kind, ReportKindWire::SCRIPT_ACCEPTED);
    assert_eq!(report.a, 5);
    assert_eq!(report.b, 0, "no steps is what it asked for");
    let state = cog.state_sess();
    assert_eq!(state.scripts_accepted(), 1);
    assert_eq!(state.script_id(), 5);
    let schedule = state.schedule();
    assert_eq!(schedule.epoch(), 1, "an acceptance is a change either way");
    assert_eq!(schedule.steps().len(), 0);
    assert_eq!(schedule.overlays().len(), 0);
}

/// An overlay's weights cross the screen exactly as they arrived, a number that
/// is no number included: the session holds no library and plans no motion, and
/// the screen that owes the refusal is the mover's at play time, where the
/// weights become a composed setpoint.
///
/// The case exists to say where that burden sits. A schedule slot carrying a
/// gain of no number is not a bug of this cog; a mover that read one and
/// composed with it would be.
#[test]
fn overlay_weights_cross_the_screen_as_they_arrived() {
    let mut cog = resting_session();

    let mut msg = script_msg(
        1,
        T0,
        &[Step {
            after_ms: 0,
            duration_ms: 1_000,
            kind: StepKindWire::BASE_KEEP,
            posture: PostureWire::STOW,
        }],
        &[Overlay {
            motion_id: 1,
            after_ms: 0,
            duration_ms: 100,
        }],
    );
    {
        let mut rows = msg.overlays_mut();
        let row = rows.get_mut(0).expect("the one window");
        row.set_gain(f64::NAN);
        row.set_speed(-1.0);
    }
    cog.publish_script(&msg, SyncTime::from_nanos(T0));

    let report = wake(&mut cog, T0 + 1).expect("an acceptance is narrated");
    assert_eq!(report.kind, ReportKindWire::SCRIPT_ACCEPTED);
    let state = cog.state_sess();
    let window = state
        .schedule()
        .overlays()
        .get(0)
        .expect("the one window")
        .clone();
    assert!(
        window.gain().is_nan(),
        "the bits arrived and the bits are what the slot holds: the mover owes \
         this window its refusal",
    );
    assert_eq!(window.speed(), -1.0);
}

/// A ring with nowhere left to put a report loses its oldest one and says so:
/// the story is bounded and a run that narrates past the bound keeps its newest
/// rows, so the count of what went is the only evidence the older ones were
/// ever told.
#[test]
fn a_full_ring_drops_its_oldest_report_and_counts_it() {
    let mut cog = resting_session();
    assert_eq!(
        TIMELINE_LEN, 64,
        "the arithmetic below is this ring's length",
    );

    // Refusals, so that each script leaves exactly one row: an acceptance is
    // also the machine beginning to arm, which is a second row and a phase that
    // answers every script after it.
    //
    // Twenty wakes of four screenings each: eighty rows into a ring of
    // sixty-four, so the sixteen oldest are dropped off the front.
    for wake_nth in 0..20i64 {
        let at = T0 + 1 + wake_nth * LAPSE_NS;
        for nth in 1..=4u32 {
            let script_id = u32::try_from(wake_nth).expect("twenty fits") * 4 + nth;
            cog.publish_script(&no_timeline_script(script_id, T0), SyncTime::from_nanos(T0));
        }
        assert!(stepped(&mut cog, at), "every wake ran");
    }

    let (dropped, story) = whole_story(&mut cog).expect("the story went out");
    let state = cog.state_sess();
    assert_eq!(state.scripts_refused(), 80, "all eighty were screened");
    assert_eq!(state.reports_narrated(), 80, "and each left a row");
    assert_eq!(state.reports_published(), 20, "one story a wake");
    assert_eq!(
        state.reports_dropped(),
        16,
        "and the sixteen the full ring could not keep",
    );
    assert_eq!(
        dropped, 16,
        "which the message says, because a reader cannot work it out from the rows",
    );
    let ids: Vec<u32> = story.iter().map(|said| said.a).collect();
    assert_eq!(
        ids,
        (17..=80).collect::<Vec<u32>>(),
        "and the newest ring's worth is what it carries, oldest first",
    );
}

/// What a story says was dropped is what the ring in front of the reader lost,
/// and not what the run has lost all told. The two are different numbers exactly
/// across a clear: the session keeps a lifetime count for its signal, and a ring
/// cleared for damage has dropped nothing of the story it now holds. A reader of
/// the message can only be told about the rows it was handed.
#[test]
fn a_story_says_what_its_own_ring_dropped_and_not_what_the_run_has() {
    let mut cog = resting_session();
    {
        let slot = cog.state_sess_mut();
        // Sixteen rows lost earlier in the run, and a head no sequence of
        // appends produces: the next execution clears the ring under itself.
        slot.set_reports_dropped(16);
        slot.set_timeline_head(TIMELINE_LEN + 8);
    }

    cog.publish_script(&no_timeline_script(3, T0), SyncTime::from_nanos(T0));
    assert!(stepped(&mut cog, T0 + 1), "the wake ran");

    let (dropped, story) = whole_story(&mut cog).expect("the refusal still goes out");
    assert_eq!(story.len(), 1, "the one row the cleared ring holds");
    assert_eq!(
        dropped, 0,
        "the ring the message describes has dropped none of it",
    );
    assert_eq!(
        cog.state_sess().reports_dropped(),
        16,
        "and the run's own total is untouched by the clear",
    );
}

/// A timeline describing a ring this build does not have is the slot gone wrong,
/// and the execution that finds it still gets its own story told: the ring is
/// cleared and the report that could not be placed is written into the empty
/// one, because what that execution did is worth more than the rows it cannot
/// read.
#[test]
fn a_timeline_this_build_cannot_read_is_cleared_and_the_execution_still_speaks() {
    let mut cog = resting_session();
    {
        let slot = cog.state_sess_mut();
        // More appended than the ring ever held, with nothing dropped: no
        // sequence of appends produces this, so it is memory nobody wrote.
        slot.set_timeline_head(TIMELINE_LEN + 8);
    }

    cog.publish_script(&no_timeline_script(3, T0), SyncTime::from_nanos(T0));
    let report = wake(&mut cog, T0 + 1).expect("the refusal still goes out");

    assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(report.a, 3);
    let state = cog.state_sess();
    assert_eq!(state.refused_state(), 1, "counted where a bad slot is");
    assert_eq!(state.reports_published(), 1);
    assert_eq!(
        state.timeline_head(),
        1,
        "one report in a ring that was cleared under it",
    );
    assert_eq!(
        state.timeline_dropped(),
        0,
        "and the clear took that with it"
    );
}

/// A row inside a story of legal length that names no report is the same damage,
/// found half-way through writing the message out.
///
/// The other damaged shape: the head says a number of rows the ring could hold,
/// and one of the rows it points at holds nothing. It is not the head that is
/// refused but the row, so the refusal lands after part of the story has been
/// written into the output. What that half-written message must not do is go
/// out: it is published on a persistent channel, so the wake cog and the
/// analyzer would read a truncated story as the whole of the session's
/// narration.
#[test]
fn a_story_with_a_row_that_names_no_report_publishes_nothing() {
    let mut cog = resting_session();
    {
        let slot = cog.state_sess_mut();
        // One row narrated, into a ring whose rows are all still cleared: the
        // head is a number a run reaches, and the row under it is not.
        slot.set_timeline_head(1);
    }

    cog.publish_script(&no_timeline_script(3, T0), SyncTime::from_nanos(T0));
    assert!(stepped(&mut cog, T0 + 1), "the wake ran");

    assert!(
        whole_story(&mut cog).is_none(),
        "half a story is not published as a whole one",
    );
    let state = cog.state_sess();
    assert_eq!(state.refused_state(), 1, "counted where a bad slot is");
    assert_eq!(state.reports_published(), 0);
    assert_eq!(
        (state.timeline_head(), state.timeline_dropped()),
        (0, 0),
        "the ring is cleared under the execution that found it",
    );

    // And the session is still running: the next report goes out normally,
    // which is what says the clear left a working ring.
    cog.publish_script(&no_timeline_script(6, T0), SyncTime::from_nanos(T0));
    let report = wake(&mut cog, T0 + LAPSE_NS).expect("the next report goes out");
    assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(report.a, 6);
}

/// The same damage found with nothing to say: the ring is cleared and counted
/// and nothing is published, because there is no row this cog could claim to be
/// publishing. The next report goes out normally, which is what says the clear
/// left a working ring rather than a wedged one.
#[test]
fn a_timeline_this_build_cannot_read_publishes_nothing() {
    let mut cog = resting_session();
    {
        let slot = cog.state_sess_mut();
        slot.set_timeline_head(TIMELINE_LEN + 8);
    }

    assert!(
        wake(&mut cog, T0 + LAPSE_NS).is_none(),
        "a report nobody can read is not published as one",
    );
    assert_eq!(cog.state_sess().refused_state(), 1);
    assert_eq!(cog.state_sess().reports_published(), 0);

    cog.publish_script(&one_step_script(6, T0), SyncTime::from_nanos(T0));
    let report = wake(&mut cog, T0 + 2 * LAPSE_NS).expect("and the next one goes out");
    assert_eq!(report.a, 6);
    let state = cog.state_sess();
    assert_eq!(state.reports_published(), 1);
    assert_eq!(state.refused_state(), 1, "the damage was found once");
}

/// A servo profile of zero is refused before anything is commissioned.
///
/// The pair lives in two configuration fields whose absence parses to zeros.
/// Zero in those two
/// registers is a servo with no rate limit at all -- the opposite of the
/// backstop the pair is written for -- so the session stops the process at its
/// first execution, with the machine de-torqued and nothing commanded, rather
/// than commissioning a machine whose one host-independent limiter is off.
#[test]
#[should_panic(expected = "execute() failed")]
fn a_servo_profile_of_zero_is_not_a_machine_this_session_commissions() {
    let mut cog = session();
    let mut params = session_params();
    params.set_profile_velocity(0);
    cog.set_config_params(&params);
    drive(&mut cog, FIRST_WAKE);
}

// The session's bus half. Every case here drives the start-up survey, which is
// the one sequence the phase machine runs so far: the session asks, the case
// answers as a driver would, and what is asserted is the shape of the exchange
// rather than what commissioning concludes. What the survey establishes about a
// machine is `reachy-motion`'s own suite; what a whole successful survey looks
// like end to end is S3, which measures its dead-man from the end of one.

/// The first execution asks the first servo whether it is there, and the slot
/// records what the answer will have to match.
///
/// Nothing else could go first: presence is the survey's first phase, and no
/// transaction in it touches torque.
#[test]
fn the_first_wake_pings_the_first_servo_and_records_the_ask() {
    let mut cog = session();

    let asked = drive(&mut cog, FIRST_WAKE).expect("the survey's first transaction");
    assert_eq!(asked.kind, SessionCmdKindWire::AUX);
    assert_eq!(asked.op, AuxOpKindWire::PING);
    assert_eq!(asked.id, SERVO_IDS[0]);
    assert_eq!(asked.reg, RegIdWire::NONE, "a ping names no register");

    let pending = cog.state_sess().aux();
    assert!(pending.active(), "the session is waiting on it");
    assert_eq!(pending.corr(), asked.corr);
    assert_eq!(pending.op(), AuxOpKindWire::PING);
    assert_eq!(pending.id(), SERVO_IDS[0]);
    assert_eq!(
        pending.issued().as_nanos(),
        FIRST_WAKE,
        "the timeout is measured from when it went out",
    );
    assert_eq!(pending.retries(), 0);
}

/// A session that has never heard a sample asks the driver for nothing, however
/// many times it wakes. What a driver that never publishes one leads to is the
/// start-up grace, unchanged and pinned by its own case below.
///
/// The driver's loop starts on a grid instant and its sockets are bound before
/// then, so a survey issued into the gap waits in a buffer nobody is serving:
/// the delivery timeout expires, the re-issue lands beside the original in the
/// driver's first cycle, and the session parks on an answer it cannot tell from
/// a decline. The sample is the evidence that there is a loop to answer.
#[test]
fn a_session_that_has_not_heard_from_its_driver_asks_it_for_nothing() {
    let mut cog = session();

    let mut at = FIRST_WAKE;
    for _ in 0..4 {
        assert!(stepped(&mut cog, at), "the session woke");
        assert_eq!(asked(&mut cog), None, "and it asked for nothing at {at}");
        at += LAPSE_NS;
    }
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::STARTING,
        "still waiting, and nothing has failed",
    );
    assert!(
        !cog.state_sess().aux().active(),
        "nothing is outstanding to time out",
    );

    let asked = drive(&mut cog, at).expect("the survey's first transaction");
    assert_eq!(asked.op, AuxOpKindWire::PING);
    assert_eq!(asked.id, SERVO_IDS[0]);
    assert_eq!(asked.corr, 0, "the session's first number");
}

/// An answer advances the survey: the next execution asks the next servo, under
/// a fresh number, and nothing is outstanding in between.
#[test]
fn an_answer_advances_the_survey_to_the_next_servo() {
    let mut cog = session();
    let first = drive(&mut cog, FIRST_WAKE).expect("the first ping");

    cog.publish_aux_out(
        &pinged(first.corr, 1200),
        SyncTime::from_nanos(FIRST_WAKE + 2),
    );
    let second = drive(&mut cog, FIRST_WAKE + 3).expect("the next ping");

    assert_eq!(second.op, AuxOpKindWire::PING);
    assert_eq!(second.id, SERVO_IDS[1]);
    assert_ne!(
        second.corr, first.corr,
        "a fresh ask carries a fresh number, so an answer to the one before it is recognisable",
    );
    assert_eq!(cog.state_sess().aux().corr(), second.corr);
}

/// A datagram nothing answered goes out again, byte for byte, under the same
/// correlation number -- and only once the window has closed.
///
/// This is the property delivery retry rests on. The channel is memory today and
/// a socket later; a re-issue that differed in any field, the number included,
/// would be a second question to a servo that may have answered the first, and a
/// late duplicate of the answer would be unrecognisable.
#[test]
fn an_unanswered_datagram_goes_out_again_under_the_same_number() {
    let mut cog = session();
    let first = drive(&mut cog, FIRST_WAKE).expect("the first ping");

    assert_eq!(
        drive(&mut cog, FIRST_WAKE + AUX_TIMEOUT_NS),
        None,
        "the window is closed only once it has elapsed",
    );

    let again = drive(&mut cog, FIRST_WAKE + AUX_TIMEOUT_NS + 1).expect("the same datagram again");
    assert_eq!(again, first, "the same bytes, the same number");
    assert_eq!(cog.state_sess().aux_retries(), 1);
    let pending = cog.state_sess().aux();
    assert_eq!(pending.retries(), 1);
    assert_eq!(
        pending.issued().as_nanos(),
        FIRST_WAKE + AUX_TIMEOUT_NS + 1,
        "and the window is measured from the re-issue",
    );
}

/// The budget is finite: past it the transaction is given up on, said so, and
/// the sequence is handed the silence to make of what it will -- which for a
/// presence sweep is to go on to the next servo with the absent one recorded.
///
/// The classification stays in the library. What the host decides is how long to
/// keep trying to deliver; what a silence *means* is the sequencer's, which is
/// why nothing here parks a machine over one unanswered ping.
#[test]
fn a_datagram_nobody_answers_is_given_up_on_and_the_sweep_moves_on() {
    let mut cog = session();
    let first = drive(&mut cog, FIRST_WAKE).expect("the first ping");

    let mut at = FIRST_WAKE;
    for retry in 1..=AUX_RETRIES {
        at += AUX_TIMEOUT_NS + 1;
        assert_eq!(
            drive(&mut cog, at).expect("a re-issue"),
            first,
            "re-issue {retry} is the same datagram",
        );
    }

    at += AUX_TIMEOUT_NS + 1;
    let next = drive(&mut cog, at).expect("the sweep moves on");
    assert_eq!(next.op, AuxOpKindWire::PING);
    assert_eq!(next.id, SERVO_IDS[1], "the next servo is asked");
    assert_ne!(next.corr, first.corr);

    let state = cog.state_sess();
    assert_eq!(state.aux_retries(), u64::from(AUX_RETRIES));
    assert_eq!(state.aux_failures(), 1);
    assert_eq!(
        state.phase(),
        SessionPhaseWire::STARTING,
        "one silent servo is a reading, not a verdict",
    );

    let report = said(&mut cog).expect("the give-up is narrated");
    assert_eq!(report.kind, ReportKindWire::AUX_GAVE_UP);
    assert_eq!(report.a, first.corr);
    assert_eq!(report.b, u32::from(SERVO_IDS[0]));
    assert!(
        (report.detail - (f64::from(AUX_RETRIES + 1) * AUX_TIMEOUT_NS as f64 / 1e9)).abs() < 1e-9,
        "the seconds it carries are every window that was waited out: {}",
        report.detail,
    );
}

/// An answer naming a number nothing is waiting on is dropped and counted.
///
/// What produces one is a re-issue whose first datagram was answered after all:
/// two answers, one question. The second is about a transaction the sequence has
/// moved past, and feeding it back would answer the wrong question.
#[test]
fn an_answer_nothing_is_waiting_on_is_dropped() {
    let mut cog = session();
    let first = drive(&mut cog, FIRST_WAKE).expect("the first ping");

    cog.publish_aux_out(
        &pinged(first.corr.wrapping_add(7), 1200),
        SyncTime::from_nanos(FIRST_WAKE + 2),
    );
    assert_eq!(
        drive(&mut cog, FIRST_WAKE + 3),
        None,
        "the survey is still waiting on the answer it asked for",
    );

    let state = cog.state_sess();
    assert_eq!(state.aux_strays(), 1);
    assert!(state.aux().active(), "and it is still waiting on it");
    assert_eq!(state.aux().corr(), first.corr);
}

/// A busy answer parks the session, like any other transaction the driver put
/// nothing on the bus for.
///
/// This cog issues one transaction at a time and waits for its outcome, so a
/// driver holding a request under another number is a disagreement about what is
/// outstanding. Nothing is retried over it: a re-issue of a request the driver
/// is already acting on is answered by that request's own outcome, and anything
/// else here would be recovering from a refusal.
#[test]
fn a_driver_that_says_it_is_busy_parks_the_session() {
    let mut cog = session();
    let first = drive(&mut cog, FIRST_WAKE).expect("the first ping");

    let mut busy = AuxOutcomeWire::new();
    busy.set_corr(first.corr);
    busy.set_status(AuxStatusWire::BUSY);
    cog.publish_aux_out(&busy, SyncTime::from_nanos(FIRST_WAKE + 2));

    assert_eq!(
        drive(&mut cog, FIRST_WAKE + 3),
        None,
        "a refusal is not something to ask again over",
    );
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);
}

/// A cycle that turned a request away sends two outcomes at once, and the one
/// the survey is waiting on is the one it reads.
///
/// The pair arriving in one window must leave the survey moving in either
/// order, since nothing about which datagram the loop publishes first is the
/// session's to depend on.
#[test]
fn a_pair_of_outcomes_in_one_window_answers_the_one_the_survey_asked() {
    for busy_first in [false, true] {
        let mut cog = session();
        let first = drive(&mut cog, FIRST_WAKE).expect("the first ping");

        let served = pinged(first.corr, 1200);
        let mut busy = AuxOutcomeWire::new();
        busy.set_corr(first.corr.wrapping_add(1));
        busy.set_status(AuxStatusWire::BUSY);

        let at = SyncTime::from_nanos(FIRST_WAKE + 2);
        if busy_first {
            cog.publish_aux_out(&busy, at);
            cog.publish_aux_out(&served, at);
        } else {
            cog.publish_aux_out(&served, at);
            cog.publish_aux_out(&busy, at);
        }

        let second = drive(&mut cog, FIRST_WAKE + 3).expect("the next ping");
        assert_eq!(
            (second.op, second.id),
            (AuxOpKindWire::PING, SERVO_IDS[1]),
            "the served answer advanced the survey (busy published first: {busy_first})",
        );
        assert_eq!(
            cog.state_sess().phase(),
            SessionPhaseWire::STARTING,
            "and the collision the session was not waiting on parked nothing",
        );
        assert_eq!(
            cog.state_sess().aux_strays(),
            1,
            "the busy answer named a number nothing was waiting on, and is counted as one",
        );
    }
}

/// A survey no servo answers ends the process's engagement with the machine: the
/// phase latches at parked, it is said, and a script arriving afterwards is
/// refused for that reason.
///
/// Nothing was torqued to reach this, so there is no maneuver and nothing to
/// make safe -- and nothing clears it either. Only an operator restarting the
/// process does, which is what the refusal below is the visible half of.
#[test]
fn a_survey_no_servo_answers_parks_the_machine() {
    let mut cog = session();
    let mut at = FIRST_WAKE;
    let mut asked = drive(&mut cog, at).expect("the first ping");

    // Every servo asked, and every ask given up on: one issue and the whole
    // retry budget each.
    for _ in 0..JOINT_COUNT * usize::try_from(AUX_RETRIES + 1).expect("a small budget") {
        at += AUX_TIMEOUT_NS + 1;
        if let Some(next) = drive(&mut cog, at) {
            asked = next;
        }
    }
    assert_eq!(
        asked.op,
        AuxOpKindWire::PING,
        "a survey that got no further than presence asked for nothing else",
    );

    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);
    assert!(
        !cog.state_sess().aux().active(),
        "and it is waiting on nothing",
    );

    // The narration drains oldest first, so the phase change is behind the
    // give-ups the survey spent on the way here.
    let mut entered = None;
    for _ in 0..=TIMELINE_LEN {
        at += LAPSE_NS;
        if let Some(report) = wake(&mut cog, at)
            && report.kind == ReportKindWire::PHASE_CHANGED
        {
            entered = Some(report);
            break;
        }
    }
    let entered = entered.expect("the phase change is narrated");
    assert_eq!(entered.a, u32::from(SessionPhaseWire::PARKED.0));
    assert_eq!(entered.b, u32::from(SessionPhaseWire::STARTING.0));

    cog.publish_script(&one_step_script(1, at), SyncTime::from_nanos(at));
    let refusal = wake(&mut cog, at + LAPSE_NS).expect("the script is answered");
    assert_eq!(refusal.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(refusal.b, u32::from(RefusalReasonWire::PARKED.0));
}

// The ladder's cases: what the session does with evidence, and the one response
// it carries out. They drive no bus except to answer the release: what is under
// test is the classification and the commanding, and the sequences that
// establish a machine are the cases above.

/// A raise the tick published becomes a fault, and the response the library
/// classifies that condition with.
///
/// Swept over the whole vocabulary rather than over the interesting values,
/// because what is asserted is that the classification is the library's: for
/// every kind, the response narrated is `fault::response`'s answer, and a kind
/// answered with a maneuver this host does not carry out yet is recorded and
/// left at that. A kind added to the vocabulary joins the sweep with no edit
/// here.
#[test]
fn every_raise_is_recorded_and_answered_as_the_library_classifies_it() {
    for kind in FaultKindWire::VARIANTS {
        let Some(known) = kind.to_known() else {
            continue;
        };
        let response = fault::response(known);
        let mut cog = resting_session();
        cog.publish_fault(
            &raise(kind, JointRefWire::NONE, T0, 0.0),
            SyncTime::from_nanos(T0),
        );
        let told = everything(&mut cog, FIRST_WAKE);

        if matches!(response, ResponseKind::None) {
            assert!(
                told.is_empty(),
                "{kind:?} asks for nothing, so it is a remark about a command rather than a \
                 condition of the machine: {told:?}",
            );
            continue;
        }
        assert_eq!(
            told[0].kind,
            ReportKindWire::FAULT_RECORDED,
            "{kind:?} is recorded first, because the record is what the story is",
        );
        assert_eq!(told[0].a, u32::from(kind.0));

        assert_eq!(told[1].kind, ReportKindWire::RESPONSE_TAKEN);
        assert_eq!(
            told[1].a,
            u32::from(ResponseKindWire::from(response).0),
            "the response narrated for {kind:?} is the one the library gives",
        );
        assert_eq!(
            told[1].b,
            u32::from(kind.0),
            "and it names what called for it"
        );

        // The one response that answers a fault without ending the session: the
        // pair is being made to let go, the head keeps its presence, and the
        // machine stays where it was.
        if matches!(response, ResponseKind::DegradeAntennas) {
            assert_eq!(
                told.len(),
                2,
                "{kind:?} is recorded and answered, and nothing is claimed about a phase: \
                 {told:?}",
            );
            assert_eq!(cog.state_sess().phase(), SessionPhaseWire::RESTING);
            assert!(
                cog.state_sess().degrade_pending(),
                "{kind:?} left a torque-off write outstanding",
            );
            continue;
        }
        // Every other response ends the session, and where it leaves the
        // machine is the ending's disposition -- the same answer for the
        // immediate release and for a stow rung whose maneuver could not be
        // run. Nothing streams to a resting machine, so a stow is not a
        // maneuver that can be carried out here: what the condition gets is the
        // release, and the disposition still decides whether anything may
        // engage the machine again.
        let to = match ending::disposition(ending::answering(response)) {
            Disposition::Rest => SessionPhaseWire::RESTING,
            Disposition::Park => SessionPhaseWire::PARKED,
        };
        assert_eq!(
            cog.state_sess().phase(),
            to,
            "{kind:?} leaves the machine where its disposition says",
        );
        assert!(
            cog.state_sess().torque_off_pending(),
            "{kind:?} let go of the machine",
        );
        assert!(
            !cog.state_sess().winddown().active(),
            "{kind:?} began no maneuver on a machine nothing is streaming to",
        );
        if to == SessionPhaseWire::RESTING {
            assert_eq!(
                told.len(),
                2,
                "{kind:?} left the machine in the phase it was already in: {told:?}",
            );
            continue;
        }
        assert_eq!(told[2].kind, ReportKindWire::PHASE_CHANGED);
        assert_eq!(told[2].a, u32::from(SessionPhaseWire::PARKED.0));
    }
}

/// An edge the driver reported about itself is recorded as the condition it is,
/// and the release goes out on the wake the report causes.
#[test]
fn the_drivers_own_evidence_is_answered_on_the_wake_it_arrives_on() {
    let mut cog = resting_session();
    cog.publish_evt(
        &event(EventKindWire::BUS_FAILURE, T0),
        SyncTime::from_nanos(T0),
    );

    let asked = drive(&mut cog, FIRST_WAKE).expect("the release goes out at once");
    assert_eq!(asked.kind, SessionCmdKindWire::TORQUE_OFF_NOW);
    assert_eq!(
        asked.op,
        AuxOpKindWire::NONE,
        "a release names no transaction: it is the one thing the driver does without being asked \
         which register to write",
    );
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);
    assert!(cog.state_sess().torque_off_pending());

    // The wake that commanded the release also carried the first of its story
    // away, so the record is read off that execution and the rest of the
    // narration off the wakes after it.
    let recorded = said(&mut cog).expect("the record of what was answered");
    assert_eq!(recorded.kind, ReportKindWire::FAULT_RECORDED);
    assert_eq!(recorded.a, u32::from(FaultKindWire::BUS_FAILURE.0));
    assert_eq!(
        recorded.b, 0,
        "a bus that carries nothing is not about one servo",
    );
    let told = everything(&mut cog, FIRST_WAKE + LAPSE_NS);
    assert_eq!(told[0].kind, ReportKindWire::RESPONSE_TAKEN);
    assert_eq!(
        told[0].a,
        u32::from(ResponseKindWire::from(ResponseKind::ImmediateAllTorqueOffToPark).0),
    );
}

/// An edge that is not a condition of the machine is not recorded as one.
///
/// The driver reports plenty that is itself working as designed -- the
/// minimum-risk write it makes at start-up, a goal it dropped, a cycle it ran
/// late -- and a session that filed those as faults would be a session that
/// parks a healthy machine.
#[test]
fn an_edge_that_says_nothing_about_the_machine_is_not_a_fault() {
    for kind in [
        EventKindWire::STARTUP_MRC_WRITE,
        EventKindWire::CYCLE_SKIPPED,
        EventKindWire::GOAL_DROPPED_QUEUE_FULL,
        EventKindWire::GOAL_STALE_OR_OUT_OF_ORDER,
        EventKindWire::HOLD_TIMEOUT_TORQUE_OFF,
    ] {
        let mut cog = resting_session();
        cog.publish_evt(&event(kind, T0), SyncTime::from_nanos(T0));
        let told = everything(&mut cog, FIRST_WAKE);
        assert!(told.is_empty(), "{kind:?} was narrated as {told:?}");
        assert_eq!(cog.state_sess().phase(), SessionPhaseWire::RESTING);
        assert!(!cog.state_sess().torque_off_pending());
    }
}

/// A servo's error byte is recorded once, however many times the rotation reads
/// it.
///
/// The byte latches in the servo, so every pass of the driver's rotation carries
/// it again. A host that recorded each pass would fill its timeline with one
/// standing condition and count a hundred faults where there is one.
///
/// Driven on head rows of a resting machine, where the response is the release
/// and not a maneuver: what is under test is the dedup, and a stow running
/// across the assertions would be publishing schedules through them. The
/// antenna pair's own dedup rides the degrade's case below, where it is the same
/// set doing the work.
#[test]
fn a_latched_error_byte_is_recorded_once_however_often_it_is_read() {
    let mut cog = resting_session();
    let leg = SERVO_IDS[usize::from(JointRefWire::LEG_3.0) - 1];
    for pass in 0..3 {
        cog.publish_readings(
            &reading(leg, 0x20, T0 + pass),
            SyncTime::from_nanos(T0 + pass),
        );
    }
    let told = everything(&mut cog, FIRST_WAKE);

    let recorded: Vec<&Said> = told
        .iter()
        .filter(|report| report.kind == ReportKindWire::FAULT_RECORDED)
        .collect();
    assert_eq!(recorded.len(), 1, "one condition, one story: {told:?}");
    assert_eq!(
        recorded[0].a,
        u32::from(FaultKindWire::HEAD_SERVO_FAULT.0),
        "every servo that holds the head up is the head's condition",
    );
    assert_eq!(recorded[0].b, u32::from(JointRefWire::LEG_3.0));
    assert_eq!(
        cog.state_sess().faults_recorded(),
        1,
        "and the count agrees with the narration",
    );
    // The masked stow this calls for cannot be run on a machine nothing is
    // streaming to, so what the condition gets is the release and the park its
    // disposition names.
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);

    // A second servo, complaining about itself: the dedup is per joint, so the
    // condition standing on the first says nothing about this one. A session
    // that had latched one flag for the whole machine would drop it.
    let yaw = SERVO_IDS[usize::from(JointRefWire::BODY_YAW.0) - 1];
    for pass in 0..2 {
        cog.publish_readings(
            &reading(yaw, 0x20, T0 + 10 + pass),
            SyncTime::from_nanos(T0 + 10 + pass),
        );
    }
    // A reading filed under an id this bus does not have: nothing to record it
    // against, and nothing counted.
    cog.publish_readings(&reading(99, 0x20, T0 + 20), SyncTime::from_nanos(T0 + 20));

    let told = everything(&mut cog, FIRST_WAKE + LAPSE_NS);
    let recorded: Vec<&Said> = told
        .iter()
        .filter(|report| report.kind == ReportKindWire::FAULT_RECORDED)
        .collect();
    assert_eq!(recorded.len(), 1, "one new condition, one story: {told:?}");
    assert_eq!(recorded[0].a, u32::from(FaultKindWire::HEAD_SERVO_FAULT.0));
    assert_eq!(recorded[0].b, u32::from(JointRefWire::BODY_YAW.0));
    assert_eq!(
        cog.state_sess().faults_recorded(),
        2,
        "the leg's and the yaw's, and nothing for a servo off the bus",
    );

    // Further passes of either add nothing. The release the first condition
    // commanded is still unacknowledged, so what these wakes have to say is
    // about that and not about a servo.
    cog.publish_readings(&reading(leg, 0x20, T0 + 30), SyncTime::from_nanos(T0 + 30));
    cog.publish_readings(&reading(yaw, 0x20, T0 + 31), SyncTime::from_nanos(T0 + 31));
    let told = everything(&mut cog, T0 + 40 * LAPSE_NS);
    assert!(
        told.iter()
            .all(|report| report.kind == ReportKindWire::TORQUE_OFF_UNCONFIRMED),
        "nothing new was recorded about a servo: {told:?}",
    );
    assert_eq!(cog.state_sess().faults_recorded(), 2);
}

/// The input-voltage bit alone is reported by the driver and acted on by nobody.
#[test]
fn the_voltage_bit_on_its_own_is_not_a_condition() {
    let mut cog = resting_session();
    cog.publish_readings(&reading(SERVO_IDS[0], 0x01, T0), SyncTime::from_nanos(T0));
    assert!(everything(&mut cog, FIRST_WAKE).is_empty());
    assert_eq!(cog.state_sess().faults_recorded(), 0);
}

/// A driver that never publishes a sample is declared dead once the start-up
/// grace is spent, and not before.
///
/// The one condition a driver cannot report about itself. Unreachable in the
/// simulated scenarios -- the modelled driver cannot die -- so this is the whole
/// of what pins it.
#[test]
fn a_driver_that_never_produces_a_cycle_is_declared_dead_after_the_grace() {
    let mut cog = session();
    let grace_ns = STARTUP_GRACE_NS;

    // Every wake inside the grace, and no sample at any of them: nothing is
    // declared, because a process comes up with its cogs in an order nothing
    // here fixes. The grace runs from the session's own first execution, which
    // is the only instant it has to measure from.
    let mut at = FIRST_WAKE;
    while at <= FIRST_WAKE + grace_ns {
        assert!(stepped(&mut cog, at));
        assert_eq!(cog.state_sess().phase(), SessionPhaseWire::STARTING);
        at += LAPSE_NS;
    }

    assert!(stepped(&mut cog, at));
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::PARKED,
        "a driver that never started is a bus nothing can be commanded through",
    );
    assert!(cog.state_sess().torque_off_pending());
    let told = everything(&mut cog, at + LAPSE_NS);
    let kinds: Vec<ReportKindWire> = told.iter().map(|report| report.kind).collect();
    assert!(
        kinds.contains(&ReportKindWire::BUS_FAILURE_DECLARED),
        "the declaration is narrated: {told:?}",
    );
    let declared = told
        .iter()
        .find(|report| report.kind == ReportKindWire::BUS_FAILURE_DECLARED)
        .expect("the declaration");
    #[expect(
        clippy::cast_precision_loss,
        reason = "a budget in whole seconds, read as one"
    )]
    let grace_s = (grace_ns as f64) / 1e9;
    assert!(
        declared.detail > grace_s,
        "and it says how long the silence was: {}",
        declared.detail,
    );
}

/// A sample stream that stops is declared dead once the staleness window is
/// spent, measured from the freshest sample and not from start-up.
#[test]
fn a_sample_stream_that_stops_is_declared_dead_after_its_window() {
    let mut cog = session();
    let window_ns = 5 * 20_000_000;

    // Fed for a while, so the arm that applies is the staleness one: the
    // start-up grace is longer than this whole case.
    let mut at = FIRST_WAKE;
    for _ in 0..3 {
        assert!(!cog.state_sess().torque_off_pending());
        heartbeat(&mut cog, at);
        assert!(stepped(&mut cog, at));
        at += LAPSE_NS;
    }
    let last_fed = at - LAPSE_NS;
    assert_eq!(
        cog.state_sess().last_sample_time().as_nanos(),
        last_fed,
        "the anchor is the freshest sample's own instant",
    );

    // The stream stops. The window is five nominal periods, which is one wake
    // floor exactly, so the wake at the window's edge is inside it and the one
    // after that is past it.
    assert!(stepped(&mut cog, last_fed + window_ns));
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::STARTING,
        "the window closes past it and not at it",
    );
    assert!(stepped(&mut cog, last_fed + window_ns + LAPSE_NS));
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);
}

/// The release is commanded on every wake until the driver says every row let
/// go, and then it stops.
#[test]
fn a_release_is_commanded_every_wake_until_the_driver_confirms_it() {
    let mut cog = resting_session();
    cog.publish_evt(
        &event(EventKindWire::BUS_FAILURE, T0),
        SyncTime::from_nanos(T0),
    );

    let mut at = FIRST_WAKE;
    for _ in 0..4 {
        let asked = drive(&mut cog, at).expect("a wake that owes a release publishes one");
        assert_eq!(
            asked.kind,
            SessionCmdKindWire::TORQUE_OFF_NOW,
            "the same command every wake: the channel is lossy and the driver's latch is a state",
        );
        at += LAPSE_NS;
    }

    cog.publish_evt(
        &event(EventKindWire::TORQUE_OFF_CONFIRMED, at),
        SyncTime::from_nanos(at),
    );
    assert!(drive(&mut cog, at).is_none(), "a confirmed release is over");
    assert!(!cog.state_sess().torque_off_pending());
    at += LAPSE_NS;
    assert!(drive(&mut cog, at).is_none(), "and stays over");
}

/// A release the driver cannot confirm is said once per budget, and commanded
/// throughout.
///
/// The budget bounds the saying and never the commanding: nothing gates
/// de-torquing, and a release nobody can confirm is the operator's problem
/// rather than a reason to stop asking.
#[test]
fn a_release_that_goes_unconfirmed_is_said_once_per_budget() {
    let mut cog = resting_session();
    let budget_ns = 500_000_000;
    cog.publish_evt(
        &event(EventKindWire::BUS_FAILURE, T0),
        SyncTime::from_nanos(T0),
    );
    drive(&mut cog, FIRST_WAKE).expect("the release");

    // Two budgets' worth of wakes, with nothing ever confirming. The narration
    // is drained as it goes, so what is counted is how often the session said
    // it -- and every wake still publishes the release.
    let mut said_it = 0;
    let mut at = FIRST_WAKE + LAPSE_NS;
    while at <= FIRST_WAKE + 2 * budget_ns {
        heartbeat(&mut cog, at);
        assert!(stepped(&mut cog, at));
        assert_eq!(
            asked(&mut cog).expect("every wake owes the release").kind,
            SessionCmdKindWire::TORQUE_OFF_NOW,
        );
        if let Some(report) = said(&mut cog)
            && report.kind == ReportKindWire::TORQUE_OFF_UNCONFIRMED
        {
            said_it += 1;
            assert_eq!(
                report.a,
                u32::from(JointFlagsWire::from(flags::all()).0),
                "the session has had no acknowledgement for any row",
            );
            // The time spent trying, measured from the instant the release was
            // commanded: the second saying is about a machine that has been
            // unconfirmed for twice as long as the first, and a flat series of
            // identical budgets would tell an operator nothing.
            let spent = (at - FIRST_WAKE) as f64 / 1e9;
            assert!(
                (report.detail - spent).abs() < 1e-9,
                "the report says {} seconds spent and it has been {spent}",
                report.detail,
            );
        }
        at += LAPSE_NS;
    }
    assert_eq!(
        said_it, 2,
        "once per budget: a machine that never lets go says so at the rate a budget was written \
         for, not at wake rate",
    );
}

/// A sample that arrives late or out of order does not move the freshness
/// anchor backwards.
///
/// The anchor is what the staleness window is measured from, so a stale sample
/// taken as the freshest one could put the silence past the window and park a
/// perfectly healthy machine -- reached from a reordering, and by doctrine
/// unrecoverable.
#[test]
fn a_stale_sample_does_not_move_the_freshness_anchor_back() {
    let mut cog = session();
    let period = 20_000_000;

    // The wakes are the floor's, because a sample is read by this cog and never
    // triggers it: the stream runs at fifty times the rate the session wakes at.
    heartbeat(&mut cog, FIRST_WAKE);
    assert!(stepped(&mut cog, FIRST_WAKE));
    heartbeat(&mut cog, FIRST_WAKE + LAPSE_NS);
    assert!(stepped(&mut cog, FIRST_WAKE + LAPSE_NS));
    let freshest = cog.state_sess().last_sample_time().as_nanos();
    assert_eq!(freshest, FIRST_WAKE + LAPSE_NS);

    // A cycle from before the freshest one, arriving after it.
    heartbeat(&mut cog, FIRST_WAKE - 5 * period);
    assert!(stepped(&mut cog, FIRST_WAKE + 2 * LAPSE_NS));
    assert_eq!(
        cog.state_sess().last_sample_time().as_nanos(),
        freshest,
        "the anchor is the freshest instant seen and not the last one delivered",
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::STARTING,
        "and nothing was declared over a reordering",
    );
}

/// A release already commanded is the answer to a further fault: nothing is
/// re-commanded and nothing is re-narrated.
///
/// The figure the second saying would carry is the whole time the release has
/// been standing, so a host that re-commanded on every piece of evidence would
/// tell an operator the release had just started when it had been standing for
/// minutes -- while changing nothing about the machine, which is already being
/// released on every wake.
#[test]
fn a_second_fault_under_a_standing_release_re_commands_nothing() {
    let mut cog = resting_session();
    cog.publish_evt(
        &event(EventKindWire::BUS_FAILURE, T0),
        SyncTime::from_nanos(T0),
    );

    // Every wake is read for both what it asked the driver for and what it said,
    // because the narration drains one report per execution and a story left
    // unread is a story lost.
    let mut told = Vec::new();
    let mut at = FIRST_WAKE;
    let wakes = |cog: &mut SessionTestWrapper, told: &mut Vec<Said>, at: i64| {
        let asked = drive(cog, at).expect("the release is owed every wake");
        assert_eq!(asked.kind, SessionCmdKindWire::TORQUE_OFF_NOW);
        told.extend(said(cog));
    };

    wakes(&mut cog, &mut told, at);
    let commanded = cog.state_sess().torque_off_commanded().as_nanos();
    assert_eq!(commanded, FIRST_WAKE);
    for _ in 0..2 {
        at += LAPSE_NS;
        wakes(&mut cog, &mut told, at);
    }

    // A second condition, of a kind whose response is the same rung.
    at += LAPSE_NS;
    cog.publish_fault(
        &raise(
            FaultKindWire::HEAD_SERVO_FAULT,
            JointRefWire::LEG_3,
            at,
            0.0,
        ),
        SyncTime::from_nanos(at),
    );
    wakes(&mut cog, &mut told, at);
    assert_eq!(
        cog.state_sess().torque_off_commanded().as_nanos(),
        commanded,
        "the release is the one that was commanded, and it has been standing since",
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::PARKED,
        "the machine was already parked, and there is no second entry to it",
    );
    for _ in 0..3 {
        at += LAPSE_NS;
        wakes(&mut cog, &mut told, at);
    }

    let count = |wanted: ReportKindWire| told.iter().filter(|report| report.kind == wanted).count();
    assert_eq!(
        count(ReportKindWire::FAULT_RECORDED),
        2,
        "both conditions are on the record: {told:?}",
    );
    assert_eq!(
        count(ReportKindWire::RESPONSE_TAKEN),
        1,
        "one response, taken once: {told:?}",
    );
    assert_eq!(
        count(ReportKindWire::PHASE_CHANGED),
        1,
        "and one entry to the parked phase: {told:?}",
    );
}

/// A sequence that asked to be woken later is not stepped before then.
///
/// The supply gate spaces its reads out because the servos refresh their own
/// voltage reading about ten times a second, so a poll faster than that reads the
/// same number twice and spends the budget it is waiting on. A host that ignored
/// the wait would fail surveys for a reason nothing in the log would explain.
#[test]
fn a_sequence_waiting_on_a_deadline_asks_for_nothing_until_it_passes() {
    let mut cog = session();
    let deadline = FIRST_WAKE + 2 * LAPSE_NS;
    cog.state_sess_mut()
        .set_wait_deadline(SyncTime::from_nanos(deadline));

    let mut at = FIRST_WAKE;
    while at < deadline {
        assert_eq!(
            drive(&mut cog, at),
            None,
            "the sequence was stepped at {at}, before the deadline it asked for",
        );
        at += LAPSE_NS;
    }

    let asked = drive(&mut cog, deadline).expect("the first wake at the deadline steps it");
    assert_eq!(asked.op, AuxOpKindWire::PING);
    assert_eq!(asked.id, SERVO_IDS[0]);
}

/// A commissioning snapshot that does not read back as a sequence mid-flight is
/// started over rather than refused.
///
/// Nothing is torqued during a survey and the sweeps are idempotent -- they read
/// registers and write the values the machine should be holding anyway -- so
/// starting again establishes the machine, where giving up would leave a session
/// that never established anything and a machine nobody may command.
#[test]
fn a_commissioning_snapshot_that_does_not_resume_starts_the_survey_over() {
    let mut cog = session();
    let first = drive(&mut cog, FIRST_WAKE).expect("the first ping");
    cog.publish_aux_out(
        &pinged(first.corr, 1200),
        SyncTime::from_nanos(FIRST_WAKE + 1),
    );
    let second = drive(&mut cog, FIRST_WAKE + 2).expect("the second ping");
    assert_eq!(second.id, SERVO_IDS[1], "the survey is under way");

    // A cursor past the sweep it is in: the numbers read back, and no sequence
    // of steps reaches them.
    cog.state_sess_mut().commission_mut().set_cursor(200);
    cog.publish_aux_out(
        &pinged(second.corr, 1200),
        SyncTime::from_nanos(FIRST_WAKE + 3),
    );

    let again = drive(&mut cog, FIRST_WAKE + 4).expect("the survey asks for something");
    assert_eq!(
        (again.op, again.id),
        (AuxOpKindWire::PING, SERVO_IDS[0]),
        "the survey starts over from the first servo",
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::STARTING,
        "and nothing is parked over memory the host itself is the only writer of",
    );
}

// The engagement arm: the resting watch that measures where the machine is
// standing, and the engagement that takes hold of it there. Every case here
// answers as a driver's bus would -- the three registers a watch reads, and the
// two an engagement writes -- and asserts what the session does with the
// answers. What each sequence establishes about a machine is `reachy-motion`'s
// own suite; what is asserted here is the phase machine around them and the
// keep-alive that holds the driver's hold timeout off an arming.

/// What every servo's supply reads on a healthy rail, volts.
///
/// Above the library's own floor, which is what the torque-on gate judges
/// against; the case that wants a refused gate names its own number.
const NOMINAL_VOLTS: f64 = 7.4;

/// The modelled machine a case answers with.
///
/// Not a plant: nothing here moves, and every read answers with where the stow
/// pose says the machine is standing. What a case varies is the supply, whether
/// an enable write is refused -- the two things an engagement's outcome turns on
/// -- and whether a servo says it let go, which is what a release's turns on.
struct Bus {
    /// What every servo's supply reads.
    volts: f64,
    /// Which torque-enable write to answer with a servo error, counting from
    /// one, or `None` to answer them all.
    refuse_enable: Option<u32>,
    /// How many enable writes have been asked for.
    enables: u32,
    /// Which torque-off write to answer badly, counting from one, and how, or
    /// `None` to acknowledge them all. The servo it names is one whose torque
    /// this session cannot establish: a silence and a servo's own error code are
    /// two ways to be that, and neither is the row letting go.
    refuse_release: Option<(u32, AuxStatusWire)>,
    /// How many torque-off writes have been asked for.
    releases: u32,
}

impl Bus {
    /// A machine that answers everything as it should.
    fn healthy() -> Self {
        Self {
            volts: NOMINAL_VOLTS,
            refuse_enable: None,
            enables: 0,
            refuse_release: None,
            releases: 0,
        }
    }

    /// What the machine answers `asked` with.
    ///
    /// The registers a watch reads and an engagement writes, and nothing else: a
    /// transaction naming another one is a sequence doing something these cases
    /// were not written for, which is louder as a panic than as a zero.
    fn answer(&mut self, asked: &Asked) -> AuxOutcomeWire {
        let row = row_of_id(asked.id).expect("the sequences address the configured servos");
        let mut msg = AuxOutcomeWire::new();
        msg.set_corr(asked.corr);
        msg.set_status(AuxStatusWire::OK);
        msg.set_value_kind(ValueShapeWire::NONE);
        match asked.op {
            AuxOpKindWire::READ_REG => {
                let held = match asked.reg {
                    RegIdWire::PRESENT_POSITION => value::radians(
                        stow_targets(default_geometry())
                            .expect("stow is reachable")
                            .get(ROWS[row])
                            .expect("nine rows"),
                    ),
                    RegIdWire::PRESENT_INPUT_VOLTAGE => value::volts(self.volts),
                    RegIdWire::HARDWARE_ERROR_STATUS => value::u8(0),
                    other => panic!("the watch reads three registers, not {other:?}"),
                };
                msg.set_value_kind(ValueShapeWire::from(held.shape()));
                msg.set_value(held.bits());
            }
            // The pin sweep, which takes no read-back: the goal register mirrors
            // the present position while torque is off, so there is nothing to
            // answer with but the acknowledgement.
            AuxOpKindWire::WRITE_REG => match asked.reg {
                RegIdWire::GOAL_POSITION => {}
                other => panic!("the pin sweep writes one register, not {other:?}"),
            },
            AuxOpKindWire::WRITE_REG_VERIFIED => match asked.reg {
                // The value says which write this is: an arming enables torque
                // and a release writes the zero, and the two are answered by
                // different halves of a case.
                RegIdWire::TORQUE_ENABLE if asked.value == 0 => {
                    self.releases += 1;
                    if let Some((refused, status)) = self.refuse_release
                        && refused == self.releases
                    {
                        msg.set_status(status);
                    }
                }
                RegIdWire::TORQUE_ENABLE => {
                    self.enables += 1;
                    if self.refuse_enable == Some(self.enables) {
                        msg.set_status(AuxStatusWire::SERVO_ERROR);
                        msg.set_value(1);
                    }
                }
                other => panic!("the verified writes are torque, not {other:?}"),
            },
            other => panic!("neither sequence asks for {other:?}"),
        }
        msg
    }
}

/// What a run of executions came to: what was asked of the driver, and what was
/// said about it.
///
/// Both, because one report leaves per execution and the story of a run is
/// spread over the wakes that carried it: a case reading only the datagrams
/// would find the ring drained and empty by the time it looked.
struct Ran {
    /// Every datagram published, in order. The last is whatever ended the
    /// sequences -- a keep-alive or a release rather than a transaction -- where
    /// they ended by publishing something.
    asks: Vec<Asked>,
    /// Every report published, in order.
    told: Vec<Said>,
    /// Every schedule published, in order. An output slot holds one message per
    /// execution, so this is collected as the wakes happen rather than read off
    /// the cog at the end.
    published: Vec<Published>,
}

/// Answer whatever the session asks for until it stops asking for transactions.
///
/// One wake per answer, a millisecond apart: an outcome is a message the session
/// wakes on, so the sweeps run at the rate the driver answers rather than at the
/// wake floor. A cap, because a case that fails to end a sequence should say so
/// rather than run forever.
fn sweep(cog: &mut SessionTestWrapper, bus: &mut Bus, from_ns: i64, first: Asked) -> Ran {
    let mut ran = Ran {
        asks: vec![first],
        told: said(cog).into_iter().collect(),
        published: publishes(cog),
    };
    let mut asked = first;
    let mut at = from_ns;
    for _ in 0..400 {
        if asked.kind != SessionCmdKindWire::AUX {
            return ran;
        }
        let outcome = bus.answer(&asked);
        at += 1_000_000;
        cog.publish_aux_out(&outcome, SyncTime::from_nanos(at));
        at += 1_000_000;
        let published = drive(cog, at);
        ran.told.extend(said(cog));
        ran.published.extend(publishes(cog));
        match published {
            Some(next) => {
                asked = next;
                ran.asks.push(next);
            }
            None => return ran,
        }
    }
    panic!("the sequences asked for four hundred transactions and did not end");
}

/// A resting session handed one script, driven until its sequences stop asking.
fn engagement(cog: &mut SessionTestWrapper, bus: &mut Bus) -> Ran {
    cog.publish_script(&one_step_script(7, T0), SyncTime::from_nanos(T0));
    let first = drive(cog, FIRST_WAKE).expect("an accepted script starts the watch");
    sweep(cog, bus, FIRST_WAKE, first)
}

/// An accepted script takes the machine out of resting and reads where it is
/// standing, in the same wake.
///
/// Both halves matter. The phase moves because a script the machine cannot take
/// is refused, so a second script arriving now is answered rather than queued;
/// and the first read goes out on the accepting wake because the pose an
/// engagement pins is only as good as the instant it was measured at.
#[test]
fn an_accepted_script_starts_the_resting_watch_on_the_same_wake() {
    let mut cog = resting_session();
    cog.publish_script(&one_step_script(7, T0), SyncTime::from_nanos(T0));

    let asked = drive(&mut cog, FIRST_WAKE).expect("the watch's first read");
    assert_eq!(asked.kind, SessionCmdKindWire::AUX);
    assert_eq!(asked.op, AuxOpKindWire::READ_REG);
    assert_eq!(asked.reg, RegIdWire::PRESENT_POSITION);
    assert_eq!(asked.id, SERVO_IDS[0]);
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::ENGAGING,
        "the machine is being taken hold of",
    );

    let mut told: Vec<Said> = said(&mut cog).into_iter().collect();
    told.extend(everything(&mut cog, FIRST_WAKE + LAPSE_NS));
    let kinds: Vec<ReportKindWire> = told.iter().map(|report| report.kind).collect();
    assert_eq!(
        kinds,
        vec![
            ReportKindWire::SCRIPT_ACCEPTED,
            ReportKindWire::PHASE_CHANGED
        ],
        "the acceptance and the phase it moved to, in that order: {told:?}",
    );
    let entered = told.last().expect("the phase report");
    assert_eq!(entered.a, u32::from(SessionPhaseWire::ENGAGING.0));
    assert_eq!(entered.b, u32::from(SessionPhaseWire::RESTING.0));
}

/// The watch and the engagement together take the machine under command.
///
/// The transaction count is the arithmetic and not a guess: three sweeps of nine
/// for the watch -- the positions, the supply and the error bits -- and three for
/// the engagement, which writes every goal, enables every servo and reads all
/// nine back. Their order is asserted too, because pinning a goal after enabling
/// torque is the slam the order exists to prevent.
#[test]
fn the_watch_and_the_engagement_take_the_machine_active() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    let log = engagement(&mut cog, &mut bus).asks;

    assert_eq!(
        log.len(),
        6 * JOINT_COUNT,
        "three sweeps of nine each side of the torque line, and nothing after \
         them: the machine is under command and the decision tick is what \
         speaks to the driver about it",
    );
    let regs: Vec<(AuxOpKindWire, RegIdWire)> =
        log.iter().map(|asked| (asked.op, asked.reg)).collect();
    let expected = |from: usize, op: AuxOpKindWire, reg: RegIdWire| {
        for (offset, (was_op, was_reg)) in regs[from..from + JOINT_COUNT].iter().enumerate() {
            assert_eq!(
                (*was_op, *was_reg),
                (op, reg),
                "transaction {} of the sweep from {from}",
                offset + 1,
            );
            assert_eq!(log[from + offset].id, SERVO_IDS[offset], "in bus order");
        }
    };
    expected(0, AuxOpKindWire::READ_REG, RegIdWire::PRESENT_POSITION);
    expected(
        JOINT_COUNT,
        AuxOpKindWire::READ_REG,
        RegIdWire::PRESENT_INPUT_VOLTAGE,
    );
    expected(
        2 * JOINT_COUNT,
        AuxOpKindWire::READ_REG,
        RegIdWire::HARDWARE_ERROR_STATUS,
    );
    expected(
        3 * JOINT_COUNT,
        AuxOpKindWire::WRITE_REG,
        RegIdWire::GOAL_POSITION,
    );
    expected(
        4 * JOINT_COUNT,
        AuxOpKindWire::WRITE_REG_VERIFIED,
        RegIdWire::TORQUE_ENABLE,
    );
    expected(
        5 * JOINT_COUNT,
        AuxOpKindWire::READ_REG,
        RegIdWire::PRESENT_POSITION,
    );
    assert!(
        log[..6 * JOINT_COUNT]
            .iter()
            .all(|asked| asked.kind == SessionCmdKindWire::AUX),
        "no keep-alive displaced a transaction",
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::ACTIVE,
        "the machine is holding where it stood",
    );
}

/// A machine under command is not kept alive by the session.
///
/// The driver de-torques a machine nobody has spoken to for its hold timeout,
/// and while a schedule is running the thing speaking to it is the decision
/// tick: a goal per sample, every one of them liveness. So the session says
/// nothing, and the dead-man keeps the one coverage it exists for -- a stream
/// that stopped mid-schedule is a commander gone away, and the driver's timeout
/// is what answers it.
#[test]
fn a_machine_under_command_is_left_to_its_own_goal_stream() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::ACTIVE);

    let mut at = FIRST_WAKE + 400 * 1_000_000;
    for _ in 0..3 {
        at += LAPSE_NS;
        assert!(
            drive(&mut cog, at).is_none(),
            "the session asked the driver for nothing",
        );
        assert_eq!(
            cog.state_sess().phase(),
            SessionPhaseWire::ACTIVE,
            "and the schedule is still running",
        );
    }
}

/// A supply under the floor declines the script and leaves the machine resting.
///
/// The gate is judged before a single transaction, so nothing was written in
/// either direction and the machine is exactly where it was: a refused
/// engagement of an untorqued machine ends the attempt, not the process. The
/// proof of that is the next script, which is accepted.
#[test]
fn a_rail_the_gate_refuses_leaves_the_machine_resting_for_the_next_script() {
    let mut cog = resting_session();
    let mut bus = Bus {
        volts: 5.0,
        ..Bus::healthy()
    };
    let ran = engagement(&mut cog, &mut bus);

    assert_eq!(
        ran.asks.len(),
        3 * JOINT_COUNT,
        "the watch swept and the engagement wrote nothing",
    );
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::RESTING);
    assert!(
        !cog.state_sess().torque_off_pending(),
        "there is nothing to let go of",
    );

    let mut at = FIRST_WAKE + 400 * 1_000_000;
    let mut told = ran.told;
    told.extend(everything(&mut cog, at));
    let entries: Vec<(ReportKindWire, u32)> =
        told.iter().map(|report| (report.kind, report.a)).collect();
    assert!(
        entries.contains(&(
            ReportKindWire::PHASE_CHANGED,
            u32::from(SessionPhaseWire::RESTING.0)
        )),
        "the machine said it went back to resting: {told:?}",
    );

    at += 10 * LAPSE_NS;
    cog.publish_script(&one_step_script(8, at), SyncTime::from_nanos(at));
    at += LAPSE_NS;
    drive(&mut cog, at).expect("the next script is taken");
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::ENGAGING);
}

/// An engagement that stops after an enable write commands the release and
/// parks.
///
/// The one path that crosses the torque line mid-sequence. Servos may be holding
/// with nothing driving them, so the answer is that torque comes off now --
/// republished every wake until the driver confirms it, because the datagram is
/// idempotent and nothing gates de-torquing -- and the phase latches, because a
/// machine that was left holding by a sequence that failed is not one to engage
/// again without an operator.
#[test]
fn an_engagement_that_fails_under_torque_commands_the_release_and_parks() {
    let mut cog = resting_session();
    let mut bus = Bus {
        refuse_enable: Some(2),
        ..Bus::healthy()
    };
    let log = engagement(&mut cog, &mut bus).asks;

    let last = log.last().expect("the sequences asked for something");
    assert_eq!(
        last.kind,
        SessionCmdKindWire::TORQUE_OFF_NOW,
        "the wake the engagement failed on published the release: {log:?}",
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::PARKED,
        "and nothing engages until an operator has been",
    );
    assert!(cog.state_sess().torque_off_pending());

    let mut at = FIRST_WAKE + 400 * 1_000_000;
    for _ in 0..3 {
        at += LAPSE_NS;
        let asked = drive(&mut cog, at).expect("the release is owed every wake");
        assert_eq!(asked.kind, SessionCmdKindWire::TORQUE_OFF_NOW);
    }

    at += LAPSE_NS;
    cog.publish_script(&one_step_script(9, at), SyncTime::from_nanos(at));
    at += LAPSE_NS;
    // What the engagement and the release had to say is read past: the story is
    // cumulative, and what this case is about is the refusal after it.
    caught_up(&mut cog);
    drive(&mut cog, at);
    let refused = said(&mut cog).expect("the refusal is narrated on the wake that screened it");
    assert_eq!(refused.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(
        refused.b,
        u32::from(RefusalReasonWire::PARKED.0),
        "a parked machine refuses a script as parked, not as busy",
    );
}

// The group-scoped de-torque: the doctrine's one response that answers a fault
// without ending the session. The antenna pair is made to let go, one verified
// write at a time, and the head keeps its presence throughout.

/// The rows a degrade releases, as the report names them.
///
/// A function because the vocabulary's union operator is not `const fn`, which
/// is the same reason the library's own "all of them" is one.
fn antenna_pair() -> JointFlags {
    JointFlags::ANTENNA_RIGHT | JointFlags::ANTENNA_LEFT
}

/// An antenna's own trouble makes the pair let go, and the session carries on.
///
/// Both antennas complain, because the response is the pair's either way: two
/// conditions are recorded -- the dedup is per joint -- and one maneuver answers
/// them, because a drain already running is the answer to a second antenna.
/// What the machine ends up as is a session still at rest with two limp
/// antennas, and the next script is taken.
#[test]
fn an_antenna_fault_makes_the_pair_let_go_and_the_session_carries_on() {
    let mut cog = resting_session();
    let right = SERVO_IDS[usize::from(JointRefWire::ANTENNA_RIGHT.0) - 1];
    let left = SERVO_IDS[usize::from(JointRefWire::ANTENNA_LEFT.0) - 1];
    cog.publish_readings(&reading(right, 0x20, T0), SyncTime::from_nanos(T0));
    cog.publish_readings(&reading(left, 0x20, T0 + 1), SyncTime::from_nanos(T0 + 1));

    let first = drive(&mut cog, FIRST_WAKE).expect("the first row is told to let go");
    let mut bus = Bus::healthy();
    let mut ran = sweep(&mut cog, &mut bus, FIRST_WAKE, first);
    assert_eq!(
        ran.asks.len(),
        2,
        "one write per antenna and nothing else: {:?}",
        ran.asks,
    );
    for (asked, id) in ran.asks.iter().zip([right, left]) {
        assert_eq!(asked.kind, SessionCmdKindWire::AUX);
        assert_eq!(
            asked.op,
            AuxOpKindWire::WRITE_REG_VERIFIED,
            "a de-torque nobody read back is a de-torque nobody knows happened",
        );
        assert_eq!(asked.reg, RegIdWire::TORQUE_ENABLE);
        assert_eq!(asked.id, id, "the pair in bus order");
        assert_eq!(asked.value_kind, ValueShapeWire::from(value::u8(0).shape()));
        assert_eq!(asked.value, value::u8(0).bits());
    }

    // Two floors past the sweep, which answered the writes a millisecond apart:
    // the wake floor is measured from the last execution.
    ran.told
        .extend(everything(&mut cog, FIRST_WAKE + 2 * LAPSE_NS));
    let recorded: Vec<&Said> = ran
        .told
        .iter()
        .filter(|report| report.kind == ReportKindWire::FAULT_RECORDED)
        .collect();
    assert_eq!(recorded.len(), 2, "one per antenna: {:?}", ran.told);
    assert_eq!(recorded[0].b, u32::from(JointRefWire::ANTENNA_RIGHT.0));
    assert_eq!(recorded[1].b, u32::from(JointRefWire::ANTENNA_LEFT.0));

    let answered: Vec<&Said> = ran
        .told
        .iter()
        .filter(|report| report.kind == ReportKindWire::RESPONSE_TAKEN)
        .collect();
    assert_eq!(
        answered.len(),
        1,
        "the pair is one maneuver, however many of it complained: {:?}",
        ran.told,
    );
    assert_eq!(
        answered[0].a,
        u32::from(ResponseKindWire::from(ResponseKind::DegradeAntennas).0),
    );
    assert_eq!(
        answered[0].b,
        u32::from(FaultKindWire::ANTENNA_SERVO_FAULT.0),
    );

    let released = ran
        .told
        .iter()
        .find(|report| report.kind == ReportKindWire::DEGRADE_RELEASED)
        .unwrap_or_else(|| panic!("the drain says when it finished: {:?}", ran.told));
    assert_eq!(
        released.a,
        u32::from(ResponseKindWire::from(ResponseKind::DegradeAntennas).0),
        "under the response whose maneuver this is",
    );
    assert_eq!(
        released.b,
        u32::from(JointFlagsWire::from(antenna_pair()).0),
        "and it names the rows that let go",
    );

    assert_eq!(
        cog.state_sess().degrade_release(),
        JointFlagsWire::from(JointFlags::NONE),
        "nothing is still owed",
    );
    assert!(!cog.state_sess().degrade_pending());
    assert!(
        !cog.state_sess().torque_off_pending(),
        "the head was never asked to let go",
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::RESTING,
        "a pair going limp is a fault answered, not a session ended",
    );

    // And the machine is still one that takes work.
    let at = FIRST_WAKE + 20 * LAPSE_NS;
    cog.publish_script(&one_step_script(4, at), SyncTime::from_nanos(at));
    drive(&mut cog, at + LAPSE_NS).expect("the next script starts the watch");
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::ENGAGING);
}

/// A degrade under command lets the pair go and leaves the session running.
///
/// The response's whole reason for existing: a pair going limp while the head
/// keeps its presence is a fault answered, not a session ended. So nothing about
/// what the machine is under command to do changes -- no phase is entered, no
/// schedule is published, the goal stream the tick is running is untouched --
/// and underneath it two verified writes go out and the antennas let go.
#[test]
fn a_degrade_under_command_lets_the_pair_go_and_leaves_the_session_running() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    everything(&mut cog, FIRST_WAKE + 300 * 1_000_000);
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::ACTIVE);
    let under_command = stow_held(&cog);

    let right = SERVO_IDS[usize::from(JointRefWire::ANTENNA_RIGHT.0) - 1];
    let left = SERVO_IDS[usize::from(JointRefWire::ANTENNA_LEFT.0) - 1];
    let at = FIRST_WAKE + 1_000 * 1_000_000;
    cog.publish_readings(&reading(right, 0x20, at), SyncTime::from_nanos(at));
    let first = drive(&mut cog, at).expect("the first row is told to let go");
    let mut ran = sweep(&mut cog, &mut bus, at, first);

    assert_eq!(
        ran.asks.len(),
        2,
        "one verified write per antenna and nothing else: {:?}",
        ran.asks,
    );
    for (asked, id) in ran.asks.iter().zip([right, left]) {
        assert_eq!(
            (asked.op, asked.reg, asked.value, asked.id),
            (
                AuxOpKindWire::WRITE_REG_VERIFIED,
                RegIdWire::TORQUE_ENABLE,
                value::u8(0).bits(),
                id,
            ),
            "the pair in bus order, each write read back",
        );
    }
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::ACTIVE,
        "the head keeps its presence",
    );
    assert!(
        ran.published.is_empty(),
        "and nothing about what it is running changed: {:?}",
        ran.published,
    );
    assert_eq!(
        stow_held(&cog),
        under_command,
        "the schedule the tick is streaming is the one it was streaming",
    );
    assert!(
        !cog.state_sess().torque_off_pending(),
        "the head was never asked to let go",
    );

    ran.told.extend(everything(&mut cog, at + 2 * LAPSE_NS));
    let released = ran
        .told
        .iter()
        .find(|report| report.kind == ReportKindWire::DEGRADE_RELEASED)
        .unwrap_or_else(|| panic!("the drain says when it finished: {:?}", ran.told));
    assert_eq!(
        released.b,
        u32::from(JointFlagsWire::from(antenna_pair()).0),
        "naming the rows that let go",
    );
    assert_eq!(
        cog.state_sess().degrade_release(),
        JointFlagsWire::from(JointFlags::NONE),
        "and nothing is still owed",
    );
}

/// An antenna complaining mid-stow is answered without touching the maneuver.
///
/// The one exception in the order responses are selected in: every other
/// condition arriving while a machine is being carried down re-ranks the
/// maneuver, and this one is asked about first because it is scoped to the pair.
/// Both halves have to hold -- the pair lets go, and the stow keeps the clock
/// and the epoch it was opened with -- because the response the doctrine says
/// never ends a session would otherwise make a maneuver's ending worse.
#[test]
fn an_antenna_complaining_mid_stow_leaves_the_maneuver_where_it_was() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    everything(&mut cog, FIRST_WAKE + 300 * 1_000_000);

    let grabbed = FIRST_WAKE + 1_000 * 1_000_000;
    cog.publish_fault(
        &raise(
            FaultKindWire::HEAD_OBSTRUCTED,
            JointRefWire::LEG_0,
            grabbed,
            0.2,
        ),
        SyncTime::from_nanos(grabbed),
    );
    coast(&mut cog, grabbed, 4);
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::WINDING_DOWN);
    let stow = stow_held(&cog);

    let right = SERVO_IDS[usize::from(JointRefWire::ANTENNA_RIGHT.0) - 1];
    let at = grabbed + 5 * LAPSE_NS;
    cog.publish_readings(&reading(right, 0x20, at), SyncTime::from_nanos(at));
    let first = drive(&mut cog, at).expect("the antenna's write goes out mid-stow");
    let mut ran = sweep(&mut cog, &mut bus, at, first);
    assert_eq!(
        ran.asks.len(),
        2,
        "the pair's two writes, over a machine being carried down: {:?}",
        ran.asks,
    );

    assert_eq!(
        stow_held(&cog),
        stow,
        "the maneuver keeps the clock and the epoch it was opened with",
    );
    assert!(
        ran.published.is_empty(),
        "so nothing was published for it: {:?}",
        ran.published,
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::WINDING_DOWN,
        "and the machine is still being carried down",
    );

    ran.told.extend(everything(&mut cog, at + 2 * LAPSE_NS));
    let answered = ran
        .told
        .iter()
        .filter(|report| report.kind == ReportKindWire::RESPONSE_TAKEN)
        .collect::<Vec<&Said>>();
    assert_eq!(
        answered.len(),
        1,
        "the antenna was answered on its own: {:?}",
        ran.told,
    );
    assert_eq!(
        answered[0].a,
        u32::from(ResponseKindWire::from(ResponseKind::DegradeAntennas).0),
    );

    let stowed_at = at + 10 * LAPSE_NS;
    wake_folded(&mut cog, stowed_at);
    let outcome = said(&mut cog).expect("the maneuver's own record");
    assert_eq!(outcome.kind, ReportKindWire::WINDDOWN_OUTCOME);
    assert_eq!(outcome.a, u32::from(WindDownOutcomeWire::COMPLETED.0));
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::RESTING);
}

/// A row that will not let go takes the whole machine limp.
///
/// The group-scoped answer rests on the write coming back verified, so a servo
/// answering with its own error is a row whose torque this session cannot
/// establish. Asking it again is the retry the doctrine forbids; what is left is
/// the release that needs no servo's cooperation -- the driver latches it and
/// sweeps its own read-back -- and the park that says nothing engages until an
/// operator has been.
#[test]
fn an_antenna_that_will_not_let_go_takes_the_whole_machine_limp() {
    let mut cog = resting_session();
    let right = SERVO_IDS[usize::from(JointRefWire::ANTENNA_RIGHT.0) - 1];
    cog.publish_readings(&reading(right, 0x20, T0), SyncTime::from_nanos(T0));

    let first = drive(&mut cog, FIRST_WAKE).expect("the row is told to let go");
    let mut bus = Bus {
        refuse_release: Some((1, AuxStatusWire::SERVO_ERROR)),
        ..Bus::healthy()
    };
    let mut ran = sweep(&mut cog, &mut bus, FIRST_WAKE, first);

    let last = ran.asks.last().expect("something went out");
    assert_eq!(
        last.kind,
        SessionCmdKindWire::TORQUE_OFF_NOW,
        "the wake that heard the refusal commanded the release: {:?}",
        ran.asks,
    );
    assert!(cog.state_sess().torque_off_pending());
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);
    assert_eq!(
        cog.state_sess().degrade_release(),
        JointFlagsWire::from(JointFlags::NONE),
        "a machine commanded fully limp owes no group write",
    );
    assert!(!cog.state_sess().degrade_pending());

    // Two floors past the sweep, which answered the writes a millisecond apart:
    // the wake floor is measured from the last execution.
    ran.told
        .extend(everything(&mut cog, FIRST_WAKE + 2 * LAPSE_NS));
    let kinds: Vec<ReportKindWire> = ran.told.iter().map(|report| report.kind).collect();
    assert!(
        !kinds.contains(&ReportKindWire::DEGRADE_RELEASED),
        "nothing was released: {:?}",
        ran.told,
    );
    let escalation = ran
        .told
        .iter()
        .filter(|report| report.kind == ReportKindWire::FAULT_RECORDED)
        .find(|report| report.a == u32::from(FaultKindWire::TORQUE_OFF_UNCONFIRMED.0))
        .unwrap_or_else(|| panic!("the row that would not let go is recorded: {:?}", ran.told));
    assert_eq!(
        escalation.b,
        u32::from(JointRefWire::ANTENNA_RIGHT.0),
        "naming the row it is about",
    );
}

/// A de-torque write nobody answers is re-issued, and then given up on.
///
/// Delivery is the drain's own problem and it is the same problem a sequence
/// has: the datagram may be lost, so it is re-issued verbatim under the same
/// number until the budget is spent, and then the delivery is given up on and
/// narrated. The accounting is what matters here -- a lost de-torque write given
/// up on with nothing counted and nothing said would be an aux failure an
/// operator could not find, on the path that decides whether a group let go --
/// and what follows it is the release: a row this session cannot make let go is
/// answered the way every other one is.
#[test]
fn a_degrade_write_nobody_answers_is_re_issued_and_then_given_up_on() {
    let mut cog = resting_session();
    let right = SERVO_IDS[usize::from(JointRefWire::ANTENNA_RIGHT.0) - 1];
    cog.publish_readings(&reading(right, 0x20, T0), SyncTime::from_nanos(T0));

    let first = drive(&mut cog, FIRST_WAKE).expect("the row is told to let go");
    assert_eq!(
        (first.op, first.reg, first.id),
        (
            AuxOpKindWire::WRITE_REG_VERIFIED,
            RegIdWire::TORQUE_ENABLE,
            right,
        ),
    );
    let mut told: Vec<Said> = said(&mut cog).into_iter().collect();

    // Nothing answers, and every wake lands past the delivery budget.
    let mut at = FIRST_WAKE;
    for _ in 0..AUX_RETRIES {
        at += AUX_TIMEOUT_NS + 1;
        let again = drive(&mut cog, at).expect("the budget was spent, so it goes out again");
        told.extend(said(&mut cog));
        assert_eq!(
            again, first,
            "the same datagram under the same number: a driver that answered the \
             first one is being asked the same question",
        );
    }

    at += AUX_TIMEOUT_NS + 1;
    let after = drive(&mut cog, at).expect("the wake that gave up let go of the machine");
    told.extend(said(&mut cog));
    assert_eq!(
        after.kind,
        SessionCmdKindWire::TORQUE_OFF_NOW,
        "a row this session cannot make let go is a machine to stop trusting",
    );
    assert!(cog.state_sess().torque_off_pending());
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);
    assert_eq!(
        cog.state_sess().aux_retries(),
        u64::from(AUX_RETRIES),
        "every re-issue is counted",
    );
    assert_eq!(cog.state_sess().aux_failures(), 1, "and the give-up is too");

    told.extend(everything(&mut cog, at + LAPSE_NS));
    let gave_up = told
        .iter()
        .find(|report| report.kind == ReportKindWire::AUX_GAVE_UP)
        .unwrap_or_else(|| panic!("the delivery given up on is narrated: {told:?}"));
    assert_eq!(gave_up.a, first.corr, "naming the transaction it was about");
    assert_eq!(
        gave_up.b,
        u32::from(right),
        "and the servo it was addressed to"
    );
    let escalation = told
        .iter()
        .filter(|report| report.kind == ReportKindWire::FAULT_RECORDED)
        .find(|report| report.a == u32::from(FaultKindWire::TORQUE_OFF_UNCONFIRMED.0))
        .unwrap_or_else(|| panic!("the row that would not let go is recorded: {told:?}"));
    assert_eq!(escalation.b, u32::from(JointRefWire::ANTENNA_RIGHT.0));
    assert!(
        !kinds(&told).contains(&ReportKindWire::DEGRADE_RELEASED),
        "and nothing was released: {told:?}",
    );
}

/// A write the drain cannot place is a row that would not let go.
///
/// The drain takes a row out of its set by the id its own record names, so an id
/// this build has no servo for is one it can never take out. Read as a row that
/// let go, the set would never empty: the same write would go out on every wake
/// for the life of the process, holding the aux path against every sequence,
/// with nothing released, nothing refused and nothing raised. So an unplaceable
/// record is answered however the write came back, the way every other
/// de-torque this session cannot establish is -- the release that needs no
/// servo's cooperation, and the park.
#[test]
fn a_degrade_write_the_drain_cannot_place_takes_the_whole_machine_limp() {
    let mut cog = resting_session();
    let right = SERVO_IDS[usize::from(JointRefWire::ANTENNA_RIGHT.0) - 1];
    cog.publish_readings(&reading(right, 0x20, T0), SyncTime::from_nanos(T0));

    let first = drive(&mut cog, FIRST_WAKE).expect("the row is told to let go");
    // Memory gone wrong, which is the only way here: the outstanding write is
    // recorded against an id no servo of this machine has. The datagram itself
    // is answered as it went out, so what the bus says is that the write landed.
    cog.state_sess_mut().aux_mut().set_id(200);
    let mut bus = Bus::healthy();
    let ran = sweep(&mut cog, &mut bus, FIRST_WAKE, first);

    assert_eq!(
        ran.asks.len(),
        2,
        "the write, and the release that answers it -- the row is never asked \
         again: {:?}",
        ran.asks,
    );
    assert_eq!(
        ran.asks[1].kind,
        SessionCmdKindWire::TORQUE_OFF_NOW,
        "the wake that could not place the answer commanded the release",
    );
    assert!(cog.state_sess().torque_off_pending());
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);
    assert_eq!(
        cog.state_sess().degrade_release(),
        JointFlagsWire::from(JointFlags::NONE),
        "a machine commanded fully limp owes no group write",
    );
    assert!(!cog.state_sess().degrade_pending());
    assert!(
        !kinds(&ran.told).contains(&ReportKindWire::DEGRADE_RELEASED),
        "and nothing was released: {:?}",
        ran.told,
    );
}

/// A drain that settles its last row inside the release still speaks to the
/// driver.
///
/// The keep-alive rule's hardest wake: a drain owns the aux path, and the two
/// outcomes that publish no write of their own -- the row it just settled being
/// the last, and a write nobody has answered yet -- land on a machine whose goal
/// stream has stopped and whose torque is still on. A build that said nothing
/// there would leave the gap between accepted datagrams at exactly the driver's
/// hold timeout, and the dead-man would take torque off in the middle of an
/// orderly ending.
#[test]
fn a_drain_settling_its_last_row_in_the_release_still_speaks_to_the_driver() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    everything(&mut cog, FIRST_WAKE + 300 * 1_000_000);
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::ACTIVE);

    let right = SERVO_IDS[usize::from(JointRefWire::ANTENNA_RIGHT.0) - 1];
    let mut at = FIRST_WAKE + 1_000 * 1_000_000;
    cog.publish_readings(&reading(right, 0x20, at), SyncTime::from_nanos(at));
    let write = drive(&mut cog, at).expect("the first row is told to let go");

    // The wake that answers it lands past the end of the schedule, so the
    // session is over and the drain still owes the second row its write.
    let outcome = bus.answer(&write);
    at = T0 + 3_000_000_000;
    cog.publish_aux_out(&outcome, SyncTime::from_nanos(at));
    at += 1_000_000;
    let second = drive(&mut cog, at).expect("the second row's write");
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::STOPPING,
        "the schedule has run out, and the release waits for the drain",
    );
    assert_eq!(second.kind, SessionCmdKindWire::AUX);

    let outcome = bus.answer(&second);
    at += 1_000_000;
    cog.publish_aux_out(&outcome, SyncTime::from_nanos(at));
    at += 1_000_000;
    let last = drive(&mut cog, at).expect("the wake spoke to the driver");
    assert_eq!(
        last.kind,
        SessionCmdKindWire::KEEP_ALIVE,
        "nothing was asked for, so what went out is the keep-alive",
    );
    assert_eq!(
        cog.state_sess().degrade_release(),
        JointFlagsWire::from(JointFlags::NONE),
        "with the pair let go of",
    );
}

/// A degrade waits for the sequence that is holding the aux path.
///
/// One transaction is outstanding at a time, so a drain that issued its write
/// over a sequence's ask would leave that sequence waiting on a datagram that
/// never went out. The wait is bounded by the sequence's own delivery budgets,
/// and the rows stay owed throughout.
#[test]
fn a_degrade_takes_the_aux_path_only_when_no_sequence_is_asking() {
    let mut cog = resting_session();
    cog.publish_script(&one_step_script(3, T0), SyncTime::from_nanos(T0));
    let read = drive(&mut cog, FIRST_WAKE).expect("the watch's first read");
    assert_eq!(read.op, AuxOpKindWire::READ_REG);

    let right = SERVO_IDS[usize::from(JointRefWire::ANTENNA_RIGHT.0) - 1];
    cog.publish_readings(&reading(right, 0x20, T0 + 1), SyncTime::from_nanos(T0 + 1));
    // The wake the unanswered read is re-issued on, which is the first wake
    // after this one that publishes anything at all.
    let next =
        drive(&mut cog, FIRST_WAKE + AUX_TIMEOUT_NS + 1).expect("the wake publishes something");
    assert_eq!(
        next.corr, read.corr,
        "the read nobody answered is the datagram that goes out, not the drain's write",
    );
    assert!(
        !cog.state_sess().degrade_pending(),
        "and the drain has asked for nothing",
    );
    assert_eq!(
        cog.state_sess().degrade_release(),
        JointFlagsWire::from(antenna_pair()),
        "while the pair still owes its writes",
    );
}

// The session's ending: the schedule runs out, the machine is let go of, and
// the next accepted script is a new engagement. The release is the fourth
// sequence the phase machine drives, and the settle at the front of it is the
// keep-alive rule's hardest case -- two seconds under held torque with nothing
// commanding the machine at all.

/// The settle the orderly release waits out before it measures anything.
///
/// The library's own default, read rather than restated: the session builds its
/// release configuration out of the library's constants, so a case naming its
/// own number could pass against a build that waits for something else.
fn stow_dwell_ns() -> i64 {
    i64::try_from(DEFAULT_STOW_DWELL.as_nanos()).expect("two seconds fits")
}

/// Drive a session at the wake floor until it asks for a transaction, then
/// answer the sequence out.
///
/// What the wakes in between publish is collected too, because the property
/// under test through most of the ending is exactly that: a machine holding
/// torque with nothing streaming to it is spoken to on every wake. Answers with
/// what the whole stretch came to and the instant the first transaction went
/// out, which is what says how long the settle took.
fn ending(cog: &mut SessionTestWrapper, bus: &mut Bus, from_ns: i64) -> (Ran, i64) {
    let mut ran = Ran {
        asks: Vec::new(),
        told: Vec::new(),
        published: Vec::new(),
    };
    let mut at = from_ns;
    for _ in 0..200 {
        at += LAPSE_NS;
        let published = drive(cog, at);
        ran.told.extend(said(cog));
        ran.published.extend(publishes(cog));
        let Some(asked) = published else { continue };
        ran.asks.push(asked);
        if asked.kind != SessionCmdKindWire::AUX {
            continue;
        }
        let swept = sweep(cog, bus, at, asked);
        ran.asks.extend(swept.asks.into_iter().skip(1));
        ran.told.extend(swept.told);
        ran.published.extend(swept.published);
        return (ran, at);
    }
    panic!("the session never asked for a transaction");
}

/// The kinds of the reports a run published, in order.
fn kinds(told: &[Said]) -> Vec<ReportKindWire> {
    told.iter().map(|report| report.kind).collect()
}

/// A schedule that has run out ends the session: the machine is measured where
/// it stands and then let go of, one servo at a time.
///
/// The whole ordinary life of a session, from a script to the rest it ends at.
/// Three things are load-bearing here. The settle is waited out and every wake
/// of it publishes a keep-alive, which is what holds the driver's hold timeout
/// off a machine that is holding torque with nothing streaming to it -- the two
/// seconds are an order of magnitude past that timeout, so a session that went
/// quiet here would be de-torqued by the dead-man instead of by this sequence.
/// The measurement runs before the writes, because the summary's claim is where
/// the head was at the moment torque left it. And the ending is rest rather
/// than park: the next accepted script is a new engagement, which the last wake
/// of this case proves.
#[test]
fn a_schedule_that_has_run_out_ends_the_session_at_rest() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::ACTIVE);

    let armed_at = FIRST_WAKE + 400 * 1_000_000;
    let (ran, first_txn_at) = ending(&mut cog, &mut bus, armed_at);

    let stopping = ran
        .told
        .iter()
        .find(|report| {
            report.kind == ReportKindWire::PHASE_CHANGED
                && report.a == u32::from(SessionPhaseWire::STOPPING.0)
        })
        .expect("the session said it was stopping");
    assert_eq!(
        stopping.b,
        u32::from(SessionPhaseWire::ACTIVE.0),
        "it came out of being under command",
    );
    assert!(
        first_txn_at - stopping.time_ns >= stow_dwell_ns(),
        "the settle was waited out: stopping at {}, first transaction at \
         {first_txn_at}",
        stopping.time_ns,
    );

    let transactions: Vec<&Asked> = ran
        .asks
        .iter()
        .filter(|asked| asked.kind == SessionCmdKindWire::AUX)
        .collect();
    assert_eq!(
        transactions.len(),
        2 * JOINT_COUNT,
        "nine measured and nine let go: {:?}",
        ran.asks,
    );
    for (offset, asked) in transactions.iter().enumerate() {
        let row = offset % JOINT_COUNT;
        let wanted = if offset < JOINT_COUNT {
            (AuxOpKindWire::READ_REG, RegIdWire::PRESENT_POSITION, 0)
        } else {
            (
                AuxOpKindWire::WRITE_REG_VERIFIED,
                RegIdWire::TORQUE_ENABLE,
                0,
            )
        };
        assert_eq!(
            (asked.op, asked.reg, asked.value),
            wanted,
            "transaction {offset} of the release",
        );
        assert_eq!(asked.id, SERVO_IDS[row], "in bus order");
    }
    assert!(
        ran.asks
            .iter()
            .take_while(|asked| asked.kind != SessionCmdKindWire::AUX)
            .all(|asked| asked.kind == SessionCmdKindWire::KEEP_ALIVE),
        "every wake up to the first read spoke to the driver: {:?}",
        ran.asks,
    );

    // The ring outlives the sequence: one row leaves per wake, so the last
    // things a release had to say are still in it when the last transaction is
    // answered.
    let mut told = ran.told.clone();
    told.extend(everything(&mut cog, first_txn_at + 10 * LAPSE_NS));
    let ended = told
        .iter()
        .find(|report| report.kind == ReportKindWire::SESSION_ENDED)
        .expect("the session said it ended");
    assert_eq!(ended.a, 7, "the script it ended");
    assert_eq!(ended.b, 0, "nine joints measured");
    assert!(
        ended.detail < 1e-9,
        "and every one of them at its stow angle: {ended:?}",
    );
    let last = told.last().expect("the story is not empty");
    assert_eq!(
        (last.kind, last.a, last.b),
        (
            ReportKindWire::PHASE_CHANGED,
            u32::from(SessionPhaseWire::RESTING.0),
            u32::from(SessionPhaseWire::STOPPING.0),
        ),
        "the rest it ended at is the last thing it said: {told:?}",
    );
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::RESTING);
    assert!(
        !cog.state_sess().torque_off_pending(),
        "the machine let go of itself, so there is nothing to command",
    );

    let mut at = first_txn_at + 300 * LAPSE_NS;
    cog.publish_script(&one_step_script(8, at), SyncTime::from_nanos(at));
    at += LAPSE_NS;
    drive(&mut cog, at).expect("the next script is taken");
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::ENGAGING,
        "a session that ended at rest takes the next script",
    );
}

/// An overlay window that outlives the last step keeps the machine under
/// command until the motion is over.
///
/// The end test is over the whole schedule and not its steps: a window may play
/// past the step it was composed over, and a session that stopped streaming at
/// the last step's edge would cut the motion off and let go of a machine that
/// was still being asked to move.
#[test]
fn a_window_outliving_the_last_step_keeps_the_machine_under_command() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    let script = script_msg(
        11,
        T0,
        &[Step {
            after_ms: 500,
            duration_ms: 500,
            kind: StepKindWire::BASE_POSTURE,
            posture: PostureWire::UP,
        }],
        &[Overlay {
            motion_id: 0,
            after_ms: 500,
            duration_ms: 2_500,
        }],
    );
    cog.publish_script(&script, SyncTime::from_nanos(T0));
    let first = drive(&mut cog, FIRST_WAKE).expect("an accepted script starts the watch");
    sweep(&mut cog, &mut bus, FIRST_WAKE, first);
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::ACTIVE);

    // Past the step's end, inside the window's.
    let mut at = T0 + 1_500_000_000;
    drive(&mut cog, at);
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::ACTIVE,
        "the window is still playing",
    );

    // And past the window's.
    at = T0 + 3_000_000_000;
    drive(&mut cog, at);
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::STOPPING,
        "the whole schedule has run out",
    );
}

/// A schedule of nothing ends the session as soon as it begins.
///
/// A script asking for nothing is accepted, so the machine arms itself and then
/// has nothing to wait for: the end test is over what the schedule asks for, and
/// a schedule that asks for nothing has already asked for all of it. The
/// arming is not wasted -- it is what the sender asked for -- and the release
/// that follows is the same release every other session ends with.
#[test]
fn a_schedule_of_nothing_ends_the_session_on_the_wake_after_it_engaged() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    cog.publish_script(&script_msg(12, T0, &[], &[]), SyncTime::from_nanos(T0));
    let first = drive(&mut cog, FIRST_WAKE).expect("an accepted script starts the watch");
    sweep(&mut cog, &mut bus, FIRST_WAKE, first);
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::ACTIVE);

    let asked = drive(&mut cog, FIRST_WAKE + 400 * 1_000_000).expect("the wake owes a keep-alive");
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::STOPPING,
        "there was nothing to wait for",
    );
    assert_eq!(
        asked.kind,
        SessionCmdKindWire::KEEP_ALIVE,
        "and the settle is spoken through from its first wake",
    );
}

/// A servo that never says it let go takes the whole machine limp, and parks it.
///
/// The doctrine's line: a release nobody acknowledged is a servo that may still
/// be holding, and a machine that may be holding with nothing driving it must
/// not be reported as a session that ended well. So the condition goes down the
/// path every other one takes -- the library classifies it, and what it
/// classifies it as is the de-torquing that needs no acknowledgement -- and the
/// phase latches, because nothing engages a machine an operator has not seen.
#[test]
fn a_servo_that_never_says_it_let_go_takes_the_machine_limp_and_parks() {
    let mut cog = resting_session();
    let mut bus = Bus {
        refuse_release: Some((3, AuxStatusWire::TIMEOUT)),
        ..Bus::healthy()
    };
    engagement(&mut cog, &mut bus);
    let (ran, first_txn_at) = ending(&mut cog, &mut bus, FIRST_WAKE + 400 * 1_000_000);

    assert_eq!(
        ran.asks.last().map(|asked| asked.kind),
        Some(SessionCmdKindWire::TORQUE_OFF_NOW),
        "the wake the release concluded on commanded the de-torquing: {:?}",
        ran.asks,
    );
    assert_eq!(
        ran.asks
            .iter()
            .filter(|asked| asked.kind == SessionCmdKindWire::AUX)
            .count(),
        2 * JOINT_COUNT,
        "every other servo was still written to and read back",
    );
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);
    assert!(cog.state_sess().torque_off_pending());
    let mut told = ran.told.clone();
    told.extend(everything(&mut cog, first_txn_at + 10 * LAPSE_NS));
    let said = kinds(&told);
    assert!(
        !said.contains(&ReportKindWire::SESSION_ENDED),
        "no session ended here: {told:?}",
    );
    assert!(
        said.contains(&ReportKindWire::FAULT_RECORDED),
        "the servo that may be holding is a condition of the machine: {told:?}",
    );
    let taken = told
        .iter()
        .find(|report| report.kind == ReportKindWire::RESPONSE_TAKEN)
        .expect("and it was answered");
    assert_eq!(
        taken.a,
        u32::from(ResponseKindWire::from(ResponseKind::ImmediateAllTorqueOffToPark).0),
    );
}

/// A release snapshot that will not read back commands the de-torquing outright
/// and latches.
///
/// The opposite answer from the one a refused commissioning snapshot gets, and
/// for the reason the two differ: a survey establishes a machine nothing is
/// holding, so starting over costs reads, where how far a release got is exactly
/// what a slot that will not read back no longer says. The command that needs
/// nothing from the slot is the one that reaches the machine fastest.
#[test]
fn a_release_snapshot_that_will_not_resume_commands_the_de_torquing_and_parks() {
    let mut cog = session();
    cog.state_sess_mut().set_phase(SessionPhaseWire::STOPPING);
    // A release under way, over a slot holding no release: the form a sequence
    // writes on its first step is what a slot nothing wrote does not have.
    cog.state_sess_mut().set_seq_kind(SeqKindWire::DISARM);

    let asked = drive(&mut cog, FIRST_WAKE).expect("the wake publishes something");
    assert_eq!(asked.kind, SessionCmdKindWire::TORQUE_OFF_NOW);
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);
    assert!(cog.state_sess().torque_off_pending());
}

/// Drive a resting session's engagement as far as its first enable write, and
/// answer with the instant that write went out on.
///
/// The write is left unanswered: what a case wants from here is the stretch
/// after the torque line, where nine servos may be holding and nothing is
/// streaming to them.
fn armed_to_the_first_enable(cog: &mut SessionTestWrapper, bus: &mut Bus) -> i64 {
    cog.publish_script(&one_step_script(7, T0), SyncTime::from_nanos(T0));
    let mut asked = drive(cog, FIRST_WAKE).expect("an accepted script starts the watch");
    let mut at = FIRST_WAKE;
    for _ in 0..400 {
        // The value says which write this is: an arming enables torque and a
        // release writes the zero.
        if asked.op == AuxOpKindWire::WRITE_REG_VERIFIED
            && asked.reg == RegIdWire::TORQUE_ENABLE
            && asked.value != 0
        {
            return at;
        }
        assert_eq!(
            asked.kind,
            SessionCmdKindWire::AUX,
            "the sequences stopped asking before the machine was armed",
        );
        let outcome = bus.answer(&asked);
        at += 1_000_000;
        cog.publish_aux_out(&outcome, SyncTime::from_nanos(at));
        at += 1_000_000;
        asked = drive(cog, at).expect("the sequences carry on asking");
    }
    panic!("the engagement never reached its first enable write");
}

/// A wake mid-arming that owes no transaction still speaks to the driver.
///
/// The window the keep-alive rule was written for: from the first enable write
/// until the engagement concludes, the machine may be holding torque with
/// nothing streaming to it, and the driver's hold timeout is what takes torque
/// off a machine nobody has spoken to. A wake spent waiting on a slow read-back
/// owes no transaction of its own -- the outstanding one is not re-issued until
/// its delivery budget is spent -- so what it publishes is the keep-alive, and a
/// build that published nothing here would have the dead-man de-torque a machine
/// mid-arming.
#[test]
fn a_wake_mid_arming_that_owes_no_transaction_publishes_a_keep_alive() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    let enabled_at = armed_to_the_first_enable(&mut cog, &mut bus);
    assert!(
        cog.state_sess().engage().torque_written(),
        "the torque line has been crossed",
    );

    // A wake floor past the write and inside the delivery budget, so nothing is
    // re-issued and the wake owes no transaction at all.
    const { assert!(LAPSE_NS < AUX_TIMEOUT_NS) };
    let asked = drive(&mut cog, enabled_at + LAPSE_NS).expect("the wake spoke to the driver");
    assert_eq!(
        asked.kind,
        SessionCmdKindWire::KEEP_ALIVE,
        "a machine that may be holding is spoken to on every wake",
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::ENGAGING,
        "and the engagement is still the sequence being driven",
    );
}

/// The same wake during the watch owes the driver nothing.
///
/// What scopes the rule is the torque line and not the phase: the reads that
/// measure where a resting machine is standing are made before anything is
/// enabled, so there is nothing holding for a hold timeout to take off, and a
/// keep-alive here would be traffic that says nothing about a machine at rest.
#[test]
fn a_wake_mid_watch_before_the_torque_line_publishes_nothing() {
    let mut cog = resting_session();
    cog.publish_script(&one_step_script(7, T0), SyncTime::from_nanos(T0));
    let read = drive(&mut cog, FIRST_WAKE).expect("the watch's first read");
    assert_eq!(read.op, AuxOpKindWire::READ_REG);
    assert!(
        !cog.state_sess().engage().torque_written(),
        "nothing has been enabled",
    );

    assert!(
        drive(&mut cog, FIRST_WAKE + LAPSE_NS).is_none(),
        "an unarmed machine is owed no keep-alive",
    );
}

// What the machine is under command to do, published: the one thing the session
// says that another cog acts on rather than reads for the record.

/// One published schedule, copied out of the message.
#[derive(Clone, Copy, PartialEq, Debug)]
struct Published {
    engaged: bool,
    epoch: u32,
    steps: usize,
}

/// Every schedule the session has published since it was last asked, in order.
fn publishes(cog: &mut SessionTestWrapper) -> Vec<Published> {
    let mut out = Vec::new();
    while let Some(msg) = cog.try_next_sched() {
        out.push(Published {
            engaged: msg.engaged(),
            epoch: msg.epoch(),
            steps: msg.steps().len(),
        });
    }
    out
}

/// The engagement taking hold publishes the schedule, and nothing republishes it.
///
/// A consumer acts on the publish edge, so it must be the engagement's own:
/// before it there is nothing to act on, and after it the record stands until
/// the session changes it. A schedule republished on the wakes between
/// changes would be the session saying the same thing over and over to a
/// consumer whose whole reading of it is the latest message -- and the epoch,
/// which is what makes a change news, would move with every repetition.
#[test]
fn the_engagement_publishes_the_schedule_it_took_hold_on() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    let ran = engagement(&mut cog, &mut bus);
    let accepted = ran
        .told
        .iter()
        .find(|report| report.kind == ReportKindWire::SCRIPT_ACCEPTED)
        .copied()
        .expect("the script was accepted");

    assert_eq!(
        ran.published,
        vec![Published {
            engaged: true,
            #[allow(clippy::cast_possible_truncation)]
            epoch: accepted.detail as u32 + 1,
            steps: 1,
        }],
        "one schedule, engaged, under the epoch after the one the acceptance \
         wrote",
    );

    let mut at = FIRST_WAKE + 400 * 1_000_000;
    for _ in 0..5 {
        at += LAPSE_NS;
        drive(&mut cog, at);
        assert!(
            publishes(&mut cog).is_empty(),
            "nothing is published between changes",
        );
    }
}

/// A session that is over publishes a schedule nobody is engaged on.
///
/// A consumer that streams while the machine is under command relies on this to
/// know the session ended. A fresh epoch on it for the same reason the
/// engagement's carried one -- the next engagement is a fresh base to move from.
#[test]
fn a_session_that_is_over_publishes_a_schedule_nobody_is_engaged_on() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    let took_hold = engagement(&mut cog, &mut bus).published;
    let engaged_epoch = took_hold.first().expect("the engagement's schedule").epoch;

    let (ran, _) = ending(&mut cog, &mut bus, FIRST_WAKE + 400 * 1_000_000);

    assert_eq!(
        ran.published,
        vec![Published {
            engaged: false,
            epoch: engaged_epoch + 1,
            steps: 1,
        }],
        "one schedule, disengaged, under a fresh epoch: the steps are still \
         there because what changed is that nobody is running them",
    );
}

// Replacement: a script accepted while the machine is already under command.

/// Where the engagement's one step ends: the horizon a refresh has to beat.
///
/// Derived from `engagement`: one step half a second after `T0`, for two
/// seconds, so the schedule runs out here.
const ENGAGED_SCHEDULE_END: i64 = T0 + 2_500_000_000;

/// The instant the cases below hand the session a replacement: under command,
/// and well inside the running schedule.
const REPLACED_AT: i64 = FIRST_WAKE + 400 * 1_000_000;

/// A session under command, with everything the arming said read past.
///
/// Driven through the real engagement rather than written into the slot: a
/// replacement is asserted against a schedule that was actually published, an
/// epoch a consumer has already seen, and torque that is actually on.
fn active_session(bus: &mut Bus) -> SessionTestWrapper {
    let mut cog = resting_session();
    engagement(&mut cog, bus);
    everything(&mut cog, FIRST_WAKE + 300 * 1_000_000);
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::ACTIVE,
        "the engagement was expected to take hold",
    );
    caught_up(&mut cog);
    let _ = publishes(&mut cog);
    cog
}

/// A one-step script that stands the machine up from its own arrival.
fn hold_script(script_id: u32, arrival_ns: i64, duration_ms: u32) -> ScriptWire {
    script_msg(
        script_id,
        arrival_ns,
        &[Step {
            after_ms: 0,
            duration_ms,
            kind: StepKindWire::BASE_POSTURE,
            posture: PostureWire::UP,
        }],
        &[],
    )
}

/// A script accepted under command replaces the schedule whole and moves no
/// phase.
///
/// The engagement is not re-run: torque is already on and the mover is already
/// streaming, so what a replacement costs is one epoch and one datagram. The
/// `engaged` flag never passes through false, which is the load-bearing half --
/// a pause in the goal stream is what the driver's dead-man is measuring.
#[test]
fn a_script_accepted_under_command_replaces_the_running_schedule() {
    let mut bus = Bus::healthy();
    let mut cog = active_session(&mut bus);
    let before = cog.state_sess().schedule().epoch();

    cog.publish_script(
        &hold_script(8, REPLACED_AT, 3_000),
        SyncTime::from_nanos(REPLACED_AT),
    );
    let report = wake(&mut cog, REPLACED_AT + 1).expect("a replacement is narrated");
    let published = publishes(&mut cog);

    assert_eq!(report.kind, ReportKindWire::SCRIPT_REPLACED);
    assert_eq!(report.a, 8, "the script that took over");
    assert_eq!(report.b, before + 1, "under the epoch it was written at");
    assert!(
        (report.detail - 1.0).abs() < f64::EPSILON,
        "and how many steps it asked for: {report:?}",
    );

    let state = cog.state_sess();
    assert_eq!(
        state.phase(),
        SessionPhaseWire::ACTIVE,
        "a replacement arms nothing, so no phase moves",
    );
    assert_eq!(state.script_id(), 8);
    assert_eq!(
        state.active_script_id(),
        8,
        "and it is what the next one must beat"
    );
    assert_eq!(state.scripts_accepted(), 2);
    assert_eq!(state.scripts_refused(), 0);

    let schedule = state.schedule();
    assert!(
        schedule.engaged(),
        "the machine was under command throughout",
    );
    assert_eq!(schedule.epoch(), before + 1, "bumped exactly once");
    assert_eq!(
        schedule.steps().len(),
        1,
        "the new schedule is the whole of it"
    );
    let step = schedule.steps().get(0).expect("the one step");
    assert_eq!(step.start().as_nanos(), REPLACED_AT);
    assert_eq!(step.end().as_nanos(), REPLACED_AT + 3_000_000_000);

    assert_eq!(
        published,
        vec![Published {
            engaged: true,
            epoch: before + 1,
            steps: 1,
        }],
        "one publish, engaged throughout, under the epoch the row names",
    );
}

/// The old schedule's overlay windows go with the old schedule.
///
/// Replacement is whole and never a merge: what the session holds afterwards is
/// the new script and nothing of the one before it, which is the seam semantics
/// a presence sender compiles its refreshes from.
#[test]
fn a_replacement_carries_its_own_windows_and_none_of_the_old_ones() {
    let mut bus = Bus::healthy();
    let mut cog = active_session(&mut bus);
    cog.state_sess_mut()
        .schedule_mut()
        .overlays_mut()
        .try_grow()
        .expect("a window the old schedule was running");

    cog.publish_script(
        &script_msg(
            8,
            REPLACED_AT,
            &[Step {
                after_ms: 0,
                duration_ms: 3_000,
                kind: StepKindWire::BASE_POSTURE,
                posture: PostureWire::UP,
            }],
            &[Overlay {
                motion_id: 4,
                after_ms: 100,
                duration_ms: 500,
            }],
        ),
        SyncTime::from_nanos(REPLACED_AT),
    );
    wake(&mut cog, REPLACED_AT + 1).expect("a replacement is narrated");

    let state = cog.state_sess();
    let overlays = state.schedule().overlays();
    assert_eq!(overlays.len(), 1, "the new script's window and only it");
    let window = overlays.get(0).expect("the one window");
    assert_eq!(window.motion_id(), 4);
    assert_eq!(window.start().as_nanos(), REPLACED_AT + 100_000_000);
}

/// A replacement numbered no higher than the engagement's own script is
/// refused, and refusing it moves nothing.
///
/// Strictly greater, so a re-delivered replacement is harmless: the channel
/// this arrives on will sit across a link that duplicates, and a duplicate
/// re-planned would be a second epoch for a schedule already running.
#[test]
fn a_replacement_that_does_not_beat_the_running_script_is_refused_as_stale() {
    for (id, what) in [(7, "the same number"), (6, "a lower one")] {
        let mut bus = Bus::healthy();
        let mut cog = active_session(&mut bus);
        let before = cog.state_sess().schedule().epoch();

        cog.publish_script(
            &hold_script(id, REPLACED_AT, 3_000),
            SyncTime::from_nanos(REPLACED_AT),
        );
        let report = wake(&mut cog, REPLACED_AT + 1).expect("a refusal is narrated");
        let published = publishes(&mut cog);

        assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED, "{what}");
        assert_eq!(report.a, id);
        assert_eq!(report.b, u32::from(RefusalReasonWire::STALE.0), "{what}");

        let state = cog.state_sess();
        assert_eq!(state.active_script_id(), 7, "a refusal advances nothing");
        assert_eq!(state.script_id(), 7);
        assert_eq!(state.schedule().epoch(), before, "and republishes nothing");
        assert_eq!(state.scripts_refused(), 1);
        assert!(published.is_empty(), "{what}: nothing went out");
    }
}

/// Nothing orders a script arriving at a resting machine.
///
/// The high-water mark is scoped to the engagement it belongs to. Kept across
/// engagements it would be a lockout: a sender that reset its counter -- which
/// is a sender breaking its own contract, but a real shape -- could never raise
/// the machine again without an operator restarting the process.
#[test]
fn a_lower_number_after_a_session_is_over_is_accepted() {
    let mut cog = resting_session();

    cog.publish_script(&one_step_script(9, T0), SyncTime::from_nanos(T0));
    let first = wake(&mut cog, T0 + 1).expect("an acceptance is narrated");
    assert_eq!(first.kind, ReportKindWire::SCRIPT_ACCEPTED);
    assert_eq!(cog.state_sess().active_script_id(), 9);

    // Back to rest without ever going through an engagement: what this case is
    // about is the screen, and the phase is the whole of its input.
    cog.state_sess_mut().set_phase(SessionPhaseWire::RESTING);
    // The phase the first acceptance moved to is read past: what this case is
    // about is the answer to the second script.
    caught_up(&mut cog);
    let at = T0 + 2 * LAPSE_NS;
    cog.publish_script(&one_step_script(2, at), SyncTime::from_nanos(at));
    let second = wake(&mut cog, at + 1).expect("the second answer");

    assert_eq!(
        second.kind,
        ReportKindWire::SCRIPT_ACCEPTED,
        "a session beginning takes whatever number it is offered",
    );
    assert_eq!(cog.state_sess().active_script_id(), 2);
}

/// A schedule reaching further ahead than the session will hold one open for is
/// refused, in either phase that takes a script.
///
/// The horizon is the dead-man on the sender: past the end of the last schedule
/// it accepted, the machine stows and lets go. A ceiling is what stops a sender
/// that died from leaving it torqued indefinitely, and it is a refusal rather
/// than a truncation -- a schedule cut to fit is a schedule nobody asked for.
#[test]
fn a_script_reaching_past_the_span_cap_is_refused_at_rest() {
    let mut cog = resting_session();

    cog.publish_script(
        &hold_script(1, T0, SCRIPT_SPAN_CAP_MS + 1),
        SyncTime::from_nanos(T0),
    );
    let report = wake(&mut cog, T0 + 1).expect("a refusal is narrated");

    assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(report.b, u32::from(RefusalReasonWire::TOO_LONG.0));
    assert_eq!(
        cog.state_sess().schedule().epoch(),
        0,
        "and nothing was written",
    );
}

/// A horizon exactly at the ceiling is inside it: the cap is what a script may
/// reach, not what it must stay under.
#[test]
fn a_script_reaching_exactly_to_the_span_cap_is_accepted() {
    let mut cog = resting_session();

    cog.publish_script(
        &hold_script(1, T0, SCRIPT_SPAN_CAP_MS),
        SyncTime::from_nanos(T0),
    );
    let report = wake(&mut cog, T0 + 1).expect("an acceptance is narrated");

    assert_eq!(report.kind, ReportKindWire::SCRIPT_ACCEPTED);
}

/// An overlay window is horizon too: a script whose steps are short and whose
/// window runs past the ceiling reaches past it.
#[test]
fn an_overlay_window_past_the_span_cap_is_refused() {
    let mut cog = resting_session();

    cog.publish_script(
        &script_msg(
            1,
            T0,
            &[Step {
                after_ms: 0,
                duration_ms: 1_000,
                kind: StepKindWire::BASE_POSTURE,
                posture: PostureWire::UP,
            }],
            &[Overlay {
                motion_id: 1,
                after_ms: SCRIPT_SPAN_CAP_MS,
                duration_ms: 1,
            }],
        ),
        SyncTime::from_nanos(T0),
    );
    let report = wake(&mut cog, T0 + 1).expect("a refusal is narrated");

    assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(report.b, u32::from(RefusalReasonWire::TOO_LONG.0));
}

/// The ceiling holds mid-session too, and the running schedule survives the
/// refusal.
#[test]
fn a_replacement_reaching_past_the_span_cap_is_refused_under_command() {
    let mut bus = Bus::healthy();
    let mut cog = active_session(&mut bus);
    let before = cog.state_sess().schedule().epoch();

    cog.publish_script(
        &hold_script(8, REPLACED_AT, SCRIPT_SPAN_CAP_MS + 1),
        SyncTime::from_nanos(REPLACED_AT),
    );
    let report = wake(&mut cog, REPLACED_AT + 1).expect("a refusal is narrated");
    let published = publishes(&mut cog);

    // The kind before the reason: `b` means something different in every row
    // kind, and an epoch that happened to equal the reason's number would read
    // as this assertion passing on an accepted replacement.
    assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(report.b, u32::from(RefusalReasonWire::TOO_LONG.0));
    let state = cog.state_sess();
    assert_eq!(state.schedule().epoch(), before, "the old schedule stands");
    assert_eq!(state.active_script_id(), 7);
    assert!(published.is_empty());
}

/// How far ahead of this machine's clock the cases below stamp a script.
///
/// Twice the ceiling, so a script carrying a horizon well inside the ceiling
/// still reaches past it measured from the wake that reads it.
const STAMPED_AHEAD_NS: i64 = 2 * (SCRIPT_SPAN_CAP_MS as i64) * 1_000_000;

/// A script stamped ahead of this machine's clock cannot buy reach with the
/// skew, at rest.
///
/// The ceiling is a bound on the machine's committed future, so it is measured
/// from the wake as well as from the sender's own stamp. A script stamped an
/// hour out with a three-second horizon would otherwise arm the head and hold
/// it for an hour: nothing in the schedule is due, so nothing ends the session
/// and no watchdog is measuring anything -- the head is simply up, torqued, for
/// as long as the stamp said.
#[test]
fn a_script_stamped_far_ahead_of_the_clock_is_refused_at_rest() {
    let mut cog = resting_session();

    cog.publish_script(
        &hold_script(1, T0 + STAMPED_AHEAD_NS, 3_000),
        SyncTime::from_nanos(T0),
    );
    let report = wake(&mut cog, T0 + 1).expect("a refusal is narrated");

    assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(report.b, u32::from(RefusalReasonWire::TOO_LONG.0));
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::RESTING,
        "and no engagement was opened on it",
    );
    assert_eq!(cog.state_sess().schedule().epoch(), 0);
}

/// The same, mid-session: a refresh stamped far ahead extends nothing.
///
/// This is the shape that matters most for a presence session, because every
/// refresh re-opens the horizon: a sender whose clock runs ahead would hold the
/// machine past the ceiling for as long as it kept refreshing, and each refresh
/// would push the end further out.
#[test]
fn a_replacement_stamped_far_ahead_of_the_clock_is_refused() {
    let mut bus = Bus::healthy();
    let mut cog = active_session(&mut bus);
    let before = cog.state_sess().schedule().epoch();

    cog.publish_script(
        &hold_script(8, REPLACED_AT + STAMPED_AHEAD_NS, 3_000),
        SyncTime::from_nanos(REPLACED_AT),
    );
    let report = wake(&mut cog, REPLACED_AT + 1).expect("a refusal is narrated");
    let published = publishes(&mut cog);

    assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(report.b, u32::from(RefusalReasonWire::TOO_LONG.0));
    let state = cog.state_sess();
    assert_eq!(state.schedule().epoch(), before, "the old schedule stands");
    assert_eq!(state.active_script_id(), 7);
    assert!(published.is_empty());
}

/// A replacement the screen cannot turn into a schedule leaves the one running
/// exactly as it was.
///
/// All-or-nothing, mid-session as much as at rest: the plan is built beside the
/// slot and only a whole one is written, so a refused refresh costs the sender
/// its refresh and the machine nothing.
#[test]
fn a_replacement_that_is_no_timeline_leaves_the_running_schedule_alone() {
    let mut bus = Bus::healthy();
    let mut cog = active_session(&mut bus);
    let before = cog.state_sess().schedule().epoch();
    let steps = cog.state_sess().schedule().steps().len();

    cog.publish_script(
        &no_timeline_script(8, REPLACED_AT),
        SyncTime::from_nanos(REPLACED_AT),
    );
    let report = wake(&mut cog, REPLACED_AT + 1).expect("a refusal is narrated");
    let published = publishes(&mut cog);

    assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED);
    assert_eq!(report.b, u32::from(RefusalReasonWire::BAD_TIMES.0));
    let state = cog.state_sess();
    assert_eq!(state.schedule().epoch(), before, "the epoch did not move");
    assert_eq!(state.schedule().steps().len(), steps);
    assert!(state.schedule().engaged());
    assert_eq!(
        state.script_id(),
        7,
        "and the session is on the script it had"
    );
    assert!(published.is_empty());
}

/// Every phase that is neither resting nor active still refuses.
///
/// A machine mid-arm-sequence, being carried down out of a fault, or being let
/// go of finishes what it is doing. Intent never preempts a fault maneuver, and
/// a sender that still wants the machine raises it again from rest.
#[test]
fn the_phases_between_the_two_that_take_a_script_still_refuse() {
    for phase in [
        SessionPhaseWire::STARTING,
        SessionPhaseWire::ENGAGING,
        SessionPhaseWire::WINDING_DOWN,
        SessionPhaseWire::STOPPING,
    ] {
        let mut cog = session();
        cog.state_sess_mut().set_phase(phase);

        cog.publish_script(&one_step_script(1, T0), SyncTime::from_nanos(T0));
        let report = wake(&mut cog, T0 + 1).expect("a refusal is narrated");

        assert_eq!(report.kind, ReportKindWire::SCRIPT_REFUSED, "in {phase:?}");
        assert_eq!(
            report.b,
            u32::from(RefusalReasonWire::NOT_RESTING.0),
            "in {phase:?}",
        );
    }
}

/// A wake that both answers a park-class fault and screens a refresh takes the
/// response and refuses the script.
///
/// The order is the deliberate one: evidence is weighed before intake, so the
/// script is answered against the phase this same wake left the machine in. A
/// refresh accepted onto a machine already parked would be an acceptance its
/// sender acts on for a session that is over.
#[test]
fn a_replacement_arriving_with_a_park_class_fault_is_refused() {
    let mut bus = Bus::healthy();
    let mut cog = active_session(&mut bus);

    cog.publish_evt(
        &event(EventKindWire::BUS_FAILURE, REPLACED_AT),
        SyncTime::from_nanos(REPLACED_AT),
    );
    cog.publish_script(
        &hold_script(8, REPLACED_AT, 3_000),
        SyncTime::from_nanos(REPLACED_AT),
    );
    let asked = drive(&mut cog, REPLACED_AT + 1).expect("the release goes out at once");

    assert_eq!(asked.kind, SessionCmdKindWire::TORQUE_OFF_NOW);
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);
    let told: Vec<Said> = said(&mut cog)
        .into_iter()
        .chain(everything(&mut cog, REPLACED_AT + 1 + LAPSE_NS))
        .collect();
    let refused = told
        .iter()
        .find(|report| report.kind == ReportKindWire::SCRIPT_REFUSED)
        .expect("the script was answered");
    assert_eq!(refused.b, u32::from(RefusalReasonWire::PARKED.0));
    assert!(
        !told
            .iter()
            .any(|report| report.kind == ReportKindWire::SCRIPT_REPLACED),
        "nothing was replaced: {told:?}",
    );
    assert_eq!(cog.state_sess().active_script_id(), 7);
}

/// A replacement arriving while an antenna pair is being let go of is taken,
/// and the pair still goes limp.
///
/// The one fault response that leaves the session in a phase that takes a
/// script: a pair going limp while the head keeps its presence is a fault
/// answered, not a session ended, so the machine is still under command and a
/// refresh still lands. Both halves are the point. The refresh is answered as a
/// replacement, and nothing about it gates the de-torquing underneath -- the
/// two verified writes go out in the same order and the same count as they
/// would have with no script in the wake at all.
#[test]
fn a_replacement_during_an_antenna_degrade_is_taken_and_the_pair_still_goes_limp() {
    let mut bus = Bus::healthy();
    let mut cog = active_session(&mut bus);
    let before = cog.state_sess().schedule().epoch();
    let right = SERVO_IDS[usize::from(JointRefWire::ANTENNA_RIGHT.0) - 1];
    let left = SERVO_IDS[usize::from(JointRefWire::ANTENNA_LEFT.0) - 1];

    cog.publish_readings(
        &reading(right, 0x20, REPLACED_AT),
        SyncTime::from_nanos(REPLACED_AT),
    );
    cog.publish_script(
        &hold_script(8, REPLACED_AT, 3_000),
        SyncTime::from_nanos(REPLACED_AT),
    );
    let first = drive(&mut cog, REPLACED_AT).expect("the first row is told to let go");
    let mut told: Vec<Said> = Vec::new();
    while let Some(row) = said(&mut cog) {
        told.push(row);
    }
    let published = publishes(&mut cog);
    let mut ran = sweep(&mut cog, &mut bus, REPLACED_AT, first);

    let replaced = told
        .iter()
        .find(|report| report.kind == ReportKindWire::SCRIPT_REPLACED)
        .unwrap_or_else(|| panic!("the refresh was taken: {told:?}"));
    assert_eq!(replaced.a, 8);
    assert_eq!(replaced.b, before + 1, "under the epoch it was written at");
    assert_eq!(
        published,
        vec![Published {
            engaged: true,
            epoch: before + 1,
            steps: 1,
        }],
        "the replacement went out engaged, on the wake it was taken",
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::ACTIVE,
        "a degrade moves no phase and neither does a replacement",
    );

    assert_eq!(
        ran.asks.len(),
        2,
        "one verified write per antenna and nothing else: {:?}",
        ran.asks,
    );
    for (asked, id) in ran.asks.iter().zip([right, left]) {
        assert_eq!(
            (asked.op, asked.reg, asked.value, asked.id),
            (
                AuxOpKindWire::WRITE_REG_VERIFIED,
                RegIdWire::TORQUE_ENABLE,
                value::u8(0).bits(),
                id,
            ),
            "the pair in bus order, each write read back",
        );
    }
    ran.told
        .extend(everything(&mut cog, REPLACED_AT + 2 * LAPSE_NS));
    assert!(
        ran.told
            .iter()
            .any(|report| report.kind == ReportKindWire::DEGRADE_RELEASED),
        "the drain finished: {:?}",
        ran.told,
    );
    assert_eq!(
        cog.state_sess().degrade_release(),
        JointFlagsWire::from(JointFlags::NONE),
        "and nothing is still owed",
    );
    assert!(
        !cog.state_sess().degrade_pending(),
        "the accept did not leave the drain waiting on the aux path",
    );
}

/// A refresh arriving on the wake the running schedule expires wins the race.
///
/// Intake runs before the bus half, so the refresh is screened while the phase
/// is still active and the extended horizon means the schedule has not run out
/// by the time anything asks. The session continues with no stow and no
/// re-engagement -- and correctness does not depend on the race: a refresh that
/// misses the wake finds the session concluded and raises it again from rest.
#[test]
fn a_refresh_on_the_expiry_wake_keeps_the_session_running() {
    let mut bus = Bus::healthy();
    let mut cog = active_session(&mut bus);

    cog.publish_script(
        &hold_script(8, ENGAGED_SCHEDULE_END, 2_000),
        SyncTime::from_nanos(ENGAGED_SCHEDULE_END),
    );
    let report = wake(&mut cog, ENGAGED_SCHEDULE_END).expect("the refresh is answered");

    assert_eq!(report.kind, ReportKindWire::SCRIPT_REPLACED);
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::ACTIVE,
        "the session runs on the horizon the refresh gave it",
    );
    let told: Vec<Said> = said(&mut cog).into_iter().collect();
    assert!(
        !told
            .iter()
            .any(|report| report.kind == ReportKindWire::SESSION_ENDED),
        "nothing ended: {told:?}",
    );
    assert!(cog.state_sess().schedule().engaged());
}

/// Every row this wake said, the one [`wake`] answers with included.
fn all_of_it(cog: &mut SessionTestWrapper, at_ns: i64) -> Vec<Said> {
    let mut told: Vec<Said> = wake(cog, at_ns).into_iter().collect();
    while let Some(row) = said(cog) {
        told.push(row);
    }
    told
}

/// Two replacements in one intake window share the wake's one epoch, and it is
/// the epoch that goes out.
///
/// The channel is latest-wins and the slot holds one schedule, so the later
/// script is what the session runs. What must not happen is a row naming an
/// epoch no consumer ever saw: the timeline is the durable account of a run, and
/// an operator reconciling the mover's answered epochs against the session's
/// published ones cannot tell a bookkeeping artefact from a dropped datagram.
/// A sender refreshing on a cadence across a link that duplicates is exactly the
/// traffic that puts two in one window.
#[test]
fn two_replacements_in_one_window_share_the_epoch_that_goes_out() {
    let mut bus = Bus::healthy();
    let mut cog = active_session(&mut bus);
    let before = cog.state_sess().schedule().epoch();

    cog.publish_script(
        &hold_script(8, REPLACED_AT, 3_000),
        SyncTime::from_nanos(REPLACED_AT),
    );
    cog.publish_script(
        &hold_script(9, REPLACED_AT, 4_000),
        SyncTime::from_nanos(REPLACED_AT),
    );
    let told = all_of_it(&mut cog, REPLACED_AT + 1);
    let published = publishes(&mut cog);

    let replaced: Vec<&Said> = told
        .iter()
        .filter(|row| row.kind == ReportKindWire::SCRIPT_REPLACED)
        .collect();
    assert_eq!(replaced.len(), 2, "both were answered: {told:?}");
    assert_eq!(replaced[0].a, 8);
    assert_eq!(replaced[1].a, 9);
    assert_eq!(replaced[0].b, before + 1, "one epoch, opened once");
    assert_eq!(replaced[1].b, before + 1);

    let sent: Vec<&Said> = told
        .iter()
        .filter(|row| row.kind == ReportKindWire::SCHEDULE_PUBLISHED)
        .collect();
    assert_eq!(sent.len(), 1, "one publish for the wake: {told:?}");
    assert_eq!(sent[0].a, before + 1, "and the rows name what went out");

    let state = cog.state_sess();
    assert_eq!(state.script_id(), 9, "the later script is what runs");
    assert_eq!(state.active_script_id(), 9);
    assert_eq!(state.schedule().epoch(), before + 1);
    let step = state.schedule().steps().get(0).expect("the one step");
    assert_eq!(step.end().as_nanos(), REPLACED_AT + 4_000_000_000);
    assert_eq!(
        published,
        vec![Published {
            engaged: true,
            epoch: before + 1,
            steps: 1,
        }],
        "one datagram, carrying the later script",
    );
}

/// A replacement whose schedule was already over ends the session on the same
/// wake, in one datagram.
///
/// A stamp behind the clock is not what the horizon screen catches: the
/// wake-relative measure only ever tightens the bound, so a schedule already
/// behind `now` is bounded by its own offsets and passes. What it leaves is a
/// schedule with no future.
///
/// The bus half ends it on the wake it arrived on, and because the wake
/// publishes once and bumps once, the machine being let go of and the
/// replacement are one epoch on the channel rather than a row for a schedule
/// nobody received.
#[test]
fn a_replacement_that_was_already_over_ends_the_session_in_one_publish() {
    let mut bus = Bus::healthy();
    let mut cog = active_session(&mut bus);
    let before = cog.state_sess().schedule().epoch();
    let stamped = REPLACED_AT - 3_000_000_000;

    cog.publish_script(
        &hold_script(8, stamped, 1_000),
        SyncTime::from_nanos(stamped),
    );
    let told = all_of_it(&mut cog, REPLACED_AT + 1);
    let published = publishes(&mut cog);

    let replaced = told
        .iter()
        .find(|row| row.kind == ReportKindWire::SCRIPT_REPLACED)
        .expect("the replacement was answered");
    assert_eq!(replaced.b, before + 1, "the epoch this wake opened");

    let sent: Vec<&Said> = told
        .iter()
        .filter(|row| row.kind == ReportKindWire::SCHEDULE_PUBLISHED)
        .collect();
    assert_eq!(sent.len(), 1, "one publish for the wake: {told:?}");
    assert_eq!(sent[0].a, before + 1, "and it is the epoch the row names");

    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::STOPPING,
        "a schedule with no future is a session over",
    );
    assert_eq!(
        published,
        vec![Published {
            engaged: false,
            epoch: before + 1,
            steps: 1,
        }],
        "one datagram, saying nobody is running it",
    );
}

/// The schedule's script and the engagement's admission floor never disagree.
///
/// They are written together and mean different things -- one names the script
/// the schedule was built from, the other is what a replacement must beat -- and
/// the stale screen is only sound while they agree. A path that re-anchored one
/// alone would move replacement admission with nothing at the site saying so.
#[test]
fn the_schedule_s_script_and_the_admission_floor_never_disagree() {
    fn agree(cog: &mut SessionTestWrapper, after: &str) {
        let state = cog.state_sess();
        assert_eq!(state.script_id(), state.active_script_id(), "after {after}",);
    }

    let mut bus = Bus::healthy();
    let mut cog = active_session(&mut bus);
    agree(&mut cog, "the script that opened the engagement");

    cog.publish_script(
        &hold_script(7, REPLACED_AT, 3_000),
        SyncTime::from_nanos(REPLACED_AT),
    );
    wake(&mut cog, REPLACED_AT + 1).expect("a refusal is narrated");
    agree(&mut cog, "a refusal, which moves neither");

    let at = REPLACED_AT + LAPSE_NS;
    cog.publish_script(&hold_script(8, at, 3_000), SyncTime::from_nanos(at));
    wake(&mut cog, at + 1).expect("a replacement is narrated");
    agree(&mut cog, "a replacement, which moves both");

    let (_, ended_at) = ending(&mut cog, &mut bus, at + LAPSE_NS);
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::RESTING,
        "the schedule was expected to run out",
    );
    agree(&mut cog, "the session ending, which resets neither");

    // And the floor a concluded engagement left behind is no floor at all: the
    // machine that just ran a session numbered 8 is raised again by a 2. Driven
    // through the real ending rather than by putting the phase back, because
    // what would lock the machine out is exactly a path that ended a session
    // while leaving the floor standing.
    caught_up(&mut cog);
    let again = ended_at + LAPSE_NS;
    cog.publish_script(&hold_script(2, again, 3_000), SyncTime::from_nanos(again));
    let raised = wake(&mut cog, again + 1).expect("the answer to the low number");
    assert_eq!(
        raised.kind,
        ReportKindWire::SCRIPT_ACCEPTED,
        "a sender that reset its counter can still raise a machine at rest",
    );
    assert_eq!(cog.state_sess().active_script_id(), 2);
    agree(&mut cog, "the engagement the low number opened");
}

// The stow maneuvers: the two responses that carry the machine down under
// control rather than letting go of it where it stands. What a maneuver decides
// is the motion library's; what these cases are about is what a session does
// with the decision -- the schedule it publishes, the once-per-stow rule it
// publishes it under, and the ending it records.

/// The driver reporting a machine standing at its stow pose.
///
/// The one piece of evidence a stow maneuver turns on: the session cannot see
/// the tick's own move, so what says the head came down is a complete reading
/// that measures at the fold.
fn folded(cog: &mut SessionTestWrapper, at_ns: i64) {
    let sample = Sample {
        nominal_time_ns: at_ns,
        sample_time_ns: at_ns,
        present_valid: true,
        commanded_valid: true,
        torque_off_latched: false,
        missing: 0,
        present: stow_rows(),
        commanded: [0.0; JOINT_COUNT],
    };
    cog.publish_sample(&sample.message(), SyncTime::from_nanos(at_ns));
}

/// The nine angles of a machine standing well clear of its fold.
///
/// Every row a whole tolerance and more away from where the stow puts it, cut
/// from the library's own number rather than named here: what a case built on
/// this wants is a reading that is complete, finite and not at the fold, and a
/// build whose tolerance moved must not turn it into one.
fn standing_rows() -> [f64; JOINT_COUNT] {
    let mut rows = stow_rows();
    for row in &mut rows {
        *row += 10.0 * DEFAULT_STOW_TOLERANCE;
    }
    rows
}

/// A reading of the machine as the driver's sample stream carries one.
fn reads(cog: &mut SessionTestWrapper, at_ns: i64, present: &[f64; JOINT_COUNT], missing: u16) {
    let sample = Sample {
        nominal_time_ns: at_ns,
        sample_time_ns: at_ns,
        present_valid: true,
        commanded_valid: true,
        torque_off_latched: false,
        missing,
        present: *present,
        commanded: [0.0; JOINT_COUNT],
    };
    cog.publish_sample(&sample.message(), SyncTime::from_nanos(at_ns));
}

/// Run one execution at `at_ns` with the machine reading as folded.
fn wake_folded(cog: &mut SessionTestWrapper, at_ns: i64) {
    folded(cog, at_ns);
    assert!(stepped(cog, at_ns), "the session was expected to wake",);
}

/// Drive `wakes` executions at the wake floor, keeping everything they said,
/// asked for and published.
///
/// An output slot holds one message per execution and an undrained one is gone,
/// so each wake is drained as it happens.
fn coast(cog: &mut SessionTestWrapper, from_ns: i64, wakes: usize) -> Ran {
    let mut ran = Ran {
        asks: Vec::new(),
        told: Vec::new(),
        published: Vec::new(),
    };
    let mut at = from_ns;
    for _ in 0..wakes {
        ran.asks.extend(drive(cog, at));
        ran.told.extend(said(cog));
        ran.published.extend(publishes(cog));
        at += LAPSE_NS;
    }
    ran
}

/// Drive `wakes` executions at the wake floor with the machine reading as
/// `present`, keeping everything they said, asked for and published.
///
/// [`coast`]'s sibling for the cases about the one piece of evidence a stow
/// turns on: what the driver publishes is the case's own reading rather than the
/// heartbeat's, which carries no pose at all.
fn coast_reading(
    cog: &mut SessionTestWrapper,
    from_ns: i64,
    wakes: usize,
    present: &[f64; JOINT_COUNT],
    missing: u16,
) -> Ran {
    let mut ran = Ran {
        asks: Vec::new(),
        told: Vec::new(),
        published: Vec::new(),
    };
    let mut at = from_ns;
    for _ in 0..wakes {
        reads(cog, at, present, missing);
        assert!(stepped(cog, at), "the session was expected to wake",);
        ran.asks.extend(asked(cog));
        ran.told.extend(said(cog));
        ran.published.extend(publishes(cog));
        at += LAPSE_NS;
    }
    ran
}

/// The stow the slot holds, which is the schedule the machine is under command
/// to run.
///
/// Read off the slot rather than off the message: the publish carries the record
/// whole, which the cases above pin, and what is asserted here is what the
/// record says.
#[derive(Clone, Copy, PartialEq, Debug)]
struct Stow {
    engaged: bool,
    epoch: u32,
    steps: usize,
    overlays: usize,
    kind: StepKindWire,
    posture: PostureWire,
    start_ns: i64,
    end_ns: i64,
}

/// The one step the slot's schedule holds, as a stow.
fn stow_held(cog: &SessionTestWrapper) -> Stow {
    let schedule = cog.state_sess().schedule();
    let step = schedule.steps().iter().next().expect("one step");
    Stow {
        engaged: schedule.engaged(),
        epoch: schedule.epoch(),
        steps: schedule.steps().len(),
        overlays: schedule.overlays().len(),
        kind: step.kind(),
        posture: step.posture(),
        start_ns: step.start().as_nanos(),
        end_ns: step.end().as_nanos(),
    }
}

/// A grabbed head is carried down under control, and the machine is let go of at
/// rest.
///
/// The whole of the rest-class rung, in order: the condition is recorded, the
/// response is selected, the machine enters the maneuver, and the stow goes out
/// as the one thing the tick is under command to do -- one step to the fold,
/// spanning the whole of the clock the maneuver was opened with, with no overlay
/// riding it. Then it is not asked for again: a stow is asked for once per stow
/// and not once per wake, which is what the maneuver's record of the epoch it
/// commanded under is for. The machine reading as folded ends it, and what the
/// record says is that the head came down.
#[test]
fn a_grabbed_head_is_stowed_under_control_and_the_machine_let_go_at_rest() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    // The engagement's own story is drained first, so what the wakes below say
    // is the maneuver's.
    everything(&mut cog, FIRST_WAKE + 300 * 1_000_000);
    let engaged_epoch = cog.state_sess().schedule().epoch();

    let grabbed = FIRST_WAKE + 1_000 * 1_000_000;
    cog.publish_fault(
        &raise(
            FaultKindWire::HEAD_OBSTRUCTED,
            JointRefWire::LEG_0,
            grabbed,
            0.2,
        ),
        SyncTime::from_nanos(grabbed),
    );
    let ran = coast(&mut cog, grabbed, 4);

    assert_eq!(
        kinds(&ran.told),
        vec![
            ReportKindWire::FAULT_RECORDED,
            ReportKindWire::RESPONSE_TAKEN,
            ReportKindWire::PHASE_CHANGED,
            ReportKindWire::SCHEDULE_PUBLISHED,
        ],
        "the condition, the answer, the phase and the stow, in that order: {:?}",
        ran.told,
    );
    assert_eq!(
        ran.told[1].a,
        u32::from(ResponseKindWire::from(ResponseKind::SlowStowToRest).0),
    );
    assert_eq!(ran.told[2].a, u32::from(SessionPhaseWire::WINDING_DOWN.0));
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::WINDING_DOWN,
        "the machine is being carried down",
    );
    assert_eq!(
        stow_held(&cog),
        Stow {
            engaged: true,
            epoch: engaged_epoch + 1,
            steps: 1,
            overlays: 0,
            kind: StepKindWire::BASE_POSTURE,
            posture: PostureWire::STOW,
            start_ns: grabbed,
            end_ns: grabbed + STOW_BUDGET_NS,
        },
        "one step to the fold, cut from the whole of the maneuver's clock",
    );
    assert_eq!(
        ran.published.len(),
        1,
        "the stow is published once: {:?}",
        ran.published,
    );
    assert!(
        ran.asks.is_empty(),
        "a machine being carried down is asked for nothing over the bus: {:?}",
        ran.asks,
    );

    // A script arriving mid-maneuver is refused: the machine is busy with a
    // session that is ending.
    cog.publish_script(
        &one_step_script(9, grabbed),
        SyncTime::from_nanos(grabbed + 5 * LAPSE_NS),
    );
    let quiet = coast(&mut cog, grabbed + 5 * LAPSE_NS, 4);
    assert_eq!(
        kinds(&quiet.told),
        vec![ReportKindWire::SCRIPT_REFUSED],
        "nothing else happened while the stow ran: {:?}",
        quiet.told,
    );
    assert_eq!(quiet.told[0].b, u32::from(RefusalReasonWire::NOT_RESTING.0),);
    assert!(
        quiet.published.is_empty(),
        "and the stow was not asked for again: {:?}",
        quiet.published,
    );

    // The machine reads as folded, which is the one thing that ends a stow.
    let stowed_at = grabbed + 10 * LAPSE_NS;
    wake_folded(&mut cog, stowed_at);
    let outcome = said(&mut cog).expect("the maneuver's own record");
    assert_eq!(outcome.kind, ReportKindWire::WINDDOWN_OUTCOME);
    assert_eq!(
        outcome.a,
        u32::from(WindDownOutcomeWire::COMPLETED.0),
        "the head came down under control",
    );
    assert_eq!(outcome.b, 0, "and the next script may engage the machine");
    assert!(
        (outcome.detail - (STOW_BUDGET_NS - (stowed_at - grabbed)) as f64 / 1e9).abs() < 1e-9,
        "and it ended with the rest of its clock in hand: {outcome:?}",
    );
    let asked = asked(&mut cog).expect("the machine is let go of");
    assert_eq!(asked.kind, SessionCmdKindWire::TORQUE_OFF_NOW);
    assert_eq!(
        publishes(&mut cog),
        vec![Published {
            engaged: false,
            epoch: engaged_epoch + 2,
            steps: 1,
        }],
        "and the tick is told nobody is running a schedule any more",
    );
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::RESTING);
    assert!(!cog.state_sess().winddown().active());

    // The session is over: the driver confirms the release, and the next script
    // is a fresh engagement.
    cog.publish_evt(
        &event(EventKindWire::TORQUE_OFF_CONFIRMED, stowed_at),
        SyncTime::from_nanos(stowed_at),
    );
    let after = coast(&mut cog, stowed_at + LAPSE_NS, 3);
    assert!(after.told.contains(&Said {
        time_ns: stowed_at,
        kind: ReportKindWire::TORQUE_OFF_CONFIRMED,
        a: 0,
        b: 0,
        detail: 0.0,
    }));
    cog.publish_script(
        &one_step_script(11, stowed_at),
        SyncTime::from_nanos(stowed_at + 5 * LAPSE_NS),
    );
    let next = coast(&mut cog, stowed_at + 5 * LAPSE_NS, 1);
    assert_eq!(
        kinds(&next.told),
        vec![ReportKindWire::SCRIPT_ACCEPTED],
        "the next script is a new engagement: {:?}",
        next.told,
    );
}

/// A head servo that dropped out is carried down on what is left, and the
/// machine is parked.
///
/// The park-class rung. The maneuver is the same maneuver; what differs is where
/// it leaves the machine, and the row that says so is the outcome's own. Nothing
/// engages a parked machine until an operator has been.
#[test]
fn a_released_head_servo_is_stowed_on_what_is_left_and_the_machine_parked() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    everything(&mut cog, FIRST_WAKE + 300 * 1_000_000);

    let dropped = FIRST_WAKE + 1_000 * 1_000_000;
    cog.publish_fault(
        &raise(
            FaultKindWire::HEAD_SERVO_FAULT,
            JointRefWire::LEG_2,
            dropped,
            0.0,
        ),
        SyncTime::from_nanos(dropped),
    );
    let ran = coast(&mut cog, dropped, 4);
    assert_eq!(
        ran.told[1].a,
        u32::from(ResponseKindWire::from(ResponseKind::MaskedSlowStowToPark).0),
    );
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::WINDING_DOWN);
    assert_eq!(ran.published.len(), 1, "the stow went out");

    let stowed_at = dropped + 6 * LAPSE_NS;
    wake_folded(&mut cog, stowed_at);
    let outcome = said(&mut cog).expect("the maneuver's own record");
    assert_eq!(outcome.kind, ReportKindWire::WINDDOWN_OUTCOME);
    assert_eq!(outcome.a, u32::from(WindDownOutcomeWire::COMPLETED.0));
    assert_eq!(
        outcome.b, 1,
        "a machine nothing engages until an operator has been",
    );
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);
    assert!(cog.state_sess().torque_off_pending());

    cog.publish_script(
        &one_step_script(13, stowed_at),
        SyncTime::from_nanos(stowed_at + LAPSE_NS),
    );
    // The maneuver's own story is still draining, so the refusal is found among
    // what the wakes after it said rather than at the head of them.
    let refused = coast(&mut cog, stowed_at + LAPSE_NS, 4)
        .told
        .into_iter()
        .find(|report| report.kind == ReportKindWire::SCRIPT_REFUSED)
        .expect("the script was answered");
    assert_eq!(refused.b, u32::from(RefusalReasonWire::PARKED.0));
}

/// A stow the machine never reaches is ended by the one clock it was opened
/// with.
///
/// The clock is the whole of what bounds a maneuver: a head that never folds --
/// held, jammed, or moving under a hand -- is a head the session stops
/// commanding when the budget is spent, and what the record says is that the
/// maneuver fell through rather than that anything was stowed.
///
/// Every wake reads the machine completely and finitely, and standing: what ends
/// this maneuver is the clock and not an unreadable sample, so the reading the
/// evidence is taken from is one that could have said the head was folded and
/// says it is not.
#[test]
fn a_stow_the_machine_never_reaches_is_ended_by_its_clock() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    everything(&mut cog, FIRST_WAKE + 300 * 1_000_000);

    let grabbed = FIRST_WAKE + 1_000 * 1_000_000;
    cog.publish_fault(
        &raise(
            FaultKindWire::HEAD_OBSTRUCTED,
            JointRefWire::LEG_0,
            grabbed,
            0.2,
        ),
        SyncTime::from_nanos(grabbed),
    );
    let ran = coast_reading(
        &mut cog,
        grabbed,
        usize::try_from(STOW_BUDGET_NS / LAPSE_NS).expect("a budget of whole wakes") + 2,
        &standing_rows(),
        0,
    );

    let outcome = ran
        .told
        .iter()
        .find(|report| report.kind == ReportKindWire::WINDDOWN_OUTCOME)
        .expect("the maneuver ended");
    assert_eq!(
        outcome.a,
        u32::from(WindDownOutcomeWire::FELL_THROUGH.0),
        "nothing was measured at the fold, so nothing is claimed to have been \
         stowed",
    );
    assert!(
        outcome.detail <= 0.0,
        "and it ran for the whole clock: {outcome:?}",
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::RESTING,
        "a grabbed head is a rest-class ending however it ended",
    );
    assert!(
        ran.asks
            .iter()
            .all(|ask| ask.kind == SessionCmdKindWire::TORQUE_OFF_NOW),
        "the only thing asked of the driver is the release: {:?}",
        ran.asks,
    );
    assert_eq!(
        ran.published.len(),
        2,
        "the stow, and the schedule nobody is running: {:?}",
        ran.published,
    );
}

/// A stow nobody can bound is not run: the machine is let go of where it stands.
///
/// The maneuver's clock comes out of the deployment's own configuration, and a
/// number that is no length of time is one no maneuver can be carried out under.
/// What answers such a condition is the rung's own disposition without the
/// maneuver: torque comes off, and where the machine is left is what the
/// condition asked for -- a grabbed head is rest-class however it was answered.
/// The alternative is the phase nobody steps out of, holding torque under a
/// maneuver that was never opened.
#[test]
fn a_stow_budget_that_is_no_clock_lets_go_of_the_machine_instead() {
    let mut cog = resting_session();
    let mut params = session_params();
    // What `duration_from_nanos` refuses, which is what a deployment that shipped
    // a nonsense number looks like from in here.
    params.set_stow_budget_ns(-1);
    cog.set_config_params(&params);
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    everything(&mut cog, FIRST_WAKE + 300 * 1_000_000);
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::ACTIVE);
    let engaged_epoch = cog.state_sess().schedule().epoch();

    let grabbed = FIRST_WAKE + 1_000 * 1_000_000;
    cog.publish_fault(
        &raise(
            FaultKindWire::HEAD_OBSTRUCTED,
            JointRefWire::LEG_0,
            grabbed,
            0.2,
        ),
        SyncTime::from_nanos(grabbed),
    );
    let asked = drive(&mut cog, grabbed).expect("the wake answered the condition");

    assert_eq!(
        asked.kind,
        SessionCmdKindWire::TORQUE_OFF_NOW,
        "the machine is let go of on the wake the condition arrived",
    );
    assert!(cog.state_sess().torque_off_pending());
    assert!(
        !cog.state_sess().winddown().active(),
        "no maneuver was opened",
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::RESTING,
        "and the rung's own disposition still says where the machine is left",
    );
    assert_eq!(
        publishes(&mut cog),
        vec![Published {
            engaged: false,
            epoch: engaged_epoch + 1,
            steps: 1,
        }],
        "the tick is told nobody is running a schedule, under a fresh epoch",
    );
}

/// A partial reading of a folded machine is not evidence the head is folded.
///
/// The claim this record must never make wrongly is that the head came down
/// under control, and it is made off one sample. A reading missing a row says
/// nothing about that row -- the joint it cannot see is the one that could still
/// be held -- so the stow keeps being commanded and the maneuver's own clock is
/// what ends it.
#[test]
fn a_reading_missing_a_row_does_not_end_a_stow() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    everything(&mut cog, FIRST_WAKE + 300 * 1_000_000);
    let engaged_epoch = cog.state_sess().schedule().epoch();

    let grabbed = FIRST_WAKE + 1_000 * 1_000_000;
    cog.publish_fault(
        &raise(
            FaultKindWire::HEAD_OBSTRUCTED,
            JointRefWire::LEG_0,
            grabbed,
            0.2,
        ),
        SyncTime::from_nanos(grabbed),
    );
    // At the fold in every row that answered, and one row did not.
    let ran = coast_reading(&mut cog, grabbed, 6, &stow_rows(), 1);

    assert!(
        !kinds(&ran.told).contains(&ReportKindWire::WINDDOWN_OUTCOME),
        "nothing was claimed about a fold nobody measured whole: {:?}",
        ran.told,
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::WINDING_DOWN,
        "the machine is still being carried down",
    );
    assert_eq!(
        stow_held(&cog).epoch,
        engaged_epoch + 1,
        "under the stow it was already commanded, asked for once",
    );
    assert_eq!(
        ran.published.len(),
        1,
        "and asked for once: {:?}",
        ran.published
    );
}

/// An angle nobody can place is not evidence the head is folded either.
///
/// The same rule for the other way a reading fails to be one: a row whose angle
/// is no number is a row whose position this session does not know, and a
/// maneuver that read it as at the fold would report a controlled fold off a
/// sample that measured nothing. Two things in the path refuse it -- the
/// session's own finiteness screen, and the library's rule that a quantity
/// nobody can place is outside every bound -- so what is pinned here is the
/// property rather than either of them.
#[test]
fn a_reading_with_an_unplaceable_angle_does_not_end_a_stow() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    everything(&mut cog, FIRST_WAKE + 300 * 1_000_000);

    let grabbed = FIRST_WAKE + 1_000 * 1_000_000;
    cog.publish_fault(
        &raise(
            FaultKindWire::HEAD_OBSTRUCTED,
            JointRefWire::LEG_0,
            grabbed,
            0.2,
        ),
        SyncTime::from_nanos(grabbed),
    );
    let mut present = stow_rows();
    present[0] = f64::NAN;
    let ran = coast_reading(&mut cog, grabbed, 6, &present, 0);

    assert!(
        !kinds(&ran.told).contains(&ReportKindWire::WINDDOWN_OUTCOME),
        "nothing was claimed about a pose that is not one: {:?}",
        ran.told,
    );
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::WINDING_DOWN);
    assert_eq!(
        ran.published.len(),
        1,
        "and the stow stands as it was commanded: {:?}",
        ran.published,
    );
}

/// A servo dropping out mid-stow re-ranks the maneuver and the stow is asked for
/// again, on what is left of the same clock.
///
/// Three things at once, and each of them is the doctrine's. The maneuver is
/// judged by the ending that asks more of whoever finds the machine, so a stow
/// that began for a grabbed head and lost a servo parks; the clock is never
/// re-opened, so the second stow ends where the first one would have; and the
/// stow *is* asked for again, because a raise leaves the tick holding at the
/// setpoint it last commanded rather than carrying on down.
#[test]
fn a_servo_dropping_out_mid_stow_re_commands_it_on_the_clock_it_had() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    everything(&mut cog, FIRST_WAKE + 300 * 1_000_000);

    let grabbed = FIRST_WAKE + 1_000 * 1_000_000;
    cog.publish_fault(
        &raise(
            FaultKindWire::HEAD_OBSTRUCTED,
            JointRefWire::LEG_0,
            grabbed,
            0.2,
        ),
        SyncTime::from_nanos(grabbed),
    );
    coast(&mut cog, grabbed, 4);
    let first = stow_held(&cog);

    let dropped = grabbed + 5 * LAPSE_NS;
    cog.publish_fault(
        &raise(
            FaultKindWire::HEAD_SERVO_FAULT,
            JointRefWire::LEG_2,
            dropped,
            0.0,
        ),
        SyncTime::from_nanos(dropped),
    );
    let ran = coast(&mut cog, dropped, 3);

    assert_eq!(
        kinds(&ran.told),
        vec![
            ReportKindWire::FAULT_RECORDED,
            ReportKindWire::SCHEDULE_PUBLISHED,
        ],
        "the condition is recorded and the stow asked for again, and no second \
         response is selected: {:?}",
        ran.told,
    );
    assert_eq!(
        stow_held(&cog),
        Stow {
            epoch: first.epoch + 1,
            start_ns: dropped,
            ..first
        },
        "a fresh stow from where the machine stands, ending where the first one \
         would have",
    );
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::WINDING_DOWN,
        "still the one maneuver",
    );

    let stowed_at = dropped + 3 * LAPSE_NS;
    wake_folded(&mut cog, stowed_at);
    let outcome = said(&mut cog).expect("the maneuver's own record");
    assert_eq!(outcome.a, u32::from(WindDownOutcomeWire::COMPLETED.0));
    assert_eq!(
        outcome.b, 1,
        "the ending that asks more of whoever finds the machine is the one it is \
         judged by",
    );
    assert_eq!(cog.state_sess().phase(), SessionPhaseWire::PARKED);
}

/// A condition that stops trusting control ends the maneuver on the wake it
/// arrives on.
///
/// A stow is a maneuver commanded through the tick, and a tick that has lost the
/// feedback it steers by cannot carry one out: commanding another one would hold
/// torque on a machine nobody can command for the rest of the clock. So the
/// maneuver falls through and the machine is let go of, in the execution the
/// evidence arrived in.
#[test]
fn a_condition_control_is_not_trusted_through_ends_the_maneuver_at_once() {
    let mut cog = resting_session();
    let mut bus = Bus::healthy();
    engagement(&mut cog, &mut bus);
    everything(&mut cog, FIRST_WAKE + 300 * 1_000_000);

    let grabbed = FIRST_WAKE + 1_000 * 1_000_000;
    cog.publish_fault(
        &raise(
            FaultKindWire::HEAD_OBSTRUCTED,
            JointRefWire::LEG_0,
            grabbed,
            0.2,
        ),
        SyncTime::from_nanos(grabbed),
    );
    coast(&mut cog, grabbed, 4);

    let blind = grabbed + 5 * LAPSE_NS;
    cog.publish_fault(
        &raise(
            FaultKindWire::POSITION_FEEDBACK_LOST,
            JointRefWire::NONE,
            blind,
            0.0,
        ),
        SyncTime::from_nanos(blind),
    );
    let asked = drive(&mut cog, blind).expect("the machine is let go of at once");
    assert_eq!(asked.kind, SessionCmdKindWire::TORQUE_OFF_NOW);
    assert_eq!(
        cog.state_sess().phase(),
        SessionPhaseWire::PARKED,
        "control is not trusted, so nothing engages until an operator has been",
    );
    assert!(!cog.state_sess().winddown().active());

    let told = coast(&mut cog, blind + LAPSE_NS, 3).told;
    let outcome = told
        .iter()
        .find(|report| report.kind == ReportKindWire::WINDDOWN_OUTCOME)
        .expect("the maneuver's own record");
    assert_eq!(outcome.a, u32::from(WindDownOutcomeWire::FELL_THROUGH.0));
    assert_eq!(outcome.b, 1);
}
