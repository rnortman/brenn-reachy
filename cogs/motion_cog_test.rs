//! Unit tests for the control-rate cogs, against the generated test wrappers.
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

use brenn_reachy__cogs__config_clk_rs::MoverParams;
use brenn_reachy__cogs__motion_clk_rs_test::{MoverTestWrapper, PoseTestWrapper};
use brenn_reachy__cogs__msgs_clk_rs::{
    FaultKind, JointRef, PoseEstimate, Posture, ScheduledStep, SessionSchedule, StepKind, TickFault,
};
use clockwork__clockwork__io__var_packet_clk_rs::{VarPacket__128, VarPacket__288};
use clockwork_rs::{Clear as _, SyncTime};
use motion_slots::{
    read_joints, read_motion_snap, read_pose, row_from_joint_ref, rows_from_joints,
};
use nalgebra::Isometry3;
use reachy_kin::{
    HeadGeometry, LegAngles, default_geometry, inverse_kinematics, neutral_head_pose,
    rest_head_pose, stow_head_pose,
};
use reachy_motion::default_motion_config;
use reachy_motion::disarm::{STOW_ANTENNAS, stow_targets};
use reachy_motion::joints::{JointId, JointSet};
use reachy_motion::snap::PoseSnapshotError;
use reachy_motion::tick::Mode;
use reachy_wire::{GoalSetpoint, JOINT_COUNT, PoseSample};
use std::time::Duration;

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

/// A complete sample reading the crank angles that hold `head`.
fn sample_at(head: &Isometry3<f64>, at_ns: i64) -> PoseSample {
    let mut present = [0.0; JOINT_COUNT];
    present[1..7].copy_from_slice(&cranks(head));
    PoseSample {
        nominal_time_ns: at_ns,
        sample_time_ns: at_ns,
        present_valid: true,
        commanded_valid: true,
        torque_off_latched: false,
        miss_mask: 0,
        present,
        commanded: [0.0; JOINT_COUNT],
    }
}

/// Hand a sample to the cog, as the driver's channel would.
fn publish(cog: &mut PoseTestWrapper, sample: &PoseSample, seq: u32, at_ns: i64) {
    let mut packet = VarPacket__288::new();
    assert!(
        packet.try_set_bytes(&sample.encode(seq)),
        "the carrier holds a PoseSample datagram",
    );
    cog.publish_sample(&packet, SyncTime::from_nanos(at_ns));
}

/// Hand the cog a datagram the codec will refuse.
///
/// The bytes are a well-formed sample with the version byte bumped past what
/// this build speaks, which is the failure a driver upgrade produces online:
/// the header is intact, the payload is unreadable, and the receiver's only
/// answer is to drop it.
fn publish_undecodable(cog: &mut PoseTestWrapper, seq: u32, at_ns: i64) {
    let mut bytes = sample_at(&neutral_head_pose(), at_ns).encode(seq);
    bytes[2] = bytes[2].wrapping_add(1);
    assert!(
        PoseSample::decode(&bytes).is_err(),
        "the case rests on these bytes not decoding",
    );

    let mut packet = VarPacket__288::new();
    assert!(packet.try_set_bytes(&bytes), "the carrier holds a datagram");
    cog.publish_sample(&packet, SyncTime::from_nanos(at_ns));
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
fn read(estimate: &PoseEstimate) -> Reading {
    let joints = read_joints(estimate.joints());
    let pos = estimate.head_pos();
    let quat = estimate.head_quat();
    Reading {
        time_of_validity_ns: estimate.time_of_validity().as_nanos(),
        joints: rows_from_joints(&joints),
        head_pos: [pos.x(), pos.y(), pos.z()],
        head_quat: [quat.w(), quat.x(), quat.y(), quat.z()],
        pose: read_pose(estimate),
        valid: estimate.valid(),
        fk_iters: estimate.fk_iters(),
        fk_residual: estimate.fk_residual(),
    }
}

/// What one execution published, or `None` if it published nothing.
fn published(cog: &mut PoseTestWrapper) -> Option<Reading> {
    cog.try_next_estimate().map(read)
}

/// Publish one sample and run one execution, returning the estimate.
fn one_sample(cog: &mut PoseTestWrapper, sample: &PoseSample, at_ns: i64) -> Reading {
    publish(cog, sample, 0, at_ns);
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
    sample.miss_mask = 1 << 3;

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
        publish(&mut cog, &sample_at(wanted, at), step as u32, T0);
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
fn an_undecodable_window_is_reported_as_an_outage_and_leaves_the_seed() {
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
    publish_undecodable(&mut cog, 1, T0 + PERIOD);
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

    assert_eq!(cog.state_est().undecodable_samples(), 1);
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
        publish_undecodable(&mut cog, u32::try_from(step).unwrap(), at);
        assert!(cog.execute(SyncTime::from_nanos(at)));
    }
    assert_eq!(cog.state_est().undecodable_samples(), 3);
    assert_eq!(cog.state_est().fk_failures(), 0);
    assert_eq!(cog.state_est().refused_seeds(), 0);

    // A sample nobody can place: a solve that answered nothing, counted apart
    // from a datagram that was never a sample.
    let mut broken = sample_at(&rest_head_pose(), T0 + 3 * PERIOD);
    broken.present[2] = f64::NAN;
    let estimate = one_sample(&mut cog, &broken, T0 + 3 * PERIOD);
    assert!(!estimate.valid);
    assert_eq!(cog.state_est().fk_failures(), 1);
    assert_eq!(cog.state_est().undecodable_samples(), 3, "still the run's");

    // A sample that solves moves neither counter.
    let good = one_sample(&mut cog, &sample_at(&rest_head_pose(), T0 + 4 * PERIOD), T0);
    assert!(good.valid);
    assert_eq!(cog.state_est().fk_failures(), 1);
    assert_eq!(cog.state_est().undecodable_samples(), 3);
    assert_eq!(cog.state_est().refused_seeds(), 0);
}

#[test]
fn an_undecodable_datagram_does_not_hide_the_samples_around_it() {
    let mut cog = pose_cog();
    cog.initialize(SyncTime::from_nanos(T0));

    // The drop is per message; it does not poison the rest of the window.
    publish(&mut cog, &sample_at(&neutral_head_pose(), T0), 0, T0);
    publish_undecodable(&mut cog, 1, T0);
    publish(&mut cog, &sample_at(&stow_head_pose(), T0 + PERIOD), 2, T0);
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
    missing.miss_mask = 1;

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

/// One published goal datagram, copied out of the carrier.
#[derive(Clone, Copy)]
struct Goal {
    /// The wire sequence number it carried.
    seq: u32,
    /// The setpoint itself.
    setpoint: GoalSetpoint,
}

/// One published report, copied out of the message.
#[derive(Clone, Copy, PartialEq, Debug)]
struct Report {
    /// The instant it is about.
    time_ns: i64,
    /// What was raised.
    kind: FaultKind,
    /// The servo concerned, or none.
    joint: JointRef,
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

fn params(period_ns: i64, up_ns: i64, stow_ns: i64) -> MoverParams {
    let mut params = MoverParams::new();
    params.set_lag_k(u32::try_from(LAG).expect("a small lag"));
    params.set_period_ns(period_ns);
    params.set_up_duration_ns(up_ns);
    params.set_stow_duration_ns(stow_ns);
    params
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
    frozen: JointSet,
    /// Whether the samples carry a reading at all.
    blind: bool,
    /// The sample stream's own sequence number.
    sample_seq: u32,
}

impl Mover {
    /// A cog at stow, disengaged, on the default parameters.
    fn new() -> Self {
        Self::on(&params(PERIOD, UP_NS, STOW_NS))
    }

    /// The same, configured as written -- including numbers the cog refuses.
    fn on(params: &MoverParams) -> Self {
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
            present: rows_from_joints(
                &stow_targets(default_geometry()).expect("the baked geometry reaches stow"),
            ),
            frozen: JointSet::EMPTY,
            blind: false,
            sample_seq: 0,
        }
    }

    /// Publish a schedule, as the session cog's channel would.
    ///
    /// The steps are half-open intervals from `T0`, one per posture named, each
    /// as long as `spans` says in cycles.
    fn schedule(&mut self, engaged: bool, epoch: u32, spans: &[(i64, Option<Posture>)]) {
        let spans: Vec<(i64, StepKind, Posture)> = spans
            .iter()
            .map(|(cycles, posture)| match posture {
                Some(posture) => (*cycles, StepKind::BASE_POSTURE, *posture),
                None => (*cycles, StepKind::BASE_KEEP, Posture::STOW),
            })
            .collect();
        self.schedule_raw(engaged, epoch, &spans);
    }

    /// The same, with each step's kind and posture written as given -- including
    /// values this build's vocabulary does not declare, which is what a schedule
    /// from a newer session cog carries.
    fn schedule_raw(&mut self, engaged: bool, epoch: u32, spans: &[(i64, StepKind, Posture)]) {
        let mut schedule = SessionSchedule::new();
        schedule.set_engaged(engaged);
        schedule.set_epoch(epoch);
        {
            let mut steps = schedule.steps_mut();
            steps.clear();
            let mut start = T0;
            for (cycles, kind, posture) in spans {
                let end = start + cycles * PERIOD;
                let step: &mut ScheduledStep = steps.try_grow().expect("sixteen steps is plenty");
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

    /// Hand the cog one sample, as the driver's channel would.
    fn publish_sample(&mut self, at_ns: i64) {
        let sample = PoseSample {
            nominal_time_ns: at_ns,
            sample_time_ns: at_ns,
            present_valid: !self.blind,
            commanded_valid: true,
            torque_off_latched: false,
            miss_mask: if self.blind { u16::MAX >> 7 } else { 0 },
            present: self.present,
            commanded: [0.0; JOINT_COUNT],
        };
        let seq = self.sample_seq;
        self.sample_seq = self.sample_seq.wrapping_add(1);
        let mut packet = VarPacket__288::new();
        assert!(
            packet.try_set_bytes(&sample.encode(seq)),
            "the carrier holds a PoseSample datagram",
        );
        self.cog
            .publish_sample(&packet, SyncTime::from_nanos(at_ns));
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
        let bytes = self
            .cog
            .try_next_goal()
            .map(|packet| packet.bytes().as_slice().to_vec());
        let report = self.cog.try_next_fault().map(read_report);
        if let Some(bytes) = &bytes {
            let mut packet = VarPacket__128::new();
            assert!(packet.try_set_bytes(bytes));
            self.cog
                .publish_own_cmd(&packet, SyncTime::from_nanos(nominal));
        }
        let goal = bytes.map(|bytes| {
            let (header, setpoint) = GoalSetpoint::decode(&bytes).expect("a goal datagram");
            Goal {
                seq: header.seq,
                setpoint,
            }
        });
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
        for joint in JointSet::from_bits(goal.setpoint.mask)
            .expect("a goal names bus rows")
            .iter()
        {
            let Some(row) = joint.index() else {
                continue;
            };
            if !self.frozen.contains(joint) {
                self.present[row] = goal.setpoint.targets[row];
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
    fn at(&self, joint: JointId) -> f64 {
        self.present[joint.index().expect("a bus row")]
    }
}

/// Copy a report out of the cog's memory.
fn read_report(fault: &TickFault) -> Report {
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

/// The set of joints a mask names.
fn rows(mask: u16) -> JointSet {
    JointSet::from_bits(mask).expect("a goal names bus rows")
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
    mover.schedule(true, 1, &[(1000, Some(Posture::UP))]);
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
        goal.setpoint.execute_at_ns,
        first.nominal + LAG * PERIOD,
        "a goal names the grid instant the lag puts it at",
    );
    assert_eq!(goal.seq, 0, "the first datagram this cog ever published");
    assert_eq!(
        rows(goal.setpoint.mask),
        JointSet::ALL,
        "nothing is out of service",
    );
    assert!(first.report.is_none(), "arming is not news");

    // The antennas unfold and the head rises: the run ends somewhere other than
    // where it started, which is what makes the assertions below about *how* it
    // got there worth making.
    let cycles = mover.run(60);
    assert!(cycles.iter().all(|cycle| cycle.goal.is_some()));
    assert!(reports(&cycles).is_empty(), "a clean move raises nothing");
    for antenna in [JointId::AntennaRight, JointId::AntennaLeft] {
        let angle = mover.at(antenna);
        assert!(
            direction(angle).abs() < 1e-9,
            "{antenna} points upright, at {angle} rad",
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
        assert_eq!(goal.setpoint.execute_at_ns, cycle.nominal + LAG * PERIOD);
        if let Some(previous) = previous {
            assert!(
                goal.setpoint.execute_at_ns > previous.setpoint.execute_at_ns,
                "the stream is monotonic",
            );
            assert_eq!(
                goal.seq,
                previous.seq.wrapping_add(1),
                "one stream, in order"
            );
            for joint in JointId::ALL {
                let row = joint.index().expect("a bus row");
                let delta = (goal.setpoint.targets[row] - previous.setpoint.targets[row]).abs();
                let bound = match joint.group() {
                    reachy_motion::joints::JointGroup::Legs => step.legs,
                    reachy_motion::joints::JointGroup::BodyYaw => step.body_yaw,
                    reachy_motion::joints::JointGroup::Antennas => step.antennas,
                };
                assert!(delta <= bound, "{joint} steps {delta} rad, past {bound}");
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
            goal.setpoint.targets, arrived.setpoint.targets,
            "the setpoint it is already on, republished",
        );
        assert_eq!(goal.seq, previous.seq.wrapping_add(1), "a fresh datagram");
        assert!(goal.setpoint.execute_at_ns > previous.setpoint.execute_at_ns);
        previous = goal;
    }
    assert!(reports(&holding).is_empty());
}

#[test]
fn a_session_nobody_engaged_is_never_armed_and_never_commands() {
    let mut mover = Mover::new();
    mover.schedule(false, 1, &[(1000, Some(Posture::UP))]);

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

    mover.schedule(false, 2, &[(1000, Some(Posture::UP))]);
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
    mover.schedule(true, 3, &[(1000, Some(Posture::STOW))]);
    let fresh = mover.step();
    assert!(mover.cog.state_ctrl().armed());
    assert!(fresh.goal.is_some());
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert!(
        matches!(snap.mode, Mode::Moving { .. }),
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
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
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
    assert_eq!(report.kind, FaultKind::POSITION_FEEDBACK_LOST);
    assert_eq!(report.joint, JointRef::NONE, "the reads, not a servo");
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
    mover.schedule(false, 2, &[(1000, Some(Posture::UP))]);
    mover.step();
    mover.schedule(true, 3, &[(1000, Some(Posture::UP))]);
    let fresh = mover.step();
    assert!(
        fresh.goal.is_some(),
        "a new engagement is a new state, which commands again",
    );
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert!(!matches!(snap.mode, Mode::Faulted(_)));
}

/// A jammed antenna is a fault confined to a group: the pair goes out of
/// service together -- one antenna limp and the other posed is a machine
/// pulling a face -- the move carries on without them, and every goal after
/// says so in its mask.
#[test]
fn a_jammed_antenna_takes_the_pair_out_of_service_and_the_goals_stop_naming_them() {
    let mut mover = standing_up();
    mover.frozen = {
        let mut set = JointSet::EMPTY;
        set.insert(JointId::AntennaRight);
        set
    };

    let cycles = mover.run(60);
    let raised = reports(&cycles);
    let first = raised.first().expect("a jammed servo is reported");
    assert_eq!(first.kind, FaultKind::ANTENNA_OBSTRUCTED);
    assert_eq!(
        first.joint,
        JointRef::ANTENNA_RIGHT,
        "the servo it is about",
    );
    assert!(first.detail.abs() > 0.5, "how far it stood from its goal");

    let after = cycles
        .iter()
        .position(|cycle| cycle.report.is_some())
        .expect("it was raised");
    for cycle in &cycles[after + 1..] {
        let goal = cycle.goal.expect("the move carries on without it");
        let mask = rows(goal.setpoint.mask);
        for antenna in [JointId::AntennaRight, JointId::AntennaLeft] {
            assert!(
                !mask.contains(antenna),
                "a servo out of service is never written again",
            );
        }
        assert!(
            mask.contains(JointId::BodyYaw) && mask.contains(JointId::Leg(0)),
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
        let mut set = JointSet::EMPTY;
        for joint in JointId::ALL {
            if joint.group() == reachy_motion::joints::JointGroup::Legs {
                set.insert(joint);
            }
        }
        set
    };

    let cycles = mover.run(60);
    let raised = reports(&cycles);
    let first = raised.first().expect("a jammed crank is reported");
    assert_eq!(first.kind, FaultKind::HEAD_OBSTRUCTED);
    assert_ne!(first.joint, JointRef::NONE, "the crank it is about");

    let after = cycles
        .iter()
        .position(|cycle| cycle.report.is_some())
        .expect("it was raised");
    let held = cycles[after].goal.expect("holding is still commanding");
    for cycle in &cycles[after + 1..] {
        let goal = cycle.goal.expect("the keep-alive outlives a hold");
        assert_eq!(
            goal.setpoint.targets, held.setpoint.targets,
            "a hold re-publishes what it is on",
        );
    }
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert_eq!(snap.mode, Mode::Holding, "it holds, it does not park");
}

#[test]
fn a_schedule_that_retargets_is_dispatched_and_the_stream_does_not_break() {
    let mut mover = standing_up();
    mover.run(20);

    // Mid-move, the session asks for stow instead. The epoch is what says the
    // schedule changed; the posture is what says where to.
    mover.schedule(true, 2, &[(1000, Some(Posture::STOW))]);
    let cycles = mover.run(120);
    assert!(
        cycles.iter().all(|cycle| cycle.goal.is_some()),
        "a retarget is a new path, not a gap in the stream",
    );
    assert!(reports(&cycles).is_empty(), "a retarget refuses nothing");

    let stow = stow_targets(default_geometry()).expect("the baked geometry reaches stow");
    for joint in JointId::ALL {
        let at = mover.at(joint);
        let wanted = stow.get(joint).expect("a bus row");
        assert!(
            (at - wanted).abs() < 1e-6,
            "{joint} settled at {at} rather than {wanted}",
        );
    }
    assert_eq!(
        mover.at(JointId::AntennaRight),
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
    let arrived = mover.step().goal.expect("a goal").setpoint.targets;

    mover.schedule(true, 1, &[(1000, Some(Posture::UP))]);
    let same = mover.run(3);
    for cycle in &same {
        assert_eq!(
            cycle.goal.expect("still commanded").setpoint.targets,
            arrived,
            "the same schedule moves nothing",
        );
    }
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert_eq!(snap.mode, Mode::Holding, "no move was started");
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
    // machine is holding when the boundary arrives.
    const BOUNDARY: i64 = 45;
    let mut mover = Mover::new();
    mover.schedule(
        true,
        1,
        &[(BOUNDARY, Some(Posture::UP)), (1000, Some(Posture::STOW))],
    );

    let up = mover.run(usize::try_from(BOUNDARY - 1).expect("cycles inside the first step"));
    assert!(reports(&up).is_empty(), "a plain stand-up refuses nothing");
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert_eq!(snap.mode, Mode::Holding, "the up move is over");
    assert_eq!(mover.cog.state_ctrl().epochs_answered(), 1);
    assert_eq!(mover.cog.state_ctrl().schedule_epoch_seen(), 1);

    // Across the boundary, on the same epoch: the posture differs, so the step
    // is dispatched by the posture rather than by a retarget.
    let stowing = mover.run(3);
    assert!(
        stowing.iter().all(|cycle| cycle.goal.is_some()),
        "a step boundary is a new path, not a gap in the stream",
    );
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert!(
        matches!(snap.mode, Mode::Moving { .. }),
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
    let arrived = mover.step().goal.expect("a goal").setpoint.targets;

    // A keep span over the samples that follow, then the posture the machine is
    // already on -- the shape of the surviving-bump case, with a second bump
    // published inside the same span.
    const KEEP: usize = 6;
    let keep_until = mover.cycles_from_start() + i64::try_from(KEEP).expect("six cycles") + 1;
    let spans = [(keep_until, None), (1000, Some(Posture::UP))];
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
            cycle.goal.expect("still commanded").setpoint.targets,
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
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert!(
        matches!(snap.mode, Mode::Moving { .. }),
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
    let arrived = mover.step().goal.expect("a goal").setpoint.targets;
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert_eq!(snap.mode, Mode::Holding, "the move is over before the bump");
    assert_eq!(mover.cog.state_ctrl().schedule_epoch_seen(), 1);
    assert_eq!(
        mover.cog.state_ctrl().epochs_answered(),
        1,
        "the schedule that engaged the machine is the first epoch answered",
    );

    // The same step and the same posture at one epoch higher, covering the
    // instants the samples that follow name.
    mover.schedule(true, 2, &[(1000, Some(Posture::UP))]);
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
                .setpoint
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
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert_eq!(
        snap.mode,
        Mode::Moving {
            elapsed: Duration::from_nanos(u64::try_from(2 * PERIOD).expect("two cycles")),
        },
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
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert!(
        matches!(snap.mode, Mode::Moving { .. }),
        "one cycle short of the move's length is still moving",
    );
    mover.step();
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert_eq!(
        snap.mode,
        Mode::Holding,
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
    let arrived = mover.step().goal.expect("a goal").setpoint.targets;

    // A keep span over the samples that follow, then the posture the machine is
    // already on. The bump is published under the keep span, which asks for
    // nothing. The span is stated from where the machine now stands rather than
    // hand-counted from `T0`, and it ends one cycle past the last sample it is
    // to cover, a step's end being exclusive.
    const KEEP: usize = 4;
    let keep_cycles = i64::try_from(KEEP).expect("four cycles");
    let keep_until = mover.cycles_from_start() + keep_cycles + 1;
    mover.schedule(true, 2, &[(keep_until, None), (1000, Some(Posture::UP))]);
    let keeping = mover.run(KEEP);
    for cycle in &keeping {
        assert_eq!(
            cycle.goal.expect("still commanded").setpoint.targets,
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
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert_eq!(snap.mode, Mode::Holding, "nothing was dispatched");

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
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert!(
        matches!(snap.mode, Mode::Moving { .. }),
        "the retarget outlived the gap it landed in",
    );
}

/// A step that keeps the base holds whatever posture is already commanded, and
/// an instant no step covers does the same: neither is a reason to send the
/// machine anywhere.
#[test]
fn a_step_that_keeps_the_base_and_a_gap_both_hold() {
    let mut mover = Mover::new();
    mover.schedule(true, 1, &[(5, None), (1000, Some(Posture::UP))]);

    let keeping = mover.run(4);
    for cycle in &keeping {
        assert!(cycle.goal.is_some(), "engaged and armed is commanded");
    }
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert_eq!(snap.mode, Mode::Holding, "nothing was dispatched");

    let moving = mover.run(2);
    assert!(moving.iter().all(|cycle| cycle.goal.is_some()));
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert!(
        matches!(snap.mode, Mode::Moving { .. }),
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
        goal.setpoint.execute_at_ns,
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
fn a_datagram_the_codec_refuses_is_counted_and_ticks_nothing() {
    let mut mover = standing_up();
    mover.run(5);
    let before = mover.cog.state_ctrl().samples_seen();

    // A well-formed sample with the version byte bumped past what this build
    // speaks: the header is intact and the payload is unreadable, which is what
    // a driver upgrade looks like from here.
    let mut bytes = PoseSample {
        nominal_time_ns: mover.now + PERIOD,
        sample_time_ns: mover.now + PERIOD,
        present_valid: true,
        commanded_valid: true,
        torque_off_latched: false,
        miss_mask: 0,
        present: mover.present,
        commanded: [0.0; JOINT_COUNT],
    }
    .encode(9);
    bytes[2] = bytes[2].wrapping_add(1);
    assert!(PoseSample::decode(&bytes).is_err());

    let mut packet = VarPacket__288::new();
    assert!(packet.try_set_bytes(&bytes));
    mover.now += PERIOD;
    mover
        .cog
        .publish_sample(&packet, SyncTime::from_nanos(mover.now));
    assert!(mover.cog.execute(SyncTime::from_nanos(mover.now)));

    let cycle = mover.collect(mover.now);
    assert!(
        cycle.goal.is_none(),
        "no sample decoded, so no cycle was decided",
    );
    assert_eq!(mover.cog.state_ctrl().undecodable_samples(), 1);
    assert_eq!(
        mover.cog.state_ctrl().samples_seen(),
        before,
        "a datagram that is not a sample is not a sample",
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
    assert_eq!(state.desired_kind(), StepKind::BASE_POSTURE);
    assert_eq!(state.desired_posture(), Posture::UP);

    let snap = read_motion_snap(state.snap()).expect("a state a tick produced");
    assert!(matches!(snap.mode, Mode::Moving { .. }));
    assert_eq!(
        rows_from_joints(&snap.last_goal),
        last.setpoint.targets,
        "the goal in the slot is the goal that went out",
    );
    assert!(
        snap.trajectory.is_some(),
        "the path the move is running on outlived the execution",
    );
    assert_eq!(snap.masked, JointSet::EMPTY);
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
    assert_eq!(report.kind, FaultKind::POSITION_FEEDBACK_LOST);
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
    mover.present[JointId::AntennaRight.index().expect("a bus row")] = stranded;

    let first = mover.step();
    assert!(mover.cog.state_ctrl().armed(), "the refusal is not arming");
    let report = first.report.expect("a refused command is reported");
    assert_eq!(report.kind, FaultKind::COMMAND_REJECTED);
    assert_eq!(
        report.joint,
        JointRef::ANTENNA_RIGHT,
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
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert_eq!(snap.mode, Mode::Holding, "no move was started");
    assert_eq!(
        mover.cog.state_ctrl().faults_raised(),
        1,
        "one refusal, one raise",
    );
}

/// A move given less time than a servo can travel in is abandoned at its first
/// step, and the report names the servo and how far it was asked to jump.
#[test]
fn a_move_no_servo_could_step_through_is_abandoned_and_names_the_servo() {
    // The whole stand-up in one cycle, which every crank would have to cross in
    // one bus period.
    let mut mover = Mover::on(&params(PERIOD, PERIOD, STOW_NS));
    mover.schedule(true, 1, &[(1000, Some(Posture::UP))]);

    let cycles = mover.run(4);
    let raised = reports(&cycles);
    let first = raised.first().expect("the move was abandoned");
    assert_eq!(first.kind, FaultKind::MOVE_ABORTED_STEP);

    let row = row_from_joint_ref(first.joint).expect("a report names a bus row or none");
    let joint = JointId::from_index(usize::from(row)).expect("the servo it is about");
    let step = default_motion_config().max_step;
    let bound = match joint.group() {
        reachy_motion::joints::JointGroup::Legs => step.legs,
        reachy_motion::joints::JointGroup::BodyYaw => step.body_yaw,
        reachy_motion::joints::JointGroup::Antennas => step.antennas,
    };
    assert!(
        first.detail.abs() > bound,
        "{joint} was asked for {} rad in a cycle, past the {bound} rad bound the abort is about",
        first.detail,
    );
    assert_eq!(first.count, 0, "an abandoned move has no failed checks");

    assert!(
        cycles.iter().all(|cycle| cycle.goal.is_some()),
        "abandoning a move does not stop commanding the machine",
    );
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert_eq!(snap.mode, Mode::Holding, "it holds where it stood");
    assert!(snap.trajectory.is_none(), "the path was dropped");
}

/// A posture this build's vocabulary does not declare is not a reason to stand
/// up: stow is where the machine rests and where the minimum risk condition is,
/// so an unrecognised value goes there.
#[test]
fn a_posture_this_build_does_not_know_goes_to_stow() {
    let mut mover = standing_up();
    mover.run(60);
    assert!(
        (mover.at(JointId::AntennaRight) - STOW_ANTENNAS[0]).abs() > 1.0,
        "the machine is up, so stow is somewhere else",
    );

    // A number no enumerator of this build's vocabulary carries, which is what a
    // schedule from a newer session cog would hold.
    mover.schedule(true, 2, &[(1000, Some(Posture(200)))]);
    let cycles = mover.run(140);
    assert!(
        cycles.iter().all(|cycle| cycle.goal.is_some()),
        "an unknown posture is a move, not a gap in the stream",
    );

    let stow = stow_targets(default_geometry()).expect("the baked geometry reaches stow");
    for joint in JointId::ALL {
        let at = mover.at(joint);
        let wanted = stow.get(joint).expect("a bus row");
        assert!(
            (at - wanted).abs() < 1e-6,
            "{joint} settled at {at} rather than the {wanted} rad stow asks for",
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
            (5, StepKind(200), Posture::UP),
            (1000, StepKind::BASE_POSTURE, Posture::UP),
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
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert_eq!(snap.mode, Mode::Holding, "nothing was dispatched");

    let moving = mover.run(2);
    assert!(moving.iter().all(|cycle| cycle.goal.is_some()));
    let snap = read_motion_snap(mover.cog.state_ctrl().snap()).expect("a state a tick produced");
    assert!(
        matches!(snap.mode, Mode::Moving { .. }),
        "the posture step that follows is dispatched",
    );
}

/// The read-back is where the next sequence number and the instant the next
/// publish is checked against come from. A carrier this build cannot read leaves
/// both unknown, so the stream restarts -- and the restart is counted, because
/// from outside it is indistinguishable from a commander that restarted.
#[test]
fn a_read_back_the_codec_refuses_is_counted_and_restarts_the_stream() {
    let mut mover = standing_up();
    let cycles = mover.run(3);
    let last = cycles.last().expect("a run").goal.expect("a goal");
    assert_eq!(last.seq, 2, "three datagrams, numbered from zero");
    assert_eq!(mover.cog.state_ctrl().refused_readback(), 0);

    // The cog's own last datagram with its version byte bumped past what this
    // build speaks: intact bytes, an unreadable payload.
    let mut bytes = last.setpoint.encode(last.seq);
    bytes[2] = bytes[2].wrapping_add(1);
    assert!(GoalSetpoint::decode(&bytes).is_err());
    let mut packet = VarPacket__128::new();
    assert!(packet.try_set_bytes(&bytes));
    mover
        .cog
        .publish_own_cmd(&packet, SyncTime::from_nanos(mover.now));

    let cycle = mover.step();
    assert_eq!(
        mover.cog.state_ctrl().refused_readback(),
        1,
        "the refusal is counted where a reader can see it",
    );
    let goal = cycle.goal.expect("the machine is still commanded");
    assert_eq!(
        goal.seq, 0,
        "no sequence to carry on from, so the stream starts again",
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
        let mut set = JointSet::EMPTY;
        for joint in JointId::ALL {
            if joint.group() == reachy_motion::joints::JointGroup::Legs {
                set.insert(joint);
            }
        }
        set
    };
    mover.run(11);
    mover.frozen = JointSet::ALL;

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
        FaultKind::ANTENNA_OBSTRUCTED,
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
    mover.schedule(true, 1, &[(1000, Some(Posture::UP))]);
    mover.step();
}

#[test]
#[should_panic(expected = "execute() failed")]
fn a_control_period_running_backwards_is_refused() {
    let mut mover = Mover::on(&params(-PERIOD, UP_NS, STOW_NS));
    mover.schedule(true, 1, &[(1000, Some(Posture::UP))]);
    mover.step();
}

#[test]
#[should_panic(expected = "execute() failed")]
fn a_move_to_the_up_posture_given_no_time_is_refused() {
    let mut mover = Mover::on(&params(PERIOD, 0, STOW_NS));
    mover.schedule(true, 1, &[(1000, Some(Posture::UP))]);
    mover.step();
}

#[test]
#[should_panic(expected = "execute() failed")]
fn a_move_to_stow_given_no_time_is_refused() {
    let mut mover = Mover::on(&params(PERIOD, UP_NS, 0));
    mover.schedule(true, 1, &[(1000, Some(Posture::UP))]);
    mover.step();
}
