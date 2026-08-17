//! The control-rate cog bodies.
//!
//! One function per cog declared in `motion.clk`, named `execute_<cog name in
//! snake case>` and taking `&mut <CogName>Dial`. The dial type, the entry point
//! that calls it and the C++ shim that calls that are all generated from the
//! same `.clk`; this file is the whole of what an author writes.
//!
//! Nothing here holds state of its own. A cog is a function over its declared
//! slots, its inputs and the execution's start time, and a `static` or a lazily
//! built cache would be a fourth input that no test could set and no restart
//! would clear. The machine's own dimensions are not state of that kind: they
//! are baked constants, shared through `reachy-kin` rather than rebuilt here per
//! execution.
//!
//! Every field of every schema type is read and written through `motion_slots`,
//! so which number lives in which field is said once for the whole system.

use brenn_reachy__cogs__motion_clk_rs::{MoverDial, MoverSignals, PoseDial, PoseSignals};
use brenn_reachy__cogs__msgs_clk_rs::{
    FaultKind, JointRef, MoverState, PoseState, Posture, SessionSchedule, StepKind,
};
use clockwork_rs::{Clear as _, SyncTime};
use core::time::Duration;
use motion_slots::{
    clear_pose, counters, joint_ref_from_row, joint_ref_of, joints_from_rows, read_motion_snap,
    read_pose, rows_from_joints, write_joints, write_motion_snap, write_pose,
};
use nalgebra::Isometry3;
use reachy_kin::{
    EnvelopeViolations, FkOptions, FkStats, LegAngles, default_geometry, forward_kinematics,
};
use reachy_motion::arm::{ArmRecord, rest_pose_seeds};
use reachy_motion::joints::{JointSet, JointVector};
use reachy_motion::postures::{neutral_targets, stow_pose_targets};
use reachy_motion::snap::FaultSnapshot;
use reachy_motion::tick::{
    CommandDisposition, CommandRejection, Fault, Mode, MotionCommand, MotionState, MoveAbort,
    TickInputs, TickOutputs, default_motion_config, motion_tick,
};
use reachy_motion::traj::{MoveDurations, Warp};
use reachy_wire::{GoalSetpoint, PoseSample};

/// Bus rows the six legs occupy: body yaw is row zero and the antennas are the
/// last two, so the cranks are the block between them.
const LEG_ROWS: core::ops::Range<usize> = 1..7;

/// What one sample was made of, in ordinary Rust.
///
/// Built per sample and kept only until the next one, because an output slot
/// carries one message per execution and a burst has to fold down to the last
/// of them. Holding it as owned data rather than writing straight into the slot
/// is what makes that fold a decision rather than a race between two writers.
struct Estimate {
    /// When the reading was taken.
    time_of_validity_ns: i64,
    /// The measured positions, radians.
    joints: JointVector,
    /// The pose found, and what the solve cost. `None` when the sample was
    /// incomplete or the solve did not converge -- the two are distinct causes
    /// but the same fact to a consumer: there is no pose for this instant.
    solved: Option<(Isometry3<f64>, FkStats)>,
}

/// Where the head is, once per sample.
///
/// Every execution produces exactly one estimate, valid or not. A consumer of a
/// pose series needs staleness to be a value it can read at the instant it
/// happened; a cog that published nothing when it could not solve would leave
/// that consumer to infer an outage from a hole in the timestamps. That holds
/// for a window of datagrams none of which decoded, too: those carry no instant
/// of their own, so the estimate is stamped with the execution's, but silence
/// there would make a codec-level outage indistinguishable from a stalled cog.
pub fn execute_pose(dial: &mut PoseDial<'_>) {
    let geometry = default_geometry();
    let options = FkOptions::default();

    // Kept in the slot, not read back out of the signal group's fold: the fold
    // covers one bounded window and restarts when the group reports, which
    // would restart these totals mid-run -- soonest during an outage that moves
    // one of them every execution, which is when they are worth reading.
    let before = PoseCounters::read(dial.states.est);
    let mut counters = before;

    // The branch of the mechanism to look on. The last pose actually found,
    // never the last one published: an invalid estimate carries no pose, and
    // seeding from one would choose an assembly mode by accident.
    let mut seed = match seed_pose(dial.states.est) {
        Ok(pose) => pose,
        Err(fallback) => {
            counters.refused_seeds += 1;
            fallback
        }
    };

    let mut latest = None;
    let mut solved_any = false;
    let mut saw_a_datagram = false;
    for packet in dial.inputs.sample.new_msgs() {
        saw_a_datagram = true;
        // A datagram that does not decode is not a sample and cannot be
        // timestamped, so no estimate can be published for it.
        let Ok((_, sample)) = PoseSample::decode(packet.bytes().as_slice()) else {
            counters.undecodable_samples += 1;
            continue;
        };

        let complete = sample.present_valid && sample.miss_mask == 0;
        let solved = if complete {
            let legs = LegAngles(
                sample.present[LEG_ROWS]
                    .try_into()
                    .expect("six leg rows between the yaw and the antennas"),
            );
            let mut pose = seed;
            match forward_kinematics(geometry, &legs, &seed, &options, &mut pose) {
                Ok(stats) => {
                    seed = pose;
                    solved_any = true;
                    Some((pose, stats))
                }
                Err(_) => {
                    // Invariant: `forward_kinematics` must not mutate `pose`
                    // on failure, so the seed stays at the last valid answer.
                    counters.fk_failures += 1;
                    None
                }
            }
        } else {
            None
        };

        latest = Some(Estimate {
            time_of_validity_ns: sample.sample_time_ns,
            joints: joints_from_rows(&sample.present),
            solved,
        });
    }

    store_seed(dial.states.est, &seed, solved_any);
    counters.store(dial.states.est);
    // Untested: no assertion in this repo covers the values a signal carries.
    // TODO(cogs-signal-report-contents)
    counters.report(&before, &mut dial.signals);

    let estimate = match latest {
        Some(estimate) => estimate,
        // Nothing in the window decoded. There is no reading and no instant of
        // its own to report one at, so the estimate is stamped with the
        // execution's start time and carries no positions -- `valid` is false
        // and the joints say nothing, which is the point: the series stays
        // continuous, and the outage is a value in it rather than a hole.
        None if saw_a_datagram => Estimate {
            time_of_validity_ns: dial.start_time().as_nanos(),
            joints: JointVector::default(),
            solved: None,
        },
        None => return,
    };

    let out = &mut dial.outputs.estimate;
    let msg = out.msg_mut();
    msg.set_time_of_validity(SyncTime::from_nanos(estimate.time_of_validity_ns));
    write_joints(msg.joints_mut(), &estimate.joints);
    msg.set_valid(estimate.solved.is_some());
    match estimate.solved {
        Some((pose, stats)) => {
            write_pose(msg, &pose);
            msg.set_fk_iters(stats.iters);
            msg.set_fk_residual(stats.residual);
        }
        None => {
            // An output slot is reused memory holding whatever the previous
            // execution left. There is no pose to write, so the pose fields are
            // zeroed rather than left carrying an older answer that `valid`
            // says nothing about.
            clear_pose(msg);
            msg.set_fk_iters(0);
            msg.set_fk_residual(0.0);
        }
    }
    out.mark_for_publish();
}

/// The pose the next solve starts from: the last one found, or the neutral pose
/// before any has been.
///
/// # Errors
///
/// The neutral pose, in the error arm, when the slot holds numbers that are not
/// a pose. Refused rather than repaired, and refused the way the crate's own
/// decoder refuses: a seed picks which configuration of the mechanism a solve
/// lands in, so reading four arbitrary numbers as a rotation is how a plausible
/// answer on the wrong assembly mode gets published. Falling back to neutral is
/// the same thing this cog does before it has ever solved, and the caller counts
/// it.
fn seed_pose(state: &PoseState) -> Result<Isometry3<f64>, Isometry3<f64>> {
    if !state.have_seed() {
        return Ok(reachy_kin::neutral_head_pose());
    }
    read_pose(state).map_err(|_| reachy_kin::neutral_head_pose())
}

/// Record the seed for the next execution.
///
/// `solved_any` is false when this execution saw no sample it could solve, and
/// then the slot is left exactly as it was: writing the same pose back would
/// cost the same and say the seed had been reconsidered.
fn store_seed(state: &mut PoseState, seed: &Isometry3<f64>, solved_any: bool) {
    if !solved_any {
        return;
    }
    state.set_have_seed(true);
    write_pose(state, seed);
}

/// What to command next, once per sample.
///
/// The decision itself is `reachy-motion`'s: this function arms a state off a
/// measured pose, works out what the session's schedule asks for at this
/// instant, hands both to [`motion_tick`], and turns what came back into a goal
/// datagram and at most one report. Nothing here decides what an obstruction is
/// or how a move is shaped.
///
/// A goal goes out on every sample of an engaged, armed, unfaulted machine --
/// including the samples where the tick emitted nothing because the setpoint
/// has not changed. That re-publication is the keep-alive the driver's dead-man
/// measures: a holding session is not a stopped one, and only a fault the tick
/// latches on stops the stream, which is how the machine reaches the minimum
/// risk condition when the loop can no longer command it.
pub fn execute_mover(dial: &mut MoverDial<'_>) {
    let cfg = default_motion_config();
    let settings = Settings::of(dial);
    let before = MoverCounters::read(dial.states.ctrl);
    let mut counters = before;

    // The whole schedule every time, so the newest message is all there is to
    // know; no message at all is a session nobody has started.
    let schedule = dial.inputs.sched.latest();
    let engaged = schedule.is_some_and(SessionSchedule::engaged);

    // The last goal this cog published, read back out of its own view rather
    // than kept in state fields: the sequence number the next datagram carries,
    // and the instant the last one named, which the publish is checked against.
    // A carrier this build cannot read is counted rather than passed over: the
    // answer is a fresh stream -- sequence from zero, no instant to check the
    // next publish against -- and a consumer watching the sequence numbers would
    // otherwise have no way to tell that restart from a commander that restarted.
    let published = match dial.inputs.own_cmd.latest() {
        None => None,
        Some(packet) => match GoalSetpoint::decode(packet.bytes().as_slice()) {
            Ok(decoded) => Some(decoded),
            Err(_) => {
                counters.refused_readback += 1;
                None
            }
        },
    };
    let last_seq = published.as_ref().map(|(header, _)| header.seq);
    let last_execute_at = published.as_ref().map(|(_, goal)| goal.execute_at_ns);

    // Read once, ticked over every sample of the window, written back once: the
    // slot is only observable between executions, and a state written per
    // sample would be the same value copied out and back for each of them.
    let mut state = restore(dial.states.ctrl, &mut counters.refused_state);
    let mut desired = Desired::of(dial.states.ctrl);
    let mut epoch_seen = dial.states.ctrl.schedule_epoch_seen();

    let mut goal_out = None;
    let mut reports = Reports::default();

    for packet in dial.inputs.sample.new_msgs() {
        // A datagram that does not decode is not a sample: it names no instant
        // and no positions, so there is no cycle to run and nothing to command
        // for it.
        let Ok((_, sample)) = PoseSample::decode(packet.bytes().as_slice()) else {
            counters.undecodable_samples += 1;
            continue;
        };
        counters.samples_seen += 1;
        let nominal = sample.nominal_time_ns;

        if !engaged {
            // Disengaged: the state dies with the engagement, and recovery is a
            // fresh one rather than a flag being cleared. Nothing is commanded,
            // so the goal stream stops and the driver's dead-man takes the
            // machine down.
            state = None;
            desired = Desired::NOTHING_DISPATCHED;
            continue;
        }

        if state.is_none() {
            // Arming, level-triggered: engaged and not armed is the whole
            // condition, so a solve that failed is retried on the next sample
            // with no edge to remember. A failure here raises nothing -- the
            // machine is not under command yet, and a pre-torque problem is
            // never a fault.
            let Some(present) = reading(&sample) else {
                continue;
            };
            let Ok(record) = ArmRecord::solve(&cfg.geom, &cfg.fk, &present, &rest_pose_seeds())
            else {
                continue;
            };
            state = Some(MotionState::new_armed(&record, JointSet::EMPTY));
            desired = Desired::NOTHING_DISPATCHED;
        }
        let state = state.as_mut().expect("armed a moment ago if not before");

        // A retarget is spent by the step that answers it, not by the schedule
        // arriving, so it cannot be lost to the gap it happens to land in: a
        // bumped epoch stands, sample over sample and across the slot, until a
        // posture step covers an instant and the machine is sent somewhere. One
        // site, so the dispatch and the consumption cannot come apart.
        let command = schedule.and_then(|schedule| {
            let retarget = schedule.epoch() != epoch_seen;
            desired
                .at(schedule, nominal)
                .filter(|asked| retarget || *asked != desired)
                .map(|asked| {
                    desired = asked;
                    epoch_seen = schedule.epoch();
                    if retarget {
                        counters.epochs_answered += 1;
                    }
                    asked.command(&settings)
                })
        });

        let before_mode = state.mode();
        let mut out = TickOutputs::default();
        motion_tick(
            cfg,
            state,
            &TickInputs {
                // Nanoseconds since the Unix epoch used directly as the
                // caller's own clock, which is stateless and non-decreasing
                // while the samples are. An instant before the epoch is not one
                // this machine runs on; read as zero it advances no move, which
                // is what the tick does with any clock that went backwards.
                now: Duration::from_nanos(u64::try_from(nominal).unwrap_or(0)),
                period: Duration::from_nanos(settings.period_ns),
                present: reading(&sample).as_ref(),
                command: command.as_ref(),
                // No health poll: this cog holds no bus and reads no error
                // bits.
                health: None,
            },
            &mut out,
        );
        reports.collect(&out, before_mode, nominal);

        // The keep-alive. Every sample of an engaged, armed machine carries a
        // goal, whether or not the tick emitted one: a setpoint unchanged is
        // still a setpoint being asked for, and a commander that fell silent
        // while merely holding would trip the driver's dead-man in the middle
        // of a session. Once the tick has latched a fault it commands nothing
        // and this stops with it, deliberately -- including a goal an earlier
        // sample of the same window decided, which is cleared rather than
        // published behind the latch: one more datagram would feed the dead-man
        // one more time and command the machine once past the point the loop
        // decided it must not be.
        if state.is_faulted() {
            goal_out = None;
        } else {
            goal_out = Some(GoalSetpoint {
                execute_at_ns: nominal.saturating_add(settings.lag_ns()),
                // Every row the tick still speaks for. A masked servo has been
                // taken out of service and is never written again.
                mask: JointSet::ALL.without(out.report.masked).bits(),
                targets: rows_from_joints(out.goal.as_ref().unwrap_or(state.last_goal())),
            });
        }
    }

    // A snapshot the slot will not hold leaves the machine unarmed rather than
    // armed over a slot that does not describe it: the next sample re-arms from
    // a measured pose, which is what a fresh engagement does anyway. The write
    // is all-or-nothing, so the slot is untouched either way.
    //
    // No configuration this cog accepts reaches the refusal: the crossing
    // refuses a length of time past what a slot's signed nanoseconds hold, and
    // every duration reaching it is bounded by the signed count the config
    // fields themselves are. The branch is the boundary's, not this cog's, and
    // it is here so a host that is not so bounded stops rather than commands
    // from a slot holding half a state.
    let mut armed = state.is_some();
    if let Some(state) = state.as_ref()
        && write_motion_snap(dial.states.ctrl.snap_mut(), &state.snapshot()).is_err()
    {
        counters.refused_state += 1;
        armed = false;
    }
    dial.states.ctrl.set_armed(armed);
    dial.states.ctrl.set_schedule_epoch_seen(epoch_seen);
    desired.store(dial.states.ctrl);

    // The burst rule, which the deterministic runner never exercises and a
    // scheduling stall online does: the goals are superseded, so the last one
    // wins the slot and the state effects of the rest are already folded into
    // the snapshot; the reports are events, so the first one wins and the rest
    // are counted rather than quietly lost.
    if let Some(goal) = goal_out {
        debug_assert!(
            last_execute_at.is_none_or(|last| goal.execute_at_ns > last),
            "a goal must name a later instant than the one before it",
        );
        let out = &mut dial.outputs.goal;
        out.msg_mut().clear();
        assert!(
            out.msg_mut()
                .try_set_bytes(&goal.encode(next_seq(last_seq))),
            "the goal carrier is too small for a GoalSetpoint datagram",
        );
        out.mark_for_publish();
        counters.goals_published += 1;
    }
    counters.faults_raised += reports.raised;
    counters.reports_dropped += reports.dropped;
    if let Some(raise) = reports.first {
        let out = &mut dial.outputs.fault;
        let msg = out.msg_mut();
        msg.set_time(SyncTime::from_nanos(raise.time_ns));
        msg.set_kind(raise.kind);
        msg.set_joint(raise.joint);
        msg.set_detail(raise.detail);
        msg.set_count(raise.count);
        out.mark_for_publish();
    }

    counters.store(dial.states.ctrl);
    // Untested: no assertion in this repo covers the values a signal carries.
    // TODO(cogs-signal-report-contents)
    counters.report(&before, &mut dial.signals);
}

/// The tick state the last execution left, or `None` when there is none to
/// restore.
///
/// A slot that does not decode is counted and answered `None`, not panicked on:
/// this is the process that commands the machine, and a cog that aborted on its
/// own memory would take the loop down with no report of why. Unarmed is the
/// safe reading — the arming path is level-triggered, so the next sample builds
/// a fresh state off a measured pose, and the goal stream stops in the meantime,
/// which is what every other loss of command does here.
fn restore(state: &MoverState, refusals: &mut u64) -> Option<MotionState> {
    if !state.armed() {
        return None;
    }
    let restored = read_motion_snap(state.snap())
        .ok()
        .and_then(|snap| MotionState::from_snapshot(&snap).ok());
    if restored.is_none() {
        *refusals += 1;
    }
    restored
}

/// The measured positions, or `None` where the sample carries no reading.
///
/// A sample the driver marked stale, or one with a row that did not answer, is
/// not a reading of anything: the tick counts it as a miss rather than being
/// handed eight good angles and one stale one.
fn reading(sample: &PoseSample) -> Option<JointVector> {
    (sample.present_valid && sample.miss_mask == 0).then(|| joints_from_rows(&sample.present))
}

/// The sequence number after the one the last datagram carried, or zero if
/// there was none.
fn next_seq(last: Option<u32>) -> u32 {
    last.map_or(0, |seq| seq.wrapping_add(1))
}

/// The grid this cog commands on, and how long a posture change takes.
///
/// Read once per execution and checked there: a scenario that asked for a
/// cycle of no length, or a move of none, would otherwise produce a plausible
/// run of a machine nobody meant to describe.
struct Settings {
    /// How many cycles ahead of the sample a goal is dated.
    lag_k: i64,
    /// The bus cycle, nanoseconds.
    period_ns: u64,
    /// How long the move to the upright posture takes.
    up: Duration,
    /// How long the move to stow takes.
    stow: Duration,
}

impl Settings {
    /// Read and check this cog's configuration.
    fn of(dial: &MoverDial<'_>) -> Self {
        let params = &dial.configs.params;
        Self {
            lag_k: i64::from(params.lag_k()),
            period_ns: length_of(params.period_ns(), "the control period"),
            up: Duration::from_nanos(length_of(
                params.up_duration_ns(),
                "the move to the up posture",
            )),
            stow: Duration::from_nanos(length_of(params.stow_duration_ns(), "the move to stow")),
        }
    }

    /// How far ahead of a sample the goal it produces is dated, nanoseconds.
    fn lag_ns(&self) -> i64 {
        self.lag_k
            .saturating_mul(i64::try_from(self.period_ns).unwrap_or(i64::MAX))
    }
}

/// A configured count of nanoseconds, as the length of time it claims to be.
///
/// Refused when it is not one. Every one of these turns a scenario-authoring
/// slip into a plausible-looking run: a machine commanded on a grid nobody
/// meant, or a move given no time to happen in, is a run whose assertions all
/// pass about something else.
fn length_of(ns: i64, what: &str) -> u64 {
    u64::try_from(ns)
        .ok()
        .filter(|ns| *ns > 0)
        .unwrap_or_else(|| panic!("{what} must be a length of time, not {ns}ns"))
}

/// The base posture last dispatched as a move.
///
/// A pair rather than a posture alone, because "nothing has been dispatched" is
/// a state of its own and the kind carries it: a step that keeps the base is
/// never dispatched, so `base_keep` is a value no dispatch produces.
#[derive(Clone, Copy, PartialEq, Eq)]
struct Desired {
    /// What the last dispatched step asked for.
    kind: StepKind,
    /// Which posture it named.
    posture: Posture,
}

impl Desired {
    /// Nothing dispatched: what a freshly armed machine has asked for.
    const NOTHING_DISPATCHED: Self = Self {
        kind: StepKind::BASE_KEEP,
        posture: Posture::STOW,
    };

    /// What the slot says was last dispatched.
    fn of(state: &MoverState) -> Self {
        Self {
            kind: state.desired_kind(),
            posture: state.desired_posture(),
        }
    }

    /// Record it for the next execution.
    fn store(self, state: &mut MoverState) {
        state.set_desired_kind(self.kind);
        state.set_desired_posture(self.posture);
    }

    /// What the schedule asks for at `nominal`, or `None` where it asks for
    /// nothing new.
    ///
    /// The step containing the instant, half-open, so two steps may share an
    /// edge without either owning it twice. A step that keeps the base, an
    /// instant no step covers, and a step this build cannot read all answer
    /// with the posture already dispatched -- the machine holds where the last
    /// move left it rather than being sent somewhere by a gap in a schedule.
    fn at(self, schedule: &SessionSchedule, nominal: i64) -> Option<Self> {
        let step = schedule
            .steps()
            .iter()
            .find(|step| (step.start().as_nanos()..step.end().as_nanos()).contains(&nominal))?;
        (step.kind() == StepKind::BASE_POSTURE).then_some(Self {
            kind: StepKind::BASE_POSTURE,
            posture: step.posture(),
        })
    }

    /// The move that reaches it.
    ///
    /// Where each posture is is the motion library's statement, not this cog's:
    /// stow is the posture the minimum risk condition names, and a host
    /// composing its own would be free to disagree with the one the bench
    /// commands and the one disarming checks.
    fn command(self, settings: &Settings) -> MotionCommand {
        let (target, duration) = match self.posture {
            Posture::UP => (neutral_targets(), settings.up),
            // Stow is the default of the vocabulary and the posture the machine
            // rests in, so a value this build does not know goes there rather
            // than to the working posture: an unknown posture must not be a
            // reason to stand up.
            _ => (stow_pose_targets(), settings.stow),
        };
        MotionCommand::MoveTo {
            target,
            durations: MoveDurations::uniform(duration),
            warp: Warp::MinJerk,
        }
    }
}

/// One thing the tick had to say, as the message that carries it.
#[derive(Clone, Copy)]
struct Raise {
    /// The nominal instant of the sample that raised it.
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

/// What an execution has to report, and how much of it fits.
///
/// An output slot carries one message per execution. Reports are events rather
/// than states -- a raise happens once and the machine is different afterwards
/// -- so the first one keeps the slot and the rest are counted, which is the
/// opposite of the goal stream's rule and for the opposite reason.
#[derive(Default)]
struct Reports {
    /// The one that will be published.
    first: Option<Raise>,
    /// How many were raised.
    raised: u64,
    /// How many lost the slot.
    dropped: u64,
}

impl Reports {
    /// Take everything this tick raised, in the order a reader would want it:
    /// a fault outranks a group taken out of service, which outranks a move
    /// abandoned, which outranks a command refused.
    fn collect(&mut self, out: &TickOutputs, before: Mode, nominal: i64) {
        // Transitions only. The tick re-reports a standing fault every cycle,
        // and a channel carrying it at the sample rate is a channel nobody can
        // read; a non-latching fault leaves the tick holding, so its detector
        // has to rebuild a whole run before it says so again, and each of those
        // is news.
        let standing = matches!(before, Mode::Faulted(held) if Some(held) == out.report.fault);
        if let Some(fault) = out.report.fault
            && !standing
        {
            self.offer(Raise::of_fault(&fault, nominal));
        }
        // A degrade with nothing newly masked is the same servos still out of
        // service, which is not news either.
        if let Some(fault) = out.report.degraded
            && !out.report.newly_masked.is_empty()
        {
            self.offer(Raise::of_fault(&fault, nominal));
        }
        if let Some(abort) = out.report.aborted {
            self.offer(Raise::of_abort(&abort, nominal));
        }
        if let CommandDisposition::Rejected(rejection) = out.report.command {
            self.offer(Raise::of_rejection(&rejection, nominal));
        }
    }

    /// Offer one raise for this execution's one slot.
    fn offer(&mut self, raise: Raise) {
        self.raised += 1;
        match self.first {
            None => self.first = Some(raise),
            Some(_) => self.dropped += 1,
        }
    }
}

impl Raise {
    /// A fault, as the numbers that classified it.
    ///
    /// Through the library's own flat form, so the message and the state slot
    /// carry one description of a fault rather than two.
    fn of_fault(fault: &Fault, nominal: i64) -> Self {
        let snap = FaultSnapshot::from(fault);
        Self {
            time_ns: nominal,
            kind: FaultKind(snap.code.as_u8()),
            joint: joint_ref_from_row(snap.joint)
                .expect("a fault names a bus row the machine has, or none"),
            detail: snap.error,
            count: snap.count,
        }
    }

    /// A move the tick abandoned.
    fn of_abort(abort: &MoveAbort, nominal: i64) -> Self {
        let (kind, joint, detail, count) = match abort {
            MoveAbort::EnvelopePath(violations) => (
                FaultKind::MOVE_ABORTED_ENVELOPE,
                JointRef::NONE,
                0.0,
                failed_checks(violations),
            ),
            MoveAbort::StepTooLarge { joint, delta } => (
                FaultKind::MOVE_ABORTED_STEP,
                joint_ref_of(*joint),
                *delta,
                0,
            ),
        };
        Self {
            time_ns: nominal,
            kind,
            joint,
            detail,
            count,
        }
    }

    /// A command the tick refused, which changed nothing.
    fn of_rejection(rejection: &CommandRejection, nominal: i64) -> Self {
        let (joint, detail, count) = match rejection {
            CommandRejection::Envelope(violations) => {
                (JointRef::NONE, 0.0, failed_checks(violations))
            }
            // No magnitude this message has a field for, and the count of
            // nothing is nothing.
            CommandRejection::Trajectory(_) => (JointRef::NONE, 0.0, 0),
            CommandRejection::AntennaUnreachable { joint, angle } => {
                (joint_ref_of(*joint), *angle, 0)
            }
            CommandRejection::StepTooLarge { joint, delta } => (joint_ref_of(*joint), *delta, 0),
        };
        Self {
            time_ns: nominal,
            kind: FaultKind::COMMAND_REJECTED,
            joint,
            detail,
            count,
        }
    }
}

/// How many envelope checks a pose failed.
///
/// The evidence an envelope refusal has is a set of failing checks rather than
/// a magnitude, so what this message can carry of it is how many -- one is a
/// pose just outside one bound, and six is a pose nowhere near the machine.
fn failed_checks(violations: &EnvelopeViolations) -> u32 {
    let flags = violations
        .unreachable
        .iter()
        .chain(violations.window.iter())
        .chain([
            &violations.margin,
            &violations.body_yaw,
            &violations.relative_yaw,
            &violations.cone,
        ]);
    u32::try_from(flags.filter(|failed| **failed).count()).unwrap_or(u32::MAX)
}

#[cfg(test)]
mod tests {
    //! What one execution has to say, as the message that carries it.
    //!
    //! The cog's own cases drive a machine through the generated wrapper, and a
    //! machine only produces the outcomes its inputs can reach: the four ways a
    //! command is refused, both ways a move is abandoned, and any two of them
    //! arriving on one tick are not all reachable from a schedule. They are all
    //! reachable here, where the mapping is a function over the tick's report.

    use super::{Raise, Reports, failed_checks};
    use brenn_reachy__cogs__msgs_clk_rs::{FaultKind, JointRef};
    use reachy_kin::EnvelopeViolations;
    use reachy_motion::joints::{JointId, JointSet};
    use reachy_motion::tick::{
        CommandDisposition, CommandRejection, Fault, Mode, MoveAbort, TickOutputs,
    };
    use reachy_motion::traj::TrajectoryError;

    /// The instant every case is about. One number, so a report carrying
    /// another one carried it from somewhere.
    const AT: i64 = 1_700_000_000_000_000_000;

    /// A pose outside two bounds, which is evidence a refusal counts rather
    /// than measures.
    fn two_violations() -> EnvelopeViolations {
        let mut window = [false; 6];
        window[3] = true;
        EnvelopeViolations {
            window,
            body_yaw: true,
            ..EnvelopeViolations::default()
        }
    }

    #[test]
    fn every_way_a_command_is_refused_carries_its_own_evidence() {
        let refused = Raise::of_rejection(&CommandRejection::Envelope(two_violations()), AT);
        assert_eq!(refused.kind, FaultKind::COMMAND_REJECTED);
        assert_eq!(refused.time_ns, AT);
        assert_eq!(refused.joint, JointRef::NONE, "the pose, not a servo");
        assert_eq!(refused.detail, 0.0, "an envelope refusal has no magnitude");
        assert_eq!(refused.count, 2, "how many checks the pose failed");

        let refused = Raise::of_rejection(
            &CommandRejection::Trajectory(TrajectoryError::NonPositiveDuration),
            AT,
        );
        assert_eq!(refused.kind, FaultKind::COMMAND_REJECTED);
        assert_eq!(refused.joint, JointRef::NONE);
        assert_eq!(refused.detail, 0.0);
        assert_eq!(refused.count, 0, "a path refused has nothing to count");

        let refused = Raise::of_rejection(
            &CommandRejection::AntennaUnreachable {
                joint: JointId::AntennaLeft,
                angle: 1600.5,
            },
            AT,
        );
        assert_eq!(
            refused.joint,
            JointRef::ANTENNA_LEFT,
            "the antenna asked for"
        );
        assert_eq!(refused.detail, 1600.5, "the arc it was asked for");
        assert_eq!(refused.count, 0);

        let refused = Raise::of_rejection(
            &CommandRejection::StepTooLarge {
                joint: JointId::Leg(4),
                delta: -0.75,
            },
            AT,
        );
        assert_eq!(refused.joint, JointRef::LEG_4, "the servo asked to jump");
        assert_eq!(refused.detail, -0.75, "how far, sign kept");
        assert_eq!(refused.count, 0);
    }

    #[test]
    fn both_ways_a_move_is_abandoned_carry_their_own_evidence() {
        let abandoned = Raise::of_abort(&MoveAbort::EnvelopePath(two_violations()), AT);
        assert_eq!(abandoned.kind, FaultKind::MOVE_ABORTED_ENVELOPE);
        assert_eq!(abandoned.time_ns, AT);
        assert_eq!(abandoned.joint, JointRef::NONE, "the path, not a servo");
        assert_eq!(abandoned.detail, 0.0);
        assert_eq!(abandoned.count, 2);

        let abandoned = Raise::of_abort(
            &MoveAbort::StepTooLarge {
                joint: JointId::BodyYaw,
                delta: 0.9,
            },
            AT,
        );
        assert_eq!(abandoned.kind, FaultKind::MOVE_ABORTED_STEP);
        assert_eq!(abandoned.joint, JointRef::BODY_YAW);
        assert_eq!(abandoned.detail, 0.9);
        assert_eq!(abandoned.count, 0);
    }

    #[test]
    fn a_joint_the_bus_has_no_row_for_names_no_servo() {
        let refused = Raise::of_rejection(
            &CommandRejection::StepTooLarge {
                joint: JointId::Leg(9),
                delta: 0.5,
            },
            AT,
        );
        assert_eq!(
            refused.joint,
            JointRef::NONE,
            "a servo this machine does not carry names none rather than another",
        );
    }

    #[test]
    fn how_many_checks_a_pose_failed_counts_every_bound() {
        assert_eq!(failed_checks(&EnvelopeViolations::default()), 0);
        let mut all = EnvelopeViolations {
            unreachable: [true; 6],
            window: [true; 6],
            margin: true,
            body_yaw: true,
            relative_yaw: true,
            cone: true,
        };
        assert_eq!(failed_checks(&all), 16, "six, six, and the four singles");
        all.unreachable = [false; 6];
        assert_eq!(failed_checks(&all), 10);
    }

    /// One tick can say more than one thing, and an execution has one slot for
    /// it. The order is what an operator needs first: a machine that must stop
    /// outranks servos taken out of service, which outranks a move abandoned,
    /// which outranks a command that changed nothing.
    #[test]
    fn a_fault_outranks_a_degrade_outranks_an_abort_outranks_a_refusal() {
        let mut out = TickOutputs::default();
        out.report.fault = Some(Fault::PositionFeedbackLost { misses: 51 });
        out.report.degraded = Some(Fault::AntennaObstructed {
            joint: JointId::AntennaRight,
            error: 0.8,
        });
        out.report.newly_masked = {
            let mut set = JointSet::EMPTY;
            set.insert(JointId::AntennaRight);
            set
        };
        out.report.aborted = Some(MoveAbort::StepTooLarge {
            joint: JointId::Leg(0),
            delta: 0.4,
        });
        out.report.command = CommandDisposition::Rejected(CommandRejection::Trajectory(
            TrajectoryError::NonPositiveDuration,
        ));

        let mut reports = Reports::default();
        reports.collect(&out, Mode::Holding, AT);
        assert_eq!(reports.raised, 4, "everything the tick said was counted");
        assert_eq!(reports.dropped, 3, "three of them lost the slot");
        let published = reports.first.expect("something was published");
        assert_eq!(
            published.kind,
            FaultKind::POSITION_FEEDBACK_LOST,
            "the machine that must stop is what goes out",
        );

        // The same tick without the fault: the next rank down takes the slot.
        out.report.fault = None;
        let mut reports = Reports::default();
        reports.collect(&out, Mode::Holding, AT);
        assert_eq!(reports.raised, 3);
        assert_eq!(reports.dropped, 2);
        assert_eq!(
            reports.first.expect("a raise").kind,
            FaultKind::ANTENNA_OBSTRUCTED,
        );

        // And without the degrade, the abandoned move outranks the refusal.
        out.report.degraded = None;
        let mut reports = Reports::default();
        reports.collect(&out, Mode::Holding, AT);
        assert_eq!(reports.raised, 2);
        assert_eq!(reports.dropped, 1);
        assert_eq!(
            reports.first.expect("a raise").kind,
            FaultKind::MOVE_ABORTED_STEP,
        );
    }

    /// A standing fault is the same fault the tick already reported, and a
    /// channel carrying it at the sample rate is a channel nobody can read.
    #[test]
    fn a_standing_fault_is_not_news_and_a_degrade_masking_nothing_new_is_not_either() {
        let standing = Fault::PositionFeedbackLost { misses: 60 };
        let mut out = TickOutputs::default();
        out.report.fault = Some(standing);

        let mut reports = Reports::default();
        reports.collect(&out, Mode::Faulted(standing), AT);
        assert_eq!(reports.raised, 0, "the same fault, still standing");
        assert!(reports.first.is_none());

        let mut reports = Reports::default();
        reports.collect(&out, Mode::Holding, AT);
        assert_eq!(reports.raised, 1, "the tick it was raised on is news");

        // A degrade whose joints were already out of service says nothing new
        // either: the mask is what changed on the raise, and it did not.
        let mut out = TickOutputs::default();
        out.report.degraded = Some(Fault::AntennaObstructed {
            joint: JointId::AntennaLeft,
            error: 0.9,
        });
        out.report.masked = JointSet::ALL;
        let mut reports = Reports::default();
        reports.collect(&out, Mode::Holding, AT);
        assert_eq!(reports.raised, 0, "nothing entered the mask on this tick");
    }
}

counters! {
    /// The run's totals, as the pose estimator keeps them.
    PoseCounters of PoseState, PoseSignals<'_>, crossing the_pose_totals_cross_their_slot {
        /// Solves that found no pose.
        fk_failures / set_fk_failures,
        /// Datagrams the codec refused.
        undecodable_samples / set_undecodable_samples,
        /// Times the seed slot held numbers that were not a pose.
        refused_seeds / set_refused_seeds,
    }
}

counters! {
    /// The run's totals, as the decision tick keeps them.
    MoverCounters of MoverState, MoverSignals<'_>, crossing the_mover_totals_cross_their_slot {
        /// Goal datagrams published.
        goals_published / set_goals_published,
        /// Reports raised.
        faults_raised / set_faults_raised,
        /// Samples decoded and ticked on.
        samples_seen / set_samples_seen,
        /// Datagrams the codec refused.
        undecodable_samples / set_undecodable_samples,
        /// Raises that lost the one report slot an execution has.
        reports_dropped / set_reports_dropped,
        /// Times the tick state in this cog's own slot could not be read back,
        /// or would not go back in.
        refused_state / set_refused_state,
        /// Times this cog's own last goal datagram would not decode, so the
        /// stream restarted its sequence.
        refused_readback / set_refused_readback,
        /// Times an epoch this cog had not answered yet was answered by a
        /// posture step. One execution sees only the latest schedule, so bumps
        /// coalesced by a gap count once: this counts epoch changes observed,
        /// not epochs the session published.
        epochs_answered / set_epochs_answered,
    }
}
