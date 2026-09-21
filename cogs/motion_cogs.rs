//! The control-rate cog bodies.
//!
//! One function per cog declared in `motion.clk`, named `execute_<cog name in
//! snake case>` and taking `&mut <CogName>Dial`. The dial type, the entry point
//! that calls it and the C++ shim that calls that are all generated from the
//! same `.clk`; this file and the session's own modules beside it are the whole
//! of what an author writes. The session sits in modules of its own because it
//! shares nothing with the two control-rate bodies but the crate the generated
//! entry points call into: `session_cog` is its body, `session_bus` the sequence
//! it drives over the driver, and `session_ladder` what it does about evidence.
//!
//! Nothing here holds state of its own. A cog is a function over its declared
//! slots, its inputs and the execution's start time, and a `static` or a lazily
//! built cache would be a fourth input that no test could set and no restart
//! would clear. The machine's own dimensions are not state of that kind: they
//! are baked constants, shared through `reachy-kin` rather than rebuilt here per
//! execution.
//!
//! Every message this file writes is written through its validated view: one
//! `clear_valid` or `validate` at the slot, then plain fields. Which number
//! lives in which field is the schema's own statement, and the readings of a
//! pose, a joint vector and a command set are `reachy-motion`'s, so a cog and
//! the library it drives cannot disagree about them.

mod mover_overlay;
mod session_cog;
// Public for the one figure the scenario harness cannot derive for itself: the
// clocks a base move runs on once this cog has floored them, which is what says
// whether a scripted step leaves the antennas time to arrive.
pub use mover_overlay::{Goal, floored_clocks, planned_path};
// Public for one figure each, both read by the scenario harness: the
// provisioning grid, whose readable cells are most of what the start-up survey
// costs in transactions, and the bus cycle the session's staleness window counts
// in, which the harness checks against the cycle the simulated driver is built
// with.
pub mod session_bus;
pub mod session_ladder;
mod session_stow;

pub use session_cog::execute_session;

use brenn_reachy__cogs__config_clk_rs::{MoverParams, PoseLibraryConfig, ServoGains, ServoProfile};
use brenn_reachy__cogs__motion_clk_rs::{MoverDial, MoverSignals, PoseDial, PoseSignals};
use brenn_reachy__cogs__mover_clk_rs::MoverStateWire;
use brenn_reachy__cogs__pose_state_clk_rs::PoseStateWire;
use brenn_reachy__cogs__schedule_clk_rs::{SessionScheduleWire, StepKindWire};
use brenn_reachy__driver__pose_clk_rs::PoseSample;
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use brenn_reachy__motion__tick_state_clk_rs::MotionSnap;
use clockwork_rs::SyncTime;
use core::time::Duration;
use motion_slots::{configured, counters};
use mover_overlay::{Anchor, Ask, Screen};
use nalgebra::Isometry3;
use reachy_kin::{FkOptions, FkStats, LegAngles, default_geometry, forward_kinematics};
use reachy_motion::arm::{ArmRecord, Gains, GroupGains, rest_pose_seeds};
use reachy_motion::fault::{self, FaultKind};
use reachy_motion::joints::{JointRef, JointVector, flags, rows_of, vector_of, write_vector};
use reachy_motion::plant::{ClassProfile, GroupPlants, GroupProfiles};
use reachy_motion::record;
use reachy_motion::tick::{
    CommandDisposition, CommandRejection, Fault, MotionConfig, MotionMode, MoveAbort, TickInputs,
    TickOutputs, arm, last_goal, motion_tick, resume, standing_fault,
};
use reachy_motion::traj::MoveDurations;
use reachy_poses::config::screen;
use reachy_poses::library::PoseLibrary;

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

/// The setpoint one sample decided on, before it reaches the output slot.
///
/// Held as ordinary Rust for the same reason [`Estimate`] is: a burst of
/// samples decides a goal each and the slot carries one, so the fold to the
/// last of them is a decision made here rather than a sequence of writes into
/// the slot, and a window that ends behind a latched fault publishes none of
/// them.
struct GoalOut {
    /// The grid instant the setpoint is to be written at.
    execute_at_ns: i64,
    /// The rows it speaks for.
    mask: JointFlags,
    /// The angles asked for.
    targets: JointVector,
}

/// Where the head is, once per sample.
///
/// Every execution produces exactly one estimate, valid or not. A consumer of a
/// pose series needs staleness to be a value it can read at the instant it
/// happened; a cog that published nothing when it could not solve would leave
/// that consumer to infer an outage from a hole in the timestamps. That holds
/// for a window of messages none of which validated, too: those carry no
/// instant of their own, so the estimate is stamped with the execution's, but
/// silence there would make a refused window indistinguishable from a stalled
/// cog.
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
    let mut saw_a_message = false;
    for message in dial.inputs.sample.new_msgs() {
        saw_a_message = true;
        // Bytes that describe no sample cannot be timestamped, so no estimate
        // can be published for them. The boundary refusal is this one call: a
        // field this build does not know is refused here rather than at each
        // reading of it below.
        let Ok(sample) = message.validate() else {
            counters.refused_samples += 1;
            continue;
        };

        let rows = rows_of(&sample.present);
        let complete = bool::from(sample.present_valid) && flags::is_empty(sample.missing);
        let solved = if complete {
            let legs = LegAngles(
                rows[LEG_ROWS]
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
            time_of_validity_ns: sample.sample_time.as_nanos(),
            joints: vector_of(&sample.present),
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
        // Nothing in the window validated. There is no reading and no instant
        // of its own to report one at, so the estimate is stamped with the
        // execution's start time and carries no positions -- `valid` is false
        // and the joints say nothing, which is the point: the series stays
        // continuous, and the outage is a value in it rather than a hole.
        None if saw_a_message => Estimate {
            time_of_validity_ns: dial.start_time().as_nanos(),
            joints: JointVector::default(),
            solved: None,
        },
        None => return,
    };

    let out = &mut dial.outputs.estimate;
    // An output slot is reused memory holding whatever the previous execution
    // left, so the message starts cleared: an estimate carrying no pose says so
    // in the pose fields as well as in `valid`, rather than leaving an older
    // answer standing behind a flag that says nothing about it.
    let msg = out.msg_mut().clear_valid();
    msg.time_of_validity = SyncTime::from_nanos(estimate.time_of_validity_ns);
    write_vector(&mut msg.joints, &estimate.joints);
    msg.valid = estimate.solved.is_some().into();
    if let Some((pose, stats)) = estimate.solved {
        record::write_pose(&mut msg.head_pos, &mut msg.head_quat, &pose);
        msg.fk_iters = stats.iters;
        msg.fk_residual = stats.residual;
    }
    out.mark_for_publish();
}

/// The pose the next solve starts from: the last one found, or the neutral pose
/// before any has been.
///
/// # Errors
///
/// The neutral pose, in the error arm, when the slot does not read as a state at
/// all or holds numbers that are not a pose. Refused rather than repaired, and
/// refused the way every boundary in this stack refuses: a seed picks which
/// configuration of the mechanism a solve lands in, so reading four arbitrary
/// numbers as a rotation is how a plausible answer on the wrong assembly mode
/// gets published. Falling back to neutral is the same thing this cog does
/// before it has ever solved, and the caller counts it.
fn seed_pose(state: &PoseStateWire) -> Result<Isometry3<f64>, Isometry3<f64>> {
    let neutral = reachy_kin::neutral_head_pose();
    // Bytes that do not read as a state are the same answer as a seed that is no
    // pose: the neutral pose, counted. The slot is cleared and reseeded by the
    // next execution that solves; until one does there is nothing to seed from.
    let Ok(state) = state.validate() else {
        return Err(neutral);
    };
    if !bool::from(state.have_seed) {
        return Ok(neutral);
    }
    record::read_pose(&state.seed_pos, &state.seed_quat).map_err(|_| neutral)
}

/// Record the seed for the next execution.
///
/// `solved_any` is false when this execution saw no sample it could solve, and
/// then the slot is left exactly as it was: writing the same pose back would
/// cost the same and say the seed had been reconsidered.
fn store_seed(state: &mut PoseStateWire, seed: &Isometry3<f64>, solved_any: bool) {
    if !solved_any {
        return;
    }
    // Nothing outside this cog writes the slot, so bytes it cannot read are
    // memory gone wrong rather than another writer's opinion. Clearing puts it
    // back to no seed at all, which is where the cog starts; the execution's
    // totals are written over the cleared bytes at its foot.
    if state.validate_mut().is_err() {
        state.clear_valid();
    }
    // Validated twice rather than once because a borrow taken by the failing
    // call outlives the arm that clears the slot; the `expect` sits against that
    // clear, which leaves bytes validation accepts.
    let state = state
        .validate_mut()
        .expect("a slot that validated, or a cleared one");
    state.have_seed = true.into();
    record::write_pose(&mut state.seed_pos, &mut state.seed_quat, seed);
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
    let params: &MoverParams = configured(dial.configs.params, "the mover's");
    let settings = Settings::of(params);
    let cfg = motion_config(
        configured(dial.configs.profile, "the servo profile's"),
        params,
    );
    let clips = dial.configs.clips;
    // Where each pose puts the machine: the pose library's, screened here as
    // the session screens it, so the base this cog commands is the base the
    // release judges arrival at. One record, one geometry. The screen this body
    // runs is arithmetic over the dial's own numbers and solves nothing -- where
    // folded is in joint space is asked for by the consumers that judge arrival
    // there, and this cog is not one of them.
    let poses: &PoseLibraryConfig = configured(dial.configs.poses, "the mover's pose library");
    let library = screen(poses).unwrap_or_else(|error| {
        panic!("the mover's pose library states no pose to move to: {error}")
    });
    let before = MoverCounters::read(dial.states.ctrl);
    let mut counters = before;

    // The whole schedule every time, so the newest message is all there is to
    // know; no message at all is a session nobody has started.
    let schedule = dial.inputs.sched.latest();
    let engaged = schedule.is_some_and(SessionScheduleWire::engaged);

    // Which overlays this run will play, screened once against the configured
    // library rather than once per sample: the schedule is one message and its
    // windows are the same windows for every period it stands for.
    let Screen {
        windows,
        mut latched,
    } = mover_overlay::screen(clips, dial.states.ctrl, schedule, engaged, &mut counters);

    // The instant the last goal this cog published named, read back out of its
    // own view rather than kept in a state field: the channel carries the
    // setpoint itself, so the instant is a field of it and the publish below is
    // checked against it.
    let last_execute_at = dial
        .inputs
        .own_cmd
        .latest()
        .map(|goal| goal.execute_at().as_nanos());

    // The state is the slot, validated once here and handed to the tick as
    // itself: what a cog carries between executions is what the slot holds, so
    // there is nothing to read out and nothing to write back.
    //
    // A slot this build cannot read is cleared and counted, which leaves the
    // machine unarmed rather than ticking on bytes nothing wrote. Unarmed is
    // the safe reading -- the arming path is level-triggered, so the next
    // sample builds a fresh state off a measured pose, and the goal stream
    // stops in the meantime, which is what every other loss of command does
    // here.
    // The count is the whole trace a refusal leaves; which of them it was is
    // recoverable from the input log, which reproduces the whole run.
    let mut armed = dial.states.ctrl.armed();
    if dial.states.ctrl.snap_mut().validate_mut().is_err() {
        dial.states.ctrl.snap_mut().clear_valid();
        counters.refused_state += 1;
        armed = false;
    }
    let mut desired = Desired::of(dial.states.ctrl);
    let mut epoch_seen = dial.states.ctrl.schedule_epoch_seen();

    let mut goal_out = None;
    let mut reports = Reports::default();

    // Where the last period left the commanded stream. Carried across samples
    // rather than re-read per sample: the state is taken up once a sample for
    // the tick, so the two numbers the overlay layer needs off it are read
    // through that same take-up -- here before the first sample, and again at
    // every point below that writes the state. `None` until a state that has
    // been armed says where the stream stands: an unarmed slot commands nothing,
    // and a machine is armed before any window is taken up.
    let mut anchor: Option<Anchor> = None;

    // Taken up once here and again per sample below. The slot is only
    // observable between executions, so every sample of a burst ticks the same
    // state in place; what a per-sample take-up buys is the base and the overlay
    // rows beside it, which live in the same slot and cannot be borrowed
    // alongside a state held for the whole window.
    {
        let Ok(state) = dial.states.ctrl.snap_mut().validate_mut() else {
            // Bytes that did not read as a state and did not read as one after
            // being cleared either. Nothing is commanded and the refusal is
            // counted: the goal stream stopping is what takes the machine down
            // safely, and a panic here would take the loop with it and say
            // nothing.
            counters.refused_state += 1;
            dial.states.ctrl.set_armed(false);
            counters.store(dial.states.ctrl);
            counters.report(&before, &mut dial.signals);
            return;
        };
        if armed && resume(state).is_err() {
            // Readable bytes that describe no state a tick could be in -- the
            // same answer as a slot that did not validate, for the same reason.
            // TODO(refused-state-names-its-reason)
            counters.refused_state += 1;
            armed = false;
        }
        if armed {
            anchor = Some(Anchor::of(state));
        }
    }

    for message in dial.inputs.sample.new_msgs() {
        // Bytes that describe no sample name no instant and no positions, so
        // there is no cycle to run and nothing to command for them. The
        // boundary refusal is this one call.
        let Ok(sample) = message.validate() else {
            counters.refused_samples += 1;
            continue;
        };
        counters.samples_seen += 1;
        let nominal = sample.nominal_time.as_nanos();

        if !engaged {
            // Disengaged: the state dies with the engagement, and recovery is a
            // fresh one rather than a flag being cleared. Nothing is commanded,
            // so the goal stream stops and the driver's dead-man takes the
            // machine down. The base and the players go with it -- they belong
            // to the engagement that ended.
            armed = false;
            desired = Desired::NOTHING_DISPATCHED;
            latched = false;
            mover_overlay::release(dial.states.ctrl, nominal);
            continue;
        }

        let present = reading(sample);
        let held = holding(sample);

        if !armed {
            // Arming, level-triggered: engaged and not armed is the whole
            // condition, so a solve that failed is retried on the next sample
            // with no edge to remember. A failure here raises nothing -- the
            // machine is not under command yet, and a pre-torque problem is
            // never a fault.
            let Some(present) = present.as_ref() else {
                continue;
            };
            let Ok(record) = ArmRecord::solve(&cfg.geom, &cfg.fk, present, &rest_pose_seeds())
            else {
                continue;
            };
            {
                let state = snap_of(dial.states.ctrl);
                arm(state, &record, JointFlags::NONE);
                // The arming wrote the state, so the setpoint the layer would
                // hand a base over from is the one it just established.
                anchor = Some(Anchor::of(state));
            }
            // A fresh engagement is a fresh base: the record in the slot belongs
            // to whatever engagement ended, and the state this arming built has
            // no history the base could be continuous with.
            mover_overlay::release(dial.states.ctrl, nominal);
            armed = true;
            desired = Desired::NOTHING_DISPATCHED;
        }

        // A retarget is spent by the step that answers it, not by the schedule
        // arriving, so it cannot be lost to the gap it happens to land in: a
        // bumped epoch stands, sample over sample and across the slot, until a
        // base step covers an instant and the machine is sent somewhere. One
        // site, so the dispatch and the consumption cannot come apart.
        let asked = schedule.and_then(|schedule| {
            let retarget = schedule.epoch() != epoch_seen;
            desired
                .at(schedule, nominal)
                .filter(|asked| retarget || *asked != desired)
                .inspect(|&asked| {
                    desired = asked;
                    epoch_seen = schedule.epoch();
                    if retarget {
                        counters.epochs_answered += 1;
                    }
                })
        });

        // An id the deployed library does not hold is refused here and nothing
        // is commanded for it: the raise travels as a command rejection and the
        // fault ladder answers it with its own stow, whose id came off the same
        // screened library. Only the dispatch raises -- a standing goal that
        // will not resolve is the refusal already reported, and one raise per
        // sample for as long as the step stood would be a channel nobody can
        // read.
        let fresh = match asked.map(|asked| asked.goal(&library)).transpose() {
            Ok(goal) => goal.flatten(),
            Err(UnknownPose { .. }) => {
                reports.offer(Raise::of_unknown_pose(nominal));
                None
            }
        };
        let standing = desired.goal(&library).unwrap_or(None);

        // What the base does this period, and what rides on it. Three answers
        // in one call: the ordinary base move while the tick owns the base, a
        // composed setpoint while an overlay window is open, and the re-anchored
        // move that hands the base back when the last one closes.
        let commanded = mover_overlay::decide(
            cfg,
            dial.states.ctrl,
            &windows,
            latched,
            &Ask {
                now_ns: nominal,
                period: Duration::from_nanos(settings.period_ns),
                tick_hz: settings.tick_hz(),
                fresh,
                standing,
            },
            // Every armed state has one: it was read off the arming that
            // established this one, off the tick that last advanced it, or off
            // the resume this execution opened with.
            anchor.as_ref().expect("an armed machine has a setpoint"),
            &mut counters,
        );
        let command = commanded.command;

        let state = snap_of(dial.states.ctrl);
        let before_fault = standing_fault(state);
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
                present: present.as_ref(),
                commanded: held.as_ref(),
                command: command.as_ref(),
                // No health poll: this cog holds no bus and reads no error
                // bits.
                health: None,
            },
            &mut out,
        );
        reports.collect(&out, before_fault, nominal);

        // A move the state claimed and held nothing to sample: bytes something
        // other than this tick wrote, which is the same thing a slot that does
        // not resume is and is counted the same way. The tick has already
        // dropped to holding, so the machine is still under command.
        if out.report.unsampleable.is_some() {
            counters.refused_state += 1;
        }

        // A composed setpoint the tick would not have. The overlays are dropped
        // for this schedule, whole: the same clips over the same base compose
        // the same setpoint, so a layer that took its windows up again would
        // offer the tick what it has just refused, once a period, for as long as
        // the windows stayed open. The refusal itself travels on the report
        // channel like any other, and it is never a fault -- a refused command
        // changes nothing, and the base carries on alone through the next
        // sample's hand-back.
        if commanded.overlaid && matches!(out.report.command, CommandDisposition::Rejected(_)) {
            latched = true;
            counters.overlays_refused += 1;
        }

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
        if state.mode == MotionMode::Faulted {
            goal_out = None;
        } else {
            goal_out = Some(GoalOut {
                execute_at_ns: nominal.saturating_add(settings.lag_ns()),
                // Every row the tick still speaks for. A masked servo has been
                // taken out of service and is never written again.
                mask: flags::without(flags::all(), out.report.masked),
                targets: out.goal.unwrap_or_else(|| last_goal(state)),
            });
        }

        // The tick wrote the state, so the next sample's layer reads its
        // setpoint and its margin off this take-up rather than paying for
        // another one.
        anchor = Some(Anchor::of(state));
    }

    dial.states.ctrl.set_armed(armed);
    dial.states.ctrl.set_schedule_epoch_seen(epoch_seen);
    dial.states.ctrl.set_overlay_latch(latched);
    desired.store(dial.states.ctrl);

    // The burst rule, which the deterministic runner never exercises and a
    // scheduling stall online does: the goals are superseded, so the last one
    // wins the slot and the state effects of the rest are already in the state
    // slot; the reports are events, so the first one wins and the rest are
    // counted rather than quietly lost.
    if let Some(goal) = goal_out {
        debug_assert!(
            last_execute_at.is_none_or(|last| goal.execute_at_ns > last),
            "a goal must name a later instant than the one before it",
        );
        let out = &mut dial.outputs.goal;
        let msg = out.msg_mut().clear_valid();
        msg.execute_at = SyncTime::from_nanos(goal.execute_at_ns);
        msg.mask = goal.mask;
        write_vector(&mut msg.targets, &goal.targets);
        out.mark_for_publish();
        counters.goals_published += 1;
    }
    counters.faults_raised += reports.raised;
    counters.reports_dropped += reports.dropped;
    if let Some(raise) = reports.first {
        let out = &mut dial.outputs.fault;
        let msg = out.msg_mut().clear_valid();
        msg.time = SyncTime::from_nanos(raise.time_ns);
        msg.kind = raise.kind;
        msg.joint = raise.joint;
        msg.detail = raise.detail;
        msg.count = raise.count;
        msg.envelope_checks = raise.envelope_checks;
        out.mark_for_publish();
    }

    counters.store(dial.states.ctrl);
    // Untested: no assertion in this repo covers the values a signal carries.
    // TODO(cogs-signal-report-contents)
    counters.report(&before, &mut dial.signals);
}

/// The tick state in this cog's slot, taken up for one sample.
///
/// The whole of what a per-sample take-up costs: validation is a check over
/// bytes rather than a copy of them, and the execution has already established
/// that these bytes read as a state.
///
/// # Panics
///
/// If they do not, which the caller ruled out before its first sample.
fn snap_of(state: &mut MoverStateWire) -> &mut MotionSnap {
    state
        .snap_mut()
        .validate_mut()
        .expect("a state this execution has already validated")
}

/// The setpoint the driver reported holding, or `None` where it held none.
///
/// What the modelled plant is stepped against: the setpoint that actually
/// reached the servos, keep-alive rewrites and refusals included, rather than
/// the goal this cog composed. A cycle the driver held nothing on -- before the
/// first goal, after a release, behind a latched torque-off -- leaves the model
/// nothing to chase and re-seeds it from the next reading.
fn holding(sample: &PoseSample) -> Option<JointVector> {
    bool::from(sample.commanded_valid).then(|| vector_of(&sample.commanded))
}

/// The measured positions, or `None` where the sample carries no reading.
///
/// A sample the driver marked stale, or one with a row that did not answer, is
/// not a reading of anything: the tick counts it as a miss rather than being
/// handed eight good angles and one stale one.
fn reading(sample: &PoseSample) -> Option<JointVector> {
    (bool::from(sample.present_valid) && flags::is_empty(sample.missing))
        .then(|| vector_of(&sample.present))
}

/// The tick's configuration, built once from the profile this deployment
/// commissions and shared from then on.
///
/// Built once: everything in it is either a hardware fact the motion library
/// states or a number a config file states before the first execution, so a cog
/// that rebuilt it every sample would pay for it at control rate. A profile handed
/// in after the first call is ignored rather than refused — in a deployment
/// there is only ever one, since a cog's config is read from a file before the
/// first execution and nothing writes it afterwards, and in a test binary the
/// first case to run fixes it for the rest.
///
/// The Mover's own parameters and not a scattering of scalars off them: the
/// grid and the detector's arming are both fields of the one file, and a new
/// knob is then a field read here rather than another positional argument on
/// two functions and every call site of both.
///
/// The period is the Mover's own configured grid and never a constant: the
/// plant model's two limits are per-period distances, so a model built for one
/// grid and stepped on another is wrong in proportion to the ratio.
///
/// # Panics
///
/// For a profile register of zero, which disables the generator on the servo,
/// and for a period that is no grid. The tick would then be judging every joint
/// against a trajectory nothing runs, which is worse than a process that
/// refuses to start: the session refuses the same pair before it commissions
/// anything, so the two readers of the one file stop for the same reason.
fn motion_config(profile: &ServoProfile, params: &MoverParams) -> &'static MotionConfig {
    static COMMISSIONED: std::sync::OnceLock<MotionConfig> = std::sync::OnceLock::new();
    COMMISSIONED.get_or_init(|| {
        commissioned_config(profile, params).unwrap_or_else(|error| {
            panic!(
                "the servo profile is {:?} on a {} ns grid, which is no plant to judge \
                 a joint against: {error}",
                group_profiles(profile),
                params.period_ns,
            )
        })
    })
}

/// The configuration this profile and these mover parameters describe, or why
/// they describe no machine.
///
/// Fallible entry point: `motion_config` panics on refusal, and the
/// configuration is built once per process, so a test that drove the panic
/// would poison a `OnceLock` for every case after it.
///
/// # Errors
///
/// The reason the profile's pairs and the parameters' period are no plant.
pub fn commissioned_config(
    profile: &ServoProfile,
    params: &MoverParams,
) -> Result<MotionConfig, reachy_motion::plant::GroupPlantError> {
    let plant = GroupPlants::from_profiles(&group_profiles(profile), params.period_ns)?;
    let mut cfg = MotionConfig {
        plant,
        ..MotionConfig::default()
    };
    cfg.tracking.armed = params.tracking_armed.into();
    Ok(cfg)
}

/// The gains file's nine scalars as the motion library's three triples.
///
/// The one place those field names and the classes are joined, for
/// [`group_profiles`]'s reason.
#[must_use]
pub fn group_gains(gains: &ServoGains) -> GroupGains {
    GroupGains {
        legs: Gains {
            p: gains.legs_p,
            i: gains.legs_i,
            d: gains.legs_d,
        },
        yaw: Gains {
            p: gains.body_yaw_p,
            i: gains.body_yaw_i,
            d: gains.body_yaw_d,
        },
        antennas: Gains {
            p: gains.antennas_p,
            i: gains.antennas_i,
            d: gains.antennas_d,
        },
    }
}

/// The configuration file's nine scalars as the motion library's three class
/// profiles.
///
/// The one place the field names and the classes are joined, so the session
/// that writes the registers and the tick that models them read the file the
/// same way.
#[must_use]
pub fn group_profiles(profile: &ServoProfile) -> GroupProfiles {
    GroupProfiles {
        legs: ClassProfile {
            acceleration: profile.legs_profile_acceleration,
            velocity: profile.legs_profile_velocity,
            following_lag_us: profile.legs_following_lag_us,
        },
        yaw: ClassProfile {
            acceleration: profile.body_yaw_profile_acceleration,
            velocity: profile.body_yaw_profile_velocity,
            following_lag_us: profile.body_yaw_following_lag_us,
        },
        antennas: ClassProfile {
            acceleration: profile.antennas_profile_acceleration,
            velocity: profile.antennas_profile_velocity,
            following_lag_us: profile.antennas_following_lag_us,
        },
    }
}

/// The grid this cog commands on.
///
/// Read once per execution and checked there: a scenario that asked for a
/// cycle of no length, or a move of none, would otherwise produce a plausible
/// run of a machine nobody meant to describe.
struct Settings {
    /// How many cycles ahead of the sample a goal is dated.
    lag_k: i64,
    /// The bus cycle, nanoseconds.
    period_ns: u64,
}

impl Settings {
    /// Read and check this cog's configuration.
    fn of(params: &MoverParams) -> Self {
        Self {
            lag_k: i64::from(params.lag_k),
            period_ns: length_of(params.period_ns, "the control period"),
        }
    }

    /// The grid rate, hertz -- what a plan's per-tick step bounds are judged
    /// at.
    ///
    /// Derived from the period rather than configured beside it: two numbers
    /// stating one grid is one of them being wrong.
    fn tick_hz(&self) -> f64 {
        1e9 / self.period_ns as f64
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

/// A pace off a schedule row, as the length of time it states.
///
/// Non-positive is taken as no time at all. The session screens every base row
/// it publishes for a positive pace, so the case is bytes gone wrong rather than
/// a schedule; non-positive maps to zero here, and the caller must refuse a
/// zero clock -- a move of no duration carries no path.
fn pace_of(pace: clockwork_rs::Duration) -> Duration {
    Duration::from_nanos(u64::try_from(pace.as_nanos()).unwrap_or(0))
}

/// The base move last dispatched: where it sends the machine, and how fast.
///
/// A triple rather than a pose alone. "Nothing has been dispatched" is a state
/// of its own and the kind carries it -- a step that keeps the base is never
/// dispatched, so `base_keep` is a value no dispatch produces -- and the pace is
/// part of what a step asks for, so a row naming the pose already commanded at
/// another pace is a fresh ask and dispatches.
#[derive(Clone, Copy, PartialEq, Eq)]
struct Desired {
    /// What the last dispatched step asked for.
    kind: StepKindWire,
    /// Which pose it named, as the deployed library numbers it.
    pose_id: u16,
    /// How long the move that step asked for takes.
    pace: Duration,
}

impl Desired {
    /// Nothing dispatched: what a freshly armed machine has asked for.
    ///
    /// The id and the pace are zero and nothing reads either: [`Desired::goal`]
    /// answers `None` for a kept base before it looks at them.
    const NOTHING_DISPATCHED: Self = Self {
        kind: StepKindWire::BASE_KEEP,
        pose_id: 0,
        pace: Duration::ZERO,
    };

    /// What the slot says was last dispatched.
    fn of(state: &MoverStateWire) -> Self {
        Self {
            kind: state.desired_kind(),
            pose_id: state.desired_pose_id(),
            pace: pace_of(state.desired_pace()),
        }
    }

    /// Record it for the next execution.
    fn store(self, state: &mut MoverStateWire) {
        state.set_desired_kind(self.kind);
        state.set_desired_pose_id(self.pose_id);
        state.set_desired_pace(clockwork_rs::Duration::from_nanos(
            i64::try_from(self.pace.as_nanos()).unwrap_or(i64::MAX),
        ));
    }

    /// What the schedule asks for at `nominal`, or `None` where it asks for
    /// nothing new.
    ///
    /// The step containing the instant, half-open, so two steps may share an
    /// edge without either owning it twice. A step that keeps the base and an
    /// instant no step covers both answer with the pose already dispatched --
    /// the machine holds where the last move left it rather than being sent
    /// somewhere by a gap in a schedule.
    fn at(self, schedule: &SessionScheduleWire, nominal: i64) -> Option<Self> {
        let step = schedule
            .steps()
            .iter()
            .find(|step| (step.start().as_nanos()..step.end().as_nanos()).contains(&nominal))?;
        (step.kind() == StepKindWire::BASE_POSTURE).then_some(Self {
            kind: StepKindWire::BASE_POSTURE,
            pose_id: step.pose_id(),
            pace: pace_of(step.pace()),
        })
    }

    /// Where this sends the base, and over what clocks — or `None` for a
    /// dispatch that names no pose, which is what nothing dispatched is.
    ///
    /// Targets come from the library, the single source of truth for where each
    /// pose sends the head.
    ///
    /// # Errors
    ///
    /// [`UnknownPose`] for an id the deployed library does not hold. A refusal
    /// and not a substitution: a head sent to the fold, or anywhere else,
    /// because an id did not resolve is a machine moving on a guess, and the
    /// answer to a command this cog cannot carry out is to say so and let the
    /// fault policy decide what the machine does about it.
    fn goal(self, library: &PoseLibrary<'_>) -> Result<Option<Goal>, UnknownPose> {
        if self.kind != StepKindWire::BASE_POSTURE {
            return Ok(None);
        }
        let (target, _) = library.targets(self.pose_id).ok_or(UnknownPose {
            pose_id: self.pose_id,
        })?;
        Ok(Some(Goal {
            target,
            durations: MoveDurations::uniform(self.pace),
        }))
    }
}

/// A base step naming a pose the deployed library does not hold.
///
/// The mover's own refusal and not [`reachy_motion::tick::CommandRejection`]'s:
/// a pose id is the library's vocabulary and this cog's, and the tick never sees
/// one — what reaches it is a target the library resolved. It travels as
/// [`FaultKind::CommandRejected`] all the same, because from the machine's side
/// it is the same event: a command that was asked for and not carried out.
///
/// Only a sidecar and a library from two different emits produce one, which the
/// payload's delivery rules out.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct UnknownPose {
    /// The index that resolved to nothing.
    pose_id: u16,
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
    /// The exact envelope checks that failed, or zero for other raises.
    envelope_checks: u32,
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
    fn collect(&mut self, out: &TickOutputs, before: Option<Fault>, nominal: i64) {
        // Transitions only. The tick re-reports a standing fault every cycle,
        // and a channel carrying it at the sample rate is a channel nobody can
        // read; a non-latching fault leaves the tick holding, so its detector
        // has to rebuild a whole run before it says so again, and each of those
        // is news.
        let standing = before.is_some() && before == out.report.fault;
        if let Some(fault) = out.report.fault
            && !standing
        {
            self.offer(Raise::of_fault(&fault, nominal));
        }
        // A degrade with nothing newly masked is the same servos still out of
        // service, which is not news either.
        if let Some(fault) = out.report.degraded
            && !flags::is_empty(out.report.newly_masked)
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
    /// Through the library's own reading of a fault, so the message, the state
    /// slot and the operator's log carry one description of it rather than
    /// three.
    fn of_fault(raised: &Fault, nominal: i64) -> Self {
        Self {
            time_ns: nominal,
            kind: fault::kind(raised),
            joint: fault::joint(raised),
            detail: fault::detail(raised),
            count: fault::count(raised),
            envelope_checks: 0,
        }
    }

    /// A move the tick abandoned.
    fn of_abort(abort: &MoveAbort, nominal: i64) -> Self {
        let (kind, joint, detail, count, envelope_checks) = match abort {
            MoveAbort::EnvelopePath(violations) => {
                let bits = violations.bits();
                (
                    FaultKind::MoveAbortedEnvelope,
                    JointRef::None,
                    0.0,
                    bits.count_ones(),
                    u32::from(bits),
                )
            }
            MoveAbort::StepTooLarge { joint, delta } => {
                (FaultKind::MoveAbortedStep, *joint, *delta, 0, 0)
            }
        };
        Self {
            time_ns: nominal,
            kind,
            joint,
            detail,
            count,
            envelope_checks,
        }
    }

    /// A base step this cog refused, which changed nothing.
    ///
    /// Takes no id, because the record has no field one belongs in: `detail` is
    /// a magnitude in the joint's own unit and an index written there would be
    /// read as one. Which id was refused is on the logged schedule slot the
    /// analyzer reads beside the raise. So the joint, the magnitude and the
    /// count are the nothing they are.
    fn of_unknown_pose(nominal: i64) -> Self {
        Self {
            time_ns: nominal,
            kind: FaultKind::CommandRejected,
            joint: JointRef::None,
            detail: 0.0,
            count: 0,
            envelope_checks: 0,
        }
    }

    /// A command the tick refused, which changed nothing.
    fn of_rejection(rejection: &CommandRejection, nominal: i64) -> Self {
        let (joint, detail, count, envelope_checks) = match rejection {
            CommandRejection::Envelope(violations) => {
                let bits = violations.bits();
                (JointRef::None, 0.0, bits.count_ones(), u32::from(bits))
            }
            // No magnitude this message has a field for, and the count of
            // nothing is nothing.
            CommandRejection::Trajectory(_) => (JointRef::None, 0.0, 0, 0),
            CommandRejection::AntennaUnreachable { joint, angle } => (*joint, *angle, 0, 0),
        };
        Self {
            time_ns: nominal,
            kind: FaultKind::CommandRejected,
            joint,
            detail,
            count,
            envelope_checks,
        }
    }
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

    use super::{Raise, Reports};
    use brenn_reachy__motion__joints_clk_rs::JointFlags;
    use reachy_kin::EnvelopeViolations;
    use reachy_motion::fault::FaultKind;
    use reachy_motion::joints::{JointRef, flags};
    use reachy_motion::tick::{
        CommandDisposition, CommandRejection, Fault, MoveAbort, TickOutputs,
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
        assert_eq!(refused.kind, FaultKind::CommandRejected);
        assert_eq!(refused.time_ns, AT);
        assert_eq!(refused.joint, JointRef::None, "the pose, not a servo");
        assert_eq!(refused.detail, 0.0, "an envelope refusal has no magnitude");
        assert_eq!(refused.count, refused.envelope_checks.count_ones());
        assert_eq!(refused.envelope_checks, u32::from(two_violations().bits()));

        let refused = Raise::of_rejection(
            &CommandRejection::Trajectory(TrajectoryError::NonPositiveDuration),
            AT,
        );
        assert_eq!(refused.kind, FaultKind::CommandRejected);
        assert_eq!(refused.joint, JointRef::None);
        assert_eq!(refused.detail, 0.0);
        assert_eq!(refused.count, 0, "a path refused has nothing to count");
        assert_eq!(refused.envelope_checks, 0);

        let refused = Raise::of_rejection(
            &CommandRejection::AntennaUnreachable {
                joint: JointRef::AntennaLeft,
                angle: 1600.5,
            },
            AT,
        );
        assert_eq!(
            refused.joint,
            JointRef::AntennaLeft,
            "the antenna asked for"
        );
        assert_eq!(refused.detail, 1600.5, "the arc it was asked for");
        assert_eq!(refused.count, 0);
        assert_eq!(refused.envelope_checks, 0);
    }

    #[test]
    fn both_ways_a_move_is_abandoned_carry_their_own_evidence() {
        let abandoned = Raise::of_abort(&MoveAbort::EnvelopePath(two_violations()), AT);
        assert_eq!(abandoned.kind, FaultKind::MoveAbortedEnvelope);
        assert_eq!(abandoned.time_ns, AT);
        assert_eq!(abandoned.joint, JointRef::None, "the path, not a servo");
        assert_eq!(abandoned.detail, 0.0);
        assert_eq!(abandoned.count, abandoned.envelope_checks.count_ones());
        assert_eq!(
            abandoned.envelope_checks,
            u32::from(two_violations().bits())
        );

        let abandoned = Raise::of_abort(
            &MoveAbort::StepTooLarge {
                joint: JointRef::BodyYaw,
                delta: 0.9,
            },
            AT,
        );
        assert_eq!(abandoned.kind, FaultKind::MoveAbortedStep);
        assert_eq!(abandoned.joint, JointRef::BodyYaw);
        assert_eq!(abandoned.detail, 0.9);
        assert_eq!(abandoned.count, 0);
        assert_eq!(abandoned.envelope_checks, 0);
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
            joint: JointRef::AntennaRight,
            error: 0.8,
            count: 30,
        });
        out.report.newly_masked = {
            let mut set = JointFlags::NONE;
            flags::insert(&mut set, JointRef::AntennaRight);
            set
        };
        out.report.aborted = Some(MoveAbort::StepTooLarge {
            joint: JointRef::Leg0,
            delta: 0.4,
        });
        out.report.command = CommandDisposition::Rejected(CommandRejection::Trajectory(
            TrajectoryError::NonPositiveDuration,
        ));

        let mut reports = Reports::default();
        reports.collect(&out, None, AT);
        assert_eq!(reports.raised, 4, "everything the tick said was counted");
        assert_eq!(reports.dropped, 3, "three of them lost the slot");
        let published = reports.first.expect("something was published");
        assert_eq!(
            published.kind,
            FaultKind::PositionFeedbackLost,
            "the machine that must stop is what goes out",
        );

        // The same tick without the fault: the next rank down takes the slot.
        out.report.fault = None;
        let mut reports = Reports::default();
        reports.collect(&out, None, AT);
        assert_eq!(reports.raised, 3);
        assert_eq!(reports.dropped, 2);
        assert_eq!(
            reports.first.expect("a raise").kind,
            FaultKind::AntennaObstructed,
        );

        // And without the degrade, the abandoned move outranks the refusal.
        out.report.degraded = None;
        let mut reports = Reports::default();
        reports.collect(&out, None, AT);
        assert_eq!(reports.raised, 2);
        assert_eq!(reports.dropped, 1);
        assert_eq!(
            reports.first.expect("a raise").kind,
            FaultKind::MoveAbortedStep,
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
        reports.collect(&out, Some(standing), AT);
        assert_eq!(reports.raised, 0, "the same fault, still standing");
        assert!(reports.first.is_none());

        let mut reports = Reports::default();
        reports.collect(&out, None, AT);
        assert_eq!(reports.raised, 1, "the tick it was raised on is news");

        // A degrade whose joints were already out of service says nothing new
        // either: the mask is what changed on the raise, and it did not.
        let mut out = TickOutputs::default();
        out.report.degraded = Some(Fault::AntennaObstructed {
            joint: JointRef::AntennaLeft,
            error: 0.9,
            count: 30,
        });
        out.report.masked = flags::all();
        let mut reports = Reports::default();
        reports.collect(&out, None, AT);
        assert_eq!(reports.raised, 0, "nothing entered the mask on this tick");
    }
}

counters! {
    /// The run's totals, as the pose estimator keeps them.
    PoseCounters of PoseStateWire, PoseSignals<'_>, crossing the_pose_totals_cross_their_slot {
        /// Solves that found no pose.
        fk_failures / set_fk_failures,
        /// Samples that failed validation.
        refused_samples / set_refused_samples,
        /// Times the seed slot held numbers that were not a pose.
        refused_seeds / set_refused_seeds,
    }
}

counters! {
    /// The run's totals, as the decision tick keeps them.
    MoverCounters of MoverStateWire, MoverSignals<'_>, crossing the_mover_totals_cross_their_slot {
        /// Goals published.
        goals_published / set_goals_published,
        /// Reports raised.
        faults_raised / set_faults_raised,
        /// Samples read and ticked on.
        samples_seen / set_samples_seen,
        /// Samples that failed validation.
        refused_samples / set_refused_samples,
        /// Raises that lost the one report slot an execution has.
        reports_dropped / set_reports_dropped,
        /// Times the tick state in this cog's own slot could not be read back,
        /// or would not go back in.
        refused_state / set_refused_state,
        /// Times an epoch this cog had not answered yet was answered by a
        /// base step. One execution sees only the latest schedule, so bumps
        /// coalesced by a gap count once: this counts epoch changes observed,
        /// not epochs the session published.
        epochs_answered / set_epochs_answered,
        /// Overlay windows this cog would not play: the screen's refusals,
        /// counted once per schedule, plus each composed setpoint an overlay
        /// rode that the tick refused.
        overlays_refused / set_overlays_refused,
        /// Overlay rows in this cog's own slot that would not read back as a
        /// player of the motion their window names.
        players_refused / set_players_refused,
        /// Base plans this cog could not make or could not read back.
        refused_base / set_refused_base,
        /// Base plans the library adjusted for a reason that is not routine: a
        /// clock that could not carry its own span, or a pair it could not part.
        base_stretched / set_base_stretched,
        /// Base plans adjusted only to part the antenna pair at their crossing.
        base_dephased / set_base_dephased,
    }
}
