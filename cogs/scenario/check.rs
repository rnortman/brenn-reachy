//! What every scenario of this system asserts about a run, once.
//!
//! The scenarios ask the same questions of the output log -- is the
//! driver's heartbeat whole, is the goal stream one datagram per sample of an
//! engaged machine, did the head reach the posture it was sent to -- and each
//! one differs from the others only in what it does to the machine while those
//! hold. The questions live here so that a scenario's own source is the part of
//! it that is about that scenario.
//!
//! Nothing here decides what a run should look like. Each function states one
//! property, takes the numbers that vary from the scenario that called it, and
//! appends a line per way the log fails it. A checker collects the lines and
//! reports all of them: a scenario that stopped at the first surprise costs a
//! whole build per finding.

use std::path::PathBuf;
use std::process::ExitCode;

use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
use brenn_reachy__driver__health_clk_rs::EventKind;
use brenn_reachy__driver__pose_clk_rs::{PoseEstimateWire, PoseSampleWire};
use brenn_reachy__motion__joints_clk_rs::JointRefWire;
use log_read::Logged;
use motion_slots::joint_set;
use nalgebra::Isometry3;
use reachy_kin::wrap_to_pi;
use reachy_motion::joints::{
    JointGroup, JointRef, JointTargets, ROW_COUNT, flags, group_of, joint_ref, row, rows_of,
};
use reachy_motion::record;

use crate::read::Run;
use crate::{
    COGS, CONTROL_DELAY_NS, EXECUTION_DURATION_NS, FIRST_CYCLE, LAG_K, PERIOD_NS, REPORT_GROUP,
    REPORT_GROUP_PREFIX, SLEW_ANTENNAS_RAD, SLEW_BODY_YAW_RAD, SLEW_LEGS_RAD, cycle_at, cycle_of,
};

/// How far the plant may be from the posture it was sent to, in metres and in
/// the length of the rotation's error, once it has had the whole step to get
/// there. Loose enough that it is not asserting the solver's last digit and
/// tight enough that a machine which stopped half way fails.
pub const ARRIVAL_TOLERANCE: f64 = 1e-3;

/// How much room a per-cycle step gets over the configured slew before it counts
/// as a jump. The motion library's own step bound and the plant's slew are the
/// same numbers, so a well-formed goal stream sits exactly on this line; the
/// slack is for the last fractional cycle of a move, not for a policy.
pub const STEP_SLACK: f64 = 1e-9;

/// The two antennas, named rather than numbered, each with the slot it occupies
/// in a command's antenna pair: which bus rows they sit on and which way round
/// the pair reads are both the motion library's statement, and a checker holding
/// its own copy would go on asserting about rows 7 and 8 whatever moved there.
pub const ANTENNAS: [(JointRef, usize); 2] =
    [(JointRef::AntennaRight, 0), (JointRef::AntennaLeft, 1)];

/// The driver's heartbeat: one sample per cycle, on the grid, without a gap,
/// from the cycle the driver first runs on through the end of the scenario. The
/// cycle the first sample sits on, for a caller that wants to say something
/// about the rest relative to it.
///
/// This is the assertion every other one rests on. The sample stream is the
/// clock the control-rate cogs run on, so a hole in it is a control cycle that
/// never happened, and a checker that assumed the stream was whole would report
/// a missing goal instead of a missing sample.
///
/// Both ends are named exactly. The run is deterministic, so a bracket at either
/// end would be a tolerance for a regression rather than for a machine: a first
/// cycle read off the run would shift every expectation derived from it, and a
/// last cycle short by one is the final control cycle silently going missing.
///
/// What a sample *says* is not checked here: a scenario that takes the reads
/// away is still entitled to its heartbeat, and that is the difference between
/// a driver that lost the bus and one that stopped.
pub fn heartbeat(run: &Run, end_cycle: i64, failures: &mut Vec<String>) -> Option<i64> {
    if run.samples.is_empty() {
        failures.push("the driver published no samples at all".to_owned());
        return None;
    }
    let first = match cycle_of(run.samples[0].message.nominal_time().as_nanos()) {
        Ok(cycle) => cycle,
        Err(complaint) => {
            failures.push(format!("the first sample is not on the grid: {complaint}"));
            return None;
        }
    };
    if first != FIRST_CYCLE {
        failures.push(format!(
            "the driver's first sample is for cycle {first}, and the driver's first execution is \
             cycle {FIRST_CYCLE}"
        ));
    }
    for (index, sample) in run.samples.iter().enumerate() {
        let nominal = sample.message.nominal_time().as_nanos();
        let expected = cycle_at(first + index as i64);
        if nominal != expected {
            failures.push(format!(
                "sample {index} is nominal at {nominal}, expected {expected}: the heartbeat has a \
                 gap or is off the grid"
            ));
            break;
        }
        if sample.at_ns != nominal + EXECUTION_DURATION_NS {
            failures.push(format!(
                "the sample nominal at {nominal} was logged at {}, expected {}",
                sample.at_ns,
                nominal + EXECUTION_DURATION_NS
            ));
            break;
        }
    }
    let last = first + run.samples.len() as i64 - 1;
    if last != end_cycle {
        failures.push(format!(
            "the heartbeat stops at cycle {last}, and the run goes to {end_cycle}"
        ));
    }
    Some(first)
}

/// Every sample carries a complete reading: the bus answered for every row.
///
/// For a scenario that takes nothing away. One that does asserts its outage
/// itself, cycle by cycle, because where the outage begins and ends is the
/// subject rather than a background condition.
pub fn readings_present(run: &Run, failures: &mut Vec<String>) {
    for sample in &run.samples {
        let sample = &sample.message;
        let complete = joint_set(sample.missing()).is_ok_and(flags::is_empty);
        if !sample.present_valid() || !complete {
            failures.push(format!(
                "the sample nominal at {} carries no reading, and this scenario takes none away",
                sample.nominal_time().as_nanos()
            ));
            break;
        }
    }
}

/// What the goal stream turned out to be, for a scenario to say when it should
/// have started and stopped.
pub struct GoalStream {
    /// The cycle the first goal was decided on.
    pub first_cycle: i64,
    /// The cycle the last one was decided on.
    pub last_cycle: i64,
}

/// The goal stream: one datagram per sample of an engaged, armed machine, each
/// dated `lag_k` cycles ahead of the sample that decided it, each speaking for
/// every row, each within one cycle's travel of the one before.
///
/// The keep-alive is what makes "one per sample" the assertion rather than "one
/// per change": a holding machine re-publishes the setpoint it is already on,
/// because silence is what the driver's dead-man measures. So a break anywhere
/// in the stream is reported here, and *where* the stream begins and ends is
/// the calling scenario's business.
pub fn goal_stream(run: &Run, failures: &mut Vec<String>) -> Option<GoalStream> {
    if run.goals.is_empty() {
        failures.push("the decision tick published no goals at all".to_owned());
        return None;
    }
    let first = match cycle_of(run.goals[0].at_ns - CONTROL_DELAY_NS) {
        Ok(cycle) => cycle,
        Err(complaint) => {
            failures.push(format!("the first goal is not on the grid: {complaint}"));
            return None;
        }
    };
    let mut due_at = PerGoal::new("goals due at the wrong instant");
    let mut speaks_for = PerGoal::new("goals speaking for the wrong rows");
    let mut ordered = PerGoal::new("goals out of order");
    let mut travel = PerGoal::new("rows moved further than the plant can travel");
    let mut previous: Option<(i64, [f64; 9])> = None;
    for (index, goal) in run.goals.iter().enumerate() {
        let cycle = first + index as i64;
        let nominal = cycle_at(cycle);
        if goal.at_ns != nominal + CONTROL_DELAY_NS {
            failures.push(format!(
                "goal {index} was logged at {}, expected {}: the stream is not one per sample",
                goal.at_ns,
                nominal + CONTROL_DELAY_NS
            ));
            break;
        }
        let setpoint = &goal.message;
        let execute_at_ns = setpoint.execute_at().as_nanos();
        let targets = targets_of(setpoint);
        let due = nominal + LAG_K * PERIOD_NS;
        if execute_at_ns != due {
            due_at.push(
                format!("the goal decided at {nominal} is due at {execute_at_ns}, expected {due}"),
                failures,
            );
        }
        match joint_set(setpoint.mask()) {
            Ok(mask) if mask == flags::all() => {}
            Ok(mask) => speaks_for.push(
                format!(
                    "the goal decided at {nominal} speaks for {}, and no servo went out of \
                     service in this scenario",
                    flags::Names(mask)
                ),
                failures,
            ),
            Err(complaint) => speaks_for.push(
                format!("the goal decided at {nominal} names no set of servos: {complaint}"),
                failures,
            ),
        }
        if let Some((was_due, was)) = previous {
            if execute_at_ns <= was_due {
                ordered.push(
                    format!(
                        "the goal decided at {nominal} is due at {execute_at_ns}, not after the \
                         one before it at {was_due}"
                    ),
                    failures,
                );
            }
            for (row, before) in was.iter().enumerate() {
                let step = (targets[row] - before).abs();
                let Some(cap) = slew_of(row) else {
                    travel.push(
                        format!(
                            "the goal decided at {nominal} speaks for row {row}, which sits on no \
                             bus row of this machine"
                        ),
                        failures,
                    );
                    continue;
                };
                if step > cap + STEP_SLACK {
                    travel.push(
                        format!(
                            "the goal decided at {nominal} moves row {row} by {step} rad in one \
                             cycle, past the {cap} rad the plant can travel"
                        ),
                        failures,
                    );
                }
            }
        }
        previous = Some((execute_at_ns, targets));
    }
    for property in [due_at, speaks_for, ordered, travel] {
        property.summarise(failures);
    }
    Some(GoalStream {
        first_cycle: first,
        last_cycle: first + run.goals.len() as i64 - 1,
    })
}

/// The angles a setpoint asks for, in bus-row order.
///
/// Read through the same mapping the cogs write with, so a checker cannot put a
/// servo's angle under another servo's row while asserting that the run did
/// not. Nine scalars carry nothing a validation can refuse, so the fall-back
/// stands only where the generated route insists on an answer.
fn targets_of(setpoint: &GoalSetpointWire) -> [f64; ROW_COUNT] {
    setpoint
        .targets()
        .validate()
        .map(rows_of)
        .unwrap_or_default()
}

/// One property of the goal stream, asserted per goal and reported without
/// burying everything else.
///
/// A run is two or three hundred cycles and a systematic regression -- the wrong
/// lag in the build, a mask that lost a row -- breaks the same property on every
/// one of them. Collecting rather than throwing exists so that one expensive
/// round trip reports everything wrong with it, and a report of three hundred
/// copies of one line defeats that as surely as stopping at the first would. So
/// each property reports its first failure in full and then says how many more
/// goals went the same way.
struct PerGoal {
    /// What the property is, for the line that counts the rest.
    what: &'static str,
    /// How many goals failed it.
    failed: u64,
}

impl PerGoal {
    /// A property nothing has failed yet.
    const fn new(what: &'static str) -> Self {
        Self { what, failed: 0 }
    }

    /// Record one goal failing it, reporting the first in full.
    fn push(&mut self, failure: String, failures: &mut Vec<String>) {
        self.failed += 1;
        if self.failed == 1 {
            failures.push(failure);
        }
    }

    /// Say how many went the same way, if more than the one already reported
    /// did.
    fn summarise(self, failures: &mut Vec<String>) {
        if self.failed > 1 {
            failures.push(format!(
                "and {} more {}, past the one reported above",
                self.failed - 1,
                self.what
            ));
        }
    }
}

/// How far the modelled servo on `row` travels in one cycle, radians, or `None`
/// for a row no joint of this machine sits on.
///
/// Off the joint's own group rather than off the row number, so a machine whose
/// rows moved keeps its antennas' figure with its antennas. Every group is
/// named: a row of no group would otherwise be given some group's number, and a
/// bound taken from the wrong group permits a jump the plant cannot make.
#[must_use]
pub fn slew_of(row: usize) -> Option<f64> {
    Some(match group_of(joint_ref(row)?)? {
        JointGroup::Antennas => SLEW_ANTENNAS_RAD,
        JointGroup::BodyYaw => SLEW_BODY_YAW_RAD,
        JointGroup::Legs => SLEW_LEGS_RAD,
    })
}

/// The goal stream covers exactly the session: it starts within a couple of
/// cycles of the engagement and stops with the disengagement.
pub fn stream_covers_session(
    stream: &GoalStream,
    engage_cycle: i64,
    disengage_cycle: i64,
    failures: &mut Vec<String>,
) {
    stream_starts_with_session(stream, engage_cycle, failures);
    stream_stops_with_session(stream, disengage_cycle, failures);
}

/// The goal stream starts with the session: not before the engagement, and
/// within a couple of cycles of it.
///
/// The tick arms off the first sample it sees while engaged, and arming from a
/// posture the machine is standing in is a solve that does not fail, so the
/// stream starts on that sample or the one after it -- never later. Never
/// earlier either, and that half is the stronger claim: a goal published before
/// the session engaged is the decision tick commanding a machine nobody asked
/// it to, which is the failure the whole gate exists to make impossible.
pub fn stream_starts_with_session(
    stream: &GoalStream,
    engage_cycle: i64,
    failures: &mut Vec<String>,
) {
    if stream.first_cycle > engage_cycle + 2 {
        failures.push(format!(
            "the goal stream starts at cycle {}; the session engaged at {engage_cycle} and arming \
             from a standing posture is not a solve that fails",
            stream.first_cycle
        ));
    }
    if stream.first_cycle < engage_cycle {
        failures.push(format!(
            "the goal stream starts at cycle {}, before the session engaged at {engage_cycle}: \
             those goals commanded a machine no session had asked for",
            stream.first_cycle
        ));
    }
}

/// The goal stream stops with the session: at the disengagement, neither past
/// it nor well before it.
///
/// It stops at the disengagement because a holding machine still publishes, so
/// an early stop is a stream that broke rather than a machine that arrived.
pub fn stream_stops_with_session(
    stream: &GoalStream,
    disengage_cycle: i64,
    failures: &mut Vec<String>,
) {
    if stream.last_cycle >= disengage_cycle {
        failures.push(format!(
            "the goal stream runs to cycle {}; the session disengaged at {disengage_cycle}",
            stream.last_cycle
        ));
    }
    if stream.last_cycle < disengage_cycle - 2 {
        failures.push(format!(
            "the goal stream stops at cycle {}, well before the session disengaged at \
             {disengage_cycle}: a holding machine still publishes",
            stream.last_cycle
        ));
    }
}

/// The pose series is one estimate per sample, each stamped with the instant of
/// the reading it is about.
///
/// Joined to the sample stream rather than checked for shape, because what a
/// consumer downstream does with an estimate is decide how old it is: a series
/// that was whole, ordered and on the grid but stamped a cycle out is exactly
/// the staleness the estimator's contract exists to rule out, and every shape
/// assertion in the world passes over it. Pinning each stamp to its sample
/// subsumes the grid and the ordering both.
pub fn estimates_per_sample(run: &Run, failures: &mut Vec<String>) {
    if run.estimates.len() != run.samples.len() {
        failures.push(format!(
            "{} estimates for {} samples: the pose series is not one per reading",
            run.estimates.len(),
            run.samples.len()
        ));
    }
    for (index, (estimate, sample)) in run.estimates.iter().zip(&run.samples).enumerate() {
        let validity = estimate.message.time_of_validity().as_nanos();
        let reading = sample.message.sample_time().as_nanos();
        if validity != reading {
            failures.push(format!(
                "estimate {index} is valid at {validity}, and the reading it is the {index}th of \
                 was taken at {reading}"
            ));
            break;
        }
    }
}

/// Every estimate found a pose. For a scenario that gives the solver nothing to
/// fail on.
pub fn estimates_valid(run: &Run, failures: &mut Vec<String>) {
    for (index, estimate) in run.estimates.iter().enumerate() {
        if !estimate.message.valid() {
            failures.push(format!(
                "estimate {index}, for the reading at {}, found no pose, and this scenario gives \
                 the solver nothing to fail on",
                estimate.message.time_of_validity().as_nanos()
            ));
            break;
        }
    }
}

/// The machine is in the posture `wanted` names at `cycle`: the head where the
/// estimate puts it, and both antennas where the sample reads them.
///
/// The head is asserted through the estimator's own output rather than against
/// the plant's joint angles, because where the head is is what a posture means
/// -- and it puts the estimator in the assertion rather than beside it. The
/// antennas are not in the head pose, so they are read off the sample and
/// compared against the angles the posture itself names: comparing them against
/// the goal instead would only say the servo tracked whatever it was told, which
/// a loop that commanded the wrong antenna angle for every posture -- or never
/// moved them at all -- satisfies perfectly.
///
/// The antennas are compared as directions rather than as angles. They run in
/// extended position mode with no travel limit, and a posture names where an
/// antenna points, not how many turns of thread it took to get there: the motion
/// library takes the long way round whenever the short arc would sweep an
/// antenna out sideways, so a machine that arrived correctly can be a whole turn
/// from the number the posture states.
pub fn arrived_at(
    run: &Run,
    what: &str,
    cycle: i64,
    wanted: &JointTargets,
    failures: &mut Vec<String>,
) {
    let at = cycle_at(cycle);
    let Some(estimate) = estimate_at(run, cycle) else {
        failures.push(format!(
            "no estimate for cycle {cycle}, where the machine should be {what}"
        ));
        return;
    };
    let Some(found) = solved_pose(&estimate.message) else {
        failures.push(format!(
            "the estimate at {at} holds numbers that are not a pose, where the machine should be \
             {what}"
        ));
        return;
    };
    let offset = (found.translation.vector - wanted.head_pose_body.translation.vector).norm();
    let turn = found.rotation.angle_to(&wanted.head_pose_body.rotation);
    if offset > ARRIVAL_TOLERANCE || turn > ARRIVAL_TOLERANCE {
        failures.push(format!(
            "at cycle {cycle} the head is {offset} m and {turn} rad from {what}"
        ));
    }
    let Some(sample) = sample_at(run, cycle) else {
        failures.push(format!("no sample for cycle {cycle}"));
        return;
    };
    for (antenna, slot) in ANTENNAS {
        let Some(row) = row(antenna) else {
            failures.push(format!("{antenna:?} sits on no bus row"));
            continue;
        };
        let present = present_rows(sample);
        let error = wrap_to_pi(present[row] - wanted.antennas[slot]).abs();
        if error > ARRIVAL_TOLERANCE {
            failures.push(format!(
                "at cycle {cycle} {antenna:?} points {error} rad away from where {what} puts it: \
                 it reads {} rad and the posture names {} rad",
                present[row], wanted.antennas[slot]
            ));
        }
    }
}

/// One schedule the session is expected to have published, as a scenario states
/// it.
pub struct Session {
    /// The cycle it was published on.
    pub cycle: i64,
    /// Whether it engages the machine.
    pub engaged: bool,
    /// Its epoch, which is what makes a change news to the decision tick.
    pub epoch: u32,
}

/// The session's schedules reached the run: these ones, in this order, on these
/// cycles.
///
/// The input log is replayed into the system by the runner, and a scenario that
/// only looked at what the cogs did could not tell a schedule the tick acted on
/// from one that was written to the wrong channel, dropped, or never sent -- the
/// run of a session that said nothing and the run of a session nobody wrote for
/// are identical in every other stream. This is the assertion that the scenario
/// really did say what it meant to say.
pub fn schedules_replayed(run: &Run, wanted: &[Session], failures: &mut Vec<String>) {
    if run.schedules.len() != wanted.len() {
        failures.push(format!(
            "the run replayed {} schedules, and the scenario published {}",
            run.schedules.len(),
            wanted.len()
        ));
        return;
    }
    for (index, (found, wanted)) in run.schedules.iter().zip(wanted).enumerate() {
        let at = cycle_at(wanted.cycle);
        if found.at_ns != at {
            failures.push(format!(
                "schedule {index} reached the run at {}, and the scenario published it at {at}",
                found.at_ns
            ));
        }
        if found.message.engaged() != wanted.engaged {
            failures.push(format!(
                "schedule {index} says engaged = {}, and the scenario published {}",
                found.message.engaged(),
                wanted.engaged
            ));
        }
        if found.message.epoch() != wanted.epoch {
            failures.push(format!(
                "schedule {index} carries epoch {}, and the scenario published {}",
                found.message.epoch(),
                wanted.epoch
            ));
        }
    }
}

/// The joint a report names, or `None` where it names none or names a servo this
/// build's vocabulary has not got.
#[must_use]
pub fn joint_of(joint: JointRefWire) -> Option<JointRef> {
    match joint.to_known()? {
        JointRef::None => None,
        named => Some(named),
    }
}

/// One step is long enough for the move it asks for.
///
/// The scenario's own arithmetic, checked rather than trusted: a step shorter
/// than the move it asks for would make every arrival assertion pass about a
/// machine that was still travelling.
pub fn room(what: &str, step_cycles: i64, move_cycles: i64, failures: &mut Vec<String>) {
    if step_cycles <= move_cycles {
        failures.push(format!(
            "the {what} step is {step_cycles} cycles and the move to it is given {move_cycles}: \
             the scenario does not leave the machine time to arrive"
        ));
    }
}

/// The decision tick reported nothing.
pub fn no_faults(run: &Run, failures: &mut Vec<String>) {
    for fault in &run.faults {
        failures.push(format!(
            "the decision tick reported {:?} at {}, and nothing in this scenario is wrong with the \
             machine",
            fault.message.kind(),
            fault.message.time().as_nanos()
        ));
    }
}

/// The driver's gate raised nothing.
///
/// The stronger half of a quiet run. The goal stream stops when the session
/// disengages and a scenario's tail runs past the dead-man's window, so an
/// event here would mean the machine was still energised when the commander
/// went quiet -- which is exactly the condition the dead-man exists for, and
/// exactly the thing a scenario's ordering is supposed to avoid.
pub fn no_events(run: &Run, failures: &mut Vec<String>) {
    for event in &run.events {
        failures.push(format!(
            "the driver's gate raised {} at {}, and nothing in this scenario asks it to",
            event.message.kind(),
            event.message.time().as_nanos()
        ));
    }
}

/// The nine measured angles a sample carries, in bus order.
///
/// The one reading of a sample's positions the checkers share, so a scenario
/// asks for a row by the number the plant and the cogs use.
#[must_use]
pub fn present_rows(sample: &PoseSampleWire) -> [f64; ROW_COUNT] {
    sample.present().validate().map(rows_of).unwrap_or_default()
}

/// The sample the driver published for `cycle`, if the log has one.
///
/// By the instant the sample is *about* rather than the instant it was logged
/// at, because those are two clocks and a scenario reasons in the first one.
#[must_use]
pub fn sample_at(run: &Run, cycle: i64) -> Option<&PoseSampleWire> {
    at_instant(&run.samples, cycle_at(cycle), |sample| {
        sample.message.nominal_time().as_nanos()
    })
    .map(|sample| &sample.message)
}

/// The message of a dense, ordered stream whose own instant is `at`.
///
/// The streams of this system carry one message per cycle -- which is what the
/// heartbeat and goal-stream assertions establish -- so the position is
/// arithmetic off the first message rather than a search. A stream with a hole
/// in it would put the arithmetic on the wrong message, so the answer is checked
/// and a stream that fails the check is searched instead: these accessors are
/// called from per-cycle loops, and the assertions that would report the hole
/// have to survive long enough to report it.
fn at_instant<T>(stream: &[T], at: i64, instant: impl Fn(&T) -> i64) -> Option<&T> {
    let first = instant(stream.first()?);
    if let Ok(index) = usize::try_from((at - first) / PERIOD_NS)
        && let Some(found) = stream.get(index)
        && instant(found) == at
    {
        return Some(found);
    }
    stream.iter().find(|message| instant(message) == at)
}

/// The positions the goal decided on `cycle` asked for, if the log has one.
///
/// A goal carries the instant it is due rather than the instant it was decided
/// on, so the cycle it belongs to is read off its log time -- which is the one
/// message in this system whose contents cannot name their own cycle.
#[must_use]
pub fn goal_at(run: &Run, cycle: i64) -> Option<[f64; 9]> {
    at_instant(&run.goals, cycle_at(cycle) + CONTROL_DELAY_NS, |goal| {
        goal.at_ns
    })
    .map(|goal| targets_of(&goal.message))
}

/// The estimate the pose cog published for `cycle`, if the log has one.
///
/// By the reading's own instant, which is the one an estimate carries: every
/// other way to reach a pose in this module goes through here, so there is one
/// answer to "which estimate is cycle N's" rather than one per caller.
#[must_use]
pub fn estimate_at(run: &Run, cycle: i64) -> Option<&Logged<PoseEstimateWire>> {
    at_instant(&run.estimates, cycle_at(cycle), |estimate| {
        estimate.message.time_of_validity().as_nanos()
    })
}

/// Where the estimator put the head on `cycle`, if it found it at all.
#[must_use]
pub fn head_pose_at(run: &Run, cycle: i64) -> Option<Isometry3<f64>> {
    let estimate = estimate_at(run, cycle)?;
    if !bool::from(estimate.message.validate().ok()?.valid) {
        return None;
    }
    solved_pose(&estimate.message)
}

/// The pose an estimate message describes, whatever its `valid` flag says.
///
/// `None` for bytes that do not read as an estimate and for a rotation that is
/// no rotation -- one question, because to a checker they are the same answer:
/// this message names no pose.
fn solved_pose(estimate: &PoseEstimateWire) -> Option<Isometry3<f64>> {
    let estimate = estimate.validate().ok()?;
    record::read_pose(&estimate.head_pos, &estimate.head_quat).ok()
}

/// The driver's gate raised exactly one event, of `kind`, on the cycle the
/// scenario's arithmetic names, carrying the silence it should carry.
///
/// Both numbers are named exactly rather than bracketed, because both are
/// arithmetic: a deterministic run does not drift, so a bracket would be a
/// tolerance for a mistake rather than for a machine.
///
/// `silence_ns` is the evidence field the two kinds this helper is asked about
/// carry. A kind whose evidence is a count or a set of servos is not one any
/// scenario asserts today, and would want its own reading rather than this
/// one's.
///
/// The cycle it fired on, for a caller that wants to say what the machine did
/// afterwards.
pub fn sole_event(
    run: &Run,
    kind: EventKind,
    at_cycle: i64,
    silence_ns: i64,
    failures: &mut Vec<String>,
) -> Option<i64> {
    let mut fired = None;
    for event in &run.events {
        let event = &event.message;
        let cycle = match cycle_of(event.time().as_nanos()) {
            Ok(cycle) => cycle,
            Err(complaint) => {
                failures.push(format!("an event is not on the grid: {complaint}"));
                continue;
            }
        };
        let raised = match event.kind().to_known() {
            Some(raised) => raised,
            None => {
                failures.push(format!(
                    "an event at cycle {cycle} names a kind this build does not know"
                ));
                continue;
            }
        };
        if raised != kind {
            failures.push(format!(
                "the driver's gate raised {raised} at cycle {cycle}, and the only event this \
                 scenario asks it for is {kind}"
            ));
            continue;
        }
        if fired.is_some() {
            failures.push(format!(
                "the gate raised {kind} again at cycle {cycle}: a latch is a transition, and the \
                 standing condition is not news"
            ));
            continue;
        }
        if cycle != at_cycle {
            failures.push(format!(
                "the gate raised {kind} at cycle {cycle}, and the scenario's arithmetic puts it \
                 at cycle {at_cycle}"
            ));
        }
        if event.silence().as_nanos() != silence_ns {
            failures.push(format!(
                "the {kind} at cycle {cycle} carries a silence of {}ns, and the scenario's \
                 arithmetic makes it {silence_ns}ns",
                event.silence().as_nanos()
            ));
        }
        fired = Some(cycle);
    }
    if fired.is_none() {
        failures.push(format!(
            "the gate never raised {kind}, and this scenario is about the cycle it does"
        ));
    }
    fired
}

/// The torque-off latch is on the wire: every sample from `fired` onwards says
/// the driver has torque off and is refusing to write, and none before it does.
///
/// The event says it happened; the sample flag is what a receiver that missed
/// the event still sees, and the two disagreeing would be a driver whose report
/// and whose state are two different stories.
pub fn latch_from(run: &Run, fired: Option<i64>, failures: &mut Vec<String>) {
    let Some(fired) = fired else {
        return;
    };
    for sample in &run.samples {
        let sample = &sample.message;
        let Ok(cycle) = cycle_of(sample.nominal_time().as_nanos()) else {
            continue;
        };
        let wanted = cycle >= fired;
        if sample.torque_off_latched() != wanted {
            failures.push(format!(
                "the sample at cycle {cycle} says the torque-off latch is {}, and the gate fired \
                 at cycle {fired}",
                sample.torque_off_latched()
            ));
            return;
        }
    }
}

/// Every cog's signal report group reached the log as a channel of its own.
///
/// By the name the framework composes for each cog's group, rather than by
/// counting channels that mention one: a count is satisfied by any three
/// channels, including three belonging to two cogs, which is exactly the
/// wiring mistake this is here to catch.
///
/// Their contents are not read -- a report group's schema is generated for the
/// group, and nothing binds a Rust type to one here -- and at this drop the
/// three channels carry no messages at all, so nothing about any total is
/// asserted anywhere in this repo: not the values, not which total reaches
/// which signal, not that a signal is ever written.
/// TODO(cogs-signal-report-contents)
pub fn signal_groups(run: &Run, failures: &mut Vec<String>) {
    for cog in COGS {
        let wanted = format!("{REPORT_GROUP_PREFIX}{cog}/{REPORT_GROUP}/");
        if !run
            .channel_names
            .iter()
            .any(|name| name.starts_with(&wanted))
        {
            failures.push(format!(
                "the log carries no channel named {wanted}*, so {cog}'s {REPORT_GROUP} group did \
                 not reach it; the channels it does carry are {:?}",
                run.channel_names
            ));
        }
    }
}

/// The machine stands still: every sample from `from_cycle` through
/// `through_cycle` reads what the sample at `from_cycle` read.
///
/// `why` says what held it there, and it is the whole content of the assertion:
/// a de-torqued servo on this machine holds where it stands, an uncommanded one
/// keeps its last setpoint, and a machine that moved anyway moved without
/// anything asking it to.
///
/// The samples the log holds are what is scanned, rather than a lookup per
/// cycle: a hole in the stream is the heartbeat's complaint to make, and making
/// it here as well would report one fault as two.
pub fn stands_still(
    run: &Run,
    from_cycle: i64,
    through_cycle: i64,
    why: &str,
    failures: &mut Vec<String>,
) {
    let Some(stood) = sample_at(run, from_cycle).map(present_rows) else {
        failures.push(format!(
            "no sample for cycle {from_cycle}, where the machine was {why}"
        ));
        return;
    };
    for sample in &run.samples {
        let sample = &sample.message;
        let Ok(cycle) = cycle_of(sample.nominal_time().as_nanos()) else {
            continue;
        };
        if cycle < from_cycle || cycle > through_cycle {
            continue;
        }
        if present_rows(sample) != stood {
            failures.push(format!(
                "the machine moved by cycle {cycle}, and from cycle {from_cycle} it was {why}"
            ));
            return;
        }
    }
}

/// Run one scenario's checker: read the log the harness produced, put the
/// scenario's assertions to it, and report every way it failed them.
///
/// The three arguments and the one-failure-per-line report are the harness's
/// protocol rather than any one scenario's, so they are stated here: a checker's
/// own source is then the list of assertions, which is what a reader of it came
/// for.
pub fn main(name: &str, assert: impl FnOnce(&Run, &mut Vec<String>)) -> ExitCode {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let [log_dir, mover_params, sim_params] = args.as_slice() else {
        eprintln!("usage: {name} <output-log-dir> <mover-params> <sim-params>");
        return ExitCode::FAILURE;
    };

    let mut failures = crate::check_params(mover_params, sim_params);
    let run = match Run::read(&PathBuf::from(log_dir)) {
        Ok(run) => run,
        Err(err) => {
            eprintln!("reading the output log under {log_dir}: {err}");
            return ExitCode::FAILURE;
        }
    };
    failures.extend(run.complaints.iter().cloned());

    assert(&run, &mut failures);

    if failures.is_empty() {
        return ExitCode::SUCCESS;
    }
    for failure in &failures {
        eprintln!("{name}: {failure}");
    }
    ExitCode::FAILURE
}
