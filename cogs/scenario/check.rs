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

use std::collections::BTreeMap;
use std::ops::Range;
use std::path::PathBuf;
use std::process::ExitCode;

use brenn_reachy__cogs__schedule_clk_rs::{PostureWire, StepKindWire};
use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__cogs__session_cmd_clk_rs::SessionCmdKindWire;
use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
use brenn_reachy__driver__health_clk_rs::EventKind;
use brenn_reachy__driver__pose_clk_rs::{PoseEstimateWire, PoseSampleWire};
use brenn_reachy__hardware__dynamixel__registers_clk_rs::RegIdWire;
use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKindWire;
use brenn_reachy__motion__faults_clk_rs::FaultKindWire;
use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointRefWire};
use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKindWire};
use log_read::Logged;
use motion_slots::joint_set;
use nalgebra::Isometry3;
use reachy_kin::wrap_to_pi;
use reachy_motion::joints::{
    JointGroup, JointRef, JointTargets, Name, ROW_COUNT, flags, group_of, joint_ref, row, rows_of,
};
use reachy_motion::record;

use crate::read::Run;
use crate::{
    BUS_WATCHDOG, COGS, CONTROL_DELAY_NS, DRIVER_CONFIRM_BUDGET_NS, EXECUTION_DURATION_NS,
    FIRST_CYCLE, LAG_K, MoveClocks, PERIOD_NS, PROFILE_ACCELERATION, PROFILE_VELOCITY,
    REPORT_GROUP, REPORT_GROUP_PREFIX, SESSION_WAKE_FLOOR_NS, SLEW_ANTENNAS_RAD, SLEW_BODY_YAW_RAD,
    SLEW_LEGS_RAD, commission_transactions, cycle_at, cycle_of, cycle_within, cycles_for,
    drain_cycle,
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

/// The goal stream of a machine every row of which stays in service.
///
/// The ordinary case, and the strongest form of the assertion: every goal speaks
/// for all nine rows.
pub fn goal_stream(run: &Run, failures: &mut Vec<String>) -> Option<GoalStream> {
    goal_stream_without(run, JointFlags::NONE, failures)
}

/// The goal stream: one datagram per sample of an engaged, armed machine, each
/// dated `lag_k` cycles ahead of the sample that decided it, each speaking for
/// every row still in service, each within one cycle's travel of the one before.
///
/// The keep-alive is what makes "one per sample" the assertion rather than "one
/// per change": a holding machine re-publishes the setpoint it is already on,
/// because silence is what the driver's dead-man measures. So a break anywhere
/// in the stream is reported here, and *where* the stream begins and ends is
/// the calling scenario's business.
///
/// `may_leave` names the rows the scenario expects the decision tick to take out
/// of service -- a masked joint is commanded nothing, which is the whole of what
/// masking does to this stream. Two properties are asserted about it rather than
/// tolerated: nothing outside that set ever stops being spoken for, and a row
/// that has left never comes back. Coming back would mean the tick had started
/// commanding a servo it had already declared it could not move, and no rung of
/// the ladder ever does that -- a masked joint is masked for the session.
pub fn goal_stream_without(
    run: &Run,
    may_leave: JointFlags,
    failures: &mut Vec<String>,
) -> Option<GoalStream> {
    goal_streams_exactly(run, may_leave, 1, failures)
        .into_iter()
        .next()
}

/// Every stretch of goal stream the run carried, in order, each asserted as
/// [`goal_stream_without`] asserts one -- and there are `engagements` of them.
///
/// The one entry point, because the count is the only thing that differs between
/// a run that engages the machine once and a run that engages it twice: a
/// scenario says the number and the wording of a run that carried another number
/// is one wording, here, rather than one per scenario.
///
/// A run engages the machine more than once when a session that ended is
/// followed by another script, and the stream stops in between: nothing is
/// commanded while the next engagement is being made, which is what the driver's
/// dead-man is held off by the keep-alive rather than by a goal. So the breaks
/// are what the count says something about, and everything within a stretch is
/// asserted per stretch.
///
/// `may_leave` is per stretch: a fresh engagement is a fresh mask, so a row the
/// tick took out of service in one session is spoken for again in the next
/// without that being a servo coming back from the dead.
pub fn goal_streams_exactly(
    run: &Run,
    may_leave: JointFlags,
    engagements: usize,
    failures: &mut Vec<String>,
) -> Vec<GoalStream> {
    if run.goals.is_empty() {
        failures.push("the decision tick published no goals at all".to_owned());
        return Vec::new();
    }
    let mut streams = Vec::new();
    let mut start = 0;
    for index in 1..=run.goals.len() {
        let broken = index == run.goals.len()
            || run.goals[index].at_ns != run.goals[index - 1].at_ns + PERIOD_NS;
        if !broken {
            continue;
        }
        if let Some(stream) = one_stream(
            &run.goals[start..index],
            streams.len() + 1,
            may_leave,
            failures,
        ) {
            streams.push(stream);
        }
        start = index;
    }
    if streams.len() != engagements {
        failures.push(format!(
            "the run carried {} stretches of goal stream, and this scenario engages the machine \
             {engagements} time(s): the stream stops between one session and the next arming \
             taking hold, so there is one stretch per engagement",
            streams.len()
        ));
        for pair in streams.windows(2) {
            failures.push(format!(
                "the goal stream stopped at cycle {} and started again at {}",
                pair[0].last_cycle, pair[1].first_cycle
            ));
        }
    }
    streams
}

/// One stretch of stream, every goal of which follows the one before it by a
/// period: the whole of [`goal_stream_without`]'s per-goal assertions.
///
/// `stretch` is which engagement this is, counted from one. Carried into every
/// complaint the stretch makes: a run that engages the machine twice breaks the
/// same property in both halves the same way, and a failure that did not say
/// which half is a failure someone has to find twice.
fn one_stream(
    goals: &[Logged<GoalSetpointWire>],
    stretch: usize,
    may_leave: JointFlags,
    failures: &mut Vec<String>,
) -> Option<GoalStream> {
    let first = match cycle_of(goals[0].at_ns - CONTROL_DELAY_NS) {
        Ok(cycle) => cycle,
        Err(complaint) => {
            failures.push(format!(
                "the first goal of stretch {stretch}, logged at {}, is not on the grid: \
                 {complaint}",
                goals[0].at_ns
            ));
            return None;
        }
    };
    let mut due_at = PerGoal::new("goals due at the wrong instant", stretch, first);
    let mut speaks_for = PerGoal::new("goals speaking for the wrong rows", stretch, first);
    let mut ordered = PerGoal::new("goals out of order", stretch, first);
    let mut travel = PerGoal::new(
        "rows moved further than the plant can travel",
        stretch,
        first,
    );
    let mut previous: Option<(i64, [f64; 9])> = None;
    // The rows that have gone out of service so far, which is what makes a row
    // coming back visible.
    let mut left = JointFlags::NONE;
    for (index, goal) in goals.iter().enumerate() {
        // A stretch is a run of goals a period apart -- that is what split it
        // -- and its first goal was read off the grid, so every goal in it sits
        // where this cycle puts it by arithmetic. A stream that skipped a sample
        // is a second stretch, and the count is where that is reported.
        let cycle = first + index as i64;
        let nominal = cycle_at(cycle);
        debug_assert_eq!(goal.at_ns, nominal + CONTROL_DELAY_NS);
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
            Ok(mask) => {
                let silent = flags::without(flags::all(), mask);
                if !flags::is_empty(flags::without(silent, may_leave)) {
                    let allowed = if flags::is_empty(may_leave) {
                        "no servo goes out of service in this scenario".to_owned()
                    } else {
                        format!(
                            "the only servos this scenario takes out of service are {}",
                            flags::Names(may_leave)
                        )
                    };
                    speaks_for.push(
                        format!(
                            "the goal decided at {nominal} speaks for {}, and {allowed}",
                            flags::Names(mask)
                        ),
                        failures,
                    );
                }
                for joint in flags::iter(left) {
                    if flags::contains(mask, joint) {
                        speaks_for.push(
                            format!(
                                "the goal decided at {nominal} speaks for {}, which had already \
                                 gone out of service: a masked joint is masked for the session",
                                Name(joint)
                            ),
                            failures,
                        );
                    }
                }
                left |= silent;
            }
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
        last_cycle: first + goals.len() as i64 - 1,
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
    /// Which stretch of stream this is counting over, from one, and the cycle it
    /// begins on: a run with two engagements in it fails the same property twice
    /// and the two counts are about different halves of the run.
    stretch: (usize, i64),
    /// How many goals failed it.
    failed: u64,
}

impl PerGoal {
    /// A property nothing has failed yet, over the stretch `stretch` beginning
    /// on cycle `from`.
    const fn new(what: &'static str, stretch: usize, from: i64) -> Self {
        Self {
            what,
            stretch: (stretch, from),
            failed: 0,
        }
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
                "and {} more {} in stretch {} of the stream, which begins on cycle {}, past the \
                 one reported above",
                self.failed - 1,
                self.what,
                self.stretch.0,
                self.stretch.1
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

/// The goal stream stops with the release: on the cycle the session let go, or
/// within a cycle either side of it.
///
/// A cycle either side and no more, because the session runs when a message
/// reaches it rather than on the bus grid: the schedule saying the machine is no
/// longer under command lands somewhere inside a cycle, and whether the streaming
/// cog's execution for that cycle came before or after it is what the one cycle
/// of slack is for. Anything wider would be a stream that broke rather than a
/// session that ended, because a holding machine still publishes.
pub fn stream_stops_with_release(stream: &GoalStream, released: i64, failures: &mut Vec<String>) {
    if stream.last_cycle > released + 1 {
        failures.push(format!(
            "the goal stream runs to cycle {}; the session let go of the machine at {released}",
            stream.last_cycle
        ));
    }
    if stream.last_cycle < released - 1 {
        failures.push(format!(
            "the goal stream stops at cycle {}, before the session let go at {released}: a \
             holding machine still publishes",
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
    // The run's own last reading is allowed to go unanswered, and only that
    // one. A sample is published inside the cycle it was taken on and the cogs
    // that read it execute after that, so the estimate for the last sample of a
    // run falls past the instant the run stops at. What the pairing below
    // asserts is unaffected: an estimate lost anywhere else shifts every pair
    // after it, and the first shifted pair is a failure.
    let answered = run.samples.len().saturating_sub(1)..=run.samples.len();
    if !answered.contains(&run.estimates.len()) {
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

/// The scripts the scenario asked for reached the run: these ids, on these
/// cycles, in this order.
///
/// A scenario that only looked at what the cogs did could not tell a script the
/// session screened from one written to the wrong channel, dropped, or never
/// sent -- the run of a session nobody asked for anything and the run of a
/// session whose request went missing are identical in every other stream. This
/// is the assertion that the scenario really did say what it meant to say.
pub fn scripts_sent(run: &Run, wanted: &[(u32, i64)], failures: &mut Vec<String>) {
    if run.scripts.len() != wanted.len() {
        failures.push(format!(
            "the run replayed {} scripts, and the scenario sent {}",
            run.scripts.len(),
            wanted.len()
        ));
        return;
    }
    for (index, (found, (script_id, cycle))) in run.scripts.iter().zip(wanted).enumerate() {
        let at = cycle_at(*cycle);
        if found.at_ns != at {
            failures.push(format!(
                "script {index} reached the run at {}, and the scenario sent it at {at}",
                found.at_ns
            ));
        }
        if found.message.script_id() != *script_id {
            failures.push(format!(
                "script {index} is numbered {}, and the scenario sent {script_id}",
                found.message.script_id()
            ));
        }
        if found.message.arrival().as_nanos() != at {
            failures.push(format!(
                "script {index} is stamped {} and was sent at {at}: every offset in it is measured \
                 from that stamp, so the two disagreeing moves the whole schedule",
                found.message.arrival().as_nanos()
            ));
        }
    }
}

/// The cycles a session held the machine under command between.
#[derive(Clone, Copy)]
pub struct Engaged {
    /// The cycle the engagement took hold on: the arming concluded, and the
    /// schedule went out engaged.
    pub taken: i64,
    /// The cycle the session let go on: what the schedule asked for was over,
    /// and it went out disengaged.
    pub released: i64,
}

/// The ordinary life of a session, as the run narrates it.
///
/// The survey, a script taken, the machine armed, the schedule it ran, and the
/// release it ended at -- five phase changes and the two schedules that bracket
/// the one phase the machine is under command in. What this buys a scenario is
/// the two cycles: everything downstream of an engagement is asserted against
/// when the run says it began rather than against an allowance, and the
/// allowances are what the scenario checks the narration itself against.
pub fn engagement(run: &Run, failures: &mut Vec<String>) -> Option<Engaged> {
    engagement_then(run, &[], failures)
}

/// The ordinary life of a session, and then `then`: the phase changes the run
/// narrates after the release, in order.
///
/// For a scenario that carries on past the session it is about -- a second
/// script taken by the machine the first one let go of, most of all. What is
/// asserted about the session itself is [`engagement`]'s exactly; `then` is
/// appended to the sequence the narration is matched against, so the scenario
/// says what happened next rather than the checker tolerating anything.
pub fn engagement_then(
    run: &Run,
    then: &[(SessionPhaseWire, SessionPhaseWire)],
    failures: &mut Vec<String>,
) -> Option<Engaged> {
    let mut expected = vec![
        (SessionPhaseWire::RESTING, SessionPhaseWire::STARTING),
        (SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
        (SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING),
        (SessionPhaseWire::STOPPING, SessionPhaseWire::ACTIVE),
        (SessionPhaseWire::RESTING, SessionPhaseWire::STOPPING),
    ];
    expected.extend_from_slice(then);
    let cycles = phases(run, &expected, failures);
    let engaged = match (cycles.get(2), cycles.get(3)) {
        (Some(&taken), Some(&released)) => Some(Engaged { taken, released }),
        _ => None,
    };
    survey_cost(run, cycles.first().copied(), failures);
    schedules_published(run, engaged, failures);
    engaged
}

/// The session let go promptly: within one wake of the instant its schedule ran
/// out.
///
/// The end test costs no datagram and is made on every wake, so what stands
/// between the schedule running out and the session noticing is the wake floor
/// alone. A session that took longer is one whose wakes stopped coming.
pub fn ended_promptly(released: Option<i64>, last_step_end: i64, failures: &mut Vec<String>) {
    let Some(released) = released else {
        return;
    };
    if released < last_step_end {
        failures.push(format!(
            "the session let go on cycle {released} and its schedule ran to {last_step_end}: a \
             session that ended early ended something it was still under command for"
        ));
    }
    let floor = cycles_for(SESSION_WAKE_FLOOR_NS);
    if released > last_step_end + floor {
        failures.push(format!(
            "the session let go on cycle {released}, more than the {floor}-cycle wake floor past \
             the {last_step_end} its schedule ran out on"
        ));
    }
}

/// The session published what the machine was under command to do: engaged when
/// the arming concluded, disengaged when the session was over, and nothing
/// between.
///
/// Two messages and no more. A schedule is state rather than an event, so a
/// republication between changes would be the session saying the same thing
/// twice -- and the epoch bumping on each is the whole mechanism by which a
/// consumer holding the last one notices, so a pair under one epoch is a change
/// nobody would act on.
pub fn schedules_published(run: &Run, engaged: Option<Engaged>, failures: &mut Vec<String>) {
    let published: Vec<(i64, bool, u32)> = run
        .schedules
        .iter()
        .map(|logged| {
            (
                cycle_within(logged.at_ns),
                logged.message.engaged(),
                logged.message.epoch(),
            )
        })
        .collect();
    let [
        (took, took_engaged, took_epoch),
        (let_go, let_go_engaged, let_go_epoch),
    ] = published.as_slice()
    else {
        failures.push(format!(
            "the session published {} schedules; a session that ran is the one that engaged and \
             the one that ended: {published:?}",
            published.len()
        ));
        return;
    };
    if !took_engaged || *let_go_engaged {
        failures.push(format!(
            "the session published engaged = {took_engaged} and then {let_go_engaged}: the first \
             is the arming taking hold and the second is the session being over"
        ));
    }
    if let_go_epoch <= took_epoch {
        failures.push(format!(
            "the session published epoch {took_epoch} and then {let_go_epoch}: a change a \
             consumer is to notice carries a fresh epoch"
        ));
    }
    let Some(engaged) = engaged else {
        return;
    };
    if *took != engaged.taken {
        failures.push(format!(
            "the session published its engagement on cycle {took} and entered the phase on \
             {}: the publish and the phase are one decision",
            engaged.taken
        ));
    }
    if *let_go != engaged.released {
        failures.push(format!(
            "the session published its disengagement on cycle {let_go} and left the phase on \
             {}: the publish and the phase are one decision",
            engaged.released
        ));
    }
}

/// The schedules of a run whose session carried the machine down: the
/// engagement's, one stow per time the maneuver had to ask for one, and the one
/// nobody is running. How many stows there were, for a scenario that says
/// something about the count.
///
/// The stows are the whole of what a wind-down does to the machine, and three
/// things about them are the doctrine's. Each names the fold and nothing else,
/// because a machine being carried down is presenting nothing. Each carries a
/// fresh epoch, which is what makes a stow commanded again news to a tick
/// holding the last one. And every one of them ends at the same instant: the
/// maneuver's clock is opened once, and a condition arriving on the way down
/// re-commands the stow on what is left of it rather than granting a new one.
///
/// `entered` is the cycle the maneuver was entered on, which the first stow is
/// expected to go out on: entering the maneuver and asking for it are one
/// decision.
pub fn stows(run: &Run, entered: Option<i64>, failures: &mut Vec<String>) -> usize {
    stows_until(run, entered, None, failures)
}

/// The same, over the schedules published up to and including cycle `until`.
///
/// For a run that engages the machine again after the one that was carried down:
/// the schedules of the next session are the next session's business, and the
/// wind-down's are the ones this says something about. `None` is the whole run.
pub fn stows_until(
    run: &Run,
    entered: Option<i64>,
    until: Option<i64>,
    failures: &mut Vec<String>,
) -> usize {
    let mut stows = Vec::new();
    let mut epochs = Vec::new();
    let published: Vec<_> = run
        .schedules
        .iter()
        .filter(|logged| until.is_none_or(|until| cycle_within(logged.at_ns) <= until))
        .collect();
    let last = published.len().saturating_sub(1);
    for (index, logged) in published.iter().enumerate() {
        let at = cycle_within(logged.at_ns);
        let schedule = &logged.message;
        epochs.push(schedule.epoch());
        if index == 0 {
            if !schedule.engaged() {
                failures.push(format!(
                    "the first schedule went out at cycle {at} with nobody engaged on it: the \
                     first thing a session publishes is the arming taking hold"
                ));
            }
            continue;
        }
        if index == last {
            if schedule.engaged() {
                failures.push(format!(
                    "the last schedule went out at cycle {at} engaged: a session that is over \
                     publishes a schedule nobody is running"
                ));
            }
            continue;
        }
        let steps: Vec<(StepKindWire, PostureWire, i64)> = schedule
            .steps()
            .iter()
            .map(|step| (step.kind(), step.posture(), step.end().as_nanos()))
            .collect();
        match steps.as_slice() {
            [(StepKindWire::BASE_POSTURE, PostureWire::STOW, end)] => stows.push((at, *end)),
            other => failures.push(format!(
                "the schedule at cycle {at} asks for {other:?}, and a machine being carried down \
                 is asked for the fold and nothing else"
            )),
        }
        if !schedule.overlays().is_empty() {
            failures.push(format!(
                "the stow at cycle {at} carries {} overlay windows: a machine being carried down \
                 is presenting nothing",
                schedule.overlays().len()
            ));
        }
    }
    if stows.is_empty() {
        failures.push(
            "the session published no stow: the response to a condition it answers under control \
             is a schedule that folds the machine"
                .to_string(),
        );
    }
    if let (Some(&(first, _)), Some(entered)) = (stows.first(), entered)
        && first != entered
    {
        failures.push(format!(
            "the maneuver was entered on cycle {entered} and its stow went out on {first}: \
             entering it and asking for it are one decision"
        ));
    }
    for window in stows.windows(2) {
        if window[0].1 != window[1].1 {
            failures.push(format!(
                "the stow at cycle {} ends at {} and the one at cycle {} ends at {}: the \
                 maneuver's clock is opened once and never again",
                window[0].0, window[0].1, window[1].0, window[1].1
            ));
        }
    }
    for pair in epochs.windows(2) {
        if pair[1] <= pair[0] {
            failures.push(format!(
                "the session published epoch {} and then {}: a change a consumer is to notice \
                 carries a fresh epoch",
                pair[0], pair[1]
            ));
        }
    }
    stows.len()
}

/// The session published no schedule: nothing asked the machine for anything, so
/// nothing was ever under command.
pub fn no_schedules(run: &Run, failures: &mut Vec<String>) {
    for schedule in &run.schedules {
        failures.push(format!(
            "the session published a schedule at {} carrying engaged = {}: nothing in this run \
             asked the machine for anything",
            schedule.at_ns,
            schedule.message.engaged()
        ));
    }
}

/// The session's whole narration, kind for kind and in order.
///
/// The reports are the session's own account of the run, and the order they are
/// in is part of it: an acceptance narrated after the phase it moved to, or a
/// session ended before the schedule that ended it went out, would be a story
/// told wrong about a run that went right. One report leaves the cog per wake,
/// oldest first, so the log's order is the order the ring was written in.
pub fn narration(run: &Run, expected: &[ReportKindWire], failures: &mut Vec<String>) {
    let told: Vec<ReportKindWire> = run
        .reports
        .iter()
        .map(|report| report.message.kind())
        .collect();
    if told != expected {
        failures.push(format!(
            "the session told: {told:?}\nand this scenario's story is: {expected:?}"
        ));
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

/// One step is long enough for the move it asks for -- every group of it.
///
/// The scenario's own arithmetic, checked rather than trusted: a step shorter
/// than the move it asks for would make every arrival assertion pass about a
/// machine that was still travelling. Judged against the longest clock any group
/// runs on rather than against the configured duration, because the antenna pair
/// is parted at its crossing and arrives after the head does: a span between the
/// two would satisfy a guard that only knew the head's and still leave the
/// antennas in flight for the arrival it is protecting.
pub fn room(what: &str, step_cycles: i64, clocks: &MoveClocks, failures: &mut Vec<String>) {
    let move_cycles = clocks.cycles();
    if step_cycles <= move_cycles {
        failures.push(format!(
            "the {what} step is {step_cycles} cycles and the move to it takes {move_cycles} for \
             {}: the scenario does not leave the machine time to arrive",
            clocks.longest_group()
        ));
    }
}

/// Nothing was ever commanded: the goal stream never started.
///
/// `why` says what the run is that nothing was asked of the machine in it. The
/// mover streams what a schedule tells it to, so a goal on a run with no
/// engagement in it is a decision tick running on an engagement nobody granted
/// it.
pub fn no_goals(run: &Run, why: &str, failures: &mut Vec<String>) {
    if !run.goals.is_empty() {
        failures.push(format!(
            "{} goals were published on a run where {why}",
            run.goals.len()
        ));
    }
}

/// The scripts the session refused, with the reason it gave and the cycle the
/// one it answers was sent on.
///
/// The reason is the whole of the assertion: a sender told the machine was busy
/// would keep asking, and `parked` is the one refusal that says nothing will
/// take a script until an operator has been. The mirror of [`scripts_sent`],
/// which is the accepted half.
pub fn refusals(run: &Run, expected: &[(u32, RefusalReasonWire, i64)], failures: &mut Vec<String>) {
    let refused: Vec<(i64, u32, u32)> = run
        .reports
        .iter()
        .filter(|report| report.message.kind() == ReportKindWire::SCRIPT_REFUSED)
        .map(|report| {
            (
                cycle_within(report.message.time().as_nanos()),
                report.message.a(),
                report.message.b(),
            )
        })
        .collect();
    if refused.len() != expected.len() {
        failures.push(format!(
            "the session refused {refused:?}, and this run expects {} refusal(s): {expected:?}",
            expected.len()
        ));
        return;
    }
    for ((at, script_id, reason), (wanted_id, wanted_reason, sent_on)) in
        refused.iter().zip(expected)
    {
        let wanted_reason = u32::from(wanted_reason.0);
        if *script_id != *wanted_id || *reason != wanted_reason {
            failures.push(format!(
                "the session refused script {script_id} for reason {reason}, and this run sends \
                 script {wanted_id} to a machine that answers {wanted_reason}"
            ));
        }
        // A window and not a floor. The session decides a refusal on the wake
        // the script arrives on -- a message is a wake, and the report carries
        // the instant it was decided at rather than the instant it was published
        // -- so a refusal dated a wake floor and a cycle past the script is an
        // intake that answered on a later wake than the one it was woken by.
        let by = *sent_on + cycles_for(SESSION_WAKE_FLOOR_NS) + 1;
        if *at < *sent_on || *at > by {
            failures.push(format!(
                "the refusal is dated cycle {at}, and the script it answers was sent on \
                 {sent_on}: a script is answered on the wake it arrives on, so the answer is due \
                 by cycle {by}"
            ));
        }
    }
}

/// How many times a scenario expects one condition to have been recorded.
#[derive(Clone, Copy, Debug)]
pub enum Recorded {
    /// Exactly this many times, which is the sharper assertion: a condition that
    /// latches in the servo is carried by every lap of the rotation, so one
    /// recorded twice is one being recorded at the poll rate.
    Times(usize),
    /// At least once, for a condition whose count is a fact about the run rather
    /// than about the scenario -- a tracking raise over a stretch, most of all.
    AtLeastOnce,
}

/// One condition a scenario expects to find in the session's timeline.
pub struct Expected<'a> {
    /// The fault kind the record names.
    pub kind: FaultKindWire,
    /// The rows the record may name. A record naming anything else is a run
    /// about a different machine than the scenario arranged. An empty set is the
    /// assertion that it names no single row, which is what a condition of the
    /// bus rather than of a servo carries.
    pub rows: JointFlags,
    /// The first cycle it can have been recorded on.
    pub from: i64,
    /// The last cycle it can have been recorded on.
    pub through: i64,
    /// How many of them there are.
    pub how_many: Recorded,
    /// Whether the decision tick is what raised it. The raises the tick
    /// published and the faults the session recorded from them are the same
    /// events -- the session is that channel's only reader -- so the records of
    /// every tick-raised condition are counted against `run.faults` below.
    pub raised_by_tick: bool,
    /// What the condition is, for the failure lines: a phrase that completes
    /// "the run says ...".
    pub why: &'a str,
}

/// The conditions the session recorded, and nothing else.
///
/// Every `fault_recorded` entry in the timeline is matched to the one
/// expectation naming its kind: an entry naming a kind no expectation does is a
/// run with something else wrong with it, and every matched entry has to name a
/// row the scenario arranged and fall in the window the scenario placed it in.
///
/// Answers with the cycles each expectation was recorded on, in the order the
/// expectations were given, for a scenario that goes on to place something else
/// from them.
pub fn faults_recorded(
    run: &Run,
    expected: &[Expected],
    failures: &mut Vec<String>,
) -> Vec<Vec<i64>> {
    let mut found: Vec<Vec<i64>> = expected.iter().map(|_| Vec::new()).collect();
    // The kind is what a record is matched on, so two expectations naming one
    // kind would send every record of it to the first and leave the second's
    // rows and window asserted against nothing. Refused here rather than
    // resolved: which of two same-kind expectations a record belongs to is the
    // scenario's statement to make, and this helper has no way to ask.
    for (index, want) in expected.iter().enumerate() {
        for prior in &expected[..index] {
            if prior.kind.0 == want.kind.0 {
                failures.push(format!(
                    "this run expects {} and {} to be recorded under the same fault kind, and a \
                     record is matched to an expectation by its kind alone",
                    prior.why, want.why
                ));
            }
        }
    }
    for report in &run.reports {
        if report.message.kind() != ReportKindWire::FAULT_RECORDED {
            continue;
        }
        let at = cycle_within(report.message.time().as_nanos());
        let joint = joint_of(JointRefWire(
            u8::try_from(report.message.b()).unwrap_or(u8::MAX),
        ));
        let matched = expected
            .iter()
            .position(|want| report.message.a() == u32::from(want.kind.0));
        let Some(index) = matched else {
            failures.push(format!(
                "the session recorded fault {} at cycle {at}, and this run has nothing in it but \
                 {}",
                report.message.a(),
                expected
                    .iter()
                    .map(|want| want.why)
                    .collect::<Vec<_>>()
                    .join(" and ")
            ));
            continue;
        };
        let want = &expected[index];
        match joint {
            Some(joint) if flags::contains(want.rows, joint) => {}
            None if flags::is_empty(want.rows) => {}
            other => failures.push(format!(
                "the session recorded {} at cycle {at} naming {other:?}, and the rows this run \
                 arranged it on are {}",
                want.why,
                flags::Names(want.rows)
            )),
        }
        if at < want.from || at > want.through {
            failures.push(format!(
                "{} was recorded at cycle {at}, outside the {}..{} this run can have it in",
                want.why, want.from, want.through
            ));
        }
        found[index].push(at);
    }
    let mut from_tick = 0;
    for (want, at) in expected.iter().zip(&found) {
        match want.how_many {
            Recorded::Times(times) if at.len() != times => failures.push(format!(
                "the session recorded {} on cycles {at:?}, and this run has {times} of them: a \
                 condition recorded more often than that is being recorded at the rate it is read \
                 rather than at the rate it happens",
                want.why
            )),
            Recorded::AtLeastOnce if at.is_empty() => failures.push(format!(
                "the session recorded nothing about {}, which this run is about",
                want.why
            )),
            _ => {}
        }
        if want.raised_by_tick {
            from_tick += at.len();
        }
    }
    if expected.iter().any(|want| want.raised_by_tick) && from_tick != run.faults.len() {
        failures.push(format!(
            "the tick raised {} times and the session recorded {from_tick} of them: the session is \
             that channel's only reader, and a raise it never recorded is a session not hearing \
             the tick",
            run.faults.len()
        ));
    }
    found
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

/// The start-up survey: what every run in this suite begins with.
///
/// The session commissions the machine before it will take a script -- nine
/// servos pinged, identified, their provisioning read, the rail waited on, the
/// health bytes read and the gains written -- and nothing in that touches torque
/// or moves anything. What it costs a scenario is the two hundred transactions
/// it spends over the aux path, which is why every run here is long enough to
/// have one in it.
///
/// What is asserted is the shape of it: a session narrates a phase change when
/// the survey ends, exactly once, and the phase it ends in is resting. A run
/// that ends mid-survey narrates none, which is not a failure -- the survey is
/// not what most of these scenarios are about. A survey that *failed* parks the
/// machine, and that is the one thing this refuses, because a parked session
/// takes no script and every assertion downstream of one would be about a run
/// that never started.
///
/// Answers the cycle the survey finished on, for a scenario that measures from
/// it.
pub fn commissioning(run: &Run, failures: &mut Vec<String>) -> Option<i64> {
    let mut finished = None;
    for report in &run.reports {
        if report.message.kind() != ReportKindWire::PHASE_CHANGED {
            continue;
        }
        let at = cycle_within(report.message.time().as_nanos());
        let to = SessionPhaseWire(u8::try_from(report.message.a()).unwrap_or(u8::MAX));
        let from = SessionPhaseWire(u8::try_from(report.message.b()).unwrap_or(u8::MAX));
        if to != SessionPhaseWire::RESTING || from != SessionPhaseWire::STARTING {
            failures.push(format!(
                "the session moved from {from:?} to {to:?} at cycle {at}: the only phase change \
                 this scenario has a survey for is the one that ends it at rest"
            ));
            continue;
        }
        if let Some(earlier) = finished {
            failures.push(format!(
                "the survey ended twice, at cycles {earlier} and {at}: it runs once per process"
            ));
        }
        finished = Some(at);
    }
    survey_cost(run, finished, failures);
    finished
}

/// What the survey cost the run: nothing given up on, and one datagram per
/// transaction the sequence has.
///
/// The traffic is the assertion nothing else makes. Every instant a scenario
/// names downstream of the survey is derived from the same allowance the survey
/// is given, so a regression that re-issued every datagram once -- a delivery
/// window compared the wrong way, a pending record whose issue instant stopped
/// being refreshed -- would double the survey's traffic and move every derived
/// instant with it, and the whole suite would stay green. Counting the datagrams
/// against the sequence's own transaction count is what notices.
///
/// Skipped for a run that ends mid-survey: there is no count to compare a part
/// of a sweep against. The refusals are checked either way.
fn survey_cost(run: &Run, finished: Option<i64>, failures: &mut Vec<String>) {
    for report in &run.reports {
        if report.message.kind() == ReportKindWire::AUX_GAVE_UP {
            failures.push(format!(
                "the session gave up on a datagram at {}: every channel in this system is memory, \
                 so a transaction goes unanswered only where a scenario arranged for it",
                report.message.time().as_nanos()
            ));
        }
    }
    let Some(finished) = finished else {
        return;
    };
    let spent = run
        .datagrams
        .iter()
        .filter(|datagram| cycle_within(datagram.at_ns) <= finished)
        .count() as i64;
    let wanted = commission_transactions();
    if spent != wanted {
        failures.push(format!(
            "the survey spent {spent} datagrams and its sequence has {wanted} transactions in it: \
             a survey that cost more re-issued something, and one that cost less did not run \
             every sweep"
        ));
    }
}

/// Every phase change the session narrated, against the sequence a scenario
/// says it should have.
///
/// The strict form of [`commissioning`], for a scenario whose session does more
/// than come up: the changes are matched in order, pair for pair, and a run with
/// one too many or one too few is a run whose session took a different path.
/// Answers the cycle each one happened on, so a scenario can say when as well as
/// what.
pub fn phases(
    run: &Run,
    expected: &[(SessionPhaseWire, SessionPhaseWire)],
    failures: &mut Vec<String>,
) -> Vec<i64> {
    let mut cycles = Vec::new();
    for report in &run.reports {
        if report.message.kind() != ReportKindWire::PHASE_CHANGED {
            continue;
        }
        let at = cycle_within(report.message.time().as_nanos());
        let to = SessionPhaseWire(u8::try_from(report.message.a()).unwrap_or(u8::MAX));
        let from = SessionPhaseWire(u8::try_from(report.message.b()).unwrap_or(u8::MAX));
        match expected.get(cycles.len()) {
            Some((wanted_to, wanted_from)) if (to, from) == (*wanted_to, *wanted_from) => {}
            Some((wanted_to, wanted_from)) => failures.push(format!(
                "the session's phase change number {} is {from:?} to {to:?} at cycle {at}, and \
                 this scenario's is {wanted_from:?} to {wanted_to:?}",
                cycles.len() + 1
            )),
            None => failures.push(format!(
                "the session moved from {from:?} to {to:?} at cycle {at}, and this scenario has \
                 {} phase changes in it",
                expected.len()
            )),
        }
        cycles.push(at);
    }
    if cycles.len() < expected.len() {
        failures.push(format!(
            "the session narrated {} phase changes, and this scenario has {}",
            cycles.len(),
            expected.len()
        ));
    }
    cycles
}

/// The cycle the driver drained the last datagram the session published.
///
/// What the dead-man's window opens on in a run where nothing is commanded:
/// every accepted datagram is liveness, so the silence the gate measures starts
/// at the last one and not at the run's first cycle.
///
/// Answers `None` for a run where the session asked for nothing at all, which is
/// a session that never executed -- and a failure, since the survey is the first
/// thing it does.
pub fn last_datagram(run: &Run, failures: &mut Vec<String>) -> Option<i64> {
    let mut last = None;
    for datagram in &run.datagrams {
        if datagram.message.kind() == SessionCmdKindWire::NONE {
            failures.push(format!(
                "the session published a datagram asking nothing at {}",
                datagram.at_ns
            ));
        }
        last = Some(drain_cycle(datagram.at_ns));
    }
    if last.is_none() {
        failures.push(
            "the session asked the driver for nothing all run: the start-up survey is the first \
             thing it does, so a run with no datagrams in it is a session that never ran"
                .to_string(),
        );
    }
    last
}

/// The configured servo profile and watchdog, as they reached the servos.
///
/// The writes the commissioning sweep makes out of
/// [`SessionParams`](crate::PROFILE_ACCELERATION)'s profile fields, found in the
/// run's own datagrams and compared against what the file said. `check_params`
/// pins the file to the constants; this pins the constants to the wire, so the
/// pair together is the whole path from the text a deployment edits to the
/// register a servo holds.
///
/// Every commissioned row is checked, because the sweep writes to all nine and a
/// machine provisioned with one servo left at whatever it held is a machine
/// whose motion is not the motion anybody configured.
///
/// The watchdog is checked as a sequence rather than a value: a latched register
/// refuses the arming write, so the clear has to come first, and the *order* is
/// the part a single-value check would miss.
pub fn commissioned_profile(run: &Run, failures: &mut Vec<String>) {
    let mut written = 0_usize;
    let mut watchdog: BTreeMap<u8, Vec<i64>> = BTreeMap::new();
    for datagram in &run.datagrams {
        let txn = datagram.message.txn();
        if datagram.message.kind() != SessionCmdKindWire::AUX
            || txn.op() != AuxOpKindWire::WRITE_REG_VERIFIED
        {
            continue;
        }
        let value = i64::try_from(txn.value()).unwrap_or(i64::MAX);
        let expected = match txn.reg() {
            RegIdWire::PROFILE_ACCELERATION => PROFILE_ACCELERATION,
            RegIdWire::PROFILE_VELOCITY => PROFILE_VELOCITY,
            RegIdWire::BUS_WATCHDOG => {
                watchdog.entry(txn.id()).or_default().push(value);
                continue;
            }
            _ => continue,
        };
        written += 1;
        if value != expected {
            failures.push(format!(
                "the session wrote {value} to servo {}'s {:?} at {} and the configuration says \
                 {expected}",
                txn.id(),
                txn.reg(),
                datagram.at_ns
            ));
        }
    }
    // Two registers on each of the nine rows, and the sweep runs once per
    // process: a run that wrote fewer of them commissioned a machine this check
    // has no evidence about.
    let expected_writes = 2 * ROW_COUNT;
    if written != expected_writes {
        failures.push(format!(
            "the commissioning sweep wrote the profile {written} times and there are \
             {expected_writes} of those writes in a commissioned machine"
        ));
    }
    if watchdog.len() != ROW_COUNT {
        failures.push(format!(
            "the commissioning sweep armed the watchdog on {} servos and a commissioned machine \
             has all {ROW_COUNT} of them watched",
            watchdog.len()
        ));
    }
    for (id, values) in &watchdog {
        if values.as_slice() != [0, BUS_WATCHDOG] {
            failures.push(format!(
                "the session wrote {values:?} to servo {id}'s watchdog and arming it is a clear \
                 then the configured [0, {BUS_WATCHDOG}]"
            ));
        }
    }
}

/// The cycle the driver drained the first release the session published.
///
/// The confirmation is a bus-row count of cycles after the command reaches the
/// driver, and the command is republished every wake until it is confirmed, so
/// the *first* one is what the handshake is measured from.
pub fn first_release(run: &Run, failures: &mut Vec<String>) -> Option<i64> {
    let released = run
        .datagrams
        .iter()
        .find(|logged| logged.message.kind() == SessionCmdKindWire::TORQUE_OFF_NOW)
        .map(|logged| drain_cycle(logged.at_ns));
    if released.is_none() {
        failures.push(
            "the session never told the driver to let go: every response that ends a session ends \
             it de-torqued"
                .to_string(),
        );
    }
    released
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

/// The outage is exactly where the scenario put it: every sample inside `blind`
/// says the bus answered for no row, and every sample outside it carries a
/// reading.
///
/// The window's own boundaries are the assertion. An injection takes effect on
/// the cycle it names -- the driver drains what arrived and then advances the
/// plant -- so the first blind sample is the one for the cycle the outage was
/// published on, and the reads are back on the cycle after the last of them. A
/// run whose outage is meant to outlive it names a window past its own end.
pub fn outage(run: &Run, blind: Range<i64>, failures: &mut Vec<String>) {
    for sample in &run.samples {
        let sample = &sample.message;
        let Ok(cycle) = cycle_of(sample.nominal_time().as_nanos()) else {
            continue;
        };
        let wanted = blind.contains(&cycle);
        let dark = !sample.present_valid();
        if dark != wanted {
            failures.push(format!(
                "the sample at cycle {cycle} says its reading is {}, and the outage runs over \
                 cycles {}..{}",
                if dark { "missing" } else { "present" },
                blind.start,
                blind.end
            ));
            return;
        }
        // A driver that read nothing says so twice: the flag and the set of rows
        // it did not hear from. A sample carrying one without the other is a
        // receiver's choice about which to believe.
        let missing = joint_set(sample.missing());
        let masked = !missing.is_ok_and(flags::is_empty);
        if masked != dark {
            let named = match missing {
                Ok(set) => flags::Names(set).to_string(),
                Err(complaint) => complaint.to_string(),
            };
            failures.push(format!(
                "the sample at cycle {cycle} says the rows missing are {named} and its validity \
                 flag is {}: the two say different things about the same reading",
                sample.present_valid()
            ));
            return;
        }
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

/// The positions the goal decided on `cycle` asked for, complaining if the log
/// has none.
///
/// The complaint is the point: a per-cycle assertion whose cycle is not in the
/// log has not passed, and `what` says what the machine was doing on that cycle
/// so the line reads as a missing message rather than as a missing property.
/// One wording for it across the suite, because a scenario author developing a
/// run meets this failure more often than any other.
pub fn goal_at_or(
    run: &Run,
    cycle: i64,
    what: &str,
    failures: &mut Vec<String>,
) -> Option<[f64; 9]> {
    let found = goal_at(run, cycle);
    if found.is_none() {
        failures.push(format!(
            "no goal for cycle {cycle}, where the machine is {what}"
        ));
    }
    found
}

/// The commanded antenna pair is de-phased over `[from, until)`: the two sides
/// are not brought to a stop together.
///
/// Two antennas swept between the stow and the working posture travel mirrored
/// arcs, and mirrored arcs put both tips at the point where the arcs cross at
/// the same instant -- the one collision on record. The motion library answers
/// it by lengthening the later side's clock, and a caller that does not ask it
/// to gets a pair commanded straight through the contact band. Nothing about the
/// modelled plant can show a collision, so this is the property that stands in
/// for it: what the goal stream carries is one side still moving after the other
/// has arrived, by at least `least` cycles.
///
/// `least` is a floor and not a measurement -- which cycle each side settles on
/// depends on the grid the run samples and on where a shaped tail falls under
/// the threshold below -- so a scenario derives it from the clocks the move
/// actually runs on ([`crate::MoveClocks::parting_least`]) and the run clears it
/// or does not.
pub fn pair_de_phased(
    run: &Run,
    what: &str,
    from: i64,
    until: i64,
    least: i64,
    failures: &mut Vec<String>,
) {
    // A tenth of a milliradian: two orders of magnitude above the arithmetic and
    // far below anything a shaped clock's tail asks for.
    const MOVED: f64 = 1e-4;
    let mut settled = [None; ANTENNAS.len()];
    let mut previous: Option<[f64; ROW_COUNT]> = None;
    for cycle in from..until {
        let Some(targets) = goal_at(run, cycle) else {
            continue;
        };
        if let Some(before) = previous {
            for (antenna, slot) in ANTENNAS {
                let Some(row) = row(antenna) else {
                    continue;
                };
                if (targets[row] - before[row]).abs() > MOVED {
                    settled[slot] = Some(cycle);
                }
            }
        }
        previous = Some(targets);
    }
    let (Some(right), Some(left)) = (settled[0], settled[1]) else {
        failures.push(format!(
            "one of the antennas was never commanded to move over the {what}, so the pair's \
             phase says nothing: {settled:?}"
        ));
        return;
    };
    if (right - left).abs() < least {
        failures.push(format!(
            "over the {what} the antennas stopped {} cycles apart, under the {least} the pair \
             separation asks for: the right settled on {right} and the left on {left}",
            (right - left).abs()
        ));
    }
}

/// What the machine read on `cycle`, complaining if the log has no sample for
/// it: [`goal_at_or`]'s other half, on the same terms.
pub fn sample_at_or<'a>(
    run: &'a Run,
    cycle: i64,
    what: &str,
    failures: &mut Vec<String>,
) -> Option<&'a PoseSampleWire> {
    let found = sample_at(run, cycle);
    if found.is_none() {
        failures.push(format!(
            "no sample for cycle {cycle}, where the machine is {what}"
        ));
    }
    found
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

/// The driver raised exactly one event of `kind`, on the cycle the scenario's
/// arithmetic names, carrying the silence it should carry.
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
/// Says nothing about the other kinds in the log: what a run is allowed to have
/// raised at all is [`only_kinds`], because a de-torquing that latches is
/// followed by the driver reading it back and saying so, and every scenario
/// with a latch in it therefore has two events rather than one.
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

/// Every event in the log names one of `kinds`.
///
/// The exhaustiveness half of an event assertion: [`sole_event`] says what a
/// kind did, and this says the run raised nothing else. Kept apart because a
/// scenario with a de-torquing in it expects two events -- the latch and the
/// read-back that confirmed it -- and a helper that judged one kind could only
/// ever call the other a surprise.
pub fn only_kinds(run: &Run, kinds: &[EventKind], failures: &mut Vec<String>) {
    for event in &run.events {
        let event = &event.message;
        let Some(raised) = event.kind().to_known() else {
            // Reported by `sole_event`, which reads the same log; naming it
            // twice would make one unreadable event two failures.
            continue;
        };
        if !kinds.contains(&raised) {
            failures.push(format!(
                "the driver raised {raised} at {}, and this scenario asks it for {kinds:?}",
                event.time().as_nanos()
            ));
        }
    }
}

/// A commanded de-torquing was read back and confirmed, one servo per cycle.
///
/// The handshake the driver owes whoever took torque away: the sweep is being
/// written, and the confirmation says the machine actually went limp. A whole
/// clean pass over the bus is the confirmation, and the pass reads one row per
/// cycle, so it lands a bus-row count of cycles after the latch -- arithmetic,
/// like every other instant a scenario names.
///
/// An `unconfirmed` report is a failure here and not a second expectation: it
/// says a row was still holding torque after the budget ran out, and every
/// scenario in this suite de-torques a machine the plant lets go of.
pub fn confirmed_off(run: &Run, latched: Option<i64>, failures: &mut Vec<String>) {
    let Some(latched) = latched else {
        return;
    };
    for event in &run.events {
        if event.message.kind().to_known() == Some(EventKind::TorqueOffUnconfirmed) {
            failures.push(format!(
                "the driver could not confirm the de-torquing it commanded, at {}: the modelled \
                 servos go limp when they are told to",
                event.message.time().as_nanos()
            ));
        }
    }
    sole_event(
        run,
        EventKind::TorqueOffConfirmed,
        latched + ROW_COUNT as i64,
        0,
        failures,
    );
}

/// The same handshake, commanded while the bus was answering nothing.
///
/// A read-back is a round trip like any other, so a pass opened during an outage
/// reads nothing and credits nothing: the driver says it cannot confirm the
/// de-torquing once its own budget is spent, and confirms it a bus-row count of
/// cycles after the reads come back. Both are asserted, because the pair *is* the
/// contract -- a driver that confirmed a de-torquing it never read back would be
/// crediting silence, and one that never confirmed after the bus returned would
/// have stopped reading.
///
/// `latched` is the cycle the release was commanded on and `reads_back` the first
/// cycle the bus answers on again.
pub fn confirmed_off_when_the_bus_returns(
    run: &Run,
    latched: i64,
    reads_back: i64,
    failures: &mut Vec<String>,
) {
    // The pass reports once its budget is *past*, so the saying lands the cycle
    // after the budget's own last one.
    let said_at = latched + cycles_for(DRIVER_CONFIRM_BUDGET_NS) + 1;
    sole_event(run, EventKind::TorqueOffUnconfirmed, said_at, 0, failures);
    sole_event(
        run,
        EventKind::TorqueOffConfirmed,
        reads_back + ROW_COUNT as i64,
        0,
        failures,
    );
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
        if !run.census.iter().any(|(name, _)| name.starts_with(&wanted)) {
            let carried: Vec<&str> = run.census.iter().map(|(name, _)| name.as_str()).collect();
            failures.push(format!(
                "the log carries no channel named {wanted}*, so {cog}'s {REPORT_GROUP} group did \
                 not reach it; the channels it does carry are {carried:?}"
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
    stands_still_rows(run, flags::all(), from_cycle, through_cycle, why, failures);
}

/// `rows` of the machine stand still, on [`stands_still`]'s terms.
///
/// For the responses that are scoped to a group: an antenna pair let go of while
/// the head keeps its presence is a machine two rows of which have stopped
/// answering their goals and seven of which have not, so what is asserted is per
/// row rather than about the machine.
pub fn stands_still_rows(
    run: &Run,
    rows: JointFlags,
    from_cycle: i64,
    through_cycle: i64,
    why: &str,
    failures: &mut Vec<String>,
) {
    // The baseline has to be a reading. An unreadable sample carries no
    // positions at all, and taking one as the baseline would compare a window of
    // dark samples against zeros it invented -- an assertion that cannot fail,
    // pointed at a stretch of the run nobody could see.
    let stood = match sample_at(run, from_cycle).map(|sample| sample.present().validate()) {
        Some(Ok(present)) => rows_of(present),
        Some(Err(complaint)) => {
            failures.push(format!(
                "the sample for cycle {from_cycle} carries no reading ({complaint}), and the \
                 machine standing still from there is measured against what it read on it"
            ));
            return;
        }
        None => {
            failures.push(format!(
                "no sample for cycle {from_cycle}, where the machine was {why}"
            ));
            return;
        }
    };
    for sample in &run.samples {
        let sample = &sample.message;
        let Ok(cycle) = cycle_of(sample.nominal_time().as_nanos()) else {
            continue;
        };
        if cycle < from_cycle || cycle > through_cycle {
            continue;
        }
        // A sample nobody could read says nothing about where the machine is.
        // Where the darkness itself is the assertion, [`outage`] is what makes
        // it.
        let Ok(present) = sample.present().validate() else {
            continue;
        };
        let reads = rows_of(present);
        for joint in flags::iter(rows) {
            let Some(row) = row(joint) else {
                failures.push(format!("{} sits on no bus row", Name(joint)));
                continue;
            };
            if reads[row] != stood[row] {
                failures.push(format!(
                    "{} moved by cycle {cycle}, reading {} where it stood at {}, and from cycle \
                     {from_cycle} it was {why}",
                    Name(joint),
                    reads[row],
                    stood[row]
                ));
                return;
            }
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
    let [log_dir, mover_params, session_params, sim_params] = args.as_slice() else {
        eprintln!("usage: {name} <output-log-dir> <mover-params> <session-params> <sim-params>");
        return ExitCode::FAILURE;
    };

    let mut failures = crate::check_params(mover_params, session_params, sim_params);
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
