//! S11's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What S11 shares with a healthy run is asserted by
//! `scenario::check` -- and *most of it holds*, which is the point of the run:
//! the session that answered a fault here is the session that ran its schedule
//! out and let go of the machine at rest, with the same five phase changes and
//! the same two schedules S1 has.
//!
//! What is here is the group-scoped answer. The condition read off the driver's
//! rotation and classified as the antennas'; the two verified torque-off writes
//! the session issues itself; the report that says the pair let go; the tick
//! taking the limp pair out of service when the fold is commanded and carrying
//! the move on with what remains; the head reaching that fold; and the release
//! reporting the antennas it could not find there.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__cogs__session_cmd_clk_rs::SessionCmdKindWire;
use brenn_reachy__hardware__dynamixel__registers_clk_rs::RegIdWire;
use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKindWire;
use brenn_reachy__motion__faults_clk_rs::{FaultKindWire, ResponseKindWire};
use brenn_reachy__motion__joints_clk_rs::JointFlagsWire;
use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use motion_cogs::session_bus::disarm_config;
use reachy_kin::wrap_to_pi;
use reachy_motion::arm::row_of_id;
use reachy_motion::joints::{Name, ROWS, flags, row};
use reachy_motion::tick::ResponseKind;
use scenario::check;
use scenario::check::present_rows;
use scenario::cycle_within;
use scenario::read::Run;
use scenario::{stow_clocks, up_clocks};

use s11_scenario::{
    SCRIPT_ID, STOW_CYCLES, UP_CYCLES, answered_by_cycle, degraded_rows, disengage_cycle,
    end_cycle, fault_cycle, faulted_joint, released_by_cycle, script_sent_cycle, stow_start_cycle,
    up_start_cycle,
};

fn main() -> ExitCode {
    check::main("s11_checker", |run, failures| {
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        check::scripts_sent(run, &[(SCRIPT_ID, script_sent_cycle())], failures);
        // The ordinary life of a session, in full: a fault answered by letting
        // go of a pair is a fault that changes nothing about the phases or about
        // the two schedules that bracket them. A run that stowed, parked or
        // republished anything fails here.
        let engaged = check::engagement(run, failures);
        check::ended_promptly(
            engaged.map(|engaged| engaged.released),
            disengage_cycle(),
            failures,
        );
        // The one healthy-run property this scenario weakens, and it weakens it
        // by naming exactly what leaves: the tick takes the limp pair out of
        // service when the fold is commanded, so its goals speak for seven rows
        // from then on and for nine before.
        let stream = check::goal_stream_without(run, degraded_rows(), failures);
        if let (Some(stream), Some(engaged)) = (stream, engaged) {
            check::stream_starts_with_session(&stream, engaged.taken, failures);
            check::stream_stops_with_release(&stream, engaged.released, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        check::room("upright", UP_CYCLES, &up_clocks(), failures);
        check::room("stow", STOW_CYCLES, &stow_clocks(), failures);
        check_the_answer_fits_the_posture(failures);

        // The two conditions of this run, and nothing else: the servo's own
        // account of itself, once -- the byte latches in the servo and the
        // rotation carries it on every lap, so a session that recorded what it
        // read would fill the timeline with one standing condition at the poll
        // rate -- and the pair that stopped closing on the fold once nothing
        // held it. The second is the tick's and is *expected*: the schedule's
        // last step asks the antennas to fold and nothing holds them any more.
        // Where it falls is the assertion: not before the fold was commanded,
        // because a limp antenna holding the posture it had already reached
        // follows it perfectly and an obstruction raised while nothing was
        // asking the pair to move is a mechanical claim this scenario never
        // made. That the pair had let go by then is the premise asserted
        // above.
        check::faults_recorded(
            run,
            &[
                check::Expected {
                    kind: FaultKindWire::ANTENNA_SERVO_FAULT,
                    rows: flags::bit(faulted_joint()),
                    from: fault_cycle(),
                    through: answered_by_cycle(),
                    how_many: check::Recorded::Times(1),
                    raised_by_tick: false,
                    why: "the antenna complaining about itself",
                },
                check::Expected {
                    kind: FaultKindWire::ANTENNA_OBSTRUCTED,
                    rows: degraded_rows(),
                    from: stow_start_cycle(),
                    through: end_cycle(),
                    how_many: check::Recorded::AtLeastOnce,
                    raised_by_tick: true,
                    why: "the limp pair no longer closing on its goals",
                },
            ],
            failures,
        );
        check_the_answer(run, failures);
        check_the_writes(run, engaged.map(|engaged| engaged.released), failures);
        // The pair really did let go: from the cycle the drain must have
        // finished by, neither antenna moves again for the rest of the run. A
        // de-torqued servo on this machine holds where it stands -- the
        // gearboxes do not back-drive -- so the evidence that the torque came
        // off is a joint that stops answering its goals while the goals keep
        // coming, and the head is moving through the whole window, so this is
        // not a claim about a machine that stopped being commanded.
        check::stands_still_rows(
            run,
            degraded_rows(),
            released_by_cycle(),
            end_cycle(),
            "let go of, with the head still under command",
            failures,
        );
        check_the_head_kept_its_presence(run, failures);
        check_the_release_says_what_it_could_not_find(run, failures);
        // The driver's gate raised nothing at all, exactly as in a healthy run.
        // The group-scoped de-torque is the session's own verified writes and
        // the orderly release at the end is the disarm sequence's, so nothing in
        // this run ever latches the gate or lets its dead-man expire -- an event
        // here would be a run where the answer went out as a whole-machine
        // torque-off instead of as the two writes it is.
        check::no_events(run, failures);
        check::signal_groups(run, failures);
    })
}

/// The answer: every response this run selected is the group-scoped de-torque,
/// and the pair it released is the pair.
///
/// The doctrine's one response scoped to a group. Nothing is stowed and nothing
/// is parked: an antenna pair going limp while the head keeps its presence is a
/// fault answered, so a `winddown_outcome` or a response of any other kind here
/// would be a session that ended over a condition it was supposed to survive.
///
/// And nothing else is narrated. Every kind the session can tell is accounted
/// for here, so a report this run has no business producing fails rather than
/// passing unseen.
fn check_the_answer(run: &Run, failures: &mut Vec<String>) {
    let mut answers = Vec::new();
    let mut releases = Vec::new();
    for report in &run.reports {
        match report.message.kind() {
            ReportKindWire::PHASE_CHANGED
            | ReportKindWire::SCRIPT_ACCEPTED
            | ReportKindWire::SCHEDULE_PUBLISHED
            | ReportKindWire::FAULT_RECORDED
            | ReportKindWire::SESSION_ENDED
            | ReportKindWire::TORQUE_OFF_CONFIRMED => {}
            ReportKindWire::RESPONSE_TAKEN => {
                answers.push((
                    cycle_within(report.message.time().as_nanos()),
                    report.message.a(),
                ));
            }
            ReportKindWire::DEGRADE_RELEASED => releases.push((
                cycle_within(report.message.time().as_nanos()),
                report.message.a(),
                report.message.b(),
            )),
            other => failures.push(format!(
                "the session narrated {other:?} at {}, and this run is a pair let go of by a \
                 session that carried on",
                report.message.time().as_nanos()
            )),
        }
    }
    let degrade = u32::from(ResponseKindWire::from(ResponseKind::DegradeAntennas).0);
    if answers.is_empty() {
        failures.push(
            "the session selected no response: a servo's own error byte is evidence of a \
             condition, and the condition has an answer"
                .to_string(),
        );
    }
    for (at, response) in &answers {
        if *response != degrade {
            failures.push(format!(
                "the session selected response {response} at cycle {at}, and an antenna in \
                 trouble is answered by letting the pair go ({degrade}): every other rung ends \
                 the session"
            ));
        }
    }
    let pair = u32::from(JointFlagsWire::from(degraded_rows()).0);
    for (at, response, rows) in &releases {
        if *response != degrade || *rows != pair {
            failures.push(format!(
                "the session released rows {rows} for response {response} at cycle {at}, and this \
                 maneuver is the antenna pair ({pair}) let go of by the group-scoped de-torque \
                 ({degrade})"
            ));
        }
    }
    // Two conditions, so two drains, and the run says which is which by when it
    // ran. The first is the servo's own byte, answered while the machine held
    // its working posture and inside the wakes one verified write apiece takes.
    // The second is the tick's own evidence once the fold is commanded: writing
    // torque off a row that is already limp is the honest answer to being told
    // again that it will not move, and it costs two more wakes.
    match (releases.first(), releases.get(1), releases.len()) {
        (Some((first, ..)), Some((second, ..)), 2) => {
            if *first < fault_cycle() || *first > released_by_cycle() {
                failures.push(format!(
                    "the pair was released at cycle {first}, outside the {}..{} one verified write \
                     per wake takes",
                    fault_cycle(),
                    released_by_cycle()
                ));
            }
            if *second < stow_start_cycle() {
                failures.push(format!(
                    "the pair was released again at cycle {second}, before the fold was commanded \
                     on {}: the second drain answers the tick's own evidence about joints that \
                     will not follow, and nothing asks the antennas to move before then",
                    stow_start_cycle()
                ));
            }
        }
        _ => failures.push(format!(
            "the session released the pair on cycles {:?}: this run has two conditions in it -- \
             the byte the servo holds, and the fold the limp pair cannot join -- and each is \
             answered by draining the group once",
            releases.iter().map(|(at, ..)| *at).collect::<Vec<_>>()
        )),
    }
}

/// The session did the de-torquing itself, one verified write at a time, and it
/// wrote to nothing but the antennas.
///
/// This is the assertion the doctrine's group scoping rests on. A verified
/// `TorqueEnable = 0` is the only thing that makes a row let go, and the run has
/// two sources of them: this maneuver, and the release sweep the disarm sequence
/// runs at the end. Everything before the session let go of the machine is the
/// maneuver's, so a write to a head row in that window is a response that took
/// the whole machine down while claiming to have taken a pair.
///
/// Both rows are asserted, because the response is the pair: a drain that
/// stopped after the servo that complained would leave the machine
/// half-presenting, which is exactly what scoping the response to the group is
/// for.
fn check_the_writes(run: &Run, released: Option<i64>, failures: &mut Vec<String>) {
    let Some(released) = released else {
        return;
    };
    let mut written = Vec::new();
    for datagram in &run.datagrams {
        let at = cycle_within(datagram.at_ns);
        if at >= released {
            break;
        }
        let txn = datagram.message.txn();
        if datagram.message.kind() != SessionCmdKindWire::AUX
            || txn.op() != AuxOpKindWire::WRITE_REG_VERIFIED
            || txn.reg() != RegIdWire::TORQUE_ENABLE
            || txn.value() != 0
        {
            continue;
        }
        let joint = row_of_id(txn.id()).and_then(|row| ROWS.get(row).copied());
        match joint {
            Some(joint) if flags::contains(degraded_rows(), joint) => written.push(joint),
            other => failures.push(format!(
                "the session took torque off {other:?} at cycle {at}, while the machine was still \
                 under command: the only de-torquing this run answers with is scoped to the \
                 antenna pair"
            )),
        }
    }
    for joint in flags::iter(degraded_rows()) {
        if !written.contains(&joint) {
            failures.push(format!(
                "the session never wrote {}'s torque off: the response is the pair, and one \
                 antenna still holding beside a dead one is a machine half-presenting",
                Name(joint)
            ));
        }
    }
}

/// The head kept its presence: it ran the rest of the schedule and reached the
/// fold, with two of its nine joints out of service.
///
/// The whole justification for scoping the response to the group. `arrived_at`
/// is not used, because it asks about the antennas too and the antennas are the
/// joints this run took away: what is asserted is the head pose the fold names
/// and every row that still had torque standing at its stow angle.
fn check_the_head_kept_its_presence(run: &Run, failures: &mut Vec<String>) {
    let cycle = disengage_cycle() - 1;
    let Some(sample) = check::sample_at_or(run, cycle, "folded", failures).map(present_rows) else {
        return;
    };
    let cfg = disarm_config();
    for (row, joint) in ROWS.into_iter().enumerate() {
        if flags::contains(degraded_rows(), joint) {
            continue;
        }
        let Some(wanted) = cfg.stow_targets.get(joint) else {
            failures.push(format!("{} has no stow angle", Name(joint)));
            continue;
        };
        let error = (sample[row] - wanted).abs();
        if error > cfg.tolerance {
            failures.push(format!(
                "at cycle {cycle} {} is {error} rad from its stow angle, and the head this \
                 session let go of had two antennas out of service and seven working joints",
                Name(joint)
            ));
        }
    }
    // And the antennas are not there, which is what makes the assertion above a
    // statement about a machine that carried on rather than about one nothing
    // happened to.
    for joint in flags::iter(degraded_rows()) {
        let (Some(row), Some(wanted)) = (row(joint), cfg.stow_targets.get(joint)) else {
            continue;
        };
        if wrap_to_pi(sample[row] - wanted).abs() <= cfg.tolerance {
            failures.push(format!(
                "at cycle {cycle} the released {} is folded after all: a limp antenna cannot \
                 reach a fold it was let go of before",
                Name(joint)
            ));
        }
    }
}

/// The release said what it could not find: the fold it measured is the head's,
/// and the antennas are reported as the distance they were left at.
///
/// The session ends at rest and says so, and the deviation it reports is outside
/// the tolerance -- because two joints are not at their stow angles and never
/// could be. A run reporting a machine fully folded would be one whose release
/// measured joints it had let go of the torque on hours before.
fn check_the_release_says_what_it_could_not_find(run: &Run, failures: &mut Vec<String>) {
    let ended: Vec<(u32, u32, f64)> = run
        .reports
        .iter()
        .filter(|report| report.message.kind() == ReportKindWire::SESSION_ENDED)
        .map(|report| {
            (
                report.message.a(),
                report.message.b(),
                report.message.detail(),
            )
        })
        .collect();
    let [(script_id, unmeasured, deviation)] = ended.as_slice() else {
        failures.push(format!(
            "the session ended {ended:?} times: this run runs one schedule out"
        ));
        return;
    };
    if *script_id != SCRIPT_ID {
        failures.push(format!(
            "the session ended script {script_id}, and this run sent {SCRIPT_ID}"
        ));
    }
    if *unmeasured != 0 {
        failures.push(format!(
            "the release could not read joints {unmeasured}: a de-torqued servo still answers a \
             read, so the limp pair is measured and found away from the fold rather than unread"
        ));
    }
    let tolerance = disarm_config().tolerance;
    if *deviation <= tolerance {
        failures.push(format!(
            "the release reported the machine {deviation} rad from the fold, inside the \
             {tolerance} it counts as folded: two of the nine joints were let go of before the \
             fold was commanded and cannot be at it"
        ));
    }
}

/// The scenario still describes the run it claims to: the byte is written after
/// the antennas have arrived, and the whole answer to it fits inside the step
/// they are holding.
///
/// Both are arithmetic over numbers this file does not own -- how long the
/// upright move is given, how long a lap of the driver's rotation takes, how long
/// the session's wake floor is -- and a move in any of them hollows the run out in
/// a way every assertion below would report as something else. A pair let go of
/// while it was still travelling would stall away from goals that keep moving,
/// and the tick would raise about the antennas inside the upright step: the
/// obstruction this run places after the fold would then be a different one, on a
/// cycle decided by where the rotation happened to be. So the ordering is
/// asserted rather than described.
fn check_the_answer_fits_the_posture(failures: &mut Vec<String>) {
    // The whole move rather than the configured duration: the antennas are the
    // group whose clock the floor lengthens, and they are the pair this guard is
    // about.
    let arrived = up_start_cycle() + up_clocks().cycles();
    if fault_cycle() < arrived {
        failures.push(format!(
            "the servo's byte is written on cycle {} and the antennas are still travelling until \
             {arrived}: a pair let go of mid-move stops closing on goals that keep moving, which \
             is a raise this run does not place",
            fault_cycle()
        ));
    }
    if released_by_cycle() >= stow_start_cycle() {
        failures.push(format!(
            "the pair is allowed until cycle {} to let go and the fold is commanded on {}: the \
             answer to the byte has to be finished before the run asks the limp pair to move",
            released_by_cycle(),
            stow_start_cycle()
        ));
    }
}
