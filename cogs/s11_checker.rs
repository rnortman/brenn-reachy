//! S11's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What S11 shares with a healthy run is asserted by
//! `scenario::check` -- and *most of it holds*, which is the point of the run:
//! the session that answered a fault here is the session that ran its schedule
//! out and let go of the machine at rest, with the same five phase changes and
//! the same two schedules S1 has.
//!
//! What is here is the group-scoped answer, twice. The condition read off the
//! driver's rotation and classified as the antennas'; the two verified
//! torque-off writes the session issues itself; the report that says the pair
//! let go; the tick taking the limp pair out of service when the fold is
//! commanded and carrying the move on with what remains; the
//! `antenna_obstructed` the detector raises about a pair that cannot follow that
//! fold, and the second drain that answers it over rows already limp; the head
//! reaching that fold; and the release reporting the antennas it could not find
//! there.
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
use reachy_motion::joints::{Name, ROW_COUNT, ROWS, flags, row};
use reachy_motion::tick::ResponseKind;
use scenario::check;
use scenario::check::present_rows;
use scenario::cycle_within;
use scenario::read::Run;
use scenario::{stow_clocks, up_clocks};

use s11_scenario::{
    SCRIPT_ID, STOW_CYCLES, answered_by_cycle, degraded_rows, disengage_cycle, end_cycle,
    fault_cycle, faulted_joint, released_by_cycle, script_sent_cycle, stow_start_cycle, up_cycles,
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
        check::room("upright", up_cycles(), &up_clocks(), failures);
        check::room("stow", STOW_CYCLES, &stow_clocks(), failures);
        check_the_answer_fits_the_posture(run, failures);

        // The two conditions of this run, and nothing else. The servo's own
        // account of itself, once -- the byte latches in the servo and the
        // rotation carries it on every lap, so a session that recorded what it
        // read would fill the timeline with one standing condition at the poll
        // rate. And the pair that cannot follow the fold once nothing holds it,
        // which the tick raises about while the fold is being commanded: also
        // once, because the answer takes the pair out of the tick's own service
        // and a masked row is stepped and never judged.
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
                    through: disengage_cycle(),
                    how_many: check::Recorded::Times(1),
                    raised_by_tick: true,
                    why: "the limp pair not following the fold",
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
/// and the pair it released is the pair, twice.
///
/// The doctrine's one response scoped to a group, and the one rung a machine can
/// be answered with twice: nothing is stowed and nothing is parked, so a
/// `winddown_outcome` or a response of any other kind here would be a session
/// that ended over a condition it was supposed to survive.
///
/// And nothing else is narrated. Every kind the session can tell is accounted
/// for here, so a report this run has no business producing fails rather than
/// passing unseen.
fn check_the_answer(run: &Run, failures: &mut Vec<String>) {
    let told = check::narrated(
        run,
        &[
            ReportKindWire::PHASE_CHANGED,
            ReportKindWire::SCRIPT_ACCEPTED,
            ReportKindWire::SCHEDULE_PUBLISHED,
            ReportKindWire::FAULT_RECORDED,
            ReportKindWire::SESSION_ENDED,
            ReportKindWire::TORQUE_OFF_CONFIRMED,
            ReportKindWire::RESPONSE_TAKEN,
            ReportKindWire::DEGRADE_RELEASED,
        ],
        "a pair let go of by a session that carried on",
        failures,
    );
    let degrade = u32::from(ResponseKindWire::from(ResponseKind::DegradeAntennas).0);
    if told.answers.is_empty() {
        failures.push(
            "the session selected no response: a servo's own error byte is evidence of a \
             condition, and the condition has an answer"
                .to_string(),
        );
    }
    for answer in &told.answers {
        if answer.response != degrade {
            failures.push(format!(
                "the session selected response {} at cycle {}, and an antenna in trouble is \
                 answered by letting the pair go ({degrade}): every other rung ends the session",
                answer.response, answer.at
            ));
        }
    }
    let pair = u32::from(JointFlagsWire::from(degraded_rows()).0);
    for release in &told.releases {
        if release.response != degrade || release.rows != pair {
            failures.push(format!(
                "the session released rows {} for response {} at cycle {}, and this maneuver is \
                 the antenna pair ({pair}) let go of by the group-scoped de-torque ({degrade})",
                release.rows, release.response, release.at
            ));
        }
    }
    // Two conditions, so two drains: the servo's own byte, answered while the
    // machine held its working posture and inside the wakes one verified write
    // apiece takes; and the pair failing to follow the fold, answered inside the
    // step that commands it. The second drain writes torque off rows that are
    // already limp, which is the doctrine's rule that nothing gates
    // de-torquing -- an answer scoped to a group is issued whether or not the
    // group is still holding.
    match told.releases.as_slice() {
        [first, second] => {
            if first.at < fault_cycle() || first.at > released_by_cycle() {
                failures.push(format!(
                    "the pair was released at cycle {}, outside the {}..{} one verified write per \
                     wake takes",
                    first.at,
                    fault_cycle(),
                    released_by_cycle()
                ));
            }
            if second.at < stow_start_cycle() || second.at > disengage_cycle() {
                failures.push(format!(
                    "the pair was released a second time at cycle {}, outside the {}..{} the fold \
                     the limp pair cannot follow is commanded in",
                    second.at,
                    stow_start_cycle(),
                    disengage_cycle()
                ));
            }
        }
        _ => failures.push(format!(
            "the session released the pair on cycles {:?}: this run has two conditions in it -- \
             the byte the servo holds, and the fold the limp pair cannot follow -- each answered \
             by draining the group",
            told.releases
                .iter()
                .map(|release| release.at)
                .collect::<Vec<_>>()
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

/// The scenario still describes the run it claims to: the pair had finished
/// travelling before the byte was written.
///
/// Measured in the samples rather than derived from the scenario's own
/// arithmetic. A pair let go of while it was still travelling would stall away
/// from goals that keep moving, and the tick would raise about the antennas
/// inside the upright step: the obstruction this run places after the fold
/// would then be a different one, on a cycle decided by where the rotation
/// happened to be. What that failure looks like in the log is a pair whose last
/// moving reading is *after* the byte -- they stopped because they were let go
/// of, not because they arrived -- so that is the reading this asserts, and it
/// holds however the travel, the rotation's lap or the wake floor move.
///
/// The other half of the ordering -- the answer finished before the fold is
/// commanded -- is structural rather than asserted: the upright step is defined
/// as the drain plus a settle (`s11_scenario::up_cycles`), so the fold cannot
/// be commanded before the drain's own allowance. What makes it observable is
/// the release window in [`check_the_answer`], which dates the first drain
/// inside that allowance from the log.
fn check_the_answer_fits_the_posture(run: &Run, failures: &mut Vec<String>) {
    let mut moved: Option<i64> = None;
    let mut stood: Option<[f64; ROW_COUNT]> = None;
    for cycle in up_start_cycle()..stow_start_cycle() {
        let Some(reads) = check::sample_at(run, cycle).map(present_rows) else {
            continue;
        };
        if let Some(before) = stood {
            for joint in flags::iter(degraded_rows()) {
                if let Some(row) = row(joint)
                    && reads[row] != before[row]
                {
                    moved = Some(cycle);
                }
            }
        }
        stood = Some(reads);
    }
    match moved {
        None => failures.push(format!(
            "neither antenna moved at all between cycles {} and {}, where this run has them \
             travelling to the working posture before anything is wrong with them",
            up_start_cycle(),
            stow_start_cycle()
        )),
        Some(moved) if moved > fault_cycle() => failures.push(format!(
            "an antenna was still moving on cycle {moved} and the servo's byte is written on {}: \
             a pair whose last movement is after the byte was let go of mid-move, which stalls it \
             away from goals that keep moving -- a raise inside the upright step, and a different \
             run from the one this scenario places",
            fault_cycle()
        )),
        Some(_) => {}
    }
}
