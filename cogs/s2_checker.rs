//! S2's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What S2 shares with a healthy run is asserted by
//! `scenario::check`; what is here is the obstruction itself -- that it was
//! reported, that it was reported as an event and not as a condition, that the
//! machine held rather than latched, and that the session answered it by
//! carrying the head down and letting go of it at rest -- and then that the
//! machine it let go of takes another script and runs it, which is the half of
//! the doctrine's park/rest split that a refusal cannot show.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__driver__health_clk_rs::EventKind;
use brenn_reachy__motion__faults_clk_rs::{FaultKindWire, ResponseKindWire};
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use brenn_reachy__motion__timeline_clk_rs::WindDownOutcomeWire;
use reachy_motion::default_motion_config;
use reachy_motion::joints::{Name, flags, row};
use reachy_motion::postures::{neutral_targets, stow_pose_targets};
use reachy_motion::tick::ResponseKind;
use scenario::check;
use scenario::check::{goal_at, present_rows, sample_at};
use scenario::cycle_of;
use scenario::read::Run;
use scenario::{stow_clocks, up_clocks};

use s2_scenario::{
    SCRIPT_ID, SECOND_SCRIPT_ID, SECOND_STOW_CYCLES, SECOND_UP_CYCLES, end_cycle, jammed_rows,
    obstruct_cycle, release_cycle, script_sent_cycle, second_disengage_cycle, second_script_cycle,
    second_stow_cycle,
};

fn main() -> ExitCode {
    check::main("s2_checker", |run, failures| {
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        check::scripts_sent(
            run,
            &[
                (SCRIPT_ID, script_sent_cycle()),
                (SECOND_SCRIPT_ID, second_script_cycle()),
            ],
            failures,
        );
        // The phases a jam takes the machine through: armed, under command,
        // carried down, and at rest -- and then, on a machine nothing latched,
        // the whole ordinary life of a second session. The strict form rather
        // than `check::engagement`, whose five changes are the ordinary life of
        // one session that ran its schedule out.
        let cycles = check::phases(
            run,
            &[
                (SessionPhaseWire::RESTING, SessionPhaseWire::STARTING),
                (SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
                (SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING),
                (SessionPhaseWire::WINDING_DOWN, SessionPhaseWire::ACTIVE),
                (SessionPhaseWire::RESTING, SessionPhaseWire::WINDING_DOWN),
                (SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
                (SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING),
                (SessionPhaseWire::STOPPING, SessionPhaseWire::ACTIVE),
                (SessionPhaseWire::RESTING, SessionPhaseWire::STOPPING),
            ],
            failures,
        );
        // Every cycle this checker measures against, named by the change it is,
        // bound here rather than indexed below: the list above has nine rows and
        // two of every transition in it, so an ordinal read at the point of use
        // is an assertion about whichever change a later edit moved into that
        // slot.
        let taken = cycles.get(2).copied();
        let carried_down = cycles.get(3).copied();
        let let_go = cycles.get(4).copied();
        let second_taken = cycles.get(6).copied();
        let second_let_go = cycles.get(7).copied();
        // Two stretches of stream, because nothing is commanded between a
        // session that ended and the next arming taking hold: the machine is
        // de-torqued for that whole interval, and what holds the driver's
        // dead-man off once it is energised again is the keep-alive.
        let streams = check::goal_streams_exactly(run, JointFlags::NONE, 2, failures);
        if let [first, second] = streams.as_slice() {
            if let (Some(taken), Some(let_go)) = (taken, let_go) {
                check::stream_starts_with_session(first, taken, failures);
                check::stream_stops_with_release(first, let_go, failures);
            }
            if let (Some(taken), Some(let_go)) = (second_taken, second_let_go) {
                check::stream_starts_with_session(second, taken, failures);
                check::stream_stops_with_release(second, let_go, failures);
            }
        }
        // The second session let go of its schedule promptly; the release it
        // then ran is the orderly one, settle and all, and the phase it ends in
        // is what says so.
        check::ended_promptly(second_let_go, second_disengage_cycle(), failures);
        check_second_engagement(run, failures);
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);

        check_jam_held(run, failures);
        let first_raise = check_reports(run, failures);
        check_narration(run, failures);
        check_hold(run, first_raise, carried_down, failures);
        // The wind-down's own schedules, up to the one it published as it let
        // go: the second session's are the second session's.
        check::stows_until(run, carried_down, let_go, failures);
        // The machine really did go limp at the end, and the only edge in the
        // run is the read-back that says so: everything else the gate raises is
        // about a commander that went quiet while the machine was energised,
        // which is what the keep-alive and the goal stream exist to prevent.
        check::confirmed_off(run, check::first_release(run, failures), failures);
        check::only_kinds(run, &[EventKind::TorqueOffConfirmed], failures);
        check::signal_groups(run, failures);
    })
}

/// The second session's schedule reached the machine and moved it: it stands up
/// where the second script asks it to and is back at the fold by the time that
/// script runs out.
///
/// The phases and the goal stream say the session ran; they do not say the
/// schedule it published was ever answered. A second engagement whose epoch was
/// never bumped, or whose schedule the mover dropped, streams the targets the
/// machine is already standing on -- which is indistinguishable from a correct
/// run of a script that asked for the posture it was in. So this half of the run
/// asks for a posture the machine is not in, and this is where that is checked.
fn check_second_engagement(run: &Run, failures: &mut Vec<String>) {
    check::room("second upright", SECOND_UP_CYCLES, &up_clocks(), failures);
    check::room("second stow", SECOND_STOW_CYCLES, &stow_clocks(), failures);
    check::arrived_at(
        run,
        "upright on the second script",
        second_stow_cycle() - 1,
        &neutral_targets(),
        failures,
    );
    check::arrived_at(
        run,
        "stowed again",
        second_disengage_cycle() - 1,
        &stow_pose_targets(),
        failures,
    );
}

/// The scenario's hand really was on the machine: the jammed cranks do not move
/// at all for as long as the jam lasts.
///
/// Asserted because everything else in this file is about what the loop did
/// about a stalled joint, and a run where nothing stalled would report none of
/// it for the wrong reason.
///
/// The window ends at the release rather than including it, because an
/// injection takes effect on the cycle it names: the driver drains what
/// arrived and then advances the plant, so the sample for the cycle the jam was
/// published on already shows the rows standing still, and the sample for the
/// cycle the release was published on already shows them moving again.
fn check_jam_held(run: &Run, failures: &mut Vec<String>) {
    let from = obstruct_cycle();
    let Some(held) = sample_at(run, from).map(present_rows) else {
        failures.push(format!(
            "no sample for cycle {from}, where the cranks are jammed"
        ));
        return;
    };
    for cycle in from..release_cycle() {
        let Some(sample) = sample_at(run, cycle).map(present_rows) else {
            failures.push(format!("no sample for cycle {cycle}, inside the jam"));
            return;
        };
        for joint in flags::iter(jammed_rows()) {
            let Some(row) = row(joint) else {
                failures.push(format!("{} sits on no bus row", Name(joint)));
                continue;
            };
            if sample[row] != held[row] {
                failures.push(format!(
                    "at cycle {cycle} the jammed {} reads {}, having stood at {} when the jam \
                     settled: the plant let a jammed row move",
                    Name(joint),
                    sample[row],
                    held[row]
                ));
                return;
            }
        }
    }
}

/// The reports: a jammed head is reported, it is reported as that fault and no
/// other, every report falls inside the jam, and consecutive reports are a
/// detector window apart.
///
/// The spacing is the point. The tick re-examines the joint every cycle and the
/// condition stands for as long as the hand does, so a loop that reported what
/// it saw would fill the log at the control rate. What it reports instead is
/// each time the detector's window runs out on a fresh run of ticks, which is
/// an event -- and the floor on the gap between two of them is the length of
/// that window.
///
/// The cycle of the first report, for the hold assertion to measure from.
fn check_reports(run: &Run, failures: &mut Vec<String>) -> Option<i64> {
    let window = i64::from(default_motion_config().tracking.ticks);
    let mut raises = Vec::new();
    for fault in &run.faults {
        let at = fault.message.time().as_nanos();
        let cycle = match cycle_of(at) {
            Ok(cycle) => cycle,
            Err(complaint) => {
                failures.push(format!("a report is not on the grid: {complaint}"));
                continue;
            }
        };
        if fault.message.kind() != FaultKindWire::HEAD_OBSTRUCTED {
            failures.push(format!(
                "the decision tick reported {:?} at cycle {cycle}, and the only thing wrong with \
                 this machine is a jammed crank",
                fault.message.kind()
            ));
            continue;
        }
        match check::joint_of(fault.message.joint()) {
            None => failures.push(format!(
                "the report at cycle {cycle} names no crank, and a jam is about a servo"
            )),
            Some(joint) if !flags::contains(jammed_rows(), joint) => failures.push(format!(
                "the report at cycle {cycle} names {}, and the hand in this scenario is on \
                 {}",
                Name(joint),
                flags::Names(jammed_rows())
            )),
            Some(_) => {}
        }
        // The evidence a jam is classified on is how far the crank stood from
        // the goal it stopped closing on, and the detector does not look at a
        // joint nearer than its threshold at all. A report carrying less than
        // that is one raised on evidence the detector cannot have had.
        let error = fault.message.detail();
        let threshold = default_motion_config().tracking.threshold_rad;
        if error < threshold {
            failures.push(format!(
                "the report at cycle {cycle} puts the crank {error} rad from its goal, inside the \
                 {threshold} rad the detector screens out"
            ));
        }
        raises.push(cycle);
    }

    let first = raises.first().copied();
    if raises.len() < 2 {
        failures.push(format!(
            "the decision tick reported a jammed crank {} times over a jam that lasted {} cycles: \
             the spacing between reports is what this scenario is about, and one report cannot \
             show a spacing",
            raises.len(),
            release_cycle() - obstruct_cycle()
        ));
    }
    for pair in raises.windows(2) {
        let gap = pair[1] - pair[0];
        if gap < window {
            failures.push(format!(
                "two reports at cycles {} and {} are {gap} cycles apart, inside the detector's \
                 {window}-cycle window: a standing condition is being reported at the poll rate",
                pair[0], pair[1]
            ));
        }
    }
    let (jammed_from, jammed_to) = (obstruct_cycle(), release_cycle());
    for cycle in &raises {
        if *cycle < jammed_from || *cycle > jammed_to + window {
            failures.push(format!(
                "a jam was reported at cycle {cycle}, outside the {jammed_from}..{jammed_to} the \
                 cranks were held over"
            ));
        }
    }
    first
}

/// The session heard about the jam, said so, and answered it.
///
/// Every raise the tick published comes back out of the session as a recorded
/// fault naming the same condition -- which is the only assertion in the suite
/// that the session is wired into the box at all: that it executes, that
/// `TickFaults` reaches it, and that what it narrates reaches the log.
///
/// One report per raise and no more: an obstruction is one condition of the
/// machine however many cycles it lasts, and the session records what the tick
/// raised rather than what it saw.
///
/// And one answer for all of them. The response the doctrine gives a grabbed
/// head is the stow to rest, selected once -- a condition arriving while the
/// maneuver runs re-ranks it rather than starting a second one -- and the
/// maneuver's own record says the head was measured at the fold, which is the
/// end-to-end statement that the machine really was carried down.
///
/// The second session's own story -- the script it took, the schedules it
/// published, the measurement its orderly release wrote -- is asserted where the
/// phases are, and passed over here: nothing about it is a condition of the
/// machine, and this run has exactly one of those.
fn check_narration(run: &Run, failures: &mut Vec<String>) {
    let obstruction = u32::from(FaultKindWire::HEAD_OBSTRUCTED.0);
    let mut recorded = 0;
    let mut answers = Vec::new();
    let mut outcomes = Vec::new();
    for report in &run.reports {
        let kind = report.message.kind();
        // The engagement's own story -- the phases, the script taken, the
        // schedules published -- is asserted where it belongs: the phases in
        // `main` and the schedules in `check::stows`.
        if matches!(
            kind,
            ReportKindWire::PHASE_CHANGED
                | ReportKindWire::SCRIPT_ACCEPTED
                | ReportKindWire::SCHEDULE_PUBLISHED
                | ReportKindWire::TORQUE_OFF_CONFIRMED
                | ReportKindWire::SESSION_ENDED
        ) {
            continue;
        }
        match kind {
            ReportKindWire::FAULT_RECORDED => {
                if report.message.a() != obstruction {
                    failures.push(format!(
                        "the session recorded fault {} where the tick raised {obstruction}: the \
                         narration and the raise are about different conditions",
                        report.message.a()
                    ));
                }
                recorded += 1;
            }
            ReportKindWire::RESPONSE_TAKEN => answers.push(report.message.a()),
            ReportKindWire::WINDDOWN_OUTCOME => {
                outcomes.push((report.message.a(), report.message.b()));
            }
            other => failures.push(format!(
                "the session narrated {other:?}, and the only thing that happened in this \
                 scenario is a jammed crank the tick raised"
            )),
        }
    }
    if recorded != run.faults.len() {
        failures.push(format!(
            "the tick raised {} times and the session narrated {recorded} of them: the session is \
             the only reader of that channel, and a raise it never recorded is a session that is \
             not hearing the tick",
            run.faults.len()
        ));
    }
    let stow_to_rest = u32::from(ResponseKindWire::from(ResponseKind::SlowStowToRest).0);
    if answers != vec![stow_to_rest] {
        failures.push(format!(
            "the session selected {answers:?}, and a grabbed head is answered once with the stow \
             to rest ({stow_to_rest}): a second answer would be a second clock over one machine"
        ));
    }
    let completed = u32::from(WindDownOutcomeWire::COMPLETED.0);
    if outcomes != vec![(completed, 0)] {
        failures.push(format!(
            "the maneuver ended as {outcomes:?}, and this scenario's hand comes off in time for \
             the head to be measured at the fold and the machine left at rest"
        ));
    }
}

/// The machine holds under command: from the raise until the session answers it,
/// the goal stream carries on and it carries on saying the same thing.
///
/// A jammed head does not latch, so the tick abandons the move and holds. What
/// holding means on the wire is a fresh datagram every cycle carrying the
/// setpoint the machine is already on -- the goal stream is the machine's
/// liveness, not its news, and a loop that fell silent because it had nothing
/// new to say would be de-torqued for it.
///
/// The window closes where the session's stow reaches the tick, which is the
/// wake it entered the maneuver on: from there the machine is under command to
/// fold, and a stream that still said the same thing would be a stow nobody
/// carried out.
fn check_hold(
    run: &Run,
    first_raise: Option<i64>,
    carried_down: Option<i64>,
    failures: &mut Vec<String>,
) {
    let (Some(from), Some(to)) = (first_raise, carried_down) else {
        return;
    };
    let Some(held) = goal_at(run, from) else {
        failures.push(format!(
            "no goal for cycle {from}, where the machine dropped its move and held"
        ));
        return;
    };
    for cycle in from..to {
        let Some(goal) = goal_at(run, cycle) else {
            failures.push(format!(
                "no goal for cycle {cycle}: the stream stopped while the machine was holding"
            ));
            return;
        };
        if goal != held {
            failures.push(format!(
                "the goal at cycle {cycle} asks for something other than what the machine held \
                 from cycle {from}: a dropped move is a hold, not a slower move"
            ));
            return;
        }
    }
}
