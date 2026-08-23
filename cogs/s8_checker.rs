//! S8's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What S8 shares with a healthy run is asserted by
//! `scenario::check`; what is here is the chain a complaining servo sets off --
//! the condition read off the driver's rotation and classified as the head's,
//! the masked stow to park selected once, the jam mid-maneuver re-commanding
//! that stow on what is left of its one clock, the head reaching the fold, the
//! park, the release, and the script that finds a machine nothing will engage.

use std::process::ExitCode;

use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__driver__health_clk_rs::EventKind;
use brenn_reachy__motion__faults_clk_rs::{FaultKindWire, ResponseKindWire};
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKindWire};
use brenn_reachy__motion__timeline_clk_rs::WindDownOutcomeWire;
use motion_cogs::session_bus::disarm_config;
use reachy_motion::default_motion_config;
use reachy_motion::disarm::at_stow;
use reachy_motion::joints::{flags, vector_of};
use reachy_motion::tick::ResponseKind;
use scenario::check;
use scenario::read::Run;

use s8_scenario::{
    REFUSED_SCRIPT_ID, SCRIPT_ID, answered_by_cycle, end_cycle, fault_cycle, faulted_joint,
    jam_cycle, jam_release_cycle, jammed_rows, refused_script_cycle, script_sent_cycle,
};

fn main() -> ExitCode {
    check::main("s8_checker", |run, failures| {
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        check::scripts_sent(
            run,
            &[
                (SCRIPT_ID, script_sent_cycle()),
                (REFUSED_SCRIPT_ID, refused_script_cycle()),
            ],
            failures,
        );
        // First, because everything below is measured against the cycles these
        // land on. Five changes and no more: the survey, the script taken, the
        // arming, the machine carried down, and the park. A sixth would be a
        // session that did something with the script it was sent afterwards.
        let cycles = check::phases(
            run,
            &[
                (SessionPhaseWire::RESTING, SessionPhaseWire::STARTING),
                (SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
                (SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING),
                (SessionPhaseWire::WINDING_DOWN, SessionPhaseWire::ACTIVE),
                (SessionPhaseWire::PARKED, SessionPhaseWire::WINDING_DOWN),
            ],
            failures,
        );
        let taken = cycles.get(2).copied();
        let carried_down = cycles.get(3).copied();
        let parked = cycles.get(4).copied();
        check_when_it_was_answered(carried_down, failures);
        // Every goal of this run speaks for all nine rows, which is the strict
        // form: the maneuver is the masked stow to park, and nothing in this
        // build takes the failed servo out of service where the goal stream can
        // show it -- the session cannot see the tick's mask, so a stow is
        // carried to its own clock rather than to a head with nothing left to
        // drive it. The set is named at the call rather than left to the
        // shorthand, so the day the mask reaches the session this assertion is
        // where the run says which joint left.
        // TODO(session-mask-view)
        if let (Some(stream), Some(taken), Some(parked)) = (
            check::goal_stream_without(run, JointFlags::NONE, failures),
            taken,
            parked,
        ) {
            check::stream_starts_with_session(&stream, taken, failures);
            check::stream_stops_with_release(&stream, parked, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);

        // The two conditions of this run, and nothing else: the servo's own
        // account of itself, once -- the byte latches in the servo and the
        // rotation carries it on every lap, so a session that recorded what it
        // read would fill the timeline with one standing condition at the poll
        // rate -- and the fold that stopped arriving while the cranks were held,
        // which is the tick's and is asserted by where it falls: inside the jam,
        // plus the detector's window, because the window is what has to run out
        // before a stalled joint is a raise.
        check::faults_recorded(
            run,
            &[
                check::Expected {
                    kind: FaultKindWire::HEAD_SERVO_FAULT,
                    rows: flags::bit(faulted_joint()),
                    from: fault_cycle(),
                    through: answered_by_cycle(),
                    how_many: check::Recorded::Times(1),
                    raised_by_tick: false,
                    why: "the servo complaining about itself",
                },
                check::Expected {
                    kind: FaultKindWire::HEAD_OBSTRUCTED,
                    rows: jammed_rows(),
                    from: jam_cycle(),
                    through: jam_release_cycle()
                        + i64::from(default_motion_config().tracking.ticks),
                    how_many: check::Recorded::AtLeastOnce,
                    raised_by_tick: true,
                    why: "the cranks being held",
                },
            ],
            failures,
        );
        check_the_answer(run, failures);
        // More than one stow, which is the whole of what the jam is here for: a
        // condition arriving on the way down re-commands the fold rather than
        // opening a second maneuver, and `check::stows` is what says every one
        // of them ends at the same instant.
        let stows = check::stows(run, carried_down, failures);
        if stows < 2 {
            failures.push(format!(
                "the session published {stows} stows: the cranks are jammed while the head is \
                 being carried down, so the tick raises about a fold that stopped arriving and \
                 the maneuver is asked for again"
            ));
        }
        check_the_jam_lands_inside_the_maneuver(carried_down, parked, failures);
        check_the_fold(run, parked, failures);
        // The script that arrived after all of it, refused as parked: a sender
        // told the machine was busy would keep asking, and parked is the one
        // refusal that says nothing will take a script until an operator has
        // been.
        check::refusals(
            run,
            &[(
                REFUSED_SCRIPT_ID,
                RefusalReasonWire::PARKED,
                refused_script_cycle(),
            )],
            failures,
        );
        // The machine really did go limp, and the only edge in the run is the
        // read-back that says so: a hold timeout here would be a commander that
        // went quiet while the machine was energised, which is what the
        // keep-alive rule and the goal stream exist to prevent.
        check::confirmed_off(run, check::first_release(run, failures), failures);
        check::only_kinds(run, &[EventKind::TorqueOffConfirmed], failures);
        check::signal_groups(run, failures);
    })
}

/// The condition was answered inside a lap of the driver's rotating read.
///
/// The one instant in this run that is not arithmetic: which cycle the rotation
/// reaches the faulted row on depends on where in its lap it was when the byte
/// was written, so what the scenario says is the bound -- the row is read within
/// one lap, and the session answers on the wake that reading causes. A run that
/// answered later would be one whose rotation had stopped walking the bus.
fn check_when_it_was_answered(carried_down: Option<i64>, failures: &mut Vec<String>) {
    let Some(carried_down) = carried_down else {
        return;
    };
    if carried_down < fault_cycle() {
        failures.push(format!(
            "the machine was carried down on cycle {carried_down}, before the servo was made to \
             complain on {}",
            fault_cycle()
        ));
    }
    if carried_down > answered_by_cycle() {
        failures.push(format!(
            "the machine was carried down on cycle {carried_down}, and the rotating read reaches \
             every row of the bus by {}: a condition answered later is one the rotation did not \
             carry",
            answered_by_cycle()
        ));
    }
}

/// The answer: one response, selected once, and one maneuver that ended
/// measured at the fold with the machine left for an operator.
///
/// One response is the doctrine's. A head servo that has stopped being
/// trustworthy is carried down by the servos that still work and then parked;
/// the jam that arrives on the way down re-ranks that maneuver rather than
/// selecting a second one, which is why the stalled joint's own answer -- the
/// stow to *rest* -- never appears.
///
/// And nothing else is narrated. Every kind the session can tell is accounted
/// for here, so a report this run has no business producing fails rather than
/// passing unseen.
fn check_the_answer(run: &Run, failures: &mut Vec<String>) {
    let mut answers = Vec::new();
    let mut outcomes = Vec::new();
    for report in &run.reports {
        match report.message.kind() {
            ReportKindWire::PHASE_CHANGED
            | ReportKindWire::SCRIPT_ACCEPTED
            | ReportKindWire::SCRIPT_REFUSED
            | ReportKindWire::SCHEDULE_PUBLISHED
            | ReportKindWire::FAULT_RECORDED
            | ReportKindWire::TORQUE_OFF_CONFIRMED => {}
            ReportKindWire::RESPONSE_TAKEN => answers.push(report.message.a()),
            ReportKindWire::WINDDOWN_OUTCOME => {
                outcomes.push((report.message.a(), report.message.b()));
            }
            other => failures.push(format!(
                "the session narrated {other:?} at {}, and this run is a complaining servo \
                 answered under control",
                report.message.time().as_nanos()
            )),
        }
    }
    let masked = u32::from(ResponseKindWire::from(ResponseKind::MaskedSlowStowToPark).0);
    if answers != vec![masked] {
        failures.push(format!(
            "the session selected {answers:?}, and a head servo in trouble is answered once with \
             the masked stow to park ({masked}): a second answer would be a second clock over one \
             machine"
        ));
    }
    let completed = u32::from(WindDownOutcomeWire::COMPLETED.0);
    if outcomes != vec![(completed, 1)] {
        failures.push(format!(
            "the maneuver ended as {outcomes:?}, and this run's hand comes off in time for the \
             head to be measured at the fold, with the park the first condition decided"
        ));
    }
}

/// The head really was carried down: the machine the session let go of is
/// standing at the fold, by the maneuver's own measure, and it stays there.
///
/// The maneuver's tolerance rather than a posture arrival, because what ended it
/// is `at_stow` over the driver's sample -- asserting a tighter number here
/// would be this checker's opinion of a fold rather than the one the machine was
/// measured against.
fn check_the_fold(run: &Run, parked: Option<i64>, failures: &mut Vec<String>) {
    let Some(parked) = parked else {
        return;
    };
    let Some(sample) = check::sample_at_or(run, parked, "measured at the fold", failures) else {
        return;
    };
    match sample.present().validate() {
        Ok(present) if at_stow(disarm_config(), &vector_of(present)) => {}
        Ok(_) => failures.push(format!(
            "the machine the session let go of on cycle {parked} is not at the fold, and the \
             maneuver it ended reported the head measured there"
        )),
        Err(complaint) => failures.push(format!(
            "the sample at cycle {parked} holds no reading: {complaint}"
        )),
    }
    // And it is left there. Nothing is streamed to a parked machine and its
    // torque is off, so a machine that moved after this moved with nobody
    // asking it to.
    check::stands_still(
        run,
        parked + 1,
        end_cycle(),
        "folded, uncommanded and de-torqued",
        failures,
    );
}

/// The jam really did arrive while the head was being carried down, and came
/// off before the machine was let go of.
///
/// The scenario's own arithmetic, checked against the run rather than trusted:
/// the cycle the maneuver opened on is a fact about the rotation, so a jam
/// placed from the outer bound could in principle have landed outside the
/// maneuver -- and if it had, every assertion about a re-commanded stow above
/// would be about a run where nothing was re-commanded.
fn check_the_jam_lands_inside_the_maneuver(
    carried_down: Option<i64>,
    parked: Option<i64>,
    failures: &mut Vec<String>,
) {
    let (Some(carried_down), Some(parked)) = (carried_down, parked) else {
        return;
    };
    if jam_cycle() <= carried_down {
        failures.push(format!(
            "the cranks are jammed on cycle {} and the head was being carried down from \
             {carried_down}: a jam before the maneuver is a different run",
            jam_cycle()
        ));
    }
    if jam_release_cycle() >= parked {
        failures.push(format!(
            "the jam comes off on cycle {} and the session let go of the machine on {parked}: the \
             fold has to be reachable after the hand comes off",
            jam_release_cycle()
        ));
    }
}
