//! S8's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What S8 shares with a healthy run is asserted by
//! `scenario::check`; what is here is the chain a complaining servo sets off --
//! the condition read off the driver's rotation and classified as the head's,
//! the masked stow to park selected once, the jam mid-maneuver that held the
//! cranks for less than the armed tracking detector's own raise latency and so
//! was answered by nothing, the head reaching the fold
//! on the maneuver's own clock, the park, the release, and the script that finds
//! a machine nothing will engage.

use std::process::ExitCode;

use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__driver__health_clk_rs::EventKind;
use brenn_reachy__motion__faults_clk_rs::{FaultKindWire, ResponseKindWire};
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKindWire};
use brenn_reachy__motion__timeline_clk_rs::WindDownOutcomeWire;
use reachy_motion::default_motion_config;
use reachy_motion::joints::{Name, flags, row};
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

        // The one condition of this run, and nothing else: the servo's own
        // account of itself, once -- the byte latches in the servo and the
        // rotation carries it on every lap, so a session that recorded what it
        // read would fill the timeline with one standing condition at the poll
        // rate.
        check::faults_recorded(
            run,
            &[check::Expected {
                kind: FaultKindWire::HEAD_SERVO_FAULT,
                rows: flags::bit(faulted_joint()),
                from: fault_cycle(),
                through: answered_by_cycle(),
                how_many: check::Recorded::Times(1),
                raised_by_tick: false,
                why: "the servo complaining about itself",
            }],
            failures,
        );
        // Nothing was raised about the cranks the hand held, and nothing else
        // was raised either.
        //
        // The masked stow runs on the one clock it was opened with, and the jam
        // that arrives while it is running stalls the head against a fold it is
        // still being commanded toward. The hand is on the cranks for the
        // detector's crossing distance and no longer -- the fewest periods a
        // generator at the profile could open a run in, and the fold's lead-in
        // is nowhere near the profile -- so the brush is below the raise
        // latency and no run opens: the maneuver is never defeated, and the
        // machine reaches the fold on the clock the first condition bought it.
        // A hand held past that latency raises inside this window, which is
        // S14's run.
        check::no_faults(run, failures);
        check_the_answer(run, failures);
        check::stows(run, carried_down, failures);
        check_the_jam_lands_inside_the_maneuver(carried_down, parked, failures);
        // The scenario's hand really was on the machine: the jammed cranks do
        // not move at all for as long as the jam lasts. The premise of
        // everything above -- what this run asserts about the jam is that
        // nothing was raised about it, which a run where nothing stalled would
        // satisfy for the wrong reason. The window ends before the release: an
        // injection takes effect on the cycle it names, so the sample for the
        // release cycle already shows the rows moving again.
        check::stands_still_rows(
            run,
            jammed_rows(),
            jam_cycle(),
            jam_release_cycle() - 1,
            "held by a hand while the maneuver carried the head down",
            failures,
        );
        check_the_jam_was_a_lag(run, failures);
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
/// the jam that arrives on the way down is answered by nothing, so the stalled
/// joint's own answer -- the stow to *rest* -- never appears. It would not
/// appear with the detector armed either: a condition arriving mid-maneuver
/// re-ranks it rather than opening a second one.
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
            ReportKindWire::SCRIPT_REFUSED,
            ReportKindWire::SCHEDULE_PUBLISHED,
            ReportKindWire::FAULT_RECORDED,
            ReportKindWire::TORQUE_OFF_CONFIRMED,
            ReportKindWire::RESPONSE_TAKEN,
            ReportKindWire::WINDDOWN_OUTCOME,
        ],
        "a complaining servo answered under control",
        failures,
    );
    let masked = u32::from(ResponseKindWire::from(ResponseKind::MaskedSlowStowToPark).0);
    if told.responses() != vec![masked] {
        failures.push(format!(
            "the session selected {:?}, and a head servo in trouble is answered once with the \
             masked stow to park ({masked}): a second answer would be a second clock over one \
             machine",
            told.responses()
        ));
    }
    let completed = u32::from(WindDownOutcomeWire::COMPLETED.0);
    if told.endings() != vec![(completed, 1)] {
        failures.push(format!(
            "the maneuver ended as {:?}, and this run's hand comes off in time for the head to \
             be measured at the fold, with the park the first condition decided",
            told.endings()
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
    if check::folded_at(run, parked, "measured at the fold", failures) == Some(false) {
        failures.push(format!(
            "the machine the session let go of on cycle {parked} is not at the fold, and the \
             maneuver it ended reported the head measured there"
        ));
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
/// maneuver -- and if it had, the absent raise above would be absent for the
/// wrong reason.
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

/// The jam is a lag in the record: some jammed crank ends it sitting further
/// from its goal than the detector's own threshold.
///
/// This is what makes the absent raise a statement: the hand really did stop
/// the machine closing on the fold it was being commanded to, so the run is
/// about a stall and not about a machine nothing happened to.
///
/// A lag against the goal, which is not the figure the detector screens on --
/// what it measures is the distance to the joint's own generator, and under the
/// fold's lead-in that generator had not travelled the screen's distance in the
/// periods the hand was on. So the two figures are the point: radians behind
/// the goal, and nothing raised.
fn check_the_jam_was_a_lag(run: &Run, failures: &mut Vec<String>) {
    let threshold = default_motion_config().tracking.threshold_rad;
    let at = jam_release_cycle() - 1;
    let what = "the jam is about to be released";
    let Some(goal) = check::goal_at_or(run, at, what, failures) else {
        return;
    };
    let Some(present) = check::sample_at(run, at).map(check::present_rows) else {
        failures.push(format!("no sample for cycle {at}, where {what}"));
        return;
    };
    let mut worst: f64 = 0.0;
    for joint in flags::iter(jammed_rows()) {
        let Some(row) = row(joint) else {
            failures.push(format!("{} sits on no bus row", Name(joint)));
            continue;
        };
        worst = worst.max((goal[row] - present[row]).abs());
    }
    if worst < threshold {
        failures.push(format!(
            "at cycle {at} the furthest jammed crank sits {worst} rad from its goal, inside the \
             {threshold} rad the detector screens on: a jam nothing measures is a jam this run \
             cannot say anything about"
        ));
    }
}
