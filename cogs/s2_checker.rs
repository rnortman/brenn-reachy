//! S2's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What S2 shares with a healthy run is asserted by
//! `scenario::check`; what is here is the hand and its answer -- that the cranks
//! really stood still, that the detector raised `head_obstructed` once and on
//! the cycle the walk says it would, that the answer was the rest-class stow and
//! that it ran end to end with the machine measured at the fold, that the hand
//! came off inside the window the raise left and that the cranks then closed
//! the distance it had opened on their own -- and then that the machine let go
//! of at rest takes another script, which is the half of the doctrine's
//! park/rest split a refusal cannot show.
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
use reachy_motion::joints::{Name, flags, row};
use reachy_motion::postures::{neutral_targets, stow_pose_targets};
use reachy_motion::tick::default_motion_config;
use scenario::check;
use scenario::check::{present_rows, sample_at};
use scenario::read::Run;
use scenario::{stow_clocks, up_clocks};

use s2_scenario::{
    SCRIPT_ID, SECOND_SCRIPT_ID, caught_up_cycle, end_cycle, jammed_rows, obstruct_cycle,
    raise_cycle, recovery_bound_cycle, release_cycle, script_sent_cycle, second_disengage_cycle,
    second_script_cycle, second_stow_cycle, second_stow_cycles, second_up_cycles,
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
        // Nine phase changes and no more: the survey, the first script taken,
        // the arming, the machine carried down over the obstruction, the rest it
        // reached, and then the ordinary life of the session that takes the
        // second script. A tenth would be a session that did something with a
        // machine it had already let go of.
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
        // The cycles this run measures from, named by the change each one is
        // rather than indexed at the point of use: an ordinal read below would
        // be an assertion about whichever change a later edit moved into that
        // slot.
        let taken = cycles.get(2).copied();
        let carried_down = cycles.get(3).copied();
        let rested = cycles.get(4).copied();
        let second_taken = cycles.get(6).copied();
        let second_let_go = cycles.get(7).copied();
        if let Some(carried_down) = carried_down {
            // The maneuver is the answer to the obstruction: the session
            // entered it on the wake that read the raise, and not before.
            check::answered_on_the_message(
                "machine being carried down",
                carried_down,
                raise_cycle(),
                failures,
            );
        }
        // The second session let go of its schedule promptly; the release it
        // then ran is the orderly one, settle and all, and the phase it ends in
        // is what says so.
        check::ended_promptly(second_let_go, second_disengage_cycle(), failures);
        // Two stretches of stream, because nothing is commanded between a
        // session that ended and the next arming taking hold: the machine is
        // de-torqued for that whole interval, and what holds the driver's
        // dead-man off once it is energised again is the keep-alive. Every
        // stretch names every joint on every cycle -- the obstruction's answer
        // is a stow, which masks nothing.
        let streams = check::goal_streams_exactly(run, JointFlags::NONE, 2, failures);
        if let [first, second] = streams.as_slice() {
            if let (Some(taken), Some(rested)) = (taken, rested) {
                check::stream_starts_with_session(first, taken, failures);
                check::stream_stops_with_release(first, rested, failures);
            }
            if let (Some(taken), Some(let_go)) = (second_taken, second_let_go) {
                check::stream_starts_with_session(second, taken, failures);
                check::stream_stops_with_release(second, let_go, failures);
            }
        }
        check_second_engagement(run, failures);
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);

        // The scenario's hand really was on the machine: the jammed cranks do
        // not move at all for as long as the jam lasts. Asserted because
        // everything else in this file is about what the loop did with a
        // stalled joint, and a run where nothing stalled would report none of
        // it for the wrong reason. The window ends before the release, because
        // an injection takes effect on the cycle it names: the sample for the
        // cycle the release was published on already shows the rows moving
        // again.
        check::stands_still_rows(
            run,
            jammed_rows(),
            obstruct_cycle(),
            release_cycle() - 1,
            "held by a hand against a goal that kept moving",
            failures,
        );
        check_the_generator_had_stopped(run, failures);
        check_the_hand_came_off_in_time(run, failures);
        check_the_cranks_caught_up(run, failures);
        // The one condition of this run: the cranks a hand held, raised once by
        // the tick and recorded once by the session. Once is the whole of the
        // claim -- a persisting obstruction re-raises every window, and this
        // hand comes off inside the first one, so a second record would be a
        // release the detector did not accept.
        check::faults_recorded(
            run,
            &[check::Expected {
                kind: FaultKindWire::HEAD_OBSTRUCTED,
                rows: jammed_rows(),
                from: raise_cycle(),
                through: raise_cycle() + 1,
                how_many: check::Recorded::Times(1),
                raised_by_tick: true,
                why: "the cranks a hand held against their trajectory",
            }],
            failures,
        );
        check_the_raise(run, failures);
        check_the_answer(run, failures);
        // The first session's schedules only: everything published from the
        // cycle the second script arrives on is the second engagement's
        // business, and what this says something about is the fold the
        // wind-down asked for.
        check::stows_until(run, carried_down, Some(second_script_cycle() - 1), failures);
        check_the_fold(run, rested, failures);

        // The machine really did go limp between the two sessions and at the
        // end, and the only edges in the run are the read-backs that say so:
        // everything else the gate raises is about a commander that went quiet
        // while the machine was energised, which is what the keep-alive and the
        // goal stream exist to prevent -- through the hold the raise leaves as
        // much as through the stow that follows it.
        check::only_kinds(run, &[EventKind::TorqueOffConfirmed], failures);
        check::signal_groups(run, failures);
    })
}

/// The tick raised the obstruction once, on the cycle the walk says the window
/// runs out on, naming the cranks and the residual it measured.
///
/// The instant is the whole point of deriving the hand's placement: the residual
/// passes the screen when the generator has travelled `threshold_rad` past the
/// held cranks, and the fault comes a window after that. A run raising a cycle
/// either side is a run whose plant or whose screen no longer matches the walk
/// this scenario is written over, which is a reading for a human rather than a
/// number to adjust.
fn check_the_raise(run: &Run, failures: &mut Vec<String>) {
    let raised = check::raises(run);
    let [only] = raised.as_slice() else {
        failures.push(format!(
            "the tick raised {}: this run has one hand on the cranks, held past the screen for \
             one window and off them again inside the next",
            raised.len()
        ));
        return;
    };
    check::raise_carries(
        only,
        FaultKindWire::HEAD_OBSTRUCTED,
        "a hand on the cranks",
        failures,
    );
    if only.at != raise_cycle() {
        failures.push(format!(
            "the tick raised on cycle {} and the stepped walk of this raise puts the window \
             running out on {}: the hand is placed so that the generator travels the screen's \
             distance past the held cranks and the fault comes a window later",
            only.at,
            raise_cycle()
        ));
    }
}

/// The answer: one response, the rest-class stow, and one maneuver that ended
/// completed with the machine let go of at rest.
///
/// One response is the doctrine's. The hand comes off inside the window the
/// raise leaves, so the stow it commanded runs end to end: nothing is raised
/// again, the maneuver is neither expanded nor defeated, and the disposition it
/// concludes with is the rest the detector's own rung names.
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
            ReportKindWire::WINDDOWN_OUTCOME,
        ],
        "an obstruction answered under control and a second script taken afterwards",
        failures,
    );
    let stow =
        u32::from(ResponseKindWire::from(reachy_motion::tick::ResponseKind::SlowStowToRest).0);
    if told.responses() != vec![stow] {
        failures.push(format!(
            "the session selected {:?}, and a head obstruction is answered once with the slow \
             stow to rest ({stow}): a second answer would be a second clock over one machine",
            told.responses()
        ));
    }
    let completed = u32::from(WindDownOutcomeWire::COMPLETED.0);
    if told.endings() != vec![(completed, 0)] {
        failures.push(format!(
            "the maneuver ended as {:?}, and this run's hand comes off inside the window the \
             raise left, so the stow reaches the fold and ends at the rest its own rung names",
            told.endings()
        ));
    }
}

/// The machine really was carried down: the machine the session let go of is
/// standing at the fold, by the maneuver's own measure.
///
/// The maneuver's tolerance rather than a posture arrival, because what ended it
/// is `at_stow` over the driver's sample -- asserting a tighter number here
/// would be this checker's opinion of a fold rather than the one the machine was
/// measured against.
fn check_the_fold(run: &Run, rested: Option<i64>, failures: &mut Vec<String>) {
    let Some(rested) = rested else {
        return;
    };
    if check::folded_at(run, rested, "measured at the fold", failures) == Some(false) {
        failures.push(format!(
            "the machine the session let go of on cycle {rested} is not at the fold, and the \
             maneuver it ended reported the head measured there"
        ));
    }
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
    check::room("second upright", second_up_cycles(), &up_clocks(), failures);
    check::room(
        "second stow",
        second_stow_cycles(),
        &stow_clocks(),
        failures,
    );
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

/// The generator the released cranks are judged against had come to rest by the
/// raise, measured in the samples rather than in the walk.
///
/// The premise the release rests on, and the one this run is the suite's only
/// pin for. A released joint sets off from rest and restarts the reopened run
/// by regaining the progress minimum on the third step of its ramp -- but only
/// against a prediction that has stopped. Against one still running at the
/// profile it can neither close nor pace, and the recovery bound this run
/// releases inside is then two periods too late: the stow would be defeated on
/// some runs and survive on others, with nothing naming the cause.
///
/// What makes the prediction stopped is the setpoint: it chases the one the
/// driver held a dead time earlier, so a held setpoint that has not moved for
/// the ramp and the response delay together is a trajectory at rest. The
/// scenario places the jam so that this holds, off the walk of the raise; this
/// is that placement read back out of the run.
fn check_the_generator_had_stopped(run: &Run, failures: &mut Vec<String>) {
    let settled_from = raise_cycle() - scenario::ramp_cycles() - scenario::response_delay_cycles();
    check::commanded_stands_still_rows(
        run,
        jammed_rows(),
        settled_from,
        raise_cycle(),
        0.0,
        "the cranks have arrived and their generator is braking to a stop before the fault lands",
        failures,
    );
}

/// The machine came back on its own: every crank the hand held is back inside
/// the screen's own distance of the goal it is being commanded to, a catch-up
/// after the release.
///
/// Nothing was reset and nothing was retried -- the hand came off and the
/// servos closed the distance it had opened, under the stow that was already
/// running. Every crank and not one of them: a release that let one twitch and
/// stall again, or recovered one row and left the other five behind, is a
/// machine that did not come back, and the fold measured at the end carries the
/// maneuver's own tolerance rather than this one.
///
/// The instant is the catch-up itself: a joint setting off from rest covers the
/// screen's distance in `pass_cycles(threshold_rad)`, and the goal it is closing
/// on is descending under the fold meanwhile, which is what the settle past that
/// figure is for.
///
/// What carries the assertion is the cranks the fold sends further from where
/// the hand left them; the fold's own goal descends *through* the others, so
/// their distance from it closes whether they moved or not. That they moved is
/// the release assertion above, and where they all ended is the fold.
fn check_the_cranks_caught_up(run: &Run, failures: &mut Vec<String>) {
    let threshold = default_motion_config().tracking.threshold_rad;
    let at = caught_up_cycle();
    let what = "the released cranks have had the catch-up the release owes them";
    let Some(goal) = check::goal_at_or(run, at, what, failures) else {
        return;
    };
    let Some(present) = sample_at(run, at).map(present_rows) else {
        failures.push(format!("no sample for cycle {at}, where {what}"));
        return;
    };
    for joint in flags::iter(jammed_rows()) {
        let Some(row) = row(joint) else {
            failures.push(format!("{} sits on no bus row", Name(joint)));
            continue;
        };
        let lag = (goal[row] - present[row]).abs();
        if lag >= threshold {
            failures.push(format!(
                "at cycle {at}, {} cycles after the hand came off, {} is still {lag} rad from the \
                 goal it is being commanded to, past the {threshold} rad the detector screens on: \
                 the machine is expected to close the distance the hand opened on its own, with \
                 nothing reset and nothing retried",
                at - release_cycle(),
                Name(joint)
            ));
        }
    }
}

/// The hand came off inside the window the raise left, measured in the samples
/// rather than in the schedule.
///
/// This is the premise the whole second half of the run rests on. A release is
/// read by the plant some cycles after the scenario published it, so what
/// decides whether the reopened run restarts is the first reading that shows the
/// cranks moving again -- a released joint sets off from rest and needs three
/// steps of its ramp to regain the progress minimum, which is why the bound is a
/// window less that ramp. A release read later than this is a second raise and a
/// defeated stow, which is a different run.
/// Every crank the hand held and not one of them: a release that let one row go
/// and left the other five held is a hand still on the machine, and the run it
/// produces is the second raise this scenario is not about.
fn check_the_hand_came_off_in_time(run: &Run, failures: &mut Vec<String>) {
    let bound = recovery_bound_cycle();
    let Some(held) = sample_at(run, obstruct_cycle()).map(present_rows) else {
        return;
    };
    for joint in flags::iter(jammed_rows()) {
        let Some(row) = row(joint) else {
            failures.push(format!("{} sits on no bus row", Name(joint)));
            continue;
        };
        let moved = (release_cycle()..=bound).any(|cycle| {
            sample_at(run, cycle)
                .map(present_rows)
                .is_some_and(|sample| sample[row] != held[row])
        });
        if !moved {
            failures.push(format!(
                "{} reads what it read at the jam through cycle {bound}, the last cycle a release \
                 restarts the run the raise reopened: the hand is published on {} and the plant \
                 reads it a cycle or two later, on every row it was on",
                Name(joint),
                release_cycle()
            ));
        }
    }
}
