//! S14's assertions, over the output log.
//!
//! Four arguments: the output log directory and the config textprotos the
//! process ran against. What S14 shares with a healthy run is asserted by
//! `scenario::check`; what is here is the hand that stays on and the ending it
//! gets -- two raises a window apart, one answer, a maneuver concluded fallen
//! through with nearly its whole clock unspent, the machine let go of nowhere
//! near the fold, and nothing driven further into the hand than the goal that
//! was already out when the joint was judged.
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
use scenario::check;
use scenario::read::Run;
use scenario::{LAG_K, PERIOD_NS};

use s14_scenario::{
    SCRIPT_ID, clock_left_ns, end_cycle, jammed_rows, obstruct_cycle, raise_cycle,
    script_sent_cycle, second_raise_cycle,
};

fn main() -> ExitCode {
    check::main("s14_checker", |run, failures| {
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        check::scripts_sent(run, &[(SCRIPT_ID, script_sent_cycle())], failures);
        // Five phase changes and no more: the survey, the script taken, the
        // arming, the machine carried down over the obstruction, and the rest it
        // was let go at. A sixth would be a session that did something with a
        // machine it had already let go of -- and this one is let go of holding
        // an obstruction, which is exactly the state nothing may re-engage
        // without an operator having seen it.
        let cycles = check::phases(
            run,
            &[
                (SessionPhaseWire::RESTING, SessionPhaseWire::STARTING),
                (SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
                (SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING),
                (SessionPhaseWire::WINDING_DOWN, SessionPhaseWire::ACTIVE),
                (SessionPhaseWire::RESTING, SessionPhaseWire::WINDING_DOWN),
            ],
            failures,
        );
        let taken = cycles.get(2).copied();
        let carried_down = cycles.get(3).copied();
        let let_go = cycles.get(4).copied();
        if let Some(carried_down) = carried_down {
            // The maneuver is the answer to the obstruction: the session
            // entered it on the wake that read the first raise, and not
            // before.
            check::answered_on_the_message(
                "machine being carried down",
                carried_down,
                raise_cycle(),
                failures,
            );
        }
        if let Some(let_go) = let_go {
            // And it ended because the second raise defeated it, not because
            // its clock ran out: the machine was let go of on the wake that
            // read that raise, most of four seconds inside the budget. A
            // fall-through the stow's own motion produced would land later and
            // an expiry at the deadline, so the instant is what makes the
            // ending the one the doctrine names rather than the one the clock
            // reaches anyway. What the clock had left is asserted beside the
            // answer.
            check::answered_on_the_message(
                "machine being let go of",
                let_go,
                second_raise_cycle(),
                failures,
            );
        }
        // One stretch of stream: one engagement, and the fold the maneuver
        // commands masks nothing, so every goal in it speaks for all nine rows.
        let streams = check::goal_streams_exactly(run, JointFlags::NONE, 1, failures);
        if let ([only], Some(taken), Some(let_go)) = (streams.as_slice(), taken, let_go) {
            check::stream_starts_with_session(only, taken, failures);
            check::stream_stops_with_release(only, let_go, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);

        // The scenario's hand really was on the machine, all the way to the
        // release: the jammed cranks do not move at all from the jam to the
        // cycle the session let go of them. Asserted because everything else
        // in this file is about what the loop did with a stalled joint, and a
        // run where nothing stalled -- or where the stall ended early -- would
        // report none of it for the wrong reason.
        if let Some(let_go) = let_go {
            check::stands_still_rows(
                run,
                jammed_rows(),
                obstruct_cycle(),
                let_go,
                "held by a hand that never comes off",
                failures,
            );
        }
        // The cranks a hand held, raised twice by the tick and recorded twice by
        // the session. Twice is the whole of the claim: an obstruction that
        // persists is re-raised every window out of the hold, and this hand is
        // never taken off, so the second record is the evidence the release
        // rests on.
        check::faults_recorded(
            run,
            &[check::Expected {
                kind: FaultKindWire::HEAD_OBSTRUCTED,
                rows: jammed_rows(),
                from: raise_cycle(),
                through: second_raise_cycle() + 1,
                how_many: check::Recorded::Times(2),
                raised_by_tick: true,
                why: "the cranks a hand held against their trajectory and never let go of",
            }],
            failures,
        );
        check_the_two_raises(run, failures);
        check_the_answer(run, failures);
        check::stows(run, carried_down, failures);
        check_the_hand_was_not_pushed_into(run, let_go, failures);
        check_the_head_was_not_folded(run, let_go, failures);
        // The release really went out and the machine really went limp, and the
        // read-backs that say so are the only edges in the run: everything else
        // the gate raises is about a commander that went quiet while the machine
        // was energised, which the keep-alive carries through the hold and the
        // stow's lead-in carries to the release. A run showing a hold timeout
        // would be a hold that fell silent.
        check::confirmed_off(run, check::first_release(run, failures), failures);
        check::only_kinds(run, &[EventKind::TorqueOffConfirmed], failures);
        check::signal_groups(run, failures);
    })
}

/// The tick raised the obstruction twice, a window apart, each raise naming the
/// cranks and the residual it measured.
///
/// The cadence is what this run is about. A non-latching raise leaves the tick
/// holding and the tracking sweep judging, so the run reopens on the tick after
/// the raise; a joint a hand is still holding neither closes on its prediction
/// nor paces it, and the window runs out on its own count. A second raise a
/// cycle either side of this would be a hold whose goal moved, which is a
/// different run.
fn check_the_two_raises(run: &Run, failures: &mut Vec<String>) {
    let raised = check::raises(run);
    let [first, second] = raised.as_slice() else {
        failures.push(format!(
            "the tick raised {} time(s): this run has one hand on the cranks, held past the \
             screen for one window and never taken off, which is answered once out of the move \
             and once out of the hold",
            raised.len()
        ));
        return;
    };
    for raise in [first, second] {
        check::raise_carries(
            raise,
            FaultKindWire::HEAD_OBSTRUCTED,
            "a hand on the cranks",
            failures,
        );
    }
    if first.at != raise_cycle() {
        failures.push(format!(
            "the tick raised first on cycle {} and the stepped walk of this raise puts the window \
             running out on {}: the hand is placed so that the generator travels the screen's \
             distance past the held cranks and the fault comes a window later",
            first.at,
            raise_cycle()
        ));
    }
    if second.at != second_raise_cycle() {
        failures.push(format!(
            "the tick raised again on cycle {} and a persisting obstruction is re-raised a window \
             after the last, on {}: the hold re-publishes the goal that was already out, so \
             nothing in it closes or paces",
            second.at,
            second_raise_cycle()
        ));
    }
}

/// The answer: one response, the rest-class stow, and one maneuver that ended
/// fallen through with the clock the defeat left it.
///
/// One response is the doctrine's -- the ladder never begins a second answer, so
/// the second raise defeats the maneuver rather than opening another. The
/// disposition is rest, which is the rung a head obstruction names, and the
/// clock left in hand is a whole window less than the budget: the maneuver was
/// opened on the wake that read the first raise and concluded on the wake that
/// read the second. An expiry would report nothing left and a defeat the stow's
/// own motion produced would report seconds less, so the figure is what tells
/// the three endings apart in the record.
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
        "an obstruction that would not let go, answered by letting the machine go",
        failures,
    );
    let stow =
        u32::from(ResponseKindWire::from(reachy_motion::tick::ResponseKind::SlowStowToRest).0);
    if told.responses() != vec![stow] {
        failures.push(format!(
            "the session selected {:?}, and a head obstruction is answered once with the slow \
             stow to rest ({stow}): the ladder never begins a second answer, so the raise that \
             defeats the maneuver narrates none",
            told.responses()
        ));
    }
    let fell_through = u32::from(WindDownOutcomeWire::FELL_THROUGH.0);
    let [ending] = told.outcomes.as_slice() else {
        failures.push(format!(
            "the maneuver ended as {:?}, and this run has one maneuver in it, defeated by the \
             raise that came out of its own hold",
            told.endings()
        ));
        return;
    };
    if (ending.outcome, ending.disposition) != (fell_through, 0) {
        failures.push(format!(
            "the maneuver ended as ({}, {}), and a stow a head obstruction defeats falls through \
             ({fell_through}) with the rest its own rung names: a machine let go of at rest is \
             one the next wake may engage",
            ending.outcome, ending.disposition
        ));
    }
    let wanted_s = clock_left_ns() as f64 / 1e9;
    let slack_s = PERIOD_NS as f64 / 1e9;
    if (ending.left_s - wanted_s).abs() > slack_s {
        failures.push(format!(
            "the maneuver concluded with {} s of its clock left, and the defeat leaves it \
             {wanted_s} s: the clock is opened on the wake that reads the first raise and \
             concluded on the wake that reads the second, a window later",
            ending.left_s
        ));
    }
}

/// The cranks the hand is on were driven no further into it than the goal that
/// was already out when they were judged.
///
/// This is what the hold costs, and the whole of it. The tick emits nothing once
/// it holds, and the keep-alive re-publishes the last goal it emitted -- which
/// the driver is holding from a commanded lag after the raise onward, since a
/// goal is dated that far ahead of the sample that decided it. From there the
/// held setpoint stands still until the stow's own lead-in begins, and a
/// min-jerk goal a window into its clock has covered a thousandth of its
/// distance: on the cranks that is a few thousandths of a radian, under the
/// progress minimum, which is why the reopened run neither closes nor paces and
/// the second raise lands on its own count.
///
/// The cranks and not every row, because this is a statement about the hand: the
/// antenna swept inboard over the head travels more than four times the
/// cranks' arc and its
/// lead-in reaches about a hundredth of a radian by the release, which is
/// nothing anything is holding and nowhere near the fold. That the machine is
/// nowhere near the fold when it is let go is asserted on the pose itself.
fn check_the_hand_was_not_pushed_into(run: &Run, let_go: Option<i64>, failures: &mut Vec<String>) {
    let Some(let_go) = let_go else {
        return;
    };
    check::commanded_stands_still_rows(
        run,
        jammed_rows(),
        raise_cycle() + LAG_K,
        let_go,
        default_motion_config().tracking.progress_min_rad,
        "the hold re-publishes the one goal that was already out and the stow's lead-in never \
         gets going",
        failures,
    );
}

/// The head is *not* at the fold when the machine is let go of.
///
/// The inverse of the fold measurement every stow that completes carries, and
/// the cost this run exists to state: a stow a hand defeats drops the head where
/// it stands rather than finishing the descent, because yielding is the point.
/// A run that reached the fold anyway would be one whose stow was never defeated.
fn check_the_head_was_not_folded(run: &Run, let_go: Option<i64>, failures: &mut Vec<String>) {
    let Some(let_go) = let_go else {
        return;
    };
    if check::folded_at(run, let_go, "let go of over the obstruction", failures) == Some(true) {
        failures.push(format!(
            "the machine the session let go of on cycle {let_go} is standing at the fold, and a \
             stow a head obstruction defeats is not driven there: the head is released where the \
             hand held it"
        ));
    }
}
