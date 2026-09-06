//! S2's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What S2 shares with a healthy run is asserted by
//! `scenario::check`; what is here is the jam itself -- that the cranks really
//! stood still, that they sat far past the detector's threshold for the whole of
//! it and closed again when the hand came off, that nothing was reported about
//! any of it, and that the machine stayed under command throughout -- and then
//! that a machine which ran its schedule out takes another script, which is the
//! half of the doctrine's park/rest split that a refusal cannot show.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__driver__health_clk_rs::EventKind;
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use reachy_motion::default_motion_config;
use reachy_motion::joints::{Name, flags, row};
use reachy_motion::postures::{neutral_targets, stow_pose_targets};
use scenario::check;
use scenario::check::{goal_at, present_rows, sample_at};
use scenario::read::Run;
use scenario::{stow_clocks, up_clocks};

use s2_scenario::{
    SCRIPT_ID, SECOND_SCRIPT_ID, SECOND_STOW_CYCLES, SECOND_UP_CYCLES, disengage_cycle, end_cycle,
    jammed_rows, obstruct_cycle, release_cycle, script_sent_cycle, second_disengage_cycle,
    second_script_cycle, second_stow_cycle, stow_start_cycle,
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
        // The ordinary life of a session, twice: nothing about the jam reaches
        // the phases, so the first session runs its schedule out and lets go,
        // and the machine it let go of is engaged again by the second script.
        let (engaged, second) = check::engagement_cycles(
            run,
            &[
                (SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
                (SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING),
                (SessionPhaseWire::STOPPING, SessionPhaseWire::ACTIVE),
                (SessionPhaseWire::RESTING, SessionPhaseWire::STOPPING),
            ],
            failures,
        );
        // The second session's cycles, named by the change each one is rather
        // than indexed at the point of use: the list above is this scenario's
        // own, and an ordinal read below would be an assertion about whichever
        // change a later edit moved into that slot.
        let second_taken = second.get(1).copied();
        let second_let_go = second.get(2).copied();
        check::ended_promptly(
            engaged.map(|engaged| engaged.released),
            disengage_cycle(),
            failures,
        );
        // The second session let go of its schedule promptly; the release it
        // then ran is the orderly one, settle and all, and the phase it ends in
        // is what says so.
        check::ended_promptly(second_let_go, second_disengage_cycle(), failures);
        // Two stretches of stream, because nothing is commanded between a
        // session that ended and the next arming taking hold: the machine is
        // de-torqued for that whole interval, and what holds the driver's
        // dead-man off once it is energised again is the keep-alive. Every
        // stretch names every joint on every cycle, jam or no jam.
        let streams = check::goal_streams_exactly(run, JointFlags::NONE, 2, failures);
        if let [first, second] = streams.as_slice() {
            if let Some(engaged) = engaged {
                check::stream_starts_with_session(first, engaged.taken, failures);
                check::stream_stops_with_release(first, engaged.released, failures);
            }
            if let (Some(taken), Some(let_go)) = (second_taken, second_let_go) {
                check::stream_starts_with_session(second, taken, failures);
                check::stream_stops_with_release(second, let_go, failures);
            }
        }
        check_second_engagement(run, failures);
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);

        check_jam_held(run, failures);
        check_lagged_and_recovered(run, failures);
        check_move_ran_its_clock(run, failures);
        // Nothing is wrong with this machine that the loop answers. The hand on
        // the cranks is a stall the disarmed detector measures and does not
        // judge, so the run carries no fault at all.
        check::no_faults(run, failures);
        // The machine really did go limp between the two sessions and at the
        // end, and the only edges in the run are the read-backs that say so:
        // everything else the gate raises is about a commander that went quiet
        // while the machine was energised, which is what the keep-alive and the
        // goal stream exist to prevent.
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
/// Asserted because everything else in this file is about what the loop did with
/// a stalled joint, and a run where nothing stalled would report none of it for
/// the wrong reason.
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

/// The jam is a lag in the record: some jammed crank ends it sitting further
/// from its goal than the detector's own threshold, and every one of them is
/// back inside that threshold once the hand has come off.
///
/// This is what the run is for. A disarmed detector answers a stall with
/// nothing, so what says the stall was real and was seen is the distance
/// between the goal and the position in the samples -- past the threshold the
/// armed detector would have screened on, so the same run re-read with the
/// detector armed is the one that raises. And the close afterwards says the
/// machine came back on its own: nothing was reset, nothing was retried, the
/// hand simply came off and the servos caught up.
///
/// The recovery is measured a whole detector window past the release rather than
/// on the cycle after it, because a crank held for eighty cycles has that much
/// commanded travel to make up and the goal is still moving while it does.
fn check_lagged_and_recovered(run: &Run, failures: &mut Vec<String>) {
    let threshold = default_motion_config().tracking.threshold_rad;
    let window = i64::from(default_motion_config().tracking.ticks);
    let at = release_cycle() - 1;
    let Some(worst) = lag_at(run, at, "the jam is about to be released", failures) else {
        return;
    };
    if worst < threshold {
        failures.push(format!(
            "at cycle {at} the furthest jammed crank sits {worst} rad from its goal, inside the \
             {threshold} rad the detector screens on: a jam nothing measures is a jam this run \
             cannot say anything about"
        ));
    }
    let settled = stow_start_cycle() - 1;
    let Some(after) = lag_at(run, settled, "the jam is long over", failures) else {
        return;
    };
    if after >= threshold {
        failures.push(format!(
            "at cycle {settled}, {} cycles after the hand came off, a jammed crank is still \
             {after} rad from its goal: the machine is expected to catch up on its own once the \
             obstruction is gone, within the detector's {window}-cycle window and a long way over",
            settled - release_cycle()
        ));
    }
}

/// How far the furthest jammed crank sits from its goal on `cycle`.
fn lag_at(run: &Run, cycle: i64, what: &str, failures: &mut Vec<String>) -> Option<f64> {
    let goal = check::goal_at_or(run, cycle, what, failures)?;
    let Some(present) = sample_at(run, cycle).map(present_rows) else {
        failures.push(format!("no sample for cycle {cycle}, where {what}"));
        return None;
    };
    let mut worst: f64 = 0.0;
    for joint in flags::iter(jammed_rows()) {
        let Some(row) = row(joint) else {
            failures.push(format!("{} sits on no bus row", Name(joint)));
            continue;
        };
        worst = worst.max((goal[row] - present[row]).abs());
    }
    Some(worst)
}

/// The move the jam happened under ran out on its own clock: the machine is
/// upright by the end of the step that sends it there, and the fold opens where
/// the script said it would.
///
/// A jam the loop answered would have taken the move away -- the tick abandons
/// it and holds, and the session commands a fold on a clock of its own. Nothing
/// does either here, so the schedule the script asked for is the schedule the
/// run has, jam and all.
fn check_move_ran_its_clock(run: &Run, failures: &mut Vec<String>) {
    check::arrived_at(
        run,
        "upright with the jam behind it",
        stow_start_cycle() - 1,
        &neutral_targets(),
        failures,
    );
    check::arrived_at(
        run,
        "stowed on the first script",
        disengage_cycle() - 1,
        &stow_pose_targets(),
        failures,
    );
    // The fold opens where the step does, which says the stow the machine ran is
    // the script's and not an answer to anything: a wind-down's stow is opened
    // on the cycle the condition was decided at, which here would be inside the
    // jam and nowhere near this.
    let at = stow_start_cycle();
    if let (Some(before), Some(after)) = (goal_at(run, at - 1), goal_at(run, at + 1)) {
        if before == after {
            failures.push(format!(
                "the goal did not change across cycle {at}, where the script's fold opens: the \
                 first script's second step is expected to reach the machine"
            ));
        }
    } else {
        failures.push(format!(
            "no goals around cycle {at}, where the script's fold opens"
        ));
    }
}
