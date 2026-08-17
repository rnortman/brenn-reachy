//! S5's assertions, over the output log.
//!
//! Three arguments: the output log directory and the two config textprotos the
//! process ran against. What S5 shares with a healthy run is asserted by
//! `scenario::check` -- and most of the retarget's own contract is in there
//! already, because "the goal stream is one datagram per sample, each due after
//! the last, each within a cycle's travel of the one before" is exactly what a
//! turnaround could break. What is here is the rest: that the machine really
//! was travelling when the schedule changed, that it turned round, and that it
//! got where the new schedule sent it.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use nalgebra::Isometry3;
use reachy_kin::{neutral_head_pose, stow_head_pose};
use reachy_motion::postures::stow_pose_targets;
use scenario::check;
use scenario::check::head_pose_at;
use scenario::read::Run;

use s5_scenario::{
    DISENGAGE_CYCLE, DISENGAGED_EPOCH, END_CYCLE, ENGAGE_CYCLE, ENGAGED_EPOCH, RETARGET_CYCLE,
    RETARGET_EPOCH, TURNAROUND_CYCLES, stow_cycles, up_cycles,
};

/// How far the head may drift back from the posture it is closing on between
/// one cycle and the next, metres.
///
/// Not a tolerance for a machine that changed its mind twice: the plant tracks
/// a monotone path here, so the only thing this covers is the last bits of the
/// solver's arithmetic. A retarget that left the machine oscillating would
/// exceed it by orders of magnitude.
const CLOSING_SLACK: f64 = 1e-9;

/// How far from each posture the head has to be at the retarget, as a share of
/// the distance between the two.
///
/// A share rather than a length, because what it asserts is that the machine
/// was somewhere in the middle of its travel, and the travel is the linkage's
/// business: a length in millimetres would quietly become the whole span on a
/// machine whose postures sat closer together.
const MID_MOVE_SHARE: f64 = 0.25;

fn main() -> ExitCode {
    check::main("s5_checker", |run, failures| {
        check::heartbeat(run, END_CYCLE, failures);
        check::readings_present(run, failures);
        // The goal stream is where a retarget breaks if it breaks at all: a gap
        // in it, an instant out of order, or a step past what the plant can
        // travel are all this scenario's failure modes, and all three are
        // asserted for every cycle of the run rather than only around the
        // turnaround.
        if let Some(stream) = check::goal_stream(run, failures) {
            check::stream_covers_session(&stream, ENGAGE_CYCLE, DISENGAGE_CYCLE, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        // The retarget is an epoch changing, so the three epochs reaching the
        // run is the mechanism itself. The turnaround alone would leave a run
        // whose second schedule never arrived looking like one that did, up to
        // the behaviour it happens to differ in.
        check::schedules_replayed(
            run,
            &[
                check::Session {
                    cycle: ENGAGE_CYCLE,
                    engaged: true,
                    epoch: ENGAGED_EPOCH,
                },
                check::Session {
                    cycle: RETARGET_CYCLE,
                    engaged: true,
                    epoch: RETARGET_EPOCH,
                },
                check::Session {
                    cycle: DISENGAGE_CYCLE,
                    engaged: false,
                    epoch: DISENGAGED_EPOCH,
                },
            ],
            failures,
        );

        check_mid_move(run, failures);
        check_closing(run, failures);
        check_arrival(run, failures);
        check::no_faults(run, failures);
        check::no_events(run, failures);
        check::signal_groups(run, failures);
    })
}

/// The schedule really did change under a move: at the cycle it landed on, the
/// machine is standing at neither posture.
///
/// The scenario's own arithmetic is checked first, because the two say
/// different things. That the retarget falls inside the raise's duration is a
/// statement about the numbers this scenario picked; that the head is nowhere
/// near either posture is a statement about the run those numbers produced, and
/// a machine that had arrived early or never set off would satisfy the first
/// while making every assertion below meaningless.
fn check_mid_move(run: &Run, failures: &mut Vec<String>) {
    let move_cycles = up_cycles();
    if RETARGET_CYCLE >= move_cycles {
        failures.push(format!(
            "the retarget is at cycle {RETARGET_CYCLE} and the raise is given {move_cycles} \
             cycles: the scenario does not change the posture during a move"
        ));
    }
    let Some(found) = head_pose_at(run, RETARGET_CYCLE) else {
        failures.push(format!(
            "no pose for cycle {RETARGET_CYCLE}, where the schedule changed"
        ));
        return;
    };
    let span = apart(&neutral_head_pose(), &stow_head_pose());
    for (what, posture) in [
        ("upright", neutral_head_pose()),
        ("stowed", stow_head_pose()),
    ] {
        let offset = apart(&found, &posture);
        if offset <= span * MID_MOVE_SHARE {
            failures.push(format!(
                "at cycle {RETARGET_CYCLE} the head is {offset} m from {what}, out of the {span} m \
                 between the two postures: the schedule changed under a machine that had all but \
                 arrived rather than under a move"
            ));
        }
    }
}

/// The machine turns round: once the new setpoint can have reached the plant,
/// the head only ever gets closer to the posture it was redirected to.
///
/// This is what tells a retarget from a move that carried on and a stow that
/// happened to follow it. The travel it measures is the plant's, read back
/// through the estimator, so it is a claim about where the machine went rather
/// than about what it was asked for.
fn check_closing(run: &Run, failures: &mut Vec<String>) {
    let stow = stow_head_pose();
    let from = RETARGET_CYCLE + TURNAROUND_CYCLES;
    let Some(start) = head_pose_at(run, from) else {
        failures.push(format!(
            "no pose for cycle {from}, where the machine should have turned round"
        ));
        return;
    };
    let mut previous = apart(&start, &stow);
    for cycle in from + 1..DISENGAGE_CYCLE {
        let Some(found) = head_pose_at(run, cycle) else {
            failures.push(format!("no pose for cycle {cycle}, inside the stow move"));
            return;
        };
        let offset = apart(&found, &stow);
        if offset > previous + CLOSING_SLACK {
            failures.push(format!(
                "at cycle {cycle} the head is {offset} m from stow, having been {previous} m from \
                 it a cycle earlier: the machine is not closing on the posture it was redirected to"
            ));
            return;
        }
        previous = offset;
    }
}

/// The machine arrives: stowed by the end of the step the retarget put it on.
fn check_arrival(run: &Run, failures: &mut Vec<String>) {
    check::arrived_at(
        run,
        "stowed",
        DISENGAGE_CYCLE - 1,
        &stow_pose_targets(),
        failures,
    );
    check::room(
        "stow",
        DISENGAGE_CYCLE - RETARGET_CYCLE,
        stow_cycles(),
        failures,
    );
}

/// How far apart two head positions are, metres.
fn apart(found: &Isometry3<f64>, wanted: &Isometry3<f64>) -> f64 {
    (found.translation.vector - wanted.translation.vector).norm()
}
