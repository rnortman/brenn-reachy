//! S5's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
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

use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use nalgebra::Isometry3;
use reachy_kin::{neutral_head_pose, stow_head_pose};
use reachy_motion::postures::stow_pose_targets;
use scenario::check;
use scenario::check::head_pose_at;
use scenario::read::Run;

use scenario::{stow_clocks, up_cycles};

use s5_scenario::{
    RETARGET_AFTER, SCRIPT_ID, TURNAROUND_CYCLES, disengage_cycle, end_cycle, retarget_cycle,
    script_sent_cycle,
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
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        check::scripts_sent(run, &[(SCRIPT_ID, script_sent_cycle())], failures);
        let engaged = check::engagement(run, failures);
        check::ended_promptly(
            engaged.map(|engaged| engaged.released),
            disengage_cycle(),
            failures,
        );
        // The goal stream is where a retarget breaks if it breaks at all: a gap
        // in it, an instant out of order, or a step past what the plant can
        // travel are all this scenario's failure modes, and all three are
        // asserted for every cycle of the run rather than only around the
        // turnaround.
        if let (Some(stream), Some(engaged)) = (check::goal_stream(run, failures), engaged) {
            check::stream_starts_with_session(&stream, engaged.taken, failures);
            check::stream_stops_with_release(&stream, engaged.released, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        check_mid_move(run, failures);
        check_closing(run, failures);
        check_arrival(run, failures);
        check::no_faults(run, failures);
        check::no_events(run, failures);
        // Exactly this narration and nothing else: a refused script or a fault
        // read off a health rotation fails here rather than passing unseen.
        // There is no report of the retarget itself -- a step handing over to
        // the next is the schedule being carried out rather than news about it.
        check::narration(
            run,
            &[
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::SCRIPT_ACCEPTED,
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::SCHEDULE_PUBLISHED,
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::SCHEDULE_PUBLISHED,
                ReportKindWire::SESSION_ENDED,
                ReportKindWire::PHASE_CHANGED,
            ],
            failures,
        );
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
    // The head group's configured clock, deliberately: this guard is about the
    // head pose being between the two postures, and the shorter figure makes it
    // stricter. The antennas' own floored clock runs past it and asserting
    // against that would loosen the window a retarget has to land inside.
    let move_cycles = up_cycles();
    if RETARGET_AFTER >= move_cycles {
        failures.push(format!(
            "the posture changes {RETARGET_AFTER} cycles into a raise given {move_cycles}: the \
             scenario does not change the posture during a move"
        ));
    }
    let at = retarget_cycle();
    let Some(found) = head_pose_at(run, at) else {
        failures.push(format!("no pose for cycle {at}, where the posture changed"));
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
                "at cycle {at} the head is {offset} m from {what}, out of the {span} m \
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
    let from = retarget_cycle() + TURNAROUND_CYCLES;
    let Some(start) = head_pose_at(run, from) else {
        failures.push(format!(
            "no pose for cycle {from}, where the machine should have turned round"
        ));
        return;
    };
    let mut previous = apart(&start, &stow);
    for cycle in from + 1..disengage_cycle() {
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
        disengage_cycle() - 1,
        &stow_pose_targets(),
        failures,
    );
    check::room(
        "stow",
        disengage_cycle() - retarget_cycle(),
        &stow_clocks(),
        failures,
    );
}

/// How far apart two head positions are, metres.
fn apart(found: &Isometry3<f64>, wanted: &Isometry3<f64>) -> f64 {
    (found.translation.vector - wanted.translation.vector).norm()
}
