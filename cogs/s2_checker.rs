//! S2's assertions, over the output log.
//!
//! Three arguments: the output log directory and the two config textprotos the
//! process ran against. What S2 shares with a healthy run is asserted by
//! `scenario::check`; what is here is the obstruction itself -- that it was
//! reported, that it was reported as an event and not as a condition, that the
//! machine held rather than parked, and that the session carried on afterwards.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__cogs__msgs_clk_rs::FaultKind;
use reachy_motion::default_motion_config;
use reachy_motion::postures::stow_pose_targets;
use scenario::check;
use scenario::check::{ARRIVAL_TOLERANCE, goal_at, sample_at};
use scenario::cycle_of;
use scenario::read::Run;

use s2_scenario::{
    DISENGAGE_CYCLE, END_CYCLE, ENGAGE_CYCLE, OBSTRUCT_CYCLE, RELEASE_CYCLE, STOW_START_CYCLE,
    jammed_rows, stow_cycles,
};

fn main() -> ExitCode {
    check::main("s2_checker", |run, failures| {
        check::heartbeat(run, END_CYCLE, failures);
        check::readings_present(run, failures);
        if let Some(stream) = check::goal_stream(run, failures) {
            check::stream_covers_session(&stream, ENGAGE_CYCLE, DISENGAGE_CYCLE, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);

        check_jam_held(run, failures);
        let first_raise = check_reports(run, failures);
        check_hold(run, first_raise, failures);
        check_recovery(run, failures);
        // The keep-alive is what keeps the dead-man fed, and a machine that
        // stopped publishing because it had a fault to report would show up
        // here as a torque-off nobody asked for.
        check::no_events(run, failures);
        check::signal_groups(run, failures);
    })
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
    let from = OBSTRUCT_CYCLE;
    let Some(held) = sample_at(run, from) else {
        failures.push(format!(
            "no sample for cycle {from}, where the cranks are jammed"
        ));
        return;
    };
    for cycle in from..RELEASE_CYCLE {
        let Some(sample) = sample_at(run, cycle) else {
            failures.push(format!("no sample for cycle {cycle}, inside the jam"));
            return;
        };
        for joint in jammed_rows().iter() {
            let Some(row) = joint.index() else {
                failures.push(format!("{joint} sits on no bus row"));
                continue;
            };
            if sample.present[row] != held.present[row] {
                failures.push(format!(
                    "at cycle {cycle} the jammed {joint} reads {}, having stood at {} when the jam \
                     settled: the plant let a jammed row move",
                    sample.present[row], held.present[row]
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
        if fault.message.kind() != FaultKind::HEAD_OBSTRUCTED {
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
            Some(joint) if !jammed_rows().contains(joint) => failures.push(format!(
                "the report at cycle {cycle} names {joint}, and the hand in this scenario is on \
                 {}",
                jammed_rows()
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
            RELEASE_CYCLE - OBSTRUCT_CYCLE
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
    for cycle in &raises {
        if *cycle < OBSTRUCT_CYCLE || *cycle > RELEASE_CYCLE + window {
            failures.push(format!(
                "a jam was reported at cycle {cycle}, outside the {OBSTRUCT_CYCLE}..\
                 {RELEASE_CYCLE} the cranks were held over"
            ));
        }
    }
    first
}

/// The machine holds under command: from the report onwards the goal stream
/// carries on, and it carries on saying the same thing.
///
/// A jammed head does not latch, so the tick abandons the move and holds. What
/// holding means on the wire is a fresh datagram every cycle carrying the
/// setpoint the machine is already on -- the goal stream is the machine's
/// liveness, not its news, and a loop that fell silent because it had nothing
/// new to say would be de-torqued for it.
fn check_hold(run: &Run, first_raise: Option<i64>, failures: &mut Vec<String>) {
    let Some(from) = first_raise else {
        return;
    };
    let Some(held) = goal_at(run, from) else {
        failures.push(format!(
            "no goal for cycle {from}, where the machine dropped its move and held"
        ));
        return;
    };
    for cycle in from..STOW_START_CYCLE {
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
    check_released(run, &held, failures);
}

/// The jam really was released: by the end of the upright step the cranks have
/// closed on the setpoint they were held against.
///
/// The interval between the release and the next step is the only one that says
/// the hand came off. Without it a plant or a gate that left the rows stuck
/// would be caught, if at all, by the stow arrival much later and only when the
/// stow move was too short to recover from it.
fn check_released(run: &Run, held: &[f64; 9], failures: &mut Vec<String>) {
    let at = STOW_START_CYCLE - 1;
    let Some(sample) = sample_at(run, at) else {
        failures.push(format!(
            "no sample for cycle {at}, where the released cranks should have caught up"
        ));
        return;
    };
    for joint in jammed_rows().iter() {
        let Some(row) = joint.index() else {
            failures.push(format!("{joint} sits on no bus row"));
            continue;
        };
        let error = (sample.present[row] - held[row]).abs();
        if error > ARRIVAL_TOLERANCE {
            failures.push(format!(
                "at cycle {at} the released {joint} is {error} rad from the setpoint the machine \
                 held it against through the jam: the release left it stuck"
            ));
        }
    }
}

/// The session carries on: released, the machine takes the stow step and gets
/// there.
///
/// The jam is over well before the step that sends it home, so nothing about
/// this differs from S1 -- which is the assertion. A fault that quietly cost
/// the machine its next command would show up nowhere else.
fn check_recovery(run: &Run, failures: &mut Vec<String>) {
    check::arrived_at(
        run,
        "stowed",
        DISENGAGE_CYCLE - 1,
        &stow_pose_targets(),
        failures,
    );
    check::room(
        "stow",
        DISENGAGE_CYCLE - STOW_START_CYCLE,
        stow_cycles(),
        failures,
    );
}
