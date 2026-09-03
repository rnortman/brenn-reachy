//! S4's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What S4 shares with a healthy run is asserted by
//! `scenario::check`; what is here is the chain the outage sets off -- the
//! samples that carry no reading, the driver's own evidence, the session
//! parking and commanding the release, the driver confirming it, the goals
//! dropped at a gate that has already let go, the goal stream ending with the
//! disengagement, and the machine standing still through all of it.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__driver__health_clk_rs::EventKind;
use reachy_motion::postures::neutral_targets;
use scenario::check;
use scenario::cycle_of;
use scenario::read::Run;

use scenario::up_clocks;

use s4_scenario::{
    OUTAGE_CYCLES, SCRIPT_ID, bus_failure_cycle, end_cycle, fault_cycle, outage_cycle,
    reads_back_cycle, release_cycle, script_sent_cycle, up_start_cycle,
};

fn main() -> ExitCode {
    check::main("s4_checker", |run, failures| {
        // The heartbeat runs to the end whatever else happened: a driver that
        // reads nothing still publishes a sample saying so, and a driver that
        // has de-torqued the machine still publishes one. A hole here would be
        // the clock stopping, which every other assertion in this file is
        // measured against.
        check::heartbeat(run, end_cycle(), failures);
        check::estimates_per_sample(run, failures);
        check::scripts_sent(run, &[(SCRIPT_ID, script_sent_cycle())], failures);

        // First, because the rest is measured against the cycles these land on.
        // Four changes and no more -- a fifth would be a session that did
        // something with the reads coming back.
        let engaged = check::parked_life(run, failures);
        if let Some(engaged) = engaged
            && engaged.released != bus_failure_cycle()
        {
            failures.push(format!(
                "the session parked on cycle {}, and the evidence it answered was published on \
                 cycle {}: an edge waits for no wake floor",
                engaged.released,
                bus_failure_cycle()
            ));
        }
        // The disengagement goes out on the same wake the machine was parked on:
        // a machine that must let go is not one to go on commanding.
        check::schedules_published(run, engaged, failures);

        check_the_reads_come_back_first(failures);
        check_arrived_before_the_outage(run, failures);
        check_outage(run, failures);
        check_estimates(run, failures);
        // And the tick reported nothing at all. The session's answer to the
        // driver's evidence takes the machine out from under it long before its
        // own tolerance for missed reads runs out, so the loop never gives up on
        // a machine it has already been told it is not commanding. Which means
        // no scenario covers the other order, where the tick's own latch is what
        // ends the stream.
        // TODO(tick-feedback-latch-composed)
        check::no_faults(run, failures);
        let disengaged = check_stream_stops(run, engaged, failures);
        // The driver said, on its own account, that its bus had stopped
        // answering: the outage outlasts what a stuttering bus looks like, and a
        // driver that read nothing for that long is the one fault it can raise
        // about itself. Once, whatever the reads do afterwards.
        check::sole_event(run, EventKind::BusFailure, bus_failure_cycle(), 0, failures);
        // The session answered it: the machine is released on the next cycle and
        // every sample from there says the latch stands. The read-back cannot
        // land while the bus is answering nothing, so the driver says it cannot
        // confirm the release and then confirms it once the reads come back --
        // the one thing it must never do is credit a de-torquing to silence. The
        // dead-man never comes into it -- there is no `HoldTimeoutTorqueOff` in
        // the kinds below -- because the release was commanded before the goal
        // stream stopped.
        check::latch_from(run, Some(release_cycle()), failures);
        check::confirmed_off_when_the_bus_returns(
            run,
            release_cycle(),
            reads_back_cycle(),
            failures,
        );
        check_dropped_goals(run, disengaged, failures);
        check::only_kinds(
            run,
            &[
                EventKind::BusFailure,
                EventKind::TorqueOffUnconfirmed,
                EventKind::TorqueOffConfirmed,
                EventKind::GoalDroppedQueueFull,
            ],
            failures,
        );
        // It had arrived and was holding when the reads went, so there was
        // nowhere left to travel; then the session let go of it, and then the
        // gate took its torque away. Measured from the last sample anybody could
        // read: the samples of the outage itself carry no positions, so what
        // this compares is where the machine was before the reads went against
        // where it is when they come back.
        check::stands_still(
            run,
            outage_cycle() - 1,
            end_cycle(),
            "holding, then uncommanded, then de-torqued",
            failures,
        );
        check::signal_groups(run, failures);
    })
}

/// The scenario still describes the run it claims to: the reads come back after
/// both observers have given up on the bus and after the machine has been
/// released, and the run outlasts the tick's own latch.
///
/// Numbers this file does not own decide that ordering -- how many misses the
/// tick tolerates, how many blind cycles the driver forgives, and how long a
/// cycle is -- and a move in any of them can flip it. If the outage ended before
/// the evidence, "nothing recovers when the reads come back" would stop being
/// tested at all while every assertion below still passed, because they are all
/// derived from the same shifted arithmetic. A hollowed scenario is worse than a
/// missing one, so the ordering is asserted rather than described.
fn check_the_reads_come_back_first(failures: &mut Vec<String>) {
    let back = reads_back_cycle();
    if fault_cycle() <= bus_failure_cycle() {
        failures.push(format!(
            "the tick would give up on the reads at cycle {} and the driver calls its bus gone at \
             cycle {}: this scenario is about the driver noticing first, and the session taking \
             the machine out from under a tick that never gave up",
            fault_cycle(),
            bus_failure_cycle()
        ));
    }
    if back <= bus_failure_cycle() {
        failures.push(format!(
            "the reads come back at cycle {back} and the driver calls its bus gone at cycle {}: \
             this scenario carries the driver's own evidence as well as the tick's, and this \
             outage ends before it",
            bus_failure_cycle()
        ));
    }
    if back <= release_cycle() {
        failures.push(format!(
            "the reads come back at cycle {back} and the machine is released at cycle {}: this \
             scenario is about a machine that stays parked when they do, and this outage ends \
             before the release",
            release_cycle()
        ));
    }
}

/// The goals the gate dropped are exactly the ones nobody could have executed.
///
/// Every one of them falls between the release and the last goal the mover
/// published, which is the whole of the window: the machine has let go, and the
/// mover has not yet read the schedule saying the session is over, so for a cycle
/// or so it goes on commanding a gate that refuses to write. Asserted as a
/// window rather than counted, because what matters is that no goal was dropped
/// while the machine could still have taken one -- a drop before the release
/// would be a queue overrunning under a working gate, which is a different fault
/// entirely.
fn check_dropped_goals(run: &Run, disengaged: Option<i64>, failures: &mut Vec<String>) {
    let Some(disengaged) = disengaged else {
        return;
    };
    for event in &run.events {
        if event.message.kind().to_known() != Some(EventKind::GoalDroppedQueueFull) {
            continue;
        }
        let Ok(cycle) = cycle_of(event.message.time().as_nanos()) else {
            continue;
        };
        if cycle < release_cycle() || cycle > disengaged {
            failures.push(format!(
                "the gate dropped a goal on cycle {cycle}, and the window in which nobody could \
                 have executed one runs from cycle {} to cycle {disengaged}",
                release_cycle()
            ));
        }
    }
}

/// The machine was upright and holding when the reads went.
///
/// Asserted because everything after it is about a loop that lost a machine it
/// had, and a run where the raise happened to fall during a move would be
/// reporting a different fault's neighbourhood.
fn check_arrived_before_the_outage(run: &Run, failures: &mut Vec<String>) {
    check::arrived_at(
        run,
        "upright",
        outage_cycle() - 1,
        &neutral_targets(),
        failures,
    );
    check::room(
        "upright",
        outage_cycle() - up_start_cycle(),
        &up_clocks(),
        failures,
    );
}

/// The outage is exactly where the scenario put it, on [`check::outage`]'s
/// terms: the window is this scenario's, and the reads come back at the end of
/// it, which is what S10 -- the same outage, never ending -- cannot say.
fn check_outage(run: &Run, failures: &mut Vec<String>) {
    let blind = outage_cycle()..outage_cycle() + i64::from(OUTAGE_CYCLES);
    check::outage(run, blind, failures);
}

/// The estimator says so too: the pose series carries an invalid estimate for
/// every cycle of the outage and a valid one for every cycle outside it.
///
/// The series is what a consumer downstream of this loop reads, and staleness
/// it cannot see is worse than a gap it can. There is no gap -- one estimate per
/// sample is asserted with the heartbeat -- so what is left to say is that each
/// one tells the truth about the reading behind it.
fn check_estimates(run: &Run, failures: &mut Vec<String>) {
    let blind = outage_cycle()..outage_cycle() + i64::from(OUTAGE_CYCLES);
    let mut invalid = 0;
    for estimate in &run.estimates {
        let at = estimate.message.time_of_validity().as_nanos();
        let Ok(cycle) = cycle_of(at) else {
            failures.push(format!(
                "an estimate is valid at {at}, which is off the grid"
            ));
            continue;
        };
        let wanted = !blind.contains(&cycle);
        if estimate.message.valid() != wanted {
            failures.push(format!(
                "the estimate for cycle {cycle} reports valid = {}, and the reading behind it is \
                 {}",
                estimate.message.valid(),
                if wanted { "whole" } else { "missing" }
            ));
            return;
        }
        if !estimate.message.valid() {
            invalid += 1;
        }
    }
    if invalid != i64::from(OUTAGE_CYCLES) {
        failures.push(format!(
            "{invalid} estimates found no pose over an outage of {OUTAGE_CYCLES} cycles"
        ));
    }
}

/// The goal stream ends with the disengagement and never starts again.
///
/// This is the fault response reaching the cog that commands: the session parks
/// the machine, publishes a schedule nobody is engaged on, and the mover drops
/// its engagement on the next sample it reads. So the last goal is the one
/// decided within a cycle of that schedule going out, and the reads coming back
/// afterwards does not bring the stream back with them.
///
/// The cycle the stream ended on, for the dropped-goal window to close at: the
/// goals a latched gate refused are the ones between the release and this.
fn check_stream_stops(
    run: &Run,
    engaged: Option<check::Engaged>,
    failures: &mut Vec<String>,
) -> Option<i64> {
    let stream = check::goal_stream(run, failures)?;
    let engaged = engaged?;
    check::stream_starts_with_session(&stream, engaged.taken, failures);
    check::stream_stops_with_release(&stream, engaged.released, failures);
    Some(stream.last_cycle)
}
