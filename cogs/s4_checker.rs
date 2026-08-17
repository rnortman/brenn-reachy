//! S4's assertions, over the output log.
//!
//! Three arguments: the output log directory and the two config textprotos the
//! process ran against. What S4 shares with a healthy run is asserted by
//! `scenario::check`; what is here is the chain the outage sets off -- the
//! samples that carry no reading, the one report, the goal stream ending with
//! it, the gate de-torquing the machine, and the machine standing still through
//! all of it.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__cogs__msgs_clk_rs::{FaultKind, JointRef};
use reachy_motion::postures::neutral_targets;
use reachy_wire::EventKind;
use scenario::check;
use scenario::cycle_of;
use scenario::read::Run;

use s4_scenario::{
    ENGAGE_CYCLE, OUTAGE_CYCLE, OUTAGE_CYCLES, end_cycle, fault_cycle, latch_cycle,
    reported_misses, reported_silence_us, up_cycles,
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

        check_the_reads_come_back_first(failures);
        check_arrived_before_the_outage(run, failures);
        check_outage(run, failures);
        check_estimates(run, failures);
        check_report(run, failures);
        check_stream_stops(run, failures);
        let latched = check::sole_event(
            run,
            EventKind::HoldTimeoutTorqueOff,
            latch_cycle(),
            reported_silence_us(),
            failures,
        );
        check::latch_from(run, latched, failures);
        // It had arrived and was holding when the reads went, so there was
        // nowhere left to travel; then the tick stopped commanding it, and then
        // the gate took its torque away.
        check::stands_still(
            run,
            OUTAGE_CYCLE,
            end_cycle(),
            "holding, then uncommanded, then de-torqued",
            failures,
        );
        check::signal_groups(run, failures);
    })
}

/// The scenario still describes the run it claims to: the reads come back after
/// the tick has given up on them and before the gate takes the torque away.
///
/// Three numbers this file does not own decide that ordering -- how many misses
/// the tick tolerates, how long the gate's window is, and how long a cycle is --
/// and a one-cycle move in any of them flips it. If the outage outlasted the
/// latch, "nothing recovers when the reads come back" would stop being tested at
/// all while every assertion below still passed, because they are all derived
/// from the same shifted arithmetic. A hollowed scenario is worse than a missing
/// one, so the ordering is asserted rather than described.
fn check_the_reads_come_back_first(failures: &mut Vec<String>) {
    let back = OUTAGE_CYCLE + i64::from(OUTAGE_CYCLES);
    if back <= fault_cycle() {
        failures.push(format!(
            "the reads come back at cycle {back} and the tick gives up on them at cycle {}: this \
             scenario is about a loop that latched, and this outage ends before it would",
            fault_cycle()
        ));
    }
    if back >= latch_cycle() {
        failures.push(format!(
            "the reads come back at cycle {back} and the gate de-torques the machine at cycle {}: \
             this scenario is about a loop that does not recover when they do, and this outage \
             outlasts the gate",
            latch_cycle()
        ));
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
        OUTAGE_CYCLE - 1,
        &neutral_targets(),
        failures,
    );
    check::room(
        "upright",
        OUTAGE_CYCLE - ENGAGE_CYCLE,
        up_cycles(),
        failures,
    );
}

/// The outage is exactly where the scenario put it: every sample inside the
/// window says the bus answered for no row, and every sample outside it carries
/// a reading.
///
/// The window's own boundaries are the assertion. An injection takes effect on
/// the cycle it names -- the driver drains what arrived and then advances the
/// plant -- so the first blind sample is the one for the cycle the outage was
/// published on, and the reads are back on the cycle after the last of them.
fn check_outage(run: &Run, failures: &mut Vec<String>) {
    let blind = OUTAGE_CYCLE..OUTAGE_CYCLE + i64::from(OUTAGE_CYCLES);
    for sample in &run.samples {
        let sample = &sample.message.message;
        let Ok(cycle) = cycle_of(sample.nominal_time_ns) else {
            continue;
        };
        let wanted = blind.contains(&cycle);
        let dark = !sample.present_valid;
        if dark != wanted {
            failures.push(format!(
                "the sample at cycle {cycle} says its reading is {}, and the outage runs over \
                 cycles {}..{}",
                if dark { "missing" } else { "present" },
                blind.start,
                blind.end
            ));
            return;
        }
        // A driver that read nothing says so twice: the flag and the mask of
        // the rows it did not hear from. A sample carrying one without the
        // other is a receiver's choice about which to believe.
        let masked = sample.miss_mask != 0;
        if masked != dark {
            failures.push(format!(
                "the sample at cycle {cycle} carries miss mask {:#b} and a validity flag of {}: \
                 the two say different things about the same reading",
                sample.miss_mask, sample.present_valid
            ));
            return;
        }
    }
}

/// The estimator says so too: the pose series carries an invalid estimate for
/// every cycle of the outage and a valid one for every cycle outside it.
///
/// The series is what a consumer downstream of this loop reads, and staleness
/// it cannot see is worse than a gap it can. There is no gap -- one estimate per
/// sample is asserted with the heartbeat -- so what is left to say is that each
/// one tells the truth about the reading behind it.
fn check_estimates(run: &Run, failures: &mut Vec<String>) {
    let blind = OUTAGE_CYCLE..OUTAGE_CYCLE + i64::from(OUTAGE_CYCLES);
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

/// The report: one message, of that kind, on the cycle the tolerance puts it
/// on, naming the run of misses it counted.
///
/// One is the whole point. The fault latches, so the tick re-reports it on every
/// cycle from here to the end of the run -- fifty of them -- and a channel
/// carrying that is a channel nobody can read. What reaches the log is the
/// transition, once.
fn check_report(run: &Run, failures: &mut Vec<String>) {
    let mut seen = false;
    for fault in &run.faults {
        let at = fault.message.time().as_nanos();
        let cycle = match cycle_of(at) {
            Ok(cycle) => cycle,
            Err(complaint) => {
                failures.push(format!("a report is not on the grid: {complaint}"));
                continue;
            }
        };
        if fault.message.kind() != FaultKind::POSITION_FEEDBACK_LOST {
            failures.push(format!(
                "the decision tick reported {:?} at cycle {cycle}, and the only thing this \
                 scenario does to the machine is stop reading it",
                fault.message.kind()
            ));
            continue;
        }
        if seen {
            failures.push(format!(
                "the tick reported the loss again at cycle {cycle}: it latches, so every cycle \
                 after the raise re-reports a standing fault, and a standing fault is not news"
            ));
            continue;
        }
        seen = true;
        if cycle != fault_cycle() {
            failures.push(format!(
                "the tick reported the loss at cycle {cycle}, and the run of misses passes what it \
                 tolerates at cycle {}",
                fault_cycle()
            ));
        }
        if fault.message.joint() != JointRef::NONE {
            failures.push(format!(
                "the report at cycle {cycle} names {:?}, and a bus that answered for nothing is \
                 not about one servo",
                fault.message.joint()
            ));
        }
        if fault.message.count() != reported_misses() {
            failures.push(format!(
                "the report at cycle {cycle} counts {} missed reads, and the raise is on the miss \
                 numbered {}",
                fault.message.count(),
                reported_misses()
            ));
        }
    }
    if !seen {
        failures.push(format!(
            "the tick never reported the loss, and the bus answered for nothing over \
             {OUTAGE_CYCLES} cycles from cycle {OUTAGE_CYCLE}"
        ));
    }
}

/// The goal stream ends with the report and never starts again.
///
/// This is the fault response in this slice: there is no sequencer to run a
/// stow ladder, and there does not need to be one -- a latched tick commands
/// nothing, and what covers the machine is the gate behind it. So the last goal
/// is the one decided on the cycle before the raise, and the reads coming back
/// afterwards does not bring the stream back with them.
fn check_stream_stops(run: &Run, failures: &mut Vec<String>) {
    let Some(stream) = check::goal_stream(run, failures) else {
        return;
    };
    check::stream_starts_with_session(&stream, ENGAGE_CYCLE, failures);
    let last = fault_cycle() - 1;
    if stream.last_cycle != last {
        failures.push(format!(
            "the goal stream runs to cycle {}, and the tick latched on cycle {}: the last goal is \
             the one decided on the cycle before the raise, and nothing commands the machine after \
             it -- not even the reads coming back",
            stream.last_cycle,
            fault_cycle()
        ));
    }
}
