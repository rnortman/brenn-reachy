//! S10's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What S10 shares with S4 -- the outage, the driver's own
//! evidence, the park, the latch on the wire -- is asserted with the same
//! helpers; what is here is the half a bus that comes back cannot show: a
//! release nobody ever acknowledges, said to be unacknowledged, and commanded
//! again on every wake to the end of the run.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__cogs__session_cmd_clk_rs::SessionCmdKindWire;
use brenn_reachy__driver__health_clk_rs::EventKind;
use brenn_reachy__motion__faults_clk_rs::{FaultKindWire, ResponseKindWire};
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use reachy_motion::postures::neutral_targets;
use reachy_motion::tick::ResponseKind;
use scenario::check;
use scenario::read::Run;
use scenario::{
    DRIVER_CONFIRM_BUDGET_NS, SESSION_WAKE_FLOOR_NS, cycle_within, cycles_for, up_clocks,
};

use s10_scenario::{
    BUDGETS_AFTER_RELEASE, SCRIPT_ID, bus_failure_cycle, confirm_budget_cycles, end_cycle,
    outage_cycle, outage_cycles, release_cycle, script_sent_cycle, up_start_cycle,
};

fn main() -> ExitCode {
    check::main("s10_checker", |run, failures| {
        // The clock ran to the end: a driver that reads nothing still publishes
        // a sample saying so, and a driver that has de-torqued the machine
        // still publishes one.
        check::heartbeat(run, end_cycle(), failures);
        check::estimates_per_sample(run, failures);
        check::scripts_sent(run, &[(SCRIPT_ID, script_sent_cycle())], failures);
        check_the_bus_never_comes_back(failures);

        // Four phase changes and no more, ending parked: a fifth would be a
        // session that did something with a machine it had let go of.
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
        check::schedules_published(run, engaged, failures);
        check_arrived_before_the_outage(run, failures);

        // The driver's own account of the bus, once: the outage outlasts what a
        // stuttering bus looks like, and a standing outage is not news.
        check::sole_event(run, EventKind::BusFailure, bus_failure_cycle(), 0, failures);
        check::latch_from(run, Some(release_cycle()), failures);
        // And the read-back could read nothing, so it credited nothing and said
        // so once its own budget was spent. The one thing this driver must never
        // do is confirm a de-torquing to silence, and the absence of a
        // confirmation over the whole run is what says it did not.
        check::sole_event(
            run,
            EventKind::TorqueOffUnconfirmed,
            release_cycle() + cycles_for(DRIVER_CONFIRM_BUDGET_NS) + 1,
            0,
            failures,
        );
        check::only_kinds(
            run,
            // No dropped goal among them, which is an assertion rather than a
            // tolerance: the session commands the release on the wake it is
            // told about the bus, the mover reads the cleared schedule a cycle
            // later, and a couple of goals refused by a latched gate do not
            // fill its queue. A drop here would be a stream that went on being
            // decided for a machine nobody was commanding.
            &[EventKind::BusFailure, EventKind::TorqueOffUnconfirmed],
            failures,
        );

        check_the_release_never_stops(run, failures);
        check_the_session_says_it_is_unacknowledged(run, failures);
        check_the_narration(run, failures);
        // The two conditions of this run, each recorded once and each about the
        // bus rather than about a servo: the driver's account of a bus that
        // stopped carrying, and the release its own read-back could not credit.
        // Both name no single row, which is what a condition of the whole bus
        // carries, and a second record of either would be a session recording a
        // standing condition at the rate it is told about it.
        check::faults_recorded(
            run,
            &[
                check::Expected {
                    kind: FaultKindWire::BUS_FAILURE,
                    rows: JointFlags::NONE,
                    from: bus_failure_cycle(),
                    through: bus_failure_cycle(),
                    how_many: check::Recorded::Times(1),
                    raised_by_tick: false,
                    why: "the driver's own account of a bus that stopped carrying",
                },
                check::Expected {
                    kind: FaultKindWire::TORQUE_OFF_UNCONFIRMED,
                    rows: JointFlags::NONE,
                    from: release_cycle(),
                    through: end_cycle(),
                    how_many: check::Recorded::Times(1),
                    raised_by_tick: false,
                    why: "the release the driver's read-back could not credit",
                },
            ],
            failures,
        );
        check::no_faults(run, failures);
        // The bus answers for no row from the outage to the end and never comes
        // back, which is what makes every absence below an absence rather than a
        // window nobody looked at. Named a cycle past the run's end, because that
        // is how far the injection was counted.
        check::outage(
            run,
            outage_cycle()..outage_cycle() + i64::from(outage_cycles()),
            failures,
        );
        // Nothing is asserted about where the machine stood after that. Every
        // sample from the outage on carries no positions at all, so this run has
        // no measurement of the plant to make a claim from: where it was when the
        // reads went is `check_arrived_before_the_outage`'s, and everything after
        // it is about what the session and the driver did rather than about the
        // machine.
        check::signal_groups(run, failures);
    })
}

/// The scenario still describes the run it claims to: the outage covers the rest
/// of the run, and the run carries several of the session's confirmation budgets
/// past the release.
///
/// Both are arithmetic over numbers this file does not own -- how many blind
/// cycles the driver forgives, how long the session's budget is, how long a
/// cycle is -- and a move in any of them can hollow the run out. A bus that came
/// back before the end would confirm the release, and every assertion below
/// about a release nobody acknowledged would be asserting nothing while still
/// passing.
fn check_the_bus_never_comes_back(failures: &mut Vec<String>) {
    if BUDGETS_AFTER_RELEASE < 2 {
        failures.push(format!(
            "the run carries {BUDGETS_AFTER_RELEASE} of the session's confirmation budgets past \
             the release: one is a session that said so once, which is what a session that gave \
             up looks like"
        ));
    }
    if release_cycle() + confirm_budget_cycles() >= end_cycle() {
        failures.push(format!(
            "the release is commanded on cycle {} and the run ends on {}: the budget that bounds \
             the reporting is {} cycles, so this run ends before the reporting begins",
            release_cycle(),
            end_cycle(),
            confirm_budget_cycles()
        ));
    }
}

/// The machine was upright and holding when the reads went.
///
/// Asserted because everything after it is about a loop that lost a machine it
/// had: a run where the outage happened to fall during a move would be reporting
/// a different neighbourhood.
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

/// The release is commanded again on every wake, from the cycle it was first
/// commanded on to the end of the run.
///
/// This is the doctrine, on the wire: nothing gates de-torquing, so a machine
/// that may still be holding is asked again for as long as the process lives.
/// Asserted as a cadence rather than as a count -- the wake floor is what puts
/// an upper bound on the gap between two of them, and the last one has to land
/// within a wake of the run's end, or the session stopped asking at some point
/// this file would otherwise never notice.
///
/// And it is the only thing the session says to the driver from there on. A
/// keep-alive would be a wake with nothing to say about a machine that is holding
/// torque, and there is no such wake in this run: every one of them owes the
/// release.
fn check_the_release_never_stops(run: &Run, failures: &mut Vec<String>) {
    let floor = cycles_for(SESSION_WAKE_FLOOR_NS);
    let mut previous: Option<i64> = None;
    for datagram in &run.datagrams {
        let cycle = cycle_within(datagram.at_ns);
        if cycle < release_cycle() {
            continue;
        }
        let kind = datagram.message.kind();
        if kind != SessionCmdKindWire::TORQUE_OFF_NOW {
            failures.push(format!(
                "the session published a {kind:?} datagram on cycle {cycle}, past the release it \
                 commanded on {}: a wake that owes a de-torquing has nothing else to say",
                release_cycle()
            ));
            return;
        }
        if let Some(previous) = previous
            && cycle - previous > floor
        {
            failures.push(format!(
                "the session commanded the release on cycles {previous} and {cycle}, further apart \
                 than its {floor}-cycle wake floor: a release nobody acknowledged is commanded \
                 every wake"
            ));
            return;
        }
        previous = Some(cycle);
    }
    let Some(last) = previous else {
        failures.push(format!(
            "the session commanded no release from cycle {} on, and this run is about the one it \
             goes on commanding",
            release_cycle()
        ));
        return;
    };
    if last + floor < end_cycle() {
        failures.push(format!(
            "the session last commanded the release on cycle {last} and the run ends on {}: it \
             stopped asking for a de-torquing nothing had acknowledged",
            end_cycle()
        ));
    }
}

/// The session said it had no acknowledgement, once per budget, for as long as
/// the release stood.
///
/// The budget bounds the *saying* and not the asking, which is the distinction
/// this run exists to pin: a session that stopped commanding when it started
/// complaining would be gating a de-torquing on being able to confirm it. So the
/// count is the run's own arithmetic -- one saying per budget over the stretch
/// the release stands for -- and the first one lands a budget after the release
/// rather than with it.
fn check_the_session_says_it_is_unacknowledged(run: &Run, failures: &mut Vec<String>) {
    let said: Vec<i64> = run
        .reports
        .iter()
        .filter(|report| report.message.kind() == ReportKindWire::TORQUE_OFF_UNCONFIRMED)
        .map(|report| cycle_within(report.message.time().as_nanos()))
        .collect();
    let budget = confirm_budget_cycles();
    if said.len() < usize::try_from(BUDGETS_AFTER_RELEASE - 1).unwrap_or(0) {
        failures.push(format!(
            "the session said the release was unacknowledged on cycles {said:?}, and the run \
             carries {BUDGETS_AFTER_RELEASE} of its {budget}-cycle budgets past the release on \
             cycle {}",
            release_cycle()
        ));
    }
    let mut previous = release_cycle();
    for at in &said {
        if at - previous < budget {
            failures.push(format!(
                "the session said the release was unacknowledged on cycle {at}, {} cycles after \
                 the last word about it: the budget it is said once per is {budget} cycles",
                at - previous
            ));
            return;
        }
        previous = *at;
    }
}

/// The whole of what the session said, and the rung it said it took.
///
/// The doctrine's answer to a machine nothing can be commanded through is the
/// immediate best-effort torque-off, and to a bus the driver itself has given up
/// on it is the park-class one: an operator has to have been before anything
/// engages again. That selection is what this run is about, so it is asserted
/// rather than inferred from the datagrams it produced -- a session that
/// commanded a release under some other rung's name would be a ladder saying one
/// thing and doing another.
///
/// And nothing else is narrated. Every kind this run has business producing is
/// listed, so one it has not -- a wind-down that concluded, a script accepted
/// after the park, a second response to the same machine -- fails rather than
/// passing unseen. How many times the session says the release is unacknowledged
/// is arithmetic over the run's length and is asserted by cadence above.
fn check_the_narration(run: &Run, failures: &mut Vec<String>) {
    let mut answers = Vec::new();
    for report in &run.reports {
        match report.message.kind() {
            ReportKindWire::PHASE_CHANGED
            | ReportKindWire::SCRIPT_ACCEPTED
            | ReportKindWire::SCHEDULE_PUBLISHED
            | ReportKindWire::FAULT_RECORDED
            | ReportKindWire::TORQUE_OFF_UNCONFIRMED => {}
            ReportKindWire::RESPONSE_TAKEN => answers.push((
                cycle_within(report.message.time().as_nanos()),
                report.message.a(),
            )),
            other => failures.push(format!(
                "the session narrated {other:?} at {}, and this run is a machine let go of and \
                 parked over a bus that never came back",
                report.message.time().as_nanos()
            )),
        }
    }
    let park = u32::from(ResponseKindWire::from(ResponseKind::ImmediateAllTorqueOffToPark).0);
    let [(at, response)] = answers.as_slice() else {
        failures.push(format!(
            "the session selected responses {answers:?}: a bus the driver has given up on is \
             answered once, and every condition after it is a condition the standing response \
             already answers"
        ));
        return;
    };
    if *response != park {
        failures.push(format!(
            "the session selected response {response} at cycle {at}, and a machine nothing can be \
             commanded through is answered by the immediate best-effort torque-off to park \
             ({park}): every other rung asks the bus for something"
        ));
    }
    if *at != bus_failure_cycle() {
        failures.push(format!(
            "the session selected its response on cycle {at}, and the driver published the \
             evidence on cycle {}: an edge waits for no wake floor",
            bus_failure_cycle()
        ));
    }
}
