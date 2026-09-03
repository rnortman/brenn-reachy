//! S13's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. Everything asserted here is read out of those.
//!
//! What is S13's among them is the answer each of the four scripts got and the
//! order of the two things that must not overlap: the release's last write and
//! the second engagement's ask. Every other property of a healthy run is
//! `scenario::check`'s, and they all still hold -- a run that answered every
//! script correctly and gapped the goal stream doing it is not the run this
//! scenario is for.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__cogs__session_cmd_clk_rs::SessionCmdKindWire;
use brenn_reachy__hardware__dynamixel__registers_clk_rs::RegIdWire;
use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKindWire;
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use brenn_reachy__motion__reports_clk_rs::RefusalReasonWire;
use reachy_motion::joints::ROW_COUNT;
use reachy_motion::postures::{neutral_targets, stow_pose_targets};
use scenario::check;
use scenario::read::Run;

use s13_scenario::{
    CLOSING_SCRIPT_ID, HELD_SCRIPT_ID, OPENING_SCRIPT_ID, closing_cycle, disengage_cycle,
    duplicate_cycle, end_cycle, script_sent_cycle, second_disengage_cycle, second_stow_start_cycle,
    stow_start_cycle,
};

fn main() -> ExitCode {
    check::main("s13_checker", |run, failures| {
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        // The four things the scenario said, in the order it said them: two at
        // one instant, and the last two the same number twice.
        check::scripts_sent(
            run,
            &[
                (OPENING_SCRIPT_ID, script_sent_cycle()),
                (HELD_SCRIPT_ID, script_sent_cycle()),
                (CLOSING_SCRIPT_ID, closing_cycle()),
                (CLOSING_SCRIPT_ID, duplicate_cycle()),
            ],
            failures,
        );
        // The first session's whole life, and then the second one the drained
        // script opens: the machine is engaged again, runs a schedule and is let
        // go of, so the run carries two of every phase an engagement has.
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
        let second_accepted = second.first().copied();
        let second_taken = second.get(1).copied();
        // Both engagements cost what a fresh picture costs: nothing on the bus,
        // and the phase entered within the ask and its answer. The second one
        // says it of an engagement opened by a drain rather than by an intake,
        // which is the path this scenario is about.
        if let Some(engaged) = engaged {
            check::engagement_cost(run, engaged.accepted, engaged.taken, failures);
        }
        if let (Some(accepted), Some(taken)) = (second_accepted, second_taken) {
            check::engagement_cost(run, accepted, taken, failures);
        }
        check::ended_promptly(
            engaged.map(|engaged| engaged.released),
            disengage_cycle(),
            failures,
        );
        // The two holds, and nothing else held: the script that shared the
        // opening script's instant was answered against the engagement that
        // acceptance opened, and the closing script against the release.
        check::holds(
            run,
            &[
                (
                    HELD_SCRIPT_ID,
                    SessionPhaseWire::ENGAGING,
                    script_sent_cycle(),
                ),
                (
                    CLOSING_SCRIPT_ID,
                    SessionPhaseWire::STOPPING,
                    closing_cycle(),
                ),
            ],
            failures,
        );
        // One refusal in the whole run, and it is about a number: a refusal
        // for any other reason would mean a phase answered a script with an
        // error instead of holding it.
        check::refusals(
            run,
            &[(
                CLOSING_SCRIPT_ID,
                RefusalReasonWire::STALE,
                duplicate_cycle(),
            )],
            failures,
        );
        // One stretch of goal stream per engagement, unbroken: the replacement
        // the first hold drained into changed the schedule without touching
        // torque, so the stream across it is one stretch and not two.
        let streams = check::goal_streams_exactly(run, JointFlags::NONE, 2, failures);
        if let (Some(stream), Some(engaged)) = (streams.first(), engaged) {
            check::stream_starts_with_session(stream, engaged.taken, failures);
            check::stream_stops_with_release(stream, engaged.released, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        check_arrival(run, failures);
        if let Some(rested) = engaged.and_then(|engaged| engaged.rested) {
            check_torque_off_before_the_second_ask(run, rested, failures);
        }
        check::no_faults(run, failures);
        // Nothing but the two engagements' own answers: a wake at an awkward
        // moment is answered by the session, and nothing about it reaches the
        // driver's gate.
        check::no_events(run, failures);
        check::signal_groups(run, failures);
    })
}

/// The machine arrives at what each session's schedule asked for: upright by the
/// end of the step that sends it there, and stowed by the end of the fold.
///
/// The first session's postures are the *held* script's, which is the whole
/// point of the drain: the schedule the machine ran is the one that was waiting
/// rather than the one the acceptance carried.
fn check_arrival(run: &Run, failures: &mut Vec<String>) {
    check::arrived_at(
        run,
        "upright",
        stow_start_cycle() - 1,
        &neutral_targets(),
        failures,
    );
    check::arrived_at(
        run,
        "stowed",
        disengage_cycle() - 1,
        &stow_pose_targets(),
        failures,
    );
    check::arrived_at(
        run,
        "upright again",
        second_stow_start_cycle() - 1,
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

/// Torque was fully off before the second engagement asked for it back.
///
/// The ordering that matters in this run. The held script is drained on the wake
/// the machine reaches rest, and that same wake takes the engagement's first bus
/// step -- so the ask goes out beside the release that has just concluded, and
/// what says the two are the right way round is the wire: every one of the nine
/// verified writes that take torque off precedes the datagram that asks for it
/// back, and the ask is published on the wake rest was entered on rather than on
/// some later one.
fn check_torque_off_before_the_second_ask(run: &Run, rested: i64, failures: &mut Vec<String>) {
    let released: Vec<i64> = run
        .datagrams
        .iter()
        .filter(|logged| {
            let txn = logged.message.txn();
            logged.message.kind() == SessionCmdKindWire::AUX
                && txn.op() == AuxOpKindWire::WRITE_REG_VERIFIED
                && txn.reg() == RegIdWire::TORQUE_ENABLE
                && txn.value() == 0
        })
        .map(|logged| logged.at_ns)
        .collect();
    let asks: Vec<i64> = run
        .datagrams
        .iter()
        .filter(|logged| logged.message.kind() == SessionCmdKindWire::ENGAGE_NOW)
        .map(|logged| logged.at_ns)
        .collect();
    // Two releases, each taking torque off every row: the first session's and
    // the second one's, which the run's tail ends after. The ordering below is
    // asserted against the ninth write, which is the last of the first
    // release -- the one the second engagement's ask has to follow.
    let wanted = 2 * ROW_COUNT;
    if released.len() != wanted {
        failures.push(format!(
            "the session wrote {} verified torque-off write(s) and two orderly releases of this \
             machine are {wanted}: a release that wrote fewer left a row this check has no \
             evidence about",
            released.len()
        ));
    }
    let (Some(&second_ask), Some(&last_write)) = (asks.get(1), released.get(ROW_COUNT - 1)) else {
        failures.push(format!(
            "the run carries {} engagement ask(s) and {} torque-off write(s), and this scenario \
             engages the machine twice",
            asks.len(),
            released.len()
        ));
        return;
    };
    if second_ask <= last_write {
        failures.push(format!(
            "the second engagement asked for torque at {second_ask} and the release's last write \
             went out at {last_write}: torque comes back on only once it has come off"
        ));
    }
    let asked_on = scenario::cycle_within(second_ask);
    if asked_on != rested {
        failures.push(format!(
            "the second engagement's ask went out on cycle {asked_on} and the machine reached \
             rest on {rested}: a script held through a release is drained on the wake the release \
             confirms, and that wake takes the engagement's first bus step"
        ));
    }
}
