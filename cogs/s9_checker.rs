//! S9's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. What is asserted is that the survey pinged the bus,
//! found a servo missing, refused the machine and parked -- and that nothing
//! else happened at all: no register was read, nothing was written, no schedule
//! went out, no goal was published, no servo was ever energised, and the script
//! that arrived afterwards was refused as parked.
//!
//! The absence of things is most of this file, which is what a run about a
//! machine that was never commanded has to say. Each absence is asserted against
//! a run long enough for the thing to have happened -- the dead-man's window
//! closes twice over before the end -- because an absence over too short a run
//! is not an assertion.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__cogs__session_cmd_clk_rs::SessionCmdKindWire;
use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKindWire;
use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKindWire};
use brenn_reachy__motion__seq_clk_rs::SeqFailureKindWire;
use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;
use reachy_motion::arm::SERVO_IDS;
use reachy_motion::joints;
use reachy_motion::joints::ROW_COUNT;
use scenario::check;
use scenario::cycle_within;
use scenario::read::Run;
use scenario::{FIRST_CYCLE, hold_timeout_cycles};

use s9_scenario::{SCRIPT_ID, absent_joint, end_cycle, parked_by_cycle, script_sent_cycle};

fn main() -> ExitCode {
    check::main("s9_checker", |run, failures| {
        // The clock ran to the end and the machine kept saying where it was
        // throughout: the absence is on the unicast path, and the driver's
        // proprioception is not.
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        check::scripts_sent(run, &[(SCRIPT_ID, script_sent_cycle())], failures);

        check_the_script_meets_a_parked_machine(failures);
        // One phase change and no more. A survey that failed has nowhere to go
        // but parked, and a run with a second change in it is a machine that
        // went on to do something with a bus it had refused.
        let parked = check::phases(
            run,
            &[(SessionPhaseWire::PARKED, SessionPhaseWire::STARTING)],
            failures,
        );
        check_parked_promptly(parked.first().copied(), failures);
        check_the_survey_stopped_at_the_sweep(run, failures);
        // The script was refused, and refused as parked: a sender told the
        // machine was busy would keep asking, and this run's ending is one
        // nothing but an operator clears.
        check::refusals(
            run,
            &[(SCRIPT_ID, RefusalReasonWire::PARKED, script_sent_cycle())],
            failures,
        );

        // Nothing was ever under command. No schedule, because nothing was
        // accepted; no goal, because the mover is engaged by a schedule and
        // there was none; no raise, because a tick that never armed has nothing
        // to say.
        check::no_schedules(run, failures);
        // No schedule was ever published, so a goal here would be a decision
        // tick running on an engagement nobody granted it.
        check::no_goals(run, "the survey refused the machine", failures);
        check::no_faults(run, failures);
        // And nothing was ever energised. The gate's dead-man measures the
        // silence after the last datagram against a machine it believes is
        // holding torque, and this one never wrote a torque-enable register at
        // all -- so the window closing on it in silence, twice over before the
        // run ends, must raise nothing.
        check::no_events(run, failures);
        // Three rows, and the whole of what a failed survey is written down as:
        // the verdict that stopped it, the phase it latched into, and the script
        // it then declined. The verdict comes first because it explains the row
        // after it; the rest of the survey's evidence stays in the state slot,
        // which does not reach this stream.
        check::narration(
            run,
            &[
                ReportKindWire::COMMISSION_FAILED,
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::SCRIPT_REFUSED,
            ],
            failures,
        );
        check_the_verdict_names_the_missing_servo(run, failures);
        check::stands_still(
            run,
            FIRST_CYCLE,
            end_cycle(),
            "limp where it stood, and nothing in this run ever commanded it",
            failures,
        );
        check::signal_groups(run, failures);
    })
}

/// The scenario still describes the run it claims to: the machine is parked well
/// before the script it refuses is sent, and the run outlasts the dead-man's
/// window by a margin.
///
/// Numbers this file does not own decide both -- how long a survey's presence
/// sweep takes, and how long the gate gives a commander to speak -- and a move in
/// either can flip them. A script that arrived while the survey was still
/// running would be held for the phase the survey ends in and refused as
/// `parked` by the drain instead, and every assertion below would still pass
/// while the run said something else entirely, so the ordering is asserted
/// rather than described.
fn check_the_script_meets_a_parked_machine(failures: &mut Vec<String>) {
    if parked_by_cycle() >= script_sent_cycle() {
        failures.push(format!(
            "the survey is allowed until cycle {} and the script is sent on cycle {}: this run is \
             about a script a parked machine refuses, and one that arrived mid-survey would be \
             refused for a different reason",
            parked_by_cycle(),
            script_sent_cycle()
        ));
    }
    if end_cycle() < script_sent_cycle() + 2 * hold_timeout_cycles() {
        failures.push(format!(
            "the run ends on cycle {} and the script is sent on cycle {}: the tail has to outlast \
             the gate's {}-cycle window for the absence of any event to mean anything",
            end_cycle(),
            script_sent_cycle(),
            hold_timeout_cycles()
        ));
    }
}

/// The verdict row says which failure stopped the survey, and at which servo.
///
/// The whole point of the row: a reader of the report stream learns that a servo
/// did not answer its ping and which one, without the state slot the rest of the
/// verdict lives in. This scenario silences exactly one crank, so the headline
/// is one absent servo and the id is that crank's.
fn check_the_verdict_names_the_missing_servo(run: &Run, failures: &mut Vec<String>) {
    let verdicts: Vec<&TimelineEntryWire> = run
        .reports
        .iter()
        .map(|logged| &logged.message)
        .filter(|report| report.kind() == ReportKindWire::COMMISSION_FAILED)
        .collect();
    let [verdict] = verdicts.as_slice() else {
        failures.push(format!(
            "the session narrated {} commission verdicts, and a survey that refused the machine \
             stops once and says so once",
            verdicts.len()
        ));
        return;
    };
    let absent = u32::from(SeqFailureKindWire::ABSENT_SERVOS.0);
    if verdict.a() != absent {
        failures.push(format!(
            "the verdict is failure kind {}, and this run's survey stops on a ping nothing \
             answered ({absent})",
            verdict.a()
        ));
    }
    let silent = u32::from(SERVO_IDS[joints::row(absent_joint()).expect("a crank has a bus row")]);
    if verdict.b() != silent {
        failures.push(format!(
            "the verdict names servo {}, and the servo this run silences is {silent}",
            verdict.b()
        ));
    }
    if (verdict.detail() - 1.0).abs() > f64::EPSILON {
        failures.push(format!(
            "the verdict counts {} absent servos, and this run unplugs one",
            verdict.detail()
        ));
    }
}

/// The park landed inside the survey's own allowance.
///
/// Which cycle exactly is a fact about the run -- whether the session's first
/// wake comes from its floor or from the driver's first health report is not this
/// file's to know -- so what is asserted is the bound the scenario places
/// everything else from.
fn check_parked_promptly(parked: Option<i64>, failures: &mut Vec<String>) {
    let Some(parked) = parked else {
        return;
    };
    if parked > parked_by_cycle() {
        failures.push(format!(
            "the session parked on cycle {parked}, and a presence sweep of {ROW_COUNT} pings is \
             allowed until cycle {}: a survey that took longer asked for something this run does \
             not have in it",
            parked_by_cycle()
        ));
    }
}

/// The survey spent one ping per servo and then stopped.
///
/// The traffic is the whole assertion that the machine was never touched. A
/// presence sweep pings each row in turn; the row that answers nothing is found
/// at the end of the sweep, because presence is about the bus as a whole and a
/// sweep that stopped at the first silence would refuse a machine it had not
/// finished looking at. So the run's datagrams are exactly the nine pings, in bus
/// order -- no identity read, no supply reading, no gains write, and above all no
/// torque-enable write, which is what says nothing was ever energised.
///
/// Every one of them was answered, too: the modelled bus answers a ping to a row
/// that is not there with the silence its own timeout is for, and that is an
/// answer the session gets on the next cycle. A run with an `aux_gave_up` in it
/// would be one whose datagrams went missing on a channel that is memory, which
/// is a different failure entirely -- and the narration asserted above is what
/// says there is none.
fn check_the_survey_stopped_at_the_sweep(run: &Run, failures: &mut Vec<String>) {
    let sent: Vec<(i64, u8)> = run
        .datagrams
        .iter()
        .map(|logged| (cycle_within(logged.at_ns), logged.message.txn().id()))
        .collect();
    // The first one that is not a ping, and no more: a survey that got further
    // than the sweep sends dozens of them, and a list of dozens says nothing the
    // first says.
    let other = run.datagrams.iter().find(|logged| {
        logged.message.kind() != SessionCmdKindWire::AUX
            || logged.message.txn().op() != AuxOpKindWire::PING
    });
    if let Some(logged) = other {
        failures.push(format!(
            "the session published a {:?} datagram asking for {:?} at {}: the only thing this \
             run's survey gets as far as is pinging the bus",
            logged.message.kind(),
            logged.message.txn().op(),
            logged.at_ns
        ));
    }
    let ids: Vec<u8> = sent.iter().map(|(_, id)| *id).collect();
    if ids != SERVO_IDS {
        failures.push(format!(
            "the survey pinged {ids:?}, and the machine this process is configured for is \
             {SERVO_IDS:?}: the sweep walks the bus once in order and refuses the machine at the \
             end of it"
        ));
    }
    let after: Vec<i64> = sent
        .iter()
        .filter(|(cycle, _)| *cycle > parked_by_cycle())
        .map(|(cycle, _)| *cycle)
        .collect();
    if !after.is_empty() {
        failures.push(format!(
            "the session asked the driver for something on cycles {after:?}, past the {} the sweep \
             is allowed: a parked machine is asked for nothing, not even a keep-alive -- there is \
             no torque for one to protect",
            parked_by_cycle()
        ));
    }
}
