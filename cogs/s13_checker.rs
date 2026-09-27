//! S13's assertions, over the output log.
//!
//! Four arguments: the output log directory and the three config textprotos the
//! process ran against. Everything asserted here is read out of those.
//!
//! What is S13's among them is the answer each of the four scripts got, the
//! order of the two things that must not overlap -- the release's last write and
//! the second engagement's ask -- and the second session's look: where it lands,
//! body included, and the clock it runs on. Every other property of a healthy run is
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
use reachy_motion::joints::{JointRef, ROW_COUNT, row};
use scenario::check;
use scenario::read::Run;

use s13_scenario::{
    CLOSING_SCRIPT_ID, HELD_SCRIPT_ID, KEEP_SCRIPT_ID, LOOK_BEARING_MRAD, LOOK_ELEVATION_MRAD,
    OPENING_SCRIPT_ID, closing_cycle, disengage_cycle, duplicate_cycle, end_cycle, keep_sent_cycle,
    keep_stamped_cycle, script_sent_cycle, second_disengage_cycle, second_look_start_cycle,
    second_stow_start_cycle, stow_start_cycle,
};

/// How many cycles past the look's clock the goal stream may still move.
///
/// The mover acts on a row the cycle it comes due or the one after, and the goal
/// is dated ahead of the sample, so the stream is still only by a couple of
/// cycles past the clock.
const LOOK_DISPATCH_SLACK_CYCLES: i64 = 2;

fn main() -> ExitCode {
    check::main("s13_checker", |run, failures| {
        check::heartbeat(run, end_cycle(), failures);
        check::readings_present(run, failures);
        // The five things the scenario said, in the order it said them: two at
        // one instant, a keep stamped ahead of its sending, and the last two
        // the same number twice.
        let sent = script_sent_cycle();
        check::scripts_sent_stamped(
            run,
            &[
                (OPENING_SCRIPT_ID, sent, sent),
                (HELD_SCRIPT_ID, sent, sent),
                (KEEP_SCRIPT_ID, keep_sent_cycle(), keep_stamped_cycle()),
                (CLOSING_SCRIPT_ID, closing_cycle(), closing_cycle()),
                (CLOSING_SCRIPT_ID, duplicate_cycle(), duplicate_cycle()),
            ],
            failures,
        );
        // The first session's whole life, and then the second one the drained
        // script opens: the machine is engaged again, runs a schedule and is let
        // go of, so the run carries two of every phase an engagement has.
        let (engaged, second) = check::ordinary_life(
            run,
            &[
                (SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
                (SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING),
                (SessionPhaseWire::STOPPING, SessionPhaseWire::ACTIVE),
                (SessionPhaseWire::RESTING, SessionPhaseWire::STOPPING),
            ],
            failures,
        );
        check::survey_cost(run, engaged.map(|engaged| engaged.commissioned), failures);
        // The first session publishes its arming, the keep's replacement and
        // the schedule nobody is running; the second session's pair follows.
        check::schedules_under_one_engagement(
            run,
            &["the arming", "the keep"],
            check::Tail::FurtherSessions,
            engaged,
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
        check_keep_froze_the_raise(run, failures);
        check_look_ran_on_its_head_clock(run, failures);
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

/// The machine arrives at what each session's schedule asked for: stowed by the
/// end of the first session's fold, and upright, then at its look -- the body
/// yaw included -- and then stowed in the second.
///
/// The look is judged against the Mover's own composition of its direction.
/// [`check::arrived_at`] reads the head pose and the antennas, and the look also
/// turns the body, so the body yaw's reading is asserted beside it.
///
/// The first session's fold is the keep script's, and "stowed" is what shows the
/// drained schedule ran: the opening script has no fold. Its raise is not
/// asserted upright, because the keep stops it partway.
fn check_arrival(run: &Run, failures: &mut Vec<String>) {
    check::arrived_at(
        run,
        "stowed",
        disengage_cycle() - 1,
        &scenario::stow_pose(),
        failures,
    );
    check::arrived_at(
        run,
        "upright again",
        second_look_start_cycle() - 1,
        &scenario::neutral_pose(),
        failures,
    );
    let look = scenario::look_pose(LOOK_BEARING_MRAD, LOOK_ELEVATION_MRAD);
    let looking = second_stow_start_cycle() - 1;
    check::arrived_at(run, "looking", looking, &look, failures);
    match (check::sample_at(run, looking), row(JointRef::BodyYaw)) {
        (None, _) => failures.push(format!(
            "no sample for cycle {looking}, where the body should be turned to the look"
        )),
        (_, None) => failures.push(format!("{:?} sits on no bus row", JointRef::BodyYaw)),
        (Some(sample), Some(body)) => {
            let present = check::present_rows(sample)[body];
            if (present - look.body_yaw).abs() > check::ARRIVAL_TOLERANCE {
                failures.push(format!(
                    "at cycle {looking} the body yaw reads {present} rad and the look puts it at \
                     {} rad",
                    look.body_yaw
                ));
            }
        }
    }
    check::arrived_at(
        run,
        "stowed again",
        second_disengage_cycle() - 1,
        &scenario::stow_pose(),
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

/// The second session's look ran on the clock the deployed head clock and the
/// mover's floor give it: the goal was still moving halfway through that clock,
/// and it does not move again from the clock's end until the fold.
///
/// The tick's `MoveTo` is an in-process call and is not logged, so this is where
/// its durations show: in the goal stream, measured against the clock the
/// mover's own floor gives the move. Its target is the arrival reads'. The
/// mover acts on the row the cycle it comes due or the one after, and the goal
/// is dated ahead of the sample, so the stream is held from a couple of cycles
/// past the clock.
fn check_look_ran_on_its_head_clock(run: &Run, failures: &mut Vec<String>) {
    let look = scenario::look_pose(LOOK_BEARING_MRAD, LOOK_ELEVATION_MRAD);
    let start = second_look_start_cycle();
    let moving = scenario::look_clocks(&scenario::neutral_pose(), &look).cycles();
    let halfway = start + moving / 2;
    let before = check::goal_at_or(run, halfway, "turning to the look", failures);
    let after = check::goal_at_or(run, halfway + 1, "turning to the look", failures);
    if let (Some(before), Some(after)) = (before, after)
        && before == after
    {
        failures.push(format!(
            "the goal stood still halfway through the look's {moving}-cycle clock (cycle \
             {halfway}): the look did not move on the clock the deployed head clock and the \
             mover's floor give it"
        ));
    }
    let settled = start + moving + LOOK_DISPATCH_SLACK_CYCLES;
    let Some(held) = check::goal_at_or(run, settled, "holding the look", failures) else {
        return;
    };
    for cycle in settled + 1..second_stow_start_cycle() {
        if check::goal_at(run, cycle) != Some(held) {
            failures.push(format!(
                "the goal moved on cycle {cycle}, {} cycles past the look's {moving}-cycle clock: \
                 the look ran on a clock other than the one the deployed head clock and the \
                 mover's floor give it",
                cycle - start - moving
            ));
            return;
        }
    }
}

/// The keep that arrived mid-raise stopped the raise where it stood: the goal
/// was still moving on the cycles before it arrived, and it does not move again
/// until the fold.
///
/// The mover acts on the new schedule on the cycle it arrives or the one after,
/// and either way every goal from the cycle after it arrived on is the held one.
fn check_keep_froze_the_raise(run: &Run, failures: &mut Vec<String>) {
    let arrived = keep_sent_cycle();
    let before = check::goal_at_or(run, arrived - 2, "raising", failures);
    let last = check::goal_at_or(run, arrived - 1, "raising", failures);
    let (Some(before), Some(last)) = (before, last) else {
        return;
    };
    if before == last {
        failures.push(format!(
            "the raise had stopped before the keep arrived at cycle {arrived}; this run no longer \
             tests a keep landing on a moving tick"
        ));
    }
    let Some(held) = check::goal_at_or(run, arrived + 1, "held by the keep", failures) else {
        return;
    };
    for cycle in arrived + 2..stow_start_cycle() {
        if check::goal_at(run, cycle) != Some(held) {
            failures.push(format!(
                "the goal moved on cycle {cycle} under the keep that arrived on {arrived}: a keep \
                 stops the move it lands on"
            ));
            return;
        }
    }
}
