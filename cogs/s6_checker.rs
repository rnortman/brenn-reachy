//! S6's assertions, over the output log.
//!
//! Everything S1 asserts about a healthy lifecycle, and then the layer: what the
//! goal stream carries while the window is open, that the hold between the
//! motion's two segments is the scaled delta the window asked for and is
//! *steady*, that the channels nothing plays over come out of the composition
//! untouched, and that the commanded stream is continuous across a window that
//! closed over a contribution the machine was still carrying.
//!
//! The last of those is the invariant the re-anchor exists for, and two
//! assertions pin it: `check::goal_stream` refuses any row that travelled
//! further in a cycle than the plant can, which a dropped contribution does by
//! construction, and [`check_continuity`] below holds the cycles bracketing the
//! close to a bound well under the contribution that was standing there.
//!
//! Every failure is collected rather than thrown, so one run reports everything
//! that was wrong with it.

use std::process::ExitCode;

use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use reachy_motion::joints::{JointRef, row};
use reachy_motion::postures::{neutral_targets, stow_pose_targets};
use scenario::check;
use scenario::read::Run;

use scenario::{stow_clocks, up_clocks};

use s6_scenario::{
    HOLD_RAD, SCRIPT_ID, SECOND_SCRIPT_ID, STOW_CYCLES, UP_CYCLES, absorbed_cycle, disengage_cycle,
    end_cycle, hold_from_cycle, hold_through_cycle, script_sent_cycle, second_script_cycle,
    standing_cycle, stow_start_cycle, sway_peak_cycle, window_close_cycle,
};

/// How far a goal may be from the number this scenario derives for it, radians.
///
/// The composition is arithmetic on numbers the documents state, so what this
/// leaves room for is the base's own solve and nothing else.
const DERIVED_TOLERANCE: f64 = 1e-3;

/// How far a row that nothing is playing over may drift while a composition
/// rides the base, radians.
///
/// The compositor's masking is exact -- a channel no overlay drives comes out of
/// the fold bit-identical to the base -- and the base is a machine standing
/// still, so a leg that moves while the antennas and the yaw carry a motion is a
/// mask that leaked.
const UNTOUCHED_TOLERANCE: f64 = 1e-9;

/// How steady the hold between the motion's segments is, radians.
///
/// A hold freezes the segment's final frame over a base that is standing still,
/// so the composed setpoint is the same setpoint every cycle. Anything moving
/// here is a player walking on through a gap it should be holding in.
const STEADY_TOLERANCE: f64 = 1e-9;

/// The least the sway's own contribution comes to, radians.
///
/// `bench/sway` reaches 0.12 rad and the window asks for half of it. What this
/// bounds away from zero is the whole point of the second segment: a seam the
/// motion crossed without the yaw ever moving would leave every other assertion
/// here passing.
const SWAY_MIN_RAD: f64 = 0.04;

/// How far any row may travel in one cycle across the window's close, radians.
///
/// Above what the motion and the hand-back's own decay ask for -- the sway moves
/// under 0.01 rad a cycle at this gain and a min-jerk absorption of the truncated
/// contribution over the configured posture clock moves under 0.006 -- and well
/// below the contribution standing when the window closed, which is what a layer
/// that dropped its weight instead of re-anchoring would put into a single
/// period.
const CONTINUITY_STEP_RAD: f64 = 0.02;

fn main() -> ExitCode {
    check::main("s6_checker", |run, failures| {
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
        // The session's own life, and then the second script taking hold of the
        // machine it let go of. Two more phase changes: the engagement is one
        // datagram and one driver cycle, so the run's tail is long enough for it
        // to conclude and the machine is under command again when the run ends.
        let engaged = check::engagement_then(
            run,
            &[
                (SessionPhaseWire::ENGAGING, SessionPhaseWire::RESTING),
                (SessionPhaseWire::ACTIVE, SessionPhaseWire::ENGAGING),
            ],
            failures,
        );
        check::ended_promptly(
            engaged.map(|engaged| engaged.released),
            disengage_cycle(),
            failures,
        );
        // Two stretches, because the second script's engagement concludes inside
        // the run's tail: the first session's, and the one the machine is under
        // command for when the run ends.
        let streams = check::goal_streams_exactly(run, JointFlags::NONE, 2, failures);
        if let (Some(stream), Some(engaged)) = (streams.first(), engaged) {
            check::stream_starts_with_session(stream, engaged.taken, failures);
            check::stream_stops_with_release(stream, engaged.released, failures);
        }
        check::estimates_per_sample(run, failures);
        check::estimates_valid(run, failures);
        check_arrival(run, failures);
        check_composition(run, failures);
        check_continuity(run, failures);
        check::no_faults(run, failures);
        // The load-bearing negative: the keep-alive rule carries every stretch
        // of this run in which nothing is streaming -- the survey, the arming,
        // and the two-second settle the release opens with -- so a hold-timeout
        // de-torquing anywhere in it is that rule failing.
        check::no_events(run, failures);
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
                ReportKindWire::SCRIPT_ACCEPTED,
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::PHASE_CHANGED,
                ReportKindWire::SCHEDULE_PUBLISHED,
            ],
            failures,
        );
        check::signal_groups(run, failures);
    })
}

/// The machine arrives where the steps send it, and each step is long enough.
///
/// The upright arrival is asserted before the window opens: the composition is
/// what the machine is commanded to do afterwards, so a posture assertion inside
/// the window would be an assertion about the layer's contribution wearing the
/// base's name.
fn check_arrival(run: &Run, failures: &mut Vec<String>) {
    check::arrived_at(
        run,
        "upright",
        standing_cycle(),
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
    check::room("upright", UP_CYCLES, &up_clocks(), failures);
    check::room("stow", STOW_CYCLES, &stow_clocks(), failures);
    // The scenario's own arithmetic: the window and the room the hand-back needs
    // after it both sit inside the step the motion is played over.
    if absorbed_cycle() >= stow_start_cycle() {
        failures.push(format!(
            "the contribution the close hands back is absorbed by cycle {} and the upright step \
             ends at {}: the scenario does not leave the base time to come home",
            absorbed_cycle(),
            stow_start_cycle()
        ));
    }
}

/// What the layer put in the goal stream: the hold, the seam, the truncation,
/// and the rows nothing played over.
fn check_composition(run: &Run, failures: &mut Vec<String>) {
    let (Some(right), Some(left), Some(yaw)) = (
        row(JointRef::AntennaRight),
        row(JointRef::AntennaLeft),
        row(JointRef::BodyYaw),
    ) else {
        failures.push("this build has no antenna or body-yaw row".to_owned());
        return;
    };
    let Some(standing) = check::goal_at_or(run, standing_cycle(), "standing upright", failures)
    else {
        return;
    };

    // The hold between the segments: the antennas parted by the delta the
    // document's last frame carries, scaled by the gain the window asked for,
    // and not moving while the motion holds them there.
    let Some(entered) = check::goal_at_or(run, hold_from_cycle(), "in the motion's hold", failures)
    else {
        return;
    };
    // One report for the whole hold: it is the same statement about every cycle
    // of it, and a run that broke it broke it for all of them.
    let told = failures.len();
    for cycle in hold_from_cycle()..=hold_through_cycle() {
        if failures.len() > told {
            break;
        }
        let Some(held) = check::goal_at_or(run, cycle, "in the motion's hold", failures) else {
            return;
        };
        for (antenna, at) in [
            (JointRef::AntennaRight, right),
            (JointRef::AntennaLeft, left),
        ] {
            let parted = held[at] - standing[at];
            if (parted.abs() - HOLD_RAD).abs() > DERIVED_TOLERANCE {
                failures.push(format!(
                    "at cycle {cycle} {antenna:?} is held {parted} rad off the posture, and the \
                     motion's last frame at this window's gain is {HOLD_RAD} rad"
                ));
                break;
            }
            if (held[at] - entered[at]).abs() > STEADY_TOLERANCE {
                failures.push(format!(
                    "at cycle {cycle} {antenna:?} reads {} rad and at cycle {} it read {}: a hold \
                     freezes the frame it holds",
                    held[at],
                    hold_from_cycle(),
                    entered[at]
                ));
                break;
            }
        }
        if (held[right] - standing[right]) * (held[left] - standing[left]) >= 0.0 {
            failures.push(format!(
                "at cycle {cycle} the antennas are both {} and {} rad off the posture: the motion \
                 parts them",
                held[right] - standing[right],
                held[left] - standing[left]
            ));
        }
        if (held[yaw] - standing[yaw]).abs() > STEADY_TOLERANCE {
            failures.push(format!(
                "at cycle {cycle} the body yaw is {} rad off the posture, and the segment holding \
                 here drives the antennas alone",
                held[yaw] - standing[yaw]
            ));
        }
    }

    // The seam: the second segment drives the yaw, and the antennas the first
    // one drove have faded out of the composition on their own exit ramp.
    if let Some(swaying) = check::goal_at_or(
        run,
        sway_peak_cycle(),
        "part way through the sway",
        failures,
    ) {
        let turned = swaying[yaw] - standing[yaw];
        if turned.abs() < SWAY_MIN_RAD {
            failures.push(format!(
                "at cycle {} the body yaw is {turned} rad off the posture, and the motion's second \
                 segment sways it by at least {SWAY_MIN_RAD}",
                sway_peak_cycle()
            ));
        }
        for (antenna, at) in [
            (JointRef::AntennaRight, right),
            (JointRef::AntennaLeft, left),
        ] {
            let parted = swaying[at] - standing[at];
            if parted.abs() > HOLD_RAD / 5.0 {
                failures.push(format!(
                    "at cycle {} {antenna:?} is still {parted} rad off the posture, and the \
                     segment that drove it ended a whole exit ramp ago",
                    sway_peak_cycle()
                ));
            }
        }
    }

    // The truncation: the window closed over a contribution the machine was
    // still carrying, which is what makes the continuity below an assertion
    // about the re-anchor rather than about nothing.
    if let Some(carried) = check::goal_at_or(
        run,
        window_close_cycle() - 1,
        "as the window closed",
        failures,
    ) {
        let turned = carried[yaw] - standing[yaw];
        if turned.abs() < SWAY_MIN_RAD {
            failures.push(format!(
                "the last cycle the window covered carries a body yaw {turned} rad off the \
                 posture: this scenario closes the window over a motion still playing"
            ));
        }
    }

    // And then the base has it back: the truncated contribution is absorbed into
    // a planned move, so the machine is standing in its posture again.
    if let Some(home) = check::goal_at_or(run, absorbed_cycle(), "back in its posture", failures) {
        for at in 0..home.len() {
            let wanted = standing[at];
            if (home[at] - wanted).abs() > check::ARRIVAL_TOLERANCE {
                failures.push(format!(
                    "at cycle {} row {at} is commanded {} rad and the posture it stands in is \
                     {wanted}: the contribution the close handed back was not absorbed",
                    absorbed_cycle(),
                    home[at]
                ));
            }
        }
    }

    // The rows nothing played over: the motion drives the antennas and the yaw,
    // so every crank the head sits on is commanded exactly what a machine
    // holding its posture is commanded.
    for cycle in [
        hold_from_cycle(),
        sway_peak_cycle(),
        window_close_cycle() - 1,
    ] {
        let Some(composed) = check::goal_at_or(run, cycle, "under the composition", failures)
        else {
            continue;
        };
        for at in 0..composed.len() {
            if at == right || at == left || at == yaw {
                continue;
            }
            if (composed[at] - standing[at]).abs() > UNTOUCHED_TOLERANCE {
                failures.push(format!(
                    "at cycle {cycle} row {at} is {} rad off the posture, and nothing this motion \
                     plays drives it",
                    composed[at] - standing[at]
                ));
            }
        }
    }
}

/// The commanded stream is continuous across the window's close.
///
/// Cycle by cycle over the close, every row: the contribution the window was
/// carrying is absorbed into the base's own planned move, so what the stream
/// shows is the same shaped, step-bounded travel it shows anywhere else. A layer
/// that let a closing window's weight fall away would put the whole standing
/// contribution into one period, which the assertion above says is at least
/// twice this bound.
fn check_continuity(run: &Run, failures: &mut Vec<String>) {
    let close = window_close_cycle();
    for cycle in (close - 6)..=(close + 8) {
        let (Some(before), Some(after)) = (
            check::goal_at_or(run, cycle - 1, "across the window's close", failures),
            check::goal_at_or(run, cycle, "across the window's close", failures),
        ) else {
            return;
        };
        for at in 0..after.len() {
            let travelled = (after[at] - before[at]).abs();
            if travelled > CONTINUITY_STEP_RAD {
                failures.push(format!(
                    "row {at} travelled {travelled} rad between cycles {} and {cycle}, across a \
                     window closing at {close}",
                    cycle - 1
                ));
            }
        }
    }
}
