//! S16, a clip replaced by another while its window is open: the scenario that
//! says a row taken over keeps the commanded stream continuous.
//!
//! One engagement carrying five scripts. The opening one raises the machine and
//! plays `pollen/dances/simple_nod` over the posture once it stands still; then
//! three replacements arrive while a window is still playing, each asking for
//! the upright posture at once and for the other clip a millisecond later; then
//! a closing script takes the machine down. The two clips alternate, so every
//! replacement lands a window of another motion in the row the outgoing clip
//! is playing in, and that row is taken over rather than picked up.
//!
//! Each replacement is sent half way through a cycle, so its window already
//! covers the first mover sample after the one it arrived during: the seam is
//! forced onto the take-over, not left to where the grid happens to fall. Where
//! each one lands is chosen for what the outgoing clip is carrying there -- the
//! nod's peak, then the chin lead three cycles before its end, then the nod
//! three cycles before its end, where it is moving fastest -- so every seam has
//! a standing contribution a dropped row would take out of the stream at once.
//!
//! What is asserted about each seam is the one sample the replacement takes
//! effect on, and nothing around it: the content on either side carries no
//! speed ceiling, and a check on the neighbourhood would measure the clips as
//! much as the seam.
//!
//! Both the author and the checker read this module, so what the run *is* is
//! stated once. Every instant is a cycle count from the epoch unless it says
//! otherwise.

use scenario::author::{Overlay, Step};
use scenario::{UP_DURATION_NS, cycle_at, cycles_for, run_end_cycle};

use brenn_reachy__motion__joints_clk_rs::JointFlags;
use reachy_motion::joints::JointGroup;

// The shape of an ordinary run, stated once for every scenario: where a run
// begins, the cycle a script may first be taken on, and the cycle the machine
// is armed and holding by.
pub use scenario::{START_CYCLE, armed_cycle as up_start_cycle, script_cycle as script_sent_cycle};

/// The number of the script that opens the engagement.
pub const OPENING_SCRIPT_ID: u32 = 16;

/// The numbers of the three replacements, in the order they are sent.
///
/// Strictly increasing, which is the whole of the ordering rule a replacement
/// is screened against.
pub const REPLACEMENT_SCRIPT_IDS: [u32; 3] = [17, 18, 19];

/// The number of the closing script: up, and then the fold.
pub const CLOSING_SCRIPT_ID: u32 = 20;

/// One of the two clips the run alternates between.
///
/// Names and not numbers: the numbering is generated and positional, and
/// [`scenario::motion_id`] reads it out of the sidecar the emitter writes.
pub const NOD: &str = "pollen/dances/simple_nod";

/// The other clip.
pub const CHIN: &str = "pollen/dances/chin_lead";

/// The clip each window plays: the opening one's, then replacements 1 to 3.
pub const MOTIONS: [&str; 4] = [NOD, CHIN, NOD, CHIN];

/// How much of each clip's delta its window asks for: the whole of it, so the
/// contribution standing at each seam is the clip's own.
pub const GAIN: f64 = 1.0;

/// How fast each clip is played: at the rate it was authored at.
pub const SPEED: f64 = 1.0;

/// Half a period: a replacement is sent mid-cycle, so its window, open from
/// one millisecond after arrival, covers every mover sample after the cycle
/// it was sent in.
pub const MID_CYCLE_NS: i64 = scenario::PERIOD_NS / 2;

/// A replacement's window opens this long after its arrival: `play@1`.
pub const PLAY_AFTER_MS: i64 = 1;

/// How many cycles into the outgoing window each replacement is sent, by the
/// outgoing window's own cycle count (see [`replacement_cycle`]).
///
/// 1: 46 -- simple_nod's frame 46, its peak (pitch +0.35, antennas ±0.79).
/// 2: 90 -- chin_lead at D − 3 cycles (1800 ms), the loop's own cadence.
/// 3: 89 -- simple_nod at D − 3 cycles (1780 ms), fast content at the seam.
pub const SENT_INTO_WINDOW_CYCLES: [i64; 3] = [46, 90, 89];

/// How long each replacement's neutral step lasts, ms: past its window's
/// close and the hand-back after it, so the third one is still running when
/// the closing script arrives.
pub const REPLACEMENT_HOLD_MS: i64 = 4000;

/// How long the closing script holds the machine up before folding it, in
/// cycles.
pub const CLOSING_UP_CYCLES: i64 = 20;

/// How many cycles after the cycle a replacement is sent in the mover first
/// composes under the replacing schedule.
///
/// Read off the goal stream, not derived: it is where the session's wake and
/// the tick's own schedule read happen to fall against the grid, and a wrong
/// value fails the restart check at every seam on a content step instead.
pub const RESTART_AFTER_SEND_CYCLES: i64 = 1;

/// The cycle the opening window opens on: past the raise's travel, so the
/// machine is standing still at the posture when the first clip starts.
#[must_use]
pub fn opening_window_open_cycle() -> i64 {
    up_start_cycle() + scenario::posture_step_cycles()
}

/// The cycle the machine is upright and standing still on, before any window
/// opens: the reference each seam's contribution is measured against.
#[must_use]
pub fn standing_cycle() -> i64 {
    opening_window_open_cycle() - 2
}

/// How long a window playing `name` occupies the timeline, ms: the motion and
/// its fade-out.
#[must_use]
pub fn window_span_ms(name: &str) -> i64 {
    let (duration, blend_out) = scenario::motion_window_ms(name);
    duration + blend_out
}

/// The cycle the `n`-th replacement is sent in, counting from zero.
///
/// # Panics
///
/// If `n` names no replacement.
#[must_use]
pub fn replacement_cycle(n: usize) -> i64 {
    let into = SENT_INTO_WINDOW_CYCLES[n];
    match n {
        0 => opening_window_open_cycle() + into,
        _ => replacement_cycle(n - 1) + into,
    }
}

/// The instant the `n`-th replacement is sent, ns: half way through its cycle.
#[must_use]
pub fn replacement_sent_ns(n: usize) -> i64 {
    cycle_at(replacement_cycle(n)) + MID_CYCLE_NS
}

/// The instant the `n`-th replacement's window opens, ns.
#[must_use]
pub fn replacement_window_open_ns(n: usize) -> i64 {
    replacement_sent_ns(n) + PLAY_AFTER_MS * 1_000_000
}

/// The instant the `n`-th replacement's window closes, ns.
#[must_use]
pub fn replacement_window_close_ns(n: usize) -> i64 {
    replacement_window_open_ns(n) + window_span_ms(MOTIONS[n + 1]) * 1_000_000
}

/// The instant the window the `n`-th replacement cuts into opened, ns.
#[must_use]
pub fn outgoing_window_open_ns(n: usize) -> i64 {
    match n {
        0 => cycle_at(opening_window_open_cycle()),
        _ => replacement_window_open_ns(n - 1),
    }
}

/// The first mover sample composed under the `n`-th replacement's schedule.
#[must_use]
pub fn restart_cycle(n: usize) -> i64 {
    replacement_cycle(n) + RESTART_AFTER_SEND_CYCLES
}

/// The first cycle the last replacement's window no longer covers.
#[must_use]
pub fn last_close_cycle() -> i64 {
    let close = replacement_window_close_ns(2);
    let mut cycle = replacement_cycle(2);
    while cycle_at(cycle) < close {
        cycle += 1;
    }
    cycle
}

/// The cycle the closing script is sent on: a whole configured posture clock
/// past the last close, so the hand-back has run before the closing script
/// replaces the schedule.
#[must_use]
pub fn closing_cycle() -> i64 {
    last_close_cycle() + cycles_for(UP_DURATION_NS) + 5
}

/// The cycle the closing script's fold begins.
#[must_use]
pub fn stow_start_cycle() -> i64 {
    closing_cycle() + CLOSING_UP_CYCLES
}

/// The cycle the closing schedule runs out on, which is what ends the session.
#[must_use]
pub fn disengage_cycle() -> i64 {
    stow_start_cycle() + scenario::posture_step_cycles()
}

/// The last cycle of the run.
#[must_use]
pub fn end_cycle() -> i64 {
    run_end_cycle(disengage_cycle())
}

/// The rows the two clips drive: the six cranks and the two antennas. Neither
/// drives the body yaw.
#[must_use]
pub fn content_rows() -> JointFlags {
    JointGroup::Legs.joints() | JointGroup::Antennas.joints()
}

/// The opening script's one step: upright, past the opening window's close,
/// which it never reaches because the first replacement ends it.
#[must_use]
pub fn opening_steps() -> [Step; 1] {
    [Step {
        start_ns: cycle_at(up_start_cycle()),
        end_ns: cycle_at(opening_window_open_cycle()) + (window_span_ms(NOD) + 200) * 1_000_000,
        pose: Some(scenario::NEUTRAL_POSE),
        move_ms: None,
    }]
}

/// The opening script's one window: the nod, over the machine standing still.
#[must_use]
pub fn opening_overlays() -> [Overlay; 1] {
    let start_ns = cycle_at(opening_window_open_cycle());
    [Overlay {
        motion_id: scenario::motion_id(NOD),
        start_ns,
        end_ns: start_ns + window_span_ms(NOD) * 1_000_000,
        gain: GAIN,
        speed: SPEED,
    }]
}

/// The `n`-th replacement's one step: upright from the instant it arrives.
#[must_use]
pub fn replacement_steps(n: usize) -> [Step; 1] {
    let start_ns = replacement_sent_ns(n);
    [Step {
        start_ns,
        end_ns: start_ns + REPLACEMENT_HOLD_MS * 1_000_000,
        pose: Some(scenario::NEUTRAL_POSE),
        move_ms: None,
    }]
}

/// The `n`-th replacement's one window: the other clip, a millisecond after it
/// arrives.
#[must_use]
pub fn replacement_overlays(n: usize) -> [Overlay; 1] {
    [Overlay {
        motion_id: scenario::motion_id(MOTIONS[n + 1]),
        start_ns: replacement_window_open_ns(n),
        end_ns: replacement_window_close_ns(n),
        gain: GAIN,
        speed: SPEED,
    }]
}

/// The closing script's two steps: the sender's last beat upright, then the
/// fold.
#[must_use]
pub fn closing_steps() -> [Step; 2] {
    [
        Step {
            start_ns: cycle_at(closing_cycle()),
            end_ns: cycle_at(stow_start_cycle()),
            pose: Some(scenario::NEUTRAL_POSE),
            move_ms: None,
        },
        Step {
            start_ns: cycle_at(stow_start_cycle()),
            end_ns: cycle_at(disengage_cycle()),
            pose: Some(scenario::STOW_POSE),
            move_ms: None,
        },
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    const MS: i64 = 1_000_000;

    #[test]
    fn the_clips_are_the_ones_this_run_is_written_for() {
        assert_eq!(scenario::motion_window_ms(NOD), (1840, 200));
        assert_eq!(scenario::motion_window_ms(CHIN), (1860, 200));
    }

    #[test]
    fn each_replacement_lands_inside_the_clip_it_replaces() {
        for (n, outgoing) in MOTIONS.iter().enumerate().take(3) {
            let restart = cycle_at(restart_cycle(n));
            let opened = outgoing_window_open_ns(n);
            let (duration, _) = scenario::motion_window_ms(outgoing);
            assert!(
                restart > opened,
                "replacement {n} restarts before its outgoing window opens"
            );
            assert!(
                restart < opened + duration * MS,
                "replacement {n} restarts after the outgoing clip has ended"
            );
        }
    }

    #[test]
    fn each_replacement_window_covers_the_mover_from_the_next_cycle() {
        for n in 0..3 {
            let open = replacement_window_open_ns(n);
            assert!(open > cycle_at(replacement_cycle(n)));
            assert!(open < cycle_at(replacement_cycle(n) + 1));
        }
    }

    #[test]
    fn the_third_replacement_is_still_running_when_the_closing_script_arrives() {
        assert!(cycle_at(closing_cycle()) < replacement_sent_ns(2) + REPLACEMENT_HOLD_MS * MS);
        assert!(closing_cycle() > last_close_cycle());
    }

    #[test]
    fn the_numbers_climb() {
        let ids: Vec<u32> = [OPENING_SCRIPT_ID]
            .into_iter()
            .chain(REPLACEMENT_SCRIPT_IDS)
            .chain([CLOSING_SCRIPT_ID])
            .collect();
        assert!(ids.windows(2).all(|pair| pair[0] < pair[1]), "{ids:?}");
        let cycles: Vec<i64> = [script_sent_cycle(), opening_window_open_cycle()]
            .into_iter()
            .chain((0..3).map(replacement_cycle))
            .chain([closing_cycle()])
            .collect();
        assert!(
            cycles.windows(2).all(|pair| pair[0] < pair[1]),
            "{cycles:?}"
        );
    }
}
