//! The compile: a motion script becomes the request the session screens.
//!
//! The two vocabularies say nearly the same thing in different words. A script
//! names instants and the machine's session names intervals, so the arithmetic
//! here is mostly turning the first into the second: a base step lasts until
//! the next base step, and the last one lasts until the schedule's horizon. A
//! play step names a motion and a speed; the wire carries an index and a
//! window, so the name resolves through the sidecar and the window is
//! `PlayWindow`'s own span — the motion's clock scaled by the speed, plus a
//! blend-out that is not.
//!
//! Two rules are this crate's own, and both are about how a schedule ends.
//!
//! **Every schedule ends stowed.** The presence contract's "up until the
//! timeout" means stowed at the deadline: the horizon is where the session
//! concludes the engagement and releases torque, and a release expects the
//! machine at stow. So a script whose last base step is not a stow gets one
//! appended, ending exactly at `timeout_ms`.
//!
//! **A schedule that stows early ends there.** When the script's own last base
//! step is a stow, the horizon is that stow's end and not the timeout: the wire
//! contract makes `timeout_ms` an unconditional ceiling on exposure, not a
//! duration to fill, and stretching the closing stow to the ceiling would hold
//! the machine stowed under torque — this machine's one pinch hazard — for as
//! long as the sender's remaining headroom. A `keep` after that stow says to
//! hold what is commanded, which is the stow, so it is the same hold written a
//! different way and is refused rather than compiled.
//!
//! **Nothing outlives the closing stow.** The session's engagement ends at the
//! last instant anything in the schedule owns — steps and overlay windows
//! alike — so a play window reaching past the closing stow would extend the
//! engagement past the ceiling the sender stated, keep adding deltas through
//! the fold, and leave the release happening off stow. A window ending after
//! the horizon is refused.
//!
//! All three rules refuse rather than shorten. A script that leaves no room for
//! its own closing stow is dropped whole; a stow shortened to fit would be a
//! machine folding faster than the step it was authored with, and a window
//! trimmed to the horizon would cut a motion off mid-blend.
//!
//! What is not decided here: whether the machine will do any of it. The session
//! screens what this produces and refuses it whole if it disagrees; every
//! commanded value still meets the mover's envelope check. The screens below
//! exist so that nothing sent to the session can be refused for *size* — the
//! one refusal that would be this edge's own fault.

use brenn_reachy__cogs__schedule_clk_rs::{PostureWire, StepKindWire};
use brenn_reachy__cogs__script_clk_rs::{ScriptOverlayWire, ScriptStepWire, ScriptWire};
use clockwork_rs::{Clear as _, SyncTime};
use motion_proto::{Base, MotionScript, Posture};
use thiserror::Error;

use crate::config::EdgeConfig;
use crate::names::MotionTable;

/// How many steps the schedule holds, and so how many the compile may produce.
///
/// The screen is counted *after* the closing stow is synthesized, which is what
/// makes it load-bearing rather than a mirror of the wire contract: a script
/// bounds its timeout and not its step count.
pub const MAX_STEPS: usize = 16;

/// How many overlay windows the schedule holds.
pub const MAX_OVERLAYS: usize = 4;

/// The gain every overlay this edge sends plays at.
///
/// Full weight, because the wire has no vocabulary for anything else: a script
/// asks for a motion at a speed, and a partially weighted delta is a thing no
/// sender can currently mean.
const FULL_GAIN: f64 = 1.0;

/// Why a script did not become a request.
///
/// Every one is a drop of the whole script. There is no disposition here that
/// sends part of a timeline.
#[derive(Clone, Debug, PartialEq, Error)]
pub enum CompileError {
    /// The timeline commands no posture: no steps at all, or nothing but
    /// `keep`.
    ///
    /// `keep` holds a base the machine is already commanding, and a machine at
    /// rest has none — so a posture-free script means nothing at rest and this
    /// edge cannot tell rest from engagement: the phase it holds is narration
    /// the session published, possibly stale, and never an input to a screen.
    /// Compiled and forwarded, this shape would torque a resting head through a
    /// pointless stow-hold cycle. No publisher emits one today.
    #[error("the script commands no posture; `keep` alone moves nothing")]
    NoPosture,

    /// A step that owns no time. Unreachable as things stand and kept as the
    /// guard it is: the wire contract's ascending offsets rule out every case
    /// but the closing stow's, that one is reported as itself, and the stow of
    /// no duration that would produce it is refused where the configuration is
    /// read.
    #[error("the step at {after_ms} ms would last no time")]
    StepWithoutDuration {
        /// Where in the timeline it sits.
        after_ms: u64,
    },

    /// The timeline leaves no room for the stow that ends it — either it runs
    /// too close to its own timeout for a synthesized stow, or its own closing
    /// stow would cross it.
    ///
    /// A refusal rather than a shortened stow: the sender sized a timeline
    /// against a stow budget it got wrong, and folding the machine faster than
    /// the step it was authored with is not this edge's decision to make.
    #[error(
        "the base step at {after_ms} ms leaves no room for a {stow_duration_ms} ms stow inside \
         the script's own {timeout_ms} ms timeout"
    )]
    NoRoomForStow {
        /// The last base step of the timeline.
        after_ms: u64,
        /// How long the stow takes.
        stow_duration_ms: u32,
        /// The ceiling it had to end inside.
        timeout_ms: u64,
    },

    /// A `keep` standing after the stow that ends the timeline.
    ///
    /// `keep` holds whatever base is commanded, and after a stow that base is
    /// the stow: the step is a stowed hold under torque written in another
    /// vocabulary, and honouring it would run the machine folded to the
    /// sender's timeout — the pinch hazard the early horizon exists to avoid.
    /// Refused rather than reinterpreted, because a sender that wrote it meant
    /// something this edge cannot tell from that hazard.
    #[error(
        "the script keeps the base at {after_ms} ms, after the stow at {stow_after_ms} ms that \
         ends the timeline"
    )]
    KeepAfterStow {
        /// The keep that stands past the close.
        after_ms: u64,
        /// The stow it would hold.
        stow_after_ms: u64,
    },

    /// A play step names a motion the deployed library does not hold. Refused
    /// rather than skipped: a script that plays two motions and finds one is
    /// not the script anybody wrote.
    #[error("the script plays `{name}`, which the library does not hold")]
    UnknownMotion {
        /// The name that did not resolve.
        name: String,
    },

    /// An overlay window longer than the wire field carries. Only a sidecar
    /// naming an absurd duration produces one; a truncated window would close
    /// early and leave a motion cut off mid-blend.
    #[error("playing `{name}` would occupy {span_ms} ms, which no window field carries")]
    WindowUnrepresentable {
        /// The motion in question.
        name: String,
        /// How long the window would be.
        span_ms: u64,
    },

    /// A play window that would still be running when the schedule's closing
    /// stow ends.
    ///
    /// The session's engagement lasts until the last instant any row owns, a
    /// window included, so this shape would hold the machine engaged past the
    /// timeout its own sender wrote, superimpose deltas through the fold, and
    /// release off stow. Refused rather than trimmed: a window cut to the
    /// horizon ends a motion mid-blend.
    #[error(
        "playing `{name}` would run to {end_ms} ms, past the schedule's {horizon_ms} ms horizon"
    )]
    OverlayPastHorizon {
        /// The motion that would still be playing.
        name: String,
        /// Where its window ends.
        end_ms: u64,
        /// Where the schedule ends.
        horizon_ms: u64,
    },

    /// More steps than the schedule holds, counted after the closing stow.
    #[error("the schedule would need {steps} steps; a script carries at most {MAX_STEPS}")]
    TooManySteps {
        /// How many the compile produced.
        steps: usize,
    },

    /// More overlay windows than the schedule holds.
    #[error(
        "the schedule would need {overlays} overlay windows; a script carries at most {MAX_OVERLAYS}"
    )]
    TooManyOverlays {
        /// How many the compile produced.
        overlays: usize,
    },
}

/// One step of the compiled schedule, before it is written into the message.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Row {
    after_ms: u64,
    duration_ms: u64,
    posture: Option<Posture>,
}

/// `script` as the request the session screens, stamped `arrival` and numbered
/// `script_id`.
///
/// # Errors
///
/// [`CompileError`] for anything that does not become a lawful schedule. Every
/// one is a drop of the whole script.
///
/// # Panics
///
/// Never in practice: the arithmetic is bounded by the wire contract's own
/// ten-minute timeout ceiling, so every offset and duration below fits the
/// message's fields, and the row counts are screened before anything is
/// written.
pub fn compile(
    script: &MotionScript,
    arrival: SyncTime,
    script_id: u32,
    config: &EdgeConfig,
    table: &MotionTable,
) -> Result<ScriptWire, CompileError> {
    let rows = base_rows(script, config)?;
    if rows.len() > MAX_STEPS {
        return Err(CompileError::TooManySteps { steps: rows.len() });
    }
    let horizon_ms = rows
        .last()
        .map(|row| row.after_ms + row.duration_ms)
        .expect("a timeline that commands a posture");
    let overlays = overlay_rows(script, table, horizon_ms)?;
    if overlays.len() > MAX_OVERLAYS {
        return Err(CompileError::TooManyOverlays {
            overlays: overlays.len(),
        });
    }

    let mut message = ScriptWire::new();
    // Cleared so that what a step does not name is the schema's zero and not a
    // field left over from whatever this message was before.
    message.clear();
    message.set_script_id(script_id);
    message.set_arrival(arrival);
    {
        let mut steps = message.steps_mut();
        for row in &rows {
            let step: &mut ScriptStepWire = steps.try_grow().expect("a screened row count");
            step.set_after_ms(field(row.after_ms));
            step.set_duration_ms(field(row.duration_ms));
            match row.posture {
                Some(posture) => {
                    step.set_kind(StepKindWire::BASE_POSTURE);
                    step.set_posture(posture_wire(posture));
                }
                None => step.set_kind(StepKindWire::BASE_KEEP),
            }
        }
    }
    {
        let mut windows = message.overlays_mut();
        for window in &overlays {
            let row: &mut ScriptOverlayWire = windows.try_grow().expect("a screened window count");
            row.set_motion_id(window.motion_id);
            row.set_after_ms(field(window.after_ms));
            row.set_duration_ms(field(window.duration_ms));
            row.set_gain(FULL_GAIN);
            row.set_speed(window.speed);
        }
    }
    Ok(message)
}

/// The base timeline as intervals, closing stow included.
fn base_rows(script: &MotionScript, config: &EdgeConfig) -> Result<Vec<Row>, CompileError> {
    let bases: Vec<(u64, Base)> = script
        .steps()
        .iter()
        .filter_map(|step| step.action.base().map(|base| (step.after_ms, base)))
        .collect();
    if !bases.iter().any(|(_, base)| base.posture().is_some()) {
        return Err(CompileError::NoPosture);
    }

    let timeout_ms = script.timeout_ms();
    let stow_ms = u64::from(config.stow_duration_ms());
    let (last_after, _) = *bases.last().expect("a timeline that commands a posture");
    // Which posture closes the timeline is a question about the last step that
    // states one, not about the last step: a `keep` states none, and a keep
    // standing after a stow holds that stow rather than ending it.
    let (posture_index, closing_posture) = bases
        .iter()
        .enumerate()
        .filter_map(|(index, (_, base))| base.posture().map(|posture| (index, posture)))
        .next_back()
        .expect("a timeline that commands a posture");
    let stow_terminal = closing_posture == Posture::Stow;
    if stow_terminal && posture_index + 1 < bases.len() {
        return Err(CompileError::KeepAfterStow {
            after_ms: bases[posture_index + 1].0,
            stow_after_ms: bases[posture_index].0,
        });
    }
    // Where the last of the script's own base steps ends. A closing stow ends
    // at its own duration; anything else runs up to the stow this compile
    // appends, which is what makes the horizon `timeout_ms`.
    let last_end = if stow_terminal {
        last_after + stow_ms
    } else {
        timeout_ms.saturating_sub(stow_ms)
    };
    let no_room = if stow_terminal {
        last_end > timeout_ms
    } else {
        last_end <= last_after
    };
    if no_room {
        return Err(CompileError::NoRoomForStow {
            after_ms: last_after,
            stow_duration_ms: config.stow_duration_ms(),
            timeout_ms,
        });
    }

    let mut rows = Vec::with_capacity(bases.len() + 1);
    for (index, (after_ms, base)) in bases.iter().enumerate() {
        let end = bases
            .get(index + 1)
            .map_or(last_end, |(next_after, _)| *next_after);
        if end <= *after_ms {
            return Err(CompileError::StepWithoutDuration {
                after_ms: *after_ms,
            });
        }
        rows.push(Row {
            after_ms: *after_ms,
            duration_ms: end - *after_ms,
            posture: base.posture(),
        });
    }
    if !stow_terminal {
        rows.push(Row {
            after_ms: last_end,
            duration_ms: stow_ms,
            posture: Some(Posture::Stow),
        });
    }
    Ok(rows)
}

/// One overlay window of the compiled schedule.
#[derive(Clone, Copy, Debug, PartialEq)]
struct Window {
    motion_id: u16,
    after_ms: u64,
    duration_ms: u64,
    speed: f64,
}

/// The play steps as windows, with every name resolved and every window inside
/// `horizon_ms`.
///
/// The horizon screen is here rather than at the session because the session's
/// engagement is measured off the plan it receives: a window past the end is not
/// dropped there, it moves the end.
fn overlay_rows(
    script: &MotionScript,
    table: &MotionTable,
    horizon_ms: u64,
) -> Result<Vec<Window>, CompileError> {
    script
        .steps()
        .iter()
        .filter_map(|step| step.action.play().map(|play| (step.after_ms, play)))
        .map(|(after_ms, play)| {
            let entry = table
                .resolve(&play.name)
                .ok_or_else(|| CompileError::UnknownMotion {
                    name: play.name.clone(),
                })?;
            let span_ms = entry.window.span_ms(play.speed);
            if u32::try_from(span_ms).is_err() {
                return Err(CompileError::WindowUnrepresentable {
                    name: play.name.clone(),
                    span_ms,
                });
            }
            let end_ms = after_ms + span_ms;
            if end_ms > horizon_ms {
                return Err(CompileError::OverlayPastHorizon {
                    name: play.name.clone(),
                    end_ms,
                    horizon_ms,
                });
            }
            Ok(Window {
                motion_id: entry.motion_id,
                after_ms,
                duration_ms: span_ms,
                speed: play.speed,
            })
        })
        .collect()
}

/// A millisecond number as the message's fields carry it.
///
/// Every one reaching this is bounded by the wire contract's timeout ceiling or
/// screened above, so the conversion is total in practice and states which fact
/// it rests on rather than saturating quietly.
fn field(ms: u64) -> u32 {
    u32::try_from(ms).expect("a millisecond offset bounded by the script's own timeout ceiling")
}

/// The posture as the schedule's own vocabulary spells it.
fn posture_wire(posture: Posture) -> PostureWire {
    match posture {
        Posture::Up => PostureWire::UP,
        Posture::Stow => PostureWire::STOW,
    }
}

#[cfg(test)]
mod tests {
    use brenn_reachy__cogs__schedule_clk_rs::{PostureWire, StepKindWire};
    use brenn_reachy__cogs__script_clk_rs::ScriptWire;
    use clockwork_rs::SyncTime;
    use motion_proto::{MotionScript, Play, PlayWindow, Posture, Step};

    use super::{CompileError, MAX_OVERLAYS, MAX_STEPS, compile};
    use crate::config::EdgeConfig;
    use crate::names::{MotionEntry, MotionTable};

    /// A round instant, so a number read out of the wrong side of the stamping
    /// is visible.
    const ARRIVAL_NS: i64 = 1_700_000_000_000_000_000;

    /// The id every case here compiles under.
    const SCRIPT_ID: u32 = 7;

    /// The shipped configuration for the machine these cases are about.
    fn config() -> EdgeConfig {
        EdgeConfig::for_pod("reachy00")
    }

    /// A two-motion library: one short, one long, at indices a truncation would
    /// show up in.
    fn table() -> MotionTable {
        MotionTable::of([
            (
                "bench/nod".to_owned(),
                MotionEntry {
                    motion_id: 2,
                    window: PlayWindow {
                        duration_ms: 1000,
                        blend_out_ms: 60,
                    },
                },
            ),
            (
                "bench/tour".to_owned(),
                MotionEntry {
                    motion_id: 4,
                    window: PlayWindow {
                        duration_ms: 4000,
                        blend_out_ms: 120,
                    },
                },
            ),
        ])
    }

    /// A script the wire contract accepts, for this machine.
    fn script(steps: Vec<Step>, timeout_ms: u64) -> MotionScript {
        MotionScript::new("reachy00", 1, steps, timeout_ms).expect("a lawful timeline")
    }

    /// The compiled request, or the refusal.
    fn compiled(steps: Vec<Step>, timeout_ms: u64) -> Result<ScriptWire, CompileError> {
        compile(
            &script(steps, timeout_ms),
            SyncTime::from_nanos(ARRIVAL_NS),
            SCRIPT_ID,
            &config(),
            &table(),
        )
    }

    /// One compiled step, as a case reads it.
    #[derive(Debug, PartialEq, Eq)]
    struct Row {
        after_ms: u32,
        duration_ms: u32,
        kind: StepKindWire,
        posture: PostureWire,
    }

    /// The steps of `message`, in order.
    fn rows(message: &ScriptWire) -> Vec<Row> {
        message
            .steps()
            .iter()
            .map(|step| Row {
                after_ms: step.after_ms(),
                duration_ms: step.duration_ms(),
                kind: step.kind(),
                posture: step.posture(),
            })
            .collect()
    }

    fn up(after_ms: u64) -> Step {
        Step::new(after_ms, Posture::Up)
    }

    fn stow(after_ms: u64) -> Step {
        Step::new(after_ms, Posture::Stow)
    }

    fn base_row(after_ms: u32, duration_ms: u32, posture: PostureWire) -> Row {
        Row {
            after_ms,
            duration_ms,
            kind: StepKindWire::BASE_POSTURE,
            posture,
        }
    }

    #[test]
    fn a_base_step_lasts_until_the_next_one_and_the_last_until_the_closing_stow() {
        let message = compiled(vec![up(0), Step::keep(1000)], 10_000).expect("a lawful script");
        assert_eq!(message.script_id(), SCRIPT_ID);
        assert_eq!(message.arrival(), SyncTime::from_nanos(ARRIVAL_NS));
        assert_eq!(
            rows(&message),
            vec![
                base_row(0, 1000, PostureWire::UP),
                Row {
                    after_ms: 1000,
                    duration_ms: 6000,
                    kind: StepKindWire::BASE_KEEP,
                    posture: PostureWire::STOW,
                },
                base_row(7000, 3000, PostureWire::STOW),
            ],
            "the keep runs to the synthesized stow, which ends at the timeout",
        );
    }

    #[test]
    fn a_schedule_that_does_not_end_stowed_is_given_a_stow_at_the_timeout() {
        let message = compiled(vec![up(500)], 20_000).expect("a lawful script");
        let rows = rows(&message);
        let last = rows.last().expect("a schedule of at least the stow");
        assert_eq!(last.posture, PostureWire::STOW);
        assert_eq!(
            u64::from(last.after_ms) + u64::from(last.duration_ms),
            20_000,
            "the synthesized stow ends exactly at the timeout",
        );
    }

    #[test]
    fn a_script_that_stows_ends_at_its_stow_and_not_at_its_timeout() {
        let message = compiled(vec![up(0), stow(2000)], 13_000).expect("a lawful script");
        assert_eq!(
            rows(&message),
            vec![
                base_row(0, 2000, PostureWire::UP),
                base_row(2000, 3000, PostureWire::STOW),
            ],
            "the horizon is the stow's end; holding the machine stowed under torque \
             to the timeout is the one pinch hazard",
        );
    }

    #[test]
    fn a_closing_stow_may_end_exactly_at_the_timeout_and_not_past_it() {
        let level =
            compiled(vec![up(0), stow(10_000)], 13_000).expect("a stow ending at the ceiling");
        let last = rows(&level).pop().expect("a stow");
        assert_eq!(
            u64::from(last.after_ms) + u64::from(last.duration_ms),
            13_000
        );

        assert_eq!(
            compiled(vec![up(0), stow(10_001)], 13_000),
            Err(CompileError::NoRoomForStow {
                after_ms: 10_001,
                stow_duration_ms: 3000,
                timeout_ms: 13_000,
            }),
            "one millisecond past the ceiling is a drop, never a shortened stow",
        );
    }

    #[test]
    fn a_timeline_with_no_room_for_the_synthesized_stow_is_dropped() {
        assert_eq!(
            compiled(vec![up(7500)], 10_000),
            Err(CompileError::NoRoomForStow {
                after_ms: 7500,
                stow_duration_ms: 3000,
                timeout_ms: 10_000,
            }),
        );
        assert!(
            matches!(
                compiled(vec![up(0)], 2000),
                Err(CompileError::NoRoomForStow { .. })
            ),
            "a timeout shorter than the stow itself leaves no room either",
        );
    }

    #[test]
    fn a_keep_after_the_closing_stow_is_dropped() {
        assert_eq!(
            compiled(vec![up(0), stow(2000), Step::keep(6000)], 600_000),
            Err(CompileError::KeepAfterStow {
                after_ms: 6000,
                stow_after_ms: 2000,
            }),
            "the keep holds the stow, so honouring it would run the machine folded under \
             torque to the sender's ceiling",
        );
        assert!(
            compiled(vec![up(0), Step::keep(2000), stow(6000)], 600_000).is_ok(),
            "a keep before the stow holds the raise, which is the ordinary shape",
        );
        assert!(
            compiled(vec![up(0), stow(2000), up(6000)], 600_000).is_ok(),
            "a stow the timeline comes back up from closes nothing",
        );
    }

    #[test]
    fn a_stow_in_the_middle_takes_the_room_the_next_step_leaves_it() {
        // Only the stow that *closes* a timeline is screened for room: an
        // intermediate one is a fold the schedule then moves off again, and the
        // mover shapes what it is given. The configured 3000 ms is the closing
        // budget, not a floor on every stow.
        let message = compiled(vec![up(0), stow(1000), up(2000)], 20_000).expect("a lawful script");
        assert_eq!(
            rows(&message),
            vec![
                base_row(0, 1000, PostureWire::UP),
                base_row(1000, 1000, PostureWire::STOW),
                base_row(2000, 15_000, PostureWire::UP),
                base_row(17_000, 3000, PostureWire::STOW),
            ],
            "the middle stow gets the 1000 ms the next step leaves it; only the closing \
             stow is the configured one",
        );
    }

    #[test]
    fn the_tightest_timeline_the_ascending_offsets_admit_still_owns_time() {
        // The guard against a step of no duration is unreachable while offsets
        // ascend strictly and the closing stow has its own room screen. This is
        // the closest shape to it that exists: a one-millisecond keep, and a
        // synthesized stow starting the instant after.
        let message = compiled(vec![up(0), Step::keep(1)], 3002).expect("a lawful script");
        assert_eq!(
            rows(&message),
            vec![
                base_row(0, 1, PostureWire::UP),
                Row {
                    after_ms: 1,
                    duration_ms: 1,
                    kind: StepKindWire::BASE_KEEP,
                    posture: PostureWire::STOW,
                },
                base_row(2, 3000, PostureWire::STOW),
            ],
        );
        assert!(
            matches!(
                compiled(vec![up(0), Step::keep(1)], 3001),
                Err(CompileError::NoRoomForStow { .. })
            ),
            "one millisecond tighter is the room screen, which is what keeps the \
             no-duration guard out of reach",
        );
    }

    #[test]
    fn a_script_that_commands_no_posture_is_dropped() {
        assert_eq!(compiled(vec![], 5000), Err(CompileError::NoPosture));
        assert_eq!(
            compiled(vec![Step::keep(0), Step::keep(1000)], 5000),
            Err(CompileError::NoPosture),
            "`keep` alone would torque a resting head through a pointless cycle",
        );
    }

    #[test]
    fn a_play_step_becomes_a_window_the_speed_scales_only_half_of() {
        let message = compiled(
            vec![up(0), Step::play(500, Play::at_speed("bench/nod", 2.0))],
            10_000,
        )
        .expect("a lawful script");
        let windows: Vec<_> = message.overlays().iter().collect();
        assert_eq!(windows.len(), 1);
        let window = &windows[0];
        assert_eq!(window.motion_id(), 2);
        assert_eq!(window.after_ms(), 500);
        assert_eq!(
            window.duration_ms(),
            560,
            "the motion's own clock halves; the blend-out does not",
        );
        assert!((window.speed() - 2.0).abs() < f64::EPSILON);
        assert!((window.gain() - 1.0).abs() < f64::EPSILON);

        let slow = compiled(
            vec![up(0), Step::play(500, Play::at_speed("bench/nod", 0.5))],
            10_000,
        )
        .expect("a lawful script");
        assert_eq!(
            slow.overlays().get(0).expect("one window").duration_ms(),
            2060,
        );
    }

    #[test]
    fn a_window_no_field_could_carry_is_dropped() {
        let absurd = MotionTable::of([(
            "bench/absurd".to_owned(),
            MotionEntry {
                motion_id: 3,
                window: PlayWindow {
                    duration_ms: u64::from(u32::MAX) + 1,
                    blend_out_ms: 0,
                },
            },
        )]);
        let script = script(
            vec![up(0), Step::play(500, Play::new("bench/absurd"))],
            10_000,
        );
        assert_eq!(
            compile(
                &script,
                SyncTime::from_nanos(ARRIVAL_NS),
                SCRIPT_ID,
                &config(),
                &absurd,
            ),
            Err(CompileError::WindowUnrepresentable {
                name: "bench/absurd".to_owned(),
                span_ms: u64::from(u32::MAX) + 1,
            }),
            "the field screen runs before the window is added to its offset",
        );
    }

    #[test]
    fn a_window_may_end_exactly_at_the_horizon_and_not_past_it() {
        // The library's long motion occupies 4120 ms at 1.0x, so a play at 5880
        // ends at 10_000 -- the timeout, which is where the synthesized stow
        // ends and so where the schedule's horizon is.
        let level = compiled(
            vec![up(0), Step::play(5880, Play::new("bench/tour"))],
            10_000,
        )
        .expect("a window ending at the horizon");
        let window = level.overlays().get(0).expect("one window");
        assert_eq!(
            u64::from(window.after_ms()) + u64::from(window.duration_ms()),
            10_000
        );

        assert_eq!(
            compiled(
                vec![up(0), Step::play(5881, Play::new("bench/tour"))],
                10_000
            ),
            Err(CompileError::OverlayPastHorizon {
                name: "bench/tour".to_owned(),
                end_ms: 10_001,
                horizon_ms: 10_000,
            }),
            "one millisecond past the horizon is a drop, never a trimmed window",
        );
    }

    #[test]
    fn a_window_is_screened_against_an_early_stow_and_not_against_the_timeout() {
        // A script that stows for itself ends at the stow -- here 3000 + 3000 --
        // so a window the timeout would have admitted still outlives the
        // schedule.
        assert_eq!(
            compiled(
                vec![up(0), stow(3000), Step::play(5500, Play::new("bench/nod"))],
                30_000
            ),
            Err(CompileError::OverlayPastHorizon {
                name: "bench/nod".to_owned(),
                end_ms: 6560,
                horizon_ms: 6000,
            }),
        );
        assert!(
            compiled(
                vec![up(0), stow(3000), Step::play(4900, Play::new("bench/nod"))],
                30_000
            )
            .is_ok(),
            "the same window inside the stow's end is a schedule the session can run",
        );
    }

    #[test]
    fn the_row_caps_are_the_capacities_the_message_carries() {
        let message = ScriptWire::new();
        assert_eq!(message.steps().capacity(), MAX_STEPS);
        assert_eq!(message.overlays().capacity(), MAX_OVERLAYS);
    }

    #[test]
    fn a_motion_the_library_does_not_hold_is_dropped() {
        assert_eq!(
            compiled(
                vec![up(0), Step::play(500, Play::new("bench/absent"))],
                10_000
            ),
            Err(CompileError::UnknownMotion {
                name: "bench/absent".to_owned()
            }),
        );
    }

    #[test]
    fn more_windows_than_the_schedule_holds_is_a_drop() {
        let mut steps = vec![up(0)];
        for index in 0..=MAX_OVERLAYS {
            let after = 500 + 100 * index as u64;
            steps.push(Step::play(after, Play::new("bench/nod")));
        }
        assert_eq!(
            compiled(steps, 20_000),
            Err(CompileError::TooManyOverlays {
                overlays: MAX_OVERLAYS + 1
            }),
        );
    }

    /// `count` base steps a second apart, alternating so the offsets ascend and
    /// nothing collapses.
    fn ladder(count: usize) -> Vec<Step> {
        (0..count)
            .map(|index| {
                let after = 1000 * index as u64;
                if index % 2 == 0 {
                    up(after)
                } else {
                    Step::keep(after)
                }
            })
            .collect()
    }

    #[test]
    fn the_step_screen_counts_the_synthesized_stow() {
        let fifteen =
            compiled(ladder(MAX_STEPS - 1), 60_000).expect("fifteen and a stow is sixteen");
        assert_eq!(rows(&fifteen).len(), MAX_STEPS);

        assert_eq!(
            compiled(ladder(MAX_STEPS), 60_000),
            Err(CompileError::TooManySteps {
                steps: MAX_STEPS + 1
            }),
            "sixteen steps needing a stow is seventeen, which the schedule does not hold",
        );

        let mut terminal = ladder(MAX_STEPS - 1);
        terminal.push(stow(1000 * (MAX_STEPS - 1) as u64));
        let sixteen = compiled(terminal, 60_000).expect("sixteen steps ending stowed");
        assert_eq!(
            rows(&sixteen).len(),
            MAX_STEPS,
            "a script that stows for itself needs no synthesis, so sixteen fit",
        );
    }
}
