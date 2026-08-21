//! The overlay layer of the decision tick: what plays over the base, and the
//! base itself while something is playing.
//!
//! Three things live here, and they are one subject:
//!
//! - **The window screen.** A schedule's overlay windows arrive as whatever the
//!   session put in them. [`Windows`] is the screened form: a window naming a
//!   motion the library does not have, asking for a gain or a speed that is not
//!   one, asking for a speed past what the motion's clips admit, or spanning no
//!   time at all is refused and counted, and the rest play.
//!   An overlay is presence, never safety, so a refusal costs the run an overlay
//!   and nothing else.
//! - **The players.** [`Overlays`] is the set of playing motions for one
//!   execution: picked up out of the state's own rows, advanced one period per
//!   sample, and left there. A player is `reachy-clips`' and its state is the
//!   row it borrows, so nothing here crosses one; what this module owns are the
//!   two fields beside it — whether the row is occupied and which motion the
//!   window named. A gain is the window's and is read from it every execution,
//!   never held in the row.
//! - **The base.** Ordinarily the tick owns the base: the cog commands a move
//!   and the tick shapes and samples it. A composed setpoint is commanded per
//!   period instead — [`MotionCommand::Track`](reachy_motion::tick::MotionCommand)
//!   — which abandons any move the tick is running. So for as long as a window
//!   is open the cog carries the base itself: [`Base`] is where it stands and
//!   the move it is on, sampled by the same [`Trajectory`] the tick would have
//!   sampled, from the same seed.
//!
//! Nothing here is a safety check and nothing here clamps. A composed setpoint
//! faces the tick's envelope check and per-tick step bound like any other
//! commanded target, and that check is the gate. What this module refuses, it
//! refuses outright: a window that will not play plays nothing.
//!
//! No clock is read and nothing is allocated. Every instant arrives as an
//! argument, and the four rows are a fixed array whose length is the schema's.

use brenn_reachy__clips__player_clk_rs::{ClipPlayerSnap, ClipPlayerSnapWire};
use brenn_reachy__cogs__mover_clk_rs::{BaseSnap, BaseSnapWire};
use brenn_reachy__cogs__schedule_clk_rs::{OverlayWindowWire, SessionScheduleWire};
use clockwork_rs::Duration as SlotDuration;
use clockwork_rs::Invalid;
use core::time::Duration;
use reachy_clips::compose::OverlaySample;
use reachy_clips::config::{MotionView, MotionViewError, ValidatedLibrary};
use reachy_clips::format::Channel;
use reachy_clips::player::{ClipPlayer, PlayerStateError};
use reachy_motion::joints::JointTargets;
use reachy_motion::snap::{DurationError, PoseSnapshotError, duration_from_nanos};
use reachy_motion::traj::{
    SeedError, Trajectory, clear_seed, read_seed, targets_of, write_seed, write_targets,
};
use thiserror::Error;

/// How many overlays may play at once: the schema's own row count, checked
/// against it by a case rather than restated.
pub const MAX_OVERLAYS: usize = 4;

/// Why an overlay slot, or a window, is no overlay.
///
/// The slot refusals are the boundary's, in the shape every other crossing in
/// this package uses: a slot is shared memory holding whatever a publisher or an
/// older run left, so a value that is not a player is refused rather than
/// repaired. The window refusals are about a script: a window asking for
/// something no motion can do.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum OverlayError {
    /// The slot's numbers are not a player of this motion.
    #[error("the overlay slot holds no player: {0}")]
    Player(#[from] PlayerStateError),

    /// The base's own pose is not one.
    #[error("the base slot holds no pose: {0}")]
    Pose(#[from] PoseSnapshotError),

    /// The base's move is not a path.
    #[error("the base slot holds no path: {0}")]
    Base(#[from] SeedError),

    /// The base's move is bytes no move reads from.
    #[error("the base slot's move is no message: {0}")]
    Unreadable(#[from] Invalid),

    /// How far into the move is not a length of time.
    #[error("the base slot's clock is not one: {0}")]
    Elapsed(#[from] DurationError),

    /// The gain is not a share of a delta.
    ///
    /// Refused rather than clamped: a weight above one is a delta amplified past
    /// what the blend floor that keeps a ramp inside the step bound was derived
    /// for, and a negative one plays the motion inside out.
    #[error("overlay gain {gain} is not in [0, 1]")]
    Gain {
        /// What the window held.
        gain: f64,
    },

    /// The speed is not a rate a motion can be played at.
    #[error("overlay speed {speed} is not finite and positive")]
    Speed {
        /// What the window held.
        speed: f64,
    },

    /// The speed is past what the motion's own clips admit.
    ///
    /// Refused rather than played for the reason the gain is: the ceiling is
    /// derived at load from each clip's frame-to-frame deltas against the
    /// machine's step bounds and by what the flattening already spent of it, and
    /// above it consecutive composed setpoints step further than the tick
    /// accepts. Played, the overlay is not slowed but shredded -- a refused
    /// setpoint and a fault report every period, while the player's clock runs
    /// on.
    #[error("overlay speed {speed} is past motion {motion_id}'s ceiling of {ceiling}")]
    PastCeiling {
        /// Which motion.
        motion_id: u16,
        /// What the window held.
        speed: f64,
        /// What the motion admits.
        ceiling: f64,
    },

    /// The motion the window names is not one the library will play.
    ///
    /// A motion id the library does not have reaches here, which is a window
    /// naming nothing. The rest of what the view refuses — a motion whose
    /// segments do not describe a playable walk — is unreachable through a
    /// handle the library walk hands back, which is the only way to one: the
    /// walk establishes every motion. Those arms are the backstop for a host
    /// that obtained the handle some other way.
    #[error("the motion the overlay names will not play: {0}")]
    Unplayable(#[from] MotionViewError),

    /// The window spans no time.
    #[error("overlay window ends at {end_ns} ns, at or before its start {start_ns} ns")]
    EmptyWindow {
        /// When it opened.
        start_ns: i64,
        /// When it closed.
        end_ns: i64,
    },
}

/// One overlay window, screened.
///
/// Every number is one a player can be built from, which is what the type says:
/// a `Window` exists only for a window this build will play, so the play path
/// has nothing left to check and nothing left to refuse.
///
/// The motion it names is carried rather than re-resolved: the screen already
/// walked it to read the ceiling, the walk is over an immutable message and
/// answers the same thing every time, and the play path would otherwise pay it
/// again for every open window, every control period.
#[derive(Clone, Copy, Debug)]
pub struct Window<'a> {
    /// Which motion, as its index in the configured library's motions.
    pub motion_id: u16,
    /// When the window opens, inclusive, nanoseconds.
    pub start_ns: i64,
    /// When it closes, exclusive, nanoseconds.
    pub end_ns: i64,
    /// How much of the motion's delta applies at full weight, in `[0, 1]`.
    pub gain: f64,
    /// How fast the motion plays, a multiple of the rate its clips were
    /// authored at.
    pub speed: f64,
    /// The motion, as the screen established it.
    motion: MotionView<'a>,
}

impl<'a> Window<'a> {
    /// Screen one window against `library`, the configured library.
    ///
    /// # Errors
    ///
    /// [`OverlayError`] for a motion the library does not have, a gain or a
    /// speed that is not one, a speed past the motion's ceiling, or a window
    /// spanning no time.
    pub fn screen(
        window: &OverlayWindowWire,
        library: &ValidatedLibrary<'a>,
    ) -> Result<Self, OverlayError> {
        let (motion_id, gain, speed) = (window.motion_id(), window.gain(), window.speed());
        let (start_ns, end_ns) = (window.start().as_nanos(), window.end().as_nanos());
        let motion = library.playable_motion(usize::from(motion_id))?;
        if !gain.is_finite() || !(0.0..=1.0).contains(&gain) {
            return Err(OverlayError::Gain { gain });
        }
        if !speed.is_finite() || speed <= 0.0 {
            return Err(OverlayError::Speed { speed });
        }
        let ceiling = motion.max_speed();
        if speed > ceiling {
            return Err(OverlayError::PastCeiling {
                motion_id,
                speed,
                ceiling,
            });
        }
        if end_ns <= start_ns {
            return Err(OverlayError::EmptyWindow { start_ns, end_ns });
        }
        Ok(Self {
            motion_id,
            start_ns,
            end_ns,
            gain,
            speed,
            motion,
        })
    }

    /// Whether the window covers `now`, half-open — the same convention a step
    /// uses, so two windows may share an edge without either owning it twice.
    #[must_use]
    pub fn covers(&self, now_ns: i64) -> bool {
        (self.start_ns..self.end_ns).contains(&now_ns)
    }

    /// How far into the window `now` is, or zero before it opens.
    ///
    /// What a player joining a window already in progress is built at: the
    /// timeline is authoritative in absolute time, so an overlay whose start has
    /// passed is picked up where it should be rather than replayed from the
    /// top.
    #[must_use]
    pub fn joined_at(&self, now_ns: i64) -> Duration {
        Duration::from_nanos(u64::try_from(now_ns.saturating_sub(self.start_ns)).unwrap_or(0))
    }
}

/// The schedule's overlay windows, screened, in the rows they arrived in.
///
/// The row is the window's position in the schedule and is what a player is kept
/// under between executions: a window keeps its row for as long as it is open,
/// which is what lets a motion be picked up again rather than restarted every
/// period. A refused window leaves its row empty and is counted.
#[derive(Clone, Copy, Debug, Default)]
pub struct Windows<'a> {
    rows: [Option<Window<'a>>; MAX_OVERLAYS],
    refused: u64,
}

impl<'a> Windows<'a> {
    /// Screen every window `schedule` carries.
    ///
    /// Windows past the fourth are refused with the rest. Unreachable while the
    /// two capacities agree -- a schedule holds four windows and the layer has
    /// four rows -- but the two are stated in different places, so the guard is
    /// what a disagreement costs: an overlay, and not a row indexed past its
    /// array.
    #[must_use]
    pub fn of(schedule: &SessionScheduleWire, library: &ValidatedLibrary<'a>) -> Self {
        let mut screened = Self::default();
        for (row, window) in schedule.overlays().iter().enumerate() {
            match (row < MAX_OVERLAYS, Window::screen(window, library)) {
                (true, Ok(window)) => screened.rows[row] = Some(window),
                _ => screened.refused += 1,
            }
        }
        screened
    }

    /// The window in `row`, if there is one this build will play.
    #[must_use]
    pub fn row(&self, row: usize) -> Option<Window<'a>> {
        self.rows.get(row).copied().flatten()
    }

    /// How many windows were refused.
    #[must_use]
    pub fn refused(&self) -> u64 {
        self.refused
    }

    /// Whether any window covers `now`.
    ///
    /// What decides whether the cog owns the base this period: an open window is
    /// a composed setpoint, and a composed setpoint is the cog's own base.
    #[must_use]
    pub fn any_covers(&self, now_ns: i64) -> bool {
        self.rows
            .iter()
            .flatten()
            .any(|window| window.covers(now_ns))
    }

    /// The last instant any window is open until, or `None` where none is left.
    ///
    /// The end of the stream when the base steps have run out: an overlay
    /// outlives the last step until its window closes.
    #[must_use]
    pub fn last_end(&self) -> Option<i64> {
        self.rows.iter().flatten().map(|window| window.end_ns).max()
    }
}

/// Where the base stands, while the cog is the one sampling it.
#[derive(Clone, Debug, PartialEq)]
pub struct Base {
    /// The configuration an overlay is composed over this period.
    pub targets: JointTargets,
    /// The move in flight, or `None` where the base is held.
    pub path: Option<BasePath>,
}

/// A base transition and how far into it the run is.
#[derive(Clone, Debug, PartialEq)]
pub struct BasePath {
    /// The path itself, as the tick shaped it.
    pub path: Trajectory,
    /// How far into it, on its own clock.
    pub elapsed: Duration,
}

impl Base {
    /// A base held at `targets`, which is what the handover from the tick to
    /// this cog produces when the tick was holding.
    #[must_use]
    pub fn held(targets: JointTargets) -> Self {
        Self {
            targets,
            path: None,
        }
    }

    /// Where the base stands now, with the run then a period older.
    ///
    /// The move is sampled exactly as the tick would have sampled it: a base
    /// taken over mid-flight carries on along the same path rather than along a
    /// second plan of the same move. A path that has run out is dropped and the
    /// base holds where it left off.
    pub fn step(&mut self, period: Duration) -> JointTargets {
        let Some(run) = self.path.take() else {
            return self.targets;
        };
        run.path.sample(run.elapsed, &mut self.targets);
        if !run.path.done(run.elapsed) {
            let elapsed = run.elapsed.saturating_add(period);
            self.path = Some(BasePath {
                path: run.path,
                elapsed,
            });
        }
        self.targets
    }

    /// Send the base along `trajectory` from wherever it stands.
    ///
    /// What a posture step arriving while the cog holds the base does. The path
    /// is planned by the caller through the motion library's own planner, so the
    /// clock is floored, the antenna directions are resolved and the envelope is
    /// judged exactly as commanding the move outright would have judged them.
    pub fn retarget(&mut self, trajectory: &Trajectory) {
        self.path = Some(BasePath {
            path: trajectory.clone(),
            elapsed: Duration::ZERO,
        });
    }
}

/// Write the base, or that the tick owns it.
///
/// The slot is cleared first, so a base written over an earlier one carries
/// nothing of it and "the tick owns it" is the cleared row itself.
///
/// Total: a path in hand is one that was shaped, so its clocks are ones a slot
/// holds and there is nothing left to refuse on the way out.
pub fn write_base(out: &mut BaseSnapWire, base: Option<&Base>) {
    let slot = out.clear_valid();
    let Some(base) = base else {
        return;
    };
    write_targets(&mut slot.targets, &base.targets);
    match base.path.as_ref() {
        Some(run) => write_seed(&mut slot.path, &run.path),
        None => clear_seed(&mut slot.path),
    }
    slot.elapsed = SlotDuration::from_nanos(
        base.path
            .as_ref()
            .map(|run| i64::try_from(run.elapsed.as_nanos()).unwrap_or(i64::MAX))
            .unwrap_or(0),
    );
    slot.owned = true.into();
}

/// The base those fields describe, or `None` where the tick owns it.
///
/// One validation at the top, which is this crossing's boundary; everything
/// under it is a plain field.
///
/// # Errors
///
/// [`OverlayError::Unreadable`] for bytes that do not read as a base at all,
/// [`OverlayError::Base`] for a move that is not a path, [`OverlayError::Pose`]
/// for targets that are not a command set, and [`OverlayError::Elapsed`] for an
/// elapsed time that is not a length of time.
pub fn read_base(slot: &BaseSnapWire) -> Result<Option<Base>, OverlayError> {
    let slot: &BaseSnap = slot.validate()?;
    if !bool::from(slot.owned) {
        return Ok(None);
    }
    let path = read_seed(&slot.path)?;
    let elapsed = duration_from_nanos(slot.elapsed.as_nanos())?;
    Ok(Some(Base {
        targets: targets_of(&slot.targets)?,
        path: path.map(|path| BasePath { path, elapsed }),
    }))
}

/// What one period of playing overlays contributes, in row order.
///
/// Row order is wire step order, which is the order the compositor folds in.
/// Rotations do not commute, so the order is fixed and deterministic rather than
/// pretended away.
#[derive(Clone, Copy, Debug)]
pub struct Samples {
    rows: [OverlaySample; MAX_OVERLAYS],
    len: usize,
}

impl Samples {
    /// Nothing contributed yet.
    fn none() -> Self {
        Self {
            rows: [OverlaySample::silent(); MAX_OVERLAYS],
            len: 0,
        }
    }

    /// The contributions, in the order they are to be folded.
    #[must_use]
    pub fn as_slice(&self) -> &[OverlaySample] {
        &self.rows[..self.len]
    }

    /// Whether anything contributed at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len == 0
    }

    fn push(&mut self, sample: OverlaySample) {
        // Unreachable: there is one row per window and one window per row. The
        // guard is here so a row count that stopped agreeing drops a
        // contribution rather than indexing past the array.
        if self.len < MAX_OVERLAYS {
            self.rows[self.len] = sample;
            self.len += 1;
        }
    }
}

/// One row of the overlay layer: a window's motion, playing.
///
/// A row that is playing nothing is `None` beside this rather than a variant of
/// it: a player is a quarter of a kilobyte and the empty case is a flag, so an
/// enum of the two would carry the player's bulk into every empty row.
struct PlayingRow<'a> {
    /// The share of its delta being applied. Held beside the player rather than
    /// read back off the row: it is the window's fact and the player's borrow of
    /// the row is exclusive.
    gain: f64,
    player: ClipPlayer<'a, 'a>,
}

/// How many overlays this execution could not take as it found them.
///
/// The rows only. What the screen refused is the screen's own count
/// ([`Windows::refused`]), taken once when the schedule arrives rather than
/// once per period: a window the screen refused never reaches a row.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Refusals {
    /// Rows the slot held that would not read back as a player of the motion
    /// the window names. The window plays on from a fresh join, which is what a
    /// window opening does anyway.
    pub players: u64,
}

/// The overlays playing this execution.
pub struct Overlays<'a> {
    rows: [Option<PlayingRow<'a>>; MAX_OVERLAYS],
}

/// Leave the row holding no player, writing only where it holds one.
///
/// Guarded rather than written every period: a row is a quarter of a kilobyte of
/// shared state slot, an idle layer would rewrite four of them at the control
/// rate forever, and every write after the first says what the row already said.
/// Bytes that are no player are emptied, because whatever they are they are not
/// the absence of one.
fn vacate(slot: &mut ClipPlayerSnapWire) {
    let empty = slot.validate().is_ok_and(|state| !bool::from(state.active));
    if !empty {
        *slot = ClipPlayerSnapWire::new();
    }
}

/// The row's state to write through, and whether the bytes that were there are
/// the ones being answered for.
///
/// `false` means the row held no player at all and has been emptied for one; the
/// caller counts that as a refusal. Validated twice rather than once because a
/// borrow taken by the failing call outlives the arm that would clear the slot —
/// the second call is what the borrow checker costs, not a second question — and
/// the `expect` sits against the clear that discharges it.
///
/// No bytes reach that arm today: every field of a player is a number, a flag or
/// a nested record of those, and none of them has a pattern validation refuses.
/// It stands for the field that does — an enum or a counted container — and is
/// the boundary refusal the moment one is added, which is why the clear is here
/// rather than in a caller that would then have to remember it.
fn readable(slot: &mut ClipPlayerSnapWire) -> (&mut ClipPlayerSnap, bool) {
    let kept = slot.validate().is_ok();
    if !kept {
        *slot = ClipPlayerSnapWire::new();
    }
    (
        slot.validate_mut()
            .expect("a row that validated, or a cleared one"),
        kept,
    )
}

impl<'a> Overlays<'a> {
    /// Take up the layer: the windows open at `now`, played by the players the
    /// slot holds where it holds one of the right motion.
    ///
    /// A row whose window has closed, or which never had one, plays nothing —
    /// and is left holding no player, so a closed window leaves no player
    /// behind. A row the slot cannot answer for is counted and joined afresh at
    /// the window's own offset: the timeline is authoritative in absolute time,
    /// so a fresh join eases onto the delta where the window says the motion
    /// should be.
    ///
    /// The window's gain and speed are read from the window, not from the row:
    /// a row picked up keeps the speed it was joined at, because the clock it
    /// has already run is in those units, while the gain applies afresh every
    /// execution and is never held.
    ///
    /// TODO(overlay-fade-continuity): a row vacates on the first execution its
    /// window does not cover, so a window that closes while its player is still
    /// carrying weight drops the whole weighted delta in one control period.
    /// The invariant owed is that a playing clip's contribution reaches zero
    /// before its row vacates; which layer enforces it is the session design's,
    /// since it owns both the schedule author and this layer's contract.
    pub fn take_up(
        rows: &'a mut [ClipPlayerSnapWire],
        windows: &Windows<'a>,
        now_ns: i64,
    ) -> (Self, Refusals) {
        let mut refusals = Refusals::default();
        let mut layer = Self {
            rows: [None, None, None, None],
        };
        for (row, slot) in rows.iter_mut().enumerate().take(MAX_OVERLAYS) {
            let Some(window) = windows.row(row).filter(|window| window.covers(now_ns)) else {
                // Emptied rather than left: a player from a window that closed
                // is still a player, and `active` is the only thing saying it is
                // over.
                vacate(slot);
                continue;
            };
            let view = window.motion;
            let (state, kept) = readable(slot);
            if !kept {
                refusals.players += 1;
            }
            let picked_up = bool::from(state.active) && state.motion_id == window.motion_id;
            let refused = picked_up && ClipPlayer::resumable(&view, state).is_err();
            if refused {
                refusals.players += 1;
            }
            let player = if picked_up && !refused {
                ClipPlayer::over(view, state)
            } else {
                // A row holding no player, one of another motion, or one this
                // build cannot pick up: the window starts afresh at its own
                // offset, since the timeline is authoritative in absolute time.
                state.active = true.into();
                state.motion_id = window.motion_id;
                ClipPlayer::joining_at(view, window.speed, window.joined_at(now_ns), state)
            };
            layer.rows[row] = Some(PlayingRow {
                gain: window.gain,
                player,
            });
        }
        (layer, refusals)
    }

    /// Whether anything is playing.
    #[must_use]
    pub fn any(&self) -> bool {
        self.rows.iter().any(Option::is_some)
    }

    /// Advance every playing row by one period and collect what it contributes.
    ///
    /// The window's gain scales the player's own weights, which is what a gain
    /// is: the share of the motion's delta that applies where the motion is at
    /// full blend. A player whose motion and fade-out are both over contributes
    /// nothing and is left in place — its row holds the finished player it is,
    /// so nothing restarts it.
    pub fn sample(&mut self, period: Duration) -> Samples {
        let mut samples = Samples::none();
        for row in &mut self.rows {
            let Some(PlayingRow { gain, player, .. }) = row else {
                continue;
            };
            let Some(mut sample) = player.advance(period) else {
                continue;
            };
            for channel in Channel::ALL {
                let weight = sample.weights.get(channel) * *gain;
                sample.weights.set(channel, weight);
            }
            samples.push(sample);
        }
        samples
    }
}

#[cfg(test)]
mod tests {
    //! The overlay layer: what a window screens to, what a player's slot holds,
    //! and where the base stands while something is playing.

    use super::*;

    use brenn_reachy__cogs__config_clk_rs::{ClipLibraryConfig, ClipLibraryConfigWire};
    use clockwork_rs::SyncTime;
    use reachy_clips::config::{ValidatedLibrary, write_clip};
    use reachy_clips::format::{Channel as Ch, Clip, ClipDoc, FrameDoc};
    use reachy_clips::speed::ClipLimits;
    use reachy_motion::FLOOR_TICK_HZ;
    use reachy_motion::joints::JointStep;
    use reachy_motion::postures::{neutral_targets, stow_pose_targets};
    use reachy_motion::traj::{MoveDurations, TrajectoryError, WarpKind};

    /// One nominal period: the grid a clip is authored on and the one a player is
    /// advanced by.
    const PERIOD: Duration = Duration::from_millis(20);

    /// Generous step bounds, so a fixture's round numbers load as written. What
    /// is under test is the layer, not the speed derivation.
    fn limits() -> ClipLimits {
        ClipLimits {
            max_step: JointStep {
                legs: 100.0,
                body_yaw: 100.0,
                antennas: 100.0,
            },
            ..ClipLimits::default()
        }
    }

    /// A clip driving all three channels, whose frames walk so that a frame
    /// confused with its neighbour shows up.
    fn doc(name: &str, frames: usize) -> ClipDoc {
        ClipDoc {
            version: 1,
            kind: "clip".to_owned(),
            name: name.to_owned(),
            description: None,
            channels: vec![Ch::Head, Ch::BodyYaw, Ch::Antennas],
            frame_hz: FLOOR_TICK_HZ,
            max_speed: 2.0,
            blend_in_ms: Some(40),
            blend_out_ms: Some(60),
            frames: (0..frames)
                .map(|index| {
                    let step = index as f64;
                    let angle = 0.001 * step;
                    FrameDoc {
                        dt: Some([0.0001 * step, 0.0002 * step, 0.0003 * step]),
                        dq: Some([(angle / 2.0).cos(), 0.0, 0.0, (angle / 2.0).sin()]),
                        body_yaw: Some(0.002 * step),
                        antennas: Some([0.003 * step, -0.003 * step]),
                    }
                })
                .collect(),
        }
    }

    /// A two-clip library: id 0 walks all three channels, id 1 is a shorter one
    /// under another name, so a row reused for another motion has one to be
    /// reused for. Each clip is a one-segment motion, which is what the emitter
    /// writes, and motion 2 composes the two in the other order — so the two id
    /// spaces are different tables here rather than the same numbers twice.
    fn library() -> Box<ClipLibraryConfigWire> {
        let clips = [
            Clip::from_doc(doc("walk", 20), &limits()).expect("fixture loads"),
            Clip::from_doc(doc("short", 6), &limits()).expect("fixture loads"),
        ];
        let mut out = Box::new(ClipLibraryConfigWire::new());
        {
            let message = out.clear_valid();
            for clip in &clips {
                let slot = message.clips.try_grow().expect("two clips fit");
                write_clip(clip, slot).expect("fixture fits");
            }
            for clip_id in 0..u16::try_from(clips.len()).expect("two clips") {
                let motion = message.motions.try_grow().expect("two motions fit");
                motion.lead_gap_ms = 0;
                let segment = motion.segments.try_grow().expect("one segment fits");
                segment.clip_id = clip_id;
                segment.speed = 1.0;
                segment.gap_after_ms = 0;
            }
            let composed = message.motions.try_grow().expect("a third motion fits");
            composed.lead_gap_ms = 0;
            for clip_id in [1, 0] {
                let segment = composed.segments.try_grow().expect("two segments fit");
                segment.clip_id = clip_id;
                segment.speed = 1.0;
                segment.gap_after_ms = 0;
            }
        }
        out
    }

    /// The validated form of the fixture library, which is what a host holds.
    fn read(library: &ClipLibraryConfigWire) -> &ClipLibraryConfig {
        library.validate().expect("a written library validates")
    }

    /// A schedule carrying `windows`, in the rows they are given in.
    fn schedule(windows: &[(u16, i64, i64, f64, f64)]) -> SessionScheduleWire {
        let mut message = SessionScheduleWire::new();
        {
            let mut rows = message.overlays_mut();
            rows.clear();
            for (motion_id, start_ns, end_ns, gain, speed) in windows {
                let row: &mut OverlayWindowWire =
                    rows.try_grow().expect("a schedule of four windows");
                row.set_motion_id(*motion_id);
                row.set_start(SyncTime::from_nanos(*start_ns));
                row.set_end(SyncTime::from_nanos(*end_ns));
                row.set_gain(*gain);
                row.set_speed(*speed);
            }
        }
        message
    }

    /// The one window a fixture schedule carries.
    fn one(message: &SessionScheduleWire) -> &OverlayWindowWire {
        message.overlays().iter().next().expect("one window")
    }

    /// One window, screened over the fixture library of three motions.
    ///
    /// A screened window carries the motion it was screened against, so the
    /// library outlives it: the fixture's is leaked rather than dropped here,
    /// which is what a caller taking the window away needs and costs a test
    /// process two clips.
    fn screened(
        motion_id: u16,
        start_ns: i64,
        end_ns: i64,
        gain: f64,
        speed: f64,
    ) -> Window<'static> {
        let message = schedule(&[(motion_id, start_ns, end_ns, gain, speed)]);
        let library: &'static ClipLibraryConfigWire = Box::leak(library());
        let validated = ValidatedLibrary::of(read(library)).expect("the fixture plays");
        Window::screen(one(&message), &validated).expect("the fixture window plays")
    }

    /// What a window's refusal is, over the fixture library of three motions.
    fn refusal(motion_id: u16, start_ns: i64, end_ns: i64, gain: f64, speed: f64) -> OverlayError {
        let message = schedule(&[(motion_id, start_ns, end_ns, gain, speed)]);
        let library = library();
        let validated = ValidatedLibrary::of(read(&library)).expect("the fixture plays");
        Window::screen(one(&message), &validated).expect_err("the fixture window is refused")
    }

    /// The ceiling the two single-clip fixture motions carry.
    fn ceiling() -> f64 {
        let library = library();
        let validated = ValidatedLibrary::of(read(&library)).expect("the fixture plays");
        validated
            .playable_motion(0)
            .expect("the fixture motion plays")
            .max_speed()
    }

    /// Every number a window carries reaches the screened form, unchanged.
    #[test]
    fn a_window_this_build_plays_crosses_the_screen_whole() {
        let window = screened(1, 1_000, 3_000, 0.25, 1.5);
        assert_eq!(window.motion_id, 1);
        assert_eq!(window.start_ns, 1_000);
        assert_eq!(window.end_ns, 3_000);
        assert_eq!(window.gain, 0.25);
        assert_eq!(window.speed, 1.5);
        // The motion it carries is the one it names: the second fixture clip is
        // the shorter of the two.
        assert_eq!(window.motion.duration_s(), 6.0 / FLOOR_TICK_HZ);
    }

    /// A window names a motion, not a clip: the third fixture motion is two
    /// segments over the two clips in the other order, so a screen that resolved
    /// the id through the clips would carry the wrong walk — and answer for a
    /// duration and a ceiling that are no single clip's.
    #[test]
    fn a_window_naming_a_composed_motion_carries_the_whole_walk() {
        let window = screened(2, 0, 10_000, 1.0, 1.0);
        let motion = window.motion;
        assert_eq!(motion.segments(), 2);
        assert_eq!(motion.segment(0).clip.frames(), 6);
        assert_eq!(motion.segment(1).clip.frames(), 20);
        assert_eq!(motion.duration_s(), 26.0 / FLOOR_TICK_HZ);

        // The tightest of the two clips' ceilings, which the single-clip
        // motions do not have to be.
        let library = library();
        let validated = ValidatedLibrary::of(read(&library)).expect("the fixture plays");
        let tightest = (0..2)
            .map(|clip_id| {
                validated
                    .playable(clip_id)
                    .expect("a fixture clip")
                    .max_speed()
            })
            .fold(f64::INFINITY, f64::min);
        assert_eq!(motion.max_speed(), tightest);
    }

    /// Each refusal is its own, and each is refused outright: no clamping, no
    /// nearest playable window.
    #[test]
    fn a_window_this_build_will_not_play_says_which_way() {
        assert_eq!(
            refusal(3, 0, 10, 1.0, 1.0),
            OverlayError::Unplayable(MotionViewError::UnknownMotion {
                motion_id: 3,
                motions: 3
            })
        );
        for gain in [1.5, -0.1, f64::NAN, f64::INFINITY] {
            let raised = refusal(0, 0, 10, gain, 1.0);
            assert!(
                matches!(raised, OverlayError::Gain { gain: held } if held.to_bits() == gain.to_bits()),
                "a gain of {gain} was refused as {raised}"
            );
        }
        for speed in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let raised = refusal(0, 0, 10, 1.0, speed);
            assert!(
                matches!(raised, OverlayError::Speed { speed: held } if held.to_bits() == speed.to_bits()),
                "a speed of {speed} was refused as {raised}"
            );
        }
        assert_eq!(
            refusal(0, 10, 10, 1.0, 1.0),
            OverlayError::EmptyWindow {
                start_ns: 10,
                end_ns: 10
            }
        );
        assert_eq!(
            refusal(0, 10, 9, 1.0, 1.0),
            OverlayError::EmptyWindow {
                start_ns: 10,
                end_ns: 9
            }
        );
    }

    /// A window asking for more speed than the motion's clips admit is refused
    /// here rather than shredded a layer later.
    #[test]
    fn a_window_faster_than_the_motion_admits_is_refused() {
        let ceiling = ceiling();
        assert_eq!(
            refusal(0, 0, 10, 1.0, ceiling + 0.5),
            OverlayError::PastCeiling {
                motion_id: 0,
                speed: ceiling + 0.5,
                ceiling,
            }
        );
        assert_eq!(
            screened(0, 0, 10, 1.0, ceiling).speed,
            ceiling,
            "the ceiling is a speed the derivation admits"
        );
    }

    /// The window is half-open, so two windows sharing an edge do not both own
    /// the instant on it, and the join offset is measured from the start.
    #[test]
    fn a_window_owns_its_own_start_and_not_its_end() {
        let window = screened(0, 1_000, 3_000, 1.0, 1.0);
        assert!(!window.covers(999));
        assert!(window.covers(1_000));
        assert!(window.covers(2_999));
        assert!(!window.covers(3_000));
        assert_eq!(window.joined_at(1_000), Duration::ZERO);
        assert_eq!(window.joined_at(2_500), Duration::from_nanos(1_500));
        assert_eq!(
            window.joined_at(0),
            Duration::ZERO,
            "a window not yet open is joined at its own beginning"
        );
    }

    /// A schedule's windows land in the rows they arrived in, a refused one
    /// leaves its row empty and is counted, and the rows say when the stream can
    /// stop.
    #[test]
    fn the_screened_windows_keep_their_rows() {
        let message = schedule(&[
            (0, 0, 1_000, 1.0, 1.0),
            (9, 0, 1_000, 1.0, 1.0),
            (1, 2_000, 5_000, 0.5, 2.0),
        ]);
        let library = library();
        let validated = ValidatedLibrary::of(read(&library)).expect("the fixture plays");
        let windows = Windows::of(&message, &validated);
        assert_eq!(windows.refused(), 1);
        assert_eq!(windows.row(0).map(|window| window.motion_id), Some(0));
        assert!(
            windows.row(1).is_none(),
            "a refused window keeps nobody's row"
        );
        assert_eq!(windows.row(2).map(|window| window.motion_id), Some(1));
        assert!(windows.row(3).is_none());
        assert!(windows.any_covers(0));
        assert!(
            !windows.any_covers(1_500),
            "the gap between two windows is nobody's"
        );
        assert!(windows.any_covers(4_999));
        assert_eq!(windows.last_end(), Some(5_000));
        assert_eq!(Windows::default().last_end(), None);
    }

    /// The row count is the schema's, in both files that state one.
    #[test]
    fn the_rows_are_the_schema_s_own() {
        let mut state = brenn_reachy__cogs__mover_clk_rs::MoverStateWire::new();
        assert_eq!(state.players_mut().len(), MAX_OVERLAYS);
        let mut message = SessionScheduleWire::new();
        let mut rows = message.overlays_mut();
        rows.clear();
        for _ in 0..MAX_OVERLAYS {
            assert!(rows.try_grow().is_some(), "a schedule of four windows");
        }
        assert!(
            rows.try_grow().is_none(),
            "the schedule holds no more windows than the layer has rows"
        );
    }

    /// A player a few ticks into a motion has written its own row: the state is
    /// the row, and the two fields beside it are the host's.
    #[test]
    fn a_playing_overlay_is_its_row() {
        let library = library();
        let validated = ValidatedLibrary::of(read(&library)).expect("the fixture plays");
        let motion = validated.playable_motion(0).expect("the fixture motion");
        let mut slot = ClipPlayerSnapWire::new();
        {
            let state = slot.validate_mut().expect("a cleared row validates");
            state.active = true.into();
            state.motion_id = 0;
            let mut player = ClipPlayer::joining_at(motion, 1.5, Duration::from_millis(60), state);
            for _ in 0..3 {
                player.advance(PERIOD);
            }
        }
        let state = slot.validate().expect("a played row validates");
        assert!(bool::from(state.active));
        assert_eq!(state.motion_id, 0);
        assert_eq!(state.speed, 1.5);
        assert_eq!(
            state.track,
            reachy_clips::config::motion_fingerprint(&motion)
        );
        assert!(state.clock_s > 0.0, "three ticks of a 1.5x invocation");
        assert!(bool::from(state.started));
        assert_eq!(state.ramps.head, 60, "the clip's own fade-out");
    }

    /// The load-bearing case: a layer that crosses the slot every period plays
    /// exactly what one that never crossed plays.
    #[test]
    fn a_layer_that_crosses_its_slot_plays_the_same_overlay() {
        let library = library();
        let validated =
            ValidatedLibrary::of(read(&library)).expect("the fixture library is playable");
        let message = schedule(&[(0, 100_000_000, 400_000_000, 0.5, 1.5)]);
        let windows = Windows::of(&message, &validated);
        let mut held = core::array::from_fn::<_, MAX_OVERLAYS, _>(|_| ClipPlayerSnapWire::new());
        let mut crossed_rows =
            core::array::from_fn::<_, MAX_OVERLAYS, _>(|_| ClipPlayerSnapWire::new());

        {
            let (mut whole, refusals) = Overlays::take_up(&mut held, &windows, 100_000_000);
            assert_eq!(refusals, Refusals::default());
            for tick in 0..15 {
                let now = 100_000_000 + tick * 20_000_000;
                let crossed_samples = {
                    let (mut crossed, refusals) =
                        Overlays::take_up(&mut crossed_rows, &windows, now);
                    assert_eq!(refusals, Refusals::default(), "tick {tick}");
                    crossed.sample(PERIOD)
                };
                let whole_samples = whole.sample(PERIOD);
                assert_eq!(
                    crossed_samples.as_slice(),
                    whole_samples.as_slice(),
                    "tick {tick} diverged across the slot"
                );
            }
        }
        assert!(
            !crossed_rows[0].finished(),
            "the fixture clip is longer than the run, so the case saw a playing player"
        );
    }

    /// The gain is the share of the delta that applies: it scales the player's
    /// own weights and touches nothing else.
    #[test]
    fn the_gain_scales_the_weights_and_nothing_else() {
        let library = library();
        let validated = ValidatedLibrary::of(read(&library)).expect("playable");
        let mut full_rows =
            core::array::from_fn::<_, MAX_OVERLAYS, _>(|_| ClipPlayerSnapWire::new());
        let mut half_rows =
            core::array::from_fn::<_, MAX_OVERLAYS, _>(|_| ClipPlayerSnapWire::new());
        let full = Windows::of(&schedule(&[(0, 0, 400_000_000, 1.0, 1.0)]), &validated);
        let half = Windows::of(&schedule(&[(0, 0, 400_000_000, 0.5, 1.0)]), &validated);
        let (mut at_full, _) = Overlays::take_up(&mut full_rows, &full, 0);
        let (mut at_half, _) = Overlays::take_up(&mut half_rows, &half, 0);
        for tick in 0..8 {
            let (full_sample, half_sample) = (at_full.sample(PERIOD), at_half.sample(PERIOD));
            let (full_sample, half_sample) = (full_sample.as_slice()[0], half_sample.as_slice()[0]);
            assert_eq!(full_sample.frame, half_sample.frame, "tick {tick}");
            for channel in Channel::ALL {
                assert!(
                    (half_sample.weights.get(channel) - full_sample.weights.get(channel) / 2.0)
                        .abs()
                        < 1e-12,
                    "tick {tick} channel {channel} did not halve"
                );
            }
        }
    }

    /// A window that has closed, and one that never opened, play nothing — and
    /// the row is left holding no player, so nothing resumes when a later window
    /// takes the row.
    #[test]
    fn a_closed_window_leaves_no_player_behind() {
        let library = library();
        let validated = ValidatedLibrary::of(read(&library)).expect("playable");
        let windows = Windows::of(&schedule(&[(0, 0, 100_000_000, 1.0, 1.0)]), &validated);
        let mut rows = core::array::from_fn::<_, MAX_OVERLAYS, _>(|_| ClipPlayerSnapWire::new());
        {
            let (mut open, _) = Overlays::take_up(&mut rows, &windows, 0);
            assert!(open.any());
            assert!(!open.sample(PERIOD).is_empty());
        }
        assert!(rows[0].active(), "an open window keeps its player");

        {
            let (mut closed, refusals) = Overlays::take_up(&mut rows, &windows, 100_000_000);
            assert!(!closed.any(), "a closed window plays nothing");
            assert!(closed.sample(PERIOD).is_empty());
            assert_eq!(refusals, Refusals::default());
        }
        assert!(!rows[0].active(), "a closed window leaves no player");
    }

    /// A row holding another motion's player is joined afresh rather than
    /// restored over the wrong track, and the reuse is not a refusal: a window
    /// opening in a used row is the ordinary case.
    #[test]
    fn a_row_reused_for_another_motion_starts_that_motion() {
        let library = library();
        let validated = ValidatedLibrary::of(read(&library)).expect("playable");
        let mut rows = core::array::from_fn::<_, MAX_OVERLAYS, _>(|_| ClipPlayerSnapWire::new());
        let first = Windows::of(&schedule(&[(0, 0, 100_000_000, 1.0, 1.0)]), &validated);
        {
            let (mut playing, _) = Overlays::take_up(&mut rows, &first, 0);
            for _ in 0..4 {
                playing.sample(PERIOD);
            }
        }

        let second = Windows::of(
            &schedule(&[(1, 200_000_000, 300_000_000, 1.0, 1.0)]),
            &validated,
        );
        {
            let (mut fresh, refusals) = Overlays::take_up(&mut rows, &second, 200_000_000);
            assert_eq!(refusals, Refusals::default());
            fresh.sample(PERIOD);
        }
        assert_eq!(rows[0].motion_id(), 1);
        assert_eq!(
            rows[0].clock_s(),
            0.0,
            "the new motion starts at its own beginning"
        );
    }

    /// A row that will not read back as a player is counted and the window plays
    /// on from a fresh join — the same thing a window opening does, which is what
    /// a slot nobody can read leaves the run needing.
    #[test]
    fn a_row_that_is_no_player_is_counted_and_rejoined() {
        let library = library();
        let validated = ValidatedLibrary::of(read(&library)).expect("playable");
        let mut rows = core::array::from_fn::<_, MAX_OVERLAYS, _>(|_| ClipPlayerSnapWire::new());
        let windows = Windows::of(&schedule(&[(0, 0, 400_000_000, 1.0, 1.0)]), &validated);
        {
            let (mut playing, _) = Overlays::take_up(&mut rows, &windows, 0);
            for _ in 0..4 {
                playing.sample(PERIOD);
            }
        }
        // A frozen head delta the pick-up refuses: the flag says a rotation is
        // there and the numbers are not one.
        rows[0].frozen_mut().set_head_present(true);
        rows[0].frozen_mut().head_quat_mut().set_w(0.0);

        {
            let (rejoined, refusals) = Overlays::take_up(&mut rows, &windows, 100_000_000);
            assert_eq!(refusals, Refusals { players: 1 });
            assert!(rejoined.any(), "the window plays on");
        }
        assert!(rows[0].active());
        assert_eq!(
            rows[0].clock_s(),
            0.1,
            "the rejoin is at the offset the window says, not at the clip's start"
        );
        assert_eq!(
            rows[0].weights().head(),
            0.0,
            "a rejoin eases onto the delta at that offset rather than stepping onto it"
        );
    }

    /// A player whose clip and fade-out are both over contributes nothing and is
    /// left in place: its row says it is finished, so nothing restarts it.
    #[test]
    fn a_finished_overlay_contributes_nothing() {
        let library = library();
        let validated = ValidatedLibrary::of(read(&library)).expect("playable");
        let mut rows = core::array::from_fn::<_, MAX_OVERLAYS, _>(|_| ClipPlayerSnapWire::new());
        // Clip 1 is six frames at 50 Hz: 120 ms, plus a 60 ms fade-out.
        let windows = Windows::of(&schedule(&[(1, 0, 10_000_000_000, 1.0, 1.0)]), &validated);
        let mut last_contribution = 0;
        {
            let mut layer = Overlays::take_up(&mut rows, &windows, 0).0;
            for tick in 0..40 {
                if !layer.sample(PERIOD).is_empty() {
                    last_contribution = tick;
                }
            }
            assert!(layer.any(), "the row is still the window's");
        }
        assert!(rows[0].finished(), "the clip ran out");
        assert!(
            last_contribution < 30,
            "a finished clip kept contributing until tick {last_contribution}"
        );
    }

    /// A held base commands the same configuration every period; a base on a
    /// move walks the tick's own path and holds where it ends.
    #[test]
    fn the_base_walks_the_path_it_was_handed() {
        let start = stow_pose_targets();
        let target = neutral_targets();
        let durations = MoveDurations::uniform(Duration::from_millis(100));
        let trajectory = Trajectory::new(&start, &target, durations, WarpKind::MinJerk)
            .expect("the fixture shapes");

        let mut held = Base::held(start);
        assert_eq!(held.step(PERIOD), start);
        assert_eq!(held.step(PERIOD), start);
        assert_eq!(held.path, None);

        let mut moving = Base::held(start);
        moving.retarget(&trajectory);
        for tick in 0u8..5 {
            let mut wanted = start;
            trajectory.sample(PERIOD * u32::from(tick), &mut wanted);
            assert_eq!(moving.step(PERIOD), wanted, "tick {tick}");
        }
        assert!(moving.path.is_some(), "the move has one sample left");
        assert_eq!(moving.step(PERIOD), target);
        assert_eq!(
            moving.path, None,
            "a path that has run out is dropped, and the base holds where it ended"
        );
        assert_eq!(moving.step(PERIOD), target);
    }

    /// The base crosses its slot: held, on a move, and not this cog's at all.
    #[test]
    fn the_base_crosses_its_slot() {
        let mut slot = BaseSnapWire::new();
        assert_eq!(
            read_base(&slot),
            Ok(None),
            "an unwritten slot is the tick's"
        );

        let held = Base::held(stow_pose_targets());
        write_base(&mut slot, Some(&held));
        assert_eq!(read_base(&slot), Ok(Some(held)));

        let mut moving = Base::held(stow_pose_targets());
        moving.retarget(
            &Trajectory::new(
                &stow_pose_targets(),
                &neutral_targets(),
                MoveDurations::uniform(Duration::from_millis(100)),
                WarpKind::MinJerk,
            )
            .expect("the fixture shapes"),
        );
        moving.step(PERIOD);
        write_base(&mut slot, Some(&moving));
        assert_eq!(read_base(&slot), Ok(Some(moving)));

        // A held base over a moving one: the ordering where residue would
        // show, because only this one leaves a path in the slot for the write
        // to clear. Read back with a path still on it, the compositor would
        // keep sampling a trajectory that ended.
        let held_again = Base::held(neutral_targets());
        write_base(&mut slot, Some(&held_again));
        let read = read_base(&slot).expect("a written base reads");
        assert_eq!(read, Some(held_again));
        assert_eq!(
            read.expect("a base was written").path,
            None,
            "the move of the base that was here is still in the slot"
        );

        write_base(&mut slot, None);
        assert_eq!(read_base(&slot), Ok(None));
    }

    /// A slot whose move is no path is refused rather than sampled: a base is
    /// rebuilt from the bytes, and bytes that shape no move describe no base.
    #[test]
    fn a_base_path_that_is_no_path_is_refused() {
        let mut slot = BaseSnapWire::new();
        let mut moving = Base::held(stow_pose_targets());
        moving.retarget(
            &Trajectory::new(
                &stow_pose_targets(),
                &neutral_targets(),
                MoveDurations::uniform(Duration::from_millis(100)),
                WarpKind::MinJerk,
            )
            .expect("the fixture shapes"),
        );
        write_base(&mut slot, Some(&moving));
        // A move of no length: what the slot says is a path, and is not one.
        slot.path_mut().set_dur_head(SlotDuration::from_nanos(0));
        assert_eq!(
            read_base(&slot),
            Err(OverlayError::Base(SeedError::Path(
                TrajectoryError::NonPositiveDuration
            )))
        );
    }

    /// A base whose move is bytes no move reads from is refused before anything
    /// is rebuilt from them. The hand-off between the compositor and the tick
    /// is a slot a peer built against another schema can write, and an
    /// undeclared discriminant in it is the shape that takes.
    #[test]
    fn a_base_whose_move_is_no_message_is_refused() {
        use brenn_reachy__motion__tick_state_clk_rs::WarpKindWire;

        let mut slot = BaseSnapWire::new();
        let mut moving = Base::held(stow_pose_targets());
        moving.retarget(
            &Trajectory::new(
                &stow_pose_targets(),
                &neutral_targets(),
                MoveDurations::uniform(Duration::from_millis(100)),
                WarpKind::MinJerk,
            )
            .expect("the fixture shapes"),
        );
        write_base(&mut slot, Some(&moving));
        // A shape the vocabulary does not declare, in the one enum field the
        // seed carries.
        slot.path_mut().set_warp(WarpKindWire(7));
        assert!(
            matches!(read_base(&slot), Err(OverlayError::Unreadable(_))),
            "{:?}",
            read_base(&slot)
        );
    }

    /// A base whose clock runs backwards is refused: how far into the move is
    /// an elapsed time, and one below zero was written by something that was
    /// not measuring one.
    #[test]
    fn a_base_clock_that_runs_backwards_is_refused() {
        let mut slot = BaseSnapWire::new();
        write_base(&mut slot, Some(&Base::held(stow_pose_targets())));
        slot.set_elapsed(SlotDuration::from_nanos(-1));
        assert_eq!(
            read_base(&slot),
            Err(OverlayError::Elapsed(DurationError::Negative(-1)))
        );
    }
}
