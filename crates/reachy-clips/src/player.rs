//! The player: one overlay's clock, its ramps, and the delta it contributes
//! this tick.
//!
//! A [`ClipPlayer`] plays one configured motion: its segments, the speed it was
//! invoked at, and where on the motion's clock it currently is. Each tick the
//! caller advances it by the elapsed period and gets back the deltas to hand
//! the compositor, or the terminal marker that says this overlay is finished
//! and can be dropped.
//!
//! Three rules do most of the work:
//!
//! - **Lateness cannot be made up.** The motion clock advances by at most one
//!   nominal period, scaled by speed, however late the caller wakes. A loop
//!   that stalled for a second resumes the motion where it left off; it does
//!   not fast-forward fifty frames into one commanded step. This is the same
//!   rule that keeps a planned move's step bound meaningful.
//! - **Speed scales the motion clock, never the ramps.** An invocation at 2×
//!   plays the clips and the holds between them in half the time, because that
//!   is the same motion faster. The blend ramps run on the wall clock instead,
//!   because their lengths are what keep a weight ramp inside the per-tick step
//!   bound, and a bound in real time cannot be scaled away by asking for speed.
//! - **Weights are per channel.** A motion's segments need not drive the same
//!   channels; when the mask changes between segments, the channel that left
//!   fades out on the outgoing clip's ramp while the rest carry on. A channel
//!   that is fading holds the last delta it was given, so it returns to the
//!   base gradually instead of vanishing.
//!
//! Sans-I/O like the rest of the crate: the elapsed period arrives as a
//! parameter and nothing here reads a clock.
//!
//! What a player plays is a [`MotionView`] — the flattened segments the host is
//! handed, borrowed, with their clips — and its whole state lives in the slot
//! that host hands it ([`ClipPlayerSnap`]), read and written in place. Neither
//! has a second form: there is one storage of a motion and one of a player, so
//! a host whose memory between ticks is a fixed-layout slot finds the player
//! exactly where the last execution left it.
//!
//! Where the player stands is derived from that clock every tick, never
//! recorded: a motion's timeline is a pure function of its lead gap and its
//! segment table, so a stored cursor would be a second answer that has to keep
//! agreeing with the clock bit for bit.

use std::time::Duration;

use brenn_reachy__clips__player_clk_rs::{ClipDelta, ClipPlayerSnap, ClipRamps, ClipWeights};
use reachy_motion::FLOOR_TICK_HZ;
use reachy_motion::record::{clear_pose, read_pose, write_pose};
use reachy_motion::snap::PoseSnapshotError;
use thiserror::Error;

use crate::compose::{ChannelWeights, OverlaySample, interpolate_pose, lerp};
use crate::config::{ClipView, MotionView, SegmentView, motion_fingerprint};
use crate::format::{Channel, ChannelMask, DeltaFrame, PerChannel};

/// The nominal control period, seconds: the most the motion clock may advance
/// in one call, before speed.
const NOMINAL_PERIOD_S: f64 = 1.0 / FLOOR_TICK_HZ;

/// How far short of a boundary the motion clock counts as being on it,
/// seconds.
///
/// The clock is a running sum of periods and the grid it meets — frame edges,
/// segment ends, the motion's end — is a multiple of the same period, so the
/// two agree in exact arithmetic and land a rounding error apart in binary. A
/// nanosecond of tolerance puts the clock back on the grid; without it a clip
/// runs an extra tick whenever the accumulated error happens to fall short.
const CLOCK_EPS_S: f64 = 1e-9;

/// How close to its target a blend weight has to get to be there.
///
/// Far below any weight difference the machine could feel, and far above the
/// rounding a few dozen accumulated ramp steps leave behind.
const RAMP_EPS: f64 = 1e-9;

/// Where the motion clock currently sits, carrying whatever is driving there.
///
/// The segment rather than its index: the walk that finds the position has the
/// segment in hand, and everything downstream of the walk wants the clip it
/// names. Carried, the answer says what is playing; as an index, four callers
/// go and look it up again.
#[derive(Clone, Copy, Debug)]
enum Position<'c> {
    /// Before the first clip, in the motion's leading hold. Nothing is driven
    /// yet: a leading gap holds the base alone, since there is no previous
    /// segment whose delta could be frozen.
    Lead,
    /// Inside a segment's clip, `local_s` seconds into it in the clip's own
    /// time.
    Playing {
        /// Which segment.
        segment: SegmentView<'c>,
        /// Elapsed within the clip, clip-native seconds.
        local_s: f64,
    },
    /// Inside the hold that follows a segment's clip, freezing its last delta.
    Holding {
        /// Which segment's hold.
        segment: SegmentView<'c>,
    },
    /// Past the final hold: the motion is over and its channels are fading.
    Ended,
}

impl Position<'_> {
    /// The channels being driven right now.
    ///
    /// A hold drives exactly what the clip it follows drove — that is what
    /// makes it a hold of the delta rather than a return to the base. A leading
    /// gap and the end of the motion drive nothing, which is what starts every
    /// channel fading.
    fn live_mask(self) -> ChannelMask {
        match self {
            Self::Lead | Self::Ended => ChannelMask::empty(),
            Self::Playing { segment, .. } | Self::Holding { segment } => segment.clip.mask(),
        }
    }

    /// Whether the motion is over.
    fn is_ended(self) -> bool {
        matches!(self, Self::Ended)
    }
}

/// A slot whose numbers name a state no sequence of calls reaches.
///
/// Picking one up would invent motion rather than resume it, so each is
/// refused. The case that needs refusing is a slot holding bytes nothing
/// wrote.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum PlayerStateError {
    /// The invocation speed is not finite and positive. A player never holds
    /// anything else — its constructors refuse one — and a clock multiplied by
    /// this would stop advancing or become non-finite.
    #[error("a player's speed must be finite and positive, not {speed}")]
    BadSpeed {
        /// What the slot held.
        speed: f64,
    },
    /// An invocation speed above what the motion's clips admit together.
    /// Played, it steps between frames further than the derivation admitted.
    #[error("a player's speed of {speed} is past the motion's ceiling of {ceiling}")]
    PastCeiling {
        /// What the slot held.
        speed: f64,
        /// The fastest invocation the motion's segments admit together.
        ceiling: f64,
    },
    /// The motion clock is not finite and non-negative. It is a sum of
    /// non-negative periods from zero.
    #[error("a player's motion clock must be finite and non-negative, not {value}")]
    BadClock {
        /// What the slot held.
        value: f64,
    },
    /// A blend weight outside `[0, 1]`. The ramps are clamped to that range and
    /// the blend-floor derivation assumes it.
    #[error("the {channel} weight of {weight} is outside [0, 1]")]
    BadWeight {
        /// Which channel.
        channel: Channel,
        /// What the slot held.
        weight: f64,
    },
    /// A frozen delta carrying a number that is not finite. It would reach the
    /// composed target, which the tick then has to refuse.
    #[error("the frozen delta carries a number that is not finite")]
    NonFiniteDelta,
    /// A frozen delta whose head rotation is not a rotation. A clip's frames are
    /// unit by construction and interpolation keeps them there, so this is bytes
    /// nothing wrote; picked up, it scales the composed target and the refusal
    /// lands a layer later as an unexplained pose.
    ///
    /// Wrapped rather than restated: a pose is a pose wherever it is held, and
    /// the check belongs to whoever owns the two fields.
    #[error("the frozen head delta is no delta: {0}")]
    NotADelta(#[from] PoseSnapshotError),
    /// A state left by a player of a different motion. The clock, the frozen
    /// delta and the fade ramps all mean something only against the walk they
    /// were taken from; replayed over another one they name the wrong frames and
    /// emit plausible deltas nobody asked for.
    #[error("the slot's track fingerprint {held:#x} is not this motion's {restored_over:#x}")]
    DifferentTrack {
        /// What the slot recorded.
        held: u64,
        /// What the motion it is being restored over answers. Not spelled
        /// `source`, which `thiserror` reads as an error's cause.
        restored_over: u64,
    },
    /// A player finished before its motion ended, or before it took a tick, or
    /// still carrying weight. A player finishes in one place — the tick that
    /// finds the motion over and every channel faded — and each of these
    /// contradicts it.
    #[error("the slot is finished but {why}")]
    ImpossibleFinish {
        /// Which part of the state contradicts the finish.
        why: &'static str,
    },
    /// A player that has taken no tick carrying weight on a channel, which the
    /// first call writes — the same call that marks the player started.
    #[error("the slot has taken no tick but {why}")]
    ImpossibleStart {
        /// Which part of the state contradicts it.
        why: &'static str,
    },
    /// A fade-out ramp that is neither zero nor the exit ramp of a clip that
    /// drives this channel. A player records one of those: zero until a tick
    /// drives the channel, then the ramp of whichever clip last drove it. A
    /// ramp of zero on a channel a clip with an exit ramp last drove drops the
    /// whole weighted delta in one period, and an arbitrary large one is a
    /// channel that never finishes fading — both past the step bound the ramp
    /// was floored against. A ramp belonging to a clip that never drives the
    /// channel is the same failure by a narrower door: the fade runs on a bound
    /// derived from another clip's frames.
    #[error("the {channel} fade-out ramp of {ms} ms is no clip of this motion's driving it")]
    BadRamp {
        /// Which channel.
        channel: Channel,
        /// What the slot held.
        ms: u32,
    },
    /// A frozen delta driving a different set of channels than the motion. The
    /// frame starts as the zero delta of the motion's union mask and is
    /// refreshed on the channels a segment drives, so any other set is bytes
    /// nothing wrote; picked up mid-fade, a channel gone missing takes its whole
    /// contribution out of the composed target in one period.
    #[error("the frozen delta drives {held:?}, not the motion's {motion:?}")]
    FrozenChannels {
        /// What the slot held.
        held: ChannelMask,
        /// What the motion drives.
        motion: ChannelMask,
    },
}

/// One playing overlay.
///
/// Constructed at the moment the overlay starts, or — for a daemon joining a
/// script already in progress — at the offset the timeline says it should be
/// at. Either way its weights start at zero and ramp in, so a mid-motion join
/// eases onto the delta at that offset rather than stepping onto it.
///
/// Everything the player carries between ticks is the borrowed slot, read and
/// written in place: a host whose per-execution memory is a fixed-layout slot
/// cannot keep a player in a local variable across the round trips of a script,
/// so it finds each overlay exactly where the last execution left it. There is
/// no second form and nothing to round-trip.
///
/// The motion is not in the slot. What is being played is configuration the
/// host is handed read-only every execution, not something the playback found
/// out; a copy in the state would be a second, divergent library. So both doors
/// take the motion alongside the state, and [`ClipPlayer::resumable`] checks the
/// state against the motion it is picked up over.
#[derive(Debug)]
pub struct ClipPlayer<'a, 'c> {
    motion: MotionView<'c>,
    state: &'a mut ClipPlayerSnap,
}

impl<'a, 'c> ClipPlayer<'a, 'c> {
    /// Start `motion` at its beginning in `state`, invoked at `speed`.
    ///
    /// # Panics
    ///
    /// If `speed` is not finite and positive; see [`ClipPlayer::joining_at`].
    #[must_use]
    pub fn new(motion: MotionView<'c>, speed: f64, state: &'a mut ClipPlayerSnap) -> Self {
        Self::joining_at(motion, speed, Duration::ZERO, state)
    }

    /// Start `motion` in `state` as though it had already been playing for
    /// `elapsed` of wall clock.
    ///
    /// What a daemon uses when a script it just accepted says an overlay
    /// started before now: the timeline stays authoritative in absolute time,
    /// so the overlay is picked up where it should be rather than replayed from
    /// the top. The weights still start at zero, because the delta at the join
    /// point can be the motion's largest excursion and stepping onto it is
    /// exactly what the ramps exist to prevent.
    ///
    /// Every field a player owns is written, so a row reused for a new overlay
    /// carries nothing of the one before it. The two the *host* owns — whether
    /// the row is occupied and which motion the window named — are left exactly
    /// as they stand.
    ///
    /// # Panics
    ///
    /// If `speed` is not finite and positive — a defect in the calling path,
    /// not user input. Left to run, it is a player whose clock never advances:
    /// an overlay that holds one frame forever, occupying a slot until the
    /// script expires, reporting neither an error nor an end.
    #[must_use]
    pub fn joining_at(
        motion: MotionView<'c>,
        speed: f64,
        elapsed: Duration,
        state: &'a mut ClipPlayerSnap,
    ) -> Self {
        assert!(
            speed.is_finite() && speed > 0.0,
            "a player's speed must be finite and positive, not {speed}"
        );
        state.track = motion_fingerprint(&motion);
        state.speed = speed;
        state.clock_s = elapsed.as_secs_f64() * speed;
        write_frozen(&mut state.frozen, &DeltaFrame::zero(motion.mask()));
        write_weights(&mut state.weights, ChannelWeights::zero());
        write_ramps(&mut state.ramps, &PerChannel::new([0; Channel::COUNT]));
        state.started = false.into();
        state.finished = false.into();
        Self { motion, state }
    }

    /// The player `state` was left by, playing `motion`.
    ///
    /// Its precondition is [`ClipPlayer::resumable`], which every caller asks
    /// first: a host holds one mutable borrow of the row and cannot hand it out
    /// twice, so the question and the pick-up are separate calls. A debug build
    /// asserts the precondition, so a host that skips or reorders the question
    /// fails a test rather than shipping the misuse. A release build is total:
    /// picked up over a state `resumable` refuses, the player plays that state
    /// — the wrong offset in the right motion, or a clock that never advances —
    /// which is exactly what the question exists to prevent, and a frozen delta
    /// that will not read degrades to the zero frame rather than panicking.
    #[must_use]
    pub fn over(motion: MotionView<'c>, state: &'a mut ClipPlayerSnap) -> Self {
        debug_assert!(
            Self::resumable(&motion, state).is_ok(),
            "a player picked up over a state its own question refuses"
        );
        Self { motion, state }
    }

    /// Whether `state` is a player of `motion` that some sequence of calls
    /// reaches.
    ///
    /// `motion` is what the host is configured with, and the state is what the
    /// playback found out. The state names the motion it was left by and its
    /// numbers are checked against the motion it is picked up over, so a state
    /// from a different walk is refused rather than replayed against the wrong
    /// frames.
    ///
    /// Where the clock stands needs no check of its own: every value from zero
    /// up is a position in the motion or past its end, and the walk that derives
    /// the segment from it is total. The speed does need one beyond finiteness,
    /// since a row keeps the speed it was joined at and nothing else asks the
    /// motion's ceiling of a stored one.
    ///
    /// # Errors
    ///
    /// [`PlayerStateError`] for a state no sequence of calls reaches.
    pub fn resumable(
        motion: &MotionView<'c>,
        state: &ClipPlayerSnap,
    ) -> Result<(), PlayerStateError> {
        let fingerprint = motion_fingerprint(motion);
        if state.track != fingerprint {
            return Err(PlayerStateError::DifferentTrack {
                held: state.track,
                restored_over: fingerprint,
            });
        }
        let speed = state.speed;
        if !speed.is_finite() || speed <= 0.0 {
            return Err(PlayerStateError::BadSpeed { speed });
        }
        // A pick-up keeps the speed it was joined at; this is the only place
        // a stored one is judged against the motion's ceiling.
        let ceiling = motion.max_speed();
        if speed > ceiling {
            return Err(PlayerStateError::PastCeiling { speed, ceiling });
        }
        let clock_s = state.clock_s;
        if !clock_s.is_finite() || clock_s < 0.0 {
            return Err(PlayerStateError::BadClock { value: clock_s });
        }
        let weights = weights_of(&state.weights);
        for channel in Channel::ALL {
            let weight = weights.get(channel);
            if !weight.is_finite() || !(0.0..=1.0).contains(&weight) {
                return Err(PlayerStateError::BadWeight { channel, weight });
            }
        }
        if let Some((channel, ms)) = stray_ramp(motion, &state.ramps) {
            return Err(PlayerStateError::BadRamp { channel, ms });
        }
        let frozen = read_frozen(&state.frozen)?;
        if frozen.mask() != motion.mask() {
            return Err(PlayerStateError::FrozenChannels {
                held: frozen.mask(),
                motion: motion.mask(),
            });
        }
        if !frozen.is_finite() {
            return Err(PlayerStateError::NonFiniteDelta);
        }
        let started = bool::from(state.started);
        if bool::from(state.finished) {
            // A finish is one tick's conclusion: the motion over, and nothing
            // left fading.
            let why = if !started {
                Some("has taken no tick")
            } else if clock_s + CLOCK_EPS_S < motion.duration_s() {
                Some("has motion left to play")
            } else if weights != ChannelWeights::zero() {
                Some("still carries weight")
            } else {
                None
            };
            if let Some(why) = why {
                return Err(PlayerStateError::ImpossibleFinish { why });
            }
        }
        if !started && weights != ChannelWeights::zero() {
            return Err(PlayerStateError::ImpossibleStart {
                why: "carries weight",
            });
        }

        Ok(())
    }

    /// The motion being played.
    #[must_use]
    pub fn motion(&self) -> &MotionView<'c> {
        &self.motion
    }

    /// The invocation speed.
    #[must_use]
    pub fn speed(&self) -> f64 {
        self.state.speed
    }

    /// Every channel this motion drives over its whole run.
    #[must_use]
    pub fn mask(&self) -> ChannelMask {
        self.motion.mask()
    }

    /// Whether the motion and its fade-out are both over.
    ///
    /// A finished player contributes nothing and is dropped by its owner.
    #[must_use]
    pub fn is_finished(&self) -> bool {
        bool::from(self.state.finished)
    }

    /// Every channel's blend weight, as the slot holds them.
    fn weights(&self) -> ChannelWeights {
        weights_of(&self.state.weights)
    }

    /// The delta each channel was last given, as the slot holds it.
    ///
    /// Total: a delta that is no delta reads as the zero frame, which is the
    /// motion contributing nothing while its channels fade. `joining_at` writes
    /// a frame whose head is a rotation and [`ClipPlayer::resumable`] refuses
    /// one that is not, so the fallback is reached only by a host that picked
    /// the row up without asking — and the process that composes head targets
    /// refuses a state it cannot read rather than aborting over it.
    fn frozen(&self) -> DeltaFrame {
        read_frozen(&self.state.frozen).unwrap_or_else(|_| DeltaFrame::zero(self.motion.mask()))
    }

    /// Advance by `elapsed` and return this tick's contribution, or `None` once
    /// the motion has ended and every channel has faded to zero.
    ///
    /// `elapsed` is the period since the previous call, capped at one nominal
    /// period before anything uses it: a late call advances the motion and the
    /// ramps by one period, not by the lateness. The first call takes no
    /// elapsed time at all — it reports the overlay at its start offset, with
    /// every weight still at zero — so the first frame it plays is commanded
    /// rather than skipped past.
    pub fn advance(&mut self, elapsed: Duration) -> Option<OverlaySample> {
        if self.is_finished() {
            return None;
        }
        let dt_s = if bool::from(self.state.started) {
            elapsed.as_secs_f64().min(NOMINAL_PERIOD_S)
        } else {
            self.state.started = true.into();
            0.0
        };
        self.state.clock_s += dt_s * self.state.speed;

        let position = self.position();
        let live = position.live_mask();
        // Read out of the slot once and written back once: the slot is the
        // storage between executions, and a tick that read a delta or a weight
        // through it per use would pay a quaternion read and three field reads
        // for each of them.
        let mut frozen = self.frozen();
        let mut weights = self.weights();
        self.sample_live(position, &mut frozen);
        self.ramp(dt_s, position, &mut weights);
        write_frozen(&mut self.state.frozen, &frozen);
        write_weights(&mut self.state.weights, weights);

        if position.is_ended() && weights == ChannelWeights::zero() {
            self.state.finished = true.into();
            return None;
        }

        let mut frame = DeltaFrame {
            head: None,
            antennas: None,
            body_yaw: None,
        };
        // A channel appears in the frame while it is driven or still fading; a
        // channel at zero weight that is not driven is absent, which is what
        // makes an antennas-only motion say nothing at all about the head.
        for channel in Channel::ALL {
            if !live.contains(channel) && weights.get(channel) <= 0.0 {
                continue;
            }
            match channel {
                Channel::Head => frame.head = frozen.head,
                Channel::BodyYaw => frame.body_yaw = frozen.body_yaw,
                Channel::Antennas => frame.antennas = frozen.antennas,
            }
        }
        Some(OverlaySample { frame, weights })
    }

    /// Where the clock sits on the motion.
    ///
    /// Walked from the lead gap rather than remembered: the segment table and
    /// the clock are the whole timeline, so the walk answers the same thing a
    /// cursor would and cannot fall out of step with the number it was derived
    /// from. It costs one pass over a handful of segments, and it is monotone in
    /// the clock, so a stalled loop that advances one period lands where that
    /// period puts it rather than skipping a segment.
    fn position(&self) -> Position<'c> {
        let clock_s = self.state.clock_s;
        let mut start_s = self.motion.lead_gap_s();
        if clock_s + CLOCK_EPS_S < start_s {
            return Position::Lead;
        }
        for index in 0..self.motion.segments() {
            let segment = self.motion.segment(index);
            let local_s = clock_s - start_s;
            if local_s + CLOCK_EPS_S < segment.play_span_s() {
                return Position::Playing {
                    segment,
                    // The clip's own time: the motion clock runs in the
                    // motion's, and the flattening's speed is what stands
                    // between the two.
                    local_s: local_s * segment.speed,
                };
            }
            if local_s + CLOCK_EPS_S < segment.span_s() {
                return Position::Holding { segment };
            }
            start_s += segment.span_s();
        }
        Position::Ended
    }

    /// Refresh `frozen` for whatever is live, and record which ramp those
    /// channels will fade out on.
    ///
    /// `frozen` is the caller's copy of what the slot holds, written back by the
    /// tick that read it out. A held channel keeps the delta it already has,
    /// which is the freeze — and that freeze is reconstructed rather than
    /// remembered: a player whose clock lands inside a hold takes that clip's
    /// final frame directly, whether it played the frames before it or joined
    /// there. Otherwise a joiner would hold the zero delta, commanding the bare
    /// base where a player that started from the top is holding the clip's last
    /// pose. Past the end of the motion nothing is refreshed at all: every
    /// channel fades out of the last delta it was given.
    fn sample_live(&mut self, position: Position<'c>, frozen: &mut DeltaFrame) {
        let (sampled, blend_out_ms) = match position {
            Position::Playing { segment, local_s } => {
                let clip = segment.clip;
                (sample_clip(&clip, local_s), clip.blend_out_ms())
            }
            Position::Holding { segment } => {
                let clip = segment.clip;
                (clip.frame(clip.frames() - 1), clip.blend_out_ms())
            }
            Position::Lead | Position::Ended => return,
        };
        for channel in position.live_mask().iter() {
            match channel {
                Channel::Head => frozen.head = sampled.head,
                Channel::BodyYaw => frozen.body_yaw = sampled.body_yaw,
                Channel::Antennas => frozen.antennas = sampled.antennas,
            }
            set_ramp(&mut self.state.ramps, channel, blend_out_ms);
        }
    }

    /// Move each channel's weight in `weights` toward where this tick wants it.
    ///
    /// `weights` is the caller's copy of what the slot holds, written back by
    /// the tick that read it out. Ramps run on the wall clock: `dt_s` is the
    /// real period, not the scaled one. A channel rising uses the entry ramp of
    /// the clip that is bringing it in; a channel falling uses the ramp recorded
    /// when it was last driven, which is the clip whose delta it is fading out
    /// of.
    fn ramp(&mut self, dt_s: f64, position: Position<'c>, weights: &mut ChannelWeights) {
        let live = position.live_mask();
        for channel in Channel::ALL {
            let current = weights.get(channel);
            let target = if live.contains(channel) { 1.0 } else { 0.0 };
            if current == target {
                continue;
            }
            // The ramps are the clips' own, already floored at load against
            // each one's largest frame delta, so a ramp reaching here is one the
            // step bounds admit over a static base.
            let ms = if target > current {
                self.entering_ramp_ms(position)
            } else {
                ramp_of(&self.state.ramps, channel)
            };
            let step = if ms == 0 {
                1.0
            } else {
                dt_s * 1000.0 / f64::from(ms)
            };
            let next = if target > current {
                (current + step).min(1.0)
            } else {
                (current - step).max(0.0)
            };
            // A ramp's step is a period divided by a duration, so the last one
            // lands a rounding error short of its target rather than on it.
            // Left alone that error is a channel that never quite finishes
            // fading and a player that never quite terminates.
            let next = if (next - target).abs() < RAMP_EPS {
                target
            } else {
                next
            };
            weights.set(channel, next);
        }
    }

    /// The entry ramp a channel rising right now runs on, milliseconds.
    ///
    /// The clip the clock stands in, which is the one whose delta the channel
    /// is rising onto — the motion's first at a leading gap, its last once the
    /// motion is over, neither of which drives anything anyway.
    fn entering_ramp_ms(&self, position: Position<'c>) -> u32 {
        let segment = match position {
            Position::Playing { segment, .. } | Position::Holding { segment } => segment,
            Position::Lead => self.motion.segment(0),
            Position::Ended => self.motion.segment(self.motion.segments() - 1),
        };
        segment.clip.blend_in_ms()
    }
}

/// Which of the three recorded fade-out ramps no clip of `motion`'s drives that
/// channel with, if any.
///
/// The ramp a channel is fading on is whichever clip last drove it, so any of
/// the motion's clips that drive the channel will do — and only those: a ramp
/// is written when a channel is sampled, so a clip the channel's mask leaves
/// out never leaves its exit ramp there. A value none of the driving clips
/// carries is a slot nothing wrote. Zero is every channel's until a tick drives
/// it, and asks nothing of the motion.
///
/// One walk of the segments for all three channels rather than one each: the
/// question is about at most a handful of distinct `u32`s, and it is asked
/// every time a row is picked up, which is per row per control period.
fn stray_ramp(motion: &MotionView<'_>, ramps: &ClipRamps) -> Option<(Channel, u32)> {
    let mut held = PerChannel::new([0u32; Channel::COUNT]);
    for channel in Channel::ALL {
        *held.get_mut(channel) = ramp_of(ramps, channel);
    }
    for index in 0..motion.segments() {
        let clip = motion.segment(index).clip;
        let ms = clip.blend_out_ms();
        for channel in Channel::ALL {
            let held = held.get_mut(channel);
            if clip.mask().contains(channel) && *held == ms {
                // Answered for: nothing left to ask of the rest of the walk.
                *held = 0;
            }
        }
    }
    Channel::ALL
        .into_iter()
        .map(|channel| (channel, *held.get(channel)))
        .find(|(_, ms)| *ms != 0)
}

/// The delta `clip` carries `local_s` seconds in.
///
/// Frame index is arithmetic, never a search: the track is uniformly
/// sampled at the tick rate, so the index is the clock times that rate and
/// the remainder is the interpolation parameter. Past the last frame — the
/// final period of the clip, and any overshoot a speed change leaves — the
/// last frame stands on its own.
fn sample_clip(clip: &ClipView<'_>, local_s: f64) -> DeltaFrame {
    // A clock a rounding error either side of a frame edge is on that edge,
    // and is snapped to it rather than nudged past it: biasing the position
    // instead would bias every interpolation between frames by the same
    // amount.
    let raw = (local_s * FLOOR_TICK_HZ).max(0.0);
    let nearest = raw.round();
    let position = if (raw - nearest).abs() < CLOCK_EPS_S * FLOOR_TICK_HZ {
        nearest
    } else {
        raw
    };
    let index = position.floor() as usize;
    let frames = clip.frames();
    if index + 1 >= frames {
        return clip.frame(frames - 1);
    }
    interpolate(
        &clip.frame(index),
        &clip.frame(index + 1),
        position - index as f64,
    )
}

/// Interpolate between two frames of one clip, `s` of the way from `a` to `b`.
///
/// Both frames come from the same clip and so carry the same keys; a channel
/// absent from `a` is absent from `b`.
fn interpolate(a: &DeltaFrame, b: &DeltaFrame, s: f64) -> DeltaFrame {
    DeltaFrame {
        head: match (a.head, b.head) {
            (Some(from), Some(to)) => Some(interpolate_pose(&from, &to, s)),
            _ => a.head,
        },
        antennas: match (a.antennas, b.antennas) {
            (Some(from), Some(to)) => Some([lerp(from[0], to[0], s), lerp(from[1], to[1], s)]),
            _ => a.antennas,
        },
        body_yaw: match (a.body_yaw, b.body_yaw) {
            (Some(from), Some(to)) => Some(lerp(from, to, s)),
            _ => a.body_yaw,
        },
    }
}

/// Write one frame of deltas into the record a slot holds one in, channel by
/// channel.
///
/// A channel the frame does not drive leaves its fields zeroed rather than
/// carrying an older delta the presence flag says nothing about — and the
/// head's zero is refused as a rotation by [`read_frozen`], which is the
/// backstop for a flag flipped over a row nothing wrote.
fn write_frozen(out: &mut ClipDelta, frame: &DeltaFrame) {
    out.head_present = frame.head.is_some().into();
    match &frame.head {
        Some(head) => write_pose(&mut out.head_pos, &mut out.head_quat, head),
        None => clear_pose(&mut out.head_pos, &mut out.head_quat),
    }
    out.body_yaw_present = frame.body_yaw.is_some().into();
    out.body_yaw = frame.body_yaw.unwrap_or(0.0);
    out.antennas_present = frame.antennas.is_some().into();
    let [right, left] = frame.antennas.unwrap_or([0.0, 0.0]);
    out.antenna_right = right;
    out.antenna_left = left;
}

/// The frame those fields describe.
///
/// # Errors
///
/// [`PlayerStateError::NotADelta`] for a head delta whose rotation is not one.
fn read_frozen(slot: &ClipDelta) -> Result<DeltaFrame, PlayerStateError> {
    Ok(DeltaFrame {
        head: bool::from(slot.head_present)
            .then(|| read_pose(&slot.head_pos, &slot.head_quat))
            .transpose()?,
        antennas: bool::from(slot.antennas_present)
            .then_some([slot.antenna_right, slot.antenna_left]),
        body_yaw: bool::from(slot.body_yaw_present).then_some(slot.body_yaw),
    })
}

/// The three blend weights the record holds.
fn weights_of(slot: &ClipWeights) -> ChannelWeights {
    let mut weights = ChannelWeights::zero();
    weights.set(Channel::Head, slot.head);
    weights.set(Channel::BodyYaw, slot.body_yaw);
    weights.set(Channel::Antennas, slot.antennas);
    weights
}

/// Write all three blend weights.
fn write_weights(out: &mut ClipWeights, weights: ChannelWeights) {
    out.head = weights.get(Channel::Head);
    out.body_yaw = weights.get(Channel::BodyYaw);
    out.antennas = weights.get(Channel::Antennas);
}

/// Write all three fade-out ramps, milliseconds.
fn write_ramps(out: &mut ClipRamps, ramps: &PerChannel<u32>) {
    out.head = *ramps.get(Channel::Head);
    out.body_yaw = *ramps.get(Channel::BodyYaw);
    out.antennas = *ramps.get(Channel::Antennas);
}

/// One channel's fade-out ramp, milliseconds.
fn ramp_of(slot: &ClipRamps, channel: Channel) -> u32 {
    match channel {
        Channel::Head => slot.head,
        Channel::BodyYaw => slot.body_yaw,
        Channel::Antennas => slot.antennas,
    }
}

/// Write one channel's fade-out ramp, milliseconds.
fn set_ramp(out: &mut ClipRamps, channel: Channel, ms: u32) {
    match channel {
        Channel::Head => out.head = ms,
        Channel::BodyYaw => out.body_yaw = ms,
        Channel::Antennas => out.antennas = ms,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use brenn_reachy__clips__player_clk_rs::ClipPlayerSnapWire;
    use brenn_reachy__cogs__config_clk_rs::ClipLibraryConfigWire;

    use crate::config::{ValidatedLibrary, write_clip};

    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

    use crate::format::Clip;

    use crate::compose::compose;
    use crate::speed::ClipLimits;
    use brenn_reachy__motion__joints_clk_rs::JointFlags;
    use reachy_kin::{LegAngles, inverse_kinematics};
    use reachy_motion::{
        ArmRecord, CommandDisposition, JointStep, JointTargets, JointVector, MotionCommand,
        MotionConfig, MotionSnapWire, TickInputs, TickOutputs, arm, motion_tick,
    };

    /// One way of spoiling a frozen frame, and what to call it in a message.
    type Spoiler = (&'static str, fn(&mut ClipDelta));

    /// A row of a player's own: the state is the slot, so a case that drives a
    /// player owns the bytes it lives in.
    ///
    /// The three doors are wrapped rather than called directly because a player
    /// borrows its row for its whole life, and the borrow is easier to see
    /// where the row is a value with a name.
    struct Row(ClipPlayerSnapWire);

    impl Row {
        /// A row nothing has written.
        fn new() -> Self {
            Self(ClipPlayerSnapWire::new())
        }

        /// The state, for a player to live in.
        fn state(&mut self) -> &mut ClipPlayerSnap {
            self.0.validate_mut().expect("a row of this build's own")
        }

        /// What the row holds now, with no player in it.
        fn held(&self) -> &ClipPlayerSnap {
            self.0.validate().expect("a row of this build's own")
        }

        /// A player of `motion` started at the top.
        fn play<'c>(&mut self, motion: MotionView<'c>, speed: f64) -> ClipPlayer<'_, 'c> {
            ClipPlayer::new(motion, speed, self.state())
        }

        /// A player of `motion` joined `at` into the run.
        fn join<'c>(
            &mut self,
            motion: MotionView<'c>,
            speed: f64,
            at: Duration,
        ) -> ClipPlayer<'_, 'c> {
            ClipPlayer::joining_at(motion, speed, at, self.state())
        }

        /// The player this row was left by, playing `motion` — the two calls a
        /// host makes, in the order it makes them.
        fn resume<'c>(
            &mut self,
            motion: MotionView<'c>,
        ) -> Result<ClipPlayer<'_, 'c>, PlayerStateError> {
            ClipPlayer::resumable(&motion, self.held())?;
            Ok(ClipPlayer::over(motion, self.state()))
        }

        /// A copy of the bytes, for a case comparing one state against another.
        fn bytes(&self) -> Vec<u8> {
            clockwork_rs::blob_as_bytes(&self.0).to_vec()
        }
    }

    /// The bounds these fixtures load under: generous ones.
    ///
    /// What is under test here is the clock and the weight ramps, so the
    /// fixtures use round numbers — a tenth of a radian a frame,
    /// a zero-length blend — that the machine's real per-tick bounds would
    /// floor or refuse outright. Deriving against wide bounds keeps those
    /// numbers exactly as written. The derivation itself is pinned in
    /// `speed.rs`, and what it does to a load in `format.rs`.
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

    use crate::format::{ClipDoc, FrameDoc};

    const TICK: Duration = Duration::from_millis(20);

    /// An antennas-only clip whose right antenna walks `values`.
    fn antenna_clip(name: &str, values: &[f64], blend_ms: u32) -> Clip {
        let doc = ClipDoc {
            version: 1,
            kind: "clip".to_owned(),
            name: name.to_owned(),
            description: None,
            channels: vec![Channel::Antennas],
            frame_hz: FLOOR_TICK_HZ,
            max_speed: 2.0,
            blend_in_ms: Some(blend_ms),
            blend_out_ms: Some(blend_ms),
            frames: values
                .iter()
                .map(|value| FrameDoc {
                    antennas: Some([*value, -*value]),
                    ..FrameDoc::default()
                })
                .collect(),
        };
        Clip::from_doc(doc, &limits()).expect("clip is valid")
    }

    /// A head-only clip whose translation walks `values` along z.
    fn head_clip(name: &str, values: &[f64], blend_ms: u32) -> Clip {
        let doc = ClipDoc {
            version: 1,
            kind: "clip".to_owned(),
            name: name.to_owned(),
            description: None,
            channels: vec![Channel::Head],
            frame_hz: FLOOR_TICK_HZ,
            max_speed: 2.0,
            blend_in_ms: Some(blend_ms),
            blend_out_ms: Some(blend_ms),
            frames: values
                .iter()
                .map(|value| FrameDoc {
                    dt: Some([0.0, 0.0, *value]),
                    dq: Some([1.0, 0.0, 0.0, 0.0]),
                    ..FrameDoc::default()
                })
                .collect(),
        };
        Clip::from_doc(doc, &limits()).expect("clip is valid")
    }

    /// A head-only clip whose frames carry `rotations` and no translation.
    fn head_rotation_clip(name: &str, rotations: &[UnitQuaternion<f64>]) -> Clip {
        let doc = ClipDoc {
            version: 1,
            kind: "clip".to_owned(),
            name: name.to_owned(),
            description: None,
            channels: vec![Channel::Head],
            frame_hz: FLOOR_TICK_HZ,
            max_speed: 2.0,
            blend_in_ms: Some(0),
            blend_out_ms: Some(0),
            frames: rotations
                .iter()
                .map(|rotation| FrameDoc {
                    dt: Some([0.0, 0.0, 0.0]),
                    dq: Some([rotation.w, rotation.i, rotation.j, rotation.k]),
                    ..FrameDoc::default()
                })
                .collect(),
        };
        Clip::from_doc(doc, &limits()).expect("clip is valid")
    }

    /// The message a motion is played out of, owned by the case that plays it.
    ///
    /// There is one storage of a track — the configuration message — so a
    /// fixture writes its loaded clips into one through the emitter's own half
    /// of the mapping, states the motion over them, and hands out views of it.
    /// The motion under test is always the library's first.
    struct Track(Box<ClipLibraryConfigWire>);

    impl Track {
        /// `clip`, as the emitter would write it, and the one-segment motion it
        /// plays as.
        fn of(clip: &Clip) -> Self {
            Self::composed(&[clip], 0, &[(0, 1.0, 0)])
        }

        /// A motion over `clips`, opening on `lead_gap_ms` of hold and made of
        /// `segments` — each a clip's index, the speed the flattening left on
        /// it, and the hold that follows.
        fn composed(clips: &[&Clip], lead_gap_ms: u32, segments: &[(u16, f64, u32)]) -> Self {
            let mut message = Box::new(ClipLibraryConfigWire::new());
            {
                let out = message.clear_valid();
                for clip in clips {
                    let slot = out.clips.try_grow().expect("the clips fit");
                    write_clip(clip, slot).expect("the clip fits the message");
                }
                let motion = out.motions.try_grow().expect("one motion fits");
                motion.lead_gap_ms = lead_gap_ms;
                for (clip_id, speed, gap_after_ms) in segments {
                    let slot = motion.segments.try_grow().expect("the segments fit");
                    slot.clip_id = *clip_id;
                    slot.speed = *speed;
                    slot.gap_after_ms = *gap_after_ms;
                }
            }
            Self(message)
        }

        /// A view of the motion, for a player to play.
        fn view(&self) -> MotionView<'_> {
            let library = self.0.validate().expect("a written library validates");
            ValidatedLibrary::of(library)
                .expect("a library the emitter wrote is playable")
                .playable_motion(0)
                .expect("the fixture's own motion")
        }
    }

    /// A clip driving all three channels, with ramps that outlast a tick: a
    /// player of it populates every field of the row, which is what the
    /// refusals below spoil one at a time.
    fn busy_clip() -> Clip {
        let doc = ClipDoc {
            version: 1,
            kind: "clip".to_owned(),
            name: "busy".to_owned(),
            description: None,
            channels: vec![Channel::Head, Channel::BodyYaw, Channel::Antennas],
            frame_hz: FLOOR_TICK_HZ,
            max_speed: 2.0,
            blend_in_ms: Some(100),
            blend_out_ms: Some(100),
            frames: (0..20)
                .map(|index| {
                    let step = f64::from(index);
                    let angle = 0.001 * step;
                    FrameDoc {
                        dt: Some([0.0, 0.0, 0.0002 * step]),
                        dq: Some([(angle / 2.0).cos(), 0.0, 0.0, (angle / 2.0).sin()]),
                        body_yaw: Some(0.002 * step),
                        antennas: Some([0.003 * step, -0.003 * step]),
                    }
                })
                .collect(),
        };
        Clip::from_doc(doc, &limits()).expect("clip is valid")
    }

    /// Advance a player until it terminates, collecting every sample.
    fn run(player: &mut ClipPlayer<'_, '_>, limit: usize) -> Vec<OverlaySample> {
        let mut samples = Vec::new();
        for _ in 0..limit {
            match player.advance(TICK) {
                Some(sample) => samples.push(sample),
                None => return samples,
            }
        }
        panic!("player did not terminate within {limit} ticks");
    }

    /// The round-trip law over `track`, joined at `join` and invoked at
    /// `speed`: a player picked up out of its row before every call plays
    /// exactly what one that was never put down plays.
    ///
    /// Asserted at every tick, on two counts — the sample is the same sample and
    /// the successor row is the same bytes — because a pick-up that reads a
    /// field wrong shows up in whichever of the two that field feeds. The
    /// picked-up side goes through the checked door, so the law also says every
    /// state a run reaches is one a host may pick up.
    fn the_resume_law_holds(track: &Track, speed: f64, join: Duration) {
        let mut held = Row::new();
        let mut crossed = Row::new();
        let _ = held.join(track.view(), speed, join);
        let _ = crossed.join(track.view(), speed, join);
        for tick in 0..500 {
            let expected = ClipPlayer::over(track.view(), held.state()).advance(TICK);
            let got = crossed
                .resume(track.view())
                .unwrap_or_else(|error| panic!("tick {tick}: a live player's own state: {error}"))
                .advance(TICK);
            assert_eq!(got, expected, "tick {tick}");
            assert_eq!(
                crossed.bytes(),
                held.bytes(),
                "tick {tick}: the successor state differs"
            );
            if expected.is_none() {
                return;
            }
        }
        panic!("player did not terminate within 500 ticks");
    }

    #[test]
    fn a_clip_plays_its_frames_in_order() {
        // No blend, so the weight is full from the first tick and the samples
        // are the frames themselves.
        let track = Track::of(&antenna_clip("walk", &[0.0, 0.1, 0.2, 0.3], 0));
        let mut row_player = Row::new();
        let mut player = row_player.play(track.view(), 1.0);
        let samples = run(&mut player, 20);
        let right: Vec<f64> = samples
            .iter()
            .map(|sample| sample.frame.antennas.expect("antennas driven")[0])
            .collect();
        assert_eq!(&right[..4], &[0.0, 0.1, 0.2, 0.3]);
        assert!(player.is_finished());
    }

    #[test]
    fn frames_interpolate_between_samples_at_fractional_speed() {
        let track = Track::of(&antenna_clip("walk", &[0.0, 0.1, 0.2, 0.3], 0));
        let mut row_player = Row::new();
        let mut player = row_player.play(track.view(), 0.5);
        let samples = run(&mut player, 30);
        let right: Vec<f64> = samples
            .iter()
            .map(|sample| sample.frame.antennas.expect("antennas driven")[0])
            .collect();
        // Half speed lands on every half-frame.
        for (index, value) in right.iter().take(7).enumerate() {
            let expected = (index as f64) * 0.05;
            assert!((value - expected).abs() < 1e-12, "tick {index}: {value}");
        }
    }

    #[test]
    fn the_head_translation_interpolates_between_frames() {
        let track = Track::of(&head_clip("lift", &[0.0, 0.02], 0));
        let mut row_player = Row::new();
        let mut player = row_player.play(track.view(), 0.5);
        let first = player.advance(TICK).expect("playing");
        let second = player.advance(TICK).expect("playing");
        assert!((first.frame.head.expect("head driven").translation.vector.z).abs() < 1e-12);
        assert!(
            (second.frame.head.expect("head driven").translation.vector.z - 0.01).abs() < 1e-12
        );
    }

    #[test]
    fn speed_scales_the_clock() {
        let frames: Vec<f64> = (0..20).map(|index| f64::from(index) * 0.01).collect();
        let track = Track::of(&antenna_clip("walk", &frames, 0));
        let mut row_fast = Row::new();
        let mut fast = row_fast.play(track.view(), 2.0);
        let mut row_slow = Row::new();
        let mut slow = row_slow.play(track.view(), 1.0);
        let fast_ticks = run(&mut fast, 100).len();
        let slow_ticks = run(&mut slow, 100).len();
        assert_eq!(slow_ticks, 20);
        assert_eq!(fast_ticks, 10);
    }

    #[test]
    fn lateness_advances_one_period_not_the_stall() {
        let frames: Vec<f64> = (0..50).map(|index| f64::from(index) * 0.01).collect();
        let track = Track::of(&antenna_clip("walk", &frames, 0));
        let mut row_player = Row::new();
        let mut player = row_player.play(track.view(), 1.0);
        let first = player.advance(Duration::from_secs(1)).expect("playing");
        assert_eq!(first.frame.antennas.expect("antennas driven")[0], 0.0);
        let second = player.advance(Duration::from_secs(1)).expect("playing");
        assert!((second.frame.antennas.expect("antennas driven")[0] - 0.01).abs() < 1e-12);
    }

    #[test]
    fn blend_in_ramps_the_weight_from_zero() {
        // 100 ms of ramp is five ticks, from the zero the first sample reports.
        let track = Track::of(&antenna_clip("walk", &[0.1; 20], 100));
        let mut row_player = Row::new();
        let mut player = row_player.play(track.view(), 1.0);
        let weights: Vec<f64> = (0..6)
            .map(|_| {
                player
                    .advance(TICK)
                    .expect("playing")
                    .weights
                    .get(Channel::Antennas)
            })
            .collect();
        for (index, weight) in weights.iter().enumerate() {
            let expected = (index as f64 * 0.2).min(1.0);
            assert!((weight - expected).abs() < 1e-12, "tick {index}: {weight}");
        }
    }

    #[test]
    fn blend_ramps_ignore_the_invocation_speed() {
        let track = Track::of(&antenna_clip("walk", &[0.1; 40], 100));
        let mut row_fast = Row::new();
        let mut fast = row_fast.play(track.view(), 2.0);
        let mut row_slow = Row::new();
        let mut slow = row_slow.play(track.view(), 1.0);
        for _ in 0..3 {
            let fast_weight = fast
                .advance(TICK)
                .expect("playing")
                .weights
                .get(Channel::Antennas);
            let slow_weight = slow
                .advance(TICK)
                .expect("playing")
                .weights
                .get(Channel::Antennas);
            assert!((fast_weight - slow_weight).abs() < 1e-12);
        }
    }

    #[test]
    fn blend_out_runs_after_the_last_frame_and_then_terminates() {
        let track = Track::of(&antenna_clip("walk", &[0.1; 10], 100));
        let mut row_player = Row::new();
        let mut player = row_player.play(track.view(), 1.0);
        let samples = run(&mut player, 40);
        // Ten frames of clip, then five ticks of fade: the sample whose weight
        // reaches zero is the terminal one and is not emitted.
        assert_eq!(samples.len(), 14);
        let tail: Vec<f64> = samples[10..]
            .iter()
            .map(|s| s.weights.get(Channel::Antennas))
            .collect();
        for (index, weight) in tail.iter().enumerate() {
            let expected = 1.0 - (index + 1) as f64 * 0.2;
            assert!((weight - expected).abs() < 1e-12, "fade {index}: {weight}");
        }
        assert!(player.is_finished());
        assert!(player.advance(TICK).is_none());
    }

    #[test]
    fn a_fading_channel_holds_the_last_delta_it_was_given() {
        // Five frames — 100 ms — so the ramps fit inside the clip; the last
        // three hold the delta the fade runs against.
        let track = Track::of(&antenna_clip("walk", &[0.0, 0.1, 0.7, 0.7, 0.7], 100));
        let mut row_player = Row::new();
        let mut player = row_player.play(track.view(), 1.0);
        let samples = run(&mut player, 40);
        for sample in &samples[3..] {
            assert!((sample.frame.antennas.expect("antennas driven")[0] - 0.7).abs() < 1e-12);
        }
    }

    #[test]
    fn a_mid_motion_join_starts_at_the_right_offset_with_zero_weight() {
        let frames: Vec<f64> = (0..20).map(|index| f64::from(index) * 0.01).collect();
        let track = Track::of(&antenna_clip("walk", &frames, 100));
        let mut row_player = Row::new();
        let mut player = row_player.join(track.view(), 1.0, Duration::from_millis(200));
        // Ten frames in: the first sample is the eleventh frame's delta, at the
        // zero weight a fresh blend-in starts from rather than stepping onto.
        let sample = player.advance(TICK).expect("playing");
        assert!((sample.frame.antennas.expect("antennas driven")[0] - 0.10).abs() < 1e-12);
        assert_eq!(sample.weights, ChannelWeights::zero());
        let next = player.advance(TICK).expect("playing");
        assert!((next.frame.antennas.expect("antennas driven")[0] - 0.11).abs() < 1e-12);
        assert!((next.weights.get(Channel::Antennas) - 0.2).abs() < 1e-12);
    }

    #[test]
    fn a_join_scales_the_offset_by_speed() {
        let frames: Vec<f64> = (0..40).map(|index| f64::from(index) * 0.01).collect();
        let track = Track::of(&antenna_clip("walk", &frames, 0));
        let mut row_player = Row::new();
        let mut player = row_player.join(track.view(), 2.0, Duration::from_millis(200));
        // Two hundred milliseconds at 2× is twenty frames of clip, and each
        // subsequent tick is two more.
        let sample = player.advance(TICK).expect("playing");
        assert!((sample.frame.antennas.expect("antennas driven")[0] - 0.20).abs() < 1e-12);
        let next = player.advance(TICK).expect("playing");
        assert!((next.frame.antennas.expect("antennas driven")[0] - 0.22).abs() < 1e-12);
    }

    #[test]
    fn the_head_rotation_walks_the_geodesic_between_frames() {
        // Two frames rotating about different axes, played at half speed so the
        // sample between them is the half-way interpolation and not a frame.
        let first = UnitQuaternion::from_scaled_axis(Vector3::y() * 0.4);
        let second = first * UnitQuaternion::from_scaled_axis(Vector3::z() * 0.6);
        let clip = head_rotation_clip("turn", &[first, second]);
        let track = Track::of(&clip);
        let mut row_player = Row::new();
        let mut player = row_player.play(track.view(), 0.5);

        let start = player.advance(TICK).expect("playing").frame.head;
        let middle = player.advance(TICK).expect("playing").frame.head;
        let start = start.expect("head driven");
        let middle = middle.expect("head driven").rotation;

        let expected = interpolate_pose(
            &Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), first),
            &Isometry3::from_parts(Translation3::new(0.0, 0.0, 0.0), second),
            0.5,
        );
        assert!((middle.angle_to(&expected.rotation)).abs() < 1e-12);

        // And it is half of the geodesic, measured from the frame it left.
        let whole = (start.rotation.inverse() * second).angle();
        let half = (start.rotation.inverse() * middle).angle();
        assert!((half - whole / 2.0).abs() < 1e-12, "{half} vs {whole}");
    }

    #[test]
    fn a_derived_ramp_longer_than_the_clip_plays_the_whole_track_at_no_weight() {
        // An authored ramp longer than its clip is refused at load, so the only
        // way to reach this state is the floor: against a tight antenna step
        // bound the derivation stretches a zero ramp to many times the clip's
        // length, which is stretch-and-report rather than refusal. The
        // accepting load below is that exemption; the weights are what the
        // player does with the result.
        let tight = ClipLimits {
            max_step: JointStep {
                antennas: 0.001,
                ..limits().max_step
            },
            ..limits()
        };
        let doc = ClipDoc {
            version: 1,
            kind: "clip".to_owned(),
            name: "walk".to_owned(),
            description: None,
            channels: vec![Channel::Antennas],
            frame_hz: FLOOR_TICK_HZ,
            max_speed: 2.0,
            blend_in_ms: Some(0),
            blend_out_ms: Some(0),
            frames: (0..10)
                .map(|_| FrameDoc {
                    antennas: Some([0.5, -0.5]),
                    ..FrameDoc::default()
                })
                .collect(),
        };
        let clip = Clip::from_doc(doc, &tight).expect("a floored ramp is not a refusal");
        // Ten frames at 50 Hz is 200 ms of clip; the floor is orders longer.
        assert!(clip.blend_in_ms() > 10_000, "{}", clip.blend_in_ms());

        let track = Track::of(&clip);
        let mut row_player = Row::new();
        let mut player = row_player.play(track.view(), 1.0);
        let samples = run(&mut player, 200);
        for sample in &samples[..10] {
            assert!(sample.weights.get(Channel::Antennas) < 0.01);
        }
    }

    #[test]
    #[should_panic(expected = "finite and positive")]
    fn a_zero_speed_player_is_refused() {
        let track = Track::of(&antenna_clip("walk", &[0.1; 4], 0));
        let mut row = Row::new();
        let _ = row.play(track.view(), 0.0);
    }

    #[test]
    #[should_panic(expected = "finite and positive")]
    fn a_nan_speed_player_is_refused() {
        let track = Track::of(&antenna_clip("walk", &[0.1; 4], 0));
        let mut row = Row::new();
        let _ = row.join(track.view(), f64::NAN, Duration::from_millis(100));
    }

    #[test]
    fn a_join_past_the_whole_motion_is_finished_immediately() {
        let track = Track::of(&antenna_clip("walk", &[0.1; 4], 0));
        let mut row_player = Row::new();
        let mut player = row_player.join(track.view(), 1.0, Duration::from_secs(5));
        assert!(player.advance(TICK).is_none());
        assert!(player.is_finished());
    }

    /// A machine armed and holding the neutral pose, and the goals it holds.
    ///
    /// The state is the slot: the caller owns the bytes for the life of the run.
    fn armed_at_neutral(cfg: &MotionConfig) -> (MotionSnapWire, JointVector) {
        let targets = JointTargets::default();
        let mut angles = LegAngles::default();
        inverse_kinematics(&cfg.geom, &targets.head_pose_body, &mut angles)
            .expect("neutral is reachable");
        let joints = JointVector {
            body_yaw: targets.body_yaw,
            legs: angles.0,
            antennas: targets.antennas,
        };
        let record = ArmRecord::solve(&cfg.geom, &cfg.fk, &joints, &[targets.head_pose_body])
            .expect("the neutral angles close the linkage");
        let mut slot = MotionSnapWire::new();
        arm(slot.clear_valid(), &record, JointFlags::NONE);
        (slot, joints)
    }

    /// The whole playback path against the machine's own per-tick guard: a clip
    /// loaded under the shipped bounds, played at the speed the loader derived
    /// for it, composed over a static neutral base and handed to the tick one
    /// setpoint per period. Every period is taken and none is refused, which is
    /// what the derivation's margin claims over a base that is not moving.
    ///
    /// The end of the chain the offline derivation exists to protect: `speed.rs`
    /// pins the arithmetic, and this pins the arithmetic against the code that
    /// actually decides what goes on the wire.
    #[test]
    fn a_clip_at_its_derived_speed_plays_through_the_real_tick() {
        let cfg = MotionConfig::default();
        let shipped = ClipLimits::from_motion_config(&cfg);

        // All three channels moving at once, at amplitudes a recording carries.
        let doc = ClipDoc {
            version: 1,
            kind: "clip".to_owned(),
            name: "test/three-channels".to_owned(),
            description: None,
            channels: vec![Channel::Head, Channel::Antennas, Channel::BodyYaw],
            frame_hz: FLOOR_TICK_HZ,
            max_speed: 2.0,
            blend_in_ms: None,
            blend_out_ms: None,
            frames: (0..40)
                .map(|index| {
                    let phase = f64::from(index) * 0.25;
                    FrameDoc {
                        dt: Some([0.0, 0.0, 0.004 * phase.sin()]),
                        dq: Some([1.0, 0.0, 0.0, 0.0]),
                        antennas: Some([0.3 * phase.sin(), -0.2 * phase.cos()]),
                        body_yaw: Some(0.15 * phase.sin()),
                    }
                })
                .collect(),
        };
        let clip = Clip::from_doc(doc, &shipped).expect("the track stays in the envelope");
        let speed = clip.max_speed();
        assert!(speed >= 1.0, "a clip plays at its own recorded speed");
        let track = Track::of(&clip);
        let mut row = Row::new();
        let (ticks, moved) = play_through_the_tick(&cfg, row.play(track.view(), speed));
        assert!(moved, "the clip commanded something");
        assert!(ticks > 20, "the whole track played: {ticks} ticks");
    }

    /// The same guard, joined mid-window at the far end of a track that is both
    /// far from zero and moving at its ceiling.
    ///
    /// This is the tick where the two terms of a blend-in coincide: the ramp
    /// commands `Δw` of the frame it joined at — the clip's largest delta — and
    /// the clock commands one period of advance beside it, on the same joint,
    /// on the same period. A floor and a ceiling each sized to spend the whole
    /// usable step would together spend twice it and be refused here.
    #[test]
    fn a_late_join_at_a_clips_far_end_stays_inside_the_step_bound() {
        let cfg = MotionConfig::default();
        let shipped = ClipLimits::from_motion_config(&cfg);

        // An antenna held far off neutral throughout, jogging back and forth by
        // a whole frame-step: every frame is near the track's largest delta and
        // every pair asks for the ceiling's worth of travel, so a ramp climbing
        // over it spends both terms on the same tick.
        let step = cfg.max_step.antennas / 3.0;
        let held = step * 20.0;
        let doc = ClipDoc {
            version: 1,
            kind: "clip".to_owned(),
            name: "test/far-end".to_owned(),
            description: None,
            channels: vec![Channel::Antennas],
            frame_hz: FLOOR_TICK_HZ,
            max_speed: 2.0,
            blend_in_ms: Some(0),
            blend_out_ms: Some(0),
            frames: (0..60)
                .map(|index| {
                    // A triangle of period eight, so the jog survives a clock
                    // advancing two frames a tick.
                    let phase = index % 8;
                    let up = if phase <= 4 { phase } else { 8 - phase };
                    FrameDoc {
                        dt: None,
                        dq: None,
                        antennas: Some([held + f64::from(up) * step, 0.0]),
                        body_yaw: None,
                    }
                })
                .collect(),
        };
        let clip = Clip::from_doc(doc, &shipped).expect("antennas stay in range");
        let speed = clip.max_speed();
        // A third of the way in, so the ramp is still climbing over a delta at
        // the track's largest while the clock keeps advancing under it.
        let joined = Duration::from_secs_f64(clip.duration_s() / speed / 3.0);
        let track = Track::of(&clip);
        let mut row = Row::new();
        let (ticks, moved) = play_through_the_tick(&cfg, row.join(track.view(), speed, joined));
        assert!(moved, "the join commanded something");
        assert!(ticks > 5, "the join played and faded: {ticks} ticks");
    }

    /// Play a player out through the real tick over a static neutral base,
    /// asserting every period is taken. Answers how many periods ran and
    /// whether any of them put a goal on the wire.
    fn play_through_the_tick(cfg: &MotionConfig, mut player: ClipPlayer<'_, '_>) -> (u32, bool) {
        let base = JointTargets::default();
        let (mut slot, pinned) = armed_at_neutral(cfg);
        let state = slot.validate_mut().expect("an armed state validates");
        let mut present = pinned;
        let mut ticks = 0u32;
        let mut moved = false;
        while let Some(sample) = player.advance(TICK) {
            let composed = compose(base, &[sample]);
            let mut out = TickOutputs::default();
            motion_tick(
                cfg,
                state,
                &TickInputs {
                    now: TICK * ticks,
                    period: TICK,
                    present: Some(&present),
                    command: Some(&MotionCommand::Track(composed)),
                    health: None,
                },
                &mut out,
            );
            assert_eq!(
                out.report.command,
                CommandDisposition::Tracked,
                "tick {ticks} was refused"
            );
            assert_eq!(out.report.fault, None, "tick {ticks}");
            if let Some(goal) = out.goal {
                moved = true;
                present = goal;
            }
            ticks += 1;
        }
        (ticks, moved)
    }

    #[test]
    fn a_pick_up_plays_what_a_player_never_put_down_plays() {
        the_resume_law_holds(&Track::of(&busy_clip()), 1.0, Duration::ZERO);
    }

    #[test]
    fn a_pick_up_holds_at_fractional_and_double_speed() {
        for speed in [0.5, 1.5, 2.0] {
            the_resume_law_holds(&Track::of(&busy_clip()), speed, Duration::ZERO);
        }
    }

    #[test]
    fn a_pick_up_holds_for_a_player_that_joined_mid_motion() {
        // Inside the blend-in, past it, and inside the last frames: the three
        // offsets whose bookkeeping differs.
        for join in [40, 160, 380] {
            the_resume_law_holds(&Track::of(&busy_clip()), 1.0, Duration::from_millis(join));
        }
    }

    #[test]
    fn a_pick_up_holds_for_a_player_that_was_already_finished() {
        let track = Track::of(&antenna_clip("walk", &[0.1; 4], 0));
        the_resume_law_holds(&track, 1.0, Duration::from_secs(5));
    }

    #[test]
    fn a_run_picked_up_mid_way_plays_the_same_tail() {
        let track = Track::of(&busy_clip());
        let mut row = Row::new();
        let mut player = row.play(track.view(), 1.0);
        for _ in 0..7 {
            player.advance(TICK).expect("playing");
        }
        let tail = run(&mut player, 200);
        let _ = player;
        // The same seven ticks again, in a row of its own, then picked up.
        let mut second = Row::new();
        let mut player = second.play(track.view(), 1.0);
        for _ in 0..7 {
            player.advance(TICK).expect("playing");
        }
        let _ = player;
        let mut resumed = second
            .resume(track.view())
            .expect("a live player's own state");
        assert_eq!(run(&mut resumed, 200), tail);
    }

    /// A slot reused for another clip is the case this refusal exists for: a
    /// host holding players by index, and a script that put a different clip in
    /// that slot. Nothing about the numbers is out of range — the other clip is
    /// long enough for the clock the row holds — so the fingerprint is the only
    /// thing that tells the two apart.
    #[test]
    fn a_state_left_by_another_track_is_refused() {
        let mine = Track::of(&busy_clip());
        let other = Track::of(&antenna_clip("other", &[0.1; 20], 100));
        let mut row = Row::new();
        let mut player = row.play(mine.view(), 1.0);
        for _ in 0..3 {
            player.advance(TICK).expect("playing");
        }
        let _ = player;
        let fingerprint = row.held().track;
        assert!(
            row.held().clock_s < other.view().duration_s(),
            "the other clip's own numbers already refuse this state"
        );

        assert_eq!(
            refusal(&other, &row),
            PlayerStateError::DifferentTrack {
                held: fingerprint,
                restored_over: motion_fingerprint(&other.view()),
            }
        );
    }

    /// A frozen rotation that is not unit is refused, rather than scaling the
    /// composed target and landing in the envelope check as an unexplained pose.
    #[test]
    fn a_frozen_rotation_that_is_not_unit_is_refused() {
        let (track, mut row) = live_row();
        let scaled = {
            let state = row.state();
            assert!(
                bool::from(state.frozen.head_present),
                "the busy clip drives the head"
            );
            let quat = &mut state.frozen.head_quat;
            quat.w *= 1.5;
            quat.x *= 1.5;
            quat.y *= 1.5;
            quat.z *= 1.5;
            (quat.w * quat.w + quat.x * quat.x + quat.y * quat.y + quat.z * quat.z).sqrt()
        };
        assert!((scaled - 1.0).abs() > 1e-9);
        assert_eq!(
            refusal(&track, &row),
            PlayerStateError::NotADelta(PoseSnapshotError::NotARotation(scaled))
        );
        // A rotation a few ulps off unit is within tolerance.
        let (track, mut row) = live_row();
        {
            let quat = &mut row.state().frozen.head_quat;
            quat.w *= 1.0 + 1e-12;
        }
        row.resume(track.view())
            .expect("a rotation within the tolerance is picked up");
    }

    /// A row nine ticks into the busy clip: a state with every field populated,
    /// for the refusals to spoil one at a time.
    fn live_row() -> (Track, Row) {
        let track = Track::of(&busy_clip());
        let mut row = Row::new();
        let mut player = row.play(track.view(), 1.0);
        for _ in 0..9 {
            player.advance(TICK).expect("playing");
        }
        let _ = player;
        {
            let held = row.held();
            assert!(bool::from(held.started) && !bool::from(held.finished));
        }
        (track, row)
    }

    /// What picking `row` up over `track` refuses with.
    fn refusal(track: &Track, row: &Row) -> PlayerStateError {
        ClipPlayer::resumable(&track.view(), row.held()).expect_err("refused")
    }

    /// Assert `left` is the refusal `right` names, comparing the two as written
    /// rather than by `PartialEq`.
    ///
    /// Half of these refusals carry the number they refuse, and a NaN is one of
    /// the numbers worth refusing — but a NaN payload is never equal to itself,
    /// so `assert_eq!` on the error would fail against the very value it names.
    fn assert_refused(left: &PlayerStateError, right: &PlayerStateError) {
        assert_eq!(format!("{left:?}"), format!("{right:?}"));
    }

    #[test]
    fn a_speed_that_is_not_finite_and_positive_is_refused() {
        for speed in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let (track, mut row) = live_row();
            row.state().speed = speed;
            assert_refused(
                &refusal(&track, &row),
                &PlayerStateError::BadSpeed { speed },
            );
        }
    }

    #[test]
    fn a_clock_that_is_not_finite_and_non_negative_is_refused() {
        for value in [-1.0, f64::NAN, f64::INFINITY] {
            let (track, mut row) = live_row();
            row.state().clock_s = value;
            assert_refused(
                &refusal(&track, &row),
                &PlayerStateError::BadClock { value },
            );
        }
    }

    #[test]
    fn a_weight_outside_the_unit_range_is_refused() {
        for weight in [-0.1, 1.1, f64::NAN] {
            let (track, mut row) = live_row();
            row.state().weights.body_yaw = weight;
            assert_refused(
                &refusal(&track, &row),
                &PlayerStateError::BadWeight {
                    channel: Channel::BodyYaw,
                    weight,
                },
            );
        }
    }

    /// A fade-out ramp is zero or the exit ramp of one of the motion's clips
    /// and nothing between them: zero until a tick drives the channel, then
    /// whichever clip last drove it. A ramp of a millisecond drops the whole
    /// weighted delta in one period and a ramp of days never lets go of it, and
    /// neither is a step the blend floor was derived for.
    #[test]
    fn a_fade_out_ramp_no_tick_could_have_recorded_is_refused() {
        let clip_ms = Track::of(&busy_clip()).view().blend_out_ms();
        assert_ne!(clip_ms, 0, "the case rests on the clip having an exit ramp");
        for ms in [1, clip_ms + 1, u32::MAX] {
            let (track, mut row) = live_row();
            set_ramp(&mut row.state().ramps, Channel::BodyYaw, ms);
            assert_eq!(
                refusal(&track, &row),
                PlayerStateError::BadRamp {
                    channel: Channel::BodyYaw,
                    ms,
                }
            );
        }

        // Zero is the other reachable value: what a channel carries until a
        // tick has driven it.
        let (track, mut row) = live_row();
        set_ramp(&mut row.state().ramps, Channel::BodyYaw, 0);
        assert!(ClipPlayer::resumable(&track.view(), row.held()).is_ok());
    }

    /// Over a motion of several clips the ramp a channel may hold is any
    /// *driving* clip's exit ramp: the walk covers every segment, and the mask
    /// is what says which of them could have written it.
    ///
    /// The second segment's clip drives the antennas alone, so its ramp is
    /// reachable there and nowhere else — a body-yaw slot holding it is a value
    /// no tick could have recorded.
    #[test]
    fn a_fade_out_ramp_from_a_clip_that_never_drove_the_channel_is_refused() {
        let busy = busy_clip();
        let tail = antenna_clip("tail", &[0.1, 0.2, 0.3, 0.4], 40);
        assert_ne!(
            busy.blend_out_ms(),
            tail.blend_out_ms(),
            "the case does not discriminate"
        );
        let track = Track::composed(&[&busy, &tail], 0, &[(0, 1.0, 0), (1, 1.0, 0)]);
        let live = |channel, ms| {
            let mut row = Row::new();
            {
                let mut player = row.play(track.view(), 1.0);
                for _ in 0..9 {
                    player.advance(TICK).expect("playing");
                }
            }
            set_ramp(&mut row.state().ramps, channel, ms);
            ClipPlayer::resumable(&track.view(), row.held())
        };

        // The later segment's clip drives the antennas, so its exit ramp is
        // one an antennas fade runs on.
        assert!(live(Channel::Antennas, tail.blend_out_ms()).is_ok());
        // It drives nothing else, so the same value on the body yaw is bytes
        // nothing wrote.
        assert_eq!(
            live(Channel::BodyYaw, tail.blend_out_ms()),
            Err(PlayerStateError::BadRamp {
                channel: Channel::BodyYaw,
                ms: tail.blend_out_ms(),
            })
        );
        // And a value neither clip carries is refused wherever it sits.
        let stray = busy.blend_out_ms() + tail.blend_out_ms();
        assert_eq!(
            live(Channel::Antennas, stray),
            Err(PlayerStateError::BadRamp {
                channel: Channel::Antennas,
                ms: stray,
            })
        );
    }

    /// A stored speed past the motion's ceiling is a slot nothing wrote,
    /// refused before the clock races.
    #[test]
    fn a_speed_past_the_motions_ceiling_is_refused() {
        let (track, mut row) = live_row();
        let ceiling = track.view().max_speed();
        row.state().speed = ceiling * 2.0;
        assert_eq!(
            refusal(&track, &row),
            PlayerStateError::PastCeiling {
                speed: ceiling * 2.0,
                ceiling,
            }
        );

        let (track, mut row) = live_row();
        row.state().speed = ceiling;
        assert!(ClipPlayer::resumable(&track.view(), row.held()).is_ok());
    }

    /// A frozen frame drives the clip's channels and no others: it starts as the
    /// zero delta of the mask and is refreshed on those channels alone. A
    /// channel dropped from it would take its whole contribution out of the
    /// composed target in one period, which is the step the fades exist to
    /// prevent, and one added would move a joint the mask says the clip does not
    /// drive.
    #[test]
    fn a_frozen_delta_driving_another_set_of_channels_is_refused() {
        let (track, mut row) = live_row();
        let motion = track.view().mask();
        row.state().frozen.body_yaw_present = false.into();
        let held = ChannelMask::of(Channel::Head).union(ChannelMask::of(Channel::Antennas));
        assert_eq!(
            refusal(&track, &row),
            PlayerStateError::FrozenChannels { held, motion }
        );
    }

    #[test]
    fn a_frozen_delta_that_is_not_finite_is_refused() {
        // Every number a frozen frame carries that the rotation check does not
        // cover: the head's translation, the antennas and the yaw.
        let spoilers: [Spoiler; 3] = [
            ("head translation", |frozen| frozen.head_pos.z = f64::NAN),
            ("antennas", |frozen| frozen.antenna_right = f64::INFINITY),
            ("body yaw", |frozen| frozen.body_yaw = f64::NAN),
        ];
        for (what, spoil) in spoilers {
            let (track, mut row) = live_row();
            {
                let state = row.state();
                state.frozen.antennas_present = true.into();
                state.frozen.body_yaw_present = true.into();
                spoil(&mut state.frozen);
            }
            assert_eq!(
                refusal(&track, &row),
                PlayerStateError::NonFiniteDelta,
                "{what}"
            );
        }
    }

    /// A rotation coordinate that is not a number is refused as no rotation, not
    /// as a non-finite delta: a quaternion whose length is not one is not one,
    /// and a NaN coordinate makes the length NaN. The frame-wide finiteness
    /// check never sees it.
    #[test]
    fn a_frozen_rotation_coordinate_that_is_not_a_number_is_no_rotation() {
        let spoilers: [Spoiler; 2] = [
            ("w", |frozen| frozen.head_quat.w = f64::NAN),
            ("y", |frozen| frozen.head_quat.y = f64::INFINITY),
        ];
        for (what, spoil) in spoilers {
            let (track, mut row) = live_row();
            spoil(&mut row.state().frozen);
            assert!(
                matches!(
                    refusal(&track, &row),
                    PlayerStateError::NotADelta(PoseSnapshotError::NotARotation(_))
                ),
                "rotation coordinate {what}"
            );
        }
    }

    #[test]
    fn a_finish_that_no_tick_could_have_produced_is_refused() {
        let (track, mut row) = live_row();
        let duration_s = track.view().duration_s();
        // Finished with motion left to play.
        row.state().finished = true.into();
        assert_eq!(
            refusal(&track, &row),
            PlayerStateError::ImpossibleFinish {
                why: "has motion left to play"
            }
        );
        // Finished at the end of the motion, but still carrying weight.
        row.state().clock_s = duration_s;
        assert_eq!(
            refusal(&track, &row),
            PlayerStateError::ImpossibleFinish {
                why: "still carries weight"
            }
        );
        // Finished before taking a tick.
        {
            let state = row.state();
            state.started = false.into();
            write_weights(&mut state.weights, ChannelWeights::zero());
        }
        assert_eq!(
            refusal(&track, &row),
            PlayerStateError::ImpossibleFinish {
                why: "has taken no tick"
            }
        );
    }

    #[test]
    fn an_untouched_player_carrying_a_ticks_work_is_refused() {
        let (track, mut row) = live_row();
        row.state().started = false.into();
        assert_eq!(
            refusal(&track, &row),
            PlayerStateError::ImpossibleStart {
                why: "carries weight"
            }
        );
        write_weights(&mut row.state().weights, ChannelWeights::zero());
        row.resume(track.view())
            .expect("a row with no weight on it has taken no tick");
    }

    /// A row reused for a new overlay carries nothing of the one before it: the
    /// player writes every field it owns, and the two the host owns are the
    /// host's to write.
    #[test]
    fn a_row_started_over_an_earlier_player_describes_this_one_only() {
        let track = Track::of(&busy_clip());
        let (_, mut used) = live_row();
        {
            let state = used.state();
            state.active = true.into();
            state.motion_id = 3;
        }
        let mut player = used.join(track.view(), 1.5, Duration::from_millis(60));
        player.advance(TICK);
        let _ = player;

        let mut fresh = Row::new();
        {
            let state = fresh.state();
            state.active = true.into();
            state.motion_id = 3;
        }
        let mut player = fresh.join(track.view(), 1.5, Duration::from_millis(60));
        player.advance(TICK);
        let _ = player;

        assert_eq!(used.bytes(), fresh.bytes());
    }

    #[test]
    fn a_fresh_players_row_is_the_state_it_was_built_in() {
        let track = Track::of(&busy_clip());
        let mut row = Row::new();
        let _ = row.play(track.view(), 1.5);
        let held = row.held();
        assert_eq!(held.speed, 1.5);
        assert_eq!(held.clock_s, 0.0);
        assert_eq!(weights_of(&held.weights), ChannelWeights::zero());
        assert!(!bool::from(held.started));
        assert!(!bool::from(held.finished));
        assert_eq!(
            read_frozen(&held.frozen).expect("a fresh frame is a frame"),
            DeltaFrame::zero(track.view().mask())
        );
    }

    /// A composed motion's clock walks one clip's frames, freezes the last of
    /// them through the hold that follows, and then plays the next clip from its
    /// own first frame, at the speed the flattening left on that segment. The
    /// seam is stepped across, which is what the load-time seam check bounds.
    ///
    /// The trailing hold is the last segment's: the motion is not over until it
    /// has been held, so the run ends where the derived duration says and the
    /// fade begins no earlier.
    #[test]
    fn a_composed_motion_walks_its_clips_across_a_seam_and_through_a_hold() {
        let first = antenna_clip("first", &[0.0, 0.1, 0.2, 0.3], 0);
        let second = antenna_clip("second", &[0.5, 0.6, 0.7, 0.8], 0);
        // Four frames — 80 ms — then 100 ms of hold, then four more frames at
        // 2.0x, which is 40 ms of the motion's clock, then 60 ms of hold.
        let track = Track::composed(&[&first, &second], 0, &[(0, 1.0, 100), (1, 2.0, 60)]);
        let motion = track.view();
        assert!(
            (motion.duration_s() - 0.28).abs() < 1e-12,
            "{}",
            motion.duration_s()
        );
        let mut row = Row::new();
        let mut player = row.play(track.view(), 1.0);
        let right: Vec<f64> = run(&mut player, 40)
            .iter()
            .map(|sample| sample.frame.antennas.expect("antennas driven")[0])
            .collect();
        assert_eq!(
            right,
            vec![
                0.0, 0.1, 0.2, 0.3, // the first clip's own frames
                0.3, 0.3, 0.3, 0.3, 0.3, // its last delta, held through the gap
                0.5, 0.7, // the second clip at 2.0x: every other frame
                0.8, 0.8, 0.8, // its last delta, held through the trailing gap
            ]
        );
        // The walk ends where the derived duration says it does, so the
        // `ImpossibleFinish` check and the player agree about being over.
        assert_eq!(
            right.len(),
            (motion.duration_s() / TICK.as_secs_f64()).round() as usize
        );
        assert!(player.is_finished());
    }

    /// An invocation's speed multiplies the segment's own rather than replacing
    /// it: a motion the flattening left at half speed runs twice as long, and
    /// invoking it at 2.0x buys back exactly that.
    #[test]
    fn the_invocation_speed_compounds_with_the_segments_own() {
        let frames: Vec<f64> = (0..20).map(|index| f64::from(index) * 0.01).collect();
        let clip = antenna_clip("walk", &frames, 0);
        let halved = Track::composed(&[&clip], 0, &[(0, 0.5, 0)]);
        let bare = Track::of(&clip);
        // Twenty frames at 50 Hz is 400 ms of clip.
        let mut row = Row::new();
        assert_eq!(run(&mut row.play(bare.view(), 1.0), 200).len(), 20);
        let mut row = Row::new();
        assert_eq!(run(&mut row.play(halved.view(), 1.0), 200).len(), 40);
        let mut row = Row::new();
        assert_eq!(run(&mut row.play(halved.view(), 2.0), 200).len(), 20);
    }

    /// A leading gap holds the base alone — nothing is driven and no channel
    /// carries weight — and it counts in the motion's duration, so the first
    /// clip starts where the gap ends.
    #[test]
    fn a_leading_gap_holds_the_base_alone_and_counts_in_the_duration() {
        let clip = antenna_clip("walk", &[0.0, 0.1, 0.2, 0.3], 0);
        let track = Track::composed(&[&clip], 60, &[(0, 1.0, 0)]);
        assert!((track.view().duration_s() - 0.14).abs() < 1e-12);
        let mut row = Row::new();
        let mut player = row.play(track.view(), 1.0);
        let samples = run(&mut player, 40);
        // Three ticks of lead: nothing driven, nothing weighted.
        for (tick, sample) in samples[..3].iter().enumerate() {
            assert_eq!(sample.frame.antennas, None, "tick {tick} drove a channel");
            assert_eq!(sample.weights, ChannelWeights::zero(), "tick {tick}");
        }
        assert_eq!(
            samples[3].frame.antennas.expect("antennas driven")[0],
            0.0,
            "the clip starts where the lead gap ends"
        );
    }

    /// A channel the incoming clip does not drive fades out on the outgoing
    /// clip's ramp, holding the last delta it was given, while the channels the
    /// incoming clip does drive ramp in beside it on their own.
    ///
    /// The two ramps differ, so the rates say which clip each weight is running
    /// on: the fade is the leaving clip's 100 ms — 0.2 of weight a period — and
    /// the rise the arriving clip's 40 ms, which is 0.5.
    #[test]
    fn a_fading_channel_holds_its_last_delta_through_a_mask_change() {
        // Five antenna frames — 100 ms — then ten head frames, so the fade and
        // the rise overlap.
        let leaving = antenna_clip("leaving", &[0.0, 0.1, 0.2, 0.7, 0.7], 100);
        let arriving = head_clip("arriving", &[0.01; 10], 40);
        assert_ne!(
            leaving.blend_out_ms(),
            arriving.blend_in_ms(),
            "the case does not discriminate"
        );
        let track = Track::composed(&[&leaving, &arriving], 0, &[(0, 1.0, 0), (1, 1.0, 0)]);
        let mut row = Row::new();
        let mut player = row.play(track.view(), 1.0);
        let samples = run(&mut player, 60);
        // The sixth tick is the first of the second clip: the antennas are no
        // longer driven and the head is.
        let (before, after) = samples.split_at(5);
        assert_eq!(
            before[4].frame.antennas.expect("antennas driven")[0],
            0.7,
            "the last delta the leaving clip gave"
        );
        assert!(
            (before[4].weights.get(Channel::Antennas) - 0.8).abs() < 1e-12,
            "the fade starts from the weight the rise reached"
        );
        // Steps of 0.2 down: the leaving clip's exit ramp, not the arriving
        // clip's entry ramp, which would let go in half the periods.
        for (tick, (expected, rising)) in [(0.6, 0.5), (0.4, 1.0), (0.2, 1.0), (0.0, 1.0)]
            .into_iter()
            .enumerate()
        {
            let sample = &after[tick];
            let weight = sample.weights.get(Channel::Antennas);
            assert!(
                (weight - expected).abs() < 1e-12,
                "tick {tick}: {weight} is not {expected}"
            );
            let head = sample.weights.get(Channel::Head);
            assert!(
                (head - rising).abs() < 1e-12,
                "tick {tick}: the head rose to {head}, not {rising}"
            );
            if expected > 0.0 {
                assert_eq!(
                    sample
                        .frame
                        .antennas
                        .expect("the fading channel is still in the frame")[0],
                    0.7,
                    "tick {tick} let go of the delta it was fading out of"
                );
            }
        }
        // Where the fade lands the channel leaves the frame, which is the delta
        // having returned to the base rather than having vanished off it.
        assert_eq!(after[3].frame.antennas, None);
    }

    /// Two motions that open on the same clip are told apart: the fingerprint
    /// covers the whole walk, so a row left by one is not restored over the
    /// other. One clip's shape pinning an n-segment walk is exactly the
    /// collision the pick-up guard cannot afford.
    #[test]
    fn two_motions_sharing_a_first_clip_have_their_own_fingerprints() {
        let first = antenna_clip("first", &[0.0, 0.1, 0.2, 0.3], 0);
        let second = antenna_clip("second", &[0.5, 0.6, 0.7, 0.8, 0.9], 0);
        let clips: [&Clip; 2] = [&first, &second];
        assert_ne!(
            first.frames().len(),
            second.frames().len(),
            "the two clips must fingerprint apart for the last pair to discriminate"
        );
        let variants = [
            // One segment, the shape every other variant is a step away from.
            Track::composed(&clips, 0, &[(0, 1.0, 0)]),
            // A second segment after the same first clip.
            Track::composed(&clips, 0, &[(0, 1.0, 0), (1, 1.0, 0)]),
            // The same one segment, held after.
            Track::composed(&clips, 0, &[(0, 1.0, 100)]),
            // The same one segment, at another speed.
            Track::composed(&clips, 0, &[(0, 2.0, 0)]),
            // The same one segment, behind a lead gap.
            Track::composed(&clips, 100, &[(0, 1.0, 0)]),
            // The same two-segment shape, differing in nothing but which clip
            // the second segment names — the collision a fold over the counts,
            // speeds and gaps alone would produce.
            Track::composed(&clips, 0, &[(0, 1.0, 0), (0, 1.0, 0)]),
        ];
        let fingerprints: std::collections::BTreeSet<u64> = variants
            .iter()
            .map(|variant| motion_fingerprint(&variant.view()))
            .collect();
        assert_eq!(
            fingerprints.len(),
            variants.len(),
            "two of the motions answer one fingerprint"
        );
    }

    /// The pick-up law over a composed motion: the seam, the hold and the lead
    /// gap are all derived from the clock, so a player picked up out of its row
    /// at each of them plays what one that was never put down plays.
    #[test]
    fn a_pick_up_holds_across_a_lead_gap_a_seam_and_a_hold() {
        // Distinct exit ramps, so a channel picked up mid-fade is judged
        // against the clip that actually drove it.
        let tail = antenna_clip("tail", &[0.1, 0.2, 0.3, 0.4, 0.5, 0.6], 40);
        let busy = busy_clip();
        assert_ne!(busy.blend_out_ms(), tail.blend_out_ms());
        let track = Track::composed(&[&busy, &tail], 40, &[(0, 1.0, 80), (1, 1.5, 0)]);
        the_resume_law_holds(&track, 1.0, Duration::ZERO);
        // The lead gap runs 0–40 ms, the first clip 40–440, its hold 440–520,
        // and the second clip — six frames at 1.5x — 520–600. Joined inside
        // each of the four, and on the seam between the last two.
        for join in [20, 200, 460, 520, 560] {
            the_resume_law_holds(&track, 1.0, Duration::from_millis(join));
        }
    }

    #[test]
    fn a_player_says_what_it_is_playing() {
        let clip = busy_clip();
        let track = Track::of(&clip);
        let mut row = Row::new();
        let player = row.play(track.view(), 1.0);
        assert_eq!(player.motion().segments(), 1);
        assert_eq!(
            player.motion().segment(0).clip.frames(),
            clip.frames().len()
        );
        assert_eq!(player.mask(), clip.mask());
        assert_eq!(player.speed(), 1.0);
    }
}
