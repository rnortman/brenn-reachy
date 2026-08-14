//! The player: one overlay's clock, its ramps, and the delta it contributes
//! this tick.
//!
//! A [`ClipPlayer`] owns a flattened motion, the speed it was invoked at, and
//! where in the motion it currently is. Each tick the caller advances it by the
//! elapsed period and gets back the deltas to hand the compositor, or the
//! terminal marker that says this overlay is finished and can be dropped.
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

use std::sync::Arc;
use std::time::Duration;

use reachy_motion::FLOOR_TICK_HZ;

use crate::compose::{ChannelWeights, OverlaySample, interpolate_pose, lerp};
use crate::format::{Channel, ChannelMask, Clip, DeltaFrame, PerChannel};
use crate::library::Motion;

/// The nominal control period, seconds: the most the motion clock may advance
/// in one call, before speed.
const NOMINAL_PERIOD_S: f64 = 1.0 / FLOOR_TICK_HZ;

/// How far short of a boundary the motion clock counts as being on it,
/// seconds.
///
/// The clock is a running sum of periods and the grid it meets — frame edges,
/// segment ends — is a multiple of the same period, so the two agree in exact
/// arithmetic and land a rounding error apart in binary. A nanosecond of
/// tolerance puts the clock back on the grid; without it a clip runs an extra
/// tick whenever the accumulated error happens to fall short.
const CLOCK_EPS_S: f64 = 1e-9;

/// How close to its target a blend weight has to get to be there.
///
/// Far below any weight difference the machine could feel, and far above the
/// rounding a few dozen accumulated ramp steps leave behind.
const RAMP_EPS: f64 = 1e-9;

/// Where the motion clock currently sits.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Position {
    /// Before the first clip, in the motion's leading hold. Nothing is driven
    /// yet: a leading gap holds the base alone.
    Lead,
    /// Inside a segment's clip, `local_s` seconds into it in the clip's own
    /// time.
    Playing {
        /// Which segment.
        index: usize,
        /// Elapsed within the clip, clip-native seconds.
        local_s: f64,
    },
    /// Inside the hold that follows a segment's clip, freezing its last delta.
    Holding {
        /// Which segment's hold.
        index: usize,
    },
    /// Past the final hold: the motion is over and its channels are fading.
    Ended,
}

/// One playing overlay.
///
/// Constructed at the moment the overlay starts, or — for a daemon joining a
/// script already in progress — at the offset the timeline says it should be
/// at. Either way its weights start at zero and ramp in, so a mid-motion join
/// eases onto the delta at that offset rather than stepping onto it.
#[derive(Clone, Debug)]
pub struct ClipPlayer {
    motion: Arc<Motion>,
    speed: f64,
    /// Elapsed in the motion's own timeline at 1.0× invocation speed, seconds.
    clock_s: f64,
    /// The segment the cursor last settled on, and where it starts on the
    /// motion clock. `index == segments.len()` means the motion is over.
    index: usize,
    segment_start_s: f64,
    /// The last delta each channel was given, held while that channel fades.
    frozen: DeltaFrame,
    weights: ChannelWeights,
    /// Each channel's fade-out ramp, from the clip that last drove it.
    blend_out_ms: PerChannel<u32>,
    /// The segment `frozen` was last taken from, if any.
    sampled: Option<usize>,
    /// Whether any tick has been taken yet.
    started: bool,
    finished: bool,
}

impl ClipPlayer {
    /// Start `motion` at its beginning, invoked at `speed`.
    ///
    /// # Panics
    ///
    /// If `speed` is not finite and positive; see [`ClipPlayer::joining_at`].
    #[must_use]
    pub fn new(motion: Arc<Motion>, speed: f64) -> Self {
        Self::joining_at(motion, speed, Duration::ZERO)
    }

    /// Start `motion` as though it had already been playing for `elapsed` of
    /// wall clock.
    ///
    /// What a daemon uses when a script it just accepted says an overlay
    /// started before now: the timeline stays authoritative in absolute time,
    /// so the overlay is picked up where it should be rather than replayed from
    /// the top. The weights still start at zero, because the delta at the join
    /// point can be the motion's largest excursion and stepping onto it is
    /// exactly what the ramps exist to prevent.
    ///
    /// # Panics
    ///
    /// If `speed` is not finite and positive — a defect in the calling path,
    /// not user input. Left to run, it is a player whose clock never advances:
    /// an overlay that holds one frame forever, occupying a slot until the
    /// script expires, reporting neither an error nor an end.
    #[must_use]
    pub fn joining_at(motion: Arc<Motion>, speed: f64, elapsed: Duration) -> Self {
        assert!(
            speed.is_finite() && speed > 0.0,
            "a player's speed must be finite and positive, not {speed}"
        );
        let mask = motion.mask();
        let mut player = Self {
            clock_s: elapsed.as_secs_f64() * speed,
            index: 0,
            segment_start_s: motion.lead_gap_s(),
            frozen: DeltaFrame::zero(mask),
            weights: ChannelWeights::zero(),
            blend_out_ms: PerChannel::new([0; Channel::COUNT]),
            sampled: None,
            started: false,
            finished: false,
            motion,
            speed,
        };
        player.settle();
        player
    }

    /// The motion being played.
    #[must_use]
    pub fn motion(&self) -> &Arc<Motion> {
        &self.motion
    }

    /// The invocation speed.
    #[must_use]
    pub fn speed(&self) -> f64 {
        self.speed
    }

    /// Every channel this motion can drive over its whole run.
    #[must_use]
    pub fn mask(&self) -> ChannelMask {
        self.motion.mask()
    }

    /// Whether the motion and its fade-out are both over.
    ///
    /// A finished player contributes nothing and is dropped by its owner.
    #[must_use]
    pub fn is_finished(&self) -> bool {
        self.finished
    }

    /// Advance by `elapsed` and return this tick's contribution, or `None` once
    /// the motion has ended and every channel has faded to zero.
    ///
    /// `elapsed` is the period since the previous call, capped at one nominal
    /// period before anything uses it: a late call advances the motion and the
    /// ramps by one period, not by the lateness. The first call takes no
    /// elapsed time at all — it reports the overlay at its start offset, with
    /// every weight still at zero — so the clip's own first frame is commanded
    /// rather than skipped past.
    pub fn advance(&mut self, elapsed: Duration) -> Option<OverlaySample> {
        if self.finished {
            return None;
        }
        let dt_s = if self.started {
            elapsed.as_secs_f64().min(NOMINAL_PERIOD_S)
        } else {
            self.started = true;
            0.0
        };
        self.clock_s += dt_s * self.speed;
        self.settle();

        let position = self.position();
        let live = self.live_mask(position);
        self.sample_live(position, live);
        self.ramp(dt_s, live);

        if position == Position::Ended && self.weights == ChannelWeights::zero() {
            self.finished = true;
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
            if !live.contains(channel) && self.weights.get(channel) <= 0.0 {
                continue;
            }
            match channel {
                Channel::Head => frame.head = self.frozen.head,
                Channel::BodyYaw => frame.body_yaw = self.frozen.body_yaw,
                Channel::Antennas => frame.antennas = self.frozen.antennas,
            }
        }
        Some(OverlaySample {
            frame,
            weights: self.weights,
        })
    }

    /// Walk the segment cursor forward to wherever the motion clock now is.
    ///
    /// Monotone, so this only ever moves forward, and a stalled loop that
    /// advances one period never skips a segment it should have played — the
    /// clock could not have jumped past one.
    fn settle(&mut self) {
        while self.index < self.motion.segments().len() {
            let segment = &self.motion.segments()[self.index];
            let span = segment.clip().duration_s() / segment.speed() + segment.gap_after_s();
            if self.clock_s + CLOCK_EPS_S < self.segment_start_s + span {
                return;
            }
            self.segment_start_s += span;
            self.index += 1;
        }
    }

    /// Where the clock sits relative to the settled segment.
    fn position(&self) -> Position {
        if self.index >= self.motion.segments().len() {
            return Position::Ended;
        }
        if self.clock_s + CLOCK_EPS_S < self.segment_start_s {
            return Position::Lead;
        }
        let segment = &self.motion.segments()[self.index];
        let motion_local = self.clock_s - self.segment_start_s;
        let play_span = segment.clip().duration_s() / segment.speed();
        if motion_local + CLOCK_EPS_S < play_span {
            Position::Playing {
                index: self.index,
                local_s: motion_local * segment.speed(),
            }
        } else {
            Position::Holding { index: self.index }
        }
    }

    /// The channels being driven right now.
    ///
    /// A hold drives exactly what the clip it follows drove — that is what
    /// makes it a hold of the delta rather than a return to the base. A leading
    /// gap and the end of the motion drive nothing.
    fn live_mask(&self, position: Position) -> ChannelMask {
        match position {
            Position::Lead | Position::Ended => ChannelMask::empty(),
            Position::Playing { index, .. } | Position::Holding { index } => {
                self.motion.segments()[index].clip().mask()
            }
        }
    }

    /// Refresh the frozen deltas for whatever is live, and record which ramp
    /// those channels will fade out on.
    ///
    /// A held channel keeps the delta it already has, which is the freeze. That
    /// freeze is reconstructible, not merely a residue of having played the
    /// frames: a player that joins a motion already inside a hold, or whose
    /// clock settles past a segment it never sampled, takes that clip's final
    /// frame directly. Otherwise a joiner would hold the zero delta — commanding
    /// the bare base where every player that started from the top is holding the
    /// clip's last pose — and would then step onto the next clip's opening frame
    /// with its weight already ramped to one.
    fn sample_live(&mut self, position: Position, live: ChannelMask) {
        let (sampled, blend_out_ms) = match position {
            Position::Playing { index, local_s } => {
                self.sampled = Some(index);
                let clip = self.motion.segments()[index].clip();
                (sample_clip(clip, local_s), clip.blend_out_ms())
            }
            Position::Holding { index } if self.sampled != Some(index) => {
                self.sampled = Some(index);
                let clip = self.motion.segments()[index].clip();
                let last = *clip.frames().last().expect("a clip has frames");
                (last, clip.blend_out_ms())
            }
            _ => return,
        };
        for channel in live.iter() {
            match channel {
                Channel::Head => self.frozen.head = sampled.head,
                Channel::BodyYaw => self.frozen.body_yaw = sampled.body_yaw,
                Channel::Antennas => self.frozen.antennas = sampled.antennas,
            }
            self.blend_out_ms.set(channel, blend_out_ms);
        }
    }

    /// Move each channel's weight toward where this tick wants it.
    ///
    /// Ramps run on the wall clock: `dt_s` is the real period, not the scaled
    /// one. A channel rising uses the incoming clip's entry ramp; a channel
    /// falling uses the ramp of whichever clip last drove it, since that is the
    /// delta it is fading out of.
    fn ramp(&mut self, dt_s: f64, live: ChannelMask) {
        for channel in Channel::ALL {
            let current = self.weights.get(channel);
            let target = if live.contains(channel) { 1.0 } else { 0.0 };
            if current == target {
                continue;
            }
            // The ramps are the clip's, already floored at load against its
            // own largest frame delta, so a ramp reaching here is one the step
            // bounds admit over a static base.
            let ms = if target > current {
                self.motion.segments()[self.index.min(self.motion.segments().len() - 1)]
                    .clip()
                    .blend_in_ms()
            } else {
                *self.blend_out_ms.get(channel)
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
            self.weights.set(channel, next);
        }
    }
}

/// The delta a clip carries `local_s` seconds into itself.
///
/// Frame index is arithmetic, never a search: the track is uniformly sampled at
/// the tick rate, so the index is the clock times that rate and the remainder
/// is the interpolation parameter. Past the last frame — the final period of
/// the clip, and any overshoot a speed change leaves — the last frame stands on
/// its own.
fn sample_clip(clip: &Clip, local_s: f64) -> DeltaFrame {
    let frames = clip.frames();
    // A clock a rounding error either side of a frame edge is on that edge, and
    // is snapped to it rather than nudged past it: biasing the position instead
    // would bias every interpolation between frames by the same amount.
    let raw = (local_s * FLOOR_TICK_HZ).max(0.0);
    let nearest = raw.round();
    let position = if (raw - nearest).abs() < CLOCK_EPS_S * FLOOR_TICK_HZ {
        nearest
    } else {
        raw
    };
    let index = position.floor() as usize;
    if index + 1 >= frames.len() {
        return frames[frames.len() - 1];
    }
    interpolate(&frames[index], &frames[index + 1], position - index as f64)
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

#[cfg(test)]
mod tests {
    use super::*;

    use nalgebra::{Isometry3, Translation3, UnitQuaternion, Vector3};

    use crate::ClipLimits;
    use crate::compose::compose;
    use reachy_kin::{LegAngles, inverse_kinematics};
    use reachy_motion::{
        ArmRecord, CommandDisposition, JointSet, JointStep, JointTargets, JointVector,
        MotionCommand, MotionConfig, MotionState, TickInputs, TickOutputs, motion_tick,
    };

    /// The bounds these fixtures load under: generous ones.
    ///
    /// What is under test here is the clock, the segment cursor and the weight
    /// ramps, so the fixtures use round numbers — a tenth of a radian a frame,
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
    use crate::library::Library;

    const TICK: Duration = Duration::from_millis(20);

    /// An antennas-only clip whose right antenna walks `values`.
    fn antenna_clip(name: &str, values: &[f64], blend_ms: u32) -> Arc<Clip> {
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
        Arc::new(Clip::from_doc(doc, &limits()).expect("clip is valid"))
    }

    /// A head-only clip whose translation walks `values` along z.
    fn head_clip(name: &str, values: &[f64], blend_ms: u32) -> Arc<Clip> {
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
        Arc::new(Clip::from_doc(doc, &limits()).expect("clip is valid"))
    }

    /// A head-only clip whose frames carry `rotations` and no translation.
    fn head_rotation_clip(name: &str, rotations: &[UnitQuaternion<f64>]) -> Arc<Clip> {
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
        Arc::new(Clip::from_doc(doc, &limits()).expect("clip is valid"))
    }

    /// A one-segment motion over `clip`, blend ramps as authored.
    fn motion_of(clip: Arc<Clip>) -> Arc<Motion> {
        let json = serde_json::to_string(&clip.to_doc()).expect("clip serialises");
        let library = Library::load([("test".to_owned(), json)], &limits()).0;
        library.motion(clip.name()).expect("clip loaded").clone()
    }

    /// A sequence motion over `documents`, resolved through the library so the
    /// flattening under test is the real one.
    fn sequence_motion(clips: &[Arc<Clip>], sequence_json: &str, name: &str) -> Arc<Motion> {
        let mut documents: Vec<(String, String)> = clips
            .iter()
            .map(|clip| {
                (
                    clip.name().to_owned(),
                    serde_json::to_string(&clip.to_doc()).expect("clip serialises"),
                )
            })
            .collect();
        documents.push(("sequence".to_owned(), sequence_json.to_owned()));
        let (library, skips) = Library::load(documents, &limits());
        assert!(skips.is_empty(), "library skipped assets: {skips:?}");
        library.motion(name).expect("sequence loaded").clone()
    }

    /// Advance a player until it terminates, collecting every sample.
    fn run(player: &mut ClipPlayer, limit: usize) -> Vec<OverlaySample> {
        let mut samples = Vec::new();
        for _ in 0..limit {
            match player.advance(TICK) {
                Some(sample) => samples.push(sample),
                None => return samples,
            }
        }
        panic!("player did not terminate within {limit} ticks");
    }

    #[test]
    fn a_clip_plays_its_frames_in_order() {
        // No blend, so the weight is full from the first tick and the samples
        // are the frames themselves.
        let motion = motion_of(antenna_clip("walk", &[0.0, 0.1, 0.2, 0.3], 0));
        let mut player = ClipPlayer::new(motion, 1.0);
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
        let motion = motion_of(antenna_clip("walk", &[0.0, 0.1, 0.2, 0.3], 0));
        let mut player = ClipPlayer::new(motion, 0.5);
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
        let motion = motion_of(head_clip("lift", &[0.0, 0.02], 0));
        let mut player = ClipPlayer::new(motion, 0.5);
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
        let motion = motion_of(antenna_clip("walk", &frames, 0));
        let mut fast = ClipPlayer::new(motion.clone(), 2.0);
        let mut slow = ClipPlayer::new(motion, 1.0);
        let fast_ticks = run(&mut fast, 100).len();
        let slow_ticks = run(&mut slow, 100).len();
        assert_eq!(slow_ticks, 20);
        assert_eq!(fast_ticks, 10);
    }

    #[test]
    fn lateness_advances_one_period_not_the_stall() {
        let frames: Vec<f64> = (0..50).map(|index| f64::from(index) * 0.01).collect();
        let motion = motion_of(antenna_clip("walk", &frames, 0));
        let mut player = ClipPlayer::new(motion, 1.0);
        let first = player.advance(Duration::from_secs(1)).expect("playing");
        assert_eq!(first.frame.antennas.expect("antennas driven")[0], 0.0);
        let second = player.advance(Duration::from_secs(1)).expect("playing");
        assert!((second.frame.antennas.expect("antennas driven")[0] - 0.01).abs() < 1e-12);
    }

    #[test]
    fn blend_in_ramps_the_weight_from_zero() {
        // 100 ms of ramp is five ticks, from the zero the first sample reports.
        let motion = motion_of(antenna_clip("walk", &[0.1; 20], 100));
        let mut player = ClipPlayer::new(motion, 1.0);
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
        let motion = motion_of(antenna_clip("walk", &[0.1; 40], 100));
        let mut fast = ClipPlayer::new(motion.clone(), 2.0);
        let mut slow = ClipPlayer::new(motion, 1.0);
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
        let motion = motion_of(antenna_clip("walk", &[0.1; 10], 100));
        let mut player = ClipPlayer::new(motion, 1.0);
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
        let motion = motion_of(antenna_clip("walk", &[0.0, 0.1, 0.7], 100));
        let mut player = ClipPlayer::new(motion, 1.0);
        let samples = run(&mut player, 40);
        for sample in &samples[3..] {
            assert!((sample.frame.antennas.expect("antennas driven")[0] - 0.7).abs() < 1e-12);
        }
    }

    #[test]
    fn a_gap_freezes_the_previous_delta_and_keeps_full_weight() {
        let clips = [
            antenna_clip("a", &[0.0, 0.4], 0),
            antenna_clip("b", &[0.9, 0.9], 0),
        ];
        let sequence = r#"{
            "version": 1, "kind": "sequence", "name": "seq",
            "entries": [{"ref": "a"}, {"gap_ms": 60}, {"ref": "b"}]
        }"#;
        let motion = sequence_motion(&clips, sequence, "seq");
        let mut player = ClipPlayer::new(motion, 1.0);
        let samples = run(&mut player, 40);
        let right: Vec<f64> = samples
            .iter()
            .map(|sample| sample.frame.antennas.expect("antennas driven")[0])
            .collect();
        assert_eq!(&right[..2], &[0.0, 0.4]);
        // Three ticks of hold at the frozen delta, then the next clip.
        assert_eq!(&right[2..5], &[0.4, 0.4, 0.4]);
        assert_eq!(right[5], 0.9);
        for sample in &samples[..7] {
            assert_eq!(sample.weights.get(Channel::Antennas), 1.0);
        }
    }

    #[test]
    fn a_leading_gap_contributes_nothing_until_the_first_clip() {
        let clips = [antenna_clip("a", &[0.5, 0.5], 0)];
        let sequence = r#"{
            "version": 1, "kind": "sequence", "name": "seq",
            "entries": [{"gap_ms": 40}, {"ref": "a"}]
        }"#;
        let motion = sequence_motion(&clips, sequence, "seq");
        let mut player = ClipPlayer::new(motion, 1.0);
        for _ in 0..2 {
            let sample = player.advance(TICK).expect("waiting");
            assert_eq!(sample.weights, ChannelWeights::zero());
            assert_eq!(sample.frame.antennas, None);
        }
        let sample = player.advance(TICK).expect("playing");
        assert_eq!(sample.frame.antennas.expect("antennas driven")[0], 0.5);
    }

    #[test]
    fn gaps_scale_with_the_invocation_speed() {
        let clips = [
            antenna_clip("a", &[0.4, 0.4], 0),
            antenna_clip("b", &[0.9, 0.9], 0),
        ];
        let sequence = r#"{
            "version": 1, "kind": "sequence", "name": "seq",
            "entries": [{"ref": "a"}, {"gap_ms": 80}, {"ref": "b"}]
        }"#;
        let motion = sequence_motion(&clips, sequence, "seq");
        let mut player = ClipPlayer::new(motion, 2.0);
        let samples = run(&mut player, 40);
        let right: Vec<f64> = samples
            .iter()
            .map(|sample| sample.frame.antennas.expect("antennas driven")[0])
            .collect();
        // At 2× the two-frame clips are one tick each and the 80 ms hold is two.
        assert_eq!(&right[..4], &[0.4, 0.4, 0.4, 0.9]);
    }

    #[test]
    fn a_channel_leaving_the_mask_fades_while_the_next_clip_plays() {
        let clips = [
            head_clip("h", &[0.01; 10], 100),
            antenna_clip("a", &[0.5; 10], 100),
        ];
        let sequence = r#"{
            "version": 1, "kind": "sequence", "name": "seq",
            "entries": [{"ref": "h"}, {"ref": "a"}]
        }"#;
        let motion = sequence_motion(&clips, sequence, "seq");
        let mut player = ClipPlayer::new(motion, 1.0);
        let samples = run(&mut player, 40);

        // The head clip runs for ten ticks; from the eleventh the antennas ramp
        // in while the head ramps out of the delta it was frozen at.
        let crossing = &samples[11];
        assert!((crossing.weights.get(Channel::Antennas) - 0.4).abs() < 1e-12);
        assert!((crossing.weights.get(Channel::Head) - 0.6).abs() < 1e-12);
        assert!(
            (crossing
                .frame
                .head
                .expect("head fading")
                .translation
                .vector
                .z
                - 0.01)
                .abs()
                < 1e-12
        );

        // Once faded, the head contributes nothing at all while the antennas
        // carry on.
        let late = &samples[16];
        assert_eq!(late.weights.get(Channel::Head), 0.0);
        assert_eq!(late.frame.head, None);
        assert!((late.weights.get(Channel::Antennas) - 1.0).abs() < 1e-12);
    }

    #[test]
    fn a_mid_motion_join_starts_at_the_right_offset_with_zero_weight() {
        let frames: Vec<f64> = (0..20).map(|index| f64::from(index) * 0.01).collect();
        let motion = motion_of(antenna_clip("walk", &frames, 100));
        let mut player = ClipPlayer::joining_at(motion, 1.0, Duration::from_millis(200));
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
        let motion = motion_of(antenna_clip("walk", &frames, 0));
        let mut player = ClipPlayer::joining_at(motion, 2.0, Duration::from_millis(200));
        // Two hundred milliseconds at 2× is twenty frames of clip, and each
        // subsequent tick is two more.
        let sample = player.advance(TICK).expect("playing");
        assert!((sample.frame.antennas.expect("antennas driven")[0] - 0.20).abs() < 1e-12);
        let next = player.advance(TICK).expect("playing");
        assert!((next.frame.antennas.expect("antennas driven")[0] - 0.22).abs() < 1e-12);
    }

    #[test]
    fn a_join_inside_a_gap_holds_the_clip_it_follows() {
        // Clip `a` is two frames — 40 ms — and the hold after it runs to
        // 240 ms, so a join at 100 ms lands inside that hold. Every player that
        // played `a` from the top is holding its final 0.4 there; the joiner
        // holds the same delta rather than the zero one nobody recorded.
        let clips = [
            antenna_clip("a", &[0.0, 0.4], 100),
            antenna_clip("b", &[0.9, 0.9], 100),
        ];
        let sequence = r#"{
            "version": 1, "kind": "sequence", "name": "seq",
            "entries": [{"ref": "a"}, {"gap_ms": 200}, {"ref": "b"}]
        }"#;
        let motion = sequence_motion(&clips, sequence, "seq");
        let mut player = ClipPlayer::joining_at(motion, 1.0, Duration::from_millis(100));

        let first = player.advance(TICK).expect("holding");
        assert!((first.frame.antennas.expect("antennas held")[0] - 0.4).abs() < 1e-12);
        assert_eq!(first.weights, ChannelWeights::zero());

        // The fresh blend-in ramps against that held delta, so it is spent
        // where it is meant to be rather than against zero.
        let second = player.advance(TICK).expect("holding");
        assert!((second.weights.get(Channel::Antennas) - 0.2).abs() < 1e-12);
        assert!((second.frame.antennas.expect("antennas held")[0] - 0.4).abs() < 1e-12);
    }

    #[test]
    fn a_join_inside_a_gap_lands_where_a_player_from_the_top_already_is() {
        let clips = [
            antenna_clip("a", &[0.0, 0.4], 100),
            antenna_clip("b", &[0.9; 10], 100),
        ];
        let sequence = r#"{
            "version": 1, "kind": "sequence", "name": "seq",
            "entries": [{"ref": "a"}, {"gap_ms": 200}, {"ref": "b"}]
        }"#;
        let motion = sequence_motion(&clips, sequence, "seq");
        let mut joined = ClipPlayer::joining_at(motion.clone(), 1.0, Duration::from_millis(100));
        let mut from_top = ClipPlayer::new(motion, 1.0);

        // The join is five ticks into the motion, so the joiner's first sample
        // is the sixth of a player that started at the top.
        const SKIP: usize = 5;
        let joined = run(&mut joined, 60);
        let top = run(&mut from_top, 60);
        assert_eq!(joined.len() + SKIP, top.len());

        for (index, sample) in joined.iter().enumerate() {
            assert_eq!(
                sample.frame.antennas,
                top[index + SKIP].frame.antennas,
                "tick {index}"
            );
        }

        // The joiner's own blend-in is what differs, and only until it is
        // spent: it starts from zero against the delta at the join point rather
        // than stepping onto it, and from there the two players are the same
        // overlay.
        assert_eq!(joined[0].weights, ChannelWeights::zero());
        for (index, sample) in joined.iter().enumerate().skip(SKIP) {
            assert_eq!(sample.weights, top[index + SKIP].weights, "tick {index}");
        }
    }

    #[test]
    fn a_join_inside_the_trailing_gap_still_fades_out_on_the_clips_ramp() {
        let clips = [antenna_clip("a", &[0.0, 0.4], 100)];
        let sequence = r#"{
            "version": 1, "kind": "sequence", "name": "seq",
            "entries": [{"ref": "a"}, {"gap_ms": 200}]
        }"#;
        let motion = sequence_motion(&clips, sequence, "seq");
        let mut player = ClipPlayer::joining_at(motion, 1.0, Duration::from_millis(100));
        let samples = run(&mut player, 40);

        // The last five samples fade over the clip's own 100 ms ramp; without
        // the freeze the join reconstructs, that ramp would be zero and the
        // weight would drop in a single tick.
        let weights: Vec<f64> = samples
            .iter()
            .map(|sample| sample.weights.get(Channel::Antennas))
            .collect();
        let tail = &weights[weights.len() - 4..];
        for (index, weight) in tail.iter().enumerate() {
            let expected = 0.8 - index as f64 * 0.2;
            assert!(
                (weight - expected).abs() < 1e-12,
                "fade {index}: {weights:?}"
            );
        }
    }

    #[test]
    fn a_trailing_gap_holds_the_frozen_delta_before_the_fade() {
        // Two frames of clip, then a 100 ms hold — five ticks — at the frozen
        // final delta, at full weight throughout because the ramps are zero.
        let clips = [antenna_clip("a", &[0.0, 0.6], 0)];
        let sequence = r#"{
            "version": 1, "kind": "sequence", "name": "seq",
            "entries": [{"ref": "a"}, {"gap_ms": 100}]
        }"#;
        let motion = sequence_motion(&clips, sequence, "seq");
        let mut player = ClipPlayer::new(motion, 1.0);
        let samples = run(&mut player, 40);
        assert_eq!(samples.len(), 7);
        for sample in &samples[1..] {
            assert!((sample.frame.antennas.expect("antennas driven")[0] - 0.6).abs() < 1e-12);
            assert_eq!(sample.weights.get(Channel::Antennas), 1.0);
        }
    }

    #[test]
    fn the_head_rotation_walks_the_geodesic_between_frames() {
        // Two frames rotating about different axes, played at half speed so the
        // sample between them is the half-way interpolation and not a frame.
        let first = UnitQuaternion::from_scaled_axis(Vector3::y() * 0.4);
        let second = first * UnitQuaternion::from_scaled_axis(Vector3::z() * 0.6);
        let clip = head_rotation_clip("turn", &[first, second]);
        let motion = motion_of(clip);
        let mut player = ClipPlayer::new(motion, 0.5);

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
    fn a_blend_longer_than_the_clip_plays_the_whole_track_at_no_weight() {
        // Pins today's behaviour rather than blessing it: nothing bounds a
        // configured ramp from above, so a ramp far longer than the clip is a
        // motion that is commanded and never appears. TODO(clip-blend-ceiling)
        let motion = motion_of(antenna_clip("walk", &[0.5; 10], 60_000));
        let mut player = ClipPlayer::new(motion, 1.0);
        let samples = run(&mut player, 200);
        for sample in &samples[..10] {
            assert!(sample.weights.get(Channel::Antennas) < 0.01);
        }
    }

    #[test]
    #[should_panic(expected = "finite and positive")]
    fn a_zero_speed_player_is_refused() {
        let motion = motion_of(antenna_clip("walk", &[0.1; 4], 0));
        let _ = ClipPlayer::new(motion, 0.0);
    }

    #[test]
    #[should_panic(expected = "finite and positive")]
    fn a_nan_speed_player_is_refused() {
        let motion = motion_of(antenna_clip("walk", &[0.1; 4], 0));
        let _ = ClipPlayer::joining_at(motion, f64::NAN, Duration::from_millis(100));
    }

    #[test]
    fn a_join_past_the_whole_motion_is_finished_immediately() {
        let motion = motion_of(antenna_clip("walk", &[0.1; 4], 0));
        let mut player = ClipPlayer::joining_at(motion, 1.0, Duration::from_secs(5));
        assert!(player.advance(TICK).is_none());
        assert!(player.is_finished());
    }

    /// A machine armed and holding the neutral pose, and the goals it holds.
    fn armed_at_neutral(cfg: &MotionConfig) -> (MotionState, JointVector) {
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
        (MotionState::new_armed(&record, JointSet::EMPTY), joints)
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
        let json = serde_json::to_string(&clip.to_doc()).expect("clip serialises");
        let library = Library::load([("test".to_owned(), json)], &shipped).0;
        let motion = library
            .motion("test/three-channels")
            .expect("the clip loaded")
            .clone();

        let speed = motion.max_speed();
        assert!(speed >= 1.0, "a clip plays at its own recorded speed");
        let (ticks, moved) = play_through_the_tick(&cfg, ClipPlayer::new(motion, speed));
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
        let json = serde_json::to_string(&clip.to_doc()).expect("clip serialises");
        let library = Library::load([("test".to_owned(), json)], &shipped).0;
        let motion = library
            .motion("test/far-end")
            .expect("the clip loaded")
            .clone();

        let speed = motion.max_speed();
        // A third of the way in, so the ramp is still climbing over a delta at
        // the track's largest while the clock keeps advancing under it.
        let joined = Duration::from_secs_f64(motion.duration_s_at(speed) / 3.0);
        let (ticks, moved) =
            play_through_the_tick(&cfg, ClipPlayer::joining_at(motion, speed, joined));
        assert!(moved, "the join commanded something");
        assert!(ticks > 5, "the join played and faded: {ticks} ticks");
    }

    /// Play a player out through the real tick over a static neutral base,
    /// asserting every period is taken. Answers how many periods ran and
    /// whether any of them put a goal on the wire.
    fn play_through_the_tick(cfg: &MotionConfig, mut player: ClipPlayer) -> (u32, bool) {
        let base = JointTargets::default();
        let (mut state, pinned) = armed_at_neutral(cfg);
        let mut present = pinned;
        let mut ticks = 0u32;
        let mut moved = false;
        while let Some(sample) = player.advance(TICK) {
            let composed = compose(base, &[sample]);
            let mut out = TickOutputs::default();
            motion_tick(
                cfg,
                &mut state,
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
}
