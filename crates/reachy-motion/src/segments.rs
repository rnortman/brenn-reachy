//! Where a hand-moved head was still and where it was moving.
//!
//! A recording session leaves one file of present positions on a 20 ms grid and
//! nothing else: the servos are de-torqued, an operator's hands are in the
//! linkage, and there is no commanded value anywhere in the record. So
//! [`stillness`](crate::stillness) cannot cut this series — it cuts on goal
//! changes, and there are none. What cuts it is the series itself: a stretch
//! where every joint is standing still is a pose the operator was showing the
//! machine, and a stretch where something is travelling is a move they were
//! demonstrating.
//!
//! The shape is the same streaming watch as [`stillness`](crate::stillness) and
//! [`phase`](crate::phase): samples go in one at a time, the state is
//! fixed-size and caller-allocated, nothing allocates per sample and nothing
//! reads a clock. One consequence is the point of it — the recorder runs this
//! live to tell the operator that a pose registered, and the offline analyzer
//! runs the same struct over the same samples to produce the segments a clip is
//! cut from, so the two cannot disagree about where a hold began.
//!
//! **Activity** is what the cut is made on: the largest per-joint speed, each
//! joint's speed read as a difference quotient over
//! [`SegmentConfig::speed_window`] rather than between neighbouring samples.
//! The window is the whole trick. A hand-held joint carries two noise sources
//! that a one-sample difference cannot tell from motion — encoder flicker,
//! which is two counts and reads half a radian per second across 20 ms, and
//! hand tremor at 8–12 Hz, which reads the same — and a 100 ms baseline puts
//! the first below any useful threshold and spans about one tremor period, so
//! the second largely cancels.
//!
//! **Hysteresis** does the rest: the stream is moving from the first sample at
//! or above [`SegmentConfig::moving_rad_per_s`], and still only after
//! [`SegmentConfig::min_still`] of unbroken quiet below
//! [`SegmentConfig::still_rad_per_s`]. A quiet run shorter than that never
//! becomes a segment; its samples belong to the motion around it, because a
//! hand pausing mid-gesture is part of the gesture.
//!
//! **Segments tile the stream.** Each one's `t1_ns` is the next one's `t0_ns`
//! and both are sample stamps. A still segment starts at the *first* sample of
//! the quiet run that produced it and not at the sample where the run got long
//! enough, so a held pose's mean does not include the tail of the move that
//! arrived at it; the bookkeeping that buys this is two accumulator sets rather
//! than a buffer of samples, one for the open motion segment and one for the
//! quiet run that may or may not become a still segment. Nothing is buffered
//! and nothing is reassigned after the fact.
//!
//! Both boundaries lag the physical event, by a bounded amount and in one
//! direction, and neither is back-dated:
//!
//! - The onset lands late by `speed_window × moving_rad_per_s / slope` — 18 ms
//!   on a 0.83 rad/s ramp, `speed_window` in the limit — because the difference
//!   quotient rises over the window. The still segment before it keeps that
//!   lead-in: at most `speed_window × moving_rad_per_s`, 0.015 rad of
//!   sub-threshold drift folded into a mean over at least `min_still`.
//! - The quiet run opens late by `speed_window × (1 − still_rad_per_s /
//!   slope)` — 94 ms on the same ramp — because the quotient decays over the
//!   window after the joint stops. The motion segment keeps that settled tail,
//!   which dilutes its duration and its rms speed by at most one window and
//!   leaves its path untouched.
//!
//! Nothing here knows any kinematics. Head displacement, head speed and pose
//! are the analyzer's, computed from the per-sample forward-kinematics solve
//! over the segments this produces; the recorder does not run forward
//! kinematics at all.

use core::fmt;
use core::time::Duration;

use brenn_reachy__motion__joints_clk_rs::JointFlags;

use crate::joints::{JointVector, ROW_COUNT, ROWS, flags};
use crate::stillness::Wobble;

/// The default difference baseline: 100 ms.
///
/// Sized against the two noise sources a hand-held joint carries. Encoder
/// flicker of two counts ([`MAX_EXCURSION_RAD`](crate::stillness::MAX_EXCURSION_RAD))
/// is 0.03 rad/s over this baseline, below [`STILL_RAD_PER_S`]; hand tremor at
/// 8–12 Hz spans about one period, so a difference across the window largely
/// cancels it, where a 20 ms difference would read it as half a radian per
/// second of travel.
pub const SPEED_WINDOW: Duration = Duration::from_millis(100);

/// The default still threshold, rad/s: below this every joint is standing.
///
/// Above the encoder's own flicker across [`SPEED_WINDOW`] and well below any
/// speed a hand moving the head reaches.
pub const STILL_RAD_PER_S: f64 = 0.05;

/// The default moving threshold, rad/s: at or above this the stream is moving.
///
/// Three times the still threshold, so that the band between them absorbs the
/// decay of the difference quotient at either end of a move rather than
/// chattering across one line. A move whose peak speed never reaches this is
/// not a move under this configuration.
pub const MOVING_RAD_PER_S: f64 = 0.15;

/// The default shortest still segment: 500 ms.
///
/// Shorter than any pose an operator holds while saying what it is, and longer
/// than any pause inside a gesture worth keeping whole.
pub const MIN_STILL: Duration = Duration::from_millis(500);

/// How many samples of history the difference baseline is chosen from.
///
/// Fixed-size, because the crate's rule is that nothing allocates per sample.
/// The consequence is a requirement on the configuration: `speed_window` must
/// be no longer than this many sample periods, or no sample is ever old enough
/// to be a baseline and the stream reads as motionless. Thirty-two covers
/// 640 ms on the recorder's 20 ms grid against a 100 ms window.
pub const HISTORY_SAMPLES: usize = 32;

/// How many samples each end of the drift measurement averages over.
///
/// Ten samples is 200 ms on the recorder's grid: long enough that encoder
/// flicker averages out of both ends, short enough that a slow slide under a
/// held head shows up as the difference between them.
pub const DRIFT_SAMPLES: usize = 10;

/// What counts as still, and how long still has to last.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SegmentConfig {
    /// The baseline every joint's speed is a difference quotient over.
    pub speed_window: Duration,
    /// Below this activity, every joint is standing still, rad/s.
    pub still_rad_per_s: f64,
    /// At or above this activity, the stream is moving, rad/s.
    pub moving_rad_per_s: f64,
    /// Shortest quiet run that becomes a still segment. A shorter one belongs
    /// to the motion around it.
    pub min_still: Duration,
}

/// A configuration no segmentation can run under.
///
/// A violated precondition is a typed refusal rather than a confident wrong
/// answer: a window longer than the history is a stream that reads as
/// motionless, a window no longer than the grid is a stream that reads its own
/// encoder noise as travel, and an inverted hysteresis band is a stream that
/// can enter motion and never leave it. Each produces a plausible document,
/// which is worse than no document.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum SegmentConfigError {
    /// The difference baseline is longer than the history holds at this grid,
    /// so no reading is ever old enough to be a baseline.
    WindowTooLong {
        /// What was asked for.
        speed_window: Duration,
        /// The longest window [`HISTORY_SAMPLES`] readings cover at this grid.
        longest: Duration,
        /// The grid the samples arrive on.
        sample_period: Duration,
    },
    /// The still threshold is above the moving threshold: the band between them
    /// is inverted and there is no hysteresis to cross.
    InvertedHysteresis {
        /// What was asked for, rad/s.
        still_rad_per_s: f64,
        /// What was asked for, rad/s.
        moving_rad_per_s: f64,
    },
    /// The grid is not a positive span, so no window covers any readings.
    SamplePeriodNotPositive {
        /// What was asked for.
        sample_period: Duration,
    },
    /// The difference baseline spans one sample or none, so every speed is a
    /// one-sample difference: the reading that turns encoder flicker into
    /// travel and hand tremor into half a radian per second of it.
    WindowTooShort {
        /// What was asked for.
        speed_window: Duration,
        /// The grid the samples arrive on. A baseline has to be longer than
        /// this to span more than one step of it.
        sample_period: Duration,
    },
    /// A threshold is not a finite, positive number of radians per second.
    ThresholdNotFinite {
        /// What was asked for, rad/s.
        still_rad_per_s: f64,
        /// What was asked for, rad/s.
        moving_rad_per_s: f64,
    },
}

impl fmt::Display for SegmentConfigError {
    fn fmt(&self, out: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::WindowTooLong {
                speed_window,
                longest,
                sample_period,
            } => write!(
                out,
                "a {} ms difference baseline over a {} ms grid needs more than the \
                 {HISTORY_SAMPLES} readings of history there are: at most {} ms",
                speed_window.as_millis(),
                sample_period.as_millis(),
                longest.as_millis()
            ),
            Self::InvertedHysteresis {
                still_rad_per_s,
                moving_rad_per_s,
            } => write!(
                out,
                "a still threshold of {still_rad_per_s} rad/s above the moving threshold of \
                 {moving_rad_per_s} rad/s inverts the hysteresis band"
            ),
            Self::SamplePeriodNotPositive { sample_period } => write!(
                out,
                "a sample grid of {} ns is not a span readings arrive on",
                sample_period.as_nanos()
            ),
            Self::WindowTooShort {
                speed_window,
                sample_period,
            } => write!(
                out,
                "a {} ms difference baseline over a {} ms grid spans one reading or none, which \
                 reads encoder flicker as travel: it must be longer than the grid",
                speed_window.as_millis(),
                sample_period.as_millis()
            ),
            Self::ThresholdNotFinite {
                still_rad_per_s,
                moving_rad_per_s,
            } => write!(
                out,
                "thresholds must be finite and positive, not {still_rad_per_s} and \
                 {moving_rad_per_s} rad/s"
            ),
        }
    }
}

impl core::error::Error for SegmentConfigError {}

impl SegmentConfig {
    /// The longest difference baseline this crate's history covers on a grid of
    /// `sample_period`.
    #[must_use]
    pub fn longest_window(sample_period: Duration) -> Duration {
        sample_period.saturating_mul(u32::try_from(HISTORY_SAMPLES).unwrap_or(u32::MAX) - 1)
    }

    /// Whether a segmentation can run under this configuration on a grid of
    /// `sample_period`.
    ///
    /// Checked here rather than in each caller's argument parsing: the live
    /// recorder, the offline pass and anything written next all cut on the same
    /// arithmetic, and an invariant enforced in one command line is an invariant
    /// the next caller re-litigates or forgets.
    ///
    /// # Errors
    ///
    /// [`SegmentConfigError`] naming which invariant the values break.
    pub fn check(&self, sample_period: Duration) -> Result<(), SegmentConfigError> {
        if sample_period.is_zero() {
            return Err(SegmentConfigError::SamplePeriodNotPositive { sample_period });
        }
        if !self.still_rad_per_s.is_finite()
            || !self.moving_rad_per_s.is_finite()
            || self.still_rad_per_s <= 0.0
            || self.moving_rad_per_s <= 0.0
        {
            return Err(SegmentConfigError::ThresholdNotFinite {
                still_rad_per_s: self.still_rad_per_s,
                moving_rad_per_s: self.moving_rad_per_s,
            });
        }
        if self.still_rad_per_s > self.moving_rad_per_s {
            return Err(SegmentConfigError::InvertedHysteresis {
                still_rad_per_s: self.still_rad_per_s,
                moving_rad_per_s: self.moving_rad_per_s,
            });
        }
        if self.speed_window <= sample_period {
            return Err(SegmentConfigError::WindowTooShort {
                speed_window: self.speed_window,
                sample_period,
            });
        }
        let longest = Self::longest_window(sample_period);
        if self.speed_window > longest {
            return Err(SegmentConfigError::WindowTooLong {
                speed_window: self.speed_window,
                longest,
                sample_period,
            });
        }
        Ok(())
    }
}

impl Default for SegmentConfig {
    fn default() -> Self {
        Self {
            speed_window: SPEED_WINDOW,
            still_rad_per_s: STILL_RAD_PER_S,
            moving_rad_per_s: MOVING_RAD_PER_S,
            min_still: MIN_STILL,
        }
    }
}

/// One reading of every joint, as the recorder took it.
///
/// No setpoint and no validity flag: a recording session commands nothing, and
/// which rows answered is [`Self::missing`]. A reading that is not a finite
/// number is treated as a row that did not answer.
#[derive(Clone, Copy, Debug)]
pub struct JointSample<'a> {
    /// When the reading was taken, on the recording's own clock.
    pub t_ns: i64,
    /// Measured positions, radians.
    pub present: &'a JointVector,
    /// The rows that did not answer.
    pub missing: JointFlags,
}

/// Which kind of stretch a segment is.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SegmentKind {
    /// Every joint standing still: a pose.
    Still,
    /// Something travelling: a move.
    Motion,
}

/// One stretch of the stream, with what every joint did over it.
///
/// Every figure is computed for both kinds, because the arithmetic is the same
/// arithmetic and a kind-dependent struct would make a report reach for an
/// `Option` it never wanted. Which figures mean something is the reader's:
/// `mean`, `excursion` and `drift` describe a pose, `path`, `peak_speed` and
/// `rms_speed` describe a move.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Segment {
    /// Still or moving.
    pub kind: SegmentKind,
    /// The stamp of the first sample in the segment.
    pub t0_ns: i64,
    /// The stamp of the next segment's first sample, or of this segment's last
    /// sample at the end of the stream.
    pub t1_ns: i64,
    /// How many readings fell in it.
    pub samples: usize,
    /// Mean angle per joint over the readings that answered, radians.
    pub mean: JointVector,
    /// Peak-to-peak spread per joint, radians.
    pub excursion: JointVector,
    /// Mean of the last [`DRIFT_SAMPLES`] readings minus the mean of the first
    /// [`DRIFT_SAMPLES`], per joint, radians: a slow slide under a held head.
    pub drift: JointVector,
    /// Sum of the absolute per-sample differences, per joint, radians: how far
    /// the joint travelled rather than how far it ended up.
    pub path: JointVector,
    /// Highest activity seen in the segment, rad/s.
    pub peak_speed: f64,
    /// Root mean square of the activity over the segment, rad/s.
    pub rms_speed: f64,
}

/// One joint's share of an accumulator set.
///
/// The spread of a joint's readings is [`Wobble`]'s reading of them, not a
/// second one: an instrument's stillness report and a session document's
/// `joints_excursion_rad` are compared against each other, so "peak-to-peak
/// spread of this joint" has one definition in this crate. What is added here
/// is what a segment needs and a held-joint reader does not: the sum a mean is
/// taken from, and the distance travelled rather than the distance ended up.
#[derive(Clone, Copy, Debug, Default)]
struct JointAccum {
    /// The readings themselves: count, extremes, and the turns in them.
    wobble: Wobble,
    sum: f64,
    path: f64,
}

impl JointAccum {
    fn take(&mut self, value: f64) {
        self.wobble.take(value);
        self.sum += value;
    }

    fn absorb(&mut self, other: &Self) {
        self.wobble.absorb(&other.wobble);
        self.sum += other.sum;
        self.path += other.path;
    }

    fn mean(&self) -> f64 {
        if self.wobble.samples() == 0 {
            0.0
        } else {
            self.sum / self.wobble.samples() as f64
        }
    }

    fn excursion(&self) -> f64 {
        self.wobble.excursion()
    }
}

/// The last `N` readings taken, oldest overwritten first.
///
/// Two windows in this module are this: the [`DRIFT_SAMPLES`] readings each end
/// of a segment is averaged over, and the [`HISTORY_SAMPLES`] a difference
/// baseline is chosen from. One declaration, because the modular index
/// arithmetic is the only error-prone code here and a second copy of it is a
/// wrapping bug to find twice.
#[derive(Clone, Copy, Debug)]
struct Ring<const N: usize> {
    t_ns: [i64; N],
    rows: [[f64; ROW_COUNT]; N],
    present: [[bool; ROW_COUNT]; N],
    len: usize,
    next: usize,
}

impl<const N: usize> Default for Ring<N> {
    fn default() -> Self {
        Self {
            t_ns: [0; N],
            rows: [[0.0; ROW_COUNT]; N],
            present: [[false; ROW_COUNT]; N],
            len: 0,
            next: 0,
        }
    }
}

impl<const N: usize> Ring<N> {
    fn is_full(&self) -> bool {
        self.len == N
    }

    fn take(&mut self, t_ns: i64, rows: &[f64; ROW_COUNT], present: &[bool; ROW_COUNT]) {
        self.t_ns[self.next] = t_ns;
        self.rows[self.next] = *rows;
        self.present[self.next] = *present;
        self.next = (self.next + 1) % N;
        self.len = (self.len + 1).min(N);
    }

    /// Oldest first, which is the order a merge has to replay them in.
    fn entries(&self) -> impl Iterator<Item = (i64, &[f64; ROW_COUNT], &[bool; ROW_COUNT])> {
        let oldest = if self.is_full() { self.next } else { 0 };
        (0..self.len).map(move |i| {
            let index = (oldest + i) % N;
            (self.t_ns[index], &self.rows[index], &self.present[index])
        })
    }

    /// Per-joint mean over the readings that answered, and how many did.
    fn mean(&self) -> ([f64; ROW_COUNT], [usize; ROW_COUNT]) {
        let mut sums = [0.0; ROW_COUNT];
        let mut counts = [0_usize; ROW_COUNT];
        for (_, rows, present) in self.entries() {
            for (row, (value, answered)) in rows.iter().zip(present.iter()).enumerate() {
                if *answered {
                    sums[row] += *value;
                    counts[row] += 1;
                }
            }
        }
        for (sum, count) in sums.iter_mut().zip(counts.iter()) {
            if *count > 0 {
                *sum /= *count as f64;
            }
        }
        (sums, counts)
    }

    /// The newest reading at or before `cutoff_ns` in which `row` answered.
    ///
    /// `None` while no reading is old enough, which is the whole of the first
    /// window of a stream and any stretch a row has been silent through: a
    /// joint with no baseline has no measured speed, and reading one over a
    /// shorter span instead would turn encoder flicker into a move. The stream
    /// therefore starts out reading as quiet, which is what it is until there
    /// is evidence otherwise.
    fn baseline(&self, row: usize, cutoff_ns: i64) -> Option<(i64, f64)> {
        (0..self.len)
            .map(|i| (self.next + N - 1 - i) % N)
            .find(|&index| self.present[index][row] && self.t_ns[index] <= cutoff_ns)
            .map(|index| (self.t_ns[index], self.rows[index][row]))
    }
}

/// Everything one candidate stretch has accumulated.
///
/// Two of these exist at a time — the open motion segment's and the quiet run
/// that may become a still segment — and the whole point of the pair is that
/// they merge: a quiet run that never got long enough is folded into the motion
/// around it without any sample having been kept.
#[derive(Clone, Copy, Debug, Default)]
struct Accum {
    samples: usize,
    t0_ns: i64,
    t1_ns: i64,
    joints: [JointAccum; ROW_COUNT],
    peak: f64,
    activity_sq: f64,
    /// The leading window: it stops taking once full.
    first: Ring<DRIFT_SAMPLES>,
    /// The trailing window: it overwrites its oldest.
    last: Ring<DRIFT_SAMPLES>,
}

impl Accum {
    fn take(
        &mut self,
        t_ns: i64,
        rows: &[f64; ROW_COUNT],
        present: &[bool; ROW_COUNT],
        deltas: &[f64; ROW_COUNT],
        activity: f64,
    ) {
        if self.samples == 0 {
            self.t0_ns = t_ns;
        }
        self.samples += 1;
        self.t1_ns = t_ns;
        for (row, joint) in self.joints.iter_mut().enumerate() {
            if present[row] {
                joint.take(rows[row]);
            }
            joint.path += deltas[row];
        }
        self.peak = self.peak.max(activity);
        self.activity_sq += activity * activity;
        if !self.first.is_full() {
            self.first.take(t_ns, rows, present);
        }
        self.last.take(t_ns, rows, present);
    }

    /// Fold a later set into this one.
    ///
    /// Sums add, extremes combine, the peak is the larger. The leading ring
    /// stays this set's and is topped up from the later set's leading ring if
    /// it was short of a full window; the trailing ring takes the later set's
    /// readings over this one's. Both ends therefore read the merged stretch's
    /// own first and last 200 ms rather than either half's.
    fn absorb(&mut self, other: &Self) {
        if other.samples == 0 {
            return;
        }
        if self.samples == 0 {
            *self = *other;
            return;
        }
        self.samples += other.samples;
        self.t1_ns = other.t1_ns;
        for (joint, incoming) in self.joints.iter_mut().zip(other.joints.iter()) {
            joint.absorb(incoming);
        }
        self.peak = self.peak.max(other.peak);
        self.activity_sq += other.activity_sq;
        for (t_ns, rows, present) in other.first.entries() {
            if self.first.is_full() {
                break;
            }
            self.first.take(t_ns, rows, present);
        }
        for (t_ns, rows, present) in other.last.entries() {
            self.last.take(t_ns, rows, present);
        }
    }

    fn into_segment(self, kind: SegmentKind, t1_ns: i64) -> Segment {
        let (first_mean, first_counts) = self.first.mean();
        let (last_mean, last_counts) = self.last.mean();
        let mut mean = JointVector::default();
        let mut excursion = JointVector::default();
        let mut drift = JointVector::default();
        let mut path = JointVector::default();
        for (row, joint) in ROWS.into_iter().enumerate() {
            mean.set(joint, self.joints[row].mean());
            excursion.set(joint, self.joints[row].excursion());
            path.set(joint, self.joints[row].path);
            if first_counts[row] > 0 && last_counts[row] > 0 {
                drift.set(joint, last_mean[row] - first_mean[row]);
            }
        }
        let rms_speed = if self.samples == 0 {
            0.0
        } else {
            (self.activity_sq / self.samples as f64).sqrt()
        };
        Segment {
            kind,
            t0_ns: self.t0_ns,
            t1_ns,
            samples: self.samples,
            mean,
            excursion,
            drift,
            path,
            peak_speed: self.peak,
            rms_speed,
        }
    }
}

/// Where a difference baseline comes from: the last [`HISTORY_SAMPLES`]
/// readings.
type History = Ring<HISTORY_SAMPLES>;

/// Cuts a stream of readings into still and moving stretches.
///
/// Fed one sample at a time by [`Self::look`]; segments come out as they close.
/// [`Self::finish`] closes the last one at the end of a stream.
#[derive(Clone, Debug)]
pub struct MotionSegmenter {
    cfg: SegmentConfig,
    window_ns: i64,
    min_still_ns: i64,
    state: SegmentKind,
    /// The open segment of the current state.
    open: Accum,
    /// The quiet run under way, if any: still `None`-shaped as an empty accum
    /// with `candidate_open` false, because a `Option<Accum>` would move two
    /// kilobytes on every break.
    candidate: Accum,
    candidate_open: bool,
    history: History,
    /// The previous reading taken, for the per-joint path increment.
    previous: Option<([f64; ROW_COUNT], [bool; ROW_COUNT])>,
}

impl MotionSegmenter {
    /// A segmenter that has seen nothing yet.
    ///
    /// The initial state is [`SegmentKind::Motion`] with an empty accumulator,
    /// which is how a stream that starts mid-move and one that starts at rest
    /// come out right from the same code: a stream that opens quiet accumulates
    /// nothing into the motion set and promotes its first quiet run straight to
    /// a still segment, emitting no empty motion segment ahead of it.
    ///
    /// `sample_period` is the grid the readings will arrive on, which is what
    /// the configuration is checked against: the state below is fixed-size, so
    /// what the difference baseline may be is a function of the grid.
    ///
    /// # Errors
    ///
    /// [`SegmentConfigError`] if the configuration cannot be cut on at that
    /// grid, so a configuration nobody can measure under refuses instead of
    /// answering that nothing ever moved.
    pub fn new(cfg: SegmentConfig, sample_period: Duration) -> Result<Self, SegmentConfigError> {
        cfg.check(sample_period)?;
        Ok(Self {
            cfg,
            window_ns: duration_ns(cfg.speed_window),
            min_still_ns: duration_ns(cfg.min_still),
            state: SegmentKind::Motion,
            open: Accum::default(),
            candidate: Accum::default(),
            candidate_open: false,
            history: History::default(),
            previous: None,
        })
    }

    /// The thresholds this pass is cutting on.
    #[must_use]
    pub fn config(&self) -> SegmentConfig {
        self.cfg
    }

    /// What the stream is doing as of the last sample taken.
    #[must_use]
    pub fn state(&self) -> SegmentKind {
        self.state
    }

    /// Take in one reading. Any segment that closes on it is pushed to `out`.
    ///
    /// A reading in which no row answered is not a sample: it advances
    /// nothing, because a gap is the caller's to report and a difference
    /// quotient across it would be an invention.
    pub fn look(&mut self, sample: &JointSample, out: &mut Vec<Segment>) {
        let (rows, present) = split(sample);
        if !present.iter().any(|answered| *answered) {
            return;
        }
        let activity = self.activity(sample.t_ns, &rows, &present);
        let deltas = self.deltas(&rows, &present);
        match self.state {
            SegmentKind::Motion => {
                if self.candidate_open {
                    if activity < self.cfg.still_rad_per_s {
                        self.candidate
                            .take(sample.t_ns, &rows, &present, &deltas, activity);
                        if sample.t_ns.saturating_sub(self.candidate.t0_ns) >= self.min_still_ns {
                            self.promote(out);
                        }
                    } else {
                        // The run is broken: it belongs to the move around it.
                        let candidate = self.candidate;
                        self.open.absorb(&candidate);
                        self.candidate = Accum::default();
                        self.candidate_open = false;
                        self.open
                            .take(sample.t_ns, &rows, &present, &deltas, activity);
                    }
                } else if activity < self.cfg.still_rad_per_s {
                    self.candidate = Accum::default();
                    self.candidate_open = true;
                    self.candidate
                        .take(sample.t_ns, &rows, &present, &deltas, activity);
                    if self.min_still_ns == 0 {
                        self.promote(out);
                    }
                } else {
                    self.open
                        .take(sample.t_ns, &rows, &present, &deltas, activity);
                }
            }
            SegmentKind::Still => {
                if activity >= self.cfg.moving_rad_per_s {
                    let closing = self.open;
                    out.push(closing.into_segment(SegmentKind::Still, sample.t_ns));
                    self.open = Accum::default();
                    self.open
                        .take(sample.t_ns, &rows, &present, &deltas, activity);
                    self.state = SegmentKind::Motion;
                } else {
                    self.open
                        .take(sample.t_ns, &rows, &present, &deltas, activity);
                }
            }
        }
        self.history.take(sample.t_ns, &rows, &present);
        self.previous = Some((rows, present));
    }

    /// End of stream: close the open segment at the last sample's stamp.
    ///
    /// A recording simply stops, and the last stretch in it is as much a
    /// stretch as any other. A quiet run that never got long enough goes where
    /// a broken one goes: into the motion around it.
    pub fn finish(&mut self, out: &mut Vec<Segment>) {
        if self.candidate_open {
            let candidate = self.candidate;
            self.open.absorb(&candidate);
            self.candidate = Accum::default();
            self.candidate_open = false;
        }
        if self.open.samples == 0 {
            return;
        }
        let closing = self.open;
        self.open = Accum::default();
        out.push(closing.into_segment(self.state, closing.t1_ns));
    }

    /// The quiet run got long enough: it is a still segment, and it started
    /// where the run started rather than here.
    fn promote(&mut self, out: &mut Vec<Segment>) {
        if self.open.samples > 0 {
            let closing = self.open;
            out.push(closing.into_segment(SegmentKind::Motion, self.candidate.t0_ns));
        }
        self.open = self.candidate;
        self.candidate = Accum::default();
        self.candidate_open = false;
        self.state = SegmentKind::Still;
    }

    /// The largest per-joint speed at this reading, rad/s.
    fn activity(&self, t_ns: i64, rows: &[f64; ROW_COUNT], present: &[bool; ROW_COUNT]) -> f64 {
        let cutoff = t_ns.saturating_sub(self.window_ns);
        let mut peak = 0.0_f64;
        for (row, (value, answered)) in rows.iter().zip(present.iter()).enumerate() {
            if !*answered {
                continue;
            }
            let Some((then_ns, then)) = self.history.baseline(row, cutoff) else {
                continue;
            };
            let span_s = (t_ns.saturating_sub(then_ns)) as f64 * 1e-9;
            if span_s <= 0.0 {
                continue;
            }
            let speed = ((*value - then) / span_s).abs();
            if speed.is_finite() {
                peak = peak.max(speed);
            }
        }
        peak
    }

    /// How far each joint moved since the previous reading it answered
    /// alongside, radians.
    fn deltas(&self, rows: &[f64; ROW_COUNT], present: &[bool; ROW_COUNT]) -> [f64; ROW_COUNT] {
        let mut deltas = [0.0; ROW_COUNT];
        let Some((before, before_present)) = self.previous.as_ref() else {
            return deltas;
        };
        for (row, delta) in deltas.iter_mut().enumerate() {
            if present[row] && before_present[row] {
                *delta = (rows[row] - before[row]).abs();
            }
        }
        deltas
    }
}

/// A sample as row arrays: the angles, and which rows carry a usable one.
///
/// A row that did not answer and a row that answered something that is not a
/// number are the same thing to everything downstream, so the distinction is
/// dropped here rather than carried.
fn split(sample: &JointSample) -> ([f64; ROW_COUNT], [bool; ROW_COUNT]) {
    let mut rows = [0.0; ROW_COUNT];
    let mut present = [false; ROW_COUNT];
    for (row, (joint, value)) in sample.present.joints().into_iter().enumerate() {
        rows[row] = value;
        present[row] = value.is_finite() && !flags::contains(sample.missing, joint);
    }
    (rows, present)
}

/// A configured span in nanoseconds, saturating rather than wrapping on a
/// duration no clock will ever produce.
fn duration_ns(span: Duration) -> i64 {
    i64::try_from(span.as_nanos()).unwrap_or(i64::MAX)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::joints::JointRef;
    use crate::stillness::COUNT_RAD;

    /// The recorder's grid.
    const PERIOD_NS: i64 = 20_000_000;

    /// The same grid as a [`Duration`], which is what a configuration is
    /// checked against.
    const GRID: Duration = Duration::from_nanos(PERIOD_NS as u64);

    /// One generated reading: a stamp and one angle per row.
    struct Reading {
        t_ns: i64,
        rows: [f64; ROW_COUNT],
        missing: JointFlags,
    }

    /// A stream on the recorder's grid, one joint carrying the signal.
    ///
    /// `shape` is fed sample times in seconds from the start of the stream and
    /// answers with the angle the signal joint holds.
    fn stream(seconds: f64, shape: impl Fn(f64) -> f64) -> Vec<Reading> {
        let samples = (seconds * 50.0).round() as i64;
        (0..=samples)
            .map(|n| {
                let t_ns = n * PERIOD_NS;
                let mut rows = [0.0; ROW_COUNT];
                rows[signal_row()] = shape(n as f64 * 0.02);
                Reading {
                    t_ns,
                    rows,
                    missing: JointFlags::NONE,
                }
            })
            .collect()
    }

    /// The row the tests put the signal on: the first crank.
    fn signal_row() -> usize {
        1
    }

    fn signal_joint() -> JointRef {
        ROWS[signal_row()]
    }

    fn vector(rows: &[f64; ROW_COUNT]) -> JointVector {
        let mut vector = JointVector::default();
        for (row, joint) in ROWS.into_iter().enumerate() {
            vector.set(joint, rows[row]);
        }
        vector
    }

    /// Feed a whole stream and close it.
    fn segments(cfg: SegmentConfig, readings: &[Reading]) -> Vec<Segment> {
        let mut watch = MotionSegmenter::new(cfg, GRID).expect("a configuration the grid admits");
        let mut out = Vec::new();
        for reading in readings {
            let present = vector(&reading.rows);
            watch.look(
                &JointSample {
                    t_ns: reading.t_ns,
                    present: &present,
                    missing: reading.missing,
                },
                &mut out,
            );
        }
        watch.finish(&mut out);
        out
    }

    /// 3 s held, a 0.6 s ramp of 0.5 rad — slope 0.8333 rad/s — then 2 s held.
    fn ramp_stream() -> Vec<Reading> {
        stream(5.6, |t| {
            if t <= 3.0 {
                0.0
            } else if t <= 3.6 {
                (t - 3.0) * (0.5 / 0.6)
            } else {
                0.5
            }
        })
    }

    fn secs(t_ns: i64) -> f64 {
        t_ns as f64 * 1e-9
    }

    /// The activity of `[t0_ns, t1_ns)`, computed off the readings without the
    /// segmenter: the peak and the root mean square of the same difference
    /// quotient it cuts on.
    ///
    /// The speed figures a segment carries are otherwise only comparable
    /// against themselves, which is no comparison at all: a squared-versus-
    /// unsquared slip or a lost accumulator on a merge reads as a plausible
    /// number in a session document.
    fn activity_over(
        readings: &[Reading],
        cfg: SegmentConfig,
        t0_ns: i64,
        t1_ns: i64,
    ) -> (f64, f64) {
        let window_ns = duration_ns(cfg.speed_window);
        let mut peak = 0.0_f64;
        let mut squares = 0.0_f64;
        let mut samples = 0usize;
        for (index, reading) in readings.iter().enumerate() {
            if reading.t_ns < t0_ns || reading.t_ns >= t1_ns {
                continue;
            }
            // The newest reading at or before the cutoff, which on this grid is
            // a whole number of samples back.
            let cutoff = reading.t_ns - window_ns;
            let activity = readings[..index]
                .iter()
                .rev()
                .find(|before| before.t_ns <= cutoff)
                .map_or(0.0, |before| {
                    let span = secs(reading.t_ns - before.t_ns);
                    (reading.rows[signal_row()] - before.rows[signal_row()]).abs() / span
                });
            peak = peak.max(activity);
            squares += activity * activity;
            samples += 1;
        }
        let rms = if samples == 0 {
            0.0
        } else {
            (squares / samples as f64).sqrt()
        };
        (peak, rms)
    }

    #[test]
    fn a_ramp_between_two_holds_is_three_segments() {
        let cut = segments(SegmentConfig::default(), &ramp_stream());
        assert_eq!(cut.len(), 3, "{cut:#?}");
        assert_eq!(cut[0].kind, SegmentKind::Still);
        assert_eq!(cut[1].kind, SegmentKind::Motion);
        assert_eq!(cut[2].kind, SegmentKind::Still);

        // The onset lands one 100 ms window's worth of rise after the ramp
        // starts: speed_window * moving / slope = 18 ms, within a sample.
        let onset = secs(cut[1].t0_ns);
        assert!(
            (onset - 3.018).abs() <= 0.02,
            "onset at {onset}s, expected 3.018s within a sample"
        );
        assert_eq!(cut[0].t1_ns, cut[1].t0_ns, "segments tile");

        // The quiet run opens where the difference quotient has decayed below
        // the still threshold: speed_window * (1 - still / slope) = 94 ms after
        // the ramp ends, the fifth sample after it, within a sample.
        let settled = secs(cut[2].t0_ns);
        assert!(
            (settled - 3.694).abs() <= 0.02,
            "still opened at {settled}s, expected 3.694s within a sample"
        );
        assert_eq!(cut[1].t1_ns, cut[2].t0_ns, "segments tile");

        // The move's own 0.6 s plus that settled tail, less the lead-in the
        // hold before it kept.
        let duration = secs(cut[1].t1_ns - cut[1].t0_ns);
        assert!(
            (duration - 0.676).abs() <= 0.02,
            "motion ran {duration}s, expected 0.676s within a sample"
        );

        // The path is the ramp's height, less at most the sub-threshold
        // lead-in the still segment kept.
        let path = cut[1].path.get(signal_joint()).unwrap();
        assert!(
            (0.485..=0.5 + 1e-9).contains(&path),
            "motion path {path} rad, expected the ramp's 0.5 rad less the lead-in"
        );
        assert!(
            (cut[1].peak_speed - 0.5 / 0.6).abs() < 0.05 * (0.5 / 0.6),
            "peak {} rad/s, expected the slope within 5%",
            cut[1].peak_speed
        );

        // Both speed figures against the same difference quotient computed off
        // the readings: the peak is the slope, and the root mean square is the
        // slope diluted by the settled tail the move kept.
        let (peak, rms) = activity_over(
            &ramp_stream(),
            SegmentConfig::default(),
            cut[1].t0_ns,
            cut[1].t1_ns,
        );
        assert!(
            (cut[1].peak_speed - peak).abs() < 1e-9,
            "peak {} rad/s against {peak} rad/s over the same span",
            cut[1].peak_speed
        );
        assert!(
            (cut[1].rms_speed - rms).abs() < 1e-9,
            "rms {} rad/s against {rms} rad/s over the same span",
            cut[1].rms_speed
        );
        assert!(
            rms < 0.5 / 0.6 && rms > 0.5 * (0.5 / 0.6),
            "rms {rms} rad/s, expected the slope diluted by the tail"
        );

        // The hold before the move reads the held value despite the lead-in.
        let mean = cut[0].mean.get(signal_joint()).unwrap();
        assert!(mean.abs() < COUNT_RAD, "first hold's mean {mean} rad");
        let arrived = cut[2].mean.get(signal_joint()).unwrap();
        assert!(
            (arrived - 0.5).abs() < COUNT_RAD,
            "second hold's mean {arrived} rad, expected the ramp's end"
        );
    }

    #[test]
    fn a_still_segment_is_reported_a_min_still_after_it_began() {
        let readings = ramp_stream();
        let cfg = SegmentConfig::default();
        let mut watch = MotionSegmenter::new(cfg, GRID).expect("a configuration the grid admits");
        let mut out = Vec::new();
        let mut emitted_at = None;
        for reading in &readings {
            let present = vector(&reading.rows);
            let before = out.len();
            watch.look(
                &JointSample {
                    t_ns: reading.t_ns,
                    present: &present,
                    missing: reading.missing,
                },
                &mut out,
            );
            if out.len() > before && out.last().unwrap().kind == SegmentKind::Motion {
                emitted_at = Some(reading.t_ns);
            }
        }
        watch.finish(&mut out);
        let emitted_at = emitted_at.expect("the motion segment closed");
        let still = out
            .iter()
            .filter(|segment| segment.kind == SegmentKind::Still)
            .nth(1)
            .expect("a second hold");
        assert_eq!(
            emitted_at - still.t0_ns,
            duration_ns(cfg.min_still),
            "the hold's start is retroactive by exactly min_still"
        );
    }

    #[test]
    fn encoder_flicker_is_one_still_segment() {
        // A joint flickering between one count and its neighbour, ten seconds.
        let readings = stream(10.0, |t| {
            if (t / 0.02).round() as i64 % 2 == 0 {
                0.0
            } else {
                2.0 * COUNT_RAD
            }
        });
        let cut = segments(SegmentConfig::default(), &readings);
        assert_eq!(cut.len(), 1, "{cut:#?}");
        assert_eq!(cut[0].kind, SegmentKind::Still);
        let excursion = cut[0].excursion.get(signal_joint()).unwrap();
        assert!(
            (excursion - 2.0 * COUNT_RAD).abs() < 1e-12,
            "excursion {excursion} rad, expected two counts"
        );
        assert!(
            cut[0].peak_speed < STILL_RAD_PER_S,
            "flicker read {} rad/s across the window",
            cut[0].peak_speed
        );
    }

    #[test]
    fn hand_tremor_on_a_held_joint_is_still() {
        // 10 Hz, 0.01 rad: the difference baseline spans one period of it.
        let readings = stream(4.0, |t| {
            0.25 + 0.01 * (core::f64::consts::TAU * 10.0 * t).sin()
        });
        let cut = segments(SegmentConfig::default(), &readings);
        assert_eq!(cut.len(), 1, "{cut:#?}");
        assert_eq!(cut[0].kind, SegmentKind::Still);
        assert!(
            cut[0].peak_speed < STILL_RAD_PER_S,
            "tremor read {} rad/s across the window",
            cut[0].peak_speed
        );
    }

    /// Two 0.3 s ramps of 0.25 rad with a 0.3 s hold between them, held either
    /// side.
    fn paused_move_stream() -> Vec<Reading> {
        stream(3.4, |t| {
            let slope = 0.25 / 0.3;
            if t <= 1.0 {
                0.0
            } else if t <= 1.3 {
                (t - 1.0) * slope
            } else if t <= 1.6 {
                0.25
            } else if t <= 1.9 {
                0.25 + (t - 1.6) * slope
            } else {
                0.5
            }
        })
    }

    #[test]
    fn a_short_hold_inside_a_move_is_absorbed() {
        let readings = paused_move_stream();
        let cut = segments(SegmentConfig::default(), &readings);
        let moves: Vec<&Segment> = cut
            .iter()
            .filter(|segment| segment.kind == SegmentKind::Motion)
            .collect();
        assert_eq!(moves.len(), 1, "the pause was absorbed: {cut:#?}");
        let moved = moves[0];

        // The merge loses nothing: what the absorbed segment carries is what
        // the readings over its own span say, computed here without it.
        let span: Vec<&Reading> = readings
            .iter()
            .filter(|reading| reading.t_ns >= moved.t0_ns && reading.t_ns < moved.t1_ns)
            .collect();
        assert_eq!(moved.samples, span.len(), "every sample in the span");
        let mut path = 0.0_f64;
        let mut previous: Option<f64> = None;
        for reading in &readings {
            if let Some(before) = previous
                && reading.t_ns >= moved.t0_ns
                && reading.t_ns < moved.t1_ns
            {
                path += (reading.rows[signal_row()] - before).abs();
            }
            previous = Some(reading.rows[signal_row()]);
        }
        let carried = moved.path.get(signal_joint()).unwrap();
        assert!(
            (carried - path).abs() < 1e-12,
            "path {carried} rad, expected {path} rad over the same span"
        );

        // And the speed figures the merge pools — the peak and the summed
        // squares behind the root mean square — are the span's own.
        let (peak, rms) = activity_over(
            &readings,
            SegmentConfig::default(),
            moved.t0_ns,
            moved.t1_ns,
        );
        assert!(
            (moved.peak_speed - peak).abs() < 1e-9,
            "peak {} rad/s, expected {peak} rad/s over the same span",
            moved.peak_speed
        );
        assert!(
            (moved.rms_speed - rms).abs() < 1e-9,
            "rms {} rad/s, expected {rms} rad/s over the same span",
            moved.rms_speed
        );
        assert!(
            rms > 0.0 && peak > rms,
            "peak {peak} rad/s, rms {rms} rad/s"
        );
    }

    #[test]
    fn a_min_still_that_admits_the_pause_splits_the_move() {
        let readings = paused_move_stream();
        let cfg = SegmentConfig {
            min_still: Duration::from_millis(100),
            ..SegmentConfig::default()
        };
        let cut = segments(cfg, &readings);
        let moves = cut
            .iter()
            .filter(|segment| segment.kind == SegmentKind::Motion)
            .count();
        assert_eq!(moves, 2, "the pause became a hold: {cut:#?}");
    }

    #[test]
    fn a_row_missing_on_alternate_samples_segments_the_same() {
        let complete = segments(SegmentConfig::default(), &ramp_stream());
        let mut readings = ramp_stream();
        for (index, reading) in readings.iter_mut().enumerate() {
            if index % 2 == 1 {
                flags::insert(&mut reading.missing, signal_joint());
            }
        }
        let gappy = segments(SegmentConfig::default(), &readings);
        assert_eq!(gappy.len(), complete.len(), "{gappy:#?}");
        for (a, b) in gappy.iter().zip(complete.iter()) {
            assert_eq!(a.kind, b.kind);
            // The baseline a present sample finds is up to one period older
            // than the complete stream's, so a boundary may shift by that.
            assert!(
                (a.t0_ns - b.t0_ns).abs() <= 2 * PERIOD_NS,
                "boundary moved from {} to {}",
                secs(b.t0_ns),
                secs(a.t0_ns)
            );
        }
    }

    #[test]
    fn a_row_that_never_answers_is_not_a_joint() {
        let mut readings = ramp_stream();
        for reading in &mut readings {
            reading.rows[8] = f64::NAN;
        }
        let cut = segments(SegmentConfig::default(), &readings);
        assert_eq!(cut.len(), 3, "{cut:#?}");
        let mean = cut[0].mean.get(ROWS[8]).unwrap();
        assert!(
            mean.abs() < f64::EPSILON,
            "a silent row means nothing: {mean}"
        );
    }

    #[test]
    fn a_reading_no_row_answered_is_not_a_sample() {
        let mut readings = ramp_stream();
        let mut skipped = 0;
        for (index, reading) in readings.iter_mut().enumerate() {
            if index % 7 == 0 {
                reading.missing = flags::all();
                skipped += 1;
            }
        }
        let cut = segments(SegmentConfig::default(), &readings);
        let counted: usize = cut.iter().map(|segment| segment.samples).sum();
        assert_eq!(counted, readings.len() - skipped);
    }

    #[test]
    fn finish_closes_the_open_segment_at_the_last_sample() {
        let readings = ramp_stream();
        let cut = segments(SegmentConfig::default(), &readings);
        assert_eq!(
            cut.last().unwrap().t1_ns,
            readings.last().unwrap().t_ns,
            "the last hold runs to the last reading"
        );
        assert_eq!(cut.first().unwrap().t0_ns, readings.first().unwrap().t_ns);
    }

    #[test]
    fn finish_over_nothing_emits_nothing() {
        let mut watch = MotionSegmenter::new(SegmentConfig::default(), GRID)
            .expect("a configuration the grid admits");
        let mut out = Vec::new();
        watch.finish(&mut out);
        assert!(out.is_empty());
        assert_eq!(watch.state(), SegmentKind::Motion);
    }

    #[test]
    fn a_stream_that_opens_mid_move_is_a_move() {
        // No leading hold at all: the ramp starts at the first sample.
        let readings = stream(2.0, |t| if t <= 0.6 { t * (0.5 / 0.6) } else { 0.5 });
        let cut = segments(SegmentConfig::default(), &readings);
        assert_eq!(cut.len(), 2, "{cut:#?}");
        assert_eq!(cut[0].kind, SegmentKind::Motion);
        assert_eq!(cut[0].t0_ns, 0, "the move owns the first sample");
        assert_eq!(cut[1].kind, SegmentKind::Still);
    }

    #[test]
    fn a_held_pose_that_slides_reports_the_slide_as_drift() {
        // 0.002 rad/s: below the still threshold, four seconds of it.
        let readings = stream(4.0, |t| 0.002 * t);
        let cut = segments(SegmentConfig::default(), &readings);
        assert_eq!(cut.len(), 1);
        assert_eq!(cut[0].kind, SegmentKind::Still);
        let drift = cut[0].drift.get(signal_joint()).unwrap();
        // The first 200 ms average against the last 200 ms of a four-second
        // hold: 3.8 s of slide between the two window centres.
        assert!(
            (drift - 0.002 * 3.8).abs() < 1e-3,
            "drift {drift} rad over a 0.002 rad/s slide"
        );
        assert!(
            cut[0].excursion.get(signal_joint()).unwrap() > drift * 0.9,
            "the excursion covers the slide"
        );
    }

    #[test]
    fn the_state_says_what_the_stream_is_doing() {
        let readings = ramp_stream();
        let mut watch = MotionSegmenter::new(SegmentConfig::default(), GRID)
            .expect("a configuration the grid admits");
        let mut out = Vec::new();
        let mut seen = Vec::new();
        let mut last = watch.state();
        for reading in &readings {
            let present = vector(&reading.rows);
            watch.look(
                &JointSample {
                    t_ns: reading.t_ns,
                    present: &present,
                    missing: reading.missing,
                },
                &mut out,
            );
            if watch.state() != last {
                last = watch.state();
                seen.push((last, reading.t_ns));
            }
        }
        assert_eq!(
            seen.iter().map(|(kind, _)| *kind).collect::<Vec<_>>(),
            vec![SegmentKind::Still, SegmentKind::Motion, SegmentKind::Still]
        );
        // The state change lands min_still after the hold began.
        watch.finish(&mut out);
        let (_, said_at) = seen[2];
        let hold = out.last().expect("the last hold");
        assert_eq!(hold.kind, SegmentKind::Still);
        assert_eq!(said_at - hold.t0_ns, duration_ns(MIN_STILL));
    }
    #[test]
    fn a_baseline_longer_than_the_history_covers_is_refused() {
        // The shape that used to answer "nothing ever moved": every reading's
        // window reaches past the oldest one kept, so no joint has a baseline.
        let cfg = SegmentConfig {
            speed_window: Duration::from_millis(2000),
            ..SegmentConfig::default()
        };
        let refused = MotionSegmenter::new(cfg, GRID);
        assert!(
            matches!(refused, Err(SegmentConfigError::WindowTooLong { .. })),
            "{:?}",
            refused.map(|_| ())
        );
        // The same window is admitted on a grid coarse enough to cover it.
        assert!(cfg.check(Duration::from_millis(100)).is_ok());
        assert_eq!(
            SegmentConfig::longest_window(GRID),
            Duration::from_millis(620)
        );
    }

    #[test]
    fn an_inverted_hysteresis_band_and_an_unusable_grid_are_refused() {
        let inverted = SegmentConfig {
            still_rad_per_s: 0.9,
            moving_rad_per_s: 0.1,
            ..SegmentConfig::default()
        };
        assert!(matches!(
            inverted.check(GRID),
            Err(SegmentConfigError::InvertedHysteresis { .. })
        ));
        assert!(matches!(
            SegmentConfig::default().check(Duration::ZERO),
            Err(SegmentConfigError::SamplePeriodNotPositive { .. })
        ));
        let unmeasurable = SegmentConfig {
            still_rad_per_s: f64::NAN,
            ..SegmentConfig::default()
        };
        assert!(matches!(
            unmeasurable.check(GRID),
            Err(SegmentConfigError::ThresholdNotFinite { .. })
        ));
        assert!(SegmentConfig::default().check(GRID).is_ok());
    }

    #[test]
    fn a_baseline_no_longer_than_the_grid_is_refused() {
        // The other end of the tuning surface: a window of nothing, and one of
        // exactly a sample. Both leave every speed a one-sample difference,
        // which is the reading this module is built to avoid.
        for asked in [Duration::ZERO, GRID] {
            let cfg = SegmentConfig {
                speed_window: asked,
                ..SegmentConfig::default()
            };
            assert!(
                matches!(
                    cfg.check(GRID),
                    Err(SegmentConfigError::WindowTooShort { .. })
                ),
                "a {asked:?} baseline was admitted"
            );
        }
        // One sample longer than the grid is a baseline that spans two
        // readings, and it is admitted.
        let cfg = SegmentConfig {
            speed_window: GRID * 2,
            ..SegmentConfig::default()
        };
        assert!(cfg.check(GRID).is_ok());
    }

    #[test]
    fn a_floor_of_nothing_makes_every_quiet_sample_a_hold_and_no_empty_segment() {
        // `--min-still-ms 0` is a value the flags admit: a quiet run is a hold
        // from its first sample. What it must not do is emit a segment per
        // sample, or one of no length at all.
        let cfg = SegmentConfig {
            min_still: Duration::ZERO,
            ..SegmentConfig::default()
        };
        let readings = ramp_stream();
        let cut = segments(cfg, &readings);
        let kinds: Vec<SegmentKind> = cut.iter().map(|segment| segment.kind).collect();
        assert_eq!(
            kinds,
            vec![SegmentKind::Still, SegmentKind::Motion, SegmentKind::Still],
            "{cut:#?}"
        );
        for segment in &cut {
            assert!(segment.t1_ns > segment.t0_ns, "{segment:#?}");
            assert!(segment.samples > 0, "{segment:#?}");
        }
        // Every sample landed in exactly one segment, so nothing was emitted
        // twice and nothing was dropped.
        let counted: usize = cut.iter().map(|segment| segment.samples).sum();
        assert_eq!(counted, readings.len());
        // The hold after the move opens at the first sample below the still
        // threshold rather than min_still later, which is the whole difference
        // a floor of nothing makes.
        let with_floor = segments(SegmentConfig::default(), &readings);
        assert_eq!(cut[2].t0_ns, with_floor[2].t0_ns);
    }

    #[test]
    fn a_segments_excursion_is_the_held_joint_readers_own_figure() {
        // One definition of "peak-to-peak spread of this joint" in the crate:
        // a stillness instrument's figure and a session document's are the
        // same arithmetic over the same readings.
        let readings = stream(4.0, |t| 0.002 * t);
        let cut = segments(SegmentConfig::default(), &readings);
        assert_eq!(cut.len(), 1);
        let wobble = crate::stillness::Wobble::over(
            readings.iter().map(|reading| reading.rows[signal_row()]),
        );
        let measured = cut[0].excursion.get(signal_joint()).unwrap();
        assert!(
            (measured - wobble.excursion()).abs() < 1e-12,
            "{measured} against {}",
            wobble.excursion()
        );
    }
}
