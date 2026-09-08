//! Whether a joint asked to hold one angle actually holds it.
//!
//! The driver publishes one sample per control cycle carrying every present
//! position and the setpoint it is holding. A joint whose setpoint has not
//! moved for a while is *supposed* to be standing still; whether it is doing so
//! is a property of the recorded series and not of anything this stack
//! commands. So this module is a reader: it takes those samples one at a time,
//! cuts the series into the stretches where each joint was asked to hold one
//! value, and reports what the joint did over each stretch.
//!
//! The shape is [`phase::PhaseWatch`](crate::phase::PhaseWatch)'s: a streaming
//! watch fed one sample at a time, holding fixed-size per-joint state and no
//! series at all. A live report over a running session and an offline pass over
//! a recording drive it identically, and a conversation of any length costs the
//! same handful of words per joint. This is the crate's rule — state in a
//! caller-allocated struct, nothing allocating per tick.
//!
//! Nothing here decides anything about the machine. [`judge`] compares one
//! window against a bound the caller supplies, and what a report does with that
//! is the report's opinion.
//!
//! What a window carries is four numbers, and only the first is judged:
//!
//! - the **excursion**, the peak-to-peak spread of the present position, which
//!   bounds how far the joint moved while it was meant to be still;
//! - the **reversal rate**, how often the direction of travel changed;
//! - the **reversal intervals**, the mean and spread of how many samples ran
//!   between one reversal and the next;
//! - the **mean error**, how far from the setpoint the joint sat on average.
//!
//! The last three are printed rather than judged. The reversal rate and the
//! mean error separate a joint hunting about its setpoint (many reversals,
//! error averaging near zero) from one sitting still at an offset (few
//! reversals, a standing error), which are different findings about the
//! mechanism even when the excursion is the same. The interval statistics
//! separate the two again along another axis: a regular oscillation turns round
//! on a near-constant interval and reads a spread small against its mean, while
//! encoder dither turns round as often but at intervals scattered from one
//! sample to several, and reads a spread comparable to its mean. The same
//! reversal rate is two mechanisms, and only the intervals tell them apart.
//!
//! **Nyquist caveat.** The series is the driver's 50 Hz read of the encoder, so
//! a limit cycle above 25 Hz arrives aliased: it appears at some lower
//! frequency, and the reversal rate is therefore a floor on how often the joint
//! turned round, not a measurement of it. The peak-to-peak excursion survives
//! aliasing — a sampled extreme is a real extreme — which is why the bound is
//! on amplitude and the reversal rate is not bounded at all. An apparent
//! frequency read off the intervals carries the same caveat and carries it
//! sharply: a series sampled at `r` Hz shows a component at `f` Hz as any of
//! `|r·k ± f|` for whole `k`, so a period of four samples at 50 Hz says 12.5 Hz
//! or 37.5 or 62.5 or 87.5, and this series cannot say which. What it does say
//! is that the turning is regular, and how regular; separating the candidates
//! takes a faster sampler than the driver's grid.

use core::time::Duration;

use brenn_reachy__motion__joints_clk_rs::JointFlags;
use reachy_driver::NOMINAL_CYCLE_NS;
use thiserror::Error;

use crate::joints::{JointRef, JointVector, Name, flags};
use crate::tick::COUNTS_PER_TURN;

/// How often the driver publishes a sample, taken from the grid the driver's
/// own budgets are sized against.
///
/// Used only as the default of [`StillnessConfig::period`], which says how many
/// valid readings a window of a given length ought to contain, so that a
/// stretch which is long in wall-clock time but mostly gaps is discarded rather
/// than judged on a handful of points. Derived rather than restated so a
/// machine built to run a different grid moves this with it.
pub const DRIVER_PERIOD: Duration = Duration::from_nanos(NOMINAL_CYCLE_NS.unsigned_abs());

/// One encoder count, in radians.
///
/// The unit the excursion is worth quoting in: a joint that is truly still
/// reads one count, or flickers between it and its neighbour.
pub const COUNT_RAD: f64 = core::f64::consts::TAU / COUNTS_PER_TURN;

/// The default excursion bound, radians: two encoder counts.
///
/// Deliberately the encoder's own flicker rather than a figure with slack in
/// it. Nothing on this machine has yet been measured holding still, so a
/// generous bound would be an invention; the tight one is an assertion written
/// to fail, and the failure — with its printed excursion and reversal rate — is
/// the measurement. Once a hold is confirmed still on hardware, the observed
/// figure replaces this one and cites the run it came from.
pub const MAX_EXCURSION_RAD: f64 = 2.0 * COUNT_RAD;

/// The default settle allowance: enough to cover a move as well as the coming
/// to rest after it.
///
/// The allowance runs from the last *commanded* change, and the command is not
/// the motion: the servo's own profile generator paces every move, so a joint
/// is still travelling for seconds after its setpoint stops changing. The
/// widest hold this stack judges follows the antennas' arc from stow to rest,
/// about 2.9 rad at the commissioned profile velocity — near two seconds of
/// travel after the command has settled — and the rod then rings down. Four
/// seconds covers both. A shorter allowance judges the tail of the move and
/// reads it as a joint that will not stand still.
const SETTLE: Duration = Duration::from_secs(4);

/// The default shortest judged hold.
const MIN_HOLD: Duration = Duration::from_secs(2);

/// What counts as a hold, and how still a held joint has to be.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct StillnessConfig {
    /// Settle allowance after the last goal change before a hold is judged.
    ///
    /// The servo's own profile generator paces every move, and a joint that has
    /// just arrived is still coming to rest; the head of each stretch is
    /// dropped rather than counted against the joint.
    pub settle: Duration,
    /// Shortest judged hold after settling. A stretch shorter than this says
    /// nothing and is counted rather than reported.
    pub min_hold: Duration,
    /// Widest peak-to-peak excursion a still joint may show, radians.
    pub max_excursion_rad: f64,
    /// How often the recording this is fed from sampled the machine.
    ///
    /// The caller knows what grid its recording is on; the watch uses it only
    /// to say how many readings a judged window ought to contain, and so to
    /// discard a stretch that is long but mostly gaps. A period longer than the
    /// recording's own makes that floor stricter, never a window shorter.
    pub period: Duration,
}

impl Default for StillnessConfig {
    fn default() -> Self {
        Self {
            settle: SETTLE,
            min_hold: MIN_HOLD,
            max_excursion_rad: MAX_EXCURSION_RAD,
            period: DRIVER_PERIOD,
        }
    }
}

impl StillnessConfig {
    /// The shortest stretch of one unchanging setpoint this watch reports on,
    /// measured from the instant that setpoint last changed.
    ///
    /// The settle allowance is spent before the watch starts looking, and the
    /// minimum hold is what it wants after that. Anything else a step has to
    /// cover is the caller's own: the clock a shaped move streams new
    /// setpoints over, the allowance an arming is given. Callers sizing a step
    /// add those to this rather than restating the rule.
    #[must_use]
    pub fn shortest_judgeable_hold(&self) -> Duration {
        self.settle + self.min_hold
    }
}

/// One driver cycle, as the pose sample reports it.
///
/// Field names and polarity are the wire's, so a caller forwarding a sample has
/// nothing to invert: `missing` is the set of rows that did not answer, and the
/// two `_valid` flags say whether the vectors beside them mean anything.
#[derive(Clone, Copy, Debug)]
pub struct Sample<'a> {
    /// When the reading was taken, on the recording's own clock.
    pub t_ns: i64,
    /// Whether `present` is a complete reading.
    pub present_valid: bool,
    /// Whether `commanded` reflects a setpoint the driver is holding.
    pub commanded_valid: bool,
    /// The rows that did not answer this cycle.
    pub missing: JointFlags,
    /// Measured positions, radians.
    pub present: &'a JointVector,
    /// The setpoint held, radians. Meaningless when `commanded_valid` is false.
    pub commanded: &'a JointVector,
}

/// What one joint did over one stretch of holding one setpoint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct HoldWindow {
    /// Whose hold this was.
    pub joint: JointRef,
    /// The first and last valid readings in the stretch, on the recording's
    /// clock.
    pub start_ns: i64,
    /// See [`Self::start_ns`].
    pub end_ns: i64,
    /// How many valid readings it was computed over.
    pub samples: usize,
    /// Peak-to-peak spread of the present position, radians.
    pub excursion_rad: f64,
    /// Sign changes of the first difference, per second.
    pub reversals_per_s: f64,
    /// Mean number of samples between consecutive reversals, or `None` when
    /// the window held fewer than two of them and there is no interval to
    /// measure.
    ///
    /// Half a period: a joint oscillating turns round twice a cycle. Counted
    /// in samples rather than seconds because the sampling grid is what the
    /// aliasing is against, and converted with the window's own measured rate
    /// by [`Self::apparent_frequency_hz`].
    pub reversal_interval_mean_samples: Option<f64>,
    /// Population standard deviation of those intervals, in samples, or `None`
    /// on the same window. Zero for a perfectly regular turning.
    pub reversal_interval_spread_samples: Option<f64>,
    /// Mean of present minus commanded, radians. Signed: which side of the
    /// setpoint the joint sat on is part of the finding.
    pub mean_error_rad: f64,
    /// How long after the setpoint last changed the first judged reading was
    /// taken — the settle allowance, plus whatever gap ran past it.
    ///
    /// Printed rather than judged, and printed because the allowance is a
    /// guess at how long the servo takes to arrive: a window that opened while
    /// the joint was still travelling shows a large one-sided error at the
    /// open and few reversals, which is a move's tail rather than a joint
    /// hunting.
    pub opened_after_ns: i64,
    /// Present minus commanded at that first reading, radians. See
    /// [`Self::opened_after_ns`].
    pub error_at_open_rad: f64,
}

impl HoldWindow {
    /// How long the stretch ran.
    #[must_use]
    pub fn length(&self) -> Duration {
        Duration::from_nanos(self.end_ns.saturating_sub(self.start_ns).unsigned_abs())
    }

    /// The excursion in encoder counts, which is the unit it is worth reading
    /// in: a still joint reads one count or its neighbour.
    #[must_use]
    pub fn excursion_counts(&self) -> f64 {
        self.excursion_rad / COUNT_RAD
    }

    /// How fast this window's own readings arrived, Hz, or `None` for a window
    /// of one sample or one that spans no time.
    ///
    /// Measured off the window rather than assumed: the recordings this watch
    /// is replayed over were driven at 20, 24 and 32 ms grids, and a frequency
    /// computed against an assumed grid is wrong by that ratio without saying
    /// so.
    #[must_use]
    pub fn sample_rate_hz(&self) -> Option<f64> {
        let span = self.end_ns.checked_sub(self.start_ns)?;
        if span <= 0 || self.samples < 2 {
            return None;
        }
        Some((self.samples - 1) as f64 * 1e9 / span as f64)
    }

    /// The apparent period of the turning, in samples: two reversal intervals.
    #[must_use]
    pub fn apparent_period_samples(&self) -> Option<f64> {
        self.reversal_interval_mean_samples
            .filter(|mean| *mean > 0.0)
            .map(|mean| 2.0 * mean)
    }

    /// The apparent frequency of the turning, Hz, at the window's own sample
    /// rate.
    ///
    /// *Apparent* is the whole of the claim: see the aliasing caveat in the
    /// module header. A component this reads at `f` is at any of `|r·k ± f|`
    /// for the window's rate `r`.
    #[must_use]
    pub fn apparent_frequency_hz(&self) -> Option<f64> {
        Some(self.sample_rate_hz()? / self.apparent_period_samples()?)
    }
}

/// Why a hold is not still.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum StillnessError {
    /// The joint moved further over the hold than a still joint may.
    #[error(
        "{joint} moved {excursion_counts:.1} counts ({excursion_rad:.4} rad) over a {length_s:.1} s hold, \
         past the {bound_counts:.1} count bound, reversing {reversals_per_s:.1} times a second"
    )]
    Excursion {
        /// The joint that moved.
        joint: Name,
        /// What it covered, radians and counts.
        excursion_rad: f64,
        /// See [`Self::Excursion::excursion_rad`].
        excursion_counts: f64,
        /// The bound it passed, in counts.
        bound_counts: f64,
        /// How long the hold ran, seconds.
        length_s: f64,
        /// How often the direction of travel changed, per second.
        reversals_per_s: f64,
    },
}

/// Whether `window` is a still joint, against the bound in `cfg`.
///
/// # Errors
///
/// [`StillnessError::Excursion`] when the peak-to-peak spread exceeds
/// [`StillnessConfig::max_excursion_rad`].
pub fn judge(window: &HoldWindow, cfg: &StillnessConfig) -> Result<(), StillnessError> {
    if window.excursion_rad <= cfg.max_excursion_rad {
        return Ok(());
    }
    Err(StillnessError::Excursion {
        joint: Name(window.joint),
        excursion_rad: window.excursion_rad,
        excursion_counts: window.excursion_counts(),
        bound_counts: cfg.max_excursion_rad / COUNT_RAD,
        length_s: window.length().as_secs_f64(),
        reversals_per_s: window.reversals_per_s,
    })
}

/// What the watch saw, for the one line a report prints about the pass as a
/// whole.
///
/// The counts are what distinguishes a machine that never held still long
/// enough to judge from one that was judged and found wanting, and — through
/// `goal_changes` against `samples` — a setpoint that stopped being republished
/// verbatim from a joint that is hunting.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct StillnessCounts {
    /// Samples fed to the watch.
    pub samples: usize,
    /// Goal changes seen, summed over the watched joints, so one posture change
    /// counts once per row. Close to `samples` times the number of watched
    /// joints means the setpoint is not being held still to begin with.
    pub goal_changes: usize,
    /// Windows reported, one per joint per hold.
    pub judged: usize,
    /// Stretches dropped for being too short, or too sparse, to say anything;
    /// counted per joint in the same way.
    pub discarded_short: usize,
    /// Readings dropped for not being finite, counted per joint in the same
    /// way.
    ///
    /// Nothing on this machine can produce one — the driver's readings come
    /// from integer counts — so a non-zero figure here says the series came
    /// from somewhere else and arrived corrupted, which is worth seeing rather
    /// than averaging into a mean error of `NaN`.
    pub non_finite: usize,
}

/// A held joint's series, read the way this module reads one.
///
/// The three figures a hold is described by that are properties of the values
/// alone — how far they spread, how often they turned round, and how evenly
/// spaced those turns were — with the rule that a plateau is not a reversal: a
/// zero step neither breaks a run of one direction nor makes a change of one.
///
/// Public and separate from the watch because the same reading is taken by
/// instruments outside the driver's grid: a bench probe sampling one servo as
/// fast as the wire answers judges its hold against this module's bound, and a
/// second definition of "reversal" beside the bound would be an instrument and
/// a session disagreeing about the same joint.
///
/// Streaming and fixed-size, so the watch can hold one per joint: values go in
/// one at a time and nothing keeps the series.
#[derive(Clone, Copy, Debug, Default)]
pub struct Wobble {
    samples: usize,
    lowest: f64,
    highest: f64,
    previous: f64,
    /// Sign of the last non-zero first difference; zero before there was one.
    direction: i8,
    reversals: usize,
    /// Which reading the last reversal fell on, counting from one; zero before
    /// there was one.
    last_reversal_sample: usize,
    /// Welford over the sample counts between consecutive reversals: how many
    /// intervals, their running mean, and the sum of squared deviations from
    /// it. Three words rather than the series, which is the crate's rule.
    intervals: usize,
    interval_mean: f64,
    interval_m2: f64,
}

impl Wobble {
    /// Take in one reading.
    pub fn take(&mut self, value: f64) {
        self.samples += 1;
        if self.samples == 1 {
            self.lowest = value;
            self.highest = value;
            self.previous = value;
            return;
        }
        self.lowest = self.lowest.min(value);
        self.highest = self.highest.max(value);
        let step = value - self.previous;
        let direction = if step > 0.0 {
            1
        } else if step < 0.0 {
            -1
        } else {
            0
        };
        if direction != 0 {
            if self.direction != 0 && direction != self.direction {
                self.reverse();
            }
            self.direction = direction;
        }
        self.previous = value;
    }

    /// Take in every reading of a series that is already in hand.
    ///
    /// For a caller that sampled first and reads afterwards; the watch feeds
    /// [`Self::take`] instead.
    pub fn over(values: impl IntoIterator<Item = f64>) -> Self {
        let mut wobble = Self::default();
        for value in values {
            wobble.take(value);
        }
        wobble
    }

    /// How many readings went in.
    #[must_use]
    pub fn samples(&self) -> usize {
        self.samples
    }

    /// Peak-to-peak spread of the readings, in whatever unit they carried.
    #[must_use]
    pub fn excursion(&self) -> f64 {
        if self.samples == 0 {
            return 0.0;
        }
        self.highest - self.lowest
    }

    /// How many times the direction of travel changed.
    #[must_use]
    pub fn reversals(&self) -> usize {
        self.reversals
    }

    /// Direction changes per second, at a span measured by the caller.
    #[must_use]
    pub fn reversals_per_s(&self, span_s: f64) -> f64 {
        if span_s > 0.0 {
            self.reversals as f64 / span_s
        } else {
            0.0
        }
    }

    /// Mean and population standard deviation of the sample counts between
    /// consecutive reversals, or `None` when fewer than two reversals left
    /// nothing to measure.
    #[must_use]
    pub fn interval_stats(&self) -> (Option<f64>, Option<f64>) {
        if self.intervals == 0 {
            return (None, None);
        }
        let variance = self.interval_m2 / self.intervals as f64;
        (Some(self.interval_mean), Some(variance.max(0.0).sqrt()))
    }

    /// The rate the readings arrived at, Hz, over a span the caller measured,
    /// or `None` for one reading or no span.
    ///
    /// Measured rather than assumed, which is the whole reason a span is asked
    /// for: a series read at whatever rate it achieved must not be reported as
    /// though it ran at the rate it meant to.
    #[must_use]
    pub fn rate_hz(&self, span_s: f64) -> Option<f64> {
        if self.samples < 2 || span_s <= 0.0 {
            return None;
        }
        Some((self.samples - 1) as f64 / span_s)
    }

    /// Take in a reversal seen on the reading just accepted.
    fn reverse(&mut self) {
        self.reversals += 1;
        if self.last_reversal_sample != 0 {
            let interval = (self.samples - self.last_reversal_sample) as f64;
            self.intervals += 1;
            let delta = interval - self.interval_mean;
            self.interval_mean += delta / self.intervals as f64;
            self.interval_m2 += delta * (interval - self.interval_mean);
        }
        self.last_reversal_sample = self.samples;
    }
}

/// The nanoseconds in `duration`, saturating at the longest a signed count of
/// them can express.
///
/// A settle allowance or a minimum hold of nearly three centuries is not a
/// configuration this distinguishes from an infinite one.
fn nanos(duration: Duration) -> i64 {
    i64::try_from(duration.as_nanos()).unwrap_or(i64::MAX)
}

/// One joint's hold in progress: the running numbers, and nothing else.
#[derive(Clone, Copy, Debug)]
struct Open {
    start_ns: i64,
    opened_after_ns: i64,
    error_at_open: f64,
    last_ns: i64,
    /// The present position series, read by the shared reader.
    wobble: Wobble,
    error_sum: f64,
}

/// One watched joint.
#[derive(Clone, Copy, Debug)]
struct Watched {
    joint: JointRef,
    /// The setpoint this joint is currently holding, or `None` when the driver
    /// is holding nothing for it.
    held: Option<f64>,
    /// When `held` last became what it is.
    changed_ns: Option<i64>,
    open: Option<Open>,
}

/// The stillness measurement, carried across the samples of one recording.
///
/// Fed one cycle at a time by [`Self::look`], in order. Each watched joint is
/// cut into the stretches where the driver held one setpoint for it:
///
/// - A **goal change** is a commanded value not bit-equal to the one this joint
///   last held, or the driver ceasing to hold anything. Exact equality is the
///   right test because a held setpoint is republished verbatim rather than
///   recomputed; were that ever to stop being true, every stretch would split,
///   none would survive [`StillnessConfig::min_hold`], and
///   [`StillnessCounts`] would show goal changes about equal to samples times
///   the number of watched joints, since the count is summed over them — a
///   failure with its own signature rather than one that reads as a hunting
///   joint.
/// - A hold **opens** at the first reading a full [`StillnessConfig::settle`]
///   after a goal change, and **closes** at the next goal change or at
///   [`Self::finish`].
/// - A reading is **skipped** for a joint, contributing nothing and splitting
///   nothing, when the sample is not a valid reading or the joint is among the
///   rows that did not answer. A gap in the reading is not a goal change.
#[derive(Clone, Debug)]
pub struct StillnessWatch {
    cfg: StillnessConfig,
    settle_ns: i64,
    min_hold_ns: i64,
    /// Fewest valid readings a judged window may be computed over.
    min_samples: usize,
    watched: Vec<Watched>,
    counts: StillnessCounts,
}

impl StillnessWatch {
    /// A watch that has seen nothing, following `joints`.
    ///
    /// The one allocation is here: a slot per watched joint, sized once and
    /// never grown. [`JointRef::None`] names no servo and is dropped.
    #[must_use]
    pub fn new(cfg: StillnessConfig, joints: &[JointRef]) -> Self {
        let period = cfg.period.as_nanos().max(1);
        let min_samples = usize::try_from(cfg.min_hold.as_nanos() / period).unwrap_or(usize::MAX);
        Self {
            settle_ns: nanos(cfg.settle),
            min_hold_ns: nanos(cfg.min_hold),
            min_samples,
            cfg,
            watched: joints
                .iter()
                .copied()
                .filter(|joint| *joint != JointRef::None)
                .map(|joint| Watched {
                    joint,
                    held: None,
                    changed_ns: None,
                    open: None,
                })
                .collect(),
            counts: StillnessCounts::default(),
        }
    }

    /// The bounds and allowances this watch is measuring against.
    #[must_use]
    pub fn config(&self) -> StillnessConfig {
        self.cfg
    }

    /// What the pass has seen so far.
    #[must_use]
    pub fn counts(&self) -> StillnessCounts {
        self.counts
    }

    /// Take in one driver cycle. Any window that closes on this sample is
    /// pushed to `out`.
    pub fn look(&mut self, sample: &Sample, out: &mut Vec<HoldWindow>) {
        self.counts.samples += 1;
        for index in 0..self.watched.len() {
            self.look_at(index, sample, out);
        }
    }

    /// End of series: close every open window that has reached
    /// [`StillnessConfig::min_hold`].
    ///
    /// A recording simply stops, and the last hold in it is as much a hold as
    /// any other; without this the longest stretch of the run would be the one
    /// never reported.
    pub fn finish(&mut self, out: &mut Vec<HoldWindow>) {
        for index in 0..self.watched.len() {
            self.close(index, out);
        }
    }

    /// One joint's share of one cycle.
    fn look_at(&mut self, index: usize, sample: &Sample, out: &mut Vec<HoldWindow>) {
        let joint = self.watched[index].joint;
        let goal = if sample.commanded_valid {
            sample.commanded.get(joint)
        } else {
            None
        };
        if goal != self.watched[index].held {
            self.counts.goal_changes += 1;
            self.close(index, out);
            let watched = &mut self.watched[index];
            watched.held = goal;
            watched.changed_ns = Some(sample.t_ns);
        }
        let Some(held) = self.watched[index].held else {
            return;
        };
        if !sample.present_valid || flags::contains(sample.missing, joint) {
            return;
        }
        let Some(present) = sample.present.get(joint) else {
            return;
        };
        if !present.is_finite() || !held.is_finite() {
            self.counts.non_finite += 1;
            return;
        }
        let settled = self.watched[index]
            .changed_ns
            .is_some_and(|changed| sample.t_ns.saturating_sub(changed) >= self.settle_ns);
        if !settled {
            return;
        }
        let error = present - held;
        let changed_ns = self.watched[index].changed_ns.unwrap_or(sample.t_ns);
        match &mut self.watched[index].open {
            None => {
                let mut wobble = Wobble::default();
                wobble.take(present);
                self.watched[index].open = Some(Open {
                    start_ns: sample.t_ns,
                    opened_after_ns: sample.t_ns.saturating_sub(changed_ns),
                    error_at_open: error,
                    last_ns: sample.t_ns,
                    wobble,
                    error_sum: error,
                });
            }
            Some(open) => {
                open.last_ns = sample.t_ns;
                open.error_sum += error;
                open.wobble.take(present);
            }
        }
    }

    /// Close one joint's window, reporting it if it says anything.
    fn close(&mut self, index: usize, out: &mut Vec<HoldWindow>) {
        let joint = self.watched[index].joint;
        let Some(open) = self.watched[index].open.take() else {
            return;
        };
        let span_ns = open.last_ns.saturating_sub(open.start_ns);
        if span_ns < self.min_hold_ns || open.wobble.samples() < self.min_samples {
            self.counts.discarded_short += 1;
            return;
        }
        let seconds = Duration::from_nanos(span_ns.unsigned_abs()).as_secs_f64();
        let (interval_mean, interval_spread) = open.wobble.interval_stats();
        self.counts.judged += 1;
        out.push(HoldWindow {
            joint,
            start_ns: open.start_ns,
            end_ns: open.last_ns,
            samples: open.wobble.samples(),
            excursion_rad: open.wobble.excursion(),
            reversals_per_s: open.wobble.reversals_per_s(seconds),
            reversal_interval_mean_samples: interval_mean,
            reversal_interval_spread_samples: interval_spread,
            mean_error_rad: open.error_sum / open.wobble.samples() as f64,
            opened_after_ns: open.opened_after_ns,
            error_at_open_rad: open.error_at_open,
        });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The side every case here watches, unless it says otherwise.
    const RIGHT: JointRef = JointRef::AntennaRight;

    /// The driver's cycle, in nanoseconds, as the cases lay a series out on it.
    const PERIOD_NS: i64 = 20_000_000;

    /// One cycle of a synthetic series, for the one joint a case is watching.
    #[derive(Clone, Copy, Debug)]
    struct Tick {
        present: f64,
        /// The setpoint, or `None` for a driver holding nothing.
        commanded: Option<f64>,
        present_valid: bool,
        /// Whether the watched row is among those that did not answer.
        missing: bool,
    }

    impl Tick {
        /// A clean cycle: read `present`, holding `commanded`.
        fn held(present: f64, commanded: f64) -> Self {
            Self {
                present,
                commanded: Some(commanded),
                present_valid: true,
                missing: false,
            }
        }

        /// A cycle in which the driver holds nothing.
        fn released(present: f64) -> Self {
            Self {
                commanded: None,
                ..Self::held(present, 0.0)
            }
        }

        /// A cycle whose reading is not a complete one.
        fn blind(self) -> Self {
            Self {
                present_valid: false,
                ..self
            }
        }

        /// A cycle the watched row did not answer in.
        fn absent(self) -> Self {
            Self {
                missing: true,
                ..self
            }
        }
    }

    /// How many cycles `seconds` of series is.
    fn cycles(seconds: f64) -> usize {
        (seconds / 0.02).round() as usize
    }

    /// `seconds` of holding `goal` with the joint reading whatever `present`
    /// says of the cycle index.
    fn holding(seconds: f64, goal: f64, mut present: impl FnMut(usize) -> f64) -> Vec<Tick> {
        (0..cycles(seconds))
            .map(|index| Tick::held(present(index), goal))
            .collect()
    }

    /// `seconds` of a joint sitting exactly on `goal`.
    fn still(seconds: f64, goal: f64) -> Vec<Tick> {
        holding(seconds, goal, |_| goal)
    }

    /// Feed a series to a watch on `joints` and close it out, the series laid
    /// out on the driver's grid.
    fn watch(
        cfg: StillnessConfig,
        joints: &[JointRef],
        ticks: &[Tick],
    ) -> (Vec<HoldWindow>, StillnessCounts) {
        watch_on(PERIOD_NS, cfg, joints, ticks)
    }

    /// The same, with the series laid out on a grid of `period_ns`.
    ///
    /// The recordings this watch is replayed over were driven at three
    /// different grids, so what a window says about frequency has to come off
    /// its own spacing.
    fn watch_on(
        period_ns: i64,
        cfg: StillnessConfig,
        joints: &[JointRef],
        ticks: &[Tick],
    ) -> (Vec<HoldWindow>, StillnessCounts) {
        let mut watch = StillnessWatch::new(cfg, joints);
        let mut out = Vec::new();
        for (index, tick) in ticks.iter().enumerate() {
            let mut present = JointVector::default();
            let mut commanded = JointVector::default();
            for joint in joints {
                present.set(*joint, tick.present);
                commanded.set(*joint, tick.commanded.unwrap_or(0.0));
            }
            let missing = if tick.missing {
                let mut set = JointFlags::NONE;
                for joint in joints {
                    flags::insert(&mut set, *joint);
                }
                set
            } else {
                JointFlags::NONE
            };
            watch.look(
                &Sample {
                    t_ns: index as i64 * period_ns,
                    present_valid: tick.present_valid,
                    commanded_valid: tick.commanded.is_some(),
                    missing,
                    present: &present,
                    commanded: &commanded,
                },
                &mut out,
            );
        }
        watch.finish(&mut out);
        (out, watch.counts())
    }

    /// The common case: the shipped configuration, one antenna.
    fn seen(ticks: &[Tick]) -> (Vec<HoldWindow>, StillnessCounts) {
        watch(StillnessConfig::default(), &[RIGHT], ticks)
    }

    /// The only window a case expects, or a panic naming how many there were.
    fn only(windows: &[HoldWindow]) -> HoldWindow {
        assert_eq!(windows.len(), 1, "{windows:?}");
        windows[0]
    }

    #[test]
    fn a_joint_sitting_on_its_setpoint_is_still() {
        let (windows, counts) = seen(&still(10.0, 0.3));
        let window = only(&windows);
        assert_eq!(window.joint, RIGHT);
        assert_eq!(window.excursion_rad, 0.0);
        assert_eq!(window.reversals_per_s, 0.0);
        assert_eq!(window.mean_error_rad, 0.0);
        assert_eq!(judge(&window, &StillnessConfig::default()), Ok(()));
        assert_eq!(counts.judged, 1);
        assert_eq!(counts.discarded_short, 0);
        assert_eq!(counts.goal_changes, 1);
    }

    #[test]
    fn the_settle_allowance_drops_the_head_of_the_hold() {
        let cfg = StillnessConfig::default();
        let (windows, _) = seen(&still(10.0, 0.0));
        let window = only(&windows);
        assert_eq!(window.start_ns, nanos(cfg.settle));
        assert_eq!(window.end_ns, (cycles(10.0) as i64 - 1) * PERIOD_NS);
        assert_eq!(window.samples, cycles(10.0) - cycles(4.0));
        // The allowance is the whole of the gap here: the setpoint changed on
        // the first sample of the series.
        assert_eq!(window.opened_after_ns, nanos(cfg.settle));
        assert_eq!(window.error_at_open_rad, 0.0);
    }

    #[test]
    fn a_one_count_flicker_is_still() {
        let goal = 0.0;
        let (windows, _) = seen(&holding(10.0, goal, |index| {
            if index % 2 == 0 {
                goal
            } else {
                goal + COUNT_RAD
            }
        }));
        let window = only(&windows);
        assert!((window.excursion_counts() - 1.0).abs() < 1e-9, "{window:?}");
        assert_eq!(judge(&window, &StillnessConfig::default()), Ok(()));
        // Every cycle turns round, which is the encoder's least bit and not a
        // frequency: the rate is printed for exactly this reason.
        assert!(window.reversals_per_s > 40.0, "{window:?}");
    }

    #[test]
    fn a_three_count_square_wave_is_not_still() {
        let goal = 0.0;
        let (windows, _) = seen(&holding(10.0, goal, |index| {
            let seconds = index as f64 * 0.02;
            // 8 Hz: half a period every sixteenth of a second.
            if (seconds * 16.0) as i64 % 2 == 0 {
                3.0 * COUNT_RAD
            } else {
                -3.0 * COUNT_RAD
            }
        }));
        let window = only(&windows);
        assert!((window.excursion_counts() - 6.0).abs() < 1e-9, "{window:?}");
        let Err(error) = judge(&window, &StillnessConfig::default()) else {
            panic!("{window:?} judged still");
        };
        let said = error.to_string();
        assert!(said.contains("right antenna"), "{said}");
        assert!(said.contains("6.0 counts"), "{said}");
        assert!(said.contains("2.0 count bound"), "{said}");
    }

    #[test]
    fn a_goal_change_splits_the_hold() {
        let mut ticks = still(10.0, 0.0);
        ticks.extend(still(10.0, 0.3));
        let (windows, counts) = seen(&ticks);
        assert_eq!(windows.len(), 2, "{windows:?}");
        assert_eq!(counts.goal_changes, 2);
        assert_eq!(windows[0].start_ns, nanos(SETTLE));
        assert_eq!(windows[0].end_ns, (cycles(10.0) as i64 - 1) * PERIOD_NS);
        // The second window's settle counts from the change, not from the run.
        assert_eq!(
            windows[1].start_ns,
            (cycles(10.0) as i64) * PERIOD_NS + nanos(SETTLE)
        );
        assert_eq!(windows[1].mean_error_rad, 0.0);
    }

    #[test]
    fn holding_nothing_closes_the_window_and_opens_none() {
        let mut ticks = still(10.0, 0.0);
        ticks.extend((0..cycles(10.0)).map(|_| Tick::released(0.0)));
        let (windows, counts) = seen(&ticks);
        let window = only(&windows);
        assert_eq!(window.end_ns, (cycles(10.0) as i64 - 1) * PERIOD_NS);
        // Ceasing to hold is one change; every cycle after it holds the same
        // nothing.
        assert_eq!(counts.goal_changes, 2);
        assert_eq!(counts.discarded_short, 0);
    }

    #[test]
    fn a_series_shorter_than_the_settle_and_the_hold_says_nothing() {
        let (windows, counts) = seen(&still(5.0, 0.0));
        assert!(windows.is_empty(), "{windows:?}");
        assert_eq!(counts.judged, 0);
        assert_eq!(counts.discarded_short, 1);
    }

    /// The figure callers size their steps against is the one the watch acts
    /// on: a stretch of a setpoint the length that figure names is judged, and
    /// the same stretch a hair shorter is not.
    ///
    /// The one sample of slack is the span itself: a window is timed from its
    /// first judged reading to its last, so the last cycle of a stretch is its
    /// end rather than a further period of it.
    #[test]
    fn the_shortest_judgeable_hold_is_what_a_step_has_to_cover() {
        let cfg = StillnessConfig::default();
        assert_eq!(cfg.shortest_judgeable_hold(), cfg.settle + cfg.min_hold);
        let floor = cfg.shortest_judgeable_hold().as_secs_f64();
        let (judged, _) = seen(&still(floor + 0.02, 0.0));
        assert_eq!(only(&judged).joint, RIGHT);
        let (none, counts) = seen(&still(floor, 0.0));
        assert!(none.is_empty(), "{none:?}");
        assert_eq!(counts.discarded_short, 1);
    }

    #[test]
    fn a_gap_in_the_reading_is_not_a_goal_change() {
        let mut ticks = still(10.0, 0.0);
        for tick in &mut ticks[cycles(5.0)..cycles(5.5)] {
            *tick = tick.absent();
        }
        for tick in &mut ticks[cycles(6.0)..cycles(6.5)] {
            *tick = tick.blind();
        }
        let (windows, counts) = seen(&ticks);
        let window = only(&windows);
        assert_eq!(window.start_ns, nanos(SETTLE));
        assert_eq!(window.end_ns, (cycles(10.0) as i64 - 1) * PERIOD_NS);
        assert_eq!(window.samples, cycles(10.0) - cycles(4.0) - cycles(1.0));
        assert_eq!(counts.goal_changes, 1);
    }

    #[test]
    fn a_hold_that_is_mostly_gap_is_discarded() {
        let mut ticks = still(10.0, 0.0);
        // Everything after the settle allowance but a scatter of readings.
        for (index, tick) in ticks.iter_mut().enumerate() {
            if index >= cycles(2.0) && index % 10 != 0 {
                *tick = tick.blind();
            }
        }
        let (windows, counts) = seen(&ticks);
        assert!(windows.is_empty(), "{windows:?}");
        assert_eq!(counts.discarded_short, 1);
    }

    #[test]
    fn a_setpoint_that_jitters_never_holds_anything() {
        let ticks: Vec<Tick> = (0..cycles(30.0))
            .map(|index| Tick::held(0.0, f64::from(index as u32) * f64::EPSILON))
            .collect();
        let (windows, counts) = seen(&ticks);
        assert!(windows.is_empty(), "{windows:?}");
        assert_eq!(counts.goal_changes, counts.samples);
        assert_eq!(counts.samples, cycles(30.0));
    }

    /// The same jitter over three rows: the printed signature is the one the
    /// counts document, which is a goal change per row per sample rather than
    /// one per sample.
    #[test]
    fn the_jitter_signature_scales_with_the_watched_rows() {
        let joints = [JointRef::BodyYaw, RIGHT, JointRef::AntennaLeft];
        let ticks: Vec<Tick> = (0..cycles(30.0))
            .map(|index| Tick::held(0.0, f64::from(index as u32) * f64::EPSILON))
            .collect();
        let (windows, counts) = watch(StillnessConfig::default(), &joints, &ticks);
        assert!(windows.is_empty(), "{windows:?}");
        assert_eq!(counts.goal_changes, counts.samples * joints.len());
    }

    /// A recording on a slower grid than the driver's: the sparsity floor
    /// follows the period the caller states, so a hold that is long and fully
    /// sampled is judged rather than discarded as mostly gaps.
    #[test]
    fn the_sparsity_floor_follows_the_configured_period() {
        let cfg = StillnessConfig {
            period: Duration::from_millis(100),
            ..StillnessConfig::default()
        };
        // The series a_hold_that_is_mostly_gap_is_discarded throws away: one
        // reading in ten, which is a full one on a 100 ms grid.
        let mut ticks = still(10.0, 0.0);
        for (index, tick) in ticks.iter_mut().enumerate() {
            if index >= cycles(2.0) && index % 10 != 0 {
                *tick = tick.blind();
            }
        }
        let (windows, counts) = watch(cfg, &[RIGHT], &ticks);
        assert_eq!(windows.len(), 1, "{windows:?}");
        assert_eq!(counts.discarded_short, 0);
    }

    #[test]
    fn finishing_closes_what_a_goal_change_would_have() {
        let ticks = still(10.0, 0.0);
        let (finished, _) = seen(&ticks);
        let mut changed = ticks.clone();
        changed.push(Tick::held(0.0, 0.3));
        let (split, _) = seen(&changed);
        assert_eq!(finished.len(), 1, "{finished:?}");
        assert_eq!(split.first(), finished.first());
    }

    #[test]
    fn every_watched_joint_gets_its_own_window() {
        let joints = [JointRef::BodyYaw, RIGHT, JointRef::AntennaLeft];
        let (windows, counts) = watch(StillnessConfig::default(), &joints, &still(10.0, 0.1));
        assert_eq!(windows.len(), joints.len(), "{windows:?}");
        for (window, joint) in windows.iter().zip(joints) {
            assert_eq!(window.joint, joint);
            assert_eq!(window.excursion_rad, 0.0);
        }
        assert_eq!(counts.judged, joints.len());
        assert_eq!(counts.goal_changes, joints.len());
        assert_eq!(counts.samples, cycles(10.0));
    }

    #[test]
    fn a_joint_that_names_no_servo_is_not_watched() {
        let (windows, counts) = watch(
            StillnessConfig::default(),
            &[JointRef::None, RIGHT],
            &still(10.0, 0.0),
        );
        assert_eq!(only(&windows).joint, RIGHT);
        assert_eq!(counts.goal_changes, 1);
    }

    #[test]
    fn a_standing_offset_shows_as_a_mean_error() {
        let goal = 0.0;
        let (windows, _) = seen(&holding(10.0, goal, |_| 4.0 * COUNT_RAD));
        let window = only(&windows);
        assert_eq!(window.excursion_rad, 0.0);
        assert_eq!(window.reversals_per_s, 0.0);
        assert!(
            (window.mean_error_rad - 4.0 * COUNT_RAD).abs() < 1e-12,
            "{window:?}"
        );
        // Sitting off the setpoint without moving is not what the bound is on.
        assert_eq!(judge(&window, &StillnessConfig::default()), Ok(()));
    }

    #[test]
    fn the_bound_is_the_encoders_own_flicker() {
        assert!((COUNT_RAD - core::f64::consts::TAU / 4096.0).abs() < 1e-15);
        assert!((MAX_EXCURSION_RAD / COUNT_RAD - 2.0).abs() < 1e-12);
        let cfg = StillnessConfig::default();
        assert_eq!(cfg.max_excursion_rad, MAX_EXCURSION_RAD);
        assert_eq!(cfg.settle, SETTLE);
        assert_eq!(cfg.min_hold, MIN_HOLD);
        assert_eq!(cfg.period, DRIVER_PERIOD);
        assert_eq!(DRIVER_PERIOD, Duration::from_millis(20));
    }

    /// The bound is a limit the joint may reach, not one it must stay under.
    /// The figure in it is going to be replaced by an observation of a hold
    /// that was accepted as still, so that hold reading its own figure back has
    /// to pass.
    #[test]
    fn the_bound_admits_a_hold_that_reads_exactly_it() {
        let cfg = StillnessConfig {
            max_excursion_rad: 5.0 * COUNT_RAD,
            ..StillnessConfig::default()
        };
        let at = |excursion_rad| HoldWindow {
            joint: RIGHT,
            start_ns: 0,
            end_ns: nanos(cfg.min_hold),
            samples: 100,
            excursion_rad,
            reversals_per_s: 0.0,
            reversal_interval_mean_samples: None,
            reversal_interval_spread_samples: None,
            mean_error_rad: 0.0,
            opened_after_ns: nanos(cfg.settle),
            error_at_open_rad: 0.0,
        };
        assert_eq!(judge(&at(cfg.max_excursion_rad), &cfg), Ok(()));
        let over = cfg.max_excursion_rad * (1.0 + f64::EPSILON);
        let Err(error) = judge(&at(over), &cfg) else {
            panic!("a hold past the bound judged still");
        };
        let StillnessError::Excursion { bound_counts, .. } = error;
        // The configured bound, not the default one.
        assert!((bound_counts - 5.0).abs() < 1e-9, "{error}");
    }

    /// The other half of holding nothing: the setpoint comes back, and the
    /// stretch after it is a hold of its own whose settle allowance runs from
    /// the moment it came back rather than from the run.
    #[test]
    fn a_setpoint_that_comes_back_opens_a_second_window() {
        let mut ticks = still(10.0, 0.0);
        ticks.extend((0..cycles(10.0)).map(|_| Tick::released(0.0)));
        ticks.extend(still(10.0, 0.0));
        let (windows, counts) = seen(&ticks);
        assert_eq!(windows.len(), 2, "{windows:?}");
        assert_eq!(windows[0].start_ns, nanos(SETTLE));
        assert_eq!(
            windows[1].start_ns,
            (cycles(20.0) as i64) * PERIOD_NS + nanos(SETTLE)
        );
        assert_eq!(windows[1].opened_after_ns, nanos(SETTLE));
        // Held, released, held again.
        assert_eq!(counts.goal_changes, 3);
    }

    /// A real series at a couple of counts of amplitude is mostly flat: a step
    /// of zero is not a turn, and the direction stands until the joint moves
    /// the other way. Otherwise the reversal rate would collapse on exactly the
    /// data it exists to characterise.
    #[test]
    fn a_plateau_between_steps_is_not_a_reversal() {
        // The allowance first, so every reading below is inside the window.
        let mut ticks = still(4.0, 0.0);
        // Up in steps with a flat stretch at each, then back down the same way:
        // one turn at the top of a repeat, one at the trough between repeats.
        let repeats = 10;
        let staircase = [
            0.0, 0.0, 1.0, 1.0, 1.0, 2.0, 2.0, 3.0, 3.0, 3.0, 2.0, 2.0, 1.0, 1.0, 0.0,
        ];
        for _ in 0..repeats {
            ticks.extend(
                staircase
                    .iter()
                    .map(|counts| Tick::held(counts * COUNT_RAD, 0.0)),
            );
        }
        let (windows, _) = seen(&ticks);
        let window = only(&windows);
        assert!((window.excursion_counts() - 3.0).abs() < 1e-9, "{window:?}");
        let reversals = window.reversals_per_s * window.length().as_secs_f64();
        assert!(
            (reversals - f64::from(2 * repeats - 1)).abs() < 1e-6,
            "{window:?} counted {reversals} turns"
        );
    }

    /// A reading that is not a number is dropped and counted rather than
    /// averaged: a window whose mean error is `NaN` would print as a passing
    /// hold with a garbage figure beside it.
    #[test]
    fn a_reading_that_is_not_finite_is_dropped_and_counted() {
        let mut ticks = still(10.0, 0.0);
        for tick in &mut ticks[cycles(6.0)..cycles(6.5)] {
            tick.present = f64::NAN;
        }
        ticks[cycles(7.0)].present = f64::INFINITY;
        let (windows, counts) = seen(&ticks);
        let window = only(&windows);
        assert_eq!(counts.non_finite, cycles(0.5) + 1);
        assert_eq!(window.samples, cycles(10.0) - cycles(4.0) - cycles(0.5) - 1);
        assert!(window.mean_error_rad.is_finite(), "{window:?}");
        assert_eq!(window.excursion_rad, 0.0);
        assert_eq!(counts.goal_changes, 1);
    }

    /// A regular oscillation reads its own period back, in samples and as a
    /// frequency against the grid the series was laid out on — never against
    /// an assumed one.
    ///
    /// The series turns round every second sample, which is a four-sample
    /// period whatever the spacing; the frequency that comes out is therefore
    /// a quarter of the series' own rate, and the two spacings here are the
    /// check that it is the series' rate and not the driver's.
    #[test]
    fn a_regular_oscillation_reads_its_period_off_its_own_sample_rate() {
        // 0, 1, 2, 1 counts: two reversals per four samples.
        let ticks = holding(20.0, 0.0, |index| {
            f64::from(2 - (2 - i32::try_from(index % 4).unwrap_or(0)).abs()) * COUNT_RAD
        });
        for period_ns in [PERIOD_NS, 32_000_000] {
            let (windows, _) = watch_on(period_ns, StillnessConfig::default(), &[RIGHT], &ticks);
            let window = only(&windows);
            let mean = window
                .reversal_interval_mean_samples
                .expect("a window that turned round many times has intervals");
            let spread = window
                .reversal_interval_spread_samples
                .expect("and a spread of them");
            assert!((mean - 2.0).abs() < 1e-9, "{window:?}");
            assert!(spread < 1e-9, "{window:?}");
            assert!(
                (window.apparent_period_samples().expect("a period") - 4.0).abs() < 1e-9,
                "{window:?}"
            );
            let rate = window.sample_rate_hz().expect("a measured rate");
            assert!((rate - 1e9 / period_ns as f64).abs() < 1e-6, "{window:?}");
            let hz = window.apparent_frequency_hz().expect("a frequency");
            assert!((hz - rate / 4.0).abs() < 1e-9, "{window:?} read {hz} Hz");
        }
    }

    /// Encoder dither turns round as often as a fast oscillation does and is
    /// not one: it turns round at scattered intervals, and the spread beside
    /// the mean is what says so.
    #[test]
    fn lsb_dither_reads_a_spread_comparable_to_its_mean() {
        // A fixed sequence, so the figures this asserts are the same every
        // run: a joint flickering between one count and its neighbour.
        let mut state: u32 = 0x2545_f491;
        let ticks = holding(20.0, 0.0, |_| {
            state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
            f64::from((state >> 30) & 1) * COUNT_RAD
        });
        let (windows, _) = seen(&ticks);
        let window = only(&windows);
        let mean = window.reversal_interval_mean_samples.expect("intervals");
        let spread = window.reversal_interval_spread_samples.expect("a spread");
        assert!(window.excursion_counts() <= 1.0 + 1e-9, "{window:?}");
        assert!(
            spread > 0.5 * mean,
            "{window:?}: dither read a spread of {spread} against a mean of {mean}"
        );
    }

    /// One turn is not a period. A joint that drifts out and comes back has a
    /// reversal and no interval, and the window says so rather than inventing
    /// a figure out of one turn.
    #[test]
    fn a_single_reversal_leaves_no_interval_to_measure() {
        let turn = cycles(16.0);
        let ticks = holding(20.0, 0.0, |index| {
            let from_turn = index.abs_diff(turn) as f64;
            (10.0 - from_turn * 0.01) * COUNT_RAD
        });
        let (windows, _) = seen(&ticks);
        let window = only(&windows);
        let reversals = window.reversals_per_s * window.length().as_secs_f64();
        assert!((reversals - 1.0).abs() < 1e-6, "{window:?}");
        assert_eq!(window.reversal_interval_mean_samples, None, "{window:?}");
        assert_eq!(window.reversal_interval_spread_samples, None, "{window:?}");
        assert_eq!(window.apparent_period_samples(), None, "{window:?}");
        assert_eq!(window.apparent_frequency_hz(), None, "{window:?}");
    }

    /// The reader the watch is built out of says the same things standing on
    /// its own, which is how an instrument off the driver's grid takes this
    /// module's reading rather than a second one of its own.
    #[test]
    fn the_series_reader_reads_a_series_the_way_the_watch_does() {
        // A four-sample cycle with a plateau in it: the plateau neither breaks
        // a run of one direction nor makes a reversal, which is the rule the
        // whole module is written around.
        let wobble = Wobble::over([0.0, 1.0, 1.0, 2.0, 1.0, 0.0, 1.0, 2.0]);
        assert_eq!(wobble.samples(), 8);
        assert!((wobble.excursion() - 2.0).abs() < 1e-12);
        assert_eq!(wobble.reversals(), 2);
        assert_eq!(wobble.interval_stats(), (Some(2.0), Some(0.0)));
        // The rate is the caller's span over the caller's readings; a span of
        // none is no rate rather than an infinity.
        assert!(
            wobble
                .rate_hz(0.14)
                .is_some_and(|rate| (rate - 50.0).abs() < 1e-9)
        );
        assert_eq!(wobble.rate_hz(0.0), None);
        assert!((wobble.reversals_per_s(0.14) - 2.0 / 0.14).abs() < 1e-9);

        // Nothing read: no spread, no excursion, no division by an empty
        // series.
        let empty = Wobble::default();
        assert_eq!(empty.samples(), 0);
        assert!(empty.excursion().abs() < 1e-12);
        assert_eq!(empty.interval_stats(), (None, None));
        assert_eq!(empty.rate_hz(1.0), None);

        // And the watch's own window is that reader's answer over the same
        // series, not a second reading of it.
        let (windows, _) = seen(&holding(20.0, 0.0, |index| {
            if index % 2 == 0 { COUNT_RAD } else { 0.0 }
        }));
        let window = only(&windows);
        let alternating = Wobble::over(
            (0..window.samples).map(|index| if index % 2 == 0 { COUNT_RAD } else { 0.0 }),
        );
        // Every figure the window sources from the reader, not two of them: a
        // field mis-wired during the extraction would leave the watch's own
        // cases green while the session and the bench probe reported different
        // numbers for the same hold.
        let span = (window.end_ns - window.start_ns) as f64 / 1e9;
        assert_eq!(window.samples, alternating.samples(), "{window:?}");
        assert!(
            (window.excursion_rad - alternating.excursion()).abs() < 1e-12,
            "{window:?}"
        );
        assert!(
            (window.reversals_per_s - alternating.reversals_per_s(span)).abs() < 1e-12,
            "{window:?}"
        );
        let (mean, spread) = alternating.interval_stats();
        assert_eq!(window.reversal_interval_mean_samples, mean, "{window:?}");
        assert_eq!(
            window.reversal_interval_spread_samples, spread,
            "{window:?}"
        );
        // And the figures are figures, not two `None`s agreeing.
        assert!(window.reversals_per_s > 0.0, "{window:?}");
        assert!(
            spread.is_some_and(|spread| spread.abs() < 1e-12),
            "{spread:?}"
        );
    }
}
