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
//! What a window carries is three numbers, and only the first is judged:
//!
//! - the **excursion**, the peak-to-peak spread of the present position, which
//!   bounds how far the joint moved while it was meant to be still;
//! - the **reversal rate**, how often the direction of travel changed;
//! - the **mean error**, how far from the setpoint the joint sat on average.
//!
//! The last two are printed rather than judged: together they separate a joint
//! hunting about its setpoint (many reversals, error averaging near zero) from
//! one sitting still at an offset (few reversals, a standing error), which are
//! different findings about the mechanism even when the excursion is the same.
//!
//! **Nyquist caveat.** The series is the driver's 50 Hz read of the encoder, so
//! a limit cycle above 25 Hz arrives aliased: it appears at some lower
//! frequency, and the reversal rate is therefore a floor on how often the joint
//! turned round, not a measurement of it. The peak-to-peak excursion survives
//! aliasing — a sampled extreme is a real extreme — which is why the bound is
//! on amplitude and the reversal rate is not bounded at all.

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
    samples: usize,
    lowest: f64,
    highest: f64,
    previous: f64,
    /// Sign of the last non-zero first difference; zero before there was one.
    direction: i8,
    reversals: usize,
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
                self.watched[index].open = Some(Open {
                    start_ns: sample.t_ns,
                    opened_after_ns: sample.t_ns.saturating_sub(changed_ns),
                    error_at_open: error,
                    last_ns: sample.t_ns,
                    samples: 1,
                    lowest: present,
                    highest: present,
                    previous: present,
                    direction: 0,
                    reversals: 0,
                    error_sum: error,
                });
            }
            Some(open) => {
                open.last_ns = sample.t_ns;
                open.samples += 1;
                open.lowest = open.lowest.min(present);
                open.highest = open.highest.max(present);
                open.error_sum += error;
                let step = present - open.previous;
                let direction = if step > 0.0 {
                    1
                } else if step < 0.0 {
                    -1
                } else {
                    0
                };
                if direction != 0 {
                    if open.direction != 0 && direction != open.direction {
                        open.reversals += 1;
                    }
                    open.direction = direction;
                }
                open.previous = present;
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
        if span_ns < self.min_hold_ns || open.samples < self.min_samples {
            self.counts.discarded_short += 1;
            return;
        }
        let seconds = Duration::from_nanos(span_ns.unsigned_abs()).as_secs_f64();
        let reversals_per_s = if seconds > 0.0 {
            open.reversals as f64 / seconds
        } else {
            0.0
        };
        self.counts.judged += 1;
        out.push(HoldWindow {
            joint,
            start_ns: open.start_ns,
            end_ns: open.last_ns,
            samples: open.samples,
            excursion_rad: open.highest - open.lowest,
            reversals_per_s,
            mean_error_rad: open.error_sum / open.samples as f64,
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
    fn holding(seconds: f64, goal: f64, present: impl Fn(usize) -> f64) -> Vec<Tick> {
        (0..cycles(seconds))
            .map(|index| Tick::held(present(index), goal))
            .collect()
    }

    /// `seconds` of a joint sitting exactly on `goal`.
    fn still(seconds: f64, goal: f64) -> Vec<Tick> {
        holding(seconds, goal, |_| goal)
    }

    /// Feed a series to a watch on `joints` and close it out.
    fn watch(
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
                    t_ns: index as i64 * PERIOD_NS,
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
}
