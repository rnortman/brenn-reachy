//! The measurements the replay guards take over a recorded run.
//!
//! Test support for `replay_test.rs` and nothing else. Reading the file is the
//! library's ([`reachy_motion::trace`]), because the offline judge reads the
//! same CSVs; what is here is the handful of figures only the guards ask for —
//! how far a joint travelled, how fast, how long it stood still — taken over
//! the library's own periods.
//!
//! The two cautions about what a recorded series is are in the parser's header,
//! and they apply to every figure below.

use core::time::Duration;

use brenn_reachy__motion__joints_clk_rs::JointFlags;

use reachy_motion::stillness::Sample as StillnessSample;
use reachy_motion::trace::{Run, Sample, Trace};
use reachy_motion::{
    HoldWindow, JointRef, JointVector, PhaseSeparation, PhaseWatch, StillnessConfig, StillnessWatch,
};

/// How near its final goal a joint must be to count as having arrived, radians.
///
/// The figure the live release measures stow against, so a run measured off its
/// trace and the same run measured as it happened agree about where the machine
/// got to.
pub const ARRIVED_TOLERANCE_RAD: f64 = reachy_motion::disarm::DEFAULT_STOW_TOLERANCE;

/// The variable naming the directory the recordings arrive in.
///
/// The test target sets it beside the `data` attribute that puts the files in
/// runfiles, so the two halves stay in one place; its value is relative to the
/// runfiles root, which is a test's working directory.
const TRACE_FIXTURES_ENV: &str = "REACHY_MOTION_TRACE_FIXTURES";

/// The recording checked in under `name`.
///
/// Panics rather than answers: neither a missing fixture nor a missing
/// environment is a test case.
///
/// `//cogs:trace_export` cuts a window of a log into a file this reads, and
/// what such a file must carry is what the parser refuses to guess at. A hold
/// fixture is cut from before the raise's last goal write, so the shipped
/// settle allowance opens the window inside it.
pub fn fixture(name: &str) -> Trace {
    let dir = std::env::var(TRACE_FIXTURES_ENV).unwrap_or_else(|_| {
        panic!(
            "{TRACE_FIXTURES_ENV} is unset: the test target has to name the trace fixture \
             directory beside the data attribute that supplies it"
        )
    });
    let path = std::path::PathBuf::from(dir).join(format!("{name}.csv"));
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
    Trace::parse(&text)
}

/// The figures the guards take over one recorded run.
///
/// An extension trait because the run itself is the library's type: the parser
/// belongs to the crate under test and these measurements belong to the suite,
/// and a guard reads them off the run it is holding either way.
pub trait RunMetrics {
    /// How far from mirrored the antennas stood when the second of them
    /// reached the contact band, over one of the run's two series.
    fn separation(
        &self,
        contact_band_rad: f64,
        cell: fn(&Sample, JointRef) -> Option<f64>,
    ) -> Option<PhaseSeparation>;

    /// The holds `joints` showed over this run, as the live stillness watch
    /// would have cut them.
    fn holds(&self, cfg: StillnessConfig, joints: &[JointRef]) -> Vec<HoldWindow>;

    /// What `joint` did over this run.
    fn joint(&self, joint: JointRef) -> JointMetrics;

    /// The longest stretch `joint` stood still for, within `still` radians of
    /// where the stretch began.
    fn longest_stall(&self, joint: JointRef, still: f64) -> Option<Stall>;

    /// The last period still commanding, or `None` if none was.
    fn commanding_end(&self) -> Option<Duration>;

    /// One joint's series of whatever `read` answers for, with the periods it
    /// answers `None` for left out.
    fn series(
        &self,
        joint: JointRef,
        read: fn(&Sample, JointRef) -> Option<f64>,
    ) -> Vec<(Duration, f64)>;
}

impl RunMetrics for Run {
    /// How far from mirrored the antennas stood when the second of them reached
    /// the contact band, over one of the run's two series.
    ///
    /// The same measurement the resolver holds a planned pair to, taken over a
    /// recording: [`Sample::goal_of`] reads what was commanded, which is what a
    /// clock pair can be judged by, and [`Sample::present_of`] reads where the
    /// tips actually were, which is what a collision is decided by. A period
    /// missing either cell is skipped rather than guessed at.
    ///
    /// `None` for a run that does not carry both antennas across the band's
    /// edge, a pair jammed inside it included.
    fn separation(
        &self,
        contact_band_rad: f64,
        cell: fn(&Sample, JointRef) -> Option<f64>,
    ) -> Option<PhaseSeparation> {
        let mut watch = PhaseWatch::new(contact_band_rad);
        for sample in &self.samples {
            if let (Some(right), Some(left)) = (
                cell(sample, JointRef::AntennaRight),
                cell(sample, JointRef::AntennaLeft),
            ) {
                watch.look(sample.at, [right, left]);
            }
        }
        watch.separation()
    }

    /// The holds `joints` showed over this run, as the live stillness watch
    /// would have cut them.
    ///
    /// The shipped watch, driven period by period out of the file exactly as
    /// the report drives it off the pose stream, so a window measured here is
    /// the window the machine reported. A period whose grouped read fell short
    /// is fed as an invalid reading rather than skipped — a gap is not a goal
    /// change and the watch is the one that decides that — and a period
    /// holding no goal for one of the watched joints is fed as a driver
    /// holding nothing, which closes the window as it did on the machine.
    fn holds(&self, cfg: StillnessConfig, joints: &[JointRef]) -> Vec<HoldWindow> {
        let mut watch = StillnessWatch::new(cfg, joints);
        let mut windows = Vec::new();
        for sample in &self.samples {
            let mut commanded = JointVector::default();
            let mut commanded_valid = true;
            for joint in joints {
                match sample.goal_of(*joint) {
                    Some(goal) => {
                        commanded.set(*joint, goal);
                    }
                    None => commanded_valid = false,
                }
            }
            let present = sample.present.unwrap_or_default();
            watch.look(
                &StillnessSample {
                    t_ns: i64::try_from(sample.at.as_nanos()).expect("a trace within an epoch"),
                    present_valid: sample.present.is_some(),
                    commanded_valid,
                    missing: JointFlags::NONE,
                    present: &present,
                    commanded: &commanded,
                },
                &mut windows,
            );
        }
        watch.finish(&mut windows);
        windows
    }

    fn commanding_end(&self) -> Option<Duration> {
        self.samples
            .iter()
            .filter(|sample| !sample.settling)
            .map(|sample| sample.at)
            .max()
    }

    fn series(
        &self,
        joint: JointRef,
        read: fn(&Sample, JointRef) -> Option<f64>,
    ) -> Vec<(Duration, f64)> {
        self.samples
            .iter()
            .filter_map(|sample| read(sample, joint).map(|angle| (sample.at, angle)))
            .collect()
    }

    /// What `joint` did over this run.
    fn joint(&self, joint: JointRef) -> JointMetrics {
        let measured = self.series(joint, Sample::present_of);
        let commanded = self.series(joint, Sample::goal_of);
        // A joint taken out of service part way through is not measured against
        // the goal it was abandoned at: nothing is writing it, the servo is
        // limp, and it has neither arrived nor failed to.
        let released = commanded.len() < self.samples.len();
        let final_goal = if released {
            None
        } else {
            commanded.last().map(|(_, goal)| *goal)
        };
        JointMetrics {
            span: span(&measured),
            peak_speed: peak_rate(&measured),
            peak_goal_speed: peak_rate(&commanded),
            peak_goal_step: peak_step(&commanded),
            worst_lag: self
                .samples
                .iter()
                .filter_map(|sample| {
                    let present = sample.present_of(joint)?;
                    let goal = sample.goal_of(joint)?;
                    Some((goal - present).abs())
                })
                .fold(0.0, f64::max),
            arrived: final_goal.and_then(|goal| arrival(&measured, goal)),
            residual: final_goal
                .zip(measured.last())
                .map(|(goal, (_, at))| (at - goal).abs()),
        }
    }

    /// The longest stretch `joint` stood still for, within `still` radians of
    /// where the stretch began, or `None` if it never stood still for two
    /// periods running.
    ///
    /// What a jam looks like from the outside: the goal walks away and the
    /// measurement does not follow. Ties keep the earliest stretch, and a
    /// period whose read fell short ends the one it is in — a joint nobody
    /// measured is not a joint observed holding still. What this does *not*
    /// decide is whether standing still is a fault: a joint parked at the goal
    /// it arrived at also stands still, and [`Stall::worst_lag`] is what tells
    /// the two apart.
    fn longest_stall(&self, joint: JointRef, still: f64) -> Option<Stall> {
        let mut longest: Option<Stall> = None;
        let mut open: Option<Stall> = None;
        for sample in &self.samples {
            let Some(present) = sample.present_of(joint) else {
                open = None;
                continue;
            };
            let lag = sample
                .goal_of(joint)
                .map_or(0.0, |goal| (goal - present).abs());
            let stall = match open {
                Some(stall) if (present - stall.at).abs() <= still => Stall {
                    periods: stall.periods + 1,
                    worst_lag: stall.worst_lag.max(lag),
                    ..stall
                },
                _ => Stall {
                    at: present,
                    periods: 1,
                    worst_lag: lag,
                },
            };
            open = Some(stall);
            // Two periods is the shortest thing worth calling a stretch: every
            // joint stands where it stands for the one period it was read on.
            if stall.periods > longest.map_or(1, |best: Stall| best.periods) {
                longest = Some(stall);
            }
        }
        longest
    }
}

/// A stretch of periods one joint spent standing still.
#[derive(Clone, Copy)]
pub struct Stall {
    /// Where it stood, radians — the first period's measurement.
    pub at: f64,
    /// How many periods that is, the first included.
    pub periods: usize,
    /// The furthest the goal got from it while it stood there, radians.
    pub worst_lag: f64,
}

/// What one joint did over one run.
pub struct JointMetrics {
    /// How far it travelled, radians: the measured extremes.
    pub span: f64,
    /// Its fastest measured period, radians per second.
    pub peak_speed: f64,
    /// The fastest its goal ever moved, radians per second — the commanded
    /// speed a lag is read against, since what a joint sits behind by is set by
    /// how fast it is being asked to move.
    pub peak_goal_speed: f64,
    /// The largest single-period change in the goal, radians — the recorded
    /// command, inflated by whatever lateness the loop had that night. Read the
    /// module's caution before sizing anything against it.
    pub peak_goal_step: f64,
    /// The furthest it was ever behind its goal, radians.
    pub worst_lag: f64,
    /// When it first came within tolerance of its final goal and stayed, or
    /// `None` if it never did.
    pub arrived: Option<Duration>,
    /// How far from the final goal it finished, radians, or `None` if it was
    /// holding no goal at the end.
    pub residual: Option<f64>,
}

/// How far a series travelled: its extremes, and zero for an empty one.
fn span(series: &[(Duration, f64)]) -> f64 {
    let (mut low, mut high) = match series.first() {
        Some((_, first)) => (*first, *first),
        None => return 0.0,
    };
    for (_, angle) in series {
        low = low.min(*angle);
        high = high.max(*angle);
    }
    high - low
}

/// The fastest a series ever moved, radians per second.
///
/// Central differences, one-sided at the ends: the rate over a period is the
/// only rate this data holds, and a one-sided difference at an endpoint reads
/// the same motion over half the window rather than inventing a value for it.
fn peak_rate(series: &[(Duration, f64)]) -> f64 {
    let mut peak: f64 = 0.0;
    for index in 0..series.len() {
        let (before, angle_before) = series[index.saturating_sub(1)];
        let (after, angle_after) = series[(index + 1).min(series.len() - 1)];
        let window = after.saturating_sub(before).as_secs_f64();
        if window > 0.0 {
            peak = peak.max(((angle_after - angle_before) / window).abs());
        }
    }
    peak
}

/// The largest single-period change in a series, radians.
fn peak_step(series: &[(Duration, f64)]) -> f64 {
    series
        .windows(2)
        .map(|pair| (pair[1].1 - pair[0].1).abs())
        .fold(0.0, f64::max)
}

/// When a series first came within tolerance of `goal` and stayed there.
///
/// "And stayed" is the whole definition: a joint sweeping through its final
/// angle on the way past is not a joint that arrived, and a run that overshoots
/// and comes back arrived on the way back.
fn arrival(series: &[(Duration, f64)], goal: f64) -> Option<Duration> {
    let mut arrived = None;
    for (at, angle) in series {
        if (angle - goal).abs() <= ARRIVED_TOLERANCE_RAD {
            arrived = arrived.or(Some(*at));
        } else {
            arrived = None;
        }
    }
    arrived
}
