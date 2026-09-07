//! Reading a recorded run back off its CSV, and the few measurements the
//! replay guards take over it.
//!
//! Test support for `replay_test.rs` and nothing else: the recordings beside
//! this crate are the measurements its bounds are sized against, and this is
//! the least code that turns one of those files into the series the shipped
//! functions are driven over. A malformed file panics naming its line — a
//! fixture that no longer parses is a broken checkout, not a test case.
//!
//! Two cautions about what a recorded series is, carried from the loop that
//! wrote these files.
//!
//! - **A goal column is what the loop commanded, not what it planned.** These
//!   recordings predate the per-period move clock, so a period that started
//!   late sampled the trajectory further along and commanded a correspondingly
//!   larger step. The recorded step is an upper bound inflated by whatever the
//!   scheduler did that night, and nothing may be sized against it; it is
//!   measured because the inflation is itself a measurement.
//! - **A measured column is the servo's own encoder**, at the rate the loop
//!   read it. Speeds here are differences of that series, so they are averages
//!   over a period rather than instantaneous rates.

use core::time::Duration;
use std::path::PathBuf;

use brenn_reachy__motion__joints_clk_rs::JointFlags;
use reachy_motion::joints::{ROWS, flags};
use reachy_motion::{JointRef, JointVector, PhaseSeparation, PhaseWatch};

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
/// TODO(antenna-hold-fixture): the antenna hold has no recording here yet, so
/// nothing replays the stillness watch over a machine. `//cogs:trace_export`
/// cuts a window of a log into a file this reads; what such a file must carry
/// is what the parser here refuses to guess at: the header is
/// `run,tick,t_s,phase,<joint>_present_rad…,<joint>_goal_rad…`, the `phase`
/// cell is exactly `commanding` or `settling` and panics otherwise, present
/// cells are all nine or all blank, and a joint holding no goal has a blank
/// goal cell rather than a zero.
pub fn fixture(name: &str) -> Trace {
    let dir = std::env::var(TRACE_FIXTURES_ENV).unwrap_or_else(|_| {
        panic!(
            "{TRACE_FIXTURES_ENV} is unset: the test target has to name the trace fixture \
             directory beside the data attribute that supplies it"
        )
    });
    let path = PathBuf::from(dir).join(format!("{name}.csv"));
    let text = std::fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("{}: {error}", path.display()));
    Trace::parse(&text)
}

/// One period, as the file recorded it.
///
/// The measured half is all nine angles or none — a grouped read either came
/// back or fell short. The commanded half is per joint, because a servo taken
/// out of service is commanded nothing while the rest of the machine is still
/// being driven.
pub struct Sample {
    /// Periods since the run's first, counted from zero.
    pub tick: u64,
    /// When the period began, on the run's own epoch.
    pub at: Duration,
    /// Whether commanding had finished and the period was one of those spent
    /// waiting for the machine to arrive.
    pub settling: bool,
    /// The nine measured angles, or `None` for a period whose grouped read fell
    /// short.
    pub present: Option<JointVector>,
    /// The goal each joint was being held to, in bus order, and `None` for a
    /// joint holding none.
    goal: [Option<f64>; ROWS.len()],
}

impl Sample {
    /// The goal `joint` was being held to, or `None` if it was commanded
    /// nothing.
    pub fn goal_of(&self, joint: JointRef) -> Option<f64> {
        reachy_motion::joints::row(joint).and_then(|row| self.goal[row])
    }

    /// What `joint` measured, or `None` if the period's read fell short.
    pub fn present_of(&self, joint: JointRef) -> Option<f64> {
        self.present.and_then(|present| present.get(joint))
    }

    /// The joints that were holding no goal — torqued off and out of service.
    ///
    /// What the live loop hands the tracking comparison as its mask: a joint
    /// nothing is writing is behind a goal it never had.
    pub fn released(&self) -> JointFlags {
        let mut released = JointFlags::NONE;
        for joint in ROWS {
            if self.goal_of(joint).is_none() {
                flags::insert(&mut released, joint);
            }
        }
        released
    }
}

/// One move, as the file recorded it.
pub struct Run {
    /// Its periods, in the order they were recorded.
    pub samples: Vec<Sample>,
}

/// A trace file, split into the runs it holds.
pub struct Trace {
    runs: Vec<Run>,
}

impl Trace {
    /// Read a trace out of `text`, panicking on anything that is not a period.
    ///
    /// Every refusal is a panic naming the line: a trace exists to have guard
    /// values drawn from it, and skipping an unreadable row would silently drop
    /// a period out of every series taken from the file.
    fn parse(text: &str) -> Self {
        let mut lines = text
            .lines()
            .enumerate()
            .map(|(index, line)| (index + 1, line))
            .filter(|(_, line)| !line.trim().is_empty());
        let (_, header) = lines.next().expect("the trace has a header row");
        let columns = Columns::resolve(header);

        let mut runs: Vec<Run> = Vec::new();
        for (line, row) in lines {
            let sample = columns.sample(line, row);
            // A period continues the run when it is later than the last. The
            // tick counter rather than the `run` column: these files carry
            // `run = 0` throughout, and a counter that fails to advance is a
            // fresh move either way.
            match runs.last_mut() {
                Some(last)
                    if last
                        .samples
                        .last()
                        .is_none_or(|previous| sample.tick > previous.tick) =>
                {
                    last.samples.push(sample);
                }
                _ => runs.push(Run {
                    samples: vec![sample],
                }),
            }
        }
        Self { runs }
    }

    /// How many runs the file holds.
    pub fn runs(&self) -> usize {
        self.runs.len()
    }

    /// The run at `index`, counting the file's runs from zero — which is not the
    /// number in the `run` column.
    pub fn run(&self, index: usize) -> &Run {
        self.runs
            .get(index)
            .unwrap_or_else(|| panic!("the trace holds a run {index}"))
    }
}

impl Run {
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
    pub fn separation(
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

    /// The grid this run was driven on: the median time one period took,
    /// nanoseconds.
    ///
    /// A recording's own grid and not the deployment's. The bench loops that
    /// wrote the older fixtures here ran at 32 ms and 24 ms a period against
    /// the 20 ms the machine ships, and the servo's trajectory generator is
    /// stepped in periods: a model built at one period and driven over a
    /// recording made at another is slow or fast by the ratio, and on fast
    /// content that shows up as radians of residual that were never on the
    /// machine. The median rather than the mean, because a period the loop
    /// overslept and the period after it are both in the series.
    ///
    /// Panics on a run of one period, which carries no grid at all.
    pub fn period_ns(&self) -> i64 {
        let mut periods: Vec<i64> = self
            .samples
            .windows(2)
            .filter_map(|pair| {
                let slots = pair[1].tick.checked_sub(pair[0].tick)?;
                let elapsed = pair[1].at.checked_sub(pair[0].at)?;
                i64::try_from(elapsed.as_nanos() / u128::from(slots.max(1))).ok()
            })
            .collect();
        assert!(
            !periods.is_empty(),
            "a run of one period says nothing about the grid it was driven on"
        );
        periods.sort_unstable();
        periods[periods.len() / 2]
    }

    /// The last period's timestamp.
    pub fn span(&self) -> Duration {
        self.samples.last().map_or(Duration::ZERO, |last| last.at)
    }

    /// The last period still commanding, or `None` if none was.
    pub fn commanding_end(&self) -> Option<Duration> {
        self.samples
            .iter()
            .filter(|sample| !sample.settling)
            .map(|sample| sample.at)
            .max()
    }

    /// What `joint` did over this run.
    pub fn joint(&self, joint: JointRef) -> JointMetrics {
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
    pub fn longest_stall(&self, joint: JointRef, still: f64) -> Option<Stall> {
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

    /// One joint's series of whatever `read` answers for, with the periods it
    /// answers `None` for left out.
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

/// Where each column stands, resolved from the header once.
struct Columns {
    tick: usize,
    at: usize,
    phase: usize,
    /// Measured columns, in bus order.
    present: [usize; ROWS.len()],
    /// Commanded columns, in bus order.
    goal: [usize; ROWS.len()],
    /// How many columns the header names.
    width: usize,
}

impl Columns {
    /// Find every column the guards need in `header`.
    fn resolve(header: &str) -> Self {
        let headings: Vec<&str> = header.split(',').map(str::trim).collect();
        let find = |heading: &str| {
            headings
                .iter()
                .position(|candidate| *candidate == heading)
                .unwrap_or_else(|| panic!("the trace header names no `{heading}` column"))
        };
        let mut present = [0; ROWS.len()];
        let mut goal = [0; ROWS.len()];
        for (row, joint) in ROWS.into_iter().enumerate() {
            present[row] = find(&format!("{}_present_rad", column(joint)));
            goal[row] = find(&format!("{}_goal_rad", column(joint)));
        }
        Self {
            tick: find("tick"),
            at: find("t_s"),
            phase: find("phase"),
            present,
            goal,
            width: headings.len(),
        }
    }

    /// One row as the period it records.
    fn sample(&self, line: usize, row: &str) -> Sample {
        let cells: Vec<&str> = row.split(',').map(str::trim).collect();
        assert_eq!(
            cells.len(),
            self.width,
            "line {line} has {} cells; the header names {}",
            cells.len(),
            self.width
        );
        let tick = cells[self.tick]
            .parse()
            .unwrap_or_else(|_| panic!("line {line}: `{}` is no period", cells[self.tick]));
        let at = cells[self.at]
            .parse::<f64>()
            .ok()
            .and_then(|secs| Duration::try_from_secs_f64(secs).ok())
            .unwrap_or_else(|| {
                panic!(
                    "line {line}: `{}` is not seconds since the run began",
                    cells[self.at]
                )
            });
        let settling = match cells[self.phase] {
            "settling" => true,
            "commanding" => false,
            cell => panic!("line {line}: `{cell}` is neither `commanding` nor `settling`"),
        };

        // All nine measured cells or none: the writer blanks a period whose
        // grouped read fell short, and reading a mixture as absent
        // measurements would invent a period the machine never had.
        let measured = self
            .present
            .iter()
            .filter(|column| !cells[**column].is_empty())
            .count();
        let present = match measured {
            0 => None,
            count if count == ROWS.len() => {
                let mut angles = JointVector::default();
                for (row, joint) in ROWS.into_iter().enumerate() {
                    angles.set(joint, angle(line, cells[self.present[row]]));
                }
                Some(angles)
            }
            count => panic!(
                "line {line} measures {count} of {} joints; a period read all of them or none",
                ROWS.len()
            ),
        };

        let mut goal = [None; ROWS.len()];
        for (row, cell) in self.goal.iter().enumerate() {
            let cell = cells[*cell];
            if !cell.is_empty() {
                goal[row] = Some(angle(line, cell));
            }
        }

        Sample {
            tick,
            at,
            settling,
            present,
            goal,
        }
    }
}

/// The prefix a joint's two columns are written under: the library's own, so
/// the fixtures and whatever writes them name a crank the same way.
fn column(joint: JointRef) -> &'static str {
    reachy_motion::joints::column_name(joint).expect("the nine bus rows each name a column")
}

/// A cell holding an angle in radians.
fn angle(line: usize, cell: &str) -> f64 {
    match cell.parse::<f64>() {
        Ok(angle) if angle.is_finite() => angle,
        _ => panic!("line {line}: `{cell}` is no angle in radians"),
    }
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

#[cfg(test)]
mod tests {
    //! What a recorded run's grid reads as.
    //!
    //! [`Run::period_ns`] is the one measurement in this module that every
    //! residual pin in the replay suite flows through: the plant a recording is
    //! judged against is built at the period this answers, so an error here
    //! moves all four bench figures at once and in one direction — the shape a
    //! reader is most likely to accept and re-bake under the bring-up rule.
    //! Hence cases of its own, over traces small enough to read, rather than
    //! only the implicit exercise the checked-in 20 ms fixtures give it.

    use core::time::Duration;

    use reachy_motion::joints::ROWS;

    use super::{Run, Sample, Trace, column};

    /// A trace holding one run of `periods`, each named by its grid slot and
    /// the instant it began.
    ///
    /// Every angle cell is zero: what these cases measure is the grid, and the
    /// two columns a period's timing lives in are `tick` and `t_s`.
    fn trace(periods: &[(u64, f64)]) -> Trace {
        let mut text = String::from("run,tick,t_s,phase");
        for joint in ROWS {
            text.push_str(&format!(",{}_present_rad", column(joint)));
        }
        for joint in ROWS {
            text.push_str(&format!(",{}_goal_rad", column(joint)));
        }
        text.push('\n');
        for (tick, secs) in periods {
            text.push_str(&format!("0,{tick},{secs:.6},commanding"));
            for _ in 0..2 * ROWS.len() {
                text.push_str(",0.000000");
            }
            text.push('\n');
        }
        Trace::parse(&text)
    }

    /// The grid is the median period and not the mean: one period the loop
    /// overslept does not move it.
    ///
    /// The median is chosen for exactly this, and the choice is what the case
    /// is about. An overslept period lands in the series twice over — long
    /// once, and the period after it is measured from the late instant — so a
    /// mean drags toward the outlier while every other period says 20 ms.
    #[test]
    fn the_grid_is_the_median_period_and_not_the_mean() {
        // Six periods on a 20 ms grid, with the fourth two hundred
        // milliseconds late: the loop lost the CPU and caught up.
        let trace = trace(&[
            (0, 0.00),
            (1, 0.02),
            (2, 0.04),
            (3, 0.24),
            (4, 0.26),
            (5, 0.28),
        ]);
        let run = trace.run(0);
        assert_eq!(run.period_ns(), 20_000_000);
        let mean: i64 = 280_000_000 / 5;
        assert!(
            mean > 20_000_000,
            "the mean of this series is {mean} ns, so a mean would have read the outlier"
        );
    }

    /// A slot the driver dropped is divided over the slots it covers, so a hole
    /// in the grid reads as the grid it is a hole in.
    ///
    /// What the elapsed time between two samples measures is the periods
    /// between their slots, not one period: a fixture cut across a dropped read
    /// carries a forty-millisecond step over two slots, and a series that took
    /// that for a period would read the grid at twice its rate and build a
    /// plant that moves twice as far per period as the servo did.
    #[test]
    fn a_dropped_period_is_divided_over_the_slots_it_covers() {
        let trace = trace(&[(0, 0.00), (1, 0.02), (3, 0.06), (4, 0.08)]);
        assert_eq!(trace.run(0).period_ns(), 20_000_000);
    }

    /// Two readings on one slot are one period, not a period of no length.
    ///
    /// Not a shape any fixture in the tree carries — the parser reads a period
    /// that fails to advance as a fresh run — so the run is built here
    /// directly. What it guards is the division: a slot difference of zero
    /// divided into an elapsed time is what would make a grid of nanoseconds
    /// out of one duplicated sample.
    #[test]
    fn two_readings_on_one_slot_are_one_period() {
        let run = Run {
            samples: vec![
                Sample {
                    tick: 0,
                    at: Duration::ZERO,
                    settling: false,
                    present: None,
                    goal: [None; ROWS.len()],
                },
                Sample {
                    tick: 0,
                    at: Duration::from_millis(20),
                    settling: false,
                    present: None,
                    goal: [None; ROWS.len()],
                },
            ],
        };
        assert_eq!(run.period_ns(), 20_000_000);
    }

    /// A run of one period carries no grid at all, and says so rather than
    /// answering.
    ///
    /// The alternative is a plant built at a period nobody measured, which is
    /// the one failure this figure must not have: it would be wrong by a ratio
    /// and read as residual.
    #[test]
    #[should_panic(expected = "says nothing about the grid")]
    fn a_run_of_one_period_has_no_grid() {
        let trace = trace(&[(0, 0.00)]);
        let _ = trace.run(0).period_ns();
    }
}
