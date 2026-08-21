//! Reading a trace back, and measuring the run it recorded.
//!
//! The inverse of the writer beside it, plus the arithmetic a reader has to do
//! before a CSV says anything: differences for velocity, goal against measured
//! for lag, the first period a joint was measurably at its final goal and
//! stayed.
//!
//! It is here rather than in a tool because of what consumes it: recorded runs
//! are checked into this crate as fixtures, and the guards this repo ships are
//! sized against them by tests. A measurement that decides a step bound is
//! product code.
//!
//! Two cautions about what a recorded series is.
//!
//! - **A goal column is what the loop commanded, not what it planned.** These
//!   recordings predate the tick's per-period move clock, so a period that
//!   started late sampled the trajectory further along and commanded a
//!   correspondingly larger step. The recorded step is therefore an upper bound
//!   inflated by whatever the scheduler did that night, and nothing may be
//!   sized against it; it is reported because the inflation is itself a
//!   measurement.
//! - **A measured column is the servo's own encoder**, at the rate the loop
//!   read it. Velocities here are differences of that series, so they are
//!   averages over a period rather than instantaneous rates.
//!
//! Not in the build: this file is part of the bench's retired motion layer, kept
//! on disk as the record of how this machine was driven. It no longer compiles.
//! TODO(bench-motion-delete)

use core::time::Duration;
use std::fs;
use std::path::{Path, PathBuf};

use reachy_motion::{JointId, JointSet, JointVector, PhaseSeparation, PhaseWatch, mirror_offset};
use thiserror::Error;

use crate::config::{ARRIVED_TOLERANCE_DEG, ONE_COUNT_RAD};

/// How near its final goal a joint must be to count as having arrived, radians.
///
/// The figure the live settle watch waits on, so a run measured from its trace
/// and the same run measured as it happened agree about when the machine got
/// there.
const ARRIVED_TOLERANCE_RAD: f64 = ARRIVED_TOLERANCE_DEG * core::f64::consts::PI / 180.0;

/// How far a joint must have travelled — measured or commanded — before it is
/// reported at all, radians.
///
/// Six encoder counts. Below that a span is read noise on a joint that was
/// holding still, and a table that reported it would put a peak velocity beside
/// every stationary servo in the machine.
const MOVED_RAD: f64 = 6.0 * ONE_COUNT_RAD;

/// Why a trace cannot be read.
///
/// Every arm is a refusal. A trace exists to have guard values drawn from it,
/// and a file with a row nobody can read is not a measurement — skipping the
/// row would silently drop a period out of a velocity series.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum TraceError {
    /// The file has no rows at all, not even a header.
    #[error("the trace is empty")]
    Empty,

    /// The header names no column something needs.
    #[error("the trace header names no `{heading}` column")]
    MissingColumn {
        /// The heading that is not there.
        heading: String,
    },

    /// A row has a different number of cells than the header has columns.
    #[error("line {line} has {cells} cells; the header names {columns}")]
    WrongWidth {
        /// 1-based line number in the file.
        line: usize,
        /// How many cells the row has.
        cells: usize,
        /// How many the header names.
        columns: usize,
    },

    /// A cell does not hold what its column means.
    #[error("line {line}, column `{heading}`: `{cell}` is not {want}")]
    BadCell {
        /// 1-based line number in the file.
        line: usize,
        /// The column's heading.
        heading: String,
        /// The cell as it stands in the file.
        cell: String,
        /// What the column holds.
        want: &'static str,
    },

    /// A period measured some joints and not others.
    ///
    /// The writer blanks all nine measured cells or none: a grouped read either
    /// came back or fell short. A row with a mixture came from something else,
    /// and treating the blanks as absent measurements would invent a period the
    /// machine never had.
    #[error("line {line} measures {measured} of {count} joints; a period read all of them or none")]
    HalfBlind {
        /// 1-based line number in the file.
        line: usize,
        /// How many measured cells carry a number.
        measured: usize,
        /// How many joints there are.
        count: usize,
    },
}

/// Why a trace file cannot be read, with the path that could not be read.
#[derive(Debug, Error)]
pub enum ReadError {
    /// The file could not be opened or read.
    #[error("{}: {source}", path.display())]
    Unreadable {
        /// The file asked for.
        path: PathBuf,
        /// What the filesystem said.
        #[source]
        source: std::io::Error,
    },

    /// The file was read and is not a trace.
    #[error("{}: {source}", path.display())]
    Malformed {
        /// The file asked for.
        path: PathBuf,
        /// What is wrong with it.
        #[source]
        source: TraceError,
    },
}

/// One period, as the file recorded it.
///
/// The measured half is all nine angles or none. The commanded half is per
/// joint, because a servo taken out of service is commanded nothing while the
/// rest of the machine is still being driven.
#[derive(Clone, Debug, PartialEq)]
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
    /// joint that was holding none.
    goal: [Option<f64>; JointId::COUNT],
}

impl Sample {
    /// The goal `joint` was being held to, or `None` if it was commanded
    /// nothing.
    #[must_use]
    pub fn goal_of(&self, joint: JointId) -> Option<f64> {
        joint.index().and_then(|index| self.goal[index])
    }

    /// What `joint` measured, or `None` if the period's read fell short.
    #[must_use]
    pub fn present_of(&self, joint: JointId) -> Option<f64> {
        self.present.and_then(|present| present.get(joint))
    }

    /// The joints that were holding no goal — torqued off and out of service.
    #[must_use]
    pub fn released(&self) -> JointSet {
        let mut released = JointSet::EMPTY;
        for joint in JointId::ALL {
            if self.goal_of(joint).is_none() {
                released.insert(joint);
            }
        }
        released
    }
}

/// One move, as the file recorded it.
#[derive(Clone, Debug, PartialEq)]
pub struct Run {
    /// The number the run was written under.
    pub run: u64,
    /// Its periods, in the order they were recorded.
    pub samples: Vec<Sample>,
}

/// A trace file, split into the runs it holds.
#[derive(Clone, Debug, PartialEq)]
pub struct Trace {
    runs: Vec<Run>,
}

impl Trace {
    /// Read the file at `path`.
    ///
    /// # Errors
    ///
    /// [`ReadError`] naming the path, whether the filesystem or the contents
    /// refused it.
    pub fn read(path: &Path) -> Result<Self, ReadError> {
        let text = fs::read_to_string(path).map_err(|source| ReadError::Unreadable {
            path: path.to_path_buf(),
            source,
        })?;
        Self::parse(&text).map_err(|source| ReadError::Malformed {
            path: path.to_path_buf(),
            source,
        })
    }

    /// Read a trace out of `text`.
    ///
    /// # Errors
    ///
    /// [`TraceError`] for a header that names no column something needs, or any
    /// row that is not a period.
    pub fn parse(text: &str) -> Result<Self, TraceError> {
        let mut lines = text
            .lines()
            .enumerate()
            .map(|(index, line)| (index + 1, line))
            .filter(|(_, line)| !line.trim().is_empty());
        let (_, header) = lines.next().ok_or(TraceError::Empty)?;
        let columns = Columns::resolve(header)?;

        let mut runs: Vec<Run> = Vec::new();
        for (line, row) in lines {
            let (run, sample) = columns.sample(line, row)?;
            match runs.last_mut() {
                Some(last) if last.run == run && last.follows(&sample) => {
                    last.samples.push(sample);
                }
                _ => runs.push(Run {
                    run,
                    samples: vec![sample],
                }),
            }
        }
        Ok(Self { runs })
    }

    /// The runs the file holds, in the order they were written.
    #[must_use]
    pub fn runs(&self) -> &[Run] {
        &self.runs
    }

    /// The run at `index`, counting the file's runs from zero — which is not
    /// the number in the `run` column.
    #[must_use]
    pub fn run(&self, index: usize) -> Option<&Run> {
        self.runs.get(index)
    }
}

impl Run {
    /// Whether `next` continues this run rather than starting a new one.
    ///
    /// A period continues the run when it is later in it than the last one
    /// recorded. The tick count is the test rather than the `run` column alone
    /// because the column cannot always tell: runs written by the per-caller
    /// counter this crate used to keep all carry `run = 0`, and the recordings
    /// checked in as fixtures are from that era. A period counter that fails to
    /// advance is a fresh move either way.
    fn follows(&self, next: &Sample) -> bool {
        self.samples.last().is_none_or(|last| next.tick > last.tick)
    }

    /// How far from mirrored the antennas stood when the second of them reached
    /// the contact band, over one of the run's two series.
    ///
    /// The same measurement the resolver holds a planned pair to, taken over a
    /// recording: `Sample::goal_of` reads what was commanded, which is what a
    /// clock pair can be judged by, and `Sample::present_of` reads where the
    /// tips actually were, which is what a collision is decided by. A period
    /// missing either cell — a read that did not arrive, an antenna out of
    /// service — is skipped rather than guessed at.
    ///
    /// The band is the configured one, so a recording is read against the same
    /// geometry the live resolver measures against.
    ///
    /// `None` for a run that does not carry both antennas across the band's
    /// edge, a pair jammed inside it included.
    #[must_use]
    pub fn separation(
        &self,
        contact_band_rad: f64,
        cell: fn(&Sample, JointId) -> Option<f64>,
    ) -> Option<PhaseSeparation> {
        let mut watch = PhaseWatch::new(contact_band_rad);
        for sample in &self.samples {
            if let (Some(right), Some(left)) = (
                cell(sample, JointId::AntennaRight),
                cell(sample, JointId::AntennaLeft),
            ) {
                watch.look(sample.at, [right, left]);
            }
        }
        watch.separation()
    }

    /// The widest the pair ever stood from mirrored over one of the run's
    /// series, radians.
    ///
    /// What a clock pair is capable of, as against what it happened to be
    /// showing at the band's edge: a pair whose widest is under the separation
    /// the tips need is one no phasing of that shape ever clears.
    ///
    /// `None` when no period carried both cells — an antenna out of service, a
    /// stretch of dropped reads, a column the writer left blank. A run nothing
    /// was measured over is not a run that stood mirrored throughout, and a
    /// caller asserting an upper bound on this figure has to be able to tell
    /// the two apart.
    #[must_use]
    pub fn widest_offset(&self, cell: fn(&Sample, JointId) -> Option<f64>) -> Option<f64> {
        self.samples
            .iter()
            .filter_map(|sample| {
                Some(mirror_offset(
                    cell(sample, JointId::AntennaRight)?,
                    cell(sample, JointId::AntennaLeft)?,
                ))
            })
            .reduce(f64::max)
    }

    /// Everything this run measured, joint by joint.
    #[must_use]
    pub fn metrics(&self) -> RunMetrics {
        let commanding_end = self
            .samples
            .iter()
            .filter(|sample| !sample.settling)
            .map(|sample| sample.at)
            .max();
        let joints: Vec<JointMetrics> = JointId::ALL
            .into_iter()
            .filter_map(|joint| self.joint_metrics(joint))
            .collect();
        RunMetrics {
            run: self.run,
            periods: self.samples.len(),
            span: self.samples.last().map_or(Duration::ZERO, |last| last.at),
            commanding_end,
            arrival: joints.iter().filter_map(|joint| joint.arrived).max(),
            joints,
        }
    }

    /// What `joint` did, or `None` if it neither moved nor was asked to.
    fn joint_metrics(&self, joint: JointId) -> Option<JointMetrics> {
        let measured = self.series(joint, Sample::present_of);
        let commanded = self.series(joint, Sample::goal_of);
        if span(&measured) < MOVED_RAD && span(&commanded) < MOVED_RAD {
            return None;
        }
        // A joint taken out of service is not measured against the goal it was
        // abandoned at: nothing is writing it, the servo is limp, and it has
        // neither arrived nor failed to.
        let released = commanded.len() < self.samples.len();
        let final_goal = if released {
            None
        } else {
            commanded.last().map(|(_, goal)| *goal)
        };
        Some(JointMetrics {
            joint,
            released,
            span: span(&measured),
            peak_speed: peak_rate(&measured),
            peak_goal_speed: peak_rate(&commanded),
            peak_goal_step: peak_step(&commanded),
            worst_lag: self.worst_lag(joint),
            arrived: final_goal.and_then(|goal| arrival(&measured, goal)),
            residual: final_goal
                .zip(measured.last())
                .map(|(goal, (_, at))| (at - goal).abs()),
        })
    }

    /// One joint's series of whatever `read` answers for, with the periods it
    /// answers `None` for left out.
    fn series(
        &self,
        joint: JointId,
        read: fn(&Sample, JointId) -> Option<f64>,
    ) -> Vec<(Duration, f64)> {
        self.samples
            .iter()
            .filter_map(|sample| read(sample, joint).map(|angle| (sample.at, angle)))
            .collect()
    }

    /// The furthest `joint` was ever from the goal it was being held to.
    ///
    /// Over the periods that have both a measurement and a goal: a period whose
    /// read fell short measured no error, and a joint out of service is behind
    /// a goal nothing is writing.
    fn worst_lag(&self, joint: JointId) -> f64 {
        self.samples
            .iter()
            .filter_map(|sample| {
                let present = sample.present_of(joint)?;
                let goal = sample.goal_of(joint)?;
                Some((goal - present).abs())
            })
            .fold(0.0, f64::max)
    }

    /// The longest stretch `joint` stood still for, within `still` radians of
    /// where the stretch began, or `None` if it never stood still for two
    /// periods running.
    ///
    /// What a jam looks like from the outside: the goal walks away and the
    /// measurement does not follow. Ties keep the earliest stretch, and a
    /// period whose read fell short ends the one it is in — a joint nobody
    /// measured is not a joint observed holding still.
    ///
    /// Note what this does *not* decide: a joint parked at the goal it arrived
    /// at is also standing still. [`Stall::worst_lag`] is what tells the two
    /// apart, and it is the caller's to judge.
    #[must_use]
    pub fn longest_stall(&self, joint: JointId, still: f64) -> Option<Stall> {
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
                    until: sample.at,
                    periods: stall.periods + 1,
                    worst_lag: stall.worst_lag.max(lag),
                    ..stall
                },
                _ => Stall {
                    at: present,
                    from: sample.at,
                    until: sample.at,
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
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Stall {
    /// Where it stood, radians — the first period's measurement.
    pub at: f64,
    /// When it stopped.
    pub from: Duration,
    /// The last period it was still standing there.
    pub until: Duration,
    /// How many periods that is, the first included.
    pub periods: usize,
    /// The furthest the goal got from it while it stood there, radians.
    pub worst_lag: f64,
}

/// What one joint did over one run.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct JointMetrics {
    /// Which joint.
    pub joint: JointId,
    /// Whether it stopped being commanded part way through — released, and so
    /// measured against nothing from there on.
    pub released: bool,
    /// How far it travelled, radians: the measured extremes.
    pub span: f64,
    /// Its fastest measured period, radians per second.
    pub peak_speed: f64,
    /// The fastest the goal moved, radians per second.
    pub peak_goal_speed: f64,
    /// The largest single-period change in the goal, radians.
    ///
    /// The recorded command, inflated by whatever lateness the loop had that
    /// night. Read the module's caution before sizing anything against it.
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

/// What a whole run did.
#[derive(Clone, Debug, PartialEq)]
pub struct RunMetrics {
    /// The number the run was written under.
    pub run: u64,
    /// How many periods it lasted.
    pub periods: usize,
    /// Its last period's timestamp.
    pub span: Duration,
    /// The last period still commanding, or `None` if none was.
    pub commanding_end: Option<Duration>,
    /// When the last joint to arrive arrived, or `None` if one never did.
    pub arrival: Option<Duration>,
    /// Every joint that moved or was asked to, in bus order.
    pub joints: Vec<JointMetrics>,
}

impl RunMetrics {
    /// What `joint` did, or `None` if it neither moved nor was asked to.
    #[must_use]
    pub fn joint(&self, joint: JointId) -> Option<&JointMetrics> {
        self.joints.iter().find(|row| row.joint == joint)
    }

    /// How long after commanding ended the machine finished arriving.
    ///
    /// The whole point of measuring arrival: a move commanded over a clock
    /// shorter than the machine's own response spends this much of its motion
    /// after the last goal went out. `None` when either end is unknown, and
    /// zero when the machine was there as the commanding stopped.
    #[must_use]
    pub fn settle_gap(&self) -> Option<Duration> {
        Some(self.arrival?.saturating_sub(self.commanding_end?))
    }
}

/// Where each column stands, resolved from the header once.
struct Columns {
    run: usize,
    tick: usize,
    at: usize,
    phase: usize,
    /// Measured columns, in bus order.
    present: [usize; JointId::COUNT],
    /// Commanded columns, in bus order.
    goal: [usize; JointId::COUNT],
    /// How many columns the header names.
    width: usize,
}

impl Columns {
    /// Find every column the reader needs in `header`.
    fn resolve(header: &str) -> Result<Self, TraceError> {
        let headings: Vec<&str> = header.split(',').map(str::trim).collect();
        let find = |heading: &str| {
            headings
                .iter()
                .position(|candidate| *candidate == heading)
                .ok_or_else(|| TraceError::MissingColumn {
                    heading: heading.to_string(),
                })
        };
        let mut present = [0; JointId::COUNT];
        let mut goal = [0; JointId::COUNT];
        for (slot, joint) in JointId::ALL.into_iter().enumerate() {
            present[slot] = find(&super::present_heading(joint))?;
            goal[slot] = find(&super::goal_heading(joint))?;
        }
        Ok(Self {
            run: find("run")?,
            tick: find("tick")?,
            at: find("t_s")?,
            phase: find("phase")?,
            present,
            goal,
            width: headings.len(),
        })
    }

    /// One row as the run it belongs to and the period it records.
    fn sample(&self, line: usize, row: &str) -> Result<(u64, Sample), TraceError> {
        let cells: Vec<&str> = row.split(',').map(str::trim).collect();
        if cells.len() != self.width {
            return Err(TraceError::WrongWidth {
                line,
                cells: cells.len(),
                columns: self.width,
            });
        }
        let run = count(line, "run", cells[self.run])?;
        let tick = count(line, "tick", cells[self.tick])?;
        let at = elapsed(line, cells[self.at])?;
        let settling = match cells[self.phase] {
            "settling" => true,
            "commanding" => false,
            cell => {
                return Err(TraceError::BadCell {
                    line,
                    heading: "phase".to_string(),
                    cell: cell.to_string(),
                    want: "`commanding` or `settling`",
                });
            }
        };

        let measured = self
            .present
            .iter()
            .filter(|column| !cells[**column].is_empty())
            .count();
        let present = match measured {
            0 => None,
            JointId::COUNT => {
                let mut angles = JointVector::default();
                for (slot, joint) in JointId::ALL.into_iter().enumerate() {
                    let column = self.present[slot];
                    angles.set(
                        joint,
                        angle(line, super::present_heading(joint), cells[column])?,
                    );
                }
                Some(angles)
            }
            measured => {
                return Err(TraceError::HalfBlind {
                    line,
                    measured,
                    count: JointId::COUNT,
                });
            }
        };

        let mut goal = [None; JointId::COUNT];
        for (slot, joint) in JointId::ALL.into_iter().enumerate() {
            let cell = cells[self.goal[slot]];
            if !cell.is_empty() {
                goal[slot] = Some(angle(line, super::goal_heading(joint), cell)?);
            }
        }

        Ok((
            run,
            Sample {
                tick,
                at,
                settling,
                present,
                goal,
            },
        ))
    }
}

/// A cell holding a count.
fn count(line: usize, heading: &str, cell: &str) -> Result<u64, TraceError> {
    cell.parse().map_err(|_| TraceError::BadCell {
        line,
        heading: heading.to_string(),
        cell: cell.to_string(),
        want: "a whole number",
    })
}

/// A cell holding an angle.
fn angle(line: usize, heading: String, cell: &str) -> Result<f64, TraceError> {
    match cell.parse::<f64>() {
        Ok(angle) if angle.is_finite() => Ok(angle),
        _ => Err(TraceError::BadCell {
            line,
            heading,
            cell: cell.to_string(),
            want: "an angle in radians",
        }),
    }
}

/// A cell holding seconds since the run began.
///
/// Refused rather than clamped when it is not a time a duration can hold: a
/// negative or unplaceable timestamp would put a period out of order in every
/// series drawn from the file. The conversion is the fallible one, so a
/// timestamp too large for a `Duration` is a refusal naming its line and cell
/// like every other bad cell, and never a panic in whatever is reading the
/// file.
fn elapsed(line: usize, cell: &str) -> Result<Duration, TraceError> {
    cell.parse::<f64>()
        .ok()
        .and_then(|secs| Duration::try_from_secs_f64(secs).ok())
        .ok_or_else(|| TraceError::BadCell {
            line,
            heading: "t_s".to_string(),
            cell: cell.to_string(),
            want: "seconds since the run began",
        })
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
    use super::*;
    use crate::pump::TickSample;
    use crate::testutil::trace_fixture;
    use crate::trace::write_csv;

    /// The loop's period in these synthetic runs.
    const PERIOD: Duration = Duration::from_millis(20);

    /// Degrees as radians, for tests written in the units the machine is
    /// measured in.
    fn deg(degrees: f64) -> f64 {
        degrees.to_radians()
    }

    /// `series` as one joint's per-period measurements and goals, everything
    /// else in the machine held at zero, rendered and read back.
    ///
    /// Round-tripped through the writer rather than constructed directly, so
    /// every measurement below is taken from a file the loop could have
    /// written.
    fn recorded(joint: JointId, series: &[(Option<f64>, f64)], commanding: usize) -> Trace {
        let samples: Vec<TickSample> = series
            .iter()
            .enumerate()
            .map(|(tick, (present, goal))| {
                let mut goals = JointVector::default();
                goals.set(joint, *goal);
                TickSample {
                    tick: tick as u64,
                    at: PERIOD * u32::try_from(tick).expect("a short run"),
                    present: present.map(|angle| {
                        let mut angles = JointVector::default();
                        angles.set(joint, angle);
                        angles
                    }),
                    goal: goals,
                    released: JointSet::EMPTY,
                    settling: tick >= commanding,
                }
            })
            .collect();
        parsed(&samples)
    }

    /// `samples` written out as one run and read back.
    fn parsed(samples: &[TickSample]) -> Trace {
        let mut out = Vec::new();
        write_csv(&mut out, 0, samples, true).expect("a vector takes writes");
        Trace::parse(&String::from_utf8(out).expect("the rows are text")).expect("the rows parse")
    }

    /// A goal ramp with the measurement one period behind it, then three
    /// periods holding at the end: a move whose machine is following.
    fn ramp() -> Trace {
        let mut series: Vec<(Option<f64>, f64)> = (0..10)
            .map(|tick| {
                (
                    Some(0.1 * f64::from(tick - 1).max(0.0)),
                    0.1 * f64::from(tick),
                )
            })
            .collect();
        series.extend([(Some(0.9), 0.9); 3]);
        recorded(JointId::Leg(1), &series, 10)
    }

    /// Every period written comes back as the period it was, joint by joint —
    /// the measured half, the commanded half, the phase and the clock.
    #[test]
    fn a_run_reads_back_as_what_was_written() {
        let angles = JointVector {
            body_yaw: 0.5,
            legs: [1.0, 2.0, 3.0, 4.0, 5.0, 6.0],
            antennas: [7.0, 8.0],
        };
        let written = [
            TickSample {
                tick: 0,
                at: Duration::ZERO,
                present: Some(angles),
                goal: JointVector::default(),
                released: JointSet::EMPTY,
                settling: false,
            },
            TickSample {
                tick: 1,
                at: PERIOD,
                present: Some(JointVector::default()),
                goal: angles,
                released: JointSet::EMPTY,
                settling: true,
            },
        ];

        let trace = parsed(&written);

        let run = trace.run(0).expect("one run");
        assert_eq!(run.run, 0);
        assert_eq!(run.samples.len(), 2);
        for (read, written) in run.samples.iter().zip(&written) {
            assert_eq!(read.tick, written.tick);
            assert_eq!(read.at, written.at);
            assert_eq!(read.settling, written.settling);
            assert_eq!(read.released(), JointSet::EMPTY);
            for joint in JointId::ALL {
                let present = written.present.and_then(|angles| angles.get(joint));
                assert_eq!(read.present_of(joint), present, "{joint} measured");
                assert_eq!(read.goal_of(joint), written.goal.get(joint), "{joint} goal");
            }
        }
    }

    /// A period whose read fell short measured nothing, and reads back as
    /// nothing rather than as nine joints at the origin.
    #[test]
    fn a_blind_period_reads_back_as_no_measurement() {
        let trace = recorded(JointId::Leg(0), &[(None, 0.25)], 1);

        let sample = &trace.run(0).expect("one run").samples[0];
        assert_eq!(sample.present, None);
        assert_eq!(sample.present_of(JointId::Leg(0)), None);
        assert_eq!(sample.goal_of(JointId::Leg(0)), Some(0.25));
    }

    /// A servo that was commanded nothing reads back holding no goal, not
    /// holding the last one it was sent — and is measured against none.
    #[test]
    fn a_released_servo_reads_back_commanded_nothing() {
        let mut released = JointSet::EMPTY;
        released.insert(JointId::AntennaLeft);
        // Two periods under command, then two limp: a pair degraded part way
        // through the move, drifting for the rest of it.
        let period = |tick: u64, present: f64, released: JointSet| TickSample {
            tick,
            at: PERIOD * u32::try_from(tick).expect("a short run"),
            present: Some(JointVector {
                antennas: [present, present],
                ..JointVector::default()
            }),
            goal: JointVector {
                antennas: [0.5, 0.5],
                ..JointVector::default()
            },
            released,
            settling: false,
        };
        let trace = parsed(&[
            period(0, 0.0, JointSet::EMPTY),
            period(1, 0.2, JointSet::EMPTY),
            period(2, 0.3, released),
            period(3, 0.3, released),
        ]);

        let run = trace.run(0).expect("one run");
        assert_eq!(run.samples[1].released(), JointSet::EMPTY);
        assert_eq!(run.samples[2].released(), released);
        assert_eq!(run.samples[2].goal_of(JointId::AntennaLeft), None);
        assert_eq!(run.samples[2].goal_of(JointId::AntennaRight), Some(0.5));

        let metrics = run.metrics();
        let left = metrics.joint(JointId::AntennaLeft).expect("it moved");
        assert!(left.released);
        assert_eq!(left.residual, None, "abandoned, not left short");
        assert_eq!(left.arrived, None);
        // Its lag stops where its goal did: the two periods it was commanded
        // over, not the drift after them.
        assert!((left.worst_lag - 0.5).abs() < 1e-6, "{}", left.worst_lag);
        // The side that kept its goal is measured as usual.
        let right = metrics.joint(JointId::AntennaRight).expect("it moved");
        assert!(!right.released);
        assert_eq!(right.residual, Some(0.2));
    }

    /// A run nothing could be measured over answers nothing, rather than the
    /// zero a perfectly mirrored pair would read as.
    ///
    /// The widest offset is what a calibration asserts an upper bound against
    /// — the clashing pair never gets near the shipped separation — and a fold
    /// that starts at zero would let a run with an antenna out of service, or
    /// a stretch of dropped reads, satisfy that bound by measuring nothing.
    #[test]
    fn a_pair_that_was_never_both_read_is_not_a_mirrored_pair() {
        let mut released = JointSet::EMPTY;
        released.insert(JointId::AntennaLeft);
        let period = |tick: u64, released: JointSet| TickSample {
            tick,
            at: PERIOD * u32::try_from(tick).expect("a short run"),
            present: Some(JointVector {
                antennas: [1.2, -0.1],
                ..JointVector::default()
            }),
            goal: JointVector {
                antennas: [1.2, -0.1],
                ..JointVector::default()
            },
            released,
            settling: false,
        };
        let trace = parsed(&[period(0, released), period(1, released)]);
        let run = trace.run(0).expect("one run");

        assert_eq!(run.widest_offset(Sample::goal_of), None);
        // The measurement the same run does carry, so the `None` above is
        // about the missing goal cells and not about the run.
        let measured = run
            .widest_offset(Sample::present_of)
            .expect("both antennas were read every period");
        assert!((measured - 1.1).abs() < 1e-9, "{measured}");
    }

    /// Runs written under one number are still told apart, because the period
    /// counter starting again is a fresh move whatever the `run` cell says.
    ///
    /// Every recorded run in the fixtures is numbered zero: they were written
    /// by the per-caller counter this crate replaced.
    #[test]
    fn runs_written_under_one_number_are_still_told_apart() {
        let period = |tick: u64| TickSample {
            tick,
            at: PERIOD * u32::try_from(tick).expect("a short run"),
            present: Some(JointVector::default()),
            goal: JointVector::default(),
            released: JointSet::EMPTY,
            settling: false,
        };

        let trace = parsed(&[period(0), period(1), period(0), period(1), period(2)]);

        let lengths: Vec<usize> = trace.runs().iter().map(|run| run.samples.len()).collect();
        assert_eq!(lengths, vec![2, 3]);
        assert!(trace.runs().iter().all(|run| run.run == 0));
    }

    /// A header that names no column the reader needs is refused by name,
    /// rather than read against whatever stands in that position.
    #[test]
    fn a_header_missing_a_column_is_refused() {
        let text = "run,tick,t_s,phase\n0,0,0.0,commanding\n";

        let refusal = Trace::parse(text).expect_err("the header is short");

        assert_eq!(
            refusal,
            TraceError::MissingColumn {
                heading: "body_yaw_present_rad".to_string()
            }
        );
    }

    /// A row with a different number of cells than the header has columns is
    /// refused: every column after the missing cell would be read off by one.
    #[test]
    fn a_row_the_header_does_not_fit_is_refused() {
        let text = format!("{}\n0,0,0.0,commanding,1.0\n", crate::trace::header());

        let refusal = Trace::parse(&text).expect_err("the row is short");

        assert_eq!(
            refusal,
            TraceError::WrongWidth {
                line: 2,
                cells: 5,
                columns: 4 + 2 * JointId::COUNT,
            }
        );
    }

    /// A period that measured some joints and not others is refused: the
    /// grouped read either came back or fell short, and a mixture is a file
    /// something else wrote.
    #[test]
    fn a_period_that_measured_some_joints_and_not_others_is_refused() {
        let mut row = String::from("0,0,0.0,commanding,0.1");
        for _ in 1..JointId::COUNT {
            row.push(',');
        }
        for _ in 0..JointId::COUNT {
            row.push_str(",0.0");
        }
        let text = format!("{}\n{row}\n", crate::trace::header());

        let refusal = Trace::parse(&text).expect_err("the period is half measured");

        assert_eq!(
            refusal,
            TraceError::HalfBlind {
                line: 2,
                measured: 1,
                count: JointId::COUNT,
            }
        );
    }

    /// A cell that does not hold what its column means is refused, named, and
    /// quoted — a trace with a row nobody can read is not a measurement.
    #[test]
    fn a_cell_that_is_not_what_its_column_means_is_refused() {
        let row = |at: &str, angle: &str| {
            let mut row = format!("0,0,{at},commanding");
            for _ in 0..JointId::COUNT {
                row.push_str(&format!(",{angle}"));
            }
            for _ in 0..JointId::COUNT {
                row.push_str(",0.0");
            }
            format!("{}\n{row}\n", crate::trace::header())
        };

        let backwards = Trace::parse(&row("-0.02", "0.0")).expect_err("time runs forwards");
        let unplaceable = Trace::parse(&row("0.0", "nan")).expect_err("an angle is a number");
        let empty = Trace::parse("").expect_err("a file with no header is not a trace");

        assert_eq!(
            backwards,
            TraceError::BadCell {
                line: 2,
                heading: "t_s".to_string(),
                cell: "-0.02".to_string(),
                want: "seconds since the run began",
            }
        );
        assert_eq!(
            unplaceable,
            TraceError::BadCell {
                line: 2,
                heading: "body_yaw_present_rad".to_string(),
                cell: "nan".to_string(),
                want: "an angle in radians",
            }
        );
        assert_eq!(empty, TraceError::Empty);
    }

    /// A timestamp no duration can hold is refused like any other bad cell,
    /// rather than panicking whatever is reading the file.
    ///
    /// A finite, non-negative number is not automatically a `Duration`, and the
    /// conversion that clears the guard panics on the ones that are not. This
    /// reader is fed operator-written and hand-edited CSVs, so the refusal has
    /// to name the line and the cell where a std panic would name nothing.
    #[test]
    fn a_timestamp_no_duration_can_hold_is_refused() {
        let mut row = String::from("0,0,1e30,commanding");
        for _ in 0..2 * JointId::COUNT {
            row.push_str(",0.0");
        }
        let text = format!("{}\n{row}\n", crate::trace::header());

        let refusal = Trace::parse(&text).expect_err("1e30 seconds is not a duration");

        assert_eq!(
            refusal,
            TraceError::BadCell {
                line: 2,
                heading: "t_s".to_string(),
                cell: "1e30".to_string(),
                want: "seconds since the run began",
            }
        );
    }

    /// A file the reader cannot open, and a file that is not a trace, both come
    /// back naming the path.
    ///
    /// The wrapper is the whole reason [`ReadError`] exists: the parse errors
    /// know the line and the column and nothing about which file they came out
    /// of, and a fixture path typo or a truncated checked-in CSV is exactly the
    /// case where the file is what a reader needs told.
    #[test]
    fn a_file_that_is_not_a_trace_is_refused_by_name() {
        let missing = PathBuf::from("/nonexistent/never-written.csv");
        let refusal = Trace::read(&missing).expect_err("no such file");
        assert!(
            matches!(refusal, ReadError::Unreadable { .. }),
            "{refusal:?}"
        );
        assert!(
            refusal.to_string().contains("never-written.csv"),
            "the refusal names no file: {refusal}"
        );

        let scratch = crate::testutil::scratch_path("not-a-trace.csv");
        std::fs::write(&scratch, "not a trace\n").expect("the scratch path is writable");
        let refusal = Trace::read(&scratch).expect_err("that is not a trace");
        std::fs::remove_file(&scratch).ok();
        let ReadError::Malformed { source, .. } = &refusal else {
            panic!("a file that read fine is malformed, not unreadable: {refusal:?}");
        };
        assert!(
            matches!(source, TraceError::MissingColumn { .. }),
            "{source}"
        );
        assert!(
            refusal.to_string().contains("not-a-trace.csv"),
            "the refusal names no file: {refusal}"
        );
    }

    /// Every figure in the table is the one the series carries: how far the
    /// joint went, how fast it and its goal moved, the largest single command,
    /// how far behind it fell, and when it got there.
    #[test]
    fn the_table_measures_the_series_it_was_given() {
        let trace = ramp();

        let metrics = trace.run(0).expect("one run").metrics();
        let leg = metrics.joint(JointId::Leg(1)).expect("it moved");
        assert!(!leg.released);
        assert!((leg.span - 0.9).abs() < 1e-6, "{}", leg.span);
        // Central differences over a ramp of 0.1 rad per 20 ms period.
        assert!((leg.peak_speed - 5.0).abs() < 1e-4, "{}", leg.peak_speed);
        assert!(
            (leg.peak_goal_speed - 5.0).abs() < 1e-4,
            "{}",
            leg.peak_goal_speed
        );
        assert!(
            (leg.peak_goal_step - 0.1).abs() < 1e-6,
            "{}",
            leg.peak_goal_step
        );
        assert!((leg.worst_lag - 0.1).abs() < 1e-6, "{}", leg.worst_lag);
        assert_eq!(leg.arrived, Some(PERIOD * 10));
        assert_eq!(leg.residual, Some(0.0));
        // And the run's own figures: it commanded for ten periods and the
        // machine was there one period later.
        assert_eq!(metrics.periods, 13);
        assert_eq!(metrics.span, PERIOD * 12);
        assert_eq!(metrics.commanding_end, Some(PERIOD * 9));
        assert_eq!(metrics.arrival, Some(PERIOD * 10));
        assert_eq!(metrics.settle_gap(), Some(PERIOD));
    }

    /// A joint that neither moved nor was asked to gets no row: its span is
    /// read noise, and a velocity beside a stationary servo is a number that
    /// means nothing.
    #[test]
    fn a_joint_that_never_moved_is_not_reported() {
        let metrics = ramp().run(0).expect("one run").metrics();

        assert_eq!(metrics.joints.len(), 1);
        assert!(metrics.joint(JointId::BodyYaw).is_none());
        assert!(metrics.joint(JointId::AntennaRight).is_none());
    }

    /// A joint that passes through its final angle and leaves again has not
    /// arrived: arrival is where the machine stopped, not where it was seen.
    #[test]
    fn a_joint_that_passes_its_goal_and_leaves_has_not_arrived() {
        let series = [
            (Some(0.0), 0.5),
            (Some(0.5), 0.5),
            (Some(1.0), 0.5),
            (Some(1.5), 0.5),
        ];

        let trace = recorded(JointId::Leg(2), &series, 1);

        let metrics = trace.run(0).expect("one run").metrics();
        let leg = metrics.joint(JointId::Leg(2)).expect("it moved");
        assert_eq!(leg.arrived, None);
        assert_eq!(metrics.arrival, None);
        assert_eq!(metrics.settle_gap(), None);
        assert_eq!(leg.residual, Some(1.0));
    }

    /// A stall is the longest stretch the joint stood still for, and it carries
    /// how far the goal got away while it stood there — which is what tells a
    /// jam from a joint parked at the goal it arrived at.
    #[test]
    fn a_stall_is_the_longest_stretch_the_joint_stood_still_for() {
        let series = [
            (Some(0.0), 0.0),
            (Some(0.1), 0.1),
            // Jammed: four periods where the goal walks away.
            (Some(0.2), 0.2),
            (Some(0.2), 0.4),
            (Some(0.2), 0.6),
            (Some(0.2), 0.8),
            // Free again, and parked at the goal for three.
            (Some(0.6), 0.8),
            (Some(0.8), 0.8),
            (Some(0.8), 0.8),
            (Some(0.8), 0.8),
        ];

        let trace = recorded(JointId::AntennaRight, &series, 6);
        let run = trace.run(0).expect("one run");

        let stall = run
            .longest_stall(JointId::AntennaRight, deg(1.0))
            .expect("it stood still");
        assert_eq!(stall.periods, 4);
        assert_eq!((stall.from, stall.until), (PERIOD * 2, PERIOD * 5));
        assert!((stall.at - 0.2).abs() < 1e-6, "{}", stall.at);
        assert!((stall.worst_lag - 0.6).abs() < 1e-6, "{}", stall.worst_lag);
        // A joint that stood still the whole run also has a stall, and it is
        // the lag that says the joint was holding its goal rather than fighting
        // something. Nothing here decides that for the caller.
        let parked = run
            .longest_stall(JointId::Leg(0), deg(1.0))
            .expect("it never moved");
        assert_eq!(parked.periods, series.len());
        assert!(parked.worst_lag < f64::EPSILON, "{}", parked.worst_lag);
    }

    /// The validated gesture: the whole machine, head and both antennas, up in
    /// 0.82 s and measurably there before the last goal went out.
    ///
    /// Zero settle is the claim this recording is kept for — not one period was
    /// spent waiting after the commanding stopped — and it is the shape every
    /// guard in this repo has to admit.
    #[test]
    fn the_validated_gesture_arrives_inside_its_own_clock() {
        let trace = trace_fixture("trace-verify2");

        assert_eq!(trace.runs().len(), 1);
        let metrics = trace.run(0).expect("the run").metrics();
        assert_eq!(metrics.periods, 27);
        assert!(
            (metrics.span.as_secs_f64() - 0.8196).abs() < 5e-4,
            "{:?}",
            metrics.span
        );
        assert_eq!(metrics.commanding_end, Some(metrics.span));
        assert_eq!(metrics.settle_gap(), Some(Duration::ZERO));
        assert_eq!(metrics.joints.len(), JointId::COUNT);
        for joint in &metrics.joints {
            assert!(joint.arrived.is_some(), "{} arrived", joint.joint);
            let residual = joint.residual.expect("it was holding a goal");
            assert!(
                residual < ARRIVED_TOLERANCE_RAD,
                "{}: {residual}",
                joint.joint
            );
        }
        // The fastest leg was asked for 0.107 rad in a period. That is more
        // than the same command's dry-pass step: this run was driven by a loop
        // that sampled the trajectory at wall-clock time, so a period that
        // started late commanded the extra. It is recorded, and nothing is
        // sized against it.
        let leg = metrics.joint(JointId::Leg(1)).expect("it moved");
        assert!(
            (leg.peak_goal_step - 0.1068).abs() < 5e-4,
            "{}",
            leg.peak_goal_step
        );
        assert!(
            (leg.peak_speed - deg(199.3)).abs() < deg(0.5),
            "{}",
            leg.peak_speed
        );
    }

    /// The antenna speed record, and the staggered pair that made it safe: one
    /// side sweeping 187° in 0.40 s at 855°/s while the other takes 0.93 s over
    /// the same arc.
    #[test]
    fn the_fast_sweep_is_the_speed_record_and_a_staggered_pair() {
        let trace = trace_fixture("trace-fast4");

        let metrics = trace.run(0).expect("the run").metrics();
        let fast = metrics.joint(JointId::AntennaLeft).expect("it swept");
        let slow = metrics.joint(JointId::AntennaRight).expect("it swept");
        assert!((fast.span - deg(187.0)).abs() < deg(0.5), "{}", fast.span);
        assert!(
            (fast.peak_speed - deg(855.5)).abs() < deg(1.0),
            "{}",
            fast.peak_speed
        );
        let arrived = fast.arrived.expect("it got there");
        assert!((arrived.as_secs_f64() - 0.4035).abs() < 5e-4, "{arrived:?}");
        // The other side is on its own clock, more than half a second behind —
        // which is the whole point of the pair having two.
        let behind = slow.arrived.expect("it got there") - arrived;
        assert!(behind > Duration::from_millis(500), "{behind:?}");
        assert!(
            slow.peak_speed < fast.peak_speed / 2.0,
            "{} against {}",
            slow.peak_speed,
            fast.peak_speed
        );
    }

    /// The gain change, recorded as the same step command twice: the shipped
    /// P-only gains park the loaded pair ~4° short of the goal for good, and
    /// the tuned gains bring that to about a degree.
    ///
    /// Neither run is a clean any guard can be replayed against. Both command
    /// the whole span in one period by construction — that is what a step
    /// response is — so the goal steps here are records of the instrument, not
    /// of a move anything should admit.
    #[test]
    fn the_gain_change_is_two_step_responses_and_the_droop_between_them() {
        let trace = trace_fixture("trace-newgains");

        assert_eq!(trace.runs().len(), 2);
        let shipped = trace.run(0).expect("the first run").metrics();
        let tuned = trace.run(1).expect("the second run").metrics();

        for (run, metrics) in [(0, &shipped), (1, &tuned)] {
            for joint in metrics.joints.iter().filter(|joint| joint.span > deg(10.0)) {
                assert!(
                    joint.peak_goal_step > joint.span / 2.0,
                    "run {run}, {}: the goal jumped {} of a {} span",
                    joint.joint,
                    joint.peak_goal_step,
                    joint.span
                );
            }
        }

        // Legs 2 and 5 carry the load. Under the shipped gains they stop short
        // and stay short — the steady-state droop of a proportional term
        // holding gravity, which nothing had ever measured because nothing had
        // ever measured arrival.
        for leg in [JointId::Leg(1), JointId::Leg(4)] {
            let droop = shipped.joint(leg).expect("it moved");
            let residual = droop.residual.expect("it was holding a goal");
            assert!(
                (deg(3.9)..=deg(4.4)).contains(&residual),
                "{leg}: {}",
                residual.to_degrees()
            );
            assert_eq!(droop.arrived, None, "{leg} never got there");

            let tuned = tuned.joint(leg).expect("it moved");
            let residual = tuned.residual.expect("it was holding a goal");
            assert!(residual < deg(1.3), "{leg}: {}", residual.to_degrees());
            assert!(tuned.arrived.is_some(), "{leg} got there");
        }
    }

    /// The one collision on record: both antenna tips meet at the inboard
    /// crossing, stall against each other at mirrored angles for over 40
    /// periods — about 1.06 s — while the goal walks away, and spring back when
    /// the servos give up.
    ///
    /// This is the run every guard has to catch. It is the third in its file —
    /// the same session's earlier raise, run 1, went through cleanly.
    #[test]
    fn the_collision_stalls_both_antennas_at_mirrored_angles() {
        let trace = trace_fixture("trace-stagger");

        assert_eq!(trace.runs().len(), 3);
        let jam = trace.run(2).expect("the failed stow");
        let right = jam
            .longest_stall(JointId::AntennaRight, deg(1.0))
            .expect("it stopped");
        let left = jam
            .longest_stall(JointId::AntennaLeft, deg(1.0))
            .expect("it stopped");

        // Mirrored, which is what tip-to-tip means: the two sides stop at the
        // same angle on opposite sides, a few degrees apart.
        assert!(
            (deg(52.0)..=deg(56.6)).contains(&right.at.abs()),
            "{}",
            right.at.to_degrees()
        );
        assert!(
            (deg(52.0)..=deg(56.6)).contains(&left.at.abs()),
            "{}",
            left.at.to_degrees()
        );
        assert!(right.at.signum() != left.at.signum(), "opposite sides");
        assert!(
            (right.at + left.at).abs() < deg(5.0),
            "{} against {}",
            right.at.to_degrees(),
            left.at.to_degrees()
        );

        // Held there while the goal ran the rest of the way home.
        for (side, stall) in [("right", right), ("left", left)] {
            assert!(stall.periods >= 40, "{side}: {} periods", stall.periods);
            assert!(
                stall.worst_lag > deg(120.0),
                "{side}: {}",
                stall.worst_lag.to_degrees()
            );
        }

        // Neither arrived, and both finished further from the goal than they
        // stalled: the tips sprang apart as the servos dropped out.
        let metrics = jam.metrics();
        for (joint, stall) in [(JointId::AntennaRight, right), (JointId::AntennaLeft, left)] {
            let antenna = metrics.joint(joint).expect("it was asked to move");
            assert_eq!(antenna.arrived, None, "{joint}");
            let residual = antenna.residual.expect("it was holding a goal");
            assert!(residual > stall.worst_lag, "{joint}: {residual}");
        }
    }
}
