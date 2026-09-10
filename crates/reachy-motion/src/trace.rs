//! Reading a recorded run back off the CSV `//cogs:trace_export` writes.
//!
//! A parser over `&str` and nothing else: no file is opened here and no figure
//! is judged. What a caller gets is the periods a recording holds, in the order
//! they were recorded, split into the runs the file carries — the series every
//! offline reading of a hardware run is taken over.
//!
//! It lives in the library because two readers need it and they must not be two
//! parsers. The replay suite drives the shipped comparison over checked-in
//! fixture windows; `//cogs:trace_judge` drives the same comparison over a
//! whole exported tour, which is too large to check in. A second copy of the
//! reader would let a fixture and a tour disagree about what a blank cell
//! means.
//!
//! A malformed file is refused naming its line, never read around. A recording
//! exists to have measurements drawn from it, and skipping an unreadable row
//! would silently drop a period out of every series taken from the file.
//! [`Trace::try_parse`] hands the refusal back as a sentence, which is what an
//! operator tool prints beside its exit code; [`Trace::parse`] panics with the
//! same sentence, which is what the fixture-backed suite wants — a checked-in
//! fixture that no longer parses is a broken checkout, not a case.
//!
//! Two cautions about what a recorded series is, carried from the loop that
//! wrote these files.
//!
//! - **A goal column is what the loop commanded, not what it planned.** The
//!   oldest recordings here predate the per-period move clock, so a period that
//!   started late sampled the trajectory further along and commanded a
//!   correspondingly larger step. The recorded step is an upper bound inflated
//!   by whatever the scheduler did that night, and nothing may be sized against
//!   it; it is measured because the inflation is itself a measurement.
//! - **A measured column is the servo's own encoder**, at the rate the loop
//!   read it. Speeds taken here are differences of that series, so they are
//!   averages over a period rather than instantaneous rates.

use core::time::Duration;

use brenn_reachy__motion__joints_clk_rs::JointFlags;

use crate::joints::{JointRef, JointVector, ROWS, flags};

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
        crate::joints::row(joint).and_then(|row| self.goal[row])
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
    /// [`Self::try_parse`]'s refusal raised as a panic, for the callers whose
    /// input is a checked-in fixture: a fixture that stopped parsing is a
    /// broken checkout and there is nothing for the caller to do about it.
    ///
    /// # Panics
    ///
    /// On any text [`Self::try_parse`] refuses, with that refusal's own words.
    #[must_use]
    pub fn parse(text: &str) -> Self {
        match Self::try_parse(text) {
            Ok(trace) => trace,
            Err(refusal) => panic!("{refusal}"),
        }
    }

    /// Read a trace out of `text`, or say what about it is not a recording.
    ///
    /// # Errors
    ///
    /// A header naming no column this reader needs, a row of the wrong width, a
    /// cell that is no period, no instant, no phase or no angle, or a period
    /// measuring some joints and not others. Every refusal names the line: a
    /// trace exists to have guard values drawn from it, and skipping an
    /// unreadable row would silently drop a period out of every series taken
    /// from the file.
    pub fn try_parse(text: &str) -> Result<Self, String> {
        let mut lines = text
            .lines()
            .enumerate()
            .map(|(index, line)| (index + 1, line))
            .filter(|(_, line)| !line.trim().is_empty());
        let (_, header) = lines
            .next()
            .ok_or_else(|| "the trace has no header row".to_string())?;
        let columns = Columns::resolve(header)?;

        let mut runs: Vec<Run> = Vec::new();
        for (line, row) in lines {
            let sample = columns.sample(line, row)?;
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
        Ok(Self { runs })
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
    fn resolve(header: &str) -> Result<Self, String> {
        let headings: Vec<&str> = header.split(',').map(str::trim).collect();
        let find = |heading: &str| {
            headings
                .iter()
                .position(|candidate| *candidate == heading)
                .ok_or_else(|| format!("the trace header names no `{heading}` column"))
        };
        let mut present = [0; ROWS.len()];
        let mut goal = [0; ROWS.len()];
        for (row, joint) in ROWS.into_iter().enumerate() {
            present[row] = find(&present_column(joint))?;
            goal[row] = find(&goal_column(joint))?;
        }
        Ok(Self {
            tick: find("tick")?,
            at: find("t_s")?,
            phase: find("phase")?,
            present,
            goal,
            width: headings.len(),
        })
    }

    /// One row as the period it records.
    fn sample(&self, line: usize, row: &str) -> Result<Sample, String> {
        let cells: Vec<&str> = row.split(',').map(str::trim).collect();
        if cells.len() != self.width {
            return Err(format!(
                "line {line} has {} cells; the header names {}",
                cells.len(),
                self.width
            ));
        }
        let tick = cells[self.tick]
            .parse()
            .map_err(|_| format!("line {line}: `{}` is no period", cells[self.tick]))?;
        let at = cells[self.at]
            .parse::<f64>()
            .ok()
            .and_then(|secs| Duration::try_from_secs_f64(secs).ok())
            .ok_or_else(|| {
                format!(
                    "line {line}: `{}` is not seconds since the run began",
                    cells[self.at]
                )
            })?;
        let settling = match cells[self.phase] {
            "settling" => true,
            "commanding" => false,
            cell => {
                return Err(format!(
                    "line {line}: `{cell}` is neither `commanding` nor `settling`"
                ));
            }
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
                    angles.set(joint, angle(line, cells[self.present[row]])?);
                }
                Some(angles)
            }
            count => {
                return Err(format!(
                    "line {line} measures {count} of {} joints; a period read all of them or none",
                    ROWS.len()
                ));
            }
        };

        let mut goal = [None; ROWS.len()];
        for (row, cell) in self.goal.iter().enumerate() {
            let cell = cells[*cell];
            if !cell.is_empty() {
                goal[row] = Some(angle(line, cell)?);
            }
        }

        Ok(Sample {
            tick,
            at,
            settling,
            present,
            goal,
        })
    }
}

/// The prefix a joint's two columns are written under: the library's own, so
/// the fixtures and whatever writes them name a crank the same way.
fn column(joint: JointRef) -> &'static str {
    crate::joints::column_name(joint).expect("the nine bus rows each name a column")
}

/// What the trace calls the column holding `joint`'s reading.
///
/// The spelling of a column name lives here and only here: the header a writer
/// emits and the header this parser resolves are the same two functions, so a
/// suffix cannot be changed on one side alone.
#[must_use]
pub fn present_column(joint: JointRef) -> String {
    format!("{}_present_rad", column(joint))
}

/// What the trace calls the column holding `joint`'s setpoint.
#[must_use]
pub fn goal_column(joint: JointRef) -> String {
    format!("{}_goal_rad", column(joint))
}

/// A cell holding an angle in radians.
fn angle(line: usize, cell: &str) -> Result<f64, String> {
    match cell.parse::<f64>() {
        Ok(angle) if angle.is_finite() => Ok(angle),
        _ => Err(format!("line {line}: `{cell}` is no angle in radians")),
    }
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

    use crate::joints::ROWS;

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

    /// The text of a two-period trace, which the refusal cases below damage one
    /// cell at a time.
    fn text(periods: &[(u64, f64)]) -> String {
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
        text
    }

    /// Every shape of malformed file is refused naming the line, and none is
    /// read around.
    ///
    /// The refusals are the whole contract of this reader: a period dropped
    /// because a cell would not parse is a hole in a series somebody is about
    /// to take a residual pin off, and it would not be visible in the figure.
    /// The tool that hands an operator's file to this reader prints the
    /// sentence beside its exit code, which is why they are sentences.
    #[test]
    fn every_malformed_trace_is_refused_naming_its_line() {
        let sound = text(&[(0, 0.00), (1, 0.02)]);
        let damaged = |patch: &dyn Fn(&str) -> String| -> String {
            sound.lines().map(patch).collect::<Vec<_>>().join("\n")
        };
        let second = |patch: &dyn Fn(&str) -> String| {
            damaged(&|line: &str| {
                if line.starts_with("0,1,") {
                    patch(line)
                } else {
                    line.to_string()
                }
            })
        };
        let cell = |line: &str, index: usize, value: &str| {
            let mut cells: Vec<&str> = line.split(',').collect();
            cells[index] = value;
            cells.join(",")
        };

        for (text, expected) in [
            (
                damaged(&|line: &str| line.replace("tick,", "slot,")),
                "the trace header names no `tick` column",
            ),
            (
                second(&|line: &str| {
                    line.rsplit_once(',')
                        .expect("a row has cells")
                        .0
                        .to_string()
                }),
                "line 3 has 21 cells; the header names 22",
            ),
            (
                second(&|line: &str| cell(line, 1, "soon")),
                "line 3: `soon` is no period",
            ),
            (
                second(&|line: &str| cell(line, 2, "later")),
                "line 3: `later` is not seconds since the run began",
            ),
            (
                second(&|line: &str| cell(line, 3, "resting")),
                "line 3: `resting` is neither `commanding` nor `settling`",
            ),
            (
                second(&|line: &str| cell(line, 4, "")),
                "line 3 measures 8 of 9 joints",
            ),
            (
                second(&|line: &str| cell(line, 4, "sideways")),
                "line 3: `sideways` is no angle in radians",
            ),
            (
                second(&|line: &str| cell(line, 13, "inf")),
                "line 3: `inf` is no angle in radians",
            ),
            (String::new(), "the trace has no header row"),
        ] {
            let refusal = Trace::try_parse(&text).err().unwrap_or_else(|| {
                panic!("a trace that should have been refused with {expected:?} was read")
            });
            assert!(refusal.contains(expected), "{refusal:?}");
            assert_eq!(
                std::panic::catch_unwind(|| Trace::parse(&text))
                    .err()
                    .and_then(|panic| panic
                        .downcast_ref::<String>()
                        .map(std::string::ToString::to_string))
                    .as_deref(),
                Some(refusal.as_str()),
                "the panicking reader raises the fallible one's own words"
            );
        }
    }
}
