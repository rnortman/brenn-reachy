//! The schedule, as arithmetic.
//!
//! The driver's cycles sit on a fixed grid `g(N) = S + N × period` on the
//! realtime clock, `S` a top of a second. Every wake is computed from the grid
//! and never from "now plus a period", which is the difference between a loop
//! that holds its phase for a week and one that drifts by however long each
//! cycle happened to take.
//!
//! One rule beyond that, and it is a safety rule rather than a tidiness one: a
//! cycle that ran long does **not** get run again to catch up. The next wake is
//! the next grid point still in the future, the grid points that went by
//! unattended are counted, and the count is published. Running four late cycles
//! back to back would put four setpoints on the bus in a burst, each dated for
//! an instant already past — a machine asked to be in four places at once, which
//! is exactly the jump the goal gate exists to prevent.
//!
//! No clock is read here and nothing sleeps, so every rule above is a function
//! of two numbers and a test can state it directly.

/// A second, in nanoseconds. What a grid's start is rounded to, and what a
/// cadence counted in cycles of a grid is measured against.
pub(crate) const NANOS_PER_SECOND: i64 = 1_000_000_000;

/// A cycle the loop is about to run.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Cycle {
    /// Which grid point this is, counting from the grid's start.
    pub index: i64,
    /// The instant the grid puts it at: what the loop sleeps until, and the
    /// `nominal_time` every sample of this cycle is stamped with.
    pub nominal_ns: i64,
    /// How many grid points were passed over to reach this one — zero when the
    /// previous cycle finished in time. Never made up for; published as
    /// `cycle_skipped` and counted.
    pub skipped: u32,
}

/// A grid that could not be built.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum GridError {
    /// The period was zero or negative. A schedule with no spacing is not a
    /// schedule, and dividing by it is how a loop spins.
    #[error("the bus cycle is {period_ns}ns; a cycle has to be a positive number of nanoseconds")]
    PeriodNotPositive {
        /// What was asked for.
        period_ns: i64,
    },
}

/// The fixed grid the driver's cycles sit on.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Grid {
    start_ns: i64,
    period_ns: i64,
}

impl Grid {
    /// A grid starting at `start_ns`, spaced `period_ns` apart.
    ///
    /// # Errors
    ///
    /// [`GridError::PeriodNotPositive`] when the period is not a positive
    /// number of nanoseconds.
    pub const fn new(start_ns: i64, period_ns: i64) -> Result<Self, GridError> {
        if period_ns <= 0 {
            return Err(GridError::PeriodNotPositive { period_ns });
        }
        Ok(Self {
            start_ns,
            period_ns,
        })
    }

    /// The first top of a second at or after `now_ns`.
    ///
    /// Where a grid starts. A round second rather than the instant the process
    /// happened to finish setting up, so that two runs of this driver on one
    /// machine, or a driver and something reading its samples, describe the same
    /// instants: a grid point is then a number a human reading a log can divide.
    #[must_use]
    pub const fn top_of_second_at(now_ns: i64) -> i64 {
        let whole = now_ns.div_euclid(NANOS_PER_SECOND);
        let at = whole * NANOS_PER_SECOND;
        if at == now_ns {
            at
        } else {
            at + NANOS_PER_SECOND
        }
    }

    /// The grid's spacing, nanoseconds.
    #[must_use]
    pub const fn period_ns(&self) -> i64 {
        self.period_ns
    }

    /// Where grid point `index` falls.
    #[must_use]
    pub const fn instant(&self, index: i64) -> i64 {
        self.start_ns + index * self.period_ns
    }

    /// The first cycle to run, given the clock reads `now_ns`.
    ///
    /// The earliest grid point at or after the grid's own start that has not
    /// already gone by. Nothing is skipped on the way in: the points before the
    /// start are cycles this process did not exist for.
    ///
    /// A `now_ns` before the start is the ordinary startup case, not an odd one
    /// — a grid starts at the next top of a second, so setup finishes up to a
    /// second early — and it gets index 0 at the start. The alternative is a run
    /// whose first cycles carry negative indices and whose first wake is not the
    /// second the grid says it began on.
    #[must_use]
    pub fn first_from(&self, now_ns: i64) -> Cycle {
        let index = self.index_at_or_after(now_ns).max(0);
        Cycle {
            index,
            nominal_ns: self.instant(index),
            skipped: 0,
        }
    }

    /// The cycle after `done`, given that the clock reads `now_ns` now that it
    /// has finished.
    ///
    /// The next grid point if it is still ahead — including the case where it is
    /// exactly now, which is on time to the nanosecond and not late. Otherwise
    /// the first point that is still ahead, with every point in between counted
    /// as skipped and none of them run.
    #[must_use]
    pub fn next_after(&self, done: &Cycle, now_ns: i64) -> Cycle {
        let intended = done.index + 1;
        let index = self.index_at_or_after(now_ns).max(intended);
        let skipped = u32::try_from(index - intended).unwrap_or(u32::MAX);
        Cycle {
            index,
            nominal_ns: self.instant(index),
            skipped,
        }
    }

    /// The lowest grid index whose instant is at or after `now_ns`.
    fn index_at_or_after(&self, now_ns: i64) -> i64 {
        let from_start = now_ns - self.start_ns;
        let whole = from_start.div_euclid(self.period_ns);
        if whole * self.period_ns == from_start {
            whole
        } else {
            whole + 1
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Grid, GridError, NANOS_PER_SECOND};

    /// A round instant, so a number that came from the wrong side of an
    /// arithmetic mistake is visibly not one of these.
    const S: i64 = 1_700_000_000 * NANOS_PER_SECOND;

    /// The cycle the driver ships with.
    const PERIOD: i64 = 20_000_000;

    fn grid() -> Grid {
        Grid::new(S, PERIOD).expect("20ms is a period")
    }

    #[test]
    fn a_period_has_to_be_positive() {
        for period in [0, -1, -PERIOD] {
            assert_eq!(
                Grid::new(S, period),
                Err(GridError::PeriodNotPositive { period_ns: period }),
                "a {period}ns cycle"
            );
        }
    }

    #[test]
    fn a_grid_starts_at_a_top_of_second() {
        assert_eq!(Grid::top_of_second_at(S), S, "already one");
        assert_eq!(Grid::top_of_second_at(S + 1), S + NANOS_PER_SECOND);
        assert_eq!(
            Grid::top_of_second_at(S + NANOS_PER_SECOND - 1),
            S + NANOS_PER_SECOND
        );
    }

    #[test]
    fn a_point_is_the_start_plus_whole_periods() {
        let grid = grid();
        assert_eq!(grid.instant(0), S);
        assert_eq!(grid.instant(1), S + PERIOD);
        assert_eq!(grid.instant(50), S + NANOS_PER_SECOND, "50 cycles a second");
    }

    #[test]
    fn the_first_cycle_is_the_next_point_not_yet_gone_by() {
        let grid = grid();
        let at_start = grid.first_from(S);
        assert_eq!((at_start.index, at_start.nominal_ns), (0, S));
        let just_after = grid.first_from(S + 1);
        assert_eq!((just_after.index, just_after.nominal_ns), (1, S + PERIOD));
        let late = grid.first_from(S + 10 * PERIOD + 5);
        assert_eq!(
            (late.index, late.nominal_ns),
            (11, S + 11 * PERIOD),
            "the points before the process existed are not skips"
        );
        assert_eq!(late.skipped, 0);
    }

    #[test]
    fn a_clock_still_short_of_the_start_gets_the_grid_s_own_first_point() {
        let grid = grid();
        // The startup case: the grid starts at the next top of a second, so
        // setup finishes before it. The first cycle is the start, at index zero
        // — not a negative index at a point the grid does not claim.
        for early in [S - 1, S - PERIOD, S - NANOS_PER_SECOND + 1] {
            let first = grid.first_from(early);
            assert_eq!(
                (first.index, first.nominal_ns, first.skipped),
                (0, S, 0),
                "the run begins on the second the grid says it does"
            );
        }
    }

    #[test]
    fn a_cycle_that_finished_in_time_is_followed_by_the_next_point() {
        let grid = grid();
        let first = grid.first_from(S);
        // Finished a millisecond into its own period.
        let next = grid.next_after(&first, S + 1_000_000);
        assert_eq!(next.index, 1);
        assert_eq!(next.nominal_ns, S + PERIOD);
        assert_eq!(next.skipped, 0);
    }

    #[test]
    fn finishing_exactly_on_the_next_point_is_on_time() {
        let grid = grid();
        let first = grid.first_from(S);
        let next = grid.next_after(&first, S + PERIOD);
        assert_eq!(
            (next.index, next.skipped),
            (1, 0),
            "a wake due this instant is due, not missed"
        );
    }

    #[test]
    fn an_overrun_skips_to_the_next_future_point_and_counts_what_it_passed() {
        let grid = grid();
        let first = grid.first_from(S);
        // The cycle ran two and a half periods long.
        let next = grid.next_after(&first, S + 2 * PERIOD + PERIOD / 2);
        assert_eq!(next.index, 3, "the next point still ahead");
        assert_eq!(next.nominal_ns, S + 3 * PERIOD);
        assert_eq!(next.skipped, 2, "points 1 and 2 went by unattended");
    }

    #[test]
    fn a_long_outage_is_one_skip_forward_and_never_a_run_of_late_cycles() {
        let grid = grid();
        let mut cycle = grid.first_from(S);
        // Half a second of lost CPU, which is 25 periods.
        cycle = grid.next_after(&cycle, S + 25 * PERIOD + 1);
        assert_eq!(cycle.index, 26);
        assert_eq!(cycle.skipped, 25);
        // And the cycle after that is the ordinary next one: the debt is not
        // carried, because nothing is owed.
        let after = grid.next_after(&cycle, cycle.nominal_ns + 1_000_000);
        assert_eq!((after.index, after.skipped), (27, 0));
    }

    #[test]
    fn the_grid_does_not_drift_over_a_long_run() {
        let grid = grid();
        let mut cycle = grid.first_from(S);
        // Every cycle finishes at a different offset into its period, which is
        // what a loop that slept for "a period" from wherever it woke would
        // accumulate.
        for step in 0..1_000 {
            let jitter = (step % 17) * 100_000;
            cycle = grid.next_after(&cycle, cycle.nominal_ns + jitter);
            assert_eq!(cycle.skipped, 0);
        }
        assert_eq!(cycle.index, 1_000);
        assert_eq!(cycle.nominal_ns, S + 1_000 * PERIOD);
    }
}
