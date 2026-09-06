//! Reading a driver's sample stream, for whatever analyzer is reading it.
//!
//! Two tools judge two different runs off the same log: the wake gesture's
//! report and the library tour's. What they conclude is their own, but the
//! vocabulary underneath — what a sample's nine angles are, how far the machine
//! ran behind its goals, what grid the samples sit on, and which slots the
//! driver already said it missed — is one subject and belongs in one place.
//!
//! The reason it is shared rather than copied is what an operator does with the
//! output: a lag figure from a gesture run and a lag figure from a tour run are
//! compared against each other and against the recorded gestures, so the two
//! must be the same measurement said the same way. A second copy diverges on
//! the first change nobody made twice, and the divergence is invisible until
//! somebody reads two runs side by side.
//!
//! Sans-log: everything here takes the messages it needs and a report to write
//! into. Which channels those came off, and which of them a tool binds, is the
//! tool's.

#![forbid(unsafe_code)]

use std::collections::BTreeSet;

use brenn_reachy__driver__health_clk_rs::{DriverEventWire, EventKindWire};
use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
use brenn_reachy__motion__faults_clk_rs::TickFaultWire;
use log_read::Logged;
use reachy_motion::joints::{JointGroup, ROWS, group_of, row, rows_of};
use reachy_motion::tick::{
    RECORDED_WORST_ANTENNA_LAG_RAD, RECORDED_WORST_HEAD_LAG_RAD, TrackingFaultConfig,
};
use run_report::Report;

/// The nine angles a sample was read at, or nothing where it carried no
/// reading.
#[must_use]
pub fn present_rows(sample: &PoseSampleWire) -> Option<[f64; ROWS.len()]> {
    if !sample.present_valid() {
        return None;
    }
    sample.present().validate().ok().map(rows_of)
}

/// The nine angles a sample was holding, or nothing where it held none.
#[must_use]
pub fn commanded_rows(sample: &PoseSampleWire) -> Option<[f64; ROWS.len()]> {
    if !sample.commanded_valid() {
        return None;
    }
    sample.commanded().validate().ok().map(rows_of)
}

/// How far the machine ran behind what it was told to hold, over a whole run.
///
/// Off the samples alone: each carries the setpoint the driver is holding
/// beside the position it read, so the lag needs no join against the goal
/// stream. Two figures, head and antennas, because those are the two the
/// recorded hardware gestures pinned and the tracking screen is sized against —
/// and all three of those numbers are printed beside the measurement, so a run
/// can be read against the only hardware evidence this repo has.
///
/// The screen is printed whether or not the run's detector was armed: it is the
/// figure a plant model has to replace, so what content did against it is the
/// reading that says how far off it is.
pub fn lags(samples: &[Logged<PoseSampleWire>], report: &mut Report) {
    let mut head = 0_f64;
    let mut antenna = 0_f64;
    let mut compared = 0_usize;
    for sample in samples {
        let (Some(present), Some(commanded)) = (
            present_rows(&sample.message),
            commanded_rows(&sample.message),
        ) else {
            continue;
        };
        compared += 1;
        for joint in ROWS {
            let Some(index) = row(joint) else { continue };
            let lag = (commanded[index] - present[index]).abs();
            match group_of(joint) {
                Some(JointGroup::Antennas) => antenna = antenna.max(lag),
                Some(_) => head = head.max(lag),
                None => {}
            }
        }
    }
    let threshold = TrackingFaultConfig::default().threshold_rad;
    // How many samples the two figures came off, because a zero lag and a
    // measurement nothing was compared on print the same otherwise -- and a run
    // in which the driver held nothing is the second one.
    report.note(format!(
        "{compared} of {} samples carried both a reading and a setpoint to compare",
        samples.len()
    ));
    report.note(format!(
        "worst head lag {head:.4} rad; the recorded healthy gesture ran at \
         {RECORDED_WORST_HEAD_LAG_RAD:.4} rad and the tracking screen sits at {threshold:.4} rad"
    ));
    report.note(format!(
        "worst antenna lag {antenna:.4} rad; the recorded fast sweep ran at \
         {RECORDED_WORST_ANTENNA_LAG_RAD:.4} rad"
    ));
}

/// The decision tick raised nothing.
///
/// The count travels with the kind: on a tracking fault it is the window the
/// run was judged by, which is what says whether the figure was too short or
/// the run was never recognised as a reversal, and reading it out here saves a
/// dig through the log.
///
/// One wording for every run: a fault row that read differently depending on
/// which harness recorded it would be the same event with two names.
pub fn no_faults(faults: &[Logged<TickFaultWire>], report: &mut Report) {
    for fault in faults {
        report.fail(format!(
            "the decision tick raised {:?} at {}, count {}",
            fault.message.kind(),
            fault.message.time().as_nanos(),
            fault.message.count()
        ));
    }
}

/// The grid a run's samples sit on, derived from the samples themselves.
///
/// A hardware run starts at whatever top of a second the driver started at, so
/// nothing about the epoch can be assumed; what can be is the period, which is
/// the one number both hosts are built against. The origin is the first
/// sample's own nominal instant.
#[derive(Clone, Copy)]
pub struct Grid {
    /// The instant cycle zero of this run sits at.
    pub origin_ns: i64,
    /// How long one cycle is.
    pub period_ns: i64,
}

impl Grid {
    /// The cycle index of a nominal instant, and how far off the grid it sits.
    #[must_use]
    pub fn at(&self, nominal_ns: i64) -> (i64, i64) {
        let elapsed = nominal_ns - self.origin_ns;
        (
            elapsed.div_euclid(self.period_ns),
            elapsed.rem_euclid(self.period_ns),
        )
    }

    /// The same, with an instant within `jitter_ns` of a cycle counted as being
    /// on it.
    ///
    /// An instant that arrived late is over its own cycle's mark by the offset;
    /// one that arrived early is under the *next* cycle's, which the remainder
    /// reports as nearly a whole period. So both ends of the band are checked
    /// and the answer is the cycle the instant is nearest, with a zero offset
    /// when it is inside the band.
    #[must_use]
    pub fn within(&self, nominal_ns: i64, jitter_ns: i64) -> (i64, i64) {
        let (cycle, off) = self.at(nominal_ns);
        if off <= jitter_ns {
            (cycle, 0)
        } else if off >= self.period_ns - jitter_ns {
            (cycle + 1, 0)
        } else {
            (cycle, off)
        }
    }
}

/// The grid points the driver reported missing, and the reports themselves.
///
/// A skip report is published by the first cycle attended after the run of
/// missed slots, and it says how many they were, so the slots it accounts for
/// are the ones immediately before it. Which cycles those are is what lets a
/// hole in the sample stream be recognised as the same event rather than
/// counted a second time — which is the difference between a machine reading
/// and a harness defect.
pub struct Skips<'a> {
    /// Every skip report the run carried.
    pub events: Vec<&'a Logged<DriverEventWire>>,
    /// Every cycle a report accounts for.
    pub missed: BTreeSet<i64>,
}

impl<'a> Skips<'a> {
    /// The skips `events` reports, placed on `grid`.
    #[must_use]
    pub fn of(events: &'a [Logged<DriverEventWire>], grid: Grid, jitter_ns: i64) -> Self {
        let events: Vec<&Logged<DriverEventWire>> = events
            .iter()
            .filter(|event| event.message.kind() == EventKindWire::CYCLE_SKIPPED)
            .collect();
        let mut missed = BTreeSet::new();
        for event in &events {
            let (cycle, off) = grid.within(event.message.time().as_nanos(), jitter_ns);
            // A report that does not sit on the grid places no slots: it is
            // still counted as a report, and the gap it would have explained
            // stays unexplained rather than being explained by a guess.
            if off != 0 {
                continue;
            }
            for slot in cycle - i64::from(event.message.count())..cycle {
                missed.insert(slot);
            }
        }
        Self { events, missed }
    }

    /// The slots the reports account for, all told.
    #[must_use]
    pub fn slots(&self) -> u64 {
        self.events
            .iter()
            .map(|event| u64::from(event.message.count()))
            .sum()
    }

    /// Whether every cycle in `cycles` is one a report accounts for.
    ///
    /// An empty range is accounted for by nothing and answers true, which is
    /// what a caller asking about two consecutive samples wants.
    #[must_use]
    pub fn account_for(&self, cycles: std::ops::Range<i64>) -> bool {
        cycles.into_iter().all(|slot| self.missed.contains(&slot))
    }
}

#[cfg(test)]
mod tests {
    //! The grid arithmetic and the skip accounting, which is where the two
    //! analyzers agree about what a hole in the stream means.

    use brenn_reachy__driver__health_clk_rs::{DriverEventWire, EventKindWire};
    use clockwork_rs::SyncTime;
    use log_read::Logged;

    use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
    use brenn_reachy__motion__faults_clk_rs::{FaultKindWire, TickFaultWire};
    use reachy_motion::joints::{JointRef, ROW_COUNT, row, write_rows};
    use run_report::Report;

    use super::{Grid, Skips, lags, no_faults};

    /// A period nothing round, so an arithmetic that assumed one shows.
    const PERIOD_NS: i64 = 20_000_000;

    /// An origin nothing round, for the same reason.
    const ORIGIN_NS: i64 = 1_772_000_000_123_456_789;

    /// The grid a case reads instants against.
    fn grid() -> Grid {
        Grid {
            origin_ns: ORIGIN_NS,
            period_ns: PERIOD_NS,
        }
    }

    /// A skip report at cycle `n` accounting for `count` slots before it.
    fn skipped(n: i64, count: u32) -> Logged<DriverEventWire> {
        let mut message = DriverEventWire::new();
        message.set_kind(EventKindWire::CYCLE_SKIPPED);
        message.set_time(SyncTime::from_nanos(ORIGIN_NS + n * PERIOD_NS));
        message.set_count(count);
        Logged {
            at_ns: ORIGIN_NS + n * PERIOD_NS,
            sequence_number: u32::try_from(n).unwrap_or(0),
            message,
        }
    }

    /// One sample: read at `present`, holding `commanded`.
    fn sample(
        n: i64,
        present: &[f64; ROW_COUNT],
        commanded: &[f64; ROW_COUNT],
    ) -> Logged<PoseSampleWire> {
        let mut message = PoseSampleWire::new();
        {
            let read = message.clear_valid();
            read.nominal_time = SyncTime::from_nanos(ORIGIN_NS + n * PERIOD_NS);
            read.sample_time = read.nominal_time;
            read.present_valid = true.into();
            read.commanded_valid = true.into();
            write_rows(&mut read.present, present);
            write_rows(&mut read.commanded, commanded);
        }
        Logged {
            at_ns: ORIGIN_NS + n * PERIOD_NS,
            sequence_number: u32::try_from(n).unwrap_or(0),
            message,
        }
    }

    /// The head and the antennas are two figures, and which joint belongs to
    /// which group is the reason this crate exists: two analyzers print these
    /// numbers side by side, and a joint counted into the wrong group makes
    /// both reports consistently wrong.
    #[test]
    fn the_two_lag_figures_are_the_head_and_the_antennas_apart() {
        let mut present = [0.0; ROW_COUNT];
        present[row(JointRef::Leg2).expect("a bus row")] = 0.25;
        present[row(JointRef::AntennaLeft).expect("a bus row")] = 1.5;
        let samples = vec![
            sample(0, &present, &[0.0; ROW_COUNT]),
            sample(1, &present, &[0.0; ROW_COUNT]),
        ];
        let mut report = Report::default();
        lags(&samples, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("worst head lag 0.2500 rad")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("worst antenna lag 1.5000 rad")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("2 of 2 samples carried both")),
            "{:?}",
            report.measured
        );
    }

    /// A sample carrying no setpoint is compared against nothing, and the count
    /// says so: a run in which the driver held nothing would otherwise print
    /// the same zero as a run that tracked perfectly.
    #[test]
    fn a_sample_with_no_setpoint_is_not_compared() {
        let mut held = sample(1, &[0.5; ROW_COUNT], &[0.0; ROW_COUNT]);
        held.message.set_commanded_valid(false);
        let samples = vec![sample(0, &[0.0; ROW_COUNT], &[0.0; ROW_COUNT]), held];
        let mut report = Report::default();
        lags(&samples, &mut report);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("1 of 2 samples carried both")),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("worst head lag 0.0000 rad")),
            "the uncompared sample's half-radian is not a lag: {:?}",
            report.measured
        );
    }

    /// A fault the tick raised is a finding carrying the kind, the instant and
    /// the window it was judged over.
    #[test]
    fn a_fault_is_reported_with_its_kind_its_instant_and_its_count() {
        let mut fault = TickFaultWire::new();
        fault.set_kind(FaultKindWire::HEAD_SERVO_FAULT);
        fault.set_time(SyncTime::from_nanos(ORIGIN_NS + 3 * PERIOD_NS));
        fault.set_count(4);
        let faults = vec![Logged {
            at_ns: ORIGIN_NS + 3 * PERIOD_NS,
            sequence_number: 0,
            message: fault,
        }];
        let mut report = Report::default();
        no_faults(&faults, &mut report);
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        let said = &report.findings[0];
        assert!(said.contains("HEAD_SERVO_FAULT"), "{said}");
        assert!(
            said.contains(&(ORIGIN_NS + 3 * PERIOD_NS).to_string()),
            "{said}"
        );
        assert!(said.contains("count 4"), "{said}");
    }

    #[test]
    fn an_instant_on_the_grid_is_its_own_cycle_and_nothing_off_it() {
        assert_eq!(grid().at(ORIGIN_NS + 7 * PERIOD_NS), (7, 0));
        assert_eq!(grid().at(ORIGIN_NS + 7 * PERIOD_NS + 3), (7, 3));
    }

    #[test]
    fn an_instant_inside_the_jitter_band_is_on_the_cycle_it_is_nearest() {
        // Late for its own cycle, and early for the next one: both ends of the
        // band answer with a cycle and no offset.
        assert_eq!(grid().within(ORIGIN_NS + 7 * PERIOD_NS + 3, 5), (7, 0));
        assert_eq!(grid().within(ORIGIN_NS + 8 * PERIOD_NS - 3, 5), (8, 0));
        assert_eq!(
            grid().within(ORIGIN_NS + 7 * PERIOD_NS + 3, 0),
            (7, 3),
            "with no band allowed, an instant off the grid stays off it",
        );
    }

    #[test]
    fn a_report_accounts_for_the_slots_immediately_before_it() {
        let events = vec![skipped(10, 3)];
        let skips = Skips::of(&events, grid(), 0);
        assert_eq!(skips.slots(), 3);
        assert!(skips.account_for(7..10));
        assert!(
            !skips.account_for(6..10),
            "a gap wider than the report is a hole the driver did not explain",
        );
        assert!(!skips.account_for(10..11));
    }

    #[test]
    fn a_report_off_the_grid_places_no_slots() {
        let mut events = vec![skipped(10, 3)];
        events[0].message.set_time(SyncTime::from_nanos(
            ORIGIN_NS + 10 * PERIOD_NS + PERIOD_NS / 2,
        ));
        let skips = Skips::of(&events, grid(), 0);
        assert_eq!(skips.slots(), 3, "the report is still a report");
        assert!(
            !skips.account_for(7..10),
            "where the report sits is a guess, and a guess explains nothing",
        );
    }
}
