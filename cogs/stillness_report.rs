//! Whether the machine held still, read off a driver sample stream.
//!
//! The reduction both log analyzers in this package need over `DriverPose`, in
//! one crate for the reason `motion_evidence` is one: the two of them must not
//! disagree about it. `first_motion_report` asks whether the harness gesture's
//! long hold was still; `speech_run_report` asks the same of every listening
//! hold a conversation left behind. The segmentation, the figures and the lines
//! they are printed on are the same in both.
//!
//! The measurement itself is `reachy_motion::stillness`, which is sans-I/O and
//! knows nothing about a log. What is here is the two things a report needs
//! beside it: turning a recorded sample into the measurement's own view of a
//! cycle, and saying what came out in the words an operator reads.
//!
//! The fold is streaming, because one of the two callers reads a channel whose
//! message count is however long somebody talked to the robot. Nothing is kept
//! per cycle, and nothing kept grows without bound: the head rows fold into one
//! running worst apiece, and the antenna rows are printed a line each up to a
//! cap and fold into a running worst apiece past it. An hour of conversation
//! with a gesture every few seconds is hundreds of holds, and a section
//! hundreds of lines long is one an operator learns to skim — which is the one
//! thing the report it rides in must not become. The cap is high enough that
//! the harness run, whose whole point is one long hold, prints every hold it
//! has.
//!
//! Only the antennas are judged. Nobody has complained about a head row and
//! this package has no measurement to bound one with, so the head is reported
//! as its worst hold apiece and never fails — the baseline for the day somebody
//! does complain. What a failing antenna hold *means* is the caller's: the
//! harness report exists to produce one long hold and judge it, and the speech
//! report prints the same finding as a figure until the bound has been baked
//! from a hardware run.

use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
use motion_slots::joint_set;
use reachy_motion::joints::{
    JointGroup, JointVector, Name, ROW_COUNT, ROWS, group_of, row, vector_of, write_rows,
};
use reachy_motion::stillness::{
    COUNT_RAD, HoldWindow, Sample, StillnessConfig, StillnessCounts, StillnessWatch, judge,
};
use run_report::Report;

/// How many antenna holds are printed one line each before the rest fold into
/// one worst apiece.
///
/// Well past what any harness run holds and well short of what an afternoon's
/// conversation would. The verdict does not depend on it: what folds is still
/// judged, as the widest of what it folded.
const ANTENNA_LINES: usize = 24;

/// Whether a hold past the bound is a finding or a figure.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Standard {
    /// An antenna hold past the bound fails the run, and a run holding no
    /// antenna still long enough to judge fails too.
    Judged,
    /// Every hold is a number, whatever it says.
    Printed,
}

/// The stillness measurement over one recorded sample stream.
///
/// Fed the samples of a run in order, then [`Self::finish`]ed. The windows it
/// has closed are what [`say`] prints; the counts beside them are what
/// distinguishes a run nothing could be judged over from one that was.
#[derive(Debug)]
pub struct Stillness {
    watch: StillnessWatch,
    /// The first [`ANTENNA_LINES`] antenna holds: those are what is judged, and
    /// what a reader of a failing run wants one line each of.
    windows: Vec<HoldWindow>,
    /// One entry per antenna row that held past that cap, folded as the head
    /// rows are and judged as the holds above are.
    beyond: Vec<Head>,
    /// One entry per head row that held at all, and nothing more however many
    /// holds a run contains.
    heads: Vec<Head>,
    /// Where the watch hands a closed hold over, drained on every sample so it
    /// never grows and nothing allocates per cycle.
    closed: Vec<HoldWindow>,
    /// When the first sample was taken, which every window's start is printed
    /// relative to: an operator reads a hold as "so far into the run".
    first_ns: Option<i64>,
    /// Samples this build could not make sense of, and so fed to nothing.
    unreadable: usize,
}

/// One head row's holds, folded: the widest one, and how many there were.
///
/// The head is a baseline rather than a verdict, and a conversation holds the
/// head as often as it is left alone, so keeping every hold would make the
/// section as long as the run. The worst is the figure that would start the
/// conversation the day somebody complains about a head row.
#[derive(Clone, Copy, Debug)]
pub struct Head {
    /// The widest hold this row showed.
    pub worst: HoldWindow,
    /// How many holds it was the widest of.
    pub holds: usize,
}

impl Default for Stillness {
    fn default() -> Self {
        Self {
            watch: StillnessWatch::new(StillnessConfig::default(), &ROWS),
            windows: Vec::new(),
            beyond: Vec::new(),
            heads: Vec::new(),
            closed: Vec::new(),
            first_ns: None,
            unreadable: 0,
        }
    }
}

impl Stillness {
    /// Take in one recorded cycle.
    ///
    /// On the sample's own clock — when the bus read completed — rather than
    /// the instant the logger wrote it down or the grid instant the cycle was
    /// due at: the excursion is a property of the readings, so the readings'
    /// own timestamps are what a hold is measured in.
    ///
    /// A sample whose servo set or whose angles this build cannot read is
    /// counted and fed to nothing. Not silently: a stream of them reads as a
    /// run with no holds in it, and the count beside the summary is what says
    /// which of the two happened.
    pub fn sample(&mut self, sample: &PoseSampleWire) {
        let Ok(missing) = joint_set(sample.missing()) else {
            self.unreadable += 1;
            return;
        };
        let Ok(present) = sample.present().validate() else {
            self.unreadable += 1;
            return;
        };
        let present = vector_of(present);
        // The wire calls the commanded row meaningless when it says so, and a
        // cycle the driver held nothing through is a cycle with a good reading
        // in it: only a row the driver claims is a setpoint has to be readable.
        let commanded = if sample.commanded_valid() {
            let Ok(commanded) = sample.commanded().validate() else {
                self.unreadable += 1;
                return;
            };
            vector_of(commanded)
        } else {
            JointVector::default()
        };
        let t_ns = sample.sample_time().as_nanos();
        self.first_ns.get_or_insert(t_ns);
        self.watch.look(
            &Sample {
                t_ns,
                present_valid: sample.present_valid(),
                commanded_valid: sample.commanded_valid(),
                missing,
                present: &present,
                commanded: &commanded,
            },
            &mut self.closed,
        );
        self.take_closed();
    }

    /// Sort what the watch just closed: antenna holds kept, head holds folded.
    fn take_closed(&mut self) {
        for window in self.closed.drain(..) {
            let antenna = group_of(window.joint) == Some(JointGroup::Antennas);
            if antenna && self.windows.len() < ANTENNA_LINES {
                self.windows.push(window);
                continue;
            }
            let into = if antenna {
                &mut self.beyond
            } else {
                &mut self.heads
            };
            match into
                .iter_mut()
                .find(|head| head.worst.joint == window.joint)
            {
                Some(head) => {
                    head.holds += 1;
                    if window.excursion_rad > head.worst.excursion_rad {
                        head.worst = window;
                    }
                }
                None => into.push(Head {
                    worst: window,
                    holds: 1,
                }),
            }
        }
    }

    /// End of the stream: close the last hold of the run.
    ///
    /// A recording simply stops, and the hold it stopped during is as much a
    /// hold as any other. Idempotent, so a caller that cannot tell whether it
    /// has finished may call it again.
    pub fn finish(&mut self) {
        self.watch.finish(&mut self.closed);
        self.take_closed();
    }

    /// The antenna holds printed a line each: every one the run held, up to
    /// [`ANTENNA_LINES`].
    #[must_use]
    pub fn windows(&self) -> &[HoldWindow] {
        &self.windows
    }

    /// The antenna rows that held past that cap, one folded entry each.
    #[must_use]
    pub fn beyond(&self) -> &[Head] {
        &self.beyond
    }

    /// The head rows that held, one folded entry each.
    #[must_use]
    pub fn heads(&self) -> &[Head] {
        &self.heads
    }

    /// What the pass saw.
    #[must_use]
    pub fn counts(&self) -> StillnessCounts {
        self.watch.counts()
    }

    /// The bounds and allowances the holds were measured against.
    #[must_use]
    pub fn config(&self) -> StillnessConfig {
        self.watch.config()
    }

    /// How many samples this build could not read.
    #[must_use]
    pub const fn unreadable(&self) -> usize {
        self.unreadable
    }

    /// How far into the run `at_ns` is, seconds, or the bare instant where
    /// nothing has been read yet to measure it from.
    fn run_offset(&self, at_ns: i64) -> String {
        match self.first_ns {
            Some(first) => format!("+{:.2} s", (at_ns - first) as f64 / 1e9),
            None => format!("{at_ns} ns"),
        }
    }
}

/// One hold, as an operator reads it.
///
/// The excursion twice, in counts and in radians: counts are what the bound is
/// argued in — a still joint reads one, or flickers between it and its
/// neighbour — and radians are what every other figure in these reports is in.
fn line(held: &Stillness, window: &HoldWindow) -> String {
    format!(
        "  {} over a {:.2} s hold from {}: {:.1} counts ({:.4} rad) peak to peak, {:.1} \
         reversals/s, mean error {:+.4} rad, over {} reading(s), opened {:.2} s after the \
         setpoint last moved at {:+.4} rad of error",
        Name(window.joint),
        window.length().as_secs_f64(),
        held.run_offset(window.start_ns),
        window.excursion_counts(),
        window.excursion_rad,
        window.reversals_per_s,
        window.mean_error_rad,
        window.samples,
        window.opened_after_ns as f64 / 1e9,
        window.error_at_open_rad
    )
}

/// Say what the run held still for, and — under [`Standard::Judged`] — what it
/// did not.
///
/// Every antenna hold is printed up to the cap and the rest fold to one line
/// per row; each head row is printed once, as the widest hold it showed. The
/// judging is the antennas' alone, folded or not, and the two sentences it can
/// produce are the ones the bring-up assertion is made of: a hold that moved
/// further than a still joint may, and a run that never held one long enough to
/// ask.
pub fn say(held: &Stillness, standard: Standard, report: &mut Report) {
    let counts = held.counts();
    let cfg = held.config();
    // Every figure in the first line is counted per row: one posture change is
    // a goal change for each of the nine, and one hold of the machine is nine
    // row-holds. Said that way because the alternative reads as nine postures.
    report.note(format!(
        "stillness: {} row-hold(s) judged across {} watched row(s), {} discarded as too short or \
         too sparse, {} goal change(s) counted the same way, over {} sample(s){}",
        counts.judged,
        ROWS.len(),
        counts.discarded_short,
        counts.goal_changes,
        counts.samples,
        match (held.unreadable(), counts.non_finite) {
            (0, 0) => String::new(),
            (unreadable, 0) => format!(", and {unreadable} sample(s) this build could not read"),
            (0, non_finite) => format!(", and {non_finite} reading(s) that were not numbers"),
            (unreadable, non_finite) => format!(
                ", and {unreadable} sample(s) this build could not read, {non_finite} \
                 reading(s) that were not numbers"
            ),
        }
    ));
    report.note(format!(
        "  a hold is {:.1} s of one setpoint after a {:.1} s settle, and a still joint stays \
         inside {:.1} counts ({:.4} rad)",
        cfg.min_hold.as_secs_f64(),
        cfg.settle.as_secs_f64(),
        cfg.max_excursion_rad / COUNT_RAD,
        cfg.max_excursion_rad
    ));
    for window in held.windows() {
        report.note(line(held, window));
        if let Err(err) = judge(window, &cfg) {
            match standard {
                Standard::Judged => report.fail(err.to_string()),
                Standard::Printed => report.note(err.to_string()),
            }
        }
    }
    for folded in held.beyond() {
        report.note(format!(
            "{}, the widest of {} further hold(s) this row held",
            line(held, &folded.worst),
            folded.holds
        ));
        if let Err(err) = judge(&folded.worst, &cfg) {
            match standard {
                Standard::Judged => report.fail(err.to_string()),
                Standard::Printed => report.note(err.to_string()),
            }
        }
    }
    for head in held.heads() {
        report.note(format!(
            "{}, the widest of {} hold(s) this row held",
            line(held, &head.worst),
            head.holds
        ));
    }
    if !held.windows().is_empty() {
        return;
    }
    let says = format!(
        "no antenna held one setpoint for {:.1} s after a {:.1} s settle, so this run says \
         nothing about whether the antennas stand still: {} sample(s) read, {} goal change(s) \
         over them",
        cfg.min_hold.as_secs_f64(),
        cfg.settle.as_secs_f64(),
        counts.samples,
        counts.goal_changes
    );
    match standard {
        Standard::Judged => report.fail(says),
        Standard::Printed => report.note(says),
    }
}

/// The synthetic run every suite that touches this section is written against.
///
/// Not test-gated, because the two report crates that print this section are
/// each their own compilation unit and each needs it: the figures a hunting
/// antenna reads as are pinned once, here, and a report suite proves only that
/// it wired the section up and asked for the right standard.
pub mod fixture {
    use super::{PoseSampleWire, ROW_COUNT, row, write_rows};
    use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointFlagsWire};
    use clockwork_rs::SyncTime;
    use reachy_motion::joints::{JointRef, flags};

    /// One recorded cycle at `t_ns`: every row reading `present`, and — when
    /// the driver was holding one — every row held at `commanded`.
    #[must_use]
    pub fn cycle(
        t_ns: i64,
        present: &[f64; ROW_COUNT],
        commanded: Option<&[f64; ROW_COUNT]>,
    ) -> PoseSampleWire {
        let mut msg = PoseSampleWire::new();
        {
            let read = msg.clear_valid();
            read.nominal_time = SyncTime::from_nanos(t_ns);
            read.sample_time = SyncTime::from_nanos(t_ns);
            read.present_valid = true.into();
            write_rows(&mut read.present, present);
            if let Some(goals) = commanded {
                read.commanded_valid = true.into();
                write_rows(&mut read.commanded, goals);
            }
        }
        msg
    }

    /// The same cycle with its reading marked incomplete.
    ///
    /// Models a bus-quiet cycle: a sample arrives every cycle regardless,
    /// carrying whatever was last read.
    #[must_use]
    pub fn blind(mut cycle: PoseSampleWire) -> PoseSampleWire {
        cycle.set_present_valid(false);
        cycle
    }

    /// The same cycle with `joints` named as the rows that did not answer.
    ///
    /// A partial read: the rest of the row set is a reading like any other.
    #[must_use]
    pub fn absent(mut cycle: PoseSampleWire, joints: &[JointRef]) -> PoseSampleWire {
        let mut set = JointFlags::NONE;
        for joint in joints {
            flags::insert(&mut set, *joint);
        }
        cycle.set_missing(JointFlagsWire::from(set));
        cycle
    }

    /// The one row `joint` occupies, holding `value` and the rest at zero.
    #[must_use]
    pub fn only(joint: JointRef, value: f64) -> [f64; ROW_COUNT] {
        let mut rows = [0.0; ROW_COUNT];
        rows[row(joint).expect("a bus row")] = value;
        rows
    }

    /// A run of `cycles` on a `period_ns` grid from `start_ns`, every row held
    /// at zero, both antennas reading `wobble` radians either side of it and
    /// alternating every cycle.
    ///
    /// Long enough to be judged only past the settle allowance plus the
    /// shortest hold, which is six seconds of it.
    #[must_use]
    pub fn holding(start_ns: i64, period_ns: i64, cycles: i64, wobble: f64) -> Vec<PoseSampleWire> {
        (0..cycles)
            .map(|n| {
                let swing = if n % 2 == 0 { wobble } else { -wobble };
                let mut present = [0.0; ROW_COUNT];
                present[row(JointRef::AntennaRight).expect("a bus row")] = swing;
                present[row(JointRef::AntennaLeft).expect("a bus row")] = swing;
                cycle(start_ns + n * period_ns, &present, Some(&[0.0; ROW_COUNT]))
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::fixture;
    use super::{ROW_COUNT, Standard, Stillness, say};
    use brenn_reachy__driver__pose_clk_rs::PoseSampleWire;
    use reachy_motion::joints::JointRef;
    use reachy_motion::stillness::COUNT_RAD;
    use run_report::Report;

    /// The driver's cycle, nanoseconds.
    const PERIOD_NS: i64 = 20_000_000;

    /// An arbitrary instant a synthetic run starts at, chosen for being nothing
    /// round.
    const T0: i64 = 1_772_000_000_123_456_789;

    /// One recorded cycle: `joint` reading `present` and, when the driver held
    /// one, held at `commanded`; every other row at zero.
    fn sample(n: i64, joint: JointRef, present: f64, commanded: Option<f64>) -> PoseSampleWire {
        fixture::cycle(
            T0 + n * PERIOD_NS,
            &fixture::only(joint, present),
            commanded.map(|goal| fixture::only(joint, goal)).as_ref(),
        )
    }

    /// A run of `cycles` in which `joint` is held at zero and reads `wobble`
    /// radians either side of it, alternating every cycle.
    fn wobbling(joint: JointRef, cycles: i64, wobble: f64) -> Stillness {
        let mut held = Stillness::default();
        for n in 0..cycles {
            let present = if n % 2 == 0 { wobble } else { -wobble };
            held.sample(&sample(n, joint, present, Some(0.0)));
        }
        held.finish();
        held
    }

    /// What one report would print and find over `held`.
    fn said(held: &Stillness, standard: Standard) -> Report {
        let mut report = Report::default();
        say(held, standard, &mut report);
        report
    }

    /// Whether anything in `lines` says `what`.
    fn says(lines: &[String], what: &str) -> bool {
        lines.iter().any(|line| line.contains(what))
    }

    /// A run of nothing but held setpoints and steady readings is a run with a
    /// hold in it, and the hold is still.
    #[test]
    fn a_steady_antenna_is_a_hold_that_passes() {
        let held = wobbling(JointRef::AntennaRight, 500, 0.0);
        let report = said(&held, Standard::Judged);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            says(&report.measured, "right antenna over a"),
            "{:?}",
            report.measured
        );
        assert_eq!(
            held.counts().judged,
            ROW_COUNT,
            "every row was held at one setpoint, so every row has a hold"
        );
    }

    /// The bring-up assertion: an antenna crossing the bound is a finding in
    /// the harness report, naming the joint, the excursion and the hold.
    #[test]
    fn a_hunting_antenna_fails_where_it_is_judged() {
        let held = wobbling(JointRef::AntennaRight, 500, 3.0 * COUNT_RAD);
        let report = said(&held, Standard::Judged);
        assert_eq!(report.findings.len(), 1, "{:?}", report.findings);
        let finding = &report.findings[0];
        assert!(finding.contains("right antenna"), "{finding}");
        assert!(finding.contains("6.0 counts"), "{finding}");
        assert!(finding.contains("reversing"), "{finding}");
    }

    /// The same hold, in the report whose section is a set of figures: the same
    /// sentence, on the other list.
    #[test]
    fn a_hunting_antenna_is_a_figure_where_it_is_printed() {
        let held = wobbling(JointRef::AntennaRight, 500, 3.0 * COUNT_RAD);
        let report = said(&held, Standard::Printed);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            says(&report.measured, "6.0 counts"),
            "{:?}",
            report.measured
        );
    }

    /// A head row that hunts is printed with the same three figures and fails
    /// nothing: nobody has complained about one, and this package has no
    /// measurement to bound it with.
    #[test]
    fn a_hunting_head_row_is_printed_and_never_judged() {
        let held = wobbling(JointRef::Leg2, 500, 10.0 * COUNT_RAD);
        let report = said(&held, Standard::Judged);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            says(&report.measured, "leg 3 over a"),
            "{:?}",
            report.measured
        );
        assert!(
            says(&report.measured, "20.0 counts"),
            "the head row's excursion is printed with the rest: {:?}",
            report.measured
        );
    }

    /// A run too short to hold anything fails the harness report and notes in
    /// the other: the harness exists to produce one long hold.
    #[test]
    fn a_run_with_no_long_hold_is_a_finding_only_where_it_is_judged() {
        let mut held = Stillness::default();
        for n in 0..50 {
            held.sample(&sample(n, JointRef::AntennaRight, 0.0, Some(0.0)));
        }
        held.finish();
        assert!(held.windows().is_empty());
        assert!(
            says(
                &said(&held, Standard::Judged).findings,
                "no antenna held one setpoint"
            ),
            "the harness report fails a run it could judge nothing over"
        );
        let printed = said(&held, Standard::Printed);
        assert!(printed.findings.is_empty(), "{:?}", printed.findings);
        assert!(
            says(&printed.measured, "no antenna held one setpoint"),
            "{:?}",
            printed.measured
        );
    }

    /// A driver holding nothing holds no setpoint to judge a joint against,
    /// however long it read cleanly.
    #[test]
    fn a_run_holding_nothing_holds_no_window() {
        let mut held = Stillness::default();
        for n in 0..500 {
            held.sample(&sample(n, JointRef::AntennaRight, 0.0, None));
        }
        held.finish();
        assert!(held.windows().is_empty());
        assert_eq!(held.counts().samples, 500);
    }

    /// The summary line is the pass's own account: how many holds were judged,
    /// how many were dropped, and what the bound was.
    #[test]
    fn the_summary_says_what_the_pass_saw() {
        let held = wobbling(JointRef::AntennaLeft, 500, 0.0);
        let report = said(&held, Standard::Judged);
        assert!(
            says(
                &report.measured,
                "stillness: 9 row-hold(s) judged across 9 watched row(s)"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            says(&report.measured, "2.0 counts"),
            "the bound is printed beside the holds it judged: {:?}",
            report.measured
        );
    }

    /// A run with hold after hold in it — a conversation that was left alone
    /// between gestures — says one line per head row however many there were,
    /// so the section's length is the machine's shape and not the run's.
    /// A run of `holds` holds of ten seconds each, ended by a goal change, in
    /// which every head row wobbles wider on each successive hold: hold `n`
    /// swings `n + 1` counts either side of its setpoint.
    fn hold_after_hold(holds: i32) -> Stillness {
        let mut held = Stillness::default();
        let mut n = 0_i64;
        for hold in 0..holds {
            let swing = f64::from(hold + 1) * COUNT_RAD;
            for cycle in 0..500 {
                let reading = f64::from(hold) + if cycle % 2 == 0 { swing } else { -swing };
                let present = [reading; ROW_COUNT];
                held.sample(&fixture::cycle(
                    T0 + n * PERIOD_NS,
                    &present,
                    Some(&[f64::from(hold); ROW_COUNT]),
                ));
                n += 1;
            }
        }
        held.finish();
        held
    }

    #[test]
    fn the_head_rows_fold_however_many_holds_a_run_holds() {
        let held = hold_after_hold(6);
        let report = said(&held, Standard::Judged);
        assert_eq!(held.heads().len(), 7, "one entry per head row");
        assert_eq!(
            report
                .measured
                .iter()
                .filter(|line| line.contains("leg 3 over a"))
                .count(),
            1,
            "{:?}",
            report.measured
        );
        assert!(
            says(&report.measured, "the widest of 6 hold(s) this row held"),
            "{:?}",
            report.measured
        );
        // The sixth hold swings six counts either side: the widest of the six,
        // and the one the baseline has to keep.
        assert!(
            says(&report.measured, "leg 3 over a 5.98 s hold"),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("leg 3 over a") && line.contains("12.0 counts")),
            "the widest head hold is not the one kept: {:?}",
            report.measured
        );
        assert_eq!(
            held.windows().len(),
            12,
            "both antennas, every hold: under the cap they are kept in full"
        );
        assert!(held.beyond().is_empty(), "{:?}", held.beyond());
    }

    /// Past the cap the antenna rows fold the way the head rows do, so an
    /// afternoon of conversation is not an afternoon of lines — and what folded
    /// is still judged, as the widest of what it folded.
    #[test]
    fn antenna_holds_past_the_cap_fold_and_are_still_judged() {
        let held = hold_after_hold(20);
        assert_eq!(held.windows().len(), 24, "the cap, and no further");
        assert_eq!(held.beyond().len(), 2, "one folded entry per antenna");
        let report = said(&held, Standard::Judged);
        assert_eq!(
            report
                .measured
                .iter()
                .filter(|line| line.contains("right antenna over a"))
                .count(),
            13,
            "twelve lines under the cap and one fold: {:?}",
            report.measured
        );
        assert!(
            says(
                &report.measured,
                "the widest of 8 further hold(s) this row held"
            ),
            "{:?}",
            report.measured
        );
        // The twentieth hold swings twenty counts either side, is past the cap,
        // and is past the bound: folding it must not lose the finding.
        assert!(
            report
                .findings
                .iter()
                .any(|line| line.contains("right antenna moved 40.0 counts")),
            "{:?}",
            report.findings
        );
    }

    /// A bus that went quiet mid-hold is a gap in the reading, not the end of
    /// the hold and not a sample this build could not read.
    #[test]
    fn a_blind_stretch_mid_hold_neither_splits_it_nor_reads_as_unreadable() {
        let mut held = Stillness::default();
        for n in 0..500 {
            let cycle = fixture::cycle(
                T0 + n * PERIOD_NS,
                &[0.0; ROW_COUNT],
                Some(&[0.0; ROW_COUNT]),
            );
            held.sample(&if (250..275).contains(&n) {
                fixture::blind(cycle)
            } else {
                cycle
            });
        }
        held.finish();
        assert_eq!(held.unreadable(), 0, "a blind cycle is a reading gap");
        assert_eq!(held.counts().samples, 500);
        assert_eq!(held.windows().len(), 2, "{:?}", held.windows());
        let window = held.windows()[0];
        assert_eq!(window.samples, 500 - 200 - 25);
        assert!(
            (window.length().as_secs_f64() - 5.98).abs() < 1e-6,
            "{window:?}"
        );
    }

    /// One row that did not answer is one row's gap: the rows beside it are
    /// read, and the row itself keeps the hold it was in.
    #[test]
    fn one_row_that_did_not_answer_does_not_split_the_hold() {
        let mut held = Stillness::default();
        for n in 0..500 {
            let cycle = fixture::cycle(
                T0 + n * PERIOD_NS,
                &[0.0; ROW_COUNT],
                Some(&[0.0; ROW_COUNT]),
            );
            held.sample(&if (250..275).contains(&n) {
                fixture::absent(cycle, &[JointRef::AntennaRight])
            } else {
                cycle
            });
        }
        held.finish();
        assert_eq!(held.unreadable(), 0);
        assert_eq!(held.windows().len(), 2, "{:?}", held.windows());
        let right = held
            .windows()
            .iter()
            .find(|window| window.joint == JointRef::AntennaRight)
            .expect("the right antenna held");
        let left = held
            .windows()
            .iter()
            .find(|window| window.joint == JointRef::AntennaLeft)
            .expect("the left antenna held");
        assert_eq!(right.start_ns, left.start_ns, "one gap, not one split");
        assert_eq!(right.end_ns, left.end_ns);
        assert_eq!(right.samples, left.samples - 25);
    }

    /// Read jitter puts one sample's completion before its predecessor's. The
    /// hold is the stretch it always was: a hold reported short, or discarded,
    /// would read to an operator as a machine that never stood still.
    #[test]
    fn a_sample_out_of_order_does_not_shorten_the_hold() {
        let mut held = Stillness::default();
        for n in 0..500 {
            // Every third cycle completes 2 ms late, so the one after it
            // completes before its predecessor did.
            let jitter = if n % 3 == 0 { 2_000_000 } else { 0 };
            held.sample(&fixture::cycle(
                T0 + n * PERIOD_NS + jitter,
                &[0.0; ROW_COUNT],
                Some(&[0.0; ROW_COUNT]),
            ));
        }
        held.finish();
        assert_eq!(held.windows().len(), 2, "{:?}", held.windows());
        let window = held.windows()[0];
        // Within a sample of the run without the jitter in it: the allowance
        // runs from a first sample that was itself late.
        assert!(window.samples >= 500 - 200 - 1, "{window:?}");
        assert!(window.length().as_secs_f64() > 5.9, "{window:?}");
    }

    /// A sample naming a set of servos this build cannot read is counted rather
    /// than read as a clean cycle, and the count is printed.
    #[test]
    fn samples_this_build_cannot_read_are_counted_and_said() {
        let mut held = Stillness::default();
        for n in 0..10 {
            let mut msg = sample(n, JointRef::AntennaRight, 0.0, Some(0.0));
            msg.set_missing(brenn_reachy__motion__joints_clk_rs::JointFlagsWire(0xFFFF));
            held.sample(&msg);
        }
        held.finish();
        assert_eq!(held.unreadable(), 10);
        assert_eq!(held.counts().samples, 0);
        assert!(
            says(
                &said(&held, Standard::Printed).measured,
                "10 sample(s) this build could not read"
            ),
            "the unreadable samples went unsaid"
        );
    }
}
