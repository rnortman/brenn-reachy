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
    COUNT_RAD, HoldWindow, Sample, StillnessConfig, StillnessCounts, StillnessWatch, Wobbled, judge,
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
    /// The same, over the antenna holds the head stood still across; a hold the
    /// head moved during is printed and not judged, and a run that holds none
    /// with the head still says so without failing.
    ///
    /// For a run whose content moves the head while the antennas hold: the rod
    /// follows the platform it is mounted on, so such a hold reads the head's
    /// motion rather than the antenna's loop, and the bound is written for the
    /// loop. A run that holds the antennas only under head motion has measured
    /// nothing about them, which is a reading and not a defect.
    JudgedWhereHeadStill,
    /// The same qualifier, over a run that exists to produce such a hold: one
    /// that holds none fails.
    ///
    /// For a probe run. A probe steps the antennas to a pose and holds it with
    /// the head standing at the raised base, so every one of its holds is a
    /// head-still hold; a report of none is a run whose stimulus did not
    /// arrive or whose head was commanded across it, and a green verdict there
    /// would make a rung's *quiet* reading vacuously true over nothing.
    JudgedWhereHeadStillRequired,
    /// Every hold is a number, whatever it says.
    Printed,
}

impl Standard {
    /// Whether an antenna hold is judged only where the head stood still
    /// across it.
    const fn head_still_only(self) -> bool {
        matches!(
            self,
            Self::JudgedWhereHeadStill | Self::JudgedWhereHeadStillRequired
        )
    }
}

/// The stillness measurement over one recorded sample stream.
///
/// Fed the samples of a run in order, then [`Self::finish`]ed. The windows it
/// has closed are what [`say`] prints; the counts beside them are what
/// distinguishes a run nothing could be judged over from one that was.
#[derive(Debug)]
pub struct Stillness {
    /// The measurement, told that the head rows are the platform the antennas
    /// are mounted on: every hold it closes says whether the head stood still
    /// across it.
    watch: StillnessWatch,
    /// The first [`ANTENNA_LINES`] antenna holds: those are what is judged, and
    /// what a reader of a failing run wants one line each of.
    windows: Vec<Hold>,
    /// One entry per antenna row that held past that cap, folded as the head
    /// rows are and judged as the holds above are — one per row per side of the
    /// head-still question, so a hold the head moved through cannot fold away
    /// the one hold a run took with the head still.
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

/// Every row that is not an antenna: the platform the antennas are mounted on.
///
/// The body yaw and the six legs, taken off the group rather than listed, so a
/// machine with another row in its head carries it here without an edit.
fn head_rows() -> Vec<reachy_motion::joints::JointRef> {
    ROWS.into_iter()
        .filter(|joint| group_of(*joint) != Some(JointGroup::Antennas))
        .collect()
}

/// One hold, and whether the head stood still across it.
///
/// The flag is the hold's own property and is taken on every run: what a
/// report does with it is the [`Standard`] it prints under.
#[derive(Clone, Copy, Debug)]
pub struct Hold {
    /// What the joint did.
    pub window: HoldWindow,
    /// Whether no head row was commanded somewhere new between this hold's
    /// setpoint change and its last judged reading.
    pub head_still: bool,
}

/// One row's holds, folded: the widest one, and how many there were.
///
/// The head is a baseline rather than a verdict, and a conversation holds the
/// head as often as it is left alone, so keeping every hold would make the
/// section as long as the run. The worst is the figure that would start the
/// conversation the day somebody complains about a head row.
#[derive(Clone, Copy, Debug)]
pub struct Head {
    /// The widest hold this row showed, and whether the head stood still
    /// across it. An antenna row folds one of these per side of that question.
    pub worst: Hold,
    /// How many holds it was the widest of.
    pub holds: usize,
}

impl Default for Stillness {
    fn default() -> Self {
        Self {
            watch: StillnessWatch::with_platform(StillnessConfig::default(), &ROWS, &head_rows()),
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
        let cycle = Sample {
            t_ns,
            present_valid: sample.present_valid(),
            commanded_valid: sample.commanded_valid(),
            missing,
            present: &present,
            commanded: &commanded,
        };
        self.watch.look(&cycle, &mut self.closed);
        self.take_closed();
    }

    /// Sort what the watch just closed: antenna holds kept, head holds folded.
    fn take_closed(&mut self) {
        for window in self.closed.drain(..) {
            let antenna = group_of(window.joint) == Some(JointGroup::Antennas);
            // The head's own holds are not qualified on the head standing
            // still: the row in question *is* the head.
            let head_still = !antenna || window.platform_still;
            let hold = Hold { window, head_still };
            if antenna && self.windows.len() < ANTENNA_LINES {
                self.windows.push(hold);
                continue;
            }
            let into = if antenna {
                &mut self.beyond
            } else {
                &mut self.heads
            };
            match into.iter_mut().find(|head| {
                head.worst.window.joint == hold.window.joint
                    && head.worst.head_still == hold.head_still
            }) {
                Some(head) => {
                    head.holds += 1;
                    if hold.window.readings.excursion_rad > head.worst.window.readings.excursion_rad
                    {
                        head.worst = hold;
                    }
                }
                None => into.push(Head {
                    worst: hold,
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
    pub fn windows(&self) -> &[Hold] {
        &self.windows
    }

    /// How many antenna holds the run held, folded ones included.
    #[must_use]
    pub fn antenna_holds(&self) -> usize {
        self.windows.len() + self.beyond.iter().map(|folded| folded.holds).sum::<usize>()
    }

    /// How many of those the head stood still across — the holds a run's
    /// verdict about the antennas' own loop can be taken over.
    #[must_use]
    pub fn head_still_holds(&self) -> usize {
        self.windows.iter().filter(|hold| hold.head_still).count()
            + self
                .beyond
                .iter()
                .filter(|folded| folded.worst.head_still)
                .map(|folded| folded.holds)
                .sum::<usize>()
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
///
/// The setpoint is on the line beside the instant, because which pose a hold
/// was taken at is half of what the record carries per hold: a probe run holds
/// three of them, and a run offset alone leaves the reader to infer which.
fn line(held: &Stillness, window: &HoldWindow) -> String {
    let read = &window.readings;
    format!(
        "  {} over a {:.2} s hold from {} at {:+.4} rad: {:.1} counts ({:.4} rad) peak to peak, \
         {:.1} reversals/s, {}, mean error {:+.4} rad, over {} reading(s), opened {:.2} s after \
         the setpoint last moved at {:+.4} rad of error",
        Name(window.joint),
        read.length().as_secs_f64(),
        held.run_offset(read.start_ns),
        window.held_rad,
        read.excursion_counts(),
        read.excursion_rad,
        read.reversals_per_s,
        period(read),
        window.mean_error_rad,
        read.samples,
        window.opened_after_ns as f64 / 1e9,
        window.error_at_open_rad
    )
}

/// What the joint did over the settle allowance the judged window drops, on a
/// line of its own under the hold's.
///
/// Printed, never judged. The arrival is what this reads: a joint that reached
/// a far goal at its own profile's stop swings and rings down inside the
/// allowance, and the figures here say how wide that swing was and how fast it
/// turned round, beside the judged tail's verdict that it then stood still. A
/// hold whose allowance the recording holds nothing over prints no line.
fn settle_line(held: &Stillness, window: &HoldWindow) -> Option<String> {
    let settle = window.settle?;
    Some(format!(
        "    settling into it over {:.2} s from {}: {:.1} counts ({:.4} rad) peak to peak, {:.1} reversals/s, {}, over {} reading(s), not judged",
        settle.length().as_secs_f64(),
        held.run_offset(settle.start_ns),
        settle.excursion_counts(),
        settle.excursion_rad,
        settle.reversals_per_s,
        period(&settle),
        settle.samples
    ))
}

/// How regular the turning was, for the middle of either of those lines.
///
/// The frequency is *apparent* and says so: it is computed against the
/// stretch's own measured sample rate, and anything above half that rate
/// arrives folded down onto it. What the spread beside it says is whether the
/// turning was a regular oscillation (small against the mean) or scattered
/// encoder dither (comparable to it). A stretch with fewer than two reversals
/// has no interval to measure and says so rather than printing a figure made
/// of one turn.
fn period(read: &Wobbled) -> String {
    match (
        read.apparent_period_samples(),
        read.reversal_interval_spread_samples,
        read.apparent_frequency_hz(),
    ) {
        (Some(samples), Some(spread), Some(hz)) => {
            format!("period ≈ {samples:.1} samples ({hz:.1} Hz apparent, spread {spread:.1})")
        }
        (Some(samples), Some(spread), None) => {
            format!("period ≈ {samples:.1} samples (spread {spread:.1})")
        }
        _ => "no period".to_string(),
    }
}

/// Say what the run held still for, and — where it is judged — what it did not.
///
/// Every antenna hold is printed up to the cap and the rest fold to one line
/// per row; each head row is printed once, as the widest hold it showed. Under
/// each hold goes the settle line, the arrival the hold's own figures skip. The
/// judging is the antennas' alone, folded or not, and the two sentences it can
/// produce are the ones the bring-up assertion is made of: a hold that moved
/// further than a still joint may, and a run that never held one long enough to
/// ask.
///
/// Under [`Standard::JudgedWhereHeadStill`] an antenna hold the head moved
/// across is printed with what it read and no verdict, because what it read is
/// the platform; the run's own sentence then asks for a head-still hold rather
/// than any hold.
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
    for hold in held.windows() {
        report.note(format!(
            "{}{}",
            line(held, &hold.window),
            unjudged(standard, hold)
        ));
        if let Some(settling) = settle_line(held, &hold.window) {
            report.note(settling);
        }
        verdict(hold, standard, &cfg, report);
    }
    for folded in held.beyond() {
        report.note(format!(
            "{}, the widest of {} further hold(s) this row held{}",
            line(held, &folded.worst.window),
            folded.holds,
            unjudged(standard, &folded.worst)
        ));
        if let Some(settling) = settle_line(held, &folded.worst.window) {
            report.note(settling);
        }
        verdict(&folded.worst, standard, &cfg, report);
    }
    for head in held.heads() {
        report.note(format!(
            "{}, the widest of {} hold(s) this row held",
            line(held, &head.worst.window),
            head.holds
        ));
        if let Some(settling) = settle_line(held, &head.worst.window) {
            report.note(settling);
        }
    }
    // What the run has to have held for its verdict to mean anything: any
    // antenna hold, or — where the head's motion disqualifies one — a hold the
    // head stood still across.
    let held_one = if standard.head_still_only() {
        held.head_still_holds() > 0
    } else {
        !held.windows().is_empty()
    };
    if held_one {
        return;
    }
    let says = if standard.head_still_only() {
        format!(
            "no antenna held one setpoint for {:.1} s after a {:.1} s settle with the head still, \
             so this run says nothing about whether the antennas stand still: {} antenna hold(s) \
             read under head motion, {} sample(s) read, {} goal change(s) over them",
            cfg.min_hold.as_secs_f64(),
            cfg.settle.as_secs_f64(),
            held.antenna_holds(),
            counts.samples,
            counts.goal_changes
        )
    } else {
        format!(
            "no antenna held one setpoint for {:.1} s after a {:.1} s settle, so this run says \
             nothing about whether the antennas stand still: {} sample(s) read, {} goal change(s) \
             over them",
            cfg.min_hold.as_secs_f64(),
            cfg.settle.as_secs_f64(),
            counts.samples,
            counts.goal_changes
        )
    };
    match standard {
        // A probe run's whole purpose is the hold, so a run without one is a
        // run that measured nothing it was asked to measure.
        Standard::Judged | Standard::JudgedWhereHeadStillRequired => report.fail(says),
        // A content tour holds the antennas almost only while the head moves,
        // and having taken no reading of the loop is a reading of the content.
        Standard::JudgedWhereHeadStill | Standard::Printed => report.note(says),
    }
}

/// What a hold's own line says about not being judged, or nothing.
fn unjudged(standard: Standard, hold: &Hold) -> &'static str {
    if standard.head_still_only() && !hold.head_still {
        ", under head motion, not judged"
    } else {
        ""
    }
}

/// Whether `hold` passes the bound, said in the words its standard asks for.
fn verdict(hold: &Hold, standard: Standard, cfg: &StillnessConfig, report: &mut Report) {
    if standard.head_still_only() && !hold.head_still {
        return;
    }
    if let Err(err) = judge(&hold.window, cfg) {
        match standard {
            Standard::Judged
            | Standard::JudgedWhereHeadStill
            | Standard::JudgedWhereHeadStillRequired => report.fail(err.to_string()),
            Standard::Printed => report.note(err.to_string()),
        }
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
    use super::{ROW_COUNT, Standard, Stillness, row, say};
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

    /// How regular the turning was, printed beside the rate: a joint reversing
    /// every cycle is a two-sample period, which at the driver's grid is the
    /// fastest thing this series can show and reads as half the sample rate.
    #[test]
    fn a_regular_wobble_prints_its_apparent_period() {
        let held = wobbling(JointRef::AntennaRight, 500, 3.0 * COUNT_RAD);
        let report = said(&held, Standard::Printed);
        assert!(
            says(
                &report.measured,
                "period ≈ 2.0 samples (25.0 Hz apparent, spread 0.0)"
            ),
            "{:?}",
            report.measured
        );
    }

    /// A hold with no turning in it has no period, and says so rather than
    /// printing a figure divided by nothing.
    #[test]
    fn a_hold_that_never_turns_round_has_no_period() {
        let held = wobbling(JointRef::AntennaRight, 500, 0.0);
        let report = said(&held, Standard::Printed);
        assert!(says(&report.measured, "no period"), "{:?}", report.measured);
        assert!(
            !says(&report.measured, "Hz apparent"),
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

    /// The invariant the fold key exists for: past the cap an antenna row
    /// folds *twice*, once per side of the head-still question, so the one hold
    /// a run took with the head still cannot be folded away by the wider holds
    /// the head moved through.
    ///
    /// The shape this is written against is a long run whose head is commanded
    /// somewhere new in the middle of every hold but the last: with one folded
    /// entry per row the run's own reading would be a head-moved hold, and the
    /// section would say it measured nothing about the antennas over a run in
    /// which the last hold measured exactly that — and withhold the verdict on
    /// a hunt in it.
    #[test]
    fn a_head_still_hold_past_the_cap_folds_apart_from_the_head_moved_ones() {
        let mut held = Stillness::default();
        let leg = row(JointRef::Leg1).expect("a bus row");
        let right = row(JointRef::AntennaRight).expect("a bus row");
        let left = row(JointRef::AntennaLeft).expect("a bus row");
        let mut n = 0_i64;
        // Twenty holds of ten seconds: twelve per antenna fill the cap and the
        // remaining eight fold, so the last one is a folded hold.
        for hold in 0..20 {
            let last = hold == 19;
            for cycle in 0..500 {
                let mut commanded = [0.0; ROW_COUNT];
                commanded[right] = f64::from(hold);
                commanded[left] = f64::from(hold);
                // The head is commanded somewhere new halfway through every
                // hold but the last, which leaves the last hold — and only it
                // — one the head stood still across.
                commanded[leg] = if last {
                    f64::from(hold)
                } else {
                    f64::from(hold) + if cycle < 250 { 0.0 } else { 0.5 }
                };
                let swing = if last { 3.0 } else { 10.0 } * COUNT_RAD;
                let mut present = commanded;
                present[right] += if cycle % 2 == 0 { swing } else { -swing };
                present[left] += if cycle % 2 == 0 { swing } else { -swing };
                held.sample(&fixture::cycle(
                    T0 + n * PERIOD_NS,
                    &present,
                    Some(&commanded),
                ));
                n += 1;
            }
        }
        held.finish();
        assert_eq!(held.windows().len(), 24, "the cap, and no further");
        assert_eq!(
            held.beyond().len(),
            4,
            "two antenna rows, each side of the head-still question: {:?}",
            held.beyond()
        );
        assert_eq!(
            held.head_still_holds(),
            2,
            "the last hold of each antenna, and nothing else"
        );
        let report = said(&held, Standard::JudgedWhereHeadStill);
        // The run measured the loop, so it does not say it measured nothing.
        assert!(
            !says(&report.measured, "so this run says nothing"),
            "{:?}",
            report.measured
        );
        // The head-still fold is judged, and it is the six-count hunt of the
        // last hold rather than the twenty-count sway of the folded rest.
        assert_eq!(report.findings.len(), 2, "{:?}", report.findings);
        assert!(
            report
                .findings
                .iter()
                .any(|line| line.contains("right antenna moved 6.0 counts")),
            "{:?}",
            report.findings
        );
        // And the head-moved fold is still on the page, unjudged.
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("20.0 counts")
                    && line.contains("under head motion, not judged")),
            "{:?}",
            report.measured
        );
    }

    /// The other half of the settle line's rule: a hold whose allowance the
    /// recording holds nothing over prints **no** line.
    ///
    /// The watch reads `None` there — the case beside this one, at the watch —
    /// and what matters here is what the report then does with it. A fallback
    /// standing in for the missing reading would print an arrival of zero
    /// counts over zero seconds, which an operator reads as a joint that
    /// arrived perfectly rather than as an arrival nobody recorded.
    #[test]
    fn a_hold_whose_allowance_was_never_read_prints_no_settle_line() {
        let mut held = Stillness::default();
        for n in 0..500 {
            let cycle = fixture::cycle(
                T0 + n * PERIOD_NS,
                &[0.0; ROW_COUNT],
                Some(&[0.0; ROW_COUNT]),
            );
            // The whole four-second allowance is bus-quiet, so it holds no
            // readings; the hold that opens after it is read as ever.
            held.sample(&if n < 200 {
                fixture::blind(cycle)
            } else {
                cycle
            });
        }
        held.finish();
        let report = said(&held, Standard::Judged);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            says(&report.measured, "right antenna over a"),
            "the hold itself is still printed: {:?}",
            report.measured
        );
        assert!(
            !says(&report.measured, "settling into it"),
            "{:?}",
            report.measured
        );
    }

    /// The settle line's period arm with no frequency beside it, which only a
    /// recording whose clock stood still can reach.
    ///
    /// The turning is counted in samples and the rate is measured off the
    /// stretch's own instants, so a stretch that reversed but spans no time has
    /// an apparent period and no rate to convert it with. A judged window
    /// cannot be that stretch — it has to span the minimum hold — so this is
    /// the settle allowance's arm alone: the reading is a stuck sample clock,
    /// and the line says the period in samples and claims no hertz.
    #[test]
    fn a_settle_allowance_on_a_stuck_clock_reads_a_period_and_no_frequency() {
        let mut held = Stillness::default();
        // Six readings inside the allowance, all stamped the same instant: the
        // goal changes on the first of them and the joint turns round on every
        // one after it.
        for n in 0..6 {
            let swing = if n % 2 == 0 { 5.0 } else { -5.0 } * COUNT_RAD;
            held.sample(&fixture::cycle(
                T0,
                &[swing; ROW_COUNT],
                Some(&[0.0; ROW_COUNT]),
            ));
        }
        // Then the clock runs again, past the allowance, and the hold is judged
        // over a joint at rest.
        for n in 0..400 {
            held.sample(&fixture::cycle(
                T0 + 4 * 1_000_000_000 + n * PERIOD_NS,
                &[0.0; ROW_COUNT],
                Some(&[0.0; ROW_COUNT]),
            ));
        }
        held.finish();
        let report = said(&held, Standard::Judged);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        let settling: Vec<&String> = report
            .measured
            .iter()
            .filter(|line| line.contains("settling into it"))
            .collect();
        assert_eq!(settling.len(), ROW_COUNT, "{:?}", report.measured);
        for line in settling {
            assert!(
                line.contains("period \u{2248} 2.0 samples (spread 0.0)"),
                "{line}"
            );
            assert!(!line.contains("Hz apparent"), "{line}");
            assert!(line.contains("10.0 counts"), "{line}");
        }
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
        let window = held.windows()[0].window;
        assert_eq!(window.readings.samples, 500 - 200 - 25);
        assert!(
            (window.readings.length().as_secs_f64() - 5.98).abs() < 1e-6,
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
            .find(|hold| hold.window.joint == JointRef::AntennaRight)
            .expect("the right antenna held")
            .window;
        let left = held
            .windows()
            .iter()
            .find(|hold| hold.window.joint == JointRef::AntennaLeft)
            .expect("the left antenna held")
            .window;
        assert_eq!(
            right.readings.start_ns, left.readings.start_ns,
            "one gap, not one split"
        );
        assert_eq!(right.readings.end_ns, left.readings.end_ns);
        assert_eq!(right.readings.samples, left.readings.samples - 25);
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
        let window = held.windows()[0].window;
        // Within a sample of the run without the jitter in it: the allowance
        // runs from a first sample that was itself late.
        assert!(window.readings.samples >= 500 - 200 - 1, "{window:?}");
        assert!(window.readings.length().as_secs_f64() > 5.9, "{window:?}");
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

    /// A run in which every row is held at zero and both antennas read `wobble`
    /// either side of it, with the first head row commanded somewhere new every
    /// `every` cycles — a hold of the antennas across a moving platform.
    fn under_head_motion(cycles: i64, wobble: f64, every: Option<i64>) -> Stillness {
        let mut held = Stillness::default();
        let leg = row(JointRef::Leg1).expect("a bus row");
        let right = row(JointRef::AntennaRight).expect("a bus row");
        let left = row(JointRef::AntennaLeft).expect("a bus row");
        for n in 0..cycles {
            let swing = if n % 2 == 0 { wobble } else { -wobble };
            let mut present = [0.0; ROW_COUNT];
            present[right] = swing;
            present[left] = swing;
            let mut commanded = [0.0; ROW_COUNT];
            if let Some(every) = every {
                commanded[leg] = (n / every) as f64 * 0.1;
            }
            held.sample(&fixture::cycle(
                T0 + n * PERIOD_NS,
                &present,
                Some(&commanded),
            ));
        }
        held.finish();
        held
    }

    /// The tour's rule: an antenna holding one setpoint while the head moves
    /// reads the head's motion, so the hold is printed with what it read and no
    /// verdict — where the same hold under the unqualified standard is a
    /// finding.
    #[test]
    fn an_antenna_hold_the_head_moved_across_is_printed_and_not_judged() {
        let held = under_head_motion(500, 3.0 * COUNT_RAD, Some(100));
        assert_eq!(held.antenna_holds(), 2, "one hold per antenna");
        assert_eq!(held.head_still_holds(), 0);
        let report = said(&held, Standard::JudgedWhereHeadStill);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            says(&report.measured, "under head motion, not judged"),
            "{:?}",
            report.measured
        );
        // The figures are printed all the same: what the hold read is the
        // reading, and only the verdict is withheld.
        assert!(
            says(&report.measured, "6.0 counts"),
            "{:?}",
            report.measured
        );
        // And the run says it measured nothing about the antennas' own loop,
        // without failing over content that moves the head.
        assert!(
            says(
                &report.measured,
                "with the head still, so this run says nothing"
            ),
            "{:?}",
            report.measured
        );
        assert!(
            says(&report.measured, "2 antenna hold(s) read under head motion"),
            "{:?}",
            report.measured
        );
        // The same hold, unqualified, is the finding it always was.
        assert_eq!(said(&held, Standard::Judged).findings.len(), 2);
    }

    /// A posture change commands every row on one cycle, so the hold it opens
    /// is a hold the head stood still across — and the verdict over it is
    /// taken, not excused.
    ///
    /// The one behaviour the head-still reading is timed for: the antenna's
    /// hold begins at the same instant the head was last commanded, and a
    /// reading that counted that command as falling *inside* the hold would
    /// withhold the verdict on exactly the holds a probe run exists to
    /// produce — a report saying it measured nothing about a run that measured
    /// the thing.
    #[test]
    fn a_hold_opened_by_a_posture_change_is_still_judged_head_still() {
        let mut held = Stillness::default();
        let leg = row(JointRef::Leg1).expect("a bus row");
        let right = row(JointRef::AntennaRight).expect("a bus row");
        let left = row(JointRef::AntennaLeft).expect("a bus row");
        for n in 0..700 {
            let mut present = [0.0; ROW_COUNT];
            let mut commanded = [0.0; ROW_COUNT];
            if n >= 200 {
                commanded[right] = 0.3;
                commanded[left] = 0.3;
                commanded[leg] = 0.5;
                let swing = if n % 2 == 0 {
                    3.0 * COUNT_RAD
                } else {
                    -3.0 * COUNT_RAD
                };
                present[right] = 0.3 + swing;
                present[left] = 0.3 + swing;
            }
            held.sample(&fixture::cycle(
                T0 + n * PERIOD_NS,
                &present,
                Some(&commanded),
            ));
        }
        held.finish();
        assert_eq!(held.antenna_holds(), 2, "one hold per antenna");
        assert_eq!(held.head_still_holds(), 2, "{:?}", held.windows());
        let report = said(&held, Standard::JudgedWhereHeadStill);
        assert!(
            !says(&report.measured, "under head motion"),
            "{:?}",
            report.measured
        );
        // And the line says which pose the hold was taken at, which is what a
        // record of three poses in one run is filled from.
        assert!(
            says(&report.measured, "right antenna over a")
                && says(&report.measured, "at +0.3000 rad:"),
            "{:?}",
            report.measured
        );
        // The hunt in that hold is the finding it would be under either
        // standard, which is what says the hold was judged rather than skipped.
        assert_eq!(report.findings.len(), 2, "{:?}", report.findings);
        assert_eq!(said(&held, Standard::Judged).findings.len(), 2);
    }

    /// The qualifier withholds nothing where the head stood still: a hunting
    /// antenna over a still platform is the finding the section exists for.
    #[test]
    fn an_antenna_that_hunts_with_the_head_still_fails_the_qualified_standard() {
        let held = under_head_motion(500, 3.0 * COUNT_RAD, None);
        assert_eq!(held.head_still_holds(), 2);
        let report = said(&held, Standard::JudgedWhereHeadStill);
        assert_eq!(report.findings.len(), 2, "{:?}", report.findings);
        assert!(
            !says(&report.measured, "under head motion"),
            "{:?}",
            report.measured
        );
    }

    /// The settle line: the arrival the judged window drops is printed under
    /// it, so a joint that swung ten counts on the way in and then stood still
    /// passes with the swing on the page.
    #[test]
    fn the_settle_line_reads_the_arrival_the_hold_skips() {
        let mut held = Stillness::default();
        for n in 0..500 {
            let swing = if n >= 200 {
                0.0
            } else if n % 2 == 0 {
                5.0 * COUNT_RAD
            } else {
                -5.0 * COUNT_RAD
            };
            held.sample(&fixture::cycle(
                T0 + n * PERIOD_NS,
                &[swing; ROW_COUNT],
                Some(&[0.0; ROW_COUNT]),
            ));
        }
        held.finish();
        let report = said(&held, Standard::Judged);
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            says(&report.measured, "settling into it over 3.98 s"),
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("settling into it")
                    && line.contains("10.0 counts")
                    && line.contains("25.0 Hz apparent")
                    && line.contains("not judged")),
            "{:?}",
            report.measured
        );
        // One line per printed hold, and the hold's own line still reads the
        // rest that followed the arrival.
        assert_eq!(
            report
                .measured
                .iter()
                .filter(|line| line.contains("settling into it"))
                .count(),
            ROW_COUNT,
            "{:?}",
            report.measured
        );
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("right antenna over a") && line.contains("0.0 counts")),
            "{:?}",
            report.measured
        );
    }
}
