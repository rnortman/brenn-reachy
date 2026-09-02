//! Where the head actually went, read off an estimate stream.
//!
//! The pose arithmetic both log analyzers in this package need, in one crate
//! because the two of them must not disagree about it. `first_motion_report`
//! asks whether the wake gesture happened; `speech_run_report` asks whether the
//! head moved at all on a supervised speech run. Both answers are read off the
//! same channel with the same solver, against the same two tolerances, and a
//! second copy of any of that drifts on the first change nobody made twice --
//! at which point one report passes a run the other fails and neither says why.
//!
//! Sans-I/O and sans-log: an estimate message and a slice of solved poses come
//! in, a number or a finding comes out. Nothing here opens a file, so the
//! reading machinery each analyzer holds stays its own.
//!
//! The tolerances are a machine's and not a solver's, which is what keeps this
//! out of the scenario checkers: they are sized for servo quantisation, linkage
//! compliance and a real bus, and a deterministic run is asserted a thousand
//! times tighter elsewhere.

use brenn_reachy__driver__pose_clk_rs::PoseEstimateWire;
use nalgebra::Isometry3;
use reachy_motion::record;
use run_report::Report;

/// How far the head may be from the posture it was sent to, in metres, and still
/// count as having arrived.
///
/// Five millimetres, which is a machine's tolerance rather than a solver's: the
/// servos quantise at 0.088 degrees of crank, the linkage flexes under the
/// head's own weight, and a real leg sits wherever its load leaves it. A
/// scenario asserts a thousand times tighter than this because its plant is
/// arithmetic; asserting that here would fail every hardware run on physics.
///
/// Nobody has measured this machine. This figure and the one below were sized
/// from the mechanism on paper, so what they are is a guess of the right order;
/// a verdict that turns on one of them is a verdict about a number an agent
/// chose. The runbook's first-run checks say to confirm or reset them before
/// reading a report as a pass or a fail.
pub const ARRIVAL_OFFSET_M: f64 = 5e-3;

/// How far the head's orientation may be from the posture it was sent to,
/// radians, on the same terms, and unmeasured on the same terms.
pub const ARRIVAL_TURN_RAD: f64 = 0.05;

// Held apart from the figures a deterministic run is asserted to, so tightening
// one to a scenario's is a deliberate edit rather than a report that fails every
// hardware run on physics.
const _: () = assert!(ARRIVAL_OFFSET_M >= 1e-3);
const _: () = assert!(ARRIVAL_TURN_RAD >= 1e-2);

/// The pose an estimate describes, whatever its own flag says.
#[must_use]
pub fn solved_pose(estimate: &PoseEstimateWire) -> Option<Isometry3<f64>> {
    let estimate = estimate.validate().ok()?;
    record::read_pose(&estimate.head_pos, &estimate.head_quat).ok()
}

/// How far one component is from its own tolerance, the two summed.
///
/// A score of one is the tolerance box's corner, and nothing about it is a
/// distance. It exists so that one ranking answers both questions a report asks
/// of a sample -- which sample was closest, and which departed furthest -- with
/// translation and rotation weighted by what each is allowed.
fn score(offset: f64, turn: f64) -> f64 {
    offset / ARRIVAL_OFFSET_M + turn / ARRIVAL_TURN_RAD
}

/// How far one sample is from a wanted head pose: metres, then radians.
fn away(pose: &Isometry3<f64>, wanted: &Isometry3<f64>) -> (f64, f64) {
    (
        (pose.translation.vector - wanted.translation.vector).norm(),
        pose.rotation.angle_to(&wanted.rotation),
    )
}

/// The furthest the head got from where it started.
///
/// The two numbers are independent maxima over the whole run, not one sample's
/// pair: a run that moved in translation alone and a run that only turned are
/// both movement, and reading both components off whichever single sample
/// scored highest would hide one behind the other. `at` is that highest-scoring
/// sample, which is the departure a reader wants the instant of.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct Excursion {
    /// The largest distance any sample reached from the first one, metres.
    pub offset_m: f64,
    /// The largest angle any sample reached from the first one, radians.
    pub turn_rad: f64,
    /// The log instant of the sample that departed furthest by both together.
    pub at: i64,
}

/// The largest departure of any solved pose from the first one.
///
/// Measured from the run's own first sample rather than from a posture, because
/// the question is whether the machine moved and not whether it arrived: a head
/// commanded nowhere sits where the last run left it, and where that is has no
/// fixed answer. An empty stream, or one sample alone, is no excursion.
#[must_use]
pub fn excursion(poses: &[(i64, Isometry3<f64>)]) -> Excursion {
    let mut departure = Departure::default();
    for (at, pose) in poses {
        departure.sample(*at, pose);
    }
    departure.excursion()
}

/// The excursion so far, sample by sample.
///
/// The fold [`excursion`] is written as, kept apart from it so a reader that
/// walks a stream once and a reader that holds a slice get the same numbers out
/// of the same arithmetic.
#[derive(Clone, Copy, Debug, Default)]
struct Departure {
    /// The pose everything is measured from: the run's own first solved sample.
    first: Option<Isometry3<f64>>,
    excursion: Excursion,
    /// The best score seen, which is what `at` names the sample of.
    furthest: f64,
}

impl Departure {
    /// One more solved pose, at its log instant.
    fn sample(&mut self, at: i64, pose: &Isometry3<f64>) {
        let Some(first) = self.first else {
            self.first = Some(*pose);
            self.excursion.at = at;
            return;
        };
        let (offset, turn) = away(pose, &first);
        self.excursion.offset_m = self.excursion.offset_m.max(offset);
        self.excursion.turn_rad = self.excursion.turn_rad.max(turn);
        let scored = score(offset, turn);
        if scored > self.furthest {
            self.furthest = scored;
            self.excursion.at = at;
        }
    }

    /// The largest departure so far. An empty stream is no excursion.
    const fn excursion(&self) -> Excursion {
        self.excursion
    }
}

/// How near the head got to a wanted pose, and whether that counts as arriving.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Approach {
    /// The log instant of the best approach.
    pub at: i64,
    /// How far that sample was from the wanted pose, metres.
    pub offset_m: f64,
    /// How far it was turned from it, radians.
    pub turn_rad: f64,
    /// The first sample that was inside both tolerances at once, where one was.
    pub arrived: Option<(i64, f64, f64)>,
}

/// How near the head came to `wanted` from `from` onwards.
///
/// Arrival is both tolerances at once, on one sample: a machine 4 mm away but
/// badly turned has not arrived, and a machine that arrived is a sample that was
/// close in translation *and* aligned. The sample answered is the best by one
/// score over both components, so a printed number and a verdict taken from it
/// come from the same definition -- ranking on translation alone would name a
/// well-placed, badly turned sample as the closest approach and then fail a run
/// on it while a further, aligned sample went unmentioned.
///
/// Says nothing about the run: a caller that fails on a missed arrival and a
/// caller that only prints how near the head got read the same arithmetic.
#[must_use]
pub fn approach(
    poses: &[(i64, Isometry3<f64>)],
    from: usize,
    wanted: &reachy_motion::joints::JointTargets,
) -> Option<Approach> {
    let mut nearest = Nearest::towards(wanted.head_pose_body);
    for (at, pose) in &poses[from.min(poses.len())..] {
        nearest.sample(*at, pose);
    }
    nearest.approach()
}

/// The nearest approach to one wanted pose so far, sample by sample.
///
/// The fold [`approach`] is written as, kept apart from it for the reason
/// [`Departure`] is.
#[derive(Clone, Copy, Debug)]
struct Nearest {
    /// Where the head was asked to be.
    wanted: Isometry3<f64>,
    /// The best sample so far: its instant, its two distances, and its score.
    best: Option<(i64, f64, f64, f64)>,
    /// The first sample inside both tolerances at once, where one was.
    arrived: Option<(i64, f64, f64)>,
}

impl Nearest {
    /// A fold towards `wanted`, with nothing sampled yet.
    const fn towards(wanted: Isometry3<f64>) -> Self {
        Self {
            wanted,
            best: None,
            arrived: None,
        }
    }

    /// One more solved pose, at its log instant.
    fn sample(&mut self, at: i64, pose: &Isometry3<f64>) {
        let (offset, turn) = away(pose, &self.wanted);
        let scored = score(offset, turn);
        if self.best.is_none_or(|(_, _, _, was)| scored < was) {
            self.best = Some((at, offset, turn, scored));
        }
        if self.arrived.is_none() && offset <= ARRIVAL_OFFSET_M && turn <= ARRIVAL_TURN_RAD {
            self.arrived = Some((at, offset, turn));
        }
    }

    /// How near the head came, over what was sampled. Nothing sampled is no
    /// approach.
    fn approach(&self) -> Option<Approach> {
        let (at, offset_m, turn_rad, _) = self.best?;
        Some(Approach {
            at,
            offset_m,
            turn_rad,
            arrived: self.arrived,
        })
    }
}

/// What the head did over a run, folded off the estimate stream.
///
/// Both questions a report asks of the pose channel — how far the head departed
/// from where it started, and how near it came to where it was sent — answered
/// without keeping a sample. The channel carries one message per control cycle
/// and a supervised session ends when an operator ends it, so a reader that
/// collected the stream would be unable to run over the longest sessions, which
/// are the ones most worth judging.
#[derive(Clone, Copy, Debug)]
pub struct Motion {
    departure: Departure,
    nearest: Nearest,
    solved: usize,
    seen: usize,
}

impl Motion {
    /// A fold measuring approach against `wanted`.
    #[must_use]
    pub fn towards(wanted: &reachy_motion::joints::JointTargets) -> Self {
        Self {
            departure: Departure::default(),
            nearest: Nearest::towards(wanted.head_pose_body),
            solved: 0,
            seen: 0,
        }
    }

    /// One estimate off the channel, at the instant the log recorded it.
    ///
    /// An estimate this build cannot solve a pose out of is counted and
    /// otherwise ignored: it says nothing about where the head was, and the
    /// count of them against the count seen is what tells a reader that.
    pub fn estimate(&mut self, at_ns: i64, estimate: &PoseEstimateWire) {
        self.seen += 1;
        let Some(pose) = solved_pose(estimate) else {
            return;
        };
        self.solved += 1;
        self.departure.sample(at_ns, &pose);
        self.nearest.sample(at_ns, &pose);
    }

    /// The largest departure from the run's first solved pose.
    #[must_use]
    pub const fn excursion(&self) -> Excursion {
        self.departure.excursion()
    }

    /// How near the head came to the wanted pose, over the whole run.
    #[must_use]
    pub fn approach(&self) -> Option<Approach> {
        self.nearest.approach()
    }

    /// How many estimates a pose was solved out of.
    #[must_use]
    pub const fn solved(&self) -> usize {
        self.solved
    }

    /// How many estimates the channel carried.
    #[must_use]
    pub const fn seen(&self) -> usize {
        self.seen
    }
}

/// Where the head came to `wanted` from `from` onwards, reported and judged
/// against the hardware tolerances. Answers the instant it arrived, or the
/// instant of its best approach where it never did.
///
/// The judging half of [`approach`]: a report that asked for a posture is a
/// report a missed one is a finding of.
pub fn closest(
    poses: &[(i64, Isometry3<f64>)],
    from: usize,
    what: &str,
    wanted: &reachy_motion::joints::JointTargets,
    report: &mut Report,
) -> Option<i64> {
    let Approach {
        at: best_at,
        offset_m: offset,
        turn_rad: turn,
        arrived,
    } = approach(poses, from, wanted)?;
    report.note(format!(
        "closest to {what}: {offset:.4} m and {turn:.4} rad away, at {best_at}"
    ));
    let Some((at, offset, turn)) = arrived else {
        report.fail(format!(
            "the head never came within {ARRIVAL_OFFSET_M} m and {ARRIVAL_TURN_RAD} rad of \
             {what} on one sample: its best approach was {offset:.4} m and {turn:.4} rad, at \
             {best_at}"
        ));
        return Some(best_at);
    };
    report.note(format!(
        "reached {what} at {at}: {offset:.4} m and {turn:.4} rad away"
    ));
    Some(at)
}

#[cfg(test)]
mod tests {
    use nalgebra::{Isometry3, UnitQuaternion, Vector3};
    use reachy_motion::joints::JointTargets;
    use reachy_motion::postures::neutral_targets;
    use run_report::Report;

    use super::{
        ARRIVAL_OFFSET_M, ARRIVAL_TURN_RAD, Excursion, Motion, PoseEstimateWire, approach, closest,
        excursion,
    };

    /// A pose displaced from `from` by `metres` along x and `radians` about z.
    fn moved(from: &Isometry3<f64>, metres: f64, radians: f64) -> Isometry3<f64> {
        Isometry3::from_parts(
            (from.translation.vector + Vector3::new(metres, 0.0, 0.0)).into(),
            UnitQuaternion::from_axis_angle(&Vector3::z_axis(), radians) * from.rotation,
        )
    }

    /// A machine that never moved has no excursion, whatever pose it sat at.
    #[test]
    fn a_still_run_has_no_excursion() {
        let held = neutral_targets().head_pose_body;
        let poses = vec![(1, held), (2, held), (3, held)];
        let excursion = excursion(&poses);
        assert_eq!(excursion.offset_m, 0.0);
        assert_eq!(excursion.turn_rad, 0.0);
        assert_eq!(excursion.at, 1, "the first sample is its own reference");
    }

    /// A stream with nothing in it is not a movement either, and is not a
    /// panic: a log that lost its estimates off the front can be empty here.
    #[test]
    fn an_empty_stream_has_no_excursion() {
        let excursion = excursion(&[]);
        assert_eq!(excursion.offset_m, 0.0);
        assert_eq!(excursion.turn_rad, 0.0);
    }

    /// One displaced sample is the whole excursion, and it is measured from the
    /// first sample rather than from any posture.
    #[test]
    fn one_displaced_sample_is_the_excursion() {
        let held = neutral_targets().head_pose_body;
        let poses = vec![(1, held), (2, moved(&held, 0.04, 0.0)), (3, held)];
        let excursion = excursion(&poses);
        assert!(
            (excursion.offset_m - 0.04).abs() < 1e-9,
            "{}",
            excursion.offset_m
        );
        assert_eq!(excursion.turn_rad, 0.0);
        assert_eq!(excursion.at, 2, "the instant it was furthest away");
    }

    /// The two components are maxima in their own right: a run that translated
    /// on one sample and turned on another moved in both, and neither number is
    /// hidden by the other sample's score.
    #[test]
    fn the_two_components_are_independent_maxima() {
        let held = neutral_targets().head_pose_body;
        let poses = vec![
            (1, held),
            (2, moved(&held, 0.04, 0.0)),
            (3, moved(&held, 0.0, 0.3)),
        ];
        let excursion = excursion(&poses);
        assert!((excursion.offset_m - 0.04).abs() < 1e-9);
        assert!((excursion.turn_rad - 0.3).abs() < 1e-9);
    }

    /// Arrival is both tolerances on one sample, and the instant answered is
    /// the sample that arrived.
    #[test]
    fn a_sample_within_both_tolerances_has_arrived() {
        let wanted = neutral_targets();
        let near = moved(
            &wanted.head_pose_body,
            ARRIVAL_OFFSET_M / 2.0,
            ARRIVAL_TURN_RAD / 2.0,
        );
        let poses = vec![(1, moved(&wanted.head_pose_body, 0.2, 0.0)), (2, near)];
        let mut report = Report::default();
        assert_eq!(closest(&poses, 0, "upright", &wanted, &mut report), Some(2));
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(
            report
                .measured
                .iter()
                .any(|line| line.contains("reached upright at 2")),
            "{:?}",
            report.measured
        );
    }

    /// Close in translation and badly turned is not an arrival, and the run
    /// says by how much it missed.
    #[test]
    fn a_well_placed_badly_turned_sample_has_not_arrived() {
        let wanted = neutral_targets();
        let askew = moved(&wanted.head_pose_body, 0.0, ARRIVAL_TURN_RAD * 4.0);
        let mut report = Report::default();
        closest(&[(7, askew)], 0, "upright", &wanted, &mut report);
        assert!(
            report.findings[0].contains("never came within"),
            "{:?}",
            report.findings
        );
    }

    /// The arithmetic holds no opinion: a head that never arrived is a missed
    /// arrival to the caller that fails on one and two numbers to the caller
    /// that only prints how near it got.
    #[test]
    fn an_approach_that_missed_is_numbers_rather_than_a_verdict() {
        let wanted = neutral_targets();
        let askew = moved(&wanted.head_pose_body, 0.0, ARRIVAL_TURN_RAD * 4.0);
        let near = approach(&[(7, askew)], 0, &wanted).expect("one sample is an approach");
        assert_eq!(near.at, 7);
        assert!(near.offset_m < 1e-9, "{}", near.offset_m);
        assert!(
            (near.turn_rad - ARRIVAL_TURN_RAD * 4.0).abs() < 1e-9,
            "{}",
            near.turn_rad
        );
        assert_eq!(near.arrived, None, "it never came within both tolerances");
    }

    /// One solved estimate carrying `pose`.
    fn solved(pose: &Isometry3<f64>) -> PoseEstimateWire {
        let mut estimate = PoseEstimateWire::new();
        let valid = estimate.clear_valid();
        reachy_motion::record::write_pose(&mut valid.head_pos, &mut valid.head_quat, pose);
        valid.valid = true.into();
        estimate
    }

    /// The fold a stream is read through and the slice functions answer the
    /// same numbers: the two readers exist because one of them must not hold
    /// the samples, not because they are allowed to disagree.
    #[test]
    fn the_fold_answers_what_the_slice_functions_do() {
        let wanted = neutral_targets();
        let held = wanted.head_pose_body;
        let poses = vec![
            (1, held),
            (2, moved(&held, 0.02, 0.0)),
            (3, moved(&held, 0.005, ARRIVAL_TURN_RAD * 3.0)),
        ];
        let mut motion = Motion::towards(&wanted);
        for (at, pose) in &poses {
            motion.estimate(*at, &solved(pose));
        }
        assert_eq!(motion.excursion(), excursion(&poses));
        assert_eq!(motion.approach(), approach(&poses, 0, &wanted));
        assert_eq!(motion.solved(), 3);
        assert_eq!(motion.seen(), 3);
    }

    /// An estimate this build cannot solve a pose out of is counted and says
    /// nothing about where the head was: the two counts are what tells a
    /// reader a channel carried samples nobody could read.
    #[test]
    fn an_unsolved_estimate_is_counted_and_not_measured() {
        let wanted = neutral_targets();
        let mut motion = Motion::towards(&wanted);
        motion.estimate(4, &PoseEstimateWire::new());
        assert_eq!(motion.seen(), 1);
        assert_eq!(motion.solved(), 0);
        assert_eq!(motion.excursion(), Excursion::default());
        assert_eq!(motion.approach(), None);
    }

    /// `from` is where the reading starts, and a sample before it is not this
    /// gesture's evidence: an arrival recorded while the previous step was
    /// still holding the head must not be read as this step's.
    #[test]
    fn samples_before_from_are_not_read() {
        let wanted = neutral_targets();
        let arrived = wanted.head_pose_body;
        let askew = moved(&wanted.head_pose_body, 0.0, ARRIVAL_TURN_RAD * 4.0);

        let whole = approach(&[(1, arrived), (2, askew)], 0, &wanted)
            .expect("both samples are an approach");
        assert_eq!(
            whole.arrived.map(|(at, _, _)| at),
            Some(1),
            "the first sample is inside both tolerances"
        );

        let after = approach(&[(1, arrived), (2, askew)], 1, &wanted)
            .expect("the sample from `from` onwards is an approach");
        assert_eq!(after.at, 2, "only the sample at or after `from` was read");
        assert_eq!(
            after.arrived, None,
            "the arrival ahead of `from` belongs to whatever ran before this one"
        );
    }

    /// A `from` past the end is an empty reading and not a panic: which sample
    /// a gesture began at comes off a log, and a log is free to end before it.
    #[test]
    fn a_from_past_the_end_is_no_approach() {
        let wanted = neutral_targets();
        let poses = vec![(1, wanted.head_pose_body)];
        assert_eq!(approach(&poses, 9, &wanted), None);
        assert_eq!(approach(&[], 9, &wanted), None);
    }

    /// Nothing to judge is nothing said: a caller with an empty slice gets no
    /// finding invented for it.
    #[test]
    fn no_samples_is_no_verdict() {
        let mut report = Report::default();
        assert_eq!(
            closest(&[], 0, "upright", &JointTargets::default(), &mut report),
            None
        );
        assert!(report.findings.is_empty(), "{:?}", report.findings);
        assert!(report.measured.is_empty(), "{:?}", report.measured);
    }
}
