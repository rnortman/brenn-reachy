//! Which reading of the legs' counts is the model's crank angle.
//!
//! A converted count is either the crank angle the kinematic model means, or it
//! is a quarter turn away from it on every leg — alternate legs each way. Both
//! readings close the linkage exactly, so no computation on the angles alone
//! separates them: the wrong one describes a pose the mechanism could genuinely
//! hold, tens of millimetres from where the head actually is. That is why this
//! module classifies rather than decides, and why a clean result needs a head
//! height measured with a ruler.
//!
//! ## What the classification rests on
//!
//! Three tests, in order, and each can only ever move a candidate from
//! "recordable" to "a human looks at this":
//!
//! 1. **Window membership.** The provisioned travel windows admit one reading
//!    and not the other. Cheap, and wrong on its own — see the next paragraph.
//! 2. **The linkage closes.** The candidate's angles solve to a plausible head
//!    pose. This breaks no tie by itself; both readings solve.
//! 3. **The height agrees.** The pose the candidate implies puts the head where
//!    the operator measured it. This is the only test with information from
//!    outside the model in it, and it is the one that decides.
//!
//! The membership test prefers the *wrong* datum at one configuration this
//! project has on record — the tight resting configuration, where the direct
//! reading sits outside four of the six windows and the shifted reading is
//! inside all six. A machine resting there therefore classifies for review
//! under either truth, forever, and that is the intended steady state: the
//! resolution is a person's, written into the bench configuration with a note
//! saying who resolved it and from which record.
//!
//! Nothing here ever writes a datum anywhere. The classification is recorded;
//! arming re-verifies the configured datum against the machine on every run.

use serde::{Deserialize, Serialize};

use reachy_bus::CrankDatum;
use reachy_kin::{FkOptions, HeadGeometry, baked, outside_limit, rest_head_pose, stow_head_pose};
use reachy_motion::{ArmRecord, JointVector};

/// How far a reading may sit from the recorded tight resting configuration and
/// still be treated as being at it, radians (3° per leg).
///
/// The configuration is on record to two decimal places of a degree and the
/// linkage is near-singular there, where a hundredth of a degree of head pose
/// moves a crank by rather more, so the comparison is deliberately loose: this
/// is a "the platform is resting where it rests" test, not an identity test.
const CANDIDATE_B_TOLERANCE: f64 = 3.0 * (core::f64::consts::PI / 180.0);

/// What a rest-pose reading says about the datum.
///
/// A `human-review` row with no reason is a state this type does not have,
/// and reading one is a refusal rather than a reason invented to fill the gap.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(
    tag = "classification",
    content = "review_reason",
    rename_all = "kebab-case"
)]
pub enum DatumClass {
    /// A converted count is the crank angle as the model means it.
    Direct,
    /// Alternate legs sit a quarter turn either side of it.
    ParityShifted,
    /// The reading does not resolve the datum, for a stated reason.
    HumanReview(ReviewReason),
}

impl DatumClass {
    /// The datum this reading resolves, or `None` if it resolves none.
    #[must_use]
    pub fn resolved(self) -> Option<CrankDatum> {
        match self {
            Self::Direct => Some(CrankDatum::Direct),
            Self::ParityShifted => Some(CrankDatum::ParityShifted),
            Self::HumanReview(_) => None,
        }
    }

    /// Whether this classification is waiting on a person.
    #[must_use]
    pub fn needs_review(self) -> bool {
        matches!(self, Self::HumanReview(_))
    }
}

impl core::fmt::Display for DatumClass {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Direct => f.write_str("direct"),
            Self::ParityShifted => f.write_str("parity shifted"),
            Self::HumanReview(reason) => write!(f, "for human review: {reason}"),
        }
    }
}

/// Why a reading was left for a person.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "kebab-case")]
pub enum ReviewReason {
    /// Neither reading places all six legs inside their travel windows.
    Neither,
    /// Both readings do, so membership says nothing.
    Both,
    /// Membership picked the shifted reading at the one configuration where it
    /// is known to pick the wrong one.
    ShiftedAtCandidateB,
    /// The candidate's angles close no plausible linkage.
    FkInconsistent,
    /// No head height was measured, and membership alone cannot resolve the
    /// datum.
    HeightUnmeasured,
    /// The measured head height is not where the candidate puts the head.
    HeightMismatch,
}

impl core::fmt::Display for ReviewReason {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Neither => f.write_str("neither reading is inside every travel window"),
            Self::Both => f.write_str("both readings are inside every travel window"),
            Self::ShiftedAtCandidateB => f.write_str(
                "the shifted reading is inside every window at the tight resting configuration, \
                 where it is known to be the wrong reading",
            ),
            Self::FkInconsistent => f.write_str("the candidate's angles close no plausible pose"),
            Self::HeightUnmeasured => f.write_str("no head height was measured"),
            Self::HeightMismatch => {
                f.write_str("the measured head height is not where the candidate puts the head")
            }
        }
    }
}

/// The model angles the six legs hold if the datum is parity shifted.
///
/// The shift is the servo map's own, taken as the six legs' array rather than
/// restated, so a classification and a conversion can never disagree about which
/// way a leg moves and no leg can arrive here without a shift.
#[must_use]
pub fn shifted_reading(rest_direct: &[f64; 6]) -> [f64; 6] {
    let shifts = CrankDatum::ParityShifted.leg_shifts();
    std::array::from_fn(|leg| rest_direct[leg] + shifts[leg])
}

/// Whether every leg's angle is inside its own travel window.
///
/// The membership test the envelope uses, so an angle nobody can place is
/// outside rather than quietly inside.
fn all_inside(windows: &[(f64, f64); 6], angles: &[f64; 6]) -> bool {
    windows
        .iter()
        .zip(angles.iter())
        .all(|(&(lo, hi), &angle)| angle >= lo && angle <= hi)
}

/// Whether a reading is at the recorded tight resting configuration.
fn at_candidate_b(rest_direct: &[f64; 6]) -> bool {
    baked::REST_CRANK_ANGLES_DEG
        .iter()
        .zip(rest_direct.iter())
        .all(|(&recorded_deg, &angle)| {
            !outside_limit(
                (recorded_deg.to_radians() - angle).abs(),
                CANDIDATE_B_TOLERANCE,
            )
        })
}

/// Classify the crank datum from a resting-pose reading.
///
/// `rest_direct` is the six legs' angles as converted counts, with no shift
/// applied. `fk_probe` returns the head height a candidate's angles imply, or
/// `None` if they close no plausible linkage; `measured_height_m` is what the
/// operator measured, and `height_tol_m` how far the two may differ.
///
/// Recording a datum takes all three tests. Anything else is
/// [`DatumClass::HumanReview`] with the reason it stopped there — never a guess
/// and never a preference between two readings that both close the linkage.
pub fn classify_datum(
    windows: &[(f64, f64); 6],
    rest_direct: &[f64; 6],
    measured_height_m: Option<f64>,
    fk_probe: impl Fn(&[f64; 6]) -> Option<f64>,
    height_tol_m: f64,
) -> DatumClass {
    let shifted = shifted_reading(rest_direct);

    // Membership, which proposes a candidate and never confirms one.
    let (candidate, model_angles) = match (
        all_inside(windows, rest_direct),
        all_inside(windows, &shifted),
    ) {
        (true, true) => return DatumClass::HumanReview(ReviewReason::Both),
        (false, false) => return DatumClass::HumanReview(ReviewReason::Neither),
        (true, false) => (DatumClass::Direct, *rest_direct),
        (false, true) => {
            // The one configuration where membership is known to prefer the
            // wrong reading. Refusing here is what keeps a machine resting
            // there from recording a datum nobody checked.
            if at_candidate_b(rest_direct) {
                return DatumClass::HumanReview(ReviewReason::ShiftedAtCandidateB);
            }
            (DatumClass::ParityShifted, shifted)
        }
    };

    // The linkage closes under the candidate. Breaks no tie on its own — both
    // readings solve — but a candidate that solves nothing is not a candidate.
    let Some(fk_height) = fk_probe(&model_angles).filter(|height| height.is_finite()) else {
        return DatumClass::HumanReview(ReviewReason::FkInconsistent);
    };

    // The measurement from outside the model, which is the test that decides.
    let Some(measured) = measured_height_m else {
        return DatumClass::HumanReview(ReviewReason::HeightUnmeasured);
    };
    if outside_limit((fk_height - measured).abs(), height_tol_m) {
        return DatumClass::HumanReview(ReviewReason::HeightMismatch);
    }

    candidate
}

/// The head height a set of leg angles holds, metres above the base, or `None`
/// if they close no plausible linkage.
///
/// The probe [`classify_datum`] is meant to be handed: the solver is seeded from
/// both configurations this platform is known to come to rest at, and takes the
/// first that closes.
#[must_use]
pub fn head_height(geom: &HeadGeometry, opts: &FkOptions, legs: &[f64; 6]) -> Option<f64> {
    let joints = JointVector {
        body_yaw: 0.0,
        legs: *legs,
        antennas: [0.0; 2],
    };
    ArmRecord::solve(geom, opts, &joints, &[rest_head_pose(), stow_head_pose()])
        .ok()
        .map(|record| record.head_pose_body.translation.z)
}

#[cfg(test)]
mod tests {
    use super::*;
    use reachy_kin::{EnvelopeConfig, LegAngles, inverse_kinematics};
    use reachy_motion::JointId;

    /// The six travel windows the bench runs under.
    fn windows() -> [(f64, f64); 6] {
        EnvelopeConfig::default().crank_windows
    }

    /// The model's crank angles at the stow pose — a configuration the machine
    /// can hold, well inside every window.
    fn stow_angles() -> [f64; 6] {
        let geom = HeadGeometry::default();
        let mut angles = LegAngles([0.0; 6]);
        inverse_kinematics(&geom, &stow_head_pose(), &mut angles).expect("stow is reachable");
        angles.0
    }

    /// The probe the bench hands the classifier.
    fn probe(legs: &[f64; 6]) -> Option<f64> {
        head_height(&HeadGeometry::default(), &FkOptions::default(), legs)
    }

    /// The direct reading a machine would report if the datum were shifted and
    /// its model angles were `model`.
    fn direct_under_shift(model: &[f64; 6]) -> [f64; 6] {
        let mut out = [0.0; 6];
        for (leg, angle) in model.iter().enumerate() {
            out[leg] = angle - shifted_reading(&[0.0; 6])[leg];
        }
        out
    }

    /// The shift is the servo map's, taken per leg: alternate legs a quarter
    /// turn each way, and no leg left where it was.
    ///
    /// The every-leg-moves half is asserted rather than assumed: a leg that came
    /// back with a zero shift would make the shifted candidate a near-copy of
    /// the direct one, and the classifier would compare a reading against
    /// itself.
    #[test]
    fn the_shift_is_the_servo_maps_own() {
        let shifted = shifted_reading(&[0.0; 6]);
        for (leg, shift) in shifted.iter().enumerate() {
            let expected = CrankDatum::ParityShifted
                .shift_for(JointId::Leg(u8::try_from(leg).unwrap()))
                .expect("a leg has a shift");
            assert_eq!(*shift, expected, "leg {}", leg + 1);
            assert_ne!(*shift, 0.0, "leg {}", leg + 1);
        }
        assert!(shifted[0] > 0.0 && shifted[1] < 0.0);
    }

    /// A reading inside every window whose height is where it was measured
    /// resolves to the direct datum.
    #[test]
    fn a_reading_inside_the_windows_at_its_measured_height_is_direct() {
        let rest_direct = stow_angles();
        let height = probe(&rest_direct).expect("the stow angles close");

        assert_eq!(
            classify_datum(&windows(), &rest_direct, Some(height), probe, 0.005),
            DatumClass::Direct,
        );
    }

    /// The same machine under the other datum: what it reports directly is
    /// outside the windows, and the shifted reading is the one that closes at
    /// the measured height.
    #[test]
    fn a_shifted_machine_at_the_stow_candidate_is_parity_shifted() {
        let model = stow_angles();
        let rest_direct = direct_under_shift(&model);
        let height = probe(&model).expect("the stow angles close");

        assert!(
            !all_inside(&windows(), &rest_direct),
            "the direct reading of a shifted machine should leave the windows"
        );
        assert_eq!(
            classify_datum(&windows(), &rest_direct, Some(height), probe, 0.005),
            DatumClass::ParityShifted,
        );
    }

    /// The recorded tight resting configuration: membership prefers the shifted
    /// reading there, and that is exactly where it is known to be wrong.
    #[test]
    fn the_recorded_rest_goes_to_review_however_it_measures() {
        let mut rest_direct = [0.0; 6];
        for (leg, deg) in baked::REST_CRANK_ANGLES_DEG.iter().enumerate() {
            rest_direct[leg] = deg.to_radians();
        }

        assert!(
            !all_inside(&windows(), &rest_direct),
            "the recorded rest leaves four windows"
        );
        assert!(
            all_inside(&windows(), &shifted_reading(&rest_direct)),
            "and the shifted reading of it is inside all six — the counterexample"
        );

        // Whatever height accompanies it, and whether one accompanies it at all.
        for measured in [None, Some(0.1265), Some(0.176)] {
            assert_eq!(
                classify_datum(&windows(), &rest_direct, measured, probe, 0.005),
                DatumClass::HumanReview(ReviewReason::ShiftedAtCandidateB),
            );
        }
    }

    /// Neither reading inside the windows resolves nothing.
    #[test]
    fn neither_reading_inside_the_windows_is_review() {
        let rest_direct = [3.0; 6];
        assert_eq!(
            classify_datum(&windows(), &rest_direct, Some(0.177), probe, 0.005),
            DatumClass::HumanReview(ReviewReason::Neither),
        );
    }

    /// Both readings inside the windows resolves nothing either — membership is
    /// silent, not confirming.
    #[test]
    fn both_readings_inside_the_windows_is_review() {
        let rest_direct = [-30.0, 30.0, -30.0, 30.0, -30.0, 30.0].map(f64::to_radians);
        assert!(all_inside(&windows(), &rest_direct));
        assert!(all_inside(&windows(), &shifted_reading(&rest_direct)));
        assert_eq!(
            classify_datum(&windows(), &rest_direct, Some(0.177), probe, 0.005),
            DatumClass::HumanReview(ReviewReason::Both),
        );
    }

    /// A candidate that closes no linkage is not a candidate, however the
    /// windows read.
    #[test]
    fn a_candidate_that_closes_nothing_is_review() {
        let rest_direct = stow_angles();
        assert_eq!(
            classify_datum(&windows(), &rest_direct, Some(0.177), |_| None, 0.005),
            DatumClass::HumanReview(ReviewReason::FkInconsistent),
        );
        // A height that is not a number is the same refusal: it places nothing.
        assert_eq!(
            classify_datum(
                &windows(),
                &rest_direct,
                Some(0.177),
                |_| Some(f64::NAN),
                0.005
            ),
            DatumClass::HumanReview(ReviewReason::FkInconsistent),
        );
    }

    /// Without the ruler there is no recordable result, because the two
    /// readings differ by tens of millimetres of head height and by nothing a
    /// computation can see.
    #[test]
    fn an_unmeasured_height_never_resolves() {
        let rest_direct = stow_angles();
        assert_eq!(
            classify_datum(&windows(), &rest_direct, None, probe, 0.005),
            DatumClass::HumanReview(ReviewReason::HeightUnmeasured),
        );
    }

    /// A height that disagrees with the candidate refuses it rather than
    /// widening the tolerance.
    #[test]
    fn a_height_that_disagrees_is_review() {
        let rest_direct = stow_angles();
        let height = probe(&rest_direct).expect("the stow angles close");

        assert_eq!(
            classify_datum(&windows(), &rest_direct, Some(height + 0.02), probe, 0.005),
            DatumClass::HumanReview(ReviewReason::HeightMismatch),
        );
        // A measurement nobody can place is a disagreement, not a pass.
        assert_eq!(
            classify_datum(&windows(), &rest_direct, Some(f64::NAN), probe, 0.005),
            DatumClass::HumanReview(ReviewReason::HeightMismatch),
        );
    }

    /// The probe solves the recorded resting configuration to the height that
    /// configuration is on record at.
    #[test]
    fn the_probe_solves_the_recorded_rest() {
        let mut legs = [0.0; 6];
        for (leg, deg) in baked::REST_CRANK_ANGLES_DEG.iter().enumerate() {
            legs[leg] = deg.to_radians();
        }
        let height = probe(&legs).expect("the recorded rest closes");
        let recorded = baked::REST_TRANSLATION[2];
        // The two records of that configuration — the angles and the pose —
        // disagree by microns of height, which is the gap measured here (3.46
        // µm). The datum classification's height test has to be wide against
        // it, and it is, by five orders of magnitude.
        assert!(
            (height - recorded).abs() < 5e-6,
            "the probe puts the head at {height} m against the recorded {recorded} m"
        );
    }

    /// A classification says which datum it resolved, or that it resolved none.
    #[test]
    fn a_classification_reports_what_it_resolved() {
        assert_eq!(DatumClass::Direct.resolved(), Some(CrankDatum::Direct));
        assert_eq!(
            DatumClass::ParityShifted.resolved(),
            Some(CrankDatum::ParityShifted)
        );
        assert_eq!(DatumClass::HumanReview(ReviewReason::Both).resolved(), None);
        assert!(DatumClass::HumanReview(ReviewReason::Both).needs_review());
        assert!(!DatumClass::Direct.needs_review());
    }
}
