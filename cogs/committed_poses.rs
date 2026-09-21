//! The committed pose library, as the tests and checkers of this package read
//! it.
//!
//! `pose_library.textproto` is the asset every box binds by path, parsed on the
//! C++ side, so nothing in Rust sees the value a running box was configured
//! with. What is available is the bytes: this crate embeds the committed file
//! and reads it back through the same reader and the same screen a bound
//! library goes through, so a checker judging where the machine folded, and a
//! cog case handing a cog its configuration, both work off the file a unit
//! runs on rather than off a number restated beside it.
//!
//! The emitter's name sidecar comes with it, for the same reason and read
//! through the intent edge's own reader: a pose id is positional, so a reader
//! that wants the pose called `neutral` asks the sidecar rather than counting
//! documents.
//!
//! One crate rather than a copy per reader: two embeddings of one asset are two
//! answers able to disagree about which emit the tree holds.

use reachy_kin::{EnvelopeReport, check_envelope};
use reachy_motion::joints::JointTargets;
use reachy_motion::tick::default_motion_config;
use reachy_motion::traj::{MoveDurations, Trajectory, WarpKind};
use reachy_poses::config::{Library, parse_library, screen};
use reachy_poses::library::PoseLibrary;
use std::time::Duration;

/// The committed pose library, as the protobuf text a configuration dial takes.
const POSE_LIBRARY: &str = include_str!("pose_library.textproto");

/// The committed name-to-number sidecar the same emitter writes.
const LIBRARY_NAMES: &str = include_str!("library.names.json");

/// The committed library as the message a cog's dial hands a cog.
///
/// What a cog case sets on its wrapper's configuration, so the cog under test
/// reads the library the deployment ships.
///
/// # Panics
///
/// If the committed asset is not a library this build can read, which is a
/// broken emit rather than a case.
#[must_use]
pub fn message() -> &'static Library {
    static MESSAGE: std::sync::OnceLock<Library> = std::sync::OnceLock::new();
    MESSAGE.get_or_init(|| {
        parse_library(POSE_LIBRARY).expect("the committed pose library is the emitter's")
    })
}

/// The committed library as a message of the caller's own.
///
/// For a case that hands a cog a library differing from the committed one in
/// one field — a stow paced past a budget, most of all. Read afresh rather than
/// cloned from [`message`], so what a case alters is its own copy and the
/// shared one stays the deployment's.
///
/// # Panics
///
/// As [`message`].
#[must_use]
pub fn read() -> Library {
    parse_library(POSE_LIBRARY).expect("the committed pose library is the emitter's")
}

/// The same library, screened.
///
/// Where a reader outside a cog asks what a pose puts the machine at — the
/// stow most of all, which is what a release is judged against.
///
/// # Panics
///
/// If the committed asset does not screen, which is a broken emit: the emitter
/// screens what it writes.
#[must_use]
pub fn library() -> &'static PoseLibrary<'static> {
    static SCREENED: std::sync::OnceLock<PoseLibrary<'static>> = std::sync::OnceLock::new();
    SCREENED.get_or_init(|| {
        let bound = message()
            .validate()
            .expect("the committed pose library is a message this build reads");
        screen(bound).expect("the committed pose library screens")
    })
}

/// Where the committed library's stow puts every joint.
///
/// Solved once for the process, like the record a running session builds from
/// the same bytes: a checker that asked per judgement would solve a linkage per
/// row of a log.
///
/// # Panics
///
/// As [`library`], or if the committed stow does not solve.
#[must_use]
pub fn stow_joints() -> &'static reachy_motion::joints::JointVector {
    static FOLDED: std::sync::OnceLock<reachy_motion::joints::JointVector> =
        std::sync::OnceLock::new();
    FOLDED.get_or_init(|| {
        library()
            .stow_joints()
            .expect("the committed stow is a pose the linkage reaches")
    })
}

/// Both halves of the committed sidecar, as the intent edge reads them.
///
/// The edge's own reader rather than a walk of the JSON here: the sidecar is
/// one artifact with one shape, and a second reader of it is a second thing to
/// update when the emitter grows a field. Read once and handed out by
/// reference: a second read of one emit is a second opinion about which emit
/// this process holds, and every name lookup beside it would otherwise be a
/// fresh parse of the whole document.
///
/// # Panics
///
/// If the committed sidecar is not the emitter's JSON.
#[must_use]
pub fn tables() -> &'static (reachy_edge::MotionTable, reachy_edge::PoseTable) {
    static TABLES: std::sync::OnceLock<(reachy_edge::MotionTable, reachy_edge::PoseTable)> =
        std::sync::OnceLock::new();
    TABLES.get_or_init(|| {
        reachy_edge::parse(LIBRARY_NAMES).expect("the committed sidecar is the emitter's")
    })
}

/// What the sidecar says about the pose called `name`: its id and its pace.
///
/// # Panics
///
/// As [`tables`], or if the committed library carries no pose of that name.
#[must_use]
pub fn entry(name: &str) -> reachy_edge::PoseEntry {
    tables()
        .1
        .resolve(name)
        .unwrap_or_else(|| panic!("the committed pose library carries no pose named {name}"))
}

/// Where the pose called `name` puts the machine, as the committed library
/// states it.
///
/// # Panics
///
/// As [`entry`] and [`library`], or if the sidecar numbers a pose the library
/// does not hold — a skew between the two halves of one emit.
#[must_use]
pub fn targets(name: &str) -> reachy_motion::joints::JointTargets {
    let id = entry(name).pose_id;
    library()
        .targets(id)
        .unwrap_or_else(|| panic!("the committed library holds no pose {id}, which {name} names"))
        .0
}

/// A candidate pose under authoring, with the pace the transition checker must
/// use when arriving at it.
#[derive(Clone, Debug, PartialEq)]
pub struct Candidate {
    /// The name used in transition evidence.
    pub name: String,
    /// The complete command set the candidate would publish.
    pub targets: JointTargets,
    /// The positive duration of a move to the candidate.
    pub pace: Duration,
}

/// The first sample at which one directed pose transition leaves the envelope.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TransitionFailure {
    /// The pose the transition leaves.
    pub from: String,
    /// The pose the transition reaches.
    pub to: String,
    /// The elapsed sample that failed.
    pub elapsed: Duration,
    /// The trajectory or envelope refusal at that sample.
    pub reason: String,
}

impl core::fmt::Display for TransitionFailure {
    fn fmt(&self, out: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            out,
            "{} -> {} at {} ms: {}",
            self.from,
            self.to,
            self.elapsed.as_millis(),
            self.reason
        )
    }
}

impl std::error::Error for TransitionFailure {}

/// Check every directed minimum-jerk transition in the committed library, or
/// the committed library and one candidate in both directions.
pub fn transition_verdict(
    poses: &PoseLibrary<'_>,
    table: &reachy_edge::PoseTable,
    candidate: Option<&Candidate>,
) -> Result<(), TransitionFailure> {
    let mut committed: Vec<(String, u16, JointTargets, Duration)> = table
        .entries()
        .map(|(name, entry)| {
            let id = entry.pose_id;
            let (targets, pace) = poses.targets(id).ok_or_else(|| TransitionFailure {
                from: name.to_owned(),
                to: name.to_owned(),
                elapsed: Duration::ZERO,
                reason: format!("sidecar pose id {id} is not in the screened library"),
            })?;
            if pace.as_millis() != u128::from(entry.duration_ms) {
                return Err(TransitionFailure {
                    from: name.to_owned(),
                    to: name.to_owned(),
                    elapsed: Duration::ZERO,
                    reason: format!(
                        "sidecar pace is {} ms but screened library pace is {} ms",
                        entry.duration_ms,
                        pace.as_millis()
                    ),
                });
            }
            Ok((name.to_owned(), id, targets, pace))
        })
        .collect::<Result<_, _>>()?;
    committed.sort_by_key(|(_, id, _, _)| *id);

    let mut pairs = Vec::new();
    for from in &committed {
        for to in &committed {
            if from.1 != to.1 {
                pairs.push((from.0.clone(), from.2, to.0.clone(), to.2, to.3));
            }
        }
    }
    if let Some(candidate) = candidate {
        for committed in &committed {
            if committed.0 == candidate.name {
                continue;
            }
            pairs.push((
                committed.0.clone(),
                committed.2,
                candidate.name.clone(),
                candidate.targets,
                candidate.pace,
            ));
            pairs.push((
                candidate.name.clone(),
                candidate.targets,
                committed.0.clone(),
                committed.2,
                committed.3,
            ));
        }
    }

    let config = default_motion_config();
    for (from_name, from, to_name, to, pace) in pairs {
        let trajectory =
            Trajectory::new(&from, &to, MoveDurations::uniform(pace), WarpKind::MinJerk).map_err(
                |error| TransitionFailure {
                    from: from_name.clone(),
                    to: to_name.clone(),
                    elapsed: Duration::ZERO,
                    reason: error.to_string(),
                },
            )?;
        let end = pace;
        let mut elapsed = Duration::ZERO;
        loop {
            let mut sample = JointTargets::default();
            trajectory.sample(elapsed, &mut sample);
            let mut report = EnvelopeReport::default();
            if let Err(error) = check_envelope(
                &config.geom,
                &config.env,
                &sample.head_pose_body,
                sample.body_yaw,
                None,
                &mut report,
            ) {
                return Err(TransitionFailure {
                    from: from_name,
                    to: to_name,
                    elapsed,
                    reason: error.to_string(),
                });
            }
            if elapsed == end {
                break;
            }
            let next = elapsed.saturating_add(Duration::from_millis(20));
            elapsed = if next < end { next } else { end };
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{entry, library, message, tables, targets, transition_verdict};
    use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion, Vector3};
    use reachy_kin::{EnvelopeReport, check_envelope};
    use reachy_motion::joints::JointTargets;
    use reachy_motion::tick::default_motion_config;
    use std::time::Duration;

    fn recorded_relative(translation: [f64; 3], quaternion: [f64; 4]) -> Isometry3<f64> {
        Isometry3::from_parts(
            Translation3::from(Vector3::from(translation)),
            UnitQuaternion::from_quaternion(Quaternion::new(
                quaternion[0],
                quaternion[1],
                quaternion[2],
                quaternion[3],
            )),
        )
    }

    fn hello_candidate(factor: f64) -> super::Candidate {
        let raw = recorded_relative(
            [
                0.000682726154096681,
                0.011895116632182626,
                0.011244240721731691,
            ],
            [
                0.9715892712442394,
                0.23320556601419767,
                0.010037961906443866,
                0.039098482117108535,
            ],
        );
        let relative = Isometry3::from_parts(
            Translation3::from(raw.translation.vector * factor),
            UnitQuaternion::identity().slerp(&raw.rotation, factor),
        );
        super::Candidate {
            name: format!("hello-{factor:.2}"),
            targets: JointTargets {
                head_pose_body: reachy_kin::neutral_head_pose() * relative,
                body_yaw: 0.0,
                antennas: [-0.8303838173726208, -0.23610866451374868],
            },
            pace: Duration::from_millis(1000),
        }
    }

    fn static_report(candidate: &super::Candidate) -> (Result<(), String>, EnvelopeReport) {
        let config = default_motion_config();
        let mut report = EnvelopeReport::default();
        let verdict = check_envelope(
            &config.geom,
            &config.env,
            &candidate.targets.head_pose_body,
            candidate.targets.body_yaw,
            None,
            &mut report,
        )
        .map_err(|error| error.to_string());
        (verdict, report)
    }

    fn assert_rotation_close(actual: &UnitQuaternion<f64>, expected: &UnitQuaternion<f64>) {
        for (actual, expected) in actual.coords.iter().zip(expected.coords.iter()) {
            assert!((actual - expected).abs() <= 1e-12, "{actual} != {expected}");
        }
    }

    /// The committed asset reads, screens, and holds the stow every consumer
    /// resolves through it.
    #[test]
    fn the_committed_library_reads_and_screens() {
        let held = message()
            .validate()
            .expect("the committed pose library is a message this build reads");
        assert!(!held.poses.is_empty());
        let (stow_id, _, pace) = library().stow();
        assert_eq!(usize::from(stow_id), usize::from(held.stow));
        assert!(!pace.is_zero(), "a pace the screen admits is positive");
    }

    /// The two halves of one emit agree: the sidecar's stow is the library's,
    /// and a name resolves to the configuration the library holds under it.
    #[test]
    fn the_sidecar_numbers_the_library_this_crate_holds() {
        let stow = entry(reachy_poses::STOW_POSE);
        assert_eq!(stow.pose_id, library().stow().0);
        assert_eq!(targets(reachy_poses::STOW_POSE), library().stow().1);
        let neutral = targets(reachy_poses::NEUTRAL_POSE);
        assert_ne!(neutral, library().stow().1, "the fold is not the raise");
    }

    /// Every ordered pair in the emitted pose library stays inside the command
    /// envelope at the 20 ms samples the mover can publish.
    #[test]
    fn every_directed_pose_transition_passes() {
        transition_verdict(library(), &tables().1, None)
            .unwrap_or_else(|failure| panic!("directed pose matrix failed: {failure}"));
        assert_eq!(tables().1.len(), 5);
        for name in ["neutral", "peek", "peek_tilt", "hello", "stow"] {
            assert!(tables().1.resolve(name).is_some(), "{name}");
        }
    }

    /// The two authored poses retain the recording derivations and deliberate
    /// content choices in the emitted target table.
    #[test]
    fn derived_pose_targets_are_pinned_to_their_recordings() {
        let neutral = targets("neutral").head_pose_body;
        let relative =
            |name: &str| reachy_kin::neutral_head_pose().inverse() * targets(name).head_pose_body;
        let peek = relative("peek_tilt");
        let raw_peek = recorded_relative(
            [
                0.01049154187016598,
                -0.02274613777854499,
                -0.037654360599018194,
            ],
            [
                0.9633281022256456,
                -0.26442242963013424,
                -0.012830615551910443,
                0.04376210090247136,
            ],
        );
        let expected_peek = Isometry3::from_parts(
            Translation3::from(raw_peek.translation.vector + Vector3::new(0.0, 0.0, 0.002)),
            raw_peek.rotation,
        );
        assert_eq!(
            peek.translation.vector, expected_peek.translation.vector,
            "peek_tilt is raw S007 with a 2 mm Z lift"
        );
        assert_rotation_close(&peek.rotation, &expected_peek.rotation);
        assert_eq!(targets("peek_tilt").body_yaw, 0.0);
        assert_eq!(
            targets("peek_tilt").antennas,
            [-0.8303838173726208, -0.5722637603039527]
        );
        let hello = relative("hello");
        let expected_hello = hello_candidate(0.98).targets.head_pose_body;
        let expected_hello_relative = reachy_kin::neutral_head_pose().inverse() * expected_hello;
        assert_eq!(
            hello.translation.vector,
            expected_hello_relative.translation.vector
        );
        assert_rotation_close(&hello.rotation, &expected_hello_relative.rotation);
        assert_eq!(targets("hello").body_yaw, 0.0);
        assert_eq!(
            targets("hello").antennas,
            [-0.8303838173726208, -0.23610866451374868]
        );
        assert_eq!(targets("neutral").head_pose_body, neutral);
        assert_eq!(entry("peek_tilt").duration_ms, 1000);
        assert_eq!(entry("hello").duration_ms, 1000);
    }

    /// The recorded hello only becomes a commandable pose at the largest
    /// passing hundredth on the scale-to-neutral derivation.
    #[test]
    fn hello_scale_sweep_pins_the_largest_passing_factor() {
        transition_verdict(library(), &tables().1, None)
            .unwrap_or_else(|failure| panic!("committed matrix failed: {failure}"));

        for factor in [1.00, 0.99] {
            let candidate = hello_candidate(factor);
            let (static_failure, report) = static_report(&candidate);
            let static_failure = static_failure.expect_err("candidate must fail");
            assert!(
                report.violations.window[1],
                "factor {factor:.2}: {report:?}"
            );
            assert_eq!(
                report.violations.margin,
                report.min_margin < default_motion_config().env.min_toggle_margin,
                "factor {factor:.2}: margin reason changed"
            );
            assert!(!report.violations.unreachable.iter().any(|bit| *bit));
            assert!(!report.violations.body_yaw);
            assert!(!report.violations.relative_yaw);
            assert!(!report.violations.cone);
            let transition_failure = transition_verdict(library(), &tables().1, Some(&candidate))
                .expect_err("candidate transition must fail");
            assert_eq!(transition_failure.to, candidate.name);
            assert!(!transition_failure.from.is_empty());
            assert!(
                !transition_failure.elapsed.is_zero(),
                "factor {factor:.2} failure omitted its sample: {transition_failure}"
            );
            assert!(static_failure.contains("leg 2"), "{static_failure}");
            assert!(
                transition_failure.reason.contains("leg 2"),
                "{transition_failure}"
            );
        }

        let passing = hello_candidate(0.98);
        let (verdict, report) = static_report(&passing);
        verdict.unwrap_or_else(|error| panic!("0.98 static verdict: {error}"));
        assert!(!report.violations.window[1]);
        assert!(!report.violations.margin);
        assert!(!report.violations.body_yaw);
        assert!(!report.violations.relative_yaw);
        assert!(!report.violations.cone);
        assert!((report.min_margin - 0.00078).abs() < 0.00001, "{report:?}");
        transition_verdict(library(), &tables().1, Some(&passing))
            .unwrap_or_else(|failure| panic!("0.98 transition verdict: {failure}"));
    }

    /// The geometric rules the two antenna poses are authored under, judged
    /// against the committed documents.
    ///
    /// Nothing between an authored pose and the machine expresses these: the
    /// document loader checks the envelope, which says nothing about which way
    /// an antenna leans, and the poses are operator-authorable. Each rule is a
    /// way an antenna pose can be wrong that no other check catches.
    #[test]
    fn the_antenna_poses_lean_the_way_the_motion_stack_reads_them() {
        use core::f64::consts::FRAC_PI_2;
        use reachy_motion::phase::ANTENNA_CONTACT_BAND_RAD;

        let rest = targets(reachy_poses::NEUTRAL_POSE).antennas;
        let fold = targets(reachy_poses::STOW_POSE).antennas;
        for side in 0..2 {
            // A lean the wrong way is a lean into the other antenna's arc.
            assert!(
                rest[side] * [-FRAC_PI_2, FRAC_PI_2][side] > 0.0,
                "the rest lean of antenna {side} is {}, which is not toward its own outboard \
                 direction {}",
                rest[side],
                [-FRAC_PI_2, FRAC_PI_2][side],
            );
            // Resting outside the band would put the pair where no phase
            // judgement is made, so a pair that rests there is never judged.
            assert!(
                rest[side].abs() < ANTENNA_CONTACT_BAND_RAD,
                "the rest lean of antenna {side} is {} rad, outside the contact band of {} rad \
                 the phase check judges a pair in",
                rest[side],
                ANTENNA_CONTACT_BAND_RAD,
            );
            // Past straight down, on its own side: that is what loads the
            // gearbox play to one side and what makes the fold a fold.
            assert!(
                fold[side].abs() > core::f64::consts::PI,
                "antenna {side} folds to {} rad, which is short of straight down",
                fold[side],
            );
            assert!(
                fold[side] * [-FRAC_PI_2, FRAC_PI_2][side] > 0.0,
                "antenna {side} folds to {} rad, which is past straight down on the other \
                 antenna's side",
                fold[side],
            );
        }
    }

    /// The figures `reachy-motion`'s stillness watch sizes its settle allowance
    /// on, held to the documents they were transcribed from.
    ///
    /// That crate is sans-I/O and reads no asset, so its widest-arc case states
    /// the fold and the rest as literals; this is the assertion that fails when
    /// a re-recorded `stow` or a hand-edited `neutral` moves out from under
    /// them. Without it the allowance can stop covering the travel with every
    /// test green, and the watch then reads a rod still arriving as one that
    /// will not stand still.
    #[test]
    fn the_antennas_are_where_the_stillness_allowance_was_sized_on() {
        assert_eq!(
            targets(reachy_poses::STOW_POSE).antennas,
            [-3.32, 3.32],
            "crates/reachy-motion/src/stillness.rs states this fold as FOLD",
        );
        assert_eq!(
            targets(reachy_poses::NEUTRAL_POSE).antennas,
            [-0.1745, 0.1745],
            "crates/reachy-motion/src/stillness.rs states this rest as REST",
        );
    }
}
