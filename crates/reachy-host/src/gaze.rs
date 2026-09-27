//! Where the head looks when the wake word is heard: the voice host's answer to
//! the voice pipeline's gaze seam.
//!
//! Five steps, each a typed decline when it fails: a reading exists, the
//! auto-select beam is a copy of a focused beam, a head pose estimate is fresh,
//! the reading yields a world bearing, and the bearing picks the nearest rung
//! of [`LADDER`]. Every decision, a choice or a decline, is said as one `gaze`
//! line.
//!
//! A world bearing is measured from world x, positive to the robot's left. The
//! chip's azimuth α is the angle from the array axis, whose 0° end is the
//! robot's left. The talker's elevation is an assumption handed in, not a
//! reading: the array reports none.
//!
//! The pose estimate used is the newest one within [`MAX_POSE_AGE`] of the
//! choice, not the one at the reading's own instant. Nothing is clamped: every
//! outcome is a rung or a typed decline.
//!
//! Nothing here moves anything. It names a library pose, which the edge and the
//! mover screen like any other.

use std::sync::Arc;
use std::time::Duration;

use clockwork_rs::SyncTime;
use reachy_edge::{edge_line_with, now};
use reachy_kin::{MicError, array_axis_world, bearing_from_azimuth, talker_elevation_in_domain};
use serde_json::{Value, json};
use speech_pipeline::PodId;
use speech_surface::{DoaSample, GazePose, WakeGaze};

use crate::pose_feed::{HeadAttitude, PoseReader};
use crate::sinks::Lines;
use crate::words::GAZE;

/// The ladder: each pose and the world yaw it faces, degrees, positive to the
/// robot's left. Neutral first and then outward, so a bearing exactly between
/// two rungs takes the smaller turn.
pub const LADDER: [(&str, f64); 5] = [
    ("neutral", 0.0),
    ("look_l30", 30.0),
    ("look_r30", -30.0),
    ("look_l60", 60.0),
    ("look_r60", -60.0),
];

/// How old the head's estimate may be when a gaze is chosen.
pub const MAX_POSE_AGE: Duration = Duration::from_millis(200);

/// How close the auto-select beam must be to a focused beam to be a copy of it,
/// radians. The chip's copy reads bit-identical; this admits a difference of a
/// few steps of an f32 reading (one step is at most 2.4e-7 rad on [0, π]), not
/// a neighbouring bearing.
const COPY_TOLERANCE: f64 = 1e-6;

/// The talker's assumed elevation above the horizontal, validated.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Elevation(f64);

impl Elevation {
    /// The elevation `degrees` above the horizontal.
    ///
    /// # Errors
    ///
    /// [`ElevationError::NonFinite`] for NaN or an infinity;
    /// [`ElevationError::OutOfRange`] for a magnitude of 90 degrees or more.
    pub fn from_degrees(degrees: f64) -> Result<Self, ElevationError> {
        if !degrees.is_finite() {
            return Err(ElevationError::NonFinite);
        }
        let radians = degrees.to_radians();
        if !talker_elevation_in_domain(radians) {
            return Err(ElevationError::OutOfRange);
        }
        Ok(Self(radians))
    }

    /// The elevation in radians, strictly between −π/2 and π/2.
    #[must_use]
    pub const fn radians(self) -> f64 {
        self.0
    }
}

/// An elevation that is not one.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ElevationError {
    /// NaN or an infinity.
    #[error("the talker's elevation is not a finite number")]
    NonFinite,
    /// Outside the elevations the kinematics take: at or past straight up or
    /// straight down.
    #[error("the talker's elevation must be strictly between -90 and 90 degrees")]
    OutOfRange,
}

/// Where the head looks, and why.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Look {
    /// The ladder pose chosen.
    pub pose: &'static str,
    /// The world bearing it was chosen for, radians in (−π, π].
    pub bearing: f64,
    /// The bearing is the cone's end-fire edge, not a solution (see
    /// [`MicError::BeyondEndfire`]).
    pub end_fire: bool,
}

impl Look {
    /// The `reason` a choice is said under: `end_fire` when the bearing is the
    /// cone's edge, `bearing` otherwise.
    #[must_use]
    pub const fn word(&self) -> &'static str {
        if self.end_fire { "end_fire" } else { "bearing" }
    }
}

/// Why no gaze was chosen.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Decline {
    /// No azimuth reading arrived in the span the wake covers.
    NoDoa,
    /// The latest reading's auto-select beam tracked nothing.
    Nan,
    /// The latest reading's auto-select beam copied neither focused beam.
    AutoOnFreeBeam,
    /// No head pose estimate within [`MAX_POSE_AGE`].
    NoPose,
    /// The array axis was vertical, so the reading carries no bearing.
    ArrayVertical,
    /// The kinematics refused the reading as outside its domain or non-finite.
    Unreadable(MicError),
}

impl Decline {
    /// The `reason` a decline is said under.
    #[must_use]
    pub const fn word(&self) -> &'static str {
        match self {
            Self::NoDoa => "no_doa",
            Self::Nan => "nan",
            Self::AutoOnFreeBeam => "auto_on_free_beam",
            Self::NoPose => "no_pose",
            Self::ArrayVertical => "array_vertical",
            Self::Unreadable(_) => "unreadable",
        }
    }

    /// Why, as the clause a decline's line says.
    fn why(&self) -> String {
        match self {
            Self::NoDoa => "the array reported no azimuth during the end of the phrase".to_owned(),
            Self::Nan => "the auto-select beam tracked nothing".to_owned(),
            Self::AutoOnFreeBeam => {
                "the auto-select beam was on the free-running beam, not a focused one".to_owned()
            }
            Self::NoPose => format!(
                "no head pose estimate within {} ms",
                MAX_POSE_AGE.as_millis()
            ),
            Self::ArrayVertical => "the array axis was vertical".to_owned(),
            Self::Unreadable(error) => format!("the kinematics refused the reading: {error}"),
        }
    }
}

/// One wake's decision, with the auto-select azimuth it read when there was one.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Decision {
    /// `azimuths[3]` of the latest reading, radians, when a reading exists and
    /// it is finite.
    pub azimuth: Option<f64>,
    /// The look chosen, or why none was.
    pub outcome: Result<Look, Decline>,
}

/// The gaze for one wake: the first of the five steps that fails is the
/// decline.
///
/// Only the latest reading in `doa` is judged; `attitude` is the head's pose
/// already screened for age by the caller.
#[must_use]
pub fn decide(
    doa: &[DoaSample],
    attitude: Option<&HeadAttitude>,
    elevation: Elevation,
) -> Decision {
    let Some(sample) = doa.last() else {
        return Decision {
            azimuth: None,
            outcome: Err(Decline::NoDoa),
        };
    };
    let [a0, a1, _, a3] = sample.azimuths;
    if !a3.is_finite() {
        return Decision {
            azimuth: None,
            outcome: Err(Decline::Nan),
        };
    }
    let alpha = chip_azimuth(a3);
    let copies = |focused: f32| {
        focused.is_finite() && (f64::from(a3) - f64::from(focused)).abs() <= COPY_TOLERANCE
    };
    let outcome = if !(copies(a0) || copies(a1)) {
        Err(Decline::AutoOnFreeBeam)
    } else if let Some(attitude) = attitude {
        from_mic(bearing_from_azimuth(
            alpha,
            elevation.radians(),
            &array_axis_world(&attitude.head_quat_body, attitude.body_yaw),
        ))
        .map(|(bearing, end_fire)| Look {
            pose: rung(bearing),
            bearing,
            end_fire,
        })
    } else {
        Err(Decline::NoPose)
    };
    Decision {
        azimuth: Some(alpha),
        outcome,
    }
}

/// The chip's f32 azimuth as the f64 the kinematics takes.
///
/// The f32 nearest π is 3.14159274…, above f64 π, so the chip's own reading of
/// π — the right end-fire — would otherwise be refused as outside [0, π]. No
/// other f32 lies between the two, so that one value is read as f64 π: the
/// chip's π, not a saturation. Every other value converts exactly, and one above
/// f32 π stays above f64 π and is refused.
fn chip_azimuth(azimuth: f32) -> f64 {
    if azimuth.to_bits() == core::f32::consts::PI.to_bits() {
        core::f64::consts::PI
    } else {
        f64::from(azimuth)
    }
}

/// The kinematics' answer as a bearing and whether it is the end-fire edge, or
/// the decline it maps to.
fn from_mic(solved: Result<f64, MicError>) -> Result<(f64, bool), Decline> {
    match solved {
        Ok(bearing) => Ok((bearing, false)),
        Err(MicError::BeyondEndfire { edge, .. }) => Ok((edge, true)),
        Err(MicError::ArrayVertical) => Err(Decline::ArrayVertical),
        Err(error @ (MicError::NonFinite | MicError::OutOfRange)) => {
            Err(Decline::Unreadable(error))
        }
    }
}

/// The ladder pose nearest `bearing` (radians) by world yaw.
///
/// A rung replaces the one held only when strictly nearer, so a bearing exactly
/// between two takes the earlier one in [`LADDER`], the smaller turn. The
/// distance is taken in degrees, the unit the rungs are stated in, so those
/// midpoints are exact. Beyond ±60° the end rung on that side is chosen. No
/// wrapping is needed: for any bearing in (−π, π] the nearest rung is within
/// 120°. A NaN bearing is never nearer than anything and yields `neutral`.
#[must_use]
pub fn rung(bearing: f64) -> &'static str {
    let degrees = bearing.to_degrees();
    let (mut nearest, first) = LADDER[0];
    let mut distance = (degrees - first).abs();
    for (name, yaw) in &LADDER[1..] {
        let this = (degrees - yaw).abs();
        if this < distance {
            nearest = name;
            distance = this;
        }
    }
    nearest
}

/// The voice host's gaze policy, as the seam the voice pipeline asks.
pub struct Gaze {
    elevation: Elevation,
    poses: Option<PoseReader>,
    lines: Arc<dyn Lines>,
}

impl Gaze {
    /// A policy assuming the talker at `elevation`, reading the head's pose
    /// through `poses` and saying each decision on `lines`.
    ///
    /// `poses: None` is a host whose pose feed did not start: every wake then
    /// declines `no_pose`.
    #[must_use]
    pub fn new(elevation: Elevation, poses: Option<PoseReader>, lines: Arc<dyn Lines>) -> Self {
        Self {
            elevation,
            poses,
            lines,
        }
    }
}

impl WakeGaze for Gaze {
    fn choose(&self, pod: &PodId, doa: &[DoaSample], wake_end_sample: u64) -> Option<GazePose> {
        let at = now();
        let attitude = self
            .poses
            .as_ref()
            .and_then(|poses| poses.latest(at, MAX_POSE_AGE));
        let decision = decide(doa, attitude.as_ref(), self.elevation);
        self.lines
            .say(gaze_line(&pod.0, &decision, doa.len(), wake_end_sample, at));
        decision.outcome.ok().map(|look| GazePose {
            name: look.pose.to_owned(),
            move_ms: None,
        })
    }
}

/// The `gaze` line for one decision.
#[must_use]
pub fn gaze_line(
    pod: &str,
    decision: &Decision,
    doa_count: usize,
    wake_end_sample: u64,
    at: SyncTime,
) -> String {
    let degrees = |radians: Option<f64>| radians.map_or(Value::Null, |r| json!(r.to_degrees()));
    let (chosen, reason, bearing, says) = match &decision.outcome {
        Ok(look) => {
            let edge = if look.end_fire {
                "; the reading is past end-fire, so that is the cone's edge on that side"
            } else {
                ""
            };
            (
                json!(look.pose),
                look.word(),
                Some(look.bearing),
                format!(
                    "the talker is at {:.0}° world, so the head looks with `{}`{edge}",
                    look.bearing.to_degrees(),
                    look.pose,
                ),
            )
        }
        Err(decline) => (
            Value::Null,
            decline.word(),
            None,
            format!(
                "no gaze for this wake ({}): {}; the wake takes the configured pose",
                decline.word(),
                decline.why(),
            ),
        ),
    };
    edge_line_with(
        GAZE,
        at,
        &says,
        &[
            ("pod", json!(pod)),
            ("chosen", chosen),
            ("reason", json!(reason)),
            ("azimuth_deg", degrees(decision.azimuth)),
            ("bearing_deg", degrees(bearing)),
            ("doa_count", json!(doa_count)),
            ("wake_end_sample", json!(wake_end_sample)),
        ],
    )
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use core::f32::consts::PI as PI32;
    use core::f64::consts::PI;

    use nalgebra::{UnitQuaternion, Vector3};

    use super::*;

    /// Every line said, in order.
    #[derive(Default)]
    struct Said(Mutex<Vec<String>>);

    impl Lines for Said {
        fn say(&self, line: String) {
            self.0.lock().expect("an unpoisoned recorder").push(line);
        }
    }

    impl Said {
        fn lines(&self) -> Vec<Value> {
            self.0
                .lock()
                .expect("an unpoisoned recorder")
                .iter()
                .map(|line| serde_json::from_str(line).expect("a JSON line"))
                .collect()
        }
    }

    /// Bearings go through an f32 azimuth, so they agree to this, radians.
    const TOLERANCE: f64 = 1e-4;

    fn elevation() -> Elevation {
        Elevation::from_degrees(27.0).expect("a lawful elevation")
    }

    /// The head yawed `head_yaw_deg` in the body frame, the body `body_yaw_deg`.
    fn attitude(head_yaw_deg: f64, body_yaw_deg: f64) -> HeadAttitude {
        HeadAttitude {
            at: SyncTime::from_nanos(0),
            head_quat_body: UnitQuaternion::from_euler_angles(0.0, 0.0, head_yaw_deg.to_radians()),
            body_yaw: body_yaw_deg.to_radians(),
        }
    }

    /// The head rolled `roll_deg`, body square.
    fn rolled(roll_deg: f64) -> HeadAttitude {
        HeadAttitude {
            at: SyncTime::from_nanos(0),
            head_quat_body: UnitQuaternion::from_euler_angles(roll_deg.to_radians(), 0.0, 0.0),
            body_yaw: 0.0,
        }
    }

    /// The azimuth the array reads for a talker at world bearing `bearing_deg`
    /// and 27° elevation, with the head at `attitude`.
    #[allow(clippy::cast_possible_truncation)]
    fn toward(bearing_deg: f64, attitude: &HeadAttitude) -> f32 {
        let (phi, theta) = (27f64.to_radians(), bearing_deg.to_radians());
        let d = Vector3::new(phi.cos() * theta.cos(), phi.cos() * theta.sin(), phi.sin());
        array_axis_world(&attitude.head_quat_body, attitude.body_yaw)
            .dot(&d)
            .acos() as f32
    }

    /// A reading whose auto-select beam copies the first focused beam at `alpha`.
    fn focused(alpha: f32) -> DoaSample {
        DoaSample {
            sample: 0,
            azimuths: [alpha, 1.0, 1.5, alpha],
        }
    }

    fn reading(azimuths: [f32; 4]) -> DoaSample {
        DoaSample {
            sample: 0,
            azimuths,
        }
    }

    fn look(decision: &Decision) -> Look {
        decision
            .outcome
            .unwrap_or_else(|decline| panic!("a look, not {decline:?}"))
    }

    #[test]
    fn no_reading_declines_no_doa() {
        let decision = decide(&[], Some(&attitude(0.0, 0.0)), elevation());
        assert_eq!(
            decision,
            Decision {
                azimuth: None,
                outcome: Err(Decline::NoDoa)
            }
        );
    }

    #[test]
    fn a_nan_auto_select_declines_nan() {
        let decision = decide(
            &[reading([1.0, 1.0, 1.5, f32::NAN])],
            Some(&attitude(0.0, 0.0)),
            elevation(),
        );
        assert_eq!(
            decision,
            Decision {
                azimuth: None,
                outcome: Err(Decline::Nan)
            }
        );
    }

    #[test]
    fn an_auto_select_on_the_free_beam_declines() {
        let decision = decide(
            &[reading([0.5, 0.7, 1.2, 1.2])],
            Some(&attitude(0.0, 0.0)),
            elevation(),
        );
        assert_eq!(
            decision,
            Decision {
                azimuth: Some(f64::from(1.2f32)),
                outcome: Err(Decline::AutoOnFreeBeam)
            }
        );
    }

    #[test]
    fn a_tie_between_the_focused_beams_is_a_copy() {
        let level = attitude(0.0, 0.0);
        let alpha = toward(0.0, &level);
        look(&decide(
            &[reading([alpha, alpha, 0.3, alpha])],
            Some(&level),
            elevation(),
        ));
    }

    #[test]
    fn a_copy_of_the_second_focused_beam_is_a_copy() {
        let level = attitude(0.0, 0.0);
        let alpha = toward(0.0, &level);
        look(&decide(
            &[reading([f32::NAN, alpha, 0.3, alpha])],
            Some(&level),
            elevation(),
        ));
    }

    #[test]
    fn a_copy_is_within_float_steps_and_a_neighbour_is_not() {
        let level = attitude(0.0, 0.0);
        let alpha = toward(0.0, &level);
        let step = f32::from_bits(alpha.to_bits() + 1);
        assert!(
            (f64::from(step) - f64::from(alpha)).abs() <= COPY_TOLERANCE,
            "one step is inside the tolerance"
        );
        look(&decide(
            &[reading([alpha, 1.0, 0.3, step])],
            Some(&level),
            elevation(),
        ));
        let near = alpha + 1e-3;
        assert_eq!(
            decide(
                &[reading([alpha, 1.0, 0.3, near])],
                Some(&level),
                elevation()
            )
            .outcome,
            Err(Decline::AutoOnFreeBeam)
        );
    }

    #[test]
    fn no_pose_declines_no_pose() {
        let decision = decide(&[focused(1.0)], None, elevation());
        assert_eq!(decision.outcome, Err(Decline::NoPose));
        assert_eq!(decision.azimuth, Some(1.0));
    }

    #[test]
    fn the_latest_reading_decides() {
        let level = attitude(0.0, 0.0);
        let good = focused(toward(0.0, &level));
        let free = reading([0.5, 0.7, 1.2, 1.2]);
        let decision = decide(&[good, free], Some(&level), elevation());
        assert_eq!(decision.outcome, Err(Decline::AutoOnFreeBeam));
        look(&decide(&[free, good], Some(&level), elevation()));
    }

    #[test]
    fn the_mic_s_refusals_map_to_declines() {
        assert_eq!(
            from_mic(Err(MicError::ArrayVertical)),
            Err(Decline::ArrayVertical)
        );
        assert!(matches!(
            from_mic(Err(MicError::NonFinite)),
            Err(Decline::Unreadable(_))
        ));
        assert!(matches!(
            from_mic(Err(MicError::OutOfRange)),
            Err(Decline::Unreadable(_))
        ));
        assert_eq!(
            from_mic(Err(MicError::BeyondEndfire {
                left: false,
                edge: -1.0
            })),
            Ok((-1.0, true))
        );
        assert_eq!(from_mic(Ok(0.3)), Ok((0.3, false)));
    }

    #[test]
    fn the_chip_s_pi_is_read_as_pi_and_nothing_past_it() {
        assert_eq!(chip_azimuth(PI32).to_bits(), PI.to_bits());
        let end = look(&decide(
            &[focused(PI32)],
            Some(&attitude(0.0, 0.0)),
            elevation(),
        ));
        assert!(end.end_fire, "{end:?}");
        assert_eq!(end.pose, "look_r60");
        let past = f32::from_bits(PI32.to_bits() + 1);
        assert_eq!(
            decide(&[focused(past)], Some(&attitude(0.0, 0.0)), elevation()).outcome,
            Err(Decline::Unreadable(MicError::OutOfRange))
        );
    }

    #[test]
    fn every_decline_has_its_word() {
        let words: Vec<&str> = [
            Decline::NoDoa,
            Decline::Nan,
            Decline::AutoOnFreeBeam,
            Decline::NoPose,
            Decline::ArrayVertical,
            Decline::Unreadable(MicError::OutOfRange),
        ]
        .iter()
        .map(Decline::word)
        .collect();
        assert_eq!(
            words,
            [
                "no_doa",
                "nan",
                "auto_on_free_beam",
                "no_pose",
                "array_vertical",
                "unreadable"
            ]
        );
    }

    /// A talker at `target_deg` world, heard with the head yawed +20° on a body
    /// yawed −10°, gets `pose`.
    fn talker_at(target_deg: f64, pose: &str) {
        let yawed = attitude(20.0, -10.0);
        let chosen = look(&decide(
            &[focused(toward(target_deg, &yawed))],
            Some(&yawed),
            elevation(),
        ));
        assert_eq!(chosen.pose, pose);
        assert!(!chosen.end_fire, "{chosen:?}");
        assert!(
            (chosen.bearing - target_deg.to_radians()).abs() < TOLERANCE,
            "{chosen:?}"
        );
    }

    #[test]
    fn a_talker_at_each_rung_gets_that_rung() {
        for (pose, yaw) in LADDER {
            talker_at(yaw, pose);
        }
    }

    #[test]
    fn a_rolled_head_reads_a_talker_ahead_as_ahead() {
        let tilted = rolled(26.0);
        let chosen = look(&decide(
            &[focused(toward(0.0, &tilted))],
            Some(&tilted),
            elevation(),
        ));
        assert_eq!(chosen.pose, "neutral");
        assert!(chosen.bearing.abs() < TOLERANCE, "{chosen:?}");
    }

    #[test]
    fn an_end_fire_reading_bins_by_the_edge_not_ninety() {
        let turned = look(&decide(
            &[focused(PI32)],
            Some(&attitude(30.0, 20.0)),
            elevation(),
        ));
        assert_eq!(turned.pose, "look_r30");
        assert!(turned.end_fire, "{turned:?}");
        assert!(
            (turned.bearing - (-40f64).to_radians()).abs() < TOLERANCE,
            "{turned:?}"
        );
        let square = look(&decide(
            &[focused(PI32)],
            Some(&attitude(0.0, 0.0)),
            elevation(),
        ));
        assert_eq!(square.pose, "look_r60");
        assert!(square.end_fire, "{square:?}");
        assert!(
            (square.bearing - (-90f64).to_radians()).abs() < TOLERANCE,
            "{square:?}"
        );
    }

    #[test]
    fn rung_boundaries_take_the_smaller_turn() {
        for (degrees, pose) in [
            (15.0, "neutral"),
            (-15.0, "neutral"),
            (45.0, "look_l30"),
            (-45.0, "look_r30"),
            (16.0, "look_l30"),
            (46.0, "look_l60"),
            (120.0, "look_l60"),
            (-170.0, "look_r60"),
            (180.0, "look_l60"),
        ] {
            assert_eq!(rung(f64::to_radians(degrees)), pose, "{degrees}°");
        }
        assert_eq!(rung(f64::NAN), "neutral");
    }

    #[test]
    fn the_elevation_is_refused_outside_the_open_quarter_turn() {
        let lawful = Elevation::from_degrees(27.0).expect("27° is lawful");
        assert!((lawful.radians() - 27f64.to_radians()).abs() < 1e-15);
        assert!(Elevation::from_degrees(-10.0).is_ok());
        assert!(Elevation::from_degrees(89.999).is_ok());
        for degrees in [90.0, -90.0, 120.0] {
            assert_eq!(
                Elevation::from_degrees(degrees),
                Err(ElevationError::OutOfRange),
                "{degrees}"
            );
        }
        for degrees in [f64::NAN, f64::INFINITY] {
            assert_eq!(
                Elevation::from_degrees(degrees),
                Err(ElevationError::NonFinite),
                "{degrees}"
            );
        }
    }

    fn pod() -> PodId {
        PodId("pod-a".into())
    }

    #[test]
    fn a_fresh_pose_and_a_focused_reading_choose_a_look_and_say_it() {
        let level = attitude(0.0, 0.0);
        let said = Arc::new(Said::default());
        let gaze = Gaze::new(
            elevation(),
            Some(PoseReader::holding(HeadAttitude { at: now(), ..level })),
            Arc::clone(&said) as Arc<dyn Lines>,
        );
        let chosen = gaze.choose(&pod(), &[focused(toward(30.0, &level))], 12_345);
        assert_eq!(
            chosen,
            Some(GazePose {
                name: "look_l30".to_owned(),
                move_ms: None
            })
        );
        let lines = said.lines();
        assert_eq!(lines.len(), 1, "{lines:?}");
        let line = &lines[0];
        assert_eq!(line["kind"], "gaze");
        assert_eq!(line["pod"], "pod-a");
        assert_eq!(line["chosen"], "look_l30");
        assert_eq!(line["reason"], "bearing");
        assert_eq!(line["doa_count"], 1);
        assert_eq!(line["wake_end_sample"], 12_345);
        assert!(line["azimuth_deg"].is_f64(), "{line}");
        let bearing = line["bearing_deg"].as_f64().expect("a numeric bearing");
        assert!((bearing - 30.0).abs() < 0.01, "{line}");
    }

    #[test]
    fn a_stale_pose_declines_no_pose() {
        let level = attitude(0.0, 0.0);
        let stale = SyncTime::from_nanos(now().as_nanos() - 1_000_000_000);
        let said = Arc::new(Said::default());
        let gaze = Gaze::new(
            elevation(),
            Some(PoseReader::holding(HeadAttitude { at: stale, ..level })),
            Arc::clone(&said) as Arc<dyn Lines>,
        );
        assert_eq!(
            gaze.choose(&pod(), &[focused(toward(30.0, &level))], 1),
            None
        );
        let lines = said.lines();
        let line = &lines[0];
        assert!(line["chosen"].is_null(), "{line}");
        assert_eq!(line["reason"], "no_pose");
        assert!(line["bearing_deg"].is_null(), "{line}");
        assert!(line["azimuth_deg"].is_f64(), "{line}");
    }

    #[test]
    fn a_gaze_with_no_feed_declines_no_pose() {
        let said = Arc::new(Said::default());
        let gaze = Gaze::new(elevation(), None, Arc::clone(&said) as Arc<dyn Lines>);
        assert_eq!(gaze.choose(&pod(), &[focused(1.0)], 1), None);
        let lines = said.lines();
        assert_eq!(lines.len(), 1, "{lines:?}");
        assert_eq!(lines[0]["reason"], "no_pose");
    }

    #[test]
    fn a_decline_with_no_reading_says_nulls() {
        let said = Arc::new(Said::default());
        let gaze = Gaze::new(elevation(), None, Arc::clone(&said) as Arc<dyn Lines>);
        assert_eq!(gaze.choose(&pod(), &[], 7), None);
        let lines = said.lines();
        let line = &lines[0];
        assert!(line["azimuth_deg"].is_null(), "{line}");
        assert!(line["bearing_deg"].is_null(), "{line}");
        assert!(line["chosen"].is_null(), "{line}");
        assert_eq!(line["reason"], "no_doa");
        assert_eq!(line["doa_count"], 0);
    }
}
