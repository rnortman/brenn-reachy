//! The extractor's half: turning what a recording read into what a document
//! states.
//!
//! A hold the recorder cut becomes a [`PoseDoc`] here and protobuf text beside
//! it, with the envelope's verdict on the pose reported for the author to read.
//! The reporting is not a gate: the loader runs the same check and refuses, and
//! a draft is written whatever the verdict says, because the numbers of a pose
//! that cannot be an asset are what tell an author whether the recording or the
//! limit is the thing to look at.
//!
//! An antenna target is a **direction** — an angle mod 2π — everywhere on the
//! command path, and the runtime resolves one to the short arc from wherever the
//! antenna is standing. A recording is not: `counts_to_rad` is deliberately
//! unwrapped, because a de-torqued servo reports multi-turn position and
//! throwing the turns away would lose the fact that a hand spun the rod round.
//! So a reading transcribes into a document through one step, and it is the one
//! this module performs.
//!
//! The reduction is a change of representative, not a clamp: the angle it writes
//! names the same direction the servo reported, and no value is moved to a
//! bound. That distinction is why it is allowed to exist at all in a tree whose
//! rule is that a value out of range is a refusal — there is no range here, only
//! a choice of which of infinitely many equal spellings to write down.
//!
//! **Pose extraction only.** A clip's antenna channel carries *deltas* against
//! the base the clip was recorded over, and a whole-turn error in one of those
//! is a content error rather than a spelling: two directions that differ by a
//! turn are the same place, but two deltas that differ by a turn are two
//! different sweeps.

use core::f64::consts::TAU;
use core::fmt::Write as _;

use nalgebra::Isometry3;
use reachy_kin::{
    EnvelopeConfig, EnvelopeReport, check_envelope, default_geometry, neutral_head_pose,
};
use thiserror::Error;

use crate::format::PoseDoc;

const REDUCTION_CENTRES: [f64; 2] = [-core::f64::consts::FRAC_PI_2, core::f64::consts::FRAC_PI_2];

/// Both antenna readings reduced to the directions they name, right then left.
#[must_use]
pub fn reduce_antennas(readings: [f64; 2]) -> [f64; 2] {
    [
        reduce_to_turn(readings[0], REDUCTION_CENTRES[0]),
        reduce_to_turn(readings[1], REDUCTION_CENTRES[1]),
    ]
}

/// The one angle in `(outboard − π, outboard + π]` that names the same
/// direction as `angle`.
///
/// Centred on the antenna's own outboard direction rather than on zero, so the
/// window's seam falls where the rod physically never stands: each antenna's
/// whole travel — the rest lean a tenth of a radian off straight up, the fold a
/// little past straight down — sits on one side of its outboard point and well
/// inside half a turn of it. A window centred anywhere else could cut that
/// travel in two and give one pose two spellings.
///
/// Non-finite in, non-finite out: what a document may state is the loader's
/// question, and inventing a number here would hide a reading that never was
/// one.
#[must_use]
pub fn reduce_to_turn(angle: f64, outboard: f64) -> f64 {
    if !angle.is_finite() {
        return angle;
    }
    let offset = (angle - outboard).rem_euclid(TAU);
    // `rem_euclid` lands in [0, τ); the half-open window is (−π, π], so the
    // upper half folds down and π itself stays where it is.
    let offset = if offset > core::f64::consts::PI {
        offset - TAU
    } else {
        offset
    };
    outboard + offset
}

/// What a recording read of one hold, as the joints stood.
///
/// Every quantity is optional and absent means *unread*, not zero: zero is a
/// legal reading of all three, and a slot carrying the missing case as a number
/// would extract into a document nobody could tell from a measurement.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RecordedPose {
    /// The head pose in the body frame, absolute, as forward kinematics solved
    /// it; `None` where no seed closed the linkage.
    pub head: Option<Isometry3<f64>>,
    /// Body yaw, radians, absolute.
    pub body_yaw: Option<f64>,
    /// Antenna angles, right then left, radians, as the servos reported them —
    /// unwrapped, so possibly several turns from the direction they name.
    pub antennas: [Option<f64>; 2],
}

/// Why a recorded stretch is not a pose document.
///
/// Each arm refuses the whole extraction. A pose is a whole configuration, so
/// there is no partial document and no channel left unsaid.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum PoseExtractError {
    /// The stretch is a move. A pose is where the machine *stood*, and a move
    /// has no such place — its mean is an average of everywhere it went.
    #[error("a pose is extracted from a hold; this stretch is a move")]
    NotStill,
    /// A channel went unread over the whole stretch.
    #[error("the recording carries no reading of {channel}: a pose states every channel")]
    Unread {
        /// Which channel had nothing in it.
        channel: &'static str,
    },
}

/// The pose document one hold extracts to.
///
/// `duration_ms` is left at zero, which the loader refuses (`DurationZero`): a
/// recording says where the machine stood and nothing about how fast a move to
/// there should go, so the author states the pace and a draft nobody edited
/// cannot be committed. The antennas are reduced to the directions they name;
/// everything else transcribes with a unit change and nothing else.
///
/// # Errors
///
/// [`PoseExtractError`]: a move rather than a hold, or a channel the recording
/// never read.
pub fn pose_doc(
    name: &str,
    description: String,
    still: bool,
    read: &RecordedPose,
) -> Result<PoseDoc, PoseExtractError> {
    if !still {
        return Err(PoseExtractError::NotStill);
    }
    let unread = |channel: &'static str| PoseExtractError::Unread { channel };
    let head = read.head.ok_or_else(|| unread("the head pose"))?;
    let body_yaw = read.body_yaw.ok_or_else(|| unread("body yaw"))?;
    let mut antennas = [0.0; 2];
    for (side, angle) in antennas.iter_mut().enumerate() {
        *angle = read.antennas[side].ok_or_else(|| {
            unread(if side == 0 {
                "the right antenna"
            } else {
                "the left antenna"
            })
        })?;
    }
    let relative = neutral_head_pose().inverse() * head;
    let q = relative.rotation.quaternion();
    Ok(PoseDoc {
        name: name.to_owned(),
        description,
        dt: [
            relative.translation.vector.x,
            relative.translation.vector.y,
            relative.translation.vector.z,
        ],
        dq: [q.w, q.i, q.j, q.k],
        body_yaw,
        antennas: reduce_antennas(antennas),
        duration_ms: 0,
    })
}

/// A document as the protobuf text a `cogs/poses/` file holds.
///
/// Field by field rather than through a serializer: the text is read by a
/// person as much as by a parser, the two comments below are part of what it
/// says, and every field is stated because a pose states every channel. The numbers are printed
/// at the shortest spelling that reads back as the same `f64`, so a document
/// round-trips through the reader unchanged.
#[must_use]
pub fn pose_text(doc: &PoseDoc) -> String {
    let mut out = String::new();
    let _ = writeln!(out, "name: {}", quoted(&doc.name));
    let _ = writeln!(out, "description: {}", quoted(&doc.description));
    let _ = writeln!(
        out,
        "# Head relative to the neutral head pose: metres, unit quaternion [w, x, y, z]."
    );
    let _ = writeln!(out, "dt: [{}]", numbers(&doc.dt));
    let _ = writeln!(out, "dq: [{}]", numbers(&doc.dq));
    let _ = writeln!(out, "body_yaw: {}", number(doc.body_yaw));
    let _ = writeln!(out, "# Right, then left; radians, a direction.");
    let _ = writeln!(out, "antennas: [{}]", numbers(&doc.antennas));
    let _ = writeln!(out, "duration_ms: {}", doc.duration_ms);
    out
}

/// One string as a protobuf text literal.
///
/// Not Rust's own `Debug`, which spells an unprintable character `\u{XXXX}`:
/// the text format's escape is `\uXXXX`, four hex digits and no braces, and a
/// brace after `\u` is a lexer error rather than a character. A description
/// carries whatever the session it names carries, so the writer for this format
/// is this format's.
///
/// Printable text goes through as the UTF-8 it is — the format's string
/// literals hold bytes — and only the three characters a literal cannot contain
/// and the control characters are escaped.
fn quoted(text: &str) -> String {
    let mut out = String::with_capacity(text.len() + 2);
    out.push('"');
    for ch in text.chars() {
        match ch {
            '"' => out.push_str("\\\""),
            '\\' => out.push_str("\\\\"),
            '\n' => out.push_str("\\n"),
            '\r' => out.push_str("\\r"),
            '\t' => out.push_str("\\t"),
            ch if (ch as u32) < 0x20 || ch as u32 == 0x7f => {
                let _ = write!(out, "\\u{:04x}", ch as u32);
            }
            ch => out.push(ch),
        }
    }
    out.push('"');
    out
}

/// One number as protobuf text: the shortest spelling that reads back as the
/// same `f64`.
fn number(value: f64) -> String {
    format!("{value:?}")
}

/// A list of them, comma-separated.
fn numbers(values: &[f64]) -> String {
    values
        .iter()
        .map(|value| number(*value))
        .collect::<Vec<String>>()
        .join(", ")
}

/// What the envelope says about a document's pose, as the author reads it.
///
/// The check the loader itself runs — the default configuration, no margin
/// baseline — reported rather than enforced, so the numbers behind a refusal
/// are in front of whoever is choosing the recording. The per-leg clearances
/// are the ones a rest is judged by, and the minimum is printed against the
/// floor it has to clear, because a still that misses it by a tenth of a
/// millimetre and one that misses it by ten are different situations.
///
/// Reporting only: nothing here moves a figure, and a pose this refuses is
/// still the pose the draft states.
#[must_use]
pub fn envelope_verdict(doc: &PoseDoc) -> String {
    let env = EnvelopeConfig::default();
    let mut report = EnvelopeReport::default();
    let verdict = check_envelope(
        default_geometry(),
        &env,
        &doc.head_pose_body(),
        doc.body_yaw,
        None,
        &mut report,
    );
    let margins = report
        .toggle_margins
        .iter()
        .map(|margin| format!("{:.2}", margin * 1e3))
        .collect::<Vec<String>>()
        .join(", ");
    let mut out = String::new();
    let _ = writeln!(out, "  toggle margins, mm, leg 1..6: [{margins}]");
    let _ = writeln!(
        out,
        "  smallest {:.2} mm against a floor of {:.2} mm",
        report.min_margin * 1e3,
        env.min_toggle_margin * 1e3
    );
    let _ = writeln!(
        out,
        "  head cone {:.2}° of {:.2}°, relative yaw {:.2}° of {:.2}°",
        report.cone_angle.to_degrees(),
        env.head_cone_limit.to_degrees(),
        report.relative_yaw.to_degrees(),
        env.relative_yaw_limit.to_degrees()
    );
    match verdict {
        Ok(()) => {
            let _ = writeln!(out, "  envelope: ok");
        }
        Err(error) => {
            let _ = writeln!(out, "  envelope: {error}");
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::format::Pose;
    use core::f64::consts::{PI, TAU};
    use nalgebra::{Translation3, UnitQuaternion, Vector3};
    use reachy_kin::ik::min_pose_margin;
    /// The same direction, to within the arithmetic.
    fn same_direction(a: f64, b: f64) {
        let gap = (a - b).rem_euclid(TAU);
        let gap = gap.min(TAU - gap);
        assert!(gap < 1e-9, "{a} and {b} are not the same direction");
    }

    /// Readings from a real recording session. The left antenna physically
    /// stood a little past straight down, at +3.36; the right stood just off
    /// straight up, at −0.29.
    #[test]
    fn the_recorded_readings_reduce_to_where_the_rods_stood() {
        let left = reduce_to_turn(-2.92, REDUCTION_CENTRES[1]);
        assert!(
            (left - 3.3632).abs() < 1e-3,
            "left read -2.92, reduced to {left}"
        );
        same_direction(left, -2.92);

        let right = reduce_to_turn(-6.57, REDUCTION_CENTRES[0]);
        assert!(
            (right - -0.2868).abs() < 1e-3,
            "right read -6.57, reduced to {right}"
        );
        same_direction(right, -6.57);

        assert_eq!(reduce_antennas([-6.57, -2.92]), [right, left]);
    }

    /// A reading already inside its own window is the number the document
    /// states: the reduction is a change of representative and nothing else.
    #[test]
    fn a_reading_already_in_its_window_is_untouched() {
        // Each side's own travel: the rest lean off straight up, the fold past
        // straight down, and straight up between them.
        for (side, (&outboard, travel)) in REDUCTION_CENTRES
            .iter()
            .zip([[-0.1745, -3.32, 0.0], [0.1745, 3.32, 0.0]])
            .enumerate()
        {
            for angle in travel {
                let reduced = reduce_to_turn(angle, outboard);
                assert!(
                    (reduced - angle).abs() < 1e-12,
                    "antenna {side} reading {angle} moved to {reduced}"
                );
            }
        }
    }

    /// The window is half-open: its upper edge is inside and its lower edge
    /// folds up to the upper one, so one direction has exactly one spelling.
    #[test]
    fn the_window_is_half_open_at_its_lower_edge() {
        for &outboard in &REDUCTION_CENTRES {
            let upper = outboard + PI;
            assert!((reduce_to_turn(upper, outboard) - upper).abs() < 1e-12);
            assert!((reduce_to_turn(outboard - PI, outboard) - upper).abs() < 1e-12);
        }
    }

    /// Whole turns either way name the same place and reduce to the same
    /// spelling.
    #[test]
    fn whole_turns_either_way_reduce_to_one_spelling() {
        for &outboard in &REDUCTION_CENTRES {
            let stood = outboard + 0.4;
            for turns in [-3.0, -1.0, 0.0, 1.0, 2.0] {
                let read = stood + turns * TAU;
                assert!(
                    (reduce_to_turn(read, outboard) - stood).abs() < 1e-9,
                    "{read} does not reduce to {stood}"
                );
            }
        }
    }

    /// A reading that is not a number stays one: what a document may state is
    /// the loader's question.
    #[test]
    fn a_reading_that_is_not_a_number_is_carried_through() {
        assert!(reduce_to_turn(f64::NAN, REDUCTION_CENTRES[0]).is_nan());
        assert!(reduce_to_turn(f64::INFINITY, REDUCTION_CENTRES[1]).is_infinite());
    }

    /// A hold recorded with the head a little below neutral, the antennas read
    /// a turn away from where they stood.
    fn recorded() -> RecordedPose {
        let head = neutral_head_pose()
            * Isometry3::from_parts(
                Translation3::new(0.0, 0.0, -0.02),
                UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.1),
            );
        RecordedPose {
            head: Some(head),
            body_yaw: Some(0.25),
            antennas: [Some(-6.57), Some(-2.92)],
        }
    }

    /// The transcription a document states: the head relative to neutral in
    /// metres, the antennas as the directions they name, and a pace the author
    /// has to supply.
    #[test]
    fn a_hold_extracts_to_the_document_it_stood_at() {
        let doc = pose_doc("peek", "a session".to_owned(), true, &recorded())
            .expect("a hold with every channel read");
        assert_eq!(doc.name, "peek");
        assert!((doc.dt[2] - -0.02).abs() < 1e-12, "dt {:?}", doc.dt);
        assert!(doc.dt[0].abs() < 1e-12 && doc.dt[1].abs() < 1e-12);
        assert!((doc.dq[0] - (0.1_f64 / 2.0).cos()).abs() < 1e-9);
        assert!((doc.body_yaw - 0.25).abs() < 1e-12);
        assert_eq!(doc.antennas, reduce_antennas([-6.57, -2.92]));
        assert_eq!(doc.duration_ms, 0, "the author states the pace");
    }

    /// A move has no place the machine stood, and a channel the recording never
    /// read has no reading a whole configuration could carry.
    #[test]
    fn a_move_and_an_unread_channel_are_both_refused() {
        assert_eq!(
            pose_doc("peek", String::new(), false, &recorded()),
            Err(PoseExtractError::NotStill)
        );
        for (index, channel) in [
            "the head pose",
            "body yaw",
            "the right antenna",
            "the left antenna",
        ]
        .into_iter()
        .enumerate()
        {
            let mut read = recorded();
            match index {
                0 => read.head = None,
                1 => read.body_yaw = None,
                2 => read.antennas[0] = None,
                // The side label is a branch, so both arms are driven: an
                // operator sent to the wrong antenna is sent to a recording
                // they may not be able to repeat.
                _ => read.antennas[1] = None,
            }
            assert_eq!(
                pose_doc("peek", String::new(), true, &read),
                Err(PoseExtractError::Unread { channel })
            );
        }
    }

    /// The text is the document: what the extractor prints reads back through
    /// the parser as the same figures, to the bit. A draft that did not is a
    /// draft an author would have to retype.
    #[test]
    fn the_printed_draft_reads_back_as_the_document_it_states() {
        let mut doc = pose_doc(
            "peek",
            "S003 of a session, \"quoted\" and all".to_owned(),
            true,
            &recorded(),
        )
        .expect("a hold with every channel read");
        doc.duration_ms = 800;
        let text = pose_text(&doc);
        assert_eq!(
            crate::format::parse_document(&text).expect("the draft parses"),
            doc
        );
        // The pace the author has to supply is refused until they do.
        let mut draft = doc.clone();
        draft.duration_ms = 0;
        assert!(Pose::from_text(&pose_text(&draft)).is_err());
    }

    /// A description carries whatever the session it names carries. The text
    /// format's escape is `\uXXXX`; Rust's own is `\u{XXXX}`, which the parser
    /// reads as a broken escape rather than as a character, so the draft has to
    /// be written by a writer for this format.
    #[test]
    fn a_description_of_awkward_characters_reads_back_as_itself() {
        for description in [
            "a session whose name carries a combining mark: e\u{0301}",
            "a zero-width\u{200b}join and a control\u{0001}byte",
            "\"quoted\", back\\slashed, tabbed\tand\nbroken across lines",
            "unicode beyond the basic plane: \u{1f9ff}",
        ] {
            let mut doc =
                pose_doc("peek", description.to_owned(), true, &recorded()).expect("a hold");
            doc.duration_ms = 800;
            let text = pose_text(&doc);
            assert!(
                !text.contains("\\u{"),
                "the draft carries a Rust escape the parser cannot read: {text}"
            );
            assert_eq!(
                crate::format::parse_document(&text).expect("the draft parses"),
                doc
            );
        }
    }

    /// The verdict reports the loader's own check without enforcing it: the
    /// minimum it prints is the pose's clearance, and a pose outside the
    /// envelope is named rather than refused.
    #[test]
    fn the_verdict_reports_the_clearance_and_names_a_violation() {
        let mut doc = pose_doc("peek", String::new(), true, &recorded()).expect("a hold");
        doc.duration_ms = 800;
        let margin = min_pose_margin(reachy_kin::default_geometry(), &doc.head_pose_body());
        let said = envelope_verdict(&doc);
        assert!(
            said.contains(&format!("smallest {:.2} mm", margin * 1e3)),
            "{said}"
        );
        assert!(said.contains("envelope: ok"), "{said}");

        // A head pitched past the cone: the verdict names the check, and the
        // document is untouched by having been asked.
        let mut past = doc.clone();
        let tipped = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 0.8);
        let q = tipped.quaternion();
        past.dq = [q.w, q.i, q.j, q.k];
        let said = envelope_verdict(&past);
        assert!(said.contains("head attitude outside the cone"), "{said}");
        assert!(Pose::from_doc(doc.clone()).is_ok());
    }
}
