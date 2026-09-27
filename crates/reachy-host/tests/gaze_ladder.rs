//! The committed gaze ladder against the committed pose documents and name
//! table.
//!
//! The gaze bins a talker's world bearing by the yaw [`reachy_host::LADDER`]
//! states for each rung and names the rung's pose; the pose documents are what
//! the head then goes to. These cases hold every rung's document to facing that
//! yaw, level and at nominal height, with the head carrying the first 45° and
//! the body only the rest, and hold the shipped name table to numbering every
//! rung. Both files arrive through runfiles, and the environment variables name
//! them beside the `data` attribute that supplies them.

use std::path::PathBuf;

use reachy_host::LADDER;
use reachy_poses::format::Pose;

/// The committed document for pose `name`, loaded as the emitter loads it.
fn document(name: &str) -> Pose {
    let dir = std::env::var("POSE_DOCUMENTS").expect("the target names the pose documents");
    let path = PathBuf::from(dir).join(format!("{name}.textproto"));
    let text = std::fs::read_to_string(&path).unwrap_or_else(|why| panic!("{path:?}: {why}"));
    Pose::from_text(&text).unwrap_or_else(|why| panic!("{name} does not load: {why}"))
}

#[test]
fn every_rung_s_document_faces_its_world_yaw() {
    for (name, yaw_deg) in LADDER {
        let pose = document(name);
        let targets = pose.targets();
        let (roll, pitch, yaw) = targets.head_pose_body.rotation.euler_angles();
        assert!(
            (yaw + targets.body_yaw - yaw_deg.to_radians()).abs() < 1e-9,
            "{name}: head yaw {yaw}, body yaw {}, rung {yaw_deg}°",
            targets.body_yaw
        );
        assert!(roll.abs() < 1e-12, "{name}: roll {roll}");
        assert!(pitch.abs() < 1e-12, "{name}: pitch {pitch}");
        let offset = pose.relative().translation.vector.norm();
        assert!(offset < 1e-12, "{name}: {offset} m off nominal height");
    }
}

#[test]
fn the_head_carries_the_first_forty_five_degrees() {
    let fifteen = 15f64.to_radians();
    for (name, body_yaw) in [
        ("look_l60", fifteen),
        ("look_r60", -fifteen),
        ("look_l30", 0.0),
        ("look_r30", 0.0),
        ("neutral", 0.0),
    ] {
        let stated = document(name).targets().body_yaw;
        assert!(
            (stated - body_yaw).abs() < 1e-12,
            "{name}: body yaw {stated}"
        );
    }
}

#[test]
fn the_shipped_name_table_holds_every_rung() {
    let names = PathBuf::from(
        std::env::var("CLIP_NAMES").expect("the target names the shipped name table"),
    );
    let (_, poses) = reachy_host::check::name_tables(&names)
        .unwrap_or_else(|why| panic!("{}: {why}", names.display()));
    for (name, _) in LADDER {
        assert!(poses.resolve(name).is_some(), "{name} is not in the table");
    }
}
