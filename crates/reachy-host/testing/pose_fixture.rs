//! The pose library both of this package's test crates resolve a raise through.
//!
//! The library's own cases and the binary's are separate crates, and both hand
//! a `HostEdge` a table of poses. One statement of it, here, so the two poses
//! and the ids they are numbered under are spelled once -- a copy inside either
//! crate is unreachable from the other, and the two can drift into disagreeing
//! about what a resolved id means without anything saying so.
//!
//! Stated here rather than read out of the committed sidecar: a fixture that
//! names its own two poses cannot drift from a library it never reads.
//!
//! Test support only.

#![forbid(unsafe_code)]

use reachy_edge::PoseTable;

/// The pose these cases raise to, and the name they resolve through the table.
pub const NEUTRAL_POSE: &str = "neutral";

/// The name table a deployment lays beside the two libraries, as this fixture's
/// library would have been emitted into one.
///
/// The document and the table below are the same two poses because the table is
/// read out of this text: a case that writes the sidecar a preflight reads and a
/// case that hands an edge a table are then holding one library, which is what
/// the deployment guarantees and what a second literal here would let drift.
/// The motions array is empty — nothing in this package plays one.
pub const SIDECAR: &str = r#"{
  "motions": [],
  "poses": [
    {"pose_id": 0, "name": "neutral", "duration_ms": 800},
    {"pose_id": 2, "name": "stow", "duration_ms": 3000}
  ]
}
"#;

/// A two-pose library, the stow away from zero so a row carrying the schema's
/// zero instead of a resolved index is visible.
///
/// The stow row's spelling is checked here, not merely resembled: a table
/// holding no pose named `motion_proto::STOW_POSE` is `SidecarError::NoStow`,
/// so a re-spelled reserved name fails this call rather than leaving every case
/// green against a library that no longer holds the pose the product commands.
///
/// # Panics
///
/// If [`SIDECAR`] is not a document this build resolves poses through, which is
/// a fixture that no longer states a library or no longer names the stow.
#[must_use]
pub fn poses() -> PoseTable {
    let (_, poses) = reachy_edge::parse(SIDECAR).expect("a library holding the stow");
    poses
}
