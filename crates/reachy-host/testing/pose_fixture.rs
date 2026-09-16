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

use reachy_edge::{PoseEntry, PoseTable};

/// The pose these cases raise to, and the name they resolve through the table.
pub const NEUTRAL_POSE: &str = "neutral";

/// A two-pose library, the stow away from zero so a row carrying the schema's
/// zero instead of a resolved index is visible.
///
/// # Panics
///
/// If the table refuses these rows, which is a fixture that no longer states a
/// library.
#[must_use]
pub fn poses() -> PoseTable {
    PoseTable::of([
        (
            NEUTRAL_POSE.to_owned(),
            PoseEntry {
                pose_id: 0,
                duration_ms: 800,
            },
        ),
        (
            motion_proto::STOW_POSE.to_owned(),
            PoseEntry {
                pose_id: 2,
                duration_ms: 3000,
            },
        ),
    ])
    .expect("a library holding the stow")
}
