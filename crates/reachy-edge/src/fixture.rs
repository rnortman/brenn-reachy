//! The library this crate's own cases resolve names through.
//!
//! One table, built here rather than per module, because nearly every case in
//! the crate has to name a pose and the number it resolves to is what the rows
//! carry. Numbered the way an emitted library numbers: path-sorted names, the
//! stow away from zero, so a row carrying the schema's zero instead of a
//! resolved index is visible in a diff.
//!
//! Deliberately not a constructor on [`PoseTable`]: a table of names a caller
//! did not read out of a sidecar is a second opinion about which emit a process
//! is holding, and the one place that is harmless is a test.

use motion_proto::STOW_POSE;

use crate::names::{PoseEntry, PoseTable};

/// The pose these cases raise to.
pub const NEUTRAL_POSE: &str = "neutral";

/// The id the fixture library gives [`NEUTRAL_POSE`].
pub const NEUTRAL_ID: u16 = 0;

/// A third pose, so that a case can name one that is neither the raise nor the
/// fold.
pub const PEEK_POSE: &str = "peek";

/// The id the fixture library gives [`PEEK_POSE`].
pub const PEEK_ID: u16 = 1;

/// The id it gives the stow.
pub const STOW_ID: u16 = 2;

/// How long the fixture library takes to fold, milliseconds: the room every
/// timeline here is measured against.
pub const STOW_MS: u32 = 3000;

/// How long it takes to raise, milliseconds.
pub const NEUTRAL_MS: u32 = 800;

/// The three poses, numbered and paced.
#[must_use]
pub fn poses() -> PoseTable {
    PoseTable::of([
        (
            NEUTRAL_POSE.to_owned(),
            PoseEntry {
                pose_id: NEUTRAL_ID,
                duration_ms: NEUTRAL_MS,
            },
        ),
        (
            PEEK_POSE.to_owned(),
            PoseEntry {
                pose_id: PEEK_ID,
                duration_ms: NEUTRAL_MS,
            },
        ),
        (
            STOW_POSE.to_owned(),
            PoseEntry {
                pose_id: STOW_ID,
                duration_ms: STOW_MS,
            },
        ),
    ])
    .expect("a library holding the stow")
}
