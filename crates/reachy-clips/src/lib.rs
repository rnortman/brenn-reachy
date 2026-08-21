//! `reachy-clips` — recorded motions as masked deltas, and how they layer.
//!
//! A clip is a sampled trajectory asset: a uniform-rate track of per-channel
//! **deltas** plus the **mask** naming which of the three commandable channels
//! — head pose, body yaw, antennas — it drives. It stores what a motion *does*,
//! not where the robot *is*, which is the whole difference from the vendor
//! model this replaces: a nod is a pitch excursion that can ride whatever the
//! robot is already doing, and an antennas-only wiggle says nothing at all
//! about the head.
//!
//! That makes playback a composition rather than a takeover. Something else
//! produces the base target each tick — today a posture timeline, later a
//! tracker — and every playing clip contributes a masked, weighted delta on top
//! of it. Channels no clip masks pass through untouched.
//!
//! Pure and sans-I/O like the crates below it: time arrives as a parameter,
//! assets arrive as strings the caller read, and nothing here reads a clock.
//! The exceptions are both at the crate's edge and both touch files: the
//! importer binary, an offline host-side tool, and [`files`], which is the one
//! place the rule for which files in a directory *are* assets lives — a rule
//! the daemon, the bench and the importer all have to agree on.
//!
//! The authoring half — [`format`], [`library`], [`vendor`], [`files`] and the
//! importer binary — is host-side only: playback reads clips out of the
//! configuration message, so nothing the running machine reaches enters it. It
//! is one build target with the playback half all the same, which is why this
//! header has to say so instead of the build.
//! TODO(clips-authoring-split)
//!
//! **Nothing here is a safety gate.** Validation refuses assets that are
//! malformed or that could not be played over the base they were recorded
//! against, which is a content-sanity gate. What actually protects the machine
//! is the per-tick envelope check and step bound in `reachy-motion`, applied to
//! the composed target, every tick, with no bypass for playback.
//!
//! Every type is imported from the module that declares it. There is no
//! crate-root re-export of the whole surface: two paths to one type is two
//! spellings of every import for no reader's benefit.

#![forbid(unsafe_code)]

pub mod compose;
pub mod config;
pub mod files;
pub mod format;
pub mod library;
pub mod player;
pub mod sequence;
pub mod speed;
pub mod vendor;
