//! `reachy-poses` — named base postures: the document format, its validation,
//! and the library a pose id resolves through.
//!
//! A pose is **not** a one-frame clip. A clip is a masked *delta* layered over
//! whatever base is standing; a pose *is* the base — a whole configuration the
//! mover moves to over a duration, and the thing the disarm sequence judges
//! arrival at. So the two libraries are separate, a script step names one or
//! the other, and a pose document has no mask: every channel is required,
//! because a base that left a channel unsaid is not a place the machine can
//! stand.
//!
//! **Pose documents are protobuf text; clip documents are JSON.** The two
//! sibling asset formats differ, deliberately and not by accident of history: a
//! pose's schema is stated once, in `cogs/config.clk`, and read here against
//! the descriptor generated from it, the way the driver reads its own
//! parameters — so there is no serde mirror of the schema to keep in step.
//! Converting the clip documents is a change to the vendor import pipeline and
//! its committed report, which is work of its own.
//!
//! Pure and sans-I/O: documents arrive as strings the caller read, and nothing
//! here opens a file or reads a clock. The authoring/playback split that
//! `reachy-clips` carries applies identically — the document reader is host-side
//! only, and the running machine reaches a pose out of the configuration message
//! — and, as there, it is one build target all the same.
//! TODO(clips-authoring-split)
//!
//! **Nothing here is a safety gate.** A document whose pose leaves the envelope
//! is refused, which is a content gate: an asset nobody can stand at should
//! never become an asset. What actually protects the machine is the per-tick
//! envelope check and step bound in `reachy-motion`, applied to every commanded
//! target, with no bypass for a pose that loaded.

#![forbid(unsafe_code)]

pub mod config;
pub mod format;
pub mod library;
pub mod record;

/// The one pose name this system reserves: where the machine rests, what every
/// compiled schedule ends at, what the fault ladder commands, and what the
/// disarm sequence judges folded against.
///
/// Must equal `motion_proto::STOW_POSE`, which is authoritative — the wire
/// states the contract and takes no dependency. The two crates share none
/// either, so `cogs/edge_caps_test`, which links both, is the only enforcement
/// and holds this copy to that one.
pub const STOW_POSE: &str = "stow";

/// The pose every delta in this system is measured over: head square and level,
/// body square, antennas at their rest lean.
///
/// A second name the library cannot do without, for a different reason than
/// [`STOW_POSE`]'s: a clip document carries deltas over the base it was recorded
/// on, and the probe instruments step between this pose's antennas and the
/// stow's, so an emit whose documents hold no `neutral` is an emit that cannot
/// state what its clips are deltas of.
///
/// Not a wire word: nothing in `motion-proto` reserves it, and a script names
/// it the way it names any other pose. Not the only spelling in the tree: a
/// sender that names a pose spells its own and pins it against the sidecar,
/// and a fixture library states the names its rows are numbered under.
pub const NEUTRAL_POSE: &str = "neutral";

/// The base spelling that means *do not move the base*, and therefore the one
/// name no pose may take.
///
/// A script names a base either by a pose name or by this word, so a pose
/// carrying it would be an asset on the unit that no script could ever reach.
/// Refused at the document loader, which is every path a pose reaches a library
/// by.
///
/// Must equal `motion_proto::KEEP_BASE`, which is authoritative, and is joined
/// to it in `cogs/edge_caps_test` for the reason [`STOW_POSE`] gives.
pub const KEEP_BASE: &str = "keep";
