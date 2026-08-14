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
//! **Nothing here is a safety gate.** Validation refuses assets that are
//! malformed or that could not be played over the base they were recorded
//! against, which is a content-sanity gate. What actually protects the machine
//! is the per-tick envelope check and step bound in `reachy-motion`, applied to
//! the composed target, every tick, with no bypass for playback.

#![forbid(unsafe_code)]

pub mod compose;
pub mod files;
pub mod format;
pub mod library;
pub mod player;
pub mod sequence;
pub mod speed;
pub mod vendor;

pub use compose::{ChannelWeights, OverlaySample, compose, interpolate_pose, lerp, scale_delta};
pub use files::{DOCUMENT_EXT, document_paths, documents};
pub use format::{
    BlendEnd, Channel, ChannelMask, Clip, ClipDoc, ClipError, ClipNote, DEFAULT_BLEND_MS,
    DeltaFrame, FORMAT_VERSION, FrameDoc, MAX_MOTION_NAME_LEN, MAX_SPEED, MIN_SPEED, NameError,
    PerChannel, QUAT_NORM_TOL, SPEED_CACHE_TOL, document_name, validate_name,
};
pub use library::{
    AssetNote, AssetSkip, Library, LibraryBuilder, LoadError, MAX_SEQUENCE_DEPTH, Motion,
    ResolveError, Segment,
};
pub use player::ClipPlayer;
pub use sequence::{Entry, EntryDoc, Sequence, SequenceDoc, SequenceError};
pub use speed::{
    ClipLimits, Derivation, DeriveError, FrameMetrics, RAMP_SAMPLES, STEP_MARGIN, derive, seam_step,
};
pub use vendor::{
    CONSTANT_TOL, Import, ImportError, ImportOptions, ROTATION_TOL, VendorFrame, VendorMove,
    convert,
};
