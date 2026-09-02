//! The words the host's console is read by, in one place.
//!
//! A stable word per screen, defined beside the screens rather than restated at
//! every surface that reports one: a log whose spelling drifts stops joining
//! against the runs that came before it, silently. These are the host's own
//! lifecycle words — the four the edge stream carries about how a process
//! started and what it composed — and the prefix the binary refuses under on its
//! way out.
//!
//! They are here rather than beside each emitter because one of the emitters is
//! `src/main.rs`, which is a crate of its own and nothing else can import. The
//! reader that matters is out of this tree entirely: `//cogs:speech_run_report`
//! decides whether a supervised session came up by looking for these exact
//! strings, and a rename it did not hear about is an analyzer that keeps
//! building, keeps exiting green, and stops detecting the failure it exists for.

/// The voice host announcing itself: the first line of a run.
pub const STARTED: &str = "started";

/// The voice pipeline running: what makes a run the production pipeline.
pub const COMPOSED: &str = "composed";

/// The edge half running alone, no speech configuration having been named.
pub const VOICELESS: &str = "voiceless";

/// A speech configuration named and not found where it was named.
pub const AWAITING_SPEECH_CONFIG: &str = "awaiting_speech_config";

/// An alert that was narrated and did not reach the bus.
pub const UNPUBLISHED: &str = "unpublished";

/// An alert whose sentence the robot was asked to say and did not.
pub const UNSPOKEN: &str = "unspoken";

/// A body that never reached the gate: a sink's queue would not take it.
pub const UNOFFERED: &str = "unoffered";

/// An accepted script that never reached the session's port.
pub const UNSENT: &str = "unsent";

/// How the binary spells a startup it refused, on its way out.
///
/// On stderr and not on the JSONL stream, because a process refusing its own
/// configuration has no stream yet. A console ending in one of these is a host
/// that never ran rather than one that drained.
pub const REFUSAL_PREFIX: &str = "reachy-host: ";
