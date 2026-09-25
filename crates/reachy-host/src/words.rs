//! The words the host's console is read by, in one place.
//!
//! A stable word per screen, defined beside the screens rather than restated at
//! every surface that reports one: a log whose spelling drifts stops joining
//! against the runs that came before it, silently. These are the host's own
//! lifecycle words — the four the edge stream carries about how a process
//! started and what it composed — and the prefix the binary refuses under on its
//! way out. The idle loop's words live here too, the ones it narrates its seams,
//! its yielding to speech and its backoff under, so that a run analyzer joins on
//! one spelling.
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

/// The idle loop opened a resting machine with a clip.
pub const IDLE_OPENED: &str = "idle_opened";

/// The idle loop replaced its standing clip with the next one.
pub const IDLE_REPLACED: &str = "idle_replaced";

/// Speech took the head and the idle loop stepped back.
pub const IDLE_SUSPENDED: &str = "idle_suspended";

/// Speech was done with the head and the idle loop took it back mid-session.
pub const IDLE_RESUMED: &str = "idle_resumed";

/// A session ended on a fault and the idle loop waits before opening again.
pub const IDLE_BACKOFF: &str = "idle_backoff";

/// The machine parked and the idle loop sends nothing more this process run.
pub const IDLE_PARKED: &str = "idle_parked";

/// The idle loop's script was refused and the loop halted until a phase change.
/// Its `reason` is `session`, `edge`, `unbuildable` or `unsent`: the session
/// refused it, the edge gate refused it, the loop failed to build its own
/// script, or the script could not be sent to the control process.
pub const IDLE_REFUSED: &str = "idle_refused";

/// A stale stow from a sender that no longer owns the head, not offered.
pub const IDLE_DROPPED_STOW: &str = "idle_dropped_stow";

/// The idle loop has heard no story from the control process within
/// `NO_STORY_GRACE_MS` of its first poll. Said once per process run, on the
/// poll that offers the loop's one opening in the dark; the session's answer
/// to it carries the story.
pub const IDLE_NO_STORY: &str = "idle_no_story";

/// The loop's lines whose `script_id` names a script the idle loop itself built
/// and offered. This is the set a reader attributes the session's script rows to
/// the loop by. A kind belongs here only if the id it carries is the loop's own
/// script's; an id on a line about another sender's body does not qualify.
pub const IDLE_SCRIPT_KINDS: [&str; 4] = [IDLE_OPENED, IDLE_REPLACED, IDLE_RESUMED, IDLE_REFUSED];

/// How the binary spells a startup it refused, on its way out.
///
/// On stderr and not on the JSONL stream, because a process refusing its own
/// configuration has no stream yet. A console ending in one of these is a host
/// that never ran rather than one that drained.
pub const REFUSAL_PREFIX: &str = "reachy-host: ";
