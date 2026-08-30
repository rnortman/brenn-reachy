//! `reachy-edge` — the intent edge: a motion script from outside, screened and
//! compiled into the request the session understands.
//!
//! One side of this crate speaks the intent vocabulary a speech interaction
//! publishes — `motion-proto`'s [`MotionScript`], JSON, offsets in
//! milliseconds. The other speaks the control system's own `Script`: a
//! `script_id`, an arrival instant, sixteen step slots and four overlay
//! windows. Between them sit the screens that decide whether a body becomes a
//! request at all.
//!
//! [`MotionScript`]: motion_proto::MotionScript
//!
//! Sans-I/O, and deliberately so. Bytes and an instant come in; a message to
//! send, or a typed refusal to narrate, comes out. It binds no socket and holds
//! no bus client, and nothing here reads a clock to screen with — the arrival
//! instant is always the caller's — because the two things it screens, a body
//! off the wire and a body handed to it in-process, must meet exactly one gate,
//! and a gate with a socket in it is a gate that can only be tested one way.
//! ([`run::now`] is the one wall-clock read the callers share, offered so that
//! two processes outside the composition cannot stamp an arrival two ways.)
//!
//! What the edge refuses, it refuses whole. Nothing here truncates a timeline
//! to fit, repairs a step to make it lawful, or retries anything: a script that
//! does not compile is a script the sender got wrong, and the sender's next
//! refresh — not this crate — is what recovers. Nor does it touch a pose
//! number. Postures and library indices are the whole of its vocabulary; every
//! commanded value is still the mover's to check, and nothing here is a path
//! around that.
//!
//! Three screens and a compile, in order ([`Edge::accept`]):
//!
//! 1. the size cap, applied to the bytes before anything parses them;
//! 2. the decode, and the addressee — a script for another pod is not this
//!    machine's to run;
//! 3. the sequence gate, which is where a redelivery stops. The session at rest
//!    deliberately does not defend against a stale delivery, on the grounds
//!    that the edge which received it is where that belongs. This is that edge.
//! 4. the compile ([`compile`]), which is where a timeline becomes a schedule's
//!    worth of steps or becomes a refusal.
//!
//! [`run`] is the state a process holds around all of that — one gate, one
//! story follower, one alert latch — and the trait its lines and alerts leave
//! through. It lives here rather than in a binary because every process that
//! stands outside the composition does the same thing with the edge, and a
//! second copy of that loop would prove a rehearsal of the edge rather than the
//! edge itself.
//!
//! The other direction is narration ([`story`], [`narrate`], [`alerts`]). The
//! session publishes its whole timeline whenever it appends a row; the edge
//! follows that story, renders what is new as lines, and keeps the small
//! latched table of what an operator should be interrupted for. None of it
//! feeds back into the intake: the timeline is an observation of what the
//! machine last said, and every screen above is phase-blind by design.

#![forbid(unsafe_code)]

pub mod alerts;
pub mod compile;
pub mod config;
pub mod intake;
pub mod names;
pub mod narrate;
pub mod ports;
pub mod run;
pub mod story;

pub use alerts::{Alert, Alerts, STALE_ALERT_RUN, Severity};
pub use compile::{CompileError, compile};
pub use config::{BODY_CAP_BYTES, ConfigError, EdgeConfig, MIN_BODY_CAP_BYTES, STOW_DURATION_MS};
pub use intake::{Accepted, Edge, Refusal};
pub use names::{MAX_MOTIONS, MotionEntry, MotionTable, SidecarError};
pub use narrate::{lost_line, refusal_line, restart_line, timeline_line};
pub use ports::{LOOPBACK, REPORTS_OUT_PORT, SCRIPTS_IN_PORT};
pub use run::{DATAGRAM_CAP, HostEdge, POLL, Surface, alert_line, now, severity_word};
pub use story::{NotAStory, Story, Update};
