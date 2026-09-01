//! `reachy-host` — the robot's voice host: what it is configured with, and its
//! half of the intent edge.
//!
//! One process on the robot's own computer holds the voice pipeline, the
//! robot's single Brenn-bus attachment, and the edge that turns intent into the
//! request the control process runs. It is on the robot because the wake word's
//! path to the motors must not leave the machine: a bus round trip to a server
//! that is not in this house is a head that hesitates after somebody speaks.
//!
//! What is here is both halves. The edge half is the configuration
//! ([`params`]), the surface its narration lands on ([`edge`]), and the queue
//! both intent sources hand bodies to ([`intents`]); the gate and the narration
//! themselves are `reachy-edge`'s, held by the same [`HostEdge`] the harness
//! runs. The voice half is the pod platform's own server, composed here
//! ([`voice`]) with its two motion seams filled by this process's gate
//! ([`sinks`]) instead of by a bus that is not on this machine. What both
//! halves would load, decided on a workstation before anything is pushed, is
//! [`check`].
//!
//! [`HostEdge`]: reachy_edge::HostEdge
//!
//! The process boundary between this and the control process is architectural,
//! not incidental. The composition's incoming socket fails its process on a
//! datagram of the wrong size, and that process's death is a minimum-risk-
//! condition event: network-facing code — TLS, reconnects, JSON off the wire —
//! does not share a failure domain with the thing whose crash de-torques the
//! machine. Everything that can be malformed dies, is refused, or is narrated
//! *here*.
//!
//! Nothing here decides anything about motion. The edge screens and compiles,
//! the session decides what it will run, and the mover checks every commanded
//! value; a host that has stopped is a machine whose running schedule concludes
//! at its own horizon and rests.

#![forbid(unsafe_code)]

pub mod check;
pub mod edge;
pub mod intents;
pub mod params;
pub mod sinks;
pub mod voice;
pub mod words;

pub use check::{Conclusion, conclusion_line, inspect, settled};
pub use edge::Console;
pub use intents::{INTENT_BACKLOG, Intents, NotOffered, Waiting, queue};
pub use params::{HostSettings, ParamsError, ParamsErrorKind, load, parse};
pub use sinks::{BusIntents, Lines, ScripterIntents, Stdout};
pub use voice::{Voice, absent_line, composed_line, silent_line};
pub use words::{
    AWAITING_SPEECH_CONFIG, COMPOSED, REFUSAL_PREFIX, STARTED, UNPUBLISHED, VOICELESS,
};
