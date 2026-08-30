//! `reachy-host` — the robot's voice host: what it is configured with, and its
//! half of the intent edge.
//!
//! One process on the robot's own computer holds the voice pipeline, the
//! robot's single Brenn-bus attachment, and the edge that turns intent into the
//! request the control process runs. It is on the robot because the wake word's
//! path to the motors must not leave the machine: a bus round trip to a server
//! that is not in this house is a head that hesitates after somebody speaks.
//!
//! What is here is the edge half — the configuration ([`params`]), the surface
//! its narration lands on ([`edge`]), and the queue both intent sources hand
//! bodies to ([`intents`]). The gate and the narration themselves are
//! `reachy-edge`'s, held by the same [`HostEdge`] the harness runs. The speech
//! pipeline this composes with is a library of the pod platform, and the wiring
//! that links it is the process's other half.
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

pub mod edge;
pub mod intents;
pub mod params;

pub use edge::Console;
pub use intents::{INTENT_BACKLOG, Intents, NotOffered, Waiting, queue};
pub use params::{HostSettings, ParamsError, ParamsErrorKind, load, parse};
