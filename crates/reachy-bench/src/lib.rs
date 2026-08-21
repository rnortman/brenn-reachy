//! `reachy-bench` — the hardware test tool: a read-only self-test registry and
//! the bare-bus commands.
//!
//! No control loop and no coordinated motion. What this crate does to a machine
//! is read registers, write the antennas' operating mode, restart the servos,
//! and sweep torque off. It is also the only crate here that reads a
//! configuration file, logs, or prints.
//!
//! Two halves:
//!
//! - **The read-only registry.** Port open, presence sweep naming any absent
//!   servo, model-number grouping, the provisioned-register sweep, rail voltage,
//!   hardware health, the legs' homing offsets against the vendor constant, and
//!   the resting pose with its clearance margins. No torque, no motion, nothing
//!   written to a servo. One line per case, and a case that did not run counts as
//!   a failure rather than as silence.
//! - **The bare-bus commands.** `provision`, `reboot` and `off`: a bus, a
//!   roster, and no sequencer between them. Nothing here arms anything, and
//!   `off` reaches the minimum risk condition's de-torqued half from wherever
//!   the machine stands.
//!
//! The registry is how this project brings up hardware: write a case that
//! asserts the behaviour we expect, let it fail, and read the discovery out of
//! the failure. A confirmed value is then baked into the case, which stays as a
//! permanent regression guard. An unexpected value gets human review before any
//! case is changed to accept it.
//!
//! The binary is a thin entry point over this library, so everything the bench
//! decides — configuration and the registry's verdicts — is reachable from tests
//! that need no port and no machine.
//!
//! `commands.rs`, `pump.rs`, `trace.rs` and `trace/metrics.rs` sit beside these
//! files but are not in the build: they are the bench's retired motion layer,
//! kept on disk as the executable record of how this machine was driven.
//! TODO(bench-motion-delete)

#![forbid(unsafe_code)]

pub mod bare;
pub mod config;
pub mod selftest;
#[cfg(test)]
mod testutil;
