//! `reachy-bench` — the bring-up host: a self-test registry and the supervised
//! motion commands.
//!
//! Everything else in this workspace is a library that owns no loop. This crate
//! is the loop: it opens the port, commissions the machine, polls it, takes
//! hold of it, then runs a fixed-rate cycle of read present positions, call the
//! tick, write the goals that changed. It is also the only crate here that
//! reads a configuration file, logs, or prints.
//!
//! Two halves, and the order between them is the whole point:
//!
//! - **The read-only registry.** Port open, presence sweep naming any absent
//!   servo, model-number grouping, the provisioned-register sweep, rail voltage,
//!   hardware health, the legs' homing offsets against the vendor constant, and
//!   the resting pose with its clearance margins. No torque, no motion, nothing
//!   written to a servo. One line per case, and a case that did not run counts as
//!   a failure rather than as silence.
//! - **The supervised commands.** Arm, raise, hold, stow, release, plus antenna
//!   and base moves. Each re-establishes the machine for itself and needs a
//!   crank datum a human has written into the configuration; nothing consults a
//!   registry pass, because a record is evidence about a past moment and the
//!   commissioning ceremony asks the machine in front of it.
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

#![forbid(unsafe_code)]

pub mod commands;
pub mod config;
pub mod pump;
pub mod selftest;
#[cfg(test)]
mod testutil;
