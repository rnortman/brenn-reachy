//! `reachy-motord` — the process that holds the servo bus.
//!
//! One OS process, not a cog and not in any composition: it owns the serial
//! port exclusively, runs its own phase-locked schedule, and talks to the
//! control process over loopback UDP. Why it is a process and not a cog is a
//! standing decision of the motion stack's design rather than a preference —
//! nothing sanctioned in a generated cog body can hold a port handle, a
//! multi-millisecond blocking transaction in an execute body spends half the
//! online runner's worker pool, and the framework's spacing between executions
//! is start-to-start rather than the grid a bus cycle has to sit on.
//!
//! What it decides, it does not decide here: the arbitration over goals, the
//! dead-man that de-torques on silence, the torque belief and the auxiliary
//! slot are [`reachy_driver`], hosted exactly as the simulated driver hosts
//! them. This crate is the parts a simulation does not have — a clock, two
//! sockets, a configuration file and a bus.
//!
//! Seven pieces, in the order a cycle meets them:
//!
//! - [`params`] — the configuration, read once at startup from protobuf text
//!   into the message its schema declares.
//! - [`ports`] — the six loopback ports the seam is laid out on, and which end
//!   binds which.
//! - [`grid`] — the schedule, as arithmetic: where the next wake is, and what
//!   happens to a cycle that ran long. No clock and no sleeping, so the rules
//!   are testable without waiting for them.
//! - [`inbound`] — the reading half of the seam: two bound sockets, a datagram
//!   decoded by the port it arrived on, and a queue the loop thread drains.
//!   Every refusal is counted; nothing on this path panics on bad input,
//!   because the sender of a bad datagram is not the machine's fault to die of.
//!
//! - [`tick`] — one bus cycle: the grouped read of the nine present positions,
//!   the gate's write, the one out-of-band transaction, and the reports the
//!   cycle publishes. The bus work itself, and the only place in this process
//!   that decides what reaches a servo.
//! - [`aux`] — the out-of-band transaction against the wire: a host's request,
//!   the torque-off confirmation's read-back, or the health rotation's read of
//!   one servo's status registers.
//!
//! - [`loop_ctl`] — the loop that runs the tick on the grid: the sleep, the
//!   drain, the publish, and the two conditions a cycle cannot see for itself
//!   — a driver nobody is talking to, and an inbound port that stopped being
//!   read. Both are answered by letting the machine go.
//!
//! `src/main.rs` is the rest: the configuration's path, the exclusive open, and
//! the line of counts the process prints as it runs.
//!
//! Nothing here clamps a commanded value and nothing retries an operation with
//! perturbed inputs. A transaction that did not happen is a transaction that
//! did not happen, reported as such.

#![forbid(unsafe_code)]

pub mod aux;
pub mod grid;
pub mod inbound;
pub mod loop_ctl;
pub mod params;
pub mod ports;
pub mod tick;
