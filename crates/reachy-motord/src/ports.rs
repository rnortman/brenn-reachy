//! The loopback seam: which port carries which schema, and which end binds it.
//!
//! Seven ports, one per direction per subject, because a datagram on this seam
//! carries no header — the payload is the schema's own bytes and nothing else,
//! which is what the socket layer on the control side reads and writes. With no
//! discriminator in the bytes, the port a datagram arrived on *is* the type, so
//! two subjects may not share one.
//!
//! Everything binds `127.0.0.1`. That is the whole of the foreign-datagram
//! defence on this seam: the control side's sockets fail the process on a
//! datagram of the wrong size rather than counting it, so the guard has to be
//! that nothing off the machine can reach them. This driver's own two ports
//! refuse and count instead ([`crate::inbound`]).
//!
//! What loopback binding does *not* defend against, stated where the seam is
//! declared: it restricts hosts, not local processes or users. Any process on
//! this board, under any user, can send a well-formed session command that arms
//! the machine and well-formed setpoints that move it — and those setpoints
//! reach the servos without passing the envelope check, which runs in the mover
//! upstream of the goal port. The driver's own refusals are narrower by design
//! (an angle no servo count represents, and nothing else), so this seam is the
//! one command path on the machine whose only guard against a violating pose is
//! that no untrusted code runs locally.
//!
//! TODO(motord-seam-trust-boundary): a transport with an access boundary of its
//! own, or a driver-side refusal of the same travel windows the mover enforces.

use std::net::Ipv4Addr;

/// The one address every socket of this seam binds or sends to.
///
/// Loopback, not a configured interface: both ends are processes on the same
/// board by construction — one holds the serial port the other's cogs command
/// through — and a seam that could be pointed off the machine would be a seam
/// that can be commanded from off the machine.
pub const LOOPBACK: Ipv4Addr = Ipv4Addr::LOCALHOST;

/// Goal setpoints, control process to driver. Bound here.
pub const GOAL_PORT: u16 = 7401;

/// Pose samples, driver to control process. Bound there.
pub const POSE_PORT: u16 = 7402;

/// Driver events, driver to control process. Bound there.
pub const EVENT_PORT: u16 = 7403;

/// Auxiliary transaction outcomes, driver to control process. Bound there.
pub const AUX_OUT_PORT: u16 = 7404;

/// Health reports, driver to control process. Bound there.
pub const HEALTH_PORT: u16 = 7405;

/// Session commands, control process to driver. Bound here.
pub const SESSION_PORT: u16 = 7406;

/// The driver's cumulative status record, driver to control process. Bound
/// there.
pub const STATUS_PORT: u16 = 7407;

/// Every port of the seam, one per subject.
///
/// The one list: the disjointness below is over this, so a subject added to the
/// seam is added in one place and the guard covers it without being remembered.
pub const ALL: [u16; 7] = [
    GOAL_PORT,
    POSE_PORT,
    EVENT_PORT,
    AUX_OUT_PORT,
    HEALTH_PORT,
    SESSION_PORT,
    STATUS_PORT,
];

const _: () = {
    // No port carries two subjects: with no header on the seam, a shared port
    // would be a datagram nobody can type.
    let mut i = 0;
    while i < ALL.len() {
        let mut j = i + 1;
        while j < ALL.len() {
            assert!(ALL[i] != ALL[j]);
            j += 1;
        }
        i += 1;
    }
};
