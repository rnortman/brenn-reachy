//! The intent edge's two loopback ports: which carries which schema, and which
//! end binds it.
//!
//! The same rules as the driver seam next door, for the same reasons. A
//! datagram carries the schema's own bytes and nothing else, so the port a
//! datagram arrived on *is* its type and two subjects may not share one; and
//! the numbers here are disjoint from that seam's (7401–7407) and from the host
//! composition's injection port (7408), because a workstation runs the lot on
//! one loopback.
//!
//! Everything binds `127.0.0.1`, which is the whole of the defence on this
//! seam: the control process's incoming socket fails the process on a datagram
//! of the wrong size, so the guard has to be that nothing off the machine can
//! reach it. That the sender is built from the same tree as the socket is the
//! other half — a skew between the two is what that rule punishes, and one
//! payload built and pushed whole is what designs it out.
//!
//! TODO(motord-seam-trust-boundary): a transport with an access boundary of its
//! own. Loopback binding restricts hosts, not local processes or users, and
//! these two ports inherit that exactly as the driver seam's seven do.

use std::net::Ipv4Addr;

/// The one address both sockets of this seam bind or send to.
pub const LOOPBACK: Ipv4Addr = Ipv4Addr::LOCALHOST;

/// Compiled scripts, host to control process. Bound there.
pub const SCRIPTS_IN_PORT: u16 = 7409;

/// The session's cumulative narration, control process to host. Bound here.
pub const REPORTS_OUT_PORT: u16 = 7410;

/// Every port of the intent edge, one per subject.
///
/// The one list, as the driver seam keeps its own: a subject added to this seam
/// is added in one place. Disjointness is proven over the union of both lists
/// rather than inside either — a number this seam shares with the driver's is
/// the collision that costs a datagram its type, and neither list can see the
/// other from here. `cogs/seam_ports_test.rs` is where the two meet.
pub const ALL: [u16; 2] = [SCRIPTS_IN_PORT, REPORTS_OUT_PORT];
