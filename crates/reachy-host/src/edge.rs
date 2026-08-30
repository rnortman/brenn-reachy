//! Where the host's lines and alerts go.
//!
//! The running edge itself — the one gate, the one story follower, the one
//! alert latch — is `reachy-edge`'s [`HostEdge`], because the harness that
//! drives a motion run holds exactly the same thing. What is left here is the
//! part that is this process's own: the surface it writes on.
//!
//! An alert is narration and nothing else. That is the whole behaviour for a
//! host with no bus attachment configured, and it is also all a host with one
//! does today: the attachment lives inside the composed server, and the seam
//! that would hand a publish out of it does not exist yet.
//! TODO(host-alert-publish)
//!
//! Nothing is lost by the narration itself: the alert table's whole job is to
//! pick out what an operator should be interrupted for, and a deployment with
//! nothing to interrupt them through still wants the picking recorded.
//!
//! [`HostEdge`]: reachy_edge::HostEdge

use reachy_edge::{Alert, Surface, alert_line, now};

/// The surface the host runs on.
///
/// Lines to stdout, and alerts onto the same stream as one more line.
///
/// This stream is the whole of the operator surface. It says what happened and
/// it interrupts for what matters, and it answers no question about what the
/// machine is doing at the moment somebody asks.
/// TODO(host-status-egress)
#[derive(Clone, Copy, Debug, Default)]
pub struct Console;

impl Surface for Console {
    fn say(&mut self, line: String) {
        println!("{line}");
    }

    fn alert(&mut self, alert: &Alert) {
        // The alert's own instant: the table raised it while a line was being
        // written, and the clock read here is the same clock that stamped it.
        println!("{}", alert_line(alert, now()));
    }
}
