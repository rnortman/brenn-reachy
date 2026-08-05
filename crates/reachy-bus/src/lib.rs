//! `reachy-bus` — the one place in this workspace that touches hardware.
//!
//! Nine servos share one half-duplex serial bus. This crate owns the port and the
//! transaction semantics on top of it: ping, unicast read, verified unicast write,
//! per-ID synchronous read, and group goal writes. Blocking and synchronous, on
//! one thread, by design — a request-response bus at a fixed baud rate is an owned
//! loop under every host substrate, and pretending otherwise buys nothing.
//!
//! The port is reached through a narrow trait of our own — write, read with a
//! deadline, discard input — rather than the serial crate's full interface. Three
//! lines of surface is all the transactions need, and it makes the whole
//! transaction layer testable against a scripted fake port with no hardware and no
//! timing luck.
//!
//! The error taxonomy is the load-bearing part:
//!
//! - **A timeout and a corrupt frame are different things.** A timeout may be
//!   retried within a bounded budget. A corrupt frame is never retried: it fails
//!   its transaction immediately, because a retry that succeeds afterwards
//!   launders a wire problem into an apparent success.
//! - **A group read reports per ID.** One silent or corrupt responder does not
//!   discard the replies that did arrive and does not abort the call. Replies are
//!   matched by the ID in the packet, so an out-of-order or missing response
//!   cannot be attributed to the wrong servo.
//! - **A write is verified.** Unicast, status packet checked, register read back.
//!   Group writes are unacknowledged by the protocol and are used for goal
//!   streaming only, where a tracking monitor above is the compensating detection.
//! - **Errors are never flattened to a boolean.** "Did not answer" and "answered
//!   with an error bit" are different diagnoses and reach the caller as such.
//!
//! Writes to the servos' non-volatile registers are refused in software unless
//! torque is known to be off, because the hardware silently ignores them
//! otherwise and a write that is ignored but verified-as-sent is the worst of the
//! available outcomes.

#![forbid(unsafe_code)]

// TODO(bus-echo-policy): the receive loop has to decide what a well-formed
// non-status frame is. If this port reflects what the host writes, the reflected
// instruction frame arrives first on every exchange and the decoder reports it
// as a corrupt candidate — which, under the no-retry rule above, would fail
// every transaction. Either such frames are skipped when they match the bytes
// just written, or the port is established not to reflect and that is written
// down where a future reader will find it.
