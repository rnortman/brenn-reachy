//! `reachy-bus` — the one place in this workspace that touches hardware.
//!
//! Nine servos share one half-duplex serial bus. This crate owns the port and the
//! transaction semantics on top of it: ping, unicast read, verified unicast
//! write, the grouped read that gathers nine positions in one request, the
//! grouped write that streams nine goals, and the reboot that restarts a servo
//! — which drops the torque it was holding, so nothing here sends one on its
//! own initiative. Blocking and synchronous, on one
//! thread, by design — a request-response bus at a fixed baud rate is an owned
//! loop under every host substrate, and pretending otherwise buys nothing.
//!
//! It is also where joints meet the wire. The servo map turns a joint's index
//! into a servo ID, a register name into an address, and a count into the angle
//! the model means. Nothing above this crate learns what an address or a count
//! is, and nothing here shifts a reading: the offset between a crank's
//! mechanical zero and the model's is provisioned into the servo itself.
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
//! - **Replies are attributed by the ID in the packet.** Never by arrival order,
//!   so a late answer to a previous exchange is recognised as one, counted, and
//!   skipped rather than read as the answer to the question just asked.
//! - **A write is verified.** Unicast, status packet checked, register read back
//!   and compared. The grouped write is the exception: the protocol
//!   acknowledges nothing, so the caller must detect goals that did not
//!   take by other means.
//! - **A grouped read is nine verdicts, not one.** A silent servo, a refusing
//!   servo and a damaged frame each affect their own slot and no other, and
//!   none of them aborts the call.
//! - **Errors are never flattened to a boolean.** "Did not answer" and "answered
//!   with an error number" are different diagnoses and reach the caller as such.
//! - **The device is opened exclusively.** An advisory lock on the node, taken
//!   without waiting; a second opener is refused by name. One half-duplex line
//!   carries one speaker, and two hosts sharing it corrupt each other's replies
//!   rather than taking turns.
//!
//! Writes to the servos' non-volatile registers are refused outright. A servo
//! silently ignores such a write while its torque is on, and a write that is
//! ignored but verified-as-sent is the worst of the available outcomes — so the
//! operation is simply not offered rather than guarded by torque-state evidence
//! the caller would have to thread through.

#![forbid(unsafe_code)]

pub mod bus;
pub mod error;
pub mod map;
pub mod port;

pub use bus::{
    Bus, BusCounters, BusTiming, CYCLE_HOST_ALLOWANCE, ExchangeSpans, MAX_SYNC_IDS, PingInfo,
    RawValue, with_retry,
};
pub use error::{IdOutcome, SyncReadOutcome, XactError};
pub use map::{MapError, ServoMap, named_reg, reg_for, value_kind};
pub use port::{BusPort, DEFAULT_BAUD, OpenError, SerialBusPort};
