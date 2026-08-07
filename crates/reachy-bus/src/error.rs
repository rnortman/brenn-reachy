//! What a transaction can fail with.
//!
//! The distinctions here are the reason this layer exists. A timeout is the
//! absence of bytes and may be retried within a budget. A corrupt frame is
//! bytes that arrived and disagreed with themselves; retrying it would launder
//! a wire fault into an apparent success, so it never is. A servo that answers
//! with its error field set has answered — that is a diagnosis about the servo,
//! not about the bus, and it reaches the caller with the byte intact because
//! one of its values means two different things and only the caller knows
//! which.
//!
//! None of the three is a boolean, and none of them is an `io::Error`.
//!
//! A grouped read fails differently: it asks nine servos at once, so its result
//! is nine independent verdicts rather than one. [`SyncReadOutcome`] is that
//! result, and nothing in it can abort the rest — one silent servo leaves the
//! other eight readings intact.

use std::io;
use std::time::Duration;

use dxl_proto::{BROADCAST_ID, EncodeError, FrameError, StatusError};
use thiserror::Error;

use crate::bus::{MAX_SYNC_IDS, RawValue};

/// A unicast transaction's failure.
#[derive(Debug, Error)]
pub enum XactError {
    /// Nothing arrived from `id` before the deadline. The one failure the retry
    /// wrapper is allowed to repeat.
    #[error("servo {id} did not answer within {waited:?}")]
    Timeout {
        /// The servo addressed.
        id: u8,
        /// How long the exchange waited before giving up.
        waited: Duration,
    },

    /// Bytes arrived and the decoder rejected them. Never retried.
    #[error("servo {id}: {cause}")]
    Corrupt {
        /// The servo addressed. The corrupt bytes carry no trustworthy ID of
        /// their own, so this names who was asked, not who replied.
        id: u8,
        /// The decoder's verdict.
        cause: FrameError,
    },

    /// The servo replied with its error field set. Surfaced verbatim: the Data
    /// Range number is both an out-of-window goal refusal and the signature of
    /// a latched bus watchdog, and this layer cannot tell those apart.
    #[error("servo {id} answered with error field {:#04x} ({:?})", .error.0, .error.code())]
    ServoError {
        /// The servo that replied.
        id: u8,
        /// The error field, whole.
        error: StatusError,
    },

    /// A reply carried fewer parameter bytes than the request asked for.
    #[error("servo {id} returned {actual} parameter bytes, expected {expected}")]
    ShortReply {
        /// The servo that replied.
        id: u8,
        /// Parameter bytes the request asked for.
        expected: usize,
        /// Parameter bytes that arrived.
        actual: usize,
    },

    /// A reply carried more parameter bytes than the request asked for. A
    /// request's answer is exactly as wide as the request, so a wider one is a
    /// frame from some *other* exchange wearing this servo's ID — the tail of
    /// an abandoned read that landed after the line was cleared. Taking its
    /// head as the value would read position bytes as an error byte.
    #[error("servo {id} returned {actual} parameter bytes, expected {expected}")]
    LongReply {
        /// The servo that replied.
        id: u8,
        /// Parameter bytes the request asked for.
        expected: usize,
        /// Parameter bytes that arrived.
        actual: usize,
    },

    /// A verified write read back something other than what was written.
    #[error("servo {id} register {addr}: wrote {wrote}, read back {read_back}")]
    VerifyMismatch {
        /// The servo written to.
        id: u8,
        /// Control-table address.
        addr: u16,
        /// The value sent.
        wrote: RawValue,
        /// The value the read-back returned.
        read_back: RawValue,
    },

    /// A write to a non-volatile register. Refused in software: a servo
    /// silently ignores such a write while its torque is on, which is the
    /// worst available outcome.
    #[error("servo {id} register {addr} is non-volatile; writes to it are refused")]
    EepromRefused {
        /// The servo addressed.
        id: u8,
        /// Control-table address.
        addr: u16,
    },

    /// A non-volatile write to a servo that was holding torque, refused before
    /// anything went out. Torque is read rather than assumed: a servo ignores a
    /// non-volatile write while its torque is on and acknowledges it anyway.
    #[error("servo {id} is holding torque; register {addr} takes a write only once it is released")]
    TorqueHeld {
        /// The servo addressed.
        id: u8,
        /// Control-table address.
        addr: u16,
    },

    /// A value whose width disagrees with the register it addresses.
    #[error("servo {id} register {addr} is {expected} bytes wide, value is {actual}")]
    ValueWidth {
        /// The servo addressed.
        id: u8,
        /// Control-table address.
        addr: u16,
        /// The register's width.
        expected: usize,
        /// The value's width.
        actual: usize,
    },

    /// A register wider than a [`RawValue`] can carry. The control table holds
    /// nothing that wide today; the check is what keeps a future entry from
    /// silently losing its tail.
    #[error("servo {id} register {addr} is {len} bytes wide, over the {max}-byte limit")]
    RegisterTooWide {
        /// The servo addressed.
        id: u8,
        /// Control-table address.
        addr: u16,
        /// The register's width.
        len: usize,
        /// The widest value this layer carries.
        max: usize,
    },

    /// The request could not be encoded, so nothing went out.
    #[error("encoding a request for servo {id}")]
    Encode {
        /// The servo addressed.
        id: u8,
        /// The encoder's refusal.
        source: EncodeError,
    },

    /// A grouped request naming more servos than this bus carries.
    #[error("{count} servos in one grouped request, over the limit of {max}")]
    TooManyIds {
        /// Servos the request named.
        count: usize,
        /// Servos a grouped request may name.
        max: usize,
    },

    /// The port itself failed.
    #[error("port i/o while addressing servo {id}")]
    Io {
        /// The servo addressed.
        id: u8,
        /// The underlying failure.
        source: io::Error,
    },
}

impl XactError {
    /// True for the one failure a retry may repeat.
    ///
    /// A convenience for callers that report or count; the retry loop enforces
    /// the policy independently.
    #[must_use]
    pub fn is_retryable(&self) -> bool {
        matches!(self, Self::Timeout { .. })
    }

    /// The servo the failed transaction addressed.
    ///
    /// A grouped request is addressed to the broadcast ID rather than to any
    /// one servo, so that is what a failure of one reports.
    #[must_use]
    pub fn id(&self) -> u8 {
        match self {
            Self::Timeout { id, .. }
            | Self::Corrupt { id, .. }
            | Self::ServoError { id, .. }
            | Self::ShortReply { id, .. }
            | Self::LongReply { id, .. }
            | Self::VerifyMismatch { id, .. }
            | Self::EepromRefused { id, .. }
            | Self::TorqueHeld { id, .. }
            | Self::ValueWidth { id, .. }
            | Self::RegisterTooWide { id, .. }
            | Self::Encode { id, .. }
            | Self::Io { id, .. } => *id,
            Self::TooManyIds { .. } => BROADCAST_ID,
        }
    }
}

/// What one servo's slot of a grouped read came to.
///
/// Five outcomes, none of them collapsible into the others. Silence and a
/// refusal are different diagnoses; a reply that does not hold exactly the
/// register is neither, because the servo did answer and its frame did pass its
/// CRC. A corrupt frame is not here at all: corrupt bytes carry no ID anyone can
/// trust, so they cannot be attributed to a servo and are counted against the
/// read as a whole.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub enum IdOutcome {
    /// Nothing arrived from this servo before the deadline.
    #[default]
    Timeout,
    /// The register, as this servo reported it.
    Ok(RawValue),
    /// The servo answered with its error field set. Carried whole: one of its
    /// numbers means two different things and only the caller knows which.
    ServoError(StatusError),
    /// The servo answered with fewer parameter bytes than the register is wide.
    ShortReply {
        /// Bytes the register is wide.
        expected: usize,
        /// Bytes that arrived.
        actual: usize,
    },
    /// The servo answered with more parameter bytes than the register is wide,
    /// which no answer to this read is: it is a frame from a wider exchange
    /// carrying this servo's ID. Its head is not this register's value.
    LongReply {
        /// Bytes the register is wide.
        expected: usize,
        /// Bytes that arrived.
        actual: usize,
    },
}

impl IdOutcome {
    /// The value, if this servo produced one.
    #[must_use]
    pub fn value(&self) -> Option<RawValue> {
        match self {
            Self::Ok(value) => Some(*value),
            _ => None,
        }
    }
}

/// How a grouped read came out, servo by servo.
///
/// One bad responder never discards the others and never aborts the call: every
/// slot is filled independently, and the ones nothing arrived for age out as
/// [`IdOutcome::Timeout`]. The caller decides what a partial result is worth.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct SyncReadOutcome {
    ids: [u8; MAX_SYNC_IDS],
    outcomes: [IdOutcome; MAX_SYNC_IDS],
    count: usize,
    corrupt_frames: u32,
}

impl Default for SyncReadOutcome {
    fn default() -> Self {
        Self::new()
    }
}

impl SyncReadOutcome {
    /// An empty outcome, ready to be filled by a read.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            ids: [0; MAX_SYNC_IDS],
            outcomes: [IdOutcome::Timeout; MAX_SYNC_IDS],
            count: 0,
            corrupt_frames: 0,
        }
    }

    /// How many servos the read asked.
    #[must_use]
    pub fn len(&self) -> usize {
        self.count
    }

    /// True before any read has filled this.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    /// The servos asked, in the order they were asked in.
    #[must_use]
    pub fn ids(&self) -> &[u8] {
        &self.ids[..self.count]
    }

    /// The servo and outcome at `index` in the order the read asked them.
    #[must_use]
    pub fn at(&self, index: usize) -> Option<(u8, IdOutcome)> {
        if index >= self.count {
            return None;
        }
        Some((self.ids[index], self.outcomes[index]))
    }

    /// What `id` came to, or `None` if the read never asked it.
    #[must_use]
    pub fn get(&self, id: u8) -> Option<IdOutcome> {
        let index = self.ids().iter().position(|asked| *asked == id)?;
        Some(self.outcomes[index])
    }

    /// Frames that arrived damaged during the read. Unattributable by
    /// construction, so they are counted here rather than blamed on a servo.
    #[must_use]
    pub fn corrupt_frames(&self) -> u32 {
        self.corrupt_frames
    }

    /// True when every servo asked produced a value.
    #[must_use]
    pub fn all_ok(&self) -> bool {
        self.count > 0
            && self.outcomes[..self.count]
                .iter()
                .all(|outcome| matches!(outcome, IdOutcome::Ok(_)))
    }

    /// Starts a read of `ids`, discarding whatever the last one left.
    pub(crate) fn begin(&mut self, ids: &[u8]) {
        *self = Self::new();
        self.count = ids.len().min(MAX_SYNC_IDS);
        self.ids[..self.count].copy_from_slice(&ids[..self.count]);
    }

    /// Files `outcome` under `id`, reporting whether it filled a slot that was
    /// still waiting. A servo that was never asked, or that has already
    /// answered, fills nothing.
    pub(crate) fn record(&mut self, id: u8, outcome: IdOutcome) -> bool {
        let Some(index) = self.ids().iter().position(|asked| *asked == id) else {
            return false;
        };
        if !matches!(self.outcomes[index], IdOutcome::Timeout) {
            return false;
        }
        self.outcomes[index] = outcome;
        true
    }

    /// Counts a frame that arrived damaged.
    pub(crate) fn count_corrupt(&mut self) {
        self.corrupt_frames += 1;
    }
}

#[cfg(test)]
mod tests {
    use dxl_proto::FrameError;

    use super::*;

    /// A grouped read nobody has run reports no readings. The guard on the
    /// count is what holds this: `all()` over an empty slice is true, so
    /// without it an outcome whose read never happened — or whose `begin` was
    /// skipped on an error path — would report every servo as answering, and
    /// the tick would proceed on a joint vector of zeros.
    #[test]
    fn an_outcome_no_read_has_filled_reports_nothing() {
        let fresh = SyncReadOutcome::new();
        assert!(!fresh.all_ok(), "no servo answered, because none was asked");
        assert!(fresh.is_empty());
        assert_eq!(fresh.len(), 0);
        assert_eq!(fresh.ids(), &[] as &[u8]);
        assert_eq!(fresh.at(0), None);
        assert_eq!(fresh.get(11), None);
        assert_eq!(fresh.corrupt_frames(), 0);
        assert_eq!(SyncReadOutcome::default(), fresh);
    }

    /// The ID a failure names is the servo an operator goes and looks at, on a
    /// mechanism with six identical-looking legs. A grouped request is
    /// addressed to nobody in particular, and says so.
    #[test]
    fn every_failure_names_the_servo_it_was_addressed_to() {
        let value = RawValue::new(&[0]).expect("one byte");
        let failures = [
            XactError::Timeout {
                id: 11,
                waited: Duration::ZERO,
            },
            XactError::Corrupt {
                id: 12,
                cause: FrameError::BadCrc,
            },
            XactError::ServoError {
                id: 13,
                error: StatusError(0x04),
            },
            XactError::ShortReply {
                id: 14,
                expected: 4,
                actual: 2,
            },
            XactError::LongReply {
                id: 15,
                expected: 1,
                actual: 4,
            },
            XactError::VerifyMismatch {
                id: 16,
                addr: 64,
                wrote: value,
                read_back: value,
            },
            XactError::EepromRefused { id: 17, addr: 20 },
            XactError::TorqueHeld { id: 17, addr: 11 },
            XactError::ValueWidth {
                id: 18,
                addr: 64,
                expected: 1,
                actual: 2,
            },
            XactError::RegisterTooWide {
                id: 10,
                addr: 200,
                len: 7,
                max: RawValue::MAX_LEN,
            },
            XactError::Encode {
                id: 11,
                source: EncodeError::BroadcastNotAllowed,
            },
            XactError::Io {
                id: 12,
                source: io::Error::from(io::ErrorKind::BrokenPipe),
            },
        ];
        let expected = [11, 12, 13, 14, 15, 16, 17, 17, 18, 10, 11, 12];
        for (failure, id) in failures.iter().zip(expected) {
            assert_eq!(failure.id(), id, "{failure}");
        }

        let grouped = XactError::TooManyIds {
            count: 10,
            max: MAX_SYNC_IDS,
        };
        assert_eq!(
            grouped.id(),
            BROADCAST_ID,
            "a grouped request is addressed to nobody in particular"
        );
    }
}
