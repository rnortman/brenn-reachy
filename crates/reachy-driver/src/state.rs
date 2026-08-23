//! What a driver-state slot can hold that no driver ever wrote.
//!
//! Every machine's state is a Clockwork schema, and validating one says the
//! bytes are the shape they claim: the enumerations are declared values, the
//! nested transaction record is a transaction, the torque belief names servos
//! this bus has. What it cannot say is which plain numbers name a bus row, and
//! which combinations of a cursor and a report a run of the machine produces —
//! so a slot never written by a driver, or written by a version that disagreed
//! about a field's meaning, can hold a rotation cursor past the last servo or a
//! pass claiming a confirmation it did not read back. The aux slot and the
//! confirmation refuse those through this one type, so a host checking them has
//! one error to handle rather than one per machine: they answer the same shape
//! of question. A field typed as what it means needs no entry here, which is
//! why the belief has none.
//! The gate is not among them, and needs no entry: nothing about a queue of
//! setpoints is beyond what its schema already says.
//!
//! Nothing here panics on a state that fails validation. The process this runs
//! in is the one that de-torques the machine, and it does not get to crash over
//! a bad slot: every read of a cursor is clamped, and a host that wants to know
//! asks.

use crate::JOINT_COUNT;

/// A driver-cycle machine restored from a slot holding something impossible.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DriverStateError {
    /// The health rotation's cursor names a row past the end of the bus.
    HealthCursorOutOfRange {
        /// What the slot said.
        row: u8,
    },
    /// The confirmation pass's cursor names a row past the end of the bus.
    ///
    /// One past the last row is not this error: that is the pass having read
    /// every row clean, which is the state confirmation is reached in.
    ConfirmCursorOutOfRange {
        /// What the slot said.
        row: u8,
    },
    /// A confirmation machine that is not running, carrying the work of one
    /// that was: a pass part-way through, or a report already made.
    IdleConfirmWithProgress {
        /// How far the pass had got.
        cursor: u8,
    },
    /// A confirmation claiming it has said so while its cursor has not read
    /// every row back: progress no pass produces, and a shape that would keep
    /// the confirmation silent for the rest of the process.
    ConfirmedWithIncompletePass {
        /// How far the pass had got.
        cursor: u8,
    },
}

impl core::fmt::Display for DriverStateError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::HealthCursorOutOfRange { row } => write!(
                f,
                "health rotation cursor {row} is past the {JOINT_COUNT} bus rows"
            ),
            Self::ConfirmCursorOutOfRange { row } => write!(
                f,
                "confirmation cursor {row} is past the {JOINT_COUNT} bus rows"
            ),
            Self::IdleConfirmWithProgress { cursor } => write!(
                f,
                "a confirmation that is not running carries a pass at row {cursor}"
            ),
            Self::ConfirmedWithIncompletePass { cursor } => write!(
                f,
                "a confirmation says it confirmed with the pass at row {cursor} of {JOINT_COUNT}"
            ),
        }
    }
}

impl core::error::Error for DriverStateError {}
