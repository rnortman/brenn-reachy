//! The one mapping between the session's own values and the schema fields its
//! slot holds them in.
//!
//! This module owns the report timeline — one row per report, the ring's head
//! and what it has dropped. A report has no second form: the row is the report,
//! and what this module owns is the ring around it, which row a count names and
//! how much of the story the slot's two numbers say is written.
//!
//! The four sequencers and the wind-down keep their state in the schema itself,
//! so there is nothing to map there either: a host validates the slot once and
//! hands the view to `start`/`begin`/`resume`, and which of a state's field
//! pairings are reachable is that `resume`'s question, asked where the
//! sequence's own bounds live.
//!
//! Nothing here holds state or allocates, and none of it looks at a clock. Every
//! read is total and refuses rather than repairs: the case this crossing exists
//! for is a slot holding bytes nothing wrote.
//!
//! A sequencer's verdict is not crossed here either: the schema is the form the
//! library itself writes, and the sequencer that owns the snapshot writes and
//! reads it in place.

use brenn_reachy__cogs__session_clk_rs::SessionStateWire;
use brenn_reachy__motion__reports_clk_rs::ReportKind;
use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;
use clockwork_rs::Invalid;
use reachy_motion::vocab::known_nonzero;
use thiserror::Error;

/// Why a session slot's numbers describe nothing the session could have been
/// doing.
///
/// Every variant is a refusal rather than a repair, and each one is about a
/// field a wrong reading would make dangerous: a timeline row with no kind would
/// narrate nothing, and a wind-down restored with the wrong disposition would
/// leave a machine resting after a fault that asked for an operator.
#[derive(Clone, Copy, Debug, Error, Eq, PartialEq)]
pub enum SessionSlotError {
    /// A report kind this build does not know, zero included — which is what an
    /// unwritten timeline row holds.
    #[error("{0} names no report this build can narrate")]
    NoSuchReportKind(u8),
    /// Bytes in a slot that the schema does not declare — an enumeration field
    /// holding a number this build does not name, or a container counting past
    /// its capacity. The one refusal a validated crossing has, raised at the
    /// boundary rather than at every field that would have narrowed one.
    #[error("the slot does not hold what its schema declares: {0}")]
    Invalid(#[from] Invalid),
    /// A timeline saying more was appended than the ring ever held, with nothing
    /// recorded as dropped. Refused rather than clamped.
    #[error("{appended} reports were appended to a ring of {TIMELINE_LEN} and none dropped")]
    TimelineCount {
        /// How many the head says were appended.
        appended: u8,
    },
}

/// Leave the row holding no report at all.
///
/// A slot is reused memory, so a row that carries nothing has to say so: the
/// zero kind is the one [`report_row`] refuses, and the numbers beside it
/// go with it so a later reader cannot mistake a cleared row for a report whose
/// kind was lost.
pub fn clear_timeline_entry(out: &mut TimelineEntryWire) {
    *out = TimelineEntryWire::new();
}

/// How many rows the report timeline holds.
pub const TIMELINE_LEN: u8 = 64;

// The head is a count of reports rather than a row number, and it wraps with its
// own width. That is only sound while the width is a whole number of ring
// lengths: otherwise the row a count names would jump at the wrap, and a story
// would be read out of order exactly once every two hundred and fifty-six
// reports.
const _: () = assert!((u8::MAX as usize + 1).is_multiple_of(TIMELINE_LEN as usize));

/// The row a cursor names.
fn timeline_row(cursor: u8) -> usize {
    (cursor % TIMELINE_LEN) as usize
}

/// How many rows the story holds.
///
/// The ring is full exactly while something has been dropped off its front, so
/// the two numbers the slot carries say between them how much of it is written:
/// nothing dropped means the head is the count, and anything dropped means every
/// row is a row of the story.
///
/// # Errors
///
/// [`SessionSlotError::TimelineCount`] for a head past the ring's length with
/// nothing dropped, which is a slot describing a ring this build has not got.
pub fn held_reports(state: &SessionStateWire) -> Result<u8, SessionSlotError> {
    if state.timeline_dropped() > 0 {
        return Ok(TIMELINE_LEN);
    }
    let head = state.timeline_head();
    if head > TIMELINE_LEN {
        return Err(SessionSlotError::TimelineCount { appended: head });
    }
    Ok(head)
}

/// Append a report -- `write` fills the claimed row -- and say whether the
/// oldest row of the story was dropped to make room.
///
/// The row is handed to `write` cleared rather than handed back to the caller:
/// the head moves with the append, so a row claimed and left unwritten would be
/// a row the story counts and no reader can narrate. Passing the writing in is
/// what makes the claim and the write one act.
///
/// The ring is bounded and the append never blocks -- a full ring loses its
/// oldest row and counts it, which is the one thing a reader of the story cannot
/// work out from the rows it was handed.
///
/// # Errors
///
/// [`SessionSlotError::TimelineCount`] for a slot describing a ring this build
/// does not have. On a refusal the slot is left exactly as it was -- nothing is
/// appended over rows whose place in the story is unknown, and `write` is not
/// called.
pub fn push_report(
    state: &mut SessionStateWire,
    write: impl FnOnce(&mut TimelineEntryWire),
) -> Result<bool, SessionSlotError> {
    let dropped = held_reports(state)? == TIMELINE_LEN;
    let head = state.timeline_head();
    state.set_timeline_head(head.wrapping_add(1));
    if dropped {
        // The row about to be written is the oldest row of the story, so the
        // count of what the story has lost moves with it.
        state.set_timeline_dropped(state.timeline_dropped().saturating_add(1));
    }
    let row = &mut state.timeline_mut()[timeline_row(head)];
    clear_timeline_entry(row);
    write(row);
    Ok(dropped)
}

/// The `nth` row of the story, oldest first, and the kind it narrates -- or
/// `None` past the end of it.
///
/// The oldest row is row zero until the ring has wrapped, and the row the head
/// is about to be written to after that: a full ring's oldest row is the one the
/// next append will take.
///
/// # Errors
///
/// [`SessionSlotError::TimelineCount`] for a slot describing a ring this build
/// does not have, and [`SessionSlotError::NoSuchReportKind`] for a row the story
/// holds that narrates nothing -- the zero kind an unwritten row carries
/// included, which is what makes a damaged ring refuse rather than read as a
/// story about nothing.
pub fn report_row(
    state: &SessionStateWire,
    nth: u8,
) -> Result<Option<(&TimelineEntryWire, ReportKind)>, SessionSlotError> {
    let held = held_reports(state)?;
    if nth >= held {
        return Ok(None);
    }
    let oldest = if held == TIMELINE_LEN {
        state.timeline_head()
    } else {
        0
    };
    let row = &state.timeline()[timeline_row(oldest.wrapping_add(nth))];
    match known_nonzero(row.kind().to_known()) {
        None => Err(SessionSlotError::NoSuchReportKind(row.kind().0)),
        Some(kind) => Ok(Some((row, kind))),
    }
}

/// Leave the timeline holding no story at all: every row cleared, the head at
/// the start and nothing recorded as dropped.
///
/// The rows go with the head so a later reader cannot find a report in a ring
/// that says it is empty. The dropped count goes too: it is what says whether
/// the ring has wrapped, and a cleared ring has not.
pub fn clear_timeline(state: &mut SessionStateWire) {
    for row in state.timeline_mut() {
        clear_timeline_entry(row);
    }
    state.set_timeline_head(0);
    state.set_timeline_dropped(0);
}
