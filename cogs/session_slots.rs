//! The one mapping between the session's own values and the schema fields its
//! slot holds them in.
//!
//! This module owns the report timeline — one row per report, the ring's two
//! cursors. A report has no second form: the row is the report, and what this
//! module owns is the ring around it, which row a cursor names and what a cursor
//! pair may say.
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
//! library itself writes ([`reachy_motion::verdict`]), so this module validates
//! that field once and hands the view down.

use brenn_reachy__cogs__msgs_clk_rs::SessionStateWire;
use brenn_reachy__motion__seq_clk_rs::{SeqFailureKind, SeqFailureSnapWire};
use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;
use clockwork_rs::Invalid;
use reachy_motion::SeqError;
use reachy_motion::verdict::{self, VerdictError};
use reachy_motion::vocab::known_nonzero;
use thiserror::Error;

/// Why a session slot's numbers describe nothing the session could have been
/// doing.
///
/// Every variant is a refusal rather than a repair, and each one is about a
/// field a wrong reading would make dangerous: a timeline row with no kind would
/// narrate nothing, and a wind-down restored with the wrong disposition would
/// leave a machine resting after a fault that asked for an operator.
///
/// Not `Eq`: a refusal about a solve failure carries the number that was not a
/// count, and a non-finite one is not equal to itself. Compared as written where
/// that matters, exactly as the verdicts carrying such a number are.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
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
    /// A verdict whose evidence does not suit the failure it is filed under, or
    /// which will not cross into the slot's numbers at all.
    #[error("the slot's verdict is not one: {0}")]
    Verdict(#[from] VerdictError),
    /// Timeline cursors that describe a ring this build does not have: more
    /// reports waiting than the ring holds. Refused rather than clamped.
    #[error("{unpublished} reports are waiting in a ring of {TIMELINE_LEN}")]
    TimelineCursors {
        /// How many the two cursors say are waiting.
        unpublished: u8,
    },
    /// A publication marked on a timeline with nothing waiting. The cursor is not
    /// advanced past the appends: a report the ring never held cannot be counted
    /// as published, and the next real report would be skipped if it were.
    #[error("nothing is waiting in the timeline to publish")]
    NothingUnpublished,
}

/// Leave the row holding no report at all.
///
/// A slot is reused memory, so a row that carries nothing has to say so: the
/// zero kind is the one [`oldest_unpublished`] refuses, and the numbers beside it
/// go with it so a later reader cannot mistake a cleared row for a report whose
/// kind was lost.
pub fn clear_timeline_entry(out: &mut TimelineEntryWire) {
    *out = TimelineEntryWire::new();
}

/// How many rows the report timeline holds.
pub const TIMELINE_LEN: u8 = 32;

// The cursors are counts of reports rather than row numbers, and they wrap with
// their own width. That is only sound while the width is a whole number of ring
// lengths: otherwise the row a count names would jump at the wrap, and a story
// would be read out of order exactly once every two hundred and fifty-six
// reports.
const _: () = assert!((u8::MAX as usize + 1).is_multiple_of(TIMELINE_LEN as usize));

/// The row a cursor names.
fn timeline_row(cursor: u8) -> usize {
    (cursor % TIMELINE_LEN) as usize
}

/// How many reports are appended and not yet published.
///
/// The difference between the two cursors, which is a count of reports and not a
/// distance between rows: both cursors are totals modulo their own width, so the
/// wrapping subtraction is the answer whether or not either has wrapped.
///
/// # Errors
///
/// [`SessionSlotError::TimelineCursors`] for a pair of cursors saying more is
/// waiting than the ring holds.
pub fn unpublished_reports(state: &SessionStateWire) -> Result<u8, SessionSlotError> {
    let unpublished = state
        .timeline_head()
        .wrapping_sub(state.timeline_published());
    if unpublished > TIMELINE_LEN {
        return Err(SessionSlotError::TimelineCursors { unpublished });
    }
    Ok(unpublished)
}

/// Append a report — `write` fills the claimed row — and say whether an
/// unpublished one was dropped to make room.
///
/// The row is handed to `write` cleared rather than handed back to the caller:
/// the cursors move with the append, so a row claimed and left unwritten would
/// be a report the ring counts and no reader can narrate, wedging the drain
/// until enough further appends wrapped past it. Passing the writing in is what
/// makes the claim and the write one act.
///
/// The ring is bounded and the append never blocks — a full ring loses its
/// oldest unpublished report, which a caller counts rather than reacts to.
///
/// # Errors
///
/// [`SessionSlotError::TimelineCursors`] for cursors describing a ring this build
/// does not have. On a refusal the slot is left exactly as it was — nothing is
/// appended over rows whose place in the story is unknown, and `write` is not
/// called.
pub fn push_report(
    state: &mut SessionStateWire,
    write: impl FnOnce(&mut TimelineEntryWire),
) -> Result<bool, SessionSlotError> {
    let dropped = unpublished_reports(state)? == TIMELINE_LEN;
    let head = state.timeline_head();
    state.set_timeline_head(head.wrapping_add(1));
    if dropped {
        // The row about to be written is the row the drain cursor was on, so the
        // cursor moves with it: a reader that stayed put would hand out the
        // newest report as the oldest.
        state.set_timeline_published(state.timeline_published().wrapping_add(1));
    }
    let row = &mut state.timeline_mut()[timeline_row(head)];
    clear_timeline_entry(row);
    write(row);
    Ok(dropped)
}

/// The row the oldest unpublished report stands in, or `None` where none is
/// waiting.
///
/// A peek rather than a take: the cursor advances only on [`mark_published`], so
/// a caller that could not publish leaves the report in place.
///
/// # Errors
///
/// [`SessionSlotError::TimelineCursors`] for cursors describing a ring this build
/// does not have, and [`SessionSlotError::NoSuchReportKind`] for a row the
/// cursors call a report and which narrates none — the zero kind an unwritten row
/// holds included, which is what makes an empty ring readable rather than a ring
/// of reports about nothing.
pub fn oldest_unpublished(
    state: &SessionStateWire,
) -> Result<Option<&TimelineEntryWire>, SessionSlotError> {
    if unpublished_reports(state)? == 0 {
        return Ok(None);
    }
    let row = &state.timeline()[timeline_row(state.timeline_published())];
    match known_nonzero(row.kind().to_known()) {
        None => Err(SessionSlotError::NoSuchReportKind(row.kind().0)),
        Some(_) => Ok(Some(row)),
    }
}

/// Record that the oldest unpublished report has gone out.
///
/// # Errors
///
/// [`SessionSlotError::TimelineCursors`] for cursors describing a ring this build
/// does not have, and [`SessionSlotError::NothingUnpublished`] where nothing was
/// waiting: a cursor advanced past the appends would skip the next real report.
pub fn mark_published(state: &mut SessionStateWire) -> Result<(), SessionSlotError> {
    if unpublished_reports(state)? == 0 {
        return Err(SessionSlotError::NothingUnpublished);
    }
    state.set_timeline_published(state.timeline_published().wrapping_add(1));
    Ok(())
}

/// Leave the timeline holding no story at all: every row cleared and both cursors
/// at the start.
///
/// The rows go with the cursors so a later reader cannot find a report in a ring
/// that says it is empty.
pub fn clear_timeline(state: &mut SessionStateWire) {
    for row in state.timeline_mut() {
        clear_timeline_entry(row);
    }
    state.set_timeline_head(0);
    state.set_timeline_published(0);
}

/// Write the verdict a stopped sequence left, or that the sequence is running.
///
/// The slot is cleared to the schema's declared initial state and then filled by
/// [`verdict::write`], so a field the verdict does not name carries nothing from
/// the verdict before it. A cleared slot is the no-failure zero, which is what
/// [`read_opt_seq_failure`] answers "no verdict" for.
///
/// # Errors
///
/// [`SessionSlotError::Verdict`] for a verdict that will not cross — a
/// non-finite register value, or a wait past what the slot's count reaches.
pub fn write_opt_seq_failure(
    out: &mut SeqFailureSnapWire,
    failure: Option<&SeqError>,
) -> Result<(), SessionSlotError> {
    let out = out.clear_valid();
    let Some(failure) = failure else {
        return Ok(());
    };
    Ok(verdict::write(out, failure)?)
}

/// The verdict those fields describe, or `None` where the slot holds none.
///
/// The one validation of this field, and the only place its numbers are narrowed
/// to the vocabulary. The zero kind is no verdict rather than a refusal: a
/// snapshot in a running phase carries no failure, and that is the common case,
/// not a corrupt slot. Whether a *phase* may carry none is
/// [`CommissionSequencer::resume`](reachy_motion::CommissionSequencer::resume)'s
/// question, asked once for every host of a sequencer.
///
/// # Errors
///
/// [`SessionSlotError::Invalid`] for bytes the schema does not declare, and
/// [`SessionSlotError::Verdict`] for fields that name no failure this build
/// raises or evidence that does not suit the failure it is filed under.
pub fn read_opt_seq_failure(
    slot: &SeqFailureSnapWire,
) -> Result<Option<SeqError>, SessionSlotError> {
    let slot = slot.validate()?;
    if slot.kind == SeqFailureKind::None {
        return Ok(None);
    }
    Ok(Some(verdict::read(slot)?))
}
