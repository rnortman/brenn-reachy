//! The session's story, as the edge follows it.
//!
//! The control process publishes its whole timeline every time it appends a
//! row: one cumulative message, oldest row first, plus a count of the rows that
//! fell off the front of its ring. So a reader that arrives late holds the
//! whole account from the first message it sees, and a reader that misses a
//! datagram misses nothing — the next one carries what the lost one did.
//!
//! What that costs is a diff. This module keeps the one number that makes the
//! diff possible: how many rows of the story have already been narrated. A
//! message's own total is `dropped + entries`, so the rows past that number are
//! the new ones, and the arithmetic says three different things:
//!
//! - the total advanced by no more than the message carries — the ordinary
//!   case, and the new rows are the tail of `entries`;
//! - the total advanced by more than the message carries — the session
//!   narrated faster than these datagrams arrived and its ring lost the middle.
//!   The rows that fell through are counted and said; nothing pretends to have
//!   them;
//! - the total went *backwards* — a story never shrinks, so the process that
//!   was telling it restarted. The follower resets and narrates the new story
//!   from its first row.
//!
//! Nothing here is an input to anything the machine does. The rows are
//! narration, and the phase the last of them names is an observation of what
//! the session last *said* — already stale by the time it is read, and never a
//! screen the intake consults.

use clockwork_rs::{SyncTime, blob_from_bytes};
use thiserror::Error;

use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, TimelineWire};

/// A datagram that is not a story.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
#[error("a story is {expected} bytes and the datagram is {bytes}")]
pub struct NotAStory {
    /// What arrived.
    pub bytes: usize,
    /// What a `Timeline` measures.
    pub expected: usize,
}

/// What one story datagram added to what was already narrated.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct Update {
    /// Whether the story went backwards, which says the process telling it
    /// restarted. The rows below are then the new story from its beginning.
    pub restarted: bool,
    /// Rows the session narrated that no datagram carried here — the ring
    /// overran between two messages. Counted rather than reconstructed.
    pub lost: u64,
    /// The rows to narrate, oldest first.
    pub rows: Vec<TimelineEntryWire>,
}

/// The follower: how much of the session's story has been narrated already.
///
/// One per stream. It holds a count and the last phase row it saw, and nothing
/// else: the rows themselves are rendered and let go, because the session's own
/// message is the durable copy and holding a second one here would be a second
/// account able to disagree with it.
#[derive(Clone, Debug, Default)]
pub struct Story {
    /// Rows of this story already narrated, counted from the session's own
    /// beginning — its `dropped` plus the rows it carried.
    narrated: u64,
    /// The last phase the session said it entered, and when it said so.
    /// Narration: possibly stale, never a screen.
    phase: Option<(SessionPhaseWire, SyncTime)>,
}

impl Story {
    /// A follower that has narrated nothing.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// How many rows of the current story have been narrated.
    #[must_use]
    pub fn narrated(&self) -> u64 {
        self.narrated
    }

    /// The last phase the session narrated entering, and when.
    ///
    /// An observation of what the machine last said about itself. The session
    /// is the only authority on its own phase, so this is for a reader, never
    /// for a decision.
    #[must_use]
    pub fn phase(&self) -> Option<(SessionPhaseWire, SyncTime)> {
        self.phase
    }

    /// Follow a datagram off the reports socket.
    ///
    /// # Errors
    ///
    /// [`NotAStory`] when the datagram is not a `Timeline`'s size. Refused
    /// rather than read: a blob of the wrong length is some other schema, and
    /// reading it as this one would narrate garbage in the vocabulary of a
    /// report.
    pub fn follow_bytes(&mut self, bytes: &[u8]) -> Result<Update, NotAStory> {
        let story: TimelineWire = blob_from_bytes(bytes).ok_or(NotAStory {
            bytes: bytes.len(),
            expected: size_of::<TimelineWire>(),
        })?;
        Ok(self.follow(&story))
    }

    /// Follow a decoded story.
    #[must_use]
    pub fn follow(&mut self, story: &TimelineWire) -> Update {
        let entries = story.entries();
        // The count is clamped to the array's capacity on read, so the total
        // below cannot exceed what the schema can hold however the sender's
        // bytes read.
        let carried = entries.len() as u64;
        let total = u64::from(story.dropped()).saturating_add(carried);

        let mut update = Update::default();
        if total < self.narrated {
            // A story never shrinks. The teller restarted, so the follower
            // starts over on the story it is telling now.
            //
            // Length is the only evidence a cumulative stream without an
            // identity carries, and it misses a restart whose new story has
            // already outgrown the old count by the time a datagram of it
            // arrives. TODO(story-restart-discriminator)
            update.restarted = true;
            self.narrated = 0;
            self.phase = None;
        }
        let fresh = total - self.narrated;
        if fresh > carried {
            update.lost = fresh - carried;
        }
        let take = fresh.min(carried) as usize;
        update.rows = entries.iter().skip(entries.len() - take).cloned().collect();
        for row in &update.rows {
            if row.kind() == ReportKindWire::PHASE_CHANGED {
                let entered = SessionPhaseWire(u8::try_from(row.a()).unwrap_or(u8::MAX));
                self.phase = Some((entered, row.time()));
            }
        }
        self.narrated = total;
        update
    }
}

#[cfg(test)]
mod tests {
    use clockwork_rs::{Blob as _, SyncTime, blob_as_bytes};

    use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
    use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
    use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, TimelineWire};

    use super::Story;

    /// One row, identified by its `a` so a diff that hands back the wrong
    /// window of the story is visible.
    fn row(n: u32) -> TimelineEntryWire {
        let mut entry = TimelineEntryWire::new();
        entry.set_time(SyncTime::from_nanos(i64::from(n) * 1_000_000));
        entry.set_kind(ReportKindWire::SCHEDULE_PUBLISHED);
        entry.set_a(n);
        entry
    }

    /// A story carrying `rows`, with `dropped` said to have fallen off its
    /// front.
    fn story(rows: &[u32], dropped: u32) -> TimelineWire {
        let mut message = TimelineWire::new();
        message.set_dropped(dropped);
        {
            let mut entries = message.entries_mut();
            for n in rows {
                *entries
                    .try_grow()
                    .expect("a story of no more rows than the message holds") = row(*n);
            }
        }
        message
    }

    /// What the rows of an update say they are.
    fn narrated(update: &super::Update) -> Vec<u32> {
        update.rows.iter().map(TimelineEntryWire::a).collect()
    }

    #[test]
    fn the_first_story_is_narrated_whole() {
        let mut follower = Story::new();
        let update = follower.follow(&story(&[1, 2, 3], 0));
        assert!(!update.restarted);
        assert_eq!(update.lost, 0);
        assert_eq!(narrated(&update), vec![1, 2, 3]);
        assert_eq!(follower.narrated(), 3);
    }

    #[test]
    fn a_story_that_grew_narrates_only_what_grew() {
        let mut follower = Story::new();
        let _ = follower.follow(&story(&[1, 2, 3], 0));
        let update = follower.follow(&story(&[1, 2, 3, 4, 5], 0));
        assert_eq!(narrated(&update), vec![4, 5]);
        assert_eq!(update.lost, 0);
    }

    #[test]
    fn a_republished_story_narrates_nothing_twice() {
        let mut follower = Story::new();
        let _ = follower.follow(&story(&[1, 2, 3], 0));
        let update = follower.follow(&story(&[1, 2, 3], 0));
        assert!(update.rows.is_empty());
        assert!(!update.restarted);
        assert_eq!(update.lost, 0);
    }

    /// The ring overran between two datagrams: the session's total advanced by
    /// six and the message carries four, so two rows exist that nothing here
    /// will ever hold.
    #[test]
    fn rows_that_fell_through_the_ring_are_counted_not_invented() {
        let mut follower = Story::new();
        let _ = follower.follow(&story(&[1, 2, 3], 0));
        let update = follower.follow(&story(&[6, 7, 8, 9], 5));
        assert_eq!(narrated(&update), vec![6, 7, 8, 9]);
        assert_eq!(update.lost, 2);
        assert_eq!(follower.narrated(), 9);
    }

    /// Rows fall off the front while the total advances by exactly what the
    /// message carries: nothing was missed, because the rows that dropped were
    /// already narrated.
    #[test]
    fn a_ring_dropping_rows_already_narrated_loses_nothing() {
        let mut follower = Story::new();
        let _ = follower.follow(&story(&[1, 2, 3, 4], 0));
        let update = follower.follow(&story(&[3, 4, 5], 2));
        assert_eq!(narrated(&update), vec![5]);
        assert_eq!(update.lost, 0);
    }

    #[test]
    fn a_shrunken_story_is_a_restart_and_is_narrated_from_its_beginning() {
        let mut follower = Story::new();
        let _ = follower.follow(&story(&[1, 2, 3, 4, 5], 0));
        let update = follower.follow(&story(&[1, 2], 0));
        assert!(update.restarted);
        assert_eq!(narrated(&update), vec![1, 2]);
        assert_eq!(update.lost, 0);
        assert_eq!(follower.narrated(), 2);
    }

    /// The restart clears the phase the old process left behind: the new one
    /// has said nothing yet, and a phase held across the gap would be an
    /// observation of a process that no longer exists.
    #[test]
    fn a_restart_forgets_the_phase_the_old_process_was_in() {
        let mut follower = Story::new();
        let mut said = TimelineEntryWire::new();
        said.set_kind(ReportKindWire::PHASE_CHANGED);
        said.set_a(u32::from(SessionPhaseWire::ACTIVE.0));
        said.set_time(SyncTime::from_nanos(7));
        let mut message = TimelineWire::new();
        {
            let mut entries = message.entries_mut();
            *entries.try_grow().expect("one row") = said;
        }
        let _ = follower.follow(&message);
        assert_eq!(
            follower.phase(),
            Some((SessionPhaseWire::ACTIVE, SyncTime::from_nanos(7)))
        );

        let _ = follower.follow(&story(&[], 0));
        assert_eq!(follower.phase(), None);
    }

    #[test]
    fn a_datagram_of_the_wrong_size_is_not_a_story() {
        let mut follower = Story::new();
        let message = story(&[1], 0);
        let bytes = blob_as_bytes(&message);
        assert!(follower.follow_bytes(bytes).is_ok());
        let short = follower.follow_bytes(&bytes[..bytes.len() - 1]);
        let error = short.expect_err("a short datagram is not a story");
        assert_eq!(error.expected, TimelineWire::SIZE);
        assert_eq!(error.bytes, TimelineWire::SIZE - 1);
        // The refusal costs the follower nothing: the next whole datagram is
        // read against the same mark.
        assert_eq!(follower.narrated(), 1);
    }
}
