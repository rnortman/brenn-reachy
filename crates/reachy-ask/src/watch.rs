//! Watching the session's narration for the one row worth asking on.
//!
//! The session accepts a script only while it is resting, and it says when it
//! got there: the phase change out of `starting` into `resting` is its own
//! narration of having finished commissioning. That row is the trigger, and it
//! is read off the story rather than waited out on a clock — a timer would be a
//! guess about how long a bus survey takes on whatever machine this is.
//!
//! Only the first such row counts. A later entry into `resting` is a session
//! that *ended*, and answering it with a second gesture would be a machine that
//! wakes itself in a loop. The run asks once.
//!
//! Sans-I/O: rows in, an answer out. The socket, the clock and the deadline are
//! the runner's.

use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;

/// Whether `row` is the session saying it finished commissioning.
///
/// The transition is named on both sides — into `resting`, out of `starting` —
/// because entry into `resting` on its own is also how a session ends, and the
/// two mean opposite things to something waiting to ask for motion.
#[must_use]
pub fn is_commissioned(row: &TimelineEntryWire) -> bool {
    row.kind() == ReportKindWire::PHASE_CHANGED
        && row.a() == u32::from(SessionPhaseWire::RESTING.0)
        && row.b() == u32::from(SessionPhaseWire::STARTING.0)
}

/// The trigger, latched: has the run asked yet.
#[derive(Clone, Copy, Debug, Default)]
pub struct Watch {
    asked: bool,
}

impl Watch {
    /// A watch that has not asked.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Whether the gesture has already gone out.
    #[must_use]
    pub fn asked(&self) -> bool {
        self.asked
    }

    /// Whether `rows` is where this run asks, latching so that it answers true
    /// at most once.
    ///
    /// The latch is set here rather than by the caller after a successful send:
    /// a send that failed is not a reason to ask again, for the same reason a
    /// refusal is not — nothing here retries, and a second datagram would be a
    /// second engagement nobody asked for.
    pub fn should_ask<'a>(
        &mut self,
        rows: impl IntoIterator<Item = &'a TimelineEntryWire>,
    ) -> bool {
        if self.asked {
            return false;
        }
        if rows.into_iter().any(is_commissioned) {
            self.asked = true;
            return true;
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
    use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
    use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;

    use super::{Watch, is_commissioned};

    /// One narration row: a kind and its two numbers.
    fn row(kind: ReportKindWire, a: u32, b: u32) -> TimelineEntryWire {
        let mut entry = TimelineEntryWire::new();
        entry.set_kind(kind);
        entry.set_a(a);
        entry.set_b(b);
        entry
    }

    /// The session having finished commissioning.
    fn commissioned() -> TimelineEntryWire {
        row(
            ReportKindWire::PHASE_CHANGED,
            u32::from(SessionPhaseWire::RESTING.0),
            u32::from(SessionPhaseWire::STARTING.0),
        )
    }

    /// The session having ended an engagement: into `resting` from somewhere
    /// else entirely.
    fn session_ended() -> TimelineEntryWire {
        row(
            ReportKindWire::PHASE_CHANGED,
            u32::from(SessionPhaseWire::RESTING.0),
            u32::from(SessionPhaseWire::STOPPING.0),
        )
    }

    #[test]
    fn the_commissioning_row_is_the_one_out_of_starting() {
        assert!(is_commissioned(&commissioned()));
        assert!(
            !is_commissioned(&session_ended()),
            "a session that ended is also in `resting`, and it is not an invitation to ask again",
        );
    }

    #[test]
    fn a_row_of_another_kind_is_not_the_trigger() {
        assert!(!is_commissioned(&row(
            ReportKindWire::SCRIPT_ACCEPTED,
            u32::from(SessionPhaseWire::RESTING.0),
            u32::from(SessionPhaseWire::STARTING.0),
        )));
    }

    #[test]
    fn the_watch_asks_on_the_first_commissioning_and_never_again() {
        let mut watch = Watch::new();
        assert!(!watch.should_ask(&[row(ReportKindWire::SCRIPT_ACCEPTED, 0, 0)]));
        assert!(!watch.asked());
        assert!(watch.should_ask(&[row(ReportKindWire::SCRIPT_ACCEPTED, 0, 0), commissioned()]));
        assert!(watch.asked());
        assert!(
            !watch.should_ask(&[commissioned()]),
            "the row repeats on every datagram, because every datagram carries the whole story",
        );
    }

    #[test]
    fn an_empty_update_asks_nothing() {
        let mut watch = Watch::new();
        assert!(!watch.should_ask(&[]));
        assert!(!watch.asked());
    }
}
