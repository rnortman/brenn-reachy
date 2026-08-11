//! The session's fault timeline: what went wrong, and what was done about it.
//!
//! One reporting channel for the whole stack. A fault is a condition of the
//! machine, a maneuver is what answers it, and the two together are a story
//! with an order: the head was grabbed, the stow started, a servo dropped out
//! mid-way, the stow carried on without it, everything ended limp. Nothing
//! reconstructs that from an error code, and nothing should have to parse it
//! back out of a rendered line — so it is appended here as it happens, typed,
//! and read back as data.
//!
//! Two ways to read it, both from the start. **Pull**: the session hands out
//! [`FaultTimeline::entries`] while it runs and the whole record when it ends.
//! **Push**: one subscriber ([`FaultTimeline::subscribe`]) receives every entry
//! as it appends, which is how a daemon turns a fault into an alert without
//! polling. An operator line is a *rendering* of an entry and never the channel
//! itself.
//!
//! Append-only, and never at poll rate: the tick appends a fault the once, on
//! the period it raises it, so a servo whose error bits stay latched for the
//! rest of the session adds nothing after the entry that took it out of
//! service.

use core::fmt;
use std::sync::mpsc::{Receiver, Sender, channel};
use std::time::Duration;

use crate::joints::JointSet;
use crate::tick::Fault;

/// What a response actually does to the machine.
///
/// The doctrine's minimum-risk maneuvers, one slug each. A response is a
/// maneuver plus the state it leaves behind ([`crate::tick::Response`]); this
/// is the maneuver half — the part an operator watches happen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Maneuver {
    /// Abandon the move and stow every commanded joint under control, checks
    /// live.
    SlowStow,
    /// Torque off the servo that dropped out, then stow on what still
    /// commands. A further servo dropping out expands it rather than ending
    /// it.
    MaskedSlowStow,
    /// Torque off the antenna pair and stop commanding it. The head is
    /// untouched and the move carries on.
    AntennaTorqueOff,
    /// Immediate best-effort torque-off of all nine.
    ImmediateAllTorqueOff,
}

impl Maneuver {
    /// The maneuver's slug — the name it is reported under everywhere.
    #[must_use]
    pub fn slug(self) -> &'static str {
        match self {
            Self::SlowStow => "slow_stow",
            Self::MaskedSlowStow => "masked_slow_stow",
            Self::AntennaTorqueOff => "antenna_torque_off",
            Self::ImmediateAllTorqueOff => "immediate_all_torque_off",
        }
    }
}

impl fmt::Display for Maneuver {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.slug())
    }
}

/// How far a maneuver got.
///
/// A maneuver starts once and ends once; in between it may expand any number of
/// times, which is what the escalation ladder does with servos that drop out
/// while a stow is running.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// The maneuver began.
    Started,
    /// These joints were torqued off and dropped from the maneuver, which
    /// carries on without them.
    Expanded(JointSet),
    /// The maneuver did not reach its end, and the immediate torque-off took
    /// over.
    FellThrough,
    /// The maneuver ran to its end and nothing confirmed it landed — a servo
    /// that never acknowledged its torque-off, a wire that stopped answering
    /// part way through the walk, or a stow whose mask grew to cover every
    /// joint that carries the head, so the head came down under nothing.
    ///
    /// Distinct from [`Self::Completed`], because the difference is whether
    /// the machine is known to have reached what the maneuver was for: a
    /// minimum risk condition nobody confirmed must never be recorded as one
    /// that was reached. Distinct from [`Self::FellThrough`] too — this
    /// maneuver reached its own end rather than being given up on.
    Unconfirmed,
    /// The maneuver ran to its end.
    Completed,
}

impl Outcome {
    /// Whether this outcome closes the maneuver it belongs to.
    ///
    /// What tells a fault raised *inside* a running maneuver from one that
    /// starts a new answer: the ladder never begins a second wind-down, so a
    /// maneuver still open is the one that absorbs whatever happens next.
    #[must_use]
    pub fn ends(self) -> bool {
        match self {
            Self::Started | Self::Expanded(_) => false,
            Self::FellThrough | Self::Unconfirmed | Self::Completed => true,
        }
    }
}

impl fmt::Display for Outcome {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Started => f.write_str("started"),
            Self::Expanded(joints) => write!(f, "expanded, released {joints}"),
            Self::FellThrough => f.write_str("fell through"),
            Self::Unconfirmed => f.write_str("ran unconfirmed"),
            Self::Completed => f.write_str("completed"),
        }
    }
}

/// One thing that happened, and when.
///
/// Typed all the way down: the fault carries its own detail, the response
/// carries the joints an expansion released. A reader keys on the values, not
/// on the sentence [`fmt::Display`] makes of them.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Entry {
    /// A condition of the machine, raised.
    Fault {
        /// What was found.
        fault: Fault,
        /// When, on the session's own clock.
        at: Duration,
    },
    /// A maneuver, at one of the points its progress is worth recording.
    Response {
        /// Which maneuver.
        maneuver: Maneuver,
        /// How far it got.
        outcome: Outcome,
        /// When, on the session's own clock.
        at: Duration,
    },
}

impl Entry {
    /// When this happened, on the session's own clock.
    #[must_use]
    pub fn at(&self) -> Duration {
        match self {
            Self::Fault { at, .. } | Self::Response { at, .. } => *at,
        }
    }
}

impl fmt::Display for Entry {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Fault { fault, .. } => write!(f, "{}: {fault}", fault.slug()),
            Self::Response {
                maneuver, outcome, ..
            } => write!(f, "{maneuver} {outcome}"),
        }
    }
}

/// Everything a session has raised and everything it did about it, in order.
///
/// Owned by the session — a [`crate::tick::MotionState`] carries one from the
/// moment arming produces it — because that is the scope the story has: a fault
/// answered by a park is the end of this session, and the next engagement
/// starts a record of its own.
#[derive(Debug, Default)]
pub struct FaultTimeline {
    entries: Vec<Entry>,
    /// The maneuver currently running, if one is.
    ///
    /// Kept as state rather than read back out of `entries`, because the
    /// escalation ladder turns on it: deriving it from the record would make
    /// every retention decision about the record — a cap, a clear, a filter —
    /// a silent change to whether the machine starts a second wind-down.
    open: Option<Maneuver>,
    /// Where each entry is also sent as it appends, when anyone asked.
    ///
    /// One, not many: the consumer is whatever owns the session — a daemon
    /// turning entries into alerts — and a second subscriber would be a second
    /// owner. A receiver that has been dropped is not an error; the record
    /// stands on its own and the send is best-effort.
    subscriber: Option<Sender<Entry>>,
}

/// A clone is the record, and never the subscription.
///
/// Written out rather than derived: a derived clone would hand a second record
/// the one sender, and both would push into a receiver that has no way to tell
/// their entries apart — two stories interleaved as one, with nothing anywhere
/// reporting an error. The copy carries the entries and listens to nobody.
impl Clone for FaultTimeline {
    fn clone(&self) -> Self {
        Self {
            entries: self.entries.clone(),
            open: self.open,
            subscriber: None,
        }
    }
}

impl FaultTimeline {
    /// An empty record.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Receive every entry appended from here on.
    ///
    /// Replaces any previous subscriber, and does not replay what is already
    /// recorded: a subscriber taken mid-session reads the history through
    /// [`Self::entries`] and the future through the channel.
    pub fn subscribe(&mut self) -> Receiver<Entry> {
        let (sender, receiver) = channel();
        self.subscriber = Some(sender);
        receiver
    }

    /// Everything recorded so far, oldest first.
    #[must_use]
    pub fn entries(&self) -> &[Entry] {
        &self.entries
    }

    /// Whether anything has gone wrong at all.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }

    /// The most recent fault, which is the one an ending is reported under.
    #[must_use]
    pub fn last_fault(&self) -> Option<Fault> {
        self.entries.iter().rev().find_map(|entry| match entry {
            Entry::Fault { fault, .. } => Some(*fault),
            Entry::Response { .. } => None,
        })
    }

    /// The maneuver that is running, if one is.
    ///
    /// The ladder's rule as a query: a fault raised while this answers `Some`
    /// expands that maneuver instead of starting another one.
    #[must_use]
    pub fn open_maneuver(&self) -> Option<Maneuver> {
        self.open
    }

    /// Record a fault, raised at `at`.
    pub fn fault(&mut self, fault: Fault, at: Duration) {
        self.append(Entry::Fault { fault, at });
    }

    /// Record how far a maneuver got, at `at`.
    pub fn response(&mut self, maneuver: Maneuver, outcome: Outcome, at: Duration) {
        // The maneuver this entry is about is the one running from here on,
        // unless the entry is what ends it. An outcome that ends one leaves
        // nothing open: the ladder does not resume a maneuver it closed.
        self.open = (!outcome.ends()).then_some(maneuver);
        self.append(Entry::Response {
            maneuver,
            outcome,
            at,
        });
    }

    /// Keep the entry and tell whoever is listening.
    fn append(&mut self, entry: Entry) {
        self.entries.push(entry);
        if let Some(subscriber) = &self.subscriber {
            // Nothing here may fail: this is the reporting path of a machine
            // in trouble.
            let _ = subscriber.send(entry);
        }
    }
}

/// All entries in order, arrow-separated — the whole story of a session that
/// had one.
///
/// What an operator reads at the end of an incident, and what a report attaches
/// verbatim.
impl fmt::Display for FaultTimeline {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (index, entry) in self.entries.iter().enumerate() {
            if index > 0 {
                f.write_str(" → ")?;
            }
            write!(f, "{entry}")?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::joints::{JointGroup, JointId};

    fn at(millis: u64) -> Duration {
        Duration::from_millis(millis)
    }

    /// The record is what happened, in the order it happened.
    #[test]
    fn a_timeline_keeps_every_entry_in_order() {
        let mut timeline = FaultTimeline::new();
        assert!(timeline.is_empty());
        assert_eq!(timeline.last_fault(), None);
        assert_eq!(timeline.open_maneuver(), None);

        let fault = Fault::HeadObstructed {
            joint: JointId::Leg(1),
            error: 0.4,
        };
        timeline.fault(fault, at(10));
        timeline.response(Maneuver::SlowStow, Outcome::Started, at(20));
        timeline.response(Maneuver::SlowStow, Outcome::Completed, at(900));

        assert_eq!(
            timeline.entries(),
            [
                Entry::Fault { fault, at: at(10) },
                Entry::Response {
                    maneuver: Maneuver::SlowStow,
                    outcome: Outcome::Started,
                    at: at(20),
                },
                Entry::Response {
                    maneuver: Maneuver::SlowStow,
                    outcome: Outcome::Completed,
                    at: at(900),
                },
            ]
        );
        assert_eq!(timeline.last_fault(), Some(fault));
        assert_eq!(timeline.entries()[1].at(), at(20));
    }

    /// A maneuver is open from its start until whatever ends it, and an
    /// expansion is not an ending.
    #[test]
    fn the_open_maneuver_is_the_one_still_running() {
        let mut timeline = FaultTimeline::new();
        timeline.response(Maneuver::MaskedSlowStow, Outcome::Started, at(1));
        assert_eq!(timeline.open_maneuver(), Some(Maneuver::MaskedSlowStow));

        // A second servo going does not start a second wind-down.
        timeline.fault(
            Fault::HeadServoFault {
                joint: JointId::Leg(3),
                id: 13,
                bits: 0x20,
            },
            at(2),
        );
        assert_eq!(timeline.open_maneuver(), Some(Maneuver::MaskedSlowStow));
        let mut released = JointSet::EMPTY;
        released.insert(JointId::Leg(3));
        timeline.response(Maneuver::MaskedSlowStow, Outcome::Expanded(released), at(3));
        assert_eq!(timeline.open_maneuver(), Some(Maneuver::MaskedSlowStow));

        timeline.response(Maneuver::MaskedSlowStow, Outcome::FellThrough, at(4));
        assert_eq!(timeline.open_maneuver(), None);
        timeline.response(Maneuver::ImmediateAllTorqueOff, Outcome::Completed, at(5));
        assert_eq!(timeline.open_maneuver(), None);
    }

    /// A subscriber gets the same values the record keeps, as they append.
    #[test]
    fn a_subscriber_receives_the_entries_the_record_keeps() {
        let mut timeline = FaultTimeline::new();
        let fault = Fault::AntennaObstructed {
            joint: JointId::AntennaLeft,
            error: 0.5,
        };
        // Nothing before the subscription is replayed; everything after it
        // arrives.
        timeline.fault(fault, at(1));
        let entries = timeline.subscribe();
        timeline.response(
            Maneuver::AntennaTorqueOff,
            Outcome::Expanded(JointGroup::Antennas.joints()),
            at(2),
        );
        timeline.response(Maneuver::AntennaTorqueOff, Outcome::Completed, at(3));

        let pushed: Vec<Entry> = entries.try_iter().collect();
        assert_eq!(pushed, timeline.entries()[1..]);
    }

    /// A copy of a record is a record, and not a second sender on one channel.
    #[test]
    fn a_clone_keeps_the_entries_and_drops_the_subscription() {
        let mut timeline = FaultTimeline::new();
        let entries = timeline.subscribe();
        timeline.response(Maneuver::SlowStow, Outcome::Started, at(1));

        let mut copy = timeline.clone();
        assert_eq!(copy.entries(), timeline.entries());
        assert_eq!(copy.open_maneuver(), Some(Maneuver::SlowStow));

        // The copy's own entries reach nobody; the original's still arrive.
        copy.response(Maneuver::SlowStow, Outcome::Completed, at(2));
        timeline.response(Maneuver::SlowStow, Outcome::FellThrough, at(3));
        let pushed: Vec<Entry> = entries.try_iter().collect();
        assert_eq!(pushed, timeline.entries());
        assert_eq!(copy.open_maneuver(), None);
    }

    /// An outcome nobody could confirm still closes the maneuver, and says as
    /// much rather than claiming it landed.
    #[test]
    fn an_unconfirmed_ending_closes_the_maneuver_without_claiming_it_landed() {
        let mut timeline = FaultTimeline::new();
        timeline.response(Maneuver::ImmediateAllTorqueOff, Outcome::Started, at(1));
        timeline.response(Maneuver::ImmediateAllTorqueOff, Outcome::Unconfirmed, at(2));
        assert!(Outcome::Unconfirmed.ends());
        assert_eq!(timeline.open_maneuver(), None);
        assert_eq!(
            timeline.to_string(),
            "immediate_all_torque_off started → immediate_all_torque_off ran unconfirmed"
        );
    }

    /// Subscribing a second time replaces the first subscriber rather than
    /// adding one.
    ///
    /// The type's stated safety property: one consumer, because a second would
    /// be a second owner of a session's story. A `Vec<Sender>` would pass every
    /// other test here, and so would a second `subscribe` that quietly did
    /// nothing — this is the test that tells the three apart.
    #[test]
    fn a_second_subscriber_replaces_the_first() {
        let mut timeline = FaultTimeline::new();
        let first = timeline.subscribe();
        timeline.response(Maneuver::SlowStow, Outcome::Started, at(1));

        let second = timeline.subscribe();
        timeline.response(Maneuver::SlowStow, Outcome::Completed, at(2));

        let heard_first: Vec<Entry> = first.try_iter().collect();
        let heard_second: Vec<Entry> = second.try_iter().collect();
        assert_eq!(
            heard_first,
            timeline.entries()[..1],
            "the replaced subscriber heard nothing appended after it was replaced"
        );
        assert_eq!(
            heard_second,
            timeline.entries()[1..],
            "the replacement hears everything from where it came in"
        );
    }

    /// Every maneuver says a word of its own.
    ///
    /// The slugs are the vocabulary an alert rule and a status cell key on, so
    /// a typo or a copy-pasted duplicate is a condition nobody can name. The
    /// table is driven off a wildcard-free slot function, so a maneuver added
    /// to the doctrine cannot be left out of it.
    #[test]
    fn every_maneuver_names_itself_and_no_other() {
        let table = [
            (Maneuver::SlowStow, "slow_stow"),
            (Maneuver::MaskedSlowStow, "masked_slow_stow"),
            (Maneuver::AntennaTorqueOff, "antenna_torque_off"),
            (Maneuver::ImmediateAllTorqueOff, "immediate_all_torque_off"),
        ];

        let mut seen = [false; 4];
        let mut slugs: Vec<&str> = Vec::new();
        for (maneuver, slug) in table {
            seen[maneuver_slot(maneuver)] = true;
            assert_eq!(maneuver.slug(), slug, "{maneuver:?}");
            assert_eq!(maneuver.to_string(), slug, "{maneuver:?}");
            slugs.push(slug);
        }
        assert!(seen.iter().all(|named| *named), "a maneuver went unnamed");
        slugs.sort_unstable();
        let distinct = slugs.len();
        slugs.dedup();
        assert_eq!(
            slugs.len(),
            distinct,
            "two maneuvers share a slug: {slugs:?}"
        );
    }

    /// Which maneuver this is, as a slot in the table above.
    fn maneuver_slot(maneuver: Maneuver) -> usize {
        match maneuver {
            Maneuver::SlowStow => 0,
            Maneuver::MaskedSlowStow => 1,
            Maneuver::AntennaTorqueOff => 2,
            Maneuver::ImmediateAllTorqueOff => 3,
        }
    }

    /// A record with nobody listening is still a record.
    #[test]
    fn a_dropped_subscriber_costs_the_record_nothing() {
        let mut timeline = FaultTimeline::new();
        drop(timeline.subscribe());
        timeline.response(Maneuver::ImmediateAllTorqueOff, Outcome::Completed, at(7));
        assert_eq!(timeline.entries().len(), 1);
    }

    /// Every entry says itself in words, and the whole record reads as the
    /// sequence it was.
    #[test]
    fn the_record_reads_as_the_story_it_is() {
        let mut timeline = FaultTimeline::new();
        timeline.fault(
            Fault::HeadServoFault {
                joint: JointId::Leg(4),
                id: 14,
                bits: 0x20,
            },
            at(1),
        );
        timeline.response(Maneuver::MaskedSlowStow, Outcome::Started, at(2));
        timeline.response(Maneuver::MaskedSlowStow, Outcome::FellThrough, at(3));
        timeline.response(Maneuver::ImmediateAllTorqueOff, Outcome::Completed, at(4));

        let rendered = timeline.to_string();
        assert!(rendered.starts_with("head_servo_fault: "), "{rendered}");
        assert!(
            rendered.ends_with(
                "masked_slow_stow started → masked_slow_stow fell through → \
                 immediate_all_torque_off completed"
            ),
            "{rendered}"
        );
        assert_eq!(FaultTimeline::new().to_string(), "");
    }

    /// An expansion names the joints it released.
    #[test]
    fn an_expansion_says_which_joints_went() {
        let mut released = JointSet::EMPTY;
        released.insert(JointId::AntennaRight);
        let entry = Entry::Response {
            maneuver: Maneuver::AntennaTorqueOff,
            outcome: Outcome::Expanded(released),
            at: at(1),
        };
        assert_eq!(
            entry.to_string(),
            format!("antenna_torque_off expanded, released {released}")
        );
    }
}
