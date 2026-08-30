//! Running the edge: the state a process holds around it, and where the lines
//! and the alerts it produces go.
//!
//! The rest of this crate is functions and tables. This module is the small
//! amount of routing that turns them into a process's behaviour: one [`Edge`],
//! one [`Story`] follower and one [`Alerts`] table, because two intent sources
//! have to meet one gate and two narrating sources have to share one latch.
//!
//! It is here rather than in a binary because both processes that stand outside
//! the composition — the voice host in production, the harness on a run — do
//! exactly this, and a harness whose follow loop is a copy of the host's proves
//! a rehearsal of the edge rather than the edge. What each binary keeps is what
//! is genuinely its own: its sockets, its deadlines, and the [`Surface`] its
//! lines land on.
//!
//! It still binds nothing and it still stamps nothing. A [`Surface`] is where
//! the lines go and the caller passes the instant, so the whole of this
//! behaviour runs over an in-memory surface with no port and no clock. [`now`]
//! is offered for the callers that need a real one, and it is deliberately the
//! only spelling of that read in this tree: the arrival stamp is this machine's
//! clock at receipt, never a sender's, and two binaries stamping it two ways
//! would be two answers to a settled question.

use std::time::{Duration, SystemTime, UNIX_EPOCH};

use clockwork_rs::SyncTime;
use serde_json::json;

use crate::alerts::{Alert, Alerts, Severity};
use crate::config::EdgeConfig;
use crate::intake::{Accepted, Edge};
use crate::names::MotionTable;
use crate::narrate::{lost_line, refusal_line, restart_line, timeline_line};
use crate::story::{Story, Update};

/// How long a read on the reports port waits before its caller looks at
/// everything else it owns.
///
/// A liveness tick rather than the path anything urgent takes: a body handed
/// over in-process wakes the loop that holds it, and a stop signal is acted on
/// within this. Long enough that a process with nothing to do is asleep rather
/// than spinning.
pub const POLL: Duration = Duration::from_millis(250);

/// The read buffer for the reports port, comfortably past any story datagram.
///
/// A datagram longer than this arrives truncated, which the story follower
/// refuses as a wrong-sized blob rather than reading under the wrong schema —
/// so an oversize is a narrated drop, never a misreading.
pub const DATAGRAM_CAP: usize = 65_536;

/// This machine's wall clock, as the edge stamps an arrival with it.
///
/// Never the sender's stamp, even for a body off the bus: the offsets in a
/// script are measured from the instant it landed here, and a remote sender's
/// clock is not this machine's.
///
/// # Panics
///
/// If the wall clock reads before the Unix epoch, which is a machine whose
/// timekeeping nothing here can compensate for.
#[must_use]
pub fn now() -> SyncTime {
    let since = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .expect("a wall clock at or after the epoch");
    SyncTime::from_nanos(i64::try_from(since.as_nanos()).unwrap_or(i64::MAX))
}

/// Where a process running the edge says things.
///
/// Two kinds, because they have two audiences. A line is narration: the whole
/// story, read afterwards, by a person or a log shipper. An alert interrupts
/// somebody, and the table that decides which ones are worth that is this
/// crate's — this is only where the ones it raised are handed on.
pub trait Surface {
    /// One line of narration, already rendered.
    fn say(&mut self, line: String);

    /// One alert the edge's table raised.
    fn alert(&mut self, alert: &Alert);
}

/// An alert as one line, in the shape the narration stream's other lines have.
///
/// What a deployment with nothing to interrupt an operator through renders an
/// alert as, and what a deployment that does publish still writes to its log.
#[must_use]
pub fn alert_line(alert: &Alert, at: SyncTime) -> String {
    json!({
        "stream": "alert",
        "at_ns": at.as_nanos(),
        "severity": severity_word(alert.severity),
        "title": alert.title,
        "says": alert.body,
    })
    .to_string()
}

/// How an alert's loudness is spelled on the wire and in a log.
///
/// The edge's two words, spelled once here.
#[must_use]
pub const fn severity_word(severity: Severity) -> &'static str {
    match severity {
        Severity::Warning => "warning",
        Severity::Critical => "critical",
    }
}

/// The running edge: one gate, one story, one latch.
///
/// One per process, and deliberately not `Clone`: two of these would be two
/// script counters and two sequence marks, and the session would see one
/// sender's numbering jump backwards.
#[derive(Debug)]
pub struct HostEdge {
    edge: Edge,
    story: Story,
    alerts: Alerts,
}

impl HostEdge {
    /// The edge this configuration and this name table describe.
    #[must_use]
    pub fn new(config: EdgeConfig, table: MotionTable) -> Self {
        Self {
            edge: Edge::new(config, table),
            story: Story::new(),
            alerts: Alerts::new(),
        }
    }

    /// An intent body, from an authoring task in-process or from the bus.
    ///
    /// Both sources arrive here, which is what makes the sequence gate one
    /// authority. The answer is the message to send to the control process, or
    /// nothing at all: a refusal is narrated here, and never retried and never
    /// repaired.
    pub fn offer(
        &mut self,
        body: &[u8],
        arrival: SyncTime,
        surface: &mut impl Surface,
    ) -> Option<Accepted> {
        match self.edge.accept(body, arrival) {
            Ok(accepted) => {
                self.alerts.accepted();
                Some(accepted)
            }
            Err(refusal) => {
                surface.say(refusal_line(&refusal, arrival));
                if let Some(alert) = self.alerts.on_refusal(&refusal) {
                    surface.alert(&alert);
                }
                None
            }
        }
    }

    /// A datagram off the reports socket, at the instant it was read.
    ///
    /// Narrates what is new — a restart, a hole, then the rows — and hands the
    /// table each row in the order the session wrote them. A datagram that is
    /// not a story is said and dropped, and answered with nothing: a blob of
    /// the wrong length is some other schema, and reading it as this one would
    /// narrate garbage in the vocabulary of a report.
    ///
    /// What comes back is what the story gained, for a caller with its own
    /// reason to read the rows.
    pub fn follow(
        &mut self,
        datagram: &[u8],
        at: SyncTime,
        surface: &mut impl Surface,
    ) -> Option<Update> {
        // Read before the follow: a restart resets the count, and the line that
        // reports one says how much story it is replacing.
        let narrated = self.story.narrated();
        let update = match self.story.follow_bytes(datagram) {
            Ok(update) => update,
            Err(error) => {
                surface.say(not_a_story_line(&error.to_string(), at));
                return None;
            }
        };
        if update.restarted {
            // A new control process is a new machine run, and the latches say
            // "once this run": a table that carried the last run's fault latch
            // across would answer this run's fault with silence.
            self.alerts.restarted();
            surface.say(restart_line(narrated, at));
        }
        if update.lost > 0 {
            surface.say(lost_line(update.lost, at));
            if let Some(alert) = self.alerts.narration_hole(update.lost) {
                surface.alert(&alert);
            }
        }
        for row in &update.rows {
            surface.say(timeline_line(row));
            if let Some(alert) = self.alerts.on_row(row) {
                surface.alert(&alert);
            }
        }
        Some(update)
    }
}

/// A datagram on the reports port that is not a story, as a line.
fn not_a_story_line(detail: &str, at: SyncTime) -> String {
    json!({
        "stream": "edge",
        "at_ns": at.as_nanos(),
        "kind": "not_a_story",
        "says": format!(
            "a datagram on the reports port is not a story and was dropped unread: {detail}"
        ),
    })
    .to_string()
}

#[cfg(test)]
mod tests {
    use clockwork_rs::{SyncTime, blob_as_bytes};
    use motion_proto::{MotionScript, Posture, Step};

    use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
    use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, TimelineWire};

    use super::{HostEdge, Surface, alert_line, severity_word};
    use crate::alerts::{Alert, STALE_ALERT_RUN, Severity};
    use crate::config::EdgeConfig;
    use crate::names::MotionTable;

    /// The pod every fixture body is addressed to.
    const POD: &str = "fixture-reachy";

    /// A round instant, so a stamp read off the wrong side shows.
    const ARRIVAL_NS: i64 = 1_700_000_000_000_000_000;

    /// The instant a caller passes in, in place of a clock.
    fn at() -> SyncTime {
        SyncTime::from_nanos(ARRIVAL_NS)
    }

    /// Everything said, kept rather than printed.
    #[derive(Debug, Default)]
    struct Recorded {
        lines: Vec<String>,
        alerts: Vec<Alert>,
    }

    impl Surface for Recorded {
        fn say(&mut self, line: String) {
            self.lines.push(line);
        }

        fn alert(&mut self, alert: &Alert) {
            self.alerts.push(alert.clone());
        }
    }

    impl Recorded {
        /// Whether any line said carries `word`.
        fn said(&self, word: &str) -> bool {
            self.lines.iter().any(|line| line.contains(word))
        }
    }

    /// The edge under test.
    fn host() -> HostEdge {
        HostEdge::new(EdgeConfig::for_pod(POD), MotionTable::default())
    }

    /// A script body for `POD`, numbered `seq`, as the wire contract encodes
    /// one: a raise now, with the closing stow left to the compile.
    fn body(seq: u64) -> String {
        MotionScript::new(POD, seq, vec![Step::new(0, Posture::Up)], 13_000)
            .expect("a lawful script")
            .encode()
    }

    /// A story of `rows` rows of `kind`, with `dropped` said to have fallen off
    /// its front. Each row is stamped with its own number, so a diff handing
    /// back the wrong window of the story shows.
    fn story(dropped: u32, rows: u32, kind: ReportKindWire) -> TimelineWire {
        let mut message = TimelineWire::new();
        message.set_dropped(dropped);
        {
            let mut entries = message.entries_mut();
            for n in 0..rows {
                let mut entry = TimelineEntryWire::new();
                entry.set_time(SyncTime::from_nanos(ARRIVAL_NS + i64::from(n)));
                entry.set_kind(kind);
                entry.set_a(n);
                *entries
                    .try_grow()
                    .expect("a story of no more rows than the message holds") = entry;
            }
        }
        message
    }

    #[test]
    fn a_script_that_compiles_becomes_a_message_and_narrates_nothing() {
        let mut surface = Recorded::default();
        let accepted = host()
            .offer(body(1).as_bytes(), at(), &mut surface)
            .expect("the fixture compiles");
        assert_eq!(accepted.script_id, 1);
        assert!(surface.lines.is_empty(), "{:?}", surface.lines);
        assert!(surface.alerts.is_empty(), "{:?}", surface.alerts);
    }

    #[test]
    fn both_sources_meet_one_gate() {
        // The scripter's emission and a bus delivery are the same call, so a
        // redelivery of one is stale against the other. This is the whole
        // reason a process holds one edge rather than one per source.
        let mut host = host();
        let mut surface = Recorded::default();
        assert!(host.offer(body(4).as_bytes(), at(), &mut surface).is_some());
        assert!(host.offer(body(4).as_bytes(), at(), &mut surface).is_none());
        assert!(surface.said("stale"), "{:?}", surface.lines);
    }

    #[test]
    fn a_refusal_is_narrated_and_alerted_once() {
        let mut host = host();
        let mut surface = Recorded::default();
        for _ in 0..3 {
            assert!(host.offer(b"not a script", at(), &mut surface).is_none());
        }
        assert_eq!(surface.lines.len(), 3, "{:?}", surface.lines);
        assert_eq!(surface.alerts.len(), 1, "{:?}", surface.alerts);
        assert_eq!(surface.alerts[0].severity, Severity::Warning);
    }

    /// Every line the edge writes carries the instant its caller read, so a
    /// post-mortem joins on a number rather than on line ordering.
    #[test]
    fn an_edge_line_carries_the_instant_it_was_told() {
        let mut host = host();
        let mut surface = Recorded::default();
        host.offer(b"not a script", at(), &mut surface);
        host.follow(b"neither a story nor the right size", at(), &mut surface);
        for line in &surface.lines {
            let value: serde_json::Value =
                serde_json::from_str(line).expect("a line is one JSON object");
            assert_eq!(value["at_ns"], ARRIVAL_NS, "{line}");
        }
    }

    #[test]
    fn a_run_of_stale_drops_is_the_loud_one() {
        let mut host = host();
        let mut surface = Recorded::default();
        assert!(host.offer(body(9).as_bytes(), at(), &mut surface).is_some());
        for _ in 0..STALE_ALERT_RUN {
            assert!(host.offer(body(9).as_bytes(), at(), &mut surface).is_none());
        }
        let loud: Vec<&Alert> = surface
            .alerts
            .iter()
            .filter(|alert| alert.severity == Severity::Critical)
            .collect();
        assert_eq!(loud.len(), 1, "{:?}", surface.alerts);
        assert!(loud[0].title.contains("dropping"), "{}", loud[0].title);
    }

    #[test]
    fn every_new_row_of_the_story_is_a_line() {
        let mut host = host();
        let mut surface = Recorded::default();
        let first = story(0, 2, ReportKindWire::PHASE_CHANGED);
        let update = host
            .follow(blob_as_bytes(&first), at(), &mut surface)
            .expect("a story");
        assert_eq!(update.rows.len(), 2);
        assert_eq!(surface.lines.len(), 2, "{:?}", surface.lines);

        // The stream is cumulative: the same two rows plus a third is one new
        // line, not three.
        let again = story(0, 3, ReportKindWire::PHASE_CHANGED);
        host.follow(blob_as_bytes(&again), at(), &mut surface);
        assert_eq!(surface.lines.len(), 3, "{:?}", surface.lines);
    }

    #[test]
    fn a_story_that_went_backwards_says_so_before_it_narrates() {
        let mut host = host();
        let mut surface = Recorded::default();
        host.follow(
            blob_as_bytes(&story(0, 3, ReportKindWire::PHASE_CHANGED)),
            at(),
            &mut surface,
        );
        surface.lines.clear();

        host.follow(
            blob_as_bytes(&story(0, 1, ReportKindWire::PHASE_CHANGED)),
            at(),
            &mut surface,
        );
        assert!(surface.lines[0].contains("story_restarted"), "{surface:?}");
        assert_eq!(surface.lines.len(), 2, "{:?}", surface.lines);
    }

    #[test]
    fn rows_that_fell_off_the_ring_are_counted_before_the_tail() {
        let mut host = host();
        let mut surface = Recorded::default();
        host.follow(
            blob_as_bytes(&story(0, 1, ReportKindWire::PHASE_CHANGED)),
            at(),
            &mut surface,
        );
        surface.lines.clear();

        host.follow(
            blob_as_bytes(&story(4, 1, ReportKindWire::PHASE_CHANGED)),
            at(),
            &mut surface,
        );
        assert!(surface.lines[0].contains("story_rows_lost"), "{surface:?}");
    }

    #[test]
    fn a_datagram_that_is_not_a_story_is_said_and_dropped() {
        let mut host = host();
        let mut surface = Recorded::default();
        assert!(
            host.follow(b"neither a story nor the right size", at(), &mut surface)
                .is_none()
        );
        assert!(surface.said("not_a_story"), "{:?}", surface.lines);
        assert!(surface.alerts.is_empty(), "{:?}", surface.alerts);
    }

    #[test]
    fn a_fault_row_is_one_critical_alert_for_the_run() {
        let mut host = host();
        let mut surface = Recorded::default();
        host.follow(
            blob_as_bytes(&story(0, 2, ReportKindWire::FAULT_RECORDED)),
            at(),
            &mut surface,
        );
        host.follow(
            blob_as_bytes(&story(0, 4, ReportKindWire::FAULT_RECORDED)),
            at(),
            &mut surface,
        );
        assert_eq!(surface.alerts.len(), 1, "{:?}", surface.alerts);
        assert_eq!(surface.alerts[0].severity, Severity::Critical);
    }

    /// The latches say "once this run", and a control process that restarted is
    /// a new run. The second fault is a second machine's, and an operator who
    /// heard about the first one has heard nothing about it.
    #[test]
    fn a_restart_lets_the_next_run_raise_its_own_fault() {
        let mut host = host();
        let mut surface = Recorded::default();
        host.follow(
            blob_as_bytes(&story(0, 2, ReportKindWire::FAULT_RECORDED)),
            at(),
            &mut surface,
        );
        assert_eq!(surface.alerts.len(), 1, "{:?}", surface.alerts);

        // A shorter story than the one already narrated: the teller restarted.
        host.follow(
            blob_as_bytes(&story(0, 1, ReportKindWire::FAULT_RECORDED)),
            at(),
            &mut surface,
        );
        assert!(surface.said("story_restarted"), "{:?}", surface.lines);
        assert_eq!(surface.alerts.len(), 2, "{:?}", surface.alerts);
        assert_eq!(surface.alerts[1].severity, Severity::Critical);
    }

    /// A hole is rows nothing classified, so it is said on the alert plane too:
    /// every latch is computed from rows, and the rows that never arrived could
    /// have carried anything.
    #[test]
    fn a_hole_in_the_narration_is_one_warning() {
        let mut host = host();
        let mut surface = Recorded::default();
        host.follow(
            blob_as_bytes(&story(0, 1, ReportKindWire::PHASE_CHANGED)),
            at(),
            &mut surface,
        );
        assert!(surface.alerts.is_empty(), "{:?}", surface.alerts);

        host.follow(
            blob_as_bytes(&story(4, 1, ReportKindWire::PHASE_CHANGED)),
            at(),
            &mut surface,
        );
        host.follow(
            blob_as_bytes(&story(9, 1, ReportKindWire::PHASE_CHANGED)),
            at(),
            &mut surface,
        );
        assert_eq!(surface.alerts.len(), 1, "{:?}", surface.alerts);
        assert_eq!(surface.alerts[0].severity, Severity::Warning);
        assert!(
            surface.alerts[0].title.contains("hole"),
            "{:?}",
            surface.alerts[0]
        );
    }

    #[test]
    fn an_alert_is_a_line_of_the_same_shape() {
        let line = alert_line(
            &Alert {
                severity: Severity::Critical,
                title: "a title".to_owned(),
                body: "a body".to_owned(),
            },
            at(),
        );
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("one JSON object");
        assert_eq!(parsed["stream"], "alert");
        assert_eq!(parsed["at_ns"], ARRIVAL_NS);
        assert_eq!(parsed["severity"], "critical");
        assert_eq!(parsed["title"], "a title");
        assert_eq!(parsed["says"], "a body");
        assert_eq!(severity_word(Severity::Warning), "warning");
    }
}
