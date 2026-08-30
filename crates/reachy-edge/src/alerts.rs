//! The alert table: which of the things narrated are worth waking somebody for.
//!
//! Narration goes to a log and a log is read afterwards. An alert interrupts a
//! person, so the table here is deliberately small and deliberately latched:
//! the conditions worth an alert are conditions that persist, and a condition
//! that persists must not produce an alert per datagram over the channel a
//! fault has to arrive on.
//!
//! Three conditions, three latches:
//!
//! - **A fault or a park.** The machine stopped doing what it was asked, or
//!   latched out of engaging until an operator has been. One alert for the run,
//!   ever: a fault does not improve, and a machine parked for an hour must not
//!   alert for an hour. Critical.
//! - **Refused scripts.** The session declined one, or the edge dropped one
//!   before it got there. Either way a sender and this machine disagree about
//!   something no retry fixes, and nothing else would tell anybody. One alert
//!   per run, because a sender emitting garbage at the refresh cadence would
//!   otherwise be an alert every few seconds. Warning: nothing about the
//!   machine is wrong when it declines a script it cannot run.
//! - **A run of stale drops.** The shape of a machine that has gone
//!   permanently deaf — every script arriving at or below a mark it will not
//!   go below — which is indistinguishable from an idle one to everything but
//!   this counter. Critical, once, after a run of them.
//! - **A hole in the narration.** Rows the session narrated that no datagram
//!   carried here. Every latch above is computed from rows, so rows that never
//!   arrived are conditions this table cannot have classified — including a
//!   fault. One Warning per run, saying the account is incomplete rather than
//!   pretending to know what fell through.
//!
//! What a run is, for the latches: the machine's, not this process's. The
//! control process restarting begins a new one, and [`Alerts::restarted`] is
//! where the table starts over — a fault the last run had is not a reason for
//! the next one's fault to go unannounced.
//!
//! What a row means is not written here. An alert body quotes the sentence the
//! narration renders that row with, so the line in the log and the alert on
//! somebody's phone are one account of one event rather than two that have to
//! be recognised as the same. This module decides *which* rows are worth the
//! interruption; [`crate::narrate`] says what they say.
//!
//! What this module does *not* do is send anything. It is a table over the
//! things the edge already saw, and the process that holds the bus attachment
//! is what publishes what it hands back. That split is what lets a deployment
//! with no attachment configured run the whole edge with its alerts as
//! narration and nothing else.

use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKind, ReportKindWire};
use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;

use crate::intake::Refusal;
use crate::narrate::{refusal_reason, says};

/// How many drops in a row say the machine has gone deaf rather than idle.
///
/// Three, because a sender's presence refresh arrives every few seconds: one
/// stale drop is a redelivery, and three in a row is a sender whose numbering
/// this run will never accept again.
pub const STALE_ALERT_RUN: u64 = 3;

/// How loud an alert is.
///
/// The edge's own vocabulary rather than the bus client's, because this crate
/// holds no bus client: the process that publishes maps these two onto whatever
/// its attachment calls them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Severity {
    /// Something an operator should look at, on a machine that is not itself in
    /// trouble.
    Warning,
    /// A machine that has stopped doing what it was asked.
    Critical,
}

/// One alert, ready to publish.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Alert {
    /// How loud.
    pub severity: Severity,
    /// The headline, which is what a phone notification shows.
    pub title: String,
    /// What happened, what it means, and what it does not mean.
    pub body: String,
}

/// The latches, and the counts they are latched over.
///
/// One per run. Nothing here is persisted and nothing resets it but a restart:
/// a latch that forgot would alert twice about one standing condition, which is
/// how the alert that says something gets buried.
#[derive(Clone, Debug, Default)]
pub struct Alerts {
    /// Whether the fault-or-park alert has gone out.
    faulted: bool,
    /// Scripts refused or dropped this run.
    refusals: u64,
    /// Whether the refusal alert has gone out.
    alerted_refusal: bool,
    /// Drops for staleness since the last script that was accepted.
    stale_run: u64,
    /// Whether the staleness alert has gone out.
    alerted_stale: bool,
    /// Whether the incomplete-narration alert has gone out.
    alerted_hole: bool,
}

impl Alerts {
    /// A table that has raised nothing.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// How many scripts were refused or dropped this run.
    #[must_use]
    pub fn refusals(&self) -> u64 {
        self.refusals
    }

    /// How many scripts in a row were dropped as stale.
    #[must_use]
    pub fn stale_run(&self) -> u64 {
        self.stale_run
    }

    /// The process telling the story restarted: a new run, and a new table.
    ///
    /// Every latch here says "once this run", and the run they mean is the
    /// machine's: a fault that stopped the last control process is not a reason
    /// for the next one's fault to raise nothing. A table that carried the last
    /// run's latches across would go quiet on the one channel a fault has to
    /// arrive on for as long as this process lives.
    pub fn restarted(&mut self) {
        *self = Self::new();
    }

    /// Rows the session narrated that no datagram carried here.
    ///
    /// Everything else in this table is computed from rows, so a hole is a
    /// stretch of story nothing classified — a fault or a park among them, for
    /// all this can say. One Warning for the run: the condition is the
    /// account's, not the machine's, and saying it once is what keeps it from
    /// burying the alerts that are.
    #[must_use]
    pub fn narration_hole(&mut self, lost: u64) -> Option<Alert> {
        if lost == 0 || self.alerted_hole {
            return None;
        }
        self.alerted_hole = true;
        Some(Alert {
            severity: Severity::Warning,
            title: "reachy narration has a hole in it".to_owned(),
            body: format!(
                "{lost} row(s) of the session's story never reached this process: it narrated \
                 them faster than the datagrams carrying them arrived, and its ring overran. \
                 what fell through was not classified, so a fault or a park inside the hole \
                 raised nothing — the session's own log is the whole account. this is the only \
                 alert of its kind this run."
            ),
        })
    }

    /// A script the edge accepted: the deafness run ends here.
    ///
    /// The run counts *consecutive* drops, so an accepted script is what says
    /// the sender and this machine still agree about numbering. The refusal
    /// count is not reset by it: a run that refused scripts refused them,
    /// whatever happened afterwards.
    pub fn accepted(&mut self) {
        self.stale_run = 0;
    }

    /// A body the edge dropped before the session saw it.
    ///
    /// The count rises for every drop, whichever alert the drop goes on to
    /// raise: a run of stale drops raises the louder one, and the count still
    /// has to be the number of scripts that did not run.
    #[must_use]
    pub fn on_refusal(&mut self, refusal: &Refusal) -> Option<Alert> {
        self.refusals += 1;
        if matches!(refusal, Refusal::Stale { .. }) {
            self.stale_run += 1;
            if let Some(alert) = self.stale_alert() {
                return Some(alert);
            }
        }
        self.refusal_alert(&refusal.to_string())
    }

    /// One row of the session's story.
    #[must_use]
    pub fn on_row(&mut self, row: &TimelineEntryWire) -> Option<Alert> {
        if let Some(refused) = refused_script(row) {
            self.refusals += 1;
            if refused == RefusalReasonWire::STALE {
                self.stale_run += 1;
                if let Some(alert) = self.stale_alert() {
                    return Some(alert);
                }
            }
            return self.refusal_alert(&format!(
                "the session refused script {}: {}",
                row.a(),
                refusal_reason(row.b())
            ));
        }
        if !stops_the_machine(row) {
            return None;
        }
        if self.faulted {
            return None;
        }
        self.faulted = true;
        let stopped = says(row);
        Some(Alert {
            severity: Severity::Critical,
            title: "reachy head motion stopped".to_owned(),
            body: format!(
                "the session's story says: {stopped}. reaching the minimum risk condition is the machine's own to do and \
                 needs nothing from an operator to happen — it stows what it can still command \
                 and then takes torque off. nothing is cleared automatically and nothing is \
                 retried: a park-class response waits for an operator to restart the stack, and \
                 a rest-class one ends the session, so the next wake is a fresh engagement \
                 rather than a recovery of this one. this is the only alert of its kind this \
                 run; the narration carries every row."
            ),
        })
    }

    /// The refusal alert, once a run, over a count its callers have already
    /// raised.
    fn refusal_alert(&mut self, detail: &str) -> Option<Alert> {
        if self.alerted_refusal {
            return None;
        }
        self.alerted_refusal = true;
        Some(Alert {
            severity: Severity::Warning,
            title: "reachy motion scripts refused".to_owned(),
            body: format!(
                "{} script(s) refused so far; the most recent: {detail}. nothing about the \
                 machine is wrong — a refused script is declined whole and never partly run, and \
                 whatever timeline is already running still bounds the head. what it means is \
                 that a sender and this machine disagree about something no retry fixes. this is \
                 the only refusal alert of the run; the narration carries every one.",
                self.refusals
            ),
        })
    }

    /// The deafness alert, once a run, when the run of drops is long enough.
    fn stale_alert(&mut self) -> Option<Alert> {
        if self.alerted_stale || self.stale_run < STALE_ALERT_RUN {
            return None;
        }
        self.alerted_stale = true;
        Some(Alert {
            severity: Severity::Critical,
            title: "reachy head is dropping every script".to_owned(),
            body: format!(
                "{} script(s) in a row dropped as stale: they are numbered at or below the \
                 highest this run has accepted, and the mark only goes up. the head will not \
                 move again until a script arrives above it, or until the stack is restarted — a \
                 restart forgets the mark. the likeliest cause is two senders numbering one \
                 machine, or one sender that restarted its numbering.",
                self.stale_run
            ),
        })
    }
}

/// The reason a row refuses a script with, or `None` for any other row.
fn refused_script(row: &TimelineEntryWire) -> Option<RefusalReasonWire> {
    (row.kind() == ReportKindWire::SCRIPT_REFUSED)
        .then(|| RefusalReasonWire(u8::try_from(row.b()).unwrap_or(u8::MAX)))
}

/// Whether this row says the machine stopped doing what it was asked.
///
/// A classification and nothing more. What a row *says* is spelled once, in the
/// narration every row is rendered through, so the alert body and the line
/// about one row cannot come to describe it differently.
///
/// The wildcard-free `match` is the point: a report kind the vocabulary grows
/// is a compile error here rather than a new way for the machine to stop
/// quietly. A park is in the set because it is the same news to an operator —
/// nothing engages until somebody has been — even though nothing is wrong with
/// the machine that parked out of a survey it refused.
fn stops_the_machine(row: &TimelineEntryWire) -> bool {
    let (a, b) = (row.a(), row.b());
    let Some(kind) = row.kind().to_known() else {
        return false;
    };
    match kind {
        ReportKind::FaultRecorded
        | ReportKind::ResponseTaken
        | ReportKind::TorqueOffUnconfirmed
        | ReportKind::BusFailureDeclared
        | ReportKind::CommissionFailed => true,
        // A wind-down that concluded park-class is the row that says the
        // machine latched; the outcome alone is not, because a wind-down runs
        // for endings that rest instead.
        ReportKind::WinddownOutcome => b != 0,
        ReportKind::PhaseChanged => {
            SessionPhaseWire(u8::try_from(a).unwrap_or(u8::MAX)) == SessionPhaseWire::PARKED
        }
        ReportKind::None
        | ReportKind::ScriptAccepted
        | ReportKind::ScriptRefused
        | ReportKind::TorqueOffConfirmed
        | ReportKind::SessionEnded
        | ReportKind::AuxGaveUp
        | ReportKind::SchedulePublished
        | ReportKind::DegradeReleased
        | ReportKind::ScriptReplaced => false,
    }
}

#[cfg(test)]
mod tests {
    use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
    use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKindWire};
    use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;

    use super::{Alerts, STALE_ALERT_RUN, Severity};
    use crate::intake::Refusal;

    fn row(kind: ReportKindWire, a: u32, b: u32) -> TimelineEntryWire {
        let mut entry = TimelineEntryWire::new();
        entry.set_kind(kind);
        entry.set_a(a);
        entry.set_b(b);
        entry
    }

    fn phase(entered: SessionPhaseWire) -> TimelineEntryWire {
        row(ReportKindWire::PHASE_CHANGED, u32::from(entered.0), 0)
    }

    fn stale(seq: u64) -> Refusal {
        Refusal::Stale { seq, accepted: 99 }
    }

    #[test]
    fn a_fault_is_one_critical_alert_and_then_silence() {
        let mut alerts = Alerts::new();
        let first = alerts
            .on_row(&row(ReportKindWire::FAULT_RECORDED, 3, 12))
            .expect("a fault owes an alert");
        assert_eq!(first.severity, Severity::Critical);
        assert!(first.title.contains("stopped"), "{}", first.title);
        // A fault does not improve, and a machine that stays stopped must not
        // alert for as long as it stays stopped.
        assert_eq!(
            alerts.on_row(&row(ReportKindWire::RESPONSE_TAKEN, 2, 3)),
            None
        );
        assert_eq!(
            alerts.on_row(&row(ReportKindWire::BUS_FAILURE_DECLARED, 0, 0)),
            None
        );
    }

    #[test]
    fn a_park_is_the_same_news_as_a_fault() {
        let mut alerts = Alerts::new();
        let alert = alerts
            .on_row(&phase(SessionPhaseWire::PARKED))
            .expect("a park owes an alert");
        assert_eq!(alert.severity, Severity::Critical);
        assert!(alert.body.contains("operator"), "{}", alert.body);
    }

    /// The rows of an ordinary run. A gesture that worked must produce nothing.
    #[test]
    fn the_rows_of_a_clean_run_owe_no_alert() {
        let mut alerts = Alerts::new();
        for entry in [
            phase(SessionPhaseWire::RESTING),
            row(ReportKindWire::SCRIPT_ACCEPTED, 1, 3),
            row(ReportKindWire::SCHEDULE_PUBLISHED, 1, 3),
            phase(SessionPhaseWire::ENGAGING),
            phase(SessionPhaseWire::ACTIVE),
            phase(SessionPhaseWire::STOPPING),
            row(ReportKindWire::TORQUE_OFF_CONFIRMED, 0, 0),
            row(ReportKindWire::SESSION_ENDED, 1, 0),
            phase(SessionPhaseWire::RESTING),
        ] {
            assert_eq!(alerts.on_row(&entry), None, "{entry:?}");
        }
    }

    /// A wind-down that rested is the presence contract concluding, not a park.
    #[test]
    fn a_winddown_that_did_not_park_is_not_an_alert() {
        let mut alerts = Alerts::new();
        assert_eq!(
            alerts.on_row(&row(ReportKindWire::WINDDOWN_OUTCOME, 1, 0)),
            None
        );
        assert!(
            alerts
                .on_row(&row(ReportKindWire::WINDDOWN_OUTCOME, 1, 1))
                .is_some(),
            "a park-class wind-down owes an alert"
        );
    }

    #[test]
    fn refusals_are_one_warning_a_run_over_a_count_that_keeps_rising() {
        let mut alerts = Alerts::new();
        let first = alerts
            .on_refusal(&Refusal::NotText)
            .expect("a refusal owes an alert");
        assert_eq!(first.severity, Severity::Warning);
        assert!(first.body.contains("1 script(s)"), "{}", first.body);
        assert_eq!(
            alerts.on_row(&row(
                ReportKindWire::SCRIPT_REFUSED,
                2,
                u32::from(RefusalReasonWire::BAD_TIMES.0)
            )),
            None
        );
        assert_eq!(alerts.refusals(), 2);
    }

    /// The edge's own drops and the session's refusals are one condition: a
    /// sender disagreeing with this machine, wherever the disagreement was
    /// caught.
    #[test]
    fn a_session_refusal_and_an_edge_drop_share_the_one_latch() {
        let mut alerts = Alerts::new();
        assert!(
            alerts
                .on_row(&row(
                    ReportKindWire::SCRIPT_REFUSED,
                    1,
                    u32::from(RefusalReasonWire::TOO_LONG.0)
                ))
                .is_some(),
            "the first refusal owes an alert"
        );
        assert_eq!(alerts.on_refusal(&Refusal::NotText), None);
    }

    #[test]
    fn a_run_of_stale_drops_is_a_critical_alert_once_the_run_is_long_enough() {
        let mut alerts = Alerts::new();
        // The first drops raise the refusal warning, not the deafness alert.
        for n in 0..STALE_ALERT_RUN - 1 {
            let alert = alerts.on_refusal(&stale(n));
            assert!(
                alert.is_none_or(|alert| alert.severity == Severity::Warning),
                "a short run of drops is not deafness"
            );
        }
        let alert = alerts
            .on_refusal(&stale(STALE_ALERT_RUN))
            .expect("a run of drops owes an alert");
        assert_eq!(alert.severity, Severity::Critical);
        assert!(alert.title.contains("dropping"), "{}", alert.title);
        // Once. A sender that keeps its numbering keeps producing these.
        assert_eq!(alerts.on_refusal(&stale(STALE_ALERT_RUN + 1)), None);
    }

    /// A stale drop the session caught counts toward the same run: the mark it
    /// was measured against is the running engagement's rather than the edge's,
    /// and to an operator it is the same deafness.
    #[test]
    fn the_session_s_own_stale_refusals_count_toward_the_run() {
        let mut alerts = Alerts::new();
        let refused = row(
            ReportKindWire::SCRIPT_REFUSED,
            1,
            u32::from(RefusalReasonWire::STALE.0),
        );
        for _ in 0..STALE_ALERT_RUN {
            let _ = alerts.on_row(&refused);
        }
        assert_eq!(alerts.stale_run(), STALE_ALERT_RUN);
    }

    /// The count is what a body quotes, so a drop that raised the louder alert
    /// still has to be in it: three scripts that did not run are three,
    /// whichever alert each of them went on to raise.
    #[test]
    fn every_drop_counts_toward_the_refusal_tally_whatever_it_alerted() {
        let mut alerts = Alerts::new();
        for n in 0..STALE_ALERT_RUN {
            let _ = alerts.on_refusal(&stale(n));
        }
        assert_eq!(alerts.refusals(), STALE_ALERT_RUN);

        let mut session = Alerts::new();
        let refused = row(
            ReportKindWire::SCRIPT_REFUSED,
            1,
            u32::from(RefusalReasonWire::STALE.0),
        );
        for _ in 0..STALE_ALERT_RUN {
            let _ = session.on_row(&refused);
        }
        assert_eq!(session.refusals(), STALE_ALERT_RUN);
    }

    /// The alert an operator reads names the reason in words. The number is the
    /// one thing a reader away from a terminal cannot look up.
    #[test]
    fn a_session_refusal_alert_spells_its_reason_out() {
        let mut alerts = Alerts::new();
        let alert = alerts
            .on_row(&row(
                ReportKindWire::SCRIPT_REFUSED,
                2,
                u32::from(RefusalReasonWire::PARKED.0),
            ))
            .expect("the first refusal owes an alert");
        assert!(
            alert.body.contains("the machine is parked"),
            "{}",
            alert.body
        );
    }

    /// One row, one account of it: the alert body quotes the sentence the
    /// narration renders the same row with.
    #[test]
    fn a_stopping_row_is_described_the_way_the_narration_describes_it() {
        let mut alerts = Alerts::new();
        let entry = row(ReportKindWire::FAULT_RECORDED, 3, 12);
        let alert = alerts.on_row(&entry).expect("a fault owes an alert");
        assert!(
            alert.body.contains(&crate::narrate::says(&entry)),
            "{}",
            alert.body
        );
    }

    /// An accepted script says the sender and this machine still agree about
    /// numbering, which is what the run was counting the absence of.
    #[test]
    fn an_accepted_script_ends_the_run_of_drops() {
        let mut alerts = Alerts::new();
        let _ = alerts.on_refusal(&stale(1));
        let _ = alerts.on_refusal(&stale(2));
        alerts.accepted();
        assert_eq!(alerts.stale_run(), 0);
        assert_eq!(alerts.on_refusal(&stale(3)), None);
        assert_eq!(alerts.stale_run(), 1);
    }
}
