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
//! - **An antenna pair released.** A fault scoped to the antennas is answered
//!   by letting the pair go, and the head keeps its session — so it is neither
//!   the machine stopping nor nothing. Warning, and the fault row it answers
//!   raises nothing of its own: the two rows are one event, and classifying
//!   both would say it twice.
//! - **Refused scripts.** The session declined one, or the edge dropped one
//!   before it got there. Either way a sender and this machine disagree about
//!   something no retry fixes, and nothing else would tell anybody. One alert
//!   per run, because a sender emitting garbage at the refresh cadence would
//!   otherwise be an alert every few seconds. Warning: nothing about the
//!   machine is wrong when it declines a script it cannot run.
//! - **A refused script this machine authored for itself.** Not the same news:
//!   a body the edge dropped that its own process wrote is this machine
//!   disagreeing with itself, and no sender's refresh recovers it — the head
//!   will not move for anything said to the robot until somebody edits a file.
//!   Critical, once a run. The staleness screen is the exception, whatever the
//!   origin: a local script numbered below the mark is a pipeline that
//!   restarted its numbering under a host that kept its own, which is the
//!   two-senders news the Warning already carries.
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
//! A Critical also carries a sentence to say out loud, for the deployments that
//! can speak: the person standing in front of the robot is the one the machine
//! has stopped working for, and a log they will read tomorrow is not a report to
//! them. Only the Criticals carry one — spoken words interrupt a room, and a
//! Warning is something an operator reads afterwards.
//!
//! What this module does *not* do is send anything. It is a table over the
//! things the edge already saw, and the process that holds the bus attachment
//! is what publishes what it hands back. That split is what lets a deployment
//! with no attachment configured run the whole edge with its alerts as
//! narration and nothing else.

use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__motion__faults_clk_rs::{
    FaultKind, FaultKindWire, ResponseKind, ResponseKindWire,
};
use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKind, ReportKindWire};
use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;

use crate::intake::{Origin, Refusal};
use crate::narrate::{refusal_reason, row_says};

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
    /// One sentence the robot says out loud, where the deployment can speak,
    /// or `None` for an alert not worth interrupting a room with.
    ///
    /// Must be short, free of identifiers a listener cannot use, and never a
    /// claim about the machine's state the row itself does not make.
    pub spoken: Option<String>,
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
    /// Whether the own-scripts alert has gone out.
    alerted_own: bool,
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
            spoken: None,
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

    /// A body the edge dropped before the session saw it, and where it was
    /// authored.
    ///
    /// The count rises for every drop, whichever alert the drop goes on to
    /// raise: a run of stale drops raises the louder one, and the count still
    /// has to be the number of scripts that did not run.
    ///
    /// A [`Origin::Local`] drop that is not the staleness screen raises the
    /// own-scripts Critical instead of the refusal Warning, and never touches
    /// the Warning's latch: the two conditions are different news, and a
    /// machine refusing its own scripts must not spend the quieter latch on
    /// the way to the louder one.
    #[must_use]
    pub fn on_refusal(&mut self, refusal: &Refusal, origin: Origin) -> Option<Alert> {
        self.refusals += 1;
        if matches!(refusal, Refusal::Stale { .. }) {
            self.stale_run += 1;
            if let Some(alert) = self.stale_alert() {
                return Some(alert);
            }
            return self.refusal_alert(&refusal.to_string());
        }
        match origin {
            Origin::Local => self.own_alert(refusal),
            Origin::Remote => self.refusal_alert(&refusal.to_string()),
        }
    }

    /// One row of the session's story.
    #[must_use]
    pub fn on_row(&mut self, row: &TimelineEntryWire) -> Option<Alert> {
        if let Some(refused) = refused_script(row) {
            self.refusals += 1;
            // The refusal alert says a sender and this machine disagree about
            // something no retry fixes; a fault's ending is not that -- the fix
            // is to ask again once the machine has rested, and the fault's own
            // rows have already carried the critical.
            if refused == RefusalReasonWire::FAULT_ENDING {
                return None;
            }
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
        match antenna_scoped(row) {
            // The pair let go of, with the head still under command: news, and
            // not the news that the machine stopped.
            Some(AntennaRow::Released) => return Some(antennas_alert(row)),
            // The condition itself. The response row that follows it in the
            // same wake is what carries this to an operator, so classifying
            // the fault as well would say it twice, the second time louder.
            Some(AntennaRow::Fault) => return None,
            None => {}
        }
        if !stops_the_machine(row) {
            return None;
        }
        if self.faulted {
            return None;
        }
        self.faulted = true;
        let stopped = row_says(row);
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
            spoken: Some(spoken_stop(row)),
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
            spoken: None,
        })
    }

    /// The own-scripts alert, once a run.
    ///
    /// Louder than the refusal Warning because it is a different condition:
    /// the process that authored the script and the process that screened it
    /// are on this one machine, so there is no other sender to blame and no
    /// refresh that recovers. Until the two configurations agree, the head
    /// will not move for anything said to the robot.
    fn own_alert(&mut self, refusal: &Refusal) -> Option<Alert> {
        if self.alerted_own {
            return None;
        }
        self.alerted_own = true;
        Some(Alert {
            severity: Severity::Critical,
            title: "reachy head refuses its own scripts".to_owned(),
            body: format!(
                "the edge dropped a motion script this machine authored for itself: {refusal}. \
                 both ends of that disagreement are on this machine, so nothing retries it and \
                 no sender's next refresh recovers it — the head will not move for anything said \
                 to this robot until the two configurations agree. this is the only alert of its \
                 kind this run; the narration carries every one."
            ),
            spoken: Some(spoken_own(refusal)),
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
            spoken: Some("My head is dropping every motion script.".to_owned()),
        })
    }
}

/// What the robot says out loud about a row that stopped it.
///
/// Two sentences, because one row means something a bystander has to act on.
/// [`ReportKind::TorqueOffUnconfirmed`] exists to say de-torquing was *not*
/// confirmed, and stowed with torque held is this machine's one pinch hazard,
/// so what a person in the room hears has to say the same. Every other row here
/// says the head stopped and nothing about limpness: reaching the minimum risk
/// condition is the machine's own to do, and these rows do not report it
/// reached.
fn spoken_stop(row: &TimelineEntryWire) -> String {
    if row.kind().to_known() == Some(ReportKind::TorqueOffUnconfirmed) {
        return "My head motion has stopped, and I could not confirm my motors are off. Do not \
                touch my head."
            .to_owned();
    }
    "My head motion has stopped.".to_owned()
}

/// What the robot says out loud about a script of its own that was refused.
///
/// The two names, where the refusal holds them: they are the whole of what
/// somebody has to reconcile, and a listener who has them can act without
/// reading a log. Any other screen has nothing a listener could use, so the
/// sentence says only what happened.
fn spoken_own(refusal: &Refusal) -> String {
    match refusal {
        Refusal::ForeignPod { addressed, pod } => format!(
            "My head is not moving. My motion scripts are addressed to {addressed}, but I answer \
             to {pod}."
        ),
        Refusal::Oversize { .. }
        | Refusal::NotText
        | Refusal::Undecodable(_)
        | Refusal::Stale { .. }
        | Refusal::Uncompilable(_) => {
            "My head is not moving. My own motion scripts are being refused.".to_owned()
        }
    }
}

/// The reason a row refuses a script with, or `None` for any other row.
fn refused_script(row: &TimelineEntryWire) -> Option<RefusalReasonWire> {
    (row.kind() == ReportKindWire::SCRIPT_REFUSED)
        .then(|| RefusalReasonWire(u8::try_from(row.b()).unwrap_or(u8::MAX)))
}

/// Which half of an antenna pair's own trouble a row is.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum AntennaRow {
    /// The condition: an antenna obstructed, or an antenna servo's own error
    /// byte.
    Fault,
    /// The answer: the pair released, the head carrying on.
    Released,
}

/// Whether this row is about an antenna pair alone, and which half.
///
/// The two rows a scoped de-torque produces, and nothing else: a response that
/// covers the head is the machine stopping, and a fault the head shares is
/// answered by a maneuver that stops it.
fn antenna_scoped(row: &TimelineEntryWire) -> Option<AntennaRow> {
    let kind = row.kind().to_known()?;
    if kind == ReportKind::FaultRecorded && antenna_fault(row.a()) {
        return Some(AntennaRow::Fault);
    }
    if kind == ReportKind::ResponseTaken && releases_the_antennas(row.a()) {
        return Some(AntennaRow::Released);
    }
    None
}

/// Whether a fault kind is one only the antennas raise.
///
/// Wildcard-free over the fault vocabulary: a fault the vocabulary grows is a
/// compile error here rather than a condition silently classified as the head's.
fn antenna_fault(fault: u32) -> bool {
    match FaultKindWire(u8::try_from(fault).unwrap_or(u8::MAX)).to_known() {
        Some(FaultKind::AntennaObstructed | FaultKind::AntennaServoFault) => true,
        Some(
            FaultKind::None
            | FaultKind::HeadObstructed
            | FaultKind::HeadServoFault
            | FaultKind::PositionFeedbackLost
            | FaultKind::MeasuredPoseInvalid
            | FaultKind::BusFailure
            | FaultKind::TorqueOffUnconfirmed
            | FaultKind::MoveAbortedEnvelope
            | FaultKind::MoveAbortedStep
            | FaultKind::CommandRejected,
        )
        | None => false,
    }
}

/// Whether a response is the one that lets the antenna pair go and carries on.
///
/// Wildcard-free for the same reason [`antenna_fault`] is: every other response
/// in the vocabulary ends the session or parks the machine.
fn releases_the_antennas(response: u32) -> bool {
    match ResponseKindWire(u8::try_from(response).unwrap_or(u8::MAX)).to_known() {
        Some(ResponseKind::DegradeAntennas) => true,
        Some(
            ResponseKind::None
            | ResponseKind::Refuse
            | ResponseKind::SlowStowToRest
            | ResponseKind::MaskedSlowStowToPark
            | ResponseKind::ImmediateAllTorqueOffToRest
            | ResponseKind::ImmediateAllTorqueOffToPark,
        )
        | None => false,
    }
}

/// The alert a released antenna pair is worth.
///
/// A Warning, unlatched and unspoken: the row is one event rather than a
/// standing condition, the head still has its session, and a room does not need
/// to be interrupted for a pair of antennas.
fn antennas_alert(row: &TimelineEntryWire) -> Alert {
    Alert {
        severity: Severity::Warning,
        title: "reachy antennas released".to_owned(),
        body: format!(
            "the session's story says: {}. the head keeps its session and goes on doing what it \
             was asked; the antennas are limp and out of service until the next engagement.",
            row_says(row)
        ),
        spoken: None,
    }
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
        | ReportKind::ScriptReplaced
        | ReportKind::ScriptHeld => false,
    }
}

#[cfg(test)]
mod tests {
    use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
    use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKindWire};
    use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;

    use super::{
        Alerts, FaultKind, FaultKindWire, ResponseKind, ResponseKindWire, STALE_ALERT_RUN, Severity,
    };
    use crate::intake::{Origin, Refusal};

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

    /// A fault kind as one of a report's numbers.
    fn fault_of(kind: FaultKind) -> u32 {
        u32::from(FaultKindWire::from(kind).0)
    }

    /// A response as one of a report's numbers.
    fn response_of(kind: ResponseKind) -> u32 {
        u32::from(ResponseKindWire::from(kind).0)
    }

    /// The two rows an obstructed antenna produces: the condition, which says
    /// nothing on its own, and the answer, which is a Warning about a machine
    /// that is still doing what it was asked.
    #[test]
    fn a_released_antenna_pair_is_a_warning_the_fault_row_leaves_to_it() {
        let mut alerts = Alerts::new();
        assert_eq!(
            alerts.on_row(&row(
                ReportKindWire::FAULT_RECORDED,
                fault_of(FaultKind::AntennaObstructed),
                8
            )),
            None,
            "the condition is carried by the answer that follows it"
        );
        let alert = alerts
            .on_row(&row(
                ReportKindWire::RESPONSE_TAKEN,
                response_of(ResponseKind::DegradeAntennas),
                fault_of(FaultKind::AntennaObstructed),
            ))
            .expect("a released pair owes an alert");
        assert_eq!(alert.severity, Severity::Warning);
        assert_eq!(alert.title, "reachy antennas released");
        assert_eq!(alert.spoken, None, "a room is not interrupted for this");
        assert!(alert.body.contains("keeps its session"), "{}", alert.body);

        // The Critical latch is untouched: the head stopping after this is
        // still the news it was.
        let head = alerts
            .on_row(&row(
                ReportKindWire::FAULT_RECORDED,
                fault_of(FaultKind::HeadObstructed),
                2,
            ))
            .expect("a head fault owes an alert of its own");
        assert_eq!(head.severity, Severity::Critical);
    }

    /// An antenna servo's own error byte is the same news by another route.
    #[test]
    fn an_antenna_servo_fault_row_is_left_to_its_answer_too() {
        let mut alerts = Alerts::new();
        assert_eq!(
            alerts.on_row(&row(
                ReportKindWire::FAULT_RECORDED,
                fault_of(FaultKind::AntennaServoFault),
                9
            )),
            None
        );
        let alert = alerts
            .on_row(&row(
                ReportKindWire::RESPONSE_TAKEN,
                response_of(ResponseKind::DegradeAntennas),
                fault_of(FaultKind::AntennaServoFault),
            ))
            .expect("a released pair owes an alert");
        assert_eq!(alert.severity, Severity::Warning);
    }

    /// Every response that covers the head is the machine stopping, and every
    /// fault the head shares is answered by one.
    #[test]
    fn a_response_that_covers_the_head_is_still_critical() {
        for kind in [
            ResponseKind::SlowStowToRest,
            ResponseKind::MaskedSlowStowToPark,
            ResponseKind::ImmediateAllTorqueOffToRest,
            ResponseKind::ImmediateAllTorqueOffToPark,
        ] {
            let mut alerts = Alerts::new();
            let alert = alerts
                .on_row(&row(ReportKindWire::RESPONSE_TAKEN, response_of(kind), 3))
                .unwrap_or_else(|| panic!("{kind:?} owes an alert"));
            assert_eq!(alert.severity, Severity::Critical, "{kind:?}");
        }
        for kind in [
            FaultKind::HeadObstructed,
            FaultKind::HeadServoFault,
            FaultKind::PositionFeedbackLost,
            FaultKind::MeasuredPoseInvalid,
            FaultKind::BusFailure,
            FaultKind::TorqueOffUnconfirmed,
        ] {
            let mut alerts = Alerts::new();
            let alert = alerts
                .on_row(&row(ReportKindWire::FAULT_RECORDED, fault_of(kind), 1))
                .unwrap_or_else(|| panic!("{kind:?} owes an alert"));
            assert_eq!(alert.severity, Severity::Critical, "{kind:?}");
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

    /// A script held for the phase a maneuver ends in is not news to a room.
    ///
    /// Nothing about the machine is wrong and nothing is being refused: the
    /// sender is owed an answer and will get one on the wake the maneuver ends.
    #[test]
    fn a_held_script_owes_no_alert() {
        let mut alerts = Alerts::new();
        assert_eq!(alerts.on_row(&row(ReportKindWire::SCRIPT_HELD, 7, 5)), None);
        assert_eq!(
            alerts.on_row(&row(ReportKindWire::SCRIPT_HELD, 8, 2)),
            None,
            "and a second one is no louder than the first",
        );
    }

    /// A script refused because a fault was standing the machine down is
    /// counted and left there.
    ///
    /// The refusal warning says a sender and this machine disagree about
    /// something no retry fixes; this is the opposite -- the answer is to ask
    /// again once the machine has rested -- and the fault's own rows have
    /// already carried the critical to whoever is holding the robot.
    #[test]
    fn a_refusal_by_a_faults_ending_owes_no_alert() {
        let mut alerts = Alerts::new();
        assert_eq!(
            alerts.on_row(&row(
                ReportKindWire::SCRIPT_REFUSED,
                7,
                u32::from(RefusalReasonWire::FAULT_ENDING.0)
            )),
            None,
        );
        assert_eq!(alerts.refusals(), 1, "counted all the same");
        let after = alerts
            .on_row(&row(
                ReportKindWire::SCRIPT_REFUSED,
                8,
                u32::from(RefusalReasonWire::BAD_TIMES.0),
            ))
            .expect("an ordinary refusal still owes the warning");
        assert_eq!(after.severity, Severity::Warning);
    }

    #[test]
    fn refusals_are_one_warning_a_run_over_a_count_that_keeps_rising() {
        let mut alerts = Alerts::new();
        let first = alerts
            .on_refusal(&Refusal::NotText, Origin::Remote)
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
        assert_eq!(alerts.on_refusal(&Refusal::NotText, Origin::Remote), None);
    }

    #[test]
    fn a_run_of_stale_drops_is_a_critical_alert_once_the_run_is_long_enough() {
        let mut alerts = Alerts::new();
        // The first drops raise the refusal warning, not the deafness alert.
        for n in 0..STALE_ALERT_RUN - 1 {
            let alert = alerts.on_refusal(&stale(n), Origin::Remote);
            assert!(
                alert.is_none_or(|alert| alert.severity == Severity::Warning),
                "a short run of drops is not deafness"
            );
        }
        let alert = alerts
            .on_refusal(&stale(STALE_ALERT_RUN), Origin::Remote)
            .expect("a run of drops owes an alert");
        assert_eq!(alert.severity, Severity::Critical);
        assert!(alert.title.contains("dropping"), "{}", alert.title);
        // Once. A sender that keeps its numbering keeps producing these.
        assert_eq!(
            alerts.on_refusal(&stale(STALE_ALERT_RUN + 1), Origin::Remote),
            None
        );
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
            let _ = alerts.on_refusal(&stale(n), Origin::Remote);
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
            alert.body.contains(&crate::narrate::row_says(&entry)),
            "{}",
            alert.body
        );
    }

    /// The condition this latch exists for: the pipeline's own scripter
    /// addressed to a name this host does not answer to. Nobody else is
    /// involved, so nothing recovers it.
    #[test]
    fn a_local_refusal_is_one_critical_alert_naming_both_names() {
        let mut alerts = Alerts::new();
        let refusal = Refusal::ForeignPod {
            addressed: "reachy00".to_owned(),
            pod: "kitchen-reachy".to_owned(),
        };
        let alert = alerts
            .on_refusal(&refusal, Origin::Local)
            .expect("a refused local script owes an alert");
        assert_eq!(alert.severity, Severity::Critical);
        assert!(alert.title.contains("its own scripts"), "{}", alert.title);
        assert!(alert.body.contains("reachy00"), "{}", alert.body);
        assert!(alert.body.contains("kitchen-reachy"), "{}", alert.body);
        // Once. The scripter re-authors at the refresh cadence, and the head
        // stays still for all of them.
        assert_eq!(alerts.on_refusal(&refusal, Origin::Local), None);
        assert_eq!(alerts.on_refusal(&Refusal::NotText, Origin::Local), None);
    }

    /// The two conditions are different news and keep separate latches: a local
    /// refusal must not spend the Warning on its way to the Critical, or a
    /// remote sender's garbage afterwards would raise nothing at all.
    #[test]
    fn a_local_refusal_leaves_the_warning_latch_alone() {
        let mut alerts = Alerts::new();
        let critical = alerts
            .on_refusal(&Refusal::NotText, Origin::Local)
            .expect("a refused local script owes an alert");
        assert_eq!(critical.severity, Severity::Critical);
        let warning = alerts
            .on_refusal(&Refusal::NotText, Origin::Remote)
            .expect("a remote refusal owes the warning");
        assert_eq!(warning.severity, Severity::Warning);
        assert_eq!(alerts.refusals(), 2);
    }

    /// A remote sender addressing another machine is the disagreement the intent
    /// channel is expected to carry, and stays the Warning it is today.
    #[test]
    fn a_remote_refusal_is_the_warning_it_has_always_been() {
        let mut alerts = Alerts::new();
        let alert = alerts
            .on_refusal(
                &Refusal::ForeignPod {
                    addressed: "somebody-else".to_owned(),
                    pod: "fixture-reachy".to_owned(),
                },
                Origin::Remote,
            )
            .expect("a refusal owes an alert");
        assert_eq!(alert.severity, Severity::Warning);
        assert!(alert.title.contains("refused"), "{}", alert.title);
    }

    /// Staleness takes today's path whatever the origin: a local script below
    /// the mark is a pipeline that restarted its numbering under a host that
    /// kept its own, which is the two-senders news the Warning carries.
    #[test]
    fn a_local_stale_drop_takes_the_path_a_remote_one_takes() {
        let mut alerts = Alerts::new();
        let first = alerts
            .on_refusal(&stale(1), Origin::Local)
            .expect("the first drop owes an alert");
        assert_eq!(first.severity, Severity::Warning);
        for n in 2..STALE_ALERT_RUN {
            assert_eq!(alerts.on_refusal(&stale(n), Origin::Local), None);
        }
        let deafness = alerts
            .on_refusal(&stale(STALE_ALERT_RUN), Origin::Local)
            .expect("a run of drops owes an alert");
        assert_eq!(deafness.severity, Severity::Critical);
        assert!(deafness.title.contains("dropping"), "{}", deafness.title);
        assert_eq!(alerts.refusals(), STALE_ALERT_RUN);
    }

    /// An accepted script says the sender and this machine still agree about
    /// numbering, which is what the run was counting the absence of.
    #[test]
    fn an_accepted_script_ends_the_run_of_drops() {
        let mut alerts = Alerts::new();
        let _ = alerts.on_refusal(&stale(1), Origin::Remote);
        let _ = alerts.on_refusal(&stale(2), Origin::Remote);
        alerts.accepted();
        assert_eq!(alerts.stale_run(), 0);
        assert_eq!(alerts.on_refusal(&stale(3), Origin::Remote), None);
        assert_eq!(alerts.stale_run(), 1);
    }

    /// Every row the fault-or-park latch fires on, one per arm of
    /// `stops_the_machine` that answers true. Each on its own table, because
    /// the latch is once a run.
    fn stopping_rows() -> Vec<TimelineEntryWire> {
        vec![
            row(ReportKindWire::FAULT_RECORDED, 3, 12),
            row(ReportKindWire::RESPONSE_TAKEN, 2, 3),
            row(ReportKindWire::TORQUE_OFF_UNCONFIRMED, 0, 0),
            row(ReportKindWire::BUS_FAILURE_DECLARED, 0, 0),
            row(ReportKindWire::COMMISSION_FAILED, 0, 0),
            row(ReportKindWire::WINDDOWN_OUTCOME, 1, 1),
            phase(SessionPhaseWire::PARKED),
        ]
    }

    /// The person in front of the robot is who a Critical is for, and a
    /// deployment that can speak has nothing to say without this sentence.
    #[test]
    fn every_critical_the_table_raises_carries_a_sentence() {
        for entry in stopping_rows() {
            let mut alerts = Alerts::new();
            let alert = alerts.on_row(&entry).expect("a stopping row owes an alert");
            assert_eq!(alert.severity, Severity::Critical);
            assert!(alert.spoken.is_some(), "{entry:?}: {alert:?}");
        }

        let mut own = Alerts::new();
        let foreign = own
            .on_refusal(
                &Refusal::ForeignPod {
                    addressed: "reachy00".to_owned(),
                    pod: "kitchen-reachy".to_owned(),
                },
                Origin::Local,
            )
            .expect("a refused local script owes an alert");
        assert_eq!(
            foreign.spoken.as_deref(),
            Some(
                "My head is not moving. My motion scripts are addressed to reachy00, but I \
                 answer to kitchen-reachy."
            ),
        );

        let mut other = Alerts::new();
        let refused = other
            .on_refusal(&Refusal::NotText, Origin::Local)
            .expect("a refused local script owes an alert");
        assert_eq!(
            refused.spoken.as_deref(),
            Some("My head is not moving. My own motion scripts are being refused."),
            "a screen with no names to reconcile says only what happened",
        );

        let mut deaf = Alerts::new();
        let deafness = (1..=STALE_ALERT_RUN)
            .filter_map(|n| deaf.on_refusal(&stale(n), Origin::Local))
            .find(|alert| alert.severity == Severity::Critical)
            .expect("a run of drops owes an alert");
        assert_eq!(
            deafness.spoken.as_deref(),
            Some("My head is dropping every motion script."),
        );
    }

    /// Spoken words interrupt a room. A Warning is what an operator reads
    /// afterwards, and the robot announcing every declined script to whoever is
    /// standing there is how the sentences that matter stop being listened to.
    #[test]
    fn no_warning_the_table_raises_is_spoken() {
        let mut refused = Alerts::new();
        let warning = refused
            .on_refusal(&Refusal::NotText, Origin::Remote)
            .expect("a refusal owes an alert");
        assert_eq!(warning.severity, Severity::Warning);
        assert_eq!(warning.spoken, None);

        let mut session = Alerts::new();
        let declined = session
            .on_row(&row(
                ReportKindWire::SCRIPT_REFUSED,
                1,
                u32::from(RefusalReasonWire::TOO_LONG.0),
            ))
            .expect("a refusal owes an alert");
        assert_eq!(declined.severity, Severity::Warning);
        assert_eq!(declined.spoken, None);

        let mut hole = Alerts::new();
        let lost = hole.narration_hole(4).expect("a hole owes an alert");
        assert_eq!(lost.severity, Severity::Warning);
        assert_eq!(lost.spoken, None);

        let mut stale_once = Alerts::new();
        let below = stale_once
            .on_refusal(&stale(1), Origin::Local)
            .expect("the first drop owes an alert");
        assert_eq!(below.severity, Severity::Warning);
        assert_eq!(below.spoken, None);
    }

    /// The row exists to say de-torquing was *not* confirmed, and stowed with
    /// torque held is this machine's one pinch hazard. What a bystander hears
    /// has to say the same, and no other row's sentence may imply the motors
    /// are off.
    #[test]
    fn the_unconfirmed_row_is_the_only_one_that_warns_a_bystander_off() {
        let mut alerts = Alerts::new();
        let unconfirmed = alerts
            .on_row(&row(ReportKindWire::TORQUE_OFF_UNCONFIRMED, 0, 0))
            .expect("a stopping row owes an alert");
        assert_eq!(
            unconfirmed.spoken.as_deref(),
            Some(
                "My head motion has stopped, and I could not confirm my motors are off. Do not \
                 touch my head."
            ),
        );

        for entry in stopping_rows() {
            if entry.kind() == ReportKindWire::TORQUE_OFF_UNCONFIRMED {
                continue;
            }
            let mut alerts = Alerts::new();
            let alert = alerts.on_row(&entry).expect("a stopping row owes an alert");
            assert_eq!(
                alert.spoken.as_deref(),
                Some("My head motion has stopped."),
                "{entry:?}: nothing about limpness, either way",
            );
        }
    }
}
