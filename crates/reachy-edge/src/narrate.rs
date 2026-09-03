//! Narration: what the edge says happened, one JSON object per line.
//!
//! Two streams of events end up on one operator-facing stream. The session's
//! own timeline rows arrive over the reports socket and are rendered from the
//! report vocabulary; the edge's own drops — a body too big, a script for
//! another machine, a redelivery, something that does not compile — never reach
//! the session at all and are rendered here from the refusal that stopped them.
//! A reader of the stream wants both, and wants to be able to tell them apart,
//! so each line names which it is in `stream`.
//!
//! A line carries the numbers as the wire holds them *and* the sentence they
//! mean. The numbers are what joins against a log; the sentence is what a
//! person holding the robot reads. Where the vocabulary numbers something this
//! build has no name for — a report kind or a refusal reason from a newer
//! build — the line says the number and says that it is unnamed, rather than
//! guessing at a neighbour.
//!
//! Every line carries `at_ns`, so a reader reconstructing an incident joins on
//! a number rather than on the order two files happen to be in. The two streams
//! stamp from two clocks and mean two things by it: on a `timeline` line it is
//! the session's own clock, written when the session appended the row; on an
//! `edge` line it is this machine's wall clock, read when the edge did the
//! thing the line reports. Both are `CLOCK_REALTIME` on the same machine, which
//! is what makes them comparable at all.
//!
//! One builder puts the `edge` envelope together — [`edge_line`] and its
//! variant carrying a line's own fields — and every process narrating on this
//! stream renders through it, because a stream defined by a literal per caller
//! is a stream that grows a field in some of its lines.
//!
//! The text in a line is bounded and stripped of control characters. Most of it
//! is this tree's own words, but not all: a refusal quotes the pod a foreign
//! script was addressed to, and a decode failure quotes what it could not read.
//! That text belongs to whoever sent the body, and a line-oriented stream is
//! exactly where a newline in it would forge a line that reads like ours.

use clockwork_rs::SyncTime;
use serde_json::{Value, json};

use brenn_reachy__cogs__session_clk_rs::{SessionPhase, SessionPhaseWire};
use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointFlagsWire};
use brenn_reachy__motion__reports_clk_rs::{
    RefusalReason, RefusalReasonWire, ReportKind, ReportKindWire,
};
use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;

use crate::alerts::Severity;
use crate::intake::{Origin, Refusal};

/// How much of a quoted text a line carries.
///
/// The unabridged text is not kept anywhere else, which is a deliberate
/// narrowing: this stream is the operator surface, and a body long enough to
/// bury a terminal is a body whose first two hundred characters already say
/// what is wrong with it.
const TEXT_LIMIT: usize = 200;

/// Text made safe to put on a line-oriented stream: bounded, and with control
/// characters spent as spaces.
fn one_line(text: &str) -> String {
    let mut clean: String = text
        .chars()
        .take(TEXT_LIMIT)
        .map(|c| if c.is_control() { ' ' } else { c })
        .collect();
    if text.chars().count() > TEXT_LIMIT {
        clean.push('…');
    }
    clean
}

/// One row of the session's story, as a line.
#[must_use]
pub fn timeline_line(row: &TimelineEntryWire) -> String {
    line(&json!({
        "stream": "timeline",
        "at_ns": row.time().as_nanos(),
        "kind": kind_name(row.kind()),
        "a": row.a(),
        "b": row.b(),
        "detail": finite(row.detail()),
        "says": one_line(&row_says(row)),
    }))
}

/// One event of this process's own, as a line on the edge's stream.
///
/// The envelope every `edge` line carries — the stream it is on, the machine's
/// clock at the event, what kind of event it was, and the sentence a person
/// reads — built in one place. Every process that narrates on this stream goes
/// through here rather than spelling the object itself, so a field the envelope
/// grows reaches every line, and so `says` is bounded and stripped of control
/// characters wherever the text came from.
#[must_use]
pub fn edge_line(kind: &str, at: SyncTime, says: &str) -> String {
    edge_line_with(kind, at, says, &[])
}

/// The same envelope, carrying the fields that are this line's own.
///
/// A field here is what a reader joins on — a script's id, a sender's sequence
/// number — beside the sentence that spells it out. An envelope field named
/// again in `fields` overwrites the envelope's, which is a caller's mistake and
/// not something worth a refusal on a narration path.
#[must_use]
pub fn edge_line_with(kind: &str, at: SyncTime, says: &str, fields: &[(&str, Value)]) -> String {
    let mut value = json!({
        "stream": "edge",
        "at_ns": at.as_nanos(),
        "kind": kind,
        "says": one_line(says),
    });
    let object = value
        .as_object_mut()
        .expect("a JSON object, built as one just above");
    for (name, field) in fields {
        object.insert((*name).to_string(), field.clone());
    }
    line(&value)
}

/// One body the edge dropped, as a line.
///
/// `at` is this machine's clock at the drop — the same instant the body would
/// have been stamped with had it been accepted.
///
/// `origin` is on the line because it is what separates a sender disagreeing
/// with this machine from this machine disagreeing with itself, and a reader
/// after the fact cannot recover it from anything else the line carries.
#[must_use]
pub fn refusal_line(refusal: &Refusal, origin: Origin, at: SyncTime) -> String {
    edge_line_with(
        refusal.kind(),
        at,
        &refusal.to_string(),
        &[("origin", json!(origin_word(origin)))],
    )
}

/// The story went backwards: the process telling it restarted.
#[must_use]
pub fn restart_line(narrated: u64, at: SyncTime) -> String {
    edge_line(
        "story_restarted",
        at,
        &format!(
            "the session's story went backwards after {narrated} row(s): the control process \
             restarted, and what follows is the new one from its beginning"
        ),
    )
}

/// Rows the session narrated that no datagram carried here.
#[must_use]
pub fn lost_line(lost: u64, at: SyncTime) -> String {
    edge_line_with(
        "story_rows_lost",
        at,
        &format!(
            "{lost} row(s) of the session's story fell off its ring before a datagram carried \
             them: the narration has a hole, and what they classified travelled on the channels \
             that raised it"
        ),
        &[("lost", json!(lost))],
    )
}

/// A JSON object as one line. Compact, because the reader is `grep` and a log
/// shipper before it is a person.
fn line(value: &Value) -> String {
    value.to_string()
}

/// A measurement as JSON can carry it.
///
/// A report's `detail` is carried bit for bit and a non-finite number is a
/// legal one — a story dropped for an unreadable measurement loses the part
/// that was readable. JSON has no infinity, so it goes as its own spelling
/// rather than as `null`, which would read as a field nobody set.
fn finite(detail: f64) -> Value {
    if detail.is_finite() {
        json!(detail)
    } else {
        json!(format!("{detail}"))
    }
}

/// How a kind this build has no word for is spelled, ahead of its number.
///
/// Exported because the words are read outside this tree: `//cogs:speech_run_report`
/// separates the kinds it can count from the ones it can only carry by this
/// prefix, and a spelling restated there would drift without either side
/// noticing.
pub const UNKNOWN_KIND_PREFIX: &str = "kind_";

/// How an alert's loudness is spelled on the wire and in a log.
///
/// The edge's two words, spelled once here. Here and not beside the loop that
/// raises an alert: this module is where the vocabulary of every line this
/// crate writes lives, so a reader joining on one of these words has one place
/// to look for it.
#[must_use]
pub const fn severity_word(severity: Severity) -> &'static str {
    match severity {
        Severity::Warning => "warning",
        Severity::Critical => "critical",
    }
}

/// How a body's origin is spelled in a log.
///
/// The edge's two words, spelled once here, so the analyzer that reads a
/// refusal line joins on the same string the host writes.
#[must_use]
pub const fn origin_word(origin: Origin) -> &'static str {
    match origin {
        Origin::Local => "local",
        Origin::Remote => "remote",
    }
}

/// A report kind's own word, or the number where this build has no word for it.
fn kind_name(kind: ReportKindWire) -> String {
    match kind.to_known() {
        Some(known) => row_word(known).to_owned(),
        None => format!("{UNKNOWN_KIND_PREFIX}{}", kind.0),
    }
}

/// The wire spelling of a report kind.
///
/// Written out rather than derived from the Rust identifier: the identifier is
/// a name a rename would rewrite this stream with, and a stream whose spelling
/// drifts stops joining against the runs that came before it, silently. The
/// `match` is wildcard-free, so a kind the vocabulary grows is a compile error
/// here rather than a line that says nothing.
///
/// Public because a reader of this stream that counts rows by kind must count
/// the spelling this writes rather than a copy of it: `//cogs:speech_run_report`
/// asks how many scripts the session accepted by looking for one of these
/// words.
#[must_use]
pub const fn row_word(kind: ReportKind) -> &'static str {
    match kind {
        ReportKind::None => "none",
        ReportKind::PhaseChanged => "phase_changed",
        ReportKind::ScriptAccepted => "script_accepted",
        ReportKind::ScriptRefused => "script_refused",
        ReportKind::FaultRecorded => "fault_recorded",
        ReportKind::ResponseTaken => "response_taken",
        ReportKind::WinddownOutcome => "winddown_outcome",
        ReportKind::TorqueOffConfirmed => "torque_off_confirmed",
        ReportKind::TorqueOffUnconfirmed => "torque_off_unconfirmed",
        ReportKind::BusFailureDeclared => "bus_failure_declared",
        ReportKind::SessionEnded => "session_ended",
        ReportKind::AuxGaveUp => "aux_gave_up",
        ReportKind::SchedulePublished => "schedule_published",
        ReportKind::DegradeReleased => "degrade_released",
        ReportKind::CommissionFailed => "commission_failed",
        ReportKind::ScriptReplaced => "script_replaced",
        ReportKind::ScriptHeld => "script_held",
    }
}

/// What a row means, in a sentence.
///
/// The two numbers mean something different per kind, and this is where that
/// table is spelled — the same table the report vocabulary states beside each
/// kind. A reader of a line should not have to hold the vocabulary in their
/// head to know whether `b` is a servo, a phase or a reason.
///
/// Public for the same reason [`row_word`] is: a reader of this stream that
/// quotes a row back to an operator quotes the sentence the edge would have
/// written for it rather than keeping a second table of what the two numbers
/// mean. `//cogs:speech_run_report` says what the session refused a script for
/// by asking here.
#[must_use]
pub fn row_says(row: &TimelineEntryWire) -> String {
    let (a, b, detail) = (row.a(), row.b(), row.detail());
    let Some(kind) = row.kind().to_known() else {
        return format!(
            "report kind {}, which this build does not name: {a}, {b}, {detail}",
            row.kind().0
        );
    };
    match kind {
        ReportKind::None => "an unwritten row, which no sender produces".to_owned(),
        ReportKind::PhaseChanged => {
            format!("phase {} entered from {}", phase_name(a), phase_name(b))
        }
        ReportKind::ScriptAccepted => {
            format!("script {a} accepted: {b} step(s), schedule epoch {detail}")
        }
        ReportKind::ScriptRefused => format!("script {a} refused: {}", refusal_reason(b)),
        ReportKind::FaultRecorded => {
            format!("fault {a} recorded on servo {b}, magnitude {detail}")
        }
        ReportKind::ResponseTaken => format!("response {a} taken, for fault {b}"),
        ReportKind::WinddownOutcome => format!(
            "wind-down concluded with outcome {a}{}, {detail:.3} s of its clock left",
            if b == 0 { "" } else { ", park-class" }
        ),
        ReportKind::TorqueOffConfirmed => "every row confirmed torque off".to_owned(),
        ReportKind::TorqueOffUnconfirmed => {
            format!("torque off unconfirmed: {a} row(s) unread after {detail:.3} s")
        }
        ReportKind::BusFailureDeclared => {
            format!("the bus was declared failed: {detail:.3} s since a fresh sample")
        }
        ReportKind::SessionEnded => format!(
            "the session ended at rest: script {a}, servo set {b:#x} unread at the release, \
             worst deviation from stow {detail:.4} rad"
        ),
        ReportKind::AuxGaveUp => {
            format!("aux transaction {a} on servo {b} gave up after {detail:.3} s")
        }
        ReportKind::SchedulePublished => format!("schedule epoch {a} published: {b} step(s)"),
        ReportKind::DegradeReleased => {
            format!(
                "a group de-torque for response {a} released {}",
                joint_rows(b)
            )
        }
        ReportKind::CommissionFailed => format!(
            "the survey refused the machine: failure kind {a} at servo {b}, headline {detail}"
        ),
        ReportKind::ScriptReplaced => format!(
            "script {a} replaced the running schedule under epoch {b}, asking for {detail} step(s)"
        ),
        ReportKind::ScriptHeld => format!(
            "script {a} held in phase {}, for the phase this maneuver ends in",
            phase_name(b)
        ),
    }
}

/// The nine bus rows and the joint vocabulary's word for each, in bus order.
///
/// The words are the vocabulary's own, written out rather than derived, for the
/// same reason [`row_word`]'s are: a reader joining this stream against a
/// recorded log matches on the spelling, and a spelling that follows a Rust
/// identifier drifts with a rename.
///
/// A second rendering of a joint set, and knowingly: `reachy_motion::joints::
/// flags::Names` spells the same set in prose ("right antenna") for the reports
/// the control process writes, and this edge links the vocabulary modules and
/// no Rust crate of this project's, which is what keeps an alert stream
/// buildable from the wire alone. So the two spellings differ on purpose --
/// this one is the recorded word, that one is the sentence -- and neither is
/// derived from the other.
const ROW_NAMES: [(JointFlags, &str); 9] = [
    (JointFlags::BODY_YAW, "body_yaw"),
    (JointFlags::LEG_0, "leg_0"),
    (JointFlags::LEG_1, "leg_1"),
    (JointFlags::LEG_2, "leg_2"),
    (JointFlags::LEG_3, "leg_3"),
    (JointFlags::LEG_4, "leg_4"),
    (JointFlags::LEG_5, "leg_5"),
    (JointFlags::ANTENNA_RIGHT, "antenna_right"),
    (JointFlags::ANTENNA_LEFT, "antenna_left"),
];

/// A bus row the machine grows is a compile error here rather than a row this
/// stream renders as nothing. The declared values carry the empty set as well
/// as the nine rows.
const _: () = assert!(ROW_NAMES.len() + 1 == JointFlags::VARIANTS.len());

/// A set of bus rows by the joints in it, or by number where the set holds a
/// bit this build does not name.
///
/// A number is what a reader joins on, and it is right there in the row's own
/// `b`; what a person holding the robot needs is which servos went limp.
fn joint_rows(rows: u32) -> String {
    let Some(set) = u16::try_from(rows)
        .ok()
        .map(JointFlagsWire)
        .and_then(JointFlagsWire::to_known)
    else {
        return format!("servo set {rows:#x}, which this build does not name");
    };
    let named: Vec<&str> = ROW_NAMES
        .iter()
        .filter(|(row, _)| set.contains(*row))
        .map(|(_, word)| *word)
        .collect();
    if named.is_empty() {
        return "no rows at all".to_owned();
    }
    named.join(", ")
}

/// A phase by its own word, or by number where this build has none for it.
fn phase_name(phase: u32) -> String {
    let wire = SessionPhaseWire(u8::try_from(phase).unwrap_or(u8::MAX));
    match wire.to_known() {
        Some(SessionPhase::Starting) => "starting".to_owned(),
        Some(SessionPhase::Resting) => "resting".to_owned(),
        Some(SessionPhase::Engaging) => "engaging".to_owned(),
        Some(SessionPhase::Active) => "active".to_owned(),
        Some(SessionPhase::WindingDown) => "winding_down".to_owned(),
        Some(SessionPhase::Stopping) => "stopping".to_owned(),
        Some(SessionPhase::Parked) => "parked".to_owned(),
        None => format!("phase {phase}, which this build does not name"),
    }
}

/// A refusal reason as a sentence, or by number where this build has none.
pub(crate) fn refusal_reason(reason: u32) -> String {
    let wire = RefusalReasonWire(u8::try_from(reason).unwrap_or(u8::MAX));
    match wire.to_known() {
        Some(RefusalReason::None) => "no refusal at all".to_owned(),
        Some(RefusalReason::Parked) => "the machine is parked".to_owned(),
        Some(RefusalReason::NotResting) => "the machine was busy with a session".to_owned(),
        Some(RefusalReason::TooManySteps) => "more steps than a schedule holds".to_owned(),
        Some(RefusalReason::TooManyOverlays) => {
            "more overlay windows than a schedule holds".to_owned()
        }
        Some(RefusalReason::BadTimes) => "times that are not a timeline".to_owned(),
        Some(RefusalReason::UnknownMotion) => "a motion index no library could hold".to_owned(),
        Some(RefusalReason::Undecodable) => "a datagram that is not a script".to_owned(),
        Some(RefusalReason::Stale) => "a number no higher than the running engagement's".to_owned(),
        Some(RefusalReason::TooLong) => "a schedule reaching past the session's horizon".to_owned(),
        Some(RefusalReason::FaultEnding) => {
            "the machine was being stood down by a fault".to_owned()
        }
        None => format!("reason {reason}, which this build does not name"),
    }
}

#[cfg(test)]
mod tests {
    use clockwork_rs::SyncTime;
    use serde_json::{Value, json};

    use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
    use brenn_reachy__motion__joints_clk_rs::JointFlagsWire;
    use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKindWire};
    use brenn_reachy__motion__timeline_clk_rs::TimelineEntryWire;

    use super::{
        TEXT_LIMIT, edge_line, edge_line_with, lost_line, refusal_line, restart_line, timeline_line,
    };
    use crate::intake::{Origin, Refusal};

    /// The host's clock at the moment an edge line reports, distinct from the
    /// 42 a fixture row carries, so a line stamped from the wrong side shows.
    fn at() -> SyncTime {
        SyncTime::from_nanos(1_700_000_000_000_000_000)
    }

    fn row(kind: ReportKindWire, a: u32, b: u32, detail: f64) -> TimelineEntryWire {
        let mut entry = TimelineEntryWire::new();
        entry.set_time(SyncTime::from_nanos(42));
        entry.set_kind(kind);
        entry.set_a(a);
        entry.set_b(b);
        entry.set_detail(detail);
        entry
    }

    #[test]
    fn every_edge_line_carries_the_same_envelope() {
        let line = parsed(&edge_line(
            "started",
            at(),
            "the host answers for this machine",
        ));
        assert_eq!(line["stream"], "edge");
        assert_eq!(line["kind"], "started");
        assert_eq!(line["at_ns"], at().as_nanos());
        assert_eq!(line["says"], "the host answers for this machine");
    }

    #[test]
    fn a_line_carries_the_fields_that_are_its_own_beside_the_envelope() {
        let line = parsed(&edge_line_with(
            "unsent",
            at(),
            "script 7 never left",
            &[("script_id", json!(7)), ("pod", json!("kitchen-reachy"))],
        ));
        assert_eq!(line["stream"], "edge");
        assert_eq!(line["kind"], "unsent");
        assert_eq!(line["script_id"], 7);
        assert_eq!(line["pod"], "kitchen-reachy");
    }

    /// The documented rule for a collision, held by a case: a caller's field
    /// wins over the envelope's. It holds only because the insert loop runs
    /// after the literal, and a builder assembled the other way round would
    /// invert it silently -- with the doc comment left saying something untrue
    /// about a stream other tools parse.
    #[test]
    fn a_field_a_caller_names_again_wins_over_the_envelope_s() {
        let line = parsed(&edge_line_with(
            "started",
            at(),
            "the host answers for this machine",
            &[("kind", json!("overridden"))],
        ));
        assert_eq!(line["kind"], "overridden");
        assert_eq!(line["stream"], "edge", "the rest of the envelope stands");
    }

    /// The envelope bounds and cleans `says` wherever the text came from: the
    /// reason the builder is shared at all is that a caller holding sender text
    /// must not be the one deciding this.
    #[test]
    fn the_envelope_bounds_and_cleans_what_a_caller_hands_it() {
        let shouted = format!("a\nb{}", "x".repeat(TEXT_LIMIT));
        let line = parsed(&edge_line("started", at(), &shouted));
        let says = line["says"].as_str().expect("a sentence");
        assert!(!says.contains('\n'), "{says}");
        assert_eq!(
            says.chars().count(),
            TEXT_LIMIT + 1,
            "bounded, with the mark"
        );
    }

    /// One line, parsed back.
    fn parsed(line: &str) -> Value {
        assert!(!line.contains('\n'), "a line holds no newline: {line}");
        serde_json::from_str(line).expect("a line is one JSON object")
    }

    #[test]
    fn a_row_carries_its_numbers_and_its_sentence() {
        let value = parsed(&timeline_line(&row(
            ReportKindWire::PHASE_CHANGED,
            u32::from(SessionPhaseWire::ACTIVE.0),
            u32::from(SessionPhaseWire::ENGAGING.0),
            0.0,
        )));
        assert_eq!(value["stream"], "timeline");
        assert_eq!(value["at_ns"], 42);
        assert_eq!(value["kind"], "phase_changed");
        assert_eq!(value["a"], u32::from(SessionPhaseWire::ACTIVE.0));
        assert_eq!(value["b"], u32::from(SessionPhaseWire::ENGAGING.0));
        assert_eq!(value["says"], "phase active entered from engaging");
    }

    #[test]
    fn a_refusal_row_spells_the_reason_out() {
        let value = parsed(&timeline_line(&row(
            ReportKindWire::SCRIPT_REFUSED,
            7,
            u32::from(RefusalReasonWire::STALE.0),
            0.0,
        )));
        assert_eq!(value["kind"], "script_refused");
        assert_eq!(
            value["says"],
            "script 7 refused: a number no higher than the running engagement's"
        );
    }

    /// A hold names itself and names the phase in words.
    ///
    /// Both halves are read by something outside this process: the word is what
    /// a run's own assertions look for in the stream, and the phase is a number
    /// on the wire that means nothing to a reader who does not hold this
    /// vocabulary. A row rendering `b` as the number it is would be a hold
    /// nobody can place.
    #[test]
    fn a_hold_names_the_phase_it_found_in_words() {
        let value = parsed(&timeline_line(&row(
            ReportKindWire::SCRIPT_HELD,
            11,
            u32::from(SessionPhaseWire::STOPPING.0),
            0.0,
        )));
        assert_eq!(value["kind"], "script_held");
        assert_eq!(
            value["says"],
            "script 11 held in phase stopping, for the phase this maneuver ends in"
        );
    }

    /// The reason a fault's ending refuses says what the machine was doing,
    /// which is the whole of what tells a reader to ask again later.
    #[test]
    fn a_refusal_by_a_faults_ending_says_what_stood_the_machine_down() {
        let value = parsed(&timeline_line(&row(
            ReportKindWire::SCRIPT_REFUSED,
            9,
            u32::from(RefusalReasonWire::FAULT_ENDING.0),
            0.0,
        )));
        assert_eq!(
            value["says"],
            "script 9 refused: the machine was being stood down by a fault"
        );
    }

    /// The row a released antenna pair produces. The number stays in `b`,
    /// where a reader joins on it; the sentence says which servos went limp.
    #[test]
    fn a_released_set_says_which_joints_it_let_go() {
        let value = parsed(&timeline_line(&row(
            ReportKindWire::DEGRADE_RELEASED,
            3,
            u32::from((JointFlagsWire::ANTENNA_RIGHT | JointFlagsWire::ANTENNA_LEFT).0),
            0.0,
        )));
        assert_eq!(value["kind"], "degrade_released");
        assert_eq!(value["b"], 384);
        assert_eq!(
            value["says"],
            "a group de-torque for response 3 released antenna_right, antenna_left"
        );
    }

    /// A set holding a bit this build has no row for. The number is what the
    /// two builds have in common, so the sentence carries it rather than
    /// naming the rows it did recognise and quietly dropping the rest.
    #[test]
    fn a_released_set_this_build_cannot_place_says_so() {
        let value = parsed(&timeline_line(&row(
            ReportKindWire::DEGRADE_RELEASED,
            3,
            1024,
            0.0,
        )));
        assert!(
            value["says"]
                .as_str()
                .expect("a sentence")
                .contains("does not name"),
            "{value}"
        );
    }

    /// A build reading a log written by a newer one. The number is what the two
    /// have in common, so the line carries it rather than guessing at a
    /// neighbouring name.
    #[test]
    fn a_kind_this_build_does_not_know_says_so() {
        let value = parsed(&timeline_line(&row(ReportKindWire(200), 1, 2, 0.0)));
        assert_eq!(value["kind"], "kind_200");
        assert!(
            value["says"]
                .as_str()
                .expect("a sentence")
                .contains("does not name"),
            "{value}"
        );
    }

    #[test]
    fn a_reason_this_build_does_not_know_says_so() {
        let value = parsed(&timeline_line(&row(
            ReportKindWire::SCRIPT_REFUSED,
            1,
            250,
            0.0,
        )));
        assert!(
            value["says"]
                .as_str()
                .expect("a sentence")
                .contains("reason 250"),
            "{value}"
        );
    }

    /// A measurement JSON has no number for. It goes as its own spelling: a
    /// `null` here would read as a field the session never set.
    #[test]
    fn a_non_finite_measurement_is_carried_as_its_spelling() {
        let value = parsed(&timeline_line(&row(
            ReportKindWire::FAULT_RECORDED,
            1,
            2,
            f64::INFINITY,
        )));
        assert_eq!(value["detail"], "inf");
        let nan = parsed(&timeline_line(&row(
            ReportKindWire::FAULT_RECORDED,
            1,
            2,
            f64::NAN,
        )));
        assert_eq!(nan["detail"], "NaN");
    }

    #[test]
    fn an_edge_drop_names_its_screen() {
        let value = parsed(&refusal_line(
            &Refusal::Stale {
                seq: 4,
                accepted: 9,
            },
            Origin::Remote,
            at(),
        ));
        assert_eq!(value["origin"], "remote");
        assert_eq!(value["stream"], "edge");
        assert_eq!(value["at_ns"], at().as_nanos());
        assert_eq!(value["kind"], "stale");
        assert!(
            value["says"]
                .as_str()
                .expect("a sentence")
                .contains("numbered 4"),
            "{value}"
        );
    }

    /// The text a refusal quotes is the sender's, and this stream is
    /// line-oriented: a newline in it would forge a line that reads like one of
    /// ours, and an unbounded one would bury the terminal.
    #[test]
    fn a_refusal_quoting_a_sender_is_bounded_and_holds_no_control_characters() {
        let addressed = format!("kitchen\nreachy: forged {}", "x".repeat(400));
        let value = parsed(&refusal_line(
            &Refusal::ForeignPod {
                addressed,
                pod: "reachy00".to_owned(),
            },
            Origin::Local,
            at(),
        ));
        assert_eq!(value["origin"], "local");
        let says = value["says"].as_str().expect("a sentence");
        assert!(!says.contains('\n'), "{says}");
        assert!(says.chars().count() <= TEXT_LIMIT + 1, "{says}");
        assert!(says.ends_with('…'), "{says}");
    }

    #[test]
    fn a_restart_and_a_hole_each_have_a_line() {
        let restart = parsed(&restart_line(12, at()));
        assert_eq!(restart["kind"], "story_restarted");
        assert_eq!(restart["at_ns"], at().as_nanos());
        assert!(
            restart["says"]
                .as_str()
                .expect("a sentence")
                .contains("12 row(s)"),
            "{restart}"
        );

        let lost = parsed(&lost_line(3, at()));
        assert_eq!(lost["kind"], "story_rows_lost");
        assert_eq!(lost["at_ns"], at().as_nanos());
        assert_eq!(lost["lost"], 3);
    }
}
