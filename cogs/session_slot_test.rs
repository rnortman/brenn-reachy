//! What the session's slot mapping has to get right, and the numbers its
//! vocabularies are pinned to.
//!
//! Two kinds of case. The crossings are round trips over values with nothing
//! round about them, so a mapping that dropped or transposed a field shows up as
//! a different number rather than as a coincidence of zeroes. The pins are
//! wildcard-free tables of the discriminants a recorded story and a later build
//! have in common -- the report kinds and the session phases -- which nothing in
//! either language holds still on its own.

use brenn_reachy__cogs__session_clk_rs::{SessionPhaseWire, SessionStateWire};
use brenn_reachy__driver__health_clk_rs::{AuxStatus, AuxStatusWire as AuxStatusSlot};
use brenn_reachy__hardware__dynamixel__registers_clk_rs::{ValueShape, ValueShapeWire};
use brenn_reachy__motion__bus_txn_clk_rs::{AuxOpKind, AuxOpKindWire};
use brenn_reachy__motion__joints_clk_rs::JointFlagsWire;
use brenn_reachy__motion__reports_clk_rs::{ReportKind, ReportKindWire as ReportKindSlot};
use brenn_reachy__motion__seq_clk_rs::SeqKindWire;
use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, TimelineWire};
use clockwork_rs::SyncTime;
use session_slots::{
    SessionSlotError, TIMELINE_LEN, clear_timeline, clear_timeline_entry, held_reports,
    push_report, report_row,
};

/// An instant far from zero, so a dropped or defaulted timestamp reads as
/// obviously wrong rather than as a plausible small number.
const T0_NS: i64 = 1_700_000_000_000_000_000;

/// A cleared row narrates nothing, and the numbers go with the kind: a slot is
/// reused memory, and a later reader must not find the tail of an older story
/// under a kind that was cleared.
#[test]
fn a_cleared_row_holds_no_report_and_no_numbers() {
    let mut state = SessionStateWire::new();
    push_numbered(&mut state, 7).expect("an empty ring takes a report");

    let row = &mut state.timeline_mut()[0];
    clear_timeline_entry(row);
    assert_eq!(row.kind(), ReportKindSlot::NONE);
    assert_eq!(row.a(), 0);
    assert_eq!(row.b(), 0);
    assert_eq!(row.time().as_nanos(), 0);
    assert_eq!(
        report_row(&state, 0),
        Err(SessionSlotError::NoSuchReportKind(0)),
        "and the row the head still calls a report is refused"
    );
}

/// A kind this build cannot narrate is refused where the ring hands the row out.
/// The zero an unwritten row holds is refused with it, which is what makes an
/// empty ring readable rather than a ring of reports about nothing.
#[test]
fn a_report_kind_this_build_cannot_narrate_is_refused() {
    let mut state = SessionStateWire::new();
    push_numbered(&mut state, 1).expect("an empty ring takes a report");
    state.timeline_mut()[0].set_kind(ReportKindSlot(99));

    assert_eq!(
        report_row(&state, 0),
        Err(SessionSlotError::NoSuchReportKind(99))
    );
}

/// What an unwritten slot says about a transaction it does not have.
///
/// Three enumerations key the aux fields, and each spends its zero on purpose:
/// a slot nothing wrote asks for no transaction and carries no value, while an
/// outcome's zero is `ok`, because a driver reporting an outcome ran one. A
/// reader of a freshly cleared slot depends on all three.
#[test]
fn an_unwritten_aux_slot_asks_for_nothing_and_carries_no_value() {
    assert_eq!(AuxOpKindWire::NONE.0, 0);
    assert_eq!(ValueShapeWire::NONE.0, 0, "a ping carries no value");
    assert_eq!(AuxStatusSlot::OK.0, 0, "a driver's outcome always happened");
}

/// The phase numbering is the session's own and mirrors no Rust type, so what
/// there is to pin is the one number a reader of an unwritten slot depends on:
/// a session that has not run yet is starting, which is the phase that
/// commissions rather than the one that commands.
#[test]
fn an_unwritten_phase_is_the_one_that_has_not_begun() {
    assert_eq!(SessionPhaseWire::STARTING.0, 0);

    // And the phases are the machine's life in the order it lives them, each
    // with its own number -- a pair sharing one would make a phase change
    // invisible to the report that narrates it.
    let phases = [
        SessionPhaseWire::STARTING,
        SessionPhaseWire::RESTING,
        SessionPhaseWire::ENGAGING,
        SessionPhaseWire::ACTIVE,
        SessionPhaseWire::WINDING_DOWN,
        SessionPhaseWire::STOPPING,
        SessionPhaseWire::PARKED,
    ];
    for (nth, phase) in phases.iter().enumerate() {
        assert_eq!(usize::from(phase.0), nth, "{phase:?}");
    }
}

reachy_motion::vocab_numbering! {
    /// The report numbering is the one written down here.
    ///
    /// The strongest append-only claim in the tree: a recorded timeline row and
    /// the build reading it have nothing in common but the number, so a kind
    /// inserted among these renarrates every story already on disk. Nothing in
    /// either language says so, which is why it is said here.
    the_report_numbering_is_the_one_written_down:
        ReportKind as ReportKindSlot, past the end 15 {
        ReportKind::None => 0,
        ReportKind::PhaseChanged => 1,
        ReportKind::ScriptAccepted => 2,
        ReportKind::ScriptRefused => 3,
        ReportKind::FaultRecorded => 4,
        ReportKind::ResponseTaken => 5,
        ReportKind::WinddownOutcome => 6,
        ReportKind::TorqueOffConfirmed => 7,
        ReportKind::TorqueOffUnconfirmed => 8,
        ReportKind::BusFailureDeclared => 9,
        ReportKind::SessionEnded => 10,
        ReportKind::AuxGaveUp => 11,
        ReportKind::SchedulePublished => 12,
        ReportKind::DegradeReleased => 13,
        ReportKind::CommissionFailed => 14,
    }
}

reachy_motion::vocab_numbering! {
    /// The aux operation numbering is the one written down here.
    ///
    /// What a session asks a driver to do sits in a slot and, at the process
    /// edge, in a transaction; the number is all the two ends have in common.
    the_aux_operation_numbering_is_the_one_written_down:
        AuxOpKind as AuxOpKindWire, past the end 5 {
        AuxOpKind::None => 0,
        AuxOpKind::Ping => 1,
        AuxOpKind::ReadReg => 2,
        AuxOpKind::WriteRegVerified => 3,
        AuxOpKind::WriteReg => 4,
    }
}

reachy_motion::vocab_numbering! {
    /// The aux outcome numbering is the one written down here.
    ///
    /// It crosses the process edge in an outcome datagram, whose golden vector
    /// pins one of these numbers; the rest are pinned here. The zero is `ok`,
    /// which is a number like any other.
    the_aux_status_numbering_is_the_one_written_down:
        AuxStatus as AuxStatusSlot, past the end 8 {
        AuxStatus::Ok => 0,
        AuxStatus::Timeout => 1,
        AuxStatus::DecodeError => 2,
        AuxStatus::WireError => 3,
        AuxStatus::Refused => 4,
        AuxStatus::ServoError => 5,
        AuxStatus::VerifyMismatch => 6,
        AuxStatus::Busy => 7,
    }
}

reachy_motion::vocab_numbering! {
    /// The value-shape numbering is the one written down here.
    ///
    /// It says how eight bytes of a transaction or an outcome are to be read, on
    /// both sides of the process edge: a shape renumbered reads a voltage as an
    /// angle.
    the_value_shape_numbering_is_the_one_written_down:
        ValueShape as ValueShapeWire, past the end 8 {
        ValueShape::None => 0,
        ValueShape::U8 => 1,
        ValueShape::U16 => 2,
        ValueShape::U32 => 3,
        ValueShape::I32 => 4,
        ValueShape::Radians => 5,
        ValueShape::Volts => 6,
        ValueShape::Gains => 7,
    }
}

/// Write a report distinct in every field into `row`, keyed by `n` so a story
/// read out of order reads as the wrong numbers rather than as a coincidence.
fn write_numbered(row: &mut TimelineEntryWire, n: u32) {
    row.set_kind(ReportKindSlot::SCHEDULE_PUBLISHED);
    row.set_time(SyncTime::from_nanos(T0_NS + i64::from(n)));
    row.set_a(n);
    row.set_b(n.wrapping_mul(0x0001_0001));
    row.set_detail(f64::from(n) * 0.5);
}

/// Append the `n`th numbered report, answering what the append cost.
fn push_numbered(state: &mut SessionStateWire, n: u32) -> Result<bool, SessionSlotError> {
    push_report(state, |row| write_numbered(row, n))
}

/// Which numbered report this row holds, every field asserted: a row that had
/// been half overwritten would answer with the right number and the wrong story.
fn numbered(row: &TimelineEntryWire) -> u32 {
    let n = row.a();
    assert_eq!(row.kind(), ReportKindSlot::SCHEDULE_PUBLISHED, "{n}");
    assert_eq!(row.time().as_nanos(), T0_NS + i64::from(n), "{n}");
    assert_eq!(row.b(), n.wrapping_mul(0x0001_0001), "{n}");
    assert_eq!(row.detail(), f64::from(n) * 0.5, "{n}");
    n
}

/// The whole story the slot holds, oldest first, every row's fields asserted.
fn story(state: &SessionStateWire) -> Result<Vec<u32>, SessionSlotError> {
    let mut told = Vec::new();
    for nth in 0..TIMELINE_LEN {
        match report_row(state, nth)? {
            Some((row, _)) => told.push(numbered(row)),
            None => break,
        }
    }
    Ok(told)
}

#[test]
fn a_report_appended_to_the_timeline_comes_back_out_of_it() {
    let mut state = SessionStateWire::new();
    assert_eq!(held_reports(&state), Ok(0));
    assert_eq!(story(&state), Ok(vec![]));

    assert_eq!(
        push_numbered(&mut state, 9),
        Ok(false),
        "a ring with room drops nothing"
    );
    assert_eq!(held_reports(&state), Ok(1));
    assert_eq!(story(&state), Ok(vec![9]));
    assert_eq!(
        story(&state),
        Ok(vec![9]),
        "a story read twice is the same story: nothing is drained by reading it"
    );
}

#[test]
fn the_story_is_in_the_order_the_reports_arrived() {
    let mut state = SessionStateWire::new();
    for n in 0..8 {
        assert_eq!(push_numbered(&mut state, n), Ok(false));
    }
    assert_eq!(held_reports(&state), Ok(8));
    assert_eq!(story(&state), Ok((0..8).collect()));
    assert_eq!(
        state.timeline_dropped(),
        0,
        "a story shorter than the ring loses nothing",
    );
}

/// The ring holds its own length and the append past that drops the oldest row
/// rather than refusing the newest, counting what it cost.
#[test]
fn an_append_past_the_ring_drops_the_oldest_row_and_counts_it() {
    let mut state = SessionStateWire::new();
    for n in 0..u32::from(TIMELINE_LEN) {
        assert_eq!(
            push_numbered(&mut state, n),
            Ok(false),
            "the ring is not full until it holds its own length"
        );
    }
    assert_eq!(held_reports(&state), Ok(TIMELINE_LEN));
    assert_eq!(story(&state), Ok((0..u32::from(TIMELINE_LEN)).collect()));

    assert_eq!(
        push_numbered(&mut state, TIMELINE_LEN.into()),
        Ok(true),
        "and the one past it says what it cost"
    );
    assert_eq!(
        held_reports(&state),
        Ok(TIMELINE_LEN),
        "the ring still holds exactly its length"
    );
    assert_eq!(
        story(&state),
        Ok((1..=u32::from(TIMELINE_LEN)).collect()),
        "the second report is at the front and the newest at the back: the first was dropped"
    );
    assert_eq!(
        state.timeline_dropped(),
        1,
        "which is the one thing a reader of the story cannot work out from it",
    );
}

/// The head is a count of reports modulo its own width, and the width is a whole
/// number of ring lengths -- so a story longer than the width reads in order
/// across the wrap. Three hundred reports into a full ring is what crosses it.
#[test]
fn a_story_longer_than_the_head_s_own_width_still_reads_in_order() {
    let mut state = SessionStateWire::new();
    for n in 0..300 {
        push_numbered(&mut state, n).expect("a ring takes every report");
    }
    assert_eq!(held_reports(&state), Ok(TIMELINE_LEN));
    assert_eq!(
        story(&state),
        Ok((300 - u32::from(TIMELINE_LEN)..300).collect()),
        "the newest ring's worth, oldest first, across two wraps of the head",
    );
    assert_eq!(
        state.timeline_dropped(),
        300 - u32::from(TIMELINE_LEN),
        "and every row that went is counted",
    );
}

#[test]
fn a_head_past_the_ring_with_nothing_dropped_is_refused() {
    let mut state = SessionStateWire::new();
    push_numbered(&mut state, 3).expect("an empty ring takes a report");
    state.set_timeline_head(TIMELINE_LEN + 1);

    let refusal = SessionSlotError::TimelineCount {
        appended: TIMELINE_LEN + 1,
    };
    assert_eq!(held_reports(&state), Err(refusal));
    assert_eq!(report_row(&state, 0), Err(refusal));

    let head = state.timeline_head();
    assert_eq!(push_numbered(&mut state, 4), Err(refusal));
    assert_eq!(
        (state.timeline_head(), state.timeline_dropped()),
        (head, 0),
        "a refused ring is left exactly as it was found"
    );
    assert_eq!(
        numbered(&state.timeline()[0]),
        3,
        "and nothing was appended over the row it already held"
    );
}

/// The head says a row is a report and the row says it is none. Refused, which
/// is the whole reason a cleared row reads as no report: this is memory nobody
/// wrote being called a story.
#[test]
fn a_row_the_head_calls_a_report_and_which_holds_none_is_refused() {
    let mut state = SessionStateWire::new();
    state.set_timeline_head(1);
    assert_eq!(held_reports(&state), Ok(1));
    assert_eq!(
        report_row(&state, 0),
        Err(SessionSlotError::NoSuchReportKind(0))
    );
}

/// The message that carries a story holds a row per row of the ring.
///
/// The publish writes every row the ring holds into one message and can do
/// nothing with a row that will not fit, so the two lengths are one number
/// declared in two files. Growing the ring past the message is what this
/// refuses: it would be a panic in a cog's execute body, which ends the control
/// process, reached by a change that looks free.
#[test]
fn the_story_message_holds_a_row_for_every_row_of_the_ring() {
    let mut message = TimelineWire::new();
    assert_eq!(
        message.entries_mut().capacity(),
        usize::from(TIMELINE_LEN),
        "the ring and the message that carries it are the same length"
    );
}

#[test]
fn a_cleared_timeline_holds_no_story_at_all() {
    let mut state = SessionStateWire::new();
    for n in 0..5 {
        push_numbered(&mut state, n).expect("a ring with room takes a report");
    }
    clear_timeline(&mut state);

    assert_eq!(state.timeline_head(), 0);
    assert_eq!(state.timeline_dropped(), 0);
    assert_eq!(held_reports(&state), Ok(0));
    assert_eq!(report_row(&state, 0), Ok(None));
    for row in state.timeline() {
        assert_eq!(
            row.kind(),
            ReportKindSlot::NONE,
            "every row went with the head"
        );
    }
}

/// The ring's length is written in two places -- this crossing's constant and the
/// schema's own array -- and the head arithmetic is sound only while the head's
/// width is a whole number of that length.
#[test]
fn the_ring_is_as_long_as_the_slot_that_holds_it() {
    assert_eq!(
        usize::from(TIMELINE_LEN),
        SessionStateWire::new().timeline().len()
    );
    assert!(
        (usize::from(u8::MAX) + 1).is_multiple_of(usize::from(TIMELINE_LEN)),
        "otherwise the row the head names would jump at the wrap"
    );
}

/// What a slot nothing wrote is: a session that has not begun. Every default here
/// is load-bearing -- the first execution reads this slot back and needs no flag
/// to know it is the first, and every watchdog anchor has to read as unanchored
/// rather than as an instant at the epoch.
#[test]
fn a_session_slot_nobody_wrote_is_a_session_that_has_not_begun() {
    let state = SessionStateWire::new();

    assert_eq!(state.phase(), SessionPhaseWire::STARTING);
    assert_eq!(state.seq_kind(), SeqKindWire::NONE);
    assert!(!state.started(), "no execution has run");
    assert!(!state.saw_sample(), "and no sample has arrived");
    assert_eq!(state.script_id(), 0);
    assert!(!state.torque_off_pending());

    assert!(!state.aux().active(), "nothing is outstanding");
    assert!(!state.winddown().active(), "and no maneuver is running");
    assert!(
        !state.schedule().engaged(),
        "nothing is commanded before a script is accepted"
    );
    assert_eq!(state.schedule().epoch(), 0);
    assert_eq!(state.schedule().steps().len(), 0);
    assert_eq!(state.schedule().overlays().len(), 0);

    assert_eq!(held_reports(&state), Ok(0));
    assert_eq!(state.timeline_dropped(), 0);
    assert_eq!(state.next_corr(), 0);
    assert_eq!(
        state.degrade_release(),
        JointFlagsWire::NONE,
        "and no row is owed a torque-off write"
    );

    for total in [
        state.scripts_accepted(),
        state.scripts_refused(),
        state.faults_recorded(),
        state.responses_taken(),
        state.aux_retries(),
        state.aux_failures(),
        state.reports_published(),
        state.reports_dropped(),
        state.undecodable_inbound(),
        state.refused_state(),
    ] {
        assert_eq!(total, 0, "every total starts at nothing");
    }
}
