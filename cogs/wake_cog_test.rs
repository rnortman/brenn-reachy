//! Unit tests for the wake cog, against the generated test wrapper.
//!
//! The wrapper wires no channels: each case publishes the reports it wants seen,
//! runs one execution, and reads back what was published and what the slot
//! holds. That is the whole harness -- no box, no process, no clock, and no
//! socket.
//!
//! Time is passed per call, so a case says at what instant each execution
//! happens. The reports carry their own instants, and the two are deliberately
//! different numbers here: the arrival this cog stamps is derived from the
//! execution's start time and not from whatever the session said it was
//! narrating.
//!
//! Unit tests are this cog's whole coverage, deliberately. A second system
//! target in the scenario suite is real cost spent on a cog that goes when the
//! intent bridge lands.

use brenn_reachy__cogs__config_clk_rs::{
    WakeParamsWire, WakePostureWire, WakeStepKindWire, WakeStepWire,
};
use brenn_reachy__cogs__schedule_clk_rs::{Posture, StepKind};
use brenn_reachy__cogs__script_clk_rs::ScriptWire;
use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__cogs__wake_clk_rs_test::WakeTestWrapper;
use brenn_reachy__motion__reports_clk_rs::ReportKindWire;
use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, TimelineWire};
use clockwork_rs::SyncTime;
use motion_slots::MS_NS;

/// The instant every case starts from. Round rather than zero, so a time that
/// travelled through the wrong field is a number nothing else in the case is.
const T0: i64 = 1_700_000_000_000_000_000;

/// The execution the reports are seen on, one tenth of a second past the start:
/// the session's own wake floor, which is the rate its narration arrives at.
const WAKE: i64 = T0 + 100_000_000;

/// The instant the reports themselves carry. Deliberately neither [`T0`] nor
/// [`WAKE`]: nothing this cog stamps may come off a report's own clock.
const REPORTED: i64 = T0 + 37_000_000;

/// The configured lead, milliseconds.
const LEAD_MS: u32 = 4000;

/// The configured script number.
const SCRIPT_ID: u32 = 1;

/// One step of the gesture as a case states it.
struct Step {
    after_ms: u32,
    duration_ms: u32,
    kind: WakeStepKindWire,
    posture: WakePostureWire,
}

/// The shipped gesture's shape: up and hold, then stow and hold.
fn gesture() -> [Step; 2] {
    [
        Step {
            after_ms: 0,
            duration_ms: 2000,
            kind: WakeStepKindWire::BASE_POSTURE,
            posture: WakePostureWire::UP,
        },
        Step {
            after_ms: 2000,
            duration_ms: 3000,
            kind: WakeStepKindWire::BASE_POSTURE,
            posture: WakePostureWire::STOW,
        },
    ]
}

/// The configuration a case runs on.
fn params(lead_ms: u32, steps: &[Step]) -> WakeParamsWire {
    let mut msg = WakeParamsWire::new();
    msg.set_script_id(SCRIPT_ID);
    msg.set_lead_ms(lead_ms);
    {
        let mut rows = msg.steps_mut();
        rows.clear();
        for step in steps {
            let row: &mut WakeStepWire = rows.try_grow().expect("sixteen steps is plenty");
            row.set_after_ms(step.after_ms);
            row.set_duration_ms(step.duration_ms);
            row.set_kind(step.kind);
            row.set_posture(step.posture);
        }
    }
    msg
}

/// A cog with its input sized, stood up at [`T0`], and configured.
///
/// Unprimed matters for the sizing: an input can only be sized before
/// `initialize`. The configuration is seeded after it and before the first
/// execution, the way every cog's is here -- a config slot is read at the top of
/// every execution.
fn wake(config: &WakeParamsWire) -> WakeTestWrapper {
    let mut cog = WakeTestWrapper::new();
    cog.input_reports_set_num_slots(8);
    cog.initialize(SyncTime::from_nanos(T0));
    cog.set_config_params(config);
    cog
}

/// The same, on the shipped gesture.
fn shipped() -> WakeTestWrapper {
    wake(&params(LEAD_MS, &gesture()))
}

/// One report as the session's channel carries it.
fn report(kind: ReportKindWire, a: u32, b: u32) -> TimelineEntryWire {
    let mut msg = TimelineEntryWire::new();
    msg.set_time(SyncTime::from_nanos(REPORTED));
    msg.set_kind(kind);
    msg.set_a(a);
    msg.set_b(b);
    msg
}

/// The session's story as it goes out: every row it has narrated, in one
/// message.
fn story(rows: &[TimelineEntryWire]) -> TimelineWire {
    let mut msg = TimelineWire::new();
    {
        let mut entries = msg.entries_mut();
        for row in rows {
            *entries
                .try_grow()
                .expect("a story of no more rows than the message holds") = row.clone();
        }
    }
    msg
}

/// The narration of commissioning having finished: entering `resting` from
/// `starting`.
fn commissioned() -> TimelineEntryWire {
    report(
        ReportKindWire::PHASE_CHANGED,
        u32::from(SessionPhaseWire::RESTING.0),
        u32::from(SessionPhaseWire::STARTING.0),
    )
}

/// Run one execution at `at_ns` over the reports, and answer with what it
/// published.
fn drive(
    cog: &mut WakeTestWrapper,
    at_ns: i64,
    reports: &[TimelineEntryWire],
) -> Option<Published> {
    cog.publish_reports(&story(reports), SyncTime::from_nanos(REPORTED));
    assert!(
        cog.execute(SyncTime::from_nanos(at_ns)),
        "a report is this cog's only execution condition",
    );
    published(cog)
}

/// The request one execution published, copied out of the message.
#[derive(Clone, PartialEq, Debug)]
struct Published {
    script_id: u32,
    arrival_ns: i64,
    steps: Vec<(u32, u32, StepKind, Posture)>,
    overlays: usize,
}

/// What the execution asked for, or `None` where it asked nothing.
fn published(cog: &mut WakeTestWrapper) -> Option<Published> {
    let msg: &ScriptWire = cog.try_next_script()?;
    let script = msg
        .validate()
        .expect("a request this cog wrote whole through its own view");
    Some(Published {
        script_id: script.script_id,
        arrival_ns: script.arrival.as_nanos(),
        steps: script
            .steps
            .iter()
            .map(|step| (step.after_ms, step.duration_ms, step.kind, step.posture))
            .collect(),
        overlays: script.overlays.iter().count(),
    })
}

/// Whether the slot says the script has gone out, which is the count of scripts
/// published being non-zero: the cog keeps no second field saying it.
fn recorded(cog: &WakeTestWrapper) -> bool {
    cog.state_sent().scripts_published() > 0
}

#[test]
fn the_commissioned_report_publishes_the_configured_gesture() {
    let mut cog = shipped();
    let asked = drive(&mut cog, WAKE, &[commissioned()]).expect("the gesture");
    assert_eq!(
        asked,
        Published {
            script_id: SCRIPT_ID,
            // Off the execution's own start time, not off the report's instant
            // and not off the run's start.
            arrival_ns: WAKE + i64::from(LEAD_MS) * MS_NS,
            steps: vec![
                (0, 2000, StepKind::BasePosture, Posture::Up),
                (2000, 3000, StepKind::BasePosture, Posture::Stow),
            ],
            // A wake gesture is the base moving and nothing composed over it.
            overlays: 0,
        },
    );
    assert!(recorded(&cog), "the slot records having asked");
}

/// A gesture using every step the configuration can hold arrives whole.
///
/// The boundary the shipped two-step gesture never reaches: the configuration's
/// array and the script's are declared apart, in two schemas, and the host grows
/// one into the other. Their capacities agreeing is what keeps that growth from
/// panicking in the control process at commissioning time on a powered unit, so
/// it is asserted rather than read off the two files.
#[test]
fn a_gesture_filling_the_configuration_reaches_the_script_whole() {
    let full: Vec<Step> = (0..16_u32)
        .map(|n| Step {
            after_ms: n * 100,
            duration_ms: 100,
            kind: WakeStepKindWire::BASE_POSTURE,
            // Alternating, so a step landing in the wrong row is visible.
            posture: if n % 2 == 0 {
                WakePostureWire::UP
            } else {
                WakePostureWire::STOW
            },
        })
        .collect();
    let mut cog = wake(&params(LEAD_MS, &full));
    let asked = drive(&mut cog, WAKE, &[commissioned()]).expect("the gesture");
    let wanted: Vec<(u32, u32, StepKind, Posture)> = (0..16_u32)
        .map(|n| {
            (
                n * 100,
                100,
                StepKind::BasePosture,
                if n % 2 == 0 {
                    Posture::Up
                } else {
                    Posture::Stow
                },
            )
        })
        .collect();
    assert_eq!(asked.steps, wanted);
}

#[test]
fn the_lead_is_the_configured_one() {
    let mut cog = wake(&params(250, &gesture()));
    let asked = drive(&mut cog, WAKE, &[commissioned()]).expect("the gesture");
    assert_eq!(asked.arrival_ns, WAKE + 250 * MS_NS);
}

#[test]
fn a_second_commissioned_report_publishes_nothing() {
    let mut cog = shipped();
    drive(&mut cog, WAKE, &[commissioned()]).expect("the gesture");
    // Unreachable in a real run -- a session commissions once -- and asserted
    // anyway: the record of having asked is what makes every refusal final, and
    // a case that only fed a refusal would pass on the refusal's own shape
    // rather than on that record.
    assert_eq!(
        drive(&mut cog, WAKE + 100_000_000, &[commissioned()]),
        None,
        "the script goes out once for the life of a process",
    );
}

#[test]
fn a_refusal_is_final() {
    let mut cog = shipped();
    drive(&mut cog, WAKE, &[commissioned()]).expect("the gesture");
    // The session declined it. Nothing here retries: a refusal names a fact
    // about the script or about the phase it arrived in, and neither is
    // answered by sending the same request again.
    let refused = report(ReportKindWire::SCRIPT_REFUSED, SCRIPT_ID, 0);
    assert_eq!(drive(&mut cog, WAKE + 100_000_000, &[refused]), None);
    assert!(recorded(&cog));
}

#[test]
fn a_session_that_ended_is_not_a_machine_that_woke() {
    let mut cog = shipped();
    // Entering `resting` from `stopping` is a session that finished. Answering
    // it with a fresh gesture would be a machine that wakes itself in a loop.
    let ended = report(
        ReportKindWire::PHASE_CHANGED,
        u32::from(SessionPhaseWire::RESTING.0),
        u32::from(SessionPhaseWire::STOPPING.0),
    );
    assert_eq!(drive(&mut cog, WAKE, &[ended]), None);
    assert!(!recorded(&cog));
}

#[test]
fn every_other_report_kind_is_read_and_ignored() {
    let mut cog = shipped();
    let others = [
        report(ReportKindWire::SCRIPT_ACCEPTED, 1, 2),
        report(ReportKindWire::SCHEDULE_PUBLISHED, 1, 2),
        report(ReportKindWire::FAULT_RECORDED, 1, 2),
        report(ReportKindWire::RESPONSE_TAKEN, 1, 2),
        report(ReportKindWire::TORQUE_OFF_CONFIRMED, 0, 0),
        report(ReportKindWire::SESSION_ENDED, 1, 0),
        report(ReportKindWire::BUS_FAILURE_DECLARED, 0, 0),
        // A phase change into another phase, with the numbers this cog
        // watches for in the other order: `a` is the phase entered.
        report(
            ReportKindWire::PHASE_CHANGED,
            u32::from(SessionPhaseWire::STARTING.0),
            u32::from(SessionPhaseWire::RESTING.0),
        ),
    ];
    assert_eq!(drive(&mut cog, WAKE, &others), None);
    assert!(!recorded(&cog));
    assert_eq!(cog.state_sent().refused_reports(), 0, "all eight read");
}

#[test]
fn the_transition_is_found_in_a_burst() {
    let mut cog = shipped();
    // What a real wake looks like: the commissioning transition among the rows
    // the session drained with it. The window is sized to see every one.
    let burst = [
        report(ReportKindWire::TORQUE_OFF_CONFIRMED, 0, 0),
        commissioned(),
        report(ReportKindWire::SCHEDULE_PUBLISHED, 1, 2),
    ];
    let asked = drive(&mut cog, WAKE, &burst).expect("the gesture");
    assert_eq!(asked.script_id, SCRIPT_ID);
}

/// A story that has wrapped may no longer carry the transition, and no later
/// message will bring it back: the wake never happens, and the count of what the
/// story said it had lost is the only thing that says why. A machine that
/// silently did not move is the failure this counter exists to name.
#[test]
fn a_story_that_dropped_its_front_is_counted_and_wakes_nothing() {
    let mut cog = shipped();
    let mut wrapped = story(&[report(ReportKindWire::SCHEDULE_PUBLISHED, 1, 2)]);
    wrapped.set_dropped(9);
    cog.publish_reports(&wrapped, SyncTime::from_nanos(REPORTED));
    assert!(
        cog.execute(SyncTime::from_nanos(WAKE)),
        "the report woke it"
    );

    assert_eq!(published(&mut cog), None, "the transition is not in it");
    assert!(!recorded(&cog));
    assert_eq!(cog.state_sent().lost_story(), 9);

    // The newest reading, not a sum: the next story carries the same losses and
    // says so once.
    let mut later = story(&[report(ReportKindWire::SCHEDULE_PUBLISHED, 3, 4)]);
    later.set_dropped(11);
    cog.publish_reports(&later, SyncTime::from_nanos(REPORTED));
    assert!(cog.execute(SyncTime::from_nanos(WAKE + 100_000_000)));
    assert_eq!(cog.state_sent().lost_story(), 11);
}

/// A story that has lost nothing says so, and the gesture goes out: the counter
/// above is a reading of the message and not a state of this cog.
#[test]
fn a_whole_story_leaves_the_count_at_nothing() {
    let mut cog = shipped();
    drive(&mut cog, WAKE, &[commissioned()]).expect("the gesture");
    assert_eq!(cog.state_sent().lost_story(), 0);
}

#[test]
fn a_report_this_build_cannot_read_is_counted_and_not_acted_on() {
    let mut cog = shipped();
    // A session built against a narration vocabulary with more kinds in it than
    // this binary has. Refused at the one boundary call rather than read for its
    // numbers: reading those anyway is how a wrong transition gets acted on.
    let unknown = report(ReportKindWire::from(200), 1, 0);
    assert_eq!(drive(&mut cog, WAKE, &[unknown]), None);
    assert_eq!(cog.state_sent().refused_reports(), 1);
    assert!(!recorded(&cog));

    // And the run's count survives the execution that publishes.
    drive(&mut cog, WAKE + 100_000_000, &[commissioned()]).expect("the gesture");
    assert_eq!(cog.state_sent().refused_reports(), 1);
    assert_eq!(cog.state_sent().scripts_published(), 1);
}

#[test]
#[should_panic(expected = "execute() failed")]
fn a_configuration_this_build_cannot_ask_for_refuses_the_execution() {
    let mut cog = wake(&params(LEAD_MS, &[]));
    // Refused on the first execution, not on the one that would have published:
    // a configuration this build cannot ask for stops the process where the
    // machine is still de-torqued and nothing has been commanded. The wrapper
    // surfaces any panic in the body as "execute() failed", which is why the
    // refusal's own message is asserted in `wake_cogs.rs`'s cases instead, on a
    // direct call to the check that raises it.
    let harmless = report(ReportKindWire::TORQUE_OFF_CONFIRMED, 0, 0);
    drive(&mut cog, WAKE, &[harmless]);
}
