//! The one gesture the harness asks for, in the intent vocabulary.
//!
//! What a motion run proves is that the machine commissions, engages, raises,
//! stows and releases, and the analyzer reads exactly that story out of the
//! log. So the gesture here is the shape the analyzer was written against —
//! raise, hold, stow, run out — expressed as a `MotionScript` rather than as a
//! schedule handed straight to the session. Everything between this text and
//! the session is the real edge: the decode, the screens, the compile and the
//! datagram.
//!
//! The offsets include an arming lead: a machine that has just commissioned is
//! still arming, and a step opening immediately would be a step commanded to an
//! unarmed machine. The edge stamps arrival at receipt and cannot date a request
//! forward, so the lead lives in the offsets: the raise sits at [`UP_AFTER_MS`],
//! and the window before it is the arming's.
//!
//! The numbers are pinned by the cases below rather than configured. A harness
//! gesture that can be edited without a case failing is a harness whose verdict
//! stops meaning what the analyzer says it means.

use motion_proto::{MotionScript, Posture, Step};

/// Whose head the harness asks about.
///
/// The sender and the screen are one process here, so the name means nothing
/// beyond agreeing with itself — which is why it is one constant and not a
/// knob. A real pod name would invite the harness to be pointed at a machine
/// whose own host is running.
pub const ASK_POD: &str = "reachy-ask";

/// The ordering number the one script carries.
///
/// One, and there is never a second: the run asks once, and a refusal is final.
pub const ASK_SEQ: u64 = 1;

/// When the head goes up, measured from the instant the edge receives the
/// script.
///
/// Eight seconds, which is the arming budget: the start-up survey is 99 bus
/// transactions and the allowance around them, and a step opening inside it
/// would be a step commanded to a machine that is not yet armed. An overrun
/// costs the gesture its window — a late or truncated raise — never a command
/// to an unarmed machine.
pub const UP_AFTER_MS: u64 = 8000;

/// When the head folds again: two seconds of holding the raise.
///
/// Most of the step is the hold rather than the move, and deliberately: a goal
/// stream that stopped when the move finished would trip the driver's dead-man.
pub const STOW_AFTER_MS: u64 = 10_000;

/// The ceiling on the whole request.
///
/// Equal to the stow's own end, which is what makes the schedule horizon fall
/// on the stow rather than past it: the edge ends a stow-terminal script at the
/// stow, and this timeout is that instant exactly — the legal boundary case,
/// not one past it.
pub const TIMEOUT_MS: u64 = 13_000;

/// The gesture as a script.
///
/// # Panics
///
/// Never: the offsets ascend and sit inside the timeout, which is the whole of
/// what the wire contract asks of a timeline, and all three are constants above.
#[must_use]
pub fn gesture(pod: &str) -> MotionScript {
    MotionScript::new(
        pod,
        ASK_SEQ,
        vec![
            Step::new(UP_AFTER_MS, Posture::Up),
            Step::new(STOW_AFTER_MS, Posture::Stow),
        ],
        TIMEOUT_MS,
    )
    .expect("the pinned gesture is a lawful timeline")
}

/// The gesture as the JSON body an intake screens.
///
/// The bytes go through the same decode a bus delivery meets, rather than
/// handing the intake a script it built itself: what the harness is for is the
/// whole edge, and a path that skipped the decode would leave it unproven on
/// every run.
#[must_use]
pub fn body(pod: &str) -> String {
    gesture(pod).encode()
}

#[cfg(test)]
mod tests {
    use brenn_reachy__cogs__schedule_clk_rs::{PostureWire, StepKindWire};
    use clockwork_rs::SyncTime;
    use motion_proto::{MotionScript, Posture};
    use reachy_edge::{Edge, EdgeConfig, MotionTable, STOW_DURATION_MS};

    use scenario::{PERIOD_NS, commission_allowance_cycles, commission_transactions};

    use super::{ASK_POD, ASK_SEQ, STOW_AFTER_MS, TIMEOUT_MS, UP_AFTER_MS, body, gesture};

    /// A round instant, so a stamp read off the wrong side of the edge shows.
    const ARRIVAL_NS: i64 = 1_700_000_000_000_000_000;

    #[test]
    fn the_gesture_is_a_raise_and_a_fold_at_the_pinned_offsets() {
        let script = gesture(ASK_POD);
        assert_eq!(script.pod(), ASK_POD);
        assert_eq!(script.seq(), ASK_SEQ);
        assert_eq!(script.timeout_ms(), TIMEOUT_MS);
        let offsets: Vec<(u64, Option<Posture>)> = script
            .steps()
            .iter()
            .map(|step| {
                (
                    step.after_ms,
                    step.action.base().and_then(motion_proto::Base::posture),
                )
            })
            .collect();
        assert_eq!(
            offsets,
            vec![
                (UP_AFTER_MS, Some(Posture::Up)),
                (STOW_AFTER_MS, Some(Posture::Stow)),
            ],
            "the analyzer reads a raise and a fold out of the log, in that order",
        );
    }

    #[test]
    fn the_timeout_falls_exactly_on_the_end_of_the_closing_stow() {
        assert_eq!(
            STOW_AFTER_MS + u64::from(STOW_DURATION_MS),
            TIMEOUT_MS,
            "a stow-terminal script ends at its stow, and a stow ending past the timeout is a \
             refusal: this gesture sits on the boundary, so a stow budget that changes without \
             this number following it is red here rather than on a unit",
        );
    }

    /// The margin the raise's offset must hold over the survey's allowance, as a
    /// fraction. Bare sufficiency — an offset that merely equals the allowance —
    /// leaves no room for bus-retry noise; asserting the margin catches an edit
    /// that erodes it before it shows up as a late first step.
    const HEADROOM_NUMERATOR: i64 = 5;
    const HEADROOM_DENOMINATOR: i64 = 4;

    /// What the raise's offset has to cover: every transaction that takes hold of
    /// the machine, at the cycles a transaction costs.
    ///
    /// Both halves of that product are derived from tables this tree grows — the
    /// provisioning table's cells and the gains-and-profile write set — so the
    /// constant above is the one part of the derivation that cannot grow by
    /// itself. This is what makes it grow: a register added to the sweep widens
    /// the survey and fails here, rather than costing the gesture its window on a
    /// machine whose survey is still running. An overrun costs a late, truncated
    /// or skipped raise; it never commands an unarmed machine, because the
    /// session is what arms and the session is what runs the step.
    #[test]
    fn the_raise_waits_out_the_survey_that_has_to_finish_before_it() {
        let up_ns =
            i64::try_from(UP_AFTER_MS).expect("a pinned offset is a count of ms") * 1_000_000;
        let allowance_ns = commission_allowance_cycles() * PERIOD_NS;
        let required_ns = allowance_ns * HEADROOM_NUMERATOR / HEADROOM_DENOMINATOR;
        assert!(
            up_ns >= required_ns,
            "the gesture raises at {} ms and taking hold of the machine allows {} transactions \
             at three cycles each, which is {} ms: the offset has to clear that by a quarter of \
             it again -- {} ms -- so that what a noisy bus costs beyond the per-transaction \
             allowance still fits, and an edit that eats the margin is read here rather than as \
             a late, truncated or skipped raise",
            up_ns / 1_000_000,
            commission_transactions(),
            allowance_ns / 1_000_000,
            required_ns / 1_000_000
        );
    }

    #[test]
    fn the_body_decodes_back_into_the_gesture() {
        let text = body(ASK_POD);
        let decoded = MotionScript::decode(&text).expect("the harness emits what the edge reads");
        assert_eq!(decoded.pod(), ASK_POD);
        assert_eq!(decoded.timeout_ms(), TIMEOUT_MS);
        assert_eq!(decoded.steps().len(), 2);
    }

    #[test]
    fn the_edge_compiles_it_into_the_schedule_the_analyzer_expects() {
        let mut edge = Edge::new(EdgeConfig::for_pod(ASK_POD), MotionTable::default());
        let accepted = edge
            .accept(body(ASK_POD).as_bytes(), SyncTime::from_nanos(ARRIVAL_NS))
            .expect("the harness gesture passes every screen of the edge it drives");
        assert_eq!(accepted.script_id, 1);
        assert_eq!(accepted.seq, ASK_SEQ);
        assert_eq!(accepted.message.arrival(), SyncTime::from_nanos(ARRIVAL_NS));

        let rows: Vec<(u32, u32, StepKindWire, PostureWire)> = accepted
            .message
            .steps()
            .iter()
            .map(|step| {
                (
                    step.after_ms(),
                    step.duration_ms(),
                    step.kind(),
                    step.posture(),
                )
            })
            .collect();
        assert_eq!(
            rows,
            vec![
                (8000, 2000, StepKindWire::BASE_POSTURE, PostureWire::UP),
                (10_000, 3000, StepKindWire::BASE_POSTURE, PostureWire::STOW),
            ],
            "raise at the arming budget, hold two seconds, fold on the configured stow, and end \
             at the timeout",
        );
        assert!(
            accepted.message.overlays().is_empty(),
            "a raise-and-fold is the base moving and nothing composed over it",
        );
    }

    #[test]
    fn a_second_ask_under_the_same_number_is_stale() {
        let mut edge = Edge::new(EdgeConfig::for_pod(ASK_POD), MotionTable::default());
        let text = body(ASK_POD);
        edge.accept(text.as_bytes(), SyncTime::from_nanos(ARRIVAL_NS))
            .expect("the first ask");
        let refusal = edge
            .accept(text.as_bytes(), SyncTime::from_nanos(ARRIVAL_NS))
            .expect_err("the run asks once, and the gate says so if anything asks twice");
        assert_eq!(refusal.kind(), "stale");
    }
}
