//! The wake cog's body: one script, once.
//!
//! What it exists for: the session engages only on a script accepted while it
//! is resting, and on a machine nothing publishes one. So the box carries a
//! minimal closed-loop intent source, and this is it. It reads the session's own
//! narration, waits for the transition that says commissioning finished, and
//! publishes the configured gesture with an arrival instant far enough ahead
//! that the arming has room to finish before the first step opens.
//!
//! Nothing about it retries. A script the session refuses is refused for a
//! reason about the machine or about the script, and neither is answered by
//! sending it again: no automatic recovery, and a second script would be a
//! second engagement nobody asked for. So the slot's count of scripts published
//! is written the moment a script goes out and is never cleared: that count
//! being non-zero is the whole of the decision memory, and there is no second
//! field able to disagree with it.
//!
//! It holds no state of its own beyond that slot.
//!
//! Scaffolding: goes with `wake.clk`, `wake_state.clk` and `WakeParams` when
//! the intent bridge lands. TODO(reachy-pod-motion-integration)

use brenn_reachy__cogs__config_clk_rs::{WakeParams, WakePosture, WakeStepKind};
use brenn_reachy__cogs__schedule_clk_rs::{PostureWire, StepKindWire};
use brenn_reachy__cogs__script_clk_rs::ScriptStepWire;
use brenn_reachy__cogs__session_clk_rs::SessionPhaseWire;
use brenn_reachy__cogs__wake_clk_rs::WakeDial;
use brenn_reachy__cogs__wake_state_clk_rs::WakeStateWire;
use brenn_reachy__motion__reports_clk_rs::ReportKind;
use clockwork_rs::{Clear as _, SyncTime};
use motion_slots::{MS_NS, configured, counters};

/// Publish the configured gesture on the report that says the machine is
/// commissioned, and never again.
///
/// The transition watched for is the entry into `resting` from `starting`: the
/// session's own narration of having finished commissioning, which is the first
/// instant a script can be accepted. It is looked for over every row of the
/// story the session publishes, which is why a view of one message is enough.
/// Every other report is read and ignored --
/// including a later entry into `resting`, which is a session that *ended*, and
/// answering that with a fresh gesture would be a machine that wakes itself in
/// a loop.
pub fn execute_wake(dial: &mut WakeDial<'_>) {
    let params: &WakeParams = configured(dial.configs.params, "the wake cog's");
    check(params);

    let mut counters = WakeCounters::read(dial.states.sent);

    let mut commissioned = false;
    for message in dial.inputs.reports.new_msgs() {
        // Bytes that describe no story name no phase. Refused at this one call
        // rather than at each field below: a narration vocabulary with a kind
        // this build does not know is a session built against a newer tree, and
        // reading its numbers anyway is how a wrong transition gets acted on.
        let Ok(story) = message.validate() else {
            counters.refused_reports += 1;
            continue;
        };
        // What the story says it has already lost. The row this cog waits for
        // is the first of a run, so a story that dropped rows off its front may
        // no longer carry it -- and then this cog waits forever over a machine
        // that commissioned. Nothing here can recover it; the count is what
        // says, on the run's own dashboard, why nothing woke.
        counters.lost_story = u64::from(story.dropped);
        // Every row of it, every time: the message carries the whole story from
        // wherever it now begins, so the row this cog waits for is in it
        // whether it arrived on this message or on one published before this
        // cog first ran. Reading a row twice costs nothing -- the gesture goes
        // out once, and the slot below is what says so.
        for entry in story.entries.iter() {
            if entry.kind == ReportKind::PhaseChanged
                && entry.a == u32::from(SessionPhaseWire::RESTING.0)
                && entry.b == u32::from(SessionPhaseWire::STARTING.0)
            {
                commissioned = true;
            }
        }
    }

    let already = published(dial.states.sent);
    let publishing = commissioned && !already;
    if publishing {
        counters.scripts_published += 1;
    }
    // Written before the datagram: the count of having asked is what makes a
    // refusal final, and a slot updated after the publish would leave a window
    // where a second execution could ask again.
    counters.store(dial.states.sent);

    if !publishing {
        return;
    }

    // The offsets are measured from the arrival instant this cog stamps, which
    // is what keeps a clock out of the session: it turns the request into
    // absolute times by arithmetic off the stamp and never by reading one.
    let arrival = dial.start_time().as_nanos() + i64::from(params.lead_ms) * MS_NS;

    let out = &mut dial.outputs.script;
    // Cleared so that what a step does not name is the schema's zero and not a
    // field from a prior execution.
    let msg = out.msg_mut();
    msg.clear();
    msg.set_script_id(params.script_id);
    msg.set_arrival(SyncTime::from_nanos(arrival));
    {
        let mut steps = msg.steps_mut();
        for step in params.steps.iter() {
            let row: &mut ScriptStepWire = steps
                .try_grow()
                .expect("a gesture of no more steps than the script holds");
            row.set_after_ms(step.after_ms);
            row.set_duration_ms(step.duration_ms);
            row.set_kind(step_kind(step.kind));
            row.set_posture(posture(step.posture));
        }
    }
    // No overlays. A wake gesture is the base moving and nothing composed over
    // it, and a cleared message says so.
    out.mark_for_publish();
}

/// Refuse a gesture that asks for nothing, before anything acts on it.
///
/// Run on every execution rather than only on the one that publishes, so a
/// configuration this build cannot carry stops the process at the first report
/// rather than at the first commissioning. The step count needs no bound: the
/// configuration's own array holds what a script holds. What a step *says* needs
/// no check at all -- its two enumerated fields are the schema's own vocabulary,
/// refused by the parser that read the text and again by the validation
/// [`configured`] runs.
///
/// # Panics
///
/// For a gesture with no steps. A configuration is checked-in text read once at
/// process setup, so that is a build mistake rather than a case: the process
/// refuses to run on it, which is the same answer [`configured`] gives to text
/// that is not the message at all. Refusing at start-up is the safe end of it
/// -- the machine is de-torqued and nothing has been commanded.
fn check(params: &WakeParams) {
    assert!(
        params.steps.iter().count() > 0,
        "the wake gesture has no steps, so there is nothing to ask for",
    );
}

/// What a configured step asks the base for, in the schedule's own vocabulary.
///
/// A match rather than arithmetic over discriminants: the two vocabularies are
/// declared apart because a config schema cannot import the schedule's, and this
/// is where they are held to being the same set. A third step kind in the schedule
/// is a compile error here.
fn step_kind(kind: WakeStepKind) -> StepKindWire {
    match kind {
        WakeStepKind::BaseKeep => StepKindWire::BASE_KEEP,
        WakeStepKind::BasePosture => StepKindWire::BASE_POSTURE,
    }
}

/// Which posture a configured step names, in the schedule's own vocabulary.
fn posture(posture: WakePosture) -> PostureWire {
    match posture {
        WakePosture::Stow => PostureWire::STOW,
        WakePosture::Up => PostureWire::UP,
    }
}

/// Whether the one script has already gone out.
///
/// True for a slot whose bytes do not read as a state at all, which is the safe
/// reading of memory nobody wrote: a second script would be a second engagement
/// nobody asked for.
///
/// No value this build can put in the slot takes that arm. The state is two
/// plain counters, every bit pattern of which is a state, so the refusal is a
/// guard against a schema that later gains a field with a range -- not a path
/// this binary has, and no test drives it because none can.
fn published(state: &WakeStateWire) -> bool {
    match state.validate() {
        Ok(state) => state.scripts_published > 0,
        Err(_) => true,
    }
}

counters! {
    /// The run's totals, as the wake cog keeps them.
    ///
    /// Slot-only: this cog declares no signal group, for the reason `wake.clk`'s
    /// header states, so the totals are read out of the slot and nowhere else.
    WakeCounters of WakeStateWire, crossing the_wake_totals_cross_their_slot {
        /// Scripts published, which is also the record of having asked: one for
        /// a process that got as far as commissioning, zero for one that did
        /// not.
        scripts_published / set_scripts_published,
        /// Reports that failed validation.
        refused_reports / set_refused_reports,
        /// Rows the newest story said had gone off its front. A latest reading
        /// rather than a sum: every message carries the whole count, so adding
        /// them would count the same losses once per message.
        lost_story / set_lost_story,
    }
}

#[cfg(test)]
mod tests {
    //! The configuration's own refusal, and the two vocabularies crossing
    //! enumerator for enumerator.
    //!
    //! Here rather than beside the wrapper cases next door because these are
    //! about a refusal and about two enum declarations agreeing, neither of
    //! which needs a cog: the wrapper turns any panic in the body into one
    //! message, so which refusal it was can only be asserted here. A
    //! configuration naming a step kind neither declares is refused by the
    //! protobuf parser that read the text, before any of this runs.

    use super::{check, posture, step_kind};
    use brenn_reachy__cogs__config_clk_rs::{WakeParamsWire, WakePosture, WakeStepKind};
    use brenn_reachy__cogs__schedule_clk_rs::{PostureWire, StepKindWire};

    /// A gesture with no steps is refused by name, beside the code that raises
    /// it: what the wrapper's own case next door can see is only that the
    /// execution failed.
    #[test]
    #[should_panic(expected = "the wake gesture has no steps")]
    fn a_gesture_asking_for_nothing_is_refused_by_name() {
        let params = WakeParamsWire::new();
        check(params.validate().expect("zeros are a configuration"));
    }

    #[test]
    fn the_two_declared_step_kinds_cross() {
        assert_eq!(step_kind(WakeStepKind::BaseKeep), StepKindWire::BASE_KEEP);
        assert_eq!(
            step_kind(WakeStepKind::BasePosture),
            StepKindWire::BASE_POSTURE
        );
    }

    #[test]
    fn the_two_declared_postures_cross() {
        assert_eq!(posture(WakePosture::Stow), PostureWire::STOW);
        assert_eq!(posture(WakePosture::Up), PostureWire::UP);
    }
}
