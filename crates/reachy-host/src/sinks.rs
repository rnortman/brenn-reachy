//! The two seams the voice pipeline is composed through, filled with the edge.
//!
//! The pod platform's server publishes motion intent on the bus and hears none,
//! because on a pod there is no body on the same machine to move. On the robot
//! there is, and the two halves of one motion path meet in this process: the
//! scripter's decisions go to the edge's gate instead of a publish, and a body
//! a remote sender put on the bus arrives at the same gate. Both seams are the
//! pod platform's own, named in terms of an encoded body and nothing else, so
//! neither of them knows what a robot is.
//!
//! Neither sink may block. The scripter's sink runs on the task that drains the
//! pipeline's taps; the intent sink runs on the loop draining the bridge's
//! event channel, and an embedder that stops draining that back-pressures a
//! socket read. So both do the same small thing: hand the body to the bounded
//! queue the edge's loop takes from, and say so when it would not fit.
//!
//! What a body is worth is not decided here. The gate screens it — size, pod,
//! sequence, and the compile — and everything either sink can report is that
//! the body never reached the gate at all.

use std::sync::Arc;

use reachy_edge::{Origin, edge_line_with, now};
use serde_json::json;
use speech_surface::{IntentSink, ScriptOut, ScriptSink};

use crate::intents::{Intents, NotOffered};
use crate::words;

/// Where a sink's own lines go.
///
/// Not [`reachy_edge::Surface`]: that one is owned by the loop that follows the
/// session's story, on its own thread, and the sinks run on the runtime's. What
/// a sink says is a line and never an alert — a body that did not reach the
/// gate was never classified, and the table that decides what interrupts an
/// operator only sees what the gate answered.
pub trait Lines: Send + Sync + 'static {
    /// One line of narration, already rendered.
    fn say(&self, line: String);
}

/// The shipped one: the same stdout stream every other line lands on.
#[derive(Clone, Copy, Debug, Default)]
pub struct Stdout;

impl Lines for Stdout {
    fn say(&self, line: String) {
        println!("{line}");
    }
}

/// The scripter's decisions, handed to the edge instead of published.
///
/// This is the wake word's path to the motors: a decision made in this process,
/// compiled in this process, and one loopback datagram. It crosses no bus, and
/// the bus it does not cross is off-site.
pub struct ScripterIntents {
    intents: Intents,
    lines: Arc<dyn Lines>,
}

impl ScripterIntents {
    /// Point the scripter at `intents`, saying what it drops on `lines`.
    #[must_use]
    pub fn new(intents: Intents, lines: Arc<dyn Lines>) -> Self {
        Self { intents, lines }
    }
}

impl ScriptSink for ScripterIntents {
    fn offer(&self, out: ScriptOut) {
        if let Err(error) = self.intents.offer(out.body.into_bytes(), Origin::Local) {
            self.lines
                .say(unoffered_line("scripter", &out.pod, out.seq, error));
        }
    }
}

/// A motion intent body off the bus, handed to the same gate.
///
/// The receiver cannot tell a remote sender's script from the one this process
/// authored, because both meet one sequence gate and one compile.
pub struct BusIntents {
    intents: Intents,
}

impl BusIntents {
    /// Point the bus's motion channel at `intents`.
    #[must_use]
    pub fn new(intents: Intents) -> Self {
        Self { intents }
    }
}

impl IntentSink for BusIntents {
    fn deliver(&self, body: &str) -> Result<(), &'static str> {
        self.intents
            .offer(body.as_bytes().to_vec(), Origin::Remote)
            .map_err(reason_word)
    }
}

/// The one word the driver's drop line reports a refusal with.
///
/// A fixed vocabulary, which is what the seam asks for: whatever detail sits
/// behind one of these words belongs on the stream of whoever has it.
const fn reason_word(error: NotOffered) -> &'static str {
    match error {
        NotOffered::Backlogged => "edge_backlogged",
        NotOffered::Stopped => "edge_stopped",
    }
}

/// A body that never reached the gate, as a line.
///
/// On the edge's own stream, because that is where a reader looking for what
/// happened to a script is already looking — but distinct in kind from a
/// refusal, which is the gate having seen a body and declined it.
fn unoffered_line(source: &str, pod: &str, seq: u64, error: NotOffered) -> String {
    edge_line_with(
        words::UNOFFERED,
        now(),
        &format!(
            "a script for `{pod}` at seq {seq} never reached the gate: {error}. nothing is \
             retried; the sender's next refresh is what recovers"
        ),
        &[
            ("source", json!(source)),
            ("pod", json!(pod)),
            ("seq", json!(seq)),
        ],
    )
}

#[cfg(test)]
mod tests {
    use std::sync::Mutex;

    use motion_proto::{MotionScript, Posture, Step};
    use reachy_edge::{EdgeConfig, HostEdge, MotionTable, Origin, Surface};

    use super::*;
    use crate::intents::{INTENT_BACKLOG, queue};

    /// The pod the fixture bodies are addressed to.
    const POD: &str = "fixture-reachy";

    /// Everything said, kept rather than printed.
    #[derive(Debug, Default)]
    struct Recorded {
        lines: Mutex<Vec<String>>,
    }

    impl Lines for Arc<Recorded> {
        fn say(&self, line: String) {
            self.lines
                .lock()
                .expect("an unpoisoned recorder")
                .push(line);
        }
    }

    /// What said returns: the lines, cloned out from behind the lock.
    fn said(recorder: &Recorded) -> Vec<String> {
        recorder
            .lines
            .lock()
            .expect("an unpoisoned recorder")
            .clone()
    }

    /// A lawful script body for `POD`, as the wire contract encodes one.
    fn body(seq: u64) -> String {
        MotionScript::new(POD, seq, vec![Step::new(0, Posture::Up)], 13_000)
            .expect("a lawful script")
            .encode()
    }

    /// One decision, as the scripter's sink is handed one.
    fn decision(seq: u64) -> ScriptOut {
        ScriptOut {
            pod: POD.to_string(),
            seq,
            body: body(seq),
        }
    }

    /// A surface that keeps what the gate narrated.
    #[derive(Debug, Default)]
    struct Narration {
        lines: Vec<String>,
    }

    impl Surface for Narration {
        fn say(&mut self, line: String) {
            self.lines.push(line);
        }

        fn alert(&mut self, _alert: &reachy_edge::Alert) {}
    }

    #[test]
    fn a_scripter_decision_reaches_the_gate_and_compiles() {
        let (intents, waiting) = queue();
        let recorder = Arc::new(Recorded::default());
        let sink = ScripterIntents::new(intents, Arc::new(Arc::clone(&recorder)));

        sink.offer(decision(1));

        let queued = waiting.next().expect("the body the scripter decided");
        assert_eq!(
            queued.body,
            body(1).into_bytes(),
            "the encoded body, verbatim"
        );
        assert_eq!(queued.origin, Origin::Local, "the scripter's own decision");

        let mut host = HostEdge::new(EdgeConfig::for_pod(POD), MotionTable::default());
        let mut surface = Narration::default();
        let accepted = host
            .offer(
                &queued.body,
                queued.origin,
                reachy_edge::now(),
                &mut surface,
            )
            .expect("a lawful script the gate accepts");
        assert_eq!(accepted.script_id, 1);
        assert!(said(&recorder).is_empty(), "{:?}", said(&recorder));
    }

    #[test]
    fn a_bus_delivery_reaches_the_same_gate() {
        let (intents, waiting) = queue();
        let sink = BusIntents::new(intents);

        sink.deliver(&body(4)).expect("a queue with room");

        let queued = waiting.next().expect("the body the bus delivered");
        assert_eq!(queued.origin, Origin::Remote, "somebody else's script");
        let mut host = HostEdge::new(EdgeConfig::for_pod(POD), MotionTable::default());
        let mut surface = Narration::default();
        assert!(
            host.offer(
                &queued.body,
                queued.origin,
                reachy_edge::now(),
                &mut surface
            )
            .is_some(),
            "the bus's body meets the gate the scripter's does",
        );
    }

    #[test]
    fn both_sources_meet_one_sequence_gate() {
        // The interleave the shared gate exists for: a bus sender re-offering a
        // number the in-process scripter already spent is a redelivery, and the
        // gate says so whichever seam it arrived through.
        let (intents, waiting) = queue();
        let scripter = ScripterIntents::new(intents.clone(), Arc::new(Stdout));
        let bus = BusIntents::new(intents);
        scripter.offer(decision(9));
        bus.deliver(&body(9)).expect("a queue with room");

        let mut host = HostEdge::new(EdgeConfig::for_pod(POD), MotionTable::default());
        let mut surface = Narration::default();
        let first = waiting.next().expect("the scripter's body");
        let second = waiting.next().expect("the bus's body");
        assert!(
            host.offer(&first.body, first.origin, reachy_edge::now(), &mut surface)
                .is_some()
        );
        assert!(
            host.offer(
                &second.body,
                second.origin,
                reachy_edge::now(),
                &mut surface
            )
            .is_none(),
            "the same sequence number twice is a redelivery",
        );
    }

    #[test]
    fn a_full_queue_is_a_word_the_driver_reports() {
        let (intents, _waiting) = queue();
        let bus = BusIntents::new(intents);
        for seq in 1..=INTENT_BACKLOG as u64 {
            bus.deliver(&body(seq)).expect("a queue with room");
        }
        assert_eq!(
            bus.deliver(&body(99)),
            Err("edge_backlogged"),
            "the word is fixed vocabulary, not a sentence",
        );
    }

    #[test]
    fn a_stopped_loop_is_its_own_word() {
        let (intents, waiting) = queue();
        drop(waiting);
        assert_eq!(
            BusIntents::new(intents).deliver(&body(1)),
            Err("edge_stopped")
        );
    }

    #[test]
    fn a_scripter_decision_that_did_not_fit_is_said() {
        let (intents, _waiting) = queue();
        let recorder = Arc::new(Recorded::default());
        let sink = ScripterIntents::new(intents, Arc::new(Arc::clone(&recorder)));
        for seq in 1..=INTENT_BACKLOG as u64 + 1 {
            sink.offer(decision(seq));
        }

        let lines = said(&recorder);
        assert_eq!(lines.len(), 1, "{lines:?}");
        let line: serde_json::Value = serde_json::from_str(&lines[0]).expect("one JSON object");
        assert_eq!(line["stream"], "edge");
        assert_eq!(line["kind"], "unoffered");
        assert_eq!(line["source"], "scripter");
        assert_eq!(line["pod"], POD);
        assert_eq!(line["seq"], INTENT_BACKLOG as u64 + 1);
        assert!(
            line["says"]
                .as_str()
                .expect("a sentence")
                .contains("retried"),
            "{lines:?}",
        );
    }

    #[test]
    fn every_refusal_has_a_word() {
        // Wildcard-free, so a third way a body misses the queue is a decision
        // here rather than a word the driver's vocabulary never grew.
        assert_eq!(reason_word(NotOffered::Backlogged), "edge_backlogged");
        assert_eq!(reason_word(NotOffered::Stopped), "edge_stopped");
    }
}
