//! Where the host's lines and alerts go.
//!
//! The running edge itself — the one gate, the one story follower, the one
//! alert latch — is `reachy-edge`'s [`HostEdge`], because the harness that
//! drives a motion run holds exactly the same thing. What is left here is the
//! part that is this process's own: the surface it writes on, and the one place
//! an alert leaves this machine.
//!
//! Two layers, because a deployment decides only the second. [`Console`] is
//! where every line goes and is the whole surface of a host with no bus:
//! narration, and each alert rendered as one more line of it. [`Publishing`]
//! wraps any surface and, where the composed pipeline handed one out, also
//! raises each alert on the seam that carries it to the robot's single bus
//! attachment — and, where it handed out something to speak through, says the
//! alert's own sentence out loud.
//!
//! Speaking is the third audience and the only one that is present: the person
//! standing in front of a robot that has stopped working is who the machine
//! stopped working for, and neither a log nor a phone notification reaches them
//! now. Only the alerts the table gave a sentence to are spoken.
//!
//! An alert is narrated either way. The line is what a log carries and what an
//! operator reads afterwards; the publish is what interrupts somebody now, and
//! a deployment with nothing to interrupt them through still wants the alert
//! table's picking recorded.
//!
//! Nothing about the machine rests on the publish. A raise that would not fit
//! or that has nowhere left to go is one more line saying so, never a retry and
//! never a wait: the edge's loop must not block on a socket, and reaching the
//! Minimum Risk Condition needs no reporting path at all.
//!
//! What a raise the seam accepts means is that the alert was queued, not that
//! it was carried: the attachment takes it from there, and alerts still in the
//! queue when the pipeline stops go away with it, said — if at all — on the
//! pipeline's own stream rather than this one. So a line saying an alert was
//! not published is proof it did not travel, and the absence of one is not
//! proof that it did.
//!
//! [`HostEdge`]: reachy_edge::HostEdge

use std::fmt::Display;

use clockwork_rs::SyncTime;
use reachy_edge::{Alert, Severity, Surface, alert_line, edge_line_with, now, severity_word};
use serde_json::json;
use speech_surface::{Alert as Raised, AlertRaiser, AlertRefused, AlertSeverity};

/// The surface the host runs on.
///
/// Lines to stdout, and alerts onto the same stream as one more line.
///
/// This stream is the whole of the operator surface. It says what happened and
/// it interrupts for what matters, and it answers no question about what the
/// machine is doing at the moment somebody asks.
/// TODO(host-status-egress)
#[derive(Clone, Copy, Debug, Default)]
pub struct Console;

impl Surface for Console {
    fn say(&mut self, line: String) {
        println!("{line}");
    }

    fn alert(&mut self, alert: &Alert) {
        // The alert's own instant: the table raised it while a line was being
        // written, and the clock read here is the same clock that stamped it.
        println!("{}", alert_line(alert, now()));
    }
}

/// What the robot says out loud, where this deployment can.
///
/// A trait rather than a concrete handle because the ability to speak is a
/// property of what the payload composed: a unit with a brain and a synthesizer
/// configured can say a sentence to the person standing in front of it, and one
/// without has only its log. The implementor lives beside whatever seam the
/// composed pipeline hands out.
///
/// Saying a sentence must not block the edge's loop, so an implementation
/// queues and returns. What a [`Result::Ok`] means is that the sentence was
/// queued, not that anybody heard it: synthesis that fails afterwards is said on
/// the pipeline's own stream, and [`Unspoken`] covers only a queue that would
/// not take it.
pub trait Speaker: Send {
    /// Say `sentence`, without waiting for it to be spoken.
    ///
    /// # Errors
    ///
    /// [`Unspoken`] when the queue carrying it would not take it. Nothing is
    /// retried; the sentence is dropped.
    fn speak(&self, sentence: &str) -> Result<(), Unspoken>;
}

/// Why a sentence the robot was asked to say was not queued.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Unspoken {
    /// The queue is full: the pipeline is behind, or nothing is draining it.
    #[error("the announcement queue is full")]
    Backlogged,
    /// The far end is gone: the pipeline stopped, or was never composed with a
    /// seam to speak through.
    #[error("the far end of the announcement seam is gone")]
    Gone,
}

impl Unspoken {
    /// The fixed word this refusal is reported under, so the console's spelling
    /// does not drift from the seam's own.
    #[must_use]
    pub const fn reason(self) -> &'static str {
        match self {
            Self::Backlogged => "backlogged",
            Self::Gone => "gone",
        }
    }
}

/// A surface that also puts its alerts on the bus, where a seam was handed out.
///
/// Wrapping rather than replacing: what an alert *says* is settled by the
/// surface underneath, and whether it also travels is a property of this
/// deployment. A host whose payload carries no speech configuration composes no
/// pipeline, and one whose speech configuration names no bus composes a
/// pipeline that holds no attachment; neither is handed a seam, and both run
/// the identical edge with their alerts as narration.
///
/// The seam is installed after construction because that is the order the
/// process comes up in: the surface says the host's first line before the
/// pipeline it would publish through has been composed at all.
pub struct Publishing<S: Surface> {
    inner: S,
    alerts: Option<AlertRaiser>,
    speaker: Option<Box<dyn Speaker>>,
}

impl<S: Surface> Publishing<S> {
    /// Narration only, until [`Publishing::publish_through`] or
    /// [`Publishing::speak_through`] says otherwise.
    pub const fn new(inner: S) -> Self {
        Self {
            inner,
            alerts: None,
            speaker: None,
        }
    }

    /// Carry every alert from here on to the bus as well, through `raiser`.
    pub fn publish_through(&mut self, raiser: AlertRaiser) {
        self.alerts = Some(raiser);
    }

    /// Say every alert's sentence out loud from here on, through `speaker`.
    ///
    /// Installed after construction for the same reason the raiser is: the
    /// surface writes this process's first line before the pipeline it would
    /// speak through has been composed.
    pub fn speak_through(&mut self, speaker: Box<dyn Speaker>) {
        self.speaker = Some(speaker);
    }
}

impl<S: Surface + std::fmt::Debug> std::fmt::Debug for Publishing<S> {
    /// The speaker is a trait object and says nothing about itself, so what is
    /// printed of it is whether one is installed.
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Publishing")
            .field("inner", &self.inner)
            .field("alerts", &self.alerts)
            .field("speaker", &self.speaker.is_some())
            .finish()
    }
}

impl<S: Surface> Surface for Publishing<S> {
    fn say(&mut self, line: String) {
        self.inner.say(line);
    }

    fn alert(&mut self, alert: &Alert) {
        self.inner.alert(alert);
        if let Some(raiser) = &self.alerts
            && let Err(refused) = raiser.raise(carried(alert))
        {
            // Said on the same stream the alert itself just landed on, so a
            // reader who has the alert can see whether anyone else does.
            let line = unpublished_line(alert, refused, now());
            self.inner.say(line);
        }
        if let Some(sentence) = &alert.spoken
            && let Some(speaker) = &self.speaker
            && let Err(unspoken) = speaker.speak(sentence)
        {
            let line = unspoken_line(alert, unspoken, now());
            self.inner.say(line);
        }
    }
}

/// The edge's alert, as the attachment's own type.
///
/// The title and the body cross unchanged: the table wrote them for a person,
/// and this is the same person reading them on a phone instead of in a log.
fn carried(alert: &Alert) -> Raised {
    Raised {
        severity: severity(alert.severity),
        title: alert.title.clone(),
        body: alert.body.clone(),
    }
}

/// The edge's two words as the attachment spells them, through the seam.
///
/// `AlertSeverity` arrives from `speech_surface` beside the `Alert` it fills
/// in, so the two halves of one value cannot come from two revisions.
///
/// Wildcard-free, so a third word in either vocabulary is a compile-time
/// decision here rather than an alert silently published as the wrong loudness.
/// `Info` is deliberately unreachable: nothing crossing this boundary should be
/// below Warning.
const fn severity(severity: Severity) -> AlertSeverity {
    match severity {
        Severity::Warning => AlertSeverity::Warning,
        Severity::Critical => AlertSeverity::Critical,
    }
}

/// An alert that was narrated and did not reach one of its delivery legs, as a
/// line.
///
/// Its own kind per leg rather than a sentence inside the alert line: the alert
/// is still true and still worth reading, and what this adds is that nobody was
/// interrupted by it. One builder for every leg, because what a reader joins on
/// is the same three fields whichever leg failed, and a leg with its own copy
/// of this is a leg whose fields drift out from under the report that reads
/// them.
///
/// `kind` is the leg's word, `what_failed` the clause naming what did not
/// happen, and `refused` the far end's own reason for it, both its sentence and
/// its word.
fn undelivered_line(
    alert: &Alert,
    kind: &str,
    what_failed: &str,
    refused: (&dyn Display, &'static str),
    at: SyncTime,
) -> String {
    let (says, reason) = refused;
    edge_line_with(
        kind,
        at,
        &format!(
            "the alert `{}` {what_failed}: {says}. nothing is retried; the condition is in the \
             line above and in the session's own story",
            alert.title
        ),
        &[
            // The alert's own title as a field and not only inside the
            // sentence: pairing this line with the alert it is about is what a
            // reader does with it, and the sentence is bounded and may be cut.
            ("title", json!(alert.title)),
            ("severity", json!(severity_word(alert.severity))),
            ("reason", json!(reason)),
        ],
    )
}

/// An alert that was narrated and did not reach the bus, as a line.
fn unpublished_line(alert: &Alert, refused: AlertRefused, at: SyncTime) -> String {
    let reason = refused.reason();
    undelivered_line(
        alert,
        crate::words::UNPUBLISHED,
        "was not carried to the bus",
        (&refused, reason),
        at,
    )
}

/// An alert whose sentence the robot was asked to say and did not, as a line.
///
/// It covers a queue that refused the sentence only — synthesis that fails
/// after the sentence was queued is the pipeline's own to say.
fn unspoken_line(alert: &Alert, unspoken: Unspoken, at: SyncTime) -> String {
    let reason = unspoken.reason();
    undelivered_line(
        alert,
        crate::words::UNSPOKEN,
        "was not said out loud",
        (&unspoken, reason),
        at,
    )
}

#[cfg(test)]
mod tests {
    use std::sync::{Arc, Mutex};

    use speech_surface::{ALERT_QUEUE_DEPTH, alert_seam};

    use super::*;

    /// Everything the surface was handed, in the order it was handed it.
    ///
    /// One sequence rather than a vector each: what an alert's own line says
    /// about it is only readable beside the alert, so the order the two arrive
    /// in is part of what these cases assert.
    #[derive(Clone, Debug, Default)]
    struct Recorded {
        seen: Vec<Seen>,
    }

    /// One thing a surface was handed.
    #[derive(Clone, Debug, PartialEq, Eq)]
    enum Seen {
        Said(String),
        Raised(Alert),
    }

    impl Recorded {
        /// Every line, in order.
        fn lines(&self) -> Vec<String> {
            self.seen
                .iter()
                .filter_map(|seen| match seen {
                    Seen::Said(line) => Some(line.clone()),
                    Seen::Raised(_) => None,
                })
                .collect()
        }

        /// Every alert, in order.
        fn alerts(&self) -> Vec<Alert> {
            self.seen
                .iter()
                .filter_map(|seen| match seen {
                    Seen::Raised(alert) => Some(alert.clone()),
                    Seen::Said(_) => None,
                })
                .collect()
        }
    }

    impl Surface for Recorded {
        fn say(&mut self, line: String) {
            self.seen.push(Seen::Said(line));
        }

        fn alert(&mut self, alert: &Alert) {
            self.seen.push(Seen::Raised(alert.clone()));
        }
    }

    /// One alert, as the edge's table raises one: a Critical, so it carries a
    /// sentence.
    fn alert(title: &str) -> Alert {
        Alert {
            severity: Severity::Critical,
            title: title.to_owned(),
            body: "the head is parked and will not engage until an operator has been".to_owned(),
            spoken: Some("My head motion has stopped.".to_owned()),
        }
    }

    /// The same, as the table raises the quieter kind: no sentence, because
    /// nobody in the room is interrupted for a Warning.
    fn warning(title: &str) -> Alert {
        Alert {
            severity: Severity::Warning,
            title: title.to_owned(),
            body: "the session declined a script".to_owned(),
            spoken: None,
        }
    }

    /// A speaker that records what it was asked to say, or refuses everything.
    ///
    /// Shared behind a lock because the trait speaks through `&self`.
    #[derive(Clone, Debug)]
    struct Heard {
        said: Arc<Mutex<Vec<String>>>,
        refusing: Option<Unspoken>,
    }

    impl Heard {
        /// A speaker that takes every sentence.
        fn listening() -> Self {
            Self {
                said: Arc::default(),
                refusing: None,
            }
        }

        /// A speaker that refuses every sentence with `unspoken`.
        fn refusing(unspoken: Unspoken) -> Self {
            Self {
                said: Arc::default(),
                refusing: Some(unspoken),
            }
        }

        /// Every sentence it was asked to say, in order.
        fn sentences(&self) -> Vec<String> {
            self.said.lock().expect("an unpoisoned lock").clone()
        }
    }

    impl Speaker for Heard {
        fn speak(&self, sentence: &str) -> Result<(), Unspoken> {
            if let Some(unspoken) = self.refusing {
                return Err(unspoken);
            }
            self.said
                .lock()
                .expect("an unpoisoned lock")
                .push(sentence.to_owned());
            Ok(())
        }
    }

    #[test]
    fn a_host_with_no_seam_narrates_its_alerts_and_nothing_else() {
        // A unit with no bus attachment configured: the whole edge runs and the
        // alerts are its narration.
        let mut surface = Publishing::new(Recorded::default());

        surface.alert(&alert("parked"));
        surface.say("a line".to_owned());

        assert_eq!(surface.inner.alerts(), vec![alert("parked")]);
        assert_eq!(
            surface.inner.lines(),
            vec!["a line".to_owned()],
            "an alert nobody could publish is not an unpublished one",
        );
    }

    #[test]
    fn a_host_with_a_seam_raises_every_alert_it_narrates() {
        // The queue is what proves the raise happened: it is bounded, so
        // filling it exactly is only possible if each alert was taken.
        let (raiser, _inbox) = alert_seam(ALERT_QUEUE_DEPTH);
        let mut surface = Publishing::new(Recorded::default());
        surface.publish_through(raiser);

        for n in 0..ALERT_QUEUE_DEPTH {
            surface.alert(&alert(&format!("parked {n}")));
        }
        assert_eq!(surface.inner.alerts().len(), ALERT_QUEUE_DEPTH);
        assert!(
            surface.inner.lines().is_empty(),
            "every one of them was carried: {:?}",
            surface.inner.lines(),
        );

        // One past the depth: narrated as ever, and said to be uncarried.
        surface.alert(&alert("parked again"));
        let line: serde_json::Value =
            serde_json::from_str(&surface.inner.lines()[0]).expect("one JSON object");
        assert_eq!(line["stream"], "edge");
        assert_eq!(line["kind"], "unpublished");
        assert_eq!(line["reason"], "backlogged");
        assert_eq!(line["severity"], "critical");
        assert_eq!(
            surface.inner.alerts().len(),
            ALERT_QUEUE_DEPTH + 1,
            "an alert that could not be carried is still narrated",
        );
    }

    #[test]
    fn a_seam_whose_far_end_is_gone_is_said_of_every_alert() {
        // The far end is gone before any alert: every raise refuses, and every
        // refusal is its own line. Two alerts, because one cannot tell said
        // once from said per alert.
        let (raiser, inbox) = alert_seam(2);
        drop(inbox);
        let mut surface = Publishing::new(Recorded::default());
        surface.publish_through(raiser);

        surface.alert(&alert("parked"));
        surface.alert(&alert("faulted"));

        let said_lines = surface.inner.lines();
        assert_eq!(said_lines.len(), 2, "{said_lines:?}");
        for (said, title) in said_lines.iter().zip(["parked", "faulted"]) {
            let line: serde_json::Value = serde_json::from_str(said).expect("one JSON object");
            assert_eq!(line["kind"], "unpublished");
            assert_eq!(line["reason"], "gone");
            assert_eq!(line["title"], title);
            assert!(
                line["says"].as_str().expect("a sentence").contains(title),
                "{line}",
            );
        }
    }

    #[test]
    fn an_uncarried_alert_is_said_after_the_alert_it_is_about() {
        // The point of saying it on this stream is that a reader has the alert
        // and its fate together, which is a claim about order and not only
        // about both lines existing.
        let (raiser, inbox) = alert_seam(1);
        drop(inbox);
        let mut surface = Publishing::new(Recorded::default());
        surface.publish_through(raiser);

        surface.alert(&alert("parked"));

        let [Seen::Raised(raised), Seen::Said(said)] = &surface.inner.seen[..] else {
            panic!("an alert and then its line: {:?}", surface.inner.seen);
        };
        assert_eq!(raised, &alert("parked"));
        let line: serde_json::Value = serde_json::from_str(said).expect("one JSON object");
        assert_eq!(line["kind"], "unpublished");
        assert_eq!(line["title"], "parked");
    }

    #[test]
    fn an_uncarried_warning_is_said_as_the_warning_it_is() {
        // The quieter of the two words, on the same path: a refusal that was
        // narrated and not carried must not read as a critical one.
        let (raiser, inbox) = alert_seam(1);
        drop(inbox);
        let mut surface = Publishing::new(Recorded::default());
        surface.publish_through(raiser);

        surface.alert(&warning("a script was refused"));

        let line: serde_json::Value =
            serde_json::from_str(&surface.inner.lines()[0]).expect("one JSON object");
        assert_eq!(line["kind"], "unpublished");
        assert_eq!(line["severity"], "warning");
        assert_eq!(line["title"], "a script was refused");
    }

    #[test]
    fn the_two_severities_are_the_planes_own() {
        assert_eq!(severity(Severity::Warning), AlertSeverity::Warning);
        assert_eq!(severity(Severity::Critical), AlertSeverity::Critical);
    }

    #[test]
    fn what_crosses_the_seam_is_what_the_table_wrote() {
        let raised = carried(&alert("parked"));
        assert_eq!(raised.severity, AlertSeverity::Critical);
        assert_eq!(raised.title, "parked");
        assert_eq!(raised.body, alert("parked").body);
    }

    #[test]
    fn a_critical_with_a_sentence_is_said_out_loud_once() {
        // The whole of what the speaking leg is for: the person in front of the
        // robot is told, once, and the log carries no complaint about it.
        let heard = Heard::listening();
        let mut surface = Publishing::new(Recorded::default());
        surface.speak_through(Box::new(heard.clone()));

        surface.alert(&alert("parked"));

        assert_eq!(
            heard.sentences(),
            vec!["My head motion has stopped.".to_owned()],
        );
        assert_eq!(surface.inner.alerts(), vec![alert("parked")]);
        assert!(
            surface.inner.lines().is_empty(),
            "it was said: {:?}",
            surface.inner.lines(),
        );
    }

    #[test]
    fn a_spoken_alert_is_narrated_and_raised_before_it_is_said() {
        // The order matters: the log and the bus are the accounts that survive
        // the run, and a speaker that blocked or panicked must not be able to
        // cost them the alert.
        let (raiser, _inbox) = alert_seam(1);
        let heard = Heard::listening();
        let mut surface = Publishing::new(Recorded::default());
        surface.publish_through(raiser);
        surface.speak_through(Box::new(heard.clone()));

        surface.alert(&alert("parked"));
        // One deep, so the second raise refuses and its line lands after the
        // first alert was both raised and said.
        surface.alert(&alert("parked again"));

        let [Seen::Raised(first), Seen::Raised(second), Seen::Said(said)] = &surface.inner.seen[..]
        else {
            panic!("two alerts and one line: {:?}", surface.inner.seen);
        };
        assert_eq!(first, &alert("parked"));
        assert_eq!(second, &alert("parked again"));
        let line: serde_json::Value = serde_json::from_str(said).expect("one JSON object");
        assert_eq!(line["kind"], "unpublished");
        assert_eq!(heard.sentences().len(), 2, "both were said out loud");
    }

    #[test]
    fn a_sentence_the_speaker_would_not_take_is_said_on_the_stream() {
        // The alert is still true and the room still did not hear it, which is
        // the only thing this line adds.
        let mut surface = Publishing::new(Recorded::default());
        surface.speak_through(Box::new(Heard::refusing(Unspoken::Backlogged)));

        surface.alert(&alert("parked"));

        let [Seen::Raised(raised), Seen::Said(said)] = &surface.inner.seen[..] else {
            panic!("an alert and then its line: {:?}", surface.inner.seen);
        };
        assert_eq!(raised, &alert("parked"));
        let line: serde_json::Value = serde_json::from_str(said).expect("one JSON object");
        assert_eq!(line["stream"], "edge");
        assert_eq!(line["kind"], "unspoken");
        assert_eq!(line["title"], "parked");
        assert_eq!(line["severity"], "critical");
        assert_eq!(line["reason"], "backlogged");
    }

    #[test]
    fn a_speaker_whose_far_end_is_gone_is_said_of_every_alert() {
        let mut surface = Publishing::new(Recorded::default());
        surface.speak_through(Box::new(Heard::refusing(Unspoken::Gone)));

        surface.alert(&alert("parked"));
        surface.alert(&alert("faulted"));

        let said_lines = surface.inner.lines();
        assert_eq!(said_lines.len(), 2, "{said_lines:?}");
        for (said, title) in said_lines.iter().zip(["parked", "faulted"]) {
            let line: serde_json::Value = serde_json::from_str(said).expect("one JSON object");
            assert_eq!(line["kind"], "unspoken");
            assert_eq!(line["reason"], "gone");
            assert_eq!(line["title"], title);
        }
    }

    #[test]
    fn a_warning_is_never_said_out_loud() {
        // Spoken words interrupt a room. The table decides that by handing over
        // no sentence, and this leg does not second-guess it.
        let heard = Heard::listening();
        let mut surface = Publishing::new(Recorded::default());
        surface.speak_through(Box::new(heard.clone()));

        surface.alert(&warning("a script was refused"));

        assert!(heard.sentences().is_empty());
        assert!(
            surface.inner.lines().is_empty(),
            "an alert with nothing to say is not an unspoken one: {:?}",
            surface.inner.lines(),
        );
    }

    #[test]
    fn a_host_with_no_speaker_narrates_its_criticals_and_says_nothing() {
        // The deployment that composed no pipeline, or one with no synthesizer:
        // the whole edge runs and the alert is its narration.
        let mut surface = Publishing::new(Recorded::default());

        surface.alert(&alert("parked"));

        assert_eq!(surface.inner.alerts(), vec![alert("parked")]);
        assert!(
            surface.inner.lines().is_empty(),
            "an alert nobody could say is not an unspoken one: {:?}",
            surface.inner.lines(),
        );
    }

    #[test]
    fn the_two_refusals_are_spelled_as_the_seam_spells_them() {
        assert_eq!(Unspoken::Backlogged.reason(), "backlogged");
        assert_eq!(Unspoken::Gone.reason(), "gone");
        assert_eq!(
            AlertRefused::Backlogged.reason(),
            Unspoken::Backlogged.reason()
        );
        assert_eq!(AlertRefused::Gone.reason(), Unspoken::Gone.reason());
    }

    #[test]
    fn an_unsaid_alert_names_its_own_title_and_stamp() {
        let at = SyncTime::from_nanos(1_700_000_000_000_000_000);
        let line = unspoken_line(&alert("parked"), Unspoken::Gone, at);
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("one JSON object");
        assert_eq!(parsed["at_ns"], at.as_nanos());
        assert_eq!(parsed["kind"], "unspoken");
        assert_eq!(parsed["title"], "parked");
        assert!(
            parsed["says"]
                .as_str()
                .expect("a sentence")
                .contains("parked"),
            "{parsed}",
        );
    }

    #[test]
    fn an_uncarried_alert_names_its_own_title_and_stamp() {
        let at = SyncTime::from_nanos(1_700_000_000_000_000_000);
        let line = unpublished_line(&alert("parked"), AlertRefused::Gone, at);
        let parsed: serde_json::Value = serde_json::from_str(&line).expect("one JSON object");
        assert_eq!(parsed["at_ns"], at.as_nanos());
        assert_eq!(parsed["kind"], "unpublished");
        assert_eq!(parsed["title"], "parked");
    }
}
