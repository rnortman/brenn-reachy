//! The seam intent bodies arrive through, from wherever in the process they
//! were authored.
//!
//! Two sources hand the edge a body — the scripter, in-process, and a delivery
//! off the bus — and both of them run on tasks that must not be waiting on a
//! socket or on the story loop. So they hand the body to a queue, and the loop
//! that owns the edge takes it from there. That is the whole of this module:
//! one bounded queue, and the rule about what happens when it is full.
//!
//! A body handed over here wakes the loop rather than waiting for it. The
//! handle sends an empty datagram to the reports port the loop is asleep in, so
//! the wake word's path to the motors is a compile and one loopback datagram
//! and not a quarter second of read timeout on top. That path is the reason
//! this process is on the robot at all, and a fixed delay on it would be a
//! fixed delay nothing could configure away.
//!
//! Bounded and dropping, deliberately. A queue that grew would trade the
//! machine's memory for bodies whose moment has passed: a presence script is
//! refreshed every few seconds and the latest one is the only one that matters,
//! so a backlog is a queue of stale intent. A queue that blocked would put the
//! speech pipeline's turn lifecycle behind the story loop's next wake-up. A
//! dropped body is narrated by the sender's own handle and superseded by the
//! next refresh.

use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::mpsc::{Receiver, SyncSender, TrySendError, sync_channel};

/// How many bodies may be waiting for the loop at once.
///
/// Small on purpose. The loop drains the queue on every pass and every offer
/// wakes it, and one sender authority per pod refreshes every few seconds, so
/// anything past a handful is a sender that has stopped agreeing with the edge
/// about what a refresh cadence is.
pub const INTENT_BACKLOG: usize = 8;

/// The sending half: what a body-authoring task holds.
///
/// Cloneable, because the two sources are two tasks and both hand bodies to one
/// gate. Cloning the handle does not clone the edge — there is exactly one of
/// those, at the other end of this queue.
#[derive(Clone, Debug)]
pub struct Intents {
    bodies: SyncSender<Vec<u8>>,
    /// What an offer nudges the loop awake through, where the caller gave one.
    nudge: Option<Arc<UdpSocket>>,
}

/// Why a body did not reach the edge at all.
///
/// Neither of these is a refusal — the edge never saw the body — and neither is
/// retried here. They are said by the caller and superseded by the next
/// refresh.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum NotOffered {
    /// The queue was full: the loop is behind, or a sender is emitting faster
    /// than a refresh cadence.
    #[error("the intent queue holds {INTENT_BACKLOG} bodies already")]
    Backlogged,

    /// The loop is gone. In a live process this is the shutdown path, and a
    /// body offered during it has nothing to run on.
    #[error("the edge loop has stopped")]
    Stopped,
}

impl Intents {
    /// Hand a body to the edge, or say why it did not get there.
    ///
    /// Never waits. The caller is a speech pipeline task or a bus delivery, and
    /// neither may be parked behind the loop's next wake-up.
    ///
    /// # Errors
    ///
    /// [`NotOffered`] when the queue is full or the loop has stopped.
    pub fn offer(&self, body: Vec<u8>) -> Result<(), NotOffered> {
        self.bodies.try_send(body).map_err(|error| match error {
            TrySendError::Full(_) => NotOffered::Backlogged,
            TrySendError::Disconnected(_) => NotOffered::Stopped,
        })?;
        if let Some(nudge) = &self.nudge {
            // Best effort, and deliberately not an error: the body is already
            // queued, and a nudge that did not go out costs it one read timeout
            // rather than the run. Empty, because the loop reads the length and
            // nothing else — a nudge is not a story and is never followed as
            // one.
            let _ = nudge.send(&[]);
        }
        Ok(())
    }
}

/// The receiving half: what the loop that owns the edge holds.
#[derive(Debug)]
pub struct Waiting {
    bodies: Receiver<Vec<u8>>,
}

impl Waiting {
    /// The next body, or nothing at all. Never waits.
    ///
    /// A disconnected queue reads the same as an empty one: every sending
    /// handle being gone is not a reason for the loop to stop following the
    /// session's story, which is the half of this process that says what the
    /// machine is doing.
    #[must_use]
    pub fn next(&self) -> Option<Vec<u8>> {
        self.bodies.try_recv().ok()
    }
}

/// The queue, both ends, with nothing to wake.
///
/// What a caller that drains the queue itself holds — a test, or a process with
/// no reports port to nudge.
#[must_use]
pub fn queue() -> (Intents, Waiting) {
    build(None)
}

/// The queue, both ends, where an offer wakes the loop.
///
/// `nudge` is a socket already connected to wherever the loop that owns the
/// edge is asleep — its reports port. Connected rather than addressed here so
/// that this module names no port: what an offer sends is empty, and where it
/// goes is the caller's.
#[must_use]
pub fn waking_queue(nudge: Arc<UdpSocket>) -> (Intents, Waiting) {
    build(Some(nudge))
}

fn build(nudge: Option<Arc<UdpSocket>>) -> (Intents, Waiting) {
    let (bodies, waiting) = sync_channel(INTENT_BACKLOG);
    (Intents { bodies, nudge }, Waiting { bodies: waiting })
}

#[cfg(test)]
mod tests {
    use std::net::UdpSocket;
    use std::sync::Arc;
    use std::time::Duration;

    use super::{INTENT_BACKLOG, NotOffered, queue, waking_queue};

    #[test]
    fn a_body_offered_is_a_body_waiting() {
        let (intents, waiting) = queue();
        intents.offer(b"one".to_vec()).expect("room for one");
        intents.offer(b"two".to_vec()).expect("room for two");
        assert_eq!(waiting.next(), Some(b"one".to_vec()));
        assert_eq!(waiting.next(), Some(b"two".to_vec()));
        assert_eq!(waiting.next(), None);
    }

    #[test]
    fn a_full_queue_drops_rather_than_waits() {
        let (intents, waiting) = queue();
        for _ in 0..INTENT_BACKLOG {
            intents.offer(b"body".to_vec()).expect("room");
        }
        assert_eq!(intents.offer(b"body".to_vec()), Err(NotOffered::Backlogged));
        // Draining one makes room: the queue is a backlog, not a latch.
        assert!(waiting.next().is_some());
        intents.offer(b"body".to_vec()).expect("room again");
    }

    #[test]
    fn a_loop_that_has_gone_is_said_rather_than_waited_for() {
        let (intents, waiting) = queue();
        drop(waiting);
        assert_eq!(intents.offer(b"body".to_vec()), Err(NotOffered::Stopped));
    }

    /// The wake path: an offer does not wait for the loop's read timeout, and
    /// what it sends carries nothing. Two ephemeral ports rather than the
    /// reports port, because a test must not contend with a host or a harness
    /// on this machine, and the nudge socket is connected either way.
    #[test]
    fn an_offer_nudges_whatever_is_asleep_on_the_reports_port() {
        let asleep = UdpSocket::bind("127.0.0.1:0").expect("an ephemeral port");
        asleep
            .set_read_timeout(Some(Duration::from_millis(250)))
            .expect("a read timeout");
        let sender = Arc::new(UdpSocket::bind("127.0.0.1:0").expect("an ephemeral port"));
        sender
            .connect(asleep.local_addr().expect("its own address"))
            .expect("a loopback peer");
        let (intents, waiting) = waking_queue(Arc::clone(&sender));
        intents.offer(b"body".to_vec()).expect("room for one");
        assert!(waiting.next().is_some());

        // The nudge is empty, which is what tells the loop it is not a story.
        let mut buffer = [0u8; 8];
        let (read, _) = asleep.recv_from(&mut buffer).expect("the nudge");
        assert_eq!(read, 0);
    }

    #[test]
    fn a_queue_whose_senders_are_gone_still_reads_empty() {
        let (intents, waiting) = queue();
        intents.offer(b"body".to_vec()).expect("room");
        drop(intents);
        assert_eq!(waiting.next(), Some(b"body".to_vec()));
        assert_eq!(waiting.next(), None);
    }
}
