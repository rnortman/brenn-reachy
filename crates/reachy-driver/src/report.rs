//! What a cycle has to say about itself, before a host publishes it.
//!
//! A driver cycle can hit more than one reportable edge and publishes one, and
//! the ranking between them is a decision rather than a sequence of writes into
//! a message. That decision lives here for the reason every other decision in
//! this crate does: the simulated driver and the real one raising different
//! events out of the same cycle would be two machines, and a fault one of them
//! reported would not mean what the other's did.
//!
//! What is here is the event and the choosing: the fields the vocabulary names,
//! the one-slot ranking, and the blind-cycle run counter that turns a silent bus
//! into the one fault a driver raises about itself. What is not here is where
//! the present rows came from or how a record is published — that is the half
//! that genuinely differs between a state slot and a datagram.

use brenn_reachy__driver__health_clk_rs::{DriverEvent, EventKind};
use brenn_reachy__motion__joints_clk_rs::JointFlags;
use clockwork_rs::{Duration, SyncTime};

use crate::BLIND_CYCLES_BEFORE_BUS_FAILURE;

/// One edge a cycle hit, held as ordinary Rust until it is published.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Event {
    /// Which edge.
    pub kind: EventKind,
    /// The grid instant it was raised at.
    pub time_ns: i64,
    /// The silence or the lateness, where the kind names one.
    pub silence_ns: i64,
    /// How long the cycle the kind names took, where something measured it.
    /// Nothing in this crate does: every span on this field is read off a clock
    /// by the host that owns one, and a raiser here leaves the zero that says so.
    pub work_ns: i64,
    /// How long the out-of-band exchange the kind names took, where something
    /// measured it. Nothing in this crate does, for the reason `work_ns` states.
    pub exchange_ns: i64,
    /// How many of whatever the kind counts.
    pub count: u32,
    /// How many of the cycles a windowed kind counted ran an out-of-band
    /// transaction.
    pub out_of_band: u32,
    /// The servos the kind names, where it names a set of them.
    pub rows: JointFlags,
    /// The one servo the kind names, as its bus id.
    pub id: u8,
}

impl Event {
    /// An event at `time_ns` with no evidence yet: a raiser fills in the fields
    /// its own kind names and leaves the rest, which is what the schema says a
    /// kind that does not name a field carries.
    #[must_use]
    pub fn at(time_ns: i64) -> Self {
        Self {
            kind: EventKind::None,
            time_ns,
            silence_ns: 0,
            work_ns: 0,
            exchange_ns: 0,
            count: 0,
            out_of_band: 0,
            rows: JointFlags::NONE,
            id: 0,
        }
    }

    /// Write this event into the message that carries it.
    ///
    /// Every field, always: a message is written whole rather than field by
    /// field as a cycle goes, so a kind that names no evidence carries the
    /// zeroes that say so rather than whatever the last event left behind.
    pub fn write(&self, out: &mut DriverEvent) {
        out.kind = self.kind;
        out.time = SyncTime::from_nanos(self.time_ns);
        out.silence = Duration::from_nanos(self.silence_ns);
        out.work = Duration::from_nanos(self.work_ns);
        out.exchange = Duration::from_nanos(self.exchange_ns);
        out.rows = self.rows;
        out.count = self.count;
        out.out_of_band = self.out_of_band;
        out.id = self.id;
    }
}

/// Offer `event` for a cycle's one slot, counting the one it displaces.
///
/// The dead-man's latch outranks the rest: it is the machine changing state,
/// where the others are remarks about a datagram whose sender can see the
/// consequences anyway.
pub fn raise(slot: &mut Option<Event>, dropped: &mut u64, event: Event) {
    match *slot {
        None => *slot = Some(event),
        Some(held) => {
            *dropped += 1;
            if event.kind == EventKind::HoldTimeoutTorqueOff && held.kind != event.kind {
                *slot = Some(event);
            }
        }
    }
}

/// Count how long the bus has been answering nothing, and answer with the event
/// once the run is long enough to mean the bus is gone.
///
/// The one fault a driver raises about itself, and the run of blind cycles is
/// all the evidence there is: a cycle that read no row read nothing about what
/// is wrong with it either. `run` goes back to zero on the first cycle that
/// reads something, and the event comes back on the cycle the run reaches its
/// length and not again while it continues — a standing outage is not news, and
/// a host told once has already stopped trusting the bus.
pub fn count_blind(run: &mut u32, blind: bool, nominal_ns: i64) -> Option<Event> {
    if !blind {
        *run = 0;
        return None;
    }
    *run = run.saturating_add(1);
    if *run != BLIND_CYCLES_BEFORE_BUS_FAILURE {
        return None;
    }
    Some(Event {
        kind: EventKind::BusFailure,
        // How many cycles went unanswered, which is what says whether the
        // report is about the threshold or about a bus that has been gone far
        // longer. No `id`: a run of silence names no single failing servo.
        count: *run,
        ..Event::at(nominal_ns)
    })
}

#[cfg(test)]
mod tests {
    use super::{Event, count_blind, raise};
    use crate::BLIND_CYCLES_BEFORE_BUS_FAILURE;
    use brenn_reachy__driver__health_clk_rs::EventKind;

    /// A round instant, so a number read out of the wrong field is visible.
    const T0: i64 = 1_700_000_000_000_000_000;

    fn kinded(kind: EventKind) -> Event {
        Event {
            kind,
            ..Event::at(T0)
        }
    }

    #[test]
    fn the_first_event_of_a_cycle_takes_the_slot() {
        let mut slot = None;
        let mut dropped = 0;

        raise(&mut slot, &mut dropped, kinded(EventKind::CycleSkipped));

        assert_eq!(slot.map(|event| event.kind), Some(EventKind::CycleSkipped));
        assert_eq!(dropped, 0);
    }

    #[test]
    fn a_remark_never_displaces_what_the_cycle_already_has() {
        let mut slot = None;
        let mut dropped = 0;

        raise(&mut slot, &mut dropped, kinded(EventKind::CycleSkipped));
        raise(
            &mut slot,
            &mut dropped,
            kinded(EventKind::GoalDroppedQueueFull),
        );

        assert_eq!(
            slot.map(|event| event.kind),
            Some(EventKind::CycleSkipped),
            "the first answer wins between two remarks"
        );
        assert_eq!(dropped, 1, "the displaced one is counted, never silent");
    }

    #[test]
    fn the_dead_mans_latch_displaces_a_remark_and_not_itself() {
        let mut slot = None;
        let mut dropped = 0;

        raise(
            &mut slot,
            &mut dropped,
            kinded(EventKind::GoalStaleOrOutOfOrder),
        );
        raise(
            &mut slot,
            &mut dropped,
            kinded(EventKind::HoldTimeoutTorqueOff),
        );
        assert_eq!(
            slot.map(|event| event.kind),
            Some(EventKind::HoldTimeoutTorqueOff),
            "the machine changing state outranks a remark about a datagram"
        );

        let latched_at = slot.map(|event| event.time_ns);
        raise(
            &mut slot,
            &mut dropped,
            Event {
                kind: EventKind::HoldTimeoutTorqueOff,
                ..Event::at(T0 + 1)
            },
        );
        assert_eq!(
            slot.map(|event| event.time_ns),
            latched_at,
            "a latch does not displace a latch"
        );
        assert_eq!(dropped, 2);
    }

    #[test]
    fn a_bus_that_answers_is_a_run_of_nothing() {
        let mut run = 7;

        assert_eq!(count_blind(&mut run, false, T0), None);
        assert_eq!(run, 0, "the run ends on the first cycle that reads a row");
    }

    #[test]
    fn a_silent_bus_is_announced_once_at_the_threshold_and_not_after() {
        let mut run = 0;
        let mut raised = Vec::new();
        for cycle in 0..3 * BLIND_CYCLES_BEFORE_BUS_FAILURE {
            if let Some(event) = count_blind(&mut run, true, T0 + i64::from(cycle)) {
                raised.push(event);
            }
        }

        assert_eq!(raised.len(), 1, "a standing outage is not news");
        let event = raised[0];
        assert_eq!(event.kind, EventKind::BusFailure);
        assert_eq!(
            event.count, BLIND_CYCLES_BEFORE_BUS_FAILURE,
            "the run of unanswered cycles is the evidence"
        );
        assert_eq!(event.id, 0, "a run of silence names no failing servo");
        assert_eq!(
            event.time_ns,
            T0 + i64::from(BLIND_CYCLES_BEFORE_BUS_FAILURE) - 1,
            "raised on the cycle the run reached its length"
        );
    }
}
