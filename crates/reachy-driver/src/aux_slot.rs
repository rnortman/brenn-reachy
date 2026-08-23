//! The auxiliary transaction slot: which one out-of-band transaction a driver
//! cycle spends its bus time on.
//!
//! A driver cycle's bus budget is mostly the goal write and the proprioception
//! read. Whatever is left is one transaction, and three things want it: a host
//! asking for a register read or a verified write, the confirmation pass reading
//! torque-enable back after a de-torquing, and the health rotation walking the
//! servos to see whether any of them is complaining. [`AuxSlot`] is the policy
//! that picks, and nothing else — the host executes the transaction it names,
//! decides which register that means, and publishes whatever answer comes back.
//!
//! Why one per cycle: the bus is one wire and a transaction is a round trip on
//! it. Two would mean a cycle that sometimes overruns its period, which on this
//! machine means a late goal write, which is a stutter in the motion. One is the
//! budget that is always affordable.
//!
//! The order is host request, then confirmation, then rotation. The host's
//! request goes first because there is at most one of them outstanding — the
//! session that issues them is strictly serial — so serving it costs the
//! confirmation one cycle and never starves it, while the other order could
//! stall a sequencer's transaction for as long as a de-torquing stays
//! unconfirmed, which is unbounded. The rotation goes last because it is
//! surveillance: it has a cadence, not a deadline, and a lap that takes a few
//! cycles longer than nominal is not a fact anybody acts on.

use brenn_reachy__driver__aux_clk_rs::{AuxSlotState, AuxSlotStateWire};
use brenn_reachy__motion__bus_txn_clk_rs::BusTxn;
use clockwork_rs::SyncTime;

use crate::JOINT_COUNT;
use crate::state::DriverStateError;
use crate::torque::BelievedTorqued;

/// What the host should spend this cycle's one aux transaction on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum AuxTask {
    /// Nothing: no request is pending, no confirmation is running, and the
    /// rotation is not due.
    #[default]
    Nothing,
    /// Execute the transaction [`AuxSlot::taken`] hands back for this `corr`
    /// and answer it with an outcome carrying the same number.
    ///
    /// The transaction itself stays in the state rather than riding here: it is
    /// the vocabulary's own record, which is a buffer of bytes and not a value
    /// a task arm can carry without copying it.
    Host {
        /// The host's correlation number, which the outcome echoes.
        corr: u32,
    },
    /// Read this row's torque-enable register back, for the confirmation pass.
    ConfirmTorqueOff {
        /// The bus row.
        row: u8,
    },
    /// Read this row's status registers and publish a health report.
    Health {
        /// The bus row.
        row: u8,
    },
}

/// What became of a host request offered to the slot.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AuxOffer {
    /// Held, and named by the next [`AuxSlot::take`].
    Accepted,
    /// Refused: a request is already pending.
    ///
    /// The queue is one deep because the host that fills it is serial by
    /// construction — it issues one transaction and waits for its outcome — so
    /// a second request arriving is a host that is not what it claims to be. The
    /// refusal is loud on purpose: it comes back as an outcome with a refused
    /// status, against the `corr` of the request that was turned away, rather
    /// than being absorbed into a deeper queue where the two would silently
    /// interleave.
    RefusedBusy,
}

/// The slot: one pending host request, the rotation's place in its lap, and the
/// torque belief the answers move — decided over the state they live in.
///
/// Borrows the state for as long as a decision is being made and holds nothing
/// of its own, so a host keeps its state wherever it keeps state — a cog's slot
/// or a process's memory — and every read a host wants is a field of that state
/// rather than something to ask for here.
pub struct AuxSlot<'a> {
    /// The state being decided over.
    state: &'a mut AuxSlotState,
}

impl<'a> AuxSlot<'a> {
    /// Take up a slot as a driver that has been told nothing.
    ///
    /// The producer's route: the state is cleared, which is no request pending,
    /// the rotation at row 0 and due, and nothing believed torqued.
    pub fn start(slot: &'a mut AuxSlotStateWire) -> Self {
        Self {
            state: slot.clear_valid(),
        }
    }

    /// Decide over the state a previous cycle left.
    pub fn over(state: &'a mut AuxSlotState) -> Self {
        Self { state }
    }

    /// The state being decided over.
    #[must_use]
    pub fn state(&self) -> &AuxSlotState {
        self.state
    }

    /// The torque belief this state carries, for one decision.
    ///
    /// The belief lives here because the answers that move it are this slot's:
    /// a verified write to a torque-enable register is one of the transactions
    /// the slot hands out.
    pub fn belief(&mut self) -> BelievedTorqued<'_> {
        BelievedTorqued::over(&mut self.state.believed_torqued)
    }

    /// Whether this describes a slot a driver can be in.
    ///
    /// What the schema cannot say: which values of a row cursor name a bus row.
    /// Everything else about these bytes — that the pending record is a
    /// transaction at all, that the belief names servos this bus has — is said
    /// by the host's one validation at the boundary, because every one of those
    /// fields is typed as what it means.
    ///
    /// # Errors
    ///
    /// [`DriverStateError::HealthCursorOutOfRange`] for a rotation cursor past
    /// the bus.
    pub fn validate(state: &AuxSlotState) -> Result<(), DriverStateError> {
        if usize::from(state.next_row) >= JOINT_COUNT {
            return Err(DriverStateError::HealthCursorOutOfRange {
                row: state.next_row,
            });
        }
        Ok(())
    }

    /// The row the rotation will read next: what `next_row` says, or row 0 if
    /// it says something past the bus.
    ///
    /// Every read of the cursor goes through this, so a host that skipped
    /// [`Self::validate`] gets a rotation that rotates rather than a panic in
    /// the process whose other job is to de-torque the machine.
    fn rotation_row(&self) -> u8 {
        if usize::from(self.state.next_row) < JOINT_COUNT {
            self.state.next_row
        } else {
            0
        }
    }

    /// Offer a host request to the slot, under the host's correlation number.
    ///
    /// The record is copied in rather than borrowed: the state outlives the
    /// cycle that offered it, and a re-issue must be the same transaction.
    pub fn offer(&mut self, corr: u32, request: &BusTxn) -> AuxOffer {
        if self.state.has_pending.get() {
            return AuxOffer::RefusedBusy;
        }
        crate::copy_whole(&mut self.state.pending, request);
        self.state.corr = corr;
        self.state.has_pending = true.into();
        AuxOffer::Accepted
    }

    /// The transaction taken under `corr`, or `None` where the slot no longer
    /// holds it.
    ///
    /// Held rather than handed over, because a record is a buffer: the host
    /// runs it where it lies. The correlation number is the ask because the
    /// slot is free to take another offer the moment [`Self::take`] names one:
    /// a host that took `AuxTask::Host { corr }` and comes back for the
    /// transaction after a later offer landed gets nothing rather than the
    /// other host's transaction answered under its number.
    #[must_use]
    pub fn taken(&self, corr: u32) -> Option<&BusTxn> {
        (self.state.corr == corr).then_some(&self.state.pending)
    }

    /// Drop the pending host request, if one is held, and say whether there was
    /// one.
    ///
    /// What a commanded de-torquing does to the queue. A release commanded after
    /// a request was offered outranks it: a pending verified torque-enable write
    /// run after the latch stands would energise the row again and stand the
    /// confirmation pass down, so the machine a host asked to let go would be
    /// holding torque until the host's next release datagram. Nothing goes back
    /// to the host for it — the transaction never reached the bus, which is the
    /// silence its own delivery timeout is for, and the release it was overtaken
    /// by is the answer to whatever the sequence was doing.
    pub fn abandon(&mut self) -> bool {
        let held = self.state.has_pending.get();
        self.state.has_pending = false.into();
        held
    }

    /// Whether the health rotation is due at `now_ns`.
    ///
    /// A stamp ahead of `now_ns` is due immediately: it is not a stamp this
    /// clock made, so there is no elapsed time to measure against it, and
    /// waiting for the clock to catch up would stop surveillance for as long as
    /// the difference — silently, since nothing downstream watches the
    /// rotation's cadence.
    #[must_use]
    pub fn health_due(&self, now_ns: i64, health_period_ns: i64) -> bool {
        let last = self.state.last_report.as_nanos();
        !self.state.has_reported.get()
            || last > now_ns
            || now_ns.saturating_sub(last) >= health_period_ns
    }

    /// Name this cycle's one transaction.
    ///
    /// `confirm_row` is the row the torque-off confirmation is waiting on, if
    /// one is running — [`crate::TorqueOffConfirm::waiting_on`]. Must be a bus
    /// row or `None`; this takes it as given rather than re-deciding which rows
    /// exist.
    /// `health_period_ns` is the minimum spacing between successive health
    /// reports; the rotation advances one row per report, so a whole lap takes
    /// the bus row count times that.
    ///
    /// A pending host request is consumed here: the transaction is now the
    /// host's to execute and answer, and the slot is free for the next one. A
    /// scheduled health read stamps the cadence here too, for the same reason —
    /// the ask *is* the report as far as spacing goes, and a driver that
    /// re-stamped only on a successful reply would poll a silent servo every
    /// cycle.
    pub fn take(&mut self, now_ns: i64, health_period_ns: i64, confirm_row: Option<u8>) -> AuxTask {
        if self.state.has_pending.get() {
            self.state.has_pending = false.into();
            return AuxTask::Host {
                corr: self.state.corr,
            };
        }
        if let Some(row) = confirm_row {
            return AuxTask::ConfirmTorqueOff { row };
        }
        if self.health_due(now_ns, health_period_ns) {
            let row = self.rotation_row();
            self.state.next_row = (row + 1) % JOINT_COUNT as u8;
            self.state.last_report = SyncTime::from_nanos(now_ns);
            self.state.has_reported = true.into();
            return AuxTask::Health { row };
        }
        AuxTask::Nothing
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use brenn_reachy__driver__aux_clk_rs::{AuxSlotStateWire, TorqueConfirmStateWire};
    use brenn_reachy__hardware__dynamixel__registers_clk_rs::{RegId, ValueShape};
    use brenn_reachy__motion__bus_txn_clk_rs::{AuxOpKind, BusTxnWire};
    use brenn_reachy__motion__joints_clk_rs::JointFlagsWire;
    use clockwork_rs::{as_raw, blob_as_bytes};

    const T0: i64 = 1_700_000_000_000_000_000;
    const PERIOD: i64 = 20_000_000;
    const HEALTH_PERIOD: i64 = 120_000_000;

    /// A slot's state and the record it lives in, so a case can hold one value
    /// and still hand out the borrow every decision takes.
    struct Fixture {
        /// The state the slot decides over.
        held: AuxSlotStateWire,
    }

    impl Fixture {
        /// A driver that has been told nothing.
        fn new() -> Self {
            let mut held = AuxSlotStateWire::new();
            AuxSlot::start(&mut held);
            Self { held }
        }

        /// The slot, for as long as a case needs it.
        fn slot(&mut self) -> AuxSlot<'_> {
            AuxSlot::over(
                self.held
                    .validate_mut()
                    .expect("a slot leaves a slot in its state"),
            )
        }
    }

    fn read_request(id: u8) -> BusTxnWire {
        let mut txn = BusTxnWire::new();
        let held = txn.clear_valid();
        held.active = true.into();
        held.op = AuxOpKind::ReadReg;
        held.id = id;
        held.reg = RegId::PresentPosition;
        txn
    }

    /// A request as the slot takes one: validated, since the state it is copied
    /// into is.
    fn as_txn(txn: &BusTxnWire) -> &BusTxn {
        txn.validate().expect("a request is a transaction")
    }

    /// Two records are the same transaction when they are the same bytes.
    fn same_txn(held: Option<&BusTxn>, expected: &BusTxnWire) -> bool {
        held.map(|txn| blob_as_bytes(as_raw(txn))) == Some(blob_as_bytes(expected))
    }

    #[test]
    fn a_fresh_slot_is_a_slot_and_the_rotation_is_due() {
        let mut fixture = Fixture::new();
        let slot = fixture.slot();
        AuxSlot::validate(slot.state).expect("a fresh slot is a slot");
        assert!(!slot.state.has_pending.get());
        assert!(slot.health_due(T0, HEALTH_PERIOD));
        assert_eq!(slot.rotation_row(), 0);
    }

    /// A pending request abandoned is one the next `take` does not name -- and
    /// abandoning nothing says so.
    ///
    /// What a commanded de-torquing does to the queue. The transaction being
    /// dropped is the point: a torque-enable write run out of the slot after the
    /// latch stands would energise the row the release exists to let go of.
    #[test]
    fn an_abandoned_request_is_not_taken() {
        let mut fixture = Fixture::new();
        let mut slot = fixture.slot();
        let request = read_request(3);
        assert_eq!(slot.offer(7, as_txn(&request)), AuxOffer::Accepted);
        assert!(slot.abandon(), "there was one to abandon");
        assert!(!slot.abandon(), "and now there is not");
        assert_eq!(
            slot.take(T0, HEALTH_PERIOD, None),
            AuxTask::Health { row: 0 },
            "the cycle spends its transaction on what is left to want it"
        );
        assert_eq!(
            slot.offer(8, as_txn(&request)),
            AuxOffer::Accepted,
            "and the slot is free for the request the host issues next"
        );
    }

    #[test]
    fn a_pending_host_request_outranks_the_rotation() {
        let mut fixture = Fixture::new();
        let mut slot = fixture.slot();
        let request = read_request(3);
        assert_eq!(slot.offer(7, as_txn(&request)), AuxOffer::Accepted);
        assert_eq!(
            slot.take(T0, HEALTH_PERIOD, None),
            AuxTask::Host { corr: 7 },
            "the rotation was due and the request still went first"
        );
        assert!(same_txn(slot.taken(7), &request));
        // The rotation kept its place: serving the host did not consume a lap
        // step, and it is still due.
        assert!(slot.health_due(T0, HEALTH_PERIOD));
        assert_eq!(
            slot.take(T0, HEALTH_PERIOD, None),
            AuxTask::Health { row: 0 }
        );
    }

    #[test]
    fn a_pending_host_request_outranks_the_confirmation_by_one_cycle() {
        let mut fixture = Fixture::new();
        let mut slot = fixture.slot();
        let request = read_request(1);
        slot.offer(1, as_txn(&request));
        assert_eq!(
            slot.take(T0, HEALTH_PERIOD, Some(4)),
            AuxTask::Host { corr: 1 }
        );
        assert_eq!(
            slot.take(T0 + PERIOD, HEALTH_PERIOD, Some(4)),
            AuxTask::ConfirmTorqueOff { row: 4 },
            "one cycle later the confirmation has the slot"
        );
    }

    #[test]
    fn the_confirmation_outranks_the_rotation_for_as_long_as_it_runs() {
        let mut fixture = Fixture::new();
        let mut slot = fixture.slot();
        let mut now = T0;
        for row in 0..JOINT_COUNT as u8 {
            assert_eq!(
                slot.take(now, HEALTH_PERIOD, Some(row)),
                AuxTask::ConfirmTorqueOff { row }
            );
            now += PERIOD;
        }
        assert!(
            slot.health_due(now, HEALTH_PERIOD),
            "the rotation was starved and is still owed a report"
        );
        assert_eq!(
            slot.take(now, HEALTH_PERIOD, None),
            AuxTask::Health { row: 0 }
        );
    }

    /// A slot whose rotation has already been served, so what it names next is
    /// only ever a request or a confirmation. The rotation is due on a fresh
    /// slot, which would otherwise answer every quiet cycle.
    fn rotated(fixture: &mut Fixture, now_ns: i64) -> AuxSlot<'_> {
        let mut slot = fixture.slot();
        assert_eq!(
            slot.take(now_ns, i64::MAX, None),
            AuxTask::Health { row: 0 },
            "a fresh slot owes its first report"
        );
        slot
    }

    #[test]
    fn one_request_is_taken_once() {
        let mut fixture = Fixture::new();
        let mut slot = rotated(&mut fixture, T0);
        let request = read_request(2);
        slot.offer(9, as_txn(&request));
        assert_eq!(
            slot.take(T0 + PERIOD, i64::MAX, None),
            AuxTask::Host { corr: 9 }
        );
        assert_eq!(slot.take(T0 + 2 * PERIOD, i64::MAX, None), AuxTask::Nothing);
    }

    /// A host that comes back for its transaction after the slot took another
    /// offer gets nothing. Taking frees the slot, so the record under it is the
    /// newer request's — and answering that one under the older correlation
    /// number would put a register write nobody asked for on the bus.
    #[test]
    fn a_transaction_asked_for_under_a_spent_correlation_is_not_handed_over() {
        let mut fixture = Fixture::new();
        let mut slot = rotated(&mut fixture, T0);
        let first = read_request(1);
        slot.offer(7, as_txn(&first));
        assert_eq!(
            slot.take(T0 + PERIOD, i64::MAX, None),
            AuxTask::Host { corr: 7 }
        );
        assert_eq!(slot.offer(8, as_txn(&read_request(2))), AuxOffer::Accepted);
        assert_eq!(
            slot.taken(7),
            None,
            "the slot no longer holds the transaction taken under 7"
        );
        assert_eq!(
            slot.take(T0 + 2 * PERIOD, i64::MAX, None),
            AuxTask::Host { corr: 8 }
        );
        assert!(same_txn(slot.taken(8), &read_request(2)));
    }

    #[test]
    fn a_second_request_while_one_is_pending_is_refused() {
        let mut fixture = Fixture::new();
        let mut slot = rotated(&mut fixture, T0);
        assert_eq!(slot.offer(1, as_txn(&read_request(1))), AuxOffer::Accepted);
        assert_eq!(
            slot.offer(2, as_txn(&read_request(2))),
            AuxOffer::RefusedBusy
        );
        assert_eq!(
            slot.take(T0 + PERIOD, i64::MAX, None),
            AuxTask::Host { corr: 1 },
            "the first request is what is held; the second was turned away"
        );
        assert!(
            same_txn(slot.taken(1), &read_request(1)),
            "and it is the first request's transaction, not the second's"
        );
        assert_eq!(slot.offer(2, as_txn(&read_request(2))), AuxOffer::Accepted);
    }

    #[test]
    fn a_request_carrying_a_value_crosses_the_slot_unchanged() {
        let mut fixture = Fixture::new();
        let mut slot = rotated(&mut fixture, T0);
        let mut request = BusTxnWire::new();
        let held = request.clear_valid();
        held.active = true.into();
        held.op = AuxOpKind::WriteRegVerified;
        held.id = 5;
        held.reg = RegId::GoalPosition;
        held.value_kind = ValueShape::Radians;
        held.value = 0.5f64.to_bits();
        slot.offer(42, as_txn(&request));
        assert_eq!(
            slot.take(T0 + PERIOD, i64::MAX, None),
            AuxTask::Host { corr: 42 }
        );
        assert!(
            same_txn(slot.taken(42), &request),
            "every field of it, the value included"
        );
    }

    #[test]
    fn the_rotation_walks_every_row_once_per_lap_and_wraps() {
        let mut fixture = Fixture::new();
        let mut slot = fixture.slot();
        let mut now = T0;
        let mut rows = Vec::new();
        for _ in 0..2 * JOINT_COUNT {
            match slot.take(now, HEALTH_PERIOD, None) {
                AuxTask::Health { row } => rows.push(row),
                other => panic!("the rotation was due and answered {other:?}"),
            }
            now += HEALTH_PERIOD;
        }
        let lap: Vec<u8> = (0..JOINT_COUNT as u8).collect();
        assert_eq!(rows[..JOINT_COUNT], lap[..]);
        assert_eq!(rows[JOINT_COUNT..], lap[..], "the second lap is the first");
    }

    #[test]
    fn the_rotation_is_not_due_again_until_a_whole_period_has_passed() {
        let mut fixture = Fixture::new();
        let mut slot = fixture.slot();
        assert_eq!(
            slot.take(T0, HEALTH_PERIOD, None),
            AuxTask::Health { row: 0 }
        );
        let mut now = T0 + PERIOD;
        while now < T0 + HEALTH_PERIOD {
            assert_eq!(
                slot.take(now, HEALTH_PERIOD, None),
                AuxTask::Nothing,
                "a report at {now} is inside the cadence"
            );
            now += PERIOD;
        }
        assert_eq!(
            slot.take(T0 + HEALTH_PERIOD, HEALTH_PERIOD, None),
            AuxTask::Health { row: 1 }
        );
    }

    #[test]
    fn a_lap_is_a_row_count_of_report_periods() {
        // One report per period, one row per report: the last row of the first
        // lap comes eight periods after the first.
        let mut fixture = Fixture::new();
        let mut slot = fixture.slot();
        let mut now = T0;
        loop {
            if let AuxTask::Health { row } = slot.take(now, HEALTH_PERIOD, None)
                && row == JOINT_COUNT as u8 - 1
            {
                break;
            }
            now += PERIOD;
        }
        assert_eq!(
            now - T0,
            (JOINT_COUNT as i64 - 1) * HEALTH_PERIOD,
            "the last row of the first lap"
        );
    }

    /// The slot takes the confirmation's row as given, so what keeps an off-bus
    /// row out of a transaction is the answer it is given: a confirmation
    /// carrying an impossible cursor reads the bus from row 0, and the row it
    /// names is a row the slot can ask for.
    #[test]
    fn a_confirmation_with_a_cursor_past_the_bus_reads_from_row_zero() {
        let mut held = TorqueConfirmStateWire::new();
        held.set_active(true);
        held.set_started(SyncTime::from_nanos(T0));
        held.set_cursor(JOINT_COUNT as u8 + 3);
        let confirm = crate::TorqueOffConfirm::over(
            held.validate_mut()
                .expect("a cursor is a number either way"),
        );
        assert_eq!(confirm.waiting_on(), Some(0), "the pass starts over");
        let confirm_row = confirm.waiting_on();
        let mut fixture = Fixture::new();
        let mut slot = rotated(&mut fixture, T0);
        assert_eq!(
            slot.take(T0 + PERIOD, i64::MAX, confirm_row),
            AuxTask::ConfirmTorqueOff { row: 0 },
            "the confirmation outranks a rotation that is not due"
        );
    }

    /// A stamp ahead of the clock is not a stamp: the rotation is due at once
    /// rather than waiting for the clock to reach it, which for an i64 of
    /// nanoseconds can be never — and nothing downstream watches the cadence, so
    /// the surveillance would stop in silence.
    #[test]
    fn a_report_stamp_from_the_future_is_due_now() {
        let mut fixture = Fixture::new();
        let mut slot = fixture.slot();
        slot.state.last_report = SyncTime::from_nanos(T0 + 10 * HEALTH_PERIOD);
        slot.state.has_reported = true.into();
        assert!(slot.health_due(T0, HEALTH_PERIOD));
        assert_eq!(
            slot.take(T0, HEALTH_PERIOD, None),
            AuxTask::Health { row: 0 }
        );
        assert_eq!(
            slot.state.last_report.as_nanos(),
            T0,
            "the ask stamps this clock's now"
        );
        assert!(
            !slot.health_due(T0 + PERIOD, HEALTH_PERIOD),
            "and the cadence runs from it"
        );
    }

    #[test]
    fn a_rotation_cursor_past_the_bus_is_refused_and_reads_as_row_zero() {
        let mut fixture = Fixture::new();
        let mut slot = fixture.slot();
        slot.state.next_row = JOINT_COUNT as u8 + 3;
        assert_eq!(
            AuxSlot::validate(slot.state),
            Err(DriverStateError::HealthCursorOutOfRange {
                row: JOINT_COUNT as u8 + 3
            })
        );
        assert_eq!(slot.rotation_row(), 0);
        assert_eq!(
            slot.take(T0, HEALTH_PERIOD, None),
            AuxTask::Health { row: 0 }
        );
        AuxSlot::validate(slot.state).expect("taking a task re-establishes the cursor");
    }

    /// A belief naming a servo the bus does not have is refused where the state
    /// is validated, by the schema itself: the field is the joint vocabulary's
    /// set, so a stray bit is not a value it can carry, and a host has one call
    /// to make at its boundary either way.
    #[test]
    fn a_belief_in_servos_the_bus_does_not_have_is_not_a_state_at_all() {
        let mut fixture = Fixture::new();
        fixture
            .held
            .set_believed_torqued(JointFlagsWire::from(crate::every_row()));
        assert!(
            fixture.held.validate().is_ok(),
            "the bus's own rows are a belief a driver can hold",
        );

        fixture.held.set_believed_torqued(JointFlagsWire(
            u16::from(crate::every_row()) | 1 << JOINT_COUNT,
        ));
        assert!(
            fixture.held.validate().is_err(),
            "a tenth servo is not a servo, and the record is not a driver's",
        );
    }

    #[test]
    fn a_slot_crossing_carries_the_lap_and_the_cadence() {
        let mut fixture = Fixture::new();
        assert_eq!(
            fixture.slot().take(T0, HEALTH_PERIOD, None),
            AuxTask::Health { row: 0 }
        );
        // What a state slot holds, byte for byte, is what the next execution
        // restores: the record is the whole state and there is no other.
        let mut crossed = Fixture {
            held: fixture.held.clone(),
        };
        assert_eq!(
            crossed.slot().take(T0 + HEALTH_PERIOD, HEALTH_PERIOD, None),
            AuxTask::Health { row: 1 }
        );
        assert_eq!(fixture.slot().state.next_row, 1);
    }
}
