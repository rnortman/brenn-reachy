//! What the driver believes about torque, and how it confirms torque is off.
//!
//! Two machines that answer the two halves of one question. [`BelievedTorqued`]
//! is what the driver believes right now, derived from the register writes it
//! actually saw land — the belief the dead-man is measured against, since a
//! gate only de-torques a machine it thinks is holding. [`TorqueOffConfirm`] is
//! the evidence that a commanded de-torquing happened: while the gate's latch
//! stands, one servo per cycle is read back, and a whole pass of zeroes is the
//! confirmation.
//!
//! Both are pure: no I/O, no clock of their own, no register vocabulary. A
//! belief changes when the host says a verified write landed; a confirmation
//! names a row and the host builds whatever transaction reads that row's
//! torque-enable register. Which register that is, and how it is addressed, is
//! the bus layer's table and stays there — this crate is shared between a
//! simulated driver and the real one precisely so the *policy* is one
//! implementation, and the policy does not depend on an address.
//!
//! Confirmation does not clear the latch. The latch is a state and only a fresh
//! arming ends it ([`crate::GoalGate::release_latch`]); a confirmation is a
//! report about the machine, and a report is not a release. Nothing here gates
//! de-torquing either: the confirmation runs *after* the sweep is already being
//! written, it can only ever say whether the sweep took, and a pass that never
//! comes clean keeps reading rather than concluding anything.

use crate::state::DriverStateError;
use crate::{JOINT_COUNT, JOINT_MASK_ALL};

/// Every bus row, as torque bits. One definition with the goal masks, because a
/// torque belief covering a different row set is a machine believed idle while
/// holding.
pub const TORQUE_BITS_ALL: u16 = JOINT_MASK_ALL;

/// Which servos the driver believes are holding torque.
///
/// One bit per bus row, in the same order and with the same convention as a
/// goal's mask. The bits move only on evidence the driver has itself:
///
/// - a **verified nonzero** write to a row's torque-enable register sets it,
/// - a **verified zero** write clears it,
/// - a confirmed torque-off sweep clears all of them.
///
/// "Verified" is load-bearing and is the host's judgement to make: a write
/// whose read-back was not compared says nothing about the machine, and a
/// belief built out of unverified writes is a dead-man measuring the wrong
/// thing in both directions — a machine believed torqued that is not gets
/// de-torqued for nothing, and a machine believed idle that is holding is a
/// machine the dead-man will not save.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BelievedTorqued {
    /// The bits, one per bus row.
    pub bits: u16,
}

impl BelievedTorqued {
    /// Nothing believed torqued: what a driver starts up believing.
    #[must_use]
    pub const fn new() -> Self {
        Self { bits: 0 }
    }

    /// Every row believed torqued.
    #[must_use]
    pub const fn all() -> Self {
        Self {
            bits: TORQUE_BITS_ALL,
        }
    }

    /// Whether this describes a belief a driver can hold.
    ///
    /// # Errors
    ///
    /// [`DriverStateError::TorqueBitsOutOfRange`] for bits above the bus.
    pub fn validate(&self) -> Result<(), DriverStateError> {
        if self.bits & !TORQUE_BITS_ALL != 0 {
            return Err(DriverStateError::TorqueBitsOutOfRange { bits: self.bits });
        }
        Ok(())
    }

    /// The bits that name real rows: what every read here goes through, so a
    /// slot carrying a stray bit cannot make the driver believe in a tenth
    /// servo.
    #[must_use]
    pub const fn rows(&self) -> u16 {
        self.bits & TORQUE_BITS_ALL
    }

    /// Whether any row is believed torqued: what [`crate::GoalGate::tick`]'s
    /// `believed_torqued` argument is.
    #[must_use]
    pub const fn any(&self) -> bool {
        self.rows() != 0
    }

    /// Whether one row is believed torqued. A row past the bus is not.
    #[must_use]
    pub fn row(&self, row: u8) -> bool {
        usize::from(row) < JOINT_COUNT && self.rows() & (1 << row) != 0
    }

    /// Record a verified write to one row's torque-enable register.
    ///
    /// `enabled` is what the read-back confirmed the register now holds:
    /// nonzero sets the belief, zero clears it. A row past the bus records
    /// nothing — the driver has no belief to hold about a servo it does not
    /// address — and says so, because a host computing a row wrong writes a
    /// belief that never lands and the only other trace of it is a machine that
    /// does not de-torque.
    pub fn verified_write(&mut self, row: u8, enabled: bool) -> BeliefWrite {
        if usize::from(row) >= JOINT_COUNT {
            return BeliefWrite::RowNotOnBus;
        }
        let bit = 1 << row;
        if enabled {
            self.bits = self.rows() | bit;
        } else {
            self.bits = self.rows() & !bit;
        }
        BeliefWrite::Recorded
    }

    /// Record a confirmed torque-off sweep: every row is off.
    ///
    /// Called on the confirmation, not on the command. A sweep that has been
    /// commanded and not yet confirmed leaves the belief exactly where it was,
    /// which is what keeps the dead-man running over a machine that may still
    /// be holding.
    pub fn confirmed_off(&mut self) {
        self.bits = 0;
    }
}

/// What a verified write did to the belief.
///
/// The host counts a [`Self::RowNotOnBus`] the way it counts any other refusal:
/// nothing about the machine changed, and the caller asked about a servo the
/// driver does not have.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum BeliefWrite {
    /// The row's bit now says what the read-back confirmed.
    Recorded,
    /// Nothing recorded: the row is past the bus.
    RowNotOnBus,
}

/// What a read-back did to a confirmation pass.
///
/// Distinguishes the two reasons a reading changes nothing — a late reply for a
/// row the pass is no longer waiting on, and a row the bus does not have — so a
/// host can count the second, which is a host bug rather than lossy timing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ConfirmCredit {
    /// The row read back clean and the pass advanced.
    Credited,
    /// The row read back still torqued; the pass is back at row 0.
    Restarted,
    /// Not the row the pass is waiting on, or not a row at all.
    Ignored,
}

/// What a confirmation pass has to say this cycle.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum ConfirmReport {
    /// Nothing to report: the pass is still walking, or it has already said
    /// everything it has to say.
    #[default]
    Nothing,
    /// Every row read back zero. Said once per pass, on the cycle the pass
    /// completes.
    Confirmed,
    /// The budget ran out with the pass not clean. Said once per pass; the pass
    /// keeps reading afterwards, and still reports [`Self::Confirmed`] if it
    /// eventually comes clean.
    Unconfirmed,
}

/// What the confirmation wants read this cycle, and what it has to say.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ConfirmStep {
    /// The bus row whose torque-enable register the host should read back, if
    /// the confirmation wants one. `None` once the pass is confirmed, and while
    /// the machine is not running.
    pub read_row: Option<u8>,
    /// What to report.
    pub report: ConfirmReport,
}

/// The torque-off confirmation machine: one read-back per cycle, a whole clean
/// pass is the confirmation.
///
/// The pass is a consecutive run of clean read-backs from row 0 up. A row that
/// reads back still torqued sends the pass back to the start rather than
/// leaving a hole: what is being confirmed is that the machine is off *now*,
/// and a row read clean before another row was found holding is a reading from
/// before whatever is still writing torque was accounted for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TorqueOffConfirm {
    /// Whether a pass is running.
    pub active: bool,
    /// When the running pass opened. Meaningful only while `active`.
    pub started_ns: i64,
    /// How many rows from row 0 have read back clean in a row. Equal to
    /// [`JOINT_COUNT`] once the pass is complete.
    pub cursor: u8,
    /// Whether [`ConfirmReport::Confirmed`] has been said for this pass.
    pub said_confirmed: bool,
    /// Whether [`ConfirmReport::Unconfirmed`] has been said for this pass.
    pub said_unconfirmed: bool,
}

impl TorqueOffConfirm {
    /// A machine that is not running.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            active: false,
            started_ns: 0,
            cursor: 0,
            said_confirmed: false,
            said_unconfirmed: false,
        }
    }

    /// Whether this describes a state a confirmation can be in.
    ///
    /// # Errors
    ///
    /// [`DriverStateError::ConfirmCursorOutOfRange`] for a cursor past a
    /// complete pass, [`DriverStateError::IdleConfirmWithProgress`] for a
    /// machine that is not running while carrying a pass's work, or
    /// [`DriverStateError::ConfirmedWithIncompletePass`] for one claiming a
    /// confirmation its cursor did not earn.
    pub fn validate(&self) -> Result<(), DriverStateError> {
        if usize::from(self.cursor) > JOINT_COUNT {
            return Err(DriverStateError::ConfirmCursorOutOfRange { row: self.cursor });
        }
        if !self.active && (self.cursor != 0 || self.said_confirmed || self.said_unconfirmed) {
            return Err(DriverStateError::IdleConfirmWithProgress {
                cursor: self.cursor,
            });
        }
        if self.said_confirmed && usize::from(self.cursor) < JOINT_COUNT {
            return Err(DriverStateError::ConfirmedWithIncompletePass {
                cursor: self.cursor,
            });
        }
        Ok(())
    }

    /// Open a pass at `now_ns`, if one is not already running.
    ///
    /// Called every cycle the gate's latch stands, so it has to be idempotent:
    /// a pass already running keeps its own opening instant, because the budget
    /// is measured from when the de-torquing was commanded and a second call
    /// naming a later instant would extend it. This is the one-clock rule the
    /// wind-down maneuver holds to, in the small.
    pub fn begin(&mut self, now_ns: i64) {
        if self.active {
            return;
        }
        *self = Self {
            active: true,
            started_ns: now_ns,
            cursor: 0,
            said_confirmed: false,
            said_unconfirmed: false,
        };
    }

    /// Stop confirming, and forget the pass.
    ///
    /// What a fresh arming does: the latch is gone, the machine is being
    /// torqued on deliberately, and read-backs saying so are not evidence of
    /// anything failing. A stood-down machine reports nothing and asks for
    /// nothing until it is begun again.
    pub fn stand_down(&mut self) {
        *self = Self::new();
    }

    /// Record a read-back of one row's torque-enable register.
    ///
    /// Only the row the pass is waiting for advances it: an answer for any
    /// other row is a late reply from before the pass restarted, and counting
    /// it would credit the pass with a reading it did not ask for. A row found
    /// still torqued restarts the pass.
    ///
    /// The reading it did with the answer comes back, so a host handing it rows
    /// nothing asked for can count them.
    pub fn observed(&mut self, row: u8, torqued: bool) -> ConfirmCredit {
        if self.waiting_on() != Some(row) {
            return ConfirmCredit::Ignored;
        }
        if torqued {
            self.cursor = 0;
            ConfirmCredit::Restarted
        } else {
            // `row` is the clamped cursor, so this cannot climb past a complete
            // pass however the cursor arrived.
            self.cursor = row + 1;
            ConfirmCredit::Credited
        }
    }

    /// How far the pass has got: what `cursor` says, or nothing read yet if it
    /// says something past a complete pass.
    ///
    /// Every read of the cursor goes through this. A cursor beyond a complete
    /// pass is a slot nothing here wrote, and the one reading of it that must
    /// never happen is "the pass is over": that credits a de-torquing nobody
    /// read back, which clears the belief and disarms the dead-man's sweep over
    /// a machine that may still be holding torque. So it reads as a pass with
    /// nothing yet credited, which costs one more lap of reads and cannot
    /// mislead.
    fn pass_cursor(&self) -> u8 {
        if usize::from(self.cursor) > JOINT_COUNT {
            0
        } else {
            self.cursor
        }
    }

    /// The row this pass is waiting on, if any.
    #[must_use]
    pub fn waiting_on(&self) -> Option<u8> {
        let cursor = self.pass_cursor();
        if self.active && usize::from(cursor) < JOINT_COUNT {
            Some(cursor)
        } else {
            None
        }
    }

    /// One cycle of the pass: what to read, and what to report.
    ///
    /// `budget_ns` is how long a commanded de-torquing may go unconfirmed
    /// before the driver says so. Running out of it changes nothing about what
    /// the driver does — the sweep is still being written every cycle and the
    /// pass still reads — it only makes the silence visible, because a
    /// de-torquing that cannot be confirmed is the operator's problem and
    /// nothing here is allowed to gate on it.
    pub fn step(&mut self, now_ns: i64, budget_ns: i64) -> ConfirmStep {
        if !self.active {
            return ConfirmStep::default();
        }
        let Some(row) = self.waiting_on() else {
            let report = if self.said_confirmed {
                ConfirmReport::Nothing
            } else {
                self.said_confirmed = true;
                ConfirmReport::Confirmed
            };
            return ConfirmStep {
                read_row: None,
                report,
            };
        };
        let overdue = now_ns.saturating_sub(self.started_ns) > budget_ns;
        let report = if overdue && !self.said_unconfirmed {
            self.said_unconfirmed = true;
            ConfirmReport::Unconfirmed
        } else {
            ConfirmReport::Nothing
        };
        ConfirmStep {
            read_row: Some(row),
            report,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const T0: i64 = 1_700_000_000_000_000_000;
    const PERIOD: i64 = 20_000_000;
    const BUDGET: i64 = 500_000_000;

    #[test]
    fn a_fresh_belief_holds_nothing() {
        let believed = BelievedTorqued::new();
        assert!(!believed.any());
        assert_eq!(believed.rows(), 0);
        for row in 0..JOINT_COUNT as u8 {
            assert!(!believed.row(row));
        }
        believed.validate().expect("a fresh belief is a belief");
    }

    #[test]
    fn every_row_is_its_own_bit() {
        for row in 0..JOINT_COUNT as u8 {
            let mut believed = BelievedTorqued::new();
            believed.verified_write(row, true);
            assert!(believed.any());
            for other in 0..JOINT_COUNT as u8 {
                assert_eq!(believed.row(other), other == row, "row {other} after {row}");
            }
        }
    }

    #[test]
    fn a_verified_zero_write_clears_one_row_and_leaves_the_rest() {
        let mut believed = BelievedTorqued::all();
        assert_eq!(believed.rows(), TORQUE_BITS_ALL);
        believed.verified_write(3, false);
        assert!(!believed.row(3));
        assert!(believed.any());
        for row in (0..JOINT_COUNT as u8).filter(|r| *r != 3) {
            assert!(believed.row(row));
        }
    }

    #[test]
    fn clearing_every_row_one_at_a_time_ends_the_belief() {
        let mut believed = BelievedTorqued::all();
        for row in 0..JOINT_COUNT as u8 {
            assert!(believed.any(), "still torqued before clearing row {row}");
            believed.verified_write(row, false);
        }
        assert!(!believed.any());
    }

    #[test]
    fn a_confirmed_sweep_clears_the_whole_belief_at_once() {
        let mut believed = BelievedTorqued::all();
        believed.confirmed_off();
        assert!(!believed.any());
    }

    #[test]
    fn a_write_to_a_row_the_bus_does_not_have_is_ignored_and_says_so() {
        let mut believed = BelievedTorqued::new();
        for row in [JOINT_COUNT as u8, 200] {
            assert_eq!(
                believed.verified_write(row, true),
                BeliefWrite::RowNotOnBus,
                "row {row} is not a row, and a host that wrote it hears that"
            );
        }
        assert_eq!(believed.verified_write(0, true), BeliefWrite::Recorded);
        believed.verified_write(0, false);
        assert!(!believed.any());
        assert!(!believed.row(JOINT_COUNT as u8));
    }

    #[test]
    fn a_belief_with_bits_above_the_bus_is_refused_and_reads_as_the_bus() {
        let stray = BelievedTorqued {
            bits: TORQUE_BITS_ALL | 1 << JOINT_COUNT,
        };
        assert_eq!(
            stray.validate(),
            Err(DriverStateError::TorqueBitsOutOfRange { bits: stray.bits })
        );
        assert_eq!(stray.rows(), TORQUE_BITS_ALL);
        assert!(!stray.row(JOINT_COUNT as u8));
    }

    /// Walk a running pass to completion, answering every read clean.
    ///
    /// `begin` on every cycle, because that is the call pattern the driver has:
    /// the latch stands, so the cycle opens by beginning the pass it is already
    /// running.
    fn confirm_clean(confirm: &mut TorqueOffConfirm, mut now: i64) -> (i64, Vec<ConfirmReport>) {
        let mut reports = Vec::new();
        for _ in 0..2 * JOINT_COUNT + 2 {
            confirm.begin(now);
            let step = confirm.step(now, BUDGET);
            reports.push(step.report);
            if let Some(row) = step.read_row {
                confirm.observed(row, false);
            }
            now += PERIOD;
            if confirm.said_confirmed {
                break;
            }
        }
        (now, reports)
    }

    #[test]
    fn an_idle_confirmation_asks_for_nothing_and_says_nothing() {
        let mut confirm = TorqueOffConfirm::new();
        assert_eq!(confirm.step(T0, BUDGET), ConfirmStep::default());
        assert_eq!(confirm.waiting_on(), None);
        confirm.validate().expect("a fresh confirmation is a state");
    }

    #[test]
    fn a_clean_pass_reads_every_row_in_order_and_confirms_once() {
        let mut confirm = TorqueOffConfirm::new();
        confirm.begin(T0);
        let mut now = T0;
        let mut read = Vec::new();
        loop {
            let step = confirm.step(now, BUDGET);
            match step.read_row {
                Some(row) => {
                    read.push(row);
                    assert_eq!(step.report, ConfirmReport::Nothing);
                    confirm.observed(row, false);
                }
                None => {
                    assert_eq!(step.report, ConfirmReport::Confirmed);
                    break;
                }
            }
            now += PERIOD;
        }
        assert_eq!(read, (0..JOINT_COUNT as u8).collect::<Vec<_>>());
        // Said once: every later cycle has nothing to add.
        for _ in 0..4 {
            now += PERIOD;
            assert_eq!(
                confirm.step(now, BUDGET),
                ConfirmStep {
                    read_row: None,
                    report: ConfirmReport::Nothing,
                }
            );
        }
    }

    #[test]
    fn a_row_still_torqued_sends_the_pass_back_to_the_start() {
        let mut confirm = TorqueOffConfirm::new();
        confirm.begin(T0);
        let mut now = T0;
        for row in 0..4u8 {
            assert_eq!(confirm.step(now, BUDGET).read_row, Some(row));
            confirm.observed(row, false);
            now += PERIOD;
        }
        assert_eq!(confirm.step(now, BUDGET).read_row, Some(4));
        assert_eq!(confirm.observed(4, true), ConfirmCredit::Restarted);
        now += PERIOD;
        assert_eq!(confirm.step(now, BUDGET).read_row, Some(0));
        assert!(!confirm.said_confirmed);
    }

    #[test]
    fn an_answer_for_a_row_the_pass_is_not_waiting_on_does_not_advance_it() {
        let mut confirm = TorqueOffConfirm::new();
        confirm.begin(T0);
        assert_eq!(confirm.observed(5, false), ConfirmCredit::Ignored);
        assert_eq!(confirm.waiting_on(), Some(0));
        assert_eq!(confirm.observed(0, false), ConfirmCredit::Credited);
        assert_eq!(confirm.waiting_on(), Some(1));
        // A late duplicate of the row already credited is not a second credit.
        assert_eq!(confirm.observed(0, false), ConfirmCredit::Ignored);
        assert_eq!(confirm.waiting_on(), Some(1));
    }

    #[test]
    fn a_read_back_of_a_row_the_bus_does_not_have_is_ignored() {
        let mut confirm = TorqueOffConfirm::new();
        confirm.begin(T0);
        confirm.cursor = JOINT_COUNT as u8;
        assert_eq!(
            confirm.observed(JOINT_COUNT as u8, false),
            ConfirmCredit::Ignored
        );
        assert_eq!(confirm.cursor, JOINT_COUNT as u8);
    }

    #[test]
    fn the_budget_runs_from_the_first_begin_and_a_second_does_not_extend_it() {
        let mut confirm = TorqueOffConfirm::new();
        confirm.begin(T0);
        for step in 1..6 {
            confirm.begin(T0 + step * PERIOD);
            assert_eq!(confirm.started_ns, T0);
        }
    }

    /// The whole pass is what `begin` leaves alone, not just its opening
    /// instant: a `begin` that reset the cursor every cycle would be a pass that
    /// never completes, so a de-torquing that took would read as unconfirmed for
    /// the life of the process.
    #[test]
    fn a_begin_part_way_through_a_pass_disturbs_none_of_it() {
        let mut confirm = TorqueOffConfirm::new();
        confirm.begin(T0);
        let mut now = T0;
        for _ in 0..4 {
            let step = confirm.step(now, BUDGET);
            confirm.observed(step.read_row.expect("still reading"), false);
            now += PERIOD;
        }
        assert_eq!(confirm.cursor, 4, "four rows read back clean");
        // Past the budget, so the pass has said its piece as well.
        let overdue = T0 + BUDGET + PERIOD;
        assert_eq!(
            confirm.step(overdue, BUDGET).report,
            ConfirmReport::Unconfirmed
        );
        confirm.begin(overdue + PERIOD);
        assert_eq!(confirm.cursor, 4, "a begin restarted the pass");
        assert_eq!(confirm.started_ns, T0);
        assert!(confirm.said_unconfirmed, "a begin unsaid the report");
        assert!(!confirm.said_confirmed);
        assert_eq!(confirm.waiting_on(), Some(4));
    }

    #[test]
    fn an_overdue_pass_says_so_once_and_keeps_reading() {
        let mut confirm = TorqueOffConfirm::new();
        // Begun every cycle, as the driver begins it while the latch stands.
        let mut now = T0;
        while now <= T0 + BUDGET {
            confirm.begin(now);
            let step = confirm.step(now, BUDGET);
            assert_eq!(step.report, ConfirmReport::Nothing);
            // Answer every read as still torqued: the pass never completes.
            confirm.observed(step.read_row.expect("still reading"), true);
            now += PERIOD;
        }
        confirm.begin(now);
        let step = confirm.step(now, BUDGET);
        assert_eq!(step.report, ConfirmReport::Unconfirmed);
        assert_eq!(step.read_row, Some(0));
        for _ in 0..4 {
            now += PERIOD;
            confirm.begin(now);
            let step = confirm.step(now, BUDGET);
            assert_eq!(step.report, ConfirmReport::Nothing);
            assert!(step.read_row.is_some(), "an overdue pass keeps reading");
            confirm.observed(step.read_row.expect("still reading"), true);
        }
    }

    #[test]
    fn a_pass_that_comes_clean_after_saying_unconfirmed_still_confirms() {
        let mut confirm = TorqueOffConfirm::new();
        confirm.begin(T0);
        let mut now = T0 + BUDGET + PERIOD;
        assert_eq!(confirm.step(now, BUDGET).report, ConfirmReport::Unconfirmed);
        now += PERIOD;
        let (_, reports) = confirm_clean(&mut confirm, now);
        assert!(reports.contains(&ConfirmReport::Confirmed));
        assert!(confirm.said_unconfirmed);
    }

    #[test]
    fn standing_down_forgets_the_pass_and_a_later_begin_starts_over() {
        let mut confirm = TorqueOffConfirm::new();
        confirm.begin(T0);
        confirm.observed(0, false);
        confirm.stand_down();
        assert_eq!(confirm, TorqueOffConfirm::new());
        confirm.begin(T0 + 10 * PERIOD);
        assert_eq!(confirm.started_ns, T0 + 10 * PERIOD);
        assert_eq!(confirm.waiting_on(), Some(0));
    }

    #[test]
    fn a_cursor_past_a_complete_pass_is_refused() {
        let confirm = TorqueOffConfirm {
            active: true,
            started_ns: T0,
            cursor: JOINT_COUNT as u8 + 1,
            said_confirmed: false,
            said_unconfirmed: false,
        };
        assert_eq!(
            confirm.validate(),
            Err(DriverStateError::ConfirmCursorOutOfRange {
                row: JOINT_COUNT as u8 + 1
            })
        );
    }

    /// The reading that must never happen. A cursor past a complete pass is a
    /// slot nothing here wrote, and reading it as "every row came back clean"
    /// would confirm a de-torquing nobody read back: the host clears its belief,
    /// the dead-man stops watching, and the machine may still be holding torque.
    /// It reads as a pass with nothing credited instead.
    #[test]
    fn a_cursor_past_the_bus_does_not_read_as_a_completed_pass() {
        let mut confirm = TorqueOffConfirm::new();
        confirm.begin(T0);
        confirm.cursor = JOINT_COUNT as u8 + 3;
        assert_eq!(confirm.waiting_on(), Some(0), "the pass reads from row 0");
        let step = confirm.step(T0 + PERIOD, BUDGET);
        assert_eq!(step.read_row, Some(0));
        assert_eq!(step.report, ConfirmReport::Nothing, "nothing was read back");
        assert!(!confirm.said_confirmed);
        // And it walks a whole clean lap from there, as any pass does.
        assert_eq!(confirm.observed(0, false), ConfirmCredit::Credited);
        assert_eq!(
            confirm.cursor, 1,
            "a credit climbed from the clamped cursor"
        );
        let (_, reports) = confirm_clean(&mut confirm, T0 + 2 * PERIOD);
        assert!(reports.contains(&ConfirmReport::Confirmed));
    }

    #[test]
    fn a_complete_pass_is_not_a_cursor_out_of_range() {
        let confirm = TorqueOffConfirm {
            active: true,
            started_ns: T0,
            cursor: JOINT_COUNT as u8,
            said_confirmed: true,
            said_unconfirmed: false,
        };
        confirm.validate().expect("a complete pass is a state");
    }

    /// Progress no run produces, and a slot can hold: a report said over a pass
    /// that has not read every row. Refused, because tolerated it is a machine
    /// whose `step` takes the already-said branch forever — the confirmation is
    /// never emitted, the host never clears its belief, and a de-torquing that
    /// took reads as unconfirmed for the rest of the process.
    #[test]
    fn a_confirmation_said_over_an_incomplete_pass_is_refused() {
        let confirm = TorqueOffConfirm {
            active: true,
            started_ns: T0,
            cursor: 3,
            said_confirmed: true,
            said_unconfirmed: false,
        };
        assert_eq!(
            confirm.validate(),
            Err(DriverStateError::ConfirmedWithIncompletePass { cursor: 3 })
        );
        // Unconfirmed is a different matter: an overdue pass says so with rows
        // still to read, which is the state it is meant to be said in.
        let overdue = TorqueOffConfirm {
            said_confirmed: false,
            said_unconfirmed: true,
            ..confirm
        };
        overdue.validate().expect("an overdue pass is a state");
    }

    #[test]
    fn a_stood_down_machine_carrying_a_pass_is_refused() {
        let mid = TorqueOffConfirm {
            active: false,
            started_ns: T0,
            cursor: 3,
            said_confirmed: false,
            said_unconfirmed: false,
        };
        assert_eq!(
            mid.validate(),
            Err(DriverStateError::IdleConfirmWithProgress { cursor: 3 })
        );
        let said = TorqueOffConfirm {
            active: false,
            started_ns: T0,
            cursor: 0,
            said_confirmed: true,
            said_unconfirmed: false,
        };
        assert_eq!(
            said.validate(),
            Err(DriverStateError::IdleConfirmWithProgress { cursor: 0 })
        );
    }

    #[test]
    fn a_confirmation_is_what_clears_a_belief_and_the_command_is_not() {
        let mut believed = BelievedTorqued::all();
        let mut confirm = TorqueOffConfirm::new();
        confirm.begin(T0);
        // Commanded, not yet confirmed: the belief stands, so the dead-man
        // still has a machine to watch.
        assert!(believed.any());
        let (_, reports) = confirm_clean(&mut confirm, T0);
        assert!(reports.contains(&ConfirmReport::Confirmed));
        believed.confirmed_off();
        assert!(!believed.any());
    }

    #[test]
    fn every_state_error_prints_the_number_it_is_about() {
        let cases = [
            (
                DriverStateError::TorqueBitsOutOfRange { bits: 0x0400 },
                "0x0400",
            ),
            (DriverStateError::HealthCursorOutOfRange { row: 12 }, "12"),
            (DriverStateError::ConfirmCursorOutOfRange { row: 12 }, "12"),
            (DriverStateError::IdleConfirmWithProgress { cursor: 3 }, "3"),
        ];
        for (error, number) in cases {
            let text = error.to_string();
            assert!(
                text.contains(number),
                "{error:?} does not say {number}: {text}"
            );
        }
    }
}
