//! Taking hold of the machine: the one engagement a host asks for, and when a
//! cycle may act on it.
//!
//! The host that commands this machine engages it in one cycle — the goal
//! registers pinned at what that cycle's own grouped read returned, torque
//! enabled over the same rows, and the enable read back — so the decision here
//! is small and is entirely about *when*: a cycle may engage a row only if this
//! cycle's read answered it, because the pin is the reading the cycle took and
//! never an older one. A request that keeps meeting reads which miss a row it
//! names waits [`crate::ENGAGE_READ_CYCLES`] cycles and is then dropped with
//! nothing written.
//!
//! It is a decision and not two lines in a host for the reason every decision
//! in this crate is one: a simulated driver and a real one disagreeing about
//! when an engagement may be written, or about how long one stands, would be
//! two machines, and the answer one of them gave the session would not mean
//! what the other's did.
//!
//! Pure, like the rest: no register addresses, no wire, no clock. What a host
//! brings is this cycle's answered rows and what it believes about torque; what
//! it gets back is whether to write, what to answer, or nothing.
//!
//! Nothing here gates de-torquing, and nothing here can hold torque on: an
//! engagement is written once and answered once, and the belief a confirmed
//! read-back earns is what the dead-man then runs over.

use brenn_reachy__driver__aux_clk_rs::{AuxSlotState, EngageRequestState, TorqueConfirmState};
use brenn_reachy__driver__gate_clk_rs::GateState;
use brenn_reachy__driver__health_clk_rs::EventKind;
use brenn_reachy__motion__joints_clk_rs::JointFlags;

use crate::ENGAGE_READ_CYCLES;
use crate::torque::{ROW_FLAGS, TorqueOffConfirm};
use crate::{AuxSlot, GoalGate};

/// What a cycle should do about the engagement a host asked for.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum EngageStep {
    /// Nothing: no engagement is asked for, or this cycle's read did not answer
    /// every row one names and the request still stands.
    #[default]
    Nothing,
    /// Every row is already believed torqued. Write nothing and answer
    /// confirmed again: a re-issue whose first answer was lost is the host
    /// repeating itself, and repeating the answer is what heals it.
    Held {
        /// The rows the request named, all of them believed holding.
        rows: JointFlags,
    },
    /// Write the engagement over these rows in this cycle: pin, enable, read
    /// the enable back.
    Run {
        /// The rows to take hold of.
        rows: JointFlags,
    },
    /// The request is dropped with nothing written: no cycle inside its window
    /// had a read that answered all of it.
    Declined {
        /// The rows the reads kept missing.
        rows: JointFlags,
    },
}

impl EngageStep {
    /// The event a step that writes nothing owes the host, and the rows it
    /// names.
    ///
    /// Two of the four endings are answered without touching the bus, and what
    /// they are answered *with* is as much a decision as when they arise: the
    /// session parks on an unconfirmed engagement and rests on a declined one,
    /// so a host that mapped these itself could put the machine somewhere the
    /// other host would not. [`Nothing`](EngageStep::Nothing) owes no answer,
    /// and [`Run`](EngageStep::Run) is answered by [`verdict`] once the write
    /// has happened.
    #[must_use]
    pub fn answer(self) -> Option<(EventKind, JointFlags)> {
        match self {
            EngageStep::Nothing | EngageStep::Run { .. } => None,
            EngageStep::Held { rows } => Some((EventKind::EngageConfirmed, rows)),
            EngageStep::Declined { rows } => Some((EventKind::EngageDeclined, rows)),
        }
    }
}

/// The event a written engagement earns, and the rows it names.
///
/// `requested` is what the engagement took hold of and `unconfirmed` is what
/// did not read its enable back — including any row the host could not write at
/// all. Confirmed names the whole engagement, because that is what took;
/// unconfirmed names only the rows that failed, because that is what a person
/// reading the row needs and what the session's own answer is scoped to.
///
/// Here rather than in each host for the reason [`EngageStep::answer`] is: the
/// session's phase turns on which of the two arrives.
#[must_use]
pub fn verdict(requested: JointFlags, unconfirmed: JointFlags) -> (EventKind, JointFlags) {
    if unconfirmed == JointFlags::NONE {
        (EventKind::EngageConfirmed, requested)
    } else {
        (EventKind::EngageUnconfirmed, unconfirmed)
    }
}

/// Move the belief a confirmed engagement earns, and end a standing latch.
///
/// The same credit a verified torque-enable write earns through the aux slot,
/// over a set of rows at once: the belief is what the dead-man is measured
/// against, so a row read back enabled has to be in it. An engagement that
/// confirmed anything also ends a torque-off latch and stands the confirmation
/// pass down — the machine has been energised again on purpose, and a pass still
/// reading back the release it replaced would report on a de-torquing nobody is
/// asking for any more. A row that confirmed nothing leaves all three alone, and
/// the cycle's own liveness is noted instead.
///
/// Both hosts, from one place, because these three writes are the whole of what
/// an engagement changes about the machine the dead-man runs over: a host that
/// believed a row without releasing the latch, or released it without standing
/// the pass down, would be a driver holding a torque the other one would not.
pub fn credit(
    aux: &mut AuxSlotState,
    gate: &mut GateState,
    confirm: &mut TorqueConfirmState,
    confirmed: JointFlags,
    nominal_ns: i64,
) {
    if confirmed == JointFlags::NONE {
        return;
    }
    {
        let mut slot = AuxSlot::over(aux);
        let mut belief = slot.belief();
        for (index, flag) in ROW_FLAGS.into_iter().enumerate() {
            if confirmed.contains(flag) {
                belief.verified_write(u8::try_from(index).unwrap_or(u8::MAX), true);
            }
        }
    }
    let mut gate = GoalGate::over(gate);
    if gate.state().latched.get() {
        gate.release_latch(nominal_ns);
        TorqueOffConfirm::over(confirm).stand_down();
    } else {
        gate.note_liveness(nominal_ns);
    }
}

/// Let go of the machine, because an engagement did not confirm.
///
/// The latch is set, whatever was queued goes with it, the aux slot is
/// abandoned and the confirmation pass opens — the same three writes a
/// commanded de-torquing makes, on the driver's own authority. An engagement
/// that wrote a torque-enable and could not read it back leaves servos that
/// have very likely taken torque behind a belief that says none, and the
/// dead-man only de-torques a machine it believes is holding; a torque state
/// nobody can vouch for is answered by taking torque off, not by believing it
/// is already off.
///
/// No event: the `engage_unconfirmed` the cycle raises is the cause and the
/// confirmation pass's own report is the outcome, which is how every other
/// latch reads. Idempotent over a latch already standing, so a cycle whose
/// exits overlap and a re-issue meeting a standing latch both cost nothing.
///
/// Called after [`credit`], where anything confirmed: the latch stands over
/// whatever the credit released, and the sweep that follows covers every row
/// regardless of belief.
///
/// Beside `credit` and in this crate for the same reason — this is the fourth
/// thing an engagement changes about the machine the dead-man runs over, and a
/// host that credited what confirmed without letting go of what did not would
/// be a driver holding a torque the other one would not.
pub fn fail(
    aux: &mut AuxSlotState,
    gate: &mut GateState,
    confirm: &mut TorqueConfirmState,
    nominal_ns: i64,
) {
    GoalGate::over(gate).latch_torque_off();
    AuxSlot::over(aux).abandon();
    TorqueOffConfirm::over(confirm).begin(nominal_ns);
}

/// The engagement a host asked for, decided over the state it lives in.
///
/// Borrows the state for as long as a decision is being made and holds nothing
/// of its own, so a host keeps it wherever it keeps state — a cog's slot or a
/// process's memory.
pub struct EngageRequest<'a> {
    /// The state being decided over.
    state: &'a mut EngageRequestState,
}

impl<'a> EngageRequest<'a> {
    /// Decide over the state a previous cycle left.
    pub fn over(state: &'a mut EngageRequestState) -> Self {
        Self { state }
    }

    /// The state being decided over.
    #[must_use]
    pub fn state(&self) -> &EngageRequestState {
        self.state
    }

    /// Whether an engagement is asked for.
    #[must_use]
    pub fn pending(&self) -> bool {
        self.state.pending.get()
    }

    /// Take up a host's ask, and answer whether it opened a fresh window.
    ///
    /// Idempotent over the same rows: a re-issue is the host repeating itself
    /// on a lossy link, so the window it is already waiting in keeps running
    /// rather than starting again — a host re-issuing every wake could
    /// otherwise hold a request open for ever. A *different* set of rows is a
    /// different engagement and starts its own window.
    ///
    /// An empty set is not an engagement and opens nothing: there is no row to
    /// take hold of, and a request that named none would wait out its window
    /// only to decline an empty set.
    pub fn offer(&mut self, rows: JointFlags) -> bool {
        if rows == JointFlags::NONE {
            return false;
        }
        if self.pending() && self.state.rows == rows {
            return false;
        }
        self.state.rows = rows;
        self.state.cycles = 0;
        self.state.pending = true.into();
        true
    }

    /// Drop whatever engagement was asked for, unanswered.
    ///
    /// What a commanded de-torquing does to it. An engagement is a request to
    /// take hold of the machine and a de-torquing outranks it, so it is
    /// abandoned rather than written after the sweep — and unanswered, because
    /// the host that asked for the release is the host that asked for the
    /// engagement and has its answer in the release.
    pub fn abandon(&mut self) {
        self.state.pending = false.into();
        self.state.rows = JointFlags::NONE;
        self.state.cycles = 0;
    }

    /// What this cycle should do, given the rows its grouped read answered and
    /// the rows it believes are already holding.
    ///
    /// The belief is asked first, so a re-issue of an engagement that already
    /// took is answered even on a cycle whose read missed a row: what the host
    /// is owed there is the answer it did not hear, and nothing is written for
    /// it.
    pub fn step(&mut self, answered: JointFlags, believed: JointFlags) -> EngageStep {
        if !self.pending() {
            return EngageStep::Nothing;
        }
        let rows = self.state.rows;
        if believed.contains(rows) {
            self.abandon();
            return EngageStep::Held { rows };
        }
        if answered.contains(rows) {
            self.abandon();
            return EngageStep::Run { rows };
        }
        self.state.cycles = self.state.cycles.saturating_add(1);
        if self.state.cycles < ENGAGE_READ_CYCLES {
            return EngageStep::Nothing;
        }
        self.abandon();
        EngageStep::Declined {
            rows: without(rows, answered),
        }
    }
}

/// The rows in `set` and not in `other`.
///
/// Over the rows rather than over the word, for the reason the vocabulary has
/// no complement operator: the bits above the ninth belong to no servo, and a
/// complement would hand them out.
///
/// A second copy of `reachy_motion::joints::flags::without`, deliberately: this
/// crate links the vocabulary modules and nothing else, which is what lets the
/// simulated driver and the real one host the same decisions without either
/// pulling in the motion library. The two are the same convention over the same
/// nine bits, so a change to one is a change to both.
fn without(set: JointFlags, other: JointFlags) -> JointFlags {
    ROW_FLAGS
        .into_iter()
        .filter(|row| set.contains(*row) && !other.contains(*row))
        .fold(JointFlags::NONE, |kept, row| kept | row)
}

#[cfg(test)]
mod tests {
    use super::{EngageRequest, EngageStep, credit, fail, verdict, without};
    use crate::ENGAGE_READ_CYCLES;
    use crate::{AuxSlot, GoalGate, TorqueOffConfirm};
    use brenn_reachy__driver__aux_clk_rs::{
        AuxSlotStateWire, EngageRequestStateWire, TorqueConfirmStateWire,
    };
    use brenn_reachy__driver__gate_clk_rs::GateStateWire;
    use brenn_reachy__driver__health_clk_rs::EventKind;
    use brenn_reachy__motion__joints_clk_rs::JointFlags;

    const T0: i64 = 1_700_000_000_000_000_000;

    /// The three states an engagement's outcome writes, so a case can hold one
    /// value and still hand out all three borrows at once.
    struct Machine {
        gate: GateStateWire,
        aux: AuxSlotStateWire,
        confirm: TorqueConfirmStateWire,
    }

    impl Machine {
        /// A driver that has been told nothing: unlatched, an empty slot, no
        /// pass.
        fn new() -> Self {
            let mut gate = GateStateWire::new();
            GoalGate::start(&mut gate);
            let mut aux = AuxSlotStateWire::new();
            AuxSlot::start(&mut aux);
            let mut confirm = TorqueConfirmStateWire::new();
            TorqueOffConfirm::start(&mut confirm);
            Self { gate, aux, confirm }
        }

        /// Let go of the machine, as a cycle whose engagement did not confirm
        /// does.
        fn fail_at(&mut self, now_ns: i64) {
            fail(
                self.aux.validate_mut().expect("a slot reads as one"),
                self.gate.validate_mut().expect("a gate reads as one"),
                self.confirm.validate_mut().expect("a pass reads as one"),
                now_ns,
            );
        }

        /// Credit `rows`, as a cycle whose engagement confirmed them does.
        fn credit_at(&mut self, rows: JointFlags, now_ns: i64) {
            credit(
                self.aux.validate_mut().expect("a slot reads as one"),
                self.gate.validate_mut().expect("a gate reads as one"),
                self.confirm.validate_mut().expect("a pass reads as one"),
                rows,
                now_ns,
            );
        }

        fn latched(&self) -> bool {
            self.gate
                .validate()
                .expect("a gate reads as one")
                .latched
                .get()
        }

        fn pass_open(&self) -> bool {
            self.confirm
                .validate()
                .expect("a pass reads as one")
                .active
                .get()
        }

        /// When the pass that is open was opened.
        fn pass_started(&self) -> i64 {
            self.confirm
                .validate()
                .expect("a pass reads as one")
                .started
                .as_nanos()
        }

        /// Whether a host request is waiting in the slot.
        fn slot_pending(&self) -> bool {
            self.aux
                .validate()
                .expect("a slot reads as one")
                .has_pending
                .get()
        }
    }

    /// The head and the antennas, which is what a gate that degraded nothing
    /// hands over.
    fn every_row() -> JointFlags {
        crate::every_row()
    }

    /// The antennas, which is what a degraded gate leaves out.
    fn antennas() -> JointFlags {
        JointFlags::ANTENNA_RIGHT | JointFlags::ANTENNA_LEFT
    }

    fn cleared() -> EngageRequestStateWire {
        let mut wire = EngageRequestStateWire::new();
        wire.clear_valid();
        wire
    }

    fn over(wire: &mut EngageRequestStateWire) -> EngageRequest<'_> {
        EngageRequest::over(wire.validate_mut().expect("a cleared state reads as one"))
    }

    #[test]
    fn a_state_nothing_wrote_asks_for_no_engagement() {
        let mut wire = cleared();
        let mut request = over(&mut wire);

        assert!(!request.pending());
        assert_eq!(
            request.step(every_row(), JointFlags::NONE),
            EngageStep::Nothing,
            "a driver that has just started has been asked for nothing"
        );
    }

    #[test]
    fn a_read_that_answered_every_named_row_runs_the_engagement() {
        let mut wire = cleared();
        let mut request = over(&mut wire);
        assert!(request.offer(every_row()), "a fresh window");

        assert_eq!(
            request.step(every_row(), JointFlags::NONE),
            EngageStep::Run { rows: every_row() }
        );
        assert!(!request.pending(), "answered once and not standing");
    }

    #[test]
    fn a_read_missing_a_named_row_writes_nothing_and_the_request_stands() {
        let mut wire = cleared();
        let mut request = over(&mut wire);
        request.offer(every_row());

        let answered = without(every_row(), JointFlags::LEG_3);
        for _ in 1..ENGAGE_READ_CYCLES {
            assert_eq!(
                request.step(answered, JointFlags::NONE),
                EngageStep::Nothing,
                "the pin is this cycle's reading, so a cycle that cannot see a row does not write"
            );
            assert!(request.pending());
        }

        assert_eq!(
            request.step(answered, JointFlags::NONE),
            EngageStep::Declined {
                rows: JointFlags::LEG_3
            },
            "dropped naming the row the reads kept missing"
        );
        assert!(!request.pending());
    }

    #[test]
    fn a_row_that_answers_inside_the_window_still_engages() {
        let mut wire = cleared();
        let mut request = over(&mut wire);
        request.offer(every_row());
        let answered = without(every_row(), JointFlags::LEG_3);

        assert_eq!(
            request.step(answered, JointFlags::NONE),
            EngageStep::Nothing
        );
        assert_eq!(
            request.step(every_row(), JointFlags::NONE),
            EngageStep::Run { rows: every_row() },
            "the first cycle whose read answered every named row"
        );
    }

    #[test]
    fn a_re_issue_keeps_the_window_it_is_already_waiting_in() {
        let mut wire = cleared();
        let mut request = over(&mut wire);
        request.offer(every_row());
        let answered = without(every_row(), JointFlags::LEG_3);

        for _ in 1..ENGAGE_READ_CYCLES {
            assert!(
                !request.offer(every_row()),
                "the same rows again is the host repeating itself"
            );
            assert_eq!(
                request.step(answered, JointFlags::NONE),
                EngageStep::Nothing
            );
        }
        assert_eq!(
            request.step(answered, JointFlags::NONE),
            EngageStep::Declined {
                rows: JointFlags::LEG_3
            },
            "a host re-issuing every cycle cannot hold the window open"
        );
    }

    #[test]
    fn a_different_set_of_rows_is_a_different_engagement() {
        let mut wire = cleared();
        let mut request = over(&mut wire);
        request.offer(every_row());
        let answered = without(every_row(), JointFlags::LEG_3);
        request.step(answered, JointFlags::NONE);

        assert!(
            request.offer(without(every_row(), antennas())),
            "a degraded gate's rows are their own engagement"
        );
        assert_eq!(request.state().cycles, 0, "with its own window");
    }

    #[test]
    fn rows_already_believed_torqued_are_confirmed_without_a_write() {
        let mut wire = cleared();
        let mut request = over(&mut wire);
        request.offer(every_row());

        assert_eq!(
            request.step(JointFlags::NONE, every_row()),
            EngageStep::Held { rows: every_row() },
            "a lost answer heals on the re-issue, whatever this cycle's read saw"
        );
        assert!(!request.pending());
    }

    #[test]
    fn a_belief_covering_only_some_rows_is_written_like_any_other() {
        let mut wire = cleared();
        let mut request = over(&mut wire);
        request.offer(every_row());

        assert_eq!(
            request.step(every_row(), antennas()),
            EngageStep::Run { rows: every_row() },
            "a partly held machine is engaged, not confirmed"
        );
    }

    #[test]
    fn an_empty_set_is_not_an_engagement() {
        let mut wire = cleared();
        let mut request = over(&mut wire);

        assert!(!request.offer(JointFlags::NONE));
        assert!(!request.pending(), "there is no row to take hold of");
    }

    #[test]
    fn a_de_torquing_drops_the_engagement_unanswered() {
        let mut wire = cleared();
        let mut request = over(&mut wire);
        request.offer(every_row());

        request.abandon();

        assert!(!request.pending());
        assert_eq!(
            request.step(every_row(), JointFlags::NONE),
            EngageStep::Nothing,
            "nothing is written after the release that outranked it"
        );
    }

    /// The difference is over the servos, so no bit above the ninth is ever
    /// handed out and a row named on neither side stays out of both.
    #[test]
    fn the_difference_names_servos_and_nothing_else() {
        let kept = without(every_row(), JointFlags::LEG_3 | antennas());

        assert!(!kept.contains(JointFlags::LEG_3));
        assert!(!kept.contains(antennas()));
        assert!(kept.contains(JointFlags::BODY_YAW));
        assert_eq!(
            kept | JointFlags::LEG_3 | antennas(),
            every_row(),
            "what was taken out and what was kept are the whole bus"
        );
    }
    /// The two endings that write nothing carry their own answer, and the two
    /// that are not endings carry none: a host asking a step what to say gets
    /// the same sentence in both processes.
    #[test]
    fn a_step_that_writes_nothing_answers_for_itself() {
        assert_eq!(
            EngageStep::Held { rows: every_row() }.answer(),
            Some((EventKind::EngageConfirmed, every_row()))
        );
        assert_eq!(
            EngageStep::Declined { rows: antennas() }.answer(),
            Some((EventKind::EngageDeclined, antennas()))
        );
        assert_eq!(EngageStep::Nothing.answer(), None);
        assert_eq!(
            EngageStep::Run { rows: every_row() }.answer(),
            None,
            "a written engagement is answered by its verdict, not by the step"
        );
    }

    /// An engagement that did not confirm leaves the machine let go of: the
    /// latch stands, the slot is abandoned and the pass is reading the release
    /// back.
    ///
    /// Idempotent over a standing latch, and it re-latches a gate a credit in
    /// the same cycle just released -- a cycle where some rows confirmed and
    /// others did not credits first and lets go after, and what has to survive
    /// that order is the latch.
    #[test]
    fn a_failed_engagement_latches_and_opens_the_pass() {
        let mut machine = Machine::new();

        machine.fail_at(T0);

        assert!(machine.latched(), "the machine is being let go of");
        assert!(machine.pass_open(), "with the release read back");
        assert_eq!(machine.pass_started(), T0);
        assert!(
            !machine.slot_pending(),
            "and nothing out of band outstanding",
        );

        machine.fail_at(T0 + 20_000_000);

        assert!(machine.latched(), "a second call changes nothing");
        assert_eq!(
            machine.pass_started(),
            T0,
            "and the budget is still measured from when the de-torquing began",
        );
    }

    /// A credit that released the latch does not survive the failure that
    /// follows it in the same cycle.
    #[test]
    fn a_failure_after_a_credit_re_latches_the_gate() {
        let mut machine = Machine::new();
        machine.fail_at(T0);
        machine.credit_at(every_row(), T0 + 20_000_000);
        assert!(!machine.latched(), "the credit is the way out of a latch");

        machine.fail_at(T0 + 20_000_000);

        assert!(
            machine.latched(),
            "and the rows that did not confirm put it back",
        );
        assert!(machine.pass_open());
    }

    /// Confirmed names the whole engagement and unconfirmed names only what
    /// failed: the session parks on one and goes active on the other, so the two
    /// hosts cannot each decide it.
    #[test]
    fn the_verdict_names_the_engagement_or_the_rows_that_failed() {
        assert_eq!(
            verdict(every_row(), JointFlags::NONE),
            (EventKind::EngageConfirmed, every_row())
        );
        assert_eq!(
            verdict(every_row(), JointFlags::LEG_3),
            (EventKind::EngageUnconfirmed, JointFlags::LEG_3),
            "the rows that did confirm are not named: what a reader needs is what failed"
        );
    }
}
