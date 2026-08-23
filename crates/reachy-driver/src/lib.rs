//! `reachy-driver` — a motor driver's decisions, with no motors attached.
//!
//! A motor driver's cycle is mostly bus traffic, but the part of it that
//! decides *whether and what to write* is pure: a small FIFO of goals stamped
//! with the grid instant they are to be executed at, a held setpoint that gets
//! rewritten between due goals, and a timer that de-torques the machine when
//! the process feeding it goals goes quiet. That is [`GoalGate`], and beside it
//! sit the three smaller decisions a cycle also makes — what the driver
//! believes about torque ([`BelievedTorqued`]), whether a commanded
//! de-torquing has been seen to take ([`TorqueOffConfirm`]), and which one
//! out-of-band transaction the cycle spends its spare bus time on
//! ([`AuxSlot`]). Those decisions are this crate, and nothing else is: no
//! register addresses, no wire, no clock of its own.
//!
//! It exists separately because the same decision has to run in three places
//! and be the same decision in all of them: the real driver process, the
//! simulated driver that the control loop is tested against, and the unit
//! tests that pin the semantics down. A second implementation of a dead-man
//! timer is a second set of conditions under which a machine does not
//! de-torque.
//!
//! What it links is the joint vocabulary and the transaction vocabulary: the
//! set of servos this machine has, and the record of one bus transaction. A
//! gate whose idea of the bus disagreed with its host's would silently stop
//! writing the rows above the disagreement, so the row count is read off the
//! vocabulary rather than restated here.
//!
//! Every cross-cycle state here is a schema — `driver/gate.clk` for the gate,
//! `driver/aux.clk` for the aux slot, the torque belief and the confirmation
//! pass — and this crate decides over them in place: a host validates the slot
//! once at its boundary and hands the view down, so the simulated driver's
//! state slots and the driver process's own memory hold one description of a
//! queue, a belief and a pass rather than two. There is no second form and
//! nothing to restore: a queue's length is the array's own, so no number can
//! claim more goals than there are, and a record that is not a transaction is
//! refused by the host's one validation rather than met halfway down here. What
//! the schemas cannot say — which cursor values name a bus row, which
//! combinations of a cursor and a report a run produces — the machines refuse
//! themselves, as typed errors over the state they were handed.
//!
//! What the gate guarantees to whoever hosts it:
//!
//! - **Goals are never reordered.** A goal that arrives late, or stamped for
//!   an instant already past, is accepted and executed in arrival order, with
//!   the anomaly reported to the caller. Reordering a position command is how
//!   a machine ends up going somewhere nobody asked for.
//! - **One goal executes per cycle.** A backlog drains at one goal per tick,
//!   so a burst cannot turn into a jump.
//! - **Silence de-torques.** If the goal stream stops for longer than the hold
//!   timeout while the machine is believed torqued, the gate latches torque
//!   off by itself. Nothing gates that, and no caller has to remember to ask.
//! - **A latch is only cleared by a fresh arming.** [`GoalGate::release_latch`]
//!   is the one way out, and it starts from an empty queue, nothing held, and a
//!   fresh hold-timeout window: the new session commands the machine, and
//!   nothing the old one asked for is written on its way in.
//!
//! Times taken and answered here are nanoseconds since the Unix epoch, because
//! a cycle's own instant is a number its host already has; a setpoint carries
//! its instant as `SyncTime`, which is the vocabulary's own. Angles are radians
//! and a set of servos is `JointFlags` — the joint vocabulary, never a mask of
//! bits a comment explains.

#![forbid(unsafe_code)]

use brenn_reachy__driver__gate_clk_rs::{GateState, GateStateWire};
use brenn_reachy__driver__goal_clk_rs::{GoalSetpoint, GoalSetpointWire};
use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointRef};
use clockwork_rs::SyncTime;

pub mod aux_slot;
pub mod state;
pub mod torque;

pub use aux_slot::{AuxOffer, AuxSlot, AuxTask};
pub use state::DriverStateError;
pub use torque::{
    BeliefWrite, BelievedTorqued, ConfirmCredit, ConfirmReport, ConfirmStep, TorqueOffConfirm,
};

/// Servo rows on the bus, and so the length of every position array here.
///
/// Counted off the joint vocabulary rather than stated: `JointRef` names every
/// servo plus the `none` a report about the whole machine carries, so a tenth
/// servo declared there is a tenth row here without an edit.
pub const JOINT_COUNT: usize = JointRef::VARIANTS.len() - 1;

/// Every joint in a mask: the low [`JOINT_COUNT`] bits.
///
/// Bit `n` is bus row `n`, which is what the vocabulary's `JointFlags`
/// declares; `every_joint_is_a_bit_of_the_mask` holds the two together.
pub const JOINT_MASK_ALL: u16 = (1 << JOINT_COUNT) - 1;

/// Every servo on the bus, as a set.
///
/// The driver layer's one spelling of "all of them", so a belief, a goal mask
/// and a test fixture cannot each build the set a different way and disagree
/// when a row is added. Folded over the vocabulary's own declared values rather
/// than converted from [`JOINT_MASK_ALL`]: nothing here can fail, which the
/// process that de-torques the machine cannot afford, and a tenth servo declared
/// there is in this set without an edit. A function and not a constant because
/// the vocabulary's union operator is not a `const fn`.
#[must_use]
pub fn every_row() -> JointFlags {
    JointFlags::VARIANTS
        .into_iter()
        .fold(JointFlags::NONE, |set, row| set | row)
}

/// What became of a goal offered to the gate.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AcceptOutcome {
    /// Queued, in order, for its stated instant.
    Accepted,
    /// Queued — but its instant is already past, or not after the goal ahead
    /// of it. It will execute in arrival order like any other. The caller
    /// reports this; it is a sender defect, not a machine fault.
    AcceptedStaleOrOutOfOrder,
    /// Refused: the queue was full. The sender is overrunning the gate.
    DroppedQueueFull,
}

/// What the host should do to the bus this cycle.
///
/// The two writing answers name no setpoint: what is to be written is the
/// gate's own `held`, which the host reads off the state it already has. A
/// setpoint handed back here would be a copy of it, and a copy is a second
/// description of what the machine was asked for.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GateAction {
    /// De-torque every row. The gate is latched and will say this every cycle
    /// until [`GoalGate::release_latch`].
    WriteTorqueOffSweep {
        /// Whether the dead-man latched on this very cycle.
        ///
        /// True exactly once per latch, on the cycle the hold timeout ran out,
        /// which is the edge a host reports as an event. Every cycle a
        /// standing latch answers afterwards is false: the condition is
        /// visible in the sample stream, and an event per cycle for as long as
        /// a machine stays de-torqued is a report that grows at poll rate
        /// instead of marking what happened.
        ///
        /// A latch the host asked for with [`GoalGate::latch_torque_off`] is
        /// never announced here — the host already knows, and the edge worth
        /// reporting is the one nobody asked for.
        just_latched: bool,
    },
    /// Write the held setpoint: a newly due goal has just become it.
    WriteGoal,
    /// Write the held setpoint again: no new goal is due, and it stands.
    Rewrite,
    /// Write nothing. Nothing has ever been commanded.
    Nothing,
}

/// The gate: a goal queue, the setpoint it is holding, and the dead-man,
/// decided over the state they live in.
///
/// Borrows the state for as long as a decision is being made and holds nothing
/// of its own, so a host keeps its state wherever it keeps state — a cog's slot
/// or a process's memory — and every read a host wants (what is held, whether
/// the gate is latched, how deep the queue is) is a field of that state rather
/// than something to ask for here.
pub struct GoalGate<'a> {
    /// The state being decided over.
    state: &'a mut GateState,
}

impl<'a> GoalGate<'a> {
    /// Take up a slot as a gate that has been told nothing.
    ///
    /// The producer's route: the slot is cleared, which is an empty queue,
    /// nothing held, no latch and a dead-man that is not running.
    pub fn start(slot: &'a mut GateStateWire) -> Self {
        Self {
            state: slot.clear_valid(),
        }
    }

    /// Decide over the gate state a previous cycle left.
    pub fn over(state: &'a mut GateState) -> Self {
        Self { state }
    }

    /// The state being decided over.
    #[must_use]
    pub fn state(&self) -> &GateState {
        self.state
    }

    /// The instant the last goal in line is stamped for: the one a fresh goal
    /// has to come after to be in order. The tail of the queue, or the held
    /// setpoint when the queue is empty, or nothing at all when neither has
    /// been commanded — a first goal is measured against nothing.
    fn last_instant(&self) -> Option<i64> {
        let queued = self.state.queue.len();
        if queued > 0 {
            return self
                .state
                .queue
                .get(queued - 1)
                .map(|goal| goal.execute_at.as_nanos());
        }
        self.state
            .has_held
            .get()
            .then(|| self.state.held.execute_at.as_nanos())
    }

    /// Offer a goal to the gate.
    ///
    /// An accepted goal is liveness: it is the evidence that whatever is
    /// supposed to be commanding this machine is still there, which is what
    /// the dead-man measures. A dropped one is not — a sender overrunning the
    /// queue must not be able to hold torque on by overrunning it faster.
    pub fn accept(&mut self, goal: &GoalSetpoint, now_ns: i64) -> AcceptOutcome {
        let due_at = goal.execute_at.as_nanos();
        let out_of_order = self.last_instant().is_some_and(|last| due_at <= last);
        let stale = due_at < now_ns;
        let Some(slot) = self.state.queue.try_grow() else {
            return AcceptOutcome::DroppedQueueFull;
        };
        copy_whole(slot, goal);
        self.note_liveness(now_ns);
        if stale || out_of_order {
            AcceptOutcome::AcceptedStaleOrOutOfOrder
        } else {
            AcceptOutcome::Accepted
        }
    }

    /// Record that the sender is alive as of `now_ns`.
    ///
    /// Called by [`GoalGate::accept`] for every accepted goal, and by the host
    /// at the moment torque goes on — arming grants a fresh hold-timeout
    /// window, because the first goal of a session cannot arrive before the
    /// session starts. A driver with an auxiliary request path counts those
    /// datagrams as liveness the same way.
    pub fn note_liveness(&mut self, now_ns: i64) {
        self.state.last_accept = SyncTime::from_nanos(now_ns);
        self.state.has_accepted = true.into();
    }

    /// Latch torque off and drop everything queued.
    ///
    /// The queue goes with the latch deliberately: goals stamped for instants
    /// during a de-torqued stretch describe a machine that no longer exists,
    /// and the only correct thing to do with them is forget them.
    ///
    /// The held setpoint stays, because a latch writes nothing and the sample
    /// stream still has to say what was last commanded. It is dropped at
    /// [`GoalGate::release_latch`], which is where it would otherwise become a
    /// write.
    pub fn latch_torque_off(&mut self) {
        self.state.latched = true.into();
        self.state.queue.clear();
    }

    /// End a torque-off latch at `now_ns`, as part of a fresh arming.
    ///
    /// Everything the dead session commanded goes here: the queue, so a goal
    /// that arrived while the gate was latched cannot execute after it, and the
    /// held setpoint, so the first torqued cycle of the new session does not
    /// rewrite a position the old one asked for. What happens next is whatever
    /// the newly armed session commands, and until it commands something the
    /// gate writes nothing — recovery is a fresh engagement, and a machine that
    /// moved at torque-on because of a setpoint predating its own de-torquing
    /// is uncommanded motion however plausible the number was.
    ///
    /// The hold-timeout window is granted here too, in the same call. A latch
    /// that ended with the old liveness instant still standing would re-latch
    /// on the very next cycle — the window it is measured against is already
    /// past by definition, since running out of it is what latched the gate —
    /// and it would do so by emitting the same event a genuinely stalled
    /// commander does. A machine that will not stay armed, reported as somebody
    /// else's fault, is not something to leave to a caller remembering a second
    /// call.
    pub fn release_latch(&mut self, now_ns: i64) {
        self.state.latched = false.into();
        self.clear_commanded();
        self.note_liveness(now_ns);
    }

    /// Forget what was commanded, without latching anything.
    ///
    /// What a confirmed disarm leaves behind: the machine has been de-torqued
    /// deliberately, by a sequence that ran to completion, so there is no fault
    /// to latch and nothing left to hold. The held setpoint goes with the queue
    /// because it describes a machine that is no longer holding anything — a
    /// driver that kept reporting it as `commanded` would be reporting a
    /// setpoint nothing is tracking.
    ///
    /// The dead-man is untouched: whether the goal stream may go quiet is a
    /// question about torque, and this call says nothing about torque. A host
    /// that de-torqued the machine stops believing it is torqued in the same
    /// breath, which is what actually holds the dead-man off.
    pub fn clear_commanded(&mut self) {
        self.state.queue.clear();
        *clockwork_rs::into_raw(&mut self.state.held) = GoalSetpointWire::new();
        self.state.has_held = false.into();
    }

    /// Decide what to write this cycle.
    ///
    /// `believed_torqued` is the host's belief about whether the machine is
    /// holding torque right now, and `hold_timeout_ns` is how long the goal
    /// stream may be silent before the gate de-torques it.
    ///
    /// The dead-man is checked before anything about held setpoints, so it
    /// also covers the window between arming and the first goal — the stretch
    /// where a machine is torqued, holding, and nobody has commanded it yet,
    /// which is exactly when a commander that failed to start is least
    /// visible.
    pub fn tick(
        &mut self,
        nominal_ns: i64,
        believed_torqued: bool,
        hold_timeout_ns: i64,
    ) -> GateAction {
        if self.state.latched.get() {
            return GateAction::WriteTorqueOffSweep {
                just_latched: false,
            };
        }
        if believed_torqued
            && self.state.has_accepted.get()
            && nominal_ns.saturating_sub(self.state.last_accept.as_nanos()) > hold_timeout_ns
        {
            self.latch_torque_off();
            return GateAction::WriteTorqueOffSweep { just_latched: true };
        }
        let due = self
            .state
            .queue
            .get(0)
            .is_some_and(|head| head.execute_at.as_nanos() <= nominal_ns);
        if due {
            self.take_head();
            return GateAction::WriteGoal;
        }
        if self.state.has_held.get() {
            return GateAction::Rewrite;
        }
        GateAction::Nothing
    }

    /// Make the head of the queue the held setpoint and close the gap behind
    /// it. Only called with a head to take.
    ///
    /// The queue is a line rather than a ring: the goals behind the head move
    /// up, so "oldest first" is the array's own order and there is no cursor to
    /// disagree with it.
    fn take_head(&mut self) {
        let Some(head) = self.state.queue.get(0).map(clockwork_rs::as_raw).cloned() else {
            return;
        };
        *clockwork_rs::into_raw(&mut self.state.held) = head;
        self.state.has_held = true.into();
        for behind in 1..self.state.queue.len() {
            let Some(goal) = self
                .state
                .queue
                .get(behind)
                .map(clockwork_rs::as_raw)
                .cloned()
            else {
                break;
            };
            if let Some(ahead) = self.state.queue.get_mut(behind - 1) {
                *clockwork_rs::into_raw(ahead) = goal;
            }
        }
        self.state.queue.pop();
    }
}

/// Copy a validated record into the place `dst` names, whole.
///
/// One statement of what a record is worth copying: all of it. Copying field by
/// field is where a queue entry ends up carrying one goal's instant and
/// another's angles, or a slot one request's register and another's value, and
/// the bytes are a validated message either way.
pub(crate) fn copy_whole<V>(dst: &mut V, src: &V)
where
    V: clockwork_rs::ValidView,
    V::Raw: Clone,
{
    *clockwork_rs::into_raw(dst) = clockwork_rs::as_raw(src).clone();
}

#[cfg(test)]
mod tests {
    use super::*;
    use brenn_reachy__motion__joints_clk_rs::JointFlagsWire;
    use clockwork_rs::blob_as_bytes;

    const PERIOD: i64 = 20_000_000;
    const HOLD_TIMEOUT: i64 = 200_000_000;
    const T0: i64 = 1_700_000_000_000_000_000;

    /// What the cycle a dead-man latch fires on answers: the edge a host turns
    /// into one event.
    const LATCHING_SWEEP: GateAction = GateAction::WriteTorqueOffSweep { just_latched: true };

    /// What every cycle a standing latch answers afterwards says, and what a
    /// latch the host asked for says from its first cycle: keep de-torquing,
    /// nothing new happened.
    const STANDING_SWEEP: GateAction = GateAction::WriteTorqueOffSweep {
        just_latched: false,
    };

    /// A gate's state and the slot it lives in, so a case can hold one value
    /// and still hand out the borrow every decision takes.
    struct Fixture {
        /// The slot the gate decides over.
        slot: GateStateWire,
    }

    impl Fixture {
        /// A gate that has been told nothing.
        fn new() -> Self {
            let mut slot = GateStateWire::new();
            GoalGate::start(&mut slot);
            Self { slot }
        }

        /// The gate, for one decision.
        fn gate(&mut self) -> GoalGate<'_> {
            GoalGate::over(
                self.slot
                    .validate_mut()
                    .expect("a gate leaves a gate in its slot"),
            )
        }

        /// What the last decision left.
        fn state(&self) -> &GateState {
            self.slot
                .validate()
                .expect("a gate leaves a gate in its slot")
        }

        /// How many goals are waiting.
        fn queued(&self) -> usize {
            self.state().queue.len()
        }
    }

    /// A gate that has just been armed: torque on, liveness seeded, nothing
    /// commanded yet.
    fn armed_at(now_ns: i64) -> Fixture {
        let mut fx = Fixture::new();
        fx.gate().note_liveness(now_ns);
        fx
    }

    /// A setpoint for every servo at one angle, stamped for `execute_at_ns`.
    fn goal_at(execute_at_ns: i64, value: f64) -> GoalSetpointWire {
        let mut wire = GoalSetpointWire::new();
        let goal = wire.clear_valid();
        goal.execute_at = SyncTime::from_nanos(execute_at_ns);
        goal.mask = crate::every_row();
        for row in [
            &mut goal.targets.body_yaw,
            &mut goal.targets.leg_0,
            &mut goal.targets.leg_1,
            &mut goal.targets.leg_2,
            &mut goal.targets.leg_3,
            &mut goal.targets.leg_4,
            &mut goal.targets.leg_5,
            &mut goal.targets.antenna_right,
            &mut goal.targets.antenna_left,
        ] {
            *row = value;
        }
        wire
    }

    /// A made setpoint, read the way a host hands one to the gate.
    fn view(goal: &GoalSetpointWire) -> &GoalSetpoint {
        goal.validate().expect("a made setpoint is a setpoint")
    }

    /// Assert the gate is holding exactly that setpoint, byte for byte.
    fn assert_held(fx: &Fixture, goal: &GoalSetpointWire, what: &str) {
        assert!(fx.state().has_held.get(), "{what}: nothing is held");
        assert_eq!(
            blob_as_bytes(clockwork_rs::as_raw(&fx.state().held)),
            blob_as_bytes(goal),
            "{what}"
        );
    }

    /// Assert the gate is holding nothing at all.
    fn assert_holds_nothing(fx: &Fixture, what: &str) {
        assert!(!fx.state().has_held.get(), "{what}");
        assert_eq!(
            blob_as_bytes(clockwork_rs::as_raw(&fx.state().held)),
            blob_as_bytes(&GoalSetpointWire::new()),
            "{what}: a setpoint was left behind"
        );
    }

    #[test]
    fn a_fresh_gate_writes_nothing() {
        let mut fx = Fixture::new();
        assert_eq!(fx.gate().tick(T0, false, HOLD_TIMEOUT), GateAction::Nothing);
        assert_eq!(
            fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT),
            GateAction::Nothing
        );
        assert!(!fx.state().has_held.get());
    }

    #[test]
    fn a_goal_executes_at_its_instant_and_not_before() {
        let mut fx = armed_at(T0);
        let goal = goal_at(T0 + 2 * PERIOD, 0.5);
        assert_eq!(fx.gate().accept(view(&goal), T0), AcceptOutcome::Accepted);
        assert_eq!(fx.gate().tick(T0, true, HOLD_TIMEOUT), GateAction::Nothing);
        assert_eq!(
            fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT),
            GateAction::Nothing
        );
        assert_eq!(
            fx.gate().tick(T0 + 2 * PERIOD, true, HOLD_TIMEOUT),
            GateAction::WriteGoal
        );
    }

    #[test]
    fn a_due_goal_becomes_the_held_setpoint_and_is_rewritten_after() {
        let mut fx = armed_at(T0);
        let goal = goal_at(T0 + PERIOD, 0.25);
        fx.gate().accept(view(&goal), T0);
        assert_eq!(
            fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT),
            GateAction::WriteGoal
        );
        for step in 2..6 {
            assert_eq!(
                fx.gate().tick(T0 + step * PERIOD, true, HOLD_TIMEOUT),
                GateAction::Rewrite,
                "cycle {step}"
            );
        }
        assert_held(&fx, &goal, "the setpoint stands between goals");
    }

    #[test]
    fn a_backlog_drains_one_goal_per_tick_in_order() {
        let mut fx = armed_at(T0);
        let goals: Vec<GoalSetpointWire> = (1..=3)
            .map(|i| goal_at(T0 + i * PERIOD, f64::from(i as i32)))
            .collect();
        for goal in &goals {
            assert_eq!(fx.gate().accept(view(goal), T0), AcceptOutcome::Accepted);
        }
        // All three are due at once: the gate is a cycle behind.
        let late = T0 + 10 * PERIOD;
        for (nth, goal) in goals.iter().enumerate() {
            assert_eq!(
                fx.gate().tick(late, true, HOLD_TIMEOUT),
                GateAction::WriteGoal,
                "tick {nth}"
            );
            assert_held(&fx, goal, &format!("the {nth}th goal out of the backlog"));
            // What is still queued is what was behind it, in the order it
            // arrived: the take shifts the entries down, which is where a
            // duplicated head or a transposed pair would show.
            assert_eq!(fx.queued(), goals.len() - nth - 1, "tick {nth}");
            for (behind, queued) in goals[nth + 1..].iter().enumerate() {
                assert_eq!(
                    fx.state()
                        .queue
                        .get(behind)
                        .map(clockwork_rs::as_raw)
                        .map(blob_as_bytes),
                    Some(blob_as_bytes(queued)),
                    "tick {nth}, queue entry {behind}"
                );
            }
        }
        assert_eq!(
            fx.gate().tick(late, true, HOLD_TIMEOUT),
            GateAction::Rewrite
        );
        assert_held(&fx, &goals[2], "and the last one stands");
        assert_eq!(fx.queued(), 0);
    }

    #[test]
    fn a_full_queue_drops_and_says_so_without_disturbing_what_is_queued() {
        let mut fx = armed_at(T0);
        let depth = fx.state().queue.capacity();
        let mut queued = Vec::new();
        for i in 1..=depth {
            let goal = goal_at(T0 + i as i64 * PERIOD, i as f64);
            assert_eq!(fx.gate().accept(view(&goal), T0), AcceptOutcome::Accepted);
            queued.push(goal);
        }
        let overflow = goal_at(T0 + 99 * PERIOD, 99.0);
        assert_eq!(
            fx.gate().accept(view(&overflow), T0),
            AcceptOutcome::DroppedQueueFull
        );
        assert_eq!(fx.queued(), depth);
        for (nth, goal) in queued.iter().enumerate() {
            assert_eq!(
                fx.state()
                    .queue
                    .get(nth)
                    .map(clockwork_rs::as_raw)
                    .map(blob_as_bytes),
                Some(blob_as_bytes(goal)),
                "queue entry {nth} was disturbed by the refusal"
            );
        }
    }

    #[test]
    fn a_dropped_goal_is_not_liveness() {
        let mut fx = armed_at(T0);
        for i in 1..=fx.state().queue.capacity() {
            fx.gate()
                .accept(view(&goal_at(T0 + i as i64 * PERIOD, 0.0)), T0);
        }
        assert_eq!(fx.state().last_accept.as_nanos(), T0);
        fx.gate()
            .accept(view(&goal_at(T0 + 99 * PERIOD, 0.0)), T0 + 5 * PERIOD);
        assert_eq!(
            fx.state().last_accept.as_nanos(),
            T0,
            "a refused goal must not feed the dead-man"
        );
    }

    #[test]
    fn a_stale_goal_is_accepted_warned_and_executed_in_arrival_order() {
        let mut fx = armed_at(T0);
        let stale = goal_at(T0 - PERIOD, 1.0);
        assert_eq!(
            fx.gate().accept(view(&stale), T0),
            AcceptOutcome::AcceptedStaleOrOutOfOrder
        );
        assert_eq!(
            fx.gate().tick(T0, true, HOLD_TIMEOUT),
            GateAction::WriteGoal
        );
    }

    #[test]
    fn an_out_of_order_goal_is_accepted_warned_and_never_reordered() {
        let mut fx = armed_at(T0);
        let later = goal_at(T0 + 4 * PERIOD, 1.0);
        let earlier = goal_at(T0 + 2 * PERIOD, 2.0);
        assert_eq!(fx.gate().accept(view(&later), T0), AcceptOutcome::Accepted);
        assert_eq!(
            fx.gate().accept(view(&earlier), T0),
            AcceptOutcome::AcceptedStaleOrOutOfOrder
        );
        let late = T0 + 8 * PERIOD;
        assert_eq!(
            fx.gate().tick(late, true, HOLD_TIMEOUT),
            GateAction::WriteGoal
        );
        assert_held(&fx, &later, "the one that arrived first executes first");
        assert_eq!(
            fx.gate().tick(late, true, HOLD_TIMEOUT),
            GateAction::WriteGoal
        );
        assert_held(
            &fx,
            &earlier,
            "and the earlier instant executes after it, unreordered",
        );
    }

    #[test]
    fn a_goal_no_later_than_the_held_setpoint_is_out_of_order_too() {
        let mut fx = armed_at(T0);
        let first = goal_at(T0 + PERIOD, 1.0);
        fx.gate().accept(view(&first), T0);
        fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        assert_eq!(
            fx.gate()
                .accept(view(&goal_at(T0 + PERIOD, 2.0)), T0 + PERIOD),
            AcceptOutcome::AcceptedStaleOrOutOfOrder
        );
    }

    #[test]
    fn the_dead_man_fires_one_nanosecond_past_the_timeout_and_not_at_it() {
        let mut fx = armed_at(T0);
        fx.gate().accept(view(&goal_at(T0 + PERIOD, 0.0)), T0);
        fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        assert_eq!(
            fx.gate().tick(T0 + HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            GateAction::Rewrite
        );
        assert_eq!(
            fx.gate().tick(T0 + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        assert!(fx.state().latched.get());
    }

    #[test]
    fn the_dead_man_fires_with_nothing_ever_commanded() {
        // Armed, torqued, and the commander never sent a first goal. This is
        // the window a dead-man written around the held setpoint would miss.
        let mut fx = armed_at(T0);
        assert_eq!(
            fx.gate().tick(T0 + HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            GateAction::Nothing
        );
        assert_eq!(
            fx.gate().tick(T0 + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        assert!(fx.state().latched.get());
        assert!(!fx.state().has_held.get());
    }

    #[test]
    fn the_dead_man_does_not_run_before_anything_is_armed() {
        let mut fx = Fixture::new();
        assert_eq!(
            fx.gate().tick(T0 + 10 * HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            GateAction::Nothing
        );
        assert!(!fx.state().latched.get());
    }

    #[test]
    fn the_dead_man_does_not_run_on_a_machine_that_is_not_torqued() {
        let mut fx = armed_at(T0);
        fx.gate().accept(view(&goal_at(T0 + PERIOD, 0.0)), T0);
        fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        for step in 1..20 {
            let action = fx
                .gate()
                .tick(T0 + step * HOLD_TIMEOUT, false, HOLD_TIMEOUT);
            assert!(action == GateAction::Rewrite, "cycle {step}: {action:?}");
        }
        assert!(!fx.state().latched.get());
    }

    #[test]
    fn a_rewrite_is_not_liveness() {
        // Only a datagram from outside refreshes the window. If rewriting the
        // held setpoint counted, the dead-man would be measuring the driver's
        // own pulse and would never fire.
        let mut fx = armed_at(T0);
        fx.gate().accept(view(&goal_at(T0 + PERIOD, 0.0)), T0);
        let mut now = T0 + PERIOD;
        let mut rewrites = 0;
        loop {
            match fx.gate().tick(now, true, HOLD_TIMEOUT) {
                GateAction::WriteTorqueOffSweep { .. } => break,
                _ => rewrites += 1,
            }
            now += PERIOD;
            assert!(rewrites < 1000, "the dead-man never fired");
        }
        assert_eq!(now, T0 + HOLD_TIMEOUT + PERIOD);
    }

    #[test]
    fn a_live_goal_stream_holds_the_dead_man_off_indefinitely() {
        let mut fx = armed_at(T0);
        let mut now = T0;
        for step in 1..500 {
            now = T0 + step * PERIOD;
            fx.gate().accept(view(&goal_at(now + 2 * PERIOD, 0.0)), now);
            let action = fx.gate().tick(now, true, HOLD_TIMEOUT);
            assert!(
                !matches!(action, GateAction::WriteTorqueOffSweep { .. }),
                "cycle {step}"
            );
        }
        assert!(!fx.state().latched.get());
        assert_eq!(fx.state().last_accept.as_nanos(), now);
    }

    #[test]
    fn a_latch_clears_the_queue_and_stays_until_released() {
        let mut fx = armed_at(T0);
        fx.gate().accept(view(&goal_at(T0 + PERIOD, 0.0)), T0);
        fx.gate().accept(view(&goal_at(T0 + 2 * PERIOD, 0.0)), T0);
        fx.gate().latch_torque_off();
        assert_eq!(fx.queued(), 0);
        for step in 0..10 {
            assert_eq!(
                fx.gate().tick(T0 + step * PERIOD, true, HOLD_TIMEOUT),
                STANDING_SWEEP
            );
        }
    }

    #[test]
    fn a_goal_that_arrives_during_a_latch_does_not_execute_after_it() {
        let mut fx = armed_at(T0);
        fx.gate().latch_torque_off();
        fx.gate().accept(view(&goal_at(T0 + PERIOD, 1.0)), T0);
        fx.gate().release_latch(T0);
        assert_eq!(fx.queued(), 0);
        assert_eq!(
            fx.gate().tick(T0 + PERIOD, false, HOLD_TIMEOUT),
            GateAction::Nothing
        );
    }

    #[test]
    fn releasing_the_latch_is_all_it_takes_to_stay_armed() {
        // The whole of a re-arming, with no second call to remember: if the
        // release left the old liveness instant standing, this cycle would
        // re-latch and report it as a stalled commander.
        let mut fx = armed_at(T0);
        let latched_at = T0 + HOLD_TIMEOUT + 1;
        assert_eq!(
            fx.gate().tick(latched_at, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        let rearmed_at = latched_at + PERIOD;
        fx.gate().release_latch(rearmed_at);
        assert_eq!(
            fx.gate().tick(rearmed_at, true, HOLD_TIMEOUT),
            GateAction::Nothing,
            "a released latch must not fire the dead-man on stale evidence"
        );
        assert!(!fx.state().latched.get());
    }

    #[test]
    fn releasing_the_latch_grants_a_fresh_dead_man_window() {
        let mut fx = armed_at(T0);
        let latched_at = T0 + HOLD_TIMEOUT + 1;
        assert_eq!(
            fx.gate().tick(latched_at, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        let rearmed_at = latched_at + PERIOD;
        fx.gate().release_latch(rearmed_at);
        assert_eq!(
            fx.gate().tick(rearmed_at, true, HOLD_TIMEOUT),
            GateAction::Nothing
        );
        assert_eq!(
            fx.gate()
                .tick(rearmed_at + HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            GateAction::Nothing,
            "the window is measured from the re-arming, not from the old latch"
        );
        assert_eq!(
            fx.gate()
                .tick(rearmed_at + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
    }

    #[test]
    fn a_held_setpoint_survives_a_latch_so_the_sample_stream_stays_truthful() {
        // The latch stops writes; it does not erase what was last commanded,
        // which is what a driver reports as `commanded` in its samples.
        let mut fx = armed_at(T0);
        let goal = goal_at(T0 + PERIOD, 0.75);
        fx.gate().accept(view(&goal), T0);
        fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        fx.gate().latch_torque_off();
        assert_held(&fx, &goal, "the latch leaves the sample stream truthful");
    }

    /// A re-arming after a dead-man latch starts from nothing commanded.
    ///
    /// The hazard this pins is uncommanded motion at torque-on: a gate that
    /// kept the old session's setpoint across the latch would rewrite it on the
    /// first torqued cycle of the new one, moving the machine to a position no
    /// live commander asked for, in the window before the new commander's first
    /// goal lands.
    #[test]
    fn a_re_armed_gate_writes_nothing_until_the_new_session_commands_something() {
        let mut fx = armed_at(T0);
        let old = goal_at(T0 + PERIOD, 0.75);
        fx.gate().accept(view(&old), T0);
        fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        let latched_at = T0 + PERIOD + HOLD_TIMEOUT + 1;
        assert_eq!(
            fx.gate().tick(latched_at, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        assert_held(&fx, &old, "the latch leaves the sample stream truthful");

        let rearmed_at = latched_at + PERIOD;
        fx.gate().release_latch(rearmed_at);
        assert_holds_nothing(&fx, "the dead session's setpoint went with it");
        for step in 0..5 {
            assert_eq!(
                fx.gate()
                    .tick(rearmed_at + step * PERIOD, true, HOLD_TIMEOUT),
                GateAction::Nothing,
                "cycle {step}: a re-armed gate writes nothing of the old session's",
            );
        }

        let fresh = goal_at(rearmed_at + 6 * PERIOD, 0.1);
        fx.gate().accept(view(&fresh), rearmed_at + 5 * PERIOD);
        assert_eq!(
            fx.gate().tick(rearmed_at + 6 * PERIOD, true, HOLD_TIMEOUT),
            GateAction::WriteGoal,
            "and writes what the new session asks for",
        );
    }

    /// The ordering baseline goes with the held setpoint: the first goal of a
    /// new session is measured against nothing, not against an instant from
    /// before the machine was de-torqued.
    #[test]
    fn the_first_goal_after_a_re_arming_is_not_out_of_order() {
        let mut fx = armed_at(T0);
        fx.gate().accept(view(&goal_at(T0 + 100 * PERIOD, 1.0)), T0);
        fx.gate().tick(T0 + 100 * PERIOD, true, HOLD_TIMEOUT);
        fx.gate().latch_torque_off();

        let rearmed_at = T0 + 101 * PERIOD;
        fx.gate().release_latch(rearmed_at);
        assert_eq!(
            fx.gate()
                .accept(view(&goal_at(rearmed_at + PERIOD, 2.0)), rearmed_at),
            AcceptOutcome::Accepted,
            "nothing from the dead session is an ordering baseline",
        );
    }

    #[test]
    fn a_confirmed_disarm_forgets_what_was_commanded_without_latching() {
        let mut fx = armed_at(T0);
        let goal = goal_at(T0 + PERIOD, 0.5);
        fx.gate().accept(view(&goal), T0);
        fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        fx.gate()
            .accept(view(&goal_at(T0 + 4 * PERIOD, 0.75)), T0 + PERIOD);

        fx.gate().clear_commanded();
        assert_holds_nothing(&fx, "nothing is being held any more");
        assert_eq!(fx.queued(), 0);
        assert!(
            !fx.state().latched.get(),
            "a deliberate disarm is not a fault"
        );
        assert_eq!(
            fx.gate().tick(T0 + 5 * PERIOD, false, HOLD_TIMEOUT),
            GateAction::Nothing,
            "a gate holding nothing writes nothing"
        );
    }

    /// A disarm says nothing about the dead-man. The machine is de-torqued, so
    /// what holds the timer off is the host's belief and not this call.
    #[test]
    fn a_confirmed_disarm_leaves_the_dead_man_where_it_was() {
        let mut fx = armed_at(T0);
        fx.gate().clear_commanded();
        assert!(fx.state().has_accepted.get(), "the arming still happened");
        assert_eq!(fx.state().last_accept.as_nanos(), T0);
        assert_eq!(
            fx.gate().tick(T0 + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP,
            "a machine believed torqued with nobody talking still de-torques"
        );
    }

    #[test]
    fn extreme_timestamps_do_not_overflow_the_dead_man() {
        let mut fx = Fixture::new();
        fx.gate().note_liveness(i64::MIN);
        assert_eq!(fx.gate().tick(i64::MAX, true, HOLD_TIMEOUT), LATCHING_SWEEP);

        let mut fx = Fixture::new();
        fx.gate().note_liveness(i64::MAX);
        assert_eq!(
            fx.gate().tick(i64::MIN, true, HOLD_TIMEOUT),
            GateAction::Nothing
        );
        assert!(!fx.state().latched.get());
    }

    /// The dead-man's own latch is announced once and never again.
    ///
    /// The edge is the event: a host reporting one per cycle for as long as a
    /// machine stands de-torqued reports at poll rate, and a host trying to
    /// recover the edge from the held setpoint misses the latch that fires
    /// before anything was ever commanded. Neither is left to the host to
    /// work out.
    #[test]
    fn a_dead_man_latch_is_announced_on_its_own_cycle_and_no_other() {
        let mut fx = armed_at(T0);
        fx.gate().accept(view(&goal_at(T0 + PERIOD, 0.0)), T0);
        fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        assert_eq!(
            fx.gate().tick(T0 + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        for step in 2..12 {
            assert_eq!(
                fx.gate()
                    .tick(T0 + HOLD_TIMEOUT + step * PERIOD, true, HOLD_TIMEOUT),
                STANDING_SWEEP,
                "cycle {step}"
            );
        }
    }

    /// The same, for the latch that fires with nothing ever commanded: the
    /// case a host keying its event off the held setpoint would report as
    /// nothing at all.
    #[test]
    fn a_dead_man_latch_before_the_first_goal_is_announced_too() {
        let mut fx = armed_at(T0);
        assert_eq!(
            fx.gate().tick(T0 + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        assert!(!fx.state().has_held.get(), "nothing was ever commanded");
        assert_eq!(
            fx.gate()
                .tick(T0 + HOLD_TIMEOUT + PERIOD, true, HOLD_TIMEOUT),
            STANDING_SWEEP
        );
    }

    /// A latch the host asked for is not the dead-man announcing anything: the
    /// host already knows why the machine is de-torqued, and an edge reported
    /// here would be a second event for one de-torque.
    #[test]
    fn a_latch_the_host_asked_for_announces_no_edge() {
        let mut fx = armed_at(T0);
        fx.gate().latch_torque_off();
        assert_eq!(
            fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT),
            STANDING_SWEEP
        );
    }

    /// Two latches in one session are two edges: a release and a fresh stall
    /// announce the second one as loudly as the first.
    #[test]
    fn a_second_stall_after_a_re_arming_is_announced_again() {
        let mut fx = armed_at(T0);
        assert_eq!(
            fx.gate().tick(T0 + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        let rearmed_at = T0 + 2 * HOLD_TIMEOUT;
        fx.gate().release_latch(rearmed_at);
        assert_eq!(
            fx.gate()
                .tick(rearmed_at + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
    }

    /// The latch outranks a goal that has come due.
    ///
    /// `accept` does not refuse goals while latched, so a latched gate really
    /// can hold one that is due. Writing it would put a setpoint on a machine
    /// that is meant to be de-torqued, which is the one ordering the gate
    /// promises unconditionally.
    #[test]
    fn a_due_goal_does_not_escape_a_standing_latch() {
        let mut fx = armed_at(T0);
        fx.gate().latch_torque_off();
        let due = goal_at(T0 - PERIOD, 1.0);
        fx.gate().accept(view(&due), T0);
        assert_eq!(fx.queued(), 1, "the goal really is queued and due");

        for step in 0..3 {
            assert_eq!(
                fx.gate().tick(T0 + step * PERIOD, true, HOLD_TIMEOUT),
                STANDING_SWEEP,
                "cycle {step}"
            );
            assert_holds_nothing(&fx, &format!("cycle {step}: a setpoint was taken up"));
        }
    }

    /// Torque coming back on after a long de-torqued stretch, with nobody
    /// having said anything: the window it is measured against ran out while
    /// the machine was down, so the first torqued cycle de-torques it again.
    ///
    /// The fail-safe direction, and the reason the host's arming path has to
    /// call [`GoalGate::note_liveness`]. A gate that refreshed the window on
    /// the torqued edge by itself would arm a machine with the dead-man
    /// already spent.
    #[test]
    fn belief_returning_without_a_word_latches_on_the_first_torqued_cycle() {
        let mut fx = armed_at(T0);
        fx.gate().accept(view(&goal_at(T0 + PERIOD, 0.0)), T0);
        fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        for step in 1..20 {
            let quiet = fx
                .gate()
                .tick(T0 + step * HOLD_TIMEOUT, false, HOLD_TIMEOUT);
            assert!(quiet == GateAction::Rewrite, "cycle {step}");
        }
        assert_eq!(
            fx.gate().tick(T0 + 20 * HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
    }

    /// The same stretch, armed properly: the arming grants the window, and it
    /// is measured from the arming rather than from whenever the last goal
    /// happened to arrive.
    #[test]
    fn belief_returning_after_an_arming_starts_the_window_at_the_arming() {
        let mut fx = armed_at(T0);
        fx.gate().accept(view(&goal_at(T0 + PERIOD, 0.0)), T0);
        fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        for step in 1..20 {
            fx.gate()
                .tick(T0 + step * HOLD_TIMEOUT, false, HOLD_TIMEOUT);
        }

        let armed_again = T0 + 20 * HOLD_TIMEOUT;
        fx.gate().note_liveness(armed_again);
        assert_eq!(
            fx.gate().tick(armed_again, true, HOLD_TIMEOUT),
            GateAction::Rewrite
        );
        assert_eq!(
            fx.gate()
                .tick(armed_again + HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            GateAction::Rewrite
        );
        assert_eq!(
            fx.gate()
                .tick(armed_again + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP,
            "the window is measured from the arming"
        );
    }

    /// A gate out of a slot nothing wrote is a gate that has been told nothing.
    ///
    /// The state a fresh slot holds is the state the gate starts from, so there
    /// is nothing to restore and no length to disbelieve: this crate runs in the
    /// process that de-torques the machine when the commander stops answering,
    /// and a queue whose depth was a separate number is a way for that process
    /// to meet one it cannot trust.
    #[test]
    fn a_slot_nothing_wrote_is_a_gate_that_has_been_told_nothing() {
        let slot = GateStateWire::new();
        let state = slot.validate().expect("a cleared slot is a gate");
        assert_eq!(state.queue.len(), 0);
        assert!(!state.has_held.get());
        assert!(!state.latched.get());
        assert!(!state.has_accepted.get());

        let mut fx = Fixture { slot };
        assert_eq!(
            fx.gate().tick(T0 + 10 * HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            GateAction::Nothing,
            "the dead-man does not run on a driver nobody has spoken to"
        );
    }

    /// The queue is as deep as it is declared, and the depth is one number.
    #[test]
    fn the_queue_is_exactly_as_deep_as_the_schema_declares() {
        let mut fx = armed_at(T0);
        let depth = fx.state().queue.capacity();
        for i in 1..=depth {
            assert_eq!(
                fx.gate()
                    .accept(view(&goal_at(T0 + i as i64 * PERIOD, 0.0)), T0),
                AcceptOutcome::Accepted,
                "goal {i}"
            );
        }
        assert_eq!(
            fx.gate().accept(view(&goal_at(T0 + 99 * PERIOD, 0.0)), T0),
            AcceptOutcome::DroppedQueueFull
        );
        assert_eq!(fx.queued(), depth);
    }

    /// The row count and the mask are read off the joint vocabulary, so the
    /// two cannot disagree about which bit is which servo. What is checked here
    /// is that every declared servo is inside the mask and that the mask is
    /// exactly the declared servos -- a tenth servo widens both or fails here.
    #[test]
    fn every_joint_is_a_bit_of_the_mask() {
        let mut every = 0u16;
        for flag in JointFlags::VARIANTS {
            let bits = JointFlagsWire::from(flag).0;
            assert_eq!(
                bits & !JOINT_MASK_ALL,
                0,
                "{flag} names a bus row the mask does not have"
            );
            every |= bits;
        }
        assert_eq!(every, JOINT_MASK_ALL, "the mask is the declared servos");
        assert_eq!(usize::from(JOINT_MASK_ALL.count_ones() as u16), JOINT_COUNT);
        assert_eq!(
            u16::from(every_row()),
            JOINT_MASK_ALL,
            "the set and the mask are the same servos said two ways"
        );
    }

    /// A decision is made on the state itself, not on a copy of it.
    ///
    /// The whole point of the gate's state being a schema: what a decision
    /// leaves is what the next cycle reads. A gate that could be copied would be
    /// a second description of a queue, which is how a machine ends up holding
    /// torque because the copy nobody looked at was the one that timed out.
    #[test]
    fn a_decision_lands_in_the_state_the_next_one_reads() {
        let mut fx = armed_at(T0);
        let goal = goal_at(T0 + PERIOD, 0.5);
        fx.gate().accept(view(&goal), T0);
        assert_eq!(fx.queued(), 1);
        assert_eq!(
            fx.gate().tick(T0 + PERIOD, true, HOLD_TIMEOUT),
            GateAction::WriteGoal
        );
        assert_eq!(fx.queued(), 0, "the executed goal left the queue");
        assert_held(&fx, &goal, "and became what is held");
    }
}
