//! `reachy-driver` — the goal gate and the dead-man, with no motors attached.
//!
//! A motor driver's cycle is mostly bus traffic, but the part of it that
//! decides *whether and what to write* is pure: a small FIFO of goals stamped
//! with the grid instant they are to be executed at, a held setpoint that gets
//! rewritten between due goals, and a timer that de-torques the machine when
//! the process feeding it goals goes quiet. That decision is this crate, and
//! nothing else is.
//!
//! It exists separately because the same decision has to run in three places
//! and be the same decision in all of them: the real driver process, the
//! simulated driver that the control loop is tested against, and the unit
//! tests that pin the semantics down. A second implementation of a dead-man
//! timer is a second set of conditions under which a machine does not
//! de-torque.
//!
//! The one crate it links is `reachy-wire`, which is the layout the goals it
//! gates arrive in and is itself dependency-free plain data. Sharing that is
//! the point: the row count, the mask convention and the setpoint's own fields
//! are stated once, so a host decoding a datagram and handing it to the gate
//! writes no conversion of its own that could disagree.
//!
//! The whole state is public `Copy` data, because it is mirrored field for
//! field into a Clockwork state schema: a cog's cross-tick state lives in a
//! declared slot, so anything with a private field or a heap allocation cannot
//! be a cog's gate. Public fields mean the one invariant they carry —
//! `queue_len` is at most [`QUEUE_CAP`] — is not a constructor's to establish,
//! so a host restoring a gate out of a slot checks it with
//! [`GoalGate::validate`]. Nothing here panics on a gate that fails it; a
//! queue length past the end is read as the end, because the process this runs
//! in is the one that de-torques the machine and it does not get to crash over
//! a bad slot.
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
//! Times are nanoseconds since the Unix epoch, positions are radians in bus
//! order (body yaw, legs 0 through 5, right antenna, left antenna), and masks
//! are one bit per bus row — the same conventions as `reachy-wire`, which is
//! what carries these values between processes.

#![forbid(unsafe_code)]

use reachy_wire::GoalSetpoint;

/// Servo rows on the bus, and every joint in a mask: re-exported from the wire
/// layout rather than restated, because a gate whose row count disagreed with
/// the datagrams feeding it would silently stop writing the rows above the
/// disagreement.
pub use reachy_wire::{JOINT_COUNT, JOINT_MASK_ALL};

/// How many goals may be queued ahead of execution.
///
/// A sender running a lag of `k` cycles has `k` goals in flight, so the queue
/// only has to be deeper than the largest lag anything sensibly uses. Five is
/// that with room to spare; a sender that overruns it is malfunctioning, and
/// the drop is reported rather than absorbed.
pub const QUEUE_CAP: usize = 5;

/// A commanded setpoint and the grid instant it is for.
///
/// The same three values a [`GoalSetpoint`] carries on the wire, in the form
/// the queue holds them. The two convert both ways with [`From`], so no host
/// writes that conversion by hand: a driver process and a simulated one must
/// not be able to disagree about which field is which.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Goal {
    /// The instant this setpoint is to be written at.
    pub execute_at_ns: i64,
    /// The rows it writes, one bit per bus row.
    pub mask: u16,
    /// Targets in bus order, radians. Rows outside `mask` are not written.
    pub targets: [f64; JOINT_COUNT],
}

impl Default for Goal {
    fn default() -> Self {
        Self::EMPTY
    }
}

impl Goal {
    /// A goal that writes nothing, used to fill unoccupied queue slots.
    pub const EMPTY: Self = Self {
        execute_at_ns: 0,
        mask: 0,
        targets: [0.0; JOINT_COUNT],
    };

    /// Write this goal's targets into `targets`, and report which rows moved.
    ///
    /// The mask is write-side filtering and nothing else: a row outside it
    /// keeps whatever it was already holding, and the value carried for that
    /// row here is not written anywhere. Every host of the gate applies a
    /// setpoint through this function so that "what does a partial mask mean"
    /// has exactly one answer.
    pub fn apply_to(&self, targets: &mut [f64; JOINT_COUNT]) -> u16 {
        for (row, target) in targets.iter_mut().enumerate() {
            if self.mask & (1 << row) != 0 {
                *target = self.targets[row];
            }
        }
        self.mask
    }
}

impl From<GoalSetpoint> for Goal {
    fn from(setpoint: GoalSetpoint) -> Self {
        Self {
            execute_at_ns: setpoint.execute_at_ns,
            mask: setpoint.mask,
            targets: setpoint.targets,
        }
    }
}

impl From<Goal> for GoalSetpoint {
    fn from(goal: Goal) -> Self {
        Self {
            execute_at_ns: goal.execute_at_ns,
            mask: goal.mask,
            targets: goal.targets,
        }
    }
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

/// A gate restored from a slot holding something no gate ever wrote.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum GateStateError {
    /// More queued goals claimed than the queue holds.
    QueueLenOutOfRange {
        /// What the slot said.
        queue_len: u8,
    },
}

impl core::fmt::Display for GateStateError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::QueueLenOutOfRange { queue_len } => write!(
                f,
                "queue_len {queue_len} is past the {QUEUE_CAP}-goal queue"
            ),
        }
    }
}

impl core::error::Error for GateStateError {}

/// What the host should do to the bus this cycle.
#[derive(Clone, Copy, Debug, PartialEq)]
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
    /// Write this setpoint: it is a newly due goal.
    WriteGoal(Goal),
    /// Write this setpoint again: no new goal is due, and the held one stands.
    Rewrite(Goal),
    /// Write nothing. Nothing has ever been commanded.
    Nothing,
}

/// The gate: a goal queue, the setpoint it is holding, and the dead-man.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GoalGate {
    /// Queued goals, oldest first. Only the first `queue_len` are occupied.
    pub queue: [Goal; QUEUE_CAP],
    /// How many of `queue` are occupied.
    pub queue_len: u8,
    /// The setpoint currently held. Meaningful only when `has_held`.
    pub held: Goal,
    /// Whether anything has ever been commanded.
    pub has_held: bool,
    /// Whether the gate has latched torque off.
    pub latched: bool,
    /// When liveness was last observed. Meaningful only when `has_accepted`.
    pub last_accept_ns: i64,
    /// Whether liveness has ever been observed. Until it has, the dead-man
    /// does not run: a driver that has never been armed is not a machine going
    /// quiet, it is a machine nobody has spoken to yet.
    pub has_accepted: bool,
}

impl Default for GoalGate {
    fn default() -> Self {
        Self::new()
    }
}

impl GoalGate {
    /// A gate holding nothing, with the dead-man not yet running.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            queue: [Goal::EMPTY; QUEUE_CAP],
            queue_len: 0,
            held: Goal::EMPTY,
            has_held: false,
            latched: false,
            last_accept_ns: 0,
            has_accepted: false,
        }
    }

    /// Whether this gate describes a state a gate can be in.
    ///
    /// What a host restoring the gate out of a slot calls before ticking it. A
    /// slot that no gate wrote — one nothing has written at all, or one written
    /// by something that disagrees about the layout — can claim more queued
    /// goals than the queue holds, and a host is better told that than handed a
    /// gate quietly reading a shorter queue than its own state says it has.
    ///
    /// # Errors
    ///
    /// [`GateStateError`], naming what about the state is impossible.
    pub fn validate(&self) -> Result<(), GateStateError> {
        if usize::from(self.queue_len) > QUEUE_CAP {
            return Err(GateStateError::QueueLenOutOfRange {
                queue_len: self.queue_len,
            });
        }
        Ok(())
    }

    /// How many queued goals there really are: what `queue_len` says, or the
    /// end of the queue if it says something past it.
    ///
    /// Every read of the queue goes through this. A host that skipped
    /// [`Self::validate`] gets a gate that gates, rather than an index panic in
    /// the process whose whole job is to de-torque a machine when things go
    /// wrong.
    fn len(&self) -> usize {
        usize::from(self.queue_len).min(QUEUE_CAP)
    }

    /// The queued goals, oldest first.
    #[must_use]
    pub fn queued(&self) -> &[Goal] {
        &self.queue[..self.len()]
    }

    /// Offer a goal to the gate.
    ///
    /// An accepted goal is liveness: it is the evidence that whatever is
    /// supposed to be commanding this machine is still there, which is what
    /// the dead-man measures. A dropped one is not — a sender overrunning the
    /// queue must not be able to hold torque on by overrunning it faster.
    pub fn accept(&mut self, goal: Goal, now_ns: i64) -> AcceptOutcome {
        let len = self.len();
        if len >= QUEUE_CAP {
            return AcceptOutcome::DroppedQueueFull;
        }
        let previous = if len > 0 {
            Some(self.queue[len - 1])
        } else if self.has_held {
            Some(self.held)
        } else {
            None
        };
        let out_of_order = previous.is_some_and(|p| goal.execute_at_ns <= p.execute_at_ns);
        let stale = goal.execute_at_ns < now_ns;
        self.queue[len] = goal;
        self.queue_len += 1;
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
        self.last_accept_ns = now_ns;
        self.has_accepted = true;
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
        self.latched = true;
        self.clear_queue();
    }

    /// Empty the queue, in the one shape [`GoalGate::validate`] accepts.
    ///
    /// Written once because the invariant is one invariant: entries past the
    /// length are `EMPTY`, and a re-establishment that missed a field would
    /// leave a goal behind for a machine that no longer exists.
    fn clear_queue(&mut self) {
        self.queue_len = 0;
        self.queue = [Goal::EMPTY; QUEUE_CAP];
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
        self.latched = false;
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
        self.clear_queue();
        self.held = Goal::EMPTY;
        self.has_held = false;
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
        if self.latched {
            return GateAction::WriteTorqueOffSweep {
                just_latched: false,
            };
        }
        if believed_torqued
            && self.has_accepted
            && nominal_ns.saturating_sub(self.last_accept_ns) > hold_timeout_ns
        {
            self.latch_torque_off();
            return GateAction::WriteTorqueOffSweep { just_latched: true };
        }
        if self.len() > 0 && self.queue[0].execute_at_ns <= nominal_ns {
            let goal = self.pop();
            self.held = goal;
            self.has_held = true;
            return GateAction::WriteGoal(goal);
        }
        if self.has_held {
            return GateAction::Rewrite(self.held);
        }
        GateAction::Nothing
    }

    /// Remove and return the head of the queue. Only called with a non-empty
    /// queue.
    fn pop(&mut self) -> Goal {
        let head = self.queue[0];
        let len = self.len();
        self.queue.copy_within(1..len, 0);
        self.queue_len = (len - 1) as u8;
        self.queue[self.len()] = Goal::EMPTY;
        head
    }
}

#[cfg(test)]
mod tests {
    use super::*;

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

    fn goal_at(execute_at_ns: i64, value: f64) -> Goal {
        Goal {
            execute_at_ns,
            mask: JOINT_MASK_ALL,
            targets: [value; JOINT_COUNT],
        }
    }

    /// A gate that has just been armed: torque on, liveness seeded, nothing
    /// commanded yet.
    fn armed_at(now_ns: i64) -> GoalGate {
        let mut gate = GoalGate::new();
        gate.note_liveness(now_ns);
        gate
    }

    #[test]
    fn a_fresh_gate_writes_nothing() {
        let mut gate = GoalGate::new();
        assert_eq!(gate.tick(T0, false, HOLD_TIMEOUT), GateAction::Nothing);
        assert_eq!(
            gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT),
            GateAction::Nothing
        );
        assert!(!gate.has_held);
    }

    #[test]
    fn a_goal_executes_at_its_instant_and_not_before() {
        let mut gate = armed_at(T0);
        let goal = goal_at(T0 + 2 * PERIOD, 0.5);
        assert_eq!(gate.accept(goal, T0), AcceptOutcome::Accepted);
        assert_eq!(gate.tick(T0, true, HOLD_TIMEOUT), GateAction::Nothing);
        assert_eq!(
            gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT),
            GateAction::Nothing
        );
        assert_eq!(
            gate.tick(T0 + 2 * PERIOD, true, HOLD_TIMEOUT),
            GateAction::WriteGoal(goal)
        );
    }

    #[test]
    fn a_due_goal_becomes_the_held_setpoint_and_is_rewritten_after() {
        let mut gate = armed_at(T0);
        let goal = goal_at(T0 + PERIOD, 0.25);
        gate.accept(goal, T0);
        assert_eq!(
            gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT),
            GateAction::WriteGoal(goal)
        );
        for step in 2..6 {
            assert_eq!(
                gate.tick(T0 + step * PERIOD, true, HOLD_TIMEOUT),
                GateAction::Rewrite(goal),
                "cycle {step}"
            );
        }
        assert_eq!(gate.held, goal);
    }

    #[test]
    fn a_backlog_drains_one_goal_per_tick_in_order() {
        let mut gate = armed_at(T0);
        let goals: Vec<Goal> = (1..=3)
            .map(|i| goal_at(T0 + i * PERIOD, f64::from(i as i32)))
            .collect();
        for goal in &goals {
            assert_eq!(gate.accept(*goal, T0), AcceptOutcome::Accepted);
        }
        // All three are due at once: the gate is a cycle behind.
        let late = T0 + 10 * PERIOD;
        assert_eq!(
            gate.tick(late, true, HOLD_TIMEOUT),
            GateAction::WriteGoal(goals[0])
        );
        assert_eq!(
            gate.tick(late, true, HOLD_TIMEOUT),
            GateAction::WriteGoal(goals[1])
        );
        assert_eq!(
            gate.tick(late, true, HOLD_TIMEOUT),
            GateAction::WriteGoal(goals[2])
        );
        assert_eq!(
            gate.tick(late, true, HOLD_TIMEOUT),
            GateAction::Rewrite(goals[2])
        );
        assert_eq!(gate.queue_len, 0);
    }

    #[test]
    fn a_full_queue_drops_and_says_so_without_disturbing_what_is_queued() {
        let mut gate = armed_at(T0);
        for i in 1..=QUEUE_CAP {
            let goal = goal_at(T0 + i as i64 * PERIOD, i as f64);
            assert_eq!(gate.accept(goal, T0), AcceptOutcome::Accepted);
        }
        let before = gate.queued().to_vec();
        let overflow = goal_at(T0 + 99 * PERIOD, 99.0);
        assert_eq!(gate.accept(overflow, T0), AcceptOutcome::DroppedQueueFull);
        assert_eq!(gate.queued(), before.as_slice());
        assert_eq!(gate.queue_len as usize, QUEUE_CAP);
    }

    #[test]
    fn a_dropped_goal_is_not_liveness() {
        let mut gate = armed_at(T0);
        for i in 1..=QUEUE_CAP {
            gate.accept(goal_at(T0 + i as i64 * PERIOD, 0.0), T0);
        }
        assert_eq!(gate.last_accept_ns, T0);
        gate.accept(goal_at(T0 + 99 * PERIOD, 0.0), T0 + 5 * PERIOD);
        assert_eq!(
            gate.last_accept_ns, T0,
            "a refused goal must not feed the dead-man"
        );
    }

    #[test]
    fn a_stale_goal_is_accepted_warned_and_executed_in_arrival_order() {
        let mut gate = armed_at(T0);
        let stale = goal_at(T0 - PERIOD, 1.0);
        assert_eq!(
            gate.accept(stale, T0),
            AcceptOutcome::AcceptedStaleOrOutOfOrder
        );
        assert_eq!(
            gate.tick(T0, true, HOLD_TIMEOUT),
            GateAction::WriteGoal(stale)
        );
    }

    #[test]
    fn an_out_of_order_goal_is_accepted_warned_and_never_reordered() {
        let mut gate = armed_at(T0);
        let later = goal_at(T0 + 4 * PERIOD, 1.0);
        let earlier = goal_at(T0 + 2 * PERIOD, 2.0);
        assert_eq!(gate.accept(later, T0), AcceptOutcome::Accepted);
        assert_eq!(
            gate.accept(earlier, T0),
            AcceptOutcome::AcceptedStaleOrOutOfOrder
        );
        let late = T0 + 8 * PERIOD;
        assert_eq!(
            gate.tick(late, true, HOLD_TIMEOUT),
            GateAction::WriteGoal(later)
        );
        assert_eq!(
            gate.tick(late, true, HOLD_TIMEOUT),
            GateAction::WriteGoal(earlier)
        );
    }

    #[test]
    fn a_goal_no_later_than_the_held_setpoint_is_out_of_order_too() {
        let mut gate = armed_at(T0);
        let first = goal_at(T0 + PERIOD, 1.0);
        gate.accept(first, T0);
        gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        assert_eq!(
            gate.accept(goal_at(T0 + PERIOD, 2.0), T0 + PERIOD),
            AcceptOutcome::AcceptedStaleOrOutOfOrder
        );
    }

    #[test]
    fn the_dead_man_fires_one_nanosecond_past_the_timeout_and_not_at_it() {
        let mut gate = armed_at(T0);
        gate.accept(goal_at(T0 + PERIOD, 0.0), T0);
        gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        assert!(matches!(
            gate.tick(T0 + HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            GateAction::Rewrite(_)
        ));
        assert_eq!(
            gate.tick(T0 + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        assert!(gate.latched);
    }

    #[test]
    fn the_dead_man_fires_with_nothing_ever_commanded() {
        // Armed, torqued, and the commander never sent a first goal. This is
        // the window a dead-man written around the held setpoint would miss.
        let mut gate = armed_at(T0);
        assert_eq!(
            gate.tick(T0 + HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            GateAction::Nothing
        );
        assert_eq!(
            gate.tick(T0 + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        assert!(gate.latched);
        assert!(!gate.has_held);
    }

    #[test]
    fn the_dead_man_does_not_run_before_anything_is_armed() {
        let mut gate = GoalGate::new();
        assert_eq!(
            gate.tick(T0 + 10 * HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            GateAction::Nothing
        );
        assert!(!gate.latched);
    }

    #[test]
    fn the_dead_man_does_not_run_on_a_machine_that_is_not_torqued() {
        let mut gate = armed_at(T0);
        gate.accept(goal_at(T0 + PERIOD, 0.0), T0);
        gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        for step in 1..20 {
            let action = gate.tick(T0 + step * HOLD_TIMEOUT, false, HOLD_TIMEOUT);
            assert!(
                matches!(action, GateAction::Rewrite(_)),
                "cycle {step}: {action:?}"
            );
        }
        assert!(!gate.latched);
    }

    #[test]
    fn a_rewrite_is_not_liveness() {
        // Only a datagram from outside refreshes the window. If rewriting the
        // held setpoint counted, the dead-man would be measuring the driver's
        // own pulse and would never fire.
        let mut gate = armed_at(T0);
        gate.accept(goal_at(T0 + PERIOD, 0.0), T0);
        let mut now = T0 + PERIOD;
        let mut rewrites = 0;
        loop {
            match gate.tick(now, true, HOLD_TIMEOUT) {
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
        let mut gate = armed_at(T0);
        let mut now = T0;
        for step in 1..500 {
            now = T0 + step * PERIOD;
            gate.accept(goal_at(now + 2 * PERIOD, 0.0), now);
            let action = gate.tick(now, true, HOLD_TIMEOUT);
            assert!(
                !matches!(action, GateAction::WriteTorqueOffSweep { .. }),
                "cycle {step}"
            );
        }
        assert!(!gate.latched);
        assert_eq!(gate.last_accept_ns, now);
    }

    #[test]
    fn a_latch_clears_the_queue_and_stays_until_released() {
        let mut gate = armed_at(T0);
        gate.accept(goal_at(T0 + PERIOD, 0.0), T0);
        gate.accept(goal_at(T0 + 2 * PERIOD, 0.0), T0);
        gate.latch_torque_off();
        assert_eq!(gate.queue_len, 0);
        assert_eq!(gate.queued(), &[]);
        for step in 0..10 {
            assert_eq!(
                gate.tick(T0 + step * PERIOD, true, HOLD_TIMEOUT),
                STANDING_SWEEP
            );
        }
    }

    #[test]
    fn a_goal_that_arrives_during_a_latch_does_not_execute_after_it() {
        let mut gate = armed_at(T0);
        gate.latch_torque_off();
        gate.accept(goal_at(T0 + PERIOD, 1.0), T0);
        gate.release_latch(T0);
        assert_eq!(gate.queue_len, 0);
        assert_eq!(
            gate.tick(T0 + PERIOD, false, HOLD_TIMEOUT),
            GateAction::Nothing
        );
    }

    #[test]
    fn releasing_the_latch_is_all_it_takes_to_stay_armed() {
        // The whole of a re-arming, with no second call to remember: if the
        // release left the old liveness instant standing, this cycle would
        // re-latch and report it as a stalled commander.
        let mut gate = armed_at(T0);
        let latched_at = T0 + HOLD_TIMEOUT + 1;
        assert_eq!(gate.tick(latched_at, true, HOLD_TIMEOUT), LATCHING_SWEEP);
        let rearmed_at = latched_at + PERIOD;
        gate.release_latch(rearmed_at);
        assert_eq!(
            gate.tick(rearmed_at, true, HOLD_TIMEOUT),
            GateAction::Nothing,
            "a released latch must not fire the dead-man on stale evidence"
        );
        assert!(!gate.latched);
    }

    #[test]
    fn releasing_the_latch_grants_a_fresh_dead_man_window() {
        let mut gate = armed_at(T0);
        let latched_at = T0 + HOLD_TIMEOUT + 1;
        assert_eq!(gate.tick(latched_at, true, HOLD_TIMEOUT), LATCHING_SWEEP);
        let rearmed_at = latched_at + PERIOD;
        gate.release_latch(rearmed_at);
        assert_eq!(
            gate.tick(rearmed_at, true, HOLD_TIMEOUT),
            GateAction::Nothing
        );
        assert_eq!(
            gate.tick(rearmed_at + HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            GateAction::Nothing,
            "the window is measured from the re-arming, not from the old latch"
        );
        assert_eq!(
            gate.tick(rearmed_at + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
    }

    #[test]
    fn a_held_setpoint_survives_a_latch_so_the_sample_stream_stays_truthful() {
        // The latch stops writes; it does not erase what was last commanded,
        // which is what a driver reports as `commanded` in its samples.
        let mut gate = armed_at(T0);
        let goal = goal_at(T0 + PERIOD, 0.75);
        gate.accept(goal, T0);
        gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        gate.latch_torque_off();
        assert!(gate.has_held);
        assert_eq!(gate.held, goal);
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
        let mut gate = armed_at(T0);
        let old = goal_at(T0 + PERIOD, 0.75);
        gate.accept(old, T0);
        gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        let latched_at = T0 + PERIOD + HOLD_TIMEOUT + 1;
        assert_eq!(gate.tick(latched_at, true, HOLD_TIMEOUT), LATCHING_SWEEP);
        assert_eq!(
            gate.held, old,
            "the latch leaves the sample stream truthful"
        );

        let rearmed_at = latched_at + PERIOD;
        gate.release_latch(rearmed_at);
        assert!(!gate.has_held, "the dead session's setpoint went with it");
        assert_eq!(gate.held, Goal::EMPTY);
        for step in 0..5 {
            assert_eq!(
                gate.tick(rearmed_at + step * PERIOD, true, HOLD_TIMEOUT),
                GateAction::Nothing,
                "cycle {step}: a re-armed gate writes nothing of the old session's",
            );
        }

        let fresh = goal_at(rearmed_at + 6 * PERIOD, 0.1);
        gate.accept(fresh, rearmed_at + 5 * PERIOD);
        assert_eq!(
            gate.tick(rearmed_at + 6 * PERIOD, true, HOLD_TIMEOUT),
            GateAction::WriteGoal(fresh),
            "and writes what the new session asks for",
        );
    }

    /// The ordering baseline goes with the held setpoint: the first goal of a
    /// new session is measured against nothing, not against an instant from
    /// before the machine was de-torqued.
    #[test]
    fn the_first_goal_after_a_re_arming_is_not_out_of_order() {
        let mut gate = armed_at(T0);
        gate.accept(goal_at(T0 + 100 * PERIOD, 1.0), T0);
        gate.tick(T0 + 100 * PERIOD, true, HOLD_TIMEOUT);
        gate.latch_torque_off();

        let rearmed_at = T0 + 101 * PERIOD;
        gate.release_latch(rearmed_at);
        assert_eq!(
            gate.accept(goal_at(rearmed_at + PERIOD, 2.0), rearmed_at),
            AcceptOutcome::Accepted,
            "nothing from the dead session is an ordering baseline",
        );
    }

    #[test]
    fn a_confirmed_disarm_forgets_what_was_commanded_without_latching() {
        let mut gate = armed_at(T0);
        let goal = goal_at(T0 + PERIOD, 0.5);
        gate.accept(goal, T0);
        gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        gate.accept(goal_at(T0 + 4 * PERIOD, 0.75), T0 + PERIOD);

        gate.clear_commanded();
        assert!(!gate.has_held, "nothing is being held any more");
        assert_eq!(gate.held, Goal::EMPTY);
        assert_eq!(gate.queued(), &[]);
        assert!(!gate.latched, "a deliberate disarm is not a fault");
        assert_eq!(
            gate.tick(T0 + 5 * PERIOD, false, HOLD_TIMEOUT),
            GateAction::Nothing,
            "a gate holding nothing writes nothing"
        );
    }

    /// A disarm says nothing about the dead-man. The machine is de-torqued, so
    /// what holds the timer off is the host's belief and not this call.
    #[test]
    fn a_confirmed_disarm_leaves_the_dead_man_where_it_was() {
        let mut gate = armed_at(T0);
        gate.clear_commanded();
        assert!(gate.has_accepted, "the arming still happened");
        assert_eq!(gate.last_accept_ns, T0);
        assert_eq!(
            gate.tick(T0 + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP,
            "a machine believed torqued with nobody talking still de-torques"
        );
    }

    #[test]
    fn a_partial_mask_writes_only_its_own_rows() {
        let mut targets = [0.0; JOINT_COUNT];
        let goal = Goal {
            execute_at_ns: T0,
            mask: 0b0_0000_0101,
            targets: [1.0; JOINT_COUNT],
        };
        assert_eq!(goal.apply_to(&mut targets), 0b0_0000_0101);
        assert_eq!(targets, [1.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]);

        let other = Goal {
            execute_at_ns: T0,
            mask: JOINT_MASK_ALL & !0b0_0000_0101,
            targets: [2.0; JOINT_COUNT],
        };
        other.apply_to(&mut targets);
        assert_eq!(targets, [1.0, 2.0, 1.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0]);
    }

    #[test]
    fn an_empty_mask_writes_nothing() {
        let mut targets = [7.0; JOINT_COUNT];
        let goal = Goal {
            execute_at_ns: T0,
            mask: 0,
            targets: [1.0; JOINT_COUNT],
        };
        assert_eq!(goal.apply_to(&mut targets), 0);
        assert_eq!(targets, [7.0; JOINT_COUNT]);
    }

    #[test]
    fn extreme_timestamps_do_not_overflow_the_dead_man() {
        let mut gate = GoalGate::new();
        gate.note_liveness(i64::MIN);
        assert_eq!(gate.tick(i64::MAX, true, HOLD_TIMEOUT), LATCHING_SWEEP);

        let mut gate = GoalGate::new();
        gate.note_liveness(i64::MAX);
        assert_eq!(gate.tick(i64::MIN, true, HOLD_TIMEOUT), GateAction::Nothing);
        assert!(!gate.latched);
    }

    #[test]
    fn a_wire_setpoint_and_a_queued_goal_are_the_same_three_values() {
        // Every host of this gate receives goals as datagrams. The conversion
        // is here, once, so no host writes a field-by-field copy that could put
        // the mask where the instant goes.
        let setpoint = reachy_wire::GoalSetpoint {
            execute_at_ns: T0 + 3 * PERIOD,
            mask: 0b0_0001_0110,
            targets: core::array::from_fn(|row| row as f64 * 0.25),
        };
        let goal = Goal::from(setpoint);
        assert_eq!(goal.execute_at_ns, setpoint.execute_at_ns);
        assert_eq!(goal.mask, setpoint.mask);
        assert_eq!(goal.targets, setpoint.targets);
        assert_eq!(reachy_wire::GoalSetpoint::from(goal), setpoint);
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
        let mut gate = armed_at(T0);
        gate.accept(goal_at(T0 + PERIOD, 0.0), T0);
        gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        assert_eq!(
            gate.tick(T0 + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        for step in 2..12 {
            assert_eq!(
                gate.tick(T0 + HOLD_TIMEOUT + step * PERIOD, true, HOLD_TIMEOUT),
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
        let mut gate = armed_at(T0);
        assert_eq!(
            gate.tick(T0 + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        assert!(!gate.has_held, "nothing was ever commanded");
        assert_eq!(
            gate.tick(T0 + HOLD_TIMEOUT + PERIOD, true, HOLD_TIMEOUT),
            STANDING_SWEEP
        );
    }

    /// A latch the host asked for is not the dead-man announcing anything: the
    /// host already knows why the machine is de-torqued, and an edge reported
    /// here would be a second event for one de-torque.
    #[test]
    fn a_latch_the_host_asked_for_announces_no_edge() {
        let mut gate = armed_at(T0);
        gate.latch_torque_off();
        assert_eq!(gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT), STANDING_SWEEP);
    }

    /// Two latches in one session are two edges: a release and a fresh stall
    /// announce the second one as loudly as the first.
    #[test]
    fn a_second_stall_after_a_re_arming_is_announced_again() {
        let mut gate = armed_at(T0);
        assert_eq!(
            gate.tick(T0 + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
        let rearmed_at = T0 + 2 * HOLD_TIMEOUT;
        gate.release_latch(rearmed_at);
        assert_eq!(
            gate.tick(rearmed_at + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
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
        let mut gate = armed_at(T0);
        gate.latch_torque_off();
        let due = goal_at(T0 - PERIOD, 1.0);
        gate.accept(due, T0);
        assert_eq!(gate.queued(), &[due], "the goal really is queued and due");

        for step in 0..3 {
            assert_eq!(
                gate.tick(T0 + step * PERIOD, true, HOLD_TIMEOUT),
                STANDING_SWEEP,
                "cycle {step}"
            );
            assert!(!gate.has_held, "cycle {step}: a setpoint was taken up");
            assert_eq!(gate.held, Goal::EMPTY, "cycle {step}");
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
        let mut gate = armed_at(T0);
        gate.accept(goal_at(T0 + PERIOD, 0.0), T0);
        gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        for step in 1..20 {
            let quiet = gate.tick(T0 + step * HOLD_TIMEOUT, false, HOLD_TIMEOUT);
            assert!(matches!(quiet, GateAction::Rewrite(_)), "cycle {step}");
        }
        assert_eq!(
            gate.tick(T0 + 20 * HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            LATCHING_SWEEP
        );
    }

    /// The same stretch, armed properly: the arming grants the window, and it
    /// is measured from the arming rather than from whenever the last goal
    /// happened to arrive.
    #[test]
    fn belief_returning_after_an_arming_starts_the_window_at_the_arming() {
        let mut gate = armed_at(T0);
        gate.accept(goal_at(T0 + PERIOD, 0.0), T0);
        gate.tick(T0 + PERIOD, true, HOLD_TIMEOUT);
        for step in 1..20 {
            gate.tick(T0 + step * HOLD_TIMEOUT, false, HOLD_TIMEOUT);
        }

        let armed_again = T0 + 20 * HOLD_TIMEOUT;
        gate.note_liveness(armed_again);
        assert!(matches!(
            gate.tick(armed_again, true, HOLD_TIMEOUT),
            GateAction::Rewrite(_)
        ));
        assert!(matches!(
            gate.tick(armed_again + HOLD_TIMEOUT, true, HOLD_TIMEOUT),
            GateAction::Rewrite(_)
        ));
        assert_eq!(
            gate.tick(armed_again + HOLD_TIMEOUT + 1, true, HOLD_TIMEOUT),
            LATCHING_SWEEP,
            "the window is measured from the arming"
        );
    }

    #[test]
    fn a_queue_length_no_gate_wrote_is_named_rather_than_trusted() {
        let mut gate = armed_at(T0);
        assert_eq!(gate.validate(), Ok(()));
        gate.accept(goal_at(T0 + PERIOD, 0.0), T0);
        assert_eq!(gate.validate(), Ok(()));

        gate.queue_len = 200;
        assert_eq!(
            gate.validate(),
            Err(GateStateError::QueueLenOutOfRange { queue_len: 200 })
        );
        assert!(
            GateStateError::QueueLenOutOfRange { queue_len: 200 }
                .to_string()
                .contains("200")
        );
    }

    /// A gate restored from a slot nothing wrote still gates.
    ///
    /// This crate runs in the process that de-torques the machine when the
    /// commander stops answering. A panic there over a bad state field takes
    /// the dead-man down with it, so a queue length past the end of the queue
    /// is read as the end and every path keeps working.
    #[test]
    fn a_corrupt_queue_length_gates_instead_of_panicking() {
        let mut gate = armed_at(T0);
        let due = goal_at(T0, 1.0);
        gate.accept(due, T0);
        gate.queue_len = u8::MAX;

        assert_eq!(
            gate.queued().len(),
            QUEUE_CAP,
            "a queue read no further than it has slots"
        );
        assert_eq!(
            gate.accept(goal_at(T0 + PERIOD, 2.0), T0),
            AcceptOutcome::DroppedQueueFull
        );
        assert_eq!(
            gate.tick(T0, true, HOLD_TIMEOUT),
            GateAction::WriteGoal(due)
        );
        assert_eq!(gate.queued().len(), QUEUE_CAP - 1);
        assert_eq!(gate.validate(), Ok(()), "the length is in range again");
    }

    #[test]
    fn the_gate_is_plain_copy_data() {
        // The gate is mirrored into a Clockwork state slot field for field.
        // If it ever stops being `Copy` with public fields, that mirror stops
        // being possible, and this is where that is noticed.
        fn assert_copy<T: Copy>() {}
        assert_copy::<GoalGate>();
        assert_copy::<Goal>();
        let gate = armed_at(T0);
        let mut clone = gate;
        clone.accept(goal_at(T0 + PERIOD, 0.0), T0);
        assert_eq!(gate.queue_len, 0, "a copy is a copy");
    }
}
