//! The one mapping between the simulated driver's working values and the schema
//! fields it keeps them in.
//!
//! A cog's cross-tick state lives in a declared slot, so the goal gate and the
//! plant have to cross into fixed fields and back on every execution. Both
//! directions are here, once, rather than in the cog body: the cog's own tests
//! and, later, a scenario checker read the same slot, and a second copy of this
//! crossing is a second opinion about which field holds which number.
//!
//! Two things this module refuses to do quietly. A set of servos is read
//! through the motion library's own decoder, so a value with a bit above the
//! ninth bus row is refused rather than masked down to something plausible; the
//! refusal is counted in [`SimSlot::counters`] rather than raised, because this
//! cog is the only writer of the slot and a driver that panicked over its own
//! memory would take the dead-man down with it. And a queue longer than the
//! queue cannot be expressed at all: the slot's queue is a variable-length
//! array, so its length is its own and never a separate number that can
//! disagree with it.
//!
//! Nothing here holds state or allocates, and none of it looks at a clock.

use brenn_reachy__cogs__msgs_clk_rs::{JointFlags, QueuedGoal, SimState};
use clockwork_rs::SyncTime;
use motion_slots::{
    counters, joint_flags, joint_set, joints_from_rows, read_joints, rows_from_joints, write_joints,
};
use reachy_driver::{Goal, GoalGate, JOINT_COUNT, QUEUE_CAP};
use reachy_motion::joints::JointSet;

/// The modelled machine: where the servos are and what is being done to them.
///
/// A set of servos is a [`JointSet`] here and never a bare integer: the bits are
/// the wire's and the gate's business, and a plant that held them would be
/// asking every one of its own features to be written in shifts.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Plant {
    /// Where the servos actually are, radians in bus order.
    pub positions: [f64; JOINT_COUNT],
    /// What each servo is being asked to hold, radians in bus order.
    pub targets: [f64; JOINT_COUNT],
    /// Which servos' targets have ever been written.
    pub has_target: JointSet,
    /// Which servos are energised.
    pub torqued: JointSet,
    /// Which servos are jammed where they stand.
    pub obstructed: JointSet,
    /// How many more cycles of position replies are swallowed.
    pub drop_replies_left: u32,
}

impl Default for Plant {
    fn default() -> Self {
        Self {
            positions: [0.0; JOINT_COUNT],
            targets: [0.0; JOINT_COUNT],
            has_target: JointSet::EMPTY,
            torqued: JointSet::EMPTY,
            obstructed: JointSet::EMPTY,
            drop_replies_left: 0,
        }
    }
}

counters! {
    /// The run's totals, as the slot holds them.
    ///
    /// Every one of these is an absolute count since the process started, so a
    /// report carries the run's number whichever reporting window it lands in.
    ///
    /// Declared without the cog's signals type: that type lives in the generated
    /// crate which depends, through the cog bodies, on this one. So the totals
    /// cross the slot here and the change-guarded report is written in the cog
    /// body, where the type is reachable.
    Counters of SimState, crossing the_run_totals_cross_the_slot {
        /// Goals written to the modelled servos.
        goals_executed / set_goals_executed,
        /// Goals refused because the queue was full.
        goals_dropped / set_goals_dropped,
        /// Times the dead-man latched torque off.
        hold_timeouts / set_hold_timeouts,
        /// Datagrams on the goal channel the codec refused.
        undecodable_goals / set_undecodable_goals,
        /// Events raised on a cycle that had already reported one.
        events_dropped / set_events_dropped,
        /// Fields of the slot that named servos this build does not know.
        refused_state_fields / set_refused_state_fields,
        /// Injections this build could not carry out: an operation it does not
        /// know, or one naming servos it does not know.
        refused_injections / set_refused_injections,
    }
}

/// Everything the simulated driver carries between cycles.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SimSlot {
    /// Whether the first execution has run.
    pub initialized: bool,
    /// The goal gate: the same decision the real driver runs.
    pub gate: GoalGate,
    /// The modelled machine.
    pub plant: Plant,
    /// The run's totals.
    pub counters: Counters,
}

impl Default for SimSlot {
    fn default() -> Self {
        Self {
            initialized: false,
            gate: GoalGate::new(),
            plant: Plant::default(),
            counters: Counters::default(),
        }
    }
}

/// The set of servos those flags name, or none of them.
///
/// The one refusal policy for "a set of servos this build cannot read", used
/// wherever such a value crosses: a refusal is counted on `refusals` rather than
/// returned, because a caller here is the only writer of what it is reading and
/// a value it cannot read is memory nobody wrote. The safe reading of "which
/// servos is this about" is none of them, and the caller says which count the
/// refusal belongs to.
pub fn rows_or_none(flags: JointFlags, refusals: &mut u64) -> JointSet {
    match joint_set(flags) {
        Ok(set) => set,
        Err(_) => {
            *refusals += 1;
            JointSet::EMPTY
        }
    }
}

/// The same, for a mask in the bus-row bits the gate and the wire speak.
pub fn rows_of(bits: u16, refusals: &mut u64) -> JointSet {
    rows_or_none(JointFlags(bits), refusals)
}

/// The goal that queue entry describes.
fn read_goal(slot: &QueuedGoal, refusals: &mut u64) -> Goal {
    Goal {
        execute_at_ns: slot.execute_at().as_nanos(),
        mask: rows_or_none(slot.mask(), refusals).bits(),
        targets: rows_from_joints(&read_joints(slot.targets())),
    }
}

/// Write a goal into a queue entry.
fn write_goal(slot: &mut QueuedGoal, goal: &Goal, refusals: &mut u64) {
    slot.set_execute_at(SyncTime::from_nanos(goal.execute_at_ns));
    slot.set_mask(joint_flags(rows_of(goal.mask, refusals)));
    write_joints(slot.targets_mut(), &joints_from_rows(&goal.targets));
}

/// Read the whole of what the last execution left.
///
/// The queue's length is the stored array's own, which is why a gate read back
/// this way always passes [`GoalGate::validate`]: there is no separate length
/// field for a slot nobody wrote to disagree with.
#[must_use]
pub fn read_sim(state: &SimState) -> SimSlot {
    // The totals as the last execution left them, which reading the rest of the
    // slot can itself add to: a set of servos this build does not know is
    // refused and counted, so what comes back with a gate already includes
    // whatever that reading cost.
    let mut counters = Counters::read(state);
    let refusals = &mut counters.refused_state_fields;
    let mut gate = GoalGate::new();
    for (slot, goal) in gate.queue.iter_mut().zip(state.queue().iter()) {
        *slot = read_goal(goal, refusals);
    }
    gate.queue_len = state.queue().len().min(QUEUE_CAP) as u8;
    gate.held = read_goal(state.held(), refusals);
    gate.has_held = state.has_held();
    gate.latched = state.latched();
    gate.has_accepted = state.has_accepted();
    gate.last_accept_ns = state.last_accept().as_nanos();

    let plant = Plant {
        positions: rows_from_joints(&read_joints(state.positions())),
        targets: rows_from_joints(&read_joints(state.targets())),
        has_target: rows_or_none(state.has_target(), refusals),
        torqued: rows_or_none(state.torqued(), refusals),
        obstructed: rows_or_none(state.obstructed(), refusals),
        drop_replies_left: state.drop_replies_left(),
    };

    SimSlot {
        initialized: state.initialized(),
        gate,
        plant,
        counters,
    }
}

/// Write the whole of what this execution leaves.
///
/// Takes the working values mutably because writing them can itself refuse one:
/// a set of servos that does not cross is counted on the way out exactly as it
/// is on the way in, so `refused_state_fields` covers both directions and the
/// totals written here include whatever this call cost.
pub fn write_sim(state: &mut SimState, sim: &mut SimSlot) {
    state.set_initialized(sim.initialized);

    let queued = sim.gate.queued();
    let mut refusals = 0;
    let mut queue = state.queue_mut();
    queue.clear();
    for goal in queued {
        let entry = queue
            .try_grow()
            .expect("the slot's queue is as deep as the gate's");
        write_goal(entry, goal, &mut refusals);
    }
    write_goal(state.held_mut(), &sim.gate.held, &mut refusals);
    sim.counters.refused_state_fields += refusals;
    state.set_has_held(sim.gate.has_held);
    state.set_latched(sim.gate.latched);
    state.set_has_accepted(sim.gate.has_accepted);
    state.set_last_accept(SyncTime::from_nanos(sim.gate.last_accept_ns));

    write_joints(
        state.positions_mut(),
        &joints_from_rows(&sim.plant.positions),
    );
    write_joints(state.targets_mut(), &joints_from_rows(&sim.plant.targets));
    state.set_has_target(joint_flags(sim.plant.has_target));
    state.set_torqued(joint_flags(sim.plant.torqued));
    state.set_obstructed(joint_flags(sim.plant.obstructed));
    state.set_drop_replies_left(sim.plant.drop_replies_left);

    sim.counters.store(state);
}
