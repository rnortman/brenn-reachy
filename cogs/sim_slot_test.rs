//! What the simulated driver's slot crossing has to get right.
//!
//! The cog's own tests drive this crossing the way the cog does -- write a
//! state, read back the state that was written -- which is the half that
//! works. These are the other half: a slot holding numbers no build of this cog
//! wrote, and a queue crossing with every field made awkward on purpose.

use brenn_reachy__cogs__msgs_clk_rs::{JointFlags, SimState};
use clockwork_rs::SyncTime;
use motion_slots::joint_flags;
use reachy_driver::{Goal, JOINT_COUNT, JOINT_MASK_ALL, QUEUE_CAP};
use reachy_motion::joints::{JointId, JointSet};
use sim_slots::{Counters, SimSlot, read_sim, write_sim};

/// The instant the goals in these cases are stamped around. Off the twenty
/// millisecond grid, so an instant that travelled through a rounding step shows
/// up as a different number.
const T0: i64 = 1_700_000_000_123_456_789;

/// A set of servos with a tenth bus row in it, which no machine has.
const NO_SUCH_SET: JointFlags = JointFlags(1 << JOINT_COUNT);

/// A gate and a plant with nothing round about them, so a crossing that
/// transposed two fields shows as a value mismatch rather than a coincidence of
/// zeroes.
fn awkward_slot() -> SimSlot {
    let mut sim = SimSlot {
        initialized: true,
        ..SimSlot::default()
    };
    for row in 0..QUEUE_CAP {
        let goal = awkward_goal(row);
        assert_eq!(
            sim.gate.accept(goal, T0),
            reachy_driver::AcceptOutcome::Accepted,
            "the fixture's own queue",
        );
    }
    sim.gate.held = awkward_goal(9);
    sim.gate.has_held = true;
    sim.gate.latched = true;
    sim.gate.has_accepted = true;
    sim.gate.last_accept_ns = T0 - 7;

    sim.plant.positions = core::array::from_fn(|row| 0.011 * (row as f64 + 1.0));
    sim.plant.targets = core::array::from_fn(|row| -0.023 * (row as f64 + 1.0));
    sim.plant.has_target = set_of(&[JointId::BodyYaw, JointId::Leg(4), JointId::AntennaLeft]);
    sim.plant.torqued = set_of(&[JointId::Leg(0), JointId::Leg(5)]);
    sim.plant.obstructed = set_of(&[JointId::AntennaRight]);
    sim.plant.drop_replies_left = 17;

    sim.counters = Counters {
        goals_executed: 101,
        goals_dropped: 3,
        hold_timeouts: 2,
        undecodable_goals: 5,
        events_dropped: 7,
        refused_state_fields: 11,
        refused_injections: 13,
    };
    sim
}

/// A goal no two of which share a number, mask included.
fn awkward_goal(nth: usize) -> Goal {
    Goal {
        execute_at_ns: T0 + 1_000_003 * (nth as i64 + 1),
        // A mask with holes in it, different per goal, and never every row.
        mask: JOINT_MASK_ALL ^ (1 << (nth % JOINT_COUNT)),
        targets: core::array::from_fn(|row| (nth as f64) + 0.001 * (row as f64 + 1.0)),
    }
}

#[test]
fn a_gate_and_a_plant_cross_the_slot_field_for_field() {
    let sim = awkward_slot();
    let mut written = sim;
    let mut state = SimState::new();
    write_sim(&mut state, &mut written);

    assert_eq!(
        written.counters, sim.counters,
        "a state this build can express costs no refusals on the way out"
    );
    let back = read_sim(&state);
    assert_eq!(back, sim, "everything, in the fields it went out through");
    assert_eq!(
        back.gate.queued().len(),
        QUEUE_CAP,
        "a full queue crosses full"
    );
}

/// The queue's length is the stored array's own, so a gate read out of a slot
/// cannot claim more goals than it holds -- the one thing
/// [`reachy_driver::GoalGate::validate`] refuses.
#[test]
fn a_gate_read_out_of_a_slot_is_always_a_gate() {
    assert_eq!(
        read_sim(&SimState::new()),
        SimSlot::default(),
        "a slot nobody wrote is the state a cog starts from"
    );
    assert_eq!(read_sim(&SimState::new()).gate.validate(), Ok(()));

    let mut state = SimState::new();
    for len in 0..=QUEUE_CAP {
        let mut sim = SimSlot::default();
        for nth in 0..len {
            sim.gate.accept(awkward_goal(nth), T0);
        }
        write_sim(&mut state, &mut sim);
        let back = read_sim(&state);
        assert_eq!(back.gate.validate(), Ok(()), "{len} queued");
        assert_eq!(back.gate.queued(), sim.gate.queued(), "{len} queued");
    }
}

/// A field naming servos this build does not know is refused rather than masked
/// down to the rows it does know, and each refusal is counted where a scenario
/// checker can see it. A build reading a slot an older or newer one wrote is
/// what this is for, and a checker reading zero refusals is trusting that.
#[test]
fn a_field_naming_servos_this_build_does_not_know_is_refused_and_counted() {
    /// What a build that disagreed about the servos would have left in a field.
    type Damage = fn(&mut SimState);

    let cases: [(&str, Damage); 5] = [
        ("torqued", |state| state.set_torqued(NO_SUCH_SET)),
        ("has_target", |state| state.set_has_target(NO_SUCH_SET)),
        ("obstructed", |state| state.set_obstructed(NO_SUCH_SET)),
        ("held.mask", |state| state.held_mut().set_mask(NO_SUCH_SET)),
        ("a queued goal's mask", |state| {
            state
                .queue_mut()
                .get_mut(0)
                .expect("the fixture's queue is full")
                .set_mask(NO_SUCH_SET);
        }),
    ];

    for (what, damage) in cases {
        let mut sim = awkward_slot();
        let mut state = SimState::new();
        write_sim(&mut state, &mut sim);
        let before = read_sim(&state);
        damage(&mut state);
        let after = read_sim(&state);

        assert_eq!(
            after.counters.refused_state_fields,
            before.counters.refused_state_fields + 1,
            "{what}: one field, one refusal",
        );
        let rows = match what {
            "torqued" => after.plant.torqued,
            "has_target" => after.plant.has_target,
            "obstructed" => after.plant.obstructed,
            "held.mask" => JointSet::from_bits(after.gate.held.mask).expect("nine rows at most"),
            _ => JointSet::from_bits(after.gate.queued()[0].mask).expect("nine rows at most"),
        };
        assert_eq!(
            rows,
            JointSet::EMPTY,
            "{what}: read as no servos rather than as the nine it could make out",
        );
    }
}

/// The same on the way out: a set of servos this build cannot write is counted
/// too, so the total covers both directions and a checker reading it is reading
/// the whole story.
#[test]
fn a_set_of_servos_that_cannot_be_written_is_counted_on_the_way_out() {
    let mut sim = SimSlot::default();
    sim.gate.held = Goal {
        mask: NO_SUCH_SET.0,
        ..awkward_goal(0)
    };
    sim.gate.has_held = true;
    sim.gate.accept(
        Goal {
            mask: NO_SUCH_SET.0,
            ..awkward_goal(1)
        },
        T0,
    );

    let mut state = SimState::new();
    write_sim(&mut state, &mut sim);
    assert_eq!(
        sim.counters.refused_state_fields, 2,
        "the held setpoint and the queued goal, one each"
    );
    assert_eq!(
        state.held().mask(),
        joint_flags(JointSet::EMPTY),
        "and no servo was named by a number nothing could read"
    );
}

/// Times cross as the instants they are, not as a count of something.
#[test]
fn the_instants_in_a_slot_are_the_instants_that_went_in() {
    let sim = awkward_slot();
    let mut written = sim;
    let mut state = SimState::new();
    write_sim(&mut state, &mut written);

    assert_eq!(state.last_accept(), SyncTime::from_nanos(T0 - 7));
    assert_eq!(
        state.held().execute_at(),
        SyncTime::from_nanos(sim.gate.held.execute_at_ns)
    );
    for (nth, entry) in state.queue().iter().enumerate() {
        assert_eq!(
            entry.execute_at(),
            SyncTime::from_nanos(awkward_goal(nth).execute_at_ns),
            "goal {nth}",
        );
    }
}

/// A set holding those joints.
fn set_of(joints: &[JointId]) -> JointSet {
    let mut set = JointSet::EMPTY;
    for joint in joints {
        set.insert(*joint);
    }
    set
}
