//! What the joint vocabulary has to get right on the way in from a slot.
//!
//! The `*Wire` spellings below are also the tree's generator naming policy in
//! force: no build file names that policy any more, so hand-written Rust naming
//! a type the other policy would not emit is what fails if it changes.

use brenn_reachy__motion__faults_clk_rs::FaultKindWire;
use brenn_reachy__motion__joints_clk_rs::{JointFlagsWire, JointRefWire};
use brenn_reachy__motion__tick_state_clk_rs::{FaultSnap, FaultSnapWire};
use motion_slots::{JointSlotError, configured, joint_set};
use reachy_motion::joints::{JointRef, ROWS, row};

/// A servo's bus row and the vocabulary's number for it are one apart, because
/// a Clockwork default carries zero and "no joint" is what a default must mean.
/// That offset is exactly the kind of thing a comment cannot hold still, so
/// this holds it.
#[test]
fn every_servo_is_one_past_its_bus_row() {
    for joint in ROWS {
        let row = row(joint).expect("every servo has a bus row");
        let row = u8::try_from(row).expect("nine rows");
        let reference = JointRefWire::from(joint);
        assert_eq!(
            u32::from(reference.0),
            u32::from(row) + 1,
            "{joint:?} sits one past its bus row"
        );
        assert_eq!(
            reference.to_known(),
            Some(joint),
            "{joint:?} narrows back to itself"
        );
        assert!(reference.is_known(), "{joint:?} is a declared value");
    }
}
/// Naming no servo is a value of the vocabulary rather than a gap in it: the
/// zero a slot nobody wrote holds is the value that says "no joint", which is
/// why a Clockwork default can carry it.
#[test]
fn naming_no_servo_is_a_value_and_not_a_gap() {
    assert_eq!(JointRefWire::NONE.0, 0, "the default carries zero");
    assert_eq!(JointRefWire::NONE.to_known(), Some(JointRef::None));
    assert!(
        row(JointRef::None).is_none(),
        "and it occupies no bus row on the library's side either"
    );
}

/// A generated enum is open: a publisher may write any number into the slot, and
/// the newtype carries it rather than refusing it. Refusing it is the
/// validation's job, and a message holding an undeclared number for a servo has
/// no validated surface at all.
#[test]
fn a_number_naming_no_servo_is_refused_at_the_boundary() {
    for value in u8::MIN..=u8::MAX {
        let declared = value <= 9;
        assert_eq!(
            JointRefWire(value).is_known(),
            declared,
            "{value} against the declared vocabulary"
        );

        let mut slot = FaultSnapWire::new();
        slot.set_code(FaultKindWire::HEAD_OBSTRUCTED);
        slot.set_joint(JointRefWire(value));
        assert_eq!(
            slot.validate().is_ok(),
            declared,
            "{value} narrowed at the boundary"
        );
    }
}

/// Every value the field can hold, and what the boundary makes of it: the 512
/// sets of nine servos cross whole, and everything else is refused rather than
/// masked down to something plausible.
#[test]
fn a_set_of_servos_crosses_whole_and_nothing_else_crosses() {
    for value in u16::MIN..=u16::MAX {
        let flags = JointFlagsWire(value);
        match joint_set(flags) {
            Ok(set) => {
                assert!(value < 512, "{value} is a set of the nine bus rows");
                assert_eq!(JointFlagsWire::from(set), flags, "carried, not repaired");
            }
            Err(err) => {
                assert!(value >= 512, "{value} names servos this machine has");
                assert_eq!(err, JointSlotError::NoSuchJointSet(value));
            }
        }
    }
}

/// A configuration this build cannot read stops the run, rather than being
/// counted and cleared the way a state slot is. The possessive is the string an
/// operator greps the panic for, so it is part of what is asserted.
///
/// Driven with a fault snapshot rather than with either cog's own configuration
/// message: a config schema has to carry an enum or a counted container for
/// validation to have anything to refuse, and neither of the two does today.
#[test]
#[should_panic(expected = "the plant's configuration is unreadable")]
fn a_configuration_this_build_cannot_read_stops_the_run() {
    let mut message = FaultSnapWire::new();
    message.set_joint(JointRefWire(u8::MAX));
    assert!(message.validate().is_err(), "the case rests on this");
    let _: &FaultSnap = configured(&message, "the plant's");
}

/// The three non-fault outcomes share the fault channel's enum, and nothing
/// publishes them yet -- which is why their numbers are cheap to state now and
/// expensive to discover later, from a checker asserting on a number the
/// emitter never produced.
///
/// The numbers are the ones the schema compiler emits, which are the
/// declaration order rather than the `#N` tags. They are written out here as
/// literals so that a value inserted among the faults above turns this red
/// instead of quietly moving the whole family.
#[test]
fn the_abort_family_has_the_numbers_that_follow_the_faults() {
    assert_eq!(FaultKindWire::MOVE_ABORTED_ENVELOPE.0, 9);
    assert_eq!(FaultKindWire::MOVE_ABORTED_STEP.0, 10);
    assert_eq!(FaultKindWire::COMMAND_REJECTED.0, 11);

    assert_eq!(FaultKindWire::NONE.0, 0, "no report is no number");
}
