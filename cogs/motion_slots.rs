//! The few things a cog needs that no schema and no library states for it.
//!
//! A message's fields are read and written through its validated view, and what
//! a pose, a joint vector and a command set mean is `reachy-motion`'s to say, so
//! no mapping layer is needed. This module provides two refusals and a macro.
//!
//! [`joint_set`] is one: a set of servos arriving on an open surface, narrowed
//! once at the boundary by whoever reads the raw bits rather than the validated
//! message.
//!
//! [`configured`] is the other, and it is the whole of what a cog does with a
//! configuration message: one validation, stated once here rather than per cog.
//!
//! The [`counters!`] macro is about a different part of a slot: every cog keeps
//! its run's totals in state fields and reports them on signals of the same
//! names, and a cog writing that bookkeeping out by hand is a cog where a
//! counter can be added to the struct and forgotten in the change guard.
//!
//! Nothing here holds state or allocates, and none of it looks at a clock.

use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointFlagsWire};
use clockwork_rs::{ValidView, validate};
use thiserror::Error;

/// Why a slot's joint vocabulary names no joint.
///
/// A generated schema enum is open — a value the schema does not declare is
/// carried rather than refused, because a publisher can write any bit pattern
/// into a shared slot. Refusing it is therefore this boundary's job, and this
/// is its refusal.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum JointSlotError {
    /// A set with a bit above the ninth bus row set. Refused rather than masked
    /// off: the bit means something to whoever wrote it, and this build does not
    /// know what.
    #[error("{0:#x} is not a set of servos -- it has bits above the ninth bus row")]
    NoSuchJointSet(u16),
}

/// The set of servos those flags name.
///
/// The one place a set arriving from outside is checked. Not
/// [`JointFlagsWire::is_known`]: that answers for the declared bits, so a
/// genuine combination of two servos answers false. The membership question a
/// set actually has is whether every bit set is a bus row, which is what
/// `to_known` asks.
///
/// # Errors
///
/// [`JointSlotError::NoSuchJointSet`] for a value with a bit above the ninth bus
/// row.
pub fn joint_set(flags: JointFlagsWire) -> Result<JointFlags, JointSlotError> {
    flags
        .to_known()
        .ok_or(JointSlotError::NoSuchJointSet(flags.0))
}

/// The configuration `message` states, `whose` naming the cog it configures.
///
/// A cog's one validation of its config message, in one place because every cog
/// answers a config the same way. A refusal stops the run: a config is chosen
/// before the run starts and nothing writes it, so bytes this build cannot read
/// are a scenario built against another schema rather than memory gone wrong.
/// That is not the policy for a state slot, which is reused memory and is
/// counted, cleared and carried on from.
///
/// `whose` is the possessive an operator greps the panic for -- "the mover's",
/// "the plant's".
///
/// # Panics
///
/// If `message` does not read as its schema.
pub fn configured<'a, V: ValidView>(message: &'a V::Raw, whose: &str) -> &'a V {
    validate::<V>(message)
        .unwrap_or_else(|error| panic!("{whose} configuration is unreadable: {error}"))
}

/// Declare a cog's run totals: the struct, the two slot crossings, and the
/// change-guarded report.
///
/// One line per total, naming the field and the setter both the state slot and
/// the signal group spell for it, so a counter is added in one place instead of
/// four. The change guard is the load-bearing part: a total written on every
/// execution would put an observation in the report group at the control rate
/// and roll the group's window in seconds, and a total is an absolute count, so
/// whichever window it lands in carries the whole run.
///
/// The slot and signals types are parameters because they are generated per cog
/// and nothing here can name them. The form without a signals type is for a
/// crate the generated cog crate depends on, which therefore cannot name it: the
/// totals cross the slot here and the report is written where the type is
/// reachable.
///
/// The `crossing` clause names the round-trip case this emits into the calling
/// crate's test build, and is required so that a totals type cannot be declared
/// without one. The case is generated rather than written because the field list
/// is here: a pair declared with each other's setters compiles, and only a
/// distinct value in every field shows what it corrupts -- so the values are
/// counted out over the same repetition that declares the fields, and no caller
/// can hand two fields the same one.
#[macro_export]
macro_rules! counters {
    (
        $(#[$totals_doc:meta])*
        $name:ident of $slot:ty, crossing $crossing:ident {
            $($(#[$field_doc:meta])* $field:ident / $set:ident),+ $(,)?
        }
    ) => {
        $(#[$totals_doc])*
        #[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
        pub struct $name {
            $($(#[$field_doc])* pub $field: u64,)+
        }

        impl $name {
            /// The totals the slot holds.
            #[must_use]
            pub fn read(state: &$slot) -> Self {
                Self { $($field: state.$field(),)+ }
            }

            /// Record them for the next execution.
            pub fn store(&self, state: &mut $slot) {
                $(state.$set(self.$field);)+
            }
        }

        /// Every total crosses the slot as itself: a distinct value in each
        /// field, stored, and read back field for field.
        #[cfg(test)]
        #[test]
        fn $crossing() {
            let mut totals = $name::default();
            let mut nth = 0u64;
            $(
                nth += 1;
                totals.$field = nth;
            )+
            let mut state = <$slot>::new();
            totals.store(&mut state);
            assert_eq!($name::read(&state), totals);
        }
    };

    (
        $(#[$totals_doc:meta])*
        $name:ident of $slot:ty, $signals:ty, crossing $crossing:ident {
            $($(#[$field_doc:meta])* $field:ident / $set:ident),+ $(,)?
        }
    ) => {
        $crate::counters! {
            $(#[$totals_doc])*
            $name of $slot, crossing $crossing {
                $($(#[$field_doc])* $field / $set),+
            }
        }

        impl $name {
            /// Report the ones that moved since `before`.
            pub fn report(&self, before: &Self, signals: &mut $signals) {
                $(
                    if self.$field != before.$field {
                        signals.$set(self.$field);
                    }
                )+
            }
        }
    };
}
