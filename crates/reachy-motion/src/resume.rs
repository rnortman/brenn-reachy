//! The bounds and refusals a sequencer's state is checked against when it is
//! picked up again.
//!
//! A sequencer is a state machine that hands out one transaction at a time and
//! is stepped again when the answer comes back. Its state is the schema its host
//! hands it: a slot, validated once at the boundary, written through the
//! validated view, and found again exactly where the last execution left it.
//! There is no second representation, so there is nothing here to carry — what
//! is here is the reading of a slot's numbers that every `resume` makes before
//! it agrees to run.
//!
//! # What a state does not carry
//!
//! The configuration. [`ArmConfig`](crate::arm::ArmConfig),
//! [`HeadGeometry`](reachy_kin::HeadGeometry),
//! [`FkOptions`](reachy_kin::FkOptions) and
//! [`DisarmConfig`](crate::disarm::DisarmConfig) are what the host was
//! configured with, not what the sequence found out, and a host of the shape
//! this exists for is handed them read-only on every execution anyway. Carrying
//! them would put a second copy of the fences in the state slot, where it could
//! disagree with the configured one. So every `resume` takes them alongside the
//! state.
//!
//! # The verdict
//!
//! A stopped sequence's failure is [`SeqError`], one shape per way a sequence
//! can stop, each carrying its own evidence. Which numbers hold it is
//! [`crate::verdict`]'s business, over the schema the slot declares; what this
//! module says about it is only whether a phase may hold one at all.
//!
//! # Which pairings are refused
//!
//! A phase plus a cursor is a wider vocabulary than the phases: a cursor past
//! the sweep it indexes, a failed phase with no failure in it, a finished engage
//! with no record, a field one phase owns left behind in another. Each is a state
//! no sequence of steps produces, and each is refused by [`ResumeError`] rather
//! than run — the case that needs refusing is a slot holding bytes nothing here
//! wrote. A pairing that *is* reachable is never refused, including the ones that
//! look odd: a running phase with nothing outstanding is the step after a wait,
//! and a terminal phase reached on the very first call is a sequence that
//! refused before it transacted.

use thiserror::Error;

use crate::joints::ROW_COUNT;

/// The scaffolding every schema-resident sequencer carries, written once.
///
/// What differs between the sequencers is their phases: what each step asks for
/// and what an answer to it means. What does not differ is the handful of
/// operations over the three fields every one of their states declares — where
/// the sweep stands, entering a phase, and the verdict a stop is filed as — and
/// the invariants those operations hold: a phase is entered at the start of its
/// sweep with the fields of the phase being left blanked, and a failure leaves
/// nothing outstanding and files its verdict where the next execution reads it.
/// A copy of that quietly missing a line is the failure mode this exists to
/// prevent, so there is one copy.
///
/// Invoked without a failed phase for a sequence that has none — the release
/// refuses nothing, so it takes the sweep and the phase door and no verdict.
///
/// Everything the expansion reaches for is named through `$crate`, so an invoker
/// imports nothing on the macro's behalf.
macro_rules! phase_state {
    ($seq:ident, $phase:ty, $failed:expr) => {
        phase_state!($seq, $phase);

        impl $seq<'_> {
            /// The verdict the failed phase stopped on.
            ///
            /// Read off the state rather than carried beside it: the slot is
            /// where the failure lives, and a copy in the sequencer would be the
            /// second representation this design deletes. A verdict that does
            /// not read back is reported as exactly that — [`Self::resume`]
            /// refuses such a slot, and a verdict this sequence wrote reads, so
            /// what remains is a slot written by something else between two
            /// steps.
            fn verdict(&self) -> $crate::seq::SeqError {
                $crate::verdict::read(&self.state.failure).unwrap_or(
                    $crate::seq::SeqError::VerdictUnreadable {
                        context: $crate::seq::StepContext::servo(
                            self.state.failure.step,
                            self.state.failure.servo_id,
                        ),
                    },
                )
            }

            /// Stop the sequence on `error`, answering the error to report it
            /// under.
            ///
            /// Nothing is outstanding once a sequence has stopped, and the
            /// verdict is written where the next execution reads it from. A
            /// verdict that will not cross leaves the fields blank, which the
            /// next resume refuses as a failed phase with no failure rather than
            /// reporting a failure nobody observed. The error is answered rather
            /// than left to be read back out of the slot, so the one failure
            /// whose evidence does not cross is still reported as itself.
            fn fail(&mut self, error: $crate::seq::SeqError) -> $crate::seq::SeqError {
                self.enter($failed);
                $crate::txn::set_none(&mut self.state.pending);
                let _ = $crate::verdict::write(&mut self.state.failure, &error);
                error
            }
        }
    };
    ($seq:ident, $phase:ty) => {
        impl $seq<'_> {
            /// Where the running phase's sweep stands.
            fn cursor(&self) -> usize {
                $crate::resume::cursor_usize(self.state.cursor)
            }

            /// Move the running phase's sweep on to `cursor`.
            fn seek(&mut self, cursor: usize) {
                self.state.cursor = $crate::resume::cursor_u32(cursor);
            }

            /// Enter `phase` at the start of its sweep.
            ///
            /// The fields the phase being left owns are blanked on the way out,
            /// so a state's fields describe the phase it is in and no earlier
            /// one — which is the invariant [`Self::resume`] refuses a slot for.
            fn enter(&mut self, phase: $phase) {
                self.blank_phase_fields();
                self.state.phase = phase;
                self.state.cursor = 0;
            }
        }
    };
}

/// How many cells the provisioning sweep walks: one per servo per provisioned
/// register.
///
/// Stated here as the bound a cursor is checked against; the register list
/// itself is [`crate::arm::PROVISION_REGS`].
pub const PROVISION_CELLS: usize = crate::arm::PROVISION_REGS.len() * ROW_COUNT;

/// How many writes the gains-and-profiles sweep makes: one position-gains write
/// per servo, then one write per servo per entry in the sweep's register table.
///
/// Derived from the sweep's own table rather than restated, so an entry added
/// there widens this bound with it.
pub const GAINS_PROFILE_WRITES: usize = (1 + crate::arm::PROFILE_REGS.len()) * ROW_COUNT;

/// A snapshot whose numbers name a state no sequence of steps reaches.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum ResumeError {
    /// A cursor stands at or past the end of the sweep its phase indexes. Every
    /// sweep hands over to the next phase on its last element rather than
    /// counting past it, so a cursor at the bound indexes nothing.
    #[error("a {phase} cursor of {cursor} indexes past the sweep's {bound} elements")]
    CursorOutOfRange {
        /// Which phase the cursor belongs to.
        phase: &'static str,
        /// What the snapshot held.
        cursor: u32,
        /// How many elements that phase's sweep has.
        bound: u32,
    },
    /// A non-zero cursor in a phase that walks no sweep. The endings and the
    /// dwell index nothing, so the only cursor they could have been left at is
    /// zero — and a refusal here is read by whoever is looking at a corrupt slot
    /// on a machine that has just stopped commanding, so it says that rather
    /// than describing a one-element sweep that does not exist.
    #[error("the {phase} phase walks no sweep, so a cursor of {cursor} indexes nothing")]
    CursorInPhaseWithNoSweep {
        /// Which phase the cursor belongs to.
        phase: &'static str,
        /// What the snapshot held.
        cursor: u32,
    },
    /// A slot whose phase names no phase at all, which is what an unwritten slot
    /// holds. Refused rather than read as the first sweep: a commission restored
    /// at presence when it had already written gains would write them twice.
    #[error("the {phase} phase is no phase, which is what a slot nothing wrote holds")]
    NoPhase {
        /// The name the refusal reads under.
        phase: &'static str,
    },
    /// A release whose form names neither of the two releases, which is what a
    /// slot nothing wrote holds. The form is what says whether the measurement
    /// fields describe a failed look or no look at all, so a restore does not
    /// guess it: a release that never measured anything would otherwise be
    /// filed as one that looked and found the head at stow.
    #[error("the slot runs neither release this build performs")]
    NoReleaseForm,
    /// A verdict that does not read as a failure, in a phase that says the
    /// sequence stopped on one.
    #[error("the failed phase's verdict does not read: {0}")]
    Verdict(#[from] crate::verdict::VerdictError),
    /// A failure recorded in a phase that is still running. Nothing carries a
    /// failure forward: a sequencer that produces one lands in its failed
    /// phase in the same statement.
    #[error("the snapshot records a failure while still in the {phase} phase")]
    ErrorWithoutFailedPhase {
        /// The phase the snapshot claims to be in.
        phase: &'static str,
    },
    /// A field belonging to one phase, set while another phase runs. The supply
    /// gate's clock and the dwell's flag are written by their own phase and
    /// zeroed everywhere else, so a slot carrying one outside it was written by
    /// something that does not agree with this type about what a phase holds —
    /// and restoring it silently would drop the evidence along with the value.
    #[error(
        "the snapshot carries {field}, which only the {owner} phase writes, while in the {phase} phase"
    )]
    StrayPhaseField {
        /// The field, as its snapshot member is spelled.
        field: &'static str,
        /// The phase that field belongs to.
        owner: &'static str,
        /// The phase the snapshot claims to be in.
        phase: &'static str,
    },
    /// A finished engage with no armed record in it. The record is what the
    /// summary is built from and what the first trajectory starts from.
    #[error("the snapshot is a finished engage with no armed record")]
    CompleteWithoutRecord,
    /// An armed record recorded before the engage finished. It is written by
    /// the settle sweep's last read, in the same statement that completes the
    /// phase.
    #[error("the snapshot carries an armed record while still in the {phase} phase")]
    RecordWithoutCompletePhase {
        /// The phase the snapshot claims to be in.
        phase: &'static str,
    },
    /// A record a resumed sequence plans its remaining writes from, in a phase
    /// that only follows the step which writes it.
    #[error("the snapshot's {record} record is absent in the {phase} phase, which follows it")]
    RecordMissing {
        /// Which of the state's records, as its field is spelled.
        record: &'static str,
        /// The phase the snapshot claims to be in.
        phase: &'static str,
    },
    /// A record whose quaternion is no rotation, which is what a slot written by
    /// something else holds.
    #[error("the snapshot's {record} record is no pose: {source}")]
    RecordNotAPose {
        /// Which of the state's records, as its field is spelled.
        record: &'static str,
        /// What the pose read refused it as.
        source: crate::snap::PoseSnapshotError,
    },
    /// A number a resumed sequence plans from that is not a number. Generated
    /// validation covers enums, counts and strings and never a float, so a
    /// structurally valid slot carrying a NaN reads as a state and behaves as a
    /// wedge — or, on the command path, as a write nobody can place.
    #[error("the snapshot's {record} {field} is not a number")]
    NonFinite {
        /// Which part of the state holds it, as its field is spelled.
        record: &'static str,
        /// The number, as its field is spelled.
        field: &'static str,
    },
    /// A pinned goal outside the travel window its leg is placed in. Placement
    /// only ever produces an in-window pin, so a stored one outside is a slot
    /// written by something else — and it is refused rather than placed again,
    /// because pulling a commanded value to the window edge here would be a
    /// clamp on the torque path.
    #[error("the pinned goal {pin} for leg {leg} is outside its window {low}..={high}")]
    PinOutOfWindow {
        /// Which crank, in leg order.
        leg: usize,
        /// The goal the slot holds, radians.
        pin: f64,
        /// The bottom of that leg's travel window, radians.
        low: f64,
        /// The top of it, radians.
        high: f64,
    },
    /// A pinned antenna goal outside the span extended position mode's goal
    /// register represents. A stored pin out there prefigures a goal write no
    /// count can carry, which the verified engage sweep refuses anyway, so the
    /// refusal only moves that failure ahead of torque-on.
    #[error("the pinned goal {pin} for the {joint} antenna is outside the span {low}..={high}")]
    AntennaPinNoCount {
        /// Which antenna, as an operator names it.
        joint: &'static str,
        /// The goal the slot holds, radians.
        pin: f64,
        /// The bottom of the representable span, radians.
        low: f64,
        /// The top of it, radians.
        high: f64,
    },
    /// A pinned body-yaw goal no count of the single-turn goal register holds.
    /// The yaw servo's mode is a checked provisioning expectation, so the one
    /// turn is the set its goal register takes; a stored pin outside it — a
    /// multi-turn reading off a body a hand turned while it was limp — is a
    /// write that servo refuses, moved ahead of torque-on.
    #[error(
        "the pinned body yaw goal {pin} rounds to count {counts}, which the one-turn goal register does not hold (0..={bound})"
    )]
    YawPinNoCount {
        /// The goal the slot holds, radians.
        pin: f64,
        /// The count it rounds to.
        counts: f64,
        /// The highest count that register holds.
        bound: f64,
    },
}

/// A cursor within the sweep its phase indexes, as a `usize`.
///
/// One check for all four sequencers, over the two ways a cursor can be wrong: a
/// phase that walks no sweep carrying one at all, and a cursor at or past the end
/// of the sweep it does index. Every sweep hands over to the next phase on its
/// last element, so the bound is exclusive and a cursor at it indexes nothing.
pub(crate) fn checked_cursor(
    phase: &'static str,
    sweep: Option<usize>,
    cursor: u32,
) -> Result<usize, ResumeError> {
    let Some(bound) = sweep else {
        if cursor != 0 {
            return Err(ResumeError::CursorInPhaseWithNoSweep { phase, cursor });
        }
        return Ok(0);
    };
    let out_of_range = ResumeError::CursorOutOfRange {
        phase,
        cursor,
        bound: cursor_u32(bound),
    };
    let index = usize::try_from(cursor).map_err(|_| out_of_range)?;
    if index >= bound {
        return Err(out_of_range);
    }
    Ok(index)
}

/// A sweep cursor or bound as a `u32`, for the slot field that holds it.
///
/// # Panics
///
/// If it does not fit. Every sweep in this crate is nine servos, the provisioning
/// grid or the gains-and-profiles writes, so nothing here comes near the bound;
/// a truncation would mint a number `from_snapshot` then refuses, turning a
/// broken invariant into a corrupt-slot report somewhere else entirely.
pub(crate) fn cursor_u32(cursor: usize) -> u32 {
    u32::try_from(cursor).expect("a sweep cursor fits a u32")
}

/// A sweep cursor as the `usize` the sweeps are indexed by.
///
/// # Panics
///
/// If it does not fit, which needs a `usize` narrower than 32 bits; every target
/// this repo builds for is 64-bit. The alternative — an index this refused — would
/// report a corrupt slot for a cursor the sweep legitimately reached.
pub(crate) fn cursor_usize(cursor: u32) -> usize {
    usize::try_from(cursor).expect("a sweep cursor fits a usize")
}

/// Whether a state's phase names a phase at all.
pub(crate) fn no_phase(phase: &'static str, nameless: bool) -> Result<(), ResumeError> {
    if nameless {
        return Err(ResumeError::NoPhase { phase });
    }
    Ok(())
}

/// Whether a field one phase owns is unset, for a snapshot that is in another
/// phase.
///
/// `set` is the caller's reading of the field, because what "unset" means is the
/// field's own business: a zero duration for the supply gate's clock, a false
/// flag for the two waits.
pub(crate) fn no_stray_field(
    field: &'static str,
    owner: &'static str,
    phase: &'static str,
    set: bool,
) -> Result<(), ResumeError> {
    if set {
        return Err(ResumeError::StrayPhaseField {
            field,
            owner,
            phase,
        });
    }
    Ok(())
}

/// Whether a state's failure sits in a phase that can hold one, for a phase that
/// is not the failed one.
///
/// `recorded` is the caller's reading of its own failure field: a `Some` in a
/// snapshot, a kind that is not the no-failure zero in a state slot.
pub(crate) fn no_stray_failure(phase: &'static str, recorded: bool) -> Result<(), ResumeError> {
    if recorded {
        return Err(ResumeError::ErrorWithoutFailedPhase { phase });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The sweep bounds are the sweeps the sequencers actually walk.
    #[test]
    fn the_sweep_bounds_are_the_transaction_counts() {
        assert_eq!(
            PROVISION_CELLS,
            crate::arm::PROVISION_REGS.len() * ROW_COUNT
        );
        assert_eq!(GAINS_PROFILE_WRITES, 45);
    }

    /// A cursor at the end of its sweep indexes nothing: the last element hands
    /// over rather than counting past.
    #[test]
    fn a_cursor_at_the_bound_is_refused() {
        assert_eq!(
            checked_cursor("presence", Some(9), 9),
            Err(ResumeError::CursorOutOfRange {
                phase: "presence",
                cursor: 9,
                bound: 9,
            })
        );
        assert_eq!(checked_cursor("presence", Some(9), 8), Ok(8));
    }

    /// A phase with no sweep carries no cursor, so only zero is a cursor it
    /// could have been left at — and the refusal says that rather than naming a
    /// sweep the phase does not have.
    #[test]
    fn a_phase_with_no_sweep_takes_only_a_zero_cursor() {
        assert_eq!(checked_cursor("complete", None, 0), Ok(0));
        assert_eq!(
            checked_cursor("complete", None, 1),
            Err(ResumeError::CursorInPhaseWithNoSweep {
                phase: "complete",
                cursor: 1,
            })
        );
        assert_eq!(
            checked_cursor("complete", None, 1).unwrap_err().to_string(),
            "the complete phase walks no sweep, so a cursor of 1 indexes nothing"
        );
    }

    #[test]
    fn a_failure_in_a_running_phase_is_refused() {
        assert_eq!(
            no_stray_failure("presence", true),
            Err(ResumeError::ErrorWithoutFailedPhase { phase: "presence" })
        );
        assert_eq!(no_stray_failure("presence", false), Ok(()));
    }
}
