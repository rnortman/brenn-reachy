//! The wind-down: what a host does with a machine that has to come down.
//!
//! One decision core for the two controlled responses of the fault doctrine —
//! the stow to rest and the masked stow to park. It owns the whole of the
//! judgement: the single clock the maneuver is bounded by, the escalation a
//! servo dropping out mid-stow causes, the outcome the record gets, and where
//! the machine is left. What it never owns is the doing: it names an action and
//! the host carries it out, so a blocking bench loop and a cog woken every
//! hundred milliseconds run the identical maneuver rather than two maneuvers
//! that agree today.
//!
//! Shaped like the tick and the sequencers: time arrives as a parameter, the
//! evidence of what the machine did arrives as a parameter, and the state is
//! the schema the host persists in its slot — `WindDown` borrows it for one
//! step rather than copying it out.
//!
//! Every path ends in [`WindDownAction::Conclude`], and concluding always
//! de-torques. Nothing here can refuse or delay that, and nothing a caller
//! supplies — evidence or a fault raised mid-maneuver — makes this core ask for
//! torque to be held: a machine that defeated the stow, one whose clock ran out,
//! one with no head joint left to drive, and one whose ending has escalated to
//! the immediate torque-off all get the same immediate release, differing only
//! in what the record says about how far the maneuver got.

use core::fmt;
use std::time::Duration;

use brenn_reachy__motion__timeline_clk_rs::{WindDownSnap, WindDownSnapWire};
use clockwork_rs::SyncTime;
use thiserror::Error;

use crate::snap::{DurationError, duration_from_nanos, duration_nanos};
use crate::tick::{Fault, ResponseKind};
use crate::traj::MoveDurations;

/// What a response actually does to the machine.
///
/// The doctrine's minimum-risk maneuvers, one slug each. A response is a
/// maneuver plus the state it leaves behind ([`ResponseKind`]); this is the
/// maneuver half — the part an operator watches happen.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Maneuver {
    /// Abandon the move and stow every commanded joint under control, checks
    /// live.
    SlowStow,
    /// Torque off the servo that dropped out, then stow on what still
    /// commands. A further servo dropping out expands it rather than ending
    /// it.
    MaskedSlowStow,
    /// Torque off the antenna pair and stop commanding it. The head is
    /// untouched and the move carries on.
    AntennaTorqueOff,
    /// Immediate best-effort torque-off of all nine.
    ImmediateAllTorqueOff,
}

impl Maneuver {
    /// The maneuver's slug — the name it is reported under everywhere.
    #[must_use]
    pub fn slug(self) -> &'static str {
        match self {
            Self::SlowStow => "slow_stow",
            Self::MaskedSlowStow => "masked_slow_stow",
            Self::AntennaTorqueOff => "antenna_torque_off",
            Self::ImmediateAllTorqueOff => "immediate_all_torque_off",
        }
    }
}

impl fmt::Display for Maneuver {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.slug())
    }
}

/// The maneuver `response` runs, or `None` where it runs none.
///
/// The response half of the same table [`ending::maneuver`] answers for an
/// ending, and the two differ in exactly one place: degrading a pair is not an
/// ending, so an ending that names that response answers with the stow, while
/// the response itself runs the antenna torque-off. A host carrying out a
/// response asks here — which is what keeps the one group-scoped maneuver
/// classified beside the rest of them rather than spelled out by whoever runs
/// it.
///
/// Total, wildcard-free: a response added to the doctrine is a decision made
/// here at compile time.
#[must_use]
pub fn maneuver_of(response: ResponseKind) -> Option<Maneuver> {
    match response {
        ResponseKind::SlowStowToRest => Some(Maneuver::SlowStow),
        ResponseKind::MaskedSlowStowToPark => Some(Maneuver::MaskedSlowStow),
        ResponseKind::DegradeAntennas => Some(Maneuver::AntennaTorqueOff),
        ResponseKind::ImmediateAllTorqueOffToRest | ResponseKind::ImmediateAllTorqueOffToPark => {
            Some(Maneuver::ImmediateAllTorqueOff)
        }
        // A refusal does nothing to the machine -- an ask was declined -- and no
        // answer runs no maneuver.
        ResponseKind::Refuse | ResponseKind::None => None,
    }
}

/// What a run's ending asks its caller to do.
///
/// The response vocabulary of the fault doctrine, less the one response that is
/// not an ending: degrading the antennas leaves the move running, and travels
/// in the tick's report rather than as an ending.
///
/// The vocabulary's own enum, declared in `motion/faults.clk`: the variants
/// this core matches on and the numbers a slot holds an ending in are one
/// thing. [`EndingKind::None`] is what a slot running no maneuver holds — every
/// function in [`ending`] says what it means, and neither opening a maneuver nor
/// restoring one carries it: both boundaries refuse it, so no maneuver this core
/// runs is judged by no ending.
pub use brenn_reachy__motion__faults_clk_rs::EndingKind;

/// What an ending asks of the machine, and of whoever finds it.
///
/// Free functions rather than inherent methods, because the type belongs to the
/// vocabulary crate. Every question about what an ending does is answered here
/// and nowhere else: a caller spelling one out is a second table that can come
/// to disagree with this one, and what these decide is where a machine is left
/// after a fault.
pub mod ending {
    use super::{Disposition, EndingKind, Maneuver, ResponseKind};

    /// The ending that carries out `response`.
    ///
    /// Every ending a host can name maps onto one of these, so what a caller
    /// does about an ending is decided once, here, at compile time — never by a
    /// caller reading a message, and never by a default that a new variant
    /// falls into.
    #[must_use]
    pub fn answering(response: ResponseKind) -> EndingKind {
        match response {
            ResponseKind::Refuse => EndingKind::Refuse,
            ResponseKind::SlowStowToRest => EndingKind::SlowStowToRest,
            ResponseKind::ImmediateAllTorqueOffToRest => EndingKind::ImmediateAllTorqueOffToRest,
            ResponseKind::MaskedSlowStowToPark => EndingKind::MaskedSlowStowToPark,
            ResponseKind::ImmediateAllTorqueOffToPark => EndingKind::ImmediateAllTorqueOffToPark,
            // Degrading a pair is not an ending: the antennas go limp and the
            // move carries on, reported through the tick. An ending that
            // nonetheless names this response says the head is healthy and
            // still commanding, and a stow under control is what that gets.
            ResponseKind::DegradeAntennas => EndingKind::SlowStowToRest,
            // Nothing was answered, so there is nothing to end. Never produced
            // by a classification — only by a slot that has answered nothing —
            // and it stays no ending rather than becoming the mildest one.
            ResponseKind::None => EndingKind::None,
        }
    }

    /// What this ending actually does to the machine, for the record it is
    /// reported in.
    ///
    /// Beside [`disposition`] and for the same reason: which maneuver an ending
    /// runs is the ending's own fact. A refusal runs none — nothing is done to a
    /// machine whose ask was declined — and no ending runs none for the same
    /// reason.
    #[must_use]
    pub fn maneuver(ending: EndingKind) -> Option<Maneuver> {
        match ending {
            EndingKind::None | EndingKind::Refuse => None,
            EndingKind::SlowStowToRest => Some(Maneuver::SlowStow),
            EndingKind::MaskedSlowStowToPark => Some(Maneuver::MaskedSlowStow),
            EndingKind::ImmediateAllTorqueOffToRest | EndingKind::ImmediateAllTorqueOffToPark => {
                Some(Maneuver::ImmediateAllTorqueOff)
            }
        }
    }

    /// Whether this ending carries the head down under control, rather than
    /// letting go where it stands.
    ///
    /// Read off [`maneuver`] rather than listed again: the endings whose
    /// maneuver is the immediate torque-off are exactly the ones a stow is no
    /// longer an answer for, and a refusal does nothing to the machine at all.
    #[must_use]
    pub fn stows(ending: EndingKind) -> bool {
        match maneuver(ending) {
            Some(Maneuver::SlowStow | Maneuver::MaskedSlowStow) => true,
            // An ending never runs the antenna torque-off: degrading a pair is
            // not an ending, and the ending that names that response answers
            // with the stow.
            Some(Maneuver::AntennaTorqueOff | Maneuver::ImmediateAllTorqueOff) | None => false,
        }
    }

    /// Which of two endings the machine is judged by: the one that asks more of
    /// whoever finds it.
    ///
    /// What answers a compound ending — a stow defeated by a servo dropping
    /// out, a torque-off nobody acknowledged on the way out of another fault.
    ///
    /// Ranked rather than compared field by field, because the order is a
    /// judgement and not an arithmetic: a park outranks any rest, and within the
    /// parks the controlled descent outranks going limp on the spot.
    /// Wildcard-free, so an ending added to the doctrine is ranked here or does
    /// not compile.
    #[must_use]
    pub fn worse(left: EndingKind, right: EndingKind) -> EndingKind {
        let rank = |ending| match ending {
            EndingKind::None => 0,
            EndingKind::Refuse => 1,
            EndingKind::SlowStowToRest => 2,
            EndingKind::ImmediateAllTorqueOffToRest => 3,
            EndingKind::MaskedSlowStowToPark => 4,
            EndingKind::ImmediateAllTorqueOffToPark => 5,
        };
        if rank(right) > rank(left) {
            right
        } else {
            left
        }
    }

    /// Whether the machine this ending leaves behind may be engaged again by
    /// whatever asks next, or has to wait for a person.
    ///
    /// A refusal changed nothing, so there is nothing to wait for, and no
    /// ending changed nothing either.
    #[must_use]
    pub fn disposition(ending: EndingKind) -> Disposition {
        match ending {
            EndingKind::None
            | EndingKind::Refuse
            | EndingKind::SlowStowToRest
            | EndingKind::ImmediateAllTorqueOffToRest => Disposition::Rest,
            EndingKind::MaskedSlowStowToPark | EndingKind::ImmediateAllTorqueOffToPark => {
                Disposition::Park
            }
        }
    }
}

/// Where a response leaves the machine.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Disposition {
    /// Torque off, and the next wake engages as usual.
    Rest,
    /// Torque off, and nothing engages until an operator has been.
    Park,
}

/// How far a wind-down got, in the three shapes it can end in.
///
/// The vocabulary's own enum, declared in `motion/timeline.clk` beside the
/// narration that carries it: the number is what a `winddown_outcome` report's
/// first specific holds. Narrower than the response vocabulary, because a
/// wind-down that has begun ends — the question left is whether the machine was
/// measured where the maneuver was to leave it, reached the end unmeasured, or
/// was given up on. [`WindDownOutcome::None`] is what a slot that has concluded
/// no maneuver holds, and no conclusion [`WindDown::next`] ever reaches.
pub use brenn_reachy__motion__timeline_clk_rs::WindDownOutcome;

/// How the stow this core last asked for ended.
///
/// Three answers rather than a `Result`, because the middle one is not a
/// failure: a head servo dropping out of the moves expands the maneuver, and
/// telling that from a stow the machine defeated is what decides whether the
/// head is carried the rest of the way down or dropped where it stands.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum StowEnding {
    /// The stow ran to its end and the machine was measured at stow.
    Stowed,
    /// A servo that carries the head dropped out; it is already released, and
    /// whatever is left carries the head the rest of the way down.
    MaskGrew(Fault),
    /// The stow ended on anything else — an abort, a wire that stopped
    /// carrying, a condition control is not trusted through.
    Defeated,
}

/// What the host learned since the last call.
///
/// Everything the decision turns on that this core cannot see for itself. The
/// two judgements are the caller's to make from its own latest reading — which
/// joints are still commanded, and how the stow it was asked for ended — and
/// neither of them is a clock: `now` is the only time this core reads, and it
/// arrives beside this.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Evidence {
    /// No joint that carries the head is still commanded, so there is nothing
    /// left to drive a stow with. Judged by the caller over its latest reading.
    pub head_released: bool,
    /// The stow the last [`WindDownAction::CommandStow`] asked for, if one has
    /// ended since. `None` on the opening call, and on any call made while a
    /// commanded stow is still running.
    pub stow: Option<StowEnding>,
}

impl Evidence {
    /// Nothing has happened yet: the head is still carried and no stow has
    /// ended.
    #[must_use]
    pub fn nothing() -> Self {
        Self {
            head_released: false,
            stow: None,
        }
    }
}

/// What the host is to do next.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum WindDownAction {
    /// Command the stow pose on whatever still commands, with every move clock
    /// cut to `remaining` — [`within`] is that cut.
    ///
    /// Asked again after an expansion, from wherever the machine now stands,
    /// with whatever is left of the one clock.
    CommandStow {
        /// What is left of the maneuver's single clock.
        remaining: Duration,
    },
    /// The maneuver is over: release torque, and record it as this says.
    Conclude {
        /// How far the maneuver got.
        outcome: WindDownOutcome,
        /// Where the machine is left, and whether the next ask may engage it.
        disposition: Disposition,
    },
}

/// Why a slot's fields describe no maneuver the machine could be running.
///
/// Every variant is a refusal rather than a repair: an ending a maneuver cannot
/// be judged by, and a deadline that is not an instant, are both slots written by
/// something other than a maneuver, and either repair available would report a
/// maneuver nobody started.
#[derive(Clone, Copy, Debug, Error, PartialEq, Eq)]
pub enum WindDownError {
    /// The ending a maneuver is judged by is `none`, which is no ending. Refused
    /// as a maneuver opens and refused in a slot claiming one is running, so
    /// neither boundary can mint the state the other reads as corrupt.
    #[error("a running maneuver is judged by no ending")]
    NoEnding,
    /// The deadline is not a length of time on the caller's scale.
    #[error("the maneuver's deadline is not an instant: {0}")]
    Deadline(#[from] DurationError),
}

/// The wind-down maneuver, mid-flight, over the slot that holds it.
///
/// Two facts and nothing else: which ending the machine is being judged by —
/// which a servo dropping out can only make worse — and when the one clock is
/// spent. Both live in the schema the host persists, so there is nothing to
/// flatten across an execution boundary and nothing that can be out of step
/// with what the slot says.
#[derive(Debug)]
pub struct WindDown<'a> {
    state: &'a mut WindDownSnap,
}

impl<'a> WindDown<'a> {
    /// Open the maneuver in `slot`: one clock, taken once, spent `stow_budget`
    /// from now.
    ///
    /// The deadline is written from `now` here and never again, so however many
    /// times a servo dropping out re-commands the stow, the whole maneuver is
    /// bounded by the one stow it began as. The slot is cleared first, so no
    /// part of an earlier maneuver survives into this one.
    ///
    /// # Errors
    ///
    /// [`WindDownError::NoEnding`] for an ending of `none`, which is no ending:
    /// what a maneuver is judged by is decided before it opens, and `none` is the
    /// value a slot holds when nothing has answered anything.
    /// [`WindDownError::Deadline`] for a deadline past what the slot's count
    /// reaches. The slot is untouched on either refusal, so a caller carrying on
    /// past the error does not hold a maneuver wearing half of two clocks, or one
    /// nothing can judge.
    pub fn begin(
        class: EndingKind,
        now: Duration,
        stow_budget: Duration,
        slot: &'a mut WindDownSnapWire,
    ) -> Result<Self, WindDownError> {
        // Both refusals are decided before the first write, so a refusal costs
        // the slot nothing.
        if class == EndingKind::None {
            return Err(WindDownError::NoEnding);
        }
        let deadline = duration_nanos(now.saturating_add(stow_budget))?;
        let state = slot.clear_valid();
        state.active = true.into();
        state.deadline = SyncTime::from_nanos(deadline);
        Ok(Self { state }.judged_by(class))
    }

    /// The maneuver `state` holds, or that it is holding none.
    ///
    /// What a host resumes with at the top of an execution, and what a caller
    /// that commanded part of the answer itself — a host that stowed under
    /// control and had the stow defeated — picks up again. It gets the remainder
    /// of the clock rather than a fresh one, which is the same rule
    /// [`Self::begin`] enforces from the other side.
    ///
    /// # Errors
    ///
    /// [`WindDownError`] for fields that describe no running maneuver: an
    /// ending of `none`, or a deadline that is not an instant.
    pub fn resume(state: &'a mut WindDownSnap) -> Result<Option<Self>, WindDownError> {
        if !bool::from(state.active) {
            return Ok(None);
        }
        // `none` is refused rather than carried: a maneuver is running, and an
        // ending of none is not one the machine can be judged by.
        if state.ending == EndingKind::None {
            return Err(WindDownError::NoEnding);
        }
        duration_from_nanos(state.deadline.as_nanos())?;
        Ok(Some(Self { state }))
    }

    /// The ending the machine is currently judged by.
    ///
    /// The sticky maximum of the class it began with and every class raised
    /// since, so this only ever asks more of whoever finds the machine.
    #[must_use]
    pub fn class(&self) -> EndingKind {
        self.state.ending
    }

    /// When the one clock is spent, on the caller's own scale.
    ///
    /// Total: both constructors refuse a deadline that is not a length of time,
    /// and the borrow this holds is exclusive, so nothing has written the field
    /// since — and nothing here writes it at all.
    #[must_use]
    pub fn deadline(&self) -> Duration {
        duration_from_nanos(self.state.deadline.as_nanos())
            .unwrap_or_else(|err| unreachable!("a checked deadline stopped being one: {err}"))
    }

    /// Where the machine will be left if it concluded now.
    #[must_use]
    pub fn disposition(&self) -> Disposition {
        ending::disposition(self.state.ending)
    }

    /// Re-rank the maneuver against a condition raised while it runs.
    ///
    /// The ladder never begins a second answer to a machine already being
    /// answered: a fault arriving mid-maneuver can make the disposition worse
    /// and can change which servos carry the head down, and it never re-opens
    /// the clock and never starts a second wind-down.
    pub fn raised(&mut self, fault: &Fault) {
        self.re_ranked(crate::fault::kind(fault));
    }

    /// The same, for a host whose evidence is the classified condition rather
    /// than the tick's own value.
    ///
    /// A raise that crossed a process boundary arrives as its kind: the
    /// evidence that classified it travels beside it as numbers a report
    /// carries, and the kind is the whole of what the ranking reads. So both
    /// callers rank through this, and neither of them decides what a condition
    /// asks of whoever finds the machine.
    pub fn re_ranked(&mut self, kind: crate::fault::FaultKind) {
        let worse = ending::worse(
            self.state.ending,
            ending::answering(crate::fault::response(kind)),
        );
        self.set_ending(worse);
    }

    /// Judge the maneuver by `class`, whatever it was judged by before.
    fn judged_by(mut self, class: EndingKind) -> Self {
        self.set_ending(class);
        self
    }

    /// The ending the maneuver is judged by from here on.
    ///
    /// The one fact stored; where it leaves the machine is
    /// [`ending::disposition`] over it, asked wherever it is wanted rather than
    /// kept beside it.
    fn set_ending(&mut self, class: EndingKind) {
        self.state.ending = class;
    }

    /// What to do next, given the time and what the machine has done.
    ///
    /// One step of the maneuver. The order is the doctrine's: an ended stow is
    /// judged first, then whether anything is left to drive one with, then
    /// whether the ending the machine is now judged by still carries it down at
    /// all, then the clock — and only a maneuver that survives all four is asked
    /// for again.
    pub fn next(&mut self, now: Duration, evidence: Evidence) -> WindDownAction {
        match evidence.stow {
            Some(StowEnding::Stowed) => return self.conclude(WindDownOutcome::Completed),
            // The mask grew. Nothing about the maneuver changes but which
            // servos carry the head the rest of the way down — and, where the
            // condition asks more of whoever finds the machine, where it is
            // left.
            Some(StowEnding::MaskGrew(fault)) => self.raised(&fault),
            Some(StowEnding::Defeated) => return self.conclude(WindDownOutcome::FellThrough),
            None => {}
        }
        if evidence.head_released {
            // Not a stow: there is nothing left to drive one with. The maneuver
            // is over all the same — the mask growing to cover the head is the
            // torque-off it was walking towards — but nothing put the head down
            // and nothing measured where it came to rest, so the record says as
            // much. Saying the head came down under control when every joint
            // that carries it has gone limp is the one claim this record must
            // not make.
            return self.conclude(WindDownOutcome::Unconfirmed);
        }
        if !ending::stows(self.state.ending) {
            // The ending the machine is now judged by no longer carries the head
            // down. Whatever raised it says control is not trusted, and what
            // that gets is the release on the spot: commanding another stow here
            // would hold torque on a machine nobody can command for the rest of
            // the clock.
            return self.conclude(WindDownOutcome::FellThrough);
        }
        let remaining = self.deadline().saturating_sub(now);
        if remaining.is_zero() {
            return self.conclude(WindDownOutcome::FellThrough);
        }
        WindDownAction::CommandStow { remaining }
    }

    /// End the maneuver as `outcome`, where this leaves the machine.
    ///
    /// The slot stops claiming a running maneuver in the same statement that
    /// names the conclusion, so a host that persists the slot and comes back
    /// resumes nothing rather than the maneuver it has already ended: what the
    /// slot says and what this decided are one thing. The ending and the deadline
    /// are left as they stand — they mean nothing while nothing is running, and
    /// how the maneuver was judged travels in the action.
    fn conclude(&mut self, outcome: WindDownOutcome) -> WindDownAction {
        let disposition = self.disposition();
        self.state.active = false.into();
        WindDownAction::Conclude {
            outcome,
            disposition,
        }
    }
}

/// The same move clocks, with nothing over `left` on any of them.
///
/// What a stow this core asks for is commanded with: the maneuver's clock is the
/// one it started with, so an expansion gets the remainder of it rather than a
/// fresh one. A remainder shorter than the move can be run in is floored by the
/// move's own guard, as any other under-clocked move is, and the deadline
/// catches the overrun on the next pass.
#[must_use]
pub fn within(durations: MoveDurations, left: Duration) -> MoveDurations {
    MoveDurations {
        head: durations.head.min(left),
        antennas: durations.antennas.map(|antenna| antenna.min(left)),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use brenn_reachy__motion__timeline_clk_rs::WindDownOutcomeWire;

    crate::vocab_numbering! {
        /// The outcomes carry the numbers they are reported under.
        ///
        /// The number is what a `winddown_outcome` report's first specific
        /// holds, so a recorded story and a running build have it in common: a
        /// value inserted among these would move the whole family under a
        /// reader that had already recorded one. The zero is the
        /// concluded-nothing slot rather than the first outcome.
        the_outcomes_carry_the_numbers_they_are_reported_under:
            WindDownOutcome as WindDownOutcomeWire, past the end 4 {
            WindDownOutcome::None => 0,
            WindDownOutcome::Completed => 1,
            WindDownOutcome::Unconfirmed => 2,
            WindDownOutcome::FellThrough => 3,
        }
    }

    use crate::joints::JointRef;
    use brenn_reachy__motion__faults_clk_rs::EndingKindWire;

    /// A second on the caller's scale.
    const SECOND: Duration = Duration::from_secs(1);

    /// A head servo dropping out, as the tick raises it.
    fn head_servo_lost() -> Fault {
        Fault::HeadServoFault {
            joint: JointRef::Leg2,
            id: 13,
            bits: 0x20,
        }
    }

    /// A grabbed head, whose response is the stow to rest.
    fn head_grabbed() -> Fault {
        Fault::HeadObstructed {
            joint: JointRef::Leg0,
            error: 0.2,
        }
    }

    /// The head still carried, and a stow that ended as `ending`.
    fn after(ending: StowEnding) -> Evidence {
        Evidence {
            head_released: false,
            stow: Some(ending),
        }
    }

    /// A maneuver opened on `slot`, ready to be stepped.
    ///
    /// The slot is the state, so every fixture owns one and the maneuver borrows
    /// it for as long as it is stepped.
    fn begun<'a>(
        slot: &'a mut WindDownSnapWire,
        class: EndingKind,
        now: Duration,
        stow_budget: Duration,
    ) -> WindDown<'a> {
        WindDown::begin(class, now, stow_budget, slot)
            .expect("a clock on this scale is an instant the slot holds")
    }

    /// Every response the doctrine names maps onto an ending, and the mapping
    /// is total.
    ///
    /// The one divergence is the response that is not an ending: an error that
    /// names `DegradeAntennas` comes from a caller that surfaced the tick's
    /// degrade as an ending, and what a still-commanding head gets for it is
    /// the stow — not the antenna torque-off, which has already happened.
    #[test]
    fn every_class_names_the_maneuver_it_runs() {
        let table: Vec<(ResponseKind, Option<Maneuver>)> = vec![
            (ResponseKind::Refuse, None),
            (ResponseKind::SlowStowToRest, Some(Maneuver::SlowStow)),
            (
                ResponseKind::MaskedSlowStowToPark,
                Some(Maneuver::MaskedSlowStow),
            ),
            (
                ResponseKind::ImmediateAllTorqueOffToRest,
                Some(Maneuver::ImmediateAllTorqueOff),
            ),
            (
                ResponseKind::ImmediateAllTorqueOffToPark,
                Some(Maneuver::ImmediateAllTorqueOff),
            ),
            (ResponseKind::DegradeAntennas, Some(Maneuver::SlowStow)),
            // No answer ends nothing: the value a slot that answered nothing
            // holds stays no ending rather than becoming the mildest one.
            (ResponseKind::None, None),
        ];

        let mut judged = [false; 7];
        for (response, maneuver) in table {
            judged[response_slot(response)] = true;
            assert_eq!(
                ending::maneuver(ending::answering(response)),
                maneuver,
                "{response:?}"
            );
        }
        assert!(
            judged.iter().all(|seen| *seen),
            "a response named no maneuver: {judged:?}"
        );
        assert_eq!(
            ending::answering(ResponseKind::None),
            EndingKind::None,
            "no answer is no ending"
        );
    }

    /// A response names the maneuver it runs, and it is the ending's maneuver
    /// everywhere but the degrade.
    ///
    /// The whole vocabulary, and the one divergence asserted as the divergence:
    /// a host that reads a response off the classifier and asks what to do about
    /// it gets the antenna torque-off, where a host that turned the same
    /// response into an ending first would get the stow. Both are right for
    /// their question, and this is what says so.
    #[test]
    fn a_response_names_the_maneuver_it_runs() {
        let mut judged = [false; 7];
        for (response, maneuver) in [
            (ResponseKind::Refuse, None),
            (ResponseKind::SlowStowToRest, Some(Maneuver::SlowStow)),
            (
                ResponseKind::MaskedSlowStowToPark,
                Some(Maneuver::MaskedSlowStow),
            ),
            (
                ResponseKind::DegradeAntennas,
                Some(Maneuver::AntennaTorqueOff),
            ),
            (
                ResponseKind::ImmediateAllTorqueOffToRest,
                Some(Maneuver::ImmediateAllTorqueOff),
            ),
            (
                ResponseKind::ImmediateAllTorqueOffToPark,
                Some(Maneuver::ImmediateAllTorqueOff),
            ),
            (ResponseKind::None, None),
        ] {
            judged[response_slot(response)] = true;
            assert_eq!(maneuver_of(response), maneuver, "{response:?}");
            let as_ending = ending::maneuver(ending::answering(response));
            if matches!(response, ResponseKind::DegradeAntennas) {
                assert_eq!(
                    as_ending,
                    Some(Maneuver::SlowStow),
                    "an ending that names the degrade stows the still-commanding head",
                );
            } else {
                assert_eq!(as_ending, maneuver, "{response:?} runs one maneuver");
            }
        }
        assert!(
            judged.iter().all(|seen| *seen),
            "a response named no maneuver: {judged:?}"
        );
    }

    /// A compound ending is judged by the one that asks more of whoever finds
    /// it, whichever order the two arrived in.
    ///
    /// The order is the doctrine's: any park outranks any rest, and within each
    /// pair the maneuver that carries the head down outranks going limp where it
    /// stands. Pinned as the whole 25-pair table, so a re-ranking — which
    /// compiles, unlike an added class — has to be re-justified here.
    #[test]
    fn the_worse_of_two_endings_is_the_one_that_asks_more() {
        // Weakest first: this order *is* the ranking, and the assertions below
        // read it off the position rather than restating the arithmetic.
        let ranked = [
            EndingKind::None,
            EndingKind::Refuse,
            EndingKind::SlowStowToRest,
            EndingKind::ImmediateAllTorqueOffToRest,
            EndingKind::MaskedSlowStowToPark,
            EndingKind::ImmediateAllTorqueOffToPark,
        ];
        for (i, left) in ranked.into_iter().enumerate() {
            for (j, right) in ranked.into_iter().enumerate() {
                let expected = ranked[i.max(j)];
                assert_eq!(
                    ending::worse(left, right),
                    expected,
                    "{left:?} against {right:?}"
                );
                assert_eq!(
                    ending::worse(right, left),
                    expected,
                    "{right:?} against {left:?}"
                );
            }
        }
        // The rank is not the disposition: a rest can outrank another rest, and
        // every park outranks every rest.
        for rest in [
            EndingKind::None,
            EndingKind::Refuse,
            EndingKind::SlowStowToRest,
            EndingKind::ImmediateAllTorqueOffToRest,
        ] {
            for park in [
                EndingKind::MaskedSlowStowToPark,
                EndingKind::ImmediateAllTorqueOffToPark,
            ] {
                assert_eq!(
                    ending::disposition(ending::worse(rest, park)),
                    Disposition::Park
                );
            }
        }
    }

    /// Which answer this is, as a slot in the coverage above.
    ///
    /// Wildcard-free, so a response added to the doctrine cannot be left out of
    /// the maneuver table by the table simply not mentioning it.
    fn response_slot(response: ResponseKind) -> usize {
        match response {
            ResponseKind::Refuse => 0,
            ResponseKind::SlowStowToRest => 1,
            ResponseKind::DegradeAntennas => 2,
            ResponseKind::MaskedSlowStowToPark => 3,
            ResponseKind::ImmediateAllTorqueOffToRest => 4,
            ResponseKind::ImmediateAllTorqueOffToPark => 5,
            ResponseKind::None => 6,
        }
    }

    /// The clock is opened once, from the instant the maneuver begins, and the
    /// first thing asked for is the stow with the whole of it.
    #[test]
    fn a_maneuver_opens_one_clock_and_asks_for_the_stow() {
        let now = 10 * SECOND;
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(&mut slot, EndingKind::SlowStowToRest, now, 4 * SECOND);
        assert_eq!(wd.deadline(), 14 * SECOND);
        assert_eq!(
            wd.next(now, Evidence::nothing()),
            WindDownAction::CommandStow {
                remaining: 4 * SECOND
            }
        );
        assert_eq!(
            wd.next(now + 2 * SECOND, Evidence::nothing()),
            WindDownAction::CommandStow {
                remaining: 2 * SECOND
            }
        );
    }

    /// A stow that reached its end concludes completed, at rest for a
    /// rest-class ending.
    #[test]
    fn a_stow_that_lands_concludes_completed() {
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(&mut slot, EndingKind::SlowStowToRest, SECOND, 4 * SECOND);
        assert_eq!(
            wd.next(2 * SECOND, after(StowEnding::Stowed)),
            WindDownAction::Conclude {
                outcome: WindDownOutcome::Completed,
                disposition: Disposition::Rest,
            }
        );
    }

    /// A park-class maneuver concludes parked however well the stow went.
    #[test]
    fn a_park_class_maneuver_concludes_parked() {
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(
            &mut slot,
            EndingKind::MaskedSlowStowToPark,
            SECOND,
            4 * SECOND,
        );
        assert_eq!(
            wd.next(2 * SECOND, after(StowEnding::Stowed)),
            WindDownAction::Conclude {
                outcome: WindDownOutcome::Completed,
                disposition: Disposition::Park,
            }
        );
    }

    /// A head servo dropping out mid-stow re-commands the stow on what is left
    /// of the same clock, and latches the park.
    #[test]
    fn a_servo_dropping_out_expands_the_maneuver_without_re_opening_the_clock() {
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(
            &mut slot,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            4 * SECOND,
        );
        assert_eq!(wd.disposition(), Disposition::Rest);
        assert_eq!(
            wd.next(SECOND, after(StowEnding::MaskGrew(head_servo_lost()))),
            WindDownAction::CommandStow {
                remaining: 3 * SECOND
            }
        );
        assert_eq!(wd.deadline(), 4 * SECOND, "the clock never restarts");
        assert_eq!(wd.class(), EndingKind::MaskedSlowStowToPark);
        assert_eq!(wd.disposition(), Disposition::Park);
        assert_eq!(
            wd.next(2 * SECOND, after(StowEnding::MaskGrew(head_servo_lost()))),
            WindDownAction::CommandStow {
                remaining: 2 * SECOND
            }
        );
        assert_eq!(wd.class(), EndingKind::MaskedSlowStowToPark);
    }

    /// A condition raised mid-maneuver only ever asks more of whoever finds the
    /// machine.
    #[test]
    fn a_condition_raised_mid_maneuver_never_softens_the_ending() {
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(
            &mut slot,
            EndingKind::MaskedSlowStowToPark,
            Duration::ZERO,
            4 * SECOND,
        );
        wd.raised(&head_grabbed());
        assert_eq!(
            wd.class(),
            EndingKind::MaskedSlowStowToPark,
            "a grabbed head does not unpark a released servo"
        );
        wd.raised(&Fault::BusFailure {
            source: crate::tick::BusFailureSource::Transaction {
                id: 13,
                kind: crate::tick::WireFailure::Silent,
            },
        });
        assert_eq!(wd.class(), EndingKind::ImmediateAllTorqueOffToPark);
    }

    /// A stow the machine defeated falls through to the immediate release.
    #[test]
    fn a_defeated_stow_falls_through() {
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(
            &mut slot,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            4 * SECOND,
        );
        assert_eq!(
            wd.next(SECOND, after(StowEnding::Defeated)),
            WindDownAction::Conclude {
                outcome: WindDownOutcome::FellThrough,
                disposition: Disposition::Rest,
            }
        );
    }

    /// A clock that is spent falls through, and the expansion path cannot buy
    /// past it.
    #[test]
    fn a_spent_clock_falls_through() {
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(
            &mut slot,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            4 * SECOND,
        );
        assert_eq!(
            wd.next(4 * SECOND, Evidence::nothing()),
            WindDownAction::Conclude {
                outcome: WindDownOutcome::FellThrough,
                disposition: Disposition::Rest,
            }
        );
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(
            &mut slot,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            4 * SECOND,
        );
        assert_eq!(
            wd.next(5 * SECOND, after(StowEnding::MaskGrew(head_servo_lost()))),
            WindDownAction::Conclude {
                outcome: WindDownOutcome::FellThrough,
                disposition: Disposition::Park,
            },
            "the expansion is ranked, and then the clock is still spent"
        );
    }

    /// Nothing left to drive a stow with concludes unconfirmed, not completed —
    /// and does so before the clock is consulted, because the clock cannot
    /// change the answer.
    #[test]
    fn a_head_with_nothing_carrying_it_concludes_unconfirmed() {
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(
            &mut slot,
            EndingKind::MaskedSlowStowToPark,
            Duration::ZERO,
            4 * SECOND,
        );
        let evidence = Evidence {
            head_released: true,
            stow: None,
        };
        assert_eq!(
            wd.next(SECOND, evidence),
            WindDownAction::Conclude {
                outcome: WindDownOutcome::Unconfirmed,
                disposition: Disposition::Park,
            }
        );
    }

    /// A stow that landed is judged on its landing, whatever the mask has done:
    /// the ended stow is the first question, so a caller reporting both facts
    /// in one call gets the completion.
    #[test]
    fn an_ended_stow_is_judged_before_the_mask_is() {
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(
            &mut slot,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            4 * SECOND,
        );
        let evidence = Evidence {
            head_released: true,
            stow: Some(StowEnding::Stowed),
        };
        assert_eq!(
            wd.next(SECOND, evidence),
            WindDownAction::Conclude {
                outcome: WindDownOutcome::Completed,
                disposition: Disposition::Rest,
            }
        );
    }

    /// A condition that escalates the ending to the immediate torque-off ends
    /// the stow there and then, rather than commanding a machine nobody can
    /// command for the rest of the clock.
    #[test]
    fn an_escalation_past_the_controlled_stow_stops_commanding() {
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(
            &mut slot,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            4 * SECOND,
        );
        wd.raised(&Fault::BusFailure {
            source: crate::tick::BusFailureSource::Transaction {
                id: 13,
                kind: crate::tick::WireFailure::Silent,
            },
        });
        assert_eq!(
            wd.next(SECOND, Evidence::nothing()),
            WindDownAction::Conclude {
                outcome: WindDownOutcome::FellThrough,
                disposition: Disposition::Park,
            },
            "the clock has three seconds left and the answer is still the release"
        );

        // The same escalation arriving as the stow's own ending.
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(
            &mut slot,
            EndingKind::MaskedSlowStowToPark,
            Duration::ZERO,
            4 * SECOND,
        );
        let lost_feedback = Fault::PositionFeedbackLost { misses: 4 };
        assert!(
            !ending::stows(ending::answering(crate::fault::response(
                crate::fault::kind(&lost_feedback)
            ))),
            "a fixture that no longer escalates past the stow proves nothing"
        );
        assert_eq!(
            wd.next(SECOND, after(StowEnding::MaskGrew(lost_feedback))),
            WindDownAction::Conclude {
                outcome: WindDownOutcome::FellThrough,
                disposition: Disposition::Park,
            }
        );
    }

    /// Which endings carry the head down, as the maneuver each one runs.
    #[test]
    fn only_the_stowing_endings_command_a_stow() {
        for class in [EndingKind::SlowStowToRest, EndingKind::MaskedSlowStowToPark] {
            assert!(ending::stows(class), "{class:?}");
        }
        for class in [
            EndingKind::Refuse,
            EndingKind::ImmediateAllTorqueOffToRest,
            EndingKind::ImmediateAllTorqueOffToPark,
        ] {
            assert!(!ending::stows(class), "{class:?}");
        }
    }

    /// A maneuver picked up out of its slot runs on the clock it finds there,
    /// not a fresh one.
    #[test]
    fn a_resumed_maneuver_gets_the_remainder() {
        let mut slot = WindDownSnapWire::new();
        begun(
            &mut slot,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            3 * SECOND,
        );

        let mut wd = WindDown::resume(slot.validate_mut().expect("the slot holds a maneuver"))
            .expect("the maneuver reads")
            .expect("and it is running");
        assert_eq!(
            wd.next(2 * SECOND, Evidence::nothing()),
            WindDownAction::CommandStow { remaining: SECOND }
        );
    }

    /// A maneuver waiting in its slot is the same maneuver when it is picked up:
    /// the same ending, judged by the same deadline, leaving the machine in the
    /// same place.
    ///
    /// Over every ending there is, so an ending added to the doctrine arrives
    /// here without this test naming it.
    #[test]
    fn every_maneuver_survives_the_slot_it_waits_in() {
        for class in EndingKind::VARIANTS {
            // `none` is not an ending a maneuver can be judged by; the resume
            // refuses it, which the case below this one pins.
            if class == EndingKind::None {
                continue;
            }
            let mut slot = WindDownSnapWire::new();
            begun(&mut slot, class, SECOND, 4 * SECOND);
            assert_eq!(slot.deadline().as_nanos(), 5_000_000_000, "{class:?}");

            let wd = WindDown::resume(slot.validate_mut().expect("the slot holds a maneuver"))
                .expect("the maneuver reads")
                .expect("and it is running");
            assert_eq!(wd.class(), class);
            assert_eq!(wd.deadline(), 5 * SECOND, "{class:?}");
            assert_eq!(wd.disposition(), ending::disposition(class), "{class:?}");
        }
    }

    /// An expansion writes the ending it escalated to, so the maneuver its own
    /// slot reads back is the escalated one.
    #[test]
    fn an_expanded_maneuver_is_one_its_slot_still_reads() {
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(
            &mut slot,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            4 * SECOND,
        );
        wd.raised(&head_servo_lost());

        let wd = WindDown::resume(slot.validate_mut().expect("the slot holds a maneuver"))
            .expect("an escalated maneuver still reads")
            .expect("and it is running");
        assert_eq!(wd.class(), EndingKind::MaskedSlowStowToPark);
        assert_eq!(wd.disposition(), Disposition::Park);
    }

    #[test]
    fn a_slot_nobody_wrote_is_running_no_maneuver() {
        let mut slot = WindDownSnapWire::new();
        assert!(matches!(
            WindDown::resume(slot.validate_mut().expect("a zeroed slot validates")),
            Ok(None)
        ));
    }

    #[test]
    fn an_ending_a_running_maneuver_cannot_be_judged_by_is_refused() {
        let mut slot = WindDownSnapWire::new();
        slot.set_active(true);

        // What an unwritten ending field holds, claimed by an active maneuver.
        assert_eq!(
            WindDown::resume(slot.validate_mut().expect("the number is declared")).err(),
            Some(WindDownError::NoEnding)
        );

        // And a number no ending is: refused at the boundary, before anything
        // here is asked about it.
        slot.set_ending(EndingKindWire(77));
        assert!(
            slot.validate_mut().is_err(),
            "a number this build does not name is not an ending"
        );
    }

    #[test]
    fn a_deadline_that_is_not_an_instant_is_refused() {
        let mut slot = WindDownSnapWire::new();
        begun(
            &mut slot,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            3 * SECOND,
        );
        slot.set_deadline(SyncTime::from_nanos(-1));

        assert!(
            matches!(
                WindDown::resume(slot.validate_mut().expect("the slot validates")),
                Err(WindDownError::Deadline(_))
            ),
            "a deadline before the epoch is not one"
        );
    }

    /// A deadline the slot's count cannot reach is refused as the maneuver
    /// opens, and the slot keeps whatever it already held — not a cleared slot,
    /// and not a live maneuver wearing half of two clocks.
    #[test]
    fn a_deadline_the_slot_cannot_hold_is_refused_and_costs_the_slot_nothing() {
        let mut slot = WindDownSnapWire::new();
        begun(
            &mut slot,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            3 * SECOND,
        );
        let mut held = WindDownSnapWire::new();
        begun(
            &mut held,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            3 * SECOND,
        );

        assert!(
            matches!(
                WindDown::begin(
                    EndingKind::MaskedSlowStowToPark,
                    Duration::ZERO,
                    Duration::MAX,
                    &mut slot,
                ),
                Err(WindDownError::Deadline(_))
            ),
            "a deadline past what the count reaches is not one"
        );
        assert_eq!(slot, held, "the refused open touched nothing");
    }

    /// The stow's clocks are cut to what is left, and a clock already inside
    /// the remainder is untouched.
    #[test]
    fn the_stow_clocks_are_cut_to_the_remainder() {
        let durations = MoveDurations {
            head: 3 * SECOND,
            antennas: [SECOND, 4 * SECOND],
        };
        assert_eq!(
            within(durations, 2 * SECOND),
            MoveDurations {
                head: 2 * SECOND,
                antennas: [SECOND, 2 * SECOND],
            }
        );
        assert_eq!(within(durations, 10 * SECOND), durations);
    }

    /// Every maneuver reports under its own word, and no two report under the
    /// same one.
    ///
    /// The slug is the join key an operator greps a log for and an alert keys
    /// on, so the four strings are pinned here rather than read off the code
    /// that produces them. Wildcard-free coverage, so a maneuver added to the
    /// doctrine cannot be left unnamed by this table simply not mentioning it.
    #[test]
    fn every_maneuver_names_itself_and_no_other() {
        let table = [
            (Maneuver::SlowStow, "slow_stow"),
            (Maneuver::MaskedSlowStow, "masked_slow_stow"),
            (Maneuver::AntennaTorqueOff, "antenna_torque_off"),
            (Maneuver::ImmediateAllTorqueOff, "immediate_all_torque_off"),
        ];

        let mut named = [false; 4];
        for (maneuver, slug) in table {
            named[maneuver_slot(maneuver)] = true;
            assert_eq!(maneuver.slug(), slug, "{maneuver:?}");
            assert_eq!(maneuver.to_string(), slug, "{maneuver:?}");
        }
        assert!(
            named.iter().all(|seen| *seen),
            "a maneuver named no slug: {named:?}"
        );

        let mut slugs: Vec<&str> = table.iter().map(|(_, slug)| *slug).collect();
        slugs.sort_unstable();
        slugs.dedup();
        assert_eq!(slugs.len(), table.len(), "two maneuvers share a word");
    }

    /// Which maneuver this is, as a slot in the coverage above.
    fn maneuver_slot(maneuver: Maneuver) -> usize {
        match maneuver {
            Maneuver::SlowStow => 0,
            Maneuver::MaskedSlowStow => 1,
            Maneuver::AntennaTorqueOff => 2,
            Maneuver::ImmediateAllTorqueOff => 3,
        }
    }

    /// A maneuver that concluded is running nothing: the slot it concluded in
    /// resumes as no maneuver, so the host that persists it neither re-ends the
    /// maneuver nor commands the stow again.
    ///
    /// Over every outcome the core can conclude in, because the write that ends
    /// the maneuver must not depend on how far it got.
    #[test]
    fn a_concluded_maneuver_is_running_none() {
        let concluding = [
            (EndingKind::SlowStowToRest, after(StowEnding::Stowed)),
            (EndingKind::SlowStowToRest, after(StowEnding::Defeated)),
            (
                EndingKind::SlowStowToRest,
                Evidence {
                    head_released: true,
                    stow: None,
                },
            ),
            (EndingKind::ImmediateAllTorqueOffToPark, Evidence::nothing()),
        ];

        for (class, evidence) in concluding {
            let mut slot = WindDownSnapWire::new();
            let mut wd = begun(&mut slot, class, SECOND, 4 * SECOND);
            assert!(
                matches!(
                    wd.next(2 * SECOND, evidence),
                    WindDownAction::Conclude { .. }
                ),
                "{class:?} with {evidence:?}"
            );

            assert!(
                matches!(
                    WindDown::resume(slot.validate_mut().expect("a concluded slot validates")),
                    Ok(None)
                ),
                "{class:?} with {evidence:?} left a maneuver running"
            );
        }
    }

    /// A spent clock concludes and the slot says so too, which is the case a
    /// host would otherwise re-command the stow in: the deadline is still there
    /// to be read, and nothing is running against it.
    #[test]
    fn a_maneuver_whose_clock_ran_out_is_running_none() {
        let mut slot = WindDownSnapWire::new();
        let mut wd = begun(
            &mut slot,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            3 * SECOND,
        );
        assert_eq!(
            wd.next(4 * SECOND, Evidence::nothing()),
            WindDownAction::Conclude {
                outcome: WindDownOutcome::FellThrough,
                disposition: Disposition::Rest,
            }
        );
        assert!(matches!(
            WindDown::resume(slot.validate_mut().expect("a concluded slot validates")),
            Ok(None)
        ));
    }

    /// No maneuver opens judged by no ending, and the refusal costs the slot
    /// nothing.
    ///
    /// The write and the read agree about what an ending is: `resume` calls a
    /// running maneuver judged by `none` a slot something else wrote, so this
    /// core must not be the something else.
    #[test]
    fn no_maneuver_opens_judged_by_no_ending() {
        let mut slot = WindDownSnapWire::new();
        let untouched = WindDownSnapWire::new();

        assert_eq!(
            WindDown::begin(EndingKind::None, SECOND, 4 * SECOND, &mut slot).err(),
            Some(WindDownError::NoEnding)
        );
        assert_eq!(slot, untouched, "the refused open touched nothing");
    }

    /// A maneuver opened in a slot an earlier one concluded in describes itself
    /// and nothing of the earlier one.
    ///
    /// The clear on the way in is what stands between a reused slot and a
    /// maneuver wearing the escalated ending, or the spent clock, of the one
    /// before it — nothing else re-blanks the slot.
    #[test]
    fn a_maneuver_begun_over_a_concluded_one_keeps_nothing_of_it() {
        let mut reused = WindDownSnapWire::new();
        let mut spent = begun(
            &mut reused,
            EndingKind::SlowStowToRest,
            Duration::ZERO,
            3 * SECOND,
        );
        spent.raised(&head_servo_lost());
        assert!(matches!(
            spent.next(9 * SECOND, after(StowEnding::Defeated)),
            WindDownAction::Conclude { .. }
        ));

        let mut fresh = WindDownSnapWire::new();
        let opened = begun(
            &mut reused,
            EndingKind::SlowStowToRest,
            10 * SECOND,
            4 * SECOND,
        );
        assert_eq!(opened.class(), EndingKind::SlowStowToRest);
        assert_eq!(opened.deadline(), 14 * SECOND);
        begun(
            &mut fresh,
            EndingKind::SlowStowToRest,
            10 * SECOND,
            4 * SECOND,
        );
        assert_eq!(reused, fresh, "the reused slot holds this maneuver only");
    }
}
