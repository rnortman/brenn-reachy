//! The stow maneuvers: carrying a machine down under control, one wake at a
//! time.
//!
//! Two rungs of the response ladder end this way -- the stow to rest and the
//! masked stow to park -- and both are the same maneuver over a different
//! disposition. Every judgement in them belongs to `reachy-motion`'s wind-down
//! core: the single clock the maneuver is bounded by, what a condition raised
//! while it runs does to the answer, when it is over and where the machine is
//! left. What is here is the doing, in the only way a session can do it: the
//! stow is a schedule the decision tick is asked to run, so commanding one is
//! publishing one, and the evidence the core is fed comes off the driver's own
//! pose stream.
//!
//! A stow is asked for once per stow and not once per wake, and one fact says
//! whether one is outstanding: the epoch the maneuver last commanded a stow
//! under, against the epoch the schedule is standing at. They agree while the
//! published stow is still being run, and a maneuver that owes one records the
//! epoch before the schedule's so that they cannot -- which is what a maneuver
//! just opened owes, and what a condition raised mid-maneuver leaves behind,
//! since a raise leaves the tick holding rather than moving and the stow has to
//! be asked for again from wherever the machine now stands.
//!
//! Nothing here reads a clock and nothing holds state of its own: the instant
//! arrives from the caller, and the maneuver lives in the session's slot.

use brenn_reachy__cogs__schedule_clk_rs::{PostureWire, ScheduledStepWire, StepKindWire};
use brenn_reachy__cogs__session_clk_rs::{SessionPhaseWire, SessionStateWire};
use brenn_reachy__driver__pose_clk_rs::PoseSample;
use clockwork_rs::SyncTime;
use reachy_motion::disarm::at_stow;
use reachy_motion::fault::FaultKind;
use reachy_motion::joints::{flags, vector_of};
use reachy_motion::snap::{duration_from_nanos, duration_nanos};
use reachy_motion::tick::ResponseKind;
use reachy_motion::winddown::{
    Disposition, Evidence, StowEnding, WindDown, WindDownAction, WindDownOutcome, ending,
};

use crate::session_bus::disarm_config;

/// Whether a stow maneuver is running.
///
/// Read before a response is selected: the ladder never begins a second answer
/// to a machine already being answered, so a condition arriving now re-ranks
/// this maneuver instead.
#[must_use]
pub fn running(slot: &SessionStateWire) -> bool {
    slot.winddown().active()
}

/// Open the maneuver `response` runs, and answer whether it opened.
///
/// The clock is taken once, here, and never re-opened: however many conditions
/// arrive on the way down, the whole maneuver is bounded by the one stow it
/// began as. `false` for a budget or an instant that is no length of time,
/// which is a number nobody can carry out a maneuver under -- the caller lets
/// go of the machine instead, because a maneuver that cannot be bounded is not
/// one to run under torque.
pub fn begin(
    slot: &mut SessionStateWire,
    response: ResponseKind,
    now_ns: i64,
    stow_budget_ns: i64,
) -> bool {
    let class = ending::answering(response);
    let (Ok(now), Ok(budget)) = (
        duration_from_nanos(now_ns),
        duration_from_nanos(stow_budget_ns),
    ) else {
        return false;
    };
    let opened = WindDown::begin(class, now, budget, slot.winddown_mut()).is_ok();
    if opened {
        // Nothing has been published for this maneuver, so the first wake owes
        // the stow.
        owe(slot);
    }
    opened
}

/// Re-rank the running maneuver against a condition raised while it runs, and
/// answer whether it could be.
///
/// The sticky-worse rule is the library's and the ranking is made there: what
/// this adds is the consequence for the machine, which is that the stow has to
/// be commanded again. A non-latching raise leaves the tick holding at the
/// setpoint it last commanded, so the move the published stow was driving has
/// stopped, and the maneuver's remaining clock is what the next one gets.
///
/// `false` for a record that stands as a maneuver and does not read back as one:
/// there is nothing to rank, and the caller lets go of the machine instead.
pub fn re_rank(slot: &mut SessionStateWire, kind: FaultKind) -> bool {
    let ranked = match slot.winddown_mut().validate_mut() {
        Ok(state) => match WindDown::resume(state) {
            Ok(Some(mut maneuver)) => {
                maneuver.re_ranked(kind);
                true
            }
            _ => false,
        },
        Err(_) => false,
    };
    if ranked {
        owe(slot);
    }
    ranked
}

/// Take the maneuver out of the record.
///
/// What is left of a maneuver whose record could not be read: the bit that says
/// one is running is cleared so that nothing steps it again, and the caller has
/// let go of the machine instead of carrying it down.
pub fn abandon(slot: &mut SessionStateWire) {
    slot.winddown_mut().set_active(false);
}

/// What one wake of the maneuver came to.
pub enum Step {
    /// Nothing: no maneuver is running, or the stow it is running is already
    /// out with the tick.
    Nothing,
    /// A maneuver stands in the record and the record does not read back as
    /// one. There is no clock to bound it by and no ending to judge it against,
    /// so it cannot be carried out: the caller lets go of the machine.
    Ungoverned,
    /// The stow was written into the slot as the schedule the machine is under
    /// command to run, and it is the caller's to publish.
    Commanded,
    /// The maneuver is over.
    Concluded(Concluded),
}

/// How a maneuver ended, for the record and for the phase it leaves behind.
pub struct Concluded {
    /// How far it got.
    pub outcome: WindDownOutcome,
    /// Where the machine is left, and whether the next ask may engage it.
    pub disposition: Disposition,
    /// How much of the maneuver's one clock was left when it ended,
    /// nanoseconds. Negative where it ended past its deadline, which is what a
    /// maneuver that ran out of clock rather than reaching the pose looks like.
    pub left_ns: i64,
}

impl Concluded {
    /// The phase a machine this maneuver left behind is in.
    #[must_use]
    pub fn phase(&self) -> SessionPhaseWire {
        match self.disposition {
            Disposition::Rest => SessionPhaseWire::RESTING,
            Disposition::Park => SessionPhaseWire::PARKED,
        }
    }
}

/// Step the maneuver: what the machine has done, then what it is asked for
/// next.
///
/// `stowed` is the one judgement the core cannot make for itself -- whether the
/// machine has reached the pose the stow was driving it to -- taken from the
/// freshest sample the driver published. A stow that reached it is over and the
/// record says the head came down under control; anything else is the clock's
/// to end.
pub fn step(slot: &mut SessionStateWire, now_ns: i64, stowed: bool) -> Step {
    let Ok(now) = duration_from_nanos(now_ns) else {
        return Step::Nothing;
    };
    // A record that stands as a maneuver and describes none: the deadline that
    // bounds every stow lives inside it, so a machine left under command by it
    // would be held toward the fold with no clock to end it. Answered by letting
    // go, which is what every other unreadable record of a machine that may be
    // holding is answered with.
    let stepped = {
        let Ok(state) = slot.winddown_mut().validate_mut() else {
            return Step::Ungoverned;
        };
        let Ok(Some(mut maneuver)) = WindDown::resume(state) else {
            return Step::Ungoverned;
        };
        let evidence = Evidence {
            // The tick's mask is the tick's: nothing published carries it, so
            // the session cannot tell a head with no joint left to drive from
            // one still commanding. False is the conservative reading -- the
            // stow keeps being commanded until the clock ends it, where a wrong
            // `true` would let go of a head that could still have been carried
            // down.
            // TODO(session-mask-view)
            head_released: false,
            // A stow ends when the machine is measured at the pose. A stow the
            // machine defeated is not evidence this host has: what stops a move
            // is a condition, and a condition arrives as a raise and re-ranks
            // the maneuver.
            stow: stowed.then_some(StowEnding::Stowed),
        };
        let deadline = maneuver.deadline();
        (maneuver.next(now, evidence), deadline)
    };

    let (action, deadline) = stepped;
    match action {
        WindDownAction::CommandStow { remaining } => {
            if !owed(slot) {
                return Step::Nothing;
            }
            // A remaining clock that is no count of nanoseconds is not one to
            // publish an instant off: nothing is commanded this wake, the
            // schedule the machine is running stands, and the maneuver's own
            // deadline still ends it.
            let Ok(remaining_ns) = duration_nanos(remaining) else {
                return Step::Nothing;
            };
            command(slot, now_ns, remaining_ns);
            Step::Commanded
        }
        WindDownAction::Conclude {
            outcome,
            disposition,
        } => Step::Concluded(Concluded {
            outcome,
            disposition,
            // What the record holds, and nothing derived from a configured
            // number: the deadline is the one instant the maneuver wrote down,
            // so how much of the clock was left is a subtraction inside the
            // record rather than a claim about when it opened. A deadline that
            // is no count of nanoseconds reads as a clock exactly spent, which
            // is the reading that claims nothing.
            left_ns: duration_nanos(deadline).map_or(0, |ns| ns.saturating_sub(now_ns)),
        }),
    }
}

/// Whether the stow the maneuver is running still has to be asked for.
fn owed(slot: &SessionStateWire) -> bool {
    slot.winddown().commanded_epoch() != slot.schedule().epoch()
}

/// Record that the machine has not been asked for the stow this maneuver runs.
///
/// The epoch before the one the schedule is standing at, which is the one value
/// [`owed`] cannot read as a stow already commanded: nothing else moves the
/// schedule's epoch while a maneuver runs, so one fact with one writer says
/// whether a stow is outstanding, and there is no second flag to keep beside it.
fn owe(slot: &mut SessionStateWire) {
    let epoch = slot.schedule().epoch().wrapping_sub(1);
    slot.winddown_mut().set_commanded_epoch(epoch);
}

/// Write the stow into the slot as the whole of what the machine is under
/// command to do.
///
/// One step, from now until the clock is spent, and no overlay windows: a
/// machine being carried down is presenting nothing. The epoch is bumped
/// because that is what makes the change news to the tick -- a stow commanded
/// again after a condition stopped the last one looks like the schedule it
/// replaces -- and it is recorded in the maneuver so the wakes that follow ask
/// for nothing.
fn command(slot: &mut SessionStateWire, now_ns: i64, remaining_ns: i64) {
    let end_ns = now_ns.saturating_add(remaining_ns);
    let epoch = slot.schedule().epoch().wrapping_add(1);
    {
        let schedule = slot.schedule_mut();
        schedule.set_engaged(true);
        schedule.set_epoch(epoch);
        schedule.overlays_mut().clear();
        let mut steps = schedule.steps_mut();
        steps.clear();
        let row: &mut ScheduledStepWire = steps
            .try_grow()
            .expect("a schedule cleared of its steps holds one");
        row.set_start(SyncTime::from_nanos(now_ns));
        row.set_end(SyncTime::from_nanos(end_ns));
        row.set_kind(StepKindWire::BASE_POSTURE);
        row.set_posture(PostureWire::STOW);
    }
    slot.winddown_mut().set_commanded_epoch(epoch);
}

/// Whether `sample` reads as a machine standing at its stow pose.
///
/// False for a sample that is not a complete reading of every row, and false
/// for one whose angles are not all numbers: an unreadable pose is not evidence
/// that the head is folded, and the conservative answer keeps the stow being
/// commanded until the maneuver's own clock ends it.
///
/// Freshness is not read here and is bounded elsewhere: what the caller hands
/// over is the newest sample the driver published, and a stream that stopped
/// producing them is the freshness watchdog's condition. So the oldest reading
/// this can turn on is one wake old, and the claim it can make wrongly over that
/// window is that a machine measured at the fold a wake ago is still there.
#[must_use]
pub fn stowed(sample: &PoseSample) -> bool {
    if !bool::from(sample.present_valid) || !flags::is_empty(sample.missing) {
        return false;
    }
    let present = vector_of(&sample.present);
    present.first_non_finite().is_none() && at_stow(disarm_config(), &present)
}
