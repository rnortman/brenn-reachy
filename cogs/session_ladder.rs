//! What the session does about evidence: which conditions the driver's own
//! records are evidence of, which response answers one, and the de-torquing the
//! response that trusts nothing commands.
//!
//! Three streams carry evidence about the machine, and the decision tick's
//! raises are only one of them. The driver reports edges the pose stream cannot
//! show -- a de-torquing it latched, a confirmation it read back, a bus it
//! declared gone -- and its rotating read carries each servo's own hardware-error
//! byte. Both become faults here, classified by the same library functions the
//! tick's raises are classified by, so one condition has one response wherever
//! the evidence for it came from.
//!
//! The sample stream is evidence of a different kind: not about the machine but
//! about the driver. A driver cannot report its own death, so the only thing
//! that notices one is a host measuring the gap since the last sample it saw --
//! which is what the two watchdog arms here do, one for a stream that stopped
//! and one for a stream that never started.
//!
//! What the response ladder does about a fault is the doctrine's, and this
//! module is where a response is selected and begun. Two rungs are carried out
//! here and the two stow maneuvers are begun here and run in `session_stow`,
//! which is where a schedule the tick carries the head down with is commanded.
//! The immediate best-effort torque-off is
//! commanded on every wake until the driver says every row let go: nothing gates
//! that -- not the park latch, not a budget running out, not a confirmation that
//! never comes -- and the budget bounds what is *said* about an unconfirmed
//! release rather than how long it is commanded for. The group-scoped de-torque
//! is begun here and drained over the wakes after it, one verified write per
//! antenna, and the session carries on: a pair going limp while the head keeps
//! its presence is a fault answered rather than a session ended.
//!
//! Nothing here reads a clock or holds state of its own: the instant arrives
//! from the caller and everything remembered is in the session's slot.

use brenn_reachy__cogs__config_clk_rs::SessionParams;
use brenn_reachy__cogs__session_clk_rs::{SessionPhaseWire, SessionStateWire};
use brenn_reachy__driver__health_clk_rs::EventKind;
use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointFlagsWire};
use clockwork_rs::SyncTime;
use reachy_motion::fault::{self, FaultKind};
use reachy_motion::joints;
use reachy_motion::tick::ResponseKind;
use reachy_motion::winddown::{Disposition, Maneuver, ending, maneuver_of};

use crate::session_bus::{self, Entered, enter};
use crate::session_stow;

/// The bus cycle the sample stream sits on, nanoseconds.
///
/// The staleness window is a count of nominal periods, and this is the period it
/// counts. A constant rather than a configured key because the session's
/// configuration carries budgets and not the driver's cycle, and the two records
/// are held to agree by the scenario harness, which checks this against the
/// period the simulated driver is built with.
pub const NOMINAL_PERIOD_NS: i64 = 20_000_000;

/// The budgets the ladder spends, copied off the configuration once per
/// execution.
///
/// A plain value rather than the validated view, so the numbers can be read
/// while the slot they are spent on is held open: a view is a borrow of the
/// config message, and every function below writes the state.
#[derive(Clone, Copy)]
pub struct Budgets {
    /// How many nominal periods may pass with no fresh sample.
    pub sample_stale_after: u32,
    /// How long the first sample has to arrive, nanoseconds.
    pub startup_grace_ns: i64,
    /// How long a commanded release has to be confirmed, nanoseconds.
    pub torque_off_confirm_budget_ns: i64,
    /// How long a stow maneuver has, from the instant it opens, nanoseconds.
    pub stow_budget_ns: i64,
}

impl Budgets {
    /// The numbers the ladder spends, off the session's configuration.
    #[must_use]
    pub fn of(params: &SessionParams) -> Self {
        Self {
            sample_stale_after: params.sample_stale_after,
            startup_grace_ns: params.startup_grace_ns,
            torque_off_confirm_budget_ns: params.torque_off_confirm_budget_ns,
            stow_budget_ns: params.stow_budget_ns,
        }
    }
}

/// The condition a driver's own record is evidence of, or `None` where it is
/// evidence of nothing about the machine.
///
/// Two of the driver's edges are conditions: a bus that stopped carrying, and a
/// commanded release no row acknowledged. The rest are either the machine
/// working as designed -- the minimum-risk write it makes at start-up, the
/// confirmation of a release, a goal it dropped or ran late -- or a report about
/// this host's own timing rather than about the servos, which is what a window
/// of cycle measurements is.
///
/// A hold timeout says the goal stream went quiet for longer than the driver
/// waits, and the machine was de-torqued because of it. Nothing here answers it
/// and the fault vocabulary names no such condition, so it is evidence this
/// session reads and does nothing with.
/// TODO(session-hold-timeout-evidence)
#[must_use]
pub fn fault_of_event(kind: EventKind) -> Option<FaultKind> {
    match kind {
        EventKind::BusFailure => Some(FaultKind::BusFailure),
        EventKind::TorqueOffUnconfirmed => Some(FaultKind::TorqueOffUnconfirmed),
        EventKind::None
        | EventKind::HoldTimeoutTorqueOff
        | EventKind::StartupMrcWrite
        | EventKind::TorqueOffConfirmed
        | EventKind::CycleSkipped
        | EventKind::CycleStats
        | EventKind::GoalDroppedQueueFull
        | EventKind::GoalStaleOrOutOfOrder => None,
    }
}

/// A response selected and begun.
pub struct Answered {
    /// The response, for the record.
    pub response: ResponseKind,
    /// The phase it moved the machine to, where it moved it at all.
    pub entered: Option<Entered>,
}

/// Answer a fault, and say what was begun.
///
/// `None` where nothing was begun, which is two different situations and the
/// same answer: a response already standing is the response to a further fault,
/// and a response this host does not carry out yet leaves the fault recorded and
/// nothing else.
///
/// The immediate release's post-state is the response's own -- park for every
/// fault the library classifies this way, rest for the response that names it --
/// and it is entered when the release is *commanded* rather than when it is
/// confirmed: what park means is that nothing engages until an operator has
/// been, and that is true from the instant the machine stopped being
/// trustworthy. The commanding runs on past it.
///
/// A machine already being carried down is answering every further condition
/// with the maneuver it is running: the ladder never begins a second answer, so
/// the condition re-ranks that maneuver and the wake carries on. The one
/// exception is the group-scoped de-torque, which is why it is asked about
/// first: an antenna pair going limp is scoped to the pair and is a fault
/// answered whatever else the machine is doing.
pub fn answer(
    slot: &mut SessionStateWire,
    kind: FaultKind,
    now_ns: i64,
    budgets: &Budgets,
) -> Option<Answered> {
    let response = fault::response(kind);
    // A machine already commanded limp is answering every fault there is: the
    // release takes all nine rows and it goes out on every wake, so nothing is
    // re-commanded and nothing is re-narrated.
    if slot.torque_off_pending() {
        return None;
    }
    // Which maneuver a response runs is the library's question, asked here
    // rather than by reading the response's name: the group-scoped de-torque is
    // the one maneuver a response names and an ending does not.
    let maneuver = maneuver_of(response);
    if !matches!(maneuver, Some(Maneuver::AntennaTorqueOff)) && session_stow::running(slot) {
        // Re-ranked and nothing else: the maneuver's own next step is what acts
        // on a condition that has stopped trusting control, and it runs in this
        // same execution. A second answer would be a second clock over one
        // machine.
        if session_stow::re_rank(slot, kind) {
            return None;
        }
        // Except where the record standing as a maneuver does not read back as
        // one: there is nothing to rank and nothing carrying the machine down,
        // so it is let go of where it stands and the phase latches.
        session_stow::abandon(slot);
        command_release(slot, now_ns, budgets);
        let entered = (slot.phase() != SessionPhaseWire::PARKED)
            .then(|| enter(slot, SessionPhaseWire::PARKED));
        return Some(Answered { response, entered });
    }
    match maneuver {
        Some(Maneuver::AntennaTorqueOff) => begin_degrade(slot, response),
        Some(Maneuver::ImmediateAllTorqueOff) => {
            let to = match response {
                ResponseKind::ImmediateAllTorqueOffToRest => SessionPhaseWire::RESTING,
                // Every fault the library answers with the immediate release is
                // park-class but one, and the rest-class arm above is written
                // for the one.
                _ => SessionPhaseWire::PARKED,
            };
            command_release(slot, now_ns, budgets);
            let entered = (slot.phase() != to).then(|| enter(slot, to));
            Some(Answered { response, entered })
        }
        Some(Maneuver::SlowStow | Maneuver::MaskedSlowStow) => {
            stow(slot, response, now_ns, budgets)
        }
        // `None` is no maneuver at all -- the refusal of an ask, and what a slot
        // that has answered nothing holds.
        None => None,
    }
}

/// Begin the stow maneuver `response` runs, or let go of the machine where it
/// cannot be run at all.
///
/// A stow is a schedule the decision tick carries out, so it can only be run on
/// a machine that is under command. Nothing streams to a machine that is being
/// armed, released, or standing at rest, and commanding a stow there would ask a
/// tick that is not running to carry the head down. What such a machine gets is
/// the release: the condition's own disposition still holds -- a park-class
/// ending latches the phase whether or not the head could be carried down -- and
/// letting go is the answer that needs nothing of a tick.
///
/// A budget that is no length of time is the same case: a maneuver nobody can
/// bound is not one to run under torque.
fn stow(
    slot: &mut SessionStateWire,
    response: ResponseKind,
    now_ns: i64,
    budgets: &Budgets,
) -> Option<Answered> {
    let commanded = matches!(slot.phase(), SessionPhaseWire::ACTIVE);
    if commanded && session_stow::begin(slot, response, now_ns, budgets.stow_budget_ns) {
        return Some(Answered {
            response,
            entered: Some(enter(slot, SessionPhaseWire::WINDING_DOWN)),
        });
    }
    command_release(slot, now_ns, budgets);
    let to = match ending::disposition(ending::answering(response)) {
        Disposition::Rest => SessionPhaseWire::RESTING,
        Disposition::Park => SessionPhaseWire::PARKED,
    };
    let entered = (slot.phase() != to).then(|| enter(slot, to));
    Some(Answered { response, entered })
}

/// Begin the group-scoped de-torque: the antenna pair owes a verified
/// torque-off write, and the wakes after this one carry them out.
///
/// The session carries on. An antenna pair going limp while the head keeps its
/// presence is a fault answered, not a session ended, so no phase is entered
/// and nothing about the schedule changes; the decision tick notices the limp
/// rows through its own tracking evidence and masks them itself.
///
/// A drain already running is the answer to a second antenna complaining: the
/// set it holds is the pair, so there is nothing to add to it and nothing new to
/// say.
fn begin_degrade(slot: &mut SessionStateWire, response: ResponseKind) -> Option<Answered> {
    if owes_degrade(slot) {
        return None;
    }
    slot.set_degrade_release(JointFlagsWire::from(session_bus::degrade_rows()));
    Some(Answered {
        response,
        entered: None,
    })
}

/// Whether a group-scoped de-torque is still being carried out.
///
/// Either a write is outstanding or rows still owe one. Read by the caller to
/// decide whose turn the aux path is, and here to tell a fresh degrade from one
/// already running.
#[must_use]
pub fn owes_degrade(slot: &SessionStateWire) -> bool {
    if slot.degrade_pending() {
        return true;
    }
    !joints::flags::is_empty(
        slot.degrade_release()
            .to_known()
            .expect("the slot reads back as a session"),
    )
}

/// Command the release, and start the budget its confirmation is judged against.
///
/// Every path that leaves the machine untrustworthy while it may be holding
/// converges here: the release means the same thing to the driver regardless of
/// what decided it.
pub fn command_release(slot: &mut SessionStateWire, now_ns: i64, budgets: &Budgets) {
    slot.set_torque_off_pending(true);
    slot.set_torque_off_commanded(SyncTime::from_nanos(now_ns));
    slot.set_torque_off_deadline(SyncTime::from_nanos(
        now_ns.saturating_add(budgets.torque_off_confirm_budget_ns),
    ));
    // Whatever transaction was outstanding is abandoned: the sequence driving it
    // has been overtaken by a machine that is no longer trustworthy, and an
    // answer arriving for it now is an answer to a question nobody is still
    // asking.
    slot.aux_mut().set_active(false);
    // A machine commanded fully limp owes no group-scoped write: the release
    // takes the antennas with everything else, and the driver latches it and
    // sweeps its own read-back rather than waiting on nine verified writes.
    slot.set_degrade_pending(false);
    slot.set_degrade_release(JointFlagsWire::from(JointFlags::NONE));
}

/// Whether this wake owes the driver a release.
///
/// Every wake owes one while it stands, because the datagram is idempotent, the
/// channel is lossy, and nothing gates de-torquing.
#[must_use]
pub fn owes_release(slot: &SessionStateWire) -> bool {
    slot.torque_off_pending()
}

/// The driver confirmed the release: stop commanding it.
///
/// Answers whether one was standing, so a confirmation of a release nobody
/// commanded is not narrated as the end of one.
pub fn confirmed(slot: &mut SessionStateWire) -> bool {
    if !slot.torque_off_pending() {
        return false;
    }
    slot.set_torque_off_pending(false);
    true
}

/// A release still unconfirmed with its budget spent, and how long it has been
/// commanded for.
///
/// The budget bounds the saying and never the commanding: this answers once per
/// budget for as long as the release goes unconfirmed, and the caller keeps
/// publishing either way. Re-arming rather than latching is what makes a
/// machine that never lets go say so more than once, and at the rate a budget
/// was written for rather than at wake rate.
///
/// What is answered is the whole time the release has been commanded for,
/// measured from the instant it was commanded rather than from the deadline that
/// just re-armed: a machine that will not let go has been unconfirmed for longer
/// every time it is said, and a flat series of identical budgets would tell an
/// operator nothing about how long this has been going on.
pub fn overdue_release(slot: &mut SessionStateWire, now_ns: i64, budgets: &Budgets) -> Option<i64> {
    if !slot.torque_off_pending() {
        return None;
    }
    let deadline = slot.torque_off_deadline().as_nanos();
    if now_ns < deadline {
        return None;
    }
    slot.set_torque_off_deadline(SyncTime::from_nanos(
        now_ns.saturating_add(budgets.torque_off_confirm_budget_ns),
    ));
    Some(now_ns.saturating_sub(slot.torque_off_commanded().as_nanos()))
}

/// Note that this execution is the session's first, for the start-up grace to be
/// measured from.
///
/// A flag beside the instant, because a zero instant is a legitimate one under a
/// deterministic runner and a slot nothing wrote holds exactly that.
pub fn note_start(slot: &mut SessionStateWire, now_ns: i64) {
    if slot.started() {
        return;
    }
    slot.set_started(true);
    slot.set_started_at(SyncTime::from_nanos(now_ns));
}

/// Note the freshest sample instant the driver has published.
///
/// The nominal instant and not the arrival: what is being watched is a driver
/// producing its cycle, and the nominal instant is what says which cycle it
/// produced. Answers whether the sample could be read at all.
pub fn note_sample(slot: &mut SessionStateWire, nominal_ns: i64) {
    if slot.saw_sample() && nominal_ns <= slot.last_sample_time().as_nanos() {
        return;
    }
    slot.set_saw_sample(true);
    slot.set_last_sample_time(SyncTime::from_nanos(nominal_ns));
}

/// How long the sample stream has been silent, where that is long enough to
/// declare the bus failed.
///
/// Two arms, and which one applies is whether a sample has ever arrived: a
/// stream that stopped is judged against a count of nominal periods, and one
/// that never started against the start-up grace, because there is no last
/// sample to measure from. Idle once parked -- the machine is latched and the
/// response has been taken, and a second declaration would answer a condition
/// already answered.
#[must_use]
pub fn silent_for(slot: &SessionStateWire, now_ns: i64, budgets: &Budgets) -> Option<i64> {
    if matches!(slot.phase(), SessionPhaseWire::PARKED) {
        return None;
    }
    let (from, window) = if slot.saw_sample() {
        (
            slot.last_sample_time().as_nanos(),
            i64::from(budgets.sample_stale_after).saturating_mul(NOMINAL_PERIOD_NS),
        )
    } else if slot.started() {
        (slot.started_at().as_nanos(), budgets.startup_grace_ns)
    } else {
        return None;
    };
    let silent = now_ns.saturating_sub(from);
    (silent > window).then_some(silent)
}
