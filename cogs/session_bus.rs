//! The session's bus half: driving one sequence over the driver, one
//! transaction at a time.
//!
//! A sequencer in `reachy-motion` is a state machine that yields one abstract
//! transaction and waits for its result. It touches no port, and this is the
//! host that gives it one: the transaction it asks for becomes the datagram the
//! session publishes, the driver's answer becomes the result handed back, and
//! everything the sequence has established in between lives in the session's
//! own slot rather than on a stack.
//!
//! Two retry layers exist here on purpose. This one is *delivery* retry: an
//! unanswered datagram is re-issued verbatim, correlation number included,
//! because the channel it goes out on is in-process memory today and a socket
//! later. Bus-level retry -- a servo that did not answer a frame the driver did
//! put on the wire -- is the driver's, spent before an answer comes back at all,
//! which is what `BusResult::NoAnswer` already means. Re-issuing a datagram
//! nobody answered is delivery, never the banned retry with perturbed inputs:
//! the bytes are the same bytes.
//!
//! One transaction is outstanding at a time, always. The driver refuses a
//! second one loudly rather than queueing it, and the record of the one in
//! flight is the slot's `aux` field, which is exactly what a re-issue needs:
//! the datagram's own fields, when it last went out, and how many times it has.
//!
//! Three sequences are driven here, and which one a wake steps is read off the
//! phase: the survey a process runs once before anything is torqued, the resting
//! watch that re-reads a stale rail row on the way into an engagement, and the
//! orderly release that lets go of the machine when the schedule has run out.
//! All three take the same three steps in the same order -- settle what is
//! outstanding, step once, publish what the step asked for -- and differ only in
//! what their endings mean.
//!
//! One thing driven here is no sequence at all: the group-scoped de-torque, one
//! verified write per wake over the same outstanding-transaction machinery. It
//! is a list of rows rather than a state machine -- there is nothing to decide,
//! only servos to make let go of -- so it settles its own write and the slot
//! says whose answer an outcome is.
//!
//! Taking hold of the machine is no sequence at all, and that is the point. The
//! torque-on gate is arithmetic over the rail picture this module keeps from the
//! driver's health rotation, and what it clears is one datagram: the driver pins
//! the goal registers, enables torque and reads the enable back inside a single
//! cycle of its own, and answers with an event. So the engaging phase's whole
//! bus traffic is that datagram -- and, on the rare wake whose picture has a row
//! two laps old, the two reads the watch spends on it first.

use core::time::Duration;

use brenn_reachy__cogs__schedule_clk_rs::SessionScheduleWire;
use brenn_reachy__cogs__session_clk_rs::{AuxPendingWire, SessionPhaseWire, SessionStateWire};
use brenn_reachy__hardware__dynamixel__registers_clk_rs::{
    RegId, RegIdWire, ValueShape, ValueShapeWire,
};
use brenn_reachy__motion__bus_txn_clk_rs::{AuxOpKind, AuxOpKindWire, BusTxn, BusTxnWire};
use brenn_reachy__motion__commission_clk_rs::CommissionSnapWire;
use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointFlagsWire};
use brenn_reachy__motion__poll_clk_rs::PollSnapWire;
use brenn_reachy__motion__seq_clk_rs::SeqKindWire;
use clockwork_rs::SyncTime;
use reachy_kin::EnvelopeConfig;
use reachy_motion::arm;
use reachy_motion::arm::{
    ArmConfig, CommissionSequencer, EXPECTED_OPERATING_MODES, PollSequencer, ProfileConfig,
    ProvisionExpect, ProvisionTable, Rail, SERVO_IDS, VENDOR_HOMING_OFFSETS, engage_gates,
};
use reachy_motion::cells::{self, RailRecord};
use reachy_motion::disarm::{
    DEFAULT_STOW_DWELL, DEFAULT_STOW_TOLERANCE, DisarmConfig, DisarmSequencer, DisarmSummary,
    stow_targets,
};
use reachy_motion::joints::{self, JointGroup, JointRef, ROW_COUNT, ROWS, ServoHealth, flags};
use reachy_motion::seq::{BusResult, SeqAction, Sequencer};
use reachy_motion::snap::{duration_from_nanos, duration_nanos};
use reachy_motion::tick::default_motion_config;
use reachy_motion::{txn, value};

/// Build the record of the machine this session commissions, if nothing has.
///
/// Built once and shared, the arrangement [`reachy_motion::tick::
/// default_motion_config`] makes for the tick's configuration and for the same
/// reason: the provisioning grid is nine rows of expectations, and a host that
/// rebuilt it on every wake would pay for it at wake rate.
///
/// Every value in it is a hardware fact the motion library states once -- the
/// nine ids, the supply floor, the gains, the servo-side fences drawn in from
/// the envelope's own crank windows -- and not a configuration key: a second
/// copy of any of them somewhere a deployment could edit is two records able to
/// render two verdicts on one truth. So the record itself is the library's
/// [`arm::arm_config`], and what this host supplies is the two things that are
/// not hardware facts: the provisioning grid below, and `profile`.
///
/// The profile is the caller's because it is the one part of this record a
/// deployment chooses: the library states plainly that it has no default for
/// it, since what it should be is a property of the machine a host drives
/// rather than of the motion arithmetic. It arrives from the session's own
/// configuration and is taken on the first call, which is the wake that first
/// needs the record -- built once for the same reason the rest of it is, and
/// therefore fixed for the life of the process at the value the process started
/// with.
///
/// A profile handed in after the first call is ignored rather than refused. The
/// record is process-wide for the life of the binary, tests included: the first
/// call any test case makes fixes the profile for every case after it in the
/// same test binary, so a case that seeded a different pair would be measuring
/// the first one. In a deployment there is only ever one pair to seed -- a cog's
/// config is read from a file before the first execution and nothing writes it
/// afterwards.
///
/// Called at the top of the cog body, which is the whole of how the record is
/// delivered: everything below reaches [`arm_config`] rather than being handed
/// a copy.
///
/// # Panics
///
/// For a profile whose acceleration or velocity is zero. Zero in those two
/// registers is what a servo reads as *no limit*, which is the opposite of the
/// backstop the pair exists to be, and it is what a configuration that lost the
/// two lines parses to. Refusing here stops the process at start-up with the
/// machine de-torqued and nothing commanded, rather than commissioning a
/// machine whose servo-side rate limit is off with nothing said about it.
///
/// And for a zero bus watchdog, which is the register disabled: armed is the
/// policy, so a file that says nothing about it is a session that would command
/// servos with nothing at all watching for a dead driver.
pub fn init_arm_config(profile: ProfileConfig) {
    assert!(
        profile.acceleration > 0 && profile.velocity > 0,
        "the servo profile is {profile:?}, and zero in either register is a servo running \
         unlimited rather than the backstop the pair is for",
    );
    assert!(
        profile.bus_watchdog > 0,
        "the servo profile is {profile:?}, and zero in the watchdog register is a machine that \
         keeps chasing a stale goal after the driver commanding it has died",
    );
    let _ = CONFIGURED
        .get_or_init(|| arm::arm_config(&EnvelopeConfig::default(), provision_table(), profile));
}

/// The record, for the machinery below.
///
/// The one way this module's own functions reach it: the record is a
/// process-lifetime value, so a copy of it threaded through every private
/// function beside the per-execution context would be a second delivery
/// mechanism for one value.
///
/// # Panics
///
/// If nothing initialised it. Unreachable: the cog body's first act is
/// [`init_arm_config`], before a single byte of the slot is read.
fn arm_config() -> &'static ArmConfig {
    CONFIGURED
        .get()
        .expect("the commissioned record, taken from configuration at the top of the execution")
}

/// The record itself, built once for the life of the process.
static CONFIGURED: std::sync::OnceLock<ArmConfig> = std::sync::OnceLock::new();

/// The release this session runs, and what it judges the machine against.
///
/// Every value is the motion library's own: the nine ids, the stow pose derived
/// from the same geometry the stow *move* is planned against, and the tolerance
/// and settle the library states as its defaults. Nothing here is a
/// configuration key, for [`arm_config`]'s reason -- a second copy of the stow
/// pose is two records able to disagree about where folded is.
///
/// The settle is why the keep-alive rule covers this phase: two seconds pass
/// with the machine holding torque and nothing streaming to it, which is an
/// order of magnitude past the driver's hold timeout.
///
/// Public because the stow maneuvers judge a machine folded against this same
/// record: where the release verifies stow and where a wind-down decides it is
/// over must be one pose.
pub fn disarm_config() -> &'static DisarmConfig {
    static CONFIGURED: std::sync::OnceLock<DisarmConfig> = std::sync::OnceLock::new();
    CONFIGURED.get_or_init(|| DisarmConfig {
        ids: SERVO_IDS,
        // The geometry is a constant of the library and stow is inside its
        // envelope, so a machine whose geometry cannot fold is a build that
        // could never have stowed anything.
        stow_targets: stow_targets(&default_motion_config().geom)
            .expect("the configured geometry reaches stow"),
        tolerance: DEFAULT_STOW_TOLERANCE,
        dwell: DEFAULT_STOW_DWELL,
    })
}

/// What a commissioning sweep checks each servo's provisioned registers against.
///
/// Two columns are checked and the rest are read and passed over. The two are
/// the ones this workspace holds a baked expectation for: the operating mode
/// each kind of joint runs in, and the datum -- the homing offset that makes a
/// converted count the model's own crank angle. Both are this project's
/// provisioning rather than the vendor's, recorded in the library as constants,
/// and a servo answering otherwise is a machine that must not be commanded.
///
/// The rest are skipped rather than recorded because a reading nobody compares
/// is a line in a report this host does not print. What the per-unit limits,
/// alarms and delays should hold is bench configuration, carried in the bench's
/// own file beside the unit it was measured on; a number invented here would be
/// a second expectation able to disagree with it.
///
/// Public because the scenario suite counts the survey's transactions from it:
/// how many cells this grid asks to be read is most of that count, and a second
/// spelling of the grid's size somewhere else could disagree with the grid.
pub fn provision_table() -> ProvisionTable {
    let mut table = ProvisionTable::new();
    for (row, joint) in ROWS.into_iter().enumerate() {
        table.set(
            joint,
            RegId::OperatingMode,
            ProvisionExpect::Check(value::u8(EXPECTED_OPERATING_MODES[row])),
        );
        table.set(
            joint,
            RegId::HomingOffset,
            ProvisionExpect::Check(value::i32(VENDOR_HOMING_OFFSETS[row])),
        );
    }
    table
}

/// The delivery budget a transaction is issued under.
#[derive(Clone, Copy)]
pub struct Timing {
    /// How long an answer has to arrive before the datagram goes out again.
    pub aux_timeout_ns: i64,
    /// How many re-issues before the sequence is handed a silence.
    pub aux_retries: u32,
    /// How old a retained rail reading may be and still be gated on.
    pub rail_stale_after_ns: i64,
}

/// A transaction as it crosses out of the sequencer's slot.
///
/// Copied field for field rather than borrowed: the record lives inside the slot
/// the sequencer is holding open, and what is done with it -- written into the
/// pending record, written into the datagram -- writes that same slot. A
/// validated view is a reference into the message it validated, so there is no
/// value form of one to hand across that write.
///
/// Every crossing a transaction makes is field for field for the same reason,
/// and the completeness of each is pinned by test rather than owned by the
/// compiler: this module's own cases carry a record with every field distinct
/// through the read, the pending record and the datagram, and compare what comes
/// out with what went in through the generated comparison, which is every field
/// the schema declares. A record whose fields the pending schema carried whole
/// would put that completeness back in the compiler's hands.
// TODO(aux-pending-carries-bustxn)
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Txn {
    /// Which transaction.
    pub op: AuxOpKind,
    /// Which servo, as its bus id.
    pub id: u8,
    /// Which register, or the no-register zero.
    pub reg: RegId,
    /// What shape the value is.
    pub value_kind: ValueShape,
    /// The value, as eight little-endian bytes.
    pub value: u64,
}

impl Txn {
    /// Write it into the transaction field of a datagram.
    ///
    /// The one place a `BusTxn` is written from one of these, so a datagram and
    /// the pending record it is rebuilt from cannot carry different fields.
    pub fn write(&self, out: &mut BusTxn) {
        out.active = true.into();
        out.op = self.op;
        out.id = self.id;
        out.reg = self.reg;
        out.value_kind = self.value_kind;
        out.value = self.value;
    }

    /// The record a sequencer is waiting on.
    ///
    /// Expected rather than refused: this is read in the execution that has
    /// already established the slot reads back as a session, and the record was
    /// written by the library moments earlier in this same call.
    fn of(txn: &BusTxnWire) -> Self {
        let view = txn.validate().expect("the sequencer built this record");
        Self {
            op: view.op,
            id: view.id,
            reg: view.reg,
            value_kind: view.value_kind,
            value: view.value,
        }
    }

    /// The record the pending slot holds, for a re-issue.
    fn of_pending(pending: &AuxPendingWire) -> Self {
        let view = pending.validate().expect("this cog wrote this record");
        Self {
            op: view.op,
            id: view.id,
            reg: view.reg,
            value_kind: view.value_kind,
            value: view.value,
        }
    }
}

/// One datagram the session owes the driver.
#[derive(Clone, Copy)]
pub struct Datagram {
    /// The correlation number the answer will carry back.
    pub corr: u32,
    /// The transaction to run.
    pub txn: Txn,
}

/// A transaction given up on: the retry budget is spent and the sequence is
/// being handed a silence.
#[derive(Clone, Copy)]
pub struct GaveUp {
    /// The number nothing answered.
    pub corr: u32,
    /// The servo it was addressed to.
    pub id: u8,
    /// How long delivery was attempted for in total, nanoseconds.
    pub waited_ns: i64,
}

/// A phase entered, and the one left.
#[derive(Clone, Copy)]
pub struct Entered {
    /// The phase now.
    pub to: SessionPhaseWire,
    /// The phase before.
    pub from: SessionPhaseWire,
}

/// What one wake's turn at the bus came to.
///
/// The slot is written here; what a story is made of is the caller's, because
/// the ring and the counters are its. Every field is what happened, never what
/// to do next: a turn that owes a datagram says so, and publishing it is one
/// statement in the caller.
pub struct Turn {
    /// What the wake's one transaction came to.
    pub delivery: Delivery,
    /// The phase this turn moved to.
    pub entered: Option<Entered>,
    /// Whether the machine must be taken limp: an engagement stopped with
    /// servos possibly holding and nothing driving them.
    pub release: bool,
    /// A servo whose torque-off write went unacknowledged, where the release
    /// found one. The write was made and read back and nothing came back to say
    /// it landed, which is a servo that may still be holding: the caller answers
    /// it as a condition of the machine.
    pub unreleased: Option<JointRef>,
    /// What the release found, on the wake the session ended.
    pub ended: Option<Ended>,
    /// The rows to take hold of, where this turn asked the driver for an
    /// engagement. Every row the torque-on gate passed; the caller publishes the
    /// one datagram that asks for them.
    pub engage: Option<JointFlags>,
    /// Whether this turn rested an engagement the driver never answered. The
    /// caller counts it: four things decline an engagement and the timeline
    /// tells them apart by nothing yet.
    pub silenced: bool,
}

/// What the orderly release measured before torque came off.
///
/// Reported and never acted on: torque comes off whatever the measurement says,
/// and where the head was at the moment it did is the one thing a session's last
/// word can carry.
#[derive(Clone, Copy)]
pub struct Ended {
    /// The joints the release looked at and could not read. Empty for a release
    /// that measured all nine.
    pub unmeasured: JointFlags,
    /// How far the joint furthest from its stow angle was, radians. Zero where
    /// nothing was measured, which `unmeasured` is what distinguishes.
    pub deviation: f64,
}

/// What one wake's delivery of one transaction came to.
///
/// The half every kind of turn at the bus shares: a sequence's step and the
/// group-scoped de-torque's drain both settle the one transaction outstanding
/// and ask for at most one more, so the delivery contract is one vocabulary and
/// neither of them transposes it into a second.
pub struct Delivery {
    /// The datagram to publish, if this wake owes one.
    pub datagram: Option<Datagram>,
    /// The transaction whose delivery was given up on.
    pub gave_up: Option<GaveUp>,
    /// Whether the datagram above is a re-issue rather than a fresh ask.
    pub retried: bool,
}

impl Delivery {
    /// A wake that asked for nothing.
    fn quiet() -> Self {
        Self {
            datagram: None,
            gave_up: None,
            retried: false,
        }
    }
}

impl Turn {
    /// A turn that asked for nothing and changed nothing.
    fn quiet() -> Self {
        Self {
            delivery: Delivery::quiet(),
            entered: None,
            release: false,
            unreleased: None,
            ended: None,
            engage: None,
            silenced: false,
        }
    }
}

/// Take one turn at the bus: settle the outstanding transaction, then step the
/// sequence.
///
/// `answer` is the driver's reply to the request the slot is waiting on, matched
/// by correlation number and classified before it arrives here. `None` is a wake
/// with nothing to settle -- a lapse, a script, a raise -- which is when a
/// timeout is noticed and a re-issue goes out.
///
/// The order is the safety order and not a convenience: the answer in hand is
/// what the sequence's next decision is made from, so nothing new is asked for
/// until the last thing asked for is resolved either way.
pub fn run(
    slot: &mut SessionStateWire,
    answer: Option<BusResult>,
    now_ns: i64,
    timing: &Timing,
) -> Turn {
    let mut turn = Turn::quiet();
    let Some(live) = live(slot) else {
        return turn;
    };
    // Converted once, before anything is settled or asked for: a count that is
    // no length of time is a clock this host cannot reason about, and a wake
    // that cannot place itself in time asks the machine for nothing.
    let Ok(since_epoch) = duration_from_nanos(now_ns) else {
        return turn;
    };
    let now = Now {
        ns: now_ns,
        since_epoch,
    };

    let prior = match settle(slot, answer, now_ns, timing, &mut turn.delivery) {
        Settled::Answered(result) => Some(result),
        Settled::Waiting => return turn,
        Settled::Fresh => None,
    };

    // A sequence that asked to be woken later is not stepped before then: the
    // supply gate spaces its reads out because the servos refresh their own
    // voltage reading ten times a second, and stepping it early would read the
    // same number twice and spend the budget it is polling against.
    if prior.is_none() && now_ns < slot.wait_deadline().as_nanos() {
        return turn;
    }

    match live {
        Live::Commission => commission(slot, prior, now, &mut turn),
        Live::Engage => engage(slot, prior, now, timing, &mut turn),
        Live::Disarm => letting_go(slot, prior, now, &mut turn),
    }
    turn
}

/// Take an engagement's first bus step, outside a wake's ordinary turn.
///
/// The `Live::Engage` arm entered with no prior answer -- the same code [`run`]
/// dispatches to -- for the one caller that reaches `ENGAGING` after this wake's
/// turn at the bus has already been taken: the drain that applies a script held
/// across a maneuver's ending. The engagement's first step is the gate and then
/// either the `engage_now` or the fallback poll's first read, and readiness is
/// the whole requirement, so it goes out on the wake the phase moved rather than
/// on the next one.
///
/// Nothing may be outstanding when this is called: the wake's turn is what would
/// have left something, and the drain reaches here only on a wake whose turn
/// issued nothing at all.
pub fn open_engagement(slot: &mut SessionStateWire, now_ns: i64, timing: &Timing) -> Turn {
    debug_assert!(
        !slot.aux().active(),
        "an engagement's first step is taken on a wake that issued nothing",
    );
    let mut turn = Turn::quiet();
    let Ok(since_epoch) = duration_from_nanos(now_ns) else {
        return turn;
    };
    engage(
        slot,
        None,
        Now {
            ns: now_ns,
            since_epoch,
        },
        timing,
        &mut turn,
    );
    turn
}

/// The schedule has run out: the session is over, and the machine is to be let
/// go of.
///
/// `None` unless the machine is under command and the last thing its schedule
/// asks for has passed. The windows are tested beside the steps, because an
/// overlay may outlive the step it played over and a stream that stopped at the
/// last step's edge would cut a motion off mid-play; a schedule that asks for
/// nothing at all has nothing to wait for and ends on the wake it is engaged on.
///
/// A phase change and no datagram, so it costs the wake nothing: the release's
/// own first step is taken by the same execution.
pub fn ended(slot: &mut SessionStateWire, now_ns: i64) -> Option<Entered> {
    if !matches!(slot.phase(), SessionPhaseWire::ACTIVE) {
        return None;
    }
    // The ends are exclusive, so an instant at one is an instant the interval no
    // longer owns.
    match last_instant(slot.schedule()) {
        Some(end) if now_ns < end => None,
        _ => Some(enter(slot, SessionPhaseWire::STOPPING)),
    }
}

/// The last instant `schedule` owns, or `None` for a schedule that asks for
/// nothing at all.
///
/// The end of its last step or its last overlay window, whichever is later, and
/// the one place that is worked out: a schedule is over when nothing in it has a
/// future, and a row kind the vocabulary grows has to be answered here once
/// rather than wherever a caller happened to scan. The ends are exclusive.
pub fn last_instant(schedule: &SessionScheduleWire) -> Option<i64> {
    schedule
        .steps()
        .iter()
        .map(|step| step.end().as_nanos())
        .chain(
            schedule
                .overlays()
                .iter()
                .map(|window| window.end().as_nanos()),
        )
        .max()
}

/// Which sequence the phase this slot is in drives, or that it drives none.
///
/// The phase is the whole of it: an engaging machine is judging its torque-on
/// gate and asking the driver to take hold, and the resting watch it may fall to
/// on the way is a step of that arm rather than a phase of its own. Resting,
/// active and the two endings drive nothing over the bus.
///
/// A starting session drives nothing until the driver has published a sample it
/// could read. A transaction issued before then goes to a process whose loop has
/// not begun, where it waits in a socket buffer with nobody serving it: the
/// delivery timeout expires, the re-issue lands beside the original in the
/// driver's first cycle, and what comes back says nothing this session can tell
/// a decline from. The sample is the evidence the driver is cycling, and this
/// cog already keeps it for the silence it measures. A driver that never
/// publishes one is the start-up grace's business, which is where the machine
/// this session cannot commission is declared absent.
fn live(slot: &SessionStateWire) -> Option<Live> {
    match slot.phase() {
        SessionPhaseWire::STARTING => slot.saw_sample().then_some(Live::Commission),
        SessionPhaseWire::ENGAGING => Some(Live::Engage),
        SessionPhaseWire::STOPPING => Some(Live::Disarm),
        _ => None,
    }
}

/// The instant a wake is at, in the two forms it is needed in.
///
/// The count the slot keeps and the length of time the sequencers measure in,
/// converted once at [`run`]'s boundary: a conversion in the middle of the
/// machinery is a second place for a clock to go wrong, and the one in the
/// middle is the one that goes wrong silently.
#[derive(Clone, Copy)]
struct Now {
    /// The count.
    ns: i64,
    /// The same instant, as a sequencer reads it.
    since_epoch: Duration,
}

/// What a step did to the sequence it stepped, where it ended one.
///
/// Only the endings, because the two asks -- a transaction, a wake later on --
/// are [`paced`]'s and mean the same thing in every sequence. What an ending
/// means is the whole of what the four differ in.
enum Ending<T> {
    /// It reached its summary.
    Done(T),
    /// It stopped, and the summary it would have had does not exist.
    Failed,
}

/// Pace what one step asked for, and hand back its ending if it ended.
///
/// The half every sequence shares, stated once: at most one transaction goes out
/// per wake and its record is written before the datagram exists, the kind is
/// recorded so the next wake resumes this same sequence, a wake asked for later
/// is noted as a deadline, and a sequence that ended leaves nothing behind for
/// the next one to trip over. A rule fixed in one copy of this and missed in
/// another is a transaction recorded outstanding that never went out.
fn paced<T>(
    slot: &mut SessionStateWire,
    kind: SeqKindWire,
    action: SeqAction<T>,
    now: Now,
    delivery: &mut Delivery,
) -> Option<Ending<T>> {
    match action {
        SeqAction::Transact => {
            slot.set_seq_kind(kind);
            let txn = pending_of(slot, kind);
            issue(slot, txn, now.ns, delivery);
            None
        }
        SeqAction::Wait { until } => {
            slot.set_seq_kind(kind);
            park(slot, until);
            None
        }
        SeqAction::Done(summary) => {
            conclude(slot);
            Some(Ending::Done(summary))
        }
        SeqAction::Fail(_) => {
            conclude(slot);
            Some(Ending::Failed)
        }
    }
}

/// The transaction the sequence of `kind` is asking for, off its own snapshot.
fn pending_of(slot: &SessionStateWire, kind: SeqKindWire) -> Txn {
    match kind {
        SeqKindWire::COMMISSION => Txn::of(slot.commission().pending()),
        SeqKindWire::POLL => Txn::of(slot.poll().pending()),
        _ => Txn::of(slot.disarm().pending()),
    }
}

/// Let go of a machine whose own record of a sequence no longer reads as one.
///
/// The de-torquing that needs no answer from the slot, and the latch: how far a
/// sequence got is exactly what such a record has stopped saying, and a machine
/// that may be holding torque with nothing driving it is not one to reason
/// further about. Nothing engages it again until an operator has been.
fn let_go(slot: &mut SessionStateWire, turn: &mut Turn) {
    conclude(slot);
    turn.release = true;
    turn.entered = Some(enter(slot, SessionPhaseWire::PARKED));
}

/// The sequence a wake steps.
enum Live {
    /// The survey, before anything is torqued.
    Commission,
    /// The engagement: the torque-on gate, the rail re-read it may fall to, and
    /// the one datagram that asks the driver to take hold.
    Engage,
    /// The orderly release that lets go of it.
    Disarm,
}

/// Step the survey.
///
/// Its two endings are the whole of the starting phase: a machine established as
/// the one this process was configured for, or one that must not be commanded.
fn commission(slot: &mut SessionStateWire, prior: Option<BusResult>, now: Now, turn: &mut Turn) {
    let fresh = !matches!(slot.seq_kind(), SeqKindWire::COMMISSION);
    let action = {
        let snap = slot.commission_mut();
        act(snap, fresh, now.since_epoch, prior.as_ref())
    };

    let entered = match paced(
        slot,
        SeqKindWire::COMMISSION,
        action,
        now,
        &mut turn.delivery,
    ) {
        None => return,
        // The survey established the machine is the one this process was
        // configured for. Nothing was torqued to get here and nothing is now:
        // the machine is limp, commissioned, and waiting to be asked for
        // something.
        //
        // Its own rail readings seed the picture the torque-on gate judges, so
        // the machine is never at rest with an empty one: the first wake after
        // this gates on numbers this survey read rather than waiting on the
        // driver's rotation to lap.
        Some(Ending::Done(summary)) => {
            merge_rail(slot, &summary.rail, flags::all(), now.ns);
            SessionPhaseWire::RESTING
        }
        // The survey refused the machine. Nothing was torqued, so there is no
        // maneuver to run and nothing to make safe: the verdict stays in the
        // snapshot, the phase latches, and only an operator restarting the
        // process clears it -- no automatic recovery, ever.
        //
        // The verdict stays in the snapshot; its headline is narrated by the
        // caller, immediately before the phase row -- the caller owns the report
        // ring, and nothing here publishes.
        Some(Ending::Failed) => SessionPhaseWire::PARKED,
    };
    turn.entered = Some(enter(slot, entered));
}

/// Take the engagement's turn: the silence deadline, the rail re-read it may
/// have fallen to, and the torque-on gate.
///
/// The whole of what the wake path does over the bus, and by design it is
/// arithmetic in the ordinary case: the gate judges the rail picture the session
/// keeps from the driver's health rotation, and what it clears is one datagram
/// asking the driver to take hold of the rows it passed. The driver answers with
/// one of three events, which the caller reads.
///
/// The order is the safety order. The deadline first, because a driver that has
/// answered nothing is one to stop asking rather than one to ask again; then the
/// re-read, because a row nobody has looked at for two laps of the rotation is
/// not a row to gate on; then the gate, whose refusal is the script declined
/// with nothing written in either direction. The re-read and the gate belong to
/// the first ask alone: once the datagram is on the wire the gate has passed and
/// the wake's whole business is asking again.
///
/// A wake past the gate re-issues the same `engage_now`, over the rows the gate
/// passed. That is liveness to the driver and draws the same answer again, which
/// is what makes a lost answer heal on the next wake -- and it is why nothing
/// here is ever held torqued on the strength of a silence: the driver's own
/// dead-man takes torque off a machine whose session has stopped re-issuing.
fn engage(
    slot: &mut SessionStateWire,
    prior: Option<BusResult>,
    now: Now,
    timing: &Timing,
    turn: &mut Turn,
) {
    // The driver answered none of the re-issues inside the delivery window.
    // Nothing is held torqued on the strength of a silence: the session stops
    // asking and rests, and the driver's own dead-man takes torque off a machine
    // whose session has stopped feeding it.
    //
    // Which of the four reasons declined an engagement -- this silence, the
    // gate, the fallback sweep, the driver's own `engage_declined` -- is a
    // report kind that does not exist, so what distinguishes this one is the
    // count the caller keeps.
    // TODO(engagement-declined-narration)
    if silent_too_long(slot, now.ns, timing) {
        debug_assert!(
            !matches!(slot.seq_kind(), SeqKindWire::POLL),
            "the deadline measures asking, and a sweep runs only before the first ask",
        );
        slot.set_engage_pending(false);
        turn.silenced = true;
        turn.entered = Some(enter(slot, SessionPhaseWire::RESTING));
        return;
    }

    if matches!(slot.seq_kind(), SeqKindWire::POLL) {
        let action = {
            let snap = slot.poll_mut();
            watch(snap, false, now.since_epoch, prior.as_ref())
        };
        match paced(slot, SeqKindWire::POLL, action, now, &mut turn.delivery) {
            // The sweep is mid-flight: its transaction is this wake's datagram.
            None => return,
            // Only the rows the sweep was named for: it hands back zero for
            // every other row, and a row nobody read is not a row to write a
            // reading for.
            Some(Ending::Done(rail)) => {
                let read = swept_rows(slot);
                merge_rail(slot, &rail, read, now.ns);
            }
            // Nothing was written in either direction -- a watch reads -- so
            // there is nothing to undo, and a rail nobody could read is a
            // reason to decline the script rather than a condition of the
            // machine.
            Some(Ending::Failed) => {
                slot.set_engage_pending(false);
                turn.entered = Some(enter(slot, SessionPhaseWire::RESTING));
                return;
            }
        }
    }

    // An ask already on the wire is asked again verbatim, and nothing about the
    // picture is looked at twice. The gate passed for these rows and the driver
    // may be holding torque on the strength of it: the re-issue is the only
    // thing feeding its dead-man, so a sweep started here would stop the
    // datagram for as long as it ran while the deadline kept running from the
    // first issue, and a gate re-judged over a picture that has since aged out
    // would rest a session whose machine is holding. A reading that says
    // something is wrong with a servo is a condition, and the ladder answers it
    // wherever the session is.
    if slot.engage_issued().as_nanos() != 0 {
        turn.engage = slot.engage_rows().to_known();
        return;
    }

    let (picture, stale) = rail_picture(slot, now.ns, timing.rail_stale_after_ns);
    if !flags::is_empty(stale) {
        let action = {
            let snap = slot.poll_mut();
            start_watch(snap, stale, now.since_epoch)
        };
        paced(slot, SeqKindWire::POLL, action, now, &mut turn.delivery);
        return;
    }

    match engage_gates(arm_config(), &picture) {
        // Nothing was torqued, so the machine is where it was and the phase is
        // the whole of what the record carries: why the engagement was declined
        // is a report kind that does not exist.
        // TODO(engagement-declined-narration)
        Err(_) => {
            slot.set_engage_pending(false);
            turn.entered = Some(enter(slot, SessionPhaseWire::RESTING));
        }
        Ok(degraded) => {
            let rows = flags::without(flags::all(), degraded);
            slot.set_engage_rows(JointFlagsWire::from(rows));
            slot.set_engage_issued(SyncTime::from_nanos(now.ns));
            turn.engage = Some(rows);
        }
    }
}

/// Whether an engagement asked for has gone unanswered past its whole delivery
/// budget.
///
/// Measured from the first issue and not from the last, because every wake
/// re-issues: a deadline that moved with the re-issues would never arrive. What
/// it bounds is the asking. The machine is not holding anything on the strength
/// of this silence -- the driver either wrote torque and said so, or wrote
/// nothing -- and a driver that wrote torque and lost its answer has its own
/// dead-man to take it off once the re-issues stop.
fn silent_too_long(slot: &SessionStateWire, now_ns: i64, timing: &Timing) -> bool {
    let issued = slot.engage_issued().as_nanos();
    if issued == 0 {
        return false;
    }
    let budget = timing
        .aux_timeout_ns
        .saturating_mul(i64::from(timing.aux_retries) + 1);
    now_ns.saturating_sub(issued) >= budget
}

/// Start the resting watch over `rows` and ask for its first read.
fn start_watch(
    snap: &mut PollSnapWire,
    rows: JointFlags,
    now: Duration,
) -> SeqAction<reachy_motion::arm::Rail> {
    let mut seq = PollSequencer::start(arm_config(), snap, rows);
    // A fresh sequence is told nothing, whatever answer was in hand: the
    // sequencer contract is that the first call carries no prior, and an answer
    // to the old sequence's transaction is an answer to a question this one
    // never asked.
    seq.next(now, None)
}

/// Step the orderly release: settle, measure, let go.
///
/// The session's ordinary ending. The settle is waited out under held torque
/// with nothing streaming -- the keep-alive rule is what carries it -- then every
/// joint is measured against the stow pose, and then torque is written off one
/// servo at a time with each write read back. The order is what makes the
/// measurement mean anything: it describes where the head was at the moment
/// torque left it.
///
/// It has one ending. A release does not fail: torque comes off whatever any
/// phase of it found, and what a measurement it could not take or a write nobody
/// acknowledged produces is a summary that says so. Two things the caller may do
/// about that summary: a servo that never acknowledged its release may still be
/// holding, which is a condition of the machine; anything else is the session
/// ended and the machine at rest.
///
/// A snapshot that will not read back as a release is answered by commanding the
/// de-torquing outright and latching, the engagement's rule and for a sharper
/// reason: how far a release got is exactly what such a slot no longer says, and
/// the command that needs no answer from it is the one that reaches the machine
/// fastest.
fn letting_go(slot: &mut SessionStateWire, prior: Option<BusResult>, now: Now, turn: &mut Turn) {
    let fresh = !matches!(slot.seq_kind(), SeqKindWire::DISARM);
    let stepped = {
        let snap = slot.disarm_mut();
        if fresh {
            // A fresh sequence is told nothing, whatever answer was in hand:
            // `act`'s rule, for `act`'s reason.
            Some(DisarmSequencer::start(disarm_config(), snap).next(now.since_epoch, None))
        } else {
            match snap.validate_mut() {
                Ok(state) => match DisarmSequencer::resume(disarm_config(), state) {
                    Ok(mut seq) => Some(seq.next(now.since_epoch, prior.as_ref())),
                    Err(_) => None,
                },
                Err(_) => None,
            }
        }
    };

    let Some(action) = stepped else {
        let_go(slot, turn);
        return;
    };

    match paced(slot, SeqKindWire::DISARM, action, now, &mut turn.delivery) {
        None => {}
        Some(Ending::Done(summary)) => conclude_release(slot, &summary, turn),
        // A release yields no failure, and this arm is what the safe answer
        // would be if one were ever added: let go of everything and latch,
        // rather than reporting a session that ended well.
        Some(Ending::Failed) => let_go(slot, turn),
    }
}

/// Act on what the release found.
///
/// A servo that did not acknowledge its own torque-off outranks everything else
/// in the summary: the machine may be holding, so the session does not get to
/// end at rest, and the caller answers it as the condition it is. Otherwise the
/// session is over and the next accepted script is a new engagement -- the
/// doctrine's rest, which is a session that finished rather than a machine that
/// was cleared of anything.
fn conclude_release(slot: &mut SessionStateWire, summary: &DisarmSummary, turn: &mut Turn) {
    if let Some(joint) = summary.unreleased().next() {
        turn.unreleased = Some(joint);
        return;
    }
    let mut unmeasured = JointFlags::NONE;
    for (joint, _) in summary.unreadable() {
        flags::insert(&mut unmeasured, joint);
    }
    turn.ended = Some(Ended {
        unmeasured,
        deviation: summary.worst_deviation().1,
    });
    turn.entered = Some(enter(slot, SessionPhaseWire::RESTING));
}

/// Issue the transaction a step asked for: a fresh number, the record a re-issue
/// is rebuilt from, and the datagram.
fn issue(slot: &mut SessionStateWire, txn: Txn, now_ns: i64, delivery: &mut Delivery) {
    let corr = slot.next_corr();
    slot.set_next_corr(corr.wrapping_add(1));
    record_pending(slot, corr, txn, now_ns);
    delivery.datagram = Some(Datagram { corr, txn });
}

/// Note the instant a step asked to be woken at again.
///
/// A deadline that is no count of nanoseconds is not waited for at all: the
/// wake floor brings the sequence back and it decides again, where a saturated
/// instant would hold its next transaction back for the life of the process.
fn park(slot: &mut SessionStateWire, until: Duration) {
    slot.set_wait_deadline(SyncTime::from_nanos(duration_nanos(until).unwrap_or(0)));
}

/// A sequence has ended: no sequence is live, and nothing is waiting on a clock.
///
/// The deadline is cleared as well as the kind, because it is the whole slot's
/// and not the ended sequence's: one left standing in the future by a gate that
/// finished would hold the *next* sequence's first transaction back until it
/// passed.
fn conclude(slot: &mut SessionStateWire) {
    slot.set_seq_kind(SeqKindWire::NONE);
    slot.set_wait_deadline(SyncTime::from_nanos(0));
}

/// Whether this wake owes the driver a keep-alive: the machine may be holding
/// torque and nothing else is streaming to it.
///
/// The driver de-torques a machine nobody has spoken to for the hold timeout,
/// which is what makes an unattended arming safe and is exactly what must not
/// fire while an arming is in progress. Every accepted datagram is liveness, so
/// a wake that already owes one needs no keep-alive -- the caller publishes at
/// most one message either way.
///
/// One window is covered, and it is the one in which nothing streams to a
/// machine that may be holding: the whole of the release, whose two-second
/// settle sits under held torque with nothing commanding the machine at all and
/// is an order of magnitude past the hold timeout. It closes when the release
/// writes conclude, because concluding is what leaves the phase.
///
/// The engagement is deliberately not covered. Torque comes on inside one driver
/// cycle and the session's re-issued `engage_now` is what feeds the dead-man
/// until the answer arrives; a keep-alive here would either say what the
/// re-issue already says or hold the dead-man off a machine the session has
/// stopped asking about.
///
/// A machine under command is deliberately not covered. The decision tick
/// streams a goal per sample while a schedule is running -- a holding machine
/// still publishes -- and every accepted goal is liveness, so a keep-alive here
/// would buy nothing and cost the one coverage this rule must never weaken: a
/// stream that stopped while the schedule ran would go unnoticed by the driver,
/// and its hold timeout is what takes torque off a machine whose commander has
/// gone away.
#[must_use]
pub fn keep_alive_owed(slot: &SessionStateWire) -> bool {
    match slot.phase() {
        // Nothing is owed while the machine is being taken hold of. Before the
        // gate nothing is torqued, so there is no dead-man to hold off; after it
        // the re-issued `engage_now` is the liveness, and every wake that owes
        // one publishes it.
        SessionPhaseWire::ENGAGING => false,
        SessionPhaseWire::STOPPING => true,
        _ => false,
    }
}

/// The rows a degrade lets go of: the antenna pair.
///
/// The pair and not the one servo that complained, because the maneuver is the
/// pair going limp -- an antenna still holding beside a dead one is a machine
/// half-presenting, and the doctrine scopes this response to the group.
#[must_use]
pub fn degrade_rows() -> JointFlags {
    JointGroup::Antennas.joints()
}

/// What one wake's turn at draining a group-scoped de-torque came to.
///
/// Like [`Turn`], every field is what happened rather than what to do next: the
/// caller publishes the datagram and tells the story.
pub struct Drain {
    /// What the wake's one write came to.
    pub delivery: Delivery,
    /// The row whose write did not come back verified. A row this session
    /// cannot make let go is a machine to stop trusting, which is the caller's
    /// to answer.
    pub refused: Option<JointRef>,
    /// The rows released, on the wake the last of them let go.
    pub released: Option<JointFlags>,
}

impl Drain {
    /// A wake that asked for nothing and released nothing.
    fn quiet() -> Self {
        Self {
            delivery: Delivery::quiet(),
            refused: None,
            released: None,
        }
    }
}

/// Drain one row of the group-scoped de-torque: settle the write outstanding,
/// then ask for the next.
///
/// One verified `TorqueEnable = 0` per wake, addressed to one antenna, until the
/// set is empty. Verified because a de-torque nobody read back is a de-torque
/// nobody knows happened, which is the one thing this maneuver has to
/// establish; one at a time because the bus is unicast and the driver refuses a
/// second outstanding transaction.
///
/// The answer that settles a write is the driver's word about that row, matched
/// by correlation number before it arrives here. Delivery is retried exactly as
/// a sequence's is -- the same datagram under the same number -- and a delivery
/// given up on is a row that did not let go, which the caller answers as a
/// condition of the machine rather than by asking again.
pub fn degrade(
    slot: &mut SessionStateWire,
    answer: Option<BusResult>,
    now_ns: i64,
    timing: &Timing,
) -> Drain {
    let mut drain = Drain::quiet();
    if slot.degrade_pending() {
        match settle(slot, answer, now_ns, timing, &mut drain.delivery) {
            Settled::Answered(result) => {
                slot.set_degrade_pending(false);
                let joint = addressed(slot);
                // Anything but a verified write leaves the row where it was: a
                // servo that answered nothing, answered wrongly, read back
                // something else, or only acknowledged the instruction is a
                // servo whose torque state this session does not know.
                // `BusResult::Written` is the read-back and nothing else, so an
                // acknowledgement never reaches this test as a success.
                //
                // A record naming an id this configuration does not have is
                // answered the same way however the write came back: there is no
                // bit to take out of the set for it, so a drain that read it as
                // released would ask for the same row every wake for as long as
                // the process ran.
                if matches!(joint, JointRef::None) || !matches!(result, BusResult::Written) {
                    drain.refused = Some(joint);
                    return drain;
                }
                let left = flags::without(owed(slot), flags::bit(joint));
                slot.set_degrade_release(JointFlagsWire::from(left));
                if flags::is_empty(left) {
                    // Every row the drain began with, because a row leaves the
                    // set only by letting go: a refusal returns above instead.
                    drain.released = Some(degrade_rows());
                    return drain;
                }
            }
            Settled::Waiting => return drain,
            // The record was cleared by something else while the drain believed
            // it held one. Nothing is outstanding, so the next row is asked for
            // below.
            Settled::Fresh => slot.set_degrade_pending(false),
        }
    }

    let Some(joint) = flags::iter(owed(slot)).next() else {
        return drain;
    };
    // The set holds servos, and every servo the vocabulary names has a bus row.
    let Some(row) = joints::row(joint) else {
        return drain;
    };
    let mut wire = BusTxnWire::new();
    txn::set_write_reg_verified(
        wire.clear_valid(),
        SERVO_IDS[row],
        RegId::TorqueEnable,
        value::u8(0),
    );
    issue(slot, Txn::of(&wire), now_ns, &mut drain.delivery);
    slot.set_degrade_pending(true);
    drain
}

/// The rows the drain still owes a write.
fn owed(slot: &SessionStateWire) -> JointFlags {
    slot.degrade_release()
        .to_known()
        .expect("the slot reads back as a session")
}

/// The servo the outstanding write is addressed to, or that it is addressed to
/// none.
///
/// [`JointRef::None`] where the record names an id this configuration does not
/// have, which is memory gone wrong rather than an answer about a servo: the
/// caller answers it as a row that would not let go, which is the safe reading
/// of a record it cannot place.
fn addressed(slot: &SessionStateWire) -> JointRef {
    arm::row_of_id(slot.aux().id())
        .and_then(joints::joint_ref)
        .unwrap_or(JointRef::None)
}

/// What became of the transaction the slot was waiting on.
enum Settled {
    /// It came back, or its delivery was given up on, and this is what the
    /// sequence is handed.
    Answered(BusResult),
    /// It is still outstanding: inside its window, or re-issued just now.
    Waiting,
    /// Nothing was outstanding.
    Fresh,
}

/// Resolve the outstanding transaction, if there is one.
///
/// Three ways out, in this order: the answer arrived, the window closed and the
/// datagram goes out again, or the budget is spent and the sequence is handed
/// the silence to classify. The classification stays in the library --
/// exhausted delivery is `NoAnswer` and the sequencer decides what that means
/// for the phase it is in, which is where every other bus failure is decided
/// too.
fn settle(
    slot: &mut SessionStateWire,
    answer: Option<BusResult>,
    now_ns: i64,
    timing: &Timing,
    delivery: &mut Delivery,
) -> Settled {
    if !slot.aux().active() {
        return Settled::Fresh;
    }
    if let Some(result) = answer {
        clear_pending(slot);
        return Settled::Answered(result);
    }

    let issued = slot.aux().issued().as_nanos();
    if now_ns.saturating_sub(issued) <= timing.aux_timeout_ns {
        return Settled::Waiting;
    }

    let retries = slot.aux().retries();
    if retries < timing.aux_retries {
        // The same datagram, under the same number: what makes delivery retry
        // safe is that a driver which answered the first one and lost the answer
        // is being asked the same question, so a late duplicate is recognisable
        // and a servo is never asked twice for two different things.
        let txn = Txn::of_pending(slot.aux());
        let corr = slot.aux().corr();
        let pending = slot.aux_mut();
        pending.set_retries(retries + 1);
        pending.set_issued(SyncTime::from_nanos(now_ns));
        delivery.retried = true;
        delivery.datagram = Some(Datagram { corr, txn });
        return Settled::Waiting;
    }

    delivery.gave_up = Some(GaveUp {
        corr: slot.aux().corr(),
        id: slot.aux().id(),
        // Every window that was waited out, the one that just closed included:
        // what was given up on is the delivery, and its cost is the whole of it.
        waited_ns: (i64::from(retries) + 1).saturating_mul(timing.aux_timeout_ns),
    });
    clear_pending(slot);
    Settled::Answered(BusResult::NoAnswer)
}

/// Forget the outstanding transaction.
///
/// The active flag alone: every other field means nothing while it is false, and
/// blanking them would make a slot mid-flight and a slot at rest indistinguish-
/// able in a dump of the state.
fn clear_pending(slot: &mut SessionStateWire) {
    slot.aux_mut().set_active(false);
}

/// Take up the commissioning snapshot and ask it for one action.
///
/// A snapshot that does not read back as a sequence mid-flight is started over
/// rather than refused: nothing is torqued during a commission, the sweep is
/// idempotent -- it reads registers and writes the gains the machine should hold
/// anyway -- and a session that gave up here would leave a machine nobody has
/// established anything about, which is the one state this phase exists to end.
fn act(
    snap: &mut CommissionSnapWire,
    fresh: bool,
    now: Duration,
    prior: Option<&BusResult>,
) -> SeqAction<reachy_motion::arm::CommissionSummary> {
    if !fresh
        && let Ok(state) = snap.validate_mut()
        && let Ok(mut seq) = CommissionSequencer::resume(arm_config(), state)
    {
        return seq.next(now, prior);
    }
    // A fresh sequence is told nothing, whatever answer was in hand: the
    // sequencer contract is that the first call carries no prior, and an answer
    // to the old sequence's transaction is an answer to a question this one
    // never asked.
    let mut seq = CommissionSequencer::start(arm_config(), snap);
    seq.next(now, None)
}

/// Take up the resting watch's snapshot and ask it for one action.
///
/// Started over rather than refused, for the commissioning sweep's reason and
/// more plainly: a watch writes nothing in either direction, so a fresh one
/// costs two reads per row it was named for and establishes the readings the
/// torque-on gate judges, where a refusal would leave a script declined over a
/// slot rather than over a machine. A sweep started over here is named for every
/// row, because the mask the stale rows made is exactly what the slot no longer
/// says.
///
/// What the gate sees is never half a picture: the rows this sweep reads are
/// merged into the picture the session keeps, and the gate is judged over that.
fn watch(
    snap: &mut PollSnapWire,
    fresh: bool,
    now: Duration,
    prior: Option<&BusResult>,
) -> SeqAction<Rail> {
    if !fresh
        && let Ok(state) = snap.validate_mut()
        && let Ok(mut seq) = PollSequencer::resume(arm_config(), state)
    {
        return seq.next(now, prior);
    }
    start_watch(snap, flags::all(), now)
}

/// The rail picture the gate judges, and the rows too old to judge from.
///
/// One reading per servo, kept from the driver's health rotation and seeded by
/// the commissioning survey. A row unseen or older than `stale_after_ns` is
/// named in the second answer: it is re-read before the gate runs, and its
/// numbers in the picture are whatever was last recorded, which nothing gates on
/// until the re-read has replaced them.
///
/// A slot that will not read back as a session hands every row over as stale.
/// The reading is this cog's own memory, so bytes it cannot read are memory gone
/// wrong -- and the safe answer to a picture that cannot be read is to look at
/// the machine rather than to gate on it.
fn rail_picture(slot: &SessionStateWire, now_ns: i64, stale_after_ns: i64) -> (Rail, JointFlags) {
    let mut picture = Rail {
        voltages: [0.0; ROW_COUNT],
        health: SERVO_IDS.map(|id| ServoHealth { id, bits: 0 }),
    };
    let Ok(state) = slot.validate() else {
        return (picture, flags::all());
    };
    let mut stale = JointFlags::NONE;
    for (row, joint) in ROWS.into_iter().enumerate() {
        let Some(held) = cells::rail_row(&state.rail, joint) else {
            flags::insert(&mut stale, joint);
            continue;
        };
        if !bool::from(held.seen)
            || now_ns.saturating_sub(held.sample_time.as_nanos()) > stale_after_ns
        {
            flags::insert(&mut stale, joint);
            continue;
        }
        picture.voltages[row] = held.volts;
        picture.health[row] = ServoHealth {
            id: SERVO_IDS[row],
            bits: held.bits,
        };
    }
    (picture, stale)
}

/// Record one servo's rail reading, with the instant it was taken at.
///
/// Every health report the driver's rotation publishes lands here, in every
/// phase: the picture the torque-on gate judges is kept while the machine rests
/// rather than read on the wake that needs it, which is what makes engaging one
/// driver cycle.
///
/// The picture alone is read back rather than the whole session: a reading
/// arrives up to eight times a wake in every phase, and the rail record is nine
/// rows of four numbers against the largest schema this process carries.
pub fn record_reading(
    slot: &mut SessionStateWire,
    joint: JointRef,
    bits: u8,
    volts: f64,
    at_ns: i64,
) {
    let Ok(record) = slot.rail_mut().validate_mut() else {
        return;
    };
    write_reading(record, joint, bits, volts, at_ns);
}

/// One row of the picture, written in place.
fn write_reading(record: &mut RailRecord, joint: JointRef, bits: u8, volts: f64, at_ns: i64) {
    let Some(row) = cells::rail_row_mut(record, joint) else {
        return;
    };
    row.bits = bits;
    row.volts = volts;
    row.sample_time = SyncTime::from_nanos(at_ns);
    row.seen = true.into();
}

/// Merge what a sweep read into the picture: the commissioning survey's own
/// readings, or the resting watch's fallback re-read.
///
/// `rows` is what the sweep was named for, and nothing outside it is touched. A
/// sweep hands back zero for every row it did not read, so merging the whole
/// picture would write a rail of zero volts over rows the driver's rotation had
/// just reported healthy -- and the gate, judged in the same wake, would refuse
/// a machine whose supply nobody had said anything against.
///
/// The survey is what makes the machine never at rest with an empty picture: it
/// reads the supply and the error byte of all nine before the first rest, and
/// those readings are the ones the first wake's gate judges.
pub fn merge_rail(slot: &mut SessionStateWire, rail: &Rail, rows: JointFlags, at_ns: i64) {
    let Ok(record) = slot.rail_mut().validate_mut() else {
        return;
    };
    for (row, joint) in ROWS.into_iter().enumerate() {
        if !flags::contains(rows, joint) {
            continue;
        }
        write_reading(
            record,
            joint,
            rail.health[row].bits,
            rail.voltages[row],
            at_ns,
        );
    }
}

/// The rows the sweep in the slot was named for.
///
/// Read off the sweep's own snapshot rather than remembered by the caller: a
/// sweep whose slot no longer reads as one has established nothing, and the
/// honest merge for it is of no row at all.
fn swept_rows(slot: &SessionStateWire) -> JointFlags {
    slot.poll()
        .validate()
        .map_or(JointFlags::NONE, |snap| snap.rows)
}

/// Write the record a re-issue is rebuilt from.
fn record_pending(slot: &mut SessionStateWire, corr: u32, txn: Txn, now_ns: i64) {
    let pending = slot.aux_mut();
    pending.set_active(true);
    pending.set_corr(corr);
    pending.set_op(AuxOpKindWire::from(txn.op));
    pending.set_id(txn.id);
    pending.set_reg(RegIdWire::from(txn.reg));
    pending.set_value_kind(ValueShapeWire::from(txn.value_kind));
    pending.set_value(txn.value);
    pending.set_issued(SyncTime::from_nanos(now_ns));
    pending.set_retries(0);
}

/// Move to `to`, and say where from.
///
/// The one place a phase is written, so a story about a machine changing phase
/// is the same story wherever the decision was made -- a sequence ending, or a
/// response taking the machine out of service.
///
/// Leaving `engaging` closes whatever ask was open, wherever the leaving was
/// decided. A fault answered while the machine is being taken hold of moves the
/// phase from outside the engagement's own arm -- a rest-class response, a park
/// -- and the driver's answer to the ask already on the wire arrives a wake or
/// two later. Answered against a standing `engage_pending` it would take a
/// parked machine active, which nothing may do until an operator has been, so
/// the ask is closed here: the answer is then a stray and is counted as one.
///
/// Entering `parked` clears `ending_by_fault`, wherever the parking was decided
/// -- the ladder's own park endings, a stow re-ranked to one, a maneuver record
/// that stopped reading as a maneuver. The flag says a fault is carrying this
/// session to rest; a parked session is on its way nowhere, and the script held
/// for it is answered `parked`, which is the truer of the two reasons.
pub fn enter(slot: &mut SessionStateWire, to: SessionPhaseWire) -> Entered {
    let from = slot.phase();
    if from == SessionPhaseWire::ENGAGING && to != SessionPhaseWire::ENGAGING {
        slot.set_engage_pending(false);
        slot.set_engage_issued(SyncTime::from_nanos(0));
    }
    if to == SessionPhaseWire::PARKED {
        slot.set_ending_by_fault(false);
    }
    slot.set_phase(to);
    Entered { to, from }
}

#[cfg(test)]
mod tests {
    //! That a transaction survives every crossing it makes whole.
    //!
    //! A validated view is a reference into the message it validated, so a
    //! transaction crossing out of a sequencer's slot into a datagram, and out
    //! of the pending record a re-issue is rebuilt from, is copied field by
    //! field. The comparisons below are the generated ones, which compare every
    //! field the schema declares: a field this module does not carry is a
    //! mismatch here rather than a register written wrong on a bus.

    use super::{
        BusResult, JointFlagsWire, JointRef, ProfileConfig, Rail, ServoHealth, SessionPhaseWire,
        Timing, Txn, degrade, degrade_rows, enter, flags, init_arm_config, merge_rail,
        rail_picture, record_pending,
    };
    use brenn_reachy__cogs__session_clk_rs::SessionStateWire;
    use brenn_reachy__hardware__dynamixel__registers_clk_rs::{RegId, ValueShape};
    use brenn_reachy__motion__bus_txn_clk_rs::{AuxOpKind, BusTxnWire};

    /// The survey's own readings seed the picture the torque-on gate judges, row
    /// for row and stamped with the instant they were merged at.
    ///
    /// What makes the first wake after a boot cost one datagram: the commission
    /// ends by merging its summary, so a machine at rest is never at rest with an
    /// empty picture waiting on the driver's rotation to lap. Row for row,
    /// because the gate judges each row's own supply and error byte -- a merge
    /// that spread one row's numbers over nine would gate every row on one
    /// servo's answer.
    #[test]
    fn a_surveys_own_readings_seed_the_gates_picture_row_for_row() {
        let mut slot = SessionStateWire::new();
        let surveyed = Rail {
            voltages: [7.1, 7.2, 7.3, 7.4, 7.5, 7.6, 7.7, 7.8, 7.9],
            health: core::array::from_fn(|row| ServoHealth {
                id: 10 + row as u8,
                bits: row as u8,
            }),
        };

        merge_rail(&mut slot, &surveyed, flags::all(), 4_000);
        let (picture, stale) = rail_picture(&slot, 5_000, 2_000);

        assert!(
            flags::is_empty(stale),
            "every row was seen, at the instant the merge was told",
        );
        assert_eq!(picture.voltages, surveyed.voltages);
        for (row, health) in picture.health.into_iter().enumerate() {
            assert_eq!(
                health.bits, surveyed.health[row].bits,
                "row {row} carries its own error byte",
            );
        }
        let (_, aged) = rail_picture(&slot, 7_000, 2_000);
        assert_eq!(
            aged,
            flags::all(),
            "and the stamp is the merge's, so the whole picture ages together",
        );
    }

    /// Entering `parked` leaves no fault ending standing, whatever decided the
    /// parking.
    ///
    /// The flag says a fault is carrying this session to rest. A parked session
    /// is carried nowhere -- nothing engages it until an operator has been -- and
    /// the paths that park a machine mid-stow reach the phase without the ladder
    /// answering again, so the clearing belongs to the one place a phase is
    /// written.
    #[test]
    fn entering_parked_clears_a_fault_ending() {
        let mut slot = SessionStateWire::new();
        slot.set_phase(SessionPhaseWire::WINDING_DOWN);
        slot.set_ending_by_fault(true);

        let entered = enter(&mut slot, SessionPhaseWire::PARKED);

        assert_eq!(entered.from, SessionPhaseWire::WINDING_DOWN);
        assert_eq!(entered.to, SessionPhaseWire::PARKED);
        assert!(!slot.ending_by_fault());
    }

    /// The size of a transaction record, which is every byte this module copies.
    ///
    /// A tripwire: the cases below carry the fields the schema declares today,
    /// and a field added to it would be dropped by each copy in silence. Growing
    /// the record fails here instead, which sends the next reader to the copies.
    #[test]
    fn a_transaction_is_the_record_these_cases_carry() {
        assert_eq!(core::mem::size_of::<BusTxnWire>(), 16);
    }

    /// A transaction with every field distinct from its neighbours, so a
    /// crossing that swapped two of them fails rather than passing on symmetry.
    fn asked() -> BusTxnWire {
        let mut wire = BusTxnWire::new();
        let held = wire.clear_valid();
        held.active = true.into();
        held.op = AuxOpKind::WriteRegVerified;
        held.id = 14;
        held.reg = RegId::GoalPosition;
        held.value_kind = ValueShape::I32;
        held.value = 0x0123_4567_89ab_cdef;
        wire
    }

    /// A slot mid-drain: the antenna pair owed a de-torque, the first row's
    /// write outstanding.
    fn draining() -> (SessionStateWire, JointRef) {
        let mut slot = SessionStateWire::new();
        slot.set_degrade_release(JointFlagsWire::from(degrade_rows()));
        let drain = degrade(&mut slot, None, 0, &timing());
        assert!(drain.delivery.datagram.is_some(), "a row was asked for");
        let asked = flags::iter(degrade_rows()).next().expect("a row is owed");
        (slot, asked)
    }

    fn timing() -> Timing {
        Timing {
            aux_timeout_ns: 1_000_000,
            aux_retries: 3,
            rail_stale_after_ns: 2_200_000_000,
        }
    }

    /// A de-torque the servo only acknowledged is a row this session cannot say
    /// went limp, so the row stays owed and the drain reports it refused.
    ///
    /// The one place the two writes have to stay apart: every de-torque is
    /// issued verified, and the guard against a later one being issued the
    /// cheaper way is that an acknowledgement is not the result this test
    /// accepts.
    #[test]
    fn a_de_torque_nothing_read_back_does_not_release_a_row() {
        let (mut slot, asked) = draining();
        let drain = degrade(&mut slot, Some(BusResult::Acknowledged), 1, &timing());
        assert_eq!(drain.refused, Some(asked));
        assert_eq!(drain.released, None);
        assert_eq!(
            slot.degrade_release().to_known().expect("a session"),
            degrade_rows(),
            "the row is still owed a write",
        );
    }

    /// The same drain, answered by a write that read back: the row lets go.
    #[test]
    fn a_de_torque_read_back_releases_its_row() {
        let (mut slot, asked) = draining();
        let drain = degrade(&mut slot, Some(BusResult::Written), 1, &timing());
        assert_eq!(drain.refused, None);
        assert_eq!(
            slot.degrade_release().to_known().expect("a session"),
            flags::without(degrade_rows(), flags::bit(asked)),
        );
    }

    /// What the sequencer asked for is what the datagram carries.
    #[test]
    fn a_transaction_reaches_a_datagram_whole() {
        let wire = asked();
        let mut carried = BusTxnWire::new();
        Txn::of(&wire).write(carried.clear_valid());
        assert_eq!(carried, wire);
    }

    /// And what the pending record holds is what goes out again, which is what
    /// makes a re-issue the same datagram rather than a similar one.
    #[test]
    fn a_re_issue_is_rebuilt_whole() {
        let wire = asked();
        let mut slot = Box::new(SessionStateWire::new());
        slot.clear_valid();
        record_pending(&mut slot, 7, Txn::of(&wire), 1_000);

        let mut again = BusTxnWire::new();
        Txn::of_pending(slot.aux()).write(again.clear_valid());
        assert_eq!(again, wire);
        assert_eq!(slot.aux().corr(), 7, "under the number it was issued as");
    }

    /// The shipped profile, for the cases about what is refused.
    ///
    /// Every field non-zero, so a case that zeroes one is a case about that one
    /// field. The numbers are not the deployed ones and do not need to be: what
    /// is under test is which values are refused, and the refusal is about zero.
    fn profile() -> ProfileConfig {
        ProfileConfig {
            acceleration: 20,
            velocity: 50,
            bus_watchdog: 10,
        }
    }

    /// A profile a configuration that lost its watchdog line parses to.
    ///
    /// The realistic drift: a textproto omitting the field parses to zero, which
    /// is the register disabled -- a machine that keeps chasing a stale goal
    /// after the driver commanding it has died. Refusing it here is the whole
    /// of what stands between that file and a commissioned machine, so it is
    /// asserted rather than assumed.
    #[test]
    #[should_panic(expected = "zero in the watchdog register")]
    fn a_session_with_the_watchdog_disabled_is_refused_before_anything_is_armed() {
        init_arm_config(ProfileConfig {
            bus_watchdog: 0,
            ..profile()
        });
    }

    /// And its older sibling: zero acceleration is a servo running unlimited
    /// rather than the backstop the pair exists to be.
    #[test]
    #[should_panic(expected = "zero in either register")]
    fn a_session_whose_servo_profile_is_unlimited_is_refused_the_same_way() {
        init_arm_config(ProfileConfig {
            acceleration: 0,
            ..profile()
        });
    }

    /// The same for velocity, which the assert covers with one condition and so
    /// could lose without a case to say so.
    #[test]
    #[should_panic(expected = "zero in either register")]
    fn a_session_whose_servo_velocity_is_unlimited_is_refused_the_same_way() {
        init_arm_config(ProfileConfig {
            velocity: 0,
            ..profile()
        });
    }
}
