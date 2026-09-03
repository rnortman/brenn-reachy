//! The session cog's body: what the machine is doing, and what it says about it.
//!
//! One function, `execute_session`, over the slot the session keeps between
//! executions. It is a module of the cog-bodies crate rather than a crate of its
//! own because the entry point the compiler generates for a cog calls into the
//! crate its module names, and `cogs/motion.clk` names one crate for all of its
//! cogs.
//!
//! Six things happen here. Scripts are screened -- whole or not at all,
//! against the phase and against their own numbers -- and an accepted one
//! becomes the schedule the session holds. Evidence is weighed: the decision
//! tick's raises, the edges the driver reported, the error bits its rotating
//! read carried, and the silence of a sample stream that stopped, each becoming
//! a recorded fault and the response the library classifies it with, which is
//! `session_ladder`'s. A stow maneuver running is stepped, which is
//! `session_stow`'s: the schedule it asks for is published from here and the
//! ending it reaches is recorded here. The bus is driven: one turn per wake at
//! whichever sequence the phase machine is running, which is `session_bus`'s,
//! and the datagram the wake comes to owe is published from here. What the
//! machine is under command to do is published: the schedule the session holds
//! goes out whole when the engagement takes hold and again when what it was
//! engaged for is over. And the machine's story is
//! written: a refusal, an acceptance, a fault answered and a phase
//! entered each leave a row in the report ring, and every execution that added
//! a row publishes the whole ring as one message. What a row means is
//! `motion/reports.clk`'s statement, and the ring itself is `session_slots`'.
//!
//! Nothing here holds state of its own and nothing reads a clock: the execution
//! start time arrives on the dial, and everything the session remembers is in
//! the slot. A `static` would be a second memory no test could set and no
//! restart would clear.
//!
//! Screening classifies and never repairs. A script that cannot be run is
//! refused whole with one reason, because a partially applied request is a
//! request nobody made; and a refusal is a report rather than a fault, since
//! nothing about the machine is wrong when it declines a script.

use crate::session_bus::{self, Datagram, Delivery, Entered, Timing};
use crate::session_ladder::{self, Budgets};
use crate::session_stow;
use brenn_reachy__cogs__config_clk_rs::SessionParams;
use brenn_reachy__cogs__motion_clk_rs::{SessionDial, SessionSignals};
use brenn_reachy__cogs__schedule_clk_rs::{
    OverlayWindowWire, PostureWire, ScheduledStepWire, SessionScheduleWire, StepKindWire,
};
use brenn_reachy__cogs__script_clk_rs::Script;
use brenn_reachy__cogs__session_clk_rs::{SessionPhase, SessionPhaseWire, SessionStateWire};
use brenn_reachy__cogs__session_cmd_clk_rs::SessionCmdKind;
use brenn_reachy__driver__health_clk_rs::{AuxStatus, EventKind};
use brenn_reachy__hardware__dynamixel__registers_clk_rs::ValueShape;
use brenn_reachy__motion__bus_txn_clk_rs::AuxOpKind;
use brenn_reachy__motion__faults_clk_rs::FaultKindWire;
use brenn_reachy__motion__faults_clk_rs::ResponseKindWire;
use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointFlagsWire, JointRefWire};
use brenn_reachy__motion__reports_clk_rs::{RefusalReasonWire, ReportKind, ReportKindWire};
use brenn_reachy__motion__seq_clk_rs::SeqFailureKindWire;
use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, WindDownOutcomeWire};
use clockwork_rs::{Clear as _, SyncTime};
use motion_slots::{MS_NS, configured, counters};
use reachy_motion::arm::{ProfileConfig, row_of_id};
use reachy_motion::fault::{self, FaultKind};
use reachy_motion::joints::{self, JointRef};
use reachy_motion::seq::{BusResult, SeqFailureKind};
use reachy_motion::tick::ResponseKind;
use reachy_motion::value;
use reachy_motion::verdict::{self, VerdictError};
use reachy_motion::winddown::{Disposition, Maneuver, ending, maneuver_of};
use session_slots::{clear_timeline, held_reports, push_report, report_row};

/// How many motions a library can hold: `cogs/config.clk`'s capacity for them,
/// which is what an overlay's motion id indexes.
///
/// Stated here rather than borrowed from the library crate, which the session
/// holds no part of: the number is the schema's, and a case below reads it off
/// the schema and fails if the two ever disagree.
const MAX_MOTIONS: usize = 32;

counters! {
    /// The session's run totals, as the slot holds them and the signal group
    /// reports them.
    ///
    /// Every one is the run's absolute count rather than a window's, which is
    /// what makes a report of them readable whichever window it lands in.
    SessionCounters of SessionStateWire, SessionSignals, crossing session_totals_cross_the_slot {
        /// Scripts that passed the screen.
        scripts_accepted / set_scripts_accepted,
        /// Scripts refused whole, a datagram that was no script at all
        /// included.
        scripts_refused / set_scripts_refused,
        /// Raises recorded as faults: the ones the classifier answers with
        /// something.
        faults_recorded / set_faults_recorded,
        /// Story datagrams that went out: one per execution that added a row.
        reports_published / set_reports_published,
        /// Reports appended to the timeline.
        reports_narrated / set_reports_narrated,
        /// Reports dropped off the front of a full ring.
        reports_dropped / set_reports_dropped,
        /// Inbound datagrams the codec refused.
        undecodable_inbound / set_undecodable_inbound,
        /// Times this cog's own slot did not read back as a session.
        refused_state / set_refused_state,
        /// Datagrams re-issued because nothing answered the one before.
        aux_retries / set_aux_retries,
        /// Transactions given up on with the delivery budget spent.
        aux_failures / set_aux_failures,
        /// Answers that matched no outstanding request: a late duplicate of one
        /// already settled, dropped as it is read.
        aux_strays / set_aux_strays,
        /// Engagement answers that arrived for an ask nobody was waiting on: a
        /// late duplicate, or an answer to an ask a fault closed by moving the
        /// phase.
        engage_strays / set_engage_strays,
        /// Engagements rested at the silence deadline, the driver having
        /// answered none of the re-issues.
        engage_silences / set_engage_silences,
        /// Responses selected. Fewer than the faults recorded: a fault arriving
        /// while the response it calls for is already being carried out is
        /// answered by the standing one.
        responses_taken / set_responses_taken,
    }
}

/// One thing the session has to say, before it reaches the ring.
///
/// A plain struct rather than the schema row, because the same values are
/// written twice on the path a story takes: into the ring here, and out of it
/// into the one message an execution publishes. What each number means is stated
/// per kind in `motion/reports.clk`, which is where a reader of a recorded row
/// looks.
#[derive(Clone, Copy, PartialEq, Debug)]
struct Report {
    /// The instant it is about -- not the execution that publishes it.
    time_ns: i64,
    /// What it is about.
    kind: ReportKind,
    /// The first specific, meaning per kind.
    a: u32,
    /// The second specific, likewise.
    b: u32,
    /// The measured specific, likewise. Carried bit for bit, a non-finite
    /// number included: a story dropped for an unreadable measurement loses the
    /// part that was readable with it.
    detail: f64,
}

impl Report {
    /// Fill a ring row with it.
    fn write(&self, row: &mut TimelineEntryWire) {
        row.set_time(SyncTime::from_nanos(self.time_ns));
        row.set_kind(ReportKindWire::from(self.kind));
        row.set_a(self.a);
        row.set_b(self.b);
        row.set_detail(self.detail);
    }
}

/// What the session says about a session, once per execution.
///
/// The order is the order the story is told in: the slot is read back, the
/// evidence that arrived is weighed, the scripts that arrived are screened
/// against what it left, a maneuver running takes its step, the bus takes its
/// turn, what the machine is under
/// command to do goes out if it changed, and then the story goes out if this
/// execution added to it. Publishing last is what lets a wake that raised the
/// only report of it also carry the story away.
pub fn execute_session(dial: &mut SessionDial<'_>) {
    let before = SessionCounters::read(dial.states.sess);
    let mut counters = before;
    let now = dial.start_time().as_nanos();

    let params: &SessionParams = configured(dial.configs.params, "the session's");
    let budgets = Budgets::of(params);
    let screens = Screens::of(params);
    let timing = Timing {
        aux_timeout_ns: params.aux_timeout_ns,
        aux_retries: params.aux_retries,
        rail_stale_after_ns: params.rail_stale_after_ns,
    };
    // The commissioned record, taken from configuration on the first wake and
    // shared from then on. The profile is the one part of it this deployment
    // chooses; everything else in it is a hardware fact the motion library
    // states once. Delivered by initialising the record rather than by carrying
    // it: the machinery that needs it reaches the one copy.
    session_bus::init_arm_config(ProfileConfig {
        acceleration: params.profile_acceleration,
        velocity: params.profile_velocity,
        bus_watchdog: params.bus_watchdog,
    });

    // The slot is this cog's own memory and nothing else writes it, so bytes it
    // cannot read are memory gone wrong rather than another writer's opinion.
    // Cleared and counted, and then the machine is let go of: what the slot
    // holds is the whole record of a machine that may be under command -- the
    // engagement, a release still owed, a maneuver being carried out -- so a
    // reading that came back as nothing is a session with no idea what it is
    // commanding.
    if dial.states.sess.validate_mut().is_err() {
        let _ = dial.states.sess.clear_valid();
        counters.refused_state += 1;
        lost_the_session(dial, &mut counters, now, &budgets);
    }

    watch(dial, &mut counters, now, &budgets);
    record_raises(dial, &mut counters, now, &budgets);
    record_events(dial, &mut counters, now, &budgets);
    record_readings(dial, &mut counters, now, &budgets);
    // Intake last of the three, because a script is answered against the phase
    // the evidence of this same wake leaves the machine in: a wake that both
    // accepted a script and parked the machine would have narrated an
    // acceptance its sender will act on for a session that is already over.
    let replaced = screen_scripts(dial, &mut counters, now, &screens);
    // One datagram, decided in one place, with one exception: the drain below
    // may take an engagement's first bus step on a wake whose turn here issued
    // nothing at all, and that step's datagram is the wake's. A release owed
    // supersedes any bus work: nothing gates de-torquing, and a sequence stepped
    // into a slot whose datagram was then overwritten would leave a transaction
    // recorded as outstanding that never went out.
    // Before the bus, because a maneuver that concluded commands the release and
    // the bus half is what publishes one: a machine being let go of is not one
    // to ask anything else of on the same wake.
    run_winddown(dial, &mut counters, now, &budgets);
    run_bus(dial, &mut counters, now, &timing, &budgets);
    // After the bus and before the schedule: the phase this wake leaves the
    // machine in is what a held script is answered against, and a drain that
    // took a replacement is one the wake's one publish is owed.
    let drained = drain_pending(
        dial,
        &mut counters,
        now,
        &screens,
        &timing,
        &budgets,
        replaced,
    );
    // After the bus, because the phase this wake leaves the machine in is what
    // decides whether it is under command, and every path that moves the phase
    // has run by here. A replacement taken this wake is published from here for
    // that same reason: a schedule that was already over when it arrived is one
    // the bus half has by now ended, and the machine being let go of and the
    // replacement are then one change on the channel rather than two.
    publish_schedule(dial, &mut counters, now, replaced || drained);
    say_unconfirmed(dial, &mut counters, now, &budgets);
    publish_timeline(dial, &mut counters, before.reports_narrated);

    counters.store(dial.states.sess);
    // Untested: no assertion in this repo covers the values a signal carries.
    // TODO(cogs-signal-report-contents)
    counters.report(&before, &mut dial.signals);
}

/// Screen every script that arrived, in arrival order.
///
/// Each one is answered as it is read: a refusal says nothing about the next
/// datagram in the window, and a script accepted after a refused one is the
/// session's answer to the request it could run. The slot and the message
/// window are separate borrows, so each decision is written where it is made.
///
/// Answers whether a replacement was taken, which the wake's one publish is
/// owed. Nothing is published from here: the output and the input window are one
/// borrow of the dial apiece and the loop holds the input one, and the wake
/// publishes once, at one site, after every path that could move the phase.
///
/// The channel is latest-wins and the slot holds one schedule, so a window
/// carrying two replacements runs the later one: the first is answered as the
/// acceptance it was and its plan is then overwritten by the next. They share
/// the wake's one epoch, which is the epoch that goes out -- a row here never
/// names an epoch no consumer saw.
fn screen_scripts(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    now: i64,
    screens: &Screens,
) -> bool {
    let mut replaced = false;
    for message in dial.inputs.script.new_msgs() {
        // Bytes that describe no script name no times and no motions, so there
        // is nothing to screen. The boundary refusal is this one call, and the
        // refusal is still narrated: a sender whose datagram this build cannot
        // read has asked for something and is owed an answer.
        let answer = match message.validate() {
            Err(_) => {
                counters.undecodable_inbound += 1;
                Answer {
                    report: refusal(0, RefusalReasonWire::UNDECODABLE, counters, now),
                    entered: None,
                }
            }
            Ok(script) => decide(dial.states.sess, script, counters, now, screens, replaced),
        };
        // What a replacement is, said once: the row it narrates. A second flag
        // beside it could disagree with it, and a replacement that published
        // nothing would strand the mover on the schedule before it.
        replaced |= answer.report.kind == ReportKind::ScriptReplaced;
        narrate(dial.states.sess, counters, &answer.report);
        if let Some(entered) = answer.entered {
            narrate(dial.states.sess, counters, &phase_report(&entered, now));
        }
    }
    replaced
}

/// Open an engagement: the phase, and the three fields the ask is kept in.
///
/// No reading is touched and no gate is judged here. The gate is arithmetic over
/// the rail picture and is judged on this wake's bus turn, which is where the
/// datagram it clears is issued from: this is the intake's half of it, and it is
/// bookkeeping.
fn open_engagement(slot: &mut SessionStateWire) -> Entered {
    slot.set_engage_pending(true);
    slot.set_engage_issued(SyncTime::from_nanos(0));
    slot.set_engage_rows(JointFlagsWire::from(JointFlags::NONE));
    session_bus::enter(slot, SessionPhaseWire::ENGAGING)
}

/// What one script was answered with: the story of the answer, and the phase the
/// answer moved the machine to.
struct Answer {
    /// The acceptance or the refusal.
    report: Report,
    /// The phase entered, where accepting moved the machine.
    entered: Option<Entered>,
}

/// Answer one script: the schedule it became, or the reason it was refused.
///
/// All-or-nothing and in this order: the intake screens first, because a script
/// the machine cannot take now is refused whatever it says; then the numbers,
/// because a schedule is built from them. An acceptance is the schedule the
/// session holds from here, which is why the last accepted script in a window
/// wins and why they are answered in arrival order.
///
/// Two shapes of acceptance, and which one it is depends only on the phase the
/// script arrived in. From `resting` it opens an engagement: the schedule is
/// written with nothing engaged and the arming sequence begins. From `active`
/// it replaces the schedule of an engagement already running: the machine is
/// already torqued and already being streamed goals, so nothing arms, no phase
/// moves, and what changes is the whole of the commanded future.
///
/// A machine mid-maneuver takes neither shape now: the script is held whole, and
/// the wake that maneuver ends on calls this again over it -- except where the
/// maneuver is a fault's, which is refused rather than held. Both an acceptance
/// and a replacement clear whatever was held, so nothing ever applies a held
/// script after a newer one has been taken.
fn decide(
    slot: &mut SessionStateWire,
    script: &Script,
    counters: &mut SessionCounters,
    now: i64,
    screens: &Screens,
    bumped: bool,
) -> Answer {
    let phase = slot.phase();
    match phase_intake(slot) {
        Intake::Refuse(reason) => {
            return Answer {
                report: refusal(script.script_id, reason, counters, now),
                entered: None,
            };
        }
        Intake::Hold => return hold(slot, phase, script, counters, now),
        Intake::Take => {}
    }
    let outcome = match ordering_refusal(phase, slot, script) {
        Some(reason) => Err(reason),
        None => screened_plan(script, now, screens).and_then(|plan| {
            if engages_for_nothing(phase, &plan, now) {
                return Err(RefusalReasonWire::BAD_TIMES);
            }
            Ok(plan)
        }),
    };
    match outcome {
        Ok(plan) => {
            let replacing = phase == SessionPhaseWire::ACTIVE;
            // One bump per wake: a second replacement in one window lands under
            // the epoch the first opened, because that is the one epoch this
            // wake publishes.
            let epoch = store(slot, script.script_id, &plan, replacing, !bumped);
            slot.set_active_script_id(script.script_id);
            // The newer script supersedes the held one, in both directions: a
            // hold is answered by whatever the machine takes next, and a
            // schedule taken here is never overwritten afterwards by something
            // older that was waiting.
            slot.set_pending_valid(false);
            counters.scripts_accepted += 1;
            let steps = u32::try_from(plan.steps().len()).unwrap_or(u32::MAX);
            if replacing {
                Answer {
                    report: Report {
                        time_ns: now,
                        kind: ReportKind::ScriptReplaced,
                        a: script.script_id,
                        b: epoch,
                        detail: f64::from(steps),
                    },
                    entered: None,
                }
            } else {
                Answer {
                    report: Report {
                        time_ns: now,
                        kind: ReportKind::ScriptAccepted,
                        a: script.script_id,
                        b: steps,
                        detail: f64::from(epoch),
                    },
                    // An accepted script is a machine to take hold of, so the
                    // engagement's first bus step -- the gate, then the
                    // `engage_now` or the fallback poll's first read -- is taken
                    // by this same execution.
                    entered: Some(open_engagement(slot)),
                }
            }
        }
        Err(reason) => Answer {
            report: refusal(script.script_id, reason, counters, now),
            entered: None,
        },
    }
}

/// Why the number `script` carries is no number this session will take, or that
/// it is.
///
/// Decided before anything else in the script is read: the number it carries
/// against the engagement it would replace. What the script says -- its times,
/// its motions, how far ahead it reaches -- is [`screened_plan`]'s, because
/// those are screened as the schedule is built rather than by reading the rows
/// twice.
///
/// `phase` is the caller's read of the slot, so one wake's answer is decided
/// against one phase.
fn ordering_refusal(
    phase: SessionPhaseWire,
    slot: &SessionStateWire,
    script: &Script,
) -> Option<RefusalReasonWire> {
    // Ordering only mid-session: at rest the session takes whatever number it
    // is offered. A high-water mark kept across engagements would let one
    // sender that reset its counter lock the machine out until an operator
    // restarted the process, and defending a resting machine against a stale
    // delivery belongs at the edge that received it.
    //
    // Against the engagement's own number and not the slot's `script_id`: the
    // two are written together today, and this reads the one whose meaning is
    // "what a replacement must beat".
    if phase == SessionPhaseWire::ACTIVE && script.script_id <= slot.active_script_id() {
        return Some(RefusalReasonWire::STALE);
    }
    None
}

/// The schedule `script` asks for, screened against what the session will hold
/// open, or why it is no schedule this session will run.
///
/// The horizon is measured off the plan rather than off the script's own
/// offsets, so there is one scan of "the last instant a schedule owns" in the
/// session and a row kind the vocabulary grows is inside it the day the planner
/// writes it.
///
/// Measured twice, against two different origins, and the further of the two
/// decides. From the script's own arrival, which is exactly the offsets it
/// wrote: what the sender asked for is bounded whatever its clock says. And
/// from this wake, which is what the ceiling is actually for: the schedule is
/// the machine's committed future, the session lets go when it runs out, and
/// what a sender that stops talking may leave behind is that long and no
/// longer. The two differ by the sender's skew, and a stamp reading ahead of
/// this machine is the direction that matters -- a script stamped an hour out
/// with in-cap offsets would otherwise commit the head for an hour past the
/// ceiling, with the session concluding nothing until it got there.
///
/// # Errors
///
/// [`plan_of`]'s reasons for a script that is no timeline, and
/// [`RefusalReasonWire::TOO_LONG`] for one reaching further ahead than the
/// session will hold a schedule open for.
fn screened_plan(
    script: &Script,
    now: i64,
    screens: &Screens,
) -> Result<SessionScheduleWire, RefusalReasonWire> {
    let plan = plan_of(script)?;
    let arrival = script.arrival.as_nanos();
    let horizon_ns = session_bus::last_instant(&plan).map_or(0, |end| {
        // Saturating because both origins are numbers off the wire: a stamp no
        // clock could hold is a refusal rather than arithmetic that wrapped.
        end.saturating_sub(arrival).max(end.saturating_sub(now))
    });
    if horizon_ns > screens.span_cap_ns {
        return Err(RefusalReasonWire::TOO_LONG);
    }
    Ok(plan)
}

/// Whether taking `plan` would engage the machine for a schedule with no
/// future.
///
/// A schedule whose last instant is already behind this wake commands nothing.
/// Reached two ways, both of them ordinary: a sender whose stamp reads behind
/// this machine's clock, and a script held across a maneuver for longer than the
/// whole span it asked for -- the instants are the sender's offsets off the
/// sender's own stamp, and holding does not move them.
///
/// From `active` such a schedule is a session over, which is a sender's way of
/// saying stop and is answered by the release. From `resting` there is nothing
/// to say stop about: engaging would torque the machine on, reach no goal, and
/// let go of it again on the next wake, so it is refused. A schedule that asks
/// for nothing at all is not this -- it owns no instant to have outrun.
fn engages_for_nothing(phase: SessionPhaseWire, plan: &SessionScheduleWire, now: i64) -> bool {
    // The ends are exclusive wherever a schedule's end is judged, so an instant
    // at one is an instant the interval no longer owns.
    phase != SessionPhaseWire::ACTIVE
        && session_bus::last_instant(plan).is_some_and(|end| end <= now)
}

/// What the intake screens are configured with, copied off the configuration
/// once per execution.
///
/// A plain value beside [`Budgets`] and for the same reason: a screen writes
/// the slot, and the configuration is a borrow of the dial that holds it. A
/// struct rather than loose arguments because the screens are a set that grows,
/// and the next one is a field here rather than a signature everywhere.
#[derive(Clone, Copy)]
struct Screens {
    /// How far ahead a script may schedule anything, nanoseconds: from its own
    /// arrival stamp and from the wake that reads it, whichever leaves the
    /// further horizon.
    span_cap_ns: i64,
}

impl Screens {
    /// The numbers the screens are held to, off the session's configuration.
    fn of(params: &SessionParams) -> Self {
        Self {
            // Every millisecond a `UInt32` holds is a count of nanoseconds an
            // `i64` holds, so the widening is the whole of the arithmetic.
            span_cap_ns: i64::from(params.script_span_cap_ms) * MS_NS,
        }
    }
}

/// The report a refused script leaves, counted as it is built.
fn refusal(
    script_id: u32,
    reason: RefusalReasonWire,
    counters: &mut SessionCounters,
    now: i64,
) -> Report {
    counters.scripts_refused += 1;
    Report {
        time_ns: now,
        kind: ReportKind::ScriptRefused,
        a: script_id,
        b: u32::from(reason.0),
        detail: 0.0,
    }
}

/// What the session does with a script that arrived in a given phase.
enum Intake {
    /// Answer it now: an acceptance or a replacement, or a refusal for what the
    /// script itself says.
    Take,
    /// Keep it for the phase the maneuver under way ends in.
    Hold,
    /// Refuse it outright, with this reason.
    Refuse(RefusalReasonWire),
}

/// What the session does with a script arriving at the phase `slot` stands in.
///
/// Two phases answer one now. `resting` opens an engagement, and `active`
/// replaces the schedule of one already running: a session that means to hold a
/// conversation is refreshed by its sender as it goes, and answering each
/// refresh with a disarm and a fresh engagement would cycle torque on the head
/// for every one of them.
///
/// Three are mid-something -- being commissioned, being taken hold of, being let
/// go of -- and they hold it. A wake is the next command and never an error, and
/// the maneuver under way is not interrupted for it: every one of them runs to a
/// phase this session can answer from, and the wake it gets there takes the
/// script up. Holding rather than interrupting is what keeps the doctrine's rule
/// that nothing gates or defers a de-torquing.
///
/// One maneuver is not held across: a fault's controlled stow, because a script
/// applied at its end would re-energise a machine that a fault just stood down,
/// on no act made after it stood. `winding_down` is entered only by that stow,
/// so the phase is the whole of the test; and a machine at rest that still owes
/// a fault's release is inside the same ending, which is what `ending_by_fault`
/// says. Both refuse.
///
/// `parked` refuses. It is latched: nothing engages a parked machine until an
/// operator has been, so a script held for it would be held for ever.
fn phase_intake(slot: &SessionStateWire) -> Intake {
    match slot.phase().to_known() {
        // TODO(script-cause): a script carries no cause, so a sender's periodic
        // refresh arriving here after a fault ended the last session is
        // indistinguishable from someone saying the wake word and is engaged.
        Some(SessionPhase::Resting) if slot.ending_by_fault() => {
            Intake::Refuse(RefusalReasonWire::FAULT_ENDING)
        }
        Some(SessionPhase::Resting | SessionPhase::Active) => Intake::Take,
        Some(SessionPhase::Starting | SessionPhase::Engaging | SessionPhase::Stopping) => {
            Intake::Hold
        }
        Some(SessionPhase::WindingDown) => Intake::Refuse(RefusalReasonWire::FAULT_ENDING),
        // A phase this build cannot name is a slot that did not read back, which
        // the execution's own validation has already answered by letting go of
        // the machine; refusing engages nothing, which is the answer that holds
        // either way.
        Some(SessionPhase::Parked) | None => Intake::Refuse(RefusalReasonWire::PARKED),
    }
}

/// Hold `script` for the phase the maneuver under way ends in.
///
/// One slot and not a queue, screened against what it already holds by the same
/// ordering rule a replacement is screened by: strictly greater, so a
/// re-delivered script is refused `stale` rather than held twice. The running
/// engagement's own number is not a floor here -- the ordering rules are the
/// drain's, applied to the phase the maneuver ends in. The whole
/// script is kept rather than the schedule it would become, because the horizon
/// screen is measured from the wake it takes effect on -- a script held across a
/// release keeps the whole span it asked for.
///
/// Nothing about the machine changes: no phase moves, nothing is engaged,
/// nothing is published. The row says the sender is owed an answer and will get
/// one.
fn hold(
    slot: &mut SessionStateWire,
    phase: SessionPhaseWire,
    script: &Script,
    counters: &mut SessionCounters,
    now: i64,
) -> Answer {
    // TODO(hold-ordering-floor): the floor here is the held id alone, so a
    // duplicate of the script the running engagement is already on is held and
    // then answered by the drain -- `stale` at `active`, an acceptance at
    // `resting` -- where an arrival in `active` is refused at once.
    if slot.pending_valid() && script.script_id <= slot.pending().script_id() {
        return Answer {
            report: refusal(script.script_id, RefusalReasonWire::STALE, counters, now),
            entered: None,
        };
    }
    // Copied whole into the slot: the held script outlives the message window it
    // arrived in, and the drain screens it from the slot's own copy.
    *slot.pending_mut() = clockwork_rs::as_raw(script).clone();
    slot.set_pending_valid(true);
    Answer {
        report: Report {
            time_ns: now,
            kind: ReportKind::ScriptHeld,
            a: script.script_id,
            b: u32::from(phase.0),
            detail: 0.0,
        },
        entered: None,
    }
}

/// The schedule `script` asks for, or why it is no schedule.
///
/// Built whole beside the slot and not in it: a script is taken all or not at
/// all, so a row that will not do leaves the session holding the schedule it
/// had. The schedule's own message is what it is built in, so a step or a
/// window is written once, in the vocabulary a consumer reads it in, and the
/// row count a script may ask for is the one the schedule can hold rather than
/// a number restated here.
///
/// Offsets become instants by arithmetic off the sender's own stamp, never off a
/// clock read here: a sender whose clock is wrong asks for a script that plays
/// at the wrong time, which is visible, where a consumer substituting its own
/// instant would silently reinterpret the request.
///
/// # Errors
///
/// [`RefusalReasonWire::TOO_MANY_STEPS`] or
/// [`RefusalReasonWire::TOO_MANY_OVERLAYS`] for more rows than the schedule
/// holds, [`RefusalReasonWire::UNKNOWN_MOTION`] for a motion no library could
/// name, and [`span_of`]'s reason for times that are no timeline.
fn plan_of(script: &Script) -> Result<SessionScheduleWire, RefusalReasonWire> {
    let arrival = script.arrival.as_nanos();
    let mut plan = SessionScheduleWire::new();

    // Non-decreasing offsets and positive durations, checked as the rows are
    // written: a script whose steps go backwards is not a timeline, and a step
    // of no length is an interval that owns no instant.
    {
        let mut steps = plan.steps_mut();
        let mut earliest = 0;
        for step in script.steps.iter() {
            let (start, end) = span_of(arrival, step.after_ms, step.duration_ms, &mut earliest)?;
            let Some(row) = steps.try_grow() else {
                return Err(RefusalReasonWire::TOO_MANY_STEPS);
            };
            let row: &mut ScheduledStepWire = row;
            row.set_start(SyncTime::from_nanos(start));
            row.set_end(SyncTime::from_nanos(end));
            // The script and the schedule share the step vocabulary, so what a
            // step asks for crosses the screen without being reinterpreted.
            row.set_kind(StepKindWire::from(step.kind));
            row.set_posture(PostureWire::from(step.posture));
        }
    }

    {
        let mut overlays = plan.overlays_mut();
        let mut earliest = 0;
        for overlay in script.overlays.iter() {
            // A motion is named by its index among the configured motions, and the
            // session holds no library: what it can say is that an index this large
            // could name a motion in no library at all. Whether *this* library holds
            // one is the mover's question, asked at play time.
            if usize::from(overlay.motion_id) >= MAX_MOTIONS {
                return Err(RefusalReasonWire::UNKNOWN_MOTION);
            }
            let (start, end) = span_of(
                arrival,
                overlay.after_ms,
                overlay.duration_ms,
                &mut earliest,
            )?;
            let Some(row) = overlays.try_grow() else {
                return Err(RefusalReasonWire::TOO_MANY_OVERLAYS);
            };
            let row: &mut OverlayWindowWire = row;
            row.set_motion_id(overlay.motion_id);
            row.set_start(SyncTime::from_nanos(start));
            row.set_end(SyncTime::from_nanos(end));
            row.set_gain(overlay.gain);
            row.set_speed(overlay.speed);
        }
    }
    Ok(plan)
}

/// The half-open interval an offset and a duration name, measured from
/// `arrival`.
///
/// `earliest` is the offset the previous interval began at and is advanced to
/// this one, which is what makes the sequence's order a property of the whole
/// script rather than of a pair.
///
/// # Errors
///
/// [`RefusalReasonWire::BAD_TIMES`] for an offset behind the one before it, a
/// duration of zero, or an instant that does not fit the clock this system
/// keeps.
fn span_of(
    arrival: i64,
    after_ms: u32,
    duration_ms: u32,
    earliest: &mut u32,
) -> Result<(i64, i64), RefusalReasonWire> {
    if after_ms < *earliest || duration_ms == 0 {
        return Err(RefusalReasonWire::BAD_TIMES);
    }
    *earliest = after_ms;
    // Checked rather than saturating: an offset that does not fit is a request
    // for an instant this machine does not have, and clamping it to one the
    // machine does have would run a script nobody asked for.
    let start = i64::from(after_ms)
        .checked_mul(MS_NS)
        .and_then(|offset| arrival.checked_add(offset))
        .ok_or(RefusalReasonWire::BAD_TIMES)?;
    let end = i64::from(duration_ms)
        .checked_mul(MS_NS)
        .and_then(|length| start.checked_add(length))
        .ok_or(RefusalReasonWire::BAD_TIMES)?;
    Ok((start, end))
}

/// Write `plan` into the slot as the schedule this session runs, and answer with
/// the epoch it was written under.
///
/// One statement, so a schedule is never half replaced: what a reader finds is
/// the schedule it had or the schedule it has.
///
/// `bump` is whether this write opens a fresh epoch, and it is the caller's
/// because the epoch belongs to the wake rather than to the write: a wake
/// publishes once, so it bumps once, and a second acceptance inside one intake
/// window lands under the epoch the first opened. The bump is what makes a
/// change observable when the steps happen to look the same.
///
/// `engaged` is the caller's, and it is the whole difference between the two
/// shapes of acceptance. A script that opens an engagement leaves it false: an
/// accepted script says what the machine is to do and not that it is under
/// command, and what makes it engaged is the arming that follows. A script that
/// replaces one leaves it true: the machine is under command throughout, and a
/// pass through false would pause the goal stream the driver's dead-man is fed
/// by.
///
/// Whole and not merged: the new plan is the entire commanded future, so
/// overlay windows the old schedule still had open end with the old epoch.
fn store(
    slot: &mut SessionStateWire,
    script_id: u32,
    plan: &SessionScheduleWire,
    engaged: bool,
    bump: bool,
) -> u32 {
    slot.set_script_id(script_id);
    let held = slot.schedule().epoch();
    let epoch = if bump { held.wrapping_add(1) } else { held };
    let schedule = slot.schedule_mut();
    *schedule = plan.clone();
    schedule.set_engaged(engaged);
    schedule.set_epoch(epoch);
    epoch
}

/// Record every raise that arrived, as a fault the machine has.
///
/// Only the raises the classifier answers with something: a rejected command and
/// an abandoned move are remarks about a plan rather than conditions of the
/// machine, they already stand on the channel that carried them, and narrating
/// them here would make the timeline a second copy of that channel.
///
/// Recorded as read, one raise at a time: the slot and the message window are
/// separate borrows, so no intermediate buffer can drop evidence.
fn record_raises(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    now: i64,
    budgets: &Budgets,
) {
    for message in dial.inputs.fault.new_msgs() {
        // Bytes that describe no raise name no condition and no instant, so
        // there is nothing to record.
        let Ok(raise) = message.validate() else {
            counters.undecodable_inbound += 1;
            continue;
        };
        if matches!(fault::response(raise.kind), ResponseKind::None) {
            continue;
        }
        record_fault(
            dial.states.sess,
            counters,
            &Evidence {
                kind: raise.kind,
                joint: raise.joint,
                detail: raise.detail,
                at_ns: raise.time.as_nanos(),
            },
            now,
            budgets,
        );
    }
}

/// Record every edge the driver reported, and act on the ones that answer
/// something this session asked for.
///
/// Two of the driver's edges are conditions of the machine and become faults
/// like any other; the confirmation of a release and the three endings of an
/// engagement are not conditions at all but the answers this session is waiting
/// for, so they stop the asking and move the phase rather than being recorded.
/// The rest are the driver working as designed.
///
/// An answer to an engagement nobody has open is counted and ignored, the way a
/// confirmation of a release nobody commanded is: a re-issue whose first
/// datagram was answered after all draws a second answer, and the phase it
/// would move has already moved.
fn record_events(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    now: i64,
    budgets: &Budgets,
) {
    for message in dial.inputs.evt.new_msgs() {
        // Bytes that describe no event name no edge, so nothing happened as far
        // as this cog can tell.
        let Ok(event) = message.validate() else {
            counters.undecodable_inbound += 1;
            continue;
        };
        let at_ns = event.time.as_nanos();
        if matches!(event.kind, EventKind::TorqueOffConfirmed) {
            // A confirmation of a release nobody commanded is the driver's own
            // dead-man reporting, and there is nothing here to end.
            if session_ladder::confirmed(dial.states.sess) {
                let report = Report {
                    time_ns: at_ns,
                    kind: ReportKind::TorqueOffConfirmed,
                    a: 0,
                    b: 0,
                    detail: 0.0,
                };
                narrate(dial.states.sess, counters, &report);
            }
            continue;
        }
        if let Some(answered) = engagement_answer(event.kind) {
            answer_engagement(dial, counters, answered, at_ns, budgets);
            continue;
        }
        let Some(kind) = session_ladder::fault_of_event(event.kind) else {
            continue;
        };
        record_fault(
            dial.states.sess,
            counters,
            &Evidence {
                kind,
                // Every condition the driver reports about itself is about the
                // machine rather than one servo: the id an event carries names
                // the transaction it failed on, and the vocabulary has a number
                // for "no single servo" that a bus row cannot say.
                joint: JointRef::None,
                detail: 0.0,
                at_ns,
            },
            now,
            budgets,
        );
    }
}

/// What one of the driver's three engagement endings says became of the ask.
///
/// Stated once here so the phase each ending lands in is decided in one place.
fn engagement_answer(kind: EventKind) -> Option<Engaged> {
    match kind {
        EventKind::EngageConfirmed => Some(Engaged::Confirmed),
        EventKind::EngageUnconfirmed => Some(Engaged::Unconfirmed),
        EventKind::EngageDeclined => Some(Engaged::Declined),
        _ => None,
    }
}

/// What became of an engagement this session asked for.
#[derive(Clone, Copy)]
enum Engaged {
    /// Every named row read its enable back: the machine is holding where it
    /// stood, and what happens to it from here is the schedule's.
    Confirmed,
    /// A row did not. The rows that did confirm are believed torqued, so the
    /// machine may be holding with nothing driving it: it is let go of and
    /// latched, as an engagement that failed after its enable write always was.
    Unconfirmed,
    /// The driver wrote nothing and gave up on the ask. The machine is exactly
    /// where it was, so the session rests and the next script tries again.
    Declined,
}

/// Answer an engagement the driver has said what became of.
///
/// Only while one is open. A late duplicate for an engagement already answered
/// is counted and dropped: the ask is over, and re-deciding a phase from it
/// would move a machine on the strength of an answer to a question nobody is
/// still asking.
fn answer_engagement(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    answered: Engaged,
    at_ns: i64,
    budgets: &Budgets,
) {
    if !dial.states.sess.engage_pending() {
        counters.engage_strays += 1;
        return;
    }
    dial.states.sess.set_engage_pending(false);
    let entered = match answered {
        Engaged::Confirmed => session_bus::enter(dial.states.sess, SessionPhaseWire::ACTIVE),
        // The release is commanded here and published by the bus half, which
        // every wake that owes one does: one datagram, decided in one place.
        Engaged::Unconfirmed => {
            session_ladder::command_release(dial.states.sess, at_ns, budgets);
            session_bus::enter(dial.states.sess, SessionPhaseWire::PARKED)
        }
        Engaged::Declined => session_bus::enter(dial.states.sess, SessionPhaseWire::RESTING),
    };
    narrate(dial.states.sess, counters, &phase_report(&entered, at_ns));
}

/// Record what the driver's rotating read saw: the rail picture every report
/// refreshes, and a fault for the bits that say something.
///
/// Every report lands in the picture the torque-on gate judges, whatever its
/// bits say and whatever phase the machine is in. The classification is
/// `reachy-motion`'s, the same judgement the decision tick
/// makes over its own health poll. What is here is the bookkeeping the tick does
/// with its mask instead: a hardware-error byte latches in the servo, so the
/// rotation carries it on every pass for the rest of the session, and a host
/// that recorded each pass would fill its timeline with one standing condition.
/// The row is remembered as having been recorded and is never recorded again --
/// nothing clears it, because nothing clears the servo's own byte either, and
/// there is no automatic recovery from a fault.
fn record_readings(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    now: i64,
    budgets: &Budgets,
) {
    for message in dial.inputs.readings.new_msgs() {
        // Bytes that describe no reading name no servo and no byte.
        let Ok(reading) = message.validate() else {
            counters.undecodable_inbound += 1;
            continue;
        };
        // A reading filed under an id this configuration does not have is
        // evidence about no servo this session commands.
        let Some(row) = row_of_id(reading.id) else {
            continue;
        };
        let Some(joint) = joints::joint_ref(row) else {
            continue;
        };
        // The picture the torque-on gate judges, kept in every phase: this is
        // the whole of what makes engaging arithmetic rather than eighteen
        // transactions on the wake path.
        session_bus::record_reading(
            dial.states.sess,
            joint,
            reading.bits,
            reading.volts,
            reading.sample_time.as_nanos(),
        );
        let Some(kind) = fault::fault_of_health(row, reading.bits) else {
            continue;
        };
        // The slot was read back as a session at the top of this execution, so
        // a set of servos in it is a set of servos.
        let mut recorded = dial
            .states
            .sess
            .health_faulted()
            .to_known()
            .expect("the slot reads back as a session");
        if joints::flags::contains(recorded, joint) {
            continue;
        }
        joints::flags::insert(&mut recorded, joint);
        dial.states
            .sess
            .set_health_faulted(JointFlagsWire::from(recorded));
        record_fault(
            dial.states.sess,
            counters,
            &Evidence {
                kind,
                joint,
                // The byte itself, as the number a reader of the row sees. It is
                // not a magnitude, and the kind is what says so.
                detail: f64::from(reading.bits),
                at_ns: reading.sample_time.as_nanos(),
            },
            now,
            budgets,
        );
    }
}

/// Answer a slot that came back as no session at all: let go of the machine,
/// and latch.
///
/// A cleared slot is a session that has not begun, and starting one over is the
/// wrong answer to memory gone wrong: the machine on the other side of it may be
/// torqued, may be running a schedule the tick is still streaming, and may owe a
/// release nobody has confirmed, and none of that is recoverable from a record
/// that no longer says which. So the machine is commanded limp and the tick is
/// told nobody is engaged, and the phase latches: a session with no idea what it
/// was commanding is not one to commission a machine again, and nothing engages
/// a parked machine until an operator has been. The same answer an unreadable
/// sequence snapshot gets, for the same reason.
fn lost_the_session(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    now: i64,
    budgets: &Budgets,
) {
    session_ladder::command_release(dial.states.sess, now, budgets);
    let entered = session_bus::enter(dial.states.sess, SessionPhaseWire::PARKED);
    narrate(dial.states.sess, counters, &phase_report(&entered, now));
    // A fresh epoch on a record nobody is engaged on: the tick is holding
    // whatever the last schedule it read asked for, and this is the news that
    // stops it. Bumped rather than left where the cleared slot put it, because
    // the epoch is what makes the change news to a consumer.
    {
        let schedule = dial.states.sess.schedule_mut();
        let epoch = schedule.epoch().wrapping_add(1);
        schedule.set_engaged(false);
        schedule.set_epoch(epoch);
    }
    send_schedule(dial, counters, now);
}

/// The driver is still there, and this is the first execution.
///
/// Two anchors and one verdict. The sample stream's freshest instant is noted
/// from a view that never wakes this cog, so what a wake reads is the newest
/// sample the driver published before it; and a silence past what the
/// configuration allows is the bus declared failed -- the one condition the
/// driver cannot report about itself, because a driver that has stopped
/// publishing has stopped reporting.
fn watch(dial: &mut SessionDial<'_>, counters: &mut SessionCounters, now: i64, budgets: &Budgets) {
    session_ladder::note_start(dial.states.sess, now);
    if let Some(sample) = dial.inputs.sample.latest() {
        match sample.validate() {
            // The nominal instant, which is the cycle the driver produced --
            // not when the read finished, and not when this cog read it.
            Ok(sample) => {
                session_ladder::note_sample(dial.states.sess, sample.nominal_time.as_nanos());
            }
            Err(_) => counters.undecodable_inbound += 1,
        }
    }
    let Some(silent_ns) = session_ladder::silent_for(dial.states.sess, now, budgets) else {
        return;
    };
    record_fault(
        dial.states.sess,
        counters,
        &Evidence {
            kind: FaultKind::BusFailure,
            joint: JointRef::None,
            detail: 0.0,
            at_ns: now,
        },
        now,
        budgets,
    );
    let report = Report {
        time_ns: now,
        kind: ReportKind::BusFailureDeclared,
        // The declaration's evidence is the silence itself, which is the
        // measurement below; there is no second number to carry.
        a: 0,
        b: 0,
        detail: seconds(silent_ns),
    };
    narrate(dial.states.sess, counters, &report);
}

/// One condition, and where the evidence for it came from.
struct Evidence {
    /// What was observed.
    kind: FaultKind,
    /// The servo it is about, or [`JointRef::None`] where it is about the
    /// machine.
    joint: JointRef,
    /// The number the evidence carried, meaning per kind.
    detail: f64,
    /// The instant the evidence is about.
    at_ns: i64,
}

/// Record a fault and answer it.
///
/// The one path every kind of evidence joins, so a condition is narrated and
/// answered the same way wherever it was observed: the timeline row first,
/// because it is the record, then the response the library classifies the
/// condition with. A response already being carried out is not selected again
/// and not narrated again.
fn record_fault(
    slot: &mut SessionStateWire,
    counters: &mut SessionCounters,
    evidence: &Evidence,
    now: i64,
    budgets: &Budgets,
) {
    let report = Report {
        time_ns: evidence.at_ns,
        kind: ReportKind::FaultRecorded,
        a: u32::from(kind_number(evidence.kind)),
        b: u32::from(joint_number(evidence.joint)),
        detail: evidence.detail,
    };
    counters.faults_recorded += 1;
    narrate(slot, counters, &report);

    // When the ladder answers nothing, no held script needs attention: the paths
    // that reach it have already resolved anything held or refuse arrivals
    // outright, so nothing is waiting when a second condition arrives behind.
    let Some(answered) = session_ladder::answer(slot, evidence.kind, now, budgets) else {
        return;
    };
    counters.responses_taken += 1;
    let report = Report {
        time_ns: now,
        kind: ReportKind::ResponseTaken,
        a: u32::from(ResponseKindWire::from(answered.response).0),
        b: u32::from(kind_number(evidence.kind)),
        detail: 0.0,
    };
    narrate(slot, counters, &report);
    if let Some(entered) = answered.entered {
        narrate(slot, counters, &phase_report(&entered, now));
    }
    // `ending_by_fault` must not stand over a parked session: parked never
    // leaves, so the flag would be a permanent claim that the session is on its
    // way to rest. Entering the phase is what clears it, wherever that was
    // decided; what is left here is not to set it again, and to leave a script
    // held for that machine for the drain to answer `parked`.
    if slot.phase() == SessionPhaseWire::PARKED {
        return;
    }
    if !ends_toward_rest(answered.response) {
        return;
    }
    slot.set_ending_by_fault(true);
    if slot.pending_valid() {
        let held = slot.pending().script_id();
        slot.set_pending_valid(false);
        let report = refusal(held, RefusalReasonWire::FAULT_ENDING, counters, now);
        narrate(slot, counters, &report);
    }
}

/// Whether `response` is a fault response that ends the session toward rest.
///
/// Both tests are needed: disposition alone would include the group-scoped
/// de-torque, which disposes to `Rest` but is not a session ending. Stated over
/// the library's functions rather than over the response names so that a
/// response added to the doctrine is classified by the crate that owns the
/// classification.
fn ends_toward_rest(response: ResponseKind) -> bool {
    matches!(
        maneuver_of(response),
        Some(Maneuver::SlowStow | Maneuver::ImmediateAllTorqueOff)
    ) && ending::disposition(ending::answering(response)) == Disposition::Rest
}

/// The survey's verdict, as the one row that says what parked the machine.
///
/// The verdict itself is a state slot nothing publishes -- where a live debugger
/// reads it, and where it stays. What goes on the wire is the headline: which
/// failure, which servo it names, and the kind's own number. A reader of the
/// report stream then learns which servo the survey refused the machine over,
/// which is the question a parked run leaves an operator holding.
///
/// A verdict whose fields do not read back as a failure is narrated as exactly
/// that: `verdict_unreadable` is a failure kind of its own, and a parked machine
/// with no row at all is the silence this closes. Nothing is assembled out of
/// the fields that are still there -- but the refusal's own number goes in
/// `detail`, because a slot nobody wrote, a slot holding another build's bytes
/// and a slot whose evidence does not suit its kind are three different stories
/// and this row is the only place they are told apart.
fn commission_report(slot: &SessionStateWire, now: i64) -> Report {
    let read = match slot.commission().failure().validate() {
        Ok(snap) => verdict::read(snap).map_err(VerdictError::code),
        Err(_) => Err(verdict::BYTES_UNREADABLE),
    };
    let (kind, id, detail) = match read {
        Ok(failure) => (
            failure.kind(),
            failure.context().id,
            verdict::headline(&failure),
        ),
        Err(code) => (SeqFailureKind::VerdictUnreadable, 0, f64::from(code)),
    };
    Report {
        time_ns: now,
        kind: ReportKind::CommissionFailed,
        a: u32::from(SeqFailureKindWire::from(kind).0),
        b: u32::from(id),
        detail,
    }
}

/// A phase change, as the story of one.
fn phase_report(entered: &Entered, now: i64) -> Report {
    Report {
        time_ns: now,
        kind: ReportKind::PhaseChanged,
        a: u32::from(entered.to.0),
        b: u32::from(entered.from.0),
        detail: 0.0,
    }
}

/// Publish the schedule where what the machine is under command to do has
/// changed, and say so.
///
/// One site and one edge: a machine is under command while it is running a
/// schedule and while it is being carried down out of one, and in no other
/// phase, so the engagement taking hold and the machine being let go of are the
/// two changes, and every other way out of those phases -- a response that parks
/// it, a fault that ends it -- is the second change reached differently. The
/// slot's own `engaged` is what the comparison is against, which is what makes
/// this idempotent: nothing is republished between changes, and a consumer
/// reading the latest message has the whole of what the session asked for.
///
/// A stow the wind-down commands is the one republish while the machine stays
/// engaged, and it is `run_winddown`'s: what changed there is the schedule
/// rather than whether anybody is running one. A wind-down is never a wake that
/// took a replacement -- a machine being carried down out of a fault refuses
/// scripts -- so the two never publish on one wake.
///
/// `replaced` is a schedule this wake's intake swapped out, which is a change
/// the edge cannot see: the machine was under command before it and stays under
/// command after it. That accept opened this wake's epoch already, so what
/// happens here is the send and not a second bump -- one epoch, one datagram,
/// and the row intake narrated names the schedule that went out. A replacement
/// whose schedule was already over arrives here with the edge fired too, and
/// then the same one datagram carries the machine being let go of.
fn publish_schedule(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    now: i64,
    replaced: bool,
) {
    let commanded = matches!(
        dial.states.sess.phase(),
        SessionPhaseWire::ACTIVE | SessionPhaseWire::WINDING_DOWN
    );
    let edge = dial.states.sess.schedule().engaged() != commanded;
    if !edge && !replaced {
        return;
    }
    {
        let schedule = dial.states.sess.schedule_mut();
        if !replaced {
            let epoch = schedule.epoch().wrapping_add(1);
            schedule.set_epoch(epoch);
        }
        schedule.set_engaged(commanded);
    }
    send_schedule(dial, counters, now);
}

/// Publish the schedule the slot holds, and say so.
///
/// The one place the channel is written. The whole record goes out, with
/// whatever epoch the slot has: the epoch is what makes a change news to a
/// consumer holding the last one when two schedules happen to look alike -- a
/// fresh engagement of the same script is a fresh base to move from, not a
/// continuation -- and bumping it belongs to whoever decided what changed.
fn send_schedule(dial: &mut SessionDial<'_>, counters: &mut SessionCounters, now: i64) {
    let epoch = dial.states.sess.schedule().epoch();
    let steps = dial.states.sess.schedule().steps().len();
    {
        let out = &mut dial.outputs.sched;
        // The record as the slot holds it, copied whole: a schedule is state, so
        // what goes out is what the session has rather than a summary of it.
        *out.msg_mut() = dial.states.sess.schedule().clone();
        out.mark_for_publish();
    }
    let report = Report {
        time_ns: now,
        kind: ReportKind::SchedulePublished,
        a: epoch,
        b: u32::try_from(steps).unwrap_or(u32::MAX),
        detail: 0.0,
    };
    narrate(dial.states.sess, counters, &report);
}

/// Take one step of the stow maneuver, if one is running.
///
/// The two controlled responses of the doctrine, carried out: the machine is
/// asked to fold itself, one schedule at a time, until it is measured folded or
/// the one clock the maneuver was opened with is spent. What the maneuver
/// decides is `reachy-motion`'s and what a wake does about it is here -- publish
/// the stow it asked for, or record the ending and let go of the machine.
///
/// The evidence is the freshest sample the driver published, which is the only
/// thing that says whether the head actually came down: a stow measured at the
/// pose is a head carried down under control, and that is the one claim this
/// record must never make wrongly.
///
/// Concluding commands the de-torquing rather than driving the orderly release.
/// Every conclusion ends with torque off and nothing gates that: the machine has
/// either folded, or defeated the fold, or run out of clock, and none of those is
/// a machine to hold torque on for a settle it was never promised. The phase the
/// maneuver's disposition names is entered here, and the release goes out on this
/// same wake.
fn run_winddown(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    now: i64,
    budgets: &Budgets,
) {
    if !session_stow::running(dial.states.sess) {
        return;
    }
    let stowed = dial
        .inputs
        .sample
        .latest()
        .and_then(|sample| sample.validate().ok())
        .is_some_and(session_stow::stowed);
    match session_stow::step(dial.states.sess, now, stowed) {
        session_stow::Step::Nothing => {}
        // A maneuver nobody can read is a maneuver nobody can bound, and what it
        // was last commanded to do is a stow with no clock to end it. So it is
        // taken out of the record and the machine is let go of where it stands,
        // and the phase latches: nothing engages a machine an operator has not
        // seen, and a record gone wrong mid-wind-down is exactly that.
        session_stow::Step::Ungoverned => {
            session_stow::abandon(dial.states.sess);
            session_ladder::command_release(dial.states.sess, now, budgets);
            let entered = session_bus::enter(dial.states.sess, SessionPhaseWire::PARKED);
            narrate(dial.states.sess, counters, &phase_report(&entered, now));
        }
        session_stow::Step::Commanded => send_schedule(dial, counters, now),
        session_stow::Step::Concluded(concluded) => {
            let report = Report {
                time_ns: now,
                kind: ReportKind::WinddownOutcome,
                a: u32::from(WindDownOutcomeWire::from(concluded.outcome).0),
                // Non-zero for a machine nothing engages until an operator has
                // been, which is the one thing a reader of this row acts on.
                b: u32::from(matches!(concluded.disposition, Disposition::Park)),
                detail: seconds(concluded.left_ns),
            };
            narrate(dial.states.sess, counters, &report);
            session_ladder::command_release(dial.states.sess, now, budgets);
            let entered = session_bus::enter(dial.states.sess, concluded.phase());
            narrate(dial.states.sess, counters, &phase_report(&entered, now));
        }
    }
}

/// Say that a commanded release has gone unacknowledged for longer than its
/// budget.
///
/// Said once per budget for as long as it stands, and the commanding is never
/// interrupted by it: the budget bounds the reporting, because a release nobody
/// can confirm is the operator's problem and not a reason to stop asking. The
/// driver's own reading of which rows are still holding travels on its event
/// channel; what this says is that the session has had no acknowledgement for
/// any of them, which is why it names every row.
fn say_unconfirmed(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    now: i64,
    budgets: &Budgets,
) {
    let Some(spent_ns) = session_ladder::overdue_release(dial.states.sess, now, budgets) else {
        return;
    };
    let report = Report {
        time_ns: now,
        kind: ReportKind::TorqueOffUnconfirmed,
        a: u32::from(JointFlagsWire::from(joints::flags::all()).0),
        b: 0,
        detail: seconds(spent_ns),
    };
    narrate(dial.states.sess, counters, &report);
}

/// Publish the release this wake owes the driver.
///
/// The only datagram such a wake publishes: the output slot carries one, and a
/// machine that must let go is what it carries. Idempotent by construction --
/// the driver's latch is a state and this is the same command every wake --
/// which is what makes republishing it on a lossy channel safe.
fn publish_release(dial: &mut SessionDial<'_>) {
    let out = &mut dial.outputs.cmd;
    let msg = out.msg_mut().clear_valid();
    msg.kind = SessionCmdKind::TorqueOffNow;
    out.mark_for_publish();
}

/// Publish the engagement this wake's bus turn cleared.
///
/// The rows the torque-on gate passed, and nothing else: the driver handles them
/// in one cycle of its own and needs nothing more from here. Idempotent, and
/// republished every wake until the driver answers: a re-issue of the same rows
/// is liveness and draws the same answer again.
fn publish_engage(dial: &mut SessionDial<'_>, rows: JointFlags) {
    let out = &mut dial.outputs.cmd;
    let msg = out.msg_mut().clear_valid();
    msg.kind = SessionCmdKind::EngageNow;
    msg.rows = rows;
    out.mark_for_publish();
}

/// Take one turn at the bus, and say what it came to.
///
/// The session's other half: the sequences that establish the machine and the
/// release that lets go of it, driven one transaction per wake through
/// `session_bus`. A schedule that has run out is what turns the second into the
/// first's ending, tested here before any sequence is stepped so that the
/// release's own first step is taken by the wake that decided the session was
/// over. What is decided there is written into the slot there; what a decision
/// *says* is narrated here, because the ring and the totals are this module's.
///
/// A wake that already owes the driver a de-torquing steps no sequence and
/// publishes the release: the outstanding transaction was abandoned when the
/// release was commanded, and a machine that must let go is not one to ask
/// anything else of. The answers that arrived are still read, so a late outcome
/// is counted rather than left to wake this cog again.
///
/// A turn that *asks* for the release is an engagement that stopped with servos
/// possibly holding. It is commanded here rather than there, because commanding
/// it is spending a budget out of the configuration and the ladder is where that
/// is done.
///
/// One message leaves either way. The release and the group-scoped de-torque's
/// next write are decided here and outrank everything -- making a group let go
/// outranks establishing anything -- and what a turn itself owes is
/// [`conclude_turn`]'s.
///
/// The engagement and a transaction never coexist: the engaging arm issues the
/// fallback poll's read or asks for the `engage_now`, never both.
fn run_bus(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    now: i64,
    timing: &Timing,
    budgets: &Budgets,
) {
    let answer = take_answer(dial, counters);
    if session_ladder::owes_release(dial.states.sess) {
        publish_release(dial);
        return;
    }
    // A schedule that has run out ends the session, which is a phase change and
    // no datagram: the release's own first step is taken by this same execution,
    // and a group-scoped de-torque still draining keeps the aux path until it is
    // done.
    if let Some(entered) = session_bus::ended(dial.states.sess, now) {
        narrate(dial.states.sess, counters, &phase_report(&entered, now));
    }
    // The group-scoped de-torque takes the wake whenever the aux path is free
    // for it: a drain settling its own write, or a fresh row to ask about with
    // nothing outstanding. A sequence mid-transaction keeps its turn, because
    // one transaction is outstanding at a time and overwriting the record would
    // leave a sequence waiting on a datagram that never went out. The wait is
    // bounded by that sequence's own delivery budgets, and the phases a degrade
    // usually arrives in -- resting, active -- drive no sequence at all.
    if session_ladder::owes_degrade(dial.states.sess)
        && (dial.states.sess.degrade_pending() || !dial.states.sess.aux().active())
    {
        if !drain_degrade(dial, counters, answer, now, timing, budgets)
            && session_bus::keep_alive_owed(dial.states.sess)
        {
            // A drain settling its last row, or waiting on a write nobody has
            // answered yet, publishes nothing -- and a wake in the arming or the
            // release that says nothing to the driver is a wake its hold timeout
            // counts against a machine that is holding.
            publish_keep_alive(dial);
        }
        return;
    }
    let turn = session_bus::run(dial.states.sess, answer, now, timing);

    record_delivery(dial.states.sess, counters, &turn.delivery, now);
    // A servo that never acknowledged its own release may still be holding,
    // which is a condition of the machine and goes down the path every other one
    // takes: the library answers it with the de-torquing that needs no
    // acknowledgement, and the machine latches. The session does not end at rest
    // over it.
    if let Some(joint) = turn.unreleased {
        record_fault(
            dial.states.sess,
            counters,
            &Evidence {
                kind: FaultKind::TorqueOffUnconfirmed,
                joint,
                detail: 0.0,
                at_ns: now,
            },
            now,
            budgets,
        );
        if session_ladder::owes_release(dial.states.sess) {
            publish_release(dial);
        }
        return;
    }
    if let Some(ended) = turn.ended {
        let report = Report {
            time_ns: now,
            kind: ReportKind::SessionEnded,
            a: dial.states.sess.script_id(),
            b: u32::from(JointFlagsWire::from(ended.unmeasured).0),
            detail: ended.deviation,
        };
        narrate(dial.states.sess, counters, &report);
    }
    conclude_turn(dial, counters, &turn, now, budgets);
}

/// Say what a turn at the bus came to, and publish the one datagram it owes.
///
/// The tail of a turn, in one place because two callers reach it: a wake's own
/// turn, and the drain that opens an engagement after that turn has been taken.
/// A second copy of this would be a second opinion about what a turn's datagram
/// is.
///
/// The order is which datagram outranks which: the release, the transaction a
/// sequence asked for, the engagement a cleared gate asks for, and a keep-alive
/// on a wake that owes none of them. A keep-alive can therefore never displace
/// a transaction; an engagement may displace a keep-alive, which is what lets
/// the drain's step take a wake whose turn published one, and either datagram
/// is liveness to the driver so nothing the keep-alive said is lost.
fn conclude_turn(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    turn: &session_bus::Turn,
    now: i64,
    budgets: &Budgets,
) {
    if turn.silenced {
        counters.engage_silences += 1;
    }
    if let Some(entered) = turn.entered {
        // The survey's verdict, immediately before the phase row it explains.
        // Before, because a reader of the stream meets the row that says the
        // machine was refused and then the row that says why -- and because
        // this is the only ordering enforceable anywhere: the phase row is
        // narrated here, and nothing else about a parked survey ever leaves the
        // session.
        if entered.to == SessionPhaseWire::PARKED && entered.from == SessionPhaseWire::STARTING {
            let report = commission_report(dial.states.sess, now);
            narrate(dial.states.sess, counters, &report);
        }
        narrate(dial.states.sess, counters, &phase_report(&entered, now));
    }
    if turn.release {
        session_ladder::command_release(dial.states.sess, now, budgets);
        publish_release(dial);
        return;
    }
    if let Some(datagram) = turn.delivery.datagram {
        publish_cmd(dial, &datagram);
    } else if let Some(rows) = turn.engage {
        publish_engage(dial, rows);
    } else if session_bus::keep_alive_owed(dial.states.sess) {
        publish_keep_alive(dial);
    }
}

/// Answer the script this session is holding, on the phase this wake left the
/// machine in.
///
/// Conditioned on the phase rather than on a named transition, so every path
/// that reaches a phase able to answer one drains without being listed here.
/// `resting` with nothing outstanding takes it as an acceptance and opens the
/// engagement; `active` takes it as a replacement; `parked` clears the slot and
/// answers the sender, because nothing engages a parked machine and a script
/// held for it would be held for ever. Every other phase leaves it held, and so
/// does a `resting` machine that still owes a release or is still draining a
/// group-scoped de-torque: a machine being let go of is not one to ask anything
/// else of, and the wake that finishes letting go drains it instead.
///
/// The engagement an acceptance opens takes its first bus step here, on this
/// same wake: `run_bus` has already had its turn, and readiness is the whole
/// requirement. Safe because that turn issued nothing -- the phases this drains
/// from drive no sequence, a release owed holds, and a drain still writing holds
/// -- so what the turn left in the output slot was a keep-alive or nothing.
///
/// Answers whether a replacement was taken, which the wake's one publish is
/// owed. `bumped` is whether the wake's intake already bumped the epoch, so a
/// drain and an intake in one wake share the one epoch that goes out.
fn drain_pending(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    now: i64,
    screens: &Screens,
    timing: &Timing,
    budgets: &Budgets,
    bumped: bool,
) -> bool {
    if !dial.states.sess.pending_valid() {
        return false;
    }
    let phase = dial.states.sess.phase();
    let outstanding = dial.states.sess.aux().active() || dial.states.sess.degrade_pending();
    match phase {
        SessionPhaseWire::PARKED => {
            let held = dial.states.sess.pending().script_id();
            dial.states.sess.set_pending_valid(false);
            let report = refusal(held, RefusalReasonWire::PARKED, counters, now);
            narrate(dial.states.sess, counters, &report);
            return false;
        }
        SessionPhaseWire::RESTING
            if !session_ladder::owes_release(dial.states.sess) && !outstanding => {}
        SessionPhaseWire::ACTIVE => {}
        _ => return false,
    }

    // Taken out of the slot before it is decided: a script is answered once,
    // whatever the answer is, and a decision that refused it must not leave it
    // waiting for the next wake to refuse it again.
    let held = dial.states.sess.pending().clone();
    let held_id = dial.states.sess.pending().script_id();
    dial.states.sess.set_pending_valid(false);
    // The slot validated whole at the top of this execution, the script nested
    // in it included, so this cannot fail. Answered rather than dropped if it
    // ever does: a script nobody can read is a script nobody can act on, and the
    // row the hold promised is owed to the sender whichever way it turns out.
    // The number the slot carries is a field of its own and reads back whatever
    // the rest of the script did.
    let Ok(script) = held.validate() else {
        debug_assert!(false, "a held script read back as no script");
        let report = refusal(held_id, RefusalReasonWire::UNDECODABLE, counters, now);
        narrate(dial.states.sess, counters, &report);
        return false;
    };
    let answer = decide(dial.states.sess, script, counters, now, screens, bumped);
    let replaced = answer.report.kind == ReportKind::ScriptReplaced;
    narrate(dial.states.sess, counters, &answer.report);
    if let Some(entered) = answer.entered {
        narrate(dial.states.sess, counters, &phase_report(&entered, now));
        let turn = session_bus::open_engagement(dial.states.sess, now, timing);
        conclude_turn(dial, counters, &turn, now, budgets);
    }
    replaced
}

/// Carry out one row of the group-scoped de-torque, and say what it came to.
///
/// The doctrine's one response scoped to a group: the antenna pair is made to
/// let go, one verified write at a time, and the head keeps its presence
/// throughout. The write itself is `session_bus`'s -- it rides the same
/// outstanding-transaction machinery a sequence's asks do -- and what a wake
/// *says* is here, for the reason every other narration is.
///
/// A row that will not let go is answered by letting go of everything. The
/// evidence is a commanded torque-off nobody acknowledged, which is a condition
/// the library already classifies, so it goes down the same path every other
/// fault does and is answered with the immediate release: a group this session
/// cannot make limp is a machine it has stopped being able to command, and
/// asking the same servo again is the retry the doctrine forbids.
///
/// Answers whether the wake's one message went out, because a drain that
/// published nothing leaves the wake owing whatever it owed without it.
fn drain_degrade(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    answer: Option<BusResult>,
    now: i64,
    timing: &Timing,
    budgets: &Budgets,
) -> bool {
    let drain = session_bus::degrade(dial.states.sess, answer, now, timing);
    record_delivery(dial.states.sess, counters, &drain.delivery, now);
    let mut published = false;
    if let Some(joint) = drain.refused {
        record_fault(
            dial.states.sess,
            counters,
            &Evidence {
                kind: FaultKind::TorqueOffUnconfirmed,
                joint,
                detail: 0.0,
                at_ns: now,
            },
            now,
            budgets,
        );
        if session_ladder::owes_release(dial.states.sess) {
            publish_release(dial);
            published = true;
        }
        return published;
    }
    if let Some(rows) = drain.released {
        let report = Report {
            time_ns: now,
            kind: ReportKind::DegradeReleased,
            // The response whose maneuver is this de-torque, which is what the
            // row is read against: the maneuver has no number of its own in the
            // report vocabulary.
            a: u32::from(ResponseKindWire::from(ResponseKind::DegradeAntennas).0),
            b: u32::from(JointFlagsWire::from(rows).0),
            detail: 0.0,
        };
        narrate(dial.states.sess, counters, &report);
    }
    if let Some(datagram) = drain.delivery.datagram {
        publish_cmd(dial, &datagram);
        published = true;
    }
    published
}

/// Count and narrate what a wake's one transaction came to.
///
/// One place for one contract: a sequence's turn and the group-scoped
/// de-torque's drain deliver a datagram the same way, so a re-issue is counted
/// and a delivery given up on is narrated the same way for both. A delivery that
/// was given up on and counted by only one of them would be an aux failure
/// nobody could find.
fn record_delivery(
    slot: &mut SessionStateWire,
    counters: &mut SessionCounters,
    delivery: &Delivery,
    now: i64,
) {
    if delivery.retried {
        counters.aux_retries += 1;
    }
    if let Some(gave_up) = delivery.gave_up {
        counters.aux_failures += 1;
        let report = Report {
            time_ns: now,
            kind: ReportKind::AuxGaveUp,
            a: gave_up.corr,
            b: u32::from(gave_up.id),
            detail: seconds(gave_up.waited_ns),
        };
        narrate(slot, counters, &report);
    }
}

/// Publish the keep-alive this wake owes the driver.
///
/// Nothing is asked for: what the datagram carries is that this host is still
/// here, which is what holds the driver's hold timeout off a machine that is
/// holding torque with nothing streaming to it. Every accepted datagram counts,
/// so this is the one a wake with nothing else to say sends.
fn publish_keep_alive(dial: &mut SessionDial<'_>) {
    let out = &mut dial.outputs.cmd;
    let msg = out.msg_mut().clear_valid();
    msg.kind = SessionCmdKind::KeepAlive;
    out.mark_for_publish();
}

/// The answer to the transaction the slot is waiting on, out of the window of
/// outcomes that arrived.
///
/// Matched by correlation number, which is the whole join: an outcome naming a
/// number nothing is waiting on is a late duplicate of a request already settled
/// -- what a re-issued datagram produces when the first one was answered after
/// all -- and it is counted and dropped rather than fed to a sequence as an
/// answer to a question it has moved on from.
fn take_answer(dial: &mut SessionDial<'_>, counters: &mut SessionCounters) -> Option<BusResult> {
    let pending = dial.states.sess.aux();
    let waiting = pending.active();
    let wanted = pending.corr();
    // The record was written by this cog and validated on the way into this
    // execution, so an operation outside the vocabulary is memory gone wrong.
    let asked = pending.op().to_known();

    let mut answer = None;
    for message in dial.inputs.aux_out.new_msgs() {
        // Bytes that describe no outcome name no correlation number, so there is
        // nothing they could be an answer to.
        let Ok(outcome) = message.validate() else {
            counters.undecodable_inbound += 1;
            continue;
        };
        match asked {
            Some(op) if waiting && outcome.corr == wanted && answer.is_none() => {
                answer = Some(result_of(
                    op,
                    outcome.status,
                    outcome.value_kind,
                    outcome.value,
                    outcome.model,
                ));
            }
            _ => counters.aux_strays += 1,
        }
    }
    if asked.is_none() && waiting {
        counters.refused_state += 1;
    }
    answer
}

/// The outcome, as the vocabulary a sequencer classifies failures in.
///
/// The transaction that was asked is what says how a success reads: the driver
/// answers a ping with a model number, a read with a register value and a
/// verified write with nothing, and which of those an `ok` is cannot be told
/// from the outcome alone. Every failure carries its own evidence and needs no
/// help from the question.
///
/// Two statuses land on one arm deliberately. A frame that would not decode and
/// a bus that failed the transaction are both "something happened on the wire
/// and nothing about the servo was established" -- and the library's word for
/// that is never retried, which is the right answer for both: a corrupted answer
/// carries no evidence about what the servo did.
fn result_of(
    op: AuxOpKind,
    status: AuxStatus,
    value_kind: ValueShape,
    bits: u64,
    model: u16,
) -> BusResult {
    match status {
        AuxStatus::Ok => match op {
            AuxOpKind::Ping => BusResult::Pinged { model },
            AuxOpKind::ReadReg => BusResult::Value(value::carried(value_kind, bits)),
            AuxOpKind::WriteRegVerified => BusResult::Written,
            // An unverified write's answer is the servo's acknowledgement and
            // nothing about the register, so it is its own result: a step that
            // judges a write asks whether the register holds what it wrote, and
            // this cannot answer that question.
            AuxOpKind::WriteReg => BusResult::Acknowledged,
            // A datagram asking nothing is one this cog never issues, so an
            // answer to one is an answer to a question nobody asked: the
            // library's word for a transaction that never reached the bus.
            AuxOpKind::None => BusResult::DriverRefused,
        },
        AuxStatus::Timeout => BusResult::NoAnswer,
        AuxStatus::DecodeError | AuxStatus::WireError => BusResult::WireCorrupt,
        // Both are the driver putting nothing on the bus, and this cog issues
        // one transaction at a time — so a busy answer means the driver holds a
        // request this session does not think it has outstanding, which is a
        // disagreement about the wire and not something to try again.
        AuxStatus::Refused | AuxStatus::Busy => BusResult::DriverRefused,
        // The code the servo gave, which is a number and not a register value.
        AuxStatus::ServoError => BusResult::ServoError { code: bits as u8 },
        AuxStatus::VerifyMismatch => BusResult::VerifyMismatch {
            read_back: value::carried(value_kind, bits),
        },
    }
}

/// Publish the datagram this wake owes the driver.
///
/// One per execution, because an output slot carries one message: a wake with a
/// transaction to send therefore cannot also send a keep-alive, which is the
/// property the keep-alive rule is built on.
fn publish_cmd(dial: &mut SessionDial<'_>, datagram: &Datagram) {
    let out = &mut dial.outputs.cmd;
    // An output slot is reused memory holding whatever the previous execution
    // left, so the datagram starts cleared: a re-issue is the same bytes as the
    // issue it repeats, and a field carried over from an older datagram would
    // make it something else.
    let msg = out.msg_mut().clear_valid();
    msg.kind = SessionCmdKind::Aux;
    msg.corr = datagram.corr;
    datagram.txn.write(&mut msg.txn);
    out.mark_for_publish();
}

/// A count of nanoseconds as the seconds a report carries.
fn seconds(ns: i64) -> f64 {
    ns as f64 / 1e9
}

/// The number the fault vocabulary gives `kind`.
///
/// The number rather than the name: what a recorded story and a later build have
/// in common is the number, and a report is read out of a log by a reader that
/// has only the numbering.
fn kind_number(kind: FaultKind) -> u8 {
    FaultKindWire::from(kind).0
}

/// The number the joint vocabulary gives `joint`, zero where the raise names no
/// single servo.
fn joint_number(joint: JointRef) -> u8 {
    // The vocabulary's number rather than the bus row, so that "no single
    // servo" has a value of its own: there is a number for the absence of a
    // joint and no bus row for it.
    JointRefWire::from(joint).0
}

/// Append a report to the ring, and count what the ring said about it.
///
/// A full ring drops its oldest row, which is narration lost and never a record:
/// what mattered about a fault travelled on the channel that raised it. The
/// count of appends is what tells the publish below that this execution had
/// something to add. A slot describing a ring this build does not have is the slot
/// gone wrong: the ring is cleared, counted, and the report is written into the
/// empty one, because the story of the execution that found the damage is worth
/// more than the rows it cannot place.
fn narrate(slot: &mut SessionStateWire, counters: &mut SessionCounters, report: &Report) {
    counters.reports_narrated += 1;
    match push_report(slot, |row| report.write(row)) {
        Ok(true) => counters.reports_dropped += 1,
        Ok(false) => {}
        Err(_) => {
            counters.refused_state += 1;
            clear_timeline(slot);
            if let Ok(true) = push_report(slot, |row| report.write(row)) {
                counters.reports_dropped += 1;
            }
        }
    }
}

/// Publish the whole story, if this execution added to it.
///
/// Every message carries every row the timeline holds, so a reader has the
/// account of the run from whichever copy it saw first and there is no head of
/// the story to lose. Published only where a row was added: republishing an
/// unchanged story would be the same bytes over and over for nothing.
///
/// The rows are copied out under the ring's own reading, so a ring this build
/// cannot read publishes nothing and is cleared, exactly as an append that found
/// the same damage does. That reading happens on every execution, so a slot
/// describing a ring nobody wrote is found by the first wake after the damage
/// rather than by the first wake with something to say.
fn publish_timeline(
    dial: &mut SessionDial<'_>,
    counters: &mut SessionCounters,
    narrated_before: u64,
) {
    // How much of a story the slot says it holds is read back on every
    // execution, whether or not this one added to it: a ring this build has not
    // got is memory gone wrong, and an execution that says nothing is still an
    // execution that can find it.
    let Ok(held) = held_reports(dial.states.sess) else {
        counters.refused_state += 1;
        clear_timeline(dial.states.sess);
        return;
    };
    if counters.reports_narrated == narrated_before || held == 0 {
        return;
    }

    // The slot and the output are separate fields of the dial, so a row is read
    // and written in one step and nothing is carried between them: no execution
    // of this cog allocates.
    let out = &mut dial.outputs.report;
    // An output slot is reused memory holding whatever the previous execution
    // left, so the message starts cleared: a story is written from its first row
    // every time rather than over the rows of the last one.
    let msg = out.msg_mut();
    msg.clear();
    // The number about the ring in front of the reader, not the session's
    // lifetime total: a ring cleared for damage has dropped nothing of the story
    // it now holds, and the message describes that story.
    msg.set_dropped(dial.states.sess.timeline_dropped());
    let mut damaged = false;
    {
        let mut rows = msg.entries_mut();
        for nth in 0..held {
            match report_row(dial.states.sess, nth) {
                // The kind arrives with the row: the ring hands out a row only
                // where it narrates something this build knows, so there is one
                // refusal for a damaged ring rather than a second answer here.
                Ok(Some((row, kind))) => {
                    let report = Report {
                        time_ns: row.time().as_nanos(),
                        kind,
                        a: row.a(),
                        b: row.b(),
                        detail: row.detail(),
                    };
                    // The message holds a row per row of the ring, which a case
                    // beside the ring's own length pins. `try_grow` can fail
                    // only where the two have been made to differ.
                    let out_row = rows
                        .try_grow()
                        .expect("a story of no more rows than the ring holds");
                    report.write(out_row);
                }
                Ok(None) => break,
                Err(_) => {
                    damaged = true;
                    break;
                }
            }
        }
    }
    if damaged {
        counters.refused_state += 1;
        clear_timeline(dial.states.sess);
        return;
    }
    out.mark_for_publish();
    counters.reports_published += 1;
}

#[cfg(test)]
mod tests {
    //! What the screen's restated numbers are, measured against the schemas that
    //! own them, and how one transaction's outcome reads as a result.

    use super::{
        AuxOpKind, AuxStatus, BusResult, MAX_MOTIONS, ReportKind, SeqFailureKind,
        SeqFailureKindWire, SessionStateWire, ValueShape, commission_report, result_of, value,
        verdict,
    };
    use brenn_reachy__cogs__config_clk_rs::ClipLibraryConfigWire;
    use brenn_reachy__cogs__schedule_clk_rs::SessionScheduleWire;
    use brenn_reachy__cogs__script_clk_rs::ScriptWire;

    /// The bound an overlay's motion id is screened against is the library
    /// message's own capacity. Boxed because the message is most of a megabyte
    /// of frames and only its count is being asked about.
    #[test]
    fn the_motion_bound_is_the_library_messages_capacity() {
        let library = Box::new(ClipLibraryConfigWire::new());
        assert_eq!(library.motions().capacity(), MAX_MOTIONS);
    }

    /// A script may ask for exactly as many rows as a schedule holds, which is
    /// why the screen's row refusals cannot fire today.
    ///
    /// They are the guard for the day the two capacities diverge, and this is
    /// what says they are currently unreachable rather than forgotten: widen the
    /// script and `too_many_steps` starts answering senders; narrow the schedule
    /// and it starts refusing scripts that used to run.
    #[test]
    fn a_script_holds_exactly_the_rows_a_schedule_holds() {
        let script = ScriptWire::new();
        let schedule = SessionScheduleWire::new();
        assert_eq!(script.steps().capacity(), schedule.steps().capacity());
        assert_eq!(script.overlays().capacity(), schedule.overlays().capacity());
    }

    /// A verdict of `error` in a fresh session slot, as the survey leaves one.
    fn parked_on(error: &reachy_motion::seq::SeqError) -> SessionStateWire {
        let mut slot = SessionStateWire::new();
        verdict::write(slot.commission_mut().failure_mut().clear_valid(), error)
            .expect("a reachable verdict crosses");
        slot
    }

    /// Where a step this crate's cases file a failure under: a servo and a
    /// register, so the row's servo number is one a wrong field cannot produce.
    fn context() -> reachy_motion::seq::StepContext {
        reachy_motion::seq::StepContext::servo(reachy_motion::seq::SeqStepKind::Provision, 14)
    }

    /// The row a parked machine is narrated by, per failure shape.
    ///
    /// For a parked run this row is the only recorded clue an operator gets, so
    /// what each kind puts in `detail` is asserted rather than sampled: a
    /// refusal whose status code did not travel, or an unhealthy servo narrated
    /// by a zero, reads exactly like a correct row and would leave the next
    /// parked run saying the machine was refused without saying over what.
    #[test]
    fn the_commission_row_states_the_kind_the_servo_and_the_kinds_own_number() {
        let context = context();
        let readings = [11.4, 11.5, 11.6, 11.7, 11.8, 11.9, 12.0, 12.1, 12.2];
        let table: Vec<(reachy_motion::seq::SeqError, f64)> = vec![
            (
                reachy_motion::seq::SeqError::Refused { context, code: 7 },
                7.0,
            ),
            (
                reachy_motion::seq::SeqError::UnhealthyServo {
                    context,
                    bits: 0b0000_0001,
                },
                1.0,
            ),
            (
                reachy_motion::seq::SeqError::VerifyMismatch {
                    context,
                    expected: value::radians(-1.003_2),
                    read_back: value::radians(-1.001_7),
                },
                -1.001_7,
            ),
            (
                reachy_motion::seq::SeqError::VoltageLow {
                    context,
                    readings,
                    lowest: 11.4,
                    limit: 11.9,
                    waited: core::time::Duration::from_secs(3),
                },
                11.4,
            ),
        ];
        for (error, detail) in &table {
            let report = commission_report(&parked_on(error), 123);
            assert_eq!(report.kind, ReportKind::CommissionFailed, "{error}");
            assert_eq!(
                report.a,
                u32::from(SeqFailureKindWire::from(error.kind()).0),
                "{error}"
            );
            assert_eq!(report.b, 14, "the servo the failure names: {error}");
            assert_eq!(report.detail, *detail, "{error}");
            assert_eq!(report.time_ns, 123);
        }
    }

    /// A slot holding bytes that are not a verdict at all narrates that, by the
    /// number the schema's own refusal carries.
    ///
    /// The alternative is the silence this row exists to close: a parked machine
    /// with no verdict beside it. Which of the three unreadable stories it is --
    /// nobody wrote it, another build wrote it, its evidence does not suit its
    /// kind -- is what `detail` carries, and it is all a later reader gets.
    #[test]
    fn a_verdict_this_build_cannot_read_is_narrated_as_exactly_that() {
        let mut slot = parked_on(&reachy_motion::seq::SeqError::Refused {
            context: context(),
            code: 7,
        });
        // A failure kind from no vocabulary this build has, which is what
        // another build's bytes look like from here.
        slot.commission_mut()
            .failure_mut()
            .set_kind(SeqFailureKindWire(200));
        let report = commission_report(&slot, 123);
        assert_eq!(
            report.a,
            u32::from(SeqFailureKindWire::from(SeqFailureKind::VerdictUnreadable).0)
        );
        assert_eq!(report.b, 0, "no servo is named by bytes nobody can read");
        assert_eq!(
            report.detail,
            f64::from(verdict::BYTES_UNREADABLE),
            "the schema's own refusal, by number"
        );
    }

    /// And a slot nobody ever wrote is the third story, told apart from the
    /// other two by its own number rather than by a zero.
    #[test]
    fn a_verdict_nobody_wrote_is_narrated_by_the_refusals_own_number() {
        let report = commission_report(&SessionStateWire::new(), 123);
        assert_eq!(
            report.a,
            u32::from(SeqFailureKindWire::from(SeqFailureKind::VerdictUnreadable).0)
        );
        assert_eq!(
            report.detail,
            f64::from(super::VerdictError::NoFailure.code()),
            "a slot nobody wrote says so, rather than reading as a failure of no kind"
        );
    }

    /// An `ok` outcome reads as whatever the transaction that was asked answers
    /// with, and only the question says which of the three it is.
    ///
    /// In the crate rather than beside the wrapper cases: a session driven
    /// through its wrapper can be handed one outcome per execution, and what is
    /// asserted here is a total mapping over three operations and seven statuses.
    #[test]
    fn what_a_success_reads_as_is_what_the_question_was() {
        assert_eq!(
            result_of(AuxOpKind::Ping, AuxStatus::Ok, ValueShape::None, 0, 1200),
            BusResult::Pinged { model: 1200 },
        );
        let held = value::u8(4);
        assert_eq!(
            result_of(
                AuxOpKind::ReadReg,
                AuxStatus::Ok,
                held.shape(),
                held.bits(),
                0
            ),
            BusResult::Value(held),
        );
        assert_eq!(
            result_of(
                AuxOpKind::WriteRegVerified,
                AuxStatus::Ok,
                ValueShape::None,
                0,
                0
            ),
            BusResult::Written,
        );
        assert_eq!(
            result_of(AuxOpKind::WriteReg, AuxStatus::Ok, ValueShape::None, 0, 0),
            BusResult::Acknowledged,
            "a write nothing read back settles nothing about the register",
        );
        assert_eq!(
            result_of(AuxOpKind::None, AuxStatus::Ok, ValueShape::None, 0, 0),
            BusResult::DriverRefused,
            "a datagram asking nothing is one this cog never issued",
        );
    }

    /// Every failure carries its own evidence, and none of them needs the
    /// question: the same status reads the same way whatever was asked.
    #[test]
    fn every_failure_reads_as_itself_whatever_was_asked() {
        let read_back = value::i32(-7);
        let expected = [
            (AuxStatus::Timeout, BusResult::NoAnswer),
            (AuxStatus::DecodeError, BusResult::WireCorrupt),
            (AuxStatus::WireError, BusResult::WireCorrupt),
            (AuxStatus::Refused, BusResult::DriverRefused),
            (AuxStatus::Busy, BusResult::DriverRefused),
            (AuxStatus::ServoError, BusResult::ServoError { code: 32 }),
            (
                AuxStatus::VerifyMismatch,
                BusResult::VerifyMismatch { read_back },
            ),
        ];
        for (status, wanted) in expected {
            for op in [
                AuxOpKind::Ping,
                AuxOpKind::ReadReg,
                AuxOpKind::WriteRegVerified,
            ] {
                let bits = match status {
                    AuxStatus::ServoError => 32,
                    _ => read_back.bits(),
                };
                assert_eq!(
                    result_of(op, status, read_back.shape(), bits, 1200),
                    wanted,
                    "{status:?} answering {op:?}",
                );
            }
        }
    }
}
