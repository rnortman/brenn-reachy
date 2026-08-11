//! The bench's commands: what each one does to the machine, and in what order.
//!
//! Every command that moves the head has the same shape, and the shape is the
//! safety property. Nothing is remembered between invocations — each is a fresh
//! process — so a command walks the whole path: it commissions the machine
//! (which verifies the nine servos), polls to see where they are standing,
//! engages — pinning each joint where it stands and enabling torque, which
//! holds it there — and only then injects one `MoveTo` over the fixed-rate
//! loop. A machine that has drifted, been handled, or was never armed at all is
//! therefore re-established from scratch every time, and a command that cannot
//! establish it does not move anything.
//!
//! ## What a command does when a run goes wrong
//!
//! What an ending asks for is [`PumpError::class`], and the command carries it
//! out. A refusal leaves the machine holding: nothing was written, and where
//! the head is standing is where the last accepted command put it. So does a
//! defect of ours with a healthy machine — a step bound violated, a budget run
//! out — because an operator is standing here, `off` is one command away, and
//! the bench ticks nothing between commands, so a held pose cannot re-raise.
//!
//! A fault is the other case, and it comes in two shapes. Where the motors
//! still command — a grabbed head, a servo dropping out of a six-crank
//! platform — the machine winds down under control and then releases: yielding
//! is what helps a hand pushing the head down, and holding torque against it
//! would be a fault answered by holding. Where control itself is what stopped
//! being trustworthy — no feedback, no pose, no wire — torque comes off on the
//! spot, because a head held up by motors whose loop has stopped is the one
//! state this platform should never be left in.
//!
//! `off` is the exception, and deliberately: it releases torque, so re-arming
//! first would enable torque in order to switch it off. It drives the disarm
//! sequence against the machine as found. Nothing in that sequence refuses —
//! the nine positions it measures against the stow pose are a report of where
//! the head was when torque left it, not a condition on torque leaving.
//!
//! `provision` is the other exception: it writes a non-volatile register on a
//! machine whose torque is off and moves nothing, so arming it first would be
//! the one thing that stops the write from being accepted.
//!
//! Ports are the caller's: every command here takes one already open, so the
//! whole surface is exercisable against a scripted machine with no device in
//! sight. What a command prints goes out through a callback for the same
//! reason.

use core::time::Duration;

use dxl_proto::{HardwareError, counts_to_rad};
use reachy_bus::{
    Bus, BusPort, BusTiming, MapError, ServoMap, XactError, reg_for, value_kind, with_retry,
};
use reachy_kin::{neutral_head_pose, stow_head_pose};
use reachy_motion::disarm::STOW_ANTENNAS;
use reachy_motion::{
    CommissionSequencer, DisarmSequencer, DisarmSummary, EXPECTED_MODELS, EXPECTED_OPERATING_MODES,
    EngageSequencer, Fault, JointGroup, JointId, JointTargets, MotionCommand, MotionState,
    MoveDurations, PollCadence, PollSequencer, Posture, RegId, RegValue, ValueKind, Warp,
};

use crate::config::Resolved;
use crate::pump::{
    Clock, DISARM_ACTIONS, Disposition, ErrorClass, MotionPump, MoveSummary, Phase, PumpError,
    TickEvent, action_budget, commission_report, drive, drive_release, engage_phase, engage_report,
};
use crate::trace;

/// The shaping every bench move uses.
///
/// Min-jerk starts and ends at zero velocity and zero acceleration, which is
/// what keeps a move from stepping the goal registers at either end. Nothing in
/// this milestone wants the linear warp; it exists for tests that need a
/// constant rate.
const WARP: Warp = Warp::MinJerk;

/// How far the demo sweeps the body, degrees either side of square.
///
/// Well inside the bench's own yaw cap, because the demo is a thing to watch
/// rather than a limit to probe: a sweep that stopped at the cap would make
/// every run a boundary test of the envelope.
const DEMO_YAW_DEG: f64 = 30.0;

/// The joints `provision` writes: the two antennas, whose extended-position
/// mode is this project's own provisioning rather than the vendor's.
const PROVISIONED_JOINTS: [JointId; 2] = [JointId::AntennaRight, JointId::AntennaLeft];

/// Pings before a rebooted servo is reported as gone.
///
/// Spaced by the bus's own retry spacing, each one costing an exchange deadline
/// when nothing answers, so the budget is a couple of seconds at the shipped
/// timing. How long one of these servos takes to come back is not something
/// this project has measured — the count is set well above any restart a
/// reboot is likely to take, and the run prints what it actually waited.
const BOOT_POLLS: u32 = 100;

/// How far the demo swings the antennas, radians either side of upright.
///
/// A visible motion rather than a big one: the antennas turn freely and a whole
/// swing would read as a spin rather than as a gesture.
const DEMO_ANTENNA_RAD: f64 = 1.0;

/// The neutral configuration: head square and level at nominal height, body
/// square, antennas upright.
///
/// This is what `up` commands. It is the whole configuration and not just the
/// head pose, because the pose the machine is lifted *from* is stow, which
/// folds the antennas back — a lift that left them folded would raise a head
/// that is not up.
#[must_use]
pub fn neutral_targets() -> JointTargets {
    JointTargets {
        head_pose_body: neutral_head_pose(),
        body_yaw: 0.0,
        antennas: [0.0, 0.0],
    }
}

/// The stow configuration: the head lowered and pitched forward, the body
/// square, the antennas folded back.
///
/// The same pose the disarm sequence measures against, expressed as the
/// Cartesian command the tick path takes. Disarming compares joint angles and
/// this commands a head pose; both are derived from the one stow pose, so the
/// motion and the check cannot describe different places.
#[must_use]
pub fn stow_pose_targets() -> JointTargets {
    JointTargets {
        head_pose_body: stow_head_pose(),
        body_yaw: 0.0,
        antennas: STOW_ANTENNAS,
    }
}

/// A commissioned machine: the bus, and the last thing the poll saw.
///
/// Holding one is the evidence that the nine servos answered, were found
/// provisioned as recorded, and took their gains — established once, because
/// none of it changes while a process runs. Torque is not part of that
/// evidence: a commissioned machine is ordinarily limp, and stays that way
/// until something asks for a move.
///
/// It owns the bus for its whole life, which is what makes engaging and
/// releasing repeatable: the port never changes hands, so a wake word costs an
/// engage and not an open.
pub struct Commissioned<'a, P: BusPort> {
    resolved: &'a Resolved,
    bus: Bus<P>,
    posture: Posture,
    /// Whether `posture` still describes the machine. False after a release
    /// that did not measure all nine joints: the head went limp and settled
    /// somewhere nobody looked at.
    fresh: bool,
}

/// Commission the machine over `port` and take one look at where it stands.
///
/// The once-per-process ceremony — presence, identity, the provisioned
/// registers, the supply gate, the health sweep, the gains — followed by one
/// resting sweep, so the returned machine already knows a pose to engage from.
/// Around two hundred transactions, none of which touches torque in either
/// direction.
///
/// The phases are announced as they are entered: the supply gate alone can take
/// the whole of its configured budget, and a supervised run that said nothing
/// until the end would look hung at exactly the moment an operator is deciding
/// whether to reach for the power.
pub fn commission<'a, P: BusPort>(
    resolved: &'a Resolved,
    port: P,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<Commissioned<'a, P>, PumpError> {
    let mut bus = Bus::new(port, resolved.timing);
    let budget = action_budget(&resolved.arm);

    let mut commissioning = CommissionSequencer::new(&resolved.arm);
    let commission = drive(
        &mut bus,
        &resolved.map,
        &mut commissioning,
        clock,
        budget,
        &mut |step| line(&format!("  {step}")),
    )?;
    line(commission_report(&commission).trim_end());

    // The rail was read moments ago by the sweep that gated the gains writes,
    // so the opening sweep asks about positions alone.
    let mut polling = PollSequencer::new(&resolved.arm, commission.rail, PollCadence::Positions);
    let posture = drive(
        &mut bus,
        &resolved.map,
        &mut polling,
        clock,
        budget,
        &mut |step| line(&format!("  {step}")),
    )?;

    Ok(Commissioned {
        resolved,
        bus,
        posture,
        fresh: true,
    })
}

impl<'a, P: BusPort> Commissioned<'a, P> {
    /// Where the last sweep found the machine, and what its rail read.
    ///
    /// Only as current as [`Self::fresh`] says: a release that measured nothing
    /// leaves this describing the pose the head was held at rather than the one
    /// it fell into.
    #[must_use]
    pub fn posture(&self) -> &Posture {
        &self.posture
    }

    /// Whether [`Self::posture`] still describes the machine.
    ///
    /// False from the moment anything goes by without measuring every joint —
    /// a release that measured none, a sweep that did not complete — until the
    /// next sweep that does.
    #[must_use]
    pub fn fresh(&self) -> bool {
        self.fresh
    }

    /// Take another look, and keep it.
    ///
    /// The resting watch. A hand can turn the head while the machine lies limp,
    /// so the pose an engage plans from is only as good as the last sweep; the
    /// rail and the error bits change on the timescale of a power supply, which
    /// is why `cadence` exists — a caller polling ten times a second re-reads
    /// them far less often than that.
    pub fn poll(
        &mut self,
        cadence: PollCadence,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
    ) -> Result<Posture, PumpError> {
        let mut polling = PollSequencer::new(&self.resolved.arm, self.posture.rail, cadence);
        // Cleared before the sweep, not after it: a sweep that fails measured
        // nothing, and a caller left believing the posture in hand is current
        // would pin a limp head at where it was however long ago. The engage
        // precondition is what acts on this, and it is the reason a failed
        // sweep costs a later engage nine reads instead of a slam.
        self.fresh = false;
        self.posture = drive(
            &mut self.bus,
            &self.resolved.map,
            &mut polling,
            clock,
            action_budget(&self.resolved.arm),
            &mut |step| line(&format!("  {step}")),
        )?;
        self.fresh = true;
        Ok(self.posture)
    }

    /// Pin every joint where the last sweep found it and enable torque.
    ///
    /// Twenty-seven transactions and two pieces of arithmetic: the supply floor
    /// and the error bits are judged against the poll already in hand, so
    /// nothing here waits on the machine to answer a question before torque can
    /// come on. A refusal costs the bus nothing and leaves the machine exactly
    /// as it was.
    ///
    /// Everything that can fail *after* the first enable write is the one case
    /// that needs undoing: some servos may be holding torque with nothing
    /// driving them. Every such path takes the fault release on the way out, so
    /// the error a caller sees always describes a limp machine.
    ///
    /// Twenty-seven is the count from a posture the machine is still in. A
    /// posture left stale by a release that measured nothing costs a sweep of
    /// nine reads first: those angles are written to the goal registers before
    /// the enables, and pinning a limp head at where it *was* is the slam that
    /// sweep exists to prevent.
    pub fn engage(
        &mut self,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
    ) -> Result<Engaged<'_, 'a, P>, PumpError> {
        if !self.fresh {
            self.poll(PollCadence::Positions, clock, line)?;
        }
        let resolved = self.resolved;
        let mut engaging = EngageSequencer::new(
            &resolved.arm,
            &resolved.motion.geom,
            &resolved.motion.fk,
            &self.posture,
        );
        let engaged = match drive(
            &mut self.bus,
            &resolved.map,
            &mut engaging,
            clock,
            action_budget(&resolved.arm),
            &mut |step| line(&format!("  {step}")),
        ) {
            Ok(engaged) => engaged,
            Err(error) => {
                // The one path that crosses the torque line mid-run, so the
                // phase its ending is classified in is the sequencer's own
                // record of whether an enable went out. There is no engagement
                // yet and so no tick to drive a controlled stow with: what a
                // half-enabled machine gets is the release, whatever the class
                // would ask of a caller that had one.
                if engage_phase(engaging.torque_written()) == Phase::UnderTorque {
                    line("engage failed with torque possibly on; releasing immediately");
                    self.detorque_now(clock, line);
                }
                return Err(error);
            }
        };

        line(engage_report(&engaged).trim_end());
        // Nine servos are holding by now, so anything that fails between here
        // and the caller owning an `Engaged` is a machine torqued with no loop
        // driving it — the same undoing the enable walk's own failures take.
        let pump = match MotionPump::new(
            &resolved.motion,
            &resolved.map,
            resolved.tick_hz,
            resolved.health_poll_hz,
            engaged.armed.joints,
            resolved.settle,
        ) {
            Ok(mut pump) => {
                pump.record_trace(resolved.trace.is_some());
                pump
            }
            Err(error) => {
                line("the control loop would not start with torque on; releasing immediately");
                self.detorque_now(clock, line);
                return Err(error);
            }
        };
        Ok(Engaged {
            state: MotionState::new_armed(&engaged.armed),
            machine: self,
            pump,
        })
    }

    /// Take the machine limp now, on a path that already has an error to
    /// return.
    ///
    /// The release reports itself through `line` as it walks, so the failure
    /// worth exiting with is the one that made the release necessary — except
    /// for the single case the release cannot report, which is a walk that
    /// never reached its end and therefore printed nothing at all.
    fn detorque_now(&mut self, clock: &mut dyn Clock, line: &mut dyn FnMut(&str)) {
        let immediate = DisarmSequencer::immediate(&self.resolved.disarm);
        let outcome = self.detorque_with(immediate, clock, line);
        report_release(outcome, line);
    }

    /// Run `sequencer` against the bus and keep what it measured.
    ///
    /// The positions a release measured, if it measured any, replace the
    /// posture this machine was carrying: they are the freshest thing anyone
    /// knows about where the head is, and the next engage plans from them.
    ///
    /// A joint the release did not measure is a joint nobody has looked at
    /// since torque came off it, and torque coming off is exactly when a head
    /// moves. The posture is stale from here until a sweep measures all nine.
    fn detorque_with(
        &mut self,
        sequencer: DisarmSequencer,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
    ) -> Result<DisarmSummary, PumpError> {
        let outcome = detorque(self.resolved, &mut self.bus, sequencer, clock, line);
        self.fresh = false;
        if let Ok(summary) = &outcome {
            for row in 0..JointId::COUNT {
                if summary.measured(row) {
                    let joint = JointId::ALL[row];
                    let angle = summary.present.get(joint).expect("nine joints");
                    self.posture.present.set(joint, angle);
                }
            }
            self.fresh = summary.measured_all();
        }
        outcome
    }
}

/// What an orderly release's outcome leaves to be done.
///
/// The doctrine's fall-through: a refusal or fault during the settle-measure
/// form takes the immediate one. The orderly walk is a convenience, and one
/// that stopped part way through may have left servos holding with no sequence
/// driving them — a head held up by a loop nobody is running is the state this
/// whole path exists to avoid. An unacknowledged torque-off is the exception:
/// every release was written, and writing them again would say no more.
///
/// A function of its own rather than a tail inside [`Engaged::disengage`],
/// because the outcome it turns on is one the composed path cannot presently
/// produce — the release driver absorbs every wire failure and the action
/// budget is three orders of magnitude above the walk — and a rule this
/// load-bearing has to be drivable directly to be checked at all.
fn fall_through_to_immediate<P: BusPort>(
    machine: &mut Commissioned<'_, P>,
    outcome: Result<DisarmSummary, PumpError>,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<DisarmSummary, PumpError> {
    let Err(error) = outcome else {
        return outcome;
    };
    // A refusal wrote nothing, so there is nothing part way through. An
    // unacknowledged torque-off is the other exception, and a degenerate one:
    // every servo was asked and every write retried, so writing them again
    // would say no more than the report already does.
    if error.class(Phase::UnderTorque) == ErrorClass::Refuse
        || matches!(error, PumpError::TorqueOffUnacked { .. })
    {
        return Err(error);
    }
    line("the orderly release did not finish; releasing immediately");
    machine.detorque_now(clock, line);
    Err(error)
}

/// Say so when a release did not finish, and nothing when it did.
///
/// Every ending that runs a release has an error of its own to return — the
/// fault, or the engage failure — and that is the one worth exiting with. What
/// the release has to say it says through `line` servo by servo, including any
/// servo that never acknowledged its own torque-off. The exception is a walk
/// that ended early: it prints nothing, and dropping its error would leave the
/// operator with no record that the machine may still be holding.
fn report_release(outcome: Result<DisarmSummary, PumpError>, line: &mut dyn FnMut(&str)) {
    match outcome {
        Ok(_) | Err(PumpError::TorqueOffUnacked { .. }) => {}
        Err(error) => line(&format!("  the release itself did not finish: {error}")),
    }
}

/// A machine holding torque, and the loop that moves it.
///
/// Constructed only by [`Commissioned::engage`], so holding one is the evidence
/// that nine servos took torque at the pose they reported. It borrows the
/// commissioned machine rather than taking it, so the bus and the port never
/// change hands across an engage/release cycle — a process can raise the head
/// and let it go all day without reopening anything.
pub struct Engaged<'m, 'a, P: BusPort> {
    machine: &'m mut Commissioned<'a, P>,
    state: MotionState,
    pump: MotionPump<'a>,
}

impl<P: BusPort> Engaged<'_, '_, P> {
    /// The configuration the machine was last commanded to, which is where the
    /// next move starts.
    #[must_use]
    pub fn targets(&self) -> JointTargets {
        *self.state.last_targets()
    }

    /// Whether every joint that carries the head has been taken out of service.
    ///
    /// The end of the mask's growth: with the cranks and the body yaw all
    /// released there is nothing left to stow the head with, and what a
    /// wind-down has to do is what it was always going to end with.
    fn head_released(&self) -> bool {
        let masked = self.state.masked();
        masked.covers(JointGroup::Legs) && masked.covers(JointGroup::BodyYaw)
    }

    /// Carry one move to its endpoint, each mechanical group on its own clock.
    pub fn move_to(
        &mut self,
        target: JointTargets,
        durations: MoveDurations,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
    ) -> Result<MoveSummary, PumpError> {
        self.move_retargeting(target, durations, clock, line, &mut || None)
    }

    /// Carry one move, asking `retarget` at every control period whether it is
    /// still the move the caller wants.
    ///
    /// Answering `Some((target, durations))` replaces the move in flight: the
    /// new path is shaped from the setpoint the last period commanded, so the
    /// head turns around where it is rather than finishing the old move first.
    /// The call returns when whichever move was last accepted reaches its
    /// endpoint.
    ///
    /// This is what a caller executing a timeline it does not control needs:
    /// the instruction can change while the head is halfway up, and waiting out
    /// the raise before starting the fold is the head being late by a whole
    /// move. [`Engaged::move_to`] is this with nothing to say.
    pub fn move_retargeting(
        &mut self,
        target: JointTargets,
        durations: MoveDurations,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
        retarget: &mut dyn FnMut() -> Option<(JointTargets, MoveDurations)>,
    ) -> Result<MoveSummary, PumpError> {
        self.move_retargeting_events(target, durations, clock, line, &mut |_| {}, retarget)
    }

    /// The same move, with the run's tick events handed to `event` as values
    /// as well as printed.
    ///
    /// Unlike [`Engaged::hold_events`], the bench's printed report stays —
    /// `event` observes beside it. A rendered line is not a fact downstream
    /// can key on; this gives a caller its own machine-readable record.
    pub fn move_retargeting_events(
        &mut self,
        target: JointTargets,
        durations: MoveDurations,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
        event: &mut dyn FnMut(TickEvent),
        retarget: &mut dyn FnMut() -> Option<(JointTargets, MoveDurations)>,
    ) -> Result<MoveSummary, PumpError> {
        let command = MotionCommand::MoveTo {
            target,
            durations,
            warp: WARP,
        };
        let outcome = self.pump.run_retargeting(
            &mut self.machine.bus,
            &mut self.state,
            command,
            clock,
            &mut |reported| {
                line(&format!("  {reported}"));
                event(reported);
            },
            &mut || {
                retarget().map(|(target, durations)| MotionCommand::MoveTo {
                    target,
                    durations,
                    warp: WARP,
                })
            },
        );
        self.report(line);
        outcome
    }

    /// What the run measured, printed whether or not it got where it was going.
    ///
    /// Printed on the count of periods rather than on the kind of ending: a
    /// fault, a stall and a lost wire all reach the end of the run having
    /// measured what the loop cost and what the servos were doing, and those
    /// are the runs whose numbers are worth the most. A command refused on the
    /// period that accepted it measured none of that, and its zeros would read
    /// as a clean run of nothing, so nothing is printed for it.
    fn report(&self, line: &mut dyn FnMut(&str)) {
        let summary = self.pump.last_summary();
        if summary.ticks == 0 {
            return;
        }
        line(&move_line(&summary));
        if let Some(settle) = settle_line(&summary) {
            line(&settle);
        }
        line(&lag_line(&summary));
        self.write_trace(line);
    }

    /// Append this run's per-period trace to the file the session was given, if
    /// it was given one.
    ///
    /// A trace that cannot be written is said and nothing more: it is
    /// diagnostic output, and a move that ran is not undone by a file that
    /// would not open. Which run in the file this move became is said too: the
    /// number comes from the file rather than from this session, so an operator
    /// reading rows back knows which ones this command wrote. A file that was
    /// already ending mid-row is said as well — the rows in it are not all what
    /// they claim to be, and that is worth hearing before the analysis and not
    /// after.
    fn write_trace(&self, line: &mut dyn FnMut(&str)) {
        let Some(path) = &self.machine.resolved.trace else {
            return;
        };
        let samples = self.pump.last_trace();
        match trace::append_csv(path, samples) {
            Ok(appended) => {
                line(&format!(
                    "  {} period(s) of trace appended to {} as run {}",
                    samples.len(),
                    path.display(),
                    appended.run
                ));
                if appended.mended {
                    line(&format!(
                        "  {} ended mid-row before this — an earlier trace was cut short, and \
                         that row is damaged",
                        path.display()
                    ));
                }
            }
            Err(error) => line(&format!(
                "  the trace could not be written to {}: {error}",
                path.display()
            )),
        }
    }

    /// Watch the machine hold for `duration`, commanding nothing.
    ///
    /// The loop paces and measures this exactly as it does a move, so what a
    /// hold reports about the host's timekeeping is the truth about the loop
    /// that ran it — which is the whole point of the command.
    pub fn hold(
        &mut self,
        duration: Duration,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
    ) -> Result<MoveSummary, PumpError> {
        // The disposition of a hold is that it is holding, which is the one
        // thing this command already knows. Everything else — a lost read, a
        // health latch, a fault — is news.
        let outcome = self.hold_events(duration, clock, &mut |event| {
            if !matches!(event, TickEvent::Command(_)) {
                line(&format!("  {event}"));
            }
        });
        self.report(line);
        outcome
    }

    /// Watch the machine hold for `duration`, handing the caller the raw tick
    /// events instead of rendered lines.
    ///
    /// No filtering and no timing report: what to say about a hold — which
    /// events are worth a line, in what words, and whether the periods are
    /// worth printing at all — is the caller's policy, and a program holding on
    /// a cadence for as long as it runs has a different one from an operator
    /// reading a single bench run. The numbers are not lost either way; they
    /// come back in the summary, typed.
    ///
    /// [`Engaged::hold`] is this with the bench's own policy applied.
    pub fn hold_events(
        &mut self,
        duration: Duration,
        clock: &mut dyn Clock,
        event: &mut dyn FnMut(TickEvent),
    ) -> Result<MoveSummary, PumpError> {
        self.pump.hold(
            &mut self.machine.bus,
            &mut self.state,
            duration,
            clock,
            event,
        )
    }

    /// Let the machine settle, write down where it came to rest, and release
    /// torque.
    ///
    /// The orderly ending, for a move that finished where it was told to: the
    /// dwell lets a joint still closing its lag arrive, the sweep measures the
    /// nine, and then torque comes off. The measurement is a report and never a
    /// condition — a machine found away from stow is released and said so.
    ///
    /// Consumes the engagement, because an engagement is the evidence that nine
    /// servos are holding torque and this is what makes that false: a released
    /// one left callable would take a `move_to` and pump goal frames at a limp
    /// machine, diverging further every period until the tracking budget ran
    /// out.
    /// An orderly release that does not finish falls through to the immediate
    /// one: the settle-and-measure form is a convenience, and a machine part
    /// way through it is a machine that may still be holding torque with a
    /// sequence nobody is driving.
    pub fn disengage(
        self,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
    ) -> Result<DisarmSummary, PumpError> {
        let machine = self.machine;
        let orderly = DisarmSequencer::new(&machine.resolved.disarm);
        let outcome = machine.detorque_with(orderly, clock, line);
        fall_through_to_immediate(machine, outcome, clock, line)
    }

    /// Write torque off to all nine servos now, and do nothing else.
    ///
    /// The fault ending. No settle and no measurement: whatever is wrong,
    /// getting the motors unpowered is the whole of the answer, and the head
    /// falls gently into near-stow under gearbox resistance from wherever it
    /// is. Every servo is asked whatever the ones before it answered.
    pub fn disengage_now(
        self,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
    ) -> Result<DisarmSummary, PumpError> {
        let machine = self.machine;
        let immediate = DisarmSequencer::immediate(&machine.resolved.disarm);
        machine.detorque_with(immediate, clock, line)
    }
}

/// Drive a disarm sequence against whatever is on `bus`, and report it.
///
/// Takes a sequencer rather than building one, because the two ways to release
/// differ only in which one this runs: the orderly settle-measure-release, or
/// the nine writes a fault takes. Nothing after the last release — the platform
/// settles limp into its true rest, which is the state the next engage is
/// entitled to measure.
///
/// Nothing here refuses, and nothing here stops it either: a transaction that
/// fails in a way the sequencer has no vocabulary for is printed and the walk
/// carries on to the next servo, so all nine are always asked whatever the wire
/// does. The one error this can return is raised after all nine releases have
/// been written, and says a servo never acknowledged its own — the report of an
/// incomplete release rather than a release that did not happen.
fn detorque<P: BusPort>(
    resolved: &Resolved,
    bus: &mut Bus<P>,
    mut sequencer: DisarmSequencer,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<DisarmSummary, PumpError> {
    // Reported after the walk rather than during it: nine writes take
    // milliseconds, and getting them out is worth more than narrating them.
    let mut trouble: Vec<PumpError> = Vec::new();
    let summary = drive_release(
        bus,
        &resolved.map,
        &mut sequencer,
        clock,
        DISARM_ACTIONS,
        &mut |step| line(&format!("  {step}")),
        &mut trouble,
    )?;
    for error in &trouble {
        line(&format!("  wire trouble mid-release, carried on: {error}"));
    }
    line(&disarm_line(&summary));
    // After the report, not before it: the operator reads what the machine did
    // and then the command's verdict on it.
    if let Some(joint) = summary.unreleased().next() {
        let row = joint.index().expect("a named joint has a bus row");
        return Err(PumpError::TorqueOffUnacked {
            id: resolved.disarm.ids[row],
        });
    }
    Ok(summary)
}

/// Release torque on the machine as found, without engaging it first.
///
/// What `off` runs: the bus is bare, nothing is commissioned, and nothing is
/// commanded. Releasing must never require taking hold first — that would
/// enable torque in order to switch it off.
pub fn release<P: BusPort>(
    resolved: &Resolved,
    bus: &mut Bus<P>,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<DisarmSummary, PumpError> {
    detorque(
        resolved,
        bus,
        DisarmSequencer::new(&resolved.disarm),
        clock,
        line,
    )
}

/// What a move cost, as a run prints it.
fn move_line(summary: &MoveSummary) -> String {
    format!(
        "  {ticks} period(s), {goals} commanding, {frames} frame(s), {misses} blind, \
         {overruns} overrun(s), worst jitter {jitter:.1} ms, {slip:.1} ms slip, {elapsed:.2} s",
        ticks = summary.ticks,
        goals = summary.goals,
        frames = summary.frames,
        misses = summary.misses,
        overruns = summary.overruns,
        jitter = summary.worst_jitter.as_secs_f64() * 1e3,
        slip = summary.slip.as_secs_f64() * 1e3,
        elapsed = summary.elapsed.as_secs_f64(),
    )
}

/// The two instants a move ends on: when the last goal went out, and when the
/// machine was measured to have got there.
///
/// The gap between them is the settle, and it is the part of a move the elapsed
/// time alone hides — a move commanded over a hundredth of a second reports its
/// commanding over in a twentieth, with the head still on its way up. Nothing is
/// printed for a run that never finished commanding: a hold has no such instant,
/// and a faulted move's is a moment it did not reach.
fn settle_line(summary: &MoveSummary) -> Option<String> {
    let commanded = summary.commanded?;
    let head = format!("  commanding finished {:.2} s in", commanded.as_secs_f64());
    Some(match (summary.settled, summary.unsettled) {
        (Some(settled), _) => format!(
            "{head}; measurably at the goal {:.2} s later, at {:.2} s",
            settled.saturating_sub(commanded).as_secs_f64(),
            settled.as_secs_f64(),
        ),
        (None, Some((joint, error))) => format!(
            "{head}; {joint} was still {:.2}° from its goal when the settle window ran out",
            error.to_degrees(),
        ),
        // The run ended during the settle for a reason of its own — a fault, a
        // lost wire, a caller that steered elsewhere. Where the machine got to
        // is a question this run did not finish asking.
        (None, None) => format!("{head}; the run ended before the machine was measured there"),
    })
}

/// How far each joint ran behind its goal at worst, in bus order.
///
/// The measurement the tracking threshold, its window and the stow tolerance
/// have all been provisional against. Degrees, like arming's droop and pull-in
/// lines, so the nine numbers a run brings back are read on one scale.
fn lag_line(summary: &MoveSummary) -> String {
    let lags: Vec<String> = summary
        .worst_lag
        .iter()
        .map(|rad| format!("{:.3}", rad.to_degrees()))
        .collect();
    format!("  worst lag [{}] deg", lags.join(", "))
}

/// Where the machine was when torque came off, and which servos said so.
///
/// The fault form gets a line of its own: it did not fail to measure the
/// machine, it deliberately did not look, and that is a different report from
/// the same nine joints going unread. Which one happened is read off the
/// summary's form rather than inferred from empty measurements — a dead adapter
/// during an orderly `off` produces the same nine blanks a fault release does,
/// and printing it as a fault release would hide the bus having stopped
/// answering.
fn disarm_line(summary: &DisarmSummary) -> String {
    if !summary.looked() {
        return format!(
            "  torque off, immediately; nothing was measured{}",
            release_tail(summary)
        );
    }
    let unreadable: Vec<String> = summary
        .unreadable()
        .map(|(joint, cause)| format!("{joint} ({cause})"))
        .collect();
    let mut out = if unreadable.len() == JointId::COUNT {
        "  torque off; not one of the nine joints could be read".to_string()
    } else {
        let (joint, deviation) = summary.worst_deviation();
        format!(
            "  torque off; {at}, furthest measured joint {joint} at {:.3} deg from stow",
            deviation.to_degrees(),
            at = if summary.at_stow {
                "at stow"
            } else {
                "not at stow"
            },
        )
    };
    if !unreadable.is_empty() {
        out.push_str(&format!("\n  unmeasured: {}", unreadable.join(", ")));
    }
    out.push_str(&release_tail(summary));
    out
}

/// Which servos acknowledged their torque-off write — the part of a release
/// that decides whether a hand can go on the head.
fn release_tail(summary: &DisarmSummary) -> String {
    let unreleased: Vec<String> = summary.unreleased().map(|j| j.to_string()).collect();
    if unreleased.is_empty() {
        "\n  every servo acknowledged torque off".to_string()
    } else {
        format!(
            "\n  NO ACKNOWLEDGEMENT of torque off: {} — may still be holding",
            unreleased.join(", ")
        )
    }
}

/// How a bench command ends once it has finished with an engaged machine.
///
/// A clean run leaves the head where it is, holding: the bench's model is that
/// a command takes hold and `off` lets go, so an operator can chain `up`,
/// `yaw`, `stow` without the head dropping in between. So does every ending
/// that names no condition of the machine — a refusal changed nothing, and a
/// defect of ours leaves a healthy machine an operator is standing next to.
///
/// A fault is what ends differently, and by which maneuver its class asks for:
/// a wind-down under control where the motors still command, torque off on the
/// spot where they no longer can be trusted to.
fn settle<T, P: BusPort>(
    engaged: Engaged<'_, '_, P>,
    outcome: Result<T, PumpError>,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let Err(error) = outcome else {
        return Ok(());
    };
    let class = error.class(Phase::UnderTorque);
    let faulted = error.fault(Phase::UnderTorque).is_some();
    match class {
        ErrorClass::Refuse => {}
        // The abort-class endings: the plan was wrong and the platform is
        // fine. Stowing them would be a machine winding itself down over a
        // bug in front of somebody who can read the message and type `off`.
        ErrorClass::SlowStowToRest if !faulted => {
            line(&format!("the run did not finish: {error}"));
            line("the machine is healthy and still holding; `off` releases it");
        }
        ErrorClass::SlowStowToRest | ErrorClass::MaskedSlowStowToPark => {
            line(&format!("fault: {error}"));
            report_disposition(wind_down(engaged, class.disposition(), clock, line), line);
        }
        ErrorClass::ImmediateAllTorqueOffToRest | ErrorClass::ImmediateAllTorqueOffToPark => {
            line(&format!("fault: {error}"));
            line("releasing torque now; the head settles into near-stow");
            report_release(engaged.disengage_now(clock, line), line);
            report_disposition(class.disposition(), line);
        }
    }
    Err(error)
}

/// Stow under control on the tick state the fault left commanding, then release
/// everything.
///
/// The maneuver both controlled responses run. The raise that brought us here
/// dropped the move and left the tick holding at its last goal — and, for a
/// servo that dropped out, released that servo and took it out of every check —
/// so the stow is commanded on the same live state rather than on a fresh
/// engage of a machine nobody has looked at.
///
/// A head servo dropping out mid-stow expands the maneuver instead of ending
/// it: that servo is already released by the time the ending arrives here, and
/// the stow is re-commanded from where the machine now stands, on what is left
/// of the clock this started with. The clock never restarts, so however many
/// servos go, the whole maneuver is bounded by the one stow it began as — and
/// a wind-down the machine defeats, or one whose clock runs out, falls through
/// to the immediate release. Every path through here ends with torque off.
///
/// The disposition is the sticky maximum: a head servo dropping out latches,
/// so a wind-down that started for a grabbed head and lost a servo on the way
/// down ends parked rather than at rest.
fn wind_down<P: BusPort>(
    mut engaged: Engaged<'_, '_, P>,
    disposition: Disposition,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Disposition {
    let stow = engaged.machine.resolved.stow_durations();
    // The stow's own clock plus the window its arrival is measured in: the
    // whole of what one stow is allowed, spent once across every expansion.
    let deadline = clock
        .now()
        .saturating_add(stow.longest())
        .saturating_add(engaged.machine.resolved.settle.timeout);
    let mut disposition = disposition;
    let stowed = loop {
        if engaged.head_released() {
            // Not a stow: there is nothing left to drive one with, and saying
            // the head came down under control when every joint that carries
            // it has gone limp is the one claim this record must not make.
            line("  no head joint is still commanded; releasing torque now");
            break false;
        }
        let left = deadline.saturating_sub(clock.now());
        if left.is_zero() {
            line("  the stow clock is spent; releasing torque now");
            break false;
        }
        line("  stowing under control on what still commands");
        match engaged.move_to(stow_pose_targets(), within(stow, left), clock, line) {
            Ok(_) => break true,
            Err(error) => match error.fault(Phase::UnderTorque) {
                // The mask grew. Nothing about the maneuver changes but which
                // servos carry the head the rest of the way down.
                Some(fault @ Fault::HeadServoFault { .. }) => {
                    line(&format!("  {fault}; the stow carries on without it"));
                    disposition = Disposition::Park;
                }
                _ => {
                    line(&format!("  the stow did not finish: {error}"));
                    break false;
                }
            },
        }
    };
    if stowed {
        line("  stowed; releasing torque");
    }
    report_release(engaged.disengage_now(clock, line), line);
    disposition
}

/// The same move clocks, with nothing over `left` on any of them.
///
/// What a re-commanded stow is asked for: the maneuver's clock is the one it
/// started with, so an expansion gets the remainder of it rather than a fresh
/// one. A remainder shorter than the move can be run in is floored by the
/// guard, as any other under-clocked move is, and the deadline catches the
/// overrun on the next pass.
fn within(durations: MoveDurations, left: Duration) -> MoveDurations {
    MoveDurations {
        head: durations.head.min(left),
        antennas: durations.antennas.map(|antenna| antenna.min(left)),
    }
}

/// What the machine is left waiting for.
///
/// The disposition and nothing else: a park is asked for by five endings,
/// and a line that guessed which one would send an operator whose bus died,
/// or whose torque-off went unacknowledged, to look at the servos.
fn report_disposition(disposition: Disposition, line: &mut dyn FnMut(&str)) {
    line(match disposition {
        Disposition::Rest => "  at rest; the next command engages the machine again",
        Disposition::Park => "  parked: look at the machine before arming it again",
    });
}

/// Verify the machine, pin every joint where it stands, and enable torque.
pub fn arm<P: BusPort>(
    resolved: &Resolved,
    port: P,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let mut machine = commission(resolved, port, clock, line)?;
    machine.engage(clock, line)?;
    line("armed; torque is on and the machine is holding.");
    Ok(())
}

/// Lift the head from wherever it is to the neutral configuration.
pub fn up<P: BusPort>(
    resolved: &Resolved,
    port: P,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let mut machine = commission(resolved, port, clock, line)?;
    let mut engaged = machine.engage(clock, line)?;
    line("up: to the neutral configuration");
    let outcome = engaged.move_to(neutral_targets(), resolved.up_durations(), clock, line);
    settle(engaged, outcome, clock, line)
}

/// Hold where the machine already is, and watch it do so.
pub fn hold<P: BusPort>(
    resolved: &Resolved,
    port: P,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let mut machine = commission(resolved, port, clock, line)?;
    let mut engaged = machine.engage(clock, line)?;
    line("hold: commanding nothing, measuring every period");
    let outcome = engaged.hold(resolved.hold_duration, clock, line);
    settle(engaged, outcome, clock, line)
}

/// Move the head to the stow configuration, leaving torque on.
pub fn stow<P: BusPort>(
    resolved: &Resolved,
    port: P,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let mut machine = commission(resolved, port, clock, line)?;
    let mut engaged = machine.engage(clock, line)?;
    line("stow: to the stow configuration; torque stays on until `off`");
    let outcome = engaged.move_to(stow_pose_targets(), resolved.stow_durations(), clock, line);
    settle(engaged, outcome, clock, line)
}

/// Release torque, wherever the machine is.
pub fn off<P: BusPort>(
    resolved: &Resolved,
    port: P,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let mut bus = Bus::new(port, resolved.timing);
    line("off: settling, measuring against stow, then releasing torque");
    release(resolved, &mut bus, clock, line)?;
    Ok(())
}

/// Write the antennas' operating mode, and nothing else.
///
/// The one register this project provisions itself. It is non-volatile, so it
/// survives a power cycle and is written once per unit rather than per session;
/// the self-test's provisioning sweep checks it on every run thereafter and
/// refuses a machine that does not hold it.
///
/// No arming and no motion: the bus is bare, as `off`'s is, and torque must
/// already be off — the guarded write path reads Torque Enable itself and
/// refuses otherwise. Presence, identity and torque are established on both
/// servos before either is written, so a refusal on the second one leaves the
/// first unwritten too.
pub fn provision<P: BusPort>(
    map: &ServoMap,
    timing: BusTiming,
    port: P,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let mut bus = Bus::new(port, timing);
    let mut found = Vec::new();

    for joint in PROVISIONED_JOINTS {
        let row = joint.index().expect("a named joint has a bus row");
        let id = map.ids()[row];
        let info = with_retry(&mut bus, |bus| bus.ping(id))
            .map_err(|source| PumpError::Bus { id, source })?;
        let expected = EXPECTED_MODELS[row];
        if info.model != expected {
            return Err(PumpError::WrongPart {
                id,
                model: info.model,
                expected,
            });
        }
        if read_byte(&mut bus, map, row, RegId::TorqueEnable)? != 0 {
            return Err(PumpError::TorqueHeld { id });
        }
        let mode = read_byte(&mut bus, map, row, RegId::OperatingMode)?;
        line(&format!(
            "  {joint}: servo {id}, model {model}, torque off, operating mode {mode}",
            model = info.model
        ));
        found.push((row, id, mode));
    }

    for (row, id, mode) in found {
        let wanted = EXPECTED_OPERATING_MODES[row];
        if mode == wanted {
            line(&format!(
                "  servo {id} already holds operating mode {wanted}; nothing written"
            ));
            continue;
        }
        let raw = map
            .encode_value(row, RegId::OperatingMode, RegValue::U8(wanted))
            .map_err(|source| PumpError::Map {
                id,
                reg: RegId::OperatingMode,
                source,
            })?;
        let entry = reg_for(RegId::OperatingMode);
        with_retry(&mut bus, |bus| bus.write_eeprom_verified(id, entry, &raw))
            .map_err(|source| PumpError::Bus { id, source })?;
        line(&format!(
            "  servo {id} operating mode {mode} -> {wanted}, read back and verified"
        ));
    }

    line("provisioned; run `reachy-bench selftest` before arming.");
    Ok(())
}

/// Restart the servos, and report what they come back holding.
///
/// The way to clear a latched hardware error — an overload above all — without
/// cutting the machine's power. A restart clears Torque Enable, so every servo
/// this reaches lets go of whatever it was holding.
///
/// Nothing here arms, enables torque or asks the machine's permission: a reboot
/// is a de-torque, and nothing gates a de-torque. The one refusal is a servo ID
/// the configured roster does not carry, which is a command line to correct
/// rather than a machine to judge.
///
/// The instruction is sent to every target first and the whole set polled
/// afterwards, so nine servos restart alongside each other rather than one at a
/// time. What each one is holding when it answers again — the error byte the
/// reboot was sent to clear, the torque a restart drops, and the position it
/// came back at — is read once they are all back.
///
/// Answering is not restarting: a servo that never took the instruction answers
/// exactly as one that took it and came back, and the only difference on the
/// wire is the torque it is still holding. So the torque is read rather than
/// assumed, and a servo still holding it fails the command — otherwise a
/// corrupted frame ends in a success and a report of a restart that did not
/// happen.
///
/// Torque cannot tell them apart on a machine that was already limp, and that
/// is the machine this command is usually run on: a latched shutdown de-torques
/// the servo it latches on, and every fault response de-torques the lot. So the
/// acknowledgement is kept too. A servo that answered its reboot and came back
/// limp restarted; one that answered nothing and came back limp is
/// indeterminate, and an indeterminate restart fails the command rather than
/// riding out on the closing line — an operator scripting a recovery around the
/// exit code is owed the difference.
pub fn reboot<P: BusPort>(
    map: &ServoMap,
    timing: BusTiming,
    port: P,
    target: Option<u8>,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let targets = reboot_targets(map, target)?;
    let mut bus = Bus::new(port, timing);

    line(
        "reboot: every servo addressed restarts, which clears its Torque Enable — whatever it \
         was holding, it lets go of. The head settles as it goes, so take its weight if it is \
         up. Where the machine is standing gates nothing here.",
    );

    // Which servos never acknowledged the instruction, by row. Kept because it
    // is half of the restart verdict below: a servo that acknowledged took the
    // reboot, and one that did not has only its torque left to say so.
    let mut unacked: Vec<(u8, XactError)> = Vec::new();
    for (_, id) in &targets {
        match bus.reboot(*id) {
            Ok(()) => line(&format!("  servo {id}: reboot sent")),
            // The frame is on the wire either way, and a servo that took it has
            // no answer left to give. Whether it restarted is what the poll
            // below establishes, so nothing stops here.
            Err(source) => {
                line(&format!(
                    "  servo {id}: reboot sent, unacknowledged ({source})"
                ));
                unacked.push((*id, source));
            }
        }
    }

    let mut trouble: Option<PumpError> = None;
    let mut back = Vec::with_capacity(targets.len());
    for (row, id) in &targets {
        match wait_for(&mut bus, *id, clock) {
            Ok(waited) => {
                line(&format!(
                    "  servo {id}: answering {:.2} s after the instruction went out",
                    waited.as_secs_f64()
                ));
                back.push((*row, *id));
            }
            Err(error) => {
                line(&format!(
                    "  servo {id}: NO ANSWER since its reboot ({error})"
                ));
                trouble.get_or_insert(error);
            }
        }
    }

    for (row, id) in back {
        match reading(&mut bus, map, row, id) {
            Ok(read) => {
                line(&read.report);
                if read.holding {
                    trouble.get_or_insert(PumpError::NotRestarted { id });
                } else if let Some(at) = unacked.iter().position(|(deaf, _)| *deaf == id) {
                    let (_, source) = unacked.remove(at);
                    line(&format!(
                        "  servo {id}: it took no acknowledgement and had no torque to drop, so \
                         nothing here says it restarted"
                    ));
                    trouble.get_or_insert(PumpError::RestartUnconfirmed { id, source });
                }
            }
            Err(error) => {
                line(&format!(
                    "  servo {id}: answered, but reads back as {error}"
                ));
                trouble.get_or_insert(error);
            }
        }
    }

    match trouble {
        Some(error) => Err(error),
        None => {
            line("rebooted; every servo that came back is limp. `selftest` reads the machine.");
            Ok(())
        }
    }
}

/// The servos a reboot addresses: the one asked for, or the whole roster in bus
/// order.
///
/// Each carries its row, because the row is what turns a reading into the
/// register widths and the joint it belongs to.
fn reboot_targets(map: &ServoMap, target: Option<u8>) -> Result<Vec<(usize, u8)>, PumpError> {
    let roster = map.ids();
    let all = || roster.iter().copied().enumerate().collect();
    let Some(id) = target else {
        return Ok(all());
    };
    match roster.iter().position(|held| *held == id) {
        Some(row) => Ok(vec![(row, id)]),
        None => Err(PumpError::OffRoster { id, roster }),
    }
}

/// Ping `id` until it answers again, and say how long that took.
///
/// A servo that is restarting answers nothing at all, so every poll before it
/// is back fails on its own deadline; the pause between them is the bus's own
/// retry spacing, which is the cadence this configuration carries for exactly
/// this — asking again in a moment.
fn wait_for<P: BusPort>(
    bus: &mut Bus<P>,
    id: u8,
    clock: &mut dyn Clock,
) -> Result<Duration, PumpError> {
    let started = clock.now();
    let spacing = bus.timing().retry_spacing;
    let mut asked = 0;
    loop {
        let failed = match bus.ping(id) {
            Ok(_) => return Ok(clock.now().saturating_sub(started)),
            Err(error) => error,
        };
        asked += 1;
        if asked >= BOOT_POLLS {
            return Err(PumpError::NotBack {
                id,
                polls: asked,
                waited: clock.now().saturating_sub(started),
                source: failed,
            });
        }
        clock.sleep_until(clock.now() + spacing);
    }
}

/// What one servo came back holding.
struct Reading {
    /// The line an operator reads.
    report: String,
    /// Whether it is still holding torque, which a restart clears — so a servo
    /// that answers holding it did not restart.
    holding: bool,
}

/// What a servo holds now: the hardware-error byte, its torque, and where it is
/// standing.
///
/// The error byte is the operator's reason for rebooting at all, and this is
/// the reading that says whether it went. Torque is the restart's own
/// observable: a servo comes back with it cleared, so reading it is how
/// answering is told from restarting. The position is reported as the servo's
/// own count and the angle that count is, unshifted by anything the host knows.
/// Neither the byte nor the position is compared against an expectation: what a
/// restart does to a latched byte or to a position is not something this
/// project has established on its own hardware, so both are reported as they
/// read.
fn reading<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    id: u8,
) -> Result<Reading, PumpError> {
    let bits = read_byte(bus, map, row, RegId::HardwareErrorStatus)?;
    let torque = read_byte(bus, map, row, RegId::TorqueEnable)?;
    let counts = read_counts(bus, id)?;
    let latched = if bits == 0 {
        "clear".to_string()
    } else if HardwareError(bits).healthy_or_voltage_only() {
        "input voltage only".to_string()
    } else {
        format!("still latched: {bits:#04x}")
    };
    let holding = torque != 0;
    let torque = if holding {
        "STILL HOLDING TORQUE, so it did not restart"
    } else {
        "limp"
    };
    Ok(Reading {
        report: format!(
            "  servo {id}: hardware error {bits:#04x} ({latched}), {torque}, at {counts} counts, \
             {:.3} deg unshifted",
            counts_to_rad(counts).to_degrees()
        ),
        holding,
    })
}

/// One servo's present position, as the count it reports.
///
/// Unshifted, which is why this reads the register rather than going through
/// the map's decoding: what a restart left in the servo is the count, and the
/// host's own idea of where zero is would be an interpretation laid over it.
fn read_counts<P: BusPort>(bus: &mut Bus<P>, id: u8) -> Result<i32, PumpError> {
    let entry = reg_for(RegId::PresentPosition);
    let raw = with_retry(bus, |bus| bus.read_reg(id, entry))
        .map_err(|source| PumpError::Bus { id, source })?;
    // A successful read is exactly the register's declared width, and this
    // register is four bytes wide, so this only fails if the two have drifted
    // apart.
    Ok(raw.i32().expect("a position register is four bytes wide"))
}

/// One servo's one-byte register, with retry.
///
/// The width the answer must have is the map's to know, not this function's.
fn read_byte<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    reg: RegId,
) -> Result<u8, PumpError> {
    let id = map.ids()[row];
    let entry = reg_for(reg);
    let raw = with_retry(bus, |bus| bus.read_reg(id, entry))
        .map_err(|source| PumpError::Bus { id, source })?;
    let value = map
        .decode_value(row, reg, &raw)
        .map_err(|source| PumpError::Map { id, reg, source })?;
    match value {
        RegValue::U8(byte) => Ok(byte),
        _ => Err(PumpError::Map {
            id,
            reg,
            source: MapError::WrongShape {
                reg,
                expected: ValueKind::U8,
                observed: value_kind(reg),
            },
        }),
    }
}

/// Rotate the body to `degrees`, leaving the head where it is relative to it.
pub fn yaw<P: BusPort>(
    resolved: &Resolved,
    port: P,
    degrees: f64,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let mut machine = commission(resolved, port, clock, line)?;
    let mut engaged = machine.engage(clock, line)?;
    line(&format!("yaw: to {degrees:.1} deg"));
    let target = JointTargets {
        body_yaw: degrees.to_radians(),
        ..engaged.targets()
    };
    let outcome = engaged.move_to(target, resolved.move_durations(), clock, line);
    settle(engaged, outcome, clock, line)
}

/// Move the antennas to `right` and `left`, radians.
pub fn antennas<P: BusPort>(
    resolved: &Resolved,
    port: P,
    right: f64,
    left: f64,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let mut machine = commission(resolved, port, clock, line)?;
    let mut engaged = machine.engage(clock, line)?;
    line(&format!("antennas: to [{right:.3}, {left:.3}] rad"));
    let target = JointTargets {
        antennas: [right, left],
        ..engaged.targets()
    };
    let outcome = engaged.move_to(target, resolved.move_durations(), clock, line);
    settle(engaged, outcome, clock, line)
}

/// The milestone sequence, end to end: up, a dwell, the antennas, the body,
/// stow, and torque off.
///
/// Armed once and chained: each move starts from where the last one ended, so
/// the whole run is one continuous trajectory through the same tick path every
/// individual command uses. It ends released, at stow, which is the only way a
/// bench session ends without the head falling.
pub fn demo<P: BusPort>(
    resolved: &Resolved,
    port: P,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let mut machine = commission(resolved, port, clock, line)?;
    let mut engaged = machine.engage(clock, line)?;

    if let Err(error) = demo_moves(&mut engaged, resolved, clock, line) {
        return settle(engaged, Err::<(), _>(error), clock, line);
    }

    line("demo 6/6: off");
    engaged.disengage(clock, line)?;
    Ok(())
}

/// The demo's five moving steps, from the lift to the stow.
///
/// Split out so the whole chain has one ending: a fault at any step takes the
/// machine limp, and only a chain that ran clean reaches the orderly release.
fn demo_moves<P: BusPort>(
    engaged: &mut Engaged<'_, '_, P>,
    resolved: &Resolved,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    line("demo 1/6: up");
    engaged.move_to(neutral_targets(), resolved.up_durations(), clock, line)?;

    line("demo 2/6: holding");
    engaged.hold(resolved.hold_duration, clock, line)?;

    line("demo 3/6: antennas");
    for antennas in [
        [DEMO_ANTENNA_RAD, -DEMO_ANTENNA_RAD],
        [-DEMO_ANTENNA_RAD, DEMO_ANTENNA_RAD],
        [0.0, 0.0],
    ] {
        let target = JointTargets {
            antennas,
            ..engaged.targets()
        };
        engaged.move_to(target, resolved.move_durations(), clock, line)?;
    }

    line("demo 4/6: body yaw");
    for degrees in [DEMO_YAW_DEG, -DEMO_YAW_DEG, 0.0] {
        let target = JointTargets {
            body_yaw: degrees.to_radians(),
            ..engaged.targets()
        };
        engaged.move_to(target, resolved.move_durations(), clock, line)?;
    }

    line("demo 5/6: stow");
    engaged.move_to(stow_pose_targets(), resolved.stow_durations(), clock, line)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::rc::Rc;

    use dxl_proto::frame::{INST_PING, INST_REBOOT, INST_WRITE};
    use reachy_kin::{HeadGeometry, LegAngles, inverse_kinematics, wrap_to_pi};
    use reachy_motion::{
        ANTENNA_GOAL_MAX_RAD, ANTENNA_GOAL_MIN_RAD, CommandDisposition, JointGroup, JointVector,
        Rail, RegId, RegValue, ReleaseForm, SeqError, SeqStep, ServoHealth, StepContext,
        engage_gates,
    };

    use super::*;
    use crate::testutil::{
        FakeMachine, Flaky, GroupedWrite, Spy, TestClock, datumed_config, machine_at, resolved,
        stow_legs,
    };

    /// The six crank angles the neutral pose holds.
    fn neutral_legs() -> [f64; 6] {
        let mut angles = LegAngles([0.0; 6]);
        inverse_kinematics(&HeadGeometry::default(), &neutral_head_pose(), &mut angles)
            .expect("the neutral pose is reachable");
        angles.0
    }

    /// What a command left behind: the registers, every instruction that
    /// crossed the wire, the servos each grouped write carried, every grouped
    /// write in full, and every line it printed.
    struct Run {
        outcome: Result<(), PumpError>,
        registers: Rc<RefCell<FakeMachine>>,
        log: Rc<RefCell<Vec<(u8, u8)>>>,
        addressed: Rc<RefCell<Vec<Vec<u8>>>>,
        commanded: Rc<RefCell<Vec<GroupedWrite>>>,
        printed: Vec<String>,
    }

    impl Run {
        /// The command succeeded, or this says which one did not and why.
        fn ok(&self, what: &str) {
            if let Err(error) = &self.outcome {
                panic!("{what}: {error}");
            }
        }

        /// The command refused, and this is the refusal.
        fn err(&self, what: &str) -> &PumpError {
            match &self.outcome {
                Ok(()) => panic!("{what}"),
                Err(error) => error,
            }
        }

        /// Every goal this run commanded for `joint`, in the order it went out.
        ///
        /// The registers only hold where a run *ended*; a sweep is what it
        /// passed through on the way, and this is the only record of that.
        fn goal_series(&self, cfg: &Resolved, joint: JointId) -> Vec<f64> {
            let row = joint.index().expect("a named joint has a bus row");
            let id = cfg.map.ids()[row];
            let goal = reg_for(RegId::GoalPosition).addr;
            self.commanded
                .borrow()
                .iter()
                .filter(|write| write.addr == goal)
                .filter_map(|write| {
                    let (_, bytes) = write.entries.iter().find(|(servo, _)| *servo == id)?;
                    let counts =
                        i32::from_le_bytes(bytes.as_slice().try_into().expect("a goal is four"));
                    Some(cfg.map.present_rad(row, counts).expect("a goal places"))
                })
                .collect()
        }

        /// Whether this run ever commanded `joint` to `angle`, to within the
        /// whole count a goal register holds.
        fn commanded_angle(&self, cfg: &Resolved, joint: JointId, angle: f64) -> bool {
            let half_count = core::f64::consts::PI / 4096.0;
            self.goal_series(cfg, joint)
                .iter()
                .any(|commanded| (commanded - angle).abs() <= half_count)
        }

        /// Every servo's goal register is untouched: nothing here pinned or
        /// commanded anything.
        fn commanded_nothing(&self, cfg: &Resolved) {
            let machine = self.registers.borrow();
            for id in cfg.map.ids() {
                assert!(
                    machine.get(id, reg_for(RegId::GoalPosition)).is_none(),
                    "servo {id} was given a goal",
                );
            }
        }

        /// Nothing here took hold of the machine.
        ///
        /// Engaging is the only thing that prints the record of what it found
        /// and what it left, so the record's absence is the absence of an
        /// engage.
        fn armed_nothing(&self) {
            assert!(
                !self.printed.iter().any(|line| line.starts_with("found ")),
                "{:?}",
                self.printed,
            );
        }
    }

    /// Run one command against `machine`.
    fn run<F>(machine: FakeMachine, command: F) -> Run
    where
        F: FnOnce(Spy, &mut dyn Clock, &mut dyn FnMut(&str)) -> Result<(), PumpError>,
    {
        let spy = Spy::new(machine);
        let registers = spy.machine();
        let log = spy.log();
        let addressed = spy.addressed();
        let commanded = spy.commanded();
        let mut clock = TestClock::default();
        let mut printed = Vec::new();
        let outcome = command(spy, &mut clock, &mut |line| printed.push(line.to_string()));
        Run {
            outcome,
            registers,
            log,
            addressed,
            commanded,
            printed,
        }
    }

    /// Every servo's goal register, as the joint angles they encode.
    ///
    /// A servo whose goal register is absent is holding where it stands: its
    /// present position is the goal it is on. This keeps the check on where
    /// the machine ends up rather than on which registers happened to be
    /// written.
    fn goals(cfg: &Resolved, run: &Run) -> JointVector {
        let machine = run.registers.borrow();
        let mut joints = JointVector::default();
        for (row, id) in cfg.map.ids().iter().enumerate() {
            let held = machine
                .get(*id, reg_for(RegId::GoalPosition))
                .or_else(|| machine.get(*id, reg_for(RegId::PresentPosition)))
                .expect("every servo has a position");
            let counts = i32::from_le_bytes(held.try_into().expect("a goal is four bytes"));
            joints.set(
                JointId::ALL[row],
                cfg.map.present_rad(row, counts).expect("nine joints"),
            );
        }
        joints
    }

    /// Whether every servo is holding torque.
    fn torque(cfg: &Resolved, run: &Run) -> Vec<u8> {
        let machine = run.registers.borrow();
        cfg.map
            .ids()
            .iter()
            .map(|id| {
                machine
                    .get(*id, reg_for(RegId::TorqueEnable))
                    .map_or(0, |bytes| bytes[0])
            })
            .collect()
    }

    /// Two joint vectors agreeing to within the quantisation a goal register
    /// imposes — a servo holds whole counts, so a commanded angle and the angle
    /// read back from its goal register differ by up to half of one.
    ///
    /// An antenna is compared as the direction it points, which is how disarming
    /// compares it: the frame it is counted in is unbounded, and which turn of
    /// that frame an antenna ends on is a property of the arc it took rather
    /// than of where it came to rest.
    fn within_a_count(left: &JointVector, right: &JointVector, what: &str) {
        let half_count = core::f64::consts::PI / 4096.0;
        for (joint, angle) in left.joints() {
            let other = right.get(joint).expect("nine joints");
            let apart = match joint {
                JointId::AntennaRight | JointId::AntennaLeft => {
                    reachy_kin::wrap_to_pi(angle - other).abs()
                }
                _ => (angle - other).abs(),
            };
            assert!(
                apart <= half_count,
                "{what}: {joint} at {angle} against {other}"
            );
        }
    }

    /// The stow pose the tick path is commanded to and the stow angles
    /// disarming measures against are one pose.
    ///
    /// They are derived by different routes — one is a head pose the linkage is
    /// solved for per tick, the other is the joint vector the disarm
    /// configuration carries — and a drift between them would leave a machine
    /// that stowed perfectly refusing to release.
    #[test]
    fn the_commanded_stow_and_the_verified_stow_are_one_pose() {
        let cfg = resolved();
        let mut angles = LegAngles([0.0; 6]);
        inverse_kinematics(
            &cfg.motion.geom,
            &stow_pose_targets().head_pose_body,
            &mut angles,
        )
        .expect("the stow pose is reachable");
        let commanded = JointVector {
            body_yaw: stow_pose_targets().body_yaw,
            legs: angles.0,
            antennas: stow_pose_targets().antennas,
        };
        assert_eq!(commanded, cfg.disarm.stow_targets);
    }

    /// Every command that takes hold prints both records: what commissioning
    /// established, and what engaging found and left.
    ///
    /// They are two blocks because they happen at two different rates —
    /// commissioning once per process, engaging once per wake — and each is
    /// printed by the thing that did it, so a command cannot leave either out.
    /// Once each, however many moves follow.
    #[test]
    fn every_arming_command_prints_both_records() {
        /// A command as the dispatch hands it one.
        type Command =
            fn(&Resolved, Spy, &mut dyn Clock, &mut dyn FnMut(&str)) -> Result<(), PumpError>;

        let cfg = resolved();
        for (name, command) in [
            ("arm", arm as Command),
            ("up", up),
            ("hold", hold),
            ("stow", stow),
            ("demo", demo),
        ] {
            let machine = machine_at(&datumed_config(), &stow_legs());
            let run = run(machine, |port, clock, line| {
                command(&cfg, port, clock, line)
            });
            run.ok(name);

            let block = |prefix: &str| -> String {
                let found: Vec<&String> = run
                    .printed
                    .iter()
                    .filter(|line| line.starts_with(prefix))
                    .collect();
                assert_eq!(found.len(), 1, "{name}/{prefix}: {:?}", run.printed);
                found[0].clone()
            };

            let engaged = block("found ");
            for label in ["armed", "pull-in", "torque-on"] {
                assert!(engaged.contains(label), "{name}: no {label} line");
            }
            let commissioned = block("models ");
            for label in ["supply", "health", "registers"] {
                assert!(commissioned.contains(label), "{name}: no {label} line");
            }
        }
    }

    /// `up` lifts the machine from stow to the whole neutral configuration —
    /// head square and level, body square, antennas upright — and leaves torque
    /// on.
    #[test]
    fn up_lifts_the_machine_to_the_neutral_configuration() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &stow_legs());
        let run = run(machine, |port, clock, line| up(&cfg, port, clock, line));
        run.ok("the fixture machine lifts");

        within_a_count(
            &goals(&cfg, &run),
            &JointVector {
                body_yaw: 0.0,
                legs: neutral_legs(),
                antennas: [0.0, 0.0],
            },
            "the goals a lift leaves",
        );
        assert_eq!(torque(&cfg, &run), vec![1; JointId::COUNT]);
    }

    /// The line a run prints of what each period measured, or nothing if it
    /// never printed one.
    fn line_starting(run: &Run, what: &str) -> Option<String> {
        run.printed
            .iter()
            .find(|line| line.trim_start().starts_with(what))
            .cloned()
    }

    /// The nine figures a run's worst-lag line carries, degrees.
    fn lag_figures(run: &Run) -> Vec<f64> {
        let line = line_starting(run, "worst lag").expect("a run reports its per-joint lag");
        let inside = line
            .split_once('[')
            .and_then(|(_, rest)| rest.split_once(']'))
            .expect("the lag line is a bracketed row")
            .0;
        inside
            .split(", ")
            .map(|figure| figure.parse().expect("a lag figure is a number"))
            .collect()
    }

    /// A faulted move still says what it cost and what it measured.
    ///
    /// The fault is the final word and the command still fails, but the period
    /// counts and the per-joint lag are the numbers a bench run exists to bring
    /// back — and a fault is when they are worth the most, because it is the run
    /// nobody can explain without them.
    #[test]
    fn a_faulted_move_still_reports_what_it_measured() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        // A leg, because a lift moves the legs: the body and the antennas are
        // already where neutral wants them and are never commanded.
        let stuck = cfg.map.ids()[2];
        machine.stalled.push(stuck);
        let run = run(machine, |port, clock, line| up(&cfg, port, clock, line));

        let error = run.err("a servo that takes its goals and does not move");
        assert!(matches!(error, PumpError::Fault(_)), "{error}");

        let summary = run
            .printed
            .iter()
            .find(|line| line.contains("period(s)"))
            .expect("the faulted run reports its periods");
        assert!(
            summary.contains("blind")
                && summary.contains("worst jitter")
                && summary.contains("slip"),
            "{summary}"
        );

        let lags = lag_figures(&run);
        assert_eq!(lags.len(), JointId::COUNT);
        let threshold = cfg.motion.tracking.threshold_rad.to_degrees();
        assert!(
            lags[2] > threshold,
            "the stalled leg ran past the threshold: {lags:?}"
        );
        // Everything else tracked its goal to the count: a servo holds whole
        // counts and the goal it is compared against is the interpolant's
        // float, so half a count is the floor a perfect machine reports.
        let half_count = 180.0 / 4096.0;
        for (row, lag) in lags.iter().enumerate() {
            if row != 2 {
                assert!(*lag <= half_count, "row {row} tracked its goal: {lags:?}");
            }
        }
    }

    /// An obstruction the machine cannot get out from under still ends with the
    /// machine limp.
    ///
    /// The one assertion this whole shape exists for, and the fall-through that
    /// guarantees it. A grabbed head is answered by yielding under control, so
    /// the stow is tried first — but a jam that defeats the stow re-raises
    /// inside it, and there the wind-down gives up and torque comes off from
    /// wherever the head is. A head left held up by motors under a loop nobody
    /// is driving is the single state this platform must never be parked in.
    #[test]
    fn an_obstruction_that_defeats_the_stow_ends_with_torque_off() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        machine.stalled.push(cfg.map.ids()[2]);
        let run = run(machine, |port, clock, line| up(&cfg, port, clock, line));

        let error = run.err("a servo that takes its goals and does not move");
        assert!(
            matches!(error, PumpError::Fault(Fault::HeadObstructed { .. })),
            "{error}"
        );
        assert_eq!(
            error.class(Phase::UnderTorque),
            ErrorClass::SlowStowToRest,
            "the motors still command, so the answer is a controlled stow"
        );
        assert_eq!(
            torque(&cfg, &run),
            vec![0; JointId::COUNT],
            "the fault left a servo holding: {:?}",
            run.printed
        );

        for said in [
            "stowing under control",
            "the stow did not finish",
            "torque off, immediately",
            "at rest",
        ] {
            assert!(
                run.printed.iter().any(|line| line.contains(said)),
                "no {said:?} line: {:?}",
                run.printed
            );
        }
    }

    /// A command the tick refuses is not a fault: nothing was written, the
    /// machine is standing where the last accepted command put it, and it goes
    /// on holding.
    ///
    /// The dividing line the fault release turns on, asserted from the other
    /// side so that widening it into "any error releases" gets caught here.
    #[test]
    fn a_refused_command_leaves_the_machine_holding() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &neutral_legs());
        let run = run(machine, |port, clock, line| {
            yaw(&cfg, port, 90.0, clock, line)
        });

        let error = run.err("ninety degrees is past the cap");
        assert!(matches!(error, PumpError::Rejected(_)), "{error}");
        assert_eq!(error.class(Phase::UnderTorque), ErrorClass::Refuse);
        assert_eq!(
            error.fault(Phase::UnderTorque),
            None,
            "a command the tick would not take says nothing about the machine"
        );
        assert_eq!(torque(&cfg, &run), vec![1; JointId::COUNT]);
        assert!(
            !run.printed.iter().any(|line| line.contains("torque off")),
            "{:?}",
            run.printed
        );
    }

    /// Both torque-on gates refuse whichever phase they are classified in, and
    /// they name no condition of the machine.
    ///
    /// What an unattended caller turns on: a gate wrote nothing, so the machine
    /// is still limp where it was and the next request may simply ask again,
    /// while everything else out of an engage leaves something to bring back to
    /// the minimum risk condition. The phase is stated both ways here because
    /// the gates are the case where it must not matter — both are judged before
    /// a transaction goes out, so a caller that got the derivation wrong must
    /// still not de-torque and park a machine that was never torqued. Built
    /// from `engage_gates` itself rather than from hand-written variants, so a
    /// gate that changed shape is caught here.
    #[test]
    fn the_two_torque_on_gates_refuse_in_either_phase() {
        let cfg = resolved();
        let healthy = Rail {
            voltages: [12.0; JointId::COUNT],
            health: [ServoHealth::default(); JointId::COUNT],
        };
        engage_gates(&cfg.arm, &healthy).expect("a healthy rail passes both gates");

        let mut sagging = healthy;
        sagging.voltages[4] = cfg.arm.min_arm_voltage - 0.5;
        let mut latched = healthy;
        latched.health[2].bits = 0x20;

        for rail in [sagging, latched] {
            let refused = engage_gates(&cfg.arm, &rail).expect_err("the gate refuses");
            let error = PumpError::from(refused);
            for phase in [Phase::PreTorque, Phase::UnderTorque] {
                assert_eq!(
                    error.class(phase),
                    ErrorClass::Refuse,
                    "a gate wrote nothing, so de-torquing and parking would be a machine that \
                     was never torqued: {error}"
                );
                assert_eq!(error.fault(phase), None, "{error}");
            }
        }

        // Two `Sequence` errors that are not gates, because without them
        // relaxing the arm to `Sequence(_)` passes: an engage that lost a servo
        // mid-flight would then read as "nothing was written and the machine is
        // limp where it stands", on a machine that may be part way through
        // taking torque.
        let context = StepContext::reg(SeqStep::PinAndEnable, 13, RegId::TorqueEnable);
        for other in [
            PumpError::Sequence(SeqError::NoAnswer { context }),
            PumpError::Sequence(SeqError::VerifyMismatch {
                context,
                expected: RegValue::U8(1),
                read_back: RegValue::U8(0),
            }),
            PumpError::TorqueOffUnacked { id: 13 },
        ] {
            assert_ne!(
                other.class(Phase::UnderTorque),
                ErrorClass::Refuse,
                "{other}"
            );
        }
    }

    /// An engage that dies after the first enable takes the machine limp on its
    /// way out.
    ///
    /// Half-enabled is the worst state the torque-on path can reach: some
    /// servos holding, no loop driving them, and a caller with an error and no
    /// handle to release through. The releasing is the sequence's own to do.
    #[test]
    fn an_engage_that_fails_after_an_enable_releases_torque() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        // The last enable acknowledged and dropped, which is a read-back
        // mismatch from the host: the eight before it took torque.
        machine
            .ignored
            .push((cfg.map.ids()[8], reg_for(RegId::TorqueEnable).addr));
        let run = run(machine, |port, clock, line| up(&cfg, port, clock, line));

        run.err("an enable that does not land stops the engage");
        assert_eq!(torque(&cfg, &run), vec![0; JointId::COUNT]);
        assert!(
            run.printed
                .iter()
                .any(|line| line.contains("releasing immediately")),
            "{:?}",
            run.printed
        );
        assert!(
            run.addressed.borrow().is_empty(),
            "nothing was ever commanded: {:?}",
            run.addressed.borrow()
        );
    }

    /// A machine chasing its goals reports the lag it ran, and is not faulted
    /// for it.
    ///
    /// The threshold, its window and the stow tolerance are all provisional
    /// against a lag nobody has measured; this is the measurement, and it is
    /// taken on every move rather than on a run someone remembered to instrument.
    #[test]
    fn a_chasing_machine_reports_the_lag_it_ran() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        for id in cfg.map.ids() {
            machine.delay.insert(id, 3);
        }
        let run = run(machine, |port, clock, line| up(&cfg, port, clock, line));
        run.ok("a joint closing on its goal is not a joint that lost it");

        let lags = lag_figures(&run);
        // The six legs are what a lift moves; the body and the antennas are
        // already at neutral and are never commanded, so they never run behind.
        for row in 1..=6 {
            assert!(
                lags[row] > 0.0,
                "leg row {row} moved, so it ran behind: {lags:?}"
            );
        }
        assert_eq!([lags[0], lags[7], lags[8]], [0.0; 3], "{lags:?}");
    }

    /// A session asked for a trace gets a row for every control period it
    /// turned, in order, and is told where they went.
    ///
    /// The rows are the point: a summary says a move took two seconds, and only
    /// the periods say what the machine was doing during them.
    #[test]
    fn a_traced_session_writes_a_row_for_every_period() {
        let path = crate::testutil::scratch_path("session.csv");
        let mut cfg = resolved();
        cfg.trace = Some(path.clone());
        let machine = machine_at(&datumed_config(), &stow_legs());
        let run = run(machine, |port, clock, line| up(&cfg, port, clock, line));
        run.ok("the fixture machine lifts");

        let text = std::fs::read_to_string(&path).expect("the trace is where it was said to be");
        std::fs::remove_file(&path).expect("the scratch file goes away");
        let lines: Vec<&str> = text.lines().collect();
        assert!(lines[0].starts_with("run,tick,t_s,phase"), "{}", lines[0]);
        assert!(lines.len() > 1, "{text}");
        for (period, row) in lines[1..].iter().enumerate() {
            let cells: Vec<&str> = row.split(',').collect();
            assert_eq!(cells[0], "0", "one run in this session: {row}");
            assert_eq!(cells[1], period.to_string(), "periods in order: {row}");
        }
        assert!(
            run.printed
                .iter()
                .any(|line| line.contains(&format!("{}", path.display()))),
            "{:?}",
            run.printed
        );
    }

    /// Two commands appending to one trace file write two runs, not two copies
    /// of run zero.
    ///
    /// This is the workflow the file format exists for — `up --trace f.csv`
    /// then `stow --trace f.csv` — and each command engages the machine for
    /// itself, so nothing in the process carries a count between them. A reader
    /// grouping rows by `run` has to get two series out of this file.
    #[test]
    fn two_commands_sharing_a_trace_file_write_two_runs() {
        let path = crate::testutil::scratch_path("two-commands.csv");
        let mut cfg = resolved();
        cfg.trace = Some(path.clone());

        let first = run(
            machine_at(&datumed_config(), &stow_legs()),
            |port, clock, line| up(&cfg, port, clock, line),
        );
        first.ok("the fixture machine lifts");
        let second = run(
            machine_at(&datumed_config(), &stow_legs()),
            |port, clock, line| stow(&cfg, port, clock, line),
        );
        second.ok("the fixture machine folds");

        let text = std::fs::read_to_string(&path).expect("the trace is where it was said to be");
        std::fs::remove_file(&path).expect("the scratch file goes away");
        let runs: Vec<&str> = text
            .lines()
            .skip(1)
            .map(|row| row.split(',').next().expect("a row has a first cell"))
            .collect();
        assert!(runs.contains(&"0"), "{text}");
        assert!(
            runs.contains(&"1"),
            "the second command reused the first's run number: {text}"
        );
        assert!(
            second.printed.iter().any(|line| line.contains("as run 1")),
            "{:?}",
            second.printed
        );
    }

    /// A run that faulted writes its trace too — and the trace is one row per
    /// period the run turned.
    ///
    /// This is the trace's headline use: the periods of the run that lost the
    /// machine are what says why. Reporting on the success path only, or
    /// returning before the report on a fault, would lose exactly the runs
    /// worth reading.
    #[test]
    fn a_faulted_run_still_writes_its_trace() {
        let path = crate::testutil::scratch_path("faulted.csv");
        let mut cfg = resolved();
        cfg.trace = Some(path.clone());
        cfg.motion.tracking.threshold_rad = 0.02;
        cfg.motion.tracking.progress_min_rad = 1.0;
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        // Nine servos following their goals a few periods behind, against a
        // progress minimum nothing closes: the tracking monitor gives up.
        for id in cfg.map.ids() {
            machine.delay.insert(id, 4);
        }
        let run = run(machine, |port, clock, line| up(&cfg, port, clock, line));

        let error = run.err("nothing closes a whole radian in a window");
        assert!(error.fault(Phase::UnderTorque).is_some(), "{error}");
        let text = std::fs::read_to_string(&path).expect("the trace is where it was said to be");
        std::fs::remove_file(&path).expect("the scratch file goes away");

        // The count the summary reports and the count that reached the file are
        // the same count, faulting period included.
        let periods = |line: &str| {
            line.split_whitespace()
                .next()
                .expect("a report line starts with its count")
                .parse::<usize>()
                .expect("a count")
        };
        let turned = run
            .printed
            .iter()
            .find(|line| line.contains("period(s), ") && line.contains("commanding"))
            .map(|line| periods(line.trim()))
            .expect("a faulted run still reports what it measured");
        let appended = run
            .printed
            .iter()
            .find(|line| line.contains("appended to"))
            .expect("and still writes its periods");
        assert_eq!(periods(appended.trim()), turned, "{appended}");
        assert!(appended.contains("as run 0"), "{appended}");
        // Run 0's rows, and not the file's: the wind-down that answers the
        // fault is a run of its own and appends its own periods behind these.
        let rows = text
            .lines()
            .skip(1)
            .filter(|row| row.starts_with("0,"))
            .count();
        assert_eq!(rows, turned, "the faulted run's rows");
    }

    /// A trace that cannot be written is said, and the move that ran still ran.
    ///
    /// `--trace` takes any path an operator types, so a destination that will
    /// not open is an ordinary typo. Diagnostic output failing is not the
    /// machine failing.
    #[test]
    fn a_trace_that_cannot_be_written_does_not_undo_the_move() {
        let path = crate::testutil::scratch_path("no-such-directory").join("session.csv");
        let mut cfg = resolved();
        cfg.trace = Some(path.clone());
        let machine = machine_at(&datumed_config(), &stow_legs());
        let run = run(machine, |port, clock, line| up(&cfg, port, clock, line));
        run.ok("a file that will not open is not the machine's problem");

        assert!(
            run.printed.iter().any(|line| {
                line.contains("could not be written")
                    && line.contains(&format!("{}", path.display()))
            }),
            "{:?}",
            run.printed
        );
        assert!(!path.exists(), "and nothing was created on the way");
    }

    /// A session that asked for no trace writes none, and says nothing about
    /// one.
    #[test]
    fn an_untraced_session_writes_nothing() {
        let cfg = resolved();
        assert_eq!(cfg.trace, None);
        let machine = machine_at(&datumed_config(), &stow_legs());
        let run = run(machine, |port, clock, line| up(&cfg, port, clock, line));
        run.ok("the fixture machine lifts");
        assert!(
            !run.printed.iter().any(|line| line.contains("trace")),
            "{:?}",
            run.printed
        );
    }

    /// The two instants a move reports, in the three shapes a run can end in.
    ///
    /// A run whose elapsed time is all the operator gets cannot tell a move that
    /// finished commanding quickly from a machine that got there quickly, and on
    /// this platform those differ by most of the motion.
    #[test]
    fn the_settle_line_says_when_commanding_ended_and_when_the_machine_arrived() {
        let arrived = MoveSummary {
            commanded: Some(Duration::from_millis(50)),
            settled: Some(Duration::from_millis(670)),
            ..MoveSummary::default()
        };
        let line = settle_line(&arrived).expect("a move that finished commanding says so");
        assert!(line.contains("0.05 s"), "{line}");
        assert!(line.contains("0.62 s later"), "{line}");
        assert!(line.contains("0.67 s"), "{line}");

        let short = MoveSummary {
            commanded: Some(Duration::from_millis(50)),
            unsettled: Some((JointId::Leg(3), 0.1)),
            ..MoveSummary::default()
        };
        let line = settle_line(&short).expect("a move that finished commanding says so");
        assert!(line.contains("leg 4"), "{line}");
        assert!(line.contains("5.73°"), "{line}");

        let cut = MoveSummary {
            commanded: Some(Duration::from_millis(50)),
            ..MoveSummary::default()
        };
        let line = settle_line(&cut).expect("a move that finished commanding says so");
        assert!(line.contains("ended before"), "{line}");

        // A hold commands nothing, so it has no such instant and says nothing.
        assert_eq!(settle_line(&MoveSummary::default()), None);
    }

    /// `stow` puts the machine at the pose disarming verifies, and leaves
    /// torque on: the release is a separate, explicit command.
    #[test]
    fn stow_moves_the_machine_to_the_pose_disarming_verifies() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &neutral_legs());
        let run = run(machine, |port, clock, line| stow(&cfg, port, clock, line));
        run.ok("the fixture machine stows");

        within_a_count(
            &goals(&cfg, &run),
            &cfg.disarm.stow_targets,
            "the goals a stow leaves",
        );
        assert_eq!(torque(&cfg, &run), vec![1; JointId::COUNT]);
    }

    /// The line an operator reads after a release says where the machine was,
    /// which joints could not be measured, and — the part that decides whether
    /// a hand can go on the head — which servos never acknowledged torque off.
    #[test]
    fn the_release_report_names_the_servos_that_may_still_be_holding() {
        let clean = DisarmSummary {
            form: ReleaseForm::Orderly,
            present: JointVector::default(),
            deviation: [0.0; JointId::COUNT],
            unmeasured: [None; JointId::COUNT],
            released: [true; JointId::COUNT],
            at_stow: true,
        };
        let printed = disarm_line(&clean);
        assert!(printed.contains("at stow"), "{printed}");
        assert!(
            printed.contains("every servo acknowledged torque off"),
            "{printed}"
        );
        assert!(!printed.contains("unmeasured"), "{printed}");

        let mut ragged = clean;
        ragged.at_stow = false;
        ragged.unmeasured[7] = Some(SeqError::Refused {
            context: StepContext::reg(SeqStep::VerifyAtStow, 41, RegId::PresentPosition),
            code: 0x20,
        });
        ragged.released[4] = false;
        ragged.released[8] = false;
        let printed = disarm_line(&ragged);
        assert!(printed.contains("not at stow"), "{printed}");
        assert!(printed.contains("unmeasured: right antenna"), "{printed}");
        // With the cause, not just the name: which of the several ways a read
        // can fail this was is what decides the operator's next move.
        assert!(printed.contains("status code 0x20"), "{printed}");
        assert!(printed.contains("NO ACKNOWLEDGEMENT"), "{printed}");
        assert!(printed.contains("leg 4"), "{printed}");
        assert!(printed.contains("left antenna"), "{printed}");
    }

    /// Nine failed reads during an orderly release is not the fault release,
    /// and does not print as one.
    ///
    /// A dead adapter or an unpowered rail during `off` leaves every joint
    /// unread, which is exactly what the fault release's summary also carries.
    /// Reading them as the same thing would tell an operator the machine was
    /// deliberately let go without a look, when in fact the bus stopped
    /// answering and the release writes may have gone nowhere either.
    #[test]
    fn an_orderly_release_that_could_read_nothing_says_so() {
        let context = StepContext::reg(SeqStep::VerifyAtStow, 41, RegId::PresentPosition);
        let blind = DisarmSummary {
            form: ReleaseForm::Orderly,
            present: JointVector::default(),
            deviation: [0.0; JointId::COUNT],
            unmeasured: [Some(SeqError::NoAnswer { context }); JointId::COUNT],
            released: [true; JointId::COUNT],
            at_stow: false,
        };
        let printed = disarm_line(&blind);
        assert!(
            !printed.contains("immediately"),
            "this release looked and got nothing: {printed}"
        );
        assert!(
            printed.contains("not one of the nine joints could be read"),
            "{printed}"
        );
        assert!(printed.contains("unmeasured: "), "{printed}");
        assert!(printed.contains("no answer"), "{printed}");
    }

    /// The fault release reads as what it is: a release that deliberately did
    /// not look, rather than one whose nine reads all failed.
    #[test]
    fn the_fault_release_report_says_nothing_was_measured() {
        let summary = DisarmSummary {
            form: ReleaseForm::Immediate,
            present: JointVector::default(),
            deviation: [0.0; JointId::COUNT],
            unmeasured: [None; JointId::COUNT],
            released: [true; JointId::COUNT],
            at_stow: false,
        };
        let printed = disarm_line(&summary);
        assert!(printed.contains("torque off, immediately"), "{printed}");
        assert!(printed.contains("nothing was measured"), "{printed}");
        assert!(
            !printed.contains("unmeasured:"),
            "nine joint names would say the same thing worse: {printed}"
        );
        assert!(
            !printed.contains("not at stow"),
            "nothing here is a claim about where the head is: {printed}"
        );
        assert!(
            printed.contains("every servo acknowledged torque off"),
            "{printed}"
        );
    }

    /// `off` releases torque on a machine at stow, and never arms to do it: the
    /// only writes it makes are the releases themselves.
    #[test]
    fn off_releases_a_machine_at_stow_without_arming_it() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &cfg.disarm.stow_targets.legs);
        for (row, id) in cfg.map.ids().iter().enumerate() {
            let joint = JointId::ALL[row];
            let counts = cfg
                .map
                .goal_counts(
                    row,
                    cfg.disarm.stow_targets.get(joint).expect("nine joints"),
                )
                .expect("a stow angle places");
            machine.set(*id, reg_for(RegId::PresentPosition), &counts.to_le_bytes());
            machine.set(*id, reg_for(RegId::TorqueEnable), &[1]);
        }
        let run = run(machine, |port, clock, line| off(&cfg, port, clock, line));
        run.ok("a machine at stow releases");

        assert_eq!(torque(&cfg, &run), vec![0; JointId::COUNT]);
        assert!(
            run.printed.iter().any(|line| line.contains("at stow")),
            "{:?}",
            run.printed
        );
        // Torque ending at zero is true of an `off` that armed first as well,
        // so it says nothing on its own. Arming enables torque where every
        // joint stands and pulls one outside its window to the nearer bound: on a
        // machine about to be released that is a movement nobody is standing
        // ready for, and these two are what say it did not happen.
        run.armed_nothing();
        run.commanded_nothing(&cfg);
    }

    /// A machine physically folded, whose antennas read a turn away from the
    /// fold, releases: the whole path — counts past one turn decoded by the map,
    /// the circular deviation, the gate — carries the frame extended position
    /// mode leaves an antenna in.
    ///
    /// A limp antenna on this unit rests past the half turn, and nothing
    /// renormalises it.
    #[test]
    fn off_releases_a_machine_whose_antennas_read_a_turn_from_their_fold() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &cfg.disarm.stow_targets.legs);
        // The right antenna a turn below its fold, the left a turn above it:
        // both physically at stow, neither within a turn of it in the reading.
        let turns = [-core::f64::consts::TAU, core::f64::consts::TAU];
        for (row, id) in cfg.map.ids().iter().enumerate() {
            let joint = JointId::ALL[row];
            let mut angle = cfg.disarm.stow_targets.get(joint).expect("nine joints");
            if let Some(side) = [JointId::AntennaRight, JointId::AntennaLeft]
                .iter()
                .position(|antenna| *antenna == joint)
            {
                angle += turns[side];
            }
            let counts = cfg
                .map
                .goal_counts(row, angle)
                .expect("a stow angle a turn out still places");
            machine.set(*id, reg_for(RegId::PresentPosition), &counts.to_le_bytes());
            machine.set(*id, reg_for(RegId::TorqueEnable), &[1]);
        }

        let run = run(machine, |port, clock, line| off(&cfg, port, clock, line));
        run.ok("a machine at its fold is at stow whichever turn it reads");
        assert_eq!(torque(&cfg, &run), vec![0; JointId::COUNT]);
        assert!(
            run.printed.iter().any(|line| line.contains("at stow")),
            "{:?}",
            run.printed
        );
        run.armed_nothing();
        run.commanded_nothing(&cfg);
    }

    /// The bound the motion layer refuses an antenna goal on is a count bound,
    /// and this is the one place the two layers that hold it meet: the tick
    /// works in radians and the map is what turns a radian into the count the
    /// goal register takes.
    ///
    /// Each end is exactly the last count extended position mode accepts —
    /// asymmetric, because zero radians sits at count 2048 — and a radian past
    /// either end is a count the register would refuse.
    #[test]
    fn the_antenna_goal_bounds_are_the_last_counts_the_register_holds() {
        const REGISTER_LIMIT: i32 = 1_048_575;
        let cfg = resolved();
        let ends = [
            (7usize, ANTENNA_GOAL_MAX_RAD, REGISTER_LIMIT, 1.0),
            (8, ANTENNA_GOAL_MIN_RAD, -REGISTER_LIMIT, -1.0),
        ];
        for (row, edge, limit, past) in ends {
            assert_eq!(
                cfg.map.goal_counts(row, edge).expect("the bound places"),
                limit
            );
            let over = cfg
                .map
                .goal_counts(row, edge + past)
                .expect("a count outside the register's range is still an i32");
            assert!(
                over.abs() > REGISTER_LIMIT,
                "a radian past the bound is {over} counts"
            );
        }
    }

    /// An antenna found several turns from zero keeps that frame: it pins where
    /// it stands, and the direction it is then commanded to resolves into the
    /// turn it was found in rather than folding back into the first one.
    ///
    /// The motion layer decides this in radians; what only the map and a
    /// register file can show is that the counts written are past one turn and
    /// the sweep commanded is still the short way round.
    #[test]
    fn an_antenna_found_turns_from_zero_is_commanded_in_the_turn_it_was_found_in() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &neutral_legs());
        // Three turns and 38 counts past the count frame's own zero: a limp
        // antenna in extended position mode reads where it physically is.
        let found_counts: i32 = 3 * 4096 + 38;
        machine.set(
            cfg.map.ids()[7],
            reg_for(RegId::PresentPosition),
            &found_counts.to_le_bytes(),
        );
        let found = cfg
            .map
            .present_rad(7, found_counts)
            .expect("a multi-turn reading places");

        let run = run(machine, |port, clock, line| {
            antennas(&cfg, port, 0.5, -0.5, clock, line)
        });
        run.ok("an antenna a few turns out still moves");

        let resolved_goal = found + wrap_to_pi(0.5 - found);
        assert!(
            (resolved_goal - found).abs() <= core::f64::consts::PI,
            "the commanded sweep is the short way: {}",
            resolved_goal - found
        );
        assert!(
            run.commanded_angle(&cfg, JointId::AntennaRight, resolved_goal),
            "the goal that went out is the direction resolved into the found turn: {:?}",
            run.goal_series(&cfg, JointId::AntennaRight).last()
        );

        // And in counts, which is where a fold back into the first turn would
        // show: every goal written stays in the turn the antenna was found in.
        let goal = run
            .registers
            .borrow()
            .get(cfg.map.ids()[7], reg_for(RegId::GoalPosition))
            .map(|bytes| i32::from_le_bytes(bytes.try_into().expect("a goal is four bytes")))
            .expect("the right antenna was given a goal");
        assert!(
            goal > 2 * 4096,
            "a goal folded into one turn would read {goal}"
        );
        assert!(
            (goal - found_counts).abs() <= 2048,
            "and it is half a turn at most from where the antenna was found: {goal}"
        );
    }

    /// A machine anywhere but stow releases anyway, and says where it was.
    /// `off` arms nothing and commands nothing on the way.
    #[test]
    fn off_away_from_stow_still_releases() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &neutral_legs());
        for id in cfg.map.ids() {
            machine.set(id, reg_for(RegId::TorqueEnable), &[1]);
        }

        let released = run(machine, |port, clock, line| off(&cfg, port, clock, line));
        released.ok("nothing gates a release");
        assert_eq!(torque(&cfg, &released), vec![0; JointId::COUNT]);
        assert!(
            released
                .printed
                .iter()
                .any(|line| line.contains("not at stow")),
            "{:?}",
            released.printed
        );
        assert!(
            released
                .printed
                .iter()
                .any(|line| line.contains("every servo acknowledged torque off")),
            "{:?}",
            released.printed
        );
        released.armed_nothing();
        released.commanded_nothing(&cfg);
    }

    /// `hold` commands nothing for the configured length of time, and measures
    /// the machine every period while it does.
    #[test]
    fn hold_commands_nothing_and_measures_every_period() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &neutral_legs());
        let run = run(machine, |port, clock, line| hold(&cfg, port, clock, line));
        run.ok("a machine holds");

        assert!(
            run.addressed.borrow().is_empty(),
            "a hold writes no goals: {:?}",
            run.addressed.borrow()
        );
        // One period per tick of the configured dwell, plus the period the
        // deadline is noticed in.
        let periods = u64::from(cfg.tick_hz) * cfg.hold_duration.as_secs();
        let summary = run
            .printed
            .iter()
            .find(|line| line.contains("period(s)"))
            .expect("the hold reports what it cost");
        assert!(
            summary.starts_with(&format!("  {} period(s)", periods + 1)),
            "{summary}"
        );
    }

    /// A hold prints what the periods turned up and not the disposition it
    /// already knows.
    ///
    /// The suppressed half and the reported half are one condition, and getting
    /// it backwards — or dropping it — leaves a supervised hold silent about a
    /// lost read or a health latch while announcing, once, that it is holding.
    /// That is the command an operator runs to decide whether the rig is
    /// behaving.
    #[test]
    fn a_hold_reports_the_news_and_not_the_disposition_it_already_knows() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &neutral_legs());
        // The input-voltage bit set on a healthy rail: what this unit's servos
        // actually report, raising no fault and reported verbatim.
        for id in cfg.map.ids() {
            machine.set(id, reg_for(RegId::HardwareErrorStatus), &[0x01]);
        }
        let run = run(machine, |port, clock, line| hold(&cfg, port, clock, line));
        run.ok("the input-voltage bit alone is not a fault");

        assert!(
            run.printed
                .iter()
                .any(|line| line.trim_start().starts_with("health ") && line.contains("0x01")),
            "the health latch is news: {:?}",
            run.printed
        );
        assert!(
            !run.printed.iter().any(|line| line.trim() == "holding"),
            "a hold already knows it is holding: {:?}",
            run.printed
        );
    }

    /// `hold_events` hands every tick event to its caller and renders nothing.
    ///
    /// A filter or a report leaking into this entry point would reach every
    /// caller as text it cannot suppress — the zero-policy guarantee is the
    /// point.
    #[test]
    fn hold_events_hands_over_every_event_and_renders_nothing() {
        // Arming narrates, so the mark separates what the sequence said from
        // what the hold said — and what the hold said is meant to be nothing at
        // all, not merely nothing of three shapes anybody thought to name.
        const ARMED: &str = "-- armed --";

        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &neutral_legs());
        let events: Rc<RefCell<Vec<TickEvent>>> = Rc::new(RefCell::new(Vec::new()));
        let summary = Rc::new(RefCell::new(None));
        let seen = events.clone();
        let measured = summary.clone();
        let run = run(machine, move |port, clock, line| {
            let mut machine = commission(&cfg, port, clock, line)?;
            let mut engaged = machine.engage(clock, line)?;
            line(ARMED);
            let outcome = engaged.hold_events(cfg.hold_duration, clock, &mut |event| {
                seen.borrow_mut().push(event);
            })?;
            *measured.borrow_mut() = Some(outcome);
            Ok(())
        });
        run.ok("a machine holds");

        let events = events.borrow();
        assert!(
            events
                .iter()
                .any(|event| matches!(event, TickEvent::Command(CommandDisposition::Held))),
            "the disposition is the caller's to drop: {events:?}",
        );
        // The event `hold` filters and one it does not: a passthrough that let
        // the disposition alone through would have re-added a filter.
        assert!(
            events
                .iter()
                .any(|event| matches!(event, TickEvent::Health(_))),
            "the health sweep reaches the caller too: {events:?}",
        );
        let mark = run
            .printed
            .iter()
            .position(|line| line == ARMED)
            .expect("the mark is printed");
        assert_eq!(
            &run.printed[mark + 1..],
            [] as [String; 0],
            "hold_events renders nothing: {:?}",
            &run.printed[mark + 1..],
        );
        // +1: the tick that notices the deadline has elapsed.
        let cfg = resolved();
        let summary = summary.borrow().expect("the hold returns its summary");
        assert_eq!(
            summary.ticks,
            u64::from(cfg.tick_hz) * cfg.hold_duration.as_secs() + 1
        );
    }

    /// `move_retargeting` hands the caller the move in flight: answering with
    /// another target turns the head around and the call returns at that one.
    ///
    /// The entry point a program executing a timeline it does not own needs.
    /// The pump's own tests pin the periods; what this pins is the wrapper —
    /// that the pair the caller answers with really becomes the command, both
    /// halves of it, and that nothing about the reply is dropped on the way.
    #[test]
    fn a_move_the_caller_replaces_returns_at_the_replacement() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &stow_legs());
        let asked = Rc::new(RefCell::new(0usize));
        let asks = asked.clone();
        let ran = Rc::new(RefCell::new(0u64));
        let counted = ran.clone();
        let run = run(machine, move |port, clock, line| {
            let mut machine = commission(&cfg, port, clock, line)?;
            let mut engaged = machine.engage(clock, line)?;
            let summary = engaged.move_retargeting(
                neutral_targets(),
                cfg.up_durations(),
                clock,
                line,
                &mut || {
                    let mut asked = asks.borrow_mut();
                    *asked += 1;
                    (*asked == 20).then(|| (stow_pose_targets(), cfg.stow_durations()))
                },
            )?;
            *counted.borrow_mut() = summary.ticks;
            Ok(())
        });
        run.ok("the replacement carries the head back down");

        assert!(
            *asked.borrow() > 20,
            "the caller is asked every period, not once: {}",
            asked.borrow()
        );
        // The durations half of the caller's answer is what the replacement ran
        // on. Dropping it — keeping the `durations` still in scope from the
        // parameter list — reaches the same pose and completes after the same
        // splice, so nothing else in this test would notice; what it changes is
        // the fold's pace, and a fold carried faster than the clock its step
        // bounds were sized against faults partway and drops the head.
        let cfg = resolved();
        let periods = |span: Duration| {
            let period = Duration::from_secs(1) / cfg.tick_hz;
            u64::try_from(span.as_nanos() / period.as_nanos()).expect("a small count")
        };
        let (turn_at, ticks) = (20, *ran.borrow());
        assert!(
            ticks >= turn_at + periods(cfg.stow_duration)
                && ticks < turn_at + periods(cfg.up_duration),
            "the fold ran on the fold's clock: {ticks} periods, turned at {turn_at}, fold {}, \
             raise {}",
            periods(cfg.stow_duration),
            periods(cfg.up_duration),
        );
        within_a_count(
            &goals(&resolved(), &run),
            &JointVector {
                body_yaw: 0.0,
                legs: stow_legs(),
                antennas: STOW_ANTENNAS,
            },
            "the machine came back to its fold",
        );
        assert!(
            run.printed
                .iter()
                .any(|line| line.contains("replacing the move that was running")),
            "the splice is narrated: {:?}",
            run.printed
        );
    }

    /// `move_retargeting_events` hands the caller the run's events as values
    /// and still prints the run.
    #[test]
    fn a_move_hands_out_its_events_and_still_prints_them() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &stow_legs());
        let events: Rc<RefCell<Vec<TickEvent>>> = Rc::new(RefCell::new(Vec::new()));
        let seen = events.clone();
        // Far under the head group's floor for the stow-to-neutral span, so the
        // pump right-sizes the clock before it commands anything.
        let hurried = MoveDurations::uniform(Duration::from_millis(200));
        let run = run(machine, move |port, clock, line| {
            let mut machine = commission(&cfg, port, clock, line)?;
            let mut engaged = machine.engage(clock, line)?;
            engaged.move_retargeting_events(
                neutral_targets(),
                hurried,
                clock,
                line,
                &mut |event| seen.borrow_mut().push(event),
                &mut || None,
            )?;
            Ok(())
        });
        run.ok("a hurried raise runs on the clock it was right-sized to");

        let events = events.borrow();
        let stretch = events
            .iter()
            .find_map(|event| match event {
                TickEvent::Stretched(stretch) => Some(*stretch),
                _ => None,
            })
            .unwrap_or_else(|| panic!("the caller is told the clock moved: {events:?}"));
        assert_eq!(stretch.requested, hurried);
        assert!(
            stretch.effective.head > hurried.head,
            "{:?}",
            stretch.effective
        );
        assert!(
            events
                .iter()
                .any(|event| matches!(event, TickEvent::Completed)),
            "every event reaches the caller, not only the interesting one: {events:?}",
        );
        assert!(
            run.printed
                .iter()
                .any(|line| line.contains("clock stretched to fit the span")),
            "the run is still printed: {:?}",
            run.printed
        );
    }

    /// `yaw` turns the body and leaves the head where it was relative to it, so
    /// the six cranks do not move at all.
    #[test]
    fn yaw_turns_the_body_and_leaves_the_head_where_it_was() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &neutral_legs());
        let run = run(machine, |port, clock, line| {
            yaw(&cfg, port, 30.0, clock, line)
        });
        run.ok("thirty degrees is inside the bench cap");

        let held = goals(&cfg, &run);
        assert!(
            (held.body_yaw - 30f64.to_radians()).abs() < core::f64::consts::PI / 4096.0,
            "{}",
            held.body_yaw.to_degrees()
        );
        within_a_count(
            &JointVector {
                body_yaw: 0.0,
                ..held
            },
            &JointVector {
                body_yaw: 0.0,
                legs: neutral_legs(),
                antennas: [0.0, 0.0],
            },
            "the joints a yaw leaves alone",
        );
    }

    /// A yaw past the bench's own cap is refused by the envelope on the
    /// accepting period, and the machine is left armed and holding with nothing
    /// commanded.
    #[test]
    fn a_yaw_past_the_bench_cap_is_refused_with_nothing_commanded() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &neutral_legs());
        let run = run(machine, |port, clock, line| {
            yaw(&cfg, port, 90.0, clock, line)
        });

        let error = run.err("ninety degrees is past the cap");
        assert!(matches!(error, PumpError::Rejected(_)), "{error}");
        assert!(error.to_string().contains("body yaw"), "{error}");
        assert!(
            run.addressed.borrow().is_empty(),
            "a refused command writes no goals"
        );
        assert_eq!(torque(&cfg, &run), vec![1; JointId::COUNT]);

        // A refusal on the accepting period measured nothing, so it reports
        // nothing: a period line and a lag row of zeros would read as a clean
        // run of nothing at the moment an operator is diagnosing the refusal.
        assert_eq!(line_starting(&run, "worst lag"), None, "{:?}", run.printed);
        for line in &run.printed {
            assert!(!line.contains("period(s)"), "{line}");
        }
    }

    /// `antennas` moves the antennas and addresses nothing else: the legs' and
    /// the body's goal registers are never in a frame.
    #[test]
    fn antennas_address_only_the_antennas() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &neutral_legs());
        let run = run(machine, |port, clock, line| {
            antennas(&cfg, port, 1.0, -0.5, clock, line)
        });
        run.ok("an antenna angle inside the bound moves");

        let held = goals(&cfg, &run);
        within_a_count(
            &held,
            &JointVector {
                body_yaw: 0.0,
                legs: neutral_legs(),
                antennas: [1.0, -0.5],
            },
            "the goals an antenna command leaves",
        );

        let antenna_servos: Vec<u8> = JointId::ALL
            .into_iter()
            .enumerate()
            .filter(|(_, joint)| joint.group() == JointGroup::Antennas)
            .map(|(row, _)| cfg.map.ids()[row])
            .collect();
        // Every frame but the first addresses the antennas alone. The first
        // period re-commands the legs once because the goals arming pinned and
        // the head pose solved from them differ by a rounding error — enough to
        // count as a change, far too little to be a count, so the goal
        // registers do not move.
        let frames = run.addressed.borrow();
        for frame in frames.iter().skip(1) {
            assert_eq!(*frame, antenna_servos, "{frames:?}");
        }
        assert!(frames.len() > 1, "{frames:?}");
    }

    /// An antenna direction past the half turn is an ordinary command, and the
    /// bench command inherits the machine's one arc policy: four radians, from a
    /// right antenna found at zero, sweeps four radians up over the head rather
    /// than 2.283 the short way down through its own outboard point. The short
    /// way is the maximal-interference arc, whoever asked for it.
    #[test]
    fn a_bench_antenna_command_takes_the_inboard_arc() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &neutral_legs());
        let run = run(machine, |port, clock, line| {
            antennas(&cfg, port, 4.0, 0.0, clock, line)
        });
        run.ok("a direction past the half turn is still a direction");

        let series = run.goal_series(&cfg, JointId::AntennaRight);
        assert!(!series.is_empty(), "the antenna was commanded");
        assert!(
            series.iter().all(|goal| *goal >= -1e-9),
            "the sweep went down through the outboard side: {series:?}"
        );
        let landed = *series.last().expect("a last goal");
        assert!((landed - 4.0).abs() < 0.01, "landed at {landed}");
    }

    /// Staggered antenna clocks reach the machine: the side given the shorter
    /// clock lands first and waits, and the other is still sweeping. Two
    /// inboard arcs on one clock put both tips at the point where they cross at
    /// the same instant, which is how a pair meets tip to tip and stalls.
    #[test]
    fn staggered_antenna_clocks_land_one_side_before_the_other() {
        let mut cfg = datumed_config();
        crate::testutil::wind_down_bus(&mut cfg);
        cfg.motion.antenna_duration_right_s = Some(1.6);
        cfg.motion.antenna_duration_left_s = Some(1.2);
        let cfg = cfg.resolve().expect("staggered antenna clocks resolve");
        assert_eq!(
            cfg.move_durations().antennas,
            [Duration::from_millis(1600), Duration::from_millis(1200)]
        );

        let machine = machine_at(&datumed_config(), &neutral_legs());
        let run = run(machine, |port, clock, line| {
            antennas(&cfg, port, 1.0, -1.0, clock, line)
        });
        run.ok("a staggered antenna command runs");

        let half_count = core::f64::consts::PI / 4096.0;
        let right = run.goal_series(&cfg, JointId::AntennaRight);
        let left = run.goal_series(&cfg, JointId::AntennaLeft);
        // The period each side first commanded its endpoint.
        let arrival = |series: &[f64], joint, target: f64| {
            series
                .iter()
                .position(|goal| (goal - target).abs() <= half_count)
                .unwrap_or_else(|| panic!("{joint} never reached {target}: {series:?}"))
        };
        let right_at = arrival(&right, JointId::AntennaRight, 1.0);
        let left_at = arrival(&left, JointId::AntennaLeft, -1.0);
        assert!(
            left_at < right_at,
            "the left antenna landed at period {left_at}, the right at {right_at}"
        );

        // The landed side then sits on its endpoint while the other travels.
        assert!(
            left[left_at..]
                .iter()
                .all(|goal| (goal + 1.0).abs() <= half_count),
            "the left antenna moved after landing: {left:?}"
        );
        assert!(
            (right[left_at] - 1.0).abs() > half_count,
            "the right antenna was already there at period {left_at}: {right:?}"
        );
    }

    /// A machine as the vendor provisions it: every servo in single-turn
    /// position mode, torque off.
    fn unprovisioned(cfg: &Resolved) -> FakeMachine {
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        for id in [cfg.map.ids()[7], cfg.map.ids()[8]] {
            machine.set(id, reg_for(RegId::OperatingMode), &[3]);
        }
        machine
    }

    /// What each servo's Operating Mode register holds after a run.
    fn modes(cfg: &Resolved, run: &Run) -> Vec<u8> {
        let machine = run.registers.borrow();
        cfg.map
            .ids()
            .iter()
            .map(|id| {
                machine
                    .get(*id, reg_for(RegId::OperatingMode))
                    .map_or(0, |bytes| bytes[0])
            })
            .collect()
    }

    /// `provision` writes the two antennas into extended position mode and
    /// touches nothing else — no goals, no torque, no arm sequence.
    #[test]
    fn provision_writes_the_antennas_into_extended_position_mode() {
        let cfg = resolved();
        let run = run(unprovisioned(&cfg), |port, _, line| {
            provision(&cfg.map, cfg.timing, port, line)
        });
        run.ok("a machine with torque off takes the write");

        assert_eq!(modes(&cfg, &run), vec![3, 3, 3, 3, 3, 3, 3, 4, 4]);
        assert_eq!(
            torque(&cfg, &run),
            vec![0; JointId::COUNT],
            "provisioning enables torque on nothing"
        );
        run.armed_nothing();
        run.commanded_nothing(&cfg);
        assert!(
            run.printed.iter().any(|line| line.contains("3 -> 4")),
            "{:?}",
            run.printed
        );
        assert!(
            run.printed.iter().any(|line| line.contains("selftest")),
            "the run says what to do next: {:?}",
            run.printed
        );
    }

    /// A servo already in the mode is left alone: the command is idempotent, so
    /// an operator can run it twice without a second non-volatile write.
    #[test]
    fn provision_writes_nothing_to_a_machine_already_in_the_mode() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &stow_legs());
        let run = run(machine, |port, _, line| {
            provision(&cfg.map, cfg.timing, port, line)
        });
        run.ok("a provisioned machine provisions to itself");

        assert!(
            !run.log
                .borrow()
                .iter()
                .any(|(_, instruction)| *instruction == INST_WRITE),
            "nothing was written: {:?}",
            run.log.borrow()
        );
        assert_eq!(
            run.printed
                .iter()
                .filter(|line| line.contains("already holds"))
                .count(),
            2,
            "{:?}",
            run.printed
        );
    }

    /// A servo holding torque refuses the whole command, and the other antenna
    /// is not written either: a servo ignores a non-volatile write under torque
    /// and acknowledges it anyway, and half a provisioning is worse than none.
    #[test]
    fn provision_refuses_a_machine_holding_torque_with_nothing_written() {
        let cfg = resolved();
        let mut machine = unprovisioned(&cfg);
        machine.set(cfg.map.ids()[8], reg_for(RegId::TorqueEnable), &[1]);
        let run = run(machine, |port, _, line| {
            provision(&cfg.map, cfg.timing, port, line)
        });

        let error = run.err("a torqued antenna is refused");
        let PumpError::TorqueHeld { id } = error else {
            panic!("expected a torque refusal, got {error}");
        };
        assert_eq!(*id, cfg.map.ids()[8]);
        assert_eq!(
            modes(&cfg, &run),
            vec![3; JointId::COUNT],
            "the first antenna was not written either"
        );
    }

    /// A servo answering as another part is refused before anything is written:
    /// whatever holds that ID is not the servo this project provisions.
    #[test]
    fn provision_refuses_a_servo_that_is_not_the_part_it_should_be() {
        let cfg = resolved();
        let mut machine = unprovisioned(&cfg);
        machine.set(cfg.map.ids()[7], reg_for(RegId::ModelNumber), &[0xB0, 0x04]);
        let run = run(machine, |port, _, line| {
            provision(&cfg.map, cfg.timing, port, line)
        });

        let error = run.err("that is not an antenna servo");
        let PumpError::WrongPart {
            id,
            model,
            expected,
        } = error
        else {
            panic!("expected an identity refusal, got {error}");
        };
        assert_eq!((*id, *model, *expected), (cfg.map.ids()[7], 1200, 1190));
        assert_eq!(modes(&cfg, &run), vec![3; JointId::COUNT]);
    }

    /// A machine holding torque on all nine, with one servo carrying a latched
    /// overload — the state an operator reaches for `reboot` in.
    fn overloaded(cfg: &Resolved) -> FakeMachine {
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        for id in cfg.map.ids() {
            machine.set(id, reg_for(RegId::TorqueEnable), &[1]);
        }
        machine.set(
            cfg.map.ids()[3],
            reg_for(RegId::HardwareErrorStatus),
            &[0x20],
        );
        machine
    }

    /// Which servos a run sent the reboot instruction to, in the order it went
    /// out.
    fn rebooted(run: &Run) -> Vec<u8> {
        run.log
            .borrow()
            .iter()
            .filter(|(_, instruction)| *instruction == INST_REBOOT)
            .map(|(id, _)| *id)
            .collect()
    }

    /// `reboot` restarts every servo in bus order, waits for each to answer
    /// again, and reports the error byte and the position it came back with.
    ///
    /// The torque that comes off is the restart's doing and not a write: an
    /// operator reboots to clear a latch, and a command that quietly wrote
    /// torque off as well would be doing something they did not ask for on a
    /// machine whose head is in their hand.
    #[test]
    fn reboot_restarts_every_servo_and_reports_what_each_came_back_with() {
        let cfg = resolved();
        let run = run(overloaded(&cfg), |port, clock, line| {
            reboot(&cfg.map, cfg.timing, port, None, clock, line)
        });
        run.ok("a machine that answers reboots");

        assert_eq!(rebooted(&run), cfg.map.ids().to_vec());
        assert_eq!(
            torque(&cfg, &run),
            vec![0; JointId::COUNT],
            "a restart drops torque"
        );
        assert!(
            !run.log
                .borrow()
                .iter()
                .any(|(_, instruction)| *instruction == INST_WRITE),
            "nothing was written: {:?}",
            run.log.borrow()
        );
        run.armed_nothing();
        run.commanded_nothing(&cfg);

        // What the operator came for: the byte, per servo, read after the
        // restart rather than assumed to have gone.
        for id in cfg.map.ids() {
            let reading = run
                .printed
                .iter()
                .find(|line| line.contains(&format!("servo {id}: hardware error")))
                .unwrap_or_else(|| panic!("servo {id} was not reported: {:?}", run.printed));
            assert!(reading.contains("counts"), "{reading}");
            assert!(reading.contains("deg"), "{reading}");
        }
        assert!(
            run.printed
                .iter()
                .any(|line| line.contains("0x20") && line.contains("still latched")),
            "a byte that survived the restart is reported as it read: {:?}",
            run.printed
        );
        // The torque is measured, not assumed: the closing line claims every
        // servo that came back is limp, and this is what makes that a reading.
        for id in cfg.map.ids() {
            let reading = run
                .printed
                .iter()
                .find(|line| line.contains(&format!("servo {id}: hardware error")))
                .unwrap_or_else(|| panic!("servo {id} was not reported: {:?}", run.printed));
            assert!(reading.contains("limp"), "{reading}");
        }
    }

    /// A servo the reboot instruction never reached is caught by its torque,
    /// and the command fails rather than reporting a restart that did not
    /// happen.
    ///
    /// A lost or corrupted frame leaves a servo answering pings exactly as one
    /// that restarted does, so the poll cannot tell them apart. What can is the
    /// torque a restart clears — and an operator scripting a recovery around
    /// this command needs the exit code to mean what it says.
    #[test]
    fn a_servo_that_never_took_its_reboot_is_caught_by_the_torque_it_kept() {
        let cfg = resolved();
        let deaf = cfg.map.ids()[2];
        let mut machine = overloaded(&cfg);
        machine.deaf_to_reboot.push(deaf);
        let run = run(machine, |port, clock, line| {
            reboot(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a servo still holding torque did not restart");
        let PumpError::NotRestarted { id } = error else {
            panic!("expected a servo that did not restart, got {error}");
        };
        assert_eq!(*id, deaf);
        assert!(
            run.printed
                .iter()
                .any(|line| line.contains(&format!("servo {deaf}: reboot sent, unacknowledged"))),
            "{:?}",
            run.printed
        );
        assert!(
            run.printed
                .iter()
                .any(|line| line.contains(&format!("servo {deaf}"))
                    && line.contains("STILL HOLDING TORQUE")),
            "{:?}",
            run.printed
        );
        // The eight that did restart are still read, and the run does not claim
        // the machine is limp.
        for id in cfg.map.ids().iter().filter(|id| **id != deaf) {
            assert!(
                run.printed
                    .iter()
                    .any(|line| line.contains(&format!("servo {id}: hardware error"))
                        && line.contains("limp")),
                "servo {id} went unreported: {:?}",
                run.printed
            );
        }
        assert!(
            !run.printed.iter().any(|line| line.contains("rebooted;")),
            "{:?}",
            run.printed
        );
    }

    /// A servo that acknowledged nothing and had no torque to drop is not a
    /// restart anybody observed, and the command says so instead of passing.
    ///
    /// This is the command's own primary scenario: a latched overload is in the
    /// shutdown mask, so the servo has already switched its own torque off, and
    /// every fault response de-torques the rest. On that machine the torque
    /// check cannot fire — a servo that never took the instruction is limp
    /// exactly like one that restarted — and the acknowledgement is the only
    /// thing left that distinguishes them.
    #[test]
    fn a_reboot_unacknowledged_by_a_limp_servo_is_not_a_confirmed_restart() {
        let cfg = resolved();
        // The servo carrying the latch, in the state a latch leaves it: shut
        // down, holding nothing.
        let deaf = cfg.map.ids()[3];
        let mut machine = overloaded(&cfg);
        machine.set(deaf, reg_for(RegId::TorqueEnable), &[0]);
        machine.deaf_to_reboot.push(deaf);
        let run = run(machine, |port, clock, line| {
            reboot(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("an unobserved restart is not a restart");
        let PumpError::RestartUnconfirmed { id, .. } = error else {
            panic!("expected an unconfirmed restart, got {error}");
        };
        assert_eq!(*id, deaf);
        assert!(
            run.printed
                .iter()
                .any(|line| line.contains(&format!("servo {deaf}"))
                    && line.contains("nothing here says it restarted")),
            "{:?}",
            run.printed
        );
        // The latch it was rebooted for is still reported as it reads, and the
        // command does not close by calling the machine restarted.
        assert!(
            run.printed
                .iter()
                .any(|line| line.contains("0x20") && line.contains("still latched")),
            "{:?}",
            run.printed
        );
        assert!(
            !run.printed.iter().any(|line| line.contains("rebooted;")),
            "{:?}",
            run.printed
        );
        // The eight that did acknowledge came back limp and pass.
        for id in cfg.map.ids().iter().filter(|id| **id != deaf) {
            assert!(
                run.printed
                    .iter()
                    .any(|line| line.contains(&format!("servo {id}: hardware error"))
                        && line.contains("limp")),
                "servo {id} went unreported: {:?}",
                run.printed
            );
        }
    }

    /// A servo whose only latched bit is input voltage is reported as that and
    /// not as a fault, and one that answers its ping but fails a register read
    /// is named without stopping the other eight being read.
    ///
    /// The voltage rendering is what tells an operator the reboot cleared what
    /// they rebooted for; the read-error arm is the path a bus failure takes in
    /// the middle of a report.
    #[test]
    fn a_reboot_reports_a_voltage_only_byte_and_a_servo_that_will_not_read() {
        let cfg = resolved();
        let voltage = cfg.map.ids()[1];
        let unreadable = cfg.map.ids()[5];
        let mut machine = overloaded(&cfg);
        machine.set(
            voltage,
            reg_for(RegId::HardwareErrorStatus),
            &[dxl_proto::conv::HW_INPUT_VOLTAGE],
        );
        // Answers its ping, answers nothing about where it is standing.
        machine
            .mute
            .insert((unreadable, reg_for(RegId::PresentPosition).addr), u32::MAX);
        let run = run(machine, |port, clock, line| {
            reboot(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a servo that will not read is not a clean reboot");
        let PumpError::Bus { id, .. } = error else {
            panic!("expected a bus failure, got {error}");
        };
        assert_eq!(*id, unreadable);
        assert!(
            run.printed
                .iter()
                .any(|line| line.contains(&format!("servo {voltage}"))
                    && line.contains("input voltage only")),
            "{:?}",
            run.printed
        );
        assert!(
            run.printed
                .iter()
                .any(|line| line.contains(&format!("servo {unreadable}"))
                    && line.contains("reads back as")),
            "{:?}",
            run.printed
        );
        // The one that would not read did not take the other eight with it.
        for id in cfg.map.ids().iter().filter(|id| **id != unreadable) {
            assert!(
                run.printed
                    .iter()
                    .any(|line| line.contains(&format!("servo {id}: hardware error"))),
                "servo {id} went unreported: {:?}",
                run.printed
            );
        }
    }

    /// The command says what a reboot costs before it sends one: torque goes,
    /// the head settles, and nothing about where the machine is standing stops
    /// it.
    #[test]
    fn reboot_says_the_head_will_settle_before_it_sends_anything() {
        let cfg = resolved();
        let run = run(overloaded(&cfg), |port, clock, line| {
            reboot(&cfg.map, cfg.timing, port, None, clock, line)
        });
        run.ok("a machine that answers reboots");

        let warning = &run.printed[0];
        for word in ["Torque Enable", "settles", "weight"] {
            assert!(warning.contains(word), "no {word}: {warning}");
        }
    }

    /// A named servo is the only one restarted; the other eight are still
    /// holding when it is over.
    #[test]
    fn a_reboot_of_one_servo_leaves_the_other_eight_holding() {
        let cfg = resolved();
        let one = cfg.map.ids()[4];
        let run = run(overloaded(&cfg), |port, clock, line| {
            reboot(&cfg.map, cfg.timing, port, Some(one), clock, line)
        });
        run.ok("one servo reboots");

        assert_eq!(rebooted(&run), vec![one]);
        let held: Vec<u8> = torque(&cfg, &run);
        for (row, holding) in held.iter().enumerate() {
            let expected = u8::from(cfg.map.ids()[row] != one);
            assert_eq!(*holding, expected, "row {row}");
        }
    }

    /// A servo ID the roster does not carry is refused by name, and nothing
    /// goes out to whatever holds it.
    #[test]
    fn a_reboot_of_a_servo_off_the_roster_sends_nothing() {
        let cfg = resolved();
        let stranger = 99;
        assert!(!cfg.map.ids().contains(&stranger));
        let run = run(overloaded(&cfg), |port, clock, line| {
            reboot(&cfg.map, cfg.timing, port, Some(stranger), clock, line)
        });

        let error = run.err("that servo is not on this machine");
        let PumpError::OffRoster { id, roster } = error else {
            panic!("expected a roster refusal, got {error}");
        };
        assert_eq!((*id, *roster), (stranger, cfg.map.ids()));
        assert!(run.log.borrow().is_empty(), "{:?}", run.log.borrow());
        assert_eq!(
            torque(&cfg, &run),
            vec![1; JointId::COUNT],
            "a refused reboot left the machine exactly as it was"
        );
    }

    /// A servo that takes its reboot and never answers again is named, the run
    /// fails, and the eight that did come back are still read and reported.
    ///
    /// The command has nothing to release and nothing to catch: no torque was
    /// ever enabled on this path, and the eight that answered are limp because
    /// they restarted. What is left is the report and a non-zero exit.
    #[test]
    fn a_servo_that_never_comes_back_is_named_and_the_rest_still_reported() {
        let cfg = resolved();
        let lost = cfg.map.ids()[6];
        let mut machine = overloaded(&cfg);
        machine.gone_on_reboot.push(lost);
        let run = run(machine, |port, clock, line| {
            reboot(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a servo that never answered is not a success");
        let PumpError::NotBack { id, polls, .. } = error else {
            panic!("expected a servo that did not come back, got {error}");
        };
        assert_eq!((*id, *polls), (lost, BOOT_POLLS));
        assert!(
            run.printed
                .iter()
                .any(|line| line.contains(&format!("servo {lost}: NO ANSWER"))),
            "{:?}",
            run.printed
        );
        for id in cfg.map.ids().iter().filter(|id| **id != lost) {
            assert!(
                run.printed
                    .iter()
                    .any(|line| line.contains(&format!("servo {id}: hardware error"))),
                "servo {id} went unreported: {:?}",
                run.printed
            );
        }
    }

    /// Every command that moves commissions first, so a machine that cannot be
    /// commissioned moves nothing — and says so as a commissioning refusal
    /// rather than as a failed move.
    #[test]
    fn a_machine_that_will_not_arm_moves_nothing() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        machine.silent = vec![13];
        let run = run(machine, |port, clock, line| up(&cfg, port, clock, line));

        let error = run.err("a silent servo does not arm");
        assert!(
            matches!(error, PumpError::Sequence(SeqError::AbsentServos { .. })),
            "{error}"
        );
        assert!(run.addressed.borrow().is_empty(), "nothing was commanded");
    }

    /// The milestone sequence end to end: armed once, six moves chained through
    /// the one tick path, and released at stow with the head where the next
    /// boot expects to find it.
    #[test]
    fn the_demo_runs_end_to_end_and_ends_released_at_stow() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &stow_legs());
        let run = run(machine, |port, clock, line| demo(&cfg, port, clock, line));
        run.ok("the fixture machine runs the demo");

        within_a_count(
            &goals(&cfg, &run),
            &cfg.disarm.stow_targets,
            "the goals the demo leaves",
        );
        assert_eq!(
            torque(&cfg, &run),
            vec![0; JointId::COUNT],
            "the demo ends released"
        );
        // The six steps are announced in order, so a run that skipped one says
        // so rather than being read off the register file.
        let steps: Vec<&String> = run
            .printed
            .iter()
            .filter(|line| line.starts_with("demo "))
            .collect();
        assert_eq!(steps.len(), 6, "{steps:?}");

        // The step lines are printed before their loops and the final register
        // state is the same whether the sweeps ran or not, so neither says the
        // demo swept anything. What it commanded on the way does: eight moves
        // and one hold, each reporting what it cost, and both sweeps reaching
        // their amplitudes either side of square.
        let summaries = run
            .printed
            .iter()
            .filter(|line| line.contains("period(s)"))
            .count();
        assert_eq!(summaries, 9, "eight moves and a hold: {:?}", run.printed);
        for angle in [DEMO_YAW_DEG.to_radians(), -DEMO_YAW_DEG.to_radians()] {
            assert!(
                run.commanded_angle(&cfg, JointId::BodyYaw, angle),
                "the body never reached {} deg",
                angle.to_degrees()
            );
        }
        for angle in [DEMO_ANTENNA_RAD, -DEMO_ANTENNA_RAD] {
            assert!(
                run.commanded_angle(&cfg, JointId::AntennaRight, angle),
                "the right antenna never reached {angle} rad"
            );
        }
    }

    /// A fixture and the two handles the typestate tests watch it through.
    ///
    /// `run` is about what a whole command left behind; these are about what
    /// happens *between* one engage and the next, so they drive the typestate
    /// directly and read the wire as they go.
    struct Rig {
        port: Spy,
        registers: Rc<RefCell<FakeMachine>>,
        log: Rc<RefCell<Vec<(u8, u8)>>>,
    }

    fn rig(machine: FakeMachine) -> Rig {
        let port = Spy::new(machine);
        let registers = port.machine();
        let log = port.log();
        Rig {
            port,
            registers,
            log,
        }
    }

    /// What the nine torque flags hold, in bus order — the one reading that
    /// says whether a hand can go on the head.
    fn torque_bits(cfg: &Resolved, registers: &Rc<RefCell<FakeMachine>>) -> Vec<u8> {
        cfg.map
            .ids()
            .iter()
            .map(|id| {
                registers
                    .borrow()
                    .get(*id, reg_for(RegId::TorqueEnable))
                    .map_or(0, |bytes| bytes[0])
            })
            .collect()
    }

    /// Commissioning is paid once and every engage after it is free of the
    /// ceremony.
    ///
    /// The whole point of the split: a process that raises the head, puts it
    /// down, and raises it again pings nine servos once, and the two lifts
    /// either side of the release cost nothing but the pins, the enables and
    /// the read-back.
    #[test]
    fn one_commissioning_serves_every_engage_over_the_same_bus() {
        let cfg = resolved();
        let rig = rig(machine_at(&datumed_config(), &stow_legs()));
        let log = rig.log;
        let mut clock = TestClock::default();
        let mut printed: Vec<String> = Vec::new();

        let mut machine = commission(&cfg, rig.port, &mut clock, &mut |line| {
            printed.push(line.to_string());
        })
        .expect("the fixture commissions");

        for pass in 0..2 {
            let mut engaged = machine
                .engage(&mut clock, &mut |line| printed.push(line.to_string()))
                .expect("the fixture engages");
            let target = if pass == 0 {
                neutral_targets()
            } else {
                stow_pose_targets()
            };
            engaged
                .move_to(target, cfg.move_durations(), &mut clock, &mut |line| {
                    printed.push(line.to_string());
                })
                .expect("the move runs");
            engaged
                .disengage(&mut clock, &mut |line| printed.push(line.to_string()))
                .expect("the release runs");
        }

        let pings = log
            .borrow()
            .iter()
            .filter(|(_, instruction)| *instruction == INST_PING)
            .count();
        assert_eq!(
            pings,
            JointId::COUNT,
            "the roster is pinged once, by the one commissioning"
        );
        let count = |prefix: &str| {
            printed
                .iter()
                .filter(|line| line.starts_with(prefix))
                .count()
        };
        assert_eq!(count("models "), 1, "{printed:?}");
        assert_eq!(count("found "), 2, "{printed:?}");
    }

    /// A head somebody turned while the machine lay limp is where the next
    /// engage pins it.
    ///
    /// The resting watch's whole job. Nothing about the pose is refused — the
    /// machine is where it is — and the poll is what makes the difference
    /// between planning from that and planning from a reading taken before a
    /// hand touched it.
    #[test]
    fn a_head_turned_while_resting_is_where_the_next_engage_pins_it() {
        let cfg = resolved();
        let rig = rig(machine_at(&datumed_config(), &stow_legs()));
        let registers = rig.registers;
        let mut clock = TestClock::default();
        let mut printed: Vec<String> = Vec::new();

        let mut machine = commission(&cfg, rig.port, &mut clock, &mut |line| {
            printed.push(line.to_string());
        })
        .expect("the fixture commissions");
        assert!(machine.posture().present.body_yaw.abs() < 1e-9);

        let turned = 12.0_f64.to_radians();
        let counts = cfg.map.goal_counts(0, turned).expect("a yaw angle places");
        registers.borrow_mut().set(
            cfg.map.ids()[0],
            reg_for(RegId::PresentPosition),
            &counts.to_le_bytes(),
        );

        let posture = machine
            .poll(PollCadence::Positions, &mut clock, &mut |line| {
                printed.push(line.to_string());
            })
            .expect("the sweep reads");
        let half_count = core::f64::consts::PI / 4096.0;
        assert!(
            (posture.present.body_yaw - turned).abs() <= half_count,
            "the sweep found the body at {} deg",
            posture.present.body_yaw.to_degrees()
        );

        let engaged = machine
            .engage(&mut clock, &mut |line| printed.push(line.to_string()))
            .expect("a turned body is a measurement, not a refusal");
        assert!(
            (engaged.targets().body_yaw - turned).abs() <= half_count,
            "the first move would start from {} deg",
            engaged.targets().body_yaw.to_degrees()
        );
    }

    /// A sweep that did not complete measured nothing, so the posture in hand
    /// stops describing the machine — and the next engage looks again before it
    /// pins anything.
    ///
    /// A watch that keeps sweeping over a flaky bus is the whole point: the
    /// machine is limp and safe, and a failed sweep costs it its picture rather
    /// than its safety. What must not survive the outage is the *belief* that
    /// the picture is current, because the engage's pin sweep writes that
    /// picture into nine goal registers immediately before the enables.
    #[test]
    fn a_sweep_that_failed_costs_the_posture_its_freshness() {
        let cfg = resolved();
        let rig = rig(machine_at(&datumed_config(), &stow_legs()));
        let log = rig.log;
        let port = Flaky::new(rig.port);
        let down = port.switch();
        let mut clock = TestClock::default();
        let mut printed: Vec<String> = Vec::new();

        let mut machine = commission(&cfg, port, &mut clock, &mut |line| {
            printed.push(line.to_string());
        })
        .expect("the fixture commissions");
        assert!(machine.fresh(), "commissioning takes its own sweep");

        down.set(true);
        machine
            .poll(PollCadence::Positions, &mut clock, &mut |line| {
                printed.push(line.to_string());
            })
            .expect_err("a sweep over an adapter that went away does not complete");
        assert!(
            !machine.fresh(),
            "a sweep that failed left the posture describing a machine nobody has looked at"
        );

        // The bus comes back, and the engage pays for the outage in reads
        // rather than in a slam.
        down.set(false);
        let before = log.borrow().len();
        machine
            .engage(&mut clock, &mut |line| printed.push(line.to_string()))
            .expect("the fixture engages once the adapter is back");
        assert_eq!(
            log.borrow().len() - before,
            JointId::COUNT + 5 * JointId::COUNT,
            "a sweep of nine reads, then the engage's own twenty-seven"
        );
    }

    /// An engage after a release that measured nothing measures first.
    ///
    /// The pin sweep writes the posture's angles into nine goal registers
    /// *before* the enables, and it exists to stop a firmware that keeps a
    /// stale goal from slamming the joint at torque-on. Pinning at a pose the
    /// head has since fallen out of is that slam, delivered by the very sweep
    /// meant to prevent it — and the release that leaves the posture stale is
    /// the fault release, which is the one that leaves the head high up.
    #[test]
    fn an_engage_after_a_release_that_measured_nothing_looks_first() {
        let cfg = resolved();
        let rig = rig(machine_at(&datumed_config(), &stow_legs()));
        let registers = rig.registers;
        let log = rig.log;
        let mut clock = TestClock::default();
        let mut printed: Vec<String> = Vec::new();

        let mut machine = commission(&cfg, rig.port, &mut clock, &mut |line| {
            printed.push(line.to_string());
        })
        .expect("the fixture commissions");
        assert!(machine.fresh(), "commissioning takes its own sweep");

        let engaged = machine
            .engage(&mut clock, &mut |line| printed.push(line.to_string()))
            .expect("the fixture engages");
        engaged
            .disengage_now(&mut clock, &mut |line| printed.push(line.to_string()))
            .expect("nothing gates a release");
        assert!(
            !machine.fresh(),
            "a release that looked at nothing knows nothing about where the head is"
        );

        // The head settles somewhere else while limp, which is what the fault
        // release leaves it doing.
        let fallen = 12.0_f64.to_radians();
        let counts = cfg.map.goal_counts(0, fallen).expect("a yaw angle places");
        registers.borrow_mut().set(
            cfg.map.ids()[0],
            reg_for(RegId::PresentPosition),
            &counts.to_le_bytes(),
        );

        let before = log.borrow().len();
        let engaged = machine
            .engage(&mut clock, &mut |line| printed.push(line.to_string()))
            .expect("the fixture engages again");
        let half_count = core::f64::consts::PI / 4096.0;
        assert!(
            (engaged.targets().body_yaw - fallen).abs() <= half_count,
            "the engage pinned {} deg, where the head was before the release",
            engaged.targets().body_yaw.to_degrees(),
        );
        // Nine reads for the sweep, then the engage's own twenty-seven — of
        // which the eighteen verified writes are two frames each on the wire.
        let engage_frames = 5 * JointId::COUNT;
        assert_eq!(
            log.borrow().len() - before,
            JointId::COUNT + engage_frames,
            "a sweep of nine reads, then the engage"
        );

        // And the orderly release costs the next engage nothing: it measured
        // all nine on its way out, so there is nothing to re-read.
        engaged
            .disengage(&mut clock, &mut |line| printed.push(line.to_string()))
            .expect("nothing gates a release");
        assert!(machine.fresh(), "the orderly release measured all nine");
        let before = log.borrow().len();
        machine
            .engage(&mut clock, &mut |line| printed.push(line.to_string()))
            .expect("the fixture engages again");
        assert_eq!(
            log.borrow().len() - before,
            engage_frames,
            "the engage alone, with no sweep in front of it"
        );
    }

    /// Nothing between the enable writes and the caller owning an `Engaged`
    /// may return without taking the machine back to limp.
    ///
    /// Configuration refuses a zero control rate, so this is unreachable by any
    /// route through the binary — which is the point: it is the only fallible
    /// step in that window, and a machine left torqued with no loop driving it
    /// is the state the doctrine names as this platform's only pinch hazard.
    /// The next fallible step added there should find this test already
    /// standing.
    #[test]
    fn an_engage_that_cannot_start_its_loop_releases_torque() {
        let mut cfg = resolved();
        cfg.tick_hz = 0;
        let rig = rig(machine_at(&datumed_config(), &stow_legs()));
        let registers = rig.registers;
        let mut clock = TestClock::default();
        let mut printed: Vec<String> = Vec::new();

        let mut machine = commission(&cfg, rig.port, &mut clock, &mut |line| {
            printed.push(line.to_string());
        })
        .expect("the fixture commissions");
        let error = machine
            .engage(&mut clock, &mut |line| printed.push(line.to_string()))
            .err()
            .expect("a loop at zero hertz does not start");

        assert!(
            matches!(error, PumpError::Rate { tick_hz: 0, .. }),
            "{error}"
        );
        assert!(
            printed
                .iter()
                .any(|line| line.contains("releasing immediately")),
            "{printed:?}"
        );
        assert_eq!(
            torque_bits(&cfg, &registers),
            vec![0; JointId::COUNT],
            "nine servos were left holding with no loop driving them: {printed:?}"
        );
    }

    /// The fault release waits for nothing and measures nothing: it is the
    /// orderly release minus the settle and minus the nine reads, and those
    /// nine transactions are exactly the difference.
    #[test]
    fn the_fault_release_is_the_orderly_one_without_the_looking() {
        let cfg = resolved();

        let mut transactions = Vec::new();
        let mut dwells = Vec::new();
        for immediate in [false, true] {
            let rig = rig(machine_at(&datumed_config(), &stow_legs()));
            let log = rig.log;
            let mut clock = TestClock::default();
            let mut printed: Vec<String> = Vec::new();
            let mut machine = commission(&cfg, rig.port, &mut clock, &mut |line| {
                printed.push(line.to_string());
            })
            .expect("the fixture commissions");
            let engaged = machine
                .engage(&mut clock, &mut |line| printed.push(line.to_string()))
                .expect("the fixture engages");

            let before = log.borrow().len();
            let waits = clock.waits.len();
            let release = &mut |line: &str| printed.push(line.to_string());
            let summary = if immediate {
                engaged.disengage_now(&mut clock, release)
            } else {
                engaged.disengage(&mut clock, release)
            }
            .expect("nothing gates a release");

            assert!(summary.all_released(), "every servo came off");
            transactions.push(log.borrow().len() - before);
            dwells.push(clock.waits.len() - waits);
        }

        assert_eq!(
            transactions[1] + JointId::COUNT,
            transactions[0],
            "the difference is the nine positions the orderly form reads: {transactions:?}"
        );
        assert_eq!(dwells, vec![1, 0], "a fault waits out no settle");
    }

    /// An orderly release that does not finish takes the immediate one.
    ///
    /// The doctrine's MRM-A → MRM-B fall-through, and the state it exists for:
    /// a walk that stopped part way through has left some servos holding with
    /// no sequence driving them. The error the caller gets back is the one that
    /// made the release necessary; what it must never get back is that error
    /// over a machine still holding the head up.
    #[test]
    fn an_orderly_release_that_stops_short_falls_through_to_the_immediate_one() {
        let cfg = resolved();
        let rig = rig(machine_at(&datumed_config(), &stow_legs()));
        let registers = rig.registers;
        let log = rig.log;
        let mut clock = TestClock::default();
        let mut printed: Vec<String> = Vec::new();

        let mut machine = commission(&cfg, rig.port, &mut clock, &mut |line| {
            printed.push(line.to_string());
        })
        .expect("the fixture commissions");
        // Torque on, and the engagement let go without a release: the machine an
        // orderly release that stopped part way through leaves behind.
        {
            machine
                .engage(&mut clock, &mut |line| printed.push(line.to_string()))
                .expect("the fixture engages");
        }
        assert_eq!(torque_bits(&cfg, &registers), vec![1; JointId::COUNT]);

        let before = log.borrow().len();
        let stopped = PumpError::Runaway { budget: 10_000 };
        let outcome =
            fall_through_to_immediate(&mut machine, Err(stopped), &mut clock, &mut |line| {
                printed.push(line.to_string())
            });

        assert!(
            matches!(
                outcome.expect_err("the failure that made the release necessary"),
                PumpError::Runaway { .. }
            ),
            "the original error comes back, not the release's"
        );
        assert_eq!(
            torque_bits(&cfg, &registers),
            vec![0; JointId::COUNT],
            "{printed:?}"
        );
        assert_eq!(
            log.borrow().len() - before,
            2 * JointId::COUNT,
            "nine verified torque-off writes and nothing else"
        );
        assert!(
            printed
                .iter()
                .any(|line| line.contains("the orderly release did not finish")),
            "{printed:?}"
        );
        assert!(
            clock.waits.is_empty(),
            "the fall-through waits out no settle"
        );
    }

    /// An unacknowledged torque-off is not a fault, and does not buy a second
    /// walk.
    ///
    /// The other side of the same rule. Every release was written; a servo that
    /// did not answer its own is what the report says, and nine more writes
    /// would say no more. Widening the fault classifier to include it would
    /// double every ragged release silently.
    #[test]
    fn an_unacknowledged_release_is_not_walked_a_second_time() {
        let cfg = resolved();
        let rig = rig(machine_at(&datumed_config(), &stow_legs()));
        let registers = rig.registers;
        let log = rig.log;
        let mut clock = TestClock::default();
        let mut printed: Vec<String> = Vec::new();

        let mut machine = commission(&cfg, rig.port, &mut clock, &mut |line| {
            printed.push(line.to_string());
        })
        .expect("the fixture commissions");
        let engaged = machine
            .engage(&mut clock, &mut |line| printed.push(line.to_string()))
            .expect("the fixture engages");

        // The third servo's torque-off read-back answers a byte too wide, so
        // its release goes unacknowledged while all nine are still written.
        registers
            .borrow_mut()
            .verbose
            .push((cfg.map.ids()[2], reg_for(RegId::TorqueEnable).addr));
        let before = log.borrow().len();
        let outcome = engaged.disengage(&mut clock, &mut |line| printed.push(line.to_string()));

        let error = outcome.expect_err("an unacknowledged release is reported");
        assert!(
            matches!(error, PumpError::TorqueOffUnacked { id } if id == cfg.map.ids()[2]),
            "{error}"
        );
        // The release ran and every servo was asked, so the class is the
        // degenerate one: park and alert over a minimum risk condition nobody
        // could confirm, and no second walk to write the same nine writes again.
        assert_eq!(
            error.class(Phase::UnderTorque),
            ErrorClass::ImmediateAllTorqueOffToPark
        );
        assert_eq!(
            error.fault(Phase::UnderTorque),
            Some(Fault::TorqueOffUnconfirmed {
                id: cfg.map.ids()[2]
            })
        );
        assert!(
            !printed
                .iter()
                .any(|line| line.contains("the orderly release did not finish")),
            "no second walk was announced: {printed:?}"
        );
        // The orderly walk and nothing after it: nine reads, then nine verified
        // writes, of which the third's read-back is the wide reply.
        assert_eq!(
            log.borrow().len() - before,
            3 * JointId::COUNT,
            "{printed:?}"
        );
        assert_eq!(torque_bits(&cfg, &registers), vec![0; JointId::COUNT]);
    }

    /// The last angle a run commanded `row`'s servo to, if it ever commanded
    /// it one.
    ///
    /// The registers only hold where a machine ended, and a servo released part
    /// way through stops taking goals at all — so the frames are the only
    /// record of which joints a stow was actually driven on.
    fn last_commanded(
        cfg: &Resolved,
        commanded: &Rc<RefCell<Vec<GroupedWrite>>>,
        row: usize,
    ) -> Option<f64> {
        let id = cfg.map.ids()[row];
        let goal = reg_for(RegId::GoalPosition).addr;
        commanded
            .borrow()
            .iter()
            .filter(|write| write.addr == goal)
            .filter_map(|write| {
                let (_, bytes) = write.entries.iter().find(|(servo, _)| *servo == id)?;
                let counts =
                    i32::from_le_bytes(bytes.as_slice().try_into().expect("a goal is four"));
                cfg.map.present_rad(row, counts).ok()
            })
            .next_back()
    }

    /// Take a fixture machine to holding torque and run `body` against the
    /// engagement, with the registers reachable while it runs.
    ///
    /// What the whole-command harness cannot do: a servo that latches an error
    /// *after* the gates looked at it is the only way a head servo drops out
    /// mid-move, and the gates refuse a machine that was already flagging.
    fn engaged_run<F>(cfg: &Resolved, machine: FakeMachine, body: F) -> Engagement
    where
        F: FnOnce(
            Engaged<'_, '_, Spy>,
            &Rc<RefCell<FakeMachine>>,
            &mut dyn Clock,
            &mut dyn FnMut(&str),
        ) -> Result<(), PumpError>,
    {
        let rig = rig(machine);
        let registers = Rc::clone(&rig.registers);
        let commanded = rig.port.commanded();
        let mut clock = TestClock::default();
        let mut printed: Vec<String> = Vec::new();
        let mut say = |line: &str| printed.push(line.to_string());

        let mut commissioned =
            commission(cfg, rig.port, &mut clock, &mut say).expect("the fixture commissions");
        let engaged = commissioned
            .engage(&mut clock, &mut say)
            .expect("the fixture engages");
        let outcome = body(engaged, &registers, &mut clock, &mut say);
        drop(commissioned);
        Engagement {
            outcome,
            registers,
            commanded,
            printed,
        }
    }

    /// What an engagement left behind.
    struct Engagement {
        outcome: Result<(), PumpError>,
        registers: Rc<RefCell<FakeMachine>>,
        commanded: Rc<RefCell<Vec<GroupedWrite>>>,
        printed: Vec<String>,
    }

    /// A head servo dropping out mid-move stows on the eight that still
    /// command, releases everything, and leaves the machine for an operator.
    ///
    /// The semi-controlled descent. The servo that flagged is released on the
    /// spot and never commanded again — nothing reboots it, because a reboot of
    /// a servo holding this head drops it — and the head comes down on what is
    /// left rather than falling out of the sky.
    #[test]
    fn a_head_servo_fault_stows_on_what_still_commands_and_parks() {
        let cfg = resolved();
        let dropped = 4;
        let held = 3;
        let ended = engaged_run(
            &cfg,
            machine_at(&datumed_config(), &stow_legs()),
            |mut engaged, registers, clock, line| {
                // A leg latching an overload once the machine is holding: the
                // gates looked at a healthy machine, which is the only way this
                // reaches a move at all.
                registers.borrow_mut().set(
                    cfg.map.ids()[dropped],
                    reg_for(RegId::HardwareErrorStatus),
                    &[0x20],
                );
                let outcome = engaged.move_to(neutral_targets(), cfg.up_durations(), clock, line);
                settle(engaged, outcome, clock, line)
            },
        );

        let error = ended
            .outcome
            .expect_err("a servo dropping out ends the move");
        assert!(
            matches!(error, PumpError::Fault(Fault::HeadServoFault { .. })),
            "{error}"
        );
        assert_eq!(
            error.class(Phase::UnderTorque),
            ErrorClass::MaskedSlowStowToPark
        );
        assert_eq!(
            torque_bits(&cfg, &ended.registers),
            vec![0; JointId::COUNT],
            "{:?}",
            ended.printed
        );

        // The stow ran on the eight that still commanded, and the ninth was
        // never given another goal.
        let stow = cfg.disarm.stow_targets.legs;
        let half_count = core::f64::consts::PI / 4096.0;
        let arrived = |row: usize| {
            last_commanded(&cfg, &ended.commanded, row)
                .is_some_and(|goal| (goal - stow[row - 1]).abs() <= half_count)
        };
        assert!(
            arrived(held),
            "the stow was commanded on the legs that hold"
        );
        assert!(
            !arrived(dropped),
            "a released servo took another goal: {:?}",
            ended.printed
        );
        for said in ["stowing under control", "stowed", "look at the machine"] {
            assert!(
                ended.printed.iter().any(|line| line.contains(said)),
                "no {said:?} line: {:?}",
                ended.printed
            );
        }
    }

    /// A second servo dropping out mid-stow expands the maneuver rather than
    /// ending it.
    ///
    /// The quick-succession case the bench saw. Each servo that goes is
    /// released and dropped from the stow, which carries on with what is left —
    /// the alternative is going limp over the second of nine failures, on a
    /// machine still perfectly able to put the head down.
    #[test]
    fn a_second_servo_dropping_out_expands_the_stow() {
        let cfg = resolved();
        let ended = engaged_run(
            &cfg,
            machine_at(&datumed_config(), &stow_legs()),
            |mut engaged, registers, clock, line| {
                // Two legs flagging together. The health sweep names the first
                // in bus order, so the second is found by the stow's own polls.
                for row in [4, 5] {
                    registers.borrow_mut().set(
                        cfg.map.ids()[row],
                        reg_for(RegId::HardwareErrorStatus),
                        &[0x20],
                    );
                }
                let outcome = engaged.move_to(neutral_targets(), cfg.up_durations(), clock, line);
                settle(engaged, outcome, clock, line)
            },
        );

        ended
            .outcome
            .expect_err("a servo dropping out ends the move");
        let carried_on = ended
            .printed
            .iter()
            .filter(|line| line.contains("the stow carries on without it"))
            .count();
        assert_eq!(carried_on, 1, "{:?}", ended.printed);
        assert_eq!(
            ended
                .printed
                .iter()
                .filter(|line| line.contains("stowing under control"))
                .count(),
            2,
            "the stow is re-commanded on what is left: {:?}",
            ended.printed
        );
        assert_eq!(
            torque_bits(&cfg, &ended.registers),
            vec![0; JointId::COUNT],
            "{:?}",
            ended.printed
        );
        assert!(
            ended
                .printed
                .iter()
                .any(|line| line.contains("look at the machine")),
            "{:?}",
            ended.printed
        );
    }

    /// A clock that runs on while it is being read: every look at it costs
    /// `per_look`.
    ///
    /// A maneuver bounded by a wall clock needs a wall clock that can outrun
    /// it, and the periods a run turns are not where the time goes — the pump
    /// counts its budgets in periods, and a run that faults on its first one
    /// waits for nothing at all. This is the machine whose recovery is taking
    /// far longer than the work it is doing, which is the only condition the
    /// deadline exists for.
    struct RunningClock {
        now: std::cell::Cell<Duration>,
        per_look: Duration,
    }

    impl Clock for RunningClock {
        fn now(&self) -> Duration {
            let now = self.now.get();
            self.now.set(now.saturating_add(self.per_look));
            now
        }

        fn sleep_until(&mut self, until: Duration) {
            self.now.set(until.max(self.now.get()));
        }
    }

    /// A re-commanded stow is asked for what is left of the clock the maneuver
    /// began with, and a clock with room to spare is handed on untouched.
    #[test]
    fn an_expanded_stow_is_clamped_to_the_clock_that_is_left() {
        let stow = MoveDurations {
            head: Duration::from_millis(2000),
            antennas: [Duration::from_millis(1500), Duration::from_millis(700)],
        };
        let left = Duration::from_millis(900);

        assert_eq!(
            within(stow, left),
            MoveDurations {
                head: left,
                antennas: [left, Duration::from_millis(700)],
            },
            "every clock over what is left is cut to it, and the short one is not"
        );
        assert_eq!(
            within(stow, Duration::from_secs(30)),
            stow,
            "a maneuver with time in hand asks for its own clocks"
        );
    }

    /// A wind-down whose clock runs out releases torque rather than starting
    /// another stow.
    ///
    /// The deadline is the only thing bounding an expanding maneuver: a machine
    /// shedding a servo per attempt would otherwise be re-commanded a
    /// full-length stow for as many attempts as it has servos, torque on
    /// throughout, and never reach the release that ends it.
    #[test]
    fn a_wind_down_that_runs_out_of_clock_releases_torque() {
        let cfg = resolved();
        let ended = engaged_run(
            &cfg,
            machine_at(&datumed_config(), &stow_legs()),
            |mut engaged, registers, _clock, line| {
                for row in [4, 5] {
                    registers.borrow_mut().set(
                        cfg.map.ids()[row],
                        reg_for(RegId::HardwareErrorStatus),
                        &[0x20],
                    );
                }
                // A second of wall time per look at the clock: the maneuver
                // has its whole deadline — one stow clock plus one settle
                // window — when it commands its first stow, and none of it by
                // the time that stow comes back with a second servo gone.
                let mut clock = RunningClock {
                    now: std::cell::Cell::new(Duration::ZERO),
                    per_look: Duration::from_secs(1),
                };
                let outcome =
                    engaged.move_to(neutral_targets(), cfg.up_durations(), &mut clock, line);
                settle(engaged, outcome, &mut clock, line)
            },
        );

        ended
            .outcome
            .expect_err("a servo dropping out ends the move");
        let said = |text: &str| {
            ended
                .printed
                .iter()
                .filter(|line| line.contains(text))
                .count()
        };
        assert_eq!(
            said("the stow carries on without it"),
            1,
            "{:?}",
            ended.printed
        );
        assert_eq!(
            said("stowing under control"),
            1,
            "a second stow was commanded on a clock that was spent: {:?}",
            ended.printed
        );
        assert_eq!(said("the stow clock is spent"), 1, "{:?}", ended.printed);
        assert_eq!(
            said("stowed; releasing torque"),
            0,
            "a maneuver that ran out of clock is not a stow: {:?}",
            ended.printed
        );
        assert_eq!(
            torque_bits(&cfg, &ended.registers),
            vec![0; JointId::COUNT],
            "{:?}",
            ended.printed
        );
        assert_eq!(said("look at the machine"), 1, "{:?}", ended.printed);
    }

    /// A wind-down that runs out of head joints releases torque rather than
    /// commanding a stow nothing can carry.
    ///
    /// The end of the mask's growth, and the one place the maneuver's two
    /// outcomes are furthest apart: with the cranks and the yaw all released
    /// there is nothing left to put the head down with, and a `move_to` over a
    /// fully masked machine would emit nothing, finish on its first period and
    /// report a controlled descent that never happened.
    #[test]
    fn a_wind_down_with_no_head_left_releases_torque_and_says_so() {
        let cfg = resolved();
        let ended = engaged_run(
            &cfg,
            machine_at(&datumed_config(), &stow_legs()),
            |mut engaged, registers, clock, line| {
                // Every joint that carries the head flagging, one health sweep
                // apart: the sweep names the first unmasked one in bus order,
                // so the mask grows by one servo per stow attempt.
                for row in 0..=6 {
                    registers.borrow_mut().set(
                        cfg.map.ids()[row],
                        reg_for(RegId::HardwareErrorStatus),
                        &[0x20],
                    );
                }
                let outcome = engaged.move_to(neutral_targets(), cfg.up_durations(), clock, line);
                settle(engaged, outcome, clock, line)
            },
        );

        ended
            .outcome
            .expect_err("a servo dropping out ends the move");
        let position = |text: &str| {
            ended
                .printed
                .iter()
                .position(|line| line.contains(text))
                .unwrap_or_else(|| panic!("no {text:?} line: {:?}", ended.printed))
        };
        let out_of_joints = position("no head joint is still commanded");
        let said = |text: &str| {
            ended
                .printed
                .iter()
                .filter(|line| line.contains(text))
                .count()
        };
        assert_eq!(
            said("no head joint is still commanded"),
            1,
            "{:?}",
            ended.printed
        );
        // It gave up only when there was nothing left: every head joint but the
        // one the move itself lost was stowed on and then dropped out in turn.
        let head_joints = JointId::ALL
            .into_iter()
            .filter(|joint| joint.group() != JointGroup::Antennas)
            .count();
        assert_eq!(
            said("stowing under control"),
            head_joints - 1,
            "the maneuver stopped stowing while joints still commanded: {:?}",
            ended.printed
        );
        assert!(
            !ended.printed[out_of_joints..]
                .iter()
                .any(|line| line.contains("stowing under control")),
            "a stow was commanded on a machine with nothing to stow with: {:?}",
            ended.printed
        );
        assert_eq!(
            ended
                .printed
                .iter()
                .filter(|line| line.contains("stowed; releasing torque"))
                .count(),
            0,
            "nothing stowed the head: {:?}",
            ended.printed
        );
        assert_eq!(
            torque_bits(&cfg, &ended.registers),
            vec![0; JointId::COUNT],
            "{:?}",
            ended.printed
        );
        assert!(
            ended
                .printed
                .iter()
                .any(|line| line.contains("look at the machine")),
            "{:?}",
            ended.printed
        );
    }

    /// A run that ended on a defect of ours leaves the bench machine holding.
    ///
    /// The whole reason the classification is not a yes-or-no about faulting.
    /// Nothing here says the machine stopped being commandable: our accounting
    /// ran out, in front of an operator who can read the message and type
    /// `off`. Winding the head down over it would be a machine that flinches at
    /// its own bugs.
    #[test]
    fn a_defect_of_ours_leaves_the_bench_machine_holding() {
        let cfg = resolved();
        let ended = engaged_run(
            &cfg,
            machine_at(&datumed_config(), &stow_legs()),
            |engaged, _registers, clock, line| {
                let outcome: Result<(), PumpError> = Err(PumpError::Runaway { budget: 10 });
                settle(engaged, outcome, clock, line)
            },
        );

        let error = ended.outcome.expect_err("the run's own error comes back");
        assert_eq!(error.class(Phase::UnderTorque), ErrorClass::SlowStowToRest);
        assert_eq!(error.fault(Phase::UnderTorque), None);
        assert_eq!(
            torque_bits(&cfg, &ended.registers),
            vec![1; JointId::COUNT],
            "{:?}",
            ended.printed
        );
        assert!(
            !ended
                .printed
                .iter()
                .any(|line| line.contains("torque off") || line.contains("stowing")),
            "{:?}",
            ended.printed
        );
        assert!(
            ended
                .printed
                .iter()
                .any(|line| line.contains("still holding")),
            "{:?}",
            ended.printed
        );
    }

    /// A wire failure the sequencer has no word for does not end the release
    /// walk. The driver is a layer too, and the doctrine binds it: a reply of
    /// the wrong width from the third servo would otherwise abort the run and
    /// leave the six after it holding the head up — the exact state a fault
    /// release exists to get out of.
    #[test]
    fn a_reply_of_the_wrong_width_mid_release_does_not_stop_the_walk() {
        let cfg = resolved();
        let rig = rig(machine_at(&datumed_config(), &stow_legs()));
        let registers = rig.registers;
        let mut clock = TestClock::default();
        let mut printed: Vec<String> = Vec::new();

        let mut machine = commission(&cfg, rig.port, &mut clock, &mut |line| {
            printed.push(line.to_string());
        })
        .expect("the fixture commissions");
        let engaged = machine
            .engage(&mut clock, &mut |line| printed.push(line.to_string()))
            .expect("the fixture engages");

        // From here on the read-back of this servo's torque-off write answers a
        // byte too wide: a frame that passed its own checksum and is nobody's
        // answer. The fault this stands in for arrives mid-run, so it is put on
        // the wire mid-run.
        registers
            .borrow_mut()
            .verbose
            .push((cfg.map.ids()[2], reg_for(RegId::TorqueEnable).addr));
        let outcome = engaged.disengage_now(&mut clock, &mut |line| printed.push(line.to_string()));

        assert_eq!(
            torque_bits(&cfg, &registers),
            vec![0; JointId::COUNT],
            "the walk stopped at the servo that answered badly: {printed:?}"
        );

        // The servo is named rather than silently forgiven: its write went out,
        // nothing came back to say it landed, and that is what the run reports.
        let error = outcome.expect_err("an unacknowledged release is reported");
        assert!(
            matches!(error, PumpError::TorqueOffUnacked { id } if id == cfg.map.ids()[2]),
            "{error}"
        );
        assert!(
            printed
                .iter()
                .any(|line| line.contains("wire trouble mid-release, carried on")),
            "{printed:?}"
        );
        assert!(
            printed
                .iter()
                .any(|line| line.contains("NO ACKNOWLEDGEMENT of torque off")),
            "{printed:?}"
        );
    }
}
