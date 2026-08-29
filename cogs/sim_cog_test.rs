//! Unit tests for the simulated driver, against the generated test wrapper.
//!
//! The wrapper wires no channels, so each case publishes the goals and
//! injections it wants seen, runs one cycle, and reads back the datagrams that
//! went out and what the slot holds. The one thing it does not do for us is the
//! self-loop: this cog reads its own last published sample and event to number
//! the next ones, so every cycle here hands the wrapper back what the previous
//! cycle published, which is exactly what a box's `connect` would have done.
//!
//! Time is passed per call rather than simulated. The cog is woken by a timer,
//! so a case says at what instant each cycle happens and the harness decides
//! whether the condition is met -- which is itself asserted: a cycle offered
//! early must not run.

use brenn_reachy__cogs__config_clk_rs::{SimParams, SimParamsWire};
use brenn_reachy__cogs__session_cmd_clk_rs::{SessionCmdKindWire, SessionCmdWire};
use brenn_reachy__cogs__sim_clk_rs_test::MotorSimTestWrapper;
use brenn_reachy__cogs__sim_state_clk_rs::{SimCmdWire, SimOpWire, SimState, SimStateWire};
use brenn_reachy__driver__gate_clk_rs::GateState;
use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
use brenn_reachy__driver__health_clk_rs::{AuxStatus, EventKind};
use brenn_reachy__driver__pose_clk_rs::PoseSample;
use brenn_reachy__hardware__dynamixel__registers_clk_rs::{RegId, RegIdWire, ValueShapeWire};
use brenn_reachy__motion__bus_txn_clk_rs::{AuxOpKindWire, BusTxnWire};
use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointFlagsWire, JointsWire};
use clockwork_rs::SyncTime;
use reachy_driver::{BLIND_CYCLES_BEFORE_BUS_FAILURE, JOINT_COUNT, JOINT_MASK_ALL};
use reachy_kin::default_geometry;
use reachy_motion::arm::{
    DEFAULT_GAINS, DEFAULT_MIN_ARM_VOLTAGE, EXPECTED_MODELS, EXPECTED_OPERATING_MODES,
    VENDOR_HOMING_OFFSETS,
};
use reachy_motion::disarm::stow_targets;
use reachy_motion::joints::{
    self, JointRef, ServoHealth, flags, rows_of, write_rows, write_vector,
};
use reachy_motion::value;
use sim_cogs::{sim_aux, sim_regs};

/// The instant every case starts from. Round rather than zero, so a time that
/// travelled through the wrong field is a number nothing else in the case is.
const T0: i64 = 1_700_000_000_000_000_000;

/// The bus cycle, which is what the cog's one execution condition waits for.
const PERIOD: i64 = 20_000_000;

/// How long the goal stream may be silent before the gate de-torques.
const HOLD_TIMEOUT: i64 = 200_000_000;

/// Per-cycle slew of the cranks and the body yaw, radians.
const SLEW_LEGS: f64 = 0.15;

/// How long the driver waits between health reports.
const HEALTH_PERIOD: i64 = 120_000_000;

/// How long a commanded de-torquing may go unconfirmed before the driver says
/// so. Restated rather than read: a case that took the number from the driver
/// could not notice it changing, and the assert below is what holds the two
/// together.
const TORQUE_OFF_CONFIRM_BUDGET: i64 = 300_000_000;

const _: () = assert!(TORQUE_OFF_CONFIRM_BUDGET == reachy_driver::TORQUE_OFF_CONFIRM_BUDGET_NS);

/// Per-cycle slew of the antennas, radians.
const SLEW_ANTENNAS: f64 = 0.65;

/// The lag a well-formed goal stream commands at: a goal names the grid instant
/// two cycles ahead of the sample that produced it.
const LAG: i64 = 2;

/// The most cycles of motion one execution may make up, which is the cog's own
/// `MAX_CATCHUP_CYCLES` -- a private constant of a crate this test drives
/// through its wrapper, so it is restated here and asserted against.
const MAX_CATCHUP_CYCLES: f64 = 8.0;

/// The nine angles the machine rests at, which is where every case finds it.
fn stow_rows() -> [f64; JOINT_COUNT] {
    let mut slot = JointsWire::new();
    write_vector(
        slot.clear_valid(),
        &stow_targets(default_geometry()).expect("the baked geometry reaches stow"),
    );
    rows_of(
        slot.validate()
            .expect("a cleared vector of angles reads back"),
    )
}

/// What one cycle published.
struct Cycle {
    /// The cycle's grid instant.
    nominal: i64,
    /// The sample, which every cycle publishes without exception.
    sample: Sample,
    /// The event, if the cycle had one to report.
    event: Option<Event>,
    /// The answer to one aux transaction, if the cycle ran or refused one.
    outcome: Option<Outcome>,
    /// The health report, if the rotation was due.
    health: Option<Health>,
    /// The whole run so far, if this cycle was one the record goes out on.
    status: Option<Status>,
}

/// One published status record, copied for the reason [`Sample`] is.
///
/// Every field of the record, and not the subset a case happens to read: the
/// whole value is asserted at once by
/// [`the_record_this_driver_composes_is_this_one_whole`], which is what makes a
/// field added to the schema and left unwritten here visible.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Status {
    /// When the record was composed.
    time_ns: i64,
    /// When the process released the machine it met.
    sweep_time_ns: i64,
    /// The rows whose release did not take, as the bits the schema holds.
    sweep_failed_rows: u16,
    /// Whether the gate stands in a torque-off latch.
    torque_latched: bool,
    /// When the first sample went out, and zero until one has.
    first_pose_ns: i64,
    /// When the first of the host's datagrams was taken, and zero until one
    /// has been.
    first_session_cmd_ns: i64,
    /// Datagrams handed to a cycle.
    queued: u64,
    /// Setpoints that arrived.
    goals: u64,
    /// Host datagrams that arrived, whatever they turned out to say.
    session_cmds: u64,
    /// Datagrams that arrived whole and read as nothing this build knows.
    invalid: u64,
    /// Datagrams the transport delivered at the wrong length.
    wrong_size: u64,
    /// Datagrams a full queue lost.
    overflowed: u64,
    /// Datagrams the sender could not deliver.
    undelivered: u64,
    /// Receives the transport refused.
    recv_errors: u64,
    /// Cycles run with a reader thread stopped.
    readers_stopped: u64,
    /// Goals written to the modelled servos.
    goals_executed: u64,
    /// Goals a cycle would not act on.
    goals_dropped: u64,
    /// Cycles the dead-man expired on.
    hold_timeouts: u64,
    /// Reads that came back short.
    read_misses: u64,
    /// Writes that did not land.
    write_failures: u64,
    /// Events a full slot lost.
    events_dropped: u64,
    /// Host datagrams turned away as asks the driver could not act on.
    aux_refused: u64,
    /// Delivery re-issues the slot recognised.
    aux_duplicates: u64,
    /// Transactions a cycle had no room for.
    aux_deferred: u64,
    /// Health reports published.
    health_reports: u64,
    /// Rotation turns the bus did not answer.
    health_misses: u64,
    /// Torque-off read-backs nothing answered.
    confirm_misses: u64,
    /// Cycles in which the bus answered nothing.
    blind_cycles: u64,
    /// Cycles run.
    cycles: u64,
    /// Grid points passed over without a cycle running.
    skipped: u64,
    /// Times the process-start release was written.
    startup_mrc: u64,
    /// Cycles a stopped reader was answered with a release on.
    wire_failures: u64,
    /// Datagrams taken off the seam.
    taken: u64,
    /// Times the clock stepped under the loop.
    clock_steps: u64,
    /// Messages this driver has published.
    published: u64,
    /// Publishes the transport refused.
    publish_failures: u64,
    /// Whether this is the copy a wind-down published on its way out.
    wound_down: bool,
}

/// One published aux outcome, copied out of the message for the reason
/// [`Sample`] is.
#[derive(Clone, Copy)]
struct Outcome {
    /// The correlation number of the request it answers.
    corr: u32,
    status: AuxStatus,
    /// The answer's bits, whatever shape they are in.
    value: u64,
    /// The shape those bits are in, which is `none` for an answer that carries
    /// no value at all.
    value_kind: ValueShapeWire,
    /// The model number a ping answered with.
    model: u16,
}

/// One published health report, copied for the same reason.
#[derive(Clone, Copy)]
struct Health {
    /// The servo, by bus id.
    id: u8,
    /// Its latched error byte.
    bits: u8,
    /// Its rail, volts.
    volts: f64,
    /// Its temperature, degrees Celsius.
    temp_c: i8,
    /// When the reading was taken.
    sample_time_ns: i64,
}

/// One published event, copied out of the message for the reason [`Sample`] is.
#[derive(Clone, Copy)]
struct Event {
    kind: EventKind,
    time_ns: i64,
    /// The silence or the lateness the kind names, nanoseconds.
    silence_ns: i64,
    /// How many of whatever the kind counts.
    count: u32,
    /// The servos the kind names, as the bits the schema holds.
    rows: u16,
}

/// One published sample, copied out of the message.
///
/// Copied because the wrapper returns a borrow into cog memory that the next
/// cog call invalidates, and a case compares two cycles.
struct Sample {
    /// The cycle's grid instant.
    nominal_time_ns: i64,
    /// When the read completed.
    sample_time_ns: i64,
    /// Whether the reading is a complete one.
    present_valid: bool,
    /// Whether a setpoint is being held.
    commanded_valid: bool,
    torque_off_latched: bool,
    /// The rows that did not answer, as the bits the schema holds.
    missing: u16,
    /// Measured positions, radians in bus order.
    present: [f64; JOINT_COUNT],
    /// The setpoint held, radians in bus order.
    commanded: [f64; JOINT_COUNT],
}

impl Sample {
    fn of(msg: &PoseSample) -> Self {
        Self {
            nominal_time_ns: msg.nominal_time.as_nanos(),
            sample_time_ns: msg.sample_time.as_nanos(),
            present_valid: msg.present_valid.into(),
            commanded_valid: msg.commanded_valid.into(),
            torque_off_latched: msg.torque_off_latched.into(),
            missing: JointFlagsWire::from(msg.missing).0,
            present: rows_of(&msg.present),
            commanded: rows_of(&msg.commanded),
        }
    }
}

/// One setpoint, as the control loop's channel carries it.
///
/// Written here in the bus-row terms the cases reason in -- a mask of bits and
/// nine angles in row order -- and mapped into the schema's vocabulary once, so
/// a case says what it means and only this function knows which field is which.
fn setpoint(execute_at_ns: i64, mask: u16, targets: [f64; JOINT_COUNT]) -> GoalSetpointWire {
    let mut msg = GoalSetpointWire::new();
    let view = msg.clear_valid();
    view.execute_at = SyncTime::from_nanos(execute_at_ns);
    view.mask = JointFlagsWire(mask)
        .to_known()
        .expect("the cases name servos this build knows");
    write_rows(&mut view.targets, &targets);
    msg
}

/// A cog under test, with its own channels looped back by hand.
struct Sim {
    /// The wrapper.
    cog: MotorSimTestWrapper,
    /// The instant of the last cycle run.
    now: i64,
}

impl Sim {
    /// A simulated driver at stow, de-torqued, on the default parameters.
    fn new() -> Self {
        Self::with_params(|_| {})
    }

    /// The same, energised before its first cycle ends.
    ///
    /// By injection, which is the vocabulary an arming sequencer's hand on this
    /// machine has: the process itself releases every row as its first act, so
    /// a machine that is energised when a case starts asserting is one that
    /// something else energised. The injection is queued before the first
    /// execution and read in it, after the release.
    fn armed() -> Self {
        let mut sim = Self::new();
        sim.inject(SimOpWire::TORQUE_ON, flags::all());
        sim
    }

    /// The same, meeting a machine a predecessor left energised. What a case
    /// built this way asserts is the release, not the arming.
    fn met_torqued() -> Self {
        Self::with_params(|params| params.start_torqued = true.into())
    }

    /// The same, with the scenario's parameters altered before the cog reads
    /// them -- which is how a case says what a badly configured scenario is.
    fn with_params(edit: impl FnOnce(&mut SimParams)) -> Self {
        let mut cog = MotorSimTestWrapper::new();
        cog.input_goals_set_num_slots(8);
        cog.input_cmds_set_num_slots(8);
        cog.input_session_cmds_set_num_slots(8);
        cog.input_own_pose_set_num_slots(1);

        // Seeded after `initialize`: a config record is not reachable before
        // the wrapper has stood the cog up, and the first execution has not run
        // yet, so nothing reads it in between.
        cog.initialize(SyncTime::from_nanos(T0));

        let mut message = SimParamsWire::new();
        let params = message.clear_valid();
        params.period_ns = PERIOD;
        params.hold_timeout_ns = HOLD_TIMEOUT;
        params.slew_legs_rad = SLEW_LEGS;
        params.slew_body_yaw_rad = SLEW_LEGS;
        params.slew_antennas_rad = SLEW_ANTENNAS;
        params.health_poll_period_ns = HEALTH_PERIOD;
        edit(params);
        cog.set_config_params(&message);

        Self { cog, now: T0 }
    }

    /// Hand the cog a setpoint, as the control loop's channel would.
    fn send_goal(&mut self, goal: &GoalSetpointWire) {
        self.cog.publish_goals(goal, SyncTime::from_nanos(self.now));
    }

    /// Hand the cog one of the scenario's injections.
    fn inject(&mut self, op: SimOpWire, mask: JointFlags) {
        let mut cmd = SimCmdWire::new();
        cmd.set_op(op);
        cmd.set_mask(JointFlagsWire::from(mask));
        self.cog.publish_cmds(&cmd, SyncTime::from_nanos(self.now));
    }

    /// Inject a command carrying a payload the plain form has no room for.
    fn inject_full(&mut self, cmd: &SimCmdWire) {
        self.cog.publish_cmds(cmd, SyncTime::from_nanos(self.now));
    }

    /// Hand the cog one of the host's datagrams, as the session's channel would.
    fn ask(&mut self, cmd: &SessionCmdWire) {
        self.cog
            .publish_session_cmds(cmd, SyncTime::from_nanos(self.now));
    }

    /// A datagram that asks for liveness and nothing else.
    fn keep_alive(&mut self) {
        let mut cmd = SessionCmdWire::new();
        cmd.set_kind(SessionCmdKindWire::KEEP_ALIVE);
        self.ask(&cmd);
    }

    /// Ask for one transaction under `corr`.
    fn transact(&mut self, corr: u32, txn: &BusTxnWire) {
        let mut cmd = SessionCmdWire::new();
        cmd.set_kind(SessionCmdKindWire::AUX);
        cmd.set_corr(corr);
        *cmd.txn_mut() = txn.clone();
        self.ask(&cmd);
    }

    /// Run one cycle and loop this cog's own publications back to it.
    fn step(&mut self) -> Cycle {
        self.step_by(1)
    }

    /// The same, run `cycles` after the last one rather than one -- what a
    /// process that lost the CPU for a while looks like to a cog on a timer.
    fn step_by(&mut self, cycles: i64) -> Cycle {
        self.now += cycles * PERIOD;
        assert!(
            self.cog.execute(SyncTime::from_nanos(self.now)),
            "the cycle's timer is due at {}",
            self.now
        );

        let published = self
            .cog
            .try_next_pose()
            .expect("every cycle publishes a sample");
        let sample = Sample::of(published.validate().expect("a sample the driver wrote"));
        let message = published.clone();
        let outcome = self.cog.try_next_aux_out().map(|published| {
            let outcome = published.validate().expect("an outcome the driver wrote");
            Outcome {
                corr: outcome.corr,
                status: outcome.status,
                value: outcome.value,
                value_kind: ValueShapeWire::from(outcome.value_kind),
                model: outcome.model,
            }
        });
        let health = self.cog.try_next_health_out().map(|published| {
            let report = published.validate().expect("a report the driver wrote");
            Health {
                id: report.id,
                bits: report.bits,
                volts: report.volts,
                temp_c: report.temp_c,
                sample_time_ns: report.sample_time.as_nanos(),
            }
        });
        let event = self.cog.try_next_evt().map(|published| {
            let event = published.validate().expect("an event the driver wrote");
            Event {
                kind: event.kind,
                time_ns: event.time.as_nanos(),
                silence_ns: event.silence.as_nanos(),
                count: event.count,
                rows: JointFlagsWire::from(event.rows).0,
            }
        });

        let status = self.cog.try_next_status().map(|published| {
            let status = published.validate().expect("a status the driver wrote");
            Status {
                time_ns: status.time.as_nanos(),
                sweep_time_ns: status.sweep_time.as_nanos(),
                sweep_failed_rows: JointFlagsWire::from(status.sweep_failed_rows).0,
                torque_latched: status.torque_latched.into(),
                first_pose_ns: status.first_pose.as_nanos(),
                first_session_cmd_ns: status.first_session_cmd.as_nanos(),
                queued: status.seam.queued,
                goals: status.seam.goals,
                session_cmds: status.seam.session_cmds,
                invalid: status.seam.invalid,
                wrong_size: status.seam.wrong_size,
                overflowed: status.seam.overflowed,
                undelivered: status.seam.undelivered,
                recv_errors: status.seam.recv_errors,
                readers_stopped: status.seam.readers_stopped,
                goals_executed: status.cycle.goals_executed,
                goals_dropped: status.cycle.goals_dropped,
                hold_timeouts: status.cycle.hold_timeouts,
                read_misses: status.cycle.read_misses,
                write_failures: status.cycle.write_failures,
                events_dropped: status.cycle.events_dropped,
                aux_refused: status.cycle.aux_refused,
                aux_duplicates: status.cycle.aux_duplicates,
                aux_deferred: status.cycle.aux_deferred,
                health_reports: status.cycle.health_reports,
                health_misses: status.cycle.health_misses,
                confirm_misses: status.cycle.confirm_misses,
                blind_cycles: status.cycle.blind_cycles,
                cycles: status.loop_counts.cycles,
                skipped: status.loop_counts.skipped,
                startup_mrc: status.loop_counts.startup_mrc,
                wire_failures: status.loop_counts.wire_failures,
                taken: status.loop_counts.taken,
                clock_steps: status.loop_counts.clock_steps,
                published: status.published,
                publish_failures: status.publish_failures,
                wound_down: status.wound_down.into(),
            }
        });

        // The self-loop, which a box would have made: the sample the cog reads
        // its own last publication back from.
        self.cog
            .publish_own_pose(&message, SyncTime::from_nanos(self.now));

        Cycle {
            nominal: self.now,
            sample,
            event,
            outcome,
            health,
            status,
        }
    }

    /// Run `count` cycles, sending nothing.
    fn quiet(&mut self, count: usize) -> Vec<Cycle> {
        (0..count).map(|_| self.step()).collect()
    }

    /// Run one cycle having commanded `targets` for the rows in `mask`, at the
    /// lag a well-formed goal stream uses.
    fn commanded_step(&mut self, targets: &[f64; JOINT_COUNT], mask: u16) -> Cycle {
        let goal = setpoint(self.now + PERIOD + LAG * PERIOD, mask, *targets);
        self.send_goal(&goal);
        self.step()
    }

    /// The same, `cycles` after the last one: the goal stream carries on, so
    /// what the late cycle shows is the plant catching up and not the dead-man.
    fn commanded_step_by(&mut self, targets: &[f64; JOINT_COUNT], mask: u16, cycles: i64) -> Cycle {
        let goal = setpoint(self.now + cycles * PERIOD, mask, *targets);
        self.send_goal(&goal);
        self.step_by(cycles)
    }

    /// What the state slot holds, read the way the cog reads it.
    fn slot(&self) -> &SimState {
        self.cog
            .state_sim()
            .validate()
            .expect("the cog leaves a state in its slot")
    }

    /// The gate the last cycle left.
    fn gate(&self) -> &GateState {
        &self.slot().gate
    }
}

/// The driver layer's "every bus row" and the motion vocabulary's are one set.
///
/// They are two definitions on purpose: `reachy-driver` links the schema and
/// nothing else, so it cannot reach `flags::all()`, and this cog is where both
/// are in scope. Pinned rather than commented, because a servo added to the
/// vocabulary has to reach the driver's belief as well as the plant's, and a
/// divergence would show up as rows that never de-torque.
#[test]
fn the_driver_and_the_vocabulary_name_the_same_bus() {
    assert_eq!(reachy_driver::every_row(), flags::all());
}

#[test]
fn the_machine_starts_stowed_de_torqued_and_saying_so() {
    let mut sim = Sim::new();

    assert!(!sim.cog.execute(SyncTime::from_nanos(T0 + PERIOD - 1)));
    assert!(sim.cog.try_next_pose().is_none());

    let cycle = sim.step();
    assert_eq!(cycle.sample.nominal_time_ns, cycle.nominal);
    assert_eq!(
        cycle.sample.sample_time_ns, cycle.nominal,
        "a simulated bus read has no jitter to report"
    );
    assert_eq!(cycle.sample.present, stow_rows(), "the machine is at rest");
    assert!(cycle.sample.present_valid);
    assert!(
        !cycle.sample.commanded_valid,
        "nothing has been commanded yet"
    );
    assert!(!cycle.sample.torque_off_latched);
    assert_eq!(cycle.sample.missing, 0);
    assert!(cycle.event.is_none(), "a quiet cycle reports nothing");

    let slot = sim.slot();
    assert!(slot.initialized.get());
    assert_eq!(
        slot.torqued,
        JointFlags::NONE,
        "the scenario has not armed it"
    );
    assert_eq!(rows_of(&slot.positions), stow_rows());
}

/// The process releases the machine it met, before its first cycle ends.
///
/// The real driver's first act on the bus, modelled: a driver that has just
/// started cannot know what a predecessor left energised. `start_torqued` says
/// the process met an energised machine, and what this asserts is that it does
/// not stay that way -- rows limp, belief released, gate never commanded, and
/// the release stamped in the record every status copy carries.
#[test]
fn the_process_releases_the_machine_it_met_before_its_first_cycle_ends() {
    let mut sim = Sim::met_torqued();

    let cycle = sim.step();

    let slot = sim.slot();
    assert_eq!(
        slot.torqued,
        JointFlags::NONE,
        "the machine the process met is limp by the end of the first cycle",
    );
    assert_eq!(slot.has_target, JointFlags::NONE);
    assert_eq!(
        slot.aux.believed_torqued,
        JointFlags::NONE,
        "the belief goes with the rows: the release is verified as it is written",
    );
    let gate = sim.gate();
    assert!(!gate.latched.get(), "a verified release latches nothing");
    assert!(!gate.has_held.get());
    assert!(!gate.has_accepted.get(), "the gate has commanded nothing");
    assert!(!cycle.sample.torque_off_latched);

    let status = cycle.status.expect("the first cycle publishes a status");
    assert_eq!(
        status.sweep_time_ns, cycle.nominal,
        "the release ran in the first execution",
    );
    assert_eq!(
        status.sweep_failed_rows, 0,
        "the modelled release has no write that can fail",
    );
    assert!(!status.torque_latched);
    assert_eq!(status.startup_mrc, 1, "written once, on every run");
}

/// A machine armed after that release stays armed.
///
/// The other half of the case above: the injection is read in the same
/// execution, after the release, so an arming sequencer's hand still lands.
#[test]
fn a_machine_armed_after_the_release_is_energised_at_the_end_of_the_first_cycle() {
    let mut sim = Sim::armed();

    sim.step();

    assert_eq!(
        sim.slot().torqued,
        flags::all(),
        "the injection is read after the release, not before it",
    );
    assert_eq!(sim.slot().aux.believed_torqued, flags::all());
    assert!(!sim.gate().latched.get());
}

/// The status record goes out on the first cycle and once a simulated second.
///
/// Cumulative and complete, which is the whole point of it: a reader that has
/// seen one copy has read the run. So the cadence only decides how much
/// redundancy a log holds, and the first cycle publishes one so that the
/// release is on the wire from the first instant.
#[test]
fn a_status_record_goes_out_on_the_first_cycle_and_once_a_simulated_second() {
    let mut sim = Sim::new();

    let first = sim.step().status.expect("the first cycle publishes one");
    assert_eq!(first.cycles, 1);
    assert_eq!(first.time_ns, T0 + PERIOD);
    assert!(!first.wound_down, "nothing winds this driver down");

    let cadence = usize::try_from(1_000_000_000 / PERIOD).expect("a second of cycles");
    let between = sim.quiet(cadence - 1);
    assert!(
        between.iter().all(|cycle| cycle.status.is_none()),
        "one copy a simulated second, and not one a cycle",
    );

    let second = sim
        .quiet(1)
        .pop()
        .expect("a cycle")
        .status
        .expect("a copy a second after the first");
    assert_eq!(
        second.time_ns - first.time_ns,
        1_000_000_000,
        "the cadence is a simulated second",
    );
    assert_eq!(second.cycles, u64::try_from(cadence).expect("a count") + 1);
    assert!(
        second.published > first.published,
        "every count is the run's total at the copy's instant",
    );
    assert!(!second.wound_down);
}

/// The whole record, field for field, over one known run.
///
/// `DriverStatus` is composed twice -- once by the real driver's `publish_status`
/// and once by this cog's `write_status` -- as two hand-written mappings out of
/// different state, and the analyzer that verifies a run reads whichever the log
/// carries. Nothing in either language joins them. So the value is asserted
/// whole rather than field by field: a field added to the schema, or one this
/// driver quietly stops writing, changes this literal and has to be answered
/// here.
///
/// The zeros are the justified set, and each says why it is one. They are the
/// fields naming something an in-process channel cannot do -- a datagram of the
/// wrong length, a queue that overflowed, a reader thread that stopped, a send
/// or a write the transport refused -- plus the states this run did not reach.
/// A zero appearing beside a *new* field is what this case exists to make
/// somebody look at.
#[test]
fn the_record_this_driver_composes_is_this_one_whole() {
    let mut sim = Sim::new();
    sim.keep_alive();
    let first = sim.step().status.expect("the first cycle publishes one");

    assert_eq!(
        first,
        Status {
            // The first execution's own instant, written out rather than read
            // off the value under test: four fields carry it, and asserting
            // each against itself would pass any clock the driver stamped them
            // from so long as it stamped all four from the same one.
            time_ns: T0 + PERIOD,
            sweep_time_ns: T0 + PERIOD,
            // The modelled release writes every row and cannot leave one
            // behind, and a verified release latches nothing.
            sweep_failed_rows: 0,
            torque_latched: false,
            first_pose_ns: T0 + PERIOD,
            first_session_cmd_ns: T0 + PERIOD,
            queued: 1,
            goals: 0,
            session_cmds: 1,
            invalid: 0,
            // The seam here is a channel: no length, no queue, no reader
            // thread and no send of its own to fail.
            wrong_size: 0,
            overflowed: 0,
            undelivered: 0,
            recv_errors: 0,
            readers_stopped: 0,
            goals_executed: 0,
            goals_dropped: 0,
            hold_timeouts: 0,
            // A cycle either reads every row or reads none: there is no short
            // read and no write that fails to land.
            read_misses: 0,
            write_failures: 0,
            events_dropped: 0,
            aux_refused: 0,
            aux_duplicates: 0,
            // One transaction a cycle, and the slot is never asked twice.
            aux_deferred: 0,
            // The rotation visits a servo on the first cycle.
            health_reports: 1,
            health_misses: 0,
            confirm_misses: 0,
            blind_cycles: 0,
            cycles: 1,
            skipped: 0,
            // Once, on the first execution, as it is on every run of the real
            // driver.
            startup_mrc: 1,
            // The wire this driver's fail-safes measure cannot fail, and the
            // clock is the runner's.
            wire_failures: 0,
            taken: 1,
            clock_steps: 0,
            // The sample and the health report. The record's own publication
            // is not in it: it has not happened yet.
            published: 2,
            // An in-process publish is a slot write.
            publish_failures: 0,
            // Nothing winds this driver down: the runner stops the process.
            wound_down: false,
        },
    );
}

/// Every counter the status carries is this driver's own honest count.
///
/// The record is what a run is verified from, so each number has to be the one
/// the thing it names actually did: a goal written, an ask turned away, a
/// delivery re-issue recognised, a health report published, a cycle the bus
/// answered nothing on, and a grid point no cycle attended.
#[test]
fn the_status_counts_what_this_driver_did() {
    let mut sim = Sim::armed();
    sim.step();

    // A goal, executed at its instant.
    let mut asked = stow_rows();
    asked[0] += 0.05;
    sim.commanded_step(&asked, JointFlagsWire::from(flags::all()).0);
    sim.quiet(3);

    // A datagram asking nothing at all: an ask the driver turned away.
    sim.ask(&SessionCmdWire::new());
    // Two copies of one transaction in one cycle: the second is the transport
    // repeating itself, which the first one's outcome answers.
    let txn = transaction(AuxOpKindWire::PING, 10, RegIdWire::NONE, None);
    sim.transact(31, &txn);
    sim.transact(31, &txn);
    sim.step();

    // A cycle whose replies were all lost, and an execution that stepped over
    // two grid points on its way.
    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::DROP_REPLIES);
    cmd.set_count(1);
    sim.inject_full(&cmd);
    sim.step();
    sim.step_by(3);

    let cadence = usize::try_from(1_000_000_000 / PERIOD).expect("a second of cycles");
    let status = sim
        .quiet(cadence)
        .into_iter()
        .filter_map(|cycle| cycle.status)
        .next_back()
        .expect("a copy within a second of cycles");
    assert_eq!(status.goals_executed, 1);
    assert_eq!(status.aux_refused, 1, "the datagram that asked nothing");
    assert_eq!(status.aux_duplicates, 1, "the second copy of one request");
    assert!(
        status.health_reports > 0,
        "the rotation has come round by now",
    );
    assert_eq!(status.blind_cycles, 1);
    assert_eq!(status.skipped, 2, "the two grid points nothing attended");
}

/// The two first-contact instants are stamped once, and a host that commanded
/// the machine on the cycle that first showed it one ties.
///
/// What the run report reads to know the session waited for a sample before it
/// commissioned. Both are the driver's own clock, and the datagrams are taken
/// before the sample goes out, so the tie is what a session commanding on the
/// first cycle looks like from here.
#[test]
fn the_first_sample_and_the_first_host_datagram_are_stamped_once() {
    let mut sim = Sim::new();

    sim.keep_alive();
    let first = sim.step().status.expect("the first cycle publishes one");
    assert_eq!(first.first_pose_ns, first.time_ns);
    assert_eq!(
        first.first_session_cmd_ns, first.first_pose_ns,
        "a datagram taken on the cycle that published the first sample ties",
    );

    let cadence = usize::try_from(1_000_000_000 / PERIOD).expect("a second of cycles");
    sim.keep_alive();
    let later = sim
        .quiet(cadence)
        .pop()
        .expect("a cycle")
        .status
        .expect("the next copy");
    assert_eq!(later.first_pose_ns, first.first_pose_ns);
    assert_eq!(
        later.first_session_cmd_ns, first.first_session_cmd_ns,
        "the instants are the first ones, not the latest",
    );
    assert!(later.session_cmds > first.session_cmds);
}

/// What the status counts of the seam is what arrived on it.
///
/// Including the datagrams no ask could be read out of: this driver's transport
/// is the channel itself, so the census of that channel counts them and a count
/// that did not could not be checked against it.
#[test]
fn the_status_counts_every_datagram_that_arrived_including_the_unreadable() {
    let mut sim = Sim::new();
    sim.step();

    // A kind past the vocabulary: bytes that describe no datagram.
    let mut cmd = SessionCmdWire::new();
    cmd.set_kind(SessionCmdKindWire(9));
    sim.ask(&cmd);
    sim.keep_alive();
    sim.send_goal(&setpoint(sim.now + PERIOD, 0, stow_rows()));

    let cadence = usize::try_from(1_000_000_000 / PERIOD).expect("a second of cycles");
    let status = sim
        .quiet(cadence)
        .pop()
        .expect("a cycle")
        .status
        .expect("the next copy");
    assert_eq!(status.session_cmds, 2, "both of them arrived");
    assert_eq!(status.invalid, 1, "one of them read as nothing");
    assert_eq!(status.goals, 1);
    assert_eq!(
        status.queued,
        status.session_cmds + status.goals,
        "a cycle reads the channels directly: what arrived is what reached it",
    );
}

#[test]
fn a_sample_goes_out_every_cycle_stamped_with_the_cycle_it_is_for() {
    let mut sim = Sim::new();
    for (step, cycle) in sim.quiet(5).into_iter().enumerate() {
        assert_eq!(
            cycle.sample.nominal_time_ns,
            T0 + (step as i64 + 1) * PERIOD,
            "one sample per cycle, on the grid",
        );
    }
}

/// A partial mask writes its own rows and leaves every other servo holding what
/// it already had.
///
/// The whole meaning of a mask to a driver, and the one place in the tree that
/// turns a commanded setpoint into servo targets, so this is where it is held.
/// A mask applied the other way round -- or ignored -- moves servos nobody
/// asked to move.
#[test]
fn a_partial_mask_commands_only_its_own_rows() {
    let mut sim = Sim::armed();
    let stow = stow_rows();
    let mut asked = stow;
    for row in &mut asked {
        *row += 0.05;
    }

    let commanded = set_of(&[JointRef::BodyYaw, JointRef::Leg1]);
    sim.commanded_step(&asked, JointFlagsWire::from(commanded).0);
    let due = sim.quiet(1 + LAG as usize).pop().expect("a cycle ran");

    assert!(due.sample.commanded_valid);
    assert_eq!(
        due.sample.commanded, asked,
        "the sample reports the setpoint as it was sent, mask and all"
    );

    // Which rows those are comes from the set, not from a hand-written pair of
    // indices: the joint-to-row numbering is the vocabulary's, and this is the
    // case whose whole subject is that a mask reaches the rows it names.
    let mut wanted = stow;
    for joint in flags::iter(commanded) {
        let row = joints::row(joint).expect("a masked joint is a servo");
        wanted[row] = asked[row];
    }

    let moved = sim.quiet(20).pop().expect("a cycle ran");
    for (row, (found, want)) in moved.sample.present.iter().zip(wanted.iter()).enumerate() {
        assert_close(*found, *want, &format!("present row {row}"));
    }
}

#[test]
fn a_goal_is_written_at_its_instant_and_not_before() {
    let mut sim = Sim::armed();
    let mut targets = stow_rows();
    targets[0] += 0.05;

    let first = sim.commanded_step(&targets, JOINT_MASK_ALL);
    assert!(
        !first.sample.commanded_valid,
        "the goal is queued, not yet due"
    );
    assert_eq!(sim.gate().queue.len(), 1);

    let waiting = sim.step();
    assert!(!waiting.sample.commanded_valid, "still one cycle early");

    let due = sim.step();
    assert!(due.sample.commanded_valid, "its instant has come round");
    assert_eq!(due.sample.commanded, targets);
    assert_eq!(sim.slot().goals_executed, 1);
    assert_eq!(
        due.sample.present[0],
        stow_rows()[0] + 0.05,
        "a step inside one cycle's slew arrives whole"
    );

    // Every quiet cycle after it rewrites the same setpoint, which is what
    // holds a servo's position loop awake. A rewrite is not a goal executed:
    // counting one would make this total climb at the bus rate and wreck the
    // number a scenario checker reads it for.
    for step in 0..4 {
        let quiet = sim.step();
        assert_eq!(quiet.sample.commanded, targets, "cycle {step}");
        assert_eq!(
            sim.slot().goals_executed,
            1,
            "cycle {step}: one goal has been executed, however many rewrites",
        );
    }
}

#[test]
fn a_servo_moves_no_further_than_its_slew_in_one_cycle() {
    let mut sim = Sim::armed();
    // Further than any servo can travel in a cycle, so every cycle is a
    // full-rate one and the rate is what is asserted.
    let mut targets = stow_rows();
    targets[0] += 10.0;
    targets[1] += 10.0;
    targets[7] += 10.0;

    let mut previous = stow_rows();
    for step in 0..8 {
        let cycle = sim.commanded_step(&targets, JOINT_MASK_ALL);
        if step < LAG {
            assert_eq!(cycle.sample.present, previous, "cycle {step}");
            continue;
        }
        assert_close(
            cycle.sample.present[0] - previous[0],
            SLEW_LEGS,
            "the body yaw moves at its own rate",
        );
        assert_close(
            cycle.sample.present[1] - previous[1],
            SLEW_LEGS,
            "a crank moves at the legs' rate",
        );
        assert_close(
            cycle.sample.present[7] - previous[7],
            SLEW_ANTENNAS,
            "an antenna moves at the antennas' rate",
        );
        previous = cycle.sample.present;
    }
}

#[test]
fn a_partial_mask_moves_only_its_own_rows() {
    let mut sim = Sim::armed();
    let start = stow_rows();
    let mut targets = start;
    for row in &mut targets {
        *row += 0.05;
    }
    // Two rows, chosen apart so a mask applied to the wrong end is visible.
    let rows = set_of(&[JointRef::BodyYaw, JointRef::AntennaLeft]);
    let mask = JointFlagsWire::from(rows).0;

    for _ in 0..(LAG + 2) {
        sim.commanded_step(&targets, mask);
    }
    let settled = sim.step();

    assert_close(settled.sample.present[0], targets[0], "a masked row moves");
    assert_close(settled.sample.present[8], targets[8], "so does the other");
    for (row, resting) in start.iter().enumerate().take(8).skip(1) {
        assert_eq!(
            settled.sample.present[row], *resting,
            "row {row} was not in the mask and must not have moved",
        );
    }
    let slot = sim.slot();
    assert_eq!(slot.has_target, rows, "only those rows are commanded");
}

#[test]
fn a_jammed_servo_holds_while_the_rest_of_the_machine_tracks() {
    let mut sim = Sim::armed();
    let start = stow_rows();
    let mut targets = start;
    for row in &mut targets {
        *row += 0.05;
    }

    // One crank jammed where it stands. Everything else is asked for the same
    // move, so what separates them is the obstruction and nothing else.
    sim.inject(SimOpWire::OBSTRUCT, one(JointRef::Leg2));
    for _ in 0..(LAG + 2) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }
    let jammed = sim.step();
    assert_eq!(
        jammed.sample.present[3], start[3],
        "a jammed servo does not move, whatever it is asked for"
    );
    assert_close(jammed.sample.present[1], targets[1], "its neighbour tracks");

    sim.inject(SimOpWire::RELEASE_OBSTRUCTION, one(JointRef::Leg2));
    let freed = sim.commanded_step(&targets, JOINT_MASK_ALL);
    assert_close(
        freed.sample.present[3],
        targets[3],
        "a released servo resumes tracking",
    );
}

#[test]
fn a_de_torqued_servo_holds_where_it_stands() {
    let mut sim = Sim::new();
    let start = stow_rows();
    let mut targets = start;
    targets[1] += 1.0;

    for _ in 0..(LAG + 3) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }
    let cycle = sim.step();
    assert_eq!(
        cycle.sample.present, start,
        "a write to a de-torqued servo moves nothing"
    );
    assert!(
        cycle.sample.commanded_valid,
        "the setpoint was taken up all the same, as a real bus write would be"
    );
}

#[test]
fn a_teleport_puts_the_servos_where_the_scenario_says() {
    let mut sim = Sim::new();
    let mut positions = JointsWire::new();
    let mut wanted = stow_rows();
    wanted[2] = 1.25;
    wanted[8] = -0.5;
    write_rows(positions.clear_valid(), &wanted);

    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::SET_POSITIONS);
    cmd.set_mask(JointFlagsWire::from(set_of(&[
        JointRef::Leg1,
        JointRef::AntennaLeft,
    ])));
    *cmd.positions_mut() = positions;
    sim.inject_full(&cmd);

    let cycle = sim.step();
    assert_eq!(
        cycle.sample.present, wanted,
        "the named rows, and only them"
    );
}

#[test]
fn a_cycle_whose_replies_were_lost_says_so_and_the_machine_keeps_moving() {
    let mut sim = Sim::armed();
    let mut targets = stow_rows();
    targets[1] += 10.0;

    for _ in 0..(LAG + 1) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }

    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::DROP_REPLIES);
    cmd.set_count(3);
    sim.inject_full(&cmd);

    let mut last = None;
    for step in 0..3 {
        let cycle = sim.commanded_step(&targets, JOINT_MASK_ALL);
        assert!(
            !cycle.sample.present_valid,
            "cycle {step}: nothing was read this cycle"
        );
        assert_eq!(cycle.sample.missing, JOINT_MASK_ALL, "cycle {step}");
        last = Some(cycle.sample.present[1]);
    }

    let back = sim.commanded_step(&targets, JOINT_MASK_ALL);
    assert!(
        back.sample.present_valid,
        "the outage was three cycles long"
    );
    assert_eq!(back.sample.missing, 0);
    assert!(
        back.sample.present[1] > last.expect("three cycles ran"),
        "the machine kept moving while nobody could see it"
    );
}

#[test]
fn silence_de_torques_the_machine_and_is_announced_exactly_once() {
    let mut sim = Sim::armed();
    let mut targets = stow_rows();
    targets[1] += 0.05;
    sim.commanded_step(&targets, JOINT_MASK_ALL);

    // The dead-man's window is measured from the last accepted goal, so it
    // runs out one cycle past the timeout.
    let mut latched = Vec::new();
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 4) {
        let cycle = sim.step();
        if let Some(event) = cycle.event {
            latched.push((event, cycle.nominal));
        }
    }

    assert_eq!(latched.len(), 1, "one stall, one event");
    let (event, at) = latched[0];
    assert_eq!(event.kind, EventKind::HoldTimeoutTorqueOff);
    assert_eq!(event.time_ns, at);
    let silence = event.silence_ns;
    assert!(
        (HOLD_TIMEOUT..=HOLD_TIMEOUT + PERIOD).contains(&silence),
        "the event says how long the stream was silent, and it is the window it \
         ran past: {silence} ns",
    );
    assert!(
        at <= T0 + 2 * PERIOD + HOLD_TIMEOUT + PERIOD,
        "the dead-man fires within a cycle of its window running out, not at {at}",
    );

    let slot = sim.slot();
    let gate = sim.gate();
    assert!(gate.latched.get(), "the latch stands until a fresh arming");
    assert_eq!(slot.torqued, JointFlags::NONE);
    assert_eq!(slot.hold_timeouts, 1);

    let after = sim.step();
    assert!(
        after.sample.torque_off_latched,
        "a standing latch is visible in every sample"
    );
    assert!(
        after.event.is_none(),
        "a standing condition is not re-announced"
    );
}

#[test]
fn silence_before_the_first_goal_de_torques_too() {
    // The window a dead-man written around the held setpoint would miss: armed,
    // torqued, and the commander never said anything at all.
    let mut sim = Sim::new();
    sim.inject(SimOpWire::TORQUE_ON, all_joints());

    let mut events = Vec::new();
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 3) {
        if let Some(event) = sim.step().event {
            events.push(event);
        }
    }

    assert_eq!(events.len(), 1);
    assert_eq!(events[0].kind, EventKind::HoldTimeoutTorqueOff);
    let slot = sim.slot();
    let gate = sim.gate();
    assert!(!gate.has_held.get(), "nothing was ever commanded");
    assert_eq!(slot.torqued, JointFlags::NONE);
}

#[test]
fn arming_ends_a_latch_and_grants_a_fresh_window() {
    let mut sim = Sim::new();
    sim.inject(SimOpWire::TORQUE_ON, all_joints());
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 3) {
        sim.step();
    }
    assert!(
        sim.gate().latched.get(),
        "the case rests on a standing latch"
    );

    sim.inject(SimOpWire::TORQUE_ON, all_joints());
    let armed = sim.step();
    assert!(
        !armed.sample.torque_off_latched,
        "a fresh arming ends the latch"
    );
    assert_eq!(sim.slot().torqued, all_joints());

    // And the window is measured from the arming rather than from the stall it
    // ended, so nothing fires for another whole timeout.
    for _ in 0..(HOLD_TIMEOUT / PERIOD - 1) {
        assert!(sim.step().event.is_none());
    }
}

/// A machine energised again after the dead-man took it down stands still until
/// something commands it.
///
/// The hazard is uncommanded motion at torque-on: a gate or a plant that kept
/// the dead session's setpoint would move the machine to a position no live
/// commander asked for, in the whole window before the new commander's first
/// goal lands. Recovery is a fresh engagement, and nothing from before the
/// de-torquing is part of it.
#[test]
fn a_machine_re_armed_after_a_stall_moves_nothing_until_it_is_told_to() {
    let mut sim = Sim::armed();
    let mut targets = stow_rows();
    targets[1] += 10.0;
    for _ in 0..(LAG + 2) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }
    assert!(
        sim.step().sample.commanded_valid,
        "a setpoint is held, and the machine is well short of it"
    );

    // The stream stops. The machine slews on under the held setpoint until the
    // dead-man takes the torque away, and where it stood then is where it must
    // still stand when it is energised again.
    let mut stalled_at = stow_rows();
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 3) {
        stalled_at = sim.step().sample.present;
    }
    assert!(
        sim.gate().latched.get(),
        "the case rests on a standing latch"
    );

    sim.inject(SimOpWire::TORQUE_ON, all_joints());
    for step in 0..5 {
        let cycle = sim.step();
        assert!(!cycle.sample.torque_off_latched, "cycle {step}");
        assert_eq!(
            cycle.sample.present, stalled_at,
            "cycle {step}: the machine moved with nobody commanding it",
        );
        assert!(
            !cycle.sample.commanded_valid,
            "cycle {step}: the dead session's setpoint is not this session's",
        );
    }

    let mut fresh = stalled_at;
    fresh[1] += 0.05;
    for _ in 0..(LAG + 1) {
        sim.commanded_step(&fresh, JOINT_MASK_ALL);
    }
    let moving = sim.step();
    assert!(moving.sample.commanded_valid);
    assert_close(
        moving.sample.present[1],
        fresh[1],
        "the machine follows what this session asked for",
    );
}

/// A disarm that did not reach every servo is not a confirmed one: the rows
/// still energised are still being held, and forgetting the setpoint would stop
/// the gate rewriting to servos that are holding a position under torque.
#[test]
fn a_partial_disarm_keeps_the_setpoint_the_rest_of_the_machine_is_holding() {
    let mut sim = Sim::armed();
    let mut targets = stow_rows();
    targets[1] += 0.05;
    for _ in 0..(LAG + 2) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }
    assert!(sim.gate().has_held.get(), "something is being held");

    let antennas = set_of(&[JointRef::AntennaRight, JointRef::AntennaLeft]);
    sim.inject(SimOpWire::TORQUE_OFF, antennas);
    let cycle = sim.step();

    assert!(
        cycle.sample.commanded_valid,
        "the cranks are still holding what they were asked for"
    );
    assert_eq!(cycle.sample.commanded, targets);
    assert!(!cycle.sample.torque_off_latched, "and nothing latched");

    let slot = sim.slot();
    let gate = sim.gate();
    assert!(gate.has_held.get());
    assert_eq!(
        slot.torqued,
        flags::without(all_joints(), antennas),
        "the named rows went off and no others"
    );
    assert_eq!(
        slot.has_target,
        all_joints(),
        "and every row still has the setpoint it was given"
    );
}

#[test]
fn a_confirmed_disarm_forgets_the_setpoint_without_latching() {
    let mut sim = Sim::armed();
    let mut targets = stow_rows();
    targets[1] += 0.05;
    for _ in 0..(LAG + 2) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }
    assert!(sim.gate().has_held.get(), "something is being held");

    sim.inject(SimOpWire::TORQUE_OFF, all_joints());
    let cycle = sim.step();
    assert!(
        !cycle.sample.torque_off_latched,
        "a deliberate disarm is not a fault"
    );
    assert!(
        !cycle.sample.commanded_valid,
        "nothing is being held any more"
    );

    let slot = sim.slot();
    let gate = sim.gate();
    assert_eq!(slot.torqued, JointFlags::NONE);
    assert_eq!(slot.has_target, JointFlags::NONE);
    assert!(!gate.latched.get());
}

#[test]
fn a_sender_overrunning_the_queue_is_dropped_and_told() {
    let mut sim = Sim::armed();
    let targets = stow_rows();

    // More goals in one cycle than the queue holds, all stamped far enough
    // ahead that none of them becomes due and drains a slot.
    for i in 0..(queue_depth() as i64 + 2) {
        let goal = setpoint(sim.now + (10 + i) * PERIOD, JOINT_MASK_ALL, targets);
        sim.send_goal(&goal);
    }
    let cycle = sim.step();

    let event = cycle.event.expect("the overrun is reported");
    assert_eq!(event.kind, EventKind::GoalDroppedQueueFull);
    assert_eq!(
        event.count,
        queue_depth() as u32,
        "and says how deep the queue was when it hit it"
    );
    assert_eq!(
        event.silence_ns, 0,
        "a queue overrun is not a lateness, so the field it does not name is zero"
    );
    let slot = sim.slot();
    let gate = sim.gate();
    assert_eq!(slot.goals_dropped, 2);
    assert_eq!(
        slot.events_dropped, 1,
        "two drops, one slot to report them in"
    );
    assert_eq!(gate.queue.len(), queue_depth());
}

#[test]
fn a_goal_stamped_for_an_instant_already_past_is_taken_and_warned() {
    let mut sim = Sim::armed();
    let goal = setpoint(T0 - PERIOD, JOINT_MASK_ALL, stow_rows());
    sim.send_goal(&goal);
    let cycle = sim.step();

    let event = cycle.event.expect("a stale goal is remarked on");
    assert_eq!(event.kind, EventKind::GoalStaleOrOutOfOrder);
    assert_eq!(
        event.silence_ns,
        2 * PERIOD,
        "and says how far past its instant it arrived",
    );
    assert!(
        cycle.sample.commanded_valid,
        "and executed all the same, in arrival order"
    );
}

/// A goal stamped ahead of now but behind the one before it has missed nothing
/// yet, so what it is late by is nothing. The silence a stale goal carries is
/// how far past its instant it arrived, and this is the other situation the
/// same outcome covers.
#[test]
fn a_goal_merely_out_of_order_says_it_is_late_by_nothing() {
    let mut sim = Sim::armed();
    let targets = stow_rows();
    let later = setpoint(sim.now + 5 * PERIOD, JOINT_MASK_ALL, targets);
    let earlier = setpoint(sim.now + 3 * PERIOD, JOINT_MASK_ALL, targets);
    sim.send_goal(&later);
    sim.send_goal(&earlier);
    let cycle = sim.step();

    let event = cycle.event.expect("the reordering is remarked on");
    assert_eq!(event.kind, EventKind::GoalStaleOrOutOfOrder);
    assert_eq!(
        event.silence_ns, 0,
        "a goal whose instant is still ahead is late by nothing"
    );
    assert_eq!(
        sim.gate().queue.len(),
        2,
        "and both are queued, in arrival order"
    );
}

/// Two events on one cycle, and the slot goes to the one about the machine.
///
/// The queue is kept full so every cycle drops a goal and says so; a dropped
/// goal is not liveness, so the dead-man runs out underneath that stream and
/// one cycle raises both. The machine de-torquing itself is what an operator
/// has to see, and the goal event is the one that is counted instead.
#[test]
fn the_dead_mans_latch_takes_the_slot_from_a_goal_event_raised_first() {
    let mut sim = Sim::armed();
    let targets = stow_rows();
    // Far enough ahead that none of them ever becomes due and drains a slot.
    fn overrun(sim: &mut Sim, targets: &[f64; JOINT_COUNT]) {
        let goal = setpoint(sim.now + 1_000 * PERIOD, JOINT_MASK_ALL, *targets);
        sim.send_goal(&goal);
    }
    for _ in 0..queue_depth() {
        overrun(&mut sim, &targets);
    }
    sim.step();
    assert_eq!(sim.gate().queue.len(), queue_depth());

    let mut latches = Vec::new();
    let mut dropped_before_latch = 0;
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 4) {
        let before = sim.slot().events_dropped;
        overrun(&mut sim, &targets);
        let cycle = sim.step();
        let after = sim.slot().events_dropped;
        match cycle.event {
            Some(event) if event.kind == EventKind::HoldTimeoutTorqueOff => {
                assert_eq!(
                    after,
                    before + 1,
                    "the goal event it displaced is counted, not lost quietly"
                );
                latches.push(event);
            }
            Some(event) => {
                assert_eq!(event.kind, EventKind::GoalDroppedQueueFull);
                assert_eq!(after, before, "one event, one slot, nothing displaced");
                if latches.is_empty() {
                    dropped_before_latch += 1;
                }
            }
            None => {}
        }
    }

    assert_eq!(latches.len(), 1, "one stall, one latch, announced once");
    assert!(
        dropped_before_latch > 0,
        "the case rests on goal events being raised on the way there"
    );
    assert!(sim.gate().latched.get());
}

/// An injection this build cannot read does not reach the machine at all -- and
/// the part of a torque-on that is not about its mask is the part that matters:
/// ending a torque-off latch. A refusal that still ended one would clear the
/// state the latch exists to hold, and the counter would say the injection did
/// nothing.
#[test]
fn a_refused_arming_does_not_end_a_latch() {
    let mut sim = Sim::new();
    sim.inject(SimOpWire::TORQUE_ON, all_joints());
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 3) {
        sim.step();
    }
    assert!(
        sim.gate().latched.get(),
        "the case rests on a standing latch"
    );

    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::TORQUE_ON);
    cmd.set_mask(JointFlagsWire(1 << JOINT_COUNT));
    sim.inject_full(&cmd);
    let cycle = sim.step();

    assert!(
        cycle.sample.torque_off_latched,
        "a set nobody could read energises nothing and ends nothing"
    );
    let slot = sim.slot();
    let gate = sim.gate();
    assert!(gate.latched.get());
    assert_eq!(slot.torqued, JointFlags::NONE);
    assert_eq!(
        slot.refused_injections, 1,
        "counted once for the one injection"
    );
}

/// An injection is refused whole at the one reading of it, whichever of its
/// fields this build could not read. Dropping replies acts on no servo, but a
/// scenario that named servos this build does not know was written against a
/// different machine, and carrying out the readable half of such a message is
/// how a scenario runs green with part of its hand on the machine discarded.
#[test]
fn an_injection_naming_servos_this_build_does_not_know_is_refused_whole() {
    let mut sim = Sim::armed();
    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::DROP_REPLIES);
    cmd.set_mask(JointFlagsWire(1 << JOINT_COUNT));
    cmd.set_count(2);
    sim.inject_full(&cmd);

    let cycle = sim.step();
    assert!(cycle.sample.present_valid, "no outage was carried out");
    assert_eq!(
        sim.slot().refused_injections,
        1,
        "and the refusal was counted"
    );
}

/// The same operation with a set this build can read is carried out.
#[test]
fn an_injection_this_build_can_read_whole_is_carried_out() {
    let mut sim = Sim::armed();
    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::DROP_REPLIES);
    cmd.set_count(2);
    sim.inject_full(&cmd);

    let cycle = sim.step();
    assert!(!cycle.sample.present_valid, "the outage happened");
    assert_eq!(sim.slot().refused_injections, 0);
}

/// A slot this build cannot read is cleared and counted, and the run's totals
/// survive the clear: they are plain numbers whatever the bytes around them
/// say, and a scenario checker reading a total must not see it restart because
/// one field of the slot was damaged.
#[test]
fn a_state_this_build_cannot_read_is_cleared_and_counted() {
    let mut sim = Sim::armed();
    let mut goal = setpoint(T0 + PERIOD, JOINT_MASK_ALL, stow_rows());
    goal.set_mask(JointFlagsWire(1 << 15));
    sim.send_goal(&goal);
    sim.step();
    assert_eq!(
        sim.slot().refused_goals,
        1,
        "the case rests on a total to lose"
    );

    // A bit above the ninth bus row, which no build of this cog wrote: what a
    // slot written by a machine with more servos would look like from here.
    sim.cog
        .state_sim_mut()
        .set_torqued(JointFlagsWire(1 << JOINT_COUNT));
    let cycle = sim.step();

    let slot = sim.slot();
    assert_eq!(slot.refused_state_fields, 1, "counted once");
    assert_eq!(slot.refused_goals, 1, "and the totals survive the clear");
    assert!(
        slot.initialized.get(),
        "the run starts again from a machine this build can describe"
    );
    assert_eq!(rows_of(&slot.positions), stow_rows());
    assert_eq!(
        slot.torqued,
        JointFlags::NONE,
        "and it comes back de-torqued: the arming was in the bytes that were refused"
    );
    assert!(cycle.sample.present_valid, "and the cycle still reported");
}

/// The last status record a run of cycles carried, where one of them did.
///
/// Which cycle of a window publishes it depends on where the cadence's count
/// stood when the window opened, and a case about what a copy says is not a case
/// about that.
fn newest_status(cycles: Vec<Cycle>) -> Option<Status> {
    cycles.into_iter().rev().find_map(|cycle| cycle.status)
}

/// A restart from a slot this build could not read still reports the whole run.
///
/// The record says of itself that it is cumulative since the process started, so
/// a reader that has seen one copy has read the run -- and the analyzer reads
/// the newest copy. A restart that took the counters back to zero would make
/// every copy after it undercount in the one direction the drift check forgives,
/// so a log that really did lose messages would balance. The two first-instants
/// go the same way: they name when something first happened in the run, and the
/// survey ordering is decided from them.
///
/// The sweep instant is the exception, and is asserted as one: the restart runs
/// the release again, so what it reports is when this state's release ran.
#[test]
fn a_restart_from_a_refused_slot_still_reports_the_whole_run() {
    let mut sim = Sim::new();
    sim.keep_alive();
    let first = sim.step().status.expect("the first cycle publishes one");

    let cadence = usize::try_from(1_000_000_000 / PERIOD).expect("a second of cycles");
    let before = newest_status(sim.quiet(cadence)).expect("a copy a simulated second on");
    assert!(before.cycles > 1, "the case rests on a run worth losing");

    // A bit above the ninth bus row, which no build of this cog wrote: what a
    // slot written by a machine with more servos would look like from here.
    sim.cog
        .state_sim_mut()
        .set_torqued(JointFlagsWire(1 << JOINT_COUNT));
    let restart = sim.step();
    let swept_again = sim.now;
    assert!(
        restart.status.is_none(),
        "a restart is not a process start: the run's first copy already went out",
    );

    let after = newest_status(sim.quiet(cadence)).expect("a copy a simulated second on");

    assert_eq!(sim.slot().refused_state_fields, 1, "counted once");
    assert_eq!(
        after.sweep_time_ns, swept_again,
        "the release ran again, and the record says when",
    );
    assert_eq!(
        (after.first_pose_ns, after.first_session_cmd_ns),
        (first.first_pose_ns, first.first_session_cmd_ns),
        "the run's first sample and first datagram are still the run's first",
    );
    for (name, then, now) in [
        ("cycles", before.cycles, after.cycles),
        ("skipped", before.skipped, after.skipped),
        ("session_cmds", before.session_cmds, after.session_cmds),
        ("taken", before.taken, after.taken),
        ("published", before.published, after.published),
        (
            "health_reports",
            before.health_reports,
            after.health_reports,
        ),
        ("health_misses", before.health_misses, after.health_misses),
        (
            "aux_duplicates",
            before.aux_duplicates,
            after.aux_duplicates,
        ),
        ("blind_cycles", before.blind_cycles, after.blind_cycles),
        (
            "confirm_misses",
            before.confirm_misses,
            after.confirm_misses,
        ),
    ] {
        assert!(
            now >= then,
            "{name} went backwards across the restart: {then} then {now}",
        );
    }
    assert!(
        after.cycles > before.cycles,
        "and the run carried on counting from where it was",
    );
}

/// A slot damaged while the dead-man holds the machine off does not re-energise
/// it. The latch is in the bytes the cycle refused, so the restart cannot know
/// whether it stands -- and a modelled machine that torqued itself out of a
/// memory fault would certify the one transition the latch exists to prevent.
#[test]
fn a_slot_damaged_while_the_dead_man_is_latched_comes_back_de_torqued() {
    let mut sim = Sim::armed();
    let mut targets = stow_rows();
    targets[1] += 0.05;
    sim.commanded_step(&targets, JOINT_MASK_ALL);
    for _ in 0..(HOLD_TIMEOUT / PERIOD + 4) {
        sim.step();
    }
    assert!(
        sim.gate().latched.get(),
        "the case rests on a standing latch"
    );
    assert_eq!(sim.slot().torqued, JointFlags::NONE);

    sim.cog
        .state_sim_mut()
        .set_torqued(JointFlagsWire(1 << JOINT_COUNT));
    let cycle = sim.step();

    let slot = sim.slot();
    assert_eq!(slot.refused_state_fields, 1);
    assert_eq!(
        slot.torqued,
        JointFlags::NONE,
        "nothing re-energises the machine but an arming"
    );
    assert!(
        !cycle.sample.torque_off_latched,
        "and the latch itself is gone with the bytes, which is why the torque is off"
    );
}

#[test]
fn a_goal_naming_servos_this_build_does_not_know_is_counted_and_nothing_else() {
    let mut sim = Sim::armed();
    let mut goal = setpoint(T0 + PERIOD, JOINT_MASK_ALL, stow_rows());
    // A bit above the ninth bus row: a set this build cannot read, which is
    // what a machine with more servos than this one would publish.
    goal.set_mask(JointFlagsWire(1 << 15));
    sim.send_goal(&goal);

    let cycle = sim.step();
    assert!(
        cycle.event.is_none(),
        "a set of servos this build cannot read names nobody to report about"
    );
    assert!(!cycle.sample.commanded_valid, "nothing was queued");
    let slot = sim.slot();
    assert_eq!(slot.refused_goals, 1);
    assert_eq!(slot.goals_dropped, 0, "a drop is a different thing");
}

/// Events are sparse and the channel is not a queue, so two of them separated
/// by quiet cycles are two published messages, each naming the cycle it is
/// about. Neither is lost behind the quiet between them, and neither is the
/// other one read twice.
#[test]
fn two_events_separated_by_quiet_cycles_each_name_their_own_cycle() {
    let mut sim = Sim::armed();
    let stale = setpoint(T0 - PERIOD, JOINT_MASK_ALL, stow_rows());
    sim.send_goal(&stale);
    let first = sim.step();
    let first_event = first.event.expect("the first event");
    assert_eq!(first_event.time_ns, first.nominal);

    for cycle in sim.quiet(5) {
        assert!(cycle.event.is_none(), "nothing happened on a quiet cycle");
    }
    let later = setpoint(sim.now - PERIOD, JOINT_MASK_ALL, stow_rows());
    sim.send_goal(&later);
    let second = sim.step();
    let second_event = second.event.expect("the second event");
    assert_eq!(second_event.time_ns, second.nominal);
    assert!(
        second_event.time_ns > first_event.time_ns,
        "and the two are five quiet cycles apart, not one message read twice"
    );
}

#[test]
fn a_late_cycle_makes_up_whole_cycles_of_motion_and_no_more() {
    let mut sim = Sim::armed();
    // Further than any catch-up covers, so every cycle asserted here is a
    // full-rate one and the rate is what is being counted.
    let mut targets = stow_rows();
    targets[1] += 100.0;
    for _ in 0..(LAG + 2) {
        sim.commanded_step(&targets, JOINT_MASK_ALL);
    }

    let on_time = sim.commanded_step(&targets, JOINT_MASK_ALL);
    let late = sim.commanded_step_by(&targets, JOINT_MASK_ALL, 5);
    assert_close(
        late.sample.present[1] - on_time.sample.present[1],
        5.0 * SLEW_LEGS,
        "five cycles' worth of motion for five cycles of lost time",
    );

    let very_late = sim.commanded_step_by(&targets, JOINT_MASK_ALL, 20);
    assert_close(
        very_late.sample.present[1] - late.sample.present[1],
        MAX_CATCHUP_CYCLES * SLEW_LEGS,
        "and no further than the clamp, whatever the gap was",
    );
}

#[test]
fn an_injection_this_build_cannot_carry_out_is_counted_and_does_nothing() {
    let mut sim = Sim::armed();

    // A set naming a tenth bus row, which no machine has: refused rather than
    // masked down to the nine it does have.
    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::TORQUE_OFF);
    cmd.set_mask(JointFlagsWire(1 << JOINT_COUNT));
    sim.inject_full(&cmd);
    sim.step();

    let slot = sim.slot();
    assert_eq!(slot.refused_injections, 1);
    assert_eq!(
        slot.torqued,
        all_joints(),
        "nothing was de-torqued by a set nobody could read"
    );
    assert_eq!(
        slot.refused_state_fields, 0,
        "an injection is not a field of this cog's own slot"
    );

    // An operation this build does not know, which is what a scenario written
    // against a newer vocabulary sends.
    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire(200));
    cmd.set_mask(JointFlagsWire::from(all_joints()));
    sim.inject_full(&cmd);
    sim.step();

    let slot = sim.slot();
    assert_eq!(slot.refused_injections, 2);
    assert_eq!(slot.torqued, all_joints());
}

/// A cycle that is not the cycle this cog is woken on is a scenario describing
/// a bus nobody built. Refused on the first execution, where it is a
/// configuration mistake, rather than modelled into plausible numbers.
///
/// What the refusal said is on stderr rather than in the panic these cases
/// see: a panic in a cog body crosses the generated C++ boundary and reaches
/// the wrapper as an exception, which re-panics in its own words.
#[test]
#[should_panic(expected = "MotorSim test wrapper: execute() failed")]
fn a_scenario_whose_cycle_is_not_the_bus_cycle_is_refused() {
    let mut sim = Sim::with_params(|params| params.period_ns = PERIOD / 2);
    sim.step();
}

/// The same for a slew that is not a distance: an unset rate leaves a machine
/// that never moves, and a negative one is nobody's intent.
#[test]
#[should_panic(expected = "MotorSim test wrapper: execute() failed")]
fn a_scenario_whose_servos_have_no_rate_is_refused() {
    let mut sim = Sim::with_params(|params| params.slew_antennas_rad = 0.0);
    sim.step();
}

/// And the same for a dead-man that allows no silence at all: an unset
/// `hold_timeout_ns` is zero, and a machine that de-torques itself on the first
/// cycle it holds through passes every dead-man assertion a scenario could make
/// while failing every other one somewhere unrelated.
#[test]
#[should_panic(expected = "MotorSim test wrapper: execute() failed")]
fn a_scenario_whose_dead_man_allows_no_silence_is_refused() {
    let mut sim = Sim::with_params(|params| params.hold_timeout_ns = 0);
    sim.step();
}

/// How many goals the gate's queue holds -- the schema's own depth, which is
/// the only place the number is written.
fn queue_depth() -> usize {
    SimStateWire::new().gate().queue().capacity()
}

/// A set holding one joint.
fn one(joint: JointRef) -> JointFlags {
    set_of(&[joint])
}

/// A set holding those joints.
fn set_of(joints: &[JointRef]) -> JointFlags {
    let mut set = JointFlags::NONE;
    for joint in joints {
        flags::insert(&mut set, *joint);
    }
    set
}

/// Assert two angles are the same to within a band far tighter than any step
/// this plant takes.
fn assert_close(found: f64, wanted: f64, what: &str) {
    assert!(
        (found - wanted).abs() < 1e-9,
        "{what}: {found} is not {wanted}"
    );
}

/// Every servo on the bus.
fn all_joints() -> JointFlags {
    JointFlagsWire(JOINT_MASK_ALL)
        .to_known()
        .expect("every bus row is a bus row")
}

/// The provisioning is in the control tables from the first cycle, because that
/// is what a commissioning sequence reads and it reads it over the bus.
#[test]
fn the_modelled_servos_come_up_correctly_provisioned() {
    let mut sim = Sim::new();
    sim.step();

    let regs = sim.slot().regs;
    for (row, joint) in joints::ROWS.into_iter().enumerate() {
        assert_eq!(
            sim_regs::read(&regs, row, RegId::ModelNumber),
            Ok(value::u16(EXPECTED_MODELS[row])),
        );
        assert_eq!(
            sim_regs::read(&regs, row, RegId::OperatingMode),
            Ok(value::u8(EXPECTED_OPERATING_MODES[row])),
        );
        assert_eq!(
            sim_regs::read(&regs, row, RegId::HomingOffset),
            Ok(value::i32(VENDOR_HOMING_OFFSETS[row])),
        );
        assert_eq!(
            sim_regs::read(&regs, row, RegId::PositionGains),
            Ok(DEFAULT_GAINS.for_joint(joint).value()),
        );
        assert_eq!(
            sim_regs::read(&regs, row, RegId::HardwareErrorStatus),
            Ok(value::u8(0)),
            "nothing is complaining about row {row}",
        );
        assert!(
            sim_regs::read(&regs, row, RegId::PresentInputVoltage)
                .expect("a rail reading")
                .as_volts()
                .is_some_and(|volts| volts > DEFAULT_MIN_ARM_VOLTAGE),
            "the modelled rail is up at row {row}",
        );
    }
}

/// The sample is a read of the present-position registers, so the two agree by
/// construction rather than by two paths out of the plant staying in step.
#[test]
fn the_live_registers_are_what_the_sample_reports() {
    let mut sim = Sim::new();
    sim.inject(SimOpWire::TORQUE_ON, flags::all());
    let mut targets = stow_rows();
    targets[1] += 0.05;
    // Four cycles, because a well-formed goal names an instant two cycles out:
    // the register holds what has been written to the servos, not what is queued.
    let mut cycle = sim.commanded_step(&targets, JOINT_MASK_ALL);
    for _ in 0..3 {
        cycle = sim.commanded_step(&targets, JOINT_MASK_ALL);
    }

    let regs = sim.slot().regs;
    assert_eq!(sim_regs::present_rows(&regs), cycle.sample.present);
    assert_eq!(
        sim_regs::read(&regs, 1, RegId::GoalPosition),
        Ok(value::radians(targets[1])),
        "the goal register holds what the servo was asked for",
    );
    for row in 0..JOINT_COUNT {
        assert_eq!(
            sim_regs::read(&regs, row, RegId::TorqueEnable),
            Ok(value::u8(1)),
            "row {row} is energised and says so",
        );
    }
}

/// The dead-man's sweep reaches the torque-enable registers, which is what a
/// confirmation read of them will find.
#[test]
fn a_de_torqued_row_says_so_in_its_torque_enable_register() {
    let mut sim = Sim::new();
    sim.inject(SimOpWire::TORQUE_ON, flags::all());
    sim.commanded_step(&stow_rows(), JOINT_MASK_ALL);
    sim.quiet((HOLD_TIMEOUT / PERIOD + 1) as usize);

    assert!(sim.gate().latched.get(), "the dead-man latched");
    let regs = sim.slot().regs;
    for row in 0..JOINT_COUNT {
        assert_eq!(
            sim_regs::read(&regs, row, RegId::TorqueEnable),
            Ok(value::u8(0)),
            "row {row} is off",
        );
    }
}

/// A scenario's hand on a register: the one injection that stands in for a servo
/// whose error byte latched or whose rail sagged.
#[test]
fn an_injection_writes_the_register_it_names() {
    let mut sim = Sim::new();
    sim.step();

    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::SET_REGISTER);
    cmd.set_mask(JointFlagsWire::from(flags::bit(JointRef::Leg2)));
    cmd.set_reg(RegIdWire::HARDWARE_ERROR_STATUS);
    cmd.set_value(0x20);
    sim.inject_full(&cmd);
    sim.step();

    let regs = sim.slot().regs;
    assert_eq!(
        sim_regs::read(&regs, 3, RegId::HardwareErrorStatus),
        Ok(value::u8(0x20)),
        "the third leg is complaining",
    );
    assert_eq!(
        sim_regs::read(&regs, 2, RegId::HardwareErrorStatus),
        Ok(value::u8(0)),
        "and nothing else is",
    );
    assert_eq!(sim.slot().refused_injections, 0);
}

/// The registers the plant owns are not an injection's to write: this cycle's
/// proprioception writes them from the modelled machine whatever else put a
/// number there, so an accepted write would be a scenario's premise lost in
/// silence. Refused and counted instead, per row named.
///
/// Torque enable is the one that matters: a scenario faking a de-torqued row
/// here would leave the plant energised and the checker asserting about a
/// machine that was never off.
#[test]
fn an_injection_naming_a_register_the_plant_owns_is_refused() {
    let mut sim = Sim::new();
    sim.inject(SimOpWire::TORQUE_ON, flags::bit(JointRef::AntennaRight));
    sim.step();

    for reg in [
        RegIdWire::TORQUE_ENABLE,
        RegIdWire::PRESENT_POSITION,
        RegIdWire::GOAL_POSITION,
    ] {
        let mut cmd = SimCmdWire::new();
        cmd.set_op(SimOpWire::SET_REGISTER);
        cmd.set_mask(JointFlagsWire::from(flags::bit(JointRef::AntennaRight)));
        cmd.set_reg(reg);
        cmd.set_value(0);
        sim.inject_full(&cmd);
        sim.step();
    }

    let regs = sim.slot().regs;
    assert_eq!(
        sim_regs::read(&regs, 7, RegId::TorqueEnable),
        Ok(value::u8(1)),
        "the antenna is still energised, and its register still says so",
    );
    assert!(
        flags::contains(sim.slot().torqued, JointRef::AntennaRight),
        "and the plant was never touched",
    );
    assert_eq!(
        sim.slot().refused_injections,
        3,
        "one refusal per write, counted where every other refused injection is",
    );
}

/// A non-volatile register takes a write from a de-torqued servo and not from an
/// energised one, which is the rule the real bus enforces before anything goes
/// out. The refusal is per row, so a mask spanning both lands on half of it.
#[test]
fn a_non_volatile_write_is_refused_by_the_rows_holding_torque() {
    let mut sim = Sim::new();
    sim.inject(SimOpWire::TORQUE_ON, flags::bit(JointRef::AntennaRight));
    sim.step();

    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::SET_REGISTER);
    cmd.set_mask(JointFlagsWire::from(
        flags::bit(JointRef::AntennaRight) | flags::bit(JointRef::AntennaLeft),
    ));
    cmd.set_reg(RegIdWire::OPERATING_MODE);
    cmd.set_value(3);
    sim.inject_full(&cmd);
    sim.step();

    let regs = sim.slot().regs;
    assert_eq!(
        sim_regs::read(&regs, 7, RegId::OperatingMode),
        Ok(value::u8(EXPECTED_OPERATING_MODES[7])),
        "the energised antenna ignored it",
    );
    assert_eq!(
        sim_regs::read(&regs, 8, RegId::OperatingMode),
        Ok(value::u8(3)),
        "the de-torqued one took it",
    );
    assert_eq!(
        sim.slot().refused_injections,
        1,
        "one row refused, and counted",
    );
}

/// One transaction, as the host's channel carries it.
///
/// Written in the terms the cases reason in -- which op, which servo, which
/// register, what value -- and mapped into the schema's vocabulary once, so a
/// case says what it means and only this function knows which field is which.
fn transaction(
    op: AuxOpKindWire,
    id: u8,
    reg: RegIdWire,
    value: Option<value::Value>,
) -> BusTxnWire {
    let mut txn = BusTxnWire::new();
    let held = txn.clear_valid();
    held.active = true.into();
    held.op = op
        .to_known()
        .expect("the cases name transactions this build knows");
    held.id = id;
    held.reg = reg
        .to_known()
        .expect("the cases name registers this build knows");
    if let Some(held_value) = value {
        held.value_kind = ValueShapeWire::from(held_value.shape())
            .to_known()
            .expect("a value's shape is a shape");
        held.value = held_value.bits();
    }
    txn
}

/// A read of a provisioned cell comes back as what the unit holds, and a ping
/// comes back as what the servo says it is.
#[test]
fn the_bus_answers_a_read_and_a_ping_out_of_the_control_tables() {
    let mut sim = Sim::new();
    sim.step();

    sim.transact(
        1,
        &transaction(AuxOpKindWire::READ_REG, 17, RegIdWire::OPERATING_MODE, None),
    );
    let cycle = sim.step();
    let outcome = cycle.outcome.expect("the transaction was answered");
    assert_eq!(outcome.corr, 1);
    assert_eq!(outcome.status, AuxStatus::Ok);
    assert_eq!(
        outcome.value,
        value::u8(EXPECTED_OPERATING_MODES[7]).bits(),
        "the right antenna is provisioned for extended position",
    );

    sim.transact(
        2,
        &transaction(AuxOpKindWire::PING, 17, RegIdWire::NONE, None),
    );
    let cycle = sim.step();
    let outcome = cycle.outcome.expect("the ping was answered");
    assert_eq!(outcome.status, AuxStatus::Ok);
    assert_eq!(outcome.model, EXPECTED_MODELS[7]);
    assert_eq!(outcome.value, 0, "a ping names no register");
}

/// Nothing on this bus holds that id, so nothing answers. A timeout and not a
/// refusal: the datagram went out and the window closed on silence.
#[test]
fn a_transaction_addressed_off_the_bus_times_out() {
    let mut sim = Sim::new();
    sim.step();

    sim.transact(
        7,
        &transaction(AuxOpKindWire::PING, 99, RegIdWire::NONE, None),
    );
    let outcome = sim.step().outcome.expect("the host is answered either way");
    assert_eq!(outcome.corr, 7);
    assert_eq!(outcome.status, AuxStatus::Timeout);
}

/// A verified torque-enable write is the whole of what arming this machine is:
/// the row energises, the register says so, the driver believes it, and the
/// dead-man's window runs from the write.
#[test]
fn a_verified_torque_enable_write_arms_the_row_and_the_belief() {
    let mut sim = Sim::new();
    sim.step();

    sim.transact(
        3,
        &transaction(
            AuxOpKindWire::WRITE_REG_VERIFIED,
            18,
            RegIdWire::TORQUE_ENABLE,
            Some(value::u8(1)),
        ),
    );
    let cycle = sim.step();
    let outcome = cycle.outcome.expect("the write was answered");
    assert_eq!(outcome.status, AuxStatus::Ok);
    assert_eq!(
        outcome.value,
        value::u8(1).bits(),
        "the answer is the read-back and not the write",
    );

    let slot = sim.slot();
    assert!(
        flags::contains(slot.torqued, JointRef::AntennaLeft),
        "the plant is energised",
    );
    assert_eq!(
        slot.aux.believed_torqued,
        JointFlags::ANTENNA_LEFT,
        "and the driver believes exactly that row",
    );
    assert_eq!(
        slot.gate.last_accept.as_nanos(),
        cycle.nominal,
        "arming grants a fresh hold-timeout window",
    );
    assert_eq!(
        sim_regs::read(&slot.regs, 8, RegId::TorqueEnable),
        Ok(value::u8(1)),
    );
}

/// A verified goal-position write reaches the plant, because a goal register
/// holds what it was written whether or not the row is energised -- which is
/// why an engagement writes the goals before it enables anything.
#[test]
fn a_verified_goal_write_reaches_a_limp_servo() {
    let mut sim = Sim::new();
    sim.step();
    let target = stow_rows()[8] + 0.25;

    sim.transact(
        4,
        &transaction(
            AuxOpKindWire::WRITE_REG_VERIFIED,
            18,
            RegIdWire::GOAL_POSITION,
            Some(value::radians(target)),
        ),
    );
    let outcome = sim.step().outcome.expect("the write was answered");
    assert_eq!(outcome.status, AuxStatus::Ok);
    assert_eq!(outcome.value, value::radians(target).bits());

    let slot = sim.slot();
    assert_eq!(rows_of(&slot.targets)[8], target);
    assert!(
        flags::contains(slot.has_target, JointRef::AntennaLeft),
        "the row has been commanded, and will move when it is energised",
    );
    assert_eq!(
        rows_of(&slot.positions)[8],
        stow_rows()[8],
        "and it has not moved, because nothing is holding it",
    );
}

/// The pin sweep's write, which asks for no read-back: the register takes it and
/// the answer carries nothing at all. The modelled driver has to answer that
/// transaction the way the real one does, because the sweep is the one place a
/// sequence issues it.
#[test]
fn an_unverified_goal_write_reaches_the_servo_and_answers_with_no_value() {
    let mut sim = Sim::new();
    sim.step();
    let target = stow_rows()[8] + 0.25;

    sim.transact(
        6,
        &transaction(
            AuxOpKindWire::WRITE_REG,
            18,
            RegIdWire::GOAL_POSITION,
            Some(value::radians(target)),
        ),
    );
    let outcome = sim.step().outcome.expect("the write was answered");
    assert_eq!(outcome.status, AuxStatus::Ok);
    assert_eq!(
        outcome.value_kind,
        ValueShapeWire::NONE,
        "an acknowledgement is not a reading of anything",
    );
    assert_eq!(outcome.value, 0);

    let slot = sim.slot();
    assert_eq!(
        rows_of(&slot.targets)[8],
        target,
        "the write reached the cell, which is the half that does not change",
    );
    assert!(flags::contains(slot.has_target, JointRef::AntennaLeft));
    assert_eq!(
        sim_regs::read(&slot.regs, 8, RegId::GoalPosition),
        Ok(value::radians(target)),
    );
}

/// An unverified torque-enable write energises the plant and moves nothing the
/// driver believes: the answer is the driver's own send, and a dead-man measured
/// against that would be watching the driver rather than the machine. The real
/// driver refuses the credit for exactly that reason, and a simulator that
/// granted it would let a test of the dead-man pass against a driver that does
/// the opposite.
#[test]
fn an_unverified_torque_write_energises_the_plant_and_not_the_belief() {
    let mut sim = Sim::new();
    sim.step();
    assert_eq!(
        sim.slot().aux.believed_torqued,
        JointFlags::NONE,
        "nothing is believed holding to begin with",
    );
    // A standing torque-off latch, which is the other thing an arming ends.
    let mut off = SessionCmdWire::new();
    off.set_kind(SessionCmdKindWire::TORQUE_OFF_NOW);
    sim.ask(&off);
    sim.step();
    assert!(sim.gate().latched.get());

    sim.transact(
        3,
        &transaction(
            AuxOpKindWire::WRITE_REG,
            18,
            RegIdWire::TORQUE_ENABLE,
            Some(value::u8(1)),
        ),
    );
    let cycle = sim.step();
    assert_eq!(
        cycle.outcome.expect("the write was answered").status,
        AuxStatus::Ok,
        "the write is admitted -- it is the belief that is not earned",
    );

    let slot = sim.slot();
    assert!(
        flags::contains(slot.torqued, JointRef::AntennaLeft),
        "the servo took the instruction, so the plant is energised",
    );
    assert_eq!(
        slot.aux.believed_torqued,
        JointFlags::NONE,
        "and the driver believes nothing, because nothing was read back",
    );
    assert!(
        slot.gate.latched.get(),
        "and the latch stands, because nothing read back says the row is holding",
    );
}

/// A non-volatile register takes no write from an energised row, over the bus
/// as much as from an injection: the real bus refuses it outright because a
/// servo ignores such a write and acknowledges it anyway.
#[test]
fn a_non_volatile_write_over_the_bus_is_refused_under_torque() {
    let mut sim = Sim::new();
    sim.inject(SimOpWire::TORQUE_ON, flags::bit(JointRef::AntennaRight));
    sim.step();

    sim.transact(
        5,
        &transaction(
            AuxOpKindWire::WRITE_REG_VERIFIED,
            17,
            RegIdWire::OPERATING_MODE,
            Some(value::u8(3)),
        ),
    );
    let outcome = sim.step().outcome.expect("the host is answered either way");
    assert_eq!(outcome.status, AuxStatus::Refused);
    assert_eq!(
        sim_regs::read(&sim.slot().regs, 7, RegId::OperatingMode),
        Ok(value::u8(EXPECTED_OPERATING_MODES[7])),
        "and nothing was written",
    );
}

/// The host that fills the slot is serial by construction, so a second request
/// while one is pending is a host that is not what it claims to be. The refusal
/// is loud both ways: an outcome against the turned-away request's own number,
/// and a count.
#[test]
fn a_second_transaction_in_one_cycle_is_refused_against_its_own_number() {
    let mut sim = Sim::new();
    sim.step();

    let txn = transaction(AuxOpKindWire::PING, 10, RegIdWire::NONE, None);
    sim.transact(11, &txn);
    sim.transact(12, &txn);
    let cycle = sim.step();

    let outcome = cycle.outcome.expect("the refusal is an answer");
    assert_eq!(outcome.corr, 12, "the second request is the refused one");
    assert_eq!(
        outcome.status,
        AuxStatus::Busy,
        "the slot was full, which is not a decline of the transaction"
    );
    assert_eq!(sim.slot().aux_refused, 1);

    // The first request was accepted and run; its answer lost the cycle's one
    // outcome slot to the refusal. The real driver keeps both and publishes
    // both, so this is the simulated host's single slot showing and not a
    // behaviour to carry back to it.
    // TODO(sim-aux-turned-away)
    assert!(!sim.slot().aux.has_pending.get());
}

/// A datagram this build cannot read is counted as the boundary failure it is,
/// apart from the asks the driver turned away: the two send a reader to
/// different places -- a schema-version mismatch at the boundary, or a host that
/// is not what it claims to be.
#[test]
fn a_datagram_this_build_cannot_read_is_counted_apart_and_is_not_liveness() {
    let mut sim = Sim::new();
    sim.step();

    // A kind past the vocabulary: bytes that describe no datagram, so there is
    // no ask in them to refuse.
    let mut cmd = SessionCmdWire::new();
    cmd.set_kind(SessionCmdKindWire(9));
    sim.ask(&cmd);
    sim.step();

    assert_eq!(sim.slot().undecodable_inbound, 1);
    assert_eq!(
        sim.slot().aux_refused,
        0,
        "and not counted as an ask the driver would not run",
    );
    assert!(
        !sim.gate().has_accepted.get(),
        "and the driver has still never heard from a commander",
    );
}

/// A datagram asking nothing is a slot nothing wrote, published. Counted, and
/// deliberately not liveness: feeding the dead-man off bytes nobody could read
/// is holding a machine energised on the strength of noise.
#[test]
fn a_datagram_asking_nothing_is_refused_and_is_not_liveness() {
    let mut sim = Sim::new();
    let first = sim.step();

    sim.ask(&SessionCmdWire::new());
    sim.step();

    assert_eq!(sim.slot().aux_refused, 1);
    assert_eq!(
        sim.slot().undecodable_inbound,
        0,
        "the bytes read perfectly well; what they asked for was nothing",
    );
    assert!(
        !sim.gate().has_accepted.get(),
        "and the driver has still never heard from a commander",
    );
    assert_eq!(first.nominal, T0 + PERIOD, "the run started where it says");
}

/// Every accepted datagram is liveness, whichever kind it is: the dead-man
/// measures silence, and a host with nothing to ask still owes the driver a
/// word. This is the rule that carries a disarm's dwell, where the goal stream
/// has stopped and torque is still on.
#[test]
fn keep_alives_hold_the_dead_man_off_with_no_goal_stream() {
    let mut sim = Sim::armed();
    sim.step();

    // Twice the hold timeout, with nothing but keep-alives.
    for _ in 0..2 * HOLD_TIMEOUT / PERIOD {
        sim.keep_alive();
        let cycle = sim.step();
        assert!(
            cycle.event.is_none(),
            "a fed dead-man says nothing at {}",
            cycle.nominal
        );
    }
    assert!(!sim.gate().latched.get());
    assert!(
        flags::contains(sim.slot().torqued, JointRef::BodyYaw),
        "and the machine is still holding",
    );
}

/// The host takes torque off, and the driver reads it back: the sweep it wrote
/// is a claim, and a whole clean pass over the bus is the evidence. Said once
/// per pass, and only then does the belief go to nothing -- a de-torquing
/// nobody read back is one the dead-man must keep running over.
#[test]
fn a_commanded_torque_off_is_swept_and_then_confirmed() {
    let mut sim = Sim::armed();
    sim.step();

    let mut cmd = SessionCmdWire::new();
    cmd.set_kind(SessionCmdKindWire::TORQUE_OFF_NOW);
    sim.ask(&cmd);
    let latched = sim.step();
    assert_eq!(
        sim.slot().torqued,
        JointFlags::NONE,
        "the plant went limp on the cycle it was asked",
    );
    assert!(sim.gate().latched.get());
    assert_eq!(
        sim.slot().aux.believed_torqued,
        flags::all(),
        "and the belief stands until the read-backs come in",
    );

    // One row per cycle, so the pass lands a bus-row count of cycles later.
    let cycles = sim.quiet(JOINT_COUNT);
    let confirmed = cycles.last().expect("the pass runs");
    let event = confirmed.event.expect("the pass reports");
    assert_eq!(event.kind, EventKind::TorqueOffConfirmed);
    assert_eq!(
        confirmed.nominal,
        latched.nominal + JOINT_COUNT as i64 * PERIOD,
    );
    assert_eq!(sim.slot().aux.believed_torqued, JointFlags::NONE);

    // Said once: the standing condition is not news.
    for cycle in sim.quiet(3) {
        assert!(cycle.event.is_none());
    }
}

/// A fresh arming ends a confirmation pass part-way through. The machine is
/// being energised deliberately, and read-backs saying so are not evidence of
/// anything failing.
///
/// The pass's other exit from a part-finished state -- a row read back still
/// holding, which sends it to the start -- is not reachable from here: the only
/// hand that puts torque back on a row is a fresh arming, and that stands the
/// pass down first. It is `TorqueOffConfirm`'s own case.
#[test]
fn a_fresh_arming_stands_the_confirmation_pass_down() {
    let mut sim = Sim::armed();
    sim.step();

    let mut cmd = SessionCmdWire::new();
    cmd.set_kind(SessionCmdKindWire::TORQUE_OFF_NOW);
    sim.ask(&cmd);
    sim.step();
    // Four rows read clean -- one on the cycle the sweep was written, three
    // after it -- and then a hand puts one back on the machine.
    sim.quiet(3);
    assert_eq!(sim.slot().confirm.cursor, 4);
    sim.inject(SimOpWire::TORQUE_ON, flags::bit(JointRef::Leg0));
    sim.step();
    assert!(
        !sim.slot().confirm.active.get(),
        "a fresh arming stands the pass down: read-backs are no longer evidence \
         of anything failing",
    );
}

/// The rotation is surveillance with a cadence: one servo's status registers per
/// report, one row per report, and a whole lap takes the bus-row count of them.
#[test]
fn the_health_rotation_walks_the_bus_at_its_cadence() {
    let mut sim = Sim::new();
    let first = sim.step();
    let report = first.health.expect("a fresh driver owes its first report");
    assert_eq!(report.id, 10, "the rotation starts at row 0");
    assert_eq!(report.bits, 0, "a healthy servo is not complaining");
    assert!(report.volts > DEFAULT_MIN_ARM_VOLTAGE);
    assert_eq!(report.sample_time_ns, first.nominal);

    // Nothing until the cadence comes round.
    let period_cycles = HEALTH_PERIOD / PERIOD;
    for cycle in sim.quiet((period_cycles - 1) as usize) {
        assert!(
            cycle.health.is_none(),
            "inside the cadence at {}",
            cycle.nominal
        );
    }
    let next = sim.step().health.expect("the cadence came round");
    assert_eq!(next.id, 11, "and the rotation advanced one row");
}

/// The temperature a report carries and the temperature that register holds are
/// one number.
///
/// This plant models nothing thermal, so the figure is a constant either way --
/// and a constant in the report with a zero in the cell would be a machine that
/// answers two ways about one servo, which is the thing a hardware log and a
/// scenario log being one record rules out. Both halves are read here, so
/// dropping either side is red.
#[test]
fn a_report_and_the_temperature_cell_answer_the_same_degrees() {
    let mut sim = Sim::new();
    let report = sim
        .step()
        .health
        .expect("a fresh driver owes its first report");
    assert_eq!(report.temp_c, sim_aux::SIM_TEMP_C);

    sim.transact(
        3,
        &transaction(
            AuxOpKindWire::READ_REG,
            report.id,
            RegIdWire::PRESENT_TEMPERATURE,
            None,
        ),
    );
    let outcome = sim.step().outcome.expect("the read was answered");
    assert_eq!(outcome.status, AuxStatus::Ok);
    assert_eq!(
        outcome.value,
        value::u8(sim_aux::SIM_TEMP_C.cast_unsigned()).bits(),
        "the cell the same servo's report was written from",
    );
}

/// A servo whose error byte latched is what the rotation reports about it: the
/// report is a read of that servo's own cells, so a scenario's hand on the
/// register is what a host reading the bus itself would find.
#[test]
fn the_rotation_reports_the_error_byte_a_servo_holds() {
    let mut sim = Sim::new();
    sim.step();

    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::SET_REGISTER);
    cmd.set_mask(JointFlagsWire::from(flags::bit(JointRef::Leg0)));
    cmd.set_reg(RegIdWire::HARDWARE_ERROR_STATUS);
    cmd.set_value(ServoHealth::INPUT_VOLTAGE.into());
    sim.inject_full(&cmd);

    // Round to the cadence, then the row after the first.
    let period_cycles = (HEALTH_PERIOD / PERIOD) as usize;
    let reports: Vec<Health> = sim
        .quiet(period_cycles)
        .into_iter()
        .filter_map(|cycle| cycle.health)
        .collect();
    let report = reports.last().expect("the rotation reported");
    assert_eq!(report.id, 11, "the first leg");
    assert_eq!(
        report.bits,
        ServoHealth::INPUT_VOLTAGE,
        "and its rail bit is latched",
    );
}

/// A bus that answers nothing for long enough is the one fault a driver raises
/// about itself, and it says so once.
///
/// The run of unanswered cycles is all the evidence there is: a cycle that read
/// no row read nothing about what is wrong with it either. Said on the cycle the
/// run reaches its length and not again while it stands, because a host told
/// once has already stopped trusting the bus.
#[test]
fn a_bus_that_answers_nothing_for_long_enough_says_so_once() {
    let mut sim = Sim::new();
    let outage = BLIND_CYCLES_BEFORE_BUS_FAILURE + 10;

    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::DROP_REPLIES);
    cmd.set_count(outage);
    sim.inject_full(&cmd);

    let cycles = sim.quiet(outage as usize);
    let raised: Vec<&Cycle> = cycles
        .iter()
        .filter(|cycle| cycle.event.is_some())
        .collect();
    assert_eq!(raised.len(), 1, "one report for one outage");
    let cycle = raised[0];
    let event = cycle.event.expect("the filter found it");
    assert_eq!(event.kind, EventKind::BusFailure);
    assert_eq!(
        event.count, BLIND_CYCLES_BEFORE_BUS_FAILURE,
        "carrying how many cycles went unanswered",
    );
    assert_eq!(
        cycle.nominal,
        T0 + i64::from(BLIND_CYCLES_BEFORE_BUS_FAILURE) * PERIOD,
        "on the cycle the run reached its length, the first blind one counting",
    );

    // The reads come back, and the count with them: a second outage is a second
    // report, because what the driver is describing is what its bus is doing
    // rather than a verdict it latched.
    let back = sim.step();
    assert!(back.sample.present_valid);
    assert_eq!(sim.slot().blind_cycles, 0);

    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::DROP_REPLIES);
    cmd.set_count(BLIND_CYCLES_BEFORE_BUS_FAILURE);
    sim.inject_full(&cmd);
    let again = sim.quiet(BLIND_CYCLES_BEFORE_BUS_FAILURE as usize);
    let event = again
        .last()
        .expect("the outage ran")
        .event
        .expect("the second outage is reported too");
    assert_eq!(event.kind, EventKind::BusFailure);
}

/// A burst of lost replies shorter than that says nothing at all.
///
/// The decision tick tolerates a run of blind reads and keeps commanding through
/// them; a driver crying failure inside that window would be reporting the same
/// outage twice from two places, and a scenario about a stuttering bus would
/// look like one about a bus that had gone.
#[test]
fn a_burst_of_lost_replies_shorter_than_the_run_says_nothing() {
    let mut sim = Sim::new();

    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::DROP_REPLIES);
    cmd.set_count(BLIND_CYCLES_BEFORE_BUS_FAILURE - 1);
    sim.inject_full(&cmd);

    for cycle in sim.quiet(BLIND_CYCLES_BEFORE_BUS_FAILURE as usize) {
        assert!(
            cycle.event.is_none(),
            "one cycle short of the run is not a bus failure, at {}",
            cycle.nominal
        );
    }
    assert_eq!(
        sim.slot().blind_cycles,
        0,
        "and the count went back to zero when the reads came back",
    );
}

/// A transaction swallowed before it reaches the bus is answered with nothing at
/// all, and the host's re-issue is taken like any other request.
///
/// The silence is the point: an outcome saying "timeout" would be an answer, and
/// a host that got one would know more than a host whose datagram was lost. What
/// the world does with a lost request is exactly nothing, which is what the
/// host's own delivery timeout exists for.
#[test]
fn a_swallowed_transaction_answers_nothing_and_the_re_issue_is_taken() {
    let mut sim = Sim::new();
    sim.step();

    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::REFUSE_AUX);
    cmd.set_count(1);
    sim.inject_full(&cmd);

    let txn = transaction(AuxOpKindWire::PING, 10, RegIdWire::NONE, None);
    sim.transact(21, &txn);
    let lost = sim.step();
    assert!(lost.outcome.is_none(), "the request never reached the bus");
    assert_eq!(
        sim.slot().aux_refused,
        0,
        "the driver turned nothing away: the world swallowed it",
    );
    assert!(
        !sim.slot().aux.has_pending.get(),
        "and the slot is free for the re-issue",
    );

    // The same datagram again, under the same number: the property the host's
    // delivery retry rests on.
    sim.transact(21, &txn);
    let answered = sim.step().outcome.expect("the re-issue was answered");
    assert_eq!(answered.corr, 21);
    assert_eq!(answered.status, AuxStatus::Ok);
    assert_eq!(answered.model, EXPECTED_MODELS[0]);
}

/// A servo off the bus answers nothing -- no ping, no read, no write -- and a
/// write to one reaches neither its cell nor the plant.
///
/// What a dead or unplugged servo is to a driver, and what a commissioning
/// sequence has to fail on. The status is a timeout and not a refusal: the
/// datagram went out and the window closed on silence.
#[test]
fn a_servo_off_the_bus_answers_nothing_and_takes_no_write() {
    let mut sim = Sim::new();
    sim.step();
    sim.inject(SimOpWire::ABSENT_SERVO, flags::bit(JointRef::AntennaLeft));
    sim.step();

    for (corr, txn) in [
        (
            31,
            transaction(AuxOpKindWire::PING, 18, RegIdWire::NONE, None),
        ),
        (
            32,
            transaction(AuxOpKindWire::READ_REG, 18, RegIdWire::MODEL_NUMBER, None),
        ),
        (
            33,
            transaction(
                AuxOpKindWire::WRITE_REG_VERIFIED,
                18,
                RegIdWire::TORQUE_ENABLE,
                Some(value::u8(1)),
            ),
        ),
    ] {
        sim.transact(corr, &txn);
        let outcome = sim.step().outcome.expect("the host is answered either way");
        assert_eq!(outcome.corr, corr);
        assert_eq!(outcome.status, AuxStatus::Timeout, "request {corr}");
    }

    let slot = sim.slot();
    assert_eq!(
        slot.torqued,
        JointFlags::NONE,
        "the write reached no plant: a servo that says nothing does nothing",
    );
    assert_eq!(
        slot.aux.believed_torqued,
        JointFlags::NONE,
        "and nothing is believed",
    );
    assert_eq!(
        sim_regs::read(&slot.regs, 8, RegId::TorqueEnable),
        Ok(value::u8(0)),
        "and its control table was never written",
    );

    // Put it back on the bus: a mask naming fewer servos is what ends an outage,
    // and a scenario that could not end one could not show a machine surviving.
    sim.inject(SimOpWire::ABSENT_SERVO, JointFlags::NONE);
    sim.step();
    sim.transact(
        34,
        &transaction(AuxOpKindWire::PING, 18, RegIdWire::NONE, None),
    );
    let outcome = sim.step().outcome.expect("it is back");
    assert_eq!(outcome.status, AuxStatus::Ok);
    assert_eq!(outcome.model, EXPECTED_MODELS[8]);
}

/// A servo off the bus makes no health report, and the rotation walks on.
///
/// A report of zeroes about a machine nobody heard from would read as a healthy
/// servo. The cadence was stamped when the read was named, so the row after it
/// is reported at the next cadence either way.
#[test]
fn a_servo_off_the_bus_makes_no_health_report_and_the_rotation_walks_on() {
    let mut sim = Sim::new();
    sim.inject(SimOpWire::ABSENT_SERVO, flags::bit(JointRef::BodyYaw));
    let first = sim.step();
    assert!(
        first.health.is_none(),
        "the rotation starts at row 0, which is not answering",
    );

    let period_cycles = (HEALTH_PERIOD / PERIOD) as usize;
    let next = sim
        .quiet(period_cycles)
        .into_iter()
        .filter_map(|cycle| cycle.health)
        .next_back()
        .expect("the cadence came round");
    assert_eq!(next.id, 11, "and the rotation had walked on to row 1");
}

/// A de-torquing cannot be confirmed while a servo is off the bus: a row that
/// answers nothing has not been seen to go limp.
///
/// The one report this driver must never make is a de-torquing credited to
/// silence. So the pass keeps reading, the budget runs out, and what it says is
/// which servo it could not see -- after which the sweep is still written every
/// cycle and the belief still stands, because nothing gates de-torquing.
#[test]
fn a_de_torquing_is_not_confirmed_while_a_servo_is_off_the_bus() {
    let mut sim = Sim::armed();
    sim.step();
    sim.inject(SimOpWire::ABSENT_SERVO, flags::bit(JointRef::Leg3));
    sim.step();

    let mut cmd = SessionCmdWire::new();
    cmd.set_kind(SessionCmdKindWire::TORQUE_OFF_NOW);
    sim.ask(&cmd);
    sim.step();

    let events: Vec<Event> = sim
        .quiet((TORQUE_OFF_CONFIRM_BUDGET / PERIOD + 2) as usize)
        .into_iter()
        .filter_map(|cycle| cycle.event)
        .collect();
    let said = events
        .iter()
        .find(|event| event.kind == EventKind::TorqueOffUnconfirmed)
        .expect("the budget ran out on a servo nobody could read");
    assert!(
        !events
            .iter()
            .any(|event| event.kind == EventKind::TorqueOffConfirmed),
        "and nothing was confirmed",
    );
    assert_eq!(
        sim.slot().aux.believed_torqued,
        flags::all(),
        "the belief stands, so the dead-man keeps running over the machine",
    );
    assert_eq!(
        said.rows,
        JointFlagsWire::from(flags::bit(JointRef::Leg3)).0,
        "naming the one servo it could not read, off the control tables",
    );
    assert_eq!(said.silence_ns, 0, "the kind names no silence");
}

/// A value built in a shape the register does not take is refused, and nothing
/// is written.
///
/// One of the two malformed requests a host can actually provoke, and the reason
/// the answer is a refusal rather than a servo's complaint: nothing went on the
/// wire. A simulator that accepted it would paper over a sequencer that built its
/// value wrong, in every scenario that ran one.
#[test]
fn a_write_whose_value_is_the_wrong_shape_is_refused() {
    let mut sim = Sim::new();
    sim.step();

    // The operating mode is a byte, offered as a four-byte signed number.
    sim.transact(
        41,
        &transaction(
            AuxOpKindWire::WRITE_REG_VERIFIED,
            17,
            RegIdWire::OPERATING_MODE,
            Some(value::i32(3)),
        ),
    );
    let outcome = sim.step().outcome.expect("the host is answered either way");
    assert_eq!(outcome.corr, 41);
    assert_eq!(outcome.status, AuxStatus::Refused);
    assert_eq!(
        sim_regs::read(&sim.slot().regs, 7, RegId::OperatingMode),
        Ok(value::u8(EXPECTED_OPERATING_MODES[7])),
        "and the cell holds what it was provisioned with",
    );
}

/// A write to a servo's own reading of where it is is refused: no such write
/// reaches a servo on the real bus, and the plant is what moves this one.
#[test]
fn a_write_to_a_present_position_register_is_refused() {
    let mut sim = Sim::new();
    sim.step();
    let standing = rows_of(&sim.slot().positions)[8];

    sim.transact(
        42,
        &transaction(
            AuxOpKindWire::WRITE_REG_VERIFIED,
            18,
            RegIdWire::PRESENT_POSITION,
            Some(value::radians(standing + 0.5)),
        ),
    );
    let outcome = sim.step().outcome.expect("the host is answered either way");
    assert_eq!(outcome.corr, 42);
    assert_eq!(outcome.status, AuxStatus::Refused);
    assert_eq!(
        rows_of(&sim.slot().positions)[8],
        standing,
        "the modelled servo did not move",
    );
    assert_eq!(
        sim_regs::read(&sim.slot().regs, 8, RegId::PresentPosition),
        Ok(value::radians(standing)),
        "and its cell still says where it is",
    );
}

/// A release commanded after a transaction was offered outranks it: the pending
/// request is abandoned rather than run out of the slot behind the latch.
///
/// The hazard is a torque-enable write, which is the one transaction that undoes
/// a release: run after the latch it would energise the row again, release the
/// latch and stand the confirmation pass down, leaving a machine holding torque
/// that its host had asked to let go. Nothing goes back to the host for the
/// abandoned request -- the transaction never reached the bus, and the host's own
/// delivery timeout is what covers that.
#[test]
fn a_release_commanded_after_a_request_abandons_it() {
    let mut sim = Sim::new();
    sim.step();

    // Both in the same window, so both are drained by one cycle: the request
    // first, then the release.
    sim.transact(
        50,
        &transaction(
            AuxOpKindWire::WRITE_REG_VERIFIED,
            18,
            RegIdWire::TORQUE_ENABLE,
            Some(value::u8(1)),
        ),
    );
    let mut release = SessionCmdWire::new();
    release.set_kind(SessionCmdKindWire::TORQUE_OFF_NOW);
    sim.ask(&release);

    let cycle = sim.step();
    assert!(
        cycle.outcome.is_none(),
        "the abandoned request went nowhere, so nothing answers it",
    );
    let slot = sim.slot();
    assert!(
        slot.gate.latched.get(),
        "the release stands: nothing this cycle re-armed the machine",
    );
    assert_eq!(
        slot.torqued,
        JointFlags::NONE,
        "and the plant is limp, which is what the release commanded",
    );
    assert_eq!(
        slot.aux.believed_torqued,
        JointFlags::NONE,
        "the driver believes nothing is holding",
    );
    assert!(
        slot.confirm.active.get(),
        "the confirmation pass is running rather than stood down",
    );

    // And it stays that way: the abandoned request is not run a cycle later
    // either.
    let after = sim.step();
    assert!(after.outcome.is_none());
    assert!(sim.slot().gate.latched.get());
}

/// A cycle whose bus answers nothing answers nothing at all: not the
/// proprioception, not the host's transaction, not the read-back, not the
/// rotation.
///
/// The outage is the wire's rather than one read's. A driver that kept executing
/// transactions against its register file through an outage it had itself
/// declared would be a simulator whose contract no real driver could keep -- and
/// would credit a de-torquing to a read-back that never happened.
#[test]
fn a_blind_cycle_answers_nothing_on_any_path() {
    let mut sim = Sim::armed();
    sim.step();
    let held = sim_regs::read(&sim.slot().regs, 8, RegId::TorqueEnable);
    assert_eq!(held, Ok(value::u8(1)), "the machine came up energised");

    // The bus goes away, and the host commands a release into the silence.
    let mut cmd = SimCmdWire::new();
    cmd.set_op(SimOpWire::DROP_REPLIES);
    // Long enough that the driver's own confirmation budget runs out inside it,
    // short enough that it is not the outage the driver calls its bus gone.
    let outage = (TORQUE_OFF_CONFIRM_BUDGET / PERIOD + 5) as u32;
    assert!(outage < BLIND_CYCLES_BEFORE_BUS_FAILURE);
    cmd.set_count(outage);
    sim.inject_full(&cmd);
    let mut release = SessionCmdWire::new();
    release.set_kind(SessionCmdKindWire::TORQUE_OFF_NOW);
    sim.ask(&release);
    sim.step();

    // A read the host asks for over the outage: nothing comes back.
    sim.transact(
        60,
        &transaction(AuxOpKindWire::READ_REG, 17, RegIdWire::OPERATING_MODE, None),
    );
    let cycles = sim.quiet(outage as usize - 1);
    assert!(
        cycles.iter().all(|cycle| cycle.outcome.is_none()),
        "a transaction over a bus that answers nothing is answered by nothing",
    );
    assert!(
        cycles.iter().all(|cycle| cycle.health.is_none()),
        "and the rotation reports nothing about a machine nobody heard from",
    );
    let events: Vec<EventKind> = cycles
        .iter()
        .filter_map(|cycle| cycle.event)
        .map(|event| event.kind)
        .collect();
    assert!(
        !events.contains(&EventKind::TorqueOffConfirmed),
        "and no de-torquing is credited to silence: {events:?}",
    );
    assert!(
        events.contains(&EventKind::TorqueOffUnconfirmed),
        "the budget ran out with nothing read back: {events:?}",
    );
    assert_eq!(
        sim_regs::read(&sim.slot().regs, 8, RegId::TorqueEnable),
        held,
        "the cells hold what they held when the bus went, because nothing read them",
    );

    // The reads come back, and the pass reads the whole bus back clean.
    let after = sim.quiet(JOINT_COUNT + 2);
    let confirmed = after
        .iter()
        .filter_map(|cycle| cycle.event)
        .find(|event| event.kind == EventKind::TorqueOffConfirmed)
        .expect("a bus that answers again confirms the release");
    assert!(confirmed.time_ns > cycles.last().expect("the outage ran").nominal);
    assert_eq!(
        sim.slot().aux.believed_torqued,
        JointFlags::NONE,
        "and the belief goes with the confirmation",
    );
}
