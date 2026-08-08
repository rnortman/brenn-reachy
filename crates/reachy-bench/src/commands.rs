//! The bench's commands: what each one does to the machine, and in what order.
//!
//! Every command that moves the head has the same shape, and the shape is the
//! safety property. Nothing is remembered between invocations — each is a fresh
//! process — so a command re-drives the whole arm sequence, which verifies the
//! nine servos, enables torque — which holds every joint where it stands — pins
//! each joint there, and only then injects one `MoveTo` over the fixed-rate
//! loop. A machine that has
//! drifted, been handled, or was never armed at all is therefore re-established
//! from scratch every time, and a command that cannot establish it does not
//! move anything.
//!
//! `off` is the exception, and deliberately: it releases torque, so re-arming
//! first would enable torque in order to switch it off. It drives the disarm
//! sequence against the machine as found, and that sequence's own first phase —
//! nine positions measured against the stow pose — is the gate. A machine
//! somewhere else is refused unless the operator has accepted that the head
//! will fall.
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

use reachy_bus::{Bus, BusPort, BusTiming, MapError, ServoMap, reg_for, value_kind, with_retry};
use reachy_kin::{neutral_head_pose, stow_head_pose};
use reachy_motion::disarm::STOW_ANTENNAS;
use reachy_motion::{
    ArmSequencer, DisarmSequencer, DisarmSummary, EXPECTED_MODELS, EXPECTED_OPERATING_MODES,
    JointId, JointTargets, MotionCommand, MotionState, RegId, RegValue, ValueKind, Warp,
};

use crate::config::Resolved;
use crate::pump::{
    Clock, DISARM_ACTIONS, MotionPump, MoveSummary, PumpError, TickEvent, action_budget,
    arm_report, drive,
};

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

/// A live session: an armed machine, and the loop that moves it.
///
/// Constructed only by [`Session::arm`], so holding one is the evidence that
/// the nine servos answered, were found provisioned as recorded, are on a
/// healthy rail, and are holding torque at a pose the envelope admits.
pub struct Session<'a, P: BusPort> {
    resolved: &'a Resolved,
    bus: Bus<P>,
    state: MotionState,
    pump: MotionPump<'a>,
}

impl<'a, P: BusPort> Session<'a, P> {
    /// Drive the arm sequence over `port` and take the machine holding where it
    /// stands.
    ///
    /// The phases are announced as they are entered: the supply gate alone can
    /// take the whole of its configured budget, and a supervised run that said
    /// nothing until the end would look hung at exactly the moment an operator
    /// is deciding whether to reach for the power. The record of what arming
    /// found is the last thing it has to say, and it is printed here rather
    /// than by each caller, so no command can arm the machine without it.
    pub fn arm(
        resolved: &'a Resolved,
        port: P,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
    ) -> Result<Self, PumpError> {
        let mut bus = Bus::new(port, resolved.timing);
        let mut sequencer = ArmSequencer::new(
            &resolved.arm,
            &resolved.motion.geom,
            &resolved.motion.env,
            &resolved.motion.fk,
        );
        let armed = drive(
            &mut bus,
            &resolved.map,
            &mut sequencer,
            clock,
            action_budget(&resolved.arm),
            &mut |step| line(&format!("  {step}")),
        )?;
        line(arm_report(&armed).trim_end());
        let pump = MotionPump::new(
            &resolved.motion,
            &resolved.map,
            resolved.tick_hz,
            resolved.health_poll_hz,
            armed.armed.joints,
        )?;
        Ok(Self {
            resolved,
            state: MotionState::new_armed(&armed.armed),
            bus,
            pump,
        })
    }

    /// The configuration the machine was last commanded to, which is where the
    /// next move starts.
    #[must_use]
    pub fn targets(&self) -> JointTargets {
        *self.state.last_targets()
    }

    /// Carry one move to its endpoint.
    pub fn move_to(
        &mut self,
        target: JointTargets,
        duration: Duration,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
    ) -> Result<MoveSummary, PumpError> {
        let command = MotionCommand::MoveTo {
            target,
            duration,
            warp: WARP,
        };
        let outcome = self.pump.run(
            &mut self.bus,
            &mut self.state,
            command,
            clock,
            &mut |event| line(&format!("  {event}")),
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
        line(&lag_line(&summary));
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
    /// [`Session::hold`] is this with the bench's own policy applied.
    pub fn hold_events(
        &mut self,
        duration: Duration,
        clock: &mut dyn Clock,
        event: &mut dyn FnMut(TickEvent),
    ) -> Result<MoveSummary, PumpError> {
        self.pump
            .hold(&mut self.bus, &mut self.state, duration, clock, event)
    }

    /// Release torque, having verified the machine is at stow, and end the
    /// session.
    ///
    /// `force_drop` is the operator's explicit acceptance that the head falls;
    /// without it a machine measured anywhere but stow is refused.
    ///
    /// Consumes the session, because a session is the evidence that nine servos
    /// are holding torque and this is what makes that false: a released session
    /// left callable would take a `move_to` and pump goal frames at a limp
    /// machine, diverging further every period until the tracking budget ran
    /// out.
    pub fn release(
        mut self,
        force_drop: bool,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
    ) -> Result<DisarmSummary, PumpError> {
        release(self.resolved, &mut self.bus, force_drop, clock, line)
    }
}

/// Drive the disarm sequence against whatever is on `bus`.
///
/// Free of a [`Session`] because `off` must not arm to disarm: the sequence
/// reads nine positions, compares them against the stow pose, settles, and then
/// releases torque one servo at a time with each release read back. Nothing
/// after the release: the platform settles limp into its true rest, which is
/// the state the next boot's arm sequence is entitled to assume.
pub fn release<P: BusPort>(
    resolved: &Resolved,
    bus: &mut Bus<P>,
    force_drop: bool,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<DisarmSummary, PumpError> {
    let mut cfg = resolved.disarm;
    // The drop flag is given per invocation and never stored, so it is written
    // here and nowhere else.
    cfg.force_drop = force_drop;
    let mut sequencer = DisarmSequencer::new(&cfg);
    let summary = drive(
        bus,
        &resolved.map,
        &mut sequencer,
        clock,
        DISARM_ACTIONS,
        &mut |step| line(&format!("  {step}")),
    )?;
    line(&disarm_line(&summary));
    Ok(summary)
}

/// What a move cost, as a run prints it.
fn move_line(summary: &MoveSummary) -> String {
    format!(
        "  {ticks} period(s), {goals} commanding, {frames} frame(s), {misses} blind, \
         {overruns} overrun(s), worst jitter {jitter:.1} ms, {elapsed:.2} s",
        ticks = summary.ticks,
        goals = summary.goals,
        frames = summary.frames,
        misses = summary.misses,
        overruns = summary.overruns,
        jitter = summary.worst_jitter.as_secs_f64() * 1e3,
        elapsed = summary.elapsed.as_secs_f64(),
    )
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

/// Where the machine was when torque came off.
fn disarm_line(summary: &DisarmSummary) -> String {
    let (joint, deviation) = summary.worst_deviation();
    format!(
        "  torque off; {at}, furthest joint {joint} at {:.3} deg from stow",
        deviation.to_degrees(),
        at = if summary.at_stow {
            "at stow"
        } else {
            "away from stow, released on the drop flag"
        },
    )
}

/// Verify the machine, pin every joint where it stands, and enable torque.
pub fn arm<P: BusPort>(
    resolved: &Resolved,
    port: P,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    Session::arm(resolved, port, clock, line)?;
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
    let mut session = Session::arm(resolved, port, clock, line)?;
    line("up: to the neutral configuration");
    session.move_to(neutral_targets(), resolved.up_duration, clock, line)?;
    Ok(())
}

/// Hold where the machine already is, and watch it do so.
pub fn hold<P: BusPort>(
    resolved: &Resolved,
    port: P,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let mut session = Session::arm(resolved, port, clock, line)?;
    line("hold: commanding nothing, measuring every period");
    session.hold(resolved.hold_duration, clock, line)?;
    Ok(())
}

/// Move the head to the stow configuration, leaving torque on.
pub fn stow<P: BusPort>(
    resolved: &Resolved,
    port: P,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let mut session = Session::arm(resolved, port, clock, line)?;
    line("stow: to the stow configuration; torque stays on until `off`");
    session.move_to(stow_pose_targets(), resolved.stow_duration, clock, line)?;
    Ok(())
}

/// Release torque, having verified the machine is at stow.
pub fn off<P: BusPort>(
    resolved: &Resolved,
    port: P,
    force_drop: bool,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), PumpError> {
    let mut bus = Bus::new(port, resolved.timing);
    line("off: settling, verifying the machine is at stow, then releasing torque");
    release(resolved, &mut bus, force_drop, clock, line)?;
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
    let mut session = Session::arm(resolved, port, clock, line)?;
    line(&format!("yaw: to {degrees:.1} deg"));
    let target = JointTargets {
        body_yaw: degrees.to_radians(),
        ..session.targets()
    };
    session.move_to(target, resolved.move_duration, clock, line)?;
    Ok(())
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
    let mut session = Session::arm(resolved, port, clock, line)?;
    line(&format!("antennas: to [{right:.3}, {left:.3}] rad"));
    let target = JointTargets {
        antennas: [right, left],
        ..session.targets()
    };
    session.move_to(target, resolved.move_duration, clock, line)?;
    Ok(())
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
    let mut session = Session::arm(resolved, port, clock, line)?;

    line("demo 1/6: up");
    session.move_to(neutral_targets(), resolved.up_duration, clock, line)?;

    line("demo 2/6: holding");
    session.hold(resolved.hold_duration, clock, line)?;

    line("demo 3/6: antennas");
    for antennas in [
        [DEMO_ANTENNA_RAD, -DEMO_ANTENNA_RAD],
        [-DEMO_ANTENNA_RAD, DEMO_ANTENNA_RAD],
        [0.0, 0.0],
    ] {
        let target = JointTargets {
            antennas,
            ..session.targets()
        };
        session.move_to(target, resolved.move_duration, clock, line)?;
    }

    line("demo 4/6: body yaw");
    for degrees in [DEMO_YAW_DEG, -DEMO_YAW_DEG, 0.0] {
        let target = JointTargets {
            body_yaw: degrees.to_radians(),
            ..session.targets()
        };
        session.move_to(target, resolved.move_duration, clock, line)?;
    }

    line("demo 5/6: stow");
    session.move_to(stow_pose_targets(), resolved.stow_duration, clock, line)?;

    line("demo 6/6: off");
    session.release(false, clock, line)?;
    Ok(())
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::rc::Rc;

    use dxl_proto::frame::INST_WRITE;
    use reachy_kin::{HeadGeometry, LegAngles, inverse_kinematics, wrap_to_pi};
    use reachy_motion::{
        ANTENNA_GOAL_MAX_RAD, ANTENNA_GOAL_MIN_RAD, CommandDisposition, JointGroup, JointVector,
        SeqError,
    };

    use super::*;
    use crate::testutil::{
        FakeMachine, GroupedWrite, Spy, TestClock, datumed_config, machine_at, resolved, stow_legs,
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

        /// Nothing here armed the machine.
        ///
        /// Arming is the only thing that prints the record of what it found, so
        /// the record's absence is the absence of an arm sequence.
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
    fn goals(cfg: &Resolved, run: &Run) -> JointVector {
        let machine = run.registers.borrow();
        let mut joints = JointVector::default();
        for (row, id) in cfg.map.ids().iter().enumerate() {
            let held = machine
                .get(*id, reg_for(RegId::GoalPosition))
                .expect("every servo has a goal");
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
    fn within_a_count(left: &JointVector, right: &JointVector, what: &str) {
        let half_count = core::f64::consts::PI / 4096.0;
        for (joint, angle) in left.joints() {
            let other = right.get(joint).expect("nine joints");
            assert!(
                (angle - other).abs() <= half_count,
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

    /// Arming prints its record, whichever command did the arming.
    ///
    /// The record is the point of a supervised arm — where the machine was
    /// found, what it was left holding, what had to be pulled — and it is
    /// printed by the arming itself, so a command cannot leave it out.
    #[test]
    fn every_arming_command_prints_what_arming_found() {
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

            // The whole record goes out as one block, so it is one printed
            // item, and it is printed exactly once however many moves follow.
            let records: Vec<&String> = run
                .printed
                .iter()
                .filter(|line| line.starts_with("found "))
                .collect();
            assert_eq!(records.len(), 1, "{name}: {:?}", run.printed);
            for label in [
                "armed",
                "pull-in",
                "droop",
                "torque-on",
                "models",
                "supply",
                "health",
                "torque was",
                "registers",
            ] {
                assert!(records[0].contains(label), "{name}: no {label} line");
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
            summary.contains("blind") && summary.contains("worst jitter"),
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
        let run = run(machine, |port, clock, line| {
            off(&cfg, port, false, clock, line)
        });
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

        let run = run(machine, |port, clock, line| {
            off(&cfg, port, false, clock, line)
        });
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

    /// A machine anywhere but stow is refused, with nothing released — and the
    /// operator's explicit acceptance of a falling head is what changes that.
    #[test]
    fn off_away_from_stow_is_refused_unless_the_head_may_fall() {
        let cfg = resolved();
        let armed = || {
            let mut machine = machine_at(&datumed_config(), &neutral_legs());
            for id in cfg.map.ids() {
                machine.set(id, reg_for(RegId::TorqueEnable), &[1]);
            }
            machine
        };

        let refused = run(armed(), |port, clock, line| {
            off(&cfg, port, false, clock, line)
        });
        let error = refused.err("the head is not at stow");
        assert!(
            matches!(error, PumpError::Sequence(SeqError::NotAtStow { .. })),
            "{error}"
        );
        assert_eq!(
            torque(&cfg, &refused),
            vec![1; JointId::COUNT],
            "a refused release releases nothing"
        );
        refused.armed_nothing();
        refused.commanded_nothing(&cfg);

        let dropped = run(armed(), |port, clock, line| {
            off(&cfg, port, true, clock, line)
        });
        dropped.ok("the drop flag accepts the fall");
        assert_eq!(torque(&cfg, &dropped), vec![0; JointId::COUNT]);
        assert!(
            dropped
                .printed
                .iter()
                .any(|line| line.contains("away from stow")),
            "{:?}",
            dropped.printed
        );
        dropped.armed_nothing();
        dropped.commanded_nothing(&cfg);
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
            let mut session = Session::arm(&cfg, port, clock, line)?;
            line(ARMED);
            let outcome = session.hold_events(cfg.hold_duration, clock, &mut |event| {
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

    /// An antenna direction past the half turn is an ordinary command: it
    /// resolves to the representative nearest the frame the machine is in and
    /// sweeps the short way there. Four radians, from a machine found at zero,
    /// is 2.283 rad of travel the other way — not four the long way, and not a
    /// refusal.
    #[test]
    fn an_antenna_direction_past_the_half_turn_takes_the_short_way() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &neutral_legs());
        let run = run(machine, |port, clock, line| {
            antennas(&cfg, port, 4.0, 0.0, clock, line)
        });
        run.ok("a direction past the half turn is still a direction");

        let series = run.goal_series(&cfg, JointId::AntennaRight);
        assert!(!series.is_empty(), "the antenna was commanded");
        assert!(
            series.iter().all(|goal| *goal <= 1e-9),
            "the sweep went down through the near side: {series:?}"
        );
        let landed = *series.last().expect("a last goal");
        let short_way = 4.0 - core::f64::consts::TAU;
        assert!(
            (landed - short_way).abs() < 0.01,
            "landed at {landed}, asked for {short_way}"
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

    /// Every command that moves re-drives the whole arm sequence first, so a
    /// machine that cannot be armed moves nothing — and says so as an arming
    /// refusal rather than as a failed move.
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
}
