//! The bench's commands: what each one does to the machine, and in what order.
//!
//! Every command that moves the head has the same shape, and the shape is the
//! safety property. Nothing is remembered between invocations — each is a fresh
//! process — so a command re-drives the whole arm sequence, which verifies the
//! nine servos, pins every joint where it stands and enables torque, and only
//! then injects one `MoveTo` over the fixed-rate loop. A machine that has
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
//! Ports are the caller's: every command here takes one already open, so the
//! whole surface is exercisable against a scripted machine with no device in
//! sight. What a command prints goes out through a callback for the same
//! reason.

use core::time::Duration;

use reachy_bus::{Bus, BusPort};
use reachy_kin::{neutral_head_pose, stow_head_pose};
use reachy_motion::disarm::STOW_ANTENNAS;
use reachy_motion::{
    ArmSequencer, DisarmSequencer, DisarmSummary, JointTargets, MotionCommand, MotionState, Warp,
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

/// How far the demo swings the antennas, radians either side of upright.
///
/// A visible motion that stays far from the antenna bound, for the same reason
/// the yaw sweep stays inside the yaw cap.
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
        let summary = self.pump.run(
            &mut self.bus,
            &mut self.state,
            command,
            clock,
            &mut |event| line(&format!("  {event}")),
        )?;
        line(&move_line(&summary));
        Ok(summary)
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
        let summary = self.pump.hold(
            &mut self.bus,
            &mut self.state,
            duration,
            clock,
            // The disposition of a hold is that it is holding, which is the one
            // thing this command already knows. Everything else — a lost read,
            // a health latch, a fault — is news.
            &mut |event| {
                if !matches!(event, TickEvent::Command(_)) {
                    line(&format!("  {event}"));
                }
            },
        )?;
        line(&move_line(&summary));
        Ok(summary)
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
    line("off: verifying the machine is at stow, settling, then releasing torque");
    release(resolved, &mut bus, force_drop, clock, line)?;
    Ok(())
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

    use reachy_bus::reg_for;
    use reachy_kin::{HeadGeometry, LegAngles, inverse_kinematics};
    use reachy_motion::{JointGroup, JointId, JointVector, RegId, SeqError};

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

    /// What a command left behind: the registers, the servos each grouped write
    /// carried, every grouped write in full, and every line it printed.
    struct Run {
        outcome: Result<(), PumpError>,
        registers: Rc<RefCell<FakeMachine>>,
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
        let addressed = spy.addressed();
        let commanded = spy.commanded();
        let mut clock = TestClock::default();
        let mut printed = Vec::new();
        let outcome = command(spy, &mut clock, &mut |line| printed.push(line.to_string()));
        Run {
            outcome,
            registers,
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
                "re-pin",
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
        // so it says nothing on its own. Arming pins every joint where it
        // stands and pulls one outside its window to the nearer bound: on a
        // machine about to be released that is a movement nobody is standing
        // ready for, and these two are what say it did not happen.
        run.armed_nothing();
        run.commanded_nothing(&cfg);
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

    /// An antenna past its bound is refused, on the same verdict path a leg or
    /// the body would be.
    #[test]
    fn an_antenna_past_its_bound_is_refused() {
        let cfg = resolved();
        let machine = machine_at(&datumed_config(), &neutral_legs());
        let run = run(machine, |port, clock, line| {
            antennas(&cfg, port, 4.0, 0.0, clock, line)
        });

        let error = run.err("four radians is past the bound");
        assert!(matches!(error, PumpError::Rejected(_)), "{error}");
        assert!(error.to_string().contains("antenna"), "{error}");
        assert!(run.addressed.borrow().is_empty());
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
