//! Disarming: the verified torque-off tail, and nothing else.
//!
//! Stowing the platform is not this sequencer's job. The motion to the stow pose
//! is an ordinary command through the tick — interpolated, envelope-checked on
//! every tick, emitted as bounded increments — because it is the one path where
//! the head travels near the bottom of its range and the checks that matter
//! there are the tick's. What is left over is what cannot be expressed as a
//! trajectory: confirm the platform is measured where stow put it, let it
//! settle, and release torque servo by servo with each release read back.
//!
//! ## Nothing happens after the last release
//!
//! Once torque is off the platform settles limp into its true rest, and that
//! rest is what the next arming will find and pin. So the sequence ends there:
//! no re-read, no confirmation pose, no tidying write. Anything issued after the
//! release would be describing a machine that is still moving.
//!
//! ## Releasing away from stow drops the head
//!
//! Torque is what holds the head up. Releasing it anywhere but the pose the head
//! can rest at means the head falls, so the stow check refuses by default and the
//! only way past it is the operator saying so in as many words. That flag is
//! recorded in the summary: a release that happened away from stow is a fact
//! about the machine's history worth keeping, not a detail of how the command
//! was invoked.

use core::f64::consts::PI;
use core::time::Duration;

use reachy_kin::{
    HeadGeometry, IkError, LegAngles, inverse_kinematics, outside_limit, stow_head_pose,
};

use crate::arm::{angle_at, confirm_write};
use crate::joints::{JointId, JointVector, worst_joint, worst_row};
use crate::seq::{
    BusRequest, BusResult, RegId, RegValue, SeqAction, SeqError, SeqStep, Sequencer, StepContext,
};

/// Where the antennas are stowed, right then left, radians.
///
/// Folded back against the head rather than left standing: the antennas have no
/// travel limit of their own in the servo, and this is the far end of the range
/// the platform's own shutdown procedure uses.
pub const STOW_ANTENNAS: [f64; 2] = [-3.05, 3.05];

/// How long the platform is left to settle at stow before torque comes off.
pub const DEFAULT_STOW_DWELL: Duration = Duration::from_secs(2);

/// How far a joint may be from the stow pose and still count as being at it,
/// radians (2°).
///
/// Deliberately tighter than the tick's tracking threshold, because the two ask
/// different questions: tracking asks whether the position loop is keeping up
/// with a moving goal, and this asks where the head physically is before the
/// thing holding it up is switched off. Provisional — what error the legs settle
/// to under the head's weight at stow has not been measured — and wide enough
/// that a machine holding as well as its gains allow is not refused.
pub const DEFAULT_STOW_TOLERANCE: f64 = 2.0 * PI / 180.0;

/// The nine angles the stow pose puts the machine at: the stow head pose solved
/// through the linkage, the antennas folded, the body square.
///
/// Derived rather than transcribed, so the pose the stow motion is commanded to
/// and the pose disarming verifies against are the same pose by construction. An
/// `Err` means the configured geometry cannot reach stow at all, which is a
/// question about the geometry and not about the machine in front of you.
pub fn stow_targets(geom: &HeadGeometry) -> Result<JointVector, IkError> {
    let mut angles = LegAngles([0.0; 6]);
    inverse_kinematics(geom, &stow_head_pose(), &mut angles)?;
    Ok(JointVector {
        body_yaw: 0.0,
        legs: angles.0,
        antennas: STOW_ANTENNAS,
    })
}

/// What disarming needs to know.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DisarmConfig {
    /// The nine servo IDs, in bus order.
    pub ids: [u8; JointId::COUNT],
    /// The angles stow puts the machine at, which the measured joints are
    /// compared against.
    pub stow_targets: JointVector,
    /// How far a joint may be from its stow angle, radians.
    pub tolerance: f64,
    /// How long to let the platform settle between reaching stow and releasing.
    pub dwell: Duration,
    /// The operator's explicit acceptance that the head may fall. Without it a
    /// machine that is not measured at stow is refused.
    pub force_drop: bool,
}

/// What disarming found, and what it did.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DisarmSummary {
    /// The nine angles measured before torque came off.
    pub present: JointVector,
    /// How far each joint was from its stow angle, in bus order, radians.
    pub deviation: [f64; JointId::COUNT],
    /// Whether every joint was inside the tolerance. A summary reporting
    /// `false` exists only because the drop flag was set: torque was released
    /// away from stow, deliberately.
    pub at_stow: bool,
}

impl DisarmSummary {
    /// The joint furthest from its stow angle, and how far, radians.
    #[must_use]
    pub fn worst_deviation(&self) -> (JointId, f64) {
        worst_joint(&self.deviation)
    }
}

/// Which part of the sequence is running.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    Verify { cursor: usize },
    Dwell { waiting: bool },
    Release { cursor: usize },
    Complete,
    Failed(SeqError),
}

impl Phase {
    /// The phase name a failure here is reported under.
    fn step(self) -> SeqStep {
        match self {
            Self::Verify { .. } => SeqStep::VerifyAtStow,
            Self::Dwell { .. } => SeqStep::Dwell,
            Self::Release { .. } | Self::Complete => SeqStep::TorqueOff,
            // A failure already carries the phase it happened in; taking the
            // name from anywhere else would report a stow check that refused as
            // a torque-off that did not happen.
            Self::Failed(error) => error.context().step,
        }
    }
}

/// Disarming, as a state machine that touches no port.
///
/// Three phases in a fixed order: every joint measured against the stow pose,
/// the settle waited out, then torque released one servo at a time with each
/// release read back. The order is the safety property — nothing is written
/// until the platform is confirmed to be somewhere it can be left — and it lives
/// here, testable against scripted replies.
pub struct DisarmSequencer {
    cfg: DisarmConfig,
    phase: Phase,
    pending: Option<BusRequest>,
    present: JointVector,
    deviation: [f64; JointId::COUNT],
    at_stow: bool,
}

impl DisarmSequencer {
    /// A sequence ready to run against `cfg`.
    #[must_use]
    pub fn new(cfg: &DisarmConfig) -> Self {
        Self {
            cfg: *cfg,
            phase: Phase::Verify { cursor: 0 },
            pending: None,
            present: JointVector::default(),
            deviation: [0.0; JointId::COUNT],
            at_stow: false,
        }
    }

    fn read(&self, row: usize, reg: RegId) -> BusRequest {
        BusRequest::ReadReg {
            id: self.cfg.ids[row],
            reg,
        }
    }

    fn release(&self, row: usize) -> BusRequest {
        BusRequest::WriteRegVerified {
            id: self.cfg.ids[row],
            reg: RegId::TorqueEnable,
            value: RegValue::U8(0),
        }
    }

    fn context(&self, row: usize, reg: RegId) -> StepContext {
        StepContext::reg(self.phase.step(), self.cfg.ids[row], reg)
    }

    /// The next action, the previous one having been absorbed.
    fn emit(&mut self, now: Duration) -> SeqAction<DisarmSummary> {
        let request = match self.phase {
            Phase::Verify { cursor } => self.read(cursor, RegId::PresentPosition),
            Phase::Dwell { waiting } => {
                // The settle is waited once, on entry. A configured dwell of
                // zero is no dwell at all rather than a wait until now, which
                // would hand the driver a deadline it has already passed.
                if waiting && !self.cfg.dwell.is_zero() {
                    if let Phase::Dwell { waiting } = &mut self.phase {
                        *waiting = false;
                    }
                    return SeqAction::Wait {
                        until: now + self.cfg.dwell,
                    };
                }
                self.phase = Phase::Release { cursor: 0 };
                self.release(0)
            }
            Phase::Release { cursor } => self.release(cursor),
            Phase::Complete => {
                return SeqAction::Done(DisarmSummary {
                    present: self.present,
                    deviation: self.deviation,
                    at_stow: self.at_stow,
                });
            }
            Phase::Failed(error) => return SeqAction::Fail(error),
        };
        self.pending = Some(request);
        SeqAction::Transact(request)
    }

    /// Take the previous transaction's result and move the cursor on.
    fn absorb(&mut self, prior: Option<&BusResult>) -> Result<(), SeqError> {
        let Some(request) = self.pending.take() else {
            // Nothing was outstanding — the first call, or the call after the
            // dwell. A result handed back here answers no request.
            return Ok(());
        };
        let context = StepContext {
            step: self.phase.step(),
            id: request.id(),
            reg: request.reg(),
        };
        let Some(result) = prior else {
            // A transaction ran and nothing came back, which from here is
            // indistinguishable from silence on the wire.
            return Err(SeqError::NoAnswer { context });
        };
        match self.phase {
            Phase::Verify { cursor } => self.absorb_verify(cursor, context, result),
            Phase::Release { cursor } => {
                confirm_write(result, &request, context)?;
                self.phase = if cursor + 1 < JointId::COUNT {
                    Phase::Release { cursor: cursor + 1 }
                } else {
                    Phase::Complete
                };
                Ok(())
            }
            // Terminal, or waiting: nothing is ever outstanding in these.
            Phase::Dwell { .. } | Phase::Complete | Phase::Failed(_) => Ok(()),
        }
    }

    fn absorb_verify(
        &mut self,
        cursor: usize,
        context: StepContext,
        result: &BusResult,
    ) -> Result<(), SeqError> {
        let joint = JointId::ALL[cursor];
        let angle = result.value(context)?.radians(context)?;
        if !angle.is_finite() {
            // Refused whatever the drop flag says. The flag excuses a head that
            // is not at stow; it says nothing about a bus handing back numbers
            // that are not angles, and a release still has to be written and
            // read back over that same bus.
            return Err(SeqError::UnplaceableAngle {
                context,
                joint,
                angle,
            });
        }
        self.present.set(joint, angle);
        self.deviation[cursor] = (angle - angle_at(&self.cfg.stow_targets, cursor)).abs();
        if cursor + 1 < JointId::COUNT {
            self.phase = Phase::Verify { cursor: cursor + 1 };
            return Ok(());
        }

        // Every joint is measured before any verdict, so a refusal reports the
        // joint furthest from stow rather than the first one over the line.
        self.at_stow = !self
            .deviation
            .iter()
            .any(|deviation| outside_limit(*deviation, self.cfg.tolerance));
        if !self.at_stow && !self.cfg.force_drop {
            let row = worst_row(&self.deviation);
            return Err(SeqError::NotAtStow {
                context: self.context(row, RegId::PresentPosition),
                joint: JointId::ALL[row],
                present: angle_at(&self.present, row),
                target: angle_at(&self.cfg.stow_targets, row),
                tolerance: self.cfg.tolerance,
            });
        }
        self.phase = Phase::Dwell { waiting: true };
        Ok(())
    }
}

impl Sequencer for DisarmSequencer {
    type Summary = DisarmSummary;

    fn next(&mut self, now: Duration, prior: Option<&BusResult>) -> SeqAction<DisarmSummary> {
        if let Err(error) = self.absorb(prior) {
            self.phase = Phase::Failed(error);
        }
        self.emit(now)
    }

    fn step(&self) -> SeqStep {
        self.phase.step()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arm::SERVO_IDS;
    use crate::testutil::ScriptedBus;
    use reachy_kin::{EnvelopeConfig, EnvelopeReport, check_envelope, neutral_head_pose};

    fn config() -> DisarmConfig {
        DisarmConfig {
            ids: SERVO_IDS,
            stow_targets: stow_targets(&HeadGeometry::default()).expect("stow is reachable"),
            tolerance: DEFAULT_STOW_TOLERANCE,
            dwell: DEFAULT_STOW_DWELL,
            force_drop: false,
        }
    }

    /// The nine angles a head pose puts the machine at, antennas folded, body
    /// square.
    fn joints_at(pose: &nalgebra::Isometry3<f64>) -> JointVector {
        let mut angles = LegAngles([0.0; 6]);
        inverse_kinematics(&HeadGeometry::default(), pose, &mut angles)
            .expect("the pose is reachable");
        JointVector {
            body_yaw: 0.0,
            legs: angles.0,
            antennas: STOW_ANTENNAS,
        }
    }

    /// Nine servos holding torque at wherever the test puts them. The
    /// transaction log is what the order assertions read.
    #[derive(Clone, Debug)]
    struct Machine {
        present: JointVector,
        torque: [bool; JointId::COUNT],
        silent: [bool; JointId::COUNT],
        /// One write to answer with something other than success.
        fail_write: Option<(u8, BusResult)>,
        /// One read to answer with something other than success.
        fail_read: Option<(u8, BusResult)>,
        log: Vec<(SeqStep, BusRequest)>,
        waits: Vec<(Duration, Duration)>,
    }

    /// A platform holding itself at stow, torque on.
    fn bus() -> Machine {
        Machine {
            present: config().stow_targets,
            torque: [true; JointId::COUNT],
            silent: [false; JointId::COUNT],
            fail_write: None,
            fail_read: None,
            log: Vec::new(),
            waits: Vec::new(),
        }
    }

    impl ScriptedBus for Machine {
        fn answer(&mut self, step: SeqStep, request: BusRequest) -> BusResult {
            self.log.push((step, request));
            let row = SERVO_IDS
                .iter()
                .position(|id| *id == request.id())
                .expect("addressed to a servo on this bus");
            if self.silent[row] {
                return BusResult::NoAnswer;
            }
            let scripted = match request {
                BusRequest::WriteRegVerified { .. } => self.fail_write,
                _ => self.fail_read,
            };
            if let Some((id, result)) = scripted
                && request.id() == id
            {
                return result;
            }
            match request {
                BusRequest::ReadReg { .. } => {
                    BusResult::Value(RegValue::Radians(angle_at(&self.present, row)))
                }
                BusRequest::WriteRegVerified { value, .. } => {
                    if value == RegValue::U8(0) {
                        self.torque[row] = false;
                    }
                    BusResult::Written
                }
                BusRequest::Ping { .. } => panic!("disarming pings nothing"),
            }
        }

        fn waited(&mut self, now: Duration, until: Duration) {
            self.waits.push((now, until));
        }
    }

    impl Machine {
        fn writes(&self) -> Vec<(u8, RegValue)> {
            self.log
                .iter()
                .filter_map(|(_, request)| match request {
                    BusRequest::WriteRegVerified { id, value, .. } => Some((*id, *value)),
                    _ => None,
                })
                .collect()
        }
    }

    /// The shared driver, against this crate's disarming sequencer.
    fn drive(cfg: &DisarmConfig, machine: &mut Machine) -> Result<DisarmSummary, SeqError> {
        let mut seq = DisarmSequencer::new(cfg);
        crate::testutil::drive(&mut seq, machine)
    }

    /// The order is the whole safety property: every joint is measured, the
    /// settle is waited out, and only then does torque come off — servo by
    /// servo, in bus order, each release read back.
    #[test]
    fn torque_comes_off_only_after_the_stow_check_and_the_dwell() {
        let cfg = config();
        let mut machine = bus();
        let summary = drive(&cfg, &mut machine).expect("a machine at stow disarms");

        let steps: Vec<SeqStep> = machine.log.iter().map(|(step, _)| *step).collect();
        assert_eq!(
            steps,
            [
                vec![SeqStep::VerifyAtStow; JointId::COUNT],
                vec![SeqStep::TorqueOff; JointId::COUNT],
            ]
            .concat()
        );
        assert_eq!(
            machine.writes(),
            SERVO_IDS
                .iter()
                .map(|id| (*id, RegValue::U8(0)))
                .collect::<Vec<_>>()
        );
        assert_eq!(machine.torque, [false; JointId::COUNT]);
        assert_eq!(machine.waits.len(), 1, "the settle is waited exactly once");

        assert!(summary.at_stow);
        assert_eq!(summary.present, cfg.stow_targets);
        assert_eq!(summary.deviation, [0.0; JointId::COUNT]);
        assert_eq!(summary.worst_deviation(), (JointId::BodyYaw, 0.0));
    }

    /// A joint away from stow stops the sequence with nothing written: the head
    /// is not where it can be left, and releasing torque would drop it.
    #[test]
    fn a_joint_away_from_stow_stops_the_release() {
        let cfg = config();
        let mut machine = bus();
        machine.present.legs[2] += 5.0_f64.to_radians();

        let error = drive(&cfg, &mut machine).expect_err("five degrees is past the gate");
        let SeqError::NotAtStow {
            context,
            joint,
            present,
            target,
            tolerance,
        } = error
        else {
            panic!("expected a stow refusal, got {error}");
        };
        assert_eq!(joint, JointId::Leg(2));
        assert_eq!(context.id, SERVO_IDS[3]);
        assert_eq!(context.step, SeqStep::VerifyAtStow);
        assert!((present - target).abs() > tolerance);
        assert!(
            error.to_string().contains("leg 3"),
            "the message names the crank the way the envelope does: {error}"
        );

        assert!(machine.writes().is_empty(), "nothing was written");
        assert_eq!(machine.torque, [true; JointId::COUNT]);
        assert!(machine.waits.is_empty(), "the settle was never entered");
    }

    /// Every joint is measured before the verdict, so the one named is the one
    /// furthest from stow rather than the first one found over the line.
    #[test]
    fn the_joint_named_is_the_one_furthest_from_stow() {
        let cfg = config();
        let mut machine = bus();
        machine.present.legs[1] += 3.0_f64.to_radians();
        machine.present.antennas[1] -= 9.0_f64.to_radians();

        let error = drive(&cfg, &mut machine).expect_err("both are past the gate");
        let SeqError::NotAtStow { joint, .. } = error else {
            panic!("expected a stow refusal, got {error}");
        };
        assert_eq!(joint, JointId::AntennaLeft);
    }

    /// The drop flag is the operator accepting that the head falls: the check
    /// still runs and still records what it found, and the release proceeds.
    #[test]
    fn the_drop_flag_releases_from_anywhere() {
        let cfg = DisarmConfig {
            force_drop: true,
            ..config()
        };
        let mut machine = bus();
        machine.present = joints_at(&neutral_head_pose());

        let summary = drive(&cfg, &mut machine).expect("the flag excuses the stow check");
        assert!(!summary.at_stow, "the machine was not at stow and says so");
        assert_eq!(summary.present, machine.present);
        assert_eq!(machine.torque, [false; JointId::COUNT]);

        let (joint, deviation) = summary.worst_deviation();
        let row = JointId::ALL
            .iter()
            .position(|id| *id == joint)
            .expect("a joint of this bus");
        assert_eq!(
            deviation,
            (angle_at(&machine.present, row) - angle_at(&cfg.stow_targets, row)).abs()
        );
        assert!(deviation > cfg.tolerance);
    }

    /// A reading that is not an angle is refused whichever way the drop flag is
    /// set: the flag excuses a head away from stow, not a bus handing back
    /// numbers nothing can be decided from.
    #[test]
    fn a_reading_nobody_can_place_is_refused_even_with_the_drop_flag() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            for force_drop in [false, true] {
                let cfg = DisarmConfig {
                    force_drop,
                    ..config()
                };
                let mut machine = bus();
                machine.present.antennas[0] = value;

                let error = drive(&cfg, &mut machine)
                    .expect_err("a reading that is not an angle places no joint");
                let SeqError::UnplaceableAngle { joint, angle, .. } = error else {
                    panic!("expected an unplaceable reading, got {error}");
                };
                assert_eq!(joint, JointId::AntennaRight);
                // Bit patterns, not `is_nan`: an infinity compared that way
                // constrains nothing, and an infinity is exactly what a
                // decode-scale slip hands back. The number the refusal carries
                // is the number the bus produced.
                assert_eq!(angle.to_bits(), value.to_bits());
                assert!(machine.writes().is_empty(), "nothing was written");
            }
        }
    }

    /// The settle is one wait of the configured length, between the last read
    /// and the first release. A dwell of zero is no wait at all rather than a
    /// deadline the driver has already passed.
    #[test]
    fn the_dwell_is_waited_once_at_the_configured_length() {
        let cfg = config();
        let mut machine = bus();
        drive(&cfg, &mut machine).expect("a machine at stow disarms");
        assert_eq!(machine.waits.len(), 1);
        let (from, until) = machine.waits[0];
        assert_eq!(until - from, DEFAULT_STOW_DWELL);

        let brisk = DisarmConfig {
            dwell: Duration::ZERO,
            ..config()
        };
        let mut machine = bus();
        drive(&brisk, &mut machine).expect("a machine at stow disarms");
        assert!(machine.waits.is_empty());
        assert_eq!(machine.torque, [false; JointId::COUNT]);
    }

    /// A release the servo refuses stops the sequence where it stood: the servos
    /// already released stay released, and the rest keep holding.
    #[test]
    fn a_refused_release_stops_where_it_stood() {
        let cfg = config();
        let mut machine = bus();
        machine.fail_write = Some((SERVO_IDS[4], BusResult::ServoError { code: 0x04 }));

        let error = drive(&cfg, &mut machine).expect_err("the release was refused");
        let SeqError::Refused { context, code } = error else {
            panic!("expected a refusal, got {error}");
        };
        assert_eq!(code, 0x04);
        assert_eq!(context.id, SERVO_IDS[4]);
        assert_eq!(context.step, SeqStep::TorqueOff);
        assert_eq!(context.reg, Some(RegId::TorqueEnable));

        assert_eq!(
            machine.torque,
            [false, false, false, false, true, true, true, true, true]
        );
    }

    /// A servo that does not answer the stow check stops the sequence before
    /// anything is written: a joint nobody can measure is a joint nobody can
    /// place at stow.
    #[test]
    fn a_silent_servo_at_the_stow_check_refuses() {
        let cfg = config();
        let mut machine = bus();
        machine.silent[6] = true;

        let error = drive(&cfg, &mut machine).expect_err("a silent servo stops the check");
        let SeqError::NoAnswer { context } = error else {
            panic!("expected silence, got {error}");
        };
        assert_eq!(context.id, SERVO_IDS[6]);
        assert_eq!(context.step, SeqStep::VerifyAtStow);
        assert!(machine.writes().is_empty());
        assert_eq!(machine.torque, [true; JointId::COUNT]);
    }

    /// A driver that runs a transaction and brings nothing back is reported as
    /// silence rather than quietly retried: from in here the two are the same
    /// observation, and inventing a retry would be the sequencer deciding a
    /// policy that belongs to whoever owns the port.
    #[test]
    fn a_driver_that_brings_nothing_back_is_silence() {
        let cfg = config();
        let mut seq = DisarmSequencer::new(&cfg);
        let first = seq.next(Duration::ZERO, None);
        assert!(matches!(first, SeqAction::Transact(_)));

        let SeqAction::Fail(error) = seq.next(Duration::ZERO, None) else {
            panic!("expected a failure after an unanswered transaction");
        };
        assert!(matches!(error, SeqError::NoAnswer { .. }));
        assert_eq!(seq.step(), SeqStep::VerifyAtStow);
    }

    /// The stow pose disarming verifies against is a pose the envelope admits
    /// and the servos' own travel windows contain. It is the last pose the
    /// machine is commanded to before torque comes off, so a stow target the
    /// envelope refused would be a stow command that faults on its first tick.
    #[test]
    fn the_stow_targets_are_a_pose_the_envelope_admits() {
        let cfg = config();
        let env = EnvelopeConfig::default();
        let mut report = EnvelopeReport::default();
        check_envelope(
            &HeadGeometry::default(),
            &env,
            &stow_head_pose(),
            0.0,
            STOW_ANTENNAS,
            None,
            &mut report,
        )
        .expect("stow is inside the envelope");

        for (leg, (angle, (low, high))) in report
            .leg_angles
            .expect("stow is reachable")
            .0
            .iter()
            .zip(env.crank_windows)
            .enumerate()
        {
            assert!(
                *angle > low && *angle < high,
                "leg {} stows at {:.3}°, outside [{:.3}°, {:.3}°]",
                leg + 1,
                angle.to_degrees(),
                low.to_degrees(),
                high.to_degrees()
            );
        }
        assert_eq!(cfg.stow_targets.antennas, STOW_ANTENNAS);
        assert_eq!(cfg.stow_targets.legs, report.leg_angles.unwrap().0);
        assert_eq!(cfg.stow_targets.body_yaw, 0.0);
    }
}
