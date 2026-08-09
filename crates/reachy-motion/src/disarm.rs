//! Disarming: the verified torque-off tail, and nothing else.
//!
//! Stowing the platform is not this sequencer's job. The motion to the stow pose
//! is an ordinary command through the tick — interpolated, envelope-checked on
//! every tick, emitted as bounded increments — because it is the one path where
//! the head travels near the bottom of its range and the checks that matter
//! there are the tick's. What is left over is what cannot be expressed as a
//! trajectory: let the platform settle, measure where stow left it, and release
//! torque servo by servo with each release read back.
//!
//! ## The settle comes before the measurement
//!
//! A move is over when its trajectory ends, not when the joints arrive, so a
//! joint still closing the lag a proportional loop runs is legitimately short
//! of its fold at the moment `off` starts — and `demo` reaches here at exactly
//! that moment. The dwell exists to let the machine settle; waiting it out
//! first is what makes the measurement that follows a measurement of where the
//! head came to rest.
//!
//! ## Nothing happens after the last release
//!
//! Once torque is off the platform settles limp into its true rest, and that
//! rest is what the next arming will find and pin. So the sequence ends there:
//! no re-read, no confirmation pose, no tidying write. Anything issued after the
//! release would be describing a machine that is still moving.
//!
//! ## Nothing gates the release
//!
//! Every phase here reports; none of them refuses. The measurement against stow
//! says where the head was when torque came off, and a joint found somewhere
//! else — or one that will not answer at all — is carried in the summary rather
//! than stopping the sequence. A release write that goes unacknowledged is
//! recorded and the walk carries on to the next servo, so all nine are always
//! asked. The reason is the machine: stowed with torque held is its only pinch
//! hazard, the head falls gently into near-stow under gearbox resistance
//! wherever it is released from, and there is nothing a refusal here could be
//! protecting.
//!
//! ## Two ways in
//!
//! [`DisarmSequencer::new`] is the orderly release — settle, measure, release —
//! and it is what an expected ending runs: a commanded stow has just finished
//! and the machine is trusted to be where it was told to go, so there is time
//! to write down where that was.
//!
//! [`DisarmSequencer::immediate`] skips both and goes straight to the nine
//! release writes. It is what a *fault* runs, and the reasoning is the same
//! reasoning that makes the orderly form report rather than refuse: a fault
//! means position feedback or motor control is no longer trusted, so the dwell
//! is time spent holding torque for a measurement nobody could act on and the
//! measurement itself is of doubtful provenance. The summary comes back with
//! every joint unmeasured, which is the truth about what it looked at.

use core::f64::consts::PI;
use core::time::Duration;

use reachy_kin::{
    HeadGeometry, IkError, LegAngles, inverse_kinematics, outside_limit, stow_head_pose, wrap_to_pi,
};

use crate::arm::{angle_at, confirm_write, placeable};
use crate::joints::{JointId, JointVector, worst_joint};
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
}

/// Which of the two releases ran.
///
/// On the summary rather than inferred from it: a release that measured nothing
/// is either one that deliberately did not look or one that looked at nine
/// servos and got nine silences, and those call for opposite things from
/// whoever reads the report.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ReleaseForm {
    /// Settle, measure, release — [`DisarmSequencer::new`].
    Orderly,
    /// The nine release writes and nothing else — [`DisarmSequencer::immediate`].
    Immediate,
}

/// What disarming found, and what it did.
///
/// Every field is a report. Torque comes off whatever any of them say, so a
/// summary describing a machine away from stow, a joint that would not answer,
/// or a servo whose release went unacknowledged is the record of a release that
/// happened anyway — the thing a caller alerts on, not a thing it could have
/// prevented.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DisarmSummary {
    /// Which release ran, which is what says whether the measurement fields
    /// below describe a failed look or no look at all.
    pub form: ReleaseForm,
    /// The nine angles measured before torque came off. A joint whose read did
    /// not land keeps whatever the vector held, and `unmeasured` is what says
    /// so.
    pub present: JointVector,
    /// How far each joint was from its stow angle, in bus order, radians;
    /// circular on the antennas, linear elsewhere. Zero for a joint that was
    /// not measured.
    pub deviation: [f64; JointId::COUNT],
    /// Why each joint's position read produced no angle, in bus order, and
    /// `None` where it produced one.
    ///
    /// The cause and not a bit: a servo gone silent, one refusing with a status
    /// code, a frame mangled on the wire and a reading that is not a number are
    /// four different problems with four different answers, and this is the one
    /// diagnostic an unconditional release still produces. All `None` in the
    /// immediate form, which read nothing and so failed at nothing — [`form`]
    /// is what distinguishes that.
    ///
    /// [`form`]: DisarmSummary::form
    pub unmeasured: [Option<SeqError>; JointId::COUNT],
    /// Whether each servo acknowledged its torque-off write.
    pub released: [bool; JointId::COUNT],
    /// Whether every joint was measured and every one inside the tolerance.
    pub at_stow: bool,
}

impl DisarmSummary {
    /// The joint furthest from its stow angle, and how far, radians.
    #[must_use]
    pub fn worst_deviation(&self) -> (JointId, f64) {
        worst_joint(&self.deviation)
    }

    /// Whether the release looked at the machine before letting go.
    #[must_use]
    pub fn looked(&self) -> bool {
        matches!(self.form, ReleaseForm::Orderly)
    }

    /// Whether the joint in bus row `row` was measured.
    #[must_use]
    pub fn measured(&self, row: usize) -> bool {
        self.looked() && self.unmeasured[row].is_none()
    }

    /// Whether every joint's position read landed.
    #[must_use]
    pub fn measured_all(&self) -> bool {
        (0..JointId::COUNT).all(|row| self.measured(row))
    }

    /// The joints a release looked at and could not read, each with why, in bus
    /// order. Empty for a release that did not look.
    pub fn unreadable(&self) -> impl Iterator<Item = (JointId, SeqError)> + '_ {
        self.unmeasured
            .iter()
            .enumerate()
            .filter(move |_| self.looked())
            .filter_map(|(row, cause)| cause.map(|cause| (JointId::ALL[row], cause)))
    }

    /// Whether all nine servos acknowledged their torque-off write.
    #[must_use]
    pub fn all_released(&self) -> bool {
        self.released.iter().all(|released| *released)
    }

    /// The joints whose torque-off write went unacknowledged, in bus order.
    ///
    /// These are the servos that may still be holding: the write was issued and
    /// retried by the bus, and nothing came back to say it landed.
    pub fn unreleased(&self) -> impl Iterator<Item = JointId> + '_ {
        self.released
            .iter()
            .enumerate()
            .filter(|(_, released)| !**released)
            .map(|(row, _)| JointId::ALL[row])
    }
}

/// Whether a pose measured elsewhere is at stow, joint by joint, within
/// `cfg.tolerance`.
///
/// The comparison the orderly release makes, offered to a caller holding a
/// sweep of its own: a resting watch can then tell a machine already folded
/// from one a crash or a hand left standing, and re-stow the second without
/// asking anything about the first. Never a verdict about whether anything may
/// happen — where the machine stands gates nothing — only about whether there
/// is something to put right.
///
/// Must agree with the sequencer's release verdict about where stow is:
/// an angle nobody can place is not evidence that the head is folded.
#[must_use]
pub fn at_stow(cfg: &DisarmConfig, present: &JointVector) -> bool {
    (0..JointId::COUNT).all(|row| {
        !outside_limit(
            deviation_from(
                JointId::ALL[row],
                angle_at(present, row),
                angle_at(&cfg.stow_targets, row),
            ),
            cfg.tolerance,
        )
    })
}

/// How far one joint is from its stow angle, radians, never negative.
///
/// Circular for the antennas and linear for everything else. An antenna is a
/// free rotor whose reading is a direction in a continuous frame: a machine
/// physically folded is at its fold whichever turn the reading sits on, and a
/// linear difference there reports the turns rather than the distance — run 4's
/// refusal put an antenna 231° from a fold it was 129° from. The legs and the
/// body are bounded joints working far from the half turn, where the two agree
/// and the linear form is the honest one: a leg reading a turn away from its
/// target is a broken reading, not a joint at its target.
fn deviation_from(joint: JointId, present: f64, target: f64) -> f64 {
    match joint {
        JointId::AntennaRight | JointId::AntennaLeft => wrap_to_pi(present - target).abs(),
        _ => (present - target).abs(),
    }
}

/// Which part of the sequence is running.
///
/// There is no failed phase: nothing in this sequence refuses, so the only way
/// out is through the releases.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Phase {
    Dwell { waiting: bool },
    Verify { cursor: usize },
    Release { cursor: usize },
    Complete,
}

impl Phase {
    /// The phase name this part of the sequence is reported under.
    fn step(self) -> SeqStep {
        match self {
            Self::Verify { .. } => SeqStep::VerifyAtStow,
            Self::Dwell { .. } => SeqStep::Dwell,
            Self::Release { .. } | Self::Complete => SeqStep::TorqueOff,
        }
    }
}

/// Disarming, as a state machine that touches no port.
///
/// Three phases in a fixed order: the settle waited out, every joint then
/// measured against the stow pose, then torque released one servo at a time with
/// each release read back. The order is what makes the measurement mean
/// something — it describes where the head was at the moment torque left it —
/// and it lives here, testable against scripted replies.
pub struct DisarmSequencer {
    cfg: DisarmConfig,
    form: ReleaseForm,
    phase: Phase,
    pending: Option<BusRequest>,
    present: JointVector,
    deviation: [f64; JointId::COUNT],
    unmeasured: [Option<SeqError>; JointId::COUNT],
    released: [bool; JointId::COUNT],
    at_stow: bool,
}

impl DisarmSequencer {
    /// A sequence ready to run against `cfg`: settle, measure, release.
    #[must_use]
    pub fn new(cfg: &DisarmConfig) -> Self {
        Self::from(cfg, ReleaseForm::Orderly, Phase::Dwell { waiting: true })
    }

    /// A sequence that writes the nine releases and nothing else.
    ///
    /// The fault response. No settle and no measurement: whatever is wrong with
    /// the machine, the answer is that torque comes off now, and both of the
    /// things this skips exist only to describe a machine whose description can
    /// still be believed. The summary reports every joint unmeasured and
    /// `at_stow` false — not a verdict about where the head is, a statement
    /// that nobody looked.
    #[must_use]
    pub fn immediate(cfg: &DisarmConfig) -> Self {
        Self::from(cfg, ReleaseForm::Immediate, Phase::Release { cursor: 0 })
    }

    fn from(cfg: &DisarmConfig, form: ReleaseForm, phase: Phase) -> Self {
        Self {
            cfg: *cfg,
            form,
            phase,
            pending: None,
            present: JointVector::default(),
            deviation: [0.0; JointId::COUNT],
            unmeasured: [None; JointId::COUNT],
            released: [false; JointId::COUNT],
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

    /// The next action, the previous one having been absorbed.
    fn emit(&mut self, now: Duration) -> SeqAction<DisarmSummary> {
        let request = match self.phase {
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
                self.phase = Phase::Verify { cursor: 0 };
                self.read(0, RegId::PresentPosition)
            }
            Phase::Verify { cursor } => self.read(cursor, RegId::PresentPosition),
            Phase::Release { cursor } => self.release(cursor),
            Phase::Complete => {
                return SeqAction::Done(DisarmSummary {
                    form: self.form,
                    present: self.present,
                    deviation: self.deviation,
                    unmeasured: self.unmeasured,
                    released: self.released,
                    at_stow: self.at_stow,
                });
            }
        };
        self.pending = Some(request);
        SeqAction::Transact(request)
    }

    /// Take the previous transaction's result and move the cursor on.
    ///
    /// Infallible by construction. A transaction that brought nothing back, a
    /// value of the wrong shape, a servo error, a reading that is not a number:
    /// each is recorded against its own servo, with its cause, and the cursor
    /// advances. Torque coming off the other eight is worth more than stopping
    /// over the ninth — but the reason the ninth could not be described is kept,
    /// because it is the only thing distinguishing an unplugged servo from an
    /// overloaded one.
    fn absorb(&mut self, prior: Option<&BusResult>) {
        let Some(request) = self.pending.take() else {
            // Nothing was outstanding — the first call, or the call after the
            // dwell. A result handed back here answers no request.
            return;
        };
        let context = StepContext {
            step: self.phase.step(),
            id: request.id(),
            reg: request.reg(),
        };
        match self.phase {
            Phase::Verify { cursor } => self.absorb_verify(cursor, context, prior),
            Phase::Release { cursor } => {
                self.released[cursor] =
                    prior.is_some_and(|result| confirm_write(result, &request, context).is_ok());
                self.phase = if cursor + 1 < JointId::COUNT {
                    Phase::Release { cursor: cursor + 1 }
                } else {
                    Phase::Complete
                };
            }
            // Terminal, or waiting: nothing is ever outstanding in these.
            Phase::Dwell { .. } | Phase::Complete => {}
        }
    }

    fn absorb_verify(&mut self, cursor: usize, context: StepContext, prior: Option<&BusResult>) {
        let joint = JointId::ALL[cursor];
        let angle = prior
            .ok_or(SeqError::NoAnswer { context })
            .and_then(|result| placeable(cursor, context, result));
        match angle {
            Ok(angle) => {
                self.present.set(joint, angle);
                self.deviation[cursor] =
                    deviation_from(joint, angle, angle_at(&self.cfg.stow_targets, cursor));
            }
            Err(cause) => self.unmeasured[cursor] = Some(cause),
        }
        if cursor + 1 < JointId::COUNT {
            self.phase = Phase::Verify { cursor: cursor + 1 };
            return;
        }

        // Every joint is measured before the verdict, so the summary describes
        // the whole machine rather than stopping at the first joint over the
        // line. A joint nobody could read is not at stow as far as this says:
        // the claim is that the head was found where it can be left, and an
        // unread joint is no evidence of that.
        self.at_stow = self.unmeasured.iter().all(|cause| cause.is_none())
            && !self
                .deviation
                .iter()
                .any(|deviation| outside_limit(*deviation, self.cfg.tolerance));
        self.phase = Phase::Release { cursor: 0 };
    }
}

impl Sequencer for DisarmSequencer {
    type Summary = DisarmSummary;

    fn next(&mut self, now: Duration, prior: Option<&BusResult>) -> SeqAction<DisarmSummary> {
        self.absorb(prior);
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
        /// How many transactions had run when each wait began, which is what
        /// places the settle in the order rather than merely counting it.
        waited_after: Vec<usize>,
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
            waited_after: Vec::new(),
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
            self.waited_after.push(self.log.len());
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
    fn drive(
        cfg: &DisarmConfig,
        machine: &mut Machine,
    ) -> Result<DisarmSummary, crate::seq::SeqError> {
        let mut seq = DisarmSequencer::new(cfg);
        crate::testutil::drive(&mut seq, machine)
    }

    /// The order is the whole safety property: the settle is waited out, every
    /// joint is then measured, and only then does torque come off — servo by
    /// servo, in bus order, each release read back.
    #[test]
    fn the_dwell_precedes_the_stow_check_and_torque_comes_off_last() {
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
        assert_eq!(
            machine.waited_after,
            [0],
            "the settle runs before anything is read: the gate judges the \
             machine where it came to rest"
        );

        assert!(summary.at_stow);
        assert_eq!(summary.form, ReleaseForm::Orderly);
        assert_eq!(summary.present, cfg.stow_targets);
        assert_eq!(summary.deviation, [0.0; JointId::COUNT]);
        assert!(summary.measured_all());
        assert_eq!(summary.unreadable().count(), 0);
        assert_eq!(summary.released, [true; JointId::COUNT]);
        assert!(summary.all_released());
        assert_eq!(summary.unreleased().count(), 0);
        assert_eq!(summary.worst_deviation(), (JointId::BodyYaw, 0.0));
    }

    /// A joint away from stow is reported and torque comes off anyway. The head
    /// resting a few degrees off its fold is not a hazard; the head held up by
    /// torque nobody is watching is the one this machine has.
    #[test]
    fn a_joint_away_from_stow_is_released_and_reported() {
        let cfg = config();
        let mut machine = bus();
        machine.present.legs[2] += 5.0_f64.to_radians();

        let summary = drive(&cfg, &mut machine).expect("nothing here refuses");
        assert!(!summary.at_stow, "five degrees is past the tolerance");
        assert!(summary.measured_all());
        assert_eq!(summary.worst_deviation().0, JointId::Leg(2));
        assert!((summary.deviation[3].to_degrees() - 5.0).abs() < 1e-9);

        assert_eq!(
            machine.writes(),
            SERVO_IDS
                .iter()
                .map(|id| (*id, RegValue::U8(0)))
                .collect::<Vec<_>>()
        );
        assert_eq!(machine.torque, [false; JointId::COUNT]);
        assert!(summary.all_released());
    }

    /// Every joint is measured before the verdict, so the joint the summary
    /// names is the one furthest from stow rather than the first one over the
    /// line.
    #[test]
    fn the_joint_named_is_the_one_furthest_from_stow() {
        let cfg = config();
        let mut machine = bus();
        machine.present.legs[1] += 3.0_f64.to_radians();
        machine.present.antennas[1] -= 9.0_f64.to_radians();

        let summary = drive(&cfg, &mut machine).expect("nothing here refuses");
        assert!(!summary.at_stow);
        assert_eq!(summary.worst_deviation().0, JointId::AntennaLeft);
        assert_eq!(machine.torque, [false; JointId::COUNT]);
    }

    /// Released from the neutral pose — the head up, as far from stow as this
    /// machine goes. The summary carries where it was; the release happens.
    #[test]
    fn a_release_from_anywhere_releases() {
        let cfg = config();
        let mut machine = bus();
        machine.present = joints_at(&neutral_head_pose());

        let summary = drive(&cfg, &mut machine).expect("nothing here refuses");
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

    /// An antenna is judged on where it physically points, not on which turn its
    /// reading sits on. A machine whose right antenna is folded 23° short of its
    /// −174.75° fold but reads it from the other side of the half turn, at
    /// +162.25°, is 23° from stow — not the 337° a linear difference reports and
    /// refuses on.
    ///
    /// Both antennas, and each read across the half turn from its own fold: a
    /// reading on the same side of it needs no wrapping to come out right, and
    /// `stow` folds the two symmetrically, so a rule reaching only one of them
    /// would report every release of a machine whose other antenna was found a
    /// turn out as a release away from stow.
    #[test]
    fn an_antenna_is_judged_around_the_circle() {
        let stow = stow_targets(&HeadGeometry::default()).expect("stow is reachable");
        let wide = DisarmConfig {
            tolerance: 25.0_f64.to_radians(),
            ..config()
        };

        let cases = [
            (0usize, 7usize, 162.248_f64, JointId::AntennaRight),
            (1, 8, -162.248, JointId::AntennaLeft),
        ];
        for (side, row, degrees, joint) in cases {
            let mut machine = bus();
            machine.present.antennas[side] = degrees.to_radians();

            let summary = drive(&wide, &mut machine).expect("23° is inside a 25° gate");
            assert!(summary.at_stow, "{joint}");
            assert!(
                (summary.deviation[row].to_degrees() - 23.0).abs() < 1e-3,
                "the distance around the circle is {}°",
                summary.deviation[row].to_degrees()
            );
            assert_eq!(machine.torque, [false; JointId::COUNT]);

            // The same reading against the 2° tolerance reads as away from
            // stow, and by the circular figure: nothing here waives the
            // measurement, it measures it.
            let mut machine = bus();
            machine.present.antennas[side] = degrees.to_radians();
            let summary = drive(&config(), &mut machine).expect("nothing here refuses");
            assert!(!summary.at_stow, "23° is past a 2° tolerance");
            assert_eq!(summary.worst_deviation().0, joint);
            assert!(
                (summary.deviation[row].to_degrees() - 23.0).abs() < 1e-3,
                "the report carries the distance around the circle: {}°",
                summary.deviation[row].to_degrees()
            );
            assert_eq!(machine.torque, [false; JointId::COUNT]);
        }

        // A leg keeps the linear difference: it is a windowed joint working far
        // from the half turn, and a reading a whole turn from its target is a
        // broken reading rather than a leg at stow.
        let mut machine = bus();
        machine.present.legs[0] += core::f64::consts::TAU;
        let summary = drive(&wide, &mut machine).expect("nothing here refuses");
        assert!(!summary.at_stow, "a turn is not zero on a leg");
        assert_eq!(summary.worst_deviation().0, JointId::Leg(0));
        assert!((summary.present.legs[0] - stow.legs[0] - core::f64::consts::TAU).abs() < 1e-12);
    }

    /// A reading that is not an angle places no joint, so it is carried as
    /// unmeasured — and torque still comes off. A bus handing back numbers
    /// nothing can be decided from is a reason to leave the machine limp, not a
    /// reason to leave it holding.
    #[test]
    fn a_reading_nobody_can_place_is_reported_and_the_release_runs() {
        for value in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let cfg = config();
            let mut machine = bus();
            machine.present.antennas[0] = value;

            let summary = drive(&cfg, &mut machine).expect("nothing here refuses");
            assert!(!summary.measured(7), "the right antenna placed nothing");
            assert_eq!(summary.deviation[7], 0.0);
            assert!(
                !summary.at_stow,
                "an unread joint is no evidence the head can be left here"
            );
            // And the summary says *why* it placed nothing, which is what tells
            // a decode-scale slip from a servo that has stopped answering.
            assert!(
                matches!(
                    summary.unmeasured[7],
                    Some(SeqError::UnplaceableAngle {
                        joint: JointId::AntennaRight,
                        angle,
                        ..
                    }) if angle.to_bits() == value.to_bits()
                ),
                "{:?}",
                summary.unmeasured[7]
            );
            assert_eq!(
                summary
                    .unreadable()
                    .map(|(joint, _)| joint)
                    .collect::<Vec<_>>(),
                vec![JointId::AntennaRight]
            );
            assert_eq!(machine.torque, [false; JointId::COUNT]);
            assert!(summary.all_released());
        }
    }

    /// The settle is one wait of the configured length, before the first read.
    /// A dwell of zero is no wait at all rather than a deadline the driver has
    /// already passed, and skipping it skips nothing else: the check and the
    /// releases still run in order.
    #[test]
    fn the_dwell_is_waited_once_at_the_configured_length() {
        let cfg = config();
        let mut machine = bus();
        drive(&cfg, &mut machine).expect("a machine at stow disarms");
        assert_eq!(machine.waits.len(), 1);
        let (from, until) = machine.waits[0];
        assert_eq!(from, Duration::ZERO);
        assert_eq!(until - from, DEFAULT_STOW_DWELL);

        let brisk = DisarmConfig {
            dwell: Duration::ZERO,
            ..config()
        };
        let mut machine = bus();
        drive(&brisk, &mut machine).expect("a machine at stow disarms");
        assert!(machine.waits.is_empty());
        assert_eq!(machine.torque, [false; JointId::COUNT]);
        assert_eq!(machine.log.len(), 2 * JointId::COUNT);
    }

    /// A release the servo refuses is recorded against that servo and the walk
    /// carries on: the other eight come off, and the summary names the one that
    /// may still be holding.
    #[test]
    fn a_refused_release_is_recorded_and_the_rest_still_come_off() {
        let cfg = config();
        let mut machine = bus();
        machine.fail_write = Some((SERVO_IDS[4], BusResult::ServoError { code: 0x04 }));

        let summary = drive(&cfg, &mut machine).expect("nothing here refuses");
        assert_eq!(
            summary.released,
            [true, true, true, true, false, true, true, true, true]
        );
        assert!(!summary.all_released());
        assert_eq!(
            summary.unreleased().collect::<Vec<_>>(),
            vec![JointId::Leg(3)]
        );

        // Nine writes went out, one per servo, in bus order.
        assert_eq!(
            machine.writes(),
            SERVO_IDS
                .iter()
                .map(|id| (*id, RegValue::U8(0)))
                .collect::<Vec<_>>()
        );
        assert_eq!(
            machine.torque,
            [false, false, false, false, true, false, false, false, false]
        );
    }

    /// A servo that does not answer the stow check is carried as unmeasured and
    /// the releases still run — including its own, which is the write that
    /// matters most for a servo the bus is having trouble with.
    #[test]
    fn a_silent_servo_at_the_stow_check_does_not_stop_the_release() {
        let cfg = config();
        let mut machine = bus();
        machine.silent[6] = true;

        let summary = drive(&cfg, &mut machine).expect("nothing here refuses");
        assert!(!summary.measured(6));
        assert!(
            matches!(summary.unmeasured[6], Some(SeqError::NoAnswer { .. })),
            "silence is reported as silence: {:?}",
            summary.unmeasured[6]
        );
        assert!(!summary.at_stow);
        assert_eq!(machine.writes().len(), JointId::COUNT);
        // The silent servo never acknowledges its release either; the other
        // eight are limp.
        assert!(!summary.released[6]);
        assert_eq!(
            summary.unreleased().collect::<Vec<_>>(),
            vec![JointId::Leg(5)]
        );
        assert_eq!(
            machine.torque,
            [false, false, false, false, false, false, true, false, false]
        );
    }

    /// Each way a stow reading can fail comes back as its own cause.
    ///
    /// `unmeasured: leg 3` is the whole of what an unconditional release tells
    /// on-call about a joint it could not read, and a servo gone silent, one
    /// refusing with a status code and a frame the wire mangled are three
    /// problems with three different answers. Collapsing them to a bit would
    /// throw away the one diagnostic this path still produces.
    #[test]
    fn every_way_a_stow_reading_fails_is_reported_as_itself() {
        // Asserted on what the cause says, because what it says is what reaches
        // whoever has to act on it.
        for (answer, expected) in [
            (BusResult::NoAnswer, "no answer"),
            (
                BusResult::ServoError { code: 0x20 },
                "refused with status code 0x20",
            ),
            (BusResult::WireCorrupt, "the reply was corrupt on the wire"),
        ] {
            let cfg = config();
            let mut machine = bus();
            machine.fail_read = Some((SERVO_IDS[4], answer));

            let summary = drive(&cfg, &mut machine).expect("nothing here refuses");
            let cause = summary.unmeasured[4].expect("leg 4 could not be read");
            assert!(
                cause.to_string().contains(expected),
                "{answer:?} came back as {cause}"
            );
            assert_eq!(cause.context().id, SERVO_IDS[4], "named against its servo");
            assert_eq!(cause.context().step, SeqStep::VerifyAtStow);
            assert!(!summary.at_stow);
            // And torque still came off all nine, which is the point of not
            // refusing over any of this.
            assert_eq!(machine.torque, [false; JointId::COUNT]);
            assert!(summary.all_released());
        }
    }

    /// A driver that runs a transaction and brings nothing back leaves that one
    /// joint undescribed and moves on. Silence on the wire is exactly the
    /// condition under which the head most needs to end up limp, so it cannot
    /// be the condition that stops the walk.
    #[test]
    fn a_driver_that_brings_nothing_back_does_not_stop_the_walk() {
        let cfg = config();
        let mut seq = DisarmSequencer::new(&cfg);
        let SeqAction::Wait { until } = seq.next(Duration::ZERO, None) else {
            panic!("the settle comes first");
        };
        let first = seq.next(until, None);
        assert!(matches!(first, SeqAction::Transact(_)));

        // Every transaction unanswered, all the way through: the sequence still
        // reaches its end, having asked every servo to release.
        let mut action = seq.next(until, None);
        let mut transactions = 1;
        let summary = loop {
            match action {
                SeqAction::Transact(_) => {
                    transactions += 1;
                    assert!(transactions <= 2 * JointId::COUNT, "the walk does not loop");
                    action = seq.next(until, None);
                }
                SeqAction::Done(summary) => break summary,
                other => panic!("nothing here waits or fails: {other:?}"),
            }
        };
        assert_eq!(transactions, 2 * JointId::COUNT);
        assert!(!summary.measured_all());
        assert_eq!(summary.released, [false; JointId::COUNT]);
        assert!(!summary.at_stow);

        // Nine servos looked at and none of them readable — which the summary
        // states as nine silences under the orderly form, not as a release that
        // deliberately did not look.
        assert_eq!(summary.form, ReleaseForm::Orderly);
        assert_eq!(summary.unreadable().count(), JointId::COUNT);
        assert!(
            summary
                .unmeasured
                .iter()
                .all(|cause| matches!(cause, Some(SeqError::NoAnswer { .. }))),
            "{:?}",
            summary.unmeasured
        );
    }

    /// The fault release is nine writes: no settle, no reads, torque off in bus
    /// order. This is the maneuver a fault takes, and every transaction it does
    /// not make is time the head spends held up by motors nobody trusts.
    #[test]
    fn the_immediate_release_is_nine_writes_and_nothing_else() {
        let cfg = config();
        let mut machine = bus();
        let mut seq = DisarmSequencer::immediate(&cfg);
        let summary = crate::testutil::drive(&mut seq, &mut machine).expect("nothing here refuses");

        assert!(machine.waits.is_empty(), "a fault waits for nothing");
        let steps: Vec<SeqStep> = machine.log.iter().map(|(step, _)| *step).collect();
        assert_eq!(steps, vec![SeqStep::TorqueOff; JointId::COUNT]);
        assert_eq!(
            machine.writes(),
            SERVO_IDS
                .iter()
                .map(|id| (*id, RegValue::U8(0)))
                .collect::<Vec<_>>()
        );
        assert_eq!(machine.torque, [false; JointId::COUNT]);

        // The summary says what it looked at, which is nothing. A machine
        // physically at stow still reports `at_stow` false here: the claim
        // would be a measurement, and there was none. And nothing is recorded
        // as unreadable, because nothing was read — the form is what says that,
        // so no reader has to infer it from nine empty measurements.
        assert_eq!(summary.form, ReleaseForm::Immediate);
        assert!(!summary.measured_all());
        assert_eq!(summary.unmeasured, [None; JointId::COUNT]);
        assert_eq!(summary.unreadable().count(), 0);
        assert!(!summary.at_stow);
        assert_eq!(summary.released, [true; JointId::COUNT]);
        assert!(summary.all_released());
    }

    /// The fault release is as unstoppable as the orderly one: a servo that
    /// refuses its own write and a servo that says nothing at all are both
    /// recorded, and the other seven still come off.
    #[test]
    fn the_immediate_release_asks_every_servo_whatever_they_answer() {
        let cfg = config();
        let mut machine = bus();
        machine.fail_write = Some((SERVO_IDS[2], BusResult::ServoError { code: 0x04 }));
        machine.silent[5] = true;

        let mut seq = DisarmSequencer::immediate(&cfg);
        let summary = crate::testutil::drive(&mut seq, &mut machine).expect("nothing here refuses");

        assert_eq!(
            summary.unreleased().collect::<Vec<_>>(),
            vec![JointId::Leg(1), JointId::Leg(4)]
        );
        assert_eq!(machine.writes().len(), JointId::COUNT);
        assert_eq!(
            machine.torque,
            [false, false, true, false, false, true, false, false, false]
        );
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

    /// The standalone comparison, for a caller measuring the machine somewhere
    /// other than in a release: same tolerance, same circular treatment of the
    /// antennas, and the same answer the orderly release's own verify reaches
    /// on the same nine angles.
    #[test]
    fn a_pose_measured_elsewhere_is_judged_against_stow_the_same_way() {
        let cfg = config();
        assert!(at_stow(&cfg, &cfg.stow_targets));

        let mut folded_a_turn_further = cfg.stow_targets;
        folded_a_turn_further.antennas[0] += 2.0 * PI;
        assert!(
            at_stow(&cfg, &folded_a_turn_further),
            "an antenna is a direction: a whole turn is the same fold"
        );

        let mut leg_a_turn_out = cfg.stow_targets;
        leg_a_turn_out.legs[3] += 2.0 * PI;
        assert!(
            !at_stow(&cfg, &leg_a_turn_out),
            "a leg reading a turn from its target is a broken reading, not a joint at stow"
        );

        let mut just_inside = cfg.stow_targets;
        just_inside.body_yaw += cfg.tolerance * 0.99;
        assert!(at_stow(&cfg, &just_inside));
        let mut just_outside = cfg.stow_targets;
        just_outside.body_yaw += cfg.tolerance * 1.01;
        assert!(!at_stow(&cfg, &just_outside));

        // An angle nobody can place. Asserted rather than inherited from
        // which way round the comparison happens to be written: a machine
        // nobody can read is not a machine that may be called folded.
        for unplaceable in [f64::NAN, f64::INFINITY] {
            let mut unreadable = cfg.stow_targets;
            unreadable.legs[0] = unplaceable;
            assert!(
                !at_stow(&cfg, &unreadable),
                "an unplaceable angle is not evidence of a fold: {unplaceable}"
            );
        }

        // The release's own verdict on the same machine, so the two cannot
        // drift into disagreeing about where stow is.
        let mut machine = bus();
        machine.present = just_outside;
        let summary = drive(&cfg, &mut machine).expect("a release always finishes");
        assert_eq!(summary.at_stow, at_stow(&cfg, &just_outside));
    }
}
