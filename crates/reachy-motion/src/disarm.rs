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
use crate::cells;
use crate::joints::{self, JointRef, JointVector, ROW_COUNT, ROWS, flags, worst_joint};
use crate::resume::{ResumeError, checked_cursor, no_phase, no_stray_field};
use crate::seq::{
    BusResult, RegId, SeqAction, SeqError, SeqFailureKind, SeqStepKind, Sequencer, StepContext,
};
use crate::txn::{self, BusTxnWire};
use crate::value;
use crate::verdict;

pub use brenn_reachy__motion__disarm_clk_rs::{
    DisarmPhaseKind, DisarmSnap, DisarmSnapWire, ReleaseFormKind,
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
    pub ids: [u8; ROW_COUNT],
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
    pub deviation: [f64; ROW_COUNT],
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
    pub unmeasured: [Option<SeqError>; ROW_COUNT],
    /// Whether each servo acknowledged its torque-off write.
    pub released: [bool; ROW_COUNT],
    /// Whether every joint was measured and every one inside the tolerance.
    pub at_stow: bool,
}

impl DisarmSummary {
    /// The joint furthest from its stow angle, and how far, radians.
    #[must_use]
    pub fn worst_deviation(&self) -> (JointRef, f64) {
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
        (0..ROW_COUNT).all(|row| self.measured(row))
    }

    /// The joints a release looked at and could not read, each with why, in bus
    /// order. Empty for a release that did not look.
    pub fn unreadable(&self) -> impl Iterator<Item = (JointRef, SeqError)> + '_ {
        self.unmeasured
            .iter()
            .enumerate()
            .filter(move |_| self.looked())
            .filter_map(|(row, cause)| cause.map(|cause| (ROWS[row], cause)))
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
    pub fn unreleased(&self) -> impl Iterator<Item = JointRef> + '_ {
        self.released
            .iter()
            .enumerate()
            .filter(|(_, released)| !**released)
            .map(|(row, _)| ROWS[row])
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
    (0..ROW_COUNT).all(|row| {
        !outside_limit(
            deviation_from(
                ROWS[row],
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
fn deviation_from(joint: JointRef, present: f64, target: f64) -> f64 {
    match joint {
        JointRef::AntennaRight | JointRef::AntennaLeft => wrap_to_pi(present - target).abs(),
        _ => (present - target).abs(),
    }
}

/// How many servos a release phase's sweep walks, or `None` for the settle, the
/// ending and the phase that names nothing, which have no cursor.
fn disarm_sweep(phase: DisarmPhaseKind) -> Option<usize> {
    match phase {
        DisarmPhaseKind::Verify | DisarmPhaseKind::Release => Some(ROW_COUNT),
        DisarmPhaseKind::None | DisarmPhaseKind::Dwell | DisarmPhaseKind::Complete => None,
    }
}

/// The name a refusal about a release phase reads under.
fn disarm_phase_name(phase: DisarmPhaseKind) -> &'static str {
    match phase {
        DisarmPhaseKind::None => "no",
        DisarmPhaseKind::Dwell => "dwell",
        DisarmPhaseKind::Verify => "verify",
        DisarmPhaseKind::Release => "release",
        DisarmPhaseKind::Complete => "complete",
    }
}

/// Which release the slot's form names.
///
/// Total, and the `none` a slot nothing wrote holds answers the immediate form:
/// [`DisarmSequencer::resume`] refuses that slot, and of the two answers the one
/// claiming nothing was looked at is the one that cannot overstate what a release
/// measured.
fn release_form(form: ReleaseFormKind) -> ReleaseForm {
    match form {
        ReleaseFormKind::Orderly => ReleaseForm::Orderly,
        ReleaseFormKind::Immediate | ReleaseFormKind::None => ReleaseForm::Immediate,
    }
}

/// Whether any joint carries a filed verdict for a position read that produced
/// no angle.
///
/// The filed kinds only, and free of the sequencer, because both a running
/// release and a restore of one ask it: a row is filed or it is not, and
/// decoding the evidence to answer that would make the answer depend on whether
/// this build can read the evidence.
fn any_verdict_filed(unmeasured: &cells::SeqFailures) -> bool {
    ROWS.iter().any(|joint| {
        cells::failure_row(unmeasured, *joint)
            .is_some_and(|filed| filed.kind != SeqFailureKind::None)
    })
}

/// Disarming, as a state machine that touches no port.
///
/// Three phases in a fixed order: the settle waited out, every joint then
/// measured against the stow pose, then torque released one servo at a time with
/// each release read back. The order is what makes the measurement mean
/// something — it describes where the head was at the moment torque left it —
/// and it lives here, testable against scripted replies.
///
/// The state between two transactions is the slot the host hands over and
/// nothing else: the phase, the cursor, the measurements and the nine verdicts
/// are the schema's own fields, written through the validated view.
pub struct DisarmSequencer<'a> {
    cfg: &'a DisarmConfig,
    state: &'a mut DisarmSnap,
}

impl<'a> DisarmSequencer<'a> {
    /// A sequence ready to run against `cfg`, in `slot`: settle, measure,
    /// release.
    ///
    /// The slot is cleared to the schema's declared initial state, so a slot an
    /// earlier release left its measurements in describes this one and nothing
    /// else.
    pub fn start(cfg: &'a DisarmConfig, slot: &'a mut DisarmSnapWire) -> Self {
        let state = slot.clear_valid();
        state.form = ReleaseFormKind::Orderly;
        state.phase = DisarmPhaseKind::Dwell;
        state.dwell_waiting = true.into();
        Self { cfg, state }
    }

    /// A sequence that writes the nine releases and nothing else.
    ///
    /// The fault response. No settle and no measurement: whatever is wrong with
    /// the machine, the answer is that torque comes off now, and both of the
    /// things this skips exist only to describe a machine whose description can
    /// still be believed. The summary reports every joint unmeasured and
    /// `at_stow` false — not a verdict about where the head is, a statement
    /// that nobody looked.
    pub fn immediate(cfg: &'a DisarmConfig, slot: &'a mut DisarmSnapWire) -> Self {
        let state = slot.clear_valid();
        state.form = ReleaseFormKind::Immediate;
        state.phase = DisarmPhaseKind::Release;
        Self { cfg, state }
    }

    /// The release `state` holds, run against `cfg`.
    ///
    /// The configuration comes from the caller rather than the state: it is what
    /// the host was configured with, not what the release found out.
    ///
    /// # Errors
    ///
    /// [`ResumeError`] for a state no release reaches — a phase-and-cursor
    /// pairing, a form naming neither release, the settle's flag left behind in a
    /// later phase, or measurement and release evidence standing in a phase that
    /// cannot have produced it.
    pub fn resume(cfg: &'a DisarmConfig, state: &'a mut DisarmSnap) -> Result<Self, ResumeError> {
        let phase = state.phase;
        let name = disarm_phase_name(phase);
        no_phase(name, phase == DisarmPhaseKind::None)?;
        if state.form == ReleaseFormKind::None {
            return Err(ResumeError::NoReleaseForm);
        }
        checked_cursor(name, disarm_sweep(phase), state.cursor)?;
        // The settle's flag is written by the settle and nowhere else, so a slot
        // holding it in another phase is a slot nothing here wrote; accepting it
        // would erase the evidence along with the value.
        if phase != DisarmPhaseKind::Dwell {
            no_stray_field(
                "dwell_waiting",
                disarm_phase_name(DisarmPhaseKind::Dwell),
                name,
                state.dwell_waiting.into(),
            )?;
        }
        // What the release found and what it did are written by the phases that
        // produce them, and by nothing else. A slot holding either before its
        // phase can have run is refused on the same ground as the settle's flag,
        // and for a sharper reason: the verify sweep blanks no row it succeeds
        // on, so a verdict nobody filed survives the whole sequence and is
        // reported as a servo somebody looked at and could not measure. The
        // immediate release measures nothing at all, so those fields stay blank
        // through every phase of it.
        let verify = disarm_phase_name(DisarmPhaseKind::Verify);
        if phase == DisarmPhaseKind::Dwell {
            no_stray_field(
                "released",
                disarm_phase_name(DisarmPhaseKind::Release),
                name,
                !flags::is_empty(state.released),
            )?;
        }
        if phase == DisarmPhaseKind::Dwell || state.form == ReleaseFormKind::Immediate {
            no_stray_field("at_stow", verify, name, state.at_stow.into())?;
            no_stray_field(
                "unmeasured",
                verify,
                name,
                any_verdict_filed(&state.unmeasured),
            )?;
        }
        Ok(Self { cfg, state })
    }

    /// The fields a phase owns, blanked as it is left.
    ///
    /// The settle's flag belongs to the settle: it says whether the wait has
    /// been taken, and it means nothing once the measurements are running.
    fn blank_phase_fields(&mut self) {
        self.state.dwell_waiting = false.into();
    }

    /// Which phase a reading or a write here is reported under.
    fn phase_step(&self) -> SeqStepKind {
        match self.state.phase {
            DisarmPhaseKind::Dwell => SeqStepKind::Dwell,
            DisarmPhaseKind::Verify => SeqStepKind::VerifyAtStow,
            // A release that has not begun is reported under the writes it is
            // there to make: the phase that names nothing is refused by
            // [`Self::resume`] and never reached from either constructor.
            DisarmPhaseKind::Release | DisarmPhaseKind::Complete | DisarmPhaseKind::None => {
                SeqStepKind::TorqueOff
            }
        }
    }

    /// What the release found, and what it did, as the slot holds it.
    fn summary(&self) -> DisarmSummary {
        DisarmSummary {
            form: release_form(self.state.form),
            present: joints::vector_of(&self.state.present),
            deviation: joints::rows_of(&self.state.deviation),
            unmeasured: self.unmeasured(),
            released: flags::rows(self.state.released),
            at_stow: self.state.at_stow.into(),
        }
    }

    /// Why each joint's position read produced no angle, in bus order.
    ///
    /// A verdict the slot holds that this build cannot read is reported as
    /// exactly that: the joint was not measured either way, which is the fact
    /// the release reports, and the evidence is what a slot written by something
    /// else lost.
    fn unmeasured(&self) -> [Option<SeqError>; ROW_COUNT] {
        let mut causes = [None; ROW_COUNT];
        for (row, cause) in causes.iter_mut().enumerate() {
            let Some(filed) = joints::joint_ref(row)
                .and_then(|joint| cells::failure_row(&self.state.unmeasured, joint))
            else {
                continue;
            };
            if filed.kind == SeqFailureKind::None {
                continue;
            }
            *cause = Some(verdict::read(filed).unwrap_or(SeqError::VerdictUnreadable {
                context: StepContext::servo(filed.step, filed.servo_id),
            }));
        }
        causes
    }

    /// Whether any joint's position read produced no angle, over this
    /// sequence's own grid.
    fn any_unmeasured(&self) -> bool {
        any_verdict_filed(&self.state.unmeasured)
    }

    /// File `cause` against the servo at bus row `row` as the reason it was not
    /// measured.
    ///
    /// A cause whose own evidence will not cross is filed as unreadable rather
    /// than dropped: what matters to whoever reads the release is that this
    /// joint was not measured, and a blank row would say it was.
    fn record_unmeasured(&mut self, row: usize, cause: SeqError) {
        let context = cause.context();
        let Some(filed) = joints::joint_ref(row)
            .and_then(|joint| cells::failure_row_mut(&mut self.state.unmeasured, joint))
        else {
            return;
        };
        if verdict::write(filed, &cause).is_err() {
            // Not discarded: a refused write blanks the row, and a blank row is
            // the one claim this function exists to prevent. The unreadable
            // verdict carries neither a register value nor a wait, which are the
            // only two things a write refuses.
            verdict::write(filed, &SeqError::VerdictUnreadable { context })
                .expect("the unreadable verdict carries nothing a write can refuse");
        }
    }

    fn read(&mut self, row: usize, reg: RegId) {
        txn::set_read_reg(&mut self.state.pending, self.cfg.ids[row], reg);
    }

    fn release(&mut self, row: usize) {
        txn::set_write_reg_verified(
            &mut self.state.pending,
            self.cfg.ids[row],
            RegId::TorqueEnable,
            value::u8(0),
        );
    }

    /// The next action, the previous one having been absorbed.
    fn emit(&mut self, now: Duration) -> SeqAction<DisarmSummary> {
        let cursor = self.cursor();
        match self.state.phase {
            DisarmPhaseKind::Dwell => {
                // The settle is waited once, on entry. A configured dwell of
                // zero is no dwell at all rather than a wait until now, which
                // would hand the driver a deadline it has already passed.
                let waiting: bool = self.state.dwell_waiting.into();
                if waiting && !self.cfg.dwell.is_zero() {
                    self.state.dwell_waiting = false.into();
                    return SeqAction::Wait {
                        until: now + self.cfg.dwell,
                    };
                }
                self.enter(DisarmPhaseKind::Verify);
                self.read(0, RegId::PresentPosition);
            }
            DisarmPhaseKind::Verify => self.read(cursor, RegId::PresentPosition),
            DisarmPhaseKind::Release => self.release(cursor),
            // A release that has finished, and the phase a slot nothing wrote
            // holds — which `resume` refuses, so nothing reaches this arm with
            // work left to do.
            DisarmPhaseKind::Complete | DisarmPhaseKind::None => {
                return SeqAction::Done(self.summary());
            }
        }
        SeqAction::Transact
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
        if !txn::held(&self.state.pending) {
            // Nothing was outstanding — the first call, or the call after the
            // dwell. A result handed back here answers no request.
            return;
        }
        let read = txn::fields(&self.state.pending, self.phase_step());
        txn::set_none(&mut self.state.pending);
        let (context, wrote) = match read {
            Ok(read) => read,
            // The transaction it was waiting on cannot be read, so what came
            // back answers nothing describable. Recorded against the servo the
            // cursor is on and the sweep carries on: torque coming off the rest
            // is worth more than stopping here.
            Err(cause) => return self.abandon(cause),
        };
        let cursor = self.cursor();
        match self.state.phase {
            DisarmPhaseKind::Verify => self.absorb_verify(cursor, context, prior),
            DisarmPhaseKind::Release => {
                if prior.is_some_and(|result| confirm_write(result, wrote, context).is_ok()) {
                    flags::insert(&mut self.state.released, ROWS[cursor]);
                }
                self.advance_release(cursor);
            }
            // Terminal, or waiting: nothing is ever outstanding in these.
            DisarmPhaseKind::Dwell | DisarmPhaseKind::Complete | DisarmPhaseKind::None => {}
        }
    }

    /// Give up on the transaction the cursor is on, for `cause`, and carry on.
    ///
    /// The one thing disarming never does is stop: an outstanding transaction
    /// that cannot be read leaves this joint undescribed — unmeasured while
    /// measuring, unreleased while releasing — and the sweep moves to the next
    /// servo, which is what gets torque off the other eight.
    fn abandon(&mut self, cause: SeqError) {
        let cursor = self.cursor();
        match self.state.phase {
            DisarmPhaseKind::Verify => {
                self.record_unmeasured(cursor, cause);
                self.advance_verify(cursor);
            }
            // The servo is left out of the released set, which is what says it
            // may still be holding, and it is never written to.
            DisarmPhaseKind::Release => self.advance_release(cursor),
            DisarmPhaseKind::Dwell | DisarmPhaseKind::Complete | DisarmPhaseKind::None => {}
        }
    }

    fn absorb_verify(&mut self, cursor: usize, context: StepContext, prior: Option<&BusResult>) {
        let joint = ROWS[cursor];
        let angle = prior
            .ok_or(SeqError::NoAnswer { context })
            .and_then(|result| placeable(cursor, context, result));
        match angle {
            Ok(angle) => {
                joints::set_angle(&mut self.state.present, joint, angle);
                joints::set_angle(
                    &mut self.state.deviation,
                    joint,
                    deviation_from(joint, angle, angle_at(&self.cfg.stow_targets, cursor)),
                );
            }
            Err(cause) => self.record_unmeasured(cursor, cause),
        }
        self.advance_verify(cursor);
    }

    fn advance_verify(&mut self, cursor: usize) {
        if cursor + 1 < ROW_COUNT {
            self.seek(cursor + 1);
            return;
        }

        // Every joint is measured before the verdict, so the summary describes
        // the whole machine rather than stopping at the first joint over the
        // line. A joint nobody could read is not at stow as far as this says:
        // the claim is that the head was found where it can be left, and an
        // unread joint is no evidence of that.
        let measured_all = !self.any_unmeasured();
        let inside = !joints::rows_of(&self.state.deviation)
            .iter()
            .any(|deviation| outside_limit(*deviation, self.cfg.tolerance));
        self.state.at_stow = (measured_all && inside).into();
        self.enter(DisarmPhaseKind::Release);
    }

    fn advance_release(&mut self, cursor: usize) {
        if cursor + 1 < ROW_COUNT {
            self.seek(cursor + 1);
        } else {
            self.enter(DisarmPhaseKind::Complete);
        }
    }
}

phase_state!(DisarmSequencer, DisarmPhaseKind);

impl Sequencer for DisarmSequencer<'_> {
    fn pending(&self) -> &BusTxnWire {
        clockwork_rs::as_raw(&self.state.pending)
    }

    type Summary = DisarmSummary;

    fn next(&mut self, now: Duration, prior: Option<&BusResult>) -> SeqAction<DisarmSummary> {
        self.absorb(prior);
        self.emit(now)
    }

    fn step(&self) -> SeqStepKind {
        self.phase_step()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::arm::SERVO_IDS;
    use crate::joints::Name;
    use crate::testutil::{Asked, ScriptedBus, asked};
    use crate::txn::AuxOpKind;
    use crate::value::Value;
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
        torque: [bool; ROW_COUNT],
        silent: [bool; ROW_COUNT],
        /// One write to answer with something other than success.
        fail_write: Option<(u8, BusResult)>,
        /// One read to answer with something other than success.
        fail_read: Option<(u8, BusResult)>,
        log: Vec<(SeqStepKind, Asked)>,
        waits: Vec<(Duration, Duration)>,
        /// How many transactions had run when each wait began, which is what
        /// places the settle in the order rather than merely counting it.
        waited_after: Vec<usize>,
    }

    /// Two copies of one machine: the direct run and the crossed run.
    ///
    /// Built once and cloned rather than written twice, because the comparison
    /// between the two runs means nothing unless both faced the same bus.
    fn pair(machine: Machine) -> (Machine, Machine) {
        (machine.clone(), machine)
    }

    /// A platform holding itself at stow, torque on.
    fn bus() -> Machine {
        Machine {
            present: config().stow_targets,
            torque: [true; ROW_COUNT],
            silent: [false; ROW_COUNT],
            fail_write: None,
            fail_read: None,
            log: Vec::new(),
            waits: Vec::new(),
            waited_after: Vec::new(),
        }
    }

    impl ScriptedBus for Machine {
        fn answer(&mut self, step: SeqStepKind, request: &BusTxnWire) -> BusResult {
            let entry = asked(request, step);
            let Asked {
                op,
                context,
                value: written,
            } = entry;
            self.log.push((step, entry));
            let row = SERVO_IDS
                .iter()
                .position(|id| *id == context.id)
                .expect("addressed to a servo on this bus");
            if self.silent[row] {
                return BusResult::NoAnswer;
            }
            let scripted = match op {
                AuxOpKind::WriteRegVerified => self.fail_write,
                _ => self.fail_read,
            };
            if let Some((id, result)) = scripted
                && context.id == id
            {
                return result;
            }
            match op {
                AuxOpKind::ReadReg => {
                    BusResult::Value(value::radians(angle_at(&self.present, row)))
                }
                AuxOpKind::Ping | AuxOpKind::None => {
                    panic!("disarming pings nothing")
                }
                // A release is evidence or it is nothing: every write here is
                // read back, so an unverified one is a transaction this
                // sequencer never emits.
                AuxOpKind::WriteReg => {
                    panic!("disarming reads every write back")
                }
                AuxOpKind::WriteRegVerified => {
                    let value = written;
                    if value == value::u8(0) {
                        self.torque[row] = false;
                    }
                    BusResult::Written
                }
            }
        }

        fn waited(&mut self, now: Duration, until: Duration) {
            self.waits.push((now, until));
            self.waited_after.push(self.log.len());
        }
    }

    impl Machine {
        fn writes(&self) -> Vec<(u8, Value)> {
            self.log
                .iter()
                .filter(|(_, request)| request.op == AuxOpKind::WriteRegVerified)
                .map(|(_, request)| (request.id(), request.value))
                .collect()
        }
    }

    /// The shared driver, against this crate's disarming sequencer: the orderly
    /// release, held across its own steps.
    fn drive(cfg: &DisarmConfig, machine: &mut Machine) -> Result<DisarmSummary, SeqError> {
        let mut slot = DisarmSnapWire::new();
        let mut seq = DisarmSequencer::start(cfg, &mut slot);
        crate::testutil::drive(&mut seq, machine)
    }

    /// The same, for the fault release.
    fn drive_immediate(
        cfg: &DisarmConfig,
        machine: &mut Machine,
    ) -> Result<DisarmSummary, SeqError> {
        let mut slot = DisarmSnapWire::new();
        let mut seq = DisarmSequencer::immediate(cfg, &mut slot);
        crate::testutil::drive(&mut seq, machine)
    }

    resumed! {
        /// The release, resumed from its slot before every step: `cfg` is all
        /// its `resume` needs.
        struct Resuming { cfg: DisarmConfig }
        slot = DisarmSnapWire, summary = DisarmSummary, seq = DisarmSequencer,
        resume(host, state) = DisarmSequencer::resume(host.cfg, state);
    }

    /// Step the release `slot` holds to its end, resuming a sequencer from the
    /// slot before every step.
    fn release_from_slot(
        cfg: &DisarmConfig,
        slot: &mut DisarmSnapWire,
        machine: &mut Machine,
    ) -> Result<DisarmSummary, SeqError> {
        crate::testutil::drive_from_slot(&Resuming { cfg }, slot, machine, Duration::ZERO)
    }

    /// The orderly release, crossed at every step.
    fn drive_resumed(cfg: &DisarmConfig, machine: &mut Machine) -> Result<DisarmSummary, SeqError> {
        let mut slot = DisarmSnapWire::new();
        DisarmSequencer::start(cfg, &mut slot);
        release_from_slot(cfg, &mut slot, machine)
    }

    /// The fault release, crossed at every step.
    fn drive_immediate_resumed(
        cfg: &DisarmConfig,
        machine: &mut Machine,
    ) -> Result<DisarmSummary, SeqError> {
        let mut slot = DisarmSnapWire::new();
        DisarmSequencer::immediate(cfg, &mut slot);
        release_from_slot(cfg, &mut slot, machine)
    }

    /// The order is the whole safety property: the settle is waited out, every
    /// joint is then measured, and only then does torque come off — servo by
    /// servo, in bus order, each release read back.
    #[test]
    fn the_dwell_precedes_the_stow_check_and_torque_comes_off_last() {
        let cfg = config();
        let mut machine = bus();
        let summary = drive(&cfg, &mut machine).expect("a machine at stow disarms");

        let steps: Vec<SeqStepKind> = machine.log.iter().map(|(step, _)| *step).collect();
        assert_eq!(
            steps,
            [
                vec![SeqStepKind::VerifyAtStow; ROW_COUNT],
                vec![SeqStepKind::TorqueOff; ROW_COUNT],
            ]
            .concat()
        );
        assert_eq!(
            machine.writes(),
            SERVO_IDS
                .iter()
                .map(|id| (*id, value::u8(0)))
                .collect::<Vec<_>>()
        );
        assert_eq!(machine.torque, [false; ROW_COUNT]);
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
        assert_eq!(summary.deviation, [0.0; ROW_COUNT]);
        assert!(summary.measured_all());
        assert_eq!(summary.unreadable().count(), 0);
        assert_eq!(summary.released, [true; ROW_COUNT]);
        assert!(summary.all_released());
        assert_eq!(summary.unreleased().count(), 0);
        assert_eq!(summary.worst_deviation(), (JointRef::BodyYaw, 0.0));
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
        assert_eq!(summary.worst_deviation().0, JointRef::Leg2);
        assert!((summary.deviation[3].to_degrees() - 5.0).abs() < 1e-9);

        assert_eq!(
            machine.writes(),
            SERVO_IDS
                .iter()
                .map(|id| (*id, value::u8(0)))
                .collect::<Vec<_>>()
        );
        assert_eq!(machine.torque, [false; ROW_COUNT]);
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
        assert_eq!(summary.worst_deviation().0, JointRef::AntennaLeft);
        assert_eq!(machine.torque, [false; ROW_COUNT]);
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
        assert_eq!(machine.torque, [false; ROW_COUNT]);

        let (joint, deviation) = summary.worst_deviation();
        let row = ROWS
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
            (0usize, 7usize, 162.248_f64, JointRef::AntennaRight),
            (1, 8, -162.248, JointRef::AntennaLeft),
        ];
        for (side, row, degrees, joint) in cases {
            let mut machine = bus();
            machine.present.antennas[side] = degrees.to_radians();

            let summary = drive(&wide, &mut machine).expect("23° is inside a 25° gate");
            assert!(summary.at_stow, "{}", Name(joint));
            assert!(
                (summary.deviation[row].to_degrees() - 23.0).abs() < 1e-3,
                "the distance around the circle is {}°",
                summary.deviation[row].to_degrees()
            );
            assert_eq!(machine.torque, [false; ROW_COUNT]);

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
            assert_eq!(machine.torque, [false; ROW_COUNT]);
        }

        // A leg keeps the linear difference: it is a windowed joint working far
        // from the half turn, and a reading a whole turn from its target is a
        // broken reading rather than a leg at stow.
        let mut machine = bus();
        machine.present.legs[0] += core::f64::consts::TAU;
        let summary = drive(&wide, &mut machine).expect("nothing here refuses");
        assert!(!summary.at_stow, "a turn is not zero on a leg");
        assert_eq!(summary.worst_deviation().0, JointRef::Leg0);
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
                        joint: JointRef::AntennaRight,
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
                vec![JointRef::AntennaRight]
            );
            assert_eq!(machine.torque, [false; ROW_COUNT]);
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
        assert_eq!(machine.torque, [false; ROW_COUNT]);
        assert_eq!(machine.log.len(), 2 * ROW_COUNT);
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
            vec![JointRef::Leg3]
        );

        // Nine writes went out, one per servo, in bus order.
        assert_eq!(
            machine.writes(),
            SERVO_IDS
                .iter()
                .map(|id| (*id, value::u8(0)))
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
        assert_eq!(machine.writes().len(), ROW_COUNT);
        // The silent servo never acknowledges its release either; the other
        // eight are limp.
        assert!(!summary.released[6]);
        assert_eq!(
            summary.unreleased().collect::<Vec<_>>(),
            vec![JointRef::Leg5]
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
            assert_eq!(cause.context().step, SeqStepKind::VerifyAtStow);
            assert!(!summary.at_stow);
            // And torque still came off all nine, which is the point of not
            // refusing over any of this.
            assert_eq!(machine.torque, [false; ROW_COUNT]);
            assert!(summary.all_released());
        }
    }

    /// A release resumed with an outstanding record this build cannot read gives
    /// up on that one servo and keeps sweeping. The record is damaged where a
    /// slot's bytes are: the release phase's transaction is left naming no
    /// register, which is what something that does not agree with this build
    /// about a transaction writes.
    #[test]
    fn a_pending_record_that_cannot_be_read_abandons_one_servo_and_the_sweep_walks_on() {
        let cfg = config();
        let mut machine = bus();
        let mut slot = DisarmSnapWire::new();
        DisarmSequencer::start(&cfg, &mut slot);
        let mut now = Duration::ZERO;
        let mut prior = None;
        let mut damaged_at = None;
        let summary = loop {
            let state = slot
                .validate_mut()
                .expect("a release writes its own schema");
            let mut seq =
                DisarmSequencer::resume(&cfg, state).expect("a release mid-sweep resumes");
            match seq.next(now, prior.as_ref()) {
                SeqAction::Transact => {
                    let phase = seq.step();
                    let addressed = asked(seq.pending(), phase).id();
                    if phase == SeqStepKind::TorqueOff && addressed == SERVO_IDS[3] {
                        // The record it is waiting on, left naming no register
                        // where the operation needs one.
                        let state = slot.validate_mut().expect("the state was written here");
                        state.pending.reg = RegId::None;
                        damaged_at = Some(3);
                        prior = None;
                        continue;
                    }
                    prior = Some(machine.answer(phase, seq.pending()));
                }
                SeqAction::Wait { until } => {
                    machine.waited(now, until);
                    now = until;
                    prior = None;
                }
                SeqAction::Done(summary) => break summary,
                SeqAction::Fail(error) => panic!("a release does not stop: {error}"),
            }
        };
        assert_eq!(damaged_at, Some(3), "the release phase was reached");

        // The abandoned servo is reported as unreleased and never written to;
        // the other eight are limp, which is the whole point of walking on.
        assert!(!summary.released[3]);
        for row in 0..ROW_COUNT {
            if row == 3 {
                continue;
            }
            assert!(summary.released[row], "servo {row} released");
            assert!(!machine.torque[row], "servo {row} is limp");
        }
        assert!(
            machine.torque[3],
            "the abandoned servo was never written to"
        );
        // Measurement happened before the damage, so the sweep's own look is
        // unaffected.
        assert!(summary.unmeasured.iter().all(Option::is_none));
    }

    /// The same in the measuring sweep: the joint whose record cannot be read is
    /// filed as unmeasured, carrying the refusal that named it, and all nine
    /// still let go.
    #[test]
    fn a_pending_record_that_cannot_be_read_leaves_one_joint_unmeasured() {
        let cfg = config();
        let mut machine = bus();
        let mut slot = DisarmSnapWire::new();
        DisarmSequencer::start(&cfg, &mut slot);
        let mut now = Duration::ZERO;
        let mut prior = None;
        let mut damaged = false;
        let summary = loop {
            let state = slot
                .validate_mut()
                .expect("a release writes its own schema");
            let mut seq =
                DisarmSequencer::resume(&cfg, state).expect("a release mid-sweep resumes");
            match seq.next(now, prior.as_ref()) {
                SeqAction::Transact => {
                    let phase = seq.step();
                    let addressed = asked(seq.pending(), phase).id();
                    if !damaged && phase == SeqStepKind::VerifyAtStow && addressed == SERVO_IDS[6] {
                        let state = slot.validate_mut().expect("the state was written here");
                        state.pending.reg = RegId::None;
                        damaged = true;
                        prior = None;
                        continue;
                    }
                    prior = Some(machine.answer(phase, seq.pending()));
                }
                SeqAction::Wait { until } => {
                    machine.waited(now, until);
                    now = until;
                    prior = None;
                }
                SeqAction::Done(summary) => break summary,
                SeqAction::Fail(error) => panic!("a release does not stop: {error}"),
            }
        };
        assert!(damaged, "the measuring sweep reached the sixth crank");

        assert!(
            matches!(
                summary.unmeasured[6],
                Some(SeqError::PendingUnreadable { .. })
            ),
            "{:?}",
            summary.unmeasured[6]
        );
        assert_eq!(summary.unreadable().count(), 1);
        assert!(!summary.at_stow, "an unread joint is no evidence of a fold");
        assert!(summary.all_released(), "all nine still let go");
        assert_eq!(machine.torque, [false; ROW_COUNT]);
    }
    /// A driver that runs a transaction and brings nothing back leaves that one
    /// joint undescribed and moves on. Silence on the wire is exactly the
    /// condition under which the head most needs to end up limp, so it cannot
    /// be the condition that stops the walk.
    #[test]
    fn a_driver_that_brings_nothing_back_does_not_stop_the_walk() {
        let cfg = config();
        let mut slot = DisarmSnapWire::new();
        let mut seq = DisarmSequencer::start(&cfg, &mut slot);
        let SeqAction::Wait { until } = seq.next(Duration::ZERO, None) else {
            panic!("the settle comes first");
        };
        let first = seq.next(until, None);
        assert!(matches!(first, SeqAction::Transact));

        // Every transaction unanswered, all the way through: the sequence still
        // reaches its end, having asked every servo to release.
        let mut action = seq.next(until, None);
        let mut transactions = 1;
        let summary = loop {
            match action {
                SeqAction::Transact => {
                    transactions += 1;
                    assert!(transactions <= 2 * ROW_COUNT, "the walk does not loop");
                    action = seq.next(until, None);
                }
                SeqAction::Done(summary) => break summary,
                other => panic!("nothing here waits or fails: {other:?}"),
            }
        };
        assert_eq!(transactions, 2 * ROW_COUNT);
        assert!(!summary.measured_all());
        assert_eq!(summary.released, [false; ROW_COUNT]);
        assert!(!summary.at_stow);

        // Nine servos looked at and none of them readable — which the summary
        // states as nine silences under the orderly form, not as a release that
        // deliberately did not look.
        assert_eq!(summary.form, ReleaseForm::Orderly);
        assert_eq!(summary.unreadable().count(), ROW_COUNT);
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
        let summary = drive_immediate(&cfg, &mut machine).expect("nothing here refuses");

        assert!(machine.waits.is_empty(), "a fault waits for nothing");
        let steps: Vec<SeqStepKind> = machine.log.iter().map(|(step, _)| *step).collect();
        assert_eq!(steps, vec![SeqStepKind::TorqueOff; ROW_COUNT]);
        assert_eq!(
            machine.writes(),
            SERVO_IDS
                .iter()
                .map(|id| (*id, value::u8(0)))
                .collect::<Vec<_>>()
        );
        assert_eq!(machine.torque, [false; ROW_COUNT]);

        // The summary says what it looked at, which is nothing. A machine
        // physically at stow still reports `at_stow` false here: the claim
        // would be a measurement, and there was none. And nothing is recorded
        // as unreadable, because nothing was read — the form is what says that,
        // so no reader has to infer it from nine empty measurements.
        assert_eq!(summary.form, ReleaseForm::Immediate);
        assert!(!summary.measured_all());
        assert_eq!(summary.unmeasured, [None; ROW_COUNT]);
        assert_eq!(summary.unreadable().count(), 0);
        assert!(!summary.at_stow);
        assert_eq!(summary.released, [true; ROW_COUNT]);
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

        let summary = drive_immediate(&cfg, &mut machine).expect("nothing here refuses");

        assert_eq!(
            summary.unreleased().collect::<Vec<_>>(),
            vec![JointRef::Leg1, JointRef::Leg4]
        );
        assert_eq!(machine.writes().len(), ROW_COUNT);
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

    // ---- The release resumed from its slot ----

    /// The orderly release, resumed at every step: the settle, the nine
    /// measurements and the nine releases all survive a slot crossing, and the
    /// summary is the one a host holding the sequencer in a local variable gets.
    #[test]
    fn a_crossed_orderly_release_reaches_the_same_summary() {
        let cfg = config();

        let (mut direct, mut machine) = pair(bus());
        let held = drive(&cfg, &mut direct).expect("a machine at stow disarms");
        let crossed = drive_resumed(&cfg, &mut machine).expect("a machine at stow disarms");

        assert_eq!(crossed, held);
        assert_eq!(machine.log, direct.log);
        assert_eq!(machine.waits, direct.waits);
        assert_eq!(machine.waited_after, direct.waited_after);
        assert_eq!(machine.torque, [false; ROW_COUNT]);
    }

    /// The immediate release, resumed: nine writes and nothing else. The form is
    /// what says the measurement fields describe no look at all, and it is a
    /// field the slot holds.
    #[test]
    fn a_crossed_immediate_release_still_looks_at_nothing() {
        let cfg = config();

        let (mut direct, mut machine) = pair(bus());
        let held = drive_immediate(&cfg, &mut direct).expect("the immediate release cannot fail");
        let crossed =
            drive_immediate_resumed(&cfg, &mut machine).expect("the immediate release cannot fail");

        assert_eq!(crossed, held);
        assert_eq!(crossed.form, ReleaseForm::Immediate);
        assert!(!crossed.looked());
        assert_eq!(machine.log, direct.log);
        assert!(machine.waits.is_empty());
        assert_eq!(machine.torque, [false; ROW_COUNT]);
    }

    /// A machine away from stow, one servo silent, and one release
    /// unacknowledged: three things a release records rather than stops over,
    /// each carried across every crossing between the step that recorded it and
    /// the summary that reports it.
    #[test]
    fn a_crossed_release_carries_what_it_recorded_on_the_way() {
        let cfg = config();

        let mut standing = bus();
        standing.present = joints_at(&neutral_head_pose());
        let (mut direct, mut machine) = pair(standing);
        let held = drive(&cfg, &mut direct).expect("torque comes off wherever the head is");
        let crossed =
            drive_resumed(&cfg, &mut machine).expect("torque comes off wherever the head is");
        assert_eq!(crossed, held);
        assert!(!crossed.at_stow);
        assert_eq!(machine.torque, [false; ROW_COUNT]);

        let mut silent = [false; ROW_COUNT];
        silent[4] = true;
        let (mut direct, mut machine) = pair(Machine { silent, ..bus() });
        let held = drive(&cfg, &mut direct).expect("the other eight are released");
        let crossed = drive_resumed(&cfg, &mut machine).expect("the other eight are released");
        assert_eq!(crossed, held);
        assert!(crossed.unmeasured[4].is_some());
        assert!(!crossed.released[4]);
        assert!(!crossed.at_stow);

        let (mut direct, mut machine) = pair(Machine {
            fail_write: Some((14, BusResult::NoAnswer)),
            ..bus()
        });
        let held = drive(&cfg, &mut direct).expect("torque comes off regardless");
        let crossed = drive_resumed(&cfg, &mut machine).expect("torque comes off regardless");
        assert_eq!(crossed, held);
        assert_eq!(crossed.unreleased().collect::<Vec<_>>(), [JointRef::Leg3]);
    }

    /// A configured dwell of zero is no wait at all, and the flag that says the
    /// settle has been taken is state the slot holds: a release that lost it
    /// would wait the dwell again on every execution and never let go.
    #[test]
    fn a_crossed_settle_is_taken_once() {
        let cfg = config();
        let mut slot = DisarmSnapWire::new();
        DisarmSequencer::start(&cfg, &mut slot);
        assert!(
            bool::from(
                slot.validate()
                    .expect("a fresh release writes its own schema")
                    .dwell_waiting
            ),
            "a fresh release opens waiting"
        );

        let mut machine = bus();
        let summary =
            release_from_slot(&cfg, &mut slot, &mut machine).expect("a machine at stow disarms");
        assert!(summary.at_stow);
        assert_eq!(machine.waits.len(), 1, "the settle is waited exactly once");

        let prompt = DisarmConfig {
            dwell: Duration::ZERO,
            ..cfg
        };
        let mut machine = bus();
        let summary = drive_resumed(&prompt, &mut machine).expect("a machine at stow disarms");
        assert!(summary.at_stow);
        assert!(machine.waits.is_empty(), "a dwell of zero is no dwell");
    }

    /// A state no release reaches is refused rather than run.
    #[test]
    fn a_state_no_release_reaches_is_refused() {
        let cfg = config();
        let mut slot = DisarmSnapWire::new();

        // A slot nothing wrote names no phase, and reading it as the settle
        // would wait a dwell nobody asked for — which delays a torque-off.
        assert_eq!(
            DisarmSequencer::resume(&cfg, slot.clear_valid()).err(),
            Some(ResumeError::NoPhase { phase: "no" })
        );

        // A form naming neither release, with a phase that does. The form is
        // what says whether the measurements describe a failed look or no look
        // at all, so it is not guessed.
        let state = slot.clear_valid();
        state.phase = DisarmPhaseKind::Dwell;
        assert_eq!(
            DisarmSequencer::resume(&cfg, state).err(),
            Some(ResumeError::NoReleaseForm)
        );

        // A cursor past the sweep it indexes.
        let state = slot.clear_valid();
        state.form = ReleaseFormKind::Orderly;
        state.phase = DisarmPhaseKind::Release;
        state.cursor = 9;
        assert_eq!(
            DisarmSequencer::resume(&cfg, state).err(),
            Some(ResumeError::CursorOutOfRange {
                phase: "release",
                cursor: 9,
                bound: 9,
            })
        );

        // The settle carries no cursor, so only zero is one it could have been
        // left at.
        let state = slot.clear_valid();
        state.form = ReleaseFormKind::Orderly;
        state.phase = DisarmPhaseKind::Dwell;
        state.cursor = 1;
        assert_eq!(
            DisarmSequencer::resume(&cfg, state).err(),
            Some(ResumeError::CursorInPhaseWithNoSweep {
                phase: "dwell",
                cursor: 1,
            })
        );

        // The settle's flag outside the settle. A release that has moved on has
        // taken its wait, so a slot claiming otherwise was written by something
        // else — and accepting it would wait the dwell again.
        for phase in [
            DisarmPhaseKind::Verify,
            DisarmPhaseKind::Release,
            DisarmPhaseKind::Complete,
        ] {
            let state = slot.clear_valid();
            state.form = ReleaseFormKind::Orderly;
            state.phase = phase;
            state.dwell_waiting = true.into();
            assert_eq!(
                DisarmSequencer::resume(&cfg, state).err(),
                Some(ResumeError::StrayPhaseField {
                    field: "dwell_waiting",
                    owner: "dwell",
                    phase: disarm_phase_name(phase),
                }),
                "{phase:?}"
            );

            let state = slot.clear_valid();
            state.form = ReleaseFormKind::Orderly;
            state.phase = phase;
            assert!(
                DisarmSequencer::resume(&cfg, state).is_ok(),
                "{phase:?} is reachable with the flag down"
            );
        }
    }

    /// Evidence a phase cannot have produced is refused, not carried into the
    /// summary.
    ///
    /// The verify sweep blanks no row it succeeds on, so a verdict pre-filed in
    /// a slot survives the whole sequence and reads back as a servo somebody
    /// looked at and could not measure — a refusal nobody observed, in the record
    /// that says whether a park ending got an accurate story. The immediate form
    /// looks at nothing at all, so those fields stay blank through every phase of
    /// it.
    #[test]
    fn evidence_a_phase_cannot_have_produced_is_refused() {
        let cfg = config();
        let mut slot = DisarmSnapWire::new();
        let verify = disarm_phase_name(DisarmPhaseKind::Verify);
        let release = disarm_phase_name(DisarmPhaseKind::Release);

        // The settle has looked at nothing and let go of nothing.
        let state = slot.clear_valid();
        state.form = ReleaseFormKind::Orderly;
        state.phase = DisarmPhaseKind::Dwell;
        state.at_stow = true.into();
        assert_eq!(
            DisarmSequencer::resume(&cfg, state).err(),
            Some(ResumeError::StrayPhaseField {
                field: "at_stow",
                owner: verify,
                phase: "dwell",
            })
        );

        let state = slot.clear_valid();
        state.form = ReleaseFormKind::Orderly;
        state.phase = DisarmPhaseKind::Dwell;
        file_verdict(state, JointRef::Leg1);
        assert_eq!(
            DisarmSequencer::resume(&cfg, state).err(),
            Some(ResumeError::StrayPhaseField {
                field: "unmeasured",
                owner: verify,
                phase: "dwell",
            })
        );

        let state = slot.clear_valid();
        state.form = ReleaseFormKind::Orderly;
        state.phase = DisarmPhaseKind::Dwell;
        flags::insert(&mut state.released, JointRef::Leg3);
        assert_eq!(
            DisarmSequencer::resume(&cfg, state).err(),
            Some(ResumeError::StrayPhaseField {
                field: "released",
                owner: release,
                phase: "dwell",
            })
        );

        // The fault release measures nothing, so a measurement in one was
        // written by something else — in the phase it does run, not only ahead
        // of it.
        for phase in [DisarmPhaseKind::Release, DisarmPhaseKind::Complete] {
            let state = slot.clear_valid();
            state.form = ReleaseFormKind::Immediate;
            state.phase = phase;
            state.at_stow = true.into();
            assert_eq!(
                DisarmSequencer::resume(&cfg, state).err(),
                Some(ResumeError::StrayPhaseField {
                    field: "at_stow",
                    owner: verify,
                    phase: disarm_phase_name(phase),
                }),
                "{phase:?}"
            );

            let state = slot.clear_valid();
            state.form = ReleaseFormKind::Immediate;
            state.phase = phase;
            file_verdict(state, JointRef::AntennaLeft);
            assert_eq!(
                DisarmSequencer::resume(&cfg, state).err(),
                Some(ResumeError::StrayPhaseField {
                    field: "unmeasured",
                    owner: verify,
                    phase: disarm_phase_name(phase),
                }),
                "{phase:?}"
            );

            // What the release itself writes is not stray in the release.
            let state = slot.clear_valid();
            state.form = ReleaseFormKind::Immediate;
            state.phase = phase;
            flags::insert(&mut state.released, JointRef::Leg3);
            assert!(
                DisarmSequencer::resume(&cfg, state).is_ok(),
                "{phase:?} is where torque comes off"
            );
        }

        // And the orderly release carries its own measurements once the sweep
        // that makes them has run.
        let state = slot.clear_valid();
        state.form = ReleaseFormKind::Orderly;
        state.phase = DisarmPhaseKind::Release;
        state.at_stow = true.into();
        file_verdict(state, JointRef::Leg1);
        flags::insert(&mut state.released, JointRef::Leg0);
        assert!(DisarmSequencer::resume(&cfg, state).is_ok());
    }

    /// File a verdict against `joint`, as a measurement that produced no angle.
    fn file_verdict(state: &mut DisarmSnap, joint: JointRef) {
        let filed = cells::failure_row_mut(&mut state.unmeasured, joint)
            .expect("a joint names a row of the grid");
        filed.kind = SeqFailureKind::NoAnswer;
    }

    /// A release started in a slot an earlier release used describes this one
    /// and nothing else.
    ///
    /// The clear on the way in is the whole of that guarantee: nothing else
    /// re-blanks the slot, so without it the summary would report a head
    /// measured at stow nobody looked at, and a servo that never answered as one
    /// that let go.
    #[test]
    fn a_release_started_over_an_earlier_one_describes_this_one_only() {
        let cfg = config();

        // A first release that leaves something in every field the clear has to
        // reach: a leg away from stow, and a servo that answered neither its
        // measurement nor its release.
        let mut messy = bus();
        messy.present.legs[2] += 0.4;
        messy.silent[1] = true;
        let mut slot = DisarmSnapWire::new();
        let mut seq = DisarmSequencer::start(&cfg, &mut slot);
        let dirty =
            crate::testutil::drive(&mut seq, &mut messy).expect("a release always finishes");
        assert!(!dirty.at_stow, "the machine was away from stow");
        assert!(
            dirty.unmeasured[1].is_some(),
            "a silent servo is not measured"
        );
        assert!(!dirty.released[1], "and acknowledges no release");
        assert_ne!(dirty.deviation, [0.0; ROW_COUNT]);

        // The same slot, run again against a machine at stow.
        let mut clean = bus();
        let mut seq = DisarmSequencer::start(&cfg, &mut slot);
        let again =
            crate::testutil::drive(&mut seq, &mut clean).expect("a release always finishes");

        let mut fresh_slot = DisarmSnapWire::new();
        let mut fresh_machine = bus();
        let mut fresh_seq = DisarmSequencer::start(&cfg, &mut fresh_slot);
        let fresh = crate::testutil::drive(&mut fresh_seq, &mut fresh_machine)
            .expect("a release always finishes");

        assert_eq!(again, fresh, "the reused slot reported the release it ran");
        assert_eq!(slot, fresh_slot, "and holds nothing of the one before it");
    }
}
