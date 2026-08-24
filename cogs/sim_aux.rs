//! What the modelled bus answers to one out-of-band transaction.
//!
//! A driver cycle spends whatever bus time is left over from the goal write and
//! the proprioception read on exactly one of these, and [`reachy_driver::AuxSlot`]
//! is what picks which. What is here is the other half: running the one it
//! picked against the modelled control tables, and saying what came back in the
//! same outcome record the real driver publishes.
//!
//! Everything a transaction touches is a register. A read reads a cell, a
//! verified write writes one and reads it back, and a ping answers with the
//! model number the addressed row holds -- so a commissioning sequence verifies
//! against the same numbers here that it verifies against on hardware, and the
//! simulated driver's observable contract is the real one's.
//!
//! Three registers are the plant's, and a write to one of them is a write to
//! the modelled world rather than to a cell: torque enable energises the row,
//! goal position gives it something to hold, and present position is a servo's
//! own reading that nothing writes over the wire. Every other register is
//! storage, and the bus table decides its shape and whether a row holding
//! torque may be written at all.
//!
//! A row a scenario has taken off the bus comes before all of that: nothing
//! answers for it and nothing reaches it, which is what a dead or unplugged
//! servo is to a driver, and the transaction ends in the silence its window
//! closed on.
//!
//! Nothing here is a policy the real driver would have to agree with: which
//! transaction runs, what a confirmation pass makes of a read-back and what the
//! driver believes about torque are all `reachy-driver`'s, hosted by the cog
//! next door. This module is the servos.

use brenn_reachy__cogs__sim_state_clk_rs::SimState;
use brenn_reachy__driver__health_clk_rs::{AuxOutcome, AuxStatus, HealthReport};
use brenn_reachy__hardware__dynamixel__registers_clk_rs::{RegId, ValueShape};
use brenn_reachy__motion__bus_txn_clk_rs::{AuxOpKind, BusTxn};
use brenn_reachy__motion__joints_clk_rs::{JointFlags, JointRef};
use clockwork_rs::SyncTime;
use reachy_bus::value_kind;
use reachy_driver::{AuxSlot, GoalGate, TorqueOffConfirm};
use reachy_motion::arm::{SERVO_IDS, row_of_id};
use reachy_motion::joints::{flags, joint_ref, row, set_angle};
use reachy_motion::value::{self, Value};

use crate::sim_regs::{self, Regs};

/// The temperature every simulated health report carries, degrees Celsius.
///
/// A constant of the model and not a reading: this plant has no thermal state to
/// read one from. A plausible resting figure, so a scenario's report reads like
/// a machine at room temperature rather than like a servo at freezing point, and
/// a checker comparing it against a band gets an answer that means something.
pub const SIM_TEMP_C: i8 = 25;

/// One transaction, copied out of the record the slot holds.
///
/// Copied rather than borrowed because running it writes the state the record
/// lives in: the fields are five numbers, and holding the record open across
/// the write would mean borrowing the whole slot for the length of the bus
/// transaction it describes.
#[derive(Clone, Copy)]
pub struct Request {
    /// Which transaction.
    pub op: AuxOpKind,
    /// Which servo, as its bus id.
    pub id: u8,
    /// Which register, or the no-register zero.
    pub reg: RegId,
    /// What shape the value is, as the host built it.
    pub value_kind: ValueShape,
    /// The value, as eight little-endian bytes.
    pub value: u64,
}

impl Request {
    /// The transaction a slot handed over.
    #[must_use]
    pub fn of(txn: &BusTxn) -> Self {
        Self {
            op: txn.op,
            id: txn.id,
            reg: txn.reg,
            value_kind: txn.value_kind,
            value: txn.value,
        }
    }
}

/// What the modelled bus answered, before it reaches the outcome record.
///
/// Held as ordinary Rust for the reason the cycle's event is: a cycle decides
/// what to publish and then publishes it once, rather than writing a slot as it
/// goes.
#[derive(Clone, Copy)]
pub struct Answer {
    /// The correlation number of the request this answers.
    pub corr: u32,
    /// How it went.
    pub status: AuxStatus,
    /// What shape the value is, and `none` where there is no register value to
    /// carry.
    pub value_kind: ValueShape,
    /// The value, as eight little-endian bytes.
    pub value: u64,
    /// The model number a ping answered with, and zero for everything else.
    pub model: u16,
}

impl Answer {
    /// An answer carrying no register value: a ping's, a refusal's, a silence's.
    fn bare(corr: u32, status: AuxStatus) -> Self {
        Self {
            corr,
            status,
            value_kind: ValueShape::None,
            value: 0,
            model: 0,
        }
    }

    /// The driver declining to run a transaction, so nothing reached the bus.
    ///
    /// What a malformed request gets: a transaction naming no register, a value
    /// built in a shape the register does not take, or a write to a cell the
    /// plant owns and the bus therefore cannot set. Nothing about the machine
    /// changed, which is the whole difference between this and a servo that
    /// answered badly.
    fn refused(corr: u32) -> Self {
        Self::bare(corr, AuxStatus::Refused)
    }

    /// The driver turning a request away because one is already pending.
    ///
    /// Against the turned-away request's own correlation number, so the host
    /// learns which of its two requests was not run rather than having to work
    /// it out from a silence.
    #[must_use]
    pub fn busy(corr: u32) -> Self {
        Self::bare(corr, AuxStatus::Refused)
    }

    /// A value read off a cell.
    fn value(corr: u32, held: Value) -> Self {
        Self {
            corr,
            status: AuxStatus::Ok,
            value_kind: held.shape(),
            value: held.bits(),
            model: 0,
        }
    }

    /// Write this answer into the outcome record that carries it.
    pub fn write(&self, out: &mut AuxOutcome) {
        out.corr = self.corr;
        out.status = self.status;
        out.value_kind = self.value_kind;
        out.value = self.value;
        out.model = self.model;
    }
}

/// Run one transaction against the modelled machine.
///
/// `nominal` is the cycle's own instant, which a torque-enable write needs: an
/// arming grants the dead-man a fresh window, and the window runs from the
/// write.
pub fn answer(state: &mut SimState, nominal: i64, corr: u32, request: &Request) -> Answer {
    let Some(row) = row_of_id(request.id) else {
        // Nothing on this bus has that id, so nothing answers. A timeout rather
        // than a refusal: the datagram went out and the window closed on
        // silence, which is exactly what a real bus does with an id nobody
        // holds.
        return Answer::bare(corr, AuxStatus::Timeout);
    };
    if is_absent(state, row) {
        // A servo that says nothing also does nothing: a write to an absent
        // row must not reach the cell or the plant.
        return Answer::bare(corr, AuxStatus::Timeout);
    }
    match request.op {
        // A slot nothing wrote asks for no transaction, and putting a datagram
        // on the bus for it would be commanding a machine on the strength of
        // unwritten memory.
        AuxOpKind::None => Answer::refused(corr),
        AuxOpKind::Ping => {
            let model = sim_regs::read(&state.regs, row, RegId::ModelNumber)
                .ok()
                .and_then(Value::as_u16);
            match model {
                Some(model) => Answer {
                    model,
                    ..Answer::bare(corr, AuxStatus::Ok)
                },
                // A row whose model cell reads as something other than a model
                // number: the file is not a control table, which is this
                // build's own defect and not the machine's.
                None => Answer::refused(corr),
            }
        }
        AuxOpKind::ReadReg => match sim_regs::read(&state.regs, row, request.reg) {
            Ok(held) => Answer::value(corr, held),
            Err(_) => Answer::refused(corr),
        },
        AuxOpKind::WriteRegVerified => write_verified(state, nominal, corr, row, request),
    }
}

/// Write one register and read it back, so the answer is what the modelled
/// servo holds rather than what was sent.
fn write_verified(
    state: &mut SimState,
    nominal: i64,
    corr: u32,
    row: usize,
    request: &Request,
) -> Answer {
    let Ok(shape) = value_kind(request.reg) else {
        return Answer::refused(corr);
    };
    if shape != request.value_kind {
        // The host built the value in a shape the register does not take, which
        // is a request that cannot be run rather than a servo that answered
        // badly.
        return Answer::refused(corr);
    }
    let held = value::carried(shape, request.value);
    let Some(joint) = joint_ref(row) else {
        return Answer::refused(corr);
    };
    match request.reg {
        RegId::TorqueEnable => {
            let Some(enable) = held.as_u8() else {
                return Answer::refused(corr);
            };
            enable_write(state, nominal, row, joint, enable != 0);
        }
        RegId::GoalPosition => {
            let Some(angle) = held.as_radians() else {
                return Answer::refused(corr);
            };
            // A goal register holds what it was written whether or not the row
            // is energised -- what a limp servo does with it is nothing, which
            // is why an engagement writes the goals before it enables anything.
            set_angle(&mut state.targets, joint, angle);
            state.has_target |= flags::bit(joint);
        }
        // A servo's own reading of where it is. No write reaches it on the real
        // bus and none reaches it here: the modelled world is moved by moving
        // it.
        RegId::PresentPosition => return Answer::refused(corr),
        _ => {}
    }
    let torqued = flags::contains(state.torqued, joint);
    // The cell, whichever half of the machine the write already reached: the
    // read-back below is a read of the control table, so a plant-backed
    // register has to say what the plant now holds before it is read rather
    // than waiting for the next cycle's proprioception.
    if Regs::over(&mut state.regs)
        .write(row, request.reg, held, torqued)
        .is_err()
    {
        return Answer::refused(corr);
    }
    match sim_regs::read(&state.regs, row, request.reg) {
        Ok(read_back) if read_back == held => Answer::value(corr, read_back),
        // The cell does not hold what was written to it. Nothing in this
        // simulator produces that, and the outcome says so rather than
        // reporting a write that took.
        Ok(read_back) => Answer {
            status: AuxStatus::VerifyMismatch,
            ..Answer::value(corr, read_back)
        },
        Err(_) => Answer::refused(corr),
    }
}

/// Energise or de-energise one row, as a verified torque-enable write.
///
/// The whole of what arming this machine is: the plant's bits move, the
/// driver's belief moves with them because the write was verified, and a fresh
/// arming grants the dead-man a new window and ends both the torque-off latch
/// and the confirmation pass that was reading it back. A de-energised row
/// forgets what it was holding -- these gearboxes do not back-drive, so it
/// stands where it stands -- and a machine with nothing left energised is a
/// machine holding nothing at all.
fn enable_write(state: &mut SimState, nominal: i64, row: usize, joint: JointRef, enabled: bool) {
    let bit = flags::bit(joint);
    if enabled {
        state.torqued |= bit;
    } else {
        state.torqued = flags::without(state.torqued, bit);
        state.has_target = flags::without(state.has_target, bit);
    }
    AuxSlot::over(&mut state.aux)
        .belief()
        .verified_write(u8::try_from(row).unwrap_or(u8::MAX), enabled);
    let mut gate = GoalGate::over(&mut state.gate);
    if enabled {
        if gate.state().latched.get() {
            gate.release_latch(nominal);
            TorqueOffConfirm::over(&mut state.confirm).stand_down();
        } else {
            gate.note_liveness(nominal);
        }
    } else if flags::is_empty(state.torqued) {
        gate.clear_commanded();
    }
}

/// One servo's health, as the rotation's read of its status cells.
///
/// Every field is a cell of the row's own control table, so what a report says
/// is what a host reading those registers itself would find. The temperature is
/// [`SIM_TEMP_C`] — a constant of the model rather than a measurement of
/// anything, this plant having nothing thermal to measure — and the temperature
/// cell of every control table is provisioned with that same constant, so the
/// two ways of asking agree.
pub fn health(state: &SimState, nominal: i64, row: usize, out: &mut HealthReport) {
    out.id = SERVO_IDS.get(row).copied().unwrap_or(0);
    out.bits = sim_regs::read(&state.regs, row, RegId::HardwareErrorStatus)
        .ok()
        .and_then(Value::as_u8)
        .unwrap_or(0);
    out.volts = sim_regs::read(&state.regs, row, RegId::PresentInputVoltage)
        .ok()
        .and_then(Value::as_volts)
        .unwrap_or(0.0);
    out.temp_c = SIM_TEMP_C;
    out.sample_time = SyncTime::from_nanos(nominal);
}

/// Whether this row is off the bus.
///
/// A row nothing answers for: no ping, no read, no write, and no health report.
/// The scenario's stand-in for a servo that is dead, unplugged or was never
/// there, which is what a commissioning sequence has to fail on.
#[must_use]
pub fn is_absent(state: &SimState, row: usize) -> bool {
    joint_ref(row).is_some_and(|joint| flags::contains(state.absent, joint))
}

/// Whether the row's torque-enable cell reads as energised.
///
/// What the confirmation pass is fed: a cell that does not read as a byte is
/// not a zero, so it counts as still holding and the pass keeps reading -- the
/// one reading of an unreadable cell that cannot credit a de-torquing nobody
/// saw. A row off the bus reads the same way for the same reason: a servo that
/// answers nothing has not been seen to go limp, and a de-torquing credited to
/// silence would be the one report this driver must never make.
#[must_use]
pub fn reads_torqued(state: &SimState, row: usize) -> bool {
    if is_absent(state, row) {
        return true;
    }
    sim_regs::read(&state.regs, row, RegId::TorqueEnable)
        .ok()
        .and_then(Value::as_u8)
        .is_none_or(|held| held != 0)
}

/// The rows whose torque-enable cell still reads as energised.
///
/// The evidence a de-torquing that would not confirm is reported with: which
/// servos did not go limp, read off the control tables rather than off what the
/// driver believes -- a belief is what the writes said, and this is what the
/// machine says.
#[must_use]
pub fn rows_still_torqued(state: &SimState) -> JointFlags {
    let mut rows = JointFlags::NONE;
    for joint in flags::iter(flags::all()) {
        if let Some(index) = row(joint)
            && reads_torqued(state, index)
        {
            rows |= flags::bit(joint);
        }
    }
    rows
}
