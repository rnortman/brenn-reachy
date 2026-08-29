//! A scripted nine-servo machine behind the port seam, for tests that need one.
//!
//! Both halves of this crate talk to servos: the read-only registry sweeps
//! registers, and the bare-bus commands drive their own. They need the same
//! fixture — a register file that answers protocol frames — so it lives here
//! rather than twice, where the two copies could disagree about what a servo
//! does with a write.
//!
//! Test-only: nothing here is compiled into the binary.

use std::cell::{Cell, RefCell};
use std::collections::{HashMap, HashSet, VecDeque};
use std::io;
use std::rc::Rc;
use std::time::{Duration, Instant};

use dxl_proto::frame::{
    HEADER, INST_PING, INST_READ, INST_REBOOT, INST_STATUS, INST_SYNC_READ, INST_SYNC_WRITE,
    INST_WRITE,
};
use dxl_proto::{Reg, crc16, rad_to_counts};
use reachy_bus::{BusPort, BusTiming, ServoMap, named_reg};
use reachy_kin::{HeadGeometry, LegAngles, inverse_kinematics, rest_head_pose, stow_head_pose};
use reachy_motion::reg;
use reachy_motion::{EXPECTED_MODELS, ProvisionExpect, ProvisionTable, RegId};

use crate::bare::Clock;
use crate::config::BenchConfig;

/// A rail comfortably over the arm floor, in the register's tenths of a volt.
const HEALTHY_RAIL: u16 = 118;

/// A resting temperature, whole degrees Celsius, inside the band the registry's
/// temperature case asserts.
const RESTING_TEMP_C: u8 = 28;

/// A scripted Bus Watchdog on one servo, as a fixture a test can point at a
/// verdict.
///
/// Modelling it does not launder the hardware fact the bench command exists to
/// establish -- what a servo on a bench does with silence is that command's
/// assertion and this cannot answer it, which is why the fields say which of
/// them a hardware run has settled (see `expected`). What it does check is that
/// the command reads a latch correctly when it meets one, which is otherwise
/// only ever exercised at a powered unit.
///
/// The trip has two triggers. Silence is the real one: the host's own clock is
/// shared with this fixture, so a servo trips when the gap between two frames
/// addressed to it exceeds what its armed register says it tolerates. The frame
/// countdown is the other: a trip that arrives at a chosen frame regardless of
/// traffic, which is how a case puts a latch in the middle of a phase that is
/// keeping the bus busy.
pub(crate) struct BusWatchdogModel {
    /// The servo this one is on. Every other servo has none.
    pub(crate) id: u8,
    /// The host's clock, shared: silence is measured in the time base the
    /// command sleeps in.
    pub(crate) now: Rc<Cell<Duration>>,
    /// What one count of the register is worth.
    pub(crate) unit: Duration,
    /// Frames addressed to this servo before it trips whatever the traffic;
    /// `None` for one that trips on silence alone.
    pub(crate) trips_after_frames: Option<u32>,
    /// Whether a trip leaves the marker in the register.
    pub(crate) marks_register: bool,
    /// Whether a trip lets the servo go.
    pub(crate) drops_torque: bool,
    /// The error byte a tripped servo answers an ordinary write with, or `None`
    /// for one that goes on taking writes as though nothing had happened.
    pub(crate) refuses_writes_with: Option<u8>,
    /// When this servo was last addressed.
    last_seen: Duration,
    /// Whether it has tripped.
    latched: bool,
}

impl BusWatchdogModel {
    /// A watchdog on `id` that behaves the way the bench command expects: it
    /// trips on silence, marks its register, lets the servo go, and refuses
    /// ordinary writes with Access until a zero clears it.
    ///
    /// Two of those three fields are observation-backed and one is not, and the
    /// difference matters to anything reading a case built on this fixture:
    ///
    /// - `marks_register` and `refuses_writes_with: Some(ACCESS)` are what a
    ///   servo on this bench does — `0xff` in the register, and an error field
    ///   whose masked code is Access. The vendor's manual says Data Range in its
    ///   worked example and "read-only" (which is Access) in its prose, so it is
    ///   no help; the hardware is.
    /// - `drops_torque` is what the fault policy requires of an armed watchdog,
    ///   and it is **not what this hardware does**. A tripped servo reads Torque
    ///   Enable as 1 — the vendor's manual is right that a trip halts the servo
    ///   with acceleration and velocity applied as zero, which is a stop under
    ///   torque rather than a release. The field stays true here and the bench
    ///   command goes on asserting it: an armed watchdog that does not de-torque
    ///   is a policy-level finding awaiting its own cycle, and a fixture edited
    ///   to match would be the assertion quietly retracted.
    pub(crate) fn expected(id: u8, now: &Rc<Cell<Duration>>, unit: Duration) -> Self {
        Self {
            id,
            now: Rc::clone(now),
            unit,
            trips_after_frames: None,
            marks_register: true,
            drops_torque: true,
            refuses_writes_with: Some(ACCESS),
            last_seen: now.get(),
            latched: false,
        }
    }
}

/// The status error byte a latched watchdog answers a write with on this
/// hardware: Access, with the alert bit clear.
pub(crate) const ACCESS: u8 = 7;

/// Data Range, with the alert bit clear — the code the vendor's worked example
/// claims for a latched watchdog and the hardware does not answer with. Kept as
/// a well-formed refusal that is *not* the expected signature.
pub(crate) const DATA_RANGE: u8 = 4;

/// A scripted machine: nine servos with a register file, answering pings, reads
/// and writes over the port seam.
///
/// It answers nothing else: an instruction it does not implement gets silence,
/// and the [`Spy`] wrapped around it is what a test reads the traffic back from.
pub(crate) struct FakeMachine {
    pub(crate) regs: HashMap<(u8, u16), Vec<u8>>,
    pub(crate) errors: HashMap<(u8, u16), u8>,
    pub(crate) silent: Vec<u8>,
    /// Registers this machine acknowledges a write to and does not store, which
    /// is what a read-back mismatch looks like from the host.
    pub(crate) ignored: Vec<(u8, u16)>,
    /// Reads of a register a servo answers nothing to, counted down as they
    /// arrive: a dropout that ends, rather than a servo that is gone.
    pub(crate) mute: HashMap<(u8, u16), u32>,
    /// Registers a servo has stopped answering about for good: a servo that
    /// goes away part way through a sequence, which is the shape a cut wire or
    /// a browned-out rail has. The opposite end of [`Self::mute`], which ends.
    ///
    /// Flipped by [`Self::go_deaf`] while a command runs rather than scripted
    /// up front, so a case says *when* in the command's own transcript the
    /// servo goes away instead of counting the transactions that come first.
    pub(crate) deaf: HashSet<(u8, u16)>,
    /// Reads of a register this machine answers with a frame damaged after its
    /// checksum was taken, so the host sees bytes that disagree with
    /// themselves.
    pub(crate) damaged: Vec<(u8, u16)>,
    /// Reads of a register this machine answers with one parameter byte more
    /// than the request asked for: a well-formed frame of the wrong width,
    /// which is what the tail of an abandoned exchange looks like when it lands
    /// behind somebody else's answer.
    pub(crate) verbose: Vec<(u8, u16)>,
    /// Servos whose Goal Position register stores what is written to it even
    /// with torque off, instead of reporting the present position. Not this
    /// platform: it is the firmware the goal-shadow case exists to catch, and
    /// the only way a test can put a limp servo's goal anywhere but where it
    /// stands.
    pub(crate) unmirrored: Vec<u8>,
    /// Per servo, how many goal writes behind its present position runs: a
    /// position loop chasing a streamed goal rather than arriving instantly.
    /// The error such a servo shows is proportional to how fast the goal is
    /// moving, which is what a real one does.
    pub(crate) delay: HashMap<u8, usize>,
    /// Goals written to a delayed servo and not yet reached, oldest first.
    queued: HashMap<u8, VecDeque<Vec<u8>>>,
    /// Per servo, how many counts it closes on its goal each time its position
    /// is read: a servo that arrives *over time* rather than on the write that
    /// commanded it. A goal write moves one of these nowhere by itself, which is
    /// what makes the periods after a trajectory ends visible — the machine
    /// still on its way to a goal that has stopped moving.
    pub(crate) creep: HashMap<u8, i32>,
    /// Servos that take their goals and do not move: a stalled motor, or a
    /// write the servo never applied. Nothing on the wire distinguishes the
    /// two, which is why the tick watches positions at all.
    pub(crate) stalled: Vec<u8>,
    /// Servos that acknowledge a reboot and are never heard from again: a
    /// restart that did not come back.
    pub(crate) gone_on_reboot: Vec<u8>,
    /// Per servo, how many pings it answers nothing to before it starts
    /// answering, counted down as they arrive: a servo still restarting, which
    /// comes back partway through a polling sweep rather than at once or never.
    pub(crate) pings_ignored: HashMap<u8, u32>,
    /// Servos the reboot instruction never reaches — a frame lost or corrupted
    /// on the wire. They answer nothing to it and go on holding what they were
    /// holding, which is exactly how a servo that restarted answers a ping.
    pub(crate) deaf_to_reboot: Vec<u8>,
    /// Servos that acknowledge a reboot, restart, and come back still carrying
    /// their hardware-error byte: a condition live at this instant, or a restart
    /// that took the torque and not the latch. Not what the machine on the bench
    /// does — which is what makes it worth modelling separately.
    pub(crate) keeps_latch: Vec<u8>,
    /// One servo's Bus Watchdog, if a case scripted one.
    pub(crate) watchdog: Option<BusWatchdogModel>,
    out: VecDeque<u8>,
}

impl FakeMachine {
    pub(crate) fn new() -> Self {
        Self {
            regs: HashMap::new(),
            errors: HashMap::new(),
            silent: Vec::new(),
            ignored: Vec::new(),
            mute: HashMap::new(),
            deaf: HashSet::new(),
            damaged: Vec::new(),
            verbose: Vec::new(),
            unmirrored: Vec::new(),
            delay: HashMap::new(),
            queued: HashMap::new(),
            creep: HashMap::new(),
            stalled: Vec::new(),
            gone_on_reboot: Vec::new(),
            pings_ignored: HashMap::new(),
            deaf_to_reboot: Vec::new(),
            keeps_latch: Vec::new(),
            watchdog: None,
            out: VecDeque::new(),
        }
    }

    pub(crate) fn set(&mut self, id: u8, reg: Reg, bytes: &[u8]) {
        self.regs.insert((id, reg.addr), bytes.to_vec());
    }

    /// What `id`'s `reg` holds now.
    pub(crate) fn get(&self, id: u8, reg: Reg) -> Option<&[u8]> {
        self.regs.get(&(id, reg.addr)).map(Vec::as_slice)
    }

    /// Whether this servo is holding torque.
    fn torqued(&self, id: u8) -> bool {
        self.regs
            .get(&(id, named_reg(RegId::TorqueEnable).addr))
            .is_some_and(|bytes| bytes.first().is_some_and(|byte| *byte != 0))
    }

    /// Whether this servo's Goal Position register is a mirror of Present
    /// Position rather than a store.
    ///
    /// What these servos do with torque off: a goal read comes back as the
    /// present position and a goal write is acknowledged and dropped.
    fn mirroring(&self, id: u8) -> bool {
        !self.torqued(id) && !self.unmirrored.contains(&id)
    }

    /// The register a read of `addr` actually comes out of.
    fn source(&self, id: u8, addr: u16) -> u16 {
        if addr == named_reg(RegId::GoalPosition).addr && self.mirroring(id) {
            return named_reg(RegId::PresentPosition).addr;
        }
        addr
    }

    /// Take a write, whether it came unicast or in a grouped frame.
    ///
    /// A servo in position mode reports where its goal put it. This fixture
    /// models that as instant unless the servo was given a delay, so a position
    /// read after a goal write sees the written value — or the value written
    /// that many goals ago. A goal written to a mirroring servo goes nowhere,
    /// and a goal stored by a limp unmirrored one moves nothing: torque is what
    /// makes a target a motion, and a stalled servo not even that.
    fn store(&mut self, id: u8, addr: u16, data: Vec<u8>) {
        if addr == named_reg(RegId::GoalPosition).addr {
            if self.mirroring(id) {
                return;
            }
            if self.torqued(id)
                && !self.stalled.contains(&id)
                && !self.creep.contains_key(&id)
                && let Some(reached) = self.arriving(id, &data)
            {
                self.regs
                    .insert((id, named_reg(RegId::PresentPosition).addr), reached);
            }
        }
        self.regs.insert((id, addr), data);
    }

    /// Which goal this servo reaches on this write: the one just written, or —
    /// for a delayed servo — the one written that many goals ago, once it has
    /// taken that many. A delayed servo that has not yet filled its queue is
    /// still standing where it was.
    fn arriving(&mut self, id: u8, data: &[u8]) -> Option<Vec<u8>> {
        let delay = self.delay.get(&id).copied().unwrap_or(0);
        if delay == 0 {
            return Some(data.to_vec());
        }
        let queue = self.queued.entry(id).or_default();
        queue.push_back(data.to_vec());
        if queue.len() > delay {
            return queue.pop_front();
        }
        None
    }

    /// Move a creeping servo one step of its own toward the goal it holds.
    ///
    /// Called on every read of its position, which is once per control period,
    /// so a step is per period and the servo arrives after as many periods as
    /// the distance divides into. A servo without a creep set, without torque,
    /// or with either register unwritten stands exactly where it was.
    fn crept(&mut self, id: u8) {
        let Some(step) = self.creep.get(&id).copied() else {
            return;
        };
        if !self.torqued(id) {
            return;
        }
        let (Some(at), Some(to)) = (
            self.counts(id, RegId::PresentPosition),
            self.counts(id, RegId::GoalPosition),
        ) else {
            return;
        };
        let remaining = to - at;
        let moved = if remaining > step {
            step
        } else if remaining < -step {
            -step
        } else {
            remaining
        };
        self.set(
            id,
            named_reg(RegId::PresentPosition),
            &(at + moved).to_le_bytes(),
        );
    }

    /// Stop answering reads of `addr` on `id`, from now on.
    pub(crate) fn go_deaf(&mut self, id: u8, addr: u16) {
        self.deaf.insert((id, addr));
    }

    /// What `id`'s four-byte position register holds, as the signed count it is.
    fn counts(&self, id: u8, reg: RegId) -> Option<i32> {
        let bytes: [u8; 4] = self.get(id, named_reg(reg))?.try_into().ok()?;
        Some(i32::from_le_bytes(bytes))
    }

    /// Whether this servo answers a read of `addr` at all, spending one count
    /// of a dropout if it is in the middle of one.
    fn hushed(&mut self, id: u8, addr: u16) -> bool {
        if self.silent.contains(&id) {
            return true;
        }
        if self.deaf.contains(&(id, addr)) {
            return true;
        }
        match self.mute.get_mut(&(id, addr)) {
            Some(left) if *left > 0 => {
                *left -= 1;
                true
            }
            _ => false,
        }
    }

    /// A status frame as a servo puts it on the wire — damaged after its
    /// checksum if this register is one of the damaged ones, and one byte wide
    /// of the request if it is one of the verbose ones.
    fn answer(&mut self, id: u8, addr: u16, error: u8, params: &[u8]) {
        let damaged = self.damaged.contains(&(id, addr));
        let at = self.out.len() + HEADER.len() + 4;
        if self.verbose.contains(&(id, addr)) {
            let mut wide = params.to_vec();
            wide.push(0);
            self.reply(id, error, &wide);
        } else {
            self.reply(id, error, params);
        }
        if damaged {
            // One byte flipped after the checksum was taken over the frame it
            // no longer describes, which is what a wire fault looks like from
            // the host: bytes that arrived and disagree with themselves.
            if let Some(byte) = self.out.get_mut(at) {
                *byte ^= 0xFF;
            }
        }
    }

    /// Let the scripted watchdog see this frame, tripping it if this is the
    /// frame that trips it.
    ///
    /// Called before the instruction is answered, so a servo that has just
    /// tripped answers the frame that found it as a tripped servo does.
    fn watchdog_saw(&mut self, id: u8) {
        let Some(mut dog) = self.watchdog.take() else {
            return;
        };
        if dog.id != id {
            self.watchdog = Some(dog);
            return;
        }
        let now = dog.now.get();
        let armed = self
            .get(id, named_reg(RegId::BusWatchdog))
            .and_then(|bytes| bytes.first().copied())
            .unwrap_or(0);
        let counted_out = armed > 0
            && armed != 0xFF
            && self.torqued(id)
            && now.saturating_sub(dog.last_seen) > dog.unit * u32::from(armed);
        // A forced trip fires once and is spent. A count that stayed at zero
        // would re-latch the servo on the frame after every clear, which is a
        // machine no clear can ever recover -- and it would hide whether the
        // clear worked.
        let forced = match dog.trips_after_frames {
            Some(0) => {
                dog.trips_after_frames = None;
                true
            }
            Some(left) => {
                dog.trips_after_frames = Some(left - 1);
                false
            }
            None => false,
        };
        if !dog.latched && (counted_out || forced) {
            dog.latched = true;
            if dog.marks_register {
                self.set(id, named_reg(RegId::BusWatchdog), &[0xFF]);
            }
            if dog.drops_torque {
                self.set(id, named_reg(RegId::TorqueEnable), &[0]);
            }
        }
        dog.last_seen = now;
        self.watchdog = Some(dog);
    }

    /// The error byte a tripped servo answers this write with, if it refuses it.
    ///
    /// A zero to the Bus Watchdog is the documented clear and is always taken;
    /// everything else is refused for as long as the trip stands.
    fn watchdog_refusal(&mut self, id: u8, addr: u16, data: &[u8]) -> Option<u8> {
        let dog = self.watchdog.as_mut()?;
        if dog.id != id || !dog.latched {
            return None;
        }
        let clearing = addr == named_reg(RegId::BusWatchdog).addr && data.first() == Some(&0);
        if clearing {
            dog.latched = false;
            dog.last_seen = dog.now.get();
            return None;
        }
        dog.refuses_writes_with
    }

    /// A status frame as a servo puts it on the wire.
    fn reply(&mut self, id: u8, error: u8, params: &[u8]) {
        let mut frame = Vec::from(HEADER);
        frame.push(id);
        let len = u16::try_from(params.len() + 4).expect("a fixture reply is short");
        frame.extend_from_slice(&len.to_le_bytes());
        frame.push(INST_STATUS);
        frame.push(error);
        frame.extend_from_slice(params);
        frame.extend_from_slice(&crc16(&frame).to_le_bytes());
        self.out.extend(frame);
    }
}

impl BusPort for FakeMachine {
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        let id = buf[4];
        let len = usize::from(u16::from_le_bytes([buf[5], buf[6]]));
        let instruction = buf[7];
        let params = &buf[8..8 + len - 3];
        if self.silent.contains(&id) {
            return Ok(());
        }
        self.watchdog_saw(id);
        match instruction {
            INST_PING => {
                if let Some(left) = self.pings_ignored.get_mut(&id)
                    && *left > 0
                {
                    *left -= 1;
                    return Ok(());
                }
                let model = self
                    .regs
                    .get(&(id, 0))
                    .cloned()
                    .unwrap_or_else(|| vec![0, 0]);
                self.reply(id, 0, &[model[0], model[1], 42]);
            }
            INST_READ => {
                let addr = u16::from_le_bytes([params[0], params[1]]);
                let width = usize::from(u16::from_le_bytes([params[2], params[3]]));
                if self.hushed(id, addr) {
                    return Ok(());
                }
                if addr == named_reg(RegId::PresentPosition).addr {
                    self.crept(id);
                }
                let error = self.errors.get(&(id, addr)).copied().unwrap_or(0);
                let source = self.source(id, addr);
                let mut value = self.regs.get(&(id, source)).cloned().unwrap_or_default();
                value.resize(width, 0);
                self.answer(id, addr, error, &value);
            }
            INST_WRITE => {
                let addr = u16::from_le_bytes([params[0], params[1]]);
                // A tripped watchdog refuses the write instead of taking it,
                // which is the signature a host sees rather than a value that
                // went in.
                if let Some(refusal) = self.watchdog_refusal(id, addr, &params[2..]) {
                    self.reply(id, refusal, &[]);
                    return Ok(());
                }
                let error = self.errors.get(&(id, addr)).copied().unwrap_or(0);
                if error == 0 && !self.ignored.contains(&(id, addr)) {
                    self.store(id, addr, params[2..].to_vec());
                }
                // A write is acknowledged with no parameters at all.
                self.reply(id, error, &[]);
            }
            INST_REBOOT => {
                // A servo the instruction never reached answers nothing and
                // restarts nothing: it is still there, still holding, and a
                // ping cannot tell it from one that came back.
                if self.deaf_to_reboot.contains(&id) {
                    return Ok(());
                }
                // Acknowledged with no parameters, then the servo restarts: it
                // comes back with Torque Enable cleared, holding nothing, and
                // with its hardware-error byte at zero — which is what this
                // machine does on the bench, recorded in the runbook's open
                // observations. The position register is left as it was; that
                // one is not established, so the fixture claims nothing.
                self.reply(id, 0, &[]);
                if self.gone_on_reboot.contains(&id) {
                    self.silent.push(id);
                    return Ok(());
                }
                self.set(id, named_reg(RegId::TorqueEnable), &[0]);
                if !self.keeps_latch.contains(&id) {
                    self.set(id, named_reg(RegId::HardwareErrorStatus), &[0]);
                }
            }
            INST_SYNC_READ => {
                // Broadcast: address, width, then the servos asked. Each answers
                // in the order it was asked, and a silent one does not.
                let addr = u16::from_le_bytes([params[0], params[1]]);
                let width = usize::from(u16::from_le_bytes([params[2], params[3]]));
                for &asked in &params[4..] {
                    if self.hushed(asked, addr) {
                        continue;
                    }
                    if addr == named_reg(RegId::PresentPosition).addr {
                        self.crept(asked);
                    }
                    let error = self.errors.get(&(asked, addr)).copied().unwrap_or(0);
                    let source = self.source(asked, addr);
                    let mut value = self.regs.get(&(asked, source)).cloned().unwrap_or_default();
                    value.resize(width, 0);
                    self.answer(asked, addr, error, &value);
                }
            }
            INST_SYNC_WRITE => {
                // Broadcast: address, width, then (servo, payload) per entry.
                // The protocol acknowledges nothing, so neither does this.
                let addr = u16::from_le_bytes([params[0], params[1]]);
                let width = usize::from(u16::from_le_bytes([params[2], params[3]]));
                for entry in params[4..].chunks_exact(1 + width) {
                    let target = entry[0];
                    if self.silent.contains(&target)
                        || self.errors.contains_key(&(target, addr))
                        || self.ignored.contains(&(target, addr))
                    {
                        continue;
                    }
                    self.store(target, addr, entry[1..].to_vec());
                }
            }
            // Anything else is a fixture that was asked to do something this
            // machine does not do; silence makes the caller time out and the
            // recorded instruction makes the test say why.
            _ => {}
        }
        Ok(())
    }

    fn read_some(&mut self, buf: &mut [u8], _deadline: Instant) -> io::Result<usize> {
        let mut taken = 0;
        while taken < buf.len() {
            match self.out.pop_front() {
                Some(byte) => {
                    buf[taken] = byte;
                    taken += 1;
                }
                None => break,
            }
        }
        Ok(taken)
    }

    fn discard_input(&mut self) -> io::Result<()> {
        self.out.clear();
        Ok(())
    }
}

/// A port that records every instruction that crosses it, over a machine the
/// test keeps a handle on.
///
/// Shared rather than owned: a caller hands the port to a `Bus` and does not get
/// it back, and what the registers hold *afterwards* — the goals a sequence left
/// in the servos — is exactly what several tests are about.
pub(crate) struct Spy {
    machine: Rc<RefCell<FakeMachine>>,
    log: Rc<RefCell<Vec<(u8, u8)>>>,
    reads: Rc<RefCell<Vec<(u8, u16)>>>,
    sync_ids: Rc<RefCell<Vec<u8>>>,
}

impl Spy {
    /// Wrap `machine`.
    pub(crate) fn new(machine: FakeMachine) -> Self {
        Self::sharing(Rc::new(RefCell::new(machine)))
    }

    /// Wrap a machine another port already wrapped, with a fresh traffic log.
    ///
    /// One register file behind two runs is what a second command in one power
    /// cycle is: the RAM the first run wrote is still holding what it wrote, and
    /// the servos are still holding torque.
    pub(crate) fn sharing(machine: Rc<RefCell<FakeMachine>>) -> Self {
        Self {
            machine,
            log: Rc::new(RefCell::new(Vec::new())),
            reads: Rc::new(RefCell::new(Vec::new())),
            sync_ids: Rc::new(RefCell::new(Vec::new())),
        }
    }

    /// The machine behind the port, which outlives the port.
    pub(crate) fn machine(&self) -> Rc<RefCell<FakeMachine>> {
        Rc::clone(&self.machine)
    }

    /// Every instruction that crossed the wire, as (servo, instruction) pairs.
    pub(crate) fn log(&self) -> Rc<RefCell<Vec<(u8, u8)>>> {
        Rc::clone(&self.log)
    }

    /// Every unicast read, as (servo, register address) pairs, in the order they
    /// were asked.
    ///
    /// Separate from [`Self::log`], which carries the instruction byte only: one
    /// read looks like every other there, so which register a case actually put
    /// on the wire — and how many times — is only visible here.
    pub(crate) fn reads(&self) -> Rc<RefCell<Vec<(u8, u16)>>> {
        Rc::clone(&self.reads)
    }

    /// Every servo a grouped read named, in the order the frames named them.
    ///
    /// A grouped frame is addressed to the broadcast id and carries its roster
    /// in its parameters, so [`Self::log`] cannot say who it reached. This is
    /// what keeps the roster property assertable over a request that does not
    /// wear one address.
    pub(crate) fn sync_ids(&self) -> Rc<RefCell<Vec<u8>>> {
        Rc::clone(&self.sync_ids)
    }
}

impl BusPort for Spy {
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        self.log.borrow_mut().push((buf[4], buf[7]));
        if buf[7] == INST_READ {
            self.reads
                .borrow_mut()
                .push((buf[4], u16::from_le_bytes([buf[8], buf[9]])));
        }
        if buf[7] == INST_SYNC_READ {
            // Parameters are the address and the width, two bytes each, and
            // then one byte per servo asked.
            let len = usize::from(u16::from_le_bytes([buf[5], buf[6]]));
            self.sync_ids
                .borrow_mut()
                .extend_from_slice(&buf[12..8 + len - 3]);
        }
        self.machine.borrow_mut().write_all(buf)
    }

    fn read_some(&mut self, buf: &mut [u8], deadline: Instant) -> io::Result<usize> {
        self.machine.borrow_mut().read_some(buf, deadline)
    }

    fn discard_input(&mut self) -> io::Result<()> {
        self.machine.borrow_mut().discard_input()
    }
}

/// A clock a test drives by hand: time only moves when something waits.
///
/// The real clock sleeps; this one records what it was asked to wait for
/// and jumps straight there, so a thirty-second voltage budget costs a test
/// nothing and the waits themselves are what gets asserted.
#[derive(Debug, Default)]
pub(crate) struct TestClock {
    /// Shared rather than owned, so a fixture that models a timer of its own --
    /// a servo's bus watchdog -- measures silence in the same time base the
    /// command sleeps in.
    now: Rc<Cell<Duration>>,
    pub(crate) waits: Vec<Duration>,
}

impl TestClock {
    /// A clock reading the instant `now` holds.
    pub(crate) fn sharing(now: &Rc<Cell<Duration>>) -> Self {
        Self {
            now: Rc::clone(now),
            waits: Vec::new(),
        }
    }
}

impl Clock for TestClock {
    fn now(&self) -> Duration {
        self.now.get()
    }

    fn sleep_until(&mut self, until: Duration) {
        self.waits.push(until);
        if until > self.now.get() {
            self.now.set(until);
        }
    }
}

/// The configuration the example ships with, which is what an operator copies
/// and therefore the file every command is exercised against.
pub(crate) fn example_config() -> BenchConfig {
    crate::config::parse(include_str!("../reachy-bench.example.toml")).expect("it parses")
}

/// What a command needs off a configuration — the servo map and the bus timing —
/// with the timing wound down so a test that waits on a deadline does not wait
/// in real time.
///
/// One definition, because a knob wound down in one test module and not another
/// leaves that module either spending real retry time or testing different
/// timing than the rest, and each copy looks locally complete.
pub(crate) struct Configured {
    /// Joints, IDs, registers and counts.
    pub(crate) map: ServoMap,
    /// Deadline and retry policy.
    pub(crate) timing: BusTiming,
}

/// What a command consumes off `file`.
///
/// The one place the struct is assembled, so a case that wants a knob at another
/// value changes the knob in its `BenchConfig` and comes back through here.
pub(crate) fn configured(file: &BenchConfig) -> Configured {
    Configured {
        map: ServoMap::new(file.servo_ids().expect("the roster is nine servos")),
        timing: file.bus_timing().expect("the timing resolves"),
    }
}

/// The configuration of a reviewed unit as a command consumes it.
pub(crate) fn resolved() -> Configured {
    let mut cfg = example_config();
    wind_down_bus(&mut cfg);
    configured(&cfg)
}

/// Wind `cfg`'s bus timing down to the shortest waits the transaction layer
/// will take, so a test against a scripted machine spends no real time.
pub(crate) fn wind_down_bus(cfg: &mut BenchConfig) {
    cfg.bus.host_allowance_ms = 1;
    cfg.bus.retry_attempts = 1;
    cfg.bus.retry_spacing_ms = 0;
}

/// A machine holding exactly what the configuration says it should, resting at
/// `legs`.
pub(crate) fn machine_at(cfg: &BenchConfig, legs: &[f64; 6]) -> FakeMachine {
    let ids = cfg.servo_ids().expect("the roster is nine servos");
    let map = ServoMap::new(ids);
    let table = cfg.provision_table();
    let mut machine = FakeMachine::new();

    for (row, id) in ids.iter().enumerate() {
        machine.set(
            *id,
            named_reg(RegId::ModelNumber),
            &EXPECTED_MODELS[row].to_le_bytes(),
        );
        machine.set(
            *id,
            named_reg(RegId::PresentInputVoltage),
            &HEALTHY_RAIL.to_le_bytes(),
        );
        machine.set(*id, named_reg(RegId::HardwareErrorStatus), &[0]);
        machine.set(*id, named_reg(RegId::PresentTemperature), &[RESTING_TEMP_C]);
        let angle = match row {
            0 => 0.0,
            1..=6 => legs[row - 1],
            _ => 0.0,
        };
        let counts = rad_to_counts(angle).expect("a resting angle places");
        machine.set(
            *id,
            named_reg(RegId::PresentPosition),
            &counts.to_le_bytes(),
        );
    }

    for reg in reg::named() {
        let Some(column) = ProvisionTable::column(reg) else {
            continue;
        };
        for (row, id) in ids.iter().enumerate() {
            let at = named_reg(reg);
            match table.at(row, column) {
                Some(ProvisionExpect::Check(value)) => {
                    let raw = map
                        .encode_value(row, reg, value)
                        .expect("a configured expectation encodes");
                    machine.set(*id, at, raw.as_slice());
                }
                // A recorded register holds whatever it holds; zero is a value
                // like any other and the case does not judge it.
                Some(ProvisionExpect::Record) => {
                    machine.set(*id, at, &vec![0u8; usize::from(at.len)]);
                }
                _ => {}
            }
        }
    }
    machine
}

/// The six crank angles the stow pose holds.
pub(crate) fn stow_legs() -> [f64; 6] {
    let mut angles = LegAngles([0.0; 6]);
    inverse_kinematics(&HeadGeometry::default(), &stow_head_pose(), &mut angles)
        .expect("the stow pose is reachable");
    angles.0
}

/// The six crank angles the tight resting configuration holds.
pub(crate) fn rest_legs() -> [f64; 6] {
    let mut angles = LegAngles([0.0; 6]);
    inverse_kinematics(&HeadGeometry::default(), &rest_head_pose(), &mut angles)
        .expect("the resting pose is reachable");
    angles.0
}
