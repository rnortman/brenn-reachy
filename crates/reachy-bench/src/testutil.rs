//! A scripted nine-servo machine behind the port seam, for tests that need one.
//!
//! Both halves of this crate talk to servos: the read-only registry sweeps
//! registers, and the pump drives a sequencer's writes. They need the same
//! fixture — a register file that answers protocol frames — so it lives here
//! rather than twice, where the two copies could disagree about what a servo
//! does with a write.
//!
//! Test-only: nothing here is compiled into the binary.

use std::cell::RefCell;
use std::collections::{HashMap, VecDeque};
use std::io;
use std::rc::Rc;
use std::time::{Duration, Instant};

use dxl_proto::frame::{
    HEADER, INST_PING, INST_READ, INST_STATUS, INST_SYNC_READ, INST_SYNC_WRITE, INST_WRITE,
};
use dxl_proto::{Reg, crc16, rad_to_counts};
use reachy_bus::{BusPort, ServoMap, reg_for};
use reachy_kin::{HeadGeometry, LegAngles, inverse_kinematics, rest_head_pose, stow_head_pose};
use reachy_motion::{EXPECTED_MODELS, ProvisionExpect, ProvisionTable, RegId};

use crate::config::BenchConfig;
use crate::pump::Clock;

/// A rail comfortably over the arm floor, in the register's tenths of a volt.
const HEALTHY_RAIL: u16 = 118;

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
    /// Reads of a register this machine answers with a frame damaged after its
    /// checksum was taken, so the host sees bytes that disagree with
    /// themselves.
    pub(crate) damaged: Vec<(u8, u16)>,
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
            damaged: Vec::new(),
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

    /// Take a write, whether it came unicast or in a grouped frame.
    ///
    /// A servo in position mode reports where its goal put it. This fixture
    /// models that as instant, so a position read after a goal write sees the
    /// written value.
    fn store(&mut self, id: u8, addr: u16, data: Vec<u8>) {
        if addr == reg_for(RegId::GoalPosition).addr {
            self.regs
                .insert((id, reg_for(RegId::PresentPosition).addr), data.clone());
        }
        self.regs.insert((id, addr), data);
    }

    /// Whether this servo answers a read of `addr` at all, spending one count
    /// of a dropout if it is in the middle of one.
    fn hushed(&mut self, id: u8, addr: u16) -> bool {
        if self.silent.contains(&id) {
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

    /// A status frame as a servo puts it on the wire, damaged after its
    /// checksum if this register is one of the damaged ones.
    fn answer(&mut self, id: u8, addr: u16, error: u8, params: &[u8]) {
        let damaged = self.damaged.contains(&(id, addr));
        let at = self.out.len() + HEADER.len() + 4;
        self.reply(id, error, params);
        if damaged {
            // One byte flipped after the checksum was taken over the frame it
            // no longer describes, which is what a wire fault looks like from
            // the host: bytes that arrived and disagree with themselves.
            if let Some(byte) = self.out.get_mut(at) {
                *byte ^= 0xFF;
            }
        }
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
        match instruction {
            INST_PING => {
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
                let error = self.errors.get(&(id, addr)).copied().unwrap_or(0);
                let mut value = self.regs.get(&(id, addr)).cloned().unwrap_or_default();
                value.resize(width, 0);
                self.answer(id, addr, error, &value);
            }
            INST_WRITE => {
                let addr = u16::from_le_bytes([params[0], params[1]]);
                let error = self.errors.get(&(id, addr)).copied().unwrap_or(0);
                if error == 0 && !self.ignored.contains(&(id, addr)) {
                    self.store(id, addr, params[2..].to_vec());
                }
                // A write is acknowledged with no parameters at all.
                self.reply(id, error, &[]);
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
                    let error = self.errors.get(&(asked, addr)).copied().unwrap_or(0);
                    let mut value = self.regs.get(&(asked, addr)).cloned().unwrap_or_default();
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

/// One grouped write as it went out: the register it addressed, and what each
/// servo in it was given.
///
/// The register file only holds where a run *ended*. A run that swept somewhere
/// and came back leaves no trace in it, so the frames themselves are the only
/// record of what a move passed through.
pub(crate) struct GroupedWrite {
    pub(crate) addr: u16,
    pub(crate) entries: Vec<(u8, Vec<u8>)>,
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
    grouped: Rc<RefCell<Vec<u16>>>,
    addressed: Rc<RefCell<Vec<Vec<u8>>>>,
    commanded: Rc<RefCell<Vec<GroupedWrite>>>,
}

impl Spy {
    /// Wrap `machine`.
    pub(crate) fn new(machine: FakeMachine) -> Self {
        Self {
            machine: Rc::new(RefCell::new(machine)),
            log: Rc::new(RefCell::new(Vec::new())),
            grouped: Rc::new(RefCell::new(Vec::new())),
            addressed: Rc::new(RefCell::new(Vec::new())),
            commanded: Rc::new(RefCell::new(Vec::new())),
        }
    }

    /// The servos each grouped *write* carried an entry for, one row per frame.
    ///
    /// A grouped write is addressed to the broadcast ID, so the only place the
    /// servos it commands appear is inside its payload — which is exactly what
    /// says whether a goal went to the joint it belongs to.
    pub(crate) fn addressed(&self) -> Rc<RefCell<Vec<Vec<u8>>>> {
        Rc::clone(&self.addressed)
    }

    /// Every grouped write in full, in the order it went out.
    pub(crate) fn commanded(&self) -> Rc<RefCell<Vec<GroupedWrite>>> {
        Rc::clone(&self.commanded)
    }

    /// The register each grouped read asked for, in the order they were asked.
    ///
    /// Separate from [`Self::log`] because every grouped frame is addressed to
    /// the broadcast ID with the same instruction byte, so the pair that
    /// identifies a unicast exchange cannot tell two grouped reads apart.
    pub(crate) fn grouped(&self) -> Rc<RefCell<Vec<u16>>> {
        Rc::clone(&self.grouped)
    }

    /// The machine behind the port, which outlives the port.
    pub(crate) fn machine(&self) -> Rc<RefCell<FakeMachine>> {
        Rc::clone(&self.machine)
    }

    /// Every instruction that crossed the wire, as (servo, instruction) pairs.
    pub(crate) fn log(&self) -> Rc<RefCell<Vec<(u8, u8)>>> {
        Rc::clone(&self.log)
    }
}

impl BusPort for Spy {
    fn write_all(&mut self, buf: &[u8]) -> io::Result<()> {
        self.log.borrow_mut().push((buf[4], buf[7]));
        if buf[7] == INST_SYNC_READ {
            self.grouped
                .borrow_mut()
                .push(u16::from_le_bytes([buf[8], buf[9]]));
        }
        if buf[7] == INST_SYNC_WRITE {
            let len = usize::from(u16::from_le_bytes([buf[5], buf[6]]));
            let params = &buf[8..8 + len - 3];
            let addr = u16::from_le_bytes([params[0], params[1]]);
            let width = usize::from(u16::from_le_bytes([params[2], params[3]]));
            // Parsed once and recorded two ways: which servos a frame carried,
            // and what each of them was given.
            let entries: Vec<(u8, Vec<u8>)> = params[4..]
                .chunks_exact(1 + width)
                .map(|entry| (entry[0], entry[1..].to_vec()))
                .collect();
            self.addressed
                .borrow_mut()
                .push(entries.iter().map(|(id, _)| *id).collect());
            self.commanded
                .borrow_mut()
                .push(GroupedWrite { addr, entries });
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

/// A port that fails whatever it is asked, so a test can tell a machine's
/// verdict from the host's own I/O going wrong.
pub(crate) struct BrokenPort;

impl BusPort for BrokenPort {
    fn write_all(&mut self, _buf: &[u8]) -> io::Result<()> {
        Err(io::Error::other("the adapter went away"))
    }

    fn read_some(&mut self, _buf: &mut [u8], _deadline: Instant) -> io::Result<usize> {
        Err(io::Error::other("the adapter went away"))
    }

    fn discard_input(&mut self) -> io::Result<()> {
        Ok(())
    }
}

/// A clock a test drives by hand: time only moves when something waits.
///
/// The pump's real clock sleeps; this one records what it was asked to wait for
/// and jumps straight there, so a thirty-second voltage budget costs a test
/// nothing and the waits themselves are what gets asserted.
#[derive(Debug, Default)]
pub(crate) struct TestClock {
    now: Duration,
    pub(crate) waits: Vec<Duration>,
}

impl Clock for TestClock {
    fn now(&self) -> Duration {
        self.now
    }

    fn sleep_until(&mut self, until: Duration) {
        self.waits.push(until);
        if until > self.now {
            self.now = until;
        }
    }
}

/// The configuration the example ships with, which is what an operator copies.
/// It carries no datum table — the first run has no way to have one — so it is
/// exactly the file a bring-up starts from.
pub(crate) fn undatumed_config() -> BenchConfig {
    let cfg =
        crate::config::parse(include_str!("../reachy-bench.example.toml")).expect("it parses");
    assert_eq!(cfg.datum, None, "the shipped example resolves no datum");
    cfg
}

/// The same file with a datum recorded, which is what a reviewed unit's copy
/// looks like and the only shape that resolves.
pub(crate) fn datumed_config() -> BenchConfig {
    let mut cfg = undatumed_config();
    cfg.datum = Some(crate::config::DatumSection {
        crank_datum: crate::config::DatumSetting::Direct,
        provenance: "a test, not a unit".to_string(),
    });
    cfg
}

/// The resolved configuration of a reviewed unit, with the bus timing wound
/// down so a test that waits on a deadline does not wait in real time.
///
/// One definition, because a knob wound down in one test module and not another
/// leaves that module either spending real retry time or testing different
/// timing than the rest, and each copy looks locally complete.
pub(crate) fn resolved() -> crate::config::Resolved {
    let mut cfg = datumed_config();
    wind_down_bus(&mut cfg);
    cfg.resolve().expect("a datumed example resolves")
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
            reg_for(RegId::ModelNumber),
            &EXPECTED_MODELS[row].to_le_bytes(),
        );
        machine.set(
            *id,
            reg_for(RegId::PresentInputVoltage),
            &HEALTHY_RAIL.to_le_bytes(),
        );
        machine.set(*id, reg_for(RegId::HardwareErrorStatus), &[0]);
        let angle = match row {
            0 => 0.0,
            1..=6 => legs[row - 1],
            _ => 0.0,
        };
        let counts = rad_to_counts(angle).expect("a resting angle places");
        machine.set(*id, reg_for(RegId::PresentPosition), &counts.to_le_bytes());
    }

    for reg in RegId::ALL {
        let Some(column) = ProvisionTable::column(reg) else {
            continue;
        };
        for (row, id) in ids.iter().enumerate() {
            let entry = reg_for(reg);
            match table.at(row, column) {
                Some(ProvisionExpect::Check(value)) => {
                    let raw = map
                        .encode_value(row, reg, value)
                        .expect("a configured expectation encodes");
                    machine.set(*id, entry, raw.as_slice());
                }
                // A recorded register holds whatever it holds; zero is a value
                // like any other and the case does not judge it.
                Some(ProvisionExpect::Record) => {
                    machine.set(*id, entry, &vec![0u8; usize::from(entry.len)]);
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
