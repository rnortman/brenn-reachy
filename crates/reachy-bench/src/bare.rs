//! The bench's bare-bus commands: the ones that need no tick, no sequencer and
//! no kinematics.
//!
//! Each one opens with a bus and a roster and nothing else. `provision` writes
//! the antennas' operating mode, `reboot` restarts the servos, and `off` sweeps
//! torque off. None of them commands an angle, so none of them needs an
//! envelope, a pose or a control loop; what they share is the register-level
//! plumbing at the bottom of this file.
//!
//! Ports are the caller's: every command here takes one already open, so the
//! whole surface is exercisable against a scripted machine with no device in
//! sight. What a command prints goes out through a callback for the same
//! reason.
//!
//! `off` is a de-torque, and nothing gates a de-torque: it asks every servo on
//! the roster, carries on past a servo that will not answer, and reports the
//! ones left unacknowledged after all nine have been asked. It measures
//! nothing and refuses nothing about the machine; the one thing it refuses up
//! front is a host-side failure to encode the byte all nine are written, which
//! leaves no write to attempt.

use core::time::Duration;
use std::time::Instant;

use dxl_proto::{HardwareError, counts_to_rad};
use reachy_bus::{
    Bus, BusPort, BusTiming, MapError, RawValue, ServoMap, XactError, named_reg, reg_for,
    with_retry,
};
use reachy_motion::joints::{Name, ROW_COUNT, row};
use reachy_motion::reg::Name as RegName;
use reachy_motion::value;
use reachy_motion::{
    EXPECTED_MODELS, EXPECTED_OPERATING_MODES, JointRef, RegId, Value, ValueShape,
};
use thiserror::Error;

/// The joints `provision` writes: the two antennas, whose extended-position
/// mode is this project's own provisioning rather than the vendor's.
const PROVISIONED_JOINTS: [JointRef; 2] = [JointRef::AntennaRight, JointRef::AntennaLeft];

/// Polling rounds before a rebooted servo is reported as gone.
///
/// One round asks every servo still missing, so the budget is the whole sweep's
/// and not one servo's: the reboots all went out at the same instant, and nine
/// serial budgets would have made an incident cost nine times what this doc
/// says. A round is spaced by the bus's own retry spacing and each unanswered
/// ping in it costs an exchange deadline, so the budget is a couple of seconds
/// at the shipped timing with one servo missing and grows with the number
/// missing rather than with the roster. How long one of these servos takes to
/// come back is not something this project has measured -- the count is set well
/// above any restart a reboot is likely to take, and the run prints what each
/// servo actually waited.
const BOOT_POLLS: u32 = 100;

/// A host's clock: elapsed time on an epoch it owns, and the sleep.
///
/// Library code takes `now` as a [`Duration`] since a caller-owned epoch and
/// never reads a clock itself. This is that epoch.
pub trait Clock {
    /// Elapsed time since the epoch.
    fn now(&self) -> Duration;

    /// Block until `until` has elapsed. A time already past returns at once.
    fn sleep_until(&mut self, until: Duration);
}

/// The real clock: a monotonic instant taken when the run began.
#[derive(Clone, Copy, Debug)]
pub struct MonotonicClock {
    epoch: Instant,
}

impl MonotonicClock {
    /// A clock whose epoch is now.
    #[must_use]
    pub fn new() -> Self {
        Self {
            epoch: Instant::now(),
        }
    }
}

impl Default for MonotonicClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for MonotonicClock {
    fn now(&self) -> Duration {
        self.epoch.elapsed()
    }

    fn sleep_until(&mut self, until: Duration) {
        let elapsed = self.now();
        if until > elapsed {
            std::thread::sleep(until - elapsed);
        }
    }
}

/// Why a bare-bus command stopped.
#[derive(Debug, Error)]
pub enum BareError {
    /// A transaction failed in a way that is not a verdict about the machine.
    #[error("servo {id}: {source}")]
    Bus {
        /// The servo addressed.
        id: u8,
        /// What went wrong.
        source: XactError,
    },

    /// A register read failed on the wire. Separate from [`BareError::Bus`]
    /// because a read names the register it was after, and that is the half of
    /// the message an operator reads a bench session off.
    #[error("servo {id} {}: {source}", RegName(*.reg))]
    BusRead {
        /// The servo addressed.
        id: u8,
        /// The register being read.
        reg: RegId,
        /// What went wrong.
        source: XactError,
    },

    /// A register's value could not be put on the wire, or what came back could
    /// not be read as the register's own shape.
    #[error("servo {id} {}: {source}", RegName(*.reg))]
    Map {
        /// The servo addressed.
        id: u8,
        /// The register concerned.
        reg: RegId,
        /// What the map refused.
        source: MapError,
    },

    /// A servo about to be provisioned answered as a part this platform does
    /// not carry. Whatever holds that ID is not the servo whose non-volatile
    /// registers this project writes.
    #[error("servo {id} reports model {model}, where this platform's is {expected}")]
    WrongPart {
        /// The servo addressed.
        id: u8,
        /// The model it answered with.
        model: u16,
        /// The model this platform carries at that position.
        expected: u16,
    },

    /// A servo was holding torque where a non-volatile write requires it
    /// released. Refused across the whole roster before anything is written, so
    /// a half-provisioned machine is not a state this leaves behind.
    #[error("servo {id} is holding torque; release it with `off` before provisioning")]
    TorqueHeld {
        /// The servo addressed.
        id: u8,
    },

    /// A servo did not acknowledge its torque-off write, so it may still be
    /// holding. Reported after the whole sweep has run — every servo is always
    /// asked, and this is what the run has to say about the ones that did not
    /// answer, not something it could have refused up front.
    #[error("servo {id} did not acknowledge torque off and may still be holding")]
    TorqueOffUnacked {
        /// The first servo in roster order left unacknowledged; the run's own
        /// report lists all of them.
        id: u8,
    },

    /// Torque-off could not be put on the wire at all, so no servo was written.
    /// A host-side contract break — the register table and the value shape
    /// disagree — and not a verdict about the machine: nothing was asked, so
    /// nothing withheld an acknowledgement. Raised before the sweep because the
    /// bytes are the same for every servo, so a failure here is a failure for
    /// all nine and there is no best effort left to make.
    #[error("torque off could not be encoded for {}, so nothing was written: {source}", RegName(*.reg))]
    TorqueOffUnsent {
        /// The register the value was for.
        reg: RegId,
        /// What the map refused.
        source: MapError,
    },

    /// A command was asked for a servo the configured roster does not carry.
    /// Whatever holds that ID is not one of this machine's nine joints, so it
    /// is refused by name rather than skipped or addressed anyway.
    #[error("servo {id} is not in the configured roster {roster:?}")]
    OffRoster {
        /// The ID asked for.
        id: u8,
        /// The servos the configuration carries, in bus order.
        roster: [u8; ROW_COUNT],
    },

    /// A rebooted servo never answered again within the budget it was given.
    /// Nothing on the reboot path holds torque, so nothing is released in
    /// response: this is a report, and the servo is either still restarting or
    /// gone.
    #[error("servo {id} answered none of {polls} pings over {waited:?} after its reboot: {source}")]
    NotBack {
        /// The servo that stayed silent.
        id: u8,
        /// Pings it was asked.
        polls: u32,
        /// How long those pings took.
        waited: Duration,
        /// What the last of them failed with.
        source: XactError,
    },

    /// A rebooted servo answered again but came back still holding torque, so
    /// it never restarted: the instruction was lost on the wire, or refused.
    /// Nothing on the reboot path enables torque, so this is torque the servo
    /// held all along and the reboot did not take — reported rather than
    /// written off, because the command's whole promise is that what it reaches
    /// lets go.
    #[error("servo {id} answered after its reboot still holding torque, so it did not restart")]
    NotRestarted {
        /// The servo that kept its torque.
        id: u8,
    },

    /// A servo neither acknowledged its reboot nor came back with torque to
    /// drop, so nothing observed says it restarted. The case is a machine
    /// already limp — which is what a latched shutdown leaves, and the state an
    /// operator reaches for `reboot` in: a servo that never took the
    /// instruction answers pings exactly as one that did, and the torque that
    /// tells them apart was already off. Reported rather than passed, because
    /// the alternative is a success line over a latch that is still set.
    #[error(
        "servo {id} did not acknowledge its reboot and came back holding nothing, so no reading \
         says it restarted: {source}"
    )]
    RestartUnconfirmed {
        /// The servo whose restart could not be established.
        id: u8,
        /// What its reboot instruction failed with.
        source: XactError,
    },

    /// A rebooted servo answered still carrying a hardware-error byte. A restart
    /// clears that byte, so one that survives means either the restart did not
    /// happen or the condition behind the bits is live at this instant — and a
    /// recovery command that recovered nothing must not exit as though it had.
    #[error(
        "servo {id} answered after its reboot still reporting hardware error bits {bits:#04x}, \
         which a restart clears"
    )]
    StillLatched {
        /// The servo still carrying bits.
        id: u8,
        /// The byte it came back with.
        bits: u8,
    },
}

/// Sweep torque off every servo on the roster, and say what each one answered.
///
/// The way out of any session: nothing is commissioned, nothing is measured,
/// nothing is commanded, and where the machine is standing gates nothing. Every
/// servo is asked whatever the wire does for the ones before it, because a
/// release that stopped at the first silent servo would leave the rest holding
/// the head up.
///
/// The error a machine can cause is raised after all nine writes have gone out,
/// and says a servo never acknowledged its own — the report of an incomplete
/// release rather than of a release that did not happen. The other one is
/// host-side and comes first: the byte every servo is written is encoded once,
/// up front, and a map that will not carry it means no servo can be written at
/// all.
pub fn off<P: BusPort>(
    map: &ServoMap,
    timing: BusTiming,
    port: P,
    line: &mut dyn FnMut(&str),
) -> Result<(), BareError> {
    let mut bus = Bus::new(port, timing);
    let entry = named_reg(RegId::TorqueEnable);
    let mut unacked: Vec<u8> = Vec::new();
    // One value for all nine: Torque Enable is a byte, and a byte's carriage
    // does not depend on which servo it is going to. Encoded once, before
    // anything goes out, so a host-side refusal is reported as the "nothing was
    // written" it is instead of nine servos each reported as unacknowledged.
    let raw = map
        .encode_value(0, RegId::TorqueEnable, value::u8(0))
        .map_err(|source| BareError::TorqueOffUnsent {
            reg: RegId::TorqueEnable,
            source,
        })?;

    line(
        "off: every servo on the roster is written torque-off, wherever the machine is \
         standing. The head settles as it goes, so take its weight if it is up.",
    );

    for id in map.ids().iter().copied() {
        match with_retry(&mut bus, |bus| bus.write_reg_verified(id, entry, &raw)) {
            Ok(()) => line(&format!("  servo {id}: torque off, read back")),
            Err(source) => {
                line(&format!(
                    "  servo {id}: torque off unacknowledged, may still be holding ({source})"
                ));
                unacked.push(id);
            }
        }
    }

    // After the report, not before it: the operator reads what the machine did
    // and then the command's verdict on it.
    match unacked.first() {
        Some(id) => Err(BareError::TorqueOffUnacked { id: *id }),
        None => {
            line("released; every servo on the roster acknowledged torque off.");
            Ok(())
        }
    }
}

/// Write the antennas' extended-position operating mode.
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
) -> Result<(), BareError> {
    let mut bus = Bus::new(port, timing);
    let mut found = Vec::new();

    for joint in PROVISIONED_JOINTS {
        let row = row(joint).expect("a named joint has a bus row");
        let id = map.ids()[row];
        let info = with_retry(&mut bus, |bus| bus.ping(id))
            .map_err(|source| BareError::Bus { id, source })?;
        let expected = EXPECTED_MODELS[row];
        if info.model != expected {
            return Err(BareError::WrongPart {
                id,
                model: info.model,
                expected,
            });
        }
        if read_byte(&mut bus, map, row, RegId::TorqueEnable)? != 0 {
            return Err(BareError::TorqueHeld { id });
        }
        let mode = read_byte(&mut bus, map, row, RegId::OperatingMode)?;
        line(&format!(
            "  {}: servo {id}, model {model}, torque off, operating mode {mode}",
            Name(joint),
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
            .encode_value(row, RegId::OperatingMode, value::u8(wanted))
            .map_err(|source| BareError::Map {
                id,
                reg: RegId::OperatingMode,
                source,
            })?;
        let entry = named_reg(RegId::OperatingMode);
        with_retry(&mut bus, |bus| bus.write_eeprom_verified(id, entry, &raw))
            .map_err(|source| BareError::Bus { id, source })?;
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
///
/// And the byte an operator came here to clear is judged: a servo that answers
/// still reporting hardware-error bits fails the command, whichever bits they
/// are. This machine clears them to zero on a restart — the observation is in
/// `docs/bench-runbook.md`, under the open observations, including for the
/// chronic input-voltage bit every servo latches while running. So a byte that
/// survives means the restart did not happen or the condition is live at this
/// instant, and neither of those is a recovery. There is no per-bit tolerance
/// here: the input-voltage latch is unexplained, and writing "probably fine"
/// into a recovery command would settle it by assertion.
pub fn reboot<P: BusPort>(
    map: &ServoMap,
    timing: BusTiming,
    port: P,
    target: Option<u8>,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), BareError> {
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

    let mut trouble: Option<BareError> = None;
    let mut back = Vec::with_capacity(targets.len());
    for (row, id, answer) in wait_for_all(&mut bus, &targets, clock) {
        match answer {
            Ok(waited) => {
                line(&format!(
                    "  servo {id}: answering {:.2} s after the instruction went out",
                    waited.as_secs_f64()
                ));
                back.push((row, id));
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
                    trouble.get_or_insert(BareError::NotRestarted { id });
                } else if let Some(at) = unacked.iter().position(|(deaf, _)| *deaf == id) {
                    let (_, source) = unacked.remove(at);
                    line(&format!(
                        "  servo {id}: it took no acknowledgement and had no torque to drop, so \
                         nothing here says it restarted"
                    ));
                    trouble.get_or_insert(BareError::RestartUnconfirmed { id, source });
                } else if read.bits != 0 {
                    // Last of the three, because the two above explain a byte
                    // that stayed: a servo that never restarted was never going
                    // to clear anything. This is the servo that did restart and
                    // is reporting bits anyway.
                    line(&format!(
                        "  servo {id}: the bits are still set after a restart that took, so either \
                         the reboot did not reach the latch or the condition is live now"
                    ));
                    trouble.get_or_insert(BareError::StillLatched {
                        id,
                        bits: read.bits,
                    });
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
            line(
                "rebooted; every servo that came back is limp and reporting no hardware error. \
                 `selftest` reads the machine.",
            );
            Ok(())
        }
    }
}

/// The servos a reboot addresses: the one asked for, or the whole roster in bus
/// order.
///
/// Each carries its row, because the row is what turns a reading into the
/// register widths and the joint it belongs to.
fn reboot_targets(map: &ServoMap, target: Option<u8>) -> Result<Vec<(usize, u8)>, BareError> {
    let roster = map.ids();
    let all = || roster.iter().copied().enumerate().collect();
    let Some(id) = target else {
        return Ok(all());
    };
    match roster.iter().position(|held| *held == id) {
        Some(row) => Ok(vec![(row, id)]),
        None => Err(BareError::OffRoster { id, roster }),
    }
}

/// Ping every rebooted servo until it answers again, and say how long each took.
///
/// One shared budget over round-robin rounds rather than a budget each: the
/// instruction went out to all of them at one instant, so elapsed time since
/// then is the truthful measurement, and a serial sweep would spend the whole
/// budget on the first servo that is really gone before asking the second.
/// A servo that is restarting answers nothing at all, so every ping before it is
/// back fails on its own deadline; the pause between rounds is the bus's own
/// retry spacing, which is the cadence this configuration carries for exactly
/// this -- asking again in a moment.
///
/// The answers come back in the order the targets were given, each with the time
/// that servo waited, so the report reads as a roster rather than as a race.
fn wait_for_all<P: BusPort>(
    bus: &mut Bus<P>,
    targets: &[(usize, u8)],
    clock: &mut dyn Clock,
) -> Vec<(usize, u8, Result<Duration, BareError>)> {
    let started = clock.now();
    let spacing = bus.timing().retry_spacing;
    let mut answers: Vec<Option<Duration>> = vec![None; targets.len()];
    let mut last_failure: Vec<Option<XactError>> = targets.iter().map(|_| None).collect();
    let mut rounds = 0;

    while answers.iter().any(Option::is_none) {
        for (slot, (_, id)) in targets.iter().enumerate() {
            if answers[slot].is_some() {
                continue;
            }
            match bus.ping(*id) {
                Ok(_) => answers[slot] = Some(clock.now().saturating_sub(started)),
                Err(error) => last_failure[slot] = Some(error),
            }
        }
        rounds += 1;
        if rounds >= BOOT_POLLS || answers.iter().all(Option::is_some) {
            break;
        }
        clock.sleep_until(clock.now() + spacing);
    }

    let waited = clock.now().saturating_sub(started);
    targets
        .iter()
        .enumerate()
        .map(|(slot, (row, id))| {
            let answer = match answers[slot] {
                Some(took) => Ok(took),
                None => Err(BareError::NotBack {
                    id: *id,
                    polls: rounds,
                    waited,
                    // A servo with no answer was pinged in every round, so it
                    // failed in the last one too.
                    source: last_failure[slot]
                        .take()
                        .expect("a servo that never answered failed at least once"),
                }),
            };
            (*row, *id, answer)
        })
        .collect()
}

/// What one servo came back holding.
struct Reading {
    /// The line an operator reads.
    report: String,
    /// Whether it is still holding torque, which a restart clears — so a servo
    /// that answers holding it did not restart.
    holding: bool,
    /// Its hardware-error byte, which a restart clears too.
    bits: u8,
}

/// What a servo holds now: the hardware-error byte, its torque, and where it is
/// standing.
///
/// The error byte is the operator's reason for rebooting at all, and this is
/// the reading that says whether it went. Torque is the restart's own
/// observable: a servo comes back with it cleared, so reading it is how
/// answering is told from restarting. The position is reported as the servo's
/// own count and the angle that count is, unshifted by anything the host knows.
/// The byte is reported as it reads *and* handed back for the caller to judge:
/// this machine clears it to zero on a restart, recorded in the runbook's open
/// observations, so a surviving byte is a reading with an expectation to fail
/// against. The position has no such expectation established, so it is reported
/// and nothing more.
fn reading<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    id: u8,
) -> Result<Reading, BareError> {
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
        bits,
    })
}

/// One servo's present position, as the count it reports.
///
/// Unshifted, which is why this reads the register rather than going through
/// the map's decoding: what a restart left in the servo is the count, and the
/// host's own idea of where zero is would be an interpretation laid over it.
fn read_counts<P: BusPort>(bus: &mut Bus<P>, id: u8) -> Result<i32, BareError> {
    let raw = read_raw_by_id(bus, id, RegId::PresentPosition)?;
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
) -> Result<u8, BareError> {
    let id = map.ids()[row];
    let value = read_value(bus, map, row, reg)?;
    match value.as_u8() {
        Some(byte) => Ok(byte),
        None => Err(BareError::Map {
            id,
            reg,
            source: MapError::WrongShape {
                reg,
                expected: ValueShape::U8,
                observed: value.shape(),
            },
        }),
    }
}

/// One register from one servo, as its engineering value.
///
/// The crate's one register read: `selftest` reads through this too, mapping the
/// error to the string its report carries, so retry handling, the width the map
/// enforces and the text a failure prints have one definition.
pub(crate) fn read_value<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    reg: RegId,
) -> Result<Value, BareError> {
    let raw = read_raw(bus, map, row, reg)?;
    map.decode_value(row, reg, &raw)
        .map_err(|source| BareError::Map {
            id: map.ids()[row],
            reg,
            source,
        })
}

/// One register from one servo, as the bytes it holds.
pub(crate) fn read_raw<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    reg: RegId,
) -> Result<RawValue, BareError> {
    read_raw_by_id(bus, map.ids()[row], reg)
}

/// The same read, addressed by servo ID rather than by row -- for the one caller
/// that has no row to speak of.
fn read_raw_by_id<P: BusPort>(bus: &mut Bus<P>, id: u8, reg: RegId) -> Result<RawValue, BareError> {
    let entry = reg_for(reg).map_err(|source| BareError::Map { id, reg, source })?;
    with_retry(bus, |bus| bus.read_reg(id, entry)).map_err(|source| BareError::BusRead {
        id,
        reg,
        source,
    })
}

#[cfg(test)]
mod tests {
    use std::cell::RefCell;
    use std::rc::Rc;

    use dxl_proto::frame::{INST_REBOOT, INST_WRITE};

    use super::*;
    use crate::config::Resolved;
    use crate::testutil::{
        FakeMachine, Spy, TestClock, datumed_config, machine_at, resolved, stow_legs, wind_down_bus,
    };

    /// What a command left behind: the registers it ended on, every instruction
    /// that crossed the wire, and every line it printed.
    struct Run {
        outcome: Result<(), BareError>,
        registers: Rc<RefCell<FakeMachine>>,
        log: Rc<RefCell<Vec<(u8, u8)>>>,
        printed: Vec<String>,
        /// Every deadline the command asked its clock to wait for, in order.
        waits: Vec<Duration>,
    }

    impl Run {
        /// The command succeeded, or this says which one did not and why.
        fn ok(&self, what: &str) {
            if let Err(error) = &self.outcome {
                panic!("{what}: {error}");
            }
        }

        /// The command refused, and this is the refusal.
        fn err(&self, what: &str) -> &BareError {
            match &self.outcome {
                Ok(()) => panic!("{what}"),
                Err(error) => error,
            }
        }

        /// Every servo's goal register is untouched: nothing here pinned or
        /// commanded anything.
        fn commanded_nothing(&self, cfg: &Resolved) {
            let machine = self.registers.borrow();
            for id in cfg.map.ids() {
                assert!(
                    machine.get(id, named_reg(RegId::GoalPosition)).is_none(),
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
        F: FnOnce(Spy, &mut dyn Clock, &mut dyn FnMut(&str)) -> Result<(), BareError>,
    {
        let spy = Spy::new(machine);
        let registers = spy.machine();
        let log = spy.log();
        let mut clock = TestClock::default();
        let mut printed = Vec::new();
        let outcome = command(spy, &mut clock, &mut |line| printed.push(line.to_string()));
        Run {
            outcome,
            registers,
            log,
            printed,
            waits: clock.waits,
        }
    }

    /// What each servo's Torque Enable register holds after a run.
    fn torque(cfg: &Resolved, run: &Run) -> Vec<u8> {
        let machine = run.registers.borrow();
        cfg.map
            .ids()
            .iter()
            .map(|id| {
                machine
                    .get(*id, named_reg(RegId::TorqueEnable))
                    .map_or(0, |bytes| bytes[0])
            })
            .collect()
    }

    /// `off` writes torque off on all nine and commands nothing.
    #[test]
    fn off_releases_every_servo_on_the_roster() {
        let cfg = resolved();
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        for id in cfg.map.ids() {
            machine.set(id, named_reg(RegId::TorqueEnable), &[1]);
        }
        let run = run(machine, |port, _, line| {
            off(&cfg.map, cfg.timing, port, line)
        });
        run.ok("a machine that answers releases");

        assert_eq!(torque(&cfg, &run), vec![0; ROW_COUNT]);
        run.armed_nothing();
        run.commanded_nothing(&cfg);
        assert!(
            run.printed.iter().any(|line| line.contains("released;")),
            "{:?}",
            run.printed
        );
    }

    /// A machine already limp is released again without complaint: `off` reads
    /// nothing and judges nothing, so where the machine stands gates none of it.
    #[test]
    fn off_releases_a_machine_that_is_already_limp() {
        let cfg = resolved();
        let run = run(
            machine_at(&datumed_config(), &stow_legs()),
            |port, _, line| off(&cfg.map, cfg.timing, port, line),
        );
        run.ok("a limp machine releases");
        assert_eq!(torque(&cfg, &run), vec![0; ROW_COUNT]);
    }

    /// A servo that will not answer its torque-off does not stop the sweep: the
    /// eight after it are written anyway, and the run reports the one that may
    /// still be holding.
    ///
    /// The safety property of this command. A release that stopped at the first
    /// silent servo would leave the rest of the machine holding the head up.
    #[test]
    fn a_silent_servo_does_not_stop_the_release_of_the_others() {
        let cfg = resolved();
        let silent = cfg.map.ids()[2];
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        for id in cfg.map.ids() {
            machine.set(id, named_reg(RegId::TorqueEnable), &[1]);
        }
        machine.silent = vec![silent];
        let run = run(machine, |port, _, line| {
            off(&cfg.map, cfg.timing, port, line)
        });

        let error = run.err("a servo that answered nothing is not a complete release");
        let BareError::TorqueOffUnacked { id } = error else {
            panic!("expected an unacknowledged release, got {error}");
        };
        assert_eq!(*id, silent);
        for (row, id) in cfg.map.ids().iter().copied().enumerate() {
            let held = torque(&cfg, &run)[row];
            if id == silent {
                assert_eq!(held, 1, "servo {id} answers nothing, so it kept its torque");
            } else {
                assert_eq!(
                    held, 0,
                    "servo {id} was released whatever its neighbour did"
                );
            }
        }
        assert!(
            !run.printed.iter().any(|line| line.contains("released;")),
            "an incomplete release does not claim one: {:?}",
            run.printed
        );
    }
    /// A machine as the vendor provisions it: every servo in single-turn
    /// position mode, torque off.
    fn unprovisioned(cfg: &Resolved) -> FakeMachine {
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        for id in [cfg.map.ids()[7], cfg.map.ids()[8]] {
            machine.set(id, named_reg(RegId::OperatingMode), &[3]);
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
                    .get(*id, named_reg(RegId::OperatingMode))
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
            vec![0; ROW_COUNT],
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
        machine.set(cfg.map.ids()[8], named_reg(RegId::TorqueEnable), &[1]);
        let run = run(machine, |port, _, line| {
            provision(&cfg.map, cfg.timing, port, line)
        });

        let error = run.err("a torqued antenna is refused");
        let BareError::TorqueHeld { id } = error else {
            panic!("expected a torque refusal, got {error}");
        };
        assert_eq!(*id, cfg.map.ids()[8]);
        assert_eq!(
            modes(&cfg, &run),
            vec![3; ROW_COUNT],
            "the first antenna was not written either"
        );
    }

    /// A servo answering as another part is refused before anything is written:
    /// whatever holds that ID is not the servo this project provisions.
    #[test]
    fn provision_refuses_a_servo_that_is_not_the_part_it_should_be() {
        let cfg = resolved();
        let mut machine = unprovisioned(&cfg);
        machine.set(
            cfg.map.ids()[7],
            named_reg(RegId::ModelNumber),
            &[0xB0, 0x04],
        );
        let run = run(machine, |port, _, line| {
            provision(&cfg.map, cfg.timing, port, line)
        });

        let error = run.err("that is not an antenna servo");
        let BareError::WrongPart {
            id,
            model,
            expected,
        } = error
        else {
            panic!("expected an identity refusal, got {error}");
        };
        assert_eq!((*id, *model, *expected), (cfg.map.ids()[7], 1200, 1190));
        assert_eq!(modes(&cfg, &run), vec![3; ROW_COUNT]);
    }

    /// A machine holding torque on all nine, with one servo carrying a latched
    /// overload — the state an operator reaches for `reboot` in.
    fn overloaded(cfg: &Resolved) -> FakeMachine {
        let mut machine = machine_at(&datumed_config(), &stow_legs());
        for id in cfg.map.ids() {
            machine.set(id, named_reg(RegId::TorqueEnable), &[1]);
        }
        machine.set(
            cfg.map.ids()[3],
            named_reg(RegId::HardwareErrorStatus),
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
            vec![0; ROW_COUNT],
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
        // The latch the operator came for is gone, and that is read back rather
        // than assumed: nothing here writes the byte, so a servo reporting zero
        // is a servo that restarted.
        for id in cfg.map.ids() {
            let reading = run
                .printed
                .iter()
                .find(|line| line.contains(&format!("servo {id}: hardware error")))
                .unwrap_or_else(|| panic!("servo {id} was not reported: {:?}", run.printed));
            assert!(
                reading.contains("0x00") && reading.contains("clear"),
                "{reading}"
            );
        }
        assert!(
            !run.printed
                .iter()
                .any(|line| line.contains("still latched")),
            "{:?}",
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
        let BareError::NotRestarted { id } = error else {
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
        machine.set(deaf, named_reg(RegId::TorqueEnable), &[0]);
        machine.deaf_to_reboot.push(deaf);
        let run = run(machine, |port, clock, line| {
            reboot(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("an unobserved restart is not a restart");
        let BareError::RestartUnconfirmed { id, .. } = error else {
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

    /// A servo whose only surviving bit is input voltage is reported as that,
    /// and one that answers its ping but fails a register read is named without
    /// stopping the other eight being read.
    ///
    /// The voltage rendering is what tells an operator which bit they are
    /// looking at; the read-error arm is the path a bus failure takes in the
    /// middle of a report. The surviving byte is trouble of its own — the test
    /// below pins that — and here it is a servo further down the bus than the
    /// one that would not read, so what comes back is the first verdict in bus
    /// order and not the last.
    #[test]
    fn a_reboot_reports_a_voltage_only_byte_and_a_servo_that_will_not_read() {
        let cfg = resolved();
        let voltage = cfg.map.ids()[7];
        let unreadable = cfg.map.ids()[5];
        let mut machine = overloaded(&cfg);
        machine.set(
            voltage,
            named_reg(RegId::HardwareErrorStatus),
            &[dxl_proto::conv::HW_INPUT_VOLTAGE],
        );
        machine.keeps_latch.push(voltage);
        // Answers its ping, answers nothing about where it is standing.
        machine.mute.insert(
            (unreadable, named_reg(RegId::PresentPosition).addr),
            u32::MAX,
        );
        let run = run(machine, |port, clock, line| {
            reboot(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a servo that will not read is not a clean reboot");
        let BareError::BusRead { id, reg, .. } = error else {
            panic!("expected a read failure, got {error}");
        };
        assert_eq!((*id, *reg), (unreadable, RegId::PresentPosition));
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

    /// A hardware-error byte that survives its reboot fails the command, and
    /// the chronic input-voltage bit is no exception.
    ///
    /// The one thing this command exists to do. A restart clears the byte on
    /// this machine, so a servo answering with bits still set either never
    /// restarted or is reporting a condition live at this instant — and an
    /// operator scripting a recovery around the exit code must not be told a
    /// reboot that cleared nothing succeeded. The voltage bit gets no carve-out:
    /// it is an unexplained reading on a machine whose other readings are
    /// trusted, and coding "probably fine" into the recovery path would settle
    /// that question by assertion rather than by measuring the rail.
    #[test]
    fn a_latch_that_survives_its_reboot_fails_the_command() {
        let cfg = resolved();
        for bits in [0x20, dxl_proto::conv::HW_INPUT_VOLTAGE] {
            let stuck = cfg.map.ids()[4];
            let mut machine = overloaded(&cfg);
            machine.set(stuck, named_reg(RegId::HardwareErrorStatus), &[bits]);
            machine.keeps_latch.push(stuck);
            let run = run(machine, |port, clock, line| {
                reboot(&cfg.map, cfg.timing, port, None, clock, line)
            });

            let error = run.err("a reboot that cleared nothing did not recover");
            let BareError::StillLatched { id, bits: held } = error else {
                panic!("expected a surviving latch, got {error}");
            };
            assert_eq!((*id, *held), (stuck, bits));
            assert!(
                run.printed
                    .iter()
                    .any(|line| line.contains(&format!("servo {stuck}"))
                        && line.contains("still set after a restart that took")),
                "{:?}",
                run.printed
            );
            assert!(
                !run.printed.iter().any(|line| line.contains("rebooted;")),
                "{:?}",
                run.printed
            );
            // It restarted, so the torque did come off — the byte is the whole
            // of the complaint, and the other eight still pass.
            assert_eq!(torque(&cfg, &run), vec![0; ROW_COUNT], "{:?}", run.printed);
            for id in cfg.map.ids().iter().filter(|id| **id != stuck) {
                assert!(
                    run.printed
                        .iter()
                        .any(|line| line.contains(&format!("servo {id}: hardware error"))
                            && line.contains("clear")),
                    "servo {id} went unreported: {:?}",
                    run.printed
                );
            }
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
        let BareError::OffRoster { id, roster } = error else {
            panic!("expected a roster refusal, got {error}");
        };
        assert_eq!((*id, *roster), (stranger, cfg.map.ids()));
        assert!(run.log.borrow().is_empty(), "{:?}", run.log.borrow());
        assert_eq!(
            torque(&cfg, &run),
            vec![1; ROW_COUNT],
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
        let BareError::NotBack { id, polls, .. } = error else {
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
    /// Two servos gone cost one budget between them, not one each.
    ///
    /// The reboots went out at the same instant, so the wait is a sweep over
    /// whoever is still missing and the elapsed figure both refusals carry is the
    /// same one. A serial wait would have spent the whole budget on the first and
    /// reported the second at twice the elapsed time.
    #[test]
    fn two_servos_gone_share_one_wait_budget() {
        let cfg = resolved();
        let lost = [cfg.map.ids()[2], cfg.map.ids()[6]];
        let mut machine = overloaded(&cfg);
        machine.gone_on_reboot.extend_from_slice(&lost);
        let run = run(machine, |port, clock, line| {
            reboot(&cfg.map, cfg.timing, port, None, clock, line)
        });

        run.err("two servos that never answered are not a success");
        let elapsed: Vec<String> = lost
            .iter()
            .map(|id| {
                let line = run
                    .printed
                    .iter()
                    .find(|line| line.contains(&format!("servo {id}: NO ANSWER")))
                    .unwrap_or_else(|| panic!("servo {id} went unreported: {:?}", run.printed));
                let (_, tail) = line.split_once(" over ").expect("the elapsed figure");
                let (waited, _) = tail.split_once(" after").expect("the elapsed figure");
                waited.to_string()
            })
            .collect();
        assert_eq!(elapsed[0], elapsed[1], "{:?}", run.printed);
    }
    /// A servo that comes back partway through the sweep is reported back, with
    /// the round it answered on in its elapsed figure.
    ///
    /// The middle of the polling loop: a restart takes a moment, so the case
    /// between "answers at once" and "never answers" is the ordinary one. It is
    /// what pins the round accounting, the cadence (one wait per round rather
    /// than one per ping) and the epoch the elapsed figure is measured from. The
    /// spacing is wound back up for this one test -- the shared fixture runs at
    /// zero, which would make every elapsed figure zero whatever the loop did --
    /// and the clock is the test's, so the quarter-second costs nothing.
    #[test]
    fn servos_that_come_back_on_later_rounds_are_reported_with_what_they_waited() {
        let mut file = datumed_config();
        wind_down_bus(&mut file);
        file.bus.retry_spacing_ms = 250;
        let cfg = file.resolve().expect("a datumed example resolves");
        let spacing = cfg.timing.retry_spacing;

        let (early, late) = (cfg.map.ids()[1], cfg.map.ids()[5]);
        let mut machine = overloaded(&cfg);
        machine.pings_ignored.insert(early, 1);
        machine.pings_ignored.insert(late, 3);
        let run = run(machine, |port, clock, line| {
            reboot(&cfg.map, cfg.timing, port, None, clock, line)
        });
        run.ok("servos that came back late still reboot");

        let waited = |id: u8| -> f64 {
            let line = run
                .printed
                .iter()
                .find(|line| line.contains(&format!("servo {id}: answering")))
                .unwrap_or_else(|| panic!("servo {id} went unreported: {:?}", run.printed));
            let (_, tail) = line.split_once("answering ").expect("the elapsed figure");
            let (figure, _) = tail.split_once(" s").expect("the elapsed figure");
            figure.parse().expect("the elapsed figure is a number")
        };
        // Measured from the instant the instruction went out, so a servo that
        // answered on round k reads the k-1 rounds it missed: the ones back at
        // once waited nothing, and the two that were not read the rounds they
        // took, in order.
        assert_eq!(waited(cfg.map.ids()[0]), 0.0, "{:?}", run.printed);
        assert_eq!(waited(early), spacing.as_secs_f64(), "{:?}", run.printed);
        assert_eq!(
            waited(late),
            3.0 * spacing.as_secs_f64(),
            "{:?}",
            run.printed
        );

        // Four rounds ran, the last being the one `late` answered on, and each
        // spaced the next by one retry spacing -- one wait per round, not one
        // per ping.
        assert_eq!(run.waits.len(), 3, "{:?}", run.waits);
        for (round, until) in run.waits.iter().enumerate() {
            let expected = spacing * u32::try_from(round + 1).expect("a small round count");
            assert_eq!(*until, expected, "round {round}");
        }
    }

    /// An antenna that answers nothing is a bus failure named by ID, and neither
    /// antenna is written.
    #[test]
    fn provision_refuses_an_antenna_that_answers_nothing() {
        let cfg = resolved();
        let mut machine = unprovisioned(&cfg);
        machine.silent = vec![cfg.map.ids()[8]];
        let run = run(machine, |port, _, line| {
            provision(&cfg.map, cfg.timing, port, line)
        });

        let error = run.err("an antenna that answers nothing cannot be provisioned");
        let BareError::Bus { id, .. } = error else {
            panic!("expected a bus failure, got {error}");
        };
        assert_eq!(*id, cfg.map.ids()[8]);
        assert_eq!(
            modes(&cfg, &run),
            vec![3; ROW_COUNT],
            "the antenna that did answer was not written either",
        );
    }

    /// An antenna that reads back its old mode after the non-volatile write is a
    /// bus failure, and the machine is left as it stood.
    ///
    /// The verified write is what makes provisioning a claim rather than a hope:
    /// a servo that acknowledges the write and stores nothing is exactly the
    /// EEPROM refusal the read-back exists to catch.
    #[test]
    fn provision_refuses_an_antenna_whose_eeprom_write_does_not_stick() {
        let cfg = resolved();
        let first = cfg.map.ids()[7];
        let mut machine = unprovisioned(&cfg);
        machine
            .ignored
            .push((first, named_reg(RegId::OperatingMode).addr));
        let run = run(machine, |port, _, line| {
            provision(&cfg.map, cfg.timing, port, line)
        });

        let error = run.err("a write that did not stick is not a provisioning");
        let BareError::Bus { id, .. } = error else {
            panic!("expected a bus failure, got {error}");
        };
        assert_eq!(*id, first);
        assert_eq!(
            modes(&cfg, &run),
            vec![3; ROW_COUNT],
            "nothing was left written",
        );
        assert!(
            !run.printed.iter().any(|line| line.contains("provisioned;")),
            "{:?}",
            run.printed
        );
    }

    /// The real clock's two promises: time does not go backwards, and a deadline
    /// already past returns at once.
    ///
    /// `wait_for_all` subtracts one from the other, so a clock that went
    /// backwards or slept on a past deadline would be an underflow or a hang on
    /// hardware and nowhere else.
    #[test]
    fn the_monotonic_clock_does_not_go_backwards_or_sleep_on_a_past_deadline() {
        let mut clock = MonotonicClock::new();
        let first = clock.now();
        clock.sleep_until(first + Duration::from_millis(2));
        let second = clock.now();
        assert!(second >= first, "{second:?} came before {first:?}");

        let before = Instant::now();
        clock.sleep_until(Duration::ZERO);
        assert!(
            before.elapsed() < Duration::from_millis(500),
            "a deadline already past returned at once",
        );
        assert!(clock.now() >= second);
    }
}
