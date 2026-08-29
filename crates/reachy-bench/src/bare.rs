//! The bench's bare-bus commands: the ones that need no tick, no sequencer and
//! no kinematics.
//!
//! Each one opens with a bus and a roster and nothing else. `provision` writes
//! the antennas' operating mode, `reboot` restarts the servos, `off` sweeps
//! torque off, and `watchdog` establishes what an armed Bus Watchdog does to one
//! servo. None of them commands an angle — the goal `watchdog` writes is the
//! count the servo reports for itself — so none of them needs an envelope, a
//! pose or a control loop; what they share is the register-level plumbing at the
//! bottom of this file.
//!
//! `watchdog` is the one that torques a servo, and it is the odd one here for
//! that reason: it is a supervised bring-up assertion about a register the
//! session path arms on every engagement, and its whole observation is a servo
//! letting go on its own.
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

use dxl_proto::{HardwareError, StatusCode, StatusError, counts_to_rad};
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

/// The joint the watchdog self-test addresses unless the operator names another:
/// an antenna, the servo whose going limp costs the least. Going limp is the
/// whole observation, so the joint is chosen for what it drops.
const WATCHDOG_JOINT: JointRef = JointRef::AntennaRight;

/// One count of the Bus Watchdog register, in milliseconds.
const WATCHDOG_UNIT_MS: u64 = 20;

/// The Bus Watchdog value the test arms, in the register's own units — the same
/// value a session arms, so what this establishes is what the machine runs.
///
/// Visible to the crate because the read-only sweep accepts the same figure as
/// a register's second rest state, and the two are asserted equal rather than
/// restated (`config.rs`).
pub(crate) const WATCHDOG_COUNTS: u8 = 10;

/// The silence an armed servo is meant to tolerate: the counts times the unit.
const WATCHDOG_TIMEOUT: Duration = Duration::from_millis(WATCHDOG_UNIT_MS * WATCHDOG_COUNTS as u64);

/// What the register reads once the watchdog has tripped: the vendor's -1 in
/// the byte the register is.
pub(crate) const WATCHDOG_LATCHED: u8 = 0xFF;

/// Timeouts' worth of traffic each no-trip phase keeps up. Five, so a phase
/// that passes has spent most of its length past the point a servo that ignored
/// its traffic would have tripped.
const WATCHDOG_BUSY_TIMEOUTS: u32 = 5;

/// Timeouts' worth of silence the trip is provoked with. Two, so a trip that
/// does not happen had a whole timeout of slack beyond its own.
const WATCHDOG_SILENT_TIMEOUTS: u32 = 2;

/// The cadence traffic goes out at while a no-trip phase runs: the driver's own
/// command period, which is also one register count.
const WATCHDOG_TRAFFIC_PERIOD: Duration = Duration::from_millis(WATCHDOG_UNIT_MS);

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

    /// The servo a watchdog self-test addresses was already holding torque. The
    /// test torques it itself and then watches it let go, so a servo that
    /// arrived holding is standing somewhere this test did not put it — a
    /// session's pose, or a hold left behind — and the reading would be about
    /// that instead.
    #[error("servo {id} is holding torque; release it with `off` before the watchdog self-test")]
    WatchdogTorqueHeld {
        /// The servo addressed.
        id: u8,
    },

    /// A limp servo's Goal Position register does not read as its Present
    /// Position, so this platform's servos do not track their goal while
    /// released. The test torques a servo without writing it a goal first, on
    /// exactly that property; a machine where it does not hold would be
    /// commanded to wherever the stale goal points the moment torque went on.
    #[error(
        "servo {id} reads goal {goal} and position {at} with torque off: a released servo here \
         does not track its own goal, so enabling torque would command it to the older one"
    )]
    WatchdogGoalNotTracking {
        /// The servo addressed.
        id: u8,
        /// What its goal register read.
        goal: i32,
        /// Where it was standing.
        at: i32,
    },

    /// The register would not take the value: written, acknowledged, and read
    /// back as something else. Nothing that follows would mean anything, so the
    /// test stops here.
    #[error(
        "servo {id} bus watchdog reads {read} after being armed at {armed}, so the register did \
         not take the value"
    )]
    WatchdogNotArmed {
        /// The servo addressed.
        id: u8,
        /// The value written.
        armed: u8,
        /// What the register read afterwards.
        read: u8,
    },

    /// The watchdog tripped while the bus was busy, so the traffic this machine
    /// runs on does not reset the count the way the arming policy assumes.
    #[error(
        "servo {id} bus watchdog reads {read:#04x} after {busy:?} of {phase} at one exchange every \
         {period:?}, where an armed one reads {armed}: that traffic did not reset it"
    )]
    WatchdogTrippedEarly {
        /// The servo addressed.
        id: u8,
        /// The traffic that was running.
        phase: &'static str,
        /// What the register read.
        read: u8,
        /// The value it was armed at.
        armed: u8,
        /// How long the traffic ran.
        busy: Duration,
        /// The gap between exchanges.
        period: Duration,
    },

    /// The servo stopped holding while the bus was busy. Whatever released it,
    /// it was not the silence this test is about.
    #[error(
        "servo {id} stopped holding torque during {phase}, with the bus busy and its watchdog \
         still armed"
    )]
    WatchdogReleasedEarly {
        /// The servo addressed.
        id: u8,
        /// The traffic that was running.
        phase: &'static str,
    },

    /// Silence did not trip the watchdog. The arming write lands and reads back,
    /// and it protects nothing: a driver that dies leaves this servo holding.
    #[error(
        "servo {id} bus watchdog reads {read:#04x} after {silent:?} of silence with torque held, \
         where a trip reads {latched:#04x}"
    )]
    WatchdogNeverTripped {
        /// The servo addressed.
        id: u8,
        /// What the register read.
        read: u8,
        /// How long nothing was sent.
        silent: Duration,
        /// What a trip reads as.
        latched: u8,
    },

    /// The watchdog tripped and the servo is still holding its goal — the
    /// mechanism fires and does not de-torque, which is not the backstop the
    /// fault doctrine is relying on it to be.
    ///
    /// The vendor's manual predicts exactly this: it describes a trip as a stop
    /// under torque rather than a release. So this verdict is a policy-level
    /// finding for a human, not an assertion to relax until it passes.
    #[error(
        "servo {id} bus watchdog tripped and the servo is still holding torque, so a tripped \
         watchdog does not release this machine. Stop: this is a policy-level finding about what \
         the armed watchdog is a backstop for, not a test to make green"
    )]
    WatchdogStillHolding {
        /// The servo addressed.
        id: u8,
    },

    /// A tripped watchdog took a goal write instead of refusing it, so nothing
    /// on the wire distinguishes a tripped servo from a live one.
    #[error(
        "servo {id} accepted a goal write with its bus watchdog tripped, where a trip refuses one"
    )]
    WatchdogGoalAccepted {
        /// The servo addressed.
        id: u8,
    },

    /// A tripped watchdog refused a goal write with some other error number.
    /// Surfaced verbatim rather than read as close enough: the byte is the only
    /// signature a host has for this state.
    #[error(
        "servo {id} refused a goal write with its bus watchdog tripped, but with error field \
         {:#04x} ({:?}) rather than Access", .error.0, .error.code()
    )]
    WatchdogRefusedOtherwise {
        /// The servo addressed.
        id: u8,
        /// The error field, whole.
        error: StatusError,
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

/// Establish what an armed Bus Watchdog does on this machine, on one servo.
///
/// The register is armed on every session (`crates/reachy-motion`'s
/// commissioning sweep writes it), and this is what watches it do its job. That
/// ordinary bus traffic resets the count is vendor documentation nobody here
/// has watched happen; what a servo whose count ran out does with the torque it
/// is holding has been watched, on this unit, and the answer is that it stops
/// and keeps holding. So stage 5 below asserts a release the fault policy
/// requires and this hardware does not perform: the failure is the standing
/// record of that, not a test to make green.
///
/// Supervised, on one servo, at whatever pose the machine is resting in. The
/// sequence torques the servo, keeps the bus busy, then goes quiet and reads
/// what happened:
///
/// 1. arming reads back,
/// 2. a released servo reads its goal as its present position,
/// 3. a busy bus with goal rewrites does not trip it,
/// 4. a busy bus of reads alone does not trip it either,
/// 5. silence trips it: the register reads the trip marker, the torque reading
///    is taken and reported, and a goal rewrite is answered. All three are read
///    and printed before any of them is judged; the assertions are then that
///    the register latched, that the servo let go (the expectation the fault
///    policy requires, and a failure here is a finding for a human), and that
///    the refusal carries the Access code this hardware answers with,
/// 6. writing zero clears the trip, re-arming takes, and torque can be enabled
///    again,
/// 7. the servo is left disarmed and limp.
///
/// Every write lives in the exercise half, and the make-safe half — a watchdog
/// clear and a torque-off, nothing else — runs whatever the exercise said.
///
/// It commands no angle: the only goal ever written is the count the servo just
/// reported for itself, which is a rewrite of where it already stands -- written
/// before each torque-enable, so the hold is where the servo is standing at that
/// instant whatever its goal register held. Torque
/// it does take, and the make-safe releases it whichever way the run ends —
/// which is why it addresses an antenna by default and refuses to start on a
/// servo that is already holding something.
pub fn watchdog<P: BusPort>(
    map: &ServoMap,
    timing: BusTiming,
    port: P,
    target: Option<u8>,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), BareError> {
    let (row, id) = watchdog_target(map, target)?;
    let mut bus = Bus::new(port, timing);

    line(&format!(
        "watchdog: servo {id} is armed with a {timeout:?} bus watchdog, torqued at the position \
         it already holds, and then left in silence to be read. It is commanded nowhere — the \
         only goal written is the count it reports for itself — but it does hold torque for a \
         few seconds, and on this hardware the trip stops it without releasing it: this \
         command's own make-safe is what releases the torque on the way out. Stay clear of it, \
         and do not run this with the head up.",
        timeout = WATCHDOG_TIMEOUT,
    ));

    let info =
        with_retry(&mut bus, |bus| bus.ping(id)).map_err(|source| BareError::Bus { id, source })?;
    if read_byte(&mut bus, map, row, RegId::TorqueEnable)? != 0 {
        return Err(BareError::WatchdogTorqueHeld { id });
    }
    line(&format!(
        "  servo {id}: model {model}, torque off",
        model = info.model
    ));

    let outcome = watchdog_exercise(&mut bus, map, row, clock, line);

    // The make-safe runs whatever the assertions said. A servo left armed and
    // torqued by an early return is a servo whose next silence is a trip
    // nobody is watching, and one left disarmed and limp is the machine's
    // resting state.
    let cleared = watchdog_make_safe(&mut bus, map, row, line);
    if let Err(failed) = outcome {
        // Both halves are said, and the cleanup's failure is said first: an
        // assertion that came back wrong is a discovery, while a cleanup that
        // could not finish may be a servo still holding torque, which is the one
        // state an operator has to act on before reading anything else.
        if let Err(release) = cleared {
            line(&format!(
                "  cleanup did not finish, so the torque state of this servo is unknown: {release}"
            ));
        }
        return Err(failed);
    }
    cleared?;
    line(
        "watchdog: the register resets the count on traffic, trips on silence, releases the servo \
         when it trips, and clears on a zero. Disarmed and limp now.",
    );
    Ok(())
}

/// Every write the watchdog self-test makes, and every assertion it draws from
/// them, in one place whose error the caller captures rather than propagates.
///
/// Nothing here returns past the caller's make-safe: a phase that arms the
/// register or enables torque and then fails leaves the servo in a state only
/// the make-safe half puts right, so all of them live inside this boundary.
/// The chain short-circuits on the first failing phase.
fn watchdog_exercise<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), BareError> {
    let id = map.ids()[row];

    // Clear then arm, which is the pair a session writes: a tripped watchdog
    // refuses ordinary writes, and zero is the vendor's documented clear,
    // accepted even while latched. On a servo that is not tripped the clear is
    // a write of the value the register already resets to.
    write_byte(bus, map, row, RegId::BusWatchdog, 0)?;
    write_byte(bus, map, row, RegId::BusWatchdog, WATCHDOG_COUNTS)?;
    // A read of its own, after a write path that already read the register back:
    // the write's read-back says the value went in, and this says the register
    // still holds it a transaction later. A servo that takes the value and then
    // changes it — the shape a trip already latched would have — is caught here
    // and nowhere else.
    let armed = read_byte(bus, map, row, RegId::BusWatchdog)?;
    if armed != WATCHDOG_COUNTS {
        return Err(BareError::WatchdogNotArmed {
            id,
            armed: WATCHDOG_COUNTS,
            read: armed,
        });
    }
    line(&format!(
        "  armed: bus watchdog {armed} counts, {WATCHDOG_TIMEOUT:?}, read back"
    ));

    // That a released servo reads its goal as its position, asserted rather
    // than assumed: the whole claim that this test commands nothing rests on
    // it, and it is a hardware fact of the same kind as the ones below.
    watchdog_tracks_its_goal(bus, map, row, line)?;

    // Torque on at where the servo stands, with the goal rewritten to that
    // position first. The reading above says the rewrite is a no-op on this
    // platform, and it is written anyway: a servo whose goal register held
    // something else would otherwise be commanded there by the write below.
    hold_where_it_stands(bus, map, row)?;
    line("  holding: torque on at the position it was resting at");

    watchdog_busy(bus, map, row, Busy::ReadsAndGoals, clock, line)?;
    watchdog_busy(bus, map, row, Busy::ReadsOnly, clock, line)?;
    watchdog_silence(bus, map, row, clock, line)?;
    watchdog_rearms(bus, map, row, line)
}

/// The servo a watchdog self-test addresses: the one asked for, or an antenna.
fn watchdog_target(map: &ServoMap, target: Option<u8>) -> Result<(usize, u8), BareError> {
    let roster = map.ids();
    let Some(id) = target else {
        let row = row(WATCHDOG_JOINT).expect("a named joint has a bus row");
        return Ok((row, roster[row]));
    };
    match roster.iter().position(|held| *held == id) {
        Some(row) => Ok((row, id)),
        None => Err(BareError::OffRoster { id, roster }),
    }
}

/// The traffic one no-trip phase keeps on the bus.
#[derive(Clone, Copy)]
enum Busy {
    /// What the driver does while it holds a pose: read where the servo is,
    /// write the goal again.
    ReadsAndGoals,
    /// Reads and nothing else — the case the vendor's documentation implies and
    /// nobody here has watched, and the one a contingency would hang off.
    ReadsOnly,
}

impl Busy {
    /// What an operator reads this phase as.
    fn name(self) -> &'static str {
        match self {
            Self::ReadsAndGoals => "reads and goal rewrites",
            Self::ReadsOnly => "reads alone",
        }
    }
}

/// Keep the bus busy for the observation window, then assert the watchdog did
/// not trip and the servo did not let go.
///
/// The window is several timeouts long, so a servo that ignored this traffic
/// entirely would have tripped several times over by the end of it.
fn watchdog_busy<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    kind: Busy,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), BareError> {
    let id = map.ids()[row];
    let phase = kind.name();
    let busy = WATCHDOG_TIMEOUT * WATCHDOG_BUSY_TIMEOUTS;
    let until = clock.now() + busy;
    let mut exchanges = 0u32;

    while clock.now() < until {
        let at = read_raw(bus, map, row, RegId::PresentPosition)?;
        exchanges += 1;
        if matches!(kind, Busy::ReadsAndGoals) {
            write_verified(bus, map, row, RegId::GoalPosition, &at)?;
            // A verified write is two exchanges: the write and the read-back.
            exchanges += 2;
        }
        clock.sleep_until(clock.now() + WATCHDOG_TRAFFIC_PERIOD);
    }

    let read = read_byte(bus, map, row, RegId::BusWatchdog)?;
    if read != WATCHDOG_COUNTS {
        return Err(BareError::WatchdogTrippedEarly {
            id,
            phase,
            read,
            armed: WATCHDOG_COUNTS,
            busy,
            period: WATCHDOG_TRAFFIC_PERIOD,
        });
    }
    if read_byte(bus, map, row, RegId::TorqueEnable)? == 0 {
        return Err(BareError::WatchdogReleasedEarly { id, phase });
    }
    line(&format!(
        "  {phase}: {exchanges} exchanges over {busy:?} at one every {WATCHDOG_TRAFFIC_PERIOD:?}, \
         watchdog still armed at {read} and the servo still holding"
    ));
    Ok(())
}

/// What a goal rewrite came back with, recorded before anything judges it.
enum Probe {
    /// The servo took the write.
    Accepted,
    /// The servo refused it, with this error field whole.
    Refused(StatusError),
}

/// Go quiet with torque held, take every reading of the resulting state, print
/// each one, and only then judge them.
///
/// The three readings are one state, and the order they are *taken* in is not
/// the order they are judged in. Taking comes first and completely: the
/// register, the torque, and the answer to a goal rewrite are all on the
/// transcript before any of them can end the run — an early verdict on the
/// refusal byte would finish the run without ever reading whether the trip
/// released torque, which is the one reading the fault doctrine needs.
fn watchdog_silence<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<(), BareError> {
    let id = map.ids()[row];
    let silent = WATCHDOG_TIMEOUT * WATCHDOG_SILENT_TIMEOUTS;
    line(&format!(
        "  silence: nothing goes out for {silent:?} with torque held"
    ));
    clock.sleep_until(clock.now() + silent);

    let read = read_byte(bus, map, row, RegId::BusWatchdog)?;
    line(&format!(
        "  register: bus watchdog reads {read:#04x} after {silent:?} of silence, where a trip \
         reads {WATCHDOG_LATCHED:#04x}"
    ));

    // The one the fault doctrine is relying on, taken before the goal probe:
    // a run that ends without it has established nothing about what an armed
    // watchdog is a backstop for.
    let torque = read_byte(bus, map, row, RegId::TorqueEnable)?;
    let held = torque != 0;
    line(&format!(
        "  torque after the trip: enable reads {torque} — {}",
        if held { "HELD" } else { "released" }
    ));

    // The goal it is standing at, written back to it — a rewrite of where it
    // already is, which a servo that is not tripped takes without moving. A
    // bus-level answer here is a failed observation rather than a judged one and
    // returns as itself; the two readings above are already on the transcript.
    let at = read_raw(bus, map, row, RegId::PresentPosition)?;
    let probe = match write_verified(bus, map, row, RegId::GoalPosition, &at) {
        Ok(()) => {
            line("  probe: a goal rewrite was accepted");
            Probe::Accepted
        }
        Err(BareError::Bus {
            source: XactError::ServoError { error, .. },
            ..
        }) => {
            line(&format!(
                "  probe: a goal rewrite comes back with error field {:#04x} ({:?})",
                error.0,
                error.code()
            ));
            Probe::Refused(error)
        }
        Err(other) => return Err(other),
    };

    // Judgments, now that every reading is taken and printed. The order is by
    // consequence: a watchdog that never tripped makes the other two readings
    // meaningless (a torqued servo under a live hold is correct), and after
    // that the torque outranks the refusal byte.
    if read != WATCHDOG_LATCHED {
        return Err(BareError::WatchdogNeverTripped {
            id,
            read,
            silent,
            latched: WATCHDOG_LATCHED,
        });
    }
    if held {
        return Err(BareError::WatchdogStillHolding { id });
    }
    match probe {
        Probe::Accepted => return Err(BareError::WatchdogGoalAccepted { id }),
        // Observed on this hardware: a latched watchdog answers a goal write
        // with Access. The vendor's manual contradicts itself here — its worked
        // example says Data Range, its "the goal registers become read-only"
        // prose implies Access — so this is the reading, not the document. The
        // comparison is on the masked code because this unit's standing
        // input-voltage latch sets the alert bit on every status packet; the
        // byte itself is carried verbatim into the error either way.
        Probe::Refused(error) if error.code() != Some(StatusCode::Access) => {
            return Err(BareError::WatchdogRefusedOtherwise { id, error });
        }
        Probe::Refused(_) => {}
    }
    line("  released: torque enable reads off, so the trip let the servo go");
    Ok(())
}

/// Clear the trip and prove the servo arms and takes torque again.
///
/// The last exercise phase, not part of the make-safe: the clear is the same
/// zero the arming pair opens with and it is what says a trip is recoverable at
/// all, and the torque write after it is the other half — a servo that clears
/// its register and will not take torque again has not recovered from anything.
/// Both are test steps, and a test step inside a make-safe path is a make-safe
/// path that can re-torque a servo in an unknown state.
fn watchdog_rearms<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    line: &mut dyn FnMut(&str),
) -> Result<(), BareError> {
    write_byte(bus, map, row, RegId::BusWatchdog, 0)?;
    line("  cleared: a zero to the bus watchdog, read back");
    write_byte(bus, map, row, RegId::BusWatchdog, WATCHDOG_COUNTS)?;
    // Where it stands now, written as its goal before torque goes back on. The
    // servo has been limp since the trip and may have drooped, and the goal
    // register holds what it held before the trip: re-enabling torque against
    // that would be a commanded move out of a test that says it commands none.
    hold_where_it_stands(bus, map, row)?;
    line(&format!(
        "  re-armed at {WATCHDOG_COUNTS} counts and holding torque again"
    ));
    Ok(())
}

/// Disarm the servo and release it, whatever else happened.
///
/// Exactly two writes and no assertions. The torque-off goes out even when the
/// disarm failed: nothing gates de-torquing, a failed sibling write least of
/// all, and the watchdog register is RAM that a power cycle clears. When both
/// fail the returned error is the torque one, because that is the one an
/// operator has to act on; the disarm's failure is printed beside it.
fn watchdog_make_safe<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    line: &mut dyn FnMut(&str),
) -> Result<(), BareError> {
    // Disarmed before released, so the window between the two writes is not one
    // where a trip could beat the release to it.
    let disarmed = write_byte(bus, map, row, RegId::BusWatchdog, 0);
    let released = write_byte(bus, map, row, RegId::TorqueEnable, 0);
    match (disarmed, released) {
        (Ok(()), Ok(())) => {
            line("  disarmed and released, both read back");
            Ok(())
        }
        (Err(disarm), Ok(())) => {
            line(
                "  released, but the bus watchdog would not disarm; the register is RAM and a \
                  power cycle clears it",
            );
            Err(disarm)
        }
        (Ok(()), Err(release)) => Err(release),
        (Err(disarm), Err(release)) => {
            line(&format!(
                "  the bus watchdog would not disarm either: {disarm}"
            ));
            Err(release)
        }
    }
}

/// Assert that a released servo reads its goal as its present position.
///
/// The property every "this commands nothing" claim in this command rests on.
/// It is vendor behaviour, so it is one more reading the self-test establishes
/// on each run rather than a comment stating it.
fn watchdog_tracks_its_goal<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    line: &mut dyn FnMut(&str),
) -> Result<(), BareError> {
    let id = map.ids()[row];
    let at = read_counts(bus, id)?;
    let goal = read_raw(bus, map, row, RegId::GoalPosition)?
        .i32()
        .expect("a position register is four bytes wide");
    if goal != at {
        return Err(BareError::WatchdogGoalNotTracking { id, goal, at });
    }
    line(&format!(
        "  released: goal and position both read {at}, so a limp servo tracks its own goal"
    ));
    Ok(())
}

/// Write where the servo stands as its goal, then enable torque.
///
/// Both halves together, because either alone is the hazard: a goal written to a
/// servo that is about to hold it is only safe if it is where the servo already
/// is, and torque enabled without it is a servo commanded to whatever its goal
/// register happens to hold.
fn hold_where_it_stands<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
) -> Result<(), BareError> {
    let at = read_raw(bus, map, row, RegId::PresentPosition)?;
    write_verified(bus, map, row, RegId::GoalPosition, &at)?;
    write_byte(bus, map, row, RegId::TorqueEnable, 1)
}

/// Write one of a servo's one-byte registers, with the read-back the write path
/// does itself.
fn write_byte<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    reg: RegId,
    byte: u8,
) -> Result<(), BareError> {
    let id = map.ids()[row];
    let raw = map
        .encode_value(row, reg, value::u8(byte))
        .map_err(|source| BareError::Map { id, reg, source })?;
    write_verified(bus, map, row, reg, &raw)
}

/// Write `raw` to one register and read it back, with retry.
fn write_verified<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    reg: RegId,
    raw: &RawValue,
) -> Result<(), BareError> {
    let id = map.ids()[row];
    let entry = reg_for(reg).map_err(|source| BareError::Map { id, reg, source })?;
    with_retry(bus, |bus| bus.write_reg_verified(id, entry, raw))
        .map_err(|source| BareError::Bus { id, source })
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
    use std::cell::{Cell, RefCell};
    use std::rc::Rc;

    use dxl_proto::frame::{INST_REBOOT, INST_WRITE};

    use super::*;

    use crate::testutil::{
        ACCESS, BusWatchdogModel, Configured, DATA_RANGE, FakeMachine, Spy, TestClock, configured,
        example_config, machine_at, resolved, rest_legs, stow_legs, wind_down_bus,
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
        fn commanded_nothing(&self, cfg: &Configured) {
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
        run_at(machine, &Rc::new(Cell::new(Duration::ZERO)), command)
    }

    /// As [`run`], on a clock the case holds -- for a machine that measures a
    /// timer of its own against the same instant the command sleeps to.
    fn run_at<F>(machine: FakeMachine, now: &Rc<Cell<Duration>>, command: F) -> Run
    where
        F: FnOnce(Spy, &mut dyn Clock, &mut dyn FnMut(&str)) -> Result<(), BareError>,
    {
        let spy = Spy::new(machine);
        let registers = spy.machine();
        let log = spy.log();
        let mut clock = TestClock::sharing(now);
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

    /// Run `watchdog`, with `id` going deaf about `regs` the moment the
    /// transcript reaches a line starting with `after`.
    ///
    /// A case names the reading it wants the servo to survive and no more, so it
    /// says "the servo is gone by the time the next phase writes" without
    /// counting the transactions every phase before it happens to take -- a
    /// count that moves under any change to the command's traffic and says
    /// nothing about what the case is for.
    fn run_deaf_from(
        machine: FakeMachine,
        cfg: &Configured,
        id: u8,
        after: &str,
        regs: &[RegId],
    ) -> Run {
        run_deaf_from_at(
            machine,
            &Rc::new(Cell::new(Duration::ZERO)),
            cfg,
            id,
            after,
            regs,
        )
    }

    /// As [`run_deaf_from`], on a clock the case shares with a machine that
    /// models a watchdog of its own.
    fn run_deaf_from_at(
        machine: FakeMachine,
        now: &Rc<Cell<Duration>>,
        cfg: &Configured,
        id: u8,
        after: &str,
        regs: &[RegId],
    ) -> Run {
        let addrs: Vec<u16> = regs.iter().map(|reg| named_reg(*reg).addr).collect();
        let after = after.to_string();
        run_at(machine, now, |port, clock, line| {
            let machine = port.machine();
            let mut watched = |text: &str| {
                if text.trim_start().starts_with(&after) {
                    let mut machine = machine.borrow_mut();
                    for addr in &addrs {
                        machine.go_deaf(id, *addr);
                    }
                }
                line(text);
            };
            watchdog(&cfg.map, cfg.timing, port, None, clock, &mut watched)
        })
    }

    /// What each servo's Torque Enable register holds after a run.
    fn torque(cfg: &Configured, run: &Run) -> Vec<u8> {
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
        let mut machine = machine_at(&example_config(), &stow_legs());
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
            machine_at(&example_config(), &stow_legs()),
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
        let mut machine = machine_at(&example_config(), &stow_legs());
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
    fn unprovisioned(cfg: &Configured) -> FakeMachine {
        let mut machine = machine_at(&example_config(), &stow_legs());
        for id in [cfg.map.ids()[7], cfg.map.ids()[8]] {
            machine.set(id, named_reg(RegId::OperatingMode), &[3]);
        }
        machine
    }

    /// What each servo's Operating Mode register holds after a run.
    fn modes(cfg: &Configured, run: &Run) -> Vec<u8> {
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
        let machine = machine_at(&example_config(), &stow_legs());
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
    fn overloaded(cfg: &Configured) -> FakeMachine {
        let mut machine = machine_at(&example_config(), &stow_legs());
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
        let mut file = example_config();
        wind_down_bus(&mut file);
        file.bus.retry_spacing_ms = 250;
        let cfg = configured(&file);
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

    /// The servo a watchdog self-test addresses when nothing is named.
    fn antenna(cfg: &Configured) -> u8 {
        cfg.map.ids()[row(WATCHDOG_JOINT).expect("a named joint has a bus row")]
    }

    /// What one of a servo's one-byte registers holds after a run.
    fn byte_of(run: &Run, id: u8, reg: RegId) -> Option<u8> {
        run.registers
            .borrow()
            .get(id, named_reg(reg))
            .map(|bytes| bytes[0])
    }

    /// A servo already holding torque is refused before anything is armed.
    ///
    /// The test torques the servo itself and then watches it let go; one that
    /// arrived holding is standing somewhere this command did not put it, and
    /// the reading would be about that pose instead.
    #[test]
    fn the_watchdog_test_refuses_a_servo_that_is_already_holding() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let mut machine = machine_at(&example_config(), &rest_legs());
        machine.set(id, named_reg(RegId::TorqueEnable), &[1]);

        let run = run(machine, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a servo already holding is not the state this asserts from");
        let BareError::WatchdogTorqueHeld { id: held } = error else {
            panic!("expected a torque refusal, got {error}");
        };
        assert_eq!(*held, id);
        assert_eq!(
            byte_of(&run, id, RegId::BusWatchdog),
            Some(0),
            "a refused run armed nothing",
        );
        assert_eq!(
            byte_of(&run, id, RegId::TorqueEnable),
            Some(1),
            "and left the hold exactly as it found it",
        );
    }

    /// A servo the roster does not carry is refused by name, and nothing goes
    /// out on the wire.
    #[test]
    fn the_watchdog_test_refuses_a_servo_off_the_roster() {
        let cfg = resolved();
        let stranger = 99;
        assert!(!cfg.map.ids().contains(&stranger));

        let run = run(
            machine_at(&example_config(), &rest_legs()),
            |port, clock, line| watchdog(&cfg.map, cfg.timing, port, Some(stranger), clock, line),
        );

        let error = run.err("that servo is not on this machine");
        let BareError::OffRoster { id, roster } = error else {
            panic!("expected a roster refusal, got {error}");
        };
        assert_eq!((*id, *roster), (stranger, cfg.map.ids()));
        assert!(run.log.borrow().is_empty(), "{:?}", run.log.borrow());
    }

    /// A machine that models no watchdog at all fails the silence assertion, and
    /// the run still leaves the servo disarmed and limp.
    ///
    /// This fixture is a register file: it stores what the arming write puts in
    /// the register and nothing ever trips. So the two busy phases pass, the
    /// silence does not, and what the case is really about is everything around
    /// that verdict — the phases run in order, the servo is commanded nowhere,
    /// and the cleanup runs on the failing path, which is the path a bring-up
    /// run is most likely to take. What a real servo does with silence is the
    /// assertion itself, and no fixture here is allowed to answer it.
    #[test]
    fn a_machine_that_models_no_watchdog_fails_the_silence_and_is_left_limp() {
        let cfg = resolved();
        let id = antenna(&cfg);

        let run = run(
            machine_at(&example_config(), &rest_legs()),
            |port, clock, line| watchdog(&cfg.map, cfg.timing, port, None, clock, line),
        );

        let error = run.err("a register file trips on nothing");
        let BareError::WatchdogNeverTripped {
            id: named,
            read,
            silent,
            latched,
        } = error
        else {
            panic!("expected the silence assertion to fail, got {error}");
        };
        assert_eq!(
            (*named, *read, *latched),
            (id, WATCHDOG_COUNTS, WATCHDOG_LATCHED),
        );
        assert_eq!(*silent, WATCHDOG_TIMEOUT * WATCHDOG_SILENT_TIMEOUTS);

        for phase in [Busy::ReadsAndGoals.name(), Busy::ReadsOnly.name()] {
            assert!(
                run.printed
                    .iter()
                    .any(|printed| printed.contains(phase) && printed.contains("still holding")),
                "{phase} did not report a servo that kept its torque: {:?}",
                run.printed,
            );
        }

        // The register verdict is the one that fired, and the other two readings
        // were taken anyway: this is the path a bring-up run takes, and a
        // judgment hoisted above a reading would end it with nothing said about
        // the torque -- which is the defect this shape exists to close.
        for reading in ["torque after the trip:", "probe:"] {
            assert!(
                run.printed
                    .iter()
                    .any(|printed| printed.trim_start().starts_with(reading)),
                "the run judged the register without reading {reading}: {:?}",
                run.printed,
            );
        }

        assert_eq!(
            (
                byte_of(&run, id, RegId::BusWatchdog),
                byte_of(&run, id, RegId::TorqueEnable),
            ),
            (Some(0), Some(0)),
            "a failing run still left the servo disarmed and limp",
        );

        let machine = run.registers.borrow();
        assert_eq!(
            machine.get(id, named_reg(RegId::GoalPosition)),
            machine.get(id, named_reg(RegId::PresentPosition)),
            "the only goal written is where the servo was already standing",
        );
        drop(machine);
        for other in cfg.map.ids().iter().filter(|other| **other != id) {
            assert!(
                !run.log.borrow().iter().any(|(asked, _)| asked == other),
                "servo {other} was addressed by a test scoped to one servo",
            );
        }
    }

    /// A machine whose watchdog behaves the way the command expects, and the
    /// clock both it and the command read.
    ///
    /// Silence trips it, the trip marks the register and releases the servo, and
    /// a zero clears it -- which is what lets these cases put the *command* to a
    /// latch. What a real servo does is still the hardware assertion the command
    /// exists for, and the release half of this fixture is what the fault policy
    /// requires rather than anything anyone has watched.
    fn watchdog_machine(id: u8) -> (FakeMachine, Rc<Cell<Duration>>) {
        let now = Rc::new(Cell::new(Duration::ZERO));
        let mut machine = machine_at(&example_config(), &rest_legs());
        machine.watchdog = Some(BusWatchdogModel::expected(
            id,
            &now,
            Duration::from_millis(WATCHDOG_UNIT_MS),
        ));
        (machine, now)
    }

    /// The whole sequence over a servo whose watchdog does what this command
    /// expects: every stage reported, and the servo left disarmed and limp.
    #[test]
    fn a_watchdog_that_behaves_as_expected_passes_every_stage() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let (machine, now) = watchdog_machine(id);

        let run = run_at(machine, &now, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        run.ok("a servo that trips on silence and clears on a zero passes");
        for stage in [
            "armed:",
            "released: goal and position",
            "holding:",
            Busy::ReadsAndGoals.name(),
            Busy::ReadsOnly.name(),
            "silence:",
            "register:",
            "torque after the trip:",
            "probe:",
            "released: torque enable reads off",
            "cleared:",
            "re-armed",
            "disarmed and released",
            "Disarmed and limp now.",
        ] {
            assert!(
                run.printed.iter().any(|printed| printed.contains(stage)),
                "no stage reported `{stage}`: {:?}",
                run.printed,
            );
        }
        assert_eq!(
            (
                byte_of(&run, id, RegId::BusWatchdog),
                byte_of(&run, id, RegId::TorqueEnable),
            ),
            (Some(0), Some(0)),
            "a passing run leaves the servo disarmed and limp",
        );
        let machine = run.registers.borrow();
        assert_eq!(
            machine.get(id, named_reg(RegId::GoalPosition)),
            machine.get(id, named_reg(RegId::PresentPosition)),
            "the only goal ever written is where the servo was already standing",
        );
    }

    /// A trip that does not release the servo fails the assertion the whole
    /// policy rests on.
    #[test]
    fn a_trip_that_keeps_holding_torque_fails_the_assertion_the_policy_rests_on() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let (mut machine, now) = watchdog_machine(id);
        machine
            .watchdog
            .as_mut()
            .expect("the fixture has one")
            .drops_torque = false;

        let run = run_at(machine, &now, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a servo still holding after a trip is the finding");
        assert!(
            matches!(error, BareError::WatchdogStillHolding { id: named } if *named == id),
            "expected the torque read-back to fail, got {error}",
        );
        assert_eq!(
            byte_of(&run, id, RegId::TorqueEnable),
            Some(0),
            "and the cleanup released it anyway",
        );
    }

    /// A tripped servo that takes a goal write is a trip that refuses nothing,
    /// which is the other half of the same finding.
    #[test]
    fn a_trip_that_still_takes_a_goal_write_is_reported_as_one() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let (mut machine, now) = watchdog_machine(id);
        machine
            .watchdog
            .as_mut()
            .expect("the fixture has one")
            .refuses_writes_with = None;

        let run = run_at(machine, &now, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a tripped servo that writes is not a tripped servo");
        assert!(
            matches!(error, BareError::WatchdogGoalAccepted { id: named } if *named == id),
            "expected the accepted write to be the verdict, got {error}",
        );
    }

    /// A refusal with some other code is surfaced with the byte in it, because
    /// which refusal it was is the discovery.
    ///
    /// Data Range is the code the vendor's worked example claims for a latched
    /// watchdog, and it is the *wrong* one here: the hardware answers Access, so
    /// a servo answering Data Range is a servo whose signature nobody has seen
    /// and the byte goes out whole.
    #[test]
    fn a_trip_that_refuses_with_another_code_carries_the_byte_it_answered_with() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let (mut machine, now) = watchdog_machine(id);
        machine
            .watchdog
            .as_mut()
            .expect("the fixture has one")
            .refuses_writes_with = Some(DATA_RANGE);

        let run = run_at(machine, &now, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a refusal that is not Access is not the signature");
        let BareError::WatchdogRefusedOtherwise { id: named, error } = error else {
            panic!("expected the other-refusal verdict, got {error}");
        };
        assert_eq!((*named, error.0), (id, DATA_RANGE));
        assert_ne!(
            DATA_RANGE, ACCESS,
            "the case rests on these being different"
        );
    }

    /// The refusal check reads the code and not the byte, so the alert bit this
    /// unit carries on every status packet does not turn a good signature into a
    /// finding.
    ///
    /// `0x87` is Access with the alert bit set by the standing input-voltage
    /// latch that every servo on this machine carries, which has nothing to do
    /// with the watchdog.
    #[test]
    fn a_refusal_carrying_the_alert_bit_is_still_the_expected_signature() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let (mut machine, now) = watchdog_machine(id);
        machine
            .watchdog
            .as_mut()
            .expect("the fixture has one")
            .refuses_writes_with = Some(0x80 | ACCESS);

        let run = run_at(machine, &now, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        run.ok("an alert bit beside Access is the signature this hardware sends");
        assert!(
            run.printed
                .iter()
                .any(|printed| printed.contains("probe:") && printed.contains("0x87")),
            "the byte was not printed whole: {:?}",
            run.printed,
        );
    }

    /// Every reading is taken before any of them is judged, and the torque
    /// reading outranks the refusal byte.
    ///
    /// The two failures together — an unexpected refusal code beside a servo
    /// that did not let go — are why the ordering matters. Judged in wire order
    /// the refusal ends the run and nobody learns about the torque. The verdict
    /// here is the torque one, and the probe byte is on the transcript
    /// regardless.
    #[test]
    fn the_torque_reading_outranks_the_refusal_byte_and_both_are_on_the_transcript() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let (mut machine, now) = watchdog_machine(id);
        {
            let dog = machine.watchdog.as_mut().expect("the fixture has one");
            dog.drops_torque = false;
            dog.refuses_writes_with = Some(DATA_RANGE);
        }

        let run = run_at(machine, &now, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a servo still holding after a trip is the finding");
        assert!(
            matches!(error, BareError::WatchdogStillHolding { id: named } if *named == id),
            "expected the torque verdict to outrank the refusal one, got {error}",
        );
        assert!(
            run.printed
                .iter()
                .any(|printed| printed.contains("torque after the trip:")
                    && printed.contains("HELD")),
            "the torque reading was not reported: {:?}",
            run.printed,
        );
        assert!(
            run.printed
                .iter()
                .any(|printed| printed.contains("probe:") && printed.contains("0x04")),
            "the probe byte was not reported: {:?}",
            run.printed,
        );
    }

    /// A watchdog that trips while the bus is busy fails the phase that was
    /// keeping it busy, and says which phase that was.
    #[test]
    fn a_watchdog_that_trips_under_traffic_names_the_phase_it_tripped_in() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let (mut machine, now) = watchdog_machine(id);
        {
            let dog = machine.watchdog.as_mut().expect("the fixture has one");
            // Well inside the first phase, whose traffic is a read and a
            // verified write every twenty milliseconds. The trip marks the
            // register and leaves the servo holding and its writes taken, so
            // what the phase meets is the marker rather than a refusal.
            dog.trips_after_frames = Some(20);
            dog.drops_torque = false;
            dog.refuses_writes_with = None;
        }

        let run = run_at(machine, &now, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("traffic that does not reset the count is the finding");
        let BareError::WatchdogTrippedEarly {
            id: named,
            phase,
            read,
            armed,
            ..
        } = error
        else {
            panic!("expected the early-trip verdict, got {error}");
        };
        assert_eq!(
            (*named, *phase, *read, *armed),
            (
                id,
                Busy::ReadsAndGoals.name(),
                WATCHDOG_LATCHED,
                WATCHDOG_COUNTS
            ),
        );
    }

    /// A servo that lets go under traffic without its register saying anything
    /// is the same phase failing on the other reading.
    #[test]
    fn a_servo_that_lets_go_under_traffic_fails_the_phase_on_the_torque_reading() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let (mut machine, now) = watchdog_machine(id);
        {
            let dog = machine.watchdog.as_mut().expect("the fixture has one");
            dog.trips_after_frames = Some(20);
            dog.marks_register = false;
            dog.refuses_writes_with = None;
        }

        let run = run_at(machine, &now, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a servo that stopped holding under traffic is the finding");
        let BareError::WatchdogReleasedEarly { id: named, phase } = error else {
            panic!("expected the early-release verdict, got {error}");
        };
        assert_eq!((*named, *phase), (id, Busy::ReadsAndGoals.name()));
    }

    /// A trip that lands between the arming write's read-back and the read after
    /// it is exactly the shape `WatchdogNotArmed` is for: the register took the
    /// value and holds something else a transaction later.
    #[test]
    fn a_register_that_changes_after_its_read_back_stops_the_run_unarmed() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let (mut machine, now) = watchdog_machine(id);
        // Six frames reach this servo before the read that confirms the arming:
        // the ping, the torque read, and the write-plus-read-back of each of the
        // two arming writes. So the seventh -- that read -- is the one this trip
        // lands on.
        machine
            .watchdog
            .as_mut()
            .expect("the fixture has one")
            .trips_after_frames = Some(6);

        let run = run_at(machine, &now, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a register that reads back armed and then does not is not armed");
        let BareError::WatchdogNotArmed {
            id: named,
            armed,
            read,
        } = error
        else {
            panic!("expected the arming verdict, got {error}");
        };
        assert_eq!(
            (*named, *armed, *read),
            (id, WATCHDOG_COUNTS, WATCHDOG_LATCHED)
        );
        assert_eq!(
            (
                byte_of(&run, id, RegId::BusWatchdog),
                byte_of(&run, id, RegId::TorqueEnable),
            ),
            (Some(0), Some(0)),
            "an arming-phase failure still leaves the servo disarmed and limp",
        );
    }

    /// A limp servo whose goal register points somewhere else stops the run
    /// before torque, because torquing it would be a commanded move.
    #[test]
    fn a_servo_that_does_not_track_its_goal_while_limp_stops_before_torque() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let mut machine = machine_at(&example_config(), &rest_legs());
        // A servo whose goal register is a store rather than a mirror, holding a
        // pose from before it was released.
        machine.unmirrored.push(id);
        machine.set(id, named_reg(RegId::GoalPosition), &1_234i32.to_le_bytes());

        let run = run(machine, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a stale goal is a commanded move waiting for torque");
        let BareError::WatchdogGoalNotTracking {
            id: named,
            goal,
            at,
        } = error
        else {
            panic!("expected the goal-tracking verdict, got {error}");
        };
        assert_eq!((*named, *goal), (id, 1_234));
        assert_ne!(*at, 1_234, "the case rests on the two disagreeing");
        assert_eq!(
            (
                byte_of(&run, id, RegId::BusWatchdog),
                byte_of(&run, id, RegId::TorqueEnable),
            ),
            (Some(0), Some(0)),
            "nothing was torqued, and the register the run armed was disarmed again",
        );
    }

    /// A torque-enable that will not take stops the run, and the watchdog this
    /// run armed a moment earlier is disarmed on the way out.
    ///
    /// The register is armed before the hold, so this is the first phase whose
    /// failure would otherwise leave a servo armed with nobody watching it. It
    /// is the reason every write in this command lives inside the exercise half.
    #[test]
    fn a_hold_that_will_not_take_still_leaves_the_servo_disarmed_and_limp() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let mut machine = machine_at(&example_config(), &rest_legs());
        // Torque Enable acknowledges its writes and stores nothing, so enabling
        // torque reads back as off and the verified write fails -- while the
        // make-safe's write of zero reads back as the zero it wanted.
        machine
            .ignored
            .push((id, named_reg(RegId::TorqueEnable).addr));

        let run = run(machine, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a servo that will not hold has nothing to observe");
        assert!(
            matches!(error, BareError::Bus { id: named, .. } if *named == id),
            "expected the torque write's read-back to catch it, got {error}",
        );
        assert_eq!(
            byte_of(&run, id, RegId::BusWatchdog),
            Some(0),
            "a hold that failed still left the register this run armed disarmed",
        );
        // This servo stores nothing about its torque, which is the whole point
        // of the case, so all its register can say is that nothing was stored.
        assert!(
            matches!(byte_of(&run, id, RegId::TorqueEnable), None | Some(0)),
            "nothing is holding",
        );
    }

    /// The make-safe writes the torque off even when the disarm before it
    /// failed, and says both.
    ///
    /// Nothing gates de-torquing, a failed sibling write least of all: the
    /// watchdog register is RAM a power cycle clears, and a servo left holding
    /// because an unrelated write did not land is the one state that matters.
    #[test]
    fn a_disarm_that_fails_does_not_stop_the_torque_from_being_released() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let machine = machine_at(&example_config(), &rest_legs());
        // Nothing models a watchdog, so the silence assertion is what fails; and
        // this servo goes away about its watchdog register once that phase has
        // taken its readings, so the make-safe's disarm cannot read its write
        // back.
        let run = run_deaf_from(machine, &cfg, id, "probe:", &[RegId::BusWatchdog]);

        let error = run.err("a register file trips on nothing");
        assert!(
            matches!(error, BareError::WatchdogNeverTripped { .. }),
            "the assertion's own failure is what the run returns, got {error}",
        );
        assert!(
            run.printed
                .iter()
                .any(|printed| printed.contains("would not disarm")),
            "the disarm's failure was swallowed: {:?}",
            run.printed,
        );
        assert_eq!(
            byte_of(&run, id, RegId::TorqueEnable),
            Some(0),
            "the torque came off anyway, and it read back",
        );
    }

    /// Both make-safe writes failing says both, and the error that comes out is
    /// the torque one.
    #[test]
    fn a_make_safe_that_fails_twice_reports_both_and_returns_the_torque_failure() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let machine = machine_at(&example_config(), &rest_legs());
        // Both of the make-safe's registers, gone by the time it writes.
        let run = run_deaf_from(
            machine,
            &cfg,
            id,
            "probe:",
            &[RegId::BusWatchdog, RegId::TorqueEnable],
        );

        let error = run.err("a register file trips on nothing");
        assert!(
            matches!(error, BareError::WatchdogNeverTripped { .. }),
            "the assertion's own failure is still what the run returns, got {error}",
        );
        for said in ["would not disarm either", "cleanup did not finish"] {
            assert!(
                run.printed.iter().any(|printed| printed.contains(said)),
                "no line said `{said}`: {:?}",
                run.printed,
            );
        }
    }

    /// A run whose assertion fails *and* whose cleanup fails says both, because
    /// a cleanup that did not finish may be a servo still holding torque.
    #[test]
    fn a_cleanup_that_could_not_finish_is_reported_beside_the_assertion_that_failed() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let machine = machine_at(&example_config(), &rest_legs());
        // Nothing models a watchdog here, so the silence assertion is what
        // fails; and this servo goes away about its torque register once that
        // phase has taken its readings, so the make-safe's verified write
        // cannot read anything back.
        let run = run_deaf_from(machine, &cfg, id, "probe:", &[RegId::TorqueEnable]);

        let error = run.err("a register file trips on nothing");
        assert!(
            matches!(error, BareError::WatchdogNeverTripped { .. }),
            "the assertion's own failure is what the run returns, got {error}",
        );
        assert!(
            run.printed
                .iter()
                .any(|printed| printed.contains("cleanup did not finish")),
            "the cleanup's failure was swallowed: {:?}",
            run.printed,
        );
    }

    /// A bus watchdog register that acknowledges its arming write and stores
    /// nothing stops the run before any torque is enabled.
    ///
    /// Everything after the arming write is a reading about a register that is
    /// armed, so a run that could not arm one has nothing left to observe — and
    /// no reason to have torqued a servo.
    #[test]
    fn an_arming_write_that_does_not_take_stops_before_any_torque() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let mut machine = machine_at(&example_config(), &rest_legs());
        machine
            .ignored
            .push((id, named_reg(RegId::BusWatchdog).addr));

        let run = run(machine, |port, clock, line| {
            watchdog(&cfg.map, cfg.timing, port, None, clock, line)
        });

        let error = run.err("a register that stores nothing arms nothing");
        assert!(
            matches!(error, BareError::Bus { id: named, .. } if *named == id),
            "expected the write's own read-back to catch it, got {error}",
        );
        assert!(
            matches!(byte_of(&run, id, RegId::TorqueEnable), None | Some(0)),
            "nothing was torqued",
        );
    }
    /// A re-arm phase that fails leaves the servo disarmed and limp anyway.
    ///
    /// The one phase that fails with the register freshly armed: it is why the
    /// clear and the re-arm are exercise steps rather than make-safe ones, and
    /// the residual state it can leave -- armed and holding -- is the machine's
    /// only pinch hazard, so what undoes it is asserted rather than assumed.
    #[test]
    fn a_re_arm_that_fails_still_leaves_the_servo_disarmed_and_limp() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let (machine, now) = watchdog_machine(id);

        // Through the silence and its clear, then gone about the goal register:
        // the re-arm's hold writes a goal before it enables torque, so this
        // fails the phase after the register has been armed again.
        let run = run_deaf_from_at(machine, &now, &cfg, id, "cleared:", &[RegId::GoalPosition]);

        let error = run.err("a goal write nobody reads back is not a re-arm");
        assert!(
            matches!(error, BareError::Bus { id: named, .. } if *named == id),
            "expected the re-arm's own write to catch it, got {error}",
        );
        assert!(
            !run.printed
                .iter()
                .any(|printed| printed.contains("re-armed")),
            "the phase reported a re-arm it did not finish: {:?}",
            run.printed,
        );
        assert_eq!(
            (
                byte_of(&run, id, RegId::BusWatchdog),
                byte_of(&run, id, RegId::TorqueEnable),
            ),
            (Some(0), Some(0)),
            "a servo left armed by the re-arm was disarmed and released by the make-safe",
        );
    }

    /// A run whose assertions all passed and whose make-safe could not release
    /// the servo returns the make-safe's failure, and claims nothing.
    ///
    /// The one path where the cleanup's error is what comes out of the command.
    /// A run that read green while the release was never confirmed is the report
    /// an operator would trust and walk away from.
    #[test]
    fn a_green_run_whose_make_safe_could_not_release_returns_that_failure() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let (machine, now) = watchdog_machine(id);

        // Every exercise phase passes -- the re-arm's own hold is written before
        // its line prints -- and the servo is gone about its torque register by
        // the time the make-safe writes.
        let run = run_deaf_from_at(machine, &now, &cfg, id, "re-armed", &[RegId::TorqueEnable]);

        let error = run.err("a release nobody read back is not a release");
        assert!(
            matches!(error, BareError::Bus { id: named, .. } if *named == id),
            "expected the make-safe's failure to be the verdict, got {error}",
        );
        for unsaid in ["disarmed and released", "Disarmed and limp now."] {
            assert!(
                !run.printed.iter().any(|printed| printed.contains(unsaid)),
                "the run said `{unsaid}` about a release it could not confirm: {:?}",
                run.printed,
            );
        }
        assert_eq!(
            byte_of(&run, id, RegId::BusWatchdog),
            Some(0),
            "the disarm before it did land",
        );
    }

    /// A bus-level answer to the goal probe returns as itself, after both
    /// register readings are on the transcript.
    ///
    /// A failed observation, not a judged one: the register and the torque are
    /// already read by then, and a probe hoisted above them would end a run
    /// with nothing said about the torque -- which is exactly the shape of the
    /// first hardware run's defect.
    #[test]
    fn a_bus_error_on_the_probe_returns_after_both_readings_are_printed() {
        let cfg = resolved();
        let id = antenna(&cfg);
        let machine = machine_at(&example_config(), &rest_legs());

        // Gone about its goal register the moment the torque reading prints, so
        // the probe's verified write is what cannot be read back.
        let run = run_deaf_from(
            machine,
            &cfg,
            id,
            "torque after the trip:",
            &[RegId::GoalPosition],
        );

        let error = run.err("a probe nobody can read back observes nothing");
        assert!(
            matches!(error, BareError::Bus { id: named, .. } if *named == id),
            "expected the probe's own write to be the verdict, got {error}",
        );
        for reading in ["register:", "torque after the trip:"] {
            assert!(
                run.printed
                    .iter()
                    .any(|printed| printed.trim_start().starts_with(reading)),
                "the probe failed before {reading} was said: {:?}",
                run.printed,
            );
        }
        assert!(
            !run.printed
                .iter()
                .any(|printed| printed.trim_start().starts_with("probe:")),
            "a probe that never answered was reported anyway: {:?}",
            run.printed,
        );
        assert_eq!(
            (
                byte_of(&run, id, RegId::BusWatchdog),
                byte_of(&run, id, RegId::TorqueEnable),
            ),
            (Some(0), Some(0)),
            "a failed observation still left the servo disarmed and limp",
        );
    }
}
