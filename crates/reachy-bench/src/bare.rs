//! The bench's bare-bus commands: the ones that need no tick, no sequencer and
//! no kinematics.
//!
//! Each one opens with a bus and a roster and nothing else. `provision` writes
//! the antennas' operating mode, `reboot` restarts the servos, `off` sweeps
//! torque off, `watchdog` establishes what an armed Bus Watchdog does to one
//! servo, and `hold-probe` watches one held servo far faster than a driver
//! cycle can. None of them commands an angle — the goal `watchdog` and
//! `hold-probe` write is the count the servo reports for itself — so none of
//! them needs an envelope, a pose or a control loop; what they share is the
//! register-level plumbing at the bottom of this file.
//!
//! `watchdog` and `hold-probe` are the two that torque a servo, and they are the
//! odd ones here for that reason: each is a supervised bring-up assertion, one
//! about a register the session path arms on every engagement and one about
//! whether a joint commanded to stand still does. `hold-probe` samples a single
//! servo's Present Position in a tight loop — around a kilohertz on this wire,
//! against the driver's fifty — which is what separates a slow limit cycle from
//! a buzz the driver's own rate can only alias, and it runs the same window
//! twice: once with the goal rewritten as the driver rewrites it, once with
//! reads alone, so a wobble that needs the host's writes to exist says so.
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

use core::fmt;
use core::time::Duration;
use std::cell::OnceCell;
use std::time::Instant;

use dxl_proto::{HardwareError, StatusCode, StatusError, counts_to_rad};
use reachy_bus::{
    Bus, BusPort, BusTiming, MapError, RawValue, ServoMap, XactError, named_reg, reg_for,
    with_retry,
};
use reachy_motion::joints::{Name, ROW_COUNT, row};
use reachy_motion::reg::Name as RegName;
use reachy_motion::stillness;
use reachy_motion::value;
use reachy_motion::{
    EXPECTED_MODELS, EXPECTED_OPERATING_MODES, Gains, JointRef, RegId, Value, ValueShape,
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

/// The cadence a command that keeps the bus busy writes goals at: the driver's
/// own command period, which is also one Bus Watchdog register count.
///
/// Two commands write at it, for two reasons. The watchdog's no-trip phases
/// keep the bus busy the way a session does; the hold probe rewrites the goal
/// at it because a wobble driven by the host's writes is driven by writes at
/// this spacing and no other.
const COMMAND_PERIOD: Duration = Duration::from_millis(WATCHDOG_UNIT_MS);

/// The joint the hold probe addresses unless the operator names another: an
/// antenna, which is the joint reported to hunt and the one whose torque costs
/// the least to hold.
const HOLD_PROBE_JOINT: JointRef = JointRef::AntennaRight;

/// How long each of the probe's two phases runs unless the operator says
/// otherwise. Seconds, so a hold of a few hundred cycles of anything slower
/// than the sample rate is inside one phase.
///
/// Public because the binary's own usage text quotes it: the default a command
/// runs at is stated once, where the command is.
pub const HOLD_PROBE_SECONDS: u64 = 3;

/// The longest phase an operator may ask for. A phase is a servo held under
/// torque with nobody but the operator watching, and the command is attended by
/// construction; a mistyped figure that held it for an hour would be a hold
/// nobody meant to command.
const HOLD_PROBE_MAX_SECONDS: u64 = 60;

/// The excursion a held joint is asserted to stay inside, in encoder counts.
///
/// The stillness watch's own bound, read back out of the radians it is stated
/// in rather than restated here: what the session judges a hold against and
/// what this judges one against are one figure, and a probe that passed a hold
/// the session would fail would be an instrument disagreeing with the machine.
const HOLD_PROBE_BOUND_COUNTS: f64 = stillness::MAX_EXCURSION_RAD / stillness::COUNT_RAD;

/// What a probe's series file is named for: the prefix, and everything after it
/// is the moment it was taken and the servo it came from.
///
/// Stated here, where the command is, because two other places have to know it
/// — the binary that writes the file, and the fetch that globs for it on the
/// device. The fetch is shell and cannot read this constant, so its test
/// compares the two texts; what the test compares against is this one.
pub const HOLD_PROBE_SERIES_PREFIX: &str = "hold-probe-";

/// The most readings a second of one phase can hold, for the series to be
/// sized before the loop runs. A read of one register is hundreds of
/// microseconds of wire at this baud plus the host's turnaround, so a couple of
/// thousand a second is a ceiling nothing on this bus reaches.
const HOLD_PROBE_RATE_CEILING_HZ: usize = 2000;

/// The shortest differenced series a dominant period is read off. Below it the
/// lag range is a handful of lags over a handful of products, and the largest
/// of those is noise wearing a number.
const HOLD_PROBE_MIN_SERIES: usize = 16;

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

    /// The servo the hold probe addresses was already holding torque, so it is
    /// standing somewhere this command did not put it — a session's pose, or a
    /// hold left behind. The probe torques the servo itself at the count it
    /// reports, and what it measures is that hold; a servo that arrived holding
    /// would be measured in somebody else's.
    #[error("servo {id} is holding torque; release it with `off` before the hold probe")]
    HoldProbeTorqueHeld {
        /// The servo addressed.
        id: u8,
    },

    /// The servo the hold probe addresses has its Bus Watchdog armed or
    /// latched, so the second phase — seconds of reads with no goal write —
    /// would be watched by a timer that stops the servo partway through. The
    /// quiet second phase that came back would then be a servo the watchdog
    /// halted rather than one the host stopped disturbing, which is the exact
    /// difference the two phases exist to tell apart.
    #[error(
        "servo {id} has its bus watchdog at {read}, not 0; a session or a watchdog run left it \
         armed. `off` releases torque and writes nothing else, so it does not clear this: \
         `reboot {id}`, or run `watchdog {id}`, before the hold probe"
    )]
    HoldProbeWatchdogArmed {
        /// The servo addressed.
        id: u8,
        /// What the register answered.
        read: u8,
    },

    /// A phase was asked for that is longer than an attended hold is meant to
    /// be. Refused before the port is touched.
    #[error(
        "a hold probe phase of {asked} s is longer than the {most} s this command holds a servo for"
    )]
    HoldProbeTooLong {
        /// The figure asked for, in seconds.
        asked: u64,
        /// The longest one accepted, in seconds.
        most: u64,
    },

    /// A servo commanded to stand still did not. The bring-up assertion of this
    /// command: the figures beside it are the discovery, and the phase says
    /// whether the host's own goal rewrites were running while it happened.
    #[error(
        "servo {id} moved {excursion:.0} counts while held during {phase}, past the {bound:.1} count \
         bound, {}", Shown(*.period)
    )]
    HoldProbeExcursion {
        /// The servo addressed.
        id: u8,
        /// The traffic that was running.
        phase: Busy,
        /// How far it moved, peak to peak, in counts.
        excursion: f64,
        /// The bound it passed, in counts.
        bound: f64,
        /// What the series' own period read as — the half of the discovery
        /// that says which mechanism this is. Carried as the figures, and
        /// worded by the one place that words them.
        period: Option<ProbePeriod>,
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
/// are. This machine clears them to zero on a restart -- observed on all nine
/// servos of `reachy00`, 2026-08-28, the chronic input-voltage bit every servo
/// latches while running included, which running re-latches. So a byte that
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
    let (row, id) = bare_target(map, target, WATCHDOG_JOINT)?;
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

/// The servo a bare-bus command addresses: the one asked for, or the joint the
/// command defaults to.
fn bare_target(
    map: &ServoMap,
    target: Option<u8>,
    default: JointRef,
) -> Result<(usize, u8), BareError> {
    let roster = map.ids();
    let Some(id) = target else {
        let row = row(default).expect("a named joint has a bus row");
        return Ok((row, roster[row]));
    };
    match roster.iter().position(|held| *held == id) {
        Some(row) => Ok((row, id)),
        None => Err(BareError::OffRoster { id, roster }),
    }
}

/// The traffic one phase keeps on the bus.
///
/// Public because it is what a probe's readings are labelled with: which of the
/// two a wobble appears in is the reading the tuning branches on, and a caller
/// that has to compare English to find out is a caller that cannot.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Busy {
    /// What the driver does while it holds a pose: read where the servo is,
    /// write the goal again.
    ReadsAndGoals,
    /// Reads and nothing else — the case the vendor's documentation implies and
    /// nobody here has watched, and the one a contingency would hang off.
    ReadsOnly,
}

impl Busy {
    /// What an operator reads this phase as.
    #[must_use]
    pub fn name(self) -> &'static str {
        match self {
            Self::ReadsAndGoals => "reads and goal rewrites",
            Self::ReadsOnly => "reads alone",
        }
    }
}

impl fmt::Display for Busy {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(self.name())
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
        clock.sleep_until(clock.now() + COMMAND_PERIOD);
    }

    let read = read_byte(bus, map, row, RegId::BusWatchdog)?;
    if read != WATCHDOG_COUNTS {
        return Err(BareError::WatchdogTrippedEarly {
            id,
            phase,
            read,
            armed: WATCHDOG_COUNTS,
            busy,
            period: COMMAND_PERIOD,
        });
    }
    if read_byte(bus, map, row, RegId::TorqueEnable)? == 0 {
        return Err(BareError::WatchdogReleasedEarly { id, phase });
    }
    line(&format!(
        "  {phase}: {exchanges} exchanges over {busy:?} at one every {COMMAND_PERIOD:?}, \
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

/// One reading the probe took: when, and the count that came back.
#[derive(Clone, Copy, Debug)]
pub struct ProbeSample {
    /// Elapsed since the phase began.
    pub at: Duration,
    /// The Present Position register, as the count it is.
    pub counts: i32,
}

/// One phase of a probe: the traffic that was running, every reading taken
/// while it was, and what those readings say.
pub struct ProbePhase {
    /// The traffic that was running.
    pub kind: Busy,
    /// The readings, in the order they were taken.
    pub samples: Vec<ProbeSample>,
    /// What the readings say, read at the first ask and kept.
    ///
    /// Kept because the reading walks the series once per lag: a phase of a
    /// minute is a second's arithmetic, and a caller that asked twice would
    /// spend it twice. Not read as the phase closes, because that is a second
    /// of arithmetic between the two phases and again before the release, with
    /// the servo holding torque throughout and the bus idle — so the ask comes
    /// from a caller that has already put the torque off.
    stats: OnceCell<ProbeStats>,
}

/// The regular component of a series, if it has one.
#[derive(Clone, Copy, Debug)]
pub struct ProbePeriod {
    /// The lag it repeats at, in samples.
    pub lag: usize,
    /// The autocorrelation at that lag: one is a series that repeats exactly,
    /// and a figure near zero is a lag that won only because something had to.
    pub regularity: f64,
    /// The same lag against the phase's own measured rate, in milliseconds.
    pub millis: f64,
    /// Its reciprocal, in hertz.
    pub hz: f64,
}

/// What one phase says once its readings are read.
#[derive(Clone, Copy, Debug)]
pub struct ProbeStats {
    /// Readings taken.
    pub samples: usize,
    /// The rate they were taken at, measured from their own stamps rather than
    /// assumed: what the loop achieved is what bounds the frequencies it can
    /// separate, and a run that ran slower than it meant to must not be read as
    /// though it had not.
    pub rate_hz: f64,
    /// Peak to peak, in counts.
    pub excursion: f64,
    /// Direction changes per second, under the stillness watch's rule that a
    /// plateau is not a reversal.
    pub reversals_per_s: f64,
    /// Mean sample count between consecutive reversals, or `None` for a series
    /// with fewer than two.
    ///
    /// Printed beside [`Self::period`] because the two answer the same
    /// question differently, and their disagreement is itself a finding: a
    /// limit cycle turns round on a near-constant interval and correlates
    /// strongly at one lag, while encoder dither turns round as often at
    /// intervals scattered from one sample to several and correlates at none.
    pub reversal_interval_mean_samples: Option<f64>,
    /// Population standard deviation of those intervals, on the same series.
    pub reversal_interval_spread_samples: Option<f64>,
    /// The dominant period of the differenced series, if it has one.
    pub period: Option<ProbePeriod>,
}

/// One probe run: what it saw, and how it ended.
///
/// The readings and the verdict are separate because the readings outlive the
/// verdict — a phase that ended in a refusal is the reading a bring-up most
/// wants kept, and a caller writes the series down before it acts on the
/// outcome, the way the self-test saves its record before it refuses.
pub struct ProbeRun {
    /// The servo probed.
    pub id: u8,
    /// The phases, in the order they ran. A phase cut short by a bus failure is
    /// here with the readings it got.
    pub phases: Vec<ProbePhase>,
    /// The bring-up assertion, and any failure the phases or the make-safe met.
    pub outcome: Result<(), BareError>,
}

impl ProbeRun {
    /// Both phases' readings as one comma-separated table: the phase, the
    /// elapsed milliseconds within it, and the count.
    ///
    /// The whole series rather than the figures, because the figures are a
    /// reading of it and a run that earns a fixture earns it as data. Rendered
    /// here and written by the caller, so what a file holds is exercisable
    /// without one.
    #[must_use]
    pub fn csv(&self) -> String {
        let mut out = String::from("phase,elapsed_ms,counts\n");
        for phase in &self.phases {
            for sample in &phase.samples {
                out.push_str(&format!(
                    "{phase},{at:.3},{counts}\n",
                    phase = phase.kind,
                    at = sample.at.as_secs_f64() * 1000.0,
                    counts = sample.counts,
                ));
            }
        }
        out
    }
}

impl ProbePhase {
    /// One phase and its readings, unread.
    #[must_use]
    pub fn new(kind: Busy, samples: Vec<ProbeSample>) -> Self {
        Self {
            kind,
            samples,
            stats: OnceCell::new(),
        }
    }

    /// What the readings say. Read at the first ask, kept for the rest.
    #[must_use]
    pub fn stats(&self) -> &ProbeStats {
        self.stats.get_or_init(|| read_series(&self.samples))
    }
}

/// How long a run of readings spans, in seconds.
fn span_of(samples: &[ProbeSample]) -> f64 {
    match (samples.first(), samples.last()) {
        (Some(first), Some(last)) => (last.at - first.at).as_secs_f64(),
        _ => 0.0,
    }
}

/// What one phase's readings say.
///
/// The excursion, the reversals and their intervals are read by the stillness
/// watch's own reader, so the figures this instrument judges a hold by and the
/// figures a session judges one by are one definition. The rate is measured
/// against the phase's own stamps: what the loop achieved is what bounds the
/// frequencies it can separate, and a run that ran slower than it meant to must
/// not be read as though it had not.
///
/// The dominant period is this command's own addition, and the series is
/// differenced for it: a hold's position series is a constant with a wobble on
/// it, and the constant carries no information about the wobble's shape while
/// dominating any correlation taken over it.
fn read_series(samples: &[ProbeSample]) -> ProbeStats {
    let span = span_of(samples);
    let wobble = stillness::Wobble::over(samples.iter().map(|sample| f64::from(sample.counts)));
    let (interval_mean, interval_spread) = wobble.interval_stats();
    let rate_hz = wobble.rate_hz(span).unwrap_or(0.0);
    let steps: Vec<f64> = samples
        .windows(2)
        .map(|pair| f64::from(pair[1].counts - pair[0].counts))
        .collect();
    ProbeStats {
        samples: samples.len(),
        rate_hz,
        excursion: wobble.excursion(),
        reversals_per_s: wobble.reversals_per_s(span),
        reversal_interval_mean_samples: interval_mean,
        reversal_interval_spread_samples: interval_spread,
        period: dominant_period(&steps).map(|(lag, regularity)| {
            let millis = if rate_hz > 0.0 {
                lag as f64 * 1000.0 / rate_hz
            } else {
                f64::NAN
            };
            ProbePeriod {
                lag,
                regularity,
                millis,
                hz: if millis > 0.0 {
                    1000.0 / millis
                } else {
                    f64::NAN
                },
            }
        }),
    }
}

impl fmt::Display for ProbeStats {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{samples} readings at {rate:.0} Hz, peak to peak {excursion:.0} counts, \
             {reversals:.1} reversals/s, {intervals}, {period}",
            samples = self.samples,
            rate = self.rate_hz,
            excursion = self.excursion,
            reversals = self.reversals_per_s,
            intervals = Intervals(
                self.reversal_interval_mean_samples,
                self.reversal_interval_spread_samples
            ),
            period = Shown(self.period),
        )
    }
}

/// The reversal intervals as an operator reads them, and the words for a series
/// with fewer than two reversals.
struct Intervals(Option<f64>, Option<f64>);

impl fmt::Display for Intervals {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match (self.0, self.1) {
            (Some(mean), Some(spread)) => write!(
                f,
                "reversal intervals {mean:.2} samples (spread {spread:.2})"
            ),
            _ => write!(f, "no reversal interval"),
        }
    }
}

/// A period as an operator reads it, and the words for a series that has none.
struct Shown(Option<ProbePeriod>);

impl fmt::Display for Shown {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.0 {
            Some(period) => write!(
                f,
                "period {lag} samples ({millis:.2} ms, {hz:.0} Hz, regularity {regularity:.2})",
                lag = period.lag,
                millis = period.millis,
                hz = period.hz,
                regularity = period.regularity,
            ),
            None => write!(f, "no period"),
        }
    }
}

/// The lag a differenced series repeats at, and how strongly, or nothing.
///
/// The autocorrelation of the mean-removed series at every lag from two up to
/// half its length, and the largest positive one wins. Lag one is excluded: a
/// series that alternates every sample correlates positively with itself at
/// two, and a positive correlation at one is a drift rather than a cycle.
/// Nothing is answered for a series too short to have a lag range worth
/// searching, and none for one that never moved — a flat hold has no period,
/// which is the answer, not a division by zero.
fn dominant_period(steps: &[f64]) -> Option<(usize, f64)> {
    if steps.len() < HOLD_PROBE_MIN_SERIES {
        return None;
    }
    let mean = steps.iter().sum::<f64>() / steps.len() as f64;
    let centred: Vec<f64> = steps.iter().map(|step| step - mean).collect();
    let energy: f64 = centred.iter().map(|step| step * step).sum();
    if energy <= 0.0 {
        return None;
    }
    let mut best: Option<(usize, f64)> = None;
    for lag in 2..=centred.len() / 2 {
        let sum: f64 = centred[..centred.len() - lag]
            .iter()
            .zip(&centred[lag..])
            .map(|(here, there)| here * there)
            .sum();
        let correlation = sum / energy;
        if correlation > 0.0 && best.is_none_or(|(_, held)| correlation > held) {
            best = Some((lag, correlation));
        }
    }
    best
}

/// What one probe run was asked for.
///
/// One struct rather than three arguments because they arrive together from one
/// invocation and travel together through it: the servo, the gains to run at,
/// and how long each phase is.
#[derive(Clone, Copy, Debug)]
pub struct ProbeRequest {
    /// The servo to probe, or nothing for the command's default antenna.
    pub target: Option<u8>,
    /// The position gains to swap in for the run, or nothing to leave the
    /// servo's own alone.
    pub gains: Option<Gains>,
    /// How long each of the two phases runs.
    pub seconds: Duration,
}

/// Hold one servo where it stands and watch it far faster than a driver does.
///
/// The instrument for a joint reported to wobble while it is holding still. A
/// driver cycle samples at 50 Hz, which aliases anything above 25 Hz onto some
/// other frequency and cannot say which; this loop reads one servo's Present
/// Position as fast as the wire answers — around a kilohertz at this baud — so
/// the frequency it reports is the one the joint has. What that frequency is
/// decides the lever: a slow limit cycle is the position loop working across
/// backlash, and a fast buzz is the derivative term amplifying the encoder's
/// own quantisation.
///
/// Two phases, each `seconds` long: the goal rewritten every command period as
/// the driver rewrites it, then reads alone. A wobble present in the first and
/// absent from the second is driven by the host's writes and is answered by the
/// driver rather than by a gain.
///
/// It commands no angle. The only goal ever written is the count the servo
/// reported for itself before torque went on, rewritten unchanged — the same
/// claim `watchdog` makes and for the same reason. Torque it does take, on one
/// servo, for a few seconds; the make-safe releases it and restores the gains
/// whichever way the run ends, and the command refuses to start on a servo that
/// is already holding something.
///
/// The requested gains are swapped in verified, and the kept triple is written
/// back on the way out. RAM registers: nothing here is non-volatile, and a
/// power cycle undoes the swap whatever this command managed.
///
/// The readings come back whatever the outcome, so a caller can write them down
/// before it acts on the verdict.
pub fn hold_probe<P: BusPort>(
    map: &ServoMap,
    timing: BusTiming,
    port: P,
    request: ProbeRequest,
    clock: &mut dyn Clock,
    line: &mut dyn FnMut(&str),
) -> Result<ProbeRun, BareError> {
    let asked = request.seconds.as_secs();
    if asked > HOLD_PROBE_MAX_SECONDS {
        return Err(BareError::HoldProbeTooLong {
            asked,
            most: HOLD_PROBE_MAX_SECONDS,
        });
    }
    let (row, id) = bare_target(map, request.target, HOLD_PROBE_JOINT)?;
    let mut bus = Bus::new(port, timing);

    line(&format!(
        "hold-probe: servo {id} is torqued at the position it already holds and read as fast as \
         the bus answers, for {seconds:?} with the goal rewritten every {COMMAND_PERIOD:?} and \
         {seconds:?} with reads alone. It is commanded nowhere — the only goal written is the \
         count it reports for itself — but it does hold torque for the whole of both phases. \
         Stay clear of it, and do not run this with the head up.",
        seconds = request.seconds,
    ));

    let info =
        with_retry(&mut bus, |bus| bus.ping(id)).map_err(|source| BareError::Bus { id, source })?;
    if read_byte(&mut bus, map, row, RegId::TorqueEnable)? != 0 {
        return Err(BareError::HoldProbeTorqueHeld { id });
    }
    // The second phase writes no goal for seconds, which is what the Bus
    // Watchdog stops a servo for. A servo that arrived with it armed would be
    // measured while a timer was halting it, and the quiet phase that came back
    // would read as the answer this probe exists to find.
    let watchdog = read_byte(&mut bus, map, row, RegId::BusWatchdog)?;
    if watchdog != 0 {
        return Err(BareError::HoldProbeWatchdogArmed { id, read: watchdog });
    }
    line(&format!(
        "  servo {id}: model {model}, torque off, bus watchdog disarmed",
        model = info.model
    ));

    // Read before anything is written, so a probe that cannot read the gains
    // has written none and has nothing to put back.
    let kept = match request.gains {
        Some(_) => Some(read_gains(&mut bus, map, row)?),
        None => None,
    };

    let mut probe = HeldServo {
        bus: &mut bus,
        map,
        row,
        request,
    };
    let mut phases = Vec::new();
    let exercised = probe.exercise(clock, line, &mut phases);
    let restored = probe.make_safe(kept, line);
    // Read, printed and judged after the release: which of the two phases a
    // joint wobbles in is the reading, so a refusal taken at the first would
    // leave the question that separates them unasked — and the arithmetic that
    // reads a series walks it once per lag, which is not something to spend
    // with a servo still holding torque.
    for phase in &phases {
        line(&format!("  {}: {}", phase.kind, phase.stats()));
    }
    let judged = judge_phases(id, &phases);
    let outcome = match (exercised, restored) {
        (Err(failed), Err(release)) => {
            // The failure is the discovery and is what the caller acts on; a
            // make-safe that did not finish may be a servo still holding
            // torque, which an operator has to act on before reading anything,
            // so it is said out loud first.
            line(&format!(
                "  cleanup did not finish, so the state of this servo is unknown: {release}"
            ));
            Err(failed)
        }
        (Err(failed), Ok(())) => Err(failed),
        (Ok(()), Err(release)) => Err(release),
        (Ok(()), Ok(())) => judged,
    };
    Ok(ProbeRun {
        id,
        phases,
        outcome,
    })
}

/// The probe's bring-up assertion: a held servo stays inside the stillness
/// watch's bound, in both phases.
///
/// Read off the figures each phase carries, so nothing here touches the bus.
/// The figures are the ones the per-phase lines printed, read once and kept.
fn judge_phases(id: u8, phases: &[ProbePhase]) -> Result<(), BareError> {
    for phase in phases {
        if phase.stats().excursion > HOLD_PROBE_BOUND_COUNTS {
            return Err(BareError::HoldProbeExcursion {
                id,
                phase: phase.kind,
                excursion: phase.stats().excursion,
                bound: HOLD_PROBE_BOUND_COUNTS,
                period: phase.stats().period,
            });
        }
    }
    Ok(())
}

/// One probe's wiring: the bus, the servo it is addressing, and what was asked
/// for. Held together so the phases are methods over one held servo rather than
/// five arguments repeated at every step.
struct HeldServo<'a, P: BusPort> {
    bus: &'a mut Bus<P>,
    map: &'a ServoMap,
    row: usize,
    request: ProbeRequest,
}

impl<P: BusPort> HeldServo<'_, P> {
    /// Every write the probe makes and every assertion it draws, inside the
    /// boundary the make-safe covers.
    ///
    /// The phases are pushed as they finish — a phase cut short by a bus
    /// failure is pushed with what it got — so the caller's record is whatever
    /// the run reached.
    fn exercise(
        &mut self,
        clock: &mut dyn Clock,
        line: &mut dyn FnMut(&str),
        phases: &mut Vec<ProbePhase>,
    ) -> Result<(), BareError> {
        if let Some(asked) = self.request.gains {
            write_value(
                self.bus,
                self.map,
                self.row,
                RegId::PositionGains,
                asked.value(),
            )?;
            line(&format!("  gains: {asked}, written and read back"));
        }

        let held = hold_where_it_stands(self.bus, self.map, self.row)?;
        line("  holding: torque on at the position it was resting at");

        for kind in [Busy::ReadsAndGoals, Busy::ReadsOnly] {
            self.phase(kind, &held, clock, phases)?;
        }
        Ok(())
    }

    /// One phase: read the servo's position as fast as the bus answers for the
    /// requested length, rewriting the held goal every command period if this
    /// phase is the one that does.
    fn phase(
        &mut self,
        kind: Busy,
        held: &RawValue,
        clock: &mut dyn Clock,
        phases: &mut Vec<ProbePhase>,
    ) -> Result<(), BareError> {
        let started = clock.now();
        let until = started + self.request.seconds;
        let mut next_goal = started;
        // Sized before the loop starts: a vector growing from nothing
        // reallocates and copies inside the tightest timing loop in this crate,
        // and this loop's whole purpose is that its stamps mean something. The
        // bound is the wire's ceiling, not its expected rate — a read costs
        // hundreds of microseconds at this baud, so nothing can beat it.
        let mut samples: Vec<ProbeSample> = Vec::with_capacity(
            HOLD_PROBE_RATE_CEILING_HZ * self.request.seconds.as_secs() as usize,
        );

        while clock.now() < until {
            if matches!(kind, Busy::ReadsAndGoals) && clock.now() >= next_goal {
                // The goal that was held, not the count just read: a rewrite of
                // wherever the servo has got to is a setpoint following the
                // joint around, which would hide the very wobble this is
                // looking for.
                if let Err(failed) =
                    write_verified(self.bus, self.map, self.row, RegId::GoalPosition, held)
                {
                    phases.push(ProbePhase::new(kind, samples));
                    return Err(failed);
                }
                next_goal = clock.now() + COMMAND_PERIOD;
            }
            match read_raw(self.bus, self.map, self.row, RegId::PresentPosition) {
                Ok(raw) => samples.push(ProbeSample {
                    at: clock.now() - started,
                    counts: raw.i32().expect("a position register is four bytes wide"),
                }),
                Err(failed) => {
                    phases.push(ProbePhase::new(kind, samples));
                    return Err(failed);
                }
            }
        }

        // Read and printed by the caller after the release: reading a series
        // costs a walk of it per lag, and spending that here would be seconds
        // of arithmetic between the two phases and again before the torque
        // comes off.
        phases.push(ProbePhase::new(kind, samples));
        Ok(())
    }

    /// Put the gains back and take the torque off, whatever the phases said.
    ///
    /// Both are attempted whichever fails: the gains are RAM a power cycle
    /// clears, and the torque is the one thing an operator would otherwise have
    /// to walk up to the machine for. When both fail the returned error is the
    /// torque one, for that reason, and the gains' failure is printed beside it.
    fn make_safe(
        &mut self,
        kept: Option<Gains>,
        line: &mut dyn FnMut(&str),
    ) -> Result<(), BareError> {
        let restored = match kept {
            Some(gains) => write_value(
                self.bus,
                self.map,
                self.row,
                RegId::PositionGains,
                gains.value(),
            ),
            None => Ok(()),
        };
        let released = write_byte(self.bus, self.map, self.row, RegId::TorqueEnable, 0);
        match (restored, released) {
            (Ok(()), Ok(())) => {
                match kept {
                    Some(gains) => line(&format!("  released, and the gains are back at {gains}")),
                    None => line("  released, and the gains were never touched"),
                }
                Ok(())
            }
            (Err(gains), Ok(())) => {
                line(
                    "  released, but the gains would not go back; they are RAM and a power cycle \
                     clears them",
                );
                Err(gains)
            }
            (Ok(()), Err(release)) => Err(release),
            (Err(gains), Err(release)) => {
                line(&format!("  the gains would not go back either: {gains}"));
                Err(release)
            }
        }
    }
}

/// One servo's three position gains.
fn read_gains<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
) -> Result<Gains, BareError> {
    let value = read_value(bus, map, row, RegId::PositionGains)?;
    let (p, i, d) = value.as_gains().ok_or(BareError::Map {
        id: map.ids()[row],
        reg: RegId::PositionGains,
        source: MapError::WrongShape {
            reg: RegId::PositionGains,
            expected: ValueShape::Gains,
            observed: value.shape(),
        },
    })?;
    Ok(Gains { p, i, d })
}

/// Write where the servo stands as its goal, then enable torque, and answer
/// with the goal that was written.
///
/// Both halves together, because either alone is the hazard: a goal written to a
/// servo that is about to hold it is only safe if it is where the servo already
/// is, and torque enabled without it is a servo commanded to whatever its goal
/// register happens to hold.
///
/// The count comes back because a caller that goes on rewriting the goal has to
/// rewrite *this* one: a rewrite of whatever the servo reports at that instant
/// is a goal that follows the servo around, which is a moving setpoint dressed
/// as a hold.
fn hold_where_it_stands<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
) -> Result<RawValue, BareError> {
    let at = read_raw(bus, map, row, RegId::PresentPosition)?;
    write_verified(bus, map, row, RegId::GoalPosition, &at)?;
    write_byte(bus, map, row, RegId::TorqueEnable, 1)?;
    Ok(at)
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
    write_value(bus, map, row, reg, value::u8(byte))
}

/// Write one register as the value it carries, with the read-back the write
/// path does itself.
///
/// The encoding is the map's, so a value of the wrong shape for the register is
/// refused here rather than put on the wire.
fn write_value<P: BusPort>(
    bus: &mut Bus<P>,
    map: &ServoMap,
    row: usize,
    reg: RegId,
    value: Value,
) -> Result<(), BareError> {
    let id = map.ids()[row];
    let raw = map
        .encode_value(row, reg, value)
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

    // -----------------------------------------------------------------------
    // The hold probe
    // -----------------------------------------------------------------------

    /// What one frame costs the scripted machine's clock. A millisecond, so a
    /// phase's readings land at a rate of the order the real loop achieves and
    /// the figures a case asserts are figures of the same shape.
    const FRAME: Duration = Duration::from_millis(1);

    /// A phase long enough for a series worth reading — sixty frames or so at
    /// the cost above — and short enough that a case is instant.
    const PHASE: Duration = Duration::from_millis(60);

    /// A probe run: the transcript, and the readings the command handed back.
    struct Probed {
        run: Run,
        probe: Option<ProbeRun>,
    }

    impl Probed {
        /// The readings, or a panic naming what was expected instead.
        fn probe(&self) -> &ProbeRun {
            self.probe
                .as_ref()
                .expect("the probe reached its phases and handed them back")
        }

        /// The phase that ran under `kind`.
        fn phase(&self, kind: Busy) -> &ProbePhase {
            self.probe()
                .phases
                .iter()
                .find(|phase| phase.kind == kind)
                .unwrap_or_else(|| panic!("no {kind} phase"))
        }
    }

    /// Run `hold-probe` against `machine`, on a clock the machine spends.
    ///
    /// The run's outcome is the probe's own, so every assertion the other
    /// commands' cases make about a transcript works here unchanged; the
    /// readings come back beside it because they outlive the verdict.
    fn probed(
        mut machine: FakeMachine,
        cfg: &Configured,
        target: Option<u8>,
        gains: Option<Gains>,
    ) -> Probed {
        let now = Rc::new(Cell::new(Duration::ZERO));
        machine.spends_time(&now, FRAME);
        let captured: Rc<RefCell<Option<ProbeRun>>> = Rc::new(RefCell::new(None));
        let held = Rc::clone(&captured);
        let run = run_at(machine, &now, |port, clock, line| {
            let request = ProbeRequest {
                target,
                gains,
                seconds: PHASE,
            };
            match hold_probe(&cfg.map, cfg.timing, port, request, clock, line) {
                Ok(mut probe) => {
                    let outcome = core::mem::replace(&mut probe.outcome, Ok(()));
                    *held.borrow_mut() = Some(probe);
                    outcome
                }
                Err(refused) => Err(refused),
            }
        });
        let probe = captured.borrow_mut().take();
        Probed { run, probe }
    }

    /// Run `hold-probe` against `machine`, with `id` acknowledging writes to
    /// `regs` and storing none of them from the moment the transcript reaches a
    /// line starting with `after`.
    ///
    /// The probe's own [`run_deaf_from`], with a register that takes a write
    /// and drops it rather than one that stops answering: the failure a
    /// verified write then meets names the register it was for, so a case about
    /// two cleanup writes can say which of them the run reported. A case names
    /// the moment in the command's own transcript rather than counting the
    /// transactions the phases before it happened to take.
    fn probed_dropping_from(
        mut machine: FakeMachine,
        cfg: &Configured,
        id: u8,
        after: &str,
        regs: &[RegId],
        gains: Option<Gains>,
    ) -> Probed {
        let now = Rc::new(Cell::new(Duration::ZERO));
        machine.spends_time(&now, FRAME);
        let addrs: Vec<u16> = regs.iter().map(|reg| named_reg(*reg).addr).collect();
        let after = after.to_string();
        let captured: Rc<RefCell<Option<ProbeRun>>> = Rc::new(RefCell::new(None));
        let held = Rc::clone(&captured);
        let run = run_at(machine, &now, |port, clock, line| {
            let machine = port.machine();
            let mut watched = |text: &str| {
                if text.trim_start().starts_with(&after) {
                    let mut machine = machine.borrow_mut();
                    for addr in &addrs {
                        machine.ignored.push((id, *addr));
                    }
                }
                line(text);
            };
            let request = ProbeRequest {
                target: None,
                gains,
                seconds: PHASE,
            };
            match hold_probe(&cfg.map, cfg.timing, port, request, clock, &mut watched) {
                Ok(mut probe) => {
                    let outcome = core::mem::replace(&mut probe.outcome, Ok(()));
                    *held.borrow_mut() = Some(probe);
                    outcome
                }
                Err(refused) => Err(refused),
            }
        });
        let probe = captured.borrow_mut().take();
        Probed { run, probe }
    }

    /// The servo a probe addresses when the operator names none.
    fn probe_id(cfg: &Configured) -> u8 {
        cfg.map.ids()[row(HOLD_PROBE_JOINT).expect("a named joint has a bus row")]
    }

    /// A servo that stands exactly still passes both phases, is left limp, and
    /// was never commanded anywhere but the count it was already at.
    #[test]
    fn a_servo_that_holds_still_passes_both_phases() {
        let cfg = resolved();
        let id = probe_id(&cfg);
        let machine = machine_at(&example_config(), &stow_legs());
        let stood = i32::from_le_bytes(
            machine
                .get(id, named_reg(RegId::PresentPosition))
                .expect("the fixture stands somewhere")
                .try_into()
                .expect("a position register is four bytes wide"),
        );

        let probed = probed(machine, &cfg, None, None);
        probed.run.ok("a still servo passes");

        assert_eq!(probed.probe().id, id);
        assert_eq!(probed.probe().phases.len(), 2);
        for kind in [Busy::ReadsAndGoals, Busy::ReadsOnly] {
            let stats = *probed.phase(kind).stats();
            assert!(stats.excursion.abs() < 1e-9, "{kind} moved a still servo");
            assert!(stats.samples > HOLD_PROBE_MIN_SERIES, "{kind}: {stats}");
            // The rate is the loop's own, measured off the readings: one frame
            // per read at the cost this fixture charges.
            assert!(
                (stats.rate_hz - 1000.0 / FRAME.as_secs_f64() / 1000.0).abs() < 200.0,
                "{kind}: {stats}"
            );
            assert!(
                stats.period.is_none(),
                "a flat series has no period: {stats}"
            );
        }

        let machine = probed.run.registers.borrow();
        assert_eq!(
            machine.get(id, named_reg(RegId::TorqueEnable)),
            Some(&[0][..])
        );
        assert_eq!(
            machine.get(id, named_reg(RegId::GoalPosition)),
            Some(&stood.to_le_bytes()[..])
        );
        drop(machine);
        assert!(
            probed
                .run
                .printed
                .iter()
                .any(|line| line.contains("no period") && line.contains("peak to peak 0 counts")),
            "{:?}",
            probed.run.printed
        );
        assert!(
            probed
                .run
                .printed
                .iter()
                .any(|line| line.contains("the gains were never touched")),
            "{:?}",
            probed.run.printed
        );
    }

    /// A servo wobbling on a four-sample cycle fails the bound, and the figures
    /// beside the refusal are the discovery: the period is the cycle's, read at
    /// the loop's own rate rather than at an assumed one.
    #[test]
    fn a_four_sample_wobble_fails_with_its_own_period() {
        let cfg = resolved();
        let id = probe_id(&cfg);
        let mut machine = machine_at(&example_config(), &stow_legs());
        let stood = i32::from_le_bytes(
            machine
                .get(id, named_reg(RegId::PresentPosition))
                .expect("the fixture stands somewhere")
                .try_into()
                .expect("a position register is four bytes wide"),
        );
        machine.wobbles(id, &[stood, stood + 3, stood + 6, stood + 3]);

        let probed = probed(machine, &cfg, None, None);
        let error = probed.run.err("a servo that moved 6 counts is not still");
        let BareError::HoldProbeExcursion {
            id: named,
            phase,
            excursion,
            bound,
            period,
        } = error
        else {
            panic!("expected an excursion, got {error}");
        };
        assert_eq!(*named, id);
        // The first phase is the one judged: which phase a wobble is in is the
        // reading, and this one is in both.
        assert_eq!(*phase, Busy::ReadsAndGoals);
        assert!((*excursion - 6.0).abs() < 1e-9, "{excursion}");
        assert!((*bound - 2.0).abs() < 1e-9, "{bound}");
        assert_eq!(
            period.expect("a four-sample cycle has a period").lag,
            4,
            "{}",
            Shown(*period)
        );
        // The operator reads the rendered line, not the fields.
        let shown = error.to_string();
        assert!(
            shown.starts_with(&format!(
                "servo {id} moved 6 counts while held during reads and goal rewrites, past the 2.0 count bound"
            )),
            "{shown}"
        );
        assert!(!shown.contains("  "), "{shown}");

        let stats = *probed.phase(Busy::ReadsOnly).stats();
        let read = stats.period.expect("a four-sample cycle has a period");
        assert_eq!(read.lag, 4);
        assert!(read.regularity > 0.5, "{stats}");
        // Four samples at a millisecond each: 4 ms, 250 Hz — figures the loop's
        // measured rate produces, not the driver's 50 Hz grid.
        assert!((read.millis - 4.0).abs() < 1.0, "{stats}");
        assert!((read.hz - 250.0).abs() < 60.0, "{stats}");
        // Two reversals per cycle, at a quarter of a millisecond-per-sample
        // loop: around 500 a second.
        assert!(stats.reversals_per_s > 300.0, "{stats}");

        // Both phases' readings survive the refusal, as the table a fixture is
        // cut from.
        let csv = probed.probe().csv();
        assert!(csv.starts_with("phase,elapsed_ms,counts\n"), "{csv}");
        assert_eq!(
            csv.lines().count() - 1,
            probed
                .probe()
                .phases
                .iter()
                .map(|phase| phase.samples.len())
                .sum::<usize>()
        );
        assert!(csv.contains("reads alone,"), "{csv}");

        // Every goal the rewriting phase wrote is the one count the servo
        // stood at before torque went on — the invariant the instrument rests
        // on, asserted here rather than on a still servo, because a goal
        // following the joint around is only visible on one that moves. The
        // whole run of writes, not the register's last value: a rewrite loop
        // is judged on all of them.
        let machine = probed.run.registers.borrow();
        let goals: Vec<i32> = machine
            .written
            .iter()
            .filter(|(who, addr, _)| *who == id && *addr == named_reg(RegId::GoalPosition).addr)
            .map(|(_, _, bytes)| {
                i32::from_le_bytes(bytes.as_slice().try_into().expect("four bytes of goal"))
            })
            .collect();
        assert!(goals.len() > 1, "the phase rewrote nothing: {goals:?}");
        assert!(
            goals.iter().all(|goal| *goal == stood),
            "a goal followed the joint: {goals:?}, standing at {stood}"
        );
        for moved in [stood + 3, stood + 6] {
            assert!(
                !goals.contains(&moved),
                "a wobble sample was written back as a goal: {goals:?}"
            );
        }
        // The refusal is still a released servo.
        assert_eq!(
            machine.get(id, named_reg(RegId::TorqueEnable)),
            Some(&[0][..])
        );
    }

    /// A servo that stops answering part way through a phase takes its gains
    /// back and its torque off anyway, and the readings it did give come back.
    #[test]
    fn a_read_failure_mid_phase_still_restores_the_gains_and_releases() {
        let cfg = resolved();
        let id = probe_id(&cfg);
        let kept = Gains {
            p: 500,
            i: 0,
            d: 100,
        };
        let asked = Gains { p: 200, i: 0, d: 0 };
        let mut machine = machine_at(&example_config(), &stow_legs());
        let row = row(HOLD_PROBE_JOINT).expect("a named joint has a bus row");
        let raw = cfg
            .map
            .encode_value(row, RegId::PositionGains, kept.value())
            .expect("the gains encode");
        machine.set(id, named_reg(RegId::PositionGains), raw.as_slice());
        // Answers the hold's own read and twenty of the phase's, then goes.
        machine
            .deafens_after
            .insert((id, named_reg(RegId::PresentPosition).addr), 21);

        let probed = probed(machine, &cfg, None, Some(asked));
        let error = probed
            .run
            .err("a servo that stopped answering is not a reading");
        let BareError::BusRead { id: named, reg, .. } = error else {
            panic!("expected a read failure, got {error}");
        };
        assert_eq!((*named, *reg), (id, RegId::PresentPosition));

        // The phase it died in is here with what it got, and the second never
        // ran.
        assert_eq!(probed.probe().phases.len(), 1);
        assert_eq!(probed.phase(Busy::ReadsAndGoals).samples.len(), 20);

        let machine = probed.run.registers.borrow();
        assert_eq!(
            machine.get(id, named_reg(RegId::TorqueEnable)),
            Some(&[0][..])
        );
        assert_eq!(
            machine.get(id, named_reg(RegId::PositionGains)),
            Some(raw.as_slice())
        );
        drop(machine);
        assert!(
            probed
                .run
                .printed
                .iter()
                .any(|line| line.contains("the gains are back at P 500 I 0 D 100")),
            "{:?}",
            probed.run.printed
        );
        assert!(
            probed
                .run
                .printed
                .iter()
                .any(|line| line.contains("gains: P 200 I 0 D 0")),
            "{:?}",
            probed.run.printed
        );
    }

    /// A servo already holding torque is refused before anything is written:
    /// what it is standing in is somebody else's hold, and the probe would be
    /// measuring that.
    #[test]
    fn a_probe_refuses_a_servo_that_is_already_holding() {
        let cfg = resolved();
        let id = probe_id(&cfg);
        let mut machine = machine_at(&example_config(), &stow_legs());
        machine.set(id, named_reg(RegId::TorqueEnable), &[1]);

        let probed = probed(machine, &cfg, None, None);
        let error = probed.run.err("a servo holding torque is refused");
        let BareError::HoldProbeTorqueHeld { id: named } = error else {
            panic!("expected a held servo, got {error}");
        };
        assert_eq!(*named, id);
        assert!(probed.probe.is_none(), "a refused probe has no readings");
        probed.run.commanded_nothing(&cfg);
    }

    /// A servo whose Bus Watchdog is armed is refused before anything is
    /// written. The reads-only phase is seconds of exactly the silence that
    /// register stops a servo for, so a probe run in that state would report a
    /// watchdog's halt as the answer the two phases exist to separate.
    #[test]
    fn a_probe_refuses_a_servo_whose_bus_watchdog_is_armed() {
        for armed in [WATCHDOG_COUNTS, WATCHDOG_LATCHED] {
            let cfg = resolved();
            let id = probe_id(&cfg);
            let mut machine = machine_at(&example_config(), &stow_legs());
            machine.set(id, named_reg(RegId::BusWatchdog), &[armed]);

            let probed = probed(machine, &cfg, None, None);
            let error = probed.run.err("an armed watchdog is refused");
            let BareError::HoldProbeWatchdogArmed { id: named, read } = error else {
                panic!("expected an armed watchdog, got {error}");
            };
            assert_eq!((*named, *read), (id, armed));
            assert!(probed.probe.is_none(), "a refused probe has no readings");
            probed.run.commanded_nothing(&cfg);
            let shown = error.to_string();
            assert!(
                shown.contains(&format!("bus watchdog at {armed}")),
                "{shown}"
            );
            // The remedy has to be one that works. `off` writes TorqueEnable
            // and nothing else, so an operator who followed it would come
            // straight back to this refusal.
            assert!(shown.contains("does not clear this"), "{shown}");
            // And one that works on *this* servo: both commands default to the
            // antenna the watchdog self-test addresses, so a remedy naming
            // neither servo sends an operator probing another joint to exercise
            // the antenna and come back to this same refusal.
            assert!(shown.contains(&format!("`reboot {id}`")), "{shown}");
            assert!(shown.contains(&format!("run `watchdog {id}`")), "{shown}");
        }
    }

    /// A phase longer than an attended hold is refused before the bus is
    /// touched, and a servo off the roster is refused by ID as everywhere else.
    #[test]
    fn a_probe_refuses_an_unattendable_phase_and_a_servo_off_the_roster() {
        let cfg = resolved();
        let machine = machine_at(&example_config(), &stow_legs());
        let too_long = Duration::from_secs(HOLD_PROBE_MAX_SECONDS + 1);
        let outcome = hold_probe(
            &cfg.map,
            cfg.timing,
            Spy::new(machine),
            ProbeRequest {
                target: None,
                gains: None,
                seconds: too_long,
            },
            &mut TestClock::sharing(&Rc::new(Cell::new(Duration::ZERO))),
            &mut |_| {},
        );
        let Err(BareError::HoldProbeTooLong { asked, most }) = outcome else {
            panic!("a phase of a minute and one second is not attended");
        };
        assert_eq!(
            (asked, most),
            (HOLD_PROBE_MAX_SECONDS + 1, HOLD_PROBE_MAX_SECONDS)
        );
        // The operator reads the rendered line, not the fields.
        let shown = BareError::HoldProbeTooLong { asked, most }.to_string();
        assert_eq!(
            shown,
            format!(
                "a hold probe phase of {asked} s is longer than the {most} s this command holds a servo for"
            )
        );

        let probed = probed(
            machine_at(&example_config(), &stow_legs()),
            &cfg,
            Some(99),
            None,
        );
        let error = probed.run.err("a servo nobody configured is refused");
        let BareError::OffRoster { id, .. } = error else {
            panic!("expected an off-roster refusal, got {error}");
        };
        assert_eq!(*id, 99);
    }

    /// The dominant period is read off the shape of the series and not off the
    /// count of its reversals: a drift, an alternation and a longer cycle read
    /// as three different things at the same reversal rate.
    #[test]
    fn a_period_is_the_series_shape_rather_than_its_reversal_count() {
        let at = |k: usize| Duration::from_millis(k as u64);
        let phase = |counts: &[i32]| {
            ProbePhase::new(
                Busy::ReadsOnly,
                counts
                    .iter()
                    .enumerate()
                    .map(|(k, counts)| ProbeSample {
                        at: at(k),
                        counts: *counts,
                    })
                    .collect(),
            )
        };

        // A joint alternating every sample: period two, and the reversal rate
        // says so too.
        let flicker: Vec<i32> = (0..64).map(|k| i32::from(k % 2 == 0)).collect();
        let stats = *phase(&flicker).stats();
        assert_eq!(stats.period.expect("an alternation has a period").lag, 2);
        assert!((stats.excursion - 1.0).abs() < 1e-9, "{stats}");
        // The intervals say the same thing the correlation does, which is what
        // an alternation looks like: every reversal one sample after the last.
        assert_eq!(stats.reversal_interval_mean_samples, Some(1.0));
        assert_eq!(stats.reversal_interval_spread_samples, Some(0.0));

        // A joint drifting one way: the same series length, no cycle at all.
        let drift: Vec<i32> = (0..64).collect();
        let stats = *phase(&drift).stats();
        assert!(stats.period.is_none(), "a ramp is not a cycle: {stats}");
        assert!(stats.reversals_per_s < f64::EPSILON, "{stats}");

        // A joint that never moved: no period, no reversals, no division by a
        // series with no energy in it.
        let stats = *phase(&[7; 64]).stats();
        assert!(stats.period.is_none(), "{stats}");
        assert!(stats.excursion.abs() < 1e-9, "{stats}");
        assert_eq!(stats.reversal_interval_mean_samples, None);

        // A series too short to search is not a series with a period.
        let stats = *phase(&flicker[..HOLD_PROBE_MIN_SERIES]).stats();
        assert!(stats.period.is_none(), "{stats}");
    }

    /// A phase built out of a series, for the cases that judge the figures
    /// rather than a run: one reading a millisecond, which is the order the
    /// loop achieves on the wire.
    fn phase_of(kind: Busy, counts: &[i32]) -> ProbePhase {
        ProbePhase::new(
            kind,
            counts
                .iter()
                .enumerate()
                .map(|(k, counts)| ProbeSample {
                    at: Duration::from_millis(k as u64),
                    counts: *counts,
                })
                .collect(),
        )
    }

    /// The regularity tells a limit cycle from encoder dither, which is the
    /// whole reading the gains ladder branches on.
    ///
    /// A clean cycle correlates strongly at its own lag; a series that turns
    /// round as often but at scattered intervals correlates at none of them and
    /// says so twice — a low regularity, and an interval spread of the order of
    /// its own mean.
    #[test]
    fn a_scattered_series_reads_as_dither_and_a_clean_one_as_a_cycle() {
        let cycle: Vec<i32> = (0..256).map(|k| [0, 3, 6, 3][k % 4]).collect();
        let cycle = *phase_of(Busy::ReadsOnly, &cycle).stats();
        let clean = cycle.period.expect("a four-sample cycle has a period");

        // The right antenna's own shape from the run this instrument was built
        // for: one count, turning round as often as the cycle does, at
        // intervals of one sample to several and repeating nothing. A fixed
        // sequence, so what this asserts is the same figure every run.
        let mut state: u32 = 0x2545_f491;
        let dither: Vec<i32> = (0..256)
            .map(|_| {
                state = state.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
                -i32::try_from((state >> 30) & 1).expect("one bit is a count")
            })
            .collect();
        let dither = *phase_of(Busy::ReadsOnly, &dither).stats();

        let scattered = dither
            .period
            .map_or(0.0, |period: ProbePeriod| period.regularity);
        assert!(
            scattered < clean.regularity / 2.0,
            "dither read as regular as a cycle: {dither} against {cycle}"
        );
        let (mean, spread) = (
            dither
                .reversal_interval_mean_samples
                .expect("dither turns round"),
            dither
                .reversal_interval_spread_samples
                .expect("dither turns round"),
        );
        assert!(spread > mean / 2.0, "{dither}");
        assert_eq!(cycle.reversal_interval_spread_samples, Some(0.0), "{cycle}");
    }

    /// Lag one is not a period.
    ///
    /// A series whose differenced form correlates most strongly at one sample
    /// is a joint travelling, not one turning round: the reported lag is the
    /// shortest cycle a series can carry, which is two.
    #[test]
    fn a_series_correlated_at_one_sample_is_not_reported_at_lag_one() {
        // Eight counts up, eight down: every step but two is the step before
        // it, so lag one is the strongest correlation in the differenced
        // series and the cycle is sixteen.
        let triangle: Vec<i32> = (0..128)
            .map(|k| {
                let phase = k % 16;
                if phase < 8 { phase } else { 16 - phase }
            })
            .collect();
        let stats = *phase_of(Busy::ReadsOnly, &triangle).stats();
        let period = stats.period.expect("a triangle wave has a period");
        assert!(period.lag >= 2, "lag one was reported: {stats}");
        assert_eq!(period.lag % 16, 0, "{stats}");
    }

    /// The phase a wobble is in is the phase the refusal names.
    ///
    /// Which of the two a joint moves in is the reading the whole two-phase
    /// design exists to take: a rewrite-driven wobble is answered by the
    /// driver and a wobble under reads alone is answered by a gain, so a
    /// refusal that named the wrong phase would send the campaign down the
    /// wrong branch.
    #[test]
    fn a_wobble_in_the_reads_only_phase_is_refused_as_that_phase() {
        let flat = phase_of(Busy::ReadsAndGoals, &[7; 64]);
        let wobbling: Vec<i32> = (0..64).map(|k| [0, 3, 6, 3][k % 4]).collect();
        let phases = vec![flat, phase_of(Busy::ReadsOnly, &wobbling)];

        let Err(BareError::HoldProbeExcursion {
            id,
            phase,
            excursion,
            ..
        }) = judge_phases(18, &phases)
        else {
            panic!("six counts under reads alone is not a still servo");
        };
        assert_eq!((id, phase), (18, Busy::ReadsOnly));
        assert!((excursion - 6.0).abs() < 1e-9, "{excursion}");

        // And a quiet pair is a quiet pair, whichever way round it is read.
        judge_phases(18, &[phase_of(Busy::ReadsOnly, &[7; 64])]).expect("a flat hold passes");
    }

    /// The bound is the stillness watch's, and the comparison is at it: a hold
    /// at exactly two counts passes, and one at three does not.
    ///
    /// The instrument and the session judge a hold by one figure, so the place
    /// the two could disagree is the place the bound is crossed.
    #[test]
    fn the_bound_is_crossed_at_the_count_past_it() {
        for (counts, still) in [(2, true), (3, false)] {
            let cfg = resolved();
            let id = probe_id(&cfg);
            let mut machine = machine_at(&example_config(), &stow_legs());
            let stood = i32::from_le_bytes(
                machine
                    .get(id, named_reg(RegId::PresentPosition))
                    .expect("the fixture stands somewhere")
                    .try_into()
                    .expect("a position register is four bytes wide"),
            );
            machine.wobbles(id, &[stood, stood + counts]);

            let probed = probed(machine, &cfg, None, None);
            let read = *probed.phase(Busy::ReadsAndGoals).stats();
            assert!((read.excursion - f64::from(counts)).abs() < 1e-9, "{read}");
            if still {
                probed
                    .run
                    .ok("a hold at exactly the bound is a hold inside it");
            } else {
                let error = probed.run.err("a count past the bound is not still");
                assert!(
                    matches!(error, BareError::HoldProbeExcursion { .. }),
                    "expected an excursion, got {error}"
                );
            }
        }
    }

    /// The gains not going back is reported and returned, and the torque still
    /// comes off.
    #[test]
    fn gains_that_will_not_go_back_are_reported_and_the_servo_is_still_released() {
        let cfg = resolved();
        let id = probe_id(&cfg);
        let machine = machine_at(&example_config(), &stow_legs());
        let asked = Gains { p: 200, i: 0, d: 0 };
        // The gains register takes writes and stores none from the moment the
        // servo is holding, so the swap landed and the write-back reads the
        // triple that was already there.
        let probed = probed_dropping_from(
            machine,
            &cfg,
            id,
            "holding:",
            &[RegId::PositionGains],
            Some(asked),
        );

        let error = probed.run.err("the gains could not be put back");
        let BareError::Bus {
            source: XactError::VerifyMismatch { addr, .. },
            ..
        } = error
        else {
            panic!("expected the write-back to read back the wrong triple, got {error}");
        };
        assert_eq!(*addr, named_reg(RegId::PositionGains).addr);
        assert!(
            probed
                .run
                .printed
                .iter()
                .any(|line| line.contains("the gains would not go back")),
            "{:?}",
            probed.run.printed
        );
        assert_eq!(
            probed
                .run
                .registers
                .borrow()
                .get(id, named_reg(RegId::TorqueEnable)),
            Some(&[0][..]),
            "the torque came off anyway",
        );
    }

    /// A release that could not be read back is what the run returns, whatever
    /// else went right.
    ///
    /// Nothing gates de-torquing and nothing outranks its failure: a servo that
    /// may still be holding is the one state an operator has to act on.
    #[test]
    fn a_release_that_did_not_read_back_is_the_run_s_answer() {
        let cfg = resolved();
        let id = probe_id(&cfg);
        let machine = machine_at(&example_config(), &stow_legs());
        let probed =
            probed_dropping_from(machine, &cfg, id, "holding:", &[RegId::TorqueEnable], None);

        let error = probed.run.err("the release could not be read back");
        let BareError::Bus {
            source: XactError::VerifyMismatch { addr, .. },
            ..
        } = error
        else {
            panic!("expected the release to read back as still holding, got {error}");
        };
        assert_eq!(*addr, named_reg(RegId::TorqueEnable).addr);
        assert_eq!(probed.probe().phases.len(), 2, "both phases still ran");
    }

    /// Both cleanup writes failing returns the release's failure and prints the
    /// gains' beside it; a phase that also failed adds the line that says the
    /// servo's state is unknown.
    #[test]
    fn a_cleanup_that_failed_twice_returns_the_release_and_says_both() {
        let cfg = resolved();
        let id = probe_id(&cfg);
        let machine = machine_at(&example_config(), &stow_legs());
        let asked = Gains { p: 200, i: 0, d: 0 };
        let probed = probed_dropping_from(
            machine,
            &cfg,
            id,
            "holding:",
            &[RegId::PositionGains, RegId::TorqueEnable],
            Some(asked),
        );
        let error = probed.run.err("neither cleanup write read back");
        let BareError::Bus {
            source: XactError::VerifyMismatch { addr, .. },
            ..
        } = error
        else {
            panic!("expected a write that did not read back, got {error}");
        };
        assert_eq!(
            *addr,
            named_reg(RegId::TorqueEnable).addr,
            "the release's failure is the one that comes out, not the gains'",
        );
        assert!(
            probed
                .run
                .printed
                .iter()
                .any(|line| line.contains("the gains would not go back either")),
            "{:?}",
            probed.run.printed
        );

        // And with a phase failing too: the failure the run returns is the
        // phase's, and the cleanup that did not finish is said out loud first.
        let cfg = resolved();
        let mut machine = machine_at(&example_config(), &stow_legs());
        machine
            .deafens_after
            .insert((id, named_reg(RegId::PresentPosition).addr), 21);
        let probed =
            probed_dropping_from(machine, &cfg, id, "holding:", &[RegId::TorqueEnable], None);
        let error = probed
            .run
            .err("a servo that stopped answering is not a reading");
        assert!(
            matches!(
                error,
                BareError::BusRead {
                    reg: RegId::PresentPosition,
                    ..
                }
            ),
            "the phase's failure is the discovery, got {error}"
        );
        assert!(
            probed.run.printed.iter().any(|line| line
                .contains("cleanup did not finish, so the state of this servo is unknown")),
            "{:?}",
            probed.run.printed
        );
    }
}
