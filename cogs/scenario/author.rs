//! Phase one of the harness: turn a scenario's statement into an input log.
//!
//! The two channels the deterministic runner publishes from a log are the ones
//! nothing in this system produces -- the scripts somebody asks the machine to
//! run, and the scenario's hand on the plant. Everything else in the run follows
//! from them: the session screens a script, commissions and arms the machine over
//! the bus, and publishes the schedule the decision tick streams.
//!
//! The log time and the transmit time of every message are the same instant.
//! They differ on a real recording -- one is when the message was sent, the
//! other when the logger wrote it -- but an authored scenario has no such gap to
//! represent, and the runner schedules on the transmit time.

use clockwork_logs::LogError;
use clockwork_logs::offboard::{ChannelId, OffboardWriter, OffboardWriterConfig};
use clockwork_rs::SyncTime;
use reachy_motion::joints::flags;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use brenn_reachy__cogs__schedule_clk_rs::{PostureWire, StepKindWire};
use brenn_reachy__cogs__script_clk_rs::{ScriptOverlayWire, ScriptStepWire, ScriptWire};
use brenn_reachy__cogs__sim_state_clk_rs::{SimCmdWire, SimOpWire};
use brenn_reachy__hardware__dynamixel__registers_clk_rs::RegIdWire;
use brenn_reachy__motion__joints_clk_rs::{JointFlagsWire, JointsWire};

use motion_channels::{SCRIPT_CHANNEL, SIM_CMD_CHANNEL};

/// Every servo on the bus, as the schema's flags.
///
/// Folded from the motion library's own set rather than spelled as nine bits, so
/// a machine that grew a servo energises the rows it has: a literal here would
/// leave the new one limp while every assertion in the scenario passed.
#[must_use]
pub fn all_rows() -> JointFlagsWire {
    JointFlagsWire::from(flags::all())
}

/// Run one scenario's author: write its input log where the harness asks for
/// it, and say when the run it describes ends.
///
/// The end cycle is what a scenario states; the instant it becomes is what goes
/// to standard output, because that is the harness's protocol rather than the
/// scenario's: the shell script that runs the three phases passes the number to
/// the runner without knowing what it is. Taking the cycle here is what keeps a
/// scenario from restating the conversion -- one argument in, the end time on
/// stdout, and each author is then the log it writes.
pub fn main(
    name: &str,
    end_cycle: i64,
    write: impl FnOnce(&Path) -> Result<(), LogError>,
) -> ExitCode {
    let mut args = std::env::args_os().skip(1);
    let (Some(dir), None) = (args.next(), args.next()) else {
        eprintln!("usage: {name} <input-log-dir>");
        return ExitCode::FAILURE;
    };
    let dir = PathBuf::from(dir);
    match write(&dir) {
        Ok(()) => {
            println!("{}", crate::cycle_at(end_cycle));
            ExitCode::SUCCESS
        }
        Err(err) => {
            eprintln!("writing the input log under {}: {err}", dir.display());
            ExitCode::FAILURE
        }
    }
}

/// One step of a script, as a scenario states it.
///
/// Absolute instants rather than the offsets a script carries, because a scenario
/// reasons in cycles of the run and every other assertion it makes is against the
/// same numbers. [`InputLog::script`] is where they become offsets from the
/// arrival the sender stamps, which is the only clock a script's times are
/// measured from.
pub struct Step {
    /// When the step begins, inclusive.
    pub start_ns: i64,
    /// When it ends, exclusive.
    pub end_ns: i64,
    /// The posture it asks for, or `None` for a step that keeps whatever the
    /// machine was last sent to.
    pub posture: Option<PostureWire>,
}

/// One overlay window of a script, as a scenario states it.
///
/// Absolute instants for the same reason a step's are, and the motion is named
/// by the index the configured library gives it -- the numbering the emitted
/// asset's name sidecar carries, which is what a script author reads.
pub struct Overlay {
    /// Which motion, as its index among the configured motions.
    pub motion_id: u16,
    /// When the window opens, inclusive.
    pub start_ns: i64,
    /// When it closes, exclusive. A window that closes before its motion has
    /// played out truncates it, which is a thing a scenario may want to say.
    pub end_ns: i64,
    /// How much of the motion's delta is applied at full weight.
    pub gain: f64,
    /// How fast it is played, as a multiple of the rate its clips were authored
    /// at.
    pub speed: f64,
}

/// The input log being written.
pub struct InputLog {
    writer: OffboardWriter,
    scripts: ChannelId,
    injections: ChannelId,
    script_seq: u32,
    injection_seq: u32,
}

impl InputLog {
    /// Create the log under `dir`.
    ///
    /// Both channels are created typed, from the same generated schemas the cogs
    /// read, so the log carries the schema name and definition the runner's
    /// publisher config was generated from. A channel created untyped would
    /// replay as a silence the run could not tell from an empty scenario.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses: an unwritable directory, or a channel name
    /// used twice.
    pub fn create(dir: &Path) -> Result<Self, LogError> {
        let mut writer = OffboardWriter::create(dir, OffboardWriterConfig::default())?;
        let scripts = writer.create_channel_typed::<ScriptWire>(SCRIPT_CHANNEL)?;
        let injections = writer.create_channel_typed::<SimCmdWire>(SIM_CMD_CHANNEL)?;
        Ok(Self {
            writer,
            scripts,
            injections,
            script_seq: 0,
            injection_seq: 0,
        })
    }

    /// Anchor the run at `at_ns`, stating the world it begins in: nothing is
    /// holding any row of the machine.
    ///
    /// The deterministic runner starts its clock at the first message the input
    /// log carries, so a scenario whose first message is its script would begin
    /// at the script -- and the start-up survey the session runs before it will
    /// take one would never have happened, which makes the script arrive at a
    /// machine that refuses it. So every scenario says what the world is at the
    /// epoch, and what it says is true of a machine nothing has touched.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn begin(&mut self, at_ns: i64) -> Result<(), LogError> {
        self.release(at_ns, all_rows())
    }

    /// Ask the machine to run a script, sent at `at_ns`.
    ///
    /// The instant it is sent is the arrival the sender stamps it with, and every
    /// step's offset is measured from that: a consumer never reads a clock to
    /// interpret a script, so a sender whose clock is wrong asks for a script
    /// that plays at the wrong time rather than one that is silently
    /// reinterpreted.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    ///
    /// # Panics
    ///
    /// If more steps are given than the schema holds, or a step begins before the
    /// script arrives, or a step's bounds are not a whole number of milliseconds
    /// apart -- a script carries millisecond offsets, so an instant off that grid
    /// is a scenario asking for something it cannot state.
    pub fn script(&mut self, at_ns: i64, script_id: u32, steps: &[Step]) -> Result<(), LogError> {
        self.playing(at_ns, script_id, steps, &[])
    }

    /// Ask the machine to run a script that also plays motions over its
    /// postures, sent at `at_ns`.
    ///
    /// The same request as [`Self::script`] with the overlay half filled in: a
    /// script is one message, so a scenario asking for a composition asks for it
    /// and the base it rides on together.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    ///
    /// # Panics
    ///
    /// On the same statements [`Self::script`] refuses, and for more windows
    /// than the schema holds.
    pub fn playing(
        &mut self,
        at_ns: i64,
        script_id: u32,
        steps: &[Step],
        overlays: &[Overlay],
    ) -> Result<(), LogError> {
        let mut message = ScriptWire::new();
        message.set_script_id(script_id);
        message.set_arrival(SyncTime::from_nanos(at_ns));
        {
            let mut rows = message.steps_mut();
            rows.clear();
            for step in steps {
                let row: &mut ScriptStepWire = rows
                    .try_grow()
                    .expect("a script of no more steps than the schema holds");
                row.set_after_ms(offset_ms(at_ns, step.start_ns));
                row.set_duration_ms(offset_ms(step.start_ns, step.end_ns));
                match step.posture {
                    Some(posture) => {
                        row.set_kind(StepKindWire::BASE_POSTURE);
                        row.set_posture(posture);
                    }
                    None => row.set_kind(StepKindWire::BASE_KEEP),
                }
            }
        }
        {
            let mut rows = message.overlays_mut();
            rows.clear();
            for overlay in overlays {
                let row: &mut ScriptOverlayWire = rows
                    .try_grow()
                    .expect("a script of no more windows than the schema holds");
                row.set_motion_id(overlay.motion_id);
                row.set_after_ms(offset_ms(at_ns, overlay.start_ns));
                row.set_duration_ms(offset_ms(overlay.start_ns, overlay.end_ns));
                row.set_gain(overlay.gain);
                row.set_speed(overlay.speed);
            }
        }
        let at = SyncTime::from_nanos(at_ns);
        self.writer
            .write_typed(self.scripts, self.script_seq, at, at, &message)?;
        self.script_seq += 1;
        Ok(())
    }

    /// Publish an injection at `at_ns`: the scenario's hand on the plant.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn inject(&mut self, at_ns: i64, injection: &SimCmdWire) -> Result<(), LogError> {
        let at = SyncTime::from_nanos(at_ns);
        self.writer
            .write_typed(self.injections, self.injection_seq, at, at, injection)?;
        self.injection_seq += 1;
        Ok(())
    }

    /// Energise the given rows, as an arming sequencer would.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn torque_on(&mut self, at_ns: i64, rows: JointFlagsWire) -> Result<(), LogError> {
        self.inject(at_ns, &operation(SimOpWire::TORQUE_ON, rows))
    }

    /// De-energise the given rows.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn torque_off(&mut self, at_ns: i64, rows: JointFlagsWire) -> Result<(), LogError> {
        self.inject(at_ns, &operation(SimOpWire::TORQUE_OFF, rows))
    }

    /// Write `bits` into the given rows' hardware-error byte: the servos start
    /// complaining about themselves.
    ///
    /// The one condition a scenario can state that the plant has no other way
    /// to express. What a servo's error byte means is the servo's own business
    /// and nothing in the modelled machine produces one, so a run about a
    /// machine whose motor is in trouble says so here and the driver's rotating
    /// read is what carries it to the session.
    ///
    /// Through the register the real bus holds it in, so the reading the session
    /// classifies is the reading a real rotation would have taken.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn set_error_bits(
        &mut self,
        at_ns: i64,
        rows: JointFlagsWire,
        bits: u8,
    ) -> Result<(), LogError> {
        let mut injection = operation(SimOpWire::SET_REGISTER, rows);
        injection.set_reg(RegIdWire::HARDWARE_ERROR_STATUS);
        injection.set_value(u64::from(bits));
        self.inject(at_ns, &injection)
    }

    /// Take the given rows off the bus: nothing answers for them at all.
    ///
    /// No ping, no register read, no write, no health report. The stand-in for a
    /// servo that is dead, unplugged or was never fitted, which is the one
    /// condition a start-up survey has to refuse the whole machine over.
    ///
    /// The set replaces whatever was named before, so a call naming no rows puts
    /// the whole bus back: an absence a scenario could not end is one it could
    /// not show a machine surviving.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn absent(&mut self, at_ns: i64, rows: JointFlagsWire) -> Result<(), LogError> {
        self.inject(at_ns, &operation(SimOpWire::ABSENT_SERVO, rows))
    }

    /// Jam the given rows where they stand.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn obstruct(&mut self, at_ns: i64, rows: JointFlagsWire) -> Result<(), LogError> {
        self.inject(at_ns, &operation(SimOpWire::OBSTRUCT, rows))
    }

    /// Release the given rows.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn release(&mut self, at_ns: i64, rows: JointFlagsWire) -> Result<(), LogError> {
        self.inject(at_ns, &operation(SimOpWire::RELEASE_OBSTRUCTION, rows))
    }

    /// Lose the next `cycles` cycles of position replies.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn drop_replies(&mut self, at_ns: i64, cycles: u32) -> Result<(), LogError> {
        let mut injection = operation(SimOpWire::DROP_REPLIES, JointFlagsWire::NONE);
        injection.set_count(cycles);
        self.inject(at_ns, &injection)
    }

    /// Teleport the given rows.
    ///
    /// The positions are filled in by `edit` rather than handed over whole: a
    /// generated message derives nothing, not even `Clone`, because a message is
    /// as large as its layout and copying one is the caller's decision.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn set_positions(
        &mut self,
        at_ns: i64,
        rows: JointFlagsWire,
        edit: impl FnOnce(&mut JointsWire),
    ) -> Result<(), LogError> {
        let mut injection = operation(SimOpWire::SET_POSITIONS, rows);
        edit(injection.positions_mut());
        self.inject(at_ns, &injection)
    }

    /// Finish the log.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses while flushing.
    pub fn close(self) -> Result<(), LogError> {
        self.writer.close()
    }
}

/// One injection of the given shape, with nothing else set.
fn operation(op: SimOpWire, rows: JointFlagsWire) -> SimCmdWire {
    let mut injection = SimCmdWire::new();
    injection.set_op(op);
    injection.set_mask(rows);
    injection
}

/// How many milliseconds `to` is after `from`.
///
/// # Panics
///
/// If it is before it, or if the gap is not a whole number of milliseconds: a
/// script's offsets are milliseconds, so either is a scenario stating an instant
/// no script can carry.
fn offset_ms(from: i64, to: i64) -> u32 {
    let gap = to - from;
    assert!(gap >= 0, "a step at {to} cannot precede {from}");
    assert!(
        gap % 1_000_000 == 0,
        "{gap}ns is not a whole number of milliseconds",
    );
    u32::try_from(gap / 1_000_000).expect("an offset a script can carry")
}
