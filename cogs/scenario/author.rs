//! Phase one of the harness: turn a scenario's statement into an input log.
//!
//! The two channels the deterministic runner publishes from a log are the ones
//! nothing in this system produces -- the session's schedule and the scenario's
//! hand on the plant. Everything else in the run follows from them.
//!
//! The log time and the transmit time of every message are the same instant.
//! They differ on a real recording -- one is when the message was sent, the
//! other when the logger wrote it -- but an authored scenario has no such gap to
//! represent, and the runner schedules on the transmit time.

use clockwork_logs::LogError;
use clockwork_logs::offboard::{ChannelId, OffboardWriter, OffboardWriterConfig};
use clockwork_rs::SyncTime;
use std::path::{Path, PathBuf};
use std::process::ExitCode;

use brenn_reachy__cogs__msgs_clk_rs::{
    JointFlags, Joints, Posture, ScheduledStep, SessionSchedule, SimCmd, SimOp, StepKind,
};

use reachy_motion::joints::JointSet;

use crate::{SCHEDULE_CHANNEL, SIM_CMD_CHANNEL};

/// Every servo on the bus, as the schema's flags.
///
/// Folded from the motion library's own set rather than spelled as nine bits, so
/// a machine that grew a servo energises the rows it has: a literal here would
/// leave the new one limp while every assertion in the scenario passed.
pub const ALL_ROWS: JointFlags = JointFlags(JointSet::ALL.bits());

/// Run one scenario's author: write its input log where the harness asks for
/// it, and say when the run it describes ends.
///
/// The end time goes to standard output because it is a fact about the scenario
/// rather than about the harness: the shell script that runs the three phases
/// passes it to the runner without knowing what it is, so the scenario's
/// schedule is stated in exactly one place. That protocol -- one argument, the
/// end time on stdout -- is the harness's rather than any one scenario's, so it
/// is stated here and each author is the log it writes.
pub fn main(
    name: &str,
    end_time_ns: i64,
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
            println!("{end_time_ns}");
            ExitCode::SUCCESS
        }
        Err(err) => {
            eprintln!("writing the input log under {}: {err}", dir.display());
            ExitCode::FAILURE
        }
    }
}

/// One step of a schedule, as a scenario states it.
pub struct Step {
    /// When the step begins, inclusive.
    pub start_ns: i64,
    /// When it ends, exclusive.
    pub end_ns: i64,
    /// The posture it asks for, or `None` for a step that keeps whatever the
    /// machine was last sent to.
    pub posture: Option<Posture>,
}

/// The input log being written.
pub struct InputLog {
    writer: OffboardWriter,
    schedule: ChannelId,
    injections: ChannelId,
    schedule_seq: u32,
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
        let schedule = writer.create_channel_typed::<SessionSchedule>(SCHEDULE_CHANNEL)?;
        let injections = writer.create_channel_typed::<SimCmd>(SIM_CMD_CHANNEL)?;
        Ok(Self {
            writer,
            schedule,
            injections,
            schedule_seq: 0,
            injection_seq: 0,
        })
    }

    /// Publish a schedule at `at_ns`, as the session cog's channel would.
    ///
    /// The whole schedule every time: it is state rather than an event, and the
    /// epoch is what makes a change observable when two schedules happen to look
    /// alike.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    ///
    /// # Panics
    ///
    /// If more steps are given than the schema holds.
    pub fn schedule(
        &mut self,
        at_ns: i64,
        engaged: bool,
        epoch: u32,
        steps: &[Step],
    ) -> Result<(), LogError> {
        let mut message = SessionSchedule::new();
        message.set_engaged(engaged);
        message.set_epoch(epoch);
        {
            let mut rows = message.steps_mut();
            rows.clear();
            for step in steps {
                let row: &mut ScheduledStep = rows
                    .try_grow()
                    .expect("a schedule of no more steps than the schema holds");
                row.set_start(SyncTime::from_nanos(step.start_ns));
                row.set_end(SyncTime::from_nanos(step.end_ns));
                match step.posture {
                    Some(posture) => {
                        row.set_kind(StepKind::BASE_POSTURE);
                        row.set_posture(posture);
                    }
                    None => row.set_kind(StepKind::BASE_KEEP),
                }
            }
        }
        let at = SyncTime::from_nanos(at_ns);
        self.writer
            .write_typed(self.schedule, self.schedule_seq, at, at, &message)?;
        self.schedule_seq += 1;
        Ok(())
    }

    /// Publish an injection at `at_ns`: the scenario's hand on the plant.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn inject(&mut self, at_ns: i64, injection: &SimCmd) -> Result<(), LogError> {
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
    pub fn torque_on(&mut self, at_ns: i64, rows: JointFlags) -> Result<(), LogError> {
        self.inject(at_ns, &operation(SimOp::TORQUE_ON, rows))
    }

    /// De-energise the given rows.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn torque_off(&mut self, at_ns: i64, rows: JointFlags) -> Result<(), LogError> {
        self.inject(at_ns, &operation(SimOp::TORQUE_OFF, rows))
    }

    /// Jam the given rows where they stand.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn obstruct(&mut self, at_ns: i64, rows: JointFlags) -> Result<(), LogError> {
        self.inject(at_ns, &operation(SimOp::OBSTRUCT, rows))
    }

    /// Release the given rows.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn release(&mut self, at_ns: i64, rows: JointFlags) -> Result<(), LogError> {
        self.inject(at_ns, &operation(SimOp::RELEASE_OBSTRUCTION, rows))
    }

    /// Lose the next `cycles` cycles of position replies.
    ///
    /// # Errors
    ///
    /// Whatever the writer refuses.
    pub fn drop_replies(&mut self, at_ns: i64, cycles: u32) -> Result<(), LogError> {
        let mut injection = operation(SimOp::DROP_REPLIES, JointFlags::NONE);
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
        rows: JointFlags,
        edit: impl FnOnce(&mut Joints),
    ) -> Result<(), LogError> {
        let mut injection = operation(SimOp::SET_POSITIONS, rows);
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
fn operation(op: SimOp, rows: JointFlags) -> SimCmd {
    let mut injection = SimCmd::new();
    injection.set_op(op);
    injection.set_mask(rows);
    injection
}
