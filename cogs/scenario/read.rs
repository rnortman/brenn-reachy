//! Phase three of the harness: take an output log apart into typed streams.
//!
//! One pass over the log yields every message of every channel the motion system
//! declares, lifted out of the reader's borrowed view so a checker can look at
//! the whole run before asserting anything. A failure then reports what the log
//! held rather than stopping at the first surprise.
//!
//! Nothing here asserts, and nothing here reads. What each stream should contain
//! is the scenario's business; the pass itself -- open, check every binding,
//! dispatch, census -- is `log_read::read_with`'s, shared with every other
//! reader in the repo. This module's job is to say which channel of this system
//! carries what, once. The table below is that statement: the same row drives
//! the binding check and the decode, so a channel cannot be checked and left
//! unread, or read and left unchecked.

use clockwork_logs::LogError;
use std::path::Path;

use brenn_reachy__cogs__schedule_clk_rs::SessionScheduleWire;
use brenn_reachy__cogs__script_clk_rs::ScriptWire;
use brenn_reachy__cogs__session_cmd_clk_rs::SessionCmdWire;
use brenn_reachy__cogs__sim_state_clk_rs::SimCmdWire;
use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
use brenn_reachy__driver__health_clk_rs::{DriverEventWire, HealthReportWire};
use brenn_reachy__driver__pose_clk_rs::{PoseEstimateWire, PoseSampleWire};
use brenn_reachy__motion__faults_clk_rs::TickFaultWire;
use brenn_reachy__motion__timeline_clk_rs::{TimelineEntryWire, TimelineWire};
use log_read::{Bound, Census, Complaints, Logged, Streams, binding, cumulative, read_with, typed};
use motion_channels::{
    CMD_CHANNEL, ESTIMATE_CHANNEL, EVENT_CHANNEL, FAULT_CHANNEL, HEALTH_CHANNEL, POSE_CHANNEL,
    REPORT_CHANNEL, SCHEDULE_CHANNEL, SCRIPT_CHANNEL, SESSION_CMD_CHANNEL, SIM_CMD_CHANNEL,
};

/// Everything one run put in the log.
#[derive(Default)]
pub struct Run {
    /// What was asked of the machine, replayed from the input log.
    pub scripts: Vec<Logged<ScriptWire>>,
    /// What the session published: the schedule it accepted, once per change.
    pub schedules: Vec<Logged<SessionScheduleWire>>,
    /// What the scenario did to the plant, replayed from the input log.
    pub injections: Vec<Logged<SimCmdWire>>,
    /// The driver's heartbeat: one per cycle, always.
    pub samples: Vec<Logged<PoseSampleWire>>,
    /// What the machine was asked to hold next.
    pub goals: Vec<Logged<GoalSetpointWire>>,
    /// What the gate did that the sample stream does not show.
    pub events: Vec<Logged<DriverEventWire>>,
    /// What the driver's rotating read of the status registers found: one report
    /// per row it visited, carrying the instant the reading was taken. The
    /// picture the session's torque-on gate is judged over is built from these.
    ///
    /// Named for the rail because the pose samples carry readings of their own.
    pub rail_readings: Vec<Logged<HealthReportWire>>,
    /// What the decision tick raised.
    pub faults: Vec<Logged<TickFaultWire>>,
    /// What the session said about all of it: the rows of the newest story it
    /// published, which is the whole of its narration.
    pub reports: Vec<Logged<TimelineEntryWire>>,
    /// What the session asked the driver for.
    pub datagrams: Vec<Logged<SessionCmdWire>>,
    /// Where the head was.
    pub estimates: Vec<Logged<PoseEstimateWire>>,
    /// Every channel the log carries and how many messages each held. A checker
    /// uses this to say something about the channels it does not otherwise read
    /// -- the signal report groups, for one.
    pub census: Census,
    /// Anything that went wrong reading the log itself: a channel carrying a
    /// schema this build does not know, a payload that did not decode, a
    /// counter the reader kept. Every one of these is a failure of the run.
    pub complaints: Complaints,
}

impl Streams for Run {
    fn census(&mut self) -> &mut Census {
        &mut self.census
    }

    fn complaints(&mut self) -> &mut Complaints {
        &mut self.complaints
    }
}

/// Every channel a checker of this system reads.
///
/// The signal report groups are deliberately absent: nothing binds a Rust type
/// to a group's generated schema, so they are observable through
/// [`Run::census`] and not decoded.
const CHANNELS: [Bound<Run>; 11] = [
    Bound {
        name: SCRIPT_CHANNEL,
        check: binding::<ScriptWire>,
        route: |run, message| typed(message, &mut run.scripts, &mut run.complaints),
    },
    Bound {
        name: SCHEDULE_CHANNEL,
        check: binding::<SessionScheduleWire>,
        route: |run, message| typed(message, &mut run.schedules, &mut run.complaints),
    },
    Bound {
        name: SIM_CMD_CHANNEL,
        check: binding::<SimCmdWire>,
        route: |run, message| typed(message, &mut run.injections, &mut run.complaints),
    },
    Bound {
        name: FAULT_CHANNEL,
        check: binding::<TickFaultWire>,
        route: |run, message| typed(message, &mut run.faults, &mut run.complaints),
    },
    Bound {
        name: REPORT_CHANNEL,
        check: binding::<TimelineWire>,
        route: |run, message| {
            cumulative(
                message,
                &mut run.reports,
                &mut run.complaints,
                |story: &TimelineWire, rows| rows.extend(story.entries().iter().cloned()),
            );
        },
    },
    Bound {
        name: SESSION_CMD_CHANNEL,
        check: binding::<SessionCmdWire>,
        route: |run, message| typed(message, &mut run.datagrams, &mut run.complaints),
    },
    Bound {
        name: ESTIMATE_CHANNEL,
        check: binding::<PoseEstimateWire>,
        route: |run, message| typed(message, &mut run.estimates, &mut run.complaints),
    },
    Bound {
        name: POSE_CHANNEL,
        check: binding::<PoseSampleWire>,
        route: |run, message| typed(message, &mut run.samples, &mut run.complaints),
    },
    Bound {
        name: CMD_CHANNEL,
        check: binding::<GoalSetpointWire>,
        route: |run, message| typed(message, &mut run.goals, &mut run.complaints),
    },
    Bound {
        name: EVENT_CHANNEL,
        check: binding::<DriverEventWire>,
        route: |run, message| typed(message, &mut run.events, &mut run.complaints),
    },
    Bound {
        name: HEALTH_CHANNEL,
        check: binding::<HealthReportWire>,
        route: |run, message| typed(message, &mut run.rail_readings, &mut run.complaints),
    },
];

impl Run {
    /// Read the output log under `dir`.
    ///
    /// # Errors
    ///
    /// Whatever the shared pass refuses about the log as a whole. A message the
    /// reader yielded but this build could not make sense of is a complaint
    /// rather than an error: the point is to report all of them.
    pub fn read(dir: &Path) -> Result<Self, LogError> {
        read_with(dir, &CHANNELS)
    }
}
