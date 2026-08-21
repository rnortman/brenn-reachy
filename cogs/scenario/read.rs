//! Phase three of the harness: take an output log apart into typed streams.
//!
//! One pass over the log yields every message of every channel the motion system
//! declares, lifted out of the reader's borrowed view so a checker can look at
//! the whole run before asserting anything. A failure then reports what the log
//! held rather than stopping at the first surprise.
//!
//! Nothing here asserts. What each stream should contain is the scenario's
//! business; this module's job is to say which channel of this system carries
//! what, once. The table below is that statement: the same row drives the
//! binding check and the decode, so a channel cannot be checked and left
//! unread, or read and left unchecked.
//!
//! The reading itself -- the borrowed message to an owned value, the carrier to
//! a datagram, the complaint when either refuses -- is `log_read`'s, shared with
//! every other checker in the repo.

use clockwork_logs::offboard::OffboardReader;
use clockwork_logs::{ChannelMetadata, LogError, LoggedMessage};
use std::path::Path;

use brenn_reachy__cogs__schedule_clk_rs::SessionScheduleWire;
use brenn_reachy__cogs__sim_state_clk_rs::SimCmdWire;
use brenn_reachy__driver__goal_clk_rs::GoalSetpointWire;
use brenn_reachy__driver__health_clk_rs::DriverEventWire;
use brenn_reachy__driver__pose_clk_rs::{PoseEstimateWire, PoseSampleWire};
use brenn_reachy__motion__faults_clk_rs::TickFaultWire;
use log_read::{Complaints, Logged, binding, typed};

use crate::{
    CMD_CHANNEL, ESTIMATE_CHANNEL, EVENT_CHANNEL, FAULT_CHANNEL, POSE_CHANNEL, SCHEDULE_CHANNEL,
    SIM_CMD_CHANNEL,
};

/// Everything one run put in the log.
#[derive(Default)]
pub struct Run {
    /// What the session asked for, replayed from the input log.
    pub schedules: Vec<Logged<SessionScheduleWire>>,
    /// What the scenario did to the plant, replayed from the input log.
    pub injections: Vec<Logged<SimCmdWire>>,
    /// The driver's heartbeat: one per cycle, always.
    pub samples: Vec<Logged<PoseSampleWire>>,
    /// What the machine was asked to hold next.
    pub goals: Vec<Logged<GoalSetpointWire>>,
    /// What the gate did that the sample stream does not show.
    pub events: Vec<Logged<DriverEventWire>>,
    /// What the decision tick raised.
    pub faults: Vec<Logged<TickFaultWire>>,
    /// Where the head was.
    pub estimates: Vec<Logged<PoseEstimateWire>>,
    /// Every channel the log carries, in the order the reader reports them.
    /// A checker uses this to say something about the channels it does not
    /// otherwise read -- the signal report groups, for one.
    pub channel_names: Vec<String>,
    /// How many messages each of those channels carried, in the same order. A
    /// checker with no Rust type bound to a channel can still say whether it
    /// carried anything, which is the difference between a report group that
    /// reported and one that only exists.
    pub channel_message_counts: Vec<usize>,
    /// Anything that went wrong reading the log itself: a channel carrying a
    /// schema this build does not know, a payload that did not decode, a
    /// counter the reader kept. Every one of these is a failure of the run.
    pub complaints: Complaints,
}

/// One channel of this system, and the two things done with it.
struct Bound {
    /// The channel's name in the log, which is the name the system declares.
    name: &'static str,
    /// Assert the log records the schema this build is about to decode.
    check: fn(&[ChannelMetadata], &str, &mut Complaints),
    /// Decode one message of it into the stream it belongs to.
    route: fn(&mut Run, &LoggedMessage<'_>),
}

/// Every channel a checker of this system reads.
///
/// The signal report groups are deliberately absent: nothing binds a Rust type
/// to a group's generated schema, so they are observable through
/// [`Run::channel_names`] and not decoded.
const CHANNELS: [Bound; 7] = [
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
];

impl Run {
    /// Read the output log under `dir`.
    ///
    /// # Errors
    ///
    /// Whatever the reader refuses about the log as a whole. A message the
    /// reader yielded but this build could not make sense of is a complaint
    /// rather than an error: the point is to report all of them.
    pub fn read(dir: &Path) -> Result<Self, LogError> {
        let mut reader = OffboardReader::open(dir)?;
        let mut run = Self::default();

        for metadata in reader.channels() {
            run.channel_names.push(metadata.channel_name.clone());
        }
        run.channel_message_counts = vec![0; run.channel_names.len()];
        // Before a single message is decoded: a payload that matches in size is
        // not necessarily the right schema.
        let channels: Vec<ChannelMetadata> = reader.channels().to_vec();
        for bound in &CHANNELS {
            (bound.check)(&channels, bound.name, &mut run.complaints);
        }

        while let Some(message) = reader.read_next()? {
            let name = message.metadata.channel_name.as_str();
            if let Some(index) = run.channel_names.iter().position(|known| known == name)
                && let Some(count) = run.channel_message_counts.get_mut(index)
            {
                *count += 1;
            }
            if let Some(bound) = CHANNELS.iter().find(|bound| bound.name == name) {
                (bound.route)(&mut run, &message);
            }
        }

        if !reader.error_counters().is_clean() {
            run.complaints.push(format!(
                "the reader recorded errors over the output log: {:?}",
                reader.error_counters()
            ));
        }
        Ok(run)
    }
}
