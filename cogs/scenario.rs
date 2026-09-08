//! What every scenario of the motion system is made of.
//!
//! A scenario is three programs sharing one statement of what the run is: an
//! author that turns the statement into an input log, the deterministic runner
//! that plays the log through the system, and a checker that joins the output
//! log back to the same statement. A checker whose expectations were restated in
//! its own source would pass by agreeing with itself; this crate is where the
//! agreement lives.
//!
//! Three parts, and the split is the one the harness has: [`author`] writes an
//! input log, [`read`] takes an output log apart into typed streams, and
//! [`check`] holds the assertions every scenario of this system makes about the
//! result. The facts they all rest on -- the epoch, the grid, the channel names,
//! and the configured numbers the cogs run on -- are here at the top.
//!
//! Times are absolute simulated nanoseconds since the Unix epoch. The
//! deterministic runner jumps its clock to each logged message's transmit time,
//! so a scenario's schedule is the times it writes; nothing rebases them.

// The crate root's own directory is where rustc looks for a submodule, and this
// crate's root is a file in a package of many; the two halves live in a
// directory of their own so the package's file names stay about their subjects.
#[path = "scenario/author.rs"]
pub mod author;
#[path = "scenario/check.rs"]
pub mod check;
#[path = "scenario/read.rs"]
pub mod read;

/// The scenario epoch: an arbitrary round Unix time, far enough from zero that
/// a dropped or defaulted timestamp reads as obviously wrong rather than as a
/// plausible small number.
pub const T0_NS: i64 = 1_700_000_000_000_000_000;

/// The bus cycle. Every sample sits on this grid, the plant advances one step
/// per cycle, and the decision tick dates its goals in multiples of it.
pub const PERIOD_NS: i64 = 20_000_000;

/// How many cycles ahead of the sample that decided it a goal is dated.
pub const LAG_K: i64 = 2;

/// How long a move to the upright posture is given.
pub const UP_DURATION_NS: i64 = 800_000_000;

/// How long a move to stow is given.
pub const STOW_DURATION_NS: i64 = 2_000_000_000;

/// How long the goal stream may be silent before the gate de-torques.
pub const HOLD_TIMEOUT_NS: i64 = 200_000_000;

/// Whether the modelled machine starts energised. Every scenario starts it cold:
/// the session's own engagement is what energises it, over the bus, which is the
/// arming path a real machine has.
pub const START_TORQUED: bool = false;

/// The minimum spacing between the simulated driver's health reports.
///
/// Per report rather than per lap: the rotation reads one servo each time it
/// reports, so a scenario counting reports over a stretch divides by this and a
/// scenario waiting for a full lap of the nine multiplies it by nine.
pub const HEALTH_POLL_PERIOD_NS: i64 = 120_000_000;

/// The hardware-error bits a scenario writes into a servo to make the library
/// classify it as a fault.
///
/// Anything but the input-voltage bit: that one latches on a supply dip the
/// servo rode out and is reported rather than acted on, so a run asserting a
/// response would be asserting it against evidence the library classifies as
/// nothing. Written against the library's own name for that bit, which is what
/// the assertion below can check; a bit that stops being acted on for any other
/// reason is not something this expression can notice, so a change to what the
/// classifier ignores is a change the health-evidence scenarios have to be
/// re-read against.
pub const ACTED_ON_ERROR_BITS: u8 = 0x20 & !reachy_motion::joints::ServoHealth::INPUT_VOLTAGE;

// The one case the expression above can defend itself against: the informational
// bit moving onto the one this constant names would leave every health-evidence
// scenario asserting a response against evidence the classifier discards.
const _: () = assert!(ACTED_ON_ERROR_BITS != 0);

/// How many cycles in a row the modelled bus may answer nothing before the
/// driver declares its bus gone.
///
/// The driver's own threshold, restated here so a scenario's expectation and the
/// number the driver is built with are two statements rather than one:
/// `check_params` fails the run when they disagree, which is what makes an
/// assertion that the bus failure lands *on time* an assertion at all.
pub const BLIND_CYCLES_BEFORE_BUS_FAILURE: u32 = 25;

/// How long the simulated driver gives its own read-back pass to confirm a
/// commanded de-torquing, nanoseconds.
///
/// The driver's budget, restated here for the same reason and pinned the same
/// way. Shorter than the session's below: the pass reads one row a cycle, so a
/// clean sweep of the nine is well inside it, and the host's longer budget is
/// what decides when a de-torquing is *said* to be unconfirmed.
pub const DRIVER_CONFIRM_BUDGET_NS: i64 = 300_000_000;

/// How long the session gives a transaction to be answered before the same
/// datagram goes out again, nanoseconds.
///
/// Ten driver cycles. A driver that took a transaction up answers it on the
/// cycle it took it up, so a re-issue in one of these runs means the request or
/// its answer was lost -- which in this system, where every channel is memory,
/// means a scenario that arranged for it.
pub const AUX_TIMEOUT_NS: i64 = 200_000_000;

/// How many times the session re-issues a datagram nothing answered before it
/// hands the sequence a silence.
pub const AUX_RETRIES: i64 = 3;

/// How many nominal periods may pass with no fresh sample before the session
/// declares the bus failed.
pub const SAMPLE_STALE_AFTER: i64 = 5;

/// How long after start-up the session allows the first sample, nanoseconds.
pub const STARTUP_GRACE_NS: i64 = 2_000_000_000;

/// How long a wind-down's one clock runs, nanoseconds.
pub const STOW_BUDGET_NS: i64 = 4_000_000_000;

/// How long the session gives a commanded torque-off to be confirmed,
/// nanoseconds.
///
/// The session's budget and not the driver's: the driver runs its own, shorter
/// one over the pass it reads back, and this is how long the host waits before
/// saying the de-torquing went unconfirmed. It keeps commanding either way.
pub const SESSION_CONFIRM_BUDGET_NS: i64 = 500_000_000;

/// The servo-side profile acceleration the commissioning sweep writes, in the
/// register's own units.
///
/// Mirrors the deployed `ServoProfile.legs_profile_acceleration`; a scenario
/// asserting this pair is asserting about the file the process read. What makes
/// the claim reach the wire is
/// [`check::commissioned_profile`](crate::check::commissioned_profile), which
/// finds the two writes on each of the six crank rows in the run's own
/// datagrams. What makes it the pair the decision tick judges those joints
/// against is [`check_params`], which compares the file with the motion
/// library's own `plant::SHIPPED_PROFILES`.
pub const LEGS_PROFILE_ACCELERATION: i64 = 20;

/// The six cranks' profile velocity the sweep writes, register units.
pub const LEGS_PROFILE_VELOCITY: i64 = 50;

/// The body yaw servo's profile acceleration, register units. Its own pair
/// because its motor and its load are its own.
pub const BODY_YAW_PROFILE_ACCELERATION: i64 = 20;

/// The body yaw servo's profile velocity, register units.
pub const BODY_YAW_PROFILE_VELOCITY: i64 = 50;

/// The two antennas' profile acceleration, register units. Their own pair: the
/// antenna servos are a different XL330 variant with a Velocity Limit 3.6x the
/// head's.
pub const ANTENNAS_PROFILE_ACCELERATION: i64 = 20;

/// The two antennas' profile velocity, register units.
pub const ANTENNAS_PROFILE_VELOCITY: i64 = 50;

/// Whether the deployed `MoverParams` arms the plant-model tracking detector.
///
/// Pinned true, and there is no scenario for false: the field exists for an
/// attended capability run commissioned at a profile the motors cannot follow,
/// where the model is known not to describe them. Every shipped configuration
/// carries it armed, and `check_params` is what says so about the file the
/// processes read.
pub const TRACKING_ARMED: bool = true;

/// The servos' Bus Watchdog timeout the commissioning sweep arms, in the
/// register's 20 ms units.
///
/// Mirrors the deployed `SessionParams.bus_watchdog`. The sweep writes it twice
/// per servo -- zero to clear, then this -- and
/// [`check::commissioned_profile`](crate::check::commissioned_profile) is what
/// finds both on the wire, in that order.
pub const BUS_WATCHDOG: i64 = 10;

/// How far ahead a script may schedule anything, milliseconds: from its own
/// arrival stamp and from the wake that reads it, whichever is further.
///
/// Mirrors the deployed `SessionParams.script_span_cap_ms`. What it bounds is
/// the sender that stops refreshing: its last schedule runs out inside this,
/// and the session concludes on its normal path rather than holding the machine
/// torqued indefinitely.
pub const SCRIPT_SPAN_CAP_MS: i64 = 600_000;

/// How old a retained rail reading may be and still be gated on, nanoseconds.
///
/// Mirrors the deployed `SessionParams.rail_stale_after_ns`: two laps of the
/// driver's health rotation. A row older than this is re-read by the resting
/// watch before the torque-on gate is judged, which is the only thing that puts
/// an aux transaction on an engagement's path.
pub const RAIL_STALE_AFTER_NS: i64 = 2_200_000_000;

/// How long the session may go without executing: the floor its wake condition
/// puts under a run where nothing arrives, nanoseconds.
///
/// Declared in `cogs/motion.clk` as the session's `time_since_last_exec`
/// condition rather than configured, so there is no file to check it against.
/// What it bounds for a scenario is how long a decision about time having passed
/// can wait -- the session ending because its schedule ran out, most of all.
pub const SESSION_WAKE_FLOOR_NS: i64 = 100_000_000;

/// How long an execution is modelled to take, which is the gap between the
/// instant a cog runs at and the log time of what it published.
///
/// Every cog in this system declares the same duration, so a message's log time
/// is always its cog's start time plus this. It is not jitter and not a
/// tolerance: the run is exact, and a checker that conflated a message's log
/// time with the instant its contents are about would be reading two clocks as
/// one.
///
/// The number this system's cog modules declare, not a repo-wide one: the
/// harness proof next door declares its own, and a single constant behind both
/// would let one system's `.clk` change break the other system's assertions.
pub const EXECUTION_DURATION_NS: i64 = 1_000_000;

/// How long after the cycle it is about a control-rate cog's message is logged.
///
/// Two execution durations, not one: the driver runs at the cycle's nominal
/// instant and publishes its sample a duration later, and the cogs that run on
/// that sample publish a duration after *that*. So a goal or an estimate lands
/// in the log two milliseconds after the instant its contents are about, and
/// the instants themselves -- a goal's `execute_at`, an estimate's
/// `time_of_validity` -- are arithmetic off the cycle rather than off any
/// publish.
pub const CONTROL_DELAY_NS: i64 = 2 * EXECUTION_DURATION_NS;

/// What a signal report group's channel is called, up to the cog that owns it.
///
/// A group's channel name is composed by the framework rather than declared, so
/// the shape is stated once here and the cog names below complete it. What
/// follows the group's own name is a digest of its generated schema, which is
/// why a scenario matches the prefix rather than the whole name.
pub const REPORT_GROUP_PREFIX: &str = "/_clockwork/report-groups/";

/// The group every cog of this system declares its counters in.
pub const REPORT_GROUP: &str = "stats";

/// The cogs of this system, each of which owns one report group.
pub const COGS: [&str; 4] = ["Mover", "Pose", "MotorSim", "Session"];

/// The cycle the simulated driver first executes on.
///
/// One period in rather than at the epoch: the driver runs on a periodic timer,
/// and the first firing of a timer started at the run's beginning is a period
/// later. Stated rather than observed, because a scenario that took the run's
/// first cycle from the run would move its whole expectation along with a
/// regression that delayed the driver -- and one of them, S3, has nothing else
/// pinning when its clock started.
pub const FIRST_CYCLE: i64 = 1;

/// How many cycles a stretch of `duration_ns` covers, rounded up.
///
/// Rounded up rather than down because the durations a scenario states are the
/// time a move is *given*: a move that runs into the fraction of a cycle at the
/// end of its budget has not overrun, and a scenario that rounded the other way
/// would assert the machine had arrived while it was still travelling.
#[must_use]
pub fn cycles_for(duration_ns: i64) -> i64 {
    (duration_ns + PERIOD_NS - 1) / PERIOD_NS
}

/// How many cycles the modelled machine takes to travel `distance_rad` from
/// rest to rest, including the servos' response delay.
///
/// The plant's own duration, not the planner's clock: the servos run the
/// profile the commissioning sweep wrote into them, and on content faster than
/// that profile a joint is still travelling long after the setpoint stream has
/// stopped moving. A scenario states every instant it derives from arrival as an
/// expression over this rather than as an integer somebody nudged until the run
/// went green -- the arithmetic is then the reason the number is what it is.
///
/// The figure is an upper bound on the stepped plant by a cycle or three (the
/// closed form is continuous-time), which is the direction an arrival assertion
/// needs: an instant taken from here is never before the joint got there.
///
/// The class is named because the profile is per class: a travel worked out on
/// the legs' generator and asserted about an antenna is an instant off a
/// machine nobody commissioned.
#[must_use]
pub fn travel_cycles(group: reachy_motion::joints::JointGroup, distance_rad: f64) -> i64 {
    let plant = reachy_motion::plant::GroupPlants::default().of(group);
    i64::try_from(plant.travel_cycles(distance_rad)).unwrap_or(i64::MAX)
}

/// The joint angles a posture's Cartesian targets put the machine at.
///
/// The tick's own composition -- its configured geometry, and the crank angles
/// the envelope check itself selects -- so the travel a scenario derives is the
/// travel the machine is actually asked for. Anything the tick changes about how
/// a posture becomes nine angles moves these instants with it, which is what
/// keeps the suite's timing premises the machine's rather than a second opinion
/// about it.
fn posture_joints(
    targets: &reachy_motion::joints::JointTargets,
) -> reachy_motion::joints::JointVector {
    reachy_motion::tick::joints_of(reachy_motion::tick::default_motion_config(), targets)
        .expect("a canonical posture is reachable")
}

/// Where every bus row stands on each cycle of a posture move, and the angles
/// the move ends on.
///
/// The walk itself, for a scenario that has something to say about the middle
/// of a move rather than about its end -- where a joint stands when a hand is
/// laid on it, and how far it still had to travel from there.
pub struct PostureWalk {
    /// The modelled position of every row, indexed by cycles since the move was
    /// commanded. The first [`LAG_K`] entries are the posture the machine set
    /// off from, because a goal is dated that far ahead of the sample that
    /// decided it.
    pub positions: Vec<[f64; reachy_motion::joints::ROW_COUNT]>,
    /// The angles the move puts each row on, which is what an arrival is
    /// measured against.
    pub targets: [f64; reachy_motion::joints::ROW_COUNT],
}

impl PostureWalk {
    /// The cycle every bus row is standing on its target by, in one pass over
    /// the walk.
    ///
    /// One pass rather than nine: this is what a scenario places its instants
    /// against, and the walk is the suite's own startup cost.
    ///
    /// # Panics
    ///
    /// If the plant never finishes the move, which is a bug in the walk rather
    /// than a slow machine.
    #[must_use]
    pub fn arrivals(&self) -> [i64; reachy_motion::joints::ROW_COUNT] {
        let mut arrived = [None; reachy_motion::joints::ROW_COUNT];
        for (cycle, positions) in self.positions.iter().enumerate().skip(LAG_K as usize) {
            for (row, arrived) in arrived.iter_mut().enumerate() {
                if arrived.is_none() && positions[row] == self.targets[row] {
                    *arrived = Some(
                        i64::try_from(cycle).expect("a posture move is not a century of cycles"),
                    );
                }
            }
            if arrived.iter().all(Option::is_some) {
                break;
            }
        }
        arrived.map(|cycle| {
            cycle.unwrap_or_else(|| panic!("the plant does not finish a posture move at all"))
        })
    }

    /// The cycle the whole machine is standing on the posture by: the last row
    /// to arrive, which on every posture move is an antenna.
    ///
    /// # Panics
    ///
    /// As [`PostureWalk::arrivals`] does.
    #[must_use]
    pub fn travel(&self) -> i64 {
        self.arrivals()
            .into_iter()
            .max()
            .expect("the machine has rows")
    }

    /// The same for the head alone -- the cranks and the body yaw.
    ///
    /// Its own figure because the head arrives long before the antennas do: a
    /// scenario about where the *head* stands part way through a move measures
    /// its instants against this, and one measured against the antennas'
    /// arrival would place them on a head that had finished.
    ///
    /// # Panics
    ///
    /// As [`PostureWalk::arrivals`] does.
    #[must_use]
    pub fn head_travel(&self) -> i64 {
        let arrivals = self.arrivals();
        reachy_motion::joints::ROWS
            .into_iter()
            .enumerate()
            .filter(|(_, joint)| {
                reachy_motion::joints::group_of(*joint)
                    != Some(reachy_motion::joints::JointGroup::Antennas)
            })
            .map(|(row, _)| arrivals[row])
            .max()
            .expect("the head has rows")
    }
}

/// The whole stepped walk of a posture move.
///
/// Stepped rather than solved: the plant is handed the move's own setpoint
/// stream, period by period, exactly as the driver hands it to the modelled
/// servos, and each row's answer is where it stands on that period. That is what
/// makes these the machine's own numbers -- a joint chasing a min-jerk goal
/// spends its first cycles slower than the profile allows, because the goal is,
/// so it saturates late and arrives later than a straight-line travel of the
/// same distance would. Two things the postures alone do not say are in it: an
/// antenna routed the long way round travels further than the difference
/// between the angles the postures name, and the clock the planner floors the
/// move onto is not the clock it was asked for.
///
/// The commanded lag every goal carries is the padding at the head of the walk;
/// the response delay the servos answer a setpoint at is in the walk itself.
///
/// # Panics
///
/// If the move is one this machine will not run, which for the canonical
/// postures would be a geometry change rather than a scenario's mistake, or if
/// the plant does not finish it at all.
#[must_use]
pub fn posture_walk(
    from: &reachy_motion::joints::JointTargets,
    to: &reachy_motion::joints::JointTargets,
    duration_ns: i64,
) -> PostureWalk {
    let path = motion_cogs::planned_path(
        reachy_motion::tick::default_motion_config(),
        from,
        motion_cogs::Goal {
            target: *to,
            durations: reachy_motion::traj::MoveDurations::uniform(
                core::time::Duration::from_nanos(
                    u64::try_from(duration_ns).expect("a configured duration is a duration"),
                ),
            ),
        },
        1e9 / PERIOD_NS as f64,
    )
    .unwrap_or_else(|refusal| {
        panic!("a canonical posture move is one this machine runs: {refusal}")
    });

    let plants = reachy_motion::plant::GroupPlants::default();
    let standing = posture_joints(from).joints().map(|(_, angle)| angle);
    let ends = posture_joints(path.target())
        .joints()
        .map(|(_, angle)| angle);
    let mut state = standing.map(|angle| reachy_motion::plant::Predicted {
        position: angle,
        velocity: 0.0,
    });
    // The setpoints the servos have been handed and not yet answered, oldest
    // first: the response delay, stepped the way the plant and the decision
    // tick's model of it both step it -- read, then push.
    let mut held = [standing; reachy_motion::plant::RESPONSE_DEAD_SAMPLES];
    let mut positions = vec![standing; LAG_K as usize];
    let mut sampled = *from;
    for cycle in 0..MAX_TRAVEL_CYCLES {
        for (row, predicted) in state.iter_mut().enumerate() {
            plants.for_row(row).step(predicted, held[0][row]);
        }
        positions.push(state.map(|predicted| predicted.position));
        if state
            .iter()
            .zip(ends)
            .all(|(predicted, end)| predicted.position == end)
        {
            break;
        }
        assert!(
            cycle + 1 < MAX_TRAVEL_CYCLES,
            "the plant does not finish a posture move in {MAX_TRAVEL_CYCLES} cycles",
        );
        path.sample(
            core::time::Duration::from_nanos(
                u64::try_from(cycle * PERIOD_NS).expect("a cycle count is not a century"),
            ),
            &mut sampled,
        );
        let commanded = posture_joints(&sampled).joints().map(|(_, angle)| angle);
        held.rotate_left(1);
        held[reachy_motion::plant::RESPONSE_DEAD_SAMPLES - 1] = commanded;
    }
    PostureWalk {
        positions,
        targets: ends,
    }
}

/// The stepped walk of the raise, walked once per process.
///
/// Every instant the suite derives from the way up reads this one walk. The
/// walk itself is a full plan -- trajectory, envelope check and inverse
/// kinematics -- and then a per-cycle step of the plant, and the accessors over
/// it are called from authors, checkers and their own guards several times
/// each; without this the suite's startup cost would grow with the number of
/// derived instants rather than with the number of distinct walks, of which
/// there are two.
///
/// # Panics
///
/// As [`posture_walk`] does.
#[must_use]
pub fn up_walk() -> &'static PostureWalk {
    static WALK: std::sync::OnceLock<PostureWalk> = std::sync::OnceLock::new();
    WALK.get_or_init(|| {
        posture_walk(
            &reachy_motion::postures::stow_pose_targets(),
            &reachy_motion::postures::neutral_targets(),
            UP_DURATION_NS,
        )
    })
}

/// The stepped walk of the fold, walked once per process: the same distance on
/// a longer clock.
///
/// # Panics
///
/// As [`posture_walk`] does.
#[must_use]
pub fn stow_walk() -> &'static PostureWalk {
    static WALK: std::sync::OnceLock<PostureWalk> = std::sync::OnceLock::new();
    WALK.get_or_init(|| {
        posture_walk(
            &reachy_motion::postures::neutral_targets(),
            &reachy_motion::postures::stow_pose_targets(),
            STOW_DURATION_NS,
        )
    })
}

/// How long the walk above may run before it is a bug rather than a slow
/// machine: two minutes of cycles, against the longest posture move's three
/// seconds.
const MAX_TRAVEL_CYCLES: i64 = 6_000;

/// How many cycles after it is commanded the machine is standing upright.
#[must_use]
pub fn up_travel() -> i64 {
    up_walk().travel()
}

/// The same for the head alone, on the way up.
#[must_use]
pub fn head_up_travel() -> i64 {
    up_walk().head_travel()
}

/// The same for the fold, which travels the same distance on a longer clock.
#[must_use]
pub fn stow_travel() -> i64 {
    stow_walk().travel()
}

/// The longer of the two, which is what a step carrying either has to allow.
#[must_use]
pub fn posture_travel() -> i64 {
    up_travel().max(stow_travel())
}

/// How many cycles pass between the driver holding a setpoint and the first
/// reading that shows a servo answering it: the response delay the plant and the
/// decision tick's model of it are both stepped at.
#[must_use]
pub fn response_delay_cycles() -> i64 {
    i64::try_from(reachy_motion::plant::RESPONSE_DEAD_SAMPLES)
        .expect("the response delay is a couple of cycles")
}

/// How many cycles a modelled servo takes to reach its profile velocity from
/// rest, and to come back to rest from it.
///
/// The ramp the configured acceleration gives, rounded up. What a scenario
/// wants it for is a goal that turns round under a moving joint: the joint keeps
/// going the way it was for this many cycles after the setpoint reverses,
/// whatever the setpoint says, because that is how long its own generator takes
/// to bring the speed through zero. Per class, as the profile is.
#[must_use]
pub fn ramp_cycles(group: reachy_motion::joints::JointGroup) -> i64 {
    let plant = reachy_motion::plant::GroupPlants::default().of(group);
    (plant.v_max / plant.a_max).ceil() as i64
}

/// How many cycles a modelled servo setting off from rest takes to stand at or
/// past `distance_rad`.
///
/// The ramp alone, and no response delay: what a scenario wants it for is a
/// joint a hand has just come off, and a release changes no setpoint, so there
/// is nothing for a dead time to delay. [`travel_cycles`] is the other
/// question -- a commanded arrival, rest to rest, whose setpoint really does
/// take a dead time to be answered. Per class, as the profile is.
#[must_use]
pub fn pass_cycles(group: reachy_motion::joints::JointGroup, distance_rad: f64) -> i64 {
    let plant = reachy_motion::plant::GroupPlants::default().of(group);
    i64::try_from(plant.pass_cycles(distance_rad)).unwrap_or(i64::MAX)
}

/// How many cycles a generator running at its profile velocity takes to open
/// the tracking screen's own distance between itself and a joint that stopped.
///
/// The raise latency's first term: a run opens on the tick the residual passes
/// `threshold_rad` and the fault comes `ticks` ticks later, so a jam shorter
/// than this raises nothing at all whatever the goal was doing, and one longer
/// than this plus the window raises. A scenario placing a hand on the machine
/// says which of the two it is by this figure rather than by an integer. Per
/// class, as the profile is.
#[must_use]
pub fn crossing_cycles(group: reachy_motion::joints::JointGroup) -> i64 {
    let cfg = reachy_motion::tick::default_motion_config();
    (cfg.tracking.threshold_rad / cfg.plant.of(group).v_max).ceil() as i64
}

/// The three instants a hand laid on the rows of a moving posture move decides.
///
/// Derived together because each is the next one's premise: where the hand lands
/// decides when the residual passes the screen, which decides when the fault
/// comes.
#[derive(Clone, Copy, Debug)]
pub struct Jam {
    /// The cycle of the move the rows are held from.
    pub jam: i64,
    /// The cycle the residual first stands past the detector's threshold, which
    /// is the cycle the run opens on.
    pub crossing: i64,
    /// The cycle the window runs out on, which is the cycle the fault is raised
    /// on.
    pub raise: i64,
}

/// The rows a hand laid on the head holds: the six cranks that carry it.
///
/// The whole group rather than one of them. A single frozen crank leaves the
/// platform in a shape the linkage cannot take, which the estimator reports as
/// a pose it cannot solve -- a real presentation, and a different scenario's
/// subject. Freezing all six holds the head exactly where it stood, so what a
/// run placing this hand is about is the tracking evidence and nothing else.
///
/// Shared rather than restated by each run that places such a hand, because the
/// two that do have to place the *same* hand: one recovers inside the window its
/// first raise opens and the other never lets go, and what makes them the same
/// run up to that instant is this set and the placement
/// [`jam_on_the_raise`] derives from it.
#[must_use]
pub fn head_jam_rows() -> brenn_reachy__motion__joints_clk_rs::JointFlags {
    reachy_motion::joints::JointGroup::Legs.joints()
}

/// Where a hand has to go on the raise for the detector to answer it, and what
/// the detector makes of it, read off the stepped walk of that move.
///
/// Two conditions decide it, and both are conditions on the plant rather than
/// preferences of any one scenario.
///
/// The held rows have to be far enough short of their target at the jam for
/// their generator to travel the screen's own distance after it: a hand laid on
/// a joint that had nearly arrived opens no run at all.
///
/// And their generator has to have *stopped* by the time the fault lands, a
/// response delay before it, so the reading the run reopens against is a joint
/// standing beside a trajectory that has come to rest. That is what makes a
/// release recoverable, for the run that lets go: the reopened window is
/// restarted by a released joint regaining the progress minimum, which takes
/// three steps of its ramp, while a generator still running at the profile is
/// one a joint setting off from rest cannot pace before the window runs out.
///
/// The earliest jam satisfying the second condition is the one taken, which is
/// the one that leaves the most distance for the first: it puts the residual
/// well past the screen rather than a hundredth of a radian past it.
///
/// # Panics
///
/// If no cycle of the raise satisfies both, which would be a move, a screen or a
/// profile a run can no longer be stated over rather than a number to nudge.
#[must_use]
pub fn jam_on_the_raise(rows: brenn_reachy__motion__joints_clk_rs::JointFlags) -> Jam {
    // Derived once per row-set. Both scenarios that place a hand read every
    // instant of it several times, and the derivation is a search over the
    // walk; the answer is a function of the row-set, the walk, the screen and
    // the profile, all of which are fixed within a process.
    static DERIVED: std::sync::OnceLock<
        std::sync::Mutex<
            std::collections::BTreeMap<brenn_reachy__motion__joints_clk_rs::JointFlags, Jam>,
        >,
    > = std::sync::OnceLock::new();
    let derived = DERIVED.get_or_init(|| std::sync::Mutex::new(std::collections::BTreeMap::new()));
    if let Some(jam) = derived
        .lock()
        .expect("the suite is not holding a poisoned derivation")
        .get(&rows)
    {
        return *jam;
    }
    let jam = derive_jam_on_the_raise(rows);
    derived
        .lock()
        .expect("the suite is not holding a poisoned derivation")
        .insert(rows, jam);
    jam
}

/// [`jam_on_the_raise`]'s search, run once per row-set.
///
/// # Panics
///
/// As [`jam_on_the_raise`] does.
fn derive_jam_on_the_raise(rows: brenn_reachy__motion__joints_clk_rs::JointFlags) -> Jam {
    let cfg = reachy_motion::tick::default_motion_config();
    let walk = up_walk();
    let held: Vec<usize> = reachy_motion::joints::flags::iter(rows)
        .map(|joint| reachy_motion::joints::row(joint).expect("a jammed joint sits on a bus row"))
        .collect();
    let cycles = i64::try_from(walk.positions.len()).expect("a move is not a century of cycles");
    let arrivals = walk.arrivals();
    let arrival = held
        .iter()
        .map(|&row| arrivals[row])
        .max()
        .expect("a hand is laid on at least one row");
    // How far the worst-placed row's generator has moved on since the hand
    // landed, which is the residual the detector screens on: a held joint's
    // reading does not move, so the whole of the distance is the trajectory's.
    //
    // Measured from the cycle *before* the jam: the driver drains what arrived
    // and then advances the plant, so the reading the sample for the jam's own
    // cycle carries is where the row stood on the cycle before it.
    let residual = |from: i64, at: i64| {
        held.iter()
            .map(|&row| {
                (walk.positions[at as usize][row] - walk.positions[from as usize][row]).abs()
            })
            .fold(0.0f64, f64::max)
    };
    for jam in 1..cycles {
        let Some(crossing) =
            (jam..cycles).find(|&at| residual(jam - 1, at) > cfg.tracking.threshold_rad)
        else {
            break;
        };
        let raise = crossing + i64::from(cfg.tracking.ticks) - 1;
        if arrival + response_delay_cycles() <= raise {
            return Jam {
                jam,
                crossing,
                raise,
            };
        }
    }
    panic!("no cycle of the raise both opens a run on the held rows and settles their generator")
}

/// How much room an arrival assertion gets past the travel the plant needs.
///
/// The travel figure is the cycle the last joint reached its target on; this is
/// the room a scenario leaves past it, so an arrival assertion is made on a
/// machine that has been standing still for a moment rather than on one that
/// arrived on the very cycle it is read.
pub const ARRIVAL_SETTLE_CYCLES: i64 = 5;

/// How long a step carrying a posture move has to run for the machine to be
/// asserted arrived on its last cycle.
///
/// The plant's travel and nothing else's: a step sized on the move's clock
/// would assert an arrival on a machine still climbing toward it.
#[must_use]
pub fn posture_step_cycles() -> i64 {
    posture_travel() + ARRIVAL_SETTLE_CYCLES
}

/// How many cycles a move to the upright posture is given, rounded up.
///
/// The configured clock, which is the head group's: both postures sweep the
/// antenna pair mirrored, so the later side's clock is lengthened to part their
/// tips before the move is commanded, and that side arrives later than this. It
/// is what a scenario places a fault or an injection against, since that is the
/// number it wrote down; what a step span has to cover is every group, which is
/// [`up_clocks`].
#[must_use]
pub fn up_cycles() -> i64 {
    cycles_for(UP_DURATION_NS)
}

/// How many cycles a move to stow is given, rounded up. The head group's clock,
/// on the same terms as [`up_cycles`].
#[must_use]
pub fn stow_cycles() -> i64 {
    cycles_for(STOW_DURATION_NS)
}

/// The clocks one posture move actually runs on, group by group.
///
/// The head's is the configured duration; each antenna's is what the mover's own
/// floor made of it, which on a move that sweeps the pair mirrored is longer, by
/// as much as parting their tips at the crossing takes. Derived rather than
/// stated so that a scenario's arithmetic and the shaping the cog performs are
/// one number: a configuration change that lengthens the parting moves both at
/// once.
pub struct MoveClocks {
    /// The head group's clock, nanoseconds.
    pub head_ns: i64,
    /// Each antenna's own floored clock, nanoseconds, right then left.
    pub antennas_ns: [i64; 2],
}

impl MoveClocks {
    /// The whole move, in cycles: the longest clock any group runs on.
    #[must_use]
    pub fn cycles(&self) -> i64 {
        cycles_for(
            self.head_ns
                .max(self.antennas_ns[0])
                .max(self.antennas_ns[1]),
        )
    }

    /// Which group the longest clock belongs to, for a failure that says what is
    /// short.
    #[must_use]
    pub fn longest_group(&self) -> &'static str {
        if self.head_ns >= self.antennas_ns[0].max(self.antennas_ns[1]) {
            "the head group"
        } else {
            "the later antenna"
        }
    }

    /// The fewest cycles apart the two antennas may be seen to stop moving on
    /// this move.
    ///
    /// The pair's own clocks part by a figure the geometry decides, and what a
    /// checker measures is not that figure: the detector calls a side stopped
    /// once its per-cycle travel falls under a threshold, and a min-jerk tail
    /// crosses that threshold before its clock runs out -- by a little more on
    /// the longer clock than on the shorter, since the longer one travels
    /// slower. So the parting is derived and an allowance for the two tails is
    /// subtracted, which is the one fudge in the figure. At least one cycle: two
    /// clocks that part at all part visibly on this grid, and a derivation that
    /// came back with no parting at all is itself the regression the floor is
    /// counted on to prevent -- so the answer stays a demand rather than
    /// becoming "nothing to assert".
    ///
    /// Which is the precondition: this is a figure about a move whose geometry
    /// parts the pair, the mirrored sweeps between the fold and the working
    /// posture. Asked about a move nothing de-phases -- one antenna alone, an
    /// unmirrored pair -- it answers one cycle and the check it feeds demands a
    /// parting that correctly never happens.
    #[must_use]
    pub fn parting_least(&self) -> i64 {
        let parted = (self.antennas_ns[0] - self.antennas_ns[1]).abs() / PERIOD_NS;
        (parted - PAIR_TAIL_ALLOWANCE_CYCLES).max(1)
    }
}

/// How many cycles of a min-jerk tail the de-phasing detector may not see, on
/// each side of a pair.
///
/// Not derived: it is the gap between "the clock ran out" and "the travel this
/// cycle fell under the detector's threshold", which depends on the threshold
/// and on the arc. Small, because the tail of a min-jerk profile is short, and
/// stated once so a pin that failed by a cycle is read as this number being
/// wrong rather than as the parting having gone.
const PAIR_TAIL_ALLOWANCE_CYCLES: i64 = 3;

/// The clocks the move to the upright posture runs on. Planned from the stow, in
/// which every scenario's machine starts.
#[must_use]
pub fn up_clocks() -> MoveClocks {
    posture_clocks(
        &reachy_motion::postures::stow_pose_targets(),
        &reachy_motion::postures::neutral_targets(),
        UP_DURATION_NS,
    )
}

/// The clocks the fold runs on: the same move back, on its own duration.
#[must_use]
pub fn stow_clocks() -> MoveClocks {
    posture_clocks(
        &reachy_motion::postures::neutral_targets(),
        &reachy_motion::postures::stow_pose_targets(),
        STOW_DURATION_NS,
    )
}

/// Why a step of `step_ns` ending in a hold at `clocks`' destination cannot be
/// judged for stillness, or `None` when it can. `what` names the step.
///
/// Two terms. The move is the first: it streams a new setpoint every cycle
/// until its longest clock runs out, and the watch's settle allowance only
/// starts running when the setpoint stops changing -- so the clock the pair is
/// actually parted onto, not the duration the move was asked for, is what a
/// step has to carry before the hold begins. The watch's own floor is the
/// second, and it states that itself.
///
/// The figure is a plan's, so it says nothing about a servo still travelling
/// after the last setpoint was written; that is what the settle allowance is
/// for.
#[must_use]
pub fn unjudgeable_step(
    what: &str,
    step_ns: i64,
    clocks: &MoveClocks,
    watch: &reachy_motion::stillness::StillnessConfig,
) -> Option<String> {
    let move_ns = clocks.cycles() * PERIOD_NS;
    let hold_ns = i64::try_from(watch.shortest_judgeable_hold().as_nanos())
        .expect("the watch's allowances are seconds, not centuries");
    let floor_ns = move_ns + hold_ns;
    (step_ns < floor_ns).then(|| {
        format!(
            "the {what} step runs {} ms and a hold in it can be judged only if it runs {} ms: \
             {} ms of it is {} still being written new setpoints, and the watch spends its \
             settle allowance after that before it looks and then wants the shortest stretch \
             it will call a hold",
            step_ns / 1_000_000,
            floor_ns / 1_000_000,
            move_ns / 1_000_000,
            clocks.longest_group(),
        )
    })
}

/// One posture move's clocks, floored by the same pass the mover floors its own
/// base moves with.
///
/// Two premises, stated because they are what makes the figure the cog's own
/// number rather than a second guess at it. The shaping config is the library's
/// defaults, which is what the mover runs on -- it has no configured
/// `MotionConfig`, only the durations `check_params` pins -- and the move is
/// planned from the canonical posture, which is where every engagement in this
/// suite starts settled. A scenario that measured a move begun off-posture would
/// get clocks for a different move, and nothing here would say so.
fn posture_clocks(
    from: &reachy_motion::joints::JointTargets,
    to: &reachy_motion::joints::JointTargets,
    duration_ns: i64,
) -> MoveClocks {
    let floored = motion_cogs::floored_clocks(
        reachy_motion::tick::default_motion_config(),
        from,
        motion_cogs::Goal {
            target: *to,
            durations: reachy_motion::traj::MoveDurations::uniform(
                core::time::Duration::from_nanos(
                    u64::try_from(duration_ns).expect("a configured duration is a duration"),
                ),
            ),
        },
        1e9 / PERIOD_NS as f64,
    );
    MoveClocks {
        head_ns: nanos(floored.head),
        antennas_ns: [nanos(floored.antennas[0]), nanos(floored.antennas[1])],
    }
}

/// A duration as the count of nanoseconds every figure here is in.
fn nanos(duration: core::time::Duration) -> i64 {
    i64::try_from(duration.as_nanos()).expect("a move's clock is a count of nanoseconds")
}

/// How long a whole lap of the driver's rotating read takes, in cycles.
///
/// One row per report at the configured cadence, over every row of the bus. The
/// outer bound on how long a standing condition can go unread: the rotation is
/// somewhere in its lap when a servo's error byte is written, so the faulted row
/// is read within one lap of that.
#[must_use]
pub fn health_lap_cycles() -> i64 {
    reachy_motion::joints::ROW_COUNT as i64 * cycles_for(HEALTH_POLL_PERIOD_NS)
}

/// The cycle the session must have answered a condition written on
/// `fault_cycle` by.
///
/// The read that carries it, plus the wake the report causes. An outer bound and
/// not an expectation: which cycle the rotation reaches the faulted row on is a
/// fact about the run, so what a checker asserts against the narration is that
/// the answer landed inside this, and everything a scenario places afterwards is
/// placed from here.
#[must_use]
pub fn answered_within(fault_cycle: i64) -> i64 {
    fault_cycle + health_lap_cycles() + 1
}

/// The committed name-to-number sidecar the clip-config emitter writes.
const CLIP_LIBRARY_NAMES: &str = include_str!("clip_library.names.json");

/// The number the committed clip library gives the motion called `name`.
///
/// The numbering is generated and positional -- an asset inserted in the middle
/// renumbers every one after it -- so a scenario that names a motion reads its
/// number out of the same sidecar the emitter writes rather than restating it. A
/// renumber then moves the scenario with it, and a motion nobody committed fails
/// the run naming the motion instead of failing an assertion about where the
/// antennas ended up.
///
/// # Panics
///
/// If the sidecar is not the emitter's JSON, or carries no motion of that name.
#[must_use]
pub fn motion_id(name: &str) -> u16 {
    motion_table()
        .resolve(name)
        .unwrap_or_else(|| panic!("the committed clip library carries no motion named {name}"))
        .motion_id
}

/// The committed sidecar as the intent edge reads it.
///
/// The edge's own reader rather than a walk of the JSON here: the sidecar is one
/// artifact with one shape, and a second reader of it is a second thing to
/// update when the emitter grows a field -- the one that keeps compiling while
/// it is wrong.
fn motion_table() -> reachy_edge::MotionTable {
    reachy_edge::MotionTable::from_sidecar(CLIP_LIBRARY_NAMES)
        .expect("the committed sidecar is the emitter's")
}

/// How many cycles the goal stream may be silent before the gate de-torques.
#[must_use]
pub fn hold_timeout_cycles() -> i64 {
    HOLD_TIMEOUT_NS / PERIOD_NS
}

/// The cycle the driver's gate latches its torque-off on, given the cycle the
/// silence it measures began on.
///
/// The gate compares the silence it has measured against the configured timeout
/// and latches once it is *past* it, so the latch lands one cycle further out
/// than the timeout itself.
#[must_use]
pub fn dead_man_latch_cycle(window_opened_at: i64) -> i64 {
    window_opened_at + hold_timeout_cycles() + 1
}

/// How long a stretch of cycles is, nanoseconds: what the gate reports as the
/// silence it measured.
///
/// Nothing for a stretch that ran backwards, the way the gate's own reading is:
/// two spellings of one arithmetic that disagreed at the low end would have a
/// scenario asserting a silence against a driver reporting none, and send
/// whoever read the failure to the driver rather than to the arithmetic that
/// asked for it.
#[must_use]
pub fn silence_ns(from_cycle: i64, to_cycle: i64) -> i64 {
    (to_cycle - from_cycle).max(0) * PERIOD_NS
}

/// The cycle whose interval contains `at_ns`.
///
/// For the instants that are not on the grid: the session runs when a message
/// reaches it rather than on the bus cycle, so what it stamps a report with sits
/// wherever in a cycle the message that woke it landed. A cycle number is still
/// what a scenario reasons in, and this is the cycle the machine was in.
#[must_use]
pub fn cycle_within(at_ns: i64) -> i64 {
    (at_ns - T0_NS).div_euclid(PERIOD_NS)
}

/// The cycle a message logged at `at_ns` is first visible to the driver on.
///
/// The driver runs once per cycle and sees what was published before it started,
/// so a datagram published inside a cycle is drained by the next one. The first
/// cycle at or after the instant, which is the same arithmetic an injection's
/// drain cycle is.
#[must_use]
pub fn drain_cycle(at_ns: i64) -> i64 {
    cycles_for(at_ns - T0_NS)
}

/// How many transactions the start-up survey spends.
///
/// Derived from the sweeps the sequence walks rather than measured: a ping and a
/// model read per servo, the cells the host's provisioning table asks to be read
/// (the rest of the grid is skipped without a transaction), a supply reading per
/// servo, an error byte per servo, and the gains and profile writes. A register
/// added to any of those sweeps, or a cell added to the table, widens this with
/// it.
///
/// An expectation and not an allowance: a survey that spent more than this
/// re-issued something, which in a system whose channels are memory is a
/// regression in the host's own delivery timing.
#[must_use]
pub fn commission_transactions() -> i64 {
    let rows = reachy_motion::joints::ROW_COUNT as i64;
    let provision = motion_cogs::session_bus::provision_table().reads() as i64;
    let gains = reachy_motion::resume::GAINS_PROFILE_WRITES as i64;
    4 * rows + provision + gains
}

/// How long a run must allow for the start-up survey, in cycles.
///
/// An allowance and not an expectation. One transaction costs a publish, the
/// driver cycle that runs it, and the wake its answer causes -- two cycles where
/// nothing else is in the way -- and this allows three, which covers the supply
/// gate's spacing and the cycles the aux slot spends on its own health rotation.
/// A scenario that wants to know the survey *finished* asserts that, rather than
/// counting on this.
#[must_use]
pub fn commission_allowance_cycles() -> i64 {
    3 * commission_transactions()
}

/// How many driver cycles taking hold of the machine costs.
///
/// Two: one for the driver to act on the ask, one for its answer to arrive.
/// The engagement issues no aux transaction at all -- the torque-on gate is
/// arithmetic over the rail picture the session keeps.
#[must_use]
pub fn engage_cycles() -> i64 {
    2
}

/// How many aux transactions the fallback rail re-read spends over `rows` rows.
///
/// Two per row -- the supply and the error byte -- and only for the rows the
/// driver's health rotation has left older than `rail_stale_after_ns`. Zero on
/// an engagement whose picture is fresh, which is every engagement in normal
/// operation.
#[must_use]
pub fn rail_watch_transactions(rows: i64) -> i64 {
    2 * rows
}

/// How many cycles a run must allow for taking hold of the machine.
///
/// An allowance and not an expectation, on [`commission_allowance_cycles`]'s
/// arithmetic and for its reasons: the two cycles above, plus a whole fallback
/// re-read of every row in case the picture has gone stale, at three cycles a
/// transaction. A scenario that wants to know the arming *finished* asserts the
/// phase it ended in, and every instant derived from this is an outer bound on
/// when the machine can be under command.
#[must_use]
pub fn engage_allowance_cycles() -> i64 {
    3 * (engage_cycles() + rail_watch_transactions(reachy_motion::joints::ROW_COUNT as i64))
}

/// How many cycles a run must allow for the orderly release.
///
/// The settle first -- the release waits under held torque with nothing
/// streaming, which the keep-alive rule is what carries -- and then two sweeps
/// of the bus: every joint measured against the stow pose, and then torque
/// written off one servo at a time with each write read back. An allowance, on
/// the same arithmetic as the two above.
#[must_use]
pub fn release_allowance_cycles() -> i64 {
    let dwell = i64::try_from(reachy_motion::disarm::DEFAULT_STOW_DWELL.as_nanos())
        .expect("a dwell this clock can hold");
    cycles_for(dwell) + 6 * reachy_motion::joints::ROW_COUNT as i64
}

/// The cycle a scenario may first expect a script to be taken.
///
/// The session commissions the machine before it will take one, so the first
/// instant a script is answered with anything but a refusal is past the survey's
/// allowance. Every scenario that sends a script sends it here, so what the run
/// is about begins from one number.
#[must_use]
pub fn script_cycle() -> i64 {
    commission_allowance_cycles()
}

/// The cycle a scenario's run begins on: the epoch itself, which is where the
/// world the scenario states is stated.
///
/// The driver's first cycle is one period in, because its timer fires a period
/// after the run starts -- [`FIRST_CYCLE`] is that one.
pub const START_CYCLE: i64 = 0;

/// How long a run continues past the release concluding, in cycles.
///
/// Long enough for the driver's dead-man to have fired if the machine had merely
/// been left alone: the goal stream stops when the session lets go and so do the
/// keep-alives, so a tail this long is what makes "no event at all" an assertion
/// about the release having taken torque off rather than about the run ending
/// before the timeout could.
pub const TAIL_CYCLES: i64 = 20;

/// The cycle a machine whose script was taken is armed and holding by.
///
/// A whole arming allowance after the script, so what a scenario's first step
/// opens on is a cycle it named rather than whichever cycle the arming happened
/// to finish on -- which is what makes every instant placed relative to that
/// step exact. What the machine does in between is nothing: it is armed and
/// holding where it stood, and the goal stream has not started because no step
/// covers those instants yet.
#[must_use]
pub fn armed_cycle() -> i64 {
    script_cycle() + engage_allowance_cycles()
}

/// The last cycle of a run whose session let go at `disengage`.
///
/// The release after the schedule -- the settle under held torque, every joint
/// measured against the stow pose, then torque written off one servo at a time
/// -- and then the tail. The shape of an ordinary run's ending, stated once:
/// when an allowance changes shape, every scenario's end moves with it.
#[must_use]
pub fn run_end_cycle(disengage: i64) -> i64 {
    disengage + release_allowance_cycles() + TAIL_CYCLES
}

/// The cycle `nominal` sits on, counted from the epoch.
///
/// # Errors
///
/// How far off the grid `nominal` sits. Every instant in a deterministic run is
/// on it, so an off-grid one is a run that drifted -- and this is fed log data,
/// so it is the run under test that says so, not the caller. Returned rather
/// than thrown because a checker collects every failure: a drifted run is the
/// one whose other complaints explain why it drifted, and a panic here would
/// throw them away.
pub fn cycle_of(nominal_ns: i64) -> Result<i64, String> {
    let elapsed = nominal_ns - T0_NS;
    let off = elapsed % PERIOD_NS;
    if off != 0 {
        return Err(format!(
            "{nominal_ns} is {off}ns off the {PERIOD_NS}ns grid the run is on"
        ));
    }
    Ok(elapsed / PERIOD_NS)
}

/// The instant cycle `n` begins.
#[must_use]
pub fn cycle_at(n: i64) -> i64 {
    T0_NS + n * PERIOD_NS
}

/// The configuration files a checker screens, bound by the name each file
/// carries.
///
/// The harness hands them over as a list of runfiles paths, and the list's order
/// is nobody's meaning: a file added to it, or two of them swapped for
/// tidiness, would otherwise re-bind every scenario's screen and read as a file
/// stating the wrong number rather than as a list in the wrong order. So the
/// binding is by file name, once, here.
pub struct ConfigPaths<'a> {
    /// `mover_params.textproto`.
    pub mover: &'a str,
    /// `servo_profile.textproto`.
    pub profile: &'a str,
    /// `servo_gains.textproto`.
    pub gains: &'a str,
    /// `session_params.textproto`.
    pub session: &'a str,
    /// `sim_params.textproto`.
    pub sim: &'a str,
    /// `driver/motord_params.textproto`.
    pub motord: &'a str,
}

impl<'a> ConfigPaths<'a> {
    /// Pick each file out of the paths the harness handed over, by its name.
    ///
    /// # Errors
    ///
    /// One line per file the list does not carry.
    pub fn of(paths: &'a [String]) -> Result<Self, Vec<String>> {
        let mut missing = Vec::new();
        let mut named = |name: &str| -> &'a str {
            let found = paths
                .iter()
                .map(String::as_str)
                .find(|path| path.rsplit('/').next() == Some(name));
            match found {
                Some(path) => path,
                None => {
                    missing.push(format!("the checker was handed no {name}"));
                    ""
                }
            }
        };
        let paths = Self {
            mover: named("mover_params.textproto"),
            profile: named("servo_profile.textproto"),
            gains: named("servo_gains.textproto"),
            session: named("session_params.textproto"),
            sim: named("sim_params.textproto"),
            motord: named("motord_params.textproto"),
        };
        if missing.is_empty() {
            Ok(paths)
        } else {
            Err(missing)
        }
    }
}

/// The configured numbers above, as they are written in the textprotos the box
/// binds.
///
/// The constants in this module and the files the process reads are two
/// statements of the same numbers, and a scenario asserting "the goal is due two
/// cycles out" against a build configured for three would pass while describing
/// a machine nobody ran. Every checker calls this with the paths the test target
/// hands it, so a change to either side fails the scenario rather than shifting
/// what it means.
///
/// The parse is deliberately literal -- `key: value` lines, comments and blanks
/// skipped -- because it is checking a handful of scalars in a file this repo
/// writes, not implementing protobuf text. The values are compared as the
/// numbers they are rather than as the characters they were written with: what
/// the process reads is `0.15`, whether the file spells it `0.15` or `1.5e-1`,
/// and a check that failed over the spelling would send its next reader to edit
/// the constant.
///
/// The driver's own configuration is among the files even though no scenario
/// runs that driver: its period is the third statement of the one grid every
/// process on the machine is built for, and the plant model both the tick and
/// the simulated driver step is a distance per period of it.
///
/// # Errors
///
/// One line per number the file states differently, or per number it does not
/// state at all, or the reason the file could not be read.
pub fn check_params(paths: &ConfigPaths<'_>) -> Vec<String> {
    let &ConfigPaths {
        mover: mover_textproto,
        profile: profile_textproto,
        gains: gains_textproto,
        session: session_textproto,
        sim: sim_textproto,
        motord: motord_textproto,
    } = paths;
    let mut failures = Vec::new();
    expect(
        mover_textproto,
        &[
            ("lag_k", Value::Int(LAG_K)),
            ("period_ns", Value::Int(PERIOD_NS)),
            ("up_duration_ns", Value::Int(UP_DURATION_NS)),
            ("stow_duration_ns", Value::Int(STOW_DURATION_NS)),
            ("tracking_armed", Value::Bool(TRACKING_ARMED)),
        ],
        &mut failures,
    );
    expect(
        session_textproto,
        &[
            ("aux_timeout_ns", Value::Int(AUX_TIMEOUT_NS)),
            ("aux_retries", Value::Int(AUX_RETRIES)),
            ("sample_stale_after", Value::Int(SAMPLE_STALE_AFTER)),
            ("startup_grace_ns", Value::Int(STARTUP_GRACE_NS)),
            ("stow_budget_ns", Value::Int(STOW_BUDGET_NS)),
            (
                "torque_off_confirm_budget_ns",
                Value::Int(SESSION_CONFIRM_BUDGET_NS),
            ),
            ("bus_watchdog", Value::Int(BUS_WATCHDOG)),
            ("script_span_cap_ms", Value::Int(SCRIPT_SPAN_CAP_MS)),
            ("rail_stale_after_ns", Value::Int(RAIL_STALE_AFTER_NS)),
        ],
        &mut failures,
    );
    expect(
        profile_textproto,
        &[
            (
                "legs_profile_acceleration",
                Value::Int(LEGS_PROFILE_ACCELERATION),
            ),
            ("legs_profile_velocity", Value::Int(LEGS_PROFILE_VELOCITY)),
            (
                "body_yaw_profile_acceleration",
                Value::Int(BODY_YAW_PROFILE_ACCELERATION),
            ),
            (
                "body_yaw_profile_velocity",
                Value::Int(BODY_YAW_PROFILE_VELOCITY),
            ),
            (
                "antennas_profile_acceleration",
                Value::Int(ANTENNAS_PROFILE_ACCELERATION),
            ),
            (
                "antennas_profile_velocity",
                Value::Int(ANTENNAS_PROFILE_VELOCITY),
            ),
        ],
        &mut failures,
    );
    // The gains the session commissions each class of servo with, against the
    // motion library's own default. Not a second set of constants here: the
    // library states the triples, this file is what a deployment edits when a
    // rung is tuned, and the two disagreeing is a machine commissioned with
    // numbers no test was written against.
    let gains = reachy_motion::arm::DEFAULT_GAINS;
    expect(
        gains_textproto,
        &[
            ("legs_p", Value::Int(i64::from(gains.legs.p))),
            ("legs_i", Value::Int(i64::from(gains.legs.i))),
            ("legs_d", Value::Int(i64::from(gains.legs.d))),
            ("body_yaw_p", Value::Int(i64::from(gains.yaw.p))),
            ("body_yaw_i", Value::Int(i64::from(gains.yaw.i))),
            ("body_yaw_d", Value::Int(i64::from(gains.yaw.d))),
            ("antennas_p", Value::Int(i64::from(gains.antennas.p))),
            ("antennas_i", Value::Int(i64::from(gains.antennas.i))),
            ("antennas_d", Value::Int(i64::from(gains.antennas.d))),
        ],
        &mut failures,
    );
    // Every process on the machine is built for the one grid, and the plant
    // model's two limits are distances per period of it: a tick modelling a
    // 20 ms generator over a stream of some other spacing would judge every
    // joint against a trajectory nothing runs. The driver's own file is checked
    // here rather than by the driver because it is the third statement of the
    // same number and nothing else reads all three.
    expect(
        motord_textproto,
        &[("period_ns", Value::Int(PERIOD_NS))],
        &mut failures,
    );
    // The pairs the tests of the motion library are written against, and the
    // grid they assume, against the files the processes read. `GroupPlants`'
    // own default is built from these constants, so a file that moved away from
    // them would leave every scenario and every unit test screening a machine
    // no deployment runs. Per class, because a check that compared one pair
    // would pass a file that gave the antennas the legs' numbers.
    let shipped = reachy_motion::plant::SHIPPED_PROFILES;
    for (class, (acceleration, velocity), (expected_a, expected_v)) in [
        (
            "legs",
            shipped.legs,
            (LEGS_PROFILE_ACCELERATION, LEGS_PROFILE_VELOCITY),
        ),
        (
            "body yaw",
            shipped.yaw,
            (BODY_YAW_PROFILE_ACCELERATION, BODY_YAW_PROFILE_VELOCITY),
        ),
        (
            "antennas",
            shipped.antennas,
            (ANTENNAS_PROFILE_ACCELERATION, ANTENNAS_PROFILE_VELOCITY),
        ),
    ] {
        if i64::from(acceleration) != expected_a || i64::from(velocity) != expected_v {
            failures.push(format!(
                "the motion library ships the {class} at profile {acceleration}/{velocity} and \
                 the scenarios expect {expected_a}/{expected_v}",
            ));
        }
    }
    if reachy_motion::plant::SHIPPED_PERIOD_NS != PERIOD_NS {
        failures.push(format!(
            "the motion library models a {}ns period and the processes run on {PERIOD_NS}ns",
            reachy_motion::plant::SHIPPED_PERIOD_NS,
        ));
    }
    expect(
        sim_textproto,
        &[
            ("period_ns", Value::Int(PERIOD_NS)),
            ("hold_timeout_ns", Value::Int(HOLD_TIMEOUT_NS)),
            ("start_torqued", Value::Bool(START_TORQUED)),
            ("health_poll_period_ns", Value::Int(HEALTH_POLL_PERIOD_NS)),
        ],
        &mut failures,
    );
    // The session's staleness window is a count of nominal periods and its
    // configuration carries no period, so the host holds one as a constant. This
    // is what keeps that constant and the cycle the simulated driver is actually
    // built with from drifting apart -- a session watching a 20 ms stream
    // against a 10 ms assumption would declare a healthy driver dead.
    if motion_cogs::session_ladder::NOMINAL_PERIOD_NS != PERIOD_NS {
        failures.push(format!(
            "the session's assumed bus cycle is {}ns and the driver is built for {PERIOD_NS}ns",
            motion_cogs::session_ladder::NOMINAL_PERIOD_NS
        ));
    }
    // The two driver thresholds a scenario perturbs and then asserts about. They
    // are constants of the driver layer every host shares rather than
    // configuration, so what keeps a scenario's expectation from moving with the
    // number it is checking is this pair of comparisons: a threshold changed on
    // one side alone fails the run instead of quietly rewriting what S4' claims.
    if reachy_driver::BLIND_CYCLES_BEFORE_BUS_FAILURE != BLIND_CYCLES_BEFORE_BUS_FAILURE {
        failures.push(format!(
            "the driver declares its bus gone after {} blind cycles and the scenarios expect \
             {BLIND_CYCLES_BEFORE_BUS_FAILURE}",
            reachy_driver::BLIND_CYCLES_BEFORE_BUS_FAILURE
        ));
    }
    if reachy_driver::TORQUE_OFF_CONFIRM_BUDGET_NS != DRIVER_CONFIRM_BUDGET_NS {
        failures.push(format!(
            "the driver's confirmation budget is {}ns and the scenarios expect \
             {DRIVER_CONFIRM_BUDGET_NS}ns",
            reachy_driver::TORQUE_OFF_CONFIRM_BUDGET_NS
        ));
    }
    failures
}

/// One configured scalar, as the kind of value its field holds.
#[derive(Clone, Copy, PartialEq)]
enum Value {
    /// A whole number: a count of cycles or of nanoseconds.
    Int(i64),
    /// An angle, radians.
    Float(f64),
    /// A choice.
    Bool(bool),
}

impl Value {
    /// The same value, read out of the text a file states it as, or `None` if
    /// those characters are not one of these at all.
    fn parse(self, text: &str) -> Option<Self> {
        match self {
            Self::Int(_) => text.parse().ok().map(Self::Int),
            // Exact equality on the parsed number, which is what "the file
            // states this number" means: the process gets the parse, not the
            // characters, and any rounding is the same rounding on both sides.
            Self::Float(_) => text.parse().ok().map(Self::Float),
            Self::Bool(_) => text.parse().ok().map(Self::Bool),
        }
    }
}

impl core::fmt::Display for Value {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Int(value) => write!(f, "{value}"),
            Self::Float(value) => write!(f, "{value}"),
            Self::Bool(value) => write!(f, "{value}"),
        }
    }
}

/// Assert one textproto states exactly these values for these keys.
fn expect(path: &str, wanted: &[(&str, Value)], failures: &mut Vec<String>) {
    let text = match std::fs::read_to_string(path) {
        Ok(text) => text,
        Err(err) => {
            failures.push(format!("reading {path}: {err}"));
            return;
        }
    };
    let stated: Vec<(&str, &str)> = text
        .lines()
        .map(str::trim)
        .filter(|line| !line.is_empty() && !line.starts_with('#'))
        .filter_map(|line| line.split_once(':'))
        .map(|(key, value)| (key.trim(), value.trim()))
        .collect();
    for (key, value) in wanted {
        let Some((_, found)) = stated.iter().find(|(name, _)| name == key) else {
            failures.push(format!(
                "{path} states no {key}; the scenario needs {value}"
            ));
            continue;
        };
        match value.parse(found) {
            None => failures.push(format!(
                "{path} states {key}: {found}, which is not a value of that field's kind; the \
                 scenario is written for {value}"
            )),
            Some(parsed) if parsed != *value => failures.push(format!(
                "{path} states {key}: {found}, but the scenario is written for {value}"
            )),
            Some(_) => {}
        }
    }
}

#[cfg(test)]
mod tests {
    //! The two facts of this module nothing else reads back: what the committed
    //! name sidecar has to be for the reader that resolves a `play` step out of
    //! it at run time, and what a step has to run for a hold in it to be
    //! judged.
    //!
    //! A generated artifact and the code that parses it drift silently unless
    //! something reads the committed bytes, and the complaint a guard prints
    //! when it fires is unread prose unless a case asks what it says.

    use reachy_motion::stillness::StillnessConfig;

    use super::{
        AUX_RETRIES, AUX_TIMEOUT_NS, BUS_WATCHDOG, ConfigPaths, HEALTH_POLL_PERIOD_NS,
        HOLD_TIMEOUT_NS, LAG_K, PERIOD_NS, RAIL_STALE_AFTER_NS, SAMPLE_STALE_AFTER,
        SCRIPT_SPAN_CAP_MS, SESSION_CONFIRM_BUDGET_NS, START_TORQUED, STARTUP_GRACE_NS,
        STOW_BUDGET_NS, STOW_DURATION_NS, TRACKING_ARMED, UP_DURATION_NS, check_params,
        crossing_cycles, head_jam_rows, head_up_travel, jam_on_the_raise, motion_id, motion_table,
        posture_joints, response_delay_cycles, travel_cycles, unjudgeable_step, up_clocks,
        up_travel, up_walk,
    };

    /// The sidecar the emitter committed is the sidecar the edge's reader parses,
    /// windows and all.
    ///
    /// The emitter's own case pins the numbers against the documents; this one
    /// pins the shape against the reader that has to resolve a `play` step at
    /// run time, so a field added on one side fails in this tree rather than at
    /// the machine's startup.
    #[test]
    fn the_committed_sidecar_is_a_table_the_edge_can_resolve() {
        let table = motion_table();
        let tour = table
            .resolve("bench/tour")
            .expect("the committed library holds the composed motion");
        assert_eq!(tour.motion_id, motion_id("bench/tour"));
        assert_eq!(tour.window.duration_ms, 1701);
        assert_eq!(tour.window.blend_out_ms, 200);
    }

    /// The stepped walk every re-derived instant in the suite is an expression
    /// over lands where the plant's own closed form says it must: at or after
    /// it, and within a few cycles of it.
    ///
    /// The failure direction is the silent one. A walk that over-estimates --
    /// sampling the planned path at the wrong instant, pushing a setpoint
    /// before it read one, or a row that never registers its arrival and rides
    /// the walk's own ceiling -- makes every step longer and every arrival
    /// window wider, so the suite stays green while the assertions it is built
    /// from have lost their edges. Nothing else in the tree reads this
    /// arithmetic back.
    ///
    /// The distance is the long way round: the planner routes each antenna away
    /// from its outboard direction, so the fold-to-upright arc is a whole turn
    /// less the difference between the two postures' angles rather than that
    /// difference. Arrival is at or after the closed form because the closed
    /// form is a straight-line travel from rest to rest, and a joint chasing a
    /// min-jerk goal is slower than its profile for the first cycles because
    /// the goal is -- so it saturates late and arrives later. Two small terms
    /// pull the other way and nearly cancel it: the closed form is
    /// continuous-time and over-estimates the stepped plant by a cycle or
    /// three, and the walk counts from the cycle the move is commanded on
    /// rather than from the one the first setpoint is dated at. So the band is
    /// stated wide at the top and tight at the bottom.
    #[test]
    fn the_stepped_arrival_walk_agrees_with_the_plant_it_walks() {
        let stow = reachy_motion::postures::stow_pose_targets();
        let neutral = reachy_motion::postures::neutral_targets();
        let folded = posture_joints(&stow);
        let upright = posture_joints(&neutral);
        let arc = core::f64::consts::TAU - (upright.antennas[0] - folded.antennas[0]).abs();
        let closed_form = travel_cycles(reachy_motion::joints::JointGroup::Antennas, arc);

        let arrived = up_travel();
        assert!(
            arrived >= closed_form,
            "the walk has the machine upright at cycle {arrived}, before the {closed_form} \
             cycles the plant needs for {arc:.4} rad"
        );
        assert!(
            arrived <= closed_form + LAG_K + 12,
            "the walk has the machine upright at cycle {arrived}, well past the {closed_form} \
             cycles the plant needs for {arc:.4} rad: a row that never registered its arrival \
             would widen every arrival window in the suite"
        );

        // The row that arrives last is an antenna, which is the premise
        // `PostureWalk::travel` rests on.
        let arrivals = up_walk().arrivals();
        let last = reachy_motion::joints::ROWS
            .into_iter()
            .enumerate()
            .max_by_key(|(row, _)| arrivals[*row])
            .map(|(_, joint)| joint)
            .expect("the machine has rows");
        assert_eq!(
            reachy_motion::joints::group_of(last),
            Some(reachy_motion::joints::JointGroup::Antennas),
            "the last row up is {last:?}, not an antenna"
        );
    }

    /// The head is upright long before the antennas are, which is what makes
    /// the head's own travel figure a separate one.
    ///
    /// The premise a scenario about a schedule changing under a *moving head*
    /// rests on: measured against the whole machine's travel, such a retarget
    /// would land on a head that had finished the move.
    #[test]
    fn the_head_arrives_before_the_antennas_do() {
        let head = head_up_travel();
        let machine = up_travel();
        assert!(
            head < machine,
            "the head is up at cycle {head} and the machine at {machine}"
        );
    }

    /// The three instants a hand on the raise decides hold together, and the
    /// crossing is not sooner than a generator at the profile could reach.
    ///
    /// The derivation searches the walk for a jam that both opens a run and
    /// leaves the generator stopped by the fault, and two scenarios place their
    /// whole run against what it answers. Each instant is checked here against
    /// arithmetic the search does not do: the crossing cannot come sooner than
    /// the screen's distance at the cap, because nothing in the walk moves
    /// faster than that; the raise is a window after the crossing; and the
    /// jammed rows' unjammed arrival is a response delay or more before the
    /// raise, which is the premise a released joint's recovery rests on.
    #[test]
    fn the_hand_on_the_raise_opens_a_run_and_leaves_the_generator_stopped() {
        let cfg = reachy_motion::tick::default_motion_config();
        let hand = jam_on_the_raise(head_jam_rows());
        // Counted from the cycle before the jam, which is where the search
        // measures the residual from: the driver drains what arrived and then
        // advances the plant, so the sample published for a cycle carries the
        // position of the one before it.
        assert!(
            hand.crossing - (hand.jam - 1)
                >= crossing_cycles(reachy_motion::joints::JointGroup::Legs),
            "the residual passes the {} rad screen {} periods after the reading the jam froze, \
             and the fastest a generator can open that distance is {} periods at the profile \
             velocity",
            cfg.tracking.threshold_rad,
            hand.crossing - (hand.jam - 1),
            crossing_cycles(reachy_motion::joints::JointGroup::Legs)
        );
        assert_eq!(
            hand.raise,
            hand.crossing + i64::from(cfg.tracking.ticks) - 1,
            "the fault comes a window after the run opens"
        );
        let arrival = reachy_motion::joints::flags::iter(head_jam_rows())
            .filter_map(reachy_motion::joints::row)
            .map(|row| up_walk().arrivals()[row])
            .max()
            .expect("a hand is laid on at least one row");
        assert!(
            arrival + response_delay_cycles() <= hand.raise,
            "the jammed rows' unjammed arrival is cycle {arrival} and the fault lands on \
             {}: the reading the reopened run anchors against has to show a generator that has \
             stopped, or no release recovers the stow",
            hand.raise
        );
    }

    /// The floor is inclusive: a step exactly as long as the move plus what the
    /// watch asks carries a judged hold, and the cycle under it does not.
    #[test]
    fn a_step_on_the_floor_is_judged_and_one_under_it_complains() {
        let watch = StillnessConfig::default();
        let clocks = up_clocks();
        let floor_ns = clocks.cycles() * PERIOD_NS
            + i64::try_from(watch.shortest_judgeable_hold().as_nanos()).expect("seconds of it");
        assert!(unjudgeable_step("upright", floor_ns, &clocks, &watch).is_none());
        assert!(unjudgeable_step("upright", floor_ns - PERIOD_NS, &clocks, &watch).is_some());
    }

    /// The complaint names the step, the floor it missed and the move inside
    /// it, all three in milliseconds.
    ///
    /// The message is the whole product of a guard that never fires on a
    /// healthy tree, so a units slip in it would otherwise surface for the
    /// first time in front of the operator it is written for.
    #[test]
    fn the_complaint_names_the_step_the_floor_and_the_move_in_milliseconds() {
        let watch = StillnessConfig::default();
        let clocks = up_clocks();
        let move_ms = clocks.cycles() * PERIOD_NS / 1_000_000;
        let floor_ms = move_ms + 6_000;
        let says = unjudgeable_step("upright", 2 * PERIOD_NS, &clocks, &watch)
            .expect("40 ms judges nothing");
        assert!(says.contains("the upright step runs 40 ms"), "{says}");
        assert!(
            says.contains(&format!("if it runs {floor_ms} ms")),
            "{says}"
        );
        assert!(says.contains(&format!("{move_ms} ms of it")), "{says}");
        assert!(says.contains(clocks.longest_group()), "{says}");
    }

    /// The parameter check reads every key it claims to read, and fails a file
    /// that states any of them differently.
    ///
    /// Three files carry pinned keys: the profile's six, the gains' nine, and
    /// the detector's arming. A key the check silently skipped -- misspelled
    /// here, or dropped when the list was edited -- would pass a deployment
    /// nobody screened. So each is perturbed in turn and the failure is
    /// required to name it.
    #[test]
    fn the_parameter_check_reads_every_key_it_pins() {
        // The harness's own scratch directory, private to this test target and
        // emptied by it: a fixed name under the system temporary directory
        // collides between concurrent runs, and the cleanup below is skipped on
        // exactly the path -- a failing assertion -- where the leftovers would
        // be read by the next one.
        let dir = std::env::var_os("TEST_TMPDIR")
            .map_or_else(std::env::temp_dir, std::path::PathBuf::from)
            .join("check-params");
        let write = |dir: &std::path::Path, name: &str, body: &str| -> String {
            let path = dir.join(name);
            std::fs::write(&path, body).expect("the fixture directory is writable");
            path.to_string_lossy().into_owned()
        };
        let gains = reachy_motion::arm::DEFAULT_GAINS;
        let shipped = reachy_motion::plant::SHIPPED_PROFILES;
        // Every file the check reads, as the deployment states it. Built from
        // the constants rather than copied so this case is about which keys are
        // read, never about what the numbers are -- the files themselves are
        // what the live scenarios check.
        let files = |mutate: Option<(&str, &str, &str)>| -> Vec<String> {
            let _ = std::fs::remove_dir_all(&dir);
            std::fs::create_dir_all(&dir).expect("a fixture directory");
            let mut sources = [
                (
                    "mover_params.textproto",
                    vec![
                        ("lag_k", LAG_K.to_string()),
                        ("period_ns", PERIOD_NS.to_string()),
                        ("up_duration_ns", UP_DURATION_NS.to_string()),
                        ("stow_duration_ns", STOW_DURATION_NS.to_string()),
                        ("tracking_armed", TRACKING_ARMED.to_string()),
                    ],
                ),
                (
                    "servo_profile.textproto",
                    vec![
                        ("legs_profile_acceleration", shipped.legs.0.to_string()),
                        ("legs_profile_velocity", shipped.legs.1.to_string()),
                        ("body_yaw_profile_acceleration", shipped.yaw.0.to_string()),
                        ("body_yaw_profile_velocity", shipped.yaw.1.to_string()),
                        (
                            "antennas_profile_acceleration",
                            shipped.antennas.0.to_string(),
                        ),
                        ("antennas_profile_velocity", shipped.antennas.1.to_string()),
                    ],
                ),
                (
                    "servo_gains.textproto",
                    vec![
                        ("legs_p", gains.legs.p.to_string()),
                        ("legs_i", gains.legs.i.to_string()),
                        ("legs_d", gains.legs.d.to_string()),
                        ("body_yaw_p", gains.yaw.p.to_string()),
                        ("body_yaw_i", gains.yaw.i.to_string()),
                        ("body_yaw_d", gains.yaw.d.to_string()),
                        ("antennas_p", gains.antennas.p.to_string()),
                        ("antennas_i", gains.antennas.i.to_string()),
                        ("antennas_d", gains.antennas.d.to_string()),
                    ],
                ),
                (
                    "session_params.textproto",
                    vec![
                        ("aux_timeout_ns", AUX_TIMEOUT_NS.to_string()),
                        ("aux_retries", AUX_RETRIES.to_string()),
                        ("sample_stale_after", SAMPLE_STALE_AFTER.to_string()),
                        ("startup_grace_ns", STARTUP_GRACE_NS.to_string()),
                        ("stow_budget_ns", STOW_BUDGET_NS.to_string()),
                        (
                            "torque_off_confirm_budget_ns",
                            SESSION_CONFIRM_BUDGET_NS.to_string(),
                        ),
                        ("bus_watchdog", BUS_WATCHDOG.to_string()),
                        ("script_span_cap_ms", SCRIPT_SPAN_CAP_MS.to_string()),
                        ("rail_stale_after_ns", RAIL_STALE_AFTER_NS.to_string()),
                    ],
                ),
                (
                    "sim_params.textproto",
                    vec![
                        ("period_ns", PERIOD_NS.to_string()),
                        ("hold_timeout_ns", HOLD_TIMEOUT_NS.to_string()),
                        ("start_torqued", START_TORQUED.to_string()),
                        ("health_poll_period_ns", HEALTH_POLL_PERIOD_NS.to_string()),
                    ],
                ),
                (
                    "motord_params.textproto",
                    vec![("period_ns", PERIOD_NS.to_string())],
                ),
            ];
            if let Some((file, key, value)) = mutate {
                let target = sources
                    .iter_mut()
                    .find(|(name, _)| *name == file)
                    .expect("the fixture set carries the file");
                let cell = target
                    .1
                    .iter_mut()
                    .find(|(name, _)| *name == key)
                    .expect("the file carries the key");
                cell.1 = value.to_owned();
            }
            sources
                .iter()
                .map(|(name, keys)| {
                    let body: String = keys
                        .iter()
                        .map(|(key, value)| format!("{key}: {value}\n"))
                        .collect();
                    write(&dir, name, &body)
                })
                .collect()
        };

        let paths = files(None);
        let bound = ConfigPaths::of(&paths).expect("the fixture set carries every file");
        assert_eq!(
            check_params(&bound),
            Vec::<String>::new(),
            "the constants and a file written from them are one statement",
        );

        // One perturbation per pinned key, each a value the field could
        // plausibly drift to.
        let perturbations: [(&str, &str, &str); 16] = [
            ("mover_params.textproto", "tracking_armed", "false"),
            ("servo_profile.textproto", "legs_profile_acceleration", "21"),
            ("servo_profile.textproto", "legs_profile_velocity", "51"),
            (
                "servo_profile.textproto",
                "body_yaw_profile_acceleration",
                "21",
            ),
            ("servo_profile.textproto", "body_yaw_profile_velocity", "51"),
            (
                "servo_profile.textproto",
                "antennas_profile_acceleration",
                "21",
            ),
            ("servo_profile.textproto", "antennas_profile_velocity", "51"),
            ("servo_gains.textproto", "legs_p", "801"),
            ("servo_gains.textproto", "legs_i", "101"),
            ("servo_gains.textproto", "legs_d", "301"),
            ("servo_gains.textproto", "body_yaw_p", "201"),
            ("servo_gains.textproto", "body_yaw_i", "1"),
            ("servo_gains.textproto", "body_yaw_d", "1"),
            ("servo_gains.textproto", "antennas_p", "501"),
            ("servo_gains.textproto", "antennas_i", "1"),
            ("servo_gains.textproto", "antennas_d", "101"),
        ];
        for (file, key, value) in perturbations {
            let paths = files(Some((file, key, value)));
            let bound = ConfigPaths::of(&paths).expect("the fixture set carries every file");
            let failures = check_params(&bound);
            assert!(
                failures.iter().any(|line| line.contains(key)),
                "a {file} stating {key}: {value} is a file the check has to name: {failures:?}",
            );
        }
        let _ = std::fs::remove_dir_all(&dir);
    }
}
