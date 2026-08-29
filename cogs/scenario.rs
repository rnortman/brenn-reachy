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

/// How far a crank moves in one cycle, radians.
pub const SLEW_LEGS_RAD: f64 = 0.15;

/// How far the body yaw moves in one cycle, radians. Its own number rather than
/// the cranks': the plant configures the three groups separately, and a scenario
/// that could not say they differ could not run one where they do.
pub const SLEW_BODY_YAW_RAD: f64 = 0.15;

/// Whether the modelled machine starts energised. Every scenario starts it cold:
/// the session's own engagement is what energises it, over the bus, which is the
/// arming path a real machine has.
pub const START_TORQUED: bool = false;

/// How far an antenna moves in one cycle, radians.
pub const SLEW_ANTENNAS_RAD: f64 = 0.65;

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
/// Mirrors the deployed `SessionParams.profile_acceleration`; a scenario
/// asserting this pair is asserting about the file the process read. What makes
/// the claim reach the wire is
/// [`check::commissioned_profile`](crate::check::commissioned_profile), which
/// finds the two writes in the run's own datagrams.
pub const PROFILE_ACCELERATION: i64 = 20;

/// The servo-side profile velocity the sweep writes, register units.
pub const PROFILE_VELOCITY: i64 = 50;

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
    let sidecar: serde_json::Value =
        serde_json::from_str(CLIP_LIBRARY_NAMES).expect("the committed sidecar is the emitter's");
    let motions = sidecar["motions"]
        .as_array()
        .expect("the committed sidecar names its motions");
    let found = motions
        .iter()
        .find(|motion| motion["name"].as_str() == Some(name));
    let Some(found) = found else {
        let carried: Vec<&str> = motions
            .iter()
            .filter_map(|motion| motion["name"].as_str())
            .collect();
        panic!("the committed clip library carries no motion named {name}, only {carried:?}");
    };
    u16::try_from(
        found["motion_id"]
            .as_u64()
            .expect("a motion's number is a number"),
    )
    .expect("a motion number the vocabulary can hold")
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

/// How many transactions taking hold of the machine spends.
///
/// Three sweeps of the bus for the watch -- the positions, the supply and the
/// error bits, which are what the two torque-on gates judge -- and three for the
/// engagement, which writes every goal, enables every servo and reads all nine
/// back. Derived from the machine's own row count, so a bus that grew a servo
/// widens this with it.
#[must_use]
pub fn engage_transactions() -> i64 {
    6 * reachy_motion::joints::ROW_COUNT as i64
}

/// How many cycles a run must allow for taking hold of the machine.
///
/// An allowance and not an expectation, on [`commission_allowance_cycles`]'s
/// arithmetic and for its reasons. A scenario that wants to know the arming
/// *finished* asserts the phase it ended in, and every instant derived from this
/// is an outer bound on when the machine can be under command.
#[must_use]
pub fn engage_allowance_cycles() -> i64 {
    3 * engage_transactions()
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
/// # Errors
///
/// One line per number the file states differently, or per number it does not
/// state at all, or the reason the file could not be read.
pub fn check_params(
    mover_textproto: &str,
    session_textproto: &str,
    sim_textproto: &str,
) -> Vec<String> {
    let mut failures = Vec::new();
    expect(
        mover_textproto,
        &[
            ("lag_k", Value::Int(LAG_K)),
            ("period_ns", Value::Int(PERIOD_NS)),
            ("up_duration_ns", Value::Int(UP_DURATION_NS)),
            ("stow_duration_ns", Value::Int(STOW_DURATION_NS)),
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
            ("profile_acceleration", Value::Int(PROFILE_ACCELERATION)),
            ("profile_velocity", Value::Int(PROFILE_VELOCITY)),
            ("bus_watchdog", Value::Int(BUS_WATCHDOG)),
            ("script_span_cap_ms", Value::Int(SCRIPT_SPAN_CAP_MS)),
        ],
        &mut failures,
    );
    expect(
        sim_textproto,
        &[
            ("period_ns", Value::Int(PERIOD_NS)),
            ("hold_timeout_ns", Value::Int(HOLD_TIMEOUT_NS)),
            ("start_torqued", Value::Bool(START_TORQUED)),
            ("slew_legs_rad", Value::Float(SLEW_LEGS_RAD)),
            ("slew_body_yaw_rad", Value::Float(SLEW_BODY_YAW_RAD)),
            ("slew_antennas_rad", Value::Float(SLEW_ANTENNAS_RAD)),
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
    //! What the shipped wake gesture's lead has to cover.
    //!
    //! The lead is the room the session gets between a script arriving and its
    //! first step opening, and what has to fit in it is the start-up survey:
    //! every transaction that arms the machine, at the cycles a transaction
    //! costs. Both halves of that product are derived from tables this tree
    //! grows -- the provisioning table's cells and the gains-and-profile write
    //! set -- so the file's number is the one part of the derivation that cannot
    //! grow by itself. This is what makes it grow: a register added to the sweep
    //! widens the survey and fails here, rather than costing the gesture its
    //! window on a machine whose survey is still running. What an overrun does
    //! and does not cost is argued once, beside the number: the `lead_ms`
    //! comment in `wake_params.textproto`.

    use super::{PERIOD_NS, commission_allowance_cycles, commission_transactions};

    /// The shipped gesture, embedded so the case needs no runfiles.
    const WAKE_PARAMS: &str = include_str!("wake_params.textproto");

    /// What the file says the lead is, in milliseconds.
    fn lead_ms() -> i64 {
        WAKE_PARAMS
            .lines()
            .map(str::trim)
            .filter(|line| !line.starts_with('#'))
            .filter_map(|line| line.split_once(':'))
            .find_map(|(key, value)| {
                (key.trim() == "lead_ms").then(|| {
                    value
                        .trim()
                        .parse::<i64>()
                        .expect("the shipped lead is a count of milliseconds")
                })
            })
            .expect("the shipped wake gesture states a lead")
    }

    /// The margin the lead must hold over the survey's allowance, as a fraction.
    /// Bare sufficiency — a lead that merely equals the allowance — leaves no
    /// room for bus-retry noise; asserting the margin catches an edit that
    /// erodes it before it shows up as a late first step.
    const HEADROOM_NUMERATOR: i64 = 5;
    const HEADROOM_DENOMINATOR: i64 = 4;

    #[test]
    fn the_wake_gestures_lead_covers_the_survey_that_has_to_finish_inside_it() {
        let lead_ns = lead_ms() * 1_000_000;
        let allowance_ns = commission_allowance_cycles() * PERIOD_NS;
        let required_ns = allowance_ns * HEADROOM_NUMERATOR / HEADROOM_DENOMINATOR;
        assert!(
            lead_ns >= required_ns,
            "the wake gesture leads by {} ms and taking hold of the machine allows {} \
             transactions at three cycles each, which is {} ms: the lead has to clear that \
             by a quarter of it again -- {} ms -- so that what a noisy bus costs beyond the \
             per-transaction allowance still fits, and an edit that eats the margin is read \
             here rather than as a late, truncated or skipped first step",
            lead_ns / 1_000_000,
            commission_transactions(),
            allowance_ns / 1_000_000,
            required_ns / 1_000_000
        );
    }
}
