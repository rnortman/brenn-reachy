//! The bench night, replayed against the guards it sized.
//!
//! Every bound in this crate's shipped configuration is a measurement, and
//! these are the measurements. Two questions, asked of the hardware recordings
//! beside the crate rather than of arithmetic: does a shipped guard raise
//! anything on a run that went well, and does it catch the one run that did
//! not. A change to a step bound, a tracking threshold or the antennas'
//! separation that would false-trip the validated gesture — or miss the
//! collision — fails here instead of on the machine.
//!
//! The values are the ones the cog path actually runs:
//! [`MotionConfig::default`], [`TrackingFaultConfig::default`] and the phase
//! constants, never a copy of them in some host's configuration file. A
//! re-derived figure landing outside a stated tolerance is escalated for human
//! review as a suspected computation difference; the pin is never moved and no
//! tolerance widened to make this suite green.

mod replay_trace;

use core::time::Duration;

use brenn_reachy__motion__joints_clk_rs::JointFlags;
use reachy_motion::joints::{ROW_COUNT, ROWS, flags, group_of, row};
use reachy_motion::plant::{
    ClassProfile, GroupPlants, GroupProfiles, MAX_GAP_PERIODS, RESPONSE_DEAD_SAMPLES,
    SHIPPED_PERIOD_NS, SHIPPED_PROFILES,
};
use reachy_motion::tick::{
    RECORDED_WORST_ANTENNA_LAG_RAD, RECORDED_WORST_ANTENNA_RESIDUAL_RAD,
    RECORDED_WORST_HEAD_LAG_RAD, RECORDED_WORST_HEAD_RESIDUAL_RAD, sustained_residual, tracking,
};
use reachy_motion::{
    ANTENNA_PHASE_SEPARATION_RAD, JointGroup, JointRef, JointTargets, JointVector, MotionCommand,
    MotionConfig, MotionSnapWire, MoveDurations, StillnessConfig, WarpKind, dry_pass_peaks,
    floor_move_clock, judge, stow_pose_targets, stow_targets,
};

use reachy_motion::trace::{Run, Sample, Trace};
use replay_trace::{ARRIVED_TOLERANCE_RAD, RunMetrics as _, fixture};

/// The rate the recordings were driven at and the shipped floors are derived
/// at.
const TICK_HZ: f64 = reachy_motion::FLOOR_TICK_HZ;

/// The head clock every recorded gesture was commanded over — the shipped
/// `up_duration`.
const UP: Duration = Duration::from_millis(800);

/// Degrees as radians, for a figure measured in the units the machine is
/// measured in.
fn deg(degrees: f64) -> f64 {
    degrees.to_radians()
}

/// The pose a recorded run started from, as the command path expresses one.
///
/// The head is the stow posture, which is where the recordings begin to within
/// a degree — asserted, because the fixtures only speak for the shipped
/// posture while that holds. A degree and not a count because the servos were
/// holding stow on the gains of that night, and holding a weight up on a
/// proportional term alone parks a loaded crank a little short of where it was
/// sent. The antennas come off the recording: an antenna direction has a
/// representative per turn, and the sweep the resolver picks depends on which
/// one the machine stood at.
fn started_at(cfg: &MotionConfig, run: &Run) -> JointTargets {
    let present = run.samples[0].present.expect("the first period read");
    let held = stow_targets(&cfg.geom).expect("the geometry reaches stow");
    for (leg, angle) in present.legs.iter().enumerate() {
        assert!(
            (angle - held.legs[leg]).abs() < deg(1.0),
            "leg {leg} began at {angle}, not the stow pose's {}",
            held.legs[leg]
        );
    }
    JointTargets {
        antennas: present.antennas,
        ..stow_pose_targets()
    }
}

/// The gesture the recordings are of: stow to neutral, on `durations`.
fn gesture(durations: MoveDurations) -> MotionCommand {
    MotionCommand::MoveTo {
        target: JointTargets::default(),
        durations,
        warp: WarpKind::MinJerk,
    }
}

/// The profile the recordings in `fixtures/traces` were made under, in register
/// units.
///
/// The bench nights ran a faster pair than the deployment ships, so every
/// fixture here is judged against the plant it was actually recorded on. A log
/// recorded under one pair and replayed under another is a different machine.
///
/// One pair for all three classes: the bench nights wrote one pair into all
/// nine servos, which is what the recordings were made on.
const BENCH_PROFILE: GroupProfiles = GroupProfiles {
    legs: BENCH_PAIR,
    yaw: BENCH_PAIR,
    antennas: BENCH_PAIR,
};

/// The one pair those nights wrote into all nine servos, at the following lag
/// those recordings are judged with — none, because none of these loops is a
/// loop a lag was read on. The antennas ran `500 / 0 / 100` and the legs
/// changed gains partway through `trace-newgains`, with no triple on record for
/// the other three nights; a lag belongs to one gains triple, and a class whose
/// triple has no reading is judged at the generator alone. Not that the pair is
/// too slow to show one: `400 / 600` is the fastest pair in the fixture set, and
/// 1.2 periods of it would be a third of a radian of prediction.
const BENCH_PAIR: ClassProfile = at(
    Pair {
        acceleration: 400,
        velocity: 600,
    },
    0,
);

/// The plant a recording is judged against: its own profile, on the grid that
/// recording was actually driven at.
///
/// Both halves come off the run rather than off the deployment. The bench
/// nights ran a faster profile than the machine ships and slower loops than the
/// grid it ships — 32 ms and 24 ms a period — and the generator this models
/// moves per period, so a
/// model handed the shipped period covers a fraction of what the servo covered
/// in the same sample and reads the difference as residual.
fn bench_plant(run: &Run) -> GroupPlants {
    GroupPlants::from_profiles(&BENCH_PROFILE, run.period_ns())
        .expect("the bench profile pairs are three models")
}

/// One period of a recording on which some joint's window ran out.
///
/// The residuals of that period travel with it: what the comparison decided is
/// only half of what a guard needs, since a window run out at the threshold's
/// own edge and one run out a radian past it are the same boolean and very
/// different evidence.
struct Trip {
    /// The grid slot, counted from the run's first period.
    at: u64,
    /// The joints whose window ran out on it.
    exhausted: JointFlags,
    /// How far every row stood from its own prediction, radians.
    residuals: [f64; ROW_COUNT],
}

/// One period the comparison judged, and what it measured.
///
/// No grid slot: the judged periods of a run are not necessarily adjacent
/// slots — a period whose grouped read fell short is stepped and never judged —
/// and the detector's own window is a count of judged periods rather than a
/// stretch of the grid, so nothing here needs to know which slot a reading came
/// from.
struct Judged {
    /// How far every row stood from its own prediction, radians.
    residuals: [f64; ROW_COUNT],
}

/// What one recorded run reads as, judged.
struct Replay {
    /// The periods on which some joint's window ran out, in grid order.
    trips: Vec<Trip>,
    /// How far every row stood from its own prediction on each period that
    /// carried a reading, in grid order.
    ///
    /// Every judged period and not only the ones that decided something: the
    /// threshold is sized on the worst of these and on the worst a run holds
    /// for a whole window, and neither figure is visible in a verdict.
    judged: Vec<Judged>,
}

/// The shipped tracking comparison driven over a recorded run, period by
/// period, answering the joints whose window ran out and when.
///
/// The prediction is stepped exactly as the live tick steps it: seeded from the
/// first reading, then one step-then-push per period against the setpoint the
/// driver was holding `RESPONSE_DEAD_SAMPLES` periods earlier. A period whose
/// grouped read fell short is stepped but not judged — a stale measurement
/// would freeze the run where it stands, while the servo went on moving, and
/// the servo did not stop moving because the read failed. A grid slot the
/// recording holds no period for is stepped on the setpoint already held; a gap
/// of more than `MAX_GAP_PERIODS` of them re-seeds, because past that the
/// prediction is not something arithmetic knows. A released joint is handed
/// over masked: it holds no setpoint to be judged against.
///
/// The walk is the tick's `advance_prediction` restated, and it has to stay so:
/// a fixture judged by any other order or any other gap rule is not the run the
/// machine was judged by. `plant::RESPONSE_DEAD_SAMPLES`' own
/// TODO(plant-chase-sequencer) is the one statement of this walk that would
/// make the restatement unnecessary.
fn replay(cfg: &MotionConfig, plant: &GroupPlants, run: &Run) -> Replay {
    let mut state = MotionSnapWire::new();
    let state = state.clear_valid();
    let mut out = Replay {
        trips: Vec::new(),
        judged: Vec::new(),
    };
    let mut held: Vec<JointVector> = Vec::new();
    let mut previous: Option<u64> = None;
    for sample in &run.samples {
        // A released joint is commanded nothing, so the ring carries its own
        // angle rather than a setpoint it never had.
        let mut setpoint = sample.present.unwrap_or_default();
        for joint in ROWS {
            if let Some(angle) = sample.goal_of(joint) {
                setpoint.set(joint, angle);
            }
        }
        // How many periods this one covers: its own, plus every grid slot the
        // recording holds nothing for. Floored at one.
        let periods = previous.map_or(1, |before| sample.tick.saturating_sub(before).max(1));
        previous = Some(sample.tick);
        if periods > MAX_GAP_PERIODS as u64 || held.len() < RESPONSE_DEAD_SAMPLES {
            tracking::reseed(&mut state.tracking);
            if periods > MAX_GAP_PERIODS as u64 {
                // The ring goes with the prediction over a gap that long, and
                // only over one: a prediction seeded from this period's reading
                // has no business being stepped toward a setpoint from before
                // the gap. A ring that is merely filling is left to fill.
                held.clear();
            }
        } else {
            for period in 0..periods {
                let target = held.remove(0);
                tracking::predict(plant, &target, &mut state.tracking);
                if period + 1 < periods {
                    // Nothing was written in a slot the loop missed, so the
                    // newest setpoint is pushed again: the servos chased what
                    // they were already holding.
                    let carried = *held.last().unwrap_or(&target);
                    held.push(carried);
                }
            }
        }
        held.push(setpoint);
        while held.len() > RESPONSE_DEAD_SAMPLES {
            held.remove(0);
        }
        let Some(present) = sample.present else {
            continue;
        };
        tracking::seed(&mut state.tracking, &present);
        let look = tracking::look(
            &cfg.tracking,
            sample.released(),
            &present,
            &mut state.tracking,
        );
        out.judged.push(Judged {
            residuals: look.residuals,
        });
        if !flags::is_empty(look.exhausted) {
            out.trips.push(Trip {
                at: sample.tick,
                exhausted: look.exhausted,
                residuals: look.residuals,
            });
        }
    }
    out
}

fn is_head(joint: JointRef) -> bool {
    group_of(joint) != Some(JointGroup::Antennas)
}

fn is_antenna(joint: JointRef) -> bool {
    !is_head(joint)
}

fn worst_residual(judged: &[Judged], admit: fn(JointRef) -> bool) -> f64 {
    ROWS.into_iter()
        .filter(|joint| admit(*joint))
        .filter_map(row)
        .flat_map(|row| judged.iter().map(move |judged| judged.residuals[row]))
        .fold(0.0_f64, f64::max)
}

/// One class's two profile registers, without the loop that answered them.
///
/// The rows below pair a recording's pair with the lag of the loop it was made
/// on, and the two are read from different places: a pair is what the
/// commissioning sweep wrote into the servo, a lag is what a scan read off that
/// servo's gains.
struct Pair {
    acceleration: u32,
    velocity: u32,
}

/// A class's row: the pair it ran, at the lag of the loop it ran on.
const fn at(pair: Pair, following_lag_us: u32) -> ClassProfile {
    ClassProfile {
        acceleration: pair.acceleration,
        velocity: pair.velocity,
        following_lag_us,
    }
}

/// The pair the deployment wrote into all nine servos before any class was
/// commissioned, register units, and the pair the body yaw still runs.
const TOUR_2050_PAIR: Pair = Pair {
    acceleration: 20,
    velocity: 50,
};

/// The legs' own measured capability, register units: what the cranks ship at
/// and what every recording made after their commissioning ran them at.
const LEG_PAIR: Pair = Pair {
    acceleration: 287,
    velocity: 326,
};

/// The antennas' own measured capability, register units: what the class ships
/// at, read off the capability instrument and confirmed by the armed runs of
/// 2026-09-09.
const ANTENNA_PAIR: Pair = Pair {
    acceleration: 522,
    velocity: 640,
};

/// The yaw's register as the capability sweep of 2026-09-09 left it, which is
/// what the fast-pair tour of that morning was flown at.
///
/// Two units under the round figure every other row carries, and its own
/// constant rather than a digit inside one: a recording is replayed at the
/// register it ran under.
const SWEPT_YAW_PAIR: Pair = Pair {
    acceleration: 20,
    velocity: 48,
};

/// The recordings cut from the clip-library tours, the wake gesture, and the
/// runs that confirmed the antennas' commissioned pair, which are the runs the
/// shipped screen is sized on, each with the profile it was recorded under.
///
/// Four profiles across the ten: five recordings from the nights that wrote
/// `20 / 50` into all nine servos, two from the tour that confirmed the legs'
/// commissioned pair, which ran the cranks at what the tree ships them at and
/// the other two classes at `20 / 50`, one from the tour that ran the antennas
/// at their own measured capability an hour earlier, and two from the armed
/// probe runs that confirmed that pair. All four are literals, because a
/// recording carries the profile it was made on and cannot follow a constant
/// anywhere; that the last of the four is also what the tree ships today is
/// asserted on its own, by
/// [`the_antenna_confirmation_is_still_the_shipping_profile`].
const TOUR_FIXTURES: [(&str, GroupProfiles); 10] = [
    ("trace-tour-toc-toc-toc", TOUR_2050_PROFILE),
    ("trace-tour-side-peekaboo", TOUR_2050_PROFILE),
    ("trace-tour-proud1", TOUR_2050_PROFILE),
    ("trace-tour-no-sad1", TOUR_2050_PROFILE),
    ("trace-wake-20260906", TOUR_2050_PROFILE),
    ("trace-tour-grid-snap", CONFIRM_TOUR_PROFILE),
    ("trace-tour-stumble-and-recover", CONFIRM_TOUR_PROFILE),
    ("trace-tour-sharp-side-tilt-fast", FAST_TOUR_PROFILE),
    ("trace-probe-antenna-sweep", CONFIRM_ANTENNA_PROFILE),
    ("trace-probe-antenna-step-a", CONFIRM_ANTENNA_PROFILE),
];

/// The profile the antennas' confirmation runs of 2026-09-09 were flown at, in
/// register units, with the following lag of each class's loop on them.
///
/// Every class at what the tree ships: the legs at their commissioned pair, the
/// body yaw at the pair its own capability reading agrees with, and the
/// antennas at the capability the instrument read off them, which these runs are
/// what commissioned. Stated as a literal for the reason the three profiles
/// above are — a recording is judged at the plant it was made on — and that it
/// equals [`SHIPPED_PROFILES`] today is the separate statement
/// [`the_antenna_confirmation_is_still_the_shipping_profile`] makes.
const CONFIRM_ANTENNA_PROFILE: GroupProfiles = GroupProfiles {
    // At gains `800 / 100 / 300`, the triple the legs' lag was read on and the
    // one the class runs.
    legs: at(LEG_PAIR, LEGS_LAG_US),
    // At `200 / 0 / 0`, read as no lag.
    yaw: at(TOUR_2050_PAIR, 0),
    // At `200 / 0 / 0`, the triple the antennas' lag was read on: the integral
    // rung was walked on these same probes and refused, so the gains the class
    // ships are the gains the lag was read on.
    antennas: at(ANTENNA_PAIR, ANTENNAS_LAG_US),
};

/// The profile the confirmation tour of 2026-09-09 was flown at, in register
/// units, with the following lag of each class's loop on that tour.
///
/// The legs at the pair the commissioning read off them, the other two classes
/// at the pair the earlier tours wrote everywhere. Stated as a literal for the
/// same reason [`TOUR_2050_PROFILE`] is: it is a fact about two recordings, and
/// the fixtures are judged at the profile they were recorded at. It is no
/// longer the shipping profile — the antennas were commissioned at their own
/// capability after this tour was flown — and these two rows did not move with
/// them, which is what a recording being a recording means. What they still
/// pin is the legs, whose pair and loop have not moved since.
const CONFIRM_TOUR_PROFILE: GroupProfiles = GroupProfiles {
    // At gains `800 / 100 / 300`, which is the triple the legs' lag was read
    // on and the one the class still runs.
    legs: at(LEG_PAIR, LEGS_LAG_US),
    // At `200 / 0 / 0`, read as no lag: the class is gain-bound on every
    // recording and no run holds its motor's speed to read a loop against.
    yaw: at(TOUR_2050_PAIR, 0),
    // At `200 / 0 / 0`, which is the triple the antennas' lag was read on and
    // the one the class ships.
    antennas: at(TOUR_2050_PAIR, ANTENNAS_LAG_US),
};

/// The profile the fast-pair tour of 2026-09-09 was flown at, in register
/// units, with the following lag of each class's loop on that tour.
///
/// The tour that ran the antennas at the capability instrument's own reading —
/// `522 / 640`, which the class is commissioned at — with the legs at their
/// commissioned pair and the yaw where the capability sweep of that morning had
/// put it. It is the only library reading of the antennas at the pair they
/// ship, which is what the class's own pin is read off, and it is judged at the
/// plant it was made on like every other row. The row is not the shipping
/// profile all the same: the yaw's register read 48 that morning.
///
/// All three loops are the confirmation tour's: the two tours were flown an
/// hour apart on one set of gains, so the two lag readings reach this row
/// unchanged. What differs is the antennas' pair, which is the whole reason
/// this row exists.
const FAST_TOUR_PROFILE: GroupProfiles = GroupProfiles {
    // At gains `800 / 100 / 300`, the triple the legs' lag was read on.
    legs: at(LEG_PAIR, LEGS_LAG_US),
    // At `200 / 0 / 0`, read as no lag, at the pair that morning's capability
    // sweep had left in the register.
    yaw: at(SWEPT_YAW_PAIR, 0),
    // At `200 / 0 / 0`, the triple the antennas' lag was read on, at the pair
    // the capability instrument read off the class rather than the one it is
    // commissioned with.
    antennas: at(ANTENNA_PAIR, ANTENNAS_LAG_US),
};

/// The profile every recording of the 2026-09-06 tour and the wake gesture was
/// made under, in register units, with the following lag of each class's loop
/// on those nights.
///
/// The deployment commissioned one pair for all nine servos on those nights,
/// and it does not ship that pair on the legs: the cranks run their own
/// measured capability. So these recordings need the pair they were made
/// on, stated here the way `BENCH_PROFILE` states the bench nights' — a leg
/// recorded under `20 / 50` and replayed under the commissioned `287 / 326`
/// would be a joint judged against a generator twelve times its own, and the
/// difference would read as residual the machine never showed.
///
/// The pair is one pair and the three lags are not, which is why this is three
/// rows and not one: a lag is a reading of a loop, and these nights ran
/// the antennas at `500 / 0 / 100` while the two 2026-09-09 fixtures ran them
/// at the `200 / 0 / 0` the class ships. The tree has no reading for the
/// stiffer loop, so these recordings replay at the trapezoid alone — the same
/// answer the offline analyzer gives a live run whose configuration states no
/// lag.
const TOUR_2050_PROFILE: GroupProfiles = GroupProfiles {
    // At gains `800 / 100 / 300`, the triple the legs' lag was read on: the
    // legs' gains did not move between these nights and the confirmation tour.
    legs: at(TOUR_2050_PAIR, LEGS_LAG_US),
    // At `200 / 0 / 0`, read as no lag.
    yaw: at(TOUR_2050_PAIR, 0),
    // At `500 / 0 / 100`, a proportional term two and a half times the one the
    // antennas' lag was read on, and a loop no scan has read. Nothing here
    // stands for it, so these five recordings are judged at the generator
    // alone.
    antennas: at(TOUR_2050_PAIR, 0),
};

/// The six platform servos' following lag on `800 / 100 / 300`, the gains every
/// kept tour was flown at, microseconds.
///
/// A literal and not [`SHIPPED_PROFILES`]'s figure, for the reason the pairs
/// beside it are literals: these are readings of the loops these recordings
/// were made on, and a fixture that followed a shipped constant would be
/// re-judged by a commissioning it has no part in. That the tree ships this
/// same figure today is a separate statement, asserted by
/// [`the_antenna_confirmation_is_still_the_shipping_profile`].
const LEGS_LAG_US: u32 = 24_000;

/// The antennas' following lag on the `200 / 0 / 0` gains the class ships,
/// microseconds.
///
/// A literal for the same reason, and it reaches only the five 2026-09-09
/// fixtures: the earlier five ran the stiffer `500 / 0 / 100` loop, which no
/// scan has read.
const ANTENNAS_LAG_US: u32 = 24_000;

/// The plant a recorded profile is judged against, on the shipped grid every
/// one of the kept recordings was driven at.
fn tour_plant_of(profiles: &GroupProfiles) -> GroupPlants {
    GroupPlants::from_profiles(profiles, SHIPPED_PERIOD_NS)
        .expect("a recorded tour profile is three models")
}

/// The plant a kept tour recording is judged against, found by the fixture's
/// name: the profile of its own row in [`TOUR_FIXTURES`].
///
/// For a caller holding a name and no row — a fixture the suite reads without
/// one fails here rather than being judged against whatever plant its caller
/// had to hand. A caller walking the table itself holds the profile already and
/// builds its plant with [`tour_plant_of`].
fn tour_plant(name: &str) -> GroupPlants {
    let profiles = TOUR_FIXTURES
        .into_iter()
        .find(|(fixture, _)| *fixture == name)
        .map(|(_, profiles)| profiles)
        .unwrap_or_else(|| panic!("{name} has no recorded profile in TOUR_FIXTURES"));
    tour_plant_of(&profiles)
}

/// The two antenna confirmation fixtures were cut at the profile the tree
/// ships, and the pins read off them are what the shipped screen is sized on.
///
/// The coupling the fixture rows used to carry by aliasing the shipped constant,
/// stated where it can fail in its own words. The rows themselves cannot follow
/// a commissioning — a recording is a recording — so when a class's pair next
/// moves, what has to happen is a fresh run at the new pair and a fresh cut,
/// not an edit to these two rows.
#[test]
fn the_antenna_confirmation_is_still_the_shipping_profile() {
    assert_eq!(
        CONFIRM_ANTENNA_PROFILE, SHIPPED_PROFILES,
        "the antennas' confirmation runs are no longer at the shipping profile: the pins read \
         off `trace-probe-antenna-sweep` and `trace-probe-antenna-step-a` are readings at a \
         profile the tree has stopped shipping. Run the confirmation at the new pair, cut its \
         worst windows, and move the pins onto the new fixtures -- do not rebind these two rows, \
         which are replayable only against the plant they were recorded on"
    );
}

/// Guard 1. Neither run that went well raises anything in the shipped tracking
/// comparison, and the lags they ran at are the headroom record.
///
/// The validated gesture and the fastest sweep on record, measured on the
/// machine, both with every joint following its goal at a distance the whole
/// way through — a quarter of a radian on a loaded leg, and better than a
/// radian on the antenna crossing 187° in four tenths of a second. A threshold
/// sized under those, or a progress minimum over what the machine closes in a
/// window, shows up here as a fault on a gesture the machine is known to make
/// well.
#[test]
fn the_runs_that_went_well_raise_nothing() {
    let cfg = MotionConfig::default();
    for name in ["trace-verify2", "trace-fast4"] {
        let trace = fixture(name);
        let trips = replay(&cfg, &bench_plant(trace.run(0)), trace.run(0)).trips;
        assert!(
            trips.is_empty(),
            "{name}: {:?}",
            trips
                .iter()
                .map(|trip| (trip.at, flags::Names(trip.exhausted).to_string()))
                .collect::<Vec<_>>()
        );
    }

    // The two lags the default threshold is sized over, pinned where they were
    // measured: the loaded leg on the validated gesture, and the antenna on the
    // 855°/s sweep. Both sit under the threshold's own headroom claim. The
    // figures are the library's own constants, so the recordings and the number
    // a live run is reported against are one statement.
    let head_lag = ROWS
        .into_iter()
        .filter(|joint| group_of(*joint) != Some(JointGroup::Antennas))
        .map(|joint| fixture("trace-verify2").run(0).joint(joint).worst_lag)
        .fold(0.0_f64, f64::max);
    assert!(
        (head_lag - RECORDED_WORST_HEAD_LAG_RAD).abs() < 5e-3,
        "the validated gesture's worst head lag is {head_lag:.4} rad"
    );
    let fast4 = fixture("trace-fast4");
    let antenna_lag = [JointRef::AntennaRight, JointRef::AntennaLeft]
        .into_iter()
        .map(|joint| fast4.run(0).joint(joint).worst_lag)
        .fold(0.0_f64, f64::max);
    assert!(
        (antenna_lag - RECORDED_WORST_ANTENNA_LAG_RAD).abs() < 5e-3,
        "the fast sweep's worst antenna lag is {antenna_lag:.4} rad"
    );
    assert!(
        head_lag < cfg.tracking.threshold_rad,
        "the healthy head lag {head_lag:.4} rad now reaches the {:.4} rad threshold",
        cfg.tracking.threshold_rad
    );
}

/// The worst residual any head joint ran at on the bench recordings, radians.
///
/// Local to this suite rather than a library constant, because no live run is
/// reported against the bench profile: these recordings were made at `400 / 600`
/// against the `20 / 50` the deployment commissions, and on a 32 ms grid against
/// the shipped 20 ms, so the figure screens nothing a report prints. What it is
/// for is the model itself — the same
/// arithmetic over the same machine at a second setting, which is the only
/// in-tree check that the model is the servo's generator rather than a fit to
/// one profile.
const BENCH_WORST_HEAD_RESIDUAL_RAD: f64 = 0.0315;

/// The worst residual an antenna ran at on the fastest bench sweep, radians.
///
/// Read the same way as [`BENCH_WORST_HEAD_RESIDUAL_RAD`], over the sweep that
/// crossed 187° in 0.40 s at an 855°/s peak: the joint was a radian and a third
/// behind its goal and this far from its own trajectory.
const BENCH_WORST_ANTENNA_RESIDUAL_RAD: f64 = 0.3321;

/// Guard 1. The clip library played on the unit raises nothing, and its worst
/// residuals are the figures the shipped threshold is sized over.
///
/// Every row of [`TOUR_FIXTURES`], each replayed at the profile it was recorded
/// at — the whole library's worst residual on record, its worst leg and antenna
/// reversal excursions, its longest travel and the shipped gesture, over the
/// 2026-09-06 tour, the wake gesture beside it, the 2026-09-09 confirmation
/// tour and the fast-pair tour of the same morning. A healthy machine on the
/// content it ships with, so
/// a raise here is the screen sized wrong; the pins are the library's own
/// constants, so the figure a report prints beside a live run and the figure
/// the recordings hold are one statement.
#[test]
fn the_recorded_library_raises_nothing_and_pins_the_residuals_the_screen_is_sized_on() {
    let cfg = MotionConfig::default();
    let window = cfg.tracking.ticks as usize;
    let mut judged = Vec::new();
    let mut sustained = 0.0_f64;
    for (name, profiles) in TOUR_FIXTURES {
        let trace = fixture(name);
        assert_eq!(trace.runs(), 1, "{name} holds more than the window cut");
        // A fixture cut from a run at another period would be a different
        // machine, so the grid is asserted rather than assumed.
        assert_eq!(
            trace.run(0).period_ns(),
            SHIPPED_PERIOD_NS,
            "{name} was recorded on another grid"
        );
        let outcome = replay(&cfg, &tour_plant_of(&profiles), trace.run(0));
        assert!(
            outcome.trips.is_empty(),
            "{name}: {:?}",
            outcome
                .trips
                .iter()
                .map(|trip| (trip.at, flags::Names(trip.exhausted).to_string()))
                .collect::<Vec<_>>()
        );
        // The sustained figure is taken per recording, because a window is a
        // stretch of one run's own judged periods; the worst period is a
        // maximum and takes the whole library at once.
        let held: Vec<[f64; ROW_COUNT]> = outcome
            .judged
            .iter()
            .map(|judged| judged.residuals)
            .collect();
        sustained = sustained.max(sustained_residual(&held, window));
        judged.extend(outcome.judged);
    }

    let head = worst_residual(&judged, is_head);
    let antennas = worst_residual(&judged, is_antenna);
    assert!(
        (head - RECORDED_WORST_HEAD_RESIDUAL_RAD).abs() < 5e-3,
        "the recorded library's worst head residual is {head:.4} rad"
    );
    // The antennas' maximum over these recordings is not their pin, and two
    // constants stand between them. The five 2026-09-06 recordings ran the
    // class on `500 / 0 / 100`, a loop no scan has read, so they are replayed
    // at the generator alone, and `proud1` judged by that emptier model is the
    // largest antenna sample here. The two confirmation-tour recordings ran the
    // shipping loop at the `20 / 50` pair the class has since left. The pin is
    // the class at its shipping configuration, by the rule that a class's
    // recorded figures are one configuration's reading; what these recordings
    // hold is the largest of the three, which is the figure the sizing rule
    // below screens.
    let recorded_antennas = RECORDED_WORST_ANTENNA_RESIDUAL_RAD
        .max(CLIP_WORST_ANTENNA_RESIDUAL_AT_STIFF_GAINS_RAD)
        .max(CLIP_WORST_ANTENNA_RESIDUAL_AT_THE_SLOW_PAIR_RAD);
    assert!(
        (antennas - recorded_antennas).abs() < 5e-3,
        "the recorded library's worst antenna residual is {antennas:.4} rad"
    );

    // And the margin, which is what the screen is: no excursion on record was
    // held for a whole window, so the figure a run is judged by is well under
    // the threshold even at the run's worst sustained stretch. The ratio is
    // printed because it is the headroom claim itself, and a change that eats
    // it should be readable here rather than inferred from a pass.
    println!(
        "worst residual {head:.4} rad (head) / {antennas:.4} rad (antennas), worst held for a \
         {window}-period window {sustained:.4} rad, against a {:.4} rad screen: {:.2}x headroom \
         on the worst period and {:.2}x on the worst window",
        cfg.tracking.threshold_rad,
        cfg.tracking.threshold_rad / head.max(antennas),
        cfg.tracking.threshold_rad / sustained,
    );
    assert!(
        sustained < cfg.tracking.threshold_rad,
        "the library held {sustained:.4} rad for a whole {window}-period window, which the \
         {:.4} rad screen no longer clears",
        cfg.tracking.threshold_rad
    );
    // The sizing rule itself, asserted rather than described: the shipped
    // screen stands half again over the worst residual these recordings hold.
    // The threshold's own derivation is that ratio, so a re-cut fixture or a
    // change to the model that eats the margin fails here instead of leaving
    // the derivation stated against a figure the recordings no longer show.
    assert!(
        cfg.tracking.threshold_rad >= 1.5 * head.max(antennas),
        "the screen is {:.4} rad and the recordings' worst residual is {:.4} rad: the threshold \
         is sized at half again over what a healthy machine shows",
        cfg.tracking.threshold_rad,
        head.max(antennas)
    );
}

/// The worst residual a leg ran at on the library's worst leg reversal,
/// radians.
///
/// Local to this suite: the two library constants are the figures a report
/// prints a live run against, and those are the head's and the antennas'. This
/// one is what one fixture is kept for — the excursion a crank makes when the
/// goal turns round through it, which the aggregate head maximum (a body yaw
/// reversal, half again this figure) hides. What is left after the class's
/// following lag is taken out of it, the lag being 0.0288 rad of travel at this
/// pair: the excursion is the reversal itself, which the model does not carry
/// and the threshold's margin covers.
const CLIP_WORST_LEG_RESIDUAL_RAD: f64 = 0.2691;

/// The worst residual a leg ran at on the confirmation tour of the commissioned
/// pair, radians.
///
/// The reason that tour was flown, held where it can be replayed: the same six
/// cranks on the same library at `287 / 326` instead of `20 / 50`, a generator
/// twelve times the velocity and fourteen times the ramp, judged against the
/// model of the pair they actually ran. A crank that could not follow the pair
/// it was commissioned at would stand further from its own trajectory here, not
/// nearer, and 0.4 rad is the bound that tour was read against.
///
/// Read under the lag measured on the class's own loop.
const CONFIRM_WORST_LEG_RESIDUAL_RAD: f64 = 0.1983;

/// The worst residual an antenna ran at on the stiffer gains the class no
/// longer runs, radians.
///
/// `proud1`'s reversal, at `500 / 0 / 100` and judged at the generator alone
/// because no lag has been read on that loop. It is the library's largest
/// antenna sample and it is not the class's figure — the class ships
/// `200 / 0 / 0` and is judged under the lag measured there — so the aggregate
/// assertion needs this constant beside the pin to say what the library holds.
/// What a gains change costs the follow is the lag itself, 1.2 periods at
/// `200` and unread at `500`.
const CLIP_WORST_ANTENNA_RESIDUAL_AT_STIFF_GAINS_RAD: f64 = 0.3782;

/// The worst residual an antenna ran at on the `20 / 50` pair, a pair the
/// class does not ship, radians.
///
/// `stumble_and_recover`'s reversal off the confirmation tour, at the
/// `200 / 0 / 0` gains the class runs and under the lag read on them, but at a
/// pair the class does not carry. Kept because the aggregate assertion needs
/// every kept fixture's antenna worst beside the pin: the reversal excursion
/// the model does not carry is a fixed number of radians, so it reads larger
/// against the slow pair's small lag than against the fast pair's large one.
const CLIP_WORST_ANTENNA_RESIDUAL_AT_THE_SLOW_PAIR_RAD: f64 = 0.3625;

/// The worst residual an antenna ran at on the sweep that confirmed the
/// commissioned pair, radians.
///
/// The residual instrument of the confirmation: streamed content at the
/// class's own cap, reversals at arrival and mid-move, and the outboard arc,
/// which is the library's antenna stress in miniature at a shape that keeps the
/// setpoint moving at constant speed — the shape a following lag is largest
/// under. Local to this suite because the figure a live antenna run is reported
/// against is the class's pin, and this is one run of six that confirmed it.
const SWEEP_WORST_ANTENNA_RESIDUAL_RAD: f64 = 0.2105;

/// The worst residual an antenna ran at on the step probe of the same
/// confirmation, radians.
///
/// The largest of the six confirmation runs, and the shape that produces it is
/// the one the sweep leaves out: a one-frame goal step of more than a radian,
/// where the generator ramps from rest and the shaft follows it a lag behind.
/// It is a hair under the tour's own worst at this pair, which is the class's
/// pin, and that is the sizing rule's headroom read twice on two contents.
const STEP_PROBE_WORST_ANTENNA_RESIDUAL_RAD: f64 = 0.2535;

/// The worst residual the shipped wake gesture and the hold after it ran at,
/// radians: head then antennas.
///
/// Local for the same reason, and kept because this recording is the evidence
/// for a direction no other fixture holds. A change of dead time moves the
/// model's clock by one period along the generator's own path, and it moved
/// this pin's antennas *down* a period of travel — 0.0988 to 0.0748, 0.024 rad
/// at `50` units — because the wake antennas' worst sample sat ahead of its
/// prediction, while it moved the tour's pins *up* the same period because
/// their worst samples sat behind. The two logs pull opposite ways with depth,
/// so a depth minimised on either one alone would mis-time the other; that is
/// why the depth is measured rather than fitted, and the measurement is
/// `plant::RESPONSE_DEAD_SAMPLES`' own comment. This pair is the one fixture
/// holding the downward sign.
///
/// The head half is read under the legs' following lag, 0.0288 rad of travel
/// at `50` units. The
/// antenna half is unmoved: this recording's antennas ran `500 / 0 / 100`, a
/// loop with no reading, and are judged at the generator alone.
const WAKE_WORST_RESIDUAL_RAD: (f64, f64) = (0.1058, 0.0748);

/// The worst a joint ran behind its *goal* on the library's longest travel,
/// radians.
///
/// Kept beside that fixture's residual because the pair is the whole claim: an
/// antenna three radians behind the goal a clip asked for, standing a tenth of
/// a radian from where its own servo's generator had got to. The first figure
/// is what the content asked of the machine and says nothing about health; the
/// second is what the detector screens on.
const CLIP_WORST_ANTENNA_LAG_RAD: f64 = 2.9622;

/// The most any joint on that recording stood from its own prediction, radians:
/// the bound the lag figure beside it is contrasted with.
const CLIP_LONGEST_TRAVEL_RESIDUAL_BOUND_RAD: f64 = 0.1;

fn is_leg(joint: JointRef) -> bool {
    group_of(joint) == Some(JointGroup::Legs)
}

/// Guard 1. Each recording cut from the library holds the figure it is kept
/// for.
///
/// The test beside the aggregate one, per fixture rather than over every row of
/// [`TOUR_FIXTURES`] at once. The library's two worst residuals are a maximum
/// and the aggregate pins are where they belong; the rest of what these files
/// are kept for is per file, and a maximum over the table says nothing about
/// any of them. Cut a
/// window differently, truncate one, swap two, and the aggregate is unmoved —
/// which is the rot the README's own contract says one test per file prevents.
///
/// Every figure here is the one the README row states, so a re-cut fixture
/// fails naming its own file and the README and the suite cannot drift.
#[test]
fn each_recorded_clip_pins_the_figure_it_is_kept_for() {
    let cfg = MotionConfig::default();
    let judged = |name: &str| {
        let trace = fixture(name);
        let run = trace.run(0);
        replay(&cfg, &tour_plant(name), run).judged
    };

    // The worst residual on record is a body yaw reversal on `no_sad1`, and it
    // is the figure the threshold carries its margin over.
    let no_sad1 = worst_residual(&judged("trace-tour-no-sad1"), is_head);
    assert!(
        (no_sad1 - RECORDED_WORST_HEAD_RESIDUAL_RAD).abs() < 5e-3,
        "no_sad1's worst head residual is {no_sad1:.4} rad"
    );

    // The worst antenna reversal at the `20 / 50` pair, a pair the class does
    // not ship: the same loop it runs, at a generator twelve times slower than
    // the commissioned one.
    let stumble = worst_residual(&judged("trace-tour-stumble-and-recover"), is_antenna);
    assert!(
        (stumble - CLIP_WORST_ANTENNA_RESIDUAL_AT_THE_SLOW_PAIR_RAD).abs() < 5e-3,
        "stumble_and_recover's worst antenna residual is {stumble:.4} rad"
    );

    // `proud1` is the same class on the same pair at the stiffer gains it no
    // longer runs, judged at the generator alone for want of a reading on that
    // loop, and it holds its own figure for that reason.
    let proud1 = worst_residual(&judged("trace-tour-proud1"), is_antenna);
    assert!(
        (proud1 - CLIP_WORST_ANTENNA_RESIDUAL_AT_STIFF_GAINS_RAD).abs() < 5e-3,
        "proud1's worst antenna residual is {proud1:.4} rad"
    );

    // The commissioned leg pair, on the tour that confirmed it: the cranks'
    // worst against the 0.4 rad bound that tour was read against.
    let grid_snap = worst_residual(&judged("trace-tour-grid-snap"), is_leg);
    assert!(
        (grid_snap - CONFIRM_WORST_LEG_RESIDUAL_RAD).abs() < 5e-3,
        "grid_snap's worst leg residual is {grid_snap:.4} rad"
    );

    // The antennas at the pair they are commissioned at, on the only tour
    // of the whole library that flew it: the class's worst antenna window at
    // its shipping configuration, and the antenna pin. The bound is the sizing
    // rule's own worst -- the threshold divided by the margin it is derived at
    // -- so a re-cut window or a model change that pushed this over it would be
    // a pair that cannot be commissioned under this detector, read here rather
    // than as a red sizing gate with the threshold as the only edit that clears
    // it.
    let sharp_side_tilt = worst_residual(&judged("trace-tour-sharp-side-tilt-fast"), is_antenna);
    assert!(
        (sharp_side_tilt - RECORDED_WORST_ANTENNA_RESIDUAL_RAD).abs() < 5e-3,
        "sharp_side_tilt's worst antenna residual at the commissioned pair is \
         {sharp_side_tilt:.4} rad"
    );
    assert!(
        sharp_side_tilt <= cfg.tracking.threshold_rad / 1.5,
        "the antennas read {sharp_side_tilt:.4} rad at `522 / 640`, over the {:.4} rad a \
         {:.4} rad screen sized at half again leaves a healthy machine: that pair is not \
         commissionable under this detector and the record, not the threshold, is what moves",
        cfg.tracking.threshold_rad / 1.5,
        cfg.tracking.threshold_rad
    );

    // The two runs that confirmed the pair on the machine, armed: the sweep
    // that is the residual instrument, and the step probe that holds the
    // largest antenna sample of the six. Both ran the whole shipping profile,
    // so both are screened against the same bound the tour is, and the three
    // figures together are what the commissioning rests on.
    let sweep = worst_residual(&judged("trace-probe-antenna-sweep"), is_antenna);
    assert!(
        (sweep - SWEEP_WORST_ANTENNA_RESIDUAL_RAD).abs() < 5e-3,
        "the confirmation sweep's worst antenna residual is {sweep:.4} rad"
    );
    let step_probe = worst_residual(&judged("trace-probe-antenna-step-a"), is_antenna);
    assert!(
        (step_probe - STEP_PROBE_WORST_ANTENNA_RESIDUAL_RAD).abs() < 5e-3,
        "the confirmation step probe's worst antenna residual is {step_probe:.4} rad"
    );
    for (name, worst) in [
        ("the confirmation sweep", sweep),
        ("the confirmation step probe", step_probe),
    ] {
        assert!(
            worst <= cfg.tracking.threshold_rad / 1.5,
            "{name} read {worst:.4} rad on the antennas, over the {:.4} rad a {:.4} rad screen \
             sized at half again leaves a healthy machine",
            cfg.tracking.threshold_rad / 1.5,
            cfg.tracking.threshold_rad
        );
    }

    // The worst *leg* reversal is `side_peekaboo`'s, and a crank's excursion is
    // its own figure: the head maximum is a yaw and reads a third again this.
    let peekaboo = worst_residual(&judged("trace-tour-side-peekaboo"), is_leg);
    assert!(
        (peekaboo - CLIP_WORST_LEG_RESIDUAL_RAD).abs() < 5e-3,
        "side_peekaboo's worst leg residual is {peekaboo:.4} rad"
    );

    // The longest travel on record, and the standing case that a lag says
    // nothing about health: three radians behind the goal, a tenth of a radian
    // from its own trajectory. Both halves read off the one recording, because
    // the contrast is the claim.
    let toc_toc_toc = fixture("trace-tour-toc-toc-toc");
    let lag = [JointRef::AntennaRight, JointRef::AntennaLeft]
        .into_iter()
        .map(|joint| toc_toc_toc.run(0).joint(joint).worst_lag)
        .fold(0.0_f64, f64::max);
    assert!(
        (lag - CLIP_WORST_ANTENNA_LAG_RAD).abs() < 5e-3,
        "toc_toc_toc's worst antenna lag is {lag:.4} rad"
    );
    let toc_residual = worst_residual(&judged("trace-tour-toc-toc-toc"), is_head).max(
        worst_residual(&judged("trace-tour-toc-toc-toc"), is_antenna),
    );
    assert!(
        toc_residual < CLIP_LONGEST_TRAVEL_RESIDUAL_BOUND_RAD,
        "toc_toc_toc's worst residual is {toc_residual:.4} rad, and what this recording is kept \
         for is a joint radians behind its goal and a tenth of a radian from its own generator"
    );
    assert!(
        lag > 10.0 * toc_residual,
        "the lag is {lag:.4} rad and the residual {toc_residual:.4} rad: this recording is kept \
         for the distance between the two figures"
    );

    // The shipped gesture, at the profile the deployment commissions. The
    // unsaturated run in the library, which is why the dead time is pinned
    // against it as well as against the tour.
    let wake = judged("trace-wake-20260906");
    let (wake_head, wake_antennas) = (
        worst_residual(&wake, is_head),
        worst_residual(&wake, is_antenna),
    );
    assert!(
        (wake_head - WAKE_WORST_RESIDUAL_RAD.0).abs() < 5e-3,
        "the wake gesture's worst head residual is {wake_head:.4} rad"
    );
    assert!(
        (wake_antennas - WAKE_WORST_RESIDUAL_RAD.1).abs() < 5e-3,
        "the wake gesture's worst antenna residual is {wake_antennas:.4} rad"
    );
}

/// Guard 1. The bench recordings raise nothing under their own profile either,
/// and their residuals are pinned at that profile.
///
/// The same model at a second setting: these runs were driven by servos
/// commissioned at `400 / 600` on a 32 ms grid, and the arithmetic that judges
/// them is the deployment's with two registers changed. A model carrying a
/// constant read off one profile would show up here rather than on the machine.
#[test]
fn the_bench_runs_that_went_well_pin_their_residuals_at_their_own_profile() {
    let cfg = MotionConfig::default();
    let verify2_trace = fixture("trace-verify2");
    let fast4_trace = fixture("trace-fast4");
    let verify2 = {
        let run = verify2_trace.run(0);
        replay(&cfg, &bench_plant(run), run)
    };
    let fast4 = {
        let run = fast4_trace.run(0);
        replay(&cfg, &bench_plant(run), run)
    };
    for (name, outcome) in [("trace-verify2", &verify2), ("trace-fast4", &fast4)] {
        assert!(
            outcome.trips.is_empty(),
            "{name}: {:?}",
            outcome
                .trips
                .iter()
                .map(|trip| (trip.at, flags::Names(trip.exhausted).to_string()))
                .collect::<Vec<_>>()
        );
    }

    let head = worst_residual(&verify2.judged, is_head);
    assert!(
        (head - BENCH_WORST_HEAD_RESIDUAL_RAD).abs() < 5e-3,
        "the validated gesture's worst head residual is {head:.4} rad"
    );
    let antennas = worst_residual(&fast4.judged, is_antenna);
    assert!(
        (antennas - BENCH_WORST_ANTENNA_RESIDUAL_RAD).abs() < 5e-3,
        "the fast sweep's worst antenna residual is {antennas:.4} rad"
    );
    // And what the two figures are of, which is the point of keeping them. The
    // head on the validated gesture reads three hundredths of a radian and the
    // antenna on the fastest content in the tree reads a third of one, both
    // under the shipped tour's own worsts — at `400 / 600` against the
    // commissioned `20 / 50`, on a 32 ms grid against the shipped 20 ms. So the
    // model is the servo's generator at either setting. The screen's margin
    // stays a figure about one profile on one grid; the evidence is that another
    // pair on another grid still reads under it.
    assert!(
        head < cfg.tracking.threshold_rad,
        "the validated gesture's head now reaches the {:.4} rad screen",
        cfg.tracking.threshold_rad
    );
    assert!(
        antennas < cfg.tracking.threshold_rad,
        "the fast sweep's antenna now reaches the {:.4} rad screen",
        cfg.tracking.threshold_rad
    );
}

/// Guard 1. The lag-and-speed pairs the shipped tracking comment quotes are
/// what the recordings hold.
///
/// Not a fault case: nothing here crosses a threshold. It is the documentation
/// guard behind `TrackingFaultConfig`'s argument that lag scales with commanded
/// speed — a leg and an antenna each pinned as a lag beside the speed of the
/// goal it was chasing, so a re-recording or a shaper change moves the comment
/// rather than leaving its figures standing as folklore. Both figures of a pair
/// come from one joint: a lag read against a speed some other joint was
/// commanded at would support nothing.
#[test]
fn the_lag_and_speed_figures_the_tracking_comment_quotes_are_what_the_recordings_hold() {
    let verify2 = fixture("trace-verify2");
    // Leg 2 as the bus numbers the servos, which is the second leg row.
    let leg = verify2.run(0).joint(JointRef::Leg1);
    assert!(
        (leg.worst_lag - 0.245).abs() < 5e-3,
        "leg 2's lag on the validated gesture is {:.4} rad",
        leg.worst_lag
    );
    assert!(
        (leg.peak_goal_speed - 3.34).abs() < 5e-3,
        "the goal leg 2 was following peaks at {:.4} rad/s",
        leg.peak_goal_speed
    );

    let worst_antenna = |trace: &Trace| {
        [JointRef::AntennaRight, JointRef::AntennaLeft]
            .into_iter()
            .map(|joint| trace.run(0).joint(joint))
            .max_by(|left, right| left.worst_lag.total_cmp(&right.worst_lag))
            .expect("the pair is not empty")
    };
    let antenna = worst_antenna(&verify2);
    assert!(
        (antenna.worst_lag - 0.82).abs() < 5e-3,
        "the antennas' lag on the validated gesture is {:.4} rad",
        antenna.worst_lag
    );
    assert!(
        (antenna.peak_goal_speed - 7.55).abs() < 5e-3,
        "the goal that antenna was following peaks at {:.4} rad/s",
        antenna.peak_goal_speed
    );

    // The fast sweep's own pair. The 855°/s the comment names is what the joint
    // reached, pinned by the speed-record case below; what it was asked for is
    // half again as fast, and that is the speed its 1.38 rad of lag is read
    // against.
    let fast_antenna = worst_antenna(&fixture("trace-fast4"));
    assert!(
        (fast_antenna.worst_lag - RECORDED_WORST_ANTENNA_LAG_RAD).abs() < 5e-3,
        "the fast sweep's worst antenna lag is {:.4} rad",
        fast_antenna.worst_lag
    );
    assert!(
        (fast_antenna.peak_goal_speed - deg(1123.0)).abs() < deg(2.0),
        "the goal that antenna was following peaks at {:.1} deg/s",
        fast_antenna.peak_goal_speed.to_degrees()
    );
}

/// Guard 2. The one collision on record trips it, on the pair that stalled and
/// on nothing else.
///
/// Both antenna tips met at the crossing and stood there for over forty periods
/// with the goal three radians away; the head carried on and arrived. So the
/// comparison has to name the antennas and only the antennas — which is what
/// decides the response: the pair goes out of service and the head move
/// finishes, rather than the whole machine winding down.
#[test]
fn the_collision_trips_it_on_the_antennas_and_nothing_else() {
    let cfg = MotionConfig::default();
    let trace = fixture("trace-stagger");
    let trips = replay(&cfg, &bench_plant(trace.run(2)), trace.run(2)).trips;

    assert!(
        !trips.is_empty(),
        "the stalled pair never ran its window out"
    );
    for trip in &trips {
        for joint in flags::iter(trip.exhausted) {
            assert_eq!(
                group_of(joint),
                Some(JointGroup::Antennas),
                "a head joint ran its window out at period {}: {}",
                trip.at,
                flags::Names(trip.exhausted)
            );
            // The window ran out on a joint that was genuinely far from where
            // its own generator had got to, and not on one sitting a hair past
            // the screen: the pair stood still with the goal radians away, so
            // the residual that carried this is most of a radian and clears the
            // threshold several times over. A model with the dead time or the
            // acceleration wrong could still run a window out here; one that
            // did so at the threshold's own edge would not.
            let residual = trip.residuals[row(joint).expect("a bus row")];
            assert!(
                residual > 2.0 * cfg.tracking.threshold_rad,
                "the {joint:?} window ran out at period {} on a residual of {residual:.4} rad, \
                 which is inside twice the {:.4} rad threshold: the collision on record stood \
                 the pair much further off its trajectory than that",
                trip.at,
                cfg.tracking.threshold_rad,
            );
        }
    }
    // Both sides, and inside one window of each other: they met each other, so
    // neither is the one that failed. Two antennas stalling hundreds of periods
    // apart would be two single-servo faults, which is a different condition
    // with a different answer.
    let ran_out = |side| {
        trips
            .iter()
            .find(|trip| flags::contains(trip.exhausted, side))
            .map(|trip| trip.at)
    };
    let right = ran_out(JointRef::AntennaRight).expect("the right antenna stalled");
    let left = ran_out(JointRef::AntennaLeft).expect("the left antenna stalled");
    assert!(
        right.abs_diff(left) <= u64::from(cfg.tracking.ticks),
        "the antennas ran their windows out at periods {right} and {left}, further apart than \
         the {} the window itself is: one stalled and the other did not",
        cfg.tracking.ticks
    );
}

/// Guard 3. The step bounds admit the gestures that were recorded, with the
/// headroom the shipped figures claim.
///
/// Against the *plan* and never against the record. The recorded goal column is
/// what the loop commanded, and these recordings predate the per-period move
/// clock, so a period that started late sampled the trajectory further along
/// and commanded a step no planner ever asked for. The last case below is that
/// inflation, pinned: the fastest sweep's recorded step is past the bound its
/// own plan clears comfortably.
#[test]
fn the_step_bounds_admit_the_recorded_gestures_with_headroom() {
    let cfg = MotionConfig::default();
    let verify2 = fixture("trace-verify2");
    let fast4 = fixture("trace-fast4");

    // The validated gesture, on the clock the cog path ships, and the same
    // gesture with the staggered antenna pair the recording was made on —
    // whose quick side is the 0.3 s sweep the speed record was set on.
    let cases = [
        (
            "the validated gesture",
            started_at(&cfg, verify2.run(0)),
            gesture(MoveDurations::uniform(UP)),
        ),
        (
            "the staggered pair",
            started_at(&cfg, fast4.run(0)),
            gesture(MoveDurations {
                head: UP,
                antennas: [Duration::from_millis(700), Duration::from_millis(300)],
            }),
        ),
    ];

    let mut planned = Vec::new();
    for (name, start, command) in cases {
        let peaks =
            dry_pass_peaks(&cfg, &start, &command, TICK_HZ).expect("the gesture is measurable");
        planned.push(peaks);
        assert!(
            peaks.legs * 2.0 <= cfg.max_step.legs,
            "{name}: legs plan {:.4} rad against the {:.4} rad bound",
            peaks.legs,
            cfg.max_step.legs
        );
        for (side, peak) in ["right", "left"].into_iter().zip(peaks.antennas) {
            assert!(
                peak * 1.5 <= cfg.max_step.antennas,
                "{name}: the {side} antenna plans {peak:.4} rad against the {:.4} rad bound",
                cfg.max_step.antennas
            );
        }
        // And no clock needs right-sizing for its span, which is the same
        // statement the shipped durations make about their floors. The pass may
        // still lengthen an antenna to de-phase the pair — the head's clock is
        // what says nothing was floored.
        let stretch = floor_move_clock(&cfg, &start, &command, TICK_HZ).1;
        assert!(
            stretch.is_none_or(
                |clocks| clocks.dephased && clocks.effective.head == clocks.requested.head
            ),
            "{name}: the shipped clock does not carry it"
        );
    }

    // The inflation, pinned. On the quick side the loop commanded half as much
    // again in a period as the planner ever asked for, because the periods it
    // woke on were half as long again as the grid it was sampling. A bound
    // sized to clear the record by the same margin would be half as wide again
    // for no reason the plan gives.
    let quick = planned[1].antennas[1];
    let recorded = fast4.run(0).joint(JointRef::AntennaLeft).peak_goal_step;
    assert!(
        recorded > quick * 1.4,
        "the recorded step {recorded:.4} rad is no longer inflated over the planned {quick:.4} \
         rad, so this case no longer says what it is for"
    );
}

/// Guard 4. The separation a pair is held to admits the clock pair that swept
/// clean and rejects the one that clashed.
///
/// Both figures come off the recordings' own commanded goals, which is where a
/// clock pair's phase is visible. The pair that clashed is the binding end: the
/// offset it stood at when the second tip reached the band is well under the
/// constant, so nothing phased like that gets through. The stow that clashed
/// and the raise that did not are the same two clocks recorded twice — the
/// raise got through on the odds the debrief measured, about two inboard sweeps
/// in three, and the check rejects it too.
#[test]
fn the_separation_tells_the_clean_pair_from_the_one_that_clashed() {
    let cfg = MotionConfig::default();
    let band = cfg.phase.contact_band_rad;
    let stagger = fixture("trace-stagger");
    let fast4 = fixture("trace-fast4");

    let clean = fast4
        .run(0)
        .separation(band, Sample::goal_of)
        .expect("both antennas cross the band");
    assert!(
        clean.met(ANTENNA_PHASE_SEPARATION_RAD),
        "the validated pair plans {:.4} rad, under the shipped separation",
        clean.offset
    );
    assert!(
        (clean.offset - 0.876).abs() < 5e-3,
        "the validated pair plans {:.4} rad",
        clean.offset
    );

    let clashed = stagger
        .run(2)
        .separation(band, Sample::goal_of)
        .expect("both antennas cross the band");
    assert!(
        !clashed.met(ANTENNA_PHASE_SEPARATION_RAD),
        "the pair that clashed planned {:.4} rad, which the shipped separation now admits",
        clashed.offset
    );
    assert!(
        (clashed.offset - 0.361).abs() < 5e-3,
        "the pair that clashed planned {:.4} rad",
        clashed.offset
    );

    let survived = stagger
        .run(1)
        .separation(band, Sample::goal_of)
        .expect("both antennas cross the band");
    assert!(
        !survived.met(ANTENNA_PHASE_SEPARATION_RAD),
        "the raise on the clashing clocks planned {:.4} rad, which the shipped separation now \
         admits",
        survived.offset
    );

    // And what the tips themselves did, which is what the plan is a proxy for:
    // on the raise they passed the band's edge a third of a radian apart, and on
    // the stow they never reached it at all — they met inside the band and
    // stalled there.
    let tips = stagger
        .run(1)
        .separation(band, Sample::present_of)
        .expect("both antennas cross the band");
    assert!(
        (tips.offset - 0.285).abs() < 5e-3,
        "the tips passed {:.4} rad apart",
        tips.offset
    );
    assert!(
        stagger
            .run(2)
            .separation(band, Sample::present_of)
            .is_none(),
        "the jammed pair left the band after all"
    );
}

/// Guard 5, one test per fixture. The validated gesture: the whole machine,
/// head and both antennas, up in 0.82 s and measurably there before the last
/// goal went out.
///
/// Zero settle is the claim this recording is kept for — not one period was
/// spent waiting after the commanding stopped — and it is the shape every guard
/// in this crate has to admit.
#[test]
fn the_validated_gesture_arrives_inside_its_own_clock() {
    let trace = fixture("trace-verify2");
    assert_eq!(trace.runs(), 1);
    let run = trace.run(0);

    assert_eq!(run.samples.len(), 27);
    assert!(
        (run.span().as_secs_f64() - 0.8196).abs() < 5e-4,
        "{:?}",
        run.span()
    );
    assert_eq!(run.commanding_end(), Some(run.span()));
    for joint in ROWS {
        let metrics = run.joint(joint);
        let arrived = metrics.arrived.expect("it got there");
        assert!(
            arrived <= run.span(),
            "{joint:?} arrived at {arrived:?}, after the commanding stopped"
        );
        let residual = metrics.residual.expect("it was holding a goal");
        assert!(residual < ARRIVED_TOLERANCE_RAD, "{joint:?}: {residual}");
    }
    // The fastest leg was asked for 0.107 rad in a period. That is more than
    // the same command's dry-pass step: this run was driven by a loop that
    // sampled the trajectory at wall-clock time, so a period that started late
    // commanded the extra. It is recorded, and nothing is sized against it.
    let leg = run.joint(JointRef::Leg1);
    assert!(
        (leg.peak_goal_step - 0.1068).abs() < 5e-4,
        "{}",
        leg.peak_goal_step
    );
    assert!(
        (leg.peak_speed - deg(199.3)).abs() < deg(0.5),
        "{}",
        leg.peak_speed
    );
}

/// Guard 5. The antenna speed record, and the staggered pair that made it safe:
/// one side sweeping 187° in 0.40 s at 855°/s while the other takes 0.93 s over
/// the same arc.
#[test]
fn the_fast_sweep_is_the_speed_record_and_a_staggered_pair() {
    let run = fixture("trace-fast4");
    let run = run.run(0);
    let fast = run.joint(JointRef::AntennaLeft);
    let slow = run.joint(JointRef::AntennaRight);

    assert!((fast.span - deg(187.0)).abs() < deg(0.5), "{}", fast.span);
    assert!(
        (fast.peak_speed - deg(855.5)).abs() < deg(1.0),
        "{}",
        fast.peak_speed
    );
    let arrived = fast.arrived.expect("it got there");
    assert!((arrived.as_secs_f64() - 0.4035).abs() < 5e-4, "{arrived:?}");
    // The other side is on its own clock, more than half a second behind —
    // which is the whole point of the pair having two.
    let behind = slow.arrived.expect("it got there") - arrived;
    assert!(behind > Duration::from_millis(500), "{behind:?}");
    assert!(
        slow.peak_speed < fast.peak_speed / 2.0,
        "{} against {}",
        slow.peak_speed,
        fast.peak_speed
    );
}

/// Guard 5. The gain change, recorded as the same step command twice: the
/// P-only gains of that night park the loaded pair ~4° short of the goal for
/// good, and the tuned gains bring that to about a degree.
///
/// Neither run is a clean any bound can be replayed against. Both command the
/// whole span in one period by construction — that is what a step response is —
/// so the goal steps here are records of the instrument, not of a move anything
/// should admit.
#[test]
fn the_gain_change_is_two_step_responses_and_the_droop_between_them() {
    let trace = fixture("trace-newgains");
    assert_eq!(trace.runs(), 2);
    let shipped = trace.run(0);
    let tuned = trace.run(1);

    for (index, run) in [(0, shipped), (1, tuned)] {
        for joint in ROWS {
            let metrics = run.joint(joint);
            if metrics.span > deg(10.0) {
                assert!(
                    metrics.peak_goal_step > metrics.span / 2.0,
                    "run {index}, {joint:?}: the goal jumped {} of a {} span",
                    metrics.peak_goal_step,
                    metrics.span
                );
            }
        }
    }

    // Legs 2 and 5 carry the load. Under the gains of that night they stop
    // short and stay short — the steady-state droop of a proportional term
    // holding gravity.
    for leg in [JointRef::Leg1, JointRef::Leg4] {
        let droop = shipped.joint(leg);
        let residual = droop.residual.expect("it was holding a goal");
        assert!(
            (deg(3.9)..=deg(4.4)).contains(&residual),
            "{leg:?}: {}",
            residual.to_degrees()
        );
        assert_eq!(droop.arrived, None, "{leg:?} never got there");

        let after = tuned.joint(leg);
        let residual = after.residual.expect("it was holding a goal");
        assert!(residual < deg(1.3), "{leg:?}: {}", residual.to_degrees());
        assert!(after.arrived.is_some(), "{leg:?} got there");
    }
}

/// Guard 5. The one collision on record: both antenna tips meet at the inboard
/// crossing, stall against each other at mirrored angles for over 40 periods —
/// about 1.06 s — while the goal walks away, and spring back when the servos
/// give up.
///
/// This is the run every guard has to catch. It is the third in its file — the
/// same session's earlier raise, run 1, went through cleanly.
#[test]
fn the_collision_stalls_both_antennas_at_mirrored_angles() {
    let trace = fixture("trace-stagger");
    assert_eq!(trace.runs(), 3);
    let jam = trace.run(2);
    let right = jam
        .longest_stall(JointRef::AntennaRight, deg(1.0))
        .expect("it stopped");
    let left = jam
        .longest_stall(JointRef::AntennaLeft, deg(1.0))
        .expect("it stopped");

    // Mirrored, which is what tip-to-tip means: the two sides stop at the same
    // angle on opposite sides, a few degrees apart.
    for (side, stall) in [("right", right), ("left", left)] {
        assert!(
            (deg(52.0)..=deg(56.6)).contains(&stall.at.abs()),
            "{side}: {}",
            stall.at.to_degrees()
        );
        // Held there while the goal ran the rest of the way home.
        assert!(stall.periods >= 40, "{side}: {} periods", stall.periods);
        assert!(
            stall.worst_lag > deg(120.0),
            "{side}: {}",
            stall.worst_lag.to_degrees()
        );
    }
    assert!(
        right.at.signum() != left.at.signum(),
        "opposite sides: {} and {}",
        right.at.to_degrees(),
        left.at.to_degrees()
    );
    assert!(
        (right.at + left.at).abs() < deg(5.0),
        "{} against {}",
        right.at.to_degrees(),
        left.at.to_degrees()
    );

    // Neither arrived, and both finished further from the goal than they
    // stalled: the tips sprang apart as the servos dropped out.
    for (joint, stall) in [
        (JointRef::AntennaRight, right),
        (JointRef::AntennaLeft, left),
    ] {
        let antenna = jam.joint(joint);
        assert_eq!(antenna.arrived, None, "{joint:?}");
        let residual = antenna.residual.expect("it was holding a goal");
        assert!(residual > stall.worst_lag, "{joint:?}: {residual}");
    }
}

/// Guard 5. What the model reads over the gain change, which is the only run on
/// record of a servo that did not keep up with its own generator.
///
/// Both runs command the whole span in a single period, so the modelled
/// generator ramps to its cap while the servo answers with its position loop
/// alone — the model is describing a move nobody planned, and the residual it
/// reads is most of the span. What the two runs differ in is the speed the
/// servo answered at: under the P-only gains of that night the antennas covered
/// about half the profile velocity per period and under the tuned gains most of
/// it. Half is where `pace_min` sits, so the first run is the one recording of
/// a joint the pace rule does not carry and the second is a joint it does. Read
/// as a hardware reading and not as a screen: no shipped command steps a goal
/// like this, and what the pair says is which side of the pace floor a badly
/// tuned servo lands on.
#[test]
fn the_gain_change_is_a_servo_under_the_pace_floor_and_the_same_servo_over_it() {
    let cfg = MotionConfig::default();
    let trace = fixture("trace-newgains");
    let outcomes = [0, 1].map(|index| {
        let run = trace.run(index);
        replay(&cfg, &bench_plant(run), run)
    });
    for (index, outcome) in [(0, &outcomes[0]), (1, &outcomes[1])] {
        let worst = worst_residual(&outcome.judged, is_head)
            .max(worst_residual(&outcome.judged, is_antenna));
        let settled = outcome
            .judged
            .last()
            .map(|residuals| {
                worst_residual(std::slice::from_ref(residuals), is_head)
                    .max(worst_residual(std::slice::from_ref(residuals), is_antenna))
            })
            .expect("the run was judged");
        println!(
            "run {index} reads {worst:.4} rad off the model at its worst and {settled:.4} rad on \
             the last period it was held on, and ran its window out on {} period(s)",
            outcome.trips.len()
        );
        assert!(
            worst > cfg.tracking.threshold_rad,
            "run {index} reads {worst:.4} rad off the model at its worst, which no longer says \
             what a step command does to a generator"
        );
    }

    // The pre-change run: both antennas, and only the antennas, run their
    // windows out. They are the rows whose span is radians rather than
    // fractions of one, so they are the rows a speed shortfall accumulates on.
    assert!(
        !outcomes[0].trips.is_empty(),
        "the servos that could not keep up now keep up"
    );
    for trip in &outcomes[0].trips {
        for joint in flags::iter(trip.exhausted) {
            assert!(
                is_antenna(joint),
                "{joint:?} ran its window out at period {}: {}",
                trip.at,
                flags::Names(trip.exhausted)
            );
        }
    }

    // And after the change, the same command on the same machine is carried by
    // the pace rule from end to end.
    assert!(
        outcomes[1].trips.is_empty(),
        "the tuned run: {:?}",
        outcomes[1]
            .trips
            .iter()
            .map(|trip| (trip.at, flags::Names(trip.exhausted).to_string()))
            .collect::<Vec<_>>()
    );

    // Where each run ended, which is the measurement the fixture is kept for:
    // the P-only gains leave the loaded legs standing off their goal for good
    // and the tuned gains bring them home. Once the generator has stopped the
    // model stands on the goal, so the distance from the model is the droop.
    let settled = |outcome: &Replay| {
        worst_residual(
            std::slice::from_ref(outcome.judged.last().expect("the run was judged")),
            is_head,
        )
    };
    let droop = settled(&outcomes[0]);
    assert!(
        (droop - deg(4.0)).abs() < deg(0.6),
        "the droop the shipped gains left reads {:.2}deg off the model",
        droop.to_degrees()
    );
    let tuned = settled(&outcomes[1]);
    assert!(
        tuned < deg(1.3),
        "the tuned gains leave {:.2}deg off the model",
        tuned.to_degrees()
    );
}

/// Guard 6. The watch replayed over a recording says what the watch that ran
/// on the machine said, and the two antennas of one hold are two different
/// mechanisms.
///
/// The 2026-09-07 raise, cut from before its last goal write so the shipped
/// settle allowance is spent inside the file: the left antenna hunts and the
/// right does not, under the same 50 Hz goal rewrite reaching both. What the
/// interval statistics add to the reversal rate is the separation — both rows
/// turn round about 25 times a second, and only one of them does it on a
/// regular period. This is the first fixture the stillness watch is replayed
/// over at all, so it is also the check that a live window and a replayed one
/// are the same window.
///
/// The settle allowance is read here too, because this file and its quiet
/// sibling are the only recordings in the suite that hold a real arrival
/// followed by a real hold: the segmentation — the allowance running from the
/// goal write and ending the period before the judged window opens — is
/// otherwise only ever asserted against series the cases laid out themselves.
#[test]
fn the_recorded_antenna_hold_reads_one_hunt_and_one_dither() {
    let cfg = StillnessConfig::default();
    let trace = fixture("trace-antenna-hunt");
    let windows = trace
        .run(0)
        .holds(cfg, &[JointRef::AntennaLeft, JointRef::AntennaRight]);
    let of = |joint: JointRef| {
        let mut held = windows.iter().filter(|window| window.joint == joint);
        let window = *held.next().unwrap_or_else(|| panic!("{joint:?} held once"));
        assert!(held.next().is_none(), "{joint:?} held once: {windows:?}");
        window
    };

    // The left antenna: the hunt. Every figure here was read off the live
    // run's own report, so a watch that has stopped agreeing with it fails.
    let left = of(JointRef::AntennaLeft);
    assert!(
        (left.opened_after_ns as f64 / 1e9 - 4.00).abs() < 0.03,
        "the left window opened {:.2} s after the setpoint last moved",
        left.opened_after_ns as f64 / 1e9
    );
    assert!(
        left.readings.samples.abs_diff(161) <= 1,
        "the left window judged {} readings",
        left.readings.samples
    );
    let left_reversals = left.readings.reversals_per_s * left.readings.length().as_secs_f64();
    assert!(
        (left_reversals - 81.0).abs() <= 1.0,
        "the left antenna turned round {left_reversals:.1} times"
    );
    let mean = left
        .readings
        .reversal_interval_mean_samples
        .expect("a hunting antenna has intervals");
    let spread = left
        .readings
        .reversal_interval_spread_samples
        .expect("and a spread of them");
    assert!((mean - 1.96).abs() < 0.02, "{left:?}");
    assert!((spread - 0.29).abs() < 0.02, "{left:?}");
    // Near an alias of a quarter of the sample rate, and only near: the phase
    // and amplitude drift across the hold, so the turning is not locked to the
    // grid the goal is rewritten on.
    let hz = left.readings.apparent_frequency_hz().expect("a frequency");
    assert!((hz - 12.7).abs() < 0.3, "{left:?} read {hz:.2} Hz apparent");
    let Err(complaint) = judge(&left, &cfg) else {
        panic!("the recorded hunt judged still: {left:?}");
    };
    assert!(
        (left.readings.excursion_counts() - 9.0).abs() < 1.0,
        "{complaint}"
    );
    // The settle allowance, over the one recording that holds a real arrival
    // followed by a real hold: the segmentation the synthetic square waves
    // cannot check. The allowance is read from the goal write, and it ends the
    // period before the judged window opens, so a reading pulled in from
    // before the command or carried over from a previous hold shows up here.
    let settle = left
        .settle
        .expect("the allowance ahead of the hold was read");
    assert_eq!(
        settle.end_ns,
        left.readings.start_ns - trace.run(0).period_ns(),
        "{settle:?} against a window opening at {}",
        left.readings.start_ns
    );
    assert_eq!(settle.samples, 200, "{settle:?}");
    // 1695 counts is the servo's own travel *after* the raise's last goal
    // write: at the commissioned `20 / 50` the generator trails the streamed
    // path by about two seconds, so the allowance holds the arrival itself and
    // not a ring-down. Which is the reading the allowance exists to keep out of
    // the verdict — the judged tail beside it is nine counts.
    assert!(
        (settle.excursion_counts() - 1695.0).abs() < 1.0,
        "{settle:?}"
    );

    // The right antenna: one count of encoder dither at the same reversal
    // rate, turning round at scattered intervals rather than on a period.
    let right = of(JointRef::AntennaRight);
    assert!(
        right.readings.samples.abs_diff(153) <= 1,
        "the right window judged {} readings",
        right.readings.samples
    );
    assert_eq!(judge(&right, &cfg), Ok(()), "{right:?}");
    assert!(
        (right.readings.excursion_counts() - 1.0).abs() < 0.01,
        "{right:?}"
    );
    let right_spread = right
        .readings
        .reversal_interval_spread_samples
        .expect("a dithering antenna has intervals too");
    assert!(right_spread > 1.0, "{right:?}");
}

/// Guard 6b. The other half of the hold: a recorded antenna pair that held
/// still, which is what the two-count bound was written to pass.
///
/// The 2026-09-08 raise at the vendor's `200 / 0 / 0`, cut the way the hunt
/// file is — from before the raise's last goal write, ending in the first ticks
/// of the stow — so the shipped settle allowance is spent inside the file and
/// the window opens where the live watch opened it. The left antenna is the row
/// that hunts 9 counts at the shipped gains, and over this hold it does not
/// move at all; the right sits inside the count of encoder dither it sits
/// inside everywhere. Without this file the bound is only ever asserted by
/// something failing it, and a watch that judged every hold a hunt would pass
/// the suite.
#[test]
fn the_recorded_quiet_antenna_hold_passes_the_bound() {
    let cfg = StillnessConfig::default();
    let trace = fixture("trace-antenna-still");
    let windows = trace
        .run(0)
        .holds(cfg, &[JointRef::AntennaLeft, JointRef::AntennaRight]);
    let of = |joint: JointRef| {
        let mut held = windows.iter().filter(|window| window.joint == joint);
        let window = *held.next().unwrap_or_else(|| panic!("{joint:?} held once"));
        assert!(held.next().is_none(), "{joint:?} held once: {windows:?}");
        window
    };

    // The left antenna: the hunt's own row, quiet. Zero counts is not a
    // rounding of something small -- the encoder read the same value every
    // period of the judged tail.
    let left = of(JointRef::AntennaLeft);
    assert!(
        (left.opened_after_ns as f64 / 1e9 - 4.00).abs() < 0.03,
        "the left window opened {:.2} s after the setpoint last moved",
        left.opened_after_ns as f64 / 1e9
    );
    assert!(
        left.readings.samples.abs_diff(161) <= 1,
        "the left window judged {} readings",
        left.readings.samples
    );
    assert_eq!(judge(&left, &cfg), Ok(()), "{left:?}");
    assert_eq!(left.readings.excursion_counts(), 0.0, "{left:?}");
    // The same allowance reading as the hunt file's, on the other recording:
    // this file is cut the same way, so the arrival is inside the allowance and
    // the judged tail is the joint at rest.
    let settle = left
        .settle
        .expect("the allowance ahead of the hold was read");
    assert_eq!(
        settle.end_ns,
        left.readings.start_ns - trace.run(0).period_ns(),
        "{settle:?}"
    );
    assert_eq!(settle.samples, 200, "{settle:?}");
    assert!(
        (settle.excursion_counts() - 1731.0).abs() < 1.0,
        "{settle:?}"
    );
    // A row that never turned round has no interval to report, which is the
    // reading that tells a still joint from a slow limit cycle.
    assert_eq!(left.readings.reversals_per_s, 0.0, "{left:?}");
    assert_eq!(
        left.readings.reversal_interval_mean_samples, None,
        "{left:?}"
    );
    assert_eq!(left.readings.apparent_frequency_hz(), None, "{left:?}");

    // The right antenna: the same count of dither it shows at the shipped
    // gains, so this file says the gain change did not buy the right row
    // anything and did not cost it anything either.
    let right = of(JointRef::AntennaRight);
    assert!(
        right.readings.samples.abs_diff(153) <= 1,
        "the right window judged {} readings",
        right.readings.samples
    );
    assert_eq!(judge(&right, &cfg), Ok(()), "{right:?}");
    assert!(
        (right.readings.excursion_counts() - 1.0).abs() < 0.01,
        "{right:?}"
    );
}

/// Guard 6c. The third hold: both rods held at the fold the Minimum Risk
/// Condition commands, which is the pose the stow angle was moved to reach.
///
/// The one pose a fault response leaves the machine in, and the one this cycle
/// changed. The reading that moved [`reachy_motion::disarm::STOW_ANTENNAS`]
/// inboard is a hold at this angle staying inside the bound where the same rung
/// at the same gains hunted 7–8 counts pointing straight down, so the bound
/// passing here is what says the lean did its job. What the file guards is the
/// bound itself: a tightened excursion limit, or a watch that judged every hold
/// a hunt, fails on a hold the machine was accepted at. Whether the pose is
/// *still* quiet after a gain or profile change is a probe night's reading and
/// not this file's; the fold angle the reading was taken at is pinned in
/// `postures.rs`.
///
/// Cut from the first of the six leaned-fold probe runs
/// (`probe-log-20260909T014542Z`), the way the two raise files are: the window
/// opens 0.28 s before the step onto the fold and closes in the first periods
/// of the step off it, so the shipped settle allowance is spent inside the file
/// and the judged window opens where the live watch opened it. The stimulus is
/// `probe/antenna-step-a`, whose one-frame jump makes the arrival the servo's
/// own generator and nothing else's; both rods hold `∓9.6032` rad — the fold a
/// turn up, the representative the sweep lands on.
#[test]
fn the_recorded_leaned_fold_hold_passes_the_bound() {
    let cfg = StillnessConfig::default();
    let trace = fixture("trace-antenna-fold-still");
    let windows = trace
        .run(0)
        .holds(cfg, &[JointRef::AntennaLeft, JointRef::AntennaRight]);
    let of = |joint: JointRef| {
        let mut held = windows.iter().filter(|window| window.joint == joint);
        let window = *held.next().unwrap_or_else(|| panic!("{joint:?} held once"));
        assert!(held.next().is_none(), "{joint:?} held once: {windows:?}");
        window
    };

    for (joint, held) in [
        (JointRef::AntennaRight, of(JointRef::AntennaRight)),
        (JointRef::AntennaLeft, of(JointRef::AntennaLeft)),
    ] {
        // Both windows open a settle allowance after the step, and the pose is
        // held long enough that what is judged is the hold and not the arrival.
        assert!(
            (held.opened_after_ns as f64 / 1e9 - 4.02).abs() < 0.03,
            "{joint:?}'s window opened {:.2} s after the setpoint last moved",
            held.opened_after_ns as f64 / 1e9
        );
        assert!(
            held.readings.samples.abs_diff(123) <= 1,
            "{joint:?}'s window judged {} readings",
            held.readings.samples
        );
        // One count peak to peak on each rod: the encoder's own flicker, which
        // is what the two-count bound was written to pass, and an order under
        // the unleaned pose's limit cycle.
        assert!(
            (held.readings.excursion_counts() - 1.0).abs() < 0.01,
            "{joint:?}: {held:?}"
        );
        assert_eq!(judge(&held, &cfg), Ok(()), "{joint:?}: {held:?}");
        // And the rod is on its goal, not resting short of it against
        // something: the fold is commanded and reached.
        assert!(
            held.mean_error_rad.abs() < 0.002,
            "{joint:?} held {:.4} rad from its goal",
            held.mean_error_rad
        );
        // The arrival is inside the allowance ahead of the window, so the
        // cycling this pose does on the way in is read and not judged.
        let settle = held
            .settle
            .expect("the allowance ahead of the hold was read");
        assert_eq!(
            settle.end_ns,
            held.readings.start_ns - trace.run(0).period_ns(),
            "{joint:?}: {settle:?}"
        );
        assert!(settle.excursion_counts() > 1000.0, "{joint:?}: {settle:?}");
    }
}
