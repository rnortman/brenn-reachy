//! The servo's own trajectory generator, modelled.
//!
//! Every servo on this machine runs a velocity-based motion profile. A goal
//! write does not step the shaft: it hands the servo's internal generator a new
//! target, and the generator ramps a trajectory of its own toward that target
//! at a configured acceleration, up to a configured velocity cap. The
//! commissioning sweep writes that pair into all nine servos, so what a healthy
//! joint does between a setpoint and a reading is arithmetic — and this module
//! is that arithmetic.
//!
//! What it is for is judging a joint. A comparison against the *goal* cannot
//! tell a slow servo from an obstructed one, because content faster than the
//! profile leaves every healthy joint far behind its goal; on the recorded clip
//! library the healthy machine runs 1.5-3 rad behind. A comparison against
//! where the generator's own trajectory stands answers the question the goal
//! cannot, and the same residual over the same recordings is about 0.4 rad,
//! 0.4024 rad at the worst.
//!
//! The generator is only half of what a servo does with a goal. Its position
//! loop follows that trajectory, and a proportional loop is a first-order lag:
//! at cruise the shaft stands a fixed number of periods of travel behind the
//! trajectory it is chasing. So a class has three parameters here — the two
//! registers the sweep writes, converted into the caller's control period, and
//! the loop's following lag ([`ClassProfile::following_lag_us`]). At a slow
//! pair that lag is a hundredth of a radian and invisible; at a class's own
//! measured capability it is most of the detector's screen, spent on a healthy
//! machine doing nothing wrong.
//!
//! The measured numbers here are [`RESPONSE_DEAD_SAMPLES`] and the per-class
//! lag, and each carries its measurement in its own comment. A lag is read by
//! scanning it over a recording for the figure that minimises the class's
//! p99.9 residual, is accepted only where two recordings at different pairs
//! agree on it and no kept fixture's worst sample rises under it, and is never
//! adjusted to make a fixture pass. It is measured at a gains triple and a
//! change of gains re-reads it. A class whose recordings disagree carries no
//! lag: zero is a valid figure and is the trapezoid alone.
//!
//! What it does not model: the stiction-and-backlash excursion a real joint
//! shows when a goal turns round through it — travel lost through a reversal,
//! of a fixed radian size rather than a speed-proportional one — a
//! load-dependent steady offset, and count quantisation. Those are what a
//! detector's margin is for. Fitting them would put constants read off one
//! machine on one day into a model of one machine's bad day.
//!
//! [`PlantModel::step`] saturates a *prediction*. Nothing here is written to a
//! bus and nothing here touches a goal: the no-clamp rule is about commanded
//! values, and a limit inside a model of a limiter is the model being right.
//!
//! Sans-I/O like the rest of the crate — a pure step over state the caller
//! owns, so a live tick and an offline pass over a recording step one function.

use core::f64::consts::TAU;

use thiserror::Error;

use crate::joints::{JointGroup, PerGroup};
use crate::stillness::COUNT_RAD;

/// The most periods either of the two walks below steps the model through
/// before it gives up.
///
/// A guard and not a limit anybody plans against: the longest thing this
/// machine asks of a joint is an antenna's stow-to-neutral arc, which is a few
/// hundred periods at the slowest pair the tree ships. A walk that runs past
/// this is a model whose output is not converging on its generator, not a long
/// move, and the debug assertion beside each walk says so.
pub const MAX_TRAVEL_CYCLES: usize = 6_000;

/// Radians per second per least-significant bit of the Profile Velocity
/// register, whose own unit is 0.229 rev/min.
///
/// This crate's statement of it. The motion layer reasons in radians and
/// seconds and speaks no protocol: nothing here encodes a register, so it takes
/// no dependency on the codec that does, and the two figures it needs in
/// physical units are stated here instead. The conversion itself lives at the
/// wire layer, `dxl_proto::conv::PROFILE_VELOCITY_UNIT_RAD_PER_S`, and a bench
/// test — in the crate that depends on both — pins this pair against that one,
/// the way the count-per-turn figure is pinned.
pub const PROFILE_VELOCITY_UNIT_RAD_PER_S: f64 = 0.229 * TAU / 60.0;

/// Radians per second squared per least-significant bit of Profile
/// Acceleration, whose own unit is 214.577 rev/min² — per minute *squared*,
/// hence 3600 rather than 60.
///
/// Restated here for the reason the velocity unit above it is.
pub const PROFILE_ACCELERATION_UNIT_RAD_PER_S2: f64 = 214.577 * TAU / 3600.0;

/// One servo class's commissioned profile: the two registers the sweep writes,
/// and the following lag its position loop answers them with.
///
/// Named fields rather than a run of numbers, because the orders in this tree
/// disagree: the configuration file writes acceleration first and
/// [`PlantModel::from_registers`] takes the velocity first, so a positional
/// triple is a swap that type-checks. A swapped pair commissions a class at a
/// hundredth of its speed.
///
/// The two registers are a *command* — they are written to the servo and its
/// generator changes — and the lag is the *plant*, a property of the loop that
/// follows that generator. Nothing writes the lag anywhere: it only ever
/// changes what a model believes.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ClassProfile {
    /// Profile Acceleration, the ramp the servo's generator runs.
    pub acceleration: u32,
    /// Profile Velocity, the cruise it ramps to.
    pub velocity: u32,
    /// How far the position loop stands behind the trajectory it follows, in
    /// microseconds of that trajectory's own travel.
    ///
    /// Microseconds rather than periods so that the figure does not depend on
    /// the grid it is read on: a lag stated in periods and re-read at another
    /// control period would be a different loop. [`PlantModel::from_registers`]
    /// divides by the caller's period to get the periods-of-travel figure the
    /// model steps with.
    ///
    /// Zero is the trapezoid alone, and is what a class carries until a
    /// reading is accepted for it.
    pub following_lag_us: u32,
}

/// The ceiling a Profile Acceleration may be written at.
///
/// The register's own range and nothing else: the XL330 has no separate
/// acceleration-limit register, so the field's top value is the whole of the
/// bound, and zero at the other end disables the generator's acceleration
/// stage rather than lifting it.
pub const PROFILE_ACCELERATION_MAX: u32 = 32767;

/// The profile registers per class of servo.
///
/// Three pairs and not one because the classes are three different loads on
/// two different XL330 variants: the antennas' Velocity Limit is 1620 register
/// units against the head's 445, so a pair the antennas can hold is a pair the
/// head's servos refuse.
pub type GroupProfiles = PerGroup<ClassProfile>;

/// The profile pairs this deployment ships, in register units.
///
/// Here rather than only in the configuration file because the tests are
/// written against these numbers, and a suite pinning a machine no deployment
/// runs is a suite about nothing. What keeps the two statements together is the
/// scenario harness's parameter check, which reads the file and compares it
/// with this.
///
/// Three pairs across the three classes. The legs and the antennas run their
/// own measured capability, each confirmed at it — the legs on a tour of the
/// whole clip library, the antennas on a tour read offline under the model plus
/// six armed runs at the pair; the body yaw runs the measured velocity cap of
/// the recorded library tour, which is the pair every class started at. The
/// per-class capability the machine has been measured at is
/// `cogs/pose_reading.rs`'s `RECORDED_CAPABILITY_*`, and the body yaw's reading
/// is that pair to within the instrument's own ratio, which is why the class
/// that is not at its capability is still at the pair it ships.
///
/// Two of the three classes carry a following lag, each read by the scan the
/// header describes over the 2026-09-09 tours and accepted on the agreement of
/// two recordings at different pairs. The legs read 1.2 periods of the 20 ms
/// grid at gains `800 / 100 / 300` and the antennas 1.2 and 1.3 at
/// `200 / 0 / 0`, both minima over twice as deep as the acceptance rule asks
/// for; the body yaw has no motor-bound recording to read one on, its scan runs
/// out of grid rather than finding a floor, and it carries the trapezoid alone.
/// A lag belongs to the loop it was read on, so a gains change re-reads it.
pub const SHIPPED_PROFILES: GroupProfiles = GroupProfiles {
    legs: ClassProfile {
        acceleration: 287,
        velocity: 326,
        following_lag_us: 24_000,
    },
    yaw: ClassProfile {
        acceleration: 20,
        velocity: 50,
        following_lag_us: 0,
    },
    antennas: ClassProfile {
        acceleration: 522,
        velocity: 640,
        following_lag_us: 24_000,
    },
};

/// The control period this deployment ships, nanoseconds.
///
/// Every process on the machine is configured for this grid, and the same
/// parameter check pins each of those files to this constant. A model built at
/// one period and stepped at another is wrong in proportion to the ratio, which
/// is why the period is an argument to [`PlantModel::from_registers`] and this
/// is only the figure the tests may assume.
pub const SHIPPED_PERIOD_NS: i64 = 20_000_000;

/// How many samples elapse between a setpoint being held by the driver and the
/// first reading that shows the servo answering it.
///
/// Two, both measured. One is the driver's: its cycle reads the nine present
/// positions before it writes the cycle's goal, so a reading can answer at best
/// the previous cycle's setpoint. The second is the servo's own start, late in
/// the period after the write -- read off every goal step of more than a chase
/// gap written while a joint stood still, over two capability tours at the
/// servos' own limits (75 steps total). Every one of them moves 1-5 counts in
/// the very next period and ramps in the one after, and 38 of 38 second periods
/// on the repeat tour exceed their first.
///
/// A trapezoid stepped two periods after the write fits that travel to within a
/// period's travel; stepped three periods after, it trails the joint by a whole
/// period.
///
/// A crate constant and not a configuration field: it is a property of this
/// hardware and this driver's cycle, measured once, and a deployment that could
/// choose it would be a deployment that could choose to predict wrongly.
///
/// TODO(plant-chase-sequencer): the loop this depth is the storage for -- seed
/// from a reading, step-then-push, walk the periods no sample attended, re-seed
/// past [`MAX_GAP_PERIODS`] or when nothing is held -- is written out five
/// times: the decision tick, the simulated plant, the offline residual walk,
/// the scenario suite's travel walk and the replay suite. They have to step the
/// same arithmetic in the same order for a run judged offline to be the run
/// judged live, and nothing but their comments says so. One sequencer here,
/// with each caller marshalling its own storage in and out, is the statement
/// that would. What the depth of the two rings that hold it is checked against
/// is this constant, in the tests below and in the simulated driver's own.
pub const RESPONSE_DEAD_SAMPLES: usize = 2;

/// The most periods one step of a caller's loop may advance the model through
/// when samples went missing.
///
/// A caller that lost this many periods stepped the model on setpoints it
/// mostly guessed at; past it the prediction is re-seeded from a reading
/// instead, because a gap that long is a read-loss question and not a tracking
/// one. Coupled to the simulated driver's catch-up cap.
pub const MAX_GAP_PERIODS: usize = 8;

/// The servo as configured: the generator's velocity cap and acceleration, and
/// the position loop's lag behind it, all in the caller's control period.
///
/// Radians per period and radians per period squared, not per second. The
/// period is baked in at construction ([`PlantModel::from_registers`]) so that
/// stepping costs no conversion and a mismatch between the grid the model was
/// built for and the grid it is stepped on cannot arise inside the loop.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PlantModel {
    /// The velocity cap, radians per period.
    pub v_max: f64,
    /// The acceleration, radians per period squared.
    pub a_max: f64,
    /// How much of the distance to the generator the loop's output closes each
    /// period: `1 / (1 + λ)` for a lag of `λ` periods, so a zero lag is `1.0`
    /// and the output is the generator itself.
    pub lag_alpha: f64,
}

/// Where the modelled servo stands for one joint: its generator's trajectory,
/// and the shaft following it.
///
/// The model's own state, never the machine's: the machine's position is a
/// reading, and the whole point of holding this is to have something to compare
/// that reading with.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Predicted {
    /// The generator's trajectory position, radians. The trapezoid alone,
    /// which the loop below chases.
    pub generator: f64,
    /// That trajectory's velocity, radians per period, signed.
    pub velocity: f64,
    /// Where the loop following that trajectory stands, radians. This is what
    /// a reading is compared against: it is the model's statement of where a
    /// healthy shaft is, generator and loop together.
    pub position: f64,
}

/// Why a profile could not be modelled.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
pub enum PlantError {
    /// A profile register of zero. On the servo that disables the generator
    /// altogether — a goal becomes an immediate step, at whatever speed the
    /// position loop can manage — so there is no trajectory to model and no
    /// figure to model it with.
    #[error("the {register} profile register is zero, which disables the servo's generator")]
    GeneratorDisabled {
        /// Which of the two registers was zero.
        register: &'static str,
    },
    /// A control period that is not a grid. Nothing is sampled at zero or
    /// negative spacing, and a model built on one would be arithmetic over an
    /// interval that does not exist.
    #[error("a control period of {period_ns} ns is not a grid")]
    NoPeriod {
        /// The period as the caller stated it, nanoseconds.
        period_ns: i64,
    },
}

/// The three classes' generators, as commissioned.
///
/// What a joint is judged against is its own class's model: one model for nine
/// servos would predict the antennas' trajectory from the legs' registers, and
/// a residual measured that way is the difference between two configurations
/// rather than between a machine and its own generator.
pub type GroupPlants = PerGroup<PlantModel>;

/// Why a class's profile could not be modelled: which class, and what about it.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Error)]
#[error("the {}'s profile is no generator: {source}", .group.name())]
pub struct GroupPlantError {
    /// The class whose pair was refused.
    pub group: JointGroup,
    /// What was wrong with it.
    pub source: PlantError,
}

impl GroupPlants {
    /// The three models these three pairs describe, stepped on a grid of
    /// `period_ns`.
    ///
    /// # Errors
    ///
    /// The first class whose pair is not a generator, named. A refusal here is
    /// a process that does not start: a tick judging joints against a
    /// trajectory nothing runs is worse than one that never ticks.
    pub fn from_profiles(
        profiles: &GroupProfiles,
        period_ns: i64,
    ) -> Result<Self, GroupPlantError> {
        profiles.try_map(|group, profile| {
            PlantModel::from_registers(
                profile.velocity,
                profile.acceleration,
                profile.following_lag_us,
                period_ns,
            )
            .map_err(|source| GroupPlantError { group, source })
        })
    }
}

impl Default for GroupPlants {
    /// The shipped profiles at the shipped period.
    ///
    /// For the tests and for the default configuration a test host builds. A
    /// process reads its own registers and its own period instead: this is
    /// what the deployment ships, not what any given machine is running.
    fn default() -> Self {
        Self::from_profiles(&SHIPPED_PROFILES, SHIPPED_PERIOD_NS)
            .expect("the shipped profile pairs and period are three models")
    }
}

impl PlantModel {
    /// The model of a servo commissioned with these two registers and measured
    /// with this following lag, stepped on a grid of `period_ns`.
    ///
    /// The period is the caller's configured one and never a constant here: the
    /// tick passes the grid its samples arrive on, the simulated driver its own
    /// cycle, an offline pass the grid of the log it is reading. It is also
    /// what the lag is divided by, so a lag stated in microseconds means the
    /// same loop whatever grid it is read on.
    ///
    /// A `following_lag_us` of zero is the trapezoid alone and is refused by
    /// nothing: unlike the two registers, a loop with no lag is a model, just
    /// not this machine's.
    ///
    /// # Errors
    ///
    /// [`PlantError::GeneratorDisabled`] for a zero in either register,
    /// [`PlantError::NoPeriod`] for a period that is not positive.
    pub fn from_registers(
        velocity: u32,
        acceleration: u32,
        following_lag_us: u32,
        period_ns: i64,
    ) -> Result<Self, PlantError> {
        if velocity == 0 {
            return Err(PlantError::GeneratorDisabled {
                register: "velocity",
            });
        }
        if acceleration == 0 {
            return Err(PlantError::GeneratorDisabled {
                register: "acceleration",
            });
        }
        if period_ns <= 0 {
            return Err(PlantError::NoPeriod { period_ns });
        }
        let period_s = period_ns as f64 * 1e-9;
        // The lag in periods of travel, which is what the step is written in:
        // the file states microseconds so that the figure survives a change of
        // grid.
        let lag_periods = f64::from(following_lag_us) * 1e-6 / period_s;
        Ok(Self {
            v_max: f64::from(velocity) * PROFILE_VELOCITY_UNIT_RAD_PER_S * period_s,
            a_max: f64::from(acceleration)
                * PROFILE_ACCELERATION_UNIT_RAD_PER_S2
                * period_s
                * period_s,
            lag_alpha: 1.0 / (1.0 + lag_periods),
        })
    }

    /// Advance one joint's generator and the loop following it by one period,
    /// toward `target`.
    ///
    /// The velocity the generator wants is the fastest one that still stops on
    /// the target under its own acceleration, capped at the profile velocity;
    /// it approaches that at the acceleration, and lands exactly on the target
    /// on the period where one step would carry it there or past it.
    ///
    /// A new target does not restart the ramp. The servo's generator re-plans
    /// from wherever its trajectory currently stands on every goal write, which
    /// is why the velocity is carried in `state` rather than recomputed from
    /// the distance: a target moving every period — which is what a streamed
    /// setpoint is — produces a continuous trajectory and not a train of
    /// restarted ramps.
    ///
    /// The loop then closes `lag_alpha` of its distance to wherever the
    /// generator got to, which at cruise leaves it `v_max · λ` behind and is
    /// the whole of what the lag term claims. Once the generator has stopped on
    /// its target and the loop is inside half an encoder count of it, the
    /// output takes the generator's position exactly: a first-order decay never
    /// arrives, and an arrival that is never final is one no caller can wait
    /// for. The snap is under the quantisation of the reading it is compared
    /// with, and with a zero lag it is a no-op.
    ///
    /// The two saturations here are the model of the servo's own two limiters.
    /// Nothing in `state` is ever commanded.
    pub fn step(&self, state: &mut Predicted, target: f64) {
        let d = target - state.generator;
        // The braking curve: at this speed, decelerating at `a_max` from here
        // arrives at the target with zero velocity. Approaching faster would
        // overshoot, which is what makes this and not `v_max` the binding term
        // on the last few periods of a move.
        let v_stop = (2.0 * self.a_max * d.abs()).sqrt();
        let v_want = d.signum() * self.v_max.min(v_stop);
        state.velocity += (v_want - state.velocity).clamp(-self.a_max, self.a_max);
        if state.velocity.abs() >= d.abs() && state.velocity.signum() == d.signum() {
            state.generator = target;
            state.velocity = 0.0;
        } else {
            state.generator += state.velocity;
        }
        state.position += self.lag_alpha * (state.generator - state.position);
        if state.velocity == 0.0
            && state.generator == target
            && (state.generator - state.position).abs() < COUNT_RAD / 2.0
        {
            state.position = state.generator;
        }
    }

    /// How many periods this modelled servo takes to move `distance_rad` from
    /// rest and stand on it, including the response dead time.
    ///
    /// The model's own walk and not a closed form of the profile: the answer
    /// has to be the arrival of the thing a caller is waiting on, and what a
    /// caller waits on is the shaft, which is the generator plus the loop
    /// behind it. A closed form would be the generator's arrival alone, and
    /// would have to grow a second term the day the lag did.
    ///
    /// A test or a scenario states an arrival instant as an expression over
    /// this rather than as an integer somebody nudged until it passed, so the
    /// arithmetic is the reason the number is what it is.
    ///
    /// # Panics
    ///
    /// In debug builds, if the move does not finish inside
    /// [`MAX_TRAVEL_CYCLES`], which for any distance this machine's joints
    /// cover is a model that is not converging rather than a long move. A
    /// release build carries no assertion and answers the bound itself plus the
    /// dead time, which a caller reads as a very long move: every build the
    /// tests, the scenario harness and the bench run is a debug build, and the
    /// assertion is what makes a model that does not converge a failure there.
    #[must_use]
    pub fn travel_cycles(&self, distance_rad: f64) -> usize {
        let d = distance_rad.abs();
        let mut state = Predicted::default();
        let mut periods = 0;
        while state.position != d && periods < MAX_TRAVEL_CYCLES {
            self.step(&mut state, d);
            periods += 1;
        }
        debug_assert!(
            periods < MAX_TRAVEL_CYCLES,
            "a travel of {distance_rad} rad is not a distance this servo crosses in \
             {MAX_TRAVEL_CYCLES} periods"
        );
        periods + RESPONSE_DEAD_SAMPLES
    }

    /// How many periods this generator takes to *pass* `distance_rad` from
    /// rest: the ramp alone, no dead time.
    ///
    /// The other question a from-rest trajectory gets asked, and the one a
    /// recovery is bounded by: not when a joint let go of arrives somewhere but
    /// when it has covered a given distance on its way past. What that bounds
    /// is a run of the tracking window — the first `progress_min_rad` a
    /// released joint regains is what restarts its window, and the joint is
    /// still accelerating when it gets there.
    ///
    /// The dead time is deliberately absent. A release is not a setpoint
    /// change — the setpoint stood still through the hold — so there is
    /// nothing in flight to delay: the shaft moves when the hand lifts and
    /// the same cycle's read shows it. A caller bounding a commanded pass
    /// adds the dead time itself.
    ///
    /// The answer is the first period on which the shaft stands at or past the
    /// distance, walked from the model rather than solved, for the reason
    /// [`Self::travel_cycles`] gives.
    ///
    /// # Panics
    ///
    /// In debug builds, if the distance is not passed inside
    /// [`MAX_TRAVEL_CYCLES`]. A release build answers the bound instead, as
    /// [`Self::travel_cycles`] says.
    #[must_use]
    pub fn pass_cycles(&self, distance_rad: f64) -> usize {
        let d = distance_rad.abs();
        // Aimed well beyond, on purpose: a trajectory aimed *at* the distance
        // brakes onto it, which is the other question.
        let target = 10.0 * d + 10.0;
        let mut state = Predicted::default();
        let mut periods = 0;
        while state.position < d && periods < MAX_TRAVEL_CYCLES {
            self.step(&mut state, target);
            periods += 1;
        }
        debug_assert!(
            periods < MAX_TRAVEL_CYCLES,
            "a distance of {distance_rad} rad is not one this servo passes in \
             {MAX_TRAVEL_CYCLES} periods"
        );
        periods
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::joints::{ROWS, group_of};

    /// A profile with no lag, its two registers in the order the configuration
    /// file writes them.
    ///
    /// The cases below distinguish the three classes by giving each a pair of
    /// its own, and the field names spelled out per class would bury the
    /// numbers the case is about. The lag cases state their own profiles.
    fn pair(acceleration: u32, velocity: u32) -> ClassProfile {
        ClassProfile {
            acceleration,
            velocity,
            following_lag_us: 0,
        }
    }

    /// The same at a stated following lag, microseconds.
    fn lagged(acceleration: u32, velocity: u32, following_lag_us: u32) -> ClassProfile {
        ClassProfile {
            acceleration,
            velocity,
            following_lag_us,
        }
    }

    /// The shipped pair the cases pin figures at, with a following lag of
    /// `lag_us` microseconds on top of it.
    fn shipped_with_lag(lag_us: u32) -> PlantModel {
        PlantModel::from_registers(50, 20, lag_us, SHIPPED_PERIOD_NS)
            .expect("the shipped pair is a model at any lag")
    }

    /// The model the body yaw and the antennas run on the shipped machine, and
    /// the pair every figure the cases below pin was read at. The cases are
    /// about the arithmetic of one generator, so they take one; the legs run
    /// their own faster pair, and the class-by-class plumbing is what
    /// `the_shipped_triple_is_the_legs_capability_and_the_tour_cap_on_the_rest`
    /// is for.
    fn shipped() -> PlantModel {
        GroupPlants::default().yaw
    }

    /// The state schema's setpoint ring is exactly as deep as the dead time it
    /// is the storage for.
    ///
    /// The depth is a schema fact and the walk over it is written against this
    /// constant, so the two can disagree — and a schema deeper than the
    /// constant does so *silently*: the shift and the read both stay inside the
    /// first `RESPONSE_DEAD_SAMPLES` slots, so the ring's real depth is the
    /// constant while the wire format says otherwise, and the walk an armed
    /// detector raises faults from is judging at a dead time nobody stated.
    #[test]
    fn the_state_schemas_ring_is_as_deep_as_the_dead_time() {
        let mut snap = brenn_reachy__motion__tick_state_clk_rs::MotionSnapWire::new();
        assert_eq!(snap.clear_valid().held.len(), RESPONSE_DEAD_SAMPLES);
    }

    /// The shipped model's two figures, to the precision the register units are
    /// stated to.
    #[test]
    fn the_shipped_model_is_the_configured_pair_per_period() {
        let plant = shipped();
        assert!(
            (plant.v_max - 0.023981).abs() < 5e-6,
            "v_max is {}",
            plant.v_max
        );
        assert!(
            (plant.a_max - 0.0029961).abs() < 5e-7,
            "a_max is {}",
            plant.a_max
        );
    }

    /// A step from rest reaches the cap in `⌈v_max / a_max⌉` periods and goes
    /// no faster, however far the target is.
    ///
    /// Nine periods and not eight at the yaw's and antennas' `20 / 50`, which
    /// is the pair this case's model carries. The two registers are
    /// scaled in units that are not commensurate — 0.229 rev/min against
    /// 214.577 rev/min² — so the ratio is 8.004 periods rather than a round
    /// eight, and the eighth period ends a thousandth of the cap short of it.
    #[test]
    fn a_move_from_rest_reaches_the_cap_in_the_ramp_and_never_exceeds_it() {
        let plant = shipped();
        let ramp = (plant.v_max / plant.a_max).ceil() as usize;
        assert_eq!(ramp, 9, "the ramp at the yaw's and antennas' 20 / 50");
        let mut state = Predicted::default();
        for period in 1..=ramp {
            plant.step(&mut state, 10.0);
            assert!(
                state.velocity <= plant.v_max + 1e-15,
                "period {period} ran at {}",
                state.velocity
            );
        }
        assert!(
            (state.velocity - plant.v_max).abs() < 1e-12,
            "the ramp ends at {} rather than the cap",
            state.velocity
        );
        for _ in 0..100 {
            plant.step(&mut state, 10.0);
            assert!(state.velocity <= plant.v_max + 1e-15);
        }
    }

    /// A long move is a trapezoid — ramp up, run at the cap, ramp down — and
    /// its length is the profile's own arithmetic to within a period.
    ///
    /// The continuous-time figure, kept here as the check that the walk is
    /// still walking a trapezoid: a stepped model covers half a period of
    /// velocity more than the integral on each of the two ramps, so it arrives
    /// a period or three early and never late.
    #[test]
    fn a_long_move_takes_the_time_the_profile_implies() {
        let plant = shipped();
        for distance in [1.0, 2.875, 2.0 * TAU] {
            let stepped = plant.travel_cycles(distance) - RESPONSE_DEAD_SAMPLES;
            let closed = (distance / plant.v_max + plant.v_max / plant.a_max).ceil() as usize;
            assert!(
                closed >= stepped && closed - stepped <= 3,
                "{distance} rad: stepped in {stepped} periods, the profile implies {closed}"
            );
        }
    }

    /// A short move never reaches the cap, and stops on the target rather than
    /// past it.
    #[test]
    fn a_short_move_is_a_triangle_that_does_not_overshoot() {
        let plant = shipped();
        let target = 0.05;
        assert!(
            target < plant.v_max * plant.v_max / plant.a_max,
            "the case has to be under the ramp distance to be a triangle"
        );
        let mut state = Predicted::default();
        let mut peak = 0.0_f64;
        let mut furthest = 0.0_f64;
        for _ in 0..200 {
            plant.step(&mut state, target);
            peak = peak.max(state.velocity);
            furthest = furthest.max(state.position);
        }
        assert!(peak < plant.v_max, "it reached {peak} of the cap");
        assert!(
            (state.position - target).abs() < 1e-12,
            "it settled at {}",
            state.position
        );
        assert!(
            furthest <= target + 1e-12,
            "it overshot to {furthest} on the way"
        );
    }

    /// A target that turns round through a moving trajectory decelerates at the
    /// acceleration and never jumps.
    #[test]
    fn a_target_reversal_decelerates_rather_than_stepping() {
        let plant = shipped();
        let mut state = Predicted::default();
        for _ in 0..40 {
            plant.step(&mut state, 10.0);
        }
        assert!(
            (state.velocity - plant.v_max).abs() < 1e-12,
            "it has to be running to reverse"
        );
        let mut previous = state;
        for period in 0..60 {
            plant.step(&mut state, -10.0);
            assert!(
                (state.velocity - previous.velocity).abs() <= plant.a_max + 1e-12,
                "period {period} changed velocity by {}",
                state.velocity - previous.velocity
            );
            assert!(
                (state.position - previous.position).abs() <= plant.v_max + 1e-12,
                "period {period} moved {}",
                state.position - previous.position
            );
            previous = state;
        }
        assert!(
            state.velocity < 0.0,
            "it never turned round: {}",
            state.velocity
        );
    }

    /// A target moved mid-move re-plans from the velocity the trajectory
    /// already has, which is what the servo's generator does.
    ///
    /// A model that restarted its ramp on every new target would step from rest
    /// here, which is a fifth of the travel this asserts.
    #[test]
    fn a_target_update_mid_move_keeps_the_velocity_it_had() {
        let plant = shipped();
        let mut running = Predicted::default();
        for _ in 0..4 {
            plant.step(&mut running, 10.0);
        }
        let carried = running.velocity;
        assert!(carried > 0.0 && carried < plant.v_max, "mid-ramp");
        let mut moved = running;
        plant.step(&mut moved, 20.0);
        assert!(
            (moved.velocity - (carried + plant.a_max)).abs() < 1e-12,
            "a further target still ramps from {carried}, not from rest"
        );
    }

    /// Zero in either register, or a period that is not a grid, is refused.
    #[test]
    fn a_profile_that_is_not_a_generator_is_refused() {
        let period = SHIPPED_PERIOD_NS;
        assert!(matches!(
            PlantModel::from_registers(0, 20, 0, period),
            Err(PlantError::GeneratorDisabled {
                register: "velocity"
            })
        ));
        assert!(matches!(
            PlantModel::from_registers(50, 0, 0, period),
            Err(PlantError::GeneratorDisabled {
                register: "acceleration"
            })
        ));
        for bad in [0, -1, -20_000_000] {
            assert!(
                matches!(
                    PlantModel::from_registers(50, 20, 0, bad),
                    Err(PlantError::NoPeriod { .. })
                ),
                "{bad} ns"
            );
        }
    }

    /// Every bus row reads its own class's profile, and the three classes are
    /// three separate models.
    ///
    /// Table-driven over all nine rows, because the mapping is what a differing
    /// antenna profile would otherwise get silently wrong: a row read as the
    /// legs' would be judged against a generator its servo is not running, or
    /// against a loop its servo does not have. Three distinct lags, for the
    /// second half of that.
    #[test]
    fn every_row_reads_its_own_classs_profile_and_model() {
        let profiles = GroupProfiles {
            legs: lagged(20, 50, 10_000),
            yaw: lagged(30, 60, 0),
            antennas: lagged(40, 70, 30_000),
        };
        let plants = GroupPlants::from_profiles(&profiles, SHIPPED_PERIOD_NS)
            .expect("three pairs and a grid are three models");
        for (row, joint) in ROWS.into_iter().enumerate() {
            let expected = match group_of(joint) {
                Some(JointGroup::BodyYaw) => profiles.yaw,
                Some(JointGroup::Antennas) => profiles.antennas,
                _ => profiles.legs,
            };
            assert_eq!(profiles.for_row(row), expected, "row {row}");
            assert_eq!(profiles.for_joint(joint), expected, "{joint:?}");
            assert_eq!(
                plants.for_row(row),
                PlantModel::from_registers(
                    expected.velocity,
                    expected.acceleration,
                    expected.following_lag_us,
                    SHIPPED_PERIOD_NS
                )
                .expect("the case's pairs are models"),
                "row {row}"
            );
            assert_eq!(plants.for_joint(joint), plants.for_row(row), "{joint:?}");
        }
        assert_eq!(plants.of(JointGroup::Legs), plants.legs);
        assert_eq!(plants.of(JointGroup::BodyYaw), plants.yaw);
        assert_eq!(plants.of(JointGroup::Antennas), plants.antennas);
    }

    /// The shipped triple is three pairs and two loops: the legs' and the
    /// antennas' commissioned capabilities and the pair the body yaw still
    /// runs, with the measured following lag on the two classes a scan has read
    /// one for and zero on the body yaw, which has no motor-bound recording to
    /// read. Per class, because the whole point of the plumbing is that a class
    /// judged against another class's generator is judged against a trajectory
    /// nothing runs, and all three fields, because a lag that went missing
    /// between the file and the model would be a class judged against a loop it
    /// does not have.
    #[test]
    fn the_shipped_triple_is_the_measured_capabilities_and_the_tour_cap_on_the_yaw() {
        assert_eq!(
            SHIPPED_PROFILES.legs,
            ClassProfile {
                acceleration: 287,
                velocity: 326,
                following_lag_us: 24_000,
            }
        );
        assert_eq!(SHIPPED_PROFILES.yaw, pair(20, 50));
        assert_eq!(
            SHIPPED_PROFILES.antennas,
            ClassProfile {
                acceleration: 522,
                velocity: 640,
                following_lag_us: 24_000,
            }
        );
        let plants = GroupPlants::default();
        let legs = PlantModel::from_registers(326, 287, 24_000, SHIPPED_PERIOD_NS)
            .expect("the legs' pair is a model");
        let yaw = PlantModel::from_registers(50, 20, 0, SHIPPED_PERIOD_NS)
            .expect("the tour's cap is a model");
        let antennas = PlantModel::from_registers(640, 522, 24_000, SHIPPED_PERIOD_NS)
            .expect("the antennas' capability under their own loop is a model");
        assert_eq!(plants.legs, legs);
        assert_eq!(plants.yaw, yaw);
        assert_eq!(plants.antennas, antennas);
        assert!(
            legs.v_max > yaw.v_max && legs.a_max > yaw.a_max,
            "the legs' generator is faster than the yaw's"
        );
        assert!(
            antennas.v_max > legs.v_max && antennas.a_max > legs.a_max,
            "the antennas' generator is the fastest of the three, as their motor is"
        );
        // 24 000 us is 1.2 periods of the shipped 20 ms grid, so a cruising
        // shaft stands 1.2 periods of travel behind its generator on the two
        // classes a lag was read for, and on the generator itself on the yaw.
        let lag_alpha = 1.0 / (1.0 + 1.2);
        assert!((plants.legs.lag_alpha - lag_alpha).abs() < 1e-12);
        assert!((plants.antennas.lag_alpha - lag_alpha).abs() < 1e-12);
        assert_eq!(
            plants.yaw.lag_alpha, 1.0,
            "the body yaw has no reading and runs the generator alone"
        );
    }

    /// A zero in any one class is refused, and the refusal names the class.
    #[test]
    fn a_class_whose_generator_is_disabled_is_refused_by_name() {
        for (group, profiles) in [
            (
                JointGroup::Legs,
                GroupProfiles {
                    legs: pair(20, 0),
                    ..SHIPPED_PROFILES
                },
            ),
            (
                JointGroup::BodyYaw,
                GroupProfiles {
                    yaw: pair(0, 50),
                    ..SHIPPED_PROFILES
                },
            ),
            (
                JointGroup::Antennas,
                GroupProfiles {
                    antennas: pair(0, 0),
                    ..SHIPPED_PROFILES
                },
            ),
        ] {
            let refusal = GroupPlants::from_profiles(&profiles, SHIPPED_PERIOD_NS)
                .expect_err("a zero register is no generator");
            assert_eq!(refusal.group, group);
            assert!(
                matches!(refusal.source, PlantError::GeneratorDisabled { .. }),
                "{refusal}"
            );
            assert!(refusal.to_string().contains(group.name()), "{refusal}");
        }
        assert!(matches!(
            GroupPlants::from_profiles(&SHIPPED_PROFILES, 0),
            Err(GroupPlantError {
                source: PlantError::NoPeriod { .. },
                ..
            })
        ));
    }

    /// The bench's pair is the same model at a different setting, which is what
    /// lets one model explain recordings made under either.
    #[test]
    fn the_bench_profile_is_the_same_model_an_order_of_magnitude_up() {
        let bench = PlantModel::from_registers(600, 400, 0, SHIPPED_PERIOD_NS)
            .expect("the bench pair is a model");
        assert!((bench.v_max - 0.28777).abs() < 5e-5, "{}", bench.v_max);
        assert!((bench.a_max - 0.059921).abs() < 5e-6, "{}", bench.a_max);
    }

    /// `travel_cycles` is the period the walked model really stands on its
    /// target, and it clocks the antenna raise the deterministic scenarios wait
    /// out.
    ///
    /// The figures are the walk's own, transcribed: a case computing them from
    /// the same loop the function runs would assert that the loop is itself.
    #[test]
    fn travel_cycles_is_the_period_the_model_stands_on_the_target() {
        let plant = shipped();
        for (distance, periods) in [(0.01, 5), (0.1, 12), (1.0, 49), (12.56, 531)] {
            assert_eq!(plant.travel_cycles(distance), periods, "{distance} rad");
        }
        // The stow-to-neutral travel of one antenna, which is the longest thing
        // any posture change asks of this machine.
        assert_eq!(plant.travel_cycles(2.875), 128);
        // Sign is not a distance.
        assert_eq!(plant.travel_cycles(-2.875), 128);
        assert_eq!(plant.travel_cycles(0.0), RESPONSE_DEAD_SAMPLES);
    }

    /// A lagged loop takes longer to arrive than its generator does, by the
    /// periods the output spends decaying onto a generator that has stopped.
    ///
    /// The same arc as above at a lag of one and a half periods — the order of
    /// the figures the antennas read at their motor-bound rungs — which is what
    /// makes the settle allowance in `stillness` a function of the lag as well
    /// as of the pair.
    #[test]
    fn a_lagged_arrival_is_the_generators_plus_the_decay() {
        let plain = shipped();
        let lagged = shipped_with_lag(30_000);
        assert_eq!(plain.travel_cycles(2.875), 128);
        assert_eq!(lagged.travel_cycles(2.875), 134);
    }

    /// A loop that never closes on its generator is a broken model and says so,
    /// rather than answering with the bound as though it were a long move.
    ///
    /// The guard is the one thing standing between a profile these walks cannot
    /// resolve and a scenario reading its arrival as six thousand periods of
    /// travel. A lag of an hour on the shipped pair is the shape of it: the
    /// generator arrives on the first few periods and the output crawls after
    /// it for the rest of the day.
    #[test]
    #[should_panic(expected = "is not a distance this servo crosses")]
    fn a_travel_the_model_never_finishes_is_a_model_and_not_a_move() {
        let _ = shipped_with_lag(3_600_000_000).travel_cycles(2.875);
    }

    /// The same guard on the other walk, whose caller is the tracking window's
    /// recovery bound.
    #[test]
    #[should_panic(expected = "is not one this servo passes")]
    fn a_distance_the_model_never_passes_is_a_model_and_not_a_move() {
        let _ = shipped_with_lag(3_600_000_000).pass_cycles(2.875);
    }

    /// `pass_cycles` is the ramp alone, and it is not `travel_cycles`.
    ///
    /// The two answer different questions, and at the progress minimum they are
    /// nearly twice apart: passing is three periods of ramp, arriving is three
    /// of them plus the dead time a commanded arrival takes to be read.
    #[test]
    fn pass_cycles_is_the_ramp_and_not_the_travel() {
        let plant = shipped();
        for (distance, periods) in [(0.01, 3), (0.1, 8), (1.0, 46), (2.875, 124), (12.56, 528)] {
            assert_eq!(plant.pass_cycles(distance), periods, "{distance} rad");
            assert_eq!(
                plant.pass_cycles(-distance),
                periods,
                "{distance} rad, back"
            );
        }
        // The figure the tracking window's recovery bound is stated over: the
        // first `progress_min_rad` a released joint regains, three periods of
        // ramp and nothing else. Being commanded to stop there instead costs
        // the dead time on top.
        assert_eq!(plant.pass_cycles(0.01), 3);
        assert_eq!(plant.travel_cycles(0.01), 5);
        // A loop behind its generator passes the distance later than the
        // generator did.
        assert_eq!(shipped_with_lag(30_000).pass_cycles(0.01), 4);
    }

    /// A zero lag is the trapezoid alone: the output is the generator on every
    /// period of a move, bit for bit, so the model at rest here is the model
    /// this deployment has always run.
    #[test]
    fn a_zero_lag_puts_the_output_on_the_generator_every_period() {
        let plant = shipped();
        assert_eq!(plant.lag_alpha, 1.0);
        let mut state = Predicted::default();
        for target in [1.0, 1.0, -0.5, -0.5, 0.02] {
            for _ in 0..40 {
                plant.step(&mut state, target);
                assert_eq!(
                    state.position, state.generator,
                    "the output left the generator at target {target}"
                );
            }
        }
    }

    /// At cruise a lagged loop settles exactly `v_max · λ` behind its
    /// generator, which is what makes the constant the periods-of-travel figure
    /// the recordings are read in.
    #[test]
    fn a_cruise_settles_a_lag_of_travel_behind_the_generator() {
        for (lag_us, lag_periods) in [(30_000, 1.5), (20_000, 1.0), (9_000, 0.45)] {
            let plant = shipped_with_lag(lag_us);
            let mut state = Predicted::default();
            for _ in 0..400 {
                plant.step(&mut state, 100.0);
            }
            assert!(
                (state.velocity - plant.v_max).abs() < 1e-12,
                "{lag_us} us: the generator has to be cruising to read the lag"
            );
            let behind = state.generator - state.position;
            assert!(
                (behind - plant.v_max * lag_periods).abs() < 1e-9,
                "{lag_us} us: {behind} rad behind, against {} rad of travel",
                plant.v_max * lag_periods
            );
        }
    }

    /// A lag stated in microseconds is the same loop on any grid: the periods
    /// of travel it means are the microseconds divided by the period.
    #[test]
    fn a_lag_in_microseconds_is_periods_of_travel_at_the_grid() {
        let on_20ms = shipped_with_lag(30_000);
        assert!(
            (on_20ms.lag_alpha - 1.0 / 2.5).abs() < 1e-15,
            "30 000 us on the 20 ms grid is 1.5 periods: {}",
            on_20ms.lag_alpha
        );
        let on_10ms = PlantModel::from_registers(50, 20, 30_000, SHIPPED_PERIOD_NS / 2)
            .expect("half the shipped grid is a grid");
        assert!(
            (on_10ms.lag_alpha - 1.0 / 4.0).abs() < 1e-15,
            "the same lag is three periods on a 10 ms grid: {}",
            on_10ms.lag_alpha
        );
    }

    /// A stopped generator's output lands on it exactly rather than decaying
    /// toward it for ever, and it does so inside half an encoder count.
    #[test]
    fn a_stopped_generator_is_arrived_at_and_not_approached_for_ever() {
        let plant = shipped_with_lag(30_000);
        let target = 0.4;
        let mut state = Predicted::default();
        let mut snapped = None;
        for period in 1..1_000 {
            let before = (state.generator - state.position).abs();
            plant.step(&mut state, target);
            if state.position == state.generator && state.velocity == 0.0 && snapped.is_none() {
                // What the decay left before the snap took the rest: the gap
                // shrinks by `1 - lag_alpha` on a generator that has stopped.
                let decayed = before * (1.0 - plant.lag_alpha);
                assert!(
                    decayed < COUNT_RAD / 2.0,
                    "it snapped from {decayed} rad out, which a reading could tell apart"
                );
                snapped = Some(period);
            }
        }
        let period = snapped.expect("the lagged output arrives");
        assert_eq!(state.position, target, "it arrived somewhere else");
        assert_eq!(
            period,
            plant.travel_cycles(target) - RESPONSE_DEAD_SAMPLES,
            "the walk and the step disagree about the arrival"
        );
    }
}
