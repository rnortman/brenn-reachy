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
//! The model has no fitted constants. Its two parameters are the two registers
//! the sweep writes, converted into the caller's control period. The one
//! measured number in the module is [`RESPONSE_DEAD_SAMPLES`], and its
//! measurement is in its own comment.
//!
//! What it does not model: the stiction-and-backlash excursion a real joint
//! shows when a goal turns round through it, a load-dependent steady offset,
//! and count quantisation. Those are what a detector's margin is for. Fitting
//! them would put constants read off one machine on one day into a model whose
//! whole claim is that it has none.
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

/// One servo class's two profile registers, in register units.
///
/// Named fields rather than a pair of numbers, because the two orders in this
/// tree disagree: the configuration file writes acceleration first and
/// [`PlantModel::from_registers`] takes the velocity first, so a positional
/// pair is a swap that type-checks. A swapped pair commissions a class at a
/// hundredth of its speed.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ProfilePair {
    /// Profile Acceleration, the ramp the servo's generator runs.
    pub acceleration: u32,
    /// Profile Velocity, the cruise it ramps to.
    pub velocity: u32,
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
pub type GroupProfiles = PerGroup<ProfilePair>;

/// The profile pairs this deployment ships, in register units.
///
/// Here rather than only in the configuration file because the tests are
/// written against these numbers, and a suite pinning a machine no deployment
/// runs is a suite about nothing. What keeps the two statements together is the
/// scenario harness's parameter check, which reads the file and compares it
/// with this.
///
/// Two pairs across the three classes. The legs run their own measured
/// capability, confirmed on a tour of the whole clip library at it; the body
/// yaw and the antennas run the measured velocity cap of the recorded library
/// tour, which is the pair every class started at. The per-class capability the
/// machine has been measured at is `cogs/pose_reading.rs`'s
/// `RECORDED_CAPABILITY_*`; the body yaw's reading is that pair to within the
/// instrument's own ratio, and the antennas' is a faster pair that is on record
/// and not commissioned, which is what TODO(session-servo-profile) now carries.
pub const SHIPPED_PROFILES: GroupProfiles = GroupProfiles {
    legs: ProfilePair {
        acceleration: 287,
        velocity: 326,
    },
    yaw: ProfilePair {
        acceleration: 20,
        velocity: 50,
    },
    antennas: ProfilePair {
        acceleration: 20,
        velocity: 50,
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

/// The servo's trajectory generator, as configured: the velocity cap it ramps
/// up to and the acceleration it ramps at, both in the caller's control period.
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
}

/// Where the generator's trajectory stands for one joint.
///
/// Position and velocity of the *model*, never of the machine: the machine's
/// position is a reading, and the whole point of holding this is to have
/// something to compare that reading with.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Predicted {
    /// The trajectory's position, radians.
    pub position: f64,
    /// The trajectory's velocity, radians per period, signed.
    pub velocity: f64,
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
        profiles.try_map(|group, pair| {
            PlantModel::from_registers(pair.velocity, pair.acceleration, period_ns)
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
    /// The model of a servo commissioned with these two registers, stepped on a
    /// grid of `period_ns`.
    ///
    /// The period is the caller's configured one and never a constant here: the
    /// tick passes the grid its samples arrive on, the simulated driver its own
    /// cycle, an offline pass the grid of the log it is reading.
    ///
    /// # Errors
    ///
    /// [`PlantError::GeneratorDisabled`] for a zero in either register,
    /// [`PlantError::NoPeriod`] for a period that is not positive.
    pub fn from_registers(
        velocity: u32,
        acceleration: u32,
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
        Ok(Self {
            v_max: f64::from(velocity) * PROFILE_VELOCITY_UNIT_RAD_PER_S * period_s,
            a_max: f64::from(acceleration)
                * PROFILE_ACCELERATION_UNIT_RAD_PER_S2
                * period_s
                * period_s,
        })
    }

    /// Advance one joint's trajectory by one period, toward `target`.
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
    /// The two saturations here are the model of the servo's own two limiters.
    /// Nothing in `state` is ever commanded.
    pub fn step(&self, state: &mut Predicted, target: f64) {
        let d = target - state.position;
        // The braking curve: at this speed, decelerating at `a_max` from here
        // arrives at the target with zero velocity. Approaching faster would
        // overshoot, which is what makes this and not `v_max` the binding term
        // on the last few periods of a move.
        let v_stop = (2.0 * self.a_max * d.abs()).sqrt();
        let v_want = d.signum() * self.v_max.min(v_stop);
        state.velocity += (v_want - state.velocity).clamp(-self.a_max, self.a_max);
        if state.velocity.abs() >= d.abs() && state.velocity.signum() == d.signum() {
            state.position = target;
            state.velocity = 0.0;
        } else {
            state.position += state.velocity;
        }
    }

    /// How many periods this generator takes to move `distance_rad` from rest
    /// and stop, including the response dead time.
    ///
    /// The closed form of the profile rather than a stepped simulation, so that
    /// a test or a scenario can state an arrival instant as an expression over
    /// the travel it is waiting for instead of as an integer somebody nudged
    /// until it passed. Long moves are trapezoids — ramp up, run at the cap,
    /// ramp down — and short ones never reach the cap and are triangles.
    ///
    /// Rounded up, and a period the move ends inside still counts: an arrival
    /// is not observable until the sample that follows it.
    #[must_use]
    pub fn travel_cycles(&self, distance_rad: f64) -> usize {
        let d = distance_rad.abs();
        // The distance a full ramp up and back down covers. Anything shorter
        // turns round before the cap.
        let d_ramp = self.v_max * self.v_max / self.a_max;
        let periods = if d >= d_ramp {
            d / self.v_max + self.v_max / self.a_max
        } else {
            2.0 * (d / self.a_max).sqrt()
        };
        debug_assert!(
            periods.is_finite(),
            "a travel of {distance_rad} rad is not a distance this generator crosses"
        );
        periods.ceil() as usize + RESPONSE_DEAD_SAMPLES
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
    /// Rounded up, so the answer is the first period on which the trajectory
    /// stands at or past the distance. Continued linearly once the cap binds.
    #[must_use]
    pub fn pass_cycles(&self, distance_rad: f64) -> usize {
        let d = distance_rad.abs();
        // Where the ramp ends: the period the cap binds on, and how far the
        // ramp itself covered getting there. One period of the ramp covers
        // `a_max` more than the last, so `k` of them cover `a_max · k(k+1)/2`.
        let ramp = self.v_max / self.a_max;
        let ramped = self.a_max * ramp * (ramp + 1.0) / 2.0;
        let periods = if d <= ramped {
            // The ramp's own quadratic, solved for the period it reaches `d`.
            (-1.0 + (1.0 + 8.0 * d / self.a_max).sqrt()) / 2.0
        } else {
            ramp + (d - ramped) / self.v_max
        };
        debug_assert!(
            periods.is_finite(),
            "a distance of {distance_rad} rad is not one this generator passes"
        );
        periods.ceil() as usize
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::joints::{ROWS, group_of};

    /// A profile pair, in the order the configuration file writes one.
    ///
    /// The cases below distinguish the three classes by giving each a pair of
    /// its own, and the field names spelled out per pair would bury the numbers
    /// the case is about.
    fn pair(acceleration: u32, velocity: u32) -> ProfilePair {
        ProfilePair {
            acceleration,
            velocity,
        }
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

    /// A long move is a trapezoid the closed form bounds from above, within the
    /// [`CLOSED_FORM_SLACK`] the two arithmetics differ by.
    #[test]
    fn a_long_move_takes_the_time_the_closed_form_states() {
        let plant = shipped();
        for distance in [1.0, 2.875, 2.0 * TAU] {
            let stepped = periods_to_arrive(&plant, distance);
            let closed = plant.travel_cycles(distance) - RESPONSE_DEAD_SAMPLES;
            assert!(
                closed >= stepped && closed - stepped <= CLOSED_FORM_SLACK,
                "{distance} rad: stepped in {stepped} periods, closed form says {closed}"
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
            PlantModel::from_registers(0, 20, period),
            Err(PlantError::GeneratorDisabled {
                register: "velocity"
            })
        ));
        assert!(matches!(
            PlantModel::from_registers(50, 0, period),
            Err(PlantError::GeneratorDisabled {
                register: "acceleration"
            })
        ));
        for bad in [0, -1, -20_000_000] {
            assert!(
                matches!(
                    PlantModel::from_registers(50, 20, bad),
                    Err(PlantError::NoPeriod { .. })
                ),
                "{bad} ns"
            );
        }
    }

    /// Every bus row reads its own class's pair, and the three classes are
    /// three separate models.
    ///
    /// Table-driven over all nine rows, because the mapping is what a differing
    /// antenna pair would otherwise get silently wrong: a row read as the legs'
    /// would be judged against a generator its servo is not running.
    #[test]
    fn every_row_reads_its_own_classs_profile_and_model() {
        let profiles = GroupProfiles {
            legs: pair(20, 50),
            yaw: pair(30, 60),
            antennas: pair(40, 70),
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

    /// The shipped triple is two models: the legs' commissioned capability and
    /// the pair the other two classes still run. Per class, because the whole
    /// point of the plumbing is that a class judged against another class's
    /// generator is judged against a trajectory nothing runs.
    #[test]
    fn the_shipped_triple_is_the_legs_capability_and_the_tour_cap_on_the_rest() {
        let plants = GroupPlants::default();
        let legs = PlantModel::from_registers(326, 287, SHIPPED_PERIOD_NS)
            .expect("the legs' pair is a model");
        let rest = PlantModel::from_registers(50, 20, SHIPPED_PERIOD_NS)
            .expect("the tour's cap is a model");
        assert_eq!(plants.legs, legs);
        assert_eq!(plants.yaw, rest);
        assert_eq!(plants.antennas, rest);
        assert!(
            legs.v_max > rest.v_max && legs.a_max > rest.a_max,
            "the legs' generator is the faster of the two"
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
        let bench = PlantModel::from_registers(600, 400, SHIPPED_PERIOD_NS)
            .expect("the bench pair is a model");
        assert!((bench.v_max - 0.28777).abs() < 5e-5, "{}", bench.v_max);
        assert!((bench.a_max - 0.059921).abs() < 5e-6, "{}", bench.a_max);
    }

    /// `travel_cycles` bounds a stepped simulation from above across a triangle
    /// and two trapezoids, and clocks the antenna raise the deterministic
    /// scenarios wait out.
    #[test]
    fn travel_cycles_matches_the_stepped_plant_and_clocks_the_antenna_raise() {
        let plant = shipped();
        for distance in [0.01, 0.1, 1.0, 2.875, 12.56] {
            let stepped = periods_to_arrive(&plant, distance) + RESPONSE_DEAD_SAMPLES;
            let stated = plant.travel_cycles(distance);
            assert!(
                stated >= stepped && stated - stepped <= CLOSED_FORM_SLACK,
                "{distance} rad: stepped in {stepped} periods, stated {stated}"
            );
        }
        // The stow-to-neutral travel of one antenna, which is the longest thing
        // any posture change asks of this machine.
        assert_eq!(plant.travel_cycles(2.875), 130);
        // Sign is not a distance.
        assert_eq!(plant.travel_cycles(-2.875), 130);
    }

    /// How far above the stepped model's own arrival the closed form may sit.
    ///
    /// Measured, not chosen: three periods, over distances from 0.01 rad to
    /// four turns. The closed form is continuous-time and the model steps
    /// forward-Euler, which covers half a period of velocity more than the
    /// integral on each of the two ramps, so the stepped trajectory always
    /// arrives a little early. The direction is what matters and it is the safe
    /// one — an instant derived from `travel_cycles` is never before the joint
    /// got there.
    const CLOSED_FORM_SLACK: usize = 3;

    /// How many periods the model takes to land on a target `distance` away,
    /// starting from rest at zero.
    fn periods_to_arrive(plant: &PlantModel, distance: f64) -> usize {
        let mut state = Predicted::default();
        for period in 1..100_000 {
            plant.step(&mut state, distance);
            if (state.position - distance).abs() < 1e-12 && state.velocity == 0.0 {
                return period;
            }
        }
        panic!("the model never arrived at {distance} rad");
    }

    /// `pass_cycles` is the stepped model's own answer, and it is not
    /// `travel_cycles`.
    ///
    /// Both halves matter. The first is what lets a scenario state a recovery
    /// bound as an expression: the closed form has to be the period the
    /// trajectory really stands past the distance on, not a period either side
    /// of it. The second is why the helper exists at all — the two answer
    /// different questions, and at the progress minimum they are more than
    /// twice apart: passing is three periods of ramp, arriving is four of them
    /// plus the dead time a commanded arrival takes to be read.
    #[test]
    fn pass_cycles_is_the_stepped_ramp_and_not_the_travel() {
        let plant = shipped();
        for distance in [0.001, 0.01, 0.05, 0.1, 0.192, 0.6, 1.0, 2.875, 12.56] {
            let stepped = periods_to_pass(&plant, distance);
            assert_eq!(
                plant.pass_cycles(distance),
                stepped,
                "{distance} rad: the stepped model passes it on period {stepped}"
            );
            assert_eq!(
                plant.pass_cycles(-distance),
                stepped,
                "{distance} rad, back"
            );
        }
        // The figure the tracking window's recovery bound is stated over: the
        // first `progress_min_rad` a released joint regains, three periods of
        // ramp and nothing else. Being commanded to stop there instead costs a
        // fourth period of braking and the dead time on top.
        assert_eq!(plant.pass_cycles(0.01), 3);
        assert_eq!(plant.travel_cycles(0.01), 6);
    }

    /// How many periods the model takes to stand at or past `distance`,
    /// starting from rest at zero and chasing a target well beyond it.
    ///
    /// The target is beyond on purpose: a trajectory aimed *at* the distance
    /// brakes onto it, which is the other helper's question.
    fn periods_to_pass(plant: &PlantModel, distance: f64) -> usize {
        let d = distance.abs();
        let target = 10.0 * d + 10.0;
        let mut state = Predicted::default();
        for period in 1..100_000 {
            plant.step(&mut state, target);
            if state.position >= d - 1e-15 {
                return period;
            }
        }
        panic!("the model never passed {distance} rad");
    }
}
