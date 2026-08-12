//! The antennas' tip crossing: when a sweeping pair can meet, and how far apart
//! in phase they are when they get there.
//!
//! Each antenna is a rod on a rotor of its own, mounted a little either side of
//! the skull's centre line and sweeping in its own plane. Angles are the joint
//! frame the rest of the crate speaks: zero is straight up, and the two sides'
//! inboard directions carry opposite signs, so a pair standing mirror-symmetric
//! about the centre line is `right + left = 0`. Two tips travelling inboard
//! cross where the arcs meet, which on this machine is a mirrored pair at about
//! 54° either side of vertical: every stall on record sat between 52° and 56°.
//! Arrive there together and the tips meet instead of passing, which stalls both
//! servos against each other until an overload latches. It happened about one
//! inboard sweep in three on a pair running clocks of 0.8 s and 0.7 s, and never
//! on the pair running 0.7 s and 0.3 s.
//!
//! What separates the tips is the pair being *out of phase* when they reach the
//! crossing, and what a caller controls is the two clocks. So the measurement
//! here is one number — how far from mirrored the pair stands when the second of
//! them reaches the band around the crossing — taken over a sampled path, and
//! the same number can be taken over a recorded run. Nothing in this module
//! decides anything; [`tick::floor_move_clock`](crate::tick::floor_move_clock)
//! is what holds a commanded pair to it.

use core::time::Duration;

use reachy_kin::wrap_to_pi;

use crate::joints::JointId;

/// The default contact band, radians: how near straight up an antenna has to be
/// for the two tips to be able to meet at all — see
/// [`AntennaPhaseConfig::contact_band_rad`], which it is the default for.
///
/// Sixty degrees either side of vertical: the crossing itself, with a margin
/// around the four degrees of spread the recorded stalls covered. An antenna
/// further out than this is clear of the other one's arc whatever the other one
/// is doing, so a pair is worth judging only over the stretch where both are
/// inside.
pub const ANTENNA_CONTACT_BAND_RAD: f64 = core::f64::consts::PI / 3.0;

/// The default separation, radians: how far from mirrored a pair carrying both
/// tips across the band's edge has to stand — see
/// [`AntennaPhaseConfig::separation_rad`], which it is the default for.
///
/// A bound on the plan and not on the machine: 0.60 rad is 34° of commanded
/// angle, measured against the two recorded pairs. The clocks that clashed,
/// 0.8 s against 0.7 s, plan an offset that never exceeds 0.44 rad anywhere on
/// the sweep, so nothing phased like that can pass; the clocks that swept clean,
/// 0.7 s against 0.3 s, plan 0.88 rad at the edge and pass with room. It sits
/// 1.35× above the first and 0.68× of the second, and both ends are asserted
/// against the recorded runs.
///
/// The tips' own separation is smaller than the plan's — the servos lag, and a
/// pair crossing early in a sweep lags nearly together — which is why the figure
/// is drawn from what distinguishes the recorded plans rather than from the
/// 0.30 rad the tips themselves stood apart on the clean run.
pub const ANTENNA_PHASE_SEPARATION_RAD: f64 = 0.60;

/// The tip crossing's two numbers: where the tips can meet, and how far apart
/// in phase a pair has to be when it gets there.
///
/// Both are geometry of one machine — the antennas' length and where their
/// rotors sit — so both are configuration with a measured default rather than a
/// figure welded into the library. A second unit, a different rod, or a
/// re-measurement moves them in the operator's file.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AntennaPhaseConfig {
    /// How near straight up an antenna has to be for the two tips to be able to
    /// meet at all, radians. Defaults to [`ANTENNA_CONTACT_BAND_RAD`].
    pub contact_band_rad: f64,
    /// How far from mirrored a pair carrying both tips across that band's edge
    /// has to stand, radians. It bounds the plan rather than the tips, and
    /// [`ANTENNA_PHASE_SEPARATION_RAD`] — its default — carries the
    /// measurement behind the figure.
    pub separation_rad: f64,
}

impl Default for AntennaPhaseConfig {
    /// The one machine this stack has been measured on.
    fn default() -> Self {
        Self {
            contact_band_rad: ANTENNA_CONTACT_BAND_RAD,
            separation_rad: ANTENNA_PHASE_SEPARATION_RAD,
        }
    }
}

/// Whether an antenna standing at `angle` is inside a contact band `band` wide
/// either side of vertical.
#[must_use]
fn inside_band(angle: f64, band: f64) -> bool {
    wrap_to_pi(angle).abs() <= band
}

/// How far a pair standing at `right` and `left` is from mirror-symmetric,
/// radians.
///
/// Zero is the configuration the tips share a point in. Taken modulo a turn,
/// because an antenna angle carries whichever turn of the frame the last
/// command left it in and the geometry does not.
#[must_use]
pub fn mirror_offset(right: f64, left: f64) -> f64 {
    wrap_to_pi(right + left).abs()
}

/// How far a pair stood from mirrored when the second of them crossed the
/// contact band's edge.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhaseSeparation {
    /// The angle between the pair and the mirrored configuration, radians.
    /// Zero is two tips heading for the same point at the same moment.
    pub offset: f64,
    /// The side that reached the edge second — the one a de-phasing stretch
    /// delays further.
    pub later: JointId,
    /// When it got there, on the move's own clock.
    pub at: Duration,
    /// How fast the other side was travelling then, radians per second. What a
    /// delay to `later` buys: the leader covers this much more ground per second
    /// of it, and a leader that has stopped is one no delay separates.
    pub leader_rate: f64,
}

impl PhaseSeparation {
    /// Whether the pair stands `separation_rad` or more from mirrored.
    ///
    /// The policy is the caller's — this type is what the pair came to, and
    /// [`AntennaPhaseConfig::separation_rad`] is what says whether that is
    /// enough.
    #[must_use]
    pub fn met(&self, separation_rad: f64) -> bool {
        self.offset >= separation_rad
    }
}

/// The phase measurement, carried across the samples of one path.
///
/// Fed one period at a time by [`PhaseWatch::look`], in order, with the two
/// antenna angles that period holds — sampled from a plan or read out of a
/// recorded run, which is the point: a pair is judged by the same arithmetic
/// whether it is being planned or being replayed.
///
/// Each side is watched for its **first** crossing of the band's edge, in either
/// direction. A shaped move is monotone in each joint, so a side crosses at most
/// once; inbound is a raise going up over the head, outbound is the stow coming
/// back down, and the tips can meet on either. A side that starts inside the
/// band and stays there, or never comes near it, never crosses, and a pair where
/// one of them never crosses is a pair this says nothing about — there is no
/// second arrival to be late for.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PhaseWatch {
    /// How near vertical a tip has to be to be inside the band, radians.
    band: f64,
    /// The previous period's time and angles, or `None` before the first.
    previous: Option<(Duration, [f64; 2])>,
    /// Which sides have crossed the edge already.
    crossed: [bool; 2],
    /// The verdict, taken on the period the second side crossed.
    separation: Option<PhaseSeparation>,
}

impl PhaseWatch {
    /// A watch that has seen nothing, measuring against a contact band
    /// `contact_band_rad` either side of vertical.
    #[must_use]
    pub fn new(contact_band_rad: f64) -> Self {
        Self {
            band: contact_band_rad,
            previous: None,
            crossed: [false; 2],
            separation: None,
        }
    }

    /// Take in one period: the pair at `antennas`, right then left, `at` this
    /// far into the path.
    ///
    /// The first period is the path's own start and can carry no crossing —
    /// where the machine already stands is a state and not an arrival.
    pub fn look(&mut self, at: Duration, antennas: [f64; 2]) {
        let Some((previous_at, previous)) = self.previous else {
            self.previous = Some((at, antennas));
            return;
        };
        let crossing = [
            !self.crossed[0]
                && inside_band(antennas[0], self.band) != inside_band(previous[0], self.band),
            !self.crossed[1]
                && inside_band(antennas[1], self.band) != inside_band(previous[1], self.band),
        ];
        // Which side this period is the *second* crossing of, if either. A
        // period carrying both crossings is a pair in phase; the right antenna
        // is taken as the later of the two there, so the side a de-phasing
        // stretch lands on is the same one every time.
        let later = if crossing[0] && (crossing[1] || self.crossed[1]) {
            Some(0)
        } else if crossing[1] && self.crossed[0] {
            Some(1)
        } else {
            None
        };
        if let Some(side) = later
            && self.separation.is_none()
        {
            let leader = 1 - side;
            let elapsed = at.saturating_sub(previous_at).as_secs_f64();
            let leader_rate = if elapsed > 0.0 {
                ((antennas[leader] - previous[leader]) / elapsed).abs()
            } else {
                0.0
            };
            self.separation = Some(PhaseSeparation {
                offset: mirror_offset(antennas[0], antennas[1]),
                later: if side == 0 {
                    JointId::AntennaRight
                } else {
                    JointId::AntennaLeft
                },
                at,
                leader_rate,
            });
        }
        for (crossed, now) in self.crossed.iter_mut().zip(crossing) {
            *crossed |= now;
        }
        self.previous = Some((at, antennas));
    }

    /// What the pair came to, once both sides have crossed the band's edge.
    ///
    /// `None` until then, and `None` forever for a path that carries only one of
    /// them across.
    #[must_use]
    pub fn separation(&self) -> Option<PhaseSeparation> {
        self.separation
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Radians from degrees, for reading the recorded angles as they were
    /// written down.
    fn deg(d: f64) -> f64 {
        d.to_radians()
    }

    /// The shipped geometry, which is what every case here measures against.
    fn shipped() -> AntennaPhaseConfig {
        AntennaPhaseConfig::default()
    }

    /// A watch on the shipped band.
    fn watching() -> PhaseWatch {
        PhaseWatch::new(shipped().contact_band_rad)
    }

    /// One period per twentieth of a second, the tick rate everything here runs
    /// at.
    fn at(period: u32) -> Duration {
        Duration::from_millis(u64::from(period) * 20)
    }

    /// A pair swept in step, mirror-symmetric throughout, is the shape the tips
    /// meet in: both cross the edge on one period and the offset is zero.
    #[test]
    fn a_mirrored_pair_crosses_together_with_nothing_between_them() {
        let mut watch = watching();
        for (period, angle) in [80.0, 70.0, 62.0, 55.0, 40.0].into_iter().enumerate() {
            #[expect(clippy::cast_possible_truncation, reason = "five periods")]
            watch.look(at(period as u32), [deg(angle), deg(-angle)]);
        }
        let separation = watch.separation().expect("both sides crossed");
        assert!(separation.offset < 1e-9, "{separation:?}");
        assert!(!separation.met(shipped().separation_rad));
        // The tie goes to the right antenna, and the rate is the left's.
        assert_eq!(separation.later, JointId::AntennaRight);
        assert_eq!(separation.at, at(3));
        assert!((separation.leader_rate - deg(7.0) / 0.02).abs() < 1e-9);
    }

    /// The staggered pair: the leader is well past the crossing by the time the
    /// follower reaches the edge, and the offset is what that lead is worth.
    #[test]
    fn a_staggered_pair_is_measured_when_the_second_side_arrives() {
        let mut watch = watching();
        let path = [
            (100.0, 90.0),
            (85.0, 55.0),
            (70.0, 20.0),
            (58.0, 5.0),
            (50.0, 0.0),
        ];
        for (period, (right, left)) in path.into_iter().enumerate() {
            #[expect(clippy::cast_possible_truncation, reason = "five periods")]
            watch.look(at(period as u32), [deg(right), deg(-left)]);
        }
        let separation = watch.separation().expect("both sides crossed");
        assert_eq!(separation.later, JointId::AntennaRight);
        assert_eq!(separation.at, at(3));
        assert!(
            (separation.offset - deg(53.0)).abs() < 1e-9,
            "{separation:?}"
        );
        assert!(separation.met(shipped().separation_rad));
    }

    /// A recorded antenna angle runs in whatever turn of the frame the last
    /// command left it in, so the band and the offset are both taken modulo a
    /// turn.
    #[test]
    fn the_frame_is_read_a_turn_at_a_time() {
        let mut watch = watching();
        for (period, angle) in [-280.0, -300.0, -320.0].into_iter().enumerate() {
            #[expect(clippy::cast_possible_truncation, reason = "three periods")]
            watch.look(at(period as u32), [deg(angle), deg(-angle)]);
        }
        // -300° is +60° of physical angle, so the pair crosses the edge on the
        // period that reaches it and stands mirrored there.
        let separation = watch.separation().expect("both sides crossed");
        assert_eq!(separation.at, at(1));
        assert!(separation.offset < 1e-9, "{separation:?}");
    }

    /// A side that never leaves the band is a side that never arrives at it, and
    /// a pair with one of those is a pair nobody can de-phase by waiting.
    #[test]
    fn a_side_parked_in_the_band_leaves_nothing_to_measure() {
        let mut watch = watching();
        for (period, right) in [100.0, 80.0, 55.0, 20.0].into_iter().enumerate() {
            #[expect(clippy::cast_possible_truncation, reason = "four periods")]
            watch.look(at(period as u32), [deg(right), deg(-30.0)]);
        }
        assert_eq!(watch.separation(), None);
    }

    /// Only the first crossing counts. A path that leaves the band and comes
    /// back is not a shaped move, and the verdict stays the one taken where the
    /// tips could first have met.
    #[test]
    fn a_second_crossing_does_not_revise_the_verdict() {
        let mut watch = watching();
        let path = [(70.0, 70.0), (55.0, 55.0), (70.0, 20.0), (55.0, 5.0)];
        for (period, (right, left)) in path.into_iter().enumerate() {
            #[expect(clippy::cast_possible_truncation, reason = "four periods")]
            watch.look(at(period as u32), [deg(right), deg(-left)]);
        }
        let separation = watch.separation().expect("both sides crossed");
        assert_eq!(separation.at, at(1));
        assert!(separation.offset < 1e-9, "{separation:?}");
    }

    /// The band is 60° either side of vertical, and the recorded stalls sat
    /// inside it.
    #[test]
    fn the_recorded_stalls_sit_inside_the_band() {
        let band = shipped().contact_band_rad;
        assert!((band - deg(60.0)).abs() < 1e-12);
        for stall in [52.03, 56.51] {
            assert!(inside_band(deg(stall), band), "{stall}");
            assert!(inside_band(deg(-stall), band), "{stall}");
        }
        assert!(!inside_band(deg(61.0), band));
    }
}
