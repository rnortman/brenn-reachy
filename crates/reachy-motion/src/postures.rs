//! The two base postures, as the command a tick path takes.
//!
//! A base posture is a whole configuration -- a head pose, the body yaw and both
//! antenna angles -- and it is stated here so that every host commanding one
//! states the same thing. The bench and the control-rate cog both send the
//! machine to stow, stow is the posture the minimum risk condition names, and
//! two hosts each composing their own would be free to disagree about where it
//! is.
//!
//! Both are derived from `reachy-kin`'s poses and this crate's stow antenna
//! angles rather than transcribed, so a retuned pose moves every caller at once.

use reachy_kin::{neutral_head_pose, stow_head_pose};

use crate::disarm::STOW_ANTENNAS;
use crate::joints::JointTargets;

/// Where an antenna rests when the machine is up, radians: right, then left.
///
/// Ten degrees off vertical toward each side's outboard, not straight up. At
/// vertical the gearbox backlash leaves the rod balanced on the play with no
/// load taking it up, and the position loop hunts across the gap; a small lean
/// lets the rod's own weight hold the play to one side, and the loop has
/// something to push against. Small enough that the pair still rests well
/// inside the contact band, so the rest pose changes no phase judgement.
///
/// The lean alone was not enough: at 500 / 0 / 100 the left antenna still hunts
/// at this pose, 9 counts at 12.7 Hz. At the vendor's 200 / 0 / 0 it is quiet
/// here, and the two together are what hold the joint inside the two-count
/// bound.
///
/// [`STOW_ANTENNAS`] leans by the same magnitude for the same reason: straight
/// down loads no side of the play either.
pub const NEUTRAL_ANTENNAS: [f64; 2] = [-0.1745, 0.1745];

/// The neutral configuration: head square and level at nominal height, body
/// square, antennas near upright.
///
/// This is what a lift commands. It is the whole configuration and not just the
/// head pose, because the pose the machine is lifted *from* is stow, which folds
/// the antennas back -- a lift that left them folded would raise a head that is
/// not up.
#[must_use]
pub fn neutral_targets() -> JointTargets {
    JointTargets {
        head_pose_body: neutral_head_pose(),
        body_yaw: 0.0,
        antennas: NEUTRAL_ANTENNAS,
    }
}

/// The stow configuration: the head lowered and pitched forward, the body
/// square, the antennas folded back.
///
/// The same pose the disarm sequence measures against, expressed as the
/// Cartesian command the tick path takes. Disarming compares joint angles and
/// this commands a head pose; both are derived from the one stow pose, so the
/// motion and the check cannot describe different places.
#[must_use]
pub fn stow_pose_targets() -> JointTargets {
    JointTargets {
        head_pose_body: stow_head_pose(),
        body_yaw: 0.0,
        antennas: STOW_ANTENNAS,
    }
}

#[cfg(test)]
mod tests {
    use super::{NEUTRAL_ANTENNAS, neutral_targets, stow_pose_targets};
    use crate::disarm::STOW_ANTENNAS;
    use crate::phase::ANTENNA_CONTACT_BAND_RAD;
    use crate::tick::ANTENNA_OUTBOARD;
    use reachy_kin::{neutral_head_pose, stow_head_pose};

    #[test]
    fn the_postures_are_the_poses_they_are_derived_from() {
        assert_eq!(neutral_targets().head_pose_body, neutral_head_pose());
        assert_eq!(neutral_targets().antennas, NEUTRAL_ANTENNAS);
        assert_eq!(stow_pose_targets().head_pose_body, stow_head_pose());
        assert_eq!(stow_pose_targets().antennas, STOW_ANTENNAS);
        assert_eq!(neutral_targets().body_yaw, 0.0);
        assert_eq!(stow_pose_targets().body_yaw, 0.0);
    }

    /// The rest lean has to be toward each side's own outboard -- a lean the
    /// wrong way is a lean into the other antenna's arc -- and small enough
    /// that a pair at rest is still inside the contact band, so that resting
    /// off vertical changes no phase judgement of a sweep.
    #[test]
    fn each_antenna_rests_a_little_way_toward_its_own_outboard() {
        for (side, rest) in NEUTRAL_ANTENNAS.into_iter().enumerate() {
            assert_eq!(
                rest.signum(),
                ANTENNA_OUTBOARD[side].signum(),
                "antenna {side} rests at {rest} rad and its outboard is {}: the rest lean leans \
                 the wrong way",
                ANTENNA_OUTBOARD[side],
            );
            assert!(
                rest.abs() < ANTENNA_CONTACT_BAND_RAD,
                "antenna {side} rests {} rad off vertical and the contact band reaches \
                 {ANTENNA_CONTACT_BAND_RAD} rad: a rest outside the band moves where a sweep is \
                 judged",
                rest.abs(),
            );
        }
    }

    /// The fold leans the *other* way — inboard — by the same magnitude, and
    /// the two poses' rules are written out together because a reader who finds
    /// only the rest rule above will read the fold as breaking it.
    ///
    /// Why the fold is exempt. The rest rule is about the arc a rod sweeps near
    /// upright, where the two arcs face each other and a lean the wrong way is a
    /// lean into the other antenna's. At the fold each rod points down, past the
    /// vertical by this lean, and the pair stands twice the lean apart with the
    /// arcs pointing away from each other. What the lean buys is the same thing
    /// at both poses: gravity taking up one side of the gearbox backlash instead
    /// of leaving the rod balanced on the play, which is why the magnitudes are
    /// the same figure and not two.
    ///
    /// What this case does *not* assert is clearance. No check in this workspace
    /// bounds the linkage or the antenna pair against itself
    /// (`TODO(collision-envelope)`), so the separation at the fold is a measured
    /// reading and not a derived one — [`STOW_ANTENNAS`] carries the runs. A
    /// mechanical figure arriving later belongs here, against this separation.
    #[test]
    fn each_antenna_folds_a_little_way_past_down_toward_the_centreline() {
        // Transcribed from a measurement, so written out here: the six
        // leaned-fold probe runs held *this* angle, and the rules below pass
        // over a degree of angles either side of it. A fold edited off the one
        // the runs were made at leaves the reading beside the constant
        // describing a pose the machine no longer holds, so the digit is pinned
        // beside them.
        assert_eq!(STOW_ANTENNAS, [-3.32, 3.32]);
        for (side, fold) in STOW_ANTENNAS.into_iter().enumerate() {
            assert_eq!(
                fold.signum(),
                ANTENNA_OUTBOARD[side].signum(),
                "antenna {side} folds to {fold} rad and its outboard is {}: the fold is measured \
                 the way the arc runs, up through outboard to down",
                ANTENNA_OUTBOARD[side],
            );
            assert!(
                fold.abs() > core::f64::consts::PI,
                "antenna {side} folds to {} rad, short of the {} rad that is straight down: the \
                 fold leans outboard, which is the vendor's shutdown angle and the way that \
                 splays the pair",
                fold.abs(),
                core::f64::consts::PI,
            );
            // Within a degree and not equal to the digit: the fold is rounded
            // to 3.32 rad so that the sweep between the two poses is not a tie
            // between the two half turns for the arc policy to break.
            let lean = fold.abs() - core::f64::consts::PI;
            assert!(
                (lean - NEUTRAL_ANTENNAS[side].abs()).abs() < 1.0_f64.to_radians(),
                "antenna {side} folds {lean} rad past down and rests {} rad off upright: the two \
                 leans answer the same backlash and are the same figure to a degree",
                NEUTRAL_ANTENNAS[side].abs(),
            );
        }
    }
}
