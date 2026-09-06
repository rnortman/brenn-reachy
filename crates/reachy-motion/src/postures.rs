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
}
