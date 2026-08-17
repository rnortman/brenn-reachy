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

/// The neutral configuration: head square and level at nominal height, body
/// square, antennas upright.
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
        antennas: [0.0, 0.0],
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
    use super::{neutral_targets, stow_pose_targets};
    use crate::disarm::STOW_ANTENNAS;
    use reachy_kin::{neutral_head_pose, stow_head_pose};

    #[test]
    fn the_postures_are_the_poses_they_are_derived_from() {
        assert_eq!(neutral_targets().head_pose_body, neutral_head_pose());
        assert_eq!(neutral_targets().antennas, [0.0, 0.0]);
        assert_eq!(stow_pose_targets().head_pose_body, stow_head_pose());
        assert_eq!(stow_pose_targets().antennas, STOW_ANTENNAS);
        assert_eq!(neutral_targets().body_yaw, 0.0);
        assert_eq!(stow_pose_targets().body_yaw, 0.0);
    }
}
