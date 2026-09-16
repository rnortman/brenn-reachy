//! The pose library as the running system holds it: a screened view of the
//! configuration message, and the stow solved once.
//!
//! A [`PoseLibrary`] borrows the message it was screened from and copies
//! nothing out of it: a host whose memory between executions is a fixed-layout
//! slot has nowhere to put a `Vec`.
//!
//! **Where folded is has one solver.** The disarm sequence judges arrival at
//! stow per joint, and the simulated plant starts there; both used to reach
//! that pose by their own inverse-kinematics call, against a geometry each
//! named for itself. Here [`PoseLibrary::stow_joints`] is the one call that
//! solves it, against
//! [`default_motion_config`](reachy_motion::tick::default_motion_config)'s
//! geometry, and the answer is deterministic for a given library. Two records
//! of where the machine rests is the
//! failure this arrangement exists to make impossible.
//!
//! Nothing here is a safety gate. A pose this hands out is still screened by
//! the tick's envelope check and step bound on the tick that commands it.

use core::time::Duration;

use brenn_reachy__cogs__config_clk_rs::{PoseConfig, PoseLibraryConfig};
use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use reachy_kin::{LegAngles, inverse_kinematics, neutral_head_pose};
use reachy_motion::joints::{JointTargets, JointVector};
use reachy_motion::tick::default_motion_config;

use crate::config::LibraryError;

/// Every pose the machine can be sent to, screened, with the stow in joint
/// space beside it.
///
/// Cheap to hold and to copy: it borrows the same message every read comes out
/// of, and carries one solved vector.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PoseLibrary<'a> {
    library: &'a PoseLibraryConfig,
}

impl<'a> PoseLibrary<'a> {
    /// The screened library. Reached through
    /// [`screen`](crate::config::screen), which is what establishes that every
    /// read below is answerable.
    pub(crate) fn new(library: &'a PoseLibraryConfig) -> Self {
        Self { library }
    }

    /// How many poses the library holds.
    #[must_use]
    pub fn len(&self) -> usize {
        self.library.poses.len()
    }

    /// Whether the library holds no poses at all.
    ///
    /// Never true of a screened library — the stow index has to name a pose —
    /// and stated because a length without one reads as an omission.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.len() == 0
    }

    /// Where pose `id` puts the machine, and the pace of a move to it when the
    /// command states none.
    ///
    /// `None` for an id past the library: a pose id arrives from a schedule
    /// slot, which is a number a publisher chose, and a number nothing holds is
    /// the caller's to refuse rather than this type's to substitute for.
    #[must_use]
    pub fn targets(&self, id: u16) -> Option<(JointTargets, Duration)> {
        let pose = self.library.poses.get(usize::from(id))?;
        Some((targets_of(pose), pace_of(pose)))
    }

    /// The stow: its id, where it puts the machine, and the machine's own pace
    /// to it.
    ///
    /// The one record every consumer that stows reads, so nothing that stows
    /// has to know a number.
    #[must_use]
    pub fn stow(&self) -> (u16, JointTargets, Duration) {
        let id = self.library.stow;
        let pose = self
            .library
            .poses
            .get(usize::from(id))
            .expect("the screen refuses a library whose stow index names no pose");
        (id, targets_of(pose), pace_of(pose))
    }

    /// The stow in joint space: body yaw, the six crank angles, the two
    /// antennas.
    ///
    /// What the disarm sequence judges folded against and what the simulated
    /// plant starts at. One solver and one geometry —
    /// [`default_motion_config`](reachy_motion::tick::default_motion_config)'s,
    /// which is the geometry the tick plans against — so the two consumers
    /// cannot put the machine's rest position in two places.
    ///
    /// Solved on the call: deterministic for a given library, so callers that
    /// cache the result hold the same answer the call would give again. The
    /// screen does not solve it, so callers that never read it pay nothing.
    ///
    /// # Errors
    ///
    /// [`LibraryError::StowUnsolvable`] for a stow the linkage cannot reach.
    /// The generator's own envelope check solved it before the library was
    /// written, so a refusal here is a payload nobody built, and the answer to
    /// one is a process that never arms.
    pub fn stow_joints(&self) -> Result<JointVector, LibraryError> {
        let folded = self.stow().1;
        let mut legs = LegAngles([0.0; 6]);
        inverse_kinematics(
            &default_motion_config().geom,
            &folded.head_pose_body,
            &mut legs,
        )
        .map_err(|source| LibraryError::StowUnsolvable { source })?;
        Ok(JointVector {
            body_yaw: folded.body_yaw,
            legs: legs.0,
            antennas: folded.antennas,
        })
    }
}

/// Where a configured pose puts the machine, absolute.
///
/// The message states the head relative to the neutral head pose, which is what
/// a document states and what a recording measures; the mover is handed the
/// absolute pose.
pub(crate) fn targets_of(pose: &PoseConfig) -> JointTargets {
    JointTargets {
        head_pose_body: neutral_head_pose() * relative_of(pose),
        body_yaw: pose.body_yaw,
        antennas: [pose.antenna_right, pose.antenna_left],
    }
}

/// A configured pose's head transform relative to the neutral head pose.
pub(crate) fn relative_of(pose: &PoseConfig) -> Isometry3<f64> {
    Isometry3::from_parts(
        Translation3::new(pose.dx, pose.dy, pose.dz),
        UnitQuaternion::from_quaternion(Quaternion::new(pose.qw, pose.qx, pose.qy, pose.qz)),
    )
}

/// A configured pose's own pace. Positive, because the screen refuses a library
/// where it is not.
fn pace_of(pose: &PoseConfig) -> Duration {
    Duration::from_nanos(pose.duration_ns.unsigned_abs())
}
