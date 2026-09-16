//! The emitted pose library, in both directions: loaded documents written into
//! the configuration message, the message read back out of its protobuf text,
//! and the screen every consumer of a bound library runs.
//!
//! The mapping between a [`Pose`] and the message is written once, here, in both
//! directions. A second statement of it would be a second answer able to
//! disagree — the emitter saying one thing about a library and the reader of the
//! same bytes another.
//!
//! **Why the message is read back at all.** `pose_library.textproto` is the
//! asset a cog's configuration dial takes by path, parsed on the C++ side, so a
//! Rust scenario checker cannot see the value a box binds and a checker
//! restating a pose's numbers would assert against its own copy.
//! [`parse_library`] reads the committed asset's bytes against the descriptor
//! generated from the same schema, so a checker compares the run against the
//! file the run was configured from.
//!
//! **What the screen is for.** [`screen`] is the only reader of a bound library,
//! and it establishes what every later read assumes: every number finite, every
//! rotation a unit quaternion, every pace positive, and the stow index naming a
//! pose. A library the generator emitted always passes — the documents were
//! validated and the stow's envelope check already solved it — so a refusal
//! here is a payload nobody built, and the answer to one is a process that
//! never arms. Whether that stow solves through the linkage is the one question
//! left to the consumer that asks for it
//! ([`PoseLibrary::stow_joints`](crate::library::PoseLibrary::stow_joints)),
//! because the answer is a joint vector and the solve is expensive enough that
//! callers who never read it should not pay for it.
//!
//! Nothing here is a safety gate: a pose that screens is still checked by the
//! tick on the tick that commands it.

use brenn_reachy__cogs__config_clk_rs::{PoseConfig, PoseLibraryConfig, PoseLibraryConfigWire};
use nalgebra::Quaternion;
use prost_reflect::{DynamicMessage, ReflectMessage, Value};
use reachy_kin::IkError;
use thiserror::Error;

use crate::format::{Pose, PoseDocError, QUAT_NORM_TOL, count, descriptor, float, int};
use crate::library::PoseLibrary;

/// How many poses the library message holds.
///
/// The schema's own capacity, which a test in this crate holds this number to:
/// a bound stated twice is a bound that can drift, and the edge screens a pose
/// id against its own copy of this one.
pub const MAX_POSES: usize = 32;

/// The document loader's pace ceiling in the unit the message carries.
const MAX_PACE_NS: i64 = crate::format::MAX_DURATION_MS as i64 * 1_000_000;

/// The message a pose library is emitted into, on the heap.
///
/// Larger than a comfortable stack local, and every caller hands out a borrow of
/// it rather than moving it.
pub type Library = Box<PoseLibraryConfigWire>;

/// Why a set of loaded poses cannot be emitted as a library.
///
/// Refusals about the *set*: what one document can be wrong about was refused
/// when it was loaded.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum LibraryWriteError {
    /// More poses than the message holds.
    #[error("library has {poses} poses; the message holds {MAX_POSES}")]
    TooManyPoses {
        /// How many were offered.
        poses: usize,
    },

    /// No pose is named `stow`.
    ///
    /// The stow is the one reserved name: every compiled schedule ends at it,
    /// the fault ladder commands it, and the disarm sequence judges folded
    /// against it. A library without one is a machine with nowhere to rest.
    #[error(
        "library holds no pose named {}; that is where the machine rests",
        crate::STOW_POSE
    )]
    NoStow,

    /// Two poses under one name.
    ///
    /// A name is the join key between a script and the library, so a repeated
    /// one is a script whose meaning depends on which of them the emitter
    /// happened to write first.
    #[error("library holds two poses named {name:?}")]
    DuplicateName {
        /// The name written twice.
        name: String,
    },
}

/// Why a bound library cannot be used.
///
/// Every arm is a fact about the message rather than about a run, and every one
/// of them refuses the whole library: a consumer that took the poses it could
/// read and left the rest would be a machine whose vocabulary depends on which
/// pose ids a run happens to use.
#[derive(Clone, Copy, Debug, Error, PartialEq)]
pub enum LibraryError {
    /// A pose carries a value that is not a finite number.
    #[error("pose {pose_id} field {field} is {value}, which is not a finite number")]
    NotFinite {
        /// Which pose.
        pose_id: usize,
        /// Which field of it.
        field: &'static str,
        /// What it held.
        value: f64,
    },

    /// A pose's head rotation is not a unit quaternion.
    #[error("pose {pose_id} rotation has norm {norm}, further than {QUAT_NORM_TOL} from unit")]
    QuatNotUnit {
        /// Which pose.
        pose_id: usize,
        /// The norm found.
        norm: f64,
    },

    /// A pose states a pace that is not a positive time.
    #[error("pose {pose_id} states a pace of {duration_ns} ns; a move takes a positive time")]
    PaceNotPositive {
        /// Which pose.
        pose_id: usize,
        /// What it held.
        duration_ns: i64,
    },

    /// A pose states a pace longer than a move may take.
    ///
    /// The document loader's ceiling, restated over the message: what a bound
    /// library states is the pace of every move the machine plans for itself,
    /// and a controlled stow that outlasts the ladder's budget is a fault
    /// response whose stow half never arrives.
    #[error(
        "pose {pose_id} states a pace of {duration_ns} ns; a move takes at most {} ms",
        crate::format::MAX_DURATION_MS
    )]
    PaceTooLong {
        /// Which pose.
        pose_id: usize,
        /// What it held.
        duration_ns: i64,
    },

    /// The stow index names no pose.
    #[error("library names pose {stow} as the stow and holds {poses} poses")]
    StowOutOfRange {
        /// What the field held.
        stow: u16,
        /// How many poses there are.
        poses: usize,
    },

    /// The stow pose does not solve through the linkage.
    ///
    /// Where folded is has to be reachable in joint space before anything can
    /// judge arrival at it per joint.
    #[error("the stow pose does not solve through the linkage: {source}")]
    StowUnsolvable {
        /// Which leg could not be placed, and by how much it missed.
        source: IkError,
    },
}

/// One double field of a pose: its key, and the three ways the mapping touches
/// it.
///
/// The mapping is one table, so a channel added to `PoseConfig` is one row here
/// rather than a coordinated edit of the writer, the text reader, the screen's
/// finite walk and the emitter's printer that no compiler pairs up. The same
/// shape `reachy_clips::config::FrameField` gives a clip frame's channels, and
/// for the same reason.
///
/// The pace is not here: it is the one field that is not a double, and the three
/// places it is touched each say something different about it (milliseconds in
/// the document, nanoseconds in the message, a positive bound in the screen).
pub struct PoseField {
    /// The schema's field name, which is also the protobuf text's key.
    pub key: &'static str,
    /// Read it out of a message pose.
    pub get: fn(&PoseConfig) -> f64,
    /// Write it into a message pose.
    pub set: fn(&mut PoseConfig, f64),
    /// Read it out of a loaded pose.
    pub of_pose: fn(&Pose) -> f64,
}

/// Every double field of a pose, in the schema's declared order.
///
/// The order is the order an emitted library states its fields in, which is the
/// order a reader of the text sees them in.
pub const POSE_FIELDS: [PoseField; 10] = [
    PoseField {
        key: "dx",
        get: |pose| pose.dx,
        set: |pose, value| pose.dx = value,
        of_pose: |pose| pose.relative().translation.x,
    },
    PoseField {
        key: "dy",
        get: |pose| pose.dy,
        set: |pose, value| pose.dy = value,
        of_pose: |pose| pose.relative().translation.y,
    },
    PoseField {
        key: "dz",
        get: |pose| pose.dz,
        set: |pose, value| pose.dz = value,
        of_pose: |pose| pose.relative().translation.z,
    },
    PoseField {
        key: "qw",
        get: |pose| pose.qw,
        set: |pose, value| pose.qw = value,
        of_pose: |pose| pose.relative().rotation.quaternion().w,
    },
    PoseField {
        key: "qx",
        get: |pose| pose.qx,
        set: |pose, value| pose.qx = value,
        of_pose: |pose| pose.relative().rotation.quaternion().i,
    },
    PoseField {
        key: "qy",
        get: |pose| pose.qy,
        set: |pose, value| pose.qy = value,
        of_pose: |pose| pose.relative().rotation.quaternion().j,
    },
    PoseField {
        key: "qz",
        get: |pose| pose.qz,
        set: |pose, value| pose.qz = value,
        of_pose: |pose| pose.relative().rotation.quaternion().k,
    },
    PoseField {
        key: "body_yaw",
        get: |pose| pose.body_yaw,
        set: |pose, value| pose.body_yaw = value,
        of_pose: |pose| pose.targets().body_yaw,
    },
    PoseField {
        key: "antenna_right",
        get: |pose| pose.antenna_right,
        set: |pose, value| pose.antenna_right = value,
        of_pose: |pose| pose.targets().antennas[0],
    },
    PoseField {
        key: "antenna_left",
        get: |pose| pose.antenna_left,
        set: |pose, value| pose.antenna_left = value,
        of_pose: |pose| pose.targets().antennas[1],
    },
];

/// The schema's name for the field the table above leaves out.
pub const PACE_FIELD: &str = "duration_ns";

/// Write loaded poses into a library message, in the order given.
///
/// The order is the caller's and becomes the identity of every pose: a pose id
/// is an index into what this writes. The stow index is found by name, so the
/// reserved name is the only thing an emitter has to get right about it.
///
/// # Errors
///
/// [`LibraryWriteError`] for a set of poses that is not a library: too many for
/// the message, no stow, or one name used twice.
pub fn write_library(poses: &[Pose], out: &mut PoseLibraryConfig) -> Result<(), LibraryWriteError> {
    // Everything the set can be refused on is decided before a slot is grown, so
    // a refusal leaves the message empty rather than half-written.
    if poses.len() > MAX_POSES {
        return Err(LibraryWriteError::TooManyPoses { poses: poses.len() });
    }
    for (index, pose) in poses.iter().enumerate() {
        if let Some(earlier) = poses[..index]
            .iter()
            .find(|other| other.name() == pose.name())
        {
            return Err(LibraryWriteError::DuplicateName {
                name: earlier.name().to_owned(),
            });
        }
    }
    let stow = poses
        .iter()
        .position(|pose| pose.name() == crate::STOW_POSE)
        .ok_or(LibraryWriteError::NoStow)?;

    out.poses.clear();
    for pose in poses {
        let slot = out
            .poses
            .try_grow()
            .expect("the pose count is within the message's capacity");
        write_pose(pose, slot);
    }
    out.stow = u16::try_from(stow).expect("a pose index is at most MAX_POSES");
    Ok(())
}

/// One loaded pose written into one message slot.
fn write_pose(pose: &Pose, out: &mut PoseConfig) {
    for field in &POSE_FIELDS {
        (field.set)(out, (field.of_pose)(pose));
    }
    // Milliseconds in the document because a person writes it; nanoseconds here
    // because that is the unit every duration in `cogs/config.clk` carries.
    out.duration_ns = i64::from(pose.duration_ms()) * 1_000_000;
}

/// Read an emitted library back out of its protobuf text.
///
/// Against the same embedded descriptor a document is parsed with, so what this
/// reads is what the committed asset states.
///
/// # Errors
///
/// [`PoseDocError::Text`] for anything the protobuf text parser refuses, and the
/// transcription's own refusals for a message that parsed but does not say what
/// a library says.
pub fn parse_library(text: &str) -> Result<Library, PoseDocError> {
    let message =
        DynamicMessage::parse_text_format(descriptor(LIBRARY_MESSAGE), text).map_err(|error| {
            PoseDocError::Text {
                message: error.to_string(),
            }
        })?;
    let mut out = PoseLibraryConfigWire::new_boxed();
    transcribe_library(&message, out.clear_valid())?;
    Ok(out)
}

/// The message an emitted pose library is one of.
const LIBRARY_MESSAGE: &str = "PoseLibraryConfig";

/// A parsed library written into the configuration message, field by field.
///
/// The loop is over the message's descriptor, as the document reader's is, so a
/// field the schema grows and this function does not carry is a refusal rather
/// than a value silently left at zero.
fn transcribe_library(
    message: &DynamicMessage,
    out: &mut PoseLibraryConfig,
) -> Result<(), PoseDocError> {
    out.poses.clear();
    out.stow = 0;
    for field in message.descriptor().fields() {
        match field.name() {
            "poses" => {
                let value = message.get_field(&field);
                let list = value.as_list().ok_or(PoseDocError::Schema {
                    field: "poses".to_owned(),
                    why: "this reader reads it as an array of poses",
                })?;
                if list.len() > MAX_POSES {
                    return Err(PoseDocError::TooManyPoses { poses: list.len() });
                }
                for entry in list {
                    let pose = match entry {
                        Value::Message(pose) => pose,
                        _ => {
                            return Err(PoseDocError::Schema {
                                field: "poses".to_owned(),
                                why: "this reader reads it as an array of poses",
                            });
                        }
                    };
                    let slot = out
                        .poses
                        .try_grow()
                        .expect("the pose count is within the message's capacity");
                    transcribe_pose(pose, slot)?;
                }
            }
            "stow" => {
                let stow = count(message, &field, "stow")?;
                out.stow = u16::try_from(stow).map_err(|_| PoseDocError::Schema {
                    field: "stow".to_owned(),
                    why: "this reader reads it as a pose index, which is 16 bits",
                })?;
            }
            other => {
                return Err(PoseDocError::Schema {
                    field: other.to_owned(),
                    why: "the schema grew a field this reader does not carry",
                });
            }
        }
    }
    Ok(())
}

/// One parsed pose written into one message slot.
fn transcribe_pose(message: &DynamicMessage, out: &mut PoseConfig) -> Result<(), PoseDocError> {
    for field in message.descriptor().fields() {
        if let Some(double) = POSE_FIELDS.iter().find(|entry| entry.key == field.name()) {
            (double.set)(out, float(message, &field, double.key)?);
        } else if field.name() == PACE_FIELD {
            out.duration_ns = int(message, &field, PACE_FIELD)?;
        } else {
            return Err(PoseDocError::Schema {
                field: field.name().to_owned(),
                why: "the schema grew a field this reader does not carry",
            });
        }
    }
    Ok(())
}

/// Establish every pose's invariants.
///
/// What comes back is the library every consumer reads its poses out of: every
/// number finite, every rotation a unit quaternion, every pace inside the
/// bounds a move is planned on, and the stow index naming a pose.
///
/// Arithmetic over the message's own numbers and nothing else: the IK solve
/// is [`PoseLibrary::stow_joints`]'s, deferred so that screening is cheap
/// enough to run on every execution.
///
/// # Errors
///
/// [`LibraryError`], naming the first thing wrong with the library.
pub fn screen(library: &PoseLibraryConfig) -> Result<PoseLibrary<'_>, LibraryError> {
    for pose_id in 0..library.poses.len() {
        let pose = library
            .poses
            .get(pose_id)
            .expect("an index below the count");
        for field in &POSE_FIELDS {
            let value = (field.get)(pose);
            if !value.is_finite() {
                return Err(LibraryError::NotFinite {
                    pose_id,
                    field: field.key,
                    value,
                });
            }
        }
        let norm = Quaternion::new(pose.qw, pose.qx, pose.qy, pose.qz).norm();
        if (norm - 1.0).abs() > QUAT_NORM_TOL {
            return Err(LibraryError::QuatNotUnit { pose_id, norm });
        }
        if pose.duration_ns <= 0 {
            return Err(LibraryError::PaceNotPositive {
                pose_id,
                duration_ns: pose.duration_ns,
            });
        }
        if pose.duration_ns > MAX_PACE_NS {
            return Err(LibraryError::PaceTooLong {
                pose_id,
                duration_ns: pose.duration_ns,
            });
        }
    }

    let stow = library.stow;
    if usize::from(stow) >= library.poses.len() {
        return Err(LibraryError::StowOutOfRange {
            stow,
            poses: library.poses.len(),
        });
    }
    Ok(PoseLibrary::new(library))
}

#[cfg(test)]
mod tests {
    use super::{
        LibraryError, LibraryWriteError, MAX_POSES, PACE_FIELD, POSE_FIELDS, parse_library, screen,
        write_library,
    };
    use crate::format::{MAX_DESCRIPTION_LEN, Pose, PoseDoc, PoseDocError, descriptor};
    use brenn_reachy__cogs__config_clk_rs::{PoseDocumentWire, PoseLibraryConfigWire};
    use core::time::Duration;
    use reachy_motion::asset_name::MAX_ASSET_NAME_LEN;

    /// A loaded pose under `name`, square and level, at `duration_ms`.
    fn pose(name: &str, duration_ms: u32) -> Pose {
        Pose::from_doc(PoseDoc {
            name: name.to_owned(),
            description: "a fixture".to_owned(),
            dt: [0.0, 0.0, 0.0],
            dq: [1.0, 0.0, 0.0, 0.0],
            body_yaw: 0.0,
            antennas: [-0.1745, 0.1745],
            duration_ms,
        })
        .expect("the fixture loads")
    }

    /// The machine's rest position, as a document states it: the head folded
    /// down and pitched forward, the antennas folded back.
    ///
    /// Derived from the pose the code holds today rather than transcribed,
    /// because a document states the head *relative* to neutral and the
    /// arithmetic is what the extractor performs.
    fn folded() -> Pose {
        let relative = reachy_kin::neutral_head_pose().inverse() * reachy_kin::sleep_head_pose();
        let q = relative.rotation.quaternion();
        Pose::from_doc(PoseDoc {
            name: "stow".to_owned(),
            description: "the fold".to_owned(),
            dt: [
                relative.translation.x,
                relative.translation.y,
                relative.translation.z,
            ],
            dq: [q.w, q.i, q.j, q.k],
            body_yaw: 0.0,
            antennas: [-3.32, 3.32],
            duration_ms: 2000,
        })
        .expect("the fixture loads")
    }

    /// `poses` written into a fresh library message.
    fn written(poses: &[Pose]) -> Box<PoseLibraryConfigWire> {
        let mut message = PoseLibraryConfigWire::new_boxed();
        write_library(poses, message.clear_valid()).expect("the fixture is a library");
        message
    }

    #[test]
    fn a_library_carries_every_pose_and_names_the_stow_by_index() {
        let poses = [pose("neutral", 800), folded(), pose("peek", 650)];
        let message = written(&poses);
        let library = message.validate().expect("a written library validates");
        assert_eq!(library.poses.len(), 3);
        assert_eq!(library.stow, 1);
        assert_eq!(
            library.poses.capacity(),
            MAX_POSES,
            "the constant and the schema disagree about how many poses fit"
        );

        let screened = screen(library).expect("the fixture screens");
        let (id, targets, pace) = screened.stow();
        assert_eq!(id, 1);
        assert_eq!(pace, Duration::from_millis(2000));
        assert_eq!(&targets, folded().targets());
        assert_eq!(
            screened.targets(0).expect("the first pose"),
            (*pose("neutral", 800).targets(), Duration::from_millis(800))
        );
        assert_eq!(screened.len(), 3);
        assert!(!screened.is_empty());
    }

    /// A pose id arrives from a schedule slot, which is a number a publisher
    /// chose; one the library does not hold is the caller's to refuse.
    #[test]
    fn an_id_past_the_library_answers_nothing() {
        let poses = [folded()];
        let message = written(&poses);
        let screened = screen(message.validate().expect("valid")).expect("screens");
        assert!(screened.targets(1).is_none());
        assert!(screened.targets(u16::MAX).is_none());
    }

    /// Where folded is has one solver, and the answer is the pose the library
    /// carries rather than a second record of it.
    #[test]
    fn the_stow_solves_to_the_pose_the_library_carries() {
        let message = written(&[pose("neutral", 800), folded()]);
        let screened = screen(message.validate().expect("valid")).expect("screens");
        let joints = screened.stow_joints().expect("the fold solves");
        let (_, targets, _) = screened.stow();

        let mut legs = reachy_kin::LegAngles([0.0; 6]);
        reachy_kin::inverse_kinematics(
            &reachy_motion::tick::default_motion_config().geom,
            &targets.head_pose_body,
            &mut legs,
        )
        .expect("the fold solves");
        assert_eq!(joints.legs, legs.0);
        assert_eq!(joints.antennas, targets.antennas);
        assert_eq!(joints.body_yaw, targets.body_yaw);
    }

    #[test]
    fn a_set_of_poses_that_is_not_a_library_is_refused() {
        let mut message = PoseLibraryConfigWire::new_boxed();
        assert_eq!(
            write_library(&[pose("neutral", 800)], message.clear_valid()),
            Err(LibraryWriteError::NoStow)
        );
        assert_eq!(
            message.validate().expect("valid").poses.len(),
            0,
            "a refused emit leaves the message empty rather than half-written"
        );

        assert_eq!(
            write_library(
                &[folded(), pose("peek", 1), folded()],
                message.clear_valid()
            ),
            Err(LibraryWriteError::DuplicateName {
                name: "stow".to_owned()
            })
        );

        let crowd: Vec<Pose> = (0..=MAX_POSES)
            .map(|n| pose(&format!("p{n}"), 800))
            .collect();
        assert_eq!(
            write_library(&crowd, message.clear_valid()),
            Err(LibraryWriteError::TooManyPoses {
                poses: MAX_POSES + 1
            })
        );
    }

    /// The screen is what establishes, once, what every later read assumes. A
    /// library the generator emitted always passes; these are the payloads
    /// nobody built.
    #[test]
    fn the_screen_refuses_a_number_no_pose_can_hold() {
        let mut message = written(&[pose("neutral", 800), folded()]);
        message
            .validate_mut()
            .expect("valid")
            .poses
            .get_mut(0)
            .expect("the first pose")
            .dz = f64::NAN;
        match screen(message.validate().expect("valid")) {
            // A NaN is not equal to itself, so the value is asserted as the
            // thing it is rather than against a literal.
            Err(LibraryError::NotFinite {
                pose_id,
                field,
                value,
            }) => {
                assert_eq!((pose_id, field), (0, "dz"));
                assert!(value.is_nan());
            }
            other => panic!("a NaN in a pose is refused as that, not as {other:?}"),
        }

        let mut message = written(&[folded()]);
        message
            .validate_mut()
            .expect("valid")
            .poses
            .get_mut(0)
            .expect("the only pose")
            .qw = 1.5;
        assert!(matches!(
            screen(message.validate().expect("valid")),
            Err(LibraryError::QuatNotUnit { pose_id: 0, .. })
        ));

        let mut message = written(&[folded()]);
        message
            .validate_mut()
            .expect("valid")
            .poses
            .get_mut(0)
            .expect("the only pose")
            .duration_ns = 0;
        assert_eq!(
            screen(message.validate().expect("valid")),
            Err(LibraryError::PaceNotPositive {
                pose_id: 0,
                duration_ns: 0
            })
        );

        let mut message = written(&[folded()]);
        let too_long = super::MAX_PACE_NS + 1;
        message
            .validate_mut()
            .expect("valid")
            .poses
            .get_mut(0)
            .expect("the only pose")
            .duration_ns = too_long;
        assert_eq!(
            screen(message.validate().expect("valid")),
            Err(LibraryError::PaceTooLong {
                pose_id: 0,
                duration_ns: too_long
            })
        );
    }

    /// Nothing that stows has to know a number, which only holds while the
    /// number the library states names a pose that solves.
    #[test]
    fn the_screen_refuses_a_stow_nothing_can_stand_at() {
        let mut message = written(&[folded()]);
        message.validate_mut().expect("valid").poses.clear();
        assert_eq!(
            screen(message.validate().expect("valid")),
            Err(LibraryError::StowOutOfRange { stow: 0, poses: 0 })
        );

        let mut message = written(&[folded(), pose("neutral", 800)]);
        message.validate_mut().expect("valid").stow = 7;
        assert_eq!(
            screen(message.validate().expect("valid")),
            Err(LibraryError::StowOutOfRange { stow: 7, poses: 2 })
        );

        // A head half a metre above neutral is off the linkage's reach: a
        // document stating it would never have loaded, and a message stating it
        // arms no process. The screen admits it -- every number it reads is a
        // number a pose can hold -- and the consumer that asks where folded is
        // in joint space is the one that cannot be answered.
        let mut message = written(&[folded()]);
        message
            .validate_mut()
            .expect("valid")
            .poses
            .get_mut(0)
            .expect("the only pose")
            .dz = 0.5;
        assert!(matches!(
            screen(message.validate().expect("valid"))
                .expect("a library of numbers a pose can hold")
                .stow_joints(),
            Err(LibraryError::StowUnsolvable { .. })
        ));
    }

    /// The scenario checkers read the numbers a consumer loads out of the
    /// committed asset's bytes, so what the emitter wrote and what this reads
    /// back are the same library.
    #[test]
    fn an_emitted_library_reads_back_as_what_was_written() {
        let poses = [pose("neutral", 800), folded(), pose("peek", 650)];
        let written = written(&poses);
        let text = library_text(&poses);
        let parsed = parse_library(&text).expect("the emitted text parses");
        assert_eq!(
            parsed.validate().expect("valid"),
            written.validate().expect("valid")
        );

        let screened = screen(parsed.validate().expect("valid")).expect("screens");
        assert_eq!(screened.stow().0, 1);
        assert_eq!(screened.stow().2, Duration::from_millis(2000));
    }

    /// The transcription walks the descriptor, so a library that says something
    /// the schema does not is refused rather than half-read.
    #[test]
    fn a_library_text_the_schema_does_not_describe_is_refused() {
        let text = library_text(&[folded()]);
        assert!(matches!(
            parse_library(&format!("{text}\nmood: 3")),
            Err(PoseDocError::Text { .. })
        ));
        assert!(matches!(
            parse_library(&text.replace("stow: 0", "stow: \"first\"")),
            Err(PoseDocError::Text { .. })
        ));

        // A pose field the text leaves out has no reading, as in a document.
        let without_yaw: String = text
            .lines()
            .filter(|line| !line.trim_start().starts_with("body_yaw:"))
            .map(|line| format!("{line}\n"))
            .collect();
        assert_eq!(
            parse_library(&without_yaw),
            Err(PoseDocError::MissingField { field: "body_yaw" })
        );

        // The message has a capacity and hand-edited text need not respect it.
        let one = library_text(&[folded()]);
        let poses = one
            .lines()
            .filter(|line| !line.starts_with("stow:"))
            .collect::<Vec<_>>()
            .join("\n");
        let crowd = format!(
            "{}\nstow: 0",
            std::iter::repeat_n(poses.as_str(), MAX_POSES + 1)
                .collect::<Vec<_>>()
                .join("\n")
        );
        assert_eq!(
            parse_library(&crowd),
            Err(PoseDocError::TooManyPoses {
                poses: MAX_POSES + 1
            })
        );
    }

    /// The document's name field is the wire's bound, spelled in the schema. A
    /// name the wire carries and the document cannot hold would be an asset
    /// nobody could author.
    #[test]
    fn the_document_holds_every_name_the_wire_admits() {
        assert_eq!(
            PoseDocumentWire::new_boxed()
                .validate()
                .expect("a cleared document validates")
                .name
                .capacity(),
            MAX_ASSET_NAME_LEN,
            "the schema's cap and the asset-name bound disagree: a lawful name \
             would have nowhere to be written"
        );
    }

    /// The description bound the loader holds a document to is the schema's own
    /// capacity for the field. A document that loads and cannot be written into
    /// the message would be an asset nobody could carry.
    #[test]
    fn the_document_holds_every_description_the_loader_admits() {
        assert_eq!(
            PoseDocumentWire::new_boxed()
                .validate()
                .expect("a cleared document validates")
                .description
                .capacity(),
            MAX_DESCRIPTION_LEN,
            "the schema's cap and the loader's bound disagree: a description the \
             loader accepts would have nowhere to be written"
        );
    }

    /// The field table is the schema's own field list, in the schema's order.
    ///
    /// The table is what the writer, the reader, the screen and the emitter's
    /// printer all walk, so a field the schema grows and the table does not
    /// carry is caught here by name rather than as a `Schema` refusal of every
    /// library.
    #[test]
    fn the_field_table_states_every_field_of_a_pose() {
        let declared: Vec<String> = descriptor("PoseConfig")
            .fields()
            .map(|field| field.name().to_owned())
            .collect();
        let tabled: Vec<String> = POSE_FIELDS
            .iter()
            .map(|field| field.key.to_owned())
            .chain(std::iter::once(PACE_FIELD.to_owned()))
            .collect();
        assert_eq!(declared, tabled);
    }

    /// One emitted library, as the generator writes one: the same protobuf text
    /// a consumer loads.
    ///
    /// A stand-in until the generator lands. **Every field is printed,
    /// defaults included**: the generated schema gives every field
    /// explicit presence, and [`parse_library`] reads through `has_field`, so a
    /// printer that elides `stow: 0` or a square pose's `body_yaw: 0` emits text
    /// that parses into a `MissingField` refusal — surfacing at `screen`, at
    /// process start, on the box, on a library that is well-formed as far as its
    /// writer is concerned.
    fn library_text(poses: &[Pose]) -> String {
        let mut text = String::new();
        for pose in poses {
            text.push_str("poses {\n");
            for field in &POSE_FIELDS {
                text.push_str(&format!("    {}: {:?}\n", field.key, (field.of_pose)(pose)));
            }
            text.push_str(&format!(
                "    {PACE_FIELD}: {}\n}}\n",
                i64::from(pose.duration_ms()) * 1_000_000
            ));
        }
        let stow = poses
            .iter()
            .position(|pose| pose.name() == crate::STOW_POSE)
            .expect("the fixture holds a stow");
        text.push_str(&format!("stow: {stow}\n"));
        text
    }
}
