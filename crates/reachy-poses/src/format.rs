//! The pose document: what is on disk, and what a load will accept.
//!
//! Two types for one asset, the same split a clip document has:
//!
//! - [`PoseDoc`] is the message as it was written, one field per schema field,
//!   nothing judged. It is what the extractor emits and what the parser below
//!   produces.
//! - [`Pose`] is the validated form the library and the emitter take. Its
//!   invariants are established once, at load: the name is usable, every number
//!   is finite, the rotation is a unit quaternion, the pace is positive, and the
//!   configuration the document describes is one this machine can hold.
//!
//! Nothing constructs a [`Pose`] except [`Pose::from_doc`] and its text
//! wrapper, so a `Pose` in hand is an asset that has already been refused the
//! chance to be malformed.
//!
//! **The schema is the parser.** The document is protobuf text of
//! `config.PoseDocument`, and the generated `.proto` for `cogs/config.clk` is
//! compiled in memory at first use and every document parsed against that
//! descriptor. An unknown field, a field set twice, a wrong type and a malformed
//! line are therefore the protobuf library's to refuse, and this module owns
//! none of that vocabulary. The proto is *embedded* rather than read beside the
//! binary, so the schema a tool enforces is the schema it was built with.
//!
//! There is no document `version` field, where a clip document has one. A clip
//! is imported from a published dataset and travels; a pose document is authored
//! in this tree, emitted by a tool in this tree, and read by a tool in this tree,
//! and the schema it is parsed against is generated from the same file in the
//! same build. A version would be a field nothing could ever legitimately set.

use std::sync::OnceLock;

use nalgebra::{Isometry3, Quaternion, Translation3, UnitQuaternion};
use prost_reflect::{
    DescriptorPool, DynamicMessage, FieldDescriptor, MessageDescriptor, ReflectMessage, Value,
};
use reachy_kin::{
    EnvelopeConfig, EnvelopeError, EnvelopeReport, check_envelope, default_geometry,
    neutral_head_pose,
};
use reachy_motion::asset_name::{AssetNameError, check_asset_name};
use reachy_motion::joints::JointTargets;
use thiserror::Error;

/// The generated Protobuf rendering of `cogs/config.clk`, compiled into
/// whatever links this crate.
///
/// Built in rather than read beside a tool, for the reason the module header
/// gives: a reader that looked for its schema at runtime has a failure mode
/// where the file it finds is not the one the code was built against.
const CONFIG_PROTO: &str = include_str!(env!("COGS_CONFIG_PROTO"));

/// What the embedded schema is called while it is being compiled. It names no
/// file on any disk; protobuf compilation wants a name for the unit, and this
/// is what a refusal would print.
const PROTO_NAME: &str = "config_clk_proto.proto";

/// The message an authored pose document is one of.
const DOCUMENT_MESSAGE: &str = "PoseDocument";

/// The extension a pose document carries on disk.
///
/// Here rather than in whoever walks the directory: which files are poses is a
/// fact about the asset, and a walk that guessed differently would emit a
/// library missing one.
pub const DOCUMENT_EXT: &str = "textproto";

/// How far a document's rotation quaternion may sit from unit length before it
/// is refused, rather than renormalised.
///
/// Decimal text round-trips a normalised quaternion to well within this;
/// anything beyond it did not come out of a rotation, and normalising it would
/// invent an orientation nobody recorded. The same tolerance `reachy-clips`
/// holds a frame's rotation to, stated here rather than borrowed: the two
/// crates are peers and neither is the other's authority.
pub const QUAT_NORM_TOL: f64 = 1e-6;

/// The longest a document's `description` may be, bytes.
///
/// The schema's own capacity for the field, which a test in this crate holds
/// this number to. Bytes rather than characters because that is what the field
/// stores: a description is prose and may hold anything.
///
/// The bound is checked at load, so a description too long to be written into
/// the message is refused where the author is looking rather than by whatever
/// first tries to carry a document across the wire.
pub const MAX_DESCRIPTION_LEN: usize = 511;

/// The longest pace a document may state, milliseconds.
///
/// The wire's own ceiling on a stated pace, mirrored here and joined to it in
/// `cogs/edge_caps_test`. The library's pace is not a wire value — it is what
/// every move the machine plans for itself takes, the fault ladder's controlled
/// stow among them — so nothing on the wire or at the edge screens it, and the
/// document is the only door it enters by. A stow paced in days would leave the
/// ladder's stow budget expiring before the fold arrived on every fault, with
/// nothing having refused it.
pub const MAX_DURATION_MS: u32 = 600_000;

/// Why a pose document cannot be loaded.
///
/// Every arm is a refusal of the whole asset. There is no partial load and no
/// repair: a pose is a place the machine is sent to, and guessing at half of one
/// is the silent substitution this stack refuses everywhere else.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum PoseDocError {
    /// The bytes are not the protobuf text the schema describes: a malformed
    /// line, an unknown field, a field set twice, a value of the wrong type, or
    /// a repeated value where the schema declares one.
    #[error("pose document is malformed: {message}")]
    Text {
        /// The parser's own account of the problem, position included.
        message: String,
    },

    /// A field the schema declares and this reader does not carry.
    ///
    /// The transcription walks the *descriptor*, not a list kept here, so a
    /// field added to `cogs/config.clk` without teaching this module is a red
    /// test in the same build rather than a value silently left at zero.
    #[error("pose document field {field:?} is unread by this reader: {why}")]
    Schema {
        /// Which field.
        field: String,
        /// What about it this reader cannot do.
        why: &'static str,
    },

    /// A field the document did not set.
    ///
    /// A pose is a whole configuration, so there is no field whose absence has a
    /// sensible reading: an unset `body_yaw` is not "zero", it is a document
    /// that did not say.
    #[error("pose document does not set {field}")]
    MissingField {
        /// Which field.
        field: &'static str,
    },

    /// An array field with a count other than the one the schema declares.
    ///
    /// Protobuf renders a fixed array as `repeated`, which carries no length, so
    /// the declared count is checked here.
    #[error("pose document field {field} holds {found} values; it holds {expected}")]
    ArrayLength {
        /// Which field.
        field: &'static str,
        /// How many the schema declares.
        expected: usize,
        /// How many the document wrote.
        found: usize,
    },

    /// An emitted library holding more poses than the message does.
    ///
    /// A library rather than a document, but the same reader and so the same
    /// refusal type: what `parse_library` is handed is text somebody could have
    /// hand-edited, and the message it is transcribed into has a capacity.
    #[error(
        "library holds {poses} poses; the message holds {}",
        crate::config::MAX_POSES
    )]
    TooManyPoses {
        /// How many the text stated.
        poses: usize,
    },

    /// The `name` is not a usable asset name.
    #[error("pose name {name:?} is unusable: {source}")]
    Name {
        /// What the document said.
        name: String,
        /// Which rule it broke.
        source: AssetNameError,
    },

    /// The `name` is the reserved base spelling.
    ///
    /// `keep` names *do not move the base* on the wire, so a pose under it is an
    /// asset no script could name. Refused here rather than at the emitter: the
    /// name is a fact about the one document, and the author is who can change
    /// it.
    #[error("pose is named {name:?}, which is the base spelling for holding still")]
    Reserved {
        /// What the document said, which is the reserved word.
        name: String,
    },

    /// A `description` longer than the document field holds.
    #[error("pose description is {len} bytes; a document holds {MAX_DESCRIPTION_LEN}")]
    DescriptionTooLong {
        /// How long the document's description is.
        len: usize,
    },

    /// A value that is not a finite number.
    #[error("pose document field {field} is {value}, which is not a finite number")]
    NotFinite {
        /// Which field, indexed where the field is an array.
        field: String,
        /// What the document said.
        value: f64,
    },

    /// A rotation quaternion too far from unit length to be one.
    #[error("pose rotation has norm {norm}, further than {QUAT_NORM_TOL} from unit")]
    QuatNotUnit {
        /// The quaternion's norm as written.
        norm: f64,
    },

    /// A pace of zero.
    ///
    /// Zero is the sentinel an unedited draft carries: how fast the machine
    /// should go to a pose is the author's to say, and no default is right for
    /// every pose.
    #[error("pose states a duration of 0 ms; a move takes a positive time")]
    DurationZero,

    /// A pace past the ceiling.
    ///
    /// The pace the library states is what every move the machine plans for
    /// itself takes, and the fault ladder's controlled stow is one of them: a
    /// pace longer than the ladder's budget is a stow that never arrives.
    #[error("pose states a duration of {duration_ms} ms; a move takes at most {MAX_DURATION_MS}")]
    DurationTooLong {
        /// What the document said.
        duration_ms: u32,
    },

    /// The configuration is not one this machine can hold.
    ///
    /// A content gate, not the safety gate: the tick runs its own envelope check
    /// on every commanded target regardless. A pose outside the envelope never
    /// becomes an asset, so nothing downstream ever has to decide what to do
    /// with one.
    #[error("pose is not a configuration this machine can hold: {0}")]
    Envelope(#[from] EnvelopeError),
}

/// A pose document as it was written: one field per schema field, nothing
/// judged.
///
/// The head is stated **relative to the neutral head pose**, so a recorded
/// figure transcribes with a unit change and nothing else.
#[derive(Clone, Debug, PartialEq)]
pub struct PoseDoc {
    /// The asset name, which is also the document's file stem.
    pub name: String,
    /// Provenance: which session, which segment, why this pose is this shape.
    pub description: String,
    /// Head position relative to the neutral head pose, metres.
    pub dt: [f64; 3],
    /// Head rotation relative to the neutral head pose, `[w, x, y, z]`.
    pub dq: [f64; 4],
    /// Body yaw, radians, absolute.
    pub body_yaw: f64,
    /// Antenna angles, right then left, radians, a direction.
    pub antennas: [f64; 2],
    /// The pace of a move to this pose when the command states none,
    /// milliseconds.
    pub duration_ms: u32,
}

impl PoseDoc {
    /// The head pose relative to the neutral head pose, as the document states
    /// it.
    ///
    /// The quaternion is normalised on the way through, so this answers for a
    /// document the loader has not judged yet — an extractor's draft, or one
    /// whose `dq` is a little off unit. A non-finite figure carries through as
    /// one: what a document may state is [`Pose::from_doc`]'s question.
    #[must_use]
    pub fn relative(&self) -> Isometry3<f64> {
        let quaternion = Quaternion::new(self.dq[0], self.dq[1], self.dq[2], self.dq[3]);
        Isometry3::from_parts(
            Translation3::new(self.dt[0], self.dt[1], self.dt[2]),
            UnitQuaternion::from_quaternion(quaternion),
        )
    }

    /// The head pose in the body frame, absolute, as the document's relative
    /// figures place it. The pose the envelope judges.
    #[must_use]
    pub fn head_pose_body(&self) -> Isometry3<f64> {
        neutral_head_pose() * self.relative()
    }
}

/// A validated pose: a whole base configuration, and the pace of a move to it.
#[derive(Clone, Debug, PartialEq)]
pub struct Pose {
    name: String,
    description: String,
    relative: Isometry3<f64>,
    targets: JointTargets,
    duration_ms: u32,
}

impl Pose {
    /// Parse and validate a pose document.
    ///
    /// # Errors
    ///
    /// [`PoseDocError`]: the parser's refusal, or the first thing this machine
    /// will not accept about a document that parsed.
    pub fn from_text(text: &str) -> Result<Self, PoseDocError> {
        Self::from_doc(parse_document(text)?)
    }

    /// Validate a document that is already in hand.
    ///
    /// The order is deliberate: the name first, so a misnamed document is
    /// refused as that rather than as whatever its numbers happen to be; then
    /// the text the document carries about itself; then the numbers themselves;
    /// then the kinematics, which is the only stage that
    /// solves anything and has nothing to say about a document already wrong.
    ///
    /// # Errors
    ///
    /// [`PoseDocError`], naming the first thing wrong with the document.
    pub fn from_doc(doc: PoseDoc) -> Result<Self, PoseDocError> {
        check_asset_name(&doc.name).map_err(|source| PoseDocError::Name {
            name: doc.name.clone(),
            source,
        })?;
        if doc.name == crate::KEEP_BASE {
            return Err(PoseDocError::Reserved { name: doc.name });
        }
        if doc.description.len() > MAX_DESCRIPTION_LEN {
            return Err(PoseDocError::DescriptionTooLong {
                len: doc.description.len(),
            });
        }
        finite("dt", &doc.dt)?;
        finite("dq", &doc.dq)?;
        finite("body_yaw", &[doc.body_yaw])?;
        finite("antennas", &doc.antennas)?;

        let quaternion = Quaternion::new(doc.dq[0], doc.dq[1], doc.dq[2], doc.dq[3]);
        let norm = quaternion.norm();
        if (norm - 1.0).abs() > QUAT_NORM_TOL {
            return Err(PoseDocError::QuatNotUnit { norm });
        }
        if doc.duration_ms == 0 {
            return Err(PoseDocError::DurationZero);
        }
        if doc.duration_ms > MAX_DURATION_MS {
            return Err(PoseDocError::DurationTooLong {
                duration_ms: doc.duration_ms,
            });
        }

        let relative = doc.relative();
        let head_pose_body = neutral_head_pose() * relative;

        // The envelope, with the default configuration and no margin baseline:
        // a document is judged against the machine's own limits, not against
        // where some earlier pose happened to stand.
        let mut report = EnvelopeReport::default();
        check_envelope(
            default_geometry(),
            &EnvelopeConfig::default(),
            &head_pose_body,
            doc.body_yaw,
            None,
            &mut report,
        )?;

        Ok(Self {
            name: doc.name,
            description: doc.description,
            relative,
            targets: JointTargets {
                head_pose_body,
                body_yaw: doc.body_yaw,
                antennas: doc.antennas,
            },
            duration_ms: doc.duration_ms,
        })
    }

    /// The library name this pose is resolved by, which is also its file stem.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// Where this pose came from and why it is this shape.
    #[must_use]
    pub fn description(&self) -> &str {
        &self.description
    }

    /// The head pose relative to the neutral head pose, as the document states
    /// it and as the emitted library carries it.
    #[must_use]
    pub fn relative(&self) -> &Isometry3<f64> {
        &self.relative
    }

    /// The whole base configuration, absolute: what a mover is sent to.
    #[must_use]
    pub fn targets(&self) -> &JointTargets {
        &self.targets
    }

    /// The pace of a move to this pose when the command states none,
    /// milliseconds. Positive, and no longer than [`MAX_DURATION_MS`].
    #[must_use]
    pub fn duration_ms(&self) -> u32 {
        self.duration_ms
    }
}

/// Every value of `values` is a number, or the first one that is not.
fn finite(field: &'static str, values: &[f64]) -> Result<(), PoseDocError> {
    for (index, &value) in values.iter().enumerate() {
        if !value.is_finite() {
            return Err(PoseDocError::NotFinite {
                field: if values.len() == 1 {
                    field.to_owned()
                } else {
                    format!("{field}[{index}]")
                },
                value,
            });
        }
    }
    Ok(())
}

/// Parse a pose document against the schema's own descriptor.
///
/// # Errors
///
/// [`PoseDocError::Text`] for anything the protobuf text parser refuses, and
/// the transcription's own refusals for a message that parsed but does not say
/// what a pose document says.
pub fn parse_document(text: &str) -> Result<PoseDoc, PoseDocError> {
    let message =
        DynamicMessage::parse_text_format(descriptor(DOCUMENT_MESSAGE), text).map_err(|error| {
            PoseDocError::Text {
                message: error.to_string(),
            }
        })?;
    transcribe(&message)
}

/// A parsed message written into this module's own type, field by field.
///
/// The loop is over *the message's descriptor* rather than a list kept here, so
/// a field `cogs/config.clk` grows and this function does not carry is a
/// refusal rather than a value silently left at zero.
fn transcribe(message: &DynamicMessage) -> Result<PoseDoc, PoseDocError> {
    let mut doc = PoseDoc {
        name: String::new(),
        description: String::new(),
        dt: [0.0; 3],
        dq: [0.0; 4],
        body_yaw: 0.0,
        antennas: [0.0; 2],
        duration_ms: 0,
    };
    for field in message.descriptor().fields() {
        match field.name() {
            "name" => doc.name = text_of(message, &field, "name")?,
            "description" => doc.description = text_of(message, &field, "description")?,
            "dt" => doc.dt = floats(message, &field, "dt")?,
            "dq" => doc.dq = floats(message, &field, "dq")?,
            "body_yaw" => doc.body_yaw = float(message, &field, "body_yaw")?,
            "antennas" => doc.antennas = floats(message, &field, "antennas")?,
            "duration_ms" => doc.duration_ms = count(message, &field, "duration_ms")?,
            other => {
                return Err(PoseDocError::Schema {
                    field: other.to_owned(),
                    why: "the schema grew a field this reader does not carry",
                });
            }
        }
    }
    Ok(doc)
}

/// The value a scalar field was set to, or the refusal that it was not set.
pub(crate) fn set<'a>(
    message: &'a DynamicMessage,
    field: &FieldDescriptor,
    name: &'static str,
) -> Result<std::borrow::Cow<'a, Value>, PoseDocError> {
    if message.has_field(field) {
        Ok(message.get_field(field))
    } else {
        Err(PoseDocError::MissingField { field: name })
    }
}

/// A string field's value.
pub(crate) fn text_of(
    message: &DynamicMessage,
    field: &FieldDescriptor,
    name: &'static str,
) -> Result<String, PoseDocError> {
    set(message, field, name)?
        .as_str()
        .map(str::to_owned)
        .ok_or(PoseDocError::Schema {
            field: name.to_owned(),
            why: "this reader reads it as a string",
        })
}

/// A floating-point field's value. Whether the number is one this machine will
/// take is the validator's question; this is the presence check and the read.
pub(crate) fn float(
    message: &DynamicMessage,
    field: &FieldDescriptor,
    name: &'static str,
) -> Result<f64, PoseDocError> {
    set(message, field, name)?
        .as_f64()
        .ok_or(PoseDocError::Schema {
            field: name.to_owned(),
            why: "this reader reads it as a number",
        })
}

/// A signed whole-number field's value, which is how a duration crosses.
pub(crate) fn int(
    message: &DynamicMessage,
    field: &FieldDescriptor,
    name: &'static str,
) -> Result<i64, PoseDocError> {
    set(message, field, name)?
        .as_i64()
        .ok_or(PoseDocError::Schema {
            field: name.to_owned(),
            why: "this reader reads it as a whole signed number",
        })
}

/// A count field's value.
pub(crate) fn count(
    message: &DynamicMessage,
    field: &FieldDescriptor,
    name: &'static str,
) -> Result<u32, PoseDocError> {
    set(message, field, name)?
        .as_u32()
        .ok_or(PoseDocError::Schema {
            field: name.to_owned(),
            why: "this reader reads it as a whole count",
        })
}

/// A fixed array of floats, refused unless the document wrote exactly the
/// declared count.
///
/// The Protobuf backend renders a `FixedArray` as `repeated`, which carries no
/// length of its own, so the length the schema declares is checked here — an
/// absent field is a count of zero and is refused as the short array it is,
/// which is the same thing an author sees on a two-element `dt`.
pub(crate) fn floats<const N: usize>(
    message: &DynamicMessage,
    field: &FieldDescriptor,
    name: &'static str,
) -> Result<[f64; N], PoseDocError> {
    let value = message.get_field(field);
    let list = value.as_list().ok_or(PoseDocError::Schema {
        field: name.to_owned(),
        why: "this reader reads it as an array of numbers",
    })?;
    if list.len() != N {
        return Err(PoseDocError::ArrayLength {
            field: name,
            expected: N,
            found: list.len(),
        });
    }
    let mut out = [0.0; N];
    for (slot, entry) in out.iter_mut().zip(list) {
        *slot = entry.as_f64().ok_or(PoseDocError::Schema {
            field: name.to_owned(),
            why: "this reader reads it as an array of numbers",
        })?;
    }
    Ok(out)
}

/// The descriptor for one message of the embedded schema.
///
/// # Panics
///
/// A schema that does not compile, or that does not declare the message asked
/// for, is a build defect rather than a document error: every unit test in this
/// crate runs this same compile, so nothing can ship past it.
// TODO(clk-textproto-reader-copies): this compile and the presence/type
// accessors above are the third statement of one pattern; the others are in
// `reachy-motord` and `reachy-host`'s `params.rs`.
pub(crate) fn descriptor(message: &str) -> MessageDescriptor {
    // In memory throughout: `protox::compile` reads the filesystem, and writing
    // the embedded schema to a temp file to feed it would reintroduce the
    // schema-at-runtime failure mode the embed exists to delete.
    // `config.clk` imports nothing, so one file is the whole compilation.
    static POOL: OnceLock<(DescriptorPool, String)> = OnceLock::new();
    let (pool, package) = POOL.get_or_init(|| {
        let file = protox::file::File::from_source(PROTO_NAME, CONFIG_PROTO)
            .expect("the embedded schema is generated and has to compile");
        let mut pool = DescriptorPool::new();
        pool.add_file_descriptor_proto(file.file_descriptor_proto().clone())
            .expect("the embedded schema is generated and has to link");
        let package = pool
            .get_file_by_name(PROTO_NAME)
            .expect("the file just added")
            .package_name()
            .to_owned();
        (pool, package)
    });
    let full = if package.is_empty() {
        message.to_owned()
    } else {
        format!("{package}.{message}")
    };
    pool.get_message_by_name(&full)
        .unwrap_or_else(|| panic!("`{PROTO_NAME}` declares `{full}`, which this reader reads"))
}

#[cfg(test)]
mod tests {
    use super::{
        MAX_DESCRIPTION_LEN, MAX_DURATION_MS, Pose, PoseDoc, PoseDocError, QUAT_NORM_TOL,
        parse_document,
    };
    use crate::record::reduce_antennas;
    use reachy_kin::ik::min_pose_margin;
    use reachy_kin::{
        EnvelopeConfig, EnvelopeReport, check_envelope, default_geometry, neutral_head_pose,
    };
    use reachy_motion::asset_name::AssetNameError;

    /// A document that loads, as a person would write one.
    fn full_text() -> String {
        [
            "name: \"neutral\"",
            "description: \"head square and level at nominal height\"",
            "dt: [0.0, 0.0, 0.0]",
            "dq: [1.0, 0.0, 0.0, 0.0]",
            "body_yaw: 0.0",
            "antennas: [-0.1745, 0.1745]",
            "duration_ms: 800",
        ]
        .join("\n")
    }

    fn full_doc() -> PoseDoc {
        parse_document(&full_text()).expect("the fixture parses")
    }

    #[test]
    fn a_well_formed_document_parses_into_its_fields() {
        assert_eq!(
            full_doc(),
            PoseDoc {
                name: "neutral".to_owned(),
                description: "head square and level at nominal height".to_owned(),
                dt: [0.0, 0.0, 0.0],
                dq: [1.0, 0.0, 0.0, 0.0],
                body_yaw: 0.0,
                antennas: [-0.1745, 0.1745],
                duration_ms: 800,
            }
        );
    }

    /// The schema is the parser: a field it does not declare, and a value of the
    /// wrong type, are the protobuf library's refusals and not this module's
    /// vocabulary.
    #[test]
    fn the_parser_refuses_what_the_schema_does_not_describe() {
        let unknown = format!("{}\npitch: 0.3", full_text());
        assert!(matches!(
            parse_document(&unknown),
            Err(PoseDocError::Text { .. })
        ));

        let wrong_type = full_text().replace("body_yaw: 0.0", "body_yaw: \"square\"");
        assert!(matches!(
            parse_document(&wrong_type),
            Err(PoseDocError::Text { .. })
        ));

        let twice = format!("{}\nbody_yaw: 0.1", full_text());
        assert!(matches!(
            parse_document(&twice),
            Err(PoseDocError::Text { .. })
        ));
    }

    /// `FixedArray` crosses the Protobuf backend as `repeated`, which carries no
    /// length, so the declared count is this reader's to hold — including the
    /// absent case, which is a count of zero.
    #[test]
    fn an_array_of_the_wrong_length_is_refused() {
        let short = full_text().replace("dt: [0.0, 0.0, 0.0]", "dt: [0.0, 0.0]");
        assert_eq!(
            parse_document(&short),
            Err(PoseDocError::ArrayLength {
                field: "dt",
                expected: 3,
                found: 2,
            })
        );

        let long = full_text().replace("antennas: [-0.1745, 0.1745]", "antennas: [0.0, 0.0, 0.0]");
        assert_eq!(
            parse_document(&long),
            Err(PoseDocError::ArrayLength {
                field: "antennas",
                expected: 2,
                found: 3,
            })
        );

        let absent = full_text().replace("dq: [1.0, 0.0, 0.0, 0.0]\n", "");
        assert_eq!(
            parse_document(&absent),
            Err(PoseDocError::ArrayLength {
                field: "dq",
                expected: 4,
                found: 0,
            })
        );
    }

    /// A pose is a whole configuration, so a scalar the document did not set is
    /// refused rather than read as a zero somebody might have meant.
    #[test]
    fn a_scalar_the_document_did_not_set_is_refused() {
        let absent = full_text().replace("body_yaw: 0.0\n", "");
        assert_eq!(
            parse_document(&absent),
            Err(PoseDocError::MissingField { field: "body_yaw" })
        );

        let no_pace = full_text().replace("\nduration_ms: 800", "");
        assert_eq!(
            parse_document(&no_pace),
            Err(PoseDocError::MissingField {
                field: "duration_ms"
            })
        );
    }

    #[test]
    fn the_name_is_held_to_the_asset_name_rule() {
        let doc = PoseDoc {
            name: "Neutral".to_owned(),
            ..full_doc()
        };
        assert_eq!(
            Pose::from_doc(doc),
            Err(PoseDocError::Name {
                name: "Neutral".to_owned(),
                source: AssetNameError::BadChar { ch: 'N' },
            })
        );
    }

    /// A description longer than the field holds is refused where the author
    /// can still edit it, rather than by whatever first writes the document into
    /// the message.
    #[test]
    fn a_description_past_the_field_is_refused() {
        let at_cap = PoseDoc {
            description: "a".repeat(MAX_DESCRIPTION_LEN),
            ..full_doc()
        };
        assert!(Pose::from_doc(at_cap).is_ok());

        let doc = PoseDoc {
            description: "a".repeat(MAX_DESCRIPTION_LEN + 1),
            ..full_doc()
        };
        assert_eq!(
            Pose::from_doc(doc),
            Err(PoseDocError::DescriptionTooLong {
                len: MAX_DESCRIPTION_LEN + 1
            })
        );
    }

    /// Every committed document fits the field it is written into.
    #[test]
    fn the_committed_descriptions_fit_the_document() {
        for name in ["stow", "neutral", "peek"] {
            let pose = committed(name);
            assert!(pose.description().len() <= MAX_DESCRIPTION_LEN);
        }
    }

    #[test]
    fn a_value_that_is_not_a_number_is_refused() {
        let doc = PoseDoc {
            dt: [0.0, f64::NAN, 0.0],
            ..full_doc()
        };
        // Compared field by field: a NaN is not equal to itself, so the value
        // is asserted as the thing it is rather than against a literal.
        match Pose::from_doc(doc) {
            Err(PoseDocError::NotFinite { field, value }) => {
                assert_eq!(field, "dt[1]");
                assert!(value.is_nan());
            }
            other => panic!("a NaN in `dt` is refused as that, not as {other:?}"),
        }

        let doc = PoseDoc {
            body_yaw: f64::INFINITY,
            ..full_doc()
        };
        assert_eq!(
            Pose::from_doc(doc),
            Err(PoseDocError::NotFinite {
                field: "body_yaw".to_owned(),
                value: f64::INFINITY,
            })
        );

        // The antennas are checked here and nowhere else: the envelope check
        // below knows nothing about them.
        let doc = PoseDoc {
            antennas: [0.0, f64::NAN],
            ..full_doc()
        };
        assert!(matches!(
            Pose::from_doc(doc),
            Err(PoseDocError::NotFinite { .. })
        ));
    }

    /// Refused rather than renormalised: a quaternion this far from unit did not
    /// come out of a rotation, and normalising it would invent an orientation
    /// nobody wrote.
    #[test]
    fn a_rotation_that_is_not_a_unit_quaternion_is_refused() {
        let doc = PoseDoc {
            dq: [1.1, 0.0, 0.0, 0.0],
            ..full_doc()
        };
        assert_eq!(
            Pose::from_doc(doc),
            Err(PoseDocError::QuatNotUnit { norm: 1.1 })
        );

        // Inside the tolerance is accepted as written, not snapped.
        let doc = PoseDoc {
            dq: [1.0 + QUAT_NORM_TOL / 2.0, 0.0, 0.0, 0.0],
            ..full_doc()
        };
        assert!(Pose::from_doc(doc).is_ok());
    }

    #[test]
    fn a_pace_of_zero_is_refused() {
        let doc = PoseDoc {
            duration_ms: 0,
            ..full_doc()
        };
        assert_eq!(Pose::from_doc(doc), Err(PoseDocError::DurationZero));
    }

    /// The library's pace is what every move the machine plans for itself takes,
    /// and nothing downstream of the document screens it: a typed zero too many
    /// would be a fault ladder whose controlled stow never arrives.
    #[test]
    fn a_pace_past_the_ceiling_is_refused() {
        let at_ceiling = PoseDoc {
            duration_ms: MAX_DURATION_MS,
            ..full_doc()
        };
        assert!(Pose::from_doc(at_ceiling).is_ok());

        let doc = PoseDoc {
            duration_ms: MAX_DURATION_MS + 1,
            ..full_doc()
        };
        assert_eq!(
            Pose::from_doc(doc),
            Err(PoseDocError::DurationTooLong {
                duration_ms: MAX_DURATION_MS + 1
            })
        );
    }

    /// `keep` names holding the base still, so a pose under that name is an
    /// asset on the unit no script could ever reach.
    #[test]
    fn a_pose_named_for_holding_still_is_refused() {
        let doc = PoseDoc {
            name: crate::KEEP_BASE.to_owned(),
            ..full_doc()
        };
        assert_eq!(
            Pose::from_doc(doc),
            Err(PoseDocError::Reserved {
                name: crate::KEEP_BASE.to_owned()
            })
        );
    }

    /// A pose nobody can stand at never becomes an asset. This is a content
    /// gate: the tick's own envelope check still runs on every commanded target.
    #[test]
    fn a_pose_outside_the_envelope_is_refused() {
        let doc = PoseDoc {
            dt: [0.0, 0.0, 0.5],
            ..full_doc()
        };
        assert!(matches!(
            Pose::from_doc(doc),
            Err(PoseDocError::Envelope(_))
        ));

        let doc = PoseDoc {
            body_yaw: 4.0,
            ..full_doc()
        };
        assert!(matches!(
            Pose::from_doc(doc),
            Err(PoseDocError::Envelope(_))
        ));
    }

    /// The head is stated relative to the neutral head pose, so a document of
    /// all zeros is the neutral head pose exactly, and the absolute pose is what
    /// the mover is handed.
    #[test]
    fn the_head_is_relative_to_neutral_and_the_targets_are_absolute() {
        let pose = Pose::from_doc(full_doc()).expect("the fixture loads");
        assert_eq!(pose.name(), "neutral");
        assert_eq!(pose.duration_ms(), 800);
        assert_eq!(*pose.relative(), nalgebra::Isometry3::identity());
        assert_eq!(
            pose.targets().head_pose_body,
            reachy_kin::neutral_head_pose()
        );
        assert_eq!(pose.targets().antennas, [-0.1745, 0.1745]);
        assert_eq!(pose.targets().body_yaw, 0.0);
    }

    /// The transcription walks the descriptor, so a schema with one more field
    /// refuses every document rather than leaving a number at zero. Driven
    /// against a locally extended descriptor, because the embedded schema is the
    /// one the crate is built with.
    #[test]
    fn a_field_the_schema_grew_and_this_reader_does_not_carry_is_a_defect() {
        let grown = super::CONFIG_PROTO.replace(
            "message PoseDocument {",
            "message PoseDocument {\n    optional double pitch_bias = 99;",
        );
        assert_ne!(grown, super::CONFIG_PROTO, "the message the reader reads");
        let file = protox::file::File::from_source("grown.proto", &grown)
            .expect("the extended schema compiles");
        let mut pool = prost_reflect::DescriptorPool::new();
        pool.add_file_descriptor_proto(file.file_descriptor_proto().clone())
            .expect("the extended schema links");
        let descriptor = pool
            .get_message_by_name("brenn_reachy.cogs.config_clk_proto.PoseDocument")
            .expect("the extended schema declares it");
        let message = prost_reflect::DynamicMessage::parse_text_format(descriptor, &full_text())
            .expect("the document still parses");
        assert_eq!(
            super::transcribe(&message),
            Err(PoseDocError::Schema {
                field: "pitch_bias".to_owned(),
                why: "the schema grew a field this reader does not carry",
            })
        );
    }

    /// The committed documents, read from the runfiles.
    fn committed_text(name: &str) -> String {
        let dir = std::env::var("POSE_DOCUMENTS").expect("the documents are staged for this test");
        let path = std::path::Path::new(&dir).join(format!("{name}.textproto"));
        std::fs::read_to_string(&path).unwrap_or_else(|why| panic!("{path:?}: {why}"))
    }

    /// The committed documents, read from the runfiles.
    fn committed(name: &str) -> Pose {
        Pose::from_text(&committed_text(name))
            .unwrap_or_else(|why| panic!("{name} does not load: {why}"))
    }

    /// Every document this tree commits loads, under its own file stem, and
    /// states a pace. A document the loader refuses is one no library could be
    /// emitted from, so this fails before the emitter does.
    #[test]
    fn the_committed_documents_load_under_their_own_names() {
        for name in ["stow", "neutral", "peek"] {
            let pose = committed(name);
            assert_eq!(pose.name(), name, "a document's name is its file stem");
            assert!(pose.duration_ms() > 0);
            assert!(
                !pose.description().is_empty(),
                "{name} says where it came from"
            );
        }
        let neutral = committed("neutral");
        assert_eq!(neutral.targets().head_pose_body, neutral_head_pose());
        assert_eq!(neutral.targets().antennas, [-0.1745, 0.1745]);
    }

    /// Where the committed stow folds its antennas, right then left, radians.
    ///
    /// The number the fault ladder commands, the disarm sequence judges arrival
    /// at, and the document's own description reasons about. Nothing else in the
    /// tree sees it: the envelope judges the head and the yaw, and the loader
    /// checks an antenna for finiteness alone, so a re-extraction or a hand-edit
    /// that moved the fold — or swapped the two sides — would change where the
    /// antennas go with every other test still passing.
    const STOW_ANTENNAS_RAD: [f64; 2] = [-3.4591, 3.3596];

    /// Where the committed peek leans its antennas, on the same grounds.
    const PEEK_ANTENNAS_RAD: [f64; 2] = [-0.2824, 0.2040];

    /// The two recorded documents fold and lean where they were recorded, in
    /// the spelling a document states — the reduced representative — and inside
    /// the range the command path will take an antenna to.
    #[test]
    fn the_committed_antennas_are_where_the_recording_left_them() {
        for (name, expected) in [("stow", STOW_ANTENNAS_RAD), ("peek", PEEK_ANTENNAS_RAD)] {
            let antennas = committed(name).targets().antennas;
            for (side, angle) in antennas.into_iter().enumerate() {
                assert!(
                    (angle - expected[side]).abs() < 1e-4,
                    "{name} antenna {side} is at {angle} rad where it was at {}",
                    expected[side]
                );
                assert!(
                    (reachy_motion::tick::ANTENNA_GOAL_MIN_RAD
                        ..=reachy_motion::tick::ANTENNA_GOAL_MAX_RAD)
                        .contains(&angle),
                    "{name} antenna {side} is at {angle} rad, outside what may be commanded"
                );
            }
            assert_eq!(
                reduce_antennas(antennas),
                antennas,
                "{name} states a multi-turn spelling of its fold rather than the direction"
            );
        }
    }

    /// What the committed stow clears the linkage's singular configurations by,
    /// metres, as the committed document and the present geometry give it. The
    /// tightest committed pose in the tree, and the reason the clearance floor
    /// is where it is.
    const STOW_MARGIN_M: f64 = 2.660e-3;

    /// The stow's clearance, pinned as a number rather than as an inequality
    /// against the floor.
    ///
    /// The document is parsed rather than loaded: a load runs the envelope
    /// check, so a geometry change that ate the headroom would panic at the load
    /// and this assertion would only ever be reached when it already held. Read
    /// this way, a change that halves the clearance fails here with both numbers
    /// while the pose is still readable, which is the early warning a guard on
    /// the tightest pose in the tree is for.
    #[test]
    fn the_committed_stow_clears_the_clearance_floor() {
        let doc = parse_document(&committed_text("stow")).expect("the committed stow parses");
        let floor = EnvelopeConfig::default().min_toggle_margin;
        let margin = min_pose_margin(default_geometry(), &doc.head_pose_body());
        assert!(
            (margin - STOW_MARGIN_M).abs() < 1e-5,
            "the stow clears by {:.3} mm where it cleared by {:.3} mm; the floor is {:.3} mm, so \
             the headroom is {:.3} mm where it was {:.3} mm",
            margin * 1e3,
            STOW_MARGIN_M * 1e3,
            floor * 1e3,
            (margin - floor) * 1e3,
            (STOW_MARGIN_M - floor) * 1e3
        );
        assert!(
            margin > floor,
            "and the pose is one this machine may be sent to"
        );

        // The command-path case: a stow commanded from neutral, which is where
        // every raise leaves the head. The baseline never excuses a pose the
        // floor already admits, so this is the floor answering.
        let stow = committed("stow");
        let baseline = min_pose_margin(default_geometry(), &neutral_head_pose());
        let mut report = EnvelopeReport::default();
        check_envelope(
            default_geometry(),
            &EnvelopeConfig::default(),
            &stow.targets().head_pose_body,
            stow.targets().body_yaw,
            Some(baseline),
            &mut report,
        )
        .expect("the stow is commandable from neutral");
    }
}
