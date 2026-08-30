//! The host's edge configuration: the schema is the parser, and this module is
//! the domain checks and the refusal taxonomy.
//!
//! The mechanism around those checks — the embedded proto, the descriptor and
//! its cache, the text parse, the descriptor-walking transcription and the
//! generic half of the refusals — is a second copy of the driver's reader,
//! which has already drifted: a text refusal here carries no line number and
//! there it does.
//! TODO(params-reader-shared)
//!
//! The numbers this process is built with are authored as protobuf text against
//! the `HostParams` schema, the same arrangement the driver's configuration and
//! the cogs' use: the schema is the contract, and the file is one message of
//! it. The parse is here because the reader is a Rust binary rather than the
//! Clockwork setup path.
//!
//! The accepted syntax is the protobuf text format, as the library defines it.
//! The generated `.proto` for the schema is compiled into a descriptor at
//! startup and every file is parsed against that descriptor, so an unknown
//! field, a field set twice, a bad escape and a malformed line are the
//! library's to refuse. The proto is *embedded* rather than read as a runfile:
//! the schema the binary enforces has to be the schema it was built with, and a
//! config-schema-not-found failure mode is thereby deleted.
//!
//! What is left here is what no protobuf library knows about this edge. Every
//! field is required — a file that forgot to name the machine does not get an
//! empty string and an edge that answers to nothing. And the values that become
//! screens are handed to [`EdgeConfig::new`] rather than checked twice: the
//! crate that screens with a number is the crate that says which numbers it can
//! screen with, and a second opinion here would be one that drifts.

use std::borrow::Cow;
use std::path::{Path, PathBuf};
use std::sync::OnceLock;

use prost_reflect::{
    DescriptorPool, DynamicMessage, FieldDescriptor, MessageDescriptor, ReflectMessage, Value,
};
use reachy_edge::{ConfigError, EdgeConfig};

use brenn_reachy__host__host_config_clk_rs::HostParamsWire;

/// The generated Protobuf rendering of `HostParams`, compiled into this binary.
///
/// Built into the crate rather than read beside it, because a host that looked
/// for its schema at runtime has a failure mode where the file it finds is not
/// the one the code was built against.
const PARAMS_PROTO: &str = include_str!(env!("HOST_PARAMS_PROTO"));

/// What the embedded schema is called while it is being compiled. It names no
/// file on any disk; protobuf compilation wants a name for the unit and this is
/// what a refusal would print.
const PROTO_NAME: &str = "host_config_clk_proto.proto";

/// The message this reader reads, by the name the schema gives it. The package
/// it sits in comes from the compiled file rather than being restated here.
const PARAMS_MESSAGE: &str = "HostParams";

/// What the host was told to be.
///
/// The edge's own configuration and the one path the host reads for it. Split
/// this way because [`EdgeConfig`] is the sans-I/O crate's and holds no path:
/// the sidecar is read here and handed over as a table.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct HostSettings {
    /// What the edge screens and compiles with.
    pub edge: EdgeConfig,
    /// Where the clip library's name table is.
    pub clip_names: PathBuf,
}

/// A configuration that was not read.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[error("{path}: {kind}")]
pub struct ParamsError {
    /// What was being read: a file's path, or `<text>` when the text came from
    /// a caller rather than a file.
    pub path: String,
    /// What was wrong with it.
    pub kind: ParamsErrorKind,
}

/// What was wrong with a configuration.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ParamsErrorKind {
    /// The file could not be read at all. The message is the operating
    /// system's; a host that cannot read its configuration exits rather than
    /// running on a guess about which machine it is.
    #[error("cannot be read: {message}")]
    Unreadable {
        /// What the operating system said.
        message: String,
    },

    /// The text is not a `HostParams` message. The message is the parser's,
    /// which names the offending field for the refusals about a field it
    /// resolved — an unknown name, a name set twice — and names nothing at all
    /// for a syntax refusal.
    #[error("is not HostParams text: {message}")]
    Text {
        /// What the parser said.
        message: String,
    },

    /// A field the file never set. Every field is required: there is no number
    /// here whose absence has a safe meaning.
    #[error("`{name}` is not set, and there is no default for it")]
    MissingField {
        /// Which field.
        name: &'static str,
    },

    /// A text longer than the field's room for it. Refused rather than
    /// truncated: a truncated path names a file that is not there, and a
    /// truncated pod name answers for a machine that is not this one.
    #[error("`{name}` is longer than the {capacity} bytes the schema holds: `{value}`")]
    TooLong {
        /// Which field.
        name: &'static str,
        /// What was written.
        value: String,
        /// The schema's own capacity for the field.
        capacity: usize,
    },

    /// A value of the right kind the field cannot carry: an unnamed pod, a stow
    /// of no time, a cap below any script. The reason is the edge's own, from
    /// the crate that screens with these numbers.
    #[error("the edge will not run on these numbers: {0}")]
    Rejected(#[from] ConfigError),

    /// A path field left empty. Separate from the edge's own refusals because
    /// the path is this crate's — the edge holds no path at all.
    #[error("`{name}` is empty, and the host has to be told where to read it")]
    NoPath {
        /// Which field.
        name: &'static str,
    },

    /// A count that does not fit what this reader carries it as.
    #[error("`{name}` is {value}, which is past what this reader carries it as")]
    Unrepresentable {
        /// Which field.
        name: &'static str,
        /// What was written.
        value: String,
    },

    /// The schema declares something this reader does not carry: a field it has
    /// never heard of, or a field whose type is not the one it reads.
    ///
    /// Not a configuration error — a build defect. The transcription walks the
    /// descriptor's own fields rather than a list of its own, so a field added
    /// to `host_config.clk` and not taught here is this refusal on every
    /// configuration, including the shipped one a test parses.
    #[error("the schema declares `{name}`, which this reader does not carry: {why}")]
    Schema {
        /// Which field. Owned only for the name a schema grew, which the reader
        /// has no literal for.
        name: Cow<'static, str>,
        /// What about it this reader cannot do.
        why: &'static str,
    },
}

/// The descriptor every configuration is parsed against, compiled once from the
/// embedded schema.
///
/// # Panics
///
/// As [`compiled`].
fn descriptor() -> &'static MessageDescriptor {
    static DESCRIPTOR: OnceLock<MessageDescriptor> = OnceLock::new();
    DESCRIPTOR.get_or_init(|| compiled(PROTO_NAME, PARAMS_PROTO))
}

/// The [`PARAMS_MESSAGE`] descriptor out of one protobuf source, compiled in
/// memory.
///
/// The message is looked up by the name the schema gives it, so which message
/// this reader reads does not depend on the order the emitter wrote the file's
/// messages in.
///
/// # Panics
///
/// A schema that does not compile, or that does not declare the message this
/// reader reads, is a build defect rather than a configuration error: every
/// unit test in this module runs this same compile, so nothing can ship past
/// it.
fn compiled(name: &str, source: &str) -> MessageDescriptor {
    // In memory throughout: `protox::compile` reads the filesystem, and writing
    // the embedded schema to a temp file to feed it would reintroduce the
    // schema-at-runtime failure mode the embed exists to delete.
    // `host_config.clk` imports nothing, so one file is the whole compilation.
    let file = protox::file::File::from_source(name, source)
        .expect("the embedded schema is generated and has to compile");
    let mut pool = DescriptorPool::new();
    pool.add_file_descriptor_proto(file.file_descriptor_proto().clone())
        .expect("the embedded schema is generated and has to link");
    let package = pool
        .get_file_by_name(name)
        .expect("the file just added")
        .package_name()
        .to_owned();
    let full = if package.is_empty() {
        PARAMS_MESSAGE.to_owned()
    } else {
        format!("{package}.{PARAMS_MESSAGE}")
    };
    pool.get_message_by_name(&full)
        .unwrap_or_else(|| panic!("`{name}` declares `{full}`, which this reader reads"))
}

/// Read the configuration at `path`.
///
/// # Errors
///
/// [`ParamsError`] carrying the path and either the operating system's reason
/// the file could not be read, or the first thing wrong with its contents.
pub fn load(path: &Path) -> Result<HostSettings, ParamsError> {
    let shown = path.display().to_string();
    let text = std::fs::read_to_string(path).map_err(|error| ParamsError {
        path: shown.clone(),
        kind: ParamsErrorKind::Unreadable {
            message: error.to_string(),
        },
    })?;
    parse(&text).map_err(|kind| ParamsError { path: shown, kind })
}

/// Read a configuration out of text that is already in hand.
///
/// # Errors
///
/// The parser's refusal, or the first thing this host will not accept about a
/// message that parsed.
pub fn parse(text: &str) -> Result<HostSettings, ParamsErrorKind> {
    let message =
        DynamicMessage::parse_text_format(descriptor().clone(), text).map_err(|error| {
            ParamsErrorKind::Text {
                message: error.to_string(),
            }
        })?;
    settings(&transcribe(&message)?)
}

/// A parsed message written into the schema's own Rust type, field by field.
///
/// The loop is over *the message's descriptor*, not a list this module keeps: a
/// field the schema declares and this transcription does not carry is a
/// [`ParamsErrorKind::Schema`] refusal rather than a value silently left at
/// zero. That is what makes adding a field to `host_config.clk` without
/// teaching this function a red test in the same build.
fn transcribe(message: &DynamicMessage) -> Result<HostParamsWire, ParamsErrorKind> {
    let mut wire = HostParamsWire::new();
    for field in message.descriptor().fields() {
        match field.name() {
            "pod" => {
                let pod = text(message, &field, "pod")?;
                if !wire.try_set_pod(&pod) {
                    return Err(ParamsErrorKind::TooLong {
                        name: "pod",
                        value: pod,
                        capacity: wire.pod().capacity(),
                    });
                }
            }
            "stow_duration_ms" => {
                wire.set_stow_duration_ms(count(message, &field, "stow_duration_ms")?);
            }
            "body_cap_bytes" => {
                wire.set_body_cap_bytes(count(message, &field, "body_cap_bytes")?);
            }
            "clip_names_path" => {
                let path = text(message, &field, "clip_names_path")?;
                if path.is_empty() {
                    return Err(ParamsErrorKind::NoPath {
                        name: "clip_names_path",
                    });
                }
                if !wire.try_set_clip_names_path(&path) {
                    return Err(ParamsErrorKind::TooLong {
                        name: "clip_names_path",
                        value: path,
                        capacity: wire.clip_names_path().capacity(),
                    });
                }
            }
            unread => {
                return Err(ParamsErrorKind::Schema {
                    name: Cow::Owned(unread.to_owned()),
                    why: "the schema grew a field this reader does not carry",
                });
            }
        }
    }
    Ok(wire)
}

/// The message as what the host runs on.
///
/// The edge's screens are checked by the edge: every value that becomes one is
/// handed to [`EdgeConfig::new`], and its refusal is carried through. The body
/// cap is the one conversion — the schema carries a count of bytes and the
/// screen compares against a length — and a machine whose pointers are smaller
/// than the number written is refused rather than wrapped.
fn settings(wire: &HostParamsWire) -> Result<HostSettings, ParamsErrorKind> {
    let params = wire.validate().map_err(|_| ParamsErrorKind::Schema {
        name: Cow::Borrowed("<message>"),
        why: "the transcribed message is not a valid one, which is a defect in this reader",
    })?;
    let body_cap_bytes =
        usize::try_from(params.body_cap_bytes).map_err(|_| ParamsErrorKind::Unrepresentable {
            name: "body_cap_bytes",
            value: params.body_cap_bytes.to_string(),
        })?;
    let edge = EdgeConfig::new(params.pod.as_str(), params.stow_duration_ms, body_cap_bytes)?;
    Ok(HostSettings {
        edge,
        clip_names: PathBuf::from(params.clip_names_path.as_str()),
    })
}

/// The value a field was set to, or the refusal that it was not set.
fn set<'a>(
    message: &'a DynamicMessage,
    field: &FieldDescriptor,
    name: &'static str,
) -> Result<Cow<'a, Value>, ParamsErrorKind> {
    if message.has_field(field) {
        Ok(message.get_field(field))
    } else {
        Err(ParamsErrorKind::MissingField { name })
    }
}

/// A text field's value.
fn text(
    message: &DynamicMessage,
    field: &FieldDescriptor,
    name: &'static str,
) -> Result<String, ParamsErrorKind> {
    Ok(set(message, field, name)?
        .as_str()
        .ok_or(ParamsErrorKind::Schema {
            name: Cow::Borrowed(name),
            why: "this reader reads it as text",
        })?
        .to_owned())
}

/// A count field's value.
fn count(
    message: &DynamicMessage,
    field: &FieldDescriptor,
    name: &'static str,
) -> Result<u32, ParamsErrorKind> {
    let value = set(message, field, name)?
        .as_u32()
        .ok_or(ParamsErrorKind::Schema {
            name: Cow::Borrowed(name),
            why: "this reader reads it as a whole count",
        })?;
    Ok(value)
}

#[cfg(test)]
mod tests {
    use prost_reflect::DynamicMessage;
    use reachy_edge::{ConfigError, MIN_BODY_CAP_BYTES};

    use super::{
        HostParamsWire, PARAMS_MESSAGE, PARAMS_PROTO, PROTO_NAME, ParamsErrorKind, compiled, parse,
        transcribe,
    };

    /// A configuration that parses, as lines.
    fn lines() -> Vec<&'static str> {
        vec![
            "pod: \"kitchen-reachy\"",
            "stow_duration_ms: 3000",
            "body_cap_bytes: 8192",
            "clip_names_path: \"cogs/clip_library.names.json\"",
        ]
    }

    /// The whole configuration, as text.
    fn whole() -> String {
        lines().join("\n")
    }

    /// The configuration with the line naming `field` replaced by `line`.
    fn with(field: &str, line: &str) -> String {
        let prefix = format!("{field}:");
        let mut text: Vec<String> = lines()
            .into_iter()
            .filter(|existing| !existing.starts_with(&prefix))
            .map(str::to_owned)
            .collect();
        text.push(line.to_owned());
        text.join("\n")
    }

    /// The configuration with the line naming `field` taken out.
    fn without(field: &str) -> String {
        let prefix = format!("{field}:");
        lines()
            .into_iter()
            .filter(|existing| !existing.starts_with(&prefix))
            .collect::<Vec<_>>()
            .join("\n")
    }

    /// What a text was refused with.
    fn refused(text: &str) -> ParamsErrorKind {
        parse(text).expect_err("a configuration this reader refuses")
    }

    #[test]
    fn a_whole_configuration_is_what_the_edge_runs_on() {
        let settings = parse(&whole()).expect("the fixture parses");
        assert_eq!(settings.edge.pod(), "kitchen-reachy");
        assert_eq!(settings.edge.stow_duration_ms(), 3000);
        assert_eq!(settings.edge.body_cap_bytes(), 8192);
        assert_eq!(
            settings.clip_names.to_str(),
            Some("cogs/clip_library.names.json")
        );
    }

    #[test]
    fn every_field_is_required() {
        for field in [
            "pod",
            "stow_duration_ms",
            "body_cap_bytes",
            "clip_names_path",
        ] {
            let refusal = refused(&without(field));
            assert_eq!(
                refusal,
                ParamsErrorKind::MissingField { name: field },
                "{field} left unset",
            );
        }
    }

    #[test]
    fn the_edge_is_what_refuses_the_numbers_it_screens_with() {
        // Each of these is the edge's own refusal, carried through rather than
        // re-derived: this reader states none of these rules.
        assert!(matches!(
            refused(&with("pod", "pod: \"  \"")),
            ParamsErrorKind::Rejected(ConfigError::PodUnnamed),
        ));
        assert!(matches!(
            refused(&with("stow_duration_ms", "stow_duration_ms: 0")),
            ParamsErrorKind::Rejected(ConfigError::StowTakesNoTime),
        ));
        let cap = format!("body_cap_bytes: {}", MIN_BODY_CAP_BYTES - 1);
        assert!(matches!(
            refused(&with("body_cap_bytes", &cap)),
            ParamsErrorKind::Rejected(ConfigError::CapBelowAnyScript { .. }),
        ));
    }

    #[test]
    fn an_empty_path_names_no_file() {
        assert_eq!(
            refused(&with("clip_names_path", "clip_names_path: \"\"")),
            ParamsErrorKind::NoPath {
                name: "clip_names_path"
            },
        );
    }

    #[test]
    fn a_text_past_the_schemas_room_is_refused_rather_than_cut() {
        let room = HostParamsWire::new().pod().capacity();
        let exactly = format!("pod: \"{}\"", "p".repeat(room));
        assert!(parse(&with("pod", &exactly)).is_ok(), "a name that fits");
        let long = format!("pod: \"{}\"", "p".repeat(room + 1));
        match refused(&with("pod", &long)) {
            ParamsErrorKind::TooLong { name, capacity, .. } => {
                assert_eq!((name, capacity), ("pod", room));
            }
            other => panic!("a name past the schema's room: {other}"),
        }
    }

    #[test]
    fn a_name_the_schema_does_not_declare_is_the_parsers_refusal() {
        let refusal = refused(&format!("{}\nlead_ms: 8000", whole()));
        let ParamsErrorKind::Text { message } = refusal else {
            panic!("an unknown field is the parser's to refuse");
        };
        assert!(message.contains("lead_ms"), "{message}");
    }

    #[test]
    fn a_field_set_twice_is_the_parsers_refusal() {
        assert!(matches!(
            refused(&format!("{}\npod: \"other\"", whole())),
            ParamsErrorKind::Text { .. },
        ));
    }

    #[test]
    fn a_value_of_the_wrong_kind_is_the_parsers_refusal() {
        assert!(matches!(
            refused(&with("stow_duration_ms", "stow_duration_ms: \"3000\"")),
            ParamsErrorKind::Text { .. },
        ));
        assert!(matches!(
            refused(&with("pod", "pod: 3")),
            ParamsErrorKind::Text { .. },
        ));
    }

    #[test]
    fn a_field_the_schema_grows_and_this_reader_does_not_carry_is_a_refusal() {
        // The transcription walks the descriptor, so a field the schema grows
        // and this module has no arm for refuses every configuration —
        // including the shipped one. Driven against a locally extended
        // descriptor, because the embedded schema is the one the binary is
        // built with, and through `compiled`, so the guard cannot pass against
        // a pipeline the parse no longer uses.
        let grown = PARAMS_PROTO.replace(
            "optional string clip_names_path = 4;",
            "optional string clip_names_path = 4;\n    optional uint32 lead_ms = 5;",
        );
        assert_ne!(grown, PARAMS_PROTO, "the schema still reads this way");

        let descriptor = compiled("grown.proto", &grown);
        let text = format!("{}\nlead_ms: 8000\n", whole());
        let message = DynamicMessage::parse_text_format(descriptor, &text)
            .expect("the grown schema accepts the grown text");
        let Err(ParamsErrorKind::Schema { name, why }) = transcribe(&message) else {
            panic!("a field this reader does not carry was accepted");
        };
        assert_eq!(name, "lead_ms");
        assert!(why.contains("does not carry"), "{why}");
    }

    #[test]
    fn a_field_the_schema_retyped_is_a_defect_and_not_a_value() {
        // The other half of the drift guard: a field whose *type* moved under
        // the reader. Each reader says which kind it reads, and reading one
        // field through the wrong one is a build defect rather than something
        // an operator can fix, so the refusal is `Schema`.
        for (declared, retyped, field, value, why) in [
            (
                "optional uint32 stow_duration_ms = 2;",
                "optional string stow_duration_ms = 2;",
                "stow_duration_ms",
                "stow_duration_ms: \"3000\"",
                "a whole count",
            ),
            (
                "optional string pod = 1;",
                "optional uint32 pod = 1;",
                "pod",
                "pod: 3",
                "text",
            ),
        ] {
            let source = PARAMS_PROTO.replace(declared, retyped);
            assert_ne!(source, PARAMS_PROTO, "the schema still declares {declared}");

            let descriptor = compiled("retyped.proto", &source);
            let message = DynamicMessage::parse_text_format(descriptor, &with(field, value))
                .unwrap_or_else(|error| panic!("{field} as {retyped}: {error}"));
            let Err(ParamsErrorKind::Schema { name, why: said }) = transcribe(&message) else {
                panic!("a retyped {field} was read as a value");
            };
            assert_eq!(name, field);
            assert!(said.contains(why), "{field}: {said}");
        }
    }

    #[test]
    fn the_message_this_reader_reads_is_the_one_the_schema_declares() {
        assert_eq!(compiled(PROTO_NAME, PARAMS_PROTO).name(), PARAMS_MESSAGE);
    }

    #[test]
    fn a_configuration_that_cannot_be_read_is_refused_with_the_path_in_it() {
        // A directory this test makes, so the file's absence is established
        // rather than assumed.
        let dir = std::env::temp_dir().join(format!("reachy-host-absent-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("a directory to look in");
        let missing = dir.join("host_params.textproto");
        std::fs::remove_file(&missing).ok();
        let error = super::load(&missing).expect_err("a path that is not there");
        assert!(matches!(error.kind, ParamsErrorKind::Unreadable { .. }));
        let shown = error.to_string();
        assert!(
            shown.contains(&missing.display().to_string()),
            "an operator reading this has to be told which file: {shown}"
        );
    }
}
