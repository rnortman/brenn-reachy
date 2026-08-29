//! The driver's configuration: the schema is the parser, and this module is the
//! domain checks and the refusal taxonomy.
//!
//! The numbers this process is built with are authored as protobuf text against
//! the `MotordParams` schema, the same arrangement the cogs' configs use: the
//! schema is the contract, and the file is one message of it. What is different
//! here is who reads it — a Rust binary rather than the Clockwork setup path —
//! so the parse is this module.
//!
//! The accepted syntax is the protobuf text format, as the library defines it.
//! The generated `.proto` for the schema is compiled into a descriptor at
//! startup and every file is parsed against that descriptor, so an unknown
//! field, a field set twice, a bad escape and a malformed line are the
//! library's to refuse — this module owns none of that vocabulary, and adds
//! only the line number every one of the library's refusals leaves out. The
//! proto is *embedded* rather than read as a runfile: the schema the binary
//! enforces has to be the schema it was built with, and a
//! config-schema-not-found failure mode is thereby deleted.
//!
//! What is left here is everything no protobuf library knows about this
//! machine:
//!
//! - Every field is required. A configuration file that forgot to say how long
//!   a cycle is does not get a zero and a driver spinning on it; it gets a
//!   refusal naming the field.
//! - A duration the loop spends is positive, a serial link runs at a positive
//!   rate, and a device path is non-empty and fits the schema's capacity.
//!   Refused, never clamped.
//! - The cycle is the one the driver's own budgets are sized against.
//! - Relations between fields are refused here too, not left to a test over the
//!   file that happens to ship: the file on a machine is the one that runs, and
//!   a hand-edited copy with a dead-man shorter than a cycle would de-torque a
//!   machine whose commander is keeping up perfectly.

use brenn_reachy__driver__motord_config_clk_rs::MotordParamsWire;
use prost_reflect::{
    DescriptorPool, DynamicMessage, FieldDescriptor, MessageDescriptor, ReflectMessage, Value,
};
use reachy_driver::NOMINAL_CYCLE_NS;
use std::borrow::Cow;
use std::path::Path;
use std::sync::OnceLock;

/// The generated Protobuf rendering of `MotordParams`, compiled into this
/// binary.
///
/// Built into the crate rather than read beside it, because a driver that
/// looked for its schema at runtime has a failure mode where the file it finds
/// is not the one the code was built against.
const PARAMS_PROTO: &str = include_str!(env!("MOTORD_PARAMS_PROTO"));

/// What the embedded schema is called while it is being compiled. It names no
/// file on any disk; protobuf compilation wants a name for the unit and this is
/// what a refusal would print.
const PROTO_NAME: &str = "motord_config_clk_proto.proto";

/// The message this reader reads, by the name the schema gives it. The package
/// it sits in comes from the compiled file rather than being restated here.
const PARAMS_MESSAGE: &str = "MotordParams";

/// Which line of a configuration a text refusal is about.
///
/// The line the text stops being a message at: the first line no later piece
/// of the file can rescue. Empty — and printing as nothing — only when even the
/// empty text is refused, which is a refusal no line of the file owns.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct TextLine {
    /// The line, counted from one, and what it says with its surrounding space
    /// taken off.
    pub found: Option<(usize, String)>,
}

impl std::fmt::Display for TextLine {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match &self.found {
            Some((number, text)) => write!(f, ", at line {number}: `{text}`"),
            None => Ok(()),
        }
    }
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
    /// system's; a driver that cannot read its configuration exits rather than
    /// running on a guess.
    #[error("cannot be read: {message}")]
    Unreadable {
        /// What the operating system said.
        message: String,
    },

    /// The text is not a `MotordParams` message. The message is the parser's,
    /// which names the offending field for the refusals that are about a field
    /// it resolved — an unknown name, a name set twice — and names nothing at
    /// all for a syntax refusal. Neither carries a position, so the line is
    /// found here.
    #[error("is not MotordParams text: {message}{at}")]
    Text {
        /// What the parser said.
        message: String,
        /// Which line it was about.
        at: TextLine,
    },

    /// A value of the right kind that the field cannot carry: a duration that
    /// is not positive, a baud rate of zero, a device path longer than the
    /// schema's capacity. Refused rather than clamped.
    #[error("`{name}` cannot be `{value}`: {why}")]
    OutOfRange {
        /// Which field.
        name: &'static str,
        /// What was written.
        value: String,
        /// Why the field cannot carry it.
        why: &'static str,
    },

    /// A value of the right kind that is longer than the field's room for it: a
    /// device path past the capacity the schema declared. Refused rather than
    /// truncated.
    #[error("`{name}` is longer than the {capacity} bytes the schema holds: `{value}`")]
    TooLong {
        /// Which field.
        name: &'static str,
        /// What was written.
        value: String,
        /// What the schema's own capacity for the field is.
        capacity: usize,
    },

    /// A cycle other than the one the driver's own budgets are sized against.
    /// The confirm budget and the blind-cycle count are arguments about a grid
    /// of a stated length; run a different one and those arguments stop holding
    /// while every number still looks reasonable, so the mismatch is refused
    /// here rather than discovered on a machine.
    #[error(
        "`period_ns` is {value} ns, and the budgets this driver runs are sized for {nominal} ns"
    )]
    NotTheNominalCycle {
        /// What was written.
        value: i64,
        /// The cycle the driver's constants are sized against.
        nominal: i64,
    },

    /// Two fields whose values are each acceptable and whose combination is
    /// not.
    #[error("`{name}` is {value} ns and `{against}` is {limit} ns: {why}")]
    Disagreement {
        /// The field the relation is stated about.
        name: &'static str,
        /// What it was set to.
        value: i64,
        /// The field it has to agree with.
        against: &'static str,
        /// What that one was set to.
        limit: i64,
        /// What the relation is, and why the machine wants it.
        why: &'static str,
    },

    /// A field the file never set.
    #[error("`{name}` is not set, and there is no default for it")]
    MissingField {
        /// Which field.
        name: &'static str,
    },

    /// The schema declares something this reader does not carry: a field it has
    /// never heard of, or a field whose type is not the one it reads.
    ///
    /// Not a configuration error — a build defect. The transcription below
    /// walks the descriptor's own fields rather than a list of its own, so a
    /// field added to `motord_config.clk` and not taught here is this refusal
    /// on every configuration, including the shipped one that a test parses.
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
/// messages in, and the file may grow a second one without the reader binding
/// to the wrong one.
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
    // `motord_config.clk` imports nothing, so one file is the whole
    // compilation.
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
pub fn load(path: &Path) -> Result<MotordParamsWire, ParamsError> {
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
/// The parser's refusal, or the first thing this machine will not accept about
/// a message that parsed.
pub fn parse(text: &str) -> Result<MotordParamsWire, ParamsErrorKind> {
    let message =
        DynamicMessage::parse_text_format(descriptor().clone(), text).map_err(|error| {
            ParamsErrorKind::Text {
                message: error.to_string(),
                at: narrow(text),
            }
        })?;
    let wire = transcribe(&message)?;
    agrees(&wire)?;
    Ok(wire)
}

/// The line of `text` the refusal is about, found by growing prefixes.
///
/// Only ever called on a text the parser has already refused as a whole. A line
/// is not attributed by whether it parses in isolation: a line that stands up
/// alone can still be the one refused (a name the file already set), and a line
/// that fails alone can be legal where it sits (a value carried onto the next
/// line). What the parser cannot get past is where the text stops being
/// extendable to a message, so the line reported is the one after the longest
/// prefix of the file that parses whole — the first line no later line can
/// rescue.
///
/// A blank or comment line is never the answer: appending one to a prefix that
/// parses leaves a prefix that parses, so the longest parsing prefix already
/// covers it.
fn narrow(text: &str) -> TextLine {
    let parses = |lines: &[&str]| {
        DynamicMessage::parse_text_format(descriptor().clone(), &lines.join("\n")).is_ok()
    };
    let lines: Vec<&str> = text.lines().collect();
    if !parses(&[]) {
        // The parser refuses the empty message, so the refusal is about
        // something other than what this file says. Nothing to point at.
        return TextLine::default();
    }
    let mut healed = 0;
    for taken in 1..lines.len() {
        if parses(&lines[..taken]) {
            healed = taken;
        }
    }
    TextLine {
        found: lines
            .get(healed)
            .map(|line| (healed + 1, line.trim().to_owned())),
    }
}

/// A parsed message written into the schema's own Rust type, field by field.
///
/// The loop is over *the message's descriptor*, not a list this module keeps: a
/// field the schema declares and this transcription does not carry is a
/// [`ParamsErrorKind::Schema`] refusal rather than a value silently left at
/// zero. That is what makes adding a field to `motord_config.clk` without
/// teaching this function a red test in the same build.
fn transcribe(message: &DynamicMessage) -> Result<MotordParamsWire, ParamsErrorKind> {
    let mut wire = MotordParamsWire::new();
    for field in message.descriptor().fields() {
        match field.name() {
            "period_ns" => {
                let nanos = nanos(message, &field, "period_ns")?;
                if nanos != NOMINAL_CYCLE_NS {
                    return Err(ParamsErrorKind::NotTheNominalCycle {
                        value: nanos,
                        nominal: NOMINAL_CYCLE_NS,
                    });
                }
                wire.set_period_ns(nanos);
            }
            "hold_timeout_ns" => {
                wire.set_hold_timeout_ns(nanos(message, &field, "hold_timeout_ns")?);
            }
            "health_poll_period_ns" => {
                wire.set_health_poll_period_ns(nanos(message, &field, "health_poll_period_ns")?);
            }
            "bus_baud" => {
                let baud = count(message, &field, "bus_baud")?;
                if baud == 0 {
                    return Err(ParamsErrorKind::OutOfRange {
                        name: "bus_baud",
                        value: baud.to_string(),
                        why: "a serial link runs at a positive rate",
                    });
                }
                wire.set_bus_baud(baud);
            }
            "bus_device" => {
                let path = text(message, &field, "bus_device")?;
                if path.is_empty() {
                    return Err(ParamsErrorKind::OutOfRange {
                        name: "bus_device",
                        value: path,
                        why: "the driver has to be told which serial node to open",
                    });
                }
                if !wire.try_set_bus_device(&path) {
                    return Err(ParamsErrorKind::TooLong {
                        name: "bus_device",
                        value: path,
                        capacity: wire.bus_device().capacity(),
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

/// A duration field's value, refused unless it is a positive number of
/// nanoseconds.
fn nanos(
    message: &DynamicMessage,
    field: &FieldDescriptor,
    name: &'static str,
) -> Result<i64, ParamsErrorKind> {
    let nanos = set(message, field, name)?
        .as_i64()
        .ok_or(ParamsErrorKind::Schema {
            name: Cow::Borrowed(name),
            why: "this reader reads it as a whole number of nanoseconds",
        })?;
    if nanos <= 0 {
        return Err(ParamsErrorKind::OutOfRange {
            name,
            value: nanos.to_string(),
            why: "a duration the loop spends has to be positive",
        });
    }
    Ok(nanos)
}

/// A count field's value. What counts this machine will not run is the caller's
/// question; this is the presence check and the read.
fn count(
    message: &DynamicMessage,
    field: &FieldDescriptor,
    name: &'static str,
) -> Result<u32, ParamsErrorKind> {
    set(message, field, name)?
        .as_u32()
        .ok_or(ParamsErrorKind::Schema {
            name: Cow::Borrowed(name),
            why: "this reader reads it as a count",
        })
}

/// A string field's value, copied out of the parsed message. What strings this
/// machine will not run is the caller's question; this is the presence check and
/// the read.
fn text(
    message: &DynamicMessage,
    field: &FieldDescriptor,
    name: &'static str,
) -> Result<String, ParamsErrorKind> {
    set(message, field, name)?
        .as_str()
        .map(str::to_owned)
        .ok_or(ParamsErrorKind::Schema {
            name: Cow::Borrowed(name),
            why: "this reader reads it as a string",
        })
}

/// The relations between fields, checked once every field is set.
///
/// Each of these is a pair of values that is individually reasonable and
/// together describes a driver that does the wrong thing to a machine, so the
/// pair is refused where a single value would be.
fn agrees(message: &MotordParamsWire) -> Result<(), ParamsErrorKind> {
    let period = message.period_ns();
    let hold = message.hold_timeout_ns();

    if hold <= period {
        return Err(ParamsErrorKind::Disagreement {
            name: "hold_timeout_ns",
            value: hold,
            against: "period_ns",
            limit: period,
            why: "a dead-man inside one cycle de-torques a machine whose commander is keeping up",
        });
    }
    if hold % period != 0 {
        return Err(ParamsErrorKind::Disagreement {
            name: "hold_timeout_ns",
            value: hold,
            against: "period_ns",
            limit: period,
            why: "the dead-man is counted in cycles, so a timeout has to be a whole number of them",
        });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::{ParamsErrorKind, parse};
    use brenn_reachy__driver__motord_config_clk_rs::MotordParamsWire;

    /// The schema's own capacity for `bus_device`, read off the generated field
    /// the way the refusal reads it, so the boundary the case below drives and
    /// the number an operator is told are one statement of it.
    fn device_capacity() -> usize {
        MotordParamsWire::new().bus_device().capacity()
    }

    /// The fields the shipped configuration sets, which is every field the
    /// schema declares. Not a list the parser consults — the transcription
    /// walks the descriptor — only the material the cases below build text out
    /// of.
    const FIELDS: [&str; 5] = [
        "period_ns",
        "hold_timeout_ns",
        "health_poll_period_ns",
        "bus_device",
        "bus_baud",
    ];

    /// A whole configuration, as one line per field.
    fn whole() -> String {
        [
            "period_ns: 20000000",
            "hold_timeout_ns: 200000000",
            "health_poll_period_ns: 500000000",
            "bus_device: \"/dev/ttyAMA3\"",
            "bus_baud: 1000000",
        ]
        .join("\n")
    }

    /// `whole()` with one line replaced.
    fn with(field: &str, line: &str) -> String {
        whole()
            .lines()
            .map(|had| {
                if had.starts_with(&format!("{field}:")) {
                    line
                } else {
                    had
                }
            })
            .collect::<Vec<_>>()
            .join("\n")
    }

    /// The parser's refusal of `text` as an operator reads it — what the parser
    /// said and which line it was about — or a panic saying it was accepted.
    fn refused(text: &str) -> String {
        let Err(kind @ ParamsErrorKind::Text { .. }) = parse(text) else {
            panic!("accepted, or refused for a reason other than the text: {text}");
        };
        kind.to_string()
    }

    /// Which line the parser's refusal of `text` points at, as the number and
    /// the line's own words.
    fn refused_at(text: &str) -> Option<(usize, String)> {
        let Err(ParamsErrorKind::Text { at, .. }) = parse(text) else {
            panic!("accepted, or refused for a reason other than the text: {text}");
        };
        at.found
    }

    #[test]
    fn every_field_arrives_as_its_own_value() {
        let message = parse(&whole()).expect("a whole configuration");
        let params = message.validate().expect("a written message validates");
        assert_eq!(params.period_ns, 20_000_000);
        assert_eq!(params.hold_timeout_ns, 200_000_000);
        assert_eq!(params.health_poll_period_ns, 500_000_000);
        assert_eq!(params.bus_device.as_str(), "/dev/ttyAMA3");
        assert_eq!(params.bus_baud, 1_000_000);
    }

    #[test]
    fn comments_and_blank_lines_are_not_fields() {
        let text = format!("# a header\n\n{}\n\n# a footer\n", whole());
        assert!(parse(&text).is_ok());
        let trailing = whole().replace(
            "period_ns: 20000000",
            "period_ns: 20000000 # 20ms, the grid the cogs run",
        );
        let message = parse(&trailing).expect("a trailing comment is not a value");
        assert_eq!(
            message.validate().expect("valid").period_ns,
            20_000_000,
            "the comment did not reach the number"
        );
    }

    #[test]
    fn a_hash_inside_a_quoted_path_is_part_of_the_path() {
        let text = with("bus_device", "bus_device: \"/dev/tty#3\"");
        let message = parse(&text).expect("a path with a hash in it");
        assert_eq!(
            message.validate().expect("valid").bus_device.as_str(),
            "/dev/tty#3"
        );
    }

    #[test]
    fn text_that_is_not_a_message_is_refused_by_the_parser() {
        // A bare word and a name with no value are both syntax, so what the
        // parser says is about what it wanted next rather than about a field —
        // a name it never finished reading is not yet a field it can name. The
        // line it was on is the part an operator goes and fixes, so the refusal
        // carries that.
        for (text, number, line) in [
            (format!("{}\nnonsense\n", whole()), 6, "nonsense"),
            (with("bus_baud", "bus_baud:"), 5, "bus_baud:"),
            (
                with("bus_device", "bus_device: \"unclosed"),
                4,
                "bus_device: \"unclosed",
            ),
            // Comments and blanks are lines an operator counts, so the number
            // is counted over the file's lines and not over the fields in it.
            (
                format!("# a header\n\n{}\nnonsense\n", whole()),
                8,
                "nonsense",
            ),
        ] {
            let message = refused(&text);
            assert!(
                message.contains("expected") || message.contains("invalid"),
                "the refusal says what it wanted: {message}"
            );
            assert!(
                message.contains(&format!(", at line {number}: `{line}`")),
                "the refusal shows which line it was about: {message}"
            );
        }
    }

    #[test]
    fn the_line_a_refusal_points_at_is_where_the_text_stopped_being_a_message() {
        // A file the parser refuses part-way through, with more broken text
        // after the refusal: the line reported is the one the parser stopped
        // at, not whichever later line happens to be nonsense on its own. The
        // library's message names the repeated field and carries no position,
        // so this is the whole of what tells an operator where to look.
        let text = format!("{}\nperiod_ns: 20000000\ngarbage\n", whole());
        assert_eq!(
            refused_at(&text),
            Some((6, "period_ns: 20000000".to_owned())),
            "the repeat is the refusal, and the garbage after it is not"
        );

        // A value carried onto its own line is legal protobuf text and fails
        // in isolation; the line after the longest parsing prefix is the
        // repeat, so a continuation line is never blamed for it.
        let split = format!(
            "{}\nbus_device:\n\"/dev/ttyAMA3\"\n",
            whole()
                .lines()
                .filter(|line| !line.starts_with("bus_device:"))
                .collect::<Vec<_>>()
                .join("\n")
        );
        assert!(
            parse(&split).is_ok(),
            "a value on the line after its field is protobuf text"
        );
        let with_repeat = format!("{split}bus_baud: 1000000\n");
        assert_eq!(
            refused_at(&with_repeat),
            Some((7, "bus_baud: 1000000".to_owned())),
            "the repeated field, not the continuation line above it"
        );
    }

    #[test]
    fn a_refusal_with_no_line_to_point_at_prints_only_what_the_parser_said() {
        let quiet = super::TextLine::default();
        assert_eq!(quiet.to_string(), "", "nothing to append");
        let kind = ParamsErrorKind::Text {
            message: "something the parser said".to_owned(),
            at: quiet,
        };
        assert_eq!(
            kind.to_string(),
            "is not MotordParams text: something the parser said"
        );
    }

    #[test]
    fn a_name_the_schema_does_not_declare_is_refused_by_name() {
        let message = refused(&format!("{}\nbus_parity: 1\n", whole()));
        assert!(
            message.contains("bus_parity"),
            "the refusal names the field: {message}"
        );
    }

    #[test]
    fn a_field_set_twice_is_refused_rather_than_resolved() {
        // The textproto spec makes a repeat of a non-repeated field a parse
        // error, not a last-wins assignment, and this is the pin on that: which
        // of the two was meant is nobody's to guess.
        let message = refused(&format!("{}\nperiod_ns: 10000000\n", whole()));
        assert!(
            message.contains("period_ns"),
            "the refusal names the field: {message}"
        );
        assert!(
            message.contains(", at line 6: `period_ns: 10000000`"),
            "and which of the two lines it stopped at: {message}"
        );
    }

    #[test]
    fn every_field_is_required() {
        for field in FIELDS {
            let text = whole()
                .lines()
                .filter(|line| !line.starts_with(&format!("{field}:")))
                .collect::<Vec<_>>()
                .join("\n");
            assert_eq!(
                parse(&text),
                Err(ParamsErrorKind::MissingField { name: field }),
                "{field} left out"
            );
        }
    }

    #[test]
    fn a_duration_that_is_not_a_positive_number_is_refused() {
        for value in ["0", "-20000000"] {
            let text = with("period_ns", &format!("period_ns: {value}"));
            let Err(ParamsErrorKind::OutOfRange { name, .. }) = parse(&text) else {
                panic!("a {value}ns cycle was accepted");
            };
            assert_eq!(name, "period_ns");
        }
        let message = refused(&with("period_ns", "period_ns: 20ms"));
        assert!(
            message.contains("period_ns"),
            "the refusal names the field an operator has to go fix: {message}"
        );
    }

    #[test]
    fn a_baud_rate_of_zero_is_refused() {
        let text = with("bus_baud", "bus_baud: 0");
        let Err(ParamsErrorKind::OutOfRange { name, .. }) = parse(&text) else {
            panic!("a zero baud rate was accepted");
        };
        assert_eq!(name, "bus_baud");
    }

    #[test]
    fn a_device_has_to_be_a_quoted_path_that_fits() {
        let message = refused(&with("bus_device", "bus_device: /dev/ttyAMA3"));
        assert!(
            message.contains("bus_device"),
            "the refusal names the field an operator has to go fix: {message}"
        );

        let empty = with("bus_device", "bus_device: \"\"");
        let Err(ParamsErrorKind::OutOfRange { name, .. }) = parse(&empty) else {
            panic!("an empty device path was accepted");
        };
        assert_eq!(name, "bus_device");

        let room = device_capacity();
        let exactly = format!("bus_device: \"{}\"", "t".repeat(room));
        assert!(
            parse(&with("bus_device", &exactly)).is_ok(),
            "a path of exactly the schema's capacity fits"
        );
        let long = format!("bus_device: \"{}\"", "t".repeat(room + 1));
        let Err(ParamsErrorKind::TooLong { name, capacity, .. }) =
            parse(&with("bus_device", &long))
        else {
            panic!("a path one byte past the schema's capacity was accepted");
        };
        assert_eq!((name, capacity), ("bus_device", room));
    }

    #[test]
    fn an_escaped_quote_is_a_character_of_the_path() {
        let text = with("bus_device", r#"bus_device: "/dev/a\"b""#);
        let message = parse(&text).expect("an escaped quote");
        assert_eq!(
            message.validate().expect("valid").bus_device.as_str(),
            "/dev/a\"b"
        );
    }

    #[test]
    fn a_quoted_path_and_a_trailing_comment_on_one_line_both_hold() {
        // The line where comment handling and string handling can disagree: a
        // quote inside the path, or a hash inside it, with a comment after the
        // value. The format owns this, and a wrong answer here is a truncated
        // path opening the wrong serial node, so it stays pinned.
        for (line, path) in [
            (r#"bus_device: "/dev/a\"b" # a note"#, "/dev/a\"b"),
            (r#"bus_device: "/dev/tty#3" # a note"#, "/dev/tty#3"),
        ] {
            let message = parse(&with("bus_device", line)).unwrap_or_else(|error| {
                panic!("{line}: {error}");
            });
            assert_eq!(
                message.validate().expect("valid").bus_device.as_str(),
                path,
                "{line} is the device node"
            );
        }
    }

    #[test]
    fn a_hex_escape_and_a_single_quoted_path_are_the_same_device_node() {
        // Both are legal protobuf text for the same characters, and neither is
        // spelled the way the shipped file spells the path.
        for value in [r#""/dev/tty\x41MA3""#, r#"'/dev/ttyAMA3'"#] {
            let text = with("bus_device", &format!("bus_device: {value}"));
            let message = parse(&text).unwrap_or_else(|error| panic!("{value}: {error}"));
            assert_eq!(
                message.validate().expect("valid").bus_device.as_str(),
                "/dev/ttyAMA3",
                "{value} is the device node"
            );
        }
    }

    #[test]
    fn a_cycle_other_than_the_one_the_budgets_are_sized_for_is_refused() {
        let text = with("period_ns", "period_ns: 10000000");
        assert_eq!(
            parse(&text),
            Err(ParamsErrorKind::NotTheNominalCycle {
                value: 10_000_000,
                nominal: reachy_driver::NOMINAL_CYCLE_NS
            })
        );
    }

    #[test]
    fn a_dead_man_that_does_not_outlast_a_cycle_is_refused() {
        for value in ["1", "20000000"] {
            let text = with("hold_timeout_ns", &format!("hold_timeout_ns: {value}"));
            let Err(ParamsErrorKind::Disagreement { name, against, .. }) = parse(&text) else {
                panic!("a {value}ns dead-man was accepted");
            };
            assert_eq!((name, against), ("hold_timeout_ns", "period_ns"));
        }
    }

    #[test]
    fn a_dead_man_that_is_not_a_whole_number_of_cycles_is_refused() {
        let text = with("hold_timeout_ns", "hold_timeout_ns: 210000000");
        let Err(ParamsErrorKind::Disagreement { name, why, .. }) = parse(&text) else {
            panic!("half a cycle of dead-man was accepted");
        };
        assert_eq!(name, "hold_timeout_ns");
        assert!(why.contains("cycles"), "the refusal says why: {why}");
    }

    #[test]
    fn the_startup_timers_are_names_no_configuration_may_carry() {
        // The release is written at process start and the grid is anchored at
        // the next period boundary, so a file setting either of these is a
        // stale copy whose author believes something about this driver that is
        // no longer true.
        for retired in [
            "startup_window_ns: 2000000000",
            "startup_hold_ns: 3000000000",
        ] {
            let text = format!("{}\n{retired}", whole());
            let Err(kind @ ParamsErrorKind::Text { .. }) = parse(&text) else {
                panic!("a retired timer was accepted: {retired}");
            };
            let said = kind.to_string();
            assert!(
                said.contains(retired.split(':').next().expect("a field name")),
                "the refusal does not name the field it is about: {said}"
            );
        }
    }

    #[test]
    fn the_message_this_reader_reads_is_found_by_its_name() {
        // A file with another message ahead of this one: the reader binds to
        // the message its own name says, so what the emitter wrote first, and
        // how many messages the module grows, decide nothing.
        let ahead = super::PARAMS_PROTO.replace(
            "message MotordParams {",
            "message SomethingElse {\n    optional uint32 unrelated = 1;\n}\nmessage MotordParams {",
        );
        assert_ne!(
            ahead,
            super::PARAMS_PROTO,
            "the schema still reads this way"
        );

        let descriptor = super::compiled("ahead.proto", &ahead);
        assert_eq!(descriptor.name(), "MotordParams");
        assert!(
            descriptor.get_field_by_name("period_ns").is_some(),
            "the message the reader's fields are on"
        );
    }

    #[test]
    fn a_field_the_schema_grew_and_this_reader_does_not_carry_is_a_defect() {
        // The transcription walks the descriptor, so a schema with one more
        // field refuses every configuration rather than leaving a number at
        // zero. Driven against a locally extended descriptor, because the
        // embedded schema is the one the binary is built with — and through
        // `compiled`, the same compile the binary's own descriptor comes out
        // of, so the guard cannot pass against a pipeline the parse no longer
        // uses.
        let grown = super::PARAMS_PROTO.replace(
            "optional uint32 bus_baud = 5;",
            "optional uint32 bus_baud = 5;\n    optional uint32 bus_parity = 6;",
        );
        assert_ne!(
            grown,
            super::PARAMS_PROTO,
            "the schema still reads this way"
        );

        let descriptor = super::compiled("grown.proto", &grown);

        let text = format!("{}\nbus_parity: 1\n", whole());
        let message = prost_reflect::DynamicMessage::parse_text_format(descriptor, &text)
            .expect("the grown schema accepts the grown text");
        let Err(ParamsErrorKind::Schema { name, why }) = super::transcribe(&message) else {
            panic!("a field this reader does not carry was accepted");
        };
        assert_eq!(name, "bus_parity");
        assert!(why.contains("does not carry"), "{why}");
    }

    #[test]
    fn a_field_the_schema_retyped_is_a_defect_and_not_a_value() {
        // The other half of the schema-drift guard: a field whose *type* moved
        // under the reader. Each reader says which kind it reads, and reading
        // one field through the wrong one is a build defect rather than a
        // configuration an operator can fix, so the refusal is `Schema` and not
        // `OutOfRange`. Driven against locally retyped descriptors, since the
        // embedded schema is the one the binary is built with.
        for (declared, retyped, field, value, why) in [
            (
                "optional sint64 period_ns = 1;",
                "optional string period_ns = 1;",
                "period_ns",
                "period_ns: \"20000000\"",
                "nanoseconds",
            ),
            (
                "optional uint32 bus_baud = 5;",
                "optional string bus_baud = 5;",
                "bus_baud",
                "bus_baud: \"1000000\"",
                "a count",
            ),
            (
                "optional string bus_device = 4;",
                "optional uint32 bus_device = 4;",
                "bus_device",
                "bus_device: 3",
                "a string",
            ),
        ] {
            let source = super::PARAMS_PROTO.replace(declared, retyped);
            assert_ne!(
                source,
                super::PARAMS_PROTO,
                "the schema still declares {declared}"
            );

            let descriptor = super::compiled("retyped.proto", &source);
            let text = with(field, value);
            let message = prost_reflect::DynamicMessage::parse_text_format(descriptor, &text)
                .unwrap_or_else(|error| panic!("{field} as {retyped}: {error}"));
            let Err(ParamsErrorKind::Schema { name, why: said }) = super::transcribe(&message)
            else {
                panic!("a retyped {field} was read as a value");
            };
            assert_eq!(name, field);
            assert!(
                said.contains(why),
                "{field}: the refusal says what this reader reads: {said}"
            );
        }
    }

    #[test]
    fn a_configuration_that_cannot_be_read_is_refused_with_the_path_in_it() {
        // A directory this test makes, so the file's absence is established
        // rather than assumed: a name shared with another checkout or an older
        // variant of this test would make the refusal a different one.
        let dir = std::env::temp_dir().join(format!("reachy-motord-absent-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("a directory to look in");
        let missing = dir.join("motord_params.textproto");
        std::fs::remove_file(&missing).ok();
        let error = super::load(&missing).expect_err("a path that is not there");
        assert!(matches!(error.kind, ParamsErrorKind::Unreadable { .. },));
        let shown = error.to_string();
        assert!(
            shown.contains(&missing.display().to_string()),
            "an operator reading this has to be told which file: {shown}"
        );
    }

    #[test]
    fn loading_a_file_is_parsing_its_contents_with_the_path_kept() {
        let dir = std::env::temp_dir().join(format!("reachy-motord-params-{}", std::process::id()));
        std::fs::create_dir_all(&dir).expect("a directory to write in");
        let path = dir.join("motord_params.textproto");

        std::fs::write(&path, whole()).expect("writing a whole configuration");
        let loaded = super::load(&path).expect("a whole configuration");
        assert_eq!(
            loaded.validate().expect("valid").bus_device.as_str(),
            "/dev/ttyAMA3",
            "the file's own values, not a default"
        );

        let broken = with("bus_baud", "bus_baud: 0");
        std::fs::write(&path, &broken).expect("writing a broken configuration");
        let error = super::load(&path).expect_err("a zero baud rate");
        assert_eq!(
            error.kind,
            parse(&broken).expect_err("the same text refused the same way"),
            "load refuses exactly what parse refuses"
        );
        assert!(
            error.to_string().contains("motord_params.textproto"),
            "with the file named"
        );

        std::fs::remove_file(&path).ok();
    }
}
