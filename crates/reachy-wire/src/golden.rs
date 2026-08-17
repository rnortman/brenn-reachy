//! Golden vectors: the frozen record of this layout, and the conformance gate
//! for any decoder written against it.
//!
//! Four datagrams, one per message type, each stated three times over — as
//! field values in Rust, as field values in `golden.json`, and as the exact
//! bytes both are supposed to produce. Rust asserts all three agree; a decoder
//! written in another language asserts against the same file.
//!
//! That is what makes it a gate rather than a fixture: a change to the codec
//! that also changes the vectors fails the byte assertions, a change to the
//! vectors alone fails the file comparison, and a change to the file alone
//! fails both. The layout cannot drift without something going red.
//!
//! The values are chosen so every float has an exact binary representation and
//! prints identically in any formatter — the file is compared as text, and a
//! last-digit disagreement between two float formatters is not a fact about
//! this wire format.
//!
//! Regenerating after a deliberate layout change: run the crate's tests, take
//! the JSON the mismatch prints, and write it to `golden.json`.

use crate::{
    Control, ControlOp, DriverEvent, EventKind, GoalSetpoint, JOINT_COUNT, JOINT_MASK_ALL, Message,
    PoseSample,
};

/// The shipped vector file, compiled in so both build lanes read one copy.
pub const GOLDEN_JSON: &str = include_str!("../golden.json");

/// Positions used by the pose sample's `present` and the goal's `targets`.
const PRESENT: [f64; JOINT_COUNT] = [0.0, 1.0, -1.0, 0.5, -0.5, 2.0, -2.0, 0.25, -0.25];

/// Positions used by the pose sample's `commanded`.
const COMMANDED: [f64; JOINT_COUNT] = [1.5, -1.5, 3.0, -3.0, 4.0, -4.0, 0.125, -0.125, 8.0];

/// One golden datagram: what it is, and what it must look like as bytes.
pub struct GoldenVector {
    /// Stable name, shared with the JSON file and the Python checker.
    pub name: &'static str,
    /// The sequence number in the header.
    pub seq: u32,
    /// The message.
    pub message: Message,
    /// The whole datagram, lowercase hex.
    pub hex: &'static str,
}

impl GoldenVector {
    /// The datagram this vector's message encodes to.
    #[must_use]
    pub fn encode(&self) -> Vec<u8> {
        match &self.message {
            Message::PoseSample(m) => m.encode(self.seq).to_vec(),
            Message::DriverEvent(m) => m.encode(self.seq).to_vec(),
            Message::GoalSetpoint(m) => m.encode(self.seq).to_vec(),
            Message::Control(m) => m.encode(self.seq).to_vec(),
        }
    }

    /// The bytes named by `hex`.
    #[must_use]
    pub fn bytes(&self) -> Vec<u8> {
        from_hex(self.hex)
    }
}

/// Every golden vector, in file order.
#[must_use]
pub fn vectors() -> Vec<GoldenVector> {
    vec![
        GoldenVector {
            name: "pose_sample",
            seq: 7,
            message: Message::PoseSample(PoseSample {
                nominal_time_ns: 1_700_000_000_000_000_000,
                sample_time_ns: 1_700_000_000_000_500_000,
                present_valid: true,
                commanded_valid: true,
                torque_off_latched: false,
                miss_mask: 260,
                present: PRESENT,
                commanded: COMMANDED,
            }),
            hex: "575201010700000000002a36fe9c971720a13136fe9c97170304010000000000000000000000000000f03f000000000000f0bf000000000000e03f000000000000e0bf000000000000004000000000000000c0000000000000d03f000000000000d0bf000000000000f83f000000000000f8bf000000000000084000000000000008c0000000000000104000000000000010c0000000000000c03f000000000000c0bf0000000000002040",
        },
        GoldenVector {
            name: "driver_event",
            seq: 1,
            message: Message::DriverEvent(DriverEvent {
                kind: EventKind::HoldTimeoutTorqueOff,
                detail: 42,
                time_ns: 1_700_000_000_000_000_000,
            }),
            hex: "5752010301000000012a00000000002a36fe9c9717",
        },
        GoldenVector {
            name: "goal_setpoint",
            seq: u32::MAX,
            message: Message::GoalSetpoint(GoalSetpoint {
                execute_at_ns: 1_700_000_000_040_000_000,
                mask: JOINT_MASK_ALL,
                targets: PRESENT,
            }),
            hex: "57520110ffffffff005a8c38fe9c9717ff010000000000000000000000000000f03f000000000000f0bf000000000000e03f000000000000e0bf000000000000004000000000000000c0000000000000d03f000000000000d0bf",
        },
        GoldenVector {
            name: "control",
            seq: 0,
            message: Message::Control(Control {
                op: ControlOp::TorqueOffNow,
            }),
            hex: "575201110000000001",
        },
    ]
}

/// Render the vectors as the exact text of `golden.json`.
///
/// The file is generated rather than hand-kept so the two sides of the gate
/// cannot be edited apart, and it is compared as text rather than as parsed
/// JSON so the crate needs no JSON parser to check it.
#[must_use]
pub fn golden_json() -> String {
    let mut out = String::new();
    out.push_str("{\n");
    out.push_str(
        "  \"note\": \"Golden wire vectors for reachy-wire. Generated by the crate; the Rust \
         codec and the Python checker both assert against this file. Do not hand-edit.\",\n",
    );
    out.push_str("  \"vectors\": [\n");
    let vectors = vectors();
    for (i, vector) in vectors.iter().enumerate() {
        out.push_str("    {\n");
        out.push_str(&format!("      \"name\": \"{}\",\n", vector.name));
        out.push_str(&format!(
            "      \"msg_type\": {},\n",
            msg_type_of(&vector.message)
        ));
        out.push_str(&format!("      \"seq\": {},\n", vector.seq));
        out.push_str("      \"fields\": {\n");
        out.push_str(&fields_json(&vector.message));
        out.push_str("      },\n");
        out.push_str(&format!(
            "      \"hex\": \"{}\",\n",
            to_hex(&vector.encode())
        ));
        out.push_str(&format!("      \"total_len\": {}\n", vector.encode().len()));
        out.push_str("    }");
        out.push_str(if i + 1 == vectors.len() { "\n" } else { ",\n" });
    }
    out.push_str("  ]\n}\n");
    out
}

fn msg_type_of(message: &Message) -> u8 {
    match message {
        Message::PoseSample(_) => PoseSample::MSG_TYPE,
        Message::DriverEvent(_) => DriverEvent::MSG_TYPE,
        Message::GoalSetpoint(_) => GoalSetpoint::MSG_TYPE,
        Message::Control(_) => Control::MSG_TYPE,
    }
}

/// The `fields` object's body: one `"name": value` line per field, indented to
/// sit inside it, the last without a comma.
fn fields_json(message: &Message) -> String {
    let rows: Vec<(&str, String)> = match message {
        Message::PoseSample(m) => vec![
            ("nominal_time_ns", m.nominal_time_ns.to_string()),
            ("sample_time_ns", m.sample_time_ns.to_string()),
            ("present_valid", m.present_valid.to_string()),
            ("commanded_valid", m.commanded_valid.to_string()),
            ("torque_off_latched", m.torque_off_latched.to_string()),
            ("miss_mask", m.miss_mask.to_string()),
            ("present", float_array(&m.present)),
            ("commanded", float_array(&m.commanded)),
        ],
        Message::DriverEvent(m) => vec![
            ("kind", m.kind.as_u8().to_string()),
            ("detail", m.detail.to_string()),
            ("time_ns", m.time_ns.to_string()),
        ],
        Message::GoalSetpoint(m) => vec![
            ("execute_at_ns", m.execute_at_ns.to_string()),
            ("mask", m.mask.to_string()),
            ("targets", float_array(&m.targets)),
        ],
        Message::Control(m) => vec![("op", m.op.as_u8().to_string())],
    };
    let mut out = String::new();
    for (i, (name, value)) in rows.iter().enumerate() {
        let comma = if i + 1 == rows.len() { "" } else { "," };
        out.push_str(&format!("        \"{name}\": {value}{comma}\n"));
    }
    out
}

fn float_array(values: &[f64; JOINT_COUNT]) -> String {
    let rendered: Vec<String> = values.iter().map(|v| format!("{v:?}")).collect();
    format!("[{}]", rendered.join(", "))
}

/// `bytes` as lowercase hex, two digits each and nothing between them.
///
/// The one rendering of a datagram this stack has: the vectors below are stated
/// in it, and every scenario checker that prints bytes on a failure prints them
/// this way, so two checkers' output can be compared digit for digit.
#[must_use]
pub fn to_hex(bytes: &[u8]) -> String {
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push_str(&format!("{byte:02x}"));
    }
    out
}

/// Bytes named by a lowercase hex string.
///
/// # Panics
///
/// On anything that is not whole pairs of hex digits. The inputs are stated
/// constants and test expectations, never anything read off a wire.
#[must_use]
pub fn from_hex(hex: &str) -> Vec<u8> {
    let digits: Vec<u8> = hex.bytes().filter(|b| !b.is_ascii_whitespace()).collect();
    assert!(
        digits.len().is_multiple_of(2),
        "hex string has an odd number of digits"
    );
    digits
        .chunks(2)
        .map(|pair| {
            let text = core::str::from_utf8(pair).expect("ascii hex");
            u8::from_str_radix(text, 16).expect("ascii hex")
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::decode;

    #[test]
    fn every_vector_encodes_to_its_stated_bytes() {
        for vector in vectors() {
            assert_eq!(
                to_hex(&vector.encode()),
                to_hex(&vector.bytes()),
                "vector `{}` encodes differently than it says",
                vector.name
            );
        }
    }

    #[test]
    fn every_vectors_stated_bytes_decode_to_its_stated_message() {
        for vector in vectors() {
            let bytes = vector.bytes();
            let (header, message) =
                decode(&bytes).unwrap_or_else(|e| panic!("vector `{}`: {e}", vector.name));
            assert_eq!(header.seq, vector.seq, "vector `{}`", vector.name);
            assert_eq!(
                header.msg_type,
                msg_type_of(&vector.message),
                "vector `{}`",
                vector.name
            );
            assert_eq!(message, vector.message, "vector `{}`", vector.name);
        }
    }

    #[test]
    fn the_vectors_cover_every_message_type() {
        let mut types: Vec<u8> = vectors().iter().map(|v| msg_type_of(&v.message)).collect();
        types.sort_unstable();
        assert_eq!(
            types,
            vec![
                PoseSample::MSG_TYPE,
                DriverEvent::MSG_TYPE,
                GoalSetpoint::MSG_TYPE,
                Control::MSG_TYPE
            ]
        );
    }

    #[test]
    fn the_shipped_file_is_what_the_crate_generates() {
        let generated = golden_json();
        assert_eq!(
            generated, GOLDEN_JSON,
            "golden.json is out of date. Write this in its place:\n{generated}"
        );
    }

    #[test]
    fn hex_helpers_are_inverses() {
        let bytes: Vec<u8> = (0..=255u8).collect();
        assert_eq!(from_hex(&to_hex(&bytes)), bytes);
    }
}
