//! The sequence document: a motion assembled from other motions.
//!
//! A sequence is an ordered list of entries, each either a reference to another
//! asset — a clip or another sequence — or a gap that holds whatever the
//! previous entry left. That is "motions calling motions", and it is the whole
//! composition vocabulary: no parallel tracks inside a sequence, because
//! parallelism is what layering overlays is for, and a timeline algebra nobody
//! asked for is a large thing to maintain for a use nobody has.
//!
//! This module is the document only — shape, spelling, and the checks a single
//! file can answer alone. Whether a reference resolves, whether the nesting
//! terminates, and what the flattened motion actually is are questions about
//! the whole library and live in [`crate::library`].
//!
//! The same two-type split as a clip: [`SequenceDoc`] is the JSON, [`Sequence`]
//! is the validated form, and nothing builds the latter but the validator.

use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::format::{FORMAT_VERSION, NameError, validate_name};

/// Why a sequence document cannot be loaded.
///
/// Every arm refuses the whole asset. An entry that says two things at once, or
/// nothing at all, is an authoring mistake whose most likely repair — guess
/// which half was meant — is exactly the silent substitution this stack refuses
/// everywhere.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum SequenceError {
    /// The bytes are not the JSON this format is.
    #[error("sequence document is malformed: {detail}")]
    Malformed {
        /// The parser's own account of the problem.
        detail: String,
    },

    /// A `version` this crate does not read.
    #[error("sequence document is version {version}; this reader is version {FORMAT_VERSION}")]
    UnsupportedVersion {
        /// What the document said.
        version: u32,
    },

    /// A `kind` other than `sequence`.
    #[error("expected a sequence document; this one is kind {kind:?}")]
    WrongKind {
        /// What the document said.
        kind: String,
    },

    /// The `name` is not a usable asset name.
    #[error("sequence name {name:?} is unusable: {source}")]
    Name {
        /// What the document said.
        name: String,
        /// Which rule it broke.
        source: NameError,
    },

    /// An entry's `ref` is not a usable asset name.
    #[error("entry {entry} references {name:?}, which is not a usable name: {source}")]
    RefName {
        /// The entry's index.
        entry: usize,
        /// What the document said.
        name: String,
        /// Which rule it broke.
        source: NameError,
    },

    /// An empty `entries` list.
    #[error("sequence has no entries")]
    NoEntries,

    /// An entry carrying both a `ref` and a `gap_ms`.
    #[error("entry {entry} is both a reference and a gap; an entry is one or the other")]
    RefAndGap {
        /// The entry's index.
        entry: usize,
    },

    /// An entry carrying neither a `ref` nor a `gap_ms`.
    #[error("entry {entry} is neither a reference nor a gap")]
    NeitherRefNorGap {
        /// The entry's index.
        entry: usize,
    },

    /// A `speed` on a gap entry.
    ///
    /// Refused rather than ignored: a gap already runs on the motion clock and
    /// is scaled by its neighbours' effective speed, so a speed written here
    /// reads as a second, contradictory scaling of the same hold.
    #[error("entry {entry} is a gap carrying a speed; a gap is scaled by the motion's own speed")]
    SpeedOnGap {
        /// The entry's index.
        entry: usize,
    },

    /// A `speed` that is not a usable multiplier.
    ///
    /// The bounds a speed is held to are the *flattened* ones — entry speeds
    /// multiply through nesting, so a fast entry inside a slow one can be
    /// perfectly legal — and that check belongs to resolution. All that is
    /// checkable here is that the number is one.
    #[error("entry {entry} has speed {speed}, which is not finite and positive")]
    Speed {
        /// The entry's index.
        entry: usize,
        /// What the document said.
        speed: f64,
    },

    /// A `gap_ms` of zero: a hold with no duration.
    #[error("entry {entry} is a gap of zero milliseconds, which holds nothing")]
    ZeroGap {
        /// The entry's index.
        entry: usize,
    },
}

/// One entry of a sequence document.
///
/// Exactly one of `ref` and `gap_ms` is present; the option-ness is the
/// discriminator rather than a tagged enum because that is what reads well in
/// the file, where the common entry is a bare `{"ref": "…"}`.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct EntryDoc {
    /// The asset this entry plays: a clip or another sequence.
    #[serde(rename = "ref", default, skip_serializing_if = "Option::is_none")]
    pub reference: Option<String>,
    /// The multiplier on that asset's clock, defaulting to 1.0.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub speed: Option<f64>,
    /// A hold, milliseconds, in place of a reference.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub gap_ms: Option<u32>,
}

/// A sequence as written on disk.
///
/// Unknown keys are refused, for the same reason a clip's are: this format has
/// exactly one writer, so a key the reader does not know came from somewhere
/// else or is a typo, and both are worth hearing at load.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct SequenceDoc {
    /// Format version; [`FORMAT_VERSION`] or refused.
    pub version: u32,
    /// Discriminator against a clip document; `sequence`.
    pub kind: String,
    /// The library name this asset is invoked by.
    pub name: String,
    /// Free text, written by the author.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub description: Option<String>,
    /// The entries, played in order.
    pub entries: Vec<EntryDoc>,
}

/// One validated entry: a reference with its speed, or a gap.
#[derive(Clone, Debug, PartialEq)]
pub enum Entry {
    /// Play the named asset at this multiple of its own clock.
    Play {
        /// The referenced asset's library name.
        reference: String,
        /// The multiplier on its clock; 1.0 unless the document said otherwise.
        speed: f64,
    },
    /// Hold the previous entry's final delta for this long.
    Gap {
        /// The hold, milliseconds, at the sequence's own effective speed.
        ms: u32,
    },
}

/// A validated sequence: entries whose shape is settled, references unresolved.
///
/// Deliberately still unresolved. A library loads its assets in whatever order
/// a directory hands them over, and a sequence may name one that has not been
/// read yet, so resolution is a second pass over the whole set rather than
/// something a single document can complete.
#[derive(Clone, Debug, PartialEq)]
pub struct Sequence {
    name: String,
    description: Option<String>,
    entries: Vec<Entry>,
}

impl Sequence {
    /// Parse and validate a sequence document.
    pub fn from_json(json: &str) -> Result<Self, SequenceError> {
        let doc: SequenceDoc =
            serde_json::from_str(json).map_err(|err| SequenceError::Malformed {
                detail: err.to_string(),
            })?;
        Self::from_doc(doc)
    }

    /// Validate a parsed document.
    pub fn from_doc(doc: SequenceDoc) -> Result<Self, SequenceError> {
        if doc.version != FORMAT_VERSION {
            return Err(SequenceError::UnsupportedVersion {
                version: doc.version,
            });
        }
        if doc.kind != "sequence" {
            return Err(SequenceError::WrongKind { kind: doc.kind });
        }
        validate_name(&doc.name).map_err(|source| SequenceError::Name {
            name: doc.name.clone(),
            source,
        })?;
        if doc.entries.is_empty() {
            return Err(SequenceError::NoEntries);
        }

        let mut entries = Vec::with_capacity(doc.entries.len());
        for (index, entry) in doc.entries.iter().enumerate() {
            entries.push(validated_entry(index, entry)?);
        }

        Ok(Self {
            name: doc.name,
            description: doc.description,
            entries,
        })
    }

    /// The library name this sequence is invoked by.
    #[must_use]
    pub fn name(&self) -> &str {
        &self.name
    }

    /// The author's description, if it carried one.
    #[must_use]
    pub fn description(&self) -> Option<&str> {
        self.description.as_deref()
    }

    /// The entries, in play order. Never empty.
    #[must_use]
    pub fn entries(&self) -> &[Entry] {
        &self.entries
    }

    /// The document form of this sequence, for a writer.
    #[must_use]
    pub fn to_doc(&self) -> SequenceDoc {
        SequenceDoc {
            version: FORMAT_VERSION,
            kind: "sequence".to_owned(),
            name: self.name.clone(),
            description: self.description.clone(),
            entries: self
                .entries
                .iter()
                .map(|entry| match entry {
                    Entry::Play { reference, speed } => EntryDoc {
                        reference: Some(reference.clone()),
                        speed: Some(*speed),
                        gap_ms: None,
                    },
                    Entry::Gap { ms } => EntryDoc {
                        gap_ms: Some(*ms),
                        ..EntryDoc::default()
                    },
                })
                .collect(),
        }
    }
}

/// Validate one document entry and convert it.
fn validated_entry(index: usize, entry: &EntryDoc) -> Result<Entry, SequenceError> {
    match (&entry.reference, entry.gap_ms) {
        (Some(_), Some(_)) => Err(SequenceError::RefAndGap { entry: index }),
        (None, None) => Err(SequenceError::NeitherRefNorGap { entry: index }),
        (Some(reference), None) => {
            validate_name(reference).map_err(|source| SequenceError::RefName {
                entry: index,
                name: reference.clone(),
                source,
            })?;
            let speed = entry.speed.unwrap_or(1.0);
            if !speed.is_finite() || speed <= 0.0 {
                return Err(SequenceError::Speed {
                    entry: index,
                    speed,
                });
            }
            Ok(Entry::Play {
                reference: reference.clone(),
                speed,
            })
        }
        (None, Some(ms)) => {
            if entry.speed.is_some() {
                return Err(SequenceError::SpeedOnGap { entry: index });
            }
            if ms == 0 {
                return Err(SequenceError::ZeroGap { entry: index });
            }
            Ok(Entry::Gap { ms })
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A well-formed document: a reference, a gap, a sped-up reference.
    fn doc() -> SequenceDoc {
        SequenceDoc {
            version: FORMAT_VERSION,
            kind: "sequence".to_owned(),
            name: "pod/greet".to_owned(),
            description: Some("a test".to_owned()),
            entries: vec![
                EntryDoc {
                    reference: Some("pollen/emotions/welcoming1".to_owned()),
                    ..EntryDoc::default()
                },
                EntryDoc {
                    gap_ms: Some(300),
                    ..EntryDoc::default()
                },
                EntryDoc {
                    reference: Some("pod/nod-twice".to_owned()),
                    speed: Some(1.25),
                    gap_ms: None,
                },
            ],
        }
    }

    #[test]
    fn full_document_loads() {
        let sequence = Sequence::from_doc(doc()).expect("well-formed");
        assert_eq!(sequence.name(), "pod/greet");
        assert_eq!(sequence.description(), Some("a test"));
        assert_eq!(
            sequence.entries(),
            &[
                Entry::Play {
                    reference: "pollen/emotions/welcoming1".to_owned(),
                    speed: 1.0,
                },
                Entry::Gap { ms: 300 },
                Entry::Play {
                    reference: "pod/nod-twice".to_owned(),
                    speed: 1.25,
                },
            ]
        );
    }

    #[test]
    fn wrong_version_and_kind_are_refused() {
        let bad = SequenceDoc {
            version: 7,
            ..doc()
        };
        assert_eq!(
            Sequence::from_doc(bad),
            Err(SequenceError::UnsupportedVersion { version: 7 })
        );

        let bad = SequenceDoc {
            kind: "clip".to_owned(),
            ..doc()
        };
        assert_eq!(
            Sequence::from_doc(bad),
            Err(SequenceError::WrongKind {
                kind: "clip".to_owned()
            })
        );
    }

    #[test]
    fn names_and_references_are_held_to_the_charset() {
        let bad = SequenceDoc {
            name: "Pod/Greet".to_owned(),
            ..doc()
        };
        assert_eq!(
            Sequence::from_doc(bad),
            Err(SequenceError::Name {
                name: "Pod/Greet".to_owned(),
                source: NameError::BadChar { ch: 'P' },
            })
        );

        let mut bad = doc();
        bad.entries[2].reference = Some("pod/Nod".to_owned());
        assert_eq!(
            Sequence::from_doc(bad),
            Err(SequenceError::RefName {
                entry: 2,
                name: "pod/Nod".to_owned(),
                source: NameError::BadChar { ch: 'N' },
            })
        );
    }

    #[test]
    fn an_entry_is_a_reference_or_a_gap_and_not_both() {
        let mut bad = doc();
        bad.entries[1].reference = Some("pod/x".to_owned());
        assert_eq!(
            Sequence::from_doc(bad),
            Err(SequenceError::RefAndGap { entry: 1 })
        );

        let mut bad = doc();
        bad.entries[1].gap_ms = None;
        assert_eq!(
            Sequence::from_doc(bad),
            Err(SequenceError::NeitherRefNorGap { entry: 1 })
        );
    }

    #[test]
    fn a_gap_carries_no_speed_and_no_zero_duration() {
        let mut bad = doc();
        bad.entries[1].speed = Some(2.0);
        assert_eq!(
            Sequence::from_doc(bad),
            Err(SequenceError::SpeedOnGap { entry: 1 })
        );

        let mut bad = doc();
        bad.entries[1].gap_ms = Some(0);
        assert_eq!(
            Sequence::from_doc(bad),
            Err(SequenceError::ZeroGap { entry: 1 })
        );
    }

    #[test]
    fn entry_speed_must_be_a_number_but_is_not_bounded_here() {
        for bad_speed in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            let mut bad = doc();
            bad.entries[2].speed = Some(bad_speed);
            assert!(
                matches!(
                    Sequence::from_doc(bad),
                    Err(SequenceError::Speed { entry: 2, .. })
                ),
                "speed {bad_speed} should be refused"
            );
        }

        // Out of the global invocation bounds on its own, yet legal here: a
        // fast entry nested inside a slow one flattens to something ordinary,
        // and only the flattened speed is bounded.
        let mut fast = doc();
        fast.entries[2].speed = Some(4.0);
        assert!(Sequence::from_doc(fast).is_ok());
    }

    #[test]
    fn empty_entry_list_is_refused() {
        let bad = SequenceDoc {
            entries: vec![],
            ..doc()
        };
        assert_eq!(Sequence::from_doc(bad), Err(SequenceError::NoEntries));
    }

    #[test]
    fn unknown_keys_are_refused() {
        let json = r#"{
            "version": 1, "kind": "sequence", "name": "pod/greet",
            "entries": [{"ref": "pod/x", "repeat": 3}]
        }"#;
        assert!(matches!(
            Sequence::from_json(json),
            Err(SequenceError::Malformed { .. })
        ));
    }

    #[test]
    fn json_round_trips_through_the_document() {
        let sequence = Sequence::from_doc(doc()).expect("well-formed");
        let json = serde_json::to_string(&sequence.to_doc()).expect("serialisable");
        let reloaded = Sequence::from_json(&json).expect("round-trips");
        assert_eq!(reloaded, sequence);
    }

    #[test]
    fn a_gap_document_omits_the_reference_keys() {
        let sequence = Sequence::from_doc(doc()).expect("well-formed");
        let json = serde_json::to_string(&sequence.to_doc().entries[1]).expect("serialisable");
        assert_eq!(json, r#"{"gap_ms":300}"#);
    }
}
