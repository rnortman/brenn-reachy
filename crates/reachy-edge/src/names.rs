//! The names sidecar: what a motion is called, which index invokes it, and how
//! long it occupies a timeline.
//!
//! A script names a motion; the wire carries an index. The sidecar is the join,
//! emitted beside the clip library by the same tool that numbers it, so the
//! table and the numbering the box loads come out of one walk over the assets.
//! An asset's identity is its position in that numbering, which is why nothing
//! here re-derives an index: a second opinion about the order is a wrong motion
//! played.
//!
//! Each motion also carries the two numbers the window arithmetic needs. Two
//! rather than one because they scale differently under an invocation speed:
//! the motion's own clock is what the speed divides, while the blend-out that
//! follows runs on the wall clock at any speed — it is the ramp that keeps the
//! machine's per-tick bounds satisfied, and playing a motion faster must not
//! shorten it.
//!
//! Parsing, and the one writer beside it. The sidecar arrives as text — a
//! runfile beside the payload — and reading the file is the host's business;
//! writing one is the harness's, which asks for a table back in the shape this
//! module reads, so both directions come off the one row definition here.

use std::collections::BTreeMap;

use motion_proto::PlayWindow;
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// How many motions the library message the box loads holds, and so the
/// highest index a script may invoke.
///
/// Restated here rather than imported: this crate parses text and does not link
/// the clip library. `cogs/edge_caps_test.rs` joins the two numbers, so a
/// library that grows fails there rather than at the session, which refuses an
/// index past its own table and would blame the sender for the sidecar.
pub const MAX_MOTIONS: usize = 128;

/// The name prefix that marks a motion as an instrument rather than content.
///
/// A probe steps a joint to a pose in one frame and holds it, so that a hold
/// can be judged after an arrival the planner never makes. It is played one at
/// a time, and it is not part of the tour: the tour is the content the recorded
/// fixtures come off, and a probe among them would put a step goal into the
/// run those fixtures pin the detector's screen against. The prefix is the
/// whole of the rule — a sender leaves these motions out of a tour, a probe run
/// names one of them, and an analyzer handed a table of them alone is judging a
/// probe run.
pub const PROBE_PREFIX: &str = "probe/";

/// What one motion name resolves to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MotionEntry {
    /// The index the wire carries, which is the motion's position in the
    /// library the box loads.
    pub motion_id: u16,
    /// How long the motion occupies a timeline, before an invocation speed is
    /// applied to the half of it that scales.
    pub window: PlayWindow,
}

/// Every motion the deployed library holds, by name.
///
/// The clips table in the same sidecar is read past: a clip id is what a
/// motion's segments name internally, and no script ever carries one.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct MotionTable {
    by_name: BTreeMap<String, MotionEntry>,
}

impl MotionTable {
    /// The table the sidecar text states.
    ///
    /// # Errors
    ///
    /// [`SidecarError`] for text that is not the sidecar, a motion index no
    /// wire field could carry or the deployed library does not reach, two
    /// motions under one name — a table whose lookups would depend on which row
    /// won — or two names over one index, which is the same wrong motion played
    /// from the other direction.
    pub fn from_sidecar(text: &str) -> Result<Self, SidecarError> {
        let sidecar: Sidecar =
            serde_json::from_str(text).map_err(|error| SidecarError::Malformed {
                detail: error.to_string(),
            })?;
        let mut by_name = BTreeMap::new();
        let mut by_id: BTreeMap<u16, String> = BTreeMap::new();
        for row in sidecar.motions {
            let motion_id = u16::try_from(row.motion_id).map_err(|_| {
                SidecarError::MotionIdUnrepresentable {
                    name: row.name.clone(),
                    motion_id: row.motion_id,
                }
            })?;
            if usize::from(motion_id) >= MAX_MOTIONS {
                return Err(SidecarError::MotionIdPastLibrary {
                    name: row.name,
                    motion_id,
                });
            }
            if let Some(also) = by_id.insert(motion_id, row.name.clone()) {
                return Err(SidecarError::DuplicateMotionId {
                    motion_id,
                    name: row.name,
                    also,
                });
            }
            let entry = MotionEntry {
                motion_id,
                window: PlayWindow {
                    duration_ms: row.duration_ms,
                    blend_out_ms: row.blend_out_ms,
                },
            };
            if by_name.insert(row.name.clone(), entry).is_some() {
                return Err(SidecarError::DuplicateName { name: row.name });
            }
        }
        Ok(Self { by_name })
    }

    /// A table built from `entries`, for a caller that has the numbering in
    /// hand already — a test fixture, or a tool that just emitted one.
    #[must_use]
    pub fn of(entries: impl IntoIterator<Item = (String, MotionEntry)>) -> Self {
        Self {
            by_name: entries.into_iter().collect(),
        }
    }

    /// What `name` invokes, or `None` if this library does not hold it.
    #[must_use]
    pub fn resolve(&self, name: &str) -> Option<MotionEntry> {
        self.by_name.get(name).copied()
    }

    /// Every motion the table holds, name and entry, in name order.
    ///
    /// For a caller that has to visit the whole library rather than look one
    /// motion up. Reading the table this crate already parsed rather than the
    /// sidecar again avoids a second opinion about the numbering, which is the
    /// thing this module exists to prevent.
    pub fn entries(&self) -> impl Iterator<Item = (&str, &MotionEntry)> {
        self.by_name
            .iter()
            .map(|(name, entry)| (name.as_str(), entry))
    }

    /// This table as sidecar text, in the shape [`Self::from_sidecar`] reads.
    ///
    /// What a harness hands an analyzer, and what a fetched run directory
    /// carries: the table a run was asked for, written by the same row
    /// definition that parses one, so a field added to the shape is added to
    /// both directions at once. The clips table of an emitted library's own
    /// sidecar is not restated — nothing that reads a sidecar reads it, and a
    /// clip id is not a name a script carries.
    #[must_use]
    pub fn to_sidecar(&self) -> String {
        let sidecar = Sidecar {
            motions: self
                .by_name
                .iter()
                .map(|(name, entry)| MotionRow {
                    motion_id: u32::from(entry.motion_id),
                    name: name.clone(),
                    duration_ms: entry.window.duration_ms,
                    blend_out_ms: entry.window.blend_out_ms,
                })
                .collect(),
        };
        serde_json::to_string_pretty(&sidecar).expect("a table of numbers and names serializes")
    }

    /// Whether every motion this table holds is a [`PROBE_PREFIX`] instrument,
    /// and it holds at least one.
    ///
    /// What tells a probe run's table from a content tour's, for a reader that
    /// was handed the table a run was asked for and has to know which kind of
    /// run it is judging. An empty table is neither.
    #[must_use]
    pub fn probes_only(&self) -> bool {
        !self.by_name.is_empty()
            && self
                .by_name
                .keys()
                .all(|name| name.starts_with(PROBE_PREFIX))
    }

    /// How many motions the table holds.
    #[must_use]
    pub fn len(&self) -> usize {
        self.by_name.len()
    }

    /// Whether it holds none.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.by_name.is_empty()
    }
}

/// Why a sidecar did not yield a table.
#[derive(Clone, Debug, PartialEq, Eq, Error)]
pub enum SidecarError {
    /// The text is not the sidecar: not JSON, or JSON without the motions table
    /// in the shape the emitter writes.
    #[error("the names sidecar is not readable: {detail}")]
    Malformed {
        /// What the parser said.
        detail: String,
    },

    /// A motion index past what the wire field carries. The emitter cannot
    /// produce one; a hand-edited sidecar can, and a truncated index invokes a
    /// different motion.
    #[error("motion `{name}` is numbered {motion_id}, which no script field carries")]
    MotionIdUnrepresentable {
        /// The motion that carried it.
        name: String,
        /// The index the sidecar stated.
        motion_id: u32,
    },

    /// A motion index the deployed library does not reach. The session screens
    /// the same index against its own table and refuses the whole script; that
    /// refusal names the sender for a fault that is in the sidecar, so it is
    /// caught here, where the file can be pointed at.
    #[error("motion `{name}` is numbered {motion_id}; the library holds {MAX_MOTIONS} motions")]
    MotionIdPastLibrary {
        /// The motion that carried it.
        name: String,
        /// The index the sidecar stated.
        motion_id: u16,
    },

    /// Two names over one index. The emitter's one walk over the assets cannot
    /// produce it; a hand-edited or merged sidecar can, and then one of the two
    /// names invokes a motion nobody asked for, silently.
    #[error("motions `{name}` and `{also}` are both numbered {motion_id}")]
    DuplicateMotionId {
        /// The index in question.
        motion_id: u16,
        /// The motion that arrived second.
        name: String,
        /// The one that already held the index.
        also: String,
    },

    /// Two motions under one name. Refused rather than resolved by order: a
    /// lookup whose answer depends on which row was read last is a wrong motion
    /// played, silently.
    #[error("motion `{name}` is named twice; a name resolves to one motion")]
    DuplicateName {
        /// The name in question.
        name: String,
    },
}

/// The sidecar's JSON shape, motions half only.
///
/// No `deny_unknown_fields`: the clips table sits beside this one, and the
/// emitter may add a field before a reader knows it.
#[derive(Deserialize, Serialize)]
struct Sidecar {
    motions: Vec<MotionRow>,
}

#[derive(Deserialize, Serialize)]
struct MotionRow {
    motion_id: u32,
    name: String,
    duration_ms: u64,
    blend_out_ms: u64,
}

#[cfg(test)]
mod tests {
    use super::{MAX_MOTIONS, MotionTable, SidecarError};

    const SIDECAR: &str = r#"{
  "clips": [{"clip_id": 0, "name": "bench/nod"}],
  "motions": [
    {"motion_id": 0, "name": "bench/nod", "duration_ms": 1000, "blend_out_ms": 60},
    {"motion_id": 1, "name": "bench/tour", "duration_ms": 4500, "blend_out_ms": 120}
  ]
}"#;

    /// Which kind of run a table describes, which is what an analyzer handed
    /// one asks: a table of instruments alone is a probe run's, a table with
    /// any content in it is a tour's, and an empty table is neither.
    #[test]
    fn a_table_of_instruments_alone_says_it_is_a_probe_runs() {
        let probe = MotionTable::from_sidecar(
            r#"{"motions": [
                {"motion_id": 66, "name": "probe/antenna-step-a", "duration_ms": 19520,
                 "blend_out_ms": 200}
            ]}"#,
        )
        .expect("the emitter's own shape");
        assert!(probe.probes_only());
        let mixed = MotionTable::from_sidecar(
            r#"{"motions": [
                {"motion_id": 0, "name": "bench/nod", "duration_ms": 1000, "blend_out_ms": 60},
                {"motion_id": 66, "name": "probe/antenna-step-a", "duration_ms": 19520,
                 "blend_out_ms": 200}
            ]}"#,
        )
        .expect("the emitter's own shape");
        assert!(!mixed.probes_only());
        assert!(
            !MotionTable::from_sidecar(SIDECAR)
                .expect("the emitter's own shape")
                .probes_only()
        );
        assert!(!MotionTable::default().probes_only());
    }

    #[test]
    fn the_sidecar_states_an_index_and_a_window_per_motion() {
        let table = MotionTable::from_sidecar(SIDECAR).expect("the emitter's own shape");
        assert_eq!(table.len(), 2);
        let tour = table
            .resolve("bench/tour")
            .expect("a motion of the library");
        assert_eq!(tour.motion_id, 1);
        assert_eq!(tour.window.duration_ms, 4500);
        assert_eq!(tour.window.blend_out_ms, 120);
        assert_eq!(table.resolve("bench/absent"), None);
    }

    /// The two directions are one shape: a table written out and read back is
    /// the table it started as.
    ///
    /// The case that makes the writer's shape the reader's obligation rather
    /// than a second opinion about the file — a field the parser requires and
    /// the emitter stopped writing fails here, in this crate, rather than on a
    /// device run judged against a table the analyzer could not read.
    #[test]
    fn a_table_written_out_reads_back_as_itself() {
        let table = MotionTable::from_sidecar(SIDECAR).expect("the emitter's own shape");
        let written = table.to_sidecar();
        let read = MotionTable::from_sidecar(&written).expect("this module's own writing");
        assert_eq!(read, table, "{written}");
        // The row's own field names, not a shape only this crate can read.
        for field in [
            "motions",
            "motion_id",
            "name",
            "duration_ms",
            "blend_out_ms",
        ] {
            assert!(written.contains(field), "{written}");
        }
        // The clips half is not restated: nothing reads it.
        assert!(!written.contains("clip_id"), "{written}");
    }

    /// The documented order, over a table built out of it: a caller visiting
    /// the whole library — a tour, a listing — reads the names sorted and each
    /// with the index the sidecar gave it, and swapping the map underneath for
    /// one that does not sort is a behaviour change rather than a detail.
    #[test]
    fn the_whole_table_comes_back_in_name_order() {
        let text = r#"{"motions": [
            {"motion_id": 0, "name": "pollen/emotions/oops", "duration_ms": 1, "blend_out_ms": 0},
            {"motion_id": 1, "name": "bench/nod", "duration_ms": 2, "blend_out_ms": 0},
            {"motion_id": 2, "name": "pollen/dances/nod", "duration_ms": 3, "blend_out_ms": 0}
        ]}"#;
        let table = MotionTable::from_sidecar(text).expect("a table out of name order");
        let visited: Vec<(&str, u16)> = table
            .entries()
            .map(|(name, entry)| (name, entry.motion_id))
            .collect();
        assert_eq!(
            visited,
            vec![
                ("bench/nod", 1),
                ("pollen/dances/nod", 2),
                ("pollen/emotions/oops", 0),
            ],
        );
    }

    #[test]
    fn the_clips_table_is_read_past_and_unknown_fields_are_tolerated() {
        let text = r#"{"clips": [], "motions": [
            {"motion_id": 0, "name": "a", "duration_ms": 1, "blend_out_ms": 2, "later": 3}
        ], "later": {}}"#;
        let table = MotionTable::from_sidecar(text)
            .expect("a sidecar with a field this build does not know");
        assert_eq!(table.len(), 1);
    }

    #[test]
    fn a_motion_missing_its_window_is_not_a_table() {
        let text = r#"{"motions": [{"motion_id": 0, "name": "a"}]}"#;
        assert!(matches!(
            MotionTable::from_sidecar(text),
            Err(SidecarError::Malformed { .. })
        ));
    }

    #[test]
    fn text_that_is_not_json_is_not_a_table() {
        assert!(matches!(
            MotionTable::from_sidecar("motions"),
            Err(SidecarError::Malformed { .. })
        ));
    }

    #[test]
    fn an_index_past_the_wire_field_is_refused() {
        let text = r#"{"motions": [
            {"motion_id": 70000, "name": "a", "duration_ms": 1, "blend_out_ms": 0}
        ]}"#;
        assert_eq!(
            MotionTable::from_sidecar(text),
            Err(SidecarError::MotionIdUnrepresentable {
                name: "a".to_owned(),
                motion_id: 70_000,
            })
        );
    }

    #[test]
    fn an_index_past_the_deployed_library_is_refused() {
        let last = MAX_MOTIONS - 1;
        let text = format!(
            r#"{{"motions": [
            {{"motion_id": {last}, "name": "a", "duration_ms": 1, "blend_out_ms": 0}}
        ]}}"#
        );
        assert_eq!(
            MotionTable::from_sidecar(&text)
                .expect("the last motion the library holds")
                .resolve("a")
                .expect("the motion just read")
                .motion_id,
            u16::try_from(last).expect("an index inside the wire field"),
        );

        let text = format!(
            r#"{{"motions": [
            {{"motion_id": {MAX_MOTIONS}, "name": "a", "duration_ms": 1, "blend_out_ms": 0}}
        ]}}"#
        );
        assert_eq!(
            MotionTable::from_sidecar(&text),
            Err(SidecarError::MotionIdPastLibrary {
                name: "a".to_owned(),
                motion_id: u16::try_from(MAX_MOTIONS).expect("an index inside the wire field"),
            }),
            "the session refuses this index and blames the sender; the sidecar is at fault",
        );
    }

    #[test]
    fn one_index_resolves_from_one_name() {
        let text = r#"{"motions": [
            {"motion_id": 3, "name": "a", "duration_ms": 1, "blend_out_ms": 0},
            {"motion_id": 3, "name": "b", "duration_ms": 2, "blend_out_ms": 0}
        ]}"#;
        assert_eq!(
            MotionTable::from_sidecar(text),
            Err(SidecarError::DuplicateMotionId {
                motion_id: 3,
                name: "b".to_owned(),
                also: "a".to_owned(),
            }),
            "one of the two names would invoke a motion nobody asked for",
        );
    }

    #[test]
    fn one_name_resolves_to_one_motion() {
        let text = r#"{"motions": [
            {"motion_id": 0, "name": "a", "duration_ms": 1, "blend_out_ms": 0},
            {"motion_id": 1, "name": "a", "duration_ms": 2, "blend_out_ms": 0}
        ]}"#;
        assert_eq!(
            MotionTable::from_sidecar(text),
            Err(SidecarError::DuplicateName {
                name: "a".to_owned()
            })
        );
    }
}
