//! The names sidecar: what an asset is called, which index invokes it, and the
//! numbers a timeline is built with.
//!
//! A script names a motion and a pose; the wire and the schedule carry indices.
//! The sidecar is the join, emitted beside the two libraries by the same tool
//! that numbers them, so the tables and the numbering the boxes load come out
//! of one walk over the assets. An asset's identity is its position in that
//! numbering, which is why nothing here re-derives an index: a second opinion
//! about the order is a wrong motion played or a wrong pose commanded.
//!
//! One document, two tables, one read: [`parse`]. Reading it twice would be two
//! opinions about which emit this process is holding.
//!
//! A pose carries the pace of a move to it — what a step that states none runs
//! on, and what the closing stow the compile appends is given. The stow is held
//! apart from the names as well as among them, because every schedule ends
//! there and nothing that stows should have to know a number.
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

/// How many poses the library message the box loads holds, and so the highest
/// index a schedule can carry.
///
/// Restated here for [`MAX_MOTIONS`]'s reason, and joined to
/// `reachy_poses::config::MAX_POSES` in `cogs/edge_caps_test.rs`.
pub const MAX_POSES: usize = 32;

/// The name the library reserves for where the machine rests.
///
/// The wire's own spelling, not a second one: a script closes with this name,
/// the compile recognises it, and the library must hold it. Restated as a
/// mirror nowhere — [`motion_proto::STOW_POSE`] is the authority this crate
/// already depends on.
const STOW_POSE: &str = motion_proto::STOW_POSE;

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
        Self::of_rows(sidecar.motions)
    }

    /// The table those sidecar rows state, for a caller that has read the
    /// document once for both of its tables.
    fn of_rows(rows: Vec<MotionRow>) -> Result<Self, SidecarError> {
        let mut by_name = BTreeMap::new();
        let mut by_id: BTreeMap<u16, String> = BTreeMap::new();
        for row in rows {
            let motion_id = asset_id(row.motion_id, MAX_MOTIONS).map_err(|fault| match fault {
                IdFault::Unrepresentable => SidecarError::MotionIdUnrepresentable {
                    name: row.name.clone(),
                    motion_id: row.motion_id,
                },
                IdFault::PastLibrary(motion_id) => SidecarError::MotionIdPastLibrary {
                    name: row.name.clone(),
                    motion_id,
                },
            })?;
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
            poses: Vec::new(),
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

/// What one pose name resolves to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PoseEntry {
    /// The index the schedule carries, which is the pose's position in the
    /// library the box loads.
    pub pose_id: u16,
    /// How long a move to this pose takes when the step that commands it states
    /// no pace of its own, milliseconds. The library's own pace: what every
    /// move this machine plans for itself runs on, the closing stow included.
    pub duration_ms: u32,
}

/// Every base pose the deployed library holds, by name.
///
/// The stow is held apart as well as by name: every schedule ends at it, and
/// what appends that step must not depend on a name the library happens to
/// carry twice or not at all.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PoseTable {
    by_name: BTreeMap<String, PoseEntry>,
    stow: PoseEntry,
}

impl PoseTable {
    /// What `name` commands, or `None` if this library does not hold it.
    #[must_use]
    pub fn resolve(&self, name: &str) -> Option<PoseEntry> {
        self.by_name.get(name).copied()
    }

    /// Where the machine rests: the pose every schedule ends at.
    #[must_use]
    pub fn stow(&self) -> PoseEntry {
        self.stow
    }

    /// A table built from `entries`, for a caller that has the numbering in
    /// hand already — a test fixture, or a tool that just emitted one.
    ///
    /// # Errors
    ///
    /// [`SidecarError::DuplicateName`] for two entries under one name, and
    /// [`SidecarError::NoStow`] if none carries the reserved stow name: a table
    /// without one compiles no schedule at all, so both are refused where the
    /// table is built rather than at the first script. The duplicate is refused
    /// here rather than in the sidecar walk so that every caller gets it — a
    /// collection that silently kept the last row would resolve a name to
    /// whichever entry was read last, which is a wrong pose commanded.
    pub fn of(
        entries: impl IntoIterator<Item = (String, PoseEntry)>,
    ) -> Result<Self, SidecarError> {
        let mut by_name: BTreeMap<String, PoseEntry> = BTreeMap::new();
        for (name, entry) in entries {
            if by_name.insert(name.clone(), entry).is_some() {
                return Err(SidecarError::DuplicateName { name });
            }
        }
        let stow = by_name
            .get(STOW_POSE)
            .copied()
            .ok_or(SidecarError::NoStow)?;
        Ok(Self { by_name, stow })
    }

    /// Every pose the table holds, name and entry, in name order.
    pub fn entries(&self) -> impl Iterator<Item = (&str, &PoseEntry)> {
        self.by_name
            .iter()
            .map(|(name, entry)| (name.as_str(), entry))
    }

    /// How many poses the table holds.
    #[must_use]
    pub fn len(&self) -> usize {
        self.by_name.len()
    }

    /// Whether it holds none. Never true of a table that was built: one without
    /// the stow is refused.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.by_name.is_empty()
    }
}

/// Both tables the sidecar states, read in one pass.
///
/// One file, one parse: the motions and the poses are numbered by one walk over
/// the assets and delivered in one document, and a second read of it is a
/// second opinion about which emit this process is holding.
///
/// # Errors
///
/// [`SidecarError`] for text that is not the sidecar, or for either table's own
/// refusals.
pub fn parse(text: &str) -> Result<(MotionTable, PoseTable), SidecarError> {
    let sidecar: Sidecar = serde_json::from_str(text).map_err(|error| SidecarError::Malformed {
        detail: error.to_string(),
    })?;
    let motions = MotionTable::of_rows(sidecar.motions)?;
    let poses = poses_of_rows(sidecar.poses)?;
    Ok((motions, poses))
}

/// The pose table those sidecar rows state.
fn poses_of_rows(rows: Vec<PoseRow>) -> Result<PoseTable, SidecarError> {
    let mut entries = Vec::with_capacity(rows.len());
    let mut by_id: BTreeMap<u16, String> = BTreeMap::new();
    for row in rows {
        let pose_id = asset_id(row.pose_id, MAX_POSES).map_err(|fault| match fault {
            IdFault::Unrepresentable => SidecarError::PoseIdUnrepresentable {
                name: row.name.clone(),
                pose_id: row.pose_id,
            },
            IdFault::PastLibrary(pose_id) => SidecarError::PoseIdPastLibrary {
                name: row.name.clone(),
                pose_id,
            },
        })?;
        if let Some(also) = by_id.insert(pose_id, row.name.clone()) {
            return Err(SidecarError::DuplicatePoseId {
                pose_id,
                name: row.name,
                also,
            });
        }
        // The pace is what a schedule's closing stow is given and what a step
        // that states none runs on, so a zero is a step of no span and anything
        // past the wire's ceiling is a move no timeline could hold.
        if row.duration_ms == 0 || u64::from(row.duration_ms) > motion_proto::MAX_TIMEOUT_MS {
            return Err(SidecarError::PoseUnpaced {
                name: row.name,
                duration_ms: row.duration_ms,
            });
        }
        entries.push((
            row.name,
            PoseEntry {
                pose_id,
                duration_ms: row.duration_ms,
            },
        ));
    }
    PoseTable::of(entries)
}

/// A raw asset index from a sidecar row, as the index a table carries.
///
/// The numbering rule both tables are built under, stated once: the field a
/// script carries is sixteen bits wide, and an index past the deployed
/// library's capacity reaches no asset. Each table names its own refusal for
/// each of the two, because a reader of one is looking for the file that says
/// which asset it is about.
fn asset_id(raw: u32, max: usize) -> Result<u16, IdFault> {
    let id = u16::try_from(raw).map_err(|_| IdFault::Unrepresentable)?;
    if usize::from(id) >= max {
        return Err(IdFault::PastLibrary(id));
    }
    Ok(id)
}

/// What [`asset_id`] found wrong, for the caller to say it in its own words.
enum IdFault {
    /// Wider than the field a script carries.
    Unrepresentable,
    /// Representable, and past the library's capacity.
    PastLibrary(u16),
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

    /// Two assets under one name. Refused rather than resolved by order: a
    /// lookup whose answer depends on which row was read last is a wrong asset
    /// invoked, silently.
    #[error("`{name}` is named twice; a name resolves to one asset")]
    DuplicateName {
        /// The name in question.
        name: String,
    },

    /// A pose index past what the schedule field carries. The emitter cannot
    /// produce one; a hand-edited sidecar can, and a truncated index commands a
    /// different pose.
    #[error("pose `{name}` is numbered {pose_id}, which no schedule field carries")]
    PoseIdUnrepresentable {
        /// The pose that carried it.
        name: String,
        /// The index the sidecar stated.
        pose_id: u32,
    },

    /// A pose index the deployed library does not reach. The mover refuses the
    /// same index and the fault ladder answers the refusal; that is a fault
    /// raised for something in the sidecar, so it is caught here, where the
    /// file can be pointed at.
    #[error("pose `{name}` is numbered {pose_id}; the library holds {MAX_POSES} poses")]
    PoseIdPastLibrary {
        /// The pose that carried it.
        name: String,
        /// The index the sidecar stated.
        pose_id: u16,
    },

    /// Two names over one pose index, for [`SidecarError::DuplicateMotionId`]'s
    /// reason: one of them commands a pose nobody asked for.
    #[error("poses `{name}` and `{also}` are both numbered {pose_id}")]
    DuplicatePoseId {
        /// The index in question.
        pose_id: u16,
        /// The pose that arrived second.
        name: String,
        /// The one that already held the index.
        also: String,
    },

    /// A pose whose stated pace is no move: zero, or longer than any script
    /// could wait for. It is the pace every move this machine plans for itself
    /// runs on, so a schedule built from it would carry a step of no span or
    /// one past the wire's own ceiling.
    #[error(
        "pose `{name}` paces its move at {duration_ms} ms; a pace is 1..={} ms",
        motion_proto::MAX_TIMEOUT_MS
    )]
    PoseUnpaced {
        /// The pose that carried it.
        name: String,
        /// The pace the sidecar stated.
        duration_ms: u32,
    },

    /// No pose named `stow`. Every schedule ends at it — the compile appends
    /// one where a script does not — so a library without it compiles nothing
    /// and is refused where it is read.
    #[error("the library holds no pose named `{STOW_POSE}`, which every schedule ends at")]
    NoStow,
}

/// The sidecar's JSON shape: the two tables this crate reads out of it.
///
/// No `deny_unknown_fields`: the clips table sits beside these, and the emitter
/// may add a field before a reader knows it. The poses default to none so that
/// the shape a harness writes — motions alone, [`MotionTable::to_sidecar`] —
/// still reads as the document it is.
#[derive(Deserialize, Serialize)]
struct Sidecar {
    motions: Vec<MotionRow>,
    #[serde(default, skip_serializing_if = "Vec::is_empty")]
    poses: Vec<PoseRow>,
}

#[derive(Deserialize, Serialize)]
struct MotionRow {
    motion_id: u32,
    name: String,
    duration_ms: u64,
    blend_out_ms: u64,
}

#[derive(Deserialize, Serialize)]
struct PoseRow {
    pose_id: u32,
    name: String,
    duration_ms: u32,
}

#[cfg(test)]
mod tests {
    use super::{MAX_MOTIONS, MAX_POSES, MotionTable, PoseTable, SidecarError, parse};

    const SIDECAR: &str = r#"{
  "clips": [{"clip_id": 0, "name": "bench/nod"}],
  "motions": [
    {"motion_id": 0, "name": "bench/nod", "duration_ms": 1000, "blend_out_ms": 60},
    {"motion_id": 1, "name": "bench/tour", "duration_ms": 4500, "blend_out_ms": 120}
  ],
  "poses": [
    {"pose_id": 0, "name": "neutral", "duration_ms": 800},
    {"pose_id": 1, "name": "peek", "duration_ms": 800},
    {"pose_id": 2, "name": "stow", "duration_ms": 2000}
  ]
}"#;

    /// One sidecar with the poses half rewritten, so a case states only the
    /// rows it is about.
    fn with_poses(poses: &str) -> String {
        format!(r#"{{"motions": [], "poses": {poses}}}"#)
    }

    /// The pose table the sidecar states, or the refusal.
    fn poses(text: &str) -> Result<PoseTable, SidecarError> {
        parse(text).map(|(_, poses)| poses)
    }

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

    /// One document, both tables: the motions and the poses are numbered by one
    /// walk over the assets, and read by one pass over what that walk wrote.
    #[test]
    fn one_read_yields_both_tables() {
        let (motions, poses) = parse(SIDECAR).expect("the emitter's own shape");
        assert_eq!(motions.len(), 2);
        assert_eq!(poses.len(), 3);
        let peek = poses.resolve("peek").expect("a pose of the library");
        assert_eq!(
            (peek.pose_id, peek.duration_ms),
            (1, 800),
            "a pose states the index a schedule carries and the pace a step that states none runs on",
        );
        assert_eq!(poses.resolve("crouch"), None);
        let stow = poses.stow();
        assert_eq!((stow.pose_id, stow.duration_ms), (2, 2000));
        assert_eq!(
            poses.resolve("stow"),
            Some(stow),
            "the stow is one row, reachable by name and as itself",
        );
        let visited: Vec<&str> = poses.entries().map(|(name, _)| name).collect();
        assert_eq!(visited, vec!["neutral", "peek", "stow"], "in name order");
    }

    /// Every schedule ends stowed, so a library with no stow compiles nothing:
    /// refused where the file can be pointed at rather than at the first
    /// script.
    #[test]
    fn a_library_with_no_stow_is_not_a_table() {
        assert_eq!(
            poses(&with_poses(
                r#"[{"pose_id": 0, "name": "neutral", "duration_ms": 800}]"#
            )),
            Err(SidecarError::NoStow),
        );
        assert_eq!(poses(&with_poses("[]")), Err(SidecarError::NoStow));
    }

    #[test]
    fn a_pose_index_no_schedule_or_library_carries_is_refused() {
        assert_eq!(
            poses(&with_poses(
                r#"[{"pose_id": 70000, "name": "stow", "duration_ms": 2000}]"#
            )),
            Err(SidecarError::PoseIdUnrepresentable {
                name: "stow".to_owned(),
                pose_id: 70_000,
            })
        );
        let past = MAX_POSES;
        assert_eq!(
            poses(&with_poses(&format!(
                r#"[{{"pose_id": {past}, "name": "stow", "duration_ms": 2000}}]"#
            ))),
            Err(SidecarError::PoseIdPastLibrary {
                name: "stow".to_owned(),
                pose_id: u16::try_from(past).expect("an index inside the schedule's field"),
            }),
            "the mover refuses this index and the ladder answers it; the sidecar is at fault",
        );
        let last = MAX_POSES - 1;
        assert_eq!(
            poses(&with_poses(&format!(
                r#"[{{"pose_id": {last}, "name": "stow", "duration_ms": 2000}}]"#
            )))
            .expect("the last pose the library holds")
            .stow()
            .pose_id,
            u16::try_from(last).expect("an index inside the schedule's field"),
        );
    }

    #[test]
    fn one_pose_index_resolves_from_one_name_and_one_name_from_one_pose() {
        assert_eq!(
            poses(&with_poses(
                r#"[{"pose_id": 0, "name": "stow", "duration_ms": 2000},
                    {"pose_id": 0, "name": "neutral", "duration_ms": 800}]"#
            )),
            Err(SidecarError::DuplicatePoseId {
                pose_id: 0,
                name: "neutral".to_owned(),
                also: "stow".to_owned(),
            }),
        );
        assert_eq!(
            poses(&with_poses(
                r#"[{"pose_id": 0, "name": "stow", "duration_ms": 2000},
                    {"pose_id": 1, "name": "stow", "duration_ms": 800}]"#
            )),
            Err(SidecarError::DuplicateName {
                name: "stow".to_owned(),
            }),
        );
    }

    /// The pace is what the closing stow is given and what a step that states
    /// none runs on, so the bounds a schedule's own span has are applied here.
    #[test]
    fn a_pace_that_is_no_move_is_refused() {
        for duration_ms in [
            0,
            u32::try_from(motion_proto::MAX_TIMEOUT_MS + 1).expect("a bound"),
        ] {
            assert_eq!(
                poses(&with_poses(&format!(
                    r#"[{{"pose_id": 0, "name": "stow", "duration_ms": {duration_ms}}}]"#
                ))),
                Err(SidecarError::PoseUnpaced {
                    name: "stow".to_owned(),
                    duration_ms,
                }),
            );
        }
        let ceiling = u32::try_from(motion_proto::MAX_TIMEOUT_MS).expect("a bound");
        assert!(
            poses(&with_poses(&format!(
                r#"[{{"pose_id": 0, "name": "stow", "duration_ms": {ceiling}}}]"#
            )))
            .is_ok(),
            "the ceiling itself is a pace a schedule can hold",
        );
    }

    /// A sidecar this build reads for its motions and no poses is not a library
    /// an edge can compile against: the motions still read, and the pose half
    /// refuses.
    #[test]
    fn a_sidecar_without_the_poses_table_still_yields_its_motions() {
        assert_eq!(
            MotionTable::from_sidecar(SIDECAR)
                .expect("the emitter's own shape")
                .len(),
            2,
        );
        let written = MotionTable::from_sidecar(SIDECAR)
            .expect("the emitter's own shape")
            .to_sidecar();
        assert!(
            !written.contains("poses"),
            "the harness writes the table it was asked for and no half it does not hold: {written}",
        );
        assert_eq!(poses(&written), Err(SidecarError::NoStow));
    }

    /// A table built from a numbering in hand holds the stow apart and by
    /// name, exactly as one read out of a sidecar does.
    #[test]
    fn a_table_built_from_a_numbering_holds_the_stow_both_ways() {
        let table = crate::fixture::poses();
        assert_eq!(table.len(), 3);
        assert!(!table.is_empty());
        assert_eq!(
            table.stow(),
            table.resolve(super::STOW_POSE).expect("the stow")
        );
        assert_eq!(table.stow().pose_id, crate::fixture::STOW_ID);
        assert!(table.resolve(crate::fixture::NEUTRAL_POSE).is_some());
    }
}
