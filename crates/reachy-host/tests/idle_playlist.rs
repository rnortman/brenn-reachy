//! The committed idle playlist, read through the loader the host runs, against
//! the committed name table.
//!
//! It fails at commit what would otherwise be a `voice_host` that exits at
//! start on a unit: the launcher entry names this file, and a playlist the host
//! cannot load is a refusal before any port is bound. Both files arrive through
//! runfiles, and the environment variables name them beside the `data`
//! attribute that supplies them. It also holds the loop's stow pace and
//! opening pose to the committed library.

use std::collections::HashSet;
use std::path::PathBuf;

use reachy_host::Idle;
use reachy_host::idle::EXCLUDED_PREFIXES;

/// A measured fact about the linkage: these four clips drive the head into the
/// body. Held by name until an envelope replaces the list.
/// TODO(head-body-interference)
const BODY_DIVING: [&str; 4] = [
    "pollen/emotions/mini-deep-sleep",
    "pollen/emotions/wake-mini-up",
    "pollen/emotions/toc-toc-toc",
    "pollen/emotions/waiting",
];

/// The committed playlist's path, out of the environment the target sets.
fn playlist() -> PathBuf {
    PathBuf::from(std::env::var("IDLE_PLAYLIST").expect("the target names the idle playlist"))
}

/// The library name table the payload carries, out of the same environment.
fn names() -> PathBuf {
    PathBuf::from(std::env::var("CLIP_NAMES").expect("the target names the shipped name table"))
}

/// The committed playlist, loaded as the host loads it, and the table it
/// resolved through.
fn loaded() -> (reachy_host::Playlist, reachy_edge::MotionTable) {
    let (table, _) = reachy_host::check::name_tables(&names())
        .unwrap_or_else(|detail| panic!("the committed name table {detail}"));
    let list = Idle::load(&playlist(), &table)
        .unwrap_or_else(|error| panic!("the committed playlist is refused: {error}"));
    (list, table)
}

#[test]
fn the_committed_playlist_loads_against_the_committed_table() {
    let (list, _) = loaded();
    assert!(list.len() >= 2, "{} motions", list.len());
}

#[test]
fn no_entry_is_a_probe_or_bench_motion_or_listed_twice() {
    // The loader already enforces both; this says it about this file.
    let (list, _) = loaded();
    let names: Vec<&str> = list.entries().map(|(name, _)| name).collect();
    for name in &names {
        assert!(
            !EXCLUDED_PREFIXES
                .iter()
                .any(|prefix| name.starts_with(prefix)),
            "{name} is a probe or bench motion",
        );
    }
    let distinct: HashSet<&str> = names.iter().copied().collect();
    assert_eq!(distinct.len(), names.len(), "a name is listed twice");
}

#[test]
fn every_entry_blends_out() {
    let (list, _) = loaded();
    for (name, entry) in list.entries() {
        assert!(
            entry.window.blend_out_ms > 0,
            "{name} has no blend-out, and the seam arithmetic assumes a ramp-out",
        );
    }
}

#[test]
fn the_body_diving_clips_are_absent() {
    let (list, table) = loaded();
    for name in BODY_DIVING {
        // Held in the table, so a renamed clip cannot pass this vacuously.
        assert!(
            table.resolve(name).is_some(),
            "{name} is not in the name table; the exclusion no longer names a clip",
        );
        assert!(
            list.entries().all(|(listed, _)| listed != name),
            "{name} drives the head into the body and is in the playlist",
        );
    }
}

#[test]
fn the_loop_is_paced_as_the_committed_library_paces_it() {
    let (_, poses) = reachy_host::check::name_tables(&names())
        .unwrap_or_else(|detail| panic!("the committed name table {detail}"));
    assert_eq!(
        u64::from(poses.stow().duration_ms),
        reachy_host::idle::STOW_DURATION_MS,
        "every loop script's timeout is sized on this stow pace; a longer stow has the edge refuse every one",
    );
    assert!(
        poses.resolve(reachy_host::idle::NEUTRAL_POSE).is_some(),
        "the committed library holds the pose every loop script opens on",
    );
}
