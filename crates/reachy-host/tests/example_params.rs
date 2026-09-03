//! The example configuration, read by the parser that reads an operator's copy
//! of it on the device.
//!
//! A test of its own because it reads files: the example configuration and the
//! clip name table it names both arrive through runfiles, and the environment
//! variables name them beside the `data` attribute that supplies them. What it
//! proves is that the file an operator copies to start their own is a file this
//! build accepts, and that the path it names is a table this build can resolve
//! names through — the two failures that would otherwise be discovered on a
//! unit.

use std::path::PathBuf;

use reachy_edge::{BODY_CAP_BYTES, MotionTable, STOW_DURATION_MS};

/// The machine the example answers for, which is no machine.
///
/// Pinned rather than merely required to be non-empty: `EdgeConfig` already
/// refuses an unnamed pod, so a check for emptiness cannot fail on anything
/// `load` returns. What the example is promising is that the field it
/// demonstrates is a *name* — and that the name is not a real unit's, because a
/// tracked file naming one would be a deployment file in a public tree.
const EXAMPLE_POD: &str = "example-reachy";

/// The example configuration's path, out of the environment the target sets.
fn example() -> PathBuf {
    PathBuf::from(std::env::var("HOST_PARAMS").expect("the target names the example params"))
}

/// The clip name table the payload carries, out of the same environment.
fn names() -> PathBuf {
    PathBuf::from(std::env::var("CLIP_NAMES").expect("the target names the shipped name table"))
}

#[test]
fn the_example_configuration_is_one_this_build_accepts() {
    let path = example();
    let settings = reachy_host::params::load(&path)
        .unwrap_or_else(|error| panic!("the example configuration is refused: {error}"));
    assert_eq!(settings.edge.pod(), EXAMPLE_POD);
    // The stow budget the harness gesture is pinned against: the file drifting
    // from the constant is a gesture whose last step no longer ends where its
    // timeout says it does.
    assert_eq!(settings.edge.stow_duration_ms(), STOW_DURATION_MS);
    assert_eq!(settings.edge.body_cap_bytes(), BODY_CAP_BYTES);
    assert_eq!(
        settings.clip_names.file_name(),
        Some(std::ffi::OsStr::new("clip_library.names.json")),
        "the example configuration names the payload's name table: {}",
        settings.clip_names.display(),
    );
}

#[test]
fn the_name_table_the_configuration_points_at_is_one_the_edge_can_read() {
    // The path is checked as the committed asset rather than as the example's
    // string, because the string is a payload-relative path and a test does not
    // run from the payload. What this holds is the join between the two: the
    // file the configuration names by its last component is the file the edge
    // resolves overlay names through.
    let settings = reachy_host::params::load(&example()).expect("the example configuration");
    let table = names();
    assert_eq!(
        settings.clip_names.file_name(),
        table.file_name(),
        "the example configuration names the name table the payload carries",
    );
    let text = std::fs::read_to_string(&table)
        .unwrap_or_else(|error| panic!("reading {}: {error}", table.display()));
    MotionTable::from_sidecar(&text)
        .unwrap_or_else(|error| panic!("the committed name table is refused: {error}"));
}
