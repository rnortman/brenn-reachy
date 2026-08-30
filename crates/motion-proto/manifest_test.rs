//! The two descriptions of this crate's dependencies, held to each other.
//!
//! `BUILD.bazel` is what builds the crate here. `Cargo.toml` is the export
//! surface a Cargo workspace in another repository reads when it pins this crate
//! by revision. Neither file can see the other, and the failure they produce
//! when they disagree lands in the downstream workspace rather than in this
//! repo's gate — a dependency added to the Bazel target alone builds green here
//! and fails to compile there.
//!
//! So the two lists are joined: the `deps` of the `rust_library` and the keys of
//! `[dependencies]` must name the same set of crates. Versions are not compared
//! because the Bazel side does not carry them — `MODULE.bazel` states each
//! version once for the whole repo.

/// The environment variables naming the two files, relative to the runfiles
/// root, which is a test's working directory.
const BUILD_BAZEL: &str = "BUILD_BAZEL";
const CARGO_MANIFEST: &str = "CARGO_MANIFEST";

/// The contents of the file `name` points at.
///
/// Panics rather than answers: a missing runfile is a broken test target, not a
/// case.
fn runfile(name: &str) -> String {
    let path = std::env::var(name).unwrap_or_else(|_| {
        panic!(
            "{name} is unset: the test target has to name the file beside the data attribute that \
             supplies it"
        )
    });
    std::fs::read_to_string(&path)
        .unwrap_or_else(|error| panic!("{name} names {path}, which does not read: {error}"))
}

/// The third-party crates the `rust_library` target depends on.
///
/// A line reader over the literals a person typed, not a parse of Starlark. It
/// reads the `deps` list of the first `rust_library` in the file and stops at
/// its close; a second library target in this file would need this to say which
/// one it means, and the assertion below that the list is non-empty is what
/// turns a reader that matched nothing into a failure rather than a silent pass.
fn bazel_deps(text: &str) -> Vec<String> {
    let mut found = Vec::new();
    let mut in_library = false;
    let mut in_deps = false;
    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("rust_library(") {
            in_library = true;
            continue;
        }
        if !in_library {
            continue;
        }
        if trimmed == ")" {
            break;
        }
        if trimmed.starts_with("deps = [") {
            in_deps = true;
            continue;
        }
        if !in_deps {
            continue;
        }
        if trimmed.starts_with("]") {
            in_deps = false;
            continue;
        }
        let label = trimmed.trim_matches(|c| c == '"' || c == ',');
        let crate_name = label.rsplit_once(':').unwrap_or_else(|| {
            panic!("a dep in BUILD.bazel is not a label with a target name: {label}")
        });
        found.push(crate_name.1.to_owned());
    }
    found.sort();
    found
}

/// The keys of the manifest's `[dependencies]` table.
///
/// The same kind of reader, over the same kind of literal: every dependency in
/// this manifest is written `name = ...` on one line, and a shape that is not
/// that one is a manifest this test no longer understands, which it says.
fn cargo_deps(text: &str) -> Vec<String> {
    let mut found = Vec::new();
    let mut in_dependencies = false;
    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with('[') {
            in_dependencies = trimmed == "[dependencies]";
            continue;
        }
        if !in_dependencies || trimmed.is_empty() || trimmed.starts_with('#') {
            continue;
        }
        let (name, _) = trimmed.split_once('=').unwrap_or_else(|| {
            panic!("a line in [dependencies] is not `name = ...`, so this reader cannot see it: {trimmed}")
        });
        found.push(name.trim().to_owned());
    }
    found.sort();
    found
}

#[test]
fn build_file_and_manifest_name_the_same_dependencies() {
    let bazel = bazel_deps(&runfile(BUILD_BAZEL));
    let cargo = cargo_deps(&runfile(CARGO_MANIFEST));

    assert!(
        !bazel.is_empty(),
        "no deps read out of BUILD.bazel: the reader matched nothing, which would make every \
         comparison below vacuous"
    );
    assert!(
        !cargo.is_empty(),
        "no dependencies read out of Cargo.toml: the reader matched nothing, which would make \
         every comparison below vacuous"
    );
    assert_eq!(
        bazel, cargo,
        "BUILD.bazel and Cargo.toml disagree about what this crate depends on; the manifest is \
         the export surface a downstream Cargo workspace reads, so both have to change together"
    );
}
