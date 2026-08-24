#!/usr/bin/env bash
#
# Build the bench binary for the device: an aarch64 executable.
#
#   tools/build-bench.sh
#
# A cross-compile, not an emulated native build: Bazel builds the same target
# the gate builds, for the device platform, with the hermetic clang sysroot the
# pinned Clockwork drop brings. The sysroot's aarch64 glibc is older than the one
# brenn-os carries, which is the compatible direction — glibc symbol versioning
# is backward compatible, so a binary that references the older versions runs on
# the newer library. Nothing here needs podman, qemu, or a dated archive URL.
#
# The artifact lands at target/bench-arm64/release/reachy-bench: an installed
# copy of Bazel's output rather than the output itself, because the freshness
# contract deploy-bench.sh enforces is about a file this script stamps, and
# Bazel's own outputs are read-only and keep the mtime of the action that wrote
# them.
#
# Knobs, environment only:
#
#   REACHY_BAZEL   the bazel to run (default bazel)

set -euo pipefail

# shellcheck source=lib.sh
. "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

bazel=${REACHY_BAZEL:-bazel}

# Every bazel invocation below runs from the workspace root, whatever directory
# the caller was in: that is where the module lives, and it is what the
# workspace-relative path cquery answers with is relative to.
cd -- "$repo_root"

# The device configuration lives in .bazelrc as `device`, so the gate, the bench
# build and the motion build cannot describe different configurations. Every
# invocation below passes it: the cquery that resolves the output path has to
# describe the configuration the build used, or it names a file from some other
# one.
bench_target=//crates/reachy-bench:reachy_bench
build_flags=(--config=device)

# The contract path. `target/` is a cargo-ism kept deliberately: it is a literal
# in deploy-bench.sh, in the self-checks beside both scripts, and in the runbook,
# and renaming it buys the build nothing.
target_dir="${repo_root}/target/bench-arm64"

binary_name=reachy-bench
binary="${target_dir}/release/${binary_name}"

# What Bazel calls the file, which is the target's own name and not the contract
# path's: the build emits `reachy_bench` and this script installs it as
# `reachy-bench`.
bazel_output_name=reachy_bench

compile() {
	"$bazel" build "${build_flags[@]}" -- "$bench_target"
}

# Where Bazel put it, through the shared helpers in lib.sh: the same refusals
# build-motion.sh reads, in the same words, because the words are a diagnosis an
# operator acts on and two copies of it drift.
#
# The wanted file is named rather than the listing being taken as the answer.
# The target has one output today; the day it gains a second one — a data dep, a
# debug artefact — a listing handed to `bazel_resolve` would diagnose a
# target-graph change as a renamed convenience symlink, which is a confidently
# wrong prescription in the middle of a hardware session.
locate() {
	local out
	out=$(bazel_files "$bench_target") || exit 1
	bazel_named_in "$out" "$bazel_output_name" || exit 1
}

# Install the verified output at the contract path. `install` writes a new file
# and does not carry the source's timestamps, so the destination's mtime is the
# moment this build finished — the one mechanism the freshness contract rests on,
# and the reason not to reach for `install -p` or `cp -p` here.
#
# That copy is what keeps the age honest: Bazel hands back a cached output
# exactly as it found it, so a commit that changes nothing the binary links — a
# checked-in trace fixture, a test-only edit — would otherwise leave a freshly
# built binary older than the newest commit, and deploy-bench.sh would refuse
# the very binary this build just produced with a prescription (rebuild it) that
# cannot clear it.
install_artifact() {
	local out=$1
	mkdir -p -- "$(dirname -- "$binary")"
	install -m 0755 -- "$out" "$binary"
}

report() {
	local size
	size=$(du -h -- "$binary" | cut -f1)
	echo "${prog}: device binary  ${binary}  (${size})"
	echo "${prog}: sha256         $(sha256sum -- "$binary" | cut -d' ' -f1)"
}

require_bazel "device binary"
compile
out=$(locate)
verify_aarch64 "$out"
install_artifact "$out"
report
