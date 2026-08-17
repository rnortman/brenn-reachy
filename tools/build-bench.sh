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

# The device build: opt, and the aarch64 platform. Every invocation below passes
# the same flags — the cquery that resolves the output path has to describe the
# configuration the build used, or it names a file from some other one.
bench_target=//crates/reachy-bench:reachy_bench
build_flags=(
	--compilation_mode=opt
	--platforms=//bazel/platform:reachy-device
)

# The contract path. `target/` is a cargo-ism kept deliberately: it is a literal
# in deploy-bench.sh, in the self-checks beside both scripts, and in the runbook,
# and renaming it buys the build nothing.
target_dir="${repo_root}/target/bench-arm64"

binary_name=reachy-bench
binary="${target_dir}/release/${binary_name}"

# ELF e_machine for AArch64. A platform flag that failed to take effect produces
# an x86_64 binary that runs perfectly on the workstation and not at all on the
# device.
elf_machine_aarch64=183

# The same refusal the Makefile's require-bazel target gives, deliberately said
# twice: this script runs outside make, and REACHY_BAZEL gives it a third line
# the Makefile has no equivalent of.
preflight() {
	command -v -- "$bazel" >/dev/null 2>&1 ||
		die "the device binary is built by bazel and ${bazel} is not installed." \
			"Install bazelisk; .bazelversion pins the Bazel release it fetches." \
			"Or point REACHY_BAZEL at the bazel to use."
}

compile() {
	"$bazel" build "${build_flags[@]}" -- "$bench_target"
}

# Where Bazel put it. cquery rather than the bazel-bin symlink: that symlink
# points at whatever configuration ran last, and a plain `bazel test //...`
# afterwards repoints it at the host one.
#
# The answer is a workspace-relative bazel-out/... path, which resolves through
# the convenience symlink of that name. That symlink is the one assumption here,
# and .bazelrc.user can rename it (--symlink_prefix) or switch it off
# (--noexperimental_convenience_symlinks), so the refusal below names it: the
# build succeeding and the path not resolving is that flag, not a build that
# produced nothing.
locate() {
	local out
	out=$("$bazel" cquery "${build_flags[@]}" --output=files -- "$bench_target") ||
		die "bazel cannot name the output of ${bench_target}, so there is nothing to install."
	[ -n "$out" ] ||
		die "bazel named no output file for ${bench_target}."
	[ -f "${repo_root}/${out}" ] ||
		die "the build named ${out} and no file is there." \
			"If the build itself succeeded, the bazel-out convenience symlink is renamed" \
			"or disabled — check .bazelrc.user for --symlink_prefix or" \
			"--noexperimental_convenience_symlinks."
	echo "${repo_root}/${out}"
}

# The architecture check runs on Bazel's output, before anything reaches the
# contract path: a refused build leaves whatever was there untouched, so the age
# deploy-bench.sh reads is never that of a binary no check passed.
verify() {
	local out=$1

	# e_machine — bytes 18 and 19 of the ELF header, little-endian. Read with
	# od so the check costs no tooling a workstation might not carry.
	local machine
	machine=$(od -An -tu1 -j18 -N2 -- "$out" | awk '{print $1 + $2 * 256}')
	[ "$machine" = "$elf_machine_aarch64" ] || die \
		"the build produced an ELF for machine ${machine}, not AArch64 (${elf_machine_aarch64})." \
		"The platform flag did not take effect; the device cannot execute this."
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

preflight
compile
out=$(locate)
verify "$out"
install_artifact "$out"
report
