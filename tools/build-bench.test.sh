#!/usr/bin/env bash
#
# tools/build-bench.test.sh — self-check for build-bench.sh.
#
# The script under test drives bazel: a build, then a cquery that names the
# output. Both are stubbed here — one stub on PATH answering both subcommands —
# and the subject is copied into a temporary tree beside its own lib.sh, so
# `repo_root` is that tree and the artifact is a file this test made. Nothing
# here builds anything, reaches a network, or touches this checkout.
#
# What is worth pinning is the artifact's timestamp. deploy-bench.sh refuses a
# device binary older than the newest commit to the workspace, and the answer to
# that refusal is `make bench-build` — so a build that relinked nothing has to
# leave the binary looking current anyway, or the refusal becomes one an
# operator can only clear with --stale-ok on a binary that was never stale. With
# Bazel that falls out of the install copy rather than out of a separate stamp of
# an output the build reused, so what is asserted is that every successful build
# stamps, and that a refused one stamps nothing.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"

# ---------------------------------------------------------------------------
# The tree the subject runs out of, and what it finds around it.
# ---------------------------------------------------------------------------

repo="${work}/repo"
mkdir -p -- "${repo}/tools"
cp -- "${script_dir}/build-bench.sh" "${script_dir}/lib.sh" "${repo}/tools/"

subject="${repo}/tools/build-bench.sh"
binary="${repo}/target/bench-arm64/release/reachy-bench"

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

# The bazel stub. `build` writes the output the way a real build would — into a
# path under the workspace root that cquery then names, read-only as Bazel leaves
# its outputs, and with an mtime from before this run so that a stamp at the
# contract path is distinguishable from the output's own age. STUB_MACHINE is the
# ELF e_machine the build produces.
#
# Every invocation is recorded whole, arguments included: what makes the artifact
# a device binary rather than a workstation one is the flag set, and STUB_MACHINE
# would answer aarch64 whether or not the platform flag was ever passed. The
# STUB_*_STATUS and STUB_CQUERY_OUTPUT knobs drive the refusal paths.
export STUB_CALLS="${work}/calls"
export STUB_OUTPUT=bazel-out/stub/bin/crates/reachy-bench/reachy_bench
export STUB_MACHINE=aarch64
export STUB_BUILD_STATUS=0
export STUB_CQUERY_STATUS=0
export STUB_CQUERY_OUTPUT=
cat >"${stubs}/bazel" <<'STUB'
#!/usr/bin/env bash
set -euo pipefail
printf 'bazel %s\n' "$*" >>"$STUB_CALLS"
case "${1:-}" in
	build)
		[ "${STUB_BUILD_STATUS:-0}" = 0 ] || exit "$STUB_BUILD_STATUS"
		mkdir -p -- "$(dirname -- "$STUB_OUTPUT")"
		case "$STUB_MACHINE" in
			# e_machine at bytes 18-19, little-endian: 183 for AArch64,
			# 62 for the x86_64 a platform flag that did nothing gives.
			aarch64) machine='\267\000' ;;
			*) machine='\076\000' ;;
		esac
		rm -f -- "$STUB_OUTPUT"
		# shellcheck disable=SC2059 # the format string is what carries the bytes
		printf "\177ELF\002\001\001\000\000\000\000\000\000\000\000\000\002\000${machine}" \
			>"$STUB_OUTPUT"
		touch -d '@1000000000' -- "$STUB_OUTPUT"
		chmod 0555 -- "$STUB_OUTPUT"
		;;
	cquery)
		[ "${STUB_CQUERY_STATUS:-0}" = 0 ] || exit "$STUB_CQUERY_STATUS"
		echo "${STUB_CQUERY_OUTPUT:-$STUB_OUTPUT}"
		;;
esac
exit 0
STUB
chmod 0755 -- "${stubs}/bazel"

calls_of() { grep -- "^bazel $1 " "$STUB_CALLS" || true; }

# The flags each invocation carried, target excluded: the point of comparing them
# is that the cquery describes the configuration the build used, so the two have
# to be the same string and not merely both plausible.
flags_of() { sed -e "s/^bazel $1 //" -e 's/ -- .*$//' <<<"$(calls_of "$1")"; }

# Each case starts from an empty call record, so the assertions about what bazel
# was asked for describe this run and not the sum of the ones before it.
build() {
	local out status=0
	: >"$STUB_CALLS"
	out=$("$subject" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}


mtime_of() { stat -c %Y -- "$1"; }

# An hour back, which is unambiguously older than anything this run produces.
stale=$(($(date +%s) - 3600))

# ---------------------------------------------------------------------------
# The first build, and then one Bazel has nothing to do for
# ---------------------------------------------------------------------------

result=$(build)
assert_status "a build with no artifact yet produces one" 0 "$(status_of "$result")"
assert_contains "the build names what it produced" "$(output_of "$result")" "device binary"
if [ -x "$binary" ]; then
	pass "the artifact is executable"
else
	fail "the artifact is executable"
fi

# What makes the output a device binary: the device configuration, and the
# target. The configuration is .bazelrc's `device` — named, not spelled, so this
# build, the motion build and `make check-device` cannot describe different ones
# — and what is in it is that file's business, asserted there.
asked_build_flags=$(flags_of build)
asked_cquery_flags=$(flags_of cquery)
assert_contains "the build asks for the device configuration" "$asked_build_flags" \
	"--config=device"
assert_contains "the build names the bench target" "$(calls_of build)" \
	"-- //crates/reachy-bench:reachy_bench"
assert_contains "the cquery asks about the same target" "$(calls_of cquery)" \
	"-- //crates/reachy-bench:reachy_bench"

# The cquery's flags have to open with the build's, or it answers about a
# configuration nothing built.
if [ -n "$asked_build_flags" ] &&
	[ "${asked_cquery_flags#"$asked_build_flags"}" != "$asked_cquery_flags" ]; then
	pass "the cquery describes the configuration the build used"
else
	fail "the cquery's flags are not the build's" \
		"build:  ${asked_build_flags}" \
		"cquery: ${asked_cquery_flags}"
fi

# The installed file is the output cquery named, and it is writable so the next
# build can replace it — Bazel leaves its own outputs read-only.
if cmp -s -- "$binary" "${repo}/${STUB_OUTPUT}"; then
	pass "the artifact is the output bazel named"
else
	fail "the artifact is not a copy of the output bazel named"
fi
mode=$(stat -c %a -- "$binary")
if [ "$mode" = 755 ]; then
	pass "the artifact carries the mode the install asked for"
else
	fail "the artifact's mode is ${mode}, not 755" \
		"a read-only artifact carried over from bazel's output is one the next" \
		"build cannot replace"
fi

# Bazel's no-op: the output is already current, so it is handed back with the
# mtime it has carried since it was linked. The install copy is the whole of what
# this build leaves behind, and without it the binary at the contract path would
# still carry an age from before every commit since.
touch -d "@${stale}" -- "$binary"
result=$(build)
assert_status "a build with nothing to relink still succeeds" 0 "$(status_of "$result")"
if [ "$(mtime_of "$binary")" -gt "$stale" ]; then
	pass "every successful build stamps the artifact"
else
	fail "a build that relinked nothing left the artifact's age where it was" \
		"a freshly built binary that deploy-bench.sh calls stale is a refusal" \
		"no rebuild can clear"
fi

# ---------------------------------------------------------------------------
# A stamp is only ever put on an artifact the checks passed
# ---------------------------------------------------------------------------

STUB_MACHINE=x86_64
touch -d "@${stale}" -- "$binary"
result=$(build)
assert_status "an artifact for the wrong machine refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the architecture" "$(output_of "$result")" "not AArch64"
if [ "$(mtime_of "$binary")" -eq "$stale" ]; then
	pass "a build that refused stamps nothing"
else
	fail "a build that refused stamped the artifact anyway" \
		"the age is asked about a binary the device can execute"
fi
STUB_MACHINE=aarch64

# ---------------------------------------------------------------------------
# The other ways a build fails to name something installable
#
# Each of these has to refuse *and* leave the contract path where it was: the age
# deploy-bench.sh reads must never be that of a binary no check passed.
# ---------------------------------------------------------------------------

# A build that fails. The script has no explicit check for it — set -e is what
# stops it — so what is pinned is that nothing downstream runs.
touch -d "@${stale}" -- "$binary"
STUB_BUILD_STATUS=1
result=$(build)
assert_status "a failed build refuses" 1 "$(status_of "$result")"
assert_lacks "and nothing was asked to name an output" "$(calls_of cquery)" "cquery"
if [ "$(mtime_of "$binary")" -eq "$stale" ]; then
	pass "a failed build stamps nothing"
else
	fail "a failed build stamped the artifact anyway"
fi
STUB_BUILD_STATUS=0

# The three ways Bazel can answer badly. The refusals themselves are lib.sh's
# and their wording is pinned once, in tools/lib.test.sh; what this suite owns is
# that the script stops on each of them and stamps nothing.
touch -d "@${stale}" -- "$binary"
STUB_CQUERY_STATUS=1
result=$(build)
assert_status "a cquery that fails refuses" 1 "$(status_of "$result")"
if [ "$(mtime_of "$binary")" -eq "$stale" ]; then
	pass "a build whose output cannot be named stamps nothing"
else
	fail "a build whose output cannot be named stamped the artifact anyway"
fi
STUB_CQUERY_STATUS=0

# A cquery that answers nothing.
touch -d "@${stale}" -- "$binary"
STUB_CQUERY_OUTPUT=$'\n'
result=$(build)
assert_status "an empty cquery answer refuses" 1 "$(status_of "$result")"
if [ "$(mtime_of "$binary")" -eq "$stale" ]; then
	pass "an empty cquery answer stamps nothing"
else
	fail "an empty cquery answer stamped the artifact anyway"
fi

# A cquery answer that does not carry the file this script asked for. What the
# script owns here is the basename it wants: the target's own output name, not
# the contract path's.
touch -d "@${stale}" -- "$binary"
STUB_CQUERY_OUTPUT=bazel-out/stub/bin/crates/reachy-bench/not-there
result=$(build)
assert_status "a listing without the binary in it refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names what it wanted" "$(output_of "$result")" "reachy_bench"
if [ "$(mtime_of "$binary")" -eq "$stale" ]; then
	pass "a listing without the binary stamps nothing"
else
	fail "a listing without the binary stamped the artifact anyway"
fi

# A listing that carries the name at a path nothing is at.
touch -d "@${stale}" -- "$binary"
STUB_CQUERY_OUTPUT=bazel-out/stub/bin/elsewhere/reachy_bench
result=$(build)
assert_status "an unresolvable cquery answer refuses" 1 "$(status_of "$result")"
if [ "$(mtime_of "$binary")" -eq "$stale" ]; then
	pass "an unresolvable cquery answer stamps nothing"
else
	fail "an unresolvable cquery answer stamped the artifact anyway"
fi

# A target with more than one output: the day the bench binary gains a data dep
# or a debug artefact, the listing is two lines and the wanted one is still
# named. Taking the listing itself as the answer is what this pins against.
STUB_CQUERY_OUTPUT="bazel-out/stub/bin/crates/reachy-bench/reachy_bench.data
${STUB_OUTPUT}"
result=$(build)
assert_status "a listing with a second output still builds" 0 "$(status_of "$result")"
assert_contains "and installs the binary" "$(output_of "$result")" "device binary"
STUB_CQUERY_OUTPUT=

# ---------------------------------------------------------------------------
# No bazel, no build
# ---------------------------------------------------------------------------

result=$(REACHY_BAZEL="${stubs}/no-such-bazel" build)
assert_status "a missing bazel refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says how to get one" "$(output_of "$result")" "bazelisk"

# ---------------------------------------------------------------------------

tally
