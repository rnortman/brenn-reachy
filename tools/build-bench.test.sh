#!/usr/bin/env bash
#
# tools/build-bench.test.sh — self-check for build-bench.sh.
#
# The script under test drives podman and reads the host's binfmt registration.
# Both are stubbed here — podman on PATH, the registration through the script's
# own REACHY_BINFMT_DIR knob — and the subject is copied into a temporary tree
# beside its own lib.sh, so `repo_root` is that tree and the artifact is a file
# this test made. Nothing here builds anything, reaches a network, or touches
# this checkout.
#
# What is worth pinning is the artifact's timestamp. deploy-bench.sh refuses a
# device binary older than the newest commit to the workspace, and the answer to
# that refusal is `make bench-build` — so a build that relinks nothing has to
# leave the binary looking current anyway, or the refusal becomes one an
# operator can only clear with --stale-ok on a binary that was never stale.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

passes=0
failures=0

pass() {
	echo "PASS: $1"
	passes=$((passes + 1))
}

fail() {
	echo "FAIL: $1" >&2
	failures=$((failures + 1))
	shift
	local line
	for line in "$@"; do
		printf '    %s\n' "$line" >&2
	done
}

contains() {
	printf '%s' "$1" | grep -qF -- "$2"
}

assert_contains() {
	local label=$1 haystack=$2 needle=$3
	if contains "$haystack" "$needle"; then
		pass "$label"
	else
		fail "$label" "expected to find: ${needle}" "in:" "$haystack"
	fi
}

assert_status() {
	local label=$1 want=$2 got=$3
	if [ "$want" = "$got" ]; then
		pass "$label"
	else
		fail "$label" "expected exit ${want}, got ${got}"
	fi
}

# ---------------------------------------------------------------------------
# The tree the subject runs out of, and what it finds around it.
# ---------------------------------------------------------------------------

work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT

repo="${work}/repo"
mkdir -p -- "${repo}/tools" "${repo}/containers/bench-builder"
cp -- "${script_dir}/build-bench.sh" "${script_dir}/lib.sh" "${repo}/tools/"
# The builder image is named for this file's content, so it has to exist for
# the subject to name an image at all.
echo 'FROM debian:trixie' >"${repo}/containers/bench-builder/Containerfile"

subject="${repo}/tools/build-bench.sh"
binary="${repo}/target/bench-arm64/release/reachy-bench"

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

# A binfmt registration the preflight accepts, so the test runs the same way on
# an x86_64 workstation and on an arm64 one.
binfmt="${work}/binfmt"
mkdir -p -- "$binfmt"
printf 'enabled\ninterpreter /usr/bin/qemu-aarch64-static\nflags: OCF\n' \
	>"${binfmt}/qemu-aarch64"
export REACHY_BINFMT_DIR="$binfmt"

# The compile, as cargo behaves: it writes an artifact when there is none, and
# leaves one it did not have to relink exactly as it found it — mtime included.
# STUB_MACHINE is the ELF e_machine the build produces.
export STUB_BINARY="$binary"
export STUB_MACHINE=aarch64
cat >"${stubs}/podman" <<'STUB'
#!/usr/bin/env bash
case "${1:-}" in
	image)
		# The image is already there, so nothing is ever built here.
		exit 0
		;;
	build)
		exit 0
		;;
	run)
		[ ! -e "$STUB_BINARY" ] || exit 0
		mkdir -p -- "$(dirname -- "$STUB_BINARY")"
		case "$STUB_MACHINE" in
			# e_machine at bytes 18-19, little-endian: 183 for AArch64,
			# 62 for the x86_64 an unemulated container would produce.
			aarch64) machine='\267\000' ;;
			*) machine='\076\000' ;;
		esac
		# shellcheck disable=SC2059 # the format string is what carries the bytes
		printf "\177ELF\002\001\001\000\000\000\000\000\000\000\000\000\002\000${machine}" \
			>"$STUB_BINARY"
		exit 0
		;;
esac
exit 0
STUB
chmod 0755 -- "${stubs}/podman"

build() {
	local out status=0
	out=$("$subject" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

output_of() { sed '$d' <<<"$1"; }
status_of() { sed -n '$s/^---status //p' <<<"$1"; }

mtime_of() { stat -c %Y -- "$1"; }

# An hour back, which is unambiguously older than anything this run produces.
stale=$(($(date +%s) - 3600))

# ---------------------------------------------------------------------------
# The first build, and then one with nothing to relink
# ---------------------------------------------------------------------------

result=$(build)
assert_status "a build with no artifact yet produces one" 0 "$(status_of "$result")"
assert_contains "the build names what it produced" "$(output_of "$result")" "device binary"
if [ -x "$binary" ]; then
	pass "the artifact is executable"
else
	fail "the artifact is executable"
fi

# Cargo's no-op: the artifact is already current, so the compile writes nothing.
# The stamp is the whole of what this build leaves behind, and without it the
# binary would still carry an age from before every commit since it was linked.
touch -d "@${stale}" -- "$binary"
result=$(build)
assert_status "a build with nothing to relink still succeeds" 0 "$(status_of "$result")"
if [ "$(mtime_of "$binary")" -gt "$stale" ]; then
	pass "a build that relinked nothing still stamps the artifact"
else
	fail "a build that relinked nothing left the artifact's age where it was" \
		"a freshly built binary that deploy-bench.sh calls stale is a refusal" \
		"no rebuild can clear"
fi

# ---------------------------------------------------------------------------
# A stamp is only ever put on an artifact the checks passed
# ---------------------------------------------------------------------------

rm -f -- "$binary"
STUB_MACHINE=x86_64
result=$(build)
assert_status "an artifact for the wrong machine refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the architecture" "$(output_of "$result")" "not AArch64"
touch -d "@${stale}" -- "$binary"
result=$(build)
if [ "$(mtime_of "$binary")" -eq "$stale" ]; then
	pass "a build that refused stamps nothing"
else
	fail "a build that refused stamped the artifact anyway" \
		"the age is asked about a binary the device can execute"
fi
STUB_MACHINE=aarch64

# ---------------------------------------------------------------------------

echo "${passes} passed, ${failures} failed"
[ "$failures" -eq 0 ]
