#!/usr/bin/env bash
#
# tools/deploy-bench.test.sh — self-check for deploy-bench.sh.
#
# The script under test reaches a device with ssh and rsync and reads the
# repository with git. All three are stubbed here, on PATH, recording what they
# were asked for; nothing in this file touches a network, a device, or this
# checkout. The subject is copied into a temporary tree beside its own lib.sh,
# so `repo_root` is that tree and the binary the deploy pushes is a file this
# test made.
#
# What is worth pinning: the freshness refusal and its escape hatch. A refusal
# that quietly stops refusing is discovered on the bench night it should have
# saved, which is how this check came to exist.
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

# Fixed strings throughout: every needle here is a message, and a message with
# a bracket or a dot in it is not a pattern.
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

assert_lacks() {
	local label=$1 haystack=$2 needle=$3
	if contains "$haystack" "$needle"; then
		fail "$label" "expected NOT to find: ${needle}" "in:" "$haystack"
	else
		pass "$label"
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
# The tree the subject runs out of, and the stubs it finds on PATH.
# ---------------------------------------------------------------------------

work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT

repo="${work}/repo"
mkdir -p -- "${repo}/tools" "${repo}/target/bench-arm64/release"
cp -- "${script_dir}/deploy-bench.sh" "${script_dir}/lib.sh" "${repo}/tools/"

subject="${repo}/tools/deploy-bench.sh"
binary="${repo}/target/bench-arm64/release/reachy-bench"

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

# Where the stubs write what they were asked, and the knobs the cases turn.
export CALLS="${work}/calls"
export GIT_COMMIT_TIME=""
export SSH_RUN_STATUS=0

# Every stub records its whole invocation on one line, so a case can assert
# both that a command ran and that it did not.
cat >"${stubs}/ssh" <<'STUB'
#!/usr/bin/env bash
printf 'ssh %s\n' "$*" >>"$CALLS"
# Only the run itself carries a status worth faking; the mkdir before it, and
# the fetch's cat, succeed.
for arg in "$@"; do
	case "$arg" in
		*reachy-bench*) exit "${SSH_RUN_STATUS:-0}" ;;
	esac
done
exit 0
STUB

cat >"${stubs}/rsync" <<'STUB'
#!/usr/bin/env bash
printf 'rsync %s\n' "$*" >>"$CALLS"
exit 0
STUB

# The repository question the freshness check asks. An empty GIT_COMMIT_TIME is
# a tree whose history says nothing about these paths.
cat >"${stubs}/git" <<'STUB'
#!/usr/bin/env bash
printf 'git %s\n' "$*" >>"$CALLS"
[ -n "${GIT_COMMIT_TIME:-}" ] || exit 0
echo "$GIT_COMMIT_TIME"
STUB

chmod 0755 -- "${stubs}/ssh" "${stubs}/rsync" "${stubs}/git"

# A device binary whose age is ours to set. Executable, because the subject
# refuses one that is not.
build_binary_at() {
	local when=$1
	: >"$binary"
	chmod 0755 -- "$binary"
	touch -d "@${when}" -- "$binary"
}

# Run the subject, answering with its output and its status, and start each
# case from an empty call record.
deploy() {
	: >"$CALLS"
	local out status=0
	out=$("$subject" "$@" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

output_of() { sed '$d' <<<"$1"; }
status_of() { sed -n '$s/^---status //p' <<<"$1"; }

calls() { cat -- "$CALLS"; }

# Two times an hour apart, so "older" and "newer" are unambiguous whatever the
# filesystem's timestamp resolution is.
commit_at=1750000000
before=$((commit_at - 3600))
after=$((commit_at + 3600))

# ---------------------------------------------------------------------------
# Freshness
# ---------------------------------------------------------------------------

GIT_COMMIT_TIME=$commit_at
build_binary_at "$after"
result=$(deploy unit --run selftest)
assert_status "a binary newer than the newest commit runs" 0 "$(status_of "$result")"
assert_contains "the run reaches the device" "$(calls)" "reachy-bench"

assert_contains "the freshness question is asked of the workspace paths" "$(calls)" \
	"git -C ${repo} log -1 --format=%ct -- crates containers Cargo.toml Cargo.lock rust-toolchain.toml"

build_binary_at "$before"
result=$(deploy unit --run selftest)
assert_status "a binary older than the newest commit refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says what is wrong" "$(output_of "$result")" \
	"older than the newest commit"
assert_contains "the refusal names the build" "$(output_of "$result")" "make bench-build"
assert_contains "the refusal names the override" "$(output_of "$result")" "--stale-ok"
assert_lacks "a refused run pushes nothing" "$(calls)" "rsync"
assert_lacks "a refused run reaches no device" "$(calls)" "ssh"

result=$(deploy unit --run --stale-ok selftest)
assert_status "--stale-ok runs the old binary" 0 "$(status_of "$result")"
assert_contains "--stale-ok says the age went unchecked" "$(output_of "$result")" \
	"the binary's age is not being checked"
assert_contains "--stale-ok still runs the bench" "$(calls)" "reachy-bench selftest"
assert_lacks "--stale-ok is the script's flag, not the bench's" "$(calls)" "reachy-bench --stale-ok"

# A tree that cannot answer the question is not a stale tree.
GIT_COMMIT_TIME=""
build_binary_at "$before"
result=$(deploy unit --run selftest)
assert_status "no history means no verdict, and the run proceeds" 0 "$(status_of "$result")"
assert_contains "an undecidable age is said out loud" "$(output_of "$result")" \
	"the binary's age is unknown"
GIT_COMMIT_TIME=$commit_at

# The check runs after the binary itself is accounted for, so a missing build
# is still reported as a missing build.
rm -f -- "$binary"
result=$(deploy unit --run selftest)
assert_status "a missing binary refuses" 1 "$(status_of "$result")"
assert_contains "a missing binary names the build" "$(output_of "$result")" "make bench-build"
assert_lacks "a missing binary is not reported as a stale one" "$(output_of "$result")" \
	"older than the newest commit"

# Nothing but --run has a binary to be stale.
build_binary_at "$before"
config="${work}/reachy-bench.toml"
echo 'device = "/dev/null"' >"$config"
result=$(deploy unit --config "$config")
assert_status "pushing a configuration is not a run" 0 "$(status_of "$result")"
assert_lacks "pushing a configuration asks no freshness question" "$(calls)" "git "

# ---------------------------------------------------------------------------
# What the run itself does with the device's answer
# ---------------------------------------------------------------------------

build_binary_at "$after"

SSH_RUN_STATUS=7
result=$(deploy unit --run selftest)
assert_status "the bench's own verdict is the script's" 7 "$(status_of "$result")"

SSH_RUN_STATUS=3
result=$(deploy unit --run selftest)
assert_status "a held bus refuses" 1 "$(status_of "$result")"
assert_contains "a held bus names the service holding it" "$(output_of "$result")" \
	"brenn-app.service is running"

SSH_RUN_STATUS=4
result=$(deploy unit --run selftest)
assert_status "the motion daemon holding the bus refuses" 1 "$(status_of "$result")"
assert_contains "the motion daemon is named with the way back" "$(output_of "$result")" \
	"systemctl start reachy-motiond.service"

SSH_RUN_STATUS=255
result=$(deploy unit --run selftest)
assert_status "ssh failing is not a hardware reading" 1 "$(status_of "$result")"
assert_contains "ssh failing says so" "$(output_of "$result")" "did not run"

SSH_RUN_STATUS=0

# ---------------------------------------------------------------------------

echo "${passes} passed, ${failures} failed"
[ "$failures" -eq 0 ]
