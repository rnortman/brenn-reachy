#!/usr/bin/env bash
#
# .githooks/pre-commit.test.sh — self-check for the pre-commit hook.
#
# The hook's whole job is refusals: the secret scrub before anything else, the
# git environment kept out of the project's checks, and the gate itself. A
# refusal that silently stops working is discovered by the commit it should
# have stopped — which is this repo's own history, twice in one round — so each
# of them is pinned here.
#
# Both external commands are stubbed on PATH and record what they were asked
# for; nothing in this file runs a build, reads the index, or touches this
# checkout. The hook runs in a temporary tree whose Makefile is a file this
# test wrote.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
subject="${script_dir}/pre-commit"

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

# Fixed strings throughout: every needle here is a command line, and a command
# line with a dot or a dash in it is not a pattern.
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

# Whole-line matching, for the cases that must tell `make check` from `make
# check-commit`: one is a prefix of the other, so a substring search cannot say
# which of them ran.
lacks_line() {
	! printf '%s\n' "$1" | grep -qxF -- "$2"
}

assert_lacks_line() {
	local label=$1 haystack=$2 line=$3
	if lacks_line "$haystack" "$line"; then
		pass "$label"
	else
		fail "$label" "expected NOT to find the line: ${line}" "in:" "$haystack"
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
# The tree the hook runs in, and the stubs it finds on PATH.
# ---------------------------------------------------------------------------

work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT

repo="${work}/repo"
mkdir -p -- "$repo"

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

# Where the stubs write what they were asked, and the knobs the cases turn.
export CALLS="${work}/calls"
export SCRUB_STATUS=0
export MAKE_STATUS=0

cat >"${stubs}/brenn-scrub" <<'STUB'
#!/usr/bin/env bash
printf 'brenn-scrub %s\n' "$*" >>"$CALLS"
exit "${SCRUB_STATUS:-0}"
STUB

# Records the target it was asked for and every GIT_ variable that reached it,
# which is the environment question the hook exists to answer.
cat >"${stubs}/make" <<'STUB'
#!/usr/bin/env bash
printf 'make %s\n' "$*" >>"$CALLS"
for var in $(compgen -v GIT_ || true); do
	printf 'inherited %s\n' "$var" >>"$CALLS"
done
exit "${MAKE_STATUS:-0}"
STUB

chmod 0755 -- "${stubs}/brenn-scrub" "${stubs}/make"

# The Makefile the hook reads to decide which target to ask for. Only the
# target lines matter: the hook greps, it does not parse.
write_makefile() {
	: >"${repo}/Makefile"
	local target
	for target in "$@"; do
		printf '%s:\n\t@true\n' "$target" >>"${repo}/Makefile"
	done
}

# Run the hook the way git does — from the work tree, with the index variables
# git exports into it — answering with its status, and start each case from an
# empty call record.
run_hook() {
	: >"$CALLS"
	local status=0
	(
		cd -- "$repo" || exit 1
		GIT_DIR="${repo}/.git" \
			GIT_INDEX_FILE="${repo}/.git/index" \
			"$subject" >/dev/null 2>&1
	) || status=$?
	printf '%s' "$status"
}

calls() { cat -- "$CALLS"; }

# ---------------------------------------------------------------------------
# The gate the hook asks for
# ---------------------------------------------------------------------------

write_makefile check check-bazel check-commit
status=$(run_hook)
assert_status "a tree that can run both lanes commits" 0 "$status"
assert_contains "the commit gate is both lanes" "$(calls)" "make check-commit"
assert_lacks_line "and not the Cargo lane alone" "$(calls)" "make check"

# A checkout from before check-commit existed: core.hooksPath hooks run at
# whatever revision is checked out, so the older target is still reachable.
write_makefile check
status=$(run_hook)
assert_status "a historical checkout still commits" 0 "$status"
assert_contains "and falls back to the lane it has" "$(calls)" "make check"
assert_lacks_line "without asking for a target that is not there" "$(calls)" "make check-commit"

# No Makefile at all: the scrub still runs, and nothing else is invented.
rm -f -- "${repo}/Makefile"
status=$(run_hook)
assert_status "a tree with no Makefile commits" 0 "$status"
assert_contains "the scrub ran anyway" "$(calls)" "brenn-scrub staged"
assert_lacks "and no gate was invented" "$(calls)" "make"

# ---------------------------------------------------------------------------
# The refusals
# ---------------------------------------------------------------------------

write_makefile check check-commit
MAKE_STATUS=1
status=$(run_hook)
assert_status "a failing gate fails the commit" 1 "$status"
MAKE_STATUS=0

# The scrub is first because it fails in seconds and the gate takes minutes,
# and because a secret must not reach a build log on the way to being caught.
SCRUB_STATUS=2
status=$(run_hook)
assert_status "a refused scrub fails the commit" 2 "$status"
assert_contains "the scrub was asked about the staged index" "$(calls)" "brenn-scrub staged"
assert_lacks "and nothing was built after it refused" "$(calls)" "make"
SCRUB_STATUS=0

# ---------------------------------------------------------------------------
# The environment the gate sees
# ---------------------------------------------------------------------------

# git exports GIT_DIR and GIT_INDEX_FILE into the hook, and those retarget every
# child git process — including test fixtures, which would then mutate this
# repository instead of their own tempdirs.
status=$(run_hook)
assert_status "the gate ran" 0 "$status"
assert_lacks "no GIT_ variable reaches the gate" "$(calls)" "inherited GIT_"

# ---------------------------------------------------------------------------

echo "${passes} passed, ${failures} failed"
[ "$failures" -eq 0 ]
