# shellcheck shell=bash
#
# Shared harness for the tools/*.test.sh self-checks. Sourced, never executed —
# no shebang, not executable:
#
#     # shellcheck source=test-lib.sh
#     . "$(dirname -- "${BASH_SOURCE[0]}")/test-lib.sh"
#
# One harness rather than one per self-check: `make test-scripts` is a real gate,
# and every improvement to a harness that exists six times costs six edits or
# produces six dialects.
#
# Everything here is read by the file that sources this one, so "appears unused"
# is the expected shape of every definition.
# shellcheck disable=SC2034

# ---------------------------------------------------------------------------
# The tally
# ---------------------------------------------------------------------------

passes=0
failures=0

pass() {
	echo "PASS: $1"
	passes=$((passes + 1))
}

# A failure names the case, then any number of indented detail lines: what was
# expected, and what was there instead. Both go to stderr, so a run's verdict
# reads without them and a diagnosis has them.
fail() {
	echo "FAIL: $1" >&2
	failures=$((failures + 1))
	shift
	local line
	for line in "$@"; do
		printf '    %s\n' "$line" >&2
	done
}

# The last line of every self-check. Its own status is the run's verdict, so
# calling it last makes the script exit non-zero on any failure.
tally() {
	echo "${passes} passed, ${failures} failed"
	[ "$failures" -eq 0 ]
}

# ---------------------------------------------------------------------------
# The assertions
# ---------------------------------------------------------------------------

# Fixed strings throughout: every needle a self-check here passes is a message
# or a path, and a message with a bracket or a dot in it is not a pattern.
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

assert_eq() {
	local label=$1 want=$2 got=$3
	if [ "$want" = "$got" ]; then
		pass "$label"
	else
		fail "$label" "expected: ${want}" "got:      ${got}"
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

assert_file() {
	local label=$1 path=$2
	if [ -f "$path" ]; then
		pass "$label"
	else
		fail "$label" "no file at ${path}"
	fi
}

assert_no_file() {
	local label=$1 path=$2
	if [ -e "$path" ]; then
		fail "$label" "a file is still at ${path}"
	else
		pass "$label"
	fi
}

# ---------------------------------------------------------------------------
# The run's own directory
# ---------------------------------------------------------------------------

# Every self-check builds a tree the subject runs out of, so the directory and
# its cleanup are here rather than in each of them. Removed on exit however the
# run ends, including a failure part-way through a case.
work=$(mktemp -d)
trap 'rm -rf -- "$work"' EXIT

# The two halves of a subject's run, which the self-checks capture as one string
# ending in a status line so a case can assert on both. The encoding is the
# caller's — it is the caller that knows how to invoke its subject — and these
# read it back.
output_of() { sed '$d' <<<"$1"; }
status_of() { sed -n '$s/^---status //p' <<<"$1"; }
