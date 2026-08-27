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
# calling it last makes the script exit non-zero on any failure. The tree the run
# staged is named by the cleanup below, which is the only place that knows
# whether it survived.
tally() {
	echo "${passes} passed, ${failures} failed"
	[ "$failures" -eq 0 ]
}

# ---------------------------------------------------------------------------
# The assertions
# ---------------------------------------------------------------------------

# Fixed strings throughout: every needle a self-check here passes is a message
# or a path, and a message with a bracket or a dot in it is not a pattern. The
# quoted needle inside the pattern is what makes it fixed -- a `*` or a `[` in a
# message matches itself.
#
# In the shell rather than through `grep`, and that is load-bearing: a pipeline
# under `pipefail` reports the writer's death, so a `grep -q` that matched early
# in a large haystack and closed the pipe made `printf` exit on SIGPIPE and the
# whole call answer "no match" -- intermittently, depending on which of the two
# ran first.
contains() {
	case "$1" in
	*"$2"*) return 0 ;;
	*) return 1 ;;
	esac
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

# A motion-run harness's budget against the wake lead it is mostly made of.
#
#   assert_run_budget_covers_lead <harness script in the checkout>
#
# Both harnesses — the host one and the device one — stop their launcher on a
# hand-maintained budget whose largest term, the shipped wake lead, lives in
# another file and language. A lead that grows without a budget following it is
# red here rather than a launcher stopped mid-gesture.
#
# The sum around the lead is stated once, here: commissioning's bus survey
# (~5 s), the raise-hold-stow gesture (~5 s), and the release (~4 s). The
# margin each harness carries above this is deliberately excluded — that is for
# a loaded workstation or a serial bus that retries, and each script's own
# comment says which.
assert_run_budget_covers_lead() {
	local script=$1
	local label="${script##*/}'s run budget covers the shipped wake lead and the phases around it"
	local checkout lead budget phases needed
	checkout=$(cd -- "$(dirname -- "$script")/.." && pwd)
	phases=14
	lead=$(sed -n 's/^lead_ms:[[:space:]]*\([0-9]*\).*/\1/p' \
		-- "${checkout}/cogs/wake_params.textproto")
	budget=$(sed -n 's/^run_seconds=\([0-9]*\).*/\1/p' -- "$script")
	if [ -z "$lead" ] || [ -z "$budget" ]; then
		fail "$label" \
			"read no lead_ms from cogs/wake_params.textproto or no run_seconds from" \
			"${script} — one of the two names has moved"
		return
	fi
	needed=$((lead / 1000 + phases))
	if [ "$budget" -ge "$needed" ]; then
		pass "$label"
	else
		fail "$label" \
			"the budget is ${budget} s" \
			"the shipped wake lead is ${lead} ms, and commissioning, the gesture" \
			"and the release want ${phases} s around it: ${needed} s" \
			"the launcher would be stopped mid-gesture"
	fi
}

# ---------------------------------------------------------------------------
# The run's own directory
# ---------------------------------------------------------------------------

# Every self-check builds a tree the subject runs out of, so the directory and
# its cleanup are here rather than in each of them. Removed on exit — except
# when the run ends badly: a failed case, a non-zero exit, or
# `REACHY_TEST_KEEP` set. The staged tree is every input to a failure (the
# stubs, the forced mtimes, the payload layout), and deleting it leaves a
# diagnosis nothing but the message.
#
# The path is announced from the trap rather than from `tally`, so it is printed
# however the run ended: a suite that dies mid-case under `set -e` never reaches
# `tally`, and that abort is the failure shape whose tree is least reproducible.
# To stderr, so a run's verdict on stdout reads without it.
work=$(mktemp -d)
keep_work() { [ "${1:-0}" -ne 0 ] || [ "$failures" -ne 0 ] || [ -n "${REACHY_TEST_KEEP:-}" ]; }
trap 'if keep_work "$?"; then
	echo "the tree this run staged is at ${work}" >&2
else
	rm -rf -- "$work"
fi' EXIT

# The two halves of a subject's run, which the self-checks capture as one string
# ending in a status line so a case can assert on both. The encoding is the
# caller's — it is the caller that knows how to invoke its subject — and these
# read it back.
output_of() { sed '$d' <<<"$1"; }
status_of() { sed -n '$s/^---status //p' <<<"$1"; }
