#!/usr/bin/env bash
#
# tools/test-lib.test.sh — self-check for the shared harness itself.
#
# Everything the harness does for the other self-checks it does invisibly: the
# tally, and the staged tree it keeps when a run went wrong. A break in either
# turns no suite red — it degrades every suite's diagnostics quietly. An
# inverted keep condition deletes the one tree somebody needed; a keep condition
# stuck true leaks a directory per suite per run. Both are green everywhere else
# in the gate, so they are asserted here.
#
# The harness is sourced, so the subject cannot be run: each case is a fixture
# script written into this run's own tree, which sources the harness out of this
# checkout and then behaves like a suite that passed, failed, or died. The
# fixtures run with TMPDIR pointing inside this run's directory, so a tree a
# fixture was told to keep is kept where this run's own cleanup will take it
# rather than in the machine's temp directory.
#
# Nothing here builds anything, reaches a network, or touches this checkout
# beyond reading test-lib.sh.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"

# The subject, reached by the fixtures through the environment rather than by an
# interpolated path, so the fixture bodies stay literal.
TEST_LIB="${script_dir}/test-lib.sh"
export TEST_LIB

fixtures="${work}/fixtures"
fixture_tmp="${work}/tmp"
mkdir -p -- "$fixtures" "$fixture_tmp"

# The line every fixture prints first, so a case can name the directory the
# fixture's harness made and then ask whether it is still there. The evidence
# file is what makes a kept tree a forensic artefact rather than a bare `mkdir`:
# a keep that dropped the contents would pass a mere existence check.
prelude() {
	cat <<'FIXTURE'
#!/usr/bin/env bash
set -euo pipefail
# shellcheck disable=SC1090
. "$TEST_LIB"
echo "workdir ${work}"
printf '%s\n' 'the stubs and the layout that produced the failure' >"${work}/evidence"
FIXTURE
}

write_fixture() {
	local name=$1
	{
		prelude
		cat
	} >"${fixtures}/${name}"
	chmod 0755 -- "${fixtures}/${name}"
}

# A fixture's run, encoded the way the other self-checks encode a subject's: the
# output and a trailing status line, so one string carries both. Streams are
# merged because the kept-tree announcement is on stderr and the tally on stdout,
# and most cases care only that a line appeared.
run_fixture() {
	local out status=0
	out=$(TMPDIR="$fixture_tmp" "${fixtures}/$1" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

workdir_of() { sed -n 's/^workdir //p' <<<"$1"; }

kept_line="the tree this run staged is at"

# ---------------------------------------------------------------------------
# A run that passed: the tally, and no tree left behind
# ---------------------------------------------------------------------------

write_fixture passing <<'FIXTURE'
pass "a case that holds"
tally
FIXTURE

result=$(run_fixture passing)
assert_status "a suite whose cases all pass exits 0" 0 "$(status_of "$result")"
assert_contains "and reports its tally" "$(output_of "$result")" "1 passed, 0 failed"
assert_lacks "and says nothing about a kept tree" "$(output_of "$result")" "$kept_line"
assert_no_file "and the tree it staged is gone" "$(workdir_of "$(output_of "$result")")"

# ---------------------------------------------------------------------------
# A run that failed a case: the tally, the path, and the tree behind it
# ---------------------------------------------------------------------------

write_fixture failing <<'FIXTURE'
fail "a case that does not hold" "what it wanted" "what was there"
tally
FIXTURE

result=$(run_fixture failing)
output=$(output_of "$result")
kept=$(workdir_of "$output")
assert_status "a suite with a failed case exits non-zero" 1 "$(status_of "$result")"
assert_contains "and reports its tally" "$output" "0 passed, 1 failed"
assert_contains "and names the tree it kept" "$output" "${kept_line} ${kept}"
assert_file "and the tree is still there, with what the run staged in it" "${kept}/evidence"
rm -rf -- "$kept"

# ---------------------------------------------------------------------------
# A run that died mid-case: no tally, and the path still named
# ---------------------------------------------------------------------------

# The shape a harness flake most plausibly takes — a stub that died, a `cd` that
# failed, an unbound variable — and the one whose tree is least reproducible.
# `tally` never runs, so the announcement has to come from the cleanup.
write_fixture aborting <<'FIXTURE'
pass "a case that holds"
false
tally
FIXTURE

result=$(run_fixture aborting)
output=$(output_of "$result")
kept=$(workdir_of "$output")
assert_status "a suite that dies mid-case exits non-zero" 1 "$(status_of "$result")"
assert_lacks "and never reached its tally" "$output" "passed,"
assert_contains "and still names the tree it kept" "$output" "${kept_line} ${kept}"
assert_file "and that tree holds what the run staged" "${kept}/evidence"
rm -rf -- "$kept"

# ---------------------------------------------------------------------------
# A run kept on purpose: green, and the tree named anyway
# ---------------------------------------------------------------------------

# The deliberate-inspection path. Without a case it is distinguishable from the
# failure path only by reading the source, and it is the one an operator uses to
# look at a green run's staged tree.
write_fixture keeping <<'FIXTURE'
pass "a case that holds"
tally
FIXTURE

result=$(REACHY_TEST_KEEP=1 run_fixture keeping)
output=$(output_of "$result")
kept=$(workdir_of "$output")
assert_status "REACHY_TEST_KEEP does not make a green run red" 0 "$(status_of "$result")"
assert_contains "the tally is still green" "$output" "1 passed, 0 failed"
assert_contains "and the tree is named" "$output" "${kept_line} ${kept}"
assert_file "and kept with what the run staged" "${kept}/evidence"
rm -rf -- "$kept"

# ---------------------------------------------------------------------------
# Which stream each line is on
# ---------------------------------------------------------------------------

# The verdict is stdout and the diagnostics are stderr, all through the harness,
# so a run's stdout reads as a verdict and tooling that captures only stdout is
# not looking at a path.
stdout_only=$(TMPDIR="$fixture_tmp" "${fixtures}/failing" 2>/dev/null || true)
kept=$(workdir_of "$stdout_only")
assert_contains "the tally is on stdout" "$stdout_only" "0 passed, 1 failed"
assert_lacks "the kept-tree line is not" "$stdout_only" "$kept_line"
assert_lacks "and neither is the failure detail" "$stdout_only" "what was there"
rm -rf -- "$kept"

tally
