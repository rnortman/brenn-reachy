#!/usr/bin/env bash
#
# tools/gate.test.sh — self-check for the two shell lanes of the gate itself,
# `make check-scripts` and `make test-scripts`.
#
# Both lanes are shell inside a Makefile recipe, and both decide something: which
# files the gate reads and runs, and how a script the gate cannot run is refused.
# Neither is covered by the `tools/*.test.sh` suites, which exercise the scripts
# rather than the recipes that find them — so a lost `|| {`, an `-x` on the wrong
# variable, or a walk narrowed back to the index would go green here and take the
# gate's coverage with it.
#
# The subject is this repository's own Makefile, copied into a scratch git
# repository this run makes, so the recipes walk a tree whose whole contents are
# named below. Nothing here touches this checkout, builds anything, or reaches a
# network; the two git hooks the lint lane names by path are created as trivial
# valid scripts so the lane has them to read.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"

repo_root=$(cd -- "${script_dir}/.." && pwd)

# ---------------------------------------------------------------------------
# The tree the recipes walk
# ---------------------------------------------------------------------------

repo="${work}/repo"
mkdir -p -- "${repo}/tools" "${repo}/.githooks"
cp -- "${repo_root}/Makefile" "${repo}/Makefile"

git -C "$repo" init --quiet
# A repository with no identity cannot commit, and nothing here commits: `git
# add` is enough to make a path tracked, which is the only distinction the
# recipes draw.
git -C "$repo" config user.email tester@example.invalid
git -C "$repo" config user.name Tester

# The two extensionless hooks the lint lane names by path. Valid and quiet: they
# are there so the lane has something to read, not to be the subject.
for hook in pre-commit pre-push; do
	printf '#!/usr/bin/env bash\nexit 0\n' >"${repo}/.githooks/${hook}"
	chmod 0755 -- "${repo}/.githooks/${hook}"
done

# The marker an executed self-check prints. Distinctive, so a case that asserts
# the gate ran a script is not satisfied by the recipe echoing its name.
marker="the-self-check-ran"

# A `*.test.sh` that passes and says so.
good_test_script() {
	printf '#!/usr/bin/env bash\nset -euo pipefail\necho %s\n' "$marker" >"$1"
	chmod 0755 -- "$1"
}

# A lane that refused. Which non-zero status is make's business rather than the
# recipe's, so the assertion is about the refusal and not about the number.
assert_refused() {
	local label=$1 status=$2
	if [ "$status" != 0 ]; then
		pass "$label"
	else
		fail "$label" "the lane exited 0"
	fi
}

# One lane, run out of the scratch repository, captured as output plus a status
# line the harness reads back.
#
# make is fed a recognisable stdin rather than whatever the suite has, so a
# script the recipe runs with stdin left open reads the poison and says so. Under
# `make check` the suite's own stdin is already /dev/null, which would make the
# recipe's `</dev/null` per script indistinguishable from doing nothing.
stdin_poison="poison-for-a-script-that-reads-stdin"
lane() {
	local out status=0
	out=$(printf '%s\n' "$stdin_poison" |
		make -C "$repo" --no-print-directory "$1" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

# ---------------------------------------------------------------------------
# test-scripts: the set includes what nobody staged
# ---------------------------------------------------------------------------
#
# The lane's whole reason for walking untracked files is that a self-check
# written and not yet added is one the next commit carries. What it buys is
# asserted here directly: an untracked script is run, and its output is the
# proof.

good_test_script "${repo}/tools/fresh.test.sh"
result=$(lane test-scripts)
assert_status "an untracked self-check is run" 0 "$(status_of "$result")"
assert_contains "and it really ran" "$(output_of "$result")" "$marker"

# The same file, now tracked: still one run, so the widened walk did not double
# the set.
git -C "$repo" add tools/fresh.test.sh
result=$(lane test-scripts)
assert_status "a tracked self-check is run" 0 "$(status_of "$result")"
assert_eq "and exactly once" 1 \
	"$(grep -c -- "$marker" <<<"$(output_of "$result")")"

# An ignored path is not the gate's business, however it is spelled.
mkdir -p -- "${repo}/target"
printf 'target/\n' >"${repo}/.gitignore"
good_test_script "${repo}/target/ignored.test.sh"
result=$(lane test-scripts)
assert_status "an ignored self-check is left alone" 0 "$(status_of "$result")"
assert_lacks "and is not in the set" "$(output_of "$result")" "target/ignored.test.sh"

# A self-check that reads stdin — one spawning a pty, one feeding a subject —
# must not swallow the list of the suites after it. The lane would exit 0 having
# run a prefix of the set, which is the one failure a green result cannot be
# told from.
#
# The recipe has two independent halves — the list on a descriptor of its own,
# and each script run with stdin closed — and each is asserted separately: the
# marker count says both scripts ran (the list survived), and what the hungry one
# recorded says its stdin was closed rather than the lane's poison.
cat >"${repo}/tools/aaa-hungry.test.sh" <<HUNGRY
#!/usr/bin/env bash
set -euo pipefail
cat >"\$(dirname -- "\${BASH_SOURCE[0]}")/../stdin-seen"
echo ${marker}
HUNGRY
chmod 0755 -- "${repo}/tools/aaa-hungry.test.sh"
good_test_script "${repo}/tools/zzz-later.test.sh"
result=$(lane test-scripts)
assert_status "a self-check that reads stdin leaves the lane green" 0 \
	"$(status_of "$result")"
# Three: the tracked `fresh.test.sh` from the cases above, and the two here.
assert_eq "and every suite in the set really ran" 3 \
	"$(grep -c -- "$marker" <<<"$(output_of "$result")")"
assert_eq "with the hungry one handed a closed stdin, not the lane's" "" \
	"$(cat -- "${repo}/stdin-seen")"
rm -f -- "${repo}/tools/aaa-hungry.test.sh" "${repo}/tools/zzz-later.test.sh" \
	"${repo}/stdin-seen"

# ---------------------------------------------------------------------------
# test-scripts: the two refusals
# ---------------------------------------------------------------------------
#
# Each is refused by name before the run, because each otherwise fails as
# something a reader has to decode: `Permission denied` at exit 126, or bash's
# own complaint about a path that is not there.

good_test_script "${repo}/tools/unarmed.test.sh"
chmod 0644 -- "${repo}/tools/unarmed.test.sh"
result=$(lane test-scripts)
assert_refused "a self-check with no mode bit fails the lane" "$(status_of "$result")"
assert_contains "and is refused by name" "$(output_of "$result")" \
	"tools/unarmed.test.sh is not executable"
assert_contains "with the one command that fixes it" "$(output_of "$result")" \
	"chmod +x tools/unarmed.test.sh"
assert_lacks "and nothing after it ran" "$(output_of "$result")" \
	"Permission denied"
rm -f -- "${repo}/tools/unarmed.test.sh"

# A tracked script deleted from the working tree is still in the index, so the
# walk names a path that is not there. The mode-bit prescription would be a
# confident wrong answer about it.
good_test_script "${repo}/tools/gone.test.sh"
git -C "$repo" add tools/gone.test.sh
rm -f -- "${repo}/tools/gone.test.sh"
result=$(lane test-scripts)
assert_refused "a tracked self-check missing from the tree fails the lane" "$(status_of "$result")"
assert_contains "and is refused for the reason it is missing" "$(output_of "$result")" \
	"tools/gone.test.sh is tracked but missing from the working tree"
assert_lacks "not for a mode bit it cannot have" "$(output_of "$result")" \
	"chmod +x tools/gone.test.sh"
git -C "$repo" rm --quiet --cached tools/gone.test.sh

# ---------------------------------------------------------------------------
# check-scripts: the lint lane reads the same widened set
# ---------------------------------------------------------------------------

if command -v shellcheck >/dev/null 2>&1; then
	result=$(lane check-scripts)
	assert_status "a tree of clean scripts lints green" 0 "$(status_of "$result")"

	# Untracked, and a finding shellcheck cannot miss: a `cd` whose failure
	# nothing catches, in a `.sh` nobody staged.
	printf '#!/usr/bin/env bash\ncd /tmp\necho there\n' >"${repo}/tools/sloppy.sh"
	result=$(lane check-scripts)
	assert_refused "an untracked script with a finding fails the lint lane" "$(status_of "$result")"
	assert_contains "and the finding names the file" "$(output_of "$result")" \
		"tools/sloppy.sh"
	rm -f -- "${repo}/tools/sloppy.sh"
else
	# The lane is hard-required to refuse rather than skip, so a machine without
	# the tool is a case rather than a gap.
	result=$(lane check-scripts)
	assert_refused "the lint lane refuses without shellcheck" "$(status_of "$result")"
	assert_contains "and says what is missing" "$(output_of "$result")" \
		"shellcheck not found on PATH"
fi

tally
