#!/usr/bin/env bash
#
# tools/lib.test.sh — self-check for the shared prelude: expand_home, and the
# three helpers that read where Bazel put what it built.
#
# expand_home decides two things nothing else re-checks: the value the CI
# workflow writes into BAZELISK_HOME, and whether two spellings of one directory
# compare equal inside ci-assert-bazel-caches.sh. Its coverage through that
# script is indirect, and the function is small enough to look simplifiable —
# which is how the tilde-in-a-variable trick and the HOME refusal get removed by
# someone tidying up. These cases are direct, so such an edit is red here.
#
# HOME is redirected to a fixed string, so the expectations are exact and no case
# depends on the machine. The Bazel helpers run against a stub bazel in this
# run's own directory and resolve paths against this checkout, which they only
# ever read; nothing here builds anything or reaches a network.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"


# The subject, sourced into this shell. `prog` and `repo_root` come with it and
# are what its own die() reports; nothing here depends on their values.
# shellcheck source=lib.sh
. "${script_dir}/lib.sh"

# The prelude's own path, taken before the tilde cases re-source lib.sh in a
# subshell: after that, the linter reads every mention of `repo_root` as one that
# might have been changed in there.
lib_path="${repo_root}/tools/lib.sh"

HOME=/home/tester
export HOME

# The leading-tilde marker, assembled from a variable for the reason the subject
# assembles one: a quoted literal reads to the linter as a tilde that failed to
# expand. Named for what it means rather than what it is, because the linter also
# reads a variable named after the character as one that leaked out of a subshell.
home_ref='~'

assert_eq "a bare tilde is HOME" "/home/tester" "$(expand_home "$home_ref")"
assert_eq "a tilde-rooted path expands" "/home/tester/.cache/bazelisk" \
	"$(expand_home "${home_ref}/.cache/bazelisk")"
assert_eq "a trailing slash survives" "/home/tester/.cache/" \
	"$(expand_home "${home_ref}/.cache/")"
assert_eq "an absolute path is unchanged" "/var/lib/brenn-app" \
	"$(expand_home /var/lib/brenn-app)"
assert_eq "a relative path is unchanged" "tools/lib.sh" "$(expand_home tools/lib.sh)"
assert_eq "the empty string is unchanged" "" "$(expand_home "")"

# Only a *leading* tilde is a home reference. One elsewhere in the path, and a
# user-qualified one this function deliberately does not implement, both pass
# through as themselves.
assert_eq "an embedded tilde is unchanged" "/tmp/back~up/x" "$(expand_home "/tmp/back~up/x")"
assert_eq "a user-qualified tilde is unchanged" "${home_ref}other/x" \
	"$(expand_home "${home_ref}other/x")"

# An unset HOME must stop rather than yield /.cache/..., which is what the
# workflow would otherwise export as bazelisk's download directory. The subject
# is re-sourced in a subshell so the refusal's exit does not end this run.
refusal() {
	local arg=$1 out status=0
	out=$(
		unset HOME
		# shellcheck source=lib.sh
		. "${script_dir}/lib.sh"
		expand_home "$arg" 2>&1
	) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

result=$(refusal "${home_ref}/.cache/bazelisk")
assert_eq "an unset HOME refuses a tilde path" "1" "$(sed -n '$s/^---status //p' <<<"$result")"
if grep -qF -- "HOME is unset or empty" <<<"$result"; then
	pass "the refusal says HOME is missing"
else
	fail "the refusal says HOME is missing" "in:" "$result"
fi

result=$(refusal "$home_ref")
assert_eq "an unset HOME refuses a bare tilde" "1" "$(sed -n '$s/^---status //p' <<<"$result")"

result=$(refusal /var/lib/brenn-app)
assert_eq "an unset HOME does not affect an absolute path" "0" \
	"$(sed -n '$s/^---status //p' <<<"$result")"

# ---------------------------------------------------------------------------
# The Bazel output helpers
# ---------------------------------------------------------------------------
#
# Both device build scripts read where Bazel put things through these three, and
# the three refusals are the deliverable: they are what an operator reads in the
# middle of a hardware session, and each names the thing to go and look at. Both
# scripts' own suites see them through a subject that also builds a payload;
# these cases drive them directly, so an edit to the prose or to the exact-match
# rule is red here whichever script is being worked on.
#
# `bazel` and `build_flags` are the sourcing script's to set, so this file sets
# them, as build-bench.sh and build-motion.sh do.
stub="${work}/bazel"
cat >"$stub" <<'STUB'
#!/usr/bin/env bash
[ -z "${STUB_STATUS:-}" ] || exit "$STUB_STATUS"
printf '%s' "${STUB_OUTPUT:-}"
STUB
chmod 0755 -- "$stub"

bazel=$stub
build_flags=(--config=device)

# One helper's run, output and status as one string, so a case can assert on
# both: each of them refuses by calling die, whose exit this subshell contains.
attempt() {
	local out status=0
	out=$("$@" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

STUB_OUTPUT=$'bazel-out/bin/one\nbazel-out/bin/two\n'
export STUB_OUTPUT STUB_STATUS=
result=$(attempt bazel_files //some:target)
assert_status "a cquery that answers is passed through" 0 "$(status_of "$result")"
assert_eq "and the whole listing comes back" $'bazel-out/bin/one\nbazel-out/bin/two' \
	"$(output_of "$result")"

STUB_STATUS=1
result=$(attempt bazel_files //some:target)
assert_status "a cquery that fails refuses" 1 "$(status_of "$result")"
assert_contains "and the refusal names the expression" "$(output_of "$result")" \
	"bazel cannot name the outputs of //some:target"
STUB_STATUS=

STUB_OUTPUT=""
result=$(attempt bazel_files //some:target)
assert_status "a cquery that answers nothing refuses" 1 "$(status_of "$result")"
assert_contains "and says no output was named" "$(output_of "$result")" \
	"bazel named no output file for //some:target"

# Resolution is against the real checkout, since `repo_root` is where lib.sh
# lives: a path Bazel could plausibly have named and a path nothing is at.
result=$(attempt bazel_resolve tools/lib.sh)
assert_status "a path the tree has resolves" 0 "$(status_of "$result")"
assert_eq "and comes back absolute" "$lib_path" "$(output_of "$result")"

result=$(attempt bazel_resolve bazel-out/bin/not-there)
assert_status "a path nothing is at refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the path" "$(output_of "$result")" \
	"the build named bazel-out/bin/not-there and no file is there"
assert_contains "and both flags that explain it" "$(output_of "$result")" "--symlink_prefix"
assert_contains "and the other one" "$(output_of "$result")" \
	"--noexperimental_convenience_symlinks"

# The exact-basename rule, with a decoy whose name contains the wanted one: a
# listing carries the sources of what was built as well as its outputs, so a
# substring match stages a file nobody asked for.
listing=$'tools/xlib.sh\ntools/lib.sh\ntools/test-lib.sh'
result=$(attempt bazel_named_in "$listing" lib.sh)
assert_status "a basename the listing carries resolves" 0 "$(status_of "$result")"
assert_eq "and it is the exact match, not the one containing it" \
	"$lib_path" "$(output_of "$result")"

result=$(attempt bazel_named_in "$listing" nothing.tachyon)
assert_status "a basename the listing does not carry refuses" 1 "$(status_of "$result")"
assert_contains "and the refusal names it" "$(output_of "$result")" \
	"the build emits no nothing.tachyon"

# The pinion agreement over the files that actually ship it. The checker's own
# behaviour on drift is pinned in build-motion.test.sh against a synthesized
# file in a temporary tree; what is asserted here is the claim that matters at a
# bench — that the two logger configurations in *this* checkout state the
# flagless defaults. The device path runs this check from build-motion.sh and the
# host path from host-motion-run.sh, and neither is a target `make check` runs,
# so without these two cases an edit to either file is green in CI and refused
# only in front of a powered unit.
for shipped in cogs/robot_logger.textproto cogs/host_logger.textproto; do
	result=$(attempt check_pinion_defaults "$shipped")
	assert_status "${shipped} states the flagless pinion defaults" 0 \
		"$(status_of "$result")"
done

result=$(attempt check_pinion_defaults cogs/no_such_logger.textproto)
assert_status "a logger configuration the tree does not have refuses" 1 \
	"$(status_of "$result")"
assert_contains "and the refusal names the file it wanted" "$(output_of "$result")" \
	"cogs/no_such_logger.textproto"

tally
