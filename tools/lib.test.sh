#!/usr/bin/env bash
#
# tools/lib.test.sh — self-check for the shared prelude's expand_home.
#
# expand_home decides two things nothing else re-checks: the value the CI
# workflow writes into BAZELISK_HOME, and whether two spellings of one directory
# compare equal inside ci-assert-bazel-caches.sh. Its coverage through that
# script is indirect, and the function is small enough to look simplifiable —
# which is how the tilde-in-a-variable trick and the HOME refusal get removed by
# someone tidying up. These cases are direct, so such an edit is red here.
#
# HOME is redirected to a fixed string, so the expectations are exact and no case
# depends on the machine. Nothing here touches the filesystem or a network.
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

assert_eq() {
	local label=$1 want=$2 got=$3
	if [ "$want" = "$got" ]; then
		pass "$label"
	else
		fail "$label" "expected: ${want}" "got:      ${got}"
	fi
}

# The subject, sourced into this shell. `prog` and `repo_root` come with it and
# are what its own die() reports; nothing here depends on their values.
# shellcheck source=lib.sh
. "${script_dir}/lib.sh"

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

echo "${passes} passed, ${failures} failed"
[ "$failures" -eq 0 ]
