#!/usr/bin/env bash
#
# tools/ci-assert-bazel-caches.test.sh — self-check for ci-assert-bazel-caches.sh.
#
# The subject is the only automated guard on CI's cache wiring, and every one of
# its verdicts is a refusal: a guard that has quietly stopped being able to fail
# is worse than no guard, because it retires the suspicion. So the cases here are
# mostly the red ones — each break the subject exists to catch, checked to
# actually produce a non-zero exit and a diagnostic naming the mismatch.
#
# The subject is copied into a temporary tree beside its own lib.sh, so
# `repo_root` is that tree and the .bazelrc it reads is one this test wrote. HOME
# points into the temporary tree too, so the `~` expansion is exercised without
# touching the real one. Nothing here runs Bazel, reaches a network, or reads
# this checkout's .bazelrc.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"


# ---------------------------------------------------------------------------
# The tree the subject runs out of
# ---------------------------------------------------------------------------

repo="${work}/repo"
mkdir -p -- "${repo}/tools"
cp -- "${script_dir}/ci-assert-bazel-caches.sh" "${script_dir}/lib.sh" "${repo}/tools/"
subject="${repo}/tools/ci-assert-bazel-caches.sh"

HOME="${work}/home"
export HOME
disk="${HOME}/.cache/bazel-disk"
repo_cache="${HOME}/.cache/bazel-repo"
launcher="${HOME}/.cache/bazelisk"

# A populated store: one file is all the subject asks for, and one file is what
# a store nested arbitrarily deep looks like to it.
populate() {
	mkdir -p -- "${1}/ac/00"
	echo blob >"${1}/ac/00/entry"
}

# An existing store with nothing in it — the shape a save archives when the two
# copies of the path have diverged.
empty_store() { mkdir -p -- "$1"; }

reset_stores() {
	rm -rf -- "${HOME}/.cache"
	populate "$disk"
	populate "$repo_cache"
	populate "$launcher"
}

# .bazelrc as the tree really has it: the two flags under `common:ci`, tilde
# form, with the surrounding comment a real one carries.
write_bazelrc() {
	cat >"${repo}/.bazelrc" <<'RC'
common --lockfile_mode=update

# CI's caches, behind a config so a local build neither reads nor writes them.
common:ci --disk_cache=~/.cache/bazel-disk
common:ci --repository_cache=~/.cache/bazel-repo
RC
}

gate_flags='--config=ci --lockfile_mode=error'

# One run of the subject, with the wiring the workflow really passes unless a
# case overrides it. Output and status come back together the way the other
# self-checks here report them.
run_subject() {
	local out status=0
	out=$("$subject" \
		--gate-flags "$OVERRIDE_FLAGS" \
		--disk-cache "$OVERRIDE_DISK" \
		--repository-cache "$OVERRIDE_REPO" \
		--launcher-home "$OVERRIDE_LAUNCHER_HOME" \
		--store "$launcher_arg" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}


# What the run above passes, and what each case overrides one of. The tilde stays
# unexpanded, as it does when a shell passes the workflow's environment variable
# through: expanding it is the subject's job. Assembled from a variable for the
# reason the subject's own expander assembles one — the linter reads a quoted
# literal tilde as one that failed to expand.
tilde='~'
disk_arg="${tilde}/.cache/bazel-disk"
repo_arg="${tilde}/.cache/bazel-repo"
launcher_arg="${tilde}/.cache/bazelisk"

OVERRIDE_DISK=$disk_arg
OVERRIDE_REPO=$repo_arg
OVERRIDE_FLAGS=$gate_flags
# Already expanded, as BAZELISK_HOME is when the install step writes it: the
# variable exists because bazelisk cannot expand a tilde itself.
OVERRIDE_LAUNCHER_HOME=$launcher

write_bazelrc
reset_stores

# ---------------------------------------------------------------------------
# The wiring as it ships
# ---------------------------------------------------------------------------

result=$(run_subject)
assert_status "matched paths and populated stores pass" 0 "$(status_of "$result")"
assert_contains "the report names the disk cache" "$(output_of "$result")" "$disk"
assert_contains "the report names the repository cache" "$(output_of "$result")" "$repo_cache"
assert_contains "the report names the extra store" "$(output_of "$result")" "$launcher"

# ---------------------------------------------------------------------------
# A store the run left empty, and one that is not there at all
# ---------------------------------------------------------------------------

rm -rf -- "$disk"
empty_store "$disk"
result=$(run_subject)
assert_status "an empty disk cache refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the empty store" "$(output_of "$result")" "$disk"

rm -rf -- "$disk"
result=$(run_subject)
assert_status "an absent disk cache refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the absent store" "$(output_of "$result")" "$disk"

reset_stores
rm -rf -- "$repo_cache"
result=$(run_subject)
assert_status "an absent repository cache refuses" 1 "$(status_of "$result")"

reset_stores
rm -rf -- "$launcher"
result=$(run_subject)
assert_status "an absent extra store refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the extra store" "$(output_of "$result")" "$launcher"

# ---------------------------------------------------------------------------
# The three-way path join, broken from either side. Both cases have every store
# populated: this is the check that does not need a cold run to bite.
# ---------------------------------------------------------------------------

reset_stores
OVERRIDE_DISK="${tilde}/.cache/bazel_disk"
result=$(run_subject)
assert_status "a workflow path that .bazelrc does not set refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the .bazelrc side" "$(output_of "$result")" "$disk"
assert_contains "the refusal names the workflow side" "$(output_of "$result")" \
	"${HOME}/.cache/bazel_disk"
OVERRIDE_DISK=$disk_arg

OVERRIDE_REPO="${HOME}/.cache/somewhere-else"
result=$(run_subject)
assert_status "a diverged repository cache path refuses" 1 "$(status_of "$result")"
OVERRIDE_REPO=$repo_arg

# An absolute path naming the same directory as .bazelrc's tilde form is the same
# store, and has to pass: the check is about the path, not the spelling.
OVERRIDE_DISK="${HOME}/.cache/bazel-disk"
result=$(run_subject)
assert_status "an absolute path equal to the tilde form passes" 0 "$(status_of "$result")"
OVERRIDE_DISK=$disk_arg

# ---------------------------------------------------------------------------
# .bazelrc losing the config, or setting it for everyone
# ---------------------------------------------------------------------------

cat >"${repo}/.bazelrc" <<'RC'
common --lockfile_mode=update
common:ci --repository_cache=~/.cache/bazel-repo
RC
result=$(run_subject)
assert_status "a .bazelrc with no disk_cache line refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says which flag is missing" "$(output_of "$result")" "disk_cache"

cat >"${repo}/.bazelrc" <<'RC'
common:ci --disk_cache=~/.cache/bazel-disk
common:ci --disk_cache=~/.cache/bazel-disk-two
common:ci --repository_cache=~/.cache/bazel-repo
RC
result=$(run_subject)
assert_status "two disk_cache lines refuse" 1 "$(status_of "$result")"

# The one-word invariant the whole local-builds-untouched claim rests on.
cat >"${repo}/.bazelrc" <<'RC'
common --disk_cache=~/.cache/bazel-disk
common:ci --repository_cache=~/.cache/bazel-repo
RC
result=$(run_subject)
assert_status "an unscoped disk_cache refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says local builds would use CI's stores" \
	"$(output_of "$result")" "local builds"

# A commented-out flag is not a setting, and must not be read as one.
cat >"${repo}/.bazelrc" <<'RC'
# common --disk_cache=~/.cache/bazel-disk was considered and rejected
common:ci --disk_cache=~/.cache/bazel-disk
common:ci --repository_cache=~/.cache/bazel-repo
RC
result=$(run_subject)
assert_status "a commented cache flag is not a setting" 0 "$(status_of "$result")"

write_bazelrc

# ---------------------------------------------------------------------------
# The gate's flags. Every store is populated and every path agrees: without this
# check a dropped --config=ci is a green, permanently cold job.
# ---------------------------------------------------------------------------

OVERRIDE_FLAGS='--lockfile_mode=error'
result=$(run_subject)
assert_status "a gate that does not pass --config=ci refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the flag" "$(output_of "$result")" "--config=ci"

# A prefix is not the flag: --config=cirrus must not satisfy it.
OVERRIDE_FLAGS='--config=cirrus --lockfile_mode=error'
result=$(run_subject)
assert_status "a config whose name merely starts with ci refuses" 1 "$(status_of "$result")"
OVERRIDE_FLAGS=$gate_flags

# ---------------------------------------------------------------------------
# The bazelisk pin. Every store is populated here too: on the runner the pinned
# path and bazelisk's own default name the same directory, so a pin that never
# took effect leaves nothing else to notice it.
# ---------------------------------------------------------------------------

OVERRIDE_LAUNCHER_HOME=
result=$(run_subject)
assert_status "an unset BAZELISK_HOME refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says the variable never arrived" "$(output_of "$result")" \
	"BAZELISK_HOME"

OVERRIDE_LAUNCHER_HOME="${HOME}/.cache/bazelisk-elsewhere"
result=$(run_subject)
assert_status "a BAZELISK_HOME outside the launcher stores refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the directory it will not accept" "$(output_of "$result")" \
	"${HOME}/.cache/bazelisk-elsewhere"

# A directory the job archives but not as a launcher store: the refusal is about
# which store the pin names, so its wording must not claim the path is unarchived.
OVERRIDE_LAUNCHER_HOME=$disk
result=$(run_subject)
assert_status "a BAZELISK_HOME pointing at the disk cache refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says it is not a launcher store" "$(output_of "$result")" \
	"not one of the launcher stores"

# The tilde form must refuse even though it names the archived directory once
# expanded: bazelisk takes the tilde literally, so this value means the launcher
# is being written into the workspace while the archived path looks fine.
OVERRIDE_LAUNCHER_HOME=$launcher_arg
result=$(run_subject)
assert_status "a tilde-form BAZELISK_HOME refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the unexpanded value" "$(output_of "$result")" \
	"$launcher_arg"
assert_contains "the refusal says why a tilde is wrong here" "$(output_of "$result")" \
	"absolute path"
OVERRIDE_LAUNCHER_HOME=$launcher

# ---------------------------------------------------------------------------
# Argument handling: a refusal here is worth having, because a mangled
# invocation that exits 0 is the guard silently gone.
# ---------------------------------------------------------------------------

status=0
"$subject" --disk-cache "$disk" --repository-cache "$repo_cache" >/dev/null 2>&1 || status=$?
assert_status "a missing --gate-flags refuses" 1 "$status"

status=0
"$subject" --gate-flags "$gate_flags" --disk-cache "$disk" >/dev/null 2>&1 || status=$?
assert_status "a missing --repository-cache refuses" 1 "$status"

# No --store at all: the launcher check has nothing to compare against, so the
# refusal has to blame the missing archive list rather than BAZELISK_HOME.
status=0
out=$("$subject" --gate-flags "$gate_flags" --disk-cache "$disk" \
	--repository-cache "$repo_cache" --launcher-home "$launcher" 2>&1) || status=$?
assert_status "a missing --store refuses" 1 "$status"
assert_contains "the refusal names the missing argument" "$out" "no --store given"

status=0
"$subject" --gate-flags "$gate_flags" --disk-cache "$disk" \
	--repository-cache "$repo_cache" >/dev/null 2>&1 || status=$?
assert_status "a missing --launcher-home refuses" 1 "$status"

status=0
"$subject" --gate-flags "$gate_flags" --disk-cache >/dev/null 2>&1 || status=$?
assert_status "a flag with no value refuses" 1 "$status"

status=0
"$subject" --gate-flags "$gate_flags" --disk-cache "$disk" \
	--repository-cache "$repo_cache" --nonsense >/dev/null 2>&1 || status=$?
assert_status "an unknown argument refuses" 1 "$status"

# ---------------------------------------------------------------------------

tally
