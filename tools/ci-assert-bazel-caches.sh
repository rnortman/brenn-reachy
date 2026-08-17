#!/usr/bin/env bash
#
# Check that CI's Bazel cache wiring is joined up, and report what the stores
# hold.
#
#   tools/ci-assert-bazel-caches.sh \
#       --gate-flags "--config=ci --lockfile_mode=error" \
#       --disk-cache ~/.cache/bazel-disk \
#       --repository-cache ~/.cache/bazel-repo \
#       --launcher-home "$BAZELISK_HOME" \
#       --store DIR [--store DIR]...
#
# The store paths are stated twice and cannot be stated once: the workflow
# passes them to the cache action, and .bazelrc passes them to Bazel, which
# cannot read the workflow's. Divergence between the two copies leaves a job
# green and silently uncached, which looks exactly like an ordinary build. This
# compares them — .bazelrc's `common:ci` values against the paths the caller
# says it archived — so a one-sided edit is red on any run, warm or cold, rather
# than only on the run that happens to start with empty stores.
#
# Four further checks, all cheap and all guarding an invariant nothing else
# states:
#
#   - the gate's flags actually pass --config=ci, without which Bazel writes to
#     its output base and every archived store stays as it was restored;
#   - .bazelrc sets the two flags only under `common:ci`, so a developer's build
#     neither reads nor writes CI's stores;
#   - BAZELISK_HOME is set, absolute, and names one of the archived stores, so
#     the workflow's pin is a path this job archives rather than a stray one.
#     This is a check on the workflow's side of the pin only: the value reaching
#     here is the job's own environment, so it catches a deleted or mistyped
#     export and a tilde that never got expanded (bazelisk, being Go, would take
#     one literally), but not bazelisk ceasing to honour the variable. That
#     remains covered only by the empty-store check on a cold run, and only on a
#     runner whose own default has moved away from the archived path;
#   - every named store holds at least one file, which catches a `~` that
#     reached Bazel unexpanded and anything else that put the bytes elsewhere.
#     Only a cold run proves that one — a restore populates the directories
#     whatever Bazel did — and a cold run is what an edit to .bazelrc, the
#     module graph, or the workflow produces, since all three are in the cache
#     key.
#
# Exits 0 with a size report per store, or non-zero naming the mismatch.

set -euo pipefail

# shellcheck source=lib.sh
. "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

usage="usage: ${prog} --gate-flags FLAGS --disk-cache DIR --repository-cache DIR --launcher-home DIR --store DIR [--store DIR]..."

gate_flags=
disk_cache=
repository_cache=
launcher_home=
stores=()

while [ $# -gt 0 ]; do
	case $1 in
	--gate-flags | --disk-cache | --repository-cache | --launcher-home | --store)
		[ $# -ge 2 ] || die "${1} needs a value" "$usage"
		;;
	esac
	case $1 in
	--gate-flags) gate_flags=$2 ;;
	--disk-cache) disk_cache=$2 ;;
	--repository-cache) repository_cache=$2 ;;
	--launcher-home) launcher_home=$2 ;;
	--store) stores+=("$2") ;;
	*) die "unknown argument: ${1}" "$usage" ;;
	esac
	shift 2
done

[ -n "$gate_flags" ] || die "no --gate-flags given" "$usage"
[ -n "$disk_cache" ] || die "no --disk-cache given" "$usage"
[ -n "$repository_cache" ] || die "no --repository-cache given" "$usage"
# Empty is the shape an unset BAZELISK_HOME arrives in, which is the failure
# this argument exists to catch, so the two share a refusal.
[ -n "$launcher_home" ] || die \
	"no --launcher-home given: an empty one means BAZELISK_HOME never reached this step" \
	"$usage"
# At least one, because the launcher pin below is checked against this list and
# an empty one would refuse every pin with a message about the wrong thing.
[ ${#stores[@]} -gt 0 ] || die "no --store given: the launcher store is not optional" "$usage"

bazelrc="${repo_root}/.bazelrc"
[ -f "$bazelrc" ] || die "no .bazelrc at ${bazelrc}"

# The .bazelrc-versus-workflow comparisons put both sides through lib.sh's
# expand_home, so a tilde form and an absolute form of the same directory compare
# equal and the check is about the path and not about who spelled it how. The
# launcher pin further down expands the store side only: the pinned value itself
# has to already be absolute, for the reason given there.

# The lines .bazelrc uses to set a cache flag, comments excluded. Numbered, so a
# diagnostic can send the reader to the line.
cache_flag_lines() {
	grep -nE -- '--(disk_cache|repository_cache)=' "$bazelrc" |
		grep -vE '^[0-9]+:[[:space:]]*#' || true
}

# The value .bazelrc gives one cache flag under `common:ci`. Exactly one line
# may set it: two would make which one wins a question, and zero means the
# config the gate asks for does not point Bazel anywhere.
rc_value() {
	local flag=$1 lines count
	lines=$(cache_flag_lines | grep -E "^[0-9]+:common:ci --${flag}=" || true)
	count=$(printf '%s' "$lines" | grep -c . || true)
	[ "$count" = 1 ] || die \
		"expected exactly one \`common:ci --${flag}=\` line in .bazelrc, found ${count}" \
		"${lines:-(none)}"
	printf '%s\n' "${lines#*"--${flag}="}"
}

# The flag reaches Bazel only if the gate asks for the config that carries it.
# Padded on both sides so the match is the whole word.
case " ${gate_flags} " in
*' --config=ci '*) ;;
*) die "the gate's flags do not pass --config=ci: ${gate_flags}" \
	"Without it Bazel caches into its output base and the archived stores stay as restored." ;;
esac

# A cache flag set outside the `ci` config applies to every build on every
# machine, which is a multi-gigabyte store appearing in a developer's home
# directory with CI staying green about it.
misscoped=$(cache_flag_lines | grep -vE '^[0-9]+:common:ci ' || true)
[ -z "$misscoped" ] || die \
	".bazelrc sets a cache flag outside the \`ci\` config, so local builds use CI's stores" \
	"$misscoped"

compare() {
	local label=$1 flag=$2 given=$3 want got raw
	# Two steps, not one nested substitution: a refusal inside rc_value has to
	# be this function's exit status, and an outer substitution would discard it.
	raw=$(rc_value "$flag")
	want=$(expand_home "$raw")
	got=$(expand_home "$given")
	[ "$want" = "$got" ] || die \
		"${label}: .bazelrc and the workflow disagree about where this store lives" \
		".bazelrc --${flag}: ${want}" \
		"archived by the workflow: ${got}"
}

compare "disk cache" disk_cache "$disk_cache"
compare "repository cache" repository_cache "$repository_cache"

# bazelisk's download directory, pinned by BAZELISK_HOME. The workflow's side of
# that pin is what this checks: that the value the job exported is absolute and
# equal to a store the job archives. Both halves bite on a warm run, where the
# empty-store check below cannot: a restore repopulates the archived directory
# whatever bazelisk did with it.
#
# Absolute is a requirement rather than something to expand: bazelisk is Go and
# takes a leading tilde literally, so a tilde arriving here means the launcher is
# being written to a directory named `~` in the workspace while the archived path
# stays populated from the restore. Expanding it would report a match and go
# green on exactly that break.
case $launcher_home in
/*) ;;
*) die "BAZELISK_HOME is not an absolute path: ${launcher_home}" \
	"bazelisk does not expand a leading tilde; the launcher would land in the workspace." ;;
esac
launcher_matched=
for store in "${stores[@]}"; do
	if [ "$(expand_home "$store")" = "$launcher_home" ]; then
		launcher_matched=1
		break
	fi
done
[ -n "$launcher_matched" ] || die \
	"BAZELISK_HOME is not one of the launcher stores this job archives: ${launcher_home}" \
	"Bazel's launcher would be re-downloaded on every run, and nothing else would say so." \
	"launcher stores: ${stores[*]}"

report_store() {
	local dir
	dir=$(expand_home "$1")
	if [ -z "$(find "$dir" -type f -print -quit 2>/dev/null)" ]; then
		die "store is empty or absent: ${dir}" \
			"Nothing wrote it this run and no archive restored it, so this job is uncached."
	fi
	printf '%s: %s\n' "$dir" "$(du -sh -- "$dir" | cut -f1)"
}

report_store "$disk_cache"
report_store "$repository_cache"
for store in "${stores[@]}"; do
	report_store "$store"
done
