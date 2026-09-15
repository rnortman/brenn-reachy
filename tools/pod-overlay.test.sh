#!/usr/bin/env bash
#
# tools/pod-overlay.test.sh — self-check for pod-overlay.sh.
#
# The subject rewrites the one file every build of this tree resolves its
# brenn-pod crates from, so what is pinned here is that the rewrite is total and
# reversible: all four specs or none, `off` giving back the bytes `on` started
# from, and a file it cannot read fully left exactly as it was. A half-rewritten
# `MODULE.bazel` is a voice host linking two revisions of brenn-pod, which is
# the state the build's own refusals exist to make unreachable — reaching it
# from this side would put the mixed form *into* the file the gate reads.
#
# Nothing here touches this repository's own MODULE.bazel: every case runs the
# subject over a fixture through MODULE_OVERLAY_FILE, out of a fixture tree with
# a fixture brenn-pod checkout beside it.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"

# ---------------------------------------------------------------------------
# The tree the subject runs out of, and the checkout it overlays
# ---------------------------------------------------------------------------

repo="${work}/repo"
mkdir -p -- "${repo}/tools"
cp -- "${script_dir}/pod-overlay.sh" "${script_dir}/lib.sh" "${repo}/tools/"
subject="${repo}/tools/pod-overlay.sh"

# The brenn-pod checkout as a sibling, with the four crate directories the specs
# name and a git repository around them — `off` asks git there which generated
# files are its own.
pod="${work}/brenn-pod"
for dir in host/crates/speech-surface host/crates/speech-pipeline \
	host/crates/pod-ingest firmware/crates/brenn-bridge; do
	mkdir -p -- "${pod}/${dir}"
	printf '[package]\nname = "x"\n' >"${pod}/${dir}/Cargo.toml"
done
git -C "$pod" init -q
git -C "$pod" add -A
git -C "$pod" -c user.email=t@example.invalid -c user.name=t commit -qm fixture

export BRENN_POD_DIR="$pod"

# A `MODULE.bazel` in the shape this repository's own is in: the two constants,
# the four brenn-pod specs in block form, and one spec from somewhere else that
# nothing here may touch.
pinned_fixture() {
	cat <<'BAZEL'
module(name = "fixture")

BRENN_POD_GIT = "https://example.invalid/brenn-pod.git"

BRENN_POD_REV = "0123456789abcdef0123456789abcdef01234567"

# The host library.
crate.spec(
    git = BRENN_POD_GIT,
    package = "speech-surface",
    rev = BRENN_POD_REV,
)

crate.spec(
    git = BRENN_POD_GIT,
    package = "speech-pipeline",
    rev = BRENN_POD_REV,
)

crate.spec(
    git = BRENN_POD_GIT,
    package = "brenn-bridge",
    rev = BRENN_POD_REV,
)

crate.spec(
    features = ["test-util"],
    git = BRENN_POD_GIT,
    package = "pod-ingest",
    rev = BRENN_POD_REV,
)

# Somebody else's dependency, which no overlay of brenn-pod may reach.
crate.spec(
    package = "serde",
    version = "1",
)
BAZEL
}

module="${work}/MODULE.bazel"

# The subject over a named file, with both halves of its run captured. A file
# per case rather than one the cases take turns on: a fixture the subject
# refused must be readable afterwards to show it was left alone.
overlay_file() {
	local file=$1 out status=0
	shift
	out=$(cd -- "$repo" && MODULE_OVERLAY_FILE="$file" "$subject" "$@" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

# The round-trip fixture, which most cases run over.
overlay() { overlay_file "$module" "$@"; }

output_of() { sed '$d' <<<"$1"; }
status_of() { sed -n '$s/^---status //p' <<<"$1"; }

# ---------------------------------------------------------------------------
# The round trip
# ---------------------------------------------------------------------------

pinned_fixture >"$module"
before=$(cat -- "$module")

result=$(overlay status)
assert_status "status reads a pinned file" 0 "$(status_of "$result")"
assert_contains "and names the revision" "$(output_of "$result")" "pinned at 0123456789ab"

result=$(overlay on)
assert_status "on succeeds" 0 "$(status_of "$result")"
after=$(cat -- "$module")
for pkg in speech-surface speech-pipeline pod-ingest; do
	assert_contains "${pkg} resolves from the working tree" "$after" \
		"path = \"../brenn-pod/host/crates/${pkg}\""
done
assert_contains "brenn-bridge resolves from its own half of that checkout" "$after" \
	'path = "../brenn-pod/firmware/crates/brenn-bridge"'
assert_lacks "no brenn-pod spec is left on the remote" "$after" "git = BRENN_POD_GIT"
assert_lacks "and none on the pinned revision" "$after" "rev = BRENN_POD_REV"
# The pin itself stays: the constant is what `off` puts the specs back on, and
# `refuse_unless_pod_checkout_matches` still reads the file that carries it.
assert_contains "the pin's own line is untouched" "$after" \
	'BRENN_POD_REV = "0123456789abcdef0123456789abcdef01234567"'
assert_contains "a dependency from elsewhere is untouched" "$after" 'package = "serde"'
assert_contains "as is its version" "$after" 'version = "1"'
assert_contains "the features of an overlaid spec survive" "$after" \
	'features = ["test-util"]'
# What an operator is told to do next, because the two things that outlive the
# command are the lock and the push.
assert_contains "on says the lock is not to be committed" "$(output_of "$result")" \
	"leave it uncommitted"
assert_contains "and how to put the pin back" "$(output_of "$result")" \
	"tools/pod-overlay.sh off"

result=$(overlay on)
assert_status "on again is not an error" 0 "$(status_of "$result")"
assert_contains "and says the file is already overlaid" "$(output_of "$result")" \
	"already overlaid"
assert_eq "with nothing rewritten a second time" "$after" "$(cat -- "$module")"

result=$(overlay status)
assert_contains "status reads an overlaid file" "$(output_of "$result")" \
	"overlaid from ../brenn-pod"

result=$(overlay off)
assert_status "off succeeds" 0 "$(status_of "$result")"
assert_eq "and gives back exactly the file on came from" "$before" "$(cat -- "$module")"
assert_contains "saying which revision the specs are back on" "$(output_of "$result")" \
	"back on BRENN_POD_REV (0123456789ab)"
assert_contains "and how to restore the lock" "$(output_of "$result")" \
	"checkout -- MODULE.bazel.lock"

result=$(overlay off)
assert_status "off on a pinned file is not an error" 0 "$(status_of "$result")"
assert_contains "and says so" "$(output_of "$result")" "already pinned"

# ---------------------------------------------------------------------------
# The overlay's litter in the other checkout
# ---------------------------------------------------------------------------
#
# A `path =` spec makes bazel write a generated BUILD.bazel into each overlaid
# crate directory over there. Untracked and this tree's mess: cleared by `off`,
# so a pinned build after an overlay round does not resolve those directories
# through files nobody wrote.

overlay on >/dev/null
: >"${pod}/host/crates/speech-surface/BUILD.bazel"
: >"${pod}/firmware/crates/brenn-bridge/BUILD.bazel"
# One that the other repository tracks itself, which is not ours to delete.
printf '# theirs\n' >"${pod}/host/crates/pod-ingest/BUILD.bazel"
git -C "$pod" add -f host/crates/pod-ingest/BUILD.bazel
git -C "$pod" -c user.email=t@example.invalid -c user.name=t commit -qm theirs

result=$(overlay off)
assert_status "off succeeds with litter to clear" 0 "$(status_of "$result")"
assert_no_file "the generated BUILD.bazel is gone" \
	"${pod}/host/crates/speech-surface/BUILD.bazel"
assert_no_file "in every overlaid crate directory" \
	"${pod}/firmware/crates/brenn-bridge/BUILD.bazel"
assert_file "a tracked one over there is left alone" \
	"${pod}/host/crates/pod-ingest/BUILD.bazel"

# ---------------------------------------------------------------------------
# The files it refuses, and leaves alone
# ---------------------------------------------------------------------------

# Half overlaid and half pinned: the state the build refuses, arriving through
# the file. Neither direction guesses which half the operator meant.
mixed="${work}/mixed/MODULE.bazel"
mkdir -p -- "$(dirname -- "$mixed")"
cat >"$mixed" <<'BAZEL'
module(name = "fixture")

BRENN_POD_GIT = "https://example.invalid/brenn-pod.git"

BRENN_POD_REV = "0123456789abcdef0123456789abcdef01234567"

crate.spec(
    package = "speech-surface",
    path = "../brenn-pod/host/crates/speech-surface",
)

crate.spec(
    git = BRENN_POD_GIT,
    package = "speech-pipeline",
    rev = BRENN_POD_REV,
)
BAZEL
before_mixed=$(cat -- "$mixed")
for goal in status on off; do
	result=$(overlay_file "$mixed" "$goal")
	assert_status "a mixed file refuses ${goal}" 1 "$(status_of "$result")"
	assert_contains "naming what is wrong with it (${goal})" "$(output_of "$result")" \
		"neither form"
done
assert_eq "and leaving it exactly as it was" "$before_mixed" "$(cat -- "$mixed")"

# A spec written compactly is one `pod_overlay_paths` refuses to read past, so
# every subcommand dies at the entry guard that asks which form the file is in,
# before a rewrite is ever attempted: the file is left alone and the refusal
# names the form. The rewriter carries its own refusal for the same shape behind
# that guard, which is why the message asserted here is the guard's.
compact="${work}/compact/MODULE.bazel"
mkdir -p -- "$(dirname -- "$compact")"
{
	pinned_fixture
	printf '\ncrate.spec(package = "speech-surface", git = BRENN_POD_GIT, rev = BRENN_POD_REV)\n'
} >"$compact"
before_compact=$(cat -- "$compact")
result=$(overlay_file "$compact" on)
assert_status "a compact spec refuses" 1 "$(status_of "$result")"
assert_contains "naming the form it reads" "$(output_of "$result")" "neither form"
assert_eq "and the file is untouched" "$before_compact" "$(cat -- "$compact")"

# A file that names only three of the four packages. The rewrite is all four or
# none — three specs on the working tree and a fourth package resolved from
# wherever it comes from is the same two revisions inside one voice host — so
# the awk counts what it wrote, the temporary file is discarded on the count and
# the target keeps its bytes.
short="${work}/short/MODULE.bazel"
mkdir -p -- "$(dirname -- "$short")"
pinned_fixture | awk '
	/^crate\.spec\($/ { block = ""; inspec = 1 }
	inspec { block = block $0 "\n"; if ($0 ~ /^\)/) { inspec = 0; if (block !~ /"pod-ingest"/) printf "%s", block }; next }
	{ print }
' >"$short"
before_short=$(cat -- "$short")
result=$(overlay_file "$short" on)
assert_status "a file missing one of the four specs refuses" 1 "$(status_of "$result")"
assert_contains "saying not all four were rewritten" "$(output_of "$result")" \
	"were not all rewritten"
assert_eq "and the file is untouched" "$before_short" "$(cat -- "$short")"
assert_eq "with no temporary left beside it" "MODULE.bazel" \
	"$(ls -- "$(dirname -- "$short")")"

# A checkout with no crate where a spec would point: caught before the file is
# written, because an overlay onto a directory that is not there is a build that
# fails a long way from here.
missing="${work}/missing/MODULE.bazel"
mkdir -p -- "$(dirname -- "$missing")"
pinned_fixture >"$missing"
before_missing=$(cat -- "$missing")
bare_pod="${work}/bare-pod"
mkdir -p -- "$bare_pod"
result=$(BRENN_POD_DIR="$bare_pod" overlay_file "$missing" on)
assert_status "a checkout missing a crate refuses" 1 "$(status_of "$result")"
assert_contains "naming the directory it looked for" "$(output_of "$result")" \
	"host/crates/speech-surface is not there"
assert_eq "and the file is untouched" "$before_missing" "$(cat -- "$missing")"

result=$(overlay_file "${work}/nowhere/MODULE.bazel" status)
assert_status "a file that is not there refuses" 1 "$(status_of "$result")"
assert_contains "saying there is nothing to switch" "$(output_of "$result")" \
	"no specs to switch"

result=$(overlay sideways)
assert_status "an unknown goal refuses" 1 "$(status_of "$result")"
assert_contains "with the three it takes" "$(output_of "$result")" "on|off|status"

tally
