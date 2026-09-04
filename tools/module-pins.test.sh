#!/usr/bin/env bash
#
# tools/module-pins.test.sh — how MODULE.bazel names brenn-pod, held to the form
# a machine that is not this one can resolve.
#
# The subject is this repository's own `MODULE.bazel`, not a script. It is here
# because the failure it catches cannot be caught anywhere else in the gate: a
# `crate.spec` switched from `git`/`rev` to `path = "../brenn-pod/..."` — the
# overlay a seam landing on both sides of the dependency arrow is developed
# through — resolves perfectly on the developer's workstation, so `make check`
# is green on a tree that no fresh clone and no runner can load. `CLAUDE.md`
# names the local gate as the gate and CI as the backstop; without this case the
# order is inverted for exactly this class of change, and the discovery is a red
# main or a contributor who cannot build the published tree.
#
# So: every `crate.spec` naming a brenn-pod package resolves from the published
# remote at the pinned revision, both spelled once as constants, and none of
# them names a path. The count of specs found is asserted too, because a reader
# that matched nothing would otherwise pass.
#
# Run as a plain program; exits 0 on pass, non-zero on failure. Reads one file
# and builds nothing.
#
# The file it reads is this checkout's, unless `MODULE_PINS_FILE` names another
# one. That knob is not an operator's: it is how the fixture cases at the foot of
# this file run this same script over deliberately broken files, so the reader
# above is held to detect what it claims to detect rather than only to pass on a
# healthy tree.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"
# For `pinned_pod_rev`: the build's own reader of the pin, held below against
# this suite's independent parse of the same line. Two parsers of one line drift,
# and the build's is the one whose silence reads as agreement.
# shellcheck source=lib.sh
. "${script_dir}/lib.sh"

module="${MODULE_PINS_FILE:-$(checkout_root)/MODULE.bazel}"

# The packages this tree takes from brenn-pod. Named here rather than derived
# from the file, so a spec that quietly loses its remote is a missing spec
# rather than a package this suite stops looking for.
pod_packages=" speech-surface speech-pipeline brenn-bridge pod-ingest "

# Every `crate.spec` block, one tab-separated record each: package, git, rev,
# path. A line reader over the literals a person typed, not a parse of Starlark
# — comment lines are skipped, so a spec kept as commented text is not a spec.
# Values are taken bare, so a constant reads as its name and a literal as its
# contents; an attribute a spec does not carry reads as `-`.
specs=$(awk '
	BEGIN { OFS = "\t" }
	# A tab is an IFS whitespace character, so an empty field would be
	# collapsed by the reader below rather than read as empty. Missing
	# attributes are named instead.
	function blank(v) { return v == "" ? "-" : v }
	/^#/ { next }
	/^crate\.spec\($/ { inspec = 1; pkg = ""; git = ""; rev = ""; path = ""; next }
	inspec && /^\)/ {
		if (pkg != "") print pkg, blank(git), blank(rev), blank(path)
		inspec = 0
		next
	}
	inspec {
		line = $0
		sub(/^[ \t]+/, "", line)
		sub(/[ \t]+$/, "", line)
		if (line !~ / = /) next
		key = line
		sub(/ = .*/, "", key)
		val = line
		sub(/^[^=]* = /, "", val)
		sub(/,$/, "", val)
		gsub(/"/, "", val)
		if (key == "package") pkg = val
		else if (key == "git") git = val
		else if (key == "rev") rev = val
		else if (key == "path") path = val
	}
' "$module")

# ---------------------------------------------------------------------------
# The four specs
# ---------------------------------------------------------------------------

found=0
while IFS=$'\t' read -r pkg git rev path; do
	case $pod_packages in
	*" ${pkg} "*) ;;
	*) continue ;;
	esac
	found=$((found + 1))
	assert_eq "the ${pkg} spec resolves from the pinned remote" \
		"BRENN_POD_GIT" "$git"
	assert_eq "the ${pkg} spec resolves at the pinned revision" \
		"BRENN_POD_REV" "$rev"
	if [ "$path" != "-" ]; then
		fail "the ${pkg} spec names no working tree" \
			"it resolves from ${path}, which is a directory only this machine has" \
			"put the four specs back on git = BRENN_POD_GIT, rev = BRENN_POD_REV" \
			"before committing; MODULE.bazel says what else comes back with them"
	else
		pass "the ${pkg} spec names no working tree"
	fi
done <<<"$specs"

assert_eq "all four brenn-pod specs are there to be read" 4 "$found"

# ---------------------------------------------------------------------------
# What the two constants say
# ---------------------------------------------------------------------------

git_remote=$(sed -n 's/^BRENN_POD_GIT = "\(.*\)".*/\1/p' -- "$module" | head -n 1)
assert_eq "the remote is the public one, spelled once" \
	"https://github.com/rnortman/brenn-pod.git" "$git_remote"

pinned=$(sed -n 's/^BRENN_POD_REV = "\(.*\)".*/\1/p' -- "$module" | head -n 1)
if [[ $pinned =~ ^[0-9a-f]{40}$ ]]; then
	pass "the revision is a full commit id"
else
	fail "the revision is a full commit id" \
		"BRENN_POD_REV is '${pinned}'" \
		"a branch or a short id is not a revision another machine resolves the same way"
fi

# The build's own reader of that same line, held to this suite's answer. A
# `tools/lib.sh` whose sed stops matching this file goes on printing "states no
# BRENN_POD_REV this can read" on every build -- the sentence its own comment
# says must never read as agreement -- with nothing else in the gate noticing.
assert_eq "the build's provenance line reads the same revision off this file" \
	"$pinned" "$(pinned_pod_rev "$module")"

# The other half of the overlay, which leaves no `crate.spec` behind: a module
# resolved from somewhere off this machine's remotes by an override. Every
# override form is looked at, not `local_path_override` alone -- `git_override`
# at a working-tree URL, an `archive_override`, a `single_version_override`
# carrying patches all resolve a module the same way -- and the whole block is
# read to its closing paren rather than a fixed window of lines, so an attribute
# order or a comment inside the call cannot hide the name. Only `module_name`
# and `path` are matched, so the rusty_cogs `git_override` beside them and any
# comment near one are not a false alarm.
overrides=$(awk '
	/^#/ { next }
	/^[a-z_]*_override\($/ { inblock = 1; kind = $0; sub(/\($/, "", kind); next }
	inblock && /^\)/ { inblock = 0; next }
	inblock {
		line = $0
		sub(/^[ \t]+/, "", line)
		sub(/[ \t]+$/, "", line)
		if (line !~ / = /) next
		key = line
		sub(/ = .*/, "", key)
		val = line
		sub(/^[^=]* = /, "", val)
		sub(/,$/, "", val)
		gsub(/"/, "", val)
		if ((key == "module_name" || key == "path") && val ~ /brenn-pod/)
			print kind "\t" key "\t" val
	}
' "$module")

if [ -n "$overrides" ]; then
	fail "no module override resolves brenn-pod from a working tree" \
		"${module} carries: ${overrides}" \
		"an override of any kind resolves the module off this machine's published remote"
else
	pass "no module override resolves brenn-pod from a working tree"
fi

# ---------------------------------------------------------------------------
# The reader, held to detecting what it claims to detect
# ---------------------------------------------------------------------------
#
# Everything above runs over a healthy file, so on its own it proves only that
# the tree is well formed today -- a reader that matched nothing, or that lost
# the `path` key from its attribute switch, would pass exactly the same. So each
# broken shape is a fixture file this script is re-run over as a subprocess (its
# own tally is a global, and a subprocess is what keeps the fixtures' failures
# out of this run's count).
#
# Guarded on the knob being unset, so the subprocesses do not recurse.
if [ -z "${MODULE_PINS_FILE:-}" ]; then
	fixtures=$(mktemp -d)
	trap 'rm -rf -- "$fixtures"' EXIT

	# A well-formed file in the shape of the real one: the two constants, and
	# four specs resolving from them. The cases below mutate copies of it.
	healthy_module() {
		cat <<-'MODULE'
			BRENN_POD_GIT = "https://github.com/rnortman/brenn-pod.git"

			BRENN_POD_REV = "f5fa3f77116706203adffdc28c61accdaa8e77de"

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
		MODULE
	}

	# One fixture: a file, the exit status this suite should reach over it, and
	# the case it should have failed. The case is matched as a `FAIL:` line and
	# not as bare text, because every one of these labels is also the text of
	# the `PASS:` line the healthy fixtures print.
	over_fixture() {
		local label=$1 file=$2 want=$3 case_failed=$4
		local out status=0
		out=$(MODULE_PINS_FILE="$file" "${BASH_SOURCE[0]}" 2>&1) || status=$?
		if [ "$status" != "$want" ]; then
			fail "$label" \
				"the suite exited ${status} over ${file}, expected ${want}" \
				"${out}"
			return
		fi
		if [ -n "$case_failed" ] && ! contains "$out" "FAIL: ${case_failed}"; then
			fail "$label" \
				"the suite failed, but not on '${case_failed}'" \
				"${out}"
			return
		fi
		pass "$label"
	}

	healthy_module >"${fixtures}/healthy"
	over_fixture "a well-formed file passes" "${fixtures}/healthy" 0 ""

	# The overlay itself: one spec resolved from a working tree.
	healthy_module |
		sed '5,8s|    git = BRENN_POD_GIT,|    path = "../brenn-pod/host/crates/speech-surface",|; 5,8s|    rev = BRENN_POD_REV,||' \
			>"${fixtures}/overlaid"
	over_fixture "a spec on a working-tree path is caught" \
		"${fixtures}/overlaid" 1 "the speech-surface spec names no working tree"

	# A spec that resolves from the right remote by spelling it out, which is
	# how a bump reaches some specs and not the rest.
	healthy_module |
		sed '5,9s|    git = BRENN_POD_GIT,|    git = "https://github.com/rnortman/brenn-pod.git",|' \
			>"${fixtures}/literal-remote"
	over_fixture "a spec naming the remote itself is caught" \
		"${fixtures}/literal-remote" 1 \
		"the speech-surface spec resolves from the pinned remote"

	# The same for the revision.
	healthy_module |
		sed '5,9s|    rev = BRENN_POD_REV,|    rev = "f5fa3f77116706203adffdc28c61accdaa8e77de",|' \
			>"${fixtures}/literal-rev"
	over_fixture "a spec naming a revision itself is caught" \
		"${fixtures}/literal-rev" 1 \
		"the speech-surface spec resolves at the pinned revision"

	# A spec gone: what the count assertion is for.
	healthy_module | sed '/brenn-bridge/,+2d' >"${fixtures}/missing-spec"
	over_fixture "a spec that is no longer there is caught" \
		"${fixtures}/missing-spec" 1 \
		"all four brenn-pod specs are there to be read"

	# A pin another machine does not resolve the same way.
	healthy_module | sed 's|^BRENN_POD_REV = .*|BRENN_POD_REV = "main"|' \
		>"${fixtures}/branch-pin"
	over_fixture "a branch name in place of a revision is caught" \
		"${fixtures}/branch-pin" 1 "the revision is a full commit id"

	# The three override shapes, each resolving the module off the published
	# remote.
	{
		healthy_module
		printf '\nlocal_path_override(\n    # the sibling checkout\n\n    module_name = "brenn_pod",\n    path = "../brenn-pod",\n)\n'
	} >"${fixtures}/local-override"
	over_fixture "a local_path_override on brenn-pod is caught" \
		"${fixtures}/local-override" 1 \
		"no module override resolves brenn-pod from a working tree"

	{
		healthy_module
		printf '\narchive_override(\n    urls = ["file:///tmp/x.tar"],\n    module_name = "brenn-pod",\n)\n'
	} >"${fixtures}/archive-override"
	over_fixture "an archive_override on brenn-pod is caught" \
		"${fixtures}/archive-override" 1 \
		"no module override resolves brenn-pod from a working tree"

	{
		healthy_module
		printf '\nsingle_version_override(\n    module_name = "brenn-pod",\n    patches = ["//:x.patch"],\n)\n'
	} >"${fixtures}/version-override"
	over_fixture "a single_version_override on brenn-pod is caught" \
		"${fixtures}/version-override" 1 \
		"no module override resolves brenn-pod from a working tree"

	# Negative case: another module's override, with brenn-pod named only in a
	# comment beside the block.
	{
		healthy_module
		printf '\n# nothing to do with brenn-pod\nlocal_path_override(\n    module_name = "rusty_cogs",\n    path = "../rusty-cogs",\n)\n'
	} >"${fixtures}/other-override"
	over_fixture "another module's override is not mistaken for one" \
		"${fixtures}/other-override" 0 ""
fi

tally
