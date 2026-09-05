#!/usr/bin/env bash
#
# tools/tokio-quarantine.test.sh — who in this tree links an async runtime,
# held to the list of targets allowed to.
#
# The subject is this repository's dependency graph, not a script. `README.md`
# states the constraint: the motion and control stack holds no runtime at all,
# and the voice host binary is the one exception, quarantined off that path.
# That fact lives nowhere a file reader can see it. A crate that grows a
# `tokio` edge — directly, or through a library it already depended on for
# something else — compiles, tests green, and ships to the unit with a runtime
# inside it; the discovery is a motion binary that spawns threads on a machine
# whose control loop is a blocking loop by design. `//bazel/platform` is where
# that lands, so the two filegroups that decide what ships are named here
# individually rather than by package.
#
# So: reverse deps over the whole tree of the tokio library itself, this tree's
# own labels kept, and every one of them matched against the allowlist below. A
# new consumer means editing that list in a reviewed diff.
#
# Unlike every other self-check in `make test-scripts`, this one invokes the
# real `bazel` rather than reading a file or driving a stub. That is a
# deliberate trade: the fact it guards exists only in the dependency graph.
# `make check` already requires bazel and runs `bazel test //...` immediately
# after this step, so the workspace load this query pays for is the one the test
# step reuses and a cold cache fetches externals once for both. The cost added
# to the commit hook is one warm query.
#
# Run as a plain program; exits 0 on pass, non-zero on failure. Builds nothing.
#
# The labels it matches come from that query, unless `TOKIO_RDEPS_LABELS` names
# a file of them. That knob is not an operator's: it is how the fixture cases at
# the foot of this file run this same script over canned label lists, so the
# matcher is held to detect what it claims to detect rather than only to pass on
# a healthy graph. Those cases reach no real bazel -- the last of them drives a
# stub that refuses -- so the allowlist logic and the query's own failure path
# stay in the hermetic lane even though the live run is not.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"

# ---------------------------------------------------------------------------
# The allowlist
# ---------------------------------------------------------------------------

# One shell pattern per target allowed to reach tokio, each with the reason it
# is allowed. A label matching none of these fails the run.
allowed_label() {
	case $1 in
	# The voice host: the one crate that names the `tokio` spec, for the
	# network edges it owns. Its library, its binary and their tests.
	//crates/reachy-host:*) return 0 ;;
	# The two filegroups that decide what ships to a unit. They aggregate the
	# voice host for the deploy and link nothing themselves — but a new
	# tokio-linking target in `//bazel/platform` is exactly the drift this
	# gate exists to catch, so they are named and the package is not.
	//bazel/platform:device_deployables) return 0 ;;
	//bazel/platform:motion_payload) return 0 ;;
	# The run report: an offline operator analyzer that shares the host
	# library's reader types. It runs on a workstation over a fetched run and
	# on no unit.
	//cogs:speech_run_report) return 0 ;;
	//cogs:speech_run_report_test) return 0 ;;
	*) return 1 ;;
	esac
}

# ---------------------------------------------------------------------------
# The labels
# ---------------------------------------------------------------------------

# This tree's own labels are kept and everything else is dropped, rather than
# the other way round: `@brenn_reachy_crates//:tokio` is in its own rdeps and the
# crate universe's internal edges are not this tree's to hold, but a *deny*
# filter over `@` would hand a Bazel release the power to empty this gate.
# Canonical repository names are `@@`-prefixed under bzlmod and the main
# repository's canonical form is `@@//pkg:target`; which form `--output=label`
# emits is a Bazel version detail and `.bazelversion` moves. So a main-repo label
# in canonical form is normalized back to `//`, and only `//` labels survive.
keep_this_trees_labels() {
	sed -e 's|^@@//|//|' | grep '^//' || true
}

if [ -n "${TOKIO_RDEPS_LABELS:-}" ]; then
	labels=$(keep_this_trees_labels <"$TOKIO_RDEPS_LABELS")
else
	if ! command -v bazel >/dev/null 2>&1; then
		# The same refusal `make check`'s `require-bazel` prints. Never a skip:
		# a gate that passes when its only reader is missing is not a gate.
		echo "bazel not found on PATH." >&2
		echo "Install bazelisk; .bazelversion pins the Bazel release it fetches." >&2
		exit 1
	fi
	# A query that fails and a query that names nothing both come back with no
	# labels, and they want opposite diagnoses, so the exit status is kept and
	# bazel's own stderr is held for the failure message rather than discarded.
	query_err="${work}/bazel-query.err"
	query_status=0
	# The subject is the tokio *library*, not the spec's alias.
	# `@brenn_reachy_crates//:tokio` is an alias for the versioned crate-universe
	# target, and reverse deps of an alias are only the targets that name the
	# alias directly. Crate-universe crates point at the versioned target, so a
	# tree target reaching tokio through one of them is invisible to the alias's
	# own rdeps. Depth-1 `deps` of the alias is the alias plus what it resolves
	# to, and `kind` keeps the library, so the version stays out of this file.
	labels=$(cd -- "$(checkout_root)" &&
		bazel query \
			'rdeps(//..., kind("rust_library", deps(@brenn_reachy_crates//:tokio, 1)))' \
			--output=label 2>"$query_err") || query_status=$?
	if [ "$query_status" -ne 0 ]; then
		mapfile -t query_err_lines <"$query_err"
		fail "the query over the dependency graph runs" \
			"bazel query exited ${query_status}; its stderr follows" \
			"${query_err_lines[@]}"
		tally
		exit 1
	fi
	labels=$(printf '%s\n' "$labels" | keep_this_trees_labels)
fi

# ---------------------------------------------------------------------------
# The matcher
# ---------------------------------------------------------------------------

# A reader that matched nothing would pass every case below, so the set being
# non-empty is the first assertion. It also catches the query's own name for
# the spec going stale: a renamed crate repository answers with an empty set
# rather than an error.
if [ -n "$labels" ]; then
	pass "the query names the targets that reach tokio"
else
	fail "the query names the targets that reach tokio" \
		"it named none, and the voice host reaches it" \
		"@brenn_reachy_crates//:tokio is what the crate universe calls the spec;" \
		"a rename answers with an empty set rather than an error"
fi

# The one label a healthy graph must always answer with: the voice host's
# library is what names the spec. A reader that went half blind -- a query
# narrowed back to the alias, a filter that ate too much -- is not empty, so the
# assertion above would not catch it; this one does.
if contains "$labels" "//crates/reachy-host:reachy_host_lib"; then
	pass "the voice host's own library is among them"
else
	fail "the voice host's own library is among them" \
		"//crates/reachy-host:reachy_host_lib names the tokio spec and must be" \
		"in any answer, so a reader that misses it is reading the wrong subject"
fi

foreign=""
while IFS= read -r label; do
	[ -n "$label" ] || continue
	allowed_label "$label" || foreign="${foreign}${foreign:+ }${label}"
done <<<"$labels"

if [ -z "$foreign" ]; then
	pass "every target that reaches tokio is one the quarantine allows"
else
	fail "every target that reaches tokio is one the quarantine allows" \
		"these do not: ${foreign}" \
		"the motion and control stack holds no runtime; the voice host is the" \
		"one exception (README.md). Either drop the edge, or add the target to" \
		"the allowlist in this script with the reason it is allowed"
fi

# ---------------------------------------------------------------------------
# The matcher, held to detecting what it claims to detect
# ---------------------------------------------------------------------------
#
# Everything above runs over the live graph, so on its own it proves only that
# the tree is well formed today -- a matcher whose allowlist had grown a `*`,
# or whose loop never ran, would pass exactly the same. So each shape is a
# canned label list this script is re-run over as a subprocess through the knob
# (its tally is a global, and a subprocess is what keeps the fixtures' failures
# out of this run's count). No bazel is involved in any of them.
#
# Guarded on a marker every spawn below exports, so the subprocesses do not
# recurse. The knob selects a case's input; this alone decides recursion, so a
# case that leaves the knob unset (the refusing-bazel one does) is still a leaf
# and no future change to the query path can turn one into a fork bomb.
if [ -z "${TOKIO_QUARANTINE_FIXTURE:-}" ]; then
	# The set the tree answers with today, plus the external the reader drops.
	healthy_labels() {
		cat <<-'LABELS'
			//bazel/platform:device_deployables
			//bazel/platform:motion_payload
			//cogs:speech_run_report
			//cogs:speech_run_report_test
			//crates/reachy-host:example_params_test
			//crates/reachy-host:host_closure_test
			//crates/reachy-host:reachy_host
			//crates/reachy-host:reachy_host_bin_test
			//crates/reachy-host:reachy_host_lib
			//crates/reachy-host:reachy_host_test
			//crates/reachy-host:speech_fixture
			//crates/reachy-host:stt_compare
			//crates/reachy-host:stt_compare_test
			@brenn_reachy_crates//:tokio
		LABELS
	}

	# One fixture: this same script re-run under `case_env` (the caller sets
	# that array), the exit status it should reach, the case it should have
	# failed, and any number of strings its output must carry. The case is
	# matched as a `FAIL:` line and not as bare text, because the case labels
	# are also the text of the `PASS:` lines a healthy list prints; the needles
	# are matched as bare text, being labels and bazel's own words.
	#
	# The knob is cleared before `case_env` applies it, so a case that means to
	# drive the live query path (the refusing-bazel one) is not handed a label
	# file this run inherited.
	#
	# `TMPDIR` puts each child's staging tree inside this run's, because a
	# child sources `test-lib.sh` too and keeps its tree when it fails -- and
	# most of these children are meant to fail. Nested, the parent's trap takes
	# them with it, and keeps them in the one case where they are wanted.
	children="${work}/fixtures"
	mkdir -p -- "$children"
	over_env() {
		local label=$1 want=$2 case_failed=$3
		shift 3
		local out status=0 needle
		out=$(env -u TOKIO_RDEPS_LABELS TOKIO_QUARANTINE_FIXTURE=1 \
			TMPDIR="$children" \
			"${case_env[@]}" "${BASH_SOURCE[0]}" 2>&1) || status=$?
		if [ "$status" != "$want" ]; then
			fail "$label" \
				"the suite exited ${status}, expected ${want}" \
				"${out}"
			return
		fi
		if [ -n "$case_failed" ] && ! contains "$out" "FAIL: ${case_failed}"; then
			fail "$label" \
				"the suite failed, but not on '${case_failed}'" \
				"${out}"
			return
		fi
		for needle in "$@"; do
			if ! contains "$out" "$needle"; then
				fail "$label" \
					"the failure did not name ${needle}" \
					"${out}"
				return
			fi
		done
		for needle in "${case_absent[@]}"; do
			if contains "$out" "$needle"; then
				fail "$label" \
					"the output carries ${needle}, which it must not" \
					"${out}"
				return
			fi
		done
		pass "$label"
	}

	# The strings a child must *not* print, set beside `case_env` by the cases
	# that care and empty for the rest.
	case_absent=()

	# The common shape: a canned label list through the knob.
	over_labels() {
		local label=$1 file=$2 want=$3 case_failed=$4 named=${5:-}
		case_env=(TOKIO_RDEPS_LABELS="$file")
		case_absent=()
		if [ -n "$named" ]; then
			over_env "$label" "$want" "$case_failed" "$named"
		else
			over_env "$label" "$want" "$case_failed"
		fi
	}

	healthy_labels >"${work}/healthy"
	over_labels "the set this tree answers with today passes" \
		"${work}/healthy" 0 ""

	# A reader that stopped matching: nothing to check, and nothing said.
	: >"${work}/empty"
	over_labels "a query that named nothing is caught" \
		"${work}/empty" 1 "the query names the targets that reach tokio"

	# The drift the gate is for: a cog that grew a runtime.
	{
		healthy_labels
		echo "//cogs:some_cog"
	} >"${work}/foreign"
	over_labels "a target outside the allowlist is caught, and named" \
		"${work}/foreign" 1 \
		"every target that reaches tokio is one the quarantine allows" \
		"//cogs:some_cog"

	# The same, from inside the package whose two filegroups are allowed by
	# name: a third deployable target is not covered by them.
	{
		healthy_labels
		echo "//bazel/platform:other"
	} >"${work}/platform-other"
	over_labels "an unlisted //bazel/platform target is caught" \
		"${work}/platform-other" 1 \
		"every target that reaches tokio is one the quarantine allows" \
		"//bazel/platform:other"

	# The same drift wearing bzlmod's canonical name for the main repository.
	# The reader keeps `//` labels rather than dropping `@` ones precisely so
	# this is caught rather than filtered away as an external.
	{
		healthy_labels
		echo "@@//cogs:some_cog"
	} >"${work}/canonical-foreign"
	over_labels "a foreign label in canonical @@// form is caught, not dropped" \
		"${work}/canonical-foreign" 1 \
		"every target that reaches tokio is one the quarantine allows" \
		"//cogs:some_cog"

	# A bazel that fails is not a graph that named nothing.
	stub_bin="${work}/stub-bin"
	mkdir -p -- "$stub_bin"
	cat >"${stub_bin}/bazel" <<-'STUB'
		#!/usr/bin/env bash
		echo "ERROR: no such package '@brenn_reachy_crates'" >&2
		exit 7
	STUB
	chmod +x -- "${stub_bin}/bazel"

	# No knob here: this is the live query path, over a bazel that refuses.
	case_env=(PATH="${stub_bin}:${PATH}")
	over_env "a query that fails is told apart from one that named nothing" \
		1 "the query over the dependency graph runs" \
		"bazel query exited 7" \
		"ERROR: no such package '@brenn_reachy_crates'"

	# The last case has no bazel at all. Its `PATH` is a directory of symlinks to
	# the tools the script reaches before the refusal, rather than the inherited
	# `PATH` with bazel's own directory cut out: that cut takes the coreutils
	# with it where bazel lives in `/usr/bin`, and removes no bazel at all where
	# `/bin` is a symlink to `/usr/bin`. Named by tool, the case is the same
	# everywhere.
	nobazel_bin="${work}/nobazel-bin"
	mkdir -p -- "$nobazel_bin"
	for tool in bash env dirname mktemp rm sed grep cat; do
		tool_path=$(command -v "$tool") || continue
		ln -s -- "$tool_path" "${nobazel_bin}/${tool}"
	done

	# A missing reader must be a refusal and never a skip: the difference is one
	# `exit` argument, and no other case takes this branch. So the two lines
	# `require-bazel` prints, and no `PASS:` line anywhere in the output — a
	# skip would be a run that passed everything it looked at.
	case_env=(PATH="$nobazel_bin")
	case_absent=("PASS:")
	over_env "a missing bazel is a refusal, not a skip" \
		1 "" \
		"bazel not found on PATH." \
		".bazelversion pins the Bazel release it fetches."
fi

tally
