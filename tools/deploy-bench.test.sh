#!/usr/bin/env bash
#
# tools/deploy-bench.test.sh — self-check for deploy-bench.sh.
#
# The script under test reaches a device with ssh and rsync and reads the
# repository with git. All three are stubbed here, on PATH, recording what they
# were asked for; nothing in this file touches a network, a device, or this
# checkout. The subject is copied into a temporary tree beside its own lib.sh,
# so `repo_root` is that tree and the binary the deploy pushes is a file this
# test made.
#
# What is worth pinning: the freshness refusal and its escape hatch. A refusal
# that quietly stops refusing is discovered on the bench night it should have
# saved, which is how this check came to exist.
#
# Run as a plain program; exits 0 on pass, non-zero on failure.

set -euo pipefail

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)

# shellcheck source=test-lib.sh
. "${script_dir}/test-lib.sh"

# ---------------------------------------------------------------------------
# The tree the subject runs out of, and the stubs it finds on PATH.
# ---------------------------------------------------------------------------

repo="${work}/repo"
mkdir -p -- "${repo}/tools" "${repo}/target/bench-arm64/release"
cp -- "${script_dir}/deploy-bench.sh" "${script_dir}/lib.sh" "${repo}/tools/"

subject="${repo}/tools/deploy-bench.sh"
binary="${repo}/target/bench-arm64/release/reachy-bench"

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

# Where the stubs write what they were asked, and the knobs the cases turn.
export CALLS="${work}/calls"
export GIT_COMMIT_TIME=""
export SSH_RUN_STATUS=0
export SSH_CAT_STATUS=0
export SSH_TAR_STATUS=0
export TAR_OMITS=""
export PROBE_FILES=""

# Every stub records its whole invocation on one line, so a case can assert
# both that a command ran and that it did not.
cat >"${stubs}/ssh" <<'STUB'
#!/usr/bin/env bash
printf 'ssh %s\n' "$*" >>"$CALLS"
# The state file, which only a selftest writes: a device that has run one
# answers, and one that has not is the case's to say.
for arg in "$@"; do
	case "$arg" in
		"cat "*selftest-state.toml)
			[ "${SSH_CAT_STATUS:-0}" = 0 ] || exit "$SSH_CAT_STATUS"
			printf 'state of the bench\n'
			exit 0
			;;
	esac
done
# The listing of the probe's series: what a device holds is the case's to say,
# and an empty answer is a device where the probe never ran.
for arg in "$@"; do
	case "$arg" in
		*"ls -1"*hold-probe*) printf '%s\n' ${PROBE_FILES:-} ; exit 0 ;;
	esac
done
# The stream carrying the series: a tar of the names asked for, each holding a
# line that says which file it is, so a case can tell the fetched files apart.
# A case can make the stream fail outright, or leave one name out of it.
for arg in "$@"; do
	case "$arg" in
		"tar -cf -"*hold-probe*)
			[ "${SSH_TAR_STATUS:-0}" = 0 ] || exit "$SSH_TAR_STATUS"
			names=${arg##*-- }
			staging=$(mktemp -d)
			sent=()
			for name in $names; do
				[ "$name" = "${TAR_OMITS:-}" ] && continue
				printf 'series of %s\n' "$name" >"${staging}/${name}"
				sent+=("$name")
			done
			[ ${#sent[@]} -gt 0 ] && tar -cf - -C "$staging" -- "${sent[@]}"
			rm -rf -- "$staging"
			exit 0
			;;
	esac
done
# Only the run itself carries a status worth faking; the mkdir before it, and
# the fetch's cat, succeed.
for arg in "$@"; do
	case "$arg" in
		*reachy-bench*) exit "${SSH_RUN_STATUS:-0}" ;;
	esac
done
exit 0
STUB

cat >"${stubs}/rsync" <<'STUB'
#!/usr/bin/env bash
printf 'rsync %s\n' "$*" >>"$CALLS"
exit 0
STUB

# The repository question the freshness check asks. An empty GIT_COMMIT_TIME is
# a tree whose history says nothing about these paths.
cat >"${stubs}/git" <<'STUB'
#!/usr/bin/env bash
printf 'git %s\n' "$*" >>"$CALLS"
[ -n "${GIT_COMMIT_TIME:-}" ] || exit 0
echo "$GIT_COMMIT_TIME"
STUB

chmod 0755 -- "${stubs}/ssh" "${stubs}/rsync" "${stubs}/git"

# A device binary whose age is ours to set. Executable, because the subject
# refuses one that is not.
build_binary_at() {
	local when=$1
	: >"$binary"
	chmod 0755 -- "$binary"
	touch -d "@${when}" -- "$binary"
}

# Run the subject, answering with its output and its status, and start each
# case from an empty call record.
deploy() {
	: >"$CALLS"
	local out status=0
	out=$("$subject" "$@" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}


calls() { cat -- "$CALLS"; }

# Two times an hour apart, so "older" and "newer" are unambiguous whatever the
# filesystem's timestamp resolution is.
commit_at=1750000000
before=$((commit_at - 3600))
after=$((commit_at + 3600))

# ---------------------------------------------------------------------------
# Freshness
# ---------------------------------------------------------------------------

GIT_COMMIT_TIME=$commit_at
build_binary_at "$after"
result=$(deploy unit --run selftest)
assert_status "a binary newer than the newest commit runs" 0 "$(status_of "$result")"
assert_contains "the run reaches the device" "$(calls)" "reachy-bench"

assert_contains "the freshness question is asked of the workspace paths" "$(calls)" \
	"git -C ${repo} log -1 --format=%ct -- crates bazel MODULE.bazel MODULE.bazel.lock .bazelrc .bazelversion tools/build-bench.sh tools/lib.sh"

build_binary_at "$before"
result=$(deploy unit --run selftest)
assert_status "a binary older than the newest commit refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says what is wrong" "$(output_of "$result")" \
	"older than the newest commit"
assert_contains "the refusal names the build" "$(output_of "$result")" "make bench-build"
assert_contains "the refusal names the override" "$(output_of "$result")" "--stale-ok"
assert_lacks "a refused run pushes nothing" "$(calls)" "rsync"
assert_lacks "a refused run reaches no device" "$(calls)" "ssh"

result=$(deploy unit --run --stale-ok selftest)
assert_status "--stale-ok runs the old binary" 0 "$(status_of "$result")"
assert_contains "--stale-ok says the age went unchecked" "$(output_of "$result")" \
	"the binary's age is not being checked"
assert_contains "--stale-ok still runs the bench" "$(calls)" "reachy-bench selftest"
assert_lacks "--stale-ok is the script's flag, not the bench's" "$(calls)" "reachy-bench --stale-ok"

# A tree that cannot answer the question is not a stale tree.
GIT_COMMIT_TIME=""
build_binary_at "$before"
result=$(deploy unit --run selftest)
assert_status "no history means no verdict, and the run proceeds" 0 "$(status_of "$result")"
assert_contains "an undecidable age is said out loud" "$(output_of "$result")" \
	"the device binary's age is unknown"
GIT_COMMIT_TIME=$commit_at

# The check runs after the binary itself is accounted for, so a missing build
# is still reported as a missing build.
rm -f -- "$binary"
result=$(deploy unit --run selftest)
assert_status "a missing binary refuses" 1 "$(status_of "$result")"
assert_contains "a missing binary names the build" "$(output_of "$result")" "make bench-build"
assert_lacks "a missing binary is not reported as a stale one" "$(output_of "$result")" \
	"older than the newest commit"

# Nothing but --run has a binary to be stale.
build_binary_at "$before"
config="${work}/reachy-bench.toml"
echo 'device = "/dev/null"' >"$config"
result=$(deploy unit --config "$config")
assert_status "pushing a configuration is not a run" 0 "$(status_of "$result")"
assert_lacks "pushing a configuration asks no freshness question" "$(calls)" "git "

# ---------------------------------------------------------------------------
# What the run itself does with the device's answer
# ---------------------------------------------------------------------------

build_binary_at "$after"

SSH_RUN_STATUS=7
result=$(deploy unit --run selftest)
assert_status "the bench's own verdict is the script's" 7 "$(status_of "$result")"

SSH_RUN_STATUS=3
result=$(deploy unit --run selftest)
assert_status "a held bus refuses" 1 "$(status_of "$result")"
assert_contains "a held bus names the service holding it" "$(output_of "$result")" \
	"brenn-app.service is running"

SSH_RUN_STATUS=4
result=$(deploy unit --run selftest)
assert_status "the motion daemon holding the bus refuses" 1 "$(status_of "$result")"
assert_contains "the motion daemon is named with the way back" "$(output_of "$result")" \
	"systemctl start reachy-motiond.service"

SSH_RUN_STATUS=255
result=$(deploy unit --run selftest)
assert_status "ssh failing is not a hardware reading" 1 "$(status_of "$result")"
assert_contains "ssh failing says so" "$(output_of "$result")" "did not run"

SSH_RUN_STATUS=0

# ---------------------------------------------------------------------------
# What a fetch brings back
# ---------------------------------------------------------------------------

# Each case runs with its own device listing and its own destination, set here
# rather than left as file-scope state a later case would inherit: a stub that
# answers with the previous case's series is a test-harness bug that reads as a
# product one.
# Not a subshell: the assertions inside a case count towards this file's tally
# and a failure inside one has to fail the run.
with_probes() {
	local name=$1 files=$2 case=$3
	PROBE_FILES="$files"
	records="${work}/records-${name}"
	rm -rf -- "$records"
	"$case"
	PROBE_FILES=""
	records=""
}

fetch_case_with_no_series() {
result=$(deploy unit --fetch "$records")
assert_status "a fetch with no probe series is a fetch" 0 "$(status_of "$result")"
assert_eq "the state file lands under a timestamped name" 1 \
	"$(find "$records" -name 'selftest-state-*.toml' | wc -l)"
assert_eq "a device that never probed brings back no series" 0 \
	"$(find "$records" -name 'hold-probe-*.csv' | wc -l)"
}
with_probes none "" fetch_case_with_no_series

fetch_case_with_two_series() {
result=$(deploy unit --fetch "$records")
assert_status "a fetch with two series is a fetch" 0 "$(status_of "$result")"
assert_eq "each series lands under the name the bench gave it" 2 \
	"$(find "$records" -name 'hold-probe-*.csv' | wc -l)"
assert_contains "the series is the device's file, not an empty one" \
	"$(cat "${records}/hold-probe-1750000000-17.csv")" \
	"hold-probe-1750000000-17.csv"
assert_contains "each fetched path is printed" "$(output_of "$result")" \
	"hold-probe-1750000001-18.csv"
assert_eq "no part file is left behind" 0 "$(find "$records" -name '*.part' | wc -l)"
assert_eq "no staging directory is left behind" 0 \
	"$(find "$records" -name '.probe-part' | wc -l)"

# One connection carries every series, whatever the count: a session of many
# runs must not cost a handshake each.
assert_eq "the series come back in one stream" 1 \
	"$(grep -c 'tar -cf -' "$CALLS")"

# A second fetch into the same directory lands nothing new: the bytes that
# come back are the bytes already here, so the local copy stands and the fetch
# says so. The stream still runs — the device's clock is what a name rests on,
# and only the content can say whether two names are the same reading.
: >"$CALLS"
result=$(deploy unit --fetch "$records")
assert_status "a second fetch is a fetch" 0 "$(status_of "$result")"
assert_contains "a series already here says so" "$(output_of "$result")" "already here"
assert_eq "and it is still the device's file" "series of hold-probe-1750000000-17.csv" \
	"$(cat "${records}/hold-probe-1750000000-17.csv")"
assert_eq "no staging directory survives it" 0 \
	"$(find "$records" -name '.probe-part' | wc -l)"

# A local copy of the same name holding different readings is a collision the
# operator settles: the device's clock can repeat a stamp across a boot, and
# neither copy is thrown away for the other.
printf 'a different reading\n' >"${records}/hold-probe-1750000000-17.csv"
result=$(deploy unit --fetch "$records")
assert_status "a name reused for different readings refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the file and the cause" "$(output_of "$result")" \
	"holding different readings"
assert_eq "and it leaves the local copy alone" "a different reading" \
	"$(cat "${records}/hold-probe-1750000000-17.csv")"
assert_eq "and no staging directory behind it" 0 \
	"$(find "$records" -name '.probe-part' | wc -l)"
}
with_probes two \
	"/var/lib/brenn-app/hold-probe-1750000000-17.csv /var/lib/brenn-app/hold-probe-1750000001-18.csv" \
	fetch_case_with_two_series

# A stream that fails partway leaves nothing behind under a name a later fetch
# would take for a whole series -- which is what the staging directory is for.
fetch_case_with_a_stream_that_fails() {
SSH_TAR_STATUS=1
result=$(deploy unit --fetch "$records")
SSH_TAR_STATUS=0
assert_status "a stream that failed is a failed fetch" 1 "$(status_of "$result")"
assert_contains "and it says which host" "$(output_of "$result")" \
	"could not fetch the probe series"
assert_eq "no series is left behind" 0 "$(find "$records" -name 'hold-probe-*.csv' | wc -l)"
assert_eq "and no staging directory is" 0 "$(find "$records" -name '.probe-part' | wc -l)"
}
with_probes stream-fails \
	"/var/lib/brenn-app/hold-probe-1750000000-17.csv /var/lib/brenn-app/hold-probe-1750000001-18.csv" \
	fetch_case_with_a_stream_that_fails

# A stream that came back short is the same answer: the series that did arrive
# are not moved into place under a partial fetch.
fetch_case_with_a_short_stream() {
TAR_OMITS="hold-probe-1750000001-18.csv"
result=$(deploy unit --fetch "$records")
TAR_OMITS=""
assert_status "a short stream is a failed fetch" 1 "$(status_of "$result")"
assert_contains "the missing series is named" "$(output_of "$result")" \
	"sent no hold-probe-1750000001-18.csv"
assert_eq "nothing lands from a partial stream" 0 \
	"$(find "$records" -name 'hold-probe-*.csv' | wc -l)"
assert_eq "and no staging directory is left" 0 "$(find "$records" -name '.probe-part' | wc -l)"
}
with_probes short-stream \
	"/var/lib/brenn-app/hold-probe-1750000000-17.csv /var/lib/brenn-app/hold-probe-1750000001-18.csv" \
	fetch_case_with_a_short_stream

# The listing comes out of a directory the unprivileged account owns and this
# fetch runs as root: a name that is not the shape the bench writes never
# reaches a command line.
fetch_case_with_an_unexpected_name() {
result=$(deploy unit --fetch "$records")
assert_status "a name the bench would not have written refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the file" "$(output_of "$result")" "unexpected file"
assert_eq "and nothing was streamed" 0 "$(grep -c 'tar -cf -' "$CALLS")"
}
with_probes odd-name \
	'/var/lib/brenn-app/hold-probe-1;touch\ /tmp/pwned.csv' \
	fetch_case_with_an_unexpected_name

# Only a selftest writes the state file, and the probe's own procedure is runs
# of hold-probe and then a fetch: a missing state file must not strand the
# series the session took.
fetch_case_with_no_state_file() {
SSH_CAT_STATUS=1
result=$(deploy unit --fetch "$records")
SSH_CAT_STATUS=0
assert_status "a fetch with no state file is still a fetch" 0 "$(status_of "$result")"
assert_contains "and it says the state file is not there" "$(output_of "$result")" \
	"no state file"
assert_eq "the series still come back" 2 "$(find "$records" -name 'hold-probe-*.csv' | wc -l)"
assert_eq "and no empty record is left behind" 0 \
	"$(find "$records" -name 'selftest-state-*.toml' | wc -l)"
}
with_probes no-state \
	"/var/lib/brenn-app/hold-probe-1750000000-17.csv /var/lib/brenn-app/hold-probe-1750000001-18.csv" \
	fetch_case_with_no_state_file

# Neither half there is a fetch that found nothing, which is a failure.
fetch_case_with_nothing_at_all() {
SSH_CAT_STATUS=1
result=$(deploy unit --fetch "$records")
SSH_CAT_STATUS=0
assert_status "a device holding neither refuses" 1 "$(status_of "$result")"
assert_contains "and says so" "$(output_of "$result")" "nothing to fetch"
}
with_probes nothing "" fetch_case_with_nothing_at_all

# ---------------------------------------------------------------------------
# The name the bench writes a series under is the name this script globs for
# ---------------------------------------------------------------------------

# Nothing joins a Rust format string to a shell glob but the text itself. The
# binary asserts it writes this prefix; this asserts the script looks for the
# same one, so a rename on either side fails here rather than fetching nothing.
declared=$(sed -n 's/^pub const HOLD_PROBE_SERIES_PREFIX: &str = "\(.*\)";$/\1/p' \
	"${script_dir}/../crates/reachy-bench/src/bare.rs")
globbed=$(sed -n 's/^probe_prefix="\(.*\)"$/\1/p' "$subject")
assert_eq "the bench states a series prefix" "hold-probe-" "$declared"
assert_eq "the fetch globs for the prefix the bench writes" "$declared" "$globbed"

# ---------------------------------------------------------------------------

tally
