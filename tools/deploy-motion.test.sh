#!/usr/bin/env bash
#
# tools/deploy-motion.test.sh — self-check for deploy-motion.sh.
#
# The script under test reaches a device with ssh and rsync and reads the
# repository with git. All three are stubbed here, on PATH, recording what they
# were asked for; nothing in this file touches a network, a device, or this
# checkout. The subject is copied into a temporary tree beside its own lib.sh, so
# `repo_root` is that tree and the payload it pushes is a directory this test
# made.
#
# What is worth pinning: the refusals, and the run. The refusals are a stale
# payload, a held servo bus, a fetch that brought no records, and every exit
# status of a run that is not the budgeted stop — a refusal that quietly stops
# refusing is discovered on the night it should have saved. The run is what moves
# a machine, so its remote command is pinned to the letter: the bus question and
# the launcher in one invocation, started from the release root with the config
# the payload carries, under the budget that stops it.
#
# The only value read out of the staged configuration is the log root. The
# pinion namespace and shm root are enforced at build time: nothing this script
# prints carries a pinion flag, because the launcher config cannot, and the
# build refuses a payload whose logger configuration does not restate the
# compiled-in defaults.
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
payload="${repo}/target/motion-arm64/release"
mkdir -p -- "${repo}/tools" "${repo}/cogs" "${payload}/cogs" "${payload}/driver"
cp -- "${script_dir}/deploy-motion.sh" "${script_dir}/lib.sh" "${repo}/tools/"

subject="${repo}/tools/deploy-motion.sh"

# The logger configuration as the build stages it, in the syntax the real one is
# written in: one field per line, strings quoted, and one of them indented — the
# subject reads a scalar wherever the line puts it, so the file's formatting is
# not load-bearing. The log root here is deliberately not the shipped one: a
# subject that hardcoded the real one would pass a test written against it.
#
# It sits in the payload, not in the tree. What the device will run is what was
# staged, and a value printed from a tree edited since the last build describes
# nothing that will run.
logger_config="${payload}/cogs/robot_logger.textproto"

# Written by stage_payload, because the build writes it: a case that removes the
# payload removes this too, and the next stage puts it back.
stage_logger_config() {
	cat >"$logger_config" <<'CONFIG'
# a comment, and a field name inside it: log_root_dir: "/decoy"
log_root_dir: "/run/brenn-app/logs/testing"
  pinion_shm_root: "/dev/shm"
pinion_namespace: ""
max_write_mib_per_sec: 4
CONFIG
}

# The checked-in copy, with every value wrong. A subject that read the tree
# instead of the payload would print these, which is the drift this pins: the
# tree is edited between builds and the device runs what was staged.
cat >"${repo}/cogs/robot_logger.textproto" <<'CONFIG'
log_root_dir: "/run/brenn-app/logs/from-the-tree"
pinion_shm_root: "/dev/shm-from-the-tree"
pinion_namespace: "fromthetree"
CONFIG

# The payload as the build stages it: three executables at the paths the launcher
# config spells, the launcher config itself, and the configuration the modes read.
# What the build recorded in the payload about the commit it was staged from.
# Empty stages no such file at all, which is a payload from a build script that
# wrote none — the default here, because that is the case the push has to fall
# back on and every stamp assertion below the fallback reads.
BUILD_COMMIT=""

# The other half of the same record: which brenn-pod the payload's voice host
# was linked from. Empty stages the line's absence, which is a payload from a
# build script that wrote no such field, and the fallback below reads it.
BUILD_POD=""

stage_payload() {
	local when=$1
	local name
	for name in reachy_motord reachy_host reachy_pod reachy_ask libonnxruntime.so.1 \
		cogs/robot_clk_exe simplelaunch; do
		: >"${payload}/${name}"
		chmod 0755 -- "${payload}/${name}"
	done
	: >"${payload}/robotcpu.textproto"
	: >"${payload}/robotcpu_harness.textproto"
	: >"${payload}/cogs/session_params.textproto"
	for name in models/oww/melspectrogram.onnx models/oww/embedding_model.onnx \
		models/oww/hey_jarvis_v0.1.onnx models/silero/silero_vad.onnx; do
		mkdir -p -- "$(dirname -- "${payload}/${name}")"
		: >"${payload}/${name}"
	done
	stage_logger_config
	rm -f -- "${payload}/build-commit.txt"
	if [ -n "$BUILD_COMMIT" ]; then
		echo "commit=${BUILD_COMMIT}" >"${payload}/build-commit.txt"
		[ -z "$BUILD_POD" ] ||
			echo "brenn_pod=${BUILD_POD}" >>"${payload}/build-commit.txt"
	fi
	touch -d "@${when}" -- "${payload}/cogs/robot_clk_exe"
}

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

export CALLS="${work}/calls"
export GIT_COMMIT_TIME=""
export GIT_HEAD=0123456789abcdef0123456789abcdef01234567
export GIT_DIRTY=""
export GIT_STATUS_FAILS=no
# Where the rsync stub keeps whatever provenance stamp a push handed it, so a
# case can read what the file says rather than only that the payload was sent.
export PUSHED_PROVENANCE="${work}/pushed-provenance"
export SSH_PREPARE_STATUS=0
export SSH_PROBE_STATUS=0
export SSH_RUN_STATUS=124
export SSH_RUN_REACHED=no
export RSYNC_STATUS=0
export RSYNC_OLOG=full
export RSYNC_CONSOLE=full
export RSYNC_AUDIO=full
export TEE_STATUS=0

# Every stub records its whole invocation on one line, so a case can assert both
# that a command ran and that it did not.
cat >"${stubs}/ssh" <<'STUB'
#!/usr/bin/env bash
printf 'ssh %s\n' "$*" >>"$CALLS"
# A run's ssh is the one carrying a pty, and its status is the launcher's rather
# than the probe's, so the two are separate knobs: a case can hold the bus for a
# push and answer 124 for a run.
case " $* " in
	# The read-only captures are their own knob: what a probe does when the
	# unit will not answer it is a property of the run, not of the push that
	# preceded it.
	*timedatectl*|*/proc/config.gz*) exit "${SSH_PROBE_STATUS:-0}" ;;
	*" -t "*)
		# Whether the chain got past its last refusal, which on a real
		# unit is the sentinel line the chain echoes and here is a knob
		# of its own: a status is a number both a chain step and a
		# launcher can exit with, and this is what tells them apart.
		[ "${SSH_RUN_REACHED:-no}" = yes ] && echo ---brenn-launcher-starting
		exit "${SSH_RUN_STATUS:-124}"
		;;
esac
exit "${SSH_PREPARE_STATUS:-0}"
STUB

# `tee` is stubbed so that a case can make the console copy fail: the records
# still only exist on the unit's tmpfs when it runs, so a failure here must not
# be able to end the script. The passing path writes the file the way `tee` does,
# because later cases look for it.
cat >"${stubs}/tee" <<'STUB'
#!/usr/bin/env bash
status=${TEE_STATUS:-0}
if [ "$status" != 0 ]; then
	cat >/dev/null
	exit "$status"
fi
cat >"${*: -1}"
STUB

# rsync also has to *bring something back*, because the fetch decides on what
# arrived: a run's records are a `.olog` with bytes in it, and a fetch that
# brought none is refused. RSYNC_OLOG says what this invocation delivers into the
# destination — a whole run, an empty file, or nothing at all.
cat >"${stubs}/rsync" <<'STUB'
#!/usr/bin/env bash
printf 'rsync %s\n' "$*" >>"$CALLS"
status=${RSYNC_STATUS:-0}
# A push's last argument is the remote destination and a fetch's is a local
# directory, so this only delivers into the latter: writing into `root@host:/...`
# as a relative path is how a stub makes a directory in the checkout.
if [ "$status" = 0 ]; then
	dest=${*: -1}
	# The provenance stamp rides in the payload, so what the push delivered
	# is read out of the source directory -- the argument before the
	# destination -- and kept where a case can read it.
	case "$dest" in
	*:*/releases/*)
		if [ -e "${*: -2:1}provenance.txt" ]; then
			cp -- "${*: -2:1}provenance.txt" "$PUSHED_PROVENANCE"
		fi
		;;
	esac
	case "$dest" in *@*:*) exit 0 ;; esac
	# The console copy is its own delivery: it lands beside the records
	# under a name ending in .console, and what it brings is the launcher's
	# files rather than a run directory. RSYNC_CONSOLE says whether the
	# driver's own is among them.
	# The recorded audio is its own delivery too, and its own knob: what a
	# speech run brings home depends on whether the site's configuration
	# turned recording on, and an empty store is a line rather than a
	# refusal. RSYNC_AUDIO says which of the three a case is asking for.
	case "$dest" in
	*.audio/)
		case "${RSYNC_AUDIO:-full}" in
			full)
				echo 'frames' >"${dest}20260903T225630_657Z_1.framelog"
				echo 'frames' >"${dest}20260903T225641_529Z_2.framelog"
				;;
			none) ;;
			refused) exit 23 ;;
		esac
		exit 0
		;;
	esac
	case "$dest" in
	*.console/)
		case "${RSYNC_CONSOLE:-full}" in
			full)
				echo 'cycles=1500 session_cmds=223 taken=223 aux_refused=2' \
					>"${dest}motord_0.log"
				echo 'the control process said things' >"${dest}proc_0.log"
				;;
			nodriver) echo 'no driver here' >"${dest}proc_0.log" ;;
			none) ;;
		esac
		exit 0
		;;
	esac
	case "${RSYNC_OLOG:-full}" in
		full)
			mkdir -p -- "${dest}/run-20260825T120000Z"
			echo records >"${dest}/run-20260825T120000Z/motion_0.olog"
			# What the run copied into the log root: the stamp sits
			# at the root beside the writer's run directory, so a
			# fetch carries it home with no fetch-side logic.
			echo "commit=${GIT_HEAD:-}" >"${dest}/provenance.txt"
			;;
		empty)
			mkdir -p -- "${dest}/run-20260825T120000Z"
			: >"${dest}/run-20260825T120000Z/motion_0.olog"
			;;
		none) ;;
	esac
fi
exit "$status"
STUB

# The three questions the subject asks the repository, on three knobs. An empty
# GIT_COMMIT_TIME is a tree whose history says nothing about the workspace paths;
# an empty GIT_HEAD is one that cannot state its commit at all, which is a push
# refusal rather than an unstamped push; GIT_DIRTY is the porcelain status, whose
# emptiness is the whole of the dirty question.
cat >"${stubs}/git" <<'STUB'
#!/usr/bin/env bash
printf 'git %s\n' "$*" >>"$CALLS"
case " $* " in
	*" rev-parse "*)
		[ -n "${GIT_HEAD:-}" ] || exit 128
		echo "$GIT_HEAD"
		exit 0
		;;
	*" status "*)
		[ "${GIT_STATUS_FAILS:-no}" = no ] || exit 128
		[ -z "${GIT_DIRTY:-}" ] || echo "$GIT_DIRTY"
		exit 0
		;;
esac
[ -n "${GIT_COMMIT_TIME:-}" ] || exit 0
echo "$GIT_COMMIT_TIME"
STUB

# The analyzer, whose verdict a run's verdict is. It reports its argument so a
# case can pin which run directory was judged, and its status is the knob that
# says whether the log passed.
#
# It answers two other questions for the speech run: the build of the
# configuration checker, on its own knob because a checker that will not build
# is a different refusal from a configuration it rejects, and the cquery that
# names where the built checker is. The path it names is a stub in the tree
# under test, which is what the subject then runs from the payload root.
cat >"${stubs}/bazel" <<'STUB'
#!/usr/bin/env bash
printf 'bazel %s\n' "$*" >>"$CALLS"
case " $* " in
	*" cquery "*)
		echo bazel-out/reachy_host
		exit 0
		;;
	*" build "*) exit "${BAZEL_BUILD_STATUS:-0}" ;;
esac
exit "${BAZEL_STATUS:-0}"
STUB

chmod 0755 -- "${stubs}/ssh" "${stubs}/rsync" "${stubs}/git" "${stubs}/bazel" "${stubs}/tee"
export BAZEL_STATUS=0

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
stage_payload "$after"
result=$(deploy unit --push)
assert_status "a payload newer than the newest commit pushes" 0 "$(status_of "$result")"
assert_contains "the push reaches the device" "$(calls)" \
	"rsync -a --delete -e ssh -o BatchMode=yes ${payload}/ root@unit:/run/brenn-app/releases/motion/"
assert_contains "the freshness question is asked of the workspace paths" "$(calls)" \
	"git -C ${repo} log -1 --format=%ct -- crates cogs driver motion hardware geometry clips bazel MODULE.bazel MODULE.bazel.lock .bazelrc .bazelversion tools/build-motion.sh tools/lib.sh"

stage_payload "$before"
result=$(deploy unit --push)
assert_status "a payload older than the newest commit refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says what is wrong" "$(output_of "$result")" \
	"older than the newest commit"
assert_contains "the refusal names the build" "$(output_of "$result")" "make motion-build"
assert_contains "the refusal names the override" "$(output_of "$result")" "--stale-ok"
assert_lacks "a refused push pushes nothing" "$(calls)" "rsync"
assert_lacks "a refused push reaches no device" "$(calls)" "ssh"

result=$(deploy unit --push --stale-ok)
assert_status "--stale-ok pushes the old payload" 0 "$(status_of "$result")"
assert_contains "--stale-ok says the age went unchecked" "$(output_of "$result")" \
	"the payload's age is not being checked"
assert_contains "--stale-ok still pushes" "$(calls)" "rsync"

# The two payload members no commit to this workspace can date: the prebuilt
# audio device binary and the site's speech configuration. The commit-time check
# above cannot see either, so a rebuilt binary or a rotated token would otherwise
# ship as new under a green verdict.
stage_payload "$after"
# The staged copies carry the moment they were installed, so a case dates both
# sides: the payload's copy at the build's instant, the source relative to it.
touch -d "@${after}" -- "${payload}/reachy_pod"
pod_source="${repo}/../brenn-pod/firmware/target/reachy-pod/payload/reachy-pod"
mkdir -p -- "$(dirname -- "$pod_source")"
: >"$pod_source"
touch -d "@${after}" -- "$pod_source"
result=$(deploy unit --push)
assert_status "an audio device binary no newer than the staged copy pushes" 0 \
	"$(status_of "$result")"

touch -d "@$((after + 3600))" -- "$pod_source"
result=$(deploy unit --push)
assert_status "a rebuilt audio device binary refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the member" "$(output_of "$result")" \
	"audio device binary"
assert_contains "and says which copy would be shipped" "$(output_of "$result")" \
	"newer than the copy in the payload"
assert_contains "and names the rebuild" "$(output_of "$result")" "make motion-build"
assert_lacks "and nothing is pushed" "$(calls)" "rsync"

result=$(deploy unit --push --stale-ok)
assert_status "--stale-ok covers the out-of-tree members too" 0 \
	"$(status_of "$result")"
rm -f -- "$pod_source"

# The speech configuration, whose staged copy is optional: a source that exists
# against a payload carrying none is the same mistake, because the file existed
# nowhere when the payload was staged.
speech_source="${repo}/host/speech.toml"
mkdir -p -- "$(dirname -- "$speech_source")"
printf 'listen_addr = "127.0.0.1:7380"\n' >"$speech_source"
touch -d "@${after}" -- "$speech_source"
result=$(deploy unit --push)
assert_status "a speech configuration the payload never staged refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal says the payload carries none" "$(output_of "$result")" \
	"the staged payload carries none"
assert_lacks "and that one pushes nothing either" "$(calls)" "rsync"

mkdir -p -- "${payload}/host"
: >"${payload}/host/speech.toml"
touch -d "@$((after + 60))" -- "${payload}/host/speech.toml"
result=$(deploy unit --push)
assert_status "a staged copy newer than the source pushes" 0 "$(status_of "$result")"

touch -d "@$((after + 3600))" -- "$speech_source"
result=$(deploy unit --push)
assert_status "a speech configuration edited since the build refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names that member" "$(output_of "$result")" \
	"speech configuration"
assert_lacks "and pushes nothing" "$(calls)" "rsync"
rm -f -- "$speech_source" "${payload}/host/speech.toml"

# The unit's own parameters. The build stages the operator's file from outside
# the tree, and the value in it decides which pod this head answers to: pushed
# from a payload staged before the edit, the head answers to the previous name
# and every script addressed to it is dropped as a foreign pod's.
host_params_source="${repo}/.local/host_params.textproto"
mkdir -p -- "$(dirname -- "$host_params_source")" "${payload}/host"
printf 'pod: "unit-reachy"\n' >"$host_params_source"
touch -d "@${after}" -- "$host_params_source"
: >"${payload}/host/host_params.textproto"
touch -d "@$((after + 60))" -- "${payload}/host/host_params.textproto"
result=$(deploy unit --push)
assert_status "a staged copy newer than the operator's parameters pushes" 0 \
	"$(status_of "$result")"

touch -d "@$((after + 3600))" -- "$host_params_source"
result=$(deploy unit --push)
assert_status "host parameters edited since the build refuse" 1 "$(status_of "$result")"
assert_contains "the refusal names that member" "$(output_of "$result")" \
	"host configuration"
assert_contains "and names the file it would ship the old copy of" \
	"$(output_of "$result")" "$host_params_source"
assert_lacks "and pushes nothing" "$(calls)" "rsync"

result=$(deploy unit --push --stale-ok)
assert_status "--stale-ok covers the operator's parameters too" 0 \
	"$(status_of "$result")"
rm -f -- "$host_params_source" "${payload}/host/host_params.textproto"

# The credential files that configuration names — the pod's key table, the bus
# token. They are the flavour of this an operator would otherwise chase on the
# unit: a re-provisioned key table or a freshly issued token is a file no commit
# to this workspace dates, and pushed against an old payload it is the previous
# secret shipped under a green verdict. The paths come out of the configuration,
# so the push and the build cannot disagree about where they are.
mkdir -p -- "${repo}/host/secrets" "${payload}/secrets"
cat >"$speech_source" <<'TOML'
listen_addr = "127.0.0.1:7380"
pod_psk_file = "secrets/pod-psk.toml"

[brenn.bridge]
token_file = "secrets/remote.token"
TOML
: >"${repo}/host/secrets/pod-psk.toml"
: >"${repo}/host/secrets/remote.token"
: >"${payload}/host/speech.toml"
: >"${payload}/secrets/pod-psk.toml"
: >"${payload}/secrets/remote.token"
touch -d "@${after}" -- "$speech_source" "${repo}/host/secrets/pod-psk.toml" \
	"${repo}/host/secrets/remote.token"
touch -d "@$((after + 60))" -- "${payload}/host/speech.toml" \
	"${payload}/secrets/pod-psk.toml" "${payload}/secrets/remote.token"
result=$(deploy unit --push)
assert_status "credentials no newer than their staged copies push" 0 \
	"$(status_of "$result")"

touch -d "@$((after + 3600))" -- "${repo}/host/secrets/pod-psk.toml"
result=$(deploy unit --push)
assert_status "a re-provisioned key table refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the credential by its payload path" \
	"$(output_of "$result")" "speech credential secrets/pod-psk.toml"
assert_contains "and says which copy would be shipped" "$(output_of "$result")" \
	"newer than the copy in the payload"
assert_lacks "and pushes nothing" "$(calls)" "rsync"

result=$(deploy unit --push --stale-ok)
assert_status "--stale-ok covers the credentials too" 0 "$(status_of "$result")"
touch -d "@$((after + 60))" -- "${repo}/host/secrets/pod-psk.toml"

# A credential the configuration names against a payload that carries none: the
# same mistake as a speech configuration the payload never staged, because the
# file existed nowhere when the payload was built.
rm -f -- "${payload}/secrets/remote.token"
result=$(deploy unit --push)
assert_status "a token the payload never staged refuses" 1 "$(status_of "$result")"
assert_contains "and the refusal names it" "$(output_of "$result")" \
	"speech credential secrets/remote.token"
assert_contains "and says the payload carries none" "$(output_of "$result")" \
	"the staged payload carries none"
: >"${payload}/secrets/remote.token"
touch -d "@$((after + 60))" -- "${payload}/secrets/remote.token"

# A configuration this reader cannot read is a refused push, not a push that
# skipped a credential it could not name.
printf 'pod_psk_file = "secrets/pod-psk.toml\n' >"$speech_source"
touch -d "@${after}" -- "$speech_source"
result=$(deploy unit --push)
assert_status "a speech configuration whose quoting does not close refuses" 1 \
	"$(status_of "$result")"
assert_contains "and says what it could not read" "$(output_of "$result")" \
	"the value's quoting does not close"
assert_lacks "and pushes nothing" "$(calls)" "rsync"

rm -rf -- "$speech_source" "${repo}/host/secrets" "${payload}/host/speech.toml" \
	"${payload}/secrets"
stage_payload "$after"

# A tree that cannot answer the question is not a stale tree.
GIT_COMMIT_TIME=""
result=$(deploy unit --push)
assert_status "no history means no verdict, and the push proceeds" 0 "$(status_of "$result")"
assert_contains "an undecidable age is said out loud" "$(output_of "$result")" \
	"the device payload's age is unknown"
GIT_COMMIT_TIME=$commit_at

# A payload half built is not a payload.
stage_payload "$after"
rm -f -- "${payload}/reachy_motord"
result=$(deploy unit --push)
assert_status "a payload missing the driver refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the missing binary" "$(output_of "$result")" \
	"no executable reachy_motord"
assert_lacks "a payload missing a binary is not reported as a stale one" \
	"$(output_of "$result")" "older than the newest commit"
stage_payload "$after"

# The launcher is a payload binary like the other two: without it there is
# nothing to start the run with, and the check is at the path the launcher config
# spells rather than wherever the file happens to be.
rm -f -- "${payload}/simplelaunch"
result=$(deploy unit --push)
assert_status "a payload missing the launcher refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names it" "$(output_of "$result")" "no executable simplelaunch"
stage_payload "$after"

# The voice host is a payload binary like the driver: the production launcher
# config names it, so a payload without it is a unit whose launcher starts an app
# that is not there.
rm -f -- "${payload}/reachy_host"
result=$(deploy unit --push)
assert_status "a payload missing the voice host refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the host binary" "$(output_of "$result")" \
	"no executable reachy_host"
stage_payload "$after"

# The audio device is a payload binary the build did not compile: it is staged
# from a prebuilt brenn-pod artifact. What the push checks is the same thing it
# checks for the rest -- the production launcher config names a `reachy_pod` at
# the payload root, and a unit whose payload has none is a launcher starting an
# app that is not there.
rm -f -- "${payload}/reachy_pod"
result=$(deploy unit --push)
assert_status "a payload missing the audio device refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the pod binary" "$(output_of "$result")" \
	"no executable reachy_pod"
stage_payload "$after"

# The shared object is a payload member for a reason none of the others share:
# it is nobody's launcher app, and what needs it is the loader, at the instant
# the host is exec'd. A payload without it is a host that dies before it can
# narrate anything.
rm -f -- "${payload}/libonnxruntime.so.1"
result=$(deploy unit --push)
assert_status "a payload missing the shared object refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the shared object" "$(output_of "$result")" \
	"no shared object libonnxruntime.so.1"
stage_payload "$after"

# The weights are the one part of the payload the build fetches over a network,
# so they are the one part a proxy or an outage can leave out of an otherwise
# complete staging. Refused here, because the symptom on a unit is a wake gate
# that fails at its first inference rather than a process that never starts.
rm -f -- "${payload}/models/oww/hey_jarvis_v0.1.onnx"
result=$(deploy unit --push)
assert_status "a payload missing the wake phrase's model refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the missing model" "$(output_of "$result")" \
	"no model models/oww/hey_jarvis_v0.1.onnx"
stage_payload "$after"

rm -f -- "${payload}/models/silero/silero_vad.onnx"
result=$(deploy unit --push)
assert_status "a payload missing the endpointer's model refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names that one too" "$(output_of "$result")" \
	"no model models/silero/silero_vad.onnx"
stage_payload "$after"

# The two launcher configs are payload members like the binaries: `--run` names
# the harness twin by name and the launcher resolves it against the payload root,
# so a payload missing either one is refused here rather than on a powered unit.
rm -f -- "${payload}/robotcpu_harness.textproto"
result=$(deploy unit --push)
assert_status "a payload missing the harness launcher config refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the missing config" "$(output_of "$result")" \
	"no launcher config robotcpu_harness.textproto"
stage_payload "$after"

rm -f -- "${payload}/robotcpu.textproto"
result=$(deploy unit --push)
assert_status "a payload missing the production launcher config refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names that one too" "$(output_of "$result")" \
	"no launcher config robotcpu.textproto"
stage_payload "$after"

rm -f -- "${payload}/cogs/robot_clk_exe"
result=$(deploy unit --push)
assert_status "a payload with the executable outside cogs/ refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the path the launcher config spells" \
	"$(output_of "$result")" "no executable cogs/robot_clk_exe"
stage_payload "$after"

rm -rf -- "$payload"
result=$(deploy unit --push)
assert_status "no payload at all refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the build" "$(output_of "$result")" "make motion-build"
mkdir -p -- "${payload}/cogs" "${payload}/driver"
stage_payload "$after"

# ---------------------------------------------------------------------------
# The bus, and what the push prepares on the unit
# ---------------------------------------------------------------------------

result=$(deploy unit --push)
assert_contains "the push makes the release directory" "$(calls)" \
	"mkdir -p -- /run/brenn-app/releases/motion"
assert_contains "the push makes the log root the configuration names" "$(calls)" \
	"/run/brenn-app/logs/testing"
assert_lacks "the log root is read, not retyped" "$(calls)" "/decoy"
assert_contains "the bus question is asked before the push" "$(calls)" \
	"systemctl is-active --quiet reachy-motiond.service"

SSH_PREPARE_STATUS=3
result=$(deploy unit --push)
assert_status "the payload holding the bus refuses" 1 "$(status_of "$result")"
assert_contains "it names the service holding it" "$(output_of "$result")" \
	"brenn-app.service is running"
assert_lacks "a refused push pushes nothing" "$(calls)" "rsync"

SSH_PREPARE_STATUS=4
result=$(deploy unit --push)
assert_status "the motion daemon holding the bus refuses" 1 "$(status_of "$result")"
assert_contains "the way back is named" "$(output_of "$result")" \
	"systemctl start reachy-motiond.service"

SSH_PREPARE_STATUS=255
result=$(deploy unit --push)
assert_status "ssh failing refuses" 1 "$(status_of "$result")"
assert_contains "ssh failing says nothing was pushed" "$(output_of "$result")" \
	"nothing was pushed"

SSH_PREPARE_STATUS=1
result=$(deploy unit --push)
assert_status "a unit that could not be prepared refuses" 1 "$(status_of "$result")"
assert_contains "it says the push did not happen" "$(output_of "$result")" \
	"nothing was pushed"
SSH_PREPARE_STATUS=0

# ---------------------------------------------------------------------------
# Which build a run's records came off
# ---------------------------------------------------------------------------
#
# The log reader binds each channel's schema byte for byte, so a run's records
# are readable only by the build that recorded them. The push-time facts are
# weaker than they look: the freshness refusal does not catch uncommitted edits
# and --stale-ok skips it entirely, so the stamp describes the artefact honestly
# rather than assuming a clean tree.

rm -f -- "$PUSHED_PROVENANCE"
result=$(deploy unit --push)
assert_status "a push that can state its commit succeeds" 0 "$(status_of "$result")"
assert_file "the stamp is in the payload the one rsync delivered" "$PUSHED_PROVENANCE"
assert_eq "and it took no transfer of its own" 1 \
	"$(calls | grep -c '^rsync ')"
stamp=$(cat -- "$PUSHED_PROVENANCE")
assert_contains "a payload that recorded no build commit is stamped with the tree's" \
	"$stamp" "commit=${GIT_HEAD}"
assert_contains "and the stamp says that is where the commit came from" "$stamp" \
	"commit_source=push"
assert_contains "with the pushing tree's own HEAD beside it either way" "$stamp" \
	"pushed_from=${GIT_HEAD}"
assert_contains "a clean tree is stamped clean" "$stamp" "dirty=no"
assert_contains "and a checked age says so" "$stamp" "age_unchecked=no"
assert_contains "and the file says how to read a log with that build" "$stamp" \
	"git switch --detach ${GIT_HEAD}"
assert_contains "the push says out loud what it stamped" "$(output_of "$result")" \
	"commit ${GIT_HEAD} (push), pushed from ${GIT_HEAD}, dirty=no"

# A tree with uncommitted edits is exactly what the freshness refusal cannot see,
# so the stamp is the only thing that can say it.
GIT_DIRTY=" M crates/reachy-motord/src/tick.rs"
result=$(deploy unit --push)
assert_status "a dirty tree still pushes" 0 "$(status_of "$result")"
assert_contains "and is stamped dirty" "$(cat -- "$PUSHED_PROVENANCE")" "dirty=yes"
assert_contains "and says so out loud" "$(output_of "$result")" "dirty=yes"
GIT_DIRTY=""

# A question the repository refused to answer is not a clean tree.
GIT_STATUS_FAILS=yes
result=$(deploy unit --push)
assert_status "a repository that will not answer still pushes" 0 "$(status_of "$result")"
assert_contains "and the stamp claims nothing it does not know" \
	"$(cat -- "$PUSHED_PROVENANCE")" "dirty=unknown"
GIT_STATUS_FAILS=no

result=$(deploy unit --push --stale-ok)
assert_status "--stale-ok pushes" 0 "$(status_of "$result")"
assert_contains "and the stamp records that the age went unchecked" \
	"$(cat -- "$PUSHED_PROVENANCE")" "age_unchecked=yes"

# The commit the binaries came out of is the build's fact, not the push's. A
# payload built at one commit and pushed from a checkout at another passes the
# freshness refusal — it only turns away a payload that is too old — so a stamp
# reading the pushing tree's HEAD would name a commit that never produced these
# binaries, and an operator switching to it later would find a schema that does
# not bind. The build records what it built from and the stamp prefers it.
BUILD_COMMIT=fedcba9876543210fedcba9876543210fedcba98
stage_payload "$after"
result=$(deploy unit --push)
assert_status "a push over a payload that names its build succeeds" 0 \
	"$(status_of "$result")"
stamp=$(cat -- "$PUSHED_PROVENANCE")
assert_contains "the stamp names the commit the payload was built from" "$stamp" \
	"commit=${BUILD_COMMIT}"
assert_contains "and says the payload is where that came from" "$stamp" \
	"commit_source=build"
assert_contains "and the tree it was pushed from is beside it, not averaged in" \
	"$stamp" "pushed_from=${GIT_HEAD}"
assert_contains "so reading the log names the build, not the push" "$stamp" \
	"git switch --detach ${BUILD_COMMIT}"
assert_contains "and the push says both out loud" "$(output_of "$result")" \
	"commit ${BUILD_COMMIT} (build), pushed from ${GIT_HEAD}"
assert_contains "a payload that recorded no brenn-pod is stamped unknown" "$stamp" \
	"brenn_pod=unknown"

# The voice host is linked from brenn-pod, and this tree's commit says nothing
# about which revision that was: a run's records are the only place that fact
# survives, and the pushing tree's MODULE.bazel is where it stands now rather
# than where it stood at the build. So the build records it and the push copies
# it through, whichever of the two forms it took.
BUILD_POD=97cd7889207f877538d001e56e930c68ff2ca699
stage_payload "$after"
result=$(deploy unit --push)
assert_status "a push over a payload that names its brenn-pod succeeds" 0 \
	"$(status_of "$result")"
assert_contains "and the stamp carries the revision the voice host came from" \
	"$(cat -- "$PUSHED_PROVENANCE")" "brenn_pod=${BUILD_POD}"

BUILD_POD="overlay:../brenn-pod/host/crates/speech-surface"
stage_payload "$after"
result=$(deploy unit --push)
assert_contains "a payload built over a working tree says so, naming no revision" \
	"$(cat -- "$PUSHED_PROVENANCE")" "brenn_pod=${BUILD_POD}"
BUILD_POD=""

# A build in a tree with no history records `unknown` rather than a guess, and
# the push falls back the way it does for a payload that recorded nothing.
BUILD_COMMIT=unknown
stage_payload "$after"
result=$(deploy unit --push)
assert_status "a payload whose build could not name a commit still pushes" 0 \
	"$(status_of "$result")"
assert_contains "and the stamp falls back to the pushing tree" \
	"$(cat -- "$PUSHED_PROVENANCE")" "commit=${GIT_HEAD}"
assert_contains "saying which of the two answered" \
	"$(cat -- "$PUSHED_PROVENANCE")" "commit_source=push"
BUILD_COMMIT=""
stage_payload "$after"

# A push that cannot state its own commit is a refusal, not a stamp saying
# nothing: an unnamed build is a records directory nobody can decode later.
GIT_HEAD=""
rm -f -- "$PUSHED_PROVENANCE"
result=$(deploy unit --push)
assert_status "a tree that cannot state its commit refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says why that matters" "$(output_of "$result")" \
	"could not say which build ran"
assert_lacks "and nothing is pushed" "$(calls)" "rsync"
assert_lacks "and the device is not touched" "$(calls)" "ssh"
GIT_HEAD=0123456789abcdef0123456789abcdef01234567

# The stamp travels inside the payload, so a payload that landed without one is a
# payload that did not land: the transfer's own failure is the whole story, and
# there is no second one that can go missing on its own.
RSYNC_STATUS=23
result=$(deploy unit --push)
assert_status "a payload that did not land fails" 23 "$(status_of "$result")"
RSYNC_STATUS=0

# ---------------------------------------------------------------------------
# The run
# ---------------------------------------------------------------------------

run_dest="${work}/run-records"
result=$(deploy unit --run "$run_dest")
ran=$(calls)
assert_status "a budgeted run that the analyzer passes succeeds" 0 "$(status_of "$result")"
assert_contains "the bus question, the log root's clear and the launcher are one invocation" "$ran" \
	"systemctl is-active --quiet brenn-app.service && exit 3; systemctl is-active --quiet reachy-motiond.service && exit 4; [ -f /run/brenn-app/releases/motion/robotcpu_harness.textproto ] || exit 8; [ -f /run/brenn-app/releases/motion/provenance.txt ] || exit 5; cp -- /run/brenn-app/releases/motion/provenance.txt /run/brenn-app/motion-provenance.staged || exit 6; rm -rf -- /run/brenn-app/logs/testing && mkdir -p -- /run/brenn-app/logs/testing || exit 7; mv -- /run/brenn-app/motion-provenance.staged /run/brenn-app/logs/testing/provenance.txt || exit 7; rm -rf -- /run/brenn-app/logs/launch && mkdir -p -- /run/brenn-app/logs/launch || exit 7; cd /run/brenn-app/releases/motion || exit 7; echo ---brenn-launcher-starting; ./reachy_ask --resting-timeout 30 --run-window 30 >/run/brenn-app/logs/launch/reachy_ask.log 2>&1 & ask=\$!; timeout --signal=INT --kill-after=10 30 ./simplelaunch robotcpu_harness.textproto --logdir /run/brenn-app/logs/launch; rc=\$?; kill -INT \$ask 2>/dev/null; wait \$ask 2>/dev/null; exit \$rc"
assert_contains "the run gets a pty, so the console streams and a ^C reaches it" "$ran" \
	"ssh -t -o BatchMode=yes root@unit"
# This suite's stdin is not a terminal, which is the case ssh downgrades
# silently: the pty is what carries a ^C to the unit, so its absence is said.
assert_contains "a run with no terminal to ask a pty for says the ^C is gone" \
	"$(output_of "$result")" "stdin is not a terminal"
assert_contains "and says what stops the run instead" "$(output_of "$result")" \
	"pkill -x simplelaunch"
assert_lacks "no process is started by hand" "$ran" "./robot_clk_exe"
assert_lacks "not even the driver" "$ran" "./reachy_motord"
# The twin, not the production config: `--run` already started `reachy_ask` on
# 7409 and 7410, and the host the production config names owns the same two.
assert_lacks "a run does not start the config that names the voice host" "$ran" \
	"./simplelaunch robotcpu.textproto"
assert_lacks "and the host is not started beside it either" "$ran" "./reachy_host"
assert_lacks "nothing carries a pinion flag" "$ran" "--pinion-"

assert_contains "the records are fetched from the configured log root" "$ran" \
	"root@unit:/run/brenn-app/logs/testing/"
assert_lacks "the log root is read, not retyped" "$ran" "/decoy"
assert_lacks "and read out of the payload, not out of the tree" "$ran" "fromthetree"
assert_contains "the analyzer judges the run directory that was discovered" "$ran" \
	"bazel run -- //cogs:first_motion_report ${run_dest}/motion-log-"
assert_contains "and the directory it names is the writer's own" "$ran" \
	"run-20260825T120000Z"
assert_lacks "a hardware log is read with no jitter band" "$ran" "--grid-jitter-ns"
assert_contains "the run says where the log is" "$(output_of "$result")" "run-20260825T120000Z"

# ---------------------------------------------------------------------------
# What a run brings back besides the records
# ---------------------------------------------------------------------------
#
# The console output of the processes the launcher started, and the unit's clock
# discipline. Neither says anything about the machine: the driver's counters are
# the independent witness a recorded trail is cross-checked against, and a clock
# that steps is the loss of the time base every timestamp in the log is in. Both
# lived only on a tmpfs the next boot empties, and were recovered by hand the
# first time they were wanted.

assert_contains "the launcher's console directory is fetched whole" "$ran" \
	"root@unit:/run/brenn-app/logs/launch/"
assert_lacks "and no console filename is spelled out" "$ran" "motord_0.log"
console_dir=$(find "$run_dest" -mindepth 1 -maxdepth 1 -type d -name '*.console')
assert_contains "the console lands beside the records under the fetch's own stamp" \
	"$console_dir" "${run_dest}/motion-log-"
assert_eq "the driver's console came back" 1 \
	"$(find "$console_dir" -name 'motord*' | wc -l)"
assert_eq "so did the run's own console stream" 1 \
	"$(find "$console_dir" -name 'run-console.log' | wc -l)"
assert_eq "and the clock was read on both sides of the run" 2 \
	"$(find "$console_dir" -name 'clock-*.txt' | wc -l)"
assert_eq "and the kernel facts a bus measurement is read against came back" 1 \
	"$(find "$console_dir" -name 'host-facts.txt' | wc -l)"
# A motion run records no audio, so it fetches none: an empty third directory
# beside every motion log is one nobody could tell from a speech run whose
# store was lost.
assert_lacks "a motion run asks for no recorded audio" "$ran" "framelogs"
assert_eq "and leaves no audio directory beside its records" 0 \
	"$(find "$run_dest" -mindepth 1 -maxdepth 1 -type d -name '*.audio' | wc -l)"
assert_contains "the kernel facts ask what the timer tick is" "$ran" \
	"CONFIG_(HZ|HZ_[0-9]+|NO_HZ"
assert_contains "and say so where the kernel publishes no configuration" "$ran" \
	"this kernel does not publish its configuration"
assert_contains "and ask which driver is behind the serial ports" "$ran" \
	"/sys/class/tty/ttyAMA*"
assert_lacks "without naming the port the driver opens" "$ran" "ttyAMA3"
assert_contains "the clock reading asks the time daemon what it does" "$ran" \
	"timedatectl show"
assert_contains "and asks whichever NTP service the unit runs" "$ran" \
	"systemd-timesyncd"

# A probe is a diagnostic taken after the run, and the records it sits beside
# exist only on a tmpfs until the fetch. So a unit that answers nothing about
# its kernel or its clock leaves a file saying so and the run still reports:
# losing a completed run over a reading that did not land would be the whole
# point of the wrapper going the wrong way.
SSH_PROBE_STATUS=255
silent_dest="${work}/run-silent-probes"
result=$(deploy unit --run "$silent_dest")
assert_status "a run whose probes answered nothing still reports" 0 \
	"$(status_of "$result")"
silent_console=$(find "$silent_dest" -mindepth 1 -maxdepth 1 -type d -name '*.console')
assert_eq "and the kernel facts still came back as a file" 1 \
	"$(find "$silent_console" -name 'host-facts.txt' | wc -l)"
assert_contains "saying the unit answered nothing about its kernel" \
	"$(cat -- "${silent_console}/host-facts.txt")" "answered nothing about its kernel"
assert_contains "and the clock capture says the same about its clock" \
	"$(cat -- "$(find "$silent_console" -name 'clock-before.txt')")" \
	"answered nothing about its clock"
SSH_PROBE_STATUS=0

# The console is beside the records rather than under them for a reason worth
# pinning: the run directory is the newest *directory* under the fetched root,
# so a directory of console files there would be judged as the run.
assert_eq "nothing but run directories sits under the fetched records" 0 \
	"$(find "${run_dest}"/motion-log-*/ -mindepth 1 -maxdepth 1 -type d \
		! -name 'run-*' | wc -l)"

assert_lacks "the analyzer is handed nothing but the log" "$ran" "--console"

# The push's stamp is copied into the log root before the launcher starts, so it
# comes home at the root of the records with no fetch-side logic having placed it
# -- which the pinned remote command above is the evidence for. What is pinned
# here is the risk that root-level file introduces on the fetch side: a file
# beside the writer's run directory must survive the fetch and must not be taken
# for the run itself.
assert_eq "a root-level file in the fetched log root survives the fetch" 1 \
	"$(find "${run_dest}"/motion-log-*/ -maxdepth 1 -type f -name provenance.txt | wc -l)"
assert_contains "and the run directory is still the writer's own" "$ran" \
	"run-20260825T120000Z"

# A console copy that fails is a lost console and nothing more. It happens on the
# host, after the run, while the only copy of the records is still on a tmpfs the
# next boot empties -- so the fetch and the analyzer have to happen anyway.
TEE_STATUS=1
result=$(deploy unit --run "${work}/run-teefailed")
assert_status "a run whose console copy failed still reports" 0 "$(status_of "$result")"
assert_contains "and says the copy is what failed" "$(output_of "$result")" \
	"the console copy failed"
assert_contains "and the records were fetched anyway" "$(calls)" \
	"root@unit:/run/brenn-app/logs/testing/"
assert_contains "and the analyzer still judged the run" "$(calls)" \
	"//cogs:first_motion_report"
assert_contains "and the console that did land was filed" "$(output_of "$result")" \
	"no run-console.log to file with the records"
TEE_STATUS=0

# A launcher directory that came back without the driver's own console costs the
# verdict nothing: the driver republishes its whole account of the run into the
# log, so the analyzer reads no console at all.
RSYNC_CONSOLE=nodriver
result=$(deploy unit --run "${work}/run-nodriverconsole")
assert_status "a run whose driver console did not come back still reports" 0 \
	"$(status_of "$result")"
assert_contains "and the analyzer still judged the run" "$(calls)" \
	"//cogs:first_motion_report"
assert_lacks "and it was handed no console" "$(calls)" "--console"

RSYNC_CONSOLE=none
result=$(deploy unit --run "${work}/run-noconsole")
assert_status "a run whose launcher directory was empty still reports" 0 \
	"$(status_of "$result")"
assert_contains "and says the directory held nothing" "$(output_of "$result")" \
	"held no console files"
RSYNC_CONSOLE=full

# The log root the run empties is the configured one, and only a log root inside
# the payload store is emptied at all: the value comes out of a configuration
# file and the clear runs as root on the unit.
sed -i 's|log_root_dir: "/run/brenn-app/logs/testing"|log_root_dir: "/etc"|' \
	-- "$logger_config"
result=$(deploy unit --run "${work}/run-outside")
assert_status "a run whose log root is outside the payload store refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the value and where it has to be" \
	"$(output_of "$result")" "which is not under /run/brenn-app"
assert_lacks "and nothing was emptied on the unit" "$(calls)" "rm -rf"
stage_logger_config

# A log root that keeps the prefix and walks out of it anyway. The store check is
# textual and dots are in the accepted charset, so `..` is refused as its own
# thing -- a debug edit pointing the logs at flash would otherwise hand the
# remote `rm -rf` a path outside the store, as root.
sed -i 's|log_root_dir: "/run/brenn-app/logs/testing"|log_root_dir: "/run/brenn-app/../persistent"|' \
	-- "$logger_config"
result=$(deploy unit --run "${work}/run-traversal")
assert_status "a log root that walks out of the store refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the component it will not take" \
	"$(output_of "$result")" "which carries a . or .. component"
assert_lacks "and nothing was emptied on the unit" "$(calls)" "rm -rf"
stage_logger_config

# A log root that names the path the stamp is staged at. The staging path exists
# so that no copy failure can follow the wipe, and a log root colliding with it
# would wipe the stage and turn the move in into the alarming refusal the staging
# exists to prevent.
sed -i 's|log_root_dir: "/run/brenn-app/logs/testing"|log_root_dir: "/run/brenn-app/motion-provenance.staged"|' \
	-- "$logger_config"
result=$(deploy unit --run "${work}/run-staging-collision")
assert_status "a log root that collides with the staging path refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal says what that path is for" "$(output_of "$result")" \
	"which is where a run stages its provenance stamp"
assert_lacks "and nothing was emptied on the unit" "$(calls)" "rm -rf"
stage_logger_config

# And a log root *under* the staging path, which collides just as badly: the
# stage would be written into the parent of the directory the run then removes.
sed -i 's|log_root_dir: "/run/brenn-app/logs/testing"|log_root_dir: "/run/brenn-app/motion-provenance.staged/testing"|' \
	-- "$logger_config"
result=$(deploy unit --run "${work}/run-staging-under")
assert_status "a log root under the staging path refuses" 1 "$(status_of "$result")"
assert_contains "with the same refusal" "$(output_of "$result")" \
	"which is where a run stages its provenance stamp"
assert_lacks "and nothing was emptied on the unit" "$(calls)" "rm -rf"
stage_logger_config

# A log root that holds the payload. Everything ahead of the wipe passes, so the
# run would delete the binaries it is about to start and report it as a full or
# read-only store two steps later.
sed -i 's|log_root_dir: "/run/brenn-app/logs/testing"|log_root_dir: "/run/brenn-app/releases"|' \
	-- "$logger_config"
result=$(deploy unit --run "${work}/run-release-collision")
assert_status "a log root that holds the payload refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names what is in there" "$(output_of "$result")" \
	"which holds the payload at /run/brenn-app/releases/motion"
assert_lacks "and nothing was emptied on the unit" "$(calls)" "rm -rf"
stage_logger_config

# A records directory named relatively, which is what the Makefile passes. The
# analyzer runs under `bazel run`, from its own runfiles tree, so the path it is
# handed has to be absolute -- and the failure shape when it is not is an
# analyzer reporting no log over a directory full of records.
rel_dest=$(basename -- "${work}")/run-records-relative
result=$(cd -- "$(dirname -- "${work}")" && deploy unit --run "$rel_dest")
assert_status "a relative records directory runs" 0 "$(status_of "$result")"
assert_contains "and the analyzer is handed an absolute path" "$(calls)" 	"bazel run -- //cogs:first_motion_report ${work}/run-records-relative/motion-log-"

# The report's verdict is the wrapper's: a green run over a log the analyzer
# fails is a failed run.
BAZEL_STATUS=7
result=$(deploy unit --run "${work}/run-records-failed")
assert_status "the analyzer's verdict is the run's" 7 "$(status_of "$result")"
BAZEL_STATUS=0

# The exit statuses. 124 above is the budgeted stop and the only one that
# proceeds; every other reading is a failure, and none of them is read as
# "probably fine".
SSH_RUN_STATUS=0
result=$(deploy unit --run "${work}/run-early")
assert_status "a launcher that exited before the budget refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says the gesture did not finish" "$(output_of "$result")" \
	"before the 30s budget was up"
assert_contains "and says where its console output is" "$(output_of "$result")" \
	"/run/brenn-app/logs/launch"
assert_lacks "and nothing is fetched from a run that did not happen" "$(calls)" "rsync"
assert_lacks "and the analyzer is not run over nothing" "$(calls)" "first_motion_report"

SSH_RUN_STATUS=137
result=$(deploy unit --run "${work}/run-wedged")
assert_status "a launcher killed after the SIGINT grace refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says it did not stop on SIGINT" "$(output_of "$result")" \
	"did not stop on SIGINT"
assert_lacks "and nothing is fetched" "$(calls)" "rsync"

# The stamp is asked about before the log root is emptied and copied before the
# launcher is reached, so a payload pushed by something that did not stamp it
# stops the run there rather than recording a log whose build nothing names --
# and stops it with the previous run's records still on the unit to be fetched.
SSH_RUN_STATUS=5
result=$(deploy unit --run "${work}/run-unstamped")
assert_status "a unit with no stamp beside its payload refuses the run" 1 \
	"$(status_of "$result")"
assert_contains "the refusal says the records could not name their build" \
	"$(output_of "$result")" "could not name their build"
assert_contains "and says to push again" "$(output_of "$result")" "--push"
assert_contains "and says the unit was left as it was" "$(output_of "$result")" \
	"nothing was emptied"
assert_lacks "and does not tell the copy's story" "$(output_of "$result")" \
	"copying it into"
assert_lacks "and nothing is fetched" "$(calls)" "rsync"

# The same code out of a launcher that was actually reached is the launcher's,
# not the chain's: the sentinel the chain echoes past its last refusal is what
# says which, so the refusal cannot claim that nothing was started about a run
# that ran.
SSH_RUN_STATUS=5
SSH_RUN_REACHED=yes
result=$(deploy unit --run "${work}/run-launcher-5")
assert_status "a launcher that itself exited 5 is not read as a missing stamp" 1 \
	"$(status_of "$result")"
assert_contains "it is judged as a run that failed" "$(output_of "$result")" \
	"the run on unit failed (exit 5)"
assert_lacks "and nothing claims the stamp was missing" "$(output_of "$result")" \
	"could not name their build"
SSH_RUN_REACHED=no

# The stamp is staged before the wipe rather than copied after it, so a copy
# that fails has emptied nothing and the refusal can send the operator to fetch
# the records that are still there.
SSH_RUN_STATUS=6
result=$(deploy unit --run "${work}/run-stamp-unstaged")
assert_status "a stamp that could not be staged refuses the run" 1 \
	"$(status_of "$result")"
assert_contains "the refusal says the stamp is there and the copy failed" \
	"$(output_of "$result")" "carries its provenance stamp, but copying it to"
assert_contains "and names the staging path" "$(output_of "$result")" \
	"/run/brenn-app/motion-provenance.staged"
assert_contains "and says the unit was left as it was" "$(output_of "$result")" \
	"nothing was emptied"
assert_contains "and says the previous run is still fetchable" "$(output_of "$result")" \
	"--fetch"
assert_contains "and says the launcher never started" "$(output_of "$result")" \
	"Nothing was started"
assert_lacks "and nothing is fetched" "$(calls)" "rsync"
assert_lacks "and the analyzer is not run over nothing" "$(calls)" "first_motion_report"

# The four steps that run once the wipe has begun -- the log root's mkdir, the
# stamp's move into it, the launcher console directory's clear, the cd -- share
# one code, because the one thing an operator needs from any of them is that the
# previous run's records are gone.
SSH_RUN_STATUS=7
result=$(deploy unit --run "${work}/run-post-wipe")
assert_status "a preparation step that failed after the wipe refuses the run" 1 \
	"$(status_of "$result")"
assert_contains "the refusal says the wipe had begun" "$(output_of "$result")" \
	"after the log root wipe had begun"
assert_contains "and says to treat the previous records as gone" \
	"$(output_of "$result")" "as gone"
assert_contains "and says the launcher never started" "$(output_of "$result")" \
	"launcher was not started"
assert_lacks "and does not send the operator to a console nothing wrote" \
	"$(output_of "$result")" "/run/brenn-app/logs/launch"
assert_lacks "and does not claim the unit was left as it was" "$(output_of "$result")" \
	"nothing was emptied"
assert_lacks "and nothing is fetched" "$(calls)" "rsync"
assert_lacks "and the analyzer is not run over nothing" "$(calls)" "first_motion_report"

# The launcher config `--run` names is asked about on the unit, before the stamp
# and before the wipe: the push checks the local payload carries both configs,
# but a run is a separate invocation against whatever is already there, and a
# unit still holding a payload staged before the harness twin existed would die
# inside `simplelaunch` with the log root already cleared.
SSH_RUN_STATUS=8
result=$(deploy unit --run "${work}/run-noconfig")
assert_status "a unit whose payload has no harness config refuses the run" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the config that is not there" \
	"$(output_of "$result")" "has no robotcpu_harness.textproto"
assert_contains "and says the payload predates it" "$(output_of "$result")" \
	"predates the harness twin"
assert_contains "and says to push again" "$(output_of "$result")" "--push"
assert_contains "and says the unit was left as it was" "$(output_of "$result")" \
	"nothing was emptied"
assert_lacks "and nothing is fetched" "$(calls)" "rsync"
assert_lacks "and the analyzer is not run over nothing" "$(calls)" "first_motion_report"

# A code the run path does not read at all, which is what the catch-all arm is
# for.
SSH_RUN_STATUS=9
result=$(deploy unit --run "${work}/run-failed")
assert_status "any other status refuses" 1 "$(status_of "$result")"
assert_contains "and carries the code" "$(output_of "$result")" "exit 9"
assert_lacks "and nothing is fetched" "$(calls)" "rsync"

# A launcher killed by a signal, which the remote shell reports as 128+signal
# because a shell is kept in front of it. Without one, ssh reports
# its own 255 instead and a payload binary that faults reads as an unreachable
# host — which is how a SIGILL from a payload compiled for the wrong ISA first
# looked at a bench.
SSH_RUN_STATUS=132
result=$(deploy unit --run "${work}/run-signalled")
assert_status "a launcher killed by a signal refuses" 1 "$(status_of "$result")"
assert_contains "and carries the shell's code rather than ssh's" "$(output_of "$result")" \
	"exit 132"
assert_lacks "and is not reported as ssh failing" "$(output_of "$result")" \
	"ssh to root@unit failed"

# The probe is binding for a run, unlike for a push: the question and the work
# are one invocation, so a service holding the bus stops the run before it
# starts.
SSH_RUN_STATUS=3
result=$(deploy unit --run "${work}/run-held")
assert_status "the payload holding the bus refuses a run" 1 "$(status_of "$result")"
assert_contains "it names the service holding it" "$(output_of "$result")" \
	"brenn-app.service is running"
assert_lacks "and nothing is fetched" "$(calls)" "rsync"

SSH_RUN_STATUS=4
result=$(deploy unit --run "${work}/run-motiond")
assert_status "the motion daemon holding the bus refuses a run" 1 "$(status_of "$result")"
assert_contains "the way back is named" "$(output_of "$result")" \
	"systemctl start reachy-motiond.service"

# 255 is ssh's own code and also a run's, if the launcher exits with it, so the
# refusal says both rather than asserting that nothing happened on the unit.
SSH_RUN_STATUS=255
result=$(deploy unit --run "${work}/run-unreachable")
assert_status "ssh failing refuses" 1 "$(status_of "$result")"
assert_contains "and does not claim the run never started" "$(output_of "$result")" \
	"also the run's if the launcher exited with it"
assert_contains "and says how to keep this run's records" "$(output_of "$result")" \
	"--fetch <records-dir> first"
SSH_RUN_STATUS=124

# A run whose records never arrived is the fetch's refusal, unchanged by being
# reached from here, and the analyzer is never offered an empty directory.
RSYNC_OLOG=none
result=$(deploy unit --run "${work}/run-norecords")
assert_status "a run that wrote no records refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says there are no records" "$(output_of "$result")" \
	"no .olog with anything in it"
assert_lacks "and the analyzer was never run" "$(calls)" "first_motion_report"
RSYNC_OLOG=full

# A configuration that lost the one field a run needs is a refusal before
# anything is started.
sed -i '/^log_root_dir:/d' -- "$logger_config"
result=$(deploy unit --run "${work}/run-nofield")
assert_status "a configuration with no log root refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the field" "$(output_of "$result")" \
	"states no log_root_dir"
assert_lacks "and nothing was started" "$(calls)" "simplelaunch"
stage_logger_config

# ---------------------------------------------------------------------------
# Values that mean something to a shell
# ---------------------------------------------------------------------------
#
# The log root is pasted into a remote command run as root and into a remote
# rsync path the far end re-parses. A value with a space in it means a different
# thing at each of those sites, so it is refused where it is read rather than
# quoted differently at each.

sed -i 's|^log_root_dir: .*|log_root_dir: "/run/brenn-app/logs/two words"|' -- "$logger_config"
result=$(deploy unit --push)
assert_status "a log root with a space refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the field and the value" "$(output_of "$result")" \
	"log_root_dir is '/run/brenn-app/logs/two words'"
assert_contains "and says what is accepted" "$(output_of "$result")" "[A-Za-z0-9/_.-]"
assert_lacks "and nothing is pushed" "$(calls)" "rsync"
result=$(deploy unit --fetch "${work}/spaced")
assert_status "and the fetch refuses the same value" 1 "$(status_of "$result")"
assert_lacks "the fetch reaches no device" "$(calls)" "rsync"
stage_logger_config

sed -i 's|^log_root_dir: .*|log_root_dir: "/run/brenn-app/logs; rm -rf /"|' -- "$logger_config"
result=$(deploy unit --run "${work}/run-injected")
assert_status "a log root carrying a command refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names that field and its value" "$(output_of "$result")" \
	"log_root_dir is '/run/brenn-app/logs; rm -rf /'"
assert_lacks "and nothing was started with it" "$(calls)" "simplelaunch"
stage_logger_config

# The shipped shapes are accepted, so the check above is a check on values and
# not a ban on paths.
result=$(deploy unit --run "${work}/run-ordinary")
assert_status "an ordinary path and name are accepted" 0 "$(status_of "$result")"

# ---------------------------------------------------------------------------
# Fetching the run's records
# ---------------------------------------------------------------------------

dest="${work}/records"
result=$(deploy unit --fetch "$dest")
assert_status "a fetch succeeds" 0 "$(status_of "$result")"
assert_contains "the fetch reads the log root out of the configuration" "$(calls)" \
	"root@unit:/run/brenn-app/logs/testing/"
assert_contains "the fetch names the analyzer" "$(output_of "$result")" "first_motion_report"
assert_contains "and hands it the records and nothing else" "$(output_of "$result")" \
	"first_motion_report -- ${dest}/motion-log-"
fetched=$(find "$dest" -mindepth 1 -maxdepth 1 -type d ! -name '*.console' | wc -l)
if [ "$fetched" = 1 ]; then
	pass "the fetch lands in one stamped directory"
else
	fail "the fetch lands in one stamped directory" "found ${fetched}"
fi
assert_eq "with the run's consoles beside it" 1 \
	"$(find "$dest" -mindepth 1 -maxdepth 1 -type d -name '*.console' | wc -l)"
assert_lacks "nothing partial is left behind" "$(find "$dest" -maxdepth 1)" ".part"

# A fetch that succeeded and brought nothing readable is a refusal. rsync is
# happy about an empty source, so its status says nothing about whether the run
# logged — and the report command printed over an empty directory is how a
# namespace mismatch used to survive a whole bench session unnoticed.
empty_dest="${work}/records-empty"
RSYNC_OLOG=none
result=$(deploy unit --fetch "$empty_dest")
assert_status "a fetch that brought nothing refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says there are no records" "$(output_of "$result")" \
	"no .olog with anything in it"
assert_contains "and says where the logger was told to look" "$(output_of "$result")" \
	"cogs/robot_logger.textproto"
assert_lacks "and does not offer the analyzer over nothing" "$(output_of "$result")" \
	"first_motion_report"
assert_eq "and keeps nothing" 0 \
	"$(find "$empty_dest" -mindepth 1 -maxdepth 1 | wc -l)"

# A zero-length `.olog` is the same refusal: the writer opened a file and the run
# put nothing in it.
zero_dest="${work}/records-zero"
RSYNC_OLOG=empty
result=$(deploy unit --fetch "$zero_dest")
assert_status "a fetch that brought an empty .olog refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says the same thing" "$(output_of "$result")" \
	"no .olog with anything in it"
assert_eq "and keeps nothing either" 0 \
	"$(find "$zero_dest" -mindepth 1 -maxdepth 1 | wc -l)"
RSYNC_OLOG=full

# Its own destination, because the fetch above may have been in this same second
# and a stamp collision is now its own refusal — which is the case after this
# one.
failed_dest="${work}/records-failed"
RSYNC_STATUS=23
result=$(deploy unit --fetch "$failed_dest")
assert_status "a failed fetch refuses" 1 "$(status_of "$result")"
assert_contains "it says where it looked" "$(output_of "$result")" \
	"/run/brenn-app/logs/testing"
assert_lacks "a failed fetch leaves no directory to analyse" \
	"$(find "$failed_dest" -maxdepth 1)" ".part"
RSYNC_STATUS=0

# Two fetches inside one UTC second. `mv` moves a directory into an existing one
# of the same name, so this used to file the second run's records at
# motion-log-<stamp>/motion-log-<stamp>.part while printing the first run's path.
# The stamp cannot be finer than the name it makes, so the collision is refused.
#
# The stamp is the subject's own `date`, so the destination is pre-created for
# this second and the next several: the second can tick between this line and the
# subject's read, and a self-check that fails once an hour is a self-check
# nobody believes. Ten seconds, because three cases each run a whole fetch and
# the window is measured under `make check`, where the machine is also building.
collide="${work}/collide"
occupy() {
	local dir=$1 suffix=$2 ahead
	for ahead in 0 1 2 3 4 5 6 7 8 9; do
		mkdir -p -- "${dir}/motion-log-$(date -u -d "+${ahead} seconds" +%Y%m%dT%H%M%SZ)${suffix}"
	done
}
occupy "$collide" ""
result=$(deploy unit --fetch "$collide")
assert_status "a second fetch in the same second refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says it has nowhere of its own to land" "$(output_of "$result")" \
	"has nowhere of its own to land"
nested=$(find "$collide" -mindepth 2 | wc -l)
assert_eq "and nothing is nested inside the first fetch" 0 "$nested"

# A leftover .part is refused for the same reason: rsync would merge into it.
leftover="${work}/leftover"
occupy "$leftover" ".part"
result=$(deploy unit --fetch "$leftover")
assert_status "a leftover partial directory refuses" 1 "$(status_of "$result")"
assert_lacks "and nothing is fetched into it" "$(calls)" "rsync"

# And a leftover console directory, for a reason of its own: rsync would merge
# this run's launcher files into the last run's, and a console directory holding
# two runs' driver logs is one the report refuses to cross-check against — or
# worse, cross-checks against the wrong run's counters.
stale_console="${work}/stale-console"
occupy "$stale_console" ".console"
result=$(deploy unit --fetch "$stale_console")
assert_status "a leftover console directory refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says it has nowhere of its own to land" "$(output_of "$result")" \
	"has nowhere of its own to land"
assert_lacks "and nothing is fetched beside it" "$(calls)" "rsync"

# And a leftover audio directory, for the same reason as the console's: two
# runs' framelogs merged into one directory are clips nobody can attribute to
# the turn they came from, which is the whole use of them.
stale_audio="${work}/stale-audio"
occupy "$stale_audio" ".audio"
result=$(deploy unit --fetch "$stale_audio")
assert_status "a leftover audio directory refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says it has nowhere of its own to land" "$(output_of "$result")" \
	"has nowhere of its own to land"
assert_lacks "and nothing is fetched beside it" "$(calls)" "rsync"

# And a leftover turn-clip directory: the report writes `turn-NN.wav` there by
# this run's own turn numbering, so another run's clips under those names are
# audio attributed to a turn that never carved it.
stale_turns="${work}/stale-turns"
occupy "$stale_turns" ".turns"
result=$(deploy unit --fetch "$stale_turns")
assert_status "a leftover turn-clip directory refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says it has nowhere of its own to land" "$(output_of "$result")" \
	"has nowhere of its own to land"
assert_lacks "and nothing is fetched beside it" "$(calls)" "rsync"

# ---------------------------------------------------------------------------
# The speech run
# ---------------------------------------------------------------------------
#
# The production launcher config, no budget, and a person in front of the
# machine. What is pinned here is everything that is different from `--run`: the
# preflights that refuse before a device is touched, the chain's own two
# questions of the unit, a launcher started with no `timeout` and no intent
# source, and a fetch and a report that happen however the run ended.
#
# The mode refuses a stdin that is not a terminal, and this suite's is not, so
# the cases that get past that refusal run the subject under a pty of their own.
cat >"${work}/pty-run.py" <<'PTY'
"""Run a command with a pty on its stdin, and exit with its status.

The subject refuses a speech run it could not receive a ^C for, which is the
whole of what makes the mode supervised. A suite whose own stdin is a pipe can
only exercise that refusal, so the cases past it get a terminal made here.
"""

import os
import pty
import sys

sys.exit(os.waitstatus_to_exitcode(pty.spawn(["/bin/sh", "-c", sys.argv[1]])))
PTY

# The subject under a pty, otherwise `deploy`. The carriage returns a terminal
# adds are stripped, so a case asserts on the same text either way.
deploy_tty() {
	: >"$CALLS"
	local out status=0 command
	printf -v command '%q ' "$subject" "$@"
	out=$(python3 "${work}/pty-run.py" "$command" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$(printf '%s' "$out" | tr -d '\r')" "$status"
}

# The staged speech configuration, in the payload where the build puts it: the
# copy the host will load, which is where the endpoints the preflight asks
# about are read from.
stage_speech_config() {
	mkdir -p -- "${payload}/host"
	cat >"${payload}/host/speech.toml" <<'TOML'
listen_addr = "127.0.0.1:7380"
pod_psk_file = "secrets/pod-psk.toml"

[stt]
url = "http://speaches.example:8000"

[tts]
url = "http://speaches.example:8001"
TOML
}

# The configuration checker: the host binary, built here and run with the
# payload root as its working directory. The stub records both, because what
# makes the check worth anything is that it resolved the payload's own relative
# paths.
mkdir -p -- "${repo}/bazel-out"
cat >"${repo}/bazel-out/reachy_host" <<'STUB'
#!/usr/bin/env bash
printf 'reachy_host %s cwd=%s\n' "$*" "$PWD" >>"$CALLS"
exit "${SPEECH_CHECK_STATUS:-0}"
STUB
chmod 0755 -- "${repo}/bazel-out/reachy_host"
export SPEECH_CHECK_STATUS=0
export BAZEL_BUILD_STATUS=0

stage_payload "$after"
stage_speech_config

# A payload with no speech configuration is a good motion payload and no
# pipeline at all, and the refusal says which command puts one in it.
rm -f -- "${payload}/host/speech.toml"
result=$(deploy unit --speech "${work}/speech-voiceless")
assert_status "a voiceless payload refuses with its own code" 9 "$(status_of "$result")"
assert_contains "the refusal names the member that is missing" "$(output_of "$result")" \
	"carries no host/speech.toml"
assert_contains "and the knob that names one" "$(output_of "$result")" \
	"REACHY_SPEECH_CONFIG"
assert_contains "and the build that stages it" "$(output_of "$result")" "make motion-build"
assert_lacks "and nothing reaches the device" "$(calls)" "ssh"
stage_speech_config

# This suite's stdin is a pipe, which is exactly the run nobody could stop: a
# speech run has no budget, so it is refused rather than warned about.
result=$(deploy unit --speech "${work}/speech-notty")
assert_status "a speech run with no terminal refuses with its own code" 10 \
	"$(status_of "$result")"
assert_contains "the refusal says why the terminal matters" "$(output_of "$result")" \
	"stdin is not a terminal"
assert_contains "and that this run has no budget to end it instead" "$(output_of "$result")" \
	"has no budget"
assert_lacks "and nothing reaches the device" "$(calls)" "ssh"
assert_lacks "and nothing was built for it either" "$(calls)" "bazel"

# The same question asked alone, which is what `make speech-run` asks before it
# provisions the unit and builds a payload: one spelling of the refusal, and the
# only preflight that needs neither.
result=$(deploy unit --speech-preflight)
assert_status "the preflight alone refuses with the run's own code" 10 \
	"$(status_of "$result")"
assert_contains "in the same words the run would have used" "$(output_of "$result")" \
	"stdin is not a terminal"
assert_lacks "and nothing reaches the device" "$(calls)" "ssh"
assert_lacks "and nothing is built" "$(calls)" "bazel"

# With a terminal it says nothing and succeeds: it is a gate, not a step.
result=$(deploy_tty unit --speech-preflight)
assert_status "under a terminal the preflight passes" 0 "$(status_of "$result")"
assert_lacks "having touched nothing on the device" "$(calls)" "ssh"

# It takes no destination: a mode that fetches nothing has nowhere to put it.
result=$(deploy unit --speech-preflight "${work}/speech-records")
assert_status "the preflight takes no arguments" 1 "$(status_of "$result")"
assert_contains "and says so in the usage" "$(output_of "$result")" "usage:"
# The usage line is the menu a mistyped mode lands on, so it has to list this
# mode: a mode absent from it is one an operator cannot recover their way to.
assert_contains "which names this mode among the others" "$(output_of "$result")" \
	"--speech-preflight"

# The run itself, under a terminal of its own.
speech_dest="${work}/speech-records"
result=$(deploy_tty unit --speech "$speech_dest")
ran=$(calls)
assert_status "a supervised speech run the analyzer passes succeeds" 0 "$(status_of "$result")"
assert_contains "the configuration is checked before any device is touched" "$ran" \
	"reachy_host --speech-config host/speech.toml --check cwd=${payload}"
assert_contains "the checker is built in the default configuration" "$ran" \
	"bazel build -- //crates/reachy-host:reachy_host"
assert_contains "the bus question, the pipeline's preflights and the launcher are one invocation" "$ran" \
	"systemctl is-active --quiet brenn-app.service && exit 3; systemctl is-active --quiet reachy-motiond.service && exit 4; [ -f /run/brenn-app/releases/motion/robotcpu.textproto ] || exit 8; [ -f /run/brenn-app/releases/motion/provenance.txt ] || exit 5; [ -s /run/brenn-app/conf/audio.conf ] || exit 12; curl -sS --max-time 5 -o /dev/null http://speaches.example:8000/v1/models || exit 13; curl -sS --max-time 5 -o /dev/null http://speaches.example:8001/v1/models || exit 13; cp -- /run/brenn-app/releases/motion/provenance.txt /run/brenn-app/motion-provenance.staged || exit 6; rm -rf -- /run/brenn-app/logs/testing && mkdir -p -- /run/brenn-app/logs/testing || exit 7; mv -- /run/brenn-app/motion-provenance.staged /run/brenn-app/logs/testing/provenance.txt || exit 7; rm -rf -- /run/brenn-app/logs/launch && mkdir -p -- /run/brenn-app/logs/launch || exit 7; cd /run/brenn-app/releases/motion || exit 7; echo ---brenn-launcher-starting; tail -F /run/brenn-app/logs/launch/voice_host_0.log 2>/dev/null & tail_pid=\$!; ./simplelaunch robotcpu.textproto --logdir /run/brenn-app/logs/launch; rc=\$?; kill \$tail_pid 2>/dev/null; exit \$rc"
assert_contains "the run gets a pty, so a ^C reaches the unit" "$ran" \
	"ssh -t -o BatchMode=yes root@unit"
# The voice host's console reaches the operator while the run is happening, not
# only in the fetched records afterwards: the tail starts before the launcher
# does and is killed after it returns. A run that lost this is one where a
# pipeline dying in its first second is invisible to the person talking to it.
assert_contains "the voice host's console is tailed onto the pty before the launcher starts" \
	"$ran" \
	"tail -F /run/brenn-app/logs/launch/voice_host_0.log 2>/dev/null & tail_pid=\$!; ./simplelaunch"
assert_contains "and the tail is killed once the launcher returns" "$ran" \
	"rc=\$?; kill \$tail_pid 2>/dev/null; exit \$rc"
assert_lacks "nothing puts a budget around a conversation" "$ran" "timeout --signal=INT"
assert_lacks "and the harness intent source is not started beside the host" "$ran" \
	"./reachy_ask"
assert_lacks "nor is the harness config the one that starts" "$ran" \
	"./simplelaunch robotcpu_harness.textproto"
assert_contains "the records come back under the speech name" "$ran" \
	"//cogs:speech_run_report ${speech_dest}/speech-log-"
assert_lacks "and not under the motion one" "$ran" "motion-log-"
assert_lacks "the motion analyzer has nothing to say about a conversation" "$ran" \
	"first_motion_report"
assert_contains "the run says where the console is" "$(output_of "$result")" \
	"speech-log-"

# The report is handed the fetch itself, not a run directory inside it: the
# console it reads is named from that spelling.
fetched=$(find "$speech_dest" -mindepth 1 -maxdepth 1 -type d \
	! -name '*.console' ! -name '*.audio')
assert_contains "the analyzer is handed the fetched directory" "$ran" \
	"//cogs:speech_run_report ${fetched}"
assert_eq "with the run's consoles beside it" 1 \
	"$(find "$speech_dest" -mindepth 1 -maxdepth 1 -type d -name '*.console' | wc -l)"
assert_file "and the host-side captures were filed with them" \
	"${fetched}.console/clock-before.txt"

# The third sibling: the audio the pipeline heard, which is the only evidence
# that says what Whisper was given rather than what it made of it. Beside the
# records for the reason the console is beside them — `run_directory` takes the
# newest directory under the fetched root as the run.
assert_contains "the record store is fetched from under the release directory" "$ran" \
	"rsync -a -e ssh -o BatchMode=yes root@unit:/run/brenn-app/releases/motion/framelogs/ ${fetched}.audio/"
assert_eq "and the audio lands beside the records, not inside them" 1 \
	"$(find "$speech_dest" -mindepth 1 -maxdepth 1 -type d -name '*.audio' | wc -l)"
assert_eq "with the framelogs the run wrote in it" 2 \
	"$(find "${fetched}.audio" -name '*.framelog' | wc -l)"
assert_eq "and nothing inside the run directory itself" 0 \
	"$(find "$fetched" -name '*.framelog' | wc -l)"

# A site whose configuration records nothing. The store is empty or absent and
# the run still reports: what a speech run is judged on is its console, and the
# audio is evidence beside it rather than the evidence itself.
RSYNC_AUDIO=none
result=$(deploy_tty unit --speech "${work}/speech-noaudio")
assert_status "a run whose store held no audio still reports" 0 "$(status_of "$result")"
assert_contains "and says the store held none" "$(output_of "$result")" \
	"holds no recorded audio"
RSYNC_AUDIO=full

# And a store the fetch could not reach at all: best-effort throughout, because
# a run's console is not worth losing to a directory that was never created.
RSYNC_AUDIO=refused
result=$(deploy_tty unit --speech "${work}/speech-audiorefused")
assert_status "a run whose audio fetch failed still reports" 0 "$(status_of "$result")"
assert_contains "and says nothing came back" "$(output_of "$result")" \
	"no recorded audio fetched"
# And leaves nothing behind: the line explaining the failure scrolls away, so an
# empty .audio on disk has to mean the site recorded nothing and nothing else.
assert_eq "and left no empty audio directory behind" 0 \
	"$(find "${work}/speech-audiorefused" -mindepth 1 -maxdepth 1 -type d -name '*.audio' | wc -l)"
RSYNC_AUDIO=full

# Which store is fetched is the deployed configuration's answer, not this
# script's: the build and the fetch cannot disagree about where the audio is,
# the way they cannot about where the payload's credentials are. A site that
# renames the store and a fetch that kept looking under the old name would print
# the same "holds no recorded audio" line as a site recording nothing.
cat >>"${payload}/host/speech.toml" <<'TOML'

[record]
enabled = true
dir = "heard"
TOML
result=$(deploy_tty unit --speech "${work}/speech-renamed-store")
assert_status "a run whose configuration renames the store still reports" 0 \
	"$(status_of "$result")"
assert_contains "and the fetch reads the name out of the staged configuration" "$(calls)" \
	"root@unit:/run/brenn-app/releases/motion/heard/"
stage_speech_config

# An absolute store is one the preflight refuses -- and is still where the audio
# would land if such a run ever happened, so the fetch reads it as written
# rather than under the release directory. A prefix here would look in a place
# nothing ever wrote and print the same line a site recording nothing does.
cat >>"${payload}/host/speech.toml" <<'TOML'

[record]
enabled = true
dir = "/srv/heard"
TOML
result=$(deploy_tty unit --speech "${work}/speech-absolute-store")
assert_status "a run whose configuration names an absolute store still reports" 0 \
	"$(status_of "$result")"
assert_contains "and the fetch reads that path as written" "$(calls)" \
	"root@unit:/srv/heard/"
assert_lacks "with no release directory in front of it" "$(calls)" \
	"releases/motion//srv/heard"
stage_speech_config

# With no staged configuration to read, the fetch falls back to the name every
# site uses -- and says which of the two an empty store would mean.
rm -f -- "${payload}/host/speech.toml"
result=$(deploy unit --speech-fetch "${work}/speech-fetch-fallback")
assert_contains "the fallback store is the one fetched" "$(calls)" \
	"root@unit:/run/brenn-app/releases/motion/framelogs/"
stage_speech_config

# The report's verdict is this mode's, as it is for a motion run.
BAZEL_STATUS=7
result=$(deploy_tty unit --speech "${work}/speech-failed")
assert_status "the speech analyzer's verdict is the run's" 7 "$(status_of "$result")"
BAZEL_STATUS=0

# A configuration the host itself would refuse never reaches the unit. This is
# the whole point of running the real loader here: on the unit it is a host that
# exits at start, in a log somebody has to fetch to read.
SPEECH_CHECK_STATUS=1
result=$(deploy_tty unit --speech "${work}/speech-refused")
assert_status "a configuration the checker refuses refuses the run" 11 \
	"$(status_of "$result")"
assert_contains "the refusal names the check" "$(output_of "$result")" "--check"
assert_contains "and says the fix is beside the configuration" "$(output_of "$result")" \
	"make motion-build"
assert_lacks "and nothing reaches the device" "$(calls)" "ssh -t"
SPEECH_CHECK_STATUS=0

BAZEL_BUILD_STATUS=1
result=$(deploy_tty unit --speech "${work}/speech-nochecker")
assert_status "a checker that will not build refuses the run" 11 "$(status_of "$result")"
assert_contains "and says the configuration went unchecked" "$(output_of "$result")" \
	"was not checked"
assert_lacks "and nothing reaches the device either" "$(calls)" "ssh -t"
BAZEL_BUILD_STATUS=0

# The unit's two preflights, each answered by the code the chain emits for it.
SSH_RUN_STATUS=12
# The assembly configuration this run was built from, so the refusal can be
# checked against the real path rather than against a placeholder.
mkdir -p -- "${work}/assembly"
cp -- "${payload}/host/speech.toml" "${work}/assembly/speech.toml"
REACHY_SPEECH_CONFIG="${work}/assembly/speech.toml"
export REACHY_SPEECH_CONFIG
result=$(deploy_tty unit --speech "${work}/speech-noaudioconf")
assert_status "a unit with no link credentials refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the file" "$(output_of "$result")" \
	"/run/brenn-app/conf/audio.conf"
# The lead is the target that writes the file for you. Reaching this refusal at
# all means either this script was invoked directly or the unit lost its tmpfs
# since the provisioning ran, and the first is far the likelier — so the remedy
# offered first is the one command that covers both.
assert_contains "and leads with the target that provisions it" "$(output_of "$result")" \
	"make speech-run"
assert_contains "and the other repo's command that writes it" "$(output_of "$result")" \
	"make -C firmware reachy-provision"
assert_contains "and says the pod would have parked silently" "$(output_of "$result")" \
	"park silently"
# The flag is required for a loopback listen address; without it in the
# message the operator hits a second refusal.
assert_contains "and the opt-in that command needs for an on-unit host" \
	"$(output_of "$result")" "ON_UNIT=1"
# A command the operator must edit before running is one more thing to get wrong
# while already blocked, so the configuration this run knows about is in it —
# quoted, because it is pasted as it stands and a path with a space in it splits
# into two arguments the other repo's make cannot read.
assert_contains "and the configuration to point that command at" \
	"$(output_of "$result")" "SPEECH_CONFIG=\"${work}/assembly/speech.toml\""
# And the unit, which needs no confirming dance — this refusal came from it.
# Omitted, the remediation command may target a different unit, and this one
# refuses again identically.
assert_contains "and the unit the command has to provision" \
	"$(output_of "$result")" "REACHY_HOST=unit"
assert_lacks "and nothing is fetched from a run that never started" "$(calls)" "rsync"

# The variable is read here, at deploy time; the payload was staged by an
# earlier build. A named configuration that is not the one the payload carries
# is not named at all: provisioning from it derives the pod's address and key
# from a file the host never loads, and the next run composes and sits deaf —
# the failure this refusal exists to head off, arriving by the refusal's own
# advice.
cp -- "${work}/assembly/speech.toml" "${work}/assembly/other.toml"
printf 'ident = "somewhere else"\n' >>"${work}/assembly/other.toml"
REACHY_SPEECH_CONFIG="${work}/assembly/other.toml"
result=$(deploy_tty unit --speech "${work}/speech-otherconfig")
assert_lacks "a configuration that is not the payload's is not handed back" \
	"$(output_of "$result")" "other.toml"
assert_contains "the placeholder goes back in instead" "$(output_of "$result")" \
	"SPEECH_CONFIG=\"<assembly>/speech.toml\""
assert_contains "and says why the path is the operator's to supply" \
	"$(output_of "$result")" "could not confirm"
unset REACHY_SPEECH_CONFIG

# The same with the variable unset: the default is this tree's gitignored
# host/speech.toml, which a payload built elsewhere has nothing to do with and
# which need not exist at all. Naming a file that is not there is a second dead
# end at the one moment the message is meant to unblock somebody.
result=$(deploy_tty unit --speech "${work}/speech-defaultconfig")
assert_contains "an unnamed configuration is the placeholder too" \
	"$(output_of "$result")" "SPEECH_CONFIG=\"<assembly>/speech.toml\""
assert_lacks "and this tree's default is not passed off as the payload's" \
	"$(output_of "$result")" "${repo}/host/speech.toml"

SSH_RUN_STATUS=13
result=$(deploy_tty unit --speech "${work}/speech-unreachable")
assert_status "a speech service the robot cannot reach refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says whose vantage decided" "$(output_of "$result")" \
	"from the unit"
assert_contains "and names the migration error it is usually" "$(output_of "$result")" \
	"never by localhost"
assert_lacks "and nothing is fetched" "$(calls)" "rsync"

SSH_RUN_STATUS=3
result=$(deploy_tty unit --speech "${work}/speech-held")
assert_status "a held servo bus refuses a speech run" 1 "$(status_of "$result")"
assert_contains "and names the service holding it" "$(output_of "$result")" \
	"brenn-app.service is running"
assert_lacks "and nothing is fetched" "$(calls)" "rsync"

SSH_RUN_STATUS=5
result=$(deploy_tty unit --speech "${work}/speech-nostamp")
assert_status "a payload on the unit with no stamp refuses" 1 "$(status_of "$result")"
assert_contains "and says nothing was emptied" "$(output_of "$result")" \
	"nothing was emptied"
assert_lacks "and nothing is fetched" "$(calls)" "rsync"

SSH_RUN_STATUS=7
result=$(deploy_tty unit --speech "${work}/speech-postwipe")
assert_status "a failure after the wipe refuses" 1 "$(status_of "$result")"
assert_contains "and says the previous run's records are gone" "$(output_of "$result")" \
	"as gone"

# Every other ending is a run that happened. Which of them it was is the
# report's question over the console the fetch brings back, not a refusal here:
# a launcher that stopped on the operator's ^C, one that exited by itself, and a
# terminal that died mid-run all leave records worth reading.
SSH_RUN_REACHED=yes
for ending in 0 130 137 255; do
	SSH_RUN_STATUS=$ending
	result=$(deploy_tty unit --speech "${work}/speech-ending-${ending}")
	assert_status "a run that ended with ${ending} is fetched and reported" 0 \
		"$(status_of "$result")"
	assert_contains "and the report reads it" "$(calls)" "//cogs:speech_run_report"
	assert_contains "and the ending is said rather than judged" "$(output_of "$result")" \
		"the run ended (exit ${ending})"
done

# The chain's own codes are small integers and so are a launcher's. What tells
# them apart is the sentinel the chain echoes past its last refusal, not the
# number: a launcher exiting 5 with that line in the console is a supervised
# session whose records are fetched, and reading it as a payload with no
# provenance stamp would leave them on tmpfs until the next run wiped them.
for ending in 5 6 7 8 12 13; do
	SSH_RUN_STATUS=$ending
	result=$(deploy_tty unit --speech "${work}/speech-launcher-${ending}")
	assert_status "a launcher that itself exited ${ending} is still fetched" 0 \
		"$(status_of "$result")"
	assert_contains "and its records are reported over" "$(calls)" \
		"//cogs:speech_run_report"
done
# A speech run's evidence is the console, and the channel log is the motion
# run's. A launcher that died in its first seconds -- the host refusing its
# configuration on the unit, the pod taking the composition down, an immediate
# ^C -- leaves a log root with nothing in it worth reading, and refusing there
# would drop the console rsync that explains it and leave the only copy on the
# unit's tmpfs until the next run's wipe.
RSYNC_OLOG=empty
SSH_RUN_STATUS=0
result=$(deploy_tty unit --speech "${work}/speech-empty-olog")
assert_status "a speech run that recorded no channels is still reported on" 0 \
	"$(status_of "$result")"
assert_contains "the fetch says what it kept and why" "$(output_of "$result")" \
	"keeping the fetch for its console"
assert_contains "and the console comes back with it" "$(calls)" ".console/"
assert_contains "and the analyzer reads it" "$(calls)" "//cogs:speech_run_report"
RSYNC_OLOG=full
SSH_RUN_REACHED=no
SSH_RUN_STATUS=124

# The chain's refusals are one copy for both modes, parameterized on the
# launcher config this mode starts and the flag that fetches its records. A
# supervised session cannot be repeated, so a message sending its operator to
# the harness config or to `--fetch` is a message about the wrong run.
SSH_RUN_STATUS=8
result=$(deploy_tty unit --speech "${work}/speech-nolaunchconfig")
assert_status "a unit whose payload has no production config refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the production config, not the harness twin" \
	"$(output_of "$result")" "has no robotcpu.textproto"
assert_lacks "and not the harness one" "$(output_of "$result")" "robotcpu_harness.textproto"
assert_contains "it says the config rides the payload" "$(output_of "$result")" \
	"The production config is pushed with the payload"
assert_contains "and recovers the previous run's records with this mode's flag" \
	"$(output_of "$result")" "--speech-fetch <records-dir>"

SSH_RUN_STATUS=6
result=$(deploy_tty unit --speech "${work}/speech-stampunstaged")
assert_status "a stamp that could not be staged refuses" 1 "$(status_of "$result")"
assert_contains "and says nothing was emptied" "$(output_of "$result")" \
	"nothing was emptied"
assert_contains "with the speech mode's own fetch flag" "$(output_of "$result")" \
	"--speech-fetch <records-dir>"
SSH_RUN_STATUS=124

# The log root a speech run empties is the same configured one, wiped as root on
# the unit: the shared guard is called from this mode too, and a call site that
# was dropped or reordered below the chain leaves the extracted function green
# while the supervised mode wipes /etc.
sed -i 's|log_root_dir: "/run/brenn-app/logs/testing"|log_root_dir: "/etc"|' \
	-- "$logger_config"
result=$(deploy_tty unit --speech "${work}/speech-outside")
assert_status "a speech run whose log root is outside the payload store refuses" 1 \
	"$(status_of "$result")"
assert_contains "the refusal names the value and where it has to be" \
	"$(output_of "$result")" "which is not under /run/brenn-app"
assert_lacks "and nothing was emptied on the unit" "$(calls)" "rm -rf"
stage_logger_config

# A trailing slash on a configured endpoint comes off before the probe path is
# appended: `//v1/models` is a 404, and a 404 read as an unreachable robot would
# refuse a run against a healthy service.
cat >"${payload}/host/speech.toml" <<'TOML'
listen_addr = "127.0.0.1:7380"
pod_psk_file = "secrets/pod-psk.toml"

[stt]
url = "http://speaches.example:8000/"
TOML
result=$(deploy_tty unit --speech "${work}/speech-slash")
assert_status "an endpoint written with a trailing slash runs" 0 "$(status_of "$result")"
assert_contains "and the unit is asked for one slash, not two" "$(calls)" \
	"http://speaches.example:8000/v1/models"
assert_lacks "so no doubled slash reaches the probe" "$(calls)" \
	"http://speaches.example:8000//v1/models"

# A configuration naming no speech services is asked nothing about them: a
# preflight over tables that are not there would refuse a pipeline whose shape
# is simply different.
cat >"${payload}/host/speech.toml" <<'TOML'
listen_addr = "127.0.0.1:7380"
pod_psk_file = "secrets/pod-psk.toml"
TOML
result=$(deploy_tty unit --speech "${work}/speech-noservices")
assert_status "a configuration naming no services runs" 0 "$(status_of "$result")"
assert_lacks "and the unit is asked to reach nothing" "$(calls)" "curl"
assert_contains "while the link credentials are still asked about" "$(calls)" \
	"[ -s /run/brenn-app/conf/audio.conf ]"

# A configuration whose endpoint is not one this can pass on: the value is
# pasted into a command run on the unit as root, so it is refused where it is
# read.
cat >"${payload}/host/speech.toml" <<'TOML'
[stt]
url = "http://speaches.example:8000; rm -rf /"
TOML
result=$(deploy_tty unit --speech "${work}/speech-injected")
assert_status "an endpoint carrying a command refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the table and the value" "$(output_of "$result")" \
	"[stt] url in ${payload}/host/speech.toml"
assert_lacks "and nothing was started with it" "$(calls)" "simplelaunch"
stage_speech_config

# A records directory named relatively, which is what the Makefile passes, for
# the reason `--run`'s is absolutized: the report runs out of its own runfiles
# tree.
rel_speech=$(basename -- "${work}")/speech-relative
result=$(cd -- "$(dirname -- "${work}")" && deploy_tty unit --speech "$rel_speech")
assert_status "a relative records directory runs" 0 "$(status_of "$result")"
assert_contains "and the analyzer is handed an absolute path" "$(calls)" \
	"//cogs:speech_run_report ${work}/speech-relative/speech-log-"

# The fetch on its own: the terminal that died mid-run, or a report wanted a
# second time. It needs no terminal, because it starts nothing.
speech_fetch_dest="${work}/speech-fetched"
result=$(deploy unit --speech-fetch "$speech_fetch_dest")
assert_status "a speech fetch succeeds" 0 "$(status_of "$result")"
assert_contains "it reads the log root out of the configuration" "$(calls)" \
	"root@unit:/run/brenn-app/logs/testing/"
assert_contains "and names the analyzer of a speech run" "$(output_of "$result")" \
	"speech_run_report -- ${speech_fetch_dest}/speech-log-"
assert_eq "and lands under the speech name" 1 \
	"$(find "$speech_fetch_dest" -mindepth 1 -maxdepth 1 -type d -name 'speech-log-*' \
		! -name '*.console' ! -name '*.audio' | wc -l)"
# The whole composed command, not its pieces: the comparison is only meaningful
# against the configuration the run was recorded under, and against the clips
# beside the records just fetched.
speech_fetched=$(find "$speech_fetch_dest" -mindepth 1 -maxdepth 1 -type d \
	-name 'speech-log-*' ! -name '*.console' ! -name '*.audio')
assert_contains "and the comparison over the report's clips after it" \
	"$(output_of "$result")" \
	"stt_compare -- --speech-config ${payload}/host/speech.toml ${speech_fetched}.turns"
# The fetch on its own brings the audio too: a report wanted a second time is
# read beside the clips it names, and this is the mode an operator reaches for
# when the run's own terminal died.
assert_eq "and the audio came with it" 1 \
	"$(find "$speech_fetch_dest" -mindepth 1 -maxdepth 1 -type d -name '*.audio' | wc -l)"

# ---------------------------------------------------------------------------
# The grammar
# ---------------------------------------------------------------------------

result=$(deploy unit)
assert_status "a host with no mode refuses" 1 "$(status_of "$result")"
assert_contains "the refusal is the usage" "$(output_of "$result")" "usage:"

result=$(deploy unit --wat)
assert_status "a mode the script does not have refuses" 1 "$(status_of "$result")"

result=$(deploy unit --fetch)
assert_status "a fetch with no destination refuses" 1 "$(status_of "$result")"

result=$(deploy unit --run)
assert_status "a run with no records directory refuses" 1 "$(status_of "$result")"
assert_lacks "and starts nothing" "$(calls)" "simplelaunch"

result=$(deploy unit --run "${work}/run-extra" --wat)
assert_status "an extra argument after --run refuses" 1 "$(status_of "$result")"
assert_lacks "and starts nothing either" "$(calls)" "simplelaunch"

result=$(deploy unit --speech)
assert_status "a speech run with no records directory refuses" 1 "$(status_of "$result")"
assert_contains "the refusal is the usage" "$(output_of "$result")" "usage:"
assert_lacks "and starts nothing" "$(calls)" "simplelaunch"

result=$(deploy unit --speech "${work}/speech-extra" --wat)
assert_status "an extra argument after --speech refuses" 1 "$(status_of "$result")"
assert_lacks "and starts nothing either" "$(calls)" "simplelaunch"

result=$(deploy unit --speech-fetch)
assert_status "a speech fetch with no destination refuses" 1 "$(status_of "$result")"

result=$(deploy unit --commands)
assert_status "the mode that only printed commands is gone" 1 "$(status_of "$result")"
assert_contains "and the usage names the one that runs them" "$(output_of "$result")" \
	"--run <dir>"

# A misspelling of the one flag --push takes is not "check the freshness after
# all": an operator who thinks they overrode the refusal gets the usage line.
result=$(deploy unit --push --stale-okay)
assert_status "a misspelled --stale-ok refuses" 1 "$(status_of "$result")"
assert_contains "the refusal is the usage" "$(output_of "$result")" "usage:"
assert_lacks "and nothing is pushed" "$(calls)" "rsync"

result=$(deploy unit --push --stale-ok --wat)
assert_status "an extra argument after --stale-ok refuses" 1 "$(status_of "$result")"
assert_lacks "and that pushes nothing either" "$(calls)" "rsync"

# ---------------------------------------------------------------------------
# Every mode reads the staged configuration, so every mode wants a payload
# ---------------------------------------------------------------------------
#
# --push refuses earlier, at its own directory check. These two reach the
# configuration reader itself, whose refusal is the one that says to build. The
# direction that matters is silence: a reader that answered empty instead of
# refusing would make the unit's log directory, and later look for a run's
# records, at a path naming nothing.

rm -rf -- "$payload"
result=$(deploy unit --run "${work}/run-nopayload")
assert_status "a run with no payload refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the missing configuration" "$(output_of "$result")" \
	"no logger configuration at"
assert_contains "and says to build one" "$(output_of "$result")" "make motion-build"
assert_lacks "and reaches no device" "$(calls)" "ssh -t"

result=$(deploy unit --fetch "${work}/nowhere")
assert_status "fetching with no payload refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the missing configuration" "$(output_of "$result")" \
	"no logger configuration at"
assert_lacks "and reaches no device" "$(calls)" "rsync"
mkdir -p -- "${payload}/cogs" "${payload}/driver"
stage_payload "$after"

# ---------------------------------------------------------------------------
# The two names another repository reads
# ---------------------------------------------------------------------------
#
# Coupled to brenn-pod's deploy-reachy-pod.sh — see the note at `release` in
# the script under test. A rename here passes both repos' gates silently: the
# guard stops finding a robot, and the next pod deploy replaces the motion
# stack. These assertions are the tripwire.
assert_eq "the release directory keeps the name the pod deploy refuses on" \
	"release=\"\${store_mount}/releases/motion\"" \
	"$(grep '^release=' -- "${script_dir}/deploy-motion.sh")"
assert_eq "the stamp keeps the name the pod deploy refuses on" \
	"provenance_name=provenance.txt" \
	"$(grep '^provenance_name=' -- "${script_dir}/lib.sh")"
assert_eq "and only lib.sh spells it, so the build and the push cannot disagree" \
	"" \
	"$(grep '^provenance_name=' -- "${script_dir}/deploy-motion.sh" "${script_dir}/build-motion.sh" || true)"
assert_contains "and the constants say who else reads them" \
	"$(cat -- "${script_dir}/deploy-motion.sh")" \
	"deploy-reachy-pod.sh"

# ---------------------------------------------------------------------------
# The run budget against the wake lead it is mostly made of
# ---------------------------------------------------------------------------

assert_run_budget_covers_lead "${script_dir}/deploy-motion.sh"

# ---------------------------------------------------------------------------

tally
