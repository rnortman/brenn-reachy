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
stage_payload() {
	local when=$1
	local name
	for name in reachy_motord cogs/robot_clk_exe simplelaunch; do
		: >"${payload}/${name}"
		chmod 0755 -- "${payload}/${name}"
	done
	: >"${payload}/robotcpu.textproto"
	: >"${payload}/cogs/session_params.textproto"
	stage_logger_config
	touch -d "@${when}" -- "${payload}/cogs/robot_clk_exe"
}

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

export CALLS="${work}/calls"
export GIT_COMMIT_TIME=""
export SSH_PREPARE_STATUS=0
export SSH_RUN_STATUS=124
export RSYNC_STATUS=0
export RSYNC_OLOG=full

# Every stub records its whole invocation on one line, so a case can assert both
# that a command ran and that it did not.
cat >"${stubs}/ssh" <<'STUB'
#!/usr/bin/env bash
printf 'ssh %s\n' "$*" >>"$CALLS"
# A run's ssh is the one carrying a pty, and its status is the launcher's rather
# than the probe's, so the two are separate knobs: a case can hold the bus for a
# push and answer 124 for a run.
case " $* " in
	*" -t "*) exit "${SSH_RUN_STATUS:-124}" ;;
esac
exit "${SSH_PREPARE_STATUS:-0}"
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
	case "$dest" in *@*:*) exit 0 ;; esac
	case "${RSYNC_OLOG:-full}" in
		full)
			mkdir -p -- "${dest}/run-20260825T120000Z"
			echo records >"${dest}/run-20260825T120000Z/motion_0.olog"
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

# The repository question the freshness check asks. An empty GIT_COMMIT_TIME is a
# tree whose history says nothing about these paths.
cat >"${stubs}/git" <<'STUB'
#!/usr/bin/env bash
printf 'git %s\n' "$*" >>"$CALLS"
[ -n "${GIT_COMMIT_TIME:-}" ] || exit 0
echo "$GIT_COMMIT_TIME"
STUB

# The analyzer, whose verdict a run's verdict is. It reports its argument so a
# case can pin which run directory was judged, and its status is the knob that
# says whether the log passed.
cat >"${stubs}/bazel" <<'STUB'
#!/usr/bin/env bash
printf 'bazel %s\n' "$*" >>"$CALLS"
exit "${BAZEL_STATUS:-0}"
STUB

chmod 0755 -- "${stubs}/ssh" "${stubs}/rsync" "${stubs}/git" "${stubs}/bazel"
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
# The run
# ---------------------------------------------------------------------------

run_dest="${work}/run-records"
result=$(deploy unit --run "$run_dest")
ran=$(calls)
assert_status "a budgeted run that the analyzer passes succeeds" 0 "$(status_of "$result")"
assert_contains "the bus question, the log root's clear and the launcher are one invocation" "$ran" \
	"systemctl is-active --quiet brenn-app.service && exit 3; systemctl is-active --quiet reachy-motiond.service && exit 4; rm -rf -- /run/brenn-app/logs/testing && mkdir -p -- /run/brenn-app/logs/testing || exit 1; cd /run/brenn-app/releases/motion || exit 1; timeout --signal=INT --kill-after=10 30 ./simplelaunch robotcpu.textproto --logdir /run/brenn-app/logs/launch; exit \$?"
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

SSH_RUN_STATUS=7
result=$(deploy unit --run "${work}/run-failed")
assert_status "any other status refuses" 1 "$(status_of "$result")"
assert_contains "and carries the code" "$(output_of "$result")" "exit 7"
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
fetched=$(find "$dest" -mindepth 1 -maxdepth 1 -type d | wc -l)
if [ "$fetched" = 1 ]; then
	pass "the fetch lands in one stamped directory"
else
	fail "the fetch lands in one stamped directory" "found ${fetched}"
fi
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
# this second and the next few: the second can tick between this line and the
# subject's read, and a self-check that fails once an hour is a self-check
# nobody believes.
collide="${work}/collide"
occupy() {
	local dir=$1 suffix=$2 ahead
	for ahead in 0 1 2 3; do
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
# The run budget against the wake lead it is mostly made of
# ---------------------------------------------------------------------------

assert_run_budget_covers_lead "${script_dir}/deploy-motion.sh"

# ---------------------------------------------------------------------------

tally
