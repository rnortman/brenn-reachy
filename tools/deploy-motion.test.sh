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
# What is worth pinning: the refusals, and the two numbers this script must not
# retype. The refusals are a stale payload and a held servo bus — a refusal that
# quietly stops refusing is discovered on the night it should have saved. The two
# numbers are the pinion namespace and the shm root: both are stated once in
# `cogs/robot_logger.textproto`, both have to appear in the commands that start
# the two processes, and a mismatch is silent in the worst direction — the logger
# comes up, finds no channels under its namespace, and writes an empty log while
# the gesture runs perfectly.
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
# not load-bearing. These are the values every command below has to carry, and
# they are deliberately not the shipped ones: a subject that hardcoded the real
# ones would pass a test written against them.
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
  pinion_shm_root: "/dev/shm-under-test"
pinion_namespace: "undertest"
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

stage_payload() {
	local when=$1
	local name
	for name in reachy_motord robot_clk_exe; do
		: >"${payload}/${name}"
		chmod 0755 -- "${payload}/${name}"
	done
	: >"${payload}/cogs/session_params.textproto"
	stage_logger_config
	touch -d "@${when}" -- "${payload}/robot_clk_exe"
}

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

export CALLS="${work}/calls"
export GIT_COMMIT_TIME=""
export SSH_PREPARE_STATUS=0
export RSYNC_STATUS=0

# Every stub records its whole invocation on one line, so a case can assert both
# that a command ran and that it did not.
cat >"${stubs}/ssh" <<'STUB'
#!/usr/bin/env bash
printf 'ssh %s\n' "$*" >>"$CALLS"
exit "${SSH_PREPARE_STATUS:-0}"
STUB

cat >"${stubs}/rsync" <<'STUB'
#!/usr/bin/env bash
printf 'rsync %s\n' "$*" >>"$CALLS"
exit "${RSYNC_STATUS:-0}"
STUB

# The repository question the freshness check asks. An empty GIT_COMMIT_TIME is a
# tree whose history says nothing about these paths.
cat >"${stubs}/git" <<'STUB'
#!/usr/bin/env bash
printf 'git %s\n' "$*" >>"$CALLS"
[ -n "${GIT_COMMIT_TIME:-}" ] || exit 0
echo "$GIT_COMMIT_TIME"
STUB

chmod 0755 -- "${stubs}/ssh" "${stubs}/rsync" "${stubs}/git"

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
# The commands that start the run
# ---------------------------------------------------------------------------

result=$(deploy unit --commands)
printed=$(output_of "$result")
assert_status "the commands print" 0 "$(status_of "$result")"
assert_contains "the driver is started from the release directory" "$printed" \
	"cd /run/brenn-app/releases/motion"
assert_contains "the driver comes first" "$printed" "./reachy_motord"
assert_contains "the logger is the executable plus its own description" "$printed" \
	"./robot_clk_exe cogs/brenn_reachy.cogs.system_robot.motion_robot.logger_proc.tachyon --pinion-dir /dev/shm-under-test --pinion-ns undertest"
assert_contains "the control loop is the same executable plus its own" "$printed" \
	"./robot_clk_exe cogs/brenn_reachy.cogs.system_robot.motion_robot.proc.tachyon --pinion-dir /dev/shm-under-test --pinion-ns undertest"
assert_lacks "neither process asks for the deterministic runner" "$printed" \
	"--deterministic-runner"
assert_lacks "the namespace is read, not the shipped one retyped" "$printed" '--pinion-ns motion'
assert_lacks "and read out of the payload, not out of the tree" "$printed" "fromthetree"
assert_lacks "printing commands reaches no device" "$(calls)" "ssh"

# A configuration that lost the field the commands need is a refusal, not a
# command with an empty flag.
sed -i '/^pinion_namespace:/d' -- "$logger_config"
result=$(deploy unit --commands)
assert_status "a configuration with no namespace refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the field" "$(output_of "$result")" \
	"states no pinion_namespace"
stage_logger_config

# A field that is there and says nothing is its own refusal: to every caller it
# is the same as a missing field, but it is a different edit to make.
sed -i 's|^pinion_namespace: .*|pinion_namespace: ""|' -- "$logger_config"
result=$(deploy unit --commands)
assert_status "a configuration with an empty namespace refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says it is empty" "$(output_of "$result")" \
	"states an empty pinion_namespace"
stage_logger_config

# ---------------------------------------------------------------------------
# Values that mean something to a shell
# ---------------------------------------------------------------------------
#
# These three values are pasted into a remote command run as root, a remote rsync
# path the far end re-parses, and a local command a person copies. A value with a
# space in it means a different thing at each of those sites, so it is refused
# where it is read rather than quoted differently three times.

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

sed -i 's|^  pinion_shm_root: .*|  pinion_shm_root: "/dev/shm; rm -rf /"|' -- "$logger_config"
result=$(deploy unit --commands)
assert_status "a shm root carrying a command refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names that field" "$(output_of "$result")" \
	"pinion_shm_root is '/dev/shm; rm -rf /'"
stage_logger_config

# The shipped shapes are accepted, so the check above is a check on values and
# not a ban on paths.
result=$(deploy unit --commands)
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
# refusing would print start commands with empty --pinion-dir/--pinion-ns flags,
# and the logger would come up, find no channels and write an empty log while the
# gesture ran perfectly.

rm -rf -- "$payload"
result=$(deploy unit --commands)
assert_status "printing commands with no payload refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the missing configuration" "$(output_of "$result")" \
	"no logger configuration at"
assert_contains "and says to build one" "$(output_of "$result")" "make motion-build"

result=$(deploy unit --fetch "${work}/nowhere")
assert_status "fetching with no payload refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the missing configuration" "$(output_of "$result")" \
	"no logger configuration at"
assert_lacks "and reaches no device" "$(calls)" "rsync"
mkdir -p -- "${payload}/cogs" "${payload}/driver"
stage_payload "$after"

# ---------------------------------------------------------------------------

tally
