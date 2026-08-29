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
	rm -f -- "${payload}/build-commit.txt"
	[ -z "$BUILD_COMMIT" ] ||
		echo "commit=${BUILD_COMMIT}" >"${payload}/build-commit.txt"
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
export RSYNC_STATUS=0
export RSYNC_OLOG=full
export RSYNC_CONSOLE=full
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
	*" -t "*) exit "${SSH_RUN_STATUS:-124}" ;;
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
cat >"${stubs}/bazel" <<'STUB'
#!/usr/bin/env bash
printf 'bazel %s\n' "$*" >>"$CALLS"
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
	"systemctl is-active --quiet brenn-app.service && exit 3; systemctl is-active --quiet reachy-motiond.service && exit 4; [ -f /run/brenn-app/releases/motion/provenance.txt ] || exit 5; cp -- /run/brenn-app/releases/motion/provenance.txt /run/brenn-app/motion-provenance.staged || exit 6; rm -rf -- /run/brenn-app/logs/testing && mkdir -p -- /run/brenn-app/logs/testing || exit 7; mv -- /run/brenn-app/motion-provenance.staged /run/brenn-app/logs/testing/provenance.txt || exit 7; rm -rf -- /run/brenn-app/logs/launch && mkdir -p -- /run/brenn-app/logs/launch || exit 7; cd /run/brenn-app/releases/motion || exit 7; timeout --signal=INT --kill-after=10 30 ./simplelaunch robotcpu.textproto --logdir /run/brenn-app/logs/launch; exit \$?"
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

# A code the run path does not read at all, which is what the catch-all arm is
# for.
SSH_RUN_STATUS=8
result=$(deploy unit --run "${work}/run-failed")
assert_status "any other status refuses" 1 "$(status_of "$result")"
assert_contains "and carries the code" "$(output_of "$result")" "exit 8"
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
