#!/usr/bin/env bash
#
# tools/host-motion-run.test.sh — self-check for host-motion-run.sh.
#
# The subject is the sole executor of the host online run, and its verdict is
# what says the composition, the configs, the launcher and the log format agree.
# A green run of it is persuasive out of proportion to what it covers, so what
# has to hold is that a run which proves nothing still *refuses*: a launcher that
# died early, a logger that wrote no run directory, a run directory holding a
# zero-length log, a leftover process from the last run. Each of those is a
# refusal here, and so is the staged layout the launcher resolves every path in
# its config against.
#
# Everything the subject shells out to is stubbed: one bazel answering build,
# cquery and run; a launcher that writes what a case tells it to and then waits
# for the stop; pgrep; and sleep, so the run budget costs nothing however long
# it is. The subject is copied into a temporary tree beside its own lib.sh, so
# `repo_root` is that tree and every file it stages is a file this test made.
# Nothing here builds anything, reaches a network, or touches this checkout
# beyond reading the two shipped numbers the last case checks against each
# other.
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
mkdir -p -- "${repo}/tools" "${repo}/cogs"
cp -- "${script_dir}/host-motion-run.sh" "${script_dir}/lib.sh" "${repo}/tools/"

subject="${repo}/tools/host-motion-run.sh"
staging="${repo}/target/motion-host/run"
logs="${staging}/logs"

# The configuration files the host system reads. The subject does not name them:
# it asks Bazel, and the stub answers with this list, so a cog that gains a
# config file reaches the staging with no edit here.
config_files=(
	cogs/clip_library.textproto
	cogs/host_logger.textproto
	cogs/mover_params.textproto
	cogs/session_params.textproto
	cogs/wake_params.textproto
)
for file in "${config_files[@]}"; do
	echo "# ${file}" >"${repo}/${file}"
done
export CONFIG_FILES="${config_files[*]}"

# The one staged file the subject edits, and the one it reads before it stages
# anything. Written the way the real one is — a decoy in a comment, one field
# indented — so the line-oriented reads have something to get wrong.
logger_config="${repo}/cogs/host_logger.textproto"

stage_logger_config() {
	cat >"$logger_config" <<'CONFIG'
# a decoy in a comment: pinion_namespace: "motion"
log_root_dir: "/tmp/a-hand-started-run"
  pinion_shm_root: "/dev/shm"
pinion_namespace: ""
CONFIG
}
stage_logger_config

# Bazel's outputs, in a directory the stub answers with workspace-relative paths
# into — the shape the subject resolves through the convenience symlink.
outs="${repo}/bazel-out/bin"
mkdir -p -- "$outs"

generated_files=(
	brenn_reachy.cogs.system_host.motion_host.logger_proc.tachyon
	brenn_reachy.cogs.system_host.motion_host.plant_proc.tachyon
	brenn_reachy.cogs.system_host.motion_host.proc.tachyon
	system_host.motion_host.HostCpu_event_logger_config.tachyon
)

# What the launcher stub does with the run: write a log the analyzer could read,
# write one of zero length, write no run directory at all, or exit before the
# run budget is up.
export LAUNCHER_WRITES=full
export LAUNCHER_EXITS_EARLY=""
# What pgrep says is already running, and what the analyzer says about the log.
export LEFTOVER=""
export REPORT_STATUS=0
export CALLS="${work}/calls"

stubs="${work}/bin"
mkdir -p -- "$stubs"
PATH="${stubs}:${PATH}"
export PATH

real_install=$(command -v install) || {
	echo "no install on PATH; the staging cases cannot run" >&2
	exit 1
}
export REAL_INSTALL="$real_install"

# One stub, three subcommands. `build` writes the outputs; `cquery` answers one
# of the subject's two questions by which one the set expression names; `run` is
# the analyzer, whose status the subject's own exit code is.
cat >"${stubs}/bazel" <<STUB
#!/usr/bin/env bash
printf 'bazel %s\n' "\$*" >>"\$CALLS"
sub=\$1
shift
target=""
for arg in "\$@"; do
	case "\$arg" in
		//*) target=\$arg ;;
	esac
done
case "\$sub" in
	build)
		cp -- "${stubs}/launcher" bazel-out/bin/simplelaunch
		printf '#!/bin/bash\n' >bazel-out/bin/clockwork_prelaunch.sh
		printf '#!/bin/bash\n' >bazel-out/bin/robot_host_clk_exe
		chmod 0755 -- bazel-out/bin/robot_host_clk_exe \\
			bazel-out/bin/clockwork_prelaunch.sh bazel-out/bin/simplelaunch
		echo 'app { name: "proc" }' >bazel-out/bin/hostcpu.textproto
		for f in ${generated_files[*]}; do
			echo "generated \$f" >"bazel-out/bin/\$f"
		done
		;;
	cquery)
		case "\$target" in
			*host_config_files*)
				for f in \$CONFIG_FILES; do
					echo "\$f"
				done
				;;
			*)
				echo bazel-out/bin/robot_host_clk_exe
				echo bazel-out/bin/simplelaunch
				echo bazel-out/bin/hostcpu.textproto
				echo bazel-out/bin/clockwork_prelaunch.sh
				echo cogs/robot_host.clk
				for f in bazel-out/bin/*.tachyon; do
					echo "\$f"
				done
				;;
		esac
		;;
	run)
		echo "the wake gesture happened, whole"
		exit "\$REPORT_STATUS"
		;;
	*) echo "unstubbed subcommand \$sub" >&2; exit 1 ;;
esac
exit 0
STUB
chmod 0755 -- "${stubs}/bazel"

# The launcher. It runs from the staging root, as the real one does, and it finds
# the log root the way the real logger process does: out of the staged
# configuration. So a staging that failed to rewrite that line is a launcher
# writing somewhere else, which is the failure the subject's own `grep` guard is
# for.
#
# Python rather than shell, for the reason that matters at a bench too: a
# background child of a non-interactive shell inherits SIGINT ignored, and a
# shell script cannot trap a signal that arrived ignored. A real launcher
# installs its own handler and so overrides that, which is what the stop gesture
# depends on -- so the stub installs one, and a stub that did not would hang
# here instead of stopping.
cat >"${stubs}/launcher" <<'STUB'
#!/usr/bin/env python3
"""Stand in for simplelaunch: write what the case asked for, then wait."""

import os
import re
import signal
import sys
import time

with open(os.environ["CALLS"], "a", encoding="utf-8") as calls:
    calls.write("launcher " + " ".join(sys.argv[1:]) + "\n")

if os.environ.get("LAUNCHER_EXITS_EARLY"):
    sys.exit(0)

with open("cogs/host_logger.textproto", encoding="utf-8") as config:
    root = re.search(r'^log_root_dir: "(.*)"$', config.read(), re.M).group(1)

writes = os.environ.get("LAUNCHER_WRITES", "full")
if writes in ("full", "empty"):
    run = os.path.join(root, "run_000001")
    os.makedirs(run, exist_ok=True)
    with open(os.path.join(run, "robot_000000.olog"), "w", encoding="utf-8") as log:
        if writes == "full":
            log.write("an olog\n")

stopped = False


def stop(_signum, _frame):
    global stopped
    stopped = True


signal.signal(signal.SIGINT, stop)
signal.signal(signal.SIGTERM, stop)
while not stopped:
    time.sleep(0.01)
sys.exit(0)
STUB
chmod 0755 -- "${stubs}/launcher"

# pgrep, answering what a case says is running. Exit 1 with nothing found, which
# is what the real one does and what the subject's `|| true` rests on.
cat >"${stubs}/pgrep" <<'STUB'
#!/usr/bin/env bash
name=${*: -1}
if [ -n "${LEFTOVER:-}" ] && [ "$name" = "$LEFTOVER" ]; then
	echo 4242
	exit 0
fi
exit 1
STUB
chmod 0755 -- "${stubs}/pgrep"

# Sleep, instantly: the subject's run budget is a count of one-second waits and
# the launcher's own idle loop is more of them. Nothing here is timing-dependent
# — the subject stops the launcher by signal, not by clock — so a wait that
# returns at once only makes the suite fast.
cat >"${stubs}/sleep" <<'STUB'
#!/usr/bin/env bash
exit 0
STUB
chmod 0755 -- "${stubs}/sleep"

cat >"${stubs}/install" <<'STUB'
#!/usr/bin/env bash
exec "$REAL_INSTALL" "$@"
STUB
chmod 0755 -- "${stubs}/install"

host_run() {
	: >"$CALLS"
	local out status=0
	out=$(cd -- "$repo" && "$subject" 2>&1) || status=$?
	printf '%s\n---status %s\n' "$out" "$status"
}

calls() { cat -- "$CALLS"; }

# ---------------------------------------------------------------------------
# A run that works: the staged layout and the analyzer's verdict
# ---------------------------------------------------------------------------

result=$(host_run)
assert_status "a clean host run succeeds" 0 "$(status_of "$result")"

# The layout is not this script's choice: the launcher resolves every executable
# and argument in its config against its working directory, and the three
# processes read their configuration by paths relative to the same place.
assert_file "the launcher is at the root it is started from" "${staging}/simplelaunch"
assert_file "and its config beside it" "${staging}/hostcpu.textproto"
assert_file "the executable is where the config's apps name it" \
	"${staging}/cogs/robot_host_clk_exe"
assert_no_file "and not at the staging root" "${staging}/robot_host_clk_exe"
assert_file "the prelaunch script is at the path its config names" \
	"${staging}/clockwork/launch/clockwork_prelaunch.sh"
for file in "${config_files[@]}"; do
	assert_file "the run carries ${file} at that path" "${staging}/${file}"
done
assert_file "the writer's channel set is staged under cogs/" \
	"${staging}/cogs/system_host.motion_host.HostCpu_event_logger_config.tachyon"
assert_file "the plant's process description is staged" \
	"${staging}/cogs/brenn_reachy.cogs.system_host.motion_host.plant_proc.tachyon"
assert_no_file "a composition source in the listing is not staged" \
	"${staging}/cogs/robot_host.clk"

# The one edit to a staged file, in both directions: the staged copy states one
# log root and it points into the scratch tree, and the tree's own copy is
# exactly as it was.
assert_eq "the staged logger config states one log root, in the scratch tree" \
	"log_root_dir: \"${logs}\"" \
	"$(grep '^log_root_dir:' -- "${staging}/cogs/host_logger.textproto")"
assert_contains "and the tree's own copy is untouched" \
	"$(cat -- "$logger_config")" 'log_root_dir: "/tmp/a-hand-started-run"'

assert_contains "the analyzer is run over the log the writer wrote" "$(calls)" \
	"run -- //cogs:first_motion_report --grid-jitter-ns"
assert_contains "and the run directory it names is the one under the scratch log root" \
	"$(calls)" "${logs}/run_000001"
assert_contains "the launcher is started on a probed control port" "$(calls)" \
	"launcher hostcpu.textproto --logdir"
assert_contains "the run says where the log is" "$(output_of "$result")" \
	"${logs}/run_000001"

# The analyzer's verdict is the run's verdict: a green harness over a log the
# report failed would be the whole exercise saying nothing.
REPORT_STATUS=3
result=$(host_run)
assert_status "a report that fails fails the run" 3 "$(status_of "$result")"
REPORT_STATUS=0

# ---------------------------------------------------------------------------
# The refusals
# ---------------------------------------------------------------------------

# A leftover process. Two runs here would share one shared-memory layout and one
# set of loopback ports, so the second one refuses before it stages anything —
# and it refuses for the previous run's launcher too, which the real launcher's
# own basename check does not cover.
rm -rf -- "${repo}/target"
LEFTOVER=simplelaunch
result=$(host_run)
assert_status "a leftover launcher refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names it and what to do" "$(output_of "$result")" \
	"pkill -x simplelaunch"
assert_no_file "and nothing was staged" "$staging"

LEFTOVER=robot_host_clk_exe
result=$(host_run)
assert_status "a leftover cog process refuses too" 1 "$(status_of "$result")"
assert_contains "and names that one" "$(output_of "$result")" \
	"pkill -x robot_host_clk_exe"
LEFTOVER=""

# A logger that wrote nothing at all. This is the namespace-mismatch failure the
# whole exercise exists to catch: every process ran, the gesture happened, and
# the log root is empty.
LAUNCHER_WRITES=none
result=$(host_run)
assert_status "a run with no log directory refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the log root" "$(output_of "$result")" \
	"wrote no run directory under ${logs}"
assert_lacks "and the analyzer was never run over nothing" "$(calls)" \
	"run -- //cogs:first_motion_report"

# And the same failure one step further along: a run directory with a log file of
# zero length in it, which is what an `O_DIRECT` open that failed or a writer
# that found no buffers leaves behind.
LAUNCHER_WRITES=empty
result=$(host_run)
assert_status "a zero-length log refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says what it looked for" "$(output_of "$result")" \
	"holds no non-empty .olog file"
assert_contains "and names the disagreement it is probably about" \
	"$(output_of "$result")" "pinion namespace"
assert_lacks "and the analyzer was never run" "$(calls)" \
	"run -- //cogs:first_motion_report"
LAUNCHER_WRITES=full

# A launcher that exited before the gesture was over: three processes that could
# not start, a config naming a path that is not there. The refusal points at the
# console the launcher wrote.
LAUNCHER_EXITS_EARLY=1
result=$(host_run)
assert_status "a launcher that exits early refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says it went before the run was over" \
	"$(output_of "$result")" "before the run was over"
assert_lacks "and the analyzer was never run" "$(calls)" \
	"run -- //cogs:first_motion_report"
LAUNCHER_EXITS_EARLY=""

# The pinion agreement, read out of the tree before anything is built. Its own
# wording is pinned in tools/lib.test.sh; what this owns is that the refusal
# stops this script before it stages or builds.
rm -rf -- "${repo}/target"
sed -i 's|^pinion_namespace: .*|pinion_namespace: "motion"|' -- "$logger_config"
result=$(host_run)
assert_status "a logger configuration with a namespace refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names the file" "$(output_of "$result")" \
	"cogs/host_logger.textproto"
assert_no_file "and nothing was staged" "$staging"
assert_lacks "and nothing was built" "$(calls)" "bazel build"
stage_logger_config

# A staged configuration with no log root line to rewrite. The rewrite is a `sed`
# over a line that has to be there, so the guard behind it is what keeps a
# renamed field from being a run that logs where the checked-in file says.
rm -rf -- "${repo}/target"
sed -i '/^log_root_dir:/d' -- "$logger_config"
result=$(host_run)
assert_status "a configuration with no log root refuses" 1 "$(status_of "$result")"
assert_contains "the refusal says the line was not there" "$(output_of "$result")" \
	"states no log_root_dir line to rewrite"
assert_lacks "and the launcher was never started" "$(calls)" "launcher hostcpu"
stage_logger_config

# A configuration file Bazel names and the tree does not have: staged by path, so
# the refusal is the one a composition's rename produces.
rm -rf -- "${repo}/target"
CONFIG_FILES="${config_files[*]} cogs/absent_params.textproto"
result=$(host_run)
assert_status "a configuration file the tree lost refuses" 1 "$(status_of "$result")"
assert_contains "the refusal names it" "$(output_of "$result")" \
	"wants cogs/absent_params.textproto"
CONFIG_FILES="${config_files[*]}"

result=$(host_run)
assert_status "and the shipped set runs again" 0 "$(status_of "$result")"

# ---------------------------------------------------------------------------
# The run budget against the wake lead it is mostly made of
# ---------------------------------------------------------------------------

# The run budget is a hand-maintained sum whose largest term — the shipped wake
# lead — lives in another file and language. A lead that grows without the
# budget following is red here rather than a `POST /quit` mid-gesture.
#
# Besides the lead: commissioning's bus survey (~5 s), the raise-hold-stow
# gesture (~5 s), and the release (~4 s) — fourteen seconds, deliberately
# without the subject's margin (which is for a loaded workstation, not this sum).
checkout=$(cd -- "${script_dir}/.." && pwd)
shipped_lead_ms=$(sed -n 's/^lead_ms:[[:space:]]*\([0-9]*\).*/\1/p' \
	-- "${checkout}/cogs/wake_params.textproto")
shipped_budget=$(sed -n 's/^run_seconds=\([0-9]*\).*/\1/p' \
	-- "${checkout}/tools/host-motion-run.sh")
phases_seconds=14

if [ -n "$shipped_lead_ms" ] && [ -n "$shipped_budget" ]; then
	needed=$((shipped_lead_ms / 1000 + phases_seconds))
	if [ "$shipped_budget" -ge "$needed" ]; then
		pass "the run budget covers the shipped wake lead and the phases around it"
	else
		fail "the run budget covers the shipped wake lead and the phases around it" \
			"the budget is ${shipped_budget} s" \
			"the shipped wake lead is ${shipped_lead_ms} ms, and commissioning, the" \
			"gesture and the release want ${phases_seconds} s around it: ${needed} s" \
			"the launcher would be stopped mid-gesture"
	fi
else
	fail "the run budget covers the shipped wake lead and the phases around it" \
		"read no lead_ms from cogs/wake_params.textproto or no run_seconds from" \
		"tools/host-motion-run.sh — one of the two names has moved"
fi

tally
