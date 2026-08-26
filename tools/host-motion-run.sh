#!/usr/bin/env bash
#
# Run the online motion system on this workstation, against the simulated plant.
#
#   tools/host-motion-run.sh          (or: make motion-host-run)
#
# The online system — the control process, the logger process, the process
# descriptions, the shared-memory namespace, the launcher config and the log
# writer's own open — has met a powered unit. This script is the workstation
# rehearsal of it and the gate before a hardware run: cheap, deterministic, and
# runnable without the machine. It stages the host composition the way the device
# payload is staged, starts all three processes under the same launcher a unit
# uses, lets the wake gesture happen, stops them, and runs the same analyzer a
# hardware run is judged by over the log the real writer wrote.
#
# What passing means: the composition, the configs, the launcher, the seam's wire
# contract, the writer and the analyzer agree with each other. What it does not
# mean — stated here because a green run is persuasive out of proportion to what
# it covers — no serial timing, no `O_DIRECT` on the device's kernel, no aarch64,
# and nothing whatever about real servo behaviour. Those belong to the unit and
# to the bench's own self-tests.
#
# The far end of the seam is `cogs::MotorSim`, the plant ten deterministic
# scenarios run on, in a process of its own with UDP sockets where the scenarios
# wire channel to channel. So the datagrams are really encoded, sent, received
# and decoded, and the control box is `robot::RobotBox` with nothing done to it.
#
# Two facts about running it that are not this script's choice:
#
#   * The pinion buffers land in the real `/dev/shm` empty-namespace layout, not
#     in the scratch tree — every process here is flagless, exactly as on a unit,
#     and the launcher's pre_launch step clears that layout before starting
#     anything. So two runs on one workstation would collide, and so would a run
#     beside any other flagless Clockwork system. The launcher refuses the second
#     one by name (it will not start while a process matching a configured
#     executable's basename is alive), and this script refuses before that.
#   * The seam is six fixed loopback ports (7401-7406) plus the plant's unfed
#     injection port. Two runs collide there too.
#
# It is a manual target rather than a test: it is wall-clock-long and its
# flakiness envelope — spawn races between three unordered processes, host load —
# is unmeasured. TODO(host-run-in-ci)
#
# Knobs, environment only:
#
#   REACHY_BAZEL   the bazel to run (default bazel)

set -euo pipefail

# shellcheck source=lib.sh
. "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

bazel=${REACHY_BAZEL:-bazel}

# Every bazel invocation below runs from the workspace root, whatever directory
# the caller was in, and the cqueries resolve workspace-relative paths against
# it.
cd -- "$repo_root"

# The host configuration, which is the default one: no `--config=device` here,
# and that is the whole difference between this staging and the payload's. The
# array exists because lib.sh's cquery helpers take the configuration a build
# and its queries share, and an empty one is this build's answer.
build_flags=()

# What is built. The executable all three processes are started from, the system
# target that emits their process descriptions and the writer's channel set, the
# rendered launcher config, the launcher itself, the prelaunch script the config
# names, the configuration files the processes read, and the analyzer -- whose
# label is lib.sh's `report_target`, shared with the device harness.
exe_target=//cogs:robot_host_clk_exe
system_target=//cogs:system_host_clk
launcher_target=@clockwork//jewels/simplelaunch:simplelaunch
launch_config_target=//cogs:hostcpu.textproto
prelaunch_target=//cogs:clockwork_prelaunch_sh
config_target=//cogs:host_config_files

# The generated files a process reads, by basename: three process descriptions
# and the writer's channel set. Flattened into `cogs/`, which is where the
# launcher config's args say they are.
generated_files=(
	brenn_reachy.cogs.system_host.motion_host.logger_proc.tachyon
	brenn_reachy.cogs.system_host.motion_host.plant_proc.tachyon
	brenn_reachy.cogs.system_host.motion_host.proc.tachyon
	system_host.motion_host.HostCpu_event_logger_config.tachyon
)

# The scratch tree: the payload layout, on this machine. Rebuilt from scratch
# every run, so a file the layout no longer wants cannot sit there being started
# forever. Nothing under it is committed and nothing outside it is written.
scratch="${repo_root}/target/motion-host"
staging="${scratch}/run"
logs="${staging}/logs"
launch_logs="${scratch}/launch"

# The staged logger configuration, and the line this script owns in it. The
# checked-in file names a place a hand-started run can use; a run started here
# writes under the scratch tree, so the log root is rewritten in the staged copy
# — never in the tree's.
logger_config=cogs/host_logger.textproto

# What the analyzer is told to tolerate in the sample stream's own timestamps.
#
# On a unit the driver computes each cycle's instant from an absolute grid, so its
# samples land on exact multiples of the period and the analyzer's default of zero
# is the right reading. The plant here is not a driver: it is a cog woken by the
# wall-clock runner, and it stamps a sample with when it actually ran. That
# jitter is the runner's and this machine's load, not the system's, so the band is
# stated here rather than being built into the analyzer, which has a hardware log
# to be strict about.
#
# Sized from the runs there have been rather than from a guess, because the band
# is what the heartbeat check has left to catch a dropped, repeated or reordered
# sample with, and a band approaching a quarter of the period credits a sample
# that landed a whole phase out to a cycle. Measured over two 1250-sample runs on
# an idle workstation: one run inside 0.5 ms throughout, the other the same but
# for a single 2.65 ms excursion. Three milliseconds covers that worst one, and a
# run that exceeds it is either a workstation with something else on it or the
# transport fault this check exists to find -- either way a finding worth reading
# rather than one worth widening the band for. The envelope over a handful of runs
# is what TODO(host-run-in-ci) has to measure anyway.
grid_jitter_ns=3000000

# How long the gesture is given before the launcher is stopped. Commissioning is
# about five seconds of bus transactions, the wake lead is eight, the raise-hold-
# stow gesture about five, the release about four; twenty-seven leaves margin for
# a loaded workstation. Nothing about the run is judged by the clock — the
# analyzer reads the log — so the budget only has to be long enough. The lead is
# a number in another file, so the sum is checked against the shipped one by
# host-motion-run.test.sh: a lead that grows without this following it is red
# there rather than a launcher stopped mid-gesture.
run_seconds=27

# The launcher's HTTP control port. Probed rather than defaulted: 8080 is a port
# a workstation routinely has something on, and a launcher that cannot bind it
# would fail for a reason that has nothing to do with this run. The probe is
# advisory -- the port can be taken between the connect that refused and the
# launcher's own bind -- so what it buys is a better first guess, and the bind
# failure is still the authority.
pick_port() {
	local port
	for port in $(seq 18080 18099); do
		if ! (exec 3<>"/dev/tcp/127.0.0.1/${port}") 2>/dev/null; then
			echo "$port"
			return 0
		fi
	done
	die "no free port in 18080-18099 for the launcher's control socket."
}

compile() {
	"$bazel" build "${build_flags[@]}" -- \
		"$exe_target" "$system_target" "$launcher_target" \
		"$launch_config_target" "$prelaunch_target" "$config_target" \
		"$report_target"
}

# Build the scratch tree: the launcher and its config at the root, the
# executable and everything it reads under `cogs/`, the prelaunch script at the
# path the config names. The same layout `tools/build-motion.sh` stages for a
# unit, for the same reason — the rendered config spells every path relative to
# the launcher's working directory.
stage() {
	local exe=$1 launcher=$2 launch_config=$3 prelaunch=$4 configs=$5 built=$6
	local file src

	rm -rf -- "$staging"
	mkdir -p -- "$logs" "$launch_logs"

	install -m 0755 -D -- "$launcher" "${staging}/simplelaunch"
	install -m 0644 -D -- "$launch_config" "${staging}/hostcpu.textproto"
	install -m 0755 -D -- "$prelaunch" \
		"${staging}/clockwork/launch/clockwork_prelaunch.sh"
	install -m 0755 -D -- "$exe" "${staging}/cogs/robot_host_clk_exe"

	while IFS= read -r file; do
		[ -n "$file" ] || continue
		[ -f "${repo_root}/${file}" ] ||
			die "the run wants ${file} and the tree has no such file."
		install -m 0644 -D -- "${repo_root}/${file}" "${staging}/${file}"
	done <<<"$configs"

	for file in "${generated_files[@]}"; do
		src=$(bazel_named_in "$built" "$file") || exit 1
		install -m 0644 -D -- "$src" "${staging}/cogs/${file}"
	done

	# The one edit to a staged file: the log root, which is a directory this
	# script owns and creates. `sed` over the whole line rather than an append,
	# so a staged copy states one log root and not two.
	sed -i "s|^log_root_dir: .*\$|log_root_dir: \"${logs}\"|" \
		-- "${staging}/${logger_config}"
	grep -q "^log_root_dir: \"${logs}\"\$" -- "${staging}/${logger_config}" ||
		die "could not point ${logger_config} at ${logs}." \
			"The staged copy states no log_root_dir line to rewrite."
}

# Refuse rather than launch beside something. The launcher's own check is by
# basename over every configured executable and it is the authority; this one
# runs first so the refusal names what to do about it, and covers the previous
# run's launcher too, which the launcher itself does not.
refuse_leftovers() {
	local name pids
	for name in simplelaunch robot_host_clk_exe; do
		pids=$(pgrep -x -- "$name" 2>/dev/null || true)
		[ -z "$pids" ] || die \
			"${name} is already running here (pid ${pids//$'\n'/ }), and a second run cannot share this machine." \
			"Every process is flagless, so both runs would use the same shared-memory" \
			"layout, and both would bind the same six loopback ports. Stop it first:" \
			"    pkill -x ${name}"
	done
}

# Start the launcher, let the gesture happen, stop it the way an operator does.
# SIGINT to the launcher is what Ctrl-C is: it forwards SIGINT to all three
# children, the cog processes stop cleanly on it, and a unit's driver winds its
# torque down on it.
run() {
	local port=$1 pid
	(
		cd -- "$staging" &&
			exec ./simplelaunch hostcpu.textproto \
				--logdir "$launch_logs" -p "$port"
	) &
	pid=$!
	echo "${prog}: launcher pid ${pid}, console under ${launch_logs}"
	echo "${prog}: letting the gesture run for ${run_seconds}s"

	local waited=0
	while [ "$waited" -lt "$run_seconds" ]; do
		if ! kill -0 "$pid" 2>/dev/null; then
			wait "$pid" || true
			die "the launcher exited after ${waited}s, before the run was over." \
				"Its console and the three processes' output are under ${launch_logs}."
		fi
		sleep 1
		waited=$((waited + 1))
	done

	echo "${prog}: stopping the run"
	kill -INT "$pid" 2>/dev/null || true
	wait "$pid" || true
}

require_bazel "host motion run"
check_pinion_defaults "$logger_config"
refuse_leftovers
compile
built=$(bazel_files "$(union "$exe_target" "$system_target" "$launcher_target" \
	"$launch_config_target" "$prelaunch_target")")
configs=$(bazel_files "$config_target")
exe_out=$(bazel_named_in "$built" robot_host_clk_exe)
launcher_out=$(bazel_named_in "$built" simplelaunch)
launch_config_out=$(bazel_named_in "$built" hostcpu.textproto)
prelaunch_out=$(bazel_named_in "$built" clockwork_prelaunch.sh)
stage "$exe_out" "$launcher_out" "$launch_config_out" "$prelaunch_out" \
	"$configs" "$built"
echo "${prog}: staged  ${staging}"
run "$(pick_port)"
# The refusal hints are this staging's: the launcher put every process's console
# under the scratch tree, and the namespace a logger that wrote nothing probably
# disagrees about is stated in the staged copy of the logger configuration.
run_dir=$(run_directory "$logs" \
	"Either it never started or it could not open a file there; its output is under ${launch_logs}." \
	"The logger came up and wrote nothing, which is what a pinion namespace or shm-root disagreement looks like: compare ${logger_config} against the flagless defaults every process here runs on.")
echo "${prog}: log  ${run_dir}"
report_verdict "$run_dir" --grid-jitter-ns "$grid_jitter_ns"
