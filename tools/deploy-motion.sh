#!/usr/bin/env bash
#
# Put the built motion payload on a device, and bring a run's log back.
#
#   tools/deploy-motion.sh <host> --push [--stale-ok]
#   tools/deploy-motion.sh <host> --commands
#   tools/deploy-motion.sh <host> --fetch <dir>
#
#   --push       rsync the payload into the unit's RAM and create the directory
#                the logger writes into. Refuses a payload older than the newest
#                commit to the workspace, and refuses while anything else on the
#                unit holds the servo bus.
#   --stale-ok   push the old payload anyway.
#   --commands   print the commands that start a run, watch it, and stop it.
#   --fetch      copy the run's `.olog` directories back to a local directory,
#                under a name stamped with the moment they were fetched so a
#                session's runs accumulate rather than overwrite. Refuses a fetch
#                that brought no records rather than reporting over nothing.
#
# A motion run is **three OS processes**, not one: `reachy_motord` (the servo
# bus), and two `robot_clk_exe` processes over the one synthesized executable
# (the logger, and the control loop). All three are started by one supervisor —
# `simplelaunch`, from the launcher config the compositions render — so there is
# one command to start a run and one gesture to stop it, and no ordering for a
# person to get right. This script still does not start them: it pushes the
# payload and says what to type, because the run itself moves a machine and
# belongs to whoever is standing next to it. `docs/bench-runbook.md` is the
# procedure.
#
# Two device paths, both in RAM — nothing a dev cycle pushes touches the eMMC:
#
#   /run/brenn-app/releases/motion  the payload. A tmpfs, and mounted exec where
#       /run itself is not, so a binary has to live here to run at all. The
#       processes are started with this as their working directory, because every
#       configuration file in the payload is named by a path relative to it.
#
#   /run/brenn-app/logs/motion  where the logger writes. Read out of the staged
#       payload's own cogs/robot_logger.textproto rather than stated here, and
#       created by the push: the writer makes the run's own subdirectory under
#       this root, not the root itself. Every mode here reads that file, so all
#       three want a built payload.

set -euo pipefail

# shellcheck source=lib.sh
. "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

# The payload tools/build-motion.sh stages.
payload="${repo_root}/target/motion-arm64/release"

# The three binaries, at the paths the launcher config spells, checked before
# anything is pushed. A payload directory that exists and is missing one of them
# is a build that failed halfway or a stage somebody edited by hand.
binaries=(reachy_motord cogs/robot_clk_exe simplelaunch)

# The logger configuration, read out of the staged payload rather than out of the
# tree. The values in force on the device are the ones that were staged, and an
# edit to the checked-in file after the last build is not among them: the
# freshness refusal does not catch uncommitted edits. Every value printed or
# created here must describe the artefact that will run, and a missing payload is
# a refusal that says to build one.
logger_config="${payload}/cogs/robot_logger.textproto"

# The launcher config, at the payload root, and where the launcher puts the
# console output of everything it starts. Both are named in the commands printed
# below and nowhere else here: the config names the processes, their arguments
# and their working-directory-relative executables, so this script has nothing
# left to say about them.
#
# The log directory is on the same tmpfs as the payload and the records; the
# launcher creates it.
launch_config=robotcpu.textproto
launch_logs="${store_mount}/logs/launch"

# One directory, reused. Nothing on this path activates a release, so nothing
# prunes the store either; rsync --delete is what makes reuse idempotent.
release="${store_mount}/releases/motion"

# The payload runs as root, unlike a bench run: the control process needs
# `/dev/shm` shared with the logger and the driver needs the serial node, and the
# unit runs nothing else while a motion test is on it. That is the operator's
# call and the runbook says so; this script pushes and nothing more.

# What the device payload is built out of: the sources, everything that decides
# how they are compiled, the compositions and the configuration the processes
# read, and the two scripts that decide what a built payload is: the one that
# names the platform and the compilation mode, and the shared prelude it takes
# its ELF verification from.
workspace_paths=(
	crates cogs driver motion hardware geometry clips bazel
	MODULE.bazel MODULE.bazel.lock .bazelrc .bazelversion
	tools/build-motion.sh tools/lib.sh
)

usage() {
	die "usage: ${prog} <host> --push [--stale-ok]|--commands|--fetch <dir>"
}

# A scalar out of the staged protobuf text. One field per line and quoted
# strings, which is the whole of the syntax this file is written in; a field this
# cannot find is a refusal, because every caller below writes the answer into a
# command that has to be right.
#
# Leading whitespace is allowed: the pattern is line-oriented either way, but
# refusing an indented field would make the file's formatting load-bearing, so a
# submessage or a formatter that indents would turn every command here into a
# refusal for a reason that has nothing to do with the values.
#
# Every value is checked here against the character set these fields can
# legitimately hold, so the three callers below can interpolate what they get
# plainly. One of them builds a remote command run as root, one a remote rsync
# path the far end re-parses, and one a shell command a person pastes: a value
# with a space or a metacharacter means something different at each of those
# sites, and one refusal is cheaper than three quotings that have to agree.
#
# An empty value is refused in its own words. `field: ""` and a missing field are
# the same thing to every caller and both are stops, but they are different edits
# to make.
config_string() {
	local field=$1 line value
	[ -f "$logger_config" ] || die \
		"no logger configuration at ${logger_config}" \
		"That file is staged by the build, so build the payload first: make motion-build"
	line=$(sed -n "s/^[[:space:]]*\\(${field}: \".*\"\\)\$/\\1/p" -- "$logger_config" | head -n 1)
	[ -n "$line" ] ||
		die "${logger_config} states no ${field}, so the command that needs it cannot be built." \
			"If the field was renamed, this script and the runbook both name the old one."
	value=${line#*: \"}
	value=${value%\"}
	[ -n "$value" ] ||
		die "${logger_config} states an empty ${field}, so the command that needs it would name nothing."
	case $value in
	*[!A-Za-z0-9/_.-]*)
		die "${logger_config}'s ${field} is '${value}', which is not a plain path or name." \
			"These values are pasted into a remote command and a local one, so only" \
			"[A-Za-z0-9/_.-] is accepted here."
		;;
	esac
	echo "$value"
}

host=${1:-}
mode=${2:-}
[ -n "$host" ] || usage
shift 2 || usage

case "$mode" in
	--push)
		[ -d "$payload" ] || die \
			"no device payload at ${payload}" \
			"Build one first: make motion-build"
		for name in "${binaries[@]}"; do
			[ -x "${payload}/${name}" ] || die \
				"the payload at ${payload} has no executable ${name}" \
				"Rebuild it: make motion-build"
		done

		if [ "${1:-}" = "--stale-ok" ]; then
			shift
			echo "${prog}: --stale-ok: the payload's age is not being checked" >&2
			[ $# -eq 0 ] || usage
		else
			# Anything left here is a misspelling of --stale-ok, and
			# accepting it silently means the freshness check runs
			# for an operator who believes they overrode it.
			[ $# -eq 0 ] || usage
			refuse_if_stale "${payload}/cogs/robot_clk_exe" "device payload" \
				"make motion-build" \
				"push the old payload deliberately" \
				"${prog} ${host} --push --stale-ok" \
				"${workspace_paths[@]}"
		fi

		log_root=$(config_string log_root_dir)

		# The bus question is asked before anything is pushed, in the
		# same remote invocation that makes the two directories — so a
		# refusal creates nothing and a clean answer leaves the unit
		# ready. Refused rather than stopped: what is running on a
		# device is the operator's to decide. The question and the
		# refusals it turns into are lib.sh's, shared with the bench
		# deploy: one contract, documented once.
		#
		# For a push the probe is advisory and nothing more. The rsync
		# below is a second connection, so a service can still start
		# between the two, and the run itself is started by a person
		# later with no probe at all. What actually keeps two claimants
		# off the bus is the driver's exclusive open (TIOCEXCL + flock);
		# pushing files touches no bus. The bench deploy's probe *is* a
		# gate because there the question and the run share one
		# invocation, and this script starts nothing.
		remote="$(bus_probe)"
		remote="${remote}; mkdir -p -- ${release} ${log_root}"

		rc=0
		ssh_root "$remote" || rc=$?
		bus_refusal "$rc" "a motion run" "nothing was pushed"
		[ "$rc" = 0 ] ||
			die "preparing ${host} failed (exit ${rc}); nothing was pushed."

		# The whole directory, contents-of rather than the directory itself,
		# so the layout under the release root is the layout the processes'
		# relative paths expect. --delete because a file the payload stopped
		# carrying must not stay behind being run.
		echo "${prog}: pushing ${payload}/ to ${host}:${release}/" >&2
		rsync -a --delete -e "ssh -o BatchMode=yes" \
			"${payload}/" "root@${host}:${release}/"

		echo "${prog}: pushed. The log root ${log_root} exists."
		echo "${prog}: the commands that start the run: ${prog} ${host} --commands"
		;;

	--commands)
		# Printed rather than run: the run moves a machine, and what is
		# printed is exactly what a person types with their eyes on it.
		#
		# One start command, because the launcher config names all three
		# processes and every one of them runs on the pinion defaults --
		# there are no flags to interpolate here and no order to state.
		log_root=$(config_string log_root_dir)
		cat <<COMMANDS
# On ${host}, as root. The launcher resolves every path in its config against
# its working directory, so it is started from the release root and nowhere
# else. It runs in the foreground; Ctrl-C stops all three processes.
cd ${release} && ./simplelaunch ${launch_config} --logdir ${launch_logs}

# Watch the run: the launcher redirects each process's console into its own file,
# named for the app and the number of times it has been started in this log
# directory -- so the second run in it writes motord_1.log, and so on. The three
# app names are the compositions', and build-motion.sh refuses a build whose
# rendered launcher config names any other set, so these three paths are the ones
# that will appear.
tail -f ${launch_logs}/motord_0.log        # the driver: cycles, torque, faults
tail -f ${launch_logs}/proc_0.log          # the control loop: the session
tail -f ${launch_logs}/logger_proc_0.log   # the logger

# Before powering down, while the unit is still up: the run wrote records.
ls -l ${log_root}/*/

# Stopping: Ctrl-C in the launcher's terminal, or from anywhere on the unit
curl -X POST 127.0.0.1:8080/quit
COMMANDS
		;;

	--fetch)
		dest=${1:-}
		[ -n "$dest" ] || usage
		log_root=$(config_string log_root_dir)
		mkdir -p -- "$dest"
		stamp=$(date -u +%Y%m%dT%H%M%SZ)
		out="${dest}/motion-log-${stamp}"

		# Into a partial directory first, moved into place only once rsync
		# is happy: every fetch is timestamped so nothing is ever
		# overwritten, and a failed fetch that left a plausible-looking
		# directory behind would be a log somebody analyses.
		part="${out}.part"

		# The stamp has one-second resolution and `mv` moves a directory
		# *into* an existing one of the same name, which would file a
		# run's records under an unrelated run's directory with a name
		# saying they were partial. Two fetches in the same second is a
		# refusal instead. A leftover .part is refused for the same
		# reason: rsync would merge into it.
		for existing in "$out" "$part"; do
			if [ -e "$existing" ]; then
				die "${existing} already exists, so this fetch has nowhere of its own to land." \
					"Fetches are stamped to the second. Move it aside, or wait a second and fetch again."
			fi
		done
		echo "${prog}: fetching ${host}:${log_root}/ to ${out}" >&2
		mkdir -p -- "$part"
		rsync -a -e "ssh -o BatchMode=yes" \
			"root@${host}:${log_root}/" "${part}/" || {
			rm -rf -- "$part"
			die "nothing fetched from ${host}:${log_root}." \
				"A run that wrote no records leaves that directory empty."
		}
		# A fetch that succeeded and brought nothing worth reading is a
		# refusal, not a report command over an empty directory. rsync is
		# happy about an empty source, so its exit status says nothing
		# about whether the run logged: what does is a `.olog` with bytes
		# in it. The likeliest cause is the logger having looked for the
		# control process's channels somewhere else, which is silent at
		# run time and looks exactly like this afterwards.
		if [ -z "$(find "$part" -name '*.olog' -size +0 -print -quit)" ]; then
			rm -rf -- "$part"
			die "the fetch from ${host}:${log_root} carries no .olog with anything in it." \
				"The run wrote no records. Either the logger never started, or it found no" \
				"channels: it reads the buffer directory and namespace out of the payload's" \
				"cogs/robot_logger.textproto, and every process is started with no pinion" \
				"flags at all, so those two values have to be the compiled-in defaults." \
				"Nothing was kept, so the next fetch is not merging into this one."
		fi

		mv -- "$part" "$out"
		echo "${prog}: ${out}"
		echo "${prog}: read it: bazel run //cogs:first_motion_report -- ${out}/<run>"
		;;

	*) usage ;;
esac
