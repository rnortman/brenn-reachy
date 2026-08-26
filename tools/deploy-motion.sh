#!/usr/bin/env bash
#
# Put the built motion payload on a device, and bring a run's log back.
#
#   tools/deploy-motion.sh <host> --push [--stale-ok]
#   tools/deploy-motion.sh <host> --run <dir>
#   tools/deploy-motion.sh <host> --fetch <dir>
#
#   --push       rsync the payload into the unit's RAM and create the directory
#                the logger writes into. Refuses a payload older than the newest
#                commit to the workspace, and refuses while anything else on the
#                unit holds the servo bus.
#   --stale-ok   push the old payload anyway.
#   --run        empty the unit's log root, start the pushed payload on it for a
#                fixed budget, stream its console here, then fetch the records
#                into <dir> and judge them with `first_motion_report`. Refuses
#                while anything else on the unit holds the servo bus, and the
#                exit status is the report's. The log root is emptied so the
#                records judged are this run's: a run refused before its fetch
#                leaves its records for the next run's clear, so `--fetch` them
#                first if they are wanted.
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
# person to get right. `--run` types that command, on a budget, and stops the run
# by letting the budget expire. The run still moves a machine and still belongs
# to whoever is standing next to it: what makes it theirs is that they typed
# `make motion-run` with their eyes on the machine, not that they retyped the
# launcher's arguments. `docs/bench-runbook.md` is the procedure, and its manual
# appendix is the same run started by hand.
#
# Two device paths, both in RAM — nothing a dev cycle pushes touches the eMMC:
#
#   /run/brenn-app/releases/motion  the payload. A tmpfs, and mounted exec where
#       /run itself is not, so a binary has to live here to run at all. The
#       processes are started with this as their working directory, because every
#       configuration file in the payload is named by a path relative to it.
#
#   /run/brenn-app/logs/motion  where the logger writes. Read out of the staged
#       payload's own cogs/robot_logger.textproto rather than stated here,
#       created by the push and emptied by every run: the writer makes the run's
#       own subdirectory under this root, not the root itself, so one root holds
#       a whole session's runs unless a run clears it. Every mode here reads that
#       file, so all three want a built payload.

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
# console output of everything it starts. Both are named in the start command
# below and nowhere else here: the config names the processes, their arguments
# and their working-directory-relative executables, so this script has nothing
# left to say about them.
#
# The log directory is on the same tmpfs as the payload and the records; the
# launcher creates it.
launch_config=robotcpu.textproto
launch_logs="${store_mount}/logs/launch"

# How long the run is given before the launcher is stopped. Commissioning is
# about five seconds of bus transactions, the wake lead is eight, the raise-hold-
# stow gesture about five, the release about four: the same sum the host harness
# budgets twenty-seven for, with a few seconds more of margin because this one is
# talking to a real serial bus whose transactions retry. Nothing about the run is
# judged by the clock — the analyzer reads the records — so the budget only has
# to be long enough. The lead is a number in another file, so the sum is checked
# against the shipped one by tools/deploy-motion.test.sh: a lead that grows
# without this following it is red there rather than a launcher stopped
# mid-gesture.
run_seconds=30

# The bazel that runs the analyzer, whose label and invocation are lib.sh's
# `report_verdict`. The report is a host tool over a fetched log, so it builds
# in the default configuration and the payload's --config=device has nothing to
# do with it: an empty flag set is this script's answer.
bazel=${REACHY_BAZEL:-bazel}
build_flags=()

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
	die "usage: ${prog} <host> --push [--stale-ok]|--run <dir>|--fetch <dir>"
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
# legitimately hold, so the callers below can interpolate what they get plainly.
# One of them builds a remote command run as root and one a remote rsync path the
# far end re-parses: a value with a space or a metacharacter means something
# different at each of those sites, and one refusal is cheaper than quotings that
# have to agree.
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

# Copy a run's records off the unit into a directory of their own, and echo
# where they landed.
#
#   fetch_records <destination> <device log root>
#
# Everything it says goes to stderr: the caller reads the path off stdout.
fetch_records() {
	local dest=$1 log_root=$2
	local stamp out part
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
	local existing
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
	# refusal, not a report over an empty directory. rsync is happy
	# about an empty source, so its exit status says nothing about
	# whether the run logged: what does is a `.olog` with bytes in it.
	# The likeliest cause is the logger having looked for the control
	# process's channels somewhere else, which is silent at run time
	# and looks exactly like this afterwards.
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
	echo "${prog}: ${out}" >&2
	echo "$out"
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
		echo "${prog}: start the run: ${prog} ${host} --run <records-dir>"
		;;

	--run)
		dest=${1:-}
		[ -n "$dest" ] || usage
		[ $# -eq 1 ] || usage
		# Absolute from here on. The report is invoked through `bazel run`,
		# which runs it from its own runfiles tree rather than from here,
		# so a relative records directory would be read against the wrong
		# root -- and what that looks like is a run that produced records
		# and an analyzer that says there is no log in them.
		case $dest in
		/*) ;;
		*) dest="${PWD}/${dest}" ;;
		esac
		require_bazel "the run's report"
		log_root=$(config_string log_root_dir)

		# The bus question and the run are one ssh invocation, which is
		# what makes the question binding: asked separately, a service
		# can start in between and the run meets a held bus anyway. That
		# is bus_probe's stated contract and the bench deploy's shape.
		# For --push the same probe is advisory, because a push starts
		# nothing.
		#
		# The launcher resolves every path in its config against its
		# working directory, so it is started from the release root and
		# nowhere else.
		#
		# The budget is how the run stops: SIGINT is the stop gesture,
		# and --kill-after is for a launcher that does not answer it.
		# Ten seconds is well beyond any controlled shutdown; a grace
		# that expires is a wedged launcher.
		# The log root is emptied in the same invocation, before the
		# launcher starts, so what the fetch brings back is this run and
		# only this run. The device's log root outlives a run — it is
		# RAM, but a session's runs accumulate under it — and both
		# refusals downstream read the *newest* records there: a run
		# that wrote none would be fetched as an earlier run's directory
		# with bytes in its .olog, so the empty-fetch refusal would not
		# fire and the analyzer would judge, and pass, a log from a run
		# that produced nothing. Emptying it also keeps each fetch to
		# one run's records instead of every prior run's.
		#
		# The cost is that a run refused before its fetch leaves its
		# records to be taken by the next run's clear: `--fetch` before
		# re-running is how to keep them, and the runbook says so.
		#
		# A log root outside the payload store is a refusal rather than
		# an `rm -rf` as root: the value comes out of a configuration
		# file, and every path this script builds from it is one the
		# device re-parses.
		#
		# A `.` or `..` component is refused before the prefix is
		# checked, because the prefix check is textual: dots are in the
		# accepted charset and `${store_mount}/..` reads as being under
		# the store while naming anything above it. Nothing a log root
		# ever needs is written that way.
		case $log_root in
		*/../* | */.. | */./* | */. | .. | .)
			die "${logger_config}'s log_root_dir is '${log_root}', which carries a . or .. component." \
				"A run empties that directory on the unit as root, and this check is" \
				"textual, so the path has to name where it points to."
			;;
		esac
		case $log_root in
		"${store_mount}"/?*) ;;
		*)
			die "${logger_config}'s log_root_dir is '${log_root}', which is not under ${store_mount}." \
				"A run empties that directory on the unit before it starts, as root," \
				"so it has to be inside the payload store."
			;;
		esac
		remote="$(bus_probe)"
		remote="${remote}; rm -rf -- ${log_root} && mkdir -p -- ${log_root} || exit 1"
		# The trailing `exit $?` keeps the remote shell in front of the
		# launcher: bash execs a final simple command in place of itself,
		# and a command that dies by a signal makes ssh report its own
		# 255 — the code that means "ssh failed" here, so a payload binary
		# that faults would come back as an unreachable host. With a shell
		# still there the status is 128+signal and says what happened.
		remote="${remote}; cd ${release} || exit 1"
		remote="${remote}; timeout --signal=INT --kill-after=10"
		remote="${remote} ${run_seconds} ./simplelaunch ${launch_config}"
		remote="${remote} --logdir ${launch_logs}"
		remote="${remote}; exit \$?"

		echo "${prog}: running on ${host} for ${run_seconds}s; eyes on the machine" >&2
		rc=0
		# A pty: the launcher's console prints as it goes and a ^C here
		# reaches the remote process group. A Ctrl-C aborts this script
		# before the fetch — the driver's wind-down leaves the machine
		# de-torqued, and the run is re-run.
		#
		# ssh allocates one only when this stdin is a terminal, and
		# proceeds without it otherwise, which silently costs the ^C: an
		# operator watching a run they cannot interrupt is the one thing
		# the eyes-on rule assumes they can do, so the loss is said out
		# loud rather than left to a warning from ssh.
		if [ ! -t 0 ]; then
			echo "${prog}: stdin is not a terminal, so ssh allocates no pty and a ^C here" >&2
			echo "${prog}: will not reach the unit: the run stops at the ${run_seconds}s budget," >&2
			echo "${prog}: and stopping it sooner is 'ssh root@${host} pkill -x simplelaunch'." >&2
		fi
		ssh -t -o BatchMode=yes "root@${host}" "$remote" || rc=$?

		# 3 and 4 are the probe's, and 255 is ssh's own. A launcher chain
		# that itself died with exactly 3, 4 or 255 before the budget is
		# read as one of those instead — accepted: timeout's own codes
		# are 124/125/126/127/137, so the collision needs that exact
		# exit, and either reading fails this run. Only the printed
		# reason would be wrong, which is why the 255 wording below does
		# not assert that nothing happened on the unit: a run that did
		# start and ended 255 leaves records worth fetching first.
		bus_refusal "$rc" "a motion run" \
			"255 is ssh's own code and also the run's if the launcher exited with it" \
			"Its own error is above. Check ${launch_logs} on ${host} before re-running," \
			"and ${prog} ${host} --fetch <records-dir> first if this run's records matter:" \
			"the next run empties the log root."

		case "$rc" in
		124)
			# The budget fired, SIGINT was delivered, the launcher wound
			# down: the expected end of a full run.
			;;
		0)
			die "the launcher exited before the ${run_seconds}s budget was up, so the gesture did not finish." \
				"Its console and the three processes' output are under ${launch_logs} on ${host}."
			;;
		137)
			die "the launcher did not stop on SIGINT and was killed (exit ${rc})." \
				"That is a launcher wedged in its own shutdown; its output is under ${launch_logs} on ${host}."
			;;
		*)
			die "the run on ${host} failed (exit ${rc})." \
				"Its console and the three processes' output are under ${launch_logs} on ${host}."
			;;
		esac

		out=$(fetch_records "$dest" "$log_root")

		# The records are judged here, not on the unit: the analyzer is a
		# host tool and the fetched copy is the one that outlives the
		# tmpfs. No jitter band — a hardware log sits on an absolute
		# grid, so it is read strictly.
		run_dir=$(run_directory "$out" \
			"Either it never started or it could not open a file there; its output is under ${launch_logs} on ${host}." \
			"The logger came up and wrote nothing, which is what a pinion namespace or shm-root disagreement looks like: compare the payload's cogs/robot_logger.textproto against the flagless defaults every process runs on.")
		echo "${prog}: log  ${run_dir}"

		# The report's verdict is this script's.
		report_verdict "$run_dir"
		;;

	--fetch)
		dest=${1:-}
		[ -n "$dest" ] || usage
		log_root=$(config_string log_root_dir)
		out=$(fetch_records "$dest" "$log_root")
		echo "${prog}: read it: bazel run //cogs:first_motion_report -- ${out}/<run>"
		;;

	*) usage ;;
esac
