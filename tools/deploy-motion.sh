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
#                unit holds the servo bus. Stamps the workspace's commit beside
#                the payload, which is what a fetched run's records name their
#                build by; a push that cannot state its own commit refuses.
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
# A fetch brings back two things under one stamp: `motion-log-<stamp>`, the
# records the analyzer judges — with `provenance.txt` at its root naming the
# build that recorded them — and `motion-log-<stamp>.console` beside it,
# holding the console output of everything the launcher started. A run adds its
# own console stream and the unit's clock discipline, read before and after, to
# the second of those. The analyzer reads none of it: the log is self-contained,
# and everything the driver counts about itself is republished into it. The
# console is for a person reading a run that went wrong, and the clock captures
# say whether the time base the whole log is stamped in could have stepped
# underneath it.
#
# A motion run under the launcher is **three OS processes**, not one:
# `reachy_motord` (the servo bus), and two `robot_clk_exe` processes over the one
# synthesized executable (the logger, and the control loop), with `reachy_ask`
# beside them holding the intent edge. All three are started by one supervisor —
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
#   /run/brenn-app/releases/motion  the payload, in the /run/brenn-app tmpfs
#       submount, which is mounted exec where /run itself is not, so a binary
#       has to live here to run at all. The processes are started with this as
#       their working directory, because every configuration file in the payload
#       is named by a path relative to it.
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

# The payload's binaries, at the paths the run spells, checked before anything is
# pushed. A payload directory that exists and is missing one of them is a build
# that failed halfway or a stage somebody edited by hand. Three are the launcher
# config's; the fourth is the intent source `--run` starts ahead of it.
binaries=(reachy_motord reachy_ask cogs/robot_clk_exe simplelaunch)

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
# launcher creates it, a run empties it first, and a fetch brings it back.
launch_config=robotcpu.textproto
launch_logs="${store_mount}/logs/launch"

# How long the run is given before the launcher is stopped. Commissioning is
# about five seconds of bus transactions, the harness gesture's arming offset is
# eight, the raise-hold-stow gesture about five, the release about four: the host
# harness budgets twenty-seven for the same sum, and this one carries a few
# seconds more of margin because it is talking to a real serial bus whose
# transactions retry. Nothing about the run is judged by the clock — the analyzer
# reads the records — so the budget only has to be long enough. The offset is a
# number in another file, so the sum is checked against the shipped one by
# tools/deploy-motion.test.sh: an offset growing without this following it is red
# there rather than a launcher stopped mid-gesture.
run_seconds=30

# The name of the intent source in the payload, and where its console goes.
#
# It is not a launcher app and cannot be: it binds the narration port, and the
# composition starts narrating on its first execution, so it has to be running
# before the launcher is. Started here, ahead of the launcher, and stopped with
# it — a run's verdict is the analyzer's over the fetched records, so what this
# console holds is why a run went the way it did rather than the verdict itself.
ask_binary=reachy_ask
ask_console_name=reachy_ask.log

# The bazel that runs the analyzer, whose label and invocation are lib.sh's
# `report_verdict`. The report is a host tool over a fetched log, so it builds
# in the default configuration and the payload's --config=device has nothing to
# do with it: an empty flag set is this script's answer.
bazel=${REACHY_BAZEL:-bazel}
build_flags=()

# One directory, reused. Nothing on this path activates a release, so nothing
# prunes the store either; rsync --delete is what makes reuse idempotent.
#
# Coupled to brenn-pod's deploy-reachy-pod.sh, which reads this path and the
# stamp below to guard against replacing a robot's payload with a pod-only one.
# A rename here is invisible there, so both names are pinned by a case in this
# script's self-test; changing either means changing both repos.
release="${store_mount}/releases/motion"

# The name the push's account of which build the payload is goes by, at the root
# of the payload and at the root of a run's log root. Part of the payload: it is
# written into the staged directory before the push, so the one rsync that
# delivers the payload delivers the stamp with it and a stamp on the unit
# describes the payload beside it by construction. The build stages that
# directory from scratch, so nothing stale is left there to push. The copy in the
# log root comes home with the records under the fetch's own name, with no
# fetch-side logic to put it there. It is also what a pod deploy from brenn-pod
# reads to recognise a robot — see the note at `release` above.
provenance_name=provenance.txt

# Where a run parks its copy of the stamp while the log root is being emptied.
#
# Under the payload store and outside both the log root and the launcher's
# console directory, so the wipe cannot take it, and on the same tmpfs as the log
# root (`store_mount` is one mount, `tools/lib.sh`), so moving it in afterwards is
# a rename within one filesystem rather than a copy that can fail on a full
# tmpfs. That is the point of staging it: a copy made after the wipe is a
# failure that has already destroyed the previous run's records.
#
# "Outside the log root" is a check and not a hope: the log root comes out of the
# payload's logger configuration, and a value naming this path would have the run
# stage the stamp and then wipe the stage. The `--run` validation below refuses
# that configuration before anything on the unit is touched.
staged_provenance="${store_mount}/motion-provenance.staged"

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
	for existing in "$out" "${out}.console" "$part"; do
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

	# The consoles of everything the launcher started, beside the records
	# rather than inside them. Beside, because `run_directory` takes the
	# newest *directory* under the fetched root as the run: a directory of
	# console files landing there would be taken as the run and the fetch
	# would end in a refusal about a missing .olog.
	#
	# The whole directory, never a list of names: the launcher numbers its
	# files per run and adds its own, so a spelled-out set silently misses
	# whatever it did not know about.
	#
	# Best-effort throughout. What these files carry is the driver's own
	# counters, which are evidence about the log rather than about the
	# machine, and a run that produced records is worth reporting on whether
	# or not its consoles came back.
	local console="${out}.console"
	mkdir -p -- "$console"
	if rsync -a -e "ssh -o BatchMode=yes" \
		"root@${host}:${launch_logs}/" "${console}/"; then
		if [ -z "$(find "$console" -type f -print -quit)" ]; then
			echo "${prog}: ${launch_logs} on ${host} held no console files" >&2
		fi
	else
		echo "${prog}: no console logs fetched from ${host}:${launch_logs}" >&2
	fi

	echo "${prog}: ${out}" >&2
	echo "$out"
}

# What build a run's records came off, into the file a run carries home.
#
#   stamp_provenance <file> <yes|no: was the payload's age left unchecked>
#
# The log reader binds each channel's schema byte for byte, so a run's records
# are read with the build that recorded them and a records directory that cannot
# name its build is one nobody can decode after the next `.clk` append. Nothing
# else in a fetch says which build it was.
#
# What it can honestly claim is narrow, and it claims exactly that. The push-time
# facts are weaker than they look: the freshness refusal compares the payload's
# age against the newest commit and does not catch uncommitted edits, and
# --stale-ok skips it altogether. And the pushing tree's HEAD is not by itself
# the commit the binaries came from: a payload built at one commit can be pushed
# from a checkout at any other, and the age refusal only turns away a payload
# that is too old — an older checkout passes it and would be stamped with a
# commit that never produced the binaries. So the commit the stamp names is the
# one the build recorded in the payload (`build_commit_name`, lib.sh) whenever
# the payload carries it, `commit_source` says which of the two answered, and
# `pushed_from` keeps the pushing tree's HEAD beside it so a tree that moved
# between the build and the push is visible rather than averaged away.
#
# The rest is the same honesty: whether the tree had uncommitted changes when it
# was pushed, and whether the age was checked at all — a dirty or stale push says
# so on its face instead of lying by omission, and a clean fresh one makes
# reading the log a `git switch --detach`.
#
# A tree that cannot state its commit is a push refusal, not a stamp saying
# nothing: the whole point of the file is that a fetched log names its build.
stamp_provenance() {
	local into=$1 age_unchecked=$2
	local pushed_from dirty built commit commit_source
	pushed_from=$(git -C "$repo_root" rev-parse HEAD 2>/dev/null) || pushed_from=
	[ -n "$pushed_from" ] ||
		die "this tree cannot state its own commit, so a push from it could not say which build ran." \
			"Every fetched records directory carries that commit, because a log is only" \
			"readable by the build that recorded it. Push from a checkout with history."
	built=
	if [ -f "${payload}/${build_commit_name}" ]; then
		built=$(sed -n 's/^commit=//p' -- "${payload}/${build_commit_name}")
	fi
	case $built in
	'' | unknown)
		# A payload staged by a build that recorded nothing — an older
		# build script, or a build in a tree with no history. The
		# pushing tree's HEAD is the only answer left, and the field
		# below says that is what it is.
		commit=$pushed_from
		commit_source=push
		;;
	*)
		commit=$built
		commit_source=build
		;;
	esac
	if ! dirty=$(git -C "$repo_root" status --porcelain 2>/dev/null); then
		dirty=unknown
	elif [ -n "$dirty" ]; then
		dirty=yes
	else
		dirty=no
	fi
	cat >"$into" <<STAMP
# Which build recorded the records beside this file. Written by ${prog} when the
# payload was pushed and copied here by the run.
#
# The log reader binds a channel's schema byte for byte, so read these records
# with the build that wrote them:
#     git switch --detach ${commit}
#
# commit_source=build means the payload itself recorded that commit when it was
# staged, which is the build the binaries came out of. commit_source=push means
# the payload recorded none and this is the pushing tree's HEAD instead, which
# describes the binaries only if that tree had not moved since the build.
# pushed_from is that HEAD either way: where it differs from commit, the tree
# moved between the build and the push and commit is the one that built.
#
# dirty=yes means the workspace held uncommitted changes at push time, so that
# commit does not fully describe what ran. dirty=unknown means the repository
# would not answer the status question at push time, so whether there were any is
# not known. age_unchecked=yes means the push skipped the refusal that compares
# the payload's age against the newest commit, so the payload may predate that
# commit.
commit=${commit}
commit_source=${commit_source}
pushed_from=${pushed_from}
dirty=${dirty}
age_unchecked=${age_unchecked}
pushed=$(date -u +%Y%m%dT%H%M%SZ)
STAMP
	echo "${prog}: provenance: commit ${commit} (${commit_source}), pushed from ${pushed_from}," \
		"dirty=${dirty}, age_unchecked=${age_unchecked}" >&2
}

# One read-only probe of the unit, into a file of its own.
#
#   capture_probe <file> <headline> <subject> <probe>
#
# The wrapper every capture shares, so the invariants live in one place: the ssh
# is BatchMode (a unit that wants a password is a unit that answers nothing, not
# a prompt nobody is at), stderr is captured into the record because what a
# probe could not read is part of the reading, and a unit that answers nothing
# leaves a file saying so rather than failing a run that has already happened.
#
# <subject> completes "answered nothing about its ...".
capture_probe() {
	local into=$1
	local headline=$2
	local subject=$3
	local probe=$4
	{
		echo "# ${host} ${headline}, $(date -u +%Y%m%dT%H%M%SZ)"
		ssh -o BatchMode=yes "root@${host}" "$probe" 2>&1 ||
			echo "# ${host} answered nothing about its ${subject}"
	} >"$into"
}

# The unit's clock discipline, into a file of its own.
#
#   capture_clock <file>
#
# What a run's timestamps mean depends on whether the time daemon slews or
# steps: a backwards CLOCK_REALTIME step is the loss of the driver's time base,
# and nothing in a run's records says whether the daemon can take one. Captured
# before and after a run, so a step during it shows up as two readings that
# disagree.
#
# Best-effort in every direction: whichever daemon is installed answers and
# whatever is absent says so, over the wrapper that never fails a run.
capture_clock() {
	local into=$1
	local probe
	probe='timedatectl show 2>&1; timedatectl timesync-status 2>&1'
	probe="${probe}; for unit in systemd-timesyncd chronyd ntpd ntpsec; do"
	probe="${probe} systemctl is-active --quiet \$unit &&"
	probe="${probe} systemctl status --no-pager --lines=0 \$unit 2>&1; done"
	probe="${probe}; command -v chronyc >/dev/null && chronyc tracking 2>&1"
	probe="${probe}; true"
	capture_probe "$into" "clock state" "clock" "$probe"
}

# The kernel facts a bus timing measurement is read against, into a file of its
# own.
#
#   capture_host_facts <file>
#
# A serial exchange that takes milliseconds for microseconds of wire time is
# either the port blocking or the loop thread being descheduled, and which one it
# is depends on things no record of a run carries: the timer tick the kernel
# sleeps in units of, whether it runs tickless, the kernel itself, and which
# driver is behind the serial ports. Captured once per run beside the clock
# readings, because the unit is re-flashed and re-booted between sessions and a
# measurement filed without them cannot be compared with the next one.
#
# Every serial port the unit has, rather than the one the driver opens: naming
# that one here would restate a value the driver's own configuration holds, and
# the whole set is two lines of output.
#
# Read-only and best-effort in every direction, like the clock capture: a kernel
# built without its configuration exposed says so, over the wrapper that never
# fails a run.
capture_host_facts() {
	local into=$1
	local probe
	probe='uname -srvm'
	probe="${probe}; if [ -r /proc/config.gz ]; then zcat /proc/config.gz |"
	probe="${probe} grep -E '^CONFIG_(HZ|HZ_[0-9]+|NO_HZ[A-Z_]*|HIGH_RES_TIMERS|PREEMPT[A-Z_]*)=';"
	probe="${probe} else echo '# no /proc/config.gz: this kernel does not publish its configuration';"
	probe="${probe} fi"
	probe="${probe}; for tty in /sys/class/tty/ttyAMA* /sys/class/tty/ttyS*; do"
	probe="${probe} [ -e \"\$tty\" ] || continue;"
	probe="${probe} echo \"\$tty driver \$(basename \"\$(readlink -f \"\$tty/device/driver\" 2>/dev/null)\" 2>&1)\";"
	probe="${probe} done"
	probe="${probe}; true"
	capture_probe "$into" "kernel facts" "kernel" "$probe"
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

		age_unchecked=no
		if [ "${1:-}" = "--stale-ok" ]; then
			shift
			age_unchecked=yes
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

		# Into the staged payload, so the one rsync below carries it and
		# the stamp on the unit can only describe the payload it landed
		# with. Written before anything reaches the unit, so a tree that
		# cannot state its commit refuses without having touched it.
		stamp_provenance "${payload}/${provenance_name}" "$age_unchecked"

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
		# The staging path the stamp waits at while the log root is
		# emptied has to survive that wipe, and the log root is a
		# configured value: one naming the stage, or a directory under
		# it, would have the run stage the stamp into the very path it is
		# about to remove and then fail the move in -- a
		# records-are-gone refusal caused by a name collision, with
		# nothing in the message pointing at it.
		case $log_root in
		"${staged_provenance}" | "${staged_provenance}"/*)
			die "${logger_config}'s log_root_dir is '${log_root}', which is where a run stages its provenance stamp." \
				"The stamp is put there before the log root is emptied and moved in afterwards," \
				"so the log root has to be somewhere else under ${store_mount}."
			;;
		esac
		# The payload the launcher runs out of is under the store too, and
		# a log root naming the release directory -- or any directory
		# holding it -- would have the run delete the binaries it is
		# about to start. Every check ahead of the wipe passes, the wipe
		# takes the payload, and the failure surfaces two steps later as
		# the `cd` into a release that is no longer there: a
		# records-are-gone refusal blaming the store for a payload this
		# run removed, when the fix is a push.
		case $release in
		"${log_root}" | "${log_root}"/*)
			die "${logger_config}'s log_root_dir is '${log_root}', which holds the payload at ${release}." \
				"A run empties that directory on the unit as root before it starts, so this one" \
				"would delete the binaries it then tries to run; the log root has to be somewhere" \
				"else under ${store_mount}."
			;;
		esac
		# This chain's own exit codes, named where they are emitted and
		# read by name in the status `case` below, so an emit and its
		# refusal cannot drift apart in two places 120 lines from each
		# other. 3 and 4 are bus_probe's, 255 is ssh's, and timeout's are
		# 124-127/137, so a code added here has to miss all of those; the
		# runbook documents this set beside the probe's.
		rc_no_stamp=5
		rc_stamp_unstaged=6
		rc_post_wipe=7
		remote="$(bus_probe)"
		# The stamp is asked about before the log root is emptied, not
		# after: this refusal fires on a payload pushed by an older
		# script or landed before a reboot cleared the tmpfs, and the
		# previous run's records are often still sitting in that log root
		# unfetched. A refusal that had already destroyed them would cost
		# an operator the only copy of a run's .ologs to say that this run
		# could not name its build.
		#
		# The stamp is part of the payload, so a payload carrying none
		# was put there by something else, and a run of it would record a
		# log nobody can say the schema of. Refused rather than skipped,
		# because a records directory that cannot name its build is the
		# whole failure this closes.
		remote="${remote}; [ -f ${release}/${provenance_name} ] || exit ${rc_no_stamp}"
		# The stamp is copied to its staging path before the wipe, for
		# the same reason the probe runs before it: a copy that fails
		# here has emptied nothing, so the refusal can send the operator
		# to fetch the previous run's records rather than tell them about
		# records that no longer exist.
		remote="${remote}; cp -- ${release}/${provenance_name} ${staged_provenance} || exit ${rc_stamp_unstaged}"
		# Everything from here on runs with the log root already emptied,
		# and every one of these steps answers with the one code that
		# says so.
		remote="${remote}; rm -rf -- ${log_root} && mkdir -p -- ${log_root} || exit ${rc_post_wipe}"
		# The push's stamp, into the log root the fetch brings home, so
		# the records name the build that recorded them without the
		# fetch having to know anything. A rename within the store's own
		# tmpfs, so full-tmpfs and permission failures cannot reach it.
		remote="${remote}; mv -- ${staged_provenance} ${log_root}/${provenance_name} || exit ${rc_post_wipe}"
		# The payload's first publishes are started here, and the front
		# of each stream is whatever the logger was late for: it opens
		# its subscriptions on a poll after it opens the log and attaches
		# to a channel at the write head. Nothing in the payload waits
		# for it, looks for it, or knows about it -- a logger is never a
		# precondition for driving motors -- so how much of a stream's
		# head a run holds is the logger's attach time and nothing the
		# driver decides. The log is self-contained anyway: every fact
		# the report judges is republished periodically by the driver or
		# retained on a persistent channel, so a late attach costs the
		# log stream redundancy and no fact. The report measures the
		# loss per channel and fails a run that is missing one of those
		# carriers outright.
		# The launcher's console directory is emptied with the log root,
		# for the reason the log root is: the launcher numbers its files
		# per run and never overwrites, so a directory left to
		# accumulate holds several runs' driver consoles and nothing
		# afterwards can say which run's counters are which. It is a
		# script constant under the payload store, not a configured
		# value, so it needs none of the checks above.
		remote="${remote}; rm -rf -- ${launch_logs} && mkdir -p -- ${launch_logs} || exit ${rc_post_wipe}"
		# The trailing `exit $?` keeps the remote shell in front of the
		# launcher: bash execs a final simple command in place of itself,
		# and a command that dies by a signal makes ssh report its own
		# 255 — the code that means "ssh failed" here, so a payload binary
		# that faults would come back as an unreachable host. With a shell
		# still there the status is 128+signal and says what happened.
		remote="${remote}; cd ${release} || exit ${rc_post_wipe}"
		# The intent source, before the launcher and in the
		# background: it binds the narration port, and the control
		# process narrates from its first execution, so a bind that
		# came after the launcher would be a race. Its console goes
		# beside the launcher's, which the run empties and an operator
		# reads; its exit status is deliberately not this run's, which
		# is the analyzer's over the fetched records.
		remote="${remote}; ./${ask_binary} --resting-timeout ${run_seconds}"
		remote="${remote} --run-window ${run_seconds}"
		remote="${remote} >${launch_logs}/${ask_console_name} 2>&1 &"
		remote="${remote} ask=\$!"
		remote="${remote}; timeout --signal=INT --kill-after=10"
		remote="${remote} ${run_seconds} ./simplelaunch ${launch_config}"
		remote="${remote} --logdir ${launch_logs}"
		remote="${remote}; rc=\$?"
		remote="${remote}; kill -INT \$ask 2>/dev/null"
		remote="${remote}; wait \$ask 2>/dev/null"
		remote="${remote}; exit \$rc"

		# Where the host-side evidence waits until there is a fetched
		# records directory to file it beside: the clock captures and
		# the console stream are made before that directory exists, and
		# a run that never reaches its fetch has nothing for them to
		# belong to.
		aside=$(mktemp -d)
		trap 'rm -rf -- "$aside"' EXIT
		capture_clock "${aside}/clock-before.txt"
		capture_host_facts "${aside}/host-facts.txt"

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
		# Teed as well as printed: what scrolls past an operator is the
		# only place the launcher's own bookkeeping and the driver's
		# counter summaries appear live, and a terminal scrollback is
		# not a record. The pty is unaffected — ssh decides on one by
		# this stdin, not by where its stdout goes.
		#
		# `pipefail` is off across the pipeline so that a failing run
		# does not fail the script here: the status wanted is ssh's own,
		# read out of PIPESTATUS before anything else resets it.
		#
		# Both statuses are taken in the same breath, on either branch of
		# the AND-OR list, for two reasons: `set -e` does not act on a
		# command that is part of one, and the console copy must not be
		# able to end the run's records. With `pipefail` off the
		# pipeline's own status is `tee`'s, so a `tee` that fails -- the
		# records filling mid-stream is the realistic way -- would
		# otherwise exit the script after the run had happened and
		# before the fetch, with the only copy of the records still on
		# the unit's tmpfs.
		set +o pipefail
		ran=()
		ssh -t -o BatchMode=yes "root@${host}" "$remote" 2>&1 |
			tee -- "${aside}/run-console.log" &&
			ran=("${PIPESTATUS[@]}") || ran=("${PIPESTATUS[@]}")
		set -o pipefail
		rc=${ran[0]}
		if [ "${ran[1]:-0}" -ne 0 ]; then
			echo "${prog}: the console copy failed (tee exited ${ran[1]}): the run itself" >&2
			echo "${prog}: is unaffected and its records are fetched below, but" >&2
			echo "${prog}: ${aside}/run-console.log is short or missing." >&2
		fi

		capture_clock "${aside}/clock-after.txt"

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
				"Its console and every process's output are under ${launch_logs} on ${host}."
			;;
		"$rc_no_stamp")
			# The probe above, before the launcher was reached. Same
			# accepted collision as 3, 4 and 255: a launcher chain
			# that exited with exactly this code reads as this
			# instead, and either reading fails the run.
			die "${host}'s payload carries no provenance stamp, so this run's records could not name their build." \
				"Nothing was started and nothing was emptied. The push writes that stamp, so push again:" \
				"    ${prog} ${host} --push"
			;;
		"$rc_stamp_unstaged")
			# The stamp is there — the probe proved it one command
			# earlier — and staging it failed. Staging runs before
			# the wipe, so this refusal has destroyed nothing. Same
			# accepted collision as the other codes: a launcher chain
			# that exited with exactly this code reads as this
			# instead, and either reading fails the run.
			die "${host}'s payload carries its provenance stamp, but copying it to ${staged_provenance} failed," \
				"so this run's records could not have named their build. A full tmpfs or a" \
				"permission on the payload store is the usual cause; ${host}'s own error is above." \
				"Nothing was started and nothing was emptied, so this run's log root still holds" \
				"whatever the previous run left there:" \
				"    ${prog} ${host} --fetch <records-dir>"
			;;
		"$rc_post_wipe")
			# One of the four steps that run after the wipe began:
			# the log root's own mkdir, the stamp's move into it, the
			# launcher console directory's wipe and recreate, or the
			# cd into the release. What they have in common is the
			# only thing this arm can say, and it is the thing the
			# operator needs: the previous run's records are gone.
			# Same accepted collision as the other codes: a launcher
			# chain that exited with exactly this code reads as this
			# instead, and either reading fails the run.
			#
			# It does not send anybody to the launcher's console
			# directory: on one of these paths that directory is what
			# could not be made, and on the others the launcher never
			# ran to write anything into it.
			die "preparing ${host} for the run failed after the log root wipe had begun (exit ${rc})." \
				"Treat the previous run's unfetched records on the unit as gone." \
				"The launcher was not started, so nothing moved." \
				"${host}'s own error is above; a full or read-only payload store is the usual cause."
			;;
		137)
			die "the launcher did not stop on SIGINT and was killed (exit ${rc})." \
				"That is a launcher wedged in its own shutdown; its output is under ${launch_logs} on ${host}."
			;;
		*)
			die "the run on ${host} failed (exit ${rc})." \
				"Its console and every process's output are under ${launch_logs} on ${host}."
			;;
		esac

		out=$(fetch_records "$dest" "$log_root")

		# The host-side evidence joins the device's own, under the one
		# name that says which fetch it belongs to. One file at a time
		# and per-file best-effort: each of these is captured
		# best-effort in the first place, and a run's records are not
		# worth losing over a capture that did not land.
		console="${out}.console"
		mkdir -p -- "$console"
		for captured in clock-before.txt clock-after.txt host-facts.txt run-console.log; do
			[ -e "${aside}/${captured}" ] || {
				echo "${prog}: no ${captured} to file with the records" >&2
				continue
			}
			mv -- "${aside}/${captured}" "$console" || {
				echo "${prog}: ${captured} could not be filed with the records" >&2
			}
		done
		echo "${prog}: console ${console}"

		# The records are judged here, not on the unit: the analyzer is a
		# host tool and the fetched copy is the one that outlives the
		# tmpfs. No jitter band — a hardware log sits on an absolute
		# grid, so it is read strictly.
		run_dir=$(run_directory "$out" \
			"Either it never started or it could not open a file there; its output is under ${launch_logs} on ${host}." \
			"The logger came up and wrote nothing, which is what a pinion namespace or shm-root disagreement looks like: compare the payload's cogs/robot_logger.textproto against the flagless defaults every process runs on.")
		echo "${prog}: log  ${run_dir}"

		# The report's verdict is this script's, and it is read off the
		# log alone: the driver republishes its whole account of the run
		# into it, so the analyzer needs no console and a console that
		# did not come back costs the verdict nothing.
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
