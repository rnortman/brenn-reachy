#!/usr/bin/env bash
#
# Put the built motion payload on a device, and bring a run's log back.
#
#   tools/deploy-motion.sh <host> --push [--stale-ok]
#   tools/deploy-motion.sh <host> --run <dir>
#   tools/deploy-motion.sh <host> --tour <dir>
#   tools/deploy-motion.sh <host> --probe <dir> <motion>
#   tools/deploy-motion.sh <host> --fetch <dir>
#   tools/deploy-motion.sh <host> --speech <dir>
#   tools/deploy-motion.sh <host> --speech-preflight
#   tools/deploy-motion.sh <host> --speech-fetch <dir>
#   tools/deploy-motion.sh <host> --record <dir>
#   tools/deploy-motion.sh <host> --record-preflight
#   tools/deploy-motion.sh <host> --record-fetch <dir>
#
#   --push       rsync the payload into the unit's RAM and create the directory
#                the logger writes into. Refuses a payload older than the newest
#                commit to the workspace, or one whose copy of either
#                out-of-tree member — the audio device's binary, the site's
#                speech configuration and each credential file that
#                configuration names — is older than the source it was staged
#                from, and refuses while anything else on the
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
#   --tour       `--run`'s chain, driven by the library tour instead of the wake
#                gesture: the intent source asks for every motion in the
#                committed name table at recorded pace and then stops the
#                launcher itself, so the run is as long as the library rather
#                than as long as a budget. The `timeout` around the launcher
#                stays as a backstop at the budget the tour computes here, and a
#                run that reaches it is a failure. The records are fetched
#                whatever ended the run — they are the point of it — and judged
#                by `library_tour_report` against the table the run was asked
#                for, which the fetch writes into the run directory. The tour is
#                the library's content: the `probe/` instruments are left out of
#                it, because a tour is what the recorded fixtures come off.
#                Several minutes of motion with nobody at the machine, so the
#                space around it has to be clear for the whole run.
#   --probe      `--tour`'s chain over one named motion instead of the library:
#                one script, one arrival per pose the motion steps to, and the
#                same fetch and verdict. It is how a `probe/` instrument is
#                played -- a step goal held long enough for a hold to be judged
#                -- and the motion has to be one the committed name table
#                holds. About a minute of motion with nobody at the machine.
#   --fetch      copy the run's `.olog` directories back to a local directory,
#                under a name stamped with the moment they were fetched so a
#                session's runs accumulate rather than overwrite. Refuses a fetch
#                that brought no records rather than reporting over nothing.
#   --speech     start the production launcher config — the voice host and the
#                audio device beside the motion stack — with no budget at all,
#                and stop when the operator does. Preflights the staged speech
#                configuration here (it must be in the payload, and
#                `reachy_host --check` must pass over it with the payload root
#                as its working directory) and the unit's side of it there (the
#                pod's link credentials present, and every speech service the
#                configuration names reachable *from the robot*). Refuses a
#                stdin that is not a terminal: a run with no budget is ended by
#                the person watching it, so one that cannot receive a ^C is not
#                started. Fetches and reports however the launcher ended —
#                which of the ways it did is the report's question, not this
#                script's.
#   --speech-preflight  the terminal refusal above, alone and before anything
#                else: `make speech-run` provisions the unit and builds and
#                pushes a payload, and none of that is worth doing for a run
#                that will be refused for a session property knowable first.
#                Touches nothing, on either machine.
#   --speech-fetch  what --fetch is to --run: bring a speech run's records back
#                under the `speech-log-` name, for the run whose terminal died
#                or whose report is wanted a second time.
#   --record     start the recording launcher config — the pose recorder and the
#                voice host beside the audio device, with no driver, no control
#                process and no logger at all — and stop when the operator does.
#                The session is the machine at the Minimum Risk Condition with a
#                person's hands in the linkage: the recorder reads Present
#                Position and writes to no register, and nothing this config
#                starts can arm anything. `--speech`'s chain and `--speech`'s
#                preflights, over the recording session's own speech
#                configuration and with the bench's configuration required
#                beside it, and both consoles tailed onto the operator's
#                terminal rather than the host's alone.
#   --record-preflight  the refusals answerable before anything is
#                provisioned, built or pushed, for the reason
#                `--speech-preflight` exists: the terminal, and the recording
#                session's two configurations and their agreement, asked of the
#                operator's own files. The staged copies are what `--record`
#                refuses on.
#   --record-fetch  bring a recording session's records back under the
#                `record-log-` name, for the session whose terminal died or
#                whose document is wanted a second time.
#
# A fetch brings back two things under one stamp, named for the kind of run it
# came off — `motion-log-<stamp>` for a budgeted motion run, `tour-log-<stamp>`
# for a library tour, `probe-log-<stamp>` for one motion played on its own, and
# `speech-log-<stamp>` for a supervised speech one and `record-log-<stamp>` for
# a pose recording session, so a session's kinds of
# records sit side by side and say which is which. Under that name: the
# records the analyzer judges — with `provenance.txt` at its root naming the
# build that recorded them — and a `.console` directory of the same name beside
# it, holding the console output of everything the launcher started. A run adds
# its own console stream and the unit's clock discipline, read before and after,
# to the second of those. The motion analyzer reads none of it: the log is
# self-contained, and everything the driver counts about itself is republished
# into it. The speech analyzer reads both sides — the console for what was asked
# of the head and how the process fared, the records for what the machine did
# with it. The
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
# launcher's arguments. `docs/bench-runbook.md` is the procedure.
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
# that failed halfway or a stage somebody edited by hand. Five are the production
# launcher config's; the sixth is the intent source `--run` starts ahead of it.
#
# `reachy_pod` is among them on the same terms as the rest, though it is
# compiled in brenn-pod's container rather than by this tree's build: what this
# list is about is what the launcher will look for on the unit, and the launcher
# does not care which build produced a file.
binaries=(
	reachy_motord
	reachy_host
	reachy_pod
	reachy_ask
	cogs/robot_clk_exe
	simplelaunch
)

# The payload's shared objects: its own class, and not among the binaries,
# because a refusal that calls a library an executable sends whoever reads it
# looking for the wrong mistake, and because nothing here has any reason to care
# what mode the file was staged with. Checked for the same reason the binaries
# are: the voice host resolves `libonnxruntime.so.1` through an `$ORIGIN`
# runpath at exec, so a payload without it is a host that dies at start with a
# loader message and no narration at all.
shared_objects=(libonnxruntime.so.1)

# The payload's launcher configs, at the payload root, checked with the binaries
# and for the same reason: `--run` names one of these two below and the launcher
# resolves it against the payload root, so a payload staged by an older build --
# or edited by hand -- would take a config the launcher cannot find onto a unit.
# Both are checked on every push, because a deploy without `--run` is what puts
# the production config on the unit. What `--run` names is asked about again on
# the unit itself, ahead of the wipe, because a push and a run are separate
# invocations. Not executables, so what is asked of them is that they are files.
launch_configs=(robotcpu.textproto robotcpu_harness.textproto)

# The wake and VAD weights, at the payload-relative paths the host's speech
# configuration names them by. Checked here rather than left to the host,
# because they are the only payload members the build fetches from the network:
# a build behind a proxy that answered a download with an error page produces a
# payload that stages and pushes and a wake gate that fails at its first
# inference, on a unit, with the operator watching a head that never moves.
# Files, not executables, and their contents are the build's business -- every
# one of them is fetched against a digest.
models=(
	models/oww/melspectrogram.onnx
	models/oww/embedding_model.onnx
	models/oww/hey_jarvis_v0.1.onnx
	models/silero/silero_vad.onnx
)

# One class of payload member, checked before anything is pushed.
#
#   require_members <noun> <name>...
#
# One function rather than a loop per class: the payload grows a class per slice
# -- four of them now -- and a copied loop is where the noun of the class it was
# copied from survives into a refusal about something else. The noun is the
# operator-facing half of that refusal, so it is what a caller states.
#
# What is asked of a member follows from its noun: an executable has to be one,
# and every other class -- a shared object the loader opens, a config, a weight
# file -- has to be a file and nothing more. Nothing here depends on the mode a
# member was staged with unless the launcher is going to exec it.
require_members() {
	local noun=$1 name path
	shift
	for name in "$@"; do
		path="${payload}/${name}"
		case "$noun" in
		executable) [ -x "$path" ] ;;
		*) [ -f "$path" ] ;;
		esac || die \
			"the payload at ${payload} has no ${noun} ${name}" \
			"Rebuild it: make motion-build"
	done
}

# The configuration files a run carries home beside its records, at their
# payload-relative paths.
#
# Three, and they are also the only paths an experiment overlay may write: they
# are the files a run can be varied by -- the profile the residual is judged
# against, the servo gains, and whether the tracking detector was armed -- and
# the files an analyzer reads. `session_params.textproto` and
# `motord_params.textproto` are pinned by the scenario suite's parameter check
# and read by no analyzer, so they join this list when something reads them.
#
# The overlay is a tuning knob and not a way to push arbitrary payload members,
# which is what makes the list an allowlist rather than a hint: a path outside
# it is refused.
run_config_files=(
	cogs/servo_profile.textproto
	cogs/servo_gains.textproto
	cogs/mover_params.textproto
)

# The directory of experiment configuration to lay over the staged payload, or
# empty for none. The operator's, out of `.local/reachy.conf` by way of the
# Makefile, because which experiment is being run is a property of the session
# and not of the tree.
#
# Trailing slashes are stripped, keeping a bare `/` as itself: the paths below
# are made by cutting this prefix off `find`'s output, which prints one slash
# between the directory and the rest, and shell completion writes the trailing
# one.
experiment_dir=${REACHY_EXPERIMENT_DIR:-}
while [ "${#experiment_dir}" -gt 1 ] && [ "${experiment_dir%/}" != "$experiment_dir" ]; do
	experiment_dir=${experiment_dir%/}
done

# Lay the experiment overlay over the staged payload.
#
#   overlay_experiment
#
# Every file under `${experiment_dir}` at its payload-relative path, refusing
# any path this payload does not accept an overlay for. Into the staged payload
# before the stamp is written and before the rsync, so the digests the stamp
# records are the digests of what lands on the unit, and so `--delete` cannot
# take the overlaid copy back off.
#
# The staged payload is a build output and this overwrites members of it, which
# is deliberate and is why every overlaid file is named on the console with its
# digest: the next build restores the tree's copy, and until then the operator
# has been told which files on this unit are not the tree's.
#
# The contents are not checked here. A malformed file is refused by the cog that
# binds it, at start, on the unit, by the real loader and before anything is
# commanded -- and a host-side key scan would be a hand-maintained third
# statement of a schema in the language least able to express it, whose false
# refusals would block pushes the payload would have accepted. The path check is
# the half shell states correctly, so the path check is what is here.
#
# Symlinks are followed and count as files: a rung kept as its own file and
# pointed at by `cogs/servo_profile.textproto -> ../rungs/r2.textproto` is how a
# gains or profile ladder is run, and a walk that skipped it would push the
# tree's configuration under an overlay's name -- silently, since a run that
# really did run the tree's files prints no difference note. A link with no
# readable file behind it is refused rather than skipped, and an overlay that
# contributed nothing at all is a mistake and not a configuration.
overlay_experiment() {
	local file relative allowed name overlaid=0
	[ -n "$experiment_dir" ] || return 0
	[ -d "$experiment_dir" ] ||
		die "REACHY_EXPERIMENT_DIR names ${experiment_dir}, which is no directory." \
			"Unset it to push the tree's own configuration."
	while IFS= read -r file; do
		relative=${file#"${experiment_dir}"/}
		allowed=no
		for name in "${run_config_files[@]}"; do
			[ "$relative" = "$name" ] && allowed=yes
		done
		[ "$allowed" = yes ] ||
			die "the experiment overlay states ${relative}, which is not one of the files a run may vary." \
				"Those are: ${run_config_files[*]}"
		[ -f "$file" ] ||
			die "the experiment overlay's ${relative} is no readable file, so there is nothing to overlay." \
				"A link with nothing behind it is the usual cause."
		install -m 0644 -D -- "$file" "${payload}/${relative}"
		echo "${prog}: overlay: ${relative} $(sha256_of "${payload}/${relative}")" >&2
		overlaid=$((overlaid + 1))
	done < <(find -L "$experiment_dir" \( -type f -o -type l \) | sort)
	[ "$overlaid" -gt 0 ] ||
		die "REACHY_EXPERIMENT_DIR names ${experiment_dir}, which holds none of the files a run may vary." \
			"Those are: ${run_config_files[*]}"
}

# The sha256 of one file, the digest alone.
sha256_of() {
	sha256sum -- "$1" | cut -d' ' -f1
}

# The logger configuration, read out of the staged payload rather than out of the
# tree. The values in force on the device are the ones that were staged, and an
# edit to the checked-in file after the last build is not among them: the
# freshness refusal does not catch uncommitted edits. Every value printed or
# created here must describe the artefact that will run, and a missing payload is
# a refusal that says to build one.
logger_config="${payload}/cogs/robot_logger.textproto"

# The launcher config `--run` starts, at the payload root, and where the launcher
# puts the console output of everything it starts. Both are named in the start
# command below and nowhere else here: the config names the processes, their
# arguments and their working-directory-relative executables, so this script has
# nothing left to say about them.
#
# The harness twin, not the production config the payload also carries. `--run`
# starts `reachy_ask` itself, ahead of the launcher, because the intent source
# has to hold the narration port before the control process narrates; the voice
# host binds that same port and addresses scripts to the same one, and the
# production config names it. Starting both would leave the run's verdict to
# whichever of them the kernel gave the bind to. A deploy without `--run`
# pushes both configs and starts neither, which is where the production one
# is used.
#
# The log directory is on the same tmpfs as the payload and the records; the
# launcher creates it, a run empties it first, and a fetch brings it back.
launch_config=robotcpu_harness.textproto
launch_logs="${store_mount}/logs/launch"

# The launcher config a speech run starts: the production one, which names the
# voice host and the audio device beside the motion stack. The whole point of
# `--speech` is that this is the arrangement a person talks to, so it starts no
# `reachy_ask` — the host owns the narration port here — and takes no budget.
speech_launch_config=robotcpu.textproto

# The launcher config a recording session starts: the third composition, which
# holds the audio device, the voice host on the recording session's own speech
# configuration, and the pose recorder — and no driver, no control process and
# no logger. Nothing it starts writes a servo register, which is the whole of
# why the session is safe to put a person's hands into.
record_launch_config=robotcpu_record.textproto

# The file the launcher gives the voice host's console, inside the log dir. The
# launcher redirects every app's stdout and stderr into `<name>_<index>.log`
# before exec, so nothing the host prints reaches the operator's terminal on its
# own; a speech run tails this one onto the pty for exactly that reason. The
# index is 0 because the log dir is emptied at the start of every run.
voice_host_log=voice_host_0.log

# The pose recorder's console, in the same log dir and under the same rule: the
# JSON pose stream the analyzer reads is this file, and a recording session tails
# it beside the host's so the operator hears the read-back and sees the
# recorder's `started`, `refused`, `still` and `moving` lines.
recorder_log=recorder_0.log

# The pod's link credentials on the unit, written by brenn-pod's provisioning
# and read by the audio device at start. Checked here and never written: that
# file's format, its compiled-in path, and the key generation and secret
# delivery behind it are brenn-pod's, and a second writer of it in this repo
# would be the drift its own header warns about. Its absence is a pod that parks
# silently and a run that looks deaf, so a speech run asks first. A backstop
# rather than a prompt: `make speech-run` provisions the file before it gets
# here, so the ways to reach this refusal are a direct invocation of this script
# and a unit that lost its tmpfs in between.
audio_conf="${store_mount}/conf/audio.conf"

# What the reachability preflight asks each speech service for, on the unit.
# The OpenAI-compatible listing route: a plain GET that a speaches instance
# answers without doing any work.
#
# The question is reachability and not routing, so the probe asks for a response
# rather than for a *successful* one: a build that serves no listing route
# answers 404, and a 404 from the robot is the address being reachable. Only a
# DNS, connect or TLS failure — or the five-second deadline — is an unreachable
# service. The path is appended to the URL with its trailing slashes taken off
# first, because a configured `http://host:8000/` would otherwise ask for
# `//v1/models`.
service_probe_path=/v1/models

# The host binary the staged speech configuration is checked with, built here in
# the default configuration and run with the payload root as its working
# directory: what it resolves then is exactly what the unit will resolve, since
# the launcher starts the host from that same root.
check_target=//crates/reachy-host:reachy_host

# Every refusal a run carries a code of its own for, in one table.
#
# One family and one place, because both run modes emit these and both read them
# back: two copies could disagree about what a number means while the runbook
# tabulates one set. 3 and 4 are the bus probe's, 255 is ssh's, and 124-127/137
# are timeout's, so a code added here has to miss all of those.
#
# 5 to 8 are the preparation chain's, emitted on the unit by both run modes; 9,
# 10 and 11 are the speech run's local refusals and are what the script itself
# exits with; 12 and 13 are the speech run's own remote steps and reach an
# operator as `die`'s 1, the way 5 to 8 do. 14 to 16 are the recording session's
# local refusals, which is why they are three: which of the two configurations a
# payload is missing, and a pair that would put the pod and the host on
# different addresses, are three different things to go and fix.
rc_no_stamp=5
rc_stamp_unstaged=6
rc_post_wipe=7
rc_no_launch_config=8
rc_no_speech_config=9
rc_no_tty=10
rc_check_refused=11
rc_no_audio_conf=12
rc_service_unreachable=13
rc_no_record_config=14
rc_no_bench_config=15
rc_record_config_disagreement=16

# What the remote chain prints when it is about to exec the launcher.
#
# The chain's own codes are small integers and so are a launcher's: a launcher
# that exits 5 would otherwise be read as a payload with no provenance stamp,
# and the run's records — of a supervised session, which is not repeatable —
# would never be fetched. This line is printed after the last step that can
# refuse, so its presence in the captured console says the exit belongs to the
# launcher whatever its number is, and its absence says the chain refused.
launch_sentinel=---brenn-launcher-starting

# Whether the run reached its launcher, from the console the pty captured.
#
#   launcher_reached <console file>
launcher_reached() {
	grep -qF -- "$launch_sentinel" "$1" 2>/dev/null
}

# The preparation chain's refusals, shared by both run modes.
#
#   chain_refusal <rc> <launch config> <fetch flag> <lines introducing the push,
#                 for the missing-config arm...>
#
# One copy of the four messages, because one copy of the chain emits them: the
# stamp-parking dance across the log-root wipe exists so that a refusal can say
# truthfully whether the previous run's records are still on the unit, and a
# second copy of that reasoning is a second place to get it wrong. What differs
# between the modes is which launcher config was looked for and which fetch flag
# recovers the records, so those are the parameters.
#
# Returns without saying anything for a code that is not one of the four; the
# caller has already established that the launcher was never reached.
chain_refusal() {
	local rc=$1 config=$2 fetch=$3
	shift 3
	case "$rc" in
	"$rc_no_launch_config")
		# Asked before the stamp and before the wipe, so this refusal
		# has started nothing and emptied nothing.
		die "${host}'s payload has no ${config}, so this run's launcher config is not on the unit." \
			"$@" \
			"    ${prog} ${host} --push" \
			"Nothing was started and nothing was emptied, so this unit's log root still holds" \
			"whatever the previous run left there:" \
			"    ${prog} ${host} ${fetch} <records-dir>"
		;;
	"$rc_no_stamp")
		# The stamp is asked about before the log root is emptied, not
		# after: this refusal fires on a payload pushed by an older
		# script or landed before a reboot cleared the tmpfs, and the
		# previous run's records are often still sitting in that log
		# root unfetched.
		die "${host}'s payload carries no provenance stamp, so this run's records could not name their build." \
			"Nothing was started and nothing was emptied. The push writes that stamp, so push again:" \
			"    ${prog} ${host} --push"
		;;
	"$rc_stamp_unstaged")
		# The stamp is there — the step before proved it — and staging
		# it failed. Staging runs before the wipe, so this refusal has
		# destroyed nothing.
		die "${host}'s payload carries its provenance stamp, but copying it to ${staged_provenance} failed," \
			"so this run's records could not have named their build. A full tmpfs or a" \
			"permission on the payload store is the usual cause; ${host}'s own error is above." \
			"Nothing was started and nothing was emptied, so this unit's log root still holds" \
			"whatever the previous run left there:" \
			"    ${prog} ${host} ${fetch} <records-dir>"
		;;
	"$rc_post_wipe")
		# One of the four steps that run after the wipe began: the log
		# root's own mkdir, the stamp's move into it, the launcher
		# console directory's wipe and recreate, or the cd into the
		# release. What they have in common is the one thing this arm
		# can say, and it is the thing the operator needs: the previous
		# run's records are gone.
		#
		# It does not send anybody to the launcher's console directory:
		# on one of these paths that directory is what could not be
		# made, and on the others the launcher never ran to write
		# anything into it.
		die "preparing ${host} for the run failed after the log root wipe had begun (exit ${rc})." \
			"Treat the previous run's unfetched records on the unit as gone." \
			"The launcher was not started, so nothing moved." \
			"${host}'s own error is above; a full or read-only payload store is the usual cause."
		;;
	esac
}

# A refusal carrying a code of its own.
#
#   refuse <code> <headline> [detail...]
#
# `die` is this with the code fixed at 1, which is right for everything a person
# reads off the screen. A speech run's local refusals are also read by whatever
# started it, so each says which refusal it was in its status as well as in its
# words.
refuse() {
	local code=$1
	shift
	echo "${prog}: $1" >&2
	shift
	local line
	for line in "$@"; do
		echo "    ${line}" >&2
	done
	exit "$code"
}

# How long the run is given before the launcher is stopped. The harness
# gesture ends nineteen seconds after the edge receives it — its arming offset,
# the eight-second hold the stillness section is judged over, and the closing
# stow — with commissioning's bus survey of about five seconds before it and the
# release's four after: twenty-eight. The host harness budgets thirty-three for
# the same sum, and this one carries a few seconds more of margin again because
# it is talking to a real serial bus whose transactions retry. Nothing about the
# run is judged by the clock — the analyzer reads the records — so the budget
# only has to be long enough. The gesture's end is a number in another file, so
# the sum is checked against the shipped one by tools/deploy-motion.test.sh: a
# gesture growing without this following it is red there rather than a launcher
# stopped mid-gesture.
run_seconds=36

# The library the tour plays, in the two spellings a tour needs it in: the
# committed one here, which the budget is computed from and the analyzer judges
# against, and the payload-relative one the sender is given on the unit, where
# tools/build-motion.sh stages the same file and the launcher's working
# directory is the release root.
#
# One file in both places by construction — the payload's copy is built from
# this one — so a tour whose plan and whose verdict came from the workstation
# still describes the run the unit performed, and a payload staged before the
# last `make clip-config` is the freshness refusal's business rather than this
# script's.
tour_names="${repo_root}/cogs/clip_library.names.json"
tour_names_staged=cogs/clip_library.names.json

# What the table this run was asked for is called inside the fetched run
# directory.
#
# A run plays a selection over the committed library -- the content for a tour,
# one motion for a probe run -- and the analyzer's every finding is about a
# motion that should have been asked for, so it is handed the selection rather
# than the library. The sender makes that selection for the plan it runs and
# prints the same one here for the verdict, so the two cannot disagree; the copy
# lands beside `config/` in the run directory, which is what makes a fetched run
# say for itself what it was asked to play.
asked_names_name=asked.names.json

# The intent source's own label, built and run here for the tour's backstop
# budget alone.
#
# The budget is the sender's arithmetic over the library — the plan's clock plus
# the allowances — so it is asked of the sender rather than computed here: the
# plan and the backstop come from one function, and two of them could disagree
# while the run rode the wrong one. A host build, like the analyzer's: nothing
# about printing a number runs on the device.
ask_target=//crates/reachy-ask:reachy_ask

# The name of the intent source in the payload, and where its console goes.
#
# It is not a launcher app and cannot be: it binds the narration port, and the
# composition starts narrating on its first execution, so it has to be running
# before the launcher is. Started here, ahead of the launcher, and stopped with
# it — a run's verdict is the analyzer's over the fetched records, so what this
# console holds is why a run went the way it did rather than the verdict itself.
#
# A tour is the exception: there the sender ends the run, so its status is the
# run's whenever the launcher's is 0, and its last line is what a refusal
# quotes.
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
# Coupled to brenn-pod's deploy-reachy-pod.sh, which reads this path and
# `provenance_name` (`tools/lib.sh`) to guard against replacing a robot's payload
# with a pod-only one. A rename here is invisible there, so both names are pinned
# by a case in this script's self-test; changing either means changing both
# repos.
release="${store_mount}/releases/motion"

# The store name to fetch when the staged configuration names none, and the
# spelling every site has used.
record_dir_fallback=framelogs

# Where the speech pipeline writes the audio it heard, when it is recording,
# and where that name came from — as `<device path>\t<source>`.
#
#   record_store <payload-relative speech configuration>
#
# The host resolves its store against its own working directory, which the
# launcher makes the payload root, so the store is the release directory plus
# the `[record] dir` the deployed speech configuration names. That name is the
# operator's, so it is read out of the staged configuration rather than spelled
# here: the build and the fetch cannot disagree about where the audio is, the
# way they cannot about where the payload's credentials are.
#
# Which configuration is the caller's, because the payload carries one per voice
# host entry and a session's audio was written by the host that ran: the site's
# for a speech run, the recording session's own for a `--record` one. A fetch
# reading the other file would look where a host that did not run would have
# written.
#
# A fetch with no staged configuration in hand — `--speech-fetch` after a
# rebuild, or before one — has nothing to read and falls back to the name every
# site uses. The source travels with the path so the message can say which of
# the two an empty store means.
#
# Read through a command substitution, as `toml_table_value` is.
record_store() {
	local config_path=$1
	local config="${payload}/${config_path}" dir=""
	if [ -f "$config" ]; then
		dir=$(toml_table_value "$config" record dir) || exit 1
	fi
	if [ -n "$dir" ]; then
		# An absolute store is what the preflight refuses, and it is still
		# where the audio would land if one ran: read as written, so the
		# fetch looks where the host would have written.
		case $dir in
		/*) printf '%s\t%s\n' "$dir" "the staged ${config_path} names it" ;;
		*) printf '%s\t%s\n' "${release}/${dir}" "the staged ${config_path} names it" ;;
		esac
	else
		printf '%s\t%s\n' "${release}/${record_dir_fallback}" \
			"no staged configuration names a store, so this is the fallback name"
	fi
}

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
	die "usage: ${prog} <host> --push [--stale-ok]|--run <dir>|--tour <dir>|--probe <dir> <motion>|--fetch <dir>|--speech <dir>|--speech-preflight|--speech-fetch <dir>|--record <dir>|--record-preflight|--record-fetch <dir>"
}

# Refuse a value that is not a plain path or name, saying what it was.
#
#   plain_name <what it is, in the refusal's words> <value>
#
# The one screen every value pasted into a command goes through, wherever it
# came from: the staged configuration below, or an argument this script was
# invoked with. A value with a space or a metacharacter means something
# different at each of those sites -- one of them builds a remote command run as
# root, one a remote rsync path the far end re-parses -- and one refusal is
# cheaper than quotings that have to agree. Refused before anything is pushed,
# because a refusal after the push is a unit already carrying the payload.
plain_name() {
	local label=$1 value=$2
	case $value in
	*[!A-Za-z0-9/_.-]*)
		die "${label} is '${value}', which is not a plain path or name." \
			"These values are pasted into a remote command and a local one, so only" \
			"[A-Za-z0-9/_.-] is accepted here."
		;;
	esac
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
# Every value goes through `plain_name`, so the callers below can interpolate
# what they get plainly.
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
	plain_name "${logger_config}'s ${field}" "$value"
	echo "$value"
}

# Copy a run's records off the unit into a directory of their own, and echo
# where they landed.
#
#   fetch_records <destination> <device log root> <name prefix> <mode>
#
# The prefix says which kind of run these records came off — `motion-log` for a
# budgeted motion run, `speech-log` for a supervised speech one. It is the
# caller's because the two accumulate in the same session and often in the same
# directory, and a name that does not say which is which is one nobody can pick
# a report for months later.
#
# The mode says which kind of run this is — `motion`, `speech` or `record` — and
# it decides two things.
#
# Whether a log root holding no records is a refusal. For a motion run it is:
# the verdict is arithmetic over the channel log, so there is nothing to report
# over. For the two supervised modes it is not, because their verdicts are read
# off a console, which is fetched after this: a run that died in the launcher's
# first seconds is exactly the run whose console explains it, and a refusal here
# would leave that file on tmpfs until the next run's wipe. A recording session
# writes no `.olog` at all — its composition holds no logger — so for it the
# absence is the ordinary case rather than a tolerated one, and nothing is said
# about it: a line printed after every session is one nobody reads by the third.
#
# And whether the recorded audio comes home, and out of which host's
# configuration the store is named. Only the two supervised modes record any,
# and an empty third directory beside every motion log would be one nobody can
# tell from a store that was lost.
#
# Everything it says goes to stderr: the caller reads the path off stdout.
fetch_records() {
	local dest=$1 log_root=$2 prefix=$3 mode=$4
	local stamp out part records_audio=no store_config="" olog=refuse
	case $mode in
	speech)
		records_audio=yes
		store_config=$speech_config_path
		olog=kept
		;;
	record)
		records_audio=yes
		store_config=$record_speech_config_path
		olog=none
		;;
	esac
	mkdir -p -- "$dest"
	stamp=$(date -u +%Y%m%dT%H%M%SZ)
	out="${dest}/${prefix}-${stamp}"

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
	# reason: rsync would merge into it, and a leftover .turns is
	# another run's clips under this run's turn numbers.
	local existing
	for existing in "$out" "${out}.console" "${out}.audio" "${out}.turns" "$part"; do
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
		if [ "$olog" = none ]; then
			# Nothing is said: a recording session's composition holds
			# no logger, so this is what every successful one looks
			# like. A warning here would be one an operator learns to
			# scroll past, and the speech run's is the line that says
			# its logger died.
			:
		elif [ "$olog" = kept ]; then
			echo "${prog}: nothing in ${log_root} on ${host} is a .olog with bytes in it;" \
				"keeping the fetch for its console" >&2
		else
			rm -rf -- "$part"
			die "the fetch from ${host}:${log_root} carries no .olog with anything in it." \
				"The run wrote no records. Either the logger never started, or it found no" \
				"channels: it reads the buffer directory and namespace out of the payload's" \
				"cogs/robot_logger.textproto, and every process is started with no pinion" \
				"flags at all, so those two values have to be the compiled-in defaults." \
				"Nothing was kept, so the next fetch is not merging into this one."
		fi
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

	# And the audio the pipeline heard, for a speech run only: a motion run
	# records none, and a fetch that made an empty third directory beside
	# every motion log would be a directory nobody can tell from one whose
	# store was lost.
	#
	# A third sibling rather than a member of the run directory, for the
	# reason the console is one: `run_directory` takes the newest directory
	# under the fetched root as the run, so a store landing inside would be
	# judged as one and refused for holding no .olog.
	#
	# The store is payload-relative, so it sits under the release directory
	# rather than the log root, and it is the one thing a fetch brings home
	# that the operator chose per site: a configuration recording nothing
	# leaves this empty, which is a line and not a refusal.
	if [ "$records_audio" = yes ]; then
		local audio="${out}.audio" store store_source
		store=$(record_store "$store_config") || exit 1
		store_source=${store#*$'\t'}
		store=${store%%$'\t'*}
		mkdir -p -- "$audio"
		if rsync -a -e "ssh -o BatchMode=yes" \
			"root@${host}:${store}/" "${audio}/"; then
			if [ -z "$(find "$audio" -type f -print -quit)" ]; then
				echo "${prog}: ${store} on ${host} holds no recorded audio" \
					"(${store_source}); the run's speech configuration is what" \
					"turns recording on" >&2
			fi
		else
			# The directory goes with the failure: an empty one left behind
			# is indistinguishable from a site that recorded nothing, and
			# the line saying which scrolls away.
			rmdir -- "$audio" 2>/dev/null || true
			echo "${prog}: no recorded audio fetched from ${host}:${store}" \
				"(${store_source})" >&2
		fi
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
#
# Beside the build it names the configuration: a `config_sha256=` line per file a
# run can be varied by, and the overlay directory that produced them, if any. The
# files themselves travel home in the log root's `config/` and are what the
# analyzers read; these lines are the push's own record of what it staged, so a
# fetched log says both what its configuration is and that nothing rewrote it
# between the push and the run.
stamp_provenance() {
	local into=$1 age_unchecked=$2
	local pushed_from dirty built commit commit_source brenn_pod reachy_pod name
	pushed_from=$(git -C "$repo_root" rev-parse HEAD 2>/dev/null) || pushed_from=
	[ -n "$pushed_from" ] ||
		die "this tree cannot state its own commit, so a push from it could not say which build ran." \
			"Every fetched records directory carries that commit, because a log is only" \
			"readable by the build that recorded it. Push from a checkout with history."
	built=
	brenn_pod=
	reachy_pod=
	if [ -f "${payload}/${build_commit_name}" ]; then
		built=$(sed -n 's/^commit=//p' -- "${payload}/${build_commit_name}")
		brenn_pod=$(sed -n 's/^brenn_pod=//p' -- "${payload}/${build_commit_name}")
		reachy_pod=$(sed -n 's/^reachy_pod=//p' -- "${payload}/${build_commit_name}")
	fi
	# A payload staged by a build that recorded no brenn-pod field: an older
	# build script. Nothing here can work either value out — this tree's
	# MODULE.bazel is where it stands now, not where it stood at the build, and
	# the pod binary carries no revision a reader here could ask it for.
	brenn_pod=${brenn_pod:-unknown}
	reachy_pod=${reachy_pod:-unknown}
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
#
# overlay names the directory of experiment configuration the push laid over the
# payload, or none. A config_sha256 line per file a run can be varied by, over
# the copy that was pushed: the same files are in config/ beside these records,
# so a digest that disagrees with one of them is a payload edited on the unit.
#
# brenn_pod is the other half of what built the voice host: the brenn-pod
# revision the payload's build resolved its speech crates from. A value starting
# overlay: means they came out of a working tree beside the building checkout
# rather than a published revision, so no revision names those binaries. unknown
# means the payload was staged by a build that recorded no such field.
#
# reachy_pod is the brenn-pod revision the audio-device binary was compiled from,
# with +dirty when that checkout held uncommitted changes. named means an
# operator handed the build a prebuilt artifact, whose revision nothing could
# ask. unknown means the payload was staged by a build that did not record it,
# which is also a build that did not hold this field and brenn_pod equal — so a
# run whose two fields name two revisions is such a payload.
commit=${commit}
commit_source=${commit_source}
brenn_pod=${brenn_pod}
reachy_pod=${reachy_pod}
pushed_from=${pushed_from}
dirty=${dirty}
age_unchecked=${age_unchecked}
pushed=$(date -u +%Y%m%dT%H%M%SZ)
overlay=${experiment_dir:-none}
STAMP
	for name in "${run_config_files[@]}"; do
		echo "config_sha256=${name} $(sha256_of "${payload}/${name}")" >>"$into"
	done
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

# Refuse a log root a run must not empty.
#
#   require_wipeable_log_root <log root>
#
# Both run modes empty this directory on the unit, as root, before the launcher
# starts. The value comes out of a configuration file the build staged, so what
# it is allowed to name is asked here rather than trusted there, and asked once
# for both modes: a second copy of these four questions is where one of them
# goes missing.
require_wipeable_log_root() {
	local log_root=$1
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
}

# The status the run's ssh came back with, read by the caller of the function
# below: a function cannot both stream a console and echo a number.
pty_status=0

# Run the launcher chain on the unit over a pty, streaming and keeping its
# console.
#
#   run_over_pty <remote command> <console file>
#
# A pty, because the launcher's console prints as it goes and a ^C here reaches
# the remote process group. ssh allocates one only when this stdin is a
# terminal; what a mode does about a stdin that is not one is the mode's, since
# a budgeted run without a ^C still ends and a supervised one never would.
#
# Teed as well as printed: what scrolls past an operator is the only place the
# launcher's own bookkeeping and the driver's counter summaries appear live, and
# a terminal scrollback is not a record. The pty is unaffected — ssh decides on
# one by this stdin, not by where its stdout goes.
#
# `pipefail` is off across the pipeline so that a failing run does not fail the
# script here: the status wanted is ssh's own, read out of PIPESTATUS before
# anything else resets it.
#
# Both statuses are taken in the same breath, on either branch of the AND-OR
# list, for two reasons: `set -e` does not act on a command that is part of one,
# and the console copy must not be able to end the run's records. With
# `pipefail` off the pipeline's own status is `tee`'s, so a `tee` that fails --
# the records filling mid-stream is the realistic way -- would otherwise exit
# the script after the run had happened and before the fetch, with the only copy
# of the records still on the unit's tmpfs.
run_over_pty() {
	local remote=$1 console=$2
	local ran=()
	set +o pipefail
	ssh -t -o BatchMode=yes "root@${host}" "$remote" 2>&1 |
		tee -- "$console" &&
		ran=("${PIPESTATUS[@]}") || ran=("${PIPESTATUS[@]}")
	set -o pipefail
	pty_status=${ran[0]}
	if [ "${ran[1]:-0}" -ne 0 ]; then
		echo "${prog}: the console copy failed (tee exited ${ran[1]}): the run itself" >&2
		echo "${prog}: is unaffected and its records are fetched below, but" >&2
		echo "${prog}: ${console} is short or missing." >&2
	fi
}

# File a run's host-side captures beside the records it fetched, and echo where
# they went.
#
#   file_captures <aside directory> <fetched records directory>
#
# The host-side evidence joins the device's own under the one name that says
# which fetch it belongs to. One file at a time and per-file best-effort: each
# of these is captured best-effort in the first place, and a run's records are
# not worth losing over a capture that did not land.
#
# Everything it says goes to stderr: the caller reads the console directory off
# stdout, as it reads the records directory off `fetch_records`.
file_captures() {
	local aside=$1 out=$2
	local console="${out}.console" captured
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
	echo "$console"
}

# Run a chain over a pty, with the host-side evidence captured around it.
#
#   launch_and_capture <remote chain> [<what a lost ^C costs, one line per arg>]
#
# Sets `aside`, the scratch directory the captures wait in until there is a
# fetched records directory to file them beside, and `rc`, the chain's own exit
# status. Every run mode captures the same four things — the clock either side,
# the host's own facts, and the console stream — because they are what a run
# that failed is diagnosed from, and a mode that captured three of them would
# cost an operator the evidence for the run that just failed.
#
# A pty: the launcher's console prints as it goes and a ^C here reaches the
# remote process group. A Ctrl-C aborts the caller before its fetch — the
# driver's wind-down leaves the machine de-torqued, and the run is re-run.
#
# ssh allocates one only when this stdin is a terminal, and proceeds without it
# otherwise, which silently costs the ^C: an operator watching a run they cannot
# interrupt is the one thing the eyes-on rule assumes they can do, so the loss
# is said out loud rather than left to a warning from ssh. What that loss costs
# differs by mode, so the caller says it in its own words; a caller that passes
# none is a mode nobody watches for a ^C.
launch_and_capture() {
	local remote=$1 line
	shift
	aside=$(mktemp -d)
	trap 'rm -rf -- "$aside"' EXIT
	capture_clock "${aside}/clock-before.txt"
	capture_host_facts "${aside}/host-facts.txt"
	if [ "$#" -gt 0 ] && [ ! -t 0 ]; then
		echo "${prog}: stdin is not a terminal, so ssh allocates no pty and a ^C here" >&2
		for line in "$@"; do
			echo "${prog}: ${line}" >&2
		done
	fi
	rc=0
	run_over_pty "$remote" "${aside}/run-console.log"
	rc=$pty_status
	capture_clock "${aside}/clock-after.txt"
}

# The backstop budget for a run over the committed library, in whole seconds.
#
#   tour_budget [motion]
#
# Asked of the sender, which is the only thing that knows the plan: it reads the
# same name table the run plays, makes the same selection -- the content
# library, or the one `motion` -- and prints the seconds its own clock adds up
# to. A host build, run from its runfiles tree, so the sidecar is named
# absolutely.
#
# Everything the answer is checked for is what it is about to be pasted into: a
# `timeout` argument and the messages that quote it. A build that printed
# nothing, or printed bazel's own chatter, would otherwise become a `timeout`
# with no duration and a launcher that is never stopped.
#
# Everything it says goes to stderr: the caller reads the number off stdout.
tour_budget() {
	local motion=${1:-} seconds
	local args=(--tour-budget "$tour_names")
	[ -z "$motion" ] || args+=(--motion "$motion")
	require_tour_names
	echo "${prog}: asking ${ask_target} for the run's backstop budget" >&2
	seconds=$("$bazel" run "${build_flags[@]}" -- "$ask_target" \
		"${args[@]}") || die \
		"the tour's budget could not be computed, so the run has no backstop." \
		"That is a build failure, a name table the sender refused, or a motion it does not hold;" \
		"its output is above."
	case $seconds in
	'' | *[!0-9]* | 0)
		die "${ask_target} answered '${seconds}' for the tour's budget, which is not a number of seconds." \
			"The backstop is pasted into the unit's timeout, so it is not guessed at here."
		;;
	esac
	echo "$seconds"
}

# Refuse a tree with no committed name table.
#
#   require_tour_names
#
# The sender reads the staged copy on the unit and this script reads the
# committed one; a tree with none has no library to select from, and the
# refusal comes before anything is pushed or started.
require_tour_names() {
	[ -f "$tour_names" ] || die \
		"no clip name table at ${tour_names}, so the tour has no library to play." \
		"It is generated beside the library it describes: make clip-config"
}

# Write the table this run was asked for into the fetched run directory.
#
#   asked_table <run directory> [motion]
#
# The sender's own selection over the committed library, printed as a names
# sidecar: the content library, or the one `motion` a probe run played. It is
# what the analyzer is then handed, so the verdict is taken over the motions the
# plan was built from rather than over a second reading of the library, and the
# run directory keeps it beside the `config/` copy for whoever reads the fetch
# later.
#
# After the fetch rather than before the run, because the run directory is named
# by the logger: the same selection made by the same sender over the same file
# is what makes the copy the plan's own table and not a guess at it.
asked_table() {
	local run_dir=$1 motion=${2:-}
	local args=(--tour-table "$tour_names")
	[ -z "$motion" ] || args+=(--motion "$motion")
	echo "${prog}: asking ${ask_target} for the table this run played" >&2
	# The unusable file is taken back off before the refusal: an empty or
	# half-written table left in a run directory reads later as the table the
	# run was judged against, and nothing was.
	if ! "$bazel" run "${build_flags[@]}" -- "$ask_target" "${args[@]}" \
		>"${run_dir}/${asked_names_name}"; then
		rm -f -- "${run_dir}/${asked_names_name}"
		die "the table this run played could not be printed, so its records have nothing to be judged against." \
			"The records are at ${run_dir} and its output is above."
	fi
	if [ ! -s "${run_dir}/${asked_names_name}" ]; then
		rm -f -- "${run_dir}/${asked_names_name}"
		die "${ask_target} printed no table for this run, so there is nothing to judge it against." \
			"The records are at ${run_dir}."
	fi
	echo "${run_dir}/${asked_names_name}"
}

# The sender's last word, out of the console a tour brought home.
#
#   ask_last_line <fetched console directory>
#
# A tour that ended badly ended on the sender's own line — the ending it
# reached, or the quit the launcher did not take — and that line is what a
# refusal here quotes. The console is fetched best-effort like every other one,
# so its absence is a sentence rather than a failure: the records came back
# either way and the analyzer still has them.
ask_last_line() {
	local file="${1}/${ask_console_name}"
	if [ -s "$file" ]; then
		tail -n 1 -- "$file"
	else
		echo "its console did not come back, so it said nothing here"
	fi
}

# Play the library, or one motion of it, and judge what came back.
#
#   play_from_library <records directory> [motion]
#
# `--run`'s chain with four substitutions: the sender is told to play a
# selection over the committed library instead of the wake gesture, the
# `timeout` carries the backstop the sender computed instead of a fixed budget,
# the ssh status is the sender's whenever the launcher's is 0, and the records
# are fetched before the run is judged.
#
# With no `motion` it is the content tour, which is the library minus its
# `probe/` instruments and takes several minutes. With one it is a probe run:
# one script, one motion, about a minute. Everything else -- the chain, the
# refusals, the fetch, the analyzer -- is the same, because what differs between
# the two is only which motions the plan holds.
play_from_library() {
	local dest=$1 motion=${2:-}
	local budget log_root remote out console run_dir asked
	# What the run is called in its own refusals, and what its fetch is named
	# after: a records directory says which kind of run it came off, and an
	# operator reading a refusal is told which run to look at.
	local subject="the tour" article="a library tour" prefix="tour-log"
	if [ -n "$motion" ]; then
		subject="the probe run"
		article="a probe run"
		prefix="probe-log"
	fi
	dest=$(absolute_path "$dest")
	require_bazel "the run's budget, table and report"
	budget=$(tour_budget "$motion")
	log_root=$(config_string log_root_dir)
	require_wipeable_log_root "$log_root"

	remote=$(launch_chain "$log_root")
	# The intent source, ahead of the launcher and in the background as
	# --run's is, and told which motions to play: the sidecar is named
	# relative to the release directory the chain has already cd'd into,
	# where the build staged it, and the selection over it is the sender's
	# own -- the content library, or the one motion named here. No
	# --resting-timeout, so commissioning uses the sender's own default, and
	# no --run-window: a plan knows its own end.
	remote="${remote}; ./${ask_binary} --tour ${tour_names_staged}"
	[ -z "$motion" ] || remote="${remote} --motion ${motion}"
	remote="${remote} >${launch_logs}/${ask_console_name} 2>&1 &"
	remote="${remote} ask=\$!"
	# The budget is the backstop and not the stop: the sender ends the run
	# itself, through the launcher's own quit API, and a run that reaches
	# this timeout is one where that did not happen. --kill-after is the
	# wedged-launcher grace --run's is.
	remote="${remote}; timeout --signal=INT --kill-after=10"
	remote="${remote} ${budget} ./simplelaunch ${launch_config}"
	remote="${remote} --logdir ${launch_logs}"
	# The sender's status is this run's whenever the launcher's is 0, which
	# is where --run and this differ: the launcher returns 0 both when the
	# sender quit it and when it fell over on its own, and only the sender
	# knows which of those happened.
	remote="${remote}; rc=\$?"
	remote="${remote}; kill -INT \$ask 2>/dev/null"
	remote="${remote}; wait \$ask; ask_rc=\$?"
	remote="${remote}; exit \$(( rc != 0 ? rc : ask_rc ))"

	if [ -n "$motion" ]; then
		echo "${prog}: playing ${motion} on ${host}; the machine moves for" >&2
		echo "${prog}: about a minute and stops itself. Keep the space" >&2
	else
		echo "${prog}: touring ${host}'s library; the machine moves for" >&2
		echo "${prog}: several minutes and stops itself. Keep the space" >&2
	fi
	echo "${prog}: around it clear; ${budget}s is the backstop." >&2
	launch_and_capture "$remote" \
		"will not reach the unit: ${subject} ends the run itself, and" \
		"stopping it sooner is 'ssh root@${host} pkill -x simplelaunch'."

	# The chain's own refusals, as --run's: a console with no sentinel in it
	# is a chain that refused or an ssh that never connected, and nothing was
	# recorded to fetch.
	if ! launcher_reached "${aside}/run-console.log"; then
		bus_refusal "$rc" "$article" \
			"255 is ssh's own code and also the run's if the launcher exited with it" \
			"Its own error is above. Check ${launch_logs} on ${host} before re-running," \
			"and ${prog} ${host} --fetch <records-dir> first if this run's records matter:" \
			"the next run empties the log root."
		chain_refusal "$rc" "$launch_config" --fetch \
			"The payload there predates the harness twin — push again:"
	fi

	# The fetch comes before the judgement here, which is the other way
	# round from --run. A run that ended badly ended after playing some of
	# its plan, and what it did play is the reading the run exists to take:
	# the records are the point of it whatever stopped it, and they live on a
	# tmpfs until they are brought home. The refusals below quote the
	# sender's own last line, which is in the console this fetch carries.
	out=$(fetch_records "$dest" "$log_root" "$prefix" motion)

	console=$(file_captures "$aside" "$out")
	echo "${prog}: console ${console}"

	case "$rc" in
	0)
		# The sender ended the run and said nothing red: the expected
		# end of a whole plan.
		;;
	124)
		die "${subject} did not end within its ${budget}s backstop, so the launcher was stopped for it." \
			"The sender's last line: $(ask_last_line "$console")" \
			"Its records were fetched to ${out} and say how far it got."
		;;
	137)
		die "the launcher did not stop on SIGINT and was killed (exit ${rc})." \
			"That is a launcher wedged in its own shutdown; its output is under ${launch_logs} on ${host}."
		;;
	*)
		die "${subject} on ${host} failed (exit ${rc})." \
			"The sender's last line: $(ask_last_line "$console")" \
			"Its records were fetched to ${out}; the launcher's output is under ${launch_logs} on ${host}."
		;;
	esac

	run_dir=$(fetched_run_dir "$out")

	# Judged against the motions it was supposed to play: the analyzer needs
	# the name table as well as the log, because every one of its findings is
	# about a motion that should have been asked for. The table is the
	# sender's own selection, written into the run directory here.
	asked=$(asked_table "$run_dir" "$motion")
	tour_verdict "$run_dir" "$asked"
}

# The chain fragment that puts this run's configuration beside its records.
#
#   config_into_log_root <log root> [extra payload-relative paths...]
#
# The three files a run can be varied by, copied out of the payload into the log
# root the fetch brings home, so a fetched records directory carries the files
# that produced it. Both motion analyzers read them and refuse a log with none:
# what a machine was commissioned with is a fact about the run, and an analyzing
# host's own copy of the tree is an answer to a different question. A speech run
# carries them for the same reason -- it is the same payload under the same
# overlay, and what it records is judged later by whoever asks.
#
# Straight out of the release rather than staged aside the way the stamp is: the
# release is not what the wipe empties, so these copies are made after it with
# nothing at risk. The payload-relative path is kept whole, so
# `config/cogs/servo_profile.textproto` says where on the unit the file was read
# from.
#
# The extra paths are the caller's, for the files only one kind of run can be
# read without: a recording session carries its own speech configuration home,
# because the endpointer values an utterance's interval is derived from are in
# it and a document derived from another file's numbers would be a timeline
# nobody can trust.
#
# The copy lands in the log root and not in the run directory because the run
# directory does not exist when the chain runs; the fetch moves it in
# (`config_into_run_dir`), so a fetched run directory is self-contained and the
# analyzers' one lookup stands.
config_into_log_root() {
	local log_root=$1
	shift
	local name dir fragment="" dirs=() members=("${run_config_files[@]}" "$@")
	# One `mkdir` per directory the set lands in, not one per file: the
	# payload-relative paths are known here, so the remote shell is not asked
	# to run `dirname` three times to rediscover the one directory they share.
	for name in "${members[@]}"; do
		dir=${name%/*}
		case " ${dirs[*]-} " in
		*" ${dir} "*) ;;
		*) dirs+=("$dir") ;;
		esac
	done
	for dir in "${dirs[@]}"; do
		fragment="${fragment}; mkdir -p -- ${log_root}/config/${dir} || exit ${rc_post_wipe}"
	done
	for name in "${members[@]}"; do
		fragment="${fragment}; cp -- ${release}/${name} ${log_root}/config/${name} || exit ${rc_post_wipe}"
	done
	printf '%s' "$fragment"
}

# The run's configuration copy, moved from the fetched root into the run
# directory.
#
#   config_into_run_dir <fetched root> <run directory>
#
# The analyzers look for `config/` inside the run directory they are given, and
# the chain could only leave the copy one level up (the run directory is named
# by the logger, after the chain has run). So the fetch closes the gap: a run
# directory that comes home carries the files that produced it, and a copy of
# it moved anywhere else still reads.
#
# A fetched root with no `config/` is left alone rather than refused here: the
# analyzer is what refuses a log that states nothing about the machine it was
# recorded on, and it names the missing directory when it does. Nothing in this
# script can answer for a payload that never wrote the copy.
#
# Best-effort, like the console and audio copies above: a relocation that could
# not happen is a line on stderr and a `config/` left where it landed, never a
# hardware run whose records came home and whose verdict was thrown away. A
# `config/` already in the run directory is the run's own and is kept -- `mv`
# would nest the fetched copy inside it.
#
# What makes a root-level copy this run's own is that every fetch lands in a
# destination of its own, under `motion-log-<stamp>`, `tour-log-<stamp>` or
# `probe-log-<stamp>`: a destination reused across runs could hold a `config/`
# an earlier fetch left, and this would move that copy into a newer run to be
# judged as its configuration.
config_into_run_dir() {
	local root=$1 run_dir=$2
	[ -d "${root}/config" ] || return 0
	if [ -e "${run_dir}/config" ]; then
		echo "${prog}: ${run_dir} already carries a configuration copy; the fetched one stayed at ${root}" >&2
		return 0
	fi
	mv -- "${root}/config" "${run_dir}/config" ||
		echo "${prog}: the run's configuration copy stayed at ${root}" >&2
}

# The fetched run directory, with its configuration copy moved in.
#
#   fetched_run_dir <fetched root>
#
# Echoes the run directory. One helper because both fetch paths ask exactly
# this and the two refusal hints are the same text: a second copy of them
# drifts, and a hint that tells an operator to look at the wrong thing is worse
# than no hint.
fetched_run_dir() {
	local root=$1 run_dir
	run_dir=$(run_directory "$root" \
		"Either it never started or it could not open a file there; its output is under ${launch_logs} on ${host}." \
		"The logger came up and wrote nothing, which is what a pinion namespace or shm-root disagreement looks like: compare the payload's cogs/robot_logger.textproto against the flagless defaults every process runs on.")
	echo "${prog}: log  ${run_dir}" >&2
	config_into_run_dir "$root" "$run_dir"
	printf '%s\n' "$run_dir"
}

# The remote chain both run modes start with, up to the sentinel, as one string.
#
#   launch_chain <log root>
#
# Everything before the launcher: the bus question, the checks that can still
# refuse, the wipe of the log root and the launcher's console directory, the
# stamp and this run's configuration into the log root, the `cd` into the
# release and the sentinel. One copy
# because both modes make exactly these preparations and the reasoning behind
# their order — what is asked before the wipe and what is answered after it — is
# what a second copy would drift on. What differs is what is started at the end
# of it, which is the caller's to append.
launch_chain() {
	local log_root=$1 remote
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
	# re-running is how to keep them.
	#
	# This chain's own exit codes are `rc_no_stamp` and its
	# neighbours, in the one table near the top of this script, and
	# `chain_refusal` carries the four messages for both run modes.
	remote="$(bus_probe)"
	# The launcher config this run names, on the unit, asked about
	# before anything is emptied. The push checks the local payload
	# carries both configs, but a push and a run are separate
	# invocations and a run starts whatever is already there: a
	# unit still holding a payload staged before the harness twin
	# existed passes every local check and dies inside `simplelaunch`
	# with the log root already cleared and the run already stamped.
	remote="${remote}; [ -f ${release}/${launch_config} ] || exit ${rc_no_launch_config}"
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
	remote="${remote}$(config_into_log_root "$log_root")"
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
	# Past the last step that can refuse: every code from here on is
	# the launcher's own, and this line in the console is what says
	# so.
	remote="${remote}; echo ${launch_sentinel}"
	echo "$remote"
}

# Refuse a staged speech configuration the host itself would refuse.
#
#   speech_preflight <payload-relative speech configuration>
#
# The configuration is the caller's, because the payload carries one per voice
# host entry and each launcher config names its own literally: a session is
# preflighted against the file the host it starts will load.
#
# The real loaders, over the payload that is about to run, before any device is
# touched: `speech-surface`'s configuration structs refuse an unknown field
# outright, and every path field they carry names a file that has to be inside
# the payload. On the unit that is a host that exits at start with its message
# in a log an operator has to fetch to read; here it is a message on the screen
# of the person who typed the command.
#
# The binary is this workspace's own, built in the default configuration —
# nothing here runs on the device — and it is run with the payload root as its
# working directory, so the relative paths it resolves are the ones the launcher
# will make the host resolve on the unit.
speech_preflight() {
	local config_path=$1 listing binary
	echo "${prog}: checking the staged ${config_path}" >&2
	"$bazel" build "${build_flags[@]}" -- "$check_target" >&2 ||
		refuse "$rc_check_refused" \
			"the configuration checker did not build, so the staged speech configuration was not checked." \
			"That is a build failure in this workspace and bazel's own output is above."
	listing=$(bazel_files "$check_target")
	binary=$(bazel_named_in "$listing" reachy_host)
	(cd "$payload" && "$binary" --speech-config "$config_path" --check) ||
		refuse "$rc_check_refused" \
			"the staged speech configuration did not pass ${check_target} --check, so nothing was started." \
			"Each line above is one conclusion; the last says which subjects did not hold." \
			"The configuration and the files it names are the operator's own, and the build" \
			"stages them from beside it — fix them there and build again:" \
			"    make motion-build"
}

# A speech run has no budget: it ends when the operator ends it. `--run` warns
# about a missing pty because its budget still stops the run; here the ^C is the
# only stop there is, so a stdin that cannot carry one is a refusal rather than a
# run nobody can end.
#
# One spelling, asked twice: `--speech-preflight` asks it before `make
# speech-run` provisions and builds, and `--speech` asks it again for a direct
# invocation that skipped the target.
require_speech_tty() {
	[ -t 0 ] ||
		refuse "$rc_no_tty" \
			"stdin is not a terminal, so ssh allocates no pty and a ^C here would not reach the unit." \
			"A speech run has no budget — the operator ends it — so one that cannot be" \
			"interrupted is not started. Run it from a terminal." \
			"A run already going is stopped with 'ssh root@${host} pkill -x simplelaunch'."
}

# One value as an absolute path, whatever the caller typed.
#
#   absolute_path <path>
#
# A records directory, and the name table beside it, reach an analyzer through
# `bazel run`, which runs it out of its own runfiles tree: a relative path would
# be resolved there and name nothing, and what that looks like is a run that
# produced records and an analyzer that says there is no log in them. Every mode
# that hands a path onward goes through here, so the reason is stated once.
absolute_path() {
	case $1 in
	/*) printf '%s\n' "$1" ;;
	*) printf '%s\n' "${PWD}/$1" ;;
	esac
}

# Refuse a pair of staged speech configurations that would put the pod and the
# host on different addresses or different keys.
#
#   require_record_config_agreement
#
# The recording session has its own speech configuration — bypassed wake gate,
# the parrot brain, no bridge — and the pod is provisioned from the site's,
# because provisioning is what writes the PSK table both builds stage. So the
# two files are independent operator files that have to agree on exactly the
# scalars the link is made of: the address the pod dials, and the key table it
# authenticates against.
#
# Refused here rather than discovered on the unit: what a mismatch looks like to
# a person with both hands on the head is a robot that never answers, and the
# half of the session they would spend looking at the recorder is the half that
# was working.
#
# A payload with no site configuration is asked nothing — there is nothing to
# disagree with, and the pod on such a unit was provisioned from a file this
# deploy cannot see.
require_record_config_agreement() {
	record_config_agreement \
		"${payload}/${record_speech_config_path}" "staged ${record_speech_config_path}" \
		"${payload}/${speech_config_path}" "staged ${speech_config_path}"
}

# The agreement itself, over whichever pair of files the caller has.
#
#   record_config_agreement <recording file> <its name> <site file> <its name>
#
# Two callers and one comparison: the preflight asks it of the operator's own
# files before anything is provisioned or built, and `--record` asks it of the
# staged copies, which are what a session actually runs. The names are what the
# refusal calls them, so an operator reading it knows which pair to go and fix.
#
# A file that is not there is asked nothing: the site's absence is a unit
# provisioned from a file this deploy cannot see, and the recording one's is the
# refusal its own caller makes.
record_config_agreement() {
	local mine_file=$1 mine_name=$2 site=$3 site_name=$4 key mine theirs
	[ -f "$site" ] || return 0
	[ -f "$mine_file" ] || return 0
	for key in listen_addr pod_psk_file; do
		mine=$(toml_table_value "$mine_file" "" "$key") || exit 1
		theirs=$(toml_table_value "$site" "" "$key") || exit 1
		[ "$mine" = "$theirs" ] ||
			refuse "$rc_record_config_disagreement" \
				"the ${mine_name} states ${key} = '${mine}' and the ${site_name} states '${theirs}'." \
				"The pod is provisioned from the site's configuration and the recording session's" \
				"voice host loads its own, so the two have to name one address and one key table:" \
				"a pod dialling the address it was provisioned with would find nothing listening," \
				"and a session with no pod is a session with no microphone and no speaker." \
				"Both files are the operator's own — fix the recording one to match the site's and" \
				"build again: make motion-build"
	done
}

# A supervised run: the voice host on the unit, no budget, and a person in front
# of the machine who ends it.
#
#   supervised_run <speech|record> <records directory>
#
# One chain and one set of refusals for the two modes that have them, because
# every step but the ones named below is the same question asked of the same
# unit: the bus, the launcher config, the provenance stamp, the pod's link
# credentials, each speech service asked from the robot, the wipe, the
# configuration copy, the sentinel, the launcher under a pty with its apps'
# consoles tailed onto it, and a fetch and a verdict however the run ended.
#
# What the kind decides: which staged speech configuration is preflighted and
# read for endpoints, which launcher config is started, which consoles are
# tailed, what a fetch is named and which analyzer judges it, and the two lines
# that tell the operator what they are about to be doing. The caller has already
# refused a payload that cannot run this kind at all.
supervised_run() {
	local kind=$1 dest=$2
	local config_path launcher prefix fetch_flag noun entry_target push_line
	local opening closing tailed pids_named=no
	local tails=() extra_config=()
	case $kind in
	speech)
		config_path=$speech_config_path
		launcher=$speech_launch_config
		prefix=speech-log
		fetch_flag=--speech-fetch
		noun="speech run"
		entry_target="make speech-run"
		tails=("$voice_host_log")
		push_line="The production config is pushed with the payload, so push again:"
		opening="running the pipeline"
		closing="talk to it, and stop the run with ^C when you are done"
		;;
	record)
		config_path=$record_speech_config_path
		launcher=$record_launch_config
		prefix=record-log
		fetch_flag=--record-fetch
		noun="recording session"
		entry_target="make pose-record"
		tails=("$voice_host_log" "$recorder_log")
		# The endpointer values every utterance's interval is derived
		# from are in this file, so it travels home with the session.
		extra_config=("$record_speech_config_path")
		push_line="The recording config is pushed with the payload, so push again:"
		opening="recording poses"
		closing="the servos are yours to move; say what each pose is and wait for the read-back. ^C ends it"
		;;
	*) die "supervised_run was asked for ${kind}, which is no supervised mode." ;;
	esac


	require_speech_tty

	require_bazel "the ${noun}'s preflight and report"
	speech_preflight "$config_path"

	# The endpoints are read out of the *staged* copy, not the
	# assembly source: that is the file the host will load, and
	# under --stale-ok the two can differ. Read here and pasted into
	# the remote chain, because the reader is this script's and the
	# question is the unit's.
	speech_services=$(speech_service_urls "${payload}/${config_path}") || exit 1

	log_root=$(config_string log_root_dir)
	require_wipeable_log_root "$log_root"

	# The same chain shape as `--run`'s, and the same codes for the
	# steps they share, with the pipeline's own preflights ahead of
	# the wipe and no budget around the launcher.
	remote="$(bus_probe)"
	remote="${remote}; [ -f ${release}/${launcher} ] || exit ${rc_no_launch_config}"
	remote="${remote}; [ -f ${release}/${provenance_name} ] || exit ${rc_no_stamp}"
	# The pod's link credentials, before anything is emptied. A pod
	# that cannot read them parks silently, and what that looks like
	# to a person talking to the robot is a machine that is simply
	# deaf: the whole session would be spent looking at the wrong
	# half. Non-empty rather than merely present, because the file
	# is written by another repo's provisioning and a zero-length
	# one is that write interrupted.
	remote="${remote}; [ -s ${audio_conf} ] || exit ${rc_no_audio_conf}"
	# Each speech service the configuration names, asked from the
	# unit — the vantage that decides whether this pipeline can hear
	# and speak. A workstation-era endpoint that answers on the
	# workstation and names nothing from the robot is the migration
	# error this catches, and it catches it before the launcher
	# starts rather than at the first thing a person says.
	#
	# No `-f`: with it curl exits 22 on any status from 400 up, and a
	# speaches build that serves no listing route would answer 404
	# and be refused as unreachable — a healthy service reading as a
	# dead one, with the refusal sending an operator to re-address
	# the endpoint that was right. What is asked here is whether the
	# robot gets an answer at all.
	while IFS=$'\t' read -r service_table service_url; do
		[ -n "$service_table" ] || continue
		remote="${remote}; curl -sS --max-time 5 -o /dev/null"
		remote="${remote} ${service_url}${service_probe_path}"
		remote="${remote} || exit ${rc_service_unreachable}"
	done <<<"$speech_services"
	remote="${remote}; cp -- ${release}/${provenance_name} ${staged_provenance} || exit ${rc_stamp_unstaged}"
	remote="${remote}; rm -rf -- ${log_root} && mkdir -p -- ${log_root} || exit ${rc_post_wipe}"
	remote="${remote}; mv -- ${staged_provenance} ${log_root}/${provenance_name} || exit ${rc_post_wipe}"
	remote="${remote}$(config_into_log_root "$log_root" ${extra_config[@]+"${extra_config[@]}"})"
	remote="${remote}; rm -rf -- ${launch_logs} && mkdir -p -- ${launch_logs} || exit ${rc_post_wipe}"
	remote="${remote}; cd ${release} || exit ${rc_post_wipe}"
	# Past the last step that can refuse: what a supervised session
	# records is not repeatable, so the difference between a chain
	# that refused and a launcher that happened to exit 5 has to be
	# something other than the number.
	remote="${remote}; echo ${launch_sentinel}"
	# No `timeout` and no `reachy_ask`: both supervised configs start
	# the voice host, which owns the narration port, and the run
	# ends when the person watching it ends it. A ^C through the
	# pty reaches this foreground group and a dropped ssh does the
	# same by SIGHUP — the right shape for a supervised mode.
	#
	# The trailing `exit $?` keeps a shell in front of the launcher,
	# for the reason `--run`'s does: without it a launcher killed by
	# a signal comes back as ssh's own 255.
	#
	# The tails are what put the apps' own consoles on the operator's
	# terminal. Without them the pipeline's narration — including the
	# loud line a bridge or a stage dies on, and the recorder's own
	# refusal — goes only into a file on the unit that nobody reads
	# until the run is over, and the person at the machine spends the
	# session addressing something that stopped listening in its first
	# second. `-F` because the files do not exist yet when this starts,
	# and their `2>/dev/null` suppresses only tail's own
	# waiting-for-the-file chatter.
	#
	# The pids accumulate in one variable rather than one apiece: the
	# set is this script's and the kill has to cover all of it, and a
	# name per app would be a spelling that depends on how many there
	# are.
	for tailed in "${tails[@]}"; do
		remote="${remote}; tail -F ${launch_logs}/${tailed} 2>/dev/null &"
		if [ "$pids_named" = yes ]; then
			remote="${remote} tail_pids=\"\$tail_pids \$!\""
		else
			remote="${remote} tail_pids=\$!"
			pids_named=yes
		fi
	done
	remote="${remote}; ./simplelaunch ${launcher} --logdir ${launch_logs}"
	# The kill covers the orderly exit: a ^C through the pty reaches
	# the tails with the rest of the foreground group, but a launcher
	# that returned on its own would otherwise leave one holding the
	# ssh session open.
	remote="${remote}; rc=\$?; kill \$tail_pids 2>/dev/null; exit \$rc"

	echo "${prog}: ${opening} on ${host}; eyes on the machine" >&2
	echo "${prog}: ${closing}" >&2
	# No ^C note: this run is ended by the operator's own ^C and
	# `require_speech_tty` has already refused it without a terminal.
	launch_and_capture "$remote"

	# Only a run that never reached its launcher is refused here.
	# Everything else — the launcher's own exit, a signal, a dropped
	# connection — is a run that happened, and why it ended the way
	# it did is the report's question over the console the fetch
	# brings back. That is the whole difference from `--run`, which
	# has one expected code and treats the rest as failures.
	#
	# The sentinel is what decides it, and here it earns its keep:
	# what a supervised session records is not repeatable, so a
	# launcher exiting 5 read as a payload with no provenance stamp
	# would cost the records themselves, sitting on tmpfs until the
	# next run wipes them, under a message saying nothing was
	# started.
	if ! launcher_reached "${aside}/run-console.log"; then
		bus_refusal "$rc" "a ${noun}" "nothing was started"
		chain_refusal "$rc" "$launcher" "$fetch_flag" "$push_line"
		case "$rc" in
		"$rc_no_audio_conf")
			# The line below is pasted by someone already blocked, so
			# it may only name a concrete file this run can show is the
			# one the payload carries. REACHY_SPEECH_CONFIG is read
			# here, at deploy time; the staged copy is what the host
			# loads, and --stale-ok, a direct invocation with the
			# variable unset, or a build in another shell each part the
			# two. Provisioning from the other half derives the pod's
			# address and its key from a configuration the host never
			# reads — a next run that composes and sits deaf, which is
			# the failure this refusal exists to head off. Unconfirmed,
			# the placeholder goes back in and says so: a path the
			# operator has to supply is one they have to think about.
			#
			# The host is always named: this refusal came from it.
			# Omitted, the remediation command may target a different
			# unit, and this one refuses again identically.
			provision_note=()
			if cmp -s -- "$speech_config" "${payload}/${speech_config_path}"; then
				provision_config=$speech_config
			else
				provision_config="<assembly>/speech.toml"
				provision_note=(
					"The path there is yours to fill in: this deploy could not confirm which file the"
					"staged ${speech_config_path} was built from, so name the assembly configuration"
					"that matches it — not one that merely looks like it."
				)
			fi
			die "${host} has no ${audio_conf}, so the audio device has no link credentials and would park silently." \
				"That file is written for you by the target that runs this script:" \
				"    ${entry_target}" \
				"which provisions it before every run, from the same speech configuration it" \
				"builds the payload with. Reaching this refusal means either this script was" \
				"invoked directly, or the unit lost its tmpfs since the provisioning ran." \
				"The raw command, for the first case — it is brenn-pod's, whose writer this repo" \
				"invokes and never duplicates:" \
				"    make -C firmware reachy-provision ON_UNIT=1 REACHY_HOST=${host} SPEECH_CONFIG=\"${provision_config}\"" \
				${provision_note+"${provision_note[@]}"} \
				"ON_UNIT=1 is what says the voice host runs on the unit, which is what makes the" \
				"configuration's loopback address the right one to hand the audio device." \
				"Nothing was started and nothing was emptied."
			;;
		"$rc_service_unreachable")
			die "${host} cannot reach a speech service the configuration names, so nothing was started." \
				"The failing URL is in the output above, asked for ${service_probe_path} from the unit." \
				"Any answer at all counts as reached, so this is a name that does not resolve," \
				"a connection refused or a five-second deadline. An endpoint that answers on" \
				"this workstation and not from the robot is the usual cause — a speech service" \
				"is named by an address the robot can dial, never by localhost. Nothing was emptied."
			;;
		esac
	fi

	echo "${prog}: the run ended (exit ${rc}); fetching what it recorded" >&2
	out=$(fetch_records "$dest" "$log_root" "$prefix" "$kind")
	console=$(file_captures "$aside" "$out")
	echo "${prog}: console ${console}"
	echo "${prog}: log  ${out}"
	echo "${prog}: audio ${out}.audio"

	# Both sides of the fetch are what a supervised run is read off:
	# what a person said to the robot decides what is in either, so
	# the analyzer's standard over the console is presence and
	# absence rather than arithmetic. For a speech run the records are
	# where it asks whether the head moved for the scripts the session
	# took; for a recording session there are no records at all and the
	# console is the pose stream itself. Either verdict is this
	# script's.
	case $kind in
	speech) speech_verdict "$out" ;;
	record) pose_verdict "$out" ;;
	esac
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
		require_members executable "${binaries[@]}"
		require_members "shared object" "${shared_objects[@]}"
		require_members "launcher config" "${launch_configs[@]}"
		require_members model "${models[@]}"
		require_members "run configuration" "${run_config_files[@]}"

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
			# The three members no commit to this workspace can date,
			# asked about separately and by the same override.
			refuse_if_source_newer "${payload}/reachy_pod" "$pod_binary" \
				"audio device binary" \
				"${prog} ${host} --push --stale-ok"
			refuse_if_source_newer "${payload}/${speech_config_path}" \
				"$speech_config" "speech configuration" \
				"${prog} ${host} --push --stale-ok"
			# The unit's own parameters, which name the pod this head
			# answers to: a staged copy older than the operator's file
			# is a head answering to the previous name, and every
			# script addressed to it dropped as a foreign pod's.
			refuse_if_source_newer "${payload}/${host_params_path}" \
				"$host_params" "host configuration" \
				"${prog} ${host} --push --stale-ok"
			# Credential rotation — a re-provisioned key table
			# or a fresh token — is a file no commit to this
			# workspace dates.
			speech_credentials=$(speech_credential_paths "$speech_config") || exit 1
			while IFS=$'\t' read -r credential_key credential_path credential_src; do
				[ -n "$credential_key" ] || continue
				refuse_if_source_newer \
					"${payload}/${credential_path}" \
					"$credential_src" \
					"speech credential ${credential_path}" \
					"${prog} ${host} --push --stale-ok"
			done <<<"$speech_credentials"
		fi

		log_root=$(config_string log_root_dir)

		# Before the stamp, so the digests it records are the digests of
		# the files that land, and before the rsync, so the unit gets
		# the overlaid copies rather than the build's.
		overlay_experiment

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
		dest=$(absolute_path "$dest")
		require_bazel "the run's report"
		log_root=$(config_string log_root_dir)
		require_wipeable_log_root "$log_root"

		remote=$(launch_chain "$log_root")
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
		# The budget is how the run stops: SIGINT is the stop
		# gesture, and --kill-after is for a launcher that does not
		# answer it. Ten seconds is well beyond any controlled
		# shutdown; a grace that expires is a wedged launcher.
		remote="${remote}; timeout --signal=INT --kill-after=10"
		remote="${remote} ${run_seconds} ./simplelaunch ${launch_config}"
		remote="${remote} --logdir ${launch_logs}"
		remote="${remote}; rc=\$?"
		remote="${remote}; kill -INT \$ask 2>/dev/null"
		remote="${remote}; wait \$ask 2>/dev/null"
		remote="${remote}; exit \$rc"

		echo "${prog}: running on ${host} for ${run_seconds}s; eyes on the machine" >&2
		launch_and_capture "$remote" \
			"will not reach the unit: the run stops at the ${run_seconds}s budget," \
			"and stopping it sooner is 'ssh root@${host} pkill -x simplelaunch'."

		# The chain's own refusals, asked only of a run that never
		# reached its launcher: the sentinel is what tells a launcher
		# exiting 5 apart from a payload with no provenance stamp. A
		# console without it is a chain that refused or an ssh that never
		# connected, and 3, 4 and 255 read as the probe's and ssh's own
		# there. The 255 wording does not assert that nothing happened on
		# the unit: it is also read where ssh dropped mid-run.
		if ! launcher_reached "${aside}/run-console.log"; then
			bus_refusal "$rc" "a motion run" \
				"255 is ssh's own code and also the run's if the launcher exited with it" \
				"Its own error is above. Check ${launch_logs} on ${host} before re-running," \
				"and ${prog} ${host} --fetch <records-dir> first if this run's records matter:" \
				"the next run empties the log root."
			chain_refusal "$rc" "$launch_config" --fetch \
				"The payload there predates the harness twin — push again:"
		fi

		case "$rc" in
		124)
			# The budget fired, SIGINT was delivered, the launcher wound
			# down: the expected end of a full run.
			;;
		0)
			die "the launcher exited before the ${run_seconds}s budget was up, so the gesture did not finish." \
				"Its console and every process's output are under ${launch_logs} on ${host}."
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

		out=$(fetch_records "$dest" "$log_root" motion-log motion)

		console=$(file_captures "$aside" "$out")
		echo "${prog}: console ${console}"

		# The records are judged here, not on the unit: the analyzer is a
		# host tool and the fetched copy is the one that outlives the
		# tmpfs. No jitter band — a hardware log sits on an absolute
		# grid, so it is read strictly.
		run_dir=$(fetched_run_dir "$out")

		# The report's verdict is this script's, and it is read off the
		# log alone: the driver republishes its whole account of the run
		# into it, so the analyzer needs no console and a console that
		# did not come back costs the verdict nothing.
		report_verdict "$run_dir"
		;;

	--tour)
		dest=${1:-}
		[ -n "$dest" ] || usage
		[ $# -eq 1 ] || usage
		play_from_library "$dest"
		;;

	--probe)
		dest=${1:-}
		motion=${2:-}
		[ -n "$dest" ] || usage
		[ -n "$motion" ] || usage
		[ $# -eq 2 ] || usage
		# The name is pasted into the remote command run as root, so it
		# goes through the same screen the staged configuration's values
		# do, before the build and the push.
		plain_name "the motion name" "$motion"
		play_from_library "$dest" "$motion"
		;;

	--fetch)
		dest=${1:-}
		[ -n "$dest" ] || usage
		log_root=$(config_string log_root_dir)
		out=$(fetch_records "$dest" "$log_root" motion-log motion)
		echo "${prog}: read it: bazel run //cogs:first_motion_report -- ${out}/<run>"
		;;

	--speech)
		dest=${1:-}
		[ -n "$dest" ] || usage
		[ $# -eq 1 ] || usage

		# The speech configuration is an optional payload member: a
		# payload built without one is a valid motion payload and no
		# pipeline at all.
		[ -f "${payload}/${speech_config_path}" ] ||
			refuse "$rc_no_speech_config" \
				"the staged payload carries no ${speech_config_path}, so there is no pipeline to run." \
				"The speech configuration is the operator's own file, named by REACHY_SPEECH_CONFIG" \
				"or taken from this tree's gitignored host/speech.toml, and the build stages it" \
				"with the credential files it names:" \
				"    REACHY_SPEECH_CONFIG=<assembly>/speech.toml make motion-build"

		supervised_run speech "$(absolute_path "$dest")"
		;;

	--record)
		dest=${1:-}
		[ -n "$dest" ] || usage
		[ $# -eq 1 ] || usage

		# Two optional payload members, and a recording session needs
		# both: the voice host's own arrangement of the pipeline, and
		# the serial node the recorder opens. A payload carrying
		# neither is a good motion payload and no recorder at all,
		# which is why the build stages them and this refuses them.
		[ -f "${payload}/${record_speech_config_path}" ] ||
			refuse "$rc_no_record_config" \
				"the staged payload carries no ${record_speech_config_path}, so there is no pipeline to record with." \
				"The recording session's speech configuration is the operator's own file — the" \
				"bypassed wake gate, the parrot brain, no bridge — named by" \
				"REACHY_RECORD_SPEECH_CONFIG or taken from this tree's gitignored" \
				"host/speech-record.toml, and the build stages it:" \
				"    REACHY_RECORD_SPEECH_CONFIG=<assembly>/speech-record.toml make motion-build"
		[ -f "${payload}/${bench_config_path}" ] ||
			refuse "$rc_no_bench_config" \
				"the staged payload carries no ${bench_config_path}, so the recorder does not know which serial node to open." \
				"That is the bench's own configuration, the same file a bench night pushes, and" \
				"the build stages a copy of it into the motion payload:" \
				"    BENCH_CONFIG=<path> make motion-build" \
				"One knob for both, because a unit whose bench and whose recorder looked at" \
				"different serial nodes would be one nobody could explain."
		require_record_config_agreement

		supervised_run record "$(absolute_path "$dest")"
		;;

	--record-preflight)
		[ $# -eq 0 ] || usage
		# `--speech-preflight`'s question, asked by `make pose-record`
		# before it provisions the unit and builds a payload: a
		# recording session is ended by the operator's ^C too, and a
		# session property knowable first is refused first.
		require_speech_tty

		# And the three a recording session is likeliest to hit on its
		# first run, asked of the operator's own files rather than of
		# the payload: a missing configuration is a file somebody has to
		# write, and being told so after a device cross-build and a push
		# costs the whole build again. The staged copies are still what
		# `--record` refuses on — they are what a session runs — and
		# these are the same three codes, so a chain that answers here
		# and a chain that answers there read the same.
		[ -f "$record_speech_config" ] ||
			refuse "$rc_no_record_config" \
				"there is no ${record_speech_config}, so there is no pipeline to record with." \
				"The recording session's speech configuration is the operator's own file — the" \
				"bypassed wake gate, the parrot brain, no bridge — named by" \
				"REACHY_RECORD_SPEECH_CONFIG or taken from this tree's gitignored" \
				"host/speech-record.toml. Write it, and the build stages it."
		[ -f "$bench_config" ] ||
			refuse "$rc_no_bench_config" \
				"there is no ${bench_config}, so the recorder would not know which serial node to open." \
				"That is the bench's own configuration, the same file a bench night pushes, named" \
				"by BENCH_CONFIG. One knob for both, because a unit whose bench and whose" \
				"recorder looked at different serial nodes would be one nobody could explain."
		record_config_agreement \
			"$record_speech_config" "$record_speech_config" \
			"$speech_config" "$speech_config"
		;;

	--record-fetch)
		dest=${1:-}
		[ -n "$dest" ] || usage
		[ $# -eq 1 ] || usage
		# Absolute, like every other mode that hands a path onward: the
		# command this prints is run through `bazel run`, which resolves
		# a relative path in the analyzer's own runfiles tree, and
		# `POSE_RECORDS` is spelled relatively.
		dest=$(absolute_path "$dest")
		log_root=$(config_string log_root_dir)
		out=$(fetch_records "$dest" "$log_root" record-log record)
		echo "${prog}: read it: bazel run //cogs:pose_session_report --" \
			"${out} --out ${out}.session"
		;;
	--speech-preflight)
		[ $# -eq 0 ] || usage
		# The one refusal a speech run can reach before anything has been
		# provisioned, built or pushed, so `make speech-run` asks it
		# first: a terminal is a property of the session, not of the
		# payload. The rest of `--speech`'s preflights need the staged
		# payload and stay where they are.
		require_speech_tty
		;;

	--speech-fetch)
		dest=${1:-}
		[ -n "$dest" ] || usage
		log_root=$(config_string log_root_dir)
		out=$(fetch_records "$dest" "$log_root" speech-log speech)
		echo "${prog}: read it: bazel run //cogs:speech_run_report -- ${out}"
		# Named after the report: the comparison reads the clips the report
		# writes. The staged config is the one the run was recorded under.
		echo "${prog}: compare what the recogniser hears with and without the wake word:" \
			"bazel run //crates/reachy-host:stt_compare --" \
			"--speech-config ${payload}/${speech_config_path} ${out}.turns"
		;;

	*) usage ;;
esac
