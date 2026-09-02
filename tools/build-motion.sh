#!/usr/bin/env bash
#
# Build the motion system for the device: the whole payload a run needs.
#
#   tools/build-motion.sh
#
# A cross-compile against the hermetic clang sysroot the pinned Clockwork drop
# brings. A unit is five OS processes — the driver, the logger, the control loop,
# the voice host and the audio device — over four binaries, and each of those
# processes reads configuration by a path relative to its working directory. So
# the artifact is not a binary but a directory laid out the way those paths
# expect:
#
#   target/motion-arm64/release/
#     simplelaunch                                      the launcher, started by hand
#     robotcpu.textproto                                the five apps it starts
#     robotcpu_harness.textproto                        the same without host or pod
#     clockwork/launch/clockwork_prelaunch.sh            what it runs before them
#     reachy_motord                                     the driver process
#     reachy_host                                       the voice host process
#     reachy_pod                                        the audio device process
#     reachy_ask                                        the harness's intent source
#     cogs/robot_clk_exe                                the logger and the control loop
#     driver/motord_params.textproto                    the driver's configuration
#     host/host_params.textproto                        the host's configuration
#     cogs/clip_library.names.json                      the overlay name table it reads
#     models/oww/*.onnx                                 the wake gate's three graphs
#     models/silero/silero_vad.onnx                     the endpointer's graph
#     cogs/*.textproto                                  the cogs' configuration
#     cogs/*_event_logger_config.tachyon                which channels are written
#     cogs/*.proc.tachyon, *.logger_proc.tachyon        the two process descriptions
#
# The launcher paths are not this script's choice: the rendered launcher config
# spells every executable and argument as a path relative to the launcher's
# working directory, which is the payload root, so where a file goes is what that
# config already says. `robot_clk_exe` lives under `cogs/` for that reason and no
# other, and the host's two files keep their repository paths because the host's
# own default and its configuration name them that way.
#
# Two launcher configs are staged. The production one names the host and the pod;
# the harness twin names neither, because `deploy-motion.sh --run` starts the
# intent source itself and the host would bind the same narration port and
# address scripts to the same port the control process binds, and because a
# motion run must need no audio hardware. Both are staged because a unit is
# deployed once and used for both.
#
# One payload member is not built here. `reachy_pod` is brenn-pod's binary — it
# links libusb and libasound and is compiled in that repo's arm64 container — so
# it arrives as a prebuilt artifact named by `REACHY_POD_BINARY` below. This
# script stages it, checks its machine, and reports its digest, exactly as it
# does for the binaries it did build.
#
# Everything under it is copied out of Bazel's outputs rather than symlinked:
# the freshness contract deploy-motion.sh enforces is about the age of files
# this script stamps, and Bazel's own outputs are read-only and keep the mtime
# of the action that wrote them.
#
# The generated log-writer configuration comes out of the system target, which
# depends on the executable's own rule, so it cannot be a `data` dependency of it
# and has to be collected here.
#
# Knobs, environment only:
#
#   REACHY_BAZEL           the bazel to run (default bazel)
#   BRENN_POD_DIR          the brenn-pod checkout the prebuilt audio-device
#                          binary is taken from (default: ../brenn-pod; a
#                          relative value is relative to this repository's root)
#   REACHY_POD_BINARY      the prebuilt audio-device binary to stage, overriding
#                          that resolution for the file alone (default: the one
#                          BRENN_POD_DIR's payload build leaves behind)
#   REACHY_SPEECH_CONFIG   the voice pipeline's own configuration to stage
#                          (default: the gitignored host/speech.toml of this
#                          tree; a payload built without one carries no speech
#                          configuration, which is a host that narrates and does
#                          not listen). The credential files it names are staged
#                          with it, from beside it — see the assembly directory
#                          below.
#
# A speech configuration is not one file but a small directory: the TOML, and
# the credential files it names — the pod's key table, the bus token — beside
# it. The TOML names them by the payload-relative paths they will occupy, which
# are also the paths the host resolves at run time, because the launcher starts
# it with the payload root as its working directory. This script stages them
# into the payload for the reason it stages `reachy_pod` there: a payload member
# that arrived by a different route would be the one file whose freshness,
# machine and digest nothing checked.

set -euo pipefail

# shellcheck source=lib.sh
. "$(dirname -- "${BASH_SOURCE[0]}")/lib.sh"

bazel=${REACHY_BAZEL:-bazel}

# The two payload members whose sources are outside this tree -- the audio
# device's binary (`pod_binary`) and the site's speech configuration
# (`speech_config`, staged at `speech_config_path`) -- are named by `lib.sh`:
# this script stages them and `deploy-motion.sh` asks whether either has changed
# since, and a knob spelled twice is a knob that answers two ways.

# Every bazel invocation below runs from the workspace root, whatever directory
# the caller was in: that is where the module lives, and it is what the
# workspace-relative paths cquery answers with are relative to.
cd -- "$repo_root"

# The device configuration lives in .bazelrc as `device`, so this build, the
# bench build and `make check-device` cannot describe different ones. Every
# invocation below passes it: a cquery that resolves an output path must describe
# the configuration the build used, or it names a file from some other one.
build_flags=(--config=device)

# What is built: the filegroup that names every device deployable this payload
# carries. `make check-device` builds that same list (with the bench beside it),
# so the gate cannot cover a different set than this script ships.
build_target=//bazel/platform:motion_payload

# The members, named again here only because their outputs have to be told apart
# afterwards: the driver and the harness's intent source are Rust binaries, the
# executable is synthesized from `robot.clk` (both processes start from it), and
# the system target provides the process descriptions and the generated
# log-writer configuration.
motord_target=//crates/reachy-motord:reachy_motord
host_target=//crates/reachy-host:reachy_host
ask_target=//crates/reachy-ask:reachy_ask
exe_target=//cogs:robot_clk_exe
system_target=//cogs:system_robot_clk
launcher_target=@clockwork//jewels/simplelaunch:simplelaunch
launch_config_target=//cogs:robotcpu.textproto
harness_config_target=//cogs:robotcpu_harness.textproto
prelaunch_target=//cogs:clockwork_prelaunch_sh
# The payload's one shared object. The voice host links ONNX Runtime
# dynamically, so this file has to be staged beside it at the payload root: the
# binary's runpath ends in `$ORIGIN`, and a `NEEDED` the loader cannot resolve
# is a process that never starts.
onnx_target=//bazel/third_party/onnxruntime:shared_object
# The wake and VAD weights, fetched by digest rather than committed. One target
# for all four, because they are staged as a set and none of them is told apart
# from another by anything this script does.
models_target=//bazel/third_party/models:models

# Where each of them goes, by the name it was downloaded under. The paths are
# not this script's choice either: the host's speech configuration names a model
# by a path relative to the working directory the launcher starts it in, which is
# the payload root, so this table and that configuration have to spell the same
# four paths. A model the build fetches and this table has no row for is a
# refused build rather than a file quietly left out of the payload.
model_paths=(
	"melspectrogram.onnx	models/oww/melspectrogram.onnx"
	"embedding_model.onnx	models/oww/embedding_model.onnx"
	"hey_jarvis_v0.1.onnx	models/oww/hey_jarvis_v0.1.onnx"
	"silero_vad.onnx	models/silero/silero_vad.onnx"
)

# The configuration each process reads by a relative path, named by Bazel rather
# than by a list here. `//cogs:robot_config_files` is the same target
# `robot_clk` carries as `data`, and `//driver:motord_params.textproto` the same
# label the driver binary's `data` names -- so a cog that gains a config file is
# added in the BUILD file a developer already has to edit for the host tests to
# find it, and the payload follows. A hand-typed copy here would be a second
# source of truth whose drift is invisible until a process dies at setup on a
# powered unit.
#
# The host's two are named as files rather than through a filegroup because they
# are all it reads and they live in two packages: its own configuration, and the
# clip name table that configuration points at, which is generated beside the
# library it describes and belongs to the cogs.
config_targets=(
	//cogs:clip_library.names.json
	//cogs:robot_config_files
	//driver:motord_params.textproto
	//host:host_params.textproto
)

# The staged payload path. `target/` is a cargo-era naming convention, kept
# deliberately.
payload="${repo_root}/target/motion-arm64/release"

# The generated files a process reads, by basename: two process descriptions and
# the writer's channel set. Everything else the system target emits is for the
# channel spy or the diagnostics database, neither of which this payload starts;
# the launcher's own config is not in this list because it is not staged under
# `cogs/` -- the launcher is started from the payload root and its config is
# named there.
generated_files=(
	brenn_reachy.cogs.system_robot.motion_robot.logger_proc.tachyon
	brenn_reachy.cogs.system_robot.motion_robot.proc.tachyon
	system_robot.motion_robot.RobotCpu_event_logger_config.tachyon
)

# The one file in the payload whose *contents* this script has an opinion about:
# the logger's, checked against the flagless pinion defaults by lib.sh's
# `check_pinion_defaults` before anything is built or staged. The printed start
# commands that used to interpolate these values are gone, so the agreement is
# asserted here and a drifted value is a refused build rather than a wasted bench
# night.
logger_config=cogs/robot_logger.textproto

# The apps the rendered launcher config is expected to name, sorted.
#
# The launcher writes each app's console into `<logdir>/<name>_<run>.log`, and
# those are the files an operator tails and the runbook names one by one. The
# names are the compositions' -- `proc` and `logger_proc` from the `Process`
# names, and `motord`, `voice_host` and `pod` from the apps merged in through
# `simplelaunch_src` -- so a rename in a `.clk` file would leave the documented
# tails naming files that never appear.
# This is the join: the names are pinned here against the config actually built,
# and a rename is a refused build with the two lists side by side. The other
# half -- that the runbook tails a file for every name in this list -- is
# asserted by tools/build-motion.test.sh, so the refusal below cites a document
# a self-check keeps true.
#
# Two lists because two configs are staged, and the difference between them is
# the whole point of the twin: the harness config must not name the host, or a
# motion run would start a second binder of the narration port `reachy_ask`
# holds, and it must not name the pod, or a motion run would want a mic array.
# Asserted per config below, so either app merged into the harness twin by
# accident is a refused build rather than a bind race on a powered unit.
launcher_apps=(logger_proc motord pod proc voice_host)

harness_apps=(logger_proc motord proc)

compile() {
	"$bazel" build "${build_flags[@]}" -- "$build_target"
}

# The app names in the rendered launcher config, against the list above.
#
# Read out of the config that was just built, not out of a composition: what the
# launcher will do is what this file says. Sorted comparison, because merge order
# is the renderer's business and spawn order is hash order regardless.
check_launcher_apps() {
	local config=$1 named
	shift
	local expected=("$@")
	named=$(awk '
		/^[[:space:]]*app[[:space:]]*\{/ { inside = 1 }
		inside && match($0, /name:[[:space:]]*"[^"]*"/) {
			field = substr($0, RSTART, RLENGTH)
			gsub(/^name:[[:space:]]*"|"$/, "", field)
			print field
			inside = 0
		}
		/^[[:space:]]*\}/ { inside = 0 }
	' "$config" | sort | tr '\n' ' ')
	named=${named% }
	[ "$named" = "${expected[*]}" ] || die \
		"${config##*/} names the apps '${named:-nothing}' and the run needs '${expected[*]}'." \
		"Those names are what the launcher calls each process's log file, and" \
		"docs/bench-runbook.md names each app's log file. A process renamed in" \
		"a composition, or an app merged in under another name, has to be renamed there too."
}

# Everything Bazel knows about where the payload's files are, in two questions
# through lib.sh's `bazel_files`.
#
# Two, not one per file and not one per target: each answer is a whole set, and
# `make motion-deploy` is the inner loop of a hardware session, where every
# invocation pays its own loading and analysis phase over the C++ graph. The
# split is by kind of answer -- the built targets' outputs, whose members are
# told apart by basename, and the configuration targets' files, every one of
# which is wanted -- so neither question needs a pattern to guess with.

# Every file the payload will carry, as `<absolute source>\t<path under the
# payload root>`, resolved before a byte of the previous payload is touched.
# Resolution is where the remaining refusals live — a configuration file the tree
# lost, a generated file the build did not emit — and `install` cannot be undone,
# so deciding them first is what makes a refused build leave the previous payload
# alone. A half-staged directory carries a fresh timestamp and passes every
# downstream freshness check while missing files nothing verified.
plan_files=()

# The configuration keeps the workspace-relative path Bazel answered with, which
# is the path the compositions spell (`cogs/…`, `driver/…`): the layout under the
# payload root is the repository's own, because the processes are started with the
# payload root as their working directory. The generated files are flattened into
# `cogs/`, which is where the processes look for them.
plan() {
	local built=$1 configs=$2 file src
	plan_files=()

	while IFS= read -r file; do
		[ -n "$file" ] || continue
		[ -f "${repo_root}/${file}" ] ||
			die "the payload wants ${file} and the tree has no such file."
		plan_files+=("${repo_root}/${file}"$'\t'"${file}")
	done <<<"$configs"

	# `bazel_named_in`'s own refusal runs in this command substitution, so its
	# exit is the subshell's and has to be turned back into this script's.
	for file in "${generated_files[@]}"; do
		src=$(bazel_named_in "$built" "$file") || exit 1
		plan_files+=("${src}"$'\t'"cogs/${file}")
	done
}

# The model files, as `<absolute source>\t<path under the payload root>`.
#
# Resolved the way the shared object is and not the way the configuration is:
# these are fetched repositories' files, so the paths cquery answers with are
# relative to the execution root, which the `bazel-out` convenience symlink does
# not reach. Every file the build names must have a row in `model_paths` and
# every row must be answered for, in both directions -- a model added to the
# fetch and not to the table would be silently absent from a unit, and a row
# nothing answers is a table describing a fetch that no longer happens.
model_files=()

resolve_models() {
	local listing entry name want src wanted=0
	listing=$(bazel_files "$models_target")
	model_files=()

	while IFS= read -r want; do
		[ -n "$want" ] || continue
		name=$(basename -- "$want")
		src="${execroot}/${want}"
		[ -f "$src" ] ||
			die "the build names ${src} and no file is there." \
				"A model archive pin moved, or its repository was not fetched."
		for entry in "${model_paths[@]}"; do
			[ "${entry%%$'\t'*}" = "$name" ] || continue
			model_files+=("${src}"$'\t'"${entry#*$'\t'}")
			break
		done
		[ ${#model_files[@]} -gt "$wanted" ] ||
			die "the build fetches a model called ${name} and the payload has no place for it." \
				"Give it a row in build-motion.sh's model_paths, at the path the host's" \
				"speech configuration names it by."
		wanted=${#model_files[@]}
	done <<<"$listing"

	[ "${#model_files[@]}" -eq "${#model_paths[@]}" ] ||
		die "the payload wants ${#model_paths[@]} model files and the build named ${#model_files[@]}." \
			"MODULE.bazel's fetches and build-motion.sh's model_paths describe one set."
}

# The payload paths this script installs from a name of its own rather than from
# a resolved plan: the binaries at the root, the two launcher configs, the
# launcher's prelaunch script, the speech configuration and the build stamp. The
# push writes one more, `provenance.txt`, into the same directory.
#
# They are listed for one purpose — deciding whether a credential file the
# speech configuration names would land on top of one of them. Nothing else
# reads this, and the members themselves are installed by `stage` by name,
# because an unlabelled argument list is where two cross-built binaries get
# transposed.
payload_fixed_members=(
	reachy_motord
	reachy_host
	reachy_pod
	reachy_ask
	libonnxruntime.so.1
	simplelaunch
	robotcpu.textproto
	robotcpu_harness.textproto
	cogs/robot_clk_exe
	clockwork/launch/clockwork_prelaunch.sh
	"$provenance_name"
	"$speech_config_path"
	"$build_commit_name"
)

# The credential files the staged speech configuration names, as `<absolute
# source>\t<path under the payload root>`, in the shape `plan_files` uses.
speech_credentials=()

# Resolve them, and refuse everything about them that cannot be staged.
#
# Run after `plan` and `resolve_models`, because the collision question is asked
# against the payload members those two resolved: a credential path that lands
# on a model or a cog's configuration would be a file the payload carries under
# a name something else reads, and the loser depends on install order. Still
# before `stage`, so every refusal here leaves the previous payload alone.
#
# The source is beside the configuration, because the configuration names the
# path the file will occupy in the payload rather than the path it occupies now.
# That is the assembly-directory convention: one directory holds the TOML and
# its credentials, and it is also what brenn-pod's provisioning is pointed at,
# so the two sides of the pod's key link keep deriving from one source.
resolve_speech_credentials() {
	local listing key value src entry
	speech_credentials=()
	listing=$(speech_credential_paths "$speech_config") || exit 1
	while IFS=$'\t' read -r key value src; do
		[ -n "$key" ] || continue
		for entry in "${payload_fixed_members[@]}"; do
			[ "$entry" = "$value" ] || continue
			die "${key} in ${speech_config} is ${value}, which is a payload member's own path." \
				"The credential would be installed over ${value}, or under it; name it something" \
				"the payload does not already carry."
		done
		for entry in "${plan_files[@]}" "${model_files[@]}"; do
			[ "${entry#*$'\t'}" = "$value" ] || continue
			die "${key} in ${speech_config} is ${value}, which is a payload member's own path." \
				"The credential would be installed over ${value}, or under it; name it something" \
				"the payload does not already carry."
		done
		[ -f "$src" ] ||
			die "${speech_config} names ${key} = ${value} and there is no file at ${src}." \
				"The credential files a speech configuration names live beside it, under the" \
				"payload-relative paths it spells: that is the directory the payload is staged" \
				"from and the one brenn-pod's provisioning writes the key table into."
		speech_credentials+=("${src}"$'\t'"${value}")
	done <<<"$listing"
}

# The commit the payload's binaries came out of, into the payload itself.
#
#   stamp_build_commit <file>
#
# The push writes the provenance stamp a run's records carry home, and the only
# commit it can see is the pushing tree's HEAD — which is not the same fact: a
# payload staged here and pushed from another checkout would be stamped with a
# commit that never produced it, and the freshness refusal there only catches a
# payload older than its tree. So the build records what it actually built from
# and the push reads it.
#
# A tree with no history for it says `unknown` rather than guessing: a build is
# not the place to refuse over provenance, and the push is where a stamp that
# cannot name a build says so.
stamp_build_commit() {
	local into=$1 commit
	commit=$(git -C "$repo_root" rev-parse HEAD 2>/dev/null) || commit=
	printf 'commit=%s\n' "${commit:-unknown}" >"$into"
}

# Build the payload directory from scratch every time. Removed rather than
# overwritten: a file the layout no longer wants — a configuration a composition
# stopped reading, a process description whose name changed — would otherwise sit
# there being deployed forever.
#
# Nothing here can refuse — every path it installs was resolved by `plan` — but a
# step can still die on the filesystem: a full disk, a permission, a signal. The
# payload must be either the previous complete one or a new complete one, never a
# half-written directory with a fresh timestamp: `deploy-motion.sh --push` checks
# freshness, not completeness. So the whole payload is built in a sibling `.new`
# directory and the swap is the last thing that happens. A failure before the swap
# leaves the previous payload where it was, entire, and a leftover `.new` or
# `.old` is swept by the next build.
#
# The swap moves the old payload aside before the new one takes its place and
# deletes it afterwards, so the destructive step is last: a signal in the middle
# of the swap leaves either the old payload or the new one under the payload
# path, or no directory at all — which `deploy-motion.sh --push` refuses — rather
# than a payload with half its files deleted, which it cannot tell from a whole
# one.
#
# Every input is read from the enclosing scope by name, and nothing is passed
# positionally: the payload grows a member per slice, and an unlabelled argument
# list is a place where two of them can be transposed into a staging that looks
# right and runs the wrong file — two cross-built Rust binaries especially,
# which every existence check downstream would still pass.
stage() {
	local entry staging="${payload}.new" previous="${payload}.old"
	rm -rf -- "$staging" "$previous"
	mkdir -p -- "$staging"

	install -m 0755 -D -- "$motord_out" "${staging}/reachy_motord"
	install -m 0755 -D -- "$host_out" "${staging}/reachy_host"
	# The one binary in the payload this build did not produce: brenn-pod's, at
	# the payload root under this payload's own naming, which is what the pod's
	# app entry spells.
	install -m 0755 -D -- "$pod_binary" "${staging}/reachy_pod"
	# Not a launcher app: it binds the narration port before the composition is
	# started, so `--run` starts it itself, ahead of the launcher.
	install -m 0755 -D -- "$ask_out" "${staging}/reachy_ask"
	# Beside the host, because that is what its `$ORIGIN` runpath means.
	install -m 0755 -D -- "$onnx_out" "${staging}/libonnxruntime.so.1"
	install -m 0755 -D -- "$exe_out" "${staging}/cogs/robot_clk_exe"
	install -m 0755 -D -- "$launcher_out" "${staging}/simplelaunch"
	install -m 0644 -D -- "$launch_config_out" "${staging}/robotcpu.textproto"
	install -m 0644 -D -- "$harness_config_out" "${staging}/robotcpu_harness.textproto"
	install -m 0755 -D -- "$prelaunch_out" \
		"${staging}/clockwork/launch/clockwork_prelaunch.sh"

	for entry in "${plan_files[@]}" "${model_files[@]}"; do
		install -m 0644 -D -- "${entry%%$'\t'*}" "${staging}/${entry#*$'\t'}"
	done

	# The one member that may be absent, and the one staged 0600: it carries a
	# bus token and this unit's link keys, so it is readable by the account that
	# runs the payload and by nobody else on the machine.
	if [ -f "$speech_config" ]; then
		install -m 0600 -D -- "$speech_config" "${staging}/${speech_config_path}"
	fi

	# Credentials (the pod's key table, the bus token): mode 0600 so only
	# the payload's account reads them. `rsync -a` carries the mode to the
	# unit.
	for entry in "${speech_credentials[@]}"; do
		install -m 0600 -D -- "${entry%%$'\t'*}" "${staging}/${entry#*$'\t'}"
	done

	stamp_build_commit "${staging}/${build_commit_name}"

	if [ -e "$payload" ]; then
		mv -- "$payload" "$previous"
	fi
	mv -- "$staging" "$payload"
	rm -rf -- "$previous"
}

# The digests are how a person tells two payloads apart at the bench; for
# `reachy_pod` they are more than that, because the payload's build stamp records
# this tree's commit and says nothing about a binary that came out of another
# repo's container.
report() {
	local size
	size=$(du -sh -- "$payload" | cut -f1)
	echo "${prog}: device payload  ${payload}  (${size})"
	local file
	for file in reachy_motord reachy_host reachy_pod reachy_ask libonnxruntime.so.1 \
		cogs/robot_clk_exe simplelaunch; do
		echo "${prog}: ${file}  $(sha256sum -- "${payload}/${file}" | cut -d' ' -f1)"
	done
	# The one member whose provenance a digest does not settle: reachy_pod was
	# compiled in the other repository, and the host beside it links that
	# repository's crates from a revision this tree pins. Said on every build,
	# because the two agreeing is a habit rather than a mechanism.
	echo "${prog}: brenn-pod  $(pod_provenance)"
	# Said either way, and without a digest: the contents are a site's own, and
	# what a person needs to know at the bench is whether this payload's host
	# will listen or only narrate.
	if [ -f "${payload}/${speech_config_path}" ]; then
		echo "${prog}: ${speech_config_path}  staged from ${speech_config}"
	else
		echo "${prog}: ${speech_config_path}  absent; the voice host will run its edge half alone"
	fi
	# The credential files that configuration names, path only and no digest,
	# for the reason the configuration itself gets none: what a person needs at
	# the bench is which files this payload carries and where they came from.
	local entry
	for entry in "${speech_credentials[@]}"; do
		echo "${prog}: ${entry#*$'\t'}  staged from ${entry%%$'\t'*}"
	done
}

require_bazel "device payload"
check_pinion_defaults "$logger_config"
compile
built=$(bazel_files "$(union "$motord_target" "$host_target" "$ask_target" \
	"$exe_target" "$system_target" "$launcher_target" "$launch_config_target" \
	"$harness_config_target" "$prelaunch_target")")
configs=$(bazel_files "$(union "${config_targets[@]}")")
motord_out=$(bazel_named_in "$built" reachy_motord)
host_out=$(bazel_named_in "$built" reachy_host)
ask_out=$(bazel_named_in "$built" reachy_ask)
exe_out=$(bazel_named_in "$built" robot_clk_exe)
launcher_out=$(bazel_named_in "$built" simplelaunch)
launch_config_out=$(bazel_named_in "$built" robotcpu.textproto)
harness_config_out=$(bazel_named_in "$built" robotcpu_harness.textproto)
prelaunch_out=$(bazel_named_in "$built" clockwork_prelaunch.sh)
# Resolved on its own and not out of the listing above: the shared object is a
# fetched repository's file, so the path cquery answers with is relative to the
# execution root rather than to this workspace, and the `bazel-out` symlink the
# other outputs resolve through does not reach it. The device ISA sweep reads it
# the same way.
execroot=$("$bazel" info "${build_flags[@]}" execution_root \
	--ui_event_filters=-info --noshow_progress) ||
	die "bazel cannot say where its execution root is, so the fetched shared object cannot be found."
onnx_out="${execroot}/$(bazel_files "$onnx_target")"
[ -f "$onnx_out" ] ||
	die "the build names ${onnx_out} and no file is there." \
		"The ONNX Runtime archive pin moved, or its repository was not fetched."
# The prebuilt member, decided here with the built ones and before anything is
# staged: a payload missing the pod binary is a launcher that starts an app that
# is not there, and the app entry is in the production config unconditionally
# because a unit without a working audio device is still a unit whose pod should
# be trying.
[ -f "$pod_binary" ] ||
	die "there is no audio-device binary at ${pod_binary}." \
		"It is brenn-pod's to build: run 'make -C ${brenn_pod_dir}/firmware reachy-pod'" \
		"in that checkout. If brenn-pod is somewhere else, BRENN_POD_DIR names the" \
		"checkout — for this invocation or once in the gitignored .local/reachy.conf —" \
		"and REACHY_POD_BINARY names a bare artifact copied out of a build elsewhere."
# The other member from outside the build, decided in the same place. Only the
# named case can refuse: an unnamed one that is not there is a payload whose
# host narrates and does not listen, which is what a unit runs until somebody
# has a configuration to push.
if [ -n "$speech_config_named" ] && [ ! -f "$speech_config" ]; then
	die "there is no speech configuration at ${speech_config}." \
		"REACHY_SPEECH_CONFIG names the voice pipeline's own TOML, which is a site's" \
		"file and is never in this tree; unset it to build a payload whose host runs" \
		"its edge half alone."
fi
resolve_models
check_launcher_apps "$launch_config_out" "${launcher_apps[@]}"
check_launcher_apps "$harness_config_out" "${harness_apps[@]}"
verify_aarch64 "$motord_out"
verify_aarch64 "$host_out"
# Asked of the prebuilt member the same way, and it is the one where the answer
# is in doubt: everything else here was cross-compiled by the build that just
# ran, while this file was compiled elsewhere, and a workstation build of
# brenn-pod's own pod binary sits at a path that looks just like the arm64 one.
verify_aarch64 "$pod_binary"
verify_aarch64 "$ask_out"
verify_aarch64 "$exe_out"
verify_aarch64 "$launcher_out"
verify_aarch64 "$onnx_out"
plan "$built" "$configs"
resolve_speech_credentials
stage
report
